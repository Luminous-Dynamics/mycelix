// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Local ingest cursor for a platform's store-and-forward maritime evidence stream.
//!
//! Individual [`crate::MaritimeEvidenceEnvelope`] values are content-addressed,
//! but a disconnected receiver also needs to know whether a newly observed record
//! is the direct continuation it expected, an exact duplicate, stale replay, gap,
//! generation regression, or conflicting fork. This module provides that
//! deterministic classification.
//!
//! It is deliberately **not** a global consensus or fork-resolution protocol.
//! A DHT may expose multiple valid branches; higher-level governance/evidence
//! reconciliation decides which branch, if any, supersedes another. This cursor
//! only prevents a local consumer from silently treating non-successors as a
//! linear continuation.

use crate::MaritimeEvidenceEnvelope;
use serde::{Deserialize, Deserializer, Serialize};

const MAX_PLATFORM_ID_BYTES: usize = 256;

/// Compact content-addressed head of one platform event stream.
///
/// `generation` is retained alongside sequence/digest so a restarted receiver
/// can still reject a direct successor that regresses to an older software or
/// evidence lineage generation.
///
/// Fields are private and deserialization validates the same canonical shape as
/// [`Self::from_retained`]. Persisted state therefore cannot bypass restart-time
/// platform/digest validation by constructing this positive cursor directly.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct MaritimeStreamHead {
    platform_id: String,
    generation: u64,
    sequence: u64,
    digest: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MaritimeStreamHeadWire {
    platform_id: String,
    generation: u64,
    sequence: u64,
    digest: String,
}

impl<'de> Deserialize<'de> for MaritimeStreamHead {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let wire = MaritimeStreamHeadWire::deserialize(deserializer)?;
        Self::from_retained(wire.platform_id, wire.generation, wire.sequence, wire.digest)
            .map_err(serde::de::Error::custom)
    }
}

/// Deterministic classification of an incoming record relative to a local head.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MaritimeStreamDisposition {
    /// Candidate is the exact direct successor and may advance the cursor.
    Advance,
    /// Candidate is byte/content-identical to the current head.
    Duplicate,
    /// Candidate sequence is older than the current head.
    StaleReplay,
    /// Candidate skips one or more unseen sequence numbers.
    Gap,
    /// Candidate is the direct next sequence but claims an older lineage generation.
    GenerationRegression,
    /// Candidate conflicts at the current or next sequence/digest binding.
    Fork,
    /// Candidate belongs to a different platform stream.
    WrongPlatform,
}

impl MaritimeStreamHead {
    /// Establish a cursor from an explicit chain root.
    ///
    /// A root must carry no predecessor digest. Receivers joining mid-stream
    /// should initialize from an independently trusted retained head instead of
    /// relabeling an arbitrary record as a root.
    pub fn from_root(root: &MaritimeEvidenceEnvelope) -> Result<Self, String> {
        root.validate()?;
        if root.previous_event_digest.is_some() {
            return Err("maritime stream root must not have a predecessor digest".into());
        }
        Ok(Self {
            platform_id: root.platform_id.clone(),
            generation: root.generation,
            sequence: root.sequence,
            digest: root.content_digest()?,
        })
    }

    /// Rehydrate an independently retained head.
    ///
    /// The caller must persist the lineage generation together with sequence and
    /// digest; dropping it would make generation rollback undetectable after a
    /// restart. Inputs are required to be canonical rather than silently normalized.
    pub fn from_retained(
        platform_id: impl Into<String>,
        generation: u64,
        sequence: u64,
        digest: impl Into<String>,
    ) -> Result<Self, String> {
        let platform_id = platform_id.into();
        let digest = digest.into();
        if !canonical_platform_id(&platform_id) {
            return Err(format!(
                "retained maritime stream platform_id must be non-empty, <= {MAX_PLATFORM_ID_BYTES} bytes, unpadded, and control-free"
            ));
        }
        if !is_canonical_blake3_hex(&digest) {
            return Err(
                "retained maritime stream digest must be canonical lowercase 64-character hex"
                    .into(),
            );
        }
        Ok(Self {
            platform_id,
            generation,
            sequence,
            digest,
        })
    }

    /// Platform stream this retained head belongs to.
    pub fn platform_id(&self) -> &str {
        &self.platform_id
    }

    /// Highest accepted lineage generation at this head.
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Highest accepted sequence at this head.
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Canonical BLAKE3 content digest of the accepted head record.
    pub fn digest(&self) -> &str {
        &self.digest
    }

    /// Classify an incoming envelope without mutating this head.
    pub fn classify(
        &self,
        candidate: &MaritimeEvidenceEnvelope,
    ) -> Result<MaritimeStreamDisposition, String> {
        candidate.validate()?;

        if candidate.platform_id != self.platform_id {
            return Ok(MaritimeStreamDisposition::WrongPlatform);
        }

        let candidate_digest = candidate.content_digest()?;

        if candidate.sequence == self.sequence {
            return Ok(if candidate_digest == self.digest {
                MaritimeStreamDisposition::Duplicate
            } else {
                MaritimeStreamDisposition::Fork
            });
        }

        if candidate.sequence < self.sequence {
            return Ok(MaritimeStreamDisposition::StaleReplay);
        }

        let Some(expected_sequence) = self.sequence.checked_add(1) else {
            // At u64::MAX there is no representable direct successor.
            return Ok(MaritimeStreamDisposition::Gap);
        };

        if candidate.sequence > expected_sequence {
            return Ok(MaritimeStreamDisposition::Gap);
        }

        if candidate.generation < self.generation {
            return Ok(MaritimeStreamDisposition::GenerationRegression);
        }

        if candidate.previous_event_digest.as_deref() != Some(self.digest.as_str()) {
            return Ok(MaritimeStreamDisposition::Fork);
        }

        Ok(MaritimeStreamDisposition::Advance)
    }

    /// Classify and, only for a direct successor, advance this local cursor.
    ///
    /// Duplicate/stale/gap/generation-regression/fork/wrong-platform observations
    /// leave the cursor unchanged so recovery/reconciliation code must handle them
    /// explicitly.
    pub fn ingest(
        &mut self,
        candidate: &MaritimeEvidenceEnvelope,
    ) -> Result<MaritimeStreamDisposition, String> {
        let disposition = self.classify(candidate)?;
        if disposition == MaritimeStreamDisposition::Advance {
            self.generation = candidate.generation;
            self.sequence = candidate.sequence;
            self.digest = candidate.content_digest()?;
        }
        Ok(disposition)
    }
}

fn canonical_platform_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_PLATFORM_ID_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_control)
}

fn is_canonical_blake3_hex(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MaritimeEvidenceKind;

    fn event(sequence: u64) -> MaritimeEvidenceEnvelope {
        MaritimeEvidenceEnvelope::new(
            "auv-01",
            7,
            sequence,
            1_000_000 + sequence,
            MaritimeEvidenceKind::HealthObservation,
            r#"{"severity":"healthy"}"#,
            "xenia-transcript:fixture",
        )
    }

    #[test]
    fn direct_successor_advances_and_exact_replay_is_idempotent() {
        let root = event(10);
        let next = event(11).chain_after(&root).unwrap();
        let mut head = MaritimeStreamHead::from_root(&root).unwrap();

        assert_eq!(head.generation(), 7);
        assert_eq!(
            head.ingest(&next).unwrap(),
            MaritimeStreamDisposition::Advance
        );
        assert_eq!(head.sequence(), 11);
        assert_eq!(head.generation(), 7);
        assert_eq!(
            head.ingest(&next).unwrap(),
            MaritimeStreamDisposition::Duplicate
        );
        assert_eq!(head.sequence(), 11);
    }

    #[test]
    fn generation_advance_is_retained_and_regression_is_explicit() {
        let root = event(10);
        let mut advanced = event(11);
        advanced.generation = 8;
        let advanced = advanced.chain_after(&root).unwrap();
        let mut head = MaritimeStreamHead::from_root(&root).unwrap();

        assert_eq!(
            head.ingest(&advanced).unwrap(),
            MaritimeStreamDisposition::Advance
        );
        assert_eq!(head.generation(), 8);

        let mut regressed = event(12);
        regressed.generation = 7;
        regressed.previous_event_digest = Some(head.digest().to_owned());
        assert_eq!(
            head.ingest(&regressed).unwrap(),
            MaritimeStreamDisposition::GenerationRegression
        );
        assert_eq!(head.generation(), 8);
        assert_eq!(head.sequence(), 11);
    }

    #[test]
    fn stale_gap_wrong_platform_and_fork_do_not_advance() {
        let root = event(10);
        let next = event(11).chain_after(&root).unwrap();
        let mut head = MaritimeStreamHead::from_root(&root).unwrap();
        assert_eq!(
            head.ingest(&next).unwrap(),
            MaritimeStreamDisposition::Advance
        );
        let retained = head.clone();

        assert_eq!(
            head.ingest(&root).unwrap(),
            MaritimeStreamDisposition::StaleReplay
        );

        let mut gap = event(14);
        gap.previous_event_digest = Some(head.digest().to_owned());
        assert_eq!(head.ingest(&gap).unwrap(), MaritimeStreamDisposition::Gap);

        let mut wrong = event(12);
        wrong.platform_id = "auv-02".into();
        wrong.previous_event_digest = Some(head.digest().to_owned());
        assert_eq!(
            head.ingest(&wrong).unwrap(),
            MaritimeStreamDisposition::WrongPlatform
        );

        let mut fork = event(12);
        fork.previous_event_digest = Some("00".repeat(32));
        assert_eq!(head.ingest(&fork).unwrap(), MaritimeStreamDisposition::Fork);

        assert_eq!(head, retained);
    }

    #[test]
    fn conflicting_record_at_current_sequence_is_a_fork() {
        let root = event(10);
        let head = MaritimeStreamHead::from_root(&root).unwrap();
        let mut conflicting = root;
        conflicting.payload_json = r#"{"severity":"degraded"}"#.into();

        assert_eq!(
            head.classify(&conflicting).unwrap(),
            MaritimeStreamDisposition::Fork
        );
    }

    #[test]
    fn retained_head_requires_generation_and_canonical_identity_digest() {
        assert!(MaritimeStreamHead::from_retained("", 7, 1, "11".repeat(32)).is_err());
        assert!(
            MaritimeStreamHead::from_retained(" auv-01", 7, 1, "11".repeat(32)).is_err()
        );
        assert!(
            MaritimeStreamHead::from_retained(
                "x".repeat(MAX_PLATFORM_ID_BYTES + 1),
                7,
                1,
                "11".repeat(32)
            )
            .is_err()
        );
        assert!(MaritimeStreamHead::from_retained("auv-01", 7, 1, "xyz").is_err());
        assert!(
            MaritimeStreamHead::from_retained("auv-01", 7, 1, "AA".repeat(32)).is_err()
        );
        let retained =
            MaritimeStreamHead::from_retained("auv-01", 7, 1, "aa".repeat(32)).unwrap();
        assert_eq!(retained.generation(), 7);
        assert_eq!(retained.digest(), "aa".repeat(32));
    }

    #[test]
    fn deserialization_cannot_bypass_retained_head_validation() {
        let valid = r#"{"platform_id":"auv-01","generation":7,"sequence":1,"digest":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}"#;
        let restored: MaritimeStreamHead = serde_json::from_str(valid).unwrap();
        assert_eq!(restored.platform_id(), "auv-01");
        assert_eq!(restored.generation(), 7);
        assert_eq!(restored.sequence(), 1);

        let oversized = format!(
            r#"{{"platform_id":"{}","generation":7,"sequence":1,"digest":"{}"}}"#,
            "x".repeat(MAX_PLATFORM_ID_BYTES + 1),
            "aa".repeat(32)
        );
        assert!(serde_json::from_str::<MaritimeStreamHead>(&oversized).is_err());

        let uppercase_digest = format!(
            r#"{{"platform_id":"auv-01","generation":7,"sequence":1,"digest":"{}"}}"#,
            "AA".repeat(32)
        );
        assert!(serde_json::from_str::<MaritimeStreamHead>(&uppercase_digest).is_err());

        let unknown_field = format!(
            r#"{{"platform_id":"auv-01","generation":7,"sequence":1,"digest":"{}","trusted":true}}"#,
            "aa".repeat(32)
        );
        assert!(serde_json::from_str::<MaritimeStreamHead>(&unknown_field).is_err());
    }
}
