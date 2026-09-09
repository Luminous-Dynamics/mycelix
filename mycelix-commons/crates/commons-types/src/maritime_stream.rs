// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Local ingest cursor for a platform's store-and-forward maritime evidence stream.
//!
//! Individual [`crate::MaritimeEvidenceEnvelope`] values are content-addressed,
//! but a disconnected receiver also needs to know whether a newly observed record
//! is the direct continuation it expected, an exact duplicate, stale replay, gap,
//! or conflicting fork. This module provides that deterministic classification.
//!
//! It is deliberately **not** a global consensus or fork-resolution protocol.
//! A DHT may expose multiple valid branches; higher-level governance/evidence
//! reconciliation decides which branch, if any, supersedes another. This cursor
//! only prevents a local consumer from silently treating non-successors as a
//! linear continuation.

use crate::MaritimeEvidenceEnvelope;
use serde::{Deserialize, Serialize};

/// Compact content-addressed head of one platform event stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MaritimeStreamHead {
    pub platform_id: String,
    pub sequence: u64,
    pub digest: String,
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
            sequence: root.sequence,
            digest: root.content_digest()?,
        })
    }

    /// Rehydrate a retained head, validating the digest shape before use.
    pub fn from_retained(
        platform_id: impl Into<String>,
        sequence: u64,
        digest: impl Into<String>,
    ) -> Result<Self, String> {
        let platform_id = platform_id.into();
        let digest = digest.into();
        if platform_id.trim().is_empty() {
            return Err("retained maritime stream platform_id cannot be empty".into());
        }
        if !is_blake3_hex(&digest) {
            return Err("retained maritime stream digest must be 64 hexadecimal characters".into());
        }
        Ok(Self {
            platform_id,
            sequence,
            digest: digest.to_ascii_lowercase(),
        })
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

        if candidate.previous_event_digest.as_deref() != Some(self.digest.as_str()) {
            return Ok(MaritimeStreamDisposition::Fork);
        }

        Ok(MaritimeStreamDisposition::Advance)
    }

    /// Classify and, only for a direct successor, advance this local cursor.
    ///
    /// Duplicate/stale/gap/fork/wrong-platform observations leave the cursor
    /// unchanged so recovery/reconciliation code must handle them explicitly.
    pub fn ingest(
        &mut self,
        candidate: &MaritimeEvidenceEnvelope,
    ) -> Result<MaritimeStreamDisposition, String> {
        let disposition = self.classify(candidate)?;
        if disposition == MaritimeStreamDisposition::Advance {
            self.sequence = candidate.sequence;
            self.digest = candidate.content_digest()?;
        }
        Ok(disposition)
    }
}

fn is_blake3_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
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

        assert_eq!(
            head.ingest(&next).unwrap(),
            MaritimeStreamDisposition::Advance
        );
        assert_eq!(head.sequence, 11);
        assert_eq!(
            head.ingest(&next).unwrap(),
            MaritimeStreamDisposition::Duplicate
        );
        assert_eq!(head.sequence, 11);
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
        gap.previous_event_digest = Some(head.digest.clone());
        assert_eq!(head.ingest(&gap).unwrap(), MaritimeStreamDisposition::Gap);

        let mut wrong = event(12);
        wrong.platform_id = "auv-02".into();
        wrong.previous_event_digest = Some(head.digest.clone());
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
    fn retained_head_requires_explicit_well_formed_identity_and_digest() {
        assert!(MaritimeStreamHead::from_retained("", 1, "11".repeat(32)).is_err());
        assert!(MaritimeStreamHead::from_retained("auv-01", 1, "xyz").is_err());
        assert!(MaritimeStreamHead::from_retained("auv-01", 1, "AA".repeat(32)).is_ok());
    }
}
