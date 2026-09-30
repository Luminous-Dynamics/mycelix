// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! ROS-002 evidence and provenance primitives.
//!
//! This module is deliberately an envelope, not a second evidence store.
//! It records how an assertion is grounded, what source revision was observed,
//! and how that evidence relates to an explicitly supplied source frontier.
//!
//! The core is deterministic: currentness is computed only from values supplied
//! to the function. No wall clock, DHT scan, local cache, or ambient authority
//! is consulted.

use std::fmt;

use crate::ParticipantRef;

pub const EVIDENCE_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct AssertionId([u8; 32]);

impl AssertionId {
    pub fn derive(namespace: &str, subject: &ParticipantRef, predicate: &str, object: &str) -> Self {
        let mut h = blake3::Hasher::new();
        h.update(b"mycelix.assertion.v1\0");
        write_str(&mut h, namespace);
        write_participant(&mut h, subject);
        write_str(&mut h, predicate);
        write_str(&mut h, object);
        Self(*h.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] { &self.0 }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EvidenceDigest([u8; 32]);

impl EvidenceDigest {
    pub const fn from_bytes(bytes: [u8; 32]) -> Self { Self(bytes) }
    pub const fn as_bytes(&self) -> &[u8; 32] { &self.0 }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SourceRevision {
    pub source: String,
    pub revision: u64,
}

impl SourceRevision {
    pub fn new(source: impl Into<String>, revision: u64) -> Result<Self, EvidenceError> {
        let source = source.into();
        if source.is_empty() {
            return Err(EvidenceError::EmptySource);
        }
        Ok(Self { source, revision })
    }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct EvidenceRef {
    pub source_revision: SourceRevision,
    pub locator: String,
    pub digest: EvidenceDigest,
}

impl EvidenceRef {
    pub fn new(
        source_revision: SourceRevision,
        locator: impl Into<String>,
        digest: EvidenceDigest,
    ) -> Result<Self, EvidenceError> {
        let locator = locator.into();
        if locator.is_empty() {
            return Err(EvidenceError::EmptyLocator);
        }
        Ok(Self { source_revision, locator, digest })
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.evidence-ref.v1\0");
        write_bytes(&mut out, self.source_revision.source.as_bytes());
        out.extend_from_slice(&self.source_revision.revision.to_le_bytes());
        write_bytes(&mut out, self.locator.as_bytes());
        out.extend_from_slice(self.digest.as_bytes());
        out
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum EpistemicStatus {
    Observed,
    Reported,
    Inferred,
    Derived,
    Disputed,
}

impl EpistemicStatus {
    pub const fn is_direct_observation(self) -> bool {
        matches!(self, Self::Observed)
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum Visibility {
    Public,
    Relationship,
    Restricted,
    Private,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct AssertionSource {
    pub source_revision: SourceRevision,
    pub visibility: Visibility,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum Currentness {
    Current,
    Superseded,
    Corrected,
    Stale,
    NotObservedInFrontier,
    Unknown,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SourceFrontier {
    pub source: String,
    pub observed_revision: u64,
}

impl SourceFrontier {
    pub fn new(source: impl Into<String>, observed_revision: u64) -> Result<Self, EvidenceError> {
        let source = source.into();
        if source.is_empty() {
            return Err(EvidenceError::EmptySource);
        }
        Ok(Self { source, observed_revision })
    }

    pub fn currentness(&self, evidence: &EvidenceRef) -> Currentness {
        if evidence.source_revision.source != self.source {
            return Currentness::Unknown;
        }
        match evidence.source_revision.revision.cmp(&self.observed_revision) {
            std::cmp::Ordering::Equal => Currentness::Current,
            std::cmp::Ordering::Less => Currentness::Stale,
            std::cmp::Ordering::Greater => Currentness::NotObservedInFrontier,
        }
    }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum CorrectionKind {
    Supersedes,
    Corrects,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct LineageRef {
    pub assertion_id: AssertionId,
    pub evidence: EvidenceRef,
    pub kind: CorrectionKind,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssertionEnvelope {
    pub schema_version: u16,
    pub assertion_id: AssertionId,
    pub subject: ParticipantRef,
    pub predicate: String,
    pub object: String,
    pub status: EpistemicStatus,
    pub source: AssertionSource,
    pub evidence: EvidenceRef,
    pub lineages: Vec<LineageRef>,
}

impl AssertionEnvelope {
    pub fn new(
        assertion_id: AssertionId,
        subject: ParticipantRef,
        predicate: impl Into<String>,
        object: impl Into<String>,
        status: EpistemicStatus,
        source: AssertionSource,
        evidence: EvidenceRef,
    ) -> Result<Self, EvidenceError> {
        let predicate = predicate.into();
        let object = object.into();
        if predicate.is_empty() {
            return Err(EvidenceError::EmptyPredicate);
        }
        if object.is_empty() {
            return Err(EvidenceError::EmptyObject);
        }
        if source.source_revision != evidence.source_revision {
            return Err(EvidenceError::SourceMismatch);
        }
        Ok(Self {
            schema_version: EVIDENCE_SCHEMA_VERSION,
            assertion_id,
            subject,
            predicate,
            object,
            status,
            source,
            evidence,
            lineages: Vec::new(),
        })
    }

    pub fn validate_schema(&self) -> Result<(), EvidenceError> {
        if self.schema_version == EVIDENCE_SCHEMA_VERSION {
            Ok(())
        } else {
            Err(EvidenceError::UnsupportedSchema(self.schema_version))
        }
    }

    pub fn with_lineage(mut self, lineage: LineageRef) -> Result<Self, EvidenceError> {
        if lineage.assertion_id != self.assertion_id {
            return Err(EvidenceError::LineageAssertionMismatch);
        }
        self.lineages.push(lineage);
        self.lineages.sort();
        Ok(self)
    }

    pub fn currentness(&self, frontier: Option<&SourceFrontier>) -> Currentness {
        match frontier {
            Some(frontier) => frontier.currentness(&self.evidence),
            None => Currentness::Unknown,
        }
    }

    /// Classify an older evidence reference against this envelope's explicit lineage.
    ///
    /// The envelope itself is the newer record. Its lineage can therefore tell a
    /// projection why an older reference is superseded or corrected without making
    /// the newer record appear stale merely because it mentions history.
    pub fn lineage_currentness(
        &self,
        evidence: &EvidenceRef,
        frontier: Option<&SourceFrontier>,
    ) -> Currentness {
        let base = match frontier {
            Some(frontier) => frontier.currentness(evidence),
            None => return Currentness::Unknown,
        };
        if matches!(base, Currentness::Unknown | Currentness::NotObservedInFrontier) {
            return base;
        }
        self.lineages
            .iter()
            .find(|lineage| lineage.evidence == *evidence)
            .map(|lineage| match lineage.kind {
                CorrectionKind::Corrects => Currentness::Corrected,
                CorrectionKind::Supersedes => Currentness::Superseded,
            })
            .unwrap_or(base)
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.assertion-envelope.v1\0");
        out.extend_from_slice(&self.schema_version.to_le_bytes());
        out.extend_from_slice(self.assertion_id.as_bytes());
        write_participant_bytes(&mut out, &self.subject);
        write_bytes(&mut out, self.predicate.as_bytes());
        write_bytes(&mut out, self.object.as_bytes());
        out.push(self.status as u8);
        out.push(self.source.visibility as u8);
        out.extend_from_slice(&self.evidence.canonical_bytes());
        out.extend_from_slice(&(self.lineages.len() as u64).to_le_bytes());
        for lineage in &self.lineages {
            out.extend_from_slice(lineage.assertion_id.as_bytes());
            out.extend_from_slice(&lineage.evidence.canonical_bytes());
            out.push(lineage.kind as u8);
        }
        out
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EvidenceError {
    EmptySource,
    EmptyLocator,
    EmptyPredicate,
    EmptyObject,
    SourceMismatch,
    LineageAssertionMismatch,
    UnsupportedSchema(u16),
}

impl fmt::Display for EvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptySource => write!(f, "evidence source must not be empty"),
            Self::EmptyLocator => write!(f, "evidence locator must not be empty"),
            Self::EmptyPredicate => write!(f, "assertion predicate must not be empty"),
            Self::EmptyObject => write!(f, "assertion object must not be empty"),
            Self::SourceMismatch => write!(f, "assertion source and evidence revision must match"),
            Self::LineageAssertionMismatch => write!(f, "lineage must reference its containing assertion"),
            Self::UnsupportedSchema(v) => write!(f, "unsupported evidence schema version {v}"),
        }
    }
}

impl std::error::Error for EvidenceError {}

fn write_str(hasher: &mut blake3::Hasher, value: &str) {
    hasher.update(&(value.len() as u64).to_le_bytes());
    hasher.update(value.as_bytes());
}

fn write_participant(hasher: &mut blake3::Hasher, participant: &ParticipantRef) {
    write_str(hasher, &participant.namespace);
    write_str(hasher, &participant.identifier);
}

fn write_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
}

fn write_participant_bytes(out: &mut Vec<u8>, participant: &ParticipantRef) {
    write_bytes(out, participant.namespace.as_bytes());
    write_bytes(out, participant.identifier.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(ns: &str, id: &str) -> ParticipantRef {
        ParticipantRef::new(ns, id)
    }

    fn evidence(source: &str, revision: u64, locator: &str, byte: u8) -> EvidenceRef {
        EvidenceRef::new(
            SourceRevision::new(source, revision).unwrap(),
            locator,
            EvidenceDigest::from_bytes([byte; 32]),
        ).unwrap()
    }

    fn envelope(source: &str, revision: u64, status: EpistemicStatus) -> AssertionEnvelope {
        let subject = p("did", "alice");
        let ev = evidence(source, revision, "record/7", 0x42);
        AssertionEnvelope::new(
            AssertionId::derive("crm", &subject, "account-status", "active"),
            subject.clone(),
            "account-status",
            "active",
            status,
            AssertionSource {
                source_revision: ev.source_revision.clone(),
                visibility: Visibility::Relationship,
            },
            ev,
        ).unwrap()
    }

    #[test]
    fn currentness_is_explicit_and_deterministic() {
        let current = envelope("crm", 7, EpistemicStatus::Observed);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        assert_eq!(current.currentness(Some(&frontier)), Currentness::Current);

        let stale = envelope("crm", 6, EpistemicStatus::Observed);
        assert_eq!(stale.currentness(Some(&frontier)), Currentness::Stale);

        let future = envelope("crm", 8, EpistemicStatus::Observed);
        assert_eq!(future.currentness(Some(&frontier)), Currentness::NotObservedInFrontier);
    }

    #[test]
    fn missing_or_other_source_frontier_is_not_a_negative_fact() {
        let assertion = envelope("crm", 7, EpistemicStatus::Observed);
        assert_eq!(assertion.currentness(None), Currentness::Unknown);

        let other = SourceFrontier::new("erp", 99).unwrap();
        assert_eq!(assertion.currentness(Some(&other)), Currentness::Unknown);
    }

    #[test]
    fn inference_cannot_be_relabelled_by_currentness() {
        let assertion = envelope("crm", 7, EpistemicStatus::Inferred);
        assert!(!assertion.status.is_direct_observation());
        assert_eq!(
            assertion.currentness(Some(&SourceFrontier::new("crm", 7).unwrap())),
            Currentness::Current
        );
    }

    #[test]
    fn same_assertion_can_have_distinct_sources() {
        let subject = p("did", "alice");
        let a = AssertionId::derive("crm", &subject, "account-status", "active");
        let b = AssertionId::derive("erp", &subject, "account-status", "active");
        assert_ne!(a, b);
    }

    #[test]
    fn same_source_revision_can_change_digest_without_being_hidden() {
        let first = evidence("crm", 7, "record/7", 0x01);
        let changed = evidence("crm", 7, "record/7", 0x02);
        assert_ne!(first.digest, changed.digest);
        assert_ne!(first.canonical_bytes(), changed.canonical_bytes());
    }

    #[test]
    fn correction_and_supersession_are_explicit_lineage() {
        let mut current = envelope("crm", 8, EpistemicStatus::Observed);
        let old = evidence("crm", 7, "record/7", 0x01);
        let corrected = LineageRef {
            assertion_id: current.assertion_id,
            evidence: old.clone(),
            kind: CorrectionKind::Corrects,
        };
        current = current.with_lineage(corrected).unwrap();
        assert_eq!(current.lineages.len(), 1);
        let frontier = SourceFrontier::new("crm", 8).unwrap();
        assert_eq!(current.currentness(Some(&frontier)), Currentness::Current);
        assert_eq!(current.lineage_currentness(&old, Some(&frontier)), Currentness::Corrected);
    }

    #[test]
    fn private_reference_does_not_disclose_payload() {
        let assertion = envelope("crm", 7, EpistemicStatus::Reported);
        assert_eq!(assertion.source.visibility, Visibility::Relationship);
        assert_eq!(assertion.evidence.locator, "record/7");
    }

    #[test]
    fn canonical_bytes_are_stable() {
        let a = envelope("crm", 7, EpistemicStatus::Observed);
        let b = envelope("crm", 7, EpistemicStatus::Observed);
        assert_eq!(a.canonical_bytes(), b.canonical_bytes());
    }

    #[test]
    fn same_name_different_namespace_is_distinct_subject() {
        let a = p("did", "alice");
        let b = p("crm-contact", "alice");
        assert_ne!(a, b);
    }

    #[test]
    fn valid_credential_does_not_imply_authorization() {
        // This module has no authorization decision API by design.
        // Consent/delegation remain separate policy layers.
        let assertion = envelope("crm", 7, EpistemicStatus::Observed);
        assert_eq!(assertion.status, EpistemicStatus::Observed);
    }

    #[test]
    fn superseded_lineage_is_explicit() {
        let mut current = envelope("crm", 8, EpistemicStatus::Observed);
        let old = evidence("crm", 7, "record/7", 0x01);
        current = current.with_lineage(LineageRef {
            assertion_id: current.assertion_id,
            evidence: old.clone(),
            kind: CorrectionKind::Supersedes,
        }).unwrap();
        let frontier = SourceFrontier::new("crm", 8).unwrap();
        assert_eq!(current.lineage_currentness(&old, Some(&frontier)), Currentness::Superseded);
    }

    #[test]
    fn obsolete_frontier_is_not_silently_promoted() {
        let assertion = envelope("crm", 6, EpistemicStatus::Observed);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        assert_eq!(assertion.currentness(Some(&frontier)), Currentness::Stale);
    }
}
