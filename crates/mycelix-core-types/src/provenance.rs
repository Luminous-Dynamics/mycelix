// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Provider-neutral provenance identity primitives for systemic evidence.
//!
//! This module deliberately stops short of canonical statement serialization.
//! It separates record identity, content identity, acquisition identity, and
//! provenance-family semantics so later canonical hashing cannot collapse them.
//!
//! Core invariants:
//!
//! ```text
//! record identifier != content digest
//! statement digest != source-artifact digest
//! same URL != same evidence bytes
//! different URL != independent evidence
//! mirror != independent corroboration
//! derived analysis != source observation
//! unknown independence != independent
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::systemic::UnixMillis;

/// Namespace-qualified record identifier.
///
/// A local identifier such as `12345` has no global meaning without its
/// namespace. This type is for source/import record identity, not content
/// identity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ScopedRecordId {
    pub namespace: String,
    pub local_id: String,
}

impl ScopedRecordId {
    pub fn new(
        namespace: impl Into<String>,
        local_id: impl Into<String>,
    ) -> Result<Self, &'static str> {
        let namespace = namespace.into();
        let local_id = local_id.into();
        if namespace.is_empty() {
            return Err("record-id namespace must not be empty");
        }
        if local_id.is_empty() {
            return Err("record-id local id must not be empty");
        }
        Ok(Self {
            namespace,
            local_id,
        })
    }
}

/// Digest algorithm identity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum DigestAlgorithm {
    Sha256,
    Blake3,
    Other(String),
}

impl DigestAlgorithm {
    pub fn name(&self) -> &str {
        match self {
            Self::Sha256 => "sha256",
            Self::Blake3 => "blake3",
            Self::Other(name) => name.as_str(),
        }
    }

    fn expected_len(&self) -> Option<usize> {
        match self {
            Self::Sha256 | Self::Blake3 => Some(32),
            Self::Other(_) => None,
        }
    }
}

/// Typed content digest.
///
/// Digest bytes are kept separate from record/source identifiers. This module
/// does not choose a provider or claim that a digest authenticates the meaning
/// of the bytes it identifies.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContentDigest {
    pub algorithm: DigestAlgorithm,
    pub bytes: Vec<u8>,
}

impl ContentDigest {
    pub fn new(algorithm: DigestAlgorithm, bytes: Vec<u8>) -> Result<Self, &'static str> {
        if bytes.is_empty() {
            return Err("digest bytes must not be empty");
        }
        if let Some(expected) = algorithm.expected_len() {
            if bytes.len() != expected {
                return Err("digest byte length does not match algorithm");
            }
        }
        if let DigestAlgorithm::Other(name) = &algorithm {
            if name.is_empty() {
                return Err("custom digest algorithm name must not be empty");
            }
        }
        Ok(Self { algorithm, bytes })
    }

    pub fn to_prefixed_hex(&self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(self.algorithm.name().len() + 1 + self.bytes.len() * 2);
        out.push_str(self.algorithm.name());
        out.push(':');
        for byte in &self.bytes {
            out.push(HEX[(byte >> 4) as usize] as char);
            out.push(HEX[(byte & 0x0f) as usize] as char);
        }
        out
    }
}

/// Identity for an upstream provenance family.
///
/// Two retrievals may have different URLs and acquisition IDs while still
/// belonging to the same provenance family because one mirrors or republishes
/// the other.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProvenanceFamilyId(pub ScopedRecordId);

/// How a concrete acquisition entered the evidence system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum AcquisitionKind {
    DirectSource,
    Mirror,
    DerivedAnalysis,
}

/// One concrete retrieval/acquisition event.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SourceAcquisition {
    pub acquisition_id: ScopedRecordId,
    pub source_record_id: Option<ScopedRecordId>,
    pub locator: String,
    pub retrieved_at: UnixMillis,
    pub source_artifact_digest: ContentDigest,
    pub provenance_family: ProvenanceFamilyId,
    pub kind: AcquisitionKind,
}

/// Relationship between concrete acquisition records.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ProvenanceRelationKind {
    MirrorsSource,
    DerivedFromSource,
    CommonUpstream,
    IndependentAcquisition,
    CommonUpstreamUnknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProvenanceRelation {
    pub relation_id: ScopedRecordId,
    pub from_acquisition: ScopedRecordId,
    pub relation: ProvenanceRelationKind,
    pub to_acquisition: ScopedRecordId,
}

/// Pairwise assessment of source-family independence.
///
/// `Unknown` is intentionally distinct from `Independent`; callers must not
/// promote missing lineage evidence into corroboration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IndependenceStatus {
    Independent,
    NotIndependent,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IndependenceAssessment {
    pub left_family: ProvenanceFamilyId,
    pub right_family: ProvenanceFamilyId,
    pub status: IndependenceStatus,
    /// Evidence records supporting this assessment. Empty evidence MUST NOT
    /// silently imply `Independent` at a higher authority layer.
    pub evidence: Vec<ScopedRecordId>,
}

/// Adapter identity recorded in an import receipt.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AdapterIdentity {
    pub name: String,
    pub version: String,
    pub source_schema_version: Option<String>,
}

/// Accepted/rejected record counts for one import attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImportCounts {
    pub accepted: u64,
    pub rejected: u64,
}

impl ImportCounts {
    pub fn total(self) -> Option<u64> {
        self.accepted.checked_add(self.rejected)
    }
}

/// Explicit normalization warning/loss surfaced by an adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct NormalizationWarning {
    pub code: String,
    pub message: String,
    pub affected_records: u64,
}

/// Import/normalization receipt inputs.
///
/// This structure binds provenance facts needed for deterministic receipt
/// encoding, but this tranche does not yet define the canonical receipt bytes or
/// statement canonicalization profile. Those depend on the qualified temporal
/// semantics and a versioned canonical encoding profile.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImportReceipt {
    pub receipt_id: ScopedRecordId,
    pub acquisition_id: ScopedRecordId,
    pub source_artifact_digest: ContentDigest,
    pub adapter: AdapterIdentity,
    pub counts: ImportCounts,
    pub warnings: Vec<NormalizationWarning>,
    /// Digests supplied by the later canonical-statement layer.
    pub produced_statement_digests: Vec<ContentDigest>,
    pub parent_provenance_families: Vec<ProvenanceFamilyId>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn sha(byte: u8) -> ContentDigest {
        ContentDigest::new(DigestAlgorithm::Sha256, vec![byte; 32]).unwrap()
    }

    fn id(ns: &str, local: &str) -> ScopedRecordId {
        ScopedRecordId::new(ns, local).unwrap()
    }

    #[test]
    fn equal_local_ids_in_different_namespaces_are_distinct() {
        let first = id("ocds:release", "12345");
        let second = id("companies-house", "12345");
        assert_ne!(first, second);

        let mut ids = HashSet::new();
        ids.insert(first);
        ids.insert(second);
        assert_eq!(ids.len(), 2);
    }

    #[test]
    fn known_digest_algorithms_enforce_output_length() {
        assert!(ContentDigest::new(DigestAlgorithm::Sha256, vec![0; 31]).is_err());
        assert!(ContentDigest::new(DigestAlgorithm::Sha256, vec![0; 32]).is_ok());
        assert!(ContentDigest::new(DigestAlgorithm::Blake3, vec![0; 32]).is_ok());
    }

    #[test]
    fn digest_algorithm_is_part_of_content_identity() {
        let bytes = vec![7; 32];
        let sha = ContentDigest::new(DigestAlgorithm::Sha256, bytes.clone()).unwrap();
        let blake = ContentDigest::new(DigestAlgorithm::Blake3, bytes).unwrap();
        assert_ne!(sha, blake);
        assert!(sha.to_prefixed_hex().starts_with("sha256:"));
    }

    #[test]
    fn different_urls_can_share_artifact_and_provenance_family() {
        let family = ProvenanceFamilyId(id("mycelix:provenance-family:v1", "filing-42"));
        let digest = sha(4);

        let origin = SourceAcquisition {
            acquisition_id: id("mycelix:acquisition:v1", "a"),
            source_record_id: Some(id("registry:test", "42")),
            locator: "https://registry.example/42".into(),
            retrieved_at: 100,
            source_artifact_digest: digest.clone(),
            provenance_family: family.clone(),
            kind: AcquisitionKind::DirectSource,
        };
        let mirror = SourceAcquisition {
            acquisition_id: id("mycelix:acquisition:v1", "b"),
            source_record_id: None,
            locator: "https://mirror.example/42".into(),
            retrieved_at: 200,
            source_artifact_digest: digest,
            provenance_family: family,
            kind: AcquisitionKind::Mirror,
        };

        assert_ne!(origin.acquisition_id, mirror.acquisition_id);
        assert_ne!(origin.locator, mirror.locator);
        assert_eq!(origin.source_artifact_digest, mirror.source_artifact_digest);
        assert_eq!(origin.provenance_family, mirror.provenance_family);
    }

    #[test]
    fn unknown_independence_is_not_independent() {
        let assessment = IndependenceAssessment {
            left_family: ProvenanceFamilyId(id("family", "a")),
            right_family: ProvenanceFamilyId(id("family", "b")),
            status: IndependenceStatus::Unknown,
            evidence: vec![],
        };
        assert_ne!(assessment.status, IndependenceStatus::Independent);
    }

    #[test]
    fn derived_analysis_is_explicitly_not_a_direct_source_kind() {
        assert_ne!(AcquisitionKind::DerivedAnalysis, AcquisitionKind::DirectSource);
    }

    #[test]
    fn import_counts_fail_closed_on_overflow() {
        assert_eq!(
            ImportCounts {
                accepted: 7,
                rejected: 3,
            }
            .total(),
            Some(10)
        );
        assert_eq!(
            ImportCounts {
                accepted: u64::MAX,
                rejected: 1,
            }
            .total(),
            None
        );
    }
}