// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Provenance-family and import-receipt semantics for systemic evidence.
//!
//! This module prevents source packaging from being mistaken for independent
//! corroboration. A URL, retrieval event, or byte-for-byte mirror is not an
//! independent upstream source merely because it is a separate record.
//!
//! Core invariants:
//!
//! ```text
//! source copy != independent source
//! different URL != independent evidence
//! different provenance family != proven independence
//! same provenance family => not independent
//! derived source != independent corroboration
//! retrieval event != source identity
//! source artifact digest != statement semantic digest
//! ```

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

use crate::systemic::UnixMillis;

/// Opaque reference to a digest produced by an explicitly named algorithm.
///
/// This type does not interpret digest bytes or claim that two digests refer to
/// semantically equivalent objects. Callers must preserve what object was
/// hashed: source artifact, statement semantic form, receipt, etc.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct DigestRef {
    pub algorithm: String,
    pub value: String,
}

impl DigestRef {
    pub fn new(
        algorithm: impl Into<String>,
        value: impl Into<String>,
    ) -> Result<Self, &'static str> {
        let algorithm = algorithm.into();
        let value = value.into();
        if algorithm.trim().is_empty() {
            return Err("digest algorithm must not be empty");
        }
        if value.trim().is_empty() {
            return Err("digest value must not be empty");
        }
        Ok(Self { algorithm, value })
    }
}

/// Stable identifier for a common upstream provenance family.
///
/// A family groups source records that should not be counted as independent
/// corroboration solely because they were mirrored, repackaged, or derived from
/// the same upstream evidence.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProvenanceFamilyId(pub String);

impl ProvenanceFamilyId {
    pub fn new(value: impl Into<String>) -> Result<Self, &'static str> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err("provenance family id must not be empty");
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Why a source record belongs to a provenance family.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ProvenanceMembershipKind {
    /// Earliest known/originating record for this modeled family.
    Origin,
    /// A redistribution or mirror of another source record.
    MirrorOf { source_id: String },
    /// A transformed/derived representation whose upstream source is known.
    DerivedFrom { source_ids: Vec<String> },
    /// Multiple records are known to depend on a common upstream source/feed.
    CommonUpstreamKnown { source_ids: Vec<String> },
    /// Common upstream dependence is suspected but not resolved.
    CommonUpstreamUnknown,
}

/// Membership of one source record in one provenance family.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ProvenanceMembership {
    pub source_id: String,
    pub family_id: ProvenanceFamilyId,
    pub kind: ProvenanceMembershipKind,
    /// Human/machine-readable rationale or adapter evidence reference.
    pub basis: Option<String>,
}

/// Pairwise independence status between two provenance families.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IndependenceStatus {
    Independent,
    NotIndependent,
    #[default]
    Unknown,
}

/// Explicit evidence-backed assessment of provenance-family independence.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct IndependenceAssessment {
    pub left_family: ProvenanceFamilyId,
    pub right_family: ProvenanceFamilyId,
    pub status: IndependenceStatus,
    pub assessed_at: UnixMillis,
    pub basis: Vec<String>,
}

impl IndependenceAssessment {
    pub fn applies_to(&self, left: &ProvenanceFamilyId, right: &ProvenanceFamilyId) -> bool {
        (&self.left_family == left && &self.right_family == right)
            || (&self.left_family == right && &self.right_family == left)
    }
}

/// Resolve pairwise independence conservatively.
///
/// Same-family records are always non-independent. Distinct family IDs are not
/// themselves proof of independence; without an explicit applicable assessment,
/// the result remains `Unknown`.
pub fn resolve_independence(
    left: &ProvenanceFamilyId,
    right: &ProvenanceFamilyId,
    assessments: &[IndependenceAssessment],
) -> IndependenceStatus {
    if left == right {
        return IndependenceStatus::NotIndependent;
    }

    assessments
        .iter()
        .rev()
        .find(|assessment| assessment.applies_to(left, right))
        .map(|assessment| assessment.status)
        .unwrap_or(IndependenceStatus::Unknown)
}

/// Adapter identity bound into an import receipt.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct AdapterIdentity {
    pub name: String,
    pub version: String,
}

impl AdapterIdentity {
    pub fn new(name: impl Into<String>, version: impl Into<String>) -> Result<Self, &'static str> {
        let name = name.into();
        let version = version.into();
        if name.trim().is_empty() || version.trim().is_empty() {
            return Err("adapter name and version must not be empty");
        }
        Ok(Self { name, version })
    }
}

/// Deterministic import/normalization receipt payload.
///
/// The receipt records *what happened during ingestion* and remains distinct
/// from the source artifact and statement semantic identities. A future
/// canonical-receipt tranche may hash this payload after the temporal/canonical
/// encoding rules are frozen.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImportReceipt {
    pub receipt_id: String,
    pub source_id: String,
    pub source_locator: String,
    pub retrieved_at: UnixMillis,
    /// Digest of the exact retrieved source artifact bytes.
    pub source_artifact_digest: DigestRef,
    pub adapter: AdapterIdentity,
    pub source_schema: Option<String>,
    pub accepted_records: u64,
    pub rejected_records: u64,
    pub normalization_warnings: Vec<String>,
    /// Semantic statement digests produced by the adapter, once canonical
    /// statement hashing is qualified.
    pub produced_statement_digests: Vec<DigestRef>,
    pub provenance_family: ProvenanceFamilyId,
    pub parent_families: Vec<ProvenanceFamilyId>,
}

impl ImportReceipt {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.receipt_id.trim().is_empty() {
            return Err("receipt id must not be empty");
        }
        if self.source_id.trim().is_empty() {
            return Err("source id must not be empty");
        }
        if self.source_locator.trim().is_empty() {
            return Err("source locator must not be empty");
        }
        if self.adapter.name.trim().is_empty() || self.adapter.version.trim().is_empty() {
            return Err("adapter identity must not be empty");
        }
        if self.source_artifact_digest.algorithm.trim().is_empty()
            || self.source_artifact_digest.value.trim().is_empty()
        {
            return Err("source artifact digest must not be empty");
        }
        if self.provenance_family.0.trim().is_empty() {
            return Err("provenance family id must not be empty");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn family(id: &str) -> ProvenanceFamilyId {
        ProvenanceFamilyId::new(id).unwrap()
    }

    #[test]
    fn same_family_is_never_independent() {
        let a = family("family:a");
        let contradictory = IndependenceAssessment {
            left_family: a.clone(),
            right_family: a.clone(),
            status: IndependenceStatus::Independent,
            assessed_at: 100,
            basis: vec!["bad assessment".into()],
        };

        assert_eq!(
            resolve_independence(&a, &a, &[contradictory]),
            IndependenceStatus::NotIndependent
        );
    }

    #[test]
    fn different_family_ids_do_not_prove_independence() {
        let a = family("family:a");
        let b = family("family:b");

        assert_eq!(
            resolve_independence(&a, &b, &[]),
            IndependenceStatus::Unknown
        );
    }

    #[test]
    fn explicit_pairwise_assessment_can_establish_independence() {
        let a = family("family:a");
        let b = family("family:b");
        let assessment = IndependenceAssessment {
            left_family: a.clone(),
            right_family: b.clone(),
            status: IndependenceStatus::Independent,
            assessed_at: 100,
            basis: vec!["separate primary acquisition paths".into()],
        };

        assert_eq!(
            resolve_independence(&a, &b, &[assessment]),
            IndependenceStatus::Independent
        );
    }

    #[test]
    fn latest_applicable_assessment_wins_without_erasing_history() {
        let a = family("family:a");
        let b = family("family:b");
        let earlier = IndependenceAssessment {
            left_family: a.clone(),
            right_family: b.clone(),
            status: IndependenceStatus::Independent,
            assessed_at: 100,
            basis: vec!["initial registry review".into()],
        };
        let later = IndependenceAssessment {
            left_family: b.clone(),
            right_family: a.clone(),
            status: IndependenceStatus::NotIndependent,
            assessed_at: 200,
            basis: vec!["shared upstream feed discovered".into()],
        };

        assert_eq!(
            resolve_independence(&a, &b, &[earlier, later]),
            IndependenceStatus::NotIndependent
        );
    }

    #[test]
    fn digest_and_adapter_reject_empty_identity() {
        assert!(DigestRef::new("", "abcd").is_err());
        assert!(DigestRef::new("sha256", "").is_err());
        assert!(AdapterIdentity::new("", "1.0").is_err());
        assert!(AdapterIdentity::new("ocds", "").is_err());
    }

    #[test]
    fn import_receipt_keeps_artifact_and_statement_digests_distinct() {
        let receipt = ImportReceipt {
            receipt_id: "receipt:1".into(),
            source_id: "source:1".into(),
            source_locator: "https://example.invalid/source.json".into(),
            retrieved_at: 123,
            source_artifact_digest: DigestRef::new("sha256", "artifact").unwrap(),
            adapter: AdapterIdentity::new("example", "1").unwrap(),
            source_schema: Some("example:v1".into()),
            accepted_records: 2,
            rejected_records: 1,
            normalization_warnings: vec!["one record lacked optional label".into()],
            produced_statement_digests: vec![
                DigestRef::new("sha256", "statement-a").unwrap(),
                DigestRef::new("sha256", "statement-b").unwrap(),
            ],
            provenance_family: family("family:source-1"),
            parent_families: Vec::new(),
        };

        assert!(receipt.validate().is_ok());
        assert_ne!(
            receipt.source_artifact_digest,
            receipt.produced_statement_digests[0]
        );
    }
}
