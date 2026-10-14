// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Provider-neutral provenance identity primitives for systemic evidence.
//!
//! This module deliberately stops short of canonical statement serialization
//! and checked wire admission. Validated semantic types are Serialize-only in
//! this tranche: generic deserialization would bypass constructor validation.
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
//! independence claim without evidence != admitted independence
//! serialized bytes != validated semantic admission
//! ```

#[cfg(feature = "serde")]
use serde::Serialize;

use crate::systemic::UnixMillis;

/// Namespace-qualified record identifier.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ScopedRecordId {
    namespace: String,
    local_id: String,
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

    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    pub fn local_id(&self) -> &str {
        &self.local_id
    }
}

/// Validated name for a digest algorithm not represented by a built-in variant.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct DigestAlgorithmName(String);

impl DigestAlgorithmName {
    pub fn new(name: impl Into<String>) -> Result<Self, &'static str> {
        let name = name.into();
        if name.is_empty() {
            return Err("custom digest algorithm name must not be empty");
        }
        Ok(Self(name))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Digest algorithm identity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub enum DigestAlgorithm {
    Sha256,
    Blake3,
    Other(DigestAlgorithmName),
}

impl DigestAlgorithm {
    pub fn other(name: impl Into<String>) -> Result<Self, &'static str> {
        Ok(Self::Other(DigestAlgorithmName::new(name)?))
    }

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
/// Digest identity does not imply authenticity, authorship, semantic truth, or
/// correctness of a future digest provider implementation.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ContentDigest {
    algorithm: DigestAlgorithm,
    bytes: Vec<u8>,
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
        Ok(Self { algorithm, bytes })
    }

    pub fn algorithm(&self) -> &DigestAlgorithm {
        &self.algorithm
    }

    pub fn bytes(&self) -> &[u8] {
        &self.bytes
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
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ProvenanceFamilyId(ScopedRecordId);

impl ProvenanceFamilyId {
    pub fn new(id: ScopedRecordId) -> Self {
        Self(id)
    }

    pub fn record_id(&self) -> &ScopedRecordId {
        &self.0
    }
}

/// How a concrete acquisition entered the evidence system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub enum AcquisitionKind {
    DirectSource,
    Mirror,
    DerivedAnalysis,
}

/// One concrete retrieval/acquisition event.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct SourceAcquisition {
    acquisition_id: ScopedRecordId,
    source_record_id: Option<ScopedRecordId>,
    locator: String,
    retrieved_at: UnixMillis,
    source_artifact_digest: ContentDigest,
    provenance_family: ProvenanceFamilyId,
    kind: AcquisitionKind,
}

impl SourceAcquisition {
    pub fn new(
        acquisition_id: ScopedRecordId,
        locator: impl Into<String>,
        retrieved_at: UnixMillis,
        source_artifact_digest: ContentDigest,
        provenance_family: ProvenanceFamilyId,
        kind: AcquisitionKind,
    ) -> Result<Self, &'static str> {
        let locator = locator.into();
        if locator.is_empty() {
            return Err("source locator must not be empty");
        }
        Ok(Self {
            acquisition_id,
            source_record_id: None,
            locator,
            retrieved_at,
            source_artifact_digest,
            provenance_family,
            kind,
        })
    }

    pub fn with_source_record_id(mut self, source_record_id: ScopedRecordId) -> Self {
        self.source_record_id = Some(source_record_id);
        self
    }

    pub fn acquisition_id(&self) -> &ScopedRecordId {
        &self.acquisition_id
    }

    pub fn source_record_id(&self) -> Option<&ScopedRecordId> {
        self.source_record_id.as_ref()
    }

    pub fn locator(&self) -> &str {
        &self.locator
    }

    pub fn retrieved_at(&self) -> UnixMillis {
        self.retrieved_at
    }

    pub fn source_artifact_digest(&self) -> &ContentDigest {
        &self.source_artifact_digest
    }

    pub fn provenance_family(&self) -> &ProvenanceFamilyId {
        &self.provenance_family
    }

    pub fn kind(&self) -> AcquisitionKind {
        self.kind
    }
}

/// Relationship between concrete acquisition records.
///
/// Independence is intentionally absent. It requires an evidence-bearing
/// `IndependenceAssessment`, not a bare relation label.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub enum ProvenanceRelationKind {
    MirrorsSource,
    DerivedFromSource,
    CommonUpstream,
    CommonUpstreamUnknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ProvenanceRelation {
    relation_id: ScopedRecordId,
    from_acquisition: ScopedRecordId,
    relation: ProvenanceRelationKind,
    to_acquisition: ScopedRecordId,
}

impl ProvenanceRelation {
    pub fn new(
        relation_id: ScopedRecordId,
        from_acquisition: ScopedRecordId,
        relation: ProvenanceRelationKind,
        to_acquisition: ScopedRecordId,
    ) -> Self {
        Self {
            relation_id,
            from_acquisition,
            relation,
            to_acquisition,
        }
    }

    pub fn relation(&self) -> ProvenanceRelationKind {
        self.relation
    }
}

/// Pairwise assessment of source-family independence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub enum IndependenceStatus {
    Independent,
    NotIndependent,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct IndependenceAssessment {
    left_family: ProvenanceFamilyId,
    right_family: ProvenanceFamilyId,
    status: IndependenceStatus,
    evidence: Vec<ScopedRecordId>,
}

impl IndependenceAssessment {
    pub fn new(
        left_family: ProvenanceFamilyId,
        right_family: ProvenanceFamilyId,
        status: IndependenceStatus,
        evidence: Vec<ScopedRecordId>,
    ) -> Result<Self, &'static str> {
        if status == IndependenceStatus::Independent && left_family == right_family {
            return Err("the same provenance family cannot independently corroborate itself");
        }
        if status != IndependenceStatus::Unknown && evidence.is_empty() {
            return Err("non-unknown independence assessment requires evidence");
        }
        Ok(Self {
            left_family,
            right_family,
            status,
            evidence,
        })
    }

    pub fn left_family(&self) -> &ProvenanceFamilyId {
        &self.left_family
    }

    pub fn right_family(&self) -> &ProvenanceFamilyId {
        &self.right_family
    }

    pub fn status(&self) -> IndependenceStatus {
        self.status
    }

    pub fn evidence(&self) -> &[ScopedRecordId] {
        &self.evidence
    }
}

/// Adapter identity recorded in an import receipt.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct AdapterIdentity {
    name: String,
    version: String,
    source_schema_version: Option<String>,
}

impl AdapterIdentity {
    pub fn new(name: impl Into<String>, version: impl Into<String>) -> Result<Self, &'static str> {
        let name = name.into();
        let version = version.into();
        if name.is_empty() {
            return Err("adapter name must not be empty");
        }
        if version.is_empty() {
            return Err("adapter version must not be empty");
        }
        Ok(Self {
            name,
            version,
            source_schema_version: None,
        })
    }

    pub fn with_source_schema_version(mut self, version: impl Into<String>) -> Self {
        self.source_schema_version = Some(version.into());
        self
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn version(&self) -> &str {
        &self.version
    }
}

/// Accepted/rejected record counts for one import attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ImportCounts {
    accepted: u64,
    rejected: u64,
}

impl ImportCounts {
    pub const fn new(accepted: u64, rejected: u64) -> Self {
        Self { accepted, rejected }
    }

    pub const fn accepted(self) -> u64 {
        self.accepted
    }

    pub const fn rejected(self) -> u64 {
        self.rejected
    }

    pub fn total(self) -> Option<u64> {
        self.accepted.checked_add(self.rejected)
    }
}

/// Explicit normalization warning/loss surfaced by an adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct NormalizationWarning {
    code: String,
    message: String,
    affected_records: u64,
}

impl NormalizationWarning {
    pub fn new(
        code: impl Into<String>,
        message: impl Into<String>,
        affected_records: u64,
    ) -> Result<Self, &'static str> {
        let code = code.into();
        let message = message.into();
        if code.is_empty() {
            return Err("normalization warning code must not be empty");
        }
        if message.is_empty() {
            return Err("normalization warning message must not be empty");
        }
        Ok(Self {
            code,
            message,
            affected_records,
        })
    }
}

/// Import/normalization receipt inputs.
///
/// This structure binds provenance facts needed for deterministic receipt
/// encoding, but this tranche does not yet define canonical receipt bytes or
/// canonical statement serialization. Checked wire DTO → semantic conversions
/// belong in a later tranche; generic `Deserialize` is intentionally absent.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize))]
pub struct ImportReceipt {
    receipt_id: ScopedRecordId,
    acquisition_id: ScopedRecordId,
    source_artifact_digest: ContentDigest,
    adapter: AdapterIdentity,
    counts: ImportCounts,
    warnings: Vec<NormalizationWarning>,
    produced_statement_digests: Vec<ContentDigest>,
    parent_provenance_families: Vec<ProvenanceFamilyId>,
}

impl ImportReceipt {
    pub fn new(
        receipt_id: ScopedRecordId,
        acquisition_id: ScopedRecordId,
        source_artifact_digest: ContentDigest,
        adapter: AdapterIdentity,
        counts: ImportCounts,
    ) -> Self {
        Self {
            receipt_id,
            acquisition_id,
            source_artifact_digest,
            adapter,
            counts,
            warnings: Vec::new(),
            produced_statement_digests: Vec::new(),
            parent_provenance_families: Vec::new(),
        }
    }

    pub fn add_warning(&mut self, warning: NormalizationWarning) {
        self.warnings.push(warning);
    }

    pub fn add_statement_digest(&mut self, digest: ContentDigest) {
        self.produced_statement_digests.push(digest);
    }

    pub fn add_parent_provenance_family(&mut self, family: ProvenanceFamilyId) {
        self.parent_provenance_families.push(family);
    }
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

    fn family(local: &str) -> ProvenanceFamilyId {
        ProvenanceFamilyId::new(id("mycelix:provenance-family:v1", local))
    }

    #[test]
    fn empty_record_identity_is_rejected() {
        assert!(ScopedRecordId::new("", "1").is_err());
        assert!(ScopedRecordId::new("namespace", "").is_err());
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
    fn custom_digest_algorithm_requires_non_empty_name() {
        assert!(DigestAlgorithm::other("").is_err());
        assert!(DigestAlgorithm::other("sha3-256").is_ok());
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
        let shared_family = family("filing-42");
        let digest = sha(4);

        let origin = SourceAcquisition::new(
            id("mycelix:acquisition:v1", "a"),
            "https://registry.example/42",
            100,
            digest.clone(),
            shared_family.clone(),
            AcquisitionKind::DirectSource,
        )
        .unwrap()
        .with_source_record_id(id("registry:test", "42"));
        let mirror = SourceAcquisition::new(
            id("mycelix:acquisition:v1", "b"),
            "https://mirror.example/42",
            200,
            digest,
            shared_family,
            AcquisitionKind::Mirror,
        )
        .unwrap();

        assert_ne!(origin.acquisition_id(), mirror.acquisition_id());
        assert_ne!(origin.locator(), mirror.locator());
        assert_eq!(origin.source_artifact_digest(), mirror.source_artifact_digest());
        assert_eq!(origin.provenance_family(), mirror.provenance_family());
    }

    #[test]
    fn independence_requires_evidence_and_distinct_families() {
        assert!(
            IndependenceAssessment::new(
                family("a"),
                family("b"),
                IndependenceStatus::Independent,
                vec![],
            )
            .is_err()
        );
        assert!(
            IndependenceAssessment::new(
                family("a"),
                family("a"),
                IndependenceStatus::Independent,
                vec![id("evidence", "1")],
            )
            .is_err()
        );
        assert!(
            IndependenceAssessment::new(
                family("a"),
                family("b"),
                IndependenceStatus::Independent,
                vec![id("evidence", "1")],
            )
            .is_ok()
        );
    }

    #[test]
    fn unknown_independence_remains_explicit_without_evidence() {
        let assessment = IndependenceAssessment::new(
            family("a"),
            family("b"),
            IndependenceStatus::Unknown,
            vec![],
        )
        .unwrap();
        assert_eq!(assessment.status(), IndependenceStatus::Unknown);
        assert_ne!(assessment.status(), IndependenceStatus::Independent);
    }

    #[test]
    fn derived_analysis_is_explicitly_not_a_direct_source_kind() {
        assert_ne!(AcquisitionKind::DerivedAnalysis, AcquisitionKind::DirectSource);
    }

    #[test]
    fn adapter_identity_rejects_empty_protocol_identity() {
        assert!(AdapterIdentity::new("", "1.0").is_err());
        assert!(AdapterIdentity::new("ocds", "").is_err());
        assert!(AdapterIdentity::new("ocds", "1.0").is_ok());
    }

    #[test]
    fn import_counts_fail_closed_on_overflow() {
        assert_eq!(ImportCounts::new(7, 3).total(), Some(10));
        assert_eq!(ImportCounts::new(u64::MAX, 1).total(), None);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn validated_types_remain_serializable_without_generic_deserialization() {
        let value = id("registry:test", "42");
        let encoded = serde_json::to_string(&value).unwrap();
        assert!(encoded.contains("registry:test"));
        assert!(encoded.contains("42"));
    }
}