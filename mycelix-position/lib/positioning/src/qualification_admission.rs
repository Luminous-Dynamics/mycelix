// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Durable admission profile for qualification evidence.
//!
//! This module keeps three claims separate:
//!
//! 1. encoded bytes are admitted before a parser sees them;
//! 2. parsed semantic objects fit bounded per-field and aggregate budgets; and
//! 3. the admission policy itself has architecture-independent canonical bytes.
//!
//! Canonical profile bytes are produced here, but hashing/signing remains a
//! separate cryptographic-evidence theorem.

use crate::qualification::{
    QualificationManifest, QualificationRequirementProfile, RequirementEvaluation,
    TheoremDefinition, TheoremId,
};
use crate::qualification_hardening::{
    EstablishmentExplanation, HardenedTheoremRegistry, QualificationHardeningError,
    QualificationLimits,
};
use serde::{Deserialize, Serialize};

/// Durable schema version for [`QualificationAdmissionProfileV1`].
pub const QUALIFICATION_ADMISSION_PROFILE_SCHEMA_V1: u16 = 1;

/// Identifier for the canonical admission-profile byte preimage.
pub const QUALIFICATION_ADMISSION_PROFILE_PREIMAGE_V1: &str =
    "mycelix-position-qualification-admission-profile-preimage-v1";

const PROFILE_DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-POSITION-QUALIFICATION-ADMISSION\0V1\0";
const MAX_CANONICAL_IDENTIFIER_BYTES: usize = 128;
const PORTABLE_USIZE_MAX: u64 = u32::MAX as u64;

/// Fixed-width durable admission policy.
///
/// Fields converted into [`QualificationLimits`] must fit the portable
/// `u32::MAX` ceiling before conversion to `usize`, so the same durable profile
/// has the same acceptance meaning on 64-bit native and wasm32 targets.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualificationAdmissionProfileV1 {
    pub schema_version: u16,
    pub profile_id: String,
    pub encoding_id: String,

    /// Maximum encoded qualification envelope accepted before deserialization.
    pub max_encoded_bytes: u64,

    // Per-field semantic/runtime limits mirrored from v0.2.
    pub max_theorems: u64,
    pub max_prerequisites_per_theorem: u64,
    pub max_total_prerequisite_edges: u64,
    pub max_facets: u64,
    pub max_required_theorems: u64,
    pub max_accepted_statuses: u64,
    pub max_evidence_refs_per_facet: u64,
    pub max_dependency_commitments_per_facet: u64,
    pub max_explanation_blockers: u64,
    pub max_explanation_path_theorems: u64,
    pub max_theorem_id_bytes: u64,
    pub max_profile_id_bytes: u64,
    pub max_subject_commitment_bytes: u64,
    pub max_verifier_profile_bytes: u64,
    pub max_evidence_ref_bytes: u64,
    pub max_dependency_commitment_bytes: u64,
    pub max_diagnostics_commitment_bytes: u64,

    // Aggregate post-parse budgets.
    pub max_total_registry_identifier_bytes: u64,
    pub max_total_manifest_string_bytes: u64,
    pub max_total_manifest_evidence_refs: u64,
    pub max_total_manifest_dependency_commitments: u64,
}

impl Default for QualificationAdmissionProfileV1 {
    fn default() -> Self {
        Self {
            schema_version: QUALIFICATION_ADMISSION_PROFILE_SCHEMA_V1,
            profile_id: "mycelix-position-qualification-admission-default-v1".into(),
            encoding_id: "mycelix-position-qualification-envelope-v1".into(),
            max_encoded_bytes: 2 * 1024 * 1024,
            max_theorems: 2_048,
            max_prerequisites_per_theorem: 32,
            max_total_prerequisite_edges: 65_536,
            max_facets: 2_048,
            max_required_theorems: 256,
            max_accepted_statuses: 7,
            max_evidence_refs_per_facet: 64,
            max_dependency_commitments_per_facet: 64,
            max_explanation_blockers: 256,
            max_explanation_path_theorems: 2_048,
            max_theorem_id_bytes: 192,
            max_profile_id_bytes: 192,
            max_subject_commitment_bytes: 512,
            max_verifier_profile_bytes: 512,
            max_evidence_ref_bytes: 2_048,
            max_dependency_commitment_bytes: 1_024,
            max_diagnostics_commitment_bytes: 1_024,
            max_total_registry_identifier_bytes: 16 * 1024 * 1024,
            max_total_manifest_string_bytes: 8 * 1024 * 1024,
            max_total_manifest_evidence_refs: 32_768,
            max_total_manifest_dependency_commitments: 32_768,
        }
    }
}

/// Runtime-limit field rejected by the portable cross-platform ceiling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AdmissionLimitField {
    Theorems,
    PrerequisitesPerTheorem,
    TotalPrerequisiteEdges,
    Facets,
    RequiredTheorems,
    AcceptedStatuses,
    EvidenceRefsPerFacet,
    DependencyCommitmentsPerFacet,
    ExplanationBlockers,
    ExplanationPathTheorems,
    TheoremIdBytes,
    ProfileIdBytes,
    SubjectCommitmentBytes,
    VerifierProfileBytes,
    EvidenceRefBytes,
    DependencyCommitmentBytes,
    DiagnosticsCommitmentBytes,
}

/// Aggregate semantic resource checked after parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum AdmissionAggregateResource {
    RegistryIdentifierBytes,
    ManifestStringBytes,
    ManifestEvidenceRefs,
    ManifestDependencyCommitments,
}

/// Durable admission failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationAdmissionError {
    UnsupportedSchemaVersion {
        actual: u16,
    },
    EmptyProfileId,
    EmptyEncodingId,
    NonCanonicalProfileId,
    NonCanonicalEncodingId,
    IdentifierTooLong {
        field: &'static str,
        limit: usize,
        actual: usize,
    },
    NonPortableRuntimeLimit {
        field: AdmissionLimitField,
        portable_max: u64,
        actual: u64,
    },
    EncodingMismatch,
    EncodedPayloadTooLarge {
        limit: u64,
        actual: u64,
    },
    CanonicalEncodingOverflow,
    AggregateBudgetExceeded {
        resource: AdmissionAggregateResource,
        limit: u64,
        actual: u64,
    },
    AggregateArithmeticOverflow {
        resource: AdmissionAggregateResource,
    },
    Semantic(QualificationHardeningError),
}

impl From<QualificationHardeningError> for QualificationAdmissionError {
    fn from(value: QualificationHardeningError) -> Self {
        Self::Semantic(value)
    }
}

impl std::fmt::Display for QualificationAdmissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnsupportedSchemaVersion { actual } => write!(
                f,
                "unsupported qualification admission profile schema version {actual}"
            ),
            Self::EmptyProfileId => {
                write!(f, "qualification admission profile id must not be empty")
            }
            Self::EmptyEncodingId => {
                write!(f, "qualification admission encoding id must not be empty")
            }
            Self::NonCanonicalProfileId => {
                write!(
                    f,
                    "qualification admission profile id is not canonical ASCII"
                )
            }
            Self::NonCanonicalEncodingId => {
                write!(
                    f,
                    "qualification admission encoding id is not canonical ASCII"
                )
            }
            Self::IdentifierTooLong {
                field,
                limit,
                actual,
            } => write!(
                f,
                "qualification admission {field} exceeds identifier limit: {actual} > {limit}"
            ),
            Self::NonPortableRuntimeLimit {
                field,
                portable_max,
                actual,
            } => write!(
                f,
                "qualification runtime limit {field:?} exceeds portable ceiling: {actual} > {portable_max}"
            ),
            Self::EncodingMismatch => write!(f, "qualification envelope encoding mismatch"),
            Self::EncodedPayloadTooLarge { limit, actual } => write!(
                f,
                "qualification envelope exceeds encoded-byte limit: {actual} > {limit}"
            ),
            Self::CanonicalEncodingOverflow => {
                write!(
                    f,
                    "qualification admission canonical encoding length overflow"
                )
            }
            Self::AggregateBudgetExceeded {
                resource,
                limit,
                actual,
            } => write!(
                f,
                "qualification aggregate resource {resource:?} exceeds limit: {actual} > {limit}"
            ),
            Self::AggregateArithmeticOverflow { resource } => write!(
                f,
                "qualification aggregate resource {resource:?} overflowed during accounting"
            ),
            Self::Semantic(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for QualificationAdmissionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Semantic(error) => Some(error),
            _ => None,
        }
    }
}

impl QualificationAdmissionProfileV1 {
    /// Validate durable profile structure and architecture-independent limits.
    pub fn validate(&self) -> Result<(), QualificationAdmissionError> {
        if self.schema_version != QUALIFICATION_ADMISSION_PROFILE_SCHEMA_V1 {
            return Err(QualificationAdmissionError::UnsupportedSchemaVersion {
                actual: self.schema_version,
            });
        }
        validate_protocol_identifier(&self.profile_id, "profile id", true)?;
        validate_protocol_identifier(&self.encoding_id, "encoding id", false)?;

        for (field, value) in self.runtime_limit_fields() {
            if value > PORTABLE_USIZE_MAX {
                return Err(QualificationAdmissionError::NonPortableRuntimeLimit {
                    field,
                    portable_max: PORTABLE_USIZE_MAX,
                    actual: value,
                });
            }
        }
        Ok(())
    }

    /// Convert fixed-width durable limits into the v0.2 in-memory profile.
    pub fn runtime_limits(&self) -> Result<QualificationLimits, QualificationAdmissionError> {
        self.validate()?;
        Ok(QualificationLimits {
            max_theorems: checked_runtime_usize(AdmissionLimitField::Theorems, self.max_theorems)?,
            max_prerequisites_per_theorem: checked_runtime_usize(
                AdmissionLimitField::PrerequisitesPerTheorem,
                self.max_prerequisites_per_theorem,
            )?,
            max_total_prerequisite_edges: checked_runtime_usize(
                AdmissionLimitField::TotalPrerequisiteEdges,
                self.max_total_prerequisite_edges,
            )?,
            max_facets: checked_runtime_usize(AdmissionLimitField::Facets, self.max_facets)?,
            max_required_theorems: checked_runtime_usize(
                AdmissionLimitField::RequiredTheorems,
                self.max_required_theorems,
            )?,
            max_accepted_statuses: checked_runtime_usize(
                AdmissionLimitField::AcceptedStatuses,
                self.max_accepted_statuses,
            )?,
            max_evidence_refs_per_facet: checked_runtime_usize(
                AdmissionLimitField::EvidenceRefsPerFacet,
                self.max_evidence_refs_per_facet,
            )?,
            max_dependency_commitments_per_facet: checked_runtime_usize(
                AdmissionLimitField::DependencyCommitmentsPerFacet,
                self.max_dependency_commitments_per_facet,
            )?,
            max_explanation_blockers: checked_runtime_usize(
                AdmissionLimitField::ExplanationBlockers,
                self.max_explanation_blockers,
            )?,
            max_explanation_path_theorems: checked_runtime_usize(
                AdmissionLimitField::ExplanationPathTheorems,
                self.max_explanation_path_theorems,
            )?,
            max_theorem_id_bytes: checked_runtime_usize(
                AdmissionLimitField::TheoremIdBytes,
                self.max_theorem_id_bytes,
            )?,
            max_profile_id_bytes: checked_runtime_usize(
                AdmissionLimitField::ProfileIdBytes,
                self.max_profile_id_bytes,
            )?,
            max_subject_commitment_bytes: checked_runtime_usize(
                AdmissionLimitField::SubjectCommitmentBytes,
                self.max_subject_commitment_bytes,
            )?,
            max_verifier_profile_bytes: checked_runtime_usize(
                AdmissionLimitField::VerifierProfileBytes,
                self.max_verifier_profile_bytes,
            )?,
            max_evidence_ref_bytes: checked_runtime_usize(
                AdmissionLimitField::EvidenceRefBytes,
                self.max_evidence_ref_bytes,
            )?,
            max_dependency_commitment_bytes: checked_runtime_usize(
                AdmissionLimitField::DependencyCommitmentBytes,
                self.max_dependency_commitment_bytes,
            )?,
            max_diagnostics_commitment_bytes: checked_runtime_usize(
                AdmissionLimitField::DiagnosticsCommitmentBytes,
                self.max_diagnostics_commitment_bytes,
            )?,
        })
    }

    fn runtime_limit_fields(&self) -> [(AdmissionLimitField, u64); 17] {
        [
            (AdmissionLimitField::Theorems, self.max_theorems),
            (
                AdmissionLimitField::PrerequisitesPerTheorem,
                self.max_prerequisites_per_theorem,
            ),
            (
                AdmissionLimitField::TotalPrerequisiteEdges,
                self.max_total_prerequisite_edges,
            ),
            (AdmissionLimitField::Facets, self.max_facets),
            (
                AdmissionLimitField::RequiredTheorems,
                self.max_required_theorems,
            ),
            (
                AdmissionLimitField::AcceptedStatuses,
                self.max_accepted_statuses,
            ),
            (
                AdmissionLimitField::EvidenceRefsPerFacet,
                self.max_evidence_refs_per_facet,
            ),
            (
                AdmissionLimitField::DependencyCommitmentsPerFacet,
                self.max_dependency_commitments_per_facet,
            ),
            (
                AdmissionLimitField::ExplanationBlockers,
                self.max_explanation_blockers,
            ),
            (
                AdmissionLimitField::ExplanationPathTheorems,
                self.max_explanation_path_theorems,
            ),
            (
                AdmissionLimitField::TheoremIdBytes,
                self.max_theorem_id_bytes,
            ),
            (
                AdmissionLimitField::ProfileIdBytes,
                self.max_profile_id_bytes,
            ),
            (
                AdmissionLimitField::SubjectCommitmentBytes,
                self.max_subject_commitment_bytes,
            ),
            (
                AdmissionLimitField::VerifierProfileBytes,
                self.max_verifier_profile_bytes,
            ),
            (
                AdmissionLimitField::EvidenceRefBytes,
                self.max_evidence_ref_bytes,
            ),
            (
                AdmissionLimitField::DependencyCommitmentBytes,
                self.max_dependency_commitment_bytes,
            ),
            (
                AdmissionLimitField::DiagnosticsCommitmentBytes,
                self.max_diagnostics_commitment_bytes,
            ),
        ]
    }
}

/// Canonical deterministic byte preimage for one validated durable profile.
///
/// Field order exactly follows [`QualificationAdmissionProfileV1`]. Numeric
/// values are fixed-width big-endian and text is u32-length-prefixed UTF-8.
/// No digest is computed here.
pub fn canonical_qualification_admission_profile_preimage_v1(
    profile: &QualificationAdmissionProfileV1,
) -> Result<Vec<u8>, QualificationAdmissionError> {
    profile.validate()?;

    let mut out = Vec::with_capacity(320);
    out.extend_from_slice(PROFILE_DOMAIN_SEPARATOR);
    push_u16(&mut out, profile.schema_version);
    push_text(&mut out, &profile.profile_id)?;
    push_text(&mut out, &profile.encoding_id)?;
    push_u64(&mut out, profile.max_encoded_bytes);

    for (_, value) in profile.runtime_limit_fields() {
        push_u64(&mut out, value);
    }

    push_u64(&mut out, profile.max_total_registry_identifier_bytes);
    push_u64(&mut out, profile.max_total_manifest_string_bytes);
    push_u64(&mut out, profile.max_total_manifest_evidence_refs);
    push_u64(&mut out, profile.max_total_manifest_dependency_commitments);
    Ok(out)
}

/// Encoded bytes that passed the exact profile's pre-deserialization gate.
///
/// The type is intentionally non-serializable. Parsing APIs may require this
/// wrapper rather than accepting arbitrary raw bytes directly.
#[derive(Debug, Clone)]
pub struct AdmittedQualificationBytes<'a> {
    bytes: &'a [u8],
    profile_id: String,
    encoding_id: String,
}

impl<'a> AdmittedQualificationBytes<'a> {
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    pub fn profile_id(&self) -> &str {
        &self.profile_id
    }

    pub fn encoding_id(&self) -> &str {
        &self.encoding_id
    }
}

/// Apply the exact encoding and byte-size gate before parsing.
pub fn admit_qualification_bytes<'a>(
    profile: &QualificationAdmissionProfileV1,
    claimed_encoding_id: &str,
    bytes: &'a [u8],
) -> Result<AdmittedQualificationBytes<'a>, QualificationAdmissionError> {
    profile.validate()?;

    let actual = usize_to_u64(bytes.len());
    if actual > profile.max_encoded_bytes {
        return Err(QualificationAdmissionError::EncodedPayloadTooLarge {
            limit: profile.max_encoded_bytes,
            actual,
        });
    }
    if claimed_encoding_id != profile.encoding_id {
        return Err(QualificationAdmissionError::EncodingMismatch);
    }

    Ok(AdmittedQualificationBytes {
        bytes,
        profile_id: profile.profile_id.clone(),
        encoding_id: profile.encoding_id.clone(),
    })
}

/// Hardened theorem registry bound to one durable admission profile.
///
/// Consumers cannot accidentally use the v0.2 registry while forgetting the
/// v0.3 aggregate budgets.
#[derive(Debug, Clone)]
pub struct ProfiledQualificationRegistryV1 {
    profile: QualificationAdmissionProfileV1,
    registry: HardenedTheoremRegistry,
}

impl ProfiledQualificationRegistryV1 {
    pub fn new(
        profile: QualificationAdmissionProfileV1,
        definitions: Vec<TheoremDefinition>,
    ) -> Result<Self, QualificationAdmissionError> {
        profile.validate()?;
        validate_registry_aggregate_budget(&profile, &definitions)?;
        let registry =
            HardenedTheoremRegistry::with_limits(definitions, profile.runtime_limits()?)?;
        Ok(Self { profile, registry })
    }

    pub fn profile(&self) -> &QualificationAdmissionProfileV1 {
        &self.profile
    }

    pub fn topological_order(&self) -> &[TheoremId] {
        self.registry.topological_order()
    }

    pub fn prerequisite_closure(
        &self,
        theorem_id: &TheoremId,
    ) -> Result<Vec<TheoremId>, QualificationAdmissionError> {
        Ok(self.registry.prerequisite_closure(theorem_id)?)
    }

    /// Apply this registry's exact pre-deserialization byte profile.
    pub fn admit_bytes<'a>(
        &self,
        claimed_encoding_id: &str,
        bytes: &'a [u8],
    ) -> Result<AdmittedQualificationBytes<'a>, QualificationAdmissionError> {
        admit_qualification_bytes(&self.profile, claimed_encoding_id, bytes)
    }

    pub fn validate_manifest(
        &self,
        manifest: &QualificationManifest,
    ) -> Result<(), QualificationAdmissionError> {
        self.registry.validate_manifest_shape(manifest)?;
        validate_manifest_aggregate_budget(&self.profile, manifest)?;
        self.registry.validate_manifest(manifest)?;
        Ok(())
    }

    pub fn evaluate(
        &self,
        manifest: &QualificationManifest,
        profile: &QualificationRequirementProfile,
    ) -> Result<RequirementEvaluation, QualificationAdmissionError> {
        self.validate_manifest(manifest)?;
        self.registry.validate_requirement_profile(profile)?;
        Ok(self.registry.evaluate(manifest, profile)?)
    }

    /// Explain blocked establishment without first requiring the complete
    /// establishment invariant to pass.
    pub fn explain_establishment(
        &self,
        manifest: &QualificationManifest,
        theorem_id: &TheoremId,
    ) -> Result<EstablishmentExplanation, QualificationAdmissionError> {
        self.registry.validate_manifest_shape(manifest)?;
        validate_manifest_aggregate_budget(&self.profile, manifest)?;
        Ok(self.registry.explain_establishment(manifest, theorem_id)?)
    }
}

fn validate_registry_aggregate_budget(
    profile: &QualificationAdmissionProfileV1,
    definitions: &[TheoremDefinition],
) -> Result<(), QualificationAdmissionError> {
    let mut total = 0u64;
    for definition in definitions {
        add_len(
            &mut total,
            definition.id.as_str().len(),
            AdmissionAggregateResource::RegistryIdentifierBytes,
        )?;
        for prerequisite in &definition.prerequisites {
            add_len(
                &mut total,
                prerequisite.as_str().len(),
                AdmissionAggregateResource::RegistryIdentifierBytes,
            )?;
        }
    }
    check_aggregate(
        AdmissionAggregateResource::RegistryIdentifierBytes,
        total,
        profile.max_total_registry_identifier_bytes,
    )
}

fn validate_manifest_aggregate_budget(
    profile: &QualificationAdmissionProfileV1,
    manifest: &QualificationManifest,
) -> Result<(), QualificationAdmissionError> {
    let mut string_bytes = 0u64;
    let mut evidence_refs = 0u64;
    let mut dependency_commitments = 0u64;

    add_len(
        &mut string_bytes,
        manifest.subject_commitment.len(),
        AdmissionAggregateResource::ManifestStringBytes,
    )?;

    for facet in &manifest.facets {
        add_len(
            &mut string_bytes,
            facet.theorem_id.as_str().len(),
            AdmissionAggregateResource::ManifestStringBytes,
        )?;
        add_len(
            &mut string_bytes,
            facet.subject_commitment.len(),
            AdmissionAggregateResource::ManifestStringBytes,
        )?;
        if let Some(verifier) = &facet.verifier_or_profile {
            add_len(
                &mut string_bytes,
                verifier.len(),
                AdmissionAggregateResource::ManifestStringBytes,
            )?;
        }

        evidence_refs = checked_add_u64(
            evidence_refs,
            usize_to_u64(facet.evidence_refs.len()),
            AdmissionAggregateResource::ManifestEvidenceRefs,
        )?;
        for reference in &facet.evidence_refs {
            add_len(
                &mut string_bytes,
                reference.len(),
                AdmissionAggregateResource::ManifestStringBytes,
            )?;
        }

        dependency_commitments = checked_add_u64(
            dependency_commitments,
            usize_to_u64(facet.dependency_commitments.len()),
            AdmissionAggregateResource::ManifestDependencyCommitments,
        )?;
        for commitment in &facet.dependency_commitments {
            add_len(
                &mut string_bytes,
                commitment.len(),
                AdmissionAggregateResource::ManifestStringBytes,
            )?;
        }

        if let Some(commitment) = &facet.diagnostics_commitment {
            add_len(
                &mut string_bytes,
                commitment.len(),
                AdmissionAggregateResource::ManifestStringBytes,
            )?;
        }
    }

    check_aggregate(
        AdmissionAggregateResource::ManifestStringBytes,
        string_bytes,
        profile.max_total_manifest_string_bytes,
    )?;
    check_aggregate(
        AdmissionAggregateResource::ManifestEvidenceRefs,
        evidence_refs,
        profile.max_total_manifest_evidence_refs,
    )?;
    check_aggregate(
        AdmissionAggregateResource::ManifestDependencyCommitments,
        dependency_commitments,
        profile.max_total_manifest_dependency_commitments,
    )
}

fn add_len(
    total: &mut u64,
    len: usize,
    resource: AdmissionAggregateResource,
) -> Result<(), QualificationAdmissionError> {
    *total = checked_add_u64(*total, usize_to_u64(len), resource)?;
    Ok(())
}

fn usize_to_u64(value: usize) -> u64 {
    u64::try_from(value).unwrap_or(u64::MAX)
}

fn checked_add_u64(
    left: u64,
    right: u64,
    resource: AdmissionAggregateResource,
) -> Result<u64, QualificationAdmissionError> {
    left.checked_add(right)
        .ok_or(QualificationAdmissionError::AggregateArithmeticOverflow { resource })
}

fn check_aggregate(
    resource: AdmissionAggregateResource,
    actual: u64,
    limit: u64,
) -> Result<(), QualificationAdmissionError> {
    if actual > limit {
        return Err(QualificationAdmissionError::AggregateBudgetExceeded {
            resource,
            limit,
            actual,
        });
    }
    Ok(())
}

fn checked_runtime_usize(
    field: AdmissionLimitField,
    value: u64,
) -> Result<usize, QualificationAdmissionError> {
    if value > PORTABLE_USIZE_MAX {
        return Err(QualificationAdmissionError::NonPortableRuntimeLimit {
            field,
            portable_max: PORTABLE_USIZE_MAX,
            actual: value,
        });
    }
    usize::try_from(value).map_err(|_| QualificationAdmissionError::NonPortableRuntimeLimit {
        field,
        portable_max: PORTABLE_USIZE_MAX,
        actual: value,
    })
}

fn validate_protocol_identifier(
    value: &str,
    field: &'static str,
    profile_id: bool,
) -> Result<(), QualificationAdmissionError> {
    if value.is_empty() {
        return Err(if profile_id {
            QualificationAdmissionError::EmptyProfileId
        } else {
            QualificationAdmissionError::EmptyEncodingId
        });
    }
    if value.len() > MAX_CANONICAL_IDENTIFIER_BYTES {
        return Err(QualificationAdmissionError::IdentifierTooLong {
            field,
            limit: MAX_CANONICAL_IDENTIFIER_BYTES,
            actual: value.len(),
        });
    }
    if !is_canonical_protocol_identifier(value) {
        return Err(if profile_id {
            QualificationAdmissionError::NonCanonicalProfileId
        } else {
            QualificationAdmissionError::NonCanonicalEncodingId
        });
    }
    Ok(())
}

fn is_canonical_protocol_identifier(value: &str) -> bool {
    let bytes = value.as_bytes();
    let Some(first) = bytes.first() else {
        return false;
    };
    let Some(last) = bytes.last() else {
        return false;
    };
    if !is_ascii_alphanumeric_lower(*first) || !is_ascii_alphanumeric_lower(*last) {
        return false;
    }
    bytes.iter().all(|byte| {
        is_ascii_alphanumeric_lower(*byte) || matches!(*byte, b'.' | b'_' | b':' | b'-')
    })
}

fn is_ascii_alphanumeric_lower(byte: u8) -> bool {
    byte.is_ascii_lowercase() || byte.is_ascii_digit()
}

fn push_text(out: &mut Vec<u8>, value: &str) -> Result<(), QualificationAdmissionError> {
    let length = u32::try_from(value.len())
        .map_err(|_| QualificationAdmissionError::CanonicalEncodingOverflow)?;
    out.extend_from_slice(&length.to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::qualification::{FacetStatus, QualificationFacet};
    use std::collections::BTreeSet;

    fn id(value: &str) -> TheoremId {
        TheoremId::from(value)
    }

    fn definition(value: &str, prerequisites: &[&str]) -> TheoremDefinition {
        TheoremDefinition::new(value, prerequisites.iter().map(|value| id(value)).collect())
    }

    fn facet(theorem: &str, subject: &str, status: FacetStatus) -> QualificationFacet {
        QualificationFacet {
            theorem_id: id(theorem),
            subject_commitment: subject.into(),
            status,
            verifier_or_profile: None,
            evidence_refs: Vec::new(),
            dependency_commitments: Vec::new(),
            diagnostics_commitment: None,
        }
    }

    fn definitions() -> Vec<TheoremDefinition> {
        vec![
            definition("q0.serialization-safety.v1", &[]),
            definition("q2.frame-identity.v1", &["q0.serialization-safety.v1"]),
        ]
    }

    #[test]
    fn default_profile_is_valid_and_runtime_convertible() {
        let profile = QualificationAdmissionProfileV1::default();
        profile.validate().unwrap();
        let runtime = profile.runtime_limits().unwrap();
        assert_eq!(runtime.max_theorems, 2_048);
        assert_eq!(runtime.max_total_prerequisite_edges, 65_536);
    }

    #[test]
    fn canonical_preimage_follows_declared_wire_order() {
        let profile = QualificationAdmissionProfileV1::default();
        let bytes = canonical_qualification_admission_profile_preimage_v1(&profile).unwrap();
        let mut offset = PROFILE_DOMAIN_SEPARATOR.len();

        assert_eq!(
            &bytes[offset..offset + 2],
            &profile.schema_version.to_be_bytes()
        );
        offset += 2;

        let profile_len =
            u32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
        offset += 4;
        assert_eq!(
            &bytes[offset..offset + profile_len],
            profile.profile_id.as_bytes()
        );
        offset += profile_len;

        let encoding_len =
            u32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
        offset += 4;
        assert_eq!(
            &bytes[offset..offset + encoding_len],
            profile.encoding_id.as_bytes()
        );
        offset += encoding_len;

        assert_eq!(
            &bytes[offset..offset + 8],
            &profile.max_encoded_bytes.to_be_bytes()
        );
    }

    #[test]
    fn canonical_preimage_is_field_sensitive_and_not_generic_json() {
        let left = QualificationAdmissionProfileV1::default();
        let right = QualificationAdmissionProfileV1 {
            max_facets: left.max_facets + 1,
            ..left.clone()
        };
        let left_bytes = canonical_qualification_admission_profile_preimage_v1(&left).unwrap();
        let right_bytes = canonical_qualification_admission_profile_preimage_v1(&right).unwrap();
        assert!(left_bytes.starts_with(PROFILE_DOMAIN_SEPARATOR));
        assert!(!left_bytes.starts_with(b"{"));
        assert_ne!(left_bytes, right_bytes);
    }

    #[test]
    fn runtime_limit_above_portable_u32_ceiling_fails_on_every_host() {
        let profile = QualificationAdmissionProfileV1 {
            max_theorems: u64::from(u32::MAX) + 1,
            ..Default::default()
        };
        assert!(matches!(
            profile.validate(),
            Err(QualificationAdmissionError::NonPortableRuntimeLimit {
                field: AdmissionLimitField::Theorems,
                ..
            })
        ));
    }

    #[test]
    fn wire_admission_accepts_exact_limit_and_rejects_one_over() {
        let profile = QualificationAdmissionProfileV1 {
            max_encoded_bytes: 4,
            ..Default::default()
        };
        let exact = [1u8, 2, 3, 4];
        let admitted = admit_qualification_bytes(&profile, &profile.encoding_id, &exact).unwrap();
        assert_eq!(admitted.as_bytes(), &exact);
        assert_eq!(admitted.len(), 4);
        assert_eq!(admitted.profile_id(), profile.profile_id);

        let too_large = [1u8, 2, 3, 4, 5];
        assert!(matches!(
            admit_qualification_bytes(&profile, &profile.encoding_id, &too_large),
            Err(QualificationAdmissionError::EncodedPayloadTooLarge {
                limit: 4,
                actual: 5,
            })
        ));
    }

    #[test]
    fn wire_size_rejects_before_encoding_mismatch() {
        let profile = QualificationAdmissionProfileV1 {
            max_encoded_bytes: 1,
            ..Default::default()
        };
        assert!(matches!(
            admit_qualification_bytes(&profile, "wrong-v1", b"too-large"),
            Err(QualificationAdmissionError::EncodedPayloadTooLarge { .. })
        ));
    }

    #[test]
    fn wire_admission_rejects_wrong_encoding() {
        let profile = QualificationAdmissionProfileV1::default();
        assert!(matches!(
            admit_qualification_bytes(&profile, "different-encoding-v1", b"payload"),
            Err(QualificationAdmissionError::EncodingMismatch)
        ));
    }

    #[test]
    fn registry_identifier_aggregate_can_fail_below_local_id_limits() {
        let profile = QualificationAdmissionProfileV1 {
            max_theorem_id_bytes: 64,
            max_total_registry_identifier_bytes: 3,
            ..Default::default()
        };
        let result = ProfiledQualificationRegistryV1::new(
            profile,
            vec![definition("aa", &[]), definition("bb", &[])],
        );
        assert!(matches!(
            result,
            Err(QualificationAdmissionError::AggregateBudgetExceeded {
                resource: AdmissionAggregateResource::RegistryIdentifierBytes,
                limit: 3,
                actual: 4,
            })
        ));
    }

    #[test]
    fn aggregate_registry_budget_counts_repeated_prerequisite_occurrences() {
        let profile = QualificationAdmissionProfileV1 {
            max_total_registry_identifier_bytes: 3,
            ..Default::default()
        };
        let result = ProfiledQualificationRegistryV1::new(
            profile,
            vec![definition("a", &[]), definition("bb", &["a"])],
        );
        assert!(matches!(
            result,
            Err(QualificationAdmissionError::AggregateBudgetExceeded {
                resource: AdmissionAggregateResource::RegistryIdentifierBytes,
                limit: 3,
                actual: 4,
            })
        ));
    }

    #[test]
    fn manifest_string_aggregate_can_fail_below_every_local_string_limit() {
        let profile = QualificationAdmissionProfileV1 {
            max_subject_commitment_bytes: 64,
            max_theorem_id_bytes: 64,
            max_total_manifest_string_bytes: 20,
            ..Default::default()
        };
        let registry = ProfiledQualificationRegistryV1::new(profile, definitions()).unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![facet(
                "q0.serialization-safety.v1",
                "subject",
                FacetStatus::Established,
            )],
        };
        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationAdmissionError::AggregateBudgetExceeded {
                resource: AdmissionAggregateResource::ManifestStringBytes,
                ..
            })
        ));
    }

    #[test]
    fn manifest_reference_aggregate_can_fail_below_per_facet_count_limit() {
        let profile = QualificationAdmissionProfileV1 {
            max_evidence_refs_per_facet: 4,
            max_total_manifest_evidence_refs: 1,
            ..Default::default()
        };
        let registry = ProfiledQualificationRegistryV1::new(profile, definitions()).unwrap();
        let mut q0 = facet(
            "q0.serialization-safety.v1",
            "subject",
            FacetStatus::Established,
        );
        q0.evidence_refs = vec!["e:1".into(), "e:2".into()];
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![q0],
        };
        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationAdmissionError::AggregateBudgetExceeded {
                resource: AdmissionAggregateResource::ManifestEvidenceRefs,
                limit: 1,
                actual: 2,
            })
        ));
    }

    #[test]
    fn manifest_dependency_aggregate_can_fail_below_per_facet_count_limit() {
        let profile = QualificationAdmissionProfileV1 {
            max_dependency_commitments_per_facet: 4,
            max_total_manifest_dependency_commitments: 1,
            ..Default::default()
        };
        let registry = ProfiledQualificationRegistryV1::new(profile, definitions()).unwrap();
        let mut q0 = facet(
            "q0.serialization-safety.v1",
            "subject",
            FacetStatus::Established,
        );
        q0.dependency_commitments = vec!["d:1".into(), "d:2".into()];
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![q0],
        };
        assert!(matches!(
            registry.validate_manifest(&manifest),
            Err(QualificationAdmissionError::AggregateBudgetExceeded {
                resource: AdmissionAggregateResource::ManifestDependencyCommitments,
                limit: 1,
                actual: 2,
            })
        ));
    }

    #[test]
    fn aggregate_arithmetic_overflow_fails_closed() {
        assert!(matches!(
            checked_add_u64(u64::MAX, 1, AdmissionAggregateResource::ManifestStringBytes,),
            Err(QualificationAdmissionError::AggregateArithmeticOverflow {
                resource: AdmissionAggregateResource::ManifestStringBytes,
            })
        ));
    }

    #[test]
    fn profiled_registry_preserves_hardened_evaluation_semantics() {
        let registry = ProfiledQualificationRegistryV1::new(
            QualificationAdmissionProfileV1::default(),
            definitions(),
        )
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![facet(
                "q0.serialization-safety.v1",
                "subject",
                FacetStatus::Established,
            )],
        };
        let requirement = QualificationRequirementProfile::established_only(
            "display-v1",
            vec![id("q0.serialization-safety.v1")],
        );
        assert!(
            registry
                .evaluate(&manifest, &requirement)
                .unwrap()
                .satisfied
        );
    }

    #[test]
    fn accepted_status_is_not_promoted_by_admission_layer() {
        let registry = ProfiledQualificationRegistryV1::new(
            QualificationAdmissionProfileV1::default(),
            definitions(),
        )
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![facet(
                "q0.serialization-safety.v1",
                "subject",
                FacetStatus::Indeterminate,
            )],
        };
        let requirement = QualificationRequirementProfile::established_only(
            "display-v1",
            vec![id("q0.serialization-safety.v1")],
        );
        let evaluation = registry.evaluate(&manifest, &requirement).unwrap();
        assert!(!evaluation.satisfied);
        assert_eq!(evaluation.unacceptable_facets.len(), 1);
    }

    #[test]
    fn profiled_explanation_diagnoses_missing_prerequisite() {
        let registry = ProfiledQualificationRegistryV1::new(
            QualificationAdmissionProfileV1::default(),
            definitions(),
        )
        .unwrap();
        let manifest = QualificationManifest {
            subject_commitment: "subject".into(),
            facets: vec![facet(
                "q2.frame-identity.v1",
                "subject",
                FacetStatus::Established,
            )],
        };
        let explanation = registry
            .explain_establishment(&manifest, &id("q2.frame-identity.v1"))
            .unwrap();
        assert!(!explanation.established);
        assert_eq!(explanation.blockers.len(), 1);
        assert_eq!(
            explanation.blockers[0].path,
            vec![id("q2.frame-identity.v1"), id("q0.serialization-safety.v1"),]
        );
    }

    #[test]
    fn registry_owned_wire_gate_binds_exact_profile_identity() {
        let registry = ProfiledQualificationRegistryV1::new(
            QualificationAdmissionProfileV1::default(),
            definitions(),
        )
        .unwrap();
        let admitted = registry
            .admit_bytes(&registry.profile().encoding_id, b"payload")
            .unwrap();
        assert_eq!(admitted.profile_id(), registry.profile().profile_id);
        assert_eq!(admitted.encoding_id(), registry.profile().encoding_id);
    }

    #[test]
    fn protocol_identifiers_reject_case_and_edge_separators() {
        let uppercase = QualificationAdmissionProfileV1 {
            profile_id: "Uppercase-v1".into(),
            ..Default::default()
        };
        assert!(matches!(
            uppercase.validate(),
            Err(QualificationAdmissionError::NonCanonicalProfileId)
        ));

        let bad_edge = QualificationAdmissionProfileV1 {
            profile_id: "-bad-edge".into(),
            ..Default::default()
        };
        assert!(matches!(
            bad_edge.validate(),
            Err(QualificationAdmissionError::NonCanonicalProfileId)
        ));
    }

    #[test]
    fn canonical_identifier_accepts_protocol_separator_vocabulary() {
        let values: BTreeSet<_> = [
            "mycelix-position:v1",
            "mycelix_position.v1",
            "mycelix-position-v1",
        ]
        .into_iter()
        .collect();
        for value in values {
            assert!(is_canonical_protocol_identifier(value));
        }
    }
}
