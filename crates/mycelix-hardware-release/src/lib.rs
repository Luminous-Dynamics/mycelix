// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-scoped hardware release manifests and admission profiles.
//!
//! This crate deliberately references external artifact/provenance/authorization
//! receipts by digest. It does not duplicate the cryptographic implementations
//! that produce or verify those receipts.

#![deny(unsafe_code)]

use mycelix_hardware_core::{
    ArtifactRole, ConstraintEvaluation, DesignRevision, DigestRef, DocumentationProfile,
    HardwareProject, LicenseRef, SemanticId,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const HARDWARE_RELEASE_SCHEMA: &str = "mycelix.hardware.release.v1";
pub const HARDWARE_RELEASE_PROFILE_SCHEMA: &str = "mycelix.hardware.release-profile.v1";
const MAX_ITEMS: usize = 16_384;
const MAX_TEXT: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalReceiptRef {
    pub system: String,
    pub schema: String,
    pub digest: DigestRef,
}

impl ExternalReceiptRef {
    pub fn validate(&self) -> Result<(), ReleaseModelError> {
        validate_text(&self.system, "receipt system")?;
        validate_text(&self.schema, "receipt schema")?;
        self.digest
            .validate()
            .map_err(|error| ReleaseModelError::InvalidDigest(error.to_string()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareReleaseManifest {
    pub schema_version: String,
    pub id: SemanticId,
    pub project_id: SemanticId,
    pub design_revision_id: SemanticId,
    pub release_label: String,
    /// Receipt/digest for an exact artifact inventory produced by the owning
    /// artifact subsystem (for example Symthaea's ReleaseArtifactSet).
    pub artifact_set: ExternalReceiptRef,
    /// Optional provenance statement/quorum reference. Presence is not proof
    /// that provenance has been verified.
    pub artifact_provenance: Option<ExternalReceiptRef>,
    /// Canonical digest of the design composition under the producer's named
    /// digest scheme. Verification of this digest is external to this crate.
    pub composition_digest: DigestRef,
    pub environment_digest: Option<DigestRef>,
    /// Exact design-artifact identities included in this release.
    pub included_artifact_ids: Vec<SemanticId>,
    /// Exact design-revision evidence identities carried into the release.
    pub evidence_ids: Vec<SemanticId>,
    /// Snapshot of the licenses declared for this released project state.
    pub license_snapshot: Vec<LicenseRef>,
    pub release_authority: SemanticId,
    /// Optional authorization receipt (for example a Xenia-bound action
    /// authorization). Presence alone does not establish validity.
    pub release_authorization: Option<ExternalReceiptRef>,
    pub issued_at_unix_s: u64,
}

impl HardwareReleaseManifest {
    pub fn validate(&self) -> Result<(), ReleaseModelError> {
        if self.schema_version != HARDWARE_RELEASE_SCHEMA {
            return Err(ReleaseModelError::UnsupportedSchema);
        }
        validate_text(&self.release_label, "release label")?;
        if self.issued_at_unix_s == 0 {
            return Err(ReleaseModelError::InvalidTimestamp);
        }
        self.artifact_set.validate()?;
        if let Some(provenance) = &self.artifact_provenance {
            provenance.validate()?;
        }
        self.composition_digest
            .validate()
            .map_err(|error| ReleaseModelError::InvalidDigest(error.to_string()))?;
        if let Some(environment) = &self.environment_digest {
            environment
                .validate()
                .map_err(|error| ReleaseModelError::InvalidDigest(error.to_string()))?;
        }
        if let Some(authorization) = &self.release_authorization {
            authorization.validate()?;
        }
        require_nonempty(&self.included_artifact_ids, "included artifacts")?;
        require_nonempty(&self.license_snapshot, "license snapshot")?;
        ensure_bounded(self.included_artifact_ids.len())?;
        ensure_bounded(self.evidence_ids.len())?;
        ensure_bounded(self.license_snapshot.len())?;
        ensure_unique_ids(&self.included_artifact_ids, "included artifact")?;
        ensure_unique_ids(&self.evidence_ids, "evidence")?;
        ensure_unique_licenses(&self.license_snapshot)?;
        for license in &self.license_snapshot {
            validate_text(&license.identifier, "license identifier")?;
            if let Some(reference) = &license.text_or_url {
                validate_text(reference, "license reference")?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareReleaseProfile {
    pub schema_version: String,
    pub id: SemanticId,
    pub required_artifact_roles: Vec<ArtifactRole>,
    pub required_documentation_profiles: Vec<DocumentationProfile>,
    pub require_artifact_provenance: bool,
    pub require_environment_verification: bool,
    pub require_release_authorization: bool,
    pub require_composition_verification: bool,
    pub require_blocking_claim_evaluation: bool,
}

impl HardwareReleaseProfile {
    pub fn validate(&self) -> Result<(), ReleaseModelError> {
        if self.schema_version != HARDWARE_RELEASE_PROFILE_SCHEMA {
            return Err(ReleaseModelError::UnsupportedProfileSchema);
        }
        ensure_bounded(self.required_artifact_roles.len())?;
        ensure_bounded(self.required_documentation_profiles.len())?;
        ensure_unique_by_equality(&self.required_artifact_roles, "required artifact role")?;
        ensure_unique_by_equality(
            &self.required_documentation_profiles,
            "required documentation profile",
        )?;
        for role in &self.required_artifact_roles {
            if let ArtifactRole::Other(name) = role {
                validate_text(name, "custom artifact role")?;
            }
        }
        for profile in &self.required_documentation_profiles {
            if let DocumentationProfile::Other(name) = profile {
                validate_text(name, "custom documentation profile")?;
            }
        }
        Ok(())
    }
}

/// Exact subject that an external verification result applies to.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationSubject {
    Release(SemanticId),
    DesignRevision(SemanticId),
    Digest(DigestRef),
}

impl VerificationSubject {
    fn validate(&self) -> Result<(), ReleaseModelError> {
        if let Self::Digest(digest) = self {
            digest
                .validate()
                .map_err(|error| ReleaseModelError::InvalidDigest(error.to_string()))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalVerification {
    pub subject: VerificationSubject,
    pub evaluation: ConstraintEvaluation,
    /// A Satisfied/Unsatisfied external verdict is not accepted without the
    /// exact receipt that produced that verdict. Unknown may legitimately have
    /// no receipt because the verifier has not run.
    pub receipt: Option<ExternalReceiptRef>,
}

impl ExternalVerification {
    pub fn validate(&self) -> Result<(), ReleaseModelError> {
        self.subject.validate()?;
        if let Some(receipt) = &self.receipt {
            receipt.validate()?;
        }
        if matches!(
            self.evaluation,
            ConstraintEvaluation::Satisfied | ConstraintEvaluation::Unsatisfied
        ) && self.receipt.is_none()
        {
            return Err(ReleaseModelError::VerdictMissingReceipt);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReleaseVerificationInputs {
    pub artifact_inventory: Option<ExternalVerification>,
    pub artifact_provenance: Option<ExternalVerification>,
    pub composition: Option<ExternalVerification>,
    pub environment: Option<ExternalVerification>,
    pub release_authorization: Option<ExternalVerification>,
    pub blocking_claims: Option<ExternalVerification>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReleaseViolation {
    InvalidManifest(String),
    InvalidProfile(String),
    InvalidProject(String),
    InvalidRevision(String),
    ProjectMismatch,
    RevisionProjectMismatch,
    LicenseSnapshotMismatch,
    UnknownArtifact(SemanticId),
    UnknownEvidence(SemanticId),
    MissingRequiredArtifactRole(ArtifactRole),
    MissingRequiredDocumentationProfile(DocumentationProfile),
    MissingRequiredProvenanceReference,
    MissingRequiredEnvironmentReference,
    MissingRequiredAuthorizationReference,
    ExternalVerificationInvalid {
        check: VerificationCheck,
        reason: String,
    },
    VerificationSubjectMismatch(VerificationCheck),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationCheck {
    ArtifactInventory,
    ArtifactProvenance,
    Composition,
    Environment,
    ReleaseAuthorization,
    BlockingClaims,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareReleaseVerification {
    pub semantic_integrity: ConstraintEvaluation,
    pub artifact_reference_status: ConstraintEvaluation,
    pub evidence_reference_status: ConstraintEvaluation,
    pub artifact_roles_status: ConstraintEvaluation,
    pub documentation_status: ConstraintEvaluation,
    pub artifact_inventory_status: ConstraintEvaluation,
    pub provenance_status: ConstraintEvaluation,
    pub composition_status: ConstraintEvaluation,
    pub environment_status: ConstraintEvaluation,
    pub authorization_status: ConstraintEvaluation,
    pub blocking_claim_status: ConstraintEvaluation,
    pub unresolved_refs: Vec<SemanticId>,
    pub violations: Vec<ReleaseViolation>,
}

impl HardwareReleaseVerification {
    /// Admission is deliberately profile-relative. It is not a safety,
    /// certification, physical-conformance, or regulatory verdict.
    pub fn is_admissible_under_profile(&self) -> bool {
        self.violations.is_empty()
            && [
                self.semantic_integrity,
                self.artifact_reference_status,
                self.evidence_reference_status,
                self.artifact_roles_status,
                self.documentation_status,
                self.artifact_inventory_status,
                self.provenance_status,
                self.composition_status,
                self.environment_status,
                self.authorization_status,
                self.blocking_claim_status,
            ]
            .into_iter()
            .all(|evaluation| {
                matches!(
                    evaluation,
                    ConstraintEvaluation::Satisfied | ConstraintEvaluation::NotApplicable
                )
            })
    }
}

pub fn verify_hardware_release(
    manifest: &HardwareReleaseManifest,
    profile: &HardwareReleaseProfile,
    project: &HardwareProject,
    revision: &DesignRevision,
    external: &ReleaseVerificationInputs,
) -> HardwareReleaseVerification {
    let mut report = HardwareReleaseVerification {
        semantic_integrity: ConstraintEvaluation::Satisfied,
        artifact_reference_status: ConstraintEvaluation::Satisfied,
        evidence_reference_status: ConstraintEvaluation::Satisfied,
        artifact_roles_status: if profile.required_artifact_roles.is_empty() {
            ConstraintEvaluation::NotApplicable
        } else {
            ConstraintEvaluation::Satisfied
        },
        documentation_status: if profile.required_documentation_profiles.is_empty() {
            ConstraintEvaluation::NotApplicable
        } else {
            ConstraintEvaluation::Satisfied
        },
        artifact_inventory_status: ConstraintEvaluation::Unknown,
        provenance_status: if profile.require_artifact_provenance {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        },
        composition_status: if profile.require_composition_verification {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        },
        environment_status: if profile.require_environment_verification {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        },
        authorization_status: if profile.require_release_authorization {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        },
        blocking_claim_status: if profile.require_blocking_claim_evaluation {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        },
        unresolved_refs: Vec::new(),
        violations: Vec::new(),
    };

    if let Err(error) = manifest.validate() {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::InvalidManifest(error.to_string()));
    }
    if let Err(error) = profile.validate() {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::InvalidProfile(error.to_string()));
    }
    if let Err(error) = project.validate() {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::InvalidProject(error.to_string()));
    }
    if let Err(error) = revision.validate() {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::InvalidRevision(error.to_string()));
    }

    if manifest.project_id != project.id {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report.violations.push(ReleaseViolation::ProjectMismatch);
    }
    if manifest.design_revision_id != revision.id || revision.project_id != project.id {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::RevisionProjectMismatch);
    }
    if manifest.license_snapshot != project.license_refs {
        report.semantic_integrity = ConstraintEvaluation::Unsatisfied;
        report
            .violations
            .push(ReleaseViolation::LicenseSnapshotMismatch);
    }

    resolve_artifacts(manifest, revision, &mut report);
    resolve_evidence(manifest, revision, &mut report);
    evaluate_required_artifact_roles(manifest, profile, revision, &mut report);
    evaluate_documentation_profiles(profile, revision, &mut report);

    report.artifact_inventory_status = evaluate_external_check(
        VerificationCheck::ArtifactInventory,
        VerificationSubject::Digest(manifest.artifact_set.digest.clone()),
        external.artifact_inventory.as_ref(),
        true,
        &mut report.violations,
    );

    if profile.require_artifact_provenance {
        match &manifest.artifact_provenance {
            Some(provenance) => {
                report.provenance_status = evaluate_external_check(
                    VerificationCheck::ArtifactProvenance,
                    VerificationSubject::Digest(provenance.digest.clone()),
                    external.artifact_provenance.as_ref(),
                    true,
                    &mut report.violations,
                );
            }
            None => {
                report.provenance_status = ConstraintEvaluation::Unsatisfied;
                report
                    .violations
                    .push(ReleaseViolation::MissingRequiredProvenanceReference);
            }
        }
    }

    if profile.require_composition_verification {
        report.composition_status = evaluate_external_check(
            VerificationCheck::Composition,
            VerificationSubject::Digest(manifest.composition_digest.clone()),
            external.composition.as_ref(),
            true,
            &mut report.violations,
        );
    }

    if profile.require_environment_verification {
        match &manifest.environment_digest {
            Some(environment) => {
                report.environment_status = evaluate_external_check(
                    VerificationCheck::Environment,
                    VerificationSubject::Digest(environment.clone()),
                    external.environment.as_ref(),
                    true,
                    &mut report.violations,
                );
            }
            None => {
                report.environment_status = ConstraintEvaluation::Unsatisfied;
                report
                    .violations
                    .push(ReleaseViolation::MissingRequiredEnvironmentReference);
            }
        }
    }

    if profile.require_release_authorization {
        if manifest.release_authorization.is_none() {
            report.authorization_status = ConstraintEvaluation::Unsatisfied;
            report
                .violations
                .push(ReleaseViolation::MissingRequiredAuthorizationReference);
        } else {
            report.authorization_status = evaluate_external_check(
                VerificationCheck::ReleaseAuthorization,
                VerificationSubject::Release(manifest.id.clone()),
                external.release_authorization.as_ref(),
                true,
                &mut report.violations,
            );
        }
    }

    if profile.require_blocking_claim_evaluation {
        report.blocking_claim_status = evaluate_external_check(
            VerificationCheck::BlockingClaims,
            VerificationSubject::DesignRevision(revision.id.clone()),
            external.blocking_claims.as_ref(),
            true,
            &mut report.violations,
        );
    }

    report
}

fn resolve_artifacts(
    manifest: &HardwareReleaseManifest,
    revision: &DesignRevision,
    report: &mut HardwareReleaseVerification,
) {
    for id in &manifest.included_artifact_ids {
        if !revision.artifacts.iter().any(|artifact| &artifact.id == id) {
            report.artifact_reference_status = ConstraintEvaluation::Unsatisfied;
            report.unresolved_refs.push(id.clone());
            report
                .violations
                .push(ReleaseViolation::UnknownArtifact(id.clone()));
        }
    }
}

fn resolve_evidence(
    manifest: &HardwareReleaseManifest,
    revision: &DesignRevision,
    report: &mut HardwareReleaseVerification,
) {
    for id in &manifest.evidence_ids {
        if !revision.evidence_refs.iter().any(|evidence| &evidence.id == id) {
            report.evidence_reference_status = ConstraintEvaluation::Unsatisfied;
            report.unresolved_refs.push(id.clone());
            report
                .violations
                .push(ReleaseViolation::UnknownEvidence(id.clone()));
        }
    }
}

fn evaluate_required_artifact_roles(
    manifest: &HardwareReleaseManifest,
    profile: &HardwareReleaseProfile,
    revision: &DesignRevision,
    report: &mut HardwareReleaseVerification,
) {
    if profile.required_artifact_roles.is_empty() {
        return;
    }
    for required in &profile.required_artifact_roles {
        let present = revision.artifacts.iter().any(|artifact| {
            &artifact.role == required && manifest.included_artifact_ids.contains(&artifact.id)
        });
        if !present {
            report.artifact_roles_status = ConstraintEvaluation::Unsatisfied;
            report
                .violations
                .push(ReleaseViolation::MissingRequiredArtifactRole(required.clone()));
        }
    }
}

fn evaluate_documentation_profiles(
    profile: &HardwareReleaseProfile,
    revision: &DesignRevision,
    report: &mut HardwareReleaseVerification,
) {
    if profile.required_documentation_profiles.is_empty() {
        return;
    }
    for required in &profile.required_documentation_profiles {
        match revision
            .documentation_profiles
            .iter()
            .find(|candidate| &candidate.profile == required)
        {
            None => {
                report.documentation_status = ConstraintEvaluation::Unsatisfied;
                report
                    .violations
                    .push(ReleaseViolation::MissingRequiredDocumentationProfile(
                        required.clone(),
                    ));
            }
            Some(candidate) => match candidate.assessment {
                ConstraintEvaluation::Satisfied => {}
                ConstraintEvaluation::Unsatisfied => {
                    report.documentation_status = ConstraintEvaluation::Unsatisfied;
                }
                ConstraintEvaluation::Unknown => {
                    if report.documentation_status == ConstraintEvaluation::Satisfied {
                        report.documentation_status = ConstraintEvaluation::Unknown;
                    }
                }
                ConstraintEvaluation::NotApplicable => {
                    report.documentation_status = ConstraintEvaluation::Unsatisfied;
                }
            },
        }
    }
}

fn evaluate_external_check(
    check: VerificationCheck,
    expected_subject: VerificationSubject,
    supplied: Option<&ExternalVerification>,
    required: bool,
    violations: &mut Vec<ReleaseViolation>,
) -> ConstraintEvaluation {
    let Some(verification) = supplied else {
        return if required {
            ConstraintEvaluation::Unknown
        } else {
            ConstraintEvaluation::NotApplicable
        };
    };
    if let Err(error) = verification.validate() {
        violations.push(ReleaseViolation::ExternalVerificationInvalid {
            check,
            reason: error.to_string(),
        });
        return ConstraintEvaluation::Unknown;
    }
    if verification.subject != expected_subject {
        violations.push(ReleaseViolation::VerificationSubjectMismatch(check));
        return ConstraintEvaluation::Unknown;
    }
    verification.evaluation
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReleaseModelError {
    UnsupportedSchema,
    UnsupportedProfileSchema,
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    EmptyCollection(&'static str),
    TooManyItems,
    Duplicate(&'static str),
    InvalidTimestamp,
    InvalidDigest(String),
    VerdictMissingReceipt,
}

impl fmt::Display for ReleaseModelError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSchema => write!(f, "unsupported hardware release schema"),
            Self::UnsupportedProfileSchema => write!(f, "unsupported hardware release profile schema"),
            Self::EmptyField(field) => write!(f, "{field} must not be empty"),
            Self::FieldTooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::EmptyCollection(field) => write!(f, "{field} must not be empty"),
            Self::TooManyItems => write!(f, "collection exceeds maximum item count"),
            Self::Duplicate(field) => write!(f, "duplicate {field}"),
            Self::InvalidTimestamp => write!(f, "release timestamp must be non-zero"),
            Self::InvalidDigest(reason) => write!(f, "invalid digest: {reason}"),
            Self::VerdictMissingReceipt => {
                write!(f, "external satisfied/unsatisfied verdict requires an exact receipt")
            }
        }
    }
}

impl std::error::Error for ReleaseModelError {}

fn validate_text(value: &str, field: &'static str) -> Result<(), ReleaseModelError> {
    if value.trim().is_empty() || value != value.trim() || value.chars().any(char::is_control) {
        return Err(ReleaseModelError::EmptyField(field));
    }
    if value.len() > MAX_TEXT {
        return Err(ReleaseModelError::FieldTooLong(field));
    }
    Ok(())
}

fn ensure_bounded(count: usize) -> Result<(), ReleaseModelError> {
    if count > MAX_ITEMS {
        Err(ReleaseModelError::TooManyItems)
    } else {
        Ok(())
    }
}

fn require_nonempty<T>(values: &[T], field: &'static str) -> Result<(), ReleaseModelError> {
    if values.is_empty() {
        Err(ReleaseModelError::EmptyCollection(field))
    } else {
        Ok(())
    }
}

fn ensure_unique_ids(values: &[SemanticId], field: &'static str) -> Result<(), ReleaseModelError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value) {
            return Err(ReleaseModelError::Duplicate(field));
        }
    }
    Ok(())
}

fn ensure_unique_licenses(values: &[LicenseRef]) -> Result<(), ReleaseModelError> {
    let mut seen = BTreeSet::new();
    for value in values {
        if !seen.insert(value.identifier.as_str()) {
            return Err(ReleaseModelError::Duplicate("license"));
        }
    }
    Ok(())
}

fn ensure_unique_by_equality<T: PartialEq>(
    values: &[T],
    field: &'static str,
) -> Result<(), ReleaseModelError> {
    for index in 0..values.len() {
        if values[..index].contains(&values[index]) {
            return Err(ReleaseModelError::Duplicate(field));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_hardware_core::{
        ArtifactRole, DesignArtifactRef, DesignComposition, DocumentationProfileRef,
        HardwareRequirement, RequirementCriticality, RevisionState, VerificationMethod,
        HARDWARE_SEMANTIC_SCHEMA,
    };

    fn id(value: &str) -> SemanticId {
        SemanticId::new(value).unwrap()
    }

    fn digest(byte: char) -> DigestRef {
        DigestRef::sha256(byte.to_string().repeat(64)).unwrap()
    }

    fn receipt(name: &str, byte: char) -> ExternalReceiptRef {
        ExternalReceiptRef {
            system: "symthaea-fabrication-kernel".into(),
            schema: name.into(),
            digest: digest(byte),
        }
    }

    fn project() -> HardwareProject {
        HardwareProject {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("project:microscope"),
            name: "Open Microscope".into(),
            description: None,
            maintainers: Vec::new(),
            license_refs: vec![LicenseRef {
                identifier: "CERN-OHL-S-2.0".into(),
                text_or_url: None,
            }],
        }
    }

    fn revision() -> DesignRevision {
        DesignRevision {
            schema_version: HARDWARE_SEMANTIC_SCHEMA.into(),
            id: id("revision:microscope:r1"),
            project_id: id("project:microscope"),
            revision_label: "r1".into(),
            state: RevisionState::Released,
            predecessors: Vec::new(),
            artifacts: vec![DesignArtifactRef {
                id: id("artifact:pcb"),
                role: ArtifactRole::PcbLayout,
                digest: digest('1'),
                byte_length: Some(42),
                media_type: Some("application/octet-stream".into()),
                logical_path: Some("pcb/main.kicad_pcb".into()),
            }],
            composition: DesignComposition { entries: Vec::new() },
            requirements: vec![HardwareRequirement {
                id: id("requirement:electrical"),
                statement: "electrical limits are checked".into(),
                criticality: RequirementCriticality::Blocking,
                verification_method: VerificationMethod::Test,
                source_ref: None,
            }],
            documentation_profiles: vec![DocumentationProfileRef {
                id: id("doc:okh"),
                profile: DocumentationProfile::OpenKnowHow,
                version: "2.4".into(),
                assessment: ConstraintEvaluation::Satisfied,
                report_ref: None,
            }],
            evidence_refs: Vec::new(),
        }
    }

    fn manifest() -> HardwareReleaseManifest {
        HardwareReleaseManifest {
            schema_version: HARDWARE_RELEASE_SCHEMA.into(),
            id: id("release:microscope:r1"),
            project_id: id("project:microscope"),
            design_revision_id: id("revision:microscope:r1"),
            release_label: "r1".into(),
            artifact_set: receipt("symthaea.fabrication.release-artifact-set.v1", 'a'),
            artifact_provenance: Some(receipt(
                "symthaea.fabrication.signed-artifact-provenance.v1",
                'b',
            )),
            composition_digest: digest('c'),
            environment_digest: Some(digest('d')),
            included_artifact_ids: vec![id("artifact:pcb")],
            evidence_ids: Vec::new(),
            license_snapshot: project().license_refs,
            release_authority: id("did:example:releaser"),
            release_authorization: Some(receipt("xenia.hardware-release-auth.v1", 'e')),
            issued_at_unix_s: 1_800_000_000,
        }
    }

    fn profile() -> HardwareReleaseProfile {
        HardwareReleaseProfile {
            schema_version: HARDWARE_RELEASE_PROFILE_SCHEMA.into(),
            id: id("profile:hardware:strict"),
            required_artifact_roles: vec![ArtifactRole::PcbLayout],
            required_documentation_profiles: vec![DocumentationProfile::OpenKnowHow],
            require_artifact_provenance: true,
            require_environment_verification: true,
            require_release_authorization: true,
            require_composition_verification: true,
            require_blocking_claim_evaluation: true,
        }
    }

    fn verified(subject: VerificationSubject, byte: char) -> ExternalVerification {
        ExternalVerification {
            subject,
            evaluation: ConstraintEvaluation::Satisfied,
            receipt: Some(receipt("test.verification.v1", byte)),
        }
    }

    fn fully_verified_inputs(manifest: &HardwareReleaseManifest) -> ReleaseVerificationInputs {
        ReleaseVerificationInputs {
            artifact_inventory: Some(verified(
                VerificationSubject::Digest(manifest.artifact_set.digest.clone()),
                '1',
            )),
            artifact_provenance: Some(verified(
                VerificationSubject::Digest(
                    manifest.artifact_provenance.as_ref().unwrap().digest.clone(),
                ),
                '2',
            )),
            composition: Some(verified(
                VerificationSubject::Digest(manifest.composition_digest.clone()),
                '3',
            )),
            environment: Some(verified(
                VerificationSubject::Digest(manifest.environment_digest.clone().unwrap()),
                '4',
            )),
            release_authorization: Some(verified(
                VerificationSubject::Release(manifest.id.clone()),
                '5',
            )),
            blocking_claims: Some(verified(
                VerificationSubject::DesignRevision(manifest.design_revision_id.clone()),
                '6',
            )),
        }
    }

    #[test]
    fn fully_scoped_release_can_be_admitted_under_profile() {
        let manifest = manifest();
        let report = verify_hardware_release(
            &manifest,
            &profile(),
            &project(),
            &revision(),
            &fully_verified_inputs(&manifest),
        );
        assert!(report.is_admissible_under_profile());
    }

    #[test]
    fn artifact_inventory_unknown_blocks_admission() {
        let manifest = manifest();
        let mut inputs = fully_verified_inputs(&manifest);
        inputs.artifact_inventory = None;
        let report = verify_hardware_release(
            &manifest,
            &profile(),
            &project(),
            &revision(),
            &inputs,
        );
        assert_eq!(
            report.artifact_inventory_status,
            ConstraintEvaluation::Unknown
        );
        assert!(!report.is_admissible_under_profile());
    }

    #[test]
    fn valid_receipt_for_wrong_subject_is_rejected() {
        let manifest = manifest();
        let mut inputs = fully_verified_inputs(&manifest);
        inputs.artifact_inventory = Some(verified(
            VerificationSubject::Digest(digest('f')),
            '7',
        ));
        let report = verify_hardware_release(
            &manifest,
            &profile(),
            &project(),
            &revision(),
            &inputs,
        );
        assert_eq!(
            report.artifact_inventory_status,
            ConstraintEvaluation::Unknown
        );
        assert!(report.violations.iter().any(|violation| matches!(
            violation,
            ReleaseViolation::VerificationSubjectMismatch(
                VerificationCheck::ArtifactInventory
            )
        )));
    }

    #[test]
    fn satisfied_external_verdict_without_receipt_is_not_accepted() {
        let manifest = manifest();
        let mut inputs = fully_verified_inputs(&manifest);
        inputs.composition = Some(ExternalVerification {
            subject: VerificationSubject::Digest(manifest.composition_digest.clone()),
            evaluation: ConstraintEvaluation::Satisfied,
            receipt: None,
        });
        let report = verify_hardware_release(
            &manifest,
            &profile(),
            &project(),
            &revision(),
            &inputs,
        );
        assert_eq!(report.composition_status, ConstraintEvaluation::Unknown);
        assert!(!report.is_admissible_under_profile());
    }

    #[test]
    fn missing_required_role_fails_profile() {
        let manifest = manifest();
        let mut profile = profile();
        profile.required_artifact_roles.push(ArtifactRole::Schematic);
        let report = verify_hardware_release(
            &manifest,
            &profile,
            &project(),
            &revision(),
            &fully_verified_inputs(&manifest),
        );
        assert_eq!(report.artifact_roles_status, ConstraintEvaluation::Unsatisfied);
        assert!(!report.is_admissible_under_profile());
    }

    #[test]
    fn missing_provenance_reference_fails_when_profile_requires_it() {
        let mut manifest = manifest();
        manifest.artifact_provenance = None;
        let report = verify_hardware_release(
            &manifest,
            &profile(),
            &project(),
            &revision(),
            &ReleaseVerificationInputs::default(),
        );
        assert_eq!(report.provenance_status, ConstraintEvaluation::Unsatisfied);
        assert!(report.violations.iter().any(|violation| matches!(
            violation,
            ReleaseViolation::MissingRequiredProvenanceReference
        )));
    }

    #[test]
    fn release_report_has_no_physical_conformance_shortcut() {
        let report = HardwareReleaseVerification {
            semantic_integrity: ConstraintEvaluation::Satisfied,
            artifact_reference_status: ConstraintEvaluation::Satisfied,
            evidence_reference_status: ConstraintEvaluation::Satisfied,
            artifact_roles_status: ConstraintEvaluation::NotApplicable,
            documentation_status: ConstraintEvaluation::NotApplicable,
            artifact_inventory_status: ConstraintEvaluation::Unknown,
            provenance_status: ConstraintEvaluation::NotApplicable,
            composition_status: ConstraintEvaluation::NotApplicable,
            environment_status: ConstraintEvaluation::NotApplicable,
            authorization_status: ConstraintEvaluation::NotApplicable,
            blocking_claim_status: ConstraintEvaluation::NotApplicable,
            unresolved_refs: Vec::new(),
            violations: Vec::new(),
        };
        assert!(!report.is_admissible_under_profile());
    }
}
