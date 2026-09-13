//! Offline public-election evidence-package and verifier contracts.
//!
//! This crate does not implement ballot, ZK, Merkle, signature, or audit
//! cryptography. It defines the portable package boundary and independent verifier
//! receipt/quorum structure that concrete verifier implementations must satisfy.

use std::collections::BTreeSet;

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use serde::{Deserialize, Serialize};

pub const OFFLINE_VERIFIER_PROFILE_ID: &str = "mycelix-public-election-offline-verifier-v1";
pub const EVIDENCE_PACKAGE_PROFILE_ID: &str = "mycelix-public-election-evidence-package-v1";

pub const MAX_ARTIFACTS: usize = 100_000;
pub const MAX_CANONICAL_PATH_BYTES: usize = 1024;
pub const MAX_INTEROPERABILITY_PROFILES: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EvidenceArtifactKind {
    ElectionConstitution,
    TransparencyCheckpointChain,
    WitnessAttestations,
    AnonymousAuthorityCensus,
    TallyEvidence,
    PhysicalAuditEvidence,
    ChallengeLedger,
    CertificationPolicy,
    InteroperabilityExport,
    VerifierMetadata,
}

pub const REQUIRED_PACKAGE_ARTIFACT_KINDS: [EvidenceArtifactKind; 8] = [
    EvidenceArtifactKind::ElectionConstitution,
    EvidenceArtifactKind::TransparencyCheckpointChain,
    EvidenceArtifactKind::WitnessAttestations,
    EvidenceArtifactKind::AnonymousAuthorityCensus,
    EvidenceArtifactKind::TallyEvidence,
    EvidenceArtifactKind::PhysicalAuditEvidence,
    EvidenceArtifactKind::ChallengeLedger,
    EvidenceArtifactKind::CertificationPolicy,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ArtifactDisclosureClass {
    PublicVerificationEvidence,
    NonPublicOrSecret,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceArtifactRefV1 {
    pub kind: EvidenceArtifactKind,
    pub canonical_path: String,
    pub content_digest: Digest32,
    pub byte_length: u64,
    pub disclosure: ArtifactDisclosureClass,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArtifactRefViolation {
    EmptyPath,
    PathTooLong,
    AbsolutePathForbidden,
    BackslashForbidden,
    NonCanonicalPathComponent,
    ControlCharacterInPath,
    ZeroContentDigest,
    EmptyArtifact,
    NonPublicArtifactForbidden,
}

pub fn validate_evidence_artifact_ref(
    artifact: &EvidenceArtifactRefV1,
) -> Result<(), ArtifactRefViolation> {
    let path = artifact.canonical_path.as_str();
    if path.is_empty() {
        return Err(ArtifactRefViolation::EmptyPath);
    }
    if path.len() > MAX_CANONICAL_PATH_BYTES {
        return Err(ArtifactRefViolation::PathTooLong);
    }
    if path.starts_with('/') || path.ends_with('/') {
        return Err(ArtifactRefViolation::AbsolutePathForbidden);
    }
    if path.contains('\\') {
        return Err(ArtifactRefViolation::BackslashForbidden);
    }
    if path.chars().any(char::is_control) {
        return Err(ArtifactRefViolation::ControlCharacterInPath);
    }
    if path
        .split('/')
        .any(|component| component.is_empty() || component == "." || component == "..")
    {
        return Err(ArtifactRefViolation::NonCanonicalPathComponent);
    }
    if artifact.content_digest == [0_u8; 32] {
        return Err(ArtifactRefViolation::ZeroContentDigest);
    }
    if artifact.byte_length == 0 {
        return Err(ArtifactRefViolation::EmptyArtifact);
    }
    if artifact.disclosure != ArtifactDisclosureClass::PublicVerificationEvidence {
        return Err(ArtifactRefViolation::NonPublicArtifactForbidden);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct InteroperabilityProfileRefV1 {
    pub profile_id: String,
    pub schema_or_specification_digest: Digest32,
    pub exported_artifact_digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ElectionEvidencePackageManifestV1 {
    pub public_election_profile_id: String,
    pub evidence_package_profile_id: String,
    pub election_constitution_digest: Digest32,
    pub package_canonicalization_profile_digest: Digest32,
    pub artifact_index_digest: Digest32,
    pub package_root_digest: Digest32,
    pub final_transparency_checkpoint_digest: Digest32,
    pub artifacts: Vec<EvidenceArtifactRefV1>,
    pub interoperability_profiles: Vec<InteroperabilityProfileRefV1>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvidencePackageViolation {
    WrongPublicElectionProfile,
    WrongEvidencePackageProfile,
    ZeroElectionConstitutionDigest,
    ZeroCanonicalizationProfileDigest,
    ZeroArtifactIndexDigest,
    ZeroPackageRootDigest,
    ZeroFinalCheckpointDigest,
    EmptyArtifactSet,
    TooManyArtifacts,
    Artifact {
        index: usize,
        violation: ArtifactRefViolation,
    },
    DuplicateCanonicalPath,
    DuplicateRequiredArtifactKind(EvidenceArtifactKind),
    MissingRequiredArtifactKind(EvidenceArtifactKind),
    TooManyInteroperabilityProfiles,
    EmptyInteroperabilityProfileId,
    DuplicateInteroperabilityProfileId,
    ZeroInteroperabilitySchemaDigest,
    ZeroInteroperabilityArtifactDigest,
}

pub fn validate_evidence_package_manifest(
    manifest: &ElectionEvidencePackageManifestV1,
) -> Result<(), EvidencePackageViolation> {
    let zero = [0_u8; 32];
    if manifest.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(EvidencePackageViolation::WrongPublicElectionProfile);
    }
    if manifest.evidence_package_profile_id != EVIDENCE_PACKAGE_PROFILE_ID {
        return Err(EvidencePackageViolation::WrongEvidencePackageProfile);
    }
    if manifest.election_constitution_digest == zero {
        return Err(EvidencePackageViolation::ZeroElectionConstitutionDigest);
    }
    if manifest.package_canonicalization_profile_digest == zero {
        return Err(EvidencePackageViolation::ZeroCanonicalizationProfileDigest);
    }
    if manifest.artifact_index_digest == zero {
        return Err(EvidencePackageViolation::ZeroArtifactIndexDigest);
    }
    if manifest.package_root_digest == zero {
        return Err(EvidencePackageViolation::ZeroPackageRootDigest);
    }
    if manifest.final_transparency_checkpoint_digest == zero {
        return Err(EvidencePackageViolation::ZeroFinalCheckpointDigest);
    }
    if manifest.artifacts.is_empty() {
        return Err(EvidencePackageViolation::EmptyArtifactSet);
    }
    if manifest.artifacts.len() > MAX_ARTIFACTS {
        return Err(EvidencePackageViolation::TooManyArtifacts);
    }

    let mut paths = BTreeSet::new();
    let mut required_kinds_seen = BTreeSet::new();
    for (index, artifact) in manifest.artifacts.iter().enumerate() {
        validate_evidence_artifact_ref(artifact)
            .map_err(|violation| EvidencePackageViolation::Artifact { index, violation })?;
        if !paths.insert(artifact.canonical_path.as_str()) {
            return Err(EvidencePackageViolation::DuplicateCanonicalPath);
        }
        if REQUIRED_PACKAGE_ARTIFACT_KINDS.contains(&artifact.kind)
            && !required_kinds_seen.insert(artifact.kind)
        {
            return Err(EvidencePackageViolation::DuplicateRequiredArtifactKind(
                artifact.kind,
            ));
        }
    }

    for kind in REQUIRED_PACKAGE_ARTIFACT_KINDS {
        if !required_kinds_seen.contains(&kind) {
            return Err(EvidencePackageViolation::MissingRequiredArtifactKind(kind));
        }
    }

    if manifest.interoperability_profiles.len() > MAX_INTEROPERABILITY_PROFILES {
        return Err(EvidencePackageViolation::TooManyInteroperabilityProfiles);
    }
    let mut profile_ids = BTreeSet::new();
    for profile in &manifest.interoperability_profiles {
        if profile.profile_id.trim().is_empty() {
            return Err(EvidencePackageViolation::EmptyInteroperabilityProfileId);
        }
        if !profile_ids.insert(profile.profile_id.as_str()) {
            return Err(EvidencePackageViolation::DuplicateInteroperabilityProfileId);
        }
        if profile.schema_or_specification_digest == zero {
            return Err(EvidencePackageViolation::ZeroInteroperabilitySchemaDigest);
        }
        if profile.exported_artifact_digest == zero {
            return Err(EvidencePackageViolation::ZeroInteroperabilityArtifactDigest);
        }
    }

    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CapabilityRule {
    Forbidden,
    Permitted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfflineVerifierExecutionPolicyV1 {
    pub network_access: CapabilityRule,
    pub wall_clock_dependency: CapabilityRule,
    pub nondeterministic_randomness: CapabilityRule,
    pub external_process_execution: CapabilityRule,
    pub mutable_global_state: CapabilityRule,
    pub holochain_runtime: CapabilityRule,
    pub symthaea_runtime: CapabilityRule,
}

impl Default for OfflineVerifierExecutionPolicyV1 {
    fn default() -> Self {
        Self {
            network_access: CapabilityRule::Forbidden,
            wall_clock_dependency: CapabilityRule::Forbidden,
            nondeterministic_randomness: CapabilityRule::Forbidden,
            external_process_execution: CapabilityRule::Forbidden,
            mutable_global_state: CapabilityRule::Forbidden,
            holochain_runtime: CapabilityRule::Forbidden,
            symthaea_runtime: CapabilityRule::Forbidden,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExecutionPolicyViolation {
    NetworkAccessMustBeForbidden,
    WallClockDependencyMustBeForbidden,
    NondeterministicRandomnessMustBeForbidden,
    ExternalProcessExecutionMustBeForbidden,
    MutableGlobalStateMustBeForbidden,
    HolochainRuntimeMustBeForbidden,
    SymthaeaRuntimeMustBeForbidden,
}

pub fn validate_offline_execution_policy(
    policy: &OfflineVerifierExecutionPolicyV1,
) -> Result<(), ExecutionPolicyViolation> {
    if policy.network_access != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::NetworkAccessMustBeForbidden);
    }
    if policy.wall_clock_dependency != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::WallClockDependencyMustBeForbidden);
    }
    if policy.nondeterministic_randomness != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::NondeterministicRandomnessMustBeForbidden);
    }
    if policy.external_process_execution != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::ExternalProcessExecutionMustBeForbidden);
    }
    if policy.mutable_global_state != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::MutableGlobalStateMustBeForbidden);
    }
    if policy.holochain_runtime != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::HolochainRuntimeMustBeForbidden);
    }
    if policy.symthaea_runtime != CapabilityRule::Forbidden {
        return Err(ExecutionPolicyViolation::SymthaeaRuntimeMustBeForbidden);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum VerificationStageId {
    PackageIntegrity,
    ElectionConstitution,
    TransparencyLineage,
    WitnessQuorum,
    AnonymousAuthorityCensus,
    TallyEvidence,
    PhysicalAudit,
    ChallengeLedger,
    CertificationEvidence,
}

pub const REQUIRED_VERIFICATION_STAGES: [VerificationStageId; 9] = [
    VerificationStageId::PackageIntegrity,
    VerificationStageId::ElectionConstitution,
    VerificationStageId::TransparencyLineage,
    VerificationStageId::WitnessQuorum,
    VerificationStageId::AnonymousAuthorityCensus,
    VerificationStageId::TallyEvidence,
    VerificationStageId::PhysicalAudit,
    VerificationStageId::ChallengeLedger,
    VerificationStageId::CertificationEvidence,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationStageDisposition {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerificationStageReceiptV1 {
    pub stage: VerificationStageId,
    pub package_root_digest: Digest32,
    pub subject_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub disposition: VerificationStageDisposition,
    pub finding_digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierRunReceiptV1 {
    pub offline_verifier_profile_id: String,
    pub package_root_digest: Digest32,
    pub verifier_implementation_id: String,
    pub verifier_lineage_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub source_digest: Digest32,
    pub build_provenance_digest: Digest32,
    pub execution_policy_digest: Digest32,
    pub stages: Vec<VerificationStageReceiptV1>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierRunDisposition {
    AllRequiredStagesPass,
    AtLeastOneStageFailed,
    AtLeastOneStageIndeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VerifierRunViolation {
    WrongVerifierProfile,
    ZeroPackageRootDigest,
    EmptyImplementationId,
    ZeroVerifierLineageDigest,
    ZeroVerifierReleaseDigest,
    ZeroSourceDigest,
    ZeroBuildProvenanceDigest,
    ZeroExecutionPolicyDigest,
    DuplicateStage(VerificationStageId),
    MissingStage(VerificationStageId),
    StagePackageMismatch,
    ZeroStageSubjectDigest,
    StageReleaseMismatch,
    ZeroFindingDigest,
}

pub fn classify_verifier_run(
    receipt: &VerifierRunReceiptV1,
) -> Result<VerifierRunDisposition, VerifierRunViolation> {
    let zero = [0_u8; 32];
    if receipt.offline_verifier_profile_id != OFFLINE_VERIFIER_PROFILE_ID {
        return Err(VerifierRunViolation::WrongVerifierProfile);
    }
    if receipt.package_root_digest == zero {
        return Err(VerifierRunViolation::ZeroPackageRootDigest);
    }
    if receipt.verifier_implementation_id.trim().is_empty() {
        return Err(VerifierRunViolation::EmptyImplementationId);
    }
    if receipt.verifier_lineage_digest == zero {
        return Err(VerifierRunViolation::ZeroVerifierLineageDigest);
    }
    if receipt.verifier_release_digest == zero {
        return Err(VerifierRunViolation::ZeroVerifierReleaseDigest);
    }
    if receipt.source_digest == zero {
        return Err(VerifierRunViolation::ZeroSourceDigest);
    }
    if receipt.build_provenance_digest == zero {
        return Err(VerifierRunViolation::ZeroBuildProvenanceDigest);
    }
    if receipt.execution_policy_digest == zero {
        return Err(VerifierRunViolation::ZeroExecutionPolicyDigest);
    }

    let mut stages_seen = BTreeSet::new();
    let mut failed = false;
    let mut indeterminate = false;
    for stage in &receipt.stages {
        if !stages_seen.insert(stage.stage) {
            return Err(VerifierRunViolation::DuplicateStage(stage.stage));
        }
        if stage.package_root_digest != receipt.package_root_digest {
            return Err(VerifierRunViolation::StagePackageMismatch);
        }
        if stage.subject_digest == zero {
            return Err(VerifierRunViolation::ZeroStageSubjectDigest);
        }
        if stage.verifier_release_digest != receipt.verifier_release_digest {
            return Err(VerifierRunViolation::StageReleaseMismatch);
        }
        if stage.finding_digest == zero {
            return Err(VerifierRunViolation::ZeroFindingDigest);
        }
        match stage.disposition {
            VerificationStageDisposition::Pass => {}
            VerificationStageDisposition::Fail => failed = true,
            VerificationStageDisposition::Indeterminate => indeterminate = true,
        }
    }

    for required in REQUIRED_VERIFICATION_STAGES {
        if !stages_seen.contains(&required) {
            return Err(VerifierRunViolation::MissingStage(required));
        }
    }

    if failed {
        Ok(VerifierRunDisposition::AtLeastOneStageFailed)
    } else if indeterminate {
        Ok(VerifierRunDisposition::AtLeastOneStageIndeterminate)
    } else {
        Ok(VerifierRunDisposition::AllRequiredStagesPass)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierAgreementAttestationV1 {
    pub package_root_digest: Digest32,
    pub verifier_lineage_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub builder_control_domain_digest: Digest32,
    pub run_receipt_digest: Digest32,
    pub run_disposition: VerificationStageDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifierAgreementPolicyV1 {
    pub minimum_total_verifiers: u16,
    pub minimum_distinct_implementation_lineages: u16,
    pub minimum_distinct_builder_control_domains: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierAgreementViolation {
    ZeroMinimumVerifiers,
    ZeroMinimumImplementationLineages,
    ZeroMinimumBuilderControlDomains,
    MinimumLineagesExceedTotalVerifiers,
    MinimumBuilderDomainsExceedTotalVerifiers,
    NoAttestations,
    PackageRootMismatch,
    ZeroVerifierLineageDigest,
    ZeroVerifierReleaseDigest,
    ZeroBuilderControlDomainDigest,
    ZeroRunReceiptDigest,
    NonPassingVerifier,
    DuplicateVerifierRelease,
    InsufficientTotalVerifiers,
    InsufficientImplementationLineages,
    InsufficientBuilderControlDomains,
}

pub fn validate_independent_verifier_agreement(
    package_root_digest: Digest32,
    policy: &VerifierAgreementPolicyV1,
    attestations: &[VerifierAgreementAttestationV1],
) -> Result<(), VerifierAgreementViolation> {
    if policy.minimum_total_verifiers == 0 {
        return Err(VerifierAgreementViolation::ZeroMinimumVerifiers);
    }
    if policy.minimum_distinct_implementation_lineages == 0 {
        return Err(VerifierAgreementViolation::ZeroMinimumImplementationLineages);
    }
    if policy.minimum_distinct_builder_control_domains == 0 {
        return Err(VerifierAgreementViolation::ZeroMinimumBuilderControlDomains);
    }
    if policy.minimum_distinct_implementation_lineages > policy.minimum_total_verifiers {
        return Err(VerifierAgreementViolation::MinimumLineagesExceedTotalVerifiers);
    }
    if policy.minimum_distinct_builder_control_domains > policy.minimum_total_verifiers {
        return Err(VerifierAgreementViolation::MinimumBuilderDomainsExceedTotalVerifiers);
    }
    if attestations.is_empty() {
        return Err(VerifierAgreementViolation::NoAttestations);
    }

    let zero = [0_u8; 32];
    let mut releases = BTreeSet::new();
    let mut lineages = BTreeSet::new();
    let mut builders = BTreeSet::new();
    for attestation in attestations {
        if attestation.package_root_digest != package_root_digest {
            return Err(VerifierAgreementViolation::PackageRootMismatch);
        }
        if attestation.verifier_lineage_digest == zero {
            return Err(VerifierAgreementViolation::ZeroVerifierLineageDigest);
        }
        if attestation.verifier_release_digest == zero {
            return Err(VerifierAgreementViolation::ZeroVerifierReleaseDigest);
        }
        if attestation.builder_control_domain_digest == zero {
            return Err(VerifierAgreementViolation::ZeroBuilderControlDomainDigest);
        }
        if attestation.run_receipt_digest == zero {
            return Err(VerifierAgreementViolation::ZeroRunReceiptDigest);
        }
        if attestation.run_disposition != VerificationStageDisposition::Pass {
            return Err(VerifierAgreementViolation::NonPassingVerifier);
        }
        if !releases.insert(attestation.verifier_release_digest) {
            return Err(VerifierAgreementViolation::DuplicateVerifierRelease);
        }
        lineages.insert(attestation.verifier_lineage_digest);
        builders.insert(attestation.builder_control_domain_digest);
    }

    if releases.len() < usize::from(policy.minimum_total_verifiers) {
        return Err(VerifierAgreementViolation::InsufficientTotalVerifiers);
    }
    if lineages.len() < usize::from(policy.minimum_distinct_implementation_lineages) {
        return Err(VerifierAgreementViolation::InsufficientImplementationLineages);
    }
    if builders.len() < usize::from(policy.minimum_distinct_builder_control_domains) {
        return Err(VerifierAgreementViolation::InsufficientBuilderControlDomains);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn artifact(kind: EvidenceArtifactKind, path: &str, byte: u8) -> EvidenceArtifactRefV1 {
        EvidenceArtifactRefV1 {
            kind,
            canonical_path: path.to_owned(),
            content_digest: digest(byte),
            byte_length: 10,
            disclosure: ArtifactDisclosureClass::PublicVerificationEvidence,
        }
    }

    fn valid_manifest() -> ElectionEvidencePackageManifestV1 {
        let artifacts = vec![
            artifact(
                EvidenceArtifactKind::ElectionConstitution,
                "election/constitution.bin",
                10,
            ),
            artifact(
                EvidenceArtifactKind::TransparencyCheckpointChain,
                "transparency/checkpoints.bin",
                11,
            ),
            artifact(
                EvidenceArtifactKind::WitnessAttestations,
                "transparency/witnesses.bin",
                12,
            ),
            artifact(
                EvidenceArtifactKind::AnonymousAuthorityCensus,
                "authority/census.bin",
                13,
            ),
            artifact(EvidenceArtifactKind::TallyEvidence, "tally/evidence.bin", 14),
            artifact(
                EvidenceArtifactKind::PhysicalAuditEvidence,
                "physical/audit.bin",
                15,
            ),
            artifact(
                EvidenceArtifactKind::ChallengeLedger,
                "challenges/ledger.bin",
                16,
            ),
            artifact(
                EvidenceArtifactKind::CertificationPolicy,
                "certification/policy.bin",
                17,
            ),
        ];
        ElectionEvidencePackageManifestV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            evidence_package_profile_id: EVIDENCE_PACKAGE_PROFILE_ID.to_owned(),
            election_constitution_digest: digest(1),
            package_canonicalization_profile_digest: digest(2),
            artifact_index_digest: digest(3),
            package_root_digest: digest(4),
            final_transparency_checkpoint_digest: digest(5),
            artifacts,
            interoperability_profiles: Vec::new(),
        }
    }

    fn stage_receipt(
        stage: VerificationStageId,
        disposition: VerificationStageDisposition,
    ) -> VerificationStageReceiptV1 {
        VerificationStageReceiptV1 {
            stage,
            package_root_digest: digest(4),
            subject_digest: digest(30 + stage as u8),
            verifier_release_digest: digest(20),
            disposition,
            finding_digest: digest(60 + stage as u8),
        }
    }

    fn valid_run() -> VerifierRunReceiptV1 {
        VerifierRunReceiptV1 {
            offline_verifier_profile_id: OFFLINE_VERIFIER_PROFILE_ID.to_owned(),
            package_root_digest: digest(4),
            verifier_implementation_id: "independent-rust-verifier".to_owned(),
            verifier_lineage_digest: digest(18),
            verifier_release_digest: digest(20),
            source_digest: digest(21),
            build_provenance_digest: digest(22),
            execution_policy_digest: digest(23),
            stages: REQUIRED_VERIFICATION_STAGES
                .iter()
                .copied()
                .map(|stage| stage_receipt(stage, VerificationStageDisposition::Pass))
                .collect(),
        }
    }

    #[test]
    fn package_requires_all_core_election_artifacts() {
        assert_eq!(validate_evidence_package_manifest(&valid_manifest()), Ok(()));
        let mut broken = valid_manifest();
        broken
            .artifacts
            .retain(|item| item.kind != EvidenceArtifactKind::ChallengeLedger);
        assert_eq!(
            validate_evidence_package_manifest(&broken),
            Err(EvidencePackageViolation::MissingRequiredArtifactKind(
                EvidenceArtifactKind::ChallengeLedger
            ))
        );
    }

    #[test]
    fn package_path_traversal_fails_closed() {
        let mut broken = valid_manifest();
        broken.artifacts[0].canonical_path = "../constitution.bin".to_owned();
        assert_eq!(
            validate_evidence_package_manifest(&broken),
            Err(EvidencePackageViolation::Artifact {
                index: 0,
                violation: ArtifactRefViolation::NonCanonicalPathComponent,
            })
        );
    }

    #[test]
    fn public_verification_package_rejects_secret_material() {
        let mut broken = valid_manifest();
        broken.artifacts[0].disclosure = ArtifactDisclosureClass::NonPublicOrSecret;
        assert_eq!(
            validate_evidence_package_manifest(&broken),
            Err(EvidencePackageViolation::Artifact {
                index: 0,
                violation: ArtifactRefViolation::NonPublicArtifactForbidden,
            })
        );
    }

    #[test]
    fn offline_policy_has_zero_runtime_authority_dependencies() {
        assert_eq!(
            validate_offline_execution_policy(&OfflineVerifierExecutionPolicyV1::default()),
            Ok(())
        );
        let mut broken = OfflineVerifierExecutionPolicyV1::default();
        broken.network_access = CapabilityRule::Permitted;
        assert_eq!(
            validate_offline_execution_policy(&broken),
            Err(ExecutionPolicyViolation::NetworkAccessMustBeForbidden)
        );
    }

    #[test]
    fn complete_run_is_only_a_verifier_disposition_not_certification() {
        assert_eq!(
            classify_verifier_run(&valid_run()),
            Ok(VerifierRunDisposition::AllRequiredStagesPass)
        );
    }

    #[test]
    fn any_failed_stage_dominates_indeterminate_stage() {
        let mut run = valid_run();
        run.stages[0].disposition = VerificationStageDisposition::Indeterminate;
        run.stages[1].disposition = VerificationStageDisposition::Fail;
        assert_eq!(
            classify_verifier_run(&run),
            Ok(VerifierRunDisposition::AtLeastOneStageFailed)
        );
    }

    fn agreement_attestation(
        release: u8,
        lineage: u8,
        builder: u8,
    ) -> VerifierAgreementAttestationV1 {
        VerifierAgreementAttestationV1 {
            package_root_digest: digest(4),
            verifier_lineage_digest: digest(lineage),
            verifier_release_digest: digest(release),
            builder_control_domain_digest: digest(builder),
            run_receipt_digest: digest(release + 40),
            run_disposition: VerificationStageDisposition::Pass,
        }
    }

    fn agreement_policy() -> VerifierAgreementPolicyV1 {
        VerifierAgreementPolicyV1 {
            minimum_total_verifiers: 3,
            minimum_distinct_implementation_lineages: 3,
            minimum_distinct_builder_control_domains: 2,
        }
    }

    #[test]
    fn renamed_copies_do_not_count_as_independent_implementations() {
        let attestations = vec![
            agreement_attestation(1, 9, 20),
            agreement_attestation(2, 9, 21),
            agreement_attestation(3, 9, 22),
        ];
        assert_eq!(
            validate_independent_verifier_agreement(digest(4), &agreement_policy(), &attestations),
            Err(VerifierAgreementViolation::InsufficientImplementationLineages)
        );
    }

    #[test]
    fn distinct_lineages_under_one_builder_do_not_meet_builder_independence() {
        let attestations = vec![
            agreement_attestation(1, 9, 20),
            agreement_attestation(2, 10, 20),
            agreement_attestation(3, 11, 20),
        ];
        assert_eq!(
            validate_independent_verifier_agreement(digest(4), &agreement_policy(), &attestations),
            Err(VerifierAgreementViolation::InsufficientBuilderControlDomains)
        );
    }

    #[test]
    fn independent_passing_verifiers_can_satisfy_structural_quorum() {
        let attestations = vec![
            agreement_attestation(1, 9, 20),
            agreement_attestation(2, 10, 21),
            agreement_attestation(3, 11, 22),
        ];
        assert_eq!(
            validate_independent_verifier_agreement(digest(4), &agreement_policy(), &attestations),
            Ok(())
        );
    }
}
