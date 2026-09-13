//! ELECT-013 bounded resource contracts for Mycelix public-election verification.
//!
//! The election freezes the workload it may require. Each verifier independently
//! declares what it can process safely. Verification may proceed only when the
//! verifier capability dominates the election policy and the observed package stays
//! within that envelope.

use election_integrity_types::{Digest32, PUBLIC_ELECTION_PROFILE_ID};
use election_verifier_contract as verifier;

pub const RESOURCE_ENVELOPE_PROFILE_ID: &str =
    "mycelix-public-election-verifier-resource-envelope-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerificationResourceLimitsV1 {
    pub max_artifacts: usize,
    pub max_single_artifact_bytes: u64,
    pub max_total_artifact_bytes: u64,
    pub max_canonical_path_bytes: usize,
    pub max_interoperability_profiles: usize,
    pub max_manifest_bytes: u64,
    pub max_container_bytes: u64,
    pub max_expanded_bytes: u64,
    pub max_container_entries: usize,
    pub max_nested_container_depth: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceLimitsViolation {
    ZeroArtifactLimit,
    ZeroSingleArtifactByteLimit,
    ZeroTotalArtifactByteLimit,
    ZeroPathByteLimit,
    ZeroInteroperabilityProfileLimit,
    ZeroManifestByteLimit,
    ZeroContainerByteLimit,
    ZeroExpandedByteLimit,
    ZeroContainerEntryLimit,
    SingleArtifactExceedsTotalArtifactLimit,
    TotalArtifactBytesExceedExpandedLimit,
    ContainerEntriesBelowArtifactLimit,
}

fn validate_resource_limits(
    limits: &VerificationResourceLimitsV1,
) -> Result<(), ResourceLimitsViolation> {
    if limits.max_artifacts == 0 {
        return Err(ResourceLimitsViolation::ZeroArtifactLimit);
    }
    if limits.max_single_artifact_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroSingleArtifactByteLimit);
    }
    if limits.max_total_artifact_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroTotalArtifactByteLimit);
    }
    if limits.max_canonical_path_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroPathByteLimit);
    }
    if limits.max_interoperability_profiles == 0 {
        return Err(ResourceLimitsViolation::ZeroInteroperabilityProfileLimit);
    }
    if limits.max_manifest_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroManifestByteLimit);
    }
    if limits.max_container_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroContainerByteLimit);
    }
    if limits.max_expanded_bytes == 0 {
        return Err(ResourceLimitsViolation::ZeroExpandedByteLimit);
    }
    if limits.max_container_entries == 0 {
        return Err(ResourceLimitsViolation::ZeroContainerEntryLimit);
    }
    if limits.max_single_artifact_bytes > limits.max_total_artifact_bytes {
        return Err(ResourceLimitsViolation::SingleArtifactExceedsTotalArtifactLimit);
    }
    if limits.max_total_artifact_bytes > limits.max_expanded_bytes {
        return Err(ResourceLimitsViolation::TotalArtifactBytesExceedExpandedLimit);
    }
    if limits.max_container_entries < limits.max_artifacts {
        return Err(ResourceLimitsViolation::ContainerEntriesBelowArtifactLimit);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ElectionVerificationResourcePolicyV1 {
    pub public_election_profile_id: String,
    pub resource_envelope_profile_id: String,
    pub election_constitution_digest: Digest32,
    pub package_canonicalization_profile_digest: Digest32,
    pub limits: VerificationResourceLimitsV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourcePolicyViolation {
    WrongPublicElectionProfile,
    WrongResourceEnvelopeProfile,
    ZeroElectionConstitutionDigest,
    ZeroCanonicalizationProfileDigest,
    Limits(ResourceLimitsViolation),
    ArtifactLimitExceedsPackageContract,
    PathLimitExceedsPackageContract,
    InteroperabilityLimitExceedsPackageContract,
}

pub fn validate_resource_policy(
    policy: &ElectionVerificationResourcePolicyV1,
) -> Result<(), ResourcePolicyViolation> {
    if policy.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(ResourcePolicyViolation::WrongPublicElectionProfile);
    }
    if policy.resource_envelope_profile_id != RESOURCE_ENVELOPE_PROFILE_ID {
        return Err(ResourcePolicyViolation::WrongResourceEnvelopeProfile);
    }
    if policy.election_constitution_digest == [0_u8; 32] {
        return Err(ResourcePolicyViolation::ZeroElectionConstitutionDigest);
    }
    if policy.package_canonicalization_profile_digest == [0_u8; 32] {
        return Err(ResourcePolicyViolation::ZeroCanonicalizationProfileDigest);
    }
    validate_resource_limits(&policy.limits).map_err(ResourcePolicyViolation::Limits)?;
    if policy.limits.max_artifacts > verifier::MAX_ARTIFACTS {
        return Err(ResourcePolicyViolation::ArtifactLimitExceedsPackageContract);
    }
    if policy.limits.max_canonical_path_bytes > verifier::MAX_CANONICAL_PATH_BYTES {
        return Err(ResourcePolicyViolation::PathLimitExceedsPackageContract);
    }
    if policy.limits.max_interoperability_profiles > verifier::MAX_INTEROPERABILITY_PROFILES {
        return Err(ResourcePolicyViolation::InteroperabilityLimitExceedsPackageContract);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierResourceCapabilityV1 {
    pub resource_envelope_profile_id: String,
    pub verifier_release_digest: Digest32,
    pub capability_evidence_digest: Digest32,
    pub limits: VerificationResourceLimitsV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceCapabilityViolation {
    WrongResourceEnvelopeProfile,
    ZeroVerifierReleaseDigest,
    ZeroCapabilityEvidenceDigest,
    Limits(ResourceLimitsViolation),
}

pub fn validate_resource_capability(
    capability: &VerifierResourceCapabilityV1,
) -> Result<(), ResourceCapabilityViolation> {
    if capability.resource_envelope_profile_id != RESOURCE_ENVELOPE_PROFILE_ID {
        return Err(ResourceCapabilityViolation::WrongResourceEnvelopeProfile);
    }
    if capability.verifier_release_digest == [0_u8; 32] {
        return Err(ResourceCapabilityViolation::ZeroVerifierReleaseDigest);
    }
    if capability.capability_evidence_digest == [0_u8; 32] {
        return Err(ResourceCapabilityViolation::ZeroCapabilityEvidenceDigest);
    }
    validate_resource_limits(&capability.limits).map_err(ResourceCapabilityViolation::Limits)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceLimitKind {
    Artifacts,
    SingleArtifactBytes,
    TotalArtifactBytes,
    CanonicalPathBytes,
    InteroperabilityProfiles,
    ManifestBytes,
    ContainerBytes,
    ContainerEntries,
    ExpandedBytes,
    NestedContainerDepth,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceGateDisposition {
    Proceed,
    BlockIndeterminate(ResourceLimitKind),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CapabilityAssessmentViolation {
    Policy(ResourcePolicyViolation),
    Capability(ResourceCapabilityViolation),
}

pub fn assess_verifier_capability(
    policy: &ElectionVerificationResourcePolicyV1,
    capability: &VerifierResourceCapabilityV1,
) -> Result<ResourceGateDisposition, CapabilityAssessmentViolation> {
    validate_resource_policy(policy).map_err(CapabilityAssessmentViolation::Policy)?;
    validate_resource_capability(capability).map_err(CapabilityAssessmentViolation::Capability)?;

    let required = &policy.limits;
    let supported = &capability.limits;
    let insufficient = if supported.max_artifacts < required.max_artifacts {
        Some(ResourceLimitKind::Artifacts)
    } else if supported.max_single_artifact_bytes < required.max_single_artifact_bytes {
        Some(ResourceLimitKind::SingleArtifactBytes)
    } else if supported.max_total_artifact_bytes < required.max_total_artifact_bytes {
        Some(ResourceLimitKind::TotalArtifactBytes)
    } else if supported.max_canonical_path_bytes < required.max_canonical_path_bytes {
        Some(ResourceLimitKind::CanonicalPathBytes)
    } else if supported.max_interoperability_profiles < required.max_interoperability_profiles {
        Some(ResourceLimitKind::InteroperabilityProfiles)
    } else if supported.max_manifest_bytes < required.max_manifest_bytes {
        Some(ResourceLimitKind::ManifestBytes)
    } else if supported.max_container_bytes < required.max_container_bytes {
        Some(ResourceLimitKind::ContainerBytes)
    } else if supported.max_container_entries < required.max_container_entries {
        Some(ResourceLimitKind::ContainerEntries)
    } else if supported.max_expanded_bytes < required.max_expanded_bytes {
        Some(ResourceLimitKind::ExpandedBytes)
    } else if supported.max_nested_container_depth < required.max_nested_container_depth {
        Some(ResourceLimitKind::NestedContainerDepth)
    } else {
        None
    };

    Ok(match insufficient {
        Some(kind) => ResourceGateDisposition::BlockIndeterminate(kind),
        None => ResourceGateDisposition::Proceed,
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EvidencePackagePreflightHeaderV1 {
    pub package_root_digest: Digest32,
    pub resource_policy_digest: Digest32,
    pub manifest_byte_length: u64,
    pub container_byte_length: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PreflightHeaderViolation {
    Policy(ResourcePolicyViolation),
    ZeroPackageRootDigest,
    ZeroResourcePolicyDigest,
    EmptyManifest,
    EmptyContainer,
    ManifestExceedsContainer,
}

pub fn evaluate_preflight_header(
    header: &EvidencePackagePreflightHeaderV1,
    policy: &ElectionVerificationResourcePolicyV1,
) -> Result<ResourceGateDisposition, PreflightHeaderViolation> {
    validate_resource_policy(policy).map_err(PreflightHeaderViolation::Policy)?;
    if header.package_root_digest == [0_u8; 32] {
        return Err(PreflightHeaderViolation::ZeroPackageRootDigest);
    }
    if header.resource_policy_digest == [0_u8; 32] {
        return Err(PreflightHeaderViolation::ZeroResourcePolicyDigest);
    }
    if header.manifest_byte_length == 0 {
        return Err(PreflightHeaderViolation::EmptyManifest);
    }
    if header.container_byte_length == 0 {
        return Err(PreflightHeaderViolation::EmptyContainer);
    }
    if header.manifest_byte_length > header.container_byte_length {
        return Err(PreflightHeaderViolation::ManifestExceedsContainer);
    }
    if header.manifest_byte_length > policy.limits.max_manifest_bytes {
        return Ok(ResourceGateDisposition::BlockIndeterminate(
            ResourceLimitKind::ManifestBytes,
        ));
    }
    if header.container_byte_length > policy.limits.max_container_bytes {
        return Ok(ResourceGateDisposition::BlockIndeterminate(
            ResourceLimitKind::ContainerBytes,
        ));
    }
    Ok(ResourceGateDisposition::Proceed)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ManifestResourceObservationV1 {
    pub artifact_count: usize,
    pub total_declared_artifact_bytes: u64,
    pub max_single_artifact_bytes: u64,
    pub max_canonical_path_bytes: usize,
    pub interoperability_profile_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ManifestResourceViolation {
    Policy(ResourcePolicyViolation),
    ParentManifest(verifier::EvidencePackageViolation),
    DeclaredArtifactBytesOverflow,
}

pub fn evaluate_manifest_resources(
    manifest: &verifier::ElectionEvidencePackageManifestV1,
    policy: &ElectionVerificationResourcePolicyV1,
) -> Result<(ResourceGateDisposition, ManifestResourceObservationV1), ManifestResourceViolation> {
    validate_resource_policy(policy).map_err(ManifestResourceViolation::Policy)?;
    verifier::validate_evidence_package_manifest(manifest)
        .map_err(ManifestResourceViolation::ParentManifest)?;

    let mut total_declared_artifact_bytes = 0_u64;
    let mut max_single_artifact_bytes = 0_u64;
    let mut max_canonical_path_bytes = 0_usize;
    for artifact in &manifest.artifacts {
        total_declared_artifact_bytes = total_declared_artifact_bytes
            .checked_add(artifact.byte_length)
            .ok_or(ManifestResourceViolation::DeclaredArtifactBytesOverflow)?;
        max_single_artifact_bytes = max_single_artifact_bytes.max(artifact.byte_length);
        max_canonical_path_bytes = max_canonical_path_bytes.max(artifact.canonical_path.len());
    }

    let observation = ManifestResourceObservationV1 {
        artifact_count: manifest.artifacts.len(),
        total_declared_artifact_bytes,
        max_single_artifact_bytes,
        max_canonical_path_bytes,
        interoperability_profile_count: manifest.interoperability_profiles.len(),
    };

    let limits = &policy.limits;
    let breach = if observation.artifact_count > limits.max_artifacts {
        Some(ResourceLimitKind::Artifacts)
    } else if observation.max_single_artifact_bytes > limits.max_single_artifact_bytes {
        Some(ResourceLimitKind::SingleArtifactBytes)
    } else if observation.total_declared_artifact_bytes > limits.max_total_artifact_bytes {
        Some(ResourceLimitKind::TotalArtifactBytes)
    } else if observation.max_canonical_path_bytes > limits.max_canonical_path_bytes {
        Some(ResourceLimitKind::CanonicalPathBytes)
    } else if observation.interoperability_profile_count > limits.max_interoperability_profiles {
        Some(ResourceLimitKind::InteroperabilityProfiles)
    } else {
        None
    };

    Ok((
        match breach {
            Some(kind) => ResourceGateDisposition::BlockIndeterminate(kind),
            None => ResourceGateDisposition::Proceed,
        },
        observation,
    ))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ContainerResourceObservationV1 {
    pub package_root_digest: Digest32,
    pub resource_policy_digest: Digest32,
    pub observed_container_byte_length: u64,
    pub container_entry_count: usize,
    pub expanded_bytes: u64,
    pub max_nested_container_depth_observed: u16,
    pub observation_evidence_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ContainerResourceViolation {
    Policy(ResourcePolicyViolation),
    ZeroPackageRootDigest,
    ZeroResourcePolicyDigest,
    ZeroObservationEvidenceDigest,
    EmptyContainerEntries,
    EmptyExpandedPackage,
    PackageRootMismatch,
    ResourcePolicyDigestMismatch,
    ContainerByteLengthMismatch,
}

pub fn evaluate_container_resources(
    header: &EvidencePackagePreflightHeaderV1,
    observation: &ContainerResourceObservationV1,
    policy: &ElectionVerificationResourcePolicyV1,
) -> Result<ResourceGateDisposition, ContainerResourceViolation> {
    validate_resource_policy(policy).map_err(ContainerResourceViolation::Policy)?;
    if observation.package_root_digest == [0_u8; 32] {
        return Err(ContainerResourceViolation::ZeroPackageRootDigest);
    }
    if observation.resource_policy_digest == [0_u8; 32] {
        return Err(ContainerResourceViolation::ZeroResourcePolicyDigest);
    }
    if observation.observation_evidence_digest == [0_u8; 32] {
        return Err(ContainerResourceViolation::ZeroObservationEvidenceDigest);
    }
    if observation.container_entry_count == 0 {
        return Err(ContainerResourceViolation::EmptyContainerEntries);
    }
    if observation.expanded_bytes == 0 {
        return Err(ContainerResourceViolation::EmptyExpandedPackage);
    }
    if observation.package_root_digest != header.package_root_digest {
        return Err(ContainerResourceViolation::PackageRootMismatch);
    }
    if observation.resource_policy_digest != header.resource_policy_digest {
        return Err(ContainerResourceViolation::ResourcePolicyDigestMismatch);
    }
    if observation.observed_container_byte_length != header.container_byte_length {
        return Err(ContainerResourceViolation::ContainerByteLengthMismatch);
    }

    let limits = &policy.limits;
    if observation.container_entry_count > limits.max_container_entries {
        return Ok(ResourceGateDisposition::BlockIndeterminate(
            ResourceLimitKind::ContainerEntries,
        ));
    }
    if observation.expanded_bytes > limits.max_expanded_bytes {
        return Ok(ResourceGateDisposition::BlockIndeterminate(
            ResourceLimitKind::ExpandedBytes,
        ));
    }
    if observation.max_nested_container_depth_observed > limits.max_nested_container_depth {
        return Ok(ResourceGateDisposition::BlockIndeterminate(
            ResourceLimitKind::NestedContainerDepth,
        ));
    }
    Ok(ResourceGateDisposition::Proceed)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceConsistencyViolation {
    ContainerEntriesBelowArtifactCount,
    ExpandedBytesBelowDeclaredArtifactBytes,
}

pub fn validate_resource_consistency(
    manifest: &ManifestResourceObservationV1,
    container: &ContainerResourceObservationV1,
) -> Result<(), ResourceConsistencyViolation> {
    if container.container_entry_count < manifest.artifact_count {
        return Err(ResourceConsistencyViolation::ContainerEntriesBelowArtifactCount);
    }
    if container.expanded_bytes < manifest.total_declared_artifact_bytes {
        return Err(ResourceConsistencyViolation::ExpandedBytesBelowDeclaredArtifactBytes);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn limits() -> VerificationResourceLimitsV1 {
        VerificationResourceLimitsV1 {
            max_artifacts: 16,
            max_single_artifact_bytes: 1_024,
            max_total_artifact_bytes: 8_192,
            max_canonical_path_bytes: 256,
            max_interoperability_profiles: 8,
            max_manifest_bytes: 65_536,
            max_container_bytes: 1_048_576,
            max_expanded_bytes: 2_097_152,
            max_container_entries: 64,
            max_nested_container_depth: 2,
        }
    }

    fn policy() -> ElectionVerificationResourcePolicyV1 {
        ElectionVerificationResourcePolicyV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            resource_envelope_profile_id: RESOURCE_ENVELOPE_PROFILE_ID.to_owned(),
            election_constitution_digest: digest(1),
            package_canonicalization_profile_digest: digest(2),
            limits: limits(),
        }
    }

    fn capability() -> VerifierResourceCapabilityV1 {
        VerifierResourceCapabilityV1 {
            resource_envelope_profile_id: RESOURCE_ENVELOPE_PROFILE_ID.to_owned(),
            verifier_release_digest: digest(3),
            capability_evidence_digest: digest(4),
            limits: limits(),
        }
    }

    fn artifact(
        kind: verifier::EvidenceArtifactKind,
        path: &str,
        byte: u8,
    ) -> verifier::EvidenceArtifactRefV1 {
        verifier::EvidenceArtifactRefV1 {
            kind,
            canonical_path: path.to_owned(),
            content_digest: digest(byte),
            byte_length: 10,
            disclosure: verifier::ArtifactDisclosureClass::PublicVerificationEvidence,
        }
    }

    fn manifest() -> verifier::ElectionEvidencePackageManifestV1 {
        verifier::ElectionEvidencePackageManifestV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            evidence_package_profile_id: verifier::EVIDENCE_PACKAGE_PROFILE_ID.to_owned(),
            election_constitution_digest: digest(1),
            package_canonicalization_profile_digest: digest(2),
            artifact_index_digest: digest(3),
            package_root_digest: digest(4),
            final_transparency_checkpoint_digest: digest(5),
            artifacts: vec![
                artifact(
                    verifier::EvidenceArtifactKind::ElectionConstitution,
                    "election/constitution.bin",
                    10,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::TransparencyCheckpointChain,
                    "transparency/checkpoints.bin",
                    11,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::WitnessAttestations,
                    "transparency/witnesses.bin",
                    12,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::AnonymousAuthorityCensus,
                    "authority/census.bin",
                    13,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::TallyEvidence,
                    "tally/evidence.bin",
                    14,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::PhysicalAuditEvidence,
                    "physical/audit.bin",
                    15,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::ChallengeLedger,
                    "challenges/ledger.bin",
                    16,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::CertificationPolicy,
                    "certification/policy.bin",
                    17,
                ),
            ],
            interoperability_profiles: Vec::new(),
        }
    }

    fn header() -> EvidencePackagePreflightHeaderV1 {
        EvidencePackagePreflightHeaderV1 {
            package_root_digest: digest(4),
            resource_policy_digest: digest(6),
            manifest_byte_length: 4_096,
            container_byte_length: 131_072,
        }
    }

    #[test]
    fn supported_policy_can_proceed() {
        assert_eq!(validate_resource_policy(&policy()), Ok(()));
        assert_eq!(validate_resource_capability(&capability()), Ok(()));
        assert_eq!(
            assess_verifier_capability(&policy(), &capability()),
            Ok(ResourceGateDisposition::Proceed)
        );
    }

    #[test]
    fn insufficient_verifier_capability_blocks_indeterminate() {
        let mut limited = capability();
        limited.limits.max_total_artifact_bytes = 4_096;
        assert_eq!(
            assess_verifier_capability(&policy(), &limited),
            Ok(ResourceGateDisposition::BlockIndeterminate(
                ResourceLimitKind::TotalArtifactBytes
            ))
        );
    }

    #[test]
    fn policy_cannot_relax_parent_structural_artifact_cap() {
        let mut oversized = policy();
        oversized.limits.max_artifacts = verifier::MAX_ARTIFACTS + 1;
        oversized.limits.max_container_entries = oversized.limits.max_artifacts;
        assert_eq!(
            validate_resource_policy(&oversized),
            Err(ResourcePolicyViolation::ArtifactLimitExceedsPackageContract)
        );
    }

    #[test]
    fn oversized_manifest_is_blocked_before_full_parse() {
        let mut oversized = header();
        oversized.manifest_byte_length = policy().limits.max_manifest_bytes + 1;
        assert_eq!(
            evaluate_preflight_header(&oversized, &policy()),
            Ok(ResourceGateDisposition::BlockIndeterminate(
                ResourceLimitKind::ManifestBytes
            ))
        );
    }

    #[test]
    fn aggregate_manifest_bytes_are_checked_with_overflow_protection() {
        let mut subject = manifest();
        subject.artifacts[0].byte_length = u64::MAX;
        subject.artifacts[1].byte_length = 1;
        assert_eq!(
            evaluate_manifest_resources(&subject, &policy()),
            Err(ManifestResourceViolation::DeclaredArtifactBytesOverflow)
        );
    }

    #[test]
    fn aggregate_manifest_budget_excess_blocks_indeterminate() {
        let mut tighter = policy();
        tighter.limits.max_total_artifact_bytes = 7_000;
        let mut subject = manifest();
        for artifact in &mut subject.artifacts {
            artifact.byte_length = 900;
        }
        let (disposition, observation) =
            evaluate_manifest_resources(&subject, &tighter).expect("valid manifest structure");
        assert_eq!(observation.total_declared_artifact_bytes, 7_200);
        assert_eq!(
            disposition,
            ResourceGateDisposition::BlockIndeterminate(ResourceLimitKind::TotalArtifactBytes)
        );
    }

    #[test]
    fn expanded_container_budget_excess_blocks_indeterminate() {
        let subject_header = header();
        let observation = ContainerResourceObservationV1 {
            package_root_digest: subject_header.package_root_digest,
            resource_policy_digest: subject_header.resource_policy_digest,
            observed_container_byte_length: subject_header.container_byte_length,
            container_entry_count: 8,
            expanded_bytes: policy().limits.max_expanded_bytes + 1,
            max_nested_container_depth_observed: 0,
            observation_evidence_digest: digest(7),
        };
        assert_eq!(
            evaluate_container_resources(&subject_header, &observation, &policy()),
            Ok(ResourceGateDisposition::BlockIndeterminate(
                ResourceLimitKind::ExpandedBytes
            ))
        );
    }

    #[test]
    fn resource_summary_cannot_claim_less_data_than_manifest_declares() {
        let (_, manifest_observation) =
            evaluate_manifest_resources(&manifest(), &policy()).expect("valid manifest");
        let subject_header = header();
        let container_observation = ContainerResourceObservationV1 {
            package_root_digest: subject_header.package_root_digest,
            resource_policy_digest: subject_header.resource_policy_digest,
            observed_container_byte_length: subject_header.container_byte_length,
            container_entry_count: manifest_observation.artifact_count,
            expanded_bytes: manifest_observation.total_declared_artifact_bytes - 1,
            max_nested_container_depth_observed: 0,
            observation_evidence_digest: digest(8),
        };
        assert_eq!(
            validate_resource_consistency(&manifest_observation, &container_observation),
            Err(ResourceConsistencyViolation::ExpandedBytesBelowDeclaredArtifactBytes)
        );
    }
}
