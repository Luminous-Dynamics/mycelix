//! ADV-003 adversarial resource corpus for Mycelix public-election verification.
//!
//! Capacity failures are intentionally distinguished from malformed resource
//! evidence. A verifier that cannot safely process a valid frozen workload must
//! block certification as indeterminate; inconsistent or overflowing evidence
//! fails closed as malformed.

use election_integrity_types::{Digest32, ElectionTheoremId, PUBLIC_ELECTION_PROFILE_ID};
use election_verifier_contract as verifier;
use election_verifier_resource_envelope as resource;

pub const RESOURCE_ADVERSARIAL_CORPUS_PROFILE_ID: &str =
    "mycelix-public-election-resource-adversarial-corpus-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceAttackCaseId {
    ArtifactCountBudgetExceeded,
    SingleArtifactBudgetExceeded,
    AggregateArtifactBudgetExceeded,
    CanonicalPathBudgetExceeded,
    InteroperabilityProfileBudgetExceeded,
    ManifestPreflightBudgetExceeded,
    ContainerPreflightBudgetExceeded,
    ContainerEntryBudgetExceeded,
    ExpandedByteBudgetExceeded,
    NestedContainerDepthExceeded,
    VerifierCapabilityBelowElectionPolicy,
    DeclaredArtifactByteSumOverflow,
    ManifestLargerThanContainer,
    ContainerLengthObservationMismatch,
    ExpandedBytesUnderReported,
    ElectionPolicyRelaxesParentArtifactCap,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceAttackDisposition {
    BlockIndeterminate,
    FailClosed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceFindingId {
    Limit(resource::ResourceLimitKind),
    DeclaredArtifactBytesOverflow,
    ManifestExceedsContainer,
    ContainerByteLengthMismatch,
    ExpandedBytesBelowDeclaredArtifactBytes,
    ArtifactLimitExceedsPackageContract,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResourceAttackSpecV1 {
    pub id: ResourceAttackCaseId,
    pub primary_theorem: ElectionTheoremId,
    pub expected_disposition: ResourceAttackDisposition,
    pub expected_finding: ResourceFindingId,
    pub certification_blocking: bool,
}

pub const REQUIRED_RESOURCE_LIMIT_KINDS: [resource::ResourceLimitKind; 10] = [
    resource::ResourceLimitKind::Artifacts,
    resource::ResourceLimitKind::SingleArtifactBytes,
    resource::ResourceLimitKind::TotalArtifactBytes,
    resource::ResourceLimitKind::CanonicalPathBytes,
    resource::ResourceLimitKind::InteroperabilityProfiles,
    resource::ResourceLimitKind::ManifestBytes,
    resource::ResourceLimitKind::ContainerBytes,
    resource::ResourceLimitKind::ContainerEntries,
    resource::ResourceLimitKind::ExpandedBytes,
    resource::ResourceLimitKind::NestedContainerDepth,
];

pub const REQUIRED_RESOURCE_ATTACKS: [ResourceAttackSpecV1; 16] = [
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ArtifactCountBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::Artifacts),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::SingleArtifactBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(
            resource::ResourceLimitKind::SingleArtifactBytes,
        ),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::AggregateArtifactBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::TotalArtifactBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::CanonicalPathBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::CanonicalPathBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::InteroperabilityProfileBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(
            resource::ResourceLimitKind::InteroperabilityProfiles,
        ),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ManifestPreflightBudgetExceeded,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::ManifestBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ContainerPreflightBudgetExceeded,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::ContainerBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ContainerEntryBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::ContainerEntries),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ExpandedByteBudgetExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::ExpandedBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::NestedContainerDepthExceeded,
        primary_theorem: ElectionTheoremId::RecoverableVerification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(
            resource::ResourceLimitKind::NestedContainerDepth,
        ),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::VerifierCapabilityBelowElectionPolicy,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::BlockIndeterminate,
        expected_finding: ResourceFindingId::Limit(resource::ResourceLimitKind::TotalArtifactBytes),
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::DeclaredArtifactByteSumOverflow,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::FailClosed,
        expected_finding: ResourceFindingId::DeclaredArtifactBytesOverflow,
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ManifestLargerThanContainer,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::FailClosed,
        expected_finding: ResourceFindingId::ManifestExceedsContainer,
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ContainerLengthObservationMismatch,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ResourceAttackDisposition::FailClosed,
        expected_finding: ResourceFindingId::ContainerByteLengthMismatch,
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ExpandedBytesUnderReported,
        primary_theorem: ElectionTheoremId::EvidenceContinuity,
        expected_disposition: ResourceAttackDisposition::FailClosed,
        expected_finding: ResourceFindingId::ExpandedBytesBelowDeclaredArtifactBytes,
        certification_blocking: true,
    },
    ResourceAttackSpecV1 {
        id: ResourceAttackCaseId::ElectionPolicyRelaxesParentArtifactCap,
        primary_theorem: ElectionTheoremId::EvidenceBeforeCertification,
        expected_disposition: ResourceAttackDisposition::FailClosed,
        expected_finding: ResourceFindingId::ArtifactLimitExceedsPackageContract,
        certification_blocking: true,
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResourceAttackObservationV1 {
    pub disposition: ResourceAttackDisposition,
    pub finding: ResourceFindingId,
}

fn observation(
    disposition: ResourceAttackDisposition,
    finding: ResourceFindingId,
) -> ResourceAttackObservationV1 {
    ResourceAttackObservationV1 {
        disposition,
        finding,
    }
}

fn digest(byte: u8) -> Digest32 {
    [byte; 32]
}

fn limits() -> resource::VerificationResourceLimitsV1 {
    resource::VerificationResourceLimitsV1 {
        max_artifacts: 8,
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

fn policy() -> resource::ElectionVerificationResourcePolicyV1 {
    resource::ElectionVerificationResourcePolicyV1 {
        public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
        resource_envelope_profile_id: resource::RESOURCE_ENVELOPE_PROFILE_ID.to_owned(),
        election_constitution_digest: digest(1),
        package_canonicalization_profile_digest: digest(2),
        limits: limits(),
    }
}

fn capability() -> resource::VerifierResourceCapabilityV1 {
    resource::VerifierResourceCapabilityV1 {
        resource_envelope_profile_id: resource::RESOURCE_ENVELOPE_PROFILE_ID.to_owned(),
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

fn header() -> resource::EvidencePackagePreflightHeaderV1 {
    resource::EvidencePackagePreflightHeaderV1 {
        package_root_digest: digest(4),
        resource_policy_digest: digest(6),
        manifest_byte_length: 4_096,
        container_byte_length: 131_072,
    }
}

fn container_observation() -> resource::ContainerResourceObservationV1 {
    let header = header();
    resource::ContainerResourceObservationV1 {
        package_root_digest: header.package_root_digest,
        resource_policy_digest: header.resource_policy_digest,
        observed_container_byte_length: header.container_byte_length,
        container_entry_count: 8,
        expanded_bytes: 80,
        max_nested_container_depth_observed: 0,
        observation_evidence_digest: digest(7),
    }
}

pub fn expected_observation(id: ResourceAttackCaseId) -> ResourceAttackObservationV1 {
    let spec = REQUIRED_RESOURCE_ATTACKS
        .iter()
        .find(|spec| spec.id == id)
        .expect("registered attack");
    observation(spec.expected_disposition, spec.expected_finding)
}

pub fn execute_case(id: ResourceAttackCaseId) -> ResourceAttackObservationV1 {
    match id {
        ResourceAttackCaseId::ArtifactCountBudgetExceeded => {
            let mut subject = manifest();
            subject.artifacts.push(artifact(
                verifier::EvidenceArtifactKind::VerifierMetadata,
                "metadata/verifier.bin",
                18,
            ));
            let (result, _) = resource::evaluate_manifest_resources(&subject, &policy())
                .expect("parent manifest remains valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::Artifacts
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::Artifacts),
            )
        }
        ResourceAttackCaseId::SingleArtifactBudgetExceeded => {
            let mut subject = manifest();
            subject.artifacts[0].byte_length = 1_025;
            let (result, _) = resource::evaluate_manifest_resources(&subject, &policy())
                .expect("parent manifest remains valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::SingleArtifactBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::SingleArtifactBytes),
            )
        }
        ResourceAttackCaseId::AggregateArtifactBudgetExceeded => {
            let mut subject = manifest();
            let mut constrained = policy();
            constrained.limits.max_total_artifact_bytes = 7_000;
            for item in &mut subject.artifacts {
                item.byte_length = 900;
            }
            let (result, _) = resource::evaluate_manifest_resources(&subject, &constrained)
                .expect("parent manifest remains valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::TotalArtifactBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::TotalArtifactBytes),
            )
        }
        ResourceAttackCaseId::CanonicalPathBudgetExceeded => {
            let mut constrained = policy();
            constrained.limits.max_canonical_path_bytes = 20;
            let (result, _) = resource::evaluate_manifest_resources(&manifest(), &constrained)
                .expect("parent manifest remains valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::CanonicalPathBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::CanonicalPathBytes),
            )
        }
        ResourceAttackCaseId::InteroperabilityProfileBudgetExceeded => {
            let mut subject = manifest();
            subject.interoperability_profiles = vec![
                verifier::InteroperabilityProfileRefV1 {
                    profile_id: "cdf-a".to_owned(),
                    schema_or_specification_digest: digest(20),
                    exported_artifact_digest: digest(21),
                },
                verifier::InteroperabilityProfileRefV1 {
                    profile_id: "cdf-b".to_owned(),
                    schema_or_specification_digest: digest(22),
                    exported_artifact_digest: digest(23),
                },
            ];
            let mut constrained = policy();
            constrained.limits.max_interoperability_profiles = 1;
            let (result, _) = resource::evaluate_manifest_resources(&subject, &constrained)
                .expect("parent manifest remains valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::InteroperabilityProfiles
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::InteroperabilityProfiles),
            )
        }
        ResourceAttackCaseId::ManifestPreflightBudgetExceeded => {
            let mut subject = header();
            subject.manifest_byte_length = policy().limits.max_manifest_bytes + 1;
            let result = resource::evaluate_preflight_header(&subject, &policy())
                .expect("header remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::ManifestBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::ManifestBytes),
            )
        }
        ResourceAttackCaseId::ContainerPreflightBudgetExceeded => {
            let mut subject = header();
            subject.container_byte_length = policy().limits.max_container_bytes + 1;
            let result = resource::evaluate_preflight_header(&subject, &policy())
                .expect("header remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::ContainerBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::ContainerBytes),
            )
        }
        ResourceAttackCaseId::ContainerEntryBudgetExceeded => {
            let mut subject = container_observation();
            subject.container_entry_count = policy().limits.max_container_entries + 1;
            let result = resource::evaluate_container_resources(&header(), &subject, &policy())
                .expect("observation remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::ContainerEntries
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::ContainerEntries),
            )
        }
        ResourceAttackCaseId::ExpandedByteBudgetExceeded => {
            let mut subject = container_observation();
            subject.expanded_bytes = policy().limits.max_expanded_bytes + 1;
            let result = resource::evaluate_container_resources(&header(), &subject, &policy())
                .expect("observation remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::ExpandedBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::ExpandedBytes),
            )
        }
        ResourceAttackCaseId::NestedContainerDepthExceeded => {
            let mut subject = container_observation();
            subject.max_nested_container_depth_observed =
                policy().limits.max_nested_container_depth + 1;
            let result = resource::evaluate_container_resources(&header(), &subject, &policy())
                .expect("observation remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::NestedContainerDepth
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::NestedContainerDepth),
            )
        }
        ResourceAttackCaseId::VerifierCapabilityBelowElectionPolicy => {
            let mut subject = capability();
            subject.limits.max_total_artifact_bytes = 4_096;
            let result = resource::assess_verifier_capability(&policy(), &subject)
                .expect("capability remains structurally valid");
            assert_eq!(
                result,
                resource::ResourceGateDisposition::BlockIndeterminate(
                    resource::ResourceLimitKind::TotalArtifactBytes
                )
            );
            observation(
                ResourceAttackDisposition::BlockIndeterminate,
                ResourceFindingId::Limit(resource::ResourceLimitKind::TotalArtifactBytes),
            )
        }
        ResourceAttackCaseId::DeclaredArtifactByteSumOverflow => {
            let mut subject = manifest();
            subject.artifacts[0].byte_length = u64::MAX;
            subject.artifacts[1].byte_length = 1;
            assert_eq!(
                resource::evaluate_manifest_resources(&subject, &policy()),
                Err(resource::ManifestResourceViolation::DeclaredArtifactBytesOverflow)
            );
            observation(
                ResourceAttackDisposition::FailClosed,
                ResourceFindingId::DeclaredArtifactBytesOverflow,
            )
        }
        ResourceAttackCaseId::ManifestLargerThanContainer => {
            let mut subject = header();
            subject.manifest_byte_length = 20_000;
            subject.container_byte_length = 10_000;
            assert_eq!(
                resource::evaluate_preflight_header(&subject, &policy()),
                Err(resource::PreflightHeaderViolation::ManifestExceedsContainer)
            );
            observation(
                ResourceAttackDisposition::FailClosed,
                ResourceFindingId::ManifestExceedsContainer,
            )
        }
        ResourceAttackCaseId::ContainerLengthObservationMismatch => {
            let mut subject = container_observation();
            subject.observed_container_byte_length -= 1;
            assert_eq!(
                resource::evaluate_container_resources(&header(), &subject, &policy()),
                Err(resource::ContainerResourceViolation::ContainerByteLengthMismatch)
            );
            observation(
                ResourceAttackDisposition::FailClosed,
                ResourceFindingId::ContainerByteLengthMismatch,
            )
        }
        ResourceAttackCaseId::ExpandedBytesUnderReported => {
            let (_, manifest_observation) =
                resource::evaluate_manifest_resources(&manifest(), &policy())
                    .expect("valid manifest");
            let mut subject = container_observation();
            subject.expanded_bytes = manifest_observation.total_declared_artifact_bytes - 1;
            assert_eq!(
                resource::validate_resource_consistency(&manifest_observation, &subject),
                Err(
                    resource::ResourceConsistencyViolation::ExpandedBytesBelowDeclaredArtifactBytes
                )
            );
            observation(
                ResourceAttackDisposition::FailClosed,
                ResourceFindingId::ExpandedBytesBelowDeclaredArtifactBytes,
            )
        }
        ResourceAttackCaseId::ElectionPolicyRelaxesParentArtifactCap => {
            let mut subject = policy();
            subject.limits.max_artifacts = verifier::MAX_ARTIFACTS + 1;
            subject.limits.max_container_entries = subject.limits.max_artifacts;
            assert_eq!(
                resource::validate_resource_policy(&subject),
                Err(resource::ResourcePolicyViolation::ArtifactLimitExceedsPackageContract)
            );
            observation(
                ResourceAttackDisposition::FailClosed,
                ResourceFindingId::ArtifactLimitExceedsPackageContract,
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_registered_resource_attack_has_exact_expected_result() {
        for spec in REQUIRED_RESOURCE_ATTACKS {
            assert!(spec.certification_blocking);
            assert_eq!(execute_case(spec.id), expected_observation(spec.id));
        }
    }

    #[test]
    fn every_resource_limit_axis_has_an_executable_capacity_attack() {
        for kind in REQUIRED_RESOURCE_LIMIT_KINDS {
            assert!(REQUIRED_RESOURCE_ATTACKS.iter().any(|spec| {
                spec.expected_disposition == ResourceAttackDisposition::BlockIndeterminate
                    && spec.expected_finding == ResourceFindingId::Limit(kind)
            }));
        }
    }

    #[test]
    fn malformed_evidence_is_distinct_from_capacity_exhaustion() {
        assert!(REQUIRED_RESOURCE_ATTACKS.iter().any(|spec| {
            spec.expected_disposition == ResourceAttackDisposition::FailClosed
                && spec.expected_finding == ResourceFindingId::DeclaredArtifactBytesOverflow
        }));
        assert!(REQUIRED_RESOURCE_ATTACKS.iter().any(|spec| {
            spec.expected_disposition == ResourceAttackDisposition::BlockIndeterminate
        }));
    }

    #[test]
    fn positive_resource_path_still_proceeds() {
        assert_eq!(resource::validate_resource_policy(&policy()), Ok(()));
        assert_eq!(
            resource::validate_resource_capability(&capability()),
            Ok(())
        );
        assert_eq!(
            resource::assess_verifier_capability(&policy(), &capability()),
            Ok(resource::ResourceGateDisposition::Proceed)
        );
        assert_eq!(
            resource::evaluate_preflight_header(&header(), &policy()),
            Ok(resource::ResourceGateDisposition::Proceed)
        );
        let (manifest_disposition, manifest_observation) =
            resource::evaluate_manifest_resources(&manifest(), &policy()).expect("valid manifest");
        assert_eq!(
            manifest_disposition,
            resource::ResourceGateDisposition::Proceed
        );
        let container = container_observation();
        assert_eq!(
            resource::evaluate_container_resources(&header(), &container, &policy()),
            Ok(resource::ResourceGateDisposition::Proceed)
        );
        assert_eq!(
            resource::validate_resource_consistency(&manifest_observation, &container),
            Ok(())
        );
    }
}
