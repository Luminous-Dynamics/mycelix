//! ELECT-014 non-circular resource-policy anchoring for Mycelix public elections.
//!
//! The resource-policy commitment deliberately excludes both the final election
//! constitution digest and the constitution's certification-policy digest. Those
//! values are downstream of the resource commitment and including either would
//! create a circular hash dependency.

use election_integrity_types::{
    ConstitutionViolation, Digest32, ElectionConstitutionV1, PUBLIC_ELECTION_PROFILE_ID,
    validate_election_constitution,
};
use election_verifier_contract as verifier;
use election_verifier_resource_envelope as resource;
use sha2::{Digest as ShaDigest, Sha256};

pub const RESOURCE_POLICY_ANCHOR_PROFILE_ID: &str =
    "mycelix-public-election-resource-policy-anchor-v1";
pub const RESOURCE_POLICY_COMMITMENT_HASH_ID: &str = "sha-256";

const RESOURCE_POLICY_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:RESOURCE-POLICY-COMMITMENT:V1\0";
const CERTIFICATION_BINDING_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:CERTIFICATION-POLICY-BINDING:V1\0";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResourcePolicyCommitmentSubjectV1 {
    pub election_definition_digest: Digest32,
    pub jurisdiction_snapshot_digest: Digest32,
    pub eligibility_rules_digest: Digest32,
    pub ballot_definition_digest: Digest32,
    pub trustee_policy_digest: Digest32,
    pub audit_policy_digest: Digest32,
    pub dispute_policy_digest: Digest32,
    pub package_canonicalization_profile_digest: Digest32,
    pub limits: resource::VerificationResourceLimitsV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommitmentCanonicalizationViolation {
    PlatformIntegerTooWide,
}

fn append_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn append_usize(
    bytes: &mut Vec<u8>,
    value: usize,
) -> Result<(), CommitmentCanonicalizationViolation> {
    let value = u64::try_from(value)
        .map_err(|_| CommitmentCanonicalizationViolation::PlatformIntegerTooWide)?;
    append_u64(bytes, value);
    Ok(())
}

pub fn canonical_resource_policy_commitment_bytes(
    subject: &ResourcePolicyCommitmentSubjectV1,
) -> Result<Vec<u8>, CommitmentCanonicalizationViolation> {
    let mut bytes = Vec::with_capacity(RESOURCE_POLICY_DOMAIN.len() + 8 * 32 + 10 * 8);
    bytes.extend_from_slice(RESOURCE_POLICY_DOMAIN);
    bytes.extend_from_slice(&subject.election_definition_digest);
    bytes.extend_from_slice(&subject.jurisdiction_snapshot_digest);
    bytes.extend_from_slice(&subject.eligibility_rules_digest);
    bytes.extend_from_slice(&subject.ballot_definition_digest);
    bytes.extend_from_slice(&subject.trustee_policy_digest);
    bytes.extend_from_slice(&subject.audit_policy_digest);
    bytes.extend_from_slice(&subject.dispute_policy_digest);
    bytes.extend_from_slice(&subject.package_canonicalization_profile_digest);

    append_usize(&mut bytes, subject.limits.max_artifacts)?;
    append_u64(&mut bytes, subject.limits.max_single_artifact_bytes);
    append_u64(&mut bytes, subject.limits.max_total_artifact_bytes);
    append_usize(&mut bytes, subject.limits.max_canonical_path_bytes)?;
    append_usize(&mut bytes, subject.limits.max_interoperability_profiles)?;
    append_u64(&mut bytes, subject.limits.max_manifest_bytes);
    append_u64(&mut bytes, subject.limits.max_container_bytes);
    append_u64(&mut bytes, subject.limits.max_expanded_bytes);
    append_usize(&mut bytes, subject.limits.max_container_entries)?;
    append_u64(
        &mut bytes,
        u64::from(subject.limits.max_nested_container_depth),
    );

    Ok(bytes)
}

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let output = hasher.finalize();
    let mut digest = [0_u8; 32];
    digest.copy_from_slice(&output);
    digest
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResourcePolicyCommitmentViolation {
    Constitution(ConstitutionViolation),
    Policy(resource::ResourcePolicyViolation),
    Canonicalization(CommitmentCanonicalizationViolation),
}

pub fn resource_policy_commitment_subject(
    constitution: &ElectionConstitutionV1,
    policy: &resource::ElectionVerificationResourcePolicyV1,
) -> Result<ResourcePolicyCommitmentSubjectV1, ResourcePolicyCommitmentViolation> {
    validate_election_constitution(constitution)
        .map_err(ResourcePolicyCommitmentViolation::Constitution)?;
    resource::validate_resource_policy(policy)
        .map_err(ResourcePolicyCommitmentViolation::Policy)?;

    Ok(ResourcePolicyCommitmentSubjectV1 {
        election_definition_digest: constitution.election_definition_digest,
        jurisdiction_snapshot_digest: constitution.jurisdiction_snapshot_digest,
        eligibility_rules_digest: constitution.eligibility_rules_digest,
        ballot_definition_digest: constitution.ballot_definition_digest,
        trustee_policy_digest: constitution.trustee_policy_digest,
        audit_policy_digest: constitution.audit_policy_digest,
        dispute_policy_digest: constitution.dispute_policy_digest,
        package_canonicalization_profile_digest: policy.package_canonicalization_profile_digest,
        limits: policy.limits,
    })
}

pub fn resource_policy_commitment_digest(
    constitution: &ElectionConstitutionV1,
    policy: &resource::ElectionVerificationResourcePolicyV1,
) -> Result<Digest32, ResourcePolicyCommitmentViolation> {
    let subject = resource_policy_commitment_subject(constitution, policy)?;
    let bytes = canonical_resource_policy_commitment_bytes(&subject)
        .map_err(ResourcePolicyCommitmentViolation::Canonicalization)?;
    Ok(sha256(&bytes))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CertificationPolicyBindingV1 {
    pub public_election_profile_id: String,
    pub resource_policy_anchor_profile_id: String,
    pub election_definition_digest: Digest32,
    pub resource_policy_commitment_digest: Digest32,
    pub certification_requirements_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CertificationPolicyBindingViolation {
    WrongPublicElectionProfile,
    WrongResourcePolicyAnchorProfile,
    ZeroElectionDefinitionDigest,
    ZeroResourcePolicyCommitmentDigest,
    ZeroCertificationRequirementsDigest,
}

pub fn validate_certification_policy_binding(
    binding: &CertificationPolicyBindingV1,
) -> Result<(), CertificationPolicyBindingViolation> {
    let zero = [0_u8; 32];
    if binding.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(CertificationPolicyBindingViolation::WrongPublicElectionProfile);
    }
    if binding.resource_policy_anchor_profile_id != RESOURCE_POLICY_ANCHOR_PROFILE_ID {
        return Err(CertificationPolicyBindingViolation::WrongResourcePolicyAnchorProfile);
    }
    if binding.election_definition_digest == zero {
        return Err(CertificationPolicyBindingViolation::ZeroElectionDefinitionDigest);
    }
    if binding.resource_policy_commitment_digest == zero {
        return Err(CertificationPolicyBindingViolation::ZeroResourcePolicyCommitmentDigest);
    }
    if binding.certification_requirements_digest == zero {
        return Err(CertificationPolicyBindingViolation::ZeroCertificationRequirementsDigest);
    }
    Ok(())
}

pub fn canonical_certification_policy_binding_bytes(
    binding: &CertificationPolicyBindingV1,
) -> Result<Vec<u8>, CertificationPolicyBindingViolation> {
    validate_certification_policy_binding(binding)?;
    let mut bytes = Vec::with_capacity(CERTIFICATION_BINDING_DOMAIN.len() + 3 * 32);
    bytes.extend_from_slice(CERTIFICATION_BINDING_DOMAIN);
    bytes.extend_from_slice(&binding.election_definition_digest);
    bytes.extend_from_slice(&binding.resource_policy_commitment_digest);
    bytes.extend_from_slice(&binding.certification_requirements_digest);
    Ok(bytes)
}

pub fn certification_policy_binding_digest(
    binding: &CertificationPolicyBindingV1,
) -> Result<Digest32, CertificationPolicyBindingViolation> {
    Ok(sha256(&canonical_certification_policy_binding_bytes(
        binding,
    )?))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResourcePolicyAnchorEvidenceV1 {
    pub package_root_digest: Digest32,
    pub election_constitution_digest: Digest32,
    pub election_definition_digest: Digest32,
    pub resource_policy_commitment_digest: Digest32,
    pub certification_policy_digest: Digest32,
    pub certification_requirements_digest: Digest32,
    pub preflight_resource_disposition: resource::ResourceGateDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResourcePolicyAnchorViolation {
    Constitution(ConstitutionViolation),
    Policy(resource::ResourcePolicyViolation),
    Manifest(verifier::EvidencePackageViolation),
    Preflight(resource::PreflightHeaderViolation),
    Commitment(ResourcePolicyCommitmentViolation),
    CertificationBinding(CertificationPolicyBindingViolation),
    RequiredArtifactMissing(verifier::EvidenceArtifactKind),
    ElectionConstitutionArtifactDigestMismatch,
    PolicyConstitutionDigestMismatch,
    PackageCanonicalizationDigestMismatch,
    PackageRootMismatch,
    PreflightResourcePolicyDigestMismatch,
    BindingElectionDefinitionMismatch,
    BindingResourcePolicyDigestMismatch,
    CertificationPolicyDigestMismatch,
    CertificationPolicyArtifactDigestMismatch,
}

fn required_artifact_digest(
    manifest: &verifier::ElectionEvidencePackageManifestV1,
    kind: verifier::EvidenceArtifactKind,
) -> Result<Digest32, ResourcePolicyAnchorViolation> {
    manifest
        .artifacts
        .iter()
        .find(|artifact| artifact.kind == kind)
        .map(|artifact| artifact.content_digest)
        .ok_or(ResourcePolicyAnchorViolation::RequiredArtifactMissing(kind))
}

pub fn validate_resource_policy_anchor(
    constitution: &ElectionConstitutionV1,
    policy: &resource::ElectionVerificationResourcePolicyV1,
    certification_binding: &CertificationPolicyBindingV1,
    manifest: &verifier::ElectionEvidencePackageManifestV1,
    preflight: &resource::EvidencePackagePreflightHeaderV1,
) -> Result<ResourcePolicyAnchorEvidenceV1, ResourcePolicyAnchorViolation> {
    validate_election_constitution(constitution)
        .map_err(ResourcePolicyAnchorViolation::Constitution)?;
    resource::validate_resource_policy(policy).map_err(ResourcePolicyAnchorViolation::Policy)?;
    verifier::validate_evidence_package_manifest(manifest)
        .map_err(ResourcePolicyAnchorViolation::Manifest)?;

    let preflight_resource_disposition = resource::evaluate_preflight_header(preflight, policy)
        .map_err(ResourcePolicyAnchorViolation::Preflight)?;

    let constitution_artifact_digest = required_artifact_digest(
        manifest,
        verifier::EvidenceArtifactKind::ElectionConstitution,
    )?;
    if constitution_artifact_digest != manifest.election_constitution_digest {
        return Err(ResourcePolicyAnchorViolation::ElectionConstitutionArtifactDigestMismatch);
    }
    if policy.election_constitution_digest != manifest.election_constitution_digest {
        return Err(ResourcePolicyAnchorViolation::PolicyConstitutionDigestMismatch);
    }
    if policy.package_canonicalization_profile_digest
        != manifest.package_canonicalization_profile_digest
    {
        return Err(ResourcePolicyAnchorViolation::PackageCanonicalizationDigestMismatch);
    }
    if preflight.package_root_digest != manifest.package_root_digest {
        return Err(ResourcePolicyAnchorViolation::PackageRootMismatch);
    }

    let resource_policy_digest = resource_policy_commitment_digest(constitution, policy)
        .map_err(ResourcePolicyAnchorViolation::Commitment)?;
    if preflight.resource_policy_digest != resource_policy_digest {
        return Err(ResourcePolicyAnchorViolation::PreflightResourcePolicyDigestMismatch);
    }

    validate_certification_policy_binding(certification_binding)
        .map_err(ResourcePolicyAnchorViolation::CertificationBinding)?;
    if certification_binding.election_definition_digest != constitution.election_definition_digest {
        return Err(ResourcePolicyAnchorViolation::BindingElectionDefinitionMismatch);
    }
    if certification_binding.resource_policy_commitment_digest != resource_policy_digest {
        return Err(ResourcePolicyAnchorViolation::BindingResourcePolicyDigestMismatch);
    }

    let certification_policy_digest = certification_policy_binding_digest(certification_binding)
        .map_err(ResourcePolicyAnchorViolation::CertificationBinding)?;
    if certification_policy_digest != constitution.certification_policy_digest {
        return Err(ResourcePolicyAnchorViolation::CertificationPolicyDigestMismatch);
    }

    let certification_artifact_digest = required_artifact_digest(
        manifest,
        verifier::EvidenceArtifactKind::CertificationPolicy,
    )?;
    if certification_artifact_digest != certification_policy_digest {
        return Err(ResourcePolicyAnchorViolation::CertificationPolicyArtifactDigestMismatch);
    }

    Ok(ResourcePolicyAnchorEvidenceV1 {
        package_root_digest: manifest.package_root_digest,
        election_constitution_digest: manifest.election_constitution_digest,
        election_definition_digest: constitution.election_definition_digest,
        resource_policy_commitment_digest: resource_policy_digest,
        certification_policy_digest,
        certification_requirements_digest: certification_binding.certification_requirements_digest,
        preflight_resource_disposition,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn limits() -> resource::VerificationResourceLimitsV1 {
        resource::VerificationResourceLimitsV1 {
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

    fn base_constitution() -> ElectionConstitutionV1 {
        ElectionConstitutionV1 {
            profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            election_id: "election-2030-general".to_owned(),
            election_definition_digest: digest(1),
            jurisdiction_snapshot_digest: digest(2),
            eligibility_rules_digest: digest(3),
            ballot_definition_digest: digest(4),
            trustee_policy_digest: digest(5),
            audit_policy_digest: digest(6),
            dispute_policy_digest: digest(7),
            certification_policy_digest: digest(8),
            authority_policy: Default::default(),
        }
    }

    fn policy() -> resource::ElectionVerificationResourcePolicyV1 {
        resource::ElectionVerificationResourcePolicyV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            resource_envelope_profile_id: resource::RESOURCE_ENVELOPE_PROFILE_ID.to_owned(),
            election_constitution_digest: digest(90),
            package_canonicalization_profile_digest: digest(91),
            limits: limits(),
        }
    }

    fn binding_for(
        constitution: &ElectionConstitutionV1,
        policy: &resource::ElectionVerificationResourcePolicyV1,
    ) -> CertificationPolicyBindingV1 {
        CertificationPolicyBindingV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            resource_policy_anchor_profile_id: RESOURCE_POLICY_ANCHOR_PROFILE_ID.to_owned(),
            election_definition_digest: constitution.election_definition_digest,
            resource_policy_commitment_digest: resource_policy_commitment_digest(
                constitution,
                policy,
            )
            .expect("valid resource commitment"),
            certification_requirements_digest: digest(92),
        }
    }

    fn finalized_constitution_and_binding() -> (ElectionConstitutionV1, CertificationPolicyBindingV1)
    {
        let mut constitution = base_constitution();
        let binding = binding_for(&constitution, &policy());
        constitution.certification_policy_digest =
            certification_policy_binding_digest(&binding).expect("valid certification binding");
        (constitution, binding)
    }

    fn artifact(
        kind: verifier::EvidenceArtifactKind,
        path: &str,
        content_digest: Digest32,
    ) -> verifier::EvidenceArtifactRefV1 {
        verifier::EvidenceArtifactRefV1 {
            kind,
            canonical_path: path.to_owned(),
            content_digest,
            byte_length: 10,
            disclosure: verifier::ArtifactDisclosureClass::PublicVerificationEvidence,
        }
    }

    fn manifest(
        constitution: &ElectionConstitutionV1,
    ) -> verifier::ElectionEvidencePackageManifestV1 {
        verifier::ElectionEvidencePackageManifestV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            evidence_package_profile_id: verifier::EVIDENCE_PACKAGE_PROFILE_ID.to_owned(),
            election_constitution_digest: policy().election_constitution_digest,
            package_canonicalization_profile_digest: policy()
                .package_canonicalization_profile_digest,
            artifact_index_digest: digest(93),
            package_root_digest: digest(94),
            final_transparency_checkpoint_digest: digest(95),
            artifacts: vec![
                artifact(
                    verifier::EvidenceArtifactKind::ElectionConstitution,
                    "election/constitution.bin",
                    policy().election_constitution_digest,
                ),
                artifact(
                    verifier::EvidenceArtifactKind::TransparencyCheckpointChain,
                    "transparency/checkpoints.bin",
                    digest(11),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::WitnessAttestations,
                    "transparency/witnesses.bin",
                    digest(12),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::AnonymousAuthorityCensus,
                    "authority/census.bin",
                    digest(13),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::TallyEvidence,
                    "tally/evidence.bin",
                    digest(14),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::PhysicalAuditEvidence,
                    "physical/audit.bin",
                    digest(15),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::ChallengeLedger,
                    "challenges/ledger.bin",
                    digest(16),
                ),
                artifact(
                    verifier::EvidenceArtifactKind::CertificationPolicy,
                    "certification/policy.bin",
                    constitution.certification_policy_digest,
                ),
            ],
            interoperability_profiles: Vec::new(),
        }
    }

    fn preflight(
        constitution: &ElectionConstitutionV1,
        policy: &resource::ElectionVerificationResourcePolicyV1,
    ) -> resource::EvidencePackagePreflightHeaderV1 {
        resource::EvidencePackagePreflightHeaderV1 {
            package_root_digest: digest(94),
            resource_policy_digest: resource_policy_commitment_digest(constitution, policy)
                .expect("valid resource commitment"),
            manifest_byte_length: 4_096,
            container_byte_length: 131_072,
        }
    }

    #[test]
    fn valid_anchor_closes_resource_policy_commitment_chain() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let evidence = validate_resource_policy_anchor(
            &constitution,
            &policy,
            &binding,
            &manifest(&constitution),
            &preflight(&constitution, &policy),
        )
        .expect("fully bound resource policy");

        assert_eq!(
            evidence.resource_policy_commitment_digest,
            binding.resource_policy_commitment_digest
        );
        assert_eq!(
            evidence.certification_policy_digest,
            constitution.certification_policy_digest
        );
        assert_eq!(
            evidence.preflight_resource_disposition,
            resource::ResourceGateDisposition::Proceed
        );
    }

    #[test]
    fn resource_commitment_is_acyclic_over_downstream_constitution_digests() {
        let constitution = base_constitution();
        let mut changed_constitution = constitution.clone();
        changed_constitution.certification_policy_digest = digest(40);

        let policy = policy();
        let mut changed_policy = policy.clone();
        changed_policy.election_constitution_digest = digest(41);

        let original = resource_policy_commitment_digest(&constitution, &policy).unwrap();
        assert_eq!(
            original,
            resource_policy_commitment_digest(&changed_constitution, &policy).unwrap()
        );
        assert_eq!(
            original,
            resource_policy_commitment_digest(&constitution, &changed_policy).unwrap()
        );
    }

    #[test]
    fn resource_commitment_changes_with_frozen_election_context_or_limits() {
        let constitution = base_constitution();
        let policy = policy();
        let original = resource_policy_commitment_digest(&constitution, &policy).unwrap();

        let mut changed_definition = constitution.clone();
        changed_definition.election_definition_digest = digest(42);
        assert_ne!(
            original,
            resource_policy_commitment_digest(&changed_definition, &policy).unwrap()
        );

        let mut changed_limits = policy.clone();
        changed_limits.limits.max_manifest_bytes += 1;
        assert_ne!(
            original,
            resource_policy_commitment_digest(&constitution, &changed_limits).unwrap()
        );
    }

    #[test]
    fn stale_preflight_resource_digest_fails_closed() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let mut subject = preflight(&constitution, &policy);
        subject.resource_policy_digest = digest(43);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &policy,
                &binding,
                &manifest(&constitution),
                &subject,
            ),
            Err(ResourcePolicyAnchorViolation::PreflightResourcePolicyDigestMismatch)
        );
    }

    #[test]
    fn stale_certification_binding_resource_digest_fails_closed() {
        let (constitution, mut binding) = finalized_constitution_and_binding();
        let policy = policy();
        binding.resource_policy_commitment_digest = digest(44);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &policy,
                &binding,
                &manifest(&constitution),
                &preflight(&constitution, &policy),
            ),
            Err(ResourcePolicyAnchorViolation::BindingResourcePolicyDigestMismatch)
        );
    }

    #[test]
    fn certification_policy_artifact_substitution_fails_closed() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let mut subject = manifest(&constitution);
        subject
            .artifacts
            .iter_mut()
            .find(|artifact| artifact.kind == verifier::EvidenceArtifactKind::CertificationPolicy)
            .unwrap()
            .content_digest = digest(45);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &policy,
                &binding,
                &subject,
                &preflight(&constitution, &policy),
            ),
            Err(ResourcePolicyAnchorViolation::CertificationPolicyArtifactDigestMismatch)
        );
    }

    #[test]
    fn constitution_artifact_substitution_fails_closed() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let mut subject = manifest(&constitution);
        subject
            .artifacts
            .iter_mut()
            .find(|artifact| artifact.kind == verifier::EvidenceArtifactKind::ElectionConstitution)
            .unwrap()
            .content_digest = digest(46);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &policy,
                &binding,
                &subject,
                &preflight(&constitution, &policy),
            ),
            Err(ResourcePolicyAnchorViolation::ElectionConstitutionArtifactDigestMismatch)
        );
    }

    #[test]
    fn policy_constitution_digest_substitution_fails_closed() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let mut subject_policy = policy();
        subject_policy.election_constitution_digest = digest(47);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &subject_policy,
                &binding,
                &manifest(&constitution),
                &preflight(&constitution, &subject_policy),
            ),
            Err(ResourcePolicyAnchorViolation::PolicyConstitutionDigestMismatch)
        );
    }

    #[test]
    fn package_canonicalization_substitution_fails_closed() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let mut subject = manifest(&constitution);
        subject.package_canonicalization_profile_digest = digest(48);
        assert_eq!(
            validate_resource_policy_anchor(
                &constitution,
                &policy,
                &binding,
                &subject,
                &preflight(&constitution, &policy),
            ),
            Err(ResourcePolicyAnchorViolation::PackageCanonicalizationDigestMismatch)
        );
    }

    #[test]
    fn certification_requirements_are_part_of_certification_policy_digest() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let original = certification_policy_binding_digest(&binding).unwrap();
        assert_eq!(original, constitution.certification_policy_digest);

        let mut changed = binding;
        changed.certification_requirements_digest = digest(49);
        assert_ne!(
            original,
            certification_policy_binding_digest(&changed).unwrap()
        );
    }

    #[test]
    fn oversized_but_structurally_valid_preflight_remains_anchorable() {
        let (constitution, binding) = finalized_constitution_and_binding();
        let policy = policy();
        let mut subject = preflight(&constitution, &policy);
        subject.manifest_byte_length = policy.limits.max_manifest_bytes + 1;
        subject.container_byte_length = subject.manifest_byte_length + 1;

        let evidence = validate_resource_policy_anchor(
            &constitution,
            &policy,
            &binding,
            &manifest(&constitution),
            &subject,
        )
        .expect("authentic policy may still exceed a resource budget");
        assert_eq!(
            evidence.preflight_resource_disposition,
            resource::ResourceGateDisposition::BlockIndeterminate(
                resource::ResourceLimitKind::ManifestBytes
            )
        );
    }
}
