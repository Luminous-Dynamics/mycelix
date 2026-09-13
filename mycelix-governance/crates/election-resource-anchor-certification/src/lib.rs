//! ELECT-015 certification consumption of independently verified resource anchoring.
//!
//! This layer does not replace the existing public-election certification transition
//! or verifier quorum. It revalidates the ordinary verifier agreement and then
//! requires an independent quorum of resource-anchor receipts bound to those same
//! passing base verifier runs.

use election_integrity_types::{
    Digest32, ElectionPhase, PUBLIC_ELECTION_PROFILE_ID, TransitionEvidence, TransitionViolation,
    validate_transition,
};
use election_resource_policy_anchor as anchor;
use election_verifier_contract as verifier;
use election_verifier_resource_envelope as resource;
use sha2::{Digest as ShaDigest, Sha256};

pub const RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID: &str =
    "mycelix-public-election-resource-anchor-certification-v1";
pub const RESOURCE_ANCHOR_CERTIFICATION_HASH_ID: &str = "sha-256";
pub const MIN_RESOURCE_ANCHOR_VERIFIERS: u16 = 3;
pub const MIN_RESOURCE_ANCHOR_IMPLEMENTATION_LINEAGES: u16 = 3;
pub const MIN_RESOURCE_ANCHOR_BUILDER_DOMAINS: u16 = 2;

const ANCHOR_EVIDENCE_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:RESOURCE-ANCHOR-EVIDENCE:V1\0";
const ANCHOR_VERIFIER_RECEIPT_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:RESOURCE-ANCHOR-VERIFIER-RECEIPT:V1\0";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResourceAnchorCertificationPolicyV1 {
    pub public_election_profile_id: String,
    pub resource_policy_anchor_profile_id: String,
    pub resource_anchor_certification_profile_id: String,
    pub verifier_agreement_policy: verifier::VerifierAgreementPolicyV1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceAnchorCertificationPolicyViolation {
    WrongPublicElectionProfile,
    WrongResourcePolicyAnchorProfile,
    WrongResourceAnchorCertificationProfile,
    TooFewVerifiers,
    TooFewImplementationLineages,
    TooFewBuilderControlDomains,
    ImplementationLineagesExceedTotalVerifiers,
    BuilderControlDomainsExceedTotalVerifiers,
}

pub fn validate_resource_anchor_certification_policy(
    policy: &ResourceAnchorCertificationPolicyV1,
) -> Result<(), ResourceAnchorCertificationPolicyViolation> {
    if policy.public_election_profile_id != PUBLIC_ELECTION_PROFILE_ID {
        return Err(ResourceAnchorCertificationPolicyViolation::WrongPublicElectionProfile);
    }
    if policy.resource_policy_anchor_profile_id != anchor::RESOURCE_POLICY_ANCHOR_PROFILE_ID {
        return Err(ResourceAnchorCertificationPolicyViolation::WrongResourcePolicyAnchorProfile);
    }
    if policy.resource_anchor_certification_profile_id != RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID {
        return Err(
            ResourceAnchorCertificationPolicyViolation::WrongResourceAnchorCertificationProfile,
        );
    }
    let agreement = &policy.verifier_agreement_policy;
    if agreement.minimum_total_verifiers < MIN_RESOURCE_ANCHOR_VERIFIERS {
        return Err(ResourceAnchorCertificationPolicyViolation::TooFewVerifiers);
    }
    if agreement.minimum_distinct_implementation_lineages
        < MIN_RESOURCE_ANCHOR_IMPLEMENTATION_LINEAGES
    {
        return Err(ResourceAnchorCertificationPolicyViolation::TooFewImplementationLineages);
    }
    if agreement.minimum_distinct_builder_control_domains < MIN_RESOURCE_ANCHOR_BUILDER_DOMAINS {
        return Err(ResourceAnchorCertificationPolicyViolation::TooFewBuilderControlDomains);
    }
    if agreement.minimum_distinct_implementation_lineages > agreement.minimum_total_verifiers {
        return Err(
            ResourceAnchorCertificationPolicyViolation::ImplementationLineagesExceedTotalVerifiers,
        );
    }
    if agreement.minimum_distinct_builder_control_domains > agreement.minimum_total_verifiers {
        return Err(
            ResourceAnchorCertificationPolicyViolation::BuilderControlDomainsExceedTotalVerifiers,
        );
    }
    Ok(())
}

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let output = hasher.finalize();
    let mut digest = [0_u8; 32];
    digest.copy_from_slice(&output);
    digest
}

fn resource_limit_kind_tag(kind: resource::ResourceLimitKind) -> u8 {
    match kind {
        resource::ResourceLimitKind::Artifacts => 0,
        resource::ResourceLimitKind::SingleArtifactBytes => 1,
        resource::ResourceLimitKind::TotalArtifactBytes => 2,
        resource::ResourceLimitKind::CanonicalPathBytes => 3,
        resource::ResourceLimitKind::InteroperabilityProfiles => 4,
        resource::ResourceLimitKind::ManifestBytes => 5,
        resource::ResourceLimitKind::ContainerBytes => 6,
        resource::ResourceLimitKind::ContainerEntries => 7,
        resource::ResourceLimitKind::ExpandedBytes => 8,
        resource::ResourceLimitKind::NestedContainerDepth => 9,
    }
}

fn append_resource_gate_disposition(
    bytes: &mut Vec<u8>,
    disposition: resource::ResourceGateDisposition,
) {
    match disposition {
        resource::ResourceGateDisposition::Proceed => {
            bytes.push(0);
            bytes.push(0);
        }
        resource::ResourceGateDisposition::BlockIndeterminate(kind) => {
            bytes.push(1);
            bytes.push(resource_limit_kind_tag(kind));
        }
    }
}

pub fn resource_anchor_evidence_digest(
    evidence: &anchor::ResourcePolicyAnchorEvidenceV1,
) -> Digest32 {
    let mut bytes = Vec::with_capacity(ANCHOR_EVIDENCE_DOMAIN.len() + 6 * 32 + 2);
    bytes.extend_from_slice(ANCHOR_EVIDENCE_DOMAIN);
    bytes.extend_from_slice(&evidence.package_root_digest);
    bytes.extend_from_slice(&evidence.election_constitution_digest);
    bytes.extend_from_slice(&evidence.election_definition_digest);
    bytes.extend_from_slice(&evidence.resource_policy_commitment_digest);
    bytes.extend_from_slice(&evidence.certification_policy_digest);
    bytes.extend_from_slice(&evidence.certification_requirements_digest);
    append_resource_gate_disposition(&mut bytes, evidence.preflight_resource_disposition);
    sha256(&bytes)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ResourceAnchorVerifierReceiptV1 {
    pub resource_anchor_certification_profile_id: String,
    pub package_root_digest: Digest32,
    pub election_constitution_digest: Digest32,
    pub resource_policy_commitment_digest: Digest32,
    pub certification_policy_digest: Digest32,
    pub anchor_evidence_digest: Digest32,
    pub verifier_lineage_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub builder_control_domain_digest: Digest32,
    pub base_verifier_run_receipt_digest: Digest32,
    pub anchor_disposition: verifier::VerificationStageDisposition,
    pub resource_verification_disposition: verifier::VerificationStageDisposition,
    pub finding_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceAnchorVerifierReceiptViolation {
    WrongCertificationProfile,
    ZeroPackageRootDigest,
    ZeroElectionConstitutionDigest,
    ZeroResourcePolicyCommitmentDigest,
    ZeroCertificationPolicyDigest,
    ZeroAnchorEvidenceDigest,
    ZeroVerifierLineageDigest,
    ZeroVerifierReleaseDigest,
    ZeroBuilderControlDomainDigest,
    ZeroBaseVerifierRunReceiptDigest,
    ZeroFindingDigest,
}

pub fn validate_resource_anchor_verifier_receipt(
    receipt: &ResourceAnchorVerifierReceiptV1,
) -> Result<(), ResourceAnchorVerifierReceiptViolation> {
    let zero = [0_u8; 32];
    if receipt.resource_anchor_certification_profile_id != RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID
    {
        return Err(ResourceAnchorVerifierReceiptViolation::WrongCertificationProfile);
    }
    if receipt.package_root_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroPackageRootDigest);
    }
    if receipt.election_constitution_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroElectionConstitutionDigest);
    }
    if receipt.resource_policy_commitment_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroResourcePolicyCommitmentDigest);
    }
    if receipt.certification_policy_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroCertificationPolicyDigest);
    }
    if receipt.anchor_evidence_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroAnchorEvidenceDigest);
    }
    if receipt.verifier_lineage_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroVerifierLineageDigest);
    }
    if receipt.verifier_release_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroVerifierReleaseDigest);
    }
    if receipt.builder_control_domain_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroBuilderControlDomainDigest);
    }
    if receipt.base_verifier_run_receipt_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroBaseVerifierRunReceiptDigest);
    }
    if receipt.finding_digest == zero {
        return Err(ResourceAnchorVerifierReceiptViolation::ZeroFindingDigest);
    }
    Ok(())
}

fn verification_disposition_tag(disposition: verifier::VerificationStageDisposition) -> u8 {
    match disposition {
        verifier::VerificationStageDisposition::Pass => 0,
        verifier::VerificationStageDisposition::Fail => 1,
        verifier::VerificationStageDisposition::Indeterminate => 2,
    }
}

pub fn resource_anchor_verifier_receipt_digest(
    receipt: &ResourceAnchorVerifierReceiptV1,
) -> Result<Digest32, ResourceAnchorVerifierReceiptViolation> {
    validate_resource_anchor_verifier_receipt(receipt)?;
    let mut bytes = Vec::with_capacity(ANCHOR_VERIFIER_RECEIPT_DOMAIN.len() + 10 * 32 + 2);
    bytes.extend_from_slice(ANCHOR_VERIFIER_RECEIPT_DOMAIN);
    bytes.extend_from_slice(&receipt.package_root_digest);
    bytes.extend_from_slice(&receipt.election_constitution_digest);
    bytes.extend_from_slice(&receipt.resource_policy_commitment_digest);
    bytes.extend_from_slice(&receipt.certification_policy_digest);
    bytes.extend_from_slice(&receipt.anchor_evidence_digest);
    bytes.extend_from_slice(&receipt.verifier_lineage_digest);
    bytes.extend_from_slice(&receipt.verifier_release_digest);
    bytes.extend_from_slice(&receipt.builder_control_domain_digest);
    bytes.extend_from_slice(&receipt.base_verifier_run_receipt_digest);
    bytes.extend_from_slice(&receipt.finding_digest);
    bytes.push(verification_disposition_tag(receipt.anchor_disposition));
    bytes.push(verification_disposition_tag(
        receipt.resource_verification_disposition,
    ));
    Ok(sha256(&bytes))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResourceAnchorCertificationEvidenceV1 {
    pub package_root_digest: Digest32,
    pub election_constitution_digest: Digest32,
    pub resource_policy_commitment_digest: Digest32,
    pub certification_policy_digest: Digest32,
    pub anchor_evidence_digest: Digest32,
    pub base_verifier_count: usize,
    pub anchor_verifier_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ResourceAnchorCertificationViolation {
    Policy(ResourceAnchorCertificationPolicyViolation),
    BaseCertificationTransition(TransitionViolation),
    BaseVerifierAgreement(verifier::VerifierAgreementViolation),
    ZeroAnchorEvidencePackageRoot,
    ZeroAnchorEvidenceConstitutionDigest,
    ZeroAnchorEvidenceResourcePolicyDigest,
    ZeroAnchorEvidenceCertificationPolicyDigest,
    AnchorPreflightResourceGateNotProceed,
    Receipt {
        index: usize,
        violation: ResourceAnchorVerifierReceiptViolation,
    },
    ReceiptPackageRootMismatch(usize),
    ReceiptConstitutionDigestMismatch(usize),
    ReceiptResourcePolicyDigestMismatch(usize),
    ReceiptCertificationPolicyDigestMismatch(usize),
    ReceiptAnchorEvidenceDigestMismatch(usize),
    ReceiptAnchorDispositionNotPass(usize),
    ReceiptResourceVerificationDispositionNotPass(usize),
    ReceiptNotBoundToPassingBaseVerifierRun(usize),
    ReceiptDigest(ResourceAnchorVerifierReceiptViolation),
    AnchorVerifierAgreement(verifier::VerifierAgreementViolation),
}

fn receipt_matches_base_attestation(
    receipt: &ResourceAnchorVerifierReceiptV1,
    attestation: &verifier::VerifierAgreementAttestationV1,
) -> bool {
    attestation.package_root_digest == receipt.package_root_digest
        && attestation.verifier_lineage_digest == receipt.verifier_lineage_digest
        && attestation.verifier_release_digest == receipt.verifier_release_digest
        && attestation.builder_control_domain_digest == receipt.builder_control_domain_digest
        && attestation.run_receipt_digest == receipt.base_verifier_run_receipt_digest
        && attestation.run_disposition == verifier::VerificationStageDisposition::Pass
}

pub fn validate_resource_anchor_certification(
    transition_evidence: &TransitionEvidence,
    policy: &ResourceAnchorCertificationPolicyV1,
    anchor_evidence: &anchor::ResourcePolicyAnchorEvidenceV1,
    base_verifier_attestations: &[verifier::VerifierAgreementAttestationV1],
    anchor_receipts: &[ResourceAnchorVerifierReceiptV1],
) -> Result<ResourceAnchorCertificationEvidenceV1, ResourceAnchorCertificationViolation> {
    validate_resource_anchor_certification_policy(policy)
        .map_err(ResourceAnchorCertificationViolation::Policy)?;
    validate_transition(
        ElectionPhase::ChallengeWindow,
        ElectionPhase::Certified,
        transition_evidence,
    )
    .map_err(ResourceAnchorCertificationViolation::BaseCertificationTransition)?;

    verifier::validate_independent_verifier_agreement(
        anchor_evidence.package_root_digest,
        &policy.verifier_agreement_policy,
        base_verifier_attestations,
    )
    .map_err(ResourceAnchorCertificationViolation::BaseVerifierAgreement)?;

    let zero = [0_u8; 32];
    if anchor_evidence.package_root_digest == zero {
        return Err(ResourceAnchorCertificationViolation::ZeroAnchorEvidencePackageRoot);
    }
    if anchor_evidence.election_constitution_digest == zero {
        return Err(ResourceAnchorCertificationViolation::ZeroAnchorEvidenceConstitutionDigest);
    }
    if anchor_evidence.resource_policy_commitment_digest == zero {
        return Err(ResourceAnchorCertificationViolation::ZeroAnchorEvidenceResourcePolicyDigest);
    }
    if anchor_evidence.certification_policy_digest == zero {
        return Err(
            ResourceAnchorCertificationViolation::ZeroAnchorEvidenceCertificationPolicyDigest,
        );
    }
    if anchor_evidence.preflight_resource_disposition != resource::ResourceGateDisposition::Proceed
    {
        return Err(ResourceAnchorCertificationViolation::AnchorPreflightResourceGateNotProceed);
    }

    let expected_anchor_evidence_digest = resource_anchor_evidence_digest(anchor_evidence);
    let mut anchor_attestations = Vec::with_capacity(anchor_receipts.len());

    for (index, receipt) in anchor_receipts.iter().enumerate() {
        validate_resource_anchor_verifier_receipt(receipt).map_err(|violation| {
            ResourceAnchorCertificationViolation::Receipt { index, violation }
        })?;
        if receipt.package_root_digest != anchor_evidence.package_root_digest {
            return Err(ResourceAnchorCertificationViolation::ReceiptPackageRootMismatch(index));
        }
        if receipt.election_constitution_digest != anchor_evidence.election_constitution_digest {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptConstitutionDigestMismatch(index),
            );
        }
        if receipt.resource_policy_commitment_digest
            != anchor_evidence.resource_policy_commitment_digest
        {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptResourcePolicyDigestMismatch(index),
            );
        }
        if receipt.certification_policy_digest != anchor_evidence.certification_policy_digest {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptCertificationPolicyDigestMismatch(
                    index,
                ),
            );
        }
        if receipt.anchor_evidence_digest != expected_anchor_evidence_digest {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptAnchorEvidenceDigestMismatch(index),
            );
        }
        if receipt.anchor_disposition != verifier::VerificationStageDisposition::Pass {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptAnchorDispositionNotPass(index),
            );
        }
        if receipt.resource_verification_disposition != verifier::VerificationStageDisposition::Pass
        {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptResourceVerificationDispositionNotPass(
                    index,
                ),
            );
        }
        if !base_verifier_attestations
            .iter()
            .any(|attestation| receipt_matches_base_attestation(receipt, attestation))
        {
            return Err(
                ResourceAnchorCertificationViolation::ReceiptNotBoundToPassingBaseVerifierRun(
                    index,
                ),
            );
        }

        let run_receipt_digest = resource_anchor_verifier_receipt_digest(receipt)
            .map_err(ResourceAnchorCertificationViolation::ReceiptDigest)?;
        anchor_attestations.push(verifier::VerifierAgreementAttestationV1 {
            package_root_digest: receipt.package_root_digest,
            verifier_lineage_digest: receipt.verifier_lineage_digest,
            verifier_release_digest: receipt.verifier_release_digest,
            builder_control_domain_digest: receipt.builder_control_domain_digest,
            run_receipt_digest,
            run_disposition: verifier::VerificationStageDisposition::Pass,
        });
    }

    verifier::validate_independent_verifier_agreement(
        anchor_evidence.package_root_digest,
        &policy.verifier_agreement_policy,
        &anchor_attestations,
    )
    .map_err(ResourceAnchorCertificationViolation::AnchorVerifierAgreement)?;

    Ok(ResourceAnchorCertificationEvidenceV1 {
        package_root_digest: anchor_evidence.package_root_digest,
        election_constitution_digest: anchor_evidence.election_constitution_digest,
        resource_policy_commitment_digest: anchor_evidence.resource_policy_commitment_digest,
        certification_policy_digest: anchor_evidence.certification_policy_digest,
        anchor_evidence_digest: expected_anchor_evidence_digest,
        base_verifier_count: base_verifier_attestations.len(),
        anchor_verifier_count: anchor_receipts.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn policy() -> ResourceAnchorCertificationPolicyV1 {
        ResourceAnchorCertificationPolicyV1 {
            public_election_profile_id: PUBLIC_ELECTION_PROFILE_ID.to_owned(),
            resource_policy_anchor_profile_id: anchor::RESOURCE_POLICY_ANCHOR_PROFILE_ID.to_owned(),
            resource_anchor_certification_profile_id: RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID
                .to_owned(),
            verifier_agreement_policy: verifier::VerifierAgreementPolicyV1 {
                minimum_total_verifiers: 3,
                minimum_distinct_implementation_lineages: 3,
                minimum_distinct_builder_control_domains: 2,
            },
        }
    }

    fn transition_evidence() -> TransitionEvidence {
        TransitionEvidence {
            unresolved_qualifying_challenges: false,
            independent_verifier_quorum_passed: true,
            certification_evidence_complete: true,
            ..Default::default()
        }
    }

    fn anchor_evidence() -> anchor::ResourcePolicyAnchorEvidenceV1 {
        anchor::ResourcePolicyAnchorEvidenceV1 {
            package_root_digest: digest(4),
            election_constitution_digest: digest(5),
            election_definition_digest: digest(6),
            resource_policy_commitment_digest: digest(7),
            certification_policy_digest: digest(8),
            certification_requirements_digest: digest(9),
            preflight_resource_disposition: resource::ResourceGateDisposition::Proceed,
        }
    }

    fn base_attestation(
        release: u8,
        lineage: u8,
        builder: u8,
    ) -> verifier::VerifierAgreementAttestationV1 {
        verifier::VerifierAgreementAttestationV1 {
            package_root_digest: digest(4),
            verifier_lineage_digest: digest(lineage),
            verifier_release_digest: digest(release),
            builder_control_domain_digest: digest(builder),
            run_receipt_digest: digest(release + 40),
            run_disposition: verifier::VerificationStageDisposition::Pass,
        }
    }

    fn base_attestations() -> Vec<verifier::VerifierAgreementAttestationV1> {
        vec![
            base_attestation(10, 20, 30),
            base_attestation(11, 21, 31),
            base_attestation(12, 22, 30),
        ]
    }

    fn anchor_receipt(
        release: u8,
        lineage: u8,
        builder: u8,
        anchor_evidence: &anchor::ResourcePolicyAnchorEvidenceV1,
    ) -> ResourceAnchorVerifierReceiptV1 {
        ResourceAnchorVerifierReceiptV1 {
            resource_anchor_certification_profile_id: RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID
                .to_owned(),
            package_root_digest: anchor_evidence.package_root_digest,
            election_constitution_digest: anchor_evidence.election_constitution_digest,
            resource_policy_commitment_digest: anchor_evidence.resource_policy_commitment_digest,
            certification_policy_digest: anchor_evidence.certification_policy_digest,
            anchor_evidence_digest: resource_anchor_evidence_digest(anchor_evidence),
            verifier_lineage_digest: digest(lineage),
            verifier_release_digest: digest(release),
            builder_control_domain_digest: digest(builder),
            base_verifier_run_receipt_digest: digest(release + 40),
            anchor_disposition: verifier::VerificationStageDisposition::Pass,
            resource_verification_disposition: verifier::VerificationStageDisposition::Pass,
            finding_digest: digest(release + 60),
        }
    }

    fn anchor_receipts(
        anchor_evidence: &anchor::ResourcePolicyAnchorEvidenceV1,
    ) -> Vec<ResourceAnchorVerifierReceiptV1> {
        vec![
            anchor_receipt(10, 20, 30, anchor_evidence),
            anchor_receipt(11, 21, 31, anchor_evidence),
            anchor_receipt(12, 22, 30, anchor_evidence),
        ]
    }

    #[test]
    fn certification_requires_both_base_and_anchor_quorums() {
        let anchor_evidence = anchor_evidence();
        let result = validate_resource_anchor_certification(
            &transition_evidence(),
            &policy(),
            &anchor_evidence,
            &base_attestations(),
            &anchor_receipts(&anchor_evidence),
        )
        .expect("base and anchor quorums pass");
        assert_eq!(result.base_verifier_count, 3);
        assert_eq!(result.anchor_verifier_count, 3);
        assert_eq!(
            result.anchor_evidence_digest,
            resource_anchor_evidence_digest(&anchor_evidence)
        );
    }

    #[test]
    fn weak_anchor_quorum_policy_is_rejected() {
        let mut subject = policy();
        subject.verifier_agreement_policy.minimum_total_verifiers = 2;
        assert_eq!(
            validate_resource_anchor_certification_policy(&subject),
            Err(ResourceAnchorCertificationPolicyViolation::TooFewVerifiers)
        );
    }

    #[test]
    fn unresolved_challenge_still_blocks_certification() {
        let anchor_evidence = anchor_evidence();
        let mut transition = transition_evidence();
        transition.unresolved_qualifying_challenges = true;
        assert_eq!(
            validate_resource_anchor_certification(
                &transition,
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &anchor_receipts(&anchor_evidence),
            ),
            Err(
                ResourceAnchorCertificationViolation::BaseCertificationTransition(
                    TransitionViolation::UnresolvedQualifyingChallenges
                )
            )
        );
    }

    #[test]
    fn base_verifier_quorum_is_revalidated_not_trusted_as_boolean() {
        let anchor_evidence = anchor_evidence();
        let mut base = base_attestations();
        base[0].run_disposition = verifier::VerificationStageDisposition::Fail;
        assert!(matches!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base,
                &anchor_receipts(&anchor_evidence),
            ),
            Err(ResourceAnchorCertificationViolation::BaseVerifierAgreement(
                verifier::VerifierAgreementViolation::NonPassingVerifier
            ))
        ));
    }

    #[test]
    fn anchor_receipt_must_bind_to_same_passing_base_run() {
        let anchor_evidence = anchor_evidence();
        let mut receipts = anchor_receipts(&anchor_evidence);
        receipts[0].base_verifier_run_receipt_digest = digest(99);
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &receipts,
            ),
            Err(ResourceAnchorCertificationViolation::ReceiptNotBoundToPassingBaseVerifierRun(0))
        );
    }

    #[test]
    fn stale_anchor_evidence_digest_fails_closed() {
        let anchor_evidence = anchor_evidence();
        let mut receipts = anchor_receipts(&anchor_evidence);
        receipts[0].anchor_evidence_digest = digest(98);
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &receipts,
            ),
            Err(ResourceAnchorCertificationViolation::ReceiptAnchorEvidenceDigestMismatch(0))
        );
    }

    #[test]
    fn resource_policy_subject_substitution_fails_closed() {
        let anchor_evidence = anchor_evidence();
        let mut receipts = anchor_receipts(&anchor_evidence);
        receipts[1].resource_policy_commitment_digest = digest(97);
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &receipts,
            ),
            Err(ResourceAnchorCertificationViolation::ReceiptResourcePolicyDigestMismatch(1))
        );
    }

    #[test]
    fn nonpassing_anchor_or_resource_verification_blocks_certification() {
        let anchor_evidence = anchor_evidence();
        let mut anchor_failed = anchor_receipts(&anchor_evidence);
        anchor_failed[0].anchor_disposition = verifier::VerificationStageDisposition::Fail;
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &anchor_failed,
            ),
            Err(ResourceAnchorCertificationViolation::ReceiptAnchorDispositionNotPass(0))
        );

        let mut resource_indeterminate = anchor_receipts(&anchor_evidence);
        resource_indeterminate[0].resource_verification_disposition =
            verifier::VerificationStageDisposition::Indeterminate;
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &resource_indeterminate,
            ),
            Err(
                ResourceAnchorCertificationViolation::ReceiptResourceVerificationDispositionNotPass(
                    0
                )
            )
        );
    }

    #[test]
    fn indeterminate_preflight_cannot_be_certified() {
        let mut anchor_evidence = anchor_evidence();
        anchor_evidence.preflight_resource_disposition =
            resource::ResourceGateDisposition::BlockIndeterminate(
                resource::ResourceLimitKind::ManifestBytes,
            );
        assert_eq!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &anchor_receipts(&anchor_evidence),
            ),
            Err(ResourceAnchorCertificationViolation::AnchorPreflightResourceGateNotProceed)
        );
    }

    #[test]
    fn anchor_evidence_digest_commits_to_resource_disposition() {
        let passing = anchor_evidence();
        let mut blocked = passing;
        blocked.preflight_resource_disposition =
            resource::ResourceGateDisposition::BlockIndeterminate(
                resource::ResourceLimitKind::ContainerBytes,
            );
        assert_ne!(
            resource_anchor_evidence_digest(&passing),
            resource_anchor_evidence_digest(&blocked)
        );
    }

    #[test]
    fn duplicate_anchor_verifier_release_fails_independence_quorum() {
        let anchor_evidence = anchor_evidence();
        let mut receipts = anchor_receipts(&anchor_evidence);
        receipts[1].verifier_release_digest = receipts[0].verifier_release_digest;
        receipts[1].base_verifier_run_receipt_digest = receipts[0].base_verifier_run_receipt_digest;
        receipts[1].verifier_lineage_digest = receipts[0].verifier_lineage_digest;
        receipts[1].builder_control_domain_digest = receipts[0].builder_control_domain_digest;
        assert!(matches!(
            validate_resource_anchor_certification(
                &transition_evidence(),
                &policy(),
                &anchor_evidence,
                &base_attestations(),
                &receipts,
            ),
            Err(
                ResourceAnchorCertificationViolation::AnchorVerifierAgreement(
                    verifier::VerifierAgreementViolation::DuplicateVerifierRelease
                )
            ) | Err(
                ResourceAnchorCertificationViolation::ReceiptNotBoundToPassingBaseVerifierRun(1)
            )
        ));
    }
}
