//! ELECT-016 canonical verifier receipt/provenance envelopes for Mycelix public elections.
//!
//! Mycelix owns the election receipt semantics and language-neutral byte transcripts.
//! A later Xenia adapter may authenticate the resulting semantic envelope digest
//! without redefining election/certification meaning in the signature layer.

use election_resource_anchor_certification as certification;
use election_verifier_contract as verifier;
use sha2::{Digest as ShaDigest, Sha256};

pub type Digest32 = [u8; 32];

pub const VERIFIER_RECEIPT_ENVELOPE_PROFILE_ID: &str =
    "mycelix-public-election-verifier-receipt-envelope-v1";
pub const VERIFIER_PROVENANCE_PROFILE_ID: &str = "mycelix-public-election-verifier-provenance-v1";
pub const RECEIPT_ENVELOPE_HASH_ID: &str = "sha-256";
pub const MAX_CANONICAL_STRING_BYTES: usize = 256;

const BASE_RUN_RECEIPT_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-RUN-RECEIPT:V1\0";
const RESOURCE_ANCHOR_RECEIPT_DOMAIN: &[u8] =
    b"MYCELIX:PUBLIC-ELECTION:RESOURCE-ANCHOR-VERIFIER-RECEIPT:V1\0";
const VERIFIER_PROVENANCE_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-PROVENANCE:V1\0";
const RECEIPT_ENVELOPE_DOMAIN: &[u8] = b"MYCELIX:PUBLIC-ELECTION:VERIFIER-RECEIPT-ENVELOPE:V1\0";

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CanonicalReceiptEncodingViolation {
    BaseRun(verifier::VerifierRunViolation),
    BaseRunNotPassing(verifier::VerifierRunDisposition),
    CanonicalStringTooLong,
    TooManyStages,
    AnchorReceipt(certification::ResourceAnchorVerifierReceiptViolation),
    AnchorDispositionNotPass,
    ResourceVerificationDispositionNotPass,
    AnchorCanonicalDigestParityMismatch,
}

fn sha256(bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let output = hasher.finalize();
    let mut digest = [0_u8; 32];
    digest.copy_from_slice(&output);
    digest
}

fn append_len_prefixed_utf8(
    bytes: &mut Vec<u8>,
    value: &str,
) -> Result<(), CanonicalReceiptEncodingViolation> {
    let raw = value.as_bytes();
    if raw.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(CanonicalReceiptEncodingViolation::CanonicalStringTooLong);
    }
    let length = u32::try_from(raw.len())
        .map_err(|_| CanonicalReceiptEncodingViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(raw);
    Ok(())
}

fn stage_tag(stage: verifier::VerificationStageId) -> u8 {
    match stage {
        verifier::VerificationStageId::PackageIntegrity => 0,
        verifier::VerificationStageId::ElectionConstitution => 1,
        verifier::VerificationStageId::TransparencyLineage => 2,
        verifier::VerificationStageId::WitnessQuorum => 3,
        verifier::VerificationStageId::AnonymousAuthorityCensus => 4,
        verifier::VerificationStageId::TallyEvidence => 5,
        verifier::VerificationStageId::PhysicalAudit => 6,
        verifier::VerificationStageId::ChallengeLedger => 7,
        verifier::VerificationStageId::CertificationEvidence => 8,
    }
}

fn disposition_tag(disposition: verifier::VerificationStageDisposition) -> u8 {
    match disposition {
        verifier::VerificationStageDisposition::Pass => 0,
        verifier::VerificationStageDisposition::Fail => 1,
        verifier::VerificationStageDisposition::Indeterminate => 2,
    }
}

pub fn canonical_verifier_run_receipt_bytes(
    receipt: &verifier::VerifierRunReceiptV1,
) -> Result<Vec<u8>, CanonicalReceiptEncodingViolation> {
    let disposition = verifier::classify_verifier_run(receipt)
        .map_err(CanonicalReceiptEncodingViolation::BaseRun)?;
    if disposition != verifier::VerifierRunDisposition::AllRequiredStagesPass {
        return Err(CanonicalReceiptEncodingViolation::BaseRunNotPassing(
            disposition,
        ));
    }

    let mut stages: Vec<_> = receipt.stages.iter().collect();
    stages.sort_by_key(|stage| stage.stage);
    let stage_count = u16::try_from(stages.len())
        .map_err(|_| CanonicalReceiptEncodingViolation::TooManyStages)?;

    let mut bytes = Vec::with_capacity(1_600);
    bytes.extend_from_slice(BASE_RUN_RECEIPT_DOMAIN);
    append_len_prefixed_utf8(&mut bytes, &receipt.offline_verifier_profile_id)?;
    bytes.extend_from_slice(&receipt.package_root_digest);
    append_len_prefixed_utf8(&mut bytes, &receipt.verifier_implementation_id)?;
    bytes.extend_from_slice(&receipt.verifier_lineage_digest);
    bytes.extend_from_slice(&receipt.verifier_release_digest);
    bytes.extend_from_slice(&receipt.source_digest);
    bytes.extend_from_slice(&receipt.build_provenance_digest);
    bytes.extend_from_slice(&receipt.execution_policy_digest);
    bytes.extend_from_slice(&stage_count.to_be_bytes());

    for stage in stages {
        bytes.push(stage_tag(stage.stage));
        bytes.extend_from_slice(&stage.package_root_digest);
        bytes.extend_from_slice(&stage.subject_digest);
        bytes.extend_from_slice(&stage.verifier_release_digest);
        bytes.push(disposition_tag(stage.disposition));
        bytes.extend_from_slice(&stage.finding_digest);
    }

    Ok(bytes)
}

pub fn canonical_verifier_run_receipt_digest(
    receipt: &verifier::VerifierRunReceiptV1,
) -> Result<Digest32, CanonicalReceiptEncodingViolation> {
    Ok(sha256(&canonical_verifier_run_receipt_bytes(receipt)?))
}

pub fn canonical_resource_anchor_receipt_bytes(
    receipt: &certification::ResourceAnchorVerifierReceiptV1,
) -> Result<Vec<u8>, CanonicalReceiptEncodingViolation> {
    certification::validate_resource_anchor_verifier_receipt(receipt)
        .map_err(CanonicalReceiptEncodingViolation::AnchorReceipt)?;
    if receipt.anchor_disposition != verifier::VerificationStageDisposition::Pass {
        return Err(CanonicalReceiptEncodingViolation::AnchorDispositionNotPass);
    }
    if receipt.resource_verification_disposition != verifier::VerificationStageDisposition::Pass {
        return Err(CanonicalReceiptEncodingViolation::ResourceVerificationDispositionNotPass);
    }

    let mut bytes = Vec::with_capacity(420);
    bytes.extend_from_slice(RESOURCE_ANCHOR_RECEIPT_DOMAIN);
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
    bytes.push(disposition_tag(receipt.anchor_disposition));
    bytes.push(disposition_tag(receipt.resource_verification_disposition));
    Ok(bytes)
}

pub fn canonical_resource_anchor_receipt_digest(
    receipt: &certification::ResourceAnchorVerifierReceiptV1,
) -> Result<Digest32, CanonicalReceiptEncodingViolation> {
    let digest = sha256(&canonical_resource_anchor_receipt_bytes(receipt)?);
    let existing = certification::resource_anchor_verifier_receipt_digest(receipt)
        .map_err(CanonicalReceiptEncodingViolation::AnchorReceipt)?;
    if digest != existing {
        return Err(CanonicalReceiptEncodingViolation::AnchorCanonicalDigestParityMismatch);
    }
    Ok(digest)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifierExecutionProvenanceV1 {
    pub verifier_provenance_profile_id: String,
    pub package_root_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub source_digest: Digest32,
    pub build_provenance_digest: Digest32,
    pub execution_policy_digest: Digest32,
    pub compiler_toolchain_digest: Digest32,
    pub dependency_lock_digest: Digest32,
    pub build_recipe_digest: Digest32,
    pub target_platform_digest: Digest32,
    pub builder_control_domain_digest: Digest32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifierExecutionProvenanceViolation {
    WrongProfile,
    ZeroPackageRootDigest,
    ZeroVerifierReleaseDigest,
    ZeroSourceDigest,
    ZeroBuildProvenanceDigest,
    ZeroExecutionPolicyDigest,
    ZeroCompilerToolchainDigest,
    ZeroDependencyLockDigest,
    ZeroBuildRecipeDigest,
    ZeroTargetPlatformDigest,
    ZeroBuilderControlDomainDigest,
    CanonicalStringTooLong,
}

pub fn validate_verifier_execution_provenance(
    provenance: &VerifierExecutionProvenanceV1,
) -> Result<(), VerifierExecutionProvenanceViolation> {
    let zero = [0_u8; 32];
    if provenance.verifier_provenance_profile_id != VERIFIER_PROVENANCE_PROFILE_ID {
        return Err(VerifierExecutionProvenanceViolation::WrongProfile);
    }
    if provenance.verifier_provenance_profile_id.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(VerifierExecutionProvenanceViolation::CanonicalStringTooLong);
    }
    if provenance.package_root_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroPackageRootDigest);
    }
    if provenance.verifier_release_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroVerifierReleaseDigest);
    }
    if provenance.source_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroSourceDigest);
    }
    if provenance.build_provenance_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroBuildProvenanceDigest);
    }
    if provenance.execution_policy_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroExecutionPolicyDigest);
    }
    if provenance.compiler_toolchain_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroCompilerToolchainDigest);
    }
    if provenance.dependency_lock_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroDependencyLockDigest);
    }
    if provenance.build_recipe_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroBuildRecipeDigest);
    }
    if provenance.target_platform_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroTargetPlatformDigest);
    }
    if provenance.builder_control_domain_digest == zero {
        return Err(VerifierExecutionProvenanceViolation::ZeroBuilderControlDomainDigest);
    }
    Ok(())
}

pub fn canonical_verifier_execution_provenance_bytes(
    provenance: &VerifierExecutionProvenanceV1,
) -> Result<Vec<u8>, VerifierExecutionProvenanceViolation> {
    validate_verifier_execution_provenance(provenance)?;
    let mut bytes = Vec::with_capacity(460);
    bytes.extend_from_slice(VERIFIER_PROVENANCE_DOMAIN);
    let profile_bytes = provenance.verifier_provenance_profile_id.as_bytes();
    let profile_length = u32::try_from(profile_bytes.len())
        .map_err(|_| VerifierExecutionProvenanceViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&profile_length.to_be_bytes());
    bytes.extend_from_slice(profile_bytes);
    bytes.extend_from_slice(&provenance.package_root_digest);
    bytes.extend_from_slice(&provenance.verifier_release_digest);
    bytes.extend_from_slice(&provenance.source_digest);
    bytes.extend_from_slice(&provenance.build_provenance_digest);
    bytes.extend_from_slice(&provenance.execution_policy_digest);
    bytes.extend_from_slice(&provenance.compiler_toolchain_digest);
    bytes.extend_from_slice(&provenance.dependency_lock_digest);
    bytes.extend_from_slice(&provenance.build_recipe_digest);
    bytes.extend_from_slice(&provenance.target_platform_digest);
    bytes.extend_from_slice(&provenance.builder_control_domain_digest);
    Ok(bytes)
}

pub fn canonical_verifier_execution_provenance_digest(
    provenance: &VerifierExecutionProvenanceV1,
) -> Result<Digest32, VerifierExecutionProvenanceViolation> {
    Ok(sha256(&canonical_verifier_execution_provenance_bytes(
        provenance,
    )?))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalVerifierReceiptEnvelopeV1 {
    pub verifier_receipt_envelope_profile_id: String,
    pub package_root_digest: Digest32,
    pub verifier_lineage_digest: Digest32,
    pub verifier_release_digest: Digest32,
    pub builder_control_domain_digest: Digest32,
    pub base_verifier_run_receipt_digest: Digest32,
    pub resource_anchor_receipt_digest: Digest32,
    pub verifier_provenance_digest: Digest32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VerifierReceiptEnvelopeViolation {
    Base(CanonicalReceiptEncodingViolation),
    Provenance(VerifierExecutionProvenanceViolation),
    WrongEnvelopeProfile,
    ZeroEnvelopeField,
    AnchorPackageRootMismatch,
    AnchorVerifierLineageMismatch,
    AnchorVerifierReleaseMismatch,
    BaseRunDigestMismatch,
    ProvenancePackageRootMismatch,
    ProvenanceVerifierReleaseMismatch,
    ProvenanceSourceDigestMismatch,
    ProvenanceBuildDigestMismatch,
    ProvenanceExecutionPolicyDigestMismatch,
    ProvenanceBuilderDomainMismatch,
    CanonicalStringTooLong,
}

pub fn bind_canonical_verifier_receipt_envelope(
    base_run: &verifier::VerifierRunReceiptV1,
    anchor_receipt: &certification::ResourceAnchorVerifierReceiptV1,
    provenance: &VerifierExecutionProvenanceV1,
) -> Result<CanonicalVerifierReceiptEnvelopeV1, VerifierReceiptEnvelopeViolation> {
    let base_digest = canonical_verifier_run_receipt_digest(base_run)
        .map_err(VerifierReceiptEnvelopeViolation::Base)?;
    let anchor_digest = canonical_resource_anchor_receipt_digest(anchor_receipt)
        .map_err(VerifierReceiptEnvelopeViolation::Base)?;
    validate_verifier_execution_provenance(provenance)
        .map_err(VerifierReceiptEnvelopeViolation::Provenance)?;

    if anchor_receipt.package_root_digest != base_run.package_root_digest {
        return Err(VerifierReceiptEnvelopeViolation::AnchorPackageRootMismatch);
    }
    if anchor_receipt.verifier_lineage_digest != base_run.verifier_lineage_digest {
        return Err(VerifierReceiptEnvelopeViolation::AnchorVerifierLineageMismatch);
    }
    if anchor_receipt.verifier_release_digest != base_run.verifier_release_digest {
        return Err(VerifierReceiptEnvelopeViolation::AnchorVerifierReleaseMismatch);
    }
    if anchor_receipt.base_verifier_run_receipt_digest != base_digest {
        return Err(VerifierReceiptEnvelopeViolation::BaseRunDigestMismatch);
    }
    if provenance.package_root_digest != base_run.package_root_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenancePackageRootMismatch);
    }
    if provenance.verifier_release_digest != base_run.verifier_release_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenanceVerifierReleaseMismatch);
    }
    if provenance.source_digest != base_run.source_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenanceSourceDigestMismatch);
    }
    if provenance.build_provenance_digest != base_run.build_provenance_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenanceBuildDigestMismatch);
    }
    if provenance.execution_policy_digest != base_run.execution_policy_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenanceExecutionPolicyDigestMismatch);
    }
    if provenance.builder_control_domain_digest != anchor_receipt.builder_control_domain_digest {
        return Err(VerifierReceiptEnvelopeViolation::ProvenanceBuilderDomainMismatch);
    }

    let provenance_digest = canonical_verifier_execution_provenance_digest(provenance)
        .map_err(VerifierReceiptEnvelopeViolation::Provenance)?;

    Ok(CanonicalVerifierReceiptEnvelopeV1 {
        verifier_receipt_envelope_profile_id: VERIFIER_RECEIPT_ENVELOPE_PROFILE_ID.to_owned(),
        package_root_digest: base_run.package_root_digest,
        verifier_lineage_digest: base_run.verifier_lineage_digest,
        verifier_release_digest: base_run.verifier_release_digest,
        builder_control_domain_digest: anchor_receipt.builder_control_domain_digest,
        base_verifier_run_receipt_digest: base_digest,
        resource_anchor_receipt_digest: anchor_digest,
        verifier_provenance_digest: provenance_digest,
    })
}

pub fn canonical_verifier_receipt_envelope_bytes(
    envelope: &CanonicalVerifierReceiptEnvelopeV1,
) -> Result<Vec<u8>, VerifierReceiptEnvelopeViolation> {
    let zero = [0_u8; 32];
    if envelope.verifier_receipt_envelope_profile_id != VERIFIER_RECEIPT_ENVELOPE_PROFILE_ID {
        return Err(VerifierReceiptEnvelopeViolation::WrongEnvelopeProfile);
    }
    if envelope.verifier_receipt_envelope_profile_id.len() > MAX_CANONICAL_STRING_BYTES {
        return Err(VerifierReceiptEnvelopeViolation::CanonicalStringTooLong);
    }
    if envelope.package_root_digest == zero
        || envelope.verifier_lineage_digest == zero
        || envelope.verifier_release_digest == zero
        || envelope.builder_control_domain_digest == zero
        || envelope.base_verifier_run_receipt_digest == zero
        || envelope.resource_anchor_receipt_digest == zero
        || envelope.verifier_provenance_digest == zero
    {
        return Err(VerifierReceiptEnvelopeViolation::ZeroEnvelopeField);
    }

    let mut bytes = Vec::with_capacity(360);
    bytes.extend_from_slice(RECEIPT_ENVELOPE_DOMAIN);
    let profile_bytes = envelope.verifier_receipt_envelope_profile_id.as_bytes();
    let profile_length = u32::try_from(profile_bytes.len())
        .map_err(|_| VerifierReceiptEnvelopeViolation::CanonicalStringTooLong)?;
    bytes.extend_from_slice(&profile_length.to_be_bytes());
    bytes.extend_from_slice(profile_bytes);
    bytes.extend_from_slice(&envelope.package_root_digest);
    bytes.extend_from_slice(&envelope.verifier_lineage_digest);
    bytes.extend_from_slice(&envelope.verifier_release_digest);
    bytes.extend_from_slice(&envelope.builder_control_domain_digest);
    bytes.extend_from_slice(&envelope.base_verifier_run_receipt_digest);
    bytes.extend_from_slice(&envelope.resource_anchor_receipt_digest);
    bytes.extend_from_slice(&envelope.verifier_provenance_digest);
    Ok(bytes)
}

pub fn canonical_verifier_receipt_envelope_digest(
    envelope: &CanonicalVerifierReceiptEnvelopeV1,
) -> Result<Digest32, VerifierReceiptEnvelopeViolation> {
    Ok(sha256(&canonical_verifier_receipt_envelope_bytes(
        envelope,
    )?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        [byte; 32]
    }

    fn stage(
        stage: verifier::VerificationStageId,
        subject: u8,
        finding: u8,
    ) -> verifier::VerificationStageReceiptV1 {
        verifier::VerificationStageReceiptV1 {
            stage,
            package_root_digest: digest(1),
            subject_digest: digest(subject),
            verifier_release_digest: digest(3),
            disposition: verifier::VerificationStageDisposition::Pass,
            finding_digest: digest(finding),
        }
    }

    fn base_run() -> verifier::VerifierRunReceiptV1 {
        verifier::VerifierRunReceiptV1 {
            offline_verifier_profile_id: verifier::OFFLINE_VERIFIER_PROFILE_ID.to_owned(),
            package_root_digest: digest(1),
            verifier_implementation_id: "reference-rust".to_owned(),
            verifier_lineage_digest: digest(2),
            verifier_release_digest: digest(3),
            source_digest: digest(4),
            build_provenance_digest: digest(5),
            execution_policy_digest: digest(6),
            stages: vec![
                stage(verifier::VerificationStageId::CertificationEvidence, 18, 38),
                stage(verifier::VerificationStageId::ChallengeLedger, 17, 37),
                stage(verifier::VerificationStageId::PhysicalAudit, 16, 36),
                stage(verifier::VerificationStageId::TallyEvidence, 15, 35),
                stage(
                    verifier::VerificationStageId::AnonymousAuthorityCensus,
                    14,
                    34,
                ),
                stage(verifier::VerificationStageId::WitnessQuorum, 13, 33),
                stage(verifier::VerificationStageId::TransparencyLineage, 12, 32),
                stage(verifier::VerificationStageId::ElectionConstitution, 11, 31),
                stage(verifier::VerificationStageId::PackageIntegrity, 10, 30),
            ],
        }
    }

    fn anchor_receipt(base_digest: Digest32) -> certification::ResourceAnchorVerifierReceiptV1 {
        certification::ResourceAnchorVerifierReceiptV1 {
            resource_anchor_certification_profile_id:
                certification::RESOURCE_ANCHOR_CERTIFICATION_PROFILE_ID.to_owned(),
            package_root_digest: digest(1),
            election_constitution_digest: digest(40),
            resource_policy_commitment_digest: digest(41),
            certification_policy_digest: digest(42),
            anchor_evidence_digest: digest(43),
            verifier_lineage_digest: digest(2),
            verifier_release_digest: digest(3),
            builder_control_domain_digest: digest(44),
            base_verifier_run_receipt_digest: base_digest,
            anchor_disposition: verifier::VerificationStageDisposition::Pass,
            resource_verification_disposition: verifier::VerificationStageDisposition::Pass,
            finding_digest: digest(45),
        }
    }

    fn provenance() -> VerifierExecutionProvenanceV1 {
        VerifierExecutionProvenanceV1 {
            verifier_provenance_profile_id: VERIFIER_PROVENANCE_PROFILE_ID.to_owned(),
            package_root_digest: digest(1),
            verifier_release_digest: digest(3),
            source_digest: digest(4),
            build_provenance_digest: digest(5),
            execution_policy_digest: digest(6),
            compiler_toolchain_digest: digest(50),
            dependency_lock_digest: digest(51),
            build_recipe_digest: digest(52),
            target_platform_digest: digest(53),
            builder_control_domain_digest: digest(44),
        }
    }

    fn hex_digest(digest: Digest32) -> String {
        let mut output = String::with_capacity(64);
        for byte in digest {
            use std::fmt::Write as _;
            write!(&mut output, "{byte:02x}").expect("write to string");
        }
        output
    }

    #[test]
    fn base_run_has_language_neutral_golden_digest() {
        let digest = canonical_verifier_run_receipt_digest(&base_run()).unwrap();
        assert_eq!(
            hex_digest(digest),
            "a7892dbba3e9432741f005685c5c7b5a92023affd52f22d8a0abbb6a1f42ac75"
        );
    }

    #[test]
    fn stage_vector_order_is_not_semantic() {
        let mut reordered = base_run();
        reordered.stages.reverse();
        assert_eq!(
            canonical_verifier_run_receipt_digest(&base_run()),
            canonical_verifier_run_receipt_digest(&reordered)
        );
    }

    #[test]
    fn changing_a_stage_subject_changes_base_receipt_digest() {
        let original = canonical_verifier_run_receipt_digest(&base_run()).unwrap();
        let mut changed = base_run();
        changed.stages[0].subject_digest = digest(99);
        assert_ne!(
            original,
            canonical_verifier_run_receipt_digest(&changed).unwrap()
        );
    }

    #[test]
    fn anchor_receipt_matches_elect_015_qualified_digest_semantics() {
        let base_digest = canonical_verifier_run_receipt_digest(&base_run()).unwrap();
        let receipt = anchor_receipt(base_digest);
        let digest = canonical_resource_anchor_receipt_digest(&receipt).unwrap();
        assert_eq!(
            digest,
            certification::resource_anchor_verifier_receipt_digest(&receipt).unwrap()
        );
        assert_eq!(
            hex_digest(digest),
            "1b990e3b5da5ead13d489860c4cb7e3f2f4fd6a763c676bbf4eaac1db173ecef"
        );
    }

    #[test]
    fn provenance_has_language_neutral_golden_digest() {
        let digest = canonical_verifier_execution_provenance_digest(&provenance()).unwrap();
        assert_eq!(
            hex_digest(digest),
            "cc1a912f027b6a11cf87fb5873b1a67f00a1b4482f4f002108ae5b1185982096"
        );
    }

    #[test]
    fn envelope_closes_opaque_base_run_digest_gap() {
        let base = base_run();
        let base_digest = canonical_verifier_run_receipt_digest(&base).unwrap();
        let anchor = anchor_receipt(base_digest);
        let envelope = bind_canonical_verifier_receipt_envelope(&base, &anchor, &provenance())
            .expect("fully bound canonical envelope");
        assert_eq!(envelope.base_verifier_run_receipt_digest, base_digest);
        assert_eq!(
            hex_digest(canonical_verifier_receipt_envelope_digest(&envelope).unwrap()),
            "cbbcc8bf5b69fb28ce67bcc1f7b64aa7bdd381702311966ccbcfcc87e947ed3f"
        );
    }

    #[test]
    fn stale_opaque_base_run_digest_fails_closed() {
        let base = base_run();
        let mut anchor = anchor_receipt(digest(98));
        anchor.finding_digest = digest(45);
        assert_eq!(
            bind_canonical_verifier_receipt_envelope(&base, &anchor, &provenance()),
            Err(VerifierReceiptEnvelopeViolation::BaseRunDigestMismatch)
        );
    }

    #[test]
    fn provenance_must_cross_bind_existing_run_digests() {
        let base = base_run();
        let base_digest = canonical_verifier_run_receipt_digest(&base).unwrap();
        let anchor = anchor_receipt(base_digest);
        let mut subject = provenance();
        subject.compiler_toolchain_digest = digest(60);
        let changed_toolchain = bind_canonical_verifier_receipt_envelope(&base, &anchor, &subject)
            .expect("toolchain changes are allowed but committed");
        let original =
            bind_canonical_verifier_receipt_envelope(&base, &anchor, &provenance()).unwrap();
        assert_ne!(
            canonical_verifier_receipt_envelope_digest(&original).unwrap(),
            canonical_verifier_receipt_envelope_digest(&changed_toolchain).unwrap()
        );

        let mut mismatched = provenance();
        mismatched.source_digest = digest(61);
        assert_eq!(
            bind_canonical_verifier_receipt_envelope(&base, &anchor, &mismatched),
            Err(VerifierReceiptEnvelopeViolation::ProvenanceSourceDigestMismatch)
        );
    }

    #[test]
    fn nonpassing_base_run_cannot_be_enveloped_for_certification() {
        let mut base = base_run();
        base.stages[0].disposition = verifier::VerificationStageDisposition::Indeterminate;
        assert!(matches!(
            canonical_verifier_run_receipt_digest(&base),
            Err(CanonicalReceiptEncodingViolation::BaseRunNotPassing(
                verifier::VerifierRunDisposition::AtLeastOneStageIndeterminate
            ))
        ));
    }

    #[test]
    fn canonical_strings_are_length_prefixed_not_concatenated() {
        let mut left = base_run();
        left.verifier_implementation_id = "ab".to_owned();
        let mut right = base_run();
        right.verifier_implementation_id = "a".to_owned();
        assert_ne!(
            canonical_verifier_run_receipt_bytes(&left).unwrap(),
            canonical_verifier_run_receipt_bytes(&right).unwrap()
        );
    }
}
