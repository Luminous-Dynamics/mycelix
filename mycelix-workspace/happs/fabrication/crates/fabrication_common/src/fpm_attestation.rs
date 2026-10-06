//! Challenge-bound source-system attestation qualification for FPM.
//!
//! This module authenticates the binding of an attestation-result declaration
//! to an exact source-system, acquisition root, challenge nonce, and appraisal
//! policy. It does not itself verify a TPM/TEE/EAT signature or prove physical
//! measurement truth.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const SHA256_HEX_LEN: usize = 64;
const MAX_LABEL_BYTES: usize = 128;

pub const FPM_ATTESTATION_SCHEMA_VERSION: &str =
    "fpm.registration.source-attestation.v1";
pub const FPM_ATTESTATION_PROFILE_ID: &str =
    "fpm.registration.source-attestation.challenge-bound";
pub const FPM_ATTESTATION_PROFILE_VERSION: &str = "1";

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpmAttestationDisposition {
    Appraised,
    NotAppraised,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmSourceAttestationClaim {
    pub subject_id: String,
    pub audience: String,
    pub verification_key_id: Vec<u8>,
    pub verification_key_digest: String,
    pub acquisition_root_digest: String,
    pub challenge_nonce_digest: String,
    pub evidence_digest: String,
    pub attestation_format: String,
    pub verifier_id: String,
    pub verifier_version: String,
    pub verifier_profile_digest: String,
    pub appraisal_policy_digest: String,
    pub reference_values_digest: String,
    pub endorsement_digest: String,
    pub disposition: FpmAttestationDisposition,
}

pub fn fpm_attestation_nonce_digest(nonce: &[u8]) -> String {
    let mut bytes = Vec::new();
    append_field(&mut bytes, b"fpm.attestation-nonce.v1");
    append_field(&mut bytes, nonce);
    hex_digest(&bytes)
}

impl FpmSourceAttestationClaim {
    pub fn digest(&self) -> String {
        let mut bytes = Vec::new();
        append_field(&mut bytes, b"fpm.source-attestation-claim.v1");
        append_field(&mut bytes, self.subject_id.as_bytes());
        append_field(&mut bytes, self.audience.as_bytes());
        append_field(&mut bytes, &self.verification_key_id);
        append_field(&mut bytes, self.verification_key_digest.as_bytes());
        append_field(&mut bytes, self.acquisition_root_digest.as_bytes());
        append_field(&mut bytes, self.challenge_nonce_digest.as_bytes());
        append_field(&mut bytes, self.evidence_digest.as_bytes());
        append_field(&mut bytes, self.attestation_format.as_bytes());
        append_field(&mut bytes, self.verifier_id.as_bytes());
        append_field(&mut bytes, self.verifier_version.as_bytes());
        append_field(&mut bytes, self.verifier_profile_digest.as_bytes());
        append_field(&mut bytes, self.appraisal_policy_digest.as_bytes());
        append_field(&mut bytes, self.reference_values_digest.as_bytes());
        append_field(&mut bytes, self.endorsement_digest.as_bytes());
        append_field(
            &mut bytes,
            match self.disposition {
                FpmAttestationDisposition::Appraised => b"appraised",
                FpmAttestationDisposition::NotAppraised => b"not-appraised",
            },
        );
        hex_digest(&bytes)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmAttestationQualificationInput {
    pub expected_subject_id: String,
    pub expected_audience: String,
    pub expected_verification_key_id: Vec<u8>,
    pub expected_verification_key_digest: String,
    pub expected_acquisition_root_digest: String,
    pub expected_challenge_nonce_digest: String,
    pub expected_attestation_format: String,
    pub expected_verifier_profile_digest: String,
    pub expected_appraisal_policy_digest: String,
    pub expected_reference_values_digest: String,
    pub expected_endorsement_digest: String,
    pub claim: FpmSourceAttestationClaim,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpmAttestationQualificationStatus {
    QualifiedForProfile,
    InsufficientEvidence,
    InvalidEvidence,
    ConflictingAttestation,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FpmAttestationQualificationReason {
    InvalidIdentifier,
    InvalidDigestEncoding,
    SubjectMismatch,
    AudienceMismatch,
    VerificationKeyIdMismatch,
    VerificationKeyDigestMismatch,
    AcquisitionRootMismatch,
    ChallengeNonceMismatch,
    EvidenceDigestMismatch,
    VerifierProfileMismatch,
    AppraisalPolicyMismatch,
    ReferenceValuesMissing,
    ReferenceValuesMismatch,
    EndorsementMissing,
    EndorsementMismatch,
    InvalidEvidenceFormat,
    AttestationFormatMismatch,
    EmptyVerifierIdentity,
    EmptyVerifierVersion,
    NotAppraised,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct FpmAttestationQualification {
    pub schema_version: String,
    pub profile_id: String,
    pub profile_version: String,
    pub claim_digest: String,
    pub status: FpmAttestationQualificationStatus,
    pub reasons: Vec<FpmAttestationQualificationReason>,
}

pub fn qualify_source_attestation(
    input: &FpmAttestationQualificationInput,
) -> FpmAttestationQualification {
    let mut reasons = std::collections::BTreeSet::new();

    for value in [
        &input.expected_subject_id,
        &input.claim.subject_id,
        &input.expected_audience,
        &input.claim.audience,
        &input.expected_verification_key_digest,
        &input.claim.verification_key_digest,
        &input.expected_attestation_format,
    ] {
        if !valid_label(value) {
            reasons.insert(FpmAttestationQualificationReason::InvalidIdentifier);
        }
    }

    for value in [
        &input.expected_acquisition_root_digest,
        &input.expected_challenge_nonce_digest,
        &input.expected_verifier_profile_digest,
        &input.expected_appraisal_policy_digest,
        &input.expected_reference_values_digest,
        &input.expected_endorsement_digest,
        &input.claim.acquisition_root_digest,
        &input.claim.challenge_nonce_digest,
        &input.claim.evidence_digest,
        &input.claim.verifier_profile_digest,
        &input.claim.appraisal_policy_digest,
        &input.claim.reference_values_digest,
        &input.claim.endorsement_digest,
    ] {
        if !is_canonical_digest(value) {
            reasons.insert(FpmAttestationQualificationReason::InvalidDigestEncoding);
        }
    }

    if input.claim.subject_id != input.expected_subject_id {
        reasons.insert(FpmAttestationQualificationReason::SubjectMismatch);
    }
    if input.claim.audience != input.expected_audience {
        reasons.insert(FpmAttestationQualificationReason::AudienceMismatch);
    }
    if input.claim.verification_key_id != input.expected_verification_key_id {
        reasons.insert(FpmAttestationQualificationReason::VerificationKeyIdMismatch);
    }
    if input.claim.verification_key_digest != input.expected_verification_key_digest {
        reasons.insert(FpmAttestationQualificationReason::VerificationKeyDigestMismatch);
    }
    if input.claim.acquisition_root_digest != input.expected_acquisition_root_digest {
        reasons.insert(FpmAttestationQualificationReason::AcquisitionRootMismatch);
    }
    if input.claim.challenge_nonce_digest != input.expected_challenge_nonce_digest {
        reasons.insert(FpmAttestationQualificationReason::ChallengeNonceMismatch);
    }
    if input.claim.attestation_format != input.expected_attestation_format {
        reasons.insert(FpmAttestationQualificationReason::AttestationFormatMismatch);
    }
    if input.claim.verifier_profile_digest != input.expected_verifier_profile_digest {
        reasons.insert(FpmAttestationQualificationReason::VerifierProfileMismatch);
    }
    if input.claim.appraisal_policy_digest != input.expected_appraisal_policy_digest {
        reasons.insert(FpmAttestationQualificationReason::AppraisalPolicyMismatch);
    }
    if input.claim.reference_values_digest != input.expected_reference_values_digest {
        reasons.insert(FpmAttestationQualificationReason::ReferenceValuesMismatch);
    }
    if input.claim.endorsement_digest != input.expected_endorsement_digest {
        reasons.insert(FpmAttestationQualificationReason::EndorsementMismatch);
    }
    if !valid_label(&input.claim.attestation_format) {
        reasons.insert(FpmAttestationQualificationReason::InvalidEvidenceFormat);
    }
    if !valid_label(&input.claim.verifier_id) {
        reasons.insert(FpmAttestationQualificationReason::EmptyVerifierIdentity);
    }
    if !valid_label(&input.claim.verifier_version) {
        reasons.insert(FpmAttestationQualificationReason::EmptyVerifierVersion);
    }
    if !is_canonical_digest(&input.claim.reference_values_digest) {
        reasons.insert(FpmAttestationQualificationReason::ReferenceValuesMissing);
    }
    if !is_canonical_digest(&input.claim.endorsement_digest) {
        reasons.insert(FpmAttestationQualificationReason::EndorsementMissing);
    }
    if input.claim.disposition == FpmAttestationDisposition::NotAppraised {
        reasons.insert(FpmAttestationQualificationReason::NotAppraised);
    }

    let status = if reasons.is_empty() {
        FpmAttestationQualificationStatus::QualifiedForProfile
    } else if reasons.contains(&FpmAttestationQualificationReason::SubjectMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::AudienceMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::VerificationKeyIdMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::VerificationKeyDigestMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::AcquisitionRootMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::ChallengeNonceMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::AttestationFormatMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::VerifierProfileMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::AppraisalPolicyMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::ReferenceValuesMismatch)
        || reasons.contains(&FpmAttestationQualificationReason::EndorsementMismatch)
    {
        FpmAttestationQualificationStatus::ConflictingAttestation
    } else if reasons.iter().any(|reason| {
        matches!(
            reason,
            FpmAttestationQualificationReason::InvalidIdentifier
                | FpmAttestationQualificationReason::InvalidDigestEncoding
                | FpmAttestationQualificationReason::InvalidEvidenceFormat
                | FpmAttestationQualificationReason::EmptyVerifierIdentity
                | FpmAttestationQualificationReason::EmptyVerifierVersion
        )
    }) {
        FpmAttestationQualificationStatus::InvalidEvidence
    } else {
        FpmAttestationQualificationStatus::InsufficientEvidence
    };

    FpmAttestationQualification {
        schema_version: FPM_ATTESTATION_SCHEMA_VERSION.into(),
        profile_id: FPM_ATTESTATION_PROFILE_ID.into(),
        profile_version: FPM_ATTESTATION_PROFILE_VERSION.into(),
        claim_digest: input.claim.digest(),
        status,
        reasons: reasons.into_iter().collect(),
    }
}

fn valid_label(value: &str) -> bool {
    !value.trim().is_empty()
        && value == value.trim()
        && value.len() <= MAX_LABEL_BYTES
        && !value.chars().any(char::is_control)
}

fn is_canonical_digest(value: &str) -> bool {
    value.len() == SHA256_HEX_LEN
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn append_field(buffer: &mut Vec<u8>, field: &[u8]) {
    buffer.extend_from_slice(&(field.len() as u64).to_be_bytes());
    buffer.extend_from_slice(field);
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher
        .finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat(ch).take(SHA256_HEX_LEN).collect()
    }

    fn claim() -> FpmSourceAttestationClaim {
        FpmSourceAttestationClaim {
            subject_id: "source-1".into(),
            audience: "fpm.example.consumer".into(),
            verification_key_id: b"fpm-key-1".to_vec(),
            verification_key_digest: digest('7'),
            acquisition_root_digest: digest('a'),
            challenge_nonce_digest: digest('b'),
            evidence_digest: digest('c'),
            attestation_format: "eat-cbor".into(),
            verifier_id: "verifier-1".into(),
            verifier_version: "1".into(),
            verifier_profile_digest: digest('d'),
            appraisal_policy_digest: digest('e'),
            reference_values_digest: digest('f'),
            endorsement_digest: digest('a'),
            disposition: FpmAttestationDisposition::Appraised,
        }
    }

    fn input() -> FpmAttestationQualificationInput {
        let claim = claim();
        FpmAttestationQualificationInput {
            expected_subject_id: claim.subject_id.clone(),
            expected_audience: claim.audience.clone(),
            expected_verification_key_id: claim.verification_key_id.clone(),
            expected_verification_key_digest: claim.verification_key_digest.clone(),
            expected_acquisition_root_digest: claim.acquisition_root_digest.clone(),
            expected_challenge_nonce_digest: claim.challenge_nonce_digest.clone(),
            expected_attestation_format: claim.attestation_format.clone(),
            expected_verifier_profile_digest: claim.verifier_profile_digest.clone(),
            expected_appraisal_policy_digest: claim.appraisal_policy_digest.clone(),
            expected_reference_values_digest: claim.reference_values_digest.clone(),
            expected_endorsement_digest: claim.endorsement_digest.clone(),
            claim,
        }
    }

    #[test]
    fn exact_challenge_bound_claim_qualifies() {
        let result = qualify_source_attestation(&input());
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::QualifiedForProfile
        );
        assert!(result.reasons.is_empty());
    }

    #[test]
    fn subject_substitution_conflicts() {
        let mut input = input();
        input.claim.subject_id = "other-source".into();
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::SubjectMismatch));
    }

    #[test]
    fn audience_substitution_conflicts() {
        let mut input = input();
        input.claim.audience = "other.fpm.consumer".into();
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::AudienceMismatch));
    }

    #[test]
    fn verification_key_id_substitution_conflicts() {
        let mut input = input();
        input.claim.verification_key_id = b"other-key".to_vec();
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::VerificationKeyIdMismatch));
    }

    #[test]
    fn verification_key_digest_substitution_conflicts() {
        let mut input = input();
        input.claim.verification_key_digest = digest('9');
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::VerificationKeyDigestMismatch));
    }

    #[test]
    fn verifier_profile_substitution_conflicts() {
        let mut input = input();
        input.claim.verifier_profile_digest = digest('9');
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::VerifierProfileMismatch));
    }

    #[test]
    fn policy_support_substitution_conflicts() {
        let mut input = input();
        input.claim.reference_values_digest = digest('9');
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::ReferenceValuesMismatch));
        input = input();
        input.claim.endorsement_digest = digest('9');
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::EndorsementMismatch));
    }

    #[test]
    fn nonce_substitution_conflicts() {
        let mut input = input();
        input.claim.challenge_nonce_digest = digest('9');
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::ChallengeNonceMismatch));
    }

    #[test]
    fn unappraised_attestation_cannot_qualify() {
        let mut input = input();
        input.claim.disposition = FpmAttestationDisposition::NotAppraised;
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::InsufficientEvidence
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::NotAppraised));
    }

    #[test]
    fn nonce_digest_is_deterministic_and_domain_separated() {
        let a = fpm_attestation_nonce_digest(b"nonce-a");
        let b = fpm_attestation_nonce_digest(b"nonce-a");
        let c = fpm_attestation_nonce_digest(b"nonce-b");
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_eq!(a.len(), SHA256_HEX_LEN);
    }

    #[test]
    fn attestation_format_substitution_conflicts() {
        let mut input = input();
        input.claim.attestation_format = "eat-jwt".into();
        let result = qualify_source_attestation(&input);
        assert_eq!(
            result.status,
            FpmAttestationQualificationStatus::ConflictingAttestation
        );
        assert!(result
            .reasons
            .contains(&FpmAttestationQualificationReason::AttestationFormatMismatch));
    }

    #[test]
    fn claim_digest_is_deterministic() {
        let input = input();
        assert_eq!(input.claim.digest(), input.claim.digest());
        assert_eq!(input.claim.digest().len(), SHA256_HEX_LEN);
    }
}
