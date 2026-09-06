// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure challenge-bound authentication for authority-state source heads.
//!
//! This layer proves only that one exact source identity authenticated one exact
//! challenged head. Institutional source trust and source-head completeness are
//! separate later decisions in #94/#96/#91.

use mycelix_authority_state_coverage::{
    AuthoritySourceHeadAttestation, VerifiedAuthoritySourceHead,
    SOURCE_HEAD_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::VerifiedCoverageChallenge;
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-source-head-verifier-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-source-head-authentication-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority/source-head-authentication/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Independently verified authentication of one exact source-head attestation.
///
/// This is cryptographic/authentication evidence only. The attestation's source
/// identity is still checked against institutional coverage policy later.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedSourceHeadProof {
    pub protocol_version: String,
    pub attestation_digest: Digest32,
    pub attestation_profile: String,
    pub challenge_digest: Digest32,
    pub source_identity_digest: Digest32,
    pub source_identity_profile: String,
    pub source_proof_ref: String,
    pub verified_source_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedSourceHeadProof {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), SourceHeadVerifierError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(SourceHeadVerifierError::WrongProtocolVersion);
        }
        if self.attestation_digest.is_zero()
            || self.challenge_digest.is_zero()
            || self.source_identity_digest.is_zero()
        {
            return Err(SourceHeadVerifierError::ZeroDigest);
        }
        if self.attestation_profile != SOURCE_HEAD_IDENTITY_PROFILE {
            return Err(SourceHeadVerifierError::WrongAttestationProfile);
        }
        require_profile(&self.source_identity_profile)?;
        for value in [
            self.source_proof_ref.as_str(),
            self.verified_source_proof_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.source_proof_ref != self.verified_source_proof_ref {
            return Err(SourceHeadVerifierError::SourceProofMismatch);
        }
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms < self.verified_at_ms
        {
            return Err(SourceHeadVerifierError::InvalidVerificationWindow);
        }
        Ok(())
    }
}

/// Non-deserializable positive authentication result.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedSourceHeadAuthentication {
    source_head: VerifiedAuthoritySourceHead,
    attestation_digest: Digest32,
    challenge_digest: Digest32,
    qualification_digest: Digest32,
    qualification_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedSourceHeadAuthentication {
    pub fn source_head(&self) -> &VerifiedAuthoritySourceHead {
        &self.source_head
    }
    pub fn to_verified_source_head(&self) -> VerifiedAuthoritySourceHead {
        self.source_head.clone()
    }
    pub fn attestation_digest(&self) -> Digest32 {
        self.attestation_digest
    }
    pub fn challenge_digest(&self) -> Digest32 {
        self.challenge_digest
    }
    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }
    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

pub fn qualify_source_head_authentication(
    challenge_receipt: &VerifiedCoverageChallenge,
    attestation: &AuthoritySourceHeadAttestation,
    proof: &VerifiedSourceHeadProof,
    now_ms: u64,
) -> Result<QualifiedSourceHeadAuthentication, SourceHeadVerifierError> {
    verify_challenge(challenge_receipt, now_ms)?;
    attestation
        .validate()
        .map_err(|_| SourceHeadVerifierError::InvalidAttestation)?;
    proof.validate_at(now_ms)?;

    let challenge_digest = challenge_receipt
        .challenge
        .identity_digest()
        .map_err(|_| SourceHeadVerifierError::InvalidChallenge)?;
    if attestation.challenge_digest != challenge_digest
        || proof.challenge_digest != challenge_digest
    {
        return Err(SourceHeadVerifierError::ChallengeMismatch);
    }
    if attestation.subject != challenge_receipt.challenge.subject {
        return Err(SourceHeadVerifierError::SubjectMismatch);
    }

    let attestation_digest = attestation
        .identity_digest()
        .map_err(|_| SourceHeadVerifierError::InvalidAttestation)?;
    if proof.attestation_digest != attestation_digest {
        return Err(SourceHeadVerifierError::AttestationDigestMismatch);
    }
    if proof.source_identity_digest != attestation.source_identity.digest
        || proof.source_identity_profile != attestation.source_identity.profile
    {
        return Err(SourceHeadVerifierError::SourceIdentityMismatch);
    }
    if proof.source_proof_ref != attestation.source_proof_ref {
        return Err(SourceHeadVerifierError::SourceProofMismatch);
    }

    // Causal order is part of verification, not advisory metadata.
    if attestation.responded_at_ms < challenge_receipt.challenge.issued_at_ms
        || attestation.responded_at_ms < challenge_receipt.verified_at_ms
        || attestation.responded_at_ms >= challenge_receipt.challenge.expires_at_ms
        || proof.verified_at_ms < attestation.responded_at_ms
        || now_ms >= attestation.expires_at_ms
    {
        return Err(SourceHeadVerifierError::AttestationOutsideChallengeWindow);
    }

    let verified_at_ms = challenge_receipt
        .verified_at_ms
        .max(proof.verified_at_ms)
        .max(attestation.responded_at_ms);
    let valid_until_ms = challenge_receipt
        .challenge
        .expires_at_ms
        .min(attestation.expires_at_ms)
        .min(proof.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(SourceHeadVerifierError::InvalidVerificationWindow);
    }

    let qualification_digest = qualification_digest(
        challenge_digest,
        attestation_digest,
        proof.source_identity_digest,
    );
    let verification_ref = format!(
        "source-head-auth:{}:{}",
        QUALIFICATION_PROFILE,
        hex_digest(qualification_digest)
    );

    Ok(QualifiedSourceHeadAuthentication {
        source_head: VerifiedAuthoritySourceHead {
            attestation: attestation.clone(),
            verified_source_identity: attestation.source_identity.clone(),
            verified_source_proof_ref: proof.verified_source_proof_ref.clone(),
            verification_ref,
            verified_at_ms,
        },
        attestation_digest,
        challenge_digest,
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_challenge(
    receipt: &VerifiedCoverageChallenge,
    now_ms: u64,
) -> Result<(), SourceHeadVerifierError> {
    receipt
        .challenge
        .validate()
        .map_err(|_| SourceHeadVerifierError::InvalidChallenge)?;
    if receipt.verified_nonce_digest != receipt.challenge.nonce_digest
        || receipt.verified_randomness_proof_ref != receipt.challenge.randomness_proof_ref
        || receipt.verified_challenge_issuer_ref != receipt.challenge.challenge_issuer_ref
    {
        return Err(SourceHeadVerifierError::ChallengeVerificationMismatch);
    }
    require_ref(&receipt.verification_ref)?;
    if now_ms == 0
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms < receipt.challenge.issued_at_ms
        || receipt.verified_at_ms > now_ms
        || now_ms >= receipt.challenge.expires_at_ms
    {
        return Err(SourceHeadVerifierError::InvalidChallengeWindow);
    }
    Ok(())
}

fn qualification_digest(
    challenge_digest: Digest32,
    attestation_digest: Digest32,
    source_identity_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &challenge_digest.0);
    frame(&mut hasher, SOURCE_HEAD_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &attestation_digest.0);
    frame(&mut hasher, &source_identity_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn require_ref(value: &str) -> Result<(), SourceHeadVerifierError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(SourceHeadVerifierError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), SourceHeadVerifierError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(SourceHeadVerifierError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SourceHeadVerifierError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidProfile,
    ZeroDigest,
    WrongAttestationProfile,
    InvalidVerificationWindow,
    InvalidChallenge,
    InvalidChallengeWindow,
    ChallengeVerificationMismatch,
    InvalidAttestation,
    ChallengeMismatch,
    SubjectMismatch,
    AttestationDigestMismatch,
    SourceIdentityMismatch,
    SourceProofMismatch,
    AttestationOutsideChallengeWindow,
}

impl fmt::Display for SourceHeadVerifierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong source-head verifier protocol version",
            Self::InvalidReference => "invalid source-head verifier reference",
            Self::InvalidProfile => "invalid source-head verifier profile",
            Self::ZeroDigest => "source-head verifier digest must not be zero",
            Self::WrongAttestationProfile => "wrong source-head attestation profile",
            Self::InvalidVerificationWindow => "source-head verification is not currently usable",
            Self::InvalidChallenge => "invalid source-head challenge",
            Self::InvalidChallengeWindow => "source-head challenge is not currently live",
            Self::ChallengeVerificationMismatch => "source-head challenge verification echo mismatch",
            Self::InvalidAttestation => "invalid source-head attestation",
            Self::ChallengeMismatch => "source-head attestation/proof belongs to another challenge",
            Self::SubjectMismatch => "source-head attestation belongs to another subject",
            Self::AttestationDigestMismatch => "source-head proof belongs to another attestation",
            Self::SourceIdentityMismatch => "source-head proof authenticated another source identity",
            Self::SourceProofMismatch => "source-head proof reference mismatch",
            Self::AttestationOutsideChallengeWindow => "source-head response/proof is outside causal challenge ordering",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for SourceHeadVerifierError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest};
    use mycelix_authority_state_coverage::PROTOCOL_VERSION as COVERAGE_PROTOCOL;
    use mycelix_authority_state_coverage_context::{
        CoverageChallenge, PROTOCOL_VERSION as CONTEXT_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 { Digest32([byte; 32]) }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test:authority".into(),
            subject_id: "grant:test".into(),
            identity: ProfiledDigest { digest: d(1), profile: "grant-v1".into() },
        }
    }

    fn challenge() -> VerifiedCoverageChallenge {
        let challenge = CoverageChallenge {
            protocol_version: CONTEXT_PROTOCOL.into(),
            context_policy_digest: d(2),
            context_policy_profile: mycelix_authority_state_coverage_context::CONTEXT_POLICY_PROFILE.into(),
            coverage_policy_digest: d(3),
            coverage_policy_profile: mycelix_authority_state_coverage::POLICY_IDENTITY_PROFILE.into(),
            subject: subject(),
            nonce_digest: d(4),
            randomness_proof_ref: "entropy:1".into(),
            issued_at_ms: 100,
            expires_at_ms: 500,
            challenge_issuer_ref: "did:mycelix:probe".into(),
        };
        VerifiedCoverageChallenge {
            verified_nonce_digest: challenge.nonce_digest,
            verified_randomness_proof_ref: challenge.randomness_proof_ref.clone(),
            verified_challenge_issuer_ref: challenge.challenge_issuer_ref.clone(),
            challenge,
            verification_ref: "challenge-verification:1".into(),
            verified_at_ms: 110,
        }
    }

    fn attestation(challenge: &VerifiedCoverageChallenge) -> AuthoritySourceHeadAttestation {
        AuthoritySourceHeadAttestation {
            protocol_version: COVERAGE_PROTOCOL.into(),
            subject: subject(),
            authoritative_source_ref: "authority-source:test".into(),
            source_identity: ProfiledDigest { digest: d(5), profile: "source-key-v1".into() },
            head_generation: 2,
            head_transition_digest: d(6),
            head_status_record_ref: "state:2".into(),
            source_chain_head_ref: "chain-head:2".into(),
            challenge_digest: challenge.challenge.identity_digest().unwrap(),
            responded_at_ms: 120,
            expires_at_ms: 450,
            source_proof_ref: "source-proof:1".into(),
        }
    }

    fn proof(challenge: &VerifiedCoverageChallenge, attestation: &AuthoritySourceHeadAttestation) -> VerifiedSourceHeadProof {
        VerifiedSourceHeadProof {
            protocol_version: PROTOCOL_VERSION.into(),
            attestation_digest: attestation.identity_digest().unwrap(),
            attestation_profile: SOURCE_HEAD_IDENTITY_PROFILE.into(),
            challenge_digest: challenge.challenge.identity_digest().unwrap(),
            source_identity_digest: attestation.source_identity.digest,
            source_identity_profile: attestation.source_identity.profile.clone(),
            source_proof_ref: attestation.source_proof_ref.clone(),
            verified_source_proof_ref: attestation.source_proof_ref.clone(),
            verification_ref: "crypto-verify:1".into(),
            verified_at_ms: 130,
            valid_until_ms: 430,
        }
    }

    #[test]
    fn authenticates_exact_challenged_source_head() {
        let challenge = challenge();
        let attestation = attestation(&challenge);
        let proof = proof(&challenge, &attestation);
        let qualified = qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap();
        assert_eq!(qualified.source_head().attestation, attestation);
    }

    #[test]
    fn challenge_verification_before_issue_denies() {
        let mut challenge = challenge();
        challenge.verified_at_ms = 99;
        let attestation = attestation(&challenge);
        let proof = proof(&challenge, &attestation);
        assert_eq!(
            qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap_err(),
            SourceHeadVerifierError::InvalidChallengeWindow
        );
    }

    #[test]
    fn source_response_before_challenge_denies() {
        let challenge = challenge();
        let mut attestation = attestation(&challenge);
        attestation.responded_at_ms = 90;
        let proof = proof(&challenge, &attestation);
        assert_eq!(
            qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap_err(),
            SourceHeadVerifierError::AttestationOutsideChallengeWindow
        );
    }

    #[test]
    fn source_response_before_challenge_verification_denies() {
        let mut challenge = challenge();
        challenge.verified_at_ms = 125;
        let attestation = attestation(&challenge);
        let proof = proof(&challenge, &attestation);
        assert_eq!(
            qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap_err(),
            SourceHeadVerifierError::AttestationOutsideChallengeWindow
        );
    }

    #[test]
    fn proof_verification_before_response_denies() {
        let challenge = challenge();
        let attestation = attestation(&challenge);
        let mut proof = proof(&challenge, &attestation);
        proof.verified_at_ms = 119;
        assert_eq!(
            qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap_err(),
            SourceHeadVerifierError::AttestationOutsideChallengeWindow
        );
    }

    #[test]
    fn wrong_source_identity_denies() {
        let challenge = challenge();
        let attestation = attestation(&challenge);
        let mut proof = proof(&challenge, &attestation);
        proof.source_identity_digest = d(88);
        assert_eq!(
            qualify_source_head_authentication(&challenge, &attestation, &proof, 200).unwrap_err(),
            SourceHeadVerifierError::SourceIdentityMismatch
        );
    }
}
