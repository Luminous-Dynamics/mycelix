// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Split witness verification for authority-state coverage.
//!
//! Observation authentication and institutional trust-domain classification are
//! distinct facts. This pure kernel requires independent receipts for both and
//! locally binds them to the exact challenge/context/source head before projecting
//! the existing #94/#96 compatibility receipts.

use mycelix_authority_freshness::ProfiledDigest;
use mycelix_authority_state_coverage::{
    AuthorityHeadWitnessObservation, AuthoritySourceHeadAttestation,
    VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead, WITNESS_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageChallenge, VerifiedWitnessTrustBinding,
    WitnessTrustBinding, CONTEXT_POLICY_PROFILE, WITNESS_TRUST_BINDING_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-witness-verifier-v0.1";
pub const OBSERVATION_PROOF_PROTOCOL: &str =
    "mycelix-authority-witness-observation-proof-v0.1";
pub const TRUST_CLASSIFICATION_PROOF_PROTOCOL: &str =
    "mycelix-authority-witness-trust-classification-proof-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-witness-verification-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-authority-witness-verification-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority-state/witness-verification/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority-state/witness-verification-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Evidence-shaped cryptographic authentication of one exact witness observation.
///
/// This receipt proves only that the exact observer identity authenticated the
/// exact observation bytes/proof. It does not classify that observer into an
/// institutionally trusted independence domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedWitnessObservationProof {
    pub protocol_version: String,
    pub observation_digest: Digest32,
    pub observation_profile: String,
    pub verified_observer_id: String,
    pub verified_observation_proof_ref: String,
    pub observation_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedWitnessObservationProof {
    fn validate_against(
        &self,
        observation: &AuthorityHeadWitnessObservation,
        observation_digest: Digest32,
        now_ms: u64,
    ) -> Result<(), WitnessVerificationError> {
        if self.protocol_version != OBSERVATION_PROOF_PROTOCOL {
            return Err(WitnessVerificationError::WrongObservationProofProtocol);
        }
        if self.observation_digest.is_zero()
            || self.observation_digest != observation_digest
            || self.observation_profile != WITNESS_IDENTITY_PROFILE
        {
            return Err(WitnessVerificationError::ObservationIdentityMismatch);
        }
        require_profile(&self.observation_profile)?;
        for value in [
            self.verified_observer_id.as_str(),
            self.verified_observation_proof_ref.as_str(),
            self.observation_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.verified_observer_id != observation.observer_id {
            return Err(WitnessVerificationError::ObserverIdentityMismatch);
        }
        if self.verified_observation_proof_ref != observation.observation_proof_ref {
            return Err(WitnessVerificationError::ObservationProofRefMismatch);
        }
        validate_window(
            observation.observed_at_ms,
            self.verified_at_ms,
            self.valid_until_ms,
            now_ms,
        )
        .map_err(|_| WitnessVerificationError::InvalidObservationProofWindow)
    }
}

/// Evidence-shaped institutional classification of one observer under one exact
/// witness-trust policy and designated trust-verifier domain.
///
/// This receipt does not authenticate any witness observation. It classifies only
/// the observer-to-domain relation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedWitnessTrustClassificationProof {
    pub protocol_version: String,
    pub witness_trust_policy: ProfiledDigest,
    pub observer_id: String,
    pub trust_domain_id: String,
    pub trust_domain_ref: String,
    pub classification_proof_ref: String,
    pub verified_classification_proof_ref: String,
    pub verified_trust_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedWitnessTrustClassificationProof {
    fn validate_against(
        &self,
        context: &CoverageTrustContextPolicy,
        observation: &AuthorityHeadWitnessObservation,
        now_ms: u64,
    ) -> Result<(), WitnessVerificationError> {
        if self.protocol_version != TRUST_CLASSIFICATION_PROOF_PROTOCOL {
            return Err(WitnessVerificationError::WrongTrustClassificationProtocol);
        }
        let expected_policy = context
            .witness_trust_policy
            .as_ref()
            .ok_or(WitnessVerificationError::MissingWitnessTrustPolicy)?;
        let expected_verifier = context
            .witness_trust_verifier_ref
            .as_ref()
            .ok_or(WitnessVerificationError::MissingWitnessTrustVerifier)?;
        self.witness_trust_policy
            .validate()
            .map_err(|_| WitnessVerificationError::InvalidWitnessTrustPolicy)?;
        if &self.witness_trust_policy != expected_policy {
            return Err(WitnessVerificationError::WitnessTrustPolicyMismatch);
        }
        for value in [
            self.observer_id.as_str(),
            self.trust_domain_id.as_str(),
            self.trust_domain_ref.as_str(),
            self.classification_proof_ref.as_str(),
            self.verified_classification_proof_ref.as_str(),
            self.verified_trust_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if self.observer_id != observation.observer_id {
            return Err(WitnessVerificationError::ClassificationObserverMismatch);
        }
        if self.trust_domain_id != observation.trust_domain_id {
            return Err(WitnessVerificationError::ClassificationDomainMismatch);
        }
        if self.verified_classification_proof_ref != self.classification_proof_ref {
            return Err(WitnessVerificationError::ClassificationProofRefMismatch);
        }
        if self.verified_trust_verifier_ref != *expected_verifier {
            return Err(WitnessVerificationError::TrustVerifierMismatch);
        }
        validate_window(
            1,
            self.verified_at_ms,
            self.valid_until_ms,
            now_ms,
        )
        .map_err(|_| WitnessVerificationError::InvalidTrustClassificationWindow)
    }
}

/// Non-deserializable positive witness qualification. The two compatibility
/// receipts remain coupled inside this object so neither verifier can manufacture
/// the complete #94/#96 witness authority surface by itself.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedWitnessEvidence {
    witness: VerifiedAuthorityHeadWitness,
    trust_binding: VerifiedWitnessTrustBinding,
    observation_digest: Digest32,
    observation_profile: String,
    trust_binding_digest: Digest32,
    trust_binding_profile: String,
    qualification_digest: Digest32,
    qualification_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedWitnessEvidence {
    pub fn observation_digest(&self) -> Digest32 {
        self.observation_digest
    }

    pub fn observation_profile(&self) -> &str {
        &self.observation_profile
    }

    pub fn trust_binding_digest(&self) -> Digest32 {
        self.trust_binding_digest
    }

    pub fn trust_binding_profile(&self) -> &str {
        &self.trust_binding_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn to_verified_witness(&self) -> VerifiedAuthorityHeadWitness {
        self.witness.clone()
    }

    pub fn to_verified_trust_binding(&self) -> VerifiedWitnessTrustBinding {
        self.trust_binding.clone()
    }
}

/// Qualify one exact witness observation from independent observation-authentication
/// and trust-classification proof domains.
pub fn qualify_witness_evidence(
    context: &CoverageTrustContextPolicy,
    challenge_receipt: &VerifiedCoverageChallenge,
    source_receipt: &VerifiedAuthoritySourceHead,
    observation: &AuthorityHeadWitnessObservation,
    observation_proof: &VerifiedWitnessObservationProof,
    classification_proof: &VerifiedWitnessTrustClassificationProof,
    now_ms: u64,
) -> Result<QualifiedWitnessEvidence, WitnessVerificationError> {
    if now_ms == 0 {
        return Err(WitnessVerificationError::InvalidVerificationTime);
    }
    context
        .validate()
        .map_err(|_| WitnessVerificationError::InvalidContextPolicy)?;
    if context.witness_trust_policy.is_none() || context.witness_trust_verifier_ref.is_none() {
        return Err(WitnessVerificationError::MissingWitnessTrustPolicy);
    }
    challenge_receipt
        .challenge
        .validate()
        .map_err(|_| WitnessVerificationError::InvalidChallenge)?;
    source_receipt
        .attestation
        .validate()
        .map_err(|_| WitnessVerificationError::InvalidSourceHead)?;
    observation
        .validate()
        .map_err(|_| WitnessVerificationError::InvalidObservation)?;

    let context_digest = context
        .identity_digest()
        .map_err(|_| WitnessVerificationError::InvalidContextPolicy)?;
    let challenge_digest = challenge_receipt
        .challenge
        .identity_digest()
        .map_err(|_| WitnessVerificationError::InvalidChallenge)?;
    let source_head_digest = source_receipt
        .attestation
        .identity_digest()
        .map_err(|_| WitnessVerificationError::InvalidSourceHead)?;
    let observation_digest = observation
        .identity_digest()
        .map_err(|_| WitnessVerificationError::InvalidObservation)?;

    bind_observation_to_source(
        observation,
        &source_receipt.attestation,
        source_head_digest,
        challenge_digest,
    )?;
    if challenge_receipt.challenge.context_policy_digest != context_digest
        || challenge_receipt.challenge.context_policy_profile != CONTEXT_POLICY_PROFILE
    {
        return Err(WitnessVerificationError::ChallengeContextMismatch);
    }

    observation_proof.validate_against(observation, observation_digest, now_ms)?;
    classification_proof.validate_against(context, observation, now_ms)?;

    // The legacy VerifiedAuthorityHeadWitness ABI has no independent verifier
    // expiry field. Do not project it if either verifier horizon is tighter than
    // the signed observation's own expiry; doing so would widen reusable authority.
    let proof_horizon = observation_proof
        .valid_until_ms
        .min(classification_proof.valid_until_ms);
    if observation.expires_at_ms > proof_horizon {
        return Err(WitnessVerificationError::LossyWitnessProjection);
    }

    let expected_trust_policy = context
        .witness_trust_policy
        .as_ref()
        .ok_or(WitnessVerificationError::MissingWitnessTrustPolicy)?;
    let binding = WitnessTrustBinding {
        protocol_version: mycelix_authority_state_coverage_context::PROTOCOL_VERSION.into(),
        context_policy_digest: context_digest,
        context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
        witness_trust_policy: expected_trust_policy.clone(),
        challenge_digest,
        witness_observation_digest: observation_digest,
        observer_id: observation.observer_id.clone(),
        trust_domain_id: classification_proof.trust_domain_id.clone(),
        trust_domain_ref: classification_proof.trust_domain_ref.clone(),
        classification_proof_ref: classification_proof.classification_proof_ref.clone(),
    };
    binding
        .validate()
        .map_err(|_| WitnessVerificationError::InvalidTrustBinding)?;
    let trust_binding_digest = binding
        .identity_digest()
        .map_err(|_| WitnessVerificationError::InvalidTrustBinding)?;

    let verified_at_ms = observation_proof
        .verified_at_ms
        .max(classification_proof.verified_at_ms)
        .max(challenge_receipt.verified_at_ms);
    let valid_until_ms = observation
        .expires_at_ms
        .min(observation_proof.valid_until_ms)
        .min(classification_proof.valid_until_ms)
        .min(challenge_receipt.challenge.expires_at_ms)
        .min(context.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(WitnessVerificationError::WitnessEvidenceExpired);
    }

    let qualification_digest = qualification_digest(
        context_digest,
        challenge_digest,
        source_head_digest,
        observation_digest,
        trust_binding_digest,
    );
    let evidence_digest = evidence_digest(
        qualification_digest,
        observation_proof,
        classification_proof,
    );
    let witness_verification_ref = format!(
        "authority-witness-evidence:{EVIDENCE_PROFILE}:{}",
        digest_hex(evidence_digest)
    );
    let trust_verification_ref = format!(
        "authority-witness-trust:{EVIDENCE_PROFILE}:{}",
        digest_hex(evidence_digest)
    );

    let witness = VerifiedAuthorityHeadWitness {
        observation: observation.clone(),
        verified_observation_proof_ref: observation.observation_proof_ref.clone(),
        verified_observer_id: observation.observer_id.clone(),
        verified_trust_domain_id: classification_proof.trust_domain_id.clone(),
        verified_trust_domain_ref: classification_proof.trust_domain_ref.clone(),
        verification_ref: witness_verification_ref,
        verified_at_ms,
    };
    let trust_binding = VerifiedWitnessTrustBinding {
        binding,
        verified_classification_proof_ref: classification_proof
            .verified_classification_proof_ref
            .clone(),
        verified_trust_verifier_ref: classification_proof
            .verified_trust_verifier_ref
            .clone(),
        verification_ref: trust_verification_ref,
        verified_at_ms,
        valid_until_ms,
    };

    Ok(QualifiedWitnessEvidence {
        witness,
        trust_binding,
        observation_digest,
        observation_profile: WITNESS_IDENTITY_PROFILE.into(),
        trust_binding_digest,
        trust_binding_profile: WITNESS_TRUST_BINDING_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_PROFILE.into(),
        verified_at_ms,
        valid_until_ms,
    })
}

fn bind_observation_to_source(
    observation: &AuthorityHeadWitnessObservation,
    source: &AuthoritySourceHeadAttestation,
    source_head_digest: Digest32,
    challenge_digest: Digest32,
) -> Result<(), WitnessVerificationError> {
    if observation.subject != source.subject
        || observation.authoritative_source_ref != source.authoritative_source_ref
        || observation.source_head_digest != source_head_digest
        || observation.head_generation != source.head_generation
        || observation.head_transition_digest != source.head_transition_digest
        || observation.head_status_record_ref != source.head_status_record_ref
        || observation.challenge_digest != challenge_digest
    {
        return Err(WitnessVerificationError::ObservationSourceHeadMismatch);
    }
    if observation.observed_at_ms < source.responded_at_ms {
        return Err(WitnessVerificationError::ObservationPredatesSourceResponse);
    }
    Ok(())
}

fn qualification_digest(
    context_digest: Digest32,
    challenge_digest: Digest32,
    source_head_digest: Digest32,
    observation_digest: Digest32,
    trust_binding_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, CONTEXT_POLICY_PROFILE.as_bytes());
    frame(&mut hasher, &context_digest.0);
    frame(&mut hasher, &challenge_digest.0);
    frame(&mut hasher, &source_head_digest.0);
    frame(&mut hasher, WITNESS_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &observation_digest.0);
    frame(&mut hasher, WITNESS_TRUST_BINDING_PROFILE.as_bytes());
    frame(&mut hasher, &trust_binding_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    observation: &VerifiedWitnessObservationProof,
    classification: &VerifiedWitnessTrustClassificationProof,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, observation.observation_verifier_ref.as_bytes());
    frame(&mut hasher, observation.verification_ref.as_bytes());
    frame(&mut hasher, &observation.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &observation.valid_until_ms.to_le_bytes());
    frame(&mut hasher, classification.verified_trust_verifier_ref.as_bytes());
    frame(&mut hasher, classification.verification_ref.as_bytes());
    frame(&mut hasher, &classification.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &classification.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn validate_window(
    effective_at_ms: u64,
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), ()> {
    if effective_at_ms == 0
        || verified_at_ms < effective_at_ms
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms < verified_at_ms
    {
        Err(())
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), WitnessVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(WitnessVerificationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), WitnessVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_PROFILE_BYTES {
        Err(WitnessVerificationError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn digest_hex(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum WitnessVerificationError {
    WrongObservationProofProtocol,
    WrongTrustClassificationProtocol,
    InvalidReference,
    InvalidProfile,
    InvalidVerificationTime,
    InvalidContextPolicy,
    InvalidChallenge,
    InvalidSourceHead,
    InvalidObservation,
    InvalidWitnessTrustPolicy,
    MissingWitnessTrustPolicy,
    MissingWitnessTrustVerifier,
    ObservationIdentityMismatch,
    ObserverIdentityMismatch,
    ObservationProofRefMismatch,
    InvalidObservationProofWindow,
    WitnessTrustPolicyMismatch,
    ClassificationObserverMismatch,
    ClassificationDomainMismatch,
    ClassificationProofRefMismatch,
    TrustVerifierMismatch,
    InvalidTrustClassificationWindow,
    ChallengeContextMismatch,
    ObservationSourceHeadMismatch,
    ObservationPredatesSourceResponse,
    LossyWitnessProjection,
    InvalidTrustBinding,
    WitnessEvidenceExpired,
}

impl fmt::Display for WitnessVerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongObservationProofProtocol => "wrong witness observation-proof protocol",
            Self::WrongTrustClassificationProtocol => "wrong witness trust-classification protocol",
            Self::InvalidReference => "invalid witness verifier reference",
            Self::InvalidProfile => "invalid witness verifier profile",
            Self::InvalidVerificationTime => "invalid witness qualification time",
            Self::InvalidContextPolicy => "invalid witness trust-context policy",
            Self::InvalidChallenge => "invalid witness coverage challenge",
            Self::InvalidSourceHead => "invalid authenticated source head",
            Self::InvalidObservation => "invalid witness observation",
            Self::InvalidWitnessTrustPolicy => "invalid witness trust policy identity",
            Self::MissingWitnessTrustPolicy => "witness qualification requires an exact witness trust policy",
            Self::MissingWitnessTrustVerifier => "witness qualification requires an exact trust verifier",
            Self::ObservationIdentityMismatch => "observation proof belongs to another witness observation",
            Self::ObserverIdentityMismatch => "observation proof authenticated another observer",
            Self::ObservationProofRefMismatch => "observation proof reference mismatch",
            Self::InvalidObservationProofWindow => "witness observation proof is stale or causally invalid",
            Self::WitnessTrustPolicyMismatch => "trust classification belongs to another witness trust policy",
            Self::ClassificationObserverMismatch => "trust classification belongs to another observer",
            Self::ClassificationDomainMismatch => "trust classification belongs to another trust domain",
            Self::ClassificationProofRefMismatch => "trust classification proof reference mismatch",
            Self::TrustVerifierMismatch => "trust classification came from the wrong verifier domain",
            Self::InvalidTrustClassificationWindow => "witness trust classification is stale or invalid",
            Self::ChallengeContextMismatch => "witness challenge belongs to another trust context",
            Self::ObservationSourceHeadMismatch => "witness observation does not match the authenticated source head/challenge",
            Self::ObservationPredatesSourceResponse => "witness observation predates the source response it claims to observe",
            Self::LossyWitnessProjection => "legacy witness projection would widen verifier validity",
            Self::InvalidTrustBinding => "locally constructed witness trust binding is invalid",
            Self::WitnessEvidenceExpired => "qualified witness evidence is not currently reusable",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for WitnessVerificationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest,
    };
    use mycelix_authority_state_coverage::{
        AuthoritySourceHeadAttestation, PROTOCOL_VERSION as COVERAGE_PROTOCOL,
    };
    use mycelix_authority_state_coverage_context::{
        CoverageChallenge, PROTOCOL_VERSION as CONTEXT_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test:authority".into(),
            subject_id: "grant:test".into(),
            identity: ProfiledDigest {
                digest: d(1),
                profile: "grant-v1".into(),
            },
        }
    }

    fn context() -> CoverageTrustContextPolicy {
        CoverageTrustContextPolicy {
            protocol_version: CONTEXT_PROTOCOL.into(),
            context_policy_id: "context:test".into(),
            institution_ref: "institution:test".into(),
            jurisdiction_ref: None,
            rulebook_ref: "rulebook:test".into(),
            coverage_policy: ProfiledDigest {
                digest: d(2),
                profile: mycelix_authority_state_coverage::POLICY_IDENTITY_PROFILE.into(),
            },
            witness_trust_policy: Some(ProfiledDigest {
                digest: d(3),
                profile: "witness-trust-policy-v1".into(),
            }),
            witness_trust_verifier_ref: Some("trust-verifier:test".into()),
            max_challenge_lifetime_ms: 1_000,
            valid_from_ms: 100,
            valid_until_ms: 1_000,
            authority_ref: "authority:context".into(),
            policy_proof_ref: "proof:context".into(),
        }
    }

    fn challenge(context: &CoverageTrustContextPolicy) -> VerifiedCoverageChallenge {
        let challenge = CoverageChallenge {
            protocol_version: CONTEXT_PROTOCOL.into(),
            context_policy_digest: context.identity_digest().unwrap(),
            context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
            coverage_policy_digest: context.coverage_policy.digest,
            coverage_policy_profile: context.coverage_policy.profile.clone(),
            subject: subject(),
            nonce_digest: d(4),
            randomness_proof_ref: "entropy:1".into(),
            issued_at_ms: 110,
            expires_at_ms: 900,
            challenge_issuer_ref: "did:mycelix:probe".into(),
        };
        VerifiedCoverageChallenge {
            verified_nonce_digest: challenge.nonce_digest,
            verified_randomness_proof_ref: challenge.randomness_proof_ref.clone(),
            verified_challenge_issuer_ref: challenge.challenge_issuer_ref.clone(),
            challenge,
            verification_ref: "verify:challenge".into(),
            verified_at_ms: 120,
        }
    }

    fn source(challenge: &VerifiedCoverageChallenge) -> VerifiedAuthoritySourceHead {
        let attestation = AuthoritySourceHeadAttestation {
            protocol_version: COVERAGE_PROTOCOL.into(),
            subject: subject(),
            authoritative_source_ref: "authority-source:test".into(),
            source_identity: ProfiledDigest {
                digest: d(5),
                profile: "source-key-v1".into(),
            },
            head_generation: 2,
            head_transition_digest: d(6),
            head_status_record_ref: "state:2".into(),
            source_chain_head_ref: "chain:2".into(),
            challenge_digest: challenge.challenge.identity_digest().unwrap(),
            responded_at_ms: 130,
            expires_at_ms: 800,
            source_proof_ref: "proof:source".into(),
        };
        VerifiedAuthoritySourceHead {
            verified_source_identity: attestation.source_identity.clone(),
            verified_source_proof_ref: attestation.source_proof_ref.clone(),
            verification_ref: "verify:source".into(),
            verified_at_ms: 140,
            attestation,
        }
    }

    fn observation(source: &VerifiedAuthoritySourceHead, challenge: &VerifiedCoverageChallenge) -> AuthorityHeadWitnessObservation {
        AuthorityHeadWitnessObservation {
            protocol_version: COVERAGE_PROTOCOL.into(),
            subject: subject(),
            authoritative_source_ref: source.attestation.authoritative_source_ref.clone(),
            source_head_digest: source.attestation.identity_digest().unwrap(),
            head_generation: source.attestation.head_generation,
            head_transition_digest: source.attestation.head_transition_digest,
            head_status_record_ref: source.attestation.head_status_record_ref.clone(),
            challenge_digest: challenge.challenge.identity_digest().unwrap(),
            observer_id: "observer:1".into(),
            trust_domain_id: "domain:a".into(),
            observed_at_ms: 150,
            expires_at_ms: 500,
            observation_proof_ref: "proof:observation".into(),
        }
    }

    fn observation_proof(observation: &AuthorityHeadWitnessObservation) -> VerifiedWitnessObservationProof {
        VerifiedWitnessObservationProof {
            protocol_version: OBSERVATION_PROOF_PROTOCOL.into(),
            observation_digest: observation.identity_digest().unwrap(),
            observation_profile: WITNESS_IDENTITY_PROFILE.into(),
            verified_observer_id: observation.observer_id.clone(),
            verified_observation_proof_ref: observation.observation_proof_ref.clone(),
            observation_verifier_ref: "verify:observer-signature".into(),
            verification_ref: "verify:observation:1".into(),
            verified_at_ms: 160,
            valid_until_ms: 600,
        }
    }

    fn classification(context: &CoverageTrustContextPolicy, observation: &AuthorityHeadWitnessObservation) -> VerifiedWitnessTrustClassificationProof {
        VerifiedWitnessTrustClassificationProof {
            protocol_version: TRUST_CLASSIFICATION_PROOF_PROTOCOL.into(),
            witness_trust_policy: context.witness_trust_policy.clone().unwrap(),
            observer_id: observation.observer_id.clone(),
            trust_domain_id: observation.trust_domain_id.clone(),
            trust_domain_ref: "trust-domain:a".into(),
            classification_proof_ref: "proof:classification".into(),
            verified_classification_proof_ref: "proof:classification".into(),
            verified_trust_verifier_ref: context.witness_trust_verifier_ref.clone().unwrap(),
            verification_ref: "verify:classification:1".into(),
            verified_at_ms: 155,
            valid_until_ms: 600,
        }
    }

    #[test]
    fn exact_observation_and_classification_qualify_together() {
        let context = context();
        let challenge = challenge(&context);
        let source = source(&challenge);
        let observation = observation(&source, &challenge);
        let observation_proof = observation_proof(&observation);
        let classification = classification(&context, &observation);
        let qualified = qualify_witness_evidence(
            &context,
            &challenge,
            &source,
            &observation,
            &observation_proof,
            &classification,
            200,
        )
        .unwrap();
        assert_eq!(qualified.to_verified_witness().observation, observation);
        assert_eq!(
            qualified.to_verified_trust_binding().binding.trust_domain_id,
            "domain:a"
        );
    }

    #[test]
    fn wrong_trust_domain_denies() {
        let context = context();
        let challenge = challenge(&context);
        let source = source(&challenge);
        let observation = observation(&source, &challenge);
        let observation_proof = observation_proof(&observation);
        let mut classification = classification(&context, &observation);
        classification.trust_domain_id = "domain:b".into();
        assert_eq!(
            qualify_witness_evidence(
                &context,
                &challenge,
                &source,
                &observation,
                &observation_proof,
                &classification,
                200,
            )
            .unwrap_err(),
            WitnessVerificationError::ClassificationDomainMismatch
        );
    }

    #[test]
    fn tighter_proof_horizon_cannot_be_projected_away() {
        let context = context();
        let challenge = challenge(&context);
        let source = source(&challenge);
        let observation = observation(&source, &challenge);
        let mut observation_proof = observation_proof(&observation);
        observation_proof.valid_until_ms = 400;
        let classification = classification(&context, &observation);
        assert_eq!(
            qualify_witness_evidence(
                &context,
                &challenge,
                &source,
                &observation,
                &observation_proof,
                &classification,
                200,
            )
            .unwrap_err(),
            WitnessVerificationError::LossyWitnessProjection
        );
    }
}
