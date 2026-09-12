//! Institution-adopted provider-profile trust-root semantics.
//!
//! `ProviderProfileTrustRoot` by itself is deliberately caller-constructible and
//! therefore cannot prove institutional adoption. This crate binds one exact
//! provider-profile signing root to institution/rulebook/deployment semantics and
//! independent record + adoption evidence. Generation-bound currentness remains
//! a separate child theorem using the shared authority freshness kernel.

use mycelix_institutional_core::{Digest32, InstitutionId, JurisdictionId, RulebookRef};
use mycelix_integration_core::{ConnectorInstanceId, DigestAlgorithm, ExternalSystemId};
use mycelix_integration_execution_binding::ProviderProfileTrustRoot;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const PROTOCOL_VERSION: &str = "mycelix-integration-provider-trust-policy-v0.1";
pub const RECORD_PROOF_PROTOCOL: &str =
    "mycelix-integration-provider-trust-policy-record-proof-v0.1";
pub const ADOPTION_PROOF_PROTOCOL: &str =
    "mycelix-integration-provider-trust-policy-adoption-proof-v0.1";
pub const POLICY_IDENTITY_PROFILE: &str =
    "mycelix-integration-provider-trust-policy-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-provider-trust-policy-qualified-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-integration-provider-trust-policy-evidence-v1-blake3-framed";
pub const FRESHNESS_NAMESPACE_SUFFIX: &str = "integration-provider-profile-signing";

const DOMAIN_POLICY: &[u8] = b"mycelix/integration/provider-trust-policy/v1";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/provider-trust-policy/qualified/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/integration/provider-trust-policy/evidence/v1";
const MAX_TEXT_BYTES: usize = 2048;

/// Institution-adopted statement selecting the exact signing root trusted to
/// authenticate provider execution profiles for one connector/system domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationProviderTrustPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub signer_key_id: String,
    pub verifying_key: [u8; 32],
    pub generation: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub authority_ref: String,
    pub policy_proof_ref: String,
}

impl IntegrationProviderTrustPolicy {
    pub fn validate(&self) -> Result<(), ProviderTrustPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ProviderTrustPolicyError::WrongProtocolVersion);
        }
        require_text(&self.policy_id)?;
        require_text(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_text(jurisdiction.as_str())?;
        }
        self.rulebook
            .validate()
            .map_err(|_| ProviderTrustPolicyError::InvalidRulebook)?;
        require_text(self.connector_instance.as_str())?;
        require_text(self.system.as_str())?;
        require_text(&self.signer_key_id)?;
        require_text(&self.authority_ref)?;
        require_text(&self.policy_proof_ref)?;
        if self.generation == 0 {
            return Err(ProviderTrustPolicyError::InvalidGeneration);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(ProviderTrustPolicyError::InvalidPolicyWindow);
        }
        self.candidate_trust_root()?;
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn candidate_trust_root(
        &self,
    ) -> Result<ProviderProfileTrustRoot, ProviderTrustPolicyError> {
        ProviderProfileTrustRoot::new(
            self.signer_key_id.clone(),
            self.verifying_key,
            self.generation,
        )
        .map_err(|_| ProviderTrustPolicyError::InvalidTrustRoot)
    }

    pub fn identity_digest(&self) -> Result<Digest32, ProviderTrustPolicyError> {
        self.validate()?;
        let root = self.candidate_trust_root()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_POLICY);
        frame(&mut h, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut h, self.protocol_version.as_bytes());
        frame(&mut h, self.policy_id.as_bytes());
        frame(&mut h, self.institution.as_str().as_bytes());
        frame_optional_text(
            &mut h,
            self.jurisdiction.as_ref().map(|value| value.as_str()),
        );
        frame(&mut h, self.rulebook.id.as_str().as_bytes());
        frame(&mut h, self.rulebook.version.as_bytes());
        frame(&mut h, &self.rulebook.digest.0);
        frame(&mut h, self.connector_instance.as_str().as_bytes());
        frame(&mut h, self.system.as_str().as_bytes());
        frame(&mut h, self.signer_key_id.as_bytes());
        frame(&mut h, &self.verifying_key);
        frame(&mut h, &self.generation.to_le_bytes());
        frame_commitment(&mut h, root.root_commitment());
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        frame(&mut h, self.authority_ref.as_bytes());
        frame(&mut h, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*h.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedProviderTrustPolicyRecordProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub policy_record_ref: String,
    pub record_proof_ref: String,
    pub record_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedProviderTrustPolicyAdoptionProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub verified_authority_ref: String,
    pub verified_policy_proof_ref: String,
    pub authority_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Process-local positive proof that exact provider-root semantics, immutable
/// record evidence and institutional adoption evidence all refer to the same
/// policy. The retained root is intentionally not serializable/deserializable.
#[derive(Clone, Debug)]
pub struct QualifiedAdoptedProviderTrustPolicy {
    policy: IntegrationProviderTrustPolicy,
    candidate_root: ProviderProfileTrustRoot,
    policy_record_ref: String,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedAdoptedProviderTrustPolicy {
    pub fn policy(&self) -> &IntegrationProviderTrustPolicy {
        &self.policy
    }

    pub fn candidate_root(&self) -> &ProviderProfileTrustRoot {
        &self.candidate_root
    }

    pub fn policy_record_ref(&self) -> &str {
        &self.policy_record_ref
    }

    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub fn evidence_profile(&self) -> &'static str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn institutional_adoption_evidence_bound_here(&self) -> bool {
        true
    }

    pub const fn exact_provider_root_semantics_bound_here(&self) -> bool {
        true
    }

    pub const fn record_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn adoption_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn generation_currentness_verified_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_adopted_provider_trust_policy(
    policy: &IntegrationProviderTrustPolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedProviderTrustPolicyRecordProof,
    adoption_proof: &VerifiedProviderTrustPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedAdoptedProviderTrustPolicy, ProviderTrustPolicyError> {
    policy.validate()?;
    if now_ms == 0 || !policy.is_active_at(now_ms) {
        return Err(ProviderTrustPolicyError::PolicyNotActive);
    }
    require_text(policy_record_ref)?;
    let policy_digest = policy.identity_digest()?;
    verify_record_proof(record_proof, policy_digest, policy_record_ref, now_ms)?;
    verify_adoption_proof(adoption_proof, policy, policy_digest, now_ms)?;

    let verified_at_ms = record_proof.verified_at_ms.max(adoption_proof.verified_at_ms);
    let valid_until_ms = policy
        .valid_until_ms
        .min(record_proof.valid_until_ms)
        .min(adoption_proof.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(ProviderTrustPolicyError::NoUsableQualificationWindow);
    }

    let candidate_root = policy.candidate_trust_root()?;
    let qualification_digest =
        qualification_digest(policy_digest, candidate_root.root_commitment());
    let evidence_digest = evidence_digest(
        qualification_digest,
        policy_record_ref,
        record_proof,
        adoption_proof,
        verified_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedAdoptedProviderTrustPolicy {
        policy: policy.clone(),
        candidate_root,
        policy_record_ref: policy_record_ref.to_owned(),
        policy_digest,
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_record_proof(
    proof: &VerifiedProviderTrustPolicyRecordProof,
    policy_digest: Digest32,
    policy_record_ref: &str,
    now_ms: u64,
) -> Result<(), ProviderTrustPolicyError> {
    if proof.protocol_version != RECORD_PROOF_PROTOCOL
        || proof.policy_digest != policy_digest
        || proof.policy_profile != POLICY_IDENTITY_PROFILE
        || proof.policy_record_ref != policy_record_ref
    {
        return Err(ProviderTrustPolicyError::RecordProofMismatch);
    }
    require_text(&proof.record_proof_ref)?;
    require_text(&proof.record_verifier_ref)?;
    require_text(&proof.verification_ref)?;
    validate_evidence_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn verify_adoption_proof(
    proof: &VerifiedProviderTrustPolicyAdoptionProof,
    policy: &IntegrationProviderTrustPolicy,
    policy_digest: Digest32,
    now_ms: u64,
) -> Result<(), ProviderTrustPolicyError> {
    if proof.protocol_version != ADOPTION_PROOF_PROTOCOL
        || proof.policy_digest != policy_digest
        || proof.policy_profile != POLICY_IDENTITY_PROFILE
        || proof.verified_authority_ref != policy.authority_ref
        || proof.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(ProviderTrustPolicyError::AdoptionProofMismatch);
    }
    require_text(&proof.authority_verifier_ref)?;
    require_text(&proof.verification_ref)?;
    validate_evidence_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn validate_evidence_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), ProviderTrustPolicyError> {
    if verified_at_ms == 0 || verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(ProviderTrustPolicyError::InvalidEvidenceWindow);
    }
    Ok(())
}

fn qualification_digest(
    policy_digest: Digest32,
    root_commitment: &mycelix_integration_core::ContentCommitment,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &policy_digest.0);
    frame_commitment(&mut h, root_commitment);
    Digest32(*h.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    policy_record_ref: &str,
    record: &VerifiedProviderTrustPolicyRecordProof,
    adoption: &VerifiedProviderTrustPolicyAdoptionProof,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_EVIDENCE);
    frame(&mut h, EVIDENCE_PROFILE.as_bytes());
    frame(&mut h, &qualification_digest.0);
    frame(&mut h, policy_record_ref.as_bytes());
    frame(&mut h, record.record_proof_ref.as_bytes());
    frame(&mut h, record.record_verifier_ref.as_bytes());
    frame(&mut h, record.verification_ref.as_bytes());
    frame(&mut h, adoption.authority_verifier_ref.as_bytes());
    frame(&mut h, adoption.verification_ref.as_bytes());
    frame(&mut h, &verified_at_ms.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame_commitment(
    h: &mut blake3::Hasher,
    commitment: &mycelix_integration_core::ContentCommitment,
) {
    frame(
        h,
        &[match commitment.algorithm {
            DigestAlgorithm::Sha256 => 1,
        }],
    );
    frame(h, &commitment.digest);
}

fn frame_optional_text(h: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            frame(h, &[1]);
            frame(h, value.as_bytes());
        }
        None => frame(h, &[0]),
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

fn require_text(value: &str) -> Result<(), ProviderTrustPolicyError> {
    if value.trim().is_empty()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(char::is_control)
    {
        Err(ProviderTrustPolicyError::InvalidText)
    } else {
        Ok(())
    }
}

#[derive(Debug, Error)]
pub enum ProviderTrustPolicyError {
    #[error("wrong provider trust policy protocol version")]
    WrongProtocolVersion,
    #[error("provider trust policy contains invalid text")]
    InvalidText,
    #[error("provider trust policy rulebook is invalid")]
    InvalidRulebook,
    #[error("provider trust policy generation must be non-zero")]
    InvalidGeneration,
    #[error("provider trust policy validity window is invalid")]
    InvalidPolicyWindow,
    #[error("provider trust policy root key/generation is invalid")]
    InvalidTrustRoot,
    #[error("provider trust policy is not active")]
    PolicyNotActive,
    #[error("provider trust policy record proof does not match")]
    RecordProofMismatch,
    #[error("provider trust policy adoption proof does not match")]
    AdoptionProofMismatch,
    #[error("provider trust policy evidence window is invalid")]
    InvalidEvidenceWindow,
    #[error("provider trust policy evidence leaves no usable qualification window")]
    NoUsableQualificationWindow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evidence_window_requires_live_evidence() {
        assert!(validate_evidence_window(10, 30, 20).is_ok());
        assert!(matches!(
            validate_evidence_window(10, 20, 20),
            Err(ProviderTrustPolicyError::InvalidEvidenceWindow)
        ));
    }

    #[test]
    fn zero_generation_is_rejected_before_root_use() {
        // Full typed/adopted fixtures are exercised by the currentness child.
        // This local regression freezes the policy-level zero-generation rule.
        assert!(matches!(
            ProviderProfileTrustRoot::new("provider-key", [0; 32], 0),
            Err(_)
        ));
    }
}
