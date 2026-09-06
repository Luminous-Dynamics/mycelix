// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure verification for operational coverage/context policy receipts.
//!
//! Immutable policy-record authenticity, institutional adoption, and generation-
//! bound currentness are distinct facts. This layer proves the first two through
//! independent verifier receipts and projects the legacy #94/#96 policy ABIs only
//! when doing so cannot widen verifier validity. #115 remains responsible for
//! generation-bound currentness.

use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, VerifiedAuthorityCoveragePolicy, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-operational-policy-verifier-v0.1";
pub const RECORD_PROOF_PROTOCOL: &str = "mycelix-authority-policy-record-proof-v0.1";
pub const ADOPTION_PROOF_PROTOCOL: &str = "mycelix-authority-policy-adoption-proof-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-operational-policy-verification-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-authority-operational-policy-verification-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/operational-policy-verification/v1";
const DOMAIN_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-policy-verification-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Evidence-shaped authentication of one exact immutable policy record.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedPolicyRecordProof {
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

/// Evidence-shaped verification that the institution/rulebook authority adopted
/// one exact policy identity under the policy's exact adoption-proof reference.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedPolicyAdoptionProof {
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

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCoveragePolicyVerification {
    receipt: VerifiedAuthorityCoveragePolicy,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedCoveragePolicyVerification {
    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        QUALIFICATION_PROFILE
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn to_verified_policy(&self) -> VerifiedAuthorityCoveragePolicy {
        self.receipt.clone()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedContextPolicyVerification {
    receipt: VerifiedCoverageTrustContextPolicy,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedContextPolicyVerification {
    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        QUALIFICATION_PROFILE
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn to_verified_policy(&self) -> VerifiedCoverageTrustContextPolicy {
        self.receipt.clone()
    }
}

pub fn qualify_coverage_policy(
    policy: &AuthorityCoveragePolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedPolicyRecordProof,
    adoption_proof: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedCoveragePolicyVerification, PolicyVerificationError> {
    policy
        .validate()
        .map_err(|_| PolicyVerificationError::InvalidCoveragePolicy)?;
    if !policy.is_active_at(now_ms) {
        return Err(PolicyVerificationError::PolicyNotSemanticallyActive);
    }
    let digest = policy
        .identity_digest()
        .map_err(|_| PolicyVerificationError::InvalidCoveragePolicy)?;
    let common = qualify_common(
        digest,
        POLICY_IDENTITY_PROFILE,
        policy_record_ref,
        policy.authority_ref.as_str(),
        policy.policy_proof_ref.as_str(),
        policy.valid_from_ms,
        policy.valid_until_ms,
        record_proof,
        adoption_proof,
        now_ms,
    )?;
    let receipt = VerifiedAuthorityCoveragePolicy {
        policy: policy.clone(),
        policy_record_ref: policy_record_ref.into(),
        verified_policy_proof_ref: policy.policy_proof_ref.clone(),
        verified_authority_ref: policy.authority_ref.clone(),
        verification_ref: verification_ref(common.evidence_digest),
        verified_at_ms: common.verified_at_ms,
    };
    Ok(QualifiedCoveragePolicyVerification {
        receipt,
        policy_digest: digest,
        qualification_digest: common.qualification_digest,
        evidence_digest: common.evidence_digest,
        verified_at_ms: common.verified_at_ms,
        valid_until_ms: common.valid_until_ms,
    })
}

pub fn qualify_context_policy(
    policy: &CoverageTrustContextPolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedPolicyRecordProof,
    adoption_proof: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedContextPolicyVerification, PolicyVerificationError> {
    policy
        .validate()
        .map_err(|_| PolicyVerificationError::InvalidContextPolicy)?;
    if !policy.active_at(now_ms) {
        return Err(PolicyVerificationError::PolicyNotSemanticallyActive);
    }
    let digest = policy
        .identity_digest()
        .map_err(|_| PolicyVerificationError::InvalidContextPolicy)?;
    let common = qualify_common(
        digest,
        CONTEXT_POLICY_PROFILE,
        policy_record_ref,
        policy.authority_ref.as_str(),
        policy.policy_proof_ref.as_str(),
        policy.valid_from_ms,
        policy.valid_until_ms,
        record_proof,
        adoption_proof,
        now_ms,
    )?;
    let receipt = VerifiedCoverageTrustContextPolicy {
        policy: policy.clone(),
        policy_record_ref: policy_record_ref.into(),
        verified_authority_ref: policy.authority_ref.clone(),
        verified_policy_proof_ref: policy.policy_proof_ref.clone(),
        verification_ref: verification_ref(common.evidence_digest),
        verified_at_ms: common.verified_at_ms,
    };
    Ok(QualifiedContextPolicyVerification {
        receipt,
        policy_digest: digest,
        qualification_digest: common.qualification_digest,
        evidence_digest: common.evidence_digest,
        verified_at_ms: common.verified_at_ms,
        valid_until_ms: common.valid_until_ms,
    })
}

struct CommonQualification {
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

#[allow(clippy::too_many_arguments)]
fn qualify_common(
    policy_digest: Digest32,
    policy_profile: &str,
    policy_record_ref: &str,
    authority_ref: &str,
    policy_proof_ref: &str,
    policy_valid_from_ms: u64,
    policy_valid_until_ms: u64,
    record_proof: &VerifiedPolicyRecordProof,
    adoption_proof: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<CommonQualification, PolicyVerificationError> {
    if now_ms == 0 || policy_digest.is_zero() {
        return Err(PolicyVerificationError::InvalidVerificationTime);
    }
    require_profile(policy_profile)?;
    require_ref(policy_record_ref)?;
    require_ref(authority_ref)?;
    require_ref(policy_proof_ref)?;

    verify_record_proof(
        record_proof,
        policy_digest,
        policy_profile,
        policy_record_ref,
        now_ms,
    )?;
    verify_adoption_proof(
        adoption_proof,
        policy_digest,
        policy_profile,
        authority_ref,
        policy_proof_ref,
        now_ms,
    )?;

    let verified_at_ms = record_proof
        .verified_at_ms
        .max(adoption_proof.verified_at_ms);
    let valid_until_ms = record_proof
        .valid_until_ms
        .min(adoption_proof.valid_until_ms);
    if verified_at_ms < policy_valid_from_ms
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
    {
        return Err(PolicyVerificationError::PolicyProofOutsideUsableWindow);
    }

    // The legacy policy receipts used by #94/#96/#116 have no independent proof
    // expiry field. Projecting them when a proof expires earlier than the semantic
    // policy would silently widen verification authority. Fail closed instead.
    if policy_valid_until_ms > valid_until_ms {
        return Err(PolicyVerificationError::LossyPolicyProjection);
    }

    let qualification_digest = qualification_digest(policy_digest, policy_profile);
    let evidence_digest = evidence_digest(
        qualification_digest,
        policy_record_ref,
        record_proof,
        adoption_proof,
    );
    Ok(CommonQualification {
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_record_proof(
    proof: &VerifiedPolicyRecordProof,
    policy_digest: Digest32,
    policy_profile: &str,
    policy_record_ref: &str,
    now_ms: u64,
) -> Result<(), PolicyVerificationError> {
    if proof.protocol_version != RECORD_PROOF_PROTOCOL {
        return Err(PolicyVerificationError::WrongRecordProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != policy_profile {
        return Err(PolicyVerificationError::RecordPolicyIdentityMismatch);
    }
    require_profile(&proof.policy_profile)?;
    for value in [
        proof.policy_record_ref.as_str(),
        proof.record_proof_ref.as_str(),
        proof.record_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if proof.policy_record_ref != policy_record_ref {
        return Err(PolicyVerificationError::PolicyRecordRefMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
        .map_err(|_| PolicyVerificationError::InvalidRecordProofWindow)
}

fn verify_adoption_proof(
    proof: &VerifiedPolicyAdoptionProof,
    policy_digest: Digest32,
    policy_profile: &str,
    authority_ref: &str,
    policy_proof_ref: &str,
    now_ms: u64,
) -> Result<(), PolicyVerificationError> {
    if proof.protocol_version != ADOPTION_PROOF_PROTOCOL {
        return Err(PolicyVerificationError::WrongAdoptionProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != policy_profile {
        return Err(PolicyVerificationError::AdoptionPolicyIdentityMismatch);
    }
    require_profile(&proof.policy_profile)?;
    for value in [
        proof.verified_authority_ref.as_str(),
        proof.verified_policy_proof_ref.as_str(),
        proof.authority_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if proof.verified_authority_ref != authority_ref {
        return Err(PolicyVerificationError::PolicyAuthorityMismatch);
    }
    if proof.verified_policy_proof_ref != policy_proof_ref {
        return Err(PolicyVerificationError::PolicyAdoptionProofMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
        .map_err(|_| PolicyVerificationError::InvalidAdoptionProofWindow)
}

fn qualification_digest(policy_digest: Digest32, policy_profile: &str) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, policy_profile.as_bytes());
    frame(&mut hasher, &policy_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    policy_record_ref: &str,
    record: &VerifiedPolicyRecordProof,
    adoption: &VerifiedPolicyAdoptionProof,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, policy_record_ref.as_bytes());
    frame(&mut hasher, record.record_proof_ref.as_bytes());
    frame(&mut hasher, record.record_verifier_ref.as_bytes());
    frame(&mut hasher, record.verification_ref.as_bytes());
    frame(&mut hasher, &record.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &record.valid_until_ms.to_le_bytes());
    frame(&mut hasher, adoption.authority_verifier_ref.as_bytes());
    frame(&mut hasher, adoption.verification_ref.as_bytes());
    frame(&mut hasher, &adoption.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &adoption.valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn verification_ref(digest: Digest32) -> String {
    format!(
        "operational-policy-evidence:{EVIDENCE_PROFILE}:{}",
        digest_hex(digest)
    )
}

fn validate_window(verified_at_ms: u64, valid_until_ms: u64, now_ms: u64) -> Result<(), ()> {
    if verified_at_ms == 0
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms < verified_at_ms
    {
        Err(())
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), PolicyVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(PolicyVerificationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), PolicyVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_PROFILE_BYTES {
        Err(PolicyVerificationError::InvalidProfile)
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
pub enum PolicyVerificationError {
    InvalidCoveragePolicy,
    InvalidContextPolicy,
    PolicyNotSemanticallyActive,
    WrongRecordProofProtocol,
    WrongAdoptionProofProtocol,
    InvalidReference,
    InvalidProfile,
    InvalidVerificationTime,
    RecordPolicyIdentityMismatch,
    AdoptionPolicyIdentityMismatch,
    PolicyRecordRefMismatch,
    PolicyAuthorityMismatch,
    PolicyAdoptionProofMismatch,
    InvalidRecordProofWindow,
    InvalidAdoptionProofWindow,
    PolicyProofOutsideUsableWindow,
    LossyPolicyProjection,
}

impl fmt::Display for PolicyVerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidCoveragePolicy => "invalid operational coverage policy",
            Self::InvalidContextPolicy => "invalid operational coverage trust-context policy",
            Self::PolicyNotSemanticallyActive => "operational policy is not semantically active",
            Self::WrongRecordProofProtocol => "wrong operational policy record-proof protocol",
            Self::WrongAdoptionProofProtocol => "wrong operational policy adoption-proof protocol",
            Self::InvalidReference => "invalid operational policy verifier reference",
            Self::InvalidProfile => "invalid operational policy verifier profile",
            Self::InvalidVerificationTime => "invalid operational policy qualification time",
            Self::RecordPolicyIdentityMismatch => "record proof belongs to another policy identity",
            Self::AdoptionPolicyIdentityMismatch => {
                "adoption proof belongs to another policy identity"
            }
            Self::PolicyRecordRefMismatch => "record proof authenticated another policy record",
            Self::PolicyAuthorityMismatch => {
                "policy adoption proof authenticated another authority"
            }
            Self::PolicyAdoptionProofMismatch => "policy adoption proof reference mismatch",
            Self::InvalidRecordProofWindow => "policy record proof is stale or invalid",
            Self::InvalidAdoptionProofWindow => "policy adoption proof is stale or invalid",
            Self::PolicyProofOutsideUsableWindow => {
                "policy proof timing cannot support current semantic use"
            }
            Self::LossyPolicyProjection => {
                "legacy policy receipt would outlive independent verifier evidence"
            }
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for PolicyVerificationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest};
    use mycelix_authority_state_coverage::{
        CoverageMode, PROTOCOL_VERSION as COVERAGE_PROTOCOL,
    };
    use mycelix_authority_state_coverage_context::PROTOCOL_VERSION as CONTEXT_PROTOCOL;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn coverage() -> AuthorityCoveragePolicy {
        AuthorityCoveragePolicy {
            protocol_version: COVERAGE_PROTOCOL.into(),
            policy_id: "coverage:test".into(),
            namespace: "institution:test:authority".into(),
            allowed_subject_kinds: vec![AuthoritySubjectKind::AuthorityGrant],
            authoritative_source_ref: "authority-source:test".into(),
            source_identity: ProfiledDigest {
                digest: d(1),
                profile: "source-key-v1".into(),
            },
            mode: CoverageMode::DirectSource,
            max_source_age_ms: 100,
            max_witness_age_ms: 0,
            max_coverage_lease_ms: 100,
            valid_from_ms: 100,
            valid_until_ms: 800,
            authority_ref: "authority:policy".into(),
            policy_proof_ref: "proof:coverage-adoption".into(),
        }
    }

    fn context(coverage: &AuthorityCoveragePolicy) -> CoverageTrustContextPolicy {
        CoverageTrustContextPolicy {
            protocol_version: CONTEXT_PROTOCOL.into(),
            context_policy_id: "context:test".into(),
            institution_ref: "institution:test".into(),
            jurisdiction_ref: None,
            rulebook_ref: "rulebook:test".into(),
            coverage_policy: ProfiledDigest {
                digest: coverage.identity_digest().unwrap(),
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
            witness_trust_policy: None,
            witness_trust_verifier_ref: None,
            max_challenge_lifetime_ms: 100,
            valid_from_ms: 100,
            valid_until_ms: 800,
            authority_ref: "authority:policy".into(),
            policy_proof_ref: "proof:context-adoption".into(),
        }
    }

    fn record(
        digest: Digest32,
        profile: &str,
        record_ref: &str,
    ) -> VerifiedPolicyRecordProof {
        VerifiedPolicyRecordProof {
            protocol_version: RECORD_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: profile.into(),
            policy_record_ref: record_ref.into(),
            record_proof_ref: format!("proof:record:{record_ref}"),
            record_verifier_ref: "verifier:policy-record".into(),
            verification_ref: format!("verify:record:{record_ref}"),
            verified_at_ms: 150,
            valid_until_ms: 900,
        }
    }

    fn adoption(
        digest: Digest32,
        profile: &str,
        authority: &str,
        proof_ref: &str,
    ) -> VerifiedPolicyAdoptionProof {
        VerifiedPolicyAdoptionProof {
            protocol_version: ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: profile.into(),
            verified_authority_ref: authority.into(),
            verified_policy_proof_ref: proof_ref.into(),
            authority_verifier_ref: "verifier:policy-adoption".into(),
            verification_ref: "verify:policy-adoption".into(),
            verified_at_ms: 160,
            valid_until_ms: 900,
        }
    }

    #[test]
    fn exact_coverage_policy_qualifies() {
        let policy = coverage();
        let digest = policy.identity_digest().unwrap();
        let record = record(digest, POLICY_IDENTITY_PROFILE, "record:coverage");
        let adoption = adoption(
            digest,
            POLICY_IDENTITY_PROFILE,
            &policy.authority_ref,
            &policy.policy_proof_ref,
        );
        let qualified =
            qualify_coverage_policy(&policy, "record:coverage", &record, &adoption, 200)
                .unwrap();
        assert_eq!(qualified.to_verified_policy().policy, policy);
    }

    #[test]
    fn exact_context_policy_qualifies() {
        let coverage = coverage();
        let policy = context(&coverage);
        let digest = policy.identity_digest().unwrap();
        let record = record(digest, CONTEXT_POLICY_PROFILE, "record:context");
        let adoption = adoption(
            digest,
            CONTEXT_POLICY_PROFILE,
            &policy.authority_ref,
            &policy.policy_proof_ref,
        );
        assert!(
            qualify_context_policy(&policy, "record:context", &record, &adoption, 200).is_ok()
        );
    }

    #[test]
    fn wrong_adoption_authority_denies() {
        let policy = coverage();
        let digest = policy.identity_digest().unwrap();
        let record = record(digest, POLICY_IDENTITY_PROFILE, "record:coverage");
        let adoption = adoption(
            digest,
            POLICY_IDENTITY_PROFILE,
            "authority:other",
            &policy.policy_proof_ref,
        );
        assert_eq!(
            qualify_coverage_policy(&policy, "record:coverage", &record, &adoption, 200)
                .unwrap_err(),
            PolicyVerificationError::PolicyAuthorityMismatch
        );
    }

    #[test]
    fn tighter_proof_horizon_cannot_be_projected_away() {
        let policy = coverage();
        let digest = policy.identity_digest().unwrap();
        let mut record = record(digest, POLICY_IDENTITY_PROFILE, "record:coverage");
        record.valid_until_ms = 700;
        let adoption = adoption(
            digest,
            POLICY_IDENTITY_PROFILE,
            &policy.authority_ref,
            &policy.policy_proof_ref,
        );
        assert_eq!(
            qualify_coverage_policy(&policy, "record:coverage", &record, &adoption, 200)
                .unwrap_err(),
            PolicyVerificationError::LossyPolicyProjection
        );
    }
}
