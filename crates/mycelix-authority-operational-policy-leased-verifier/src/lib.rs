// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Lease-complete operational policy verification.
//!
//! The legacy #172 verifier correctly denies when its receipt ABI would discard a
//! shorter record/adoption proof horizon. This sibling kernel preserves that exact
//! dynamic horizon explicitly while reconstructing the same legacy policy receipt.
//! Generation-bound currentness remains #115/#116's responsibility.

use mycelix_authority_evidence_lease::{EvidenceLease, LeasedEvidence};
use mycelix_authority_operational_policy_verifier::{
    VerifiedPolicyAdoptionProof, VerifiedPolicyRecordProof, ADOPTION_PROOF_PROTOCOL,
    EVIDENCE_PROFILE, QUALIFICATION_PROFILE, RECORD_PROOF_PROTOCOL,
};
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, VerifiedAuthorityCoveragePolicy, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-operational-policy-leased-verifier-v0.1";

const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/operational-policy-verification/v1";
const DOMAIN_EVIDENCE: &[u8] =
    b"mycelix/authority/operational-policy-verification-evidence/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Non-deserializable lease-complete qualification of one coverage policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedLeasedCoveragePolicyVerification {
    receipt: VerifiedAuthorityCoveragePolicy,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    lease: EvidenceLease,
}

impl QualifiedLeasedCoveragePolicyVerification {
    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }
    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }
    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }
    pub fn lease(&self) -> &EvidenceLease {
        &self.lease
    }
    pub fn to_verified_policy(&self) -> VerifiedAuthorityCoveragePolicy {
        self.receipt.clone()
    }
    pub fn to_leased_policy(&self, now_ms: u64) -> Result<LeasedEvidence<VerifiedAuthorityCoveragePolicy>, LeasedPolicyVerificationError> {
        LeasedEvidence::new(self.receipt.clone(), self.lease.clone(), now_ms)
            .map_err(|_| LeasedPolicyVerificationError::InvalidEvidenceLease)
    }
}

/// Non-deserializable lease-complete qualification of one trust-context policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedLeasedContextPolicyVerification {
    receipt: VerifiedCoverageTrustContextPolicy,
    policy_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    lease: EvidenceLease,
}

impl QualifiedLeasedContextPolicyVerification {
    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }
    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }
    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }
    pub fn lease(&self) -> &EvidenceLease {
        &self.lease
    }
    pub fn to_verified_policy(&self) -> VerifiedCoverageTrustContextPolicy {
        self.receipt.clone()
    }
    pub fn to_leased_policy(&self, now_ms: u64) -> Result<LeasedEvidence<VerifiedCoverageTrustContextPolicy>, LeasedPolicyVerificationError> {
        LeasedEvidence::new(self.receipt.clone(), self.lease.clone(), now_ms)
            .map_err(|_| LeasedPolicyVerificationError::InvalidEvidenceLease)
    }
}

pub fn qualify_coverage_policy_leased(
    policy: &AuthorityCoveragePolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedPolicyRecordProof,
    adoption_proof: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedLeasedCoveragePolicyVerification, LeasedPolicyVerificationError> {
    policy
        .validate()
        .map_err(|_| LeasedPolicyVerificationError::InvalidCoveragePolicy)?;
    if !policy.is_active_at(now_ms) {
        return Err(LeasedPolicyVerificationError::PolicyNotSemanticallyActive);
    }
    let digest = policy
        .identity_digest()
        .map_err(|_| LeasedPolicyVerificationError::InvalidCoveragePolicy)?;
    let common = qualify_common(
        digest,
        POLICY_IDENTITY_PROFILE,
        policy_record_ref,
        &policy.authority_ref,
        &policy.policy_proof_ref,
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
        verified_at_ms: common.lease.verified_at_ms,
    };
    Ok(QualifiedLeasedCoveragePolicyVerification {
        receipt,
        policy_digest: digest,
        qualification_digest: common.qualification_digest,
        evidence_digest: common.evidence_digest,
        lease: common.lease,
    })
}

pub fn qualify_context_policy_leased(
    policy: &CoverageTrustContextPolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedPolicyRecordProof,
    adoption_proof: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<QualifiedLeasedContextPolicyVerification, LeasedPolicyVerificationError> {
    policy
        .validate()
        .map_err(|_| LeasedPolicyVerificationError::InvalidContextPolicy)?;
    if !policy.active_at(now_ms) {
        return Err(LeasedPolicyVerificationError::PolicyNotSemanticallyActive);
    }
    let digest = policy
        .identity_digest()
        .map_err(|_| LeasedPolicyVerificationError::InvalidContextPolicy)?;
    let common = qualify_common(
        digest,
        CONTEXT_POLICY_PROFILE,
        policy_record_ref,
        &policy.authority_ref,
        &policy.policy_proof_ref,
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
        verified_at_ms: common.lease.verified_at_ms,
    };
    Ok(QualifiedLeasedContextPolicyVerification {
        receipt,
        policy_digest: digest,
        qualification_digest: common.qualification_digest,
        evidence_digest: common.evidence_digest,
        lease: common.lease,
    })
}

struct CommonQualification {
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    lease: EvidenceLease,
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
    record: &VerifiedPolicyRecordProof,
    adoption: &VerifiedPolicyAdoptionProof,
    now_ms: u64,
) -> Result<CommonQualification, LeasedPolicyVerificationError> {
    if now_ms == 0 || policy_digest.is_zero() {
        return Err(LeasedPolicyVerificationError::InvalidVerificationTime);
    }
    require_profile(policy_profile)?;
    require_ref(policy_record_ref)?;
    require_ref(authority_ref)?;
    require_ref(policy_proof_ref)?;
    verify_record(record, policy_digest, policy_profile, policy_record_ref, now_ms)?;
    verify_adoption(
        adoption,
        policy_digest,
        policy_profile,
        authority_ref,
        policy_proof_ref,
        now_ms,
    )?;

    let verified_at_ms = record.verified_at_ms.max(adoption.verified_at_ms);
    if verified_at_ms < policy_valid_from_ms {
        return Err(LeasedPolicyVerificationError::PolicyProofPredatesActivation);
    }
    let proof_valid_until_ms = record.valid_until_ms.min(adoption.valid_until_ms);
    let lease = EvidenceLease::new(
        verified_at_ms,
        proof_valid_until_ms.min(policy_valid_until_ms),
        now_ms,
    )
    .map_err(|_| LeasedPolicyVerificationError::InvalidEvidenceLease)?;
    let qualification_digest = qualification_digest(policy_digest, policy_profile);
    let evidence_digest = evidence_digest(
        qualification_digest,
        policy_record_ref,
        record,
        adoption,
    );
    Ok(CommonQualification {
        qualification_digest,
        evidence_digest,
        lease,
    })
}

fn verify_record(
    proof: &VerifiedPolicyRecordProof,
    policy_digest: Digest32,
    policy_profile: &str,
    policy_record_ref: &str,
    now_ms: u64,
) -> Result<(), LeasedPolicyVerificationError> {
    if proof.protocol_version != RECORD_PROOF_PROTOCOL {
        return Err(LeasedPolicyVerificationError::WrongRecordProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != policy_profile {
        return Err(LeasedPolicyVerificationError::RecordPolicyIdentityMismatch);
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
        return Err(LeasedPolicyVerificationError::PolicyRecordRefMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
        .map_err(|_| LeasedPolicyVerificationError::InvalidRecordProofWindow)
}

fn verify_adoption(
    proof: &VerifiedPolicyAdoptionProof,
    policy_digest: Digest32,
    policy_profile: &str,
    authority_ref: &str,
    policy_proof_ref: &str,
    now_ms: u64,
) -> Result<(), LeasedPolicyVerificationError> {
    if proof.protocol_version != ADOPTION_PROOF_PROTOCOL {
        return Err(LeasedPolicyVerificationError::WrongAdoptionProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != policy_profile {
        return Err(LeasedPolicyVerificationError::AdoptionPolicyIdentityMismatch);
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
        return Err(LeasedPolicyVerificationError::PolicyAuthorityMismatch);
    }
    if proof.verified_policy_proof_ref != policy_proof_ref {
        return Err(LeasedPolicyVerificationError::PolicyAdoptionProofMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
        .map_err(|_| LeasedPolicyVerificationError::InvalidAdoptionProofWindow)
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

fn require_ref(value: &str) -> Result<(), LeasedPolicyVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(LeasedPolicyVerificationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), LeasedPolicyVerificationError> {
    if value.trim().is_empty() || value.len() > MAX_PROFILE_BYTES {
        Err(LeasedPolicyVerificationError::InvalidProfile)
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
pub enum LeasedPolicyVerificationError {
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
    PolicyProofPredatesActivation,
    InvalidEvidenceLease,
}

impl fmt::Display for LeasedPolicyVerificationError {
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
            Self::AdoptionPolicyIdentityMismatch => "adoption proof belongs to another policy identity",
            Self::PolicyRecordRefMismatch => "record proof authenticated another policy record",
            Self::PolicyAuthorityMismatch => "policy adoption proof authenticated another authority",
            Self::PolicyAdoptionProofMismatch => "policy adoption proof reference mismatch",
            Self::InvalidRecordProofWindow => "policy record proof is stale or invalid",
            Self::InvalidAdoptionProofWindow => "policy adoption proof is stale or invalid",
            Self::PolicyProofPredatesActivation => "policy proof predates semantic policy activation",
            Self::InvalidEvidenceLease => "operational policy evidence lease is invalid or expired",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for LeasedPolicyVerificationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest};
    use mycelix_authority_operational_policy_verifier::{
        qualify_coverage_policy, PolicyVerificationError,
    };
    use mycelix_authority_state_coverage::{
        CoverageMode, PROTOCOL_VERSION as COVERAGE_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn policy() -> AuthorityCoveragePolicy {
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

    fn record(digest: Digest32, valid_until_ms: u64) -> VerifiedPolicyRecordProof {
        VerifiedPolicyRecordProof {
            protocol_version: RECORD_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            policy_record_ref: "record:coverage".into(),
            record_proof_ref: "proof:record:coverage".into(),
            record_verifier_ref: "verifier:record".into(),
            verification_ref: "verify:record".into(),
            verified_at_ms: 150,
            valid_until_ms,
        }
    }

    fn adoption(digest: Digest32, valid_until_ms: u64) -> VerifiedPolicyAdoptionProof {
        VerifiedPolicyAdoptionProof {
            protocol_version: ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: digest,
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            verified_authority_ref: "authority:policy".into(),
            verified_policy_proof_ref: "proof:coverage-adoption".into(),
            authority_verifier_ref: "verifier:adoption".into(),
            verification_ref: "verify:adoption".into(),
            verified_at_ms: 160,
            valid_until_ms,
        }
    }

    #[test]
    fn representable_receipt_matches_legacy_172_projection() {
        let policy = policy();
        let digest = policy.identity_digest().unwrap();
        let record = record(digest, 900);
        let adoption = adoption(digest, 900);
        let legacy = qualify_coverage_policy(
            &policy,
            "record:coverage",
            &record,
            &adoption,
            200,
        )
        .unwrap();
        let leased = qualify_coverage_policy_leased(
            &policy,
            "record:coverage",
            &record,
            &adoption,
            200,
        )
        .unwrap();
        assert_eq!(leased.to_verified_policy(), legacy.to_verified_policy());
        assert_eq!(leased.lease().valid_until_ms, 800);
    }

    #[test]
    fn short_proof_horizon_is_preserved_where_legacy_denies() {
        let policy = policy();
        let digest = policy.identity_digest().unwrap();
        let record = record(digest, 700);
        let adoption = adoption(digest, 900);
        assert_eq!(
            qualify_coverage_policy(
                &policy,
                "record:coverage",
                &record,
                &adoption,
                200,
            )
            .unwrap_err(),
            PolicyVerificationError::LossyPolicyProjection
        );
        let leased = qualify_coverage_policy_leased(
            &policy,
            "record:coverage",
            &record,
            &adoption,
            200,
        )
        .unwrap();
        assert_eq!(leased.lease().valid_until_ms, 700);
        assert_eq!(leased.to_verified_policy().policy, policy);
    }
}
