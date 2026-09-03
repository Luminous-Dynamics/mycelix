// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure threshold-authority qualification for Mycelix governance.
//!
//! This crate reuses the normative types from `mycelix-governance-authority` and
//! separates one earlier question from execution-attempt consumption:
//!
//! > Does this institution-approved signing policy and cryptographically verified
//! > threshold signature authorize this exact proposal/action digest **now**?
//!
//! It deliberately does not create an `ExecutionAttempt`, nonce, executor grant,
//! lifecycle claim, or external-effect permission.

use mycelix_governance_authority::{
    CommitteeId, GovernanceBodyId, ProposalAuthorityContext, ProposalId, SignatureAlgorithm,
    SignatureId, SigningPolicy, SigningPolicyId, ThresholdVerificationEvidence,
};
use mycelix_institutional_core::{Digest32, InstitutionId, JurisdictionId, RulebookRef};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-threshold-qualification-v0.1";
const MAX_REF_BYTES: usize = 2048;

/// Host-attested institutional verification of one exact signing policy.
///
/// The host must independently verify the policy adoption/proof chain. This pure
/// crate only requires the receipt to bind the exact digest/proof reference carried
/// by the policy instead of accepting a caller-provided `trusted=true` flag.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedSigningPolicy {
    pub policy: SigningPolicy,
    pub verified_policy_digest: Digest32,
    pub verified_policy_proof_ref: String,
    pub policy_record_ref: String,
    pub institutional_verification_ref: String,
    pub verified_at_ms: u64,
}

/// Host-attested cryptographic verification of one exact threshold signature.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedThresholdEvidence {
    pub evidence: ThresholdVerificationEvidence,
    pub verified_signature_ref: String,
    pub verified_crypto_proof_ref: String,
    pub verified_committee_key_digest: Digest32,
    pub cryptographic_verification_ref: String,
    pub verified_at_ms: u64,
}

/// Threshold authority qualified for later composition by a lifecycle/execution
/// provider. This is not an execution attempt and contains no nonce.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedThresholdAuthorization {
    pub protocol_version: String,
    pub proposal_id: ProposalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub governing_body: GovernanceBodyId,
    pub action_class: String,
    pub actions_digest: Digest32,
    pub signing_policy_id: SigningPolicyId,
    pub signing_policy_digest: Digest32,
    pub committee_id: CommitteeId,
    pub committee_key_digest: Digest32,
    pub epoch: u64,
    pub min_signers: u32,
    pub member_count: u32,
    pub signer_count: u32,
    pub algorithm: SignatureAlgorithm,
    pub signature_id: SignatureId,
    pub signature_ref: String,
    pub policy_record_ref: String,
    pub policy_verification_ref: String,
    pub cryptographic_verification_ref: String,
    pub signed_at_ms: u64,
    pub verified_at_ms: u64,
    /// Latest instant at which this threshold authorization may be consumed by a
    /// later execution-attempt/claim layer, absent an earlier revocation.
    pub valid_until_ms: u64,
}

pub fn qualify_threshold_authorization(
    context: &ProposalAuthorityContext,
    policy_receipt: &VerifiedSigningPolicy,
    signature_receipt: &VerifiedThresholdEvidence,
    now_ms: u64,
) -> Result<QualifiedThresholdAuthorization, ThresholdQualificationError> {
    context
        .validate()
        .map_err(|_| ThresholdQualificationError::InvalidProposalAuthority)?;
    policy_receipt
        .policy
        .validate()
        .map_err(|_| ThresholdQualificationError::InvalidSigningPolicy)?;
    signature_receipt
        .evidence
        .validate()
        .map_err(|_| ThresholdQualificationError::InvalidThresholdEvidence)?;

    validate_receipt_refs(policy_receipt, signature_receipt)?;
    validate_receipt_times(policy_receipt, signature_receipt, now_ms)?;

    let policy = &policy_receipt.policy;
    let evidence = &signature_receipt.evidence;

    if !context.is_active_at(now_ms) {
        return Err(ThresholdQualificationError::ProposalAuthorityInactive);
    }
    if !policy.is_active_at(now_ms) {
        return Err(ThresholdQualificationError::SigningPolicyInactive);
    }

    if context.signing_policy_id != policy.id
        || context.signing_policy_digest != policy.policy_digest
        || policy_receipt.verified_policy_digest != policy.policy_digest
    {
        return Err(ThresholdQualificationError::SigningPolicyMismatch);
    }
    if policy_receipt.verified_policy_proof_ref != policy.policy_proof_ref {
        return Err(ThresholdQualificationError::PolicyProofMismatch);
    }

    if context.institution != policy.institution {
        return Err(ThresholdQualificationError::InstitutionMismatch);
    }
    if context.jurisdiction != policy.jurisdiction {
        return Err(ThresholdQualificationError::JurisdictionMismatch);
    }
    if context.rulebook != policy.rulebook {
        return Err(ThresholdQualificationError::RulebookMismatch);
    }
    if context.governing_body != policy.governing_body {
        return Err(ThresholdQualificationError::GoverningBodyMismatch);
    }
    if !policy
        .allowed_action_classes
        .iter()
        .any(|class| class == &context.action_class)
    {
        return Err(ThresholdQualificationError::ActionClassNotAuthorized);
    }

    if evidence.proposal_id != context.proposal_id {
        return Err(ThresholdQualificationError::ProposalMismatch);
    }
    if evidence.signing_policy_id != policy.id
        || evidence.signing_policy_digest != policy.policy_digest
    {
        return Err(ThresholdQualificationError::SigningPolicyMismatch);
    }
    if evidence.committee_id != policy.committee_id {
        return Err(ThresholdQualificationError::CommitteeMismatch);
    }
    if evidence.committee_key_digest != policy.committee_key_digest
        || signature_receipt.verified_committee_key_digest != policy.committee_key_digest
    {
        return Err(ThresholdQualificationError::CommitteeKeyMismatch);
    }
    if evidence.epoch != policy.epoch {
        return Err(ThresholdQualificationError::StaleOrWrongEpoch);
    }
    if evidence.signer_count < policy.min_signers || evidence.signer_count > policy.member_count {
        return Err(ThresholdQualificationError::InvalidSignerCount);
    }
    if !policy.allowed_algorithms.contains(&evidence.algorithm) {
        return Err(ThresholdQualificationError::SignatureAlgorithmNotAllowed);
    }
    if policy.pq_required && !evidence.algorithm.is_post_quantum_capable() {
        return Err(ThresholdQualificationError::PostQuantumVerificationRequired);
    }
    if evidence.actions_digest != context.actions_digest {
        return Err(ThresholdQualificationError::ActionDigestMismatch);
    }

    if signature_receipt.verified_signature_ref != evidence.signature_ref {
        return Err(ThresholdQualificationError::SignatureRefMismatch);
    }
    if signature_receipt.verified_crypto_proof_ref != evidence.verification_proof_ref {
        return Err(ThresholdQualificationError::CryptoProofMismatch);
    }

    if evidence.signed_at_ms < context.created_at_ms
        || evidence.signed_at_ms < policy.valid_from_ms
        || evidence.signed_at_ms > now_ms
        || evidence.verified_at_ms > now_ms
    {
        return Err(ThresholdQualificationError::VerificationOutsideAuthorityLifetime);
    }

    let valid_until_ms = context.expires_at_ms.min(policy.valid_until_ms);
    if now_ms >= valid_until_ms {
        return Err(ThresholdQualificationError::AuthorizationExpired);
    }

    Ok(QualifiedThresholdAuthorization {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: context.proposal_id.clone(),
        institution: context.institution.clone(),
        jurisdiction: context.jurisdiction.clone(),
        rulebook: context.rulebook.clone(),
        governing_body: context.governing_body.clone(),
        action_class: context.action_class.clone(),
        actions_digest: context.actions_digest,
        signing_policy_id: policy.id.clone(),
        signing_policy_digest: policy.policy_digest,
        committee_id: policy.committee_id.clone(),
        committee_key_digest: policy.committee_key_digest,
        epoch: policy.epoch,
        min_signers: policy.min_signers,
        member_count: policy.member_count,
        signer_count: evidence.signer_count,
        algorithm: evidence.algorithm.clone(),
        signature_id: evidence.signature_id.clone(),
        signature_ref: evidence.signature_ref.clone(),
        policy_record_ref: policy_receipt.policy_record_ref.clone(),
        policy_verification_ref: policy_receipt.institutional_verification_ref.clone(),
        cryptographic_verification_ref: signature_receipt.cryptographic_verification_ref.clone(),
        signed_at_ms: evidence.signed_at_ms,
        verified_at_ms: signature_receipt.verified_at_ms.max(evidence.verified_at_ms),
        valid_until_ms,
    })
}

fn validate_receipt_refs(
    policy: &VerifiedSigningPolicy,
    signature: &VerifiedThresholdEvidence,
) -> Result<(), ThresholdQualificationError> {
    for value in [
        policy.verified_policy_proof_ref.as_str(),
        policy.policy_record_ref.as_str(),
        policy.institutional_verification_ref.as_str(),
        signature.verified_signature_ref.as_str(),
        signature.verified_crypto_proof_ref.as_str(),
        signature.cryptographic_verification_ref.as_str(),
    ] {
        if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
            return Err(ThresholdQualificationError::InvalidVerificationReference);
        }
    }
    if policy.verified_policy_digest.is_zero() || signature.verified_committee_key_digest.is_zero() {
        return Err(ThresholdQualificationError::ZeroVerifiedDigest);
    }
    Ok(())
}

fn validate_receipt_times(
    policy: &VerifiedSigningPolicy,
    signature: &VerifiedThresholdEvidence,
    now_ms: u64,
) -> Result<(), ThresholdQualificationError> {
    if now_ms == 0
        || policy.verified_at_ms == 0
        || policy.verified_at_ms > now_ms
        || signature.verified_at_ms == 0
        || signature.verified_at_ms > now_ms
        || signature.verified_at_ms < signature.evidence.verified_at_ms
    {
        return Err(ThresholdQualificationError::InvalidVerificationTime);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ThresholdQualificationError {
    InvalidProposalAuthority,
    InvalidSigningPolicy,
    InvalidThresholdEvidence,
    InvalidVerificationReference,
    ZeroVerifiedDigest,
    InvalidVerificationTime,
    ProposalAuthorityInactive,
    SigningPolicyInactive,
    SigningPolicyMismatch,
    PolicyProofMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    GoverningBodyMismatch,
    ActionClassNotAuthorized,
    ProposalMismatch,
    CommitteeMismatch,
    CommitteeKeyMismatch,
    StaleOrWrongEpoch,
    InvalidSignerCount,
    SignatureAlgorithmNotAllowed,
    PostQuantumVerificationRequired,
    ActionDigestMismatch,
    SignatureRefMismatch,
    CryptoProofMismatch,
    VerificationOutsideAuthorityLifetime,
    AuthorizationExpired,
}

impl fmt::Display for ThresholdQualificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidProposalAuthority => "invalid proposal authority context",
            Self::InvalidSigningPolicy => "invalid signing policy",
            Self::InvalidThresholdEvidence => "invalid threshold verification evidence",
            Self::InvalidVerificationReference => "invalid verification reference",
            Self::ZeroVerifiedDigest => "verified digest must not be zero",
            Self::InvalidVerificationTime => "invalid verification timestamp",
            Self::ProposalAuthorityInactive => "proposal authority context is inactive",
            Self::SigningPolicyInactive => "signing policy is inactive",
            Self::SigningPolicyMismatch => "signing policy identity/digest mismatch",
            Self::PolicyProofMismatch => "verified policy proof does not match policy",
            Self::InstitutionMismatch => "institution mismatch",
            Self::JurisdictionMismatch => "jurisdiction mismatch",
            Self::RulebookMismatch => "rulebook mismatch",
            Self::GoverningBodyMismatch => "governing body mismatch",
            Self::ActionClassNotAuthorized => "action class is not authorized",
            Self::ProposalMismatch => "proposal mismatch",
            Self::CommitteeMismatch => "committee mismatch",
            Self::CommitteeKeyMismatch => "committee verification-key mismatch",
            Self::StaleOrWrongEpoch => "committee epoch is stale or incorrect",
            Self::InvalidSignerCount => "threshold signer count is invalid",
            Self::SignatureAlgorithmNotAllowed => "signature algorithm is not allowed",
            Self::PostQuantumVerificationRequired => "post-quantum verification is required",
            Self::ActionDigestMismatch => "authorized action digest mismatch",
            Self::SignatureRefMismatch => "verified signature reference mismatch",
            Self::CryptoProofMismatch => "cryptographic proof reference mismatch",
            Self::VerificationOutsideAuthorityLifetime => {
                "signature verification lies outside authority lifetime"
            }
            Self::AuthorizationExpired => "qualified threshold authorization is expired",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for ThresholdQualificationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_authority::{
        CommitteeId, GovernanceBodyId, ProposalId, SignatureId, SigningPolicyId,
        PROTOCOL_VERSION as AUTHORITY_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::{
        AuthorityGrantId, InstitutionId, RulebookId, RulebookRef,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:test").unwrap(),
            version: "1".into(),
            digest: d(1),
        }
    }

    fn context() -> ProposalAuthorityContext {
        ProposalAuthorityContext {
            protocol_version: AUTHORITY_PROTOCOL_VERSION.into(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            rulebook: rulebook(),
            governing_body: GovernanceBodyId::new("body:test").unwrap(),
            action_class: "standard".into(),
            actions_digest: d(2),
            signing_policy_id: SigningPolicyId::new("policy:test").unwrap(),
            signing_policy_digest: d(3),
            created_at_ms: 10,
            expires_at_ms: 100,
        }
    }

    fn policy() -> SigningPolicy {
        SigningPolicy {
            protocol_version: AUTHORITY_PROTOCOL_VERSION.into(),
            id: SigningPolicyId::new("policy:test").unwrap(),
            policy_digest: d(3),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            rulebook: rulebook(),
            governing_body: GovernanceBodyId::new("body:test").unwrap(),
            committee_id: CommitteeId::new("committee:test").unwrap(),
            committee_key_digest: d(4),
            epoch: 1,
            min_signers: 2,
            member_count: 3,
            allowed_action_classes: vec!["standard".into()],
            allowed_algorithms: vec![SignatureAlgorithm::HybridEcdsaMlDsa65],
            pq_required: true,
            valid_from_ms: 10,
            valid_until_ms: 90,
            authorized_by: AuthorityGrantId::new("grant:policy").unwrap(),
            policy_proof_ref: "proof:policy".into(),
        }
    }

    fn evidence() -> ThresholdVerificationEvidence {
        ThresholdVerificationEvidence {
            protocol_version: AUTHORITY_PROTOCOL_VERSION.into(),
            signature_id: SignatureId::new("signature:1").unwrap(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            signing_policy_id: SigningPolicyId::new("policy:test").unwrap(),
            signing_policy_digest: d(3),
            committee_id: CommitteeId::new("committee:test").unwrap(),
            committee_key_digest: d(4),
            epoch: 1,
            actions_digest: d(2),
            signer_count: 2,
            algorithm: SignatureAlgorithm::HybridEcdsaMlDsa65,
            signed_at_ms: 20,
            verified_at_ms: 21,
            signature_ref: "signature-record:1".into(),
            verification_proof_ref: "proof:crypto:1".into(),
        }
    }

    fn policy_receipt() -> VerifiedSigningPolicy {
        VerifiedSigningPolicy {
            policy: policy(),
            verified_policy_digest: d(3),
            verified_policy_proof_ref: "proof:policy".into(),
            policy_record_ref: "policy-record:1".into(),
            institutional_verification_ref: "institution-proof:1".into(),
            verified_at_ms: 22,
        }
    }

    fn evidence_receipt() -> VerifiedThresholdEvidence {
        VerifiedThresholdEvidence {
            evidence: evidence(),
            verified_signature_ref: "signature-record:1".into(),
            verified_crypto_proof_ref: "proof:crypto:1".into(),
            verified_committee_key_digest: d(4),
            cryptographic_verification_ref: "crypto-verification:1".into(),
            verified_at_ms: 23,
        }
    }

    #[test]
    fn qualifies_exact_institution_policy_committee_and_signature() {
        let qualified = qualify_threshold_authorization(
            &context(),
            &policy_receipt(),
            &evidence_receipt(),
            30,
        )
        .unwrap();
        assert_eq!(qualified.committee_id.as_str(), "committee:test");
        assert_eq!(qualified.epoch, 1);
        assert_eq!(qualified.signer_count, 2);
        assert_eq!(qualified.valid_until_ms, 90);
    }

    #[test]
    fn wrong_committee_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.evidence.committee_id = CommitteeId::new("committee:other").unwrap();
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30)
                .unwrap_err(),
            ThresholdQualificationError::CommitteeMismatch
        );
    }

    #[test]
    fn stale_epoch_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.evidence.epoch = 2;
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30)
                .unwrap_err(),
            ThresholdQualificationError::StaleOrWrongEpoch
        );
    }

    #[test]
    fn below_threshold_signer_count_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.evidence.signer_count = 1;
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30)
                .unwrap_err(),
            ThresholdQualificationError::InvalidSignerCount
        );
    }

    #[test]
    fn pq_downgrade_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.evidence.algorithm = SignatureAlgorithm::EcdsaSecp256k1;
        assert!(matches!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30),
            Err(ThresholdQualificationError::SignatureAlgorithmNotAllowed)
                | Err(ThresholdQualificationError::PostQuantumVerificationRequired)
        ));
    }

    #[test]
    fn action_digest_mismatch_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.evidence.actions_digest = d(99);
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30)
                .unwrap_err(),
            ThresholdQualificationError::ActionDigestMismatch
        );
    }

    #[test]
    fn fake_crypto_proof_binding_is_denied() {
        let mut receipt = evidence_receipt();
        receipt.verified_crypto_proof_ref = "proof:other".into();
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &receipt, 30)
                .unwrap_err(),
            ThresholdQualificationError::CryptoProofMismatch
        );
    }

    #[test]
    fn expired_policy_is_denied() {
        assert_eq!(
            qualify_threshold_authorization(&context(), &policy_receipt(), &evidence_receipt(), 95)
                .unwrap_err(),
            ThresholdQualificationError::SigningPolicyInactive
        );
    }
}
