// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Institution-bound governance signing and execution authority.
//!
//! This crate closes a critical gap between "cryptographically valid" and
//! "institutionally authorized": a threshold signature is not governance
//! authority merely because a committee key verifies it.
//!
//! A proposal must bind an institution, jurisdiction, rulebook, governing body,
//! exact action digest, and signing-policy identity. That signing policy must in
//! turn name the exact committee, epoch, key digest, threshold, action classes,
//! algorithms, and authority grant permitted to authorize execution.

use mycelix_institutional_core::{
    AuthorityGrantId, Digest32, InstitutionId, JurisdictionId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-authority-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_CODE_BYTES: usize = 256;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, GovernanceAuthorityError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(ProposalId, "proposal_id");
id_type!(GovernanceBodyId, "governance_body_id");
id_type!(CommitteeId, "committee_id");
id_type!(SigningPolicyId, "signing_policy_id");
id_type!(SignatureId, "signature_id");
id_type!(ExecutionAttemptId, "execution_attempt_id");

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SignatureAlgorithm {
    EcdsaSecp256k1,
    MlDsa65,
    HybridEcdsaMlDsa65,
}

impl SignatureAlgorithm {
    pub fn is_post_quantum_capable(&self) -> bool {
        matches!(self, Self::MlDsa65 | Self::HybridEcdsaMlDsa65)
    }
}

/// Institutional context that is part of the proposal / intent itself.
///
/// A persistence adapter must treat changes to these fields as proposal
/// amendments requiring the normal governance process, never as metadata edits.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposalAuthorityContext {
    pub protocol_version: String,
    pub proposal_id: ProposalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub governing_body: GovernanceBodyId,
    pub action_class: String,
    /// Digest of the exact executable action plan.
    pub actions_digest: Digest32,
    /// Exact institutional signing policy this proposal must use.
    pub signing_policy_id: SigningPolicyId,
    /// Digest of that policy version, preventing same-ID policy rebinding.
    pub signing_policy_digest: Digest32,
    pub created_at_ms: u64,
    pub expires_at_ms: u64,
}

impl ProposalAuthorityContext {
    pub fn validate(&self) -> Result<(), GovernanceAuthorityError> {
        require_protocol(&self.protocol_version)?;
        validate_text(self.proposal_id.as_str(), "proposal.id", MAX_ID_BYTES)?;
        validate_text(
            self.institution.as_str(),
            "proposal.institution",
            MAX_ID_BYTES,
        )?;
        if let Some(jurisdiction) = &self.jurisdiction {
            validate_text(
                jurisdiction.as_str(),
                "proposal.jurisdiction",
                MAX_ID_BYTES,
            )?;
        }
        validate_text(
            self.governing_body.as_str(),
            "proposal.governing_body",
            MAX_ID_BYTES,
        )?;
        validate_text(&self.action_class, "proposal.action_class", MAX_CODE_BYTES)?;
        self.rulebook
            .validate()
            .map_err(|_| GovernanceAuthorityError::InvalidRulebook)?;
        require_digest(self.actions_digest, "proposal.actions_digest")?;
        require_digest(
            self.signing_policy_digest,
            "proposal.signing_policy_digest",
        )?;
        if self.created_at_ms == 0 || self.expires_at_ms <= self.created_at_ms {
            return Err(GovernanceAuthorityError::InvalidTimeRange("proposal"));
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.created_at_ms <= now_ms && now_ms < self.expires_at_ms
    }
}

/// Institution-approved rule for which threshold committee may authorize an
/// action class.
///
/// `authorized_by` and `policy_proof_ref` must be resolved by the host to an
/// actual institutional governance decision / authority grant. A non-empty
/// reference alone is not cryptographic or constitutional proof.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SigningPolicy {
    pub protocol_version: String,
    pub id: SigningPolicyId,
    pub policy_digest: Digest32,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub governing_body: GovernanceBodyId,
    pub committee_id: CommitteeId,
    /// Digest of the exact committee verification key material for this epoch.
    pub committee_key_digest: Digest32,
    pub epoch: u64,
    pub min_signers: u32,
    pub member_count: u32,
    pub allowed_action_classes: Vec<String>,
    pub allowed_algorithms: Vec<SignatureAlgorithm>,
    pub pq_required: bool,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Institutional authority that created / adopted this policy.
    pub authorized_by: AuthorityGrantId,
    pub policy_proof_ref: String,
}

impl SigningPolicy {
    pub fn validate(&self) -> Result<(), GovernanceAuthorityError> {
        require_protocol(&self.protocol_version)?;
        require_digest(self.policy_digest, "signing_policy.digest")?;
        require_digest(
            self.committee_key_digest,
            "signing_policy.committee_key_digest",
        )?;
        self.rulebook
            .validate()
            .map_err(|_| GovernanceAuthorityError::InvalidRulebook)?;
        validate_text(
            self.institution.as_str(),
            "signing_policy.institution",
            MAX_ID_BYTES,
        )?;
        if let Some(jurisdiction) = &self.jurisdiction {
            validate_text(
                jurisdiction.as_str(),
                "signing_policy.jurisdiction",
                MAX_ID_BYTES,
            )?;
        }
        validate_text(
            self.governing_body.as_str(),
            "signing_policy.governing_body",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.committee_id.as_str(),
            "signing_policy.committee_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            self.authorized_by.as_str(),
            "signing_policy.authorized_by",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.policy_proof_ref,
            "signing_policy.policy_proof_ref",
            MAX_ID_BYTES,
        )?;
        if self.epoch == 0 {
            return Err(GovernanceAuthorityError::ZeroEpoch);
        }
        if self.member_count < 3 {
            return Err(GovernanceAuthorityError::TooFewCommitteeMembers);
        }
        if self.min_signers == 0 || self.min_signers > self.member_count {
            return Err(GovernanceAuthorityError::InvalidThreshold);
        }
        let majority = (self.member_count / 2) + 1;
        if self.min_signers < majority {
            return Err(GovernanceAuthorityError::ThresholdBelowMajority);
        }
        if self.allowed_action_classes.is_empty() {
            return Err(GovernanceAuthorityError::NoActionClasses);
        }
        if self
            .allowed_action_classes
            .iter()
            .any(|class| validate_text(class, "signing_policy.action_class", MAX_CODE_BYTES).is_err())
        {
            return Err(GovernanceAuthorityError::InvalidActionClass);
        }
        let unique_classes: BTreeSet<&str> =
            self.allowed_action_classes.iter().map(String::as_str).collect();
        if unique_classes.len() != self.allowed_action_classes.len() {
            return Err(GovernanceAuthorityError::DuplicateActionClass);
        }
        if self.allowed_algorithms.is_empty() {
            return Err(GovernanceAuthorityError::NoSignatureAlgorithms);
        }
        if self.pq_required
            && !self
                .allowed_algorithms
                .iter()
                .any(SignatureAlgorithm::is_post_quantum_capable)
        {
            return Err(GovernanceAuthorityError::PqPolicyWithoutPqAlgorithm);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(GovernanceAuthorityError::InvalidTimeRange("signing policy"));
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

/// Evidence emitted by the threshold-signature verifier.
///
/// This crate validates binding and policy semantics. The integrating host must
/// ensure `verification_proof_ref` corresponds to real cryptographic
/// verification against the key whose digest appears here.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ThresholdVerificationEvidence {
    pub protocol_version: String,
    pub signature_id: SignatureId,
    pub proposal_id: ProposalId,
    pub signing_policy_id: SigningPolicyId,
    pub signing_policy_digest: Digest32,
    pub committee_id: CommitteeId,
    pub committee_key_digest: Digest32,
    pub epoch: u64,
    pub actions_digest: Digest32,
    pub signer_count: u32,
    pub algorithm: SignatureAlgorithm,
    pub signed_at_ms: u64,
    pub verified_at_ms: u64,
    pub signature_ref: String,
    pub verification_proof_ref: String,
}

impl ThresholdVerificationEvidence {
    pub fn validate(&self) -> Result<(), GovernanceAuthorityError> {
        require_protocol(&self.protocol_version)?;
        require_digest(
            self.signing_policy_digest,
            "verification.signing_policy_digest",
        )?;
        require_digest(
            self.committee_key_digest,
            "verification.committee_key_digest",
        )?;
        require_digest(self.actions_digest, "verification.actions_digest")?;
        validate_text(&self.signature_ref, "verification.signature_ref", MAX_ID_BYTES)?;
        validate_text(
            &self.verification_proof_ref,
            "verification.verification_proof_ref",
            MAX_ID_BYTES,
        )?;
        if self.epoch == 0 {
            return Err(GovernanceAuthorityError::ZeroEpoch);
        }
        if self.signer_count == 0 {
            return Err(GovernanceAuthorityError::InvalidSignerCount);
        }
        if self.signed_at_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms < self.signed_at_ms
        {
            return Err(GovernanceAuthorityError::InvalidTimeRange(
                "threshold verification",
            ));
        }
        Ok(())
    }
}

/// Fresh execution request that consumes the verified governance authorization.
/// The nonce belongs in a replay cache / capability transcript in the host.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionAttempt {
    pub protocol_version: String,
    pub id: ExecutionAttemptId,
    pub proposal_id: ProposalId,
    pub signing_policy_id: SigningPolicyId,
    pub signature_id: SignatureId,
    pub exact_actions_digest: Digest32,
    pub nonce: [u8; 16],
    pub requested_at_ms: u64,
    pub expires_at_ms: u64,
}

impl ExecutionAttempt {
    pub fn validate(&self) -> Result<(), GovernanceAuthorityError> {
        require_protocol(&self.protocol_version)?;
        require_digest(
            self.exact_actions_digest,
            "execution_attempt.actions_digest",
        )?;
        if self.nonce.iter().all(|byte| *byte == 0) {
            return Err(GovernanceAuthorityError::ZeroNonce);
        }
        if self.requested_at_ms == 0 || self.expires_at_ms <= self.requested_at_ms {
            return Err(GovernanceAuthorityError::InvalidTimeRange(
                "execution attempt",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionPermit {
    pub proposal_id: ProposalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub governing_body: GovernanceBodyId,
    pub signing_policy_id: SigningPolicyId,
    pub committee_id: CommitteeId,
    pub epoch: u64,
    pub exact_actions_digest: Digest32,
    pub signature_id: SignatureId,
    pub execution_attempt_id: ExecutionAttemptId,
    pub nonce: [u8; 16],
    pub expires_at_ms: u64,
}

/// Fail-closed structural authorization evaluation.
///
/// Cryptographic signature verification and institutional proof-chain
/// verification occur outside this crate, but every identity/digest/policy field
/// they attest must match here before execution.
pub fn evaluate_governance_authority(
    context: &ProposalAuthorityContext,
    policy: &SigningPolicy,
    verification: &ThresholdVerificationEvidence,
    attempt: &ExecutionAttempt,
    now_ms: u64,
) -> Result<ExecutionPermit, GovernanceAuthorityError> {
    context.validate()?;
    policy.validate()?;
    verification.validate()?;
    attempt.validate()?;

    if !context.is_active_at(now_ms) {
        return Err(GovernanceAuthorityError::ProposalInactive);
    }
    if !policy.is_active_at(now_ms) {
        return Err(GovernanceAuthorityError::PolicyInactive);
    }
    if attempt.requested_at_ms > now_ms || now_ms >= attempt.expires_at_ms {
        return Err(GovernanceAuthorityError::ExecutionAttemptInactive);
    }
    if attempt.requested_at_ms < context.created_at_ms
        || attempt.expires_at_ms > context.expires_at_ms
        || attempt.requested_at_ms < policy.valid_from_ms
        || attempt.expires_at_ms > policy.valid_until_ms
    {
        return Err(GovernanceAuthorityError::ExecutionOutsideAuthorityLifetime);
    }

    if context.signing_policy_id != policy.id
        || context.signing_policy_digest != policy.policy_digest
    {
        return Err(GovernanceAuthorityError::SigningPolicyMismatch);
    }

    if context.institution != policy.institution {
        return Err(GovernanceAuthorityError::InstitutionMismatch);
    }
    if context.jurisdiction != policy.jurisdiction {
        return Err(GovernanceAuthorityError::JurisdictionMismatch);
    }
    if context.rulebook != policy.rulebook {
        return Err(GovernanceAuthorityError::RulebookMismatch);
    }
    if context.governing_body != policy.governing_body {
        return Err(GovernanceAuthorityError::GoverningBodyMismatch);
    }
    if !policy
        .allowed_action_classes
        .iter()
        .any(|class| class == &context.action_class)
    {
        return Err(GovernanceAuthorityError::ActionClassNotAuthorized);
    }

    if verification.proposal_id != context.proposal_id
        || attempt.proposal_id != context.proposal_id
    {
        return Err(GovernanceAuthorityError::ProposalMismatch);
    }
    if verification.signing_policy_id != policy.id
        || attempt.signing_policy_id != policy.id
        || verification.signing_policy_digest != policy.policy_digest
    {
        return Err(GovernanceAuthorityError::SigningPolicyMismatch);
    }
    if verification.committee_id != policy.committee_id {
        return Err(GovernanceAuthorityError::CommitteeMismatch);
    }
    if verification.committee_key_digest != policy.committee_key_digest {
        return Err(GovernanceAuthorityError::CommitteeKeyMismatch);
    }
    if verification.epoch != policy.epoch {
        return Err(GovernanceAuthorityError::StaleOrWrongEpoch);
    }
    if verification.signer_count < policy.min_signers
        || verification.signer_count > policy.member_count
    {
        return Err(GovernanceAuthorityError::InvalidSignerCount);
    }
    if !policy.allowed_algorithms.contains(&verification.algorithm) {
        return Err(GovernanceAuthorityError::SignatureAlgorithmNotAllowed);
    }
    if policy.pq_required && !verification.algorithm.is_post_quantum_capable() {
        return Err(GovernanceAuthorityError::PostQuantumVerificationRequired);
    }

    if verification.actions_digest != context.actions_digest
        || attempt.exact_actions_digest != context.actions_digest
    {
        return Err(GovernanceAuthorityError::ActionDigestMismatch);
    }
    if attempt.signature_id != verification.signature_id {
        return Err(GovernanceAuthorityError::SignatureMismatch);
    }

    if verification.signed_at_ms < context.created_at_ms
        || verification.signed_at_ms < policy.valid_from_ms
        || verification.signed_at_ms > now_ms
        || verification.verified_at_ms > now_ms
    {
        return Err(GovernanceAuthorityError::VerificationOutsideAuthorityLifetime);
    }

    Ok(ExecutionPermit {
        proposal_id: context.proposal_id.clone(),
        institution: context.institution.clone(),
        jurisdiction: context.jurisdiction.clone(),
        rulebook: context.rulebook.clone(),
        governing_body: context.governing_body.clone(),
        signing_policy_id: policy.id.clone(),
        committee_id: policy.committee_id.clone(),
        epoch: policy.epoch,
        exact_actions_digest: context.actions_digest,
        signature_id: verification.signature_id.clone(),
        execution_attempt_id: attempt.id.clone(),
        nonce: attempt.nonce,
        expires_at_ms: attempt.expires_at_ms,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GovernanceAuthorityError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    ZeroDigest(&'static str),
    InvalidRulebook,
    InvalidTimeRange(&'static str),
    ZeroEpoch,
    TooFewCommitteeMembers,
    InvalidThreshold,
    ThresholdBelowMajority,
    NoActionClasses,
    InvalidActionClass,
    DuplicateActionClass,
    NoSignatureAlgorithms,
    PqPolicyWithoutPqAlgorithm,
    InvalidSignerCount,
    ZeroNonce,
    ProposalInactive,
    PolicyInactive,
    ExecutionAttemptInactive,
    ExecutionOutsideAuthorityLifetime,
    SigningPolicyMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    GoverningBodyMismatch,
    ActionClassNotAuthorized,
    ProposalMismatch,
    CommitteeMismatch,
    CommitteeKeyMismatch,
    StaleOrWrongEpoch,
    SignatureAlgorithmNotAllowed,
    PostQuantumVerificationRequired,
    ActionDigestMismatch,
    SignatureMismatch,
    VerificationOutsideAuthorityLifetime,
}

impl fmt::Display for GovernanceAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be zero"),
            Self::InvalidRulebook => write!(f, "invalid rulebook reference"),
            Self::InvalidTimeRange(field) => write!(f, "invalid time range for {field}"),
            Self::ZeroEpoch => write!(f, "committee epoch must be non-zero"),
            Self::TooFewCommitteeMembers => write!(f, "committee must have at least three members"),
            Self::InvalidThreshold => write!(f, "invalid signing threshold"),
            Self::ThresholdBelowMajority => {
                write!(f, "signing threshold must be at least a committee majority")
            }
            Self::NoActionClasses => write!(f, "signing policy has no action classes"),
            Self::InvalidActionClass => write!(f, "signing policy contains an invalid action class"),
            Self::DuplicateActionClass => write!(f, "signing policy contains duplicate action classes"),
            Self::NoSignatureAlgorithms => write!(f, "signing policy has no allowed algorithms"),
            Self::PqPolicyWithoutPqAlgorithm => write!(f, "PQ-required policy allows no PQ-capable algorithm"),
            Self::InvalidSignerCount => write!(f, "threshold signature signer count is invalid"),
            Self::ZeroNonce => write!(f, "execution nonce must not be all zero"),
            Self::ProposalInactive => write!(f, "proposal authority context is inactive"),
            Self::PolicyInactive => write!(f, "signing policy is inactive"),
            Self::ExecutionAttemptInactive => write!(f, "execution attempt is inactive"),
            Self::ExecutionOutsideAuthorityLifetime => write!(f, "execution attempt exceeds proposal/policy lifetime"),
            Self::SigningPolicyMismatch => write!(f, "signing policy identity/digest mismatch"),
            Self::InstitutionMismatch => write!(f, "institution mismatch"),
            Self::JurisdictionMismatch => write!(f, "jurisdiction mismatch"),
            Self::RulebookMismatch => write!(f, "rulebook mismatch"),
            Self::GoverningBodyMismatch => write!(f, "governing body mismatch"),
            Self::ActionClassNotAuthorized => write!(f, "action class is not authorized by signing policy"),
            Self::ProposalMismatch => write!(f, "proposal identity mismatch"),
            Self::CommitteeMismatch => write!(f, "signature came from a committee not authorized by policy"),
            Self::CommitteeKeyMismatch => write!(f, "committee verification key digest mismatch"),
            Self::StaleOrWrongEpoch => write!(f, "committee epoch is stale or incorrect"),
            Self::SignatureAlgorithmNotAllowed => write!(f, "signature algorithm is not allowed by policy"),
            Self::PostQuantumVerificationRequired => write!(f, "policy requires post-quantum verification"),
            Self::ActionDigestMismatch => write!(f, "authorized action digest does not match proposal"),
            Self::SignatureMismatch => write!(f, "execution attempt references a different signature"),
            Self::VerificationOutsideAuthorityLifetime => write!(f, "signature verification is outside authority lifetime"),
        }
    }
}

impl std::error::Error for GovernanceAuthorityError {}

fn require_protocol(version: &str) -> Result<(), GovernanceAuthorityError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(GovernanceAuthorityError::WrongProtocolVersion)
    }
}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), GovernanceAuthorityError> {
    if value.trim().is_empty() {
        return Err(GovernanceAuthorityError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(GovernanceAuthorityError::TooLong(field));
    }
    Ok(())
}

fn require_digest(
    digest: Digest32,
    field: &'static str,
) -> Result<(), GovernanceAuthorityError> {
    if digest.is_zero() {
        Err(GovernanceAuthorityError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::RulebookId;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:global:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(1),
        }
    }

    fn context() -> ProposalAuthorityContext {
        ProposalAuthorityContext {
            protocol_version: PROTOCOL_VERSION.into(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            institution: InstitutionId::new("institution:mycelix").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:global").unwrap()),
            rulebook: rulebook(),
            governing_body: GovernanceBodyId::new("council:protocol").unwrap(),
            action_class: "protocol".into(),
            actions_digest: d(2),
            signing_policy_id: SigningPolicyId::new("signing-policy:protocol:v7").unwrap(),
            signing_policy_digest: d(3),
            created_at_ms: 1_000,
            expires_at_ms: 20_000,
        }
    }

    fn policy() -> SigningPolicy {
        SigningPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            id: SigningPolicyId::new("signing-policy:protocol:v7").unwrap(),
            policy_digest: d(3),
            institution: InstitutionId::new("institution:mycelix").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:global").unwrap()),
            rulebook: rulebook(),
            governing_body: GovernanceBodyId::new("council:protocol").unwrap(),
            committee_id: CommitteeId::new("committee:protocol:7").unwrap(),
            committee_key_digest: d(4),
            epoch: 7,
            min_signers: 3,
            member_count: 5,
            allowed_action_classes: vec!["protocol".into(), "parameter".into()],
            allowed_algorithms: vec![
                SignatureAlgorithm::EcdsaSecp256k1,
                SignatureAlgorithm::HybridEcdsaMlDsa65,
            ],
            pq_required: false,
            valid_from_ms: 500,
            valid_until_ms: 25_000,
            authorized_by: AuthorityGrantId::new("grant:governance:signing-policy").unwrap(),
            policy_proof_ref: "proof:policy-adoption".into(),
        }
    }

    fn verification() -> ThresholdVerificationEvidence {
        ThresholdVerificationEvidence {
            protocol_version: PROTOCOL_VERSION.into(),
            signature_id: SignatureId::new("sig:MIP-42").unwrap(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            signing_policy_id: SigningPolicyId::new("signing-policy:protocol:v7").unwrap(),
            signing_policy_digest: d(3),
            committee_id: CommitteeId::new("committee:protocol:7").unwrap(),
            committee_key_digest: d(4),
            epoch: 7,
            actions_digest: d(2),
            signer_count: 3,
            algorithm: SignatureAlgorithm::EcdsaSecp256k1,
            signed_at_ms: 5_000,
            verified_at_ms: 5_100,
            signature_ref: "signature:bytes-or-record".into(),
            verification_proof_ref: "proof:ecdsa-verification".into(),
        }
    }

    fn attempt() -> ExecutionAttempt {
        ExecutionAttempt {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ExecutionAttemptId::new("exec:MIP-42:1").unwrap(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            signing_policy_id: SigningPolicyId::new("signing-policy:protocol:v7").unwrap(),
            signature_id: SignatureId::new("sig:MIP-42").unwrap(),
            exact_actions_digest: d(2),
            nonce: [9; 16],
            requested_at_ms: 6_000,
            expires_at_ms: 7_000,
        }
    }

    #[test]
    fn exact_institutional_binding_allows_execution() {
        let permit =
            evaluate_governance_authority(&context(), &policy(), &verification(), &attempt(), 6_500)
                .unwrap();
        assert_eq!(permit.committee_id.as_str(), "committee:protocol:7");
        assert_eq!(permit.exact_actions_digest, d(2));
    }

    #[test]
    fn arbitrary_committee_is_not_authority() {
        let mut verification = verification();
        verification.committee_id = CommitteeId::new("committee:attacker").unwrap();
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::CommitteeMismatch
        );
    }

    #[test]
    fn same_policy_id_with_different_digest_is_rejected() {
        let mut verification = verification();
        verification.signing_policy_digest = d(99);
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::SigningPolicyMismatch
        );
    }

    #[test]
    fn wrong_jurisdiction_is_rejected() {
        let mut policy = policy();
        policy.jurisdiction = Some(JurisdictionId::new("jurisdiction:other").unwrap());
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy,
                &verification(),
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::JurisdictionMismatch
        );
    }

    #[test]
    fn wrong_rulebook_is_rejected() {
        let mut policy = policy();
        policy.rulebook.digest = d(88);
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy,
                &verification(),
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::RulebookMismatch
        );
    }

    #[test]
    fn stale_epoch_is_rejected() {
        let mut verification = verification();
        verification.epoch = 6;
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::StaleOrWrongEpoch
        );
    }

    #[test]
    fn wrong_key_digest_is_rejected() {
        let mut verification = verification();
        verification.committee_key_digest = d(77);
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::CommitteeKeyMismatch
        );
    }

    #[test]
    fn wrong_action_digest_is_rejected() {
        let mut attempt = attempt();
        attempt.exact_actions_digest = d(55);
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification(),
                &attempt,
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::ActionDigestMismatch
        );
    }

    #[test]
    fn insufficient_signers_are_rejected() {
        let mut verification = verification();
        verification.signer_count = 2;
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::InvalidSignerCount
        );
    }

    #[test]
    fn signer_count_above_committee_size_is_rejected() {
        let mut verification = verification();
        verification.signer_count = 6;
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification,
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::InvalidSignerCount
        );
    }

    #[test]
    fn pq_required_policy_cannot_degrade_to_classical() {
        let mut policy = policy();
        policy.pq_required = true;
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy,
                &verification(),
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::PostQuantumVerificationRequired
        );
    }

    #[test]
    fn hybrid_satisfies_pq_required_policy() {
        let mut policy = policy();
        policy.pq_required = true;
        let mut verification = verification();
        verification.algorithm = SignatureAlgorithm::HybridEcdsaMlDsa65;
        assert!(
            evaluate_governance_authority(
                &context(),
                &policy,
                &verification,
                &attempt(),
                6_500
            )
            .is_ok()
        );
    }

    #[test]
    fn zero_nonce_is_rejected() {
        let mut attempt = attempt();
        attempt.nonce = [0; 16];
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification(),
                &attempt,
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::ZeroNonce
        );
    }

    #[test]
    fn action_class_must_be_explicitly_allowed() {
        let mut context = context();
        context.action_class = "treasury".into();
        assert_eq!(
            evaluate_governance_authority(
                &context,
                &policy(),
                &verification(),
                &attempt(),
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::ActionClassNotAuthorized
        );
    }

    #[test]
    fn execution_attempt_cannot_outlive_proposal() {
        let mut attempt = attempt();
        attempt.expires_at_ms = 21_000;
        assert_eq!(
            evaluate_governance_authority(
                &context(),
                &policy(),
                &verification(),
                &attempt,
                6_500
            )
            .unwrap_err(),
            GovernanceAuthorityError::ExecutionOutsideAuthorityLifetime
        );
    }

    #[test]
    fn wire_round_trip_preserves_policy_bindings() {
        let policy = policy();
        let json = serde_json::to_string(&policy).unwrap();
        let decoded: SigningPolicy = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, policy);
    }
}
