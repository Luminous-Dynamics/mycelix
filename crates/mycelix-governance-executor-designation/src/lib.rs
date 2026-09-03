// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure scoped executor designation for Mycelix governance.
//!
//! Institutional `AuthorityGrant` already answers who holds which capabilities
//! under an institution/rulebook. This crate adds the missing attenuation needed
//! for real execution: one exact proposal, exact action digest/profile, and exact
//! qualified threshold-authorization identity.
//!
//! A broad capability grant is therefore not silently equivalent to permission
//! to execute every proposal in the institution.

use mycelix_governance_authority::ProposalId;
use mycelix_governance_threshold_qualification::{
    QualifiedThresholdAuthorization, PROTOCOL_VERSION as THRESHOLD_PROTOCOL_VERSION,
};
use mycelix_institutional_core::{
    AuthorityGrant, AuthorityGrantId, AuthoritySourceRef, CapabilityId, Digest32, InstitutionId,
    JurisdictionId, PrincipalId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-executor-designation-v0.1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Runtime wrapper around one exact already-qualified threshold authorization.
///
/// The pure threshold types currently carry digest bytes but not their canonical
/// profile; the runtime verifier must join that profile from proposal authority /
/// execution preflight and bind it here explicitly.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedThresholdAuthorization {
    pub authorization: QualifiedThresholdAuthorization,
    pub actions_digest_profile: String,
    pub threshold_authorization_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Exact attenuation of an institutional AuthorityGrant for one governance
/// execution domain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutorDesignation {
    pub protocol_version: String,
    pub designation_id: String,
    pub executor: PrincipalId,
    pub authority_grant_id: AuthorityGrantId,
    pub proposal_id: ProposalId,
    pub actions_digest: Digest32,
    pub actions_digest_profile: String,
    pub threshold_authorization_ref: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub required_capability: CapabilityId,
    pub source: AuthoritySourceRef,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    /// Attestation over this exact scoped designation, verified by the host.
    pub designation_proof_ref: String,
}

impl ExecutorDesignation {
    pub fn validate(&self) -> Result<(), ExecutorDesignationError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ExecutorDesignationError::WrongProtocolVersion);
        }
        require_ref(&self.designation_id)?;
        require_profile(&self.actions_digest_profile)?;
        require_ref(&self.threshold_authorization_ref)?;
        self.rulebook
            .validate()
            .map_err(|_| ExecutorDesignationError::InvalidRulebook)?;
        self.source
            .validate()
            .map_err(|_| ExecutorDesignationError::InvalidAuthoritySource)?;
        require_ref(&self.designation_proof_ref)?;
        if self.actions_digest.is_zero() {
            return Err(ExecutorDesignationError::ZeroActionDigest);
        }
        if self.issued_at_ms == 0 || self.expires_at_ms <= self.issued_at_ms {
            return Err(ExecutorDesignationError::InvalidDesignationLifetime);
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.issued_at_ms <= now_ms && now_ms < self.expires_at_ms
    }
}

/// Host-verification wrapper for the generic institutional grant.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityGrant {
    pub grant: AuthorityGrant,
    pub grant_record_ref: String,
    pub verified_grant_proof_ref: String,
    /// Required when `grant.delegated_from` is present.
    pub delegation_verification_ref: Option<String>,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Host-verification wrapper for the scoped designation itself.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedExecutorDesignation {
    pub designation: ExecutorDesignation,
    pub designation_record_ref: String,
    pub verified_designation_proof_ref: String,
    pub verified_source_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Exact executor authority suitable for the later lifecycle verifier adapter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct QualifiedExecutorDesignation {
    pub protocol_version: String,
    pub proposal_id: ProposalId,
    pub actions_digest: Digest32,
    pub actions_digest_profile: String,
    pub threshold_authorization_ref: String,
    pub executor_principal: PrincipalId,
    pub executor_authority_ref: String,
    pub authority_grant_id: AuthorityGrantId,
    pub granting_institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub capability_scope: CapabilityId,
    pub grant_record_ref: String,
    pub grant_verification_ref: String,
    pub designation_verification_ref: String,
    pub valid_until_ms: u64,
    pub verified_at_ms: u64,
}

pub fn qualify_executor_designation(
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    now_ms: u64,
) -> Result<QualifiedExecutorDesignation, ExecutorDesignationError> {
    validate_threshold_receipt(threshold, now_ms)?;

    let grant = &grant_receipt.grant;
    grant
        .validate()
        .map_err(|_| ExecutorDesignationError::InvalidAuthorityGrant)?;
    if !grant.is_active_at(now_ms) {
        return Err(ExecutorDesignationError::AuthorityGrantInactive);
    }
    validate_grant_receipt(grant_receipt, now_ms)?;

    let designation = &designation_receipt.designation;
    designation.validate()?;
    if !designation.is_active_at(now_ms) {
        return Err(ExecutorDesignationError::DesignationInactive);
    }
    validate_designation_receipt(designation_receipt, now_ms)?;

    let authority = &threshold.authorization;
    if designation.proposal_id != authority.proposal_id {
        return Err(ExecutorDesignationError::ProposalMismatch);
    }
    if designation.actions_digest != authority.actions_digest {
        return Err(ExecutorDesignationError::ActionDigestMismatch);
    }
    if designation.actions_digest_profile != threshold.actions_digest_profile {
        return Err(ExecutorDesignationError::ActionDigestProfileMismatch);
    }
    if designation.threshold_authorization_ref != threshold.threshold_authorization_ref {
        return Err(ExecutorDesignationError::ThresholdAuthorizationMismatch);
    }

    if designation.institution != authority.institution
        || grant.institution != authority.institution
    {
        return Err(ExecutorDesignationError::InstitutionMismatch);
    }
    if designation.jurisdiction != authority.jurisdiction
        || grant.jurisdiction != authority.jurisdiction
    {
        return Err(ExecutorDesignationError::JurisdictionMismatch);
    }
    if designation.rulebook != authority.rulebook || grant.rulebook != authority.rulebook {
        return Err(ExecutorDesignationError::RulebookMismatch);
    }

    if designation.executor != grant.holder {
        return Err(ExecutorDesignationError::ExecutorHolderMismatch);
    }
    if designation.authority_grant_id != grant.id {
        return Err(ExecutorDesignationError::GrantIdentityMismatch);
    }
    if !grant.capabilities.contains(&designation.required_capability) {
        return Err(ExecutorDesignationError::MissingExecutionCapability);
    }

    // The scoped designation may attenuate a grant but never extend its time.
    if designation.issued_at_ms < grant.issued_at_ms
        || designation.expires_at_ms > grant.expires_at_ms
    {
        return Err(ExecutorDesignationError::DesignationExceedsGrantLifetime);
    }
    // Because the designation names this exact threshold authorization, it must
    // have been issued after that authorization was positively verified.
    if designation.issued_at_ms < threshold.verified_at_ms {
        return Err(ExecutorDesignationError::DesignationPredatesThresholdAuthority);
    }
    if designation.expires_at_ms > authority.valid_until_ms {
        return Err(ExecutorDesignationError::DesignationExceedsThresholdLifetime);
    }

    let valid_until_ms = designation
        .expires_at_ms
        .min(grant.expires_at_ms)
        .min(authority.valid_until_ms);
    if now_ms >= valid_until_ms {
        return Err(ExecutorDesignationError::DesignationInactive);
    }

    Ok(QualifiedExecutorDesignation {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: authority.proposal_id.clone(),
        actions_digest: authority.actions_digest,
        actions_digest_profile: threshold.actions_digest_profile.clone(),
        threshold_authorization_ref: threshold.threshold_authorization_ref.clone(),
        executor_principal: designation.executor.clone(),
        executor_authority_ref: designation_receipt.designation_record_ref.clone(),
        authority_grant_id: grant.id.clone(),
        granting_institution: authority.institution.clone(),
        jurisdiction: authority.jurisdiction.clone(),
        rulebook: authority.rulebook.clone(),
        capability_scope: designation.required_capability.clone(),
        grant_record_ref: grant_receipt.grant_record_ref.clone(),
        grant_verification_ref: grant_receipt.verification_ref.clone(),
        designation_verification_ref: designation_receipt.verification_ref.clone(),
        valid_until_ms,
        verified_at_ms: threshold
            .verified_at_ms
            .max(grant_receipt.verified_at_ms)
            .max(designation_receipt.verified_at_ms),
    })
}

fn validate_threshold_receipt(
    threshold: &VerifiedThresholdAuthorization,
    now_ms: u64,
) -> Result<(), ExecutorDesignationError> {
    if threshold.authorization.protocol_version != THRESHOLD_PROTOCOL_VERSION {
        return Err(ExecutorDesignationError::InvalidThresholdAuthorization);
    }
    if threshold.authorization.actions_digest.is_zero() {
        return Err(ExecutorDesignationError::ZeroActionDigest);
    }
    require_profile(&threshold.actions_digest_profile)?;
    require_ref(&threshold.threshold_authorization_ref)?;
    require_ref(&threshold.verification_ref)?;
    if threshold.verified_at_ms == 0
        || threshold.verified_at_ms > now_ms
        || threshold.verified_at_ms < threshold.authorization.verified_at_ms
        || now_ms >= threshold.authorization.valid_until_ms
    {
        return Err(ExecutorDesignationError::InvalidThresholdAuthorization);
    }
    Ok(())
}

fn validate_grant_receipt(
    receipt: &VerifiedAuthorityGrant,
    now_ms: u64,
) -> Result<(), ExecutorDesignationError> {
    for value in [
        receipt.grant_record_ref.as_str(),
        receipt.verified_grant_proof_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if receipt.verified_grant_proof_ref != receipt.grant.grant_proof_ref {
        return Err(ExecutorDesignationError::GrantProofMismatch);
    }
    match (&receipt.grant.delegated_from, &receipt.delegation_verification_ref) {
        (Some(_), Some(reference)) => require_ref(reference)?,
        (Some(_), None) => return Err(ExecutorDesignationError::MissingDelegationVerification),
        (None, _) => {}
    }
    if receipt.verified_at_ms == 0 || receipt.verified_at_ms > now_ms {
        return Err(ExecutorDesignationError::InvalidVerificationTime);
    }
    Ok(())
}

fn validate_designation_receipt(
    receipt: &VerifiedExecutorDesignation,
    now_ms: u64,
) -> Result<(), ExecutorDesignationError> {
    for value in [
        receipt.designation_record_ref.as_str(),
        receipt.verified_designation_proof_ref.as_str(),
        receipt.verified_source_proof_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if receipt.verified_designation_proof_ref != receipt.designation.designation_proof_ref {
        return Err(ExecutorDesignationError::DesignationProofMismatch);
    }
    if receipt.verified_source_proof_ref != receipt.designation.source.proof_ref {
        return Err(ExecutorDesignationError::SourceProofMismatch);
    }
    if receipt.verified_at_ms == 0 || receipt.verified_at_ms > now_ms {
        return Err(ExecutorDesignationError::InvalidVerificationTime);
    }
    Ok(())
}

fn require_ref(value: &str) -> Result<(), ExecutorDesignationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ExecutorDesignationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), ExecutorDesignationError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ExecutorDesignationError::InvalidDigestProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExecutorDesignationError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidDigestProfile,
    InvalidRulebook,
    InvalidAuthoritySource,
    ZeroActionDigest,
    InvalidDesignationLifetime,
    InvalidThresholdAuthorization,
    InvalidAuthorityGrant,
    AuthorityGrantInactive,
    DesignationInactive,
    InvalidVerificationTime,
    GrantProofMismatch,
    MissingDelegationVerification,
    DesignationProofMismatch,
    SourceProofMismatch,
    ProposalMismatch,
    ActionDigestMismatch,
    ActionDigestProfileMismatch,
    ThresholdAuthorizationMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    ExecutorHolderMismatch,
    GrantIdentityMismatch,
    MissingExecutionCapability,
    DesignationExceedsGrantLifetime,
    DesignationPredatesThresholdAuthority,
    DesignationExceedsThresholdLifetime,
}

impl fmt::Display for ExecutorDesignationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong executor-designation protocol version",
            Self::InvalidReference => "invalid executor-designation reference",
            Self::InvalidDigestProfile => "invalid executable-action digest profile",
            Self::InvalidRulebook => "invalid executor-designation rulebook",
            Self::InvalidAuthoritySource => "invalid executor-designation authority source",
            Self::ZeroActionDigest => "executor designation action digest must not be zero",
            Self::InvalidDesignationLifetime => "invalid executor-designation lifetime",
            Self::InvalidThresholdAuthorization => "invalid or inactive threshold authorization",
            Self::InvalidAuthorityGrant => "invalid institutional authority grant",
            Self::AuthorityGrantInactive => "institutional authority grant is inactive",
            Self::DesignationInactive => "executor designation is inactive",
            Self::InvalidVerificationTime => "invalid executor authority verification time",
            Self::GrantProofMismatch => "verified grant proof does not match grant",
            Self::MissingDelegationVerification => "delegated grant lacks delegation verification",
            Self::DesignationProofMismatch => "verified designation proof does not match designation",
            Self::SourceProofMismatch => "verified source proof does not match designation source",
            Self::ProposalMismatch => "executor designation targets another proposal",
            Self::ActionDigestMismatch => "executor designation targets different action bytes",
            Self::ActionDigestProfileMismatch => "executor designation uses another action digest profile",
            Self::ThresholdAuthorizationMismatch => "executor designation references another threshold authorization",
            Self::InstitutionMismatch => "executor authority institution mismatch",
            Self::JurisdictionMismatch => "executor authority jurisdiction mismatch",
            Self::RulebookMismatch => "executor authority rulebook mismatch",
            Self::ExecutorHolderMismatch => "executor is not the authority-grant holder",
            Self::GrantIdentityMismatch => "executor designation references another authority grant",
            Self::MissingExecutionCapability => "authority grant lacks required execution capability",
            Self::DesignationExceedsGrantLifetime => "executor designation exceeds grant lifetime",
            Self::DesignationPredatesThresholdAuthority => "executor designation predates referenced threshold authority",
            Self::DesignationExceedsThresholdLifetime => "executor designation exceeds threshold authority lifetime",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for ExecutorDesignationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_authority::{
        CommitteeId, GovernanceBodyId, SignatureAlgorithm, SignatureId, SigningPolicyId,
    };
    use mycelix_governance_threshold_qualification::QualifiedThresholdAuthorization;
    use mycelix_institutional_core::{
        AuthoritySourceKind, RoleId, RulebookId,
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

    fn threshold() -> VerifiedThresholdAuthorization {
        VerifiedThresholdAuthorization {
            authorization: QualifiedThresholdAuthorization {
                protocol_version: THRESHOLD_PROTOCOL_VERSION.into(),
                proposal_id: ProposalId::new("MIP-42").unwrap(),
                institution: InstitutionId::new("institution:test").unwrap(),
                jurisdiction: None,
                rulebook: rulebook(),
                governing_body: GovernanceBodyId::new("body:test").unwrap(),
                action_class: "standard".into(),
                actions_digest: d(2),
                signing_policy_id: SigningPolicyId::new("policy:test").unwrap(),
                signing_policy_digest: d(3),
                committee_id: CommitteeId::new("committee:test").unwrap(),
                committee_key_digest: d(4),
                epoch: 1,
                min_signers: 2,
                member_count: 3,
                signer_count: 2,
                algorithm: SignatureAlgorithm::HybridEcdsaMlDsa65,
                signature_id: SignatureId::new("signature:1").unwrap(),
                signature_ref: "signature-record:1".into(),
                policy_record_ref: "policy-record:1".into(),
                policy_verification_ref: "policy-verification:1".into(),
                cryptographic_verification_ref: "crypto-verification:1".into(),
                signed_at_ms: 20,
                verified_at_ms: 21,
                valid_until_ms: 90,
            },
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            threshold_authorization_ref: "threshold-authorization:1".into(),
            verification_ref: "threshold-provider:verified:1".into(),
            verified_at_ms: 22,
        }
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: mycelix_institutional_core::PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:executor").unwrap(),
            holder: PrincipalId::new("did:mycelix:executor").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            roles: vec![RoleId::new("role:executor").unwrap()],
            capabilities: vec![CapabilityId::new("governance.execute").unwrap()],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "decision:executor-grant".into(),
                proof_ref: "proof:grant-source".into(),
            }],
            issued_at_ms: 5,
            expires_at_ms: 100,
            delegated_from: None,
            grant_proof_ref: "proof:grant".into(),
        }
    }

    fn grant_receipt() -> VerifiedAuthorityGrant {
        VerifiedAuthorityGrant {
            grant: grant(),
            grant_record_ref: "grant-record:1".into(),
            verified_grant_proof_ref: "proof:grant".into(),
            delegation_verification_ref: None,
            verification_ref: "grant-verification:1".into(),
            verified_at_ms: 23,
        }
    }

    fn designation() -> ExecutorDesignation {
        ExecutorDesignation {
            protocol_version: PROTOCOL_VERSION.into(),
            designation_id: "designation:MIP-42:executor".into(),
            executor: PrincipalId::new("did:mycelix:executor").unwrap(),
            authority_grant_id: AuthorityGrantId::new("grant:executor").unwrap(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            actions_digest: d(2),
            actions_digest_profile: "mycelix-governance-execution-authority-v1-blake3-exact-json".into(),
            threshold_authorization_ref: "threshold-authorization:1".into(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            rulebook: rulebook(),
            required_capability: CapabilityId::new("governance.execute").unwrap(),
            source: AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "decision:executor-designation".into(),
                proof_ref: "proof:designation-source".into(),
            },
            issued_at_ms: 24,
            expires_at_ms: 80,
            designation_proof_ref: "proof:designation".into(),
        }
    }

    fn designation_receipt() -> VerifiedExecutorDesignation {
        VerifiedExecutorDesignation {
            designation: designation(),
            designation_record_ref: "designation-record:1".into(),
            verified_designation_proof_ref: "proof:designation".into(),
            verified_source_proof_ref: "proof:designation-source".into(),
            verification_ref: "designation-verification:1".into(),
            verified_at_ms: 25,
        }
    }

    #[test]
    fn exact_scoped_designation_qualifies() {
        let qualified = qualify_executor_designation(
            &threshold(),
            &grant_receipt(),
            &designation_receipt(),
            30,
        )
        .unwrap();
        assert_eq!(qualified.executor_principal.as_str(), "did:mycelix:executor");
        assert_eq!(qualified.capability_scope.as_str(), "governance.execute");
        assert_eq!(qualified.valid_until_ms, 80);
    }

    #[test]
    fn broad_grant_cannot_cover_another_proposal_without_designation() {
        let mut receipt = designation_receipt();
        receipt.designation.proposal_id = ProposalId::new("MIP-99").unwrap();
        assert_eq!(
            qualify_executor_designation(&threshold(), &grant_receipt(), &receipt, 30)
                .unwrap_err(),
            ExecutorDesignationError::ProposalMismatch
        );
    }

    #[test]
    fn wrong_action_profile_is_denied() {
        let mut receipt = designation_receipt();
        receipt.designation.actions_digest_profile = "other-profile".into();
        assert_eq!(
            qualify_executor_designation(&threshold(), &grant_receipt(), &receipt, 30)
                .unwrap_err(),
            ExecutorDesignationError::ActionDigestProfileMismatch
        );
    }

    #[test]
    fn missing_execute_capability_is_denied() {
        let mut grant = grant_receipt();
        grant.grant.capabilities.clear();
        assert!(matches!(
            qualify_executor_designation(&threshold(), &grant, &designation_receipt(), 30),
            Err(ExecutorDesignationError::InvalidAuthorityGrant)
                | Err(ExecutorDesignationError::MissingExecutionCapability)
        ));
    }

    #[test]
    fn different_holder_cannot_execute() {
        let mut grant = grant_receipt();
        grant.grant.holder = PrincipalId::new("did:mycelix:other").unwrap();
        assert_eq!(
            qualify_executor_designation(&threshold(), &grant, &designation_receipt(), 30)
                .unwrap_err(),
            ExecutorDesignationError::ExecutorHolderMismatch
        );
    }

    #[test]
    fn designation_cannot_outlive_threshold_authority() {
        let mut receipt = designation_receipt();
        receipt.designation.expires_at_ms = 95;
        assert_eq!(
            qualify_executor_designation(&threshold(), &grant_receipt(), &receipt, 30)
                .unwrap_err(),
            ExecutorDesignationError::DesignationExceedsThresholdLifetime
        );
    }

    #[test]
    fn delegated_grant_requires_delegation_verification() {
        let mut grant = grant_receipt();
        grant.grant.delegated_from = Some(AuthorityGrantId::new("grant:parent").unwrap());
        assert_eq!(
            qualify_executor_designation(&threshold(), &grant, &designation_receipt(), 30)
                .unwrap_err(),
            ExecutorDesignationError::MissingDelegationVerification
        );
    }
}
