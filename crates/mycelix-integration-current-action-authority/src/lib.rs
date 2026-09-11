//! Same-call current executor requalification for exact integration actions.
//!
//! This crate composes two independently reviewed theorems without collapsing
//! their remaining semantic boundary:
//!
//! ```text
//! QualifiedIntegrationAuthorityAction
//! + original threshold/grant/designation/lineage/freshness evidence
//!     -> re-run qualify_current_executor_authority(...)
//!     -> require exact proposal + action digest/profile equality
//!     -> QualifiedCurrentIntegrationAction
//! ```
//!
//! The result proves the exact integration command action is inside the same
//! currently-qualified executor action domain. It deliberately does **not**
//! prove that the designation's capability label semantically covers the
//! integration operation kind. That requires a separate institution-adopted
//! operation-to-capability mapping theorem.

use mycelix_authority_freshness::VerifiedAuthorityFreshness;
use mycelix_execution_action_digest::ACTIONS_DIGEST_PROFILE_V1;
use mycelix_governance_current_executor_authority::{
    qualify_current_executor_authority, CurrentExecutorAuthorityError,
    QualifiedCurrentExecutorAuthority,
};
use mycelix_governance_executor_designation::{
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_governance_executor_lineage::DelegationLineageEvidence;
use mycelix_institutional_core::Digest32;
use mycelix_integration_authority_action::QualifiedIntegrationAuthorityAction;
use mycelix_integration_core::ContentCommitment;
use thiserror::Error;

/// Non-deserializable positive composition of one exact integration action and
/// one same-call current executor qualification.
#[derive(Clone, Debug)]
pub struct QualifiedCurrentIntegrationAction {
    current_executor: QualifiedCurrentExecutorAuthority,
    command_commitment: ContentCommitment,
    actions_digest: Digest32,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedCurrentIntegrationAction {
    pub fn current_executor(&self) -> &QualifiedCurrentExecutorAuthority {
        &self.current_executor
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn actions_digest(&self) -> Digest32 {
        self.actions_digest
    }

    pub fn actions_digest_profile(&self) -> &'static str {
        ACTIONS_DIGEST_PROFILE_V1
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }

    pub const fn exact_action_authorized_here(&self) -> bool {
        true
    }

    pub const fn current_executor_requalified_here(&self) -> bool {
        true
    }

    pub const fn wire_projection_used_here(&self) -> bool {
        false
    }

    pub const fn capability_semantics_closed_here(&self) -> bool {
        false
    }

    pub const fn provider_profile_bound_here(&self) -> bool {
        false
    }

    pub const fn attempt_bound_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn requalify_current_integration_action(
    action: &QualifiedIntegrationAuthorityAction,
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    grant_freshness: &VerifiedAuthorityFreshness,
    threshold_freshness: &VerifiedAuthorityFreshness,
    executor_freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedCurrentIntegrationAction, CurrentIntegrationActionError> {
    if now_ms == 0 {
        return Err(CurrentIntegrationActionError::InvalidVerificationTime);
    }

    validate_exact_action_domain(
        action,
        threshold,
        designation_receipt,
    )?;

    // Re-run the generic non-deserializable current executor theorem from the
    // original evidence in this same call. A wire/provider projection cannot be
    // substituted for current authority at this boundary.
    let current_executor = qualify_current_executor_authority(
        threshold,
        grant_receipt,
        designation_receipt,
        lineage_evidence,
        grant_freshness,
        threshold_freshness,
        executor_freshness,
        now_ms,
    )?;

    if current_executor.proposal_id() != action.proposal_id() {
        return Err(CurrentIntegrationActionError::CurrentProposalMismatch);
    }
    if current_executor.verified_at_ms() > now_ms || current_executor.lease_until_ms() <= now_ms {
        return Err(CurrentIntegrationActionError::CurrentAuthorityExpired);
    }

    Ok(QualifiedCurrentIntegrationAction {
        verified_at_ms: current_executor.verified_at_ms(),
        lease_until_ms: current_executor.lease_until_ms(),
        current_executor,
        command_commitment: action.command_commitment().clone(),
        actions_digest: action.actions_digest(),
    })
}

fn validate_exact_action_domain(
    action: &QualifiedIntegrationAuthorityAction,
    threshold: &VerifiedThresholdAuthorization,
    designation_receipt: &VerifiedExecutorDesignation,
) -> Result<(), CurrentIntegrationActionError> {
    let designation = &designation_receipt.designation;

    if action.actions_digest_profile() != ACTIONS_DIGEST_PROFILE_V1
        || threshold.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1
        || designation.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1
    {
        return Err(CurrentIntegrationActionError::ActionDigestProfileMismatch);
    }
    if action.proposal_id() != &threshold.authorization.proposal_id
        || action.proposal_id() != &designation.proposal_id
    {
        return Err(CurrentIntegrationActionError::ProposalMismatch);
    }
    if action.actions_digest() != threshold.authorization.actions_digest
        || action.actions_digest() != designation.actions_digest
    {
        return Err(CurrentIntegrationActionError::ActionDigestMismatch);
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum CurrentIntegrationActionError {
    #[error("verification time must be non-zero")]
    InvalidVerificationTime,
    #[error("integration action does not use the registered exact-byte governance digest profile")]
    ActionDigestProfileMismatch,
    #[error("integration action proposal differs from threshold/designation authority")]
    ProposalMismatch,
    #[error("integration action digest differs from threshold/designation authority")]
    ActionDigestMismatch,
    #[error("same-call current executor qualification resolved a different proposal")]
    CurrentProposalMismatch,
    #[error("same-call current executor qualification is not live at the requested instant")]
    CurrentAuthorityExpired,
    #[error(transparent)]
    CurrentExecutor(#[from] CurrentExecutorAuthorityError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_authority::ProposalId;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn validate_domain_facts(
        action_proposal: &ProposalId,
        action_digest: Digest32,
        action_profile: &str,
        threshold_proposal: &ProposalId,
        threshold_digest: Digest32,
        threshold_profile: &str,
        designation_proposal: &ProposalId,
        designation_digest: Digest32,
        designation_profile: &str,
    ) -> Result<(), CurrentIntegrationActionError> {
        if action_profile != ACTIONS_DIGEST_PROFILE_V1
            || threshold_profile != ACTIONS_DIGEST_PROFILE_V1
            || designation_profile != ACTIONS_DIGEST_PROFILE_V1
        {
            return Err(CurrentIntegrationActionError::ActionDigestProfileMismatch);
        }
        if action_proposal != threshold_proposal || action_proposal != designation_proposal {
            return Err(CurrentIntegrationActionError::ProposalMismatch);
        }
        if action_digest != threshold_digest || action_digest != designation_digest {
            return Err(CurrentIntegrationActionError::ActionDigestMismatch);
        }
        Ok(())
    }

    #[test]
    fn exact_action_domain_requires_three_way_identity() {
        let proposal = ProposalId::new("proposal:1").unwrap();
        assert!(validate_domain_facts(
            &proposal,
            digest(7),
            ACTIONS_DIGEST_PROFILE_V1,
            &proposal,
            digest(7),
            ACTIONS_DIGEST_PROFILE_V1,
            &proposal,
            digest(7),
            ACTIONS_DIGEST_PROFILE_V1,
        )
        .is_ok());
    }

    #[test]
    fn action_digest_substitution_is_rejected() {
        let proposal = ProposalId::new("proposal:1").unwrap();
        assert!(matches!(
            validate_domain_facts(
                &proposal,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
                &proposal,
                digest(8),
                ACTIONS_DIGEST_PROFILE_V1,
                &proposal,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
            ),
            Err(CurrentIntegrationActionError::ActionDigestMismatch)
        ));
    }

    #[test]
    fn proposal_substitution_is_rejected() {
        let p1 = ProposalId::new("proposal:1").unwrap();
        let p2 = ProposalId::new("proposal:2").unwrap();
        assert!(matches!(
            validate_domain_facts(
                &p1,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
                &p2,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
                &p1,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
            ),
            Err(CurrentIntegrationActionError::ProposalMismatch)
        ));
    }

    #[test]
    fn digest_profile_substitution_is_rejected() {
        let proposal = ProposalId::new("proposal:1").unwrap();
        assert!(matches!(
            validate_domain_facts(
                &proposal,
                digest(7),
                "other-profile",
                &proposal,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
                &proposal,
                digest(7),
                ACTIONS_DIGEST_PROFILE_V1,
            ),
            Err(CurrentIntegrationActionError::ActionDigestProfileMismatch)
        ));
    }
}
