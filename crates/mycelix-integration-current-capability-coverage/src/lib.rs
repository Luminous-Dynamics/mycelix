//! Exact current executor capability coverage for current integration commands.
//!
//! This theorem closes the semantic gap left intentionally open by the exact
//! current action and current capability-policy tranches. It re-runs the generic
//! current executor theorem from original evidence, proves that rerun is the same
//! exact current executor already bound to the action, and only then compares the
//! institution-adopted operation capability with the exact designation capability.

use mycelix_authority_freshness::VerifiedAuthorityFreshness;
use mycelix_governance_current_executor_authority::{
    qualify_current_executor_authority, CurrentExecutorAuthorityError,
    QualifiedCurrentExecutorAuthority,
};
use mycelix_governance_executor_designation::{
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_governance_executor_lineage::DelegationLineageEvidence;
use mycelix_institutional_core::{
    CapabilityId, Digest32, InstitutionId, JurisdictionId, RulebookRef,
};
use mycelix_integration_capability_currentness::QualifiedCurrentIntegrationCapabilityMapping;
use mycelix_integration_core::{ContentCommitment, DigestAlgorithm};
use mycelix_integration_current_action_authority::QualifiedCurrentIntegrationAction;
use thiserror::Error;

pub const QUALIFICATION_PROFILE: &str =
    "mycelix-integration-current-capability-coverage-v1-blake3-framed";
const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/integration/current-capability-coverage/v1";

/// Non-deserializable same-call proof that one exact current integration command
/// is covered by the exact capability on the exact current executor designation.
///
/// This remains pre-provider, pre-attempt and pre-effect.
#[derive(Clone, Debug)]
pub struct MatchedCurrentIntegrationCapabilityCoverage {
    current_action: QualifiedCurrentIntegrationAction,
    current_mapping: QualifiedCurrentIntegrationCapabilityMapping,
    requalified_executor: QualifiedCurrentExecutorAuthority,
    required_capability: CapabilityId,
    qualification_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl MatchedCurrentIntegrationCapabilityCoverage {
    pub fn current_action(&self) -> &QualifiedCurrentIntegrationAction {
        &self.current_action
    }

    pub fn current_mapping(&self) -> &QualifiedCurrentIntegrationCapabilityMapping {
        &self.current_mapping
    }

    pub fn requalified_executor(&self) -> &QualifiedCurrentExecutorAuthority {
        &self.requalified_executor
    }

    pub fn required_capability(&self) -> &CapabilityId {
        &self.required_capability
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &'static str {
        QUALIFICATION_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub const fn exact_current_action_bound_here(&self) -> bool {
        true
    }

    pub const fn current_capability_policy_bound_here(&self) -> bool {
        true
    }

    pub const fn executor_capability_matched_here(&self) -> bool {
        true
    }

    pub const fn current_executor_requalified_here(&self) -> bool {
        true
    }

    pub const fn provider_profile_bound_here(&self) -> bool {
        false
    }

    pub const fn attempt_bound_here(&self) -> bool {
        false
    }

    pub const fn provider_payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

#[allow(clippy::too_many_arguments)]
pub fn match_current_integration_capability_coverage(
    current_action: &QualifiedCurrentIntegrationAction,
    current_mapping: &QualifiedCurrentIntegrationCapabilityMapping,
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    grant_freshness: &VerifiedAuthorityFreshness,
    threshold_freshness: &VerifiedAuthorityFreshness,
    executor_freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<MatchedCurrentIntegrationCapabilityCoverage, CurrentCapabilityCoverageError> {
    if now_ms == 0 {
        return Err(CurrentCapabilityCoverageError::InvalidVerificationTime);
    }
    if current_action.verified_at_ms() > now_ms || current_action.lease_until_ms() <= now_ms {
        return Err(CurrentCapabilityCoverageError::CurrentActionNotLive);
    }
    if current_mapping.verified_at_ms() > now_ms || current_mapping.valid_until_ms() <= now_ms {
        return Err(CurrentCapabilityCoverageError::CurrentMappingNotLive);
    }

    // Re-run the exact generic current-executor theorem in this invocation. The
    // caller cannot substitute a detached designation/capability label for the
    // executor that #624 actually bound to this exact action.
    let requalified_executor = qualify_current_executor_authority(
        threshold,
        grant_receipt,
        designation_receipt,
        lineage_evidence,
        grant_freshness,
        threshold_freshness,
        executor_freshness,
        now_ms,
    )?;
    if requalified_executor.verified_at_ms() > now_ms || requalified_executor.lease_until_ms() <= now_ms {
        return Err(CurrentCapabilityCoverageError::RequalifiedExecutorNotLive);
    }

    let action_executor = current_action.current_executor();
    let policy = current_mapping
        .current_policy()
        .adopted_policy()
        .policy();
    let designation = &designation_receipt.designation;

    validate_coverage_facts(
        current_action.command_commitment(),
        current_mapping.command_commitment(),
        action_executor.current_authority_digest(),
        action_executor.current_authority_profile(),
        requalified_executor.current_authority_digest(),
        requalified_executor.current_authority_profile(),
        action_executor.institution(),
        &policy.institution,
        &designation.institution,
        action_executor.rulebook(),
        &policy.rulebook,
        &designation.rulebook,
        &policy.jurisdiction,
        &designation.jurisdiction,
        current_mapping.required_capability(),
        &designation.required_capability,
    )?;

    if current_action.current_executor().proposal_id() != requalified_executor.proposal_id()
        || current_action.current_executor().executor_principal()
            != requalified_executor.executor_principal()
        || current_action.current_executor().authority_grant_id()
            != requalified_executor.authority_grant_id()
    {
        return Err(CurrentCapabilityCoverageError::CurrentExecutorIdentityMismatch);
    }

    let verified_at_ms = current_action
        .verified_at_ms()
        .max(current_mapping.verified_at_ms())
        .max(requalified_executor.verified_at_ms());
    let valid_until_ms = current_action
        .lease_until_ms()
        .min(current_mapping.valid_until_ms())
        .min(requalified_executor.lease_until_ms());
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(CurrentCapabilityCoverageError::NoUsableCoverageWindow);
    }

    let qualification_digest = qualification_digest(
        current_action.command_commitment(),
        current_action.actions_digest(),
        requalified_executor.current_authority_digest(),
        current_mapping.current_mapping_digest(),
        current_mapping.required_capability(),
    );

    Ok(MatchedCurrentIntegrationCapabilityCoverage {
        current_action: current_action.clone(),
        current_mapping: current_mapping.clone(),
        requalified_executor,
        required_capability: designation.required_capability.clone(),
        qualification_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

#[allow(clippy::too_many_arguments)]
fn validate_coverage_facts(
    action_command: &ContentCommitment,
    mapping_command: &ContentCommitment,
    action_current_authority_digest: Digest32,
    action_current_authority_profile: &str,
    rerun_current_authority_digest: Digest32,
    rerun_current_authority_profile: &str,
    action_institution: &InstitutionId,
    policy_institution: &InstitutionId,
    designation_institution: &InstitutionId,
    action_rulebook: &RulebookRef,
    policy_rulebook: &RulebookRef,
    designation_rulebook: &RulebookRef,
    policy_jurisdiction: &Option<JurisdictionId>,
    designation_jurisdiction: &Option<JurisdictionId>,
    mapped_capability: &CapabilityId,
    designation_capability: &CapabilityId,
) -> Result<(), CurrentCapabilityCoverageError> {
    if action_command != mapping_command {
        return Err(CurrentCapabilityCoverageError::CommandCommitmentMismatch);
    }
    if action_current_authority_digest != rerun_current_authority_digest
        || action_current_authority_profile != rerun_current_authority_profile
    {
        return Err(CurrentCapabilityCoverageError::CurrentExecutorIdentityMismatch);
    }
    if action_institution != policy_institution || action_institution != designation_institution {
        return Err(CurrentCapabilityCoverageError::InstitutionMismatch);
    }
    if action_rulebook != policy_rulebook || action_rulebook != designation_rulebook {
        return Err(CurrentCapabilityCoverageError::RulebookMismatch);
    }
    if policy_jurisdiction != designation_jurisdiction {
        return Err(CurrentCapabilityCoverageError::JurisdictionMismatch);
    }
    if mapped_capability != designation_capability {
        return Err(CurrentCapabilityCoverageError::CapabilityMismatch);
    }
    Ok(())
}

fn qualification_digest(
    command: &ContentCommitment,
    action_digest: Digest32,
    current_executor_digest: Digest32,
    current_mapping_digest: Digest32,
    capability: &CapabilityId,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(
        &mut h,
        &[match command.algorithm {
            DigestAlgorithm::Sha256 => 1,
        }],
    );
    frame(&mut h, &command.digest);
    frame(&mut h, &action_digest.0);
    frame(&mut h, &current_executor_digest.0);
    frame(&mut h, &current_mapping_digest.0);
    frame(&mut h, capability.as_str().as_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum CurrentCapabilityCoverageError {
    #[error("verification time must be non-zero")]
    InvalidVerificationTime,
    #[error("current integration action is not live at the requested instant")]
    CurrentActionNotLive,
    #[error("current integration capability mapping is not live at the requested instant")]
    CurrentMappingNotLive,
    #[error("same-call requalified executor is not live at the requested instant")]
    RequalifiedExecutorNotLive,
    #[error("action and capability mapping bind different canonical commands")]
    CommandCommitmentMismatch,
    #[error("same-call current executor differs from the executor bound to the action")]
    CurrentExecutorIdentityMismatch,
    #[error("capability policy/designation/action institutions differ")]
    InstitutionMismatch,
    #[error("capability policy/designation/action rulebooks differ")]
    RulebookMismatch,
    #[error("capability policy and executor designation jurisdictions differ")]
    JurisdictionMismatch,
    #[error("institution-adopted operation capability differs from executor designation capability")]
    CapabilityMismatch,
    #[error("current action/mapping/executor leaves no usable common verification window")]
    NoUsableCoverageWindow,
    #[error(transparent)]
    CurrentExecutor(#[from] CurrentExecutorAuthorityError),
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, ValidationError};
    use mycelix_integration_core::ContentCommitment;

    fn iid<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn rulebook(byte: u8) -> RulebookRef {
        RulebookRef {
            id: iid("rulebook:payments", RulebookId::new),
            version: "1".into(),
            digest: Digest32([byte; 32]),
        }
    }

    fn command(byte: u8) -> ContentCommitment {
        ContentCommitment::sha256(&[byte])
    }

    #[test]
    fn exact_current_capability_facts_match() {
        let institution = iid("institution:acme", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let rulebook = rulebook(1);
        assert!(validate_coverage_facts(
            &command(1),
            &command(1),
            Digest32([2; 32]),
            "current-executor-v1",
            Digest32([2; 32]),
            "current-executor-v1",
            &institution,
            &institution,
            &institution,
            &rulebook,
            &rulebook,
            &rulebook,
            &None,
            &None,
            &capability,
            &capability,
        )
        .is_ok());
    }

    #[test]
    fn command_substitution_fails_closed() {
        let institution = iid("institution:acme", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let rulebook = rulebook(1);
        assert_eq!(
            validate_coverage_facts(
                &command(1),
                &command(2),
                Digest32([2; 32]),
                "current-executor-v1",
                Digest32([2; 32]),
                "current-executor-v1",
                &institution,
                &institution,
                &institution,
                &rulebook,
                &rulebook,
                &rulebook,
                &None,
                &None,
                &capability,
                &capability,
            )
            .unwrap_err(),
            CurrentCapabilityCoverageError::CommandCommitmentMismatch
        );
    }

    #[test]
    fn capability_substitution_fails_closed() {
        let institution = iid("institution:acme", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let other = iid("payments.refund.execute", CapabilityId::new);
        let rulebook = rulebook(1);
        assert_eq!(
            validate_coverage_facts(
                &command(1),
                &command(1),
                Digest32([2; 32]),
                "current-executor-v1",
                Digest32([2; 32]),
                "current-executor-v1",
                &institution,
                &institution,
                &institution,
                &rulebook,
                &rulebook,
                &rulebook,
                &None,
                &None,
                &capability,
                &other,
            )
            .unwrap_err(),
            CurrentCapabilityCoverageError::CapabilityMismatch
        );
    }

    #[test]
    fn current_executor_substitution_fails_closed() {
        let institution = iid("institution:acme", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let rulebook = rulebook(1);
        assert_eq!(
            validate_coverage_facts(
                &command(1),
                &command(1),
                Digest32([2; 32]),
                "current-executor-v1",
                Digest32([3; 32]),
                "current-executor-v1",
                &institution,
                &institution,
                &institution,
                &rulebook,
                &rulebook,
                &rulebook,
                &None,
                &None,
                &capability,
                &capability,
            )
            .unwrap_err(),
            CurrentCapabilityCoverageError::CurrentExecutorIdentityMismatch
        );
    }

    #[test]
    fn cross_institution_policy_fails_closed() {
        let institution = iid("institution:acme", InstitutionId::new);
        let other = iid("institution:other", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let rulebook = rulebook(1);
        assert_eq!(
            validate_coverage_facts(
                &command(1),
                &command(1),
                Digest32([2; 32]),
                "current-executor-v1",
                Digest32([2; 32]),
                "current-executor-v1",
                &institution,
                &other,
                &institution,
                &rulebook,
                &rulebook,
                &rulebook,
                &None,
                &None,
                &capability,
                &capability,
            )
            .unwrap_err(),
            CurrentCapabilityCoverageError::InstitutionMismatch
        );
    }

    #[test]
    fn jurisdiction_substitution_fails_closed() {
        let institution = iid("institution:acme", InstitutionId::new);
        let capability = iid("payments.transfer.execute", CapabilityId::new);
        let jurisdiction = Some(iid("jurisdiction:one", JurisdictionId::new));
        let other = Some(iid("jurisdiction:two", JurisdictionId::new));
        let rulebook = rulebook(1);
        assert_eq!(
            validate_coverage_facts(
                &command(1),
                &command(1),
                Digest32([2; 32]),
                "current-executor-v1",
                Digest32([2; 32]),
                "current-executor-v1",
                &institution,
                &institution,
                &institution,
                &rulebook,
                &rulebook,
                &rulebook,
                &jurisdiction,
                &other,
                &capability,
                &capability,
            )
            .unwrap_err(),
            CurrentCapabilityCoverageError::JurisdictionMismatch
        );
    }
}
