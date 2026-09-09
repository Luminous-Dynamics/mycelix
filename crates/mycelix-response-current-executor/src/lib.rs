// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Direct current-executor requalification for one exact already-qualified
//! planetary response/effect-safety result.
//!
//! Earlier response layers consume a deserializable provider projection because
//! that is useful as a wire/discovery ABI. This crate deliberately does not
//! accept that projection. Instead it re-runs the non-deserializable current
//! executor theorem from the original threshold/grant/designation/lineage and
//! freshness evidence, then compares the exact action-relevant semantics to the
//! selected response.
//!
//! This closes projection-field substitution, but it still does not prove that
//! the evidence-shaped source receipts came from their designated live providers.
//! It grants no deployment, attempt, or external-effect authority.

use mycelix_authority_freshness::VerifiedAuthorityFreshness;
use mycelix_governance_current_executor_authority::{
    qualify_current_executor_authority, CurrentExecutorAuthorityError,
    QualifiedCurrentExecutorAuthority,
};
use mycelix_governance_executor_designation::{
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_governance_executor_lineage::DelegationLineageEvidence;
use mycelix_governance_executor_provider_contract::{
    executor_authority_ref, ExecutorProviderContractError,
};
use mycelix_institutional_core::Digest32;
use mycelix_response_effect_safety::QualifiedResponseEffectSafety;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-response-current-executor-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-response-current-executor-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-response-current-executor-evidence-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/response/current-executor/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/response/current-executor/evidence/v1";

/// Non-deserializable proof that the current executor authority was requalified
/// from source evidence in this invocation and matches the exact response action
/// semantics already closed through effect safety.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedResponseCurrentExecutor {
    proposal_id: String,
    proposal_digest: String,
    decision_id: String,
    option_id: String,
    action_class: String,
    actions_digest: Digest32,
    actions_digest_profile: String,
    jurisdiction: Option<String>,
    executor_principal: String,
    authority_grant_id: String,
    executor_authority_ref: String,
    current_executor_authority_digest: Digest32,
    current_executor_authority_profile: String,
    response_safety_qualification_digest: Digest32,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedResponseCurrentExecutor {
    pub fn proposal_id(&self) -> &str {
        &self.proposal_id
    }

    pub fn proposal_digest(&self) -> &str {
        &self.proposal_digest
    }

    pub fn decision_id(&self) -> &str {
        &self.decision_id
    }

    pub fn option_id(&self) -> &str {
        &self.option_id
    }

    pub fn action_class(&self) -> &str {
        &self.action_class
    }

    pub fn actions_digest(&self) -> Digest32 {
        self.actions_digest
    }

    pub fn actions_digest_profile(&self) -> &str {
        &self.actions_digest_profile
    }

    pub fn jurisdiction(&self) -> Option<&str> {
        self.jurisdiction.as_deref()
    }

    pub fn executor_principal(&self) -> &str {
        &self.executor_principal
    }

    pub fn authority_grant_id(&self) -> &str {
        &self.authority_grant_id
    }

    pub fn executor_authority_ref(&self) -> &str {
        &self.executor_authority_ref
    }

    pub fn current_executor_authority_digest(&self) -> Digest32 {
        self.current_executor_authority_digest
    }

    pub fn current_executor_authority_profile(&self) -> &str {
        &self.current_executor_authority_profile
    }

    pub fn response_safety_qualification_digest(&self) -> Digest32 {
        self.response_safety_qualification_digest
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

    /// The executor theorem was recomputed in this call rather than inferred
    /// from a deserializable provider projection.
    pub const fn current_executor_requalified_here(&self) -> bool {
        true
    }

    /// No `VerifiedCurrentExecutorAuthorityReceipt` is accepted by this API.
    pub const fn wire_projection_used_here(&self) -> bool {
        false
    }

    /// Source receipts remain evidence-shaped inputs; their live call provenance
    /// must be owned by a later native admission orchestrator.
    pub const fn source_origins_verified_here(&self) -> bool {
        false
    }

    pub const fn deployment_bound_here(&self) -> bool {
        false
    }

    pub const fn attempt_reserved_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Re-run the complete current executor theorem from source evidence and bind it
/// to one exact response/safety qualification.
#[allow(clippy::too_many_arguments)]
pub fn requalify_response_current_executor(
    response: &QualifiedResponseEffectSafety,
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    grant_freshness: &VerifiedAuthorityFreshness,
    threshold_freshness: &VerifiedAuthorityFreshness,
    executor_freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedResponseCurrentExecutor, ResponseCurrentExecutorError> {
    if now_ms == 0 {
        return Err(ResponseCurrentExecutorError::InvalidVerificationTime);
    }
    if response.verified_at_ms() > now_ms || response.valid_until_ms() <= now_ms {
        return Err(ResponseCurrentExecutorError::ResponseSafetyExpired);
    }
    if !response.all_declared_authority_subjects_satisfied() {
        return Err(ResponseCurrentExecutorError::PendingAuthoritySubjects);
    }

    let current = qualify_current_executor_authority(
        threshold,
        grant_receipt,
        designation_receipt,
        lineage_evidence,
        grant_freshness,
        threshold_freshness,
        executor_freshness,
        now_ms,
    )
    .map_err(ResponseCurrentExecutorError::CurrentExecutor)?;

    verify_exact_response_semantics(response, threshold, designation_receipt, &current)?;

    let current_ref = executor_authority_ref(
        current.current_authority_digest(),
        current.current_authority_profile(),
    )
    .map_err(ResponseCurrentExecutorError::AuthorityRef)?;
    if current_ref != response.executor_authority_ref() {
        return Err(ResponseCurrentExecutorError::ExecutorAuthorityRefMismatch {
            expected: response.executor_authority_ref().into(),
            actual: current_ref,
        });
    }

    let valid_until_ms = response.valid_until_ms().min(current.lease_until_ms());
    if valid_until_ms <= now_ms {
        return Err(ResponseCurrentExecutorError::NoUsableQualificationWindow);
    }
    let verified_at_ms = now_ms;

    let qualification_digest = qualification_digest(response, &current);
    let evidence_digest = evidence_digest(
        qualification_digest,
        current.freshness_digest(),
        current.verified_at_ms(),
        verified_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedResponseCurrentExecutor {
        proposal_id: response.proposal_id().into(),
        proposal_digest: response.proposal_digest().into(),
        decision_id: response.decision_id().into(),
        option_id: response.option_id().into(),
        action_class: response.action_class().into(),
        actions_digest: response.actions_digest(),
        actions_digest_profile: response.actions_digest_profile().into(),
        jurisdiction: response.jurisdiction().map(str::to_string),
        executor_principal: current.executor_principal().as_str().into(),
        authority_grant_id: current.authority_grant_id().as_str().into(),
        executor_authority_ref: response.executor_authority_ref().into(),
        current_executor_authority_digest: current.current_authority_digest(),
        current_executor_authority_profile: current.current_authority_profile().into(),
        response_safety_qualification_digest: response.qualification_digest(),
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_exact_response_semantics(
    response: &QualifiedResponseEffectSafety,
    threshold: &VerifiedThresholdAuthorization,
    designation_receipt: &VerifiedExecutorDesignation,
    current: &QualifiedCurrentExecutorAuthority,
) -> Result<(), ResponseCurrentExecutorError> {
    if threshold.authorization.proposal_id.as_str() != response.proposal_id()
        || current.proposal_id().as_str() != response.proposal_id()
    {
        return Err(ResponseCurrentExecutorError::ProposalMismatch);
    }
    if threshold.authorization.actions_digest != response.actions_digest() {
        return Err(ResponseCurrentExecutorError::ActionsDigestMismatch);
    }
    if threshold.actions_digest_profile != response.actions_digest_profile() {
        return Err(ResponseCurrentExecutorError::ActionsDigestProfileMismatch);
    }

    let designation = &designation_receipt.designation;
    if designation.proposal_id.as_str() != response.proposal_id()
        || designation.actions_digest != response.actions_digest()
        || designation.actions_digest_profile != response.actions_digest_profile()
    {
        return Err(ResponseCurrentExecutorError::DesignationActionMismatch);
    }
    if designation.required_capability.as_str() != response.action_class() {
        return Err(ResponseCurrentExecutorError::CapabilityMismatch {
            expected: response.action_class().into(),
            actual: designation.required_capability.as_str().into(),
        });
    }

    let threshold_jurisdiction = threshold
        .authorization
        .jurisdiction
        .as_ref()
        .map(|value| value.as_str());
    let designation_jurisdiction = designation.jurisdiction.as_ref().map(|value| value.as_str());
    if threshold_jurisdiction != response.jurisdiction()
        || designation_jurisdiction != response.jurisdiction()
    {
        return Err(ResponseCurrentExecutorError::JurisdictionMismatch);
    }

    if current.executor_principal() != &designation.executor
        || current.authority_grant_id() != &designation.authority_grant_id
    {
        return Err(ResponseCurrentExecutorError::ExecutorIdentityMismatch);
    }
    Ok(())
}

fn qualification_digest(
    response: &QualifiedResponseEffectSafety,
    current: &QualifiedCurrentExecutorAuthority,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &response.qualification_digest().0);
    frame(&mut hasher, response.qualification_profile().as_bytes());
    frame(&mut hasher, &current.current_authority_digest().0);
    frame(&mut hasher, current.current_authority_profile().as_bytes());
    frame(&mut hasher, current.executor_principal().as_str().as_bytes());
    frame(&mut hasher, current.authority_grant_id().as_str().as_bytes());
    frame(&mut hasher, response.proposal_id().as_bytes());
    frame(&mut hasher, response.action_class().as_bytes());
    frame(&mut hasher, &response.actions_digest().0);
    frame(&mut hasher, response.actions_digest_profile().as_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn evidence_digest(
    qualification_digest: Digest32,
    freshness_digest: Digest32,
    current_verified_at_ms: u64,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, &freshness_digest.0);
    frame(&mut hasher, &current_verified_at_ms.to_le_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Debug)]
pub enum ResponseCurrentExecutorError {
    InvalidVerificationTime,
    ResponseSafetyExpired,
    PendingAuthoritySubjects,
    CurrentExecutor(CurrentExecutorAuthorityError),
    AuthorityRef(ExecutorProviderContractError),
    ProposalMismatch,
    ActionsDigestMismatch,
    ActionsDigestProfileMismatch,
    DesignationActionMismatch,
    CapabilityMismatch { expected: String, actual: String },
    JurisdictionMismatch,
    ExecutorIdentityMismatch,
    ExecutorAuthorityRefMismatch { expected: String, actual: String },
    NoUsableQualificationWindow,
}

impl fmt::Display for ResponseCurrentExecutorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVerificationTime => write!(f, "verification time must be non-zero"),
            Self::ResponseSafetyExpired => write!(f, "response effect-safety qualification is not live"),
            Self::PendingAuthoritySubjects => {
                write!(f, "response still has unresolved declared authority subjects")
            }
            Self::CurrentExecutor(error) => write!(f, "current executor requalification failed: {error}"),
            Self::AuthorityRef(error) => write!(f, "executor authority reference failed: {error}"),
            Self::ProposalMismatch => write!(f, "current executor targets another response proposal"),
            Self::ActionsDigestMismatch => write!(f, "threshold action digest differs from response action digest"),
            Self::ActionsDigestProfileMismatch => write!(f, "threshold action digest profile differs from response profile"),
            Self::DesignationActionMismatch => write!(f, "executor designation action binding differs from response action binding"),
            Self::CapabilityMismatch { expected, actual } => write!(
                f,
                "executor capability {actual} does not match response action class {expected}"
            ),
            Self::JurisdictionMismatch => write!(f, "executor jurisdiction differs from response jurisdiction"),
            Self::ExecutorIdentityMismatch => write!(f, "current executor identity differs from the scoped designation"),
            Self::ExecutorAuthorityRefMismatch { expected, actual } => write!(
                f,
                "directly requalified executor authority {actual} differs from response-bound authority {expected}"
            ),
            Self::NoUsableQualificationWindow => write!(f, "response/current-executor evidence windows do not overlap"),
        }
    }
}

impl std::error::Error for ResponseCurrentExecutorError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stable_qualification_digest_is_time_independent() {
        // The source-level invariant is important enough to freeze mechanically:
        // dynamic verification horizons belong only to evidence identity.
        let source = include_str!("lib.rs");
        let body = source
            .split("fn qualification_digest(", 2)
            .nth(1)
            .unwrap()
            .split("fn evidence_digest(", 2)
            .next()
            .unwrap();
        for forbidden in ["verified_at_ms", "valid_until_ms", "freshness_digest"] {
            assert!(!body.contains(forbidden), "{forbidden}");
        }
        assert!(body.contains("current.current_authority_digest()"));
        assert!(body.contains("response.qualification_digest()"));
    }

    #[test]
    fn provider_wire_receipt_is_not_an_input_type() {
        let source = include_str!("lib.rs");
        let signature = source
            .split("pub fn requalify_response_current_executor(", 2)
            .nth(1)
            .unwrap()
            .split(") -> Result", 2)
            .next()
            .unwrap();
        assert!(!signature.contains("VerifiedCurrentExecutorAuthorityReceipt"));
        assert!(signature.contains("VerifiedThresholdAuthorization"));
        assert!(signature.contains("VerifiedAuthorityGrant"));
        assert!(signature.contains("VerifiedExecutorDesignation"));
        assert!(signature.contains("VerifiedAuthorityFreshness"));
    }
}
