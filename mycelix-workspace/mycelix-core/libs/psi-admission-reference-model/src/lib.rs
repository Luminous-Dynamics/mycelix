// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! PSI-002C0: deterministic reference transition model for PSI admission.
//!
//! This crate is intentionally pure. It proves no durable/runtime property.

use psi_abuse_control_profiles::{ProfileError as AbuseProfileError, PsiAbuseControlProfile};
use std::collections::BTreeSet;

pub const PSI_002B_SUBJECT: &str = "10ea945145066a6f79075a888840dd8266b53be1";
pub const MODEL_ID: &str = "psi-admission-reference-model-v1";
pub const MAX_CAPABILITY_ID_BYTES: usize = 128;

pub type RequestCommitment = [u8; 32];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolicySnapshot {
    /// The complete validated PSI-002B profile is retained so any structural,
    /// leakage, domain, budget, or authority-policy drift fails closed.
    pub profile: PsiAbuseControlProfile,
}

impl PolicySnapshot {
    pub fn from_profile(profile: &PsiAbuseControlProfile) -> Result<Self, ModelError> {
        profile.validate().map_err(ModelError::ProfileInvalid)?;
        Ok(Self {
            profile: profile.clone(),
        })
    }

    pub fn budget(&self) -> &psi_abuse_control_profiles::QueryBudget {
        &self.profile.budget
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdmissionState {
    pub policy: PolicySnapshot,
    pub capability_identity: String,
    pub capability_valid: bool,
    pub revoked: bool,
    pub epoch_current: bool,
    pub epoch_closed: bool,
    pub requests_used: u32,
    pub elements_used: u64,
    pub consumed_request_commitments: BTreeSet<RequestCommitment>,
    pub active_reservations: BTreeSet<RequestCommitment>,
}

impl AdmissionState {
    pub fn fresh(
        profile: &PsiAbuseControlProfile,
        capability_identity: impl Into<String>,
    ) -> Result<Self, ModelError> {
        let capability_identity = capability_identity.into();
        if !valid_capability_identity(&capability_identity) {
            return Err(ModelError::InvalidCapabilityIdentity);
        }
        Ok(Self {
            policy: PolicySnapshot::from_profile(profile)?,
            capability_identity,
            capability_valid: true,
            revoked: false,
            epoch_current: true,
            epoch_closed: false,
            requests_used: 0,
            elements_used: 0,
            consumed_request_commitments: BTreeSet::new(),
            active_reservations: BTreeSet::new(),
        })
    }

    pub fn concurrent_reserved(&self) -> usize {
        self.active_reservations.len()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdmissionRequest {
    pub request_commitment: RequestCommitment,
    pub blinded_element_count: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdmissionDecision {
    Admitted,
    CapabilityInvalid,
    CapabilityRevoked,
    EpochNotCurrent,
    EpochClosed,
    Replay,
    RequestBudgetExceeded,
    PerRequestElementBudgetExceeded,
    EpochElementBudgetExceeded,
    ConcurrencyBudgetExceeded,
    MalformedRequestCommitment,
    MalformedPolicyState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Transition {
    pub decision: AdmissionDecision,
    pub state: AdmissionState,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ModelError {
    ProfileInvalid(AbuseProfileError),
    InvalidCapabilityIdentity,
}

fn valid_capability_identity(value: &str) -> bool {
    let bytes = value.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= MAX_CAPABILITY_ID_BYTES
        && bytes.iter().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(*byte, b'-' | b'_' | b'.' | b':' | b'/')
        })
}

fn policy_matches(
    profile: &PsiAbuseControlProfile,
    state: &AdmissionState,
) -> Result<bool, ModelError> {
    Ok(PolicySnapshot::from_profile(profile)? == state.policy)
}

fn state_is_well_formed(state: &AdmissionState) -> bool {
    let budget = state.policy.budget();
    state.requests_used <= budget.max_requests_per_epoch
        && state.elements_used <= budget.max_blinded_elements_per_epoch
        && state.active_reservations.len() <= usize::from(budget.max_concurrent_requests)
        && state
            .active_reservations
            .iter()
            .all(|commitment| state.consumed_request_commitments.contains(commitment))
        && !(state.epoch_closed && state.epoch_current)
        && valid_capability_identity(&state.capability_identity)
}

fn rejected(state: &AdmissionState, decision: AdmissionDecision) -> Transition {
    Transition {
        decision,
        state: state.clone(),
    }
}

pub fn admit(
    profile: &PsiAbuseControlProfile,
    state: &AdmissionState,
    request: AdmissionRequest,
) -> Result<Transition, ModelError> {
    if !policy_matches(profile, state)? || !state_is_well_formed(state) {
        return Ok(rejected(state, AdmissionDecision::MalformedPolicyState));
    }
    if !state.capability_valid {
        return Ok(rejected(state, AdmissionDecision::CapabilityInvalid));
    }
    if state.revoked {
        return Ok(rejected(state, AdmissionDecision::CapabilityRevoked));
    }
    // Closed is stronger/more specific than merely not-current and therefore
    // has deterministic precedence in the reference decision vocabulary.
    if state.epoch_closed {
        return Ok(rejected(state, AdmissionDecision::EpochClosed));
    }
    if !state.epoch_current {
        return Ok(rejected(state, AdmissionDecision::EpochNotCurrent));
    }
    if request.request_commitment == [0; 32] {
        return Ok(rejected(state, AdmissionDecision::MalformedRequestCommitment));
    }
    if state
        .consumed_request_commitments
        .contains(&request.request_commitment)
    {
        return Ok(rejected(state, AdmissionDecision::Replay));
    }
    let budget = state.policy.budget();
    if request.blinded_element_count == 0
        || request.blinded_element_count > budget.max_blinded_elements_per_request
    {
        return Ok(rejected(
            state,
            AdmissionDecision::PerRequestElementBudgetExceeded,
        ));
    }
    if state.requests_used >= budget.max_requests_per_epoch {
        return Ok(rejected(state, AdmissionDecision::RequestBudgetExceeded));
    }
    let next_elements = state
        .elements_used
        .checked_add(u64::from(request.blinded_element_count));
    if next_elements.is_none() || next_elements.unwrap() > budget.max_blinded_elements_per_epoch {
        return Ok(rejected(state, AdmissionDecision::EpochElementBudgetExceeded));
    }
    if state.active_reservations.len() >= usize::from(budget.max_concurrent_requests) {
        return Ok(rejected(state, AdmissionDecision::ConcurrencyBudgetExceeded));
    }

    let mut next = state.clone();
    next.requests_used += 1;
    next.elements_used = next_elements.expect("checked above");
    next.consumed_request_commitments
        .insert(request.request_commitment);
    next.active_reservations.insert(request.request_commitment);
    Ok(Transition {
        decision: AdmissionDecision::Admitted,
        state: next,
    })
}

/// Idempotently releases concurrency while deliberately leaving the request
/// commitment consumed. A duplicate/late release cannot make it reusable.
pub fn release(state: &AdmissionState, request_commitment: RequestCommitment) -> AdmissionState {
    let mut next = state.clone();
    next.active_reservations.remove(&request_commitment);
    next
}

pub fn revoke(state: &AdmissionState) -> AdmissionState {
    let mut next = state.clone();
    next.revoked = true;
    next
}

pub fn invalidate_capability(state: &AdmissionState) -> AdmissionState {
    let mut next = state.clone();
    next.capability_valid = false;
    next
}

pub fn mark_epoch_not_current(state: &AdmissionState) -> AdmissionState {
    let mut next = state.clone();
    next.epoch_current = false;
    next
}

pub fn close_epoch(state: &AdmissionState) -> AdmissionState {
    let mut next = state.clone();
    next.epoch_current = false;
    next.epoch_closed = true;
    next
}

pub const fn durable_atomicity_measured() -> bool {
    false
}

pub const fn crash_recovery_measured() -> bool {
    false
}

pub const fn distributed_consistency_measured() -> bool {
    false
}

pub const fn enumeration_resistance_established() -> bool {
    false
}

pub const fn production_admitted() -> bool {
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::{LeakageDeclaration, QualificationState};
    use psi_abuse_control_profiles::{
        AdmissionMetadataLeakage, QueryBudget, RawIdentifierPolicy, RequestCommitmentPolicy,
        RevocationPolicy, PROFILE_ID, PSI_002A_SUBJECT,
    };

    fn profile() -> PsiAbuseControlProfile {
        PsiAbuseControlProfile {
            profile_id: PROFILE_ID.into(),
            psi_subject: PSI_002A_SUBJECT.into(),
            service_domain: "contact-discovery-v1".into(),
            epoch_domain: "epoch-0001".into(),
            psi_equality_domain: "mycelix-test:contact-discovery-v1".into(),
            psi_session_domain: "session-a".into(),
            capability_namespace: "contact-discovery-query-v1".into(),
            budget: QueryBudget {
                max_requests_per_epoch: 2,
                max_blinded_elements_per_request: 4,
                max_blinded_elements_per_epoch: 6,
                max_concurrent_requests: 1,
            },
            raw_identifier_policy: RawIdentifierPolicy::ForbiddenAtAdmissionBoundary,
            request_commitment_policy: RequestCommitmentPolicy::UniquePerCapabilityEpoch,
            revocation_policy: RevocationPolicy::CheckBeforeEveryAdmission,
            metadata_leakage: AdmissionMetadataLeakage {
                capability_identity: LeakageDeclaration::MayReveal,
                request_count: LeakageDeclaration::MayReveal,
                blinded_element_count: LeakageDeclaration::MayReveal,
                timing: LeakageDeclaration::MayReveal,
                network_identity: LeakageDeclaration::MayReveal,
            },
            qualification: QualificationState::Experimental,
        }
    }

    fn state() -> AdmissionState {
        AdmissionState::fresh(&profile(), "capability-a").unwrap()
    }

    fn request(byte: u8, count: u32) -> AdmissionRequest {
        AdmissionRequest {
            request_commitment: [byte; 32],
            blinded_element_count: count,
        }
    }

    fn rejected_unchanged(decision: AdmissionDecision, prior: &AdmissionState, result: &Transition) {
        assert_eq!(result.decision, decision);
        assert_eq!(&result.state, prior);
    }

    #[test]
    fn success_consumes_commitment_and_all_budget_dimensions_together() {
        let prior = state();
        let result = admit(&profile(), &prior, request(1, 3)).unwrap();
        assert_eq!(result.decision, AdmissionDecision::Admitted);
        assert_eq!(result.state.requests_used, 1);
        assert_eq!(result.state.elements_used, 3);
        assert_eq!(result.state.concurrent_reserved(), 1);
        assert!(result.state.consumed_request_commitments.contains(&[1; 32]));
        assert!(result.state.active_reservations.contains(&[1; 32]));
    }

    #[test]
    fn replay_is_rejected_without_state_change_even_after_release() {
        let admitted = admit(&profile(), &state(), request(1, 2)).unwrap().state;
        let released = release(&admitted, [1; 32]);
        let replay = admit(&profile(), &released, request(1, 2)).unwrap();
        rejected_unchanged(AdmissionDecision::Replay, &released, &replay);
    }

    #[test]
    fn release_is_idempotent_and_never_unconsumes_commitment() {
        let admitted = admit(&profile(), &state(), request(1, 2)).unwrap().state;
        let once = release(&admitted, [1; 32]);
        let twice = release(&once, [1; 32]);
        assert_eq!(once, twice);
        assert!(twice.consumed_request_commitments.contains(&[1; 32]));
        assert_eq!(twice.concurrent_reserved(), 0);
    }

    #[test]
    fn revoked_capability_rejects_without_state_change() {
        let prior = revoke(&state());
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::CapabilityRevoked, &prior, &result);
    }

    #[test]
    fn invalid_capability_rejects_without_state_change() {
        let prior = invalidate_capability(&state());
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::CapabilityInvalid, &prior, &result);
    }

    #[test]
    fn closed_epoch_has_explicit_precedence_and_preserves_state() {
        let prior = close_epoch(&state());
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::EpochClosed, &prior, &result);
    }

    #[test]
    fn not_current_epoch_is_distinct_from_closed_epoch() {
        let prior = mark_epoch_not_current(&state());
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::EpochNotCurrent, &prior, &result);
    }

    #[test]
    fn malformed_commitment_rejected_without_state_change() {
        let prior = state();
        let result = admit(&profile(), &prior, request(0, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::MalformedRequestCommitment, &prior, &result);
    }

    #[test]
    fn per_request_element_boundary_is_exact() {
        let prior = state();
        assert_eq!(
            admit(&profile(), &prior, request(1, 4)).unwrap().decision,
            AdmissionDecision::Admitted
        );
        let result = admit(&profile(), &prior, request(1, 5)).unwrap();
        rejected_unchanged(
            AdmissionDecision::PerRequestElementBudgetExceeded,
            &prior,
            &result,
        );
    }

    #[test]
    fn concurrency_boundary_rejects_without_spending_more_budget() {
        let admitted = admit(&profile(), &state(), request(1, 1)).unwrap().state;
        let result = admit(&profile(), &admitted, request(2, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::ConcurrencyBudgetExceeded, &admitted, &result);
    }

    #[test]
    fn request_budget_boundary_is_exact_after_release() {
        let first = admit(&profile(), &state(), request(1, 1)).unwrap().state;
        let first = release(&first, [1; 32]);
        let second = admit(&profile(), &first, request(2, 1)).unwrap().state;
        let second = release(&second, [2; 32]);
        let result = admit(&profile(), &second, request(3, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::RequestBudgetExceeded, &second, &result);
    }

    #[test]
    fn epoch_element_boundary_is_exact_after_release() {
        let first = admit(&profile(), &state(), request(1, 4)).unwrap().state;
        let first = release(&first, [1; 32]);
        let result = admit(&profile(), &first, request(2, 3)).unwrap();
        rejected_unchanged(AdmissionDecision::EpochElementBudgetExceeded, &first, &result);
    }

    #[test]
    fn any_profile_drift_fails_closed_without_state_change() {
        let prior = state();
        let mut changed = profile();
        changed.metadata_leakage.network_identity = LeakageDeclaration::DeclaredHidden;
        let result = admit(&changed, &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::MalformedPolicyState, &prior, &result);
    }

    #[test]
    fn malformed_state_fails_closed_without_change() {
        let mut prior = state();
        prior.requests_used = 99;
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::MalformedPolicyState, &prior, &result);
    }

    #[test]
    fn active_reservation_must_already_be_consumed() {
        let mut prior = state();
        prior.active_reservations.insert([9; 32]);
        let result = admit(&profile(), &prior, request(1, 1)).unwrap();
        rejected_unchanged(AdmissionDecision::MalformedPolicyState, &prior, &result);
    }

    #[test]
    fn zero_element_request_is_rejected_without_change() {
        let prior = state();
        let result = admit(&profile(), &prior, request(1, 0)).unwrap();
        rejected_unchanged(
            AdmissionDecision::PerRequestElementBudgetExceeded,
            &prior,
            &result,
        );
    }

    #[test]
    fn authority_ceiling_remains_false() {
        assert!(!durable_atomicity_measured());
        assert!(!crash_recovery_measured());
        assert!(!distributed_consistency_measured());
        assert!(!enumeration_resistance_established());
        assert!(!production_admitted());
    }
}
