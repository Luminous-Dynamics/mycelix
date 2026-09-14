// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Ephemeral exact-source qualification for recovery transition transport.
//!
//! This is the bridge between transport integrity and live authority. It proves
//! that one already-validated transport commitment exactly recomputes from the
//! supplied frozen recovery head, both exact fork branches, and the exact fork
//! resolution. The resulting token is intentionally non-serializable and
//! non-cloneable so it cannot become a portable/replayable authority artifact.
//!
//! This module still does not verify live governance/current-provider authority
//! and does not authorize or execute persistence.

use crate::{
    RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryReserveHeadV1, RegenerativeRecoveryTransitionTransportV1,
    validate_regenerative_recovery_transition_transport,
    verify_regenerative_recovery_transition_commitment,
};

/// In-process proof that transport integrity and exact transition recomputation
/// both succeeded against the supplied source objects.
///
/// Deliberately no `Clone`, `Serialize`, or `Deserialize` implementation.
#[derive(Debug)]
pub struct RegenerativeRecoveryTransitionSourceQualificationV1 {
    pre_state_digest: String,
    fork_resolution_content_digest: String,
    current_branch_content_digest: String,
    sibling_branch_content_digest: String,
    post_state_digest: String,
    commitment_content_digest: String,
    cursor_resumed: bool,
    _sealed: (),
}

impl RegenerativeRecoveryTransitionSourceQualificationV1 {
    pub fn pre_state_digest(&self) -> &str {
        &self.pre_state_digest
    }

    pub fn fork_resolution_content_digest(&self) -> &str {
        &self.fork_resolution_content_digest
    }

    pub fn current_branch_content_digest(&self) -> &str {
        &self.current_branch_content_digest
    }

    pub fn sibling_branch_content_digest(&self) -> &str {
        &self.sibling_branch_content_digest
    }

    pub fn post_state_digest(&self) -> &str {
        &self.post_state_digest
    }

    pub fn commitment_content_digest(&self) -> &str {
        &self.commitment_content_digest
    }

    pub const fn cursor_resumed(&self) -> bool {
        self.cursor_resumed
    }

    /// Unlike transport-only validation, this exact source recomputation has run.
    pub const fn exact_source_recomputation_verified_here(&self) -> bool {
        true
    }

    /// Live #69/#71/#82/#85 authority remains outside this theorem.
    pub const fn live_governance_authority_verified_here(&self) -> bool {
        false
    }

    /// Exact source qualification is still not permission to mutate durable state.
    pub const fn persistence_authorized_here(&self) -> bool {
        false
    }

    /// This theorem never performs storage mutation.
    pub const fn persistence_executed_here(&self) -> bool {
        false
    }
}

/// Validate transport integrity and then re-run #814's exact transition theorem
/// against all original source objects.
///
/// The returned token is an ephemeral in-process fact. A future trusted provider
/// must still directly requalify live governance/currentness and perform an atomic
/// compare-and-swap against the authoritative stored pre-state before mutation.
pub fn qualify_regenerative_recovery_transition_sources(
    transport: &RegenerativeRecoveryTransitionTransportV1,
    head: &RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
) -> Result<RegenerativeRecoveryTransitionSourceQualificationV1, String> {
    validate_regenerative_recovery_transition_transport(transport)?;
    verify_regenerative_recovery_transition_commitment(
        head,
        current_branch,
        sibling_branch,
        resolution,
        &transport.commitment,
    )?;

    Ok(RegenerativeRecoveryTransitionSourceQualificationV1 {
        pre_state_digest: transport.commitment.pre_state_digest.clone(),
        fork_resolution_content_digest: transport
            .commitment
            .fork_resolution_content_digest
            .clone(),
        current_branch_content_digest: transport
            .commitment
            .current_branch_content_digest
            .clone(),
        sibling_branch_content_digest: transport
            .commitment
            .sibling_branch_content_digest
            .clone(),
        post_state_digest: transport.commitment.post_state_digest.clone(),
        commitment_content_digest: transport.commitment_content_digest.clone(),
        cursor_resumed: transport.commitment.cursor_resumed,
        _sealed: (),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
        REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
        RegenerativeRecoveryFlowKindEvidenceV1, RegenerativeRecoveryForkResolutionOutcomeV1,
        RegenerativeRecoveryReserveDispositionV1,
        RegenerativeRecoveryTransitionTransportV1,
        qualify_regenerative_recovery_transition_commitment,
    };

    fn record(
        id: &str,
        before_sequence: u64,
        before_units: u64,
        cost: u64,
        predecessor: Option<String>,
    ) -> RegenerativeRecoveryCoordinateEvidenceV1 {
        RegenerativeRecoveryCoordinateEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
            recovery_evidence_id: id.into(),
            viability_evidence_content_digest: "11".repeat(32),
            support_closure_continuity_content_digest: "22".repeat(32),
            symthaea_recovery_binding: "symthaea:recovery:test".into(),
            symtropy_recovery_binding: "symtropy:recovery:test".into(),
            semantic_recovery_fixture_binding: "fixture:semantic:test".into(),
            dynamic_recovery_fixture_binding: "fixture:dynamic:test".into(),
            successor_profile_id: "profile-test".into(),
            successor_profile_evidence_binding: "profile:test:evidence".into(),
            successor_model_binding: "model:test".into(),
            successor_support_binding: "support:test".into(),
            recovery_policy_id: "policy-test".into(),
            recovery_policy_evidence_binding: "policy:test:evidence".into(),
            recovery_qualification_binding: "qualification:test".into(),
            disturbance_id: format!("disturbance-{id}"),
            disturbance_evidence_binding: format!("disturbance:{id}:evidence"),
            dynamic_disturbance_observation_binding: format!("observation:{id}"),
            target_dependency_id: "measurement".into(),
            flow_kind: RegenerativeRecoveryFlowKindEvidenceV1::Production,
            healthy_units_per_period: 1,
            degraded_units_per_period: 0,
            external_recovery_reserve_id: "reserve-test".into(),
            external_recovery_reserve_binding: "reserve:test:evidence".into(),
            external_recovery_reserve_initial_units: 3,
            external_recovery_reserve_units_at_qualification: 3,
            reserve_units_per_recovery: cost,
            reserve_units_before: before_units,
            reserve_units_after: before_units - cost,
            reserve_spend_sequence_before: before_sequence,
            reserve_spend_sequence_after: before_sequence + 1,
            previous_recovery_evidence_content_digest: predecessor,
            reserve_external_to_nominal_closure: true,
            dynamic_recovery_receipt_binding: format!("receipt:{id}"),
            recovery_qualified: true,
            disturbance_conditioned_recovery_authorized: true,
            observed_maturity_periods: 1,
            unrecoverable_first_unavailable_period: 1,
            recoverable_maturity_completed: true,
        }
    }

    fn frozen_fork() -> (
        RegenerativeRecoveryReserveHeadV1,
        RegenerativeRecoveryCoordinateEvidenceV1,
        RegenerativeRecoveryCoordinateEvidenceV1,
    ) {
        let first = record("spend-1", 0, 3, 1, None);
        let mut head = RegenerativeRecoveryReserveHeadV1::from_first(&first).unwrap();
        let predecessor = first.content_digest().unwrap();
        let current = record("spend-2a", 1, 2, 1, Some(predecessor.clone()));
        let sibling = record("spend-2b", 1, 2, 2, Some(predecessor));
        assert_eq!(
            head.ingest(&current).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Advance
        );
        assert_eq!(
            head.ingest(&sibling).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Fork
        );
        (head, current, sibling)
    }

    fn resolution(
        current: &RegenerativeRecoveryCoordinateEvidenceV1,
        sibling: &RegenerativeRecoveryCoordinateEvidenceV1,
        selected: String,
    ) -> RegenerativeRecoveryForkResolutionEvidenceV1 {
        let mut branches = vec![
            current.content_digest().unwrap(),
            sibling.content_digest().unwrap(),
        ];
        branches.sort();
        RegenerativeRecoveryForkResolutionEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
            resolution_id: "resolution-source-1".into(),
            external_recovery_reserve_id: current.external_recovery_reserve_id.clone(),
            external_recovery_reserve_binding: current.external_recovery_reserve_binding.clone(),
            viability_evidence_content_digest: current.viability_evidence_content_digest.clone(),
            support_closure_continuity_content_digest: current
                .support_closure_continuity_content_digest
                .clone(),
            fork_spend_sequence: current.reserve_spend_sequence_after,
            fork_predecessor_recovery_evidence_content_digest: current
                .previous_recovery_evidence_content_digest
                .clone(),
            conflicting_recovery_evidence_content_digests: branches,
            outcome: RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch,
            selected_recovery_evidence_content_digest: Some(selected),
            fork_observation_evidence_binding: "fork-observation:reserve-test:2".into(),
            governance_authority_binding: "governance:reserve-recovery-resolution:v1".into(),
            resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
        }
    }

    fn qualified_subject() -> (
        RegenerativeRecoveryReserveHeadV1,
        RegenerativeRecoveryCoordinateEvidenceV1,
        RegenerativeRecoveryCoordinateEvidenceV1,
        RegenerativeRecoveryForkResolutionEvidenceV1,
        RegenerativeRecoveryTransitionTransportV1,
    ) {
        let (head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, sibling.content_digest().unwrap());
        let commitment = qualify_regenerative_recovery_transition_commitment(
            &head,
            &current,
            &sibling,
            &decision,
        )
        .unwrap();
        let transport = RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment)
            .unwrap();
        (head, current, sibling, decision, transport)
    }

    #[test]
    fn exact_sources_mint_only_ephemeral_source_qualification() {
        let (head, current, sibling, decision, transport) = qualified_subject();
        let qualified = qualify_regenerative_recovery_transition_sources(
            &transport,
            &head,
            &current,
            &sibling,
            &decision,
        )
        .unwrap();

        assert!(qualified.exact_source_recomputation_verified_here());
        assert!(!qualified.live_governance_authority_verified_here());
        assert!(!qualified.persistence_authorized_here());
        assert!(!qualified.persistence_executed_here());
        assert_eq!(
            qualified.commitment_content_digest(),
            transport.commitment_content_digest
        );
        assert_eq!(qualified.pre_state_digest(), transport.commitment.pre_state_digest);
        assert_eq!(qualified.post_state_digest(), transport.commitment.post_state_digest);
    }

    #[test]
    fn stale_or_already_applied_head_cannot_reuse_source_qualification() {
        let (mut head, current, sibling, decision, transport) = qualified_subject();
        assert!(
            crate::apply_regenerative_recovery_fork_resolution(
                &mut head,
                &current,
                &sibling,
                &decision,
            )
            .unwrap()
        );
        assert!(
            qualify_regenerative_recovery_transition_sources(
                &transport,
                &head,
                &current,
                &sibling,
                &decision,
            )
            .is_err()
        );
    }

    #[test]
    fn changed_branch_or_resolution_cannot_reuse_old_transport() {
        let (head, current, mut sibling, decision, transport) = qualified_subject();
        sibling.recovery_evidence_id = "spend-2b-changed".into();
        assert!(
            qualify_regenerative_recovery_transition_sources(
                &transport,
                &head,
                &current,
                &sibling,
                &decision,
            )
            .is_err()
        );

        let (_, current, sibling, mut changed_decision, transport) = qualified_subject();
        changed_decision.resolution_evidence_binding = "resolution-evidence:changed:v1".into();
        assert!(
            qualify_regenerative_recovery_transition_sources(
                &transport,
                &head,
                &current,
                &sibling,
                &changed_decision,
            )
            .is_err()
        );
    }

    #[test]
    fn transport_tampering_is_rejected_before_source_token_is_minted() {
        let (head, current, sibling, decision, mut transport) = qualified_subject();
        transport.commitment.post_state_digest = "aa".repeat(32);
        assert!(
            qualify_regenerative_recovery_transition_sources(
                &transport,
                &head,
                &current,
                &sibling,
                &decision,
            )
            .is_err()
        );
    }
}
