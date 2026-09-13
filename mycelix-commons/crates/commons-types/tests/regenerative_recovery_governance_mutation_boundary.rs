// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Regression proof that portable, internally consistent governance evidence is
//! not sufficient to mutate a recovery-reserve cursor inside Commons.

use commons_types::{
    GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1, REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
    REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
    REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1, RegenerativeRecoveryCoordinateEvidenceV1,
    RegenerativeRecoveryFlowKindEvidenceV1, RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryForkResolutionOutcomeV1, RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
    RegenerativeRecoveryReserveDispositionV1, RegenerativeRecoveryReserveHeadV1,
    THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1,
    apply_regenerative_recovery_fork_resolution_with_bound_governance_authority,
    qualify_regenerative_recovery_fork_resolution_governance_binding,
    qualify_regenerative_recovery_governance_action,
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

fn resolution(
    current: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling: &RegenerativeRecoveryCoordinateEvidenceV1,
    threshold_identity: &str,
) -> RegenerativeRecoveryForkResolutionEvidenceV1 {
    let mut branches = vec![
        current.content_digest().unwrap(),
        sibling.content_digest().unwrap(),
    ];
    branches.sort();
    RegenerativeRecoveryForkResolutionEvidenceV1 {
        schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
        resolution_id: "resolution-security-boundary-1".into(),
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
        selected_recovery_evidence_content_digest: Some(sibling.content_digest().unwrap()),
        fork_observation_evidence_binding: "fork-observation:reserve-test:2".into(),
        governance_authority_binding: format!(
            "threshold-authorization:{}:{}",
            THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1, threshold_identity
        ),
        resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
    }
}

fn authority(
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    threshold_identity: &str,
) -> RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
    let action = qualify_regenerative_recovery_governance_action("MIP-9001", resolution).unwrap();
    RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
        schema_version: REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1,
        authority_evidence_id: "authority-security-boundary-1".into(),
        governance_proposal_id: "MIP-9001".into(),
        threshold_qualification_evidence_binding:
            "mycelix-governance-threshold-qualification:receipt-security-1".into(),
        threshold_identity_evidence_binding:
            "mycelix-governance-threshold-identity:receipt-security-1".into(),
        threshold_authorization_ref: "threshold-authorization:record-security-1".into(),
        threshold_authorization_identity_digest: threshold_identity.into(),
        threshold_authorization_identity_profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1
            .into(),
        qualified_actions_digest: action.actions_digest().into(),
        qualified_actions_digest_profile: GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1.into(),
    }
}

#[test]
fn internally_consistent_but_unverified_authority_never_moves_the_cursor() {
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
    assert!(head.fork_pending());

    let identity = "66".repeat(32);
    let resolution = resolution(&current, &sibling, &identity);
    let authority = authority(&resolution, &identity);

    let binding =
        qualify_regenerative_recovery_fork_resolution_governance_binding(&resolution, &authority)
            .unwrap();
    assert!(!binding.upstream_threshold_authority_verified_here());
    assert!(!binding.recovery_cursor_mutation_authorized_here());

    let frozen_before = head.clone();
    let result = apply_regenerative_recovery_fork_resolution_with_bound_governance_authority(
        &mut head,
        &current,
        &sibling,
        &resolution,
        &authority,
    );

    assert!(result.is_err());
    assert_eq!(head, frozen_before);
    assert!(head.fork_pending());
}
