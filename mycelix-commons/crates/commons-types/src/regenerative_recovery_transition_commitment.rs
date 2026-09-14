// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic commitment to one recovery-reserve fork transition.
//!
//! This module does not establish governance authority and does not persist state.
//! It converts an already well-formed fork resolution into an exact pre-state ->
//! post-state commitment so a future trusted runtime can independently load the
//! authoritative head, requalify governance, recompute this transition, and commit
//! the resulting state atomically.

use crate::{
    RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryForkResolutionOutcomeV1, RegenerativeRecoveryReserveHeadV1,
    apply_regenerative_recovery_fork_resolution, verify_regenerative_recovery_fork_resolution,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1: u8 = 1;
pub const REGENERATIVE_RECOVERY_HEAD_SNAPSHOT_PROFILE_V1: &str =
    "mycelix-regenerative-recovery-head-snapshot-v1-blake3-json";
pub const REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_PROFILE_V1: &str =
    "mycelix-regenerative-recovery-transition-v1-blake3-json";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryReserveHeadSnapshotV1 {
    pub reserve_id: String,
    pub reserve_binding: String,
    pub reserve_initial_units: u64,
    pub viability_evidence_content_digest: String,
    pub support_closure_continuity_content_digest: String,
    pub spend_sequence: u64,
    pub available_units: u64,
    pub evidence_digest: String,
    pub pending_sibling_fork_digest: Option<String>,
}

impl RegenerativeRecoveryReserveHeadSnapshotV1 {
    pub fn from_head(head: &RegenerativeRecoveryReserveHeadV1) -> Self {
        Self {
            reserve_id: head.reserve_id().to_string(),
            reserve_binding: head.reserve_binding().to_string(),
            reserve_initial_units: head.reserve_initial_units(),
            viability_evidence_content_digest: head.viability_evidence_content_digest().to_string(),
            support_closure_continuity_content_digest: head
                .support_closure_continuity_content_digest()
                .to_string(),
            spend_sequence: head.spend_sequence(),
            available_units: head.available_units(),
            evidence_digest: head.evidence_digest().to_string(),
            pending_sibling_fork_digest: head.pending_sibling_fork_digest().map(str::to_string),
        }
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = serde_json::to_vec(self)
            .map_err(|error| format!("failed to serialize recovery head snapshot: {error}"))?;
        Ok(framed_digest(
            b"mycelix-regenerative-recovery-head-snapshot-v1\0",
            REGENERATIVE_RECOVERY_HEAD_SNAPSHOT_PROFILE_V1.as_bytes(),
            &payload,
        ))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryTransitionCommitmentV1 {
    pub schema_version: u8,
    pub pre_state_digest: String,
    pub fork_resolution_content_digest: String,
    pub current_branch_content_digest: String,
    pub sibling_branch_content_digest: String,
    pub outcome: RegenerativeRecoveryForkResolutionOutcomeV1,
    pub selected_branch_content_digest: Option<String>,
    pub post_state_digest: String,
    pub cursor_resumed: bool,
}

impl RegenerativeRecoveryTransitionCommitmentV1 {
    /// This object is deterministic transition evidence, not governance authority.
    pub const fn governance_authority_verified_here(&self) -> bool {
        false
    }

    /// This object cannot authorize persistence merely because it deserializes.
    pub const fn persistence_authorized_here(&self) -> bool {
        false
    }

    pub fn content_digest(&self) -> Result<String, String> {
        if self.schema_version != REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1 {
            return Err("unsupported recovery transition commitment schema".into());
        }
        let payload = serde_json::to_vec(self)
            .map_err(|error| format!("failed to serialize recovery transition commitment: {error}"))?;
        Ok(framed_digest(
            b"mycelix-regenerative-recovery-transition-v1\0",
            REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_PROFILE_V1.as_bytes(),
            &payload,
        ))
    }
}

/// Recompute the exact pure state transition implied by one verified fork subject
/// and resolution. The caller retains responsibility for governance authority,
/// authoritative-state loading, compare-and-swap/transactionality, and persistence.
pub fn qualify_regenerative_recovery_transition_commitment(
    head: &RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
) -> Result<RegenerativeRecoveryTransitionCommitmentV1, String> {
    verify_regenerative_recovery_fork_resolution(
        head,
        current_branch,
        sibling_branch,
        resolution,
    )?;

    let pre = RegenerativeRecoveryReserveHeadSnapshotV1::from_head(head);
    let pre_state_digest = pre.content_digest()?;
    let fork_resolution_content_digest = resolution.content_digest()?;
    let current_branch_content_digest = current_branch.content_digest()?;
    let sibling_branch_content_digest = sibling_branch.content_digest()?;

    let mut post_head = head.clone();
    let cursor_resumed = apply_regenerative_recovery_fork_resolution(
        &mut post_head,
        current_branch,
        sibling_branch,
        resolution,
    )?;
    let post = RegenerativeRecoveryReserveHeadSnapshotV1::from_head(&post_head);
    let post_state_digest = post.content_digest()?;

    match resolution.outcome {
        RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve => {
            if cursor_resumed || pre != post {
                return Err("suspended recovery fork unexpectedly changed cursor state".into());
            }
        }
        RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch => {
            if !cursor_resumed || post.pending_sibling_fork_digest.is_some() {
                return Err("retained recovery branch did not resume the frozen cursor".into());
            }
            let selected = resolution
                .selected_recovery_evidence_content_digest
                .as_deref()
                .ok_or_else(|| "retained recovery branch lost selected digest".to_string())?;
            if post.evidence_digest != selected {
                return Err("post-state does not bind the governance-selected branch".into());
            }
        }
    }

    Ok(RegenerativeRecoveryTransitionCommitmentV1 {
        schema_version: REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1,
        pre_state_digest,
        fork_resolution_content_digest,
        current_branch_content_digest,
        sibling_branch_content_digest,
        outcome: resolution.outcome,
        selected_branch_content_digest: resolution
            .selected_recovery_evidence_content_digest
            .clone(),
        post_state_digest,
        cursor_resumed,
    })
}

pub fn verify_regenerative_recovery_transition_commitment(
    head: &RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    expected: &RegenerativeRecoveryTransitionCommitmentV1,
) -> Result<(), String> {
    let recomputed = qualify_regenerative_recovery_transition_commitment(
        head,
        current_branch,
        sibling_branch,
        resolution,
    )?;
    if &recomputed != expected {
        return Err("recovery transition commitment does not match exact recomputation".into());
    }
    Ok(())
}

fn framed_digest(domain: &[u8], profile: &[u8], payload: &[u8]) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(domain);
    hasher.update(&(profile.len() as u64).to_le_bytes());
    hasher.update(profile);
    hasher.update(&(payload.len() as u64).to_le_bytes());
    hasher.update(payload);
    hasher.finalize().to_hex().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
        REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
        RegenerativeRecoveryFlowKindEvidenceV1, RegenerativeRecoveryReserveDispositionV1,
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
        selected: Option<String>,
    ) -> RegenerativeRecoveryForkResolutionEvidenceV1 {
        let mut branches = vec![
            current.content_digest().unwrap(),
            sibling.content_digest().unwrap(),
        ];
        branches.sort();
        RegenerativeRecoveryForkResolutionEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
            resolution_id: "resolution-transition-1".into(),
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
            outcome: if selected.is_some() {
                RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch
            } else {
                RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve
            },
            selected_recovery_evidence_content_digest: selected,
            fork_observation_evidence_binding: "fork-observation:reserve-test:2".into(),
            governance_authority_binding: "governance:reserve-recovery-resolution:v1".into(),
            resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
        }
    }

    #[test]
    fn switch_commitment_binds_exact_pre_and_post_state() {
        let (head, current, sibling) = frozen_fork();
        let sibling_digest = sibling.content_digest().unwrap();
        let decision = resolution(&current, &sibling, Some(sibling_digest.clone()));
        let commitment = qualify_regenerative_recovery_transition_commitment(
            &head, &current, &sibling, &decision,
        )
        .unwrap();

        assert!(commitment.cursor_resumed);
        assert_eq!(
            commitment.selected_branch_content_digest.as_deref(),
            Some(sibling_digest.as_str())
        );
        assert_ne!(commitment.pre_state_digest, commitment.post_state_digest);
        assert!(!commitment.governance_authority_verified_here());
        assert!(!commitment.persistence_authorized_here());
        verify_regenerative_recovery_transition_commitment(
            &head,
            &current,
            &sibling,
            &decision,
            &commitment,
        )
        .unwrap();
    }

    #[test]
    fn suspension_commitment_is_a_no_state_change_transition() {
        let (head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, None);
        let commitment = qualify_regenerative_recovery_transition_commitment(
            &head, &current, &sibling, &decision,
        )
        .unwrap();

        assert!(!commitment.cursor_resumed);
        assert_eq!(commitment.pre_state_digest, commitment.post_state_digest);
        assert!(commitment.selected_branch_content_digest.is_none());
    }

    #[test]
    fn selecting_another_branch_changes_transition_identity() {
        let (head, current, sibling) = frozen_fork();
        let keep_current = resolution(&current, &sibling, Some(current.content_digest().unwrap()));
        let switch_sibling = resolution(&current, &sibling, Some(sibling.content_digest().unwrap()));
        let first = qualify_regenerative_recovery_transition_commitment(
            &head,
            &current,
            &sibling,
            &keep_current,
        )
        .unwrap();
        let second = qualify_regenerative_recovery_transition_commitment(
            &head,
            &current,
            &sibling,
            &switch_sibling,
        )
        .unwrap();

        assert_ne!(first.content_digest().unwrap(), second.content_digest().unwrap());
        assert_ne!(first.post_state_digest, second.post_state_digest);
    }

    #[test]
    fn commitment_is_bound_to_the_exact_frozen_prestate() {
        let (mut head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, Some(current.content_digest().unwrap()));
        let commitment = qualify_regenerative_recovery_transition_commitment(
            &head, &current, &sibling, &decision,
        )
        .unwrap();

        assert!(
            apply_regenerative_recovery_fork_resolution(&mut head, &current, &sibling, &decision)
                .unwrap()
        );
        assert!(!head.fork_pending());
        assert!(
            verify_regenerative_recovery_transition_commitment(
                &head,
                &current,
                &sibling,
                &decision,
                &commitment,
            )
            .is_err()
        );
    }

    #[test]
    fn tampered_commitment_fails_exact_recomputation() {
        let (head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, Some(current.content_digest().unwrap()));
        let mut commitment = qualify_regenerative_recovery_transition_commitment(
            &head, &current, &sibling, &decision,
        )
        .unwrap();
        commitment.post_state_digest = "aa".repeat(32);

        assert!(
            verify_regenerative_recovery_transition_commitment(
                &head,
                &current,
                &sibling,
                &decision,
                &commitment,
            )
            .is_err()
        );
    }
}
