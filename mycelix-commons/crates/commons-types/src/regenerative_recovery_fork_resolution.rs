// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Explicit evidence-bearing resolution for regenerative recovery reserve forks.
//!
//! Fork detection is not fork resolution. A local cursor freezes when it observes
//! two distinct recovery records occupying the same reserve spend sequence. This
//! module permits the cursor to resume only when a separate governance/evidence
//! record binds the exact two branches and either selects one or explicitly leaves
//! the reserve suspended.

use crate::{
    RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryReserveHeadV1,
    verify_regenerative_recovery_reserve_lineage,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1: u8 = 1;
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeRecoveryForkResolutionOutcomeV1 {
    RetainSelectedBranch,
    SuspendReserve,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryForkResolutionEvidenceV1 {
    pub schema_version: u8,
    pub resolution_id: String,
    pub external_recovery_reserve_id: String,
    pub external_recovery_reserve_binding: String,
    pub viability_evidence_content_digest: String,
    pub support_closure_continuity_content_digest: String,
    pub fork_spend_sequence: u64,
    pub fork_predecessor_recovery_evidence_content_digest: Option<String>,
    /// Exactly two distinct branch digests in canonical lexical order.
    pub conflicting_recovery_evidence_content_digests: Vec<String>,
    pub outcome: RegenerativeRecoveryForkResolutionOutcomeV1,
    /// Required for `RetainSelectedBranch`; forbidden for `SuspendReserve`.
    pub selected_recovery_evidence_content_digest: Option<String>,
    /// Evidence that the fork itself was observed by the resolver.
    pub fork_observation_evidence_binding: String,
    /// Authority/policy under which the resolver is permitted to adjudicate.
    pub governance_authority_binding: String,
    /// Exact decision record / signature / transcript binding.
    pub resolution_evidence_binding: String,
}

impl RegenerativeRecoveryForkResolutionEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative recovery fork-resolution schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.resolution_id) || !canonical_id(&self.external_recovery_reserve_id) {
            return Err("recovery fork-resolution identifier is not canonical".into());
        }
        if !canonical_reference(&self.external_recovery_reserve_binding)
            || !canonical_reference(&self.fork_observation_evidence_binding)
            || !canonical_reference(&self.governance_authority_binding)
            || !canonical_reference(&self.resolution_evidence_binding)
        {
            return Err("recovery fork-resolution binding is not canonical".into());
        }
        if !lower_hex_64(&self.viability_evidence_content_digest)
            || !lower_hex_64(&self.support_closure_continuity_content_digest)
        {
            return Err("recovery fork-resolution nominal digest is not lowercase 64-hex".into());
        }
        if let Some(predecessor) = &self.fork_predecessor_recovery_evidence_content_digest {
            if !lower_hex_64(predecessor) {
                return Err("recovery fork predecessor digest is not lowercase 64-hex".into());
            }
        }
        if self.conflicting_recovery_evidence_content_digests.len() != 2 {
            return Err("recovery fork resolution must bind exactly two branch digests".into());
        }
        let first = &self.conflicting_recovery_evidence_content_digests[0];
        let second = &self.conflicting_recovery_evidence_content_digests[1];
        if !lower_hex_64(first) || !lower_hex_64(second) || first >= second {
            return Err("recovery fork branch digests must be distinct canonical lexical order".into());
        }
        match self.outcome {
            RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch => {
                let selected = self
                    .selected_recovery_evidence_content_digest
                    .as_deref()
                    .ok_or_else(|| "retain outcome requires a selected branch digest".to_string())?;
                if !self
                    .conflicting_recovery_evidence_content_digests
                    .iter()
                    .any(|digest| digest == selected)
                {
                    return Err("selected recovery branch is not one of the bound fork branches".into());
                }
            }
            RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve => {
                if self.selected_recovery_evidence_content_digest.is_some() {
                    return Err("suspended reserve cannot also select a fork branch".into());
                }
            }
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize recovery fork resolution: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-recovery-fork-resolution-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }
}

pub fn verify_regenerative_recovery_fork_resolution(
    head: &RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
) -> Result<(), String> {
    resolution.validate()?;
    if !head.fork_pending() {
        return Err("recovery reserve cursor has no pending fork to resolve".into());
    }

    current_branch.validate()?;
    sibling_branch.validate()?;
    let current_digest = current_branch.content_digest()?;
    let sibling_digest = sibling_branch.content_digest()?;
    if current_digest == sibling_digest {
        return Err("recovery fork requires two distinct branch records".into());
    }
    if head.evidence_digest() != current_digest {
        return Err("current recovery branch does not match the frozen cursor head".into());
    }
    if head.pending_sibling_fork_digest() != Some(sibling_digest.as_str()) {
        return Err("sibling recovery branch does not match the observed pending fork".into());
    }

    if current_branch.external_recovery_reserve_id != sibling_branch.external_recovery_reserve_id
        || current_branch.external_recovery_reserve_binding
            != sibling_branch.external_recovery_reserve_binding
        || current_branch.external_recovery_reserve_initial_units
            != sibling_branch.external_recovery_reserve_initial_units
        || current_branch.viability_evidence_content_digest
            != sibling_branch.viability_evidence_content_digest
        || current_branch.support_closure_continuity_content_digest
            != sibling_branch.support_closure_continuity_content_digest
        || current_branch.reserve_spend_sequence_after != sibling_branch.reserve_spend_sequence_after
        || current_branch.reserve_spend_sequence_before
            != sibling_branch.reserve_spend_sequence_before
        || current_branch.reserve_units_before != sibling_branch.reserve_units_before
        || current_branch.previous_recovery_evidence_content_digest
            != sibling_branch.previous_recovery_evidence_content_digest
    {
        return Err("recovery fork branches do not share one exact predecessor subject".into());
    }

    if current_branch.reserve_spend_sequence_before == 0 {
        verify_regenerative_recovery_reserve_lineage(None, current_branch)?;
        verify_regenerative_recovery_reserve_lineage(None, sibling_branch)?;
    } else {
        // The cursor itself already proves the accepted branch's predecessor was
        // previously ingested. Here we require the sibling to claim that same exact
        // predecessor digest; replaying the unavailable predecessor object is not
        // necessary to bind the fork-resolution subject.
        if current_branch.previous_recovery_evidence_content_digest.is_none() {
            return Err("non-root recovery fork is missing predecessor evidence".into());
        }
    }

    if resolution.external_recovery_reserve_id != current_branch.external_recovery_reserve_id
        || resolution.external_recovery_reserve_binding
            != current_branch.external_recovery_reserve_binding
        || resolution.viability_evidence_content_digest
            != current_branch.viability_evidence_content_digest
        || resolution.support_closure_continuity_content_digest
            != current_branch.support_closure_continuity_content_digest
        || resolution.fork_spend_sequence != current_branch.reserve_spend_sequence_after
        || resolution.fork_predecessor_recovery_evidence_content_digest
            != current_branch.previous_recovery_evidence_content_digest
    {
        return Err("recovery fork-resolution subject mismatch".into());
    }

    let mut expected = vec![current_digest, sibling_digest];
    expected.sort();
    if resolution.conflicting_recovery_evidence_content_digests != expected {
        return Err("recovery fork-resolution branch set mismatch".into());
    }
    Ok(())
}

pub fn apply_regenerative_recovery_fork_resolution(
    head: &mut RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
) -> Result<bool, String> {
    verify_regenerative_recovery_fork_resolution(head, current_branch, sibling_branch, resolution)?;
    match resolution.outcome {
        RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve => Ok(false),
        RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch => {
            let selected_digest = resolution
                .selected_recovery_evidence_content_digest
                .as_deref()
                .ok_or_else(|| "retain outcome lost selected branch digest".to_string())?;
            let selected = if selected_digest == current_branch.content_digest()? {
                current_branch
            } else if selected_digest == sibling_branch.content_digest()? {
                sibling_branch
            } else {
                return Err("selected recovery branch is outside the verified fork".into());
            };
            head.resolve_pending_sibling_fork_to(selected)?;
            Ok(true)
        }
    }
}

fn canonical_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_ID_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn canonical_reference(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_BINDING_BYTES
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        RegenerativeRecoveryFlowKindEvidenceV1, RegenerativeRecoveryReserveDispositionV1,
        REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
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
        selected: Option<String>,
    ) -> RegenerativeRecoveryForkResolutionEvidenceV1 {
        let mut branches = vec![current.content_digest().unwrap(), sibling.content_digest().unwrap()];
        branches.sort();
        RegenerativeRecoveryForkResolutionEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
            resolution_id: "resolution-1".into(),
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
            fork_observation_evidence_binding: "fork-observation:reserve-test:sequence-2".into(),
            governance_authority_binding: "governance:reserve-recovery-resolution:v1".into(),
            resolution_evidence_binding: "resolution-evidence:reserve-test:sequence-2:v1".into(),
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
        assert!(head.fork_pending());
        (head, current, sibling)
    }

    #[test]
    fn explicit_resolution_can_retain_current_branch_and_resume() {
        let (mut head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, Some(current.content_digest().unwrap()));
        assert!(apply_regenerative_recovery_fork_resolution(
            &mut head,
            &current,
            &sibling,
            &decision,
        )
        .unwrap());
        assert!(!head.fork_pending());
        assert_eq!(head.available_units(), current.reserve_units_after);

        let child = record(
            "spend-3",
            2,
            current.reserve_units_after,
            1,
            Some(current.content_digest().unwrap()),
        );
        assert_eq!(
            head.ingest(&child).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Advance
        );
    }

    #[test]
    fn explicit_resolution_can_switch_to_sibling_branch() {
        let (mut head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, Some(sibling.content_digest().unwrap()));
        assert!(apply_regenerative_recovery_fork_resolution(
            &mut head,
            &current,
            &sibling,
            &decision,
        )
        .unwrap());
        assert!(!head.fork_pending());
        assert_eq!(head.evidence_digest(), sibling.content_digest().unwrap());
        assert_eq!(head.available_units(), sibling.reserve_units_after);
    }

    #[test]
    fn suspension_is_explicit_and_keeps_cursor_frozen() {
        let (mut head, current, sibling) = frozen_fork();
        let decision = resolution(&current, &sibling, None);
        assert!(!apply_regenerative_recovery_fork_resolution(
            &mut head,
            &current,
            &sibling,
            &decision,
        )
        .unwrap());
        assert!(head.fork_pending());
    }

    #[test]
    fn resolution_cannot_select_an_unobserved_third_branch() {
        let (head, current, sibling) = frozen_fork();
        let mut decision = resolution(&current, &sibling, Some(current.content_digest().unwrap()));
        decision.selected_recovery_evidence_content_digest = Some("aa".repeat(32));
        assert!(verify_regenerative_recovery_fork_resolution(
            &head,
            &current,
            &sibling,
            &decision,
        )
        .is_err());
    }
}
