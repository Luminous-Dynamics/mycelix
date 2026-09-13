// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: Apache-2.0 OR MIT
//! Local linear cursor for one evidence-bound regenerative recovery reserve.
//!
//! Pairwise predecessor verification proves that one candidate can follow one
//! predecessor. It cannot, by itself, detect two distinct children that both claim
//! the same predecessor. This cursor closes that local ingest gap: once one spend
//! at sequence N is accepted, a different record for the same N is a fork rather
//! than another valid continuation.
//!
//! A sibling fork freezes automatic advancement. This is important: otherwise the
//! first branch observed would become an implicit governance decision. A frozen
//! cursor can resume only through the explicit fork-resolution layer.
//!
//! This remains a local classification primitive, not global consensus. Multiple
//! branches may exist in a distributed store; reconciliation decides which branch,
//! if any, is authoritative.

use crate::{
    verify_regenerative_recovery_reserve_lineage, RegenerativeRecoveryCoordinateEvidenceV1,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeRecoveryReserveDispositionV1 {
    Advance,
    Duplicate,
    StaleReplay,
    Gap,
    Fork,
    FrozenPendingResolution,
    WrongReserve,
    WrongNominalSubject,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryReserveHeadV1 {
    reserve_id: String,
    reserve_binding: String,
    reserve_initial_units: u64,
    viability_evidence_content_digest: String,
    support_closure_continuity_content_digest: String,
    spend_sequence: u64,
    available_units: u64,
    evidence_digest: String,
    pending_sibling_fork_digest: Option<String>,
}

impl RegenerativeRecoveryReserveHeadV1 {
    pub fn from_first(first: &RegenerativeRecoveryCoordinateEvidenceV1) -> Result<Self, String> {
        verify_regenerative_recovery_reserve_lineage(None, first)?;
        if first.reserve_spend_sequence_before != 0 {
            return Err("recovery reserve head root must be the first spend".into());
        }
        Ok(Self {
            reserve_id: first.external_recovery_reserve_id.clone(),
            reserve_binding: first.external_recovery_reserve_binding.clone(),
            reserve_initial_units: first.external_recovery_reserve_initial_units,
            viability_evidence_content_digest: first.viability_evidence_content_digest.clone(),
            support_closure_continuity_content_digest: first
                .support_closure_continuity_content_digest
                .clone(),
            spend_sequence: first.reserve_spend_sequence_after,
            available_units: first.reserve_units_after,
            evidence_digest: first.content_digest()?,
            pending_sibling_fork_digest: None,
        })
    }

    pub fn reserve_id(&self) -> &str {
        &self.reserve_id
    }

    pub fn reserve_binding(&self) -> &str {
        &self.reserve_binding
    }

    pub const fn reserve_initial_units(&self) -> u64 {
        self.reserve_initial_units
    }

    pub fn viability_evidence_content_digest(&self) -> &str {
        &self.viability_evidence_content_digest
    }

    pub fn support_closure_continuity_content_digest(&self) -> &str {
        &self.support_closure_continuity_content_digest
    }

    pub const fn spend_sequence(&self) -> u64 {
        self.spend_sequence
    }

    pub const fn available_units(&self) -> u64 {
        self.available_units
    }

    pub fn evidence_digest(&self) -> &str {
        &self.evidence_digest
    }

    pub const fn fork_pending(&self) -> bool {
        self.pending_sibling_fork_digest.is_some()
    }

    pub fn pending_sibling_fork_digest(&self) -> Option<&str> {
        self.pending_sibling_fork_digest.as_deref()
    }

    pub fn classify(
        &self,
        candidate: &RegenerativeRecoveryCoordinateEvidenceV1,
    ) -> Result<RegenerativeRecoveryReserveDispositionV1, String> {
        candidate.validate()?;

        if candidate.external_recovery_reserve_id != self.reserve_id
            || candidate.external_recovery_reserve_binding != self.reserve_binding
            || candidate.external_recovery_reserve_initial_units != self.reserve_initial_units
        {
            return Ok(RegenerativeRecoveryReserveDispositionV1::WrongReserve);
        }
        if candidate.viability_evidence_content_digest != self.viability_evidence_content_digest
            || candidate.support_closure_continuity_content_digest
                != self.support_closure_continuity_content_digest
        {
            return Ok(RegenerativeRecoveryReserveDispositionV1::WrongNominalSubject);
        }

        let candidate_digest = candidate.content_digest()?;
        if let Some(pending) = &self.pending_sibling_fork_digest {
            if candidate_digest == self.evidence_digest {
                return Ok(RegenerativeRecoveryReserveDispositionV1::Duplicate);
            }
            if candidate_digest == *pending {
                return Ok(RegenerativeRecoveryReserveDispositionV1::Fork);
            }
            return Ok(RegenerativeRecoveryReserveDispositionV1::FrozenPendingResolution);
        }

        if candidate.reserve_spend_sequence_after == self.spend_sequence {
            return Ok(if candidate_digest == self.evidence_digest {
                RegenerativeRecoveryReserveDispositionV1::Duplicate
            } else {
                RegenerativeRecoveryReserveDispositionV1::Fork
            });
        }
        if candidate.reserve_spend_sequence_after < self.spend_sequence {
            return Ok(RegenerativeRecoveryReserveDispositionV1::StaleReplay);
        }

        let Some(expected_after) = self.spend_sequence.checked_add(1) else {
            return Ok(RegenerativeRecoveryReserveDispositionV1::Gap);
        };
        if candidate.reserve_spend_sequence_after > expected_after {
            return Ok(RegenerativeRecoveryReserveDispositionV1::Gap);
        }

        if candidate.reserve_spend_sequence_before != self.spend_sequence
            || candidate.reserve_units_before != self.available_units
            || candidate.previous_recovery_evidence_content_digest.as_deref()
                != Some(self.evidence_digest.as_str())
        {
            return Ok(RegenerativeRecoveryReserveDispositionV1::Fork);
        }

        Ok(RegenerativeRecoveryReserveDispositionV1::Advance)
    }

    pub fn ingest(
        &mut self,
        candidate: &RegenerativeRecoveryCoordinateEvidenceV1,
    ) -> Result<RegenerativeRecoveryReserveDispositionV1, String> {
        let disposition = self.classify(candidate)?;
        match disposition {
            RegenerativeRecoveryReserveDispositionV1::Advance => {
                self.spend_sequence = candidate.reserve_spend_sequence_after;
                self.available_units = candidate.reserve_units_after;
                self.evidence_digest = candidate.content_digest()?;
            }
            RegenerativeRecoveryReserveDispositionV1::Fork
                if self.pending_sibling_fork_digest.is_none()
                    && candidate.reserve_spend_sequence_after == self.spend_sequence =>
            {
                self.pending_sibling_fork_digest = Some(candidate.content_digest()?);
            }
            _ => {}
        }
        Ok(disposition)
    }

    pub(crate) fn resolve_pending_sibling_fork_to(
        &mut self,
        selected: &RegenerativeRecoveryCoordinateEvidenceV1,
    ) -> Result<(), String> {
        let pending = self
            .pending_sibling_fork_digest
            .as_deref()
            .ok_or_else(|| "recovery reserve cursor has no pending sibling fork".to_string())?;
        selected.validate()?;
        if selected.external_recovery_reserve_id != self.reserve_id
            || selected.external_recovery_reserve_binding != self.reserve_binding
            || selected.external_recovery_reserve_initial_units != self.reserve_initial_units
            || selected.viability_evidence_content_digest != self.viability_evidence_content_digest
            || selected.support_closure_continuity_content_digest
                != self.support_closure_continuity_content_digest
            || selected.reserve_spend_sequence_after != self.spend_sequence
        {
            return Err("selected fork branch does not match the frozen reserve subject".into());
        }

        let selected_digest = selected.content_digest()?;
        if selected_digest != self.evidence_digest && selected_digest != pending {
            return Err("selected recovery evidence is not one of the observed fork branches".into());
        }

        self.available_units = selected.reserve_units_after;
        self.evidence_digest = selected_digest;
        self.pending_sibling_fork_digest = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryFlowKindEvidenceV1,
        REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1,
    };

    fn record(
        id: &str,
        before_sequence: u64,
        before_units: u64,
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
            reserve_units_per_recovery: 1,
            reserve_units_before: before_units,
            reserve_units_after: before_units - 1,
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

    #[test]
    fn sibling_fork_freezes_cursor_until_explicit_resolution() {
        let first = record("spend-1", 0, 3, None);
        let mut head = RegenerativeRecoveryReserveHeadV1::from_first(&first).unwrap();

        let first_digest = first.content_digest().unwrap();
        let child_a = record("spend-2a", 1, 2, Some(first_digest.clone()));
        let child_b = record("spend-2b", 1, 2, Some(first_digest));

        assert_eq!(
            head.ingest(&child_a).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Advance
        );
        assert_eq!(
            head.ingest(&child_b).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Fork
        );
        assert!(head.fork_pending());
        assert_eq!(
            head.classify(&record(
                "spend-3",
                2,
                1,
                Some(child_a.content_digest().unwrap()),
            ))
            .unwrap(),
            RegenerativeRecoveryReserveDispositionV1::FrozenPendingResolution
        );
    }

    #[test]
    fn gaps_wrong_reserve_and_wrong_nominal_subject_do_not_advance() {
        let first = record("spend-1", 0, 3, None);
        let head = RegenerativeRecoveryReserveHeadV1::from_first(&first).unwrap();
        let first_digest = first.content_digest().unwrap();

        let mut gap = record("spend-gap", 2, 1, Some(first_digest.clone()));
        gap.reserve_spend_sequence_after = 3;
        assert_eq!(
            head.classify(&gap).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::Gap
        );

        let mut wrong_reserve =
            record("spend-wrong-reserve", 1, 2, Some(first_digest.clone()));
        wrong_reserve.external_recovery_reserve_id = "reserve-other".into();
        assert_eq!(
            head.classify(&wrong_reserve).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::WrongReserve
        );

        let mut wrong_subject = record("spend-wrong-subject", 1, 2, Some(first_digest));
        wrong_subject.viability_evidence_content_digest = "33".repeat(32);
        assert_eq!(
            head.classify(&wrong_subject).unwrap(),
            RegenerativeRecoveryReserveDispositionV1::WrongNominalSubject
        );
    }
}
