// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Causally honest evaluation of economic analyses against observed outcomes.
//!
//! The evaluator separates the information boundary, intervention influence,
//! and measurement completeness so policy-induced outcomes are not silently
//! scored as passive forecast errors.

use super::{
    policy_analysis::{EconomicAnalysisBinding, EconomicPolicyAnalysis},
    policy_observation::EconomicObservationSnapshot,
    scenario::EconomicScenarioBinding,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicEvaluationKind {
    ObservationalBacktest,
    ScenarioOutcome,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicOutcomeInfluence {
    NoKnownIntervention,
    InterventionAffected,
    InfluenceUnknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicMeasurementStatus {
    Complete,
    PartiallyObserved,
    Invalidated,
}

/// A forecast target with its native reporting unit.
///
/// Units are descriptive evidence, not a conversion or valuation rule.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicOutcomeTarget {
    pub target_ref: String,
    pub native_unit: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicOutcomeEvaluation {
    pub evaluation_id: String,
    pub kind: EconomicEvaluationKind,
    pub analysis: EconomicAnalysisBinding,
    pub scenario: Option<EconomicScenarioBinding>,
    pub targets: Vec<EconomicOutcomeTarget>,
    pub outcome_snapshot_fingerprint: String,
    /// Latest information timestamp permitted to contribute to the evaluated claim.
    pub information_cutoff_at: u64,
    /// Start and end of the forecast/scenario evaluation horizon.
    pub horizon_start_at: u64,
    pub horizon_end_at: u64,
    pub outcome_captured_at: u64,
    pub evaluation_at: u64,
    pub outcome_influence: EconomicOutcomeInfluence,
    pub measurement_status: EconomicMeasurementStatus,
    pub intervention_refs: Vec<String>,
    pub governance_decision_refs: Vec<String>,
    pub uncertainty_refs: Vec<String>,
    pub missing_data_refs: Vec<String>,
}

impl EconomicOutcomeEvaluation {
    pub fn validate(&self) -> Result<(), String> {
        if self.evaluation_id.trim().is_empty() {
            return Err("Economic outcome evaluation ID cannot be empty".into());
        }
        self.analysis.validate()?;
        if let Some(scenario) = &self.scenario { scenario.validate()?; }
        for (name, value) in [("outcome snapshot", self.outcome_snapshot_fingerprint.as_str())] {
            if !is_sha256_hex(value) {
                return Err(format!("Economic outcome evaluation {name} fingerprint must be a 64-character hexadecimal SHA-256"));
            }
        }
        if self.targets.is_empty() {
            return Err("Economic outcome evaluation requires at least one target".into());
        }
        let mut target_refs = BTreeSet::new();
        for target in &self.targets {
            if target.target_ref.trim().is_empty() {
                return Err("Economic outcome evaluation target references cannot be empty".into());
            }
            if target.native_unit.trim().is_empty() {
                return Err(format!(
                    "Economic outcome evaluation native unit cannot be empty for target: {}",
                    target.target_ref
                ));
            }
            if !target_refs.insert(&target.target_ref) {
                return Err(format!(
                    "Duplicate economic outcome evaluation target reference: {}",
                    target.target_ref
                ));
            }
        }
        validate_unique_nonempty_refs("intervention", &self.intervention_refs, false)?;
        validate_unique_nonempty_refs("governance decision", &self.governance_decision_refs, false)?;
        validate_unique_nonempty_refs("uncertainty", &self.uncertainty_refs, false)?;
        validate_unique_nonempty_refs("missing-data", &self.missing_data_refs, false)?;

        if self.evaluation_at < self.outcome_captured_at {
            return Err("Economic outcome evaluation time cannot precede outcome capture time".into());
        }
        if self.horizon_end_at < self.horizon_start_at {
            return Err("Economic outcome evaluation horizon-end cannot precede horizon-start".into());
        }
        if self.information_cutoff_at > self.horizon_start_at {
            return Err(
                "Economic outcome evaluation information cutoff cannot follow horizon start"
                    .into(),
            );
        }
        if self.outcome_captured_at < self.horizon_start_at {
            return Err(
                "Economic outcome evaluation outcome capture cannot precede horizon start".into(),
            );
        }

        if self.outcome_influence == EconomicOutcomeInfluence::NoKnownIntervention
            && (!self.intervention_refs.is_empty() || !self.governance_decision_refs.is_empty())
        {
            return Err("NoKnownIntervention cannot carry intervention or governance decision references".into());
        }
        if self.outcome_influence == EconomicOutcomeInfluence::InterventionAffected
            && self.intervention_refs.is_empty()
            && self.governance_decision_refs.is_empty()
        {
            return Err("InterventionAffected requires intervention or governance decision evidence".into());
        }
        if self.kind == EconomicEvaluationKind::ScenarioOutcome && self.scenario.is_none() {
            return Err("ScenarioOutcome requires an exact scenario binding".into());
        }
        if self.kind == EconomicEvaluationKind::ObservationalBacktest {
            if !self.intervention_refs.is_empty() || !self.governance_decision_refs.is_empty() {
                return Err("ObservationalBacktest cannot contain intervention or governance decision references".into());
            }
            if self.outcome_influence != EconomicOutcomeInfluence::NoKnownIntervention {
                return Err("ObservationalBacktest requires NoKnownIntervention influence".into());
            }
        }
        if self.outcome_influence == EconomicOutcomeInfluence::InfluenceUnknown
            && self.uncertainty_refs.is_empty()
            && self.missing_data_refs.is_empty()
        {
            return Err(
                "InfluenceUnknown must carry uncertainty or missing-data evidence".into(),
            );
        }
        match self.measurement_status {
            EconomicMeasurementStatus::Complete => {
                if !self.missing_data_refs.is_empty() {
                    return Err(
                        "Complete measurement cannot carry missing-data references".into(),
                    );
                }
                if self.outcome_captured_at < self.horizon_end_at {
                    return Err(
                        "Complete measurement requires outcome capture at or after horizon end"
                            .into(),
                    );
                }
            }
            EconomicMeasurementStatus::PartiallyObserved => {
                if self.missing_data_refs.is_empty() {
                    return Err(
                        "PartiallyObserved measurement requires missing-data evidence".into(),
                    );
                }
            }
            EconomicMeasurementStatus::Invalidated => {
                if self.uncertainty_refs.is_empty() && self.missing_data_refs.is_empty() {
                    return Err(
                        "Invalidated measurement requires uncertainty or missing-data evidence"
                            .into(),
                    );
                }
            }
        }
        Ok(())
    }

    pub fn validate_against_outcome_snapshot(
        &self,
        snapshot: &EconomicObservationSnapshot,
    ) -> Result<(), String> {
        self.validate()?;

        let fingerprint = snapshot.fingerprint()?;
        if fingerprint != self.outcome_snapshot_fingerprint {
            return Err("Economic outcome evaluation snapshot fingerprint does not match supplied snapshot".into());
        }
        if snapshot.captured_at != self.outcome_captured_at {
            return Err(
                "Economic outcome evaluation capture time does not match supplied snapshot"
                    .into(),
            );
        }
        if snapshot.captured_at > self.evaluation_at {
            return Err("Economic outcome snapshot occurs after evaluation time".into());
        }

        Ok(())
    }

    pub fn validate_against_analysis(&self, analysis: &EconomicPolicyAnalysis) -> Result<(), String> {
        self.validate()?;
        let fingerprint = analysis.fingerprint()?;
        if self.analysis.analysis_ref != analysis.analysis_id {
            return Err("Economic outcome evaluation analysis reference does not match supplied analysis".into());
        }
        if self.analysis.analysis_fingerprint != fingerprint {
            return Err("Economic outcome evaluation analysis fingerprint does not match supplied analysis".into());
        }
        if self.information_cutoff_at > analysis.generated_at {
            return Err("Economic outcome evaluation information cutoff follows analysis generation time".into());
        }
        if analysis.generated_at > self.outcome_captured_at {
            return Err("Economic analysis was generated after the evaluated outcome was captured".into());
        }
        if analysis.generated_at > self.horizon_start_at {
            return Err(
                "Economic analysis was generated after the evaluation horizon started".into(),
            );
        }
        match (&self.scenario, &analysis.scenario) {
            (Some(evaluation), Some(analysis_scenario)) if evaluation == analysis_scenario => {}
            (Some(_), Some(_)) => return Err("Economic outcome evaluation scenario does not match analysis scenario".into()),
            (Some(_), None) => return Err("Economic outcome evaluation references a scenario absent from the analysis".into()),
            (None, Some(_)) if self.kind == EconomicEvaluationKind::ScenarioOutcome => {
                return Err("ScenarioOutcome must preserve the analysis scenario binding".into());
            }
            _ => {}
        }
        Ok(())
    }

    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;
        let mut targets = self.targets.clone();
        let mut intervention_refs = self.intervention_refs.clone();
        let mut governance_decision_refs = self.governance_decision_refs.clone();
        let mut uncertainty_refs = self.uncertainty_refs.clone();
        let mut missing_data_refs = self.missing_data_refs.clone();
        targets.sort_by_key(|target| (target.target_ref.clone(), target.native_unit.clone()));
        intervention_refs.sort();
        governance_decision_refs.sort();
        uncertainty_refs.sort();
        missing_data_refs.sort();
        let payload = serde_json::json!({
            "version": 1,
            "evaluation_id": self.evaluation_id,
            "kind": self.kind,
            "analysis": self.analysis,
            "scenario": self.scenario,
            "targets": targets,
            "outcome_snapshot_fingerprint": self.outcome_snapshot_fingerprint,
            "information_cutoff_at": self.information_cutoff_at,
            "outcome_captured_at": self.outcome_captured_at,
            "evaluation_at": self.evaluation_at,
            "outcome_influence": self.outcome_influence,
            "measurement_status": self.measurement_status,
            "intervention_refs": intervention_refs,
            "governance_decision_refs": governance_decision_refs,
            "uncertainty_refs": uncertainty_refs,
            "missing_data_refs": missing_data_refs,
        });
        let canonical = serde_json::to_vec(&payload).map_err(|error| format!("Economic outcome evaluation canonicalization failed: {error}"))?;
        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-OUTCOME-EVALUATION-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.as_bytes().iter().all(u8::is_ascii_hexdigit)
}

fn validate_unique_nonempty_refs(name: &str, refs: &[String], require_nonempty: bool) -> Result<(), String> {
    if require_nonempty && refs.is_empty() {
        return Err(format!("Economic outcome evaluation requires at least one {name} reference"));
    }
    let mut seen = std::collections::BTreeSet::new();
    for reference in refs {
        if reference.trim().is_empty() {
            return Err(format!("Economic outcome evaluation {name} references cannot be empty"));
        }
        if !seen.insert(reference) {
            return Err(format!("Duplicate economic outcome evaluation {name} reference: {reference}"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::economics::policy_analysis::{EconomicAnalysisBinding, EconomicPolicyAnalysis};

    fn analysis() -> EconomicPolicyAnalysis {
        EconomicPolicyAnalysis {
            analysis_id: "analysis:history:1".into(),
            policy_profile_ref: "profile:za:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            model_ref: "model:test:v1".into(),
            model_provenance: None,
            observation_refs: vec!["observation:price".into()],
            observation_snapshot_fingerprint: "b".repeat(64),
            scenario: None,
            proposed_adjustment: None,
            confidence_bps: 7_500,
            uncertainty_refs: vec!["uncertainty:policy".into()],
            alternative_analysis_bindings: Vec::new(),
            rationale_refs: vec!["evidence:test".into()],
            generated_at: 1_000,
        }
    }

    fn evaluation() -> EconomicOutcomeEvaluation {
        let analysis = analysis();
        EconomicOutcomeEvaluation {
            evaluation_id: "evaluation:1".into(),
            kind: EconomicEvaluationKind::ObservationalBacktest,
            analysis: EconomicAnalysisBinding {
                analysis_ref: analysis.analysis_id.clone(),
                analysis_fingerprint: analysis.fingerprint().unwrap(),
            },
            scenario: None,
            target_refs: vec!["target:gdp-growth".into()],
            targets: vec![EconomicOutcomeTarget {
                target_ref: "target:gdp-growth".into(),
                native_unit: "percent-per-year".into(),
            }],
            outcome_snapshot_fingerprint: "c".repeat(64),
            information_cutoff_at: 900,
            horizon_start_at: 1_100,
            horizon_end_at: 1_400,
            outcome_captured_at: 1_500,
            evaluation_at: 1_600,
            outcome_influence: EconomicOutcomeInfluence::NoKnownIntervention,
            measurement_status: EconomicMeasurementStatus::Complete,
            intervention_refs: Vec::new(),
            governance_decision_refs: Vec::new(),
            uncertainty_refs: vec!["uncertainty:measurement".into()],
            missing_data_refs: Vec::new(),
        }
    }

    #[test] fn validates_observational_backtest() {
        let value=evaluation();
        assert!(value.validate().is_ok());
        assert!(value.validate_against_analysis(&analysis()).is_ok());

        let outcome = EconomicObservationSnapshot {
            snapshot_id: "snapshot:outcome".into(),
            observation_bindings: Vec::new(),
            captured_at: 1_500,
        };
        assert!(value.validate_against_outcome_snapshot(&outcome).is_err());

        assert_eq!(value.fingerprint().unwrap().len(), 64);
    }

    #[test] fn intervention_affected_requires_evidence() {
        let mut value=evaluation();
        value.outcome_influence=EconomicOutcomeInfluence::InterventionAffected;
        assert!(value.validate().is_err());
        value.intervention_refs.push("action:stimulus-1".into());
        assert!(value.validate().is_err());
        value.kind=EconomicEvaluationKind::ScenarioOutcome;
        value.scenario=Some(EconomicScenarioBinding { scenario_ref:"scenario:policy:1".into(), scenario_fingerprint:"d".repeat(64) });
        assert!(value.validate().is_err());
    }

    #[test]
    fn unknown_influence_requires_uncertainty_or_missing_data() {
        let mut value = evaluation();
        value.outcome_influence = EconomicOutcomeInfluence::InfluenceUnknown;
        value.uncertainty_refs.clear();
        assert!(value.validate().is_err());

        value.missing_data_refs.push("missing:1".into());
        assert!(value.validate().is_ok());
    }

    #[test]
    fn measurement_status_changes_identity() {
        let left = evaluation();
        let mut right = left.clone();
        right.measurement_status = EconomicMeasurementStatus::PartiallyObserved;
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());

        right.measurement_status = EconomicMeasurementStatus::Invalidated;
        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test] fn information_leakage_is_rejected() {
        let mut value=evaluation();
        value.information_cutoff_at=1_100;
        assert!(value.validate_against_analysis(&analysis()).is_err());
    }

    #[test] fn post_outcome_analysis_is_rejected() {
        let mut value=evaluation();
        value.outcome_captured_at=900;
        assert!(value.validate_against_analysis(&analysis()).is_err());
    }


    #[test]
    fn target_native_units_are_required_and_identity_bearing() {
        let mut value = evaluation();
        value.targets[0].native_unit = "index-points".into();
        assert_ne!(evaluation().fingerprint().unwrap(), value.fingerprint().unwrap());

        value.targets[0].native_unit.clear();
        assert!(value.validate().is_err());
    }

    #[test]
    fn duplicate_targets_are_rejected() {
        let mut value = evaluation();
        value.targets.push(value.targets[0].clone());
        assert!(value.validate().is_err());
    }

    #[test]
    fn horizon_and_information_boundary_are_fail_closed() {
        let mut value = evaluation();
        value.horizon_end_at = value.horizon_start_at - 1;
        assert!(value.validate().is_err());

        let mut value = evaluation();
        value.information_cutoff_at = value.horizon_start_at + 1;
        assert!(value.validate().is_err());

        let mut value = evaluation();
        value.horizon_start_at = value.outcome_captured_at + 1;
        assert!(value.validate().is_err());
    }

    #[test]
    fn complete_measurement_cannot_claim_missing_data_or_end_before_horizon() {
        let mut value = evaluation();
        value.missing_data_refs.push("missing:revision".into());
        assert!(value.validate().is_err());

        let mut value = evaluation();
        value.horizon_end_at = value.outcome_captured_at + 1;
        assert!(value.validate().is_err());
    }

    #[test]
    fn partial_and_invalidated_measurements_require_limitation_evidence() {
        let mut value = evaluation();
        value.measurement_status = EconomicMeasurementStatus::PartiallyObserved;
        assert!(value.validate().is_err());
        value.missing_data_refs.push("missing:final-quarter".into());
        assert!(value.validate().is_ok());

        let mut value = evaluation();
        value.measurement_status = EconomicMeasurementStatus::Invalidated;
        value.uncertainty_refs.clear();
        assert!(value.validate().is_err());
        value.uncertainty_refs.push("uncertainty:definition-change".into());
        assert!(value.validate().is_ok());
    }

    #[test]
    fn analysis_must_precede_horizon_start() {
        let mut value = evaluation();
        value.horizon_start_at = 900;
        assert!(value.validate_against_analysis(&analysis()).is_err());
    }

    #[test] fn fingerprint_changes_when_influence_context_changes() {
        let left=evaluation();
        let mut right=left.clone();
        right.uncertainty_refs.push("uncertainty:reflexivity".into());
        assert_ne!(left.fingerprint().unwrap(),right.fingerprint().unwrap());
    }
}
