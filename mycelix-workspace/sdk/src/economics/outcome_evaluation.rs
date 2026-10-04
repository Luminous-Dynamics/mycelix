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

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicOutcomeEvaluation {
    pub evaluation_id: String,
    pub kind: EconomicEvaluationKind,
    pub analysis: EconomicAnalysisBinding,
    pub scenario: Option<EconomicScenarioBinding>,
    pub target_refs: Vec<String>,
    pub outcome_snapshot_fingerprint: String,
    pub information_cutoff_at: u64,
    pub outcome_captured_at: u64,
    pub evaluation_at: u64,
    pub outcome_influence: EconomicOutcomeInfluence,
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
        validate_unique_nonempty_refs("target", &self.target_refs, true)?;
        validate_unique_nonempty_refs("intervention", &self.intervention_refs, false)?;
        validate_unique_nonempty_refs("governance decision", &self.governance_decision_refs, false)?;
        validate_unique_nonempty_refs("uncertainty", &self.uncertainty_refs, false)?;
        validate_unique_nonempty_refs("missing-data", &self.missing_data_refs, false)?;

        if self.evaluation_at < self.outcome_captured_at {
            return Err("Economic outcome evaluation time cannot precede outcome capture time".into());
        }
        if self.information_cutoff_at > self.outcome_captured_at {
            return Err("Economic outcome evaluation information cutoff cannot follow outcome capture time".into());
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
        if self.kind == EconomicEvaluationKind::ScenarioOutcome && self.missing_data_refs.is_empty()
            && self.outcome_influence == EconomicOutcomeInfluence::InfluenceUnknown
        {
            return Err("ScenarioOutcome with unknown influence must carry uncertainty or missing-data evidence".into());
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
        let mut target_refs = self.target_refs.clone();
        let mut intervention_refs = self.intervention_refs.clone();
        let mut governance_decision_refs = self.governance_decision_refs.clone();
        let mut uncertainty_refs = self.uncertainty_refs.clone();
        let mut missing_data_refs = self.missing_data_refs.clone();
        target_refs.sort();
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
            "target_refs": target_refs,
            "outcome_snapshot_fingerprint": self.outcome_snapshot_fingerprint,
            "information_cutoff_at": self.information_cutoff_at,
            "outcome_captured_at": self.outcome_captured_at,
            "evaluation_at": self.evaluation_at,
            "outcome_influence": self.outcome_influence,
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
            outcome_snapshot_fingerprint: "c".repeat(64),
            information_cutoff_at: 900,
            outcome_captured_at: 1_500,
            evaluation_at: 1_600,
            outcome_influence: EconomicOutcomeInfluence::NoKnownIntervention,
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

    #[test] fn fingerprint_changes_when_influence_context_changes() {
        let left=evaluation();
        let mut right=left.clone();
        right.uncertainty_refs.push("uncertainty:reflexivity".into());
        assert_ne!(left.fingerprint().unwrap(),right.fingerprint().unwrap());
    }
}
