// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Advisory economic cognition boundary.
//!
//! This module allows Symthaea or another reasoning engine to submit a
//! deterministic, evidence-bound policy analysis without acquiring authority
//! over economic state transitions.

use super::{metabolic_oracle::PolicyAdjustment, scenario::{EconomicPolicyScenario, EconomicScenarioBinding}};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Non-authoritative analysis produced by an economic reasoning engine.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EconomicPolicyAnalysis {
    /// Unique analysis identifier.
    pub analysis_id: String,
    /// Exact policy-profile context.
    pub policy_profile_ref: String,
    /// Exact profile content identity.
    pub policy_profile_fingerprint: String,
    /// Cognitive/reasoning engine identifier and version.
    pub model_ref: String,
    /// Observation IDs considered by the analysis.
    pub observation_refs: Vec<String>,
    /// Identity of the observation snapshot used by the model.
    pub observation_snapshot_fingerprint: String,
    /// Optional exact scenario/counterfactual identity.
    pub scenario: Option<EconomicScenarioBinding>,
    /// Proposed adjustment, if the analysis makes one.
    pub proposed_adjustment: Option<PolicyAdjustment>,
    /// Confidence in the analysis, expressed as basis points.
    pub confidence_bps: u16,
    /// References explaining uncertainty or important limitations.
    pub uncertainty_refs: Vec<String>,
    /// References to alternative analyses or models.
    pub alternative_analysis_refs: Vec<String>,
    /// Evidence/rationale references.
    pub rationale_refs: Vec<String>,
    /// Analysis generation timestamp.
    pub generated_at: u64,
}

impl EconomicPolicyAnalysis {
    /// Validate the advisory analysis envelope.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("analysis ID", self.analysis_id.as_str()),
            ("policy profile reference", self.policy_profile_ref.as_str()),
            ("model reference", self.model_ref.as_str()),
            (
                "observation snapshot fingerprint",
                self.observation_snapshot_fingerprint.as_str(),
            ),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Economic policy analysis {name} cannot be empty"));
            }
        }

        for (name, fingerprint) in [
            ("policy profile", self.policy_profile_fingerprint.as_str()),
            ("observation snapshot", self.observation_snapshot_fingerprint.as_str()),
        ] {
            if fingerprint.len() != 64
                || !fingerprint.as_bytes().iter().all(u8::is_ascii_hexdigit)
            {
                return Err(format!(
                    "Economic policy analysis {name} fingerprint must be a 64-character hexadecimal SHA-256"
                ));
            }
        }

        if self.observation_refs.is_empty() {
            return Err("Economic policy analysis requires observations".into());
        }

        let mut seen = BTreeSet::new();
        for (name, refs) in [
            ("observation", &self.observation_refs),
            ("uncertainty", &self.uncertainty_refs),
            ("alternative analysis", &self.alternative_analysis_refs),
            ("rationale", &self.rationale_refs),
        ] {
            for reference in refs {
                if reference.trim().is_empty() {
                    return Err(format!(
                        "Economic policy analysis {name} references cannot be empty"
                    ));
                }
            }
        }

        for reference in &self.observation_refs {
            if !seen.insert(reference) {
                return Err(format!(
                    "Duplicate economic policy analysis observation reference: {reference}"
                ));
            }
        }

        if self.confidence_bps > 10_000 {
            return Err("Economic policy analysis confidence exceeds 10,000 bps".into());
        }

        if let Some(scenario) = &self.scenario {
            scenario.validate()?;
        }

        if let Some(adjustment) = &self.proposed_adjustment {
            for (name, value) in [
                ("fee rate factor", adjustment.fee_rate_factor),
                ("demurrage rate factor", adjustment.demurrage_rate_factor),
                ("velocity incentive", adjustment.velocity_incentive),
            ] {
                if !value.is_finite() || value < 0.0 {
                    return Err(format!(
                        "Economic policy analysis {name} must be finite and non-negative"
                    ));
                }
            }
        }

        Ok(())
    }

    /// Validate the analysis against the exact scenario definition it references.\n    ///\n    /// This prevents a human-readable scenario reference from silently resolving\n    /// to different assumptions or model inputs after an analysis was generated.\n    pub fn validate_against_scenario(&self, scenario: &EconomicPolicyScenario) -> Result<(), String> {\n        self.validate()?;\n\n        let binding = self.scenario.as_ref().ok_or_else(|| {\n            "Economic policy analysis does not reference a scenario".to_string()\n        })?;\n        let scenario_fingerprint = scenario.fingerprint()?;\n        if binding.scenario_ref != scenario.scenario_id {\n            return Err("Economic policy analysis scenario reference does not match supplied scenario".into());\n        }\n        if binding.scenario_fingerprint != scenario_fingerprint {\n            return Err("Economic policy analysis scenario fingerprint does not match supplied scenario content".into());\n        }\n        if self.policy_profile_ref != scenario.policy_profile_ref\n            || self.policy_profile_fingerprint != scenario.policy_profile_fingerprint\n        {\n            return Err("Economic policy analysis policy profile does not match supplied scenario".into());\n        }\n        if self.observation_snapshot_fingerprint != scenario.observation_snapshot_fingerprint {\n            return Err("Economic policy analysis observation snapshot does not match supplied scenario".into());\n        }\n\n        Ok(())\n    }\n\n    /// Return a deterministic analysis content identity.
    ///
    /// This is a content identifier, not a proof that the referenced model,
    /// observations, or authority are truthful.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut observation_refs = self.observation_refs.clone();
        let mut uncertainty_refs = self.uncertainty_refs.clone();
        let mut alternative_analysis_refs = self.alternative_analysis_refs.clone();
        let mut rationale_refs = self.rationale_refs.clone();

        observation_refs.sort();
        uncertainty_refs.sort();
        alternative_analysis_refs.sort();
        rationale_refs.sort();

        let payload = serde_json::json!({
            "version": 1,
            "analysis_id": self.analysis_id,
            "policy_profile_ref": self.policy_profile_ref,
            "policy_profile_fingerprint": self.policy_profile_fingerprint,
            "model_ref": self.model_ref,
            "observation_refs": observation_refs,
            "observation_snapshot_fingerprint": self.observation_snapshot_fingerprint,
            "scenario": self.scenario,
            "proposed_adjustment": self.proposed_adjustment,
            "confidence_bps": self.confidence_bps,
            "uncertainty_refs": uncertainty_refs,
            "alternative_analysis_refs": alternative_analysis_refs,
            "rationale_refs": rationale_refs,
            "generated_at": self.generated_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic policy analysis canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-POLICY-ANALYSIS-V1 ");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn analysis() -> EconomicPolicyAnalysis {
        EconomicPolicyAnalysis {
            analysis_id: "analysis:1".into(),
            policy_profile_ref: "profile:za:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            model_ref: "symthaea:economic:2026-10".into(),
            observation_refs: vec!["observation:capacity", "observation:price"]
                .into_iter()
                .map(str::to_string)
                .collect(),
            observation_snapshot_fingerprint: "b".repeat(64),
            scenario: Some(EconomicScenarioBinding {\n                scenario_ref: "scenario:baseline:1".into(),\n                scenario_fingerprint: "c".repeat(64),\n            }),
            proposed_adjustment: None,
            confidence_bps: 8_500,
            uncertainty_refs: vec!["uncertainty:external-shock".into()],
            alternative_analysis_refs: vec!["analysis:alt-1".into()],
            rationale_refs: vec!["evidence:report-1".into()],
            generated_at: 1_000,
        }
    }

    #[test]
    fn validates_advisory_analysis() {
        assert!(analysis().validate().is_ok());
        assert_eq!(analysis().fingerprint().unwrap().len(), 64);
    }

    #[test]
    fn duplicate_observations_are_rejected() {
        let mut value = analysis();
        value.observation_refs.push("observation:price".into());
        assert!(value.validate().is_err());
    }

    #[test]
    fn confidence_is_bounded() {
        let mut value = analysis();
        value.confidence_bps = 10_001;
        assert!(value.validate().is_err());
    }

    #[test]
    fn advisory_analysis_can_recommend_governance_required_action() {
        let mut value = analysis();
        value.proposed_adjustment = Some(PolicyAdjustment {
            fee_rate_factor: 1.0,
            demurrage_rate_factor: 1.0,
            velocity_incentive: 1.0,
            tend_limit_tier: crate::economics::TendLimitTier::Normal,
            emergency_release: None,
            reason: "test".into(),
            requires_approval: true,
        });
        assert!(value.validate().is_ok());
    }

    #[test]
    fn exact_scenario_binding_matches_analysis_context() {
        let mut analysis = analysis();
        let mut scenario = EconomicPolicyScenario {
            scenario_id: "scenario:baseline:1".into(),
            kind: crate::economics::scenario::EconomicScenarioKind::Baseline,
            policy_profile_ref: analysis.policy_profile_ref.clone(),
            policy_profile_fingerprint: analysis.policy_profile_fingerprint.clone(),
            observation_snapshot_fingerprint: analysis.observation_snapshot_fingerprint.clone(),
            model_refs: vec![analysis.model_ref.clone()],
            assumption_refs: vec!["assumption:steady-state".into()],
            intervention_refs: Vec::new(),
            baseline_scenario_ref: None,
            horizon_start: 1_000,
            horizon_end: 2_000,
            uncertainty_refs: analysis.uncertainty_refs.clone(),
            backtest_refs: Vec::new(),
            alternative_scenario_refs: Vec::new(),
            generated_at: analysis.generated_at,
        };
        analysis.scenario.as_mut().unwrap().scenario_fingerprint = scenario.fingerprint().unwrap();

        assert!(analysis.validate_against_scenario(&scenario).is_ok());

        scenario.observation_snapshot_fingerprint = "d".repeat(64);
        assert!(analysis.validate_against_scenario(&scenario).is_err());
    }

    #[test]
    fn scenario_binding_is_part_of_analysis_identity() {\n        let left = analysis();\n        let mut right = analysis();\n        right.scenario.as_mut().unwrap().scenario_fingerprint = "d".repeat(64);\n        assert_ne!(left.fingerprint().unwrap(), right.fingerprint().unwrap());\n    }\n\n    #[test]\n    fn reference_order_does_not_change_identity() {
        let left = analysis();
        let mut right = analysis();
        right.observation_refs.reverse();
        right.uncertainty_refs.reverse();
        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }
}
