// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Evidence-bound economic scenario identity.
//!
//! A scenario is a conditional model object: it binds a policy profile,
//! observation snapshot, model/engine identities, assumptions, and a time
//! horizon. It is not a forecast, authorization, or policy instruction.
//!
//! The intent is to make competing counterfactuals comparable without allowing
//! one model to become the economic authority.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Stable class of economic scenario being evaluated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EconomicScenarioKind {
    /// Reference state against which alternatives may be compared.
    Baseline,
    /// Explicit departure from a named baseline state.
    Counterfactual,
    /// Adversarial or adverse-condition stress case.
    StressTest,
    /// Parameter/model sensitivity case.
    Sensitivity,
    /// Retrospective comparison of prior model output with observed outcomes.
    Backtest,
}

/// Exact identity of a scenario referenced by an advisory analysis.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicScenarioBinding {
    /// Stable scenario identifier.
    pub scenario_ref: String,
    /// Exact content identity of the scenario definition.
    pub scenario_fingerprint: String,
}

impl EconomicScenarioBinding {
    /// Validate the binding structure.
    pub fn validate(&self) -> Result<(), String> {
        if self.scenario_ref.trim().is_empty() {
            return Err("Economic scenario reference cannot be empty".into());
        }
        if !is_sha256_hex(&self.scenario_fingerprint) {
            return Err(
                "Economic scenario fingerprint must be a 64-character hexadecimal SHA-256"
                    .into(),
            );
        }
        Ok(())
    }
}

/// Evidence-bound definition of a conditional economic scenario.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicPolicyScenario {
    /// Stable scenario identifier.
    pub scenario_id: String,
    /// Scenario class.
    pub kind: EconomicScenarioKind,
    /// Exact policy profile under which the scenario is interpreted.
    pub policy_profile_ref: String,
    /// Exact policy-profile content identity.
    pub policy_profile_fingerprint: String,
    /// Exact observation snapshot consumed by the scenario.
    pub observation_snapshot_fingerprint: String,
    /// Model/engine identities, including versions.
    pub model_refs: Vec<String>,
    /// Explicit assumptions that distinguish this scenario.
    pub assumption_refs: Vec<String>,
    /// Optional intervention/action definitions being simulated.
    pub intervention_refs: Vec<String>,
    /// Baseline scenario for comparative scenario kinds.
    pub baseline_scenario_ref: Option<String>,
    /// Evaluation horizon start.
    pub horizon_start: u64,
    /// Evaluation horizon end.
    pub horizon_end: u64,
    /// Explicit uncertainty/limitation references.
    pub uncertainty_refs: Vec<String>,
    /// Evidence references for retrospective validation/backtesting.
    pub backtest_refs: Vec<String>,
    /// Alternative scenario identities considered alongside this one.
    pub alternative_scenario_refs: Vec<String>,
    /// Scenario generation timestamp.
    pub generated_at: u64,
}

impl EconomicPolicyScenario {
    /// Validate the scenario definition.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("scenario ID", self.scenario_id.as_str()),
            ("policy profile reference", self.policy_profile_ref.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(format!("Economic scenario {name} cannot be empty"));
            }
        }

        for (name, fingerprint) in [
            ("policy profile", self.policy_profile_fingerprint.as_str()),
            (
                "observation snapshot",
                self.observation_snapshot_fingerprint.as_str(),
            ),
        ] {
            if !is_sha256_hex(fingerprint) {
                return Err(format!(
                    "Economic scenario {name} fingerprint must be a 64-character hexadecimal SHA-256"
                ));
            }
        }

        if self.model_refs.is_empty() {
            return Err("Economic scenario requires at least one model reference".into());
        }
        validate_unique_nonempty_refs("model", &self.model_refs)?;
        validate_unique_nonempty_refs("assumption", &self.assumption_refs)?;
        validate_unique_nonempty_refs("intervention", &self.intervention_refs)?;
        validate_unique_nonempty_refs("uncertainty", &self.uncertainty_refs)?;
        validate_unique_nonempty_refs("backtest", &self.backtest_refs)?;
        validate_unique_nonempty_refs("alternative scenario", &self.alternative_scenario_refs)?;

        if self.horizon_end < self.horizon_start {
            return Err("Economic scenario horizon-end cannot precede horizon-start".into());
        }

        match self.kind {
            EconomicScenarioKind::Baseline | EconomicScenarioKind::Backtest => {
                if self.baseline_scenario_ref.is_some() {
                    return Err(format!(
                        "{:?} scenario cannot reference a baseline scenario",
                        self.kind
                    ));
                }
            }
            EconomicScenarioKind::Counterfactual
            | EconomicScenarioKind::StressTest
            | EconomicScenarioKind::Sensitivity => {
                let baseline = self.baseline_scenario_ref.as_deref().ok_or_else(|| {
                    format!("{:?} scenario requires a baseline scenario reference", self.kind)
                })?;
                if baseline.trim().is_empty() {
                    return Err("Economic scenario baseline reference cannot be empty".into());
                }
                if baseline == self.scenario_id {
                    return Err("Economic scenario cannot baseline itself".into());
                }
            }
        }

        if self.kind == EconomicScenarioKind::Backtest && self.backtest_refs.is_empty() {
            return Err("Economic backtest scenario requires backtest evidence".into());
        }

        Ok(())
    }

    /// Return a deterministic content identity.
    ///
    /// Collections are sorted before hashing. The fingerprint is a content
    /// identifier, not a signature and not evidence that the scenario is true.
    pub fn fingerprint(&self) -> Result<String, String> {
        self.validate()?;

        let mut model_refs = self.model_refs.clone();
        let mut assumption_refs = self.assumption_refs.clone();
        let mut intervention_refs = self.intervention_refs.clone();
        let mut uncertainty_refs = self.uncertainty_refs.clone();
        let mut backtest_refs = self.backtest_refs.clone();
        let mut alternative_scenario_refs = self.alternative_scenario_refs.clone();

        for values in [
            &mut model_refs,
            &mut assumption_refs,
            &mut intervention_refs,
            &mut uncertainty_refs,
            &mut backtest_refs,
            &mut alternative_scenario_refs,
        ] {
            values.sort();
        }

        let payload = serde_json::json!({
            "version": 1,
            "scenario_id": self.scenario_id,
            "kind": self.kind,
            "policy_profile_ref": self.policy_profile_ref,
            "policy_profile_fingerprint": self.policy_profile_fingerprint,
            "observation_snapshot_fingerprint": self.observation_snapshot_fingerprint,
            "model_refs": model_refs,
            "assumption_refs": assumption_refs,
            "intervention_refs": intervention_refs,
            "baseline_scenario_ref": self.baseline_scenario_ref,
            "horizon_start": self.horizon_start,
            "horizon_end": self.horizon_end,
            "uncertainty_refs": uncertainty_refs,
            "backtest_refs": backtest_refs,
            "alternative_scenario_refs": alternative_scenario_refs,
            "generated_at": self.generated_at,
        });

        let canonical = serde_json::to_vec(&payload)
            .map_err(|error| format!("Economic scenario canonicalization failed: {error}"))?;

        let mut hasher = Sha256::new();
        hasher.update(b"MYCELIX-ECONOMIC-SCENARIO-V1\0");
        hasher.update(canonical);
        Ok(hex::encode(hasher.finalize()))
    }
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.as_bytes().iter().all(u8::is_ascii_hexdigit)
}

fn validate_unique_nonempty_refs(name: &str, refs: &[String]) -> Result<(), String> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        if reference.trim().is_empty() {
            return Err(format!("Economic scenario {name} references cannot be empty"));
        }
        if !seen.insert(reference) {
            return Err(format!(
                "Duplicate economic scenario {name} reference: {reference}"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn baseline() -> EconomicPolicyScenario {
        EconomicPolicyScenario {
            scenario_id: "scenario:baseline:1".into(),
            kind: EconomicScenarioKind::Baseline,
            policy_profile_ref: "profile:za:v1".into(),
            policy_profile_fingerprint: "a".repeat(64),
            observation_snapshot_fingerprint: "b".repeat(64),
            model_refs: vec!["model:symthaea:2026.10".into()],
            assumption_refs: vec!["assumption:steady-energy".into()],
            intervention_refs: Vec::new(),
            baseline_scenario_ref: None,
            horizon_start: 1_000,
            horizon_end: 2_000,
            uncertainty_refs: vec!["uncertainty:external-shock".into()],
            backtest_refs: Vec::new(),
            alternative_scenario_refs: vec!["scenario:stress:1".into()],
            generated_at: 1_000,
        }
    }

    #[test]
    fn validates_baseline_and_fingerprints() {
        let value = baseline();
        assert!(value.validate().is_ok());
        assert_eq!(value.fingerprint().unwrap().len(), 64);
    }

    #[test]
    fn reference_order_does_not_change_identity() {
        let left = baseline();
        let mut right = baseline();
        right.model_refs.reverse();
        right.assumption_refs.reverse();
        right.alternative_scenario_refs.reverse();
        assert_eq!(left.fingerprint().unwrap(), right.fingerprint().unwrap());
    }

    #[test]
    fn counterfactual_requires_baseline() {
        let mut value = baseline();
        value.kind = EconomicScenarioKind::Counterfactual;
        assert!(value.validate().is_err());

        value.baseline_scenario_ref = Some("scenario:baseline:1".into());
        assert!(value.validate().is_ok());
    }

    #[test]
    fn backtest_requires_evidence() {
        let mut value = baseline();
        value.kind = EconomicScenarioKind::Backtest;
        assert!(value.validate().is_err());

        value.backtest_refs.push("backtest:2025".into());
        assert!(value.validate().is_ok());
    }

    #[test]
    fn rejects_self_baseline() {
        let mut value = baseline();
        value.kind = EconomicScenarioKind::StressTest;
        value.baseline_scenario_ref = Some(value.scenario_id.clone());
        assert!(value.validate().is_err());
    }

    #[test]
    fn binding_requires_exact_fingerprint() {
        let valid = EconomicScenarioBinding {
            scenario_ref: "scenario:baseline:1".into(),
            scenario_fingerprint: "c".repeat(64),
        };
        assert!(valid.validate().is_ok());

        let mut invalid = valid;
        invalid.scenario_fingerprint = "not-a-fingerprint".into();
        assert!(invalid.validate().is_err());
    }
}
