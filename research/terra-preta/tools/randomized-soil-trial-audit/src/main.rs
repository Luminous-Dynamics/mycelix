//! Synthetic-only randomized-block soil intervention causal auditor.
//!
//! Computes an intention-to-treat difference in change and exact paired-block
//! randomization p-value. It does not verify a real allocation record or
//! estimate per-protocol / observational treatment effects.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic blocked-randomization calculation only. Real allocation, preregistration, permissions, interference, measurement validity and field implementation are not independently verified. ITT uses assignment, not treatment received. No per-protocol effect or universal transport claim is emitted.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    design: Design,
    units: Vec<TrialUnit>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Design {
    trial_id: String,
    design_type: String,
    randomization_unit_level: String,
    estimand: String,
    treatment_label: String,
    control_label: String,
    primary_outcome: String,
    registered_at: String,
    randomized_at: String,
    preregistration_sha256: String,
    allocation_sha256: String,
    allocation_algorithm: String,
    randomization_seed: u64,
    interference_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct TrialUnit {
    unit_id: String,
    site_id: String,
    block_id: String,
    assignment: String,
    assignment_probability: f64,
    baseline_date: String,
    followup_date: String,
    baseline_value: f64,
    followup_value: Option<f64>,
    outcome_unit: String,
    measurement_method: String,
    depth_min_m: f64,
    depth_max_m: f64,
    subsample_count: usize,
    received_treatment: bool,
    contamination_status: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    trial_id: String,
    design_type: &'static str,
    estimand: &'static str,
    primary_outcome: String,
    preregistration_sha256: String,
    allocation_sha256: String,
    allocation_algorithm: String,
    randomization_seed: u64,
    registered_at: String,
    randomized_at: String,
    input_sha256: String,
    randomized_unit_count: usize,
    block_count: usize,
    subsample_count_not_in_n: usize,
    treatment_assigned_count: usize,
    control_assigned_count: usize,
    treatment_nonadherence_count: usize,
    control_crossover_count: usize,
    block_contrasts: Vec<BlockContrast>,
    descriptive_assignment_contrast_mean: f64,
    causal_itt_estimate: Option<f64>,
    causal_itt_standard_error: Option<f64>,
    exact_two_sided_randomization_p_value: Option<f64>,
    randomization_assignments_enumerated: usize,
    causal_claim_status: &'static str,
    interference_status: String,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct BlockContrast {
    block_id: String,
    treatment_change: f64,
    control_change: f64,
    assignment_contrast: f64,
}

#[derive(Debug, Serialize)]
struct Controls {
    complete_block_randomization_pass: bool,
    registration_precedes_randomization_pass: bool,
    support_basis_match_pass: bool,
    all_randomized_units_retained_pass: bool,
    subsamples_not_counted_as_units_pass: bool,
    itt_uses_assignment_not_receipt_pass: bool,
    no_per_protocol_claim_pass: bool,
    interference_gate_pass: bool,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn valid_date(value: &str) -> bool {
    value.len() == 10
        && value.as_bytes()[4] == b'-'
        && value.as_bytes()[7] == b'-'
        && value
            .bytes()
            .enumerate()
            .all(|(i, b)| i == 4 || i == 7 || b.is_ascii_digit())
        && value[0..4]
            .parse::<u32>()
            .is_ok_and(|x| (1..=9999).contains(&x))
        && value[5..7]
            .parse::<u32>()
            .is_ok_and(|x| (1..=12).contains(&x))
        && value[8..10]
            .parse::<u32>()
            .is_ok_and(|x| (1..=31).contains(&x))
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "randomized-soil-trial-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic trial schema".into());
    }
    let d = &s.design;
    if s.scenario_id.trim().is_empty()
        || d.trial_id.trim().is_empty()
        || d.primary_outcome.trim().is_empty()
        || d.allocation_algorithm.trim().is_empty()
    {
        return Err("scenario, trial, outcome and allocation identity must be non-empty".into());
    }
    if d.design_type != "randomized_complete_block"
        || d.randomization_unit_level != "plot"
        || d.estimand != "intention_to_treat_difference_in_change"
    {
        return Err("v0.1 only qualifies plot-level randomized complete-block ITT designs".into());
    }
    if d.treatment_label.trim().is_empty()
        || d.control_label.trim().is_empty()
        || d.treatment_label == d.control_label
    {
        return Err("treatment and control labels must be distinct and non-empty".into());
    }
    if !valid_sha256(&d.preregistration_sha256) || !valid_sha256(&d.allocation_sha256) {
        return Err("preregistration and allocation digests must be lowercase SHA-256".into());
    }
    if d.registered_at.len() < 20
        || d.randomized_at.len() < 20
        || d.registered_at >= d.randomized_at
    {
        return Err("preregistration timestamp must precede randomization timestamp".into());
    }
    if !matches!(
        d.interference_status.as_str(),
        "none_reported" | "possible_spillover" | "unassessed"
    ) {
        return Err("unsupported interference status".into());
    }
    if s.units.len() < 4 || s.units.len() % 2 != 0 {
        return Err("complete-block trial requires at least four units and even unit count".into());
    }

    let mut unit_ids = BTreeSet::new();
    let mut blocks: BTreeMap<&str, Vec<&TrialUnit>> = BTreeMap::new();
    let mut reference_basis: Option<(&str, &str, f64, f64)> = None;
    for unit in &s.units {
        if unit.unit_id.trim().is_empty()
            || unit.site_id.trim().is_empty()
            || unit.block_id.trim().is_empty()
            || !unit_ids.insert(unit.unit_id.as_str())
        {
            return Err(
                "randomized unit IDs, sites and blocks must be non-empty; unit IDs unique".into(),
            );
        }
        finite("assignment_probability", unit.assignment_probability)?;
        finite("baseline_value", unit.baseline_value)?;
        if let Some(value) = unit.followup_value {
            finite("followup_value", value)?;
        } else {
            return Err(format!(
                "unit {} has a missing endpoint; causal qualification abstains",
                unit.unit_id
            ));
        }
        finite("depth_min_m", unit.depth_min_m)?;
        finite("depth_max_m", unit.depth_max_m)?;
        if unit.assignment_probability != 0.5 {
            return Err("v0.1 requires 0.5 assignment probability within every block".into());
        }
        if !matches!(unit.assignment.as_str(), "treatment" | "control") {
            return Err(format!("unit {} has unsupported assignment", unit.unit_id));
        }
        if !matches!(
            unit.contamination_status.as_str(),
            "none_reported" | "possible_spillover"
        ) {
            return Err(format!(
                "unit {} has unsupported contamination status",
                unit.unit_id
            ));
        }
        if !valid_date(&unit.baseline_date) || !valid_date(&unit.followup_date) {
            return Err(format!("unit {} has an invalid date", unit.unit_id));
        }
        let randomized_date = &d.randomized_at[0..10];
        if unit.baseline_date >= randomized_date || unit.followup_date <= randomized_date {
            return Err(format!(
                "unit {} baseline/randomization/follow-up chronology is invalid",
                unit.unit_id
            ));
        }
        if unit.outcome_unit.trim().is_empty() || unit.measurement_method.trim().is_empty() {
            return Err(format!(
                "unit {} has incomplete outcome measurement basis",
                unit.unit_id
            ));
        }
        if unit.depth_min_m < 0.0 || unit.depth_max_m <= unit.depth_min_m {
            return Err(format!("unit {} has invalid depth support", unit.unit_id));
        }
        if let Some((unit_name, method, min_depth, max_depth)) = reference_basis {
            if unit.outcome_unit != unit_name
                || unit.measurement_method != method
                || (unit.depth_min_m - min_depth).abs() > 1e-9
                || (unit.depth_max_m - max_depth).abs() > 1e-9
            {
                return Err("outcome unit, analytical method and depth support must match across trial units".into());
            }
        } else {
            reference_basis = Some((
                &unit.outcome_unit,
                &unit.measurement_method,
                unit.depth_min_m,
                unit.depth_max_m,
            ));
        }
        blocks.entry(unit.block_id.as_str()).or_default().push(unit);
    }
    if blocks.len() < 2 || blocks.len() > 20 {
        return Err("v0.1 exact randomization inference requires 2 to 20 complete blocks".into());
    }
    for (block_id, units) in &blocks {
        if units.len() != 2 {
            return Err(format!(
                "block {block_id} must contain exactly one treatment and one control plot"
            ));
        }
        let treatment_count = units.iter().filter(|u| u.assignment == "treatment").count();
        let control_count = units.iter().filter(|u| u.assignment == "control").count();
        if treatment_count != 1 || control_count != 1 {
            return Err(format!(
                "block {block_id} must contain one treatment and one control assignment"
            ));
        }
        if units[0].site_id != units[1].site_id {
            return Err(format!("block {block_id} crosses site boundaries"));
        }
    }
    Ok(())
}

fn calculate(s: &Scenario, input_digest: String) -> Report {
    let mut groups: BTreeMap<&str, Vec<&TrialUnit>> = BTreeMap::new();
    for unit in &s.units {
        groups.entry(unit.block_id.as_str()).or_default().push(unit);
    }

    let mut block_contrasts = Vec::with_capacity(groups.len());
    let mut treatment_nonadherence = 0usize;
    let mut control_crossover = 0usize;
    let mut subsamples = 0usize;
    for (block_id, units) in groups {
        let treatment = units
            .iter()
            .copied()
            .find(|u| u.assignment == "treatment")
            .unwrap();
        let control = units
            .iter()
            .copied()
            .find(|u| u.assignment == "control")
            .unwrap();
        let treatment_change = treatment.followup_value.unwrap() - treatment.baseline_value;
        let control_change = control.followup_value.unwrap() - control.baseline_value;
        if !treatment.received_treatment {
            treatment_nonadherence += 1;
        }
        if control.received_treatment {
            control_crossover += 1;
        }
        subsamples += treatment.subsample_count + control.subsample_count;
        block_contrasts.push(BlockContrast {
            block_id: block_id.to_string(),
            treatment_change,
            control_change,
            assignment_contrast: treatment_change - control_change,
        });
    }
    block_contrasts.sort_by(|a, b| a.block_id.cmp(&b.block_id));
    let values: Vec<f64> = block_contrasts
        .iter()
        .map(|b| b.assignment_contrast)
        .collect();
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance =
        values.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (values.len() - 1) as f64;
    let se = (variance / values.len() as f64).sqrt();

    let interference_clear = s.design.interference_status == "none_reported"
        && s.units
            .iter()
            .all(|u| u.contamination_status == "none_reported");
    let (itt, itt_se, p_value, assignments, status) = if interference_clear {
        let n = values.len();
        let total = 1usize << n;
        let mut extreme = 0usize;
        for mask in 0..total {
            let permuted_sum: f64 = values
                .iter()
                .enumerate()
                .map(|(i, value)| {
                    if ((mask >> i) & 1) == 0 {
                        *value
                    } else {
                        -*value
                    }
                })
                .sum();
            let statistic = permuted_sum / n as f64;
            if statistic.abs() + 1e-12 >= mean.abs() {
                extreme += 1;
            }
        }
        (
            Some(mean),
            Some(se),
            Some(extreme as f64 / total as f64),
            total,
            "causal_itt_qualified_synthetic",
        )
    } else if s.design.interference_status == "unassessed" {
        (
            None,
            None,
            None,
            0,
            "causal_claim_abstain_unassessed_interference",
        )
    } else {
        (None, None, None, 0, "causal_claim_abstain_interference")
    };

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        trial_id: s.design.trial_id.clone(),
        design_type: "randomized_complete_block",
        estimand: "intention_to_treat_difference_in_change",
        primary_outcome: s.design.primary_outcome.clone(),
        preregistration_sha256: s.design.preregistration_sha256.clone(),
        allocation_sha256: s.design.allocation_sha256.clone(),
        allocation_algorithm: s.design.allocation_algorithm.clone(),
        randomization_seed: s.design.randomization_seed,
        registered_at: s.design.registered_at.clone(),
        randomized_at: s.design.randomized_at.clone(),
        input_sha256: input_digest,
        randomized_unit_count: s.units.len(),
        block_count: block_contrasts.len(),
        subsample_count_not_in_n: subsamples,
        treatment_assigned_count: s
            .units
            .iter()
            .filter(|u| u.assignment == "treatment")
            .count(),
        control_assigned_count: s.units.iter().filter(|u| u.assignment == "control").count(),
        treatment_nonadherence_count: treatment_nonadherence,
        control_crossover_count: control_crossover,
        block_contrasts,
        descriptive_assignment_contrast_mean: mean,
        causal_itt_estimate: itt,
        causal_itt_standard_error: itt_se,
        exact_two_sided_randomization_p_value: p_value,
        randomization_assignments_enumerated: assignments,
        causal_claim_status: status,
        interference_status: s.design.interference_status.clone(),
        controls: Controls {
            complete_block_randomization_pass: true,
            registration_precedes_randomization_pass: true,
            support_basis_match_pass: true,
            all_randomized_units_retained_pass: true,
            subsamples_not_counted_as_units_pass: true,
            itt_uses_assignment_not_receipt_pass: true,
            no_per_protocol_claim_pass: true,
            interference_gate_pass: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output already exists").into());
    }
    let bytes = fs::read(input)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = calculate(&scenario, digest);
    let output_bytes = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&output_bytes)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: randomized-soil-trial-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("randomized soil trial audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-randomized-trial.json"))
            .expect("fixture parses")
    }

    #[test]
    fn exact_blocked_itt_and_randomization_test_match_known_truth() {
        let s = scenario();
        validate(&s).expect("valid trial");
        let report = calculate(&s, "0".repeat(64));
        assert_eq!(report.randomized_unit_count, 8);
        assert_eq!(report.block_count, 4);
        assert_eq!(report.subsample_count_not_in_n, 46);
        assert!((report.causal_itt_estimate.unwrap() - 2.5).abs() < 1e-12);
        assert!((report.exact_two_sided_randomization_p_value.unwrap() - 0.125).abs() < 1e-12);
        assert_eq!(report.randomization_assignments_enumerated, 16);
    }

    #[test]
    fn nonadherence_does_not_change_itt_assignment() {
        let mut s = scenario();
        s.units[0].received_treatment = false;
        let a = calculate(&s, "0".repeat(64));
        assert_eq!(a.causal_itt_estimate, Some(2.5));
        assert_eq!(a.treatment_nonadherence_count, 1);
        assert_eq!(a.causal_claim_status, "causal_itt_qualified_synthetic");
    }

    #[test]
    fn possible_interference_abstains_causal_claim() {
        let mut s = scenario();
        s.design.interference_status = "possible_spillover".into();
        let r = calculate(&s, "0".repeat(64));
        assert_eq!(r.causal_claim_status, "causal_claim_abstain_interference");
        assert!(r.causal_itt_estimate.is_none());
        assert_eq!(r.descriptive_assignment_contrast_mean, 2.5);
    }

    #[test]
    fn missing_followup_is_rejected_not_silently_dropped() {
        let mut s = scenario();
        s.units[0].followup_value = None;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn future_preregistration_is_rejected() {
        let mut s = scenario();
        s.design.registered_at = "2026-02-01T00:00:00Z".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn duplicate_unit_id_is_rejected() {
        let mut s = scenario();
        s.units[1].unit_id = s.units[0].unit_id.clone();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn two_units_assigned_to_same_arm_in_block_is_rejected() {
        let mut s = scenario();
        s.units[1].assignment = "treatment".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn method_depth_and_unit_mismatch_is_rejected() {
        let mut s = scenario();
        s.units[0].measurement_method = "different assay".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.units[0].depth_max_m = 0.3;
        assert!(validate(&t).is_err());

        let mut u = scenario();
        u.units[0].outcome_unit = "g/kg".into();
        assert!(validate(&u).is_err());
    }

    #[test]
    fn assignment_probability_must_be_exactly_one_half() {
        let mut s = scenario();
        s.units[0].assignment_probability = 0.7;
        assert!(validate(&s).is_err());
    }
}
