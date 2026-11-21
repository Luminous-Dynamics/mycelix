//! Synthetic method-bridge auditor.
//!
//! Numerical agreement, rank concordance and decision agreement are distinct
//! questions. A rank correlation never licenses analytical substitution.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeSet, HashSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const UNCERTAINTY_STATUS: &str = "not_estimated_cluster_bootstrap_required_for_empirical_inference";
const BOUNDARY: &str = "Synthetic method-bridge audit only. Rank concordance does not establish analytical equivalence. Numerical agreement is emitted only for commensurate methods explicitly asserted equivalent. No commercial sensor certification, laboratory accreditation, field diagnosis or agronomic benefit is established.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Comparison {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    comparison_id: String,
    property_id: String,
    objectives: Vec<String>,
    pairing_basis: String,
    analytical_equivalence: String,
    method_a: Method,
    method_b: Method,
    decision_threshold: DecisionThreshold,
    minimum_pairs: usize,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Method {
    method_id: String,
    method_version: String,
    method_class: String,
    unit: String,
    measurement_basis: String,
    support_id: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DecisionThreshold {
    status: String,
    value: Option<f64>,
    unit: Option<String>,
    authority: Option<String>,
    approval_reference: Option<String>,
    approval_date: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    sample_id: String,
    independence_group_id: String,
    method_a_value: f64,
    method_b_value: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    comparison_id: String,
    property_id: String,
    objective_status: Vec<String>,
    method_a_id: String,
    method_b_id: String,
    pairing_basis: String,
    pair_count: usize,
    independence_group_count: usize,
    rank_status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    spearman_rho: Option<f64>,
    numerical_agreement_status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    numerical_agreement: Option<NumericalAgreement>,
    decision_agreement_status: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    decision_agreement: Option<DecisionAgreement>,
    uncertainty_status: &'static str,
    input_sha256: String,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct NumericalAgreement {
    mean_difference_b_minus_a: f64,
    mae: f64,
    rmse: f64,
    limits_of_agreement_lower: f64,
    limits_of_agreement_upper: f64,
}

#[derive(Debug, Serialize)]
struct DecisionAgreement {
    threshold_status: String,
    threshold_value: f64,
    threshold_unit: String,
    both_positive: usize,
    method_a_only_positive: usize,
    method_b_only_positive: usize,
    both_negative: usize,
    agreement_fraction: f64,
}

#[derive(Debug, Serialize)]
struct Controls {
    numeric_claim_requires_analytical_equivalence: bool,
    no_composite_method_score: bool,
    duplicate_sample_rejection: bool,
    independence_groups_preserved: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
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

fn validate_threshold(t: &DecisionThreshold) -> Result<(), String> {
    match t.status.as_str() {
        "not_set" => {
            if t.value.is_some()
                || t.unit.is_some()
                || t.authority.is_some()
                || t.approval_reference.is_some()
                || t.approval_date.is_some()
            {
                return Err("not_set threshold cannot carry value or approval metadata".into());
            }
        }
        "synthetic_fixture" => {
            let value = t
                .value
                .ok_or_else(|| "synthetic threshold needs a value".to_string())?;
            finite("decision threshold", value)?;
            if t.unit.as_deref().unwrap_or("").trim().is_empty() {
                return Err("synthetic threshold needs an explicit unit".into());
            }
        }
        "externally_approved" => {
            let value = t
                .value
                .ok_or_else(|| "approved threshold needs a value".to_string())?;
            finite("decision threshold", value)?;
            if t.unit.as_deref().unwrap_or("").trim().is_empty()
                || t.authority.as_deref().unwrap_or("").trim().is_empty()
                || t.approval_reference
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .is_empty()
                || !t.approval_date.as_deref().is_some_and(valid_date)
            {
                return Err(
                    "externally approved threshold lacks complete approval provenance".into(),
                );
            }
        }
        _ => return Err("unsupported threshold status".into()),
    }
    Ok(())
}

fn validate(c: &Comparison) -> Result<(), String> {
    if c.schema_version != "soil-method-bridge-v0.1"
        || c.fixture_status != FIXTURE_STATUS
        || c.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic schema/version".into());
    }
    if c.comparison_id.trim().is_empty()
        || c.property_id.trim().is_empty()
        || c.method_a.method_id.trim().is_empty()
        || c.method_b.method_id.trim().is_empty()
        || c.method_a.method_version.trim().is_empty()
        || c.method_b.method_version.trim().is_empty()
    {
        return Err("comparison, target property and method identities are required".into());
    }
    if c.method_a.method_id == c.method_b.method_id {
        return Err("method A and method B must be distinct methods".into());
    }
    if !matches!(
        c.pairing_basis.as_str(),
        "same_specimen" | "matched_plot" | "matched_site"
    ) {
        return Err("unsupported pairing basis".into());
    }
    if !matches!(
        c.analytical_equivalence.as_str(),
        "asserted_equivalent" | "non_equivalent" | "unassessed"
    ) {
        return Err("unsupported analytical equivalence status".into());
    }
    let mut objectives = BTreeSet::new();
    for objective in &c.objectives {
        if !matches!(
            objective.as_str(),
            "rank_concordance" | "numerical_agreement" | "decision_agreement"
        ) || !objectives.insert(objective.as_str())
        {
            return Err("objectives must be supported and unique".into());
        }
    }
    if objectives.is_empty() || c.minimum_pairs < 3 || c.observations.len() < 3 {
        return Err("at least one objective and three paired observations are required".into());
    }
    for method in [&c.method_a, &c.method_b] {
        if !matches!(
            method.method_class.as_str(),
            "laboratory" | "rapid_field" | "proximal_sensing" | "in_situ_sensor" | "derived_model"
        ) || method.unit.trim().is_empty()
            || method.measurement_basis.trim().is_empty()
            || method.support_id.trim().is_empty()
        {
            return Err("method class, unit, basis and support identity must be present".into());
        }
    }
    if c.analytical_equivalence == "asserted_equivalent"
        && (c.method_a.unit != c.method_b.unit
            || c.method_a.measurement_basis != c.method_b.measurement_basis
            || c.method_a.support_id != c.method_b.support_id)
    {
        return Err(
            "asserted analytical equivalence requires matching unit, basis and support".into(),
        );
    }
    if objectives.contains("decision_agreement") {
        if c.analytical_equivalence != "asserted_equivalent" {
            return Err(
                "decision agreement requires commensurate methods asserted analytically equivalent"
                    .into(),
            );
        }
        if c.decision_threshold.value.is_some()
            && c.decision_threshold.unit.as_deref() != Some(c.method_a.unit.as_str())
        {
            return Err("decision threshold unit must match the common method unit".into());
        }
    }
    validate_threshold(&c.decision_threshold)?;

    let mut samples = HashSet::new();
    for observation in &c.observations {
        if observation.sample_id.trim().is_empty()
            || observation.independence_group_id.trim().is_empty()
            || !samples.insert(observation.sample_id.as_str())
        {
            return Err("paired sample IDs must be present and unique".into());
        }
        finite("method A value", observation.method_a_value)?;
        finite("method B value", observation.method_b_value)?;
    }
    Ok(())
}

fn average_ranks(values: &[f64]) -> Vec<f64> {
    let mut order: Vec<usize> = (0..values.len()).collect();
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]));
    let mut ranks = vec![0.0; values.len()];
    let mut start = 0;
    while start < order.len() {
        let mut end = start + 1;
        while end < order.len() && values[order[end]] == values[order[start]] {
            end += 1;
        }
        let average_rank = ((start + 1 + end) as f64) / 2.0;
        for i in start..end {
            ranks[order[i]] = average_rank;
        }
        start = end;
    }
    ranks
}

fn pearson(a: &[f64], b: &[f64]) -> Option<f64> {
    if a.len() != b.len() || a.len() < 2 {
        return None;
    }
    let mean_a = a.iter().sum::<f64>() / a.len() as f64;
    let mean_b = b.iter().sum::<f64>() / b.len() as f64;
    let covariance = a
        .iter()
        .zip(b)
        .map(|(x, y)| (x - mean_a) * (y - mean_b))
        .sum::<f64>();
    let variance_a = a.iter().map(|x| (x - mean_a).powi(2)).sum::<f64>();
    let variance_b = b.iter().map(|y| (y - mean_b).powi(2)).sum::<f64>();
    let denominator = (variance_a * variance_b).sqrt();
    (denominator > f64::EPSILON).then_some(covariance / denominator)
}

fn spearman(c: &Comparison) -> Option<f64> {
    let a: Vec<f64> = c.observations.iter().map(|x| x.method_a_value).collect();
    let b: Vec<f64> = c.observations.iter().map(|x| x.method_b_value).collect();
    pearson(&average_ranks(&a), &average_ranks(&b))
}

fn numerical_agreement(c: &Comparison) -> NumericalAgreement {
    let n = c.observations.len() as f64;
    let differences: Vec<f64> = c
        .observations
        .iter()
        .map(|x| x.method_b_value - x.method_a_value)
        .collect();
    let bias = differences.iter().sum::<f64>() / n;
    let mae = differences.iter().map(|x| x.abs()).sum::<f64>() / n;
    let rmse = (differences.iter().map(|x| x.powi(2)).sum::<f64>() / n).sqrt();
    let sample_sd =
        (differences.iter().map(|x| (x - bias).powi(2)).sum::<f64>() / (n - 1.0)).sqrt();
    NumericalAgreement {
        mean_difference_b_minus_a: bias,
        mae,
        rmse,
        limits_of_agreement_lower: bias - 1.96 * sample_sd,
        limits_of_agreement_upper: bias + 1.96 * sample_sd,
    }
}

fn decision_agreement(c: &Comparison) -> DecisionAgreement {
    let threshold = c.decision_threshold.value.unwrap_or(0.0);
    let mut both_positive = 0;
    let mut method_a_only_positive = 0;
    let mut method_b_only_positive = 0;
    let mut both_negative = 0;
    for observation in &c.observations {
        let a_positive = observation.method_a_value >= threshold;
        let b_positive = observation.method_b_value >= threshold;
        match (a_positive, b_positive) {
            (true, true) => both_positive += 1,
            (true, false) => method_a_only_positive += 1,
            (false, true) => method_b_only_positive += 1,
            (false, false) => both_negative += 1,
        }
    }
    let agreement = (both_positive + both_negative) as f64 / c.observations.len() as f64;
    DecisionAgreement {
        threshold_status: c.decision_threshold.status.clone(),
        threshold_value: threshold,
        threshold_unit: c.decision_threshold.unit.clone().unwrap_or_default(),
        both_positive,
        method_a_only_positive,
        method_b_only_positive,
        both_negative,
        agreement_fraction: agreement,
    }
}

fn build_report(c: &Comparison, digest: String) -> Report {
    let pair_count = c.observations.len();
    let independence_group_count = c
        .observations
        .iter()
        .map(|o| o.independence_group_id.as_str())
        .collect::<BTreeSet<_>>()
        .len();
    let rank_requested = c.objectives.iter().any(|x| x == "rank_concordance");
    let numeric_requested = c.objectives.iter().any(|x| x == "numerical_agreement");
    let decision_requested = c.objectives.iter().any(|x| x == "decision_agreement");
    let rho = if rank_requested && pair_count >= c.minimum_pairs {
        spearman(c)
    } else {
        None
    };
    let rank_status = if !rank_requested {
        "not_requested"
    } else if pair_count < c.minimum_pairs {
        "insufficient_pairs_abstain"
    } else if rho.is_none() {
        "constant_ranks_abstain"
    } else {
        "qualified_rank_concordance"
    };
    let (numeric_status, numeric) = if !numeric_requested {
        ("not_requested", None)
    } else if c.analytical_equivalence != "asserted_equivalent" {
        ("not_eligible_non_equivalent", None)
    } else if pair_count < c.minimum_pairs {
        ("insufficient_pairs_abstain", None)
    } else {
        ("qualified_comparable_pair", Some(numerical_agreement(c)))
    };
    let (decision_status, decision) = if !decision_requested {
        ("not_requested", None)
    } else if c.decision_threshold.status == "not_set" {
        ("not_eligible_threshold_not_set", None)
    } else if pair_count < c.minimum_pairs {
        ("insufficient_pairs_abstain", None)
    } else {
        (
            "qualified_shared_threshold_decisions",
            Some(decision_agreement(c)),
        )
    };

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        comparison_id: c.comparison_id.clone(),
        property_id: c.property_id.clone(),
        objective_status: c.objectives.clone(),
        method_a_id: c.method_a.method_id.clone(),
        method_b_id: c.method_b.method_id.clone(),
        pairing_basis: c.pairing_basis.clone(),
        pair_count,
        independence_group_count,
        rank_status,
        spearman_rho: rho,
        numerical_agreement_status: numeric_status,
        numerical_agreement: numeric,
        decision_agreement_status: decision_status,
        decision_agreement: decision,
        uncertainty_status: UNCERTAINTY_STATUS,
        input_sha256: digest,
        controls: Controls {
            numeric_claim_requires_analytical_equivalence: true,
            no_composite_method_score: true,
            duplicate_sample_rejection: true,
            independence_groups_preserved: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "report already exists").into());
    }
    let bytes = fs::read(input)?;
    let comparison: Comparison = serde_json::from_slice(&bytes)?;
    validate(&comparison).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&comparison, digest);
    let serialized = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    file.write_all(&serialized)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: method-bridge-audit <comparison.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("method bridge audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn comparison() -> Comparison {
        serde_json::from_str(include_str!("../fixtures/synthetic-rank-only.json")).unwrap()
    }

    #[test]
    fn rank_only_pair_does_not_emit_numeric_agreement() {
        let c = comparison();
        validate(&c).unwrap();
        let report = build_report(&c, "0".repeat(64));
        assert_eq!(report.rank_status, "qualified_rank_concordance");
        assert!(report.spearman_rho.unwrap() > 0.7);
        assert_eq!(
            report.numerical_agreement_status,
            "not_eligible_non_equivalent"
        );
        assert!(report.numerical_agreement.is_none());
    }

    #[test]
    fn numerical_agreement_requires_matched_method_basis_and_support() {
        let mut c = comparison();
        c.analytical_equivalence = "asserted_equivalent".into();
        assert!(validate(&c).is_err());
        c.method_b.unit = c.method_a.unit.clone();
        c.method_b.measurement_basis = c.method_a.measurement_basis.clone();
        c.method_b.support_id = c.method_a.support_id.clone();
        validate(&c).unwrap();
        let report = build_report(&c, "0".repeat(64));
        assert_eq!(
            report.numerical_agreement_status,
            "qualified_comparable_pair"
        );
        assert!(report.numerical_agreement.is_some());
    }

    #[test]
    fn decisions_need_equivalence_and_shared_threshold() {
        let mut c = comparison();
        c.objectives = vec!["decision_agreement".into()];
        c.analytical_equivalence = "asserted_equivalent".into();
        c.method_b.unit = c.method_a.unit.clone();
        c.method_b.measurement_basis = c.method_a.measurement_basis.clone();
        c.method_b.support_id = c.method_a.support_id.clone();
        c.decision_threshold = DecisionThreshold {
            status: "synthetic_fixture".into(),
            value: Some(5.0),
            unit: Some(c.method_a.unit.clone()),
            authority: None,
            approval_reference: None,
            approval_date: None,
        };
        validate(&c).unwrap();
        let report = build_report(&c, "0".repeat(64));
        assert_eq!(
            report.decision_agreement_status,
            "qualified_shared_threshold_decisions"
        );
        assert!(report.decision_agreement.is_some());
    }

    #[test]
    fn duplicate_sample_ids_are_rejected() {
        let mut c = comparison();
        c.observations[1].sample_id = c.observations[0].sample_id.clone();
        assert!(validate(&c).is_err());
    }

    #[test]
    fn constant_ranks_abstain() {
        let mut c = comparison();
        for observation in &mut c.observations {
            observation.method_b_value = 2.0;
        }
        let report = build_report(&c, "0".repeat(64));
        assert_eq!(report.rank_status, "constant_ranks_abstain");
        assert!(report.spearman_rho.is_none());
    }

    #[test]
    fn independence_group_count_is_not_sample_count() {
        let c = comparison();
        let report = build_report(&c, "0".repeat(64));
        assert_eq!(report.pair_count, 8);
        assert_eq!(report.independence_group_count, 4);
    }
}
