//! Synthetic-only context-conditioned soil-function benchmark.
//!
//! Separates absolute measurements, peer-reference position, longitudinal
//! within-unit change and explicit guardrails. It never emits a scalar health
//! score and never infers causal treatment effects from percentiles.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic-only context-conditioned indicator benchmark. Peer percentiles are relative ranks, empirical q90/q95/q99 are not physical maxima or causal targets, longitudinal endpoint change is descriptive, and no treatment effect or aggregate soil-health score is estimated.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    config: Config,
    indicators: Vec<Indicator>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    minimum_peer_samples: usize,
    minimum_independent_sites: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Indicator {
    indicator_id: String,
    property_name: String,
    unit: String,
    measurement_method: String,
    depth_min_m: f64,
    depth_max_m: f64,
    evaluation_independence_group_id: String,
    context: Context,
    reference_model: ReferenceModel,
    measurements: Vec<Measurement>,
    guardrail: Guardrail,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Context {
    texture_class: String,
    reference_soil_group: String,
    mean_annual_temperature_c: f64,
    mean_annual_precipitation_mm: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceModel {
    model_id: String,
    model_version: String,
    reference_dataset_id: String,
    reference_dataset_sha256: String,
    reference_model_sha256: String,
    reference_target_semantics: String,
    reference_measurement_method: String,
    reference_measurement_unit: String,
    reference_depth_min_m: f64,
    reference_depth_max_m: f64,
    peer_group_id: String,
    peer_sample_count: usize,
    independent_site_count: usize,
    domain_status: String,
    training_independence_group_ids: Vec<String>,
    predictive_mean: f64,
    predictive_sd: f64,
    predictive_mean_se: f64,
    q90: f64,
    q90_se: f64,
    q95: f64,
    q95_se: f64,
    q99: f64,
    q99_se: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Measurement {
    date: String,
    experimental_unit_id: String,
    value: f64,
    standard_error: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Guardrail {
    status: String,
    minimum_value: Option<f64>,
    maximum_value: Option<f64>,
    authority: Option<String>,
    approval_reference: Option<String>,
    approval_date: Option<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    indicator_count: usize,
    indicators: Vec<IndicatorResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct IndicatorResult {
    indicator_id: String,
    property_name: String,
    absolute_condition: AbsoluteCondition,
    peer_reference: PeerReference,
    longitudinal_change: LongitudinalChange,
    guardrail_result: GuardrailResult,
    causal_inference_status: &'static str,
}

#[derive(Debug, Serialize)]
struct AbsoluteCondition {
    value: f64,
    unit: String,
    measurement_method: String,
    depth_min_m: f64,
    depth_max_m: f64,
    latest_date: String,
    standard_error: f64,
}

#[derive(Debug, Serialize)]
struct PeerReference {
    status: &'static str,
    peer_group_id: String,
    reference_dataset_id: String,
    reference_dataset_sha256: String,
    reference_model_sha256: String,
    texture_class: String,
    reference_soil_group: String,
    mean_annual_temperature_c: f64,
    mean_annual_precipitation_mm: f64,
    peer_sample_count: usize,
    independent_site_count: usize,
    percentile: Option<f64>,
    percentile_interval_95: Option<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    distance_to_q90: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    distance_to_q95: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    distance_to_q99: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    distance_standard_errors: Option<QuantileErrors>,
    interpretation: &'static str,
}

#[derive(Debug, Serialize)]
struct QuantileErrors {
    q90: Option<f64>,
    q95: Option<f64>,
    q99: Option<f64>,
}

#[derive(Debug, Serialize)]
struct LongitudinalChange {
    experimental_unit_id: String,
    start_date: String,
    end_date: String,
    first_value: f64,
    last_value: f64,
    absolute_change: f64,
    change_standard_error: f64,
    unit: String,
    interpretation: &'static str,
}

#[derive(Debug, Serialize)]
struct GuardrailResult {
    status: &'static str,
    minimum_value: Option<f64>,
    maximum_value: Option<f64>,
    interpretation: &'static str,
}

#[derive(Debug, Serialize)]
struct Controls {
    absolute_value_separate_from_peer_rank: bool,
    longitudinal_change_separate_from_peer_rank: bool,
    no_causal_claim_from_percentile: bool,
    no_aggregate_score_emitted: bool,
    sparse_and_ood_abstention_pass: bool,
    training_group_leakage_check_pass: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if !value.is_finite() {
        return Err(format!("{name} must be finite"));
    }
    Ok(())
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_date(date: &str) -> bool {
    if date.len() != 10 {
        return false;
    }
    let b = date.as_bytes();
    b[4] == b'-'
        && b[7] == b'-'
        && b.iter()
            .enumerate()
            .all(|(i, x)| i == 4 || i == 7 || x.is_ascii_digit())
        && date[0..4]
            .parse::<u32>()
            .is_ok_and(|y| (1..=9999).contains(&y))
        && date[5..7]
            .parse::<u32>()
            .is_ok_and(|m| (1..=12).contains(&m))
        && date[8..10]
            .parse::<u32>()
            .is_ok_and(|d| (1..=31).contains(&d))
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "soil-context-function-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("unsupported version or non-synthetic evidence; v0.1 is synthetic-only".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.indicators.is_empty()
        || s.config.minimum_peer_samples < 2
        || s.config.minimum_independent_sites < 2
    {
        return Err("scenario, indicators and peer minimums must be valid".into());
    }

    let mut ids = BTreeSet::new();
    for indicator in &s.indicators {
        if indicator.indicator_id.trim().is_empty()
            || indicator.property_name.trim().is_empty()
            || indicator.unit.trim().is_empty()
            || indicator.measurement_method.trim().is_empty()
            || indicator.evaluation_independence_group_id.trim().is_empty()
            || !ids.insert(indicator.indicator_id.clone())
        {
            return Err(
                "indicator IDs must be non-empty and unique; metadata must be present".into(),
            );
        }
        for (name, value) in [
            ("depth_min_m", indicator.depth_min_m),
            ("depth_max_m", indicator.depth_max_m),
            (
                "reference_depth_min_m",
                indicator.reference_model.reference_depth_min_m,
            ),
            (
                "reference_depth_max_m",
                indicator.reference_model.reference_depth_max_m,
            ),
            (
                "mean_annual_temperature_c",
                indicator.context.mean_annual_temperature_c,
            ),
            (
                "mean_annual_precipitation_mm",
                indicator.context.mean_annual_precipitation_mm,
            ),
            ("predictive_mean", indicator.reference_model.predictive_mean),
            ("predictive_sd", indicator.reference_model.predictive_sd),
            (
                "predictive_mean_se",
                indicator.reference_model.predictive_mean_se,
            ),
            ("q90", indicator.reference_model.q90),
            ("q90_se", indicator.reference_model.q90_se),
            ("q95", indicator.reference_model.q95),
            ("q95_se", indicator.reference_model.q95_se),
            ("q99", indicator.reference_model.q99),
            ("q99_se", indicator.reference_model.q99_se),
        ] {
            finite(name, value)?;
        }
        if indicator.depth_min_m < 0.0
            || indicator.depth_max_m <= indicator.depth_min_m
            || indicator.context.mean_annual_precipitation_mm < 0.0
        {
            return Err(format!(
                "{} has invalid depth or climate context",
                indicator.indicator_id
            ));
        }
        if indicator.context.reference_soil_group.trim().is_empty()
            || indicator.reference_model.model_id.trim().is_empty()
            || indicator.reference_model.model_version.trim().is_empty()
            || indicator
                .reference_model
                .reference_dataset_id
                .trim()
                .is_empty()
            || indicator
                .reference_model
                .reference_measurement_method
                .trim()
                .is_empty()
            || indicator
                .reference_model
                .reference_measurement_unit
                .trim()
                .is_empty()
            || indicator.reference_model.peer_group_id.trim().is_empty()
        {
            return Err(format!(
                "{} has incomplete peer/model identity",
                indicator.indicator_id
            ));
        }
        if !valid_sha256(&indicator.reference_model.reference_dataset_sha256)
            || !valid_sha256(&indicator.reference_model.reference_model_sha256)
        {
            return Err(format!(
                "{} reference dataset/model SHA-256 is invalid",
                indicator.indicator_id
            ));
        }
        if indicator.reference_model.reference_measurement_method != indicator.measurement_method
            || indicator.reference_model.reference_measurement_unit != indicator.unit
            || (indicator.reference_model.reference_depth_min_m - indicator.depth_min_m).abs()
                > 1e-9
            || (indicator.reference_model.reference_depth_max_m - indicator.depth_max_m).abs()
                > 1e-9
        {
            return Err(format!(
                "{} current measurement and peer reference differ in method, unit or depth support",
                indicator.indicator_id
            ));
        }
        if indicator.reference_model.reference_target_semantics != "direct_measurement" {
            return Err(format!(
                "{} reference targets must be direct measurements, not derived predictions",
                indicator.indicator_id
            ));
        }
        if !matches!(
            indicator.reference_model.domain_status.as_str(),
            "in_domain" | "out_of_domain" | "unassessed"
        ) {
            return Err("unsupported reference domain status".into());
        }
        if indicator
            .reference_model
            .training_independence_group_ids
            .is_empty()
        {
            return Err("reference model must declare training independence groups".into());
        }
        let mut training = BTreeSet::new();
        for group in &indicator.reference_model.training_independence_group_ids {
            if group.trim().is_empty() || !training.insert(group.clone()) {
                return Err("training independence groups must be non-empty and unique".into());
            }
        }
        if training.contains(&indicator.evaluation_independence_group_id) {
            return Err(format!(
                "{} evaluation unit group leaks into peer-reference training groups",
                indicator.indicator_id
            ));
        }
        if indicator.reference_model.predictive_sd <= 0.0
            || indicator.reference_model.predictive_mean_se < 0.0
            || indicator.reference_model.q90_se < 0.0
            || indicator.reference_model.q95_se < 0.0
            || indicator.reference_model.q99_se < 0.0
            || indicator.reference_model.q90 > indicator.reference_model.q95
            || indicator.reference_model.q95 > indicator.reference_model.q99
        {
            return Err(format!(
                "{} has invalid peer distribution / quantiles",
                indicator.indicator_id
            ));
        }
        if indicator.measurements.len() < 2 {
            return Err(format!(
                "{} needs at least two repeated measurements",
                indicator.indicator_id
            ));
        }
        let mut prior_date: Option<&str> = None;
        let mut experimental_unit: Option<&str> = None;
        for measurement in &indicator.measurements {
            if !valid_date(&measurement.date) {
                return Err(format!(
                    "{} has invalid measurement date",
                    indicator.indicator_id
                ));
            }
            finite("measurement value", measurement.value)?;
            finite("measurement standard_error", measurement.standard_error)?;
            if measurement.standard_error < 0.0
                || measurement.experimental_unit_id.trim().is_empty()
            {
                return Err(format!(
                    "{} has invalid measurement uncertainty/unit",
                    indicator.indicator_id
                ));
            }
            if let Some(previous) = prior_date
                && measurement.date.as_str() <= previous
            {
                return Err(format!(
                    "{} measurement dates must increase strictly",
                    indicator.indicator_id
                ));
            }
            if let Some(unit) = experimental_unit {
                if measurement.experimental_unit_id != unit {
                    return Err(format!(
                        "{} repeated observations must belong to the same experimental unit",
                        indicator.indicator_id
                    ));
                }
            } else {
                experimental_unit = Some(&measurement.experimental_unit_id);
            }
            prior_date = Some(&measurement.date);
        }
        validate_guardrail(&indicator.guardrail, &indicator.indicator_id)?;
    }
    Ok(())
}

fn validate_guardrail(g: &Guardrail, id: &str) -> Result<(), String> {
    if !matches!(
        g.status.as_str(),
        "not_set" | "synthetic_fixture" | "externally_approved"
    ) {
        return Err(format!("{id} has unsupported guardrail status"));
    }
    for (name, value) in [
        ("minimum_value", g.minimum_value),
        ("maximum_value", g.maximum_value),
    ] {
        if let Some(x) = value {
            finite(name, x)?;
        }
    }
    if g.minimum_value
        .zip(g.maximum_value)
        .is_some_and(|(a, b)| a > b)
    {
        return Err(format!("{id} guardrail minimum exceeds maximum"));
    }
    match g.status.as_str() {
        "not_set" => {
            if g.minimum_value.is_some()
                || g.maximum_value.is_some()
                || g.authority.is_some()
                || g.approval_reference.is_some()
                || g.approval_date.is_some()
            {
                return Err(format!(
                    "{id} not_set guardrail must not carry bounds or approvals"
                ));
            }
        }
        "synthetic_fixture" => {
            if g.minimum_value.is_none() && g.maximum_value.is_none() {
                return Err(format!("{id} synthetic guardrail needs at least one bound"));
            }
        }
        "externally_approved" => {
            if (g.minimum_value.is_none() && g.maximum_value.is_none())
                || g.authority.as_deref().unwrap_or("").trim().is_empty()
                || g.approval_reference
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .is_empty()
                || !g.approval_date.as_deref().is_some_and(valid_date)
            {
                return Err(format!(
                    "{id} externally approved guardrail lacks approval evidence"
                ));
            }
        }
        _ => unreachable!(),
    }
    Ok(())
}

/// Abramowitz–Stegun approximation; sufficient for a transparent fixture
/// contract, not a replacement for a validated posterior CDF in real use.
fn erf_approx(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let y = 1.0
        - (((((1.061_405_429 * t - 1.453_152_027) * t) + 1.421_413_741) * t - 0.284_496_736) * t
            + 0.254_829_592)
            * t
            * (-x * x).exp();
    sign * y
}

fn normal_cdf(z: f64) -> f64 {
    (0.5 * (1.0 + erf_approx(z / std::f64::consts::SQRT_2))).clamp(0.0, 1.0)
}

fn peer_reference(indicator: &Indicator, latest: &Measurement, config: &Config) -> PeerReference {
    let model = &indicator.reference_model;
    let status = if model.domain_status == "out_of_domain" {
        "out_of_domain_abstain"
    } else if model.domain_status == "unassessed" {
        "unassessed_domain_abstain"
    } else if model.peer_sample_count < config.minimum_peer_samples
        || model.independent_site_count < config.minimum_independent_sites
    {
        "sparse_peer_group_abstain"
    } else {
        "qualified_peer_reference"
    };

    if status != "qualified_peer_reference" {
        return PeerReference {
            status,
            peer_group_id: model.peer_group_id.clone(),
            reference_dataset_id: model.reference_dataset_id.clone(),
            reference_dataset_sha256: model.reference_dataset_sha256.clone(),
            reference_model_sha256: model.reference_model_sha256.clone(),
            texture_class: indicator.context.texture_class.clone(),
            reference_soil_group: indicator.context.reference_soil_group.clone(),
            mean_annual_temperature_c: indicator.context.mean_annual_temperature_c,
            mean_annual_precipitation_mm: indicator.context.mean_annual_precipitation_mm,
            peer_sample_count: model.peer_sample_count,
            independent_site_count: model.independent_site_count,
            percentile: None,
            percentile_interval_95: None,
            distance_to_q90: None,
            distance_to_q95: None,
            distance_to_q99: None,
            distance_standard_errors: None,
            interpretation: "No peer percentile emitted: reference domain or peer-group support is insufficient.",
        };
    }

    let combined_sd = (model.predictive_sd.powi(2)
        + model.predictive_mean_se.powi(2)
        + latest.standard_error.powi(2))
    .sqrt();
    let z = (latest.value - model.predictive_mean) / combined_sd;
    let percentile = 100.0 * normal_cdf(z);

    // Approximate interval moves the observed value by measurement uncertainty
    // and the peer mean by its standard error. This does not include covariance.
    let interval_low_value = latest.value - 1.96 * latest.standard_error;
    let interval_high_value = latest.value + 1.96 * latest.standard_error;
    let interval_low = 100.0
        * normal_cdf(
            (interval_low_value - (model.predictive_mean + 1.96 * model.predictive_mean_se))
                / model.predictive_sd,
        );
    let interval_high = 100.0
        * normal_cdf(
            (interval_high_value - (model.predictive_mean - 1.96 * model.predictive_mean_se))
                / model.predictive_sd,
        );

    let q90_se = (model.q90_se.powi(2) + latest.standard_error.powi(2)).sqrt();
    let q95_se = (model.q95_se.powi(2) + latest.standard_error.powi(2)).sqrt();
    let q99_se = (model.q99_se.powi(2) + latest.standard_error.powi(2)).sqrt();

    PeerReference {
        status,
        peer_group_id: model.peer_group_id.clone(),
        reference_dataset_id: model.reference_dataset_id.clone(),
        reference_dataset_sha256: model.reference_dataset_sha256.clone(),
        reference_model_sha256: model.reference_model_sha256.clone(),
        texture_class: indicator.context.texture_class.clone(),
        reference_soil_group: indicator.context.reference_soil_group.clone(),
        mean_annual_temperature_c: indicator.context.mean_annual_temperature_c,
        mean_annual_precipitation_mm: indicator.context.mean_annual_precipitation_mm,
        peer_sample_count: model.peer_sample_count,
        independent_site_count: model.independent_site_count,
        percentile: Some(percentile),
        percentile_interval_95: Some([
            interval_low.min(interval_high),
            interval_low.max(interval_high),
        ]),
        distance_to_q90: Some(model.q90 - latest.value),
        distance_to_q95: Some(model.q95 - latest.value),
        distance_to_q99: Some(model.q99 - latest.value),
        distance_standard_errors: Some(QuantileErrors {
            q90: Some(q90_se),
            q95: Some(q95_se),
            q99: Some(q99_se),
        }),
        interpretation: "Relative position within a conditional empirical peer reference only; quantile distances are not causal or guaranteed recoverable gaps.",
    }
}

fn guardrail_result(g: &Guardrail, value: f64) -> GuardrailResult {
    match g.status.as_str() {
        "not_set" => GuardrailResult {
            status: "not_set",
            minimum_value: None,
            maximum_value: None,
            interpretation: "No absolute decision guardrail supplied; peer rank does not imply safety or agronomic adequacy.",
        },
        "synthetic_fixture" | "externally_approved" => {
            let below = g.minimum_value.is_some_and(|x| value < x);
            let above = g.maximum_value.is_some_and(|x| value > x);
            let status = match (below, above) {
                (true, true) => "outside_declared_range",
                (true, false) => "below_minimum",
                (false, true) => "above_maximum",
                (false, false) => "within_declared_bounds",
            };
            GuardrailResult {
                status,
                minimum_value: g.minimum_value,
                maximum_value: g.maximum_value,
                interpretation: if g.status == "synthetic_fixture" {
                    "Manufactured test guardrail only; not an approved real-world soil threshold."
                } else {
                    "Comparison against an externally approved absolute bound; the approval record remains authoritative."
                },
            }
        }
        _ => unreachable!(),
    }
}

fn build_report(s: &Scenario, digest: String) -> Report {
    let mut results = Vec::with_capacity(s.indicators.len());
    let mut abstention_pass = true;
    let mut leak_pass = true;

    for indicator in &s.indicators {
        let first = &indicator.measurements[0];
        let latest = indicator
            .measurements
            .last()
            .expect("validation requires measurements");
        let change_se = (first.standard_error.powi(2) + latest.standard_error.powi(2)).sqrt();
        let peer = peer_reference(indicator, latest, &s.config);
        if peer.status != "qualified_peer_reference"
            && (peer.percentile.is_some() || peer.percentile_interval_95.is_some())
        {
            abstention_pass = false;
        }
        if indicator
            .reference_model
            .training_independence_group_ids
            .contains(&indicator.evaluation_independence_group_id)
        {
            leak_pass = false;
        }

        results.push(IndicatorResult {
            indicator_id: indicator.indicator_id.clone(),
            property_name: indicator.property_name.clone(),
            absolute_condition: AbsoluteCondition {
                value: latest.value,
                unit: indicator.unit.clone(),
                measurement_method: indicator.measurement_method.clone(),
                depth_min_m: indicator.depth_min_m,
                depth_max_m: indicator.depth_max_m,
                latest_date: latest.date.clone(),
                standard_error: latest.standard_error,
            },
            peer_reference: peer,
            longitudinal_change: LongitudinalChange {
                experimental_unit_id: first.experimental_unit_id.clone(),
                start_date: first.date.clone(),
                end_date: latest.date.clone(),
                first_value: first.value,
                last_value: latest.value,
                absolute_change: latest.value - first.value,
                change_standard_error: change_se,
                unit: indicator.unit.clone(),
                interpretation: "Descriptive first-to-last change within one experimental unit; not a treatment effect or independent replication.",
            },
            guardrail_result: guardrail_result(&indicator.guardrail, latest.value),
            causal_inference_status: "not_estimated_by_v0_1",
        });
    }

    Report {
        audit_version: AUDIT_VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256: digest,
        indicator_count: results.len(),
        indicators: results,
        controls: Controls {
            absolute_value_separate_from_peer_rank: true,
            longitudinal_change_separate_from_peer_rank: true,
            no_causal_claim_from_percentile: true,
            no_aggregate_score_emitted: true,
            sparse_and_ood_abstention_pass: abstention_pass,
            training_group_leakage_check_pass: leak_pass,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(
            io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into(),
        );
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: context-function-audit <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("context-function audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-context-function.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn synthetic_fixture_has_two_independent_indicator_records() {
        let s = scenario();
        validate(&s).expect("valid synthetic fixture");
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(report.indicator_count, 2);
        assert!(report.controls.no_aggregate_score_emitted);
    }

    #[test]
    fn peer_rank_does_not_replace_absolute_value_or_longitudinal_change() {
        let mut s = scenario();
        let report_before = build_report(&s, "0".repeat(64));
        let before = &report_before.indicators[0];
        let absolute = before.absolute_condition.value;
        let change = before.longitudinal_change.absolute_change;
        let percentile_before = before.peer_reference.percentile.unwrap();

        s.indicators[0].reference_model.predictive_mean = 3.0;
        s.indicators[0].reference_model.q90 += 0.6;
        s.indicators[0].reference_model.q95 += 0.6;
        s.indicators[0].reference_model.q99 += 0.6;
        let report_after = build_report(&s, "0".repeat(64));
        let after = &report_after.indicators[0];

        assert_ne!(percentile_before, after.peer_reference.percentile.unwrap());
        assert_eq!(absolute, after.absolute_condition.value);
        assert_eq!(change, after.longitudinal_change.absolute_change);
    }

    #[test]
    fn sparse_peer_group_abstains_instead_of_borrowing_another_reference() {
        let mut s = scenario();
        s.indicators[0].reference_model.peer_sample_count = 20;
        let report = build_report(&s, "0".repeat(64));
        let peer = &report.indicators[0].peer_reference;
        assert_eq!(peer.status, "sparse_peer_group_abstain");
        assert!(peer.percentile.is_none());
        assert!(peer.distance_to_q95.is_none());
    }

    #[test]
    fn out_of_domain_reference_abstains() {
        let mut s = scenario();
        s.indicators[0].reference_model.domain_status = "out_of_domain".into();
        let report = build_report(&s, "0".repeat(64));
        assert_eq!(
            report.indicators[0].peer_reference.status,
            "out_of_domain_abstain"
        );
        assert!(report.indicators[0].peer_reference.percentile.is_none());
    }

    #[test]
    fn high_peer_rank_cannot_erase_absolute_guardrail_failure() {
        let s = scenario();
        let report = build_report(&s, "0".repeat(64));
        let bulk = &report.indicators[1];
        assert!(bulk.peer_reference.percentile.unwrap() > 50.0);
        assert_eq!(bulk.guardrail_result.status, "above_maximum");
    }

    #[test]
    fn training_independence_group_overlap_is_rejected() {
        let mut s = scenario();
        s.indicators[0]
            .reference_model
            .training_independence_group_ids
            .push("evaluation-site-A-plot-01".into());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn repeated_measurements_must_be_same_unit_and_ordered() {
        let mut s = scenario();
        s.indicators[0].measurements[1].experimental_unit_id = "other-plot".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.indicators[0].measurements[2].date = "2022-01-01".into();
        assert!(validate(&t).is_err());
    }

    #[test]
    fn peer_reference_method_unit_and_depth_must_match_observation() {
        let mut s = scenario();
        s.indicators[0].reference_model.reference_measurement_method =
            "different laboratory method".into();
        assert!(validate(&s).is_err());

        let mut t = scenario();
        t.indicators[0].reference_model.reference_depth_max_m = 0.1;
        assert!(validate(&t).is_err());
    }

    #[test]
    fn peer_reference_digests_must_be_valid_sha256() {
        let mut s = scenario();
        s.indicators[0].reference_model.reference_dataset_sha256 = "not-a-digest".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn peer_reference_report_retains_context_and_artifact_identity() {
        let s = scenario();
        let report = build_report(&s, "0".repeat(64));
        let peer = &report.indicators[0].peer_reference;
        assert_eq!(peer.texture_class, "fine");
        assert_eq!(peer.reference_soil_group, "synthetic-RSG-X");
        assert_eq!(peer.mean_annual_temperature_c, 18.0);
        assert_eq!(peer.reference_dataset_id, "synthetic-reference-dataset-v1");
        assert_eq!(peer.reference_dataset_sha256, "a".repeat(64));
    }

    #[test]
    fn percentile_does_not_create_causal_claim() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!(
            report
                .indicators
                .iter()
                .all(|x| x.causal_inference_status == "not_estimated_by_v0_1")
        );
    }

    #[test]
    fn synthetic_guardrail_is_not_called_an_approved_real_threshold() {
        let report = build_report(&scenario(), "0".repeat(64));
        assert!(
            report.indicators[1]
                .guardrail_result
                .interpretation
                .contains("not an approved real-world")
        );
    }
}
