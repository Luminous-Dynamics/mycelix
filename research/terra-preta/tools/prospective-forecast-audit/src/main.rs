//! Synthetic prequential forecast lock and verification for soil digital twins.
//! Forecasts are immutable, issued before target windows, and scored only after
//! a compatible later reference observation. This is not a real field forecast.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const BOUNDARY: &str = "Synthetic prequential forecast audit only. No live soil model, field instrument or agronomic decision is validated. Proper scores assume the declared Normal predictive distribution; the receipt digest binds the supplied forecast record but does not authenticate its origin.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Suite {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    suite_id: String,
    targets: Vec<Target>,
    forecasts: Vec<Forecast>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Target {
    target_id: String,
    site_id: String,
    experimental_unit_id: String,
    independence_group_id: String,
    variable: String,
    unit: String,
    measurement_method: String,
    depth_min_m: f64,
    depth_max_m: f64,
    spatial_support: String,
    window_start: String,
    window_end: String,
    observed_at: String,
    observed_value: f64,
    observation_sha256: String,
    observation_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Forecast {
    forecast_id: String,
    target_id: String,
    model_role: String,
    model_id: String,
    model_version: String,
    model_sha256: String,
    input_sha256: String,
    forecast_receipt_sha256: String,
    training_cutoff: String,
    issued_at: String,
    evaluation_independence_group_id: String,
    training_independence_group_ids: Vec<String>,
    variable: String,
    unit: String,
    measurement_method: String,
    depth_min_m: f64,
    depth_max_m: f64,
    spatial_support: String,
    applicability_domain_status: String,
    predictive_mean: f64,
    predictive_sd: f64,
    interval_level: f64,
    interval_lower: f64,
    interval_upper: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    report_version: &'static str,
    fixture_status: &'static str,
    suite_id: String,
    input_sha256: String,
    target_count: usize,
    forecast_count: usize,
    issued_forecast_count: usize,
    scored_forecast_count: usize,
    abstained_out_of_domain_count: usize,
    abstained_unassessed_domain_count: usize,
    scored_group_count: usize,
    groups: Vec<GroupScore>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct GroupScore {
    model_id: String,
    model_role: String,
    forecast_count: usize,
    scored_count: usize,
    abstained_count: usize,
    mae: Option<f64>,
    rmse: Option<f64>,
    bias: Option<f64>,
    crps_normal: Option<f64>,
    interval_coverage: Option<f64>,
    mean_interval_width: Option<f64>,
    winkler_interval_score: Option<f64>,
    matched_target_count: usize,
    skill_vs_persistence_mae: Option<f64>,
    skill_vs_climatology_mae: Option<f64>,
}

struct Accumulator {
    model_id: String,
    model_role: String,
    forecast_count: usize,
    scored_count: usize,
    abstained_count: usize,
    abs_error: f64,
    squared_error: f64,
    signed_error: f64,
    crps: f64,
    covered: usize,
    interval_width: f64,
    winkler: f64,
    errors_by_target: BTreeMap<String, f64>,
}

fn finite(name: &str, x: f64) -> Result<(), String> {
    if x.is_finite() { Ok(()) } else { Err(format!("{name} must be finite")) }
}

fn valid_sha256(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
}

fn valid_timestamp(s: &str) -> bool {
    s.len() >= 20
        && s.ends_with('Z')
        && s.as_bytes().get(4) == Some(&b'-')
        && s.as_bytes().get(7) == Some(&b'-')
        && s.as_bytes().get(10) == Some(&b'T')
        && s.contains('T')
}

fn erf_approx(x: f64) -> f64 {
    let sign = if x < 0.0 { -1.0 } else { 1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.327_591_1 * x);
    let y = 1.0 - (((((1.061_405_429 * t - 1.453_152_027) * t
        + 1.421_413_741) * t - 0.284_496_736) * t + 0.254_829_592) * t)
        * (-x * x).exp();
    sign * y
}

fn normal_cdf(z: f64) -> f64 {
    0.5 * (1.0 + erf_approx(z / std::f64::consts::SQRT_2))
}

fn normal_pdf(z: f64) -> f64 {
    (-0.5 * z * z).exp() / (2.0 * std::f64::consts::PI).sqrt()
}

fn crps_normal(mean: f64, sd: f64, observed: f64) -> f64 {
    let z = (observed - mean) / sd;
    sd * (z * (2.0 * normal_cdf(z) - 1.0) + 2.0 * normal_pdf(z)
        - 1.0 / std::f64::consts::PI.sqrt())
}

fn validate(s: &Suite) -> Result<(), String> {
    if s.schema_version != "soil-prospective-forecast-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
        || s.suite_id.trim().is_empty()
        || s.targets.is_empty()
        || s.forecasts.is_empty()
    {
        return Err("unsupported schema or non-synthetic/empty suite".into());
    }
    let mut targets = BTreeMap::new();
    let mut eval_groups = BTreeSet::new();
    for t in &s.targets {
        if t.target_id.trim().is_empty()
            || t.site_id.trim().is_empty()
            || t.experimental_unit_id.trim().is_empty()
            || t.independence_group_id.trim().is_empty()
            || t.variable.trim().is_empty()
            || t.unit.trim().is_empty()
            || t.measurement_method.trim().is_empty()
            || t.spatial_support.trim().is_empty()
            || t.observation_status != "independent_reference_observation"
            || !valid_timestamp(&t.window_start)
            || !valid_timestamp(&t.window_end)
            || !valid_timestamp(&t.observed_at)
            || t.window_start >= t.window_end
            || t.observed_at < t.window_end
            || !valid_sha256(&t.observation_sha256)
            || targets.insert(t.target_id.as_str(), t).is_some()
        {
            return Err(format!("target {} has invalid identity, observation or temporal metadata", t.target_id));
        }
        for (name, value) in [
            ("observed_value", t.observed_value),
            ("depth_min_m", t.depth_min_m),
            ("depth_max_m", t.depth_max_m),
        ] {
            finite(name, value)?;
        }
        if t.depth_min_m < 0.0 || t.depth_max_m <= t.depth_min_m {
            return Err(format!("target {} has invalid depth interval", t.target_id));
        }
        eval_groups.insert(t.independence_group_id.as_str());
    }

    let mut forecast_ids = BTreeSet::new();
    let mut receipts = BTreeSet::new();
    let mut target_role_pairs = BTreeSet::new();
    for f in &s.forecasts {
        let t = targets.get(f.target_id.as_str())
            .ok_or_else(|| format!("forecast {} refers to an unknown target", f.forecast_id))?;
        if f.forecast_id.trim().is_empty()
            || f.model_id.trim().is_empty()
            || f.model_version.trim().is_empty()
            || !forecast_ids.insert(f.forecast_id.as_str())
            || !receipts.insert(f.forecast_receipt_sha256.as_str())
            || !target_role_pairs.insert((f.target_id.as_str(), f.model_role.as_str()))
        {
            return Err(format!("forecast {} has duplicate/empty identity or duplicate target-role", f.forecast_id));
        }
        if !matches!(f.model_role.as_str(), "candidate" | "persistence_baseline" | "climatology_baseline") {
            return Err(format!("forecast {} has unsupported model role", f.forecast_id));
        }
        if !matches!(f.applicability_domain_status.as_str(), "in_domain" | "out_of_domain" | "unassessed") {
            return Err(format!("forecast {} has unsupported applicability status", f.forecast_id));
        }
        if !valid_sha256(&f.model_sha256)
            || !valid_sha256(&f.input_sha256)
            || !valid_sha256(&f.forecast_receipt_sha256)
        {
            return Err(format!("forecast {} has invalid digest", f.forecast_id));
        }
        if !valid_timestamp(&f.training_cutoff)
            || !valid_timestamp(&f.issued_at)
            || f.training_cutoff > f.issued_at
            || f.issued_at >= t.window_start
        {
            return Err(format!("forecast {} violates training/issue/target chronology", f.forecast_id));
        }
        if f.evaluation_independence_group_id != t.independence_group_id
            || f.training_independence_group_ids.is_empty()
            || f.training_independence_group_ids.iter().any(|g| g == &f.evaluation_independence_group_id)
        {
            return Err(format!("forecast {} has invalid training/evaluation independence groups", f.forecast_id));
        }
        for (name, value) in [
            ("predictive_mean", f.predictive_mean),
            ("predictive_sd", f.predictive_sd),
            ("interval_level", f.interval_level),
            ("interval_lower", f.interval_lower),
            ("interval_upper", f.interval_upper),
            ("forecast_depth_min_m", f.depth_min_m),
            ("forecast_depth_max_m", f.depth_max_m),
        ] {
            finite(name, value)?;
        }
        if f.predictive_sd <= 0.0
            || !(0.0 < f.interval_level && f.interval_level < 1.0)
            || f.interval_lower > f.interval_upper
            || f.depth_min_m < 0.0
            || f.depth_max_m <= f.depth_min_m
        {
            return Err(format!("forecast {} has invalid predictive distribution/support", f.forecast_id));
        }
        if f.variable != t.variable
            || f.unit != t.unit
            || f.measurement_method != t.measurement_method
            || (f.depth_min_m - t.depth_min_m).abs() > 1e-9
            || (f.depth_max_m - t.depth_max_m).abs() > 1e-9
            || f.spatial_support != t.spatial_support
        {
            return Err(format!("forecast {} does not match target method/unit/depth/spatial support", f.forecast_id));
        }
    }
    for target_id in targets.keys() {
        for role in ["candidate", "persistence_baseline", "climatology_baseline"] {
            if !target_role_pairs.contains(&(target_id, role)) {
                return Err(format!("target {target_id} lacks required {role} forecast"));
            }
        }
    }
    Ok(())
}

fn verify(s: &Suite, input_sha256: String) -> Report {
    let targets: BTreeMap<&str, &Target> = s.targets.iter().map(|t| (t.target_id.as_str(), t)).collect();
    let mut groups: BTreeMap<(&str, &str), Accumulator> = BTreeMap::new();
    let mut issued = 0usize;
    let mut scored = 0usize;
    let mut abstained_ood = 0usize;
    let mut abstained_unassessed = 0usize;
    for f in &s.forecasts {
        issued += 1;
        let entry = groups.entry((f.model_id.as_str(), f.model_role.as_str())).or_insert_with(|| Accumulator {
            model_id: f.model_id.clone(),
            model_role: f.model_role.clone(),
            forecast_count: 0,
            scored_count: 0,
            abstained_count: 0,
            abs_error: 0.0,
            squared_error: 0.0,
            signed_error: 0.0,
            crps: 0.0,
            covered: 0,
            interval_width: 0.0,
            winkler: 0.0,
            errors_by_target: BTreeMap::new(),
        });
        entry.forecast_count += 1;
        if f.applicability_domain_status != "in_domain" {
            entry.abstained_count += 1;
            if f.applicability_domain_status == "out_of_domain" {
                abstained_ood += 1;
            } else {
                abstained_unassessed += 1;
            }
            continue;
        }
        let t = targets[f.target_id.as_str()];
        let err = f.predictive_mean - t.observed_value;
        let alpha = 1.0 - f.interval_level;
        let width = f.interval_upper - f.interval_lower;
        let penalty = if t.observed_value < f.interval_lower {
            (2.0 / alpha) * (f.interval_lower - t.observed_value)
        } else if t.observed_value > f.interval_upper {
            (2.0 / alpha) * (t.observed_value - f.interval_upper)
        } else {
            0.0
        };
        let crps = crps_normal(f.predictive_mean, f.predictive_sd, t.observed_value);
        entry.scored_count += 1;
        entry.abs_error += err.abs();
        entry.squared_error += err * err;
        entry.signed_error += err;
        entry.crps += crps;
        entry.covered += usize::from(t.observed_value >= f.interval_lower && t.observed_value <= f.interval_upper);
        entry.interval_width += width;
        entry.winkler += width + penalty;
        entry.errors_by_target.insert(f.target_id.clone(), err.abs());
        scored += 1;
    }

    let baseline_maps: BTreeMap<&str, BTreeMap<String, f64>> = [
        "persistence_baseline",
        "climatology_baseline",
    ].into_iter().map(|role| {
        let values = groups.values()
            .filter(|g| g.model_role == role)
            .flat_map(|g| g.errors_by_target.iter().map(|(k, v)| (k.clone(), *v)))
            .collect();
        (role, values)
    }).collect();

    let mut scored_groups = Vec::new();
    for acc in groups.values() {
        let n = acc.scored_count;
        let matched = acc.errors_by_target.len();
        let skill = |baseline_role: &str| -> Option<f64> {
            let baseline = baseline_maps.get(baseline_role)?;
            let common: Vec<(&String, &f64)> = acc.errors_by_target.iter()
                .filter(|(target_id, _)| baseline.contains_key(*target_id))
                .collect();
            if common.is_empty() || common.len() != matched {
                return None;
            }
            let candidate_mae = common.iter().map(|(_, e)| **e).sum::<f64>() / common.len() as f64;
            let baseline_mae = common.iter().map(|(id, _)| baseline[*id]).sum::<f64>() / common.len() as f64;
            (baseline_mae > f64::EPSILON).then_some(1.0 - candidate_mae / baseline_mae)
        };
        scored_groups.push(GroupScore {
            model_id: acc.model_id.clone(),
            model_role: acc.model_role.clone(),
            forecast_count: acc.forecast_count,
            scored_count: n,
            abstained_count: acc.abstained_count,
            mae: (n > 0).then_some(acc.abs_error / n as f64),
            rmse: (n > 0).then_some((acc.squared_error / n as f64).sqrt()),
            bias: (n > 0).then_some(acc.signed_error / n as f64),
            crps_normal: (n > 0).then_some(acc.crps / n as f64),
            interval_coverage: (n > 0).then_some(acc.covered as f64 / n as f64),
            mean_interval_width: (n > 0).then_some(acc.interval_width / n as f64),
            winkler_interval_score: (n > 0).then_some(acc.winkler / n as f64),
            matched_target_count: matched,
            skill_vs_persistence_mae: if acc.model_role == "candidate" { skill("persistence_baseline") } else { None },
            skill_vs_climatology_mae: if acc.model_role == "candidate" { skill("climatology_baseline") } else { None },
        });
    }
    scored_groups.sort_by(|a,b| a.model_id.cmp(&b.model_id));
    Report {
        report_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        suite_id: s.suite_id.clone(),
        input_sha256,
        target_count: s.targets.len(),
        forecast_count: s.forecasts.len(),
        issued_forecast_count: issued,
        scored_forecast_count: scored,
        abstained_out_of_domain_count: abstained_ood,
        abstained_unassessed_domain_count: abstained_unassessed,
        scored_group_count: scored_groups.len(),
        groups: scored_groups,
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input: &Path, output: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input)?;
    let suite: Suite = serde_json::from_slice(&bytes)?;
    validate(&suite).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = verify(&suite, digest);
    let out = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(output)?;
    file.write_all(&out)?;
    file.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: prospective-forecast-audit <suite.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(e) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("prospective forecast audit failed: {e}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn suite() -> Suite {
        serde_json::from_str(include_str!("../fixtures/synthetic-prospective-forecasts.json"))
            .expect("fixture parses")
    }

    #[test]
    fn synthetic_suite_scores_all_roles_and_keeps_ood_abstention() {
        let s = suite();
        validate(&s).expect("valid suite");
        let r = verify(&s, "0".repeat(64));
        assert_eq!(r.target_count, 4);
        assert_eq!(r.forecast_count, 12);
        assert_eq!(r.abstained_out_of_domain_count, 1);
        assert_eq!(r.scored_forecast_count, 11);
        assert_eq!(r.scored_group_count, 3);
        let candidate = r.groups.iter().find(|x| x.model_role == "candidate").unwrap();
        assert_eq!(candidate.scored_count, 3);
        assert!(candidate.skill_vs_persistence_mae.unwrap() > 0.0);
        assert!(candidate.skill_vs_climatology_mae.unwrap() > 0.0);
    }

    #[test]
    fn candidate_skill_uses_exact_candidate_target_set_when_baseline_has_more_targets() {
        let s = suite();
        let r = verify(&s, "0".repeat(64));
        let candidate = r.groups.iter().find(|x| x.model_role == "candidate").unwrap();
        assert_eq!(candidate.matched_target_count, 3);
        assert!(candidate.skill_vs_persistence_mae.is_some());
        assert!(candidate.skill_vs_climatology_mae.is_some());
    }

    #[test]
    fn rejects_non_utc_timestamp_to_keep_chronology_unambiguous() {
        let mut s = suite();
        s.forecasts[0].issued_at = "2026-03-01T01:00:00+01:00".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn every_target_requires_both_matched_baselines() {
        let mut s = suite();
        s.forecasts.retain(|f| !(f.target_id == "t-001" && f.model_role == "climatology_baseline"));
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rejects_issue_at_or_after_target_window() {
        let mut s = suite();
        s.forecasts[0].issued_at = s.targets[0].window_start.clone();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rejects_future_training_cutoff() {
        let mut s = suite();
        s.forecasts[0].training_cutoff = "2026-03-02T00:00:00Z".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rejects_evaluation_group_leakage() {
        let mut s = suite();
        s.forecasts[0].training_independence_group_ids.push("eval-site-A".into());
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rejects_support_mismatch() {
        let mut s = suite();
        s.forecasts[0].measurement_method = "different assay".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn rejects_duplicate_forecast_receipt_or_target_role() {
        let mut s = suite();
        s.forecasts[1].forecast_receipt_sha256 = s.forecasts[0].forecast_receipt_sha256.clone();
        assert!(validate(&s).is_err());

        let mut t = suite();
        t.forecasts[1].forecast_id = "different-id".into();
        t.forecasts[1].model_role = "candidate".into();
        assert!(validate(&t).is_err());
    }
}
