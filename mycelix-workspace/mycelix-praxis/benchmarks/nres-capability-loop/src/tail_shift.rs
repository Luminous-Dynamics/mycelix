//! NRES-010: importance-weighted rare-event tail calibration across declared shifts.
//! This evaluator verifies forecasts; it does not generate outages or infer causality.

use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const REGIMES: &[&str] = &[
    "in_distribution", "temporal_shift", "structural_dependency_shift", "evidence_degradation",
];
const FORBIDDEN: &[&str] = &[
    "overall_resilience_score", "composite_score", "leaderboard_score",
    "personal_suitability_score", "political_affiliation", "enlistment_interest",
    "loyalty_score", "participant_name", "participant_email", "participant_phone",
];

fn string<'a>(v: &'a Value, k: &str) -> Option<&'a str> { v.get(k)?.as_str() }
fn number(v: &Value, k: &str) -> Option<f64> { v.get(k)?.as_f64() }

fn scan_forbidden(v: &Value, path: &str, errors: &mut Vec<String>) {
    match v {
        Value::Object(map) => for (key, child) in map {
            if FORBIDDEN.contains(&key.as_str()) { errors.push(format!("forbidden field at {path}/{key}")); }
            scan_forbidden(child, &format!("{path}/{key}"), errors);
        },
        Value::Array(items) => for (i, child) in items.iter().enumerate() {
            scan_forbidden(child, &format!("{path}/{i}"), errors);
        },
        _ => {}
    }
}

#[derive(Clone)]
struct Case {
    regime: String,
    severity: f64,
    weight: f64,
    forecast: bool,
    probability: Option<f64>,
    scale: Option<f64>,
    shape: Option<f64>,
}

fn ess(weights: &[f64]) -> Option<f64> {
    let sum: f64 = weights.iter().sum();
    let squares: f64 = weights.iter().map(|w| w * w).sum();
    (squares > 0.0).then_some(sum * sum / squares)
}

fn gpd_cdf(x: f64, sigma: f64, xi: f64) -> Option<f64> {
    if x < 0.0 || !x.is_finite() || !sigma.is_finite() || sigma <= 0.0 || !xi.is_finite() { return None; }
    if xi.abs() < 1e-10 { return Some(1.0 - (-x / sigma).exp()); }
    let support = 1.0 + xi * x / sigma;
    if support < 0.0 { None } else if support == 0.0 { Some(1.0) }
    else { Some(1.0 - support.powf(-1.0 / xi)) }
}

fn gpd_quantile(q: f64, sigma: f64, xi: f64) -> Option<f64> {
    if q <= 0.0 || q >= 1.0 || !sigma.is_finite() || sigma <= 0.0 || !xi.is_finite() { return None; }
    let value = if xi.abs() < 1e-10 { -sigma * (1.0 - q).ln() }
        else { sigma / xi * ((1.0 - q).powf(-xi) - 1.0) };
    (value.is_finite() && value >= 0.0).then_some(value)
}

fn gpd_nll(x: f64, sigma: f64, xi: f64) -> Option<f64> {
    if x < 0.0 || !x.is_finite() || !sigma.is_finite() || sigma <= 0.0 || !xi.is_finite() { return None; }
    if xi.abs() < 1e-10 { return Some(sigma.ln() + x / sigma); }
    let support = 1.0 + xi * x / sigma;
    if support <= 0.0 { None }
    else { Some(sigma.ln() + (1.0 / xi + 1.0) * support.ln()) }
}

fn find_bin(p: f64, edges: &[f64]) -> usize {
    for i in 0..edges.len() - 1 {
        let last = i == edges.len() - 2;
        if p >= edges[i] && (p < edges[i + 1] || (last && p <= edges[i + 1])) { return i; }
    }
    edges.len() - 2
}

#[derive(Default)]
struct Bin { count: usize, weight: f64, p: f64, y: f64 }

fn summarize(
    cases: &[Case], threshold: f64, reference: f64, edges: &[f64], q_levels: &[f64],
) -> Value {
    let issued: Vec<&Case> = cases.iter().filter(|c| c.forecast).collect();
    let total_weight: f64 = cases.iter().map(|c| c.weight).sum();
    let weights: Vec<f64> = issued.iter().map(|c| c.weight).collect();
    let issued_weight: f64 = weights.iter().sum();
    let tail: Vec<&Case> = issued.iter().copied().filter(|c| c.severity > threshold).collect();
    let tail_weights: Vec<f64> = tail.iter().map(|c| c.weight).collect();
    let tail_weight: f64 = tail_weights.iter().sum();

    let mut brier = 0.0;
    let mut log_loss = 0.0;
    let mut reference_brier = 0.0;
    let mut predicted = 0.0;
    let mut observed = 0.0;
    let mut zero_probability = 0usize;
    let mut bins: Vec<Bin> = (0..edges.len() - 1).map(|_| Bin::default()).collect();

    for c in &issued {
        let p = c.probability.unwrap_or(0.0);
        let y = if c.severity > threshold { 1.0 } else { 0.0 };
        brier += c.weight * (p - y).powi(2);
        reference_brier += c.weight * (reference - y).powi(2);
        predicted += c.weight * p;
        observed += c.weight * y;
        let likelihood = if y == 1.0 { p } else { 1.0 - p };
        if likelihood <= 0.0 { zero_probability += 1; }
        else { log_loss -= c.weight * likelihood.ln(); }
        let bin = &mut bins[find_bin(p, edges)];
        bin.count += 1; bin.weight += c.weight;
        bin.p += c.weight * p; bin.y += c.weight * y;
    }

    let brier_score = (issued_weight > 0.0).then_some(brier / issued_weight);
    let log_score = (issued_weight > 0.0 && zero_probability == 0).then_some(log_loss / issued_weight);
    let climatology = (issued_weight > 0.0).then_some(reference_brier / issued_weight);
    let skill = brier_score.zip(climatology).and_then(|(s, b)| (b > 0.0).then_some(1.0 - s / b));
    let observed_rate = (issued_weight > 0.0).then_some(observed / issued_weight);
    let predicted_rate = (issued_weight > 0.0).then_some(predicted / issued_weight);
    let occurrence_ratio = observed_rate.zip(predicted_rate).and_then(|(o, p)| (p > 0.0).then_some(o / p));

    let reliability: Vec<Value> = bins.iter().enumerate().map(|(i, b)| json!({
        "bin_id": format!("BIN-{i}"), "lower": edges[i], "upper": edges[i + 1],
        "count": b.count, "weight": b.weight,
        "mean_forecast_probability": (b.weight > 0.0).then_some(b.p / b.weight),
        "observed_event_rate": (b.weight > 0.0).then_some(b.y / b.weight)
    })).collect();

    let mut pits = Vec::new();
    let mut tail_nll = 0.0;
    let mut zero_support = 0usize;
    for c in &tail {
        let x = c.severity - threshold;
        let sigma = c.scale.unwrap_or(1.0);
        let xi = c.shape.unwrap_or(0.0);
        let pit = gpd_cdf(x, sigma, xi);
        let nll = gpd_nll(x, sigma, xi);
        match (pit, nll) {
            (Some(pit), Some(nll)) => { pits.push((pit, c.weight)); tail_nll += c.weight * nll; }
            _ => zero_support += 1,
        }
    }
    pits.sort_by(|a, b| a.0.total_cmp(&b.0));
    let pit_weight: f64 = pits.iter().map(|x| x.1).sum();
    let mut cumulative = 0.0;
    let mut ks = 0.0_f64;
    if pit_weight > 0.0 {
        for (pit, weight) in &pits {
            let before = cumulative / pit_weight;
            cumulative += weight;
            let after = cumulative / pit_weight;
            ks = ks.max(after - pit).max(pit - before);
        }
    }
    let pit_ks = (pit_weight > 0.0).then_some(ks);
    let tail_log_score = (tail_weight > 0.0 && zero_support == 0).then_some(tail_nll / tail_weight);

    let quantile_coverage: Vec<Value> = q_levels.iter().map(|q| {
        let mut covered = 0.0;
        for c in &tail {
            let excess = c.severity - threshold;
            let limit = gpd_quantile(*q, c.scale.unwrap_or(1.0), c.shape.unwrap_or(0.0)).unwrap_or(0.0);
            if excess <= limit { covered += c.weight; }
        }
        json!({"nominal": q, "observed_weighted_coverage": (tail_weight > 0.0).then_some(covered / tail_weight)})
    }).collect();

    json!({
        "total_cases": cases.len(), "issued_forecasts": issued.len(), "abstentions": cases.len() - issued.len(),
        "population_forecast_coverage": (cases.len() > 0).then_some(issued.len() as f64 / cases.len() as f64),
        "weighted_forecast_coverage": (total_weight > 0.0).then_some(issued_weight / total_weight),
        "importance_weight_sum": issued_weight, "forecast_effective_sample_size": ess(&weights),
        "observed_tail_events": tail.len(), "tail_event_importance_weight_sum": tail_weight,
        "tail_event_effective_sample_size": ess(&tail_weights),
        "weighted_observed_exceedance_rate": observed_rate,
        "weighted_mean_predicted_exceedance_probability": predicted_rate,
        "tail_occurrence_calibration_ratio": occurrence_ratio,
        "brier_score": brier_score, "log_loss": log_score, "zero_probability_exceedances": zero_probability,
        "climatology_brier_score": climatology, "brier_skill_score": skill,
        "reliability_bins": reliability,
        "tail_excess_observations": tail.len(),
        "weighted_tail_excess_pit_ks_distance": pit_ks,
        "weighted_tail_excess_log_loss": tail_log_score, "zero_support_tail_excesses": zero_support,
        "conditional_tail_quantile_coverage": quantile_coverage
    })
}

/// Independently evaluates threshold occurrence and conditional GPD tail forecasts.
pub fn evaluate_tail_shift_robustness(fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    scan_forbidden(fixture, "", &mut errors);
    if string(fixture, "benchmark_id") != Some("PRAXIS-NRES-010-TAIL-SHIFT-ROBUSTNESS-001") {
        errors.push("unexpected NRES-010 benchmark_id".to_owned());
    }
    if string(fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-010 schema_version".to_owned());
    }
    if string(fixture, "status") != Some("synthetic-demonstration-for-review")
        || string(fixture, "evaluation_split") != Some("public-development-only")
        || fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false)
    { errors.push("NRES-010 fixture must remain synthetic, public-development-only and inactive".to_owned()); }

    let Some(target) = fixture.get("target") else { return Err(vec!["target definition missing".to_owned()]); };
    let threshold = number(target, "threshold");
    let target_horizon = number(target, "horizon_hours");
    if threshold.is_none_or(|v| !v.is_finite() || v < 0.0)
        || target_horizon.is_none_or(|v| !v.is_finite() || v <= 0.0)
        || string(target, "unit") != Some("normalized-service-deficit-hours")
    { errors.push("invalid target threshold, horizon or unit".to_owned()); }
    let threshold = threshold.unwrap_or(48.0);
    let target_horizon = target_horizon.unwrap_or(72.0);

    let Some(protocol) = fixture.get("protocol") else { return Err(vec!["protocol missing".to_owned()]); };
    let origin = number(protocol, "forecast_origin_hour");
    let horizon = number(protocol, "forecast_horizon_hours");
    if origin.is_none_or(|v| !v.is_finite() || v < 0.0)
        || horizon.is_none_or(|v| !v.is_finite() || v <= 0.0 || (v - target_horizon).abs() > 1e-12)
        || number(protocol, "exceedance_threshold").is_none_or(|v| (v - threshold).abs() > 1e-12)
    { errors.push("protocol origin/horizon/threshold differs from target definition".to_owned()); }
    let origin = origin.unwrap_or(0.0);
    let horizon = horizon.unwrap_or(0.0);

    let edges: Vec<f64> = protocol.get("probability_bins").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_f64).collect();
    if edges.len() < 3 || edges.first() != Some(&0.0) || edges.last() != Some(&1.0)
        || edges.windows(2).any(|w| w[0] >= w[1])
    { errors.push("probability bins must strictly increase from zero through one".to_owned()); }
    let q_levels: Vec<f64> = protocol.get("tail_quantile_levels").and_then(Value::as_array)
        .into_iter().flatten().filter_map(Value::as_f64).collect();
    if q_levels.is_empty() || q_levels.iter().any(|q| !q.is_finite() || *q <= 0.0 || *q >= 1.0)
        || q_levels.windows(2).any(|w| w[0] >= w[1])
    { errors.push("tail quantile levels must be ordered within (0,1)".to_owned()); }

    let Some(reference) = fixture.get("calibration_reference") else { return Err(vec!["calibration reference missing".to_owned()]); };
    let n = reference.get("sample_count").and_then(Value::as_u64);
    let events = reference.get("exceedance_count").and_then(Value::as_u64);
    let reference_p = number(reference, "exceedance_probability");
    if n.is_none_or(|v| v == 0) || events.is_none_or(|v| v > n.unwrap_or(0))
        || reference_p.is_none_or(|v| !v.is_finite() || !(0.0..=1.0).contains(&v))
        || string(reference, "status") != Some("synthetic-training-reference")
    { errors.push("invalid synthetic calibration reference".to_owned()); }
    let reference_p = reference_p.unwrap_or(0.04);
    if let (Some(n), Some(events)) = (n, events) {
        if ((events as f64 / n as f64) - reference_p).abs() > 1e-12 {
            errors.push("reference probability differs from numerator/denominator".to_owned());
        }
    }

    let Some(design) = fixture.get("sampling_design") else { return Err(vec!["sampling design missing".to_owned()]); };
    if string(design, "status") != Some("synthetic-stratified-importance-sampling-demonstration")
        || string(design, "weight_formula") != Some("importance_weight = target_stratum_probability / proposal_stratum_probability")
    { errors.push("sampling design status/formula mismatch".to_owned()); }
    let Some(strata) = design.get("strata").and_then(Value::as_array) else { return Err(vec!["sampling strata missing".to_owned()]); };
    let mut stratum_design: BTreeMap<String, (f64, f64)> = BTreeMap::new();
    for s in strata {
        let id = string(s, "sampling_stratum");
        let q = number(s, "proposal_probability");
        let p = number(s, "target_probability");
        if id.is_none_or(str::is_empty) || q.is_none_or(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
            || p.is_none_or(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
        { errors.push("invalid sampling stratum probabilities".to_owned()); continue; }
        if stratum_design.insert(id.unwrap().to_owned(), (q.unwrap(), p.unwrap())).is_some() {
            errors.push("duplicate sampling stratum".to_owned());
        }
    }
    let q_sum: f64 = stratum_design.values().map(|x| x.0).sum();
    let p_sum: f64 = stratum_design.values().map(|x| x.1).sum();
    if stratum_design.len() != 2 || !stratum_design.contains_key("routine")
        || !stratum_design.contains_key("tail_enriched") || (q_sum - 1.0).abs() > 1e-12
        || (p_sum - 1.0).abs() > 1e-12
    { errors.push("routine/tail_enriched proposal and target masses must each sum to one".to_owned()); }

    let Some(regime_defs) = fixture.get("regimes").and_then(Value::as_array) else { return Err(vec!["regimes missing".to_owned()]); };
    let mut regime_set = BTreeSet::new();
    let mut regime_order = Vec::new();
    for r in regime_defs {
        let id = string(r, "regime_id");
        if id.is_none_or(|v| !REGIMES.contains(&v) || !regime_set.insert(v.to_owned())) {
            errors.push("unknown or duplicate regime definition".to_owned());
        } else { regime_order.push(id.unwrap().to_owned()); }
    }
    if regime_set.len() != 4 { errors.push("exactly four declared regimes are required".to_owned()); }

    let Some(rows) = fixture.get("evaluation_cases").and_then(Value::as_array) else { return Err(vec!["evaluation cases missing".to_owned()]); };
    if rows.is_empty() { errors.push("evaluation cohort must not be empty".to_owned()); }
    let mut ids = BTreeSet::new();
    let mut stratum_counts: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();
    let mut parsed: Vec<Case> = Vec::new();
    let mut mandatory_reasons: BTreeMap<String, usize> = BTreeMap::new();
    let mut mandatory_abstentions = 0usize;
    let mut optional_abstentions = 0usize;

    for row in rows {
        let case_id = string(row, "case_id").unwrap_or("UNKNOWN").to_owned();
        if string(row, "case_id").is_none() || !ids.insert(case_id.clone()) {
            errors.push(format!("{case_id}: case ID absent or duplicated"));
        }
        let regime = string(row, "regime_id");
        if regime.is_none_or(|v| !regime_set.contains(v)) { errors.push(format!("{case_id}: undeclared regime")); continue; }
        let regime = regime.unwrap().to_owned();
        let stratum = string(row, "sampling_stratum");
        let Some((q, p)) = stratum.and_then(|v| stratum_design.get(v)).copied() else {
            errors.push(format!("{case_id}: undeclared sampling stratum")); continue;
        };
        if number(row, "proposal_stratum_probability").is_none_or(|v| (v-q).abs()>1e-12)
            || number(row, "target_stratum_probability").is_none_or(|v| (v-p).abs()>1e-12)
        { errors.push(format!("{case_id}: importance sampling design drift")); }
        let stratum = stratum.unwrap().to_owned();
        *stratum_counts.entry(regime.clone()).or_default().entry(stratum).or_default() += 1;
        if number(row, "forecast_origin_hour") != Some(origin) || number(row, "forecast_horizon_hours") != Some(horizon) {
            errors.push(format!("{case_id}: forecast origin/horizon drift"));
        }
        let Some(evaluator) = row.get("evaluator") else { errors.push(format!("{case_id}: evaluator fields missing")); continue; };
        let qualification = string(evaluator, "qualification_state");
        let severity = number(evaluator, "observed_normalized_service_deficit_hours");
        if qualification.is_none_or(str::is_empty) || severity.is_none_or(|v| !v.is_finite() || v < 0.0) {
            errors.push(format!("{case_id}: invalid evaluator qualification or observed severity")); continue;
        }
        let qualification = qualification.unwrap().to_owned();
        let severity = severity.unwrap();
        let Some(candidate) = row.get("candidate_output") else { errors.push(format!("{case_id}: candidate output missing")); continue; };
        let action = string(candidate, "action");
        let p_value = candidate.get("predicted_exceedance_probability");
        let s_value = candidate.get("gpd_scale");
        let x_value = candidate.get("gpd_shape");
        let reason_value = candidate.get("abstention_reason");
        let reason = reason_value.and_then(Value::as_str);
        let p_forecast = p_value.and_then(Value::as_f64);
        let scale = s_value.and_then(Value::as_f64);
        let shape = x_value.and_then(Value::as_f64);
        let mandatory = matches!(qualification.as_str(), "missing_critical_input" | "conflicting_sources" | "unsupported_distribution_shift");
        match (mandatory, action) {
            (true, Some("abstain")) => {
                if !p_value.is_some_and(Value::is_null) || !s_value.is_some_and(Value::is_null)
                    || !x_value.is_some_and(Value::is_null) || reason != Some(qualification.as_str())
                { errors.push(format!("{case_id}: mandatory abstention requires null model fields and matching reason")); }
                else { mandatory_abstentions += 1; *mandatory_reasons.entry(qualification.clone()).or_default() += 1; }
            }
            (true, Some("forecast")) => errors.push(format!("{case_id}: forecast emitted for mandatory-abstention state")),
            (true, _) => errors.push(format!("{case_id}: invalid mandatory-abstention action")),
            (false, Some("forecast")) => {
                if qualification != "qualified" || p_forecast.is_none_or(|v| !v.is_finite() || !(0.0..=1.0).contains(&v))
                    || scale.is_none_or(|v| !v.is_finite() || v <= 0.0)
                    || shape.is_none_or(|v| !v.is_finite() || !(-0.5..=1.0).contains(&v))
                    || !reason_value.is_some_and(Value::is_null)
                { errors.push(format!("{case_id}: forecast requires qualified inputs, probability [0,1], positive GPD scale and explicit null reason")); }
            }
            (false, Some("abstain")) => {
                if qualification != "qualified" || !p_value.is_some_and(Value::is_null)
                    || !s_value.is_some_and(Value::is_null) || !x_value.is_some_and(Value::is_null)
                    || reason != Some("model_uncertainty")
                { errors.push(format!("{case_id}: qualified abstention requires null model fields and model_uncertainty reason")); }
                else { optional_abstentions += 1; }
            }
            (false, _) => errors.push(format!("{case_id}: action must be forecast or abstain")),
        }
        if action == Some("forecast") && p_forecast.is_some_and(|v| v.is_finite() && (0.0..=1.0).contains(&v))
            && scale.is_some_and(|v| v.is_finite() && v > 0.0)
            && shape.is_some_and(|v| v.is_finite() && (-0.5..=1.0).contains(&v))
        {
            parsed.push(Case { regime, severity, weight: p/q, forecast: true,
                probability: p_forecast, scale, shape });
        } else if action == Some("abstain") && p_value.is_some_and(Value::is_null)
            && s_value.is_some_and(Value::is_null) && x_value.is_some_and(Value::is_null)
            && reason.is_some()
        {
            parsed.push(Case { regime, severity, weight: p/q, forecast: false,
                probability: None, scale: None, shape: None });
        }
    }

    let expected_weight = number(design, "expected_total_weight_per_regime");
    if expected_weight.is_none_or(|value| !value.is_finite() || value <= 0.0) {
        errors.push("expected_total_weight_per_regime must be finite and positive".to_owned());
    }
    let expected_weight = expected_weight.unwrap_or(32.0);
    for regime in &regime_order {
        let counts = stratum_counts.get(regime);
        let n: usize = counts.map(|v| v.values().sum()).unwrap_or(0);
        if n == 0 { errors.push(format!("{regime}: empty regime cohort")); continue; }
        if let Some(counts) = counts {
            for (stratum, (q, _)) in &stratum_design {
                let observed_q = counts.get(stratum).copied().unwrap_or(0) as f64 / n as f64;
                if (observed_q - q).abs() > 1e-12 {
                    errors.push(format!("{regime}/{stratum}: allocation ratio differs from proposal probability"));
                }
            }
        }
        let all_weight: f64 = rows.iter().filter(|r| string(r, "regime_id")==Some(regime.as_str()))
            .filter_map(|r| {
                let q=number(r,"proposal_stratum_probability")?;
                let p=number(r,"target_stratum_probability")?;
                (q>0.0).then_some(p/q)
            }).sum();
        if (all_weight-expected_weight).abs()>1e-9 {
            errors.push(format!("{regime}: full design weights do not sum to declared regime weight"));
        }
        if !parsed.iter().any(|c| c.regime==*regime && c.forecast) {
            errors.push(format!("{regime}: no forecasts issued"));
        }
    }
    if !parsed.iter().any(|c| c.forecast) { errors.push("all-abstention result is invalid".to_owned()); }
    if !errors.is_empty() { return Err(errors); }

    let overall = summarize(&parsed, threshold, reference_p, &edges, &q_levels);
    let per_regime: Vec<Value> = regime_order.iter().map(|regime| {
        let rows: Vec<Case> = parsed.iter().filter(|c| c.regime==*regime).cloned().collect();
        let mut summary = summarize(&rows, threshold, reference_p, &edges, &q_levels);
        if let Some(map)=summary.as_object_mut() { map.insert("regime_id".to_owned(), json!(regime)); }
        summary
    }).collect();
    let in_distribution = per_regime.iter().find(|v|string(v,"regime_id")==Some("in_distribution"));
    let reference_brier = in_distribution.and_then(|v| number(v,"brier_score"));
    let deltas: Vec<Value> = per_regime.iter().filter_map(|v| {
        let regime=string(v,"regime_id")?;
        if regime=="in_distribution" { return None; }
        Some(json!({"regime_id":regime,"brier_delta_vs_in_distribution":
            reference_brier.zip(number(v,"brier_score")).map(|(a,b)|b-a)}))
    }).collect();
    let worst = per_regime.iter().filter_map(|v|number(v,"brier_score")).fold(f64::NEG_INFINITY,f64::max);

    Ok(json!({
        "benchmark_id":"PRAXIS-NRES-010-TAIL-SHIFT-ROBUSTNESS-001",
        "interpretation":"synthetic tail/shift evaluation only; importance weights apply only to the declared representative within-stratum design",
        "total_cases":rows.len(),
        "forecast_cases":parsed.iter().filter(|c|c.forecast).count(),
        "mandatory_abstentions":mandatory_abstentions,
        "optional_abstentions":optional_abstentions,
        "mandatory_abstention_by_reason":mandatory_reasons,
        "overall":overall,
        "per_regime":per_regime,
        "worst_slice_brier_score":worst.is_finite().then_some(worst),
        "shifted_brier_deltas":deltas,
        "aggregation_policy":"separate occurrence, tail severity, shift and coverage metrics; no composite resilience/personal score"
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate_tail_shift_robustness;
    use serde_json::{json, Value};

    const FIXTURE:&str=include_str!("../../../examples/courses/civic-resilience-tail-shift-robustness-fixture-001.json");
    const CONFORMANCE:&str=include_str!("../../../examples/courses/civic-resilience-tail-shift-robustness-conformance-001.json");
    fn fixture()->Value { serde_json::from_str(FIXTURE).expect("NRES-010 JSON valid") }
    fn first_forecast(v:&mut Value)->&mut Value {
        v["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|c|c["candidate_output"]["action"]=="forecast").unwrap()
    }

    #[test] fn derives_weighted_tail_and_shift_metrics() {
        let r=evaluate_tail_shift_robustness(&fixture()).expect("fixture valid");
        assert_eq!(r["total_cases"].as_u64(),Some(128));
        assert_eq!(r["forecast_cases"].as_u64(),Some(123));
        assert_eq!(r["mandatory_abstentions"].as_u64(),Some(4));
        assert_eq!(r["optional_abstentions"].as_u64(),Some(1));
        assert!((r["overall"]["brier_score"].as_f64().unwrap()-0.07849683436400443).abs()<1e-12);
        assert!((r["overall"]["weighted_tail_excess_pit_ks_distance"].as_f64().unwrap()-0.5041045259197009).abs()<1e-12);
    }

    #[test] fn matches_published_overall_and_regime_goldens() {
        let r=evaluate_tail_shift_robustness(&fixture()).unwrap();
        let c:Value=serde_json::from_str(CONFORMANCE).unwrap();
        let e=&c["expected_metrics"];
        let tol=c["conformance_targets"]["golden_metric_absolute_tolerance"].as_f64().unwrap_or(1e-12);
        for k in ["total_cases","forecast_cases","mandatory_abstentions","optional_abstentions"] { assert_eq!(r[k],e[k]); }
        for k in ["brier_score","log_loss","climatology_brier_score","brier_skill_score","weighted_tail_excess_pit_ks_distance","weighted_tail_excess_log_loss","tail_occurrence_calibration_ratio","forecast_effective_sample_size","tail_event_effective_sample_size"] {
            assert!((r["overall"][k].as_f64().unwrap()-e["overall"][k].as_f64().unwrap()).abs()<=tol,"{k}");
        }
        assert_eq!(r["per_regime"].as_array().unwrap().len(),e["per_regime"].as_array().unwrap().len());
        for (a,b) in r["per_regime"].as_array().unwrap().iter().zip(e["per_regime"].as_array().unwrap()) {
            assert_eq!(a["regime_id"],b["regime_id"]);
            for k in ["brier_score","log_loss","brier_skill_score","weighted_observed_exceedance_rate","weighted_mean_predicted_exceedance_probability","weighted_tail_excess_pit_ks_distance","weighted_tail_excess_log_loss","forecast_effective_sample_size","tail_event_effective_sample_size","population_forecast_coverage","weighted_forecast_coverage"] {
                assert!((a[k].as_f64().unwrap()-b[k].as_f64().unwrap()).abs()<=tol,"{k}");
            }
            for (aq,bq) in a["conditional_tail_quantile_coverage"].as_array().unwrap().iter().zip(b["conditional_tail_quantile_coverage"].as_array().unwrap()) {
                for k in ["nominal","observed_weighted_coverage"] { assert!((aq[k].as_f64().unwrap()-bq[k].as_f64().unwrap()).abs()<=tol); }
            }
        }
        assert_eq!(r["mandatory_abstention_by_reason"],e["mandatory_abstention_by_reason"]);
        assert!((r["worst_slice_brier_score"].as_f64().unwrap()-e["worst_slice_brier_score"].as_f64().unwrap()).abs()<=tol);
    }


    #[test] fn rejects_abstention_without_reason() {
        let mut v=fixture();
        let row=v["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["candidate_output"]["action"]=="abstain").unwrap();
        row["candidate_output"].as_object_mut().unwrap().remove("abstention_reason");
        assert!(evaluate_tail_shift_robustness(&v).is_err());
    }

    #[test] fn threshold_equality_is_not_an_exceedance() {
        let mut v=fixture();
        let row=v["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["evaluator"]["observed_normalized_service_deficit_hours"].as_f64().unwrap()>48.0
                && row["candidate_output"]["action"]=="forecast").unwrap();
        row["evaluator"]["observed_normalized_service_deficit_hours"]=json!(48.0);
        let report=evaluate_tail_shift_robustness(&v).expect("threshold equality remains a valid non-exceedance");
        assert_eq!(report["overall"]["observed_tail_events"].as_u64().unwrap(),24);
    }

    #[test] fn rejects_probability_above_one() { let mut v=fixture(); first_forecast(&mut v)["candidate_output"]["predicted_exceedance_probability"]=json!(1.01); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_negative_probability() { let mut v=fixture(); first_forecast(&mut v)["candidate_output"]["predicted_exceedance_probability"]=json!(-0.01); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_missing_forecast_probability() { let mut v=fixture(); first_forecast(&mut v)["candidate_output"]["predicted_exceedance_probability"]=Value::Null; assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_probability_on_abstention() { let mut v=fixture(); let c=v["evaluation_cases"].as_array_mut().unwrap().iter_mut().find(|c|c["candidate_output"]["action"]=="abstain").unwrap(); c["candidate_output"]["predicted_exceedance_probability"]=json!(0.2); assert!(evaluate_tail_shift_robustness(&v).is_err()); }

    #[test] fn rejects_forecast_on_missing_input() {
        let mut v=fixture(); let c=v["evaluation_cases"].as_array_mut().unwrap().iter_mut().find(|c|c["evaluator"]["qualification_state"]=="missing_critical_input").unwrap();
        c["candidate_output"]["action"]=json!("forecast"); c["candidate_output"]["predicted_exceedance_probability"]=json!(0.1);
        c["candidate_output"]["gpd_scale"]=json!(20.0); c["candidate_output"]["gpd_shape"]=json!(0.1); c["candidate_output"]["abstention_reason"]=Value::Null;
        assert!(evaluate_tail_shift_robustness(&v).is_err());
    }
    #[test] fn rejects_forecast_on_conflicting_sources() { let mut v=fixture(); let c=v["evaluation_cases"].as_array_mut().unwrap().iter_mut().find(|c|c["evaluator"]["qualification_state"]=="conflicting_sources").unwrap(); c["candidate_output"]["action"]=json!("forecast"); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_non_positive_gpd_scale() { let mut v=fixture(); first_forecast(&mut v)["candidate_output"]["gpd_scale"]=json!(0.0); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_non_finite_gpd_shape() { let mut v=fixture(); first_forecast(&mut v)["candidate_output"]["gpd_shape"]=json!("NaN"); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_negative_observed_severity() { let mut v=fixture(); v["evaluation_cases"][0]["evaluator"]["observed_normalized_service_deficit_hours"]=json!(-1.0); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_zero_proposal_probability() { let mut v=fixture(); v["sampling_design"]["strata"][0]["proposal_probability"]=json!(0.0); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_negative_target_probability() { let mut v=fixture(); v["sampling_design"]["strata"][0]["target_probability"]=json!(-0.1); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_stratum_design_drift() { let mut v=fixture(); v["evaluation_cases"][0]["target_stratum_probability"]=json!(0.5); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_duplicate_case_id() { let mut v=fixture(); let id=v["evaluation_cases"][0]["case_id"].clone(); v["evaluation_cases"][1]["case_id"]=id; assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_composite_score() { let mut v=fixture(); v["overall_resilience_score"]=json!(0.9); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_missing_regime_cohort() { let mut v=fixture(); v["evaluation_cases"].as_array_mut().unwrap().retain(|c|c["regime_id"]!="temporal_shift"); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_horizon_drift() { let mut v=fixture(); v["evaluation_cases"][0]["forecast_horizon_hours"]=json!(24); assert!(evaluate_tail_shift_robustness(&v).is_err()); }
    #[test] fn rejects_all_abstention() {
        let mut v=fixture(); for c in v["evaluation_cases"].as_array_mut().unwrap() {
            c["candidate_output"]["action"]=json!("abstain"); c["candidate_output"]["predicted_exceedance_probability"]=Value::Null;
            c["candidate_output"]["gpd_scale"]=Value::Null; c["candidate_output"]["gpd_shape"]=Value::Null;
            let reason=c["evaluator"]["qualification_state"].as_str().filter(|s|*s!="qualified").unwrap_or("model_uncertainty").to_owned();
            c["candidate_output"]["abstention_reason"]=json!(reason);
        } assert!(evaluate_tail_shift_robustness(&v).is_err());
    }


    #[test] fn rejects_optional_abstention_reason_for_unqualified_input() {
        let mut v=fixture();
        let row=v["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["evaluator"]["qualification_state"]=="missing_critical_input").unwrap();
        row["candidate_output"]["abstention_reason"]=json!("model_uncertainty");
        assert!(evaluate_tail_shift_robustness(&v).is_err());
    }

    #[test] fn gpd_exponential_limit_quantile_inverts_cdf() {
        let q=0.9;
        let x=super::gpd_quantile(q,20.0,0.0).unwrap();
        let p=super::gpd_cdf(x,20.0,0.0).unwrap();
        assert!((p-q).abs()<1e-12);
    }

    #[test] fn rejects_sampling_target_mass_not_normalized() {
        let mut v=fixture();
        v["sampling_design"]["strata"][0]["target_probability"]=json!(0.5);
        for row in v["evaluation_cases"].as_array_mut().unwrap() {
            if row["sampling_stratum"]=="routine" {
                row["target_stratum_probability"]=json!(0.5);
            }
        }
        assert!(evaluate_tail_shift_robustness(&v).is_err());
    }

    #[test] fn reports_negative_shape_support_violation_without_clipping() {
        let mut v=fixture();
        let row=v["evaluation_cases"].as_array_mut().unwrap().iter_mut()
            .find(|row| row["evaluator"]["observed_normalized_service_deficit_hours"].as_f64().unwrap()-48.0>40.0
                && row["candidate_output"]["action"]=="forecast").unwrap();
        row["candidate_output"]["gpd_shape"]=json!(-0.5);
        row["candidate_output"]["gpd_scale"]=json!(20.0);
        let report=evaluate_tail_shift_robustness(&v).expect("a zero-support forecast is scored, not schema-rejected");
        assert!(report["overall"]["zero_support_tail_excesses"].as_u64().unwrap()>0);
        assert!(report["overall"]["weighted_tail_excess_log_loss"].is_null());
    }

    #[test] fn preserves_zero_probability_exceedance_as_infinite_log_loss() {
        let mut v=fixture(); let c=v["evaluation_cases"].as_array_mut().unwrap().iter_mut().find(|c|c["evaluator"]["observed_normalized_service_deficit_hours"].as_f64().unwrap()>48.0 && c["candidate_output"]["action"]=="forecast").unwrap();
        c["candidate_output"]["predicted_exceedance_probability"]=json!(0.0);
        let r=evaluate_tail_shift_robustness(&v).expect("zero probability is invalid performance, not malformed schema");
        assert!(r["overall"]["zero_probability_exceedances"].as_u64().unwrap()>0);
        assert!(r["overall"]["log_loss"].is_null());
    }
}
