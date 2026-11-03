//! Matched-run payment outcome benchmark and non-comparable market context report.
//!
//! The binary is effects-disabled. It never contacts payment providers or
//! processes customer data. Public market aggregates are kept out of performance
//! rankings unless their measurement windows and definitions exactly match.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::cmp::Ordering;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const MARKET_DEFAULT: &str = include_str!("../market-snapshots-v1.json");
const MARKET_SCHEMA: &str = include_str!("../market-snapshots-v1.schema.json");
const RUNS_DEFAULT: &str = include_str!("../matched-runs-v1.json");
const RUNS_SCHEMA: &str = include_str!("../matched-runs-v1.schema.json");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GateStatus {
    QualifiedForComparison,
    Disqualified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParetoResult {
    CandidateDominates,
    BaselineDominates,
    TradeoffOrIncomparable,
    CandidateDisqualified,
    BaselineDisqualified,
}

fn object_field<'a>(value: &'a Value, key: &str) -> Option<&'a Value> {
    value.get(key)
}

fn string_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_field(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn validate_schema(schema_text: &str, instance_text: &str) -> Result<Value, Vec<String>> {
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| vec![format!("schema parse failed: {error}")])?;
    let instance: Value = serde_json::from_str(instance_text)
        .map_err(|error| vec![format!("instance parse failed: {error}")])?;
    if !jsonschema::meta::is_valid(&schema) {
        return Err(vec![
            "schema is invalid against its Draft 2020-12 meta-schema".into(),
        ]);
    }
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| vec![format!("schema compilation failed: {error}")])?;
    let errors: Vec<String> = validator
        .iter_errors(&instance)
        .map(|error| format!("{}: {error}", error.instance_path()))
        .collect();
    if errors.is_empty() {
        Ok(instance)
    } else {
        Err(errors)
    }
}

fn public_snapshot_comparability(left: &Value, right: &Value) -> bool {
    let left_period = object_field(left, "period");
    let right_period = object_field(right, "period");
    left_period == right_period
        && string_field(left, "metric_definition_id") == string_field(right, "metric_definition_id")
        && string_field(left, "population_scope") == string_field(right, "population_scope")
        && object_field(left, "transaction_count").and_then(|v| string_field(v, "unit"))
            == object_field(right, "transaction_count").and_then(|v| string_field(v, "unit"))
        && object_field(left, "transaction_value").and_then(|v| string_field(v, "currency"))
            == object_field(right, "transaction_value").and_then(|v| string_field(v, "currency"))
}

fn snapshot_report(market: &Value) -> Value {
    let snapshots = market["snapshots"].as_array().cloned().unwrap_or_default();
    let mut pairs = Vec::new();
    for left_index in 0..snapshots.len() {
        for right_index in (left_index + 1)..snapshots.len() {
            let left = &snapshots[left_index];
            let right = &snapshots[right_index];
            let comparable = public_snapshot_comparability(left, right);
            pairs.push(json!({
                "left": string_field(left, "id"),
                "right": string_field(right, "id"),
                "classification": if comparable {
                    "definition_and_window_match_only_not_a_performance_verdict"
                } else {
                    "context_only_not_rankable"
                },
                "rankable": false,
                "reason": if comparable {
                    "Even equal published measurement definitions are not a matched task cohort; provider/product performance still requires same-input testing."
                } else {
                    "Different measurement window, population scope, metric definition, unit or currency prevents direct ranking."
                }
            }));
        }
    }
    json!({
        "snapshot_count": snapshots.len(),
        "snapshots": snapshots.iter().map(|item| json!({
            "id": item["id"],
            "system": item["system"],
            "jurisdiction": item["jurisdiction"],
            "period": item["period"],
            "transaction_count": item["transaction_count"],
            "transaction_value": item["transaction_value"],
            "active_users": item.get("active_users").cloned().unwrap_or(Value::Null),
            "institution_coverage": item.get("institution_coverage").cloned().unwrap_or(Value::Null),
            "limitations": item["limitations"]
        })).collect::<Vec<_>>(),
        "pairwise": pairs,
        "raw_volume_leaderboard": null,
        "claim_ceiling": "public system-level descriptive facts only; no direct system performance ranking or Mycelix capability claim"
    })
}

fn gate_status(metrics: &Value) -> GateStatus {
    let hard_gates = [
        "duplicate_effects",
        "false_finality_promotions",
        "unexplained_reconciliation_deltas",
        "unauthorised_secondary_data_uses",
        "essential_service_denials_for_unrelated_consent_refusal",
    ];
    if hard_gates
        .iter()
        .any(|key| u64_field(metrics, key) != Some(0))
    {
        GateStatus::Disqualified
    } else {
        GateStatus::QualifiedForComparison
    }
}

/// Wilson score interval for a binomial proportion (95% confidence).
/// Floating point is used only for statistical reporting; money remains integer.
fn wilson_interval(successes: u64, trials: u64) -> Option<(f64, f64)> {
    if trials == 0 || successes > trials {
        return None;
    }
    let n = trials as f64;
    let p = successes as f64 / n;
    let z = 1.959_963_984_540_054_f64;
    let z2 = z * z;
    let denominator = 1.0 + z2 / n;
    let centre = (p + z2 / (2.0 * n)) / denominator;
    let margin = z * ((p * (1.0 - p) / n + z2 / (4.0 * n * n)).sqrt()) / denominator;
    Some(((centre - margin).max(0.0), (centre + margin).min(1.0)))
}

fn fee_component_total(metrics: &Value) -> Option<u64> {
    [
        "payer_fee_minor_units",
        "merchant_fee_minor_units",
        "fx_cost_minor_units",
        "cashout_fee_minor_units",
        "returns_disputes_cost_minor_units",
        "provider_network_cost_minor_units",
    ]
    .into_iter()
    .try_fold(0_u64, |total, key| {
        total.checked_add(u64_field(metrics, key)?)
    })
}

fn cmp_fraction(left_num: u64, left_den: u64, right_num: u64, right_den: u64) -> Ordering {
    (left_num as u128 * right_den as u128).cmp(&(right_num as u128 * left_den as u128))
}

fn cmp_cost_per_delivery(left: &Value, right: &Value) -> Option<Ordering> {
    let left_currency = string_field(left, "cost_currency")?;
    let right_currency = string_field(right, "cost_currency")?;
    if left_currency != right_currency {
        return None;
    }
    let left_cost = u64_field(left, "total_all_in_cost_minor_units")?;
    let right_cost = u64_field(right, "total_all_in_cost_minor_units")?;
    let left_delivered = u64_field(left, "recipient_available")?;
    let right_delivered = u64_field(right, "recipient_available")?;
    if left_delivered == 0 || right_delivered == 0 {
        return None;
    }
    Some(cmp_fraction(
        left_cost,
        left_delivered,
        right_cost,
        right_delivered,
    ))
}

fn run_dominates(candidate: &Value, baseline: &Value) -> Option<bool> {
    let candidate_metrics = &candidate["metrics"];
    let baseline_metrics = &baseline["metrics"];
    if gate_status(candidate_metrics) != GateStatus::QualifiedForComparison
        || gate_status(baseline_metrics) != GateStatus::QualifiedForComparison
    {
        return None;
    }

    let candidate_attempts = u64_field(candidate_metrics, "eligible_attempts")?;
    let baseline_attempts = u64_field(baseline_metrics, "eligible_attempts")?;
    let candidate_success = u64_field(candidate_metrics, "recipient_available")?;
    let baseline_success = u64_field(baseline_metrics, "recipient_available")?;
    let candidate_user_attempts = u64_field(candidate_metrics, "user_task_attempts")?;
    let baseline_user_attempts = u64_field(baseline_metrics, "user_task_attempts")?;
    let candidate_user_success = u64_field(candidate_metrics, "user_task_successes")?;
    let baseline_user_success = u64_field(baseline_metrics, "user_task_successes")?;
    let candidate_merchants = u64_field(candidate_metrics, "active_merchants_in_cohort")?;
    let baseline_merchants = u64_field(baseline_metrics, "active_merchants_in_cohort")?;
    let candidate_merchant_denominator =
        u64_field(candidate_metrics, "eligible_merchants_in_cohort")?;
    let baseline_merchant_denominator =
        u64_field(baseline_metrics, "eligible_merchants_in_cohort")?;

    let completion = cmp_fraction(
        candidate_success,
        candidate_attempts,
        baseline_success,
        baseline_attempts,
    );
    let cost = cmp_cost_per_delivery(candidate_metrics, baseline_metrics)?;
    let latency_p50 = u64_field(candidate_metrics, "p50_initiation_to_available_ms")?.cmp(
        &u64_field(baseline_metrics, "p50_initiation_to_available_ms")?,
    );
    let latency_p95 = u64_field(candidate_metrics, "p95_initiation_to_available_ms")?.cmp(
        &u64_field(baseline_metrics, "p95_initiation_to_available_ms")?,
    );
    let latency_p99 = u64_field(candidate_metrics, "p99_initiation_to_available_ms")?.cmp(
        &u64_field(baseline_metrics, "p99_initiation_to_available_ms")?,
    );
    let operator = u64_field(candidate_metrics, "operator_minutes_per_1000_attempts")?.cmp(
        &u64_field(baseline_metrics, "operator_minutes_per_1000_attempts")?,
    );
    let usability = cmp_fraction(
        candidate_user_success,
        candidate_user_attempts,
        baseline_user_success,
        baseline_user_attempts,
    );
    let merchant_coverage = cmp_fraction(
        candidate_merchants,
        candidate_merchant_denominator,
        baseline_merchants,
        baseline_merchant_denominator,
    );
    let unknown_outcome_rate = cmp_fraction(
        u64_field(candidate_metrics, "unknown_outcomes")?,
        candidate_attempts,
        u64_field(baseline_metrics, "unknown_outcomes")?,
        baseline_attempts,
    );
    let terminal_failure_rate = cmp_fraction(
        u64_field(candidate_metrics, "terminal_failures")?,
        candidate_attempts,
        u64_field(baseline_metrics, "terminal_failures")?,
        baseline_attempts,
    );
    let return_rate = cmp_fraction(
        u64_field(candidate_metrics, "returned_or_reversed")?,
        candidate_success,
        u64_field(baseline_metrics, "returned_or_reversed")?,
        baseline_success,
    );

    // Lower is better for cost, latency, operator burden, unknowns, failures
    // and returns; higher is better for successful delivery, user task success
    // and reachable merchants. A trade-off is not represented as a win.
    let not_worse = completion != Ordering::Less
        && cost != Ordering::Greater
        && latency_p50 != Ordering::Greater
        && latency_p95 != Ordering::Greater
        && latency_p99 != Ordering::Greater
        && operator != Ordering::Greater
        && usability != Ordering::Less
        && merchant_coverage != Ordering::Less
        && unknown_outcome_rate != Ordering::Greater
        && terminal_failure_rate != Ordering::Greater
        && return_rate != Ordering::Greater;
    let strictly_better = completion == Ordering::Greater
        || cost == Ordering::Less
        || latency_p50 == Ordering::Less
        || latency_p95 == Ordering::Less
        || latency_p99 == Ordering::Less
        || operator == Ordering::Less
        || usability == Ordering::Greater
        || merchant_coverage == Ordering::Greater
        || unknown_outcome_rate == Ordering::Less
        || terminal_failure_rate == Ordering::Less
        || return_rate == Ordering::Less;
    Some(not_worse && strictly_better)
}

fn pareto_result(candidate: &Value, baseline: &Value) -> ParetoResult {
    let candidate_gate = gate_status(&candidate["metrics"]);
    let baseline_gate = gate_status(&baseline["metrics"]);
    if candidate_gate == GateStatus::Disqualified {
        return ParetoResult::CandidateDisqualified;
    }
    if baseline_gate == GateStatus::Disqualified {
        return ParetoResult::BaselineDisqualified;
    }
    match (
        run_dominates(candidate, baseline),
        run_dominates(baseline, candidate),
    ) {
        (Some(true), Some(false)) => ParetoResult::CandidateDominates,
        (Some(false), Some(true)) => ParetoResult::BaselineDominates,
        _ => ParetoResult::TradeoffOrIncomparable,
    }
}

fn result_name(result: ParetoResult) -> &'static str {
    match result {
        ParetoResult::CandidateDominates => "candidate_pareto_dominates_baseline",
        ParetoResult::BaselineDominates => "baseline_pareto_dominates_candidate",
        ParetoResult::TradeoffOrIncomparable => "tradeoff_or_incomparable",
        ParetoResult::CandidateDisqualified => "candidate_disqualified_by_hard_gate",
        ParetoResult::BaselineDisqualified => "baseline_disqualified_by_hard_gate",
    }
}

fn matched_run_report(runs: &Value) -> Result<Value, String> {
    let cohort = &runs["cohort"];
    let run_list = runs["runs"]
        .as_array()
        .ok_or_else(|| "runs: missing array".to_string())?;
    let candidate_count = run_list
        .iter()
        .filter(|run| string_field(run, "provider_class") == Some("mycelix_candidate"))
        .count();
    let baseline_count = run_list.len().saturating_sub(candidate_count);
    if candidate_count != 1 || baseline_count == 0 {
        return Err("matched runs require exactly one candidate and at least one baseline".into());
    }

    let mut ids = std::collections::BTreeSet::new();
    let mut common_denominators: Option<(u64, u64, u64)> = None;
    for (index, run) in run_list.iter().enumerate() {
        let id =
            string_field(run, "run_id").ok_or_else(|| format!("runs[{index}]: missing run_id"))?;
        if !ids.insert(id.to_owned()) {
            return Err(format!("runs[{index}]: duplicate run_id {id}"));
        }
        if object_field(run, "cohort").is_some_and(|run_cohort| run_cohort != cohort) {
            return Err(format!(
                "runs[{index}]: run cohort differs from the frozen matched cohort"
            ));
        }
        let metrics = &run["metrics"];
        let attempts = u64_field(metrics, "eligible_attempts")
            .ok_or_else(|| format!("runs[{index}]: eligible_attempts missing"))?;
        let completed = u64_field(metrics, "recipient_available")
            .ok_or_else(|| format!("runs[{index}]: recipient_available missing"))?;
        let user_attempts = u64_field(metrics, "user_task_attempts")
            .ok_or_else(|| format!("runs[{index}]: user_task_attempts missing"))?;
        let user_successes = u64_field(metrics, "user_task_successes")
            .ok_or_else(|| format!("runs[{index}]: user_task_successes missing"))?;
        let merchant_eligible = u64_field(metrics, "eligible_merchants_in_cohort")
            .ok_or_else(|| format!("runs[{index}]: eligible_merchants_in_cohort missing"))?;
        let merchant_active = u64_field(metrics, "active_merchants_in_cohort")
            .ok_or_else(|| format!("runs[{index}]: active_merchants_in_cohort missing"))?;
        let terminal_failures = u64_field(metrics, "terminal_failures")
            .ok_or_else(|| format!("runs[{index}]: terminal_failures missing"))?;
        let unknown_outcomes = u64_field(metrics, "unknown_outcomes")
            .ok_or_else(|| format!("runs[{index}]: unknown_outcomes missing"))?;
        let returned_or_reversed = u64_field(metrics, "returned_or_reversed")
            .ok_or_else(|| format!("runs[{index}]: returned_or_reversed missing"))?;
        if completed > attempts
            || user_successes > user_attempts
            || merchant_active > merchant_eligible
            || returned_or_reversed > completed
            || completed
                .saturating_add(terminal_failures)
                .saturating_add(unknown_outcomes)
                > attempts
        {
            return Err(format!(
                "runs[{index}]: outcome counts exceed the eligible attempt, delivery or return denominators"
            ));
        }
        if string_field(metrics, "cost_currency") != string_field(cohort, "currency") {
            return Err(format!(
                "runs[{index}]: cost currency differs from cohort currency"
            ));
        }
        let fee_components = fee_component_total(metrics)
            .ok_or_else(|| format!("runs[{index}]: missing fee component or integer overflow"))?;
        if fee_components != u64_field(metrics, "total_all_in_cost_minor_units").unwrap_or(u64::MAX)
        {
            return Err(format!(
                "runs[{index}]: all-in cost must equal the sum of payer, merchant, FX, cash-out, returns/disputes and provider-network cost components"
            ));
        }
        let p50 = u64_field(metrics, "p50_initiation_to_available_ms")
            .ok_or_else(|| format!("runs[{index}]: p50 latency missing"))?;
        let p95 = u64_field(metrics, "p95_initiation_to_available_ms")
            .ok_or_else(|| format!("runs[{index}]: p95 latency missing"))?;
        let p99 = u64_field(metrics, "p99_initiation_to_available_ms")
            .ok_or_else(|| format!("runs[{index}]: p99 latency missing"))?;
        if !(p50 <= p95 && p95 <= p99) {
            return Err(format!(
                "runs[{index}]: latency quantiles must satisfy p50 <= p95 <= p99"
            ));
        }
        let denominators = (attempts, user_attempts, merchant_eligible);
        if common_denominators.is_some_and(|expected| expected != denominators) {
            return Err(format!(
                "runs[{index}]: task-attempt and eligible-merchant denominators differ from the matched cohort"
            ));
        }
        common_denominators = Some(denominators);
    }

    let baselines: Vec<&Value> = run_list
        .iter()
        .filter(|run| string_field(run, "provider_class") != Some("mycelix_candidate"))
        .collect();
    let candidate = run_list
        .iter()
        .find(|run| string_field(run, "provider_class") == Some("mycelix_candidate"));

    let comparisons = if let Some(candidate) = candidate {
        baselines.iter().map(|baseline| {
            json!({
                "candidate": string_field(candidate, "run_id"),
                "baseline": string_field(baseline, "run_id"),
                "result": result_name(pareto_result(candidate, baseline)),
                "hard_gate": if gate_status(&candidate["metrics"]) == GateStatus::QualifiedForComparison {"qualified_for_comparison"} else {"disqualified"},
                "candidate_completion_rate": format!("{}/{}", u64_field(&candidate["metrics"], "recipient_available").unwrap_or(0), u64_field(&candidate["metrics"], "eligible_attempts").unwrap_or(0)),
                "baseline_completion_rate": format!("{}/{}", u64_field(&baseline["metrics"], "recipient_available").unwrap_or(0), u64_field(&baseline["metrics"], "eligible_attempts").unwrap_or(0)),
                "candidate_cost_per_delivery_minor": format!("{}/{}", u64_field(&candidate["metrics"], "total_all_in_cost_minor_units").unwrap_or(0), u64_field(&candidate["metrics"], "recipient_available").unwrap_or(0)),
                "baseline_cost_per_delivery_minor": format!("{}/{}", u64_field(&baseline["metrics"], "total_all_in_cost_minor_units").unwrap_or(0), u64_field(&baseline["metrics"], "recipient_available").unwrap_or(0))
            })
        }).collect::<Vec<_>>()
    } else {
        Vec::new()
    };

    Ok(json!({
        "cohort": cohort,
        "run_count": run_list.len(),
        "runs": run_list.iter().map(|run| json!({
            "run_id": run["run_id"],
            "provider_class": run["provider_class"],
            "hard_gate": if gate_status(&run["metrics"]) == GateStatus::QualifiedForComparison {"qualified_for_comparison"} else {"disqualified"},
            "completion_rate_95_wilson_ci": wilson_interval(
                u64_field(&run["metrics"], "recipient_available").unwrap_or(0),
                u64_field(&run["metrics"], "eligible_attempts").unwrap_or(0)
            ).map(|(lower, upper)| json!({"lower": lower, "upper": upper})),
            "user_task_success_rate_95_wilson_ci": wilson_interval(
                u64_field(&run["metrics"], "user_task_successes").unwrap_or(0),
                u64_field(&run["metrics"], "user_task_attempts").unwrap_or(0)
            ).map(|(lower, upper)| json!({"lower": lower, "upper": upper})),
            "cost_per_recipient_available_minor_units": format!(
                "{}/{}",
                u64_field(&run["metrics"], "total_all_in_cost_minor_units").unwrap_or(0),
                u64_field(&run["metrics"], "recipient_available").unwrap_or(0)
            ),
            "metrics": run["metrics"]
        })).collect::<Vec<_>>(),
        "comparisons": comparisons,
        "latency_quantiles": ["p50", "p95", "p99"],
        "cost_reconciliation": "all-in cost equals exact integer sum of fee/cost components",
        "confidence_intervals": "Wilson 95% intervals for completion and user task success",
        "claim_ceiling": "synthetic matched-run illustration only; no real provider or Mycelix performance claim"
    }))
}

fn run_self_tests(
    market: &Value,
    runs: &Value,
    market_schema: &str,
    runs_schema: &str,
) -> Result<usize, String> {
    if validate_schema(market_schema, &market.to_string()).is_err()
        || validate_schema(runs_schema, &runs.to_string()).is_err()
    {
        return Err("self-test baseline fixtures must validate against both schemas".into());
    }
    let mut count = 0usize;

    let snapshots = market["snapshots"].as_array().ok_or("snapshots missing")?;
    for i in 0..snapshots.len() {
        for j in (i + 1)..snapshots.len() {
            if public_snapshot_comparability(&snapshots[i], &snapshots[j]) {
                return Err(
                    "public national snapshots were incorrectly treated as directly comparable"
                        .into(),
                );
            }
            count += 1;
        }
    }

    let baseline = &runs["runs"][0];
    let candidate = &runs["runs"][2];
    if pareto_result(candidate, baseline) != ParetoResult::TradeoffOrIncomparable {
        return Err(
            "mixed illustrative fixture must report a trade-off, not a manufactured candidate win"
                .into(),
        );
    }
    count += 1;

    let mut mutated = runs.clone();
    mutated["runs"][2]["metrics"]["duplicate_effects"] = json!(1);
    if pareto_result(&mutated["runs"][2], baseline) != ParetoResult::CandidateDisqualified {
        return Err("candidate duplicate-effect mutation was not disqualified".into());
    }
    count += 1;

    let mut mutated = runs.clone();
    mutated["runs"][2]["metrics"]["false_finality_promotions"] = json!(1);
    if pareto_result(&mutated["runs"][2], baseline) != ParetoResult::CandidateDisqualified {
        return Err("candidate false-finality mutation was not disqualified".into());
    }
    count += 1;

    let mut mutated = runs.clone();
    mutated["runs"][2]["metrics"]["unauthorised_secondary_data_uses"] = json!(1);
    if pareto_result(&mutated["runs"][2], baseline) != ParetoResult::CandidateDisqualified {
        return Err("candidate privacy misuse mutation was not disqualified".into());
    }
    count += 1;

    let mut mutated = runs.clone();
    mutated["runs"][2]["metrics"]["unexplained_reconciliation_deltas"] = json!(1);
    if pareto_result(&mutated["runs"][2], baseline) != ParetoResult::CandidateDisqualified {
        return Err("candidate reconciliation mutation was not disqualified".into());
    }
    count += 1;

    let mut mutated = runs.clone();
    mutated["runs"][2]["metrics"]["essential_service_denials_for_unrelated_consent_refusal"] =
        json!(1);
    if pareto_result(&mutated["runs"][2], baseline) != ParetoResult::CandidateDisqualified {
        return Err("candidate unrelated-consent denial mutation was not disqualified".into());
    }
    count += 1;

    let mut outcome_overflow = runs.clone();
    outcome_overflow["runs"][2]["metrics"]["unknown_outcomes"] = json!(100);
    if matched_run_report(&outcome_overflow).is_ok() {
        return Err("outcomes exceeding eligible attempts were accepted".into());
    }
    count += 1;

    let mut changed_cohort = runs.clone();
    changed_cohort["cohort"]["task_corpus_digest"] =
        json!("sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    if changed_cohort["cohort"] == runs["cohort"] {
        return Err("cohort digest mutation did not change cohort identity".into());
    }
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut market_text = MARKET_DEFAULT.to_owned();
    let mut runs_text = RUNS_DEFAULT.to_owned();
    let mut self_test = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--market" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--market requires a path",
                    )
                })?);
                market_text = fs::read_to_string(path)?;
            }
            "--runs" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(std::io::ErrorKind::InvalidInput, "--runs requires a path")
                })?);
                runs_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("payment-outcomes-benchmark [--market PATH] [--runs PATH] [--self-test]");
                return Ok(());
            }
            other => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("unknown argument: {other}"),
                )
                .into());
            }
        }
    }

    let market = validate_schema(MARKET_SCHEMA, &market_text).map_err(|errors| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; "))
    })?;
    let runs = validate_schema(RUNS_SCHEMA, &runs_text).map_err(|errors| {
        std::io::Error::new(std::io::ErrorKind::InvalidData, errors.join("; "))
    })?;
    let run_report = matched_run_report(&runs)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error))?;
    let mutation_count = if self_test {
        run_self_tests(&market, &runs, MARKET_SCHEMA, RUNS_SCHEMA)
            .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidData, error))?
    } else {
        0
    };

    let report = json!({
        "evaluator": "mycelix.payment-outcomes-benchmark.v1",
        "status": "PASS",
        "data_status": runs["status"],
        "market_context": snapshot_report(&market),
        "matched_run_report": run_report,
        "self_test_mutations": mutation_count,
        "external_effects_enabled": false
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixture_schemas_validate() {
        validate_schema(MARKET_SCHEMA, MARKET_DEFAULT).expect("market snapshot schema");
        validate_schema(RUNS_SCHEMA, RUNS_DEFAULT).expect("matched runs schema");
    }

    #[test]
    fn wilson_interval_is_bounded_and_well_defined() {
        let (lower, upper) = wilson_interval(95, 100).expect("valid interval");
        assert!(lower > 0.0);
        assert!(upper < 1.0);
        assert!(lower < 0.95 && upper > 0.95);
        assert!(wilson_interval(101, 100).is_none());
    }

    #[test]
    fn all_in_cost_must_reconcile_to_exact_fee_components() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        assert_eq!(
            fee_component_total(&runs["runs"][0]["metrics"]),
            Some(193_000)
        );
        let mut mutated = runs["runs"][0].clone();
        mutated["metrics"]["merchant_fee_minor_units"] = json!(68_001);
        let result = matched_run_report(&json!({
            "cohort": runs["cohort"],
            "runs": [mutated, runs["runs"][2].clone()]
        }));
        assert!(result.is_err());
    }

    #[test]
    fn national_aggregate_context_is_not_ranked() {
        let market: Value = serde_json::from_str(MARKET_DEFAULT).expect("market JSON");
        let report = snapshot_report(&market);
        assert!(report["raw_volume_leaderboard"].is_null());
        assert!(
            report["pairwise"]
                .as_array()
                .unwrap()
                .iter()
                .all(|pair| pair["rankable"] == false)
        );
    }

    #[test]
    fn matched_synthetic_fixture_has_no_hard_gate_violations() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        assert_eq!(
            pareto_result(&runs["runs"][2], &runs["runs"][0]),
            ParetoResult::TradeoffOrIncomparable
        );
    }

    #[test]
    fn hard_gate_mutations_disqualify_a_candidate() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        for key in [
            "duplicate_effects",
            "false_finality_promotions",
            "unexplained_reconciliation_deltas",
            "unauthorised_secondary_data_uses",
            "essential_service_denials_for_unrelated_consent_refusal",
        ] {
            let mut candidate = runs["runs"][2].clone();
            candidate["metrics"][key] = json!(1);
            assert_eq!(
                gate_status(&candidate["metrics"]),
                GateStatus::Disqualified,
                "{key}"
            );
        }
    }

    #[test]
    fn cohort_mismatch_is_not_a_comparison() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        let mut mismatched = runs["runs"][2].clone();
        mismatched["cohort"] = json!({"cohort_id":"other"});
        let result = matched_run_report(&json!({
            "cohort": runs["cohort"],
            "runs": [runs["runs"][0].clone(), mismatched]
        }));
        assert!(result.is_err());
    }

    #[test]
    fn worse_unknown_outcome_rate_prevents_candidate_dominance() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        let mut candidate = runs["runs"][2].clone();
        candidate["metrics"]["unknown_outcomes"] = json!(40);
        assert_eq!(
            pareto_result(&candidate, &runs["runs"][0]),
            ParetoResult::TradeoffOrIncomparable
        );
    }

    #[test]
    fn changed_task_denominator_rejects_matched_run_set() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        let mut changed = runs.clone();
        changed["runs"][2]["metrics"]["eligible_attempts"] = json!(1999);
        assert!(matched_run_report(&changed).is_err());
    }

    #[test]
    fn outcome_counts_cannot_exceed_eligible_attempts() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        let mut invalid = runs.clone();
        invalid["runs"][2]["metrics"]["unknown_outcomes"] = json!(100);
        assert!(matched_run_report(&invalid).is_err());
    }

    #[test]
    fn returned_or_reversed_cannot_exceed_delivered_count() {
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        let mut invalid = runs.clone();
        invalid["runs"][2]["metrics"]["returned_or_reversed"] = json!(2000);
        assert!(matched_run_report(&invalid).is_err());
    }

    #[test]
    fn run_self_test_campaign_passes() {
        let market: Value = serde_json::from_str(MARKET_DEFAULT).expect("market JSON");
        let runs: Value = serde_json::from_str(RUNS_DEFAULT).expect("run JSON");
        assert_eq!(
            run_self_tests(&market, &runs, MARKET_SCHEMA, RUNS_SCHEMA).expect("self tests"),
            11
        );
    }
}
