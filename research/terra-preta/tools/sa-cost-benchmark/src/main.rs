//! Synthetic-only South African soil decision cost auditor.
//!
//! Reconciles ten cost categories and a reference decision matrix. It reports
//! per-hectare, per-sample and per-correct-decision costs without ranking arms
//! or treating public tariffs as actual paid costs.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_strategy_costs_and_outcomes_not_market_quote";
const REPORT_STATUS: &str = "synthetic_cost_decision_report_not_market_evidence";
const BOUNDARY: &str = "Synthetic cost and decision arithmetic only. Published tariffs, written quotes, measured actual costs and synthetic assumptions are different evidence classes. No actual farm savings, test equivalence, agronomic suitability or supplier quote is established.";

const CATEGORIES: [&str; 10] = [
    "site_access_travel",
    "sampling_labour",
    "sample_preparation",
    "sensor_or_laboratory_assay",
    "instrument_capital_depreciation",
    "calibration_quality_control",
    "transport_courier",
    "analysis_reporting",
    "compute_storage",
    "repeats_retests",
];

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    evidence_class: String,
    scenario_id: String,
    comparison_area_ha: f64,
    eligible_decisions: usize,
    reference_positive_count: usize,
    reference_negative_count: usize,
    strategies: Vec<Strategy>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Strategy {
    strategy_id: String,
    strategy_family: String,
    description: String,
    sample_count: usize,
    decision_counts: DecisionCounts,
    cost_components: Vec<CostComponent>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DecisionCounts {
    true_positive: usize,
    true_negative: usize,
    false_positive: usize,
    false_negative: usize,
    unresolved_positive: usize,
    unresolved_negative: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CostComponent {
    category: String,
    status: String,
    evidence_class: String,
    quantity: Option<f64>,
    unit: Option<String>,
    unit_cost_zar: Option<f64>,
    source_name: Option<String>,
    source_url: Option<String>,
    valid_from: Option<String>,
    valid_to: Option<String>,
    method_scope: Option<String>,
    cost_record_reference: Option<String>,
    measurement_period: Option<String>,
    not_applicable_reason: Option<String>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    comparison_area_ha: f64,
    eligible_decisions: usize,
    reference_positive_count: usize,
    reference_negative_count: usize,
    strategies: Vec<StrategyResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct StrategyResult {
    strategy_id: String,
    strategy_family: String,
    sample_count: usize,
    total_cost_zar: f64,
    cost_per_hectare_zar: f64,
    cost_per_sample_zar: f64,
    correctly_resolved_decisions: usize,
    cost_per_correct_decision_zar: Option<f64>,
    resolved_coverage_fraction: f64,
    accuracy_among_resolved: Option<f64>,
    false_negative_rate_among_resolved_positive: Option<f64>,
    unresolved_positive_fraction: Option<f64>,
    false_positive_count: usize,
    false_negative_count: usize,
    category_costs_zar: BTreeMap<String, f64>,
    not_applicable_categories: Vec<String>,
}

#[derive(Debug, Serialize)]
struct Controls {
    all_cost_categories_accounted_for: bool,
    decision_counts_reconcile: bool,
    no_unpriced_zero_assumptions: bool,
    no_aggregate_strategy_score: bool,
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
        && value[0..4].parse::<u32>().is_ok_and(|x| (1..=9999).contains(&x))
        && value[5..7].parse::<u32>().is_ok_and(|x| (1..=12).contains(&x))
        && value[8..10].parse::<u32>().is_ok_and(|x| (1..=31).contains(&x))
}

fn validate_component(c: &CostComponent) -> Result<(), String> {
    if !CATEGORIES.contains(&c.category.as_str()) {
        return Err(format!("unsupported cost category {}", c.category));
    }
    if !matches!(c.evidence_class.as_str(), "published_tariff" | "written_quote" | "measured_actual_cost" | "synthetic_assumption") {
        return Err(format!("{} has unsupported cost evidence class", c.category));
    }
    match c.status.as_str() {
        "not_applicable" => {
            if c.not_applicable_reason.as_deref().unwrap_or("").trim().is_empty()
                || c.quantity.is_some()
                || c.unit.is_some()
                || c.unit_cost_zar.is_some()
            {
                return Err(format!("{} not_applicable requires a reason and no hidden zero price", c.category));
            }
        }
        "priced" => {
            let quantity = c.quantity.ok_or_else(|| format!("{} quantity is missing", c.category))?;
            let unit_price = c.unit_cost_zar.ok_or_else(|| format!("{} unit price is missing", c.category))?;
            if !quantity.is_finite() || quantity <= 0.0 || !unit_price.is_finite() || unit_price <= 0.0
                || !(quantity * unit_price).is_finite()
            {
                return Err(format!("{} priced quantity/price is invalid", c.category));
            }
            let unit = c.unit.as_deref().unwrap_or("");
            if !matches!(unit, "sample" | "scan" | "hectare" | "hour" | "visit" | "km" | "batch" | "campaign" | "instrument_year" | "month" | "package") {
                return Err(format!("{} has unsupported cost unit", c.category));
            }
            if c.method_scope.as_deref().unwrap_or("").trim().is_empty() {
                return Err(format!("{} requires method/service scope", c.category));
            }
            match c.evidence_class.as_str() {
                "published_tariff" | "written_quote" => {
                    if c.source_name.as_deref().unwrap_or("").trim().is_empty()
                        || c.source_url.as_deref().unwrap_or("").trim().is_empty()
                        || !c.valid_from.as_deref().is_some_and(valid_date)
                        || !c.valid_to.as_deref().is_some_and(valid_date)
                    {
                        return Err(format!("{} tariff/quote requires source and validity", c.category));
                    }
                }
                "measured_actual_cost" => {
                    if c.cost_record_reference.as_deref().unwrap_or("").trim().is_empty()
                        || c.measurement_period.as_deref().unwrap_or("").trim().is_empty()
                    {
                        return Err(format!("{} actual cost requires a record reference and period", c.category));
                    }
                }
                "synthetic_assumption" => {}
                _ => unreachable!(),
            }
            if c.not_applicable_reason.is_some() {
                return Err(format!("{} priced component cannot carry not_applicable_reason", c.category));
            }
        }
        _ => return Err(format!("{} has unsupported status", c.category)),
    }
    Ok(())
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "sa-soil-decision-cost-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.evidence_class != "synthetic"
    {
        return Err("v0.1 accepts only the declared synthetic scenario".into());
    }
    if s.scenario_id.trim().is_empty()
        || s.strategies.len() < 2
        || !s.comparison_area_ha.is_finite()
        || s.comparison_area_ha <= 0.0
        || s.eligible_decisions == 0
        || s.reference_positive_count + s.reference_negative_count != s.eligible_decisions
    {
        return Err("scenario scope, area or reference decision counts are invalid".into());
    }
    let mut strategy_ids = BTreeSet::new();
    for strategy in &s.strategies {
        if strategy.strategy_id.trim().is_empty()
            || strategy.description.trim().is_empty()
            || !strategy_ids.insert(strategy.strategy_id.clone())
            || strategy.sample_count == 0
        {
            return Err("strategy identity, description and sample_count must be valid and unique".into());
        }
        if !matches!(
            strategy.strategy_family.as_str(),
            "all_reference_lab" | "soil_health_panel" | "nir_only" | "fixed_referral"
                | "uncertainty_referral" | "decision_evsi_referral" | "proximal_sensing_service"
        ) {
            return Err(format!("{} has unsupported strategy family", strategy.strategy_id));
        }
        let d = &strategy.decision_counts;
        if d.true_positive + d.false_negative + d.unresolved_positive != s.reference_positive_count
            || d.true_negative + d.false_positive + d.unresolved_negative != s.reference_negative_count
        {
            return Err(format!("{} decision counts do not reconcile to reference totals", strategy.strategy_id));
        }

        let mut categories = BTreeSet::new();
        if strategy.cost_components.len() != CATEGORIES.len() {
            return Err(format!("{} must account for all ten cost categories", strategy.strategy_id));
        }
        for component in &strategy.cost_components {
            validate_component(component)?;
            if !categories.insert(component.category.clone()) {
                return Err(format!("{} duplicates cost category {}", strategy.strategy_id, component.category));
            }
        }
        if categories.len() != CATEGORIES.len() || CATEGORIES.iter().any(|x| !categories.contains(*x)) {
            return Err(format!("{} is missing one or more cost categories", strategy.strategy_id));
        }
    }
    Ok(())
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let mut output = Vec::new();
    for strategy in &s.strategies {
        let mut total = 0.0;
        let mut by_category = BTreeMap::new();
        let mut na = Vec::new();
        for component in &strategy.cost_components {
            let value = if component.status == "priced" {
                component.quantity.unwrap_or(0.0) * component.unit_cost_zar.unwrap_or(0.0)
            } else {
                na.push(component.category.clone());
                0.0
            };
            total += value;
            by_category.insert(component.category.clone(), value);
        }
        let d = &strategy.decision_counts;
        let correct = d.true_positive + d.true_negative;
        let resolved = correct + d.false_positive + d.false_negative;
        let positive_resolved = d.true_positive + d.false_negative;
        let coverage = resolved as f64 / s.eligible_decisions as f64;
        let accuracy = (resolved > 0).then_some(correct as f64 / resolved as f64);
        let fn_rate = (positive_resolved > 0).then_some(d.false_negative as f64 / positive_resolved as f64);
        let unresolved_positive_fraction = (s.reference_positive_count > 0)
            .then_some(d.unresolved_positive as f64 / s.reference_positive_count as f64);
        output.push(StrategyResult {
            strategy_id: strategy.strategy_id.clone(),
            strategy_family: strategy.strategy_family.clone(),
            sample_count: strategy.sample_count,
            total_cost_zar: total,
            cost_per_hectare_zar: total / s.comparison_area_ha,
            cost_per_sample_zar: total / strategy.sample_count as f64,
            correctly_resolved_decisions: correct,
            cost_per_correct_decision_zar: (correct > 0).then_some(total / correct as f64),
            resolved_coverage_fraction: coverage,
            accuracy_among_resolved: accuracy,
            false_negative_rate_among_resolved_positive: fn_rate,
            unresolved_positive_fraction,
            false_positive_count: d.false_positive,
            false_negative_count: d.false_negative,
            category_costs_zar: by_category,
            not_applicable_categories: na,
        });
    }
    Report {
        audit_version: VERSION,
        fixture_status: REPORT_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        comparison_area_ha: s.comparison_area_ha,
        eligible_decisions: s.eligible_decisions,
        reference_positive_count: s.reference_positive_count,
        reference_negative_count: s.reference_negative_count,
        strategies: output,
        controls: Controls {
            all_cost_categories_accounted_for: true,
            decision_counts_reconcile: true,
            no_unpriced_zero_assumptions: true,
            no_aggregate_strategy_score: true,
        },
        interpretation_boundary: BOUNDARY,
    }
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    if output_path.exists() {
        return Err(io::Error::new(io::ErrorKind::AlreadyExists, "output report already exists").into());
    }
    let bytes = fs::read(input_path)?;
    let scenario: Scenario = serde_json::from_slice(&bytes)?;
    validate(&scenario).map_err(io::Error::other)?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let report = build_report(&scenario, digest);
    let json = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(output_path)?;
    output.write_all(&json)?;
    output.write_all(b"\n")?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: sa-cost-benchmark <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("SA soil cost benchmark failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-cost-comparison.json"))
            .expect("synthetic scenario parses")
    }

    #[test]
    fn synthetic_strategy_fixture_reconciles_and_reports_components() {
        let s = scenario();
        validate(&s).expect("synthetic ledger valid");
        let r = build_report(&s, "0".repeat(64));
        assert_eq!(r.strategies.len(), 3);
        assert!(r.controls.no_aggregate_strategy_score);
        assert!(r.strategies.iter().all(|x| x.category_costs_zar.len() == 10));
    }

    #[test]
    fn missing_cost_category_is_rejected_not_zero_filled() {
        let mut s = scenario();
        s.strategies[0].cost_components.pop();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn unpriced_item_cannot_hide_as_zero() {
        let mut s = scenario();
        s.strategies[0].cost_components[0].status = "priced".into();
        s.strategies[0].cost_components[0].quantity = None;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn invalid_decision_counts_are_rejected() {
        let mut s = scenario();
        s.strategies[0].decision_counts.false_negative += 1;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn zero_cost_cannot_be_used_as_a_priced_unknown() {
        let mut s = scenario();
        s.strategies[0].cost_components[0].unit_cost_zar = Some(0.0);
        assert!(validate(&s).is_err());
    }

    #[test]
    fn measured_actual_cost_requires_record_and_period() {
        let mut s = scenario();
        let c = &mut s.strategies[0].cost_components[0];
        c.evidence_class = "measured_actual_cost".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn written_quote_requires_validity_and_source() {
        let mut s = scenario();
        let c = &mut s.strategies[0].cost_components[0];
        c.evidence_class = "written_quote".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn not_applicable_requires_reason() {
        let mut s = scenario();
        let c = &mut s.strategies[0].cost_components[0];
        c.status = "not_applicable".into();
        c.not_applicable_reason = None;
        assert!(validate(&s).is_err());
    }
}
