//! Deterministic CIV-IMPACT synthetic schedule generator and run-census verifier.
//!
//! This tool creates a preregisterable schedule; it does not implement a route
//! optimizer, simulate operational outcomes, or establish economic benefit.
//! A valid census proves completeness/contract consistency only.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::Path;

const MANIFEST_VERSION: &str = "0.1";
const BENCHMARK_ID: &str = "CIV-IMPACT-SYNTH-001";
const GENERATOR_VERSION: &str = "civ-impact-scenario-generator/0.1";
const MAX_RUNS: usize = 250_000;

const REGIMES: [&str; 5] = [
    "nominal",
    "demand_shock",
    "capacity_loss",
    "stale_spatial_data",
    "coordinator_outage",
];

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    scenario_id: String,
    generator_version: String,
    regime: String,
    demand_units: u64,
    capacity_units: u64,
    unit_cost_microcredits: u64,
    demand_shock_bps: u32,
    capacity_loss_bps: u32,
    spatial_age_hours: u32,
    missing_spatial_permille: u16,
    coordinator_available: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Arm {
    arm_id: String,
    kind: String,
    description: String,
    enabled_components: Vec<String>,
    implementation_ref: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PlannedRun {
    run_id: String,
    scenario_id: String,
    seed: u64,
    arm_id: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Schedule {
    manifest_version: String,
    benchmark_id: String,
    evidence_class: String,
    scenario_count: usize,
    seed_count: usize,
    scenario_set_sha256: String,
    scenarios: Vec<Scenario>,
    arms: Vec<Arm>,
    runs: Vec<PlannedRun>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RunMetrics {
    observed_demand_units: u64,
    served_units: u64,
    compute_cost_microcredits: u64,
    operational_cost_microcredits: u64,
    capital_amortization_cost_microcredits: u64,
    data_cost_microcredits: u64,
    integration_cost_microcredits: u64,
    human_review_cost_microcredits: u64,
    coordination_cost_microcredits: u64,
    failure_recovery_cost_microcredits: u64,
    total_cost_microcredits: u64,
    completion_latency_ms: u64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum Disposition {
    Completed,
    InvalidInput,
    Timeout,
    NonConverged,
    MissingOutput,
    EvaluatorError,
}

impl Disposition {
    fn as_str(self) -> &'static str {
        match self {
            Self::Completed => "completed",
            Self::InvalidInput => "invalid_input",
            Self::Timeout => "timeout",
            Self::NonConverged => "non_converged",
            Self::MissingOutput => "missing_output",
            Self::EvaluatorError => "evaluator_error",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RunResult {
    run_id: String,
    scenario_id: String,
    seed: u64,
    arm_id: String,
    disposition: Disposition,
    metrics: Option<RunMetrics>,
    failure_reason: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ResultsFile {
    manifest_version: String,
    benchmark_id: String,
    scenario_set_sha256: String,
    schedule_artifact_sha256: String,
    runs: Vec<RunResult>,
}

#[derive(Debug, Serialize)]
struct VerificationReport {
    verifier: String,
    contract_valid: bool,
    benchmark_id: String,
    evidence_class: String,
    scenario_count: usize,
    seed_count: usize,
    arm_count: usize,
    scheduled_runs: usize,
    completed_runs: usize,
    noncompleted_runs: usize,
    dispositions: BTreeMap<String, usize>,
    schedule_artifact_sha256: String,
    results_artifact_sha256: String,
    claim_ceiling: String,
    economic_benefit_established: bool,
}

#[derive(Debug, Serialize)]
struct BootstrapInterval {
    lower_2_5_percentile: f64,
    upper_97_5_percentile: f64,
}

#[derive(Debug, Serialize)]
struct BootstrapUncertainty {
    method: String,
    resamples: usize,
    central_interval_level: f64,
    resampling_unit: String,
    stratification: String,
    bootstrap_seed_sha256: String,
    relative_pooled_cost_per_served_unit_change_percent: BootstrapInterval,
    candidate_minus_baseline_pooled_cost_per_served_unit_microcredits: BootstrapInterval,
    candidate_minus_baseline_service_rate_percentage_points: BootstrapInterval,
    claim_ceiling: String,
}

#[derive(Debug, Serialize)]
struct ComparisonReport {
    benchmark_id: String,
    evidence_class: String,
    baseline_arm: String,
    candidate_arm: String,
    expected_pairs: usize,
    complete_pairs: usize,
    incomplete_pairs: usize,
    zero_service_pairs: usize,
    baseline_mean_cost_per_demand_unit_microcredits: Option<f64>,
    candidate_mean_cost_per_demand_unit_microcredits: Option<f64>,
    candidate_minus_baseline_cost_per_demand_unit_microcredits: Option<f64>,
    relative_cost_per_demand_change_percent: Option<f64>,
    baseline_mean_service_rate: Option<f64>,
    candidate_mean_service_rate: Option<f64>,
    candidate_minus_baseline_service_rate_percentage_points: Option<f64>,
    baseline_mean_cost_per_served_unit_microcredits: Option<f64>,
    candidate_mean_cost_per_served_unit_microcredits: Option<f64>,
    candidate_minus_baseline_cost_per_served_unit_microcredits: Option<f64>,
    relative_cost_per_served_unit_change_percent: Option<f64>,
    baseline_pooled_cost_per_served_unit_microcredits: Option<f64>,
    candidate_pooled_cost_per_served_unit_microcredits: Option<f64>,
    candidate_minus_baseline_pooled_cost_per_served_unit_microcredits: Option<f64>,
    relative_pooled_cost_per_served_unit_change_percent: Option<f64>,
    cost_per_served_unit_defined_for_all_pairs: bool,
    stratified_scenario_cluster_bootstrap_95: Option<BootstrapUncertainty>,
    descriptive_only: bool,
    economic_benefit_established: bool,
    claim_ceiling: String,
}

struct XorShift64(u64);

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E37_79B9_7F4A_7C15 } else { seed })
    }

    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn range(&mut self, lower: u64, upper_exclusive: u64) -> u64 {
        assert!(lower < upper_exclusive, "range must be non-empty");
        let span = upper_exclusive - lower;
        // XorShift64 emits the non-zero u64 domain [1, u64::MAX]. Reject its
        // short tail so each residue class has exactly the same number of
        // accepted values; this avoids modulo bias in regime resampling.
        let accepted_count = u64::MAX - (u64::MAX % span);
        loop {
            let value = self.next();
            if value <= accepted_count {
                return lower + ((value - 1) % span);
            }
        }
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join("")
}

fn build_arms() -> Vec<Arm> {
    vec![
        Arm {
            arm_id: "incumbent_baseline".into(),
            kind: "incumbent".into(),
            description: "Declared incumbent policy; implementation must be pinned before scoring".into(),
            enabled_components: vec![],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-baseline-ref".into(),
        },
        Arm {
            arm_id: "symtropy_only".into(),
            kind: "single_component".into(),
            description: "Symtropy-only proposal under the same declared constraints".into(),
            enabled_components: vec!["symtropy".into()],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
        Arm {
            arm_id: "full_stack".into(),
            kind: "full_stack".into(),
            description: "All four projects composed behind their declared authority boundaries".into(),
            enabled_components: vec![
                "symthaea".into(),
                "symtropy".into(),
                "sol_atlas".into(),
                "mycelix".into(),
            ],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
        Arm {
            arm_id: "without_symthaea".into(),
            kind: "leave_one_out_ablation".into(),
            description: "Full composition with Symthaea disabled".into(),
            enabled_components: vec!["symtropy".into(), "sol_atlas".into(), "mycelix".into()],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
        Arm {
            arm_id: "without_symtropy".into(),
            kind: "leave_one_out_ablation".into(),
            description: "Full composition with Symtropy disabled".into(),
            enabled_components: vec!["symthaea".into(), "sol_atlas".into(), "mycelix".into()],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
        Arm {
            arm_id: "without_sol_atlas".into(),
            kind: "leave_one_out_ablation".into(),
            description: "Full composition with Sol-Atlas disabled".into(),
            enabled_components: vec!["symthaea".into(), "symtropy".into(), "mycelix".into()],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
        Arm {
            arm_id: "without_mycelix".into(),
            kind: "leave_one_out_ablation".into(),
            description: "Full composition with Mycelix disabled".into(),
            enabled_components: vec!["symthaea".into(), "symtropy".into(), "sol_atlas".into()],
            implementation_ref: "UNQUALIFIED:replace-with-pinned-implementation-ref".into(),
        },
    ]
}

fn build_scenarios(count: usize) -> Result<Vec<Scenario>, String> {
    if count < REGIMES.len() || count % REGIMES.len() != 0 {
        return Err(format!(
            "scenario count must be at least {} and divisible by {} so regimes are balanced",
            REGIMES.len(),
            REGIMES.len()
        ));
    }

    let mut scenarios = Vec::with_capacity(count);
    for index in 0..count {
        let regime = REGIMES[index % REGIMES.len()];
        let mut rng = XorShift64::new(0xC1_71_1A_C7_5EED_u64 ^ index as u64);
        let demand = rng.range(1_000, 10_001);
        let nominal_capacity = demand.saturating_mul(rng.range(85, 141)) / 100;
        let (demand_shock_bps, capacity_loss_bps, spatial_age_hours, missing_permille, coordinator_available) =
            match regime {
                "nominal" => (0, 0, rng.range(0, 6) as u32, rng.range(0, 21) as u16, true),
                "demand_shock" => (rng.range(2_500, 10_001) as u32, 0, rng.range(0, 25) as u32, rng.range(0, 51) as u16, true),
                "capacity_loss" => (0, rng.range(2_500, 7_501) as u32, rng.range(0, 25) as u32, rng.range(0, 51) as u16, true),
                "stale_spatial_data" => (0, 0, rng.range(24, 721) as u32, rng.range(50, 501) as u16, true),
                "coordinator_outage" => (0, 0, rng.range(0, 25) as u32, rng.range(0, 51) as u16, false),
                _ => return Err(format!("unknown generator regime: {regime}")),
            };
        scenarios.push(Scenario {
            scenario_id: format!("syn-transport-{index:04}"),
            generator_version: GENERATOR_VERSION.into(),
            regime: regime.into(),
            demand_units: demand,
            capacity_units: nominal_capacity,
            unit_cost_microcredits: rng.range(500, 2_501),
            demand_shock_bps,
            capacity_loss_bps,
            spatial_age_hours,
            missing_spatial_permille: missing_permille,
            coordinator_available,
        });
    }
    Ok(scenarios)
}

fn build_schedule(scenario_count: usize, seed_count: usize) -> Result<Schedule, String> {
    if seed_count == 0 {
        return Err("seed count must be at least 1".into());
    }
    // Bound allocation before constructing the scenario vector from user-supplied CLI sizes.
    let arms = build_arms();
    let expected = scenario_count
        .checked_mul(seed_count)
        .and_then(|v| v.checked_mul(arms.len()))
        .ok_or_else(|| "planned-run count overflow".to_string())?;
    if expected > MAX_RUNS {
        return Err(format!("planned run count {expected} exceeds safety cap {MAX_RUNS}"));
    }
    let scenarios = build_scenarios(scenario_count)?;

    let scenarios_bytes = serde_json::to_vec(&scenarios).map_err(|e| e.to_string())?;
    let scenario_set_sha256 = sha256_hex(&scenarios_bytes);
    let mut runs = Vec::with_capacity(expected);
    for scenario in &scenarios {
        for seed in 1..=seed_count as u64 {
            for arm in &arms {
                runs.push(PlannedRun {
                    run_id: format!("{}:seed-{seed:04}:{}", scenario.scenario_id, arm.arm_id),
                    scenario_id: scenario.scenario_id.clone(),
                    seed,
                    arm_id: arm.arm_id.clone(),
                });
            }
        }
    }

    Ok(Schedule {
        manifest_version: MANIFEST_VERSION.into(),
        benchmark_id: BENCHMARK_ID.into(),
        evidence_class: "synthetic_only".into(),
        scenario_count,
        seed_count,
        scenario_set_sha256,
        scenarios,
        arms,
        runs,
    })
}

fn validate_schedule(schedule: &Schedule) -> Result<(), String> {
    if schedule.manifest_version != MANIFEST_VERSION {
        return Err("unsupported schedule manifest_version".into());
    }
    if schedule.benchmark_id != BENCHMARK_ID {
        return Err("unexpected benchmark_id".into());
    }
    if schedule.evidence_class != "synthetic_only" {
        return Err("generator schedule must be marked synthetic_only".into());
    }
    if schedule.scenario_count != schedule.scenarios.len() {
        return Err("scenario_count does not match scenario array length".into());
    }
    if schedule.scenario_count < REGIMES.len() || schedule.scenario_count % REGIMES.len() != 0 {
        return Err("scenario count must be divisible by the number of named regimes".into());
    }
    if schedule.seed_count == 0 || schedule.scenarios.is_empty() {
        return Err("scenario and seed counts must be nonzero".into());
    }
    let scenario_bytes = serde_json::to_vec(&schedule.scenarios).map_err(|e| e.to_string())?;
    if sha256_hex(&scenario_bytes) != schedule.scenario_set_sha256 {
        return Err("scenario_set_sha256 does not match compact serialized scenario content".into());
    }

    let mut scenario_by_id = HashMap::new();
    let mut regime_counts = BTreeMap::<String, usize>::new();
    for scenario in &schedule.scenarios {
        if !scenario.scenario_id.starts_with("syn-transport-") {
            return Err(format!("unexpected synthetic scenario id: {}", scenario.scenario_id));
        }
        if !REGIMES.contains(&scenario.regime.as_str()) {
            return Err(format!("unknown scenario regime: {}", scenario.regime));
        }
        if scenario.generator_version != GENERATOR_VERSION
            || scenario.demand_units == 0
            || scenario.capacity_units == 0
            || scenario.unit_cost_microcredits == 0
        {
            return Err(format!("scenario {} has invalid generator identity or zero-valued base inputs", scenario.scenario_id));
        }
        if scenario.missing_spatial_permille > 1_000
            || scenario.demand_shock_bps > 20_000
            || scenario.capacity_loss_bps > 10_000
        {
            return Err(format!("scenario {} has out-of-range stress parameters", scenario.scenario_id));
        }
        let regime_consistent = match scenario.regime.as_str() {
            "nominal" => scenario.demand_shock_bps == 0
                && scenario.capacity_loss_bps == 0
                && scenario.coordinator_available,
            "demand_shock" => scenario.demand_shock_bps > 0
                && scenario.capacity_loss_bps == 0
                && scenario.coordinator_available,
            "capacity_loss" => scenario.demand_shock_bps == 0
                && scenario.capacity_loss_bps > 0
                && scenario.coordinator_available,
            "stale_spatial_data" => scenario.demand_shock_bps == 0
                && scenario.capacity_loss_bps == 0
                && scenario.spatial_age_hours >= 24
                && scenario.missing_spatial_permille >= 50
                && scenario.coordinator_available,
            "coordinator_outage" => !scenario.coordinator_available,
            _ => false,
        };
        if !regime_consistent {
            return Err(format!(
                "scenario {} has parameters inconsistent with regime {}",
                scenario.scenario_id, scenario.regime
            ));
        }
        if scenario_by_id.insert(scenario.scenario_id.as_str(), scenario).is_some() {
            return Err(format!("duplicate scenario_id: {}", scenario.scenario_id));
        }
        *regime_counts.entry(scenario.regime.clone()).or_default() += 1;
    }
    if regime_counts.len() != REGIMES.len()
        || regime_counts.values().any(|count| *count != schedule.scenario_count / REGIMES.len())
    {
        return Err("schedule must balance all five named stress regimes equally".into());
    }

    let mut arm_by_id = HashMap::new();
    for arm in &schedule.arms {
        if arm.arm_id.trim().is_empty() {
            return Err("arm_id cannot be empty".into());
        }
        if arm_by_id.insert(arm.arm_id.as_str(), arm).is_some() {
            return Err(format!("duplicate arm_id: {}", arm.arm_id));
        }
        let mut components = HashSet::new();
        for component in &arm.enabled_components {
            if !["symthaea", "symtropy", "sol_atlas", "mycelix"].contains(&component.as_str()) {
                return Err(format!("arm {} has unknown component {component}", arm.arm_id));
            }
            if !components.insert(component) {
                return Err(format!("arm {} repeats component {component}", arm.arm_id));
            }
        }
    }
    let required_arm_ids = [
        "incumbent_baseline",
        "symtropy_only",
        "full_stack",
        "without_symthaea",
        "without_symtropy",
        "without_sol_atlas",
        "without_mycelix",
    ];
    if arm_by_id.len() != required_arm_ids.len()
        || required_arm_ids.iter().any(|arm_id| !arm_by_id.contains_key(*arm_id))
    {
        return Err("v0.1 schedule must contain the exact seven registered comparison arms".into());
    }
    let expected_components: [(&str, &[&str]); 7] = [
        ("incumbent_baseline", &[]),
        ("symtropy_only", &["symtropy"]),
        ("full_stack", &["symthaea", "symtropy", "sol_atlas", "mycelix"]),
        ("without_symthaea", &["symtropy", "sol_atlas", "mycelix"]),
        ("without_symtropy", &["symthaea", "sol_atlas", "mycelix"]),
        ("without_sol_atlas", &["symthaea", "symtropy", "mycelix"]),
        ("without_mycelix", &["symthaea", "symtropy", "sol_atlas"]),
    ];
    for (arm_id, expected) in expected_components {
        let arm = arm_by_id.get(arm_id).expect("required arm checked above");
        if arm.enabled_components.len() != expected.len()
            || expected
                .iter()
                .any(|component| !arm.enabled_components.iter().any(|value| value.as_str() == *component))
        {
            return Err(format!("arm {arm_id} does not match its registered component set"));
        }
    }

    let expected_count = schedule
        .scenario_count
        .checked_mul(schedule.seed_count)
        .and_then(|v| v.checked_mul(schedule.arms.len()))
        .ok_or_else(|| "planned-run count overflow".to_string())?;
    if expected_count > MAX_RUNS || schedule.runs.len() != expected_count {
        return Err(format!(
            "scheduled run count mismatch: expected {expected_count}, observed {}",
            schedule.runs.len()
        ));
    }

    let mut seen = HashSet::with_capacity(schedule.runs.len());
    for run in &schedule.runs {
        if !scenario_by_id.contains_key(run.scenario_id.as_str()) {
            return Err(format!("run {} references unknown scenario", run.run_id));
        }
        if !arm_by_id.contains_key(run.arm_id.as_str()) {
            return Err(format!("run {} references unknown arm", run.run_id));
        }
        if run.seed == 0 || run.seed > schedule.seed_count as u64 {
            return Err(format!("run {} has seed outside the declared range", run.run_id));
        }
        let expected_id = format!("{}:seed-{:04}:{}", run.scenario_id, run.seed, run.arm_id);
        if run.run_id != expected_id {
            return Err(format!("run_id does not match scenario/seed/arm for {}", run.run_id));
        }
        if !seen.insert(run.run_id.as_str()) {
            return Err(format!("duplicate run_id in schedule: {}", run.run_id));
        }
    }

    // Require complete paired coverage: every scenario x seed x arm exactly once.
    for scenario in &schedule.scenarios {
        for seed in 1..=schedule.seed_count as u64 {
            for arm in &schedule.arms {
                let run_id = format!("{}:seed-{seed:04}:{}", scenario.scenario_id, arm.arm_id);
                if !seen.contains(run_id.as_str()) {
                    return Err(format!("schedule is missing planned run {run_id}"));
                }
            }
        }
    }
    Ok(())
}

fn effective_demand(scenario: &Scenario) -> Result<u64, String> {
    let factor = 10_000u64
        .checked_add(scenario.demand_shock_bps as u64)
        .ok_or_else(|| "demand shock factor overflow".to_string())?;
    scenario
        .demand_units
        .checked_mul(factor)
        .map(|value| value / 10_000)
        .ok_or_else(|| format!("effective demand overflow for {}", scenario.scenario_id))
}

fn checked_cost_total(metrics: &RunMetrics) -> Result<u64, String> {
    [
        metrics.compute_cost_microcredits,
        metrics.operational_cost_microcredits,
        metrics.capital_amortization_cost_microcredits,
        metrics.data_cost_microcredits,
        metrics.integration_cost_microcredits,
        metrics.human_review_cost_microcredits,
        metrics.coordination_cost_microcredits,
        metrics.failure_recovery_cost_microcredits,
    ]
    .into_iter()
    .try_fold(0u64, |sum, value| sum.checked_add(value))
    .ok_or_else(|| "cost-component sum overflow".to_string())
}

fn verify_census(
    schedule: &Schedule,
    result_file: &ResultsFile,
    schedule_artifact_sha256: &str,
) -> Result<BTreeMap<String, usize>, String> {
    validate_schedule(schedule)?;
    if result_file.manifest_version != MANIFEST_VERSION {
        return Err("unsupported results manifest_version".into());
    }
    if result_file.benchmark_id != schedule.benchmark_id {
        return Err("results benchmark_id does not match schedule".into());
    }
    if result_file.schedule_artifact_sha256 != schedule_artifact_sha256 {
        return Err("results schedule_artifact_sha256 does not match exact schedule bytes".into());
    }
    if result_file.scenario_set_sha256 != schedule.scenario_set_sha256 {
        return Err("results scenario_set_sha256 does not match schedule".into());
    }
    if result_file.runs.len() != schedule.runs.len() {
        return Err(format!(
            "result census length mismatch: expected {}, observed {}",
            schedule.runs.len(),
            result_file.runs.len()
        ));
    }

    let planned: HashMap<&str, &PlannedRun> =
        schedule.runs.iter().map(|run| (run.run_id.as_str(), run)).collect();
    let scenarios: HashMap<&str, &Scenario> = schedule
        .scenarios
        .iter()
        .map(|scenario| (scenario.scenario_id.as_str(), scenario))
        .collect();
    let mut seen = HashSet::with_capacity(result_file.runs.len());
    let mut dispositions = BTreeMap::<String, usize>::new();

    for result in &result_file.runs {
        let Some(expected) = planned.get(result.run_id.as_str()) else {
            return Err(format!("unexpected result run_id: {}", result.run_id));
        };
        if !seen.insert(result.run_id.as_str()) {
            return Err(format!("duplicate result run_id: {}", result.run_id));
        }
        if result.scenario_id != expected.scenario_id
            || result.seed != expected.seed
            || result.arm_id != expected.arm_id
        {
            return Err(format!("result identity mismatch for {}", result.run_id));
        }
        match result.disposition {
            Disposition::Completed => {
                let metrics = result
                    .metrics
                    .as_ref()
                    .ok_or_else(|| format!("completed run {} has no metrics", result.run_id))?;
                if result.failure_reason.is_some() {
                    return Err(format!("completed run {} must not have failure_reason", result.run_id));
                }
                let scenario = scenarios
                    .get(result.scenario_id.as_str())
                    .ok_or_else(|| format!("unknown scenario on result {}", result.run_id))?;
                let expected_demand = effective_demand(scenario)?;
                if metrics.observed_demand_units != expected_demand {
                    return Err(format!(
                        "effective demand mismatch for {}: expected {}, observed {}",
                        result.run_id, expected_demand, metrics.observed_demand_units
                    ));
                }
                if metrics.served_units > metrics.observed_demand_units {
                    return Err(format!("served_units exceeds observed demand for {}", result.run_id));
                }
                let computed = checked_cost_total(metrics)?;
                if metrics.total_cost_microcredits != computed {
                    return Err(format!(
                        "total cost mismatch for {}: recorded {}, computed {}",
                        result.run_id, metrics.total_cost_microcredits, computed
                    ));
                }
            }
            _ => {
                if result.metrics.is_some() {
                    return Err(format!("non-completed run {} must not publish complete metrics", result.run_id));
                }
                if result.failure_reason.as_deref().unwrap_or("").trim().is_empty() {
                    return Err(format!("non-completed run {} needs an explicit failure_reason", result.run_id));
                }
            }
        }
        *dispositions.entry(result.disposition.as_str().to_owned()).or_default() += 1;
    }

    if seen.len() != planned.len() {
        let missing = planned
            .keys()
            .filter(|run_id| !seen.contains(**run_id))
            .take(10)
            .copied()
            .collect::<Vec<_>>();
        return Err(format!("result census omits scheduled runs; examples: {}", missing.join(", ")));
    }
    Ok(dispositions)
}

fn mean(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        None
    } else {
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

const BOOTSTRAP_RESAMPLES: usize = 10_000;

#[derive(Debug, Clone, Copy, Default)]
struct Aggregate {
    cost_microcredits: u128,
    served_units: u128,
    demand_units: u128,
}

#[derive(Debug, Clone)]
struct ScenarioAggregate {
    regime: String,
    baseline: Aggregate,
    candidate: Aggregate,
}

fn scenario_arm_aggregate(
    scenario: &Scenario,
    arm_id: &str,
    results_by_id: &HashMap<&str, &RunResult>,
    seed_count: usize,
) -> Result<Option<Aggregate>, String> {
    let mut aggregate = Aggregate::default();
    let demand = effective_demand(scenario)? as u128;
    for seed in 1..=seed_count as u64 {
        let run_id = format!("{}:seed-{seed:04}:{arm_id}", scenario.scenario_id);
        let result = results_by_id
            .get(run_id.as_str())
            .ok_or_else(|| format!("missing result while aggregating {run_id}"))?;
        if result.disposition != Disposition::Completed {
            return Ok(None);
        }
        let Some(metrics) = result.metrics.as_ref() else {
            return Ok(None);
        };
        if metrics.served_units == 0 {
            return Ok(None);
        }
        aggregate.cost_microcredits += metrics.total_cost_microcredits as u128;
        aggregate.served_units += metrics.served_units as u128;
        aggregate.demand_units += demand;
    }
    Ok(Some(aggregate))
}

fn bootstrap_percentile(values: &mut [f64], quantile: f64) -> Result<f64, String> {
    if values.is_empty() || !(0.0..=1.0).contains(&quantile) {
        return Err("invalid percentile input".into());
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err("bootstrap produced a non-finite statistic".into());
    }
    values.sort_by(f64::total_cmp);
    let rank = ((quantile * values.len() as f64).ceil() as usize)
        .saturating_sub(1)
        .min(values.len() - 1);
    Ok(values[rank])
}

fn stratified_scenario_cluster_bootstrap(
    schedule: &Schedule,
    results_by_id: &HashMap<&str, &RunResult>,
    seed: u64,
    seed_digest: &str,
) -> Result<Option<BootstrapUncertainty>, String> {
    let mut scenario_aggregates = Vec::with_capacity(schedule.scenarios.len());
    for scenario in &schedule.scenarios {
        let Some(baseline) = scenario_arm_aggregate(
            scenario,
            "incumbent_baseline",
            results_by_id,
            schedule.seed_count,
        )? else {
            return Ok(None);
        };
        let Some(candidate) = scenario_arm_aggregate(
            scenario,
            "full_stack",
            results_by_id,
            schedule.seed_count,
        )? else {
            return Ok(None);
        };
        // Relative cost change is undefined if an entire baseline cluster has zero cost.
        // Fail closed rather than dropping such a cluster from bootstrap replicates.
        if baseline.cost_microcredits == 0 || baseline.served_units == 0 || candidate.served_units == 0 {
            return Ok(None);
        }
        scenario_aggregates.push(ScenarioAggregate {
            regime: scenario.regime.clone(),
            baseline,
            candidate,
        });
    }

    let mut strata: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (index, aggregate) in scenario_aggregates.iter().enumerate() {
        strata.entry(aggregate.regime.clone()).or_default().push(index);
    }
    if strata.len() != REGIMES.len() || strata.values().any(|indices| indices.is_empty()) {
        return Err("bootstrap requires all registered, non-empty scenario strata".into());
    }

    let mut rng = XorShift64::new(seed);
    let mut relative_cost_changes = Vec::with_capacity(BOOTSTRAP_RESAMPLES);
    let mut absolute_cost_changes = Vec::with_capacity(BOOTSTRAP_RESAMPLES);
    let mut service_changes = Vec::with_capacity(BOOTSTRAP_RESAMPLES);

    for _ in 0..BOOTSTRAP_RESAMPLES {
        let mut baseline = Aggregate::default();
        let mut candidate = Aggregate::default();
        let mut baseline_scenario_service_rate_sum = 0.0;
        let mut candidate_scenario_service_rate_sum = 0.0;
        let mut sampled_scenario_count = 0usize;
        for indices in strata.values() {
            // Resample whole scenarios within each registered regime. All seeds from
            // one scenario stay together, so seeds do not masquerade as independent sites.
            for _ in 0..indices.len() {
                let picked = rng.range(0, indices.len() as u64) as usize;
                let cluster = &scenario_aggregates[indices[picked]];
                baseline.cost_microcredits += cluster.baseline.cost_microcredits;
                baseline.served_units += cluster.baseline.served_units;
                baseline.demand_units += cluster.baseline.demand_units;
                candidate.cost_microcredits += cluster.candidate.cost_microcredits;
                candidate.served_units += cluster.candidate.served_units;
                candidate.demand_units += cluster.candidate.demand_units;
                // Match the report's equal-scenario-weight service-rate estimand,
                // rather than silently weighting higher-demand scenarios more heavily.
                baseline_scenario_service_rate_sum +=
                    cluster.baseline.served_units as f64 / cluster.baseline.demand_units as f64;
                candidate_scenario_service_rate_sum +=
                    cluster.candidate.served_units as f64 / cluster.candidate.demand_units as f64;
                sampled_scenario_count += 1;
            }
        }
        if baseline.cost_microcredits == 0
            || baseline.served_units == 0
            || candidate.served_units == 0
            || baseline.demand_units == 0
        {
            return Err("bootstrap resample has undefined primary statistics".into());
        }
        let baseline_cost_per_served =
            baseline.cost_microcredits as f64 / baseline.served_units as f64;
        let candidate_cost_per_served =
            candidate.cost_microcredits as f64 / candidate.served_units as f64;
        if baseline_cost_per_served <= 0.0 {
            return Err("bootstrap baseline cost per served unit is not positive".into());
        }
        relative_cost_changes.push(
            (candidate_cost_per_served / baseline_cost_per_served - 1.0) * 100.0,
        );
        absolute_cost_changes.push(candidate_cost_per_served - baseline_cost_per_served);

        if sampled_scenario_count == 0 {
            return Err("bootstrap sampled no scenarios".into());
        }
        let base_service_rate =
            baseline_scenario_service_rate_sum / sampled_scenario_count as f64;
        let candidate_service_rate =
            candidate_scenario_service_rate_sum / sampled_scenario_count as f64;
        service_changes.push((candidate_service_rate - base_service_rate) * 100.0);
    }

    Ok(Some(BootstrapUncertainty {
        method: "stratified scenario-cluster percentile bootstrap".into(),
        resamples: BOOTSTRAP_RESAMPLES,
        central_interval_level: 0.95,
        resampling_unit: "scenario cluster; all seeds and paired arms within a sampled scenario".into(),
        stratification: "resample with replacement within each registered stress regime".into(),
        bootstrap_seed_sha256: seed_digest.to_owned(),
        relative_pooled_cost_per_served_unit_change_percent: BootstrapInterval {
            lower_2_5_percentile: bootstrap_percentile(&mut relative_cost_changes, 0.025)?,
            upper_97_5_percentile: bootstrap_percentile(&mut relative_cost_changes, 0.975)?,
        },
        candidate_minus_baseline_pooled_cost_per_served_unit_microcredits: BootstrapInterval {
            lower_2_5_percentile: bootstrap_percentile(&mut absolute_cost_changes, 0.025)?,
            upper_97_5_percentile: bootstrap_percentile(&mut absolute_cost_changes, 0.975)?,
        },
        candidate_minus_baseline_service_rate_percentage_points: BootstrapInterval {
            lower_2_5_percentile: bootstrap_percentile(&mut service_changes, 0.025)?,
            upper_97_5_percentile: bootstrap_percentile(&mut service_changes, 0.975)?,
        },
        claim_ceiling: "descriptive_uncertainty_for_this_synthetic_corpus; not causal or economic qualification".into(),
    }))
}

fn make_comparison(
    schedule: &Schedule,
    result_file: &ResultsFile,
    schedule_artifact_sha256: &str,
    bootstrap_seed: u64,
    bootstrap_seed_digest: &str,
) -> Result<ComparisonReport, String> {
    // This also proves the census is complete and all included metric rows are internally consistent.
    verify_census(schedule, result_file, schedule_artifact_sha256)?;

    let results_by_id: HashMap<&str, &RunResult> = result_file
        .runs
        .iter()
        .map(|result| (result.run_id.as_str(), result))
        .collect();
    let expected_pairs = schedule
        .scenario_count
        .checked_mul(schedule.seed_count)
        .ok_or_else(|| "paired comparison count overflow".to_string())?;
    let mut complete_pairs = 0usize;
    let mut incomplete_pairs = 0usize;
    let mut zero_service_pairs = 0usize;
    let mut baseline_cost_per_demand = Vec::new();
    let mut candidate_cost_per_demand = Vec::new();
    let mut baseline_service_rates = Vec::new();
    let mut candidate_service_rates = Vec::new();
    let mut baseline_cost_per_served = Vec::new();
    let mut candidate_cost_per_served = Vec::new();
    let mut baseline_total_cost_microcredits: u128 = 0;
    let mut candidate_total_cost_microcredits: u128 = 0;
    let mut baseline_total_served_units: u128 = 0;
    let mut candidate_total_served_units: u128 = 0;

    for scenario in &schedule.scenarios {
        let demand = effective_demand(scenario)?;
        for seed in 1..=schedule.seed_count as u64 {
            let baseline_id = format!("{}:seed-{seed:04}:incumbent_baseline", scenario.scenario_id);
            let candidate_id = format!("{}:seed-{seed:04}:full_stack", scenario.scenario_id);
            let baseline = results_by_id
                .get(baseline_id.as_str())
                .ok_or_else(|| format!("missing baseline row {baseline_id}"))?;
            let candidate = results_by_id
                .get(candidate_id.as_str())
                .ok_or_else(|| format!("missing full-stack row {candidate_id}"))?;

            let (Some(base_metrics), Some(candidate_metrics)) = (
                baseline.metrics.as_ref().filter(|_| baseline.disposition == Disposition::Completed),
                candidate.metrics.as_ref().filter(|_| candidate.disposition == Disposition::Completed),
            ) else {
                incomplete_pairs += 1;
                continue;
            };

            complete_pairs += 1;
            let base_rate = base_metrics.served_units as f64 / demand as f64;
            let candidate_rate = candidate_metrics.served_units as f64 / demand as f64;
            let base_cost_per_demand = base_metrics.total_cost_microcredits as f64 / demand as f64;
            let candidate_cost_per_demand = candidate_metrics.total_cost_microcredits as f64 / demand as f64;

            baseline_cost_per_demand.push(base_cost_per_demand);
            candidate_cost_per_demand.push(candidate_cost_per_demand);
            baseline_service_rates.push(base_rate);
            candidate_service_rates.push(candidate_rate);
            baseline_total_cost_microcredits += base_metrics.total_cost_microcredits as u128;
            candidate_total_cost_microcredits += candidate_metrics.total_cost_microcredits as u128;
            baseline_total_served_units += base_metrics.served_units as u128;
            candidate_total_served_units += candidate_metrics.served_units as u128;

            if base_metrics.served_units == 0 || candidate_metrics.served_units == 0 {
                zero_service_pairs += 1;
            } else {
                baseline_cost_per_served.push(
                    base_metrics.total_cost_microcredits as f64 / base_metrics.served_units as f64,
                );
                candidate_cost_per_served.push(
                    candidate_metrics.total_cost_microcredits as f64 / candidate_metrics.served_units as f64,
                );
            }
        }
    }

    let all_pairs_completed = complete_pairs == expected_pairs && incomplete_pairs == 0;
    // Fail closed: do not publish complete-case means as if they described the registered
    // scenario/seed schedule when any baseline/full-stack pair is incomplete.
    let baseline_cost_mean = if all_pairs_completed { mean(&baseline_cost_per_demand) } else { None };
    let candidate_cost_mean = if all_pairs_completed { mean(&candidate_cost_per_demand) } else { None };
    let paired_cost_delta = match (baseline_cost_mean, candidate_cost_mean) {
        (Some(base), Some(candidate)) => Some(candidate - base),
        _ => None,
    };
    let relative_cost_change = match (baseline_cost_mean, candidate_cost_mean) {
        (Some(base), Some(candidate)) if base != 0.0 => Some((candidate / base - 1.0) * 100.0),
        _ => None,
    };
    let baseline_rate_mean = if all_pairs_completed { mean(&baseline_service_rates) } else { None };
    let candidate_rate_mean = if all_pairs_completed { mean(&candidate_service_rates) } else { None };
    let service_delta_pp = match (baseline_rate_mean, candidate_rate_mean) {
        (Some(base), Some(candidate)) => Some((candidate - base) * 100.0),
        _ => None,
    };
    let all_pairs_cost_per_served_defined =
        all_pairs_completed && zero_service_pairs == 0;
    let baseline_served_cost_mean = if all_pairs_cost_per_served_defined {
        mean(&baseline_cost_per_served)
    } else {
        None
    };
    let candidate_served_cost_mean = if all_pairs_cost_per_served_defined {
        mean(&candidate_cost_per_served)
    } else {
        None
    };
    let served_cost_delta = match (baseline_served_cost_mean, candidate_served_cost_mean) {
        (Some(base), Some(candidate)) => Some(candidate - base),
        _ => None,
    };
    let served_cost_relative_change = match (baseline_served_cost_mean, candidate_served_cost_mean) {
        (Some(base), Some(candidate)) if base != 0.0 => Some((candidate / base - 1.0) * 100.0),
        _ => None,
    };
    let baseline_pooled_cost_per_served = if all_pairs_cost_per_served_defined {
        Some(baseline_total_cost_microcredits as f64 / baseline_total_served_units as f64)
    } else {
        None
    };
    let candidate_pooled_cost_per_served = if all_pairs_cost_per_served_defined {
        Some(candidate_total_cost_microcredits as f64 / candidate_total_served_units as f64)
    } else {
        None
    };
    let pooled_cost_delta = match (baseline_pooled_cost_per_served, candidate_pooled_cost_per_served) {
        (Some(base), Some(candidate)) => Some(candidate - base),
        _ => None,
    };
    let pooled_cost_relative_change = match (baseline_pooled_cost_per_served, candidate_pooled_cost_per_served) {
        (Some(base), Some(candidate)) if base > 0.0 => Some((candidate / base - 1.0) * 100.0),
        _ => None,
    };

    let bootstrap = if all_pairs_cost_per_served_defined {
        stratified_scenario_cluster_bootstrap(
            schedule,
            &results_by_id,
            bootstrap_seed,
            bootstrap_seed_digest,
        )?
    } else {
        None
    };

    Ok(ComparisonReport {
        benchmark_id: schedule.benchmark_id.clone(),
        evidence_class: schedule.evidence_class.clone(),
        baseline_arm: "incumbent_baseline".into(),
        candidate_arm: "full_stack".into(),
        expected_pairs,
        complete_pairs,
        incomplete_pairs,
        zero_service_pairs,
        baseline_mean_cost_per_demand_unit_microcredits: baseline_cost_mean,
        candidate_mean_cost_per_demand_unit_microcredits: candidate_cost_mean,
        candidate_minus_baseline_cost_per_demand_unit_microcredits: paired_cost_delta,
        relative_cost_per_demand_change_percent: relative_cost_change,
        baseline_mean_service_rate: baseline_rate_mean,
        candidate_mean_service_rate: candidate_rate_mean,
        candidate_minus_baseline_service_rate_percentage_points: service_delta_pp,
        baseline_mean_cost_per_served_unit_microcredits: baseline_served_cost_mean,
        candidate_mean_cost_per_served_unit_microcredits: candidate_served_cost_mean,
        candidate_minus_baseline_cost_per_served_unit_microcredits: served_cost_delta,
        relative_cost_per_served_unit_change_percent: served_cost_relative_change,
        baseline_pooled_cost_per_served_unit_microcredits: baseline_pooled_cost_per_served,
        candidate_pooled_cost_per_served_unit_microcredits: candidate_pooled_cost_per_served,
        candidate_minus_baseline_pooled_cost_per_served_unit_microcredits: pooled_cost_delta,
        relative_pooled_cost_per_served_unit_change_percent: pooled_cost_relative_change,
        cost_per_served_unit_defined_for_all_pairs: all_pairs_cost_per_served_defined,
        stratified_scenario_cluster_bootstrap_95: bootstrap,
        descriptive_only: true,
        economic_benefit_established: false,
        claim_ceiling: "descriptive_only; bootstrap_uncertainty_does_not_establish_causal_or_economic_benefit".into(),
    })
}

fn command_generate(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() < 3 || args.len() > 5 {
        return Err("usage: civ-impact-census generate <output-dir> [scenario-count=100] [seed-count=30]".into());
    }
    let scenario_count = args.get(3).map(|v| v.parse()).transpose()?.unwrap_or(100usize);
    let seed_count = args.get(4).map(|v| v.parse()).transpose()?.unwrap_or(30usize);
    let schedule = build_schedule(scenario_count, seed_count)
        .map_err(|error| format!("cannot generate schedule: {error}"))?;
    validate_schedule(&schedule).map_err(|error| format!("generated schedule invalid: {error}"))?;

    let out_dir = Path::new(&args[2]);
    fs::create_dir_all(out_dir)?;
    let scenarios_path = out_dir.join("scenarios.json");
    let schedule_path = out_dir.join("schedule.json");
    let scenario_bytes = serde_json::to_vec(&schedule.scenarios)?;
    fs::write(&scenarios_path, &scenario_bytes)?;
    let schedule_bytes = serde_json::to_vec_pretty(&schedule)?;
    fs::write(&schedule_path, &schedule_bytes)?;
    let schedule_artifact_sha256 = sha256_hex(&schedule_bytes);

    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "status": "schedule_generated_not_executed",
            "benchmark_id": schedule.benchmark_id,
            "evidence_class": schedule.evidence_class,
            "scenario_count": schedule.scenario_count,
            "regime_count": REGIMES.len(),
            "seed_count": schedule.seed_count,
            "arm_count": schedule.arms.len(),
            "scheduled_runs": schedule.runs.len(),
            "scenario_set_sha256": schedule.scenario_set_sha256,
            "schedule_artifact_sha256": schedule_artifact_sha256,
            "scenarios_path": scenarios_path.display().to_string(),
            "schedule_path": schedule_path.display().to_string(),
            "economic_benefit_established": false,
            "claim_ceiling": "deterministic_synthetic_schedule_only"
        }))?
    );
    Ok(())
}

fn command_verify(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() != 4 {
        return Err("usage: civ-impact-census verify <schedule.json> <results.json>".into());
    }
    let schedule_bytes = fs::read(&args[2])?;
    let results_bytes = fs::read(&args[3])?;
    let schedule: Schedule = serde_json::from_slice(&schedule_bytes)?;
    let result_file: ResultsFile = serde_json::from_slice(&results_bytes)?;
    let schedule_artifact_sha256 = sha256_hex(&schedule_bytes);
    let dispositions = verify_census(&schedule, &result_file, &schedule_artifact_sha256)
        .map_err(|error| format!("census verification failed: {error}"))?;
    let completed_runs = *dispositions.get("completed").unwrap_or(&0);
    let total_runs = result_file.runs.len();
    let report = VerificationReport {
        verifier: "civ-impact-census/0.1".into(),
        contract_valid: true,
        benchmark_id: schedule.benchmark_id,
        evidence_class: schedule.evidence_class,
        scenario_count: schedule.scenario_count,
        seed_count: schedule.seed_count,
        arm_count: schedule.arms.len(),
        scheduled_runs: schedule.runs.len(),
        completed_runs,
        noncompleted_runs: total_runs - completed_runs,
        dispositions,
        schedule_artifact_sha256: sha256_hex(&schedule_bytes),
        results_artifact_sha256: sha256_hex(&results_bytes),
        claim_ceiling: "census_integrity_only; does_not_establish_benchmark_or_economic_validity".into(),
        economic_benefit_established: false,
    };
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn command_analyze(args: &[String]) -> Result<(), Box<dyn Error>> {
    if args.len() != 4 {
        return Err("usage: civ-impact-census analyze <schedule.json> <results.json>".into());
    }
    let schedule_bytes = fs::read(&args[2])?;
    let results_bytes = fs::read(&args[3])?;
    let schedule: Schedule = serde_json::from_slice(&schedule_bytes)?;
    let results: ResultsFile = serde_json::from_slice(&results_bytes)?;
    let mut seed_hasher = Sha256::new();
    seed_hasher.update(&schedule_bytes);
    seed_hasher.update(&results_bytes);
    let seed_digest_bytes = seed_hasher.finalize();
    let seed_digest = seed_digest_bytes
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join("");
    let mut seed_prefix = [0u8; 8];
    seed_prefix.copy_from_slice(&seed_digest_bytes[..8]);
    let bootstrap_seed = u64::from_be_bytes(seed_prefix);
    let schedule_artifact_sha256 = sha256_hex(&schedule_bytes);
    let report = make_comparison(
        &schedule,
        &results,
        &schedule_artifact_sha256,
        bootstrap_seed,
        &seed_digest,
    )
    .map_err(|error| format!("paired analysis failed: {error}"))?;
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "report": report,
            "schedule_artifact_sha256": sha256_hex(&schedule_bytes),
            "results_artifact_sha256": sha256_hex(&results_bytes),
            "notice": "Reports descriptive metrics and scenario-cluster resampling intervals for this synthetic corpus only; it does not establish population-level confidence, causal effect, or economic benefit."
        }))?
    );
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("generate") => command_generate(&args),
        Some("verify") => command_verify(&args),
        Some("analyze") => command_analyze(&args),
        _ => Err(
            "usage: civ-impact-census <generate <output-dir> [scenario-count] [seed-count] | verify <schedule.json> <results.json> | analyze <schedule.json> <results.json>>"
                .into(),
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small_schedule() -> Schedule {
        build_schedule(5, 2).expect("small deterministic schedule")
    }

    fn test_schedule_artifact_sha256(schedule: &Schedule) -> String {
        let bytes = serde_json::to_vec_pretty(schedule).expect("serialize test schedule");
        sha256_hex(&bytes)
    }

    fn verify_test_census(
        schedule: &Schedule,
        result_file: &ResultsFile,
    ) -> Result<BTreeMap<String, usize>, String> {
        let schedule_digest = test_schedule_artifact_sha256(schedule);
        verify_census(schedule, result_file, &schedule_digest)
    }

    fn make_test_comparison(
        schedule: &Schedule,
        result_file: &ResultsFile,
        bootstrap_seed: u64,
        bootstrap_seed_digest: &str,
    ) -> Result<ComparisonReport, String> {
        let schedule_digest = test_schedule_artifact_sha256(schedule);
        make_comparison(
            schedule,
            result_file,
            &schedule_digest,
            bootstrap_seed,
            bootstrap_seed_digest,
        )
    }

    fn complete_noncompleted_census(schedule: &Schedule) -> ResultsFile {
        ResultsFile {
            manifest_version: MANIFEST_VERSION.into(),
            benchmark_id: schedule.benchmark_id.clone(),
            scenario_set_sha256: schedule.scenario_set_sha256.clone(),
            schedule_artifact_sha256: test_schedule_artifact_sha256(schedule),
            runs: schedule
                .runs
                .iter()
                .map(|run| RunResult {
                    run_id: run.run_id.clone(),
                    scenario_id: run.scenario_id.clone(),
                    seed: run.seed,
                    arm_id: run.arm_id.clone(),
                    disposition: Disposition::NonConverged,
                    metrics: None,
                    failure_reason: Some("test fixture: no optimizer was executed".into()),
                })
                .collect(),
        }
    }

    #[test]
    fn all_checked_in_schemas_parse_as_draft_2020_12() {
        for source in [
            include_str!("../../../docs/research/schema/CIV_IMPACT_BENCHMARK_V0_1.schema.json"),
            include_str!("../../../docs/research/schema/CIV_IMPACT_RUN_CENSUS_V0_1.schema.json"),
            include_str!("../../../docs/research/schema/CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json"),
        ] {
            let schema: serde_json::Value = serde_json::from_str(source)
                .expect("schema file must parse as JSON");
            assert_eq!(
                schema["$schema"].as_str(),
                Some("https://json-schema.org/draft/2020-12/schema")
            );
            jsonschema::meta::validate(&schema)
                .expect("schema document must validate against the JSON Schema meta-schema");
        }
    }

    #[test]
    fn serialized_run_census_validates_against_its_schema() {
        let schedule = small_schedule();
        let results = complete_noncompleted_census(&schedule);
        let instance = serde_json::to_value(&results).expect("serialize run census");
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/research/schema/CIV_IMPACT_RUN_CENSUS_V0_1.schema.json"
        )).expect("run schema JSON");
        let validator = jsonschema::validator_for(&schema).expect("valid run schema");
        assert!(validator.validate(&instance).is_ok(), "run census instance must match schema");

        let mut missing_schedule_binding = instance.clone();
        missing_schedule_binding.as_object_mut()
            .expect("results envelope object")
            .remove("schedule_artifact_sha256");
        assert!(!validator.is_valid(&missing_schedule_binding));

        let mut missing_operational_cost = instance.clone();
        missing_operational_cost["runs"][0]["metrics"].as_object_mut()
            .expect("metrics object")
            .remove("operational_cost_microcredits");
        assert!(!validator.is_valid(&missing_operational_cost));
    }

    #[test]
    fn serialized_comparison_report_validates_against_its_schema() {
        let schedule = small_schedule();
        let results = complete_results_with_known_cost_tradeoff(&schedule);
        let seed_digest = "e".repeat(64);
        let report = make_test_comparison(&schedule, &results, 9876, &seed_digest)
            .expect("comparison report");
        let schedule_bytes = serde_json::to_vec_pretty(&schedule).expect("schedule bytes");
        let results_bytes = serde_json::to_vec_pretty(&results).expect("result bytes");
        let instance = serde_json::json!({
            "report": report,
            "schedule_artifact_sha256": sha256_hex(&schedule_bytes),
            "results_artifact_sha256": sha256_hex(&results_bytes),
            "notice": "synthetic test fixture; descriptive only"
        });
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/research/schema/CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json"
        )).expect("comparison schema JSON");
        let validator = jsonschema::validator_for(&schema).expect("valid comparison schema");
        assert!(validator.validate(&instance).is_ok(), "comparison report instance must match schema");

        let mut promoted_claim = instance.clone();
        promoted_claim["report"]["descriptive_only"] = serde_json::Value::Bool(false);
        promoted_claim["report"]["economic_benefit_established"] = serde_json::Value::Bool(true);
        assert!(!validator.is_valid(&promoted_claim));
    }

    #[test]
    fn run_result_schema_keys_match_rust_record_shapes() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/research/schema/CIV_IMPACT_RUN_CENSUS_V0_1.schema.json"
        )).expect("run schema JSON");
        let object_keys = |value: &serde_json::Value| -> BTreeSet<String> {
            value.as_object().expect("object").keys().cloned().collect()
        };
        let expected_root: BTreeSet<String> = [
            "manifest_version", "benchmark_id", "scenario_set_sha256", "schedule_artifact_sha256", "runs"
        ].into_iter().map(str::to_owned).collect();
        assert_eq!(object_keys(&schema["properties"]), expected_root);

        let expected_run: BTreeSet<String> = [
            "run_id", "scenario_id", "seed", "arm_id", "disposition", "metrics", "failure_reason"
        ].into_iter().map(str::to_owned).collect();
        assert_eq!(object_keys(&schema["$defs"]["runResult"]["properties"]), expected_run);

        let expected_metrics: BTreeSet<String> = [
            "observed_demand_units", "served_units", "compute_cost_microcredits",
            "operational_cost_microcredits", "capital_amortization_cost_microcredits",
            "data_cost_microcredits", "integration_cost_microcredits",
            "human_review_cost_microcredits", "coordination_cost_microcredits",
            "failure_recovery_cost_microcredits", "total_cost_microcredits",
            "completion_latency_ms"
        ].into_iter().map(str::to_owned).collect();
        assert_eq!(object_keys(&schema["$defs"]["runMetrics"]["properties"]), expected_metrics);
    }

    #[test]
    fn comparison_schema_keys_match_report_and_bootstrap_shapes() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../../../docs/research/schema/CIV_IMPACT_COMPARISON_REPORT_V0_1.schema.json"
        )).expect("comparison schema JSON");
        let object_keys = |value: &serde_json::Value| -> BTreeSet<String> {
            value.as_object().expect("object").keys().cloned().collect()
        };
        let expected_report: BTreeSet<String> = [
            "benchmark_id", "evidence_class", "baseline_arm", "candidate_arm",
            "expected_pairs", "complete_pairs", "incomplete_pairs", "zero_service_pairs",
            "baseline_mean_cost_per_demand_unit_microcredits",
            "candidate_mean_cost_per_demand_unit_microcredits",
            "candidate_minus_baseline_cost_per_demand_unit_microcredits",
            "relative_cost_per_demand_change_percent",
            "baseline_mean_service_rate", "candidate_mean_service_rate",
            "candidate_minus_baseline_service_rate_percentage_points",
            "baseline_mean_cost_per_served_unit_microcredits",
            "candidate_mean_cost_per_served_unit_microcredits",
            "candidate_minus_baseline_cost_per_served_unit_microcredits",
            "relative_cost_per_served_unit_change_percent",
            "baseline_pooled_cost_per_served_unit_microcredits",
            "candidate_pooled_cost_per_served_unit_microcredits",
            "candidate_minus_baseline_pooled_cost_per_served_unit_microcredits",
            "relative_pooled_cost_per_served_unit_change_percent",
            "cost_per_served_unit_defined_for_all_pairs",
            "stratified_scenario_cluster_bootstrap_95",
            "descriptive_only", "economic_benefit_established", "claim_ceiling"
        ].into_iter().map(str::to_owned).collect();
        let report_properties = &schema["$defs"]["comparisonReport"]["properties"];
        assert_eq!(object_keys(report_properties), expected_report);
        let required_report: BTreeSet<String> = schema["$defs"]["comparisonReport"]["required"]
            .as_array()
            .expect("report required array")
            .iter()
            .map(|value| value.as_str().expect("required key").to_owned())
            .collect();
        assert_eq!(object_keys(report_properties), required_report);

        let bootstrap_properties = &report_properties["stratified_scenario_cluster_bootstrap_95"]["oneOf"][1]["properties"];
        let expected_bootstrap: BTreeSet<String> = [
            "method", "resamples", "central_interval_level", "resampling_unit",
            "stratification", "bootstrap_seed_sha256",
            "relative_pooled_cost_per_served_unit_change_percent",
            "candidate_minus_baseline_pooled_cost_per_served_unit_microcredits",
            "candidate_minus_baseline_service_rate_percentage_points", "claim_ceiling"
        ].into_iter().map(str::to_owned).collect();
        assert_eq!(object_keys(bootstrap_properties), expected_bootstrap);
    }

    #[test]
    fn default_schedule_has_balanced_regimes_and_paired_arms() {
        let schedule = build_schedule(100, 30).expect("default schedule");
        assert_eq!(schedule.scenarios.len(), 100);
        assert_eq!(schedule.seed_count, 30);
        assert_eq!(schedule.arms.len(), 7);
        assert_eq!(schedule.runs.len(), 21_000);
        validate_schedule(&schedule).expect("schedule invariants");
        for regime in REGIMES {
            assert_eq!(
                schedule.scenarios.iter().filter(|s| s.regime == regime).count(),
                20
            );
        }
    }

    #[test]
    fn rejects_overflowing_schedule_before_large_allocation() {
        assert!(build_schedule(usize::MAX, 1).unwrap_err().contains("overflow"));
    }

    #[test]
    fn rng_range_is_bounded_and_singleton_ranges_are_exact() {
        let mut rng = XorShift64::new(7);
        for _ in 0..10_000 {
            assert_eq!(rng.range(19, 20), 19);
            let sampled = rng.range(3, 13);
            assert!((3..13).contains(&sampled));
        }
    }

    #[test]
    fn identical_input_generates_identical_scenario_identity() {
        let left = build_schedule(10, 3).expect("left");
        let right = build_schedule(10, 3).expect("right");
        assert_eq!(left.scenario_set_sha256, right.scenario_set_sha256);
        assert_eq!(
            serde_json::to_vec(&left.runs).expect("serialize left"),
            serde_json::to_vec(&right.runs).expect("serialize right")
        );
    }

    #[test]
    fn rejects_unbalanced_scenario_regimes() {
        assert!(build_schedule(6, 1).is_err());
    }

    #[test]
    fn rejects_rehashed_schedule_with_unbalanced_regimes() {
        let mut schedule = build_schedule(10, 1).expect("balanced schedule");
        schedule.scenarios[0].regime = "demand_shock".into();
        schedule.scenarios[0].demand_shock_bps = 3_000;
        let bytes = serde_json::to_vec(&schedule.scenarios).expect("serialize changed scenarios");
        schedule.scenario_set_sha256 = sha256_hex(&bytes);
        assert!(validate_schedule(&schedule).unwrap_err().contains("balance all five"));
    }

    #[test]
    fn rejects_modified_full_stack_arm_composition() {
        let mut schedule = small_schedule();
        let full_stack = schedule.arms.iter_mut()
            .find(|arm| arm.arm_id == "full_stack")
            .expect("full stack arm");
        full_stack.enabled_components.pop();
        assert!(validate_schedule(&schedule).unwrap_err().contains("registered component set"));
    }

    #[test]
    fn complete_run_census_is_not_a_benefit_claim() {
        let schedule = small_schedule();
        let results = complete_noncompleted_census(&schedule);
        let counts = verify_test_census(&schedule, &results).expect("complete census");
        assert_eq!(counts.get("non_converged"), Some(&70));
        assert_eq!(counts.get("completed"), None);
    }

    #[test]
    fn rejects_missing_result_even_when_other_rows_are_valid() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs.pop();
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("length mismatch"));
    }

    #[test]
    fn rejects_duplicate_result_id() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs[1] = results.runs[0].clone();
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("duplicate result run_id"));
    }

    #[test]
    fn rejects_results_replayed_against_changed_schedule_artifact() {
        let schedule = small_schedule();
        let results = complete_noncompleted_census(&schedule);
        let mut changed_schedule = schedule.clone();
        changed_schedule.arms[0].implementation_ref =
            "UNQUALIFIED:changed-implementation-ref".into();
        let changed_digest = test_schedule_artifact_sha256(&changed_schedule);
        assert_ne!(changed_digest, results.schedule_artifact_sha256);
        let err = verify_census(&changed_schedule, &results, &changed_digest)
            .unwrap_err();
        assert!(err.contains("schedule_artifact_sha256"));
    }

    #[test]
    fn rejects_scenario_set_digest_mismatch() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.scenario_set_sha256 = "0".repeat(64);
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("scenario_set_sha256"));
    }

    #[test]
    fn rejects_changed_scenario_without_updated_digest() {
        let mut schedule = small_schedule();
        schedule.scenarios[0].demand_units += 1;
        assert!(validate_schedule(&schedule).unwrap_err().contains("scenario_set_sha256"));
    }

    #[test]
    fn rejects_cost_sum_mismatch_on_completed_result() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        let scenario = &schedule.scenarios[0];
        results.runs[0].disposition = Disposition::Completed;
        results.runs[0].failure_reason = None;
        results.runs[0].metrics = Some(RunMetrics {
            observed_demand_units: scenario.demand_units,
            served_units: scenario.demand_units,
            compute_cost_microcredits: 10,
            operational_cost_microcredits: 0,
            capital_amortization_cost_microcredits: 0,
            data_cost_microcredits: 10,
            integration_cost_microcredits: 10,
            human_review_cost_microcredits: 10,
            coordination_cost_microcredits: 10,
            failure_recovery_cost_microcredits: 10,
            total_cost_microcredits: 59,
            completion_latency_ms: 100,
        });
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("total cost mismatch"));
    }

    #[test]
    fn operational_and_capital_costs_are_included_in_total() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        let scenario = &schedule.scenarios[0];
        let run_index = 0usize;
        let metrics = RunMetrics {
            observed_demand_units: effective_demand(scenario).expect("effective demand"),
            served_units: effective_demand(scenario).expect("effective demand"),
            compute_cost_microcredits: 1,
            operational_cost_microcredits: 4,
            capital_amortization_cost_microcredits: 5,
            data_cost_microcredits: 0,
            integration_cost_microcredits: 0,
            human_review_cost_microcredits: 0,
            coordination_cost_microcredits: 0,
            failure_recovery_cost_microcredits: 0,
            total_cost_microcredits: 10,
            completion_latency_ms: 1,
        };
        results.runs[run_index].disposition = Disposition::Completed;
        results.runs[run_index].failure_reason = None;
        results.runs[run_index].metrics = Some(metrics);
        verify_test_census(&schedule, &results)
            .expect("sum includes operational and capital amortization costs");

        results.runs[run_index].metrics.as_mut()
            .expect("metrics").total_cost_microcredits = 9;
        assert!(verify_test_census(&schedule, &results)
            .unwrap_err().contains("total cost mismatch"));
    }

    #[test]
    fn rejects_served_units_above_demand() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        let scenario = &schedule.scenarios[0];
        let total = 6u64;
        results.runs[0].disposition = Disposition::Completed;
        results.runs[0].failure_reason = None;
        results.runs[0].metrics = Some(RunMetrics {
            observed_demand_units: scenario.demand_units,
            served_units: scenario.demand_units + 1,
            compute_cost_microcredits: 1,
            operational_cost_microcredits: 0,
            capital_amortization_cost_microcredits: 0,
            data_cost_microcredits: 1,
            integration_cost_microcredits: 1,
            human_review_cost_microcredits: 1,
            coordination_cost_microcredits: 1,
            failure_recovery_cost_microcredits: 1,
            total_cost_microcredits: total,
            completion_latency_ms: 100,
        });
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("served_units exceeds"));
    }

    #[test]
    fn accepts_completed_result_when_costs_reconcile_and_shock_is_applied() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        let scenario = schedule.scenarios.iter().find(|s| s.regime == "demand_shock").expect("demand shock");
        let expected_demand = effective_demand(scenario).expect("effective demand");
        let index = schedule.runs.iter().position(|run| {
            run.scenario_id == scenario.scenario_id
                && run.seed == 1
                && run.arm_id == "incumbent_baseline"
        }).expect("scheduled matching run");
        results.runs[index].disposition = Disposition::Completed;
        results.runs[index].failure_reason = None;
        results.runs[index].metrics = Some(RunMetrics {
            observed_demand_units: expected_demand,
            served_units: expected_demand.saturating_sub(3),
            compute_cost_microcredits: 1,
            operational_cost_microcredits: 0,
            capital_amortization_cost_microcredits: 0,
            data_cost_microcredits: 2,
            integration_cost_microcredits: 3,
            human_review_cost_microcredits: 4,
            coordination_cost_microcredits: 5,
            failure_recovery_cost_microcredits: 6,
            total_cost_microcredits: 21,
            completion_latency_ms: 100,
        });
        let counts = verify_test_census(&schedule, &results).expect("valid completed result");
        assert_eq!(counts.get("completed"), Some(&1));
        assert_eq!(counts.get("non_converged"), Some(&(schedule.runs.len() - 1)));
    }

    fn complete_results_with_known_cost_tradeoff(schedule: &Schedule) -> ResultsFile {
        let runs = schedule.runs.iter().map(|run| {
            let scenario = schedule.scenarios.iter()
                .find(|scenario| scenario.scenario_id == run.scenario_id)
                .expect("scheduled scenario");
            let demand = effective_demand(scenario).expect("effective demand");
            let unit_cost = match run.arm_id.as_str() {
                "incumbent_baseline" => 100u64,
                "full_stack" => 90u64,
                _ => 95u64,
            };
            let served_units = if run.arm_id == "full_stack" {
                demand.saturating_sub(demand / 20)
            } else {
                demand
            };
            let total_cost = demand * unit_cost;
            RunResult {
                run_id: run.run_id.clone(),
                scenario_id: run.scenario_id.clone(),
                seed: run.seed,
                arm_id: run.arm_id.clone(),
                disposition: Disposition::Completed,
                metrics: Some(RunMetrics {
                    observed_demand_units: demand,
                    served_units,
                    compute_cost_microcredits: total_cost,
                    operational_cost_microcredits: 0,
                    capital_amortization_cost_microcredits: 0,
                    data_cost_microcredits: 0,
                    integration_cost_microcredits: 0,
                    human_review_cost_microcredits: 0,
                    coordination_cost_microcredits: 0,
                    failure_recovery_cost_microcredits: 0,
                    total_cost_microcredits: total_cost,
                    completion_latency_ms: 100,
                }),
                failure_reason: None,
            }
        }).collect();
        ResultsFile {
            manifest_version: MANIFEST_VERSION.into(),
            benchmark_id: schedule.benchmark_id.clone(),
            scenario_set_sha256: schedule.scenario_set_sha256.clone(),
            schedule_artifact_sha256: test_schedule_artifact_sha256(schedule),
            runs,
        }
    }

    #[test]
    fn reports_paired_cost_and_service_tradeoff_without_claiming_benefit() {
        let schedule = small_schedule();
        let results = complete_results_with_known_cost_tradeoff(&schedule);
        let report = make_test_comparison(&schedule, &results, 42, &"a".repeat(64)).expect("paired descriptive comparison");
        assert_eq!(report.expected_pairs, 10);
        assert_eq!(report.complete_pairs, 10);
        assert_eq!(report.incomplete_pairs, 0);
        assert_eq!(report.zero_service_pairs, 0);
        assert!(report.cost_per_served_unit_defined_for_all_pairs);
        assert!((report.relative_cost_per_demand_change_percent.expect("cost delta") + 10.0).abs() < 1e-9);
        let served_cost_change = report.relative_cost_per_served_unit_change_percent.expect("served cost delta");
        assert!(served_cost_change < 0.0 && served_cost_change > -10.0);
        assert!(report.candidate_minus_baseline_service_rate_percentage_points.expect("service delta") < 0.0);
        let baseline_cost = results.runs.iter()
            .filter(|row| row.arm_id == "incumbent_baseline")
            .map(|row| row.metrics.as_ref().expect("completed baseline metrics").total_cost_microcredits as u128)
            .sum::<u128>();
        let baseline_served = results.runs.iter()
            .filter(|row| row.arm_id == "incumbent_baseline")
            .map(|row| row.metrics.as_ref().expect("completed baseline metrics").served_units as u128)
            .sum::<u128>();
        let candidate_cost = results.runs.iter()
            .filter(|row| row.arm_id == "full_stack")
            .map(|row| row.metrics.as_ref().expect("completed full-stack metrics").total_cost_microcredits as u128)
            .sum::<u128>();
        let candidate_served = results.runs.iter()
            .filter(|row| row.arm_id == "full_stack")
            .map(|row| row.metrics.as_ref().expect("completed full-stack metrics").served_units as u128)
            .sum::<u128>();
        let expected_pooled_change = (
            (candidate_cost as f64 / candidate_served as f64)
                / (baseline_cost as f64 / baseline_served as f64)
                - 1.0
        ) * 100.0;
        assert!((report.relative_pooled_cost_per_served_unit_change_percent
            .expect("pooled cost delta") - expected_pooled_change).abs() < 1e-9);
        assert!(!report.economic_benefit_established);
        assert!(report.descriptive_only);
        let interval = report.stratified_scenario_cluster_bootstrap_95
            .expect("complete paired corpus has bootstrap uncertainty");
        assert_eq!(interval.resamples, BOOTSTRAP_RESAMPLES);
        assert_eq!(interval.central_interval_level, 0.95);
        assert!(interval.relative_pooled_cost_per_served_unit_change_percent.lower_2_5_percentile <=
            interval.relative_pooled_cost_per_served_unit_change_percent.upper_97_5_percentile);
        assert_eq!(interval.bootstrap_seed_sha256, "a".repeat(64));
    }

    #[test]
    fn service_rate_bootstrap_interval_matches_equal_scenario_point_estimand() {
        let mut schedule = small_schedule();
        for scenario in &mut schedule.scenarios {
            scenario.demand_units = 10_000;
        }
        let scenario_bytes = serde_json::to_vec(&schedule.scenarios)
            .expect("serialize modified scenario set");
        schedule.scenario_set_sha256 = sha256_hex(&scenario_bytes);

        let results = complete_results_with_known_cost_tradeoff(&schedule);
        let report = make_test_comparison(&schedule, &results, 456, &"f".repeat(64))
            .expect("uniform-rate comparison");
        let point = report
            .candidate_minus_baseline_service_rate_percentage_points
            .expect("point service-rate delta");
        let interval = report.stratified_scenario_cluster_bootstrap_95
            .expect("complete schedule produces bootstrap interval");
        assert!((point + 5.0).abs() < 1e-9);
        assert!((interval.candidate_minus_baseline_service_rate_percentage_points
            .lower_2_5_percentile + 5.0).abs() < 1e-9);
        assert!((interval.candidate_minus_baseline_service_rate_percentage_points
            .upper_97_5_percentile + 5.0).abs() < 1e-9);
    }

    #[test]
    fn bootstrap_interval_is_reproducible_for_identical_artifacts() {
        let schedule = build_schedule(10, 3).expect("larger schedule with multiple scenarios per stratum");
        let mut results = complete_results_with_known_cost_tradeoff(&schedule);
        for row in &mut results.runs {
            let scenario_index: u64 = row.scenario_id
                .trim_start_matches("syn-transport-")
                .parse()
                .expect("synthetic scenario index");
            let scenario = schedule.scenarios.iter()
                .find(|item| item.scenario_id == row.scenario_id)
                .expect("scenario exists");
            let demand = effective_demand(scenario).expect("effective demand");
            if row.disposition == Disposition::Completed {
                let metrics = row.metrics.as_mut().expect("completed metrics");
                let unit_cost = match row.arm_id.as_str() {
                    "incumbent_baseline" => 100 + (scenario_index % 7),
                    "full_stack" => 88 + (scenario_index % 11),
                    _ => 95,
                };
                metrics.total_cost_microcredits = demand * unit_cost;
                metrics.compute_cost_microcredits = metrics.total_cost_microcredits;
                metrics.served_units = if row.arm_id == "full_stack" {
                    demand.saturating_sub(demand / (20 + scenario_index % 5))
                } else {
                    demand
                };
            }
        }
        let seed_digest = "b".repeat(64);
        let first = make_test_comparison(&schedule, &results, 12345, &seed_digest)
            .expect("first analysis");
        let second = make_test_comparison(&schedule, &results, 12345, &seed_digest)
            .expect("repeat analysis");
        let first_interval = first.stratified_scenario_cluster_bootstrap_95
            .expect("bootstrap interval");
        let second_interval = second.stratified_scenario_cluster_bootstrap_95
            .expect("repeat bootstrap interval");
        assert_eq!(
            first_interval.relative_pooled_cost_per_served_unit_change_percent.lower_2_5_percentile,
            second_interval.relative_pooled_cost_per_served_unit_change_percent.lower_2_5_percentile
        );
        assert_eq!(
            first_interval.relative_pooled_cost_per_served_unit_change_percent.upper_97_5_percentile,
            second_interval.relative_pooled_cost_per_served_unit_change_percent.upper_97_5_percentile
        );
    }

    #[test]
    fn incomplete_pair_census_is_reported_not_silently_dropped() {
        let schedule = small_schedule();
        let results = complete_noncompleted_census(&schedule);
        let report = make_test_comparison(&schedule, &results, 42, &"a".repeat(64)).expect("valid but uncompleted census");
        assert_eq!(report.complete_pairs, 0);
        assert_eq!(report.incomplete_pairs, report.expected_pairs);
        assert_eq!(report.candidate_mean_cost_per_demand_unit_microcredits, None);
        assert!(report.stratified_scenario_cluster_bootstrap_95.is_none());
        assert!(!report.economic_benefit_established);
    }

    #[test]
    fn withholds_headline_means_when_any_pair_is_incomplete() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        let scenario = &schedule.scenarios[0];
        let demand = effective_demand(scenario).expect("effective demand");
        for arm_id in ["incumbent_baseline", "full_stack"] {
            let run_index = schedule.runs.iter().position(|run| {
                run.scenario_id == scenario.scenario_id
                    && run.seed == 1
                    && run.arm_id == arm_id
            }).expect("registered paired run");
            results.runs[run_index].disposition = Disposition::Completed;
            results.runs[run_index].failure_reason = None;
            results.runs[run_index].metrics = Some(RunMetrics {
                observed_demand_units: demand,
                served_units: demand,
                compute_cost_microcredits: 60,
                operational_cost_microcredits: 0,
                capital_amortization_cost_microcredits: 0,
                data_cost_microcredits: 0,
                integration_cost_microcredits: 0,
                human_review_cost_microcredits: 0,
                coordination_cost_microcredits: 0,
                failure_recovery_cost_microcredits: 0,
                total_cost_microcredits: 60,
                completion_latency_ms: 10,
            });
        }
        let report = make_test_comparison(&schedule, &results, 42, &"d".repeat(64))
            .expect("partial-census report");
        assert_eq!(report.complete_pairs, 1);
        assert_eq!(report.incomplete_pairs, report.expected_pairs - 1);
        assert_eq!(report.baseline_mean_cost_per_demand_unit_microcredits, None);
        assert_eq!(report.candidate_mean_cost_per_demand_unit_microcredits, None);
        assert_eq!(report.baseline_mean_service_rate, None);
        assert_eq!(report.candidate_mean_service_rate, None);
        assert!(report.stratified_scenario_cluster_bootstrap_95.is_none());
    }

    #[test]
    fn zero_service_makes_cost_per_served_unit_undefined_for_report() {
        let schedule = small_schedule();
        let mut results = complete_results_with_known_cost_tradeoff(&schedule);
        let row_index = schedule.runs.iter().position(|run| {
            run.scenario_id == schedule.scenarios[0].scenario_id
                && run.seed == 1
                && run.arm_id == "full_stack"
        }).expect("full-stack row");
        let metrics = results.runs[row_index].metrics.as_mut().expect("metrics");
        metrics.served_units = 0;
        let report = make_test_comparison(&schedule, &results, 42, &"a".repeat(64)).expect("valid zero-service outcome");
        assert_eq!(report.zero_service_pairs, 1);
        assert!(!report.cost_per_served_unit_defined_for_all_pairs);
        assert_eq!(report.candidate_mean_cost_per_served_unit_microcredits, None);
        assert!(report.stratified_scenario_cluster_bootstrap_95.is_none());
    }

    #[test]
    fn withholds_relative_bootstrap_when_baseline_cost_is_zero() {
        let schedule = small_schedule();
        let mut results = complete_results_with_known_cost_tradeoff(&schedule);
        for row in &mut results.runs {
            if row.arm_id == "incumbent_baseline" {
                let metrics = row.metrics.as_mut().expect("completed baseline metrics");
                metrics.compute_cost_microcredits = 0;
                metrics.data_cost_microcredits = 0;
                metrics.integration_cost_microcredits = 0;
                metrics.human_review_cost_microcredits = 0;
                metrics.coordination_cost_microcredits = 0;
                metrics.failure_recovery_cost_microcredits = 0;
                metrics.total_cost_microcredits = 0;
            }
        }
        let report = make_test_comparison(&schedule, &results, 42, &"c".repeat(64))
            .expect("zero-baseline-cost records remain internally consistent");
        assert!(report.cost_per_served_unit_defined_for_all_pairs);
        assert_eq!(report.relative_pooled_cost_per_served_unit_change_percent, None);
        assert!(report.stratified_scenario_cluster_bootstrap_95.is_none());
    }

    #[test]
    fn rejects_failure_without_reason() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs[0].failure_reason = None;
        assert!(verify_test_census(&schedule, &results).unwrap_err().contains("failure_reason"));
    }
}
