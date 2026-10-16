//! Deterministic CIV-IMPACT synthetic schedule generator and run-census verifier.
//!
//! This tool creates a preregisterable schedule; it does not implement a route
//! optimizer, simulate operational outcomes, or establish economic benefit.
//! A valid census proves completeness/contract consistency only.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashMap, HashSet};
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
        lower + (self.next() % (upper_exclusive - lower))
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|b| format!("{b:02x}")).collect()
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
    let scenarios = build_scenarios(scenario_count)?;
    let arms = build_arms();
    let expected = scenario_count
        .checked_mul(seed_count)
        .and_then(|v| v.checked_mul(arms.len()))
        .ok_or_else(|| "planned-run count overflow".to_string())?;
    if expected > MAX_RUNS {
        return Err(format!("planned run count {expected} exceeds safety cap {MAX_RUNS}"));
    }

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
        if scenario.missing_spatial_permille > 1_000
            || scenario.demand_shock_bps > 20_000
            || scenario.capacity_loss_bps > 10_000
        {
            return Err(format!("scenario {} has out-of-range stress parameters", scenario.scenario_id));
        }
        if scenario_by_id.insert(scenario.scenario_id.as_str(), scenario).is_some() {
            return Err(format!("duplicate scenario_id: {}", scenario.scenario_id));
        }
        *regime_counts.entry(scenario.regime.clone()).or_default() += 1;
    }
    if regime_counts.len() != REGIMES.len() || regime_counts.values().any(|count| *count == 0) {
        return Err("schedule must contain all five named stress regimes".into());
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
    if !arm_by_id.contains_key("incumbent_baseline") || !arm_by_id.contains_key("full_stack") {
        return Err("arms must include incumbent_baseline and full_stack".into());
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

fn verify_census(schedule: &Schedule, result_file: &ResultsFile) -> Result<BTreeMap<String, usize>, String> {
    validate_schedule(schedule)?;
    if result_file.manifest_version != MANIFEST_VERSION {
        return Err("unsupported results manifest_version".into());
    }
    if result_file.benchmark_id != schedule.benchmark_id {
        return Err("results benchmark_id does not match schedule".into());
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

fn write_json(path: &Path, value: &impl Serialize) -> Result<(), Box<dyn Error>> {
    let bytes = serde_json::to_vec_pretty(value)?;
    fs::write(path, bytes)?;
    Ok(())
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
    write_json(&schedule_path, &schedule)?;

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
    let dispositions = verify_census(&schedule, &result_file)
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

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("generate") => command_generate(&args),
        Some("verify") => command_verify(&args),
        _ => Err(
            "usage: civ-impact-census <generate <output-dir> [scenario-count] [seed-count] | verify <schedule.json> <results.json>>"
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

    fn complete_noncompleted_census(schedule: &Schedule) -> ResultsFile {
        ResultsFile {
            manifest_version: MANIFEST_VERSION.into(),
            benchmark_id: schedule.benchmark_id.clone(),
            scenario_set_sha256: schedule.scenario_set_sha256.clone(),
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
    fn complete_run_census_is_not_a_benefit_claim() {
        let schedule = small_schedule();
        let results = complete_noncompleted_census(&schedule);
        let counts = verify_census(&schedule, &results).expect("complete census");
        assert_eq!(counts.get("non_converged"), Some(&70));
        assert_eq!(counts.get("completed"), None);
    }

    #[test]
    fn rejects_missing_result_even_when_other_rows_are_valid() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs.pop();
        assert!(verify_census(&schedule, &results).unwrap_err().contains("length mismatch"));
    }

    #[test]
    fn rejects_duplicate_result_id() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs[1] = results.runs[0].clone();
        assert!(verify_census(&schedule, &results).unwrap_err().contains("duplicate result run_id"));
    }

    #[test]
    fn rejects_scenario_set_digest_mismatch() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.scenario_set_sha256 = "0".repeat(64);
        assert!(verify_census(&schedule, &results).unwrap_err().contains("scenario_set_sha256"));
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
            data_cost_microcredits: 10,
            integration_cost_microcredits: 10,
            human_review_cost_microcredits: 10,
            coordination_cost_microcredits: 10,
            failure_recovery_cost_microcredits: 10,
            total_cost_microcredits: 59,
            completion_latency_ms: 100,
        });
        assert!(verify_census(&schedule, &results).unwrap_err().contains("total cost mismatch"));
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
            data_cost_microcredits: 1,
            integration_cost_microcredits: 1,
            human_review_cost_microcredits: 1,
            coordination_cost_microcredits: 1,
            failure_recovery_cost_microcredits: 1,
            total_cost_microcredits: total,
            completion_latency_ms: 100,
        });
        assert!(verify_census(&schedule, &results).unwrap_err().contains("served_units exceeds"));
    }

    #[test]
    fn rejects_failure_without_reason() {
        let schedule = small_schedule();
        let mut results = complete_noncompleted_census(&schedule);
        results.runs[0].failure_reason = None;
        assert!(verify_census(&schedule, &results).unwrap_err().contains("failure_reason"));
    }
}
