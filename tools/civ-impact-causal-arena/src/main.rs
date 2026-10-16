use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::{env, process};

const REPORT_SCHEMA: &str =
    include_str!("../../../docs/research/schema/CIV_IMPACT_CAUSAL_ARENA_REPORT_V0_1.schema.json");
const SOURCE: &str = include_str!("main.rs");
const Z95: f64 = 1.959_963_984_540_054;

#[derive(Clone, Copy)]
enum Effect {
    Constant(f64),
    Heterogeneous,
}

#[derive(Clone, Copy)]
struct World {
    id: &'static str,
    gamma: f64,
    effect: Effect,
    confounding_risk: bool,
}

const WORLDS: [World; 5] = [
    World {
        id: "clean_positive",
        gamma: 0.0,
        effect: Effect::Constant(1.5),
        confounding_risk: false,
    },
    World {
        id: "hidden_confounding",
        gamma: 3.0,
        effect: Effect::Constant(1.0),
        confounding_risk: true,
    },
    World {
        id: "sign_reversal",
        gamma: 3.0,
        effect: Effect::Constant(-1.0),
        confounding_risk: true,
    },
    World {
        id: "null_effect",
        gamma: 3.0,
        effect: Effect::Constant(0.0),
        confounding_risk: true,
    },
    World {
        id: "heterogeneous_effect",
        gamma: 2.5,
        effect: Effect::Heterogeneous,
        confounding_risk: true,
    },
];

const STRATEGIES: [&str; 6] = [
    "randomized_naive",
    "observational_naive",
    "observational_adjusted",
    "stratified_randomized",
    "cue_guided",
    "always_abstain",
];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Design {
    Randomized,
    StratifiedRandomized,
    Observational,
    AdjustedObservational,
    None,
}

impl Design {
    fn name(self) -> &'static str {
        match self {
            Self::Randomized => "randomized",
            Self::StratifiedRandomized => "stratified_randomized",
            Self::Observational => "observational",
            Self::AdjustedObservational => "adjusted_observational",
            Self::None => "none",
        }
    }
}

#[derive(Clone)]
struct Unit {
    confounder: bool,
    y0: f64,
    y1: f64,
    treatment: Option<bool>,
}

#[derive(Clone)]
struct CellResult {
    json: Value,
    regime: &'static str,
    strategy: &'static str,
    truth: f64,
    estimate: Option<f64>,
    standard_error: Option<f64>,
    interval_covers_truth: Option<bool>,
    directional_claim: &'static str,
    correct_sign: Option<bool>,
    cost: f64,
    causally_identified: bool,
}

#[derive(Clone, Copy)]
struct Rng {
    state: u64,
}

impl Rng {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    // Deterministic SplitMix64. This is a replayable simulator RNG, not a security RNG.
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64) * (1.0 / 9_007_199_254_740_992.0)
    }
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn true_effect(world: World, confounder: bool) -> f64 {
    match world.effect {
        Effect::Constant(value) => value,
        Effect::Heterogeneous => {
            if confounder { 2.2 } else { 0.2 }
        }
    }
}

fn make_units(seed: u64, world: World, count: usize) -> Vec<Unit> {
    let mut rng = Rng::new(seed ^ 0x2D35_8DCC_A6C7_8A3B);
    (0..count)
        .map(|_| {
            let confounder = rng.uniform() < 0.5;
            let noise = (rng.uniform() * 2.0 - 1.0) * 1.2;
            let u_value = if confounder { 1.0 } else { 0.0 };
            let y0 = 1.0 + world.gamma * u_value + noise;
            let y1 = y0 + true_effect(world, confounder);
            Unit {
                confounder,
                y0,
                y1,
                treatment: None,
            }
        })
        .collect()
}

fn confounding_signal(seed: u64, world: World) -> bool {
    let mut rng = Rng::new(seed ^ 0xA24B_AED4_963E_E407);
    let false_positive_or_sensitivity = if world.confounding_risk { 0.8 } else { 0.2 };
    rng.uniform() < false_positive_or_sensitivity
}

fn choose_design(strategy: &str, high_signal: bool) -> (Design, bool) {
    match strategy {
        "randomized_naive" => (Design::Randomized, false),
        "observational_naive" => (Design::Observational, false),
        "observational_adjusted" => (Design::AdjustedObservational, true),
        "stratified_randomized" => (Design::StratifiedRandomized, true),
        "cue_guided" if high_signal => (Design::StratifiedRandomized, true),
        "cue_guided" => (Design::Observational, false),
        "always_abstain" => (Design::None, false),
        _ => (Design::None, false),
    }
}

fn assign_treatment(units: &mut [Unit], seed: u64, design: Design) {
    let mut rng = Rng::new(seed ^ match design {
        Design::Randomized | Design::StratifiedRandomized => 0xB4C3_9A7F_1D26_E805,
        Design::Observational | Design::AdjustedObservational => 0x7812_6D2B_9F03_A4C5,
        Design::None => 0xC6BC_2796_92B5_C323,
    });

    match design {
        Design::Randomized => {
            for unit in units {
                unit.treatment = Some(rng.uniform() < 0.5);
            }
        }
        Design::Observational | Design::AdjustedObservational => {
            for unit in units {
                let probability = if unit.confounder { 0.85 } else { 0.15 };
                unit.treatment = Some(rng.uniform() < probability);
            }
        }
        Design::StratifiedRandomized => {
            // Randomize exactly half (rounded down) to treatment within each measured stratum.
            for stratum in [false, true] {
                let mut priorities: Vec<(f64, usize)> = units
                    .iter()
                    .enumerate()
                    .filter(|(_, unit)| unit.confounder == stratum)
                    .map(|(index, _)| (rng.uniform(), index))
                    .collect();
                priorities.sort_by(|a, b| a.0.total_cmp(&b.0));
                let treated_count = priorities.len() / 2;
                for (rank, (_, index)) in priorities.into_iter().enumerate() {
                    units[index].treatment = Some(rank < treated_count);
                }
            }
        }
        Design::None => {}
    }
}

fn mean_variance(values: &[f64]) -> Option<(f64, f64)> {
    if values.len() < 2 {
        return None;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values.iter().map(|value| (value - mean).powi(2)).sum::<f64>()
        / (values.len() - 1) as f64;
    Some((mean, variance))
}

fn estimate_naive(units: &[Unit]) -> Option<(f64, f64)> {
    let treated: Vec<f64> = units
        .iter()
        .filter(|unit| unit.treatment == Some(true))
        .map(|unit| unit.y1)
        .collect();
    let control: Vec<f64> = units
        .iter()
        .filter(|unit| unit.treatment == Some(false))
        .map(|unit| unit.y0)
        .collect();
    let (treated_mean, treated_variance) = mean_variance(&treated)?;
    let (control_mean, control_variance) = mean_variance(&control)?;
    let se = (treated_variance / treated.len() as f64
        + control_variance / control.len() as f64)
        .sqrt();
    Some((treated_mean - control_mean, se))
}

fn estimate_standardized(units: &[Unit]) -> Option<(f64, f64)> {
    let mut estimate = 0.0;
    let mut variance = 0.0;
    for stratum in [false, true] {
        let cohort = units.iter().filter(|unit| unit.confounder == stratum).count();
        if cohort == 0 {
            return None;
        }
        let treated: Vec<f64> = units
            .iter()
            .filter(|unit| unit.confounder == stratum && unit.treatment == Some(true))
            .map(|unit| unit.y1)
            .collect();
        let control: Vec<f64> = units
            .iter()
            .filter(|unit| unit.confounder == stratum && unit.treatment == Some(false))
            .map(|unit| unit.y0)
            .collect();
        let (treated_mean, treated_variance) = mean_variance(&treated)?;
        let (control_mean, control_variance) = mean_variance(&control)?;
        let weight = cohort as f64 / units.len() as f64;
        estimate += weight * (treated_mean - control_mean);
        variance += weight.powi(2)
            * (treated_variance / treated.len() as f64
                + control_variance / control.len() as f64);
    }
    Some((estimate, variance.sqrt()))
}

fn estimated_effect(units: &[Unit], adjusted: bool) -> Option<(f64, f64)> {
    if adjusted {
        estimate_standardized(units)
    } else {
        estimate_naive(units)
    }
}

fn cost_units(strategy: &str, design: Design, count: usize) -> f64 {
    let n = count as f64;
    match (strategy, design) {
        ("randomized_naive", Design::Randomized) => 2.0 + n * 0.9,
        ("observational_naive", Design::Observational) => 1.0 + n * 0.2,
        ("observational_adjusted", Design::AdjustedObservational) => 1.0 + n * 0.7,
        ("stratified_randomized", Design::StratifiedRandomized) => 2.0 + n * 1.25,
        ("cue_guided", Design::StratifiedRandomized) => 2.0 + n * 1.25,
        ("cue_guided", Design::Observational) => 1.0 + n * 0.2,
        ("always_abstain", Design::None) => 0.1,
        _ => 0.0,
    }
}

fn direction(estimate: Option<f64>, lower: Option<f64>, upper: Option<f64>, design: Design) -> &'static str {
    let (Some(_), Some(low), Some(high)) = (estimate, lower, upper) else {
        return if design == Design::None { "abstain" } else { "unidentified" };
    };
    if low > 0.0 {
        "positive"
    } else if high < 0.0 {
        "negative"
    } else {
        "none"
    }
}

fn run_cell(world: World, strategy: &'static str, seed: u64, units_count: usize, replication: usize) -> CellResult {
    let mut units = make_units(seed, world, units_count);
    let truth = units
        .iter()
        .map(|unit| true_effect(world, unit.confounder))
        .sum::<f64>()
        / units.len() as f64;
    let high_signal = confounding_signal(seed, world);
    let (design, adjusted) = choose_design(strategy, high_signal);
    assign_treatment(&mut units, seed, design);

    let estimate_pair = estimated_effect(&units, adjusted);
    let (estimate, standard_error, low, high) = match estimate_pair {
        Some((estimate, se)) => (Some(estimate), Some(se), Some(estimate - Z95 * se), Some(estimate + Z95 * se)),
        None => (None, None, None, None),
    };
    let covers_truth = match (low, high) {
        (Some(low), Some(high)) => Some(low <= truth && truth <= high),
        _ => None,
    };
    let claim = direction(estimate, low, high, design);
    let correct_sign = estimate.and_then(|value| {
        if truth.abs() < 1e-12 {
            None
        } else {
            Some((value > 0.0 && truth > 0.0) || (value < 0.0 && truth < 0.0))
        }
    });
    let causal_design_identifies = estimate.is_some()
        && match design {
            Design::Randomized | Design::StratifiedRandomized | Design::AdjustedObservational => true,
            Design::Observational => !world.confounding_risk && world.gamma == 0.0,
            Design::None => false,
        };
    let cost = cost_units(strategy, design, units_count);
    let run_id = format!("{}/{}/rep-{replication:06}", world.id, strategy);
    let row = json!({
        "run_id": run_id,
        "regime": world.id,
        "strategy": strategy,
        "seed": seed,
        "confounding_signal_high": high_signal,
        "truth_ate": truth,
        "estimate": estimate,
        "standard_error": standard_error,
        "ci95_low": low,
        "ci95_high": high,
        "ci95_covers_truth": covers_truth,
        "directional_claim": claim,
        "correct_sign": correct_sign,
        "cost_units": cost,
        "design_used": design.name(),
        "estimate_defined": estimate.is_some(),
        "causally_identified": causal_design_identifies
    });

    CellResult {
        json: row,
        regime: world.id,
        strategy,
        truth,
        estimate,
        standard_error,
        interval_covers_truth: covers_truth,
        directional_claim: claim,
        correct_sign,
        cost,
        causally_identified: causal_design_identifies,
    }
}

fn mean(values: &[f64]) -> Option<f64> {
    if values.is_empty() { None } else { Some(values.iter().sum::<f64>() / values.len() as f64) }
}

fn summarize(world: World, strategy: &str, cells: &[CellResult]) -> Value {
    let estimates: Vec<&CellResult> = cells.iter().filter(|cell| cell.estimate.is_some()).collect();
    let errors: Vec<f64> = estimates.iter().map(|cell| cell.estimate.unwrap() - cell.truth).collect();
    let absolute_errors: Vec<f64> = errors.iter().map(|error| error.abs()).collect();
    let signs: Vec<f64> = estimates.iter()
        .filter_map(|cell| cell.correct_sign.map(|correct| if correct { 1.0 } else { 0.0 }))
        .collect();
    let coverage: Vec<f64> = estimates.iter()
        .filter_map(|cell| cell.interval_covers_truth.map(|covers| if covers { 1.0 } else { 0.0 }))
        .collect();
    let ses: Vec<f64> = estimates.iter().filter_map(|cell| cell.standard_error).collect();
    let precision: Vec<f64> = estimates.iter()
        .filter_map(|cell| cell.standard_error.map(|se| {
            if se > 0.0 && cell.cost > 0.0 { 1.0 / (se * se * cell.cost) } else { 0.0 }
        }))
        .collect();
    let costs: Vec<f64> = cells.iter().map(|cell| cell.cost).collect();
    let identified = cells.iter().filter(|cell| cell.causally_identified).count() as f64
        / cells.len() as f64;
    let abstention = cells.iter().filter(|cell| cell.directional_claim == "abstain").count() as f64
        / cells.len() as f64;
    let false_directional = if world.id == "null_effect" {
        Some(cells.iter().filter(|cell| matches!(cell.directional_claim, "positive" | "negative")).count() as f64
            / cells.len() as f64)
    } else {
        None
    };

    json!({
        "regime": world.id,
        "strategy": strategy,
        "runs": cells.len(),
        "estimable_fraction": estimates.len() as f64 / cells.len() as f64,
        "causal_identification_rate": identified,
        "mean_error": mean(&errors),
        "mean_absolute_error": mean(&absolute_errors),
        "sign_recovery_rate": mean(&signs),
        "ci95_coverage": mean(&coverage),
        "false_directional_claim_rate_when_null": false_directional,
        "mean_cost_units": mean(&costs).unwrap_or(0.0),
        "mean_standard_error": mean(&ses),
        "nominal_precision_per_cost": mean(&precision),
        "abstention_rate": abstention
    })
}

fn build_report(replications: usize, units: usize, seed: u64) -> Value {
    let mut cells = Vec::with_capacity(replications * WORLDS.len() * STRATEGIES.len());
    for (world_index, world) in WORLDS.iter().enumerate() {
        for replication in 0..replications {
            let run_seed = seed
                .wrapping_add((replication as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
                .wrapping_add((world_index as u64).wrapping_mul(0xD1B5_4A32_D192_ED03));
            for strategy in STRATEGIES {
                cells.push(run_cell(*world, strategy, run_seed, units, replication));
            }
        }
    }
    let scheduled = replications * WORLDS.len() * STRATEGIES.len();
    let completed = cells.len();
    let mut grouped: BTreeMap<(&str, &str), Vec<CellResult>> = BTreeMap::new();
    for cell in &cells {
        grouped.entry((cell.regime, cell.strategy)).or_default().push(cell.clone());
    }
    let summaries: Vec<Value> = WORLDS.iter()
        .flat_map(|world| STRATEGIES.iter().map(move |strategy| (*world, *strategy)))
        .map(|(world, strategy)| summarize(world, strategy, grouped.get(&(world.id, strategy)).map(Vec::as_slice).unwrap_or(&[])))
        .collect();
    json!({
        "schema_version": "civ-impact-causal-arena-report-v0.1",
        "source_sha256": sha256(SOURCE.as_bytes()),
        "seed": seed,
        "replications": replications,
        "units_per_replication": units,
        "regime_count": WORLDS.len(),
        "strategy_count": STRATEGIES.len(),
        "scheduled_cells": scheduled,
        "completed_cells": completed,
        "missing_cells": scheduled.saturating_sub(completed),
        "complete": completed == scheduled,
        "claim_ceiling": "synthetic_diagnostic_only_not_field_evidence",
        "regimes": WORLDS.iter().map(|world| world.id).collect::<Vec<_>>(),
        "strategies": STRATEGIES,
        "results": cells.into_iter().map(|cell| cell.json).collect::<Vec<_>>(),
        "summary": summaries
    })
}

fn usage() -> &'static str {
    "Usage:\n  civ-impact-causal-arena run [--replications N] [--units N] [--seed N]\n  civ-impact-causal-arena --help\n\nEmits a deterministic synthetic report to stdout. Public results are diagnostic, not field evidence."
}

fn parse_args() -> Result<(usize, usize, u64), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let mut replications = 64usize;
    let mut units = 256usize;
    let mut seed = 20_261_016u64;
    let mut index = if args.first().map(String::as_str) == Some("run") { 1 } else { 0 };
    while index < args.len() {
        let key = args[index].as_str();
        if key == "--help" || key == "-h" {
            println!("{}", usage());
            process::exit(0);
        }
        if index + 1 >= args.len() {
            return Err(format!("missing value for {key}\n{}", usage()));
        }
        let value = &args[index + 1];
        match key {
            "--replications" => replications = value.parse().map_err(|_| "invalid --replications integer")?,
            "--units" => units = value.parse().map_err(|_| "invalid --units integer")?,
            "--seed" => seed = value.parse().map_err(|_| "invalid --seed integer")?,
            _ => return Err(format!("unknown option {key}\n{}", usage())),
        }
        index += 2;
    }
    if replications == 0 {
        return Err("--replications must be at least 1".into());
    }
    if units < 16 {
        return Err("--units must be at least 16 for this contract benchmark".into());
    }
    Ok((replications, units, seed))
}

fn validate_report(report: &Value) -> Result<(), String> {
    let schema: Value = serde_json::from_str(REPORT_SCHEMA)
        .map_err(|error| format!("invalid bundled report schema: {error}"))?;
    let validator = jsonschema::validator_for(&schema)
        .map_err(|error| format!("cannot compile report schema: {error}"))?;
    let errors: Vec<String> = validator.iter_errors(report).take(10).map(|error| error.to_string()).collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("\n"))
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || args.first().map(String::as_str) == Some("--help") || args.first().map(String::as_str) == Some("-h") {
        println!("{}", usage());
        return Ok(());
    }
    if args.first().map(String::as_str) != Some("run") {
        return Err(usage().to_owned());
    }
    let (replications, units, seed) = parse_args()?;
    let report = build_report(replications, units, seed);
    validate_report(&report)?;
    println!("{}", serde_json::to_string_pretty(&report).map_err(|error| error.to_string())?);
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn report_schema_compiles() {
        let schema: Value = serde_json::from_str(REPORT_SCHEMA).unwrap();
        assert!(jsonschema::validator_for(&schema).is_ok());
    }

    #[test]
    fn report_is_schema_valid_and_complete() {
        let report = build_report(3, 64, 7);
        validate_report(&report).unwrap();
        assert_eq!(report["scheduled_cells"], 90);
        assert_eq!(report["completed_cells"], 90);
        assert_eq!(report["missing_cells"], 0);
        assert_eq!(report["complete"], true);
        assert_eq!(report["results"].as_array().unwrap().len(), 90);
        assert_eq!(report["summary"].as_array().unwrap().len(), 30);
    }

    #[test]
    fn identical_seed_and_config_replay_exactly() {
        assert_eq!(build_report(4, 128, 991), build_report(4, 128, 991));
    }

    #[test]
    fn paired_strategies_share_the_same_cohort_truth() {
        let naive = run_cell(WORLDS[2], "randomized_naive", 123, 512, 0);
        let adjusted = run_cell(WORLDS[2], "observational_adjusted", 123, 512, 0);
        assert!((naive.truth - adjusted.truth).abs() < 1e-12);
    }

    #[test]
    fn sign_reversal_breaks_naive_observational_estimate() {
        let cell = run_cell(WORLDS[2], "observational_naive", 42, 4096, 0);
        assert!(cell.truth < 0.0);
        assert!(cell.estimate.unwrap() > 0.0, "estimate was {:?}", cell.estimate);
        assert!(!cell.causally_identified);
    }

    #[test]
    fn randomized_assignment_recovers_sign_reversal_effect() {
        let cell = run_cell(WORLDS[2], "randomized_naive", 42, 4096, 0);
        assert!(cell.truth < 0.0);
        assert!(cell.estimate.unwrap() < 0.0, "estimate was {:?}", cell.estimate);
        assert!(cell.causally_identified);
        assert!((cell.estimate.unwrap() - cell.truth).abs() < 0.20);
    }

    #[test]
    fn measured_confounder_adjustment_reduces_hidden_confounding_bias() {
        let naive = run_cell(WORLDS[1], "observational_naive", 77, 4096, 0);
        let adjusted = run_cell(WORLDS[1], "observational_adjusted", 77, 4096, 0);
        let naive_error = (naive.estimate.unwrap() - naive.truth).abs();
        let adjusted_error = (adjusted.estimate.unwrap() - adjusted.truth).abs();
        assert!(adjusted_error < naive_error, "adjusted={adjusted_error}, naive={naive_error}");
        assert!(adjusted.causally_identified);
        assert!(!naive.causally_identified);
    }

    #[test]
    fn observational_null_control_exposes_spurious_directional_claim() {
        let cell = run_cell(WORLDS[3], "observational_naive", 13, 4096, 0);
        assert!(cell.truth.abs() < 1e-12);
        assert!(matches!(cell.directional_claim, "positive" | "negative"));
        assert!(!cell.causally_identified);
    }

    #[test]
    fn null_world_truth_is_exactly_zero_for_every_cohort() {
        for seed in 0..20 {
            let cell = run_cell(WORLDS[3], "randomized_naive", seed, 64, 0);
            assert!(cell.truth.abs() < 1e-12);
        }
    }

    #[test]
    fn abstention_is_not_counted_as_estimable_or_causally_identified() {
        let cell = run_cell(WORLDS[2], "always_abstain", 9, 128, 0);
        assert!(cell.estimate.is_none());
        assert_eq!(cell.directional_claim, "abstain");
        assert!(!cell.causally_identified);
        assert!(cell.cost > 0.0);
    }

    #[test]
    fn cue_guided_policy_uses_only_the_noisy_signal_to_choose_design() {
        for seed in 0..40 {
            let cell = run_cell(WORLDS[0], "cue_guided", seed, 128, 0);
            let expected_high = confounding_signal(seed, WORLDS[0]);
            if expected_high {
                assert_eq!(cell.json["design_used"], "stratified_randomized");
            } else {
                assert_eq!(cell.json["design_used"], "observational");
            }
        }
    }

    #[test]
    fn no_directional_claim_when_interval_includes_zero() {
        assert_eq!(direction(Some(0.1), Some(-0.2), Some(0.4), Design::Randomized), "none");
    }

    #[test]
    fn strategy_matrix_contains_explicit_baselines() {
        assert!(STRATEGIES.contains(&"observational_naive"));
        assert!(STRATEGIES.contains(&"randomized_naive"));
        assert!(STRATEGIES.contains(&"always_abstain"));
    }

    #[test]
    fn report_binds_a_source_digest() {
        let report = build_report(1, 32, 5);
        let digest = report["source_sha256"].as_str().unwrap();
        assert_eq!(digest.len(), 64);
        assert!(digest.bytes().all(|byte| byte.is_ascii_hexdigit()));
    }

    #[test]
    fn costed_adjustment_is_not_free() {
        let naive = run_cell(WORLDS[1], "observational_naive", 77, 512, 0);
        let adjusted = run_cell(WORLDS[1], "observational_adjusted", 77, 512, 0);
        assert!(adjusted.cost > naive.cost);
    }

    #[test]
    fn heterogeneity_regime_has_nonconstant_unit_effects() {
        let units = make_units(17, WORLDS[4], 256);
        let effects: std::collections::BTreeSet<i64> = units
            .iter()
            .map(|unit| (true_effect(WORLDS[4], unit.confounder) * 10.0) as i64)
            .collect();
        assert_eq!(effects.len(), 2);
    }

    #[test]
    fn all_planned_regime_strategy_combinations_are_present() {
        let report = build_report(2, 32, 3);
        for world in WORLDS {
            for strategy in STRATEGIES {
                assert_eq!(
                    report["summary"].as_array().unwrap().iter()
                        .filter(|row| row["regime"] == world.id && row["strategy"] == strategy).count(),
                    1
                );
            }
        }
    }
}
