//! Deterministic trace-replay model for Pulse routing-policy comparisons.
//!
//! This deliberately models route eligibility, synthetic link outcomes shared
//! within declared failure domains, latency, attempt cost, and energy estimates.
//! It provides no evidence
//! of cryptography, authenticated delivery, RF behavior, or real hardware.

use serde_json::{Value, json};
use std::collections::BTreeSet;

const MAX_SCENARIOS: usize = 100;
const MAX_PATHS_PER_SCENARIO: usize = 256;
const MAX_MESSAGES: u64 = 1_000_000;

#[derive(Clone, Debug)]
struct Path {
    id: String,
    failure_domain_id: String,
    priority: u64,
    available: bool,
    permitted: bool,
    latency_ms: u64,
    success_basis_points: u64,
    payload_budget_bytes: u64,
    cost_units: u64,
    energy_millijoules: u64,
    unavailable_ranges: Vec<(u64, u64)>,
}

#[derive(Clone, Debug)]
struct Scenario {
    id: String,
    purpose: String,
    deadline_ms: u64,
    envelope_bytes: u64,
    paths: Vec<Path>,
}

#[derive(Default)]
struct Aggregate {
    messages: u64,
    delivered: u64,
    latencies_ms: Vec<u64>,
    attempts: u128,
    cost_units: u128,
    energy_millijoules: u128,
}

impl Aggregate {
    fn record(&mut self, outcome: Outcome) {
        self.messages += 1;
        self.attempts += u128::from(outcome.attempts);
        self.cost_units += u128::from(outcome.cost_units);
        self.energy_millijoules += u128::from(outcome.energy_millijoules);
        if let Some(latency) = outcome.delivery_latency_ms {
            self.delivered += 1;
            self.latencies_ms.push(latency);
        }
    }

    fn to_json(&self) -> Value {
        json!({
            "logical_messages": self.messages,
            "modeled_path_successes_before_deadline": self.delivered,
            "modeled_path_success_rate_basis_points": rate_basis_points(self.delivered, self.messages),
            "latency_percentile_population": "modeled successes only; misses count toward deadline success rate but are not in latency percentiles",
            "p50_modeled_latency_ms": percentile(&self.latencies_ms, 50),
            "p95_modeled_latency_ms": percentile(&self.latencies_ms, 95),
            "attempts_per_1000_messages": scaled_per_message(self.attempts, self.messages, 1000),
            "illustrative_cost_units_per_message_x1000": scaled_per_message(
                self.cost_units,
                self.messages,
                1000
            ),
            "illustrative_energy_input_millijoules_per_message_x1000": scaled_per_message(
                self.energy_millijoules,
                self.messages,
                1000
            )
        })
    }
}

#[derive(Default, Clone, Copy)]
struct Outcome {
    delivery_latency_ms: Option<u64>,
    attempts: u64,
    cost_units: u64,
    energy_millijoules: u64,
}

#[derive(Clone, Copy, Debug)]
enum Strategy {
    PreferredSinglePath,
    SequentialFailover,
    ParallelRedundancy,
}

impl Strategy {
    const ALL: [Strategy; 3] = [
        Strategy::PreferredSinglePath,
        Strategy::SequentialFailover,
        Strategy::ParallelRedundancy,
    ];

    fn name(self) -> &'static str {
        match self {
            Strategy::PreferredSinglePath => "preferred_single_path",
            Strategy::SequentialFailover => "sequential_failover",
            Strategy::ParallelRedundancy => "parallel_redundancy",
        }
    }
}

/// Run a reproducible synthetic trace comparison.
///
/// Input must explicitly declare simulation_only=true and hardware_evidence=false.
/// This is a contract-level route model, not a generic network simulator.
pub fn simulate_corpus(
    corpus: &Value,
    message_count_override: Option<u64>,
    seed_override: Option<u64>,
) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();
    if corpus.get("schema_version").and_then(Value::as_str) != Some("1.0.0") {
        errors.push("trace corpus schema_version must be '1.0.0'".to_owned());
    }
    if corpus.get("simulation_only").and_then(Value::as_bool) != Some(true) {
        errors.push("trace corpus must explicitly set simulation_only=true".to_owned());
    }
    if corpus.get("hardware_evidence").and_then(Value::as_bool) != Some(false) {
        errors.push("trace corpus must explicitly set hardware_evidence=false".to_owned());
    }
    let corpus_id = corpus
        .get("corpus_id")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned();
    if corpus_id.is_empty() {
        errors.push("trace corpus corpus_id must be a non-empty string".to_owned());
    }
    let default_messages = corpus
        .get("default_message_count")
        .and_then(Value::as_u64)
        .unwrap_or(0);
    let messages = message_count_override.unwrap_or(default_messages);
    if messages == 0 || messages > MAX_MESSAGES {
        errors.push(format!("message count must be between 1 and {MAX_MESSAGES}"));
    }
    let default_seed = corpus.get("seed").and_then(Value::as_u64).unwrap_or(0);
    let seed = seed_override.unwrap_or(default_seed);

    let Some(scenarios_value) = corpus.get("scenarios").and_then(Value::as_array) else {
        errors.push("trace corpus scenarios must be an array".to_owned());
        return Err(errors);
    };
    if scenarios_value.is_empty() || scenarios_value.len() > MAX_SCENARIOS {
        errors.push(format!(
            "trace corpus must contain between 1 and {MAX_SCENARIOS} scenarios"
        ));
    }
    let scenarios = scenarios_value
        .iter()
        .enumerate()
        .filter_map(|(index, value)| match parse_scenario(value, index) {
            Ok(scenario) => Some(scenario),
            Err(mut scenario_errors) => {
                errors.append(&mut scenario_errors);
                None
            }
        })
        .collect::<Vec<_>>();

    let mut scenario_ids = BTreeSet::new();
    for scenario in &scenarios {
        if !scenario_ids.insert(scenario.id.clone()) {
            errors.push(format!("duplicate scenario ID '{}'", scenario.id));
        }
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    let mut scenario_results = Vec::new();
    let mut aggregate = [
        Aggregate::default(),
        Aggregate::default(),
        Aggregate::default(),
    ];

    for scenario in &scenarios {
        let mut per_strategy = Vec::new();
        for (strategy_index, strategy) in Strategy::ALL.into_iter().enumerate() {
            let mut metrics = Aggregate::default();
            for message_index in 0..messages {
                let outcome = simulate_message(scenario, strategy, seed, message_index);
                metrics.record(outcome);
            }
            aggregate[strategy_index].messages += metrics.messages;
            aggregate[strategy_index].delivered += metrics.delivered;
            aggregate[strategy_index]
                .latencies_ms
                .extend(metrics.latencies_ms.iter().copied());
            aggregate[strategy_index].attempts += metrics.attempts;
            aggregate[strategy_index].cost_units += metrics.cost_units;
            aggregate[strategy_index].energy_millijoules += metrics.energy_millijoules;

            per_strategy.push(json!({
                "strategy": strategy.name(),
                "metrics": metrics.to_json()
            }));
        }
        scenario_results.push(json!({
            "scenario_id": scenario.id,
            "purpose": scenario.purpose,
            "deadline_ms": scenario.deadline_ms,
            "envelope_bytes": scenario.envelope_bytes,
            "strategy_results": per_strategy
        }));
    }

    let overall = Strategy::ALL
        .into_iter()
        .enumerate()
        .map(|(index, strategy)| {
            json!({
                "strategy": strategy.name(),
                "metrics": aggregate[index].to_json()
            })
        })
        .collect::<Vec<_>>();

    Ok(json!({
        "report_version": "1.0.0",
        "report_type": "synthetic_transport_trace_simulation",
        "corpus_id": corpus_id,
        "seed": seed,
        "messages_per_scenario": messages,
        "scenario_count": scenarios.len(),
        "per_message_cost_and_energy_scale_factor": 1000,
        "simulation_only": true,
        "authenticated_delivery_established": false,
        "hardware_qualification": false,
        "cryptographic_security_evaluated": false,
        "limitations": [
            "Synthetic outcomes share draws within failure domains and are independent across domain IDs; this is not measured network reliability.",
            "No RF propagation, interference, radio duty-cycle, congestion, or real hardware is simulated.",
            "Success means only that the route model predicts a path success before the deadline; it is not an authenticated recipient receipt.",
            "No fragmentation is modelled; envelopes larger than a path's payload budget are ineligible on that path.",
            "Cost and energy values are illustrative trace inputs, not production price or power measurements."
        ],
        "aggregate_results": overall,
        "scenario_results": scenario_results
    }))
}

fn parse_scenario(value: &Value, index: usize) -> Result<Scenario, Vec<String>> {
    let prefix = format!("scenarios[{index}]");
    let mut errors = Vec::new();
    let Some(object) = value.as_object() else {
        return Err(vec![format!("{prefix}: must be an object")]);
    };

    let id = string_field(object.get("id"), &format!("{prefix}.id"), &mut errors);
    let purpose = string_field(
        object.get("purpose"),
        &format!("{prefix}.purpose"),
        &mut errors,
    );
    let deadline_ms = positive_int_field(
        object.get("deadline_ms"),
        &format!("{prefix}.deadline_ms"),
        &mut errors,
    );
    let envelope_bytes = positive_int_field(
        object.get("envelope_bytes"),
        &format!("{prefix}.envelope_bytes"),
        &mut errors,
    );

    let mut paths = Vec::new();
    match object.get("paths").and_then(Value::as_array) {
        Some(values) if !values.is_empty() && values.len() <= MAX_PATHS_PER_SCENARIO => {
            for (path_index, value) in values.iter().enumerate() {
                match parse_path(value, &prefix, path_index) {
                    Ok(path) => paths.push(path),
                    Err(mut path_errors) => errors.append(&mut path_errors),
                }
            }
        }
        Some(_) => errors.push(format!(
            "{prefix}.paths: must have between 1 and {MAX_PATHS_PER_SCENARIO} paths"
        )),
        None => errors.push(format!("{prefix}.paths: must be an array")),
    }

    let mut path_ids = BTreeSet::new();
    for path in &paths {
        if !path_ids.insert(path.id.clone()) {
            errors.push(format!("{prefix}: duplicate path ID '{}'", path.id));
        }
    }

    if errors.is_empty() {
        Ok(Scenario {
            id: id.expect("validated id"),
            purpose: purpose.expect("validated purpose"),
            deadline_ms: deadline_ms.expect("validated deadline"),
            envelope_bytes: envelope_bytes.expect("validated envelope"),
            paths,
        })
    } else {
        Err(errors)
    }
}

fn parse_path(value: &Value, scenario_prefix: &str, index: usize) -> Result<Path, Vec<String>> {
    let prefix = format!("{scenario_prefix}.paths[{index}]");
    let mut errors = Vec::new();
    let Some(object) = value.as_object() else {
        return Err(vec![format!("{prefix}: must be an object")]);
    };

    let id = string_field(object.get("id"), &format!("{prefix}.id"), &mut errors);
    let failure_domain_id = string_field(
        object.get("failure_domain_id"),
        &format!("{prefix}.failure_domain_id"),
        &mut errors,
    );
    let priority = integer_field(
        object.get("priority"),
        &format!("{prefix}.priority"),
        &mut errors,
    );
    let available = bool_field(
        object.get("available"),
        &format!("{prefix}.available"),
        &mut errors,
    );
    let permitted = bool_field(
        object.get("permitted"),
        &format!("{prefix}.permitted"),
        &mut errors,
    );
    let latency_ms = positive_int_field(
        object.get("latency_ms"),
        &format!("{prefix}.latency_ms"),
        &mut errors,
    );
    let success_basis_points = integer_field(
        object.get("success_basis_points"),
        &format!("{prefix}.success_basis_points"),
        &mut errors,
    );
    if success_basis_points.is_some_and(|value| value > 10_000) {
        errors.push(format!("{prefix}.success_basis_points: must be <= 10000"));
    }
    let payload_budget_bytes = positive_int_field(
        object.get("payload_budget_bytes"),
        &format!("{prefix}.payload_budget_bytes"),
        &mut errors,
    );
    let cost_units = integer_field(
        object.get("cost_units"),
        &format!("{prefix}.cost_units"),
        &mut errors,
    );
    let energy_millijoules = integer_field(
        object.get("energy_millijoules"),
        &format!("{prefix}.energy_millijoules"),
        &mut errors,
    );
    let unavailable_ranges = parse_unavailable_ranges(
        object.get("unavailable_ranges"),
        &format!("{prefix}.unavailable_ranges"),
        &mut errors,
    );

    if errors.is_empty() {
        Ok(Path {
            id: id.expect("validated path ID"),
            failure_domain_id: failure_domain_id.expect("validated failure domain"),
            priority: priority.expect("validated priority"),
            available: available.expect("validated availability"),
            permitted: permitted.expect("validated permission"),
            latency_ms: latency_ms.expect("validated latency"),
            success_basis_points: success_basis_points.expect("validated success rate"),
            payload_budget_bytes: payload_budget_bytes.expect("validated payload budget"),
            cost_units: cost_units.expect("validated cost"),
            energy_millijoules: energy_millijoules.expect("validated energy"),
            unavailable_ranges,
        })
    } else {
        Err(errors)
    }
}

fn parse_unavailable_ranges(
    value: Option<&Value>,
    label: &str,
    errors: &mut Vec<String>,
) -> Vec<(u64, u64)> {
    let Some(ranges) = value.and_then(Value::as_array) else {
        errors.push(format!("{label}: must be an array of [start, end] pairs"));
        return Vec::new();
    };
    let mut parsed = Vec::with_capacity(ranges.len());
    for (index, range) in ranges.iter().enumerate() {
        let Some(pair) = range.as_array() else {
            errors.push(format!("{label}[{index}]: must be an integer pair"));
            continue;
        };
        if pair.len() != 2 {
            errors.push(format!("{label}[{index}]: must have exactly two values"));
            continue;
        }
        let start = pair.first().and_then(Value::as_u64);
        let end = pair.get(1).and_then(Value::as_u64);
        match (start, end) {
            (Some(start), Some(end)) if start < end => parsed.push((start, end)),
            _ => errors.push(format!(
                "{label}[{index}]: range must contain non-negative integers with start < end"
            )),
        }
    }
    parsed
}

fn string_field(
    value: Option<&Value>,
    label: &str,
    errors: &mut Vec<String>,
) -> Option<String> {
    match value.and_then(Value::as_str) {
        Some(text) if !text.trim().is_empty() => Some(text.to_owned()),
        _ => {
            errors.push(format!("{label}: must be a non-empty string"));
            None
        }
    }
}

fn integer_field(value: Option<&Value>, label: &str, errors: &mut Vec<String>) -> Option<u64> {
    match value.and_then(Value::as_u64) {
        Some(value) => Some(value),
        None => {
            errors.push(format!("{label}: must be a non-negative integer"));
            None
        }
    }
}

fn positive_int_field(
    value: Option<&Value>,
    label: &str,
    errors: &mut Vec<String>,
) -> Option<u64> {
    match integer_field(value, label, errors) {
        Some(value) if value > 0 => Some(value),
        Some(_) => {
            errors.push(format!("{label}: must be > 0"));
            None
        }
        None => None,
    }
}

fn bool_field(value: Option<&Value>, label: &str, errors: &mut Vec<String>) -> Option<bool> {
    match value.and_then(Value::as_bool) {
        Some(value) => Some(value),
        None => {
            errors.push(format!("{label}: must be a boolean"));
            None
        }
    }
}

fn simulate_message(
    scenario: &Scenario,
    strategy: Strategy,
    seed: u64,
    message_index: u64,
) -> Outcome {
    match strategy {
        Strategy::PreferredSinglePath => single_path(scenario, seed, message_index),
        Strategy::SequentialFailover => sequential_failover(scenario, seed, message_index),
        Strategy::ParallelRedundancy => parallel_redundancy(scenario, seed, message_index),
    }
}

fn single_path(scenario: &Scenario, seed: u64, message_index: u64) -> Outcome {
    let Some(path) = scenario.paths.iter().min_by_key(|path| (path.priority, &path.id)) else {
        return Outcome::default();
    };
    if !eligible(path, scenario, message_index) || path.latency_ms > scenario.deadline_ms {
        return Outcome::default();
    }
    let mut outcome = attempt(path);
    if path_succeeds(scenario, path, seed, message_index)
        && path.latency_ms <= scenario.deadline_ms
    {
        outcome.delivery_latency_ms = Some(path.latency_ms);
    }
    outcome
}

fn sequential_failover(scenario: &Scenario, seed: u64, message_index: u64) -> Outcome {
    let mut paths = scenario.paths.iter().collect::<Vec<_>>();
    paths.sort_by_key(|path| (path.priority, &path.id));
    let mut outcome = Outcome::default();
    let mut elapsed = 0_u64;

    for path in paths {
        if !eligible(path, scenario, message_index) {
            continue;
        }
        let Some(completion) = elapsed.checked_add(path.latency_ms) else {
            continue;
        };
        if completion > scenario.deadline_ms {
            continue;
        }
        add_attempt(&mut outcome, path);
        if path_succeeds(scenario, path, seed, message_index) {
            outcome.delivery_latency_ms = Some(completion);
            break;
        }
        elapsed = completion;
    }
    outcome
}

fn parallel_redundancy(scenario: &Scenario, seed: u64, message_index: u64) -> Outcome {
    let mut outcome = Outcome::default();
    let mut best_latency = None;

    for path in &scenario.paths {
        if !eligible(path, scenario, message_index) || path.latency_ms > scenario.deadline_ms {
            continue;
        }
        add_attempt(&mut outcome, path);
        if path_succeeds(scenario, path, seed, message_index) {
            best_latency = Some(
                best_latency.map_or(path.latency_ms, |best: u64| best.min(path.latency_ms)),
            );
        }
    }
    outcome.delivery_latency_ms = best_latency;
    outcome
}

fn eligible(path: &Path, scenario: &Scenario, message_index: u64) -> bool {
    path.available
        && path.permitted
        && scenario.envelope_bytes <= path.payload_budget_bytes
        && !path
            .unavailable_ranges
            .iter()
            .any(|(start, end)| message_index >= *start && message_index < *end)
}

fn path_succeeds(scenario: &Scenario, path: &Path, seed: u64, message_index: u64) -> bool {
    let mut state = seed ^ 0xcbf29ce484222325_u64;
    for byte in scenario
        .id
        .bytes()
        .chain([0xff])
        .chain(path.failure_domain_id.bytes())
        .chain([0xfe])
        .chain(message_index.to_le_bytes())
    {
        state ^= u64::from(byte);
        state = state.wrapping_mul(0x100000001b3);
    }
    state = splitmix64(state);
    (state % 10_000) < path.success_basis_points
}

fn splitmix64(mut value: u64) -> u64 {
    value = value.wrapping_add(0x9e3779b97f4a7c15);
    value = (value ^ (value >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27)).wrapping_mul(0x94d049bb133111eb);
    value ^ (value >> 31)
}

fn attempt(path: &Path) -> Outcome {
    let mut outcome = Outcome::default();
    add_attempt(&mut outcome, path);
    outcome
}

fn add_attempt(outcome: &mut Outcome, path: &Path) {
    outcome.attempts += 1;
    outcome.cost_units = outcome.cost_units.saturating_add(path.cost_units);
    outcome.energy_millijoules = outcome
        .energy_millijoules
        .saturating_add(path.energy_millijoules);
}

fn rate_basis_points(delivered: u64, total: u64) -> u64 {
    if total == 0 {
        return 0;
    }
    ((u128::from(delivered) * 10_000) / u128::from(total)) as u64
}

fn scaled_per_message(total: u128, messages: u64, scale: u128) -> u64 {
    if messages == 0 {
        return 0;
    }
    ((total.saturating_mul(scale)) / u128::from(messages)).min(u128::from(u64::MAX)) as u64
}

fn percentile(values: &[u64], percentile: usize) -> Option<u64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (sorted.len() * percentile).div_ceil(100).max(1);
    sorted.get(rank - 1).copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn corpus() -> Value {
        serde_json::from_str(include_str!("../../../benchmarks/traces.v1.json"))
            .expect("trace corpus JSON must parse")
    }

    #[test]
    fn checked_in_trace_corpus_is_simulation_only_and_valid() {
        let report = simulate_corpus(&corpus(), Some(10_000), None)
            .expect("checked-in traces must simulate");
        assert_eq!(report["simulation_only"], true);
        assert_eq!(report["authenticated_delivery_established"], false);
        assert_eq!(report["hardware_qualification"], false);
        assert_eq!(report["scenario_count"], 11);
        assert_eq!(report["messages_per_scenario"], 10_000);
    }

    #[test]
    fn repeated_runs_are_deterministic() {
        let corpus = corpus();
        let first = simulate_corpus(&corpus, Some(100), Some(1234)).unwrap();
        let second = simulate_corpus(&corpus, Some(100), Some(1234)).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn sequential_failover_improves_outage_trace_without_claiming_authentication() {
        let report = simulate_corpus(&corpus(), Some(10_000), Some(20261016)).unwrap();
        let outage = report["scenario_results"]
            .as_array()
            .unwrap()
            .iter()
            .find(|row| row["scenario_id"] == "scheduled-ip-outage")
            .unwrap();
        let results = outage["strategy_results"].as_array().unwrap();
        let single = results
            .iter()
            .find(|row| row["strategy"] == "preferred_single_path")
            .unwrap()["metrics"]["modeled_path_successes_before_deadline"]
            .as_u64()
            .unwrap();
        let failover = results
            .iter()
            .find(|row| row["strategy"] == "sequential_failover")
            .unwrap()["metrics"]["modeled_path_successes_before_deadline"]
            .as_u64()
            .unwrap();
        assert!(failover > single, "single={single}, failover={failover}");
        assert_eq!(report["authenticated_delivery_established"], false);
    }

    #[test]
    fn common_cause_outage_disables_all_paths_in_that_failure_domain() {
        let corpus = corpus();
        let value = corpus["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .find(|scenario| scenario["id"] == "shared-carrier-failure-domain")
            .expect("shared failure-domain scenario exists");
        let scenario = parse_scenario(value, 10).expect("scenario parses");
        let path_a = scenario
            .paths
            .iter()
            .find(|path| path.id == "cellular-radio-a")
            .expect("first carrier path exists");
        let path_b = scenario
            .paths
            .iter()
            .find(|path| path.id == "cellular-radio-b")
            .expect("second carrier path exists");
        let independent = scenario
            .paths
            .iter()
            .find(|path| path.id == "independent-community-relay")
            .expect("independent relay exists");

        for message_index in [0, 1_500, 2_999] {
            assert!(!eligible(&scenario, path_a, message_index));
            assert!(!eligible(&scenario, path_b, message_index));
            assert!(eligible(&scenario, independent, message_index));
        }
        assert!(eligible(&scenario, path_a, 3_000));
        assert!(eligible(&scenario, path_b, 3_000));
    }

    #[test]
    fn paths_in_same_failure_domain_share_the_synthetic_draw() {
        let corpus = corpus();
        let value = corpus["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .find(|scenario| scenario["id"] == "shared-carrier-failure-domain")
            .expect("shared failure-domain scenario exists");
        let scenario = parse_scenario(value, 10).expect("scenario parses");
        let path_a = scenario
            .paths
            .iter()
            .find(|path| path.id == "cellular-radio-a")
            .expect("first carrier path exists");
        let path_b = scenario
            .paths
            .iter()
            .find(|path| path.id == "cellular-radio-b")
            .expect("second carrier path exists");

        for message_index in 0..1_000 {
            assert_eq!(
                path_succeeds(&scenario, path_a, 20261016, message_index),
                path_succeeds(&scenario, path_b, 20261016, message_index),
                "same-domain paths must share the same synthetic uniform draw"
            );
        }
    }

    #[test]
    fn trace_cannot_claim_hardware_or_authenticated_evidence() {
        let mut corpus = corpus();
        corpus["hardware_evidence"] = json!(true);
        assert!(simulate_corpus(&corpus, Some(10), None).is_err());

        let mut corpus = corpus();
        corpus["simulation_only"] = json!(false);
        assert!(simulate_corpus(&corpus, Some(10), None).is_err());
    }

    #[test]
    fn invalid_path_success_probability_fails_closed() {
        let mut corpus = corpus();
        corpus["scenarios"][0]["paths"][0]["success_basis_points"] = json!(10001);
        let errors = simulate_corpus(&corpus, Some(10), None).unwrap_err();
        assert!(errors.iter().any(|error| error.contains("success_basis_points")));
    }

    #[test]
    fn payload_budget_and_permission_are_hard_constraints() {
        let corpus = corpus();
        let report = simulate_corpus(&corpus, Some(1000), Some(77)).unwrap();
        let privacy = report["scenario_results"]
            .as_array()
            .unwrap()
            .iter()
            .find(|row| row["scenario_id"] == "privacy-policy-excludes-relay")
            .unwrap();
        let rows = privacy["strategy_results"].as_array().unwrap();
        let single = rows
            .iter()
            .find(|row| row["strategy"] == "preferred_single_path")
            .unwrap();
        assert_eq!(single["metrics"]["modeled_path_successes_before_deadline"], 0);
        let failover = rows
            .iter()
            .find(|row| row["strategy"] == "sequential_failover")
            .unwrap();
        assert!(failover["metrics"]["modeled_path_successes_before_deadline"].as_u64().unwrap() > 0);
    }
}
