#![forbid(unsafe_code)]

//! Matched-trace experiment for pre-partition escrow allocation policies.
//! Results are deterministic for a fixed seed and inputs; this is a single-process
//! simulation, not a concurrency, network, Holochain, or distributed benchmark.

use nested_currency_reference::allocation::{
    allocate_static_escrow, AllocationPlan, AllocationPolicy, ShardDemand, PARTS_PER_MILLION,
};
use std::{env, time::Instant};

const INSTRUMENT_ID: &str = "instrument:local";
const DEFAULT_SHARDS: usize = 16;
const DEFAULT_TRAINING_SAMPLES: usize = 32;
const DEFAULT_EVAL_SCENARIOS: usize = 1_000;
const DEFAULT_CAPACITY: u64 = 100_000;
const DEFAULT_RESERVE_PPM: u32 = 50_000;
const DEFAULT_SEED: u64 = 0x4d5943454c495831;
const MAX_SHARDS: usize = 1_000;
const MAX_TRAINING_SAMPLES: usize = 1_000;
const MAX_EVAL_SCENARIOS: usize = 10_000;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn between(&mut self, low: u64, high_inclusive: u64) -> u64 {
        low + self.next() % (high_inclusive - low + 1)
    }
}

#[derive(Clone, Copy)]
enum DemandMode {
    Stationary,
    Shifted,
}

struct PerShardMetrics {
    shard_id: String,
    requested: u128,
    served: u128,
    unserved: u128,
    worst_scenario_served_ppm: u64,
}

struct Metrics {
    scenarios: u128,
    requested: u128,
    served: u128,
    unserved: u128,
    stranded: u128,
    worst_scenario_unserved: u64,
    worst_scenario_served_ppm: u64,
    worst_shard_demand_served_ppm: u64,
    per_shard: Vec<PerShardMetrics>,
    safety_violations: u64,
}

fn required_env(name: &str) -> Result<String, String> {
    let value = env::var(name).map_err(|_| format!("required environment variable {name} is missing"))?;
    if value.trim().is_empty() {
        return Err(format!("required environment variable {name} is empty"));
    }
    Ok(value)
}

fn require_hex(name: &str, lengths: &[usize]) -> Result<String, String> {
    let value = required_env(name)?;
    if !lengths.contains(&value.len())
        || !value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(format!("{name} must be lowercase hexadecimal with length in {lengths:?}"));
    }
    Ok(value)
}

fn parse_u64(value: &str, label: &str) -> Result<u64, String> {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|e| format!("invalid {label}: {e}"))
    } else {
        value.parse::<u64>().map_err(|e| format!("invalid {label}: {e}"))
    }
}

fn json_escape(value: &str) -> String {
    let mut out = String::with_capacity(value.len() + 2);
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if ch.is_control() => out.push_str(&format!("\\u{:04x}", ch as u32)),
            ch => out.push(ch),
        }
    }
    out
}

fn propensity(shard_index: usize) -> u64 {
    1 + ((shard_index.wrapping_mul(7).wrapping_add(3)) % 10) as u64
}

fn make_training_demands(shards: usize, samples: usize, seed: u64) -> Vec<ShardDemand> {
    let mut rng = Rng(seed);
    (0..shards).map(|index| {
        let base = propensity(index) * 1_000;
        let history = (0..samples).map(|_| {
            let factor = rng.between(750_000, 1_250_000);
            ((u128::from(base) * u128::from(factor)) / u128::from(PARTS_PER_MILLION)) as u64
        }).collect();
        ShardDemand {
            instrument_id: INSTRUMENT_ID.into(),
            shard_id: format!("shard-{index:05}"),
            samples: history,
            minimum_capacity: 0,
        }
    }).collect()
}

fn make_eval_scenarios(
    shards: usize,
    scenario_count: usize,
    seed: u64,
    mode: DemandMode,
) -> Vec<Vec<u64>> {
    let mut rng = Rng(seed);
    let rotation = match mode {
        DemandMode::Stationary => 0,
        DemandMode::Shifted => (shards / 2).max(1),
    };
    let mut scenarios = Vec::with_capacity(scenario_count);
    for scenario in 0..scenario_count {
        let mut demand = Vec::with_capacity(shards);
        for index in 0..shards {
            let source_index = (index + rotation) % shards;
            let base = propensity(source_index) * 1_000;
            let factor = rng.between(750_000, 1_250_000);
            let mut amount = ((u128::from(base) * u128::from(factor))
                / u128::from(PARTS_PER_MILLION)) as u64;
            // A deterministic burst makes the held-out trace nontrivial without
            // changing any budget or using floating-point randomness.
            if scenario % 13 == 0 && index == scenario % shards {
                amount = amount.saturating_mul(2);
            }
            demand.push(amount);
        }
        scenarios.push(demand);
    }
    scenarios
}

fn evaluate(plan: &AllocationPlan, scenarios: &[Vec<u64>]) -> Result<Metrics, String> {
    plan.validate().map_err(|e| format!("allocator produced invalid plan: {e:?}"))?;
    let mut result = Metrics {
        scenarios: scenarios.len() as u128,
        requested: 0,
        served: 0,
        unserved: 0,
        stranded: 0,
        worst_scenario_unserved: 0,
        worst_scenario_served_ppm: PARTS_PER_MILLION as u64,
        worst_shard_demand_served_ppm: PARTS_PER_MILLION as u64,
        per_shard: plan.slices.iter().map(|slice| PerShardMetrics {
            shard_id: slice.shard_id.clone(),
            requested: 0,
            served: 0,
            unserved: 0,
            worst_scenario_served_ppm: PARTS_PER_MILLION as u64,
        }).collect(),
        safety_violations: 0,
    };
    let per_shard: Vec<(&str, u64)> = plan.slices.iter()
        .map(|slice| (slice.shard_id.as_str(), slice.capacity))
        .collect();
    for scenario in scenarios {
        if scenario.len() != plan.slices.len() {
            return Err("scenario shard count differs from allocation plan".into());
        }
        let mut wanted = 0_u128;
        let mut delivered = 0_u128;
        let mut unused = 0_u128;
        let mut scenario_unserved = 0_u64;
        for (index, ((_, allocation), demand)) in per_shard.iter().zip(scenario).enumerate() {
            let served = (*allocation).min(*demand);
            wanted += u128::from(*demand);
            delivered += u128::from(served);
            scenario_unserved = scenario_unserved.saturating_add(*demand - served);
            unused += u128::from(allocation.saturating_sub(*demand));

            let shard_metrics = &mut result.per_shard[index];
            shard_metrics.requested += u128::from(*demand);
            shard_metrics.served += u128::from(served);
            shard_metrics.unserved += u128::from(*demand - served);
            if *demand > 0 {
                let shard_served_ppm =
                    (u128::from(served) * u128::from(PARTS_PER_MILLION) / u128::from(*demand)) as u64;
                shard_metrics.worst_scenario_served_ppm =
                    shard_metrics.worst_scenario_served_ppm.min(shard_served_ppm);
            }
        }
        result.requested += wanted;
        result.served += delivered;
        result.unserved += u128::from(scenario_unserved);
        result.stranded += unused;
        result.worst_scenario_unserved = result.worst_scenario_unserved.max(scenario_unserved);
        let scenario_served_ppm = if wanted == 0 {
            u128::from(PARTS_PER_MILLION)
        } else {
            delivered * u128::from(PARTS_PER_MILLION) / wanted
        } as u64;
        result.worst_scenario_served_ppm = result.worst_scenario_served_ppm.min(scenario_served_ppm);
    }
    result.worst_shard_demand_served_ppm = result.per_shard.iter()
        .filter(|shard| shard.requested > 0)
        .map(|shard| {
            (shard.served * u128::from(PARTS_PER_MILLION) / shard.requested) as u64
        })
        .min()
        .unwrap_or(PARTS_PER_MILLION as u64);

    if result.served + result.stranded > u128::from(plan.spendable_capacity) * result.scenarios {
        result.safety_violations = result.safety_violations.saturating_add(1);
    }
    Ok(result)
}

fn metric_json(name: &str, mode: &str, plan: &AllocationPlan, metrics: &Metrics) -> String {
    let demand_served_ppm = if metrics.requested == 0 {
        u128::from(PARTS_PER_MILLION)
    } else {
        metrics.served * u128::from(PARTS_PER_MILLION) / metrics.requested
    };
    let allocated_utilization_ppm = if plan.spendable_capacity == 0 || metrics.scenarios == 0 {
        0
    } else {
        metrics.served * u128::from(PARTS_PER_MILLION)
            / (u128::from(plan.spendable_capacity) * metrics.scenarios)
    };
    let slices = plan.slices.iter().map(|slice| format!(
        "{{\"shard_id\":\"{}\",\"capacity\":{},\"minimum_capacity\":{},\"weight\":{}}}",
        json_escape(&slice.shard_id), slice.capacity, slice.minimum_capacity, slice.allocation_weight
    )).collect::<Vec<_>>().join(",");
    let per_shard = metrics.per_shard.iter().map(|shard| {
        let shard_demand_served_ppm = if shard.requested == 0 {
            PARTS_PER_MILLION as u64
        } else {
            (shard.served * u128::from(PARTS_PER_MILLION) / shard.requested) as u64
        };
        format!(
            "{{\"shard_id\":\"{}\",\"requested_demand_units\":{},\"served_demand_units\":{},\"unserved_demand_units\":{},\"demand_served_ppm\":{},\"worst_scenario_served_ppm\":{}}}",
            json_escape(&shard.shard_id), shard.requested, shard.served, shard.unserved,
            shard_demand_served_ppm, shard.worst_scenario_served_ppm
        )
    }).collect::<Vec<_>>().join(",");
    format!(
        "{{\"policy\":\"{}\",\"demand_mode\":\"{}\",\"instrument_id\":\"{}\",\"scenarios\":{},\"total_capacity\":{},\"spendable_capacity\":{},\"contingency_reserve\":{},\"requested_demand_units\":{},\"served_demand_units\":{},\"unserved_demand_units\":{},\"stranded_reserved_unit_scenarios\":{},\"demand_served_ppm\":{},\"allocation_utilization_ppm\":{},\"worst_scenario_unserved_units\":{},\"worst_scenario_served_ppm\":{},\"worst_shard_demand_served_ppm\":{},\"safety_violations\":{},\"slices\":[{}],\"shard_results\":[{}]}}",
        name, mode, json_escape(&plan.instrument_id), metrics.scenarios, plan.total_capacity, plan.spendable_capacity,
        plan.contingency_reserve, metrics.requested, metrics.served, metrics.unserved,
        metrics.stranded, demand_served_ppm, allocated_utilization_ppm,
        metrics.worst_scenario_unserved, metrics.worst_scenario_served_ppm,
        metrics.worst_shard_demand_served_ppm, metrics.safety_violations, slices, per_shard
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use nested_currency_reference::allocation::{AllocationSlice, AllocationPolicy};

    #[test]
    fn per_shard_metrics_expose_underserved_shard_hidden_by_aggregate() {
        let plan = AllocationPlan {
            instrument_id: "instrument:test".into(),
            total_capacity: 100,
            spendable_capacity: 100,
            contingency_reserve: 0,
            policy: AllocationPolicy::DemandWeighted,
            slices: vec![
                AllocationSlice {
                    shard_id: "shard-a".into(),
                    minimum_capacity: 0,
                    capacity: 90,
                    allocation_weight: 9,
                },
                AllocationSlice {
                    shard_id: "shard-b".into(),
                    minimum_capacity: 0,
                    capacity: 10,
                    allocation_weight: 1,
                },
            ],
        };
        let scenarios = vec![vec![90, 90], vec![90, 90]];
        let metrics = evaluate(&plan, &scenarios).unwrap();
        assert_eq!(metrics.worst_shard_demand_served_ppm, 111_111);
        assert!(metrics.worst_shard_demand_served_ppm < (
            metrics.served * u128::from(PARTS_PER_MILLION) / metrics.requested
        ) as u64);
        let underserved = metrics.per_shard.iter().find(|shard| shard.shard_id == "shard-b").unwrap();
        assert_eq!(underserved.unserved, 160);
        assert_eq!(underserved.served, 20);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let shards = args.first().map(|v| v.parse::<usize>()).transpose()
        .map_err(|e| format!("invalid shard count: {e}"))?.unwrap_or(DEFAULT_SHARDS);
    let training_samples = args.get(1).map(|v| v.parse::<usize>()).transpose()
        .map_err(|e| format!("invalid training sample count: {e}"))?.unwrap_or(DEFAULT_TRAINING_SAMPLES);
    let eval_scenarios = args.get(2).map(|v| v.parse::<usize>()).transpose()
        .map_err(|e| format!("invalid evaluation scenario count: {e}"))?.unwrap_or(DEFAULT_EVAL_SCENARIOS);
    let capacity = args.get(3).map(|v| parse_u64(v, "capacity")).transpose()?.unwrap_or(DEFAULT_CAPACITY);
    let reserve_ppm = args.get(4).map(|v| v.parse::<u32>()).transpose()
        .map_err(|e| format!("invalid reserve ppm: {e}"))?.unwrap_or(DEFAULT_RESERVE_PPM);
    let seed = args.get(5).map(|v| parse_u64(v, "seed")).transpose()?.unwrap_or(DEFAULT_SEED);

    if !(2..=MAX_SHARDS).contains(&shards) {
        return Err(format!("shards must be in 2..={MAX_SHARDS}"));
    }
    if !(1..=MAX_TRAINING_SAMPLES).contains(&training_samples) {
        return Err(format!("training samples must be in 1..={MAX_TRAINING_SAMPLES}"));
    }
    if !(1..=MAX_EVAL_SCENARIOS).contains(&eval_scenarios) {
        return Err(format!("evaluation scenarios must be in 1..={MAX_EVAL_SCENARIOS}"));
    }
    if capacity == 0 {
        return Err("capacity must be positive".into());
    }
    if reserve_ppm > PARTS_PER_MILLION {
        return Err("reserve ppm must be <= 1000000".into());
    }

    let source_sha = require_hex("NCF_SOURCE_SHA", &[40, 64])?;
    let profile_digest = require_hex("NCF_PROFILE_DIGEST", &[64])?;
    let rustc_version = required_env("NCF_RUSTC_VERSION")?;
    let toolchain_id = required_env("NCF_TOOLCHAIN_ID")?;

    let training = make_training_demands(shards, training_samples, seed);
    let stationary = make_eval_scenarios(shards, eval_scenarios, seed ^ 0x53544154494f4e, DemandMode::Stationary);
    let shifted = make_eval_scenarios(shards, eval_scenarios, seed ^ 0x53484946544544, DemandMode::Shifted);
    let policies = [
        ("equal_split", AllocationPolicy::EqualSplit),
        ("demand_weighted", AllocationPolicy::DemandWeighted),
        ("robust_quantile_p95_margin25", AllocationPolicy::RobustQuantile {
            quantile_basis_points: 9_500,
            margin_ppm: 250_000,
        }),
        ("demand_weighted_equal_floor_25", AllocationPolicy::DemandWeightedWithEqualFloor {
            equal_floor_ppm: 250_000,
        }),
        ("demand_weighted_equal_floor_50", AllocationPolicy::DemandWeightedWithEqualFloor {
            equal_floor_ppm: 500_000,
        }),
        ("demand_weighted_equal_floor_75", AllocationPolicy::DemandWeightedWithEqualFloor {
            equal_floor_ppm: 750_000,
        }),
    ];

    let started = Instant::now();
    let mut reports = Vec::with_capacity(policies.len() * 2);
    for (name, policy) in policies {
        let plan = allocate_static_escrow(&training, capacity, reserve_ppm, policy)
            .map_err(|e| format!("allocation policy {name} failed: {e:?}"))?;
        for (mode, label) in [
            (DemandMode::Stationary, "stationary"),
            (DemandMode::Shifted, "shifted"),
        ] {
            let scenarios = match mode {
                DemandMode::Stationary => &stationary,
                DemandMode::Shifted => &shifted,
            };
            let metrics = evaluate(&plan, scenarios)?;
            if metrics.safety_violations != 0 {
                return Err(format!("safety accounting failed for {name} / {label}"));
            }
            reports.push(metric_json(name, label, &plan, &metrics));
        }
    }
    let elapsed_ns = started.elapsed().as_nanos();
    let output = format!(
        "{{\"schema\":\"mycelix.ncf-allocation-benchmark\",\"schema_version\":\"1.0.0\",\"status\":\"completed_reference_simulation\",\"instrument_id\":\"{}\",\"source_commit\":\"{}\",\"profile_digest_sha256\":\"{}\",\"rustc_version\":\"{}\",\"toolchain_id\":\"{}\",\"seed\":{},\"shards\":{},\"training_samples_per_shard\":{},\"evaluation_scenarios_per_mode\":{},\"capacity\":{},\"reserve_ppm\":{},\"elapsed_ns\":{},\"policies\":[{}]}}",
        INSTRUMENT_ID, source_sha, json_escape(&profile_digest), json_escape(&rustc_version), json_escape(&toolchain_id),
        seed, shards, training_samples, eval_scenarios, capacity, reserve_ppm, elapsed_ns, reports.join(",")
    );
    println!("{output}");
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("ncf-allocation-bench: {error}");
        std::process::exit(2);
    }
}
