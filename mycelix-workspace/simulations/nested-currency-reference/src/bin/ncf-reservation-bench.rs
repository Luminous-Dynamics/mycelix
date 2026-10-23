#![forbid(unsafe_code)]

//! Deterministic workload for the partition-safe exposure reservation model.
//! This is a single-process reference benchmark, not a concurrency or network benchmark.

use nested_currency_reference::reservation::{
    ReservationApply, ReservationBook, ReservationError, ReservationKey,
};
use std::{env, fs, time::Instant};

const DEFAULT_OPERATIONS: usize = 1_000_000;
const DEFAULT_SHARDS: usize = 100;
const DEFAULT_SEED: u64 = 0x4d59_4345_4c49_5831;
const MAX_OPERATIONS: usize = 2_000_000;
const MAX_SHARDS: usize = 100_000;
const TOTAL_CAPACITY: u64 = 10_000_000;
const BATCH_SIZE: usize = 1_000;

fn required_env(name: &str) -> Result<String, String> {
    let value = env::var(name).map_err(|_| format!("required environment variable {name} is missing"))?;
    if value.trim().is_empty() {
        return Err(format!("required environment variable {name} is empty"));
    }
    Ok(value)
}

fn require_hex(name: &str, valid_lengths: &[usize]) -> Result<String, String> {
    let value = required_env(name)?;
    if !valid_lengths.contains(&value.len())
        || !value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(format!("{name} must be lowercase hexadecimal with length in {valid_lengths:?}"));
    }
    Ok(value)
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

fn parse_u64(value: &str, label: &str) -> Result<u64, String> {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|e| format!("invalid {label}: {e}"))
    } else {
        value.parse::<u64>().map_err(|e| format!("invalid {label}: {e}"))
    }
}

fn percentile(sorted: &[u128], percent: usize) -> u128 {
    if sorted.is_empty() { return 0; }
    let index = ((sorted.len() - 1) * percent).div_ceil(100);
    sorted[index.min(sorted.len() - 1)]
}

fn peak_rss_bytes() -> Option<u64> {
    let status = fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmHWM:"))?;
    let kib = line.split_whitespace().nth(1)?.parse::<u64>().ok()?;
    kib.checked_mul(1024)
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let operations = args.first().map(|s| s.parse::<usize>())
        .transpose().map_err(|e| format!("invalid operation count: {e}"))?
        .unwrap_or(DEFAULT_OPERATIONS);
    let shards = args.get(1).map(|s| s.parse::<usize>())
        .transpose().map_err(|e| format!("invalid shard count: {e}"))?
        .unwrap_or(DEFAULT_SHARDS);
    let seed = args.get(2).map(|s| parse_u64(s, "seed"))
        .transpose()?.unwrap_or(DEFAULT_SEED);

    if operations == 0 || operations > MAX_OPERATIONS {
        return Err(format!("operation count must be between 1 and {MAX_OPERATIONS}"));
    }
    if shards < 2 || shards > MAX_SHARDS || shards as u64 > TOTAL_CAPACITY {
        return Err(format!("shard count must be between 2 and {}", MAX_SHARDS.min(TOTAL_CAPACITY as usize)));
    }

    let source_sha = require_hex("NCF_SOURCE_SHA", &[40, 64])?;
    let profile_digest = require_hex("NCF_PROFILE_DIGEST", &[64])?;
    let rustc_version = required_env("NCF_RUSTC_VERSION")?;
    let toolchain_id = required_env("NCF_TOOLCHAIN_ID")?;

    let mut book = ReservationBook::default();
    for community in ["community:a", "community:b"] {
        book.set_community_cap(community, "unit:local", TOTAL_CAPACITY)
            .map_err(|e| format!("configure community cap: {e:?}"))?;
    }
    book.set_agreement_cap("edge:a-b", "unit:local", TOTAL_CAPACITY)
        .map_err(|e| format!("configure agreement cap: {e:?}"))?;

    let base_slice = TOTAL_CAPACITY / shards as u64;
    let remainder = TOTAL_CAPACITY % shards as u64;
    let mut keys = Vec::with_capacity(shards);
    let mut allocated_capacity = 0_u64;
    for index in 0..shards {
        let key = ReservationKey {
            agreement_id: "edge:a-b".into(),
            instrument: "unit:local".into(),
            shard_id: format!("shard:{index}"),
            epoch: 1,
        };
        let capacity = base_slice + if (index as u64) < remainder { 1 } else { 0 };
        book.reserve(key.clone(), "community:a", "community:b", capacity)
            .map_err(|e| format!("reserve slice for shard {index}: {e:?}"))?;
        allocated_capacity = allocated_capacity.checked_add(capacity)
            .ok_or_else(|| "allocated capacity overflow".to_string())?;
        keys.push(key);
    }

    // Adversarial idempotency controls run outside the timed loop and stay in the receipt.
    let replay_key = &keys[0];
    book.record_obligation("bench:replay-control", replay_key, 1)
        .map_err(|e| format!("initial replay-control obligation failed: {e:?}"))?;
    let same_retry = book.record_obligation("bench:replay-control", replay_key, 1)
        .map_err(|e| format!("same-key replay control failed: {e:?}"))?;
    let cross_reservation_retry = if keys.len() > 1 {
        book.record_obligation("bench:replay-control", &keys[1], 1)
    } else {
        Err(ReservationError::ReplayMismatch)
    };
    if same_retry != ReservationApply::Duplicate
        || cross_reservation_retry != Err(ReservationError::ReplayMismatch)
    {
        return Err("reservation replay negative controls did not produce expected outcomes".into());
    }

    let start = Instant::now();
    let mut state = seed;
    let mut accepted_obligations = 0_u64;
    let mut settlements = 0_u64;
    let mut capacity_rejections = 0_u64;
    let mut event_errors = 0_u64;
    let mut batch_latencies = Vec::with_capacity(operations.div_ceil(BATCH_SIZE));
    let mut batch_start = Instant::now();

    for i in 0..operations {
        state = state.wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let shard = (state as usize) % shards;
        state = state.wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let current = book.outstanding(&keys[shard])
            .map_err(|e| format!("read shard outstanding: {e:?}"))?;

        if current > 0 && state % 5 == 0 {
            let limit = current.min(50);
            let amount = (state % limit) + 1;
            match book.settle(&format!("settle:{i}"), &keys[shard], amount) {
                Ok(ReservationApply::Applied) => settlements += 1,
                Ok(ReservationApply::Duplicate) => event_errors += 1,
                Err(ReservationError::SettlementTooLarge | ReservationError::NothingOutstanding) => event_errors += 1,
                Err(e) => return Err(format!("unexpected settlement error: {e:?}")),
            }
        } else {
            let amount = (state % 50) + 1;
            match book.record_obligation(&format!("obligation:{i}"), &keys[shard], amount) {
                Ok(ReservationApply::Applied) => accepted_obligations += 1,
                Ok(ReservationApply::Duplicate) => event_errors += 1,
                Err(ReservationError::CapacityExceeded) => capacity_rejections += 1,
                Err(e) => return Err(format!("unexpected obligation error: {e:?}")),
            }
        }

        if (i + 1) % BATCH_SIZE == 0 || i + 1 == operations {
            batch_latencies.push(batch_start.elapsed().as_nanos());
            batch_start = Instant::now();
        }
    }

    let elapsed_ns = start.elapsed().as_nanos().max(1);
    batch_latencies.sort_unstable();
    let mut used_capacity = 0_u64;
    let mut max_shard_outstanding = 0_u64;
    for key in &keys {
        let used = book.outstanding(key).map_err(|e| format!("read final outstanding: {e:?}"))?;
        used_capacity = used_capacity.checked_add(used)
            .ok_or_else(|| "used capacity overflow".to_string())?;
        max_shard_outstanding = max_shard_outstanding.max(used);
        if used > book.reservation_capacity(key).map_err(|e| format!("read reservation: {e:?}"))? {
            return Err("final invariant violation: a shard exceeded its reserved capacity".into());
        }
    }
    let allocated_a = book.allocated_to_community("community:a", "unit:local")
        .map_err(|e| format!("verify community A allocation: {e:?}"))?;
    let allocated_b = book.allocated_to_community("community:b", "unit:local")
        .map_err(|e| format!("verify community B allocation: {e:?}"))?;
    let allocated_agreement = book.allocated_to_agreement("edge:a-b", "unit:local")
        .map_err(|e| format!("verify agreement allocation: {e:?}"))?;
    if allocated_a != allocated_capacity || allocated_b != allocated_capacity
        || allocated_agreement != allocated_capacity
        || allocated_capacity > TOTAL_CAPACITY
    {
        return Err("final invariant violation: reservations exceed a configured risk cap".into());
    }
    let unused_reserved_capacity = allocated_capacity.checked_sub(used_capacity)
        .ok_or_else(|| "used capacity exceeded aggregate reservation".to_string())?;
    let cap_utilization_ppm = ((used_capacity as u128) * 1_000_000 / allocated_capacity.max(1) as u128) as u64;
    let throughput = operations as f64 * 1_000_000_000.0 / elapsed_ns as f64;
    let rss = peak_rss_bytes().map(|value| value.to_string()).unwrap_or_else(|| "null".into());

    println!(
        concat!(
            "{{",
            "\"schema\":\"mycelix.ncf-reservation-benchmark.v1\",",
            "\"source_commit\":\"{}\",",
            "\"profile_digest_sha256\":\"{}\",",
            "\"rustc_version\":\"{}\",",
            "\"toolchain_id\":\"{}\",",
            "\"operations\":{},\"shards\":{},\"seed\":\"0x{:016x}\",",
            "\"allocated_capacity\":{},\"used_capacity\":{},\"unused_reserved_capacity\":{},",
            "\"cap_utilization_ppm\":{},\"max_shard_outstanding\":{},",
            "\"accepted_obligations\":{},\"settlements\":{},\"capacity_rejections\":{},",
            "\"event_errors\":{},\"replay_control_same_key\":\"duplicate\",",
            "\"replay_control_cross_reservation\":\"rejected\",",
            "\"elapsed_ns\":{},\"throughput_ops_per_second\":{:.3},",
            "\"batch_latency_ns_p50\":{},\"batch_latency_ns_p95\":{},\"batch_latency_ns_p99\":{},",
            "\"peak_rss_bytes\":{},",
            "\"claim_ceiling\":\"single-process reservation-model benchmark; no network, concurrency, Holochain, or production settlement claim\",",
            "\"measurement_note\":\"reserved capacity is not dynamically reassigned between shards; stranded capacity is a deliberate safety/utilization trade-off\",",
            "\"invariants_checked\":{{\"gross_outstanding_le_reservation\":true,\"sum_reserved_le_configured_caps\":true}}",
            "}}"
        ),
        json_escape(&source_sha),
        json_escape(&profile_digest),
        json_escape(&rustc_version),
        json_escape(&toolchain_id),
        operations,
        shards,
        seed,
        allocated_capacity,
        used_capacity,
        unused_reserved_capacity,
        cap_utilization_ppm,
        max_shard_outstanding,
        accepted_obligations,
        settlements,
        capacity_rejections,
        event_errors,
        elapsed_ns,
        throughput,
        percentile(&batch_latencies, 50),
        percentile(&batch_latencies, 95),
        percentile(&batch_latencies, 99),
        rss
    );

    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("ncf-reservation-bench: {error}");
        std::process::exit(2);
    }
}
