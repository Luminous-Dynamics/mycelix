#![forbid(unsafe_code)]

//! Deterministic, dependency-free mutual-credit benchmark.
//! Output is a machine-readable measurement record, never a qualification verdict.

use nested_currency_reference::{Apply, Exchange, Family, MutualCredit, Profile, Registry};
use std::{env, fs};
use std::time::Instant;

const DEFAULT_OPERATIONS: usize = 1_000_000;
const DEFAULT_PARTICIPANTS: usize = 1_000;
const DEFAULT_SEED: u64 = 0x4d59_4345_4c49_5831;
const MAX_OPERATIONS: usize = 2_000_000;
const MAX_PARTICIPANTS: usize = 1_000_000;
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

fn parse_seed(value: &str) -> Result<u64, String> {
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).map_err(|e| format!("invalid hexadecimal seed: {e}"))
    } else {
        value.parse::<u64>().map_err(|e| format!("invalid seed: {e}"))
    }
}

fn percentile(sorted: &[u128], percent: usize) -> u128 {
    if sorted.is_empty() {
        return 0;
    }
    let index = ((sorted.len() - 1) * percent) / 100;
    sorted[index]
}

/// Linux VmHWM reports the process's high-water resident set size in KiB.
/// Non-Linux platforms return None rather than inventing a memory value.
fn peak_rss_bytes() -> Option<u64> {
    let status = fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmHWM:"))?;
    let kib = line.split_whitespace().nth(1)?.parse::<u64>().ok()?;
    kib.checked_mul(1024)
}

fn json_optional_u64(value: Option<u64>) -> String {
    value.map(|number| number.to_string()).unwrap_or_else(|| "null".into())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("ncf-bench: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() > 3 {
        return Err("usage: ncf-bench [operations] [participants] [seed-or-0x-hex]".into());
    }

    let operations = args.first()
        .map(|v| v.parse::<usize>().map_err(|e| format!("invalid operations count: {e}")))
        .transpose()?
        .unwrap_or(DEFAULT_OPERATIONS);
    let participants = args.get(1)
        .map(|v| v.parse::<usize>().map_err(|e| format!("invalid participant count: {e}")))
        .transpose()?
        .unwrap_or(DEFAULT_PARTICIPANTS);
    let seed = args.get(2).map(|v| parse_seed(v)).transpose()?.unwrap_or(DEFAULT_SEED);

    if operations == 0 || operations > MAX_OPERATIONS {
        return Err(format!("operations must be in 1..={MAX_OPERATIONS}"));
    }
    if participants < 2 || participants > MAX_PARTICIPANTS {
        return Err(format!("participants must be in 2..={MAX_PARTICIPANTS}"));
    }

    // A result without bound source/toolchain/profile identity is not emitted.
    let source_sha = require_hex("NCF_SOURCE_SHA", &[40, 64])?;
    let profile_digest = require_hex("NCF_PROFILE_DIGEST", &[64])?;
    let rustc_version = required_env("NCF_RUSTC_VERSION")?;
    let toolchain_id = required_env("NCF_TOOLCHAIN_ID")?;

    let profile = Profile {
        id: "ncf:benchmark:mutual-credit".into(),
        issuer: "did:mycelix:benchmark".into(),
        symbol: "NCF-BENCH".into(),
        family: Family::MutualCredit,
        unit_code: "ncf:benchmark-unit".into(),
        scale: 0,
        revision: 1,
        active: true,
        max_abs_balance: 1_000_000_000,
    };
    profile.validate().map_err(|e| format!("invalid benchmark profile: {e:?}"))?;
    let instrument_id = profile.id.clone();
    let mut registry = Registry::default();
    registry.register(profile.clone()).map_err(|e| format!("profile registration failed: {e:?}"))?;

    let members: Vec<String> = (0..participants).map(|i| format!("member:{i}")).collect();
    let batch_count = operations.div_ceil(BATCH_SIZE);
    let mut batch_latency_ns = Vec::with_capacity(batch_count);
    let mut ledger = MutualCredit::default();
    let mut state = seed;
    let wall_start = Instant::now();
    let mut completed = 0_usize;

    while completed < operations {
        let count = (operations - completed).min(BATCH_SIZE);
        let batch_start = Instant::now();
        for offset in 0..count {
            state = state.wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let provider_index = state as usize % members.len();

            state = state.wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            let mut receiver_index = state as usize % members.len();
            if receiver_index == provider_index {
                receiver_index = (receiver_index + 1) % members.len();
            }
            let amount = ((state >> 32) % 3 + 1) as i64;
            let sequence = completed + offset;
            let event = Exchange {
                id: format!("bench:{sequence}"),
                instrument: instrument_id.clone(),
                profile_revision: profile.revision,
                provider: members[provider_index].clone(),
                receiver: members[receiver_index].clone(),
                amount,
            };
            match ledger.apply(&registry, event).map_err(|e| format!("operation {sequence} failed: {e:?}"))? {
                Apply::Applied => {}
                Apply::Duplicate => return Err(format!("unexpected duplicate event at sequence {sequence}")),
            }
        }
        completed += count;

        let net = ledger.net(&instrument_id);
        if net != 0 {
            return Err(format!("conservation invariant failed after {completed} operations: net={net}"));
        }
        batch_latency_ns.push(batch_start.elapsed().as_nanos());
    }

    let elapsed_ns = wall_start.elapsed().as_nanos();
    if ledger.event_count() != operations {
        return Err(format!(
            "event-count invariant failed: expected {operations}, got {}",
            ledger.event_count()
        ));
    }
    let throughput_ops_s = operations as f64 * 1_000_000_000.0 / elapsed_ns.max(1) as f64;
    batch_latency_ns.sort_unstable();
    let p50 = percentile(&batch_latency_ns, 50);
    let p95 = percentile(&batch_latency_ns, 95);
    let p99 = percentile(&batch_latency_ns, 99);
    let peak_rss = json_optional_u64(peak_rss_bytes());

    println!(
        concat!(
            "{{\n",
            "  \"schema\": \"mycelix.nested-currency-benchmark\",\n",
            "  \"schema_version\": \"1.0.0\",\n",
            "  \"status\": \"completed_invariant_checked\",\n",
            "  \"source_commit\": \"{}\",\n",
            "  \"profile_digest_sha256\": \"{}\",\n",
            "  \"rustc_version\": \"{}\",\n",
            "  \"toolchain_id\": \"{}\",\n",
            "  \"parameters\": {{\"operations\": {}, \"participants\": {}, \"seed\": \"0x{:016x}\", \"batch_size\": {}}},\n",
            "  \"metrics\": {{\"elapsed_ns\": {}, \"throughput_ops_per_second\": {:.3}, \"batch_count\": {}, \"batch_latency_ns_p50\": {}, \"batch_latency_ns_p95\": {}, \"batch_latency_ns_p99\": {}, \"net_position\": {}, \"accepted_event_count\": {}, \"peak_rss_bytes\": {}}},\n",
            "  \"measurement_notes\": [\"Batch latency includes the post-batch conservation scan.\", \"Throughput includes registry lookups, BTreeMap accounting, event persistence, and the conservation scan.\", \"peak_rss_bytes reads Linux /proc VmHWM and is null on unsupported platforms.\", \"This is a single-process reference-model benchmark, not a Holochain/DHT or external settlement benchmark.\", \"One result is not SOTA evidence; repeat across seeds, machines, toolchains, and matched implementations.\" ]\n",
            "}}"
        ),
        json_escape(&source_sha),
        json_escape(&profile_digest),
        json_escape(&rustc_version),
        json_escape(&toolchain_id),
        operations,
        participants,
        seed,
        BATCH_SIZE,
        elapsed_ns,
        throughput_ops_s,
        batch_latency_ns.len(),
        p50,
        p95,
        p99,
        ledger.net(&instrument_id),
        ledger.event_count(),
        peak_rss,
    );
    Ok(())
}
