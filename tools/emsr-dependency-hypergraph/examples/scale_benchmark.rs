//! Reproducible scale benchmark for the dependency-hypergraph evaluator.
//!
//! Run with:
//!   cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark
//!
//! This benchmark generates only synthetic dependency inventories. It does not
//! read hardware, sensor feeds, or field data.

use emsr_dependency_hypergraph::{
    independently_verify, evaluate_bytes, AssertionState, DependencyDomain,
    DependencyHyperedge, DependencyQuery, HypergraphInput, InventorySnapshot,
    Node, NodeKind, INPUT_CONTRACT,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::env;
use std::fs;
use std::process::Command;
use std::time::{Duration, Instant};

const INPUT_LIMIT_BYTES: usize = 16 * 1024 * 1024;
const DEFAULT_REPETITIONS: usize = 101;
const SOURCE_COUNTS: [usize; 5] = [10, 100, 1_000, 10_000, 100_000];

#[derive(Debug, Serialize)]
struct BenchmarkDocument {
    schema: &'static str,
    version: &'static str,
    state: &'static str,
    synthetic_only: bool,
    non_operational: bool,
    rustc_version: String,
    commit_sha: String,
    operating_system: &'static str,
    architecture: &'static str,
    debug_assertions: bool,
    repetitions: usize,
    warmup_runs_per_measurement: usize,
    cases: Vec<BenchmarkCase>,
    claim_ceiling: &'static str,
}

#[derive(Debug, Serialize)]
struct BenchmarkCase {
    source_count: usize,
    node_count: usize,
    hyperedge_count: usize,
    input_bytes: usize,
    input_sha256: String,
    evaluate: TimingSummary,
    independent_reference_verify: TimingSummary,
    evaluator_disposition: String,
    independent_verifier_status: String,
    independent_verifier_checks: usize,
    peak_rss_kib_after_case: Option<u64>,
    status: &'static str,
    note: Option<String>,
}

#[derive(Debug, Serialize)]
struct TimingSummary {
    repetitions: usize,
    p50_ms: f64,
    p95_ms: f64,
    p99_ms: f64,
    min_ms: f64,
    max_ms: f64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("scale_benchmark_error: {error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let repetitions = env::var("GCPN_BENCH_REPETITIONS")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(DEFAULT_REPETITIONS)
        .clamp(101, 501);

    let mut cases = Vec::with_capacity(SOURCE_COUNTS.len());
    for source_count in SOURCE_COUNTS {
        cases.push(run_case(source_count, repetitions)?);
    }

    let document = BenchmarkDocument {
        schema: "gcpn.dependency-hypergraph-scale-benchmark",
        version: "0.1.0",
        state: "SYNTHETIC_MEASUREMENT",
        synthetic_only: true,
        non_operational: true,
        rustc_version: rustc_version(),
        commit_sha: env::var("GITHUB_SHA").or_else(|_| env::var("GIT_COMMIT")).unwrap_or_else(|_| "unavailable".into()),
        operating_system: env::consts::OS,
        architecture: env::consts::ARCH,
        debug_assertions: cfg!(debug_assertions),
        repetitions,
        warmup_runs_per_measurement: 1,
        cases,
        claim_ceiling: "Measurements apply only to this synthetic star-hyperedge graph, exact source revision, toolchain and host. They do not establish field throughput, physical independence, sensor performance or operational safety.",
    };

    let json = serde_json::to_string_pretty(&document)?;
    if let Some(path) = env::args().nth(1) {
        fs::write(path, format!("{json}\n"))?;
    }
    println!("{json}");
    Ok(())
}

fn run_case(source_count: usize, repetitions: usize) -> Result<BenchmarkCase, Box<dyn std::error::Error>> {
    let input = build_star_graph(source_count);
    let input_bytes = serde_json::to_vec(&input)?;
    let input_sha256 = sha256_hex(&input_bytes);

    if input_bytes.len() > INPUT_LIMIT_BYTES {
        return Ok(BenchmarkCase {
            source_count,
            node_count: input.nodes.len(),
            hyperedge_count: input.hyperedges.len(),
            input_bytes: input_bytes.len(),
            input_sha256,
            evaluate: empty_timing(),
            independent_reference_verify: empty_timing(),
            evaluator_disposition: "NOT_RUN".into(),
            independent_verifier_status: "NOT_RUN".into(),
            independent_verifier_checks: 0,
            peak_rss_kib_after_case: peak_rss_kib(),
            status: "REJECTED_BY_INPUT_SIZE_LIMIT",
            note: Some("Synthetic case exceeds the production 16 MiB parser cap; the cap was not relaxed for benchmarking.".into()),
        });
    }

    // One unmeasured warm-up removes first-use allocator/codepath setup from the sample percentiles.
    let warmup_report = evaluate_bytes(&input_bytes);
    let warmup_report_bytes = serde_json::to_vec(&warmup_report)?;
    let _ = independently_verify(&input_bytes, &warmup_report_bytes);

    let mut evaluate_times = Vec::with_capacity(repetitions);
    let mut last_report = None;
    for _ in 0..repetitions {
        let start = Instant::now();
        let report = evaluate_bytes(&input_bytes);
        evaluate_times.push(start.elapsed());
        last_report = Some(report);
    }
    let report = last_report.ok_or_else(|| std::io::Error::other("no evaluator repetitions executed"))?;
    let report_bytes = serde_json::to_vec(&report)?;

    let mut verify_times = Vec::with_capacity(repetitions);
    let mut last_verification = None;
    for _ in 0..repetitions {
        let start = Instant::now();
        let verification = independently_verify(&input_bytes, &report_bytes);
        verify_times.push(start.elapsed());
        last_verification = Some(verification);
    }
    let verification = last_verification.ok_or_else(|| std::io::Error::other("no verifier repetitions executed"))?;

    let status = if report.disposition == emsr_dependency_hypergraph::Disposition::SharedDependency
        && verification.status == "verified"
    {
        "PASS"
    } else {
        "FAIL"
    };

    Ok(BenchmarkCase {
        source_count,
        node_count: input.nodes.len(),
        hyperedge_count: input.hyperedges.len(),
        input_bytes: input_bytes.len(),
        input_sha256,
        evaluate: summarize(&mut evaluate_times),
        independent_reference_verify: summarize(&mut verify_times),
        evaluator_disposition: format!("{:?}", report.disposition),
        independent_verifier_status: verification.status,
        independent_verifier_checks: verification.checks.len(),
        peak_rss_kib_after_case: peak_rss_kib(),
        status,
        note: None,
    })
}

/// Star-shaped synthetic dependency graph:
/// N sources -> one shared clock dependency, represented by one hyperedge.
/// This stresses source validation, multi-subject hyperedges, closure, set
/// intersection, strict JSON parsing and independent replay at increasing N.
fn build_star_graph(source_count: usize) -> HypergraphInput {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("s{index:08}"))
        .collect();

    let mut nodes: Vec<Node> = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect();
    nodes.push(Node {
        id: "shared-clock".into(),
        kind: NodeKind::Dependency,
    });

    let mut covered_node_ids = source_ids.clone();
    covered_node_ids.push("shared-clock".into());
    covered_node_ids.sort();

    let mut hyperedge_subjects = source_ids.clone();
    hyperedge_subjects.sort();

    let edge = DependencyHyperedge {
        id: "all-sources-share-clock".into(),
        subject_ids: hyperedge_subjects,
        dependency_id: "shared-clock".into(),
        domain: DependencyDomain::Clock,
        valid_from: Some(0),
        valid_until: None,
        recorded_at: 80,
        evidence_ref: "synthetic:clock-inventory-v0.1".into(),
        state: AssertionState::Qualified,
    };

    HypergraphInput {
        contract: INPUT_CONTRACT.into(),
        snapshot: InventorySnapshot {
            snapshot_id: format!("scale-star-{source_count}"),
            as_of: 100,
            evidence_frontier: 120,
            complete: true,
            scoped_source_ids: source_ids.clone(),
            covered_node_ids,
        },
        query: DependencyQuery {
            source_ids,
            domain: DependencyDomain::Clock,
            event_time: 90,
            expected_snapshot_id: format!("scale-star-{source_count}"),
            expected_evidence_frontier: 120,
        },
        nodes,
        hyperedges: vec![edge],
    }
}

fn summarize(times: &mut [Duration]) -> TimingSummary {
    times.sort_unstable();
    let values: Vec<f64> = times.iter().map(|time| time.as_secs_f64() * 1_000.0).collect();
    TimingSummary {
        repetitions: times.len(),
        p50_ms: quantile(&values, 0.50),
        p95_ms: quantile(&values, 0.95),
        p99_ms: quantile(&values, 0.99),
        min_ms: values.first().copied().unwrap_or(0.0),
        max_ms: values.last().copied().unwrap_or(0.0),
    }
}

fn quantile(values: &[f64], probability: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let index = ((values.len() - 1) as f64 * probability).ceil() as usize;
    values[index.min(values.len() - 1)]
}

fn empty_timing() -> TimingSummary {
    TimingSummary {
        repetitions: 0,
        p50_ms: 0.0,
        p95_ms: 0.0,
        p99_ms: 0.0,
        min_ms: 0.0,
        max_ms: 0.0,
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn rustc_version() -> String {
    Command::new("rustc")
        .arg("--version")
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
        .unwrap_or_else(|| "unavailable".into())
}

#[cfg(target_os = "linux")]
fn peak_rss_kib() -> Option<u64> {
    let status = fs::read_to_string("/proc/self/status").ok()?;
    let line = status.lines().find(|line| line.starts_with("VmHWM:"))?;
    line.split_whitespace().nth(1)?.parse().ok()
}

#[cfg(not(target_os = "linux"))]
fn peak_rss_kib() -> Option<u64> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_sizes_remain_within_declared_test_matrix() {
        assert_eq!(SOURCE_COUNTS, [10, 100, 1_000, 10_000, 100_000]);
    }

    #[test]
    fn star_graph_has_one_shared_root_and_sorted_subjects() {
        let input = build_star_graph(10);
        assert_eq!(input.nodes.len(), 11);
        assert_eq!(input.hyperedges.len(), 1);
        assert_eq!(input.hyperedges[0].subject_ids.len(), 10);
        assert!(input.hyperedges[0].subject_ids.windows(2).all(|pair| pair[0] < pair[1]));
        assert!(input.query.source_ids.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(input.hyperedges[0].dependency_id, "shared-clock");
    }

    #[test]
    fn quantile_is_deterministic_and_nearest_rank() {
        let values = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(quantile(&values, 0.50), 3.0);
        assert_eq!(quantile(&values, 0.95), 5.0);
        assert_eq!(quantile(&values, 0.99), 5.0);
    }

    #[test]
    fn thousand_source_scale_case_is_comparable_and_replayable() {
        let input = build_star_graph(1_000);
        let bytes = serde_json::to_vec(&input).unwrap();
        assert!(bytes.len() <= INPUT_LIMIT_BYTES);
        let report = evaluate_bytes(&bytes);
        assert_eq!(
            report.disposition,
            emsr_dependency_hypergraph::Disposition::SharedDependency
        );
        assert_eq!(report.common_dependency_ids, vec!["shared-clock"]);
        let report_bytes = serde_json::to_vec(&report).unwrap();
        let verification = independently_verify(&bytes, &report_bytes);
        assert_eq!(verification.status, "verified");
    }

}
