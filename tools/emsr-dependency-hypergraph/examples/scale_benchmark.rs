//! Reproducible scale benchmark for the dependency-hypergraph evaluator.
//!
//! Run with:
//!   cargo run --release --manifest-path tools/emsr-dependency-hypergraph/Cargo.toml --example scale_benchmark
//!
//! This benchmark generates only synthetic dependency inventories. It does not
//! read hardware, sensor feeds, or field data.

use emsr_dependency_hypergraph::{
    AssertionState, DependencyDomain, DependencyHyperedge, DependencyQuery, HypergraphInput,
    INPUT_CONTRACT, InventorySnapshot, Node, NodeKind, evaluate_bytes, independently_verify,
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
    topology: String,
    source_count: usize,
    expected_disposition: String,
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

    let mut cases = Vec::new();

    // Star is the only 100k case in v0.2. The 100k input remains below the
    // unchanged production parser cap; richer topologies run through 10k.
    for source_count in SOURCE_COUNTS {
        let (input, expected) = build_star_graph(source_count);
        cases.push(run_case("star_shared_root", input, expected, repetitions)?);
    }
    for source_count in [10, 100, 1_000, 10_000] {
        let (input, expected) = build_chain_graph(source_count, 128);
        cases.push(run_case("deep_shared_chain", input, expected, repetitions)?);

        let (input, expected) = build_layered_graph(source_count);
        cases.push(run_case("layered_hierarchy", input, expected, repetitions)?);

        let (input, expected) = build_independent_graph(source_count);
        cases.push(run_case("sparse_independent", input, expected, repetitions)?);

        let (input, expected) = build_unknown_graph(source_count);
        cases.push(run_case("unknown_dependency", input, expected, repetitions)?);

        for (domain, label) in [
            (DependencyDomain::Clock, "overlap_clock"),
            (DependencyDomain::Power, "overlap_power"),
            (DependencyDomain::Network, "overlap_network"),
        ] {
            let (input, expected) = build_multi_domain_graph(source_count, domain);
            cases.push(run_case(label, input, expected, repetitions)?);
        }
    }

    let document = BenchmarkDocument {
        schema: "gcpn.dependency-hypergraph-scale-benchmark",
        version: "0.2.0",
        state: "SYNTHETIC_MEASUREMENT",
        synthetic_only: true,
        non_operational: true,
        rustc_version: rustc_version(),
        commit_sha: env::var("GITHUB_SHA")
            .or_else(|_| env::var("GIT_COMMIT"))
            .unwrap_or_else(|_| "unavailable".into()),
        operating_system: env::consts::OS,
        architecture: env::consts::ARCH,
        debug_assertions: cfg!(debug_assertions),
        repetitions,
        warmup_runs_per_measurement: 1,
        cases,
        claim_ceiling: "Measurements apply only to the exact synthetic graph topologies, input digests, source revision, toolchain and host. They do not establish field throughput, physical independence, sensor performance or operational safety.",
    };

    let json = serde_json::to_string_pretty(&document)?;
    if let Some(path) = env::args().nth(1) {
        fs::write(path, format!("{json}\n"))?;
    }
    println!("{json}");
    Ok(())
}

fn run_case(
    topology: &str,
    input: HypergraphInput,
    expected_disposition: &'static str,
    repetitions: usize,
) -> Result<BenchmarkCase, Box<dyn std::error::Error>> {
    let source_count = input.query.source_ids.len();
    let input_bytes = serde_json::to_vec(&input)?;
    let input_sha256 = sha256_hex(&input_bytes);

    if input_bytes.len() > INPUT_LIMIT_BYTES {
        return Ok(BenchmarkCase {
            topology: topology.to_owned(),
            source_count,
            expected_disposition: expected_disposition.to_owned(),
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
    let report =
        last_report.ok_or_else(|| std::io::Error::other("no evaluator repetitions executed"))?;
    let report_bytes = serde_json::to_vec(&report)?;

    let mut verify_times = Vec::with_capacity(repetitions);
    let mut last_verification = None;
    for _ in 0..repetitions {
        let start = Instant::now();
        let verification = independently_verify(&input_bytes, &report_bytes);
        verify_times.push(start.elapsed());
        last_verification = Some(verification);
    }
    let verification = last_verification
        .ok_or_else(|| std::io::Error::other("no verifier repetitions executed"))?;

    let actual_disposition = format!("{:?}", report.disposition);
    let status = if actual_disposition == expected_disposition
        && verification.status == "verified"
    {
        "PASS"
    } else {
        "FAIL"
    };

    Ok(BenchmarkCase {
        topology: topology.to_owned(),
        source_count,
        expected_disposition: expected_disposition.to_owned(),
        node_count: input.nodes.len(),
        hyperedge_count: input.hyperedges.len(),
        input_bytes: input_bytes.len(),
        input_sha256,
        evaluate: summarize(&mut evaluate_times),
        independent_reference_verify: summarize(&mut verify_times),
        evaluator_disposition: actual_disposition,
        independent_verifier_status: verification.status,
        independent_verifier_checks: verification.checks.len(),
        peak_rss_kib_after_case: peak_rss_kib(),
        status,
        note: None,
    })
}

/// Star-shaped synthetic dependency graph: N sources -> one common clock.
fn build_star_graph(source_count: usize) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("s{index:08}"))
        .collect();
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    nodes.push(Node {
        id: "shared-clock".into(),
        kind: NodeKind::Dependency,
    });
    let edge = make_edge(
        "all-sources-share-clock",
        source_ids.clone(),
        "shared-clock",
        DependencyDomain::Clock,
        AssertionState::Qualified,
    );
    (
        build_input(source_ids, nodes, vec![edge], DependencyDomain::Clock, source_count, "star"),
        "SharedDependency",
    )
}

/// Every source enters a common 128-deep (or shorter) chain before reaching
/// the root. This exercises traversal depth as well as a large source set.
fn build_chain_graph(source_count: usize, requested_depth: usize) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("chain-source-{index:08}"))
        .collect();
    let depth = requested_depth.max(1);
    let root = format!("chain-node-{depth:05}");
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    for index in 0..=depth {
        nodes.push(Node {
            id: format!("chain-node-{index:05}"),
            kind: NodeKind::Dependency,
        });
    }
    let mut edges = Vec::with_capacity(depth + 1);
    edges.push(make_edge(
        "chain-source-entry",
        source_ids.clone(),
        "chain-node-00000",
        DependencyDomain::Clock,
        AssertionState::Qualified,
    ));
    for index in 0..depth {
        edges.push(make_edge(
            &format!("chain-edge-{index:05}"),
            vec![format!("chain-node-{index:05}")],
            format!("chain-node-{:05}", index + 1),
            DependencyDomain::Clock,
            AssertionState::Qualified,
        ));
    }
    let _ = root;
    (
        build_input(source_ids, nodes, edges, DependencyDomain::Clock, 20, "chain"),
        "SharedDependency",
    )
}

/// Sources are partitioned into local regions; region dependencies converge
/// through zones and a common root.
fn build_layered_graph(source_count: usize) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("layer-source-{index:08}"))
        .collect();
    let region_size = 100usize;
    let zone_size = 10usize;
    let region_count = source_count.div_ceil(region_size);
    let zone_count = region_count.div_ceil(zone_size);
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::new();
    let mut region_ids = Vec::with_capacity(region_count);
    for region in 0..region_count {
        let region_id = format!("region-{region:05}");
        region_ids.push(region_id.clone());
        nodes.push(Node {
            id: region_id.clone(),
            kind: NodeKind::Dependency,
        });
        let start = region * region_size;
        let end = (start + region_size).min(source_count);
        edges.push(make_edge(
            &format!("sources-to-{region_id}"),
            source_ids[start..end].to_vec(),
            region_id,
            DependencyDomain::DataPipeline,
            AssertionState::Qualified,
        ));
    }
    let mut zone_ids = Vec::with_capacity(zone_count);
    for zone in 0..zone_count {
        let zone_id = format!("zone-{zone:05}");
        zone_ids.push(zone_id.clone());
        nodes.push(Node {
            id: zone_id.clone(),
            kind: NodeKind::Dependency,
        });
        let start = zone * zone_size;
        let end = (start + zone_size).min(region_count);
        edges.push(make_edge(
            &format!("regions-to-{zone_id}"),
            region_ids[start..end].to_vec(),
            zone_id,
            DependencyDomain::DataPipeline,
            AssertionState::Qualified,
        ));
    }
    nodes.push(Node {
        id: "layered-global-root".into(),
        kind: NodeKind::Dependency,
    });
    edges.push(make_edge(
        "zones-to-global-root",
        zone_ids,
        "layered-global-root",
        DependencyDomain::DataPipeline,
        AssertionState::Qualified,
    ));
    (
        build_input(
            source_ids,
            nodes,
            edges,
            DependencyDomain::DataPipeline,
            20,
            "layered",
        ),
        "SharedDependency",
    )
}

/// A source set shares different dependency hierarchies by dimension. Each
/// benchmark query selects one dimension; the same graph is queried three ways.
fn build_multi_domain_graph(
    source_count: usize,
    query_domain: DependencyDomain,
) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("multi-source-{index:08}"))
        .collect();
    let dimensions = [
        (DependencyDomain::Clock, "clock", "clock-global-root"),
        (DependencyDomain::Power, "power", "power-global-root"),
        (DependencyDomain::Network, "network", "network-global-root"),
    ];
    let group_size = 100usize;
    let group_count = source_count.div_ceil(group_size);
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::new();

    // All three dependency dimensions coexist in the same inventory. The
    // evaluator query changes only its selected dimension.
    for (domain, label, root_id) in dimensions {
        let mut group_ids = Vec::with_capacity(group_count);
        for group in 0..group_count {
            let group_id = format!("{label}-group-{group:05}");
            group_ids.push(group_id.clone());
            nodes.push(Node {
                id: group_id.clone(),
                kind: NodeKind::Dependency,
            });
            let start = group * group_size;
            let end = (start + group_size).min(source_count);
            edges.push(make_edge(
                &format!("{label}-sources-to-{group_id}"),
                source_ids[start..end].to_vec(),
                group_id,
                domain.clone(),
                AssertionState::Qualified,
            ));
        }
        nodes.push(Node {
            id: root_id.into(),
            kind: NodeKind::Dependency,
        });
        edges.push(make_edge(
            &format!("{label}-groups-to-root"),
            group_ids,
            root_id,
            domain,
            AssertionState::Qualified,
        ));
    }

    (
        build_input(
            source_ids,
            nodes,
            edges,
            query_domain,
            20,
            "multi-domain",
        ),
        "SharedDependency",
    )
}

/// Every source has its own dependency; a complete inventory must remain
/// NO_DECLARED_COMMON_DOMAIN rather than inventing an independence proof.
fn build_independent_graph(source_count: usize) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("independent-source-{index:08}"))
        .collect();
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    let mut edges = Vec::with_capacity(source_count);
    for source in &source_ids {
        let dependency_id = format!("{source}-private-dependency");
        nodes.push(Node {
            id: dependency_id.clone(),
            kind: NodeKind::Dependency,
        });
        edges.push(make_edge(
            &format!("{source}-edge"),
            vec![source.clone()],
            dependency_id,
            DependencyDomain::DataPipeline,
            AssertionState::Qualified,
        ));
    }
    (
        build_input(
            source_ids,
            nodes,
            edges,
            DependencyDomain::DataPipeline,
            20,
            "independent",
        ),
        "NoDeclaredCommonDomain",
    )
}

/// Unknown common-cause metadata must produce UNKNOWN_DEPENDENCY, never a
/// no-common-domain result.
fn build_unknown_graph(source_count: usize) -> (HypergraphInput, &'static str) {
    let source_ids: Vec<String> = (0..source_count)
        .map(|index| format!("unknown-source-{index:08}"))
        .collect();
    let mut nodes = source_ids
        .iter()
        .map(|id| Node {
            id: id.clone(),
            kind: NodeKind::Source,
        })
        .collect::<Vec<_>>();
    nodes.push(Node {
        id: "unknown-shared-root".into(),
        kind: NodeKind::Dependency,
    });
    let edge = make_edge(
        "unknown-shared-edge",
        source_ids.clone(),
        "unknown-shared-root",
        DependencyDomain::Clock,
        AssertionState::Unknown,
    );
    (
        build_input(source_ids, nodes, vec![edge], DependencyDomain::Clock, 20, "unknown"),
        "UnknownDependency",
    )
}

fn make_edge(
    id: &str,
    mut subject_ids: Vec<String>,
    dependency_id: impl Into<String>,
    domain: DependencyDomain,
    state: AssertionState,
) -> DependencyHyperedge {
    subject_ids.sort();
    subject_ids.dedup();
    DependencyHyperedge {
        id: id.to_owned(),
        subject_ids,
        dependency_id: dependency_id.into(),
        domain,
        valid_from: Some(0),
        valid_until: None,
        recorded_at: 80,
        evidence_ref: format!("synthetic:benchmark:{id}"),
        state,
    }
}

fn build_input(
    mut source_ids: Vec<String>,
    mut nodes: Vec<Node>,
    mut hyperedges: Vec<DependencyHyperedge>,
    domain: DependencyDomain,
    snapshot_revision: u64,
    label: &str,
) -> HypergraphInput {
    source_ids.sort();
    source_ids.dedup();
    nodes.sort_by(|left, right| left.id.cmp(&right.id));
    hyperedges.sort_by(|left, right| left.id.cmp(&right.id));
    let mut covered_node_ids: Vec<String> = nodes.iter().map(|node| node.id.clone()).collect();
    covered_node_ids.sort();
    covered_node_ids.dedup();
    let snapshot_id = format!("topology-{label}-{}-{snapshot_revision}", source_ids.len());
    HypergraphInput {
        contract: INPUT_CONTRACT.into(),
        snapshot: InventorySnapshot {
            snapshot_id: snapshot_id.clone(),
            as_of: 100,
            evidence_frontier: 120,
            complete: true,
            scoped_source_ids: source_ids.clone(),
            covered_node_ids,
        },
        query: DependencyQuery {
            source_ids,
            domain,
            event_time: 90,
            expected_snapshot_id: snapshot_id,
            expected_evidence_frontier: 120,
        },
        nodes,
        hyperedges,
    }
}

fn summarize(times: &mut [Duration]) -> TimingSummary {
    times.sort_unstable();
    let values: Vec<f64> = times
        .iter()
        .map(|time| time.as_secs_f64() * 1_000.0)
        .collect();
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
        let (input, expected) = build_star_graph(10);
        assert_eq!(expected, "SharedDependency");
        assert_eq!(input.nodes.len(), 11);
        assert_eq!(input.hyperedges.len(), 1);
        assert_eq!(input.hyperedges[0].subject_ids.len(), 10);
        assert!(
            input.hyperedges[0]
                .subject_ids
                .windows(2)
                .all(|pair| pair[0] < pair[1])
        );
        assert!(
            input
                .query
                .source_ids
                .windows(2)
                .all(|pair| pair[0] < pair[1])
        );
        assert_eq!(input.hyperedges[0].dependency_id, "shared-clock");
    }

    #[test]
    fn graph_topology_builders_are_deterministic_and_have_expected_dispositions() {
        let builders = [
            build_chain_graph(1_000, 128),
            build_layered_graph(1_000),
            build_independent_graph(1_000),
            build_unknown_graph(1_000),
        ];
        for (input, expected) in builders {
            let bytes = serde_json::to_vec(&input).unwrap();
            let repeated = serde_json::to_vec(&input).unwrap();
            assert_eq!(sha256_hex(&bytes), sha256_hex(&repeated));
            assert!(bytes.len() <= INPUT_LIMIT_BYTES);
            let report = evaluate_bytes(&bytes);
            assert_eq!(format!("{:?}", report.disposition), expected);
            let report_bytes = serde_json::to_vec(&report).unwrap();
            assert_eq!(
                independently_verify(&bytes, &report_bytes).status,
                "verified"
            );
        }
    }

    #[test]
    fn one_inventory_can_be_queried_by_three_overlapping_dependency_dimensions() {
        for (domain, expected_root) in [
            (DependencyDomain::Clock, "clock-global-root"),
            (DependencyDomain::Power, "power-global-root"),
            (DependencyDomain::Network, "network-global-root"),
        ] {
            let (input, expected) = build_multi_domain_graph(1_000, domain);
            assert_eq!(expected, "SharedDependency");
            assert!(input.hyperedges.iter().any(|edge| {
                edge.domain == DependencyDomain::Clock
            }));
            assert!(input.hyperedges.iter().any(|edge| {
                edge.domain == DependencyDomain::Power
            }));
            assert!(input.hyperedges.iter().any(|edge| {
                edge.domain == DependencyDomain::Network
            }));
            let bytes = serde_json::to_vec(&input).unwrap();
            assert!(bytes.len() <= INPUT_LIMIT_BYTES);
            let report = evaluate_bytes(&bytes);
            assert_eq!(
                report.disposition,
                emsr_dependency_hypergraph::Disposition::SharedDependency
            );
            assert_eq!(report.common_dependency_ids, vec![expected_root]);
            let report_bytes = serde_json::to_vec(&report).unwrap();
            assert_eq!(
                independently_verify(&bytes, &report_bytes).status,
                "verified"
            );
        }
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
        let (input, expected) = build_star_graph(1_000);
        assert_eq!(expected, "SharedDependency");
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
