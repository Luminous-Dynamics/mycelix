use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "dynamic-topology-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const EPS: f64 = 1e-10;
const CLAIM_CEILING: &str = "This undirected graph evaluator qualifies manufactured 3D network topology, source-to-sink reachability and single-element structural criticality only. It does not infer hydraulic activation from images, flow direction, flux, permeability, tracer transport, matrix exchange, crop response or field effects.";

#[derive(Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    truth_support_dimension: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    sources: Vec<String>,
    sinks: Vec<String>,
    snapshots: Vec<Snapshot>,
}

#[derive(Debug, Deserialize)]
struct Node {
    node_id: String,
    coordinates_m: [f64; 3],
}

#[derive(Debug, Deserialize)]
struct Edge {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
    length_m: f64,
    origin_class: String,
}

#[derive(Debug, Deserialize)]
struct Snapshot {
    snapshot_id: String,
    time_days: f64,
    edge_states: Vec<EdgeState>,
}

#[derive(Debug, Deserialize)]
struct EdgeState {
    edge_id: String,
    root_occupancy: String,
    hydraulic_state: String,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: &'static str,
    fixture_status: &'static str,
    scenario_id: String,
    input_sha256: String,
    node_count: usize,
    edge_count: usize,
    source_count: usize,
    sink_count: usize,
    snapshot_count: usize,
    snapshot_results: Vec<SnapshotResult>,
    controls: Controls,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct SnapshotResult {
    snapshot_id: String,
    time_days: f64,
    active_edge_count: usize,
    inactive_edge_count: usize,
    unassessed_edge_count: usize,
    confirmed_component_count: usize,
    possible_component_count: usize,
    confirmed_cycle_rank: usize,
    possible_cycle_rank: usize,
    confirmed_connected_pair_count: usize,
    possible_connected_pair_count: usize,
    pair_results: Vec<PairResult>,
    critical_edge_ids: Vec<String>,
    critical_interior_node_ids: Vec<String>,
    worst_case_single_edge_failure_connected_pair_count: usize,
}

#[derive(Debug, Serialize)]
struct PairResult {
    source_node_id: String,
    sink_node_id: String,
    confirmed_connected: bool,
    possibly_connected: bool,
    status: String,
}

#[derive(Debug, Serialize)]
struct Controls {
    volumetric_3d_truth_support_required: bool,
    shared_branching_and_rejoining_graph_supported: bool,
    origin_separate_from_root_occupancy_and_hydraulic_state: bool,
    unknown_edges_excluded_from_confirmed_graph: bool,
    unknown_edges_included_only_in_possible_graph: bool,
    single_edge_and_interior_node_failure_audited: bool,
    no_flow_or_field_claim: bool,
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn is_root_origin(origin: &str) -> bool {
    matches!(origin, "live_root_channel" | "legacy_root_channel")
}

fn validate(s: &Scenario) -> Result<(), String> {
    if s.schema_version != "dynamic-topology-v0.1"
        || s.fixture_status != FIXTURE_STATUS
        || s.scenario_id.trim().is_empty()
        || s.truth_support_dimension != "volumetric_3d"
    {
        return Err("unsupported schema, non-synthetic fixture status, empty scenario ID or non-3D truth support".into());
    }
    if s.nodes.len() < 2 || s.edges.is_empty() || s.sources.is_empty() || s.sinks.is_empty() || s.snapshots.is_empty() {
        return Err("scenario requires nodes, edges, source and sink terminals, and snapshots".into());
    }

    let mut nodes = BTreeMap::new();
    for node in &s.nodes {
        if node.node_id.trim().is_empty() || nodes.contains_key(&node.node_id) {
            return Err("node IDs must be non-empty and unique".into());
        }
        for coordinate in node.coordinates_m {
            finite("node coordinate", coordinate)?;
        }
        nodes.insert(node.node_id.as_str(), node.coordinates_m);
    }

    let mut edges = BTreeSet::new();
    for edge in &s.edges {
        if edge.edge_id.trim().is_empty()
            || !edges.insert(edge.edge_id.as_str())
            || edge.from_node_id == edge.to_node_id
            || !nodes.contains_key(edge.from_node_id.as_str())
            || !nodes.contains_key(edge.to_node_id.as_str())
        {
            return Err("edge IDs must be unique and endpoints must be distinct declared nodes".into());
        }
        finite("edge length", edge.length_m)?;
        if edge.length_m <= 0.0 {
            return Err("edge length must be positive".into());
        }
        let chord = distance(
            *nodes.get(edge.from_node_id.as_str()).expect("endpoint validated"),
            *nodes.get(edge.to_node_id.as_str()).expect("endpoint validated"),
        );
        if edge.length_m + EPS < chord {
            return Err(format!("edge {} is shorter than its 3D endpoint chord", edge.edge_id));
        }
        if !matches!(
            edge.origin_class.as_str(),
            "live_root_channel" | "legacy_root_channel" | "abiotic_crack" | "textural_finger" | "faunal_burrow"
        ) {
            return Err(format!("edge {} has unsupported origin class", edge.edge_id));
        }
    }

    let mut sources = BTreeSet::new();
    for source in &s.sources {
        if !nodes.contains_key(source.as_str()) || !sources.insert(source.as_str()) {
            return Err("source terminals must be unique declared nodes".into());
        }
    }
    let mut sinks = BTreeSet::new();
    for sink in &s.sinks {
        if !nodes.contains_key(sink.as_str()) || !sinks.insert(sink.as_str()) {
            return Err("sink terminals must be unique declared nodes".into());
        }
        if sources.contains(sink.as_str()) {
            return Err("source and sink terminal sets must be disjoint".into());
        }
    }

    let mut snapshot_ids = BTreeSet::new();
    let mut previous_time = None;
    for snapshot in &s.snapshots {
        if snapshot.snapshot_id.trim().is_empty() || !snapshot_ids.insert(snapshot.snapshot_id.as_str()) {
            return Err("snapshot IDs must be non-empty and unique".into());
        }
        finite("snapshot time", snapshot.time_days)?;
        if snapshot.time_days < 0.0 || previous_time.is_some_and(|time| snapshot.time_days <= time) {
            return Err("snapshot times must be non-negative and strictly increasing".into());
        }
        previous_time = Some(snapshot.time_days);

        let mut states = BTreeSet::new();
        for state in &snapshot.edge_states {
            if !edges.contains(state.edge_id.as_str()) || !states.insert(state.edge_id.as_str()) {
                return Err("snapshot has unknown or duplicate edge state".into());
            }
            let edge = s.edges.iter().find(|edge| edge.edge_id == state.edge_id)
                .expect("edge state identity validated");
            if is_root_origin(&edge.origin_class) {
                if !matches!(
                    state.root_occupancy.as_str(),
                    "occupied" | "partially_occupied" | "unoccupied_legacy" | "not_observed"
                ) {
                    return Err(format!("root-origin edge {} requires an explicit root occupancy state", edge.edge_id));
                }
            } else if state.root_occupancy != "not_applicable" {
                return Err(format!("non-root edge {} must use not_applicable root occupancy", edge.edge_id));
            }
            if !matches!(state.hydraulic_state.as_str(), "active" | "inactive" | "unassessed") {
                return Err("hydraulic state must be active, inactive or unassessed".into());
            }
        }
        if states != edges {
            return Err("every snapshot must contain exactly one state for every edge".into());
        }
    }
    Ok(())
}

fn included_edges(snapshot: &Snapshot, include_unassessed: bool) -> BTreeSet<String> {
    snapshot.edge_states.iter()
        .filter(|state| state.hydraulic_state == "active"
            || (include_unassessed && state.hydraulic_state == "unassessed"))
        .map(|state| state.edge_id.clone())
        .collect()
}

fn adjacency(
    s: &Scenario,
    included: &BTreeSet<String>,
    removed_edge: Option<&str>,
    removed_node: Option<&str>,
) -> BTreeMap<String, Vec<String>> {
    let mut graph: BTreeMap<String, Vec<String>> = s.nodes.iter()
        .filter(|node| removed_node != Some(node.node_id.as_str()))
        .map(|node| (node.node_id.clone(), Vec::new()))
        .collect();

    for edge in &s.edges {
        if !included.contains(&edge.edge_id) || removed_edge == Some(edge.edge_id.as_str()) {
            continue;
        }
        if removed_node == Some(edge.from_node_id.as_str())
            || removed_node == Some(edge.to_node_id.as_str())
        {
            continue;
        }
        if let Some(neighbors) = graph.get_mut(&edge.from_node_id) {
            neighbors.push(edge.to_node_id.clone());
        }
        if let Some(neighbors) = graph.get_mut(&edge.to_node_id) {
            neighbors.push(edge.from_node_id.clone());
        }
    }
    for neighbors in graph.values_mut() {
        neighbors.sort();
        neighbors.dedup();
    }
    graph
}

fn reachable(graph: &BTreeMap<String, Vec<String>>, source: &str, sink: &str) -> bool {
    if !graph.contains_key(source) || !graph.contains_key(sink) {
        return false;
    }
    let mut visited = BTreeSet::new();
    let mut stack = vec![source.to_string()];
    while let Some(node) = stack.pop() {
        if node == sink {
            return true;
        }
        if !visited.insert(node.clone()) {
            continue;
        }
        if let Some(neighbors) = graph.get(&node) {
            for neighbor in neighbors.iter().rev() {
                if !visited.contains(neighbor) {
                    stack.push(neighbor.clone());
                }
            }
        }
    }
    false
}

fn connected_pair_count(
    s: &Scenario,
    included: &BTreeSet<String>,
    removed_edge: Option<&str>,
    removed_node: Option<&str>,
) -> usize {
    let graph = adjacency(s, included, removed_edge, removed_node);
    s.sources.iter().map(|source| {
        s.sinks.iter().filter(|sink| reachable(&graph, source, sink)).count()
    }).sum()
}

fn component_count(s: &Scenario, included: &BTreeSet<String>) -> usize {
    let graph = adjacency(s, included, None, None);
    let mut visited = BTreeSet::new();
    let mut components = 0;
    for start in graph.keys() {
        if visited.contains(start) {
            continue;
        }
        components += 1;
        let mut stack = vec![start.clone()];
        while let Some(node) = stack.pop() {
            if !visited.insert(node.clone()) {
                continue;
            }
            if let Some(neighbors) = graph.get(&node) {
                for neighbor in neighbors {
                    if !visited.contains(neighbor) {
                        stack.push(neighbor.clone());
                    }
                }
            }
        }
    }
    components
}

fn cycle_rank(s: &Scenario, included: &BTreeSet<String>) -> usize {
    let components = component_count(s, included);
    included.len() + components - s.nodes.len()
}

fn pair_results(s: &Scenario, active: &BTreeSet<String>, possible: &BTreeSet<String>) -> Vec<PairResult> {
    let active_graph = adjacency(s, active, None, None);
    let possible_graph = adjacency(s, possible, None, None);
    let mut pairs = Vec::new();
    for source in &s.sources {
        for sink in &s.sinks {
            let confirmed = reachable(&active_graph, source, sink);
            let possible = reachable(&possible_graph, source, sink);
            let status = if confirmed {
                "confirmed_connected"
            } else if possible {
                "indeterminate_unassessed_edge"
            } else {
                "confirmed_disconnected"
            };
            pairs.push(PairResult {
                source_node_id: source.clone(),
                sink_node_id: sink.clone(),
                confirmed_connected: confirmed,
                possibly_connected: possible,
                status: status.into(),
            });
        }
    }
    pairs
}

fn critical_edges(
    s: &Scenario,
    active: &BTreeSet<String>,
    baseline_pairs: usize,
) -> (Vec<String>, usize) {
    let mut critical = Vec::new();
    let mut worst_case = baseline_pairs;
    for edge_id in active {
        let remaining = connected_pair_count(s, active, Some(edge_id), None);
        worst_case = worst_case.min(remaining);
        if remaining < baseline_pairs {
            critical.push(edge_id.clone());
        }
    }
    (critical, worst_case)
}

fn critical_interior_nodes(
    s: &Scenario,
    active: &BTreeSet<String>,
    baseline_pairs: usize,
) -> Vec<String> {
    let terminals: BTreeSet<&str> = s.sources.iter().chain(s.sinks.iter()).map(String::as_str).collect();
    let mut critical = Vec::new();
    for node in &s.nodes {
        if terminals.contains(node.node_id.as_str()) {
            continue;
        }
        let remaining = connected_pair_count(s, active, None, Some(&node.node_id));
        if remaining < baseline_pairs {
            critical.push(node.node_id.clone());
        }
    }
    critical
}

fn build_report(s: &Scenario, input_sha256: String) -> Report {
    let mut snapshot_results = Vec::with_capacity(s.snapshots.len());
    for snapshot in &s.snapshots {
        let active = included_edges(snapshot, false);
        let possible = included_edges(snapshot, true);
        let inactive_count = snapshot.edge_states.iter()
            .filter(|state| state.hydraulic_state == "inactive").count();
        let unassessed_count = snapshot.edge_states.iter()
            .filter(|state| state.hydraulic_state == "unassessed").count();
        let confirmed_pairs = connected_pair_count(s, &active, None, None);
        let possible_pairs = connected_pair_count(s, &possible, None, None);
        let (critical_edge_ids, worst_case) = critical_edges(s, &active, confirmed_pairs);

        snapshot_results.push(SnapshotResult {
            snapshot_id: snapshot.snapshot_id.clone(),
            time_days: snapshot.time_days,
            active_edge_count: active.len(),
            inactive_edge_count: inactive_count,
            unassessed_edge_count: unassessed_count,
            confirmed_component_count: component_count(s, &active),
            possible_component_count: component_count(s, &possible),
            confirmed_cycle_rank: cycle_rank(s, &active),
            possible_cycle_rank: cycle_rank(s, &possible),
            confirmed_connected_pair_count: confirmed_pairs,
            possible_connected_pair_count: possible_pairs,
            pair_results: pair_results(s, &active, &possible),
            critical_edge_ids,
            critical_interior_node_ids: critical_interior_nodes(s, &active, confirmed_pairs),
            worst_case_single_edge_failure_connected_pair_count: worst_case,
        });
    }

    Report {
        audit_version: VERSION,
        fixture_status: FIXTURE_STATUS,
        scenario_id: s.scenario_id.clone(),
        input_sha256,
        node_count: s.nodes.len(),
        edge_count: s.edges.len(),
        source_count: s.sources.len(),
        sink_count: s.sinks.len(),
        snapshot_count: s.snapshots.len(),
        snapshot_results,
        controls: Controls {
            volumetric_3d_truth_support_required: true,
            shared_branching_and_rejoining_graph_supported: true,
            origin_separate_from_root_occupancy_and_hydraulic_state: true,
            unknown_edges_excluded_from_confirmed_graph: true,
            unknown_edges_included_only_in_possible_graph: true,
            single_edge_and_interior_node_failure_audited: true,
            no_flow_or_field_claim: true,
        },
        interpretation_boundary: CLAIM_CEILING,
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
        eprintln!("usage: dynamic-topology-reference <scenario.json> <new-report.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("dynamic topology reference failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-dynamic-topology.json"))
            .expect("synthetic fixture parses")
    }

    #[test]
    fn shared_branches_are_not_misrepresented_as_disjoint_paths() {
        let s = scenario();
        validate(&s).expect("fixture valid");
        let report = build_report(&s, "0".repeat(64));
        let first = &report.snapshot_results[0];
        assert_eq!(first.confirmed_connected_pair_count, 1);
        assert_eq!(first.confirmed_cycle_rank, 1);
        assert_eq!(first.critical_edge_ids, vec!["inlet-trunk".to_string(), "outlet-trunk".to_string()]);
        assert_eq!(first.critical_interior_node_ids, vec!["junction-in".to_string(), "junction-out".to_string()]);
        assert_eq!(first.worst_case_single_edge_failure_connected_pair_count, 0);
    }

    #[test]
    fn losing_one_redundant_branch_preserves_terminal_connectivity() {
        let report = build_report(&scenario(), "0".repeat(64));
        let second = &report.snapshot_results[1];
        assert_eq!(second.pair_results[0].status, "confirmed_connected");
        assert_eq!(second.confirmed_connected_pair_count, 1);
    }

    #[test]
    fn unassessed_branch_is_possible_but_not_confirmed() {
        let report = build_report(&scenario(), "0".repeat(64));
        let third = &report.snapshot_results[2];
        assert_eq!(third.confirmed_connected_pair_count, 0);
        assert_eq!(third.possible_connected_pair_count, 1);
        assert_eq!(third.pair_results[0].status, "indeterminate_unassessed_edge");
        assert_eq!(third.unassessed_edge_count, 1);
    }

    #[test]
    fn fully_inactive_branches_are_confirmed_disconnected() {
        let report = build_report(&scenario(), "0".repeat(64));
        let fourth = &report.snapshot_results[3];
        assert_eq!(fourth.confirmed_connected_pair_count, 0);
        assert_eq!(fourth.possible_connected_pair_count, 0);
        assert_eq!(fourth.pair_results[0].status, "confirmed_disconnected");
    }

    #[test]
    fn legacy_root_origin_is_not_reclassified_as_abiotic() {
        let s = scenario();
        validate(&s).expect("legacy root provenance and occupancy are valid");
        let legacy = s.edges.iter().find(|edge| edge.edge_id == "lower-a").unwrap();
        assert_eq!(legacy.origin_class, "legacy_root_channel");
        assert_eq!(
            s.snapshots[0].edge_states.iter().find(|state| state.edge_id == "lower-a").unwrap().root_occupancy,
            "unoccupied_legacy"
        );
    }

    #[test]
    fn invalid_edge_chord_is_rejected() {
        let mut s = scenario();
        s.edges[0].length_m = 0.1;
        assert!(validate(&s).is_err());
    }

    #[test]
    fn non_3d_truth_support_is_rejected() {
        let mut s = scenario();
        s.truth_support_dimension = "planar_2d".into();
        assert!(validate(&s).is_err());
    }

    #[test]
    fn missing_edge_state_is_rejected() {
        let mut s = scenario();
        s.snapshots[0].edge_states.pop();
        assert!(validate(&s).is_err());
    }
}
