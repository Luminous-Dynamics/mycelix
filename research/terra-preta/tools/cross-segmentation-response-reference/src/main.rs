use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "terra-cross-segmentation-response-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const TOL: f64 = 1.0e-10;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Node {
    node_id: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Edge {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
    conductance: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Variant {
    variant_id: String,
    present_node_ids: Vec<String>,
    present_edge_ids: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    source_node_id: String,
    sink_node_id: String,
    upper_probe_node_id: String,
    lower_probe_node_id: String,
    localized_path_measurement_edge_id: String,
    localized_path_edge_ids: Vec<String>,
    variants: Vec<Variant>,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    conductance_units: String,
    pressure_units: String,
    boundary_condition: String,
    variant_results: Vec<VariantResult>,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct VariantResult {
    variant_id: String,
    present_node_count: usize,
    present_edge_count: usize,
    connected_component_count: usize,
    source_sink_connected: bool,
    effective_conductance: f64,
    boundary_response_matrix: [[f64; 2]; 2],
    total_source_flow_at_unit_pressure_drop: f64,
    upper_pressure_probe: Option<f64>,
    lower_pressure_probe: Option<f64>,
    localized_path_observation: LocalizedPathObservation,
    boundary_flux_balance_abs: f64,
    response_symmetry_max_abs: f64,
    response_row_sum_max_abs: f64,
}

#[derive(Debug, Serialize)]
struct LocalizedPathObservation {
    measurement_edge_id: String,
    required_path_edge_ids: Vec<String>,
    status: String,
    local_measurement_edge_flux: Option<f64>,
    complete_path_flow: Option<f64>,
    fraction_of_total_flow: Option<f64>,
    interpretation: String,
}

struct AssembledGraph {
    node_ids: Vec<String>,
    node_index: BTreeMap<String, usize>,
    laplacian: Vec<Vec<f64>>,
    component_count: usize,
    source_sink_connected: bool,
}

struct BoundaryResponseSolution {
    response: [[f64; 2]; 2],
    potentials: Vec<f64>,
    boundary_flux: Vec<f64>,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture schema, synthetic-only status or scenario identity is invalid".into());
    }
    if fixture.nodes.len() < 2 || fixture.edges.is_empty() || fixture.variants.len() < 2 {
        return Err("fixture requires at least two nodes, one edge and two variants".into());
    }

    let mut node_ids = BTreeSet::new();
    for node in &fixture.nodes {
        if node.node_id.trim().is_empty() || !node_ids.insert(node.node_id.as_str()) {
            return Err("node IDs must be non-empty and unique".into());
        }
    }
    for node_id in [
        &fixture.source_node_id,
        &fixture.sink_node_id,
        &fixture.upper_probe_node_id,
        &fixture.lower_probe_node_id,
    ] {
        if !node_ids.contains(node_id.as_str()) {
            return Err(format!("unknown boundary or probe node ID: {node_id}"));
        }
    }
    if fixture.source_node_id == fixture.sink_node_id {
        return Err("source and sink nodes must differ".into());
    }

    let mut edge_ids = BTreeSet::new();
    for edge in &fixture.edges {
        if edge.edge_id.trim().is_empty() || !edge_ids.insert(edge.edge_id.as_str()) {
            return Err("edge IDs must be non-empty and unique".into());
        }
        if edge.from_node_id == edge.to_node_id
            || !node_ids.contains(edge.from_node_id.as_str())
            || !node_ids.contains(edge.to_node_id.as_str())
        {
            return Err(format!(
                "edge {} has a self-loop or unknown endpoint",
                edge.edge_id
            ));
        }
        if !edge.conductance.is_finite() || edge.conductance <= 0.0 {
            return Err(format!(
                "edge {} conductance must be finite and positive",
                edge.edge_id
            ));
        }
    }

    if fixture.localized_path_edge_ids.len() < 2 {
        return Err("localized path must declare at least two ordered edges".into());
    }
    let path_ids: BTreeSet<&str> = fixture
        .localized_path_edge_ids
        .iter()
        .map(String::as_str)
        .collect();
    if path_ids.len() != fixture.localized_path_edge_ids.len()
        || !path_ids.contains(fixture.localized_path_measurement_edge_id.as_str())
    {
        return Err("localized path IDs must be unique and contain the measurement edge".into());
    }
    let edge_by_id: BTreeMap<&str, &Edge> = fixture
        .edges
        .iter()
        .map(|edge| (edge.edge_id.as_str(), edge))
        .collect();
    for edge_id in &fixture.localized_path_edge_ids {
        if !edge_by_id.contains_key(edge_id.as_str()) {
            return Err(format!("localized path references unknown edge {edge_id}"));
        }
    }
    for pair in fixture.localized_path_edge_ids.windows(2) {
        let left = edge_by_id[pair[0].as_str()];
        let right = edge_by_id[pair[1].as_str()];
        let contiguous = left.from_node_id == right.from_node_id
            || left.from_node_id == right.to_node_id
            || left.to_node_id == right.from_node_id
            || left.to_node_id == right.to_node_id;
        if !contiguous {
            return Err("ordered localized path edges are not contiguous".into());
        }
    }

    let mut variant_ids = BTreeSet::new();
    for variant in &fixture.variants {
        if variant.variant_id.trim().is_empty() || !variant_ids.insert(variant.variant_id.as_str())
        {
            return Err("variant IDs must be non-empty and unique".into());
        }
        let present_nodes: BTreeSet<&str> = variant
            .present_node_ids
            .iter()
            .map(String::as_str)
            .collect();
        let present_edges: BTreeSet<&str> = variant
            .present_edge_ids
            .iter()
            .map(String::as_str)
            .collect();
        if present_nodes.len() != variant.present_node_ids.len()
            || present_edges.len() != variant.present_edge_ids.len()
            || present_nodes.len() < 2
        {
            return Err(format!(
                "variant {} has duplicate or insufficient features",
                variant.variant_id
            ));
        }
        if !present_nodes.contains(fixture.source_node_id.as_str())
            || !present_nodes.contains(fixture.sink_node_id.as_str())
        {
            return Err(format!(
                "variant {} omits a required boundary node",
                variant.variant_id
            ));
        }
        if present_nodes.iter().any(|id| !node_ids.contains(*id))
            || present_edges.iter().any(|id| !edge_by_id.contains_key(*id))
        {
            return Err(format!(
                "variant {} references an unknown feature",
                variant.variant_id
            ));
        }
        for edge_id in &variant.present_edge_ids {
            let edge = edge_by_id[edge_id.as_str()];
            if !present_nodes.contains(edge.from_node_id.as_str())
                || !present_nodes.contains(edge.to_node_id.as_str())
            {
                return Err(format!(
                    "variant {} includes edge {} without both endpoints",
                    variant.variant_id, edge_id
                ));
            }
        }
        assemble_graph(fixture, variant)?;
    }
    Ok(())
}

#[allow(clippy::needless_range_loop)]
fn assemble_graph(fixture: &Fixture, variant: &Variant) -> Result<AssembledGraph, String> {
    let present: BTreeSet<&str> = variant
        .present_node_ids
        .iter()
        .map(String::as_str)
        .collect();
    let node_ids: Vec<String> = fixture
        .nodes
        .iter()
        .filter(|node| present.contains(node.node_id.as_str()))
        .map(|node| node.node_id.clone())
        .collect();
    let node_index: BTreeMap<String, usize> = node_ids
        .iter()
        .enumerate()
        .map(|(index, id)| (id.clone(), index))
        .collect();
    let n = node_ids.len();
    let mut laplacian = vec![vec![0.0; n]; n];
    let mut adjacency = vec![Vec::<usize>::new(); n];
    let edge_by_id: BTreeMap<&str, &Edge> = fixture
        .edges
        .iter()
        .map(|edge| (edge.edge_id.as_str(), edge))
        .collect();

    for edge_id in &variant.present_edge_ids {
        let edge = edge_by_id[edge_id.as_str()];
        let from = node_index[&edge.from_node_id];
        let to = node_index[&edge.to_node_id];
        let g = edge.conductance;
        laplacian[from][from] += g;
        laplacian[to][to] += g;
        laplacian[from][to] -= g;
        laplacian[to][from] -= g;
        adjacency[from].push(to);
        adjacency[to].push(from);
    }

    let source = node_index[&fixture.source_node_id];
    let sink = node_index[&fixture.sink_node_id];
    let mut visited = vec![false; n];
    let mut component_count = 0usize;
    let mut source_sink_connected = false;
    for start in 0..n {
        if visited[start] {
            continue;
        }
        component_count += 1;
        let mut queue = VecDeque::from([start]);
        let mut contains_boundary = false;
        let mut contains_source = false;
        let mut contains_sink = false;
        visited[start] = true;
        while let Some(node) = queue.pop_front() {
            contains_boundary |= node == source || node == sink;
            contains_source |= node == source;
            contains_sink |= node == sink;
            for &neighbour in &adjacency[node] {
                if !visited[neighbour] {
                    visited[neighbour] = true;
                    queue.push_back(neighbour);
                }
            }
        }
        if !contains_boundary {
            return Err(format!(
                "variant {} contains an unanchored interior component",
                variant.variant_id
            ));
        }
        source_sink_connected |= contains_source && contains_sink;
    }
    Ok(AssembledGraph {
        node_ids,
        node_index,
        laplacian,
        component_count,
        source_sink_connected,
    })
}

#[allow(clippy::needless_range_loop)]
fn solve_linear(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = matrix.len();
    if rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("linear-system dimensions are inconsistent".into());
    }
    if n == 0 {
        return Ok(Vec::new());
    }
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if !scale.is_finite() || scale == 0.0 {
        return Err("linear system is singular or non-finite".into());
    }
    let pivot_floor = scale * 1.0e-13;
    for column in 0..n {
        let pivot_row = (column..n)
            .max_by(|&left, &right| {
                matrix[left][column]
                    .abs()
                    .total_cmp(&matrix[right][column].abs())
            })
            .ok_or_else(|| "linear system has no pivot candidate".to_string())?;
        if matrix[pivot_row][column].abs() <= pivot_floor {
            return Err("interior Laplacian is singular or numerically unresolved".into());
        }
        matrix.swap(column, pivot_row);
        rhs.swap(column, pivot_row);
        let pivot = matrix[column][column];
        for j in column..n {
            matrix[column][j] /= pivot;
        }
        rhs[column] /= pivot;
        for row in 0..n {
            if row == column {
                continue;
            }
            let factor = matrix[row][column];
            for j in column..n {
                matrix[row][j] -= factor * matrix[column][j];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    if rhs.iter().any(|value| !value.is_finite()) {
        return Err("linear solve produced a non-finite state".into());
    }
    Ok(rhs)
}

#[allow(clippy::needless_range_loop)]
fn solve_state(
    graph: &AssembledGraph,
    source_id: &str,
    sink_id: &str,
    source_pressure: f64,
    sink_pressure: f64,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    let source = graph.node_index[source_id];
    let sink = graph.node_index[sink_id];
    let interior: Vec<usize> = (0..graph.node_ids.len())
        .filter(|index| *index != source && *index != sink)
        .collect();
    let mut potentials = vec![0.0; graph.node_ids.len()];
    potentials[source] = source_pressure;
    potentials[sink] = sink_pressure;
    let mut interior_matrix = vec![vec![0.0; interior.len()]; interior.len()];
    let mut rhs = vec![0.0; interior.len()];
    for (row, &node) in interior.iter().enumerate() {
        rhs[row] = -(graph.laplacian[node][source] * source_pressure
            + graph.laplacian[node][sink] * sink_pressure);
        for (column, &other) in interior.iter().enumerate() {
            interior_matrix[row][column] = graph.laplacian[node][other];
        }
    }
    let solution = solve_linear(interior_matrix, rhs)?;
    for (index, &node) in interior.iter().enumerate() {
        potentials[node] = solution[index];
    }
    let mut flux = vec![0.0; graph.node_ids.len()];
    for (row, value) in flux.iter_mut().enumerate() {
        *value = graph.laplacian[row]
            .iter()
            .zip(potentials.iter())
            .map(|(coefficient, pressure)| coefficient * pressure)
            .sum();
    }
    Ok((potentials, flux))
}

fn solve_boundary_response(
    graph: &AssembledGraph,
    source_id: &str,
    sink_id: &str,
) -> Result<BoundaryResponseSolution, String> {
    let (source_state, source_flux) = solve_state(graph, source_id, sink_id, 1.0, 0.0)?;
    let (_, sink_flux) = solve_state(graph, source_id, sink_id, 0.0, 1.0)?;
    let source = graph.node_index[source_id];
    let sink = graph.node_index[sink_id];
    let response = [
        [source_flux[source], sink_flux[source]],
        [source_flux[sink], sink_flux[sink]],
    ];
    Ok(BoundaryResponseSolution {
        response,
        potentials: source_state,
        boundary_flux: source_flux,
    })
}

fn analyze_variant(fixture: &Fixture, variant: &Variant) -> Result<VariantResult, String> {
    let graph = assemble_graph(fixture, variant)?;
    let BoundaryResponseSolution {
        response,
        potentials,
        boundary_flux,
    } = solve_boundary_response(&graph, &fixture.source_node_id, &fixture.sink_node_id)?;
    let source = graph.node_index[&fixture.source_node_id];
    let sink = graph.node_index[&fixture.sink_node_id];
    let total_flow = boundary_flux[source];
    let effective_conductance = -response[0][1];
    if !effective_conductance.is_finite() || effective_conductance < -TOL {
        return Err(format!(
            "variant {} produced invalid effective conductance",
            variant.variant_id
        ));
    }

    let upper_pressure_probe = graph
        .node_index
        .get(&fixture.upper_probe_node_id)
        .map(|index| potentials[*index]);
    let lower_pressure_probe = graph
        .node_index
        .get(&fixture.lower_probe_node_id)
        .map(|index| potentials[*index]);
    let present_edges: BTreeSet<&str> = variant
        .present_edge_ids
        .iter()
        .map(String::as_str)
        .collect();
    let edge_by_id: BTreeMap<&str, &Edge> = fixture
        .edges
        .iter()
        .map(|edge| (edge.edge_id.as_str(), edge))
        .collect();
    let measurement_edge_id = &fixture.localized_path_measurement_edge_id;
    let measurement_edge = edge_by_id[measurement_edge_id.as_str()];
    let local_edge_flux = if present_edges.contains(measurement_edge_id.as_str()) {
        let from = graph.node_index[&measurement_edge.from_node_id];
        let to = graph.node_index[&measurement_edge.to_node_id];
        Some((measurement_edge.conductance * (potentials[from] - potentials[to])).abs())
    } else {
        None
    };
    let path_is_complete = fixture
        .localized_path_edge_ids
        .iter()
        .all(|edge_id| present_edges.contains(edge_id.as_str()));
    let complete_path_flow = if path_is_complete && graph.source_sink_connected {
        local_edge_flux
    } else {
        None
    };
    let fraction = complete_path_flow.and_then(|flow| {
        if total_flow > TOL {
            Some(flow / total_flow)
        } else {
            None
        }
    });
    let status = if !path_is_complete {
        "path_not_fully_resolved"
    } else if !graph.source_sink_connected {
        "no_source_sink_through_path"
    } else {
        "support_resolved"
    };
    let interpretation = match status {
        "support_resolved" => {
            "The declared path and measurement edge are present; the flow estimate refers only to this manufactured graph and boundary condition."
        }
        "path_not_fully_resolved" => {
            "A required path edge is absent. A local edge reading is not promoted to a complete-path flow estimate."
        }
        _ => {
            "The source and sink are disconnected. A through-flow fraction is undefined because total flow is zero."
        }
    };

    let boundary_flux_balance_abs = (boundary_flux[source] + boundary_flux[sink]).abs();
    let response_symmetry_max_abs = (response[0][1] - response[1][0]).abs();
    let response_row_sum_max_abs = response
        .iter()
        .map(|row| (row[0] + row[1]).abs())
        .fold(0.0_f64, f64::max);

    Ok(VariantResult {
        variant_id: variant.variant_id.clone(),
        present_node_count: variant.present_node_ids.len(),
        present_edge_count: variant.present_edge_ids.len(),
        connected_component_count: graph.component_count,
        source_sink_connected: graph.source_sink_connected,
        effective_conductance,
        boundary_response_matrix: response,
        total_source_flow_at_unit_pressure_drop: total_flow,
        upper_pressure_probe,
        lower_pressure_probe,
        localized_path_observation: LocalizedPathObservation {
            measurement_edge_id: measurement_edge_id.clone(),
            required_path_edge_ids: fixture.localized_path_edge_ids.clone(),
            status: status.into(),
            local_measurement_edge_flux: local_edge_flux,
            complete_path_flow,
            fraction_of_total_flow: fraction,
            interpretation: interpretation.into(),
        },
        boundary_flux_balance_abs,
        response_symmetry_max_abs,
        response_row_sum_max_abs,
    })
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let variant_results = fixture
        .variants
        .iter()
        .map(|variant| analyze_variant(fixture, variant))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        conductance_units: "dimensionless manufactured conductance".into(),
        pressure_units: "dimensionless manufactured pressure".into(),
        boundary_condition: "source pressure=1, sink pressure=0; linear steady graph; unit pressure drop".into(),
        variant_results,
        interpretation_boundary: "Synthetic topology-response reference only. Unit conductances isolate the effect of supplied segmentation topology. Variant counts are not probabilities. This is not a CT segmentation engine, nonlinear Richards solver, field validation, or evidence of hydraulic activation in real soil. A local edge flux is not promoted to a complete-path observation when required path support is missing.".into(),
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} INPUT.json OUTPUT.json",
            args.first()
                .map(String::as_str)
                .unwrap_or("terra-cross-segmentation-response-reference")
        ));
    }
    let input = fs::read(&args[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let digest = format!("{:x}", Sha256::digest(&input));
    let fixture: Fixture =
        serde_json::from_slice(&input).map_err(|error| format!("invalid fixture JSON: {error}"))?;
    let report = analyze(&fixture, digest)?;
    let bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report: {error}"))?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output
        .write_all(&bytes)
        .map_err(|error| format!("cannot write report: {error}"))?;
    output
        .write_all(b"\n")
        .map_err(|error| format!("cannot finalize report: {error}"))?;
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

    fn fixture() -> Fixture {
        let nodes = [
            "source",
            "junction-in",
            "upper",
            "lower",
            "junction-out",
            "sink",
        ];
        let edges = [
            "inlet", "upper-a", "upper-b", "lower-a", "lower-b", "outlet",
        ];
        let make_variant = |variant_id: &str, present_edges: &[&str]| Variant {
            variant_id: variant_id.into(),
            present_node_ids: nodes.iter().map(|id| (*id).into()).collect(),
            present_edge_ids: present_edges.iter().map(|id| (*id).into()).collect(),
        };
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "percentile-segmentation-two-branch-network".into(),
            nodes: nodes
                .iter()
                .map(|id| Node {
                    node_id: (*id).into(),
                })
                .collect(),
            edges: edges
                .iter()
                .map(|id| {
                    let (from, to) = match *id {
                        "inlet" => ("source", "junction-in"),
                        "upper-a" => ("junction-in", "upper"),
                        "upper-b" => ("upper", "junction-out"),
                        "lower-a" => ("junction-in", "lower"),
                        "lower-b" => ("lower", "junction-out"),
                        "outlet" => ("junction-out", "sink"),
                        _ => unreachable!(),
                    };
                    Edge {
                        edge_id: (*id).into(),
                        from_node_id: from.into(),
                        to_node_id: to.into(),
                        conductance: 1.0,
                    }
                })
                .collect(),
            source_node_id: "source".into(),
            sink_node_id: "sink".into(),
            upper_probe_node_id: "upper".into(),
            lower_probe_node_id: "lower".into(),
            localized_path_measurement_edge_id: "upper-a".into(),
            localized_path_edge_ids: vec!["upper-a".into(), "upper-b".into()],
            variants: vec![
                make_variant(
                    "alpha-10",
                    &[
                        "inlet", "upper-a", "upper-b", "lower-a", "lower-b", "outlet",
                    ],
                ),
                make_variant(
                    "alpha-30",
                    &[
                        "inlet", "upper-a", "upper-b", "lower-a", "lower-b", "outlet",
                    ],
                ),
                make_variant(
                    "alpha-50",
                    &[
                        "inlet", "upper-a", "upper-b", "lower-a", "lower-b", "outlet",
                    ],
                ),
                make_variant(
                    "alpha-70",
                    &["inlet", "upper-a", "lower-a", "lower-b", "outlet"],
                ),
                make_variant("alpha-90", &["inlet", "upper-a", "lower-a", "outlet"]),
            ],
        }
    }

    fn result_for<'a>(report: &'a Report, id: &str) -> &'a VariantResult {
        report
            .variant_results
            .iter()
            .find(|result| result.variant_id == id)
            .unwrap()
    }

    #[test]
    fn alpha10_30_50_preserve_the_two_path_response() {
        let report = analyze(&fixture(), "test-digest".into()).unwrap();
        for id in ["alpha-10", "alpha-30", "alpha-50"] {
            let result = result_for(&report, id);
            assert!((result.effective_conductance - 1.0 / 3.0).abs() < TOL);
            assert!((result.upper_pressure_probe.unwrap() - 0.5).abs() < TOL);
            assert!((result.lower_pressure_probe.unwrap() - 0.5).abs() < TOL);
            assert!(
                (result
                    .localized_path_observation
                    .complete_path_flow
                    .unwrap()
                    - 1.0 / 6.0)
                    .abs()
                    < TOL
            );
            assert!(
                (result
                    .localized_path_observation
                    .fraction_of_total_flow
                    .unwrap()
                    - 0.5)
                    .abs()
                    < TOL
            );
        }
    }

    #[test]
    fn alpha70_keeps_only_lower_through_path_and_rejects_incomplete_upper_path() {
        let report = analyze(&fixture(), "test-digest".into()).unwrap();
        let result = result_for(&report, "alpha-70");
        assert!((result.effective_conductance - 0.25).abs() < TOL);
        assert!((result.upper_pressure_probe.unwrap() - 0.75).abs() < TOL);
        assert!((result.lower_pressure_probe.unwrap() - 0.5).abs() < TOL);
        assert!(result.source_sink_connected);
        assert_eq!(
            result.localized_path_observation.status,
            "path_not_fully_resolved"
        );
        assert_eq!(
            result
                .localized_path_observation
                .local_measurement_edge_flux,
            Some(0.0)
        );
        assert_eq!(result.localized_path_observation.complete_path_flow, None);
        assert_eq!(
            result.localized_path_observation.fraction_of_total_flow,
            None
        );
    }

    #[test]
    fn alpha90_has_zero_response_and_no_through_flow_fraction() {
        let report = analyze(&fixture(), "test-digest".into()).unwrap();
        let result = result_for(&report, "alpha-90");
        assert!(!result.source_sink_connected);
        assert_eq!(result.connected_component_count, 2);
        assert!(result.effective_conductance.abs() < TOL);
        assert!(
            result
                .boundary_response_matrix
                .iter()
                .flatten()
                .all(|value| value.abs() < TOL)
        );
        assert!((result.upper_pressure_probe.unwrap() - 1.0).abs() < TOL);
        assert!((result.lower_pressure_probe.unwrap() - 1.0).abs() < TOL);
        assert_eq!(
            result.localized_path_observation.status,
            "path_not_fully_resolved"
        );
        assert_eq!(
            result.localized_path_observation.fraction_of_total_flow,
            None
        );
    }

    #[test]
    fn all_variants_preserve_response_symmetry_and_boundary_balance() {
        let report = analyze(&fixture(), "test-digest".into()).unwrap();
        for result in &report.variant_results {
            assert!(result.response_symmetry_max_abs < TOL);
            assert!(result.response_row_sum_max_abs < TOL);
            assert!(result.boundary_flux_balance_abs < TOL);
        }
    }

    #[test]
    fn node_and_edge_storage_permutations_do_not_change_physical_response() {
        let original = fixture();
        let baseline = analyze(&original, "test-digest".into()).unwrap();
        let mut permuted = original;
        permuted.nodes.reverse();
        permuted.edges.rotate_left(2);
        let changed = analyze(&permuted, "test-digest".into()).unwrap();
        for id in ["alpha-10", "alpha-70", "alpha-90"] {
            let left = result_for(&baseline, id);
            let right = result_for(&changed, id);
            assert!((left.effective_conductance - right.effective_conductance).abs() < TOL);
            assert!(
                (left.boundary_response_matrix[0][1] - right.boundary_response_matrix[0][1]).abs()
                    < TOL
            );
            assert!(
                (left.upper_pressure_probe.unwrap() - right.upper_pressure_probe.unwrap()).abs()
                    < TOL
            );
        }
    }

    #[test]
    fn rejects_unanchored_interior_components() {
        let mut invalid = fixture();
        invalid.nodes.push(Node {
            node_id: "floating".into(),
        });
        for variant in &mut invalid.variants {
            variant.present_node_ids.push("floating".into());
        }
        assert!(validate(&invalid).is_err());
    }

    #[test]
    fn rejects_unknown_edges_and_incomplete_path_declarations() {
        let mut invalid = fixture();
        invalid.variants[0]
            .present_edge_ids
            .push("unknown-edge".into());
        assert!(validate(&invalid).is_err());
        let mut invalid_path = fixture();
        invalid_path.localized_path_edge_ids = vec!["upper-a".into()];
        assert!(validate(&invalid_path).is_err());
    }
}
