use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    env, fs,
    fs::OpenOptions,
    io::{self, Write},
    path::Path,
};

const VERSION: &str = "cell-complex-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const CLAIM_CEILING: &str = "Manufactured linear network reference only. It validates oriented incidence, boundary-of-boundary identities, positive edge conductances, Dirichlet network solutions, local and global flux balance, and no-overwrite output. It is not a Richards, Stokes, LBM, heat-PDE or reactive-transport solver and does not validate soil physics.";

#[derive(Clone, Debug, Deserialize)]
struct Scenario {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    source_artifact_uri: String,
    source_artifact_sha256: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    faces: Vec<Face>,
    volumes: Vec<Volume>,
    processes: Vec<Process>,
}

#[derive(Clone, Debug, Deserialize)]
struct Node {
    node_id: String,
    coordinates_m: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
struct Edge {
    edge_id: String,
    tail_node_id: String,
    head_node_id: String,
    length_m: f64,
    cross_section_m2: f64,
}

#[derive(Clone, Debug, Deserialize)]
struct OrientedRef {
    cell_id: String,
    orientation: i8,
}

#[derive(Clone, Debug, Deserialize)]
struct Face {
    face_id: String,
    boundary_edges: Vec<OrientedRef>,
}

#[derive(Clone, Debug, Deserialize)]
struct Volume {
    volume_id: String,
    boundary_faces: Vec<OrientedRef>,
}

#[derive(Clone, Debug, Deserialize)]
struct Process {
    process_id: String,
    process_name: String,
    potential_name: String,
    potential_unit: String,
    flux_name: String,
    flux_unit: String,
    conductance_name: String,
    conductance_unit: String,
    edge_conductances: Vec<EdgeConductance>,
    dirichlet_boundaries: Vec<DirichletBoundary>,
}

#[derive(Clone, Debug, Deserialize)]
struct EdgeConductance {
    edge_id: String,
    value: f64,
}

#[derive(Clone, Debug, Deserialize)]
struct DirichletBoundary {
    node_id: String,
    role: String,
    potential_value: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    source_artifact_sha256: String,
    cell_counts: CellCounts,
    boundary_identity_checks: BoundaryIdentityChecks,
    process_results: Vec<ProcessResult>,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct CellCounts {
    zero_cells: usize,
    one_cells: usize,
    two_cells: usize,
    three_cells: usize,
}

#[derive(Debug, Serialize)]
struct BoundaryIdentityChecks {
    checked_faces: usize,
    checked_volumes: usize,
    nonzero_boundary_of_boundary_terms: usize,
}

#[derive(Debug, Serialize)]
struct ProcessResult {
    process_id: String,
    process_name: String,
    potential_name: String,
    potential_unit: String,
    flux_name: String,
    flux_unit: String,
    conductance_name: String,
    conductance_unit: String,
    node_potentials: Vec<NodePotentialResult>,
    edge_fluxes: Vec<EdgeFluxResult>,
    source_flux: f64,
    sink_flux: f64,
    relative_boundary_balance_error: f64,
    max_interior_divergence_abs: f64,
    conductance_quadratic_form: f64,
    quadratic_form_unit: String,
}

#[derive(Debug, Serialize)]
struct NodePotentialResult {
    node_id: String,
    potential_value: f64,
    incidence_balance: f64,
    boundary_role: Option<String>,
}

#[derive(Debug, Serialize)]
struct EdgeFluxResult {
    edge_id: String,
    tail_node_id: String,
    head_node_id: String,
    conductance_value: f64,
    flux_value: f64,
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn finite(name: &str, value: f64) -> Result<(), String> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} must be finite"))
    }
}

fn validate(scenario: &Scenario) -> Result<BoundaryIdentityChecks, String> {
    if scenario.schema_version != "cell-complex-reference-v0.1"
        || scenario.fixture_status != FIXTURE_STATUS
        || scenario.scenario_id.trim().is_empty()
        || scenario.source_artifact_uri.trim().is_empty()
        || !valid_sha256(&scenario.source_artifact_sha256)
    {
        return Err(
            "scenario identity, fixture status or source artifact provenance is invalid".into(),
        );
    }
    if scenario.nodes.len() < 2 || scenario.edges.is_empty() || scenario.processes.is_empty() {
        return Err("the complex requires at least two nodes, one edge and one process".into());
    }

    let mut nodes = BTreeMap::new();
    for node in &scenario.nodes {
        if node.node_id.trim().is_empty() || nodes.contains_key(node.node_id.as_str()) {
            return Err("node IDs must be non-empty and unique".into());
        }
        for coordinate in node.coordinates_m {
            finite("node coordinate", coordinate)?;
        }
        nodes.insert(node.node_id.as_str(), node);
    }

    let mut edges = BTreeMap::new();
    for edge in &scenario.edges {
        if edge.edge_id.trim().is_empty()
            || edges.contains_key(edge.edge_id.as_str())
            || edge.tail_node_id == edge.head_node_id
            || !nodes.contains_key(edge.tail_node_id.as_str())
            || !nodes.contains_key(edge.head_node_id.as_str())
        {
            return Err("edges require unique IDs and distinct registered endpoints".into());
        }
        finite("edge length", edge.length_m)?;
        finite("edge cross-section", edge.cross_section_m2)?;
        if edge.length_m <= 0.0 || edge.cross_section_m2 <= 0.0 {
            return Err("edge length and cross-section must be positive".into());
        }
        edges.insert(edge.edge_id.as_str(), edge);
    }

    let mut faces = BTreeMap::new();
    let mut nonzero_terms = 0usize;
    for face in &scenario.faces {
        if face.face_id.trim().is_empty()
            || faces.contains_key(face.face_id.as_str())
            || face.boundary_edges.len() < 3
        {
            return Err(
                "faces require unique IDs and at least three oriented boundary edges".into(),
            );
        }
        let mut seen_edges = BTreeSet::new();
        let mut vertex_boundary: BTreeMap<&str, i32> = BTreeMap::new();
        for term in &face.boundary_edges {
            if !matches!(term.orientation, -1 | 1) || !seen_edges.insert(term.cell_id.as_str()) {
                return Err(format!(
                    "face {} has duplicate edges or invalid orientation",
                    face.face_id
                ));
            }
            let edge = edges.get(term.cell_id.as_str()).ok_or_else(|| {
                format!(
                    "face {} references unknown edge {}",
                    face.face_id, term.cell_id
                )
            })?;
            *vertex_boundary
                .entry(edge.tail_node_id.as_str())
                .or_default() -= i32::from(term.orientation);
            *vertex_boundary
                .entry(edge.head_node_id.as_str())
                .or_default() += i32::from(term.orientation);
        }
        let residual = vertex_boundary
            .values()
            .filter(|value| **value != 0)
            .count();
        nonzero_terms += residual;
        if residual != 0 {
            return Err(format!(
                "boundary-of-boundary is nonzero for face {}",
                face.face_id
            ));
        }
        faces.insert(face.face_id.as_str(), face);
    }

    let mut volumes = BTreeMap::new();
    for volume in &scenario.volumes {
        if volume.volume_id.trim().is_empty()
            || volumes.contains_key(volume.volume_id.as_str())
            || volume.boundary_faces.len() < 4
        {
            return Err(
                "volumes require unique IDs and at least four oriented boundary faces".into(),
            );
        }
        let mut seen_faces = BTreeSet::new();
        let mut edge_boundary: BTreeMap<&str, i32> = BTreeMap::new();
        for term in &volume.boundary_faces {
            if !matches!(term.orientation, -1 | 1) || !seen_faces.insert(term.cell_id.as_str()) {
                return Err(format!(
                    "volume {} has duplicate faces or invalid orientation",
                    volume.volume_id
                ));
            }
            let face = faces.get(term.cell_id.as_str()).ok_or_else(|| {
                format!(
                    "volume {} references unknown face {}",
                    volume.volume_id, term.cell_id
                )
            })?;
            for edge_term in &face.boundary_edges {
                *edge_boundary.entry(edge_term.cell_id.as_str()).or_default() +=
                    i32::from(term.orientation) * i32::from(edge_term.orientation);
            }
        }
        let residual = edge_boundary.values().filter(|value| **value != 0).count();
        nonzero_terms += residual;
        if residual != 0 {
            return Err(format!(
                "boundary-of-boundary is nonzero for volume {}",
                volume.volume_id
            ));
        }
        volumes.insert(volume.volume_id.as_str(), volume);
    }

    let mut process_ids = BTreeSet::new();
    for process in &scenario.processes {
        if process.process_id.trim().is_empty()
            || !process_ids.insert(process.process_id.as_str())
            || process.process_name.trim().is_empty()
            || process.potential_name.trim().is_empty()
            || process.potential_unit.trim().is_empty()
            || process.flux_name.trim().is_empty()
            || process.flux_unit.trim().is_empty()
            || process.conductance_name.trim().is_empty()
            || process.conductance_unit.trim().is_empty()
        {
            return Err("process identity, names and units must be explicit and unique".into());
        }
        if process.edge_conductances.len() != scenario.edges.len() {
            return Err(format!(
                "process {} must declare one conductance for every edge",
                process.process_id
            ));
        }
        let mut conductance_ids = BTreeSet::new();
        for coefficient in &process.edge_conductances {
            if !edges.contains_key(coefficient.edge_id.as_str())
                || !conductance_ids.insert(coefficient.edge_id.as_str())
            {
                return Err(format!(
                    "process {} has duplicate or unknown edge conductance",
                    process.process_id
                ));
            }
            finite("edge conductance", coefficient.value)?;
            if coefficient.value <= 0.0 {
                return Err("edge conductance must be strictly positive in v0.1".into());
            }
        }
        if process.dirichlet_boundaries.len() < 2 {
            return Err(format!(
                "process {} needs at least two Dirichlet boundary nodes",
                process.process_id
            ));
        }
        let mut boundary_nodes = BTreeSet::new();
        let mut source_values = Vec::new();
        let mut sink_values = Vec::new();
        for boundary in &process.dirichlet_boundaries {
            if !nodes.contains_key(boundary.node_id.as_str())
                || !boundary_nodes.insert(boundary.node_id.as_str())
                || !matches!(boundary.role.as_str(), "source" | "sink")
            {
                return Err(format!(
                    "process {} has invalid/duplicate Dirichlet boundary",
                    process.process_id
                ));
            }
            finite("Dirichlet potential", boundary.potential_value)?;
            if boundary.role == "source" {
                source_values.push(boundary.potential_value);
            } else {
                sink_values.push(boundary.potential_value);
            }
        }
        if source_values.is_empty()
            || sink_values.is_empty()
            || source_values.iter().copied().fold(f64::INFINITY, f64::min)
                <= sink_values
                    .iter()
                    .copied()
                    .fold(f64::NEG_INFINITY, f64::max)
        {
            return Err(format!(
                "process {} requires source potentials greater than sink potentials",
                process.process_id
            ));
        }

        // Every connected component must be anchored by a declared Dirichlet node.
        let mut adjacency: BTreeMap<&str, Vec<&str>> =
            nodes.keys().map(|node_id| (*node_id, Vec::new())).collect();
        for edge in &scenario.edges {
            adjacency
                .get_mut(edge.tail_node_id.as_str())
                .expect("validated tail")
                .push(edge.head_node_id.as_str());
            adjacency
                .get_mut(edge.head_node_id.as_str())
                .expect("validated head")
                .push(edge.tail_node_id.as_str());
        }
        let mut visited = BTreeSet::new();
        let mut queue: VecDeque<&str> = process
            .dirichlet_boundaries
            .iter()
            .map(|boundary| boundary.node_id.as_str())
            .collect();
        visited.extend(queue.iter().copied());
        while let Some(node_id) = queue.pop_front() {
            if let Some(neighbours) = adjacency.get(node_id) {
                for neighbour in neighbours {
                    if visited.insert(*neighbour) {
                        queue.push_back(neighbour);
                    }
                }
            }
        }
        if visited.len() != nodes.len() {
            return Err(format!(
                "process {} contains a component not anchored by a Dirichlet boundary",
                process.process_id
            ));
        }
    }

    Ok(BoundaryIdentityChecks {
        checked_faces: scenario.faces.len(),
        checked_volumes: scenario.volumes.len(),
        nonzero_boundary_of_boundary_terms: nonzero_terms,
    })
}

fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = rhs.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    if matrix.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("linear system must be square".into());
    }

    for pivot in 0..n {
        let pivot_row = (pivot..n)
            .max_by(|left, right| {
                matrix[*left][pivot]
                    .abs()
                    .total_cmp(&matrix[*right][pivot].abs())
            })
            .ok_or("linear system has no pivot row")?;
        if matrix[pivot_row][pivot].abs() <= 1e-14 {
            return Err("linear system is singular or numerically unconstrained".into());
        }
        matrix.swap(pivot, pivot_row);
        rhs.swap(pivot, pivot_row);

        let pivot_value = matrix[pivot][pivot];
        for row in (pivot + 1)..n {
            let factor = matrix[row][pivot] / pivot_value;
            matrix[row][pivot] = 0.0;
            for column in (pivot + 1)..n {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    let mut solution = vec![0.0; n];
    for row in (0..n).rev() {
        let tail_sum: f64 = ((row + 1)..n)
            .map(|column| matrix[row][column] * solution[column])
            .sum();
        solution[row] = (rhs[row] - tail_sum) / matrix[row][row];
        finite("solved potential", solution[row])?;
    }
    Ok(solution)
}

fn solve_process(scenario: &Scenario, process: &Process) -> Result<ProcessResult, String> {
    let node_ids: Vec<&str> = scenario
        .nodes
        .iter()
        .map(|node| node.node_id.as_str())
        .collect();
    let node_index: BTreeMap<&str, usize> = node_ids
        .iter()
        .enumerate()
        .map(|(index, node_id)| (*node_id, index))
        .collect();
    let conductance_by_edge: BTreeMap<&str, f64> = process
        .edge_conductances
        .iter()
        .map(|coefficient| (coefficient.edge_id.as_str(), coefficient.value))
        .collect();
    let n = node_ids.len();
    let mut laplacian = vec![vec![0.0; n]; n];

    for edge in &scenario.edges {
        let tail = *node_index
            .get(edge.tail_node_id.as_str())
            .expect("validated tail");
        let head = *node_index
            .get(edge.head_node_id.as_str())
            .expect("validated head");
        let conductance = *conductance_by_edge
            .get(edge.edge_id.as_str())
            .expect("validated conductance");
        laplacian[tail][tail] += conductance;
        laplacian[head][head] += conductance;
        laplacian[tail][head] -= conductance;
        laplacian[head][tail] -= conductance;
    }

    let mut potentials = vec![0.0; n];
    let mut boundary_by_node = BTreeMap::new();
    let mut unknown_indices = Vec::new();
    let mut boundary_indices = Vec::new();
    for boundary in &process.dirichlet_boundaries {
        let index = *node_index
            .get(boundary.node_id.as_str())
            .expect("validated node");
        potentials[index] = boundary.potential_value;
        boundary_indices.push(index);
        boundary_by_node.insert(boundary.node_id.as_str(), boundary);
    }
    for node_id in &node_ids {
        let index = *node_index.get(node_id).expect("node index exists");
        if !boundary_by_node.contains_key(node_id) {
            unknown_indices.push(index);
        }
    }

    let mut reduced_matrix = vec![vec![0.0; unknown_indices.len()]; unknown_indices.len()];
    let mut reduced_rhs = vec![0.0; unknown_indices.len()];
    for (row, global_row) in unknown_indices.iter().copied().enumerate() {
        for (column, global_column) in unknown_indices.iter().copied().enumerate() {
            reduced_matrix[row][column] = laplacian[global_row][global_column];
        }
        for boundary_index in &boundary_indices {
            reduced_rhs[row] -=
                laplacian[global_row][*boundary_index] * potentials[*boundary_index];
        }
    }
    let interior_solution = solve_dense(reduced_matrix, reduced_rhs)?;
    for (global_index, value) in unknown_indices.iter().copied().zip(interior_solution) {
        potentials[global_index] = value;
    }

    let mut incidence_balance = vec![0.0; n];
    let mut edge_fluxes = Vec::new();
    let mut quadratic_form = 0.0;
    for edge in &scenario.edges {
        let tail = *node_index
            .get(edge.tail_node_id.as_str())
            .expect("validated tail");
        let head = *node_index
            .get(edge.head_node_id.as_str())
            .expect("validated head");
        let conductance = *conductance_by_edge
            .get(edge.edge_id.as_str())
            .expect("validated conductance");
        // Positive flux follows the registered tail -> head orientation.
        let flux = conductance * (potentials[tail] - potentials[head]);
        finite("edge flux", flux)?;
        incidence_balance[tail] -= flux;
        incidence_balance[head] += flux;
        quadratic_form += conductance * (potentials[tail] - potentials[head]).powi(2);
        edge_fluxes.push(EdgeFluxResult {
            edge_id: edge.edge_id.clone(),
            tail_node_id: edge.tail_node_id.clone(),
            head_node_id: edge.head_node_id.clone(),
            conductance_value: conductance,
            flux_value: flux,
        });
    }
    edge_fluxes.sort_by_key(|edge| edge.edge_id.clone());

    let source_flux: f64 = process
        .dirichlet_boundaries
        .iter()
        .filter(|boundary| boundary.role == "source")
        .map(|boundary| {
            -incidence_balance[*node_index
                .get(boundary.node_id.as_str())
                .expect("validated node")]
        })
        .sum();
    let sink_flux: f64 = process
        .dirichlet_boundaries
        .iter()
        .filter(|boundary| boundary.role == "sink")
        .map(|boundary| {
            incidence_balance[*node_index
                .get(boundary.node_id.as_str())
                .expect("validated node")]
        })
        .sum();

    let max_interior_divergence_abs = unknown_indices
        .iter()
        .map(|index| incidence_balance[*index].abs())
        .fold(0.0, f64::max);
    let relative_boundary_balance_error =
        (source_flux - sink_flux).abs() / source_flux.abs().max(sink_flux.abs()).max(1e-300);
    finite("source flux", source_flux)?;
    finite("sink flux", sink_flux)?;
    finite("boundary balance error", relative_boundary_balance_error)?;
    finite("conductance quadratic form", quadratic_form)?;

    let mut node_potentials = Vec::new();
    for (index, node_id) in node_ids.iter().enumerate() {
        let boundary_role = boundary_by_node
            .get(node_id)
            .map(|boundary| boundary.role.clone());
        node_potentials.push(NodePotentialResult {
            node_id: (*node_id).to_string(),
            potential_value: potentials[index],
            incidence_balance: incidence_balance[index],
            boundary_role,
        });
    }
    node_potentials.sort_by_key(|node| node.node_id.clone());

    Ok(ProcessResult {
        process_id: process.process_id.clone(),
        process_name: process.process_name.clone(),
        potential_name: process.potential_name.clone(),
        potential_unit: process.potential_unit.clone(),
        flux_name: process.flux_name.clone(),
        flux_unit: process.flux_unit.clone(),
        conductance_name: process.conductance_name.clone(),
        conductance_unit: process.conductance_unit.clone(),
        node_potentials,
        edge_fluxes,
        source_flux,
        sink_flux,
        relative_boundary_balance_error,
        max_interior_divergence_abs,
        conductance_quadratic_form: quadratic_form,
        quadratic_form_unit: format!("{}*{}", process.conductance_unit, process.potential_unit),
    })
}

fn build_report(scenario: &Scenario, input_sha256: String) -> Result<Report, String> {
    let boundary_identity_checks = validate(scenario)?;
    let mut process_results = Vec::new();
    for process in &scenario.processes {
        process_results.push(solve_process(scenario, process)?);
    }
    process_results.sort_by(|left, right| left.process_id.cmp(&right.process_id));

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: FIXTURE_STATUS.into(),
        scenario_id: scenario.scenario_id.clone(),
        input_sha256,
        source_artifact_sha256: scenario.source_artifact_sha256.clone(),
        cell_counts: CellCounts {
            zero_cells: scenario.nodes.len(),
            one_cells: scenario.edges.len(),
            two_cells: scenario.faces.len(),
            three_cells: scenario.volumes.len(),
        },
        boundary_identity_checks,
        process_results,
        interpretation_boundary: CLAIM_CEILING.into(),
    })
}

fn run(input_path: &Path, output_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let input_bytes = fs::read(input_path)?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let scenario: Scenario = serde_json::from_slice(&input_bytes)?;
    let report = build_report(&scenario, input_sha256)
        .map_err(|message| io::Error::new(io::ErrorKind::InvalidInput, message))?;
    let output = serde_json::to_vec_pretty(&report)?;
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)?;
    file.write_all(&output)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("usage: cell-complex-reference <input.json> <new-output.json>");
        std::process::exit(2);
    }
    if let Err(error) = run(Path::new(&args[1]), Path::new(&args[2])) {
        eprintln!("cell-complex-reference: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube_fixture() -> Scenario {
        serde_json::from_str(include_str!("../fixtures/synthetic-cell-complex.json"))
            .expect("synthetic cube fixture parses")
    }

    fn make_network(conductances: &[f64]) -> Scenario {
        let mut scenario = cube_fixture();
        scenario.nodes = vec![
            Node {
                node_id: "source".into(),
                coordinates_m: [0.0, 0.0, 0.0],
            },
            Node {
                node_id: "mid".into(),
                coordinates_m: [1.0, 0.0, 0.0],
            },
            Node {
                node_id: "sink".into(),
                coordinates_m: [2.0, 0.0, 0.0],
            },
        ];
        scenario.edges = vec![
            Edge {
                edge_id: "first".into(),
                tail_node_id: "source".into(),
                head_node_id: "mid".into(),
                length_m: 1.0,
                cross_section_m2: 1.0,
            },
            Edge {
                edge_id: "second".into(),
                tail_node_id: "mid".into(),
                head_node_id: "sink".into(),
                length_m: 1.0,
                cross_section_m2: 1.0,
            },
        ];
        scenario.faces.clear();
        scenario.volumes.clear();
        scenario.processes = vec![Process {
            process_id: "test".into(),
            process_name: "manufactured scalar transport".into(),
            potential_name: "potential".into(),
            potential_unit: "u".into(),
            flux_name: "flux".into(),
            flux_unit: "f".into(),
            conductance_name: "conductance".into(),
            conductance_unit: "f/u".into(),
            edge_conductances: scenario
                .edges
                .iter()
                .zip(conductances.iter())
                .map(|(edge, value)| EdgeConductance {
                    edge_id: edge.edge_id.clone(),
                    value: *value,
                })
                .collect(),
            dirichlet_boundaries: vec![
                DirichletBoundary {
                    node_id: "source".into(),
                    role: "source".into(),
                    potential_value: 12.0,
                },
                DirichletBoundary {
                    node_id: "sink".into(),
                    role: "sink".into(),
                    potential_value: 0.0,
                },
            ],
        }];
        scenario
    }

    #[test]
    fn cube_has_zero_boundary_of_boundary() {
        let scenario = cube_fixture();
        let checks = validate(&scenario).expect("cube incidence validates");
        assert_eq!(checks.checked_faces, 6);
        assert_eq!(checks.checked_volumes, 1);
        assert_eq!(checks.nonzero_boundary_of_boundary_terms, 0);
    }

    #[test]
    fn same_complex_supports_distinct_hydraulic_and_thermal_operators() {
        let report = build_report(&cube_fixture(), "0".repeat(64)).unwrap();
        assert_eq!(report.cell_counts.zero_cells, 8);
        assert_eq!(report.cell_counts.one_cells, 12);
        assert_eq!(report.cell_counts.two_cells, 6);
        assert_eq!(report.cell_counts.three_cells, 1);
        let hydraulic = report
            .process_results
            .iter()
            .find(|result| result.process_id == "hydraulic")
            .unwrap();
        let thermal = report
            .process_results
            .iter()
            .find(|result| result.process_id == "thermal")
            .unwrap();
        assert!((hydraulic.source_flux - 4.0).abs() < 1e-12);
        assert!((hydraulic.sink_flux - 4.0).abs() < 1e-12);
        assert!((thermal.source_flux - 8.0).abs() < 1e-12);
        assert!((thermal.sink_flux - 8.0).abs() < 1e-12);
        assert!(hydraulic.relative_boundary_balance_error < 1e-12);
        assert!(thermal.relative_boundary_balance_error < 1e-12);
    }

    #[test]
    fn series_network_matches_analytic_resistance() {
        let scenario = make_network(&[2.0, 4.0]);
        let result = solve_process(&scenario, &scenario.processes[0]).unwrap();
        let expected_flux = 12.0 / (1.0 / 2.0 + 1.0 / 4.0);
        assert!((result.source_flux - expected_flux).abs() < 1e-12);
        assert!((result.sink_flux - expected_flux).abs() < 1e-12);
        let middle = result
            .node_potentials
            .iter()
            .find(|node| node.node_id == "mid")
            .unwrap();
        assert!((middle.potential_value - 4.0).abs() < 1e-12);
        assert!(result.max_interior_divergence_abs < 1e-12);
    }

    #[test]
    fn parallel_edges_sum_conductance() {
        let mut scenario = make_network(&[2.0, 4.0]);
        scenario.nodes.pop();
        scenario.nodes[1].node_id = "sink".into();
        scenario.edges[0].head_node_id = "sink".into();
        scenario.edges[1].tail_node_id = "source".into();
        scenario.edges[1].head_node_id = "sink".into();
        scenario.edges[1].edge_id = "parallel".into();
        scenario.processes[0].edge_conductances[1].edge_id = "parallel".into();
        scenario.processes[0].dirichlet_boundaries = vec![
            DirichletBoundary {
                node_id: "source".into(),
                role: "source".into(),
                potential_value: 12.0,
            },
            DirichletBoundary {
                node_id: "sink".into(),
                role: "sink".into(),
                potential_value: 0.0,
            },
        ];
        let result = solve_process(&scenario, &scenario.processes[0]).unwrap();
        assert!((result.source_flux - 72.0).abs() < 1e-12);
    }

    #[test]
    fn edge_orientation_reversal_preserves_physical_solution() {
        let mut scenario = make_network(&[2.0, 4.0]);
        let before = solve_process(&scenario, &scenario.processes[0]).unwrap();
        let edge = &mut scenario.edges[0];
        std::mem::swap(&mut edge.tail_node_id, &mut edge.head_node_id);
        let after = solve_process(&scenario, &scenario.processes[0]).unwrap();
        assert!((before.source_flux - after.source_flux).abs() < 1e-12);
        assert!((before.sink_flux - after.sink_flux).abs() < 1e-12);
        let first_before = before
            .edge_fluxes
            .iter()
            .find(|edge| edge.edge_id == "first")
            .unwrap()
            .flux_value;
        let first_after = after
            .edge_fluxes
            .iter()
            .find(|edge| edge.edge_id == "first")
            .unwrap()
            .flux_value;
        assert!((first_before + first_after).abs() < 1e-12);
    }

    #[test]
    fn quadratic_form_matches_boundary_power() {
        let scenario = make_network(&[2.0, 4.0]);
        let result = solve_process(&scenario, &scenario.processes[0]).unwrap();
        assert!((result.conductance_quadratic_form - 12.0 * result.source_flux).abs() < 1e-12);
    }

    #[test]
    fn rejects_nonzero_face_boundary_residual() {
        let mut scenario = cube_fixture();
        scenario.faces[0].boundary_edges[0].orientation *= -1;
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_nonzero_volume_boundary_residual() {
        let mut scenario = cube_fixture();
        scenario.volumes[0].boundary_faces[0].orientation *= -1;
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_zero_edge_conductance_and_missing_edge_coefficient() {
        let mut scenario = cube_fixture();
        scenario.processes[0].edge_conductances[0].value = 0.0;
        assert!(validate(&scenario).is_err());
        let mut scenario = cube_fixture();
        scenario.processes[0].edge_conductances.pop();
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_disconnected_unanchored_component() {
        let mut scenario = make_network(&[2.0, 4.0]);
        scenario.nodes.push(Node {
            node_id: "island".into(),
            coordinates_m: [4.0, 0.0, 0.0],
        });
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_unknown_boundary_node_and_nonfinite_geometry() {
        let mut scenario = make_network(&[2.0, 4.0]);
        scenario.processes[0].dirichlet_boundaries[0].node_id = "unknown".into();
        assert!(validate(&scenario).is_err());
        let mut scenario = make_network(&[2.0, 4.0]);
        scenario.edges[0].length_m = f64::NAN;
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn rejects_duplicate_face_edge_and_volume_face_terms() {
        let mut scenario = cube_fixture();
        let duplicate_edge = scenario.faces[0].boundary_edges[0].clone();
        scenario.faces[0].boundary_edges.push(duplicate_edge);
        assert!(validate(&scenario).is_err());
        let mut scenario = cube_fixture();
        let duplicate_face = scenario.volumes[0].boundary_faces[0].clone();
        scenario.volumes[0].boundary_faces.push(duplicate_face);
        assert!(validate(&scenario).is_err());
    }

    #[test]
    fn report_process_order_is_deterministic() {
        let mut scenario = cube_fixture();
        let before = build_report(&scenario, "0".repeat(64)).unwrap();
        scenario.processes.reverse();
        let after = build_report(&scenario, "0".repeat(64)).unwrap();
        let before_ids: Vec<_> = before
            .process_results
            .iter()
            .map(|result| result.process_id.as_str())
            .collect();
        let after_ids: Vec<_> = after
            .process_results
            .iter()
            .map(|result| result.process_id.as_str())
            .collect();
        assert_eq!(before_ids, after_ids);
    }
}
