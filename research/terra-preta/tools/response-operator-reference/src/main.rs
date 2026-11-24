use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "response-operator-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const FD_STEP: f64 = 1.0e-5;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    boundary_nodes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Node {
    node_id: String,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Edge {
    edge_id: String,
    tail_node_id: String,
    head_node_id: String,
    conductance: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    boundary_nodes: Vec<String>,
    node_count: usize,
    edge_count: usize,
    interior_node_count: usize,
    boundary_response_matrix: Vec<Vec<f64>>,
    response_symmetry_max_abs: f64,
    response_row_sum_max_abs: f64,
    effective_two_terminal_conductance: Option<f64>,
    coarse_boundary_edges: Vec<CoarseEdge>,
    log_conductance_sensitivity: SensitivityReport,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct CoarseEdge {
    source_port: String,
    target_port: String,
    equivalent_conductance: f64,
}

#[derive(Debug, Serialize)]
struct SensitivityReport {
    observation_names: Vec<String>,
    parameter_edge_ids: Vec<String>,
    jacobian_d_response_d_log_conductance: Vec<Vec<f64>>,
    numerical_rank: usize,
    parameter_count: usize,
    local_nullity: usize,
    finite_difference_log_step: f64,
    rank_relative_tolerance: f64,
    interpretation: String,
}

fn validate(fixture: &Fixture) -> Result<BTreeMap<String, usize>, String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture identity, schema version or synthetic-only status is invalid".into());
    }
    if fixture.nodes.len() < 2 || fixture.edges.is_empty() || fixture.boundary_nodes.len() < 2 {
        return Err("fixture requires at least two nodes, one edge and two boundary nodes".into());
    }

    let mut node_index = BTreeMap::new();
    for (index, node) in fixture.nodes.iter().enumerate() {
        if node.node_id.trim().is_empty()
            || node_index.insert(node.node_id.clone(), index).is_some()
        {
            return Err("node IDs must be non-empty and unique".into());
        }
    }

    let mut edge_ids = BTreeSet::new();
    let mut adjacency = vec![Vec::new(); fixture.nodes.len()];
    for edge in &fixture.edges {
        if edge.edge_id.trim().is_empty() || !edge_ids.insert(edge.edge_id.as_str()) {
            return Err("edge IDs must be non-empty and unique".into());
        }
        let tail = *node_index
            .get(&edge.tail_node_id)
            .ok_or_else(|| format!("unknown tail node {}", edge.tail_node_id))?;
        let head = *node_index
            .get(&edge.head_node_id)
            .ok_or_else(|| format!("unknown head node {}", edge.head_node_id))?;
        if tail == head {
            return Err(format!(
                "self-loop edge {} is not supported in v0.1",
                edge.edge_id
            ));
        }
        if !edge.conductance.is_finite() || edge.conductance <= 0.0 {
            return Err(format!(
                "edge {} conductance must be finite and positive",
                edge.edge_id
            ));
        }
        adjacency[tail].push(head);
        adjacency[head].push(tail);
    }

    let mut boundary_ids = BTreeSet::new();
    let mut queue = VecDeque::new();
    for boundary in &fixture.boundary_nodes {
        if !boundary_ids.insert(boundary.as_str()) {
            return Err("boundary node IDs must be unique".into());
        }
        let index = *node_index
            .get(boundary)
            .ok_or_else(|| format!("unknown boundary node {boundary}"))?;
        queue.push_back(index);
    }

    // Every component must be anchored by at least one prescribed boundary node.
    let mut visited = vec![false; fixture.nodes.len()];
    while let Some(node) = queue.pop_front() {
        if visited[node] {
            continue;
        }
        visited[node] = true;
        for &neighbor in &adjacency[node] {
            if !visited[neighbor] {
                queue.push_back(neighbor);
            }
        }
    }
    if visited.iter().any(|seen| !seen) {
        return Err("an unanchored component has no path to any boundary node".into());
    }

    Ok(node_index)
}

fn laplacian(fixture: &Fixture, node_index: &BTreeMap<String, usize>) -> Vec<Vec<f64>> {
    let n = fixture.nodes.len();
    let mut matrix = vec![vec![0.0; n]; n];
    for edge in &fixture.edges {
        let tail = node_index[&edge.tail_node_id];
        let head = node_index[&edge.head_node_id];
        let g = edge.conductance;
        matrix[tail][tail] += g;
        matrix[head][head] += g;
        matrix[tail][head] -= g;
        matrix[head][tail] -= g;
    }
    matrix
}

#[allow(clippy::needless_range_loop)]
fn solve_linear(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = matrix.len();
    if n == 0 || rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("linear system dimensions are invalid".into());
    }
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    let pivot_floor = scale.max(f64::MIN_POSITIVE) * 1.0e-13;

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
            if factor == 0.0 {
                continue;
            }
            for j in column..n {
                matrix[row][j] -= factor * matrix[column][j];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    if rhs.iter().any(|value| !value.is_finite()) {
        return Err("linear solve produced a non-finite result".into());
    }
    Ok(rhs)
}

#[allow(clippy::needless_range_loop)]
fn response_matrix(fixture: &Fixture) -> Result<Vec<Vec<f64>>, String> {
    let node_index = validate(fixture)?;
    let l = laplacian(fixture, &node_index);
    let boundary_indices: Vec<usize> = fixture
        .boundary_nodes
        .iter()
        .map(|node| node_index[node])
        .collect();
    let boundary_set: BTreeSet<usize> = boundary_indices.iter().copied().collect();
    let interior_indices: Vec<usize> = (0..fixture.nodes.len())
        .filter(|index| !boundary_set.contains(index))
        .collect();

    let nb = boundary_indices.len();
    let ni = interior_indices.len();
    let mut response = vec![vec![0.0; nb]; nb];

    if ni == 0 {
        for row in 0..nb {
            for column in 0..nb {
                response[row][column] = l[boundary_indices[row]][boundary_indices[column]];
            }
        }
        return Ok(response);
    }

    let interior_block: Vec<Vec<f64>> = interior_indices
        .iter()
        .map(|&row| {
            interior_indices
                .iter()
                .map(|&column| l[row][column])
                .collect()
        })
        .collect();

    // Solve L_ii X = L_ib one boundary column at a time.
    let mut solved_columns = vec![vec![0.0; nb]; ni];
    for (boundary_column, &boundary_node) in boundary_indices.iter().enumerate() {
        let rhs: Vec<f64> = interior_indices
            .iter()
            .map(|&interior_node| l[interior_node][boundary_node])
            .collect();
        let solution = solve_linear(interior_block.clone(), rhs)?;
        for interior_row in 0..ni {
            solved_columns[interior_row][boundary_column] = solution[interior_row];
        }
    }

    // Kron / Schur reduction: Lambda = L_bb - L_bi L_ii^{-1} L_ib.
    for (row, &boundary_row) in boundary_indices.iter().enumerate() {
        for (column, &boundary_column) in boundary_indices.iter().enumerate() {
            let correction: f64 = interior_indices
                .iter()
                .enumerate()
                .map(|(k, &interior_node)| {
                    l[boundary_row][interior_node] * solved_columns[k][column]
                })
                .sum();
            response[row][column] = l[boundary_row][boundary_column] - correction;
        }
    }
    Ok(response)
}

fn vectorize_symmetric_upper(matrix: &[Vec<f64>]) -> (Vec<String>, Vec<f64>) {
    let mut names = Vec::new();
    let mut values = Vec::new();
    for (row, row_values) in matrix.iter().enumerate() {
        for (column, value) in row_values.iter().enumerate().skip(row) {
            names.push(format!("Lambda[{row},{column}]"));
            values.push(*value);
        }
    }
    (names, values)
}

#[allow(clippy::needless_range_loop)]
fn matrix_rank(matrix: &[Vec<f64>], relative_tolerance: f64) -> usize {
    if matrix.is_empty() || matrix[0].is_empty() {
        return 0;
    }
    let rows = matrix.len();
    let columns = matrix[0].len();
    let mut work = matrix.to_vec();
    let scale = work
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if scale == 0.0 {
        return 0;
    }
    let tolerance = scale * relative_tolerance;
    let mut pivot_row = 0usize;

    for column in 0..columns {
        if pivot_row >= rows {
            break;
        }
        let candidate = (pivot_row..rows).max_by(|&left, &right| {
            work[left][column]
                .abs()
                .total_cmp(&work[right][column].abs())
        });
        let Some(best_row) = candidate else {
            break;
        };
        if work[best_row][column].abs() <= tolerance {
            continue;
        }
        work.swap(pivot_row, best_row);
        let pivot = work[pivot_row][column];
        for row in (pivot_row + 1)..rows {
            let factor = work[row][column] / pivot;
            for col in column..columns {
                work[row][col] -= factor * work[pivot_row][col];
            }
        }
        pivot_row += 1;
    }
    pivot_row
}

#[allow(clippy::needless_range_loop)]
fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let base = response_matrix(fixture)?;
    let (observation_names, _) = vectorize_symmetric_upper(&base);

    let mut parameter_ids = Vec::with_capacity(fixture.edges.len());
    let mut parameter_columns: Vec<Vec<f64>> = Vec::with_capacity(fixture.edges.len());
    for (edge_index, edge) in fixture.edges.iter().enumerate() {
        let mut plus = fixture.clone();
        let mut minus = fixture.clone();
        plus.edges[edge_index].conductance *= FD_STEP.exp();
        minus.edges[edge_index].conductance *= (-FD_STEP).exp();
        let (_, plus_values) = vectorize_symmetric_upper(&response_matrix(&plus)?);
        let (_, minus_values) = vectorize_symmetric_upper(&response_matrix(&minus)?);
        let derivative: Vec<f64> = plus_values
            .iter()
            .zip(minus_values.iter())
            .map(|(high, low)| (high - low) / (2.0 * FD_STEP))
            .collect();
        parameter_ids.push(edge.edge_id.clone());
        parameter_columns.push(derivative);
    }

    let observation_count = observation_names.len();
    let mut jacobian = vec![vec![0.0; parameter_columns.len()]; observation_count];
    for (parameter, column) in parameter_columns.iter().enumerate() {
        for (observation, value) in column.iter().enumerate() {
            jacobian[observation][parameter] = *value;
        }
    }
    let relative_tolerance = 1.0e-8;
    let rank = matrix_rank(&jacobian, relative_tolerance);
    let parameter_count = fixture.edges.len();
    let local_nullity = parameter_count.saturating_sub(rank);

    let symmetry_max_abs = (0..base.len())
        .flat_map(|row| (0..base.len()).map(move |column| (row, column)))
        .map(|(row, column)| (base[row][column] - base[column][row]).abs())
        .fold(0.0_f64, f64::max);
    let row_sum_max_abs = base
        .iter()
        .map(|row| row.iter().sum::<f64>().abs())
        .fold(0.0_f64, f64::max);

    let mut coarse_edges = Vec::new();
    for row in 0..fixture.boundary_nodes.len() {
        for column in (row + 1)..fixture.boundary_nodes.len() {
            let conductance = -base[row][column];
            if conductance > 1.0e-12 {
                coarse_edges.push(CoarseEdge {
                    source_port: fixture.boundary_nodes[row].clone(),
                    target_port: fixture.boundary_nodes[column].clone(),
                    equivalent_conductance: conductance,
                });
            }
        }
    }

    let effective_two_terminal_conductance = if base.len() == 2 {
        Some(-base[0][1])
    } else {
        None
    };

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        boundary_nodes: fixture.boundary_nodes.clone(),
        node_count: fixture.nodes.len(),
        edge_count: fixture.edges.len(),
        interior_node_count: fixture.nodes.len() - fixture.boundary_nodes.len(),
        boundary_response_matrix: base,
        response_symmetry_max_abs: symmetry_max_abs,
        response_row_sum_max_abs: row_sum_max_abs,
        effective_two_terminal_conductance,
        coarse_boundary_edges: coarse_edges,
        log_conductance_sensitivity: SensitivityReport {
            observation_names,
            parameter_edge_ids: parameter_ids,
            jacobian_d_response_d_log_conductance: jacobian,
            numerical_rank: rank,
            parameter_count,
            local_nullity,
            finite_difference_log_step: FD_STEP,
            rank_relative_tolerance: relative_tolerance,
            interpretation: "Local nullity counts conductance-space directions not distinguishable by this boundary-response observation set at the evaluated parameter point. It is a numerical local diagnostic, not a global identifiability proof or a posterior uncertainty interval.".into(),
        },
        interpretation_boundary: "For a fixed positive linear steady conductance network, the Schur complement preserves the complete pressure-to-flux response at the declared boundary nodes. This does not establish equivalence for nonlinear unsaturated flow, transient storage, hysteresis, multiphase transport, reactive transport or unobserved boundary conditions. Synthetic software qualification only; no empirical soil validation is claimed.".into(),
    })
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().collect();
    if arguments.len() != 3 {
        return Err(format!(
            "usage: {} INPUT.json OUTPUT.json",
            arguments
                .first()
                .map(String::as_str)
                .unwrap_or("response-operator-reference")
        ));
    }
    let input_path = &arguments[1];
    let output_path = &arguments[2];
    let input_bytes =
        fs::read(input_path).map_err(|error| format!("cannot read input: {error}"))?;
    let input_sha256 = format!("{:x}", Sha256::digest(&input_bytes));
    let fixture: Fixture = serde_json::from_slice(&input_bytes)
        .map_err(|error| format!("invalid fixture JSON: {error}"))?;
    let report = analyze(&fixture, input_sha256)?;
    let output_bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report: {error}"))?;

    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output_path)
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output
        .write_all(&output_bytes)
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

    fn two_path_fixture(conductances: [f64; 4]) -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "two-parallel-series-paths".into(),
            nodes: vec![
                Node {
                    node_id: "left".into(),
                },
                Node {
                    node_id: "upper".into(),
                },
                Node {
                    node_id: "lower".into(),
                },
                Node {
                    node_id: "right".into(),
                },
            ],
            edges: vec![
                Edge {
                    edge_id: "e-left-upper".into(),
                    tail_node_id: "left".into(),
                    head_node_id: "upper".into(),
                    conductance: conductances[0],
                },
                Edge {
                    edge_id: "e-upper-right".into(),
                    tail_node_id: "upper".into(),
                    head_node_id: "right".into(),
                    conductance: conductances[1],
                },
                Edge {
                    edge_id: "e-left-lower".into(),
                    tail_node_id: "left".into(),
                    head_node_id: "lower".into(),
                    conductance: conductances[2],
                },
                Edge {
                    edge_id: "e-lower-right".into(),
                    tail_node_id: "lower".into(),
                    head_node_id: "right".into(),
                    conductance: conductances[3],
                },
            ],
            boundary_nodes: vec!["left".into(), "right".into()],
        }
    }

    fn close(left: f64, right: f64, tolerance: f64) {
        assert!((left - right).abs() <= tolerance, "{left} != {right}");
    }

    #[test]
    fn kron_reduction_preserves_two_terminal_response() {
        let fixture = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        let response = response_matrix(&fixture).expect("response");
        close(response[0][0], 2.0, 1.0e-12);
        close(response[0][1], -2.0, 1.0e-12);
        close(response[1][0], -2.0, 1.0e-12);
        close(response[1][1], 2.0, 1.0e-12);

        // The exact coarse equivalent is a single edge of conductance 2.
        let coarse = Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "coarse-equivalent".into(),
            nodes: vec![
                Node {
                    node_id: "left".into(),
                },
                Node {
                    node_id: "right".into(),
                },
            ],
            edges: vec![Edge {
                edge_id: "equivalent".into(),
                tail_node_id: "left".into(),
                head_node_id: "right".into(),
                conductance: 2.0,
            }],
            boundary_nodes: vec!["left".into(), "right".into()],
        };
        let coarse_response = response_matrix(&coarse).expect("coarse response");
        for row in 0..2 {
            for column in 0..2 {
                close(response[row][column], coarse_response[row][column], 1.0e-12);
            }
        }
    }

    #[test]
    fn edge_reversal_and_edge_storage_permutation_preserve_response() {
        let fixture = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        let original = response_matrix(&fixture).expect("original response");
        let mut transformed = fixture.clone();
        transformed.edges.reverse();
        for edge in &mut transformed.edges {
            std::mem::swap(&mut edge.tail_node_id, &mut edge.head_node_id);
        }
        let changed = response_matrix(&transformed).expect("transformed response");
        for row in 0..2 {
            for column in 0..2 {
                close(original[row][column], changed[row][column], 1.0e-12);
            }
        }
    }

    #[test]
    fn node_storage_permutation_preserves_boundary_response() {
        let fixture = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        let original = response_matrix(&fixture).expect("original response");
        let mut transformed = fixture.clone();
        transformed.nodes.reverse();
        let changed = response_matrix(&transformed).expect("permuted response");
        for row in 0..2 {
            for column in 0..2 {
                close(original[row][column], changed[row][column], 1.0e-12);
            }
        }
    }

    #[test]
    fn repeating_the_same_linear_boundary_pair_at_scaled_pressure_does_not_add_rank() {
        let fixture = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        let report = analyze(&fixture, "0".repeat(64)).expect("analysis");
        // The two-terminal effective conductance is -Lambda[left,right].
        let sensitivity = &report
            .log_conductance_sensitivity
            .jacobian_d_response_d_log_conductance[1];
        let stacked = vec![
            sensitivity.clone(),
            sensitivity.iter().map(|value| 10.0 * value).collect(),
        ];
        assert_eq!(matrix_rank(&stacked, 1.0e-8), 1);
    }

    #[test]
    fn bulk_response_exposes_local_conductance_non_identifiability() {
        let first = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        let second = two_path_fixture([4.0, 4.0 / 3.0, 3.0, 1.5]);
        let first_response = response_matrix(&first).expect("first response");
        let second_response = response_matrix(&second).expect("second response");
        close(-first_response[0][1], 2.0, 1.0e-12);
        close(-second_response[0][1], 2.0, 1.0e-12);

        let report = analyze(&first, "0".repeat(64)).expect("analysis");
        assert_eq!(report.log_conductance_sensitivity.numerical_rank, 1);
        assert_eq!(report.log_conductance_sensitivity.parameter_count, 4);
        assert_eq!(report.log_conductance_sensitivity.local_nullity, 3);
    }

    #[test]
    fn all_boundary_triangle_identifies_three_edge_conductances_locally() {
        let fixture = Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "fully-instrumented-triangle".into(),
            nodes: vec![
                Node {
                    node_id: "a".into(),
                },
                Node {
                    node_id: "b".into(),
                },
                Node {
                    node_id: "c".into(),
                },
            ],
            edges: vec![
                Edge {
                    edge_id: "ab".into(),
                    tail_node_id: "a".into(),
                    head_node_id: "b".into(),
                    conductance: 2.0,
                },
                Edge {
                    edge_id: "bc".into(),
                    tail_node_id: "b".into(),
                    head_node_id: "c".into(),
                    conductance: 3.0,
                },
                Edge {
                    edge_id: "ac".into(),
                    tail_node_id: "a".into(),
                    head_node_id: "c".into(),
                    conductance: 4.0,
                },
            ],
            boundary_nodes: vec!["a".into(), "b".into(), "c".into()],
        };
        let report = analyze(&fixture, "0".repeat(64)).expect("analysis");
        assert_eq!(report.log_conductance_sensitivity.numerical_rank, 3);
        assert_eq!(report.log_conductance_sensitivity.local_nullity, 0);
    }

    #[test]
    fn rejects_unanchored_components_and_nonpositive_conductances() {
        let mut fixture = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        fixture.nodes.push(Node {
            node_id: "isolated".into(),
        });
        assert!(validate(&fixture).is_err());

        let mut invalid = two_path_fixture([2.0, 2.0, 2.0, 2.0]);
        invalid.edges[0].conductance = 0.0;
        assert!(validate(&invalid).is_err());
    }
}
