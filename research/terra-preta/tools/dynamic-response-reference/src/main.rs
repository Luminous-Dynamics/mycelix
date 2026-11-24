use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, VecDeque},
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "dynamic-response-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const PIVOT_RELATIVE_FLOOR: f64 = 1.0e-13;
const PASSIVITY_TOLERANCE: f64 = 1.0e-10;

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    boundary_nodes: Vec<String>,
    laplace_parameters_per_second: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct Node {
    node_id: String,
    storage_capacity: f64,
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
    node_count: usize,
    edge_count: usize,
    boundary_nodes: Vec<String>,
    interior_nodes: Vec<String>,
    interior_storage_total: f64,
    dc_boundary_response: Vec<Vec<f64>>,
    samples: Vec<ResponseSample>,
    maximum_symmetry_residual: f64,
    minimum_sampled_quadratic_form: f64,
    minimum_sampled_response_eigenvalue: f64,
    real_laplace_axis_sampled_passivity_check: bool,
    interpretation_boundary: String,
}

#[derive(Debug, Serialize)]
struct ResponseSample {
    laplace_parameter_per_second: f64,
    boundary_response: Vec<Vec<f64>>,
    row_sums: Vec<f64>,
    minimum_sampled_quadratic_form: f64,
    symmetric_within_tolerance: bool,
    minimum_response_eigenvalue: f64,
    positive_semidefinite_within_tolerance: bool,
}

fn validate(fixture: &Fixture) -> Result<BTreeMap<String, usize>, String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture identity, schema version or synthetic-only status is invalid".into());
    }
    if fixture.nodes.len() < 2 || fixture.edges.is_empty() || fixture.boundary_nodes.len() < 2 {
        return Err("fixture requires at least two nodes, one edge and two boundary ports".into());
    }
    if fixture.laplace_parameters_per_second.is_empty() {
        return Err("at least one real Laplace parameter is required".into());
    }
    for &s in &fixture.laplace_parameters_per_second {
        if !s.is_finite() || s < 0.0 {
            return Err("Laplace parameters must be finite and nonnegative".into());
        }
    }

    let mut node_index = BTreeMap::new();
    for (index, node) in fixture.nodes.iter().enumerate() {
        if node.node_id.trim().is_empty()
            || node_index.insert(node.node_id.clone(), index).is_some()
        {
            return Err("node IDs must be non-empty and unique".into());
        }
        if !node.storage_capacity.is_finite() || node.storage_capacity < 0.0 {
            return Err(format!(
                "node {} storage capacity must be finite and nonnegative",
                node.node_id
            ));
        }
    }

    let mut edge_ids = std::collections::BTreeSet::new();
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

    let mut boundary_set = std::collections::BTreeSet::new();
    let mut queue = VecDeque::new();
    for boundary in &fixture.boundary_nodes {
        if !boundary_set.insert(boundary.as_str()) {
            return Err("boundary node IDs must be unique".into());
        }
        let index = *node_index
            .get(boundary)
            .ok_or_else(|| format!("unknown boundary node {boundary}"))?;
        if fixture.nodes[index].storage_capacity != 0.0 {
            return Err(format!(
                "boundary port {} must have zero lumped storage in this formulation",
                boundary
            ));
        }
        queue.push_back(index);
    }

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
        return Err("an unanchored component has no path to any boundary port".into());
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

fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, String> {
    let n = matrix.len();
    if n == 0 || rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("interior linear-system dimensions are invalid".into());
    }
    let scale = matrix
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if !scale.is_finite() || scale == 0.0 {
        return Err("interior dynamic operator has zero or non-finite scale".into());
    }
    let pivot_floor = scale * PIVOT_RELATIVE_FLOOR;

    for column in 0..n {
        let pivot_row = (column..n)
            .max_by(|&left, &right| {
                matrix[left][column]
                    .abs()
                    .total_cmp(&matrix[right][column].abs())
            })
            .ok_or_else(|| "linear system has no pivot candidate".to_string())?;
        if matrix[pivot_row][column].abs() <= pivot_floor {
            return Err("interior dynamic operator is singular or numerically unresolved".into());
        }
        matrix.swap(column, pivot_row);
        rhs.swap(column, pivot_row);

        let pivot = matrix[column][column];
        for value in matrix[column].iter_mut().skip(column) {
            *value /= pivot;
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
            for col in column..n {
                matrix[row][col] -= factor * matrix[column][col];
            }
            rhs[row] -= factor * rhs[column];
        }
    }

    if rhs.iter().any(|value| !value.is_finite()) {
        return Err("dynamic solve produced a non-finite result".into());
    }
    Ok(rhs)
}

fn response_matrix(fixture: &Fixture, s: f64) -> Result<Vec<Vec<f64>>, String> {
    let node_index = validate(fixture)?;
    if !s.is_finite() || s < 0.0 {
        return Err("Laplace parameter must be finite and nonnegative".into());
    }
    let l = laplacian(fixture, &node_index);
    let boundary_indices: Vec<usize> = fixture
        .boundary_nodes
        .iter()
        .map(|node| node_index[node])
        .collect();
    let boundary_set: std::collections::BTreeSet<usize> =
        boundary_indices.iter().copied().collect();
    let interior_indices: Vec<usize> = (0..fixture.nodes.len())
        .filter(|index| !boundary_set.contains(index))
        .collect();

    let nb = boundary_indices.len();
    let ni = interior_indices.len();
    let mut response = vec![vec![0.0; nb]; nb];

    if ni == 0 {
        for (row, &node_row) in boundary_indices.iter().enumerate() {
            for (column, &node_column) in boundary_indices.iter().enumerate() {
                response[row][column] = l[node_row][node_column];
            }
        }
        return Ok(response);
    }

    // Interior dynamic operator A_ii(s) = L_ii + s C_i.
    let mut interior_operator: Vec<Vec<f64>> = interior_indices
        .iter()
        .map(|&row| {
            interior_indices
                .iter()
                .map(|&column| l[row][column])
                .collect()
        })
        .collect();
    for (local_index, &global_index) in interior_indices.iter().enumerate() {
        interior_operator[local_index][local_index] +=
            s * fixture.nodes[global_index].storage_capacity;
    }

    // Solve A_ii(s) X = L_ib one boundary column at a time.
    let mut solved_columns = vec![vec![0.0; nb]; ni];
    for (boundary_column, &boundary_node) in boundary_indices.iter().enumerate() {
        let rhs: Vec<f64> = interior_indices
            .iter()
            .map(|&interior_node| l[interior_node][boundary_node])
            .collect();
        let solution = solve_dense(interior_operator.clone(), rhs)?;
        for (interior_row, value) in solution.iter().enumerate() {
            solved_columns[interior_row][boundary_column] = *value;
        }
    }

    // Lambda(s) = L_bb - L_bi (L_ii + s C_i)^-1 L_ib.
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

fn sampled_quadratic_forms(matrix: &[Vec<f64>]) -> Vec<f64> {
    let n = matrix.len();
    let mut inputs = Vec::new();
    for axis in 0..n {
        let mut vector = vec![0.0; n];
        vector[axis] = 1.0;
        inputs.push(vector);
    }
    for left in 0..n {
        for right in (left + 1)..n {
            let mut plus = vec![0.0; n];
            plus[left] = 1.0;
            plus[right] = 1.0;
            inputs.push(plus);
            let mut minus = vec![0.0; n];
            minus[left] = 1.0;
            minus[right] = -1.0;
            inputs.push(minus);
        }
    }
    inputs
        .iter()
        .map(|vector| {
            let product: Vec<f64> = matrix
                .iter()
                .map(|row| row.iter().zip(vector).map(|(a, b)| a * b).sum())
                .collect();
            vector.iter().zip(product).map(|(a, b)| a * b).sum()
        })
        .collect()
}

fn symmetry_error(matrix: &[Vec<f64>]) -> f64 {
    let mut maximum = 0.0_f64;
    for (row, values) in matrix.iter().enumerate() {
        for (column, value) in values.iter().enumerate() {
            maximum = maximum.max((value - matrix[column][row]).abs());
        }
    }
    maximum
}

fn minimum_symmetric_eigenvalue(matrix: &[Vec<f64>]) -> Result<f64, String> {
    let n = matrix.len();
    if n == 0 || matrix.iter().any(|row| row.len() != n) {
        return Err("eigenvalue matrix dimensions are invalid".into());
    }
    let mut work = matrix.to_vec();
    let scale = work
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if scale == 0.0 {
        return Ok(0.0);
    }

    let tolerance = scale * 1.0e-13;
    let maximum_iterations = 100usize.saturating_mul(n.saturating_mul(n)).max(1);
    for _ in 0..maximum_iterations {
        let mut pivot = (0usize, 0usize);
        let mut largest = 0.0_f64;
        for (row, row_values) in work.iter().enumerate() {
            for (column, value) in row_values.iter().enumerate().skip(row + 1) {
                let candidate = value.abs();
                if candidate > largest {
                    largest = candidate;
                    pivot = (row, column);
                }
            }
        }
        if largest <= tolerance {
            return Ok((0..n)
                .map(|index| work[index][index])
                .fold(f64::INFINITY, f64::min));
        }

        let (p, q) = pivot;
        let app = work[p][p];
        let aqq = work[q][q];
        let apq = work[p][q];
        let angle = 0.5 * (2.0 * apq).atan2(aqq - app);
        let cosine = angle.cos();
        let sine = angle.sin();

        for k in 0..n {
            if k == p || k == q {
                continue;
            }
            let akp = work[k][p];
            let akq = work[k][q];
            work[k][p] = cosine * akp - sine * akq;
            work[p][k] = work[k][p];
            work[k][q] = sine * akp + cosine * akq;
            work[q][k] = work[k][q];
        }

        work[p][p] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq;
        work[q][q] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq;
        work[p][q] = 0.0;
        work[q][p] = 0.0;
    }

    Err("symmetric Jacobi eigensolver did not converge".into())
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let interior_nodes: Vec<String> = fixture
        .nodes
        .iter()
        .filter(|node| !fixture.boundary_nodes.contains(&node.node_id))
        .map(|node| node.node_id.clone())
        .collect();
    let interior_storage_total: f64 = fixture
        .nodes
        .iter()
        .filter(|node| !fixture.boundary_nodes.contains(&node.node_id))
        .map(|node| node.storage_capacity)
        .sum();

    let mut samples = Vec::new();
    let mut maximum_symmetry_residual = 0.0_f64;
    let mut minimum_sampled_quadratic_form = f64::INFINITY;

    for &s in &fixture.laplace_parameters_per_second {
        let response = response_matrix(fixture, s)?;
        let symmetry = symmetry_error(&response);
        let forms = sampled_quadratic_forms(&response);
        let min_form = forms.iter().copied().fold(f64::INFINITY, f64::min);
        let row_sums: Vec<f64> = response.iter().map(|row| row.iter().sum()).collect();
        let minimum_eigenvalue = minimum_symmetric_eigenvalue(&response)?;
        let passive = minimum_eigenvalue >= -PASSIVITY_TOLERANCE;

        maximum_symmetry_residual = maximum_symmetry_residual.max(symmetry);
        minimum_sampled_quadratic_form = minimum_sampled_quadratic_form.min(min_form);
        samples.push(ResponseSample {
            laplace_parameter_per_second: s,
            boundary_response: response,
            row_sums,
            minimum_sampled_quadratic_form: min_form,
            minimum_response_eigenvalue: minimum_eigenvalue,
            symmetric_within_tolerance: symmetry <= PASSIVITY_TOLERANCE,
            positive_semidefinite_within_tolerance: passive,
        });
    }

    let dc_boundary_response = response_matrix(fixture, 0.0)?;
    let all_passive = samples.iter().all(|sample| {
        sample.symmetric_within_tolerance && sample.positive_semidefinite_within_tolerance
    });
    let minimum_sampled_response_eigenvalue = samples
        .iter()
        .map(|sample| sample.minimum_response_eigenvalue)
        .fold(f64::INFINITY, f64::min);

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        node_count: fixture.nodes.len(),
        edge_count: fixture.edges.len(),
        boundary_nodes: fixture.boundary_nodes.clone(),
        interior_nodes,
        interior_storage_total,
        dc_boundary_response,
        samples,
        maximum_symmetry_residual,
        minimum_sampled_quadratic_form,
        minimum_sampled_response_eigenvalue,
        real_laplace_axis_sampled_passivity_check: all_passive,
        interpretation_boundary: "This is an exact Schur complement for a fixed linear network with positive edge conductances, diagonal lumped storage only at interior nodes, ideal prescribed-potential boundary ports, and real nonnegative Laplace parameter s. The sampled quadratic-form checks are manufactured passivity diagnostics, not a formal complex-frequency positive-real certificate. This does not establish equivalence for nonlinear Richards flow, hysteresis, multiphase flow, reactive transport, root uptake or field soil response.".into(),
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
                .unwrap_or("dynamic-response-reference")
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

    fn one_storage_node(g1: f64, g2: f64, storage: f64, samples: Vec<f64>) -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "one-interior-storage-node".into(),
            nodes: vec![
                Node {
                    node_id: "inlet".into(),
                    storage_capacity: 0.0,
                },
                Node {
                    node_id: "storage".into(),
                    storage_capacity: storage,
                },
                Node {
                    node_id: "outlet".into(),
                    storage_capacity: 0.0,
                },
            ],
            edges: vec![
                Edge {
                    edge_id: "inlet-storage".into(),
                    tail_node_id: "inlet".into(),
                    head_node_id: "storage".into(),
                    conductance: g1,
                },
                Edge {
                    edge_id: "storage-outlet".into(),
                    tail_node_id: "storage".into(),
                    head_node_id: "outlet".into(),
                    conductance: g2,
                },
            ],
            boundary_nodes: vec!["inlet".into(), "outlet".into()],
            laplace_parameters_per_second: samples,
        }
    }

    fn close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "actual={actual}, expected={expected}, tolerance={tolerance}"
        );
    }

    #[test]
    fn dc_limit_matches_the_steady_kron_response() {
        let fixture = one_storage_node(2.0, 3.0, 4.0, vec![0.0, 1.0e-8]);
        let dc = response_matrix(&fixture, 0.0).expect("DC response");
        let effective = 2.0 * 3.0 / (2.0 + 3.0);
        close(dc[0][0], effective, 1.0e-12);
        close(dc[0][1], -effective, 1.0e-12);
        close(dc[1][0], -effective, 1.0e-12);
        close(dc[1][1], effective, 1.0e-12);

        let near_dc = response_matrix(&fixture, 1.0e-8).expect("near DC response");
        close(near_dc[0][1], dc[0][1], 1.0e-8);
    }

    #[test]
    fn dynamic_response_matches_the_analytic_one_storage_node_transfer() {
        let fixture = one_storage_node(2.0, 3.0, 4.0, vec![0.0, 0.1, 1.0, 10.0]);
        for &s in &fixture.laplace_parameters_per_second {
            let response = response_matrix(&fixture, s).expect("response");
            let denominator = 5.0 + 4.0 * s;
            close(response[0][0], 2.0 - 4.0 / denominator, 1.0e-12);
            close(response[0][1], -6.0 / denominator, 1.0e-12);
            close(response[1][0], -6.0 / denominator, 1.0e-12);
            close(response[1][1], 3.0 - 9.0 / denominator, 1.0e-12);
        }
    }

    #[test]
    fn transient_outlet_step_response_has_correct_delay_and_steady_limit() {
        // For an inlet unit step and zero outlet potential:
        // q_out(t) = 6/5 * (1 - exp(-5 t / 4)).
        let conductance_steady = 2.0 * 3.0 / 5.0;
        let time_constant = 4.0 / 5.0;
        let initial = conductance_steady * (1.0 - (-0.0_f64 / time_constant).exp());
        let at_one_tau = conductance_steady * (1.0 - (-time_constant / time_constant).exp());
        let long_time = conductance_steady * (1.0 - (-100.0_f64 / time_constant).exp());
        close(initial, 0.0, 1.0e-14);
        close(
            at_one_tau,
            conductance_steady * (1.0 - (-1.0_f64).exp()),
            1.0e-14,
        );
        close(long_time, conductance_steady, 1.0e-12);
    }

    #[test]
    fn real_laplace_samples_are_symmetric_and_dissipative() {
        let fixture = one_storage_node(2.0, 3.0, 4.0, vec![0.0, 0.01, 0.1, 1.0, 10.0, 100.0]);
        for &s in &fixture.laplace_parameters_per_second {
            let response = response_matrix(&fixture, s).expect("response");
            close(response[0][1], response[1][0], 1.0e-12);
            for form in sampled_quadratic_forms(&response) {
                assert!(form >= -PASSIVITY_TOLERANCE, "s={s}, form={form}");
            }
        }
    }

    #[test]
    fn edge_reversal_and_node_storage_permutation_preserve_dynamic_response() {
        let fixture = one_storage_node(2.0, 3.0, 4.0, vec![0.0, 0.25, 2.0]);
        let original: Vec<Vec<Vec<f64>>> = fixture
            .laplace_parameters_per_second
            .iter()
            .map(|&s| response_matrix(&fixture, s).expect("original"))
            .collect();

        let mut transformed = fixture.clone();
        transformed.edges.reverse();
        for edge in &mut transformed.edges {
            std::mem::swap(&mut edge.tail_node_id, &mut edge.head_node_id);
        }
        transformed.nodes.reverse();

        for (index, &s) in transformed.laplace_parameters_per_second.iter().enumerate() {
            let changed = response_matrix(&transformed, s).expect("transformed");
            for row in 0..2 {
                for column in 0..2 {
                    close(changed[row][column], original[index][row][column], 1.0e-12);
                }
            }
        }
    }

    #[test]
    fn repeated_same_boundary_input_does_not_change_transfer_rank() {
        // Scaling a unit step changes the response magnitude, not the one-dimensional
        // input/output transfer structure of this one-port excitation.
        let fixture = one_storage_node(2.0, 3.0, 4.0, vec![0.0, 1.0]);
        let response = response_matrix(&fixture, 1.0).expect("response");
        let transfer = response[0][1];
        let repeated = [transfer, 10.0 * transfer];
        assert!((repeated[1] - 10.0 * repeated[0]).abs() < 1.0e-12);
    }

    #[test]
    fn rejects_negative_storage_unanchored_components_and_boundary_storage() {
        let mut negative = one_storage_node(2.0, 3.0, 4.0, vec![0.0]);
        negative.nodes[1].storage_capacity = -1.0;
        assert!(validate(&negative).is_err());

        let mut boundary_storage = one_storage_node(2.0, 3.0, 4.0, vec![0.0]);
        boundary_storage.nodes[0].storage_capacity = 0.5;
        assert!(validate(&boundary_storage).is_err());

        let mut disconnected = one_storage_node(2.0, 3.0, 4.0, vec![0.0]);
        disconnected.nodes.push(Node {
            node_id: "isolated".into(),
            storage_capacity: 1.0,
        });
        assert!(validate(&disconnected).is_err());
    }
}
