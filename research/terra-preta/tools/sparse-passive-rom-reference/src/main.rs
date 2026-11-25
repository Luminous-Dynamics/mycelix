use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    fs::{self, OpenOptions},
    io::Write,
    process,
};

const VERSION: &str = "terra-sparse-passive-rom-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
type Dense = Vec<Vec<f64>>;

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Node {
    node_id: String,
    storage: Option<f64>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Edge {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
    conductance: f64,
}

#[derive(Clone, Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct AdaptiveOptions {
    candidate_shifts: Vec<f64>,
    target_max_relative_error: Option<f64>,
    maximum_basis_dimension: Option<usize>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    boundary_node_ids: Vec<String>,
    rational_shifts: Vec<f64>,
    laplace_samples: Vec<f64>,
    #[serde(default)]
    adaptive: AdaptiveOptions,
    cg_relative_tolerance: f64,
    cg_max_iterations: usize,
}

#[derive(Clone, Debug)]
struct CsrMatrix {
    size: usize,
    row_ptr: Vec<usize>,
    col_idx: Vec<usize>,
    values: Vec<f64>,
    diagonal: Vec<f64>,
}

impl CsrMatrix {
    fn from_rows(rows: Vec<BTreeMap<usize, f64>>) -> Result<Self, String> {
        let size = rows.len();
        let mut row_ptr = Vec::with_capacity(size + 1);
        let mut col_idx = Vec::new();
        let mut values = Vec::new();
        let mut diagonal = vec![0.0; size];
        row_ptr.push(0);
        for (row_index, row) in rows.into_iter().enumerate() {
            for (column, value) in row {
                if value.abs() > 0.0 {
                    if column == row_index {
                        diagonal[row_index] = value;
                    }
                    col_idx.push(column);
                    values.push(value);
                }
            }
            row_ptr.push(values.len());
        }
        if diagonal.iter().any(|value| !value.is_finite() || *value <= 0.0) {
            return Err("sparse interior operator must have a finite positive diagonal".into());
        }
        Ok(Self {
            size,
            row_ptr,
            col_idx,
            values,
            diagonal,
        })
    }

    fn nnz(&self) -> usize {
        self.values.len()
    }

    fn matvec(&self, vector: &[f64], shift: f64) -> Result<Vec<f64>, String> {
        if vector.len() != self.size || !shift.is_finite() || shift < 0.0 {
            return Err("sparse matrix-vector dimensions or shift are invalid".into());
        }
        let mut result = vec![0.0; self.size];
        for (row, value) in result.iter_mut().enumerate() {
            let mut sum = 0.0;
            for index in self.row_ptr[row]..self.row_ptr[row + 1] {
                sum += self.values[index] * vector[self.col_idx[index]];
            }
            *value = sum + shift * vector[row];
        }
        Ok(result)
    }

    fn shifted_diagonal(&self, shift: f64) -> Vec<f64> {
        self.diagonal.iter().map(|value| value + shift).collect()
    }

    fn validate_stieltjes(&self) -> Result<(), String> {
        for row in 0..self.size {
            if self.diagonal[row] <= 0.0 || !self.diagonal[row].is_finite() {
                return Err("M-matrix certificate requires a positive finite diagonal".into());
            }
            for index in self.row_ptr[row]..self.row_ptr[row + 1] {
                let column = self.col_idx[index];
                let value = self.values[index];
                if row != column && value > 0.0 {
                    return Err("M-matrix certificate rejected a positive off-diagonal".into());
                }
                let reverse = (self.row_ptr[column]..self.row_ptr[column + 1])
                    .find(|reverse_index| self.col_idx[*reverse_index] == row)
                    .map(|reverse_index| self.values[reverse_index])
                    .unwrap_or(0.0);
                if value != reverse {
                    return Err("M-matrix certificate requires an exactly symmetric sparse operator".into());
                }
            }
        }
        Ok(())
    }
}

#[derive(Clone)]
struct Model {
    boundary_ids: Vec<String>,
    interior_ids: Vec<String>,
    a: CsrMatrix,
    b: Dense,
    d: Dense,
}

#[derive(Clone, Copy)]
struct CgStats {
    iterations: usize,
    relative_residual: f64,
}

struct SparseResponseSolution {
    response: Dense,
    state_solutions: Vec<Vec<f64>>,
    stats: Vec<CgStats>,
}

struct Projection {
    a_r: Dense,
    b_r: Dense,
    d_r: Dense,
    basis: Vec<Vec<f64>>,
    basis_dimension: usize,
    basis_snapshots: usize,
    lambda_0_min_eigenvalue: f64,
    lambda_0_scale: f64,
    a_r_min_eigenvalue: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
struct SnapshotRequest {
    shift: f64,
    input_port_index: usize,
}

#[derive(Serialize)]
struct AdaptiveStep {
    selected_shift: f64,
    selected_input_port_index: usize,
    pre_addition_worst_relative_error: f64,
    basis_dimension_after_addition: usize,
}

struct AdaptiveProjectionResult {
    projection: Projection,
    stats: Vec<CgStats>,
    selected_shifts: Vec<f64>,
    selected_snapshots: Vec<SnapshotRequest>,
    selection_steps: Vec<AdaptiveStep>,
    target_met: bool,
    maximum_candidate_relative_error: f64,
    stop_reason: String,
}

#[derive(Serialize)]
struct SampleResult {
    laplace_s: f64,
    relative_response_error: f64,
    absolute_response_error: f64,
    absolute_response_error_upper_bound: f64,
    relative_response_error_upper_bound: f64,
    error_bound_valid: bool,
    full_cg_max_iterations: usize,
    full_cg_max_relative_residual: f64,
}

#[derive(Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    boundary_node_ids: Vec<String>,
    interior_node_count: usize,
    csr_nonzero_count: usize,
    csr_density: f64,
    rational_shifts: Vec<f64>,
    selected_port_snapshots: Vec<SnapshotRequest>,
    adaptive_selection_steps: Vec<AdaptiveStep>,
    adaptive_target_met: Option<bool>,
    adaptive_maximum_candidate_relative_error: Option<f64>,
    adaptive_stop_reason: String,
    block_arnoldi_baseline_basis_dimension: Option<usize>,
    block_arnoldi_baseline_maximum_sampled_response_error: Option<f64>,
    block_arnoldi_baseline_maximum_absolute_response_error_upper_bound: Option<f64>,
    block_arnoldi_baseline_maximum_relative_response_error_upper_bound: Option<f64>,
    block_arnoldi_baseline_error_bound_valid: Option<bool>,
    block_arnoldi_baseline_dc_relative_error: Option<f64>,
    block_arnoldi_baseline_feedthrough_limit_relative_error: Option<f64>,
    block_arnoldi_baseline_cg_solve_count: Option<usize>,
    block_arnoldi_baseline_total_cg_iterations: Option<usize>,
    block_arnoldi_baseline_passivity_certificate_valid: Option<bool>,
    basis_dimension: usize,
    basis_snapshots_attempted: usize,
    exact_dc_response: Dense,
    reduced_dc_response: Dense,
    dc_relative_error: f64,
    feedthrough_limit_relative_error: f64,
    maximum_sampled_response_error: f64,
    m_matrix_lambda_min_lower_bound: f64,
    m_matrix_supersolution_min_component: f64,
    m_matrix_supersolution_max_component: f64,
    m_matrix_supersolution_minimum_margin: f64,
    m_matrix_supersolution_sha256: String,
    m_matrix_supersolution_relative_residual: f64,
    error_certificate_valid: bool,
    error_certificate_pcg_solve_count: usize,
    error_certificate_pcg_iterations: usize,
    maximum_absolute_response_error_upper_bound: f64,
    maximum_relative_response_error_upper_bound: f64,
    overall_pcg_solve_count: usize,
    overall_pcg_iterations: usize,
    a_r_min_eigenvalue: f64,
    lambda_0_min_eigenvalue: f64,
    lambda_0_scale: f64,
    lambda_0_passivity_tolerance: f64,
    structural_passivity_certificate_valid: bool,
    cg_solve_count: usize,
    total_cg_iterations: usize,
    adaptive_candidate_shift_count: usize,
    maximum_cg_iterations: usize,
    maximum_cg_relative_residual: f64,
    frequency_samples: Vec<SampleResult>,
    interpretation_boundary: String,
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter().zip(right.iter()).map(|(a, b)| a * b).sum()
}

fn norm(vector: &[f64]) -> f64 {
    dot(vector, vector).sqrt()
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture identity, schema version or synthetic-only status is invalid".into());
    }
    if fixture.nodes.len() < 3 || fixture.edges.len() < 2 || fixture.boundary_node_ids.len() < 2 {
        return Err("fixture requires at least two boundary ports and one interior node".into());
    }
    if fixture.rational_shifts.is_empty() || fixture.laplace_samples.is_empty() {
        return Err("rational shifts and Laplace samples must be non-empty".into());
    }
    if !fixture.cg_relative_tolerance.is_finite()
        || fixture.cg_relative_tolerance <= 0.0
        || fixture.cg_relative_tolerance >= 1.0
        || fixture.cg_max_iterations == 0
    {
        return Err("CG tolerance and iteration limit are invalid".into());
    }

    let mut node_ids = BTreeSet::new();
    let boundary_ids: BTreeSet<&str> = fixture.boundary_node_ids.iter().map(String::as_str).collect();
    if boundary_ids.len() != fixture.boundary_node_ids.len() {
        return Err("boundary node IDs must be unique".into());
    }
    for node in &fixture.nodes {
        if node.node_id.trim().is_empty() || !node_ids.insert(node.node_id.as_str()) {
            return Err("node IDs must be non-empty and unique".into());
        }
        if boundary_ids.contains(node.node_id.as_str()) {
            if node.storage.is_some() {
                return Err(format!("boundary node {} must not carry interior storage", node.node_id));
            }
        } else if !node.storage.is_some_and(|value| value.is_finite() && value > 0.0) {
            return Err(format!("interior node {} needs finite positive storage", node.node_id));
        }
    }
    if boundary_ids.iter().any(|id| !node_ids.contains(id)) {
        return Err("boundary node IDs must reference known nodes".into());
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
            return Err(format!("edge {} has a self-loop or unknown endpoint", edge.edge_id));
        }
        if !edge.conductance.is_finite() || edge.conductance <= 0.0 {
            return Err(format!("edge {} conductance must be positive and finite", edge.edge_id));
        }
    }
    if fixture.rational_shifts.iter().any(|value| !value.is_finite() || *value < 0.0)
        || fixture.laplace_samples.iter().any(|value| !value.is_finite() || *value < 0.0)
    {
        return Err("all shifts and Laplace samples must be finite and nonnegative".into());
    }
    let adaptive_requested = !fixture.adaptive.candidate_shifts.is_empty()
        || fixture.adaptive.target_max_relative_error.is_some()
        || fixture.adaptive.maximum_basis_dimension.is_some();
    if adaptive_requested {
        let target = fixture
            .adaptive
            .target_max_relative_error
            .ok_or_else(|| "adaptive mode requires a target maximum relative error".to_string())?;
        let maximum_dimension = fixture
            .adaptive
            .maximum_basis_dimension
            .ok_or_else(|| "adaptive mode requires a maximum basis dimension".to_string())?;
        let interior_count = fixture.nodes.len() - boundary_ids.len();
        if fixture.adaptive.candidate_shifts.is_empty() {
            return Err("adaptive mode requires a non-empty candidate shift grid".into());
        }
        if !target.is_finite() || target <= 0.0 || target >= 1.0 {
            return Err("adaptive target error must lie strictly between zero and one".into());
        }
        if maximum_dimension == 0 || maximum_dimension > interior_count {
            return Err("adaptive maximum basis dimension must be within the interior state dimension".into());
        }
        for (index, shift) in fixture.adaptive.candidate_shifts.iter().enumerate() {
            if !shift.is_finite() || *shift < 0.0 {
                return Err("adaptive candidate shifts must be finite and nonnegative".into());
            }
            if fixture.rational_shifts.contains(shift)
                || fixture.laplace_samples.contains(shift)
                || fixture.adaptive.candidate_shifts[..index].contains(shift)
            {
                return Err("adaptive candidate shifts must be unique and disjoint from seed and audit shifts".into());
            }
        }
    }
    Ok(())
}

fn assemble(fixture: &Fixture) -> Result<Model, String> {
    validate(fixture)?;
    let boundary_set: BTreeSet<&str> = fixture.boundary_node_ids.iter().map(String::as_str).collect();
    let node_ids: Vec<String> = fixture.nodes.iter().map(|node| node.node_id.clone()).collect();
    let node_index: BTreeMap<&str, usize> = node_ids.iter().enumerate()
        .map(|(index, id)| (id.as_str(), index)).collect();
    let boundary_global: Vec<usize> = fixture.boundary_node_ids.iter()
        .map(|id| node_index[id.as_str()]).collect();
    let interior_nodes: Vec<&Node> = fixture.nodes.iter()
        .filter(|node| !boundary_set.contains(node.node_id.as_str())).collect();
    let interior_global: Vec<usize> = interior_nodes.iter()
        .map(|node| node_index[node.node_id.as_str()]).collect();
    let global_to_interior: BTreeMap<usize, usize> = interior_global.iter().enumerate()
        .map(|(index, global)| (*global, index)).collect();

    let mut adjacency = vec![Vec::<usize>::new(); node_ids.len()];
    let mut laplacian: Vec<BTreeMap<usize, f64>> = vec![BTreeMap::new(); node_ids.len()];
    for edge in &fixture.edges {
        let from = node_index[edge.from_node_id.as_str()];
        let to = node_index[edge.to_node_id.as_str()];
        let g = edge.conductance;
        adjacency[from].push(to);
        adjacency[to].push(from);
        *laplacian[from].entry(from).or_default() += g;
        *laplacian[to].entry(to).or_default() += g;
        *laplacian[from].entry(to).or_default() -= g;
        *laplacian[to].entry(from).or_default() -= g;
    }

    let mut visited = vec![false; node_ids.len()];
    for start in 0..node_ids.len() {
        if visited[start] {
            continue;
        }
        let mut stack = vec![start];
        visited[start] = true;
        let mut anchored = false;
        while let Some(node) = stack.pop() {
            anchored |= boundary_set.contains(node_ids[node].as_str());
            for neighbour in &adjacency[node] {
                if !visited[*neighbour] {
                    visited[*neighbour] = true;
                    stack.push(*neighbour);
                }
            }
        }
        if !anchored {
            return Err("an interior connected component is not anchored to any boundary port".into());
        }
    }

    let nb = boundary_global.len();
    let ni = interior_global.len();
    let mut rows: Vec<BTreeMap<usize, f64>> = vec![BTreeMap::new(); ni];
    let mut b = vec![vec![0.0; nb]; ni];
    let mut d = vec![vec![0.0; nb]; nb];

    for (interior_index, global) in interior_global.iter().enumerate() {
        let node_storage = interior_nodes[interior_index].storage.expect("validated storage");
        for (global_column, value) in &laplacian[*global] {
            if let Some(interior_column) = global_to_interior.get(global_column) {
                let column_storage = interior_nodes[*interior_column].storage.expect("validated storage");
                rows[interior_index].insert(
                    *interior_column,
                    *value / (node_storage * column_storage).sqrt(),
                );
            }
        }
        for (port, boundary_node) in boundary_global.iter().enumerate() {
            let coupling = laplacian[*global].get(boundary_node).copied().unwrap_or(0.0);
            b[interior_index][port] = coupling / node_storage.sqrt();
        }
    }
    for (row, global_row) in boundary_global.iter().enumerate() {
        for (column, global_column) in boundary_global.iter().enumerate() {
            d[row][column] = laplacian[*global_row].get(global_column).copied().unwrap_or(0.0);
        }
    }

    let a = CsrMatrix::from_rows(rows)?;
    Ok(Model {
        boundary_ids: fixture.boundary_node_ids.clone(),
        interior_ids: interior_nodes.iter().map(|node| node.node_id.clone()).collect(),
        a,
        b,
        d,
    })
}

fn pcg(
    matrix: &CsrMatrix,
    shift: f64,
    rhs: &[f64],
    relative_tolerance: f64,
    max_iterations: usize,
) -> Result<(Vec<f64>, CgStats), String> {
    if rhs.len() != matrix.size || !shift.is_finite() || shift < 0.0 {
        return Err("PCG right-hand side or shift is invalid".into());
    }
    let rhs_norm = norm(rhs);
    if rhs_norm <= f64::MIN_POSITIVE {
        return Ok((vec![0.0; matrix.size], CgStats {
            iterations: 0,
            relative_residual: 0.0,
        }));
    }
    let diagonal = matrix.shifted_diagonal(shift);
    let mut x = vec![0.0; matrix.size];
    let mut residual = rhs.to_vec();
    let mut z: Vec<f64> = residual.iter().zip(diagonal.iter())
        .map(|(r, d)| r / d).collect();
    let mut direction = z.clone();
    let mut rz = dot(&residual, &z);
    if !rz.is_finite() || rz <= 0.0 {
        return Err("PCG initial preconditioned residual is not positive and finite".into());
    }
    let mut relative_residual = norm(&residual) / rhs_norm;
    for iteration in 0..max_iterations {
        let product = matrix.matvec(&direction, shift)?;
        let denominator = dot(&direction, &product);
        if !denominator.is_finite() || denominator <= 0.0 {
            return Err("PCG detected a non-positive curvature direction".into());
        }
        let alpha = rz / denominator;
        for index in 0..matrix.size {
            x[index] += alpha * direction[index];
            residual[index] -= alpha * product[index];
        }
        let iterations = iteration + 1;
        relative_residual = norm(&residual) / rhs_norm;
        if relative_residual <= relative_tolerance {
            let action = matrix.matvec(&x, shift)?;
            let verified_residual: Vec<f64> = rhs
                .iter()
                .zip(action.iter())
                .map(|(right, left)| right - left)
                .collect();
            let verified_relative_residual = norm(&verified_residual) / rhs_norm;
            if verified_relative_residual <= relative_tolerance {
                return Ok((
                    x,
                    CgStats {
                        iterations,
                        relative_residual: verified_relative_residual,
                    },
                ));
            }
            residual = verified_residual;
            z = residual
                .iter()
                .zip(diagonal.iter())
                .map(|(r, d)| r / d)
                .collect();
            direction.clone_from(&z);
            rz = dot(&residual, &z);
            if !rz.is_finite() || rz <= 0.0 {
                return Err("PCG residual restart lost positive definiteness".into());
            }
            relative_residual = verified_relative_residual;
            continue;
        }
        z = residual.iter().zip(diagonal.iter()).map(|(r, d)| r / d).collect();
        let next_rz = dot(&residual, &z);
        if !next_rz.is_finite() || next_rz <= 0.0 {
            return Err("PCG preconditioned residual lost positive definiteness".into());
        }
        let beta = next_rz / rz;
        for index in 0..matrix.size {
            direction[index] = z[index] + beta * direction[index];
        }
        rz = next_rz;
    }

    Err(format!(
        "PCG failed to converge within {max_iterations} iterations (relative residual {relative_residual:e})"
    ))
}

struct MMatrixLowerBound {
    lambda_min_lower_bound: f64,
    supersolution_min_component: f64,
    supersolution_max_component: f64,
    minimum_supersolution_margin: f64,
    supersolution_sha256: String,
    cg_stats: CgStats,
}

fn m_matrix_lambda_min_lower_bound(
    matrix: &CsrMatrix,
    relative_tolerance: f64,
    max_iterations: usize,
) -> Result<MMatrixLowerBound, String> {
    matrix.validate_stieltjes()?;
    let rhs = vec![1.0; matrix.size];
    let (approximate_solution, cg_stats) =
        pcg(matrix, 0.0, &rhs, relative_tolerance, max_iterations)?;
    if approximate_solution
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err("M-matrix supersolution rejected non-positive PCG components".into());
    }

    let action = matrix.matvec(&approximate_solution, 0.0)?;
    if action.iter().any(|value| !value.is_finite() || *value <= 0.0) {
        return Err("M-matrix supersolution rejected non-positive A*y components".into());
    }

    // For a Stieltjes matrix, A^-1 is entrywise nonnegative. A positive y
    // satisfying A*y >= 1 is therefore a componentwise upper bound on A^-1*1.
    // Inflate once for floating-point margin, then verify the inequality again.
    let maximum_scale = action
        .iter()
        .map(|value| 1.0 / value)
        .fold(0.0_f64, f64::max);
    let mut scale = maximum_scale * (1.0 + 1.0e-10);
    let mut supersolution: Vec<f64> =
        approximate_solution.iter().map(|value| value * scale).collect();
    let mut verified_action = matrix.matvec(&supersolution, 0.0)?;
    for _ in 0..8 {
        if verified_action.iter().all(|value| value.is_finite() && *value >= 1.0) {
            break;
        }
        let correction = verified_action
            .iter()
            .map(|value| if *value > 0.0 { 1.0 / value } else { f64::INFINITY })
            .fold(1.0_f64, f64::max);
        if !correction.is_finite() {
            return Err("M-matrix supersolution could not be scaled to dominate one".into());
        }
        scale *= correction * (1.0 + 1.0e-10);
        for (value, original) in supersolution.iter_mut().zip(approximate_solution.iter()) {
            *value = *original * scale;
        }
        verified_action = matrix.matvec(&supersolution, 0.0)?;
    }
    if verified_action.iter().any(|value| !value.is_finite() || *value < 1.0) {
        return Err("M-matrix supersolution failed its final componentwise verification".into());
    }
    let supersolution_max_component = supersolution
        .iter()
        .copied()
        .reduce(f64::max)
        .ok_or_else(|| "cannot bound an empty M-matrix".to_string())?;
    if !supersolution_max_component.is_finite() || supersolution_max_component <= 0.0 {
        return Err("M-matrix supersolution maximum is invalid".into());
    }
    let lambda_min_lower_bound = 1.0 / supersolution_max_component;
    if !lambda_min_lower_bound.is_finite() || lambda_min_lower_bound <= 0.0 {
        return Err("M-matrix eigenvalue lower bound is invalid".into());
    }
    let supersolution_min_component = supersolution.iter().copied().reduce(f64::min)
        .ok_or_else(|| "cannot bound an empty M-matrix".to_string())?;
    let minimum_supersolution_margin = verified_action
        .iter()
        .copied()
        .reduce(f64::min)
        .ok_or_else(|| "cannot verify an empty M-matrix".to_string())? - 1.0;
    let mut digest = Sha256::new();
    digest.update(b"TERRA-MATRIX-SUPERSOLUTION-V1\\0");
    for value in &supersolution {
        digest.update(value.to_le_bytes());
    }
    let supersolution_sha256 = format!("{:x}", digest.finalize());

    Ok(MMatrixLowerBound {
        lambda_min_lower_bound,
        supersolution_min_component,
        supersolution_max_component,
        minimum_supersolution_margin,
        supersolution_sha256,
        cg_stats,
    })
}

fn frobenius_norm(matrix: &[Vec<f64>]) -> f64 {
    matrix
        .iter()
        .flatten()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt()
}

fn frobenius_difference(left: &[Vec<f64>], right: &[Vec<f64>]) -> Result<f64, String> {
    if left.len() != right.len()
        || left.iter().zip(right.iter()).any(|(a, b)| a.len() != b.len())
    {
        return Err("Frobenius difference requires equal matrix dimensions".into());
    }
    Ok(left
        .iter()
        .flatten()
        .zip(right.iter().flatten())
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt())
}

fn column(matrix: &[Vec<f64>], index: usize) -> Vec<f64> {
    matrix.iter().map(|row| row[index]).collect()
}

fn push_orthonormal_basis(basis: &mut Vec<Vec<f64>>, mut candidate: Vec<f64>) -> bool {
    let initial_norm = norm(&candidate);
    if !initial_norm.is_finite() || initial_norm <= f64::MIN_POSITIVE {
        return false;
    }
    for _ in 0..2 {
        for vector in basis.iter() {
            let projection = dot(&candidate, vector);
            for (value, basis_value) in candidate.iter_mut().zip(vector.iter()) {
                *value -= projection * basis_value;
            }
        }
    }
    let candidate_norm = norm(&candidate);
    if !candidate_norm.is_finite() || candidate_norm <= initial_norm * 1.0e-10 {
        return false;
    }
    for value in &mut candidate {
        *value /= candidate_norm;
    }
    basis.push(candidate);
    true
}

fn dense_solve(matrix: &[Vec<f64>], rhs: &[f64]) -> Result<Vec<f64>, String> {
    let n = matrix.len();
    if rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("dense solve dimensions are invalid".into());
    }
    let mut a = matrix.to_vec();
    let matrix_scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    let pivot_tolerance = matrix_scale.max(f64::MIN_POSITIVE) * 1.0e-14;
    let mut b = rhs.to_vec();
    for column in 0..n {
        let pivot = (column..n).max_by(|left, right| {
            a[*left][column].abs().total_cmp(&a[*right][column].abs())
        }).ok_or_else(|| "dense solve found no pivot".to_string())?;
        if a[pivot][column].abs() <= pivot_tolerance {
            return Err("reduced matrix is singular or unresolved".into());
        }
        a.swap(column, pivot);
        b.swap(column, pivot);
        let divisor = a[column][column];
        for value in a[column].iter_mut().skip(column) {
            *value /= divisor;
        }
        b[column] /= divisor;
        let pivot_row = a[column].clone();
        let pivot_rhs = b[column];
        for (row_index, row) in a.iter_mut().enumerate() {
            if row_index == column {
                continue;
            }
            let factor = row[column];
            for (target, pivot_value) in row.iter_mut().zip(pivot_row.iter()).skip(column) {
                *target -= factor * *pivot_value;
            }
            b[row_index] -= factor * pivot_rhs;
        }
    }
    if b.iter().any(|value| !value.is_finite()) {
        return Err("dense solve produced a non-finite result".into());
    }
    Ok(b)
}

fn transpose_product(basis: &[Vec<f64>], values: &[Vec<f64>]) -> Dense {
    let rows = basis.len();
    let columns = values.first().map_or(0, Vec::len);
    let mut result = vec![vec![0.0; columns]; rows];
    for (basis_index, vector) in basis.iter().enumerate() {
        for (value, column_index) in result[basis_index].iter_mut().zip(0..columns) {
            let value_column = column(values, column_index);
            *value = dot(vector, &value_column);
        }
    }
    result
}

fn multiply_transpose_left(left: &[Vec<f64>], right: &[Vec<f64>]) -> Dense {
    let mut result = vec![vec![0.0; right.len()]; left.len()];
    for (row, left_row) in left.iter().enumerate() {
        for (column_index, right_row) in right.iter().enumerate() {
            result[row][column_index] = dot(left_row, right_row);
        }
    }
    result
}

fn min_symmetric_eigenvalue(matrix: &[Vec<f64>]) -> Result<f64, String> {
    let n = matrix.len();
    if n == 0 || matrix.iter().any(|row| row.len() != n) {
        return Err("eigenvalue calculation requires a non-empty square matrix".into());
    }
    let mut a = matrix.to_vec();
    let scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    let symmetry_tolerance = scale.max(f64::MIN_POSITIVE) * 1.0e-10;
    for row in 0..n {
        for column_index in 0..n {
            if (a[row][column_index] - a[column_index][row]).abs() > symmetry_tolerance {
                return Err("eigenvalue calculation received a nonsymmetric matrix".into());
            }
        }
    }
    if scale == 0.0 {
        return Ok(0.0);
    }
    let tolerance = scale * 1.0e-13;
    for _ in 0..(n * n * 100).max(100) {
        let mut pivot = (0usize, 0usize);
        let mut largest = 0.0_f64;
        for (row, values) in a.iter().enumerate() {
            for (column_index, value) in values.iter().enumerate().skip(row + 1) {
                if value.abs() > largest {
                    largest = value.abs();
                    pivot = (row, column_index);
                }
            }
        }
        if largest <= tolerance {
            return a.iter().enumerate().map(|(index, row)| row[index])
                .reduce(f64::min).ok_or_else(|| "empty eigenvalue spectrum".into());
        }
        let (p, q) = pivot;
        let tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let tangent = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let cosine = 1.0 / (1.0 + tangent * tangent).sqrt();
        let sine = tangent * cosine;
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        a[p][p] = app - tangent * apq;
        a[q][q] = aqq + tangent * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for k in 0..n {
            if k != p && k != q {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = cosine * akp - sine * akq;
                a[p][k] = a[k][p];
                a[k][q] = sine * akp + cosine * akq;
                a[q][k] = a[k][q];
            }
        }
    }
    Err("symmetric Jacobi eigensolver did not converge".into())
}

fn response_and_state_solutions_from_sparse(
    model: &Model,
    shift: f64,
    tolerance: f64,
    max_iterations: usize,
) -> Result<SparseResponseSolution, String> {
    let nb = model.boundary_ids.len();
    let mut solution_columns = Vec::with_capacity(nb);
    let mut stats = Vec::with_capacity(nb);
    for port in 0..nb {
        let (solution, solve_stats) = pcg(
            &model.a,
            shift,
            &column(&model.b, port),
            tolerance,
            max_iterations,
        )?;
        solution_columns.push(solution);
        stats.push(solve_stats);
    }
    let mut response = model.d.clone();
    let b_columns: Vec<Vec<f64>> = (0..nb).map(|port| column(&model.b, port)).collect();
    for (row, response_row) in response.iter_mut().enumerate() {
        let b_row = &b_columns[row];
        for (response_value, solution_column) in response_row.iter_mut().zip(solution_columns.iter()) {
            let correction = dot(b_row, solution_column);
            *response_value -= correction;
        }
    }
    Ok(SparseResponseSolution {
        response,
        state_solutions: solution_columns,
        stats,
    })
}

fn response_from_sparse(
    model: &Model,
    shift: f64,
    tolerance: f64,
    max_iterations: usize,
) -> Result<(Dense, Vec<CgStats>), String> {
    let solved = response_and_state_solutions_from_sparse(
        model,
        shift,
        tolerance,
        max_iterations,
    )?;
    Ok((solved.response, solved.stats))
}

fn assemble_projection_from_basis(
    model: &Model,
    basis: Vec<Vec<f64>>,
    snapshot_count: usize,
    lambda_0: &Dense,
) -> Result<Projection, String> {
    if basis.is_empty() {
        return Err("reduced projection basis is numerically empty".into());
    }
    let nb = model.boundary_ids.len();
    let mut a_basis = Vec::with_capacity(basis.len());
    for vector in &basis {
        a_basis.push(model.a.matvec(vector, 0.0)?);
    }
    let a_r = multiply_transpose_left(&basis, &a_basis);
    let b_r = transpose_product(&basis, &model.b);
    let mut inverse_columns = Vec::with_capacity(nb);
    for port in 0..nb {
        inverse_columns.push(dense_solve(&a_r, &column(&b_r, port))?);
    }
    let b_columns: Vec<Vec<f64>> = (0..nb).map(|port| column(&b_r, port)).collect();
    let mut d_r = lambda_0.clone();
    for (row, d_row) in d_r.iter_mut().enumerate() {
        for (col, value) in d_row.iter_mut().enumerate() {
            *value += dot(&b_columns[row], &inverse_columns[col]);
        }
    }

    let a_r_min = min_symmetric_eigenvalue(&a_r)?;
    let lambda_0_min = min_symmetric_eigenvalue(lambda_0)?;
    let lambda_0_scale = lambda_0
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    Ok(Projection {
        a_r,
        b_r,
        d_r,
        basis: basis.clone(),
        basis_dimension: basis.len(),
        basis_snapshots: snapshot_count,
        lambda_0_min_eigenvalue: lambda_0_min,
        lambda_0_scale,
        a_r_min_eigenvalue: a_r_min,
    })
}

fn build_projection_from_basis(
    model: &Model,
    basis: Vec<Vec<f64>>,
    snapshot_count: usize,
    mut stats: Vec<CgStats>,
    fixture: &Fixture,
) -> Result<(Projection, Vec<CgStats>), String> {
    let (lambda_0, dc_stats) = response_from_sparse(
        model,
        0.0,
        fixture.cg_relative_tolerance,
        fixture.cg_max_iterations,
    )?;
    stats.extend(dc_stats);
    let projection =
        assemble_projection_from_basis(model, basis, snapshot_count, &lambda_0)?;
    Ok((projection, stats))
}

fn build_projection_for_requests(
    model: &Model,
    fixture: &Fixture,
    requests: &[SnapshotRequest],
) -> Result<(Projection, Vec<CgStats>), String> {
    let nb = model.boundary_ids.len();
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut stats = Vec::new();
    for request in requests {
        if request.input_port_index >= nb {
            return Err("snapshot request references an unknown boundary input port".into());
        }
        let (snapshot, solve_stats) = pcg(
            &model.a,
            request.shift,
            &column(&model.b, request.input_port_index),
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.push(solve_stats);
        push_orthonormal_basis(&mut basis, snapshot);
    }
    build_projection_from_basis(model, basis, requests.len(), stats, fixture)
}

fn requests_for_shifts(shifts: &[f64], port_count: usize) -> Vec<SnapshotRequest> {
    shifts
        .iter()
        .flat_map(|shift| {
            (0..port_count).map(|input_port_index| SnapshotRequest {
                shift: *shift,
                input_port_index,
            })
        })
        .collect()
}

fn build_projection_for_shifts(
    model: &Model,
    fixture: &Fixture,
    rational_shifts: &[f64],
) -> Result<(Projection, Vec<CgStats>), String> {
    let requests = requests_for_shifts(rational_shifts, model.boundary_ids.len());
    build_projection_for_requests(model, fixture, &requests)
}

fn build_projection(
    model: &Model,
    fixture: &Fixture,
) -> Result<(Projection, Vec<CgStats>), String> {
    build_projection_for_shifts(model, fixture, &fixture.rational_shifts)
}

/// PRIMA-style zero-shift block Krylov projection for the symmetric passive
/// conductance/storage realization. The seed block is A^-1 B; each subsequent
/// basis vector is generated by one more A^-1 application. The final model is
/// formed by congruence projection, preserving the positive-real structure.
fn build_zero_shift_block_arnoldi_projection(
    model: &Model,
    fixture: &Fixture,
    maximum_basis_dimension: usize,
) -> Result<(Projection, Vec<CgStats>), String> {
    let nb = model.boundary_ids.len();
    if maximum_basis_dimension == 0 || maximum_basis_dimension > model.a.size {
        return Err("block Krylov rank cap must be between one and the interior state dimension".into());
    }

    let mut basis = Vec::new();
    let mut stats = Vec::new();
    for port in 0..nb {
        let (snapshot, solve_stats) = pcg(
            &model.a,
            0.0,
            &column(&model.b, port),
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.push(solve_stats);
        push_orthonormal_basis(&mut basis, snapshot);
    }
    if basis.len() > maximum_basis_dimension {
        return Err(format!(
            "block Krylov seed rank {} exceeds requested cap {}",
            basis.len(), maximum_basis_dimension
        ));
    }

    let mut expansion_index = 0;
    let mut snapshot_count = basis.len();
    while expansion_index < basis.len() && basis.len() < maximum_basis_dimension {
        let direction = basis[expansion_index].clone();
        let (next_snapshot, solve_stats) = pcg(
            &model.a,
            0.0,
            &direction,
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.push(solve_stats);
        snapshot_count += 1;
        push_orthonormal_basis(&mut basis, next_snapshot);
        expansion_index += 1;
    }

    build_projection_from_basis(model, basis, snapshot_count, stats, fixture)
}

fn response_from_reduced(projection: &Projection, shift: f64) -> Result<Dense, String> {
    let nb = projection.d_r.len();
    let mut shifted = projection.a_r.clone();
    for (index, row) in shifted.iter_mut().enumerate() {
        row[index] += shift;
    }
    let mut solution_columns = Vec::with_capacity(nb);
    for port in 0..nb {
        solution_columns.push(dense_solve(&shifted, &column(&projection.b_r, port))?);
    }
    let b_columns: Vec<Vec<f64>> = (0..nb)
        .map(|port| column(&projection.b_r, port))
        .collect();
    let mut response = projection.d_r.clone();
    for (row, response_row) in response.iter_mut().enumerate() {
        let b_row = &b_columns[row];
        for (response_value, solution_column) in response_row.iter_mut().zip(solution_columns.iter()) {
            *response_value -= dot(b_row, solution_column);
        }
    }
    Ok(response)
}

fn reduced_state_in_full_space(
    model: &Model,
    projection: &Projection,
    shift: f64,
) -> Result<Dense, String> {
    let rank = projection.a_r.len();
    let port_count = projection.d_r.len();
    let mut shifted = projection.a_r.clone();
    for (index, row) in shifted.iter_mut().enumerate() {
        row[index] += shift;
    }
    let mut reduced_columns = Vec::with_capacity(port_count);
    for port in 0..port_count {
        reduced_columns.push(dense_solve(&shifted, &column(&projection.b_r, port))?);
    }
    let mut full_state = vec![vec![0.0; port_count]; model.a.size];
    for (mode, basis_vector) in projection.basis.iter().enumerate() {
        for (state, basis_value) in basis_vector.iter().enumerate() {
            for (port, reduced_column) in reduced_columns.iter().enumerate() {
                full_state[state][port] += basis_value * reduced_column[mode];
            }
        }
    }
    Ok(full_state)
}

fn response_error_upper_bound(
    model: &Model,
    projection: &Projection,
    shift: f64,
    lambda_min_lower_bound: f64,
    full_response: &[Vec<f64>],
    reduced_response: &[Vec<f64>],
) -> Result<(f64, f64, f64, bool), String> {
    if !lambda_min_lower_bound.is_finite() || lambda_min_lower_bound <= 0.0 {
        return Err("response error bound requires a positive verified eigenvalue lower bound".into());
    }
    if !shift.is_finite() || shift < 0.0 {
        return Err("response error bound only supports nonnegative real Laplace samples".into());
    }
    let full_state_approx = reduced_state_in_full_space(model, projection, shift)?;
    let mut residual_matrix = vec![vec![0.0; model.boundary_ids.len()]; model.a.size];
    for port in 0..model.boundary_ids.len() {
        let approximate_state = column(&full_state_approx, port);
        let action = model.a.matvec(&approximate_state, shift)?;
        let input = column(&model.b, port);
        for state in 0..model.a.size {
            residual_matrix[state][port] = input[state] - action[state];
        }
    }

    let residual_norm = frobenius_norm(&residual_matrix);
    let feedthrough_difference = frobenius_difference(&model.d, &projection.d_r)?;
    let b_norm = frobenius_norm(&model.b);
    let denominator = lambda_min_lower_bound + shift;
    let raw_bound = feedthrough_difference + b_norm * residual_norm / denominator;
    if !raw_bound.is_finite() {
        return Err("response error upper bound is non-finite".into());
    }
    // Explicit safety inflation for the floating-point implementation. This
    // is not interval arithmetic; the supersolution inequality is independently
    // recomputed and the resulting bound is checked against manufactured truth.
    let absolute_bound = raw_bound * (1.0 + 1.0e-10)
        + f64::EPSILON * (feedthrough_difference + b_norm * residual_norm + 1.0);
    let actual_absolute_error = frobenius_difference(full_response, reduced_response)?;
    let response_scale = frobenius_norm(full_response).max(f64::MIN_POSITIVE);
    let relative_bound = absolute_bound / response_scale;
    let valid = actual_absolute_error
        <= absolute_bound * (1.0 + 1.0e-8) + f64::EPSILON * response_scale * 32.0;
    Ok((absolute_bound, relative_bound, actual_absolute_error, valid))
}

fn relative_error(left: &[Vec<f64>], right: &[Vec<f64>]) -> f64 {
    let numerator = left.iter().flatten().zip(right.iter().flatten())
        .map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
    let denominator = right.iter().flatten().map(|value| value.powi(2))
        .sum::<f64>().sqrt().max(f64::MIN_POSITIVE);
    numerator / denominator
}

#[cfg(test)]
fn chain_fixture(interior_count: usize) -> Fixture {
    let mut nodes = vec![Node { node_id: "left".into(), storage: None }];
    for index in 0..interior_count {
        nodes.push(Node { node_id: format!("i-{index}"), storage: Some(1.0) });
    }
    nodes.push(Node { node_id: "right".into(), storage: None });
    let mut edges = Vec::with_capacity(interior_count + 1);
    for index in 0..=interior_count {
        let from_node_id = if index == 0 { "left".to_string() } else { format!("i-{}", index - 1) };
        let to_node_id = if index == interior_count { "right".to_string() } else { format!("i-{index}") };
        edges.push(Edge {
            edge_id: format!("e-{index}"),
            from_node_id,
            to_node_id,
            conductance: 1.0,
        });
    }
    Fixture {
        schema_version: VERSION.into(),
        fixture_status: FIXTURE_STATUS.into(),
        scenario_id: format!("synthetic-chain-interior-{interior_count}"),
        nodes,
        edges,
        boundary_node_ids: vec!["left".into(), "right".into()],
        rational_shifts: vec![0.0, 0.1, 1.0, 10.0],
        laplace_samples: vec![0.0, 0.1, 1.0, 10.0, 100.0],
        adaptive: AdaptiveOptions::default(),
        cg_relative_tolerance: 1.0e-10,
        cg_max_iterations: interior_count.saturating_mul(8).max(128),
    }
}

fn build_adaptive_projection(
    model: &Model,
    fixture: &Fixture,
) -> Result<AdaptiveProjectionResult, String> {
    let target = fixture
        .adaptive
        .target_max_relative_error
        .ok_or_else(|| "adaptive target error is missing".to_string())?;
    let maximum_basis_dimension = fixture
        .adaptive
        .maximum_basis_dimension
        .ok_or_else(|| "adaptive maximum basis dimension is missing".to_string())?;
    let mut candidates = fixture.adaptive.candidate_shifts.clone();
    candidates.sort_by(f64::total_cmp);

    // Cache both the full port response and the state solution columns. When
    // a candidate (frequency, input-port) pair is selected, its already
    // residual-verified state snapshot is inserted into the basis directly.
    let mut candidate_responses = Vec::with_capacity(candidates.len());
    let mut stats = Vec::new();
    for shift in candidates {
        let solved = response_and_state_solutions_from_sparse(
            model,
            shift,
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.extend(solved.stats);
        candidate_responses.push((shift, solved.response, solved.state_solutions));
    }

    // The exact DC Schur operator is independent of the growing Krylov
    // basis. Compute it once and reuse it for every reduced projection.
    let (lambda_0, dc_stats) = response_from_sparse(
        model,
        0.0,
        fixture.cg_relative_tolerance,
        fixture.cg_max_iterations,
    )?;
    stats.extend(dc_stats);

    let mut selected_shifts = fixture.rational_shifts.clone();
    let mut selected_snapshots = requests_for_shifts(&selected_shifts, model.boundary_ids.len());
    let mut basis = Vec::new();
    for request in &selected_snapshots {
        let (snapshot, solve_stats) = pcg(
            &model.a,
            request.shift,
            &column(&model.b, request.input_port_index),
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.push(solve_stats);
        push_orthonormal_basis(&mut basis, snapshot);
    }
    let mut projection = assemble_projection_from_basis(
        model,
        basis.clone(),
        selected_snapshots.len(),
        &lambda_0,
    )?;
    if projection.basis_dimension > maximum_basis_dimension {
        return Err(format!(
            "seed basis dimension {} exceeds adaptive cap {}",
            projection.basis_dimension, maximum_basis_dimension
        ));
    }

    let mut selection_steps = Vec::new();
    let mut target_met = false;
    let stop_reason;

    loop {
        let mut maximum_candidate_error = 0.0_f64;
        let mut worst_unselected: Option<(f64, f64, usize)> = None;
        for (shift, full_response, _) in &candidate_responses {
            let reduced_response = response_from_reduced(&projection, *shift)?;
            for port in 0..model.boundary_ids.len() {
                let full_column = column(full_response, port);
                let reduced_column = column(&reduced_response, port);
                let error = relative_error(
                    std::slice::from_ref(&reduced_column),
                    std::slice::from_ref(&full_column),
                );
                maximum_candidate_error = maximum_candidate_error.max(error);
                let already_selected = selected_snapshots.iter().any(|request| {
                    request.shift == *shift && request.input_port_index == port
                });
                if !already_selected
                    && worst_unselected.is_none_or(|(best_error, _, _)| error > best_error)
                {
                    worst_unselected = Some((error, *shift, port));
                }
            }
        }

        if maximum_candidate_error <= target {
            target_met = true;
            stop_reason = "target_met".to_string();
            break;
        }
        if projection.basis_dimension >= maximum_basis_dimension {
            stop_reason = "basis_dimension_cap".to_string();
            break;
        }
        let Some((worst_error, selected_shift, selected_port)) = worst_unselected else {
            stop_reason = "candidate_grid_exhausted".to_string();
            break;
        };

        let cached_snapshot = candidate_responses
            .iter()
            .find(|(shift, _, _)| *shift == selected_shift)
            .and_then(|(_, _, solutions)| solutions.get(selected_port))
            .cloned()
            .ok_or_else(|| "selected adaptive snapshot is absent from candidate cache".to_string())?;
        push_orthonormal_basis(&mut basis, cached_snapshot);

        let mut next_snapshots = selected_snapshots.clone();
        next_snapshots.push(SnapshotRequest {
            shift: selected_shift,
            input_port_index: selected_port,
        });
        let next_projection = assemble_projection_from_basis(
            model,
            basis.clone(),
            next_snapshots.len(),
            &lambda_0,
        )?;
        if next_projection.basis_dimension > maximum_basis_dimension {
            stop_reason = "basis_dimension_cap".to_string();
            break;
        }

        selection_steps.push(AdaptiveStep {
            selected_shift,
            selected_input_port_index: selected_port,
            pre_addition_worst_relative_error: maximum_candidate_error.max(worst_error),
            basis_dimension_after_addition: next_projection.basis_dimension,
        });
        if !selected_shifts.contains(&selected_shift) {
            selected_shifts.push(selected_shift);
            selected_shifts.sort_by(f64::total_cmp);
        }
        selected_snapshots = next_snapshots;
        projection = next_projection;
    }

    let mut maximum_candidate_relative_error = 0.0_f64;
    for (shift, full_response, _) in &candidate_responses {
        let reduced_response = response_from_reduced(&projection, *shift)?;
        for port in 0..model.boundary_ids.len() {
            let full_column = column(full_response, port);
            let reduced_column = column(&reduced_response, port);
            maximum_candidate_relative_error = maximum_candidate_relative_error.max(relative_error(
                std::slice::from_ref(&reduced_column),
                std::slice::from_ref(&full_column),
            ));
        }
    }

    Ok(AdaptiveProjectionResult {
        projection,
        stats,
        selected_shifts,
        selected_snapshots,
        selection_steps,
        target_met,
        maximum_candidate_relative_error,
        stop_reason,
    })
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    let model = assemble(fixture)?;
    let adaptive_requested = !fixture.adaptive.candidate_shifts.is_empty();
    let (
        projection,
        mut stats,
        effective_shifts,
        selected_port_snapshots,
        adaptive_selection_steps,
        adaptive_target_met,
        adaptive_maximum_candidate_relative_error,
        adaptive_stop_reason,
    ) = if adaptive_requested {
        let adaptive = build_adaptive_projection(&model, fixture)?;
        (
            adaptive.projection,
            adaptive.stats,
            adaptive.selected_shifts,
            adaptive.selected_snapshots,
            adaptive.selection_steps,
            Some(adaptive.target_met),
            Some(adaptive.maximum_candidate_relative_error),
            adaptive.stop_reason,
        )
    } else {
        let (projection, stats) = build_projection(&model, fixture)?;
        (
            projection,
            stats,
            fixture.rational_shifts.clone(),
            requests_for_shifts(&fixture.rational_shifts, model.boundary_ids.len()),
            Vec::new(),
            None,
            None,
            "fixed_shift_mode".to_string(),
        )
    };
    let lambda_0_passivity_tolerance =
        projection.lambda_0_scale.max(f64::MIN_POSITIVE) * 1.0e-10;
    if projection.a_r_min_eigenvalue <= 0.0
        || projection.lambda_0_min_eigenvalue < -lambda_0_passivity_tolerance
    {
        return Err("reduced model failed the scale-aware structural passivity checks".into());
    }
    let block_arnoldi_baseline = if adaptive_requested {
        Some(build_zero_shift_block_arnoldi_projection(
            &model,
            fixture,
            projection.basis_dimension,
        )?)
    } else {
        None
    };
    let error_certificate = m_matrix_lambda_min_lower_bound(
        &model.a,
        fixture.cg_relative_tolerance,
        fixture.cg_max_iterations,
    )?;
    let mut block_arnoldi_baseline_error = 0.0_f64;
    let mut block_arnoldi_baseline_absolute_error_bound = 0.0_f64;
    let mut block_arnoldi_baseline_relative_error_bound = 0.0_f64;
    let mut block_arnoldi_baseline_error_bound_valid = true;
    let mut exact_dc_from_audit: Option<Dense> = None;
    let mut frequency_samples = Vec::with_capacity(fixture.laplace_samples.len());
    for shift in &fixture.laplace_samples {
        let (full, full_stats) = response_from_sparse(
            &model,
            *shift,
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.extend(full_stats.iter().copied());
        if *shift == 0.0 {
            exact_dc_from_audit = Some(full.clone());
        }
        let reduced = response_from_reduced(&projection, *shift)?;
        let (absolute_response_error_upper_bound, relative_response_error_upper_bound,
            absolute_response_error, error_bound_valid) = response_error_upper_bound(
                &model,
                &projection,
                *shift,
                error_certificate.lambda_min_lower_bound,
                &full,
                &reduced,
            )?;
        if !error_bound_valid {
            return Err(format!(
                "residual response-error bound failed to dominate observed error at s={shift}"
            ));
        }
        if let Some((block_projection, _)) = block_arnoldi_baseline.as_ref() {
            let block_reduced = response_from_reduced(block_projection, *shift)?;
            block_arnoldi_baseline_error = block_arnoldi_baseline_error
                .max(relative_error(&block_reduced, &full));
            let (
                block_absolute_bound,
                block_relative_bound,
                _block_absolute_error,
                block_bound_valid,
            ) = response_error_upper_bound(
                &model,
                block_projection,
                *shift,
                error_certificate.lambda_min_lower_bound,
                &full,
                &block_reduced,
            )?;
            block_arnoldi_baseline_absolute_error_bound =
                block_arnoldi_baseline_absolute_error_bound.max(block_absolute_bound);
            block_arnoldi_baseline_relative_error_bound =
                block_arnoldi_baseline_relative_error_bound.max(block_relative_bound);
            block_arnoldi_baseline_error_bound_valid &= block_bound_valid;
        }
        frequency_samples.push(SampleResult {
            laplace_s: *shift,
            relative_response_error: relative_error(&reduced, &full),
            absolute_response_error,
            absolute_response_error_upper_bound,
            relative_response_error_upper_bound,
            error_bound_valid,
            full_cg_max_iterations: full_stats.iter().map(|item| item.iterations).max().unwrap_or(0),
            full_cg_max_relative_residual: full_stats.iter()
                .map(|item| item.relative_residual).fold(0.0_f64, f64::max),
        });
    }
    let exact_dc = if let Some(response) = exact_dc_from_audit {
        response
    } else {
        let (response, exact_dc_stats) = response_from_sparse(
            &model,
            0.0,
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.extend(exact_dc_stats);
        response
    };
    let reduced_dc = response_from_reduced(&projection, 0.0)?;
    let feedthrough_limit_relative_error = relative_error(&projection.d_r, &model.d);
    let maximum_sampled_response_error = frequency_samples
        .iter()
        .map(|sample| sample.relative_response_error)
        .fold(0.0_f64, f64::max);
    let cg_solve_count = stats.len();
    let total_cg_iterations = stats.iter().map(|item| item.iterations).sum();
    let maximum_absolute_response_error_upper_bound = frequency_samples
        .iter()
        .map(|sample| sample.absolute_response_error_upper_bound)
        .fold(0.0_f64, f64::max);
    let maximum_relative_response_error_upper_bound = frequency_samples
        .iter()
        .map(|sample| sample.relative_response_error_upper_bound)
        .fold(0.0_f64, f64::max);
    let error_certificate_valid = frequency_samples.iter().all(|sample| sample.error_bound_valid)
        && error_certificate.lambda_min_lower_bound > 0.0;
    let error_certificate_pcg_solve_count = 1usize;
    let error_certificate_pcg_iterations = error_certificate.cg_stats.iterations;
    let overall_pcg_solve_count = cg_solve_count + error_certificate_pcg_solve_count;
    let overall_pcg_iterations = total_cg_iterations + error_certificate_pcg_iterations;
    let adaptive_candidate_shift_count = fixture.adaptive.candidate_shifts.len();
    let maximum_cg_iterations = stats.iter().map(|item| item.iterations).max().unwrap_or(0);
    let maximum_cg_relative_residual = stats.iter()
        .map(|item| item.relative_residual).fold(0.0_f64, f64::max);
    let csr_density = model.a.nnz() as f64 / (model.a.size * model.a.size) as f64;
    let dc_relative_error = relative_error(&reduced_dc, &exact_dc);
    let block_arnoldi_baseline_dc_relative_error = if let Some((block_projection, _)) =
        block_arnoldi_baseline.as_ref()
    {
        Some(relative_error(&response_from_reduced(block_projection, 0.0)?, &exact_dc))
    } else {
        None
    };
    let block_arnoldi_baseline_feedthrough_limit_relative_error = block_arnoldi_baseline
        .as_ref()
        .map(|(block_projection, _)| relative_error(&block_projection.d_r, &model.d));
    let block_arnoldi_baseline_metrics = block_arnoldi_baseline.as_ref().map(
        |(block_projection, block_stats)| {
            let block_passivity_tolerance =
                block_projection.lambda_0_scale.max(f64::MIN_POSITIVE) * 1.0e-10;
            (
                block_projection.basis_dimension,
                block_arnoldi_baseline_error,
                block_stats.len(),
                block_stats.iter().map(|item| item.iterations).sum::<usize>(),
                block_projection.a_r_min_eigenvalue > 0.0
                    && block_projection.lambda_0_min_eigenvalue >= -block_passivity_tolerance,
            )
        },
    );

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        boundary_node_ids: model.boundary_ids.clone(),
        interior_node_count: model.interior_ids.len(),
        csr_nonzero_count: model.a.nnz(),
        csr_density,
        rational_shifts: effective_shifts,
        selected_port_snapshots,
        adaptive_selection_steps,
        adaptive_target_met,
        adaptive_maximum_candidate_relative_error,
        adaptive_stop_reason,
        block_arnoldi_baseline_basis_dimension: block_arnoldi_baseline_metrics
            .map(|metrics| metrics.0),
        block_arnoldi_baseline_maximum_sampled_response_error: block_arnoldi_baseline_metrics
            .map(|metrics| metrics.1),
        block_arnoldi_baseline_maximum_absolute_response_error_upper_bound: block_arnoldi_baseline
            .as_ref().map(|_| block_arnoldi_baseline_absolute_error_bound),
        block_arnoldi_baseline_maximum_relative_response_error_upper_bound: block_arnoldi_baseline
            .as_ref().map(|_| block_arnoldi_baseline_relative_error_bound),
        block_arnoldi_baseline_error_bound_valid: block_arnoldi_baseline
            .as_ref().map(|_| block_arnoldi_baseline_error_bound_valid),
        block_arnoldi_baseline_dc_relative_error,
        block_arnoldi_baseline_feedthrough_limit_relative_error,
        block_arnoldi_baseline_cg_solve_count: block_arnoldi_baseline_metrics
            .map(|metrics| metrics.2),
        block_arnoldi_baseline_total_cg_iterations: block_arnoldi_baseline_metrics
            .map(|metrics| metrics.3),
        block_arnoldi_baseline_passivity_certificate_valid: block_arnoldi_baseline_metrics
            .map(|metrics| metrics.4),
        basis_dimension: projection.basis_dimension,
        basis_snapshots_attempted: projection.basis_snapshots,
        exact_dc_response: exact_dc,
        reduced_dc_response: reduced_dc.clone(),
        dc_relative_error,
        feedthrough_limit_relative_error,
        maximum_sampled_response_error,
        m_matrix_lambda_min_lower_bound: error_certificate.lambda_min_lower_bound,
        m_matrix_supersolution_min_component: error_certificate.supersolution_min_component,
        m_matrix_supersolution_max_component: error_certificate.supersolution_max_component,
        m_matrix_supersolution_minimum_margin: error_certificate.minimum_supersolution_margin,
        m_matrix_supersolution_sha256: error_certificate.supersolution_sha256,
        m_matrix_supersolution_relative_residual: error_certificate.cg_stats.relative_residual,
        error_certificate_valid,
        error_certificate_pcg_solve_count,
        error_certificate_pcg_iterations,
        maximum_absolute_response_error_upper_bound,
        maximum_relative_response_error_upper_bound,
        overall_pcg_solve_count,
        overall_pcg_iterations,
        a_r_min_eigenvalue: projection.a_r_min_eigenvalue,
        lambda_0_min_eigenvalue: projection.lambda_0_min_eigenvalue,
        lambda_0_scale: projection.lambda_0_scale,
        lambda_0_passivity_tolerance,
        structural_passivity_certificate_valid: projection.a_r_min_eigenvalue > 0.0
            && projection.lambda_0_min_eigenvalue >= -lambda_0_passivity_tolerance,
        cg_solve_count,
        total_cg_iterations,
        adaptive_candidate_shift_count,
        maximum_cg_iterations,
        maximum_cg_relative_residual,
        frequency_samples,
        interpretation_boundary: "Sparse CSR and PCG reference for fixed positive linear conductance graphs with positive diagonal interior storage and ideal Dirichlet ports. The projected model uses port-driven rational-shift snapshots and exact numerical DC correction. Structural passivity is tolerance-qualified through positive A_r and positive-semidefinite Lambda_0. This is not a nonlinear Richards, hysteretic, multiphase or empirical soil model. No speedup or production-scale claim is made.".into(),
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} INPUT.json OUTPUT.json",
            args.first().map(String::as_str).unwrap_or("terra-sparse-passive-rom-reference")
        ));
    }
    let input = fs::read(&args[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let digest = format!("{:x}", Sha256::digest(&input));
    let fixture: Fixture = serde_json::from_slice(&input)
        .map_err(|error| format!("invalid fixture JSON: {error}"))?;
    let report = analyze(&fixture, digest)?;
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report: {error}"))?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(&args[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    file.write_all(&output).map_err(|error| format!("cannot write output: {error}"))?;
    file.write_all(b"\n").map_err(|error| format!("cannot finalize output: {error}"))?;
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

    const TOL: f64 = 1.0e-10;

    fn two_by_two_stieltjes(scale: f64) -> CsrMatrix {
        CsrMatrix::from_rows(vec![
            BTreeMap::from([(0, 2.0 * scale), (1, -scale)]),
            BTreeMap::from([(0, -scale), (1, 2.0 * scale)]),
        ])
        .unwrap()
    }

    #[test]
    fn m_matrix_supersolution_bounds_analytic_two_by_two_eigenvalue() {
        let matrix = two_by_two_stieltjes(1.0);
        let certificate = m_matrix_lambda_min_lower_bound(&matrix, 1.0e-12, 32).unwrap();
        // Eigenvalues of [[2,-1],[-1,2]] are 1 and 3. The exact A^-1*1
        // is [1,1], so the supersolution-derived lower bound is tight up to
        // the declared floating-point inflation.
        assert!(certificate.lambda_min_lower_bound > 0.999_999);
        assert!(certificate.lambda_min_lower_bound <= 1.0);
        assert!(certificate.supersolution_min_component > 0.0);
        assert!(certificate.supersolution_max_component >= 1.0);
        assert!(certificate.minimum_supersolution_margin >= 0.0);
        assert!(certificate.cg_stats.relative_residual <= 1.0e-12);
        assert_eq!(certificate.supersolution_sha256.len(), 64);
    }

    #[test]
    fn m_matrix_lower_bound_is_invariant_under_24_order_unit_rescaling() {
        let baseline = m_matrix_lambda_min_lower_bound(
            &two_by_two_stieltjes(1.0),
            1.0e-11,
            32,
        )
        .unwrap();
        for scale in [1.0e-24, 1.0e24] {
            let certificate = m_matrix_lambda_min_lower_bound(
                &two_by_two_stieltjes(scale),
                1.0e-11,
                32,
            )
            .unwrap();
            let normalized = certificate.lambda_min_lower_bound / scale;
            assert!((normalized - baseline.lambda_min_lower_bound).abs() < 1.0e-8);
            assert!(certificate.minimum_supersolution_margin >= 0.0);
        }
    }

    #[test]
    fn m_matrix_certificate_rejects_positive_or_asymmetric_off_diagonals() {
        let positive_off_diagonal = CsrMatrix::from_rows(vec![
            BTreeMap::from([(0, 2.0), (1, 0.1)]),
            BTreeMap::from([(0, 0.1), (1, 2.0)]),
        ])
        .unwrap();
        assert!(m_matrix_lambda_min_lower_bound(&positive_off_diagonal, 1.0e-10, 32).is_err());

        let asymmetric = CsrMatrix::from_rows(vec![
            BTreeMap::from([(0, 2.0), (1, -1.0)]),
            BTreeMap::from([(0, -0.5), (1, 2.0)]),
        ])
        .unwrap();
        assert!(m_matrix_lambda_min_lower_bound(&asymmetric, 1.0e-10, 32).is_err());
    }

    #[test]
    fn response_error_upper_bounds_dominate_every_adaptive_audit_sample() {
        let fixture = adaptive_multiport_fixture();
        let report = analyze(&fixture, "m-matrix-response-certificate-test".into()).unwrap();
        assert!(report.error_certificate_valid);
        assert!(report.m_matrix_lambda_min_lower_bound > 0.0);
        assert!(report.m_matrix_supersolution_min_component > 0.0);
        assert!(report.m_matrix_supersolution_minimum_margin >= 0.0);
        assert!(report.m_matrix_supersolution_relative_residual <= fixture.cg_relative_tolerance);
        assert_eq!(report.m_matrix_supersolution_sha256.len(), 64);
        assert_eq!(report.error_certificate_pcg_solve_count, 1);
        assert!(report.overall_pcg_solve_count > report.cg_solve_count);
        assert!(report.maximum_relative_response_error_upper_bound.is_finite());
        for sample in &report.frequency_samples {
            assert!(sample.error_bound_valid);
            assert!(
                sample.absolute_response_error_upper_bound >= sample.absolute_response_error,
                "upper bound {:?} must dominate observed error {:?} at s={}",
                sample.absolute_response_error_upper_bound,
                sample.absolute_response_error,
                sample.laplace_s
            );
            assert!(
                sample.relative_response_error_upper_bound >= sample.relative_response_error,
                "relative upper bound {:?} must dominate observed error {:?} at s={}",
                sample.relative_response_error_upper_bound,
                sample.relative_response_error,
                sample.laplace_s
            );
        }
    }

    fn high_contrast_multiport_fixture() -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "high-contrast-four-interior-three-boundary-port".into(),
            nodes: vec![
                Node { node_id: "left".into(), storage: None },
                Node { node_id: "right".into(), storage: None },
                Node { node_id: "top".into(), storage: None },
                Node { node_id: "a".into(), storage: Some(1.0e-3) },
                Node { node_id: "b".into(), storage: Some(2.0e-1) },
                Node { node_id: "c".into(), storage: Some(3.0) },
                Node { node_id: "d".into(), storage: Some(1.0e2) },
            ],
            edges: vec![
                Edge { edge_id: "left-a".into(), from_node_id: "left".into(), to_node_id: "a".into(), conductance: 1.0e-4 },
                Edge { edge_id: "left-b".into(), from_node_id: "left".into(), to_node_id: "b".into(), conductance: 1.0e-3 },
                Edge { edge_id: "a-b".into(), from_node_id: "a".into(), to_node_id: "b".into(), conductance: 5.0e1 },
                Edge { edge_id: "a-c".into(), from_node_id: "a".into(), to_node_id: "c".into(), conductance: 1.0e-3 },
                Edge { edge_id: "b-c".into(), from_node_id: "b".into(), to_node_id: "c".into(), conductance: 1.0e2 },
                Edge { edge_id: "b-d".into(), from_node_id: "b".into(), to_node_id: "d".into(), conductance: 5.0e-1 },
                Edge { edge_id: "c-d".into(), from_node_id: "c".into(), to_node_id: "d".into(), conductance: 1.0e4 },
                Edge { edge_id: "c-right".into(), from_node_id: "c".into(), to_node_id: "right".into(), conductance: 1.0e2 },
                Edge { edge_id: "d-right".into(), from_node_id: "d".into(), to_node_id: "right".into(), conductance: 1.0e1 },
                Edge { edge_id: "top-b".into(), from_node_id: "top".into(), to_node_id: "b".into(), conductance: 1.0e-1 },
                Edge { edge_id: "top-d".into(), from_node_id: "top".into(), to_node_id: "d".into(), conductance: 2.0 },
            ],
            boundary_node_ids: vec!["left".into(), "right".into(), "top".into()],
            rational_shifts: vec![0.0, 1.0e-4, 1.0e-2, 1.0, 1.0e2, 1.0e4],
            laplace_samples: vec![0.0, 1.0e-4, 1.0e-2, 1.0, 1.0e2, 1.0e4],
            adaptive: AdaptiveOptions::default(),
            cg_relative_tolerance: 1.0e-11,
            cg_max_iterations: 1024,
        }
    }

    fn fixture() -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "two-interior-node-series-chain".into(),
            nodes: vec![
                Node { node_id: "left".into(), storage: None },
                Node { node_id: "interior-a".into(), storage: Some(1.0) },
                Node { node_id: "interior-b".into(), storage: Some(2.0) },
                Node { node_id: "right".into(), storage: None },
            ],
            edges: vec![
                Edge { edge_id: "left-a".into(), from_node_id: "left".into(), to_node_id: "interior-a".into(), conductance: 2.0 },
                Edge { edge_id: "a-b".into(), from_node_id: "interior-a".into(), to_node_id: "interior-b".into(), conductance: 3.0 },
                Edge { edge_id: "b-right".into(), from_node_id: "interior-b".into(), to_node_id: "right".into(), conductance: 4.0 },
            ],
            boundary_node_ids: vec!["left".into(), "right".into()],
            rational_shifts: vec![0.0, 0.1, 1.0, 10.0],
            laplace_samples: vec![0.0, 0.1, 1.0, 10.0, 100.0],
            adaptive: AdaptiveOptions::default(),
            cg_relative_tolerance: 1.0e-11,
            cg_max_iterations: 512,
        }
    }

    fn branched_fixture() -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "four-interior-node-cross-linked-parallel-pathways".into(),
            nodes: vec![
                Node { node_id: "left".into(), storage: None },
                Node { node_id: "p1".into(), storage: Some(1.0) },
                Node { node_id: "p2".into(), storage: Some(1.7) },
                Node { node_id: "p3".into(), storage: Some(0.8) },
                Node { node_id: "p4".into(), storage: Some(2.2) },
                Node { node_id: "right".into(), storage: None },
            ],
            edges: vec![
                Edge { edge_id: "left-p1".into(), from_node_id: "left".into(), to_node_id: "p1".into(), conductance: 2.0 },
                Edge { edge_id: "p1-right".into(), from_node_id: "p1".into(), to_node_id: "right".into(), conductance: 1.3 },
                Edge { edge_id: "left-p2".into(), from_node_id: "left".into(), to_node_id: "p2".into(), conductance: 0.8 },
                Edge { edge_id: "p2-p3".into(), from_node_id: "p2".into(), to_node_id: "p3".into(), conductance: 2.1 },
                Edge { edge_id: "p3-right".into(), from_node_id: "p3".into(), to_node_id: "right".into(), conductance: 1.7 },
                Edge { edge_id: "p1-p4".into(), from_node_id: "p1".into(), to_node_id: "p4".into(), conductance: 0.7 },
                Edge { edge_id: "p4-p3".into(), from_node_id: "p4".into(), to_node_id: "p3".into(), conductance: 1.1 },
                Edge { edge_id: "p2-p4".into(), from_node_id: "p2".into(), to_node_id: "p4".into(), conductance: 0.4 },
            ],
            boundary_node_ids: vec!["left".into(), "right".into()],
            rational_shifts: vec![0.0, 0.01, 0.1, 1.0, 10.0, 100.0],
            laplace_samples: vec![0.0, 0.01, 0.1, 1.0, 10.0, 100.0],
            adaptive: AdaptiveOptions::default(),
            cg_relative_tolerance: 1.0e-11,
            cg_max_iterations: 512,
        }
    }

    fn adaptive_multiport_fixture() -> Fixture {
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "eight-interior-three-port-adaptive-rom".into(),
            nodes: vec![
                Node { node_id: "left".into(), storage: None },
                Node { node_id: "right".into(), storage: None },
                Node { node_id: "top".into(), storage: None },
                Node { node_id: "n0".into(), storage: Some(1.0e-3) },
                Node { node_id: "n1".into(), storage: Some(1.0e-2) },
                Node { node_id: "n2".into(), storage: Some(1.0e-1) },
                Node { node_id: "n3".into(), storage: Some(1.0) },
                Node { node_id: "n4".into(), storage: Some(1.0e1) },
                Node { node_id: "n5".into(), storage: Some(1.0e2) },
                Node { node_id: "n6".into(), storage: Some(1.0e3) },
                Node { node_id: "n7".into(), storage: Some(1.0e4) },
            ],
            edges: vec![
                Edge { edge_id: "left-n0".into(), from_node_id: "left".into(), to_node_id: "n0".into(), conductance: 1.0e-2 },
                Edge { edge_id: "left-n1".into(), from_node_id: "left".into(), to_node_id: "n1".into(), conductance: 1.0e-1 },
                Edge { edge_id: "left-n2".into(), from_node_id: "left".into(), to_node_id: "n2".into(), conductance: 2.0 },
                Edge { edge_id: "n0-n1".into(), from_node_id: "n0".into(), to_node_id: "n1".into(), conductance: 1.0e1 },
                Edge { edge_id: "n1-n2".into(), from_node_id: "n1".into(), to_node_id: "n2".into(), conductance: 5.0e-1 },
                Edge { edge_id: "n0-n3".into(), from_node_id: "n0".into(), to_node_id: "n3".into(), conductance: 1.0e-2 },
                Edge { edge_id: "n1-n3".into(), from_node_id: "n1".into(), to_node_id: "n3".into(), conductance: 1.0e1 },
                Edge { edge_id: "n2-n3".into(), from_node_id: "n2".into(), to_node_id: "n3".into(), conductance: 5.0 },
                Edge { edge_id: "n2-n4".into(), from_node_id: "n2".into(), to_node_id: "n4".into(), conductance: 2.0e-2 },
                Edge { edge_id: "n3-n4".into(), from_node_id: "n3".into(), to_node_id: "n4".into(), conductance: 1.0e1 },
                Edge { edge_id: "n3-n5".into(), from_node_id: "n3".into(), to_node_id: "n5".into(), conductance: 1.0e-1 },
                Edge { edge_id: "n4-n5".into(), from_node_id: "n4".into(), to_node_id: "n5".into(), conductance: 5.0e1 },
                Edge { edge_id: "n4-n6".into(), from_node_id: "n4".into(), to_node_id: "n6".into(), conductance: 1.0e-2 },
                Edge { edge_id: "n5-n6".into(), from_node_id: "n5".into(), to_node_id: "n6".into(), conductance: 2.0e1 },
                Edge { edge_id: "n5-n7".into(), from_node_id: "n5".into(), to_node_id: "n7".into(), conductance: 5.0e-1 },
                Edge { edge_id: "n6-n7".into(), from_node_id: "n6".into(), to_node_id: "n7".into(), conductance: 1.0e1 },
                Edge { edge_id: "n4-right".into(), from_node_id: "n4".into(), to_node_id: "right".into(), conductance: 1.0e-2 },
                Edge { edge_id: "n6-right".into(), from_node_id: "n6".into(), to_node_id: "right".into(), conductance: 2.0 },
                Edge { edge_id: "n7-right".into(), from_node_id: "n7".into(), to_node_id: "right".into(), conductance: 5.0 },
                Edge { edge_id: "top-n1".into(), from_node_id: "top".into(), to_node_id: "n1".into(), conductance: 2.0e-2 },
                Edge { edge_id: "top-n4".into(), from_node_id: "top".into(), to_node_id: "n4".into(), conductance: 1.0e-1 },
                Edge { edge_id: "top-n7".into(), from_node_id: "top".into(), to_node_id: "n7".into(), conductance: 1.0 },
            ],
            boundary_node_ids: vec!["left".into(), "right".into(), "top".into()],
            rational_shifts: vec![1.0],
            laplace_samples: vec![0.0, 5.0e-3, 5.0e-2, 5.0e-1, 5.0, 5.0e1, 5.0e2, 5.0e3, 5.0e4],
            adaptive: AdaptiveOptions {
                candidate_shifts: vec![1.0e-5, 1.0e-3, 1.0e-2, 1.0e-1, 3.0, 1.0e1, 1.0e2, 1.0e3, 1.0e4],
                target_max_relative_error: Some(5.0e-2),
                maximum_basis_dimension: Some(6),
            },
            cg_relative_tolerance: 1.0e-11,
            cg_max_iterations: 4096,
        }
    }

    #[test]
    fn adaptive_shift_selection_reduces_held_out_error_at_equal_basis_rank() {
        let mut fixed_fixture = adaptive_multiport_fixture();
        fixed_fixture.adaptive = AdaptiveOptions::default();
        fixed_fixture.rational_shifts = vec![1.0, 1.0e3];
        let fixed = analyze(&fixed_fixture, "fixed-two-shift-baseline".into()).unwrap();

        let adaptive_fixture = adaptive_multiport_fixture();
        let adaptive = analyze(&adaptive_fixture, "adaptive-port-shift-candidate".into()).unwrap();

        let interior_count = adaptive_fixture.nodes.len() - adaptive_fixture.boundary_node_ids.len();
        assert_eq!(
            adaptive.adaptive_target_met,
            Some(true),
            "candidate error {:?}, stop {}, audit error {}, rank {}",
            adaptive.adaptive_maximum_candidate_relative_error,
            adaptive.adaptive_stop_reason,
            adaptive.maximum_sampled_response_error,
            adaptive.basis_dimension
        );
        assert_eq!(adaptive.adaptive_stop_reason, "target_met");
        assert!(!adaptive.adaptive_selection_steps.is_empty());
        assert!(adaptive.adaptive_selection_steps.iter().all(|step| {
            step.selected_input_port_index < adaptive.boundary_node_ids.len()
        }));
        assert_eq!(adaptive.basis_dimension, fixed.basis_dimension);
        assert!(adaptive.basis_dimension < interior_count);
        assert!(adaptive.maximum_sampled_response_error <= 1.0e-2);
        assert!(adaptive.maximum_sampled_response_error < fixed.maximum_sampled_response_error);
        assert!(adaptive.dc_relative_error < 1.0e-8);
        assert!(adaptive.structural_passivity_certificate_valid);
        // Candidate full-order columns (27), seed snapshots (3), one DC
        // operator (3), and audit-grid columns (27). The audit's zero-shift
        // response is also the exact DC reference, so no duplicate solve occurs.
        assert_eq!(adaptive.cg_solve_count, 60);
        assert!(adaptive.total_cg_iterations < 469);
    }

    #[test]
    fn adaptive_rank_cap_reports_unmet_target_without_false_qualification() {
        let mut fixture = adaptive_multiport_fixture();
        fixture.adaptive.target_max_relative_error = Some(1.0e-12);
        fixture.adaptive.maximum_basis_dimension = Some(3);
        let report = analyze(&fixture, "adaptive-rank-cap-negative-control".into()).unwrap();
        assert_eq!(report.basis_dimension, 3);
        assert_eq!(report.adaptive_target_met, Some(false));
        assert_eq!(report.adaptive_stop_reason, "basis_dimension_cap");
        assert!(report
            .adaptive_maximum_candidate_relative_error
            .is_some_and(|error| error > 1.0e-12));
        assert!(report.structural_passivity_certificate_valid);
    }

    #[test]
    fn prima_style_block_arnoldi_baseline_is_passive_at_adaptive_rank() {
        let adaptive_fixture = adaptive_multiport_fixture();
        let report = analyze(&adaptive_fixture, "block-arnoldi-comparison-fixture".into()).unwrap();
        assert_eq!(
            report.block_arnoldi_baseline_basis_dimension,
            Some(report.basis_dimension)
        );
        assert_eq!(
            report.block_arnoldi_baseline_passivity_certificate_valid,
            Some(true)
        );
        assert!(
            report
                .block_arnoldi_baseline_dc_relative_error
                .is_some_and(|error| error < 1.0e-8)
        );
        assert!(
            report
                .block_arnoldi_baseline_feedthrough_limit_relative_error
                .is_some_and(|error| error < 1.0e-8)
        );
        assert!(report
            .block_arnoldi_baseline_maximum_sampled_response_error
            .is_some_and(f64::is_finite));
        assert_eq!(report.block_arnoldi_baseline_error_bound_valid, Some(true));
        assert!(
            report.block_arnoldi_baseline_maximum_relative_response_error_upper_bound
                >= report.block_arnoldi_baseline_maximum_sampled_response_error
        );
        assert!(report
            .block_arnoldi_baseline_cg_solve_count
            .is_some_and(|count| count > 0));
        assert!(report
            .block_arnoldi_baseline_total_cg_iterations
            .is_some_and(|iterations| iterations > 0));
    }

    #[test]
    fn adaptive_validation_grid_must_be_disjoint_from_seed_and_audit_grids() {
        let mut invalid = adaptive_multiport_fixture();
        invalid.adaptive.candidate_shifts[0] = invalid.laplace_samples[1];
        assert!(validate(&invalid).is_err());

        let mut invalid = adaptive_multiport_fixture();
        invalid.adaptive.maximum_basis_dimension = Some(0);
        assert!(validate(&invalid).is_err());
    }

    #[test]
    fn residual_error_certificate_bounds_actual_response_error_on_manufactured_graphs() {
        for fixture in [fixture(), branched_fixture(), high_contrast_multiport_fixture()] {
            let report = analyze(&fixture, "certificate-manufactured".into()).unwrap();
            assert!(report.error_certificate_valid);
            assert!(report.m_matrix_lambda_min_lower_bound > 0.0);
            assert!(report.m_matrix_supersolution_max_component > 0.0);
            assert!(report
                .frequency_results
                .iter()
                .all(|sample| sample.error_bound_valid));
            assert!(report
                .frequency_results
                .iter()
                .all(|sample| sample.absolute_response_error
                    <= sample.absolute_response_error_upper_bound));
            assert_eq!(report.error_certificate_pcg_solve_count, 1);
            assert!(report.error_certificate_pcg_iterations > 0);
        }
    }

    #[test]
    fn m_matrix_eigenvalue_lower_bound_is_invariant_to_joint_unit_rescaling() {
        let mut base = fixture();
        let base_model = assemble(&base).unwrap();
        let base_bound = m_matrix_lambda_min_lower_bound(
            &base_model.a,
            base.cg_relative_tolerance,
            base.cg_max_iterations,
        )
        .unwrap();

        for factor in [1.0e-24, 1.0e24] {
            let mut scaled = fixture();
            for node in &mut scaled.nodes {
                if let Some(storage) = node.storage.as_mut() {
                    *storage *= factor;
                }
            }
            for edge in &mut scaled.edges {
                edge.conductance *= factor;
            }
            let model = assemble(&scaled).unwrap();
            let bound = m_matrix_lambda_min_lower_bound(
                &model.a,
                scaled.cg_relative_tolerance,
                scaled.cg_max_iterations,
            )
            .unwrap();
            assert!(
                (bound.lambda_min_lower_bound - base_bound.lambda_min_lower_bound).abs()
                    <= base_bound.lambda_min_lower_bound * 1.0e-8
            );
        }
    }

    #[test]
    fn rejects_non_stieltjes_sparse_operator_before_certifying_error() {
        let invalid = CsrMatrix {
            size: 2,
            row_ptr: vec![0, 2, 4],
            col_idx: vec![0, 1, 0, 1],
            values: vec![2.0, 0.1, 0.1, 2.0],
            diagonal: vec![2.0, 2.0],
        };
        assert!(invalid.validate_stieltjes().is_err());

        let asymmetric = CsrMatrix {
            size: 2,
            row_ptr: vec![0, 2, 4],
            col_idx: vec![0, 1, 0, 1],
            values: vec![2.0, -0.5, -0.4, 2.0],
            diagonal: vec![2.0, 2.0],
        };
        assert!(asymmetric.validate_stieltjes().is_err());
    }

    #[test]
    fn sparse_modal_projection_preserves_exact_dc_boundary_operator() {
        let report = analyze(&fixture(), "synthetic-input-digest".into()).unwrap();
        assert!(report.dc_relative_error < TOL);
        assert!((report.exact_dc_response[0][0] - 12.0 / 13.0).abs() < TOL);
        assert!(report.structural_passivity_certificate_valid);
    }

    #[test]
    fn projection_has_positive_reduced_operator_and_psd_dc_response() {
        let report = analyze(&fixture(), "synthetic-input-digest".into()).unwrap();
        assert!(report.a_r_min_eigenvalue > 0.0);
        assert!(report.lambda_0_min_eigenvalue >= -TOL);
        assert!(report.structural_passivity_certificate_valid);
    }

    #[test]
    fn full_port_snapshot_span_recovers_all_dynamic_samples() {
        let mut fixture = fixture();
        fixture.rational_shifts = vec![0.0, 0.1, 1.0, 10.0];
        let report = analyze(&fixture, "synthetic-input-digest".into()).unwrap();
        assert_eq!(report.basis_dimension, 2);
        assert!(report.frequency_samples.iter().all(|sample| sample.relative_response_error < 1.0e-9));
    }

    #[test]
    fn branched_cross_linked_network_recovers_full_port_response() {
        let report = analyze(&branched_fixture(), "synthetic-branched-digest".into()).unwrap();
        assert_eq!(report.interior_node_count, 4);
        assert_eq!(report.basis_dimension, 4);
        assert!(report.dc_relative_error < 1.0e-9);
        assert!(report.structural_passivity_certificate_valid);
        assert!(report.frequency_samples.iter().all(|sample| sample.relative_response_error < 1.0e-8));
    }

    #[test]
    fn branched_response_is_invariant_to_node_and_edge_order() {
        let original = branched_fixture();
        let baseline = analyze(&original, "synthetic-branched-digest".into()).unwrap();
        let mut permuted = original;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let reordered = analyze(&permuted, "synthetic-branched-digest".into()).unwrap();

        assert!(relative_error(&baseline.exact_dc_response, &reordered.exact_dc_response) < 1.0e-10);
        assert!(relative_error(&baseline.reduced_dc_response, &reordered.reduced_dc_response) < 1.0e-9);
        assert_eq!(baseline.frequency_samples.len(), reordered.frequency_samples.len());
        for (left, right) in baseline.frequency_samples.iter().zip(reordered.frequency_samples.iter()) {
            assert!((left.relative_response_error - right.relative_response_error).abs() < 1.0e-8);
        }
    }

    #[test]
    fn high_contrast_three_port_network_preserves_symmetric_response() {
        let report = analyze(
            &high_contrast_multiport_fixture(),
            "synthetic-high-contrast-multiport".into(),
        )
        .unwrap();
        assert_eq!(report.boundary_node_ids.len(), 3);
        assert_eq!(report.interior_node_count, 4);
        assert_eq!(report.basis_dimension, 4);
        assert!(report.dc_relative_error < 1.0e-9);
        assert!(report.structural_passivity_certificate_valid);
        for row in 0..3 {
            for column in 0..3 {
                let left = report.exact_dc_response[row][column];
                let right = report.exact_dc_response[column][row];
                assert!((left - right).abs() < 1.0e-8 * left.abs().max(right.abs()).max(1.0));
            }
        }
        assert!(report
            .frequency_samples
            .iter()
            .all(|sample| sample.relative_response_error < 1.0e-8));
    }

    #[test]
    fn zero_shift_snapshots_preserve_dc_and_high_frequency_feedthrough() {
        let report = analyze(
            &high_contrast_multiport_fixture(),
            "synthetic-zero-shift-multiport".into(),
        )
        .unwrap();
        assert!(report.dc_relative_error < 1.0e-8);
        assert!(report.feedthrough_limit_relative_error < 1.0e-8);
        assert!(report.maximum_sampled_response_error < 1.0e-8);
    }

    #[test]
    fn nonzero_shift_model_reports_feedthrough_limit_error_separately() {
        let mut fixture = high_contrast_multiport_fixture();
        fixture.rational_shifts = vec![1.0];
        fixture.laplace_samples = vec![0.0, 1.0, 10.0, 100.0];
        let model = assemble(&fixture).unwrap();
        let (projection, _) = build_projection(&model, &fixture).unwrap();
        let expected_feedthrough_error = relative_error(&projection.d_r, &model.d);
        let report = analyze(&fixture, "synthetic-nonzero-shift-multiport".into()).unwrap();
        assert!(report.dc_relative_error < 1.0e-8);
        assert!(
            (report.feedthrough_limit_relative_error - expected_feedthrough_error).abs() < 1.0e-12
        );
        assert!(report.feedthrough_limit_relative_error.is_finite());
        assert!(report.feedthrough_limit_relative_error > 1.0e-6);
        let maximum_sampled = report
            .frequency_samples
            .iter()
            .map(|sample| sample.relative_response_error)
            .fold(0.0_f64, f64::max);
        assert!((report.maximum_sampled_response_error - maximum_sampled).abs() < 1.0e-12);
    }

    #[test]
    fn passivity_checks_and_basis_construction_are_scale_invariant() {
        for factor in [1.0e-24, 1.0e24] {
            let mut scaled = fixture();
            for node in &mut scaled.nodes {
                if let Some(storage) = node.storage.as_mut() {
                    *storage *= factor;
                }
            }
            for edge in &mut scaled.edges {
                edge.conductance *= factor;
            }
            let report = analyze(&scaled, format!("scaled-synthetic-{factor:e}")).unwrap();
            let expected = factor * (12.0 / 13.0);
            assert!((report.exact_dc_response[0][0] - expected).abs() < expected.abs() * 1.0e-9);
            assert!((report.lambda_0_min_eigenvalue).abs() < report.lambda_0_scale * 1.0e-8);
            assert!(report.lambda_0_passivity_tolerance > 0.0);
            assert!(report.structural_passivity_certificate_valid);
            assert_eq!(report.basis_dimension, 2);
            assert!(report.dc_relative_error < 1.0e-9);
        }
    }

    #[test]
    fn high_contrast_multiport_response_is_invariant_to_input_order() {
        let original = high_contrast_multiport_fixture();
        let baseline = analyze(&original, "synthetic-high-contrast-multiport".into()).unwrap();
        let mut permuted = original;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let reordered = analyze(&permuted, "synthetic-high-contrast-multiport".into()).unwrap();
        assert!(relative_error(&baseline.exact_dc_response, &reordered.exact_dc_response) < 1.0e-9);
        assert!(relative_error(&baseline.reduced_dc_response, &reordered.reduced_dc_response) < 1.0e-8);
    }

    #[test]
    fn sparse_storage_grows_linearly_on_long_chain() {
        let fixture = chain_fixture(128);
        let model = assemble(&fixture).unwrap();
        assert_eq!(model.a.size, 128);
        assert!(model.a.nnz() <= 3 * model.a.size);
        assert!(model.a.nnz() < model.a.size * model.a.size / 10);
        let report = analyze(&fixture, "synthetic-chain-digest".into()).unwrap();
        assert!(report.structural_passivity_certificate_valid);
        assert!(report.dc_relative_error < 1.0e-8);
    }

    #[test]
    fn rational_shifts_duplicate_span_do_not_create_duplicate_basis_vectors() {
        let mut fixture = fixture();
        fixture.rational_shifts = vec![0.0, 0.0, 0.1, 0.1];
        let report = analyze(&fixture, "synthetic-input-digest".into()).unwrap();
        assert!(report.basis_dimension <= report.basis_snapshots_attempted);
        assert_eq!(report.basis_dimension, 2);
    }

    #[test]
    fn rejects_nonpositive_storage_and_floating_interior_components() {
        let mut invalid = fixture();
        invalid.nodes[1].storage = Some(0.0);
        assert!(assemble(&invalid).is_err());

        let mut invalid = fixture();
        invalid.nodes.push(Node { node_id: "floating".into(), storage: Some(1.0) });
        assert!(assemble(&invalid).is_err());
    }

    #[test]
    fn rejects_negative_shift_and_invalid_cg_controls() {
        let mut invalid = fixture();
        invalid.rational_shifts[0] = -1.0;
        assert!(validate(&invalid).is_err());

        let mut invalid = fixture();
        invalid.cg_relative_tolerance = 0.0;
        assert!(validate(&invalid).is_err());
    }
}
