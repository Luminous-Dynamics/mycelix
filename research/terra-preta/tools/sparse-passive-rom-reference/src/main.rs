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

struct Projection {
    a_r: Dense,
    b_r: Dense,
    d_r: Dense,
    basis_dimension: usize,
    basis_snapshots: usize,
    lambda_0_min_eigenvalue: f64,
    lambda_0_scale: f64,
    a_r_min_eigenvalue: f64,
}

#[derive(Serialize)]
struct SampleResult {
    laplace_s: f64,
    relative_response_error: f64,
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
    basis_dimension: usize,
    basis_snapshots_attempted: usize,
    exact_dc_response: Dense,
    reduced_dc_response: Dense,
    dc_relative_error: f64,
    feedthrough_limit_relative_error: f64,
    maximum_sampled_response_error: f64,
    a_r_min_eigenvalue: f64,
    lambda_0_min_eigenvalue: f64,
    lambda_0_scale: f64,
    lambda_0_passivity_tolerance: f64,
    structural_passivity_certificate_valid: bool,
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

fn response_from_sparse(
    model: &Model,
    shift: f64,
    tolerance: f64,
    max_iterations: usize,
) -> Result<(Dense, Vec<CgStats>), String> {
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
    Ok((response, stats))
}

fn build_projection(
    model: &Model,
    fixture: &Fixture,
) -> Result<(Projection, Vec<CgStats>), String> {
    let nb = model.boundary_ids.len();
    let mut basis: Vec<Vec<f64>> = Vec::new();
    let mut stats = Vec::new();
    let mut snapshots_attempted = 0;
    for shift in &fixture.rational_shifts {
        for port in 0..nb {
            snapshots_attempted += 1;
            let (snapshot, snapshot_stats) = pcg(
                &model.a,
                *shift,
                &column(&model.b, port),
                fixture.cg_relative_tolerance,
                fixture.cg_max_iterations,
            )?;
            stats.push(snapshot_stats);
            push_orthonormal_basis(&mut basis, snapshot);
        }
    }
    if basis.is_empty() {
        return Err("rational Krylov snapshot basis is numerically empty".into());
    }

    let (lambda_0, dc_stats) = response_from_sparse(
        model,
        0.0,
        fixture.cg_relative_tolerance,
        fixture.cg_max_iterations,
    )?;
    stats.extend(dc_stats);

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
    let lambda_0_min = min_symmetric_eigenvalue(&lambda_0)?;
    let lambda_0_scale = lambda_0
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    Ok((
        Projection {
            a_r,
            b_r,
            d_r,
            basis_dimension: basis.len(),
            basis_snapshots: snapshots_attempted,
            lambda_0_min_eigenvalue: lambda_0_min,
            lambda_0_scale,
            a_r_min_eigenvalue: a_r_min,
        },
        stats,
    ))
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
        cg_relative_tolerance: 1.0e-10,
        cg_max_iterations: interior_count.saturating_mul(8).max(128),
    }
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    let model = assemble(fixture)?;
    let (projection, mut stats) = build_projection(&model, fixture)?;
    let lambda_0_passivity_tolerance =
        projection.lambda_0_scale.max(f64::MIN_POSITIVE) * 1.0e-10;
    if projection.a_r_min_eigenvalue <= 0.0
        || projection.lambda_0_min_eigenvalue < -lambda_0_passivity_tolerance
    {
        return Err("reduced model failed the scale-aware structural passivity checks".into());
    }
    let mut frequency_samples = Vec::with_capacity(fixture.laplace_samples.len());
    for shift in &fixture.laplace_samples {
        let (full, full_stats) = response_from_sparse(
            &model,
            *shift,
            fixture.cg_relative_tolerance,
            fixture.cg_max_iterations,
        )?;
        stats.extend(full_stats.iter().copied());
        let reduced = response_from_reduced(&projection, *shift)?;
        frequency_samples.push(SampleResult {
            laplace_s: *shift,
            relative_response_error: relative_error(&reduced, &full),
            full_cg_max_iterations: full_stats.iter().map(|item| item.iterations).max().unwrap_or(0),
            full_cg_max_relative_residual: full_stats.iter()
                .map(|item| item.relative_residual).fold(0.0_f64, f64::max),
        });
    }
    let (exact_dc, _) = response_from_sparse(
        &model,
        0.0,
        fixture.cg_relative_tolerance,
        fixture.cg_max_iterations,
    )?;
    let reduced_dc = response_from_reduced(&projection, 0.0)?;
    let feedthrough_limit_relative_error = relative_error(&projection.d_r, &model.d);
    let maximum_sampled_response_error = frequency_samples
        .iter()
        .map(|sample| sample.relative_response_error)
        .fold(0.0_f64, f64::max);
    let maximum_cg_iterations = stats.iter().map(|item| item.iterations).max().unwrap_or(0);
    let maximum_cg_relative_residual = stats.iter()
        .map(|item| item.relative_residual).fold(0.0_f64, f64::max);
    let csr_density = model.a.nnz() as f64 / (model.a.size * model.a.size) as f64;
    let dc_relative_error = relative_error(&reduced_dc, &exact_dc);

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        boundary_node_ids: model.boundary_ids.clone(),
        interior_node_count: model.interior_ids.len(),
        csr_nonzero_count: model.a.nnz(),
        csr_density,
        rational_shifts: fixture.rational_shifts.clone(),
        basis_dimension: projection.basis_dimension,
        basis_snapshots_attempted: projection.basis_snapshots,
        exact_dc_response: exact_dc,
        reduced_dc_response: reduced_dc.clone(),
        dc_relative_error,
        feedthrough_limit_relative_error,
        maximum_sampled_response_error,
        a_r_min_eigenvalue: projection.a_r_min_eigenvalue,
        lambda_0_min_eigenvalue: projection.lambda_0_min_eigenvalue,
        lambda_0_scale: projection.lambda_0_scale,
        lambda_0_passivity_tolerance,
        structural_passivity_certificate_valid: projection.a_r_min_eigenvalue > 0.0
            && projection.lambda_0_min_eigenvalue >= -lambda_0_passivity_tolerance,
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
            cg_relative_tolerance: 1.0e-11,
            cg_max_iterations: 512,
        }
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
