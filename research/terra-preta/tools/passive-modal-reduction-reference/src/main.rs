
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    fs::{self, OpenOptions},
    io::Write,
    ops::{Add, Div, Mul, Sub},
    process,
};

const VERSION: &str = "terra-passive-modal-reduction-reference-v0.1";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const TOL: f64 = 1.0e-10;
const JACOBI_MAX_ITERATIONS: usize = 100_000;

type Matrix = Vec<Vec<f64>>;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Node {
    node_id: String,
    storage: Option<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Edge {
    edge_id: String,
    from_node_id: String,
    to_node_id: String,
    conductance: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ComplexSample {
    real: f64,
    imag: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    schema_version: String,
    fixture_status: String,
    scenario_id: String,
    nodes: Vec<Node>,
    edges: Vec<Edge>,
    boundary_node_ids: Vec<String>,
    retained_mode_indices: Vec<usize>,
    laplace_samples: Vec<ComplexSample>,
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
struct Complex {
    real: f64,
    imag: f64,
}

impl Complex {
    const ZERO: Self = Self { real: 0.0, imag: 0.0 };

    fn new(real: f64, imag: f64) -> Self {
        Self { real, imag }
    }

    fn abs(self) -> f64 {
        self.real.hypot(self.imag)
    }

    fn conj(self) -> Self {
        Self::new(self.real, -self.imag)
    }

}

impl Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.real + rhs.real, self.imag + rhs.imag)
    }
}

impl Sub for Complex {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.real - rhs.real, self.imag - rhs.imag)
    }
}

impl Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        Self::new(
            self.real.mul_add(rhs.real, -self.imag * rhs.imag),
            self.real.mul_add(rhs.imag, self.imag * rhs.real),
        )
    }
}

impl Div for Complex {
    type Output = Self;
    fn div(self, rhs: Self) -> Self::Output {
        let denominator = rhs.real.mul_add(rhs.real, rhs.imag * rhs.imag);
        Self::new(
            (self.real * rhs.real + self.imag * rhs.imag) / denominator,
            (self.imag * rhs.real - self.real * rhs.imag) / denominator,
        )
    }
}

#[derive(Clone)]
struct Model {
    boundary_ids: Vec<String>,
    interior_ids: Vec<String>,
    l_bb: Matrix,
    l_bi: Matrix,
    l_ii: Matrix,
    storage: Vec<f64>,
    a: Matrix,
    b: Matrix,
}

#[derive(Clone)]
struct ModalModel {
    eigenvalues: Vec<f64>,
    modal_b: Matrix,
    residues: Vec<Matrix>,
    feedthrough: Matrix,
    eigenpair_residual_max: f64,
    orthogonality_error_max: f64,
}

#[derive(Debug, Serialize)]
struct FrequencyResult {
    s_real: f64,
    s_imag: f64,
    full_modal_vs_direct_relative_error: f64,
    reduced_vs_full_relative_error: f64,
    full_positive_real_min_eigenvalue: f64,
    reduced_positive_real_min_eigenvalue: f64,
    conjugate_symmetry_error: f64,
}

#[derive(Debug, Serialize)]
struct Report {
    audit_version: String,
    fixture_status: String,
    scenario_id: String,
    input_sha256: String,
    boundary_node_ids: Vec<String>,
    interior_node_ids: Vec<String>,
    eigenvalues: Vec<f64>,
    modal_residues: Vec<Matrix>,
    eigenpair_residual_max: f64,
    eigenvector_orthogonality_error_max: f64,
    dc_boundary_response: Matrix,
    reduced_mode_indices: Vec<usize>,
    reduced_feedthrough: Matrix,
    reduced_dc_boundary_response: Matrix,
    dc_response_relative_error: f64,
    full_passivity_matrix_min_eigenvalue: f64,
    reduced_passivity_matrix_min_eigenvalue: f64,
    unstable_reduced_pole_count: usize,
    frequency_results: Vec<FrequencyResult>,
    interpretation_boundary: String,
}

fn validate(fixture: &Fixture) -> Result<(), String> {
    if fixture.schema_version != VERSION
        || fixture.fixture_status != FIXTURE_STATUS
        || fixture.scenario_id.trim().is_empty()
    {
        return Err("fixture identity, schema version or synthetic-only status is invalid".into());
    }
    if fixture.nodes.len() < 3 || fixture.edges.len() < 2 || fixture.boundary_node_ids.len() < 2 {
        return Err("fixture requires at least two boundary ports and one interior state".into());
    }
    if fixture.laplace_samples.is_empty() {
        return Err("at least one complex Laplace sample is required".into());
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
            return Err(format!("interior node {} requires finite positive storage", node.node_id));
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
            return Err(format!("edge {} conductance must be finite and positive", edge.edge_id));
        }
    }

    let mut retained = BTreeSet::new();
    for index in &fixture.retained_mode_indices {
        if !retained.insert(*index) {
            return Err("retained mode indices must be unique".into());
        }
    }
    for sample in &fixture.laplace_samples {
        if !sample.real.is_finite() || !sample.imag.is_finite() || sample.real < 0.0 {
            return Err("Laplace samples must be finite and lie in the closed right half-plane".into());
        }
    }
    Ok(())
}

fn assemble_model(fixture: &Fixture) -> Result<Model, String> {
    let boundary_set: BTreeSet<&str> = fixture.boundary_node_ids.iter().map(String::as_str).collect();
    let node_ids: Vec<String> = fixture.nodes.iter().map(|node| node.node_id.clone()).collect();
    let node_index: BTreeMap<&str, usize> = node_ids.iter()
        .enumerate().map(|(index, id)| (id.as_str(), index)).collect();
    let boundary_indices: Vec<usize> = fixture.boundary_node_ids.iter()
        .map(|id| node_index[id.as_str()]).collect();
    let interior_nodes: Vec<&Node> = fixture.nodes.iter()
        .filter(|node| !boundary_set.contains(node.node_id.as_str())).collect();
    let interior_indices: Vec<usize> = interior_nodes.iter()
        .map(|node| node_index[node.node_id.as_str()]).collect();
    let n = node_ids.len();
    let mut laplacian = vec![vec![0.0; n]; n];
    let mut adjacency = vec![Vec::<usize>::new(); n];

    for edge in &fixture.edges {
        let from = node_index[edge.from_node_id.as_str()];
        let to = node_index[edge.to_node_id.as_str()];
        let g = edge.conductance;
        laplacian[from][from] += g;
        laplacian[to][to] += g;
        laplacian[from][to] -= g;
        laplacian[to][from] -= g;
        adjacency[from].push(to);
        adjacency[to].push(from);
    }

    let mut visited = vec![false; n];
    for start in 0..n {
        if visited[start] { continue; }
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
            return Err("an interior component is unanchored by every prescribed boundary port".into());
        }
    }

    let nb = boundary_indices.len();
    let ni = interior_indices.len();
    let mut l_bb = vec![vec![0.0; nb]; nb];
    let mut l_bi = vec![vec![0.0; ni]; nb];
    let mut l_ii = vec![vec![0.0; ni]; ni];
    for row in 0..nb {
        for col in 0..nb {
            l_bb[row][col] = laplacian[boundary_indices[row]][boundary_indices[col]];
        }
        for col in 0..ni {
            l_bi[row][col] = laplacian[boundary_indices[row]][interior_indices[col]];
        }
    }
    for row in 0..ni {
        for col in 0..ni {
            l_ii[row][col] = laplacian[interior_indices[row]][interior_indices[col]];
        }
    }

    let storage: Vec<f64> = interior_nodes.iter()
        .map(|node| node.storage.expect("validated interior storage")).collect();
    let mut a = vec![vec![0.0; ni]; ni];
    let mut b = vec![vec![0.0; nb]; ni];
    for row in 0..ni {
        for col in 0..ni {
            a[row][col] = l_ii[row][col] / (storage[row] * storage[col]).sqrt();
        }
        for col in 0..nb {
            b[row][col] = l_bi[col][row] / storage[row].sqrt();
        }
    }
    Ok(Model {
        boundary_ids: fixture.boundary_node_ids.clone(),
        interior_ids: interior_nodes.iter().map(|node| node.node_id.clone()).collect(),
        l_bb: l_bb.clone(),
        l_bi,
        l_ii,
        storage,
        a,
        b,
    })
}

fn identity(size: usize) -> Matrix {
    let mut result = vec![vec![0.0; size]; size];
    for (index, row) in result.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    result
}

fn symmetric_eigendecomposition(matrix: &[Vec<f64>]) -> Result<(Vec<f64>, Matrix, bool), String> {
    let n = matrix.len();
    if n == 0 || matrix.iter().any(|row| row.len() != n) {
        return Err("symmetric eigensolver requires a non-empty square matrix".into());
    }
    for row in 0..n {
        for column in 0..n {
            if (matrix[row][column] - matrix[column][row]).abs() > TOL {
                return Err("symmetric eigensolver received a nonsymmetric matrix".into());
            }
        }
    }
    let mut a = matrix.to_vec();
    let mut vectors = identity(n);
    let scale = a.iter().flatten().fold(0.0_f64, |largest, value| largest.max(value.abs()));
    let tolerance = scale.max(1.0) * 1.0e-14;
    let mut converged = false;

    for _ in 0..JACOBI_MAX_ITERATIONS {
        let mut pivot = (0usize, 0usize);
        let mut largest = 0.0_f64;
        for (row, values) in a.iter().enumerate() {
            for (col, value) in values.iter().enumerate().skip(row + 1) {
                if value.abs() > largest {
                    largest = value.abs();
                    pivot = (row, col);
                }
            }
        }
        if largest <= tolerance {
            converged = true;
            break;
        }
        let (p, q) = pivot;
        let app = a[p][p];
        let aqq = a[q][q];
        let apq = a[p][q];
        let tau = (aqq - app) / (2.0 * apq);
        let t = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        for k in 0..n {
            if k != p && k != q {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = s * akp + c * akq;
                a[q][k] = a[k][q];
            }
            let vkp = vectors[k][p];
            let vkq = vectors[k][q];
            vectors[k][p] = c * vkp - s * vkq;
            vectors[k][q] = s * vkp + c * vkq;
        }
    }

    let mut pairs: Vec<(f64, Vec<f64>)> = (0..n)
        .map(|column| (a[column][column], (0..n).map(|row| vectors[row][column]).collect()))
        .collect();
    pairs.sort_by(|left, right| left.0.total_cmp(&right.0));
    let eigenvalues = pairs.iter().map(|pair| pair.0).collect();
    let eigenvectors = (0..n).map(|row| pairs.iter().map(|pair| pair.1[row]).collect()).collect();
    Ok((eigenvalues, eigenvectors, converged))
}

fn max_eigenpair_residual(matrix: &[Vec<f64>], values: &[f64], vectors: &[Vec<f64>]) -> f64 {
    let n = matrix.len();
    let mut maximum = 0.0_f64;
    for column in 0..n {
        for row in 0..n {
            let product: f64 = (0..n).map(|k| matrix[row][k] * vectors[k][column]).sum();
            maximum = maximum.max((product - values[column] * vectors[row][column]).abs());
        }
    }
    maximum
}

fn max_orthogonality_error(vectors: &[Vec<f64>]) -> f64 {
    let n = vectors.len();
    let mut maximum = 0.0_f64;
    for left in 0..n {
        for right in 0..n {
            let dot: f64 = (0..n).map(|row| vectors[row][left] * vectors[row][right]).sum();
            let expected = if left == right { 1.0 } else { 0.0 };
            maximum = maximum.max((dot - expected).abs());
        }
    }
    maximum
}

fn build_modal_model(model: &Model) -> Result<ModalModel, String> {
    let (eigenvalues, eigenvectors, converged) = symmetric_eigendecomposition(&model.a)?;
    if !converged {
        return Err("symmetric Jacobi eigensolver did not converge".into());
    }
    if eigenvalues.iter().any(|value| !value.is_finite() || *value <= 0.0) {
        return Err("interior dynamic operator is not positive definite".into());
    }
    let ni = eigenvalues.len();
    let nb = model.boundary_ids.len();
    let mut modal_b = vec![vec![0.0; nb]; ni];
    for mode in 0..ni {
        for port in 0..nb {
            modal_b[mode][port] = (0..ni)
                .map(|row| eigenvectors[row][mode] * model.b[row][port])
                .sum();
        }
    }
    let residues: Vec<Matrix> = modal_b.iter().map(|row| {
        row.iter().map(|left| row.iter().map(|right| left * right).collect()).collect()
    }).collect();
    Ok(ModalModel {
        eigenpair_residual_max: max_eigenpair_residual(&model.a, &eigenvalues, &eigenvectors),
        orthogonality_error_max: max_orthogonality_error(&eigenvectors),
        eigenvalues,
        modal_b,
        residues,
        feedthrough: model.l_bb.clone(),
    })
}

fn matrix_subtract(left: &[Vec<f64>], right: &[Vec<f64>]) -> Matrix {
    left.iter().zip(right.iter()).map(|(a, b)| {
        a.iter().zip(b.iter()).map(|(x, y)| x - y).collect()
    }).collect()
}

fn matrix_scale(matrix: &[Vec<f64>], scale: f64) -> Matrix {
    matrix.iter().map(|row| row.iter().map(|value| value * scale).collect()).collect()
}

fn reduced_feedthrough(modal: &ModalModel, retained: &[usize]) -> Result<Matrix, String> {
    let mut retained_set = BTreeSet::new();
    for index in retained {
        if *index >= modal.eigenvalues.len() || !retained_set.insert(*index) {
            return Err("retained mode index is out of range or duplicated".into());
        }
    }
    let mut result = modal.feedthrough.clone();
    for index in 0..modal.eigenvalues.len() {
        if !retained_set.contains(&index) {
            result = matrix_subtract(
                &result,
                &matrix_scale(&modal.residues[index], 1.0 / modal.eigenvalues[index]),
            );
        }
    }
    Ok(result)
}

fn transfer_from_modes(
    modal: &ModalModel,
    feedthrough: &[Vec<f64>],
    retained: &[usize],
    s: Complex,
) -> Vec<Vec<Complex>> {
    let mut result: Vec<Vec<Complex>> = feedthrough.iter().map(|row| {
        row.iter().map(|value| Complex::new(*value, 0.0)).collect()
    }).collect();
    for index in retained {
        let inverse = Complex::new(modal.eigenvalues[*index], 0.0) + s;
        let inverse = Complex::new(1.0, 0.0) / inverse;
        for (row, values) in result.iter_mut().enumerate() {
            for (column, value) in values.iter_mut().enumerate() {
                *value = *value - Complex::new(modal.residues[*index][row][column], 0.0) * inverse;
            }
        }
    }
    result
}

fn solve_complex(mut matrix: Vec<Vec<Complex>>, mut rhs: Vec<Complex>) -> Result<Vec<Complex>, String> {
    let n = matrix.len();
    if rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err("complex linear-system dimensions are invalid".into());
    }
    for column in 0..n {
        let pivot = (column..n).max_by(|left, right| {
            matrix[*left][column].abs().total_cmp(&matrix[*right][column].abs())
        }).ok_or_else(|| "complex system has no pivot".to_string())?;
        if matrix[pivot][column].abs() <= 1.0e-14 {
            return Err("complex interior system is singular or unresolved".into());
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        let divisor = matrix[column][column];
        for value in matrix[column].iter_mut().skip(column) {
            *value = *value / divisor;
        }
        rhs[column] = rhs[column] / divisor;
        let pivot_values = matrix[column].clone();
        let pivot_rhs = rhs[column];
        for (row_index, row_values) in matrix.iter_mut().enumerate() {
            if row_index == column {
                continue;
            }
            let factor = row_values[column];
            for (target, pivot_value) in row_values.iter_mut().zip(pivot_values.iter()).skip(column) {
                *target = *target - factor * *pivot_value;
            }
            rhs[row_index] = rhs[row_index] - factor * pivot_rhs;
        }
    }
    if rhs.iter().any(|value| !value.abs().is_finite()) {
        return Err("complex solve produced a non-finite result".into());
    }
    Ok(rhs)
}

fn direct_dynamic_response(model: &Model, s: Complex) -> Result<Vec<Vec<Complex>>, String> {
    let ni = model.storage.len();
    let nb = model.boundary_ids.len();
    let mut interior = vec![vec![Complex::ZERO; ni]; ni];
    for row in 0..ni {
        for col in 0..ni {
            interior[row][col] = Complex::new(model.l_ii[row][col], 0.0);
        }
        interior[row][row] = interior[row][row] + s * Complex::new(model.storage[row], 0.0);
    }
    let mut result: Vec<Vec<Complex>> = model.l_bb.iter().map(|row| {
        row.iter().map(|value| Complex::new(*value, 0.0)).collect()
    }).collect();
    for port in 0..nb {
        let rhs: Vec<Complex> = (0..ni).map(|row| {
            Complex::new(model.l_bi[port][row], 0.0)
        }).collect();
        let solution = solve_complex(interior.clone(), rhs)?;
        for boundary in 0..nb {
            let correction = (0..ni).map(|index| {
                Complex::new(model.l_bi[boundary][index], 0.0) * solution[index]
            }).fold(Complex::ZERO, |sum, value| sum + value);
            result[boundary][port] = result[boundary][port] - correction;
        }
    }
    Ok(result)
}

fn relative_complex_error(left: &[Vec<Complex>], right: &[Vec<Complex>]) -> f64 {
    let numerator = left.iter().flatten().zip(right.iter().flatten())
        .map(|(a, b)| (*a - *b).abs().powi(2)).sum::<f64>().sqrt();
    let denominator = right.iter().flatten().map(|value| value.abs().powi(2))
        .sum::<f64>().sqrt().max(f64::MIN_POSITIVE);
    numerator / denominator
}

fn real_part(matrix: &[Vec<Complex>]) -> Matrix {
    matrix.iter().map(|row| row.iter().map(|value| value.real).collect()).collect()
}

fn minimum_symmetric_eigenvalue(matrix: &[Vec<f64>]) -> Result<f64, String> {
    let (values, _, converged) = symmetric_eigendecomposition(matrix)?;
    if !converged { return Err("passivity matrix eigensolver did not converge".into()); }
    values.iter().copied().reduce(f64::min)
        .ok_or_else(|| "cannot calculate an eigenvalue for an empty matrix".into())
}

fn passivity_matrix(feedthrough: &[Vec<f64>], modal: &ModalModel, retained: &[usize]) -> Matrix {
    let nb = feedthrough.len();
    let nr = retained.len();
    let mut q = vec![vec![0.0; nb + nr]; nb + nr];
    for row in 0..nb {
        for column in 0..nb {
            q[row][column] = feedthrough[row][column];
        }
    }
    for (slot, mode) in retained.iter().enumerate() {
        q[nb + slot][nb + slot] = modal.eigenvalues[*mode];
        for port in 0..nb {
            q[port][nb + slot] = modal.modal_b[*mode][port];
            q[nb + slot][port] = modal.modal_b[*mode][port];
        }
    }
    q
}

fn relative_real_error(left: &[Vec<f64>], right: &[Vec<f64>]) -> f64 {
    let numerator = left.iter().flatten().zip(right.iter().flatten())
        .map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
    let denominator = right.iter().flatten().map(|value| value.powi(2))
        .sum::<f64>().sqrt().max(f64::MIN_POSITIVE);
    numerator / denominator
}

fn analyze(fixture: &Fixture, input_sha256: String) -> Result<Report, String> {
    validate(fixture)?;
    let model = assemble_model(fixture)?;
    let modal = build_modal_model(&model)?;
    for index in &fixture.retained_mode_indices {
        if *index >= modal.eigenvalues.len() {
            return Err(format!("retained mode index {index} exceeds modal order"));
        }
    }
    let mut retained = fixture.retained_mode_indices.clone();
    retained.sort_unstable();
    let all_modes: Vec<usize> = (0..modal.eigenvalues.len()).collect();
    let reduced_d = reduced_feedthrough(&modal, &retained)?;
    let full_dc_complex = transfer_from_modes(&modal, &modal.feedthrough, &all_modes, Complex::ZERO);
    let reduced_dc_complex = transfer_from_modes(&modal, &reduced_d, &retained, Complex::ZERO);
    let full_dc = real_part(&full_dc_complex);
    let reduced_dc = real_part(&reduced_dc_complex);
    let dc_error = relative_real_error(&reduced_dc, &full_dc);
    let full_q = passivity_matrix(&modal.feedthrough, &modal, &all_modes);
    let reduced_q = passivity_matrix(&reduced_d, &modal, &retained);
    let full_passivity_min = minimum_symmetric_eigenvalue(&full_q)?;
    let reduced_passivity_min = minimum_symmetric_eigenvalue(&reduced_q)?;
    let unstable_poles = retained.iter()
        .filter(|index| modal.eigenvalues[**index] <= 0.0).count();

    let mut frequency_results = Vec::with_capacity(fixture.laplace_samples.len());
    for sample in &fixture.laplace_samples {
        let s = Complex::new(sample.real, sample.imag);
        let direct = direct_dynamic_response(&model, s)?;
        let full = transfer_from_modes(&modal, &modal.feedthrough, &all_modes, s);
        let reduced = transfer_from_modes(&modal, &reduced_d, &retained, s);
        let conjugate_full = transfer_from_modes(&modal, &modal.feedthrough, &all_modes, s.conj());
        let conjugate_error = conjugate_full.iter().flatten().zip(full.iter().flatten())
            .map(|(a, b)| (*a - b.conj()).abs().powi(2)).sum::<f64>().sqrt()
            / full.iter().flatten().map(|value| value.abs().powi(2))
                .sum::<f64>().sqrt().max(f64::MIN_POSITIVE);
        frequency_results.push(FrequencyResult {
            s_real: sample.real,
            s_imag: sample.imag,
            full_modal_vs_direct_relative_error: relative_complex_error(&full, &direct),
            reduced_vs_full_relative_error: relative_complex_error(&reduced, &full),
            full_positive_real_min_eigenvalue: minimum_symmetric_eigenvalue(&real_part(&full))?,
            reduced_positive_real_min_eigenvalue: minimum_symmetric_eigenvalue(&real_part(&reduced))?,
            conjugate_symmetry_error: conjugate_error,
        });
    }

    Ok(Report {
        audit_version: VERSION.into(),
        fixture_status: fixture.fixture_status.clone(),
        scenario_id: fixture.scenario_id.clone(),
        input_sha256,
        boundary_node_ids: model.boundary_ids.clone(),
        interior_node_ids: model.interior_ids.clone(),
        eigenvalues: modal.eigenvalues.clone(),
        modal_residues: modal.residues.clone(),
        eigenpair_residual_max: modal.eigenpair_residual_max,
        eigenvector_orthogonality_error_max: modal.orthogonality_error_max,
        dc_boundary_response: full_dc,
        reduced_mode_indices: retained.clone(),
        reduced_feedthrough: reduced_d,
        reduced_dc_boundary_response: reduced_dc,
        dc_response_relative_error: dc_error,
        full_passivity_matrix_min_eigenvalue: full_passivity_min,
        reduced_passivity_matrix_min_eigenvalue: reduced_passivity_min,
        unstable_reduced_pole_count: unstable_poles,
        frequency_results,
        interpretation_boundary: "Exact mathematical reference for a fixed positive linear conductance graph with positive diagonal interior storage and ideal Dirichlet boundary ports. Modal passivity certificates and DC-preserving truncation apply only to this declared RC-like model. Complex-frequency checks are numerical diagnostics over supplied samples. This is not nonlinear Richards flow, unsaturated retention, hysteresis, multiphase or reactive transport, or empirical soil validation.".into(),
    })
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!("usage: {} INPUT.json OUTPUT.json",
            args.first().map(String::as_str).unwrap_or("terra-passive-modal-reduction-reference")));
    }
    let input = fs::read(&args[1]).map_err(|error| format!("cannot read input: {error}"))?;
    let digest = format!("{:x}", Sha256::digest(&input));
    let fixture: Fixture = serde_json::from_slice(&input)
        .map_err(|error| format!("invalid fixture JSON: {error}"))?;
    let report = analyze(&fixture, digest)?;
    let output_bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report: {error}"))?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(&args[2])
        .map_err(|error| format!("refusing output overwrite or cannot create output: {error}"))?;
    output.write_all(&output_bytes).map_err(|error| format!("cannot write report: {error}"))?;
    output.write_all(b"\n").map_err(|error| format!("cannot finalize report: {error}"))?;
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
        Fixture {
            schema_version: VERSION.into(),
            fixture_status: FIXTURE_STATUS.into(),
            scenario_id: "three-edge-chain-storage-1-2".into(),
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
            retained_mode_indices: vec![0],
            laplace_samples: vec![
                ComplexSample { real: 0.0, imag: 0.0 },
                ComplexSample { real: 0.0, imag: 0.1 },
                ComplexSample { real: 0.0, imag: 1.0 },
                ComplexSample { real: 0.0, imag: 10.0 },
                ComplexSample { real: 1.0, imag: 2.0 },
            ],
        }
    }

    #[test]
    fn manufactured_spectrum_is_two_and_six_point_five() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!((report.eigenvalues[0] - 2.0).abs() < TOL);
        assert!((report.eigenvalues[1] - 6.5).abs() < TOL);
        assert!(report.eigenpair_residual_max < TOL);
        assert!(report.eigenvector_orthogonality_error_max < TOL);
    }

    #[test]
    fn exact_dc_effective_conductance_is_twelve_over_thirteen() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        let expected = 12.0 / 13.0;
        assert!((report.dc_boundary_response[0][0] - expected).abs() < TOL);
        assert!((report.dc_boundary_response[1][1] - expected).abs() < TOL);
        assert!((report.dc_boundary_response[0][1] + expected).abs() < TOL);
        assert!((report.dc_boundary_response[1][0] + expected).abs() < TOL);
    }

    #[test]
    fn full_modal_transfer_matches_direct_dynamic_schur_reference() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!(report.frequency_results.iter()
            .all(|entry| entry.full_modal_vs_direct_relative_error < TOL));
    }

    #[test]
    fn dc_corrected_modal_truncation_preserves_the_full_boundary_operator() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!(report.dc_response_relative_error < TOL);
    }

    #[test]
    fn full_and_reduced_state_space_passivity_matrices_are_positive_semidefinite() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!(report.full_passivity_matrix_min_eigenvalue >= -TOL);
        assert!(report.reduced_passivity_matrix_min_eigenvalue >= -TOL);
        assert_eq!(report.unstable_reduced_pole_count, 0);
    }

    #[test]
    fn complex_frequency_hermitian_parts_are_positive_on_declared_samples() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!(report.frequency_results.iter()
            .all(|entry| entry.full_positive_real_min_eigenvalue >= -TOL));
        assert!(report.frequency_results.iter()
            .all(|entry| entry.reduced_positive_real_min_eigenvalue >= -TOL));
    }

    #[test]
    fn modal_transfer_obeys_complex_conjugate_symmetry() {
        let report = analyze(&fixture(), "fixture-digest".into()).unwrap();
        assert!(report.frequency_results.iter()
            .all(|entry| entry.conjugate_symmetry_error < TOL));
    }

    #[test]
    fn retained_slow_mode_is_better_at_low_frequency_than_fast_mode() {
        let mut slow_fixture = fixture();
        slow_fixture.retained_mode_indices = vec![0];
        let slow = analyze(&slow_fixture, "fixture-digest".into()).unwrap();
        let mut fast_fixture = fixture();
        fast_fixture.retained_mode_indices = vec![1];
        let fast = analyze(&fast_fixture, "fixture-digest".into()).unwrap();
        assert!(slow.frequency_results[1].reduced_vs_full_relative_error
            < fast.frequency_results[1].reduced_vs_full_relative_error);
    }

    #[test]
    fn retaining_all_modes_gives_exact_complex_transfer() {
        let mut all = fixture();
        all.retained_mode_indices = vec![0, 1];
        let report = analyze(&all, "fixture-digest".into()).unwrap();
        assert!(report.frequency_results.iter()
            .all(|entry| entry.reduced_vs_full_relative_error < TOL));
    }

    #[test]
    fn rejects_nonpositive_storage_and_unanchored_interior_components() {
        let mut invalid = fixture();
        invalid.nodes[1].storage = Some(0.0);
        assert!(validate(&invalid).is_err());

        let mut invalid = fixture();
        invalid.nodes.push(Node { node_id: "floating".into(), storage: Some(1.0) });
        assert!(assemble_model(&invalid).is_err());
    }

    #[test]
    fn rejects_invalid_retained_modes_and_left_half_plane_samples() {
        let mut invalid = fixture();
        invalid.retained_mode_indices = vec![7];
        assert!(analyze(&invalid, "fixture-digest".into()).is_err());

        let mut invalid = fixture();
        invalid.laplace_samples[0].real = -1.0;
        assert!(validate(&invalid).is_err());
    }
}
