#![forbid(unsafe_code)]

use std::error::Error;
use std::fmt::{Display, Formatter};

#[derive(Clone, Copy, Debug)]
struct Edge {
    p1: usize,
    p2: usize,
}

#[derive(Clone, Debug)]
struct Case {
    pore_count: usize,
    edges: Vec<Edge>,
    conductances: Vec<f64>,
    inlet_pores: Vec<usize>,
    boundary_pressures: Vec<(usize, f64)>,
    conversion: f64,
    target_permeability: f64,
}

#[derive(Clone, Debug)]
struct AdjointGradientResult {
    pressures: Vec<f64>,
    adjoint: Vec<f64>,
    corrected_gradient: Vec<f64>,
    published_gradient: Vec<f64>,
    permeability: f64,
}

#[derive(Clone, Debug)]
struct Audit {
    permeability: f64,
    pressures: Vec<f64>,
    adjoint: Vec<f64>,
    finite_difference_gradient: Vec<f64>,
    corrected_adjoint_gradient: Vec<f64>,
    published_formula_gradient: Vec<f64>,
}

#[derive(Debug)]
struct AuditError(String);

impl Display for AuditError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl Error for AuditError {}

type Matrix = Vec<Vec<f64>>;

fn solve_dense(matrix: &[Vec<f64>], rhs: &[f64]) -> Result<Vec<f64>, AuditError> {
    let n = matrix.len();
    if n == 0 || rhs.len() != n || matrix.iter().any(|row| row.len() != n) {
        return Err(AuditError(
            "dense solve requires a nonempty square system".into(),
        ));
    }

    let mut a = matrix.to_vec();
    let mut b = rhs.to_vec();
    let scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |largest, value| largest.max(value.abs()));
    if !scale.is_finite() || scale <= f64::MIN_POSITIVE {
        return Err(AuditError("dense solve matrix has invalid scale".into()));
    }
    let pivot_tolerance = scale * 1.0e-14;

    for column in 0..n {
        let pivot = (column..n)
            .max_by(|left, right| a[*left][column].abs().total_cmp(&a[*right][column].abs()))
            .ok_or_else(|| AuditError("dense solve has no pivot candidate".into()))?;
        if a[pivot][column].abs() <= pivot_tolerance {
            return Err(AuditError(
                "dense solve encountered a singular pivot".into(),
            ));
        }
        a.swap(column, pivot);
        b.swap(column, pivot);

        let diagonal = a[column][column];
        for value in a[column].iter_mut().skip(column) {
            *value /= diagonal;
        }
        b[column] /= diagonal;

        let pivot_row = a[column].clone();
        let pivot_rhs = b[column];
        for (row_index, row) in a.iter_mut().enumerate() {
            if row_index == column {
                continue;
            }
            let factor = row[column];
            for (value, pivot_value) in row.iter_mut().zip(pivot_row.iter()).skip(column) {
                *value -= factor * pivot_value;
            }
            b[row_index] -= factor * pivot_rhs;
        }
    }

    if b.iter().any(|value| !value.is_finite()) {
        return Err(AuditError("dense solve returned a non-finite state".into()));
    }
    Ok(b)
}

fn transpose(matrix: &[Vec<f64>]) -> Matrix {
    if matrix.is_empty() {
        return Vec::new();
    }
    let mut output = vec![vec![0.0; matrix.len()]; matrix[0].len()];
    for (row_index, row) in matrix.iter().enumerate() {
        for (column_index, value) in row.iter().enumerate() {
            output[column_index][row_index] = *value;
        }
    }
    output
}

fn is_member(values: &[usize], index: usize) -> bool {
    values.contains(&index)
}

fn build_dirichlet_system(case: &Case) -> Result<(Matrix, Vec<f64>), AuditError> {
    if case.conductances.len() != case.edges.len()
        || case
            .conductances
            .iter()
            .any(|value| !value.is_finite() || *value <= 0.0)
    {
        return Err(AuditError("conductance vector is invalid".into()));
    }

    let mut laplacian = vec![vec![0.0; case.pore_count]; case.pore_count];
    for (edge, conductance) in case.edges.iter().zip(case.conductances.iter()) {
        if edge.p1 >= case.pore_count || edge.p2 >= case.pore_count || edge.p1 == edge.p2 {
            return Err(AuditError("edge endpoint is invalid".into()));
        }
        laplacian[edge.p1][edge.p1] += conductance;
        laplacian[edge.p2][edge.p2] += conductance;
        laplacian[edge.p1][edge.p2] -= conductance;
        laplacian[edge.p2][edge.p1] -= conductance;
    }

    let mut rhs = vec![0.0; case.pore_count];
    // Match the public implementation: replace Dirichlet rows only. The
    // resulting matrix is not symmetric, hence the adjoint solve uses A^T.
    for (pore, pressure) in &case.boundary_pressures {
        if *pore >= case.pore_count || !pressure.is_finite() {
            return Err(AuditError("Dirichlet boundary is invalid".into()));
        }
        laplacian[*pore].fill(0.0);
        laplacian[*pore][*pore] = 1.0;
        rhs[*pore] = *pressure;
    }
    Ok((laplacian, rhs))
}

fn solve_forward(case: &Case) -> Result<(Vec<f64>, f64, Matrix), AuditError> {
    let (matrix, rhs) = build_dirichlet_system(case)?;
    let pressures = solve_dense(&matrix, &rhs)?;
    let mut inlet_flow = 0.0;
    for (edge, conductance) in case.edges.iter().zip(case.conductances.iter()) {
        let signed_flow = conductance * (pressures[edge.p1] - pressures[edge.p2]);
        if is_member(&case.inlet_pores, edge.p1) {
            inlet_flow += signed_flow;
        }
        if is_member(&case.inlet_pores, edge.p2) {
            inlet_flow -= signed_flow;
        }
    }
    Ok((pressures, inlet_flow * case.conversion, matrix))
}

fn objective(case: &Case) -> Result<f64, AuditError> {
    let (_, permeability, _) = solve_forward(case)?;
    Ok(0.5 * (permeability - case.target_permeability).powi(2))
}

fn five_point_derivative(case: &Case, edge_index: usize, epsilon: f64) -> Result<f64, AuditError> {
    let mut plus_one = case.clone();
    let mut plus_two = case.clone();
    let mut minus_one = case.clone();
    let mut minus_two = case.clone();
    plus_one.conductances[edge_index] += epsilon;
    plus_two.conductances[edge_index] += 2.0 * epsilon;
    minus_one.conductances[edge_index] -= epsilon;
    minus_two.conductances[edge_index] -= 2.0 * epsilon;
    Ok(
        (-objective(&plus_two)? + 8.0 * objective(&plus_one)? - 8.0 * objective(&minus_one)?
            + objective(&minus_two)?)
            / (12.0 * epsilon),
    )
}

fn finite_difference_gradient(case: &Case) -> Result<Vec<f64>, AuditError> {
    let mut gradient = Vec::with_capacity(case.conductances.len());
    for (index, conductance) in case.conductances.iter().enumerate() {
        let scale = conductance.abs().max(f64::MIN_POSITIVE);
        let coarse = five_point_derivative(case, index, scale * 1.0e-2)?;
        let fine = five_point_derivative(case, index, scale * 5.0e-3)?;
        // Both five-point stencils have O(h^4) truncation error. Richardson
        // extrapolation removes the leading term without relaxing the FD gate.
        gradient.push((16.0 * fine - coarse) / 15.0);
    }
    Ok(gradient)
}

fn adjoint_gradients(case: &Case) -> Result<AdjointGradientResult, AuditError> {
    let (pressures, permeability, matrix) = solve_forward(case)?;
    let residual = permeability - case.target_permeability;
    let mut d_objective_d_pressure = vec![0.0; case.pore_count];
    let mut p1_inlet = vec![false; case.edges.len()];
    let mut p2_inlet = vec![false; case.edges.len()];

    for (index, (edge, conductance)) in case.edges.iter().zip(case.conductances.iter()).enumerate()
    {
        p1_inlet[index] = is_member(&case.inlet_pores, edge.p1);
        p2_inlet[index] = is_member(&case.inlet_pores, edge.p2);
        if p1_inlet[index] && !p2_inlet[index] {
            d_objective_d_pressure[edge.p2] -= residual * case.conversion * conductance;
        } else if p2_inlet[index] && !p1_inlet[index] {
            d_objective_d_pressure[edge.p1] -= residual * case.conversion * conductance;
        }
    }

    let adjoint = solve_dense(&transpose(&matrix), &d_objective_d_pressure)?;
    let boundary_pores: Vec<usize> = case
        .boundary_pressures
        .iter()
        .map(|(pore, _)| *pore)
        .collect();

    let mut corrected = Vec::with_capacity(case.edges.len());
    let mut published = Vec::with_capacity(case.edges.len());
    for (index, edge) in case.edges.iter().enumerate() {
        let pressure_drop = pressures[edge.p1] - pressures[edge.p2];
        let p1_boundary = is_member(&boundary_pores, edge.p1);
        let p2_boundary = is_member(&boundary_pores, edge.p2);

        let state_term = match (p1_boundary, p2_boundary) {
            (false, false) => -(adjoint[edge.p1] - adjoint[edge.p2]) * pressure_drop,
            (true, false) => adjoint[edge.p2] * pressure_drop,
            (false, true) => -adjoint[edge.p1] * pressure_drop,
            (true, true) => 0.0,
        };

        // Signed direct derivative of inlet flow: add p1-inlet, subtract
        // p2-inlet. The public formula loses this orientation in is_inlet_pore.
        let inlet_orientation =
            (if p1_inlet[index] { 1.0 } else { 0.0 }) - (if p2_inlet[index] { 1.0 } else { 0.0 });
        let direct_term = residual * case.conversion * inlet_orientation * pressure_drop;
        corrected.push(state_term + direct_term);

        // Faithful transcription of the public branch logic:
        // delta_lambda is zero except for one-boundary/one-interior edges.
        let delta_lambda = match (p1_boundary, p2_boundary) {
            (true, false) => adjoint[edge.p2],
            (false, true) => -adjoint[edge.p1],
            _ => 0.0,
        };
        let inlet_term = if p1_inlet[index] || p2_inlet[index] {
            residual * case.conversion * pressure_drop
        } else {
            0.0
        };
        let boundary_throat = p1_boundary || p2_boundary;
        let public_gradient = if boundary_throat {
            delta_lambda * pressure_drop - inlet_term
        } else {
            -delta_lambda * pressure_drop + inlet_term
        };
        published.push(public_gradient);
    }

    Ok(AdjointGradientResult {
        pressures,
        adjoint,
        corrected_gradient: corrected,
        published_gradient: published,
        permeability,
    })
}

fn audit_case(case: &Case) -> Result<Audit, AuditError> {
    let result = adjoint_gradients(case)?;
    Ok(Audit {
        permeability: result.permeability,
        pressures: result.pressures,
        adjoint: result.adjoint,
        finite_difference_gradient: finite_difference_gradient(case)?,
        corrected_adjoint_gradient: result.corrected_gradient,
        published_formula_gradient: result.published_gradient,
    })
}

fn chain_case(reverse_inlet_edge: bool) -> Case {
    let mut edges = vec![
        Edge { p1: 0, p2: 1 },
        Edge { p1: 1, p2: 2 },
        Edge { p1: 2, p2: 3 },
    ];
    if reverse_inlet_edge {
        edges[0] = Edge { p1: 1, p2: 0 };
    }
    Case {
        pore_count: 4,
        edges,
        conductances: vec![2.0, 3.0, 4.0],
        inlet_pores: vec![0],
        boundary_pressures: vec![(0, 20.0), (3, 10.0)],
        conversion: 1.0,
        target_permeability: 5.0,
    }
}

fn branched_case() -> Case {
    Case {
        pore_count: 5,
        edges: vec![
            Edge { p1: 0, p2: 1 },
            Edge { p1: 1, p2: 4 },
            Edge { p1: 0, p2: 2 },
            Edge { p1: 2, p2: 4 },
            Edge { p1: 1, p2: 3 },
            Edge { p1: 3, p2: 2 },
        ],
        conductances: vec![2.0, 1.4, 0.8, 2.3, 0.6, 1.1],
        inlet_pores: vec![0],
        boundary_pressures: vec![(0, 17.0), (4, 8.0)],
        conversion: 1.0,
        target_permeability: 3.7,
    }
}

fn max_relative_difference(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(a, b)| (a - b).abs() / a.abs().max(b.abs()).max(1.0e-12))
        .fold(0.0_f64, f64::max)
}

fn print_audit(name: &str, audit: &Audit) {
    println!("{name}");
    println!("  K = {:.12}", audit.permeability);
    println!("  pressure = {:?}", audit.pressures);
    println!("  adjoint = {:?}", audit.adjoint);
    println!(
        "  finite_difference = {:?}",
        audit.finite_difference_gradient
    );
    println!(
        "  corrected_adjoint = {:?}",
        audit.corrected_adjoint_gradient
    );
    println!("  public_formula = {:?}", audit.published_formula_gradient);
    println!(
        "  corrected_vs_fd_max_relative_error = {:.3e}",
        max_relative_difference(
            &audit.corrected_adjoint_gradient,
            &audit.finite_difference_gradient
        )
    );
    println!(
        "  public_vs_fd_max_relative_error = {:.3e}",
        max_relative_difference(
            &audit.published_formula_gradient,
            &audit.finite_difference_gradient
        )
    );
}

fn main() -> Result<(), Box<dyn Error>> {
    let standard = audit_case(&chain_case(false))?;
    let reversed = audit_case(&chain_case(true))?;
    print_audit("standard edge orientation", &standard);
    print_audit("reversed inlet-edge orientation", &reversed);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_gradient_matches_finite_difference(audit: &Audit) {
        let error = max_relative_difference(
            &audit.corrected_adjoint_gradient,
            &audit.finite_difference_gradient,
        );
        assert!(
            error < 1.0e-6,
            "correct adjoint/FD relative error {error:e}"
        );
    }

    #[test]
    fn corrected_adjoint_matches_finite_difference_for_chain() {
        assert_gradient_matches_finite_difference(&audit_case(&chain_case(false)).unwrap());
    }

    #[test]
    fn corrected_adjoint_matches_finite_difference_when_inlet_edge_is_reversed() {
        assert_gradient_matches_finite_difference(&audit_case(&chain_case(true)).unwrap());
    }

    #[test]
    fn public_formula_omits_internal_internal_conductance_gradient() {
        let audit = audit_case(&chain_case(false)).unwrap();
        assert!(audit.finite_difference_gradient[1].abs() > 1.0);
        assert!(audit.published_formula_gradient[1].abs() < 1.0e-12);
    }

    #[test]
    fn public_formula_inlet_direct_term_depends_on_edge_orientation() {
        let standard = audit_case(&chain_case(false)).unwrap();
        let reversed = audit_case(&chain_case(true)).unwrap();
        let standard_error = max_relative_difference(
            &standard.published_formula_gradient,
            &standard.finite_difference_gradient,
        );
        let reversed_error = max_relative_difference(
            &reversed.published_formula_gradient,
            &reversed.finite_difference_gradient,
        );
        assert!(standard_error > 0.1);
        assert!(reversed_error < standard_error);
    }

    #[test]
    fn corrected_gradient_is_orientation_invariant() {
        let standard = audit_case(&chain_case(false)).unwrap();
        let reversed = audit_case(&chain_case(true)).unwrap();
        assert!(
            max_relative_difference(
                &standard.corrected_adjoint_gradient,
                &reversed.corrected_adjoint_gradient
            ) < 1.0e-10
        );
    }

    #[test]
    fn corrected_adjoint_matches_finite_difference_on_cross_linked_branches() {
        let audit = audit_case(&branched_case()).unwrap();
        assert_gradient_matches_finite_difference(&audit);
        assert!(audit.corrected_adjoint_gradient.len() == 6);
        assert!(audit
            .corrected_adjoint_gradient
            .iter()
            .all(|value| value.is_finite()));
    }

    #[test]
    fn corrected_adjoint_is_invariant_to_edge_order_on_branched_network() {
        let baseline = audit_case(&branched_case()).unwrap();
        let mut permuted = branched_case();
        permuted.edges.reverse();
        permuted.conductances.reverse();
        let reordered = audit_case(&permuted).unwrap();
        assert!(
            max_relative_difference(
                &baseline.finite_difference_gradient,
                &reordered
                    .finite_difference_gradient
                    .iter()
                    .rev()
                    .copied()
                    .collect::<Vec<_>>()
            ) < 1.0e-7
        );
        assert!(
            max_relative_difference(
                &baseline.corrected_adjoint_gradient,
                &reordered
                    .corrected_adjoint_gradient
                    .iter()
                    .rev()
                    .copied()
                    .collect::<Vec<_>>()
            ) < 1.0e-6
        );
    }

    #[test]
    fn corrected_adjoint_matches_finite_difference_after_conductance_rescaling() {
        for factor in [1.0e-6, 1.0e6] {
            let mut case = branched_case();
            for conductance in &mut case.conductances {
                *conductance *= factor;
            }
            let audit = audit_case(&case).unwrap();
            assert_gradient_matches_finite_difference(&audit);
        }
    }

    #[test]
    fn corrected_gradient_is_finite_and_nonzero_on_every_edge() {
        let audit = audit_case(&chain_case(false)).unwrap();
        assert!(audit
            .corrected_adjoint_gradient
            .iter()
            .all(|value| value.is_finite() && value.abs() > 1.0e-8));
    }

    #[test]
    fn finite_difference_and_adjoint_use_same_physical_boundary_values() {
        let audit = audit_case(&chain_case(false)).unwrap();
        assert!((audit.pressures[0] - 20.0).abs() < 1.0e-12);
        assert!((audit.pressures[3] - 10.0).abs() < 1.0e-12);
    }
}
