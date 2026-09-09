// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Extended Kalman Filter for continuous position tracking.
//!
//! State: [x, y, z, vx, vy, vz] (position + velocity in body-fixed frame).
//! Predict: constant-velocity model with configurable process noise.
//! Update: range measurements from anchors (nonlinear observation model).
//!
//! This module preserves the historical serde-compatible `Vec<f64>` covariance
//! wire shape, but checked operations validate the expected 6×6 matrix before
//! use and commit state transactionally only after the candidate result passes
//! numerical validation.
//!
//! Covariance validity is stronger than finite entries plus non-negative
//! diagonal terms: checked authority paths explicitly qualify the complete
//! symmetric matrix as positive semidefinite within a declared tolerance.
//!
//! Important: this filter assumes conditionally independent measurement noise.
//! Unknown cross-correlated peer state estimates should be fused with
//! covariance-bounding methods such as Covariance Intersection before or
//! instead of a direct EKF update.

use serde::{Deserialize, Serialize};

/// EKF state dimension (3D position + 3D velocity).
const STATE_DIM: usize = 6;
const COVARIANCE_LEN: usize = STATE_DIM * STATE_DIM;
const MEASUREMENT_DIM: usize = 3;
const MEASUREMENT_COVARIANCE_LEN: usize = MEASUREMENT_DIM * MEASUREMENT_DIM;
const SYMMETRY_TOLERANCE: f64 = 1e-9;
/// Relative tolerance used to distinguish numerical roundoff from a materially
/// negative covariance eigenvalue.
const PSD_RELATIVE_TOLERANCE: f64 = 1e-10;
/// Absolute floor for PSD classification when covariance scale is very small.
const PSD_ABSOLUTE_TOLERANCE: f64 = 1e-12;
/// Relative off-diagonal convergence target for the deterministic symmetric
/// Jacobi eigensolver used only for covariance qualification.
const JACOBI_RELATIVE_TOLERANCE: f64 = 1e-12;
const JACOBI_MAX_ROTATIONS: usize = 256;

fn idx(row: usize, col: usize) -> usize {
    row * STATE_DIM + col
}

fn idx3(row: usize, col: usize) -> usize {
    row * MEASUREMENT_DIM + col
}

/// Filter state at a given time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterState {
    /// State vector [x, y, z, vx, vy, vz] in meters and m/s.
    pub state: [f64; STATE_DIM],
    /// 6×6 covariance matrix (row-major) stored as Vec for serde compatibility.
    pub covariance: Vec<f64>,
    /// Timestamp of last update (arbitrary units, caller manages).
    pub last_update_time: f64,
    /// Number of accepted measurement updates applied.
    pub update_count: u64,
}

/// Configuration for the position filter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterConfig {
    /// Additive position process-noise density used by the current profile (m²/s).
    pub position_noise: f64,
    /// Additive velocity process-noise density used by the current profile (m²/s³).
    pub velocity_noise: f64,
    /// Maximum accepted covariance diagonal (m²). Crossing this bound is an error.
    pub max_covariance: f64,
    /// Reject updates whose innovation exceeds this many sigma.
    pub innovation_gate_sigma: f64,
}

impl Default for FilterConfig {
    fn default() -> Self {
        Self {
            position_noise: 0.1,
            velocity_noise: 0.01,
            max_covariance: 1e8,
            innovation_gate_sigma: 4.0,
        }
    }
}

/// Observable result of a checked measurement update.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FilterUpdateOutcome {
    Accepted,
    InnovationRejected,
    /// Compatibility-only trust input of exactly zero is interpreted as a
    /// source-admission rejection, never as modified statistical precision.
    SourceRejected,
}

/// Explicit PSD qualification for a covariance matrix.
///
/// `NearSingular` is still mathematically admissible as PSD within tolerance;
/// it is deliberately distinct from estimator/geometry conditioning, which is
/// a separate Q4 contract. `Indeterminate` means the bounded eigensolver could
/// not establish a spectrum and therefore authority-facing validation fails
/// closed.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum CovariancePsdStatus {
    Valid {
        min_eigenvalue: f64,
        max_eigenvalue: f64,
        tolerance: f64,
    },
    NearSingular {
        min_eigenvalue: f64,
        max_eigenvalue: f64,
        tolerance: f64,
    },
    Indeterminate,
    Invalid {
        min_eigenvalue: f64,
        max_eigenvalue: f64,
        tolerance: f64,
    },
}

/// Typed errors for authority-facing EKF operations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FilterError {
    InvalidInitialState,
    InvalidConfiguration,
    InvalidCovarianceShape { have: usize, expected: usize },
    NonFiniteState,
    NonFiniteCovariance,
    AsymmetricCovariance,
    NegativeVariance { index: usize, value: f64 },
    CovarianceLimitExceeded { index: usize, value: f64, max: f64 },
    IndefiniteCovariance { min_eigenvalue: f64, tolerance: f64 },
    CovarianceQualificationIndeterminate,
    InvalidMeasurementCovariance,
    InnovationCovarianceIndeterminate,
    InvalidTimeStep { dt: f64 },
    InvalidMeasurement { field: &'static str, value: f64 },
    InvalidAnchor { component: usize, value: f64 },
    InvalidTrustWeight { value: f64 },
    DegenerateObservation,
    NonFiniteComputation,
    CounterOverflow,
}

impl std::fmt::Display for FilterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidInitialState => write!(f, "initial filter state is invalid"),
            Self::InvalidConfiguration => write!(f, "filter configuration is invalid"),
            Self::InvalidCovarianceShape { have, expected } => {
                write!(f, "covariance has {have} elements; expected {expected}")
            }
            Self::NonFiniteState => write!(f, "filter state contains non-finite values"),
            Self::NonFiniteCovariance => write!(f, "filter covariance contains non-finite values"),
            Self::AsymmetricCovariance => {
                write!(f, "filter covariance is not symmetric within tolerance")
            }
            Self::NegativeVariance { index, value } => {
                write!(f, "covariance diagonal {index} is negative: {value}")
            }
            Self::CovarianceLimitExceeded { index, value, max } => {
                write!(f, "covariance diagonal {index}={value} exceeds configured maximum {max}")
            }
            Self::IndefiniteCovariance {
                min_eigenvalue,
                tolerance,
            } => write!(
                f,
                "covariance is indefinite: minimum eigenvalue {min_eigenvalue} is below -{tolerance}"
            ),
            Self::CovarianceQualificationIndeterminate => {
                write!(f, "covariance PSD qualification did not converge")
            }
            Self::InvalidMeasurementCovariance => write!(
                f,
                "3D measurement covariance must be finite, symmetric, and positive definite"
            ),
            Self::InnovationCovarianceIndeterminate => {
                write!(f, "3D innovation covariance could not be factored")
            }
            Self::InvalidTimeStep { dt } => {
                write!(f, "time step must be finite and positive, got {dt}")
            }
            Self::InvalidMeasurement { field, value } => {
                write!(f, "invalid measurement field {field}: {value}")
            }
            Self::InvalidAnchor { component, value } => {
                write!(f, "anchor component {component} is invalid: {value}")
            }
            Self::InvalidTrustWeight { value } => {
                write!(f, "trust weight must be finite and in [0,1], got {value}")
            }
            Self::DegenerateObservation => write!(f, "observation geometry is degenerate"),
            Self::NonFiniteComputation => write!(f, "EKF computation produced non-finite arithmetic"),
            Self::CounterOverflow => write!(f, "filter update counter overflow"),
        }
    }
}

impl std::error::Error for FilterError {}

fn validate_config(config: &FilterConfig) -> Result<(), FilterError> {
    if !config.position_noise.is_finite()
        || config.position_noise < 0.0
        || !config.velocity_noise.is_finite()
        || config.velocity_noise < 0.0
        || !config.max_covariance.is_finite()
        || config.max_covariance <= 0.0
        || !config.innovation_gate_sigma.is_finite()
        || config.innovation_gate_sigma <= 0.0
    {
        return Err(FilterError::InvalidConfiguration);
    }
    Ok(())
}

fn validate_covariance_structure(covariance: &[f64], max: f64) -> Result<(), FilterError> {
    if covariance.len() != COVARIANCE_LEN {
        return Err(FilterError::InvalidCovarianceShape {
            have: covariance.len(),
            expected: COVARIANCE_LEN,
        });
    }
    if covariance.iter().any(|value| !value.is_finite()) {
        return Err(FilterError::NonFiniteCovariance);
    }
    for row in 0..STATE_DIM {
        for col in (row + 1)..STATE_DIM {
            let a = covariance[idx(row, col)];
            let b = covariance[idx(col, row)];
            let scale = a.abs().max(b.abs()).max(1.0);
            if (a - b).abs() > SYMMETRY_TOLERANCE * scale {
                return Err(FilterError::AsymmetricCovariance);
            }
        }
        let variance = covariance[idx(row, row)];
        if variance < 0.0 {
            return Err(FilterError::NegativeVariance {
                index: row,
                value: variance,
            });
        }
        if variance > max {
            return Err(FilterError::CovarianceLimitExceeded {
                index: row,
                value: variance,
                max,
            });
        }
    }
    Ok(())
}

fn covariance_psd_status(covariance: &[f64]) -> CovariancePsdStatus {
    let mut matrix = [0.0_f64; COVARIANCE_LEN];
    matrix.copy_from_slice(covariance);

    let matrix_scale = matrix
        .iter()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    let convergence_tolerance = JACOBI_RELATIVE_TOLERANCE * matrix_scale.max(1.0);

    let mut converged = false;
    for _ in 0..JACOBI_MAX_ROTATIONS {
        let mut p = 0;
        let mut q = 1;
        let mut max_off_diagonal = 0.0_f64;
        for row in 0..STATE_DIM {
            for col in (row + 1)..STATE_DIM {
                let value = matrix[idx(row, col)].abs();
                if value > max_off_diagonal {
                    max_off_diagonal = value;
                    p = row;
                    q = col;
                }
            }
        }

        if !max_off_diagonal.is_finite() {
            return CovariancePsdStatus::Indeterminate;
        }
        if max_off_diagonal <= convergence_tolerance {
            converged = true;
            break;
        }

        let app = matrix[idx(p, p)];
        let aqq = matrix[idx(q, q)];
        let apq = matrix[idx(p, q)];
        if !app.is_finite() || !aqq.is_finite() || !apq.is_finite() || apq == 0.0 {
            return CovariancePsdStatus::Indeterminate;
        }

        // For a real symmetric matrix, a Jacobi rotation with
        // theta = 0.5 * atan2(2*apq, aqq-app) diagonalizes the selected 2×2
        // plane. The equivalent atan2 form below avoids explicitly doubling
        // apq, which keeps extreme finite inputs from overflowing merely while
        // computing the angle.
        let theta = 0.5 * apq.atan2(0.5 * (aqq - app));
        let c = theta.cos();
        let s = theta.sin();
        if !c.is_finite() || !s.is_finite() {
            return CovariancePsdStatus::Indeterminate;
        }

        for k in 0..STATE_DIM {
            if k == p || k == q {
                continue;
            }
            let akp = matrix[idx(k, p)];
            let akq = matrix[idx(k, q)];
            let new_kp = c * akp - s * akq;
            let new_kq = s * akp + c * akq;
            if !new_kp.is_finite() || !new_kq.is_finite() {
                return CovariancePsdStatus::Indeterminate;
            }
            matrix[idx(k, p)] = new_kp;
            matrix[idx(p, k)] = new_kp;
            matrix[idx(k, q)] = new_kq;
            matrix[idx(q, k)] = new_kq;
        }

        let c2 = c * c;
        let s2 = s * s;
        let sc2 = 2.0 * s * c;
        let new_app = c2 * app - sc2 * apq + s2 * aqq;
        let new_aqq = s2 * app + sc2 * apq + c2 * aqq;
        if !new_app.is_finite() || !new_aqq.is_finite() {
            return CovariancePsdStatus::Indeterminate;
        }
        matrix[idx(p, p)] = new_app;
        matrix[idx(q, q)] = new_aqq;
        matrix[idx(p, q)] = 0.0;
        matrix[idx(q, p)] = 0.0;
    }

    if !converged {
        let remaining = (0..STATE_DIM)
            .flat_map(|row| ((row + 1)..STATE_DIM).map(move |col| (row, col)))
            .map(|(row, col)| matrix[idx(row, col)].abs())
            .fold(0.0_f64, f64::max);
        if remaining > convergence_tolerance || !remaining.is_finite() {
            return CovariancePsdStatus::Indeterminate;
        }
    }

    let mut min_eigenvalue = f64::INFINITY;
    let mut max_eigenvalue = f64::NEG_INFINITY;
    for i in 0..STATE_DIM {
        let value = matrix[idx(i, i)];
        if !value.is_finite() {
            return CovariancePsdStatus::Indeterminate;
        }
        min_eigenvalue = min_eigenvalue.min(value);
        max_eigenvalue = max_eigenvalue.max(value);
    }

    let tolerance =
        PSD_ABSOLUTE_TOLERANCE.max(PSD_RELATIVE_TOLERANCE * max_eigenvalue.abs().max(1.0));
    if min_eigenvalue < -tolerance {
        CovariancePsdStatus::Invalid {
            min_eigenvalue,
            max_eigenvalue,
            tolerance,
        }
    } else if min_eigenvalue <= tolerance {
        CovariancePsdStatus::NearSingular {
            min_eigenvalue,
            max_eigenvalue,
            tolerance,
        }
    } else {
        CovariancePsdStatus::Valid {
            min_eigenvalue,
            max_eigenvalue,
            tolerance,
        }
    }
}

fn validate_covariance(covariance: &[f64], max: f64) -> Result<(), FilterError> {
    validate_covariance_structure(covariance, max)?;
    match covariance_psd_status(covariance) {
        CovariancePsdStatus::Valid { .. } | CovariancePsdStatus::NearSingular { .. } => Ok(()),
        CovariancePsdStatus::Invalid {
            min_eigenvalue,
            tolerance,
            ..
        } => Err(FilterError::IndefiniteCovariance {
            min_eigenvalue,
            tolerance,
        }),
        CovariancePsdStatus::Indeterminate => {
            Err(FilterError::CovarianceQualificationIndeterminate)
        }
    }
}

fn validate_filter_state(state: &FilterState, config: &FilterConfig) -> Result<(), FilterError> {
    validate_config(config)?;
    if state.state.iter().any(|value| !value.is_finite()) || !state.last_update_time.is_finite() {
        return Err(FilterError::NonFiniteState);
    }
    validate_covariance(&state.covariance, config.max_covariance)
}

fn mat_mul(a: &[f64], b: &[f64]) -> Result<Vec<f64>, FilterError> {
    if a.len() != COVARIANCE_LEN || b.len() != COVARIANCE_LEN {
        return Err(FilterError::InvalidCovarianceShape {
            have: a.len().min(b.len()),
            expected: COVARIANCE_LEN,
        });
    }
    let mut out = vec![0.0; COVARIANCE_LEN];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            let mut sum = 0.0;
            for k in 0..STATE_DIM {
                sum += a[idx(row, k)] * b[idx(k, col)];
            }
            if !sum.is_finite() {
                return Err(FilterError::NonFiniteComputation);
            }
            out[idx(row, col)] = sum;
        }
    }
    Ok(out)
}

fn transpose(matrix: &[f64]) -> Result<Vec<f64>, FilterError> {
    if matrix.len() != COVARIANCE_LEN {
        return Err(FilterError::InvalidCovarianceShape {
            have: matrix.len(),
            expected: COVARIANCE_LEN,
        });
    }
    let mut out = vec![0.0; COVARIANCE_LEN];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            out[idx(col, row)] = matrix[idx(row, col)];
        }
    }
    Ok(out)
}

fn identity_matrix() -> Vec<f64> {
    let mut out = vec![0.0; COVARIANCE_LEN];
    for i in 0..STATE_DIM {
        out[idx(i, i)] = 1.0;
    }
    out
}

fn symmetrize(matrix: &mut [f64]) {
    for row in 0..STATE_DIM {
        for col in (row + 1)..STATE_DIM {
            let mean = 0.5 * (matrix[idx(row, col)] + matrix[idx(col, row)]);
            matrix[idx(row, col)] = mean;
            matrix[idx(col, row)] = mean;
        }
    }
}

fn cholesky_3x3(matrix: &[f64; MEASUREMENT_COVARIANCE_LEN]) -> Option<[f64; MEASUREMENT_COVARIANCE_LEN]> {
    if matrix.iter().any(|value| !value.is_finite()) {
        return None;
    }
    let scale = matrix
        .iter()
        .fold(0.0_f64, |current, value| current.max(value.abs()));
    let tolerance = PSD_ABSOLUTE_TOLERANCE.max(PSD_RELATIVE_TOLERANCE * scale);
    let mut lower = [0.0_f64; MEASUREMENT_COVARIANCE_LEN];

    for row in 0..MEASUREMENT_DIM {
        for col in 0..=row {
            let mut residual = matrix[idx3(row, col)];
            for k in 0..col {
                residual -= lower[idx3(row, k)] * lower[idx3(col, k)];
            }
            if !residual.is_finite() {
                return None;
            }
            if row == col {
                if residual <= tolerance {
                    return None;
                }
                lower[idx3(row, col)] = residual.sqrt();
            } else {
                let diagonal = lower[idx3(col, col)];
                if !diagonal.is_finite() || diagonal <= 0.0 {
                    return None;
                }
                let value = residual / diagonal;
                if !value.is_finite() {
                    return None;
                }
                lower[idx3(row, col)] = value;
            }
        }
    }
    Some(lower)
}

fn solve_cholesky_3x3(
    lower: &[f64; MEASUREMENT_COVARIANCE_LEN],
    rhs: [f64; MEASUREMENT_DIM],
) -> Option<[f64; MEASUREMENT_DIM]> {
    let mut y = [0.0_f64; MEASUREMENT_DIM];
    for row in 0..MEASUREMENT_DIM {
        let mut value = rhs[row];
        for col in 0..row {
            value -= lower[idx3(row, col)] * y[col];
        }
        let diagonal = lower[idx3(row, row)];
        if !value.is_finite() || !diagonal.is_finite() || diagonal <= 0.0 {
            return None;
        }
        y[row] = value / diagonal;
        if !y[row].is_finite() {
            return None;
        }
    }

    let mut x = [0.0_f64; MEASUREMENT_DIM];
    for row in (0..MEASUREMENT_DIM).rev() {
        let mut value = y[row];
        for col in (row + 1)..MEASUREMENT_DIM {
            value -= lower[idx3(col, row)] * x[col];
        }
        let diagonal = lower[idx3(row, row)];
        if !value.is_finite() || !diagonal.is_finite() || diagonal <= 0.0 {
            return None;
        }
        x[row] = value / diagonal;
        if !x[row].is_finite() {
            return None;
        }
    }
    Some(x)
}

fn validate_measurement_covariance(
    covariance: &[f64; MEASUREMENT_COVARIANCE_LEN],
) -> Result<(), FilterError> {
    if covariance.iter().any(|value| !value.is_finite()) {
        return Err(FilterError::InvalidMeasurementCovariance);
    }
    for row in 0..MEASUREMENT_DIM {
        for col in (row + 1)..MEASUREMENT_DIM {
            let a = covariance[idx3(row, col)];
            let b = covariance[idx3(col, row)];
            let scale = a.abs().max(b.abs()).max(1.0);
            if (a - b).abs() > SYMMETRY_TOLERANCE * scale {
                return Err(FilterError::InvalidMeasurementCovariance);
            }
        }
    }
    cholesky_3x3(covariance)
        .map(|_| ())
        .ok_or(FilterError::InvalidMeasurementCovariance)
}

/// Extended Kalman Filter for position tracking.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionFilter {
    pub state: FilterState,
    pub config: FilterConfig,
}

impl PositionFilter {
    /// Checked constructor for authority-facing callers.
    pub fn try_new(
        initial_position: [f64; 3],
        initial_sigma_m: f64,
        config: FilterConfig,
    ) -> Result<Self, FilterError> {
        validate_config(&config)?;
        if initial_position.iter().any(|value| !value.is_finite())
            || !initial_sigma_m.is_finite()
            || initial_sigma_m <= 0.0
        {
            return Err(FilterError::InvalidInitialState);
        }
        let pos_var = initial_sigma_m * initial_sigma_m;
        if !pos_var.is_finite() || pos_var > config.max_covariance {
            return Err(FilterError::InvalidInitialState);
        }

        let mut covariance = vec![0.0_f64; COVARIANCE_LEN];
        let vel_var = 100.0;
        for i in 0..3 {
            covariance[idx(i, i)] = pos_var;
            covariance[idx(i + 3, i + 3)] = vel_var;
        }

        let filter = Self {
            state: FilterState {
                state: [
                    initial_position[0],
                    initial_position[1],
                    initial_position[2],
                    0.0,
                    0.0,
                    0.0,
                ],
                covariance,
                last_update_time: 0.0,
                update_count: 0,
            },
            config,
        };
        validate_filter_state(&filter.state, &filter.config)?;
        Ok(filter)
    }

    /// Legacy constructor retained for source compatibility.
    ///
    /// New authority-facing code should use [`Self::try_new`]. This wrapper
    /// panics on invalid construction rather than manufacturing a usable state.
    pub fn new(initial_position: [f64; 3], initial_sigma_m: f64, config: FilterConfig) -> Self {
        Self::try_new(initial_position, initial_sigma_m, config)
            .expect("invalid PositionFilter construction; use try_new for fallible handling")
    }

    /// Checked constant-velocity prediction using the full covariance equation:
    /// `P' = F P F^T + Q`.
    pub fn predict_checked(&mut self, dt: f64) -> Result<(), FilterError> {
        validate_filter_state(&self.state, &self.config)?;
        if !dt.is_finite() || dt <= 0.0 {
            return Err(FilterError::InvalidTimeStep { dt });
        }

        let mut candidate_state = self.state.state;
        candidate_state[0] += candidate_state[3] * dt;
        candidate_state[1] += candidate_state[4] * dt;
        candidate_state[2] += candidate_state[5] * dt;
        if candidate_state.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }

        let mut f = identity_matrix();
        f[idx(0, 3)] = dt;
        f[idx(1, 4)] = dt;
        f[idx(2, 5)] = dt;

        let fp = mat_mul(&f, &self.state.covariance)?;
        let ft = transpose(&f)?;
        let mut candidate_covariance = mat_mul(&fp, &ft)?;

        let q_pos = self.config.position_noise * dt;
        let q_vel = self.config.velocity_noise * dt;
        if !q_pos.is_finite() || !q_vel.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        for i in 0..3 {
            candidate_covariance[idx(i, i)] += q_pos;
            candidate_covariance[idx(i + 3, i + 3)] += q_vel;
        }
        symmetrize(&mut candidate_covariance);
        validate_covariance(&candidate_covariance, self.config.max_covariance)?;

        let candidate_time = self.state.last_update_time + dt;
        if !candidate_time.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }

        self.state.state = candidate_state;
        self.state.covariance = candidate_covariance;
        self.state.last_update_time = candidate_time;
        Ok(())
    }

    /// Legacy prediction wrapper. Invalid prediction input leaves state unchanged.
    pub fn predict(&mut self, dt: f64) {
        let _ = self.predict_checked(dt);
    }

    fn scalar_update_checked(
        &mut self,
        h: [f64; STATE_DIM],
        observed_value: f64,
        predicted_value: f64,
        variance: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        validate_filter_state(&self.state, &self.config)?;
        if !observed_value.is_finite() {
            return Err(FilterError::InvalidMeasurement {
                field: "observed_value",
                value: observed_value,
            });
        }
        if !predicted_value.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        if !variance.is_finite() || variance <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "variance",
                value: variance,
            });
        }
        if h.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }

        let innovation = observed_value - predicted_value;
        if !innovation.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }

        let mut ph_t = [0.0; STATE_DIM];
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                ph_t[row] += self.state.covariance[idx(row, col)] * h[col];
            }
        }
        let mut s = variance;
        for i in 0..STATE_DIM {
            s += h[i] * ph_t[i];
        }
        if !s.is_finite() || s <= 0.0 {
            return Err(FilterError::NonFiniteComputation);
        }

        if innovation.abs() > self.config.innovation_gate_sigma * s.sqrt() {
            return Ok(FilterUpdateOutcome::InnovationRejected);
        }

        let mut k = [0.0; STATE_DIM];
        for i in 0..STATE_DIM {
            k[i] = ph_t[i] / s;
        }
        if k.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }

        let mut candidate_state = self.state.state;
        for i in 0..STATE_DIM {
            candidate_state[i] += k[i] * innovation;
        }
        if candidate_state.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }

        // Joseph stabilized covariance update:
        // P' = (I-KH) P (I-KH)^T + K R K^T
        let mut a = identity_matrix();
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                a[idx(row, col)] -= k[row] * h[col];
            }
        }
        let ap = mat_mul(&a, &self.state.covariance)?;
        let at = transpose(&a)?;
        let mut candidate_covariance = mat_mul(&ap, &at)?;
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                candidate_covariance[idx(row, col)] += k[row] * variance * k[col];
            }
        }
        symmetrize(&mut candidate_covariance);
        validate_covariance(&candidate_covariance, self.config.max_covariance)?;

        let next_count = self
            .state
            .update_count
            .checked_add(1)
            .ok_or(FilterError::CounterOverflow)?;

        self.state.state = candidate_state;
        self.state.covariance = candidate_covariance;
        self.state.update_count = next_count;
        Ok(FilterUpdateOutcome::Accepted)
    }

    fn vector_linear_update_checked(
        &mut self,
        state_offset: usize,
        observed: [f64; MEASUREMENT_DIM],
        measurement_covariance: [f64; MEASUREMENT_COVARIANCE_LEN],
    ) -> Result<FilterUpdateOutcome, FilterError> {
        if state_offset + MEASUREMENT_DIM > STATE_DIM {
            return Err(FilterError::InvalidMeasurement {
                field: "state_offset",
                value: state_offset as f64,
            });
        }
        validate_filter_state(&self.state, &self.config)?;
        if observed.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::InvalidMeasurement {
                field: "vector_observation",
                value: f64::NAN,
            });
        }
        validate_measurement_covariance(&measurement_covariance)?;

        let mut innovation = [0.0_f64; MEASUREMENT_DIM];
        for axis in 0..MEASUREMENT_DIM {
            innovation[axis] = observed[axis] - self.state.state[state_offset + axis];
            if !innovation[axis].is_finite() {
                return Err(FilterError::NonFiniteComputation);
            }
        }

        // PH^T is simply the three covariance columns selected by the linear
        // observation matrix H.
        let mut ph_t = [[0.0_f64; MEASUREMENT_DIM]; STATE_DIM];
        for row in 0..STATE_DIM {
            for axis in 0..MEASUREMENT_DIM {
                ph_t[row][axis] = self.state.covariance[idx(row, state_offset + axis)];
            }
        }

        let mut innovation_covariance = [0.0_f64; MEASUREMENT_COVARIANCE_LEN];
        for row in 0..MEASUREMENT_DIM {
            for col in 0..MEASUREMENT_DIM {
                innovation_covariance[idx3(row, col)] = self.state.covariance
                    [idx(state_offset + row, state_offset + col)]
                    + measurement_covariance[idx3(row, col)];
            }
        }
        let innovation_factor = cholesky_3x3(&innovation_covariance)
            .ok_or(FilterError::InnovationCovarianceIndeterminate)?;

        let normalized_innovation = solve_cholesky_3x3(&innovation_factor, innovation)
            .ok_or(FilterError::InnovationCovarianceIndeterminate)?;
        let mut nis = 0.0_f64;
        for axis in 0..MEASUREMENT_DIM {
            nis += innovation[axis] * normalized_innovation[axis];
        }
        if !nis.is_finite() || nis < 0.0 {
            return Err(FilterError::NonFiniteComputation);
        }
        // Preserve the scalar gate's meaning as an RMS standardized innovation:
        // mean(z^T S^-1 z) <= gate_sigma^2 for the 3-vector profile.
        let gate = self.config.innovation_gate_sigma * self.config.innovation_gate_sigma
            * MEASUREMENT_DIM as f64;
        if !gate.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        if nis > gate {
            return Ok(FilterUpdateOutcome::InnovationRejected);
        }

        let mut gain = [[0.0_f64; MEASUREMENT_DIM]; STATE_DIM];
        for row in 0..STATE_DIM {
            gain[row] = solve_cholesky_3x3(&innovation_factor, ph_t[row])
                .ok_or(FilterError::InnovationCovarianceIndeterminate)?;
        }

        let mut candidate_state = self.state.state;
        for row in 0..STATE_DIM {
            for axis in 0..MEASUREMENT_DIM {
                candidate_state[row] += gain[row][axis] * innovation[axis];
            }
        }
        if candidate_state.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }

        // Vector Joseph update:
        // P' = (I-KH) P (I-KH)^T + K R K^T
        let mut a = identity_matrix();
        for row in 0..STATE_DIM {
            for axis in 0..MEASUREMENT_DIM {
                a[idx(row, state_offset + axis)] -= gain[row][axis];
            }
        }
        let ap = mat_mul(&a, &self.state.covariance)?;
        let at = transpose(&a)?;
        let mut candidate_covariance = mat_mul(&ap, &at)?;

        let mut gain_times_r = [[0.0_f64; MEASUREMENT_DIM]; STATE_DIM];
        for row in 0..STATE_DIM {
            for col in 0..MEASUREMENT_DIM {
                for k in 0..MEASUREMENT_DIM {
                    gain_times_r[row][col] +=
                        gain[row][k] * measurement_covariance[idx3(k, col)];
                }
            }
        }
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                let mut krkt = 0.0_f64;
                for axis in 0..MEASUREMENT_DIM {
                    krkt += gain_times_r[row][axis] * gain[col][axis];
                }
                if !krkt.is_finite() {
                    return Err(FilterError::NonFiniteComputation);
                }
                candidate_covariance[idx(row, col)] += krkt;
            }
        }
        symmetrize(&mut candidate_covariance);
        validate_covariance(&candidate_covariance, self.config.max_covariance)?;

        let next_count = self
            .state
            .update_count
            .checked_add(1)
            .ok_or(FilterError::CounterOverflow)?;
        self.state.state = candidate_state;
        self.state.covariance = candidate_covariance;
        self.state.update_count = next_count;
        Ok(FilterUpdateOutcome::Accepted)
    }

    /// Checked range update whose mathematical precision is determined only by
    /// `range_sigma`; source trust/reputation is deliberately not an argument.
    pub fn update_range_checked(
        &mut self,
        anchor: &[f64; 3],
        measured_range: f64,
        range_sigma: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        for (component, value) in anchor.iter().copied().enumerate() {
            if !value.is_finite() {
                return Err(FilterError::InvalidAnchor { component, value });
            }
        }
        if !measured_range.is_finite() || measured_range <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "measured_range",
                value: measured_range,
            });
        }
        if !range_sigma.is_finite() || range_sigma <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "range_sigma",
                value: range_sigma,
            });
        }
        validate_filter_state(&self.state, &self.config)?;

        let dx = self.state.state[0] - anchor[0];
        let dy = self.state.state[1] - anchor[1];
        let dz = self.state.state[2] - anchor[2];
        let predicted_range = (dx * dx + dy * dy + dz * dz).sqrt();
        if !predicted_range.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        if predicted_range < 1e-10 {
            return Err(FilterError::DegenerateObservation);
        }

        let h = [
            dx / predicted_range,
            dy / predicted_range,
            dz / predicted_range,
            0.0,
            0.0,
            0.0,
        ];
        let variance = range_sigma * range_sigma;
        if !variance.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        self.scalar_update_checked(h, measured_range, predicted_range, variance)
    }

    /// Checked compatibility entry point retaining the historical trust field.
    ///
    /// Trust acts only as a source-admission hint: zero rejects the source;
    /// positive values do not modify sigma/covariance.
    pub fn update_with_trust_checked(
        &mut self,
        anchor: &[f64; 3],
        measured_range: f64,
        range_sigma: f64,
        trust_weight: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        if !trust_weight.is_finite() || !(0.0..=1.0).contains(&trust_weight) {
            return Err(FilterError::InvalidTrustWeight {
                value: trust_weight,
            });
        }
        if trust_weight == 0.0 {
            return Ok(FilterUpdateOutcome::SourceRejected);
        }
        self.update_range_checked(anchor, measured_range, range_sigma)
    }

    /// Legacy range-update wrapper. Prefer [`Self::update_range_checked`].
    pub fn update(
        &mut self,
        anchor: &[f64; 3],
        measured_range: f64,
        range_sigma: f64,
        trust_weight: f64,
    ) {
        let _ = self.update_with_trust_checked(anchor, measured_range, range_sigma, trust_weight);
    }

    fn update_linear_observation_checked(
        &mut self,
        state_index: usize,
        observed_value: f64,
        variance: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        if state_index >= STATE_DIM {
            return Err(FilterError::InvalidMeasurement {
                field: "state_index",
                value: state_index as f64,
            });
        }
        let mut h = [0.0; STATE_DIM];
        h[state_index] = 1.0;
        let predicted = self.state.state[state_index];
        self.scalar_update_checked(h, observed_value, predicted, variance)
    }

    /// Canonical simultaneous absolute-position update preserving the complete
    /// 3×3 measurement covariance, including cross-axis correlation.
    pub fn update_absolute_position_vector_checked(
        &mut self,
        position: [f64; MEASUREMENT_DIM],
        covariance: [f64; MEASUREMENT_COVARIANCE_LEN],
    ) -> Result<FilterUpdateOutcome, FilterError> {
        self.vector_linear_update_checked(0, position, covariance)
    }

    /// Source-compatible checked absolute-position entry point.
    ///
    /// The historical return shape is retained for callers stacked below #423,
    /// but all three entries now describe one joint vector decision and the
    /// filter update counter advances only once on acceptance.
    pub fn update_absolute_position_checked(
        &mut self,
        position: [f64; 3],
        covariance: [f64; 9],
    ) -> Result<[FilterUpdateOutcome; 3], FilterError> {
        let outcome = self.update_absolute_position_vector_checked(position, covariance)?;
        Ok([outcome; 3])
    }

    pub fn update_absolute_position(&mut self, position: [f64; 3], covariance: [f64; 9]) {
        let _ = self.update_absolute_position_checked(position, covariance);
    }

    /// Canonical simultaneous absolute-velocity update preserving the complete
    /// 3×3 measurement covariance, including cross-axis correlation.
    pub fn update_absolute_velocity_vector_checked(
        &mut self,
        velocity: [f64; MEASUREMENT_DIM],
        covariance: [f64; MEASUREMENT_COVARIANCE_LEN],
    ) -> Result<FilterUpdateOutcome, FilterError> {
        self.vector_linear_update_checked(3, velocity, covariance)
    }

    /// Source-compatible checked absolute-velocity entry point. See the
    /// position counterpart for the joint-decision return-shape semantics.
    pub fn update_absolute_velocity_checked(
        &mut self,
        velocity: [f64; 3],
        covariance: [f64; 9],
    ) -> Result<[FilterUpdateOutcome; 3], FilterError> {
        let outcome = self.update_absolute_velocity_vector_checked(velocity, covariance)?;
        Ok([outcome; 3])
    }

    pub fn update_absolute_velocity(&mut self, velocity: [f64; 3], covariance: [f64; 9]) {
        let _ = self.update_absolute_velocity_checked(velocity, covariance);
    }

    pub fn update_depth_checked(
        &mut self,
        depth_m: f64,
        sigma_m: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        if !depth_m.is_finite() || !sigma_m.is_finite() || sigma_m <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "depth",
                value: depth_m,
            });
        }
        let variance = sigma_m * sigma_m;
        if !variance.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        self.update_linear_observation_checked(2, depth_m, variance)
    }

    pub fn update_depth(&mut self, depth_m: f64, sigma_m: f64) {
        let _ = self.update_depth_checked(depth_m, sigma_m);
    }

    pub fn update_relative_position_checked(
        &mut self,
        reference_position: [f64; 3],
        relative_position: [f64; 3],
        covariance: [f64; 9],
    ) -> Result<[FilterUpdateOutcome; 3], FilterError> {
        if reference_position.iter().any(|value| !value.is_finite())
            || relative_position.iter().any(|value| !value.is_finite())
        {
            return Err(FilterError::InvalidMeasurement {
                field: "relative_position",
                value: f64::NAN,
            });
        }
        let absolute = [
            reference_position[0] + relative_position[0],
            reference_position[1] + relative_position[1],
            reference_position[2] + relative_position[2],
        ];
        if absolute.iter().any(|value| !value.is_finite()) {
            return Err(FilterError::NonFiniteComputation);
        }
        self.update_absolute_position_checked(absolute, covariance)
    }

    pub fn update_relative_position(
        &mut self,
        reference_position: [f64; 3],
        relative_position: [f64; 3],
        covariance: [f64; 9],
    ) {
        let _ = self.update_relative_position_checked(
            reference_position,
            relative_position,
            covariance,
        );
    }

    pub fn position(&self) -> [f64; 3] {
        [
            self.state.state[0],
            self.state.state[1],
            self.state.state[2],
        ]
    }

    pub fn velocity(&self) -> [f64; 3] {
        [
            self.state.state[3],
            self.state.state[4],
            self.state.state[5],
        ]
    }

    pub fn update_barometer_checked(
        &mut self,
        pressure_hpa: f64,
        reference_hpa: f64,
        sigma_m: f64,
    ) -> Result<FilterUpdateOutcome, FilterError> {
        if !pressure_hpa.is_finite() || pressure_hpa < 100.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "pressure_hpa",
                value: pressure_hpa,
            });
        }
        if !reference_hpa.is_finite() || reference_hpa <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "reference_hpa",
                value: reference_hpa,
            });
        }
        if !sigma_m.is_finite() || sigma_m <= 0.0 {
            return Err(FilterError::InvalidMeasurement {
                field: "barometer_sigma",
                value: sigma_m,
            });
        }
        let measured_alt = crate::dead_reckoning::barometric_altitude(pressure_hpa, reference_hpa);
        if !measured_alt.is_finite() {
            return Err(FilterError::NonFiniteComputation);
        }
        self.update_linear_observation_checked(2, measured_alt, sigma_m * sigma_m)
    }

    pub fn update_barometer(&mut self, pressure_hpa: f64, reference_hpa: f64, sigma_m: f64) {
        let _ = self.update_barometer_checked(pressure_hpa, reference_hpa, sigma_m);
    }

    /// Qualify the complete current covariance matrix as PSD without collapsing
    /// the result to a boolean. `NearSingular` is accepted by filter validation;
    /// downstream policy can inspect it without confusing it with invalidity.
    pub fn covariance_psd_status_checked(&self) -> Result<CovariancePsdStatus, FilterError> {
        validate_config(&self.config)?;
        validate_covariance_structure(&self.state.covariance, self.config.max_covariance)?;
        Ok(covariance_psd_status(&self.state.covariance))
    }

    /// Get 1-sigma position uncertainty (meters).
    pub fn position_sigma_checked(&self) -> Result<f64, FilterError> {
        validate_filter_state(&self.state, &self.config)?;
        let variance = self.state.covariance[idx(0, 0)]
            + self.state.covariance[idx(1, 1)]
            + self.state.covariance[idx(2, 2)];
        if !variance.is_finite() || variance < 0.0 {
            return Err(FilterError::NonFiniteComputation);
        }
        Ok(variance.sqrt())
    }

    pub fn position_sigma(&self) -> f64 {
        self.position_sigma_checked().unwrap_or(f64::INFINITY)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_filter(sigma: f64) -> PositionFilter {
        PositionFilter::try_new([0.0, 0.0, 0.0], sigma, FilterConfig::default()).unwrap()
    }

    fn identity_measurement_covariance(variance: f64) -> [f64; 9] {
        [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
    }

    fn assert_covariance_symmetric(filter: &PositionFilter) {
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                let a = filter.state.covariance[idx(row, col)];
                let b = filter.state.covariance[idx(col, row)];
                assert!(
                    (a - b).abs() < 1e-8,
                    "P[{row},{col}]={a} vs P[{col},{row}]={b}"
                );
            }
        }
    }

    fn assert_covariance_psd(filter: &PositionFilter) {
        match filter.covariance_psd_status_checked().unwrap() {
            CovariancePsdStatus::Valid { .. } | CovariancePsdStatus::NearSingular { .. } => {}
            other => panic!("expected PSD covariance, got {other:?}"),
        }
    }

    #[test]
    fn checked_constructor_rejects_invalid_state_or_config() {
        assert!(
            PositionFilter::try_new([f64::NAN, 0.0, 0.0], 1.0, FilterConfig::default()).is_err()
        );
        assert!(PositionFilter::try_new([0.0; 3], 0.0, FilterConfig::default()).is_err());
        let mut config = FilterConfig::default();
        config.innovation_gate_sigma = f64::NAN;
        assert!(PositionFilter::try_new([0.0; 3], 1.0, config).is_err());
    }

    #[test]
    fn identity_covariance_is_psd_valid() {
        let mut filter = test_filter(1.0);
        filter.state.covariance = identity_matrix();
        assert!(matches!(
            filter.covariance_psd_status_checked().unwrap(),
            CovariancePsdStatus::Valid { .. }
        ));
        assert!(filter.position_sigma_checked().is_ok());
    }

    #[test]
    fn singular_covariance_is_psd_but_explicitly_near_singular() {
        let mut filter = test_filter(1.0);
        filter.state.covariance = identity_matrix();
        filter.state.covariance[idx(0, 0)] = 0.0;
        assert!(matches!(
            filter.covariance_psd_status_checked().unwrap(),
            CovariancePsdStatus::NearSingular { .. }
        ));
        assert!(filter.position_sigma_checked().is_ok());
    }

    #[test]
    fn positive_diagonal_does_not_hide_indefinite_covariance() {
        let mut filter = test_filter(1.0);
        filter.state.covariance = identity_matrix();
        // This 2×2 principal block has eigenvalues 3 and -1 even though both
        // diagonal variances are positive. The old structural checks accepted it.
        filter.state.covariance[idx(0, 1)] = 2.0;
        filter.state.covariance[idx(1, 0)] = 2.0;

        assert!(matches!(
            filter.covariance_psd_status_checked().unwrap(),
            CovariancePsdStatus::Invalid { .. }
        ));
        assert!(matches!(
            filter.position_sigma_checked(),
            Err(FilterError::IndefiniteCovariance { .. })
        ));
    }

    #[test]
    fn checked_constructor_covariance_has_qualified_psd_status() {
        let filter = test_filter(25.0);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn predict_moves_position() {
        let mut filter = test_filter(100.0);
        filter.state.state[3] = 1.0;
        filter.predict_checked(10.0).unwrap();
        assert!((filter.position()[0] - 10.0).abs() < 0.01);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn predict_propagates_position_velocity_cross_covariance() {
        let mut filter = test_filter(10.0);
        assert_eq!(filter.state.covariance[idx(0, 3)], 0.0);
        filter.predict_checked(2.0).unwrap();
        let expected = 2.0 * 100.0;
        assert!((filter.state.covariance[idx(0, 3)] - expected).abs() < 1e-9);
        assert!((filter.state.covariance[idx(3, 0)] - expected).abs() < 1e-9);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn invalid_predict_is_transactional() {
        let mut filter = test_filter(10.0);
        let before = filter.clone();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(filter.predict_checked(bad).is_err());
            assert_eq!(filter.state.state, before.state.state);
            assert_eq!(filter.state.covariance, before.state.covariance);
            assert_eq!(
                filter.state.last_update_time,
                before.state.last_update_time
            );
        }
    }

    #[test]
    fn predict_increases_uncertainty() {
        let mut filter = test_filter(10.0);
        let sigma_before = filter.position_sigma();
        filter.predict_checked(60.0).unwrap();
        assert!(filter.position_sigma() > sigma_before);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn joseph_range_update_reduces_uncertainty_and_stays_symmetric() {
        let mut filter = test_filter(1000.0);
        let sigma_before = filter.position_sigma();
        let outcome = filter
            .update_range_checked(&[100.0, 0.0, 0.0], 100.0, 1.0)
            .unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::Accepted);
        assert!(filter.position_sigma() < sigma_before);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
        assert!(filter.state.covariance.iter().all(|v| v.is_finite()));
        for i in 0..STATE_DIM {
            assert!(filter.state.covariance[idx(i, i)] >= 0.0);
        }
    }

    #[test]
    fn trust_compatibility_cannot_rewrite_precision() {
        let mut high = test_filter(100.0);
        let mut low = high.clone();
        let high_outcome = high
            .update_with_trust_checked(&[100.0, 0.0, 0.0], 90.0, 2.0, 1.0)
            .unwrap();
        let low_outcome = low
            .update_with_trust_checked(&[100.0, 0.0, 0.0], 90.0, 2.0, 0.01)
            .unwrap();
        assert_eq!(high_outcome, low_outcome);
        assert_eq!(high.state.state, low.state.state);
        assert_eq!(high.state.covariance, low.state.covariance);
    }

    #[test]
    fn zero_trust_is_admission_rejection_not_infinite_uncertainty() {
        let mut filter = test_filter(100.0);
        let before = filter.clone();
        let outcome = filter
            .update_with_trust_checked(&[100.0, 0.0, 0.0], 90.0, 2.0, 0.0)
            .unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::SourceRejected);
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
    }

    #[test]
    fn invalid_measurements_fail_without_mutation() {
        let mut filter = test_filter(25.0);
        let before = filter.clone();
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                filter
                    .update_range_checked(&[100.0, 0.0, 0.0], bad, 1.0)
                    .is_err()
            );
            assert_eq!(filter.state.state, before.state.state);
            assert_eq!(filter.state.covariance, before.state.covariance);
        }
        assert!(
            filter
                .update_range_checked(&[f64::NAN, 0.0, 0.0], 100.0, 1.0)
                .is_err()
        );
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
    }

    #[test]
    fn outlier_measurement_is_observably_gated() {
        let mut filter = test_filter(25.0);
        let before = filter.clone();
        let outcome = filter
            .update_range_checked(&[100.0, 0.0, 0.0], 10_000.0, 1.0)
            .unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::InnovationRejected);
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
    }

    #[test]
    fn convergence_with_multiple_updates() {
        let true_pos = [50.0, 30.0, -20.0];
        let anchors: [[f64; 3]; 4] = [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 100.0],
        ];
        let mut filter = test_filter(200.0);
        for _ in 0..20 {
            filter.predict_checked(1.0).unwrap();
            for anchor in &anchors {
                let dx: f64 = true_pos[0] - anchor[0];
                let dy: f64 = true_pos[1] - anchor[1];
                let dz: f64 = true_pos[2] - anchor[2];
                let range = (dx * dx + dy * dy + dz * dz).sqrt();
                let _ = filter.update_range_checked(anchor, range, 5.0).unwrap();
            }
        }
        let pos = filter.position();
        let error = ((pos[0] - true_pos[0]).powi(2)
            + (pos[1] - true_pos[1]).powi(2)
            + (pos[2] - true_pos[2]).powi(2))
        .sqrt();
        assert!(error < 5.0, "EKF error {error}m");
        assert!(filter.position_sigma() < 50.0);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn checked_barometer_uses_joseph_scalar_update() {
        let mut filter = test_filter(100.0);
        let sigma_z_before = filter.state.covariance[idx(2, 2)];
        let outcome = filter
            .update_barometer_checked(1001.3, 1013.25, 1.0)
            .unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::Accepted);
        assert!(filter.state.covariance[idx(2, 2)] < sigma_z_before);
        assert!(filter.position()[2].abs() > 10.0);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
    }

    #[test]
    fn absolute_position_update_pulls_state_toward_fix() {
        let mut filter = PositionFilter::try_new(
            [50.0, -20.0, 10.0],
            100.0,
            FilterConfig::default(),
        )
        .unwrap();
        filter
            .update_absolute_position_checked(
                [5.0, 2.0, -3.0],
                identity_measurement_covariance(1.0),
            )
            .unwrap();
        let pos = filter.position();
        assert!((pos[0] - 5.0).abs() < 1.0);
        assert!((pos[1] - 2.0).abs() < 1.0);
        assert!((pos[2] + 3.0).abs() < 1.0);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
        assert_eq!(filter.state.update_count, 1);
    }

    #[test]
    fn correlated_absolute_fix_changes_state_and_covariance() {
        let mut diagonal = test_filter(1.0);
        let mut correlated = diagonal.clone();
        let observed = [1.0, -1.0, 0.5];
        let diagonal_r = identity_measurement_covariance(1.0);
        let correlated_r = [1.0, 0.8, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0, 1.0];

        assert_eq!(
            diagonal
                .update_absolute_position_vector_checked(observed, diagonal_r)
                .unwrap(),
            FilterUpdateOutcome::Accepted
        );
        assert_eq!(
            correlated
                .update_absolute_position_vector_checked(observed, correlated_r)
                .unwrap(),
            FilterUpdateOutcome::Accepted
        );

        assert_ne!(diagonal.state.state, correlated.state.state);
        assert_ne!(diagonal.state.covariance, correlated.state.covariance);
        assert_covariance_psd(&diagonal);
        assert_covariance_psd(&correlated);
    }

    #[test]
    fn invalid_full_measurement_covariance_fails_transactionally() {
        let mut filter = test_filter(1.0);
        let before = filter.clone();
        let asymmetric = [1.0, 0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(matches!(
            filter.update_absolute_position_vector_checked([0.5, 0.0, 0.0], asymmetric),
            Err(FilterError::InvalidMeasurementCovariance)
        ));
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
        assert_eq!(filter.state.update_count, before.state.update_count);

        let indefinite = [1.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert!(matches!(
            filter.update_absolute_position_vector_checked([0.5, 0.0, 0.0], indefinite),
            Err(FilterError::InvalidMeasurementCovariance)
        ));
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
        assert_eq!(filter.state.update_count, before.state.update_count);
    }

    #[test]
    fn vector_innovation_gate_is_transactional() {
        let mut filter = test_filter(1.0);
        let before = filter.clone();
        let outcome = filter
            .update_absolute_position_vector_checked(
                [10_000.0, -10_000.0, 10_000.0],
                identity_measurement_covariance(1.0),
            )
            .unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::InnovationRejected);
        assert_eq!(filter.state.state, before.state.state);
        assert_eq!(filter.state.covariance, before.state.covariance);
        assert_eq!(filter.state.update_count, before.state.update_count);
    }

    #[test]
    fn absolute_velocity_update_pulls_state_toward_fix() {
        let mut filter = test_filter(25.0);
        filter
            .update_absolute_velocity_checked(
                [1.5, -0.5, 0.25],
                identity_measurement_covariance(0.04),
            )
            .unwrap();
        let vel = filter.velocity();
        assert!((vel[0] - 1.5).abs() < 0.5);
        assert!((vel[1] + 0.5).abs() < 0.5);
        assert!((vel[2] - 0.25).abs() < 0.5);
        assert_covariance_symmetric(&filter);
        assert_covariance_psd(&filter);
        assert_eq!(filter.state.update_count, 1);
    }

    #[test]
    fn depth_update_reduces_vertical_error() {
        let mut filter =
            PositionFilter::try_new([0.0, 0.0, 40.0], 50.0, FilterConfig::default()).unwrap();
        let outcome = filter.update_depth_checked(12.0, 0.5).unwrap();
        assert_eq!(outcome, FilterUpdateOutcome::Accepted);
        assert!((filter.position()[2] - 12.0).abs() < 5.0);
        assert_covariance_psd(&filter);
    }
}
