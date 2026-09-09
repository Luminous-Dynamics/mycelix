// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Core positioning algorithm: inverse-variance nonlinear least-squares trilateration.
//!
//! Given N anchors at known positions and N range measurements with uncertainties,
//! compute the most likely position and its covariance matrix.
//!
//! The physical estimator weights measurements only by their declared statistical
//! uncertainty (`1 / sigma²`). Source trust/reputation is an admission-policy concern
//! and must not silently rewrite physical covariance.

use serde::{Deserialize, Serialize};

/// Maximum iterations for Gauss-Newton convergence.
const MAX_ITERATIONS: usize = 20;
/// Convergence threshold in meters.
const CONVERGENCE_M: f64 = 1e-6;

/// A 3D position estimate with uncertainty.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionEstimate {
    /// Estimated position (body-fixed Cartesian, meters).
    pub position: [f64; 3],
    /// 3×3 covariance matrix (row-major, meters²).
    pub covariance: [f64; 9],
    /// 1-sigma position uncertainty (RSS of diagonal), meters.
    pub sigma_m: f64,
    /// Number of anchors used.
    pub anchor_count: usize,
    /// Number of iterations to converge.
    pub iterations: usize,
    /// Post-fit unweighted residual RMS (meters).
    pub residual_rms: f64,
}

/// A 2D position estimate (surface positioning).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositionEstimate2D {
    pub position: [f64; 2],
    pub covariance: [f64; 4],
    pub sigma_m: f64,
    pub anchor_count: usize,
    pub iterations: usize,
    /// Post-fit unweighted residual RMS (meters).
    pub residual_rms: f64,
}

/// Errors during trilateration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TrilaterationError {
    /// Need at least 3 anchors for 2D, 4 for 3D.
    InsufficientAnchors { have: usize, need: usize },
    /// Input arrays have mismatched lengths.
    LengthMismatch,
    /// Anchors are nearly collinear/coplanar or the normal matrix is singular.
    DegenerateGeometry,
    /// Failed to converge within MAX_ITERATIONS.
    DidNotConverge { iterations: usize, residual: f64 },
    /// Range measurement is non-finite, negative, or zero.
    InvalidRange { index: usize, value: f64 },
    /// Measurement sigma is non-finite, negative, or zero.
    InvalidSigma { index: usize, value: f64 },
    /// Anchor coordinate is non-finite.
    InvalidAnchor {
        anchor_index: usize,
        component: usize,
        value: f64,
    },
    /// Compatibility trust input is not finite or outside [0, 1].
    InvalidTrustWeight { index: usize, value: f64 },
    /// Intermediate arithmetic became non-finite.
    NonFiniteComputation,
    /// Final covariance failed basic finite/positive-diagonal qualification.
    InvalidCovariance,
}

impl std::fmt::Display for TrilaterationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InsufficientAnchors { have, need } => {
                write!(f, "Need {need} anchors, have {have}")
            }
            Self::LengthMismatch => write!(f, "Input arrays have different lengths"),
            Self::DegenerateGeometry => write!(f, "Anchor geometry/normal matrix is degenerate"),
            Self::DidNotConverge {
                iterations,
                residual,
            } => write!(
                f,
                "Did not converge after {iterations} iterations (residual: {residual:.3}m)"
            ),
            Self::InvalidRange { index, value } => {
                write!(f, "Invalid range at index {index}: {value}")
            }
            Self::InvalidSigma { index, value } => {
                write!(f, "Invalid sigma at index {index}: {value}")
            }
            Self::InvalidAnchor {
                anchor_index,
                component,
                value,
            } => write!(
                f,
                "Invalid anchor coordinate at anchor {anchor_index}, component {component}: {value}"
            ),
            Self::InvalidTrustWeight { index, value } => {
                write!(f, "Invalid trust weight at index {index}: {value}")
            }
            Self::NonFiniteComputation => write!(f, "Trilateration produced non-finite arithmetic"),
            Self::InvalidCovariance => write!(f, "Trilateration covariance is not numerically valid"),
        }
    }
}

impl std::error::Error for TrilaterationError {}

fn validate_trust_compatibility(
    trust_weights: &[f64],
    expected_len: usize,
) -> Result<(), TrilaterationError> {
    if trust_weights.len() != expected_len {
        return Err(TrilaterationError::LengthMismatch);
    }
    for (index, value) in trust_weights.iter().copied().enumerate() {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(TrilaterationError::InvalidTrustWeight { index, value });
        }
    }
    Ok(())
}

fn inverse_variance_weights(sigmas: &[f64]) -> Result<Vec<f64>, TrilaterationError> {
    let mut weights = Vec::with_capacity(sigmas.len());
    for (index, sigma) in sigmas.iter().copied().enumerate() {
        if !sigma.is_finite() || sigma <= 0.0 {
            return Err(TrilaterationError::InvalidSigma { index, value: sigma });
        }
        let variance = sigma * sigma;
        let weight = 1.0 / variance;
        if !variance.is_finite() || !weight.is_finite() || weight <= 0.0 {
            return Err(TrilaterationError::InvalidSigma { index, value: sigma });
        }
        weights.push(weight);
    }
    Ok(weights)
}

fn validate_3d_inputs(
    anchors: &[[f64; 3]],
    ranges: &[f64],
    sigmas: &[f64],
) -> Result<Vec<f64>, TrilaterationError> {
    let n = anchors.len();
    if n < 4 {
        return Err(TrilaterationError::InsufficientAnchors { have: n, need: 4 });
    }
    if ranges.len() != n || sigmas.len() != n {
        return Err(TrilaterationError::LengthMismatch);
    }
    for (anchor_index, anchor) in anchors.iter().enumerate() {
        for (component, value) in anchor.iter().copied().enumerate() {
            if !value.is_finite() {
                return Err(TrilaterationError::InvalidAnchor {
                    anchor_index,
                    component,
                    value,
                });
            }
        }
    }
    for (index, value) in ranges.iter().copied().enumerate() {
        if !value.is_finite() || value <= 0.0 {
            return Err(TrilaterationError::InvalidRange { index, value });
        }
    }
    inverse_variance_weights(sigmas)
}

fn validate_2d_inputs(
    anchors: &[[f64; 2]],
    ranges: &[f64],
    sigmas: &[f64],
) -> Result<Vec<f64>, TrilaterationError> {
    let n = anchors.len();
    if n < 3 {
        return Err(TrilaterationError::InsufficientAnchors { have: n, need: 3 });
    }
    if ranges.len() != n || sigmas.len() != n {
        return Err(TrilaterationError::LengthMismatch);
    }
    for (anchor_index, anchor) in anchors.iter().enumerate() {
        for (component, value) in anchor.iter().copied().enumerate() {
            if !value.is_finite() {
                return Err(TrilaterationError::InvalidAnchor {
                    anchor_index,
                    component,
                    value,
                });
            }
        }
    }
    for (index, value) in ranges.iter().copied().enumerate() {
        if !value.is_finite() || value <= 0.0 {
            return Err(TrilaterationError::InvalidRange { index, value });
        }
    }
    inverse_variance_weights(sigmas)
}

fn validate_covariance_3d(covariance: &[[f64; 3]; 3]) -> Result<(), TrilaterationError> {
    if covariance.iter().flatten().any(|v| !v.is_finite())
        || covariance[0][0] <= 0.0
        || covariance[1][1] <= 0.0
        || covariance[2][2] <= 0.0
    {
        return Err(TrilaterationError::InvalidCovariance);
    }
    Ok(())
}

fn validate_covariance_2d(covariance: &[f64; 4]) -> Result<(), TrilaterationError> {
    if covariance.iter().any(|v| !v.is_finite())
        || covariance[0] <= 0.0
        || covariance[3] <= 0.0
    {
        return Err(TrilaterationError::InvalidCovariance);
    }
    Ok(())
}

/// Canonical 3D physical trilateration profile.
///
/// Measurement weighting is strictly `1 / sigma²`; source trust/reputation is
/// intentionally absent from this estimator.
pub fn trilaterate_3d_physical(
    anchors: &[[f64; 3]],
    ranges: &[f64],
    sigmas: &[f64],
) -> Result<PositionEstimate, TrilaterationError> {
    let n = anchors.len();
    let weights = validate_3d_inputs(anchors, ranges, sigmas)?;

    let total_w: f64 = weights.iter().sum();
    if !total_w.is_finite() || total_w <= 0.0 {
        return Err(TrilaterationError::NonFiniteComputation);
    }

    let mut pos = [0.0_f64; 3];
    for i in 0..n {
        for (component, coordinate) in anchors[i].iter().copied().enumerate() {
            pos[component] += weights[i] * coordinate;
        }
    }
    for value in &mut pos {
        *value /= total_w;
        if !value.is_finite() {
            return Err(TrilaterationError::NonFiniteComputation);
        }
    }

    let mut iterations = 0;
    let mut residual_rms = f64::MAX;

    for iter in 0..MAX_ITERATIONS {
        iterations = iter + 1;
        let mut jtw_j = [[0.0_f64; 3]; 3];
        let mut jtw_r = [0.0_f64; 3];
        let mut sum_sq_residual = 0.0;

        for i in 0..n {
            let dx = pos[0] - anchors[i][0];
            let dy = pos[1] - anchors[i][1];
            let dz = pos[2] - anchors[i][2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if !dist.is_finite() {
                return Err(TrilaterationError::NonFiniteComputation);
            }
            if dist < 1e-10 {
                continue;
            }

            let residual = dist - ranges[i];
            if !residual.is_finite() {
                return Err(TrilaterationError::NonFiniteComputation);
            }
            // Keep diagnostics in the documented physical unit: meters.
            sum_sq_residual += residual * residual;

            let j_row = [dx / dist, dy / dist, dz / dist];
            for a in 0..3 {
                jtw_r[a] += weights[i] * j_row[a] * residual;
                for b in 0..3 {
                    jtw_j[a][b] += weights[i] * j_row[a] * j_row[b];
                }
            }
        }

        residual_rms = (sum_sq_residual / n as f64).sqrt();
        if !residual_rms.is_finite()
            || jtw_r.iter().any(|v| !v.is_finite())
            || jtw_j.iter().flatten().any(|v| !v.is_finite())
        {
            return Err(TrilaterationError::NonFiniteComputation);
        }

        let delta = solve_3x3(&jtw_j, &jtw_r).ok_or(TrilaterationError::DegenerateGeometry)?;
        if delta.iter().any(|v| !v.is_finite()) {
            return Err(TrilaterationError::NonFiniteComputation);
        }

        for i in 0..3 {
            pos[i] -= delta[i];
        }
        let step = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
        if !step.is_finite() || pos.iter().any(|v| !v.is_finite()) {
            return Err(TrilaterationError::NonFiniteComputation);
        }
        if step < CONVERGENCE_M {
            break;
        }
    }

    if iterations == MAX_ITERATIONS && residual_rms > 1.0 {
        return Err(TrilaterationError::DidNotConverge {
            iterations,
            residual: residual_rms,
        });
    }

    let mut jtw_j_final = [[0.0_f64; 3]; 3];
    for i in 0..n {
        let dx = pos[0] - anchors[i][0];
        let dy = pos[1] - anchors[i][1];
        let dz = pos[2] - anchors[i][2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        if !dist.is_finite() {
            return Err(TrilaterationError::NonFiniteComputation);
        }
        if dist < 1e-10 {
            continue;
        }
        let j_row = [dx / dist, dy / dist, dz / dist];
        for a in 0..3 {
            for b in 0..3 {
                jtw_j_final[a][b] += weights[i] * j_row[a] * j_row[b];
            }
        }
    }

    if jtw_j_final.iter().flatten().any(|v| !v.is_finite()) {
        return Err(TrilaterationError::NonFiniteComputation);
    }
    let covariance = invert_3x3(&jtw_j_final).ok_or(TrilaterationError::DegenerateGeometry)?;
    validate_covariance_3d(&covariance)?;
    let sigma = (covariance[0][0] + covariance[1][1] + covariance[2][2]).sqrt();
    if !sigma.is_finite() || sigma <= 0.0 {
        return Err(TrilaterationError::InvalidCovariance);
    }

    Ok(PositionEstimate {
        position: pos,
        covariance: [
            covariance[0][0],
            covariance[0][1],
            covariance[0][2],
            covariance[1][0],
            covariance[1][1],
            covariance[1][2],
            covariance[2][0],
            covariance[2][1],
            covariance[2][2],
        ],
        sigma_m: sigma,
        anchor_count: n,
        iterations,
        residual_rms,
    })
}

/// Compatibility entry point retaining the historical trust-weight argument.
///
/// Trust values are structurally validated but **do not alter measurement
/// precision or covariance**. Move source admission/reputation policy before
/// this estimator. A future v2 API should remove this parameter entirely.
pub fn trilaterate_3d(
    anchors: &[[f64; 3]],
    ranges: &[f64],
    sigmas: &[f64],
    trust_weights: &[f64],
) -> Result<PositionEstimate, TrilaterationError> {
    validate_trust_compatibility(trust_weights, anchors.len())?;
    trilaterate_3d_physical(anchors, ranges, sigmas)
}

/// Canonical 2D physical trilateration profile.
pub fn trilaterate_2d_physical(
    anchors: &[[f64; 2]],
    ranges: &[f64],
    sigmas: &[f64],
) -> Result<PositionEstimate2D, TrilaterationError> {
    let n = anchors.len();
    let weights = validate_2d_inputs(anchors, ranges, sigmas)?;

    let total_w: f64 = weights.iter().sum();
    if !total_w.is_finite() || total_w <= 0.0 {
        return Err(TrilaterationError::NonFiniteComputation);
    }
    let mut pos = [0.0_f64; 2];
    for i in 0..n {
        pos[0] += weights[i] * anchors[i][0];
        pos[1] += weights[i] * anchors[i][1];
    }
    pos[0] /= total_w;
    pos[1] /= total_w;
    if pos.iter().any(|v| !v.is_finite()) {
        return Err(TrilaterationError::NonFiniteComputation);
    }

    let mut iterations = 0;
    let mut residual_rms = f64::MAX;

    for iter in 0..MAX_ITERATIONS {
        iterations = iter + 1;
        let mut jtw_j = [[0.0_f64; 2]; 2];
        let mut jtw_r = [0.0_f64; 2];
        let mut sum_sq = 0.0;

        for i in 0..n {
            let dx = pos[0] - anchors[i][0];
            let dy = pos[1] - anchors[i][1];
            let dist = (dx * dx + dy * dy).sqrt();
            if !dist.is_finite() {
                return Err(TrilaterationError::NonFiniteComputation);
            }
            if dist < 1e-10 {
                continue;
            }

            let residual = dist - ranges[i];
            if !residual.is_finite() {
                return Err(TrilaterationError::NonFiniteComputation);
            }
            sum_sq += residual * residual;
            let j_row = [dx / dist, dy / dist];

            for a in 0..2 {
                jtw_r[a] += weights[i] * j_row[a] * residual;
                for b in 0..2 {
                    jtw_j[a][b] += weights[i] * j_row[a] * j_row[b];
                }
            }
        }

        residual_rms = (sum_sq / n as f64).sqrt();
        let det = jtw_j[0][0] * jtw_j[1][1] - jtw_j[0][1] * jtw_j[1][0];
        if !residual_rms.is_finite() || !det.is_finite() {
            return Err(TrilaterationError::NonFiniteComputation);
        }
        if det.abs() < 1e-20 {
            return Err(TrilaterationError::DegenerateGeometry);
        }
        let delta = [
            (jtw_j[1][1] * jtw_r[0] - jtw_j[0][1] * jtw_r[1]) / det,
            (jtw_j[0][0] * jtw_r[1] - jtw_j[1][0] * jtw_r[0]) / det,
        ];
        if delta.iter().any(|v| !v.is_finite()) {
            return Err(TrilaterationError::NonFiniteComputation);
        }

        pos[0] -= delta[0];
        pos[1] -= delta[1];
        let step = (delta[0] * delta[0] + delta[1] * delta[1]).sqrt();
        if !step.is_finite() || pos.iter().any(|v| !v.is_finite()) {
            return Err(TrilaterationError::NonFiniteComputation);
        }
        if step < CONVERGENCE_M {
            break;
        }
    }

    if iterations == MAX_ITERATIONS && residual_rms > 1.0 {
        return Err(TrilaterationError::DidNotConverge {
            iterations,
            residual: residual_rms,
        });
    }

    let mut jtw_j_final = [[0.0_f64; 2]; 2];
    for i in 0..n {
        let dx = pos[0] - anchors[i][0];
        let dy = pos[1] - anchors[i][1];
        let dist = (dx * dx + dy * dy).sqrt();
        if !dist.is_finite() {
            return Err(TrilaterationError::NonFiniteComputation);
        }
        if dist < 1e-10 {
            continue;
        }
        let j_row = [dx / dist, dy / dist];
        for a in 0..2 {
            for b in 0..2 {
                jtw_j_final[a][b] += weights[i] * j_row[a] * j_row[b];
            }
        }
    }

    let det = jtw_j_final[0][0] * jtw_j_final[1][1]
        - jtw_j_final[0][1] * jtw_j_final[1][0];
    if !det.is_finite() {
        return Err(TrilaterationError::NonFiniteComputation);
    }
    if det.abs() <= 1e-20 {
        return Err(TrilaterationError::DegenerateGeometry);
    }
    let covariance = [
        jtw_j_final[1][1] / det,
        -jtw_j_final[0][1] / det,
        -jtw_j_final[1][0] / det,
        jtw_j_final[0][0] / det,
    ];
    validate_covariance_2d(&covariance)?;
    let sigma = (covariance[0] + covariance[3]).sqrt();
    if !sigma.is_finite() || sigma <= 0.0 {
        return Err(TrilaterationError::InvalidCovariance);
    }

    Ok(PositionEstimate2D {
        position: pos,
        covariance,
        sigma_m: sigma,
        anchor_count: n,
        iterations,
        residual_rms,
    })
}

/// Compatibility 2D entry point. Trust is validated but cannot rewrite sigma.
pub fn trilaterate_2d(
    anchors: &[[f64; 2]],
    ranges: &[f64],
    sigmas: &[f64],
    trust_weights: &[f64],
) -> Result<PositionEstimate2D, TrilaterationError> {
    validate_trust_compatibility(trust_weights, anchors.len())?;
    trilaterate_2d_physical(anchors, ranges, sigmas)
}

// ============================================================================
// LINEAR ALGEBRA HELPERS (3×3)
// ============================================================================

fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let inv = invert_3x3(a)?;
    Some([
        inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
        inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
        inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2],
    ])
}

fn invert_3x3(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    if m.iter().flatten().any(|v| !v.is_finite()) {
        return None;
    }
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if !det.is_finite() || det.abs() < 1e-30 {
        return None;
    }
    let inv_det = 1.0 / det;
    if !inv_det.is_finite() {
        return None;
    }
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det,
        ],
    ])
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn basic_3d_geometry() -> ([[f64; 3]; 4], [f64; 4], [f64; 4]) {
        (
            [
                [100.0, 0.0, 0.0],
                [0.0, 100.0, 0.0],
                [0.0, 0.0, 100.0],
                [-100.0, 0.0, 0.0],
            ],
            [100.0, 100.0, 100.0, 100.0],
            [1.0, 1.0, 1.0, 1.0],
        )
    }

    #[test]
    fn trilaterate_3d_basic() {
        let (anchors, ranges, sigmas) = basic_3d_geometry();
        let est = trilaterate_3d_physical(&anchors, &ranges, &sigmas).unwrap();
        assert!(est.position[0].abs() < 1.0, "x={}", est.position[0]);
        assert!(est.position[1].abs() < 1.0, "y={}", est.position[1]);
        assert!(est.position[2].abs() < 1.0, "z={}", est.position[2]);
        assert!(est.iterations <= 10);
    }

    #[test]
    fn trilaterate_3d_off_center() {
        let target: [f64; 3] = [50.0, 30.0, -20.0];
        let anchors: [[f64; 3]; 5] = [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 100.0],
            [100.0, 100.0, 0.0],
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| {
                let dx: f64 = a[0] - target[0];
                let dy: f64 = a[1] - target[1];
                let dz: f64 = a[2] - target[2];
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .collect();
        let sigmas = vec![1.0; 5];

        let est = trilaterate_3d_physical(&anchors, &ranges, &sigmas).unwrap();
        assert!((est.position[0] - 50.0).abs() < 0.1);
        assert!((est.position[1] - 30.0).abs() < 0.1);
        assert!((est.position[2] + 20.0).abs() < 0.1);
    }

    #[test]
    fn trust_compatibility_input_cannot_change_physical_estimate() {
        let anchors = [
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 100.0],
            [-100.0, 0.0, 0.0],
            [50.0, 50.0, 50.0],
        ];
        let ranges = vec![100.0, 100.0, 100.0, 100.0, 200.0];
        let sigmas = vec![1.0; 5];
        let trusts_low = vec![1.0, 1.0, 1.0, 1.0, 0.01];
        let trusts_high = vec![1.0; 5];

        let low = trilaterate_3d(&anchors, &ranges, &sigmas, &trusts_low).unwrap();
        let high = trilaterate_3d(&anchors, &ranges, &sigmas, &trusts_high).unwrap();
        for i in 0..3 {
            assert!((low.position[i] - high.position[i]).abs() < 1e-12);
        }
        for i in 0..9 {
            assert!((low.covariance[i] - high.covariance[i]).abs() < 1e-12);
        }
        assert!((low.sigma_m - high.sigma_m).abs() < 1e-12);
    }

    #[test]
    fn trilaterate_2d_basic() {
        let anchors: [[f64; 2]; 3] = [[0.0, 0.0], [100.0, 0.0], [50.0, 86.6]];
        let target: [f64; 2] = [40.0, 30.0];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| {
                let dx: f64 = a[0] - target[0];
                let dy: f64 = a[1] - target[1];
                (dx.powi(2) + dy.powi(2)).sqrt()
            })
            .collect();
        let est = trilaterate_2d_physical(&anchors, &ranges, &[1.0; 3]).unwrap();
        assert!((est.position[0] - 40.0).abs() < 0.1);
        assert!((est.position[1] - 30.0).abs() < 0.1);
    }

    #[test]
    fn insufficient_anchors_3d() {
        let result = trilaterate_3d_physical(
            &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            &[1.0, 1.0, 1.0],
            &[1.0, 1.0, 1.0],
        );
        assert!(matches!(
            result,
            Err(TrilaterationError::InsufficientAnchors { .. })
        ));
    }

    #[test]
    fn insufficient_anchors_2d() {
        let result = trilaterate_2d_physical(
            &[[0.0, 0.0], [1.0, 0.0]],
            &[1.0, 1.0],
            &[1.0, 1.0],
        );
        assert!(matches!(
            result,
            Err(TrilaterationError::InsufficientAnchors { .. })
        ));
    }

    #[test]
    fn invalid_range_rejected_including_non_finite() {
        let (anchors, _, sigmas) = basic_3d_geometry();
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let result = trilaterate_3d_physical(&anchors, &[1.0, bad, 1.0, 1.0], &sigmas);
            assert!(matches!(result, Err(TrilaterationError::InvalidRange { index: 1, .. })));
        }
    }

    #[test]
    fn invalid_sigma_rejected_including_non_finite() {
        let (anchors, ranges, _) = basic_3d_geometry();
        for bad in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let result = trilaterate_3d_physical(&anchors, &ranges, &[1.0, bad, 1.0, 1.0]);
            assert!(matches!(result, Err(TrilaterationError::InvalidSigma { index: 1, .. })));
        }
    }

    #[test]
    fn non_finite_anchor_rejected() {
        let (_, ranges, sigmas) = basic_3d_geometry();
        let anchors = [
            [100.0, 0.0, 0.0],
            [0.0, f64::NAN, 0.0],
            [0.0, 0.0, 100.0],
            [-100.0, 0.0, 0.0],
        ];
        let result = trilaterate_3d_physical(&anchors, &ranges, &sigmas);
        assert!(matches!(
            result,
            Err(TrilaterationError::InvalidAnchor {
                anchor_index: 1,
                component: 1,
                ..
            })
        ));
    }

    #[test]
    fn invalid_compatibility_trust_rejected_without_touching_covariance_math() {
        let (anchors, ranges, sigmas) = basic_3d_geometry();
        let result = trilaterate_3d(&anchors, &ranges, &sigmas, &[1.0, f64::NAN, 1.0, 1.0]);
        assert!(matches!(
            result,
            Err(TrilaterationError::InvalidTrustWeight { index: 1, .. })
        ));
    }

    #[test]
    fn degenerate_final_covariance_fails_closed() {
        let anchors = [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [200.0, 0.0, 0.0],
            [300.0, 0.0, 0.0],
        ];
        let ranges = [50.0, 50.0, 150.0, 250.0];
        let result = trilaterate_3d_physical(&anchors, &ranges, &[1.0; 4]);
        assert!(matches!(result, Err(TrilaterationError::DegenerateGeometry)));
    }

    #[test]
    fn covariance_is_finite_and_positive_on_diagonal() {
        let (anchors, ranges, sigmas) = basic_3d_geometry();
        let est = trilaterate_3d_physical(&anchors, &ranges, &sigmas).unwrap();
        assert!(est.covariance.iter().all(|v| v.is_finite()));
        assert!(est.covariance[0] > 0.0);
        assert!(est.covariance[4] > 0.0);
        assert!(est.covariance[8] > 0.0);
        assert!(est.sigma_m.is_finite() && est.sigma_m > 0.0);
    }

    #[test]
    fn scale_preserves_physical_units() {
        let target = [25.0_f64, 30.0, 40.0];
        let anchors = [
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 100.0],
            [100.0, 100.0, 100.0],
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| {
                let dx = a[0] - target[0];
                let dy = a[1] - target[1];
                let dz = a[2] - target[2];
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .collect();
        let sigmas = vec![2.0; anchors.len()];
        let base = trilaterate_3d_physical(&anchors, &ranges, &sigmas).unwrap();

        let k = 10.0;
        let scaled_anchors: Vec<[f64; 3]> = anchors
            .iter()
            .map(|a| [a[0] * k, a[1] * k, a[2] * k])
            .collect();
        let scaled_ranges: Vec<f64> = ranges.iter().map(|r| r * k).collect();
        let scaled_sigmas: Vec<f64> = sigmas.iter().map(|s| s * k).collect();
        let scaled =
            trilaterate_3d_physical(&scaled_anchors, &scaled_ranges, &scaled_sigmas).unwrap();

        for i in 0..3 {
            assert!((scaled.position[i] - base.position[i] * k).abs() < 1e-6);
        }
        assert!((scaled.sigma_m - base.sigma_m * k).abs() < 1e-6);
        assert!((scaled.residual_rms - base.residual_rms * k).abs() < 1e-6);
    }

    #[test]
    fn invert_3x3_identity() {
        let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let inv = invert_3x3(&id).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv[i][j] - expected).abs() < 1e-10);
            }
        }
    }
}
