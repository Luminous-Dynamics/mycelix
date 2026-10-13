// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Covariance Matrix Handling
//!
//! The 6x6 covariance matrix represents uncertainty in orbital state.
//! This is CRITICAL for all advanced features:
//! - Conjunction probability (miss distance alone is meaningless without uncertainty)
//! - Debris bounties (need to quantify "how well do we know this object?")
//! - Traffic negotiation (need to communicate uncertainty to other operators)
//! - Zero-knowledge proofs (proving properties about uncertain states)
//!
//! # Covariance Matrix Structure
//! ```text
//! ┌                                     ┐
//! │ σxx   σxy   σxz   σxvx  σxvy  σxvz  │
//! │ σxy   σyy   σyz   σyvx  σyvy  σyvz  │
//! │ σxz   σyz   σzz   σzvx  σzvy  σzvz  │
//! │ σxvx  σyvx  σzvx  σvxvx σvxvy σvxvz │
//! │ σxvy  σyvy  σzvy  σvxvy σvyvy σvyvz │
//! │ σxvz  σyvz  σzvz  σvxvz σvyvz σvzvz │
//! └                                     ┘
//! ```
//!
//! Units: Position in km, Velocity in km/s
//! Upper-left 3x3: Position covariance (km²)
//! Lower-right 3x3: Velocity covariance (km²/s²)
//! Off-diagonal 3x3s: Position-velocity cross-covariance (km²/s)

use nalgebra::{Matrix3, Matrix6, SymmetricEigen, Vector3, Vector6};
use serde::{Deserialize, Serialize};

/// 6x6 Covariance matrix for orbital state uncertainty
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CovarianceMatrix {
    /// The symmetric 6x6 covariance matrix
    /// Stored as full matrix but enforced symmetric
    data: Matrix6<f64>,

    /// Reference frame for this covariance
    pub frame: CovarianceFrame,

    /// Whether this is from real observations or estimated
    pub source: CovarianceSource,
}

/// Reference frame for covariance
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum CovarianceFrame {
    /// True Equator Mean Equinox (the coordinate frame emitted by the SGP4 propagator here)
    TEME,
    /// Earth-Centered Inertial (generic frame label; callers must still agree on the exact convention)
    #[default]
    ECI,
    /// Radial-Transverse-Normal (along-track frame)
    RTN,
    /// UVW frame (similar to RTN but different convention)
    UVW,
}

/// Source of covariance data
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum CovarianceSource {
    /// From Space-Track CDM or similar authoritative source
    Official,
    /// Computed from TLE age and propagation
    Estimated { tle_age_hours: f64 },
    /// From sensor observations with known characteristics
    Observed { sensor_accuracy_km: f64 },
    /// Fused from multiple sources
    Fused { source_count: u32 },
    /// Default/unknown (high uncertainty assumed)
    Unknown,
}

impl CovarianceMatrix {
    /// Create a new covariance matrix from a 6x6 array
    /// Enforces symmetry by averaging with transpose
    pub fn new(data: [[f64; 6]; 6]) -> Self {
        let mut mat = Matrix6::from_row_slice(&data.concat());
        // Enforce symmetry
        mat = (mat + mat.transpose()) / 2.0;

        Self {
            data: mat,
            frame: CovarianceFrame::default(),
            source: CovarianceSource::Unknown,
        }
    }

    /// Create from nalgebra Matrix6
    pub fn from_matrix(mat: Matrix6<f64>) -> Self {
        let symmetric = (mat + mat.transpose()) / 2.0;
        Self {
            data: symmetric,
            frame: CovarianceFrame::default(),
            source: CovarianceSource::Unknown,
        }
    }

    /// Create a diagonal covariance (uncorrelated uncertainties)
    pub fn diagonal(sigmas: [f64; 6]) -> Self {
        let variances: Vec<f64> = sigmas.iter().map(|s| s * s).collect();
        Self {
            data: Matrix6::from_diagonal(&Vector6::from_row_slice(&variances)),
            frame: CovarianceFrame::default(),
            source: CovarianceSource::Unknown,
        }
    }

    /// Create a rough, uncalibrated covariance heuristic based on elapsed TLE age.
    ///
    /// This default constructor retains the historical ECI frame label for callers
    /// that do not supply a frame. Prefer `from_tle_age_in_frame` when the associated
    /// state frame is known.
    ///
    /// This empirical LEO heuristic is not a calibrated uncertainty product and must
    /// not, by itself, be treated as authoritative collision-risk evidence.
    pub fn from_tle_age(age_hours: f64) -> Self {
        Self::from_tle_age_in_frame(age_hours, CovarianceFrame::default())
    }

    /// Create an estimated covariance with an explicit coordinate-frame label.
    ///
    /// The values remain the same approximate TLE-age heuristic regardless of frame;
    /// this method does not perform orbit determination, calibration, or covariance
    /// propagation. The caller is responsible for supplying the frame of the associated
    /// state.
    pub fn from_tle_age_in_frame(age_hours: f64, frame: CovarianceFrame) -> Self {
        // Empirical uncertainty growth for LEO (very approximate).
        // Position uncertainty grows roughly linearly with elapsed time;
        // velocity uncertainty grows more slowly. These values are heuristics only.
        let base_pos_km = 1.0; // heuristic ~1 km position uncertainty for a fresh TLE
        let base_vel_kms = 0.001; // heuristic ~1 m/s velocity uncertainty

        // Growth rates (km/day for position, km/s/day for velocity).
        let pos_growth_rate = 5.0 / 24.0;
        let vel_growth_rate = 0.01 / 24.0;

        let pos_sigma = base_pos_km + pos_growth_rate * age_hours;
        let vel_sigma = base_vel_kms + vel_growth_rate * age_hours;

        let mut cov = Self::diagonal([
            pos_sigma, pos_sigma, pos_sigma, vel_sigma, vel_sigma, vel_sigma,
        ]);
        cov.frame = frame;
        cov.source = CovarianceSource::Estimated {
            tle_age_hours: age_hours,
        };
        cov
    }

    /// Get the underlying matrix
    pub fn matrix(&self) -> &Matrix6<f64> {
        &self.data
    }

    /// Get position covariance (upper-left 3x3)
    pub fn position_covariance(&self) -> Matrix3<f64> {
        self.data.fixed_view::<3, 3>(0, 0).into_owned()
    }

    /// Get velocity covariance (lower-right 3x3)
    pub fn velocity_covariance(&self) -> Matrix3<f64> {
        self.data.fixed_view::<3, 3>(3, 3).into_owned()
    }

    /// Get position-velocity cross-covariance (upper-right 3x3)
    pub fn cross_covariance(&self) -> Matrix3<f64> {
        self.data.fixed_view::<3, 3>(0, 3).into_owned()
    }

    /// Get 1-sigma position uncertainty (RSS of diagonal elements)
    pub fn position_sigma(&self) -> f64 {
        let pos_cov = self.position_covariance();
        (pos_cov[(0, 0)] + pos_cov[(1, 1)] + pos_cov[(2, 2)]).sqrt()
    }

    /// Get 1-sigma velocity uncertainty (RSS of diagonal elements)
    pub fn velocity_sigma(&self) -> f64 {
        let vel_cov = self.velocity_covariance();
        (vel_cov[(0, 0)] + vel_cov[(1, 1)] + vel_cov[(2, 2)]).sqrt()
    }

    /// Get position uncertainty as 3D ellipsoid semi-axes
    /// Returns (semi-major, semi-intermediate, semi-minor) in km
    pub fn position_ellipsoid(&self) -> (f64, f64, f64) {
        let pos_cov = self.position_covariance();
        let eigen = SymmetricEigen::new(pos_cov);
        let mut eigenvalues: Vec<f64> = eigen.eigenvalues.iter().map(|&v| v.sqrt()).collect();
        // total_cmp is non-panicking even if callers inspect a matrix before
        // checking is_valid(); invalid inputs still yield non-meaningful axes.
        eigenvalues.sort_by(|a, b| b.total_cmp(a));
        (eigenvalues[0], eigenvalues[1], eigenvalues[2])
    }

    /// Check whether the matrix is finite, symmetric and positive semidefinite.
    ///
    /// The PSD check is performed on a dimensionless correlation-normalized
    /// matrix so position and velocity covariance scales do not mask a defective
    /// low-variance axis. This is a structural check only; it does not authenticate
    /// the source, establish covariance realism, or calibrate orbital uncertainty.
    pub fn is_valid(&self) -> bool {
        if !self.data.iter().all(|value| value.is_finite()) {
            return false;
        }

        // Deserialization can bypass constructors that symmetrize their input.
        // Negative diagonal entries are invalid variances and are never excused
        // by a numerical tolerance.
        let mut standard_deviations = [0.0_f64; 6];
        for (index, sigma) in standard_deviations.iter_mut().enumerate() {
            let variance = self.data[(index, index)];
            if variance < 0.0 {
                return false;
            }
            *sigma = variance.sqrt();
        }

        // Check symmetry relative to each pair's own covariance scale. A single
        // matrix-wide absolute floor can hide asymmetry in the smaller-valued
        // velocity block when position variances dominate the numeric norm.
        for i in 0..6 {
            for j in (i + 1)..6 {
                let upper = self.data[(i, j)];
                let lower = self.data[(j, i)];
                let pair_scale = upper
                    .abs()
                    .max(lower.abs())
                    .max(standard_deviations[i] * standard_deviations[j]);
                let difference = (upper - lower).abs();
                if !difference.is_finite() || difference > 1e-12 * pair_scale {
                    return false;
                }
            }
        }

        // A zero-variance axis in a PSD covariance must have zero covariance
        // with every other axis. Reject any contradictory cross term explicitly.
        for i in 0..6 {
            if standard_deviations[i] == 0.0
                && (0..6).any(|j| self.data[(i, j)] != 0.0 || self.data[(j, i)] != 0.0)
            {
                return false;
            }
        }

        // Normalize by per-axis standard deviations before checking eigenvalues.
        // This produces a dimensionless correlation matrix whose scale is not
        // determined by the arbitrary numeric units of position vs velocity.
        let mut correlation = Matrix6::zeros();
        for i in 0..6 {
            if standard_deviations[i] > 0.0 {
                correlation[(i, i)] = 1.0;
            }
            for j in (i + 1)..6 {
                if standard_deviations[i] == 0.0 || standard_deviations[j] == 0.0 {
                    continue;
                }
                let upper = (self.data[(i, j)] / standard_deviations[i])
                    / standard_deviations[j];
                let lower = (self.data[(j, i)] / standard_deviations[j])
                    / standard_deviations[i];
                if !upper.is_finite() || !lower.is_finite() {
                    return false;
                }
                let normalized = upper * 0.5 + lower * 0.5;
                correlation[(i, j)] = normalized;
                correlation[(j, i)] = normalized;
            }
        }

        let eigen = SymmetricEigen::new(correlation);
        let eigen_scale = eigen
            .eigenvalues
            .iter()
            .map(|value| value.abs())
            .fold(0.0_f64, f64::max);
        if !eigen_scale.is_finite() {
            return false;
        }
        let eigenvalue_tolerance = 64.0 * f64::EPSILON * eigen_scale;
        eigen
            .eigenvalues
            .iter()
            .all(|&value| value.is_finite() && value >= -eigenvalue_tolerance)
    }

    /// Scale covariance (e.g., for k-sigma calculations)
    pub fn scaled(&self, k: f64) -> Self {
        Self {
            data: self.data * (k * k),
            frame: self.frame,
            source: self.source.clone(),
        }
    }

    /// Rotate covariance axes into the local Radial-Transverse-Normal (RTN) basis.
    ///
    /// `position` and `velocity` must be expressed in the same source frame as
    /// this covariance. This applies a block-diagonal orientation rotation to the
    /// position and velocity blocks; it is **not** a full six-state transformation
    /// into a rotating RTN coordinate system because it omits frame angular-rate
    /// terms. Do not treat it as a flight-quality RTN covariance transform.
    pub fn to_rtn(&self, position: Vector3<f64>, velocity: Vector3<f64>) -> Self {
        if self.frame == CovarianceFrame::RTN {
            return self.clone();
        }

        // Compute RTN basis vectors
        let r_hat = position.normalize();
        let h = position.cross(&velocity);
        let n_hat = h.normalize();
        let t_hat = n_hat.cross(&r_hat);

        // Build the basis rotation from the caller-supplied source coordinates to RTN.
        let rot = Matrix3::from_rows(&[r_hat.transpose(), t_hat.transpose(), n_hat.transpose()]);

        // Build 6x6 rotation matrix
        let mut rot6 = Matrix6::zeros();
        rot6.fixed_view_mut::<3, 3>(0, 0).copy_from(&rot);
        rot6.fixed_view_mut::<3, 3>(3, 3).copy_from(&rot);

        // Orientation-only rotation: full rotating-frame state covariance also
        // needs the RTN angular-rate terms described in the method documentation.
        let new_data = rot6 * self.data * rot6.transpose();

        Self {
            data: new_data,
            frame: CovarianceFrame::RTN,
            source: self.source.clone(),
        }
    }

    /// Propagate covariance forward in time using state transition matrix
    /// This is a simplified linear propagation (valid for short time spans)
    pub fn propagate(&self, dt_seconds: f64) -> Self {
        // Simple two-body state transition matrix (Keplerian motion)
        // For more accuracy, use full numerical propagation
        let mut phi = Matrix6::identity();

        // Position depends on velocity (to first order)
        phi[(0, 3)] = dt_seconds;
        phi[(1, 4)] = dt_seconds;
        phi[(2, 5)] = dt_seconds;

        // Transform: C(t+dt) = Phi * C(t) * Phi^T
        let new_data = phi * self.data * phi.transpose();

        Self {
            data: new_data,
            frame: self.frame,
            source: self.source.clone(),
        }
    }

    /// Combine two covariances (e.g., from multiple observations)
    /// Uses inverse-variance weighting (optimal for Gaussian)
    pub fn fuse(&self, other: &CovarianceMatrix) -> Option<Self> {
        // Fusion is only meaningful when both inputs are structurally valid and
        // expressed in the same coordinate frame. Inversion alone is not a
        // validity test: an indefinite matrix can still be invertible.
        if !self.is_valid() || !other.is_valid() || self.frame != other.frame {
            return None;
        }

        // Inverse variance weighting
        let inv_self = self.data.try_inverse()?;
        let inv_other = other.data.try_inverse()?;
        if !inv_self.iter().all(|value| value.is_finite())
            || !inv_other.iter().all(|value| value.is_finite())
        {
            return None;
        }

        let combined_inv = inv_self + inv_other;
        if !combined_inv.iter().all(|value| value.is_finite()) {
            return None;
        }
        let combined = combined_inv.try_inverse()?;
        let fused = Self {
            data: combined,
            frame: self.frame,
            source: CovarianceSource::Fused { source_count: 2 },
        };
        if !fused.is_valid() {
            return None;
        }

        Some(fused)
    }
}

impl Default for CovarianceMatrix {
    fn default() -> Self {
        // Default to high uncertainty (10 km position, 10 m/s velocity)
        Self::diagonal([10.0, 10.0, 10.0, 0.01, 0.01, 0.01])
    }
}

/// Mahalanobis distance between two states given their covariances
/// Used for conjunction screening
pub fn mahalanobis_distance(
    delta: &Vector6<f64>,
    cov1: &CovarianceMatrix,
    cov2: &CovarianceMatrix,
) -> Option<f64> {
    // Fail closed before attempting inversion: callers of this public helper
    // must not get a distance from a malformed covariance or non-finite state.
    // The vector frame cannot be checked from this API, but mixed covariance
    // frames must not be silently combined.
    if !delta.iter().all(|value| value.is_finite())
        || !cov1.is_valid()
        || !cov2.is_valid()
        || cov1.frame != cov2.frame
    {
        return None;
    }

    let combined_cov = cov1.matrix() + cov2.matrix();
    if !combined_cov.iter().all(|value| value.is_finite()) {
        return None;
    }
    let inv = combined_cov.try_inverse()?;
    if !inv.iter().all(|value| value.is_finite()) {
        return None;
    }

    let d_squared = (delta.transpose() * inv * delta)[(0, 0)];
    // Avoid returning Some(NaN), Some(infinity), or a plausible-looking
    // distance derived from a negative quadratic form due to invalid numerics.
    if !d_squared.is_finite() || d_squared < 0.0 {
        return None;
    }
    Some(d_squared.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_diagonal_covariance() {
        let cov = CovarianceMatrix::diagonal([1.0, 2.0, 3.0, 0.1, 0.2, 0.3]);

        // Check diagonal elements are variances (sigma squared)
        let mat = cov.matrix();
        assert!((mat[(0, 0)] - 1.0).abs() < 0.001);
        assert!((mat[(1, 1)] - 4.0).abs() < 0.001); // 2^2
        assert!((mat[(2, 2)] - 9.0).abs() < 0.001); // 3^2
    }

    #[test]
    fn test_position_sigma() {
        let cov = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);
        let sigma = cov.position_sigma();

        // RSS of 1, 1, 1 = sqrt(3) ≈ 1.732
        assert!((sigma - 1.732).abs() < 0.01);
    }

    #[test]
    fn test_covariance_validity() {
        let valid = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);
        assert!(valid.is_valid());
    }

    #[test]
    fn mahalanobis_distance_rejects_mixed_covariance_frames() {
        let delta = Vector6::zeros();
        let eci = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);
        let mut teme = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);
        teme.frame = CovarianceFrame::TEME;

        assert!(mahalanobis_distance(&delta, &eci, &teme).is_none());
    }

    #[test]
    fn mahalanobis_distance_rejects_invalid_covariance() {
        let mut matrix = Matrix6::identity();
        matrix[(5, 5)] = -1e-11;
        let invalid = CovarianceMatrix::from_matrix(matrix);
        let valid = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);

        assert!(mahalanobis_distance(&Vector6::zeros(), &invalid, &valid).is_none());
    }

    #[test]
    fn covariance_fusion_rejects_invalid_inputs() {
        let mut matrix = Matrix6::identity();
        matrix[(5, 5)] = -1e-11;
        let invalid = CovarianceMatrix::from_matrix(matrix);
        let valid = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);

        assert!(invalid.fuse(&valid).is_none());
        assert!(valid.fuse(&invalid).is_none());
    }

    #[test]
    fn mahalanobis_distance_rejects_non_finite_delta() {
        let mut delta = Vector6::zeros();
        delta[0] = f64::NAN;
        let covariance = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);

        assert!(mahalanobis_distance(&delta, &covariance, &covariance).is_none());
    }

    #[test]
    fn mahalanobis_distance_rejects_non_finite_quadratic_form() {
        let mut delta = Vector6::zeros();
        delta[0] = f64::MAX;
        let covariance = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);

        assert!(mahalanobis_distance(&delta, &covariance, &covariance).is_none());
    }

    fn make_serialized_matrix_asymmetric(value: &mut serde_json::Value) -> bool {
        if let Some(array) = value.as_array_mut() {
            if array.len() == 36 && array.iter().all(|entry| entry.is_number()) {
                // For row-major or column-major flat 6x6 storage, these are a
                // transposed pair: (0, 1) and (1, 0).
                array[1] = serde_json::json!(0.25);
                array[6] = serde_json::json!(0.0);
                return true;
            }
            if array.len() == 6
                && array.iter().all(|row| {
                    row.as_array().is_some_and(|entries| {
                        entries.len() == 6 && entries.iter().all(|entry| entry.is_number())
                    })
                })
            {
                array[0][1] = serde_json::json!(0.25);
                array[1][0] = serde_json::json!(0.0);
                return true;
            }
            for child in array.iter_mut() {
                if make_serialized_matrix_asymmetric(child) {
                    return true;
                }
            }
        }
        if let Some(object) = value.as_object_mut() {
            for child in object.values_mut() {
                if make_serialized_matrix_asymmetric(child) {
                    return true;
                }
            }
        }
        false
    }

    #[test]
    fn asymmetric_deserialized_covariance_is_invalid() {
        let covariance = CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]);
        let mut serialized = serde_json::to_value(covariance).unwrap();
        assert!(
            make_serialized_matrix_asymmetric(&mut serialized),
            "test must mutate the serialized nalgebra matrix representation"
        );
        let decoded: CovarianceMatrix = serde_json::from_value(serialized).unwrap();
        assert!(!decoded.is_valid());
    }

    #[test]
    fn non_finite_covariance_is_invalid() {
        let mut matrix = Matrix6::identity();
        matrix[(0, 0)] = f64::NAN;
        let covariance = CovarianceMatrix::from_matrix(matrix);
        assert!(!covariance.is_valid());
    }

    #[test]
    fn negative_diagonal_variance_is_invalid_even_below_legacy_tolerance() {
        let mut matrix = Matrix6::identity();
        matrix[(5, 5)] = -1e-11;
        let covariance = CovarianceMatrix::from_matrix(matrix);
        assert!(!covariance.is_valid());
    }

    #[test]
    fn indefinite_covariance_is_rejected_even_below_legacy_absolute_tolerance() {
        let mut matrix = Matrix6::identity();
        // The diagonal remains positive, but this 2x2 principal block has a
        // negative eigenvalue of roughly -1e-11. The old absolute -1e-10
        // tolerance incorrectly accepted it.
        matrix[(0, 1)] = 1.0 + 1e-11;
        matrix[(1, 0)] = 1.0 + 1e-11;
        let covariance = CovarianceMatrix::from_matrix(matrix);
        assert!(!covariance.is_valid());
    }

    #[test]
    fn test_tle_age_covariance() {
        let fresh = CovarianceMatrix::from_tle_age(0.0);
        let old = CovarianceMatrix::from_tle_age(24.0);

        // Old TLE should have higher uncertainty
        assert!(old.position_sigma() > fresh.position_sigma());
    }
}
