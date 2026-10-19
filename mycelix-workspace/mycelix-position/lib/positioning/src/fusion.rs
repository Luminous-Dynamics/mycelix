// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Peer fusion via Covariance Intersection for decentralized positioning.

use serde::{Deserialize, Serialize};

/// Helper for accepting both scalar and array sigma.
pub struct SigmaInput(pub [f64; 3]);
impl From<[f64; 3]> for SigmaInput {
    fn from(v: [f64; 3]) -> Self {
        Self(v)
    }
}
impl From<f64> for SigmaInput {
    fn from(v: f64) -> Self {
        Self([v, v, v])
    }
}
impl From<f32> for SigmaInput {
    fn from(v: f32) -> Self {
        Self([v as f64, v as f64, v as f64])
    }
}

/// 3D Gaussian estimate with mean and covariance diagonal.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GaussianEstimate3D {
    pub mean: [f64; 3],
    pub covariance_diag: [f64; 3],
    /// Alias for covariance_diag (backward compat).
    #[serde(default)]
    pub covariance: [f64; 3],
}

impl GaussianEstimate3D {
    pub fn new(mean: [f64; 3], cov: [f64; 3]) -> Self {
        Self {
            mean,
            covariance_diag: cov,
            covariance: cov,
        }
    }
    pub fn from_diagonal_sigma(mean: [f64; 3], sigma: impl Into<SigmaInput>) -> Self {
        let s: SigmaInput = sigma.into();
        let c = [s.0[0] * s.0[0], s.0[1] * s.0[1], s.0[2] * s.0[2]];
        Self {
            mean,
            covariance_diag: c,
            covariance: c,
        }
    }
    pub fn uncertainty(&self) -> f64 {
        (self.covariance_diag.iter().sum::<f64>() / 3.0).sqrt()
    }
    pub fn diagonal_sigma_m(&self) -> [f64; 3] {
        [
            self.covariance_diag[0].sqrt(),
            self.covariance_diag[1].sqrt(),
            self.covariance_diag[2].sqrt(),
        ]
    }
}

impl PartialEq for GaussianEstimate3D {
    fn eq(&self, other: &Self) -> bool {
        self.mean == other.mean && self.covariance_diag == other.covariance_diag
    }
}

impl PublishableEstimate3D for GaussianEstimate3D {
    fn estimate(&self) -> &GaussianEstimate3D {
        self
    }
    fn source_count(&self) -> usize {
        1
    }
    fn timestamp_us(&self) -> u64 {
        0
    }
}

/// A peer's position estimate with trust weight.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PeerEstimate3D {
    pub peer_id: String,
    pub estimate: GaussianEstimate3D,
    pub trust_weight: f64,
    pub timestamp_us: u64,
    pub confidence: f64,
}

impl PeerEstimate3D {
    pub fn peer_estimate(&self) -> &GaussianEstimate3D {
        &self.estimate
    }
}

/// Trait for estimates that can be published to DHT.
pub trait PublishableEstimate3D {
    fn estimate(&self) -> &GaussianEstimate3D;
    fn peer_estimate(&self) -> &GaussianEstimate3D {
        self.estimate()
    }
    fn source_count(&self) -> usize;
    fn timestamp_us(&self) -> u64;
    fn confidence(&self) -> f64 {
        1.0
    }
}

/// Default publishable estimate implementation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DefaultPublishableEstimate3D {
    pub estimate: GaussianEstimate3D,
    pub source_count: usize,
    pub timestamp_us: u64,
}

impl PublishableEstimate3D for DefaultPublishableEstimate3D {
    fn estimate(&self) -> &GaussianEstimate3D {
        &self.estimate
    }
    fn source_count(&self) -> usize {
        self.source_count
    }
    fn timestamp_us(&self) -> u64 {
        self.timestamp_us
    }
}

/// Legacy trust-weighted peer aggregation. This is not Covariance Intersection.
pub struct PeerFusion3D {
    peers: Vec<PeerEstimate3D>,
    max_peers: usize,
}

impl PeerFusion3D {
    pub fn new(max_peers: usize) -> Self {
        Self {
            peers: Vec::new(),
            max_peers,
        }
    }

    pub fn add_peer(&mut self, peer: PeerEstimate3D) {
        if self.peers.len() >= self.max_peers {
            self.peers.remove(0);
        }
        self.peers.push(peer);
    }

    /// Aggregate peer estimates with the legacy trust-weighted mean; this is not CI.
    pub fn fuse(&self) -> Option<GaussianEstimate3D> {
        if self.peers.is_empty() {
            return None;
        }
        let total_w: f64 = self.peers.iter().map(|p| p.trust_weight).sum();
        if total_w < 1e-10 {
            return None;
        }
        let mut mean = [0.0; 3];
        let mut cov = [0.0; 3];
        for p in &self.peers {
            let w = p.trust_weight / total_w;
            for i in 0..3 {
                mean[i] += w * p.estimate.mean[i];
                cov[i] += w * p.estimate.covariance_diag[i];
            }
        }
        Some(GaussianEstimate3D {
            mean,
            covariance_diag: cov,
            covariance: cov,
        })
    }

    pub fn peer_count(&self) -> usize {
        self.peers.len()
    }

    pub fn upsert_peer(&mut self, peer: PeerEstimate3D) {
        if let Some(existing) = self.peers.iter_mut().find(|p| p.peer_id == peer.peer_id) {
            *existing = peer;
        } else {
            self.add_peer(peer);
        }
    }

    pub fn fused_estimate(&self) -> Option<GaussianEstimate3D> {
        self.fuse()
    }
}

/// Identifies which input failed validation during covariance fusion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionInput {
    A,
    B,
}

/// Fail-closed errors from diagonal Covariance Intersection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionError {
    NonFiniteOmega,
    OmegaOutOfRange,
    NonFiniteMean { input: FusionInput, axis: usize },
    InvalidCovariance { input: FusionInput, axis: usize },
    NonFinitePrecision { axis: usize },
    InvalidFusedCovariance { axis: usize },
    NonFiniteFusedMean { axis: usize },
}

impl std::fmt::Display for FusionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteOmega => write!(formatter, "CI omega must be finite"),
            Self::OmegaOutOfRange => write!(formatter, "CI omega must be within [0, 1]"),
            Self::NonFiniteMean { input, axis } => write!(formatter, "estimate {input:?} has a non-finite mean on axis {axis}"),
            Self::InvalidCovariance { input, axis } => write!(formatter, "estimate {input:?} covariance must be finite and strictly positive on axis {axis}"),
            Self::NonFinitePrecision { axis } => write!(formatter, "CI precision arithmetic is invalid on axis {axis}"),
            Self::InvalidFusedCovariance { axis } => write!(formatter, "fused covariance is invalid on axis {axis}"),
            Self::NonFiniteFusedMean { axis } => write!(formatter, "fused mean is invalid on axis {axis}"),
        }
    }
}

impl std::error::Error for FusionError {}

fn validate_ci_input(estimate: &GaussianEstimate3D, input: FusionInput) -> Result<(), FusionError> {
    for axis in 0..3 {
        if !estimate.mean[axis].is_finite() {
            return Err(FusionError::NonFiniteMean { input, axis });
        }
        let variance = estimate.covariance_diag[axis];
        if !variance.is_finite() || variance <= 0.0 {
            return Err(FusionError::InvalidCovariance { input, axis });
        }
    }
    Ok(())
}

fn canonical_estimate(estimate: &GaussianEstimate3D) -> GaussianEstimate3D {
    // covariance_diag is authoritative in this legacy type; normalize the alias.
    GaussianEstimate3D::new(estimate.mean, estimate.covariance_diag)
}

/// Diagonal Covariance Intersection for two 3D Gaussian estimates.
///
/// The statistical CI weight is independent of trust/reputation. omega must be
/// finite and in [0, 1]; means and diagonal variances must be finite; variances
/// must be strictly positive. Invalid input or overflow is returned as a typed
/// error, never clamped or replaced with arbitrary precision constants.
///
/// covariance_diag is authoritative in the legacy estimate type; output aliases
/// are normalized to match it. This is diagonal CI, not full-matrix or multi-source CI.
pub fn covariance_intersection_3d(
    a: &GaussianEstimate3D,
    b: &GaussianEstimate3D,
    omega: f64,
) -> Result<GaussianEstimate3D, FusionError> {
    if !omega.is_finite() {
        return Err(FusionError::NonFiniteOmega);
    }
    if !(0.0..=1.0).contains(&omega) {
        return Err(FusionError::OmegaOutOfRange);
    }
    validate_ci_input(a, FusionInput::A)?;
    validate_ci_input(b, FusionInput::B)?;

    // At endpoints, avoid unnecessary reciprocals for the estimate assigned zero
    // weight. Both input vectors are still validated above.
    if omega == 1.0 {
        return Ok(canonical_estimate(a));
    }
    if omega == 0.0 {
        return Ok(canonical_estimate(b));
    }

    let mut mean = [0.0; 3];
    let mut covariance_diag = [0.0; 3];
    for axis in 0..3 {
        let precision_a = 1.0 / a.covariance_diag[axis];
        let precision_b = 1.0 / b.covariance_diag[axis];
        let weighted_precision_a = omega * precision_a;
        let weighted_precision_b = (1.0 - omega) * precision_b;
        let fused_precision = weighted_precision_a + weighted_precision_b;
        if !precision_a.is_finite()
            || precision_a <= 0.0
            || !precision_b.is_finite()
            || precision_b <= 0.0
            || !weighted_precision_a.is_finite()
            || !weighted_precision_b.is_finite()
            || !fused_precision.is_finite()
            || fused_precision <= 0.0
        {
            return Err(FusionError::NonFinitePrecision { axis });
        }

        let variance = 1.0 / fused_precision;
        if !variance.is_finite() || variance <= 0.0 {
            return Err(FusionError::InvalidFusedCovariance { axis });
        }

        // Normalize information weights before multiplying potentially large means.
        let coefficient_a = weighted_precision_a / fused_precision;
        let coefficient_b = weighted_precision_b / fused_precision;
        let fused_mean = coefficient_a * a.mean[axis] + coefficient_b * b.mean[axis];
        if !coefficient_a.is_finite() || !coefficient_b.is_finite() || !fused_mean.is_finite() {
            return Err(FusionError::NonFiniteFusedMean { axis });
        }
        covariance_diag[axis] = variance;
        mean[axis] = fused_mean;
    }

    Ok(GaussianEstimate3D::new(mean, covariance_diag))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_ci() {
        let a = GaussianEstimate3D::new([1.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = GaussianEstimate3D::new([3.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let c = covariance_intersection_3d(&a, &b, 0.5).unwrap();
        assert!((c.mean[0] - 2.0).abs() < 0.01);
        assert_eq!(c.covariance_diag, [1.0, 1.0, 1.0]);
    }
    #[test]
    fn ci_rejects_invalid_weight_and_covariance_without_clamping() {
        let good = GaussianEstimate3D::new([0.0; 3], [1.0; 3]);
        assert_eq!(covariance_intersection_3d(&good, &good, f64::NAN), Err(FusionError::NonFiniteOmega));
        assert_eq!(covariance_intersection_3d(&good, &good, -0.1), Err(FusionError::OmegaOutOfRange));
        let invalid = GaussianEstimate3D::new([0.0; 3], [0.0, 1.0, 1.0]);
        assert_eq!(covariance_intersection_3d(&invalid, &good, 0.5), Err(FusionError::InvalidCovariance { input: FusionInput::A, axis: 0 }));
    }

    #[test]
    fn ci_rejects_non_finite_mean_and_reciprocal_overflow() {
        let good = GaussianEstimate3D::new([0.0; 3], [1.0; 3]);
        let invalid_mean = GaussianEstimate3D::new([f64::INFINITY, 0.0, 0.0], [1.0; 3]);
        assert_eq!(covariance_intersection_3d(&invalid_mean, &good, 0.5), Err(FusionError::NonFiniteMean { input: FusionInput::A, axis: 0 }));
        let tiny = GaussianEstimate3D::new([0.0; 3], [f64::from_bits(1), 1.0, 1.0]);
        assert_eq!(covariance_intersection_3d(&tiny, &good, 0.5), Err(FusionError::NonFinitePrecision { axis: 0 }));
    }

    #[test]
    fn ci_weight_endpoints_return_canonicalized_covariance() {
        let a = GaussianEstimate3D {
            mean: [1.0, 2.0, 3.0],
            covariance_diag: [2.0, 3.0, 4.0],
            covariance: [20.0, 30.0, 40.0],
        };
        let b = GaussianEstimate3D::new([9.0, 8.0, 7.0], [5.0, 6.0, 7.0]);
        let fused = covariance_intersection_3d(&a, &b, 1.0).unwrap();
        assert_eq!(fused.mean, a.mean);
        assert_eq!(fused.covariance_diag, a.covariance_diag);
        assert_eq!(fused.covariance, fused.covariance_diag);
    }

    #[test]
    fn ci_deterministic_properties_preserve_bounds_and_swap_symmetry() {
        let mut seed = 0x9e3779b97f4a7c15_u64;
        for _ in 0..10_000 {
            let mut draw = || {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                ((seed >> 11) as f64) / ((1_u64 << 53) as f64)
            };
            let means_a = [draw() * 2_000.0 - 1_000.0, draw() * 2_000.0 - 1_000.0, draw() * 2_000.0 - 1_000.0];
            let means_b = [draw() * 2_000.0 - 1_000.0, draw() * 2_000.0 - 1_000.0, draw() * 2_000.0 - 1_000.0];
            let covariance_a = [10.0_f64.powf(draw() * 12.0 - 6.0), 10.0_f64.powf(draw() * 12.0 - 6.0), 10.0_f64.powf(draw() * 12.0 - 6.0)];
            let covariance_b = [10.0_f64.powf(draw() * 12.0 - 6.0), 10.0_f64.powf(draw() * 12.0 - 6.0), 10.0_f64.powf(draw() * 12.0 - 6.0)];
            let omega = draw();
            let a = GaussianEstimate3D::new(means_a, covariance_a);
            let b = GaussianEstimate3D::new(means_b, covariance_b);
            let forward = covariance_intersection_3d(&a, &b, omega).unwrap();
            let reverse = covariance_intersection_3d(&b, &a, 1.0 - omega).unwrap();
            for axis in 0..3 {
                assert!(forward.mean[axis].is_finite());
                let low = covariance_a[axis].min(covariance_b[axis]);
                let high = covariance_a[axis].max(covariance_b[axis]);
                assert!(forward.covariance_diag[axis] >= low * (1.0 - 1.0e-12));
                assert!(forward.covariance_diag[axis] <= high * (1.0 + 1.0e-12));
                assert!((forward.mean[axis] - reverse.mean[axis]).abs() <= 1.0e-9);
                assert!((forward.covariance_diag[axis] - reverse.covariance_diag[axis]).abs()
                    <= 1.0e-9 * high.max(1.0));
            }
        }
    }

}
