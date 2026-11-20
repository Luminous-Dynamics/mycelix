// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Peer fusion via diagonal Covariance Intersection for decentralized positioning.
//!
//! This module deliberately separates statistical fusion from trust/admission.
//! Peer trust and confidence fields are not interpreted as measurement precision.

use serde::{de, Deserialize, Deserializer, Serialize};
use std::fmt;

/// Error returned when a covariance-intersection input or result is invalid.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FusionError {
    InvalidOmega,
    NonFiniteMean { axis: usize },
    NonFiniteCovariance { axis: usize },
    NonPositiveCovariance { axis: usize },
    NumericalFailure { axis: usize },
    DuplicatePeerId(String),
}

impl fmt::Display for FusionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidOmega => write!(f, "CI omega must be finite and in [0, 1]"),
            Self::NonFiniteMean { axis } => write!(f, "mean on axis {axis} is not finite"),
            Self::NonFiniteCovariance { axis } => {
                write!(f, "covariance on axis {axis} is not finite")
            }
            Self::NonPositiveCovariance { axis } => {
                write!(f, "covariance on axis {axis} must be strictly positive")
            }
            Self::NumericalFailure { axis } => {
                write!(f, "covariance intersection was numerically invalid on axis {axis}")
            }
            Self::DuplicatePeerId(peer_id) => {
                write!(f, "peer ID {peer_id:?} occurs more than once in the fusion set")
            }
        }
    }
}

impl std::error::Error for FusionError {}

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

/// 3D Gaussian estimate with a single canonical diagonal covariance.
///
/// The covariance serde alias accepts legacy payloads that used that field
/// name. Serialisation emits only covariance_diag; a payload containing both
/// names is rejected by Serde as a duplicate field rather than allowing two
/// independently authoritative covariance values.
#[derive(Debug, Clone, Serialize)]
pub struct GaussianEstimate3D {
    pub mean: [f64; 3],
    pub covariance_diag: [f64; 3],
}

#[derive(Deserialize)]
struct GaussianEstimate3DFields {
    mean: [f64; 3],
    #[serde(default)]
    covariance_diag: Option<[f64; 3]>,
    #[serde(default)]
    covariance: Option<[f64; 3]>,
}

impl<'de> Deserialize<'de> for GaussianEstimate3D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let fields = GaussianEstimate3DFields::deserialize(deserializer)?;
        let covariance_diag = match (fields.covariance_diag, fields.covariance) {
            (Some(primary), Some(legacy)) if primary == legacy => primary,
            (Some(_), Some(_)) => {
                return Err(de::Error::custom(
                    "covariance_diag and legacy covariance fields disagree",
                ));
            }
            (Some(primary), None) => primary,
            (None, Some(legacy)) => legacy,
            (None, None) => return Err(de::Error::missing_field("covariance_diag")),
        };
        Ok(Self {
            mean: fields.mean,
            covariance_diag,
        })
    }
}

impl GaussianEstimate3D {
    pub fn new(mean: [f64; 3], cov: [f64; 3]) -> Self {
        Self {
            mean,
            covariance_diag: cov,
        }
    }

    pub fn from_diagonal_sigma(mean: [f64; 3], sigma: impl Into<SigmaInput>) -> Self {
        let s: SigmaInput = sigma.into();
        let c = [s.0[0] * s.0[0], s.0[1] * s.0[1], s.0[2] * s.0[2]];
        Self {
            mean,
            covariance_diag: c,
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

    fn validate_for_fusion(&self) -> Result<(), FusionError> {
        for axis in 0..3 {
            if !self.mean[axis].is_finite() {
                return Err(FusionError::NonFiniteMean { axis });
            }
            let variance = self.covariance_diag[axis];
            if !variance.is_finite() {
                return Err(FusionError::NonFiniteCovariance { axis });
            }
            if variance <= 0.0 {
                return Err(FusionError::NonPositiveCovariance { axis });
            }
        }
        Ok(())
    }
}

impl PartialEq for GaussianEstimate3D {
    fn eq(&self, other: &Self) -> bool {
        self.mean == other.mean && self.covariance_diag == other.covariance_diag
    }
}

/// A peer's position estimate.
///
/// trust_weight and confidence belong to upstream admission/policy. They are
/// intentionally not converted into covariance precision by this module.
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

/// Trait for evidence-qualified estimates that may be published to the DHT.
///
/// A mathematical Gaussian estimate alone does not implement this trait:
/// publication requires explicit source count and time metadata.
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

/// Peer fusion engine using deterministic sequential diagonal CI.
///
/// Inputs are sorted by peer ID before fusion. Each pairwise update uses
/// omega=0.5. This is an explicit, reproducible sequential approximation; it
/// is not an optimized joint multi-source CI weight solution. The peer set is
/// assumed to have passed admission policy already.
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
        if self.max_peers == 0 {
            return;
        }
        if self.peers.len() >= self.max_peers {
            self.peers.remove(0);
        }
        self.peers.push(peer);
    }

    pub fn fuse(&self) -> Result<Option<GaussianEstimate3D>, FusionError> {
        if self.peers.is_empty() {
            return Ok(None);
        }

        let mut peers: Vec<&PeerEstimate3D> = self.peers.iter().collect();
        peers.sort_by(|left, right| left.peer_id.cmp(&right.peer_id));

        for pair in peers.windows(2) {
            if pair[0].peer_id == pair[1].peer_id {
                return Err(FusionError::DuplicatePeerId(pair[0].peer_id.clone()));
            }
        }

        let mut fused = peers[0].estimate.clone();
        fused.validate_for_fusion()?;
        for peer in peers.iter().skip(1) {
            fused = covariance_intersection_3d(&fused, &peer.estimate, 0.5)?;
        }
        Ok(Some(fused))
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

    pub fn fused_estimate(&self) -> Result<Option<GaussianEstimate3D>, FusionError> {
        self.fuse()
    }
}

/// Fallible diagonal Covariance Intersection for two 3D Gaussian estimates.
///
/// This applies the information-form CI equations independently on each
/// coordinate axis. It rejects invalid inputs instead of clamping invalid
/// weights or manufacturing precision for singular/negative covariance.
/// Full non-diagonal covariance support is intentionally not implied.
pub fn covariance_intersection_3d(
    a: &GaussianEstimate3D,
    b: &GaussianEstimate3D,
    omega: f64,
) -> Result<GaussianEstimate3D, FusionError> {
    a.validate_for_fusion()?;
    b.validate_for_fusion()?;
    if !omega.is_finite() || !(0.0..=1.0).contains(&omega) {
        return Err(FusionError::InvalidOmega);
    }

    if omega == 1.0 {
        return Ok(a.clone());
    }
    if omega == 0.0 {
        return Ok(b.clone());
    }

    let mut mean = [0.0; 3];
    let mut covariance_diag = [0.0; 3];
    for axis in 0..3 {
        let information_a = omega / a.covariance_diag[axis];
        let information_b = (1.0 - omega) / b.covariance_diag[axis];
        let total_information = information_a + information_b;
        if !information_a.is_finite()
            || !information_b.is_finite()
            || !total_information.is_finite()
            || total_information <= 0.0
        {
            return Err(FusionError::NumericalFailure { axis });
        }

        let variance = 1.0 / total_information;
        let weight_a = information_a / total_information;
        let weight_b = information_b / total_information;
        let fused_mean = weight_a * a.mean[axis] + weight_b * b.mean[axis];
        if !variance.is_finite() || variance <= 0.0 || !fused_mean.is_finite() {
            return Err(FusionError::NumericalFailure { axis });
        }
        covariance_diag[axis] = variance;
        mean[axis] = fused_mean;
    }

    Ok(GaussianEstimate3D {
        mean,
        covariance_diag,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn peer(
        peer_id: &str,
        mean: [f64; 3],
        covariance_diag: [f64; 3],
        trust_weight: f64,
    ) -> PeerEstimate3D {
        PeerEstimate3D {
            peer_id: peer_id.to_string(),
            estimate: GaussianEstimate3D::new(mean, covariance_diag),
            trust_weight,
            timestamp_us: 1,
            confidence: 0.9,
        }
    }


    #[test]
    fn legacy_dual_covariance_payload_migrates_only_when_values_agree() {
        let matching = r#"{"mean":[1.0,2.0,3.0],"covariance_diag":[4.0,5.0,6.0],"covariance":[4.0,5.0,6.0]}"#;
        let decoded: GaussianEstimate3D = serde_json::from_str(matching).unwrap();
        assert_eq!(decoded, GaussianEstimate3D::new([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]));

        let conflicting = r#"{"mean":[1.0,2.0,3.0],"covariance_diag":[4.0,5.0,6.0],"covariance":[40.0,50.0,60.0]}"#;
        assert!(serde_json::from_str::<GaussianEstimate3D>(conflicting).is_err());
    }

    #[test]
    fn canonical_payload_and_legacy_single_field_payload_are_accepted() {
        let canonical = r#"{"mean":[1.0,2.0,3.0],"covariance_diag":[4.0,5.0,6.0]}"#;
        let decoded: GaussianEstimate3D = serde_json::from_str(canonical).unwrap();
        assert_eq!(decoded, GaussianEstimate3D::new([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]));

        let legacy = r#"{"mean":[1.0,2.0,3.0],"covariance":[4.0,5.0,6.0]}"#;
        let decoded: GaussianEstimate3D = serde_json::from_str(legacy).unwrap();
        assert_eq!(decoded, GaussianEstimate3D::new([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]));
    }

    #[test]
    fn equal_covariance_ci_does_not_invent_independent_information() {
        let a = GaussianEstimate3D::new([1.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let b = GaussianEstimate3D::new([3.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let fused = covariance_intersection_3d(&a, &b, 0.5).unwrap();
        assert!((fused.mean[0] - 2.0).abs() < 1e-12);
        assert!((fused.covariance_diag[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn unequal_covariance_uses_information_weights() {
        let a = GaussianEstimate3D::new([0.0, 0.0, 0.0], [4.0, 4.0, 4.0]);
        let b = GaussianEstimate3D::new([10.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        let fused = covariance_intersection_3d(&a, &b, 0.5).unwrap();
        assert!((fused.mean[0] - 8.0).abs() < 1e-12);
        assert!((fused.covariance_diag[0] - 1.6).abs() < 1e-12);
    }

    #[test]
    fn ci_endpoints_return_the_selected_input() {
        let a = GaussianEstimate3D::new([1.0, 2.0, 3.0], [1.0, 2.0, 3.0]);
        let b = GaussianEstimate3D::new([4.0, 5.0, 6.0], [4.0, 5.0, 6.0]);
        assert_eq!(covariance_intersection_3d(&a, &b, 1.0).unwrap(), a);
        assert_eq!(covariance_intersection_3d(&a, &b, 0.0).unwrap(), b);
    }

    #[test]
    fn ci_rejects_invalid_weight_and_covariance() {
        let valid = GaussianEstimate3D::new([0.0; 3], [1.0; 3]);
        assert_eq!(
            covariance_intersection_3d(&valid, &valid, f64::NAN),
            Err(FusionError::InvalidOmega)
        );
        assert_eq!(
            covariance_intersection_3d(&valid, &valid, 1.1),
            Err(FusionError::InvalidOmega)
        );

        let zero_variance = GaussianEstimate3D::new([0.0; 3], [0.0, 1.0, 1.0]);
        assert_eq!(
            covariance_intersection_3d(&zero_variance, &valid, 0.5),
            Err(FusionError::NonPositiveCovariance { axis: 0 })
        );

        let negative_variance = GaussianEstimate3D::new([0.0; 3], [-1.0, 1.0, 1.0]);
        assert_eq!(
            covariance_intersection_3d(&negative_variance, &valid, 0.5),
            Err(FusionError::NonPositiveCovariance { axis: 0 })
        );

        let non_finite_mean = GaussianEstimate3D::new([f64::INFINITY, 0.0, 0.0], [1.0; 3]);
        assert_eq!(
            covariance_intersection_3d(&non_finite_mean, &valid, 0.5),
            Err(FusionError::NonFiniteMean { axis: 0 })
        );
    }

    #[test]
    fn peer_fusion_is_sorted_and_does_not_rewrite_covariance_from_trust() {
        let a = peer("a", [0.0; 3], [4.0; 3], 0.01);
        let b = peer("b", [10.0; 3], [1.0; 3], 0.99);

        let mut first = PeerFusion3D::new(4);
        first.add_peer(a.clone());
        first.add_peer(b.clone());

        let mut second = PeerFusion3D::new(4);
        second.add_peer(peer("b", [10.0; 3], [1.0; 3], 0.01));
        second.add_peer(peer("a", [0.0; 3], [4.0; 3], 0.99));

        assert_eq!(first.fuse().unwrap(), second.fuse().unwrap());
        let fused = first.fuse().unwrap().unwrap();
        assert!((fused.covariance_diag[0] - 1.6).abs() < 1e-12);
    }

    #[test]
    fn duplicate_peer_ids_are_rejected() {
        let mut fusion = PeerFusion3D::new(4);
        fusion.add_peer(peer("same", [0.0; 3], [1.0; 3], 0.5));
        fusion.add_peer(peer("same", [1.0; 3], [1.0; 3], 0.5));
        assert_eq!(
            fusion.fuse(),
            Err(FusionError::DuplicatePeerId("same".to_string()))
        );
    }

    #[test]
    fn zero_capacity_does_not_panic_or_accept_peers() {
        let mut fusion = PeerFusion3D::new(0);
        fusion.add_peer(peer("ignored", [0.0; 3], [1.0; 3], 0.5));
        assert_eq!(fusion.peer_count(), 0);
        assert_eq!(fusion.fuse().unwrap(), None);
    }
}
