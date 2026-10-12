// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Conjunction Analysis and Collision Probability
//!
//! Implements algorithms for:
//! - Close approach detection (screening)
//! - Probability of collision (Pc) calculation
//! - Conjunction Data Message (CDM) generation
//! - Risk assessment and prioritization
//!
//! # Key Concepts
//!
//! **Miss Distance**: The minimum distance between two objects at closest approach.
//! Alone, this is insufficient - a 1km miss with 10km uncertainty is dangerous,
//! but a 1km miss with 10m uncertainty is safe.
//!
//! **Probability of Collision (Pc)**: The integral of the combined position PDF
//! over the hard-body radius. This properly accounts for uncertainty.
//!
//! **Hard-Body Radius (HBR)**: Combined size of both objects plus safety margin.
//! Typically 5-50 meters depending on object sizes.

use chrono::{DateTime, Utc};
use nalgebra::{Matrix2, Vector3};
use serde::{Deserialize, Serialize};

use crate::covariance::{CovarianceFrame, CovarianceMatrix, CovarianceSource};
use crate::state::{OrbitalState, ReferenceFrame, StateVector};
use thiserror::Error;

/// Result of conjunction screening
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConjunctionAssessment {
    /// Time of closest approach (TCA)
    pub tca: DateTime<Utc>,

    /// Primary object (the one we're protecting)
    pub primary_norad_id: u32,

    /// Secondary object (potential threat)
    pub secondary_norad_id: u32,

    /// Miss distance at TCA (km)
    pub miss_distance_km: f64,

    /// Relative velocity at TCA (km/s)
    pub relative_velocity_kms: f64,

    /// Probability of collision
    pub collision_probability: CollisionProbability,

    /// Combined hard-body radius used (meters)
    pub hard_body_radius_m: f64,

    /// Risk assessment
    pub risk_level: RiskLevel,

    /// Primary state at TCA
    pub primary_state: StateVector,

    /// Secondary state at TCA
    pub secondary_state: StateVector,

    /// Combined covariance at TCA (if available)
    pub combined_covariance: Option<CovarianceMatrix>,
}

/// Probability of collision with confidence
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CollisionProbability {
    /// Point estimate of Pc
    pub pc: f64,

    /// Lower bound (conservative)
    pub pc_lower: f64,

    /// Upper bound (worst case)
    pub pc_upper: f64,

    /// Method used for calculation
    pub method: PcMethod,

    /// Whether covariance data was available
    pub has_covariance: bool,
}

/// Method used for Pc calculation
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum PcMethod {
    /// Alfano's method (2D projection)
    Alfano2D,
    /// Foster's method (1D simplification)
    Foster1D,
    /// Monte Carlo simulation
    MonteCarlo { samples: u32 },
    /// Estimated from miss distance only (low confidence)
    MissDistanceOnly,
}

/// Risk level for prioritization
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
pub enum RiskLevel {
    /// No action needed (Pc < 1e-7 or miss > 5km)
    Negligible,
    /// Monitor but no immediate action (Pc < 1e-5)
    Low,
    /// Increased monitoring, prepare for possible action (Pc < 1e-4)
    Medium,
    /// Maneuver planning should begin (Pc < 1e-3)
    High,
    /// Emergency - maneuver required (Pc >= 1e-3)
    Emergency,
}

impl RiskLevel {
    pub fn from_pc(pc: f64) -> Self {
        if pc >= 1e-3 {
            RiskLevel::Emergency
        } else if pc >= 1e-4 {
            RiskLevel::High
        } else if pc >= 1e-5 {
            RiskLevel::Medium
        } else if pc >= 1e-7 {
            RiskLevel::Low
        } else {
            RiskLevel::Negligible
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            RiskLevel::Negligible => "No action required",
            RiskLevel::Low => "Standard monitoring",
            RiskLevel::Medium => "Enhanced monitoring, evaluate options",
            RiskLevel::High => "Begin maneuver planning",
            RiskLevel::Emergency => "Execute collision avoidance maneuver",
        }
    }
}

/// Input validation failures from the checked conjunction-assessment path.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ConjunctionAssessmentError {
    #[error("the two states have different epochs")]
    EpochMismatch,
    #[error("the two states use different reference frames: {primary:?} vs {secondary:?}")]
    StateFrameMismatch {
        primary: ReferenceFrame,
        secondary: ReferenceFrame,
    },
    #[error("{object} state contains a non-finite position or velocity component")]
    NonFiniteState { object: &'static str },
    #[error("{object} state has no covariance; checked assessment will not synthesize one")]
    MissingCovariance { object: &'static str },
    #[error("{object} covariance frame {covariance:?} does not match state frame {state:?}")]
    CovarianceFrameMismatch {
        object: &'static str,
        state: ReferenceFrame,
        covariance: CovarianceFrame,
    },
    #[error("{object} covariance is non-finite, invalid, or singular for the collision calculation")]
    InvalidCovariance { object: &'static str },
    #[error("{object} covariance source is unknown")]
    UnknownCovarianceProvenance { object: &'static str },
    #[error("{object} covariance source metadata is invalid")]
    InvalidCovarianceProvenance { object: &'static str },
    #[error("relative speed is too small to define a stable 2D encounter plane")]
    DegenerateEncounterGeometry,
    #[error("hard-body radius must be finite and greater than zero")]
    InvalidHardBodyRadius,
}

/// Conjunction analyzer for computing collision probabilities
pub struct ConjunctionAnalyzer {
    /// Default hard-body radius (meters)
    default_hbr_m: f64,

    /// Screening threshold (km)
    screening_threshold_km: f64,

    /// Time step for TCA refinement (seconds)
    _tca_refinement_step_s: f64,
}

impl Default for ConjunctionAnalyzer {
    fn default() -> Self {
        Self {
            default_hbr_m: 20.0,         // 20 meters combined radius
            screening_threshold_km: 5.0, // Screen for approaches within 5 km
            _tca_refinement_step_s: 1.0, // 1-second refinement
        }
    }
}

impl ConjunctionAnalyzer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the hard-body radius
    pub fn with_hbr(mut self, hbr_m: f64) -> Self {
        self.default_hbr_m = hbr_m;
        self
    }

    /// Set screening threshold
    pub fn with_screening_threshold(mut self, threshold_km: f64) -> Self {
        self.screening_threshold_km = threshold_km;
        self
    }

    /// Legacy unchecked assessment retained for compatibility.
    ///
    /// This method assumes caller-provided states have matching epochs and frames
    /// and appropriately valid covariances. It may use a miss-distance-only estimate
    /// or synthesize a default covariance when the inputs are incomplete.
    ///
    /// Do not use this method in consequential screening or traffic-control paths.
    /// New code should call try_assess, which rejects missing or invalid uncertainty
    /// data rather than treating an invented value as observed evidence.
    pub fn assess(
        &self,
        primary: &OrbitalState,
        secondary: &OrbitalState,
    ) -> ConjunctionAssessment {
        let miss_distance = primary.state.distance_to(&secondary.state);
        let relative_velocity = primary.state.relative_velocity(&secondary.state);

        let collision_probability =
            self.calculate_pc(primary, secondary, miss_distance, self.default_hbr_m);
        let risk_level = RiskLevel::from_pc(collision_probability.pc);

        let combined_covariance = match (&primary.covariance, &secondary.covariance) {
            (Some(c1), Some(c2)) => c1.fuse(c2),
            (Some(c), None) | (None, Some(c)) => Some(c.clone()),
            (None, None) => None,
        };

        ConjunctionAssessment {
            tca: primary.epoch,
            primary_norad_id: primary.norad_id,
            secondary_norad_id: secondary.norad_id,
            miss_distance_km: miss_distance,
            relative_velocity_kms: relative_velocity,
            collision_probability,
            hard_body_radius_m: self.default_hbr_m,
            risk_level,
            primary_state: primary.state.clone(),
            secondary_state: secondary.state.clone(),
            combined_covariance,
        }
    }

    /// Assess a conjunction only when temporal, coordinate-frame, and covariance
    /// invariants hold.
    ///
    /// Missing covariance is a fail-closed error for this API. Callers may separately
    /// report an inconclusive pair, but must not convert absent uncertainty into a
    /// numeric collision-risk estimate or imply calibrated probability.
    pub fn try_assess(
        &self,
        primary: &OrbitalState,
        secondary: &OrbitalState,
    ) -> Result<ConjunctionAssessment, ConjunctionAssessmentError> {
        if primary.epoch != secondary.epoch {
            return Err(ConjunctionAssessmentError::EpochMismatch);
        }

        if primary.frame != secondary.frame {
            return Err(ConjunctionAssessmentError::StateFrameMismatch {
                primary: primary.frame,
                secondary: secondary.frame,
            });
        }

        if !primary.state.to_array().iter().all(|value| value.is_finite()) {
            return Err(ConjunctionAssessmentError::NonFiniteState { object: "primary" });
        }
        if !secondary.state.to_array().iter().all(|value| value.is_finite()) {
            return Err(ConjunctionAssessmentError::NonFiniteState { object: "secondary" });
        }

        if !self.default_hbr_m.is_finite() || self.default_hbr_m <= 0.0 {
            return Err(ConjunctionAssessmentError::InvalidHardBodyRadius);
        }

        Self::validate_covariance(primary, "primary")?;
        Self::validate_covariance(secondary, "secondary")?;

        let miss_distance = primary.state.distance_to(&secondary.state);
        let relative_velocity = primary.state.relative_velocity(&secondary.state);
        if !miss_distance.is_finite() || !relative_velocity.is_finite() {
            return Err(ConjunctionAssessmentError::NonFiniteState { object: "pair" });
        }
        if relative_velocity <= 1e-10 {
            return Err(ConjunctionAssessmentError::DegenerateEncounterGeometry);
        }

        let collision_probability =
            self.calculate_pc(primary, secondary, miss_distance, self.default_hbr_m);
        if !collision_probability.pc.is_finite()
            || !collision_probability.pc_lower.is_finite()
            || !collision_probability.pc_upper.is_finite()
        {
            return Err(ConjunctionAssessmentError::InvalidCovariance { object: "combined" });
        }
        let risk_level = RiskLevel::from_pc(collision_probability.pc);
        let combined_covariance = match (&primary.covariance, &secondary.covariance) {
            (Some(c1), Some(c2)) => c1
                .fuse(c2)
                .ok_or(ConjunctionAssessmentError::InvalidCovariance { object: "combined" })?,
            _ => unreachable!("try_assess validates that both covariances are present"),
        };

        Ok(ConjunctionAssessment {
            tca: primary.epoch,
            primary_norad_id: primary.norad_id,
            secondary_norad_id: secondary.norad_id,
            miss_distance_km: miss_distance,
            relative_velocity_kms: relative_velocity,
            collision_probability,
            hard_body_radius_m: self.default_hbr_m,
            risk_level,
            primary_state: primary.state.clone(),
            secondary_state: secondary.state.clone(),
            combined_covariance,
        })
    }

    fn validate_covariance(
        state: &OrbitalState,
        object: &'static str,
    ) -> Result<(), ConjunctionAssessmentError> {
        let covariance = state
            .covariance
            .as_ref()
            .ok_or(ConjunctionAssessmentError::MissingCovariance { object })?;
        if !covariance.is_valid() || covariance.matrix().clone().cholesky().is_none() {
            return Err(ConjunctionAssessmentError::InvalidCovariance { object });
        }
        let frame_matches = matches!(
            (state.frame, covariance.frame),
            (ReferenceFrame::TEME, CovarianceFrame::TEME)
                | (ReferenceFrame::ECI, CovarianceFrame::ECI)
        );
        if !frame_matches {
            return Err(ConjunctionAssessmentError::CovarianceFrameMismatch {
                object,
                state: state.frame,
                covariance: covariance.frame,
            });
        }
        match &covariance.source {
            CovarianceSource::Official => {}
            CovarianceSource::Estimated { tle_age_hours }
                if tle_age_hours.is_finite() && *tle_age_hours >= 0.0 => {}
            CovarianceSource::Observed { sensor_accuracy_km }
                if sensor_accuracy_km.is_finite() && *sensor_accuracy_km > 0.0 => {}
            CovarianceSource::Fused { source_count } if *source_count >= 2 => {}
            CovarianceSource::Unknown => {
                return Err(ConjunctionAssessmentError::UnknownCovarianceProvenance { object });
            }
            _ => return Err(ConjunctionAssessmentError::InvalidCovarianceProvenance { object }),
        }
        Ok(())
    }

    /// Calculate probability of collision
    fn calculate_pc(
        &self,
        primary: &OrbitalState,
        secondary: &OrbitalState,
        miss_distance_km: f64,
        hbr_m: f64,
    ) -> CollisionProbability {
        let hbr_km = hbr_m / 1000.0;

        // Get covariances
        let has_covariance = primary.has_covariance() || secondary.has_covariance();

        if !has_covariance {
            // Fallback: estimate Pc from miss distance alone
            // This is very approximate and should be flagged
            return self.pc_from_miss_distance(miss_distance_km, hbr_km);
        }

        // Get combined covariance in encounter frame
        let cov_primary = primary.covariance.as_ref().cloned().unwrap_or_default();
        let cov_secondary = secondary.covariance.as_ref().cloned().unwrap_or_default();

        // Combined position covariance
        let combined_pos_cov =
            cov_primary.position_covariance() + cov_secondary.position_covariance();

        // Relative position (miss vector)
        let miss_vector = secondary.state.position() - primary.state.position();

        // Relative velocity for encounter frame
        let rel_vel = secondary.state.velocity() - primary.state.velocity();

        // Project to 2D encounter plane (perpendicular to relative velocity)
        let (projected_miss, cov_2d) =
            self.project_to_encounter_plane(miss_vector, rel_vel, combined_pos_cov);

        // Alfano's 2D Pc calculation
        let pc = self.alfano_2d_pc(projected_miss, cov_2d, hbr_km);

        // Estimate bounds (±1 order of magnitude for TLE-based)
        let pc_lower = pc / 10.0;
        let pc_upper = (pc * 10.0).min(1.0);

        CollisionProbability {
            pc,
            pc_lower,
            pc_upper,
            method: PcMethod::Alfano2D,
            has_covariance: true,
        }
    }

    /// Legacy miss-distance-only fallback used solely by unchecked assess.
    /// This is a crude screening heuristic, not a validated collision-probability
    /// model. The checked try_assess API refuses missing covariance instead.
    fn pc_from_miss_distance(&self, miss_km: f64, hbr_km: f64) -> CollisionProbability {
        // Very rough approximation assuming typical LEO uncertainties
        // This should be used only as a screening metric
        let assumed_sigma_km = 1.0; // Assume 1 km 1-sigma position uncertainty

        // Simple Gaussian approximation
        let x = miss_km / assumed_sigma_km;
        let pc = (-(x * x) / 2.0).exp() * (hbr_km / assumed_sigma_km).powi(2);

        CollisionProbability {
            pc: pc.min(1.0),
            pc_lower: pc / 100.0, // Very uncertain
            pc_upper: (pc * 100.0).min(1.0),
            method: PcMethod::MissDistanceOnly,
            has_covariance: false,
        }
    }

    /// Project to 2D encounter plane
    fn project_to_encounter_plane(
        &self,
        miss_vector: Vector3<f64>,
        rel_velocity: Vector3<f64>,
        cov_3d: nalgebra::Matrix3<f64>,
    ) -> (nalgebra::Vector2<f64>, Matrix2<f64>) {
        // Build encounter frame basis
        // Guard against zero relative velocity (co-moving objects)
        let v_hat = if rel_velocity.norm() < 1e-10 {
            // Fall back to miss vector direction, or arbitrary axis
            if miss_vector.norm() < 1e-10 {
                Vector3::new(0.0, 0.0, 1.0)
            } else {
                miss_vector.normalize()
            }
        } else {
            rel_velocity.normalize()
        };

        // Choose arbitrary perpendicular vector
        let temp = if v_hat.x.abs() < 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };

        let u1 = v_hat.cross(&temp).normalize();
        let u2 = v_hat.cross(&u1);

        // Project miss vector to encounter plane
        let miss_2d = nalgebra::Vector2::new(miss_vector.dot(&u1), miss_vector.dot(&u2));

        // Project covariance to encounter plane
        // C_2d = P * C_3d * P^T where P is 2x3 projection matrix
        let proj = nalgebra::Matrix2x3::from_rows(&[u1.transpose(), u2.transpose()]);

        let cov_2d = proj * cov_3d * proj.transpose();

        (miss_2d, cov_2d)
    }

    /// Alfano's 2D Pc calculation
    fn alfano_2d_pc(
        &self,
        miss_2d: nalgebra::Vector2<f64>,
        cov_2d: Matrix2<f64>,
        hbr_km: f64,
    ) -> f64 {
        // Eigendecomposition of covariance
        let det = cov_2d.determinant();
        if det <= 0.0 {
            return 0.0; // Invalid covariance
        }

        let trace = cov_2d.trace();
        let discriminant = (trace * trace - 4.0 * det).max(0.0);
        let sqrt_disc = discriminant.sqrt();

        let lambda1 = (trace + sqrt_disc) / 2.0;
        let lambda2 = (trace - sqrt_disc) / 2.0;

        let sigma1 = lambda1.sqrt();
        let sigma2 = lambda2.sqrt();

        // Normalized miss distance
        let inv_cov = cov_2d.try_inverse().unwrap_or(Matrix2::identity());
        let d_squared = (miss_2d.transpose() * inv_cov * miss_2d)[(0, 0)];

        // Hard-body radius normalized
        let r_norm = hbr_km / (sigma1 * sigma2).sqrt();

        // Pc approximation (Foster-style simplification of full integral)
        // Pc ≈ (π * r²) / (2π * σ1 * σ2) * exp(-d²/2)
        let pc = (r_norm * r_norm) * (-d_squared / 2.0).exp() / 2.0;

        pc.clamp(0.0, 1.0)
    }
}

/// Screen multiple objects for conjunctions
pub fn screen_catalog(
    protected: &[OrbitalState],
    catalog: &[OrbitalState],
    threshold_km: f64,
) -> Vec<(u32, u32, f64)> {
    let mut conjunctions = Vec::new();

    for p in protected {
        for s in catalog {
            if p.norad_id == s.norad_id {
                continue;
            }

            let miss = p.state.distance_to(&s.state);
            if miss < threshold_km {
                conjunctions.push((p.norad_id, s.norad_id, miss));
            }
        }
    }

    // Sort by miss distance
    conjunctions.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap());
    conjunctions
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::covariance::{CovarianceFrame, CovarianceMatrix};
    use crate::state::ReferenceFrame;
    use nalgebra::Matrix6;

    fn checked_test_state(norad_id: u32, epoch: chrono::DateTime<Utc>) -> OrbitalState {
        let mut covariance =
            CovarianceMatrix::diagonal([1.0, 1.0, 1.0, 0.01, 0.01, 0.01]);
        covariance.frame = CovarianceFrame::TEME;
        covariance.source = CovarianceSource::Estimated { tle_age_hours: 0.0 };
        OrbitalState::new(
            norad_id,
            epoch,
            StateVector::new(
                7000.0 + norad_id as f64 * 0.0001,
                0.0,
                0.0,
                0.0,
                7.5 + norad_id as f64 * 1e-7,
                0.0,
            ),
            crate::state::DataSource::SpaceTrack,
        )
        .with_covariance(covariance)
    }

    #[test]
    fn checked_assessment_accepts_matching_finite_teme_inputs() {
        let epoch = Utc::now();
        let primary = checked_test_state(25544, epoch);
        let secondary = checked_test_state(12345, epoch);
        let result = ConjunctionAnalyzer::new().try_assess(&primary, &secondary);
        let assessment = result.expect("matching TEME states with valid covariances should assess");
        assert!(assessment.collision_probability.has_covariance);
        assert_eq!(assessment.collision_probability.method, PcMethod::Alfano2D);
        assert!(assessment.combined_covariance.is_some());
    }

    #[test]
    fn checked_assessment_rejects_different_epochs() {
        let epoch = Utc::now();
        let primary = checked_test_state(25544, epoch);
        let secondary = checked_test_state(12345, epoch + chrono::Duration::seconds(1));
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::EpochMismatch
        );
    }

    #[test]
    fn checked_assessment_rejects_different_state_frames() {
        let epoch = Utc::now();
        let primary = checked_test_state(25544, epoch);
        let mut secondary = checked_test_state(12345, epoch);
        secondary.frame = ReferenceFrame::ECI;
        if let Some(covariance) = secondary.covariance.as_mut() {
            covariance.frame = CovarianceFrame::ECI;
        }
        assert!(matches!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary),
            Err(ConjunctionAssessmentError::StateFrameMismatch { .. })
        ));
    }

    #[test]
    fn checked_assessment_rejects_state_covariance_frame_mismatch() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        primary.covariance.as_mut().unwrap().frame = CovarianceFrame::ECI;
        let secondary = checked_test_state(12345, epoch);
        assert!(matches!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary),
            Err(ConjunctionAssessmentError::CovarianceFrameMismatch { object: "primary", .. })
        ));
    }

    #[test]
    fn checked_assessment_rejects_unknown_covariance_provenance() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        primary.covariance.as_mut().unwrap().source = CovarianceSource::Unknown;
        let secondary = checked_test_state(12345, epoch);
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::UnknownCovarianceProvenance { object: "primary" }
        );
    }

    #[test]
    fn checked_assessment_rejects_invalid_covariance_source_metadata() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        primary.covariance.as_mut().unwrap().source =
            CovarianceSource::Estimated { tle_age_hours: f64::NAN };
        let secondary = checked_test_state(12345, epoch);
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::InvalidCovarianceProvenance { object: "primary" }
        );
    }

    #[test]
    fn checked_assessment_rejects_missing_covariance() {
        let epoch = Utc::now();
        let primary = OrbitalState::new(
            25544,
            epoch,
            StateVector::new(7000.0, 0.0, 0.0, 0.0, 7.5, 0.0),
            crate::state::DataSource::SpaceTrack,
        );
        let secondary = checked_test_state(12345, epoch);
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::MissingCovariance { object: "primary" }
        );
    }

    #[test]
    fn checked_assessment_rejects_non_finite_state_and_covariance() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        primary.state.vz = f64::INFINITY;
        let secondary = checked_test_state(12345, epoch);
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::NonFiniteState { object: "primary" }
        );

        let mut primary = checked_test_state(25544, epoch);
        let mut matrix = Matrix6::identity();
        matrix[(0, 0)] = f64::NAN;
        let mut covariance = CovarianceMatrix::from_matrix(matrix);
        covariance.frame = CovarianceFrame::TEME;
        primary.covariance = Some(covariance);
        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::InvalidCovariance { object: "primary" }
        );
    }

    #[test]
    fn checked_assessment_rejects_degenerate_encounter_geometry() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        let mut secondary = checked_test_state(12345, epoch);
        primary.state.vx = 0.0;
        primary.state.vy = 7.5;
        primary.state.vz = 0.0;
        secondary.state.vx = 0.0;
        secondary.state.vy = 7.5;
        secondary.state.vz = 0.0;

        assert_eq!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary).unwrap_err(),
            ConjunctionAssessmentError::DegenerateEncounterGeometry
        );
    }

    #[test]
    fn checked_assessment_rejects_singular_covariance_and_invalid_hbr() {
        let epoch = Utc::now();
        let mut primary = checked_test_state(25544, epoch);
        let mut matrix = Matrix6::identity();
        matrix[(0, 0)] = 0.0;
        let mut covariance = CovarianceMatrix::from_matrix(matrix);
        covariance.frame = CovarianceFrame::TEME;
        primary.covariance = Some(covariance);
        let secondary = checked_test_state(12345, epoch);
        assert!(matches!(
            ConjunctionAnalyzer::new().try_assess(&primary, &secondary),
            Err(ConjunctionAssessmentError::InvalidCovariance { object: "primary" })
        ));

        assert_eq!(
            ConjunctionAnalyzer::new().with_hbr(f64::NAN).try_assess(
                &checked_test_state(25544, epoch),
                &checked_test_state(12345, epoch)
            ).unwrap_err(),
            ConjunctionAssessmentError::InvalidHardBodyRadius
        );
    }

    #[test]
    fn test_risk_level_from_pc() {
        assert_eq!(RiskLevel::from_pc(1e-8), RiskLevel::Negligible);
        assert_eq!(RiskLevel::from_pc(1e-6), RiskLevel::Low);
        assert_eq!(RiskLevel::from_pc(5e-5), RiskLevel::Medium);
        assert_eq!(RiskLevel::from_pc(5e-4), RiskLevel::High);
        assert_eq!(RiskLevel::from_pc(5e-3), RiskLevel::Emergency);
    }

    #[test]
    fn test_conjunction_assessment() {
        use crate::state::DataSource;
        use chrono::Utc;

        let now = Utc::now();

        // Two objects 1 km apart
        let primary = OrbitalState::new(
            25544,
            now,
            StateVector::new(7000.0, 0.0, 0.0, 0.0, 7.5, 0.0),
            DataSource::SpaceTrack,
        )
        .with_covariance(CovarianceMatrix::diagonal([
            0.5, 0.5, 0.5, 0.001, 0.001, 0.001,
        ]));

        let secondary = OrbitalState::new(
            12345,
            now,
            StateVector::new(7001.0, 0.0, 0.0, 0.0, 7.5, 0.0),
            DataSource::SpaceTrack,
        )
        .with_covariance(CovarianceMatrix::diagonal([
            0.5, 0.5, 0.5, 0.001, 0.001, 0.001,
        ]));

        let analyzer = ConjunctionAnalyzer::new();
        let assessment = analyzer.assess(&primary, &secondary);

        assert!((assessment.miss_distance_km - 1.0).abs() < 0.01);
        assert!(assessment.collision_probability.has_covariance);
    }

    /// End-to-end pipeline test: TLE parse -> SGP4 propagate -> conjunction assess
    #[test]
    fn test_tle_to_conjunction_pipeline() {
        use crate::propagator::Propagator;
        use crate::tle::TwoLineElement;
        use chrono::Duration;

        // ISS TLE
        let iss_tle = TwoLineElement::parse(
            "ISS (ZARYA)\n\
             1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9997\n\
             2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391424577",
        )
        .expect("Failed to parse ISS TLE");

        // Fictional debris TLE (same epoch, slightly different orbit)
        let debris_tle = TwoLineElement::parse(
            "DEBRIS\n\
             1 49863U 21999A   24001.50000000  .00001000  00000-0  50000-4 0  9992\n\
             2 49863  51.6500 247.5000 0007000 131.0000 325.0000 15.72000000 10005",
        )
        .expect("Failed to parse debris TLE");

        // Create propagators
        let iss_prop = Propagator::from_tle(&iss_tle).expect("ISS propagator failed");
        let debris_prop = Propagator::from_tle(&debris_tle).expect("Debris propagator failed");

        // Propagate both to 1 hour after epoch
        let screen_time = iss_tle.epoch + Duration::hours(1);
        let iss_state = iss_prop
            .propagate_to(screen_time)
            .expect("ISS propagation failed");
        let debris_state = debris_prop
            .propagate_to(screen_time)
            .expect("Debris propagation failed");

        // Assess conjunction
        let analyzer = ConjunctionAnalyzer::new();
        let assessment = analyzer.assess(&iss_state, &debris_state);

        // Verify the pipeline produced meaningful results
        assert_eq!(assessment.primary_norad_id, 25544);
        assert_eq!(assessment.secondary_norad_id, 49863);
        assert!(
            assessment.miss_distance_km >= 0.0,
            "Miss distance must be non-negative"
        );
        assert!(
            assessment.relative_velocity_kms >= 0.0,
            "Relative velocity must be non-negative"
        );
        assert!(
            assessment.collision_probability.pc >= 0.0,
            "Pc must be non-negative"
        );
        assert!(
            assessment.collision_probability.pc <= 1.0,
            "Pc must be <= 1.0"
        );
        assert!(assessment.hard_body_radius_m > 0.0, "HBR must be positive");

        // The assessment should have covariance (propagator estimates it from TLE age)
        assert!(assessment.collision_probability.has_covariance);

        // Risk level should be consistent with Pc
        let expected_risk = RiskLevel::from_pc(assessment.collision_probability.pc);
        assert_eq!(assessment.risk_level, expected_risk);
    }

    #[test]
    fn test_screening() {
        use crate::state::DataSource;
        use chrono::Utc;

        let now = Utc::now();

        let objects: Vec<OrbitalState> = (0..5)
            .map(|i| {
                OrbitalState::new(
                    25544 + i,
                    now,
                    StateVector::new(7000.0 + i as f64, 0.0, 0.0, 0.0, 7.5, 0.0),
                    DataSource::SpaceTrack,
                )
            })
            .collect();

        let conjunctions = screen_catalog(&objects[0..1], &objects, 2.0);

        // Should find object 1 (1 km away) but not object 4 (4 km away)
        assert!(conjunctions.iter().any(|(_, s, _)| *s == 25545));
        assert!(!conjunctions.iter().any(|(_, s, _)| *s == 25548));
    }
}
