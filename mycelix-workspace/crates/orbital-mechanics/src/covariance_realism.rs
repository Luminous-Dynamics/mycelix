// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Descriptive 3D position-covariance realism diagnostics.
//!
//! Compares predicted position covariance against later independent residuals.
//! This module does not implement the complete NASA CARA covariance-realism
//! methodology. It emits descriptive NEES and chi-square ellipsoid coverage
//! only: no calibrated p-values, confidence intervals, covariance pass/fail
//! verdict, or claim of future predictive validity.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// One independently sourced predicted-versus-observed position residual.
///
/// Residual and covariance units are kilometres and square kilometres.
/// The covariance is the 3x3 position block, not a 6D state covariance.
/// Prediction and observation epochs must identify the same TAI instant.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PositionResidualSample {
    pub sample_id: String,
    pub object_id: String,
    /// Group of statistically dependent residuals, e.g. one OD solution arc.
    /// Repeated samples from one arc must share this ID.
    pub independent_group_id: String,
    pub prediction_epoch_tai_micros: i64,
    pub observation_epoch_tai_micros: i64,
    pub time_basis_id: String,
    pub frame_realization_id: String,
    pub covariance_profile_id: String,
    pub force_model_revision: String,
    pub covariance_source_id: String,
    pub observation_source_id: String,
    /// SHA-256 digest of the exact ephemeris prediction artifact.
    pub predicted_ephemeris_sha256: String,
    /// SHA-256 digest of the exact covariance artifact.
    pub covariance_artifact_sha256: String,
    /// SHA-256 digest of the exact as-flown / later-observed state artifact.
    pub observed_state_sha256: String,
    /// Observed position minus predicted position, in km.
    pub residual_position_km: [f64; 3],
    /// Predicted position covariance in km².
    pub position_covariance_km2: [[f64; 3]; 3],
}

/// Configuration for a single homogeneous covariance-realism cohort.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct CovarianceRealismConfig {
    pub report_id: String,
    pub cohort_id: String,
    /// A data sufficiency floor only; reaching it does not create an inferential
    /// confidence claim. The caller must choose independent-group semantics.
    pub minimum_independent_groups: usize,
    /// Relative symmetry tolerance. No matrix repair or diagonal jitter occurs.
    pub symmetry_relative_tolerance_ppb: u64,
}

/// A descriptive 3D position covariance report for one homogeneous cohort.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CovarianceRealismReport {
    pub report_id: String,
    pub cohort_id: String,
    pub sample_manifest_sha256: String,
    pub algorithm_profile_id: String,
    pub time_basis_id: String,
    pub frame_realization_id: String,
    pub covariance_profile_id: String,
    pub force_model_revision: String,
    pub covariance_source_id: String,
    pub observation_source_id: String,
    pub sample_count: usize,
    pub independent_group_count: usize,
    pub minimum_independent_groups: usize,
    pub independent_group_floor_met: bool,
    pub mean_position_bias_km: [f64; 3],
    pub mean_position_bias_norm_km: f64,
    /// Mean 3D position NEES. For a correctly specified 3D Gaussian model,
    /// the theoretical expectation is 3; this report does not perform a test.
    pub mean_nees: f64,
    pub median_nees: f64,
    /// Empirical coverage inside the 3D chi-square(3) probability ellipsoid.
    pub coverage_68_27: f64,
    pub coverage_95_45: f64,
    pub coverage_99_73: f64,
    /// Explicitly descriptive; no realism pass/fail result is emitted.
    pub interpretation: CovarianceRealismInterpretation,
}

/// Whether the cohort has enough separately labelled groups for a later
/// cluster-aware inferential stage. This is not a calibration verdict.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum CovarianceRealismInterpretation {
    DescriptiveOnlyInsufficientIndependentGroups {
        observed_groups: usize,
        required_groups: usize,
    },
    DescriptiveOnlyIndependentGroupFloorMet {
        observed_groups: usize,
        floor: usize,
    },
}

/// Invalid provenance, incompatible cohort, or unusable residual/covariance.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum CovarianceRealismError {
    EmptyReportId,
    EmptyCohortId,
    ZeroIndependentGroupFloor,
    ZeroSymmetryTolerance,
    EmptySampleSet,
    TooManySamples { count: usize, limit: usize },
    EmptySampleId,
    DuplicateSampleId { sample_id: String },
    EmptyObjectId { sample_id: String },
    EmptyIndependentGroupId { sample_id: String },
    InvalidTimeBasis { sample_id: String },
    EpochMismatch { sample_id: String },
    MixedTimeBasis { expected: String, actual: String },
    MixedFrameRealization { expected: String, actual: String },
    MixedCovarianceProfile { expected: String, actual: String },
    MixedForceModelRevision { expected: String, actual: String },
    MixedCovarianceSource { expected: String, actual: String },
    MixedObservationSource { expected: String, actual: String },
    MissingProvenance { sample_id: String, field: String },
    InvalidArtifactDigest { sample_id: String, field: String },
    NonFiniteResidual { sample_id: String },
    NonFiniteCovariance { sample_id: String },
    AsymmetricCovariance { sample_id: String },
    NonPositiveDefiniteCovariance { sample_id: String },
    NonFiniteNees { sample_id: String },
    NonFiniteAggregate,
}

const MAX_COVARIANCE_REALISM_SAMPLES: usize = 500_000;

// Chi-square(3) quantiles for central probability ellipsoids. The labels are
// probability content, not coordinate-wise +/- sigma bounds.
const CHI_SQUARE_3_Q_68_27: f64 = 3.526_740_380_261_046_4;
const CHI_SQUARE_3_Q_95_45: f64 = 8.024_881_760_283_781;
const CHI_SQUARE_3_Q_99_73: f64 = 14.156_413_609_332_269;

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn validate_nonempty(
    value: &str,
    sample_id: &str,
    field: &'static str,
) -> Result<(), CovarianceRealismError> {
    if value.trim().is_empty() {
        return Err(CovarianceRealismError::MissingProvenance {
            sample_id: sample_id.to_string(),
            field: field.to_string(),
        });
    }
    Ok(())
}

/// Compute rᵀ P⁻¹ r with a strict Cholesky solve.
///
/// This function rejects non-finite, asymmetric, singular and indefinite
/// covariance matrices. It does not symmetrize, regularize or add jitter.
fn position_nees(
    residual: [f64; 3],
    covariance: [[f64; 3]; 3],
    sample_id: &str,
    symmetry_tolerance: f64,
) -> Result<f64, CovarianceRealismError> {
    if !residual.iter().all(|value| value.is_finite()) {
        return Err(CovarianceRealismError::NonFiniteResidual {
            sample_id: sample_id.to_string(),
        });
    }
    if !covariance.iter().flatten().all(|value| value.is_finite()) {
        return Err(CovarianceRealismError::NonFiniteCovariance {
            sample_id: sample_id.to_string(),
        });
    }

    let scale = covariance
        .iter()
        .flatten()
        .fold(0.0_f64, |current, value| current.max(value.abs()))
        .max(f64::MIN_POSITIVE);
    for row in 0..3 {
        for col in (row + 1)..3 {
            if (covariance[row][col] - covariance[col][row]).abs() > symmetry_tolerance * scale {
                return Err(CovarianceRealismError::AsymmetricCovariance {
                    sample_id: sample_id.to_string(),
                });
            }
        }
    }

    // Cholesky factorisation P = L Lᵀ. Strictly positive pivots are
    // required because the NEES calculation needs an invertible covariance.
    let p00 = covariance[0][0];
    if p00 <= 0.0 {
        return Err(CovarianceRealismError::NonPositiveDefiniteCovariance {
            sample_id: sample_id.to_string(),
        });
    }
    let l00 = p00.sqrt();
    let l10 = covariance[1][0] / l00;
    let l20 = covariance[2][0] / l00;
    let pivot11 = covariance[1][1] - l10 * l10;
    if !pivot11.is_finite() || pivot11 <= 0.0 {
        return Err(CovarianceRealismError::NonPositiveDefiniteCovariance {
            sample_id: sample_id.to_string(),
        });
    }
    let l11 = pivot11.sqrt();
    let l21 = (covariance[2][1] - l20 * l10) / l11;
    let pivot22 = covariance[2][2] - l20 * l20 - l21 * l21;
    if !pivot22.is_finite() || pivot22 <= 0.0 {
        return Err(CovarianceRealismError::NonPositiveDefiniteCovariance {
            sample_id: sample_id.to_string(),
        });
    }
    let l22 = pivot22.sqrt();

    // Solve L y = r, then rᵀ P⁻¹ r = yᵀ y.
    let y0 = residual[0] / l00;
    let y1 = (residual[1] - l10 * y0) / l11;
    let y2 = (residual[2] - l20 * y0 - l21 * y1) / l22;
    let nees = y0 * y0 + y1 * y1 + y2 * y2;
    if !nees.is_finite() || nees < 0.0 {
        return Err(CovarianceRealismError::NonFiniteNees {
            sample_id: sample_id.to_string(),
        });
    }
    Ok(nees)
}

fn hash_string(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn hash_f64(hasher: &mut Sha256, value: f64) {
    hasher.update(value.to_bits().to_be_bytes());
}

/// Compute the canonical SHA-256 manifest digest for the exact sample values.
///
/// The encoding is versioned, length-prefixed for strings, uses big-endian
/// integer/f64 bit patterns, and sorts by sample ID. It covers provenance,
/// artifact digests, both epochs, residual components and all covariance
/// matrix entries. The assessment function computes this digest itself.
pub fn position_residual_manifest_sha256(samples: &[PositionResidualSample]) -> String {
    let mut ordered: Vec<&PositionResidualSample> = samples.iter().collect();
    ordered.sort_by(|left, right| left.sample_id.cmp(&right.sample_id));
    let mut hasher = Sha256::new();
    hasher.update(b"mycelix-position-residual-manifest-v1");
    hasher.update((ordered.len() as u64).to_be_bytes());

    for sample in ordered {
        for value in [
            sample.sample_id.as_str(),
            sample.object_id.as_str(),
            sample.independent_group_id.as_str(),
            sample.time_basis_id.as_str(),
            sample.frame_realization_id.as_str(),
            sample.covariance_profile_id.as_str(),
            sample.force_model_revision.as_str(),
            sample.covariance_source_id.as_str(),
            sample.observation_source_id.as_str(),
            sample.predicted_ephemeris_sha256.as_str(),
            sample.covariance_artifact_sha256.as_str(),
            sample.observed_state_sha256.as_str(),
        ] {
            hash_string(&mut hasher, value);
        }
        hasher.update(sample.prediction_epoch_tai_micros.to_be_bytes());
        hasher.update(sample.observation_epoch_tai_micros.to_be_bytes());
        for value in sample.residual_position_km {
            hash_f64(&mut hasher, value);
        }
        for row in sample.position_covariance_km2 {
            for value in row {
                hash_f64(&mut hasher, value);
            }
        }
    }

    format!("{:x}", hasher.finalize())
}

/// Evaluate observed 3D position residuals against their predicted covariance.
///
/// The report is descriptive and cohort-specific. It does not infer statistical
/// independence from the supplied group labels, calculate p-values, construct
/// confidence intervals, or emit a covariance-realism pass/fail verdict.
///
/// The residual and covariance units must be km and km², respectively. The
/// prediction and as-flown observation epochs must already be aligned in TAI.
/// Each report accepts only one time basis, frame realization, covariance
/// profile, force-model revision, covariance source and observation source.
pub fn assess_position_covariance_realism(
    config: &CovarianceRealismConfig,
    samples: &[PositionResidualSample],
) -> Result<CovarianceRealismReport, CovarianceRealismError> {
    if config.report_id.trim().is_empty() {
        return Err(CovarianceRealismError::EmptyReportId);
    }
    if config.cohort_id.trim().is_empty() {
        return Err(CovarianceRealismError::EmptyCohortId);
    }
    if config.minimum_independent_groups == 0 {
        return Err(CovarianceRealismError::ZeroIndependentGroupFloor);
    }
    if config.symmetry_relative_tolerance_ppb == 0 {
        return Err(CovarianceRealismError::ZeroSymmetryTolerance);
    }
    if samples.is_empty() {
        return Err(CovarianceRealismError::EmptySampleSet);
    }
    if samples.len() > MAX_COVARIANCE_REALISM_SAMPLES {
        return Err(CovarianceRealismError::TooManySamples {
            count: samples.len(),
            limit: MAX_COVARIANCE_REALISM_SAMPLES,
        });
    }

    let first = &samples[0];
    let cohort_time_basis = first.time_basis_id.clone();
    let cohort_frame = first.frame_realization_id.clone();
    let cohort_covariance_profile = first.covariance_profile_id.clone();
    let cohort_force_model = first.force_model_revision.clone();
    let cohort_covariance_source = first.covariance_source_id.clone();
    let cohort_observation_source = first.observation_source_id.clone();
    for (field, value) in [
        ("time_basis_id", cohort_time_basis.as_str()),
        ("frame_realization_id", cohort_frame.as_str()),
        ("covariance_profile_id", cohort_covariance_profile.as_str()),
        ("force_model_revision", cohort_force_model.as_str()),
        ("covariance_source_id", cohort_covariance_source.as_str()),
        ("observation_source_id", cohort_observation_source.as_str()),
    ] {
        validate_nonempty(value, &first.sample_id, field)?;
    }

    let symmetry_tolerance = config.symmetry_relative_tolerance_ppb as f64 / 1_000_000_000.0;
    let mut sample_ids = BTreeSet::new();
    let mut independent_groups = BTreeSet::new();
    let mut ordered_samples: Vec<&PositionResidualSample> = samples.iter().collect();
    ordered_samples.sort_by(|left, right| left.sample_id.cmp(&right.sample_id));
    let mut nees_values = Vec::with_capacity(samples.len());
    let mut residual_sum = [0.0_f64; 3];
    let mut coverage_68_27 = 0usize;
    let mut coverage_95_45 = 0usize;
    let mut coverage_99_73 = 0usize;

    for sample in ordered_samples {
        if sample.sample_id.trim().is_empty() {
            return Err(CovarianceRealismError::EmptySampleId);
        }
        if !sample_ids.insert(sample.sample_id.as_str()) {
            return Err(CovarianceRealismError::DuplicateSampleId {
                sample_id: sample.sample_id.clone(),
            });
        }
        validate_nonempty(&sample.object_id, &sample.sample_id, "object_id")?;
        validate_nonempty(
            &sample.independent_group_id,
            &sample.sample_id,
            "independent_group_id",
        )?;
        if sample.prediction_epoch_tai_micros != sample.observation_epoch_tai_micros {
            return Err(CovarianceRealismError::EpochMismatch {
                sample_id: sample.sample_id.clone(),
            });
        }

        for (field, expected, actual) in [
            ("time_basis_id", &cohort_time_basis, &sample.time_basis_id),
            (
                "frame_realization_id",
                &cohort_frame,
                &sample.frame_realization_id,
            ),
            (
                "covariance_profile_id",
                &cohort_covariance_profile,
                &sample.covariance_profile_id,
            ),
            (
                "force_model_revision",
                &cohort_force_model,
                &sample.force_model_revision,
            ),
            (
                "covariance_source_id",
                &cohort_covariance_source,
                &sample.covariance_source_id,
            ),
            (
                "observation_source_id",
                &cohort_observation_source,
                &sample.observation_source_id,
            ),
        ] {
            if actual.trim().is_empty() {
                return Err(CovarianceRealismError::MissingProvenance {
                    sample_id: sample.sample_id.clone(),
                    field: field.to_string(),
                });
            }
            if expected != actual {
                return Err(match field {
                    "time_basis_id" => CovarianceRealismError::MixedTimeBasis {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                    "frame_realization_id" => CovarianceRealismError::MixedFrameRealization {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                    "covariance_profile_id" => CovarianceRealismError::MixedCovarianceProfile {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                    "force_model_revision" => CovarianceRealismError::MixedForceModelRevision {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                    "covariance_source_id" => CovarianceRealismError::MixedCovarianceSource {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                    _ => CovarianceRealismError::MixedObservationSource {
                        expected: expected.clone(),
                        actual: actual.clone(),
                    },
                });
            }
        }

        for (field, digest) in [
            (
                "predicted_ephemeris_sha256",
                &sample.predicted_ephemeris_sha256,
            ),
            (
                "covariance_artifact_sha256",
                &sample.covariance_artifact_sha256,
            ),
            ("observed_state_sha256", &sample.observed_state_sha256),
        ] {
            if !is_sha256_hex(digest) {
                return Err(CovarianceRealismError::InvalidArtifactDigest {
                    sample_id: sample.sample_id.clone(),
                    field: field.to_string(),
                });
            }
        }

        independent_groups.insert(sample.independent_group_id.as_str());
        for axis in 0..3 {
            residual_sum[axis] += sample.residual_position_km[axis];
            if !residual_sum[axis].is_finite() {
                return Err(CovarianceRealismError::NonFiniteAggregate);
            }
        }
        let nees = position_nees(
            sample.residual_position_km,
            sample.position_covariance_km2,
            &sample.sample_id,
            symmetry_tolerance,
        )?;
        if nees <= CHI_SQUARE_3_Q_68_27 {
            coverage_68_27 += 1;
        }
        if nees <= CHI_SQUARE_3_Q_95_45 {
            coverage_95_45 += 1;
        }
        if nees <= CHI_SQUARE_3_Q_99_73 {
            coverage_99_73 += 1;
        }
        nees_values.push(nees);
    }

    let sample_count = samples.len() as f64;
    let mean_position_bias_km = [
        residual_sum[0] / sample_count,
        residual_sum[1] / sample_count,
        residual_sum[2] / sample_count,
    ];
    let mean_position_bias_norm_km = mean_position_bias_km[0]
        .hypot(mean_position_bias_km[1])
        .hypot(mean_position_bias_km[2]);
    let nees_sum = nees_values.iter().sum::<f64>();
    if !nees_sum.is_finite() {
        return Err(CovarianceRealismError::NonFiniteAggregate);
    }
    let mean_nees = nees_sum / sample_count;
    if !mean_nees.is_finite() || !mean_position_bias_norm_km.is_finite() {
        return Err(CovarianceRealismError::NonFiniteAggregate);
    }
    nees_values.sort_by(f64::total_cmp);
    let median_nees = if nees_values.len() % 2 == 0 {
        let upper = nees_values.len() / 2;
        (nees_values[upper - 1] + nees_values[upper]) / 2.0
    } else {
        nees_values[nees_values.len() / 2]
    };
    let independent_group_count = independent_groups.len();
    let interpretation = if independent_group_count < config.minimum_independent_groups {
        CovarianceRealismInterpretation::DescriptiveOnlyInsufficientIndependentGroups {
            observed_groups: independent_group_count,
            required_groups: config.minimum_independent_groups,
        }
    } else {
        CovarianceRealismInterpretation::DescriptiveOnlyIndependentGroupFloorMet {
            observed_groups: independent_group_count,
            floor: config.minimum_independent_groups,
        }
    };

    Ok(CovarianceRealismReport {
        report_id: config.report_id.clone(),
        cohort_id: config.cohort_id.clone(),
        sample_manifest_sha256: position_residual_manifest_sha256(samples),
        algorithm_profile_id: "mycelix-position-covariance-nees-v0.1".to_string(),
        time_basis_id: cohort_time_basis,
        frame_realization_id: cohort_frame,
        covariance_profile_id: cohort_covariance_profile,
        force_model_revision: cohort_force_model,
        covariance_source_id: cohort_covariance_source,
        observation_source_id: cohort_observation_source,
        sample_count: samples.len(),
        independent_group_count,
        minimum_independent_groups: config.minimum_independent_groups,
        independent_group_floor_met: independent_group_count >= config.minimum_independent_groups,
        mean_position_bias_km,
        mean_position_bias_norm_km,
        mean_nees,
        median_nees,
        coverage_68_27: coverage_68_27 as f64 / sample_count,
        coverage_95_45: coverage_95_45 as f64 / sample_count,
        coverage_99_73: coverage_99_73 as f64 / sample_count,
        interpretation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DIGEST_A: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DIGEST_B: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const DIGEST_C: &str = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";

    fn sample(id: &str, group: &str, residual: [f64; 3]) -> PositionResidualSample {
        PositionResidualSample {
            sample_id: id.to_string(),
            object_id: "25544".to_string(),
            independent_group_id: group.to_string(),
            prediction_epoch_tai_micros: 1_000,
            observation_epoch_tai_micros: 1_000,
            time_basis_id: "TAI-1970-unix-micros-v1".to_string(),
            frame_realization_id: "TEME-IGRF-2026-v1".to_string(),
            covariance_profile_id: "cov-profile-2026-01".to_string(),
            force_model_revision: "force-model-1".to_string(),
            covariance_source_id: "operator-covariance".to_string(),
            observation_source_id: "independent-as-flown-ephemeris".to_string(),
            predicted_ephemeris_sha256: DIGEST_A.to_string(),
            covariance_artifact_sha256: DIGEST_B.to_string(),
            observed_state_sha256: DIGEST_C.to_string(),
            residual_position_km: residual,
            position_covariance_km2: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }

    fn config(min_groups: usize) -> CovarianceRealismConfig {
        CovarianceRealismConfig {
            report_id: "cov-realism-report-1".to_string(),
            cohort_id: "leo-operator-cov-1".to_string(),
            minimum_independent_groups: min_groups,
            symmetry_relative_tolerance_ppb: 100,
        }
    }

    #[test]
    fn identity_covariance_known_answer_nees_is_squared_euclidean_norm() {
        let sample = sample("s1", "arc-1", [1.0, 2.0, 2.0]);
        let report = assess_position_covariance_realism(&config(1), &[sample]).unwrap();
        assert!((report.mean_nees - 9.0).abs() < 1e-12);
        assert!((report.median_nees - 9.0).abs() < 1e-12);
    }

    #[test]
    fn correlated_covariance_cholesky_known_answer_is_reproduced() {
        let mut sample = sample("s1", "arc-1", [2.0, 1.0, 2.0]);
        sample.position_covariance_km2 = [[4.0, 1.0, 0.0], [1.0, 3.0, 0.0], [0.0, 0.0, 2.0]];
        let report = assess_position_covariance_realism(&config(1), &[sample]).unwrap();
        assert!((report.mean_nees - (34.0 / 11.0)).abs() < 1e-12);
    }

    #[test]
    fn empirical_coverage_uses_three_dimensional_chi_square_ellipsoids() {
        let samples = [
            sample("inside-1", "arc-1", [0.0, 0.0, 0.0]),
            sample("inside-2", "arc-2", [1.0, 0.0, 0.0]),
            sample("outside-one", "arc-3", [2.0, 0.0, 0.0]),
            sample("outside-two", "arc-4", [3.0, 0.0, 0.0]),
        ];
        let report = assess_position_covariance_realism(&config(4), &samples).unwrap();
        assert_eq!(report.coverage_68_27, 0.5);
        assert_eq!(report.coverage_95_45, 0.75);
        assert_eq!(report.coverage_99_73, 1.0);
        assert!(report.independent_group_floor_met);
    }

    #[test]
    fn independent_group_floor_is_not_confused_with_sample_count() {
        let samples = [
            sample("s1", "one-arc", [0.1, 0.0, 0.0]),
            sample("s2", "one-arc", [0.2, 0.0, 0.0]),
            sample("s3", "one-arc", [0.3, 0.0, 0.0]),
        ];
        let report = assess_position_covariance_realism(&config(3), &samples).unwrap();
        assert_eq!(report.sample_count, 3);
        assert_eq!(report.independent_group_count, 1);
        assert!(!report.independent_group_floor_met);
        assert_eq!(
            report.interpretation,
            CovarianceRealismInterpretation::DescriptiveOnlyInsufficientIndependentGroups {
                observed_groups: 1,
                required_groups: 3,
            }
        );
    }

    #[test]
    fn results_are_deterministic_under_sample_permutation_and_bind_the_manifest() {
        let first = [
            sample("z-last", "arc-2", [0.2, 0.0, 0.0]),
            sample("a-first", "arc-1", [0.1, 0.0, 0.0]),
        ];
        let second = [first[1].clone(), first[0].clone()];
        let report_a = assess_position_covariance_realism(&config(2), &first).unwrap();
        let report_b = assess_position_covariance_realism(&config(2), &second).unwrap();
        assert_eq!(report_a, report_b);
        assert_eq!(
            report_a.sample_manifest_sha256,
            position_residual_manifest_sha256(&first)
        );
        assert_eq!(
            report_a.algorithm_profile_id,
            "mycelix-position-covariance-nees-v0.1"
        );
    }

    #[test]
    fn sample_manifest_changes_when_residual_or_covariance_changes() {
        let original = [sample("s1", "arc-1", [0.0, 0.0, 0.0])];
        let original_digest = position_residual_manifest_sha256(&original);

        let mut altered = original.clone();
        altered[0].residual_position_km[0] = 0.125;
        assert_ne!(original_digest, position_residual_manifest_sha256(&altered));

        altered = original.clone();
        altered[0].position_covariance_km2[0][0] = 1.25;
        assert_ne!(original_digest, position_residual_manifest_sha256(&altered));
    }

    #[test]
    fn canonical_manifest_digest_is_order_independent_and_binds_all_values() {
        let first = [
            sample("z-last", "arc-2", [0.2, 0.0, 0.0]),
            sample("a-first", "arc-1", [0.1, 0.0, 0.0]),
        ];
        let second = [first[1].clone(), first[0].clone()];
        assert_eq!(
            position_residual_manifest_sha256(&first),
            position_residual_manifest_sha256(&second)
        );

        let mut altered = first.clone();
        altered[0].residual_position_km[0] += 0.01;
        assert_ne!(
            position_residual_manifest_sha256(&first),
            position_residual_manifest_sha256(&altered)
        );
    }

    #[test]
    fn profile_frame_and_force_model_must_be_homogeneous() {
        let first = sample("s1", "arc-1", [0.1, 0.0, 0.0]);
        let mut second = sample("s2", "arc-2", [0.2, 0.0, 0.0]);
        second.covariance_profile_id = "different-profile".to_string();
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[first, second]),
            Err(CovarianceRealismError::MixedCovarianceProfile {
                expected: "cov-profile-2026-01".to_string(),
                actual: "different-profile".to_string(),
            })
        );
    }

    #[test]
    fn asymmetric_and_singular_covariance_are_rejected_without_repair() {
        let mut asymmetric = sample("asymmetric", "arc-1", [0.0; 3]);
        asymmetric.position_covariance_km2[0][1] = 0.5;
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[asymmetric]),
            Err(CovarianceRealismError::AsymmetricCovariance {
                sample_id: "asymmetric".to_string(),
            })
        );

        let mut singular = sample("singular", "arc-1", [0.0; 3]);
        singular.position_covariance_km2[2][2] = 0.0;
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[singular]),
            Err(CovarianceRealismError::NonPositiveDefiniteCovariance {
                sample_id: "singular".to_string(),
            })
        );
    }

    #[test]
    fn negative_variance_is_rejected() {
        let mut sample = sample("negative", "arc-1", [0.0; 3]);
        sample.position_covariance_km2[0][0] = -1.0;
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[sample]),
            Err(CovarianceRealismError::NonPositiveDefiniteCovariance {
                sample_id: "negative".to_string(),
            })
        );
    }

    #[test]
    fn mixed_epoch_and_time_basis_are_rejected() {
        let mut epoch = sample("epoch", "arc-1", [0.0; 3]);
        epoch.observation_epoch_tai_micros = 1_001;
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[epoch]),
            Err(CovarianceRealismError::EpochMismatch {
                sample_id: "epoch".to_string(),
            })
        );

        let mut time_basis = sample("time", "arc-1", [0.0; 3]);
        time_basis.time_basis_id = "UTC-unconverted".to_string();
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[time_basis]),
            Err(CovarianceRealismError::MixedTimeBasis {
                expected: "TAI-1970-unix-micros-v1".to_string(),
                actual: "UTC-unconverted".to_string(),
            })
        );
    }

    #[test]
    fn missing_or_invalid_artifact_provenance_is_rejected() {
        let mut missing = sample("missing", "arc-1", [0.0; 3]);
        missing.observation_source_id.clear();
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[missing]),
            Err(CovarianceRealismError::MissingProvenance {
                sample_id: "missing".to_string(),
                field: "observation_source_id".to_string(),
            })
        );

        let mut bad_digest = sample("digest", "arc-1", [0.0; 3]);
        bad_digest.observed_state_sha256 = "not-a-digest".to_string();
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[bad_digest]),
            Err(CovarianceRealismError::InvalidArtifactDigest {
                sample_id: "digest".to_string(),
                field: "observed_state_sha256".to_string(),
            })
        );
    }

    #[test]
    fn non_finite_residuals_and_covariance_are_rejected() {
        let mut residual = sample("nan", "arc-1", [f64::NAN, 0.0, 0.0]);
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[residual.clone()]),
            Err(CovarianceRealismError::NonFiniteResidual {
                sample_id: "nan".to_string(),
            })
        );

        residual.residual_position_km = [0.0; 3];
        residual.position_covariance_km2[0][0] = f64::INFINITY;
        assert_eq!(
            assess_position_covariance_realism(&config(1), &[residual]),
            Err(CovarianceRealismError::NonFiniteCovariance {
                sample_id: "nan".to_string(),
            })
        );
    }
}
