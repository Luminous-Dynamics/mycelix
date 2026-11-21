#![forbid(unsafe_code)]

//! Dependence-aware all-pairs collision estimation from independently sampled
//! primary and secondary trajectory ensembles.
//!
//! Given IID primary draws X_1..X_m and IID secondary draws Y_1..Y_n, with the
//! two ensembles mutually independent, and a fixed bounded collision kernel
//! h(X,Y) in {0,1}, the all-pairs estimator
//!
//!   U = (1/(m*n)) sum_{i,j} h(X_i,Y_j)
//!
//! is unbiased for E[h(X,Y)]. Its m*n pair outcomes are dependent because
//! pairs share primary or secondary trajectories. They must NOT be treated as
//! m*n IID Bernoulli trials.
//!
//! Changing one primary draw changes U by at most 1/m; changing one secondary
//! draw changes U by at most 1/n. McDiarmid's inequality therefore gives the
//! one-sided distribution-free upper bound
//!
//!   U + sqrt(0.5 * (1/m + 1/n) * ln(1/delta)).
//!
//! The guarantee assumes IID primary samples, IID secondary samples,
//! independence between the two ensembles, a fixed event kernel and correct
//! high-fidelity event labels. Shared environmental nuisance variables or
//! correlated primary-secondary errors require a separate clustered/shared-
//! latent estimator and are rejected by this E0 interface.
//!
//! This crate does not sample random numbers or propagate orbits. Callers
//! supply independently generated trajectory IDs and the high-fidelity
//! continuous-time collision matrix; the outer evidence package retains and
//! hashes those trajectory and matrix artifacts.

use std::collections::BTreeSet;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PairReuseEvidence {
    pub primary_sample_set_id: String,
    pub secondary_sample_set_id: String,
    pub primary_trajectory_ids: Vec<String>,
    pub secondary_trajectory_ids: Vec<String>,
    pub primary_sample_order_sha256: String,
    pub secondary_sample_order_sha256: String,
    pub collision_matrix_sha256: String,
    pub sampling_design_id: String,
    pub independence_evidence_ref: String,
    pub primary_draws_are_iid: bool,
    pub secondary_draws_are_iid: bool,
    pub primary_secondary_ensembles_are_independent: bool,
    pub trajectories_propagated_once: bool,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub high_fidelity_oracle_is_continuous_time: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PairReuseReport {
    pub method: &'static str,
    pub primary_sample_set_id: String,
    pub secondary_sample_set_id: String,
    pub sampling_design_id: String,
    pub independence_evidence_ref: String,
    pub primary_sample_order_sha256: String,
    pub secondary_sample_order_sha256: String,
    pub collision_matrix_sha256: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub primary_trajectory_count: usize,
    pub secondary_trajectory_count: usize,
    pub propagated_trajectory_count: usize,
    pub pair_comparison_count: usize,
    pub pair_comparisons_per_propagated_trajectory: f64,
    pub dependent_pair_outcomes: bool,
    pub collision_pair_count: usize,
    pub estimate: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub mcdiarmid_radius: f64,
    pub upper_confidence_bound: f64,
    /// 1 / (1/m + 1/n), a concentration scale, not a claim that mn pairs are
    /// independent or that this is an exact effective sample size.
    pub independent_sample_concentration_scale: f64,
    pub row_mean_sample_variance: f64,
    pub column_mean_sample_variance: f64,
    pub inference_scope: &'static str,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PairReuseError {
    EmptyPrimaryEnsemble,
    EmptySecondaryEnsemble,
    EmptyCollisionMatrix,
    RaggedCollisionMatrix {
        row: usize,
        expected_columns: usize,
        actual_columns: usize,
    },
    InvalidIdentifier(&'static str),
    InvalidSha256(&'static str),
    DuplicateTrajectoryId(String),
    PrimarySecondarySampleSetIdentityCollision,
    MissingIndependenceEvidence,
    NonIidPrimaryEnsemble,
    NonIidSecondaryEnsemble,
    CrossObjectDependenceUnsupported,
    TrajectoryReuseNotAttested,
    HighFidelityContinuousTimeOracleRequired,
    MissingEventDefinitionOrHorizon,
    InvalidDelta(f64),
    DimensionOverflow,
}

impl fmt::Display for PairReuseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyPrimaryEnsemble => {
                write!(f, "primary trajectory ensemble must not be empty")
            }
            Self::EmptySecondaryEnsemble => {
                write!(f, "secondary trajectory ensemble must not be empty")
            }
            Self::EmptyCollisionMatrix => write!(f, "collision outcome matrix must not be empty"),
            Self::RaggedCollisionMatrix {
                row,
                expected_columns,
                actual_columns,
            } => write!(
                f,
                "collision matrix row {row} has {actual_columns} columns, expected {expected_columns}"
            ),
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::InvalidSha256(field) => write!(
                f,
                "{field} must be a 64-character lowercase hexadecimal SHA-256 digest"
            ),
            Self::DuplicateTrajectoryId(value) => {
                write!(f, "duplicate trajectory sample ID {value}")
            }
            Self::PrimarySecondarySampleSetIdentityCollision => {
                write!(f, "primary and secondary sample-set IDs must be distinct")
            }
            Self::MissingIndependenceEvidence => write!(
                f,
                "an evidence reference for the independent-ensemble sampling design is required"
            ),
            Self::NonIidPrimaryEnsemble => write!(
                f,
                "primary trajectory draws must be IID under this E0 estimator"
            ),
            Self::NonIidSecondaryEnsemble => write!(
                f,
                "secondary trajectory draws must be IID under this E0 estimator"
            ),
            Self::CrossObjectDependenceUnsupported => write!(
                f,
                "correlated primary-secondary ensembles require a shared-latent or clustered estimator"
            ),
            Self::TrajectoryReuseNotAttested => write!(
                f,
                "each retained trajectory must be propagated once and reused only in the pairwise event matrix"
            ),
            Self::HighFidelityContinuousTimeOracleRequired => write!(
                f,
                "pair outcomes must use the declared high-fidelity continuous-time event oracle"
            ),
            Self::MissingEventDefinitionOrHorizon => write!(
                f,
                "fixed event definition and risk horizon IDs are required"
            ),
            Self::InvalidDelta(value) => {
                write!(f, "delta must be finite and in (0,1); got {value}")
            }
            Self::DimensionOverflow => {
                write!(f, "trajectory or pair-comparison count overflowed usize")
            }
        }
    }
}

impl std::error::Error for PairReuseError {}

/// Calculate the all-pairs U-statistic and a one-sided McDiarmid upper bound.
///
/// The matrix must have m rows (primary trajectories) and n columns
/// (secondary trajectories). The pair outcomes share rows/columns and are
/// intentionally treated as dependent. The bound is valid for independent
/// primary and secondary IID ensembles and a fixed kernel h in [0,1].
pub fn estimate_pair_reuse_collision_probability(
    collision_matrix: &[Vec<bool>],
    delta: f64,
    evidence: &PairReuseEvidence,
) -> Result<PairReuseReport, PairReuseError> {
    validate_delta(delta)?;
    validate_evidence(collision_matrix, evidence)?;

    let m = collision_matrix.len();
    let n = collision_matrix[0].len();
    let pair_count = m.checked_mul(n).ok_or(PairReuseError::DimensionOverflow)?;
    let propagated_count = m.checked_add(n).ok_or(PairReuseError::DimensionOverflow)?;
    let collision_count = collision_matrix
        .iter()
        .flat_map(|row| row.iter())
        .filter(|hit| **hit)
        .count();
    let estimate = collision_count as f64 / pair_count as f64;

    // Bounded differences: each primary draw changes the average by <= 1/m;
    // each secondary draw changes it by <= 1/n.
    let bounded_difference_sum = 1.0 / m as f64 + 1.0 / n as f64;
    let radius = (0.5 * bounded_difference_sum * (1.0 / delta).ln()).sqrt();
    let upper = (estimate + radius).min(1.0);

    let row_means: Vec<f64> = collision_matrix
        .iter()
        .map(|row| row.iter().filter(|hit| **hit).count() as f64 / n as f64)
        .collect();
    let column_means: Vec<f64> = (0..n)
        .map(|column| collision_matrix.iter().filter(|row| row[column]).count() as f64 / m as f64)
        .collect();

    Ok(PairReuseReport {
        method: "two_sample_all_pairs_u_statistic_mcdiarmid_upper_bound",
        primary_sample_set_id: evidence.primary_sample_set_id.clone(),
        secondary_sample_set_id: evidence.secondary_sample_set_id.clone(),
        sampling_design_id: evidence.sampling_design_id.clone(),
        independence_evidence_ref: evidence.independence_evidence_ref.clone(),
        primary_sample_order_sha256: evidence.primary_sample_order_sha256.clone(),
        secondary_sample_order_sha256: evidence.secondary_sample_order_sha256.clone(),
        collision_matrix_sha256: evidence.collision_matrix_sha256.clone(),
        event_definition_id: evidence.event_definition_id.clone(),
        event_horizon_id: evidence.event_horizon_id.clone(),
        high_fidelity_event_oracle_id: evidence.high_fidelity_event_oracle_id.clone(),
        high_fidelity_event_oracle_revision: evidence.high_fidelity_event_oracle_revision.clone(),
        primary_trajectory_count: m,
        secondary_trajectory_count: n,
        propagated_trajectory_count: propagated_count,
        pair_comparison_count: pair_count,
        pair_comparisons_per_propagated_trajectory: pair_count as f64 / propagated_count as f64,
        dependent_pair_outcomes: true,
        collision_pair_count: collision_count,
        estimate,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        mcdiarmid_radius: radius,
        upper_confidence_bound: upper,
        independent_sample_concentration_scale: 1.0 / bounded_difference_sum,
        row_mean_sample_variance: sample_variance(&row_means),
        column_mean_sample_variance: sample_variance(&column_means),
        inference_scope: "one_sided_distribution_free_bound_for_fixed_kernel_and_mutually_independent_iid_primary_secondary_ensembles",
        claim_ceiling: "dependence_aware_pair_reuse_statistical_bound_only_not_orbit_truth_or_operational_safety",
    })
}

fn validate_evidence(
    matrix: &[Vec<bool>],
    evidence: &PairReuseEvidence,
) -> Result<(), PairReuseError> {
    if matrix.is_empty() {
        return Err(PairReuseError::EmptyCollisionMatrix);
    }
    if evidence.primary_trajectory_ids.is_empty() {
        return Err(PairReuseError::EmptyPrimaryEnsemble);
    }
    if evidence.secondary_trajectory_ids.is_empty() {
        return Err(PairReuseError::EmptySecondaryEnsemble);
    }
    let m = evidence.primary_trajectory_ids.len();
    let n = evidence.secondary_trajectory_ids.len();
    if matrix.len() != m {
        return Err(PairReuseError::RaggedCollisionMatrix {
            row: matrix.len(),
            expected_columns: n,
            actual_columns: 0,
        });
    }
    for (index, row) in matrix.iter().enumerate() {
        if row.len() != n {
            return Err(PairReuseError::RaggedCollisionMatrix {
                row: index,
                expected_columns: n,
                actual_columns: row.len(),
            });
        }
    }
    if evidence.primary_sample_set_id == evidence.secondary_sample_set_id {
        return Err(PairReuseError::PrimarySecondarySampleSetIdentityCollision);
    }
    for (field, value) in [
        ("primary_sample_set_id", &evidence.primary_sample_set_id),
        ("secondary_sample_set_id", &evidence.secondary_sample_set_id),
        ("sampling_design_id", &evidence.sampling_design_id),
        ("event_definition_id", &evidence.event_definition_id),
        ("event_horizon_id", &evidence.event_horizon_id),
        (
            "high_fidelity_event_oracle_id",
            &evidence.high_fidelity_event_oracle_id,
        ),
        (
            "high_fidelity_event_oracle_revision",
            &evidence.high_fidelity_event_oracle_revision,
        ),
    ] {
        if !valid_identifier(value) {
            return Err(PairReuseError::InvalidIdentifier(field));
        }
    }
    if evidence.independence_evidence_ref.trim().is_empty() {
        return Err(PairReuseError::MissingIndependenceEvidence);
    }
    if !valid_sha256(&evidence.primary_sample_order_sha256) {
        return Err(PairReuseError::InvalidSha256("primary_sample_order_sha256"));
    }
    if !valid_sha256(&evidence.secondary_sample_order_sha256) {
        return Err(PairReuseError::InvalidSha256(
            "secondary_sample_order_sha256",
        ));
    }
    if !valid_sha256(&evidence.collision_matrix_sha256) {
        return Err(PairReuseError::InvalidSha256("collision_matrix_sha256"));
    }
    if !evidence.primary_draws_are_iid {
        return Err(PairReuseError::NonIidPrimaryEnsemble);
    }
    if !evidence.secondary_draws_are_iid {
        return Err(PairReuseError::NonIidSecondaryEnsemble);
    }
    if !evidence.primary_secondary_ensembles_are_independent {
        return Err(PairReuseError::CrossObjectDependenceUnsupported);
    }
    if !evidence.trajectories_propagated_once {
        return Err(PairReuseError::TrajectoryReuseNotAttested);
    }
    if !evidence.high_fidelity_oracle_is_continuous_time {
        return Err(PairReuseError::HighFidelityContinuousTimeOracleRequired);
    }

    let mut seen = BTreeSet::new();
    for id in &evidence.primary_trajectory_ids {
        if !valid_identifier(id) {
            return Err(PairReuseError::InvalidIdentifier("primary_trajectory_id"));
        }
        if !seen.insert(id.as_str()) {
            return Err(PairReuseError::DuplicateTrajectoryId(id.clone()));
        }
    }
    for id in &evidence.secondary_trajectory_ids {
        if !valid_identifier(id) {
            return Err(PairReuseError::InvalidIdentifier("secondary_trajectory_id"));
        }
        if !seen.insert(id.as_str()) {
            return Err(PairReuseError::DuplicateTrajectoryId(id.clone()));
        }
    }
    Ok(())
}

fn validate_delta(delta: f64) -> Result<(), PairReuseError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(PairReuseError::InvalidDelta(delta));
    }
    Ok(())
}

fn valid_identifier(value: &str) -> bool {
    !value.trim().is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"._:/-".contains(&byte))
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn sample_variance(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / (values.len() - 1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sha(ch: char) -> String {
        std::iter::repeat(ch).take(64).collect()
    }

    fn evidence(primary_count: usize, secondary_count: usize) -> PairReuseEvidence {
        PairReuseEvidence {
            primary_sample_set_id: "primary-ensemble-v1".into(),
            secondary_sample_set_id: "secondary-ensemble-v1".into(),
            primary_trajectory_ids: (0..primary_count)
                .map(|i| format!("primary-trajectory-{i}"))
                .collect(),
            secondary_trajectory_ids: (0..secondary_count)
                .map(|i| format!("secondary-trajectory-{i}"))
                .collect(),
            primary_sample_order_sha256: sha('a'),
            secondary_sample_order_sha256: sha('b'),
            collision_matrix_sha256: sha('c'),
            sampling_design_id: "iid-mutually-independent-ensembles-v1".into(),
            independence_evidence_ref: "fixture://sampling-design/independent-ensembles-v1".into(),
            primary_draws_are_iid: true,
            secondary_draws_are_iid: true,
            primary_secondary_ensembles_are_independent: true,
            trajectories_propagated_once: true,
            event_definition_id: "first-hard-body-contact-v1".into(),
            event_horizon_id: "seven-day-risk-window-v1".into(),
            high_fidelity_event_oracle_id: "hf-continuous-time-event-oracle".into(),
            high_fidelity_event_oracle_revision: "hf-oracle-revision-v1".into(),
            high_fidelity_oracle_is_continuous_time: true,
        }
    }

    #[test]
    fn all_pairs_estimator_reports_dependent_pair_count_and_propagation_reuse() {
        let matrix = vec![
            vec![false, false, false, false, false],
            vec![false, false, false, false, false],
            vec![false, false, false, false, false],
        ];
        let report =
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &evidence(3, 5)).unwrap();
        assert_eq!(report.estimate, 0.0);
        assert_eq!(report.pair_comparison_count, 15);
        assert_eq!(report.propagated_trajectory_count, 8);
        assert!((report.pair_comparisons_per_propagated_trajectory - 15.0 / 8.0).abs() < 1e-12);
        assert!(report.dependent_pair_outcomes);
        assert!(report.upper_confidence_bound > 0.0);
        assert_eq!(
            report.independent_sample_concentration_scale,
            1.0 / (1.0 / 3.0 + 1.0 / 5.0)
        );
    }

    #[test]
    fn diagonal_collision_matrix_does_not_treat_pairs_as_independent() {
        let mut matrix = vec![vec![false; 10]; 10];
        for (i, row) in matrix.iter_mut().enumerate() {
            row[i] = true;
        }
        let report =
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &evidence(10, 10)).unwrap();
        assert!((report.estimate - 0.1).abs() < 1e-12);
        assert_eq!(report.pair_comparison_count, 100);
        assert_eq!(report.propagated_trajectory_count, 20);
        assert!((report.independent_sample_concentration_scale - 5.0).abs() < 1e-12);
        let expected_radius = (0.5 * (0.1 + 0.1) * (1.0_f64 / 0.05).ln()).sqrt();
        assert!((report.mcdiarmid_radius - expected_radius).abs() < 1e-12);
        assert!((report.upper_confidence_bound - (0.1 + expected_radius).min(1.0)).abs() < 1e-12);
    }

    #[test]
    fn asymmetric_ensembles_use_both_independent_sample_counts() {
        let matrix = vec![vec![true; 8]; 2];
        let report =
            estimate_pair_reuse_collision_probability(&matrix, 0.1, &evidence(2, 8)).unwrap();
        assert_eq!(report.estimate, 1.0);
        assert_eq!(report.upper_confidence_bound, 1.0);
        assert_eq!(
            report.independent_sample_concentration_scale,
            1.0 / (0.5 + 0.125)
        );
    }

    #[test]
    fn rejects_ragged_collision_matrices() {
        let matrix = vec![vec![false, true], vec![false]];
        assert!(matches!(
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &evidence(2, 2)),
            Err(PairReuseError::RaggedCollisionMatrix { row: 1, .. })
        ));
    }

    #[test]
    fn rejects_primary_secondary_dependence_without_cluster_model() {
        let matrix = vec![vec![false, true], vec![true, false]];
        let mut evidence = evidence(2, 2);
        evidence.primary_secondary_ensembles_are_independent = false;
        assert_eq!(
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &evidence),
            Err(PairReuseError::CrossObjectDependenceUnsupported)
        );
    }

    #[test]
    fn rejects_reused_trajectory_ids_and_invalid_matrix_digest() {
        let matrix = vec![vec![false, true], vec![true, false]];
        let mut duplicate_evidence = evidence(2, 2);
        duplicate_evidence.primary_trajectory_ids[1] =
            duplicate_evidence.primary_trajectory_ids[0].clone();
        assert_eq!(
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &duplicate_evidence),
            Err(PairReuseError::DuplicateTrajectoryId(
                "primary-trajectory-0".into()
            ))
        );

        let mut invalid_evidence = evidence(2, 2);
        invalid_evidence.collision_matrix_sha256 = "not-a-sha256".into();
        assert_eq!(
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &invalid_evidence),
            Err(PairReuseError::InvalidSha256("collision_matrix_sha256"))
        );
    }

    #[test]
    fn rejects_invalid_oracle_and_delta() {
        let matrix = vec![vec![false], vec![true]];
        let mut invalid_evidence = evidence(2, 1);
        invalid_evidence.high_fidelity_oracle_is_continuous_time = false;
        assert_eq!(
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &invalid_evidence),
            Err(PairReuseError::HighFidelityContinuousTimeOracleRequired)
        );
        assert_eq!(
            estimate_pair_reuse_collision_probability(&matrix, 1.0, &evidence(2, 1)),
            Err(PairReuseError::InvalidDelta(1.0))
        );
    }

    #[test]
    fn row_and_column_permutations_preserve_the_all_pairs_estimate() {
        let matrix = vec![
            vec![true, false, false],
            vec![false, true, false],
            vec![false, false, true],
            vec![true, true, false],
        ];
        let mut row_reversed = matrix.clone();
        row_reversed.reverse();
        let column_reversed = row_reversed
            .iter()
            .map(|row| row.iter().rev().copied().collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let report_a =
            estimate_pair_reuse_collision_probability(&matrix, 0.05, &evidence(4, 3)).unwrap();
        let report_b =
            estimate_pair_reuse_collision_probability(&column_reversed, 0.05, &evidence(4, 3))
                .unwrap();
        assert_eq!(report_a.estimate, report_b.estimate);
        assert_eq!(report_a.collision_pair_count, report_b.collision_pair_count);
    }
}
