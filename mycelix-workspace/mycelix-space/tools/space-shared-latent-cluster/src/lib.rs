#![forbid(unsafe_code)]

//! Cluster-level collision estimation under shared environmental uncertainty.
//!
//! Each cluster k carries one independently sampled common environment Z_k
//! (e.g. atmospheric-density forecast error). Conditional on Z_k, the caller
//! supplies IID primary and secondary residual trajectory ensembles, which
//! are conditionally independent of each other in this E0 implementation.
//! Their all-pairs event average U_k is bounded in [0,1] and estimates the
//! conditional collision probability for that environment.
//!
//! The environment-marginalized estimand is Pc = E_Z[ Pc(Z) ]. The average of
//! independent cluster means estimates this target. Hoeffding's inequality is
//! applied over K independent environment clusters, never over the dependent
//! within-cluster pair outcomes.
//!
//! The model flags and evidence references are caller attestations, not
//! automatically proven facts. Residual cross-object dependence beyond the
//! declared shared latent requires a separately derived joint/cluster model.

use std::collections::BTreeSet;
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub struct SharedEnvironmentCluster {
    pub cluster_id: String,
    pub shared_latent_draw_id: String,
    pub shared_latent_artifact_sha256: String,
    pub primary_sample_set_id: String,
    pub secondary_sample_set_id: String,
    pub primary_trajectory_ids: Vec<String>,
    pub secondary_trajectory_ids: Vec<String>,
    pub primary_sample_order_sha256: String,
    pub secondary_sample_order_sha256: String,
    pub collision_matrix_sha256: String,
    /// Rows are primary trajectories; columns are secondary trajectories.
    pub collision_matrix: Vec<Vec<bool>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SharedLatentEvidence {
    pub shared_environment_model_id: String,
    pub shared_environment_model_revision: String,
    pub shared_environment_distribution_id: String,
    pub cluster_sampling_design_id: String,
    pub cluster_sampling_evidence_ref: String,
    pub cluster_order_sha256: String,
    pub conditional_sampler_model_id: String,
    pub conditional_sampler_model_revision: String,
    pub conditional_sampling_evidence_ref: String,
    pub primary_secondary_conditional_independence_evidence_ref: String,
    pub primary_secondary_conditionally_independent_given_shared_latent: bool,
    pub primary_draws_are_iid_conditionally: bool,
    pub secondary_draws_are_iid_conditionally: bool,
    pub shared_environment_draws_are_iid: bool,
    pub clusters_are_independent_and_target_homogeneous: bool,
    pub conditional_pair_sampling_is_unbiased: bool,
    pub trajectories_propagated_once_per_cluster: bool,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub high_fidelity_oracle_is_continuous_time: bool,
    pub event_kernel_is_fixed_across_clusters: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ClusterContribution {
    pub cluster_id: String,
    pub shared_latent_draw_id: String,
    pub primary_trajectory_count: usize,
    pub secondary_trajectory_count: usize,
    pub pair_comparison_count: usize,
    pub collision_pair_count: usize,
    pub cluster_collision_probability: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SharedLatentClusterReport {
    pub method: &'static str,
    pub estimand: &'static str,
    pub shared_environment_model_id: String,
    pub shared_environment_model_revision: String,
    pub shared_environment_distribution_id: String,
    pub cluster_sampling_design_id: String,
    pub cluster_sampling_evidence_ref: String,
    pub cluster_order_sha256: String,
    pub conditional_sampler_model_id: String,
    pub conditional_sampler_model_revision: String,
    pub conditional_sampling_evidence_ref: String,
    pub primary_secondary_conditional_independence_evidence_ref: String,
    pub event_definition_id: String,
    pub event_horizon_id: String,
    pub high_fidelity_event_oracle_id: String,
    pub high_fidelity_event_oracle_revision: String,
    pub cluster_count: usize,
    pub total_propagated_trajectories: usize,
    pub total_pair_comparisons: usize,
    pub total_collision_pairs: usize,
    pub mean_cluster_probability: f64,
    pub between_cluster_sample_variance: f64,
    pub confidence_delta: f64,
    pub one_sided_confidence_level: f64,
    pub hoeffding_radius: f64,
    pub upper_confidence_bound: f64,
    pub independent_statistical_unit: &'static str,
    pub within_cluster_pair_outcomes_are_dependent: bool,
    pub clusters: Vec<ClusterContribution>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ClusterError {
    EmptyClusterSet,
    InsufficientIndependentClusters(usize),
    InvalidIdentifier(&'static str),
    InvalidSha256(&'static str),
    DuplicateClusterId(String),
    DuplicateSharedLatentDrawId(String),
    DuplicateTrajectoryId(String),
    PrimarySecondarySampleSetIdentityCollision,
    EmptyPrimaryEnsemble(String),
    EmptySecondaryEnsemble(String),
    EmptyCollisionMatrix(String),
    MatrixPrimaryRowCountMismatch {
        cluster_id: String,
        expected: usize,
        actual: usize,
    },
    RaggedCollisionMatrix {
        cluster_id: String,
        row: usize,
        expected_columns: usize,
        actual_columns: usize,
    },
    MissingSamplingEvidence,
    SharedEnvironmentSamplesNotIid,
    ClustersNotIndependentOrDifferentTarget,
    PrimarySecondaryConditionalDependenceUnsupported,
    PrimaryResidualsNotIidConditionally,
    SecondaryResidualsNotIidConditionally,
    ConditionalSamplerNotUnbiased,
    TrajectoryPropagationReuseNotAttested,
    HighFidelityContinuousTimeOracleRequired,
    EventKernelChangedBetweenClusters,
    InvalidDelta(f64),
    CountOverflow,
}

impl fmt::Display for ClusterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyClusterSet => {
                write!(f, "at least one shared-environment cluster is required")
            }
            Self::InsufficientIndependentClusters(k) => write!(
                f,
                "at least two independent environment clusters are required; got {k}"
            ),
            Self::InvalidIdentifier(field) => write!(
                f,
                "{field} must be non-empty and use only ASCII letters, digits, dot, underscore, colon, slash and hyphen"
            ),
            Self::InvalidSha256(field) => write!(
                f,
                "{field} must be a 64-character lowercase hexadecimal SHA-256 digest"
            ),
            Self::DuplicateClusterId(value) => write!(f, "duplicate cluster ID {value}"),
            Self::DuplicateSharedLatentDrawId(value) => {
                write!(f, "shared environment draw reused across clusters: {value}")
            }
            Self::DuplicateTrajectoryId(value) => write!(
                f,
                "trajectory ID reused across samples or clusters: {value}"
            ),
            Self::PrimarySecondarySampleSetIdentityCollision => {
                write!(f, "primary and secondary sample-set IDs must differ")
            }
            Self::EmptyPrimaryEnsemble(id) => {
                write!(f, "cluster {id} has an empty primary ensemble")
            }
            Self::EmptySecondaryEnsemble(id) => {
                write!(f, "cluster {id} has an empty secondary ensemble")
            }
            Self::EmptyCollisionMatrix(id) => {
                write!(f, "cluster {id} has an empty collision matrix")
            }
            Self::MatrixPrimaryRowCountMismatch {
                cluster_id,
                expected,
                actual,
            } => write!(
                f,
                "cluster {cluster_id} has {actual} collision rows, expected {expected}"
            ),
            Self::RaggedCollisionMatrix {
                cluster_id,
                row,
                expected_columns,
                actual_columns,
            } => write!(
                f,
                "cluster {cluster_id} row {row} has {actual_columns} columns, expected {expected_columns}"
            ),
            Self::MissingSamplingEvidence => write!(
                f,
                "shared environment, cluster, conditional sampler and dependence evidence references are required"
            ),
            Self::SharedEnvironmentSamplesNotIid => write!(
                f,
                "shared environment draws must be IID under the declared environment distribution"
            ),
            Self::ClustersNotIndependentOrDifferentTarget => write!(
                f,
                "environment clusters must be mutually independent and estimate the same target"
            ),
            Self::PrimarySecondaryConditionalDependenceUnsupported => write!(
                f,
                "primary and secondary residuals must be conditionally independent given the sampled shared latent for this E0 estimator"
            ),
            Self::PrimaryResidualsNotIidConditionally => write!(
                f,
                "primary trajectory residual draws must be IID conditional on the shared latent"
            ),
            Self::SecondaryResidualsNotIidConditionally => write!(
                f,
                "secondary trajectory residual draws must be IID conditional on the shared latent"
            ),
            Self::ConditionalSamplerNotUnbiased => write!(
                f,
                "the within-cluster pair estimator must be conditionally unbiased for the declared conditional collision probability"
            ),
            Self::TrajectoryPropagationReuseNotAttested => write!(
                f,
                "each trajectory must be propagated once within its cluster and then reused only for pair evaluation"
            ),
            Self::HighFidelityContinuousTimeOracleRequired => write!(
                f,
                "the event kernel requires a high-fidelity continuous-time oracle"
            ),
            Self::EventKernelChangedBetweenClusters => write!(
                f,
                "event definition, horizon or high-fidelity oracle revision changed between clusters"
            ),
            Self::InvalidDelta(value) => {
                write!(f, "delta must be finite and in (0,1); got {value}")
            }
            Self::CountOverflow => {
                write!(f, "trajectory or pair comparison count overflowed usize")
            }
        }
    }
}

impl std::error::Error for ClusterError {}

/// Estimate the environment-marginalized collision probability using K
/// independent shared-latent clusters.
///
/// Each U_k is the all-pairs mean within one environment cluster. The
/// statistical sample size is K independent cluster values, not the number of
/// trajectories or the number of within-cluster pair comparisons.
pub fn estimate_environment_marginalized_probability(
    clusters: &[SharedEnvironmentCluster],
    delta: f64,
    evidence: &SharedLatentEvidence,
) -> Result<SharedLatentClusterReport, ClusterError> {
    validate_delta(delta)?;
    validate_all(clusters, evidence)?;

    let mut contributions = Vec::with_capacity(clusters.len());
    let mut total_propagations = 0usize;
    let mut total_pairs = 0usize;
    let mut total_hits = 0usize;

    for cluster in clusters {
        let m = cluster.primary_trajectory_ids.len();
        let n = cluster.secondary_trajectory_ids.len();
        let pair_count = m.checked_mul(n).ok_or(ClusterError::CountOverflow)?;
        let propagation_count = m.checked_add(n).ok_or(ClusterError::CountOverflow)?;
        let collision_count = cluster
            .collision_matrix
            .iter()
            .flat_map(|row| row.iter())
            .filter(|value| **value)
            .count();
        let probability = collision_count as f64 / pair_count as f64;

        total_propagations = total_propagations
            .checked_add(propagation_count)
            .ok_or(ClusterError::CountOverflow)?;
        total_pairs = total_pairs
            .checked_add(pair_count)
            .ok_or(ClusterError::CountOverflow)?;
        total_hits = total_hits
            .checked_add(collision_count)
            .ok_or(ClusterError::CountOverflow)?;

        contributions.push(ClusterContribution {
            cluster_id: cluster.cluster_id.clone(),
            shared_latent_draw_id: cluster.shared_latent_draw_id.clone(),
            primary_trajectory_count: m,
            secondary_trajectory_count: n,
            pair_comparison_count: pair_count,
            collision_pair_count: collision_count,
            cluster_collision_probability: probability,
        });
    }

    let k = contributions.len();
    let mean = contributions
        .iter()
        .map(|v| v.cluster_collision_probability)
        .sum::<f64>()
        / k as f64;
    let variance = sample_variance(
        &contributions
            .iter()
            .map(|v| v.cluster_collision_probability)
            .collect::<Vec<_>>(),
        mean,
    );
    let radius = ((1.0 / delta).ln() / (2.0 * k as f64)).sqrt();
    let upper = (mean + radius).min(1.0);

    Ok(SharedLatentClusterReport {
        method: "shared_latent_cluster_mean_hoeffding_upper_bound",
        estimand: "environment_marginalized_collision_probability",
        shared_environment_model_id: evidence.shared_environment_model_id.clone(),
        shared_environment_model_revision: evidence.shared_environment_model_revision.clone(),
        shared_environment_distribution_id: evidence.shared_environment_distribution_id.clone(),
        cluster_sampling_design_id: evidence.cluster_sampling_design_id.clone(),
        cluster_sampling_evidence_ref: evidence.cluster_sampling_evidence_ref.clone(),
        cluster_order_sha256: evidence.cluster_order_sha256.clone(),
        conditional_sampler_model_id: evidence.conditional_sampler_model_id.clone(),
        conditional_sampler_model_revision: evidence.conditional_sampler_model_revision.clone(),
        conditional_sampling_evidence_ref: evidence.conditional_sampling_evidence_ref.clone(),
        primary_secondary_conditional_independence_evidence_ref: evidence
            .primary_secondary_conditional_independence_evidence_ref
            .clone(),
        event_definition_id: evidence.event_definition_id.clone(),
        event_horizon_id: evidence.event_horizon_id.clone(),
        high_fidelity_event_oracle_id: evidence.high_fidelity_event_oracle_id.clone(),
        high_fidelity_event_oracle_revision: evidence.high_fidelity_event_oracle_revision.clone(),
        cluster_count: k,
        total_propagated_trajectories: total_propagations,
        total_pair_comparisons: total_pairs,
        total_collision_pairs: total_hits,
        mean_cluster_probability: mean,
        between_cluster_sample_variance: variance,
        confidence_delta: delta,
        one_sided_confidence_level: 1.0 - delta,
        hoeffding_radius: radius,
        upper_confidence_bound: upper,
        independent_statistical_unit: "independent_shared_environment_cluster",
        within_cluster_pair_outcomes_are_dependent: true,
        clusters: contributions,
        claim_ceiling: "statistical_bound_conditional_on_iid_independent_environment_clusters_correct_conditional_sampling_model_and_fixed_event_kernel_not_physical_safety",
    })
}

fn validate_all(
    clusters: &[SharedEnvironmentCluster],
    evidence: &SharedLatentEvidence,
) -> Result<(), ClusterError> {
    if clusters.is_empty() {
        return Err(ClusterError::EmptyClusterSet);
    }
    if clusters.len() < 2 {
        return Err(ClusterError::InsufficientIndependentClusters(
            clusters.len(),
        ));
    }

    for (field, value) in [
        (
            "shared_environment_model_id",
            &evidence.shared_environment_model_id,
        ),
        (
            "shared_environment_model_revision",
            &evidence.shared_environment_model_revision,
        ),
        (
            "shared_environment_distribution_id",
            &evidence.shared_environment_distribution_id,
        ),
        (
            "cluster_sampling_design_id",
            &evidence.cluster_sampling_design_id,
        ),
        (
            "conditional_sampler_model_id",
            &evidence.conditional_sampler_model_id,
        ),
        (
            "conditional_sampler_model_revision",
            &evidence.conditional_sampler_model_revision,
        ),
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
            return Err(ClusterError::InvalidIdentifier(field));
        }
    }
    if evidence.cluster_sampling_evidence_ref.trim().is_empty()
        || evidence.conditional_sampling_evidence_ref.trim().is_empty()
        || evidence
            .primary_secondary_conditional_independence_evidence_ref
            .trim()
            .is_empty()
    {
        return Err(ClusterError::MissingSamplingEvidence);
    }
    if !valid_sha256(&evidence.cluster_order_sha256) {
        return Err(ClusterError::InvalidSha256("cluster_order_sha256"));
    }
    if !evidence.shared_environment_draws_are_iid {
        return Err(ClusterError::SharedEnvironmentSamplesNotIid);
    }
    if !evidence.clusters_are_independent_and_target_homogeneous {
        return Err(ClusterError::ClustersNotIndependentOrDifferentTarget);
    }
    if !evidence.primary_secondary_conditionally_independent_given_shared_latent {
        return Err(ClusterError::PrimarySecondaryConditionalDependenceUnsupported);
    }
    if !evidence.primary_draws_are_iid_conditionally {
        return Err(ClusterError::PrimaryResidualsNotIidConditionally);
    }
    if !evidence.secondary_draws_are_iid_conditionally {
        return Err(ClusterError::SecondaryResidualsNotIidConditionally);
    }
    if !evidence.conditional_pair_sampling_is_unbiased {
        return Err(ClusterError::ConditionalSamplerNotUnbiased);
    }
    if !evidence.trajectories_propagated_once_per_cluster {
        return Err(ClusterError::TrajectoryPropagationReuseNotAttested);
    }
    if !evidence.high_fidelity_oracle_is_continuous_time {
        return Err(ClusterError::HighFidelityContinuousTimeOracleRequired);
    }
    if !evidence.event_kernel_is_fixed_across_clusters {
        return Err(ClusterError::EventKernelChangedBetweenClusters);
    }

    let first = &clusters[0];
    let mut cluster_ids = BTreeSet::new();
    let mut latent_ids = BTreeSet::new();
    let mut trajectory_ids = BTreeSet::new();

    for cluster in clusters {
        for (field, value) in [
            ("cluster_id", &cluster.cluster_id),
            ("shared_latent_draw_id", &cluster.shared_latent_draw_id),
            ("primary_sample_set_id", &cluster.primary_sample_set_id),
            ("secondary_sample_set_id", &cluster.secondary_sample_set_id),
        ] {
            if !valid_identifier(value) {
                return Err(ClusterError::InvalidIdentifier(field));
            }
        }
        if !cluster_ids.insert(cluster.cluster_id.as_str()) {
            return Err(ClusterError::DuplicateClusterId(cluster.cluster_id.clone()));
        }
        if !latent_ids.insert(cluster.shared_latent_draw_id.as_str()) {
            return Err(ClusterError::DuplicateSharedLatentDrawId(
                cluster.shared_latent_draw_id.clone(),
            ));
        }
        if cluster.primary_sample_set_id == cluster.secondary_sample_set_id {
            return Err(ClusterError::PrimarySecondarySampleSetIdentityCollision);
        }
        if cluster.primary_trajectory_ids.is_empty() {
            return Err(ClusterError::EmptyPrimaryEnsemble(
                cluster.cluster_id.clone(),
            ));
        }
        if cluster.secondary_trajectory_ids.is_empty() {
            return Err(ClusterError::EmptySecondaryEnsemble(
                cluster.cluster_id.clone(),
            ));
        }
        if cluster.collision_matrix.is_empty() {
            return Err(ClusterError::EmptyCollisionMatrix(
                cluster.cluster_id.clone(),
            ));
        }
        if cluster.collision_matrix.len() != cluster.primary_trajectory_ids.len() {
            return Err(ClusterError::MatrixPrimaryRowCountMismatch {
                cluster_id: cluster.cluster_id.clone(),
                expected: cluster.primary_trajectory_ids.len(),
                actual: cluster.collision_matrix.len(),
            });
        }
        for (row_index, row) in cluster.collision_matrix.iter().enumerate() {
            if row.len() != cluster.secondary_trajectory_ids.len() {
                return Err(ClusterError::RaggedCollisionMatrix {
                    cluster_id: cluster.cluster_id.clone(),
                    row: row_index,
                    expected_columns: cluster.secondary_trajectory_ids.len(),
                    actual_columns: row.len(),
                });
            }
        }
        for (field, digest) in [
            (
                "shared_latent_artifact_sha256",
                &cluster.shared_latent_artifact_sha256,
            ),
            (
                "primary_sample_order_sha256",
                &cluster.primary_sample_order_sha256,
            ),
            (
                "secondary_sample_order_sha256",
                &cluster.secondary_sample_order_sha256,
            ),
            ("collision_matrix_sha256", &cluster.collision_matrix_sha256),
        ] {
            if !valid_sha256(digest) {
                return Err(ClusterError::InvalidSha256(field));
            }
        }
        for id in cluster
            .primary_trajectory_ids
            .iter()
            .chain(cluster.secondary_trajectory_ids.iter())
        {
            if !valid_identifier(id) {
                return Err(ClusterError::InvalidIdentifier("trajectory_id"));
            }
            if !trajectory_ids.insert(id.as_str()) {
                return Err(ClusterError::DuplicateTrajectoryId(id.clone()));
            }
        }
        // The fixed event kernel identity is carried by one global evidence object,
        // so a single estimator invocation cannot silently mix event-oracle revisions.
        let _ = first;
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

fn validate_delta(delta: f64) -> Result<(), ClusterError> {
    if !delta.is_finite() || delta <= 0.0 || delta >= 1.0 {
        return Err(ClusterError::InvalidDelta(delta));
    }
    Ok(())
}

fn sample_variance(values: &[f64], mean: f64) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
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

    fn evidence() -> SharedLatentEvidence {
        SharedLatentEvidence {
            shared_environment_model_id: "shared-density-error-model-v1".into(),
            shared_environment_model_revision: "sha256:environment-model-v1".into(),
            shared_environment_distribution_id: "density-error-prior-v1".into(),
            cluster_sampling_design_id: "iid-environment-clusters-v1".into(),
            cluster_sampling_evidence_ref: "fixture://clusters/iid-design-v1".into(),
            cluster_order_sha256: sha('a'),
            conditional_sampler_model_id: "conditional-trajectory-sampler-v1".into(),
            conditional_sampler_model_revision: "sha256:conditional-sampler-v1".into(),
            conditional_sampling_evidence_ref: "fixture://conditional-sampler/model-v1".into(),
            primary_secondary_conditional_independence_evidence_ref:
                "fixture://conditional-independence/model-v1".into(),
            primary_secondary_conditionally_independent_given_shared_latent: true,
            primary_draws_are_iid_conditionally: true,
            secondary_draws_are_iid_conditionally: true,
            shared_environment_draws_are_iid: true,
            clusters_are_independent_and_target_homogeneous: true,
            conditional_pair_sampling_is_unbiased: true,
            trajectories_propagated_once_per_cluster: true,
            event_definition_id: "any-contact-over-horizon-v1".into(),
            event_horizon_id: "seven-day-window-v1".into(),
            high_fidelity_event_oracle_id: "hf-continuous-time-oracle".into(),
            high_fidelity_event_oracle_revision: "sha256:hf-oracle-v1".into(),
            high_fidelity_oracle_is_continuous_time: true,
            event_kernel_is_fixed_across_clusters: true,
        }
    }

    fn cluster(index: usize, matrix: Vec<Vec<bool>>) -> SharedEnvironmentCluster {
        let m = matrix.len();
        let n = matrix.first().map_or(0, |row| row.len());
        SharedEnvironmentCluster {
            cluster_id: format!("cluster-{index}"),
            shared_latent_draw_id: format!("environment-draw-{index}"),
            shared_latent_artifact_sha256: sha('b'),
            primary_sample_set_id: format!("primary-set-{index}"),
            secondary_sample_set_id: format!("secondary-set-{index}"),
            primary_trajectory_ids: (0..m)
                .map(|i| format!("cluster-{index}-primary-{i}"))
                .collect(),
            secondary_trajectory_ids: (0..n)
                .map(|i| format!("cluster-{index}-secondary-{i}"))
                .collect(),
            primary_sample_order_sha256: sha('c'),
            secondary_sample_order_sha256: sha('d'),
            collision_matrix_sha256: sha('e'),
            collision_matrix: matrix,
        }
    }

    #[test]
    fn estimates_environment_marginalized_mean_over_independent_clusters() {
        let clusters = vec![
            cluster(0, vec![vec![false, false], vec![false, false]]),
            cluster(1, vec![vec![true, false], vec![false, false]]),
            cluster(2, vec![vec![true, true], vec![false, false]]),
            cluster(3, vec![vec![true, true], vec![true, true]]),
        ];
        let report =
            estimate_environment_marginalized_probability(&clusters, 0.05, &evidence()).unwrap();
        assert_eq!(report.cluster_count, 4);
        assert_eq!(report.total_pair_comparisons, 16);
        assert_eq!(report.total_propagated_trajectories, 16);
        assert_eq!(report.total_collision_pairs, 9);
        assert!((report.mean_cluster_probability - 0.5625).abs() < 1e-12);
        assert!(report.upper_confidence_bound >= report.mean_cluster_probability);
        assert!(report.upper_confidence_bound <= 1.0);
        assert_eq!(
            report.independent_statistical_unit,
            "independent_shared_environment_cluster"
        );
        assert_eq!(report.within_cluster_pair_outcomes_are_dependent, true);
        assert_eq!(
            report.estimand,
            "environment_marginalized_collision_probability"
        );
    }

    #[test]
    fn does_not_use_within_cluster_pair_count_as_statistical_sample_size() {
        let clusters = vec![
            cluster(0, vec![vec![false; 100]; 100]),
            cluster(1, vec![vec![true; 100]; 100]),
        ];
        let report =
            estimate_environment_marginalized_probability(&clusters, 0.05, &evidence()).unwrap();
        assert_eq!(report.total_pair_comparisons, 20_000);
        assert_eq!(report.cluster_count, 2);
        let expected_radius = (0.05_f64.recip().ln() / 4.0).sqrt();
        assert!((report.hoeffding_radius - expected_radius).abs() < 1e-12);
    }

    #[test]
    fn rejects_reused_shared_environment_draws() {
        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        let mut second = clusters.clone();
        second[1].shared_latent_draw_id = second[0].shared_latent_draw_id.clone();
        assert_eq!(
            estimate_environment_marginalized_probability(&second, 0.05, &evidence()),
            Err(ClusterError::DuplicateSharedLatentDrawId(
                "environment-draw-0".into()
            ))
        );
    }

    #[test]
    fn rejects_conditional_primary_secondary_dependence_without_joint_model() {
        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        let mut model = evidence();
        model.primary_secondary_conditionally_independent_given_shared_latent = false;
        assert_eq!(
            estimate_environment_marginalized_probability(&clusters, 0.05, &model),
            Err(ClusterError::PrimarySecondaryConditionalDependenceUnsupported)
        );
    }

    #[test]
    fn rejects_non_iid_cluster_sequence_and_invalid_event_kernel() {
        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        let mut model = evidence();
        model.clusters_are_independent_and_target_homogeneous = false;
        assert_eq!(
            estimate_environment_marginalized_probability(&clusters, 0.05, &model),
            Err(ClusterError::ClustersNotIndependentOrDifferentTarget)
        );

        let mut model = evidence();
        model.event_kernel_is_fixed_across_clusters = false;
        assert_eq!(
            estimate_environment_marginalized_probability(&clusters, 0.05, &model),
            Err(ClusterError::EventKernelChangedBetweenClusters)
        );
    }

    #[test]
    fn rejects_ragged_matrix_and_invalid_cluster_order_digest() {
        let clusters = vec![
            cluster(0, vec![vec![false, true], vec![false, false]]),
            cluster(1, vec![vec![false, true], vec![false]]),
        ];
        assert!(matches!(
            estimate_environment_marginalized_probability(&clusters, 0.05, &evidence()),
            Err(ClusterError::RaggedCollisionMatrix { cluster_id, row: 1, .. }) if cluster_id == "cluster-1"
        ));

        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        let mut model = evidence();
        model.cluster_order_sha256 = "not-a-digest".into();
        assert_eq!(
            estimate_environment_marginalized_probability(&clusters, 0.05, &model),
            Err(ClusterError::InvalidSha256("cluster_order_sha256"))
        );
    }

    #[test]
    fn rejects_duplicate_trajectory_ids_across_clusters() {
        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        let mut duplicate = clusters.clone();
        duplicate[1].primary_trajectory_ids[0] = duplicate[0].primary_trajectory_ids[0].clone();
        assert_eq!(
            estimate_environment_marginalized_probability(&duplicate, 0.05, &evidence()),
            Err(ClusterError::DuplicateTrajectoryId(
                "cluster-0-primary-0".into()
            ))
        );
    }

    #[test]
    fn rejects_insufficient_clusters_and_invalid_delta() {
        let one = vec![cluster(0, vec![vec![false]])];
        assert_eq!(
            estimate_environment_marginalized_probability(&one, 0.05, &evidence()),
            Err(ClusterError::InsufficientIndependentClusters(1))
        );
        let clusters = vec![cluster(0, vec![vec![false]]), cluster(1, vec![vec![true]])];
        assert_eq!(
            estimate_environment_marginalized_probability(&clusters, 1.0, &evidence()),
            Err(ClusterError::InvalidDelta(1.0))
        );
    }
}
