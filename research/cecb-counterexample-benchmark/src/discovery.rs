//! Transparent discovery baseline for CECB.
//!
//! This module implements a deterministic univariate threshold learner. It is
//! deliberately a baseline, not a claim of state-of-the-art performance. The
//! observation budget limits how many labelled development rows the learner
//! consumes; the trusted harness must enforce process isolation and account for
//! any additional model/simulator queries independently.

use crate::protocol::{CandidateSubmission, CandidateView, Prediction, TrainingObservation};
use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThresholdDirection {
    GreaterOrEqual,
    LessOrEqual,
}

impl ThresholdDirection {
    fn rank(self) -> u8 {
        match self {
            Self::GreaterOrEqual => 0,
            Self::LessOrEqual => 1,
        }
    }

    fn matches(self, value: i32, threshold: i64) -> bool {
        match self {
            Self::GreaterOrEqual => i64::from(value) >= threshold,
            Self::LessOrEqual => i64::from(value) <= threshold,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::GreaterOrEqual => "greater_or_equal",
            Self::LessOrEqual => "less_or_equal",
        }
    }
}

/// Fitted one-feature threshold selected only from labelled development rows.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnivariateThreshold {
    pub feature_index: usize,
    pub direction: ThresholdDirection,
    pub threshold: i64,
    pub development_errors: usize,
    pub development_feasible_examples: usize,
}

/// Candidate result plus an auditable record of the labelled rows it consumed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DiscoveryResult {
    pub model: UnivariateThreshold,
    pub submission: CandidateSubmission,
    pub development_observations_used: usize,
    pub queried_sample_ids: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DiscoveryError {
    EmptyDevelopment,
    EmptyHoldout,
    ZeroObservationBudget,
    InconsistentDimensions,
    NoFeasibleTrainingExamples,
}

fn feasible_errors(
    rows: &[&TrainingObservation],
    feature_index: usize,
    direction: ThresholdDirection,
    threshold: i64,
) -> (usize, usize) {
    let mut errors = 0;
    let mut feasible_examples = 0;
    for row in rows {
        if !row.feasible {
            continue;
        }
        feasible_examples += 1;
        let predicted = direction.matches(row.features[feature_index], threshold);
        if predicted != row.observed_failure {
            errors += 1;
        }
    }
    (errors, feasible_examples)
}

/// Evenly samples a deterministic subset of development rows.
fn select_budgeted_rows(
    development: &[TrainingObservation],
    budget: usize,
) -> Vec<&TrainingObservation> {
    let count = budget.min(development.len());
    if count == 0 {
        return Vec::new();
    }
    if count == 1 {
        return vec![&development[development.len() / 2]];
    }
    let last = development.len() - 1;
    (0..count)
        .map(|slot| {
            let index = slot.saturating_mul(last) / (count - 1);
            &development[index]
        })
        .collect()
}

/// Fits the minimum-training-error single-feature threshold over an evenly
/// spaced, deterministic subset of the development observations.
///
/// Tie-breaking is fixed: fewer errors, then lower feature index, then
/// GreaterOrEqual before LessOrEqual, then lower threshold. Only observed
/// development labels are used; latent truth and holdout labels are not fields
/// on CandidateView. Holdout examples are predicted exactly once each.
///
/// The observation budget counts labelled development rows consumed. It is not
/// a wall-clock or compute budget; the harness must record those separately.
pub fn discover_univariate_threshold(
    view: &CandidateView,
    max_labeled_observations: usize,
) -> Result<DiscoveryResult, DiscoveryError> {
    if view.development.is_empty() {
        return Err(DiscoveryError::EmptyDevelopment);
    }
    if view.holdout_inputs.is_empty() {
        return Err(DiscoveryError::EmptyHoldout);
    }
    if max_labeled_observations == 0 {
        return Err(DiscoveryError::ZeroObservationBudget);
    }

    let dimensions = view.development[0].features.len();
    if dimensions == 0
        || view
            .development
            .iter()
            .any(|row| row.features.len() != dimensions)
        || view
            .holdout_inputs
            .iter()
            .any(|row| row.features.len() != dimensions)
    {
        return Err(DiscoveryError::InconsistentDimensions);
    }

    let selected = select_budgeted_rows(&view.development, max_labeled_observations);
    let feasible_count = selected.iter().filter(|row| row.feasible).count();
    if feasible_count == 0 {
        return Err(DiscoveryError::NoFeasibleTrainingExamples);
    }

    let mut best: Option<(usize, usize, u8, i64, ThresholdDirection)> = None;
    for feature_index in 0..dimensions {
        let values = selected
            .iter()
            .map(|row| i64::from(row.features[feature_index]))
            .collect::<Vec<_>>();
        let min_value = *values.iter().min().expect("selected rows are non-empty");
        let max_value = *values.iter().max().expect("selected rows are non-empty");
        let mut thresholds = values.into_iter().collect::<BTreeSet<_>>();
        thresholds.insert(min_value - 1);
        thresholds.insert(max_value + 1);

        for direction in [
            ThresholdDirection::GreaterOrEqual,
            ThresholdDirection::LessOrEqual,
        ] {
            for &threshold in &thresholds {
                let (errors, _) =
                    feasible_errors(&selected, feature_index, direction, threshold);
                let candidate = (
                    errors,
                    feature_index,
                    direction.rank(),
                    threshold,
                    direction,
                );
                let candidate_key = (candidate.0, candidate.1, candidate.2, candidate.3);
                let is_better = best.as_ref().map_or(true, |current| {
                    candidate_key < (current.0, current.1, current.2, current.3)
                });
                if is_better {
                    best = Some(candidate);
                }
            }
        }
    }

    let (development_errors, feature_index, _, threshold, direction) =
        best.ok_or(DiscoveryError::NoFeasibleTrainingExamples)?;
    let model = UnivariateThreshold {
        feature_index,
        direction,
        threshold,
        development_errors,
        development_feasible_examples: feasible_count,
    };
    let predictions = view
        .holdout_inputs
        .iter()
        .map(|input| Prediction {
            sample_id: input.sample_id,
            predicts_failure: direction.matches(input.features[feature_index], threshold),
        })
        .collect();
    let queried_sample_ids = selected.iter().map(|row| row.sample_id).collect::<Vec<_>>();
    let submission = CandidateSubmission {
        method_id: "cecb.univariate_threshold".to_owned(),
        method_version: "0.1.0".to_owned(),
        predictions,
    };

    Ok(DiscoveryResult {
        model,
        submission,
        development_observations_used: queried_sample_ids.len(),
        queried_sample_ids,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{
        build_model_family_split, FamilyTaggedPoint,
    };
    use crate::{axis_aligned_landscape, structural_family_a, structural_family_b, LabelSource};

    fn family_fixture() -> Vec<FamilyTaggedPoint> {
        let mut rows = structural_family_a()
            .into_iter()
            .map(|point| FamilyTaggedPoint {
                family_id: "family_a".to_owned(),
                point,
            })
            .collect::<Vec<_>>();
        rows.extend(
            structural_family_b()
                .into_iter()
                .map(|point| FamilyTaggedPoint {
                    family_id: "family_b".to_owned(),
                    point,
                }),
        );
        rows
    }

    #[test]
    fn recovers_axis_aligned_rule_on_full_development_set() {
        let split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let result = discover_univariate_threshold(split.candidate_view(), 41).unwrap();
        assert_eq!(result.model.feature_index, 0);
        assert_eq!(result.model.direction, ThresholdDirection::GreaterOrEqual);
        assert_eq!(result.model.threshold, 2);
        assert_eq!(result.model.development_errors, 0);
        assert_eq!(result.development_observations_used, 41);
        let report = split
            .evaluate(&result.submission, result.development_observations_used)
            .unwrap();
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
    }

    #[test]
    fn budget_is_respected_and_sample_ids_are_deterministic() {
        let split = build_model_family_split(
            &family_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            3,
        )
        .unwrap();
        let first = discover_univariate_threshold(split.candidate_view(), 3).unwrap();
        let second = discover_univariate_threshold(split.candidate_view(), 3).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.development_observations_used, 3);
        assert_eq!(first.queried_sample_ids.len(), 3);
        assert_eq!(first.queried_sample_ids.first(), Some(&0));
        assert_eq!(first.queried_sample_ids.last(), Some(&8));
    }

    #[test]
    fn fitted_rule_fails_on_reversed_structural_family() {
        let split = build_model_family_split(
            &family_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            9,
        )
        .unwrap();
        let result = discover_univariate_threshold(split.candidate_view(), 9).unwrap();
        assert_eq!(result.model.direction, ThresholdDirection::GreaterOrEqual);
        assert_eq!(result.model.threshold, 2);
        let report = split
            .evaluate(&result.submission, result.development_observations_used)
            .unwrap();
        assert_eq!(report.score.true_positive, 0);
        assert_eq!(report.score.false_positive, 3);
        assert_eq!(report.score.false_negative, 3);
        assert_eq!(report.score.true_negative, 3);
    }

    #[test]
    fn candidate_result_is_independent_of_latent_label_changes() {
        let original = axis_aligned_landscape();
        let mut relabelled = original.clone();
        for point in &mut relabelled {
            point.latent_failure = !point.latent_failure;
        }
        let original_split = crate::protocol::build_protocol_split(
            &original,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let relabelled_split = crate::protocol::build_protocol_split(
            &relabelled,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let first = discover_univariate_threshold(original_split.candidate_view(), 41).unwrap();
        let second = discover_univariate_threshold(relabelled_split.candidate_view(), 41).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn zero_budget_and_empty_views_fail_closed() {
        let split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        assert_eq!(
            discover_univariate_threshold(split.candidate_view(), 0),
            Err(DiscoveryError::ZeroObservationBudget)
        );
        let empty = CandidateView {
            development: vec![],
            holdout_inputs: vec![],
            manifest: split.candidate_view().manifest.clone(),
        };
        assert_eq!(
            discover_univariate_threshold(&empty, 1),
            Err(DiscoveryError::EmptyDevelopment)
        );
    }

    #[test]
    fn malformed_feature_dimensions_fail_closed() {
        let split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let mut view = split.candidate_view().clone();
        view.holdout_inputs[0].features.pop();
        assert_eq!(
            discover_univariate_threshold(&view, 41),
            Err(DiscoveryError::InconsistentDimensions)
        );
    }
}
