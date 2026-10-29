//! Transparent discovery baseline for CECB.
//!
//! This module implements a deterministic univariate threshold learner. It is
//! deliberately a baseline, not a claim of state-of-the-art performance. Feature
//! rows are visible, but development labels are acquired through a budgeted oracle.

use crate::protocol::{
    CandidateSubmission, CandidateView, DevelopmentInput, DevelopmentOracle, Prediction,
    ProtocolError, TrainingObservation,
};
use crate::{
    exhaustive_axis_aligned_boxes, BoxCandidate, LabelSource, Point, RuleError,
};
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

/// Fitted one-feature threshold selected only from labelled oracle responses.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnivariateThreshold {
    pub feature_index: usize,
    pub direction: ThresholdDirection,
    pub threshold: i64,
    pub development_errors: usize,
    pub development_feasible_examples: usize,
}

/// Candidate result plus the requested-observation trace for reproducibility.
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
    UnsupportedDimensions(usize),
    UnsupportedTreeDepth(usize),
    NoFeasibleTrainingExamples,
    Oracle(ProtocolError),
    RuleSearch(RuleError),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AxisBoxDiscoveryResult {
    pub model: BoxCandidate,
    pub submission: CandidateSubmission,
    pub development_observations_used: usize,
    pub queried_sample_ids: Vec<usize>,
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

/// Evenly samples a deterministic subset of visible feature rows.
fn select_budgeted_rows(
    development: &[DevelopmentInput],
    budget: usize,
) -> Vec<&DevelopmentInput> {
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
/// spaced, deterministic subset of development feature rows. Each label is
/// fetched separately from the budget-enforcing oracle; the candidate view has
/// no label field and cannot read labels without consuming a query.
///
/// Tie-breaking is fixed: fewer errors, then lower feature index, then
/// GreaterOrEqual before LessOrEqual, then lower threshold. Holdout inputs are
/// predicted exactly once each. The observation budget does not represent
/// wall-clock or compute budget; the trusted runner must measure those separately.
pub fn discover_univariate_threshold(
    view: &CandidateView,
    oracle: &mut DevelopmentOracle,
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

    let selected_inputs = select_budgeted_rows(&view.development, max_labeled_observations);
    let mut training_rows = Vec::with_capacity(selected_inputs.len());
    for input in &selected_inputs {
        training_rows.push(
            oracle
                .query(input.sample_id)
                .map_err(DiscoveryError::Oracle)?,
        );
    }
    let selected = training_rows.iter().collect::<Vec<_>>();
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
    let queried_sample_ids = training_rows
        .iter()
        .map(|row| row.sample_id)
        .collect::<Vec<_>>();
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

/// Exhaustively searches the axis-aligned box hypothesis class over the
/// observed development rows. This reference learner is intentionally limited
/// to one or two dimensions; it is useful for measuring the hypothesis-class
/// ceiling and is not a scalable substitute for PRIM/CART.
///
/// Development labels are retrieved one-by-one through the budgeted oracle.
/// Selection minimizes development errors, then rule complexity, false positives,
/// and predicted-positive count, with deterministic candidate order as the final
/// tie-break. The returned prediction covers each holdout input exactly once.
pub fn discover_exhaustive_axis_box(
    view: &CandidateView,
    oracle: &mut DevelopmentOracle,
    max_labeled_observations: usize,
) -> Result<AxisBoxDiscoveryResult, DiscoveryError> {
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
    if dimensions > 2 {
        return Err(DiscoveryError::UnsupportedDimensions(dimensions));
    }

    let selected_inputs = select_budgeted_rows(&view.development, max_labeled_observations);
    let mut training_rows = Vec::with_capacity(selected_inputs.len());
    for input in &selected_inputs {
        training_rows.push(
            oracle
                .query(input.sample_id)
                .map_err(DiscoveryError::Oracle)?,
        );
    }
    if !training_rows.iter().any(|row| row.feasible) {
        return Err(DiscoveryError::NoFeasibleTrainingExamples);
    }

    // Latent truth is deliberately a constant and the search is scored on
    // observed labels only. Holdout targets never enter this candidate object.
    let training_points = training_rows
        .iter()
        .map(|row| Point {
            features: row.features.clone(),
            feasible: row.feasible,
            latent_failure: false,
            observed_failure: row.observed_failure,
        })
        .collect::<Vec<_>>();
    let candidates = exhaustive_axis_aligned_boxes(
        &training_points,
        LabelSource::ObservedLabel,
    )
    .map_err(DiscoveryError::RuleSearch)?;
    let model = candidates
        .into_iter()
        .min_by_key(|candidate| {
            (
                candidate.score.false_positive + candidate.score.false_negative,
                candidate.complexity,
                candidate.score.false_positive,
                candidate.predicted_failure_points,
            )
        })
        .ok_or(DiscoveryError::NoFeasibleTrainingExamples)?;

    let predictions = view
        .holdout_inputs
        .iter()
        .map(|input| Prediction {
            sample_id: input.sample_id,
            predicts_failure: model.matches(&input.features),
        })
        .collect();
    let queried_sample_ids = training_rows
        .iter()
        .map(|row| row.sample_id)
        .collect::<Vec<_>>();
    let submission = CandidateSubmission {
        method_id: "cecb.exhaustive_axis_box_1d2d".to_owned(),
        method_version: "0.1.0".to_owned(),
        predictions,
    };

    Ok(AxisBoxDiscoveryResult {
        model,
        submission,
        development_observations_used: queried_sample_ids.len(),
        queried_sample_ids,
    })
}

/// A small, deterministic classification tree. Split predicates are
/// `feature[feature_index] <= threshold`; missing feature values fail closed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CartNode {
    Leaf {
        predicts_failure: bool,
    },
    Split {
        feature_index: usize,
        threshold: i32,
        left: Box<CartNode>,
        right: Box<CartNode>,
    },
}

impl CartNode {
    pub fn predict(&self, features: &[i32]) -> Option<bool> {
        match self {
            Self::Leaf { predicts_failure } => Some(*predicts_failure),
            Self::Split {
                feature_index,
                threshold,
                left,
                right,
            } => {
                let value = *features.get(*feature_index)?;
                if value <= *threshold {
                    left.predict(features)
                } else {
                    right.predict(features)
                }
            }
        }
    }

    pub fn complexity(&self) -> usize {
        match self {
            Self::Leaf { .. } => 0,
            Self::Split { left, right, .. } => 1 + left.complexity() + right.complexity(),
        }
    }
}

/// CART-style axis-aligned decision tree fit by Gini reduction on queried labels.
/// This is a compact reference implementation, not a claim of equivalence to a
/// particular vendor's CART implementation or default hyperparameters.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CartTree {
    pub root: CartNode,
    pub feature_count: usize,
    pub max_depth: usize,
    pub development_errors: usize,
    pub development_feasible_examples: usize,
}

impl CartTree {
    pub fn predict(&self, features: &[i32]) -> Option<bool> {
        if features.len() != self.feature_count {
            return None;
        }
        self.root.predict(features)
    }

    pub fn complexity(&self) -> usize {
        self.root.complexity()
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CartDiscoveryResult {
    pub model: CartTree,
    pub submission: CandidateSubmission,
    pub development_observations_used: usize,
    pub queried_sample_ids: Vec<usize>,
}

fn gini_impurity(positive: usize, negative: usize) -> f64 {
    let total = positive + negative;
    if total == 0 {
        return 0.0;
    }
    let positive_fraction = positive as f64 / total as f64;
    let negative_fraction = negative as f64 / total as f64;
    1.0 - positive_fraction * positive_fraction - negative_fraction * negative_fraction
}

fn leaf_majority(rows: &[&TrainingObservation]) -> bool {
    let positives = rows.iter().filter(|row| row.observed_failure).count();
    positives * 2 > rows.len()
}

fn fit_cart_node(
    rows: &[&TrainingObservation],
    feature_count: usize,
    depth: usize,
    max_depth: usize,
) -> CartNode {
    let positives = rows.iter().filter(|row| row.observed_failure).count();
    let negatives = rows.len().saturating_sub(positives);
    let majority = positives * 2 > rows.len();
    if rows.is_empty() || depth >= max_depth || positives == 0 || negatives == 0 {
        return CartNode::Leaf {
            predicts_failure: majority,
        };
    }

    let parent_impurity = gini_impurity(positives, negatives);
    let mut best: Option<(f64, usize, i32)> = None;

    for feature_index in 0..feature_count {
        let mut thresholds = rows
            .iter()
            .map(|row| row.features[feature_index])
            .collect::<Vec<_>>();
        thresholds.sort_unstable();
        thresholds.dedup();

        for threshold in thresholds {
            let left = rows
                .iter()
                .copied()
                .filter(|row| row.features[feature_index] <= threshold)
                .collect::<Vec<_>>();
            let right = rows
                .iter()
                .copied()
                .filter(|row| row.features[feature_index] > threshold)
                .collect::<Vec<_>>();
            if left.is_empty() || right.is_empty() {
                continue;
            }

            let left_positive = left.iter().filter(|row| row.observed_failure).count();
            let right_positive = right.iter().filter(|row| row.observed_failure).count();
            let left_negative = left.len() - left_positive;
            let right_negative = right.len() - right_positive;
            let count = rows.len() as f64;
            let weighted_impurity = (left.len() as f64 / count)
                * gini_impurity(left_positive, left_negative)
                + (right.len() as f64 / count)
                    * gini_impurity(right_positive, right_negative);
            let gain = parent_impurity - weighted_impurity;

            let better = best.as_ref().map_or(true, |(best_gain, best_feature, best_threshold)| {
                gain > *best_gain + 1.0e-12
                    || ((gain - *best_gain).abs() <= 1.0e-12
                        && (feature_index, threshold) < (*best_feature, *best_threshold))
            });
            if better {
                best = Some((gain, feature_index, threshold));
            }
        }
    }

    let Some((gain, feature_index, threshold)) = best else {
        return CartNode::Leaf {
            predicts_failure: majority,
        };
    };
    if gain <= 1.0e-12 {
        return CartNode::Leaf {
            predicts_failure: majority,
        };
    }

    let left = rows
        .iter()
        .copied()
        .filter(|row| row.features[feature_index] <= threshold)
        .collect::<Vec<_>>();
    let right = rows
        .iter()
        .copied()
        .filter(|row| row.features[feature_index] > threshold)
        .collect::<Vec<_>>();
    CartNode::Split {
        feature_index,
        threshold,
        left: Box::new(fit_cart_node(
            &left,
            feature_count,
            depth + 1,
            max_depth,
        )),
        right: Box::new(fit_cart_node(
            &right,
            feature_count,
            depth + 1,
            max_depth,
        )),
    }
}

/// Fits a bounded CART-style tree using only labels returned by the development
/// oracle. At most `max_labeled_observations` oracle requests are made, and the
/// trusted harness remains responsible for process isolation and timing/memory
/// accounting. A maximum depth of 0 is the majority-class baseline; depths above
/// 16 fail closed to keep exhaustive fixture runs bounded.
pub fn discover_cart_tree(
    view: &CandidateView,
    oracle: &mut DevelopmentOracle,
    max_labeled_observations: usize,
    max_depth: usize,
) -> Result<CartDiscoveryResult, DiscoveryError> {
    if view.development.is_empty() {
        return Err(DiscoveryError::EmptyDevelopment);
    }
    if view.holdout_inputs.is_empty() {
        return Err(DiscoveryError::EmptyHoldout);
    }
    if max_labeled_observations == 0 {
        return Err(DiscoveryError::ZeroObservationBudget);
    }
    if max_depth > 16 {
        return Err(DiscoveryError::UnsupportedTreeDepth(max_depth));
    }

    let feature_count = view.development[0].features.len();
    if feature_count == 0
        || view.development.iter().any(|row| row.features.len() != feature_count)
        || view.holdout_inputs.iter().any(|row| row.features.len() != feature_count)
    {
        return Err(DiscoveryError::InconsistentDimensions);
    }

    let selected_inputs = select_budgeted_rows(&view.development, max_labeled_observations);
    let mut training_rows = Vec::with_capacity(selected_inputs.len());
    for input in &selected_inputs {
        training_rows.push(oracle.query(input.sample_id).map_err(DiscoveryError::Oracle)?);
    }
    let feasible_rows = training_rows
        .iter()
        .filter(|row| row.feasible)
        .collect::<Vec<_>>();
    if feasible_rows.is_empty() {
        return Err(DiscoveryError::NoFeasibleTrainingExamples);
    }

    let root = fit_cart_node(&feasible_rows, feature_count, 0, max_depth);
    let development_errors = feasible_rows
        .iter()
        .filter(|row| root.predict(&row.features) != Some(row.observed_failure))
        .count();
    let model = CartTree {
        root,
        feature_count,
        max_depth,
        development_errors,
        development_feasible_examples: feasible_rows.len(),
    };
    let predictions = view
        .holdout_inputs
        .iter()
        .map(|input| {
            Ok(Prediction {
                sample_id: input.sample_id,
                predicts_failure: model
                    .predict(&input.features)
                    .ok_or(DiscoveryError::InconsistentDimensions)?,
            })
        })
        .collect::<Result<Vec<_>, DiscoveryError>>()?;
    let queried_sample_ids = training_rows
        .iter()
        .map(|row| row.sample_id)
        .collect::<Vec<_>>();
    let submission = CandidateSubmission {
        method_id: "cecb.cart_gini_depth_limited".to_owned(),
        method_version: "0.1.0".to_owned(),
        predictions,
    };

    Ok(CartDiscoveryResult {
        model,
        submission,
        development_observations_used: queried_sample_ids.len(),
        queried_sample_ids,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{build_model_family_split, FamilyTaggedPoint, ProtocolSplit};
    use crate::{
        axis_aligned_landscape, structural_family_a, structural_family_b, LabelSource,
    };

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

    fn fit(
        split: &mut ProtocolSplit,
        budget: usize,
    ) -> Result<DiscoveryResult, DiscoveryError> {
        let view = split.candidate_view().clone();
        discover_univariate_threshold(&view, split.development_oracle_mut(), budget)
    }

    fn fit_cart(
        split: &mut ProtocolSplit,
        budget: usize,
        max_depth: usize,
    ) -> Result<CartDiscoveryResult, DiscoveryError> {
        let view = split.candidate_view().clone();
        discover_cart_tree(
            &view,
            split.development_oracle_mut(),
            budget,
            max_depth,
        )
    }

    #[test]
    fn cart_depth_one_recovers_axis_aligned_truth_with_one_split() {
        let mut split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let result = fit_cart(&mut split, 41, 1).unwrap();
        assert_eq!(result.model.development_errors, 0);
        assert_eq!(result.model.complexity(), 1);
        assert_eq!(result.development_observations_used, 41);
        let report = split.evaluate(&result.submission).unwrap();
        assert_eq!(report.score.true_positive, 13);
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
        assert_eq!(report.score.true_negative, 27);
        assert_eq!(report.candidate_query_count, 41);
    }

    #[test]
    fn cart_depth_three_improves_boundary_coverage_over_stumps() {
        for (points, depth_one_errors, depth_three_errors, expected_tp, expected_fn, expected_tn) in [
            (crate::oblique_landscape(), 9, 1, 8, 4, 28),
            (crate::nonlinear_interaction_landscape(), 8, 3, 5, 3, 32),
        ] {
            let mut stump_split = crate::protocol::build_protocol_split(
                &points,
                LabelSource::LatentTruth,
                41,
            )
            .unwrap();
            let mut tree_split = crate::protocol::build_protocol_split(
                &points,
                LabelSource::LatentTruth,
                41,
            )
            .unwrap();
            let stump = fit_cart(&mut stump_split, 41, 1).unwrap();
            let tree = fit_cart(&mut tree_split, 41, 3).unwrap();
            assert_eq!(stump.model.development_errors, depth_one_errors);
            assert_eq!(tree.model.development_errors, depth_three_errors);
            assert!(tree.model.complexity() > stump.model.complexity());
            let stump_report = stump_split.evaluate(&stump.submission).unwrap();
            let tree_report = tree_split.evaluate(&tree.submission).unwrap();
            assert!(tree_report.score.true_positive > stump_report.score.true_positive);
            assert_eq!(tree_report.score.true_positive, expected_tp);
            assert_eq!(tree_report.score.false_positive, 0);
            assert_eq!(tree_report.score.false_negative, expected_fn);
            assert_eq!(tree_report.score.true_negative, expected_tn);
            assert_eq!(tree_report.candidate_query_count, 41);
        }
    }

    #[test]
    fn cart_tree_is_deterministic_and_fails_on_reversed_family_transfer() {
        let rows = family_fixture();
        let mut first_split = build_model_family_split(
            &rows,
            "family_b",
            LabelSource::LatentTruth,
            9,
        )
        .unwrap();
        let mut second_split = build_model_family_split(
            &rows,
            "family_b",
            LabelSource::LatentTruth,
            9,
        )
        .unwrap();
        let first = fit_cart(&mut first_split, 9, 3).unwrap();
        let second = fit_cart(&mut second_split, 9, 3).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.model.development_errors, 0);
        assert_eq!(first.model.complexity(), 1);
        let report = first_split.evaluate(&first.submission).unwrap();
        assert_eq!(report.score.true_positive, 0);
        assert_eq!(report.score.false_positive, 3);
        assert_eq!(report.score.false_negative, 3);
        assert_eq!(report.score.true_negative, 3);
        assert_eq!(report.candidate_query_count, 9);
    }

    #[test]
    fn cart_depth_and_budget_fail_closed_without_unbounded_queries() {
        let mut depth_split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        let depth_view = depth_split.candidate_view().clone();
        assert_eq!(
            discover_cart_tree(
                &depth_view,
                depth_split.development_oracle_mut(),
                5,
                17
            ),
            Err(DiscoveryError::UnsupportedTreeDepth(17))
        );
        assert_eq!(depth_split.development_query_count(), 0);

        let mut budget_split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            3,
        )
        .unwrap();
        let budget_view = budget_split.candidate_view().clone();
        assert!(matches!(
            discover_cart_tree(
                &budget_view,
                budget_split.development_oracle_mut(),
                9,
                3
            ),
            Err(DiscoveryError::Oracle(ProtocolError::QueryBudgetExceeded {
                supplied: 4,
                maximum: 3
            }))
        ));
        assert_eq!(budget_split.development_query_count(), 3);
    }

    #[test]
    fn exhaustive_box_learner_recovers_axis_aligned_rule() {
        let mut split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let view = split.candidate_view().clone();
        let result = discover_exhaustive_axis_box(
            &view,
            split.development_oracle_mut(),
            41,
        )
        .unwrap();
        assert_eq!(result.model.score.false_positive, 0);
        assert_eq!(result.model.score.false_negative, 0);
        assert_eq!(result.development_observations_used, 41);
        let report = split.evaluate(&result.submission).unwrap();
        assert_eq!(report.candidate_query_count, 41);
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
    }

    #[test]
    fn box_candidate_is_independent_of_latent_label_changes() {
        let original = axis_aligned_landscape();
        let mut relabelled = original.clone();
        for point in &mut relabelled {
            point.latent_failure = !point.latent_failure;
        }
        let mut original_split = crate::protocol::build_protocol_split(
            &original,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let mut relabelled_split = crate::protocol::build_protocol_split(
            &relabelled,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let original_view = original_split.candidate_view().clone();
        let relabelled_view = relabelled_split.candidate_view().clone();
        let first = discover_exhaustive_axis_box(
            &original_view,
            original_split.development_oracle_mut(),
            41,
        )
        .unwrap();
        let second = discover_exhaustive_axis_box(
            &relabelled_view,
            relabelled_split.development_oracle_mut(),
            41,
        )
        .unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn exhaustive_box_learner_fails_closed_above_two_dimensions_before_querying() {
        let mut split = crate::protocol::build_protocol_split(
            &crate::irrelevant_dimensions_landscape(),
            LabelSource::LatentTruth,
            512,
        )
        .unwrap();
        let view = split.candidate_view().clone();
        assert_eq!(
            discover_exhaustive_axis_box(
                &view,
                split.development_oracle_mut(),
                512,
            ),
            Err(DiscoveryError::UnsupportedDimensions(8))
        );
        assert_eq!(split.development_query_count(), 0);
    }

    #[test]
    fn exhaustive_box_learner_fails_on_reversed_family_transfer() {
        let mut split = build_model_family_split(
            &family_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            9,
        )
        .unwrap();
        let view = split.candidate_view().clone();
        let result = discover_exhaustive_axis_box(
            &view,
            split.development_oracle_mut(),
            9,
        )
        .unwrap();
        let report = split.evaluate(&result.submission).unwrap();
        assert_eq!(report.score.true_positive, 0);
        assert_eq!(report.score.false_positive, 3);
        assert_eq!(report.score.false_negative, 3);
        assert_eq!(report.score.true_negative, 3);
        assert_eq!(report.candidate_query_count, 9);
    }

    #[test]
    fn recovers_axis_aligned_rule_on_full_development_set() {
        let mut split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let result = fit(&mut split, 41).unwrap();
        assert_eq!(result.model.feature_index, 0);
        assert_eq!(result.model.direction, ThresholdDirection::GreaterOrEqual);
        assert_eq!(result.model.threshold, 2);
        assert_eq!(result.model.development_errors, 0);
        assert_eq!(result.development_observations_used, 41);
        assert_eq!(split.development_query_count(), 41);
        let report = split
            .evaluate(&result.submission)
            .unwrap();
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
    }

    #[test]
    fn budget_is_respected_and_sample_ids_are_deterministic() {
        let rows = family_fixture();
        let mut first_split = build_model_family_split(
            &rows,
            "family_b",
            LabelSource::LatentTruth,
            3,
        )
        .unwrap();
        let mut second_split = build_model_family_split(
            &rows,
            "family_b",
            LabelSource::LatentTruth,
            3,
        )
        .unwrap();
        let first = fit(&mut first_split, 3).unwrap();
        let second = fit(&mut second_split, 3).unwrap();
        assert_eq!(first, second);
        assert_eq!(first_split.development_query_count(), 3);
        assert_eq!(first.development_observations_used, 3);
        assert_eq!(first.queried_sample_ids, vec![0, 4, 8]);
    }

    #[test]
    fn fitted_rule_fails_on_reversed_structural_family() {
        let mut split = build_model_family_split(
            &family_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            9,
        )
        .unwrap();
        let result = fit(&mut split, 9).unwrap();
        assert_eq!(result.model.direction, ThresholdDirection::GreaterOrEqual);
        assert_eq!(result.model.threshold, 2);
        let report = split
            .evaluate(&result.submission)
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
        let mut original_split = crate::protocol::build_protocol_split(
            &original,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let mut relabelled_split = crate::protocol::build_protocol_split(
            &relabelled,
            LabelSource::ObservedLabel,
            41,
        )
        .unwrap();
        let first = fit(&mut original_split, 41).unwrap();
        let second = fit(&mut relabelled_split, 41).unwrap();
        assert_eq!(first, second);
        assert_eq!(original_split.development_query_count(), 41);
    }

    #[test]
    fn zero_budget_and_empty_views_fail_closed() {
        let mut split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let view = split.candidate_view().clone();
        assert_eq!(
            discover_univariate_threshold(&view, split.development_oracle_mut(), 0),
            Err(DiscoveryError::ZeroObservationBudget)
        );
        let empty = CandidateView {
            development: vec![],
            holdout_inputs: vec![],
            manifest: view.manifest.clone(),
        };
        assert_eq!(
            discover_univariate_threshold(
                &empty,
                split.development_oracle_mut(),
                1
            ),
            Err(DiscoveryError::EmptyDevelopment)
        );
        assert_eq!(split.development_query_count(), 0);
    }

    #[test]
    fn malformed_feature_dimensions_fail_closed() {
        let mut split = crate::protocol::build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            41,
        )
        .unwrap();
        let mut view = split.candidate_view().clone();
        view.holdout_inputs[0].features.pop();
        assert_eq!(
            discover_univariate_threshold(&view, split.development_oracle_mut(), 41),
            Err(DiscoveryError::InconsistentDimensions)
        );
        assert_eq!(split.development_query_count(), 0);
    }
}
