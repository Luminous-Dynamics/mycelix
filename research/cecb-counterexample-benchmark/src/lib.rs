//! Deterministic synthetic landscapes and a known-truth region scorer for CECB.
//!
//! This crate provides known-truth fixtures, independent scoring primitives,
//! a candidate/evaluator protocol, a transparent univariate-threshold baseline,
//! and a bounded exhaustive axis-aligned box learner for one/two dimensions.
//! It is a research benchmark, not a claim that any method is state of the art
//! or that synthetic fixtures establish real-world validity.

pub mod protocol;
pub mod discovery;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Point {
    pub features: Vec<i32>,
    pub feasible: bool,
    pub latent_failure: bool,
    pub observed_failure: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LabelSource {
    LatentTruth,
    ObservedLabel,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Interval {
    pub feature: usize,
    pub lower_inclusive: Option<i32>,
    pub upper_inclusive: Option<i32>,
}

impl Interval {
    pub fn at_least(feature: usize, value: i32) -> Self {
        Self {
            feature,
            lower_inclusive: Some(value),
            upper_inclusive: None,
        }
    }

    pub fn at_most(feature: usize, value: i32) -> Self {
        Self {
            feature,
            lower_inclusive: None,
            upper_inclusive: Some(value),
        }
    }

    fn matches(&self, features: &[i32]) -> bool {
        let value = features[self.feature];
        self.lower_inclusive.map_or(true, |lower| value >= lower)
            && self.upper_inclusive.map_or(true, |upper| value <= upper)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Rule {
    /// Intervals are combined by logical AND. An empty rule predicts failure
    /// for every feasible point and is useful as a deliberately naive baseline.
    pub clauses: Vec<Interval>,
}

impl Rule {
    pub fn new(clauses: Vec<Interval>) -> Self {
        Self { clauses }
    }

    pub fn feature_at_least(feature: usize, value: i32) -> Self {
        Self::new(vec![Interval::at_least(feature, value)])
    }

    fn matches(&self, features: &[i32]) -> bool {
        self.clauses.iter().all(|clause| clause.matches(features))
    }

    fn validate(&self, dimensions: usize) -> Result<(), RuleError> {
        let mut seen = vec![false; dimensions];
        for clause in &self.clauses {
            if clause.feature >= dimensions {
                return Err(RuleError::FeatureOutOfRange {
                    feature: clause.feature,
                    dimensions,
                });
            }
            if seen[clause.feature] {
                return Err(RuleError::DuplicateFeatureClause(clause.feature));
            }
            seen[clause.feature] = true;
            if matches!(
                (clause.lower_inclusive, clause.upper_inclusive),
                (Some(lower), Some(upper)) if lower > upper
            ) {
                return Err(RuleError::InvertedInterval(clause.feature));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RuleError {
    EmptyDataset,
    InconsistentDimensions,
    FeatureOutOfRange { feature: usize, dimensions: usize },
    DuplicateFeatureClause(usize),
    InvertedInterval(usize),
    PredictionLengthMismatch { expected: usize, actual: usize },
    UnsupportedExhaustiveDimensions(usize),
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Score {
    pub true_positive: usize,
    pub false_positive: usize,
    pub false_negative: usize,
    pub true_negative: usize,
    pub infeasible_skipped: usize,
}

impl Score {
    pub fn evaluated_points(&self) -> usize {
        self.true_positive + self.false_positive + self.false_negative + self.true_negative
    }

    pub fn predicted_failure_points(&self) -> usize {
        self.true_positive + self.false_positive
    }

    pub fn known_failure_points(&self) -> usize {
        self.true_positive + self.false_negative
    }

    /// Exact numerator/denominator for precision. Undefined 0/0 remains None.
    pub fn precision_counts(&self) -> Option<(usize, usize)> {
        let denominator = self.predicted_failure_points();
        (denominator != 0).then_some((self.true_positive, denominator))
    }

    /// Exact numerator/denominator for recall. Undefined 0/0 remains None.
    pub fn recall_counts(&self) -> Option<(usize, usize)> {
        let denominator = self.known_failure_points();
        (denominator != 0).then_some((self.true_positive, denominator))
    }
}

/// Scores an arbitrary discovery algorithm's binary predictions against the
/// known-truth fixture. Predictions must be aligned one-to-one with input points;
/// infeasible points are excluded from confusion counts and counted separately.
pub fn score_predictions(
    points: &[Point],
    predictions: &[bool],
    label_source: LabelSource,
) -> Result<Score, RuleError> {
    if points.is_empty() {
        return Err(RuleError::EmptyDataset);
    }
    if predictions.len() != points.len() {
        return Err(RuleError::PredictionLengthMismatch {
            expected: points.len(),
            actual: predictions.len(),
        });
    }
    let dimensions = points[0].features.len();
    if dimensions == 0 || points.iter().any(|point| point.features.len() != dimensions) {
        return Err(RuleError::InconsistentDimensions);
    }

    let mut score = Score::default();
    for (point, predicted_failure) in points.iter().zip(predictions.iter().copied()) {
        if !point.feasible {
            score.infeasible_skipped += 1;
            continue;
        }
        let actual_failure = match label_source {
            LabelSource::LatentTruth => point.latent_failure,
            LabelSource::ObservedLabel => point.observed_failure,
        };
        match (predicted_failure, actual_failure) {
            (true, true) => score.true_positive += 1,
            (true, false) => score.false_positive += 1,
            (false, true) => score.false_negative += 1,
            (false, false) => score.true_negative += 1,
        }
    }
    Ok(score)
}

/// Scores an interpretable axis-aligned conjunction against the fixture. This is
/// a baseline convenience wrapper; arbitrary candidate methods should use
/// score_predictions and supply one prediction per point.
pub fn score_rule(
    points: &[Point],
    rule: &Rule,
    label_source: LabelSource,
) -> Result<Score, RuleError> {
    let dimensions = points.first().ok_or(RuleError::EmptyDataset)?.features.len();
    if dimensions == 0 || points.iter().any(|point| point.features.len() != dimensions) {
        return Err(RuleError::InconsistentDimensions);
    }
    rule.validate(dimensions)?;
    let predictions = points
        .iter()
        .map(|point| rule.matches(&point.features))
        .collect::<Vec<_>>();
    score_predictions(points, &predictions, label_source)
}

/// One condition for an axis-aligned box. `Never` is an explicit empty
/// region, not a malformed interval.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FeatureCondition {
    Any,
    Never,
    Interval(Interval),
}

impl FeatureCondition {
    fn matches(&self, value: i32) -> bool {
        match self {
            Self::Any => true,
            Self::Never => false,
            Self::Interval(interval) => {
                interval.lower_inclusive.map_or(true, |lower| value >= lower)
                    && interval.upper_inclusive.map_or(true, |upper| value <= upper)
            }
        }
    }

    fn complexity(&self) -> usize {
        match self {
            Self::Any => 0,
            Self::Never | Self::Interval(_) => 1,
        }
    }
}

/// An exhaustively enumerated axis-aligned box and its evaluator score.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoxCandidate {
    pub conditions: Vec<FeatureCondition>,
    pub score: Score,
    pub predicted_failure_points: usize,
    pub complexity: usize,
}

impl BoxCandidate {
    pub fn matches(&self, features: &[i32]) -> bool {
        features.len() == self.conditions.len()
            && self.conditions.iter().zip(features.iter().copied())
                .all(|(condition, value)| condition.matches(value))
    }

    pub fn dominates(&self, other: &Self) -> bool {
        self.score.true_positive >= other.score.true_positive
            && self.score.false_positive <= other.score.false_positive
            && self.complexity <= other.complexity
            && (self.score.true_positive > other.score.true_positive
                || self.score.false_positive < other.score.false_positive
                || self.complexity < other.complexity)
    }
}

/// Enumerates the extensional hypothesis class of axis-aligned boxes for tiny
/// known-truth spaces (one or two dimensions). It is an oracle baseline, not a
/// scalable discovery method. Equivalent boxes over the supplied points are
/// deduplicated by their prediction vector, retaining the least-complex form.
pub fn exhaustive_axis_aligned_boxes(
    points: &[Point],
    label_source: LabelSource,
) -> Result<Vec<BoxCandidate>, RuleError> {
    if points.is_empty() {
        return Err(RuleError::EmptyDataset);
    }
    let dimensions = points[0].features.len();
    if dimensions == 0 || points.iter().any(|point| point.features.len() != dimensions) {
        return Err(RuleError::InconsistentDimensions);
    }
    if dimensions > 2 {
        return Err(RuleError::UnsupportedExhaustiveDimensions(dimensions));
    }

    fn condition_options(points: &[Point], feature: usize) -> Vec<FeatureCondition> {
        use std::collections::BTreeMap;
        let mut values = points.iter()
            .map(|point| point.features[feature])
            .collect::<Vec<_>>();
        values.sort_unstable();
        values.dedup();

        let mut options = Vec::new();
        options.push(FeatureCondition::Any);
        options.push(FeatureCondition::Never);
        for &value in &values {
            options.push(FeatureCondition::Interval(Interval::at_least(feature, value)));
            options.push(FeatureCondition::Interval(Interval::at_most(feature, value)));
        }
        for (left_index, &lower) in values.iter().enumerate() {
            for &upper in values.iter().skip(left_index) {
                options.push(FeatureCondition::Interval(Interval {
                    feature,
                    lower_inclusive: Some(lower),
                    upper_inclusive: Some(upper),
                }));
            }
        }

        let mut unique = BTreeMap::<Vec<bool>, FeatureCondition>::new();
        for condition in options {
            let mask = points.iter()
                .map(|point| condition.matches(point.features[feature]))
                .collect::<Vec<_>>();
            unique.entry(mask).or_insert(condition);
        }
        unique.into_values().collect()
    }

    let options = (0..dimensions)
        .map(|feature| condition_options(points, feature))
        .collect::<Vec<_>>();
    let mut unique = std::collections::BTreeMap::<Vec<bool>, BoxCandidate>::new();

    if dimensions == 1 {
        for first in &options[0] {
            let conditions = vec![first.clone()];
            let predictions = points.iter()
                .map(|point| conditions[0].matches(point.features[0]))
                .collect::<Vec<_>>();
            insert_candidate(
                &mut unique,
                points,
                conditions,
                predictions,
                label_source,
            )?;
        }
    } else {
        for first in &options[0] {
            for second in &options[1] {
                let conditions = vec![first.clone(), second.clone()];
                let predictions = points.iter()
                    .map(|point| {
                        conditions[0].matches(point.features[0])
                            && conditions[1].matches(point.features[1])
                    })
                    .collect::<Vec<_>>();
                insert_candidate(
                    &mut unique,
                    points,
                    conditions,
                    predictions,
                    label_source,
                )?;
            }
        }
    }

    let mut candidates = unique.into_values().collect::<Vec<_>>();
    candidates.sort_by_key(|candidate| (
        candidate.complexity,
        candidate.score.false_positive,
        std::cmp::Reverse(candidate.score.true_positive),
        candidate.predicted_failure_points,
    ));
    Ok(candidates)
}

fn insert_candidate(
    unique: &mut std::collections::BTreeMap<Vec<bool>, BoxCandidate>,
    points: &[Point],
    conditions: Vec<FeatureCondition>,
    predictions: Vec<bool>,
    label_source: LabelSource,
) -> Result<(), RuleError> {
    let score = score_predictions(points, &predictions, label_source)?;
    let complexity = conditions.iter().map(FeatureCondition::complexity).sum();
    let predicted_failure_points = predictions.iter()
        .zip(points.iter())
        .filter(|(prediction, point)| **prediction && point.feasible)
        .count();
    let candidate = BoxCandidate {
        conditions,
        score,
        predicted_failure_points,
        complexity,
    };
    match unique.entry(predictions) {
        std::collections::btree_map::Entry::Vacant(entry) => {
            entry.insert(candidate);
        }
        std::collections::btree_map::Entry::Occupied(mut entry) => {
            if candidate.complexity < entry.get().complexity {
                entry.insert(candidate);
            }
        }
    }
    Ok(())
}

/// Returns the non-dominated candidate subset under three separate criteria:
/// maximize true positives, minimize false positives, and minimize rule
/// complexity. No cross-unit weighted sum is formed.
pub fn pareto_box_front(candidates: &[BoxCandidate]) -> Vec<BoxCandidate> {
    candidates.iter()
        .filter(|candidate| !candidates.iter().any(|other| other.dominates(candidate)))
        .cloned()
        .collect()
}

fn point(
    features: Vec<i32>,
    feasible: bool,
    latent_failure: bool,
    observed_failure: bool,
) -> Point {
    Point {
        features,
        feasible,
        latent_failure,
        observed_failure,
    }
}

fn grid_2d(mut label: impl FnMut(i32, i32) -> (bool, bool)) -> Vec<Point> {
    let mut points = Vec::with_capacity(81);
    for x in -4..=4 {
        for y in -4..=4 {
            let (latent, observed) = label(x, y);
            points.push(point(vec![x, y], true, latent, observed));
        }
    }
    points
}

/// 81-point landscape: failure iff x0 >= 2. An exact axis rule exists.
pub fn axis_aligned_landscape() -> Vec<Point> {
    grid_2d(|x, _y| {
        let failure = x >= 2;
        (failure, failure)
    })
}

/// 81-point landscape: failure iff x0 + x1 >= 3. An axis-aligned rule
/// cannot represent the true diagonal boundary exactly.
pub fn oblique_landscape() -> Vec<Point> {
    grid_2d(|x, y| {
        let failure = x + y >= 3;
        (failure, failure)
    })
}

/// 81-point landscape: failure iff x0 * x1 >= 6. Treating either feature as
/// independently sufficient creates false positives.
pub fn nonlinear_interaction_landscape() -> Vec<Point> {
    grid_2d(|x, y| {
        let failure = x * y >= 6;
        (failure, failure)
    })
}

/// Deterministic noisy labels around an oblique latent boundary. The fixture is
/// reproducible and records latent truth and observed labels separately.
pub fn noisy_boundary_landscape() -> Vec<Point> {
    grid_2d(|x, y| {
        let latent = x + y >= 3;
        let noise = (x * 7 + y * 3 + 31).rem_euclid(11) == 0;
        (latent, latent ^ noise)
    })
}

/// Deterministic 8-dimensional sample with six irrelevant coordinates.
/// Failure depends only on x0 + x1 >= 3; this is not a real-world frequency model.
pub fn irrelevant_dimensions_landscape() -> Vec<Point> {
    let mut state = 0x4d59_5df4_d0f3_3173_u64;
    let mut points = Vec::with_capacity(1024);
    for _ in 0..1024 {
        let mut features = Vec::with_capacity(8);
        for _ in 0..8 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            features.push(((state >> 32) % 9) as i32 - 4);
        }
        let failure = features[0] + features[1] >= 3;
        points.push(point(features, true, failure, failure));
    }
    points
}

/// 25 feasible points from a 9x9 grid restricted by |x0 - x1| <= 1.
/// Failure iff x0 >= 2. Infeasible points are not part of this generated set.
pub fn correlated_feasible_landscape() -> Vec<Point> {
    let mut points = Vec::with_capacity(25);
    for x in -4..=4 {
        for y in -4..=4 {
            if (x - y).abs() <= 1 {
                let failure = x >= 2;
                points.push(point(vec![x, y], true, failure, failure));
            }
        }
    }
    points
}

/// Structural alternative A: failure iff x >= 2.
pub fn structural_family_a() -> Vec<Point> {
    (-4..=4)
        .map(|x| {
            let failure = x >= 2;
            point(vec![x], true, failure, failure)
        })
        .collect()
}

/// Structural alternative B reverses the response: failure iff x <= -2.
/// High accuracy in family A does not imply transfer to family B.
pub fn structural_family_b() -> Vec<Point> {
    (-4..=4)
        .map(|x| {
            let failure = x <= -2;
            point(vec![x], true, failure, failure)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_axis_aligned_rule_recovers_known_truth() {
        let points = axis_aligned_landscape();
        assert_eq!(points.len(), 81);
        let score = score_rule(
            &points,
            &Rule::feature_at_least(0, 2),
            LabelSource::LatentTruth,
        )
        .unwrap();
        assert_eq!(
            score,
            Score {
                true_positive: 27,
                false_positive: 0,
                false_negative: 0,
                true_negative: 54,
                infeasible_skipped: 0,
            }
        );
        assert_eq!(score.precision_counts(), Some((27, 27)));
        assert_eq!(score.recall_counts(), Some((27, 27)));
    }

    #[test]
    fn oblique_boundary_exposes_axis_rule_tradeoffs() {
        let points = oblique_landscape();
        let score = score_rule(
            &points,
            &Rule::feature_at_least(0, 2),
            LabelSource::LatentTruth,
        )
        .unwrap();
        assert_eq!(
            score,
            Score {
                true_positive: 15,
                false_positive: 12,
                false_negative: 6,
                true_negative: 48,
                infeasible_skipped: 0,
            }
        );
        assert_eq!(score.precision_counts(), Some((15, 27)));
        assert_eq!(score.recall_counts(), Some((15, 21)));
    }

    #[test]
    fn nonlinear_interaction_rejects_single_feature_shortcut() {
        let points = nonlinear_interaction_landscape();
        let score = score_rule(
            &points,
            &Rule::feature_at_least(0, 2),
            LabelSource::LatentTruth,
        )
        .unwrap();
        assert_eq!(
            score,
            Score {
                true_positive: 8,
                false_positive: 19,
                false_negative: 8,
                true_negative: 46,
                infeasible_skipped: 0,
            }
        );
    }

    #[test]
    fn arbitrary_predictions_can_recover_oblique_and_nonlinear_oracles() {
        let oblique = oblique_landscape();
        let oblique_predictions = oblique
            .iter()
            .map(|p| p.features[0] + p.features[1] >= 3)
            .collect::<Vec<_>>();
        assert_eq!(
            score_predictions(&oblique, &oblique_predictions, LabelSource::LatentTruth).unwrap(),
            Score {
                true_positive: 21,
                false_positive: 0,
                false_negative: 0,
                true_negative: 60,
                infeasible_skipped: 0,
            }
        );

        let nonlinear = nonlinear_interaction_landscape();
        let nonlinear_predictions = nonlinear
            .iter()
            .map(|p| p.features[0] * p.features[1] >= 6)
            .collect::<Vec<_>>();
        assert_eq!(
            score_predictions(
                &nonlinear,
                &nonlinear_predictions,
                LabelSource::LatentTruth,
            )
            .unwrap(),
            Score {
                true_positive: 16,
                false_positive: 0,
                false_negative: 0,
                true_negative: 65,
                infeasible_skipped: 0,
            }
        );
    }

    #[test]
    fn noisy_labels_are_deterministic_and_distinct_from_latent_truth() {
        let first = noisy_boundary_landscape();
        assert_eq!(first, noisy_boundary_landscape());
        assert_eq!(first.len(), 81);
        assert!(first.iter().any(|p| p.latent_failure != p.observed_failure));
        assert!(first.iter().any(|p| p.latent_failure == p.observed_failure));
        let rule = Rule::feature_at_least(0, 2);
        let latent = score_rule(&first, &rule, LabelSource::LatentTruth).unwrap();
        let observed = score_rule(&first, &rule, LabelSource::ObservedLabel).unwrap();
        assert_ne!(latent, observed);
    }

    #[test]
    fn irrelevant_dimensions_do_not_change_ground_truth_semantics() {
        let points = irrelevant_dimensions_landscape();
        assert_eq!(points.len(), 1024);
        assert!(points.iter().all(|p| p.features.len() == 8));
        for p in &points {
            assert_eq!(p.latent_failure, p.features[0] + p.features[1] >= 3);
            assert_eq!(p.observed_failure, p.latent_failure);
        }
        let score = score_rule(
            &points,
            &Rule::feature_at_least(0, 2),
            LabelSource::LatentTruth,
        )
        .unwrap();
        assert_eq!(score.evaluated_points(), 1024);
    }

    #[test]
    fn infeasible_points_are_excluded_and_counted() {
        let mut points = correlated_feasible_landscape();
        assert_eq!(points.len(), 25);
        assert!(points.iter().all(|p| (p.features[0] - p.features[1]).abs() <= 1));
        points.push(point(vec![4, -4], false, true, true));
        let score = score_rule(
            &points,
            &Rule::feature_at_least(0, 2),
            LabelSource::LatentTruth,
        )
        .unwrap();
        assert_eq!(score.infeasible_skipped, 1);
        assert_eq!(score.evaluated_points(), 25);
        assert_eq!(score.true_positive, 8);
        assert_eq!(score.false_positive, 0);
        assert_eq!(score.false_negative, 0);
        assert_eq!(score.true_negative, 17);
    }

    #[test]
    fn structural_reversal_blocks_naive_transfer_claim() {
        let rule = Rule::feature_at_least(0, 2);
        let score_a = score_rule(&structural_family_a(), &rule, LabelSource::LatentTruth).unwrap();
        let score_b = score_rule(&structural_family_b(), &rule, LabelSource::LatentTruth).unwrap();
        assert_eq!(
            score_a,
            Score {
                true_positive: 3,
                false_positive: 0,
                false_negative: 0,
                true_negative: 6,
                infeasible_skipped: 0,
            }
        );
        assert_eq!(
            score_b,
            Score {
                true_positive: 0,
                false_positive: 3,
                false_negative: 3,
                true_negative: 3,
                infeasible_skipped: 0,
            }
        );
    }

    #[test]
    fn malformed_regions_fail_closed() {
        let points = axis_aligned_landscape();
        let duplicate_clause = Rule::new(vec![Interval::at_least(0, 1), Interval::at_most(0, 3)]);
        assert_eq!(
            score_rule(&points, &duplicate_clause, LabelSource::LatentTruth),
            Err(RuleError::DuplicateFeatureClause(0))
        );
        let inverted_interval = Rule::new(vec![Interval {
            feature: 0, lower_inclusive: Some(4), upper_inclusive: Some(2),
        }]);
        assert_eq!(
            score_rule(&points, &inverted_interval, LabelSource::LatentTruth),
            Err(RuleError::InvertedInterval(0))
        );
        let out_of_range = Rule::feature_at_least(2, 0);
        assert_eq!(
            score_rule(&points, &out_of_range, LabelSource::LatentTruth),
            Err(RuleError::FeatureOutOfRange { feature: 2, dimensions: 2 })
        );
        assert_eq!(
            score_rule(&[], &Rule::new(vec![]), LabelSource::LatentTruth),
            Err(RuleError::EmptyDataset)
        );
        assert_eq!(
            score_predictions(&points, &[], LabelSource::LatentTruth),
            Err(RuleError::PredictionLengthMismatch { expected: 81, actual: 0 })
        );
    }

    #[test]
    fn exhaustive_boxes_recover_axis_aligned_oracle_and_keep_pareto_tradeoffs() {
        let points = axis_aligned_landscape();
        let candidates = exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth).unwrap();
        assert!(!candidates.is_empty());
        assert!(candidates.iter().any(|candidate| {
            candidate.score.true_positive == 27
                && candidate.score.false_positive == 0
                && candidate.score.false_negative == 0
                && candidate.score.true_negative == 54
        }));
        assert_eq!(
            candidates,
            exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth).unwrap()
        );
        let front = pareto_box_front(&candidates);
        assert!(!front.is_empty());
        assert!(front.iter().all(|candidate| {
            !front.iter().any(|other| other.dominates(candidate))
        }));
        assert!(front.iter().any(|candidate| {
            candidate.score.true_positive == 27
                && candidate.score.false_positive == 0
                && candidate.complexity == 1
        }));
    }

    #[test]
    fn exhaustive_boxes_cannot_claim_exact_axis_box_for_oblique_truth() {
        let points = oblique_landscape();
        let candidates = exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth).unwrap();
        assert!(!candidates.iter().any(|candidate| {
            candidate.score.false_positive == 0 && candidate.score.false_negative == 0
        }));
    }

    #[test]
    fn exhaustive_box_search_recovers_no_exact_rule_for_interacting_truth() {
        let points = nonlinear_interaction_landscape();
        let candidates =
            exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth).unwrap();
        assert!(!candidates.iter().any(|candidate| {
            candidate.score.false_positive == 0 && candidate.score.false_negative == 0
        }));
        assert_eq!(
            candidates,
            exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth).unwrap()
        );
    }

    #[test]
    fn exhaustive_boxes_fail_closed_above_two_dimensions() {
        let points = irrelevant_dimensions_landscape();
        assert_eq!(
            exhaustive_axis_aligned_boxes(&points, LabelSource::LatentTruth),
            Err(RuleError::UnsupportedExhaustiveDimensions(8))
        );
    }

    #[test]
    fn empty_rule_is_explicit_all_positive_baseline() {
        let points = axis_aligned_landscape();
        let score = score_rule(&points, &Rule::new(vec![]), LabelSource::LatentTruth).unwrap();
        assert_eq!(score.true_positive, 27);
        assert_eq!(score.false_positive, 54);
        assert_eq!(score.false_negative, 0);
        assert_eq!(score.true_negative, 0);
        assert_eq!(score.precision_counts(), Some((27, 81)));
        assert_eq!(score.recall_counts(), Some((27, 27)));
    }
}
