//! Deterministic synthetic landscapes and a known-truth region scorer for CECB.
//!
//! This crate does not implement a scenario-discovery algorithm. It provides
//! independent reference cases and scoring primitives for comparing such
//! algorithms under a fixed evaluator and budget.

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
        Self { feature, lower_inclusive: Some(value), upper_inclusive: None }
    }

    pub fn at_most(feature: usize, value: i32) -> Self {
        Self { feature, lower_inclusive: None, upper_inclusive: Some(value) }
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
    pub fn new(clauses: Vec<Interval>) -> Self { Self { clauses } }

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

/// Scores an axis-aligned conjunction against the known-truth fixture.
/// Infeasible points are excluded from confusion counts and reported separately.
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

    let mut score = Score::default();
    for point in points {
        if !point.feasible {
            score.infeasible_skipped += 1;
            continue;
        }
        let predicted_failure = rule.matches(&point.features);
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

fn point(features: Vec<i32>, feasible: bool, latent_failure: bool, observed_failure: bool) -> Point {
    Point { features, feasible, latent_failure, observed_failure }
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
    grid_2d(|x, _y| { let failure = x >= 2; (failure, failure) })
}

/// 81-point landscape: failure iff x0 + x1 >= 3. An axis-aligned rule
/// cannot represent the true diagonal boundary exactly.
pub fn oblique_landscape() -> Vec<Point> {
    grid_2d(|x, y| { let failure = x + y >= 3; (failure, failure) })
}

/// 81-point landscape: failure iff x0 * x1 >= 6. Treating either feature as
/// independently sufficient creates false positives.
pub fn nonlinear_interaction_landscape() -> Vec<Point> {
    grid_2d(|x, y| { let failure = x * y >= 6; (failure, failure) })
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
            state = state.wrapping_mul(6_364_136_223_846_793_005)
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
    (-4..=4).map(|x| {
        let failure = x >= 2;
        point(vec![x], true, failure, failure)
    }).collect()
}

/// Structural alternative B reverses the response: failure iff x <= -2.
/// High accuracy in family A does not imply transfer to family B.
pub fn structural_family_b() -> Vec<Point> {
    (-4..=4).map(|x| {
        let failure = x <= -2;
        point(vec![x], true, failure, failure)
    }).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_axis_aligned_rule_recovers_known_truth() {
        let points = axis_aligned_landscape();
        assert_eq!(points.len(), 81);
        let score = score_rule(&points, &Rule::feature_at_least(0, 2), LabelSource::LatentTruth).unwrap();
        assert_eq!(score, Score {
            true_positive: 27, false_positive: 0, false_negative: 0,
            true_negative: 54, infeasible_skipped: 0,
        });
        assert_eq!(score.precision_counts(), Some((27, 27)));
        assert_eq!(score.recall_counts(), Some((27, 27)));
    }

    #[test]
    fn oblique_boundary_exposes_axis_rule_tradeoffs() {
        let points = oblique_landscape();
        let score = score_rule(&points, &Rule::feature_at_least(0, 2), LabelSource::LatentTruth).unwrap();
        assert_eq!(score, Score {
            true_positive: 15, false_positive: 12, false_negative: 6,
            true_negative: 48, infeasible_skipped: 0,
        });
        assert_eq!(score.precision_counts(), Some((15, 27)));
        assert_eq!(score.recall_counts(), Some((15, 21)));
    }

    #[test]
    fn nonlinear_interaction_rejects_single_feature_shortcut() {
        let points = nonlinear_interaction_landscape();
        let score = score_rule(&points, &Rule::feature_at_least(0, 2), LabelSource::LatentTruth).unwrap();
        assert_eq!(score, Score {
            true_positive: 8, false_positive: 19, false_negative: 8,
            true_negative: 46, infeasible_skipped: 0,
        });
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
        let score = score_rule(&points, &Rule::feature_at_least(0, 2), LabelSource::LatentTruth).unwrap();
        assert_eq!(score.evaluated_points(), 1024);
    }

    #[test]
    fn infeasible_points_are_excluded_and_counted() {
        let mut points = correlated_feasible_landscape();
        assert_eq!(points.len(), 25);
        assert!(points.iter().all(|p| (p.features[0] - p.features[1]).abs() <= 1));
        points.push(point(vec![4, -4], false, true, true));
        let score = score_rule(&points, &Rule::feature_at_least(0, 2), LabelSource::LatentTruth).unwrap();
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
        assert_eq!(score_a, Score {
            true_positive: 3, false_positive: 0, false_negative: 0,
            true_negative: 6, infeasible_skipped: 0,
        });
        assert_eq!(score_b, Score {
            true_positive: 0, false_positive: 3, false_negative: 3,
            true_negative: 3, infeasible_skipped: 0,
        });
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
        assert_eq!(score_rule(&[], &Rule::new(vec![]), LabelSource::LatentTruth), Err(RuleError::EmptyDataset));
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
