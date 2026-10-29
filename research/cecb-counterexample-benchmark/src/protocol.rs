//! Candidate/evaluator separation for the CECB synthetic benchmark.
//!
//! This module prevents accidental label leakage through the public API: candidate
//! discovery receives observed-label development rows and feature-only holdout
//! inputs; latent/observed holdout targets remain private to the evaluator.
//! It is an API boundary, NOT a security sandbox. Run untrusted candidates in an
//! isolated process and have the trusted harness measure query counts externally.

use crate::{LabelSource, Point, Score};
use std::collections::{BTreeMap, BTreeSet};

const SCHEMA_VERSION: &str = "cecb.candidate_holdout_protocol.v0.1";
const SPLIT_RULE: &str = "alternating_input_index_v0.1";

/// One development example. Candidate code sees observed labels, never latent truth.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrainingObservation {
    pub sample_id: usize,
    pub model_family_id: Option<String>,
    pub features: Vec<i32>,
    pub feasible: bool,
    pub observed_failure: bool,
}

/// One holdout input. Deliberately contains no observed or latent outcome label.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidateInput {
    pub sample_id: usize,
    pub model_family_id: Option<String>,
    pub features: Vec<i32>,
    pub feasible: bool,
}

/// A candidate's prediction for one holdout sample.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Prediction {
    pub sample_id: usize,
    pub predicts_failure: bool,
}

/// Output contract for a discovery method. Query counts are supplied separately
/// to the evaluator by the trusted harness; candidate-reported counts are not
/// accepted as evidence of budget compliance.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidateSubmission {
    pub method_id: String,
    pub method_version: String,
    pub predictions: Vec<Prediction>,
}

/// Fixed, reviewable split description. IDs refer to original input order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SplitManifest {
    pub schema_version: &'static str,
    pub split_rule: &'static str,
    pub development_sample_ids: Vec<usize>,
    pub holdout_sample_ids: Vec<usize>,
    pub holdout_family_id: Option<String>,
    pub evaluation_label_source: LabelSource,
    pub max_candidate_queries: usize,
}

/// A reproducible benchmark split. Candidate-facing data is separate from the
/// evaluator object, whose outcome labels are not exposed by public fields.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidateView {
    pub development: Vec<TrainingObservation>,
    pub holdout_inputs: Vec<CandidateInput>,
    pub manifest: SplitManifest,
}

/// Trusted harness state. Candidates should receive only CandidateView, never
/// this evaluator-bearing wrapper. The target ledger has no public accessors.
#[derive(Clone, PartialEq, Eq)]
pub struct ProtocolSplit {
    candidate_view: CandidateView,
    evaluator: HoldoutEvaluator,
}

// Do not derive Debug here: HoldoutEvaluator contains per-sample holdout labels.
// Redacting it prevents accidental leakage via format!("{:?}", split).
impl std::fmt::Debug for ProtocolSplit {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("ProtocolSplit")
            .field("candidate_view", &self.candidate_view)
            .field("evaluator", &"[REDACTED]")
            .finish()
    }
}

/// The trusted scoring side. Targets are private and there are no getters for
/// per-sample labels. Evaluation returns aggregate counts only.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HoldoutEvaluator {
    targets: BTreeMap<usize, HoldoutTarget>,
    max_candidate_queries: usize,
    label_source: LabelSource,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct HoldoutTarget {
    feasible: bool,
    actual_failure: bool,
}

/// Aggregate evaluation receipt. No individual holdout labels are returned.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvaluationReport {
    pub schema_version: &'static str,
    pub method_id: String,
    pub method_version: String,
    pub label_source: LabelSource,
    pub holdout_sample_count: usize,
    pub candidate_query_count: usize,
    pub score: Score,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProtocolError {
    EmptyDataset,
    TooFewSamplesForSplit,
    InsufficientModelFamilies,
    EmptyModelFamilyId,
    UnknownHoldoutFamily(String),
    InconsistentDimensions,
    EmptyMethodId,
    EmptyMethodVersion,
    QueryBudgetExceeded { supplied: usize, maximum: usize },
    DuplicatePredictionId(usize),
    UnknownPredictionId(usize),
    MissingPredictionId(usize),
}

/// Creates a deterministic interleaved split over the supplied point order.
///
/// This split is a smoke-test protocol, not a statistically independent test
/// split: neighboring grid points may share structure. Training always receives
/// observed_failure; the evaluator scores holdout predictions against the
/// explicitly requested source. Model-family holdouts should be used for robust
/// generalization claims (see CECB-ROBUST-001).
pub fn build_protocol_split(
    points: &[Point],
    evaluation_label_source: LabelSource,
    max_candidate_queries: usize,
) -> Result<ProtocolSplit, ProtocolError> {
    if points.is_empty() {
        return Err(ProtocolError::EmptyDataset);
    }
    if points.len() < 2 {
        return Err(ProtocolError::TooFewSamplesForSplit);
    }

    let dimensions = points[0].features.len();
    if dimensions == 0
        || points
            .iter()
            .any(|point| point.features.len() != dimensions)
    {
        return Err(ProtocolError::InconsistentDimensions);
    }

    let mut development = Vec::new();
    let mut holdout_inputs = Vec::new();
    let mut development_sample_ids = Vec::new();
    let mut holdout_sample_ids = Vec::new();
    let mut targets = BTreeMap::new();

    for (sample_id, point) in points.iter().enumerate() {
        if sample_id % 2 == 0 {
            development_sample_ids.push(sample_id);
            development.push(TrainingObservation {
                sample_id,
                model_family_id: None,
                features: point.features.clone(),
                feasible: point.feasible,
                observed_failure: point.observed_failure,
            });
        } else {
            holdout_sample_ids.push(sample_id);
            holdout_inputs.push(CandidateInput {
                sample_id,
                model_family_id: None,
                features: point.features.clone(),
                feasible: point.feasible,
            });
            let actual_failure = match evaluation_label_source {
                LabelSource::LatentTruth => point.latent_failure,
                LabelSource::ObservedLabel => point.observed_failure,
            };
            targets.insert(
                sample_id,
                HoldoutTarget {
                    feasible: point.feasible,
                    actual_failure,
                },
            );
        }
    }

    if development.is_empty() || holdout_inputs.is_empty() {
        return Err(ProtocolError::TooFewSamplesForSplit);
    }

    let evaluator = HoldoutEvaluator {
        targets,
        max_candidate_queries,
        label_source: evaluation_label_source,
    };
    let manifest = SplitManifest {
        schema_version: SCHEMA_VERSION,
        split_rule: SPLIT_RULE,
        development_sample_ids,
        holdout_sample_ids,
        holdout_family_id: None,
        evaluation_label_source,
        max_candidate_queries,
    };

    Ok(ProtocolSplit {
        candidate_view: CandidateView {
            development,
            holdout_inputs,
            manifest,
        },
        evaluator,
    })
}

/// A point labelled with its originating structural/simulator family.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FamilyTaggedPoint {
    pub family_id: String,
    pub point: Point,
}

/// Builds a leave-one-model-family-out split. The held-out family is completely
/// absent from development labels; candidate inputs retain the family identifier
/// so transfer behaviour can be grouped and reported. At least two distinct
/// families are required, and the selected holdout family must exist.
pub fn build_model_family_split(
    points: &[FamilyTaggedPoint],
    heldout_family_id: &str,
    evaluation_label_source: LabelSource,
    max_candidate_queries: usize,
) -> Result<ProtocolSplit, ProtocolError> {
    if points.is_empty() {
        return Err(ProtocolError::EmptyDataset);
    }
    if points.len() < 2 {
        return Err(ProtocolError::TooFewSamplesForSplit);
    }
    if heldout_family_id.trim().is_empty() || points.iter().any(|row| row.family_id.trim().is_empty()) {
        return Err(ProtocolError::EmptyModelFamilyId);
    }

    let dimensions = points[0].point.features.len();
    if dimensions == 0
        || points
            .iter()
            .any(|row| row.point.features.len() != dimensions)
    {
        return Err(ProtocolError::InconsistentDimensions);
    }

    let family_ids = points
        .iter()
        .map(|row| row.family_id.as_str())
        .collect::<BTreeSet<_>>();
    if family_ids.len() < 2 {
        return Err(ProtocolError::InsufficientModelFamilies);
    }
    if !family_ids.contains(heldout_family_id) {
        return Err(ProtocolError::UnknownHoldoutFamily(
            heldout_family_id.to_owned(),
        ));
    }

    let mut development = Vec::new();
    let mut holdout_inputs = Vec::new();
    let mut development_sample_ids = Vec::new();
    let mut holdout_sample_ids = Vec::new();
    let mut targets = BTreeMap::new();

    for (sample_id, row) in points.iter().enumerate() {
        if row.family_id == heldout_family_id {
            holdout_sample_ids.push(sample_id);
            holdout_inputs.push(CandidateInput {
                sample_id,
                model_family_id: Some(row.family_id.clone()),
                features: row.point.features.clone(),
                feasible: row.point.feasible,
            });
            let actual_failure = match evaluation_label_source {
                LabelSource::LatentTruth => row.point.latent_failure,
                LabelSource::ObservedLabel => row.point.observed_failure,
            };
            targets.insert(
                sample_id,
                HoldoutTarget {
                    feasible: row.point.feasible,
                    actual_failure,
                },
            );
        } else {
            development_sample_ids.push(sample_id);
            development.push(TrainingObservation {
                sample_id,
                model_family_id: Some(row.family_id.clone()),
                features: row.point.features.clone(),
                feasible: row.point.feasible,
                observed_failure: row.point.observed_failure,
            });
        }
    }

    if development.is_empty() || holdout_inputs.is_empty() {
        return Err(ProtocolError::InsufficientModelFamilies);
    }

    let evaluator = HoldoutEvaluator {
        targets,
        max_candidate_queries,
        label_source: evaluation_label_source,
    };
    let manifest = SplitManifest {
        schema_version: SCHEMA_VERSION,
        split_rule: "leave_one_model_family_out_v0.1",
        development_sample_ids,
        holdout_sample_ids,
        holdout_family_id: Some(heldout_family_id.to_owned()),
        evaluation_label_source,
        max_candidate_queries,
    };

    Ok(ProtocolSplit {
        candidate_view: CandidateView {
            development,
            holdout_inputs,
            manifest,
        },
        evaluator,
    })
}

impl ProtocolSplit {
    /// Returns only candidate-visible observations and feature-only holdout inputs.
    pub fn candidate_view(&self) -> &CandidateView {
        &self.candidate_view
    }

    /// Trusted harness entry point. Do not pass ProtocolSplit itself to a
    /// candidate process; serialize/pass only candidate_view().
    pub fn evaluate(
        &self,
        submission: &CandidateSubmission,
        candidate_query_count: usize,
    ) -> Result<EvaluationReport, ProtocolError> {
        self.evaluator.evaluate(submission, candidate_query_count)
    }

    pub fn evaluation_label_source(&self) -> LabelSource {
        self.evaluator.label_source
    }
}

impl HoldoutEvaluator {
    pub fn max_candidate_queries(&self) -> usize {
        self.max_candidate_queries
    }

    pub fn holdout_sample_count(&self) -> usize {
        self.targets.len()
    }

    pub fn evaluation_label_source(&self) -> LabelSource {
        self.label_source
    }

    /// Evaluates a full, one-to-one prediction set. candidate_query_count must
    /// come from a trusted, instrumented harness—not from the candidate itself.
    pub fn evaluate(
        &self,
        submission: &CandidateSubmission,
        candidate_query_count: usize,
    ) -> Result<EvaluationReport, ProtocolError> {
        if submission.method_id.trim().is_empty() {
            return Err(ProtocolError::EmptyMethodId);
        }
        if submission.method_version.trim().is_empty() {
            return Err(ProtocolError::EmptyMethodVersion);
        }
        if candidate_query_count > self.max_candidate_queries {
            return Err(ProtocolError::QueryBudgetExceeded {
                supplied: candidate_query_count,
                maximum: self.max_candidate_queries,
            });
        }

        let expected_ids = self.targets.keys().copied().collect::<BTreeSet<_>>();
        let mut predictions = BTreeMap::new();
        for prediction in &submission.predictions {
            if !expected_ids.contains(&prediction.sample_id) {
                return Err(ProtocolError::UnknownPredictionId(prediction.sample_id));
            }
            if predictions
                .insert(prediction.sample_id, prediction.predicts_failure)
                .is_some()
            {
                return Err(ProtocolError::DuplicatePredictionId(prediction.sample_id));
            }
        }
        for sample_id in expected_ids {
            if !predictions.contains_key(&sample_id) {
                return Err(ProtocolError::MissingPredictionId(sample_id));
            }
        }

        let mut score = Score::default();
        for (sample_id, target) in &self.targets {
            if !target.feasible {
                score.infeasible_skipped += 1;
                continue;
            }
            let predicted_failure = predictions[sample_id];
            match (predicted_failure, target.actual_failure) {
                (true, true) => score.true_positive += 1,
                (true, false) => score.false_positive += 1,
                (false, true) => score.false_negative += 1,
                (false, false) => score.true_negative += 1,
            }
        }

        Ok(EvaluationReport {
            schema_version: SCHEMA_VERSION,
            method_id: submission.method_id.clone(),
            method_version: submission.method_version.clone(),
            label_source: self.label_source,
            holdout_sample_count: self.targets.len(),
            candidate_query_count,
            score,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{axis_aligned_landscape, noisy_boundary_landscape};

    fn exact_axis_submission(view: &CandidateView) -> CandidateSubmission {
        CandidateSubmission {
            method_id: "fixture-axis-rule".to_owned(),
            method_version: "1".to_owned(),
            predictions: view
                .holdout_inputs
                .iter()
                .map(|input| Prediction {
                    sample_id: input.sample_id,
                    predicts_failure: input.features[0] >= 2,
                })
                .collect(),
        }
    }

    #[test]
    fn split_is_deterministic_and_manifest_ids_match_data() {
        let points = axis_aligned_landscape();
        let first = build_protocol_split(&points, LabelSource::LatentTruth, 25).unwrap();
        let second = build_protocol_split(&points, LabelSource::LatentTruth, 25).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.candidate_view().manifest.split_rule, "alternating_input_index_v0.1");
        assert_eq!(
            first.candidate_view().development.iter().map(|row| row.sample_id).collect::<Vec<_>>(),
            first.candidate_view().manifest.development_sample_ids
        );
        assert_eq!(
            first.candidate_view().holdout_inputs.iter().map(|row| row.sample_id).collect::<Vec<_>>(),
            first.candidate_view().manifest.holdout_sample_ids
        );
        assert!(first.candidate_view().development.iter().all(|row| row.sample_id % 2 == 0));
        assert!(first.candidate_view().holdout_inputs.iter().all(|row| row.sample_id % 2 == 1));
    }

    #[test]
    fn candidate_facing_holdout_records_only_have_features_and_feasibility() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            25,
        )
        .unwrap();
        fn candidate_input_has_no_label(input: &CandidateInput) -> (&[i32], bool, usize) {
            (&input.features, input.feasible, input.sample_id)
        }
        assert!(split.candidate_view().holdout_inputs.iter().all(|row| {
            let (features, _feasible, id) = candidate_input_has_no_label(row);
            !features.is_empty() && id % 2 == 1
        }));
    }

    fn family_tagged_fixture() -> Vec<FamilyTaggedPoint> {
        let mut rows = crate::structural_family_a()
            .into_iter()
            .map(|point| FamilyTaggedPoint {
                family_id: "family_a".to_owned(),
                point,
            })
            .collect::<Vec<_>>();
        rows.extend(
            crate::structural_family_b()
                .into_iter()
                .map(|point| FamilyTaggedPoint {
                    family_id: "family_b".to_owned(),
                    point,
                }),
        );
        rows
    }

    #[test]
    fn leave_one_model_family_out_never_places_holdout_family_in_development() {
        let split = build_model_family_split(
            &family_tagged_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        assert_eq!(
            split.candidate_view().manifest.split_rule,
            "leave_one_model_family_out_v0.1"
        );
        assert_eq!(split.candidate_view().manifest.holdout_family_id.as_deref(), Some("family_b"));
        assert_eq!(split.candidate_view().development.len(), 9);
        assert_eq!(split.candidate_view().holdout_inputs.len(), 9);
        assert!(split
            .development
            .iter()
            .all(|row| row.model_family_id.as_deref() == Some("family_a")));
        assert!(split
            .holdout_inputs
            .iter()
            .all(|row| row.model_family_id.as_deref() == Some("family_b")));
    }

    #[test]
    fn reversed_family_exposes_failure_of_a_transferred_threshold_rule() {
        let split = build_model_family_split(
            &family_tagged_fixture(),
            "family_b",
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        let submission = CandidateSubmission {
            method_id: "transferred-x-ge-2".to_owned(),
            method_version: "1".to_owned(),
            predictions: split
                .holdout_inputs
                .iter()
                .map(|input| Prediction {
                    sample_id: input.sample_id,
                    predicts_failure: input.features[0] >= 2,
                })
                .collect(),
        };
        let report = split.evaluate(&submission, 0).unwrap();
        assert_eq!(report.score.true_positive, 0);
        assert_eq!(report.score.false_positive, 3);
        assert_eq!(report.score.false_negative, 3);
        assert_eq!(report.score.true_negative, 3);
    }

    #[test]
    fn model_family_split_rejects_unknown_empty_and_single_families() {
        let rows = family_tagged_fixture();
        assert_eq!(
            build_model_family_split(&rows, "missing", LabelSource::LatentTruth, 5),
            Err(ProtocolError::UnknownHoldoutFamily("missing".to_owned()))
        );
        assert_eq!(
            build_model_family_split(&rows, "  ", LabelSource::LatentTruth, 5),
            Err(ProtocolError::EmptyModelFamilyId)
        );
        let single_family = rows
            .into_iter()
            .map(|mut row| {
                row.family_id = "one-family".to_owned();
                row
            })
            .collect::<Vec<_>>();
        assert_eq!(
            build_model_family_split(
                &single_family,
                "one-family",
                LabelSource::LatentTruth,
                5
            ),
            Err(ProtocolError::InsufficientModelFamilies)
        );
    }

    #[test]
    fn debug_formatting_redacts_private_evaluator_targets() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        let rendered = format!("{split:?}");
        assert!(rendered.contains("[REDACTED]"));
        assert!(!rendered.contains("targets:"));
        assert!(!rendered.contains("HoldoutTarget"));
    }

    #[test]
    fn exact_rule_scores_perfectly_on_axis_aligned_holdout() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            0,
        )
        .unwrap();
        let report = split.evaluate(&exact_axis_submission(split.candidate_view()), 0).unwrap();
        assert_eq!(report.schema_version, SCHEMA_VERSION);
        assert_eq!(report.holdout_sample_count, split.candidate_view().holdout_inputs.len());
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
        assert_eq!(
            report.score.true_positive + report.score.true_negative,
            report.score.evaluated_points()
        );
        assert_eq!(report.candidate_query_count, 0);
    }

    #[test]
    fn evaluator_can_score_observed_labels_without_exposing_them_to_candidate() {
        let split = build_protocol_split(
            &noisy_boundary_landscape(),
            LabelSource::ObservedLabel,
            5,
        )
        .unwrap();
        assert!(split.candidate_view().development.iter().any(|row| row.observed_failure));
        assert_eq!(split.evaluation_label_source(), LabelSource::ObservedLabel);
        let report = split.evaluate(&exact_axis_submission(split.candidate_view()), 0).unwrap();
        assert_eq!(report.label_source, LabelSource::ObservedLabel);
    }

    #[test]
    fn evaluator_rejects_candidate_metadata_without_method_identity() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        let mut submission = exact_axis_submission(split.candidate_view());
        submission.method_id = "  ".to_owned();
        assert_eq!(
            split.evaluate(&submission, 0),
            Err(ProtocolError::EmptyMethodId)
        );
        submission.method_id = "test".to_owned();
        submission.method_version.clear();
        assert_eq!(
            split.evaluate(&submission, 0),
            Err(ProtocolError::EmptyMethodVersion)
        );
    }

    #[test]
    fn evaluator_rejects_duplicate_missing_and_unknown_predictions() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();

        let mut duplicate = exact_axis_submission(split.candidate_view());
        duplicate.predictions.push(duplicate.predictions[0].clone());
        assert!(matches!(
            split.evaluate(&duplicate, 0),
            Err(ProtocolError::DuplicatePredictionId(_))
        ));

        let mut missing = exact_axis_submission(split.candidate_view());
        missing.predictions.pop();
        assert!(matches!(
            split.evaluate(&missing, 0),
            Err(ProtocolError::MissingPredictionId(_))
        ));

        let mut unknown = exact_axis_submission(split.candidate_view());
        unknown.predictions[0].sample_id = usize::MAX;
        assert_eq!(
            split.evaluate(&unknown, 0),
            Err(ProtocolError::UnknownPredictionId(usize::MAX))
        );
    }

    #[test]
    fn evaluator_rejects_over_budget_receipts_from_harness() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            2,
        )
        .unwrap();
        assert_eq!(
            split.evaluate(&exact_axis_submission(split.candidate_view()), 3),
            Err(ProtocolError::QueryBudgetExceeded {
                supplied: 3,
                maximum: 2
            })
        );
    }

    #[test]
    fn evaluator_is_order_independent_but_requires_exact_id_coverage() {
        let split = build_protocol_split(
            &axis_aligned_landscape(),
            LabelSource::LatentTruth,
            5,
        )
        .unwrap();
        let mut submission = exact_axis_submission(split.candidate_view());
        submission.predictions.reverse();
        let report = split.evaluate(&submission, 0).unwrap();
        assert_eq!(report.score.false_positive, 0);
        assert_eq!(report.score.false_negative, 0);
    }

    #[test]
    fn split_fails_closed_for_empty_small_or_malformed_datasets() {
        assert_eq!(
            build_protocol_split(&[], LabelSource::LatentTruth, 1),
            Err(ProtocolError::EmptyDataset)
        );
        assert_eq!(
            build_protocol_split(
                &[Point {
                    features: vec![0],
                    feasible: true,
                    latent_failure: false,
                    observed_failure: false
                }],
                LabelSource::LatentTruth,
                1
            ),
            Err(ProtocolError::TooFewSamplesForSplit)
        );
        assert_eq!(
            build_protocol_split(
                &[
                    Point {
                        features: vec![0],
                        feasible: true,
                        latent_failure: false,
                        observed_failure: false
                    },
                    Point {
                        features: vec![0, 1],
                        feasible: true,
                        latent_failure: false,
                        observed_failure: false
                    }
                ],
                LabelSource::LatentTruth,
                1
            ),
            Err(ProtocolError::InconsistentDimensions)
        );
    }
}
