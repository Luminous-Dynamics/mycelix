//! Decision-sensitive evidence acquisition for procurement.
//!
//! This module ranks *which evidence action to consider next* by how the
//! possible observation outcomes change a frozen procurement Pareto frontier.
//! It does not calculate expected value of information: probabilities are not
//! accepted or inferred. Action costs, completion deadlines and evidence expiry
//! remain separate dimensions, and recommendations never execute an acquisition.

use std::collections::{BTreeMap, BTreeSet};
use std::cmp::Ordering;

use super::joint_optimizer::{JointObjective, RequestShortage};

const ACTION_PROFILE: &str = "mycelix:procurement-frontier-sensitivity:v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EvidenceOutcomeKind {
    Confirmed,
    Refuted,
    Inconclusive,
    Unavailable,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceOutcomeBranch {
    pub outcome_id: String,
    pub kind: EvidenceOutcomeKind,
    /// The frontier expected under this outcome. An empty frontier means no
    /// feasible modeled allocation remains under that branch.
    pub resulting_frontier: Vec<JointObjective>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceAcquisitionAction {
    pub action_id: String,
    /// Stable identity for the claim/version/verification need. Actions sharing
    /// a key are alternatives; the selector must not recommend them all.
    pub claim_version_key: String,
    pub source_authority_class: String,
    /// Evidence may cover multiple requests, offers, mappings, or capacity pools.
    pub covered_subject_ids: Vec<String>,
    pub acquisition_cost_minor: u64,
    pub acquisition_currency: String,
    pub latest_completion_at: u64,
    pub evidence_valid_until: u64,
    pub outcomes: Vec<EvidenceOutcomeBranch>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceAcquisitionProblem {
    pub evaluated_at: u64,
    pub decision_deadline: u64,
    pub input_manifest_digest: String,
    pub acquisition_currency: String,
    /// Current frontier before taking any evidence action.
    pub baseline_frontier: Vec<JointObjective>,
    pub actions: Vec<EvidenceAcquisitionAction>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EvidenceActionError {
    InvalidManifest,
    InvalidDecisionDeadline,
    InvalidActionTimeline,
    InvalidBaselineFrontier,
    InvalidAction,
    DuplicateActionId,
    DuplicateOutcomeId,
    MixedProcurementCurrency,
    MixedAcquisitionCurrency,
    InconsistentRequestAxes,
    DuplicateRequestAxis,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum EvidenceActionDisposition {
    Recommended,
    Dominated,
    DuplicateClaimAlternative,
    NoDecisionValue,
    CompletesAfterDeadline,
    EvidenceStaleByCompletion,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceOutcomeAssessment {
    pub outcome_id: String,
    pub kind: EvidenceOutcomeKind,
    /// Canonical branch frontier. An empty vector means no feasible allocation
    /// remains in the declared model for this outcome.
    pub resulting_frontier: Vec<JointObjective>,
    pub changes_baseline_frontier: bool,
    pub frontier_symmetric_difference_count: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceActionAssessment {
    pub action_id: String,
    pub claim_version_key: String,
    pub source_authority_class: String,
    pub outcome_assessments: Vec<EvidenceOutcomeAssessment>,
    pub covered_subject_ids: Vec<String>,
    pub acquisition_cost_minor: u64,
    pub acquisition_currency: String,
    pub latest_completion_at: u64,
    pub evidence_valid_until: u64,
    /// Count of distinct outcome-frontier states; duplicate outcomes do not
    /// increase this number and no probability is assigned to any branch.
    pub distinct_outcome_frontier_count: usize,
    /// Count of distinct outcome frontiers that differ from the current frontier.
    pub changed_frontier_count: usize,
    /// Maximum number of objective vectors inserted/removed in any outcome branch.
    /// This is structural frontier sensitivity, not expected monetary value.
    pub max_frontier_symmetric_difference: usize,
    pub disposition: EvidenceActionDisposition,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EvidenceAcquisitionReport {
    pub evaluator_profile: &'static str,
    pub input_manifest_digest: String,
    pub acquisition_currency: String,
    pub procurement_currency: String,
    pub decision_deadline: u64,
    /// Ordered by the declared lexicographic sensitivity/cost/latency comparator.
    pub recommended_action_ids: Vec<String>,
    /// All actions remain visible, including no-value and deadline-ineligible ones.
    pub assessments: Vec<EvidenceActionAssessment>,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn canonicalize_frontier(
    frontier: &[JointObjective],
    procurement_currency: &str,
    expected_axes: Option<&[(String, String)]>,
) -> Result<(Vec<JointObjective>, Vec<(String, String)>), EvidenceActionError> {
    let mut canonical = Vec::with_capacity(frontier.len());
    let mut axes_reference = expected_axes.map(|axes| axes.to_vec());

    for objective in frontier {
        if objective.currency != procurement_currency {
            return Err(EvidenceActionError::MixedProcurementCurrency);
        }
        let mut shortages = objective.unfilled_by_request.clone();
        if shortages.iter().any(|item| !non_empty(&item.request_id) || !non_empty(&item.unit)) {
            return Err(EvidenceActionError::InvalidBaselineFrontier);
        }
        shortages.sort_by(|left, right| {
            (&left.request_id, &left.unit).cmp(&(&right.request_id, &right.unit))
        });
        let axes: Vec<(String, String)> = shortages
            .iter()
            .map(|item| (item.request_id.clone(), item.unit.clone()))
            .collect();
        let distinct_request_ids: BTreeSet<&str> =
            shortages.iter().map(|item| item.request_id.as_str()).collect();
        if distinct_request_ids.len() != shortages.len() {
            return Err(EvidenceActionError::DuplicateRequestAxis);
        }

        if let Some(reference) = &axes_reference {
            if reference != &axes {
                return Err(EvidenceActionError::InconsistentRequestAxes);
            }
        } else {
            axes_reference = Some(axes.clone());
        }

        let mut normalized = objective.clone();
        normalized.unfilled_by_request = shortages;
        canonical.push(normalized);
    }

    canonical.sort();
    canonical.dedup();
    Ok((canonical, axes_reference.unwrap_or_default()))
}

fn frontier_difference_count(left: &[JointObjective], right: &[JointObjective]) -> usize {
    let left: BTreeSet<JointObjective> = left.iter().cloned().collect();
    let right: BTreeSet<JointObjective> = right.iter().cloned().collect();
    left.difference(&right).count() + right.difference(&left).count()
}

fn canonical_outcome_assessments(
    action: &EvidenceAcquisitionAction,
    procurement_currency: &str,
    axes: &[(String, String)],
    baseline: &[JointObjective],
) -> Result<(Vec<EvidenceOutcomeAssessment>, Vec<Vec<JointObjective>>), EvidenceActionError> {
    let mut outcome_ids = BTreeSet::new();
    let mut distinct_frontiers = BTreeSet::new();
    let mut assessments = Vec::with_capacity(action.outcomes.len());

    if action.outcomes.is_empty() {
        return Err(EvidenceActionError::InvalidAction);
    }
    for outcome in &action.outcomes {
        if !non_empty(&outcome.outcome_id) || !outcome_ids.insert(outcome.outcome_id.as_str()) {
            return Err(EvidenceActionError::DuplicateOutcomeId);
        }
        let (frontier, _) = canonicalize_frontier(
            &outcome.resulting_frontier,
            procurement_currency,
            Some(axes),
        )?;
        let difference = frontier_difference_count(baseline, &frontier);
        assessments.push(EvidenceOutcomeAssessment {
            outcome_id: outcome.outcome_id.clone(),
            kind: outcome.kind,
            changes_baseline_frontier: frontier.as_slice() != baseline,
            frontier_symmetric_difference_count: difference,
            resulting_frontier: frontier.clone(),
        });
        distinct_frontiers.insert(frontier);
    }
    assessments.sort_by(|left, right| left.outcome_id.cmp(&right.outcome_id));
    Ok((assessments, distinct_frontiers.into_iter().collect()))
}

fn impact(
    baseline: &[JointObjective],
    outcome_frontiers: &[Vec<JointObjective>],
) -> (usize, usize) {
    let changed_frontier_count = outcome_frontiers
        .iter()
        .filter(|frontier| frontier.as_slice() != baseline)
        .count();
    let max_delta = outcome_frontiers
        .iter()
        .map(|frontier| frontier_difference_count(baseline, frontier))
        .max()
        .unwrap_or(0);
    (changed_frontier_count, max_delta)
}

fn compare_priority(
    left: &EvidenceActionAssessment,
    right: &EvidenceActionAssessment,
) -> Ordering {
    right
        .max_frontier_symmetric_difference
        .cmp(&left.max_frontier_symmetric_difference)
        .then_with(|| right.changed_frontier_count.cmp(&left.changed_frontier_count))
        .then_with(|| {
            right
                .distinct_outcome_frontier_count
                .cmp(&left.distinct_outcome_frontier_count)
        })
        .then_with(|| left.acquisition_cost_minor.cmp(&right.acquisition_cost_minor))
        .then_with(|| left.latest_completion_at.cmp(&right.latest_completion_at))
        .then_with(|| left.action_id.cmp(&right.action_id))
}

fn dominates(
    left: &EvidenceActionAssessment,
    right: &EvidenceActionAssessment,
) -> bool {
    let no_worse = left.max_frontier_symmetric_difference
        >= right.max_frontier_symmetric_difference
        && left.changed_frontier_count >= right.changed_frontier_count
        && left.distinct_outcome_frontier_count >= right.distinct_outcome_frontier_count
        && left.acquisition_cost_minor <= right.acquisition_cost_minor
        && left.latest_completion_at <= right.latest_completion_at;
    let strictly_better = left.max_frontier_symmetric_difference
        > right.max_frontier_symmetric_difference
        || left.changed_frontier_count > right.changed_frontier_count
        || left.distinct_outcome_frontier_count > right.distinct_outcome_frontier_count
        || left.acquisition_cost_minor < right.acquisition_cost_minor
        || left.latest_completion_at < right.latest_completion_at;
    no_worse && strictly_better
}

/// Rank evidence actions by structural changes to the procurement Pareto frontier.
///
/// No probabilities are accepted, inferred, or multiplied into the score. First,
/// actions that cannot complete by the decision deadline or whose result would
/// already be stale are excluded. Actions with only one unique outcome frontier,
/// or whose outcomes all preserve the baseline frontier, have no decision value
/// under this model. Remaining actions are Pareto-filtered over frontier impact,
/// acquisition cost, and completion time, then deterministically ordered. If
/// multiple actions verify the same claim/version, at most one is recommended.
pub fn rank_evidence_actions(
    problem: &EvidenceAcquisitionProblem,
) -> Result<EvidenceAcquisitionReport, EvidenceActionError> {
    if !non_empty(&problem.input_manifest_digest)
        || !non_empty(&problem.acquisition_currency)
    {
        return Err(EvidenceActionError::InvalidManifest);
    }
    if problem.decision_deadline < problem.evaluated_at {
        return Err(EvidenceActionError::InvalidDecisionDeadline);
    }

    // Determine a single procurement currency from the baseline frontier.
    // It is intentionally distinct from the acquisition-cost currency.
    let procurement_currency = problem
        .baseline_frontier
        .first()
        .map(|objective| objective.currency.clone())
        .ok_or(EvidenceActionError::InvalidBaselineFrontier)?;
    let (baseline, axes) = canonicalize_frontier(
        &problem.baseline_frontier,
        &procurement_currency,
        None,
    )?;
    if baseline.is_empty() || axes.is_empty() {
        return Err(EvidenceActionError::InvalidBaselineFrontier);
    }

    let mut seen_action_ids = BTreeSet::new();
    let mut raw_assessments = Vec::with_capacity(problem.actions.len());

    for action in &problem.actions {
        if !non_empty(&action.action_id)
            || !non_empty(&action.claim_version_key)
            || !non_empty(&action.source_authority_class)
            || action.covered_subject_ids.is_empty()
            || action.covered_subject_ids.iter().any(|subject| !non_empty(subject))
            || !non_empty(&action.acquisition_currency)
            || !seen_action_ids.insert(action.action_id.as_str())
        {
            return Err(if seen_action_ids.contains(action.action_id.as_str()) {
                EvidenceActionError::DuplicateActionId
            } else {
                EvidenceActionError::InvalidAction
            });
        }
        if action.acquisition_currency != problem.acquisition_currency {
            return Err(EvidenceActionError::MixedAcquisitionCurrency);
        }
        if action.latest_completion_at < problem.evaluated_at {
            return Err(EvidenceActionError::InvalidActionTimeline);
        }

        let (outcome_assessments, outcome_frontiers) = canonical_outcome_assessments(
            action,
            &procurement_currency,
            &axes,
            &baseline,
        )?;
        let (changed_count, max_delta) = impact(&baseline, &outcome_frontiers);
        let distinct_count = outcome_frontiers.len();

        let disposition = if action.latest_completion_at > problem.decision_deadline {
            EvidenceActionDisposition::CompletesAfterDeadline
        } else if action.evidence_valid_until < action.latest_completion_at {
            EvidenceActionDisposition::EvidenceStaleByCompletion
        } else if distinct_count <= 1 || changed_count == 0 {
            EvidenceActionDisposition::NoDecisionValue
        } else {
            EvidenceActionDisposition::Recommended
        };

        let mut covered_subject_ids = action.covered_subject_ids.clone();
        covered_subject_ids.sort();
        covered_subject_ids.dedup();

        let assessment = EvidenceActionAssessment {
            action_id: action.action_id.clone(),
            claim_version_key: action.claim_version_key.clone(),
            source_authority_class: action.source_authority_class.clone(),
            outcome_assessments,
            covered_subject_ids,
            acquisition_cost_minor: action.acquisition_cost_minor,
            acquisition_currency: action.acquisition_currency.clone(),
            latest_completion_at: action.latest_completion_at,
            evidence_valid_until: action.evidence_valid_until,
            distinct_outcome_frontier_count: distinct_count,
            changed_frontier_count: changed_count,
            max_frontier_symmetric_difference: max_delta,
            disposition,
        };

        raw_assessments.push(assessment);
    }

    let eligible: Vec<EvidenceActionAssessment> = raw_assessments
        .iter()
        .filter(|assessment| assessment.disposition == EvidenceActionDisposition::Recommended)
        .cloned()
        .collect();

    let mut assessments = raw_assessments;
    for assessment in &mut assessments {
        if assessment.disposition != EvidenceActionDisposition::Recommended {
            continue;
        }
        if eligible
            .iter()
            .any(|other| other.action_id != assessment.action_id && dominates(other, assessment))
        {
            assessment.disposition = EvidenceActionDisposition::Dominated;
        }
    }

    let mut surviving: Vec<EvidenceActionAssessment> = assessments
        .iter()
        .filter(|assessment| assessment.disposition == EvidenceActionDisposition::Recommended)
        .cloned()
        .collect();

    // Multiple mechanisms may request the same versioned fact. They are
    // alternatives, not independent recommendations; choose one transparently.
    let mut by_claim: BTreeMap<String, Vec<EvidenceActionAssessment>> = BTreeMap::new();
    for assessment in surviving.drain(..) {
        by_claim
            .entry(assessment.claim_version_key.clone())
            .or_default()
            .push(assessment);
    }
    let mut recommendations = Vec::new();
    for alternatives in by_claim.values_mut() {
        alternatives.sort_by(compare_priority);
        if let Some(best) = alternatives.first() {
            recommendations.push(best.action_id.clone());
        }
        for alternative in alternatives.iter().skip(1) {
            if let Some(assessment) = assessments
                .iter_mut()
                .find(|assessment| assessment.action_id == alternative.action_id)
            {
                assessment.disposition = EvidenceActionDisposition::DuplicateClaimAlternative;
            }
        }
    }

    recommendations.sort_by(|left_id, right_id| {
        let left = assessments.iter().find(|item| &item.action_id == left_id);
        let right = assessments.iter().find(|item| &item.action_id == right_id);
        match (left, right) {
            (Some(left), Some(right)) => compare_priority(left, right),
            _ => left_id.cmp(right_id),
        }
    });
    assessments.sort_by(|left, right| {
        compare_priority(left, right).then_with(|| left.disposition.cmp(&right.disposition))
    });

    Ok(EvidenceAcquisitionReport {
        evaluator_profile: ACTION_PROFILE,
        input_manifest_digest: problem.input_manifest_digest.clone(),
        acquisition_currency: problem.acquisition_currency.clone(),
        procurement_currency,
        decision_deadline: problem.decision_deadline,
        recommended_action_ids: recommendations,
        assessments,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        EvidenceAcquisitionAction, EvidenceAcquisitionProblem, EvidenceActionDisposition,
        EvidenceActionError, EvidenceOutcomeBranch, EvidenceOutcomeKind, rank_evidence_actions,
    };
    use crate::joint_optimizer::{JointObjective, RequestShortage};

    fn shortage(request_id: &str, quantity: u64) -> RequestShortage {
        RequestShortage {
            request_id: request_id.to_owned(),
            unit: "kit".to_owned(),
            unfilled_quantity: quantity,
        }
    }

    fn objective(cost: u64, routine_shortage: u64, urgent_shortage: u64) -> JointObjective {
        JointObjective {
            currency: "USD".to_owned(),
            total_cost_minor: cost,
            unfilled_by_request: vec![
                shortage("routine", routine_shortage),
                shortage("urgent", urgent_shortage),
            ],
        }
    }

    fn branch(id: &str, kind: EvidenceOutcomeKind, frontier: Vec<JointObjective>) -> EvidenceOutcomeBranch {
        EvidenceOutcomeBranch {
            outcome_id: id.to_owned(),
            kind,
            resulting_frontier: frontier,
        }
    }

    fn action(
        action_id: &str,
        claim_key: &str,
        cost: u64,
        completion: u64,
        valid_until: u64,
        outcomes: Vec<EvidenceOutcomeBranch>,
    ) -> EvidenceAcquisitionAction {
        EvidenceAcquisitionAction {
            action_id: action_id.to_owned(),
            claim_version_key: claim_key.to_owned(),
            source_authority_class: "supplier-confirmation".to_owned(),
            covered_subject_ids: vec!["offer-a".to_owned()],
            acquisition_cost_minor: cost,
            acquisition_currency: "USD".to_owned(),
            latest_completion_at: completion,
            evidence_valid_until: valid_until,
            outcomes,
        }
    }

    fn problem(actions: Vec<EvidenceAcquisitionAction>) -> EvidenceAcquisitionProblem {
        EvidenceAcquisitionProblem {
            evaluated_at: 10,
            decision_deadline: 30,
            input_manifest_digest: "sha256-evidence-action-fixture".to_owned(),
            acquisition_currency: "USD".to_owned(),
            baseline_frontier: vec![objective(100, 0, 0)],
            actions,
        }
    }

    fn valuable_outcomes() -> Vec<EvidenceOutcomeBranch> {
        vec![
            branch(
                "confirmed",
                EvidenceOutcomeKind::Confirmed,
                vec![objective(80, 0, 0)],
            ),
            branch(
                "refuted",
                EvidenceOutcomeKind::Refuted,
                vec![objective(70, 1, 0)],
            ),
        ]
    }

    #[test]
    fn ranks_actions_that_distinguish_materially_different_frontiers() {
        let input = problem(vec![
            action("inspect-inventory", "inventory:site-a:v1", 5, 20, 40, valuable_outcomes()),
            action(
                "recheck-unchanged-field",
                "credential:site-a:v2",
                1,
                15,
                40,
                vec![
                    branch("yes", EvidenceOutcomeKind::Confirmed, vec![objective(100, 0, 0)]),
                    branch("no", EvidenceOutcomeKind::Refuted, vec![objective(100, 0, 0)]),
                ],
            ),
        ]);
        let report = rank_evidence_actions(&input).unwrap();
        assert_eq!(report.recommended_action_ids, vec!["inspect-inventory"]);
        assert!(report.assessments.iter().any(|item| {
            item.action_id == "recheck-unchanged-field"
                && item.disposition == EvidenceActionDisposition::NoDecisionValue
        }));
    }

    #[test]
    fn does_not_recommend_evidence_that_arrives_after_the_decision_deadline() {
        let input = problem(vec![action(
            "late-audit",
            "inventory:site-a:v1",
            1,
            31,
            40,
            valuable_outcomes(),
        )]);
        let report = rank_evidence_actions(&input).unwrap();
        assert!(report.recommended_action_ids.is_empty());
        assert_eq!(
            report.assessments[0].disposition,
            EvidenceActionDisposition::CompletesAfterDeadline
        );
    }

    #[test]
    fn refuses_to_recommend_evidence_that_is_stale_when_completed() {
        let input = problem(vec![action(
            "stale-check",
            "inventory:site-a:v1",
            1,
            20,
            19,
            valuable_outcomes(),
        )]);
        let report = rank_evidence_actions(&input).unwrap();
        assert!(report.recommended_action_ids.is_empty());
        assert_eq!(
            report.assessments[0].disposition,
            EvidenceActionDisposition::EvidenceStaleByCompletion
        );
    }

    #[test]
    fn only_one_alternative_for_the_same_claim_version_is_recommended() {
        let input = problem(vec![
            action("slow-cheap", "stock:site-a:v3", 1, 24, 40, valuable_outcomes()),
            action("fast-dear", "stock:site-a:v3", 2, 18, 40, valuable_outcomes()),
        ]);
        let report = rank_evidence_actions(&input).unwrap();
        assert_eq!(report.recommended_action_ids.len(), 1);
        assert_eq!(report.recommended_action_ids[0], "slow-cheap");
        assert_eq!(
            report
                .assessments
                .iter()
                .find(|item| item.action_id == "fast-dear")
                .unwrap()
                .disposition,
            EvidenceActionDisposition::DuplicateClaimAlternative
        );
    }

    #[test]
    fn returned_assessment_preserves_each_outcome_frontier_and_authority_class() {
        let input = problem(vec![action(
            "inspect-inventory",
            "inventory:site-a:v1",
            5,
            20,
            40,
            valuable_outcomes(),
        )]);
        let report = rank_evidence_actions(&input).unwrap();
        let assessment = report
            .assessments
            .iter()
            .find(|item| item.action_id == "inspect-inventory")
            .unwrap();
        assert_eq!(assessment.source_authority_class, "supplier-confirmation");
        assert_eq!(assessment.outcome_assessments.len(), 2);
        assert!(assessment
            .outcome_assessments
            .iter()
            .all(|outcome| outcome.changes_baseline_frontier));
        assert!(assessment
            .outcome_assessments
            .iter()
            .any(|outcome| outcome.resulting_frontier[0].total_cost_minor == 70));
    }

    #[test]
    fn acquisition_action_with_completion_before_evaluation_is_rejected() {
        let input = problem(vec![action(
            "historical-check",
            "inventory:site-a:v1",
            5,
            9,
            40,
            valuable_outcomes(),
        )]);
        assert_eq!(
            rank_evidence_actions(&input),
            Err(EvidenceActionError::InvalidActionTimeline)
        );
    }

    #[test]
    fn mixed_acquisition_currency_is_rejected_instead_of_scalarized() {
        let mut input = problem(vec![action(
            "inspect-inventory",
            "inventory:site-a:v1",
            5,
            20,
            40,
            valuable_outcomes(),
        )]);
        input.actions[0].acquisition_currency = "EUR".to_owned();
        assert_eq!(
            rank_evidence_actions(&input),
            Err(EvidenceActionError::MixedAcquisitionCurrency)
        );
    }

    #[test]
    fn request_axis_mismatch_across_outcome_branch_fails_closed() {
        let invalid = JointObjective {
            currency: "USD".to_owned(),
            total_cost_minor: 80,
            unfilled_by_request: vec![shortage("different-request", 0)],
        };
        let input = problem(vec![action(
            "bad-frontier",
            "inventory:site-a:v1",
            5,
            20,
            40,
            vec![branch("changed-axis", EvidenceOutcomeKind::Confirmed, vec![invalid])],
        )]);
        assert_eq!(
            rank_evidence_actions(&input),
            Err(EvidenceActionError::InconsistentRequestAxes)
        );
    }

    #[test]
    fn duplicate_outcome_frontiers_do_not_inflate_decision_value() {
        let same = vec![objective(100, 0, 0)];
        let input = problem(vec![action(
            "duplicate-outcomes",
            "claim:v1",
            0,
            20,
            40,
            vec![
                branch("outcome-a", EvidenceOutcomeKind::Confirmed, same.clone()),
                branch("outcome-b", EvidenceOutcomeKind::Inconclusive, same),
            ],
        )]);
        let report = rank_evidence_actions(&input).unwrap();
        assert!(report.recommended_action_ids.is_empty());
        assert_eq!(
            report.assessments[0].distinct_outcome_frontier_count,
            1
        );
        assert_eq!(
            report.assessments[0].disposition,
            EvidenceActionDisposition::NoDecisionValue
        );
    }

    #[test]
    fn covered_subjects_and_frontier_comparison_are_canonical() {
        let mut first = action(
            "inspect-inventory",
            "inventory:site-a:v1",
            5,
            20,
            40,
            valuable_outcomes(),
        );
        first.covered_subject_ids = vec!["offer-b".to_owned(), "offer-a".to_owned(), "offer-a".to_owned()];
        let mut second = first.clone();
        second.covered_subject_ids.reverse();
        let a = rank_evidence_actions(&problem(vec![first])).unwrap();
        let b = rank_evidence_actions(&problem(vec![second])).unwrap();
        assert_eq!(a, b);
    }
}
