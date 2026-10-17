//! Independent finite-domain checks for the frontier-sensitivity evidence selector.
//!
//! This oracle builds canonical objective-set keys and calculates set differences
//! directly from the literal toy manifests. It does not call the selector's
//! canonicalization, frontier-difference, or ranking helpers.

use std::collections::BTreeSet;

use economic_coordination_reference::evidence_actions::{
    EvidenceAcquisitionAction, EvidenceAcquisitionProblem, EvidenceActionDisposition,
    EvidenceOutcomeBranch, EvidenceOutcomeKind, rank_evidence_actions,
};
use economic_coordination_reference::joint_optimizer::{JointObjective, RequestShortage};

fn shortage(request_id: &str, quantity: u64) -> RequestShortage {
    RequestShortage {
        request_id: request_id.to_owned(),
        unit: "kit".to_owned(),
        unfilled_quantity: quantity,
    }
}

fn objective(cost: u64, routine: u64, urgent: u64) -> JointObjective {
    JointObjective {
        currency: "USD".to_owned(),
        total_cost_minor: cost,
        // Deliberately reverse lexical order; canonicalization is part of the oracle.
        unfilled_by_request: vec![shortage("routine", routine), shortage("urgent", urgent)],
    }
}

fn canonical_frontier(frontier: &[JointObjective]) -> Vec<JointObjective> {
    let mut result = frontier.to_vec();
    for item in &mut result {
        item.unfilled_by_request.sort_by(|left, right| {
            (&left.request_id, &left.unit).cmp(&(&right.request_id, &right.unit))
        });
    }
    result.sort();
    result.dedup();
    result
}

fn symmetric_difference_size(left: &[JointObjective], right: &[JointObjective]) -> usize {
    let left_set: BTreeSet<JointObjective> = left.iter().cloned().collect();
    let right_set: BTreeSet<JointObjective> = right.iter().cloned().collect();
    left_set.difference(&right_set).count() + right_set.difference(&left_set).count()
}

/// Independent oracle returns (distinct frontier states, changed states, maximum
/// baseline/result frontier symmetric difference) from the literal branch table.
fn oracle_metrics(
    baseline: &[JointObjective],
    branches: &[Vec<JointObjective>],
) -> (usize, usize, usize) {
    let baseline = canonical_frontier(baseline);
    let unique: BTreeSet<Vec<JointObjective>> = branches
        .iter()
        .map(|frontier| canonical_frontier(frontier))
        .collect();
    let changed = unique
        .iter()
        .filter(|frontier| **frontier != baseline)
        .count();
    let maximum_delta = unique
        .iter()
        .map(|frontier| symmetric_difference_size(&baseline, frontier))
        .max()
        .unwrap_or(0);
    (unique.len(), changed, maximum_delta)
}

fn action(branches: &[Vec<JointObjective>]) -> EvidenceAcquisitionAction {
    EvidenceAcquisitionAction {
        action_id: "action-1".to_owned(),
        claim_version_key: "inventory:site-a:version-1".to_owned(),
        source_authority_class: "supplier-confirmation".to_owned(),
        covered_subject_ids: vec!["offer-a".to_owned()],
        acquisition_cost_minor: 5,
        acquisition_currency: "USD".to_owned(),
        latest_completion_at: 20,
        evidence_valid_until: 40,
        outcomes: branches
            .iter()
            .enumerate()
            .map(|(index, frontier)| EvidenceOutcomeBranch {
                outcome_id: format!("outcome-{index}"),
                kind: if index % 2 == 0 {
                    EvidenceOutcomeKind::Confirmed
                } else {
                    EvidenceOutcomeKind::Inconclusive
                },
                resulting_frontier: frontier.clone(),
            })
            .collect(),
    }
}

fn problem(action: EvidenceAcquisitionAction) -> EvidenceAcquisitionProblem {
    EvidenceAcquisitionProblem {
        evaluated_at: 10,
        decision_deadline: 30,
        input_manifest_digest: "oracle-manifest-evidence-actions-v1".to_owned(),
        acquisition_currency: "USD".to_owned(),
        baseline_frontier: vec![objective(100, 0, 0)],
        actions: vec![action],
    }
}

#[test]
fn all_sixteen_toy_branch_pairs_match_independent_frontier_sensitivity_oracle() {
    let baseline = vec![objective(100, 0, 0)];
    let states = vec![
        vec![objective(100, 0, 0)],                     // unchanged
        vec![objective(80, 0, 0)],                      // same shortage, different cost
        vec![objective(70, 1, 0), objective(90, 0, 0)], // larger frontier
        vec![],                                         // no feasible allocation
    ];

    for left in 0..states.len() {
        for right in 0..states.len() {
            let branches = vec![states[left].clone(), states[right].clone()];
            let expected = oracle_metrics(&baseline, &branches);
            let report = rank_evidence_actions(&problem(action(&branches))).unwrap();
            assert_eq!(report.assessments.len(), 1);
            let assessment = &report.assessments[0];

            assert_eq!(
                (
                    assessment.distinct_outcome_frontier_count,
                    assessment.changed_frontier_count,
                    assessment.max_frontier_symmetric_difference,
                ),
                expected,
                "branch pair ({left}, {right})"
            );
            let should_recommend = expected.0 > 1 && expected.1 > 0;
            assert_eq!(
                assessment.disposition == EvidenceActionDisposition::Recommended,
                should_recommend,
                "branch pair ({left}, {right})"
            );
        }
    }
}
