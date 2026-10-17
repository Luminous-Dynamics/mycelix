//! Ex-ante procurement strategy selection over pre-qualified contingency options.
//!
//! This bounded, effects-disabled reference enumerates which available flexible /
//! backup options to commit before disruption, then evaluates each commitment set
//! against the same frozen scenarios. It returns a Pareto frontier rather than
//! assigning uncalibrated scenarios invented probabilities or collapsing policy
//! trade-offs into an opaque scalar score.
//!
//! The second-stage evaluator is currently the recourse module. This layer is an
//! exhaustive commitment-set enumerator, not yet an independent oracle for
//! recourse correctness and not a solver for continuous reserve quantities,
//! mutually exclusive contracts, multi-request pooling, or live contracting.

use std::collections::BTreeSet;

use super::recourse::{
    BaseAllocation, ContingencyOption, RecourseError, RecourseReport, RecourseRequest,
    RecourseScenario, evaluate_recourse,
};

const MAX_STRATEGY_OPTIONS: usize = 8;
const MAX_STRATEGY_SUBSETS: u64 = 1 << MAX_STRATEGY_OPTIONS;
const MAX_STRATEGY_SEARCH_NODES: u64 = 500_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StrategyError {
    InvalidStrategyRequest,
    DuplicateOptionId,
    SearchSpaceExceeded,
    NoFeasibleStrategy,
    Recourse(RecourseError),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StrategyMetrics {
    /// Base order cost plus all selected option reservation fees.
    pub commitment_cost_minor: u64,
    /// Largest unfilled quantity in any declared scenario; not a probability.
    pub worst_case_unfilled_quantity: u32,
    /// Sum of unfilled quantities across the listed scenarios. This is a corpus
    /// diagnostic, NOT an expected shortage and NOT probability-weighted.
    pub unfilled_quantity_sum_across_scenarios: u64,
    /// Largest all-in cost reported for any scenario, including sunk commitment
    /// cost and activated recourse costs.
    pub worst_case_scenario_cost_minor: u64,
    /// Total reserved maximum activation units across selected options. This is
    /// a physical-capacity diagnostic, not a monetary penalty.
    pub reserved_quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StrategyCandidate {
    pub selected_option_ids: Vec<String>,
    pub metrics: StrategyMetrics,
    pub evaluation: RecourseReport,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StrategyReport {
    pub evaluator_profile: &'static str,
    pub objective_profile: &'static str,
    pub request_id: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    pub input_manifest_digest: String,
    pub candidate_sets_considered: u64,
    pub feasible_strategy_count: u64,
    pub infeasible_strategy_count: u64,
    /// Counts one unit for each subset evaluated, plus second-stage nodes
    /// returned by the recourse evaluator. Exceeding the caller's limit is an
    /// error, never a partial frontier labelled complete.
    pub total_search_nodes: u64,
    pub frontier: Vec<StrategyCandidate>,
}

const EVALUATOR_PROFILE: &str = "mycelix:procurement-strategy-v1";
const OBJECTIVE_PROFILE: &str =
    "pareto-minimize-commitment-cost-worst-shortage-scenario-shortage-sum-worst-scenario-cost-v1";

fn metric_vector(metrics: &StrategyMetrics) -> [u64; 5] {
    [
        metrics.commitment_cost_minor,
        u64::from(metrics.worst_case_unfilled_quantity),
        metrics.unfilled_quantity_sum_across_scenarios,
        metrics.worst_case_scenario_cost_minor,
        metrics.reserved_quantity,
    ]
}

fn dominates(left: &StrategyCandidate, right: &StrategyCandidate) -> bool {
    let left_metrics = metric_vector(&left.metrics);
    let right_metrics = metric_vector(&right.metrics);
    let no_worse_everywhere = left_metrics
        .iter()
        .zip(right_metrics.iter())
        .all(|(left, right)| left <= right);
    let strictly_better_somewhere = left_metrics
        .iter()
        .zip(right_metrics.iter())
        .any(|(left, right)| left < right);
    no_worse_everywhere && strictly_better_somewhere
}

fn build_metrics(
    evaluation: &RecourseReport,
    options: &[ContingencyOption],
) -> Result<StrategyMetrics, StrategyError> {
    let mut worst_case_unfilled_quantity = 0u32;
    let mut unfilled_quantity_sum_across_scenarios = 0u64;
    let mut worst_case_scenario_cost_minor = evaluation.baseline_and_reservation_cost_minor;

    for scenario in &evaluation.scenario_results {
        worst_case_unfilled_quantity =
            worst_case_unfilled_quantity.max(scenario.unfilled_quantity);
        unfilled_quantity_sum_across_scenarios = unfilled_quantity_sum_across_scenarios
            .checked_add(u64::from(scenario.unfilled_quantity))
            .ok_or(StrategyError::Recourse(RecourseError::ArithmeticOverflow))?;
        worst_case_scenario_cost_minor =
            worst_case_scenario_cost_minor.max(scenario.total_scenario_cost_minor);
    }

    let reserved_quantity = options.iter().try_fold(0u64, |total, option| {
        total
            .checked_add(u64::from(option.max_activation_quantity))
            .ok_or(StrategyError::Recourse(RecourseError::ArithmeticOverflow))
    })?;

    Ok(StrategyMetrics {
        commitment_cost_minor: evaluation.baseline_and_reservation_cost_minor,
        worst_case_unfilled_quantity,
        unfilled_quantity_sum_across_scenarios,
        worst_case_scenario_cost_minor,
        reserved_quantity,
    })
}

fn is_option_set_infeasible(error: RecourseError) -> bool {
    matches!(
        error,
        RecourseError::InconsistentCapacityPool
            | RecourseError::CapacityPoolOvercommitted
            | RecourseError::BaseCommitmentOverBudget
    )
}

/// Exhaustively evaluate all subsets of the supplied already-qualified candidate
/// option records. Each subset is checked by evaluate_recourse; infeasible
/// capacity/budget combinations are omitted. Invalid baseline or request input,
/// arithmetic errors, and any search-bound exhaustion fail the whole call.
///
/// max_total_search_nodes includes one unit for every subset plus all bounded
/// second-stage search nodes. Because each subset receives a fixed upper bound,
/// the function never stops early and returns a frontier that looks exhaustive.
///
/// At most eight candidate options / 256 subsets are supported in this first
/// version. Option records represent all-or-nothing reservation choices at their
/// declared maximum activation quantities; sizing partial reservations is future
/// work. Scenarios are unweighted and must not be interpreted as random draws.
pub fn evaluate_strategy(
    request: &RecourseRequest,
    baseline: &[BaseAllocation],
    available_options: &[ContingencyOption],
    scenarios: &[RecourseScenario],
    max_total_search_nodes: u64,
) -> Result<StrategyReport, StrategyError> {
    if available_options.len() > MAX_STRATEGY_OPTIONS
        || scenarios.is_empty()
        || max_total_search_nodes == 0
        || max_total_search_nodes > MAX_STRATEGY_SEARCH_NODES
    {
        return Err(StrategyError::InvalidStrategyRequest);
    }

    let mut options = available_options.to_vec();
    options.sort_by(|left, right| left.supply.offer_id.cmp(&right.supply.offer_id));

    let mut all_ids = BTreeSet::new();
    for allocation in baseline {
        if allocation.supply.offer_id.trim().is_empty()
            || !all_ids.insert(allocation.supply.offer_id.as_str())
        {
            return Err(StrategyError::DuplicateOptionId);
        }
    }
    for option in &options {
        if option.supply.offer_id.trim().is_empty()
            || !all_ids.insert(option.supply.offer_id.as_str())
        {
            return Err(StrategyError::DuplicateOptionId);
        }
    }

    let subset_count = 1u64
        .checked_shl(options.len() as u32)
        .ok_or(StrategyError::InvalidStrategyRequest)?;
    if subset_count > MAX_STRATEGY_SUBSETS || max_total_search_nodes <= subset_count {
        return Err(StrategyError::SearchSpaceExceeded);
    }

    let per_subset_limit = ((max_total_search_nodes - subset_count) / subset_count)
        .min(request.max_search_nodes);
    if per_subset_limit < scenarios.len() as u64 {
        return Err(StrategyError::SearchSpaceExceeded);
    }

    let mut candidates = Vec::new();
    let mut infeasible_strategy_count = 0u64;
    let mut total_search_nodes = subset_count;

    for mask in 0..subset_count {
        let selected: Vec<ContingencyOption> = options
            .iter()
            .enumerate()
            .filter(|(index, _)| mask & (1u64 << index) != 0)
            .map(|(_, option)| option.clone())
            .collect();

        let mut candidate_request = request.clone();
        candidate_request.max_search_nodes = per_subset_limit;

        match evaluate_recourse(&candidate_request, baseline, &selected, scenarios) {
            Ok(evaluation) => {
                total_search_nodes = total_search_nodes
                    .checked_add(evaluation.total_search_nodes)
                    .ok_or(StrategyError::Recourse(RecourseError::ArithmeticOverflow))?;
                if total_search_nodes > max_total_search_nodes {
                    return Err(StrategyError::SearchSpaceExceeded);
                }
                let metrics = build_metrics(&evaluation, &selected)?;
                candidates.push(StrategyCandidate {
                    selected_option_ids: selected
                        .iter()
                        .map(|option| option.supply.offer_id.clone())
                        .collect(),
                    metrics,
                    evaluation,
                });
            }
            Err(RecourseError::BaseCommitmentOverBudget) => {
                // No option subset can fix a baseline whose commitment alone
                // already exceeds the request budget.
                infeasible_strategy_count = infeasible_strategy_count
                    .checked_add(1)
                    .ok_or(StrategyError::Recourse(RecourseError::ArithmeticOverflow))?;
            }
            Err(error) if mask != 0 && is_option_set_infeasible(error) => {
                infeasible_strategy_count = infeasible_strategy_count
                    .checked_add(1)
                    .ok_or(StrategyError::Recourse(RecourseError::ArithmeticOverflow))?;
            }
            Err(RecourseError::SearchSpaceExceeded) => {
                // Fail closed: no partial candidate set may be called exhaustive.
                return Err(StrategyError::SearchSpaceExceeded);
            }
            Err(error) => return Err(StrategyError::Recourse(error)),
        }
    }

    if candidates.is_empty() {
        return Err(StrategyError::NoFeasibleStrategy);
    }

    candidates.sort_by(|left, right| left.selected_option_ids.cmp(&right.selected_option_ids));
    let feasible_strategy_count = candidates.len() as u64;

    let mut frontier = Vec::new();
    for (index, candidate) in candidates.iter().enumerate() {
        let dominated = candidates
            .iter()
            .enumerate()
            .any(|(other_index, other)| other_index != index && dominates(other, candidate));
        if !dominated {
            frontier.push(candidate.clone());
        }
    }
    frontier.sort_by(|left, right| left.selected_option_ids.cmp(&right.selected_option_ids));

    Ok(StrategyReport {
        evaluator_profile: EVALUATOR_PROFILE,
        objective_profile: OBJECTIVE_PROFILE,
        request_id: request.request_id.clone(),
        scenario_set_id: request.scenario_set_id.clone(),
        scenario_set_digest: request.scenario_set_digest.clone(),
        input_manifest_digest: request.input_manifest_digest.clone(),
        candidate_sets_considered: subset_count,
        feasible_strategy_count,
        infeasible_strategy_count,
        total_search_nodes,
        frontier,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EvidenceOrigin, EvidenceStatus};
    use crate::recourse::{
        BaseAllocation, ContingencyOption, RecourseRequest, RecourseScenario, SupplyEvidence,
        SupplyMode,
    };

    fn evidence(
        offer_id: &str,
        supplier_id: &str,
        pool_id: &str,
        pool_quantity: u32,
    ) -> SupplyEvidence {
        SupplyEvidence {
            offer_id: offer_id.to_owned(),
            supplier_id: supplier_id.to_owned(),
            source_uri: format!("https://example.invalid/{offer_id}"),
            source_payload_digest: format!("sha256:{offer_id}"),
            extraction_profile_id: "fixture-v1".to_owned(),
            qualification_receipt_id: format!("qual-{offer_id}"),
            capability_id: "repair-kit".to_owned(),
            capability_profile_digest: "capability-sha256".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            qualification_status: EvidenceStatus::Verified,
            qualification_origin: EvidenceOrigin::LiveDomain,
            qualification_observed_at: 10,
            qualification_valid_until: 10_000,
            quote_status: EvidenceStatus::Verified,
            quote_origin: EvidenceOrigin::LiveDomain,
            quote_observed_at: 10,
            quote_valid_until: 10_000,
            capacity_status: EvidenceStatus::Verified,
            capacity_origin: EvidenceOrigin::LiveDomain,
            capacity_pool_id: pool_id.to_owned(),
            capacity_snapshot_digest: format!("capacity-{pool_id}-v1"),
            capacity_pool_quantity: pool_quantity,
            capacity_observed_at: 10,
            capacity_valid_until: 10_000,
            failure_domains: vec![format!("site:{supplier_id}")],
            failure_domain_status: EvidenceStatus::Verified,
            failure_domain_origin: EvidenceOrigin::LiveDomain,
            failure_domain_observed_at: 10,
            failure_domain_valid_until: 10_000,
        }
    }

    fn fixture_request() -> RecourseRequest {
        RecourseRequest {
            request_id: "strategy-fixture".to_owned(),
            capability_id: "repair-kit".to_owned(),
            capability_profile_digest: "capability-sha256".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            required_evidence_origin: EvidenceOrigin::LiveDomain,
            evaluated_at: 10,
            required_quantity: 4,
            deliver_by: 500,
            max_total_cost_minor: 1_000,
            max_qualification_age_secs: 1_000,
            max_quote_age_secs: 1_000,
            max_capacity_age_secs: 1_000,
            max_dependency_age_secs: 1_000,
            policy_id: "resilience-policy-v1".to_owned(),
            policy_digest: "policy-sha256".to_owned(),
            scenario_set_id: "two-scenario-fixture".to_owned(),
            scenario_set_digest: "scenario-sha256".to_owned(),
            input_manifest_digest: "inputs-sha256".to_owned(),
            max_search_nodes: 10_000,
        }
    }

    fn fixture_baseline() -> Vec<BaseAllocation> {
        vec![BaseAllocation {
            supply: evidence("base-offer", "base-supplier", "base-pool", 4),
            quantity: 4,
            max_allocatable_quantity: 4,
            minimum_order_quantity: 1,
            order_increment: 1,
            unit_cost_minor: 10,
            fixed_order_cost_minor: 0,
            arrival_at: 50,
        }]
    }

    fn fixture_option(id: &str, supplier: &str) -> ContingencyOption {
        ContingencyOption {
            supply: evidence(id, supplier, &format!("{id}-pool"), 4),
            mode: SupplyMode::Backup,
            max_activation_quantity: 4,
            minimum_order_quantity: 1,
            order_increment: 1,
            activation_lead_secs: 1,
            transit_secs: 1,
            unit_cost_minor: 2,
            activation_fee_minor: 1,
            reservation_fee_minor: 2,
        }
    }

    fn fixture_scenarios() -> Vec<RecourseScenario> {
        vec![
            RecourseScenario {
                scenario_id: "normal".to_owned(),
                observed_at: 20,
                failed_domains: Vec::new(),
            },
            RecourseScenario {
                scenario_id: "base-failure".to_owned(),
                observed_at: 20,
                failed_domains: vec!["supplier:base-supplier".to_owned()],
            },
        ]
    }

    #[test]
    fn frontier_exposes_cost_resilience_tradeoff_instead_of_scalar_winner() {
        let report = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &[fixture_option("backup-a", "backup-supplier")],
            &fixture_scenarios(),
            10_000,
        )
        .expect("small fixture has a complete bounded frontier");

        assert_eq!(report.candidate_sets_considered, 2);
        assert_eq!(report.feasible_strategy_count, 2);
        assert_eq!(report.frontier.len(), 2);
        assert!(report.frontier.iter().any(|candidate| {
            candidate.selected_option_ids.is_empty()
                && candidate.metrics.worst_case_unfilled_quantity == 4
                && candidate.metrics.commitment_cost_minor == 40
        }));
        assert!(report.frontier.iter().any(|candidate| {
            candidate.selected_option_ids == vec!["backup-a".to_owned()]
                && candidate.metrics.worst_case_unfilled_quantity == 0
                && candidate.metrics.commitment_cost_minor == 42
                && candidate.metrics.worst_case_scenario_cost_minor == 51
        }));
    }

    #[test]
    fn option_and_scenario_input_order_do_not_change_canonical_frontier() {
        let options = vec![
            fixture_option("backup-b", "backup-b-supplier"),
            fixture_option("backup-a", "backup-a-supplier"),
        ];
        let scenarios = fixture_scenarios();
        let first = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &options,
            &scenarios,
            100_000,
        )
        .expect("bounded fixture");
        let second = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &[options[1].clone(), options[0].clone()],
            &[scenarios[1].clone(), scenarios[0].clone()],
            100_000,
        )
        .expect("permuted bounded fixture");
        assert_eq!(first, second);
    }

    #[test]
    fn duplicate_offer_identity_fails_closed() {
        let result = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &[fixture_option("base-offer", "other-supplier")],
            &fixture_scenarios(),
            10_000,
        );
        assert_eq!(result, Err(StrategyError::DuplicateOptionId));
    }

    #[test]
    fn too_small_global_budget_returns_no_partial_frontier() {
        let result = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &[fixture_option("backup-a", "backup-supplier")],
            &fixture_scenarios(),
            3,
        );
        assert_eq!(result, Err(StrategyError::SearchSpaceExceeded));
    }

    #[test]
    fn two_option_frontier_matches_independent_subset_and_quantity_oracle() {
        let options = [
            fixture_option("backup-a", "backup-a-supplier"),
            fixture_option("backup-b", "backup-b-supplier"),
        ];
        let report = evaluate_strategy(
            &fixture_request(),
            &fixture_baseline(),
            &options,
            &fixture_scenarios(),
            100_000,
        )
        .expect("small case must complete");

        // Independent toy oracle: enumerate subsets and every integer activation
        // quantity directly. It intentionally does not call evaluate_recourse,
        // build_metrics, metric_vector, or dominates.
        let ids = ["backup-a", "backup-b"];
        let mut oracle_candidates: Vec<(Vec<String>, [u64; 5])> = Vec::new();
        for mask in 0u64..4 {
            let selected_ids: Vec<String> = ids
                .iter()
                .enumerate()
                .filter(|(index, _)| mask & (1 << index) != 0)
                .map(|(_, id)| (*id).to_owned())
                .collect();
            let selected_count = selected_ids.len() as u64;
            let commitment_cost = 40 + 2 * selected_count;

            let mut max_recovered = 0u32;
            let mut min_recourse_cost = 0u64;
            let q_a_values: Vec<u32> = if mask & 1 != 0 {
                (0..=4).collect()
            } else {
                vec![0]
            };
            let q_b_values: Vec<u32> = if mask & 2 != 0 {
                (0..=4).collect()
            } else {
                vec![0]
            };

            for q_a in &q_a_values {
                for q_b in &q_b_values {
                    let recovered = *q_a + *q_b;
                    if recovered > 4 {
                        continue;
                    }
                    let activated_options = (if *q_a > 0 { 1u64 } else { 0 }) + (if *q_b > 0 { 1u64 } else { 0 });
                    let activation_cost = u64::from(recovered) * 2 + activated_options;
                    if recovered > max_recovered
                        || (recovered == max_recovered && activation_cost < min_recourse_cost)
                    {
                        max_recovered = recovered;
                        min_recourse_cost = activation_cost;
                    }
                }
            }

            // The fixture scenarios are exactly no disruption and total base
            // supplier failure. All selected backups arrive by the deadline.
            let unfilled = 4 - max_recovered;
            let worst_scenario_cost = commitment_cost + min_recourse_cost;
            oracle_candidates.push((
                selected_ids,
                [
                    commitment_cost,
                    u64::from(unfilled),
                    u64::from(unfilled),
                    worst_scenario_cost,
                    selected_count * 4,
                ],
            ));
        }

        let mut expected: Vec<(Vec<String>, [u64; 5])> = oracle_candidates
            .iter()
            .enumerate()
            .filter(|(index, candidate)| {
                !oracle_candidates.iter().enumerate().any(|(other_index, other)| {
                    if *index == other_index {
                        return false;
                    }
                    let no_worse = other
                        .1
                        .iter()
                        .zip(candidate.1.iter())
                        .all(|(left, right)| left <= right);
                    let strictly_better = other
                        .1
                        .iter()
                        .zip(candidate.1.iter())
                        .any(|(left, right)| left < right);
                    no_worse && strictly_better
                })
            })
            .map(|(_, candidate)| candidate.clone())
            .collect();
        expected.sort_by(|left, right| left.0.cmp(&right.0));

        let mut actual: Vec<(Vec<String>, [u64; 5])> = report
            .frontier
            .iter()
            .map(|candidate| {
                (
                    candidate.selected_option_ids.clone(),
                    [
                        candidate.metrics.commitment_cost_minor,
                        u64::from(candidate.metrics.worst_case_unfilled_quantity),
                        candidate.metrics.unfilled_quantity_sum_across_scenarios,
                        candidate.metrics.worst_case_scenario_cost_minor,
                        candidate.metrics.reserved_quantity,
                    ],
                )
            })
            .collect();
        actual.sort_by(|left, right| left.0.cmp(&right.0));
        assert_eq!(actual, expected);
    }

}
