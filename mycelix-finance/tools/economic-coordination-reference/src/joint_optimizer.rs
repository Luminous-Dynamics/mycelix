//! Bounded exact joint allocation over concurrent procurement requests.
//!
//! This module returns a Pareto frontier over one common-currency total cost and
//! one shortage axis per request. It reuses the joint feasibility checker at each
//! leaf, but does not use that checker as an independent oracle. Search-bound
//! exhaustion is an error; no partial frontier is presented as complete.

use super::joint_capacity::{
    CapabilityMapping, JointAllocationError, JointAllocationInput, JointAllocationReport,
    JointAssignment, JointOffer, JointRequest, SharedCapacityPool, validate_joint_allocation,
};

const MAX_EXACT_SEARCH_NODES: u64 = 500_000;
const OPTIMIZER_PROFILE: &str = "mycelix:joint-allocation-exact-pareto:v1";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointOptimizationProblem {
    pub evaluated_at: u64,
    /// Caller-supplied digest binding the complete frozen problem manifest.
    pub input_manifest_digest: String,
    pub requests: Vec<JointRequest>,
    pub offers: Vec<JointOffer>,
    pub capacity_pools: Vec<SharedCapacityPool>,
    pub capability_mappings: Vec<CapabilityMapping>,
    /// Hard search ceiling. The implementation never silently truncates results.
    pub search_node_limit: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointOptimizationError {
    InvalidSearchBound,
    UnsupportedMixedCurrency,
    InvalidProblem(JointAllocationError),
    SearchSpaceExceeded { visited_nodes: u64, limit: u64 },
    NoFeasibleCandidate,
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct RequestShortage {
    pub request_id: String,
    pub unit: String,
    pub unfilled_quantity: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct JointObjective {
    pub currency: String,
    pub total_cost_minor: u64,
    /// Aligned in canonical request-ID order; unlike units are never summed.
    pub unfilled_by_request: Vec<RequestShortage>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointOptimizationCandidate {
    pub objective: JointObjective,
    /// Canonical representative for this unique objective vector.
    pub assignments: Vec<JointAssignment>,
    pub validated_allocation: JointAllocationReport,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointOptimizationReport {
    pub evaluator_profile: &'static str,
    pub input_manifest_digest: String,
    pub currency: String,
    pub request_count: usize,
    pub offer_count: usize,
    pub complete: bool,
    pub search_node_limit: u64,
    pub search_node_count: u64,
    pub feasible_candidate_count: u64,
    pub infeasible_candidate_count: u64,
    /// A Pareto frontier of unique objective vectors, not a scalar winner.
    pub frontier: Vec<JointOptimizationCandidate>,
}

#[derive(Clone, Debug)]
struct Edge {
    request_index: usize,
    offer_index: usize,
    pool_index: usize,
    max_quantity: u32,
}

#[derive(Default)]
struct SearchState {
    nodes: u64,
    feasible: u64,
    infeasible: u64,
    exhausted: bool,
    frontier: Vec<JointOptimizationCandidate>,
}

struct SearchContext<'a> {
    problem: &'a JointOptimizationProblem,
    edges: Vec<Edge>,
    request_positions: Vec<usize>,
    offer_positions: Vec<usize>,
    pool_positions: Vec<usize>,
    served_by_request: Vec<u32>,
    cost_by_request: Vec<u64>,
    allocated_by_offer: Vec<u32>,
    allocated_by_pool: Vec<u32>,
    assignments: Vec<JointAssignment>,
    state: SearchState,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn build_checker_input(
    problem: &JointOptimizationProblem,
    assignments: &[JointAssignment],
) -> JointAllocationInput {
    JointAllocationInput {
        evaluated_at: problem.evaluated_at,
        input_manifest_digest: problem.input_manifest_digest.clone(),
        requests: problem.requests.clone(),
        offers: problem.offers.clone(),
        capacity_pools: problem.capacity_pools.clone(),
        capability_mappings: problem.capability_mappings.clone(),
        assignments: assignments.to_vec(),
    }
}

fn common_currency(problem: &JointOptimizationProblem) -> Result<String, JointOptimizationError> {
    let Some(first) = problem.requests.first() else {
        return Err(JointOptimizationError::InvalidProblem(
            JointAllocationError::InvalidManifest,
        ));
    };
    let currency = first.currency.clone();
    if !non_empty(&currency)
        || problem.requests.iter().any(|request| request.currency != currency)
        || problem.offers.iter().any(|offer| offer.currency != currency)
    {
        return Err(JointOptimizationError::UnsupportedMixedCurrency);
    }
    Ok(currency)
}

fn pair_is_mapped(
    mappings: &[CapabilityMapping],
    request: &JointRequest,
    offer: &JointOffer,
) -> bool {
    mappings.iter().any(|mapping| {
        mapping.request_id == request.request_id && mapping.offer_id == offer.offer_id
    })
}

fn canonical_edges(problem: &JointOptimizationProblem) -> Result<Vec<Edge>, JointOptimizationError> {
    let request_positions: std::collections::BTreeMap<&str, usize> = problem
        .requests
        .iter()
        .enumerate()
        .map(|(index, request)| (request.request_id.as_str(), index))
        .collect();
    let offer_positions: std::collections::BTreeMap<&str, usize> = problem
        .offers
        .iter()
        .enumerate()
        .map(|(index, offer)| (offer.offer_id.as_str(), index))
        .collect();
    let pool_positions: std::collections::BTreeMap<&str, usize> = problem
        .capacity_pools
        .iter()
        .enumerate()
        .map(|(index, pool)| (pool.capacity_pool_id.as_str(), index))
        .collect();

    let mut edges = Vec::new();
    for (request_index, request) in problem.requests.iter().enumerate() {
        for (offer_index, offer) in problem.offers.iter().enumerate() {
            if !pair_is_mapped(&problem.capability_mappings, request, offer)
                || request.unit != offer.unit
                || request.currency != offer.currency
                || offer.arrives_at > request.deliver_by
            {
                continue;
            }
            let Some(&pool_index) = pool_positions.get(offer.capacity_pool_id.as_str()) else {
                return Err(JointOptimizationError::InvalidProblem(
                    JointAllocationError::MissingCapacityPool,
                ));
            };
            let mut maximum = request
                .required_quantity
                .min(offer.max_allocatable_quantity)
                .min(problem.capacity_pools[pool_index].capacity_quantity);
            if offer.unit_cost_minor > 0 {
                let budget_quantity = request.max_total_cost_minor / offer.unit_cost_minor;
                maximum = maximum.min(u32::try_from(budget_quantity).unwrap_or(u32::MAX));
            }
            if maximum > 0 {
                edges.push(Edge {
                    request_index,
                    offer_index,
                    pool_index,
                    max_quantity: maximum,
                });
            }
        }
    }
    edges.sort_by(|left, right| {
        let left_key = (
            problem.requests[left.request_index].request_id.as_str(),
            problem.offers[left.offer_index].offer_id.as_str(),
        );
        let right_key = (
            problem.requests[right.request_index].request_id.as_str(),
            problem.offers[right.offer_index].offer_id.as_str(),
        );
        left_key.cmp(&right_key)
    });

    // These ID position maps are constructed here as an explicit duplicate-ID
    // defense if the checker contract changes; its empty-plan validation runs first.
    if request_positions.len() != problem.requests.len()
        || offer_positions.len() != problem.offers.len()
        || pool_positions.len() != problem.capacity_pools.len()
    {
        return Err(JointOptimizationError::InvalidProblem(
            JointAllocationError::DuplicateRequestId,
        ));
    }
    Ok(edges)
}

fn objective_from_report(
    report: &JointAllocationReport,
    currency: &str,
) -> Result<JointObjective, JointOptimizationError> {
    if report.costs_by_currency.len() > 1
        || report
            .costs_by_currency
            .iter()
            .any(|item| item.currency != currency)
    {
        return Err(JointOptimizationError::UnsupportedMixedCurrency);
    }
    let total_cost_minor = report
        .costs_by_currency
        .first()
        .map(|item| item.allocated_cost_minor)
        .unwrap_or(0);
    let unfilled_by_request = report
        .requests
        .iter()
        .map(|request| RequestShortage {
            request_id: request.request_id.clone(),
            unit: request.unit.clone(),
            unfilled_quantity: request.unfilled_quantity,
        })
        .collect();
    Ok(JointObjective {
        currency: currency.to_owned(),
        total_cost_minor,
        unfilled_by_request,
    })
}

fn dominates(left: &JointObjective, right: &JointObjective) -> bool {
    if left.currency != right.currency
        || left.unfilled_by_request.len() != right.unfilled_by_request.len()
        || left.total_cost_minor > right.total_cost_minor
    {
        return false;
    }
    let mut strictly_better = left.total_cost_minor < right.total_cost_minor;
    for (left_shortage, right_shortage) in left
        .unfilled_by_request
        .iter()
        .zip(&right.unfilled_by_request)
    {
        if left_shortage.request_id != right_shortage.request_id
            || left_shortage.unit != right_shortage.unit
            || left_shortage.unfilled_quantity > right_shortage.unfilled_quantity
        {
            return false;
        }
        strictly_better |= left_shortage.unfilled_quantity < right_shortage.unfilled_quantity;
    }
    strictly_better
}

fn assignment_signature(
    assignments: &[JointAssignment],
) -> Vec<(String, String, u32)> {
    let mut signature: Vec<_> = assignments
        .iter()
        .map(|assignment| {
            (
                assignment.request_id.clone(),
                assignment.offer_id.clone(),
                assignment.quantity,
            )
        })
        .collect();
    signature.sort();
    signature
}

fn insert_frontier(
    frontier: &mut Vec<JointOptimizationCandidate>,
    candidate: JointOptimizationCandidate,
) {
    // An identical vector is represented by the lexicographically smallest
    // canonical assignment; duplicate vectors do not multiply the Pareto set.
    if let Some(position) = frontier
        .iter()
        .position(|existing| existing.objective == candidate.objective)
    {
        if assignment_signature(&candidate.assignments)
            < assignment_signature(&frontier[position].assignments)
        {
            frontier[position] = candidate;
        }
        return;
    }
    if frontier
        .iter()
        .any(|existing| dominates(&existing.objective, &candidate.objective))
    {
        return;
    }
    frontier.retain(|existing| !dominates(&candidate.objective, &existing.objective));
    frontier.push(candidate);
}

fn visit(context: &mut SearchContext<'_>, edge_index: usize) -> Result<(), JointOptimizationError> {
    if context.state.exhausted {
        return Ok(());
    }
    if context.state.nodes >= context.problem.search_node_limit {
        context.state.exhausted = true;
        return Ok(());
    }
    context.state.nodes = context
        .state
        .nodes
        .checked_add(1)
        .ok_or(JointOptimizationError::ArithmeticOverflow)?;

    if edge_index == context.edges.len() {
        let checker_input = build_checker_input(context.problem, &context.assignments);
        match validate_joint_allocation(&checker_input) {
            Ok(validated_allocation) => {
                context.state.feasible = context
                    .state
                    .feasible
                    .checked_add(1)
                    .ok_or(JointOptimizationError::ArithmeticOverflow)?;
                let currency = context
                    .problem
                    .requests
                    .first()
                    .map(|request| request.currency.as_str())
                    .unwrap_or_default();
                let objective = objective_from_report(&validated_allocation, currency)?;
                insert_frontier(
                    &mut context.state.frontier,
                    JointOptimizationCandidate {
                        objective,
                        assignments: validated_allocation
                            .assignments
                            .iter()
                            .map(|witness| JointAssignment {
                                request_id: witness.request_id.clone(),
                                offer_id: witness.offer_id.clone(),
                                quantity: witness.quantity,
                            })
                            .collect(),
                        validated_allocation,
                    },
                );
            }
            Err(_) => {
                context.state.infeasible = context
                    .state
                    .infeasible
                    .checked_add(1)
                    .ok_or(JointOptimizationError::ArithmeticOverflow)?;
            }
        }
        return Ok(());
    }

    let edge = context.edges[edge_index].clone();
    let request = &context.problem.requests[edge.request_index];
    let offer = &context.problem.offers[edge.offer_index];
    let pool = &context.problem.capacity_pools[edge.pool_index];
    let demand_remaining = request
        .required_quantity
        .saturating_sub(context.served_by_request[edge.request_index]);
    let offer_remaining = offer
        .max_allocatable_quantity
        .saturating_sub(context.allocated_by_offer[edge.offer_index]);
    let pool_remaining = pool
        .capacity_quantity
        .saturating_sub(context.allocated_by_pool[edge.pool_index]);
    let budget_remaining = request
        .max_total_cost_minor
        .saturating_sub(context.cost_by_request[edge.request_index]);
    let mut maximum = edge
        .max_quantity
        .min(demand_remaining)
        .min(offer_remaining)
        .min(pool_remaining);
    if offer.unit_cost_minor > 0 {
        let budget_quantity = budget_remaining / offer.unit_cost_minor;
        maximum = maximum.min(u32::try_from(budget_quantity).unwrap_or(u32::MAX));
    }

    for quantity in 0..=maximum {
        let line_cost = offer
            .unit_cost_minor
            .checked_mul(u64::from(quantity))
            .ok_or(JointOptimizationError::ArithmeticOverflow)?;
        let next_cost = context.cost_by_request[edge.request_index]
            .checked_add(line_cost)
            .ok_or(JointOptimizationError::ArithmeticOverflow)?;
        if next_cost > request.max_total_cost_minor {
            continue;
        }

        context.served_by_request[edge.request_index] += quantity;
        context.cost_by_request[edge.request_index] = next_cost;
        context.allocated_by_offer[edge.offer_index] += quantity;
        context.allocated_by_pool[edge.pool_index] += quantity;
        if quantity > 0 {
            context.assignments.push(JointAssignment {
                request_id: request.request_id.clone(),
                offer_id: offer.offer_id.clone(),
                quantity,
            });
        }

        visit(context, edge_index + 1)?;

        if quantity > 0 {
            context.assignments.pop();
        }
        context.served_by_request[edge.request_index] -= quantity;
        context.cost_by_request[edge.request_index] -= line_cost;
        context.allocated_by_offer[edge.offer_index] -= quantity;
        context.allocated_by_pool[edge.pool_index] -= quantity;
        if context.state.exhausted {
            break;
        }
    }
    Ok(())
}

/// Enumerate feasible integer allocations and return the complete Pareto frontier.
///
/// The node limit is mandatory and capped. If exhausted, the function returns an
/// explicit error with no candidate frontier, so a partial search cannot be
/// mislabeled exact. Requests must share one currency; FX conversion is outside v1.
pub fn optimize_joint_allocation(
    problem: &JointOptimizationProblem,
) -> Result<JointOptimizationReport, JointOptimizationError> {
    if problem.search_node_limit == 0 || problem.search_node_limit > MAX_EXACT_SEARCH_NODES {
        return Err(JointOptimizationError::InvalidSearchBound);
    }
    let currency = common_currency(problem)?;

    // Validate the complete manifest independently of any nonzero allocation.
    let empty_plan = build_checker_input(problem, &[]);
    validate_joint_allocation(&empty_plan)
        .map_err(JointOptimizationError::InvalidProblem)?;
    let edges = canonical_edges(problem)?;

    let request_positions: Vec<usize> = (0..problem.requests.len()).collect();
    let offer_positions: Vec<usize> = (0..problem.offers.len()).collect();
    let pool_positions: Vec<usize> = (0..problem.capacity_pools.len()).collect();
    let mut context = SearchContext {
        problem,
        edges,
        request_positions,
        offer_positions,
        pool_positions,
        served_by_request: vec![0; problem.requests.len()],
        cost_by_request: vec![0; problem.requests.len()],
        allocated_by_offer: vec![0; problem.offers.len()],
        allocated_by_pool: vec![0; problem.capacity_pools.len()],
        assignments: Vec::new(),
        state: SearchState::default(),
    };
    visit(&mut context, 0)?;

    if context.state.exhausted {
        return Err(JointOptimizationError::SearchSpaceExceeded {
            visited_nodes: context.state.nodes,
            limit: problem.search_node_limit,
        });
    }
    if context.state.frontier.is_empty() {
        return Err(JointOptimizationError::NoFeasibleCandidate);
    }
    context.state.frontier.sort_by(|left, right| {
        left.objective
            .cmp(&right.objective)
            .then_with(|| assignment_signature(&left.assignments).cmp(&assignment_signature(&right.assignments)))
    });

    Ok(JointOptimizationReport {
        evaluator_profile: OPTIMIZER_PROFILE,
        input_manifest_digest: problem.input_manifest_digest.clone(),
        currency,
        request_count: problem.requests.len(),
        offer_count: problem.offers.len(),
        complete: true,
        search_node_limit: problem.search_node_limit,
        search_node_count: context.state.nodes,
        feasible_candidate_count: context.state.feasible,
        infeasible_candidate_count: context.state.infeasible,
        frontier: context.state.frontier,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        JointOptimizationError, JointOptimizationProblem, JointObjective, JointOptimizationReport,
        JointOptimizationCandidate, RequestShortage, optimize_joint_allocation,
    };
    use crate::joint_capacity::{CapabilityMapping, JointAllocationError, JointOffer, JointRequest, SharedCapacityPool};
    use crate::{EvidenceOrigin, EvidenceStatus};
    use std::collections::BTreeSet;

    fn request(id: &str) -> JointRequest {
        JointRequest {
            request_id: id.to_owned(),
            capability_id: "repair-kit".to_owned(),
            capability_profile_digest: "repair-kit-profile-v1".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            required_quantity: 1,
            max_total_cost_minor: 100,
            deliver_by: 500,
            priority_rank: if id == "urgent" { 1 } else { 10 },
        }
    }

    fn offer(id: &str, cost: u64) -> JointOffer {
        JointOffer {
            offer_id: id.to_owned(),
            supplier_id: format!("supplier-{id}"),
            source_uri: format!("https://example.invalid/{id}"),
            source_payload_digest: format!("sha256-{id}"),
            qualification_receipt_id: format!("qualification-{id}"),
            offered_capability_id: "repair-kit".to_owned(),
            offered_profile_digest: "repair-kit-profile-v1".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            capacity_pool_id: "shared-pool".to_owned(),
            max_allocatable_quantity: 1,
            minimum_order_quantity: 1,
            order_increment: 1,
            unit_cost_minor: cost,
            arrives_at: 100,
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            evidence_observed_at: 10,
            evidence_valid_until: 1_000,
        }
    }

    fn pool(capacity: u32) -> SharedCapacityPool {
        SharedCapacityPool {
            capacity_pool_id: "shared-pool".to_owned(),
            unit: "kit".to_owned(),
            time_bucket: "week-44".to_owned(),
            capacity_quantity: capacity.max(1),
            snapshot_digest: "sha256-pool-snapshot".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }
    }

    fn mapping(request_id: &str, offer_id: &str) -> CapabilityMapping {
        CapabilityMapping {
            mapping_id: format!("map-{request_id}-{offer_id}"),
            mapping_digest: format!("sha256-map-{request_id}-{offer_id}"),
            request_id: request_id.to_owned(),
            offer_id: offer_id.to_owned(),
            requested_capability_id: "repair-kit".to_owned(),
            requested_profile_digest: "repair-kit-profile-v1".to_owned(),
            offered_capability_id: "repair-kit".to_owned(),
            offered_profile_digest: "repair-kit-profile-v1".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }
    }

    fn problem(pool_capacity: u32) -> JointOptimizationProblem {
        let requests = vec![request("urgent"), request("routine")];
        let offers = vec![offer("offer-a", 5), offer("offer-b", 8)];
        let mappings = requests
            .iter()
            .flat_map(|request| offers.iter().map(move |offer| mapping(&request.request_id, &offer.offer_id)))
            .collect();
        JointOptimizationProblem {
            evaluated_at: 20,
            input_manifest_digest: "sha256-exact-joint-v1".to_owned(),
            requests,
            offers,
            capacity_pools: vec![pool(pool_capacity)],
            capability_mappings: mappings,
            search_node_limit: 100_000,
        }
    }

    fn objective_key(objective: &JointObjective) -> (u64, Vec<(String, String, u64)>) {
        (
            objective.total_cost_minor,
            objective
                .unfilled_by_request
                .iter()
                .map(|item| (item.request_id.clone(), item.unit.clone(), item.unfilled_quantity))
                .collect(),
        )
    }

    fn dominates_key(
        left: &(u64, Vec<(String, String, u64)>),
        right: &(u64, Vec<(String, String, u64)>),
    ) -> bool {
        if left.0 > right.0 || left.1.len() != right.1.len() {
            return false;
        }
        let mut strict = left.0 < right.0;
        for (l, r) in left.1.iter().zip(&right.1) {
            if l.0 != r.0 || l.1 != r.1 || l.2 > r.2 {
                return false;
            }
            strict |= l.2 < r.2;
        }
        strict
    }

    /// Independent hand-enumerated binary oracle. It deliberately does not call
    /// the solver or feasibility checker; it enumerates 16 literal 2x2 masks.
    fn independent_frontier(pool_capacity: u32) -> BTreeSet<(u64, Vec<(String, String, u64)>)> {
        let costs = [5u64, 8, 5, 8];
        let edge_request = [0usize, 0, 1, 1];
        let edge_offer = [0usize, 1, 0, 1];
        let mut feasible = BTreeSet::new();
        for mask in 0u8..16 {
            let mut request_quantity = [0u32; 2];
            let mut offer_quantity = [0u32; 2];
            let mut total_pool = 0u32;
            let mut cost = 0u64;
            for edge in 0..4 {
                if mask & (1 << edge) == 0 {
                    continue;
                }
                request_quantity[edge_request[edge]] += 1;
                offer_quantity[edge_offer[edge]] += 1;
                total_pool += 1;
                cost += costs[edge];
            }
            if request_quantity.iter().any(|quantity| *quantity > 1)
                || offer_quantity.iter().any(|quantity| *quantity > 1)
                || total_pool > pool_capacity
            {
                continue;
            }
            feasible.insert((
                cost,
                vec![
                    ("routine".to_owned(), "kit".to_owned(), u64::from(1 - request_quantity[1])),
                    ("urgent".to_owned(), "kit".to_owned(), u64::from(1 - request_quantity[0])),
                ],
            ));
        }
        let all: Vec<_> = feasible.iter().cloned().collect();
        feasible
            .iter()
            .filter(|candidate| !all.iter().any(|other| other != *candidate && dominates_key(other, candidate)))
            .cloned()
            .collect()
    }

    fn actual_frontier(report: &JointOptimizationReport) -> BTreeSet<(u64, Vec<(String, String, u64)>)> {
        report.frontier.iter().map(|candidate| objective_key(&candidate.objective)).collect()
    }

    #[test]
    fn exact_frontier_matches_independent_oracle_for_every_binary_matrix_and_pool_ceiling() {
        for pool_capacity in 1..=2 {
            let report = optimize_joint_allocation(&problem(pool_capacity)).unwrap();
            assert!(report.complete);
            assert_eq!(actual_frontier(&report), independent_frontier(pool_capacity));
        }
    }

    #[test]
    fn exhausted_search_returns_error_not_a_partial_frontier() {
        let mut input = problem(2);
        input.search_node_limit = 2;
        assert_eq!(
            optimize_joint_allocation(&input),
            Err(JointOptimizationError::SearchSpaceExceeded {
                visited_nodes: 2,
                limit: 2
            })
        );
    }

    #[test]
    fn mixed_currency_problem_is_rejected_before_scalar_cost_comparison() {
        let mut input = problem(2);
        input.offers[1].currency = "EUR".to_owned();
        assert_eq!(
            optimize_joint_allocation(&input),
            Err(JointOptimizationError::UnsupportedMixedCurrency)
        );
    }

    #[test]
    fn permuting_input_vectors_does_not_change_canonical_frontier() {
        let first = problem(2);
        let mut second = problem(2);
        second.requests.reverse();
        second.offers.reverse();
        second.capability_mappings.reverse();
        second.capacity_pools.reverse();
        assert_eq!(
            optimize_joint_allocation(&first),
            optimize_joint_allocation(&second)
        );
    }

    #[test]
    fn aggregate_minimum_order_lots_are_enforced_by_feasibility_gate() {
        let mut input = problem(2);
        input.offers[0].minimum_order_quantity = 2;
        input.offers[0].max_allocatable_quantity = 2;
        input.offers[0].order_increment = 2;
        input.requests[0].required_quantity = 1;
        // A single unit from offer A cannot satisfy its minimum lot, so the
        // optimized frontier must not contain that allocation.
        let report = optimize_joint_allocation(&input).unwrap();
        assert!(report.frontier.iter().all(|candidate| {
            candidate.assignments.iter().all(|assignment| {
                assignment.offer_id != "offer-a" || assignment.quantity != 1
            })
        }));
    }

    #[test]
    fn missing_mapping_never_generates_an_assignment_edge() {
        let mut input = problem(2);
        input.capability_mappings.retain(|mapping| !(mapping.request_id == "urgent" && mapping.offer_id == "offer-a"));
        let report = optimize_joint_allocation(&input).unwrap();
        assert!(report.frontier.iter().all(|candidate| candidate.assignments.iter().all(|assignment| {
            assignment.request_id != "urgent" || assignment.offer_id != "offer-a"
        })));
    }
}
