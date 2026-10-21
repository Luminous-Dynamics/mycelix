#![forbid(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! FIN-ECO-005E2 route-aware clearing allocation reference kernel.
//!
//! This is a pure integer min-cost max-flow planner for one pre-qualified netting set
//! and one asset. It is not a ledger, legal-netting opinion, credit authority, or
//! payment executor. A returned allocation is a proposal; it is not a settlement receipt.
//!
//! The optimizer assumes linear, non-negative route cost per minor unit and divisible
//! integer-minor-unit transfers. Fixed-per-leg fees, FX spreads, deadlines, defaults,
//! multihop routing, and nonlinear tariff rules are deliberately outside this first model.
//! Cost units are comparative weights, not a quoted fiat fee.

pub mod fixed_charge;
pub mod queue;

use std::cmp::Reverse;
use std::collections::{BTreeMap, BinaryHeap};
use std::error::Error;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorityQualification {
    /// The caller has not established legal/contractual authority for this netting set.
    NotQualified,
    /// Caller assertion that authority was qualified elsewhere. The reference is for audit
    /// linkage only; this crate does not authenticate the evidence or interpret the law.
    ExternallyQualified { evidence_ref: String },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ClearingContext {
    pub network_id: String,
    pub asset_id: String,
    pub netting_set_id: String,
    pub policy_version: String,
    pub authority: AuthorityQualification,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParticipantNet {
    pub member_id: String,
    /// Positive means the participant owes the netting set; negative means it is owed.
    pub net_minor: i64,
    /// Externally evidenced liquidity available for this settlement window.
    pub available_liquidity_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettlementRoute {
    pub route_id: String,
    pub network_id: String,
    pub asset_id: String,
    pub netting_set_id: String,
    pub policy_version: String,
    pub from_member: String,
    pub to_member: String,
    pub enabled: bool,
    pub capacity_minor: u64,
    /// Non-negative linear optimization weight per minor unit; not a provider quote.
    pub cost_units_per_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RouteExclusionReason {
    Disabled,
    ProfileMismatch,
    ZeroCapacity,
    WrongDirectionOrRoles,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExcludedRoute {
    pub route_id: String,
    pub reason: RouteExclusionReason,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettlementLeg {
    pub route_id: String,
    pub from_member: String,
    pub to_member: String,
    pub amount_minor: u64,
    pub cost_units_per_minor: u64,
    pub modeled_cost_units: u128,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Residual {
    pub member_id: String,
    pub amount_minor: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AllocationPlan {
    pub network_id: String,
    pub asset_id: String,
    pub netting_set_id: String,
    pub policy_version: String,
    pub authority_evidence_ref: String,
    pub legs: Vec<SettlementLeg>,
    pub total_allocated_minor: u128,
    pub total_modeled_cost_units: u128,
    pub residual_debtors: Vec<Residual>,
    pub residual_creditors: Vec<Residual>,
    /// True only when every supplied non-zero net position is covered by the modeled
    /// route capacities and externally supplied debtor liquidity. This is not finality.
    pub fully_allocatable: bool,
    pub excluded_routes: Vec<ExcludedRoute>,
    pub deduplicated_route_observations: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OptimizeError {
    InvalidContext(&'static str),
    AuthorityNotQualified,
    InvalidParticipant(String),
    DuplicateParticipant(String),
    InvalidRoute(String),
    UnknownRouteParticipant(String),
    ConflictingDuplicateRoute(String),
    UnbalancedNetPositions {
        owed_minor: u128,
        receivable_minor: u128,
    },
    ArithmeticOverflow,
    CostOverflow,
    NegativeReducedCostInvariant,
    BrokenAugmentingPath,
}

impl fmt::Display for OptimizeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for OptimizeError {}

#[derive(Clone, Debug)]
struct Debtor {
    member_id: String,
    owed_minor: u64,
    liquidity_limit_minor: u64,
}

#[derive(Clone, Debug)]
struct Creditor {
    member_id: String,
    receivable_minor: u64,
}

#[derive(Clone, Debug)]
struct Prepared {
    context: ClearingContext,
    authority_evidence_ref: String,
    debtors: Vec<Debtor>,
    creditors: Vec<Creditor>,
    eligible_routes: Vec<SettlementRoute>,
    excluded_routes: Vec<ExcludedRoute>,
    deduplicated_route_observations: usize,
}

fn add_u128(left: u128, right: u128) -> Result<u128, OptimizeError> {
    left.checked_add(right)
        .ok_or(OptimizeError::ArithmeticOverflow)
}

fn add_u64(left: u64, right: u64) -> Result<u64, OptimizeError> {
    left.checked_add(right)
        .ok_or(OptimizeError::ArithmeticOverflow)
}

fn prepare(
    context: &ClearingContext,
    positions: &[ParticipantNet],
    routes: &[SettlementRoute],
) -> Result<Prepared, OptimizeError> {
    for (name, value) in [
        ("network_id", context.network_id.as_str()),
        ("asset_id", context.asset_id.as_str()),
        ("netting_set_id", context.netting_set_id.as_str()),
        ("policy_version", context.policy_version.as_str()),
    ] {
        if value.trim().is_empty() {
            return Err(OptimizeError::InvalidContext(name));
        }
    }

    let authority_evidence_ref = match &context.authority {
        AuthorityQualification::NotQualified => return Err(OptimizeError::AuthorityNotQualified),
        AuthorityQualification::ExternallyQualified { evidence_ref } => {
            if evidence_ref.trim().is_empty() {
                return Err(OptimizeError::InvalidContext(
                    "authority evidence reference",
                ));
            }
            evidence_ref.clone()
        }
    };

    let mut by_member = BTreeMap::<String, ParticipantNet>::new();
    let mut owed_total = 0_u128;
    let mut receivable_total = 0_u128;

    for participant in positions {
        if participant.member_id.trim().is_empty() {
            return Err(OptimizeError::InvalidParticipant("empty member_id".into()));
        }
        if by_member.contains_key(&participant.member_id) {
            return Err(OptimizeError::DuplicateParticipant(
                participant.member_id.clone(),
            ));
        }
        if participant.net_minor > 0 {
            owed_total = add_u128(owed_total, participant.net_minor as u128)?;
        } else if participant.net_minor < 0 {
            receivable_total = add_u128(
                receivable_total,
                participant.net_minor.unsigned_abs() as u128,
            )?;
        }
        by_member.insert(participant.member_id.clone(), participant.clone());
    }

    if owed_total != receivable_total {
        return Err(OptimizeError::UnbalancedNetPositions {
            owed_minor: owed_total,
            receivable_minor: receivable_total,
        });
    }

    let mut debtors = Vec::new();
    let mut creditors = Vec::new();
    for participant in by_member.values() {
        if participant.net_minor > 0 {
            let owed_minor = participant.net_minor as u64;
            debtors.push(Debtor {
                member_id: participant.member_id.clone(),
                owed_minor,
                liquidity_limit_minor: owed_minor.min(participant.available_liquidity_minor),
            });
        } else if participant.net_minor < 0 {
            creditors.push(Creditor {
                member_id: participant.member_id.clone(),
                receivable_minor: participant.net_minor.unsigned_abs(),
            });
        }
    }

    // Route IDs are canonical observation IDs. Identical repeated observations are
    // deduplicated; a reused ID with changed terms fails closed.
    let mut unique_by_id = BTreeMap::<String, SettlementRoute>::new();
    let mut deduplicated_route_observations = 0_usize;
    for route in routes {
        for (name, value) in [
            ("route_id", route.route_id.as_str()),
            ("from_member", route.from_member.as_str()),
            ("to_member", route.to_member.as_str()),
            ("network_id", route.network_id.as_str()),
            ("asset_id", route.asset_id.as_str()),
            ("netting_set_id", route.netting_set_id.as_str()),
            ("policy_version", route.policy_version.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(OptimizeError::InvalidRoute(format!("{} is empty", name)));
            }
        }

        if let Some(previous) = unique_by_id.get(&route.route_id) {
            if previous == route {
                deduplicated_route_observations = deduplicated_route_observations
                    .checked_add(1)
                    .ok_or(OptimizeError::ArithmeticOverflow)?;
                continue;
            }
            return Err(OptimizeError::ConflictingDuplicateRoute(
                route.route_id.clone(),
            ));
        }
        unique_by_id.insert(route.route_id.clone(), route.clone());
    }

    let mut eligible_routes = Vec::new();
    let mut excluded_routes = Vec::new();
    for route in unique_by_id.into_values() {
        if route.network_id != context.network_id
            || route.asset_id != context.asset_id
            || route.netting_set_id != context.netting_set_id
            || route.policy_version != context.policy_version
        {
            excluded_routes.push(ExcludedRoute {
                route_id: route.route_id,
                reason: RouteExclusionReason::ProfileMismatch,
            });
            continue;
        }
        if !route.enabled {
            excluded_routes.push(ExcludedRoute {
                route_id: route.route_id,
                reason: RouteExclusionReason::Disabled,
            });
            continue;
        }
        if route.capacity_minor == 0 {
            excluded_routes.push(ExcludedRoute {
                route_id: route.route_id,
                reason: RouteExclusionReason::ZeroCapacity,
            });
            continue;
        }

        let from = by_member
            .get(&route.from_member)
            .ok_or_else(|| OptimizeError::UnknownRouteParticipant(route.from_member.clone()))?;
        let to = by_member
            .get(&route.to_member)
            .ok_or_else(|| OptimizeError::UnknownRouteParticipant(route.to_member.clone()))?;

        if route.from_member == route.to_member || from.net_minor <= 0 || to.net_minor >= 0 {
            excluded_routes.push(ExcludedRoute {
                route_id: route.route_id,
                reason: RouteExclusionReason::WrongDirectionOrRoles,
            });
            continue;
        }
        eligible_routes.push(route);
    }

    // BTreeMap iteration makes participants and routes independent of input order.
    eligible_routes.sort_by(|a, b| a.route_id.cmp(&b.route_id));
    excluded_routes.sort_by(|a, b| a.route_id.cmp(&b.route_id));

    Ok(Prepared {
        context: context.clone(),
        authority_evidence_ref,
        debtors,
        creditors,
        eligible_routes,
        excluded_routes,
        deduplicated_route_observations,
    })
}

/// Allocate the maximum reachable integer flow, then minimize linear route cost exactly.
///
/// The min-cost max-flow result is optimal for this crate's stated model: one asset,
/// a single pre-qualified netting set, direct debtor-to-creditor arcs, integer capacities,
/// and non-negative linear unit costs. It does not make settlement final.
pub fn optimize(
    context: &ClearingContext,
    positions: &[ParticipantNet],
    routes: &[SettlementRoute],
) -> Result<AllocationPlan, OptimizeError> {
    let prepared = prepare(context, positions, routes)?;
    let debtor_start = 1_usize;
    let creditor_start = debtor_start + prepared.debtors.len();
    let sink = creditor_start + prepared.creditors.len();
    let node_count = sink + 1;
    let source = 0_usize;

    let debtor_nodes = prepared
        .debtors
        .iter()
        .enumerate()
        .map(|(i, debtor)| (debtor.member_id.clone(), debtor_start + i))
        .collect::<BTreeMap<_, _>>();
    let creditor_nodes = prepared
        .creditors
        .iter()
        .enumerate()
        .map(|(i, creditor)| (creditor.member_id.clone(), creditor_start + i))
        .collect::<BTreeMap<_, _>>();

    let mut graph = vec![Vec::<FlowEdge>::new(); node_count];
    for debtor in &prepared.debtors {
        add_edge(
            &mut graph,
            source,
            debtor_nodes[&debtor.member_id],
            debtor.liquidity_limit_minor,
            0,
        );
    }
    for creditor in &prepared.creditors {
        add_edge(
            &mut graph,
            creditor_nodes[&creditor.member_id],
            sink,
            creditor.receivable_minor,
            0,
        );
    }

    let mut route_edges = Vec::new();
    for route in &prepared.eligible_routes {
        let from_node = debtor_nodes[&route.from_member];
        let to_node = creditor_nodes[&route.to_member];
        let edge_index = add_edge(
            &mut graph,
            from_node,
            to_node,
            route.capacity_minor,
            route.cost_units_per_minor as i128,
        );
        route_edges.push((route.clone(), from_node, edge_index));
    }

    let _flow = min_cost_max_flow(&mut graph, source, sink)?;
    let mut legs = Vec::new();
    for (route, from_node, edge_index) in route_edges {
        let remaining_capacity = graph[from_node][edge_index].capacity;
        let amount_minor = route
            .capacity_minor
            .checked_sub(remaining_capacity)
            .ok_or(OptimizeError::ArithmeticOverflow)?;
        if amount_minor > 0 {
            legs.push(SettlementLeg {
                route_id: route.route_id,
                from_member: route.from_member,
                to_member: route.to_member,
                amount_minor,
                cost_units_per_minor: route.cost_units_per_minor,
                modeled_cost_units: (amount_minor as u128)
                    .checked_mul(route.cost_units_per_minor as u128)
                    .ok_or(OptimizeError::CostOverflow)?,
            });
        }
    }
    legs.sort_by(|a, b| a.route_id.cmp(&b.route_id));
    build_plan(prepared, legs)
}

/// A deliberately simple deterministic comparator: consume the cheapest available route
/// first. It is useful as a baseline, not claimed to maximize flow or minimize total cost.
pub fn greedy_match(
    context: &ClearingContext,
    positions: &[ParticipantNet],
    routes: &[SettlementRoute],
) -> Result<AllocationPlan, OptimizeError> {
    let prepared = prepare(context, positions, routes)?;
    let mut debtor_remaining = prepared
        .debtors
        .iter()
        .map(|d| (d.member_id.clone(), d.liquidity_limit_minor))
        .collect::<BTreeMap<_, _>>();
    let mut creditor_remaining = prepared
        .creditors
        .iter()
        .map(|c| (c.member_id.clone(), c.receivable_minor))
        .collect::<BTreeMap<_, _>>();
    let mut ordered_routes = prepared.eligible_routes.clone();
    ordered_routes.sort_by(|a, b| {
        a.cost_units_per_minor
            .cmp(&b.cost_units_per_minor)
            .then_with(|| a.route_id.cmp(&b.route_id))
    });

    let mut legs = Vec::new();
    for route in ordered_routes {
        let debtor_available = debtor_remaining
            .get(&route.from_member)
            .copied()
            .unwrap_or(0);
        let creditor_demand = creditor_remaining
            .get(&route.to_member)
            .copied()
            .unwrap_or(0);
        let amount_minor = route
            .capacity_minor
            .min(debtor_available)
            .min(creditor_demand);
        if amount_minor == 0 {
            continue;
        }
        debtor_remaining.insert(route.from_member.clone(), debtor_available - amount_minor);
        creditor_remaining.insert(route.to_member.clone(), creditor_demand - amount_minor);
        legs.push(SettlementLeg {
            route_id: route.route_id,
            from_member: route.from_member,
            to_member: route.to_member,
            amount_minor,
            cost_units_per_minor: route.cost_units_per_minor,
            modeled_cost_units: (amount_minor as u128)
                .checked_mul(route.cost_units_per_minor as u128)
                .ok_or(OptimizeError::CostOverflow)?,
        });
    }
    legs.sort_by(|a, b| a.route_id.cmp(&b.route_id));
    build_plan(prepared, legs)
}

fn build_plan(
    prepared: Prepared,
    mut legs: Vec<SettlementLeg>,
) -> Result<AllocationPlan, OptimizeError> {
    legs.sort_by(|a, b| a.route_id.cmp(&b.route_id));
    let mut sent = BTreeMap::<String, u64>::new();
    let mut received = BTreeMap::<String, u64>::new();
    let mut total_allocated_minor = 0_u128;
    let mut total_modeled_cost_units = 0_u128;

    for leg in &legs {
        let current_sent = sent.get(&leg.from_member).copied().unwrap_or(0);
        sent.insert(
            leg.from_member.clone(),
            add_u64(current_sent, leg.amount_minor)?,
        );
        let current_received = received.get(&leg.to_member).copied().unwrap_or(0);
        received.insert(
            leg.to_member.clone(),
            add_u64(current_received, leg.amount_minor)?,
        );
        total_allocated_minor = add_u128(total_allocated_minor, leg.amount_minor as u128)?;
        total_modeled_cost_units = total_modeled_cost_units
            .checked_add(leg.modeled_cost_units)
            .ok_or(OptimizeError::CostOverflow)?;
    }

    let mut residual_debtors = Vec::new();
    for debtor in &prepared.debtors {
        let paid = sent.get(&debtor.member_id).copied().unwrap_or(0);
        if paid > debtor.owed_minor {
            return Err(OptimizeError::ArithmeticOverflow);
        }
        let residual = debtor.owed_minor - paid;
        if residual > 0 {
            residual_debtors.push(Residual {
                member_id: debtor.member_id.clone(),
                amount_minor: residual,
            });
        }
    }

    let mut residual_creditors = Vec::new();
    for creditor in &prepared.creditors {
        let received_minor = received.get(&creditor.member_id).copied().unwrap_or(0);
        if received_minor > creditor.receivable_minor {
            return Err(OptimizeError::ArithmeticOverflow);
        }
        let residual = creditor.receivable_minor - received_minor;
        if residual > 0 {
            residual_creditors.push(Residual {
                member_id: creditor.member_id.clone(),
                amount_minor: residual,
            });
        }
    }

    let fully_allocatable = residual_debtors.is_empty() && residual_creditors.is_empty();
    Ok(AllocationPlan {
        network_id: prepared.context.network_id,
        asset_id: prepared.context.asset_id,
        netting_set_id: prepared.context.netting_set_id,
        policy_version: prepared.context.policy_version,
        authority_evidence_ref: prepared.authority_evidence_ref,
        legs,
        total_allocated_minor,
        total_modeled_cost_units,
        residual_debtors,
        residual_creditors,
        fully_allocatable,
        excluded_routes: prepared.excluded_routes,
        deduplicated_route_observations: prepared.deduplicated_route_observations,
    })
}

#[derive(Clone, Debug)]
struct FlowEdge {
    to: usize,
    reverse: usize,
    capacity: u64,
    cost: i128,
}

fn add_edge(
    graph: &mut [Vec<FlowEdge>],
    from: usize,
    to: usize,
    capacity: u64,
    cost: i128,
) -> usize {
    let forward_index = graph[from].len();
    let reverse_index = graph[to].len();
    graph[from].push(FlowEdge {
        to,
        reverse: reverse_index,
        capacity,
        cost,
    });
    graph[to].push(FlowEdge {
        to: from,
        reverse: forward_index,
        capacity: 0,
        cost: -cost,
    });
    forward_index
}

fn min_cost_max_flow(
    graph: &mut [Vec<FlowEdge>],
    source: usize,
    sink: usize,
) -> Result<u128, OptimizeError> {
    let n = graph.len();
    let infinity = i128::MAX / 4;
    let mut potential = vec![0_i128; n];
    let mut total_flow = 0_u128;

    loop {
        let mut distance = vec![infinity; n];
        let mut previous = vec![None::<(usize, usize)>; n];
        let mut heap = BinaryHeap::<Reverse<(i128, usize)>>::new();
        distance[source] = 0;
        heap.push(Reverse((0, source)));

        while let Some(Reverse((current_distance, from))) = heap.pop() {
            if current_distance != distance[from] {
                continue;
            }
            for (edge_index, edge) in graph[from].iter().enumerate() {
                if edge.capacity == 0 {
                    continue;
                }
                let reduced_cost = edge
                    .cost
                    .checked_add(potential[from])
                    .and_then(|v| v.checked_sub(potential[edge.to]))
                    .ok_or(OptimizeError::ArithmeticOverflow)?;
                if reduced_cost < 0 {
                    return Err(OptimizeError::NegativeReducedCostInvariant);
                }
                let next_distance = current_distance
                    .checked_add(reduced_cost)
                    .ok_or(OptimizeError::ArithmeticOverflow)?;
                if next_distance < distance[edge.to] {
                    distance[edge.to] = next_distance;
                    previous[edge.to] = Some((from, edge_index));
                    heap.push(Reverse((next_distance, edge.to)));
                }
            }
        }

        if previous[sink].is_none() {
            break;
        }

        for node in 0..n {
            if distance[node] != infinity {
                potential[node] = potential[node]
                    .checked_add(distance[node])
                    .ok_or(OptimizeError::ArithmeticOverflow)?;
            }
        }

        let mut amount = u64::MAX;
        let mut node = sink;
        while node != source {
            let (from, edge_index) = previous[node].ok_or(OptimizeError::BrokenAugmentingPath)?;
            amount = amount.min(graph[from][edge_index].capacity);
            node = from;
        }
        if amount == 0 {
            return Err(OptimizeError::BrokenAugmentingPath);
        }

        node = sink;
        while node != source {
            let (from, edge_index) = previous[node].ok_or(OptimizeError::BrokenAugmentingPath)?;
            let to = graph[from][edge_index].to;
            let reverse = graph[from][edge_index].reverse;
            graph[from][edge_index].capacity -= amount;
            graph[to][reverse].capacity = graph[to][reverse]
                .capacity
                .checked_add(amount)
                .ok_or(OptimizeError::ArithmeticOverflow)?;
            node = from;
        }
        total_flow = add_u128(total_flow, amount as u128)?;
    }

    Ok(total_flow)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> ClearingContext {
        ClearingContext {
            network_id: "network-test".into(),
            asset_id: "asset-usd-minor".into(),
            netting_set_id: "netting-set-alpha".into(),
            policy_version: "policy-v1".into(),
            authority: AuthorityQualification::ExternallyQualified {
                evidence_ref: "evidence://netting-authority/test-fixture".into(),
            },
        }
    }

    fn participant(id: &str, net_minor: i64, liquidity: u64) -> ParticipantNet {
        ParticipantNet {
            member_id: id.into(),
            net_minor,
            available_liquidity_minor: liquidity,
        }
    }

    fn route(id: &str, from: &str, to: &str, capacity: u64, cost: u64) -> SettlementRoute {
        SettlementRoute {
            route_id: id.into(),
            network_id: "network-test".into(),
            asset_id: "asset-usd-minor".into(),
            netting_set_id: "netting-set-alpha".into(),
            policy_version: "policy-v1".into(),
            from_member: from.into(),
            to_member: to.into(),
            enabled: true,
            capacity_minor: capacity,
            cost_units_per_minor: cost,
        }
    }

    #[test]
    fn globally_optimal_flow_beats_cheapest_edge_first_greedy() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 2, 2),
            participant("C", -2, 0),
            participant("D", -2, 0),
        ];
        let routes = vec![
            route("a-c-cheap", "A", "C", 2, 1),
            route("a-d", "A", "D", 2, 2),
            route("b-c", "B", "C", 2, 2),
            route("b-d-expensive", "B", "D", 2, 10),
        ];

        let exact = optimize(&context(), &positions, &routes).unwrap();
        let greedy = greedy_match(&context(), &positions, &routes).unwrap();

        assert_eq!(exact.total_allocated_minor, 4);
        assert_eq!(exact.total_modeled_cost_units, 8);
        assert_eq!(greedy.total_allocated_minor, 4);
        assert_eq!(greedy.total_modeled_cost_units, 22);
        assert!(exact.fully_allocatable);
    }

    #[test]
    fn liquidity_and_route_capacity_leave_explicit_residuals() {
        let positions = vec![participant("A", 5, 3), participant("B", -5, 0)];
        let routes = vec![route("a-b", "A", "B", 2, 4)];
        let plan = optimize(&context(), &positions, &routes).unwrap();

        assert_eq!(plan.total_allocated_minor, 2);
        assert!(!plan.fully_allocatable);
        assert_eq!(
            plan.residual_debtors,
            vec![Residual {
                member_id: "A".into(),
                amount_minor: 3
            }]
        );
        assert_eq!(
            plan.residual_creditors,
            vec![Residual {
                member_id: "B".into(),
                amount_minor: 3
            }]
        );
    }

    #[test]
    fn disabled_and_wrong_profile_routes_are_not_used() {
        let positions = vec![participant("A", 3, 3), participant("B", -3, 0)];
        let mut disabled = route("disabled", "A", "B", 3, 1);
        disabled.enabled = false;
        let mut wrong_profile = route("wrong-profile", "A", "B", 3, 0);
        wrong_profile.policy_version = "policy-v2".into();

        let plan = optimize(&context(), &positions, &[disabled, wrong_profile]).unwrap();
        assert_eq!(plan.total_allocated_minor, 0);
        assert_eq!(plan.excluded_routes.len(), 2);
        assert!(!plan.fully_allocatable);
    }

    #[test]
    fn absent_routes_do_not_become_available_by_default() {
        let positions = vec![participant("A", 3, 3), participant("B", -3, 0)];
        let plan = optimize(&context(), &positions, &[]).unwrap();
        assert_eq!(plan.legs.len(), 0);
        assert_eq!(plan.residual_debtors[0].amount_minor, 3);
        assert_eq!(plan.residual_creditors[0].amount_minor, 3);
    }

    #[test]
    fn unbalanced_net_positions_fail_closed() {
        let positions = vec![participant("A", 3, 3), participant("B", -2, 0)];
        assert!(matches!(
            optimize(&context(), &positions, &[]),
            Err(OptimizeError::UnbalancedNetPositions { .. })
        ));
    }

    #[test]
    fn unqualified_netting_authority_fails_closed() {
        let mut unqualified = context();
        unqualified.authority = AuthorityQualification::NotQualified;
        let positions = vec![participant("A", 1, 1), participant("B", -1, 0)];
        assert_eq!(
            optimize(&unqualified, &positions, &[route("a-b", "A", "B", 1, 1)]),
            Err(OptimizeError::AuthorityNotQualified)
        );
    }

    #[test]
    fn identical_route_observations_are_deduplicated() {
        let positions = vec![participant("A", 4, 4), participant("B", -4, 0)];
        let single = route("route-1", "A", "B", 4, 3);
        let plan = optimize(&context(), &positions, &[single.clone(), single]).unwrap();

        assert_eq!(plan.deduplicated_route_observations, 1);
        assert_eq!(plan.total_allocated_minor, 4);
    }

    #[test]
    fn conflicting_reuse_of_a_route_id_fails_closed() {
        let positions = vec![participant("A", 4, 4), participant("B", -4, 0)];
        let first = route("route-1", "A", "B", 4, 3);
        let second = route("route-1", "A", "B", 4, 5);
        assert!(matches!(
            optimize(&context(), &positions, &[first, second]),
            Err(OptimizeError::ConflictingDuplicateRoute(_))
        ));
    }

    #[test]
    fn output_is_input_order_invariant() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 2, 2),
            participant("C", -2, 0),
            participant("D", -2, 0),
        ];
        let routes = vec![
            route("a-c", "A", "C", 2, 3),
            route("a-d", "A", "D", 2, 2),
            route("b-c", "B", "C", 2, 2),
            route("b-d", "B", "D", 2, 3),
        ];
        let first = optimize(&context(), &positions, &routes).unwrap();
        let mut reversed_positions = positions.clone();
        reversed_positions.reverse();
        let mut reversed_routes = routes.clone();
        reversed_routes.reverse();
        let second = optimize(&context(), &reversed_positions, &reversed_routes).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn linear_cost_overflow_is_reported() {
        let m = i64::MAX;
        let positions = vec![
            participant("A", m, u64::MAX),
            participant("B", m, u64::MAX),
            participant("C", m, u64::MAX),
            participant("X", -m, 0),
            participant("Y", -m, 0),
            participant("Z", -m, 0),
        ];
        let routes = vec![
            route("a-x", "A", "X", u64::MAX, u64::MAX),
            route("b-y", "B", "Y", u64::MAX, u64::MAX),
            route("c-z", "C", "Z", u64::MAX, u64::MAX),
        ];
        assert_eq!(
            optimize(&context(), &positions, &routes),
            Err(OptimizeError::CostOverflow)
        );
    }

    fn brute_force_best(routes: &[SettlementRoute], positions: &[ParticipantNet]) -> (u128, u128) {
        fn visit(
            index: usize,
            routes: &[SettlementRoute],
            positions: &[ParticipantNet],
            allocations: &mut [u64],
            best: &mut (u128, u128),
        ) {
            if index == routes.len() {
                let mut sent = BTreeMap::<String, u128>::new();
                let mut received = BTreeMap::<String, u128>::new();
                let mut flow = 0_u128;
                let mut cost = 0_u128;
                for (route, amount) in routes.iter().zip(allocations.iter().copied()) {
                    *sent.entry(route.from_member.clone()).or_default() += amount as u128;
                    *received.entry(route.to_member.clone()).or_default() += amount as u128;
                    flow += amount as u128;
                    cost += amount as u128 * route.cost_units_per_minor as u128;
                }
                for p in positions {
                    if p.net_minor > 0 {
                        let sent_amount = sent.get(&p.member_id).copied().unwrap_or(0);
                        let owed = p.net_minor as u64;
                        let liquidity_limit = owed.min(p.available_liquidity_minor);
                        if sent_amount > liquidity_limit as u128 {
                            return;
                        }
                    } else if p.net_minor < 0 {
                        let received_amount = received.get(&p.member_id).copied().unwrap_or(0);
                        if received_amount > p.net_minor.unsigned_abs() as u128 {
                            return;
                        }
                    }
                }
                if flow > best.0 || (flow == best.0 && cost < best.1) {
                    *best = (flow, cost);
                }
                return;
            }

            let route = &routes[index];
            let debtor = positions
                .iter()
                .find(|p| p.member_id == route.from_member)
                .unwrap();
            let creditor = positions
                .iter()
                .find(|p| p.member_id == route.to_member)
                .unwrap();
            let debtor_limit = (debtor.net_minor as u64).min(debtor.available_liquidity_minor);
            let creditor_limit = creditor.net_minor.unsigned_abs();
            let max = route.capacity_minor.min(debtor_limit).min(creditor_limit);
            for amount in 0..=max {
                allocations[index] = amount;
                visit(index + 1, routes, positions, allocations, best);
            }
        }

        let mut allocations = vec![0; routes.len()];
        let mut best = (0, u128::MAX);
        visit(0, routes, positions, &mut allocations, &mut best);
        best
    }

    #[test]
    fn min_cost_flow_matches_exhaustive_integer_oracle_on_small_graphs() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 2, 2),
            participant("C", -2, 0),
            participant("D", -2, 0),
        ];

        // Explore many tiny integer-capacity graphs so the flow solver is checked
        // against exhaustive enumeration, including routes with zero capacity.
        for seed in 0..64_usize {
            let routes = vec![
                route(
                    "a-c",
                    "A",
                    "C",
                    ((seed * 3 + 1) % 3) as u64,
                    ((seed * 7 + 2) % 11 + 1) as u64,
                ),
                route(
                    "a-d",
                    "A",
                    "D",
                    ((seed * 5 + 2) % 3) as u64,
                    ((seed * 11 + 3) % 13 + 1) as u64,
                ),
                route(
                    "b-c",
                    "B",
                    "C",
                    ((seed * 7 + 1) % 3) as u64,
                    ((seed * 13 + 4) % 17 + 1) as u64,
                ),
                route(
                    "b-d",
                    "B",
                    "D",
                    ((seed * 11 + 2) % 3) as u64,
                    ((seed * 17 + 5) % 19 + 1) as u64,
                ),
            ];
            let oracle = brute_force_best(&routes, &positions);
            let result = optimize(&context(), &positions, &routes).unwrap();
            assert_eq!(
                result.total_allocated_minor, oracle.0,
                "flow mismatch, seed={seed}"
            );
            assert_eq!(
                result.total_modeled_cost_units, oracle.1,
                "cost mismatch, seed={seed}"
            );
        }
    }
}
