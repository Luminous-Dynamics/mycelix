#![forbid(unsafe_code)]

// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Exact small-instance optimizer for routes with a fixed activation cost.
//!
//! Fixed-charge network flow is not the same optimization problem as linear min-cost
//! flow: opening a route incurs a cost regardless of the amount carried. This module
//! makes that cost explicit and uses bounded route-subset enumeration, with the existing
//! min-cost-flow solver as its subproblem. The exponential bound is intentional and
//! fail-closed; this is an exact oracle for small graphs, not a large-network heuristic.

use super::{optimize, AllocationPlan, ClearingContext, OptimizeError, ParticipantNet, SettlementRoute};
use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

/// Maximum number of distinct candidate route IDs for exhaustive fixed-charge search.
/// At this bound the search evaluates at most 4,096 route subsets.
pub const MAX_EXACT_FIXED_CHARGE_ROUTES: usize = 12;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FixedChargeRoute {
    pub route: SettlementRoute,
    /// Charged once if and only if this route carries a positive allocation.
    /// This is a modeled cost weight, not a provider quote.
    pub activation_cost_units: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FixedChargePlan {
    pub allocation: AllocationPlan,
    /// Sorted canonical IDs for routes carrying a positive amount.
    pub activated_route_ids: Vec<String>,
    pub fixed_activation_cost_units: u128,
    /// Linear route costs plus fixed activation charges.
    pub total_cost_units: u128,
    /// Number of candidate route subsets evaluated (including the empty set).
    pub evaluated_subsets: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FixedChargeError {
    TooManyCandidateRoutes {
        observed: usize,
        maximum: usize,
    },
    ConflictingDuplicateRoute(String),
    CostOverflow,
    Optimize(OptimizeError),
}

impl fmt::Display for FixedChargeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for FixedChargeError {}

impl From<OptimizeError> for FixedChargeError {
    fn from(value: OptimizeError) -> Self {
        Self::Optimize(value)
    }
}

/// Find the maximum-flow allocation, then minimize variable plus route-activation costs.
///
/// For each subset of candidate routes, the regular min-cost-flow solver finds the
/// minimum linear-cost allocation at maximum flow over that subset. The outer search
/// evaluates fixed activation charges only for routes actually used. Because activation
/// costs are non-negative, this exhaustive subset approach is exact for the declared
/// direct-route model. It deliberately rejects instances with more than 12 distinct
/// route IDs rather than silently switching to an unqualified heuristic.
///
/// Authority and profile checks performed by the linear optimizer remain in force. The
/// return value is a proposal, not a legally enforceable netting instruction or settlement
/// receipt.
pub fn optimize_fixed_charge(
    context: &ClearingContext,
    positions: &[ParticipantNet],
    route_observations: &[FixedChargeRoute],
) -> Result<FixedChargePlan, FixedChargeError> {
    let mut unique = BTreeMap::<String, FixedChargeRoute>::new();
    let mut duplicate_observations = 0_usize;

    for observed in route_observations {
        let route_id = observed.route.route_id.clone();
        if let Some(previous) = unique.get(&route_id) {
            if previous == observed {
                duplicate_observations = duplicate_observations
                    .checked_add(1)
                    .ok_or(FixedChargeError::CostOverflow)?;
                continue;
            }
            return Err(FixedChargeError::ConflictingDuplicateRoute(route_id));
        }
        unique.insert(route_id, observed.clone());
    }

    let routes = unique.into_values().collect::<Vec<_>>();
    if routes.len() > MAX_EXACT_FIXED_CHARGE_ROUTES {
        return Err(FixedChargeError::TooManyCandidateRoutes {
            observed: routes.len(),
            maximum: MAX_EXACT_FIXED_CHARGE_ROUTES,
        });
    }

    let subset_count = 1_u64
        .checked_shl(routes.len() as u32)
        .ok_or(FixedChargeError::CostOverflow)?;
    let mut best: Option<FixedChargePlan> = None;

    for mask in 0..subset_count {
        let selected = routes
            .iter()
            .enumerate()
            .filter_map(|(index, candidate)| {
                ((mask & (1_u64 << index)) != 0).then(|| candidate.route.clone())
            })
            .collect::<Vec<_>>();

        let mut allocation = optimize(context, positions, &selected)?;
        let mut activated_route_ids = allocation
            .legs
            .iter()
            .filter(|leg| leg.amount_minor > 0)
            .map(|leg| leg.route_id.clone())
            .collect::<Vec<_>>();
        activated_route_ids.sort();
        activated_route_ids.dedup();

        let by_id = routes
            .iter()
            .map(|candidate| {
                (
                    candidate.route.route_id.as_str(),
                    candidate.activation_cost_units,
                )
            })
            .collect::<BTreeMap<_, _>>();

        let mut fixed_activation_cost_units = 0_u128;
        for route_id in &activated_route_ids {
            let activation_cost = by_id
                .get(route_id.as_str())
                .ok_or_else(|| FixedChargeError::ConflictingDuplicateRoute(route_id.clone()))?;
            fixed_activation_cost_units = fixed_activation_cost_units
                .checked_add(*activation_cost as u128)
                .ok_or(FixedChargeError::CostOverflow)?;
        }

        let total_cost_units = allocation
            .total_modeled_cost_units
            .checked_add(fixed_activation_cost_units)
            .ok_or(FixedChargeError::CostOverflow)?;

        allocation.deduplicated_route_observations = allocation
            .deduplicated_route_observations
            .checked_add(duplicate_observations)
            .ok_or(FixedChargeError::CostOverflow)?;

        let candidate = FixedChargePlan {
            allocation,
            activated_route_ids,
            fixed_activation_cost_units,
            total_cost_units,
            evaluated_subsets: subset_count,
        };

        if best
            .as_ref()
            .is_none_or(|current| is_better(&candidate, current))
        {
            best = Some(candidate);
        }
    }

    let mut best = best.ok_or(FixedChargeError::CostOverflow)?;

    // Preserve exclusions from the full candidate input, not just the subset used by
    // the chosen plan. This keeps disabled and cross-profile route observations visible.
    let all_routes = routes
        .iter()
        .map(|candidate| candidate.route.clone())
        .collect::<Vec<_>>();
    let eligibility_report = optimize(context, positions, &all_routes)?;
    best.allocation.excluded_routes = eligibility_report.excluded_routes;
    best.allocation.deduplicated_route_observations = eligibility_report
        .deduplicated_route_observations
        .checked_add(duplicate_observations)
        .ok_or(FixedChargeError::CostOverflow)?;

    Ok(best)
}

fn is_better(candidate: &FixedChargePlan, current: &FixedChargePlan) -> bool {
    let candidate_flow = candidate.allocation.total_allocated_minor;
    let current_flow = current.allocation.total_allocated_minor;

    if candidate_flow != current_flow {
        return candidate_flow > current_flow;
    }
    if candidate.total_cost_units != current.total_cost_units {
        return candidate.total_cost_units < current.total_cost_units;
    }
    if candidate.activated_route_ids != current.activated_route_ids {
        return candidate.activated_route_ids < current.activated_route_ids;
    }

    let candidate_legs = candidate
        .allocation
        .legs
        .iter()
        .map(|leg| (leg.route_id.as_str(), leg.amount_minor))
        .collect::<Vec<_>>();
    let current_legs = current
        .allocation
        .legs
        .iter()
        .map(|leg| (leg.route_id.as_str(), leg.amount_minor))
        .collect::<Vec<_>>();
    candidate_legs < current_legs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AuthorityQualification;

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

    fn route(
        id: &str,
        from: &str,
        to: &str,
        capacity: u64,
        cost: u64,
        fixed: u64,
    ) -> FixedChargeRoute {
        FixedChargeRoute {
            route: SettlementRoute {
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
            },
            activation_cost_units: fixed,
        }
    }

    #[test]
    fn fixed_charge_changes_route_choice_vs_linear_min_cost() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 2, 2),
            participant("C", -2, 0),
            participant("D", -2, 0),
        ];
        let candidates = vec![
            route("a-c", "A", "C", 2, 1, 100),
            route("a-d", "A", "D", 2, 10, 0),
            route("b-c", "B", "C", 2, 10, 0),
            route("b-d", "B", "D", 2, 1, 100),
        ];
        let linear_routes = candidates
            .iter()
            .map(|candidate| candidate.route.clone())
            .collect::<Vec<_>>();

        let linear = optimize(&context(), &positions, &linear_routes).unwrap();
        let fixed = optimize_fixed_charge(&context(), &positions, &candidates).unwrap();

        assert_eq!(linear.total_allocated_minor, 4);
        assert_eq!(linear.total_modeled_cost_units, 4);
        assert_eq!(fixed.allocation.total_allocated_minor, 4);
        assert_eq!(fixed.allocation.total_modeled_cost_units, 40);
        assert_eq!(fixed.fixed_activation_cost_units, 0);
        assert_eq!(fixed.total_cost_units, 40);
        assert_eq!(fixed.activated_route_ids, vec!["a-d", "b-c"]);
        assert_eq!(fixed.evaluated_subsets, 16);
    }

    #[test]
    fn maximum_allocated_amount_precedes_lower_cost_partial_flow() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 1, 1),
            participant("C", -2, 0),
            participant("D", -1, 0),
        ];
        let candidates = vec![
            route("a-c-cheap", "A", "C", 2, 0, 0),
            route("a-d-expensive", "A", "D", 1, 100, 0),
            route("b-c-expensive", "B", "C", 1, 100, 0),
        ];

        let result = optimize_fixed_charge(&context(), &positions, &candidates).unwrap();

        assert_eq!(result.allocation.total_allocated_minor, 3);
        assert!(result.allocation.fully_allocatable);
        assert_eq!(result.total_cost_units, 200);
    }

    #[test]
    fn activation_charge_is_paid_once_per_used_route() {
        let positions = vec![participant("A", 5, 5), participant("B", -5, 0)];
        let result = optimize_fixed_charge(
            &context(),
            &positions,
            &[route("a-b", "A", "B", 5, 2, 7)],
        )
        .unwrap();

        assert_eq!(result.allocation.total_allocated_minor, 5);
        assert_eq!(result.allocation.total_modeled_cost_units, 10);
        assert_eq!(result.fixed_activation_cost_units, 7);
        assert_eq!(result.total_cost_units, 17);
    }

    #[test]
    fn identical_route_observations_dedupe_but_changed_terms_fail_closed() {
        let positions = vec![participant("A", 2, 2), participant("B", -2, 0)];
        let observation = route("a-b", "A", "B", 2, 3, 11);
        let duplicate = optimize_fixed_charge(
            &context(),
            &positions,
            &[observation.clone(), observation.clone()],
        )
        .unwrap();
        assert_eq!(duplicate.allocation.deduplicated_route_observations, 1);
        assert_eq!(duplicate.fixed_activation_cost_units, 11);
        assert_eq!(duplicate.total_cost_units, 17);

        let conflicting = route("a-b", "A", "B", 2, 3, 12);
        assert_eq!(
            optimize_fixed_charge(&context(), &positions, &[observation, conflicting]),
            Err(FixedChargeError::ConflictingDuplicateRoute("a-b".into()))
        );
    }

    #[test]
    fn fixed_charge_search_is_input_order_invariant() {
        let positions = vec![
            participant("A", 2, 2),
            participant("B", 2, 2),
            participant("C", -2, 0),
            participant("D", -2, 0),
        ];
        let candidates = vec![
            route("a-c", "A", "C", 2, 1, 7),
            route("a-d", "A", "D", 2, 2, 4),
            route("b-c", "B", "C", 2, 2, 4),
            route("b-d", "B", "D", 2, 1, 7),
        ];
        let first = optimize_fixed_charge(&context(), &positions, &candidates).unwrap();
        let mut reversed = candidates.clone();
        reversed.reverse();
        let second = optimize_fixed_charge(&context(), &positions, &reversed).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn exact_fixed_charge_search_fails_closed_above_route_limit() {
        let positions = vec![participant("A", 1, 1), participant("B", -1, 0)];
        let candidates = (0..=MAX_EXACT_FIXED_CHARGE_ROUTES)
            .map(|i| route(&format!("route-{i:02}"), "A", "B", 1, 1, 1))
            .collect::<Vec<_>>();
        assert_eq!(
            optimize_fixed_charge(&context(), &positions, &candidates),
            Err(FixedChargeError::TooManyCandidateRoutes {
                observed: MAX_EXACT_FIXED_CHARGE_ROUTES + 1,
                maximum: MAX_EXACT_FIXED_CHARGE_ROUTES,
            })
        );
    }

    #[test]
    fn excluded_route_observations_survive_route_subset_search() {
        let positions = vec![participant("A", 2, 2), participant("B", -2, 0)];
        let active = route("active", "A", "B", 2, 1, 0);
        let mut disabled = route("disabled", "unused-from", "unused-to", 2, 1, 0);
        disabled.route.enabled = false;

        let result =
            optimize_fixed_charge(&context(), &positions, &[active, disabled]).unwrap();

        assert_eq!(result.allocation.total_allocated_minor, 2);
        assert_eq!(
            result.allocation.excluded_routes,
            vec![super::super::ExcludedRoute {
                route_id: "disabled".into(),
                reason: super::super::RouteExclusionReason::Disabled,
            }]
        );
    }

    #[test]
    fn invalid_netting_authority_stays_fail_closed() {
        let mut invalid = context();
        invalid.authority = AuthorityQualification::NotQualified;
        let positions = vec![participant("A", 1, 1), participant("B", -1, 0)];
        assert_eq!(
            optimize_fixed_charge(
                &invalid,
                &positions,
                &[route("a-b", "A", "B", 1, 1, 0)],
            ),
            Err(FixedChargeError::Optimize(OptimizeError::AuthorityNotQualified))
        );
    }
}
