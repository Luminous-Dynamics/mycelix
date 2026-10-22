use finance_route_clearing_optimizer::{
    optimize, AuthorityQualification, ClearingContext, ParticipantNet, SettlementRoute,
};
use std::collections::BTreeMap;

fn context() -> ClearingContext {
    ClearingContext {
        network_id: "oracle-network".into(),
        asset_id: "oracle-asset-minor".into(),
        netting_set_id: "oracle-netting-set".into(),
        policy_version: "route-oracle-v1".into(),
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "fixture-only://not-legal-authority".into(),
        },
    }
}

fn route(id: &str, from: &str, to: &str, capacity_minor: u64, cost_units_per_minor: u64) -> SettlementRoute {
    SettlementRoute {
        route_id: id.into(),
        network_id: "oracle-network".into(),
        asset_id: "oracle-asset-minor".into(),
        netting_set_id: "oracle-netting-set".into(),
        policy_version: "route-oracle-v1".into(),
        from_member: from.into(),
        to_member: to.into(),
        enabled: true,
        capacity_minor,
        cost_units_per_minor,
    }
}

/// Independent exhaustive oracle for a bounded 2-debtor/2-creditor network.
///
/// It enumerates every integer amount on each of the four direct arcs and scores
/// each complete allocation by the optimizer's declared lexicographic objective:
/// maximize allocated minor units, then minimize linear cost. It does not call
/// production routing, shortest-path, or plan-building helpers.
fn exhaustive_oracle(
    positions: &[ParticipantNet; 4],
    routes: &[SettlementRoute; 4],
) -> (u128, u128) {
    fn visit(
        index: usize,
        positions: &[ParticipantNet; 4],
        routes: &[SettlementRoute; 4],
        sent: &mut [u64; 2],
        received: &mut [u64; 2],
        flow_minor: u128,
        cost_units: u128,
        best: &mut Option<(u128, u128)>,
    ) {
        if index == routes.len() {
            let candidate = (flow_minor, cost_units);
            let improves = match *best {
                None => true,
                Some((best_flow, best_cost)) => {
                    candidate.0 > best_flow
                        || (candidate.0 == best_flow && candidate.1 < best_cost)
                }
            };
            if improves {
                *best = Some(candidate);
            }
            return;
        }

        let route = &routes[index];
        let debtor_index = if route.from_member == "A" { 0 } else { 1 };
        let creditor_index = if route.to_member == "C" { 0 } else { 1 };
        let debtor = &positions[debtor_index];
        let creditor = &positions[2 + creditor_index];
        let debtor_limit = (debtor.net_minor as u64).min(debtor.available_liquidity_minor);
        let creditor_limit = creditor.net_minor.unsigned_abs();
        let remaining_debtor = debtor_limit - sent[debtor_index];
        let remaining_creditor = creditor_limit - received[creditor_index];
        let upper = route
            .capacity_minor
            .min(remaining_debtor)
            .min(remaining_creditor);

        for amount in 0..=upper {
            sent[debtor_index] += amount;
            received[creditor_index] += amount;
            visit(
                index + 1,
                positions,
                routes,
                sent,
                received,
                flow_minor + u128::from(amount),
                cost_units
                    + u128::from(amount) * u128::from(route.cost_units_per_minor),
                best,
            );
            sent[debtor_index] -= amount;
            received[creditor_index] -= amount;
        }
    }

    let mut best = None;
    visit(
        0,
        positions,
        routes,
        &mut [0; 2],
        &mut [0; 2],
        0,
        0,
        &mut best,
    );
    best.expect("the all-zero allocation is always feasible")
}

fn next_bounded(state: &mut u64, upper_exclusive: u64) -> u64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    (*state >> 32) % upper_exclusive
}

#[test]
fn min_cost_max_flow_matches_exhaustive_oracle_on_256_deterministic_networks() {
    // Four arcs with capacities 0..=2 yield at most 3^4 = 81 candidate
    // allocations per case. 256 frozen-seed cases cover varied costs, route
    // bottlenecks, debtor liquidity ceilings and disconnected/sparse networks.
    for case_id in 0_u64..256 {
        let mut seed = case_id ^ 0x9e37_79b9_7f4a_7c15;
        let positions = [
            ParticipantNet {
                member_id: "A".into(),
                net_minor: 2,
                available_liquidity_minor: next_bounded(&mut seed, 3),
            },
            ParticipantNet {
                member_id: "B".into(),
                net_minor: 2,
                available_liquidity_minor: next_bounded(&mut seed, 3),
            },
            ParticipantNet {
                member_id: "C".into(),
                net_minor: -2,
                available_liquidity_minor: 0,
            },
            ParticipantNet {
                member_id: "D".into(),
                net_minor: -2,
                available_liquidity_minor: 0,
            },
        ];
        let routes = [
            route("a-c", "A", "C", next_bounded(&mut seed, 3), next_bounded(&mut seed, 10)),
            route("a-d", "A", "D", next_bounded(&mut seed, 3), next_bounded(&mut seed, 10)),
            route("b-c", "B", "C", next_bounded(&mut seed, 3), next_bounded(&mut seed, 10)),
            route("b-d", "B", "D", next_bounded(&mut seed, 3), next_bounded(&mut seed, 10)),
        ];

        let expected = exhaustive_oracle(&positions, &routes);
        let actual = optimize(&context(), &positions, &routes)
            .unwrap_or_else(|error| panic!("case={case_id}: optimizer rejected valid graph: {error}"));

        assert_eq!(
            (actual.total_allocated_minor, actual.total_modeled_cost_units),
            expected,
            "primary objective diverged from independent oracle for case={case_id}"
        );
        assert_eq!(
            actual.fully_allocatable,
            expected.0 == 4,
            "fully_allocatable disagrees with residual coverage for case={case_id}"
        );

        let route_by_id = routes
            .iter()
            .map(|route| (route.route_id.as_str(), route))
            .collect::<BTreeMap<_, _>>();
        let mut sent = BTreeMap::<String, u64>::new();
        let mut received = BTreeMap::<String, u64>::new();
        let mut route_used = BTreeMap::<String, u64>::new();
        for leg in &actual.legs {
            let source = sent.entry(leg.from_member.clone()).or_default();
            *source += leg.amount_minor;
            let target = received.entry(leg.to_member.clone()).or_default();
            *target += leg.amount_minor;
            *route_used.entry(leg.route_id.clone()).or_default() += leg.amount_minor;
            let source_route = route_by_id[leg.route_id.as_str()];
            assert_eq!(leg.cost_units_per_minor, source_route.cost_units_per_minor);
            assert_eq!(
                leg.modeled_cost_units,
                u128::from(leg.amount_minor) * u128::from(source_route.cost_units_per_minor)
            );
        }
        for (index, member) in ["A", "B"].iter().enumerate() {
            assert!(
                sent.get(*member).copied().unwrap_or(0)
                    <= positions[index].available_liquidity_minor.min(2),
                "debtor liquidity exceeded for case={case_id}, member={member}"
            );
        }
        for (index, member) in ["C", "D"].iter().enumerate() {
            assert!(
                received.get(*member).copied().unwrap_or(0) <= 2,
                "creditor receivable exceeded for case={case_id}, member={member}"
            );
            assert_eq!(
                received.get(*member).copied().unwrap_or(0) as u128
                    + u128::from(
                        actual
                            .residual_creditors
                            .iter()
                            .find(|residual| residual.member_id == *member)
                            .map(|residual| residual.amount_minor)
                            .unwrap_or(0)
                    ),
                2,
                "creditor conservation mismatch for case={case_id}, member={member}"
            );
            let _ = index;
        }
        for (route_id, used) in route_used {
            assert!(
                used <= route_by_id[route_id.as_str()].capacity_minor,
                "route capacity exceeded for case={case_id}, route={route_id}"
            );
        }

        // Canonicalization must make the complete plan independent of caller order.
        let mut reversed_positions = positions.clone();
        reversed_positions.reverse();
        let mut reversed_routes = routes.clone();
        reversed_routes.reverse();
        let reordered = optimize(&context(), &reversed_positions, &reversed_routes)
            .unwrap_or_else(|error| panic!("reordered case={case_id}: {error}"));
        assert_eq!(actual, reordered, "input-order dependence for case={case_id}");
    }
}
