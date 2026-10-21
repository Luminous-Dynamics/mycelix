use finance_route_clearing_optimizer::fixed_charge::{
    optimize_fixed_charge, FixedChargeRoute,
};
use finance_route_clearing_optimizer::{
    AuthorityQualification, ClearingContext, ParticipantNet, SettlementRoute,
};
use std::collections::BTreeMap;

fn context() -> ClearingContext {
    ClearingContext {
        network_id: "oracle-network".into(),
        asset_id: "oracle-asset".into(),
        netting_set_id: "oracle-netting-set".into(),
        policy_version: "oracle-policy-v1".into(),
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "fixture-only://not-authority-evidence".into(),
        },
    }
}

fn fixture_route(
    id: &str,
    from: &str,
    to: &str,
    capacity: u64,
    variable_cost: u64,
    activation_cost: u64,
) -> FixedChargeRoute {
    FixedChargeRoute {
        route: SettlementRoute {
            route_id: id.into(),
            network_id: "oracle-network".into(),
            asset_id: "oracle-asset".into(),
            netting_set_id: "oracle-netting-set".into(),
            policy_version: "oracle-policy-v1".into(),
            from_member: from.into(),
            to_member: to.into(),
            enabled: true,
            capacity_minor: capacity,
            cost_units_per_minor: variable_cost,
        },
        activation_cost_units: activation_cost,
    }
}

fn positions() -> Vec<ParticipantNet> {
    vec![
        ParticipantNet {
            member_id: "A".into(),
            net_minor: 2,
            available_liquidity_minor: 2,
        },
        ParticipantNet {
            member_id: "B".into(),
            net_minor: 2,
            available_liquidity_minor: 2,
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
    ]
}

/// Independent exhaustive allocation oracle for the tiny 2-debtor/2-creditor corpus.
/// It enumerates every integer flow on every route, checks node/route capacities, and
/// compares lexicographically: maximum total flow, then minimum variable + activation
/// cost. This deliberately does not call the production optimizer.
fn exhaustive_oracle(
    routes: &[FixedChargeRoute],
    positions: &[ParticipantNet],
) -> (u128, u128) {
    fn visit(
        index: usize,
        routes: &[FixedChargeRoute],
        positions: &[ParticipantNet],
        flow_by_route: &mut [u64],
        best: &mut (u128, u128),
    ) {
        if index < routes.len() {
            for amount in 0..=routes[index].route.capacity_minor.min(2) {
                flow_by_route[index] = amount;
                visit(index + 1, routes, positions, flow_by_route, best);
            }
            return;
        }

        let mut sent = BTreeMap::<&str, u128>::new();
        let mut received = BTreeMap::<&str, u128>::new();
        let mut total_flow = 0_u128;
        let mut total_cost = 0_u128;

        for (candidate, amount) in routes.iter().zip(flow_by_route.iter().copied()) {
            let amount = amount as u128;
            *sent.entry(candidate.route.from_member.as_str()).or_default() += amount;
            *received.entry(candidate.route.to_member.as_str()).or_default() += amount;
            total_flow += amount;
            total_cost += amount * candidate.route.cost_units_per_minor as u128;
            if amount > 0 {
                total_cost += candidate.activation_cost_units as u128;
            }
        }

        for position in positions {
            if position.net_minor > 0 {
                let limit = (position.net_minor as u64)
                    .min(position.available_liquidity_minor) as u128;
                if sent.get(position.member_id.as_str()).copied().unwrap_or(0) > limit {
                    return;
                }
            } else if position.net_minor < 0 {
                let limit = position.net_minor.unsigned_abs() as u128;
                if received
                    .get(position.member_id.as_str())
                    .copied()
                    .unwrap_or(0)
                    > limit
                {
                    return;
                }
            }
        }

        if total_flow > best.0 || (total_flow == best.0 && total_cost < best.1) {
            *best = (total_flow, total_cost);
        }
    }

    let mut flow_by_route = vec![0_u64; routes.len()];
    let mut best = (0_u128, u128::MAX);
    visit(0, routes, positions, &mut flow_by_route, &mut best);
    best
}

#[test]
fn fixed_charge_optimizer_matches_exhaustive_oracle_on_deterministic_small_graphs() {
    let context = context();
    let positions = positions();

    // Frozen deterministic corpus: 128 distinct combinations of route capacity,
    // per-unit cost (including zero), and one-time activation charges. Each graph has
    // only four route arcs, so the oracle enumerates at most 3^4 = 81 allocations.
    for seed in 0..128_usize {
        let routes = vec![
            fixture_route(
                "a-c",
                "A",
                "C",
                ((seed * 3 + 1) % 3) as u64,
                ((seed * 7 + 2) % 9) as u64,
                ((seed * 11 + 3) % 13) as u64,
            ),
            fixture_route(
                "a-d",
                "A",
                "D",
                ((seed * 5 + 2) % 3) as u64,
                ((seed * 11 + 4) % 9) as u64,
                ((seed * 13 + 5) % 13) as u64,
            ),
            fixture_route(
                "b-c",
                "B",
                "C",
                ((seed * 7 + 1) % 3) as u64,
                ((seed * 13 + 6) % 9) as u64,
                ((seed * 17 + 7) % 13) as u64,
            ),
            fixture_route(
                "b-d",
                "B",
                "D",
                ((seed * 11 + 2) % 3) as u64,
                ((seed * 17 + 8) % 9) as u64,
                ((seed * 19 + 9) % 13) as u64,
            ),
        ];

        let expected = exhaustive_oracle(&routes, &positions);
        let actual = optimize_fixed_charge(&context, &positions, &routes)
            .unwrap_or_else(|error| panic!("seed={seed}: {error}"));

        assert_eq!(
            actual.allocation.total_allocated_minor, expected.0,
            "maximum-flow mismatch for fixed-charge graph seed={seed}"
        );
        assert_eq!(
            actual.total_cost_units, expected.1,
            "minimum total cost mismatch for fixed-charge graph seed={seed}"
        );

        let mut reversed = routes.clone();
        reversed.reverse();
        let reordered = optimize_fixed_charge(&context, &positions, &reversed)
            .unwrap_or_else(|error| panic!("reordered seed={seed}: {error}"));
        assert_eq!(
            actual, reordered,
            "input-order determinism mismatch for seed={seed}"
        );
    }
}
