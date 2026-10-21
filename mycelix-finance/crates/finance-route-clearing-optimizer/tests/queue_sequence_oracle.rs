use finance_route_clearing_optimizer::queue::{
    simulate_queue, PaymentOrder, PaymentState, QueueAccount, QueueContext, QueuePolicy, QueueRoute,
};
use std::collections::BTreeMap;

fn context() -> QueueContext {
    QueueContext {
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        policy_version: "oracle-policy-v1".into(),
        horizon_tick: 0,
    }
}

fn account(id: &str, balance_minor: u64) -> QueueAccount {
    QueueAccount {
        participant_id: id.into(),
        balance_minor,
    }
}

fn route(id: &str, capacity_minor: u64) -> QueueRoute {
    QueueRoute {
        route_id: id.into(),
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        policy_version: "oracle-policy-v1".into(),
        enabled: true,
        capacity_minor_per_window: capacity_minor,
        capacity_window_ticks: 1,
        fixed_fee_units: 2,
        fee_units_per_minor: 1,
        outages: Vec::new(),
    }
}

fn payment(
    id: &str,
    route_id: &str,
    from: &str,
    to: &str,
    amount_minor: u64,
    priority: u8,
) -> PaymentOrder {
    PaymentOrder {
        payment_id: id.into(),
        rail_id: "oracle-rail".into(),
        asset_id: "oracle-asset".into(),
        policy_version: "oracle-policy-v1".into(),
        route_id: route_id.into(),
        from: from.into(),
        to: to.into(),
        amount_minor,
        arrival_tick: 0,
        deadline_tick: None,
        priority,
    }
}

/// Independent exhaustive one-tick settlement-sequence oracle. It explores every
/// feasible ordering of eligible gross payments, updating balances and per-route
/// capacity after each individual transfer. It does not call the production simulator.
/// Objective: maximize settled value, then minimize modeled fixed + variable fees.
fn exhaustive_sequence_oracle(
    accounts: &[QueueAccount],
    routes: &[QueueRoute],
    payments: &[PaymentOrder],
) -> (u128, u128) {
    fn visit(
        routes: &[QueueRoute],
        payments: &[PaymentOrder],
        account_index: &BTreeMap<String, usize>,
        route_index: &BTreeMap<String, usize>,
        balances: &mut [u64],
        route_used: &mut [u64],
        settled_mask: u64,
        settled_value: u128,
        modeled_cost: u128,
        best: &mut (u128, u128),
    ) {
        if settled_value > best.0 || (settled_value == best.0 && modeled_cost < best.1) {
            *best = (settled_value, modeled_cost);
        }

        for index in 0..payments.len() {
            let bit = 1_u64 << index;
            if settled_mask & bit != 0 {
                continue;
            }

            let payment = &payments[index];
            let Some(&r) = route_index.get(&payment.route_id) else {
                continue;
            };
            let Some(&from) = account_index.get(&payment.from) else {
                continue;
            };
            let Some(&to) = account_index.get(&payment.to) else {
                continue;
            };
            let route = &routes[r];
            if !route.enabled
                || payment.amount_minor == 0
                || payment.amount_minor > route.capacity_minor_per_window
                || balances[from] < payment.amount_minor
                || payment.amount_minor > route.capacity_minor_per_window - route_used[r]
            {
                continue;
            }

            let variable_cost = payment.amount_minor as u128 * route.fee_units_per_minor as u128;
            let fee = route.fixed_fee_units as u128 + variable_cost;
            balances[from] -= payment.amount_minor;
            balances[to] += payment.amount_minor;
            route_used[r] += payment.amount_minor;

            visit(
                routes,
                payments,
                account_index,
                route_index,
                balances,
                route_used,
                settled_mask | bit,
                settled_value + payment.amount_minor as u128,
                modeled_cost + fee,
                best,
            );

            route_used[r] -= payment.amount_minor;
            balances[to] -= payment.amount_minor;
            balances[from] += payment.amount_minor;
        }
    }

    assert!(
        payments.len() <= 63,
        "oracle is intentionally bounded to tiny instances"
    );
    let account_index = accounts
        .iter()
        .enumerate()
        .map(|(i, a)| (a.participant_id.clone(), i))
        .collect::<BTreeMap<_, _>>();
    let route_index = routes
        .iter()
        .enumerate()
        .map(|(i, r)| (r.route_id.clone(), i))
        .collect::<BTreeMap<_, _>>();
    let mut balances = accounts.iter().map(|a| a.balance_minor).collect::<Vec<_>>();
    let mut route_used = vec![0_u64; routes.len()];
    let mut best = (0_u128, u128::MAX);

    visit(
        routes,
        payments,
        &account_index,
        &route_index,
        &mut balances,
        &mut route_used,
        0,
        0,
        0,
        &mut best,
    );
    best
}

#[test]
fn exact_sequence_oracle_demonstrates_the_cost_of_strict_head_of_line_blocking() {
    let accounts = [account("A", 0), account("B", 0), account("C", 5)];
    let routes = [route("ab", 5), route("ca", 5)];
    let payments = [
        payment("a-first", "ab", "A", "B", 5, 0),
        payment("c-funds-a", "ca", "C", "A", 5, 0),
    ];

    let oracle = exhaustive_sequence_oracle(&accounts, &routes, &payments);
    let strict = simulate_queue(
        &context(),
        &accounts,
        &routes,
        &payments,
        QueuePolicy::StrictFifo,
    )
    .unwrap();
    let scanning = simulate_queue(
        &context(),
        &accounts,
        &routes,
        &payments,
        QueuePolicy::LiquidityAwareFifo,
    )
    .unwrap();

    assert_eq!(oracle.0, 10);
    assert_eq!(strict.settled_minor, 0);
    assert_eq!(scanning.settled_minor, oracle.0);
    assert_eq!(scanning.modeled_fee_units, oracle.1);
}

#[test]
fn all_queue_policies_stay_within_the_exact_oracle_on_64_small_traces() {
    for seed in 0..64_usize {
        let accounts = [
            account("A", ((seed * 3) % 7) as u64),
            account("B", ((seed * 5 + 2) % 7) as u64),
            account("C", ((seed * 7 + 1) % 7) as u64),
        ];
        let routes = [route("ab", 5), route("bc", 5), route("ca", 5)];
        let payments = [
            payment("p0", "ab", "A", "B", ((seed * 3 + 1) % 4 + 1) as u64, 1),
            payment("p1", "ab", "A", "B", ((seed * 5 + 2) % 4 + 1) as u64, 4),
            payment("p2", "bc", "B", "C", ((seed * 7 + 3) % 4 + 1) as u64, 2),
            payment("p3", "bc", "B", "C", ((seed * 11 + 1) % 4 + 1) as u64, 3),
            payment("p4", "ca", "C", "A", ((seed * 13 + 2) % 4 + 1) as u64, 5),
            payment("p5", "ca", "C", "A", ((seed * 17 + 3) % 4 + 1) as u64, 0),
        ];
        let oracle = exhaustive_sequence_oracle(&accounts, &routes, &payments);

        for policy in [
            QueuePolicy::StrictFifo,
            QueuePolicy::LiquidityAwareFifo,
            QueuePolicy::PriorityScan,
        ] {
            let result = simulate_queue(&context(), &accounts, &routes, &payments, policy).unwrap();
            assert!(
                result.settled_minor <= oracle.0,
                "policy={policy:?}, seed={seed} settled more than oracle"
            );
            if result.settled_minor == oracle.0 {
                assert!(
                    result.modeled_fee_units >= oracle.1,
                    "policy={policy:?}, seed={seed} beat exact cost oracle"
                );
            }
            assert_eq!(
                result.settled_count,
                result
                    .payments
                    .iter()
                    .filter(|p| matches!(p.state, PaymentState::Settled { .. }))
                    .count()
            );
        }
    }
}
