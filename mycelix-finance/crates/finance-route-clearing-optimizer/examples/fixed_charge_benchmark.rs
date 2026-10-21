use finance_route_clearing_optimizer::fixed_charge::{optimize_fixed_charge, FixedChargeRoute};
use finance_route_clearing_optimizer::{
    optimize, AuthorityQualification, ClearingContext, ParticipantNet, SettlementRoute,
};
use std::time::Instant;

fn make_case(route_count: usize) -> (ClearingContext, Vec<ParticipantNet>, Vec<FixedChargeRoute>) {
    assert!((4..=12).contains(&route_count) && route_count % 2 == 0);
    let side = route_count / 2;
    let context = ClearingContext {
        network_id: "fixed-cost-bench-network".into(),
        asset_id: "fixed-cost-bench-asset".into(),
        netting_set_id: "fixed-cost-bench-set".into(),
        policy_version: "fixed-cost-bench-v1".into(),
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "fixture-only://not-legal-evidence".into(),
        },
    };
    let mut positions = Vec::with_capacity(side * 2);
    for i in 0..side {
        positions.push(ParticipantNet {
            member_id: format!("D{i:04}"),
            net_minor: 1_000,
            available_liquidity_minor: 1_000,
        });
    }
    for i in 0..side {
        positions.push(ParticipantNet {
            member_id: format!("C{i:04}"),
            net_minor: -1_000,
            available_liquidity_minor: 0,
        });
    }

    let mut routes = Vec::with_capacity(route_count);
    for i in 0..side {
        for (suffix, creditor, offset) in
            [("primary", i, 0_usize), ("next", (i + 1) % side, 1_usize)]
        {
            let cost = ((i * (37 + 2 * offset) + 11 + offset * 13) % 47 + 1) as u64;
            let fixed = ((i * (19 + offset * 8) + 7 + offset * 5) % 53) as u64;
            routes.push(FixedChargeRoute {
                route: SettlementRoute {
                    route_id: format!("{suffix}-{i:04}"),
                    network_id: context.network_id.clone(),
                    asset_id: context.asset_id.clone(),
                    netting_set_id: context.netting_set_id.clone(),
                    policy_version: context.policy_version.clone(),
                    from_member: format!("D{i:04}"),
                    to_member: format!("C{creditor:04}"),
                    enabled: true,
                    capacity_minor: 1_000,
                    cost_units_per_minor: cost,
                },
                activation_cost_units: fixed,
            });
        }
    }

    (context, positions, routes)
}

fn percentile(values: &[u128], p: usize) -> u128 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (p * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn main() {
    println!(
        "candidate_routes,subsets,fixed_flow_minor,fixed_variable_cost,fixed_activation_cost,fixed_total_cost,linear_flow_minor,linear_variable_cost,fixed_p50_us,fixed_p95_us,linear_p50_us,linear_p95_us"
    );

    for route_count in [4_usize, 6, 8, 10, 12] {
        let (context, positions, candidates) = make_case(route_count);
        let linear_routes = candidates
            .iter()
            .map(|candidate| candidate.route.clone())
            .collect::<Vec<_>>();

        let mut fixed_samples = Vec::with_capacity(9);
        let mut linear_samples = Vec::with_capacity(9);
        for _ in 0..9 {
            let start = Instant::now();
            let _plan = optimize_fixed_charge(&context, &positions, &candidates)
                .expect("fixed-charge optimizer failed");
            fixed_samples.push(start.elapsed().as_micros());

            let start = Instant::now();
            let _plan =
                optimize(&context, &positions, &linear_routes).expect("linear optimizer failed");
            linear_samples.push(start.elapsed().as_micros());
        }

        let fixed = optimize_fixed_charge(&context, &positions, &candidates)
            .expect("fixed-charge final sample failed");
        let linear =
            optimize(&context, &positions, &linear_routes).expect("linear final sample failed");

        println!(
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            route_count,
            fixed.evaluated_subsets,
            fixed.allocation.total_allocated_minor,
            fixed.allocation.total_modeled_cost_units,
            fixed.fixed_activation_cost_units,
            fixed.total_cost_units,
            linear.total_allocated_minor,
            linear.total_modeled_cost_units,
            percentile(&fixed_samples, 50),
            percentile(&fixed_samples, 95),
            percentile(&linear_samples, 50),
            percentile(&linear_samples, 95),
        );
    }
}
