use finance_route_clearing_optimizer::{
    greedy_match, optimize, AuthorityQualification, ClearingContext, ParticipantNet,
    SettlementRoute,
};
use std::time::Instant;

fn make_case(route_count: usize) -> (ClearingContext, Vec<ParticipantNet>, Vec<SettlementRoute>) {
    assert!(route_count >= 4 && route_count % 2 == 0);
    let side = route_count / 2;
    let context = ClearingContext {
        network_id: "bench-network".into(),
        asset_id: "bench-asset-minor".into(),
        netting_set_id: "bench-netting-set".into(),
        policy_version: "bench-policy-v1".into(),
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

    // A sparse cyclic graph: every debtor can route to its paired creditor or
    // the next creditor. The deterministic costs make each run reproducible.
    let mut routes = Vec::with_capacity(route_count);
    for i in 0..side {
        let primary_cost = ((i * 37 + 11) % 97 + 1) as u64;
        let next_cost = ((i * 17 + 29) % 97 + 1) as u64;
        routes.push(SettlementRoute {
            route_id: format!("primary-{i:04}"),
            network_id: context.network_id.clone(),
            asset_id: context.asset_id.clone(),
            netting_set_id: context.netting_set_id.clone(),
            policy_version: context.policy_version.clone(),
            from_member: format!("D{i:04}"),
            to_member: format!("C{i:04}"),
            enabled: true,
            capacity_minor: 1_000,
            cost_units_per_minor: primary_cost,
        });
        routes.push(SettlementRoute {
            route_id: format!("next-{i:04}"),
            network_id: context.network_id.clone(),
            asset_id: context.asset_id.clone(),
            netting_set_id: context.netting_set_id.clone(),
            policy_version: context.policy_version.clone(),
            from_member: format!("D{i:04}"),
            to_member: format!("C{:04}", (i + 1) % side),
            enabled: true,
            capacity_minor: 1_000,
            cost_units_per_minor: next_cost,
        });
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
        "route_arcs,participants,optimized_flow_minor,optimized_cost_units,greedy_flow_minor,greedy_cost_units,opt_p50_us,opt_p95_us,greedy_p50_us,greedy_p95_us"
    );

    for route_count in [10_usize, 30, 70, 140, 700] {
        let (context, positions, routes) = make_case(route_count);

        // Warm both code paths before collecting the measured samples.
        optimize(&context, &positions, &routes).expect("optimizer warm-up failed");
        greedy_match(&context, &positions, &routes).expect("greedy warm-up failed");

        let mut optimize_samples = Vec::with_capacity(25);
        let mut greedy_samples = Vec::with_capacity(25);
        let mut optimized_plan = None;
        let mut greedy_plan = None;

        for _ in 0..25 {
            let start = Instant::now();
            optimized_plan =
                Some(optimize(&context, &positions, &routes).expect("optimizer failed"));
            optimize_samples.push(start.elapsed().as_micros());

            let start = Instant::now();
            greedy_plan =
                Some(greedy_match(&context, &positions, &routes).expect("greedy baseline failed"));
            greedy_samples.push(start.elapsed().as_micros());
        }

        let optimized_plan = optimized_plan.expect("at least one sample");
        let greedy_plan = greedy_plan.expect("at least one sample");
        println!(
            "{},{},{},{},{},{},{},{},{},{}",
            route_count,
            positions.len(),
            optimized_plan.total_allocated_minor,
            optimized_plan.total_modeled_cost_units,
            greedy_plan.total_allocated_minor,
            greedy_plan.total_modeled_cost_units,
            percentile(&optimize_samples, 50),
            percentile(&optimize_samples, 95),
            percentile(&greedy_samples, 50),
            percentile(&greedy_samples, 95),
        );
    }
}
