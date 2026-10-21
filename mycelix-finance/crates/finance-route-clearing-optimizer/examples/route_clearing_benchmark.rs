use finance_route_clearing_optimizer::{
    greedy_match, optimize, AuthorityQualification, ClearingContext, ParticipantNet,
    SettlementRoute,
};
use std::time::Instant;

const SAMPLE_COUNT: usize = 101;

fn make_case(
    route_count: usize,
    scenario: &str,
) -> (ClearingContext, Vec<ParticipantNet>, Vec<SettlementRoute>) {
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
            available_liquidity_minor: if scenario == "liquidity-stress" {
                500
            } else {
                1_000
            },
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

    for route in &mut routes {
        let source_index = route
            .route_id
            .rsplit('-')
            .next()
            .and_then(|suffix| suffix.parse::<usize>().ok())
            .expect("generated route IDs end in a source index");
        match scenario {
            "cyclic" => {}
            "capacity-stress" => route.capacity_minor = 500,
            "liquidity-stress" => {},
            "route-outage" if source_index % 11 == 0 => route.enabled = false,
            "route-outage" => {}
            _ => panic!("unknown benchmark scenario"),
        }
    }

    (context, positions, routes)
}

fn residual_total(residuals: &[finance_route_clearing_optimizer::Residual]) -> u128 {
    residuals.iter().map(|item| u128::from(item.amount_minor)).sum()
}

fn cost_per_allocated_minor(
    plan: &finance_route_clearing_optimizer::AllocationPlan,
) -> Option<f64> {
    (plan.total_allocated_minor > 0).then_some(
        plan.total_modeled_cost_units as f64 / plan.total_allocated_minor as f64,
    )
}

fn metric(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.6}"))
        .unwrap_or_else(|| "NA".to_owned())
}

fn percentile(values: &[u128], p: usize) -> u128 {
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (p * sorted.len() + 99) / 100;
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn main() {
    println!(
        "scenario,route_arcs,participants,optimized_flow_minor,optimized_cost_units,optimized_cost_per_minor,optimized_fully_allocatable,optimized_residual_debt_minor,optimized_residual_credit_minor,greedy_flow_minor,greedy_cost_units,greedy_cost_per_minor,greedy_fully_allocatable,greedy_residual_debt_minor,greedy_residual_credit_minor,flow_gap_minor,greedy_unit_cost_delta_pct,opt_p50_ns,opt_p95_ns,greedy_p50_ns,greedy_p95_ns"
    );

    for scenario in ["cyclic", "capacity-stress", "route-outage", "liquidity-stress"] {
        for route_count in [10_usize, 30, 70, 140, 700] {
            let (context, positions, routes) = make_case(route_count, scenario);

            // Warm both code paths before collecting the measured samples.
            optimize(&context, &positions, &routes).expect("optimizer warm-up failed");
            greedy_match(&context, &positions, &routes).expect("greedy warm-up failed");

            let mut optimize_samples = Vec::with_capacity(SAMPLE_COUNT);
            let mut greedy_samples = Vec::with_capacity(SAMPLE_COUNT);

            for _ in 0..SAMPLE_COUNT {
                let start = Instant::now();
                let _plan = optimize(&context, &positions, &routes).expect("optimizer failed");
                optimize_samples.push(start.elapsed().as_nanos());

                let start = Instant::now();
                let _plan =
                    greedy_match(&context, &positions, &routes).expect("greedy baseline failed");
                greedy_samples.push(start.elapsed().as_nanos());
            }

            let optimized_plan =
                optimize(&context, &positions, &routes).expect("optimizer final sample failed");
            let greedy_plan =
                greedy_match(&context, &positions, &routes).expect("greedy final sample failed");

            let optimized_cost_per_minor = cost_per_allocated_minor(&optimized_plan);
            let greedy_cost_per_minor = cost_per_allocated_minor(&greedy_plan);
            let greedy_unit_cost_delta_pct =
                match (optimized_cost_per_minor, greedy_cost_per_minor) {
                    (Some(optimal), Some(greedy)) if optimal > 0.0 => {
                        Some((greedy / optimal - 1.0) * 100.0)
                    }
                    _ => None,
                };
            println!(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                scenario,
                route_count,
                positions.len(),
                optimized_plan.total_allocated_minor,
                optimized_plan.total_modeled_cost_units,
                metric(optimized_cost_per_minor),
                optimized_plan.fully_allocatable,
                residual_total(&optimized_plan.residual_debtors),
                residual_total(&optimized_plan.residual_creditors),
                greedy_plan.total_allocated_minor,
                greedy_plan.total_modeled_cost_units,
                metric(greedy_cost_per_minor),
                greedy_plan.fully_allocatable,
                residual_total(&greedy_plan.residual_debtors),
                residual_total(&greedy_plan.residual_creditors),
                optimized_plan
                    .total_allocated_minor
                    .saturating_sub(greedy_plan.total_allocated_minor),
                metric(greedy_unit_cost_delta_pct),
                percentile(&optimize_samples, 50),
                percentile(&optimize_samples, 95),
                percentile(&greedy_samples, 50),
                percentile(&greedy_samples, 95),
            );
        }
    }
}
