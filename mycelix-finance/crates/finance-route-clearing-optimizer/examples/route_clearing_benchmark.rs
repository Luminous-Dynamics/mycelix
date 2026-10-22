use finance_route_clearing_optimizer::{
    greedy_match, optimize, AuthorityQualification, ClearingContext, ParticipantNet,
    SettlementRoute,
};
use std::time::Instant;

const MEASURED_REPETITIONS: usize = 101;

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

    for route in &mut routes {
        let source_index = route
            .route_id
            .rsplit('-')
            .next()
            .and_then(|suffix| suffix.parse::<usize>().ok())
            .expect("generated route IDs end in a source index");
        match scenario {
            "cyclic" => {}
            // Each debtor has two outgoing arcs, so 500 would still permit its
            // full 1,000-unit obligation. 400 forces a measurable 20% capacity
            // deficit even when the global flow is allocated optimally.
            "capacity-stress" => route.capacity_minor = 400,
            "route-outage" if source_index % 11 == 0 => route.enabled = false,
            "route-outage" => {}
            _ => panic!("unknown benchmark scenario"),
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
        "scenario,route_arcs,participants,optimized_flow_minor,optimized_cost_units,optimized_full,greedy_flow_minor,greedy_cost_units,greedy_full,flow_gain_minor,same_flow_cost_savings_units,opt_p50_us,opt_p95_us,opt_p99_us,greedy_p50_us,greedy_p95_us,greedy_p99_us"
    );

    for scenario in ["cyclic", "capacity-stress", "route-outage"] {
        for route_count in [10_usize, 30, 70, 140, 700] {
            let (context, positions, routes) = make_case(route_count, scenario);

            // Warm both code paths before collecting the measured samples.
            optimize(&context, &positions, &routes).expect("optimizer warm-up failed");
            greedy_match(&context, &positions, &routes).expect("greedy warm-up failed");

            let mut optimize_samples = Vec::with_capacity(MEASURED_REPETITIONS);
            let mut greedy_samples = Vec::with_capacity(MEASURED_REPETITIONS);

            for _ in 0..MEASURED_REPETITIONS {
                let start = Instant::now();
                let _plan = optimize(&context, &positions, &routes).expect("optimizer failed");
                optimize_samples.push(start.elapsed().as_micros());

                let start = Instant::now();
                let _plan =
                    greedy_match(&context, &positions, &routes).expect("greedy baseline failed");
                greedy_samples.push(start.elapsed().as_micros());
            }

            let optimized_plan =
                optimize(&context, &positions, &routes).expect("optimizer final sample failed");
            let greedy_plan =
                greedy_match(&context, &positions, &routes).expect("greedy final sample failed");

            // Fail the benchmark itself if the optimizer violates its stated
            // lexicographic objective: maximize feasible flow, then minimize cost.
            assert!(
                optimized_plan.total_allocated_minor >= greedy_plan.total_allocated_minor,
                "optimizer flow must dominate greedy baseline for {scenario}/{route_count}"
            );
            if optimized_plan.total_allocated_minor == greedy_plan.total_allocated_minor {
                assert!(
                    optimized_plan.total_modeled_cost_units <= greedy_plan.total_modeled_cost_units,
                    "optimizer cost must not exceed greedy at equal flow for {scenario}/{route_count}"
                );
            }

            let side = route_count / 2;
            let expected_flow = match scenario {
                "cyclic" => side as u128 * 1_000,
                "capacity-stress" => side as u128 * 800,
                "route-outage" => (0..side).filter(|i| i % 11 != 0).count() as u128 * 1_000,
                _ => unreachable!("scenario list is fixed above"),
            };
            assert_eq!(
                optimized_plan.total_allocated_minor, expected_flow,
                "optimized flow must match the scenario's constructed capacity ceiling"
            );
            assert_eq!(
                optimized_plan.fully_allocatable,
                scenario == "cyclic",
                "only the unconstrained cyclic case should be fully allocatable"
            );
            let flow_gain =
                optimized_plan.total_allocated_minor - greedy_plan.total_allocated_minor;
            let equal_flow_cost_savings =
                if optimized_plan.total_allocated_minor == greedy_plan.total_allocated_minor {
                    greedy_plan.total_modeled_cost_units
                        - optimized_plan.total_modeled_cost_units
                } else {
                    0
                };

            println!(
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                scenario,
                route_count,
                positions.len(),
                optimized_plan.total_allocated_minor,
                optimized_plan.total_modeled_cost_units,
                optimized_plan.fully_allocatable,
                greedy_plan.total_allocated_minor,
                greedy_plan.total_modeled_cost_units,
                greedy_plan.fully_allocatable,
                flow_gain,
                equal_flow_cost_savings,
                percentile(&optimize_samples, 50),
                percentile(&optimize_samples, 95),
                percentile(&optimize_samples, 99),
                percentile(&greedy_samples, 50),
                percentile(&greedy_samples, 95),
                percentile(&greedy_samples, 99),
            );
        }
    }
}
