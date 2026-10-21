use finance_route_clearing_optimizer::batch::{
    select_atomic_batch, BatchAccount, BatchContext, BatchPayment, BatchRoute,
};
use finance_route_clearing_optimizer::AuthorityQualification;
use std::time::Instant;

const SIZES: [usize; 4] = [4, 8, 12, 16];
const WARMUPS: usize = 5;
const REPETITIONS: usize = 101;

fn fixture(
    candidates: usize,
) -> (
    BatchContext,
    Vec<BatchAccount>,
    Vec<BatchRoute>,
    Vec<BatchPayment>,
) {
    let context = BatchContext {
        rail_id: "synthetic-atomic-batch".into(),
        asset_id: "ZAR-minor-unit".into(),
        policy_version: "atomic-batch-bench-v1".into(),
        tick: 0,
        authority: AuthorityQualification::ExternallyQualified {
            evidence_ref: "fixture-only://not-legal-evidence".into(),
        },
    };
    let accounts = (0..candidates)
        .map(|index| BatchAccount {
            participant_id: format!("P{index:02}"),
            balance_minor: 0,
        })
        .collect::<Vec<_>>();
    let routes = (0..candidates)
        .map(|index| BatchRoute {
            route_id: format!("R{index:02}"),
            rail_id: context.rail_id.clone(),
            asset_id: context.asset_id.clone(),
            policy_version: context.policy_version.clone(),
            enabled: true,
            remaining_capacity_minor: 100,
            available_now: true,
            fixed_fee_units: ((index * 7 + 3) % 11) as u64,
            fee_units_per_minor: ((index * 5 + 1) % 7) as u64,
        })
        .collect::<Vec<_>>();
    let payments = (0..candidates)
        .map(|index| BatchPayment {
            payment_id: format!("PAY{index:02}"),
            rail_id: context.rail_id.clone(),
            asset_id: context.asset_id.clone(),
            policy_version: context.policy_version.clone(),
            route_id: format!("R{index:02}"),
            from: format!("P{index:02}"),
            to: format!("P{:02}", (index + 1) % candidates),
            amount_minor: 100,
            arrival_tick: 0,
            deadline_tick: Some(1),
        })
        .collect::<Vec<_>>();
    (context, accounts, routes, payments)
}

fn percentile(samples: &mut [u128], percentile: usize) -> u128 {
    samples.sort_unstable();
    let index = ((samples.len() - 1) * percentile).div_ceil(100);
    samples[index.min(samples.len() - 1)]
}

fn main() {
    println!(
        "candidates,participants,selected_count,selected_gross_minor,modeled_fee_units,evaluated_subsets,p50_ns,p95_ns,p99_ns"
    );
    for candidates in SIZES {
        let (context, accounts, routes, payments) = fixture(candidates);
        for _ in 0..WARMUPS {
            let _ = select_atomic_batch(&context, &accounts, &routes, &payments)
                .expect("synthetic batch warmup must be valid");
        }

        let mut timings = Vec::with_capacity(REPETITIONS);
        let mut final_plan = None;
        for _ in 0..REPETITIONS {
            let start = Instant::now();
            let plan = select_atomic_batch(&context, &accounts, &routes, &payments)
                .expect("synthetic atomic-cycle fixture must be valid");
            timings.push(start.elapsed().as_nanos());
            final_plan = Some(plan);
        }

        let plan = final_plan.expect("at least one measured repetition");
        assert_eq!(plan.selected_payment_count, candidates);
        assert_eq!(plan.selected_gross_minor, (candidates as u128) * 100);
        assert_eq!(plan.evaluated_subsets, 1_u64 << candidates);
        let p50 = percentile(&mut timings.clone(), 50);
        let p95 = percentile(&mut timings.clone(), 95);
        let p99 = percentile(&mut timings, 99);
        println!(
            "{candidates},{candidates},{},{},{},{},{p50},{p95},{p99}",
            plan.selected_payment_count,
            plan.selected_gross_minor,
            plan.modeled_fee_units,
            plan.evaluated_subsets
        );
    }
}
