//! Dependency-free, deterministic microbenchmark for the sourcing evaluator.
//!
//! Run from the repository root:
//! cargo run --release --manifest-path mycelix-finance/tools/economic-coordination-reference/Cargo.toml --example sourcing_benchmark
//!
//! Output is TSV sample rows plus machine-readable summary comments. Candidate
//! construction is outside the timed region; evaluation includes validation,
//! sorting, Pareto classification and report construction. This does not fetch
//! supplier data or imply real-world sourcing quality.

use economic_coordination_reference::sourcing::{
    evaluate_sourcing, EvidenceOrigin, EvidenceStatus, SourcingRequest, SupplierOffer,
    SOURCING_EVALUATOR_PROFILE,
};
use std::hint::black_box;
use std::time::Instant;

fn make_request() -> SourcingRequest {
    let as_of = 10_000;
    SourcingRequest {
        as_of,
        required_evidence_origin: EvidenceOrigin::SyntheticSimulation,
        request_id: "benchmark-synthetic-request-v1".into(),
        capability_id: "benchmark-capability-v1".into(),
        capability_profile_digest: "benchmark-profile-sha256-v1".into(),
        unit: "each".into(),
        currency: "USD".into(),
        quantity: 10,
        max_landed_cost_minor: u64::MAX,
        deliver_by: as_of + 10_000,
        max_quote_age_secs: 60,
        max_inventory_age_secs: 60,
        risk_model_digest: "benchmark-risk-model-v1".into(),
    }
}

fn make_offers(count: usize) -> Vec<SupplierOffer> {
    let as_of = 10_000_u64;
    (0..count)
        .map(|index| {
            let id = format!("offer-{index:05}");
            SupplierOffer {
                offer_id: id.clone(),
                supplier_id: format!("supplier-{}", index % 257),
                source_uri: format!("https://supplier.example/benchmark/{id}"),
                source_payload_digest: format!("synthetic-payload-digest-{id}"),
                extractor_profile_id: "benchmark-extractor-v1".into(),
                quote_evidence_origin: EvidenceOrigin::SyntheticSimulation,
                supplier_credential_origin: EvidenceOrigin::SyntheticSimulation,
                capability_id: "benchmark-capability-v1".into(),
                capability_profile_digest: "benchmark-profile-sha256-v1".into(),
                unit: "each".into(),
                currency: "USD".into(),
                available_quantity: 10_000,
                inventory_observed_at: as_of - ((index % 30) as u64),
                inventory_valid_until: as_of + 600,
                inventory_evidence_status: EvidenceStatus::Verified,
                inventory_evidence_origin: EvidenceOrigin::SyntheticSimulation,
                unit_price_minor: 1 + ((index * 37) % 10_000) as u64,
                shipping_minor: ((index * 13) % 500) as u64,
                taxes_minor: ((index * 7) % 250) as u64,
                other_fees_minor: ((index * 3) % 100) as u64,
                lead_time_secs: ((index * 19) % 4_000) as u64,
                risk_bps: ((index * 127) % 10_001) as u16,
                risk_model_digest: "benchmark-risk-model-v1".into(),
                quote_observed_at: as_of - ((index % 30) as u64),
                quote_valid_until: as_of + 600,
                quote_evidence_status: EvidenceStatus::Verified,
                supplier_credential_status: EvidenceStatus::Verified,
                supplier_credential_valid_until: as_of + 600,
            }
        })
        .collect()
}

fn percentile(sorted_samples: &[u128], numerator: usize, denominator: usize) -> u128 {
    let rank = (sorted_samples.len() * numerator).div_ceil(denominator).max(1);
    sorted_samples[rank - 1]
}

fn main() {
    let request = make_request();
    let cases = [(10_usize, 100_usize), (100, 100), (1_000, 50), (10_000, 15)];

    println!("#benchmark=capability-first-sourcing-v1");
    println!("#evaluator_profile={SOURCING_EVALUATOR_PROFILE}");
    println!("#os={}", std::env::consts::OS);
    println!("#arch={}", std::env::consts::ARCH);
    println!("#debug_assertions={}", cfg!(debug_assertions));
    println!("#timing_scope=full evaluator call; candidate construction excluded");
    println!("record\toffers\titeration\telapsed_ns\teligible_count\tfrontier_count");

    for (offer_count, iterations) in cases {
        let offers = make_offers(offer_count);

        // Warm-up is deliberately not included in the reported sample set.
        let warmup = evaluate_sourcing(black_box(&request), black_box(&offers))
            .expect("fixed synthetic benchmark request is valid");
        black_box(warmup.eligible_count());

        let mut elapsed = Vec::with_capacity(iterations);
        let mut eligible_count = 0;
        let mut frontier_count = 0;

        for iteration in 0..iterations {
            let started = Instant::now();
            let report = evaluate_sourcing(black_box(&request), black_box(&offers))
                .expect("fixed synthetic benchmark request is valid");
            let elapsed_ns = started.elapsed().as_nanos();
            eligible_count = report.eligible_count();
            frontier_count = report.pareto_frontier.len();
            black_box((&report, eligible_count, frontier_count));
            elapsed.push(elapsed_ns);
            println!(
                "sample\t{offer_count}\t{iteration}\t{elapsed_ns}\t{eligible_count}\t{frontier_count}"
            );
        }

        elapsed.sort_unstable();
        let p50 = percentile(&elapsed, 50, 100);
        let p95 = percentile(&elapsed, 95, 100);
        let maximum = *elapsed.last().expect("at least one timing sample");
        println!(
            "#summary offers={offer_count} samples={iterations} p50_ns={p50} p95_ns={p95} max_ns={maximum} eligible={eligible_count} frontier={frontier_count}"
        );
    }
}
