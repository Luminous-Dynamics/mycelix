//! Test-only oracle for the capability-first sourcing Pareto frontier.
//!
//! This intentionally re-implements objective comparison locally instead of
//! calling the production module's private dominance predicate. All inputs are
//! synthetic and effects-disabled. It is a differential regression harness, not
//! a production-world sourcing-quality or performance result.

use economic_coordination_reference::sourcing::{
    evaluate_sourcing, EvidenceStatus, SourcingRequest, SupplierOffer,
};

const AS_OF: u64 = 1_000;
const OFFERS_PER_CASE: usize = 32;
const CASES: u64 = 2_048;

fn next(seed: &mut u64) -> u64 {
    *seed = seed
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1_442_695_040_888_963_407);
    *seed
}

fn request() -> SourcingRequest {
    SourcingRequest {
        as_of: AS_OF,
        request_id: "synthetic-need".into(),
        capability_id: "capability-v1".into(),
        capability_profile_digest: "fixture-profile-v1".into(),
        unit: "each".into(),
        currency: "USD".into(),
        quantity: 1,
        max_landed_cost_minor: u64::MAX,
        deliver_by: 10_000,
        max_quote_age_secs: 60,
        risk_model_digest: "fixture-risk-model-v1".into(),
    }
}

fn offers_for_case(case_seed: u64) -> Vec<SupplierOffer> {
    let mut seed = case_seed ^ 0xD1B5_4A32_D192_ED03;
    (0..OFFERS_PER_CASE)
        .map(|index| {
            let suffix = format!("{index:03}");
            SupplierOffer {
                offer_id: format!("offer-{suffix}"),
                supplier_id: format!("supplier-{suffix}"),
                source_uri: format!("https://supplier.example/fixture/{case_seed}/{suffix}"),
                source_payload_digest: format!("synthetic-payload-{case_seed}-{suffix}"),
                extractor_profile_id: "synthetic-extractor-v1".into(),
                capability_id: "capability-v1".into(),
                capability_profile_digest: "fixture-profile-v1".into(),
                unit: "each".into(),
                currency: "USD".into(),
                available_quantity: 1_000,
                unit_price_minor: 1 + next(&mut seed) % 10_000,
                shipping_minor: next(&mut seed) % 500,
                taxes_minor: next(&mut seed) % 250,
                other_fees_minor: next(&mut seed) % 100,
                lead_time_secs: next(&mut seed) % 4_001,
                risk_bps: (next(&mut seed) % 10_001) as u16,
                risk_model_digest: "fixture-risk-model-v1".into(),
                quote_observed_at: AS_OF,
                quote_valid_until: AS_OF + 100,
                quote_evidence_status: EvidenceStatus::Verified,
                supplier_credential_status: EvidenceStatus::Verified,
                supplier_credential_valid_until: AS_OF + 100,
            }
        })
        .collect()
}

#[derive(Clone, Debug)]
struct OracleProjection {
    id: String,
    total_landed_cost: u64,
    arrival_at: u64,
    risk_bps: u16,
}

fn oracle_frontier(offers: &[SupplierOffer]) -> (Vec<String>, Vec<String>) {
    let projected: Vec<OracleProjection> = offers
        .iter()
        .map(|offer| OracleProjection {
            id: offer.offer_id.clone(),
            total_landed_cost: offer.unit_price_minor
                + offer.shipping_minor
                + offer.taxes_minor
                + offer.other_fees_minor,
            arrival_at: AS_OF + offer.lead_time_secs,
            risk_bps: offer.risk_bps,
        })
        .collect();

    let mut frontier = Vec::new();
    let mut dominated = Vec::new();
    for candidate in &projected {
        let has_better_alternative = projected.iter().any(|alternative| {
            if alternative.id == candidate.id {
                return false;
            }
            let no_worse_on_all = alternative.total_landed_cost <= candidate.total_landed_cost
                && alternative.arrival_at <= candidate.arrival_at
                && alternative.risk_bps <= candidate.risk_bps;
            let strictly_better_on_one = alternative.total_landed_cost
                < candidate.total_landed_cost
                || alternative.arrival_at < candidate.arrival_at
                || alternative.risk_bps < candidate.risk_bps;
            no_worse_on_all && strictly_better_on_one
        });
        if has_better_alternative {
            dominated.push(candidate.id.clone());
        } else {
            frontier.push(candidate.id.clone());
        }
    }
    frontier.sort();
    dominated.sort();
    (frontier, dominated)
}

#[test]
fn frontier_matches_separate_bruteforce_oracle_for_seeded_cases() {
    for case_seed in 1..=CASES {
        let offers = offers_for_case(case_seed);
        let (expected_frontier, expected_dominated) = oracle_frontier(&offers);
        let actual = evaluate_sourcing(&request(), &offers)
            .expect("all generated offer identifiers are unique");

        let actual_frontier: Vec<String> = actual
            .pareto_frontier
            .iter()
            .map(|offer| offer.offer_id.clone())
            .collect();
        let actual_dominated: Vec<String> = actual
            .dominated
            .iter()
            .map(|offer| offer.offer_id.clone())
            .collect();

        assert_eq!(actual_frontier, expected_frontier, "frontier mismatch at seed {case_seed}");
        assert_eq!(actual_dominated, expected_dominated, "dominated mismatch at seed {case_seed}");
        assert_eq!(actual.eligible_count(), OFFERS_PER_CASE, "eligible count at seed {case_seed}");
        assert!(actual.rejected.is_empty(), "unexpected rejection at seed {case_seed}");
    }
}
