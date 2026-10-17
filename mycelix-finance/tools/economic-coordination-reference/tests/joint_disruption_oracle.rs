//! Independent tiny-instance oracle for joint disruption stress.
//!
//! The oracle enumerates literal binary allocation masks and derives feasibility,
//! cost, and shortage vectors directly. It does not call production filtering,
//! Pareto, or optimization helpers.

use std::collections::BTreeSet;

use economic_coordination_reference::joint_capacity::{
    CapabilityMapping, JointOffer, JointRequest, SharedCapacityPool,
};
use economic_coordination_reference::joint_disruption::{
    JointDisruptionScenario, JointDisruptionStressProblem, evaluate_joint_disruption_stress,
};
use economic_coordination_reference::joint_optimizer::{JointObjective, JointOptimizationProblem};
use economic_coordination_reference::{EvidenceOrigin, EvidenceStatus};

type OracleKey = (u64, Vec<(String, u64)>);

fn request(id: &str) -> JointRequest {
    JointRequest {
        request_id: id.to_owned(),
        capability_id: "repair-kit".to_owned(),
        capability_profile_digest: "repair-kit-profile-v1".to_owned(),
        unit: "kit".to_owned(),
        currency: "USD".to_owned(),
        required_quantity: 1,
        max_total_cost_minor: 100,
        deliver_by: 500,
        priority_rank: if id == "urgent" { 1 } else { 10 },
    }
}

fn offer(id: &str, supplier: &str, cost: u64) -> JointOffer {
    JointOffer {
        offer_id: id.to_owned(),
        supplier_id: supplier.to_owned(),
        source_uri: format!("https://example.invalid/{id}"),
        source_payload_digest: format!("source-{id}"),
        qualification_receipt_id: format!("qualification-{id}"),
        offered_capability_id: "repair-kit".to_owned(),
        offered_profile_digest: "repair-kit-profile-v1".to_owned(),
        unit: "kit".to_owned(),
        currency: "USD".to_owned(),
        capacity_pool_id: "pool-shared".to_owned(),
        max_allocatable_quantity: 1,
        minimum_order_quantity: 1,
        order_increment: 1,
        unit_cost_minor: cost,
        arrives_at: 100,
        evidence_status: EvidenceStatus::Verified,
        evidence_origin: EvidenceOrigin::LiveDomain,
        evidence_observed_at: 10,
        evidence_valid_until: 1_000,
    }
}

fn mapping(request_id: &str, offer_id: &str) -> CapabilityMapping {
    CapabilityMapping {
        mapping_id: format!("map-{request_id}-{offer_id}"),
        mapping_digest: format!("mapping-{request_id}-{offer_id}"),
        request_id: request_id.to_owned(),
        offer_id: offer_id.to_owned(),
        requested_capability_id: "repair-kit".to_owned(),
        requested_profile_digest: "repair-kit-profile-v1".to_owned(),
        offered_capability_id: "repair-kit".to_owned(),
        offered_profile_digest: "repair-kit-profile-v1".to_owned(),
        evidence_status: EvidenceStatus::Verified,
        evidence_origin: EvidenceOrigin::LiveDomain,
        observed_at: 10,
        valid_until: 1_000,
    }
}

fn fixture() -> JointOptimizationProblem {
    // Two requests, two one-unit suppliers, and one physical shared pool.
    // Each edge is binary in this tiny independently enumerable instance.
    let requests = vec![request("routine"), request("urgent")];
    let offers = vec![
        offer("offer-a", "supplier-a", 5),
        offer("offer-b", "supplier-b", 8),
    ];
    let mappings = requests
        .iter()
        .flat_map(|request| {
            offers
                .iter()
                .map(move |offer| mapping(&request.request_id, &offer.offer_id))
        })
        .collect();

    JointOptimizationProblem {
        evaluated_at: 20,
        input_manifest_digest: "oracle-base-manifest".to_owned(),
        requests,
        offers,
        capacity_pools: vec![SharedCapacityPool {
            capacity_pool_id: "pool-shared".to_owned(),
            unit: "kit".to_owned(),
            time_bucket: "week-44".to_owned(),
            capacity_quantity: 2,
            snapshot_digest: "oracle-pool-snapshot".to_owned(),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }],
        capability_mappings: mappings,
        search_node_limit: 100_000,
    }
}

fn dominates(left: &OracleKey, right: &OracleKey) -> bool {
    if left.1.len() != right.1.len() || left.0 > right.0 {
        return false;
    }

    let mut strict = left.0 < right.0;
    for ((left_id, left_shortage), (right_id, right_shortage)) in left.1.iter().zip(&right.1) {
        if left_id != right_id || left_shortage > right_shortage {
            return false;
        }
        strict |= left_shortage < right_shortage;
    }
    strict
}

/// Independent model: enumerate all 16 binary edge subsets. Edge order is
/// routine/a, routine/b, urgent/a, urgent/b. The failed domains eliminate the
/// corresponding edges before feasibility and Pareto filtering.
fn oracle_frontier(
    failed_supplier_a: bool,
    failed_shared_pool: bool,
) -> BTreeSet<OracleKey> {
    const EDGE_REQUEST: [usize; 4] = [0, 0, 1, 1];
    const EDGE_OFFER: [usize; 4] = [0, 1, 0, 1];
    const EDGE_COST: [u64; 4] = [5, 8, 5, 8];

    let mut feasible = BTreeSet::new();
    for mask in 0u8..16 {
        if failed_shared_pool && mask != 0 {
            continue;
        }
        if failed_supplier_a && (mask & 0b0101) != 0 {
            continue;
        }

        let mut request_quantity = [0u32; 2];
        let mut offer_quantity = [0u32; 2];
        let mut total_pool_quantity = 0u32;
        let mut total_cost = 0u64;
        for edge in 0..4 {
            if mask & (1 << edge) == 0 {
                continue;
            }
            request_quantity[EDGE_REQUEST[edge]] += 1;
            offer_quantity[EDGE_OFFER[edge]] += 1;
            total_pool_quantity += 1;
            total_cost += EDGE_COST[edge];
        }
        if request_quantity.iter().any(|quantity| *quantity > 1)
            || offer_quantity.iter().any(|quantity| *quantity > 1)
            || total_pool_quantity > 2
        {
            continue;
        }

        feasible.insert((
            total_cost,
            vec![
                ("routine".to_owned(), u64::from(1 - request_quantity[0])),
                ("urgent".to_owned(), u64::from(1 - request_quantity[1])),
            ],
        ));
    }

    let all: Vec<_> = feasible.iter().cloned().collect();
    feasible
        .iter()
        .filter(|candidate| {
            !all.iter()
                .any(|other| other != *candidate && dominates(other, candidate))
        })
        .cloned()
        .collect()
}

fn actual_frontier(objective: &JointObjective) -> OracleKey {
    (
        objective.total_cost_minor,
        objective
            .unfilled_by_request
            .iter()
            .map(|shortage| (shortage.request_id.clone(), shortage.unfilled_quantity))
            .collect(),
    )
}

fn evaluate_case(
    scenario_id: &str,
    failed_supplier_ids: Vec<String>,
    failed_capacity_pool_ids: Vec<String>,
) -> BTreeSet<OracleKey> {
    let scenario = JointDisruptionScenario {
        scenario_id: scenario_id.to_owned(),
        scenario_manifest_digest: format!("manifest-{scenario_id}"),
        failed_supplier_ids,
        failed_capacity_pool_ids,
    };
    let set = JointDisruptionStressProblem {
        scenario_set_id: "oracle-scenario-set".to_owned(),
        scenario_set_digest: "oracle-scenario-set-digest".to_owned(),
        scenarios: vec![scenario],
        max_total_search_nodes: 1_000_000,
    };
    let report = evaluate_joint_disruption_stress(&fixture(), &set).expect("valid scenario");
    report.scenario_results[0]
        .optimization
        .frontier
        .iter()
        .map(|candidate| actual_frontier(&candidate.objective))
        .collect()
}

#[test]
fn exhaustive_binary_oracle_agrees_for_control_supplier_outage_and_pool_outage() {
    assert_eq!(evaluate_case("control", vec![], vec![]), oracle_frontier(false, false));
    assert_eq!(
        evaluate_case("supplier-a-down", vec!["supplier-a".to_owned()], vec![]),
        oracle_frontier(true, false)
    );
    assert_eq!(
        evaluate_case("shared-pool-down", vec![], vec!["pool-shared".to_owned()]),
        oracle_frontier(false, true)
    );
}
