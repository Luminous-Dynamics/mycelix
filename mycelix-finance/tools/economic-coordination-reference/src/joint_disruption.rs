//! Deterministic multi-request disruption stress evaluation.
//!
//! This module applies each declared failure scenario to a frozen joint
//! procurement problem, then delegates the surviving problem to the bounded
//! exact joint optimizer. A single failed supplier or capacity pool is applied
//! portfolio-wide, so requests cannot each reuse a fictitious copy of surviving
//! shared capacity. The report retains each scenario's Pareto frontier; it does
//! not invent scenario probabilities or collapse unlike shortage axes.
//!
//! This is an effects-disabled reference model. Caller-supplied IDs and digests
//! are not authenticated by this module. It does not reserve, order, dispatch,
//! contact suppliers, change inventory, or authorize payment.

use std::collections::BTreeSet;

use super::joint_capacity::{
    JointAllocationError, JointAllocationInput, validate_joint_allocation,
};
use super::joint_optimizer::{
    JointOptimizationError, JointOptimizationProblem, JointOptimizationReport,
    optimize_joint_allocation,
};

const MAX_JOINT_SCENARIOS: usize = 32;
const MAX_TOTAL_SEARCH_NODES: u64 = 2_000_000;
const EVALUATOR_PROFILE: &str = "mycelix:joint-disruption-stress-v1";

/// A single, caller-declared common-cause scenario.
///
/// The manifest digest must bind the frozen base problem plus this scenario's
/// canonical content. The evaluator records the value but cannot recompute or
/// authenticate a caller-provided digest.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointDisruptionScenario {
    pub scenario_id: String,
    pub scenario_manifest_digest: String,
    /// All offers from any listed supplier are excluded in this scenario.
    pub failed_supplier_ids: Vec<String>,
    /// All offers drawing from any listed shared pool are excluded in this scenario.
    pub failed_capacity_pool_ids: Vec<String>,
}

/// Frozen scenario catalogue for a multi-request procurement problem.
///
/// Scenarios are deliberately unweighted: list position or catalogue size does
/// not imply probability, frequency, likelihood, or expected loss.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointDisruptionStressProblem {
    pub scenario_set_id: String,
    /// Caller-supplied digest of the canonical, complete scenario set.
    pub scenario_set_digest: String,
    pub scenarios: Vec<JointDisruptionScenario>,
    /// Aggregate search-node budget across every scenario evaluation.
    pub max_total_search_nodes: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointDisruptionError {
    InvalidScenarioSet,
    InvalidScenario,
    DuplicateScenarioId,
    DuplicateScenarioManifestDigest,
    DuplicateFailedSupplierId,
    DuplicateFailedCapacityPoolId,
    UnknownFailedSupplierId,
    UnknownFailedCapacityPoolId,
    UnsupportedMixedCurrency,
    InvalidBaseProblem(JointAllocationError),
    Optimization(JointOptimizationError),
    SearchSpaceExceeded { visited_nodes: u64, limit: u64 },
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointDisruptionScenarioReport {
    pub scenario_id: String,
    pub scenario_manifest_digest: String,
    pub failed_supplier_ids: Vec<String>,
    pub failed_capacity_pool_ids: Vec<String>,
    /// Offers removed specifically because a declared supplier/pool failed.
    pub excluded_offer_ids: Vec<String>,
    /// Remaining input offers; not a claim that each is currently available.
    pub remaining_offer_count: usize,
    /// Complete Pareto frontier for this scenario, if evaluation succeeds.
    pub optimization: JointOptimizationReport,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JointDisruptionStressReport {
    pub evaluator_profile: &'static str,
    pub base_manifest_digest: String,
    pub scenario_set_id: String,
    pub scenario_set_digest: String,
    pub scenario_count: usize,
    pub total_search_node_count: u64,
    /// Canonical scenario-ID order, independent of input catalogue order.
    pub scenario_results: Vec<JointDisruptionScenarioReport>,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn unique_non_empty(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .all(|value| non_empty(value) && seen.insert(value.as_str()))
}

/// Validate the original manifest before applying any failure. Without this
/// step, a scenario could accidentally hide malformed offers or mappings by
/// filtering them out before the underlying validator sees them.
fn validate_base_problem(
    base: &JointOptimizationProblem,
) -> Result<(), JointDisruptionError> {
    if base.requests.is_empty()
        || !non_empty(&base.input_manifest_digest)
        || base.search_node_limit == 0
    {
        return Err(JointDisruptionError::InvalidBaseProblem(
            JointAllocationError::InvalidManifest,
        ));
    }

    let currency = &base.requests[0].currency;
    if !non_empty(currency)
        || base.requests.iter().any(|request| request.currency != *currency)
        || base.offers.iter().any(|offer| offer.currency != *currency)
    {
        return Err(JointDisruptionError::UnsupportedMixedCurrency);
    }

    let input = JointAllocationInput {
        evaluated_at: base.evaluated_at,
        input_manifest_digest: base.input_manifest_digest.clone(),
        requests: base.requests.clone(),
        offers: base.offers.clone(),
        capacity_pools: base.capacity_pools.clone(),
        capability_mappings: base.capability_mappings.clone(),
        assignments: Vec::new(),
    };
    validate_joint_allocation(&input).map_err(JointDisruptionError::InvalidBaseProblem)
}

fn validate_scenario_set(
    base: &JointOptimizationProblem,
    set: &JointDisruptionStressProblem,
) -> Result<Vec<JointDisruptionScenario>, JointDisruptionError> {
    if !non_empty(&set.scenario_set_id)
        || !non_empty(&set.scenario_set_digest)
        || set.scenarios.is_empty()
        || set.scenarios.len() > MAX_JOINT_SCENARIOS
        || set.max_total_search_nodes == 0
        || set.max_total_search_nodes > MAX_TOTAL_SEARCH_NODES
    {
        return Err(JointDisruptionError::InvalidScenarioSet);
    }

    let known_suppliers: BTreeSet<&str> = base
        .offers
        .iter()
        .map(|offer| offer.supplier_id.as_str())
        .collect();
    let known_pools: BTreeSet<&str> = base
        .capacity_pools
        .iter()
        .map(|pool| pool.capacity_pool_id.as_str())
        .collect();
    let mut scenario_ids = BTreeSet::new();
    let mut manifest_digests = BTreeSet::new();
    let mut scenarios = set.scenarios.clone();

    for scenario in &mut scenarios {
        if !non_empty(&scenario.scenario_id)
            || !non_empty(&scenario.scenario_manifest_digest)
        {
            return Err(JointDisruptionError::InvalidScenario);
        }
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            return Err(JointDisruptionError::DuplicateScenarioId);
        }
        if !manifest_digests.insert(scenario.scenario_manifest_digest.as_str()) {
            return Err(JointDisruptionError::DuplicateScenarioManifestDigest);
        }
        if !unique_non_empty(&scenario.failed_supplier_ids) {
            let has_duplicate = scenario.failed_supplier_ids.iter().any(|value| {
                scenario
                    .failed_supplier_ids
                    .iter()
                    .filter(|candidate| *candidate == value)
                    .count()
                    > 1
            });
            return Err(if has_duplicate {
                JointDisruptionError::DuplicateFailedSupplierId
            } else {
                JointDisruptionError::InvalidScenario
            });
        }
        if !unique_non_empty(&scenario.failed_capacity_pool_ids) {
            let has_duplicate = scenario.failed_capacity_pool_ids.iter().any(|value| {
                scenario
                    .failed_capacity_pool_ids
                    .iter()
                    .filter(|candidate| *candidate == value)
                    .count()
                    > 1
            });
            return Err(if has_duplicate {
                JointDisruptionError::DuplicateFailedCapacityPoolId
            } else {
                JointDisruptionError::InvalidScenario
            });
        }
        if scenario
            .failed_supplier_ids
            .iter()
            .any(|supplier| !known_suppliers.contains(supplier.as_str()))
        {
            return Err(JointDisruptionError::UnknownFailedSupplierId);
        }
        if scenario
            .failed_capacity_pool_ids
            .iter()
            .any(|pool| !known_pools.contains(pool.as_str()))
        {
            return Err(JointDisruptionError::UnknownFailedCapacityPoolId);
        }

        scenario.failed_supplier_ids.sort();
        scenario.failed_capacity_pool_ids.sort();
    }

    scenarios.sort_by(|left, right| left.scenario_id.cmp(&right.scenario_id));
    Ok(scenarios)
}

/// Apply explicit supplier/pool failures to every request in a frozen problem.
///
/// The original problem is validated first; every scenario is then re-optimized
/// with all offers affected by a failed supplier or pool removed portfolio-wide.
/// The operation is deterministic and all-or-error: no partial report is
/// returned if any scenario exceeds a search budget or fails validation.
pub fn evaluate_joint_disruption_stress(
    base: &JointOptimizationProblem,
    set: &JointDisruptionStressProblem,
) -> Result<JointDisruptionStressReport, JointDisruptionError> {
    validate_base_problem(base)?;
    let scenarios = validate_scenario_set(base, set)?;
    let mut total_search_node_count = 0u64;
    let mut scenario_results = Vec::with_capacity(scenarios.len());

    for scenario in scenarios {
        let failed_suppliers: BTreeSet<&str> = scenario
            .failed_supplier_ids
            .iter()
            .map(String::as_str)
            .collect();
        let failed_pools: BTreeSet<&str> = scenario
            .failed_capacity_pool_ids
            .iter()
            .map(String::as_str)
            .collect();

        let mut excluded_offer_ids = Vec::new();
        let offers: Vec<_> = base
            .offers
            .iter()
            .filter(|offer| {
                let excluded = failed_suppliers.contains(offer.supplier_id.as_str())
                    || failed_pools.contains(offer.capacity_pool_id.as_str());
                if excluded {
                    excluded_offer_ids.push(offer.offer_id.clone());
                }
                !excluded
            })
            .cloned()
            .collect();
        excluded_offer_ids.sort();

        let surviving_offer_ids: BTreeSet<&str> =
            offers.iter().map(|offer| offer.offer_id.as_str()).collect();
        let capability_mappings = base
            .capability_mappings
            .iter()
            .filter(|mapping| surviving_offer_ids.contains(mapping.offer_id.as_str()))
            .cloned()
            .collect();

        let remaining_budget = set
            .max_total_search_nodes
            .checked_sub(total_search_node_count)
            .ok_or(JointDisruptionError::ArithmeticOverflow)?;
        if remaining_budget == 0 {
            return Err(JointDisruptionError::SearchSpaceExceeded {
                visited_nodes: total_search_node_count,
                limit: set.max_total_search_nodes,
            });
        }

        let scenario_search_limit = base.search_node_limit.min(remaining_budget);
        let scenario_problem = JointOptimizationProblem {
            evaluated_at: base.evaluated_at,
            input_manifest_digest: scenario.scenario_manifest_digest.clone(),
            requests: base.requests.clone(),
            offers,
            capacity_pools: base.capacity_pools.clone(),
            capability_mappings,
            search_node_limit: scenario_search_limit,
        };

        let optimization = match optimize_joint_allocation(&scenario_problem) {
            Ok(report) => report,
            Err(JointOptimizationError::SearchSpaceExceeded { visited_nodes, .. })
                if remaining_budget <= base.search_node_limit =>
            {
                let aggregate_visited = total_search_node_count
                    .checked_add(visited_nodes)
                    .ok_or(JointDisruptionError::ArithmeticOverflow)?;
                return Err(JointDisruptionError::SearchSpaceExceeded {
                    visited_nodes: aggregate_visited,
                    limit: set.max_total_search_nodes,
                });
            }
            Err(error) => return Err(JointDisruptionError::Optimization(error)),
        };

        total_search_node_count = total_search_node_count
            .checked_add(optimization.search_node_count)
            .ok_or(JointDisruptionError::ArithmeticOverflow)?;
        if total_search_node_count > set.max_total_search_nodes {
            return Err(JointDisruptionError::SearchSpaceExceeded {
                visited_nodes: total_search_node_count,
                limit: set.max_total_search_nodes,
            });
        }

        scenario_results.push(JointDisruptionScenarioReport {
            scenario_id: scenario.scenario_id,
            scenario_manifest_digest: scenario.scenario_manifest_digest,
            failed_supplier_ids: scenario.failed_supplier_ids,
            failed_capacity_pool_ids: scenario.failed_capacity_pool_ids,
            excluded_offer_ids,
            remaining_offer_count: optimization.offer_count,
            optimization,
        });
    }

    Ok(JointDisruptionStressReport {
        evaluator_profile: EVALUATOR_PROFILE,
        base_manifest_digest: base.input_manifest_digest.clone(),
        scenario_set_id: set.scenario_set_id.clone(),
        scenario_set_digest: set.scenario_set_digest.clone(),
        scenario_count: scenario_results.len(),
        total_search_node_count,
        scenario_results,
    })
}

#[cfg(test)]
mod tests {
    use super::{
        JointDisruptionError, JointDisruptionScenario, JointDisruptionStressProblem,
        evaluate_joint_disruption_stress,
    };
    use crate::joint_capacity::{CapabilityMapping, JointOffer, JointRequest, SharedCapacityPool};
    use crate::joint_optimizer::{JointOptimizationProblem, optimize_joint_allocation};
    use crate::{EvidenceOrigin, EvidenceStatus};

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

    fn offer(id: &str, supplier: &str, pool: &str, cost: u64) -> JointOffer {
        JointOffer {
            offer_id: id.to_owned(),
            supplier_id: supplier.to_owned(),
            source_uri: format!("https://example.invalid/{id}"),
            source_payload_digest: format!("sha256-{id}"),
            qualification_receipt_id: format!("qualification-{id}"),
            offered_capability_id: "repair-kit".to_owned(),
            offered_profile_digest: "repair-kit-profile-v1".to_owned(),
            unit: "kit".to_owned(),
            currency: "USD".to_owned(),
            capacity_pool_id: pool.to_owned(),
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

    fn pool(id: &str, capacity: u32) -> SharedCapacityPool {
        SharedCapacityPool {
            capacity_pool_id: id.to_owned(),
            unit: "kit".to_owned(),
            time_bucket: "week-44".to_owned(),
            capacity_quantity: capacity,
            snapshot_digest: format!("sha256-pool-{id}"),
            evidence_status: EvidenceStatus::Verified,
            evidence_origin: EvidenceOrigin::LiveDomain,
            observed_at: 10,
            valid_until: 1_000,
        }
    }

    fn mapping(request_id: &str, offer_id: &str) -> CapabilityMapping {
        CapabilityMapping {
            mapping_id: format!("map-{request_id}-{offer_id}"),
            mapping_digest: format!("sha256-map-{request_id}-{offer_id}"),
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

    fn problem() -> JointOptimizationProblem {
        let requests = vec![request("urgent"), request("routine")];
        let offers = vec![
            offer("offer-a", "supplier-a", "pool-shared", 5),
            offer("offer-b", "supplier-b", "pool-shared", 8),
            offer("offer-c", "supplier-c", "pool-backup", 12),
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
            input_manifest_digest: "base-manifest-v1".to_owned(),
            requests,
            offers,
            capacity_pools: vec![pool("pool-shared", 2), pool("pool-backup", 1)],
            capability_mappings: mappings,
            search_node_limit: 100_000,
        }
    }

    fn scenario(
        id: &str,
        digest: &str,
        suppliers: &[&str],
        pools: &[&str],
    ) -> JointDisruptionScenario {
        JointDisruptionScenario {
            scenario_id: id.to_owned(),
            scenario_manifest_digest: digest.to_owned(),
            failed_supplier_ids: suppliers.iter().map(|value| (*value).to_owned()).collect(),
            failed_capacity_pool_ids: pools.iter().map(|value| (*value).to_owned()).collect(),
        }
    }

    fn set(scenarios: Vec<JointDisruptionScenario>) -> JointDisruptionStressProblem {
        JointDisruptionStressProblem {
            scenario_set_id: "stress-set-v1".to_owned(),
            scenario_set_digest: "stress-set-digest-v1".to_owned(),
            scenarios,
            max_total_search_nodes: 1_000_000,
        }
    }

    #[test]
    fn supplier_failure_excludes_every_offer_from_that_supplier() {
        let report = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario("supplier-a-down", "scenario-a", &["supplier-a"], &[])]),
        )
        .expect("valid scenario");
        let result = &report.scenario_results[0];
        assert_eq!(result.excluded_offer_ids, vec!["offer-a".to_owned()]);
        assert_eq!(result.remaining_offer_count, 2);
        assert!(
            result
                .optimization
                .frontier
                .iter()
                .flat_map(|candidate| candidate.assignments.iter())
                .all(|assignment| assignment.offer_id != "offer-a")
        );
    }

    #[test]
    fn shared_pool_failure_excludes_every_offer_drawing_from_that_pool() {
        let report = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario("pool-down", "scenario-pool", &[], &["pool-shared"])]),
        )
        .expect("valid scenario");
        let result = &report.scenario_results[0];
        assert_eq!(
            result.excluded_offer_ids,
            vec!["offer-a".to_owned(), "offer-b".to_owned()]
        );
        assert_eq!(result.remaining_offer_count, 1);
        assert!(
            result
                .optimization
                .frontier
                .iter()
                .flat_map(|candidate| candidate.assignments.iter())
                .all(|assignment| assignment.offer_id == "offer-c")
        );
    }

    #[test]
    fn unknown_failure_domain_fails_closed() {
        let error = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario("typo", "scenario-typo", &["supplier-missing"], &[])]),
        )
        .expect_err("unknown supplier must fail closed");
        assert_eq!(error, JointDisruptionError::UnknownFailedSupplierId);
    }

    #[test]
    fn unknown_capacity_pool_fails_closed() {
        let error = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario("typo", "scenario-typo", &[], &["pool-missing"])]),
        )
        .expect_err("unknown pool must fail closed");
        assert_eq!(error, JointDisruptionError::UnknownFailedCapacityPoolId);
    }

    #[test]
    fn duplicate_scenario_ids_fail_closed() {
        let error = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![
                scenario("same", "digest-a", &[], &[]),
                scenario("same", "digest-b", &["supplier-a"], &[]),
            ]),
        )
        .expect_err("duplicate scenario ID must fail closed");
        assert_eq!(error, JointDisruptionError::DuplicateScenarioId);
    }

    #[test]
    fn duplicate_failed_supplier_ids_fail_closed() {
        let error = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario(
                "duplicate-failure",
                "scenario-d",
                &["supplier-a", "supplier-a"],
                &[],
            )]),
        )
        .expect_err("duplicate failure IDs must fail closed");
        assert_eq!(error, JointDisruptionError::DuplicateFailedSupplierId);
    }

    #[test]
    fn scenario_results_are_in_canonical_id_order() {
        let report = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![
                scenario("z-supplier-down", "scenario-z", &["supplier-a"], &[]),
                scenario("a-control", "scenario-a", &[], &[]),
            ]),
        )
        .expect("valid scenarios");
        assert_eq!(
            report
                .scenario_results
                .iter()
                .map(|result| result.scenario_id.as_str())
                .collect::<Vec<_>>(),
            vec!["a-control", "z-supplier-down"]
        );
    }

    #[test]
    fn failure_domain_input_order_does_not_change_report() {
        let left = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario(
                "two-outages",
                "scenario-one",
                &["supplier-c", "supplier-a"],
                &["pool-shared"],
            )]),
        )
        .expect("valid left");
        let right = evaluate_joint_disruption_stress(
            &problem(),
            &set(vec![scenario(
                "two-outages",
                "scenario-one",
                &["supplier-a", "supplier-c"],
                &["pool-shared"],
            )]),
        )
        .expect("valid right");
        assert_eq!(left, right);
    }

    #[test]
    fn aggregate_budget_exhaustion_returns_no_partial_frontier() {
        let base = problem();
        let control_nodes = optimize_joint_allocation(&base)
            .expect("baseline problem is valid")
            .search_node_count;
        let mut stress = set(vec![
            scenario("a", "scenario-a", &[], &[]),
            scenario("b", "scenario-b", &["supplier-a"], &[]),
        ]);
        // Permit the first full scenario and only one node for the second.
        stress.max_total_search_nodes = control_nodes + 1;
        let error = evaluate_joint_disruption_stress(&base, &stress)
            .expect_err("budget exhaustion after a complete scenario must discard partial output");
        assert!(matches!(
            error,
            JointDisruptionError::SearchSpaceExceeded { .. }
        ));
    }

    #[test]
    fn empty_scenario_catalogue_is_rejected() {
        let error = evaluate_joint_disruption_stress(&problem(), &set(Vec::new()))
            .expect_err("empty scenario catalogue is not evidence of resilience");
        assert_eq!(error, JointDisruptionError::InvalidScenarioSet);
    }
}
