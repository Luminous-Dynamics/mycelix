// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use praxis_core::evidence_efficiency::{
    CyberDomain, EVIDENCE_EFFICIENCY_BENCHMARK_ID, EvidenceRequestCatalog, EvidenceSourceKind,
    RequestAuthority, validate_request_catalog,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashSet};

const CATALOG_JSON: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v2/request-catalog.json");
const EVALUATOR_JSON: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v2/evaluator-only.json");
const CATALOG_SCHEMA: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v2/request-catalog.schema.json");
const EVALUATOR_SCHEMA: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v2/evaluator-only.schema.json");
const CATALOG_V1_JSON: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v1/request-catalog.json");
const EVALUATOR_V1_JSON: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v1/evaluator-only.json");
const CATALOG_V1_SCHEMA: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v1/request-catalog.schema.json");
const EVALUATOR_V1_SCHEMA: &str =
    include_str!("../../../examples/cyber-evidence-efficiency/v1/evaluator-only.schema.json");
const PREREG_TEMPLATE_JSON: &str = include_str!(
    "../../../examples/cyber-evidence-efficiency/v2/qualification-protocol.template.json"
);
const PREREG_SCHEMA_JSON: &str = include_str!(
    "../../../examples/cyber-evidence-efficiency/v2/qualification-protocol.schema.json"
);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum EvidenceQuality {
    Current,
    Stale,
    Conflicting,
    Unavailable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum EvidenceSufficiency {
    FullSupport,
    PartialSupport,
    Irrelevant,
    Absent,
    Conflicting,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorCorpus {
    schema_version: u16,
    benchmark_id: String,
    benchmark_version: String,
    disclosure: String,
    scenarios: Vec<EvaluatorScenario>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorScenario {
    scenario_id: String,
    world_hypothesis_id: String,
    request_outcomes: Vec<RequestOutcome>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RequestOutcome {
    request_id: String,
    quality: EvidenceQuality,
    sufficiency: EvidenceSufficiency,
    labels_by_hypothesis: Vec<HypothesisOutcome>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct HypothesisOutcome {
    hypothesis_id: String,
    outcome_label: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RequestDisposition {
    Executed,
    BlockedUnauthorized,
    BlockedBudget,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RequestResult {
    request_id: String,
    disposition: RequestDisposition,
    quality: Option<EvidenceQuality>,
    sufficiency: Option<EvidenceSufficiency>,
    expected_information_gain_milli_bits: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct EvaluationBudget {
    max_cost_units: u32,
    max_latency_bound_minutes: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct EfficiencyReport {
    requests_attempted: usize,
    requests_executed: usize,
    blocked_unauthorized: usize,
    blocked_by_budget: usize,
    total_cost_units: u32,
    latency_bound_minutes: u32,
    expected_information_gain_milli_bits: u32,
    realized_information_gain_milli_bits: u32,
    zero_gain_current_requests: usize,
    full_support_responses: usize,
    partial_support_responses: usize,
    irrelevant_responses: usize,
    absent_responses: usize,
    conflicting_sufficiency_responses: usize,
    stale_responses: usize,
    conflicting_responses: usize,
    unavailable_responses: usize,
    remaining_hypotheses: Vec<String>,
    request_results: Vec<RequestResult>,
}

fn catalog_fixture() -> EvidenceRequestCatalog {
    serde_json::from_str(CATALOG_JSON).expect("request catalog parses")
}

fn evaluator_fixture() -> EvaluatorCorpus {
    serde_json::from_str(EVALUATOR_JSON).expect("evaluator fixture parses")
}

/// Semantic cross-file validation is test/qualification-only. The evaluator fixture
/// is never included by the production library module.
fn validate_evaluator_pair(
    catalog: &EvidenceRequestCatalog,
    evaluator: &EvaluatorCorpus,
) -> Result<(), Vec<String>> {
    let mut issues = Vec::new();
    if evaluator.schema_version != catalog.schema_version {
        issues.push("schema version mismatch".to_string());
    }
    if evaluator.benchmark_id != catalog.benchmark_id {
        issues.push("benchmark ID mismatch".to_string());
    }
    if evaluator.benchmark_version != catalog.benchmark_version {
        issues.push("benchmark version mismatch".to_string());
    }
    if evaluator.disclosure.trim().is_empty() {
        issues.push("evaluator disclosure is required".to_string());
    }

    let catalog_ids = catalog
        .scenarios
        .iter()
        .map(|scenario| scenario.scenario_id.as_str())
        .collect::<BTreeSet<_>>();
    let evaluator_ids = evaluator
        .scenarios
        .iter()
        .map(|scenario| scenario.scenario_id.as_str())
        .collect::<BTreeSet<_>>();
    if catalog_ids.len() != catalog.scenarios.len() {
        issues.push("duplicate catalog scenario IDs".to_string());
    }
    if evaluator_ids.len() != evaluator.scenarios.len() {
        issues.push("duplicate evaluator scenario IDs".to_string());
    }
    if catalog_ids != evaluator_ids {
        issues.push("catalog/evaluator scenario IDs do not match exactly".to_string());
    }

    for evaluator_scenario in &evaluator.scenarios {
        let Some(scenario) = catalog
            .scenarios
            .iter()
            .find(|scenario| scenario.scenario_id == evaluator_scenario.scenario_id)
        else {
            issues.push(format!(
                "{}: evaluator scenario has no catalog scenario",
                evaluator_scenario.scenario_id
            ));
            continue;
        };
        if !scenario
            .hypotheses
            .iter()
            .any(|hypothesis| hypothesis == &evaluator_scenario.world_hypothesis_id)
        {
            issues.push(format!(
                "{}: world hypothesis is not in the solver-visible hypothesis set",
                evaluator_scenario.scenario_id
            ));
        }

        let request_ids = scenario
            .requests
            .iter()
            .map(|request| request.request_id.as_str())
            .collect::<BTreeSet<_>>();
        let outcome_ids = evaluator_scenario
            .request_outcomes
            .iter()
            .map(|outcome| outcome.request_id.as_str())
            .collect::<BTreeSet<_>>();
        if outcome_ids.len() != evaluator_scenario.request_outcomes.len() {
            issues.push(format!(
                "{}: duplicate evaluator request outcome",
                evaluator_scenario.scenario_id
            ));
        }
        if request_ids != outcome_ids {
            issues.push(format!(
                "{}: evaluator outcomes must exactly cover catalog request IDs",
                evaluator_scenario.scenario_id
            ));
        }

        let hypotheses = scenario
            .hypotheses
            .iter()
            .map(String::as_str)
            .collect::<BTreeSet<_>>();
        for outcome in &evaluator_scenario.request_outcomes {
            let label_ids = outcome
                .labels_by_hypothesis
                .iter()
                .map(|label| label.hypothesis_id.as_str())
                .collect::<BTreeSet<_>>();
            if label_ids.len() != outcome.labels_by_hypothesis.len() {
                issues.push(format!(
                    "{} / {}: duplicate hypothesis outcome mapping",
                    evaluator_scenario.scenario_id, outcome.request_id
                ));
            }
            if label_ids != hypotheses {
                issues.push(format!(
                    "{} / {}: hypothesis outcome map must cover every catalog hypothesis exactly once",
                    evaluator_scenario.scenario_id, outcome.request_id
                ));
            }
            if outcome.labels_by_hypothesis.iter().any(|label| {
                label.hypothesis_id.trim().is_empty() || label.outcome_label.trim().is_empty()
            }) {
                issues.push(format!(
                    "{} / {}: hypothesis and outcome labels must not be empty",
                    evaluator_scenario.scenario_id, outcome.request_id
                ));
            }
        }
    }

    if issues.is_empty() {
        Ok(())
    } else {
        Err(issues)
    }
}

/// Prior-expected information gained from a fixed set of usable requests,
/// computed as the entropy of joint outcome signatures under a uniform
/// hypothesis prior. Unlike realized reduction, this does not depend on which
/// synthetic world happened to be selected for this fixture.
fn joint_information_gain_milli_bits(hypotheses: &[String], outcomes: &[&RequestOutcome]) -> u32 {
    if hypotheses.len() < 2 || outcomes.is_empty() {
        return 0;
    }

    let hypothesis_count = hypotheses.len() as f64;
    let mut partitions = BTreeMap::<Vec<String>, usize>::new();
    for hypothesis in hypotheses {
        let mut signature = Vec::with_capacity(outcomes.len());
        for outcome in outcomes {
            let label = outcome
                .labels_by_hypothesis
                .iter()
                .find(|entry| entry.hypothesis_id == *hypothesis)
                .expect("semantic validation maps every hypothesis exactly once");
            signature.push(label.outcome_label.clone());
        }
        *partitions.entry(signature).or_default() += 1;
    }

    let posterior_entropy = partitions
        .values()
        .map(|size| {
            let probability = *size as f64 / hypothesis_count;
            probability * (*size as f64).log2()
        })
        .sum::<f64>();
    ((hypothesis_count.log2() - posterior_entropy).max(0.0) * 1000.0).round() as u32
}

/// Deterministic synthetic reference evaluator for integration tests and qualification.
/// Uniform hypothesis priors are a benchmark convention, not empirical probabilities.
fn evaluate_policy(
    catalog: &EvidenceRequestCatalog,
    evaluator: &EvaluatorCorpus,
    scenario_id: &str,
    requested_ids: &[&str],
    authorized_authorities: &[RequestAuthority],
    budget: EvaluationBudget,
) -> Result<EfficiencyReport, String> {
    validate_request_catalog(catalog)
        .map_err(|issues| format!("invalid request catalog: {}", issues[0].message))?;
    validate_evaluator_pair(catalog, evaluator)
        .map_err(|issues| format!("invalid evaluator pair: {}", issues[0]))?;
    let scenario = catalog
        .scenarios
        .iter()
        .find(|scenario| scenario.scenario_id == scenario_id)
        .ok_or_else(|| format!("unknown scenario: {scenario_id}"))?;
    let evaluator_scenario = evaluator
        .scenarios
        .iter()
        .find(|scenario| scenario.scenario_id == scenario_id)
        .ok_or_else(|| format!("unknown evaluator scenario: {scenario_id}"))?;

    let mut remaining_hypotheses = scenario.hypotheses.clone();
    remaining_hypotheses.sort();
    let initial_hypotheses = remaining_hypotheses.len();
    let mut seen_requests = HashSet::new();
    let mut usable_outcomes: Vec<&RequestOutcome> = Vec::new();
    let mut report = EfficiencyReport {
        requests_attempted: requested_ids.len(),
        requests_executed: 0,
        blocked_unauthorized: 0,
        blocked_by_budget: 0,
        total_cost_units: 0,
        latency_bound_minutes: 0,
        expected_information_gain_milli_bits: 0,
        realized_information_gain_milli_bits: 0,
        zero_gain_current_requests: 0,
        full_support_responses: 0,
        partial_support_responses: 0,
        irrelevant_responses: 0,
        absent_responses: 0,
        conflicting_sufficiency_responses: 0,
        stale_responses: 0,
        conflicting_responses: 0,
        unavailable_responses: 0,
        remaining_hypotheses: Vec::new(),
        request_results: Vec::new(),
    };

    for requested_id in requested_ids {
        if !seen_requests.insert(*requested_id) {
            return Err(format!("duplicate request attempt: {requested_id}"));
        }
        let request = scenario
            .requests
            .iter()
            .find(|request| request.request_id == *requested_id)
            .ok_or_else(|| format!("unknown request ID: {requested_id}"))?;

        if !authorized_authorities.contains(&request.required_authority) {
            report.blocked_unauthorized += 1;
            report.request_results.push(RequestResult {
                request_id: request.request_id.clone(),
                disposition: RequestDisposition::BlockedUnauthorized,
                quality: None,
                sufficiency: None,
                expected_information_gain_milli_bits: 0,
            });
            continue;
        }
        if request.cost_units
            > budget
                .max_cost_units
                .saturating_sub(report.total_cost_units)
            || request.latency_bound_minutes
                > budget
                    .max_latency_bound_minutes
                    .saturating_sub(report.latency_bound_minutes)
        {
            report.blocked_by_budget += 1;
            report.request_results.push(RequestResult {
                request_id: request.request_id.clone(),
                disposition: RequestDisposition::BlockedBudget,
                quality: None,
                sufficiency: None,
                expected_information_gain_milli_bits: 0,
            });
            continue;
        }

        report.requests_executed += 1;
        report.total_cost_units += request.cost_units;
        report.latency_bound_minutes += request.latency_bound_minutes;
        let outcome = evaluator_scenario
            .request_outcomes
            .iter()
            .find(|outcome| outcome.request_id == request.request_id)
            .expect("pair validation guarantees each request outcome");
        match outcome.sufficiency {
            EvidenceSufficiency::FullSupport => report.full_support_responses += 1,
            EvidenceSufficiency::PartialSupport => report.partial_support_responses += 1,
            EvidenceSufficiency::Irrelevant => report.irrelevant_responses += 1,
            EvidenceSufficiency::Absent => report.absent_responses += 1,
            EvidenceSufficiency::Conflicting => report.conflicting_sufficiency_responses += 1,
        }

        let mut request_gain = 0;
        match outcome.quality {
            EvidenceQuality::Current => match outcome.sufficiency {
                EvidenceSufficiency::FullSupport | EvidenceSufficiency::PartialSupport => {
                    let information_before =
                        joint_information_gain_milli_bits(&scenario.hypotheses, &usable_outcomes);
                    usable_outcomes.push(outcome);
                    let information_after =
                        joint_information_gain_milli_bits(&scenario.hypotheses, &usable_outcomes);
                    request_gain = information_after.saturating_sub(information_before);
                    report.expected_information_gain_milli_bits += request_gain;
                    if request_gain == 0 {
                        report.zero_gain_current_requests += 1;
                    }
                    let labels = outcome
                        .labels_by_hypothesis
                        .iter()
                        .map(|item| (item.hypothesis_id.as_str(), item.outcome_label.as_str()))
                        .collect::<BTreeMap<_, _>>();
                    let observed_label = labels
                        .get(evaluator_scenario.world_hypothesis_id.as_str())
                        .expect("pair validation binds the world hypothesis");
                    let before = remaining_hypotheses.len();
                    remaining_hypotheses.retain(|hypothesis| {
                        labels.get(hypothesis.as_str()) == Some(observed_label)
                    });
                    let after = remaining_hypotheses.len();
                    if after > 0 && after < before {
                        report.realized_information_gain_milli_bits +=
                            ((before as f64 / after as f64).log2() * 1000.0).round() as u32;
                    }
                }
                EvidenceSufficiency::Irrelevant
                | EvidenceSufficiency::Absent
                | EvidenceSufficiency::Conflicting => {}
            },
            EvidenceQuality::Stale => report.stale_responses += 1,
            EvidenceQuality::Conflicting => report.conflicting_responses += 1,
            EvidenceQuality::Unavailable => report.unavailable_responses += 1,
        }
        report.request_results.push(RequestResult {
            request_id: request.request_id.clone(),
            disposition: RequestDisposition::Executed,
            quality: Some(outcome.quality),
            sufficiency: Some(outcome.sufficiency),
            expected_information_gain_milli_bits: request_gain,
        });
    }

    remaining_hypotheses.sort();
    report.remaining_hypotheses = remaining_hypotheses;
    debug_assert!(report.remaining_hypotheses.len() <= initial_hypotheses);
    Ok(report)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BaselinePolicy {
    NoQuery,
    FixedPlaybook,
    CheapestAuthorizedFirst,
}

/// Returns a policy trace using solver-visible catalog metadata only.
/// Evaluator outcomes/world state are intentionally not accepted as inputs.
fn baseline_trace(
    catalog: &EvidenceRequestCatalog,
    scenario_id: &str,
    authorized_authorities: &[RequestAuthority],
    policy: BaselinePolicy,
) -> Result<Vec<String>, String> {
    let scenario = catalog
        .scenarios
        .iter()
        .find(|scenario| scenario.scenario_id == scenario_id)
        .ok_or_else(|| format!("unknown scenario: {scenario_id}"))?;

    match policy {
        BaselinePolicy::NoQuery => Ok(Vec::new()),
        BaselinePolicy::FixedPlaybook => {
            // Frozen per-domain sequences are metadata-only policies, not oracle traces.
            let playbook: &[&str] = match scenario.domain {
                CyberDomain::IdentityAccess => &[
                    "shared-egress-rollup",
                    "endpoint-device-context",
                    "session-issuance-audit",
                ],
                CyberDomain::NetworkConfiguration => {
                    &["effective-route-table", "policy-deployment-receipt"]
                }
                CyberDomain::SupplyChainProvenance => {
                    &["registry-digest-comparison", "build-provenance-attestation"]
                }
                CyberDomain::AiToolBoundaries => {
                    &["retrieved-content-lineage", "tool-invocation-audit"]
                }
                CyberDomain::EndpointResponseTelemetry => {
                    &["endpoint-action-receipt", "endpoint-sensor-health"]
                }
            };
            let available = scenario
                .requests
                .iter()
                .filter(|request| authorized_authorities.contains(&request.required_authority))
                .map(|request| request.request_id.as_str())
                .collect::<BTreeSet<_>>();
            Ok(playbook
                .iter()
                .filter(|request_id| available.contains(**request_id))
                .map(|request_id| (*request_id).to_string())
                .collect())
        }
        BaselinePolicy::CheapestAuthorizedFirst => {
            let mut requests = scenario
                .requests
                .iter()
                .filter(|request| authorized_authorities.contains(&request.required_authority))
                .collect::<Vec<_>>();
            requests.sort_by(|left, right| {
                left.cost_units
                    .cmp(&right.cost_units)
                    .then_with(|| left.latency_bound_minutes.cmp(&right.latency_bound_minutes))
                    .then_with(|| left.request_id.cmp(&right.request_id))
            });
            Ok(requests
                .into_iter()
                .map(|request| request.request_id.clone())
                .collect())
        }
    }
}

/// Exact non-adaptive oracle upper bound under the uniform synthetic prior.
/// It sees evaluator partitions unavailable to solvers; this is not a fair
/// baseline. It exhaustively chooses the authorized request subset maximizing
/// expected hypothesis information under the declared cost/latency budget.
fn oracle_information_gain_per_cost_trace(
    catalog: &EvidenceRequestCatalog,
    evaluator: &EvaluatorCorpus,
    scenario_id: &str,
    authorized_authorities: &[RequestAuthority],
    budget: EvaluationBudget,
) -> Result<Vec<String>, String> {
    validate_request_catalog(catalog)
        .map_err(|issues| format!("invalid request catalog: {}", issues[0].message))?;
    validate_evaluator_pair(catalog, evaluator)
        .map_err(|issues| format!("invalid evaluator pair: {}", issues[0]))?;
    let scenario = catalog
        .scenarios
        .iter()
        .find(|scenario| scenario.scenario_id == scenario_id)
        .ok_or_else(|| format!("unknown scenario: {scenario_id}"))?;
    let evaluator_scenario = evaluator
        .scenarios
        .iter()
        .find(|scenario| scenario.scenario_id == scenario_id)
        .ok_or_else(|| format!("unknown evaluator scenario: {scenario_id}"))?;

    let mut candidates = scenario
        .requests
        .iter()
        .filter(|request| authorized_authorities.contains(&request.required_authority))
        .filter(|request| {
            let outcome = evaluator_scenario
                .request_outcomes
                .iter()
                .find(|outcome| outcome.request_id == request.request_id)
                .expect("pair validation covers all request IDs");
            outcome.quality == EvidenceQuality::Current
                && matches!(
                    outcome.sufficiency,
                    EvidenceSufficiency::FullSupport | EvidenceSufficiency::PartialSupport
                )
                && request.cost_units <= budget.max_cost_units
                && request.latency_bound_minutes <= budget.max_latency_bound_minutes
        })
        .map(|request| request.request_id.as_str())
        .collect::<Vec<_>>();
    candidates.sort_unstable();
    if candidates.len() > 16 {
        return Err("exact oracle limited to 16 candidate requests per scenario".into());
    }

    let mut best: Option<(u32, u32, u32, Vec<String>)> = None;
    for mask in 0u64..(1u64 << candidates.len()) {
        let ids = candidates
            .iter()
            .enumerate()
            .filter(|(index, _)| mask & (1u64 << index) != 0)
            .map(|(_, id)| (*id).to_string())
            .collect::<Vec<_>>();
        let mut cost = 0u32;
        let mut latency = 0u32;
        for id in &ids {
            let request = scenario
                .requests
                .iter()
                .find(|request| request.request_id == *id)
                .expect("candidate is drawn from catalog");
            cost = cost.saturating_add(request.cost_units);
            latency = latency.saturating_add(request.latency_bound_minutes);
        }
        if cost > budget.max_cost_units || latency > budget.max_latency_bound_minutes {
            continue;
        }

        // Use the same joint-partition objective as the policy report.
        let selected_outcomes = ids
            .iter()
            .map(|id| {
                evaluator_scenario
                    .request_outcomes
                    .iter()
                    .find(|outcome| outcome.request_id == *id)
                    .expect("pair validation covers all request IDs")
            })
            .collect::<Vec<_>>();
        let gain = joint_information_gain_milli_bits(&scenario.hypotheses, &selected_outcomes);

        let better = match best.as_ref() {
            None => true,
            Some((best_gain, best_cost, best_latency, best_ids)) => {
                gain > *best_gain
                    || (gain == *best_gain && cost < *best_cost)
                    || (gain == *best_gain && cost == *best_cost && latency < *best_latency)
                    || (gain == *best_gain
                        && cost == *best_cost
                        && latency == *best_latency
                        && ids < *best_ids)
            }
        };
        if better {
            best = Some((gain, cost, latency, ids));
        }
    }

    Ok(best.map(|(_, _, _, ids)| ids).unwrap_or_default())
}

#[test]
fn v1_public_contract_fixtures_remain_valid_and_unchanged() {
    let catalog_schema: serde_json::Value = serde_json::from_str(CATALOG_V1_SCHEMA).unwrap();
    let evaluator_schema: serde_json::Value = serde_json::from_str(EVALUATOR_V1_SCHEMA).unwrap();
    let catalog: serde_json::Value = serde_json::from_str(CATALOG_V1_JSON).unwrap();
    let evaluator: serde_json::Value = serde_json::from_str(EVALUATOR_V1_JSON).unwrap();

    assert!(jsonschema::draft202012::meta::is_valid(&catalog_schema));
    assert!(jsonschema::draft202012::meta::is_valid(&evaluator_schema));
    assert!(
        jsonschema::draft202012::new(&catalog_schema)
            .unwrap()
            .is_valid(&catalog)
    );
    assert!(
        jsonschema::draft202012::new(&evaluator_schema)
            .unwrap()
            .is_valid(&evaluator)
    );
    assert_eq!(catalog["schema_version"], 1);
    assert_eq!(catalog["benchmark_version"], "0.1.0");
    assert_eq!(evaluator["schema_version"], 1);
    assert_eq!(evaluator["benchmark_version"], "0.1.0");
}

#[test]
fn v2_schemas_are_draft_2020_12_and_accept_five_domain_fixtures() {
    let catalog_schema: serde_json::Value =
        serde_json::from_str(CATALOG_SCHEMA).expect("catalog schema parses");
    let evaluator_schema: serde_json::Value =
        serde_json::from_str(EVALUATOR_SCHEMA).expect("evaluator schema parses");
    assert!(jsonschema::draft202012::meta::is_valid(&catalog_schema));
    assert!(jsonschema::draft202012::meta::is_valid(&evaluator_schema));

    let catalog_validator = jsonschema::draft202012::new(&catalog_schema).unwrap();
    let evaluator_validator = jsonschema::draft202012::new(&evaluator_schema).unwrap();
    let catalog_json: serde_json::Value = serde_json::from_str(CATALOG_JSON).unwrap();
    let evaluator_json: serde_json::Value = serde_json::from_str(EVALUATOR_JSON).unwrap();
    assert!(catalog_validator.is_valid(&catalog_json));
    assert!(evaluator_validator.is_valid(&evaluator_json));
}

#[test]
fn valid_catalog_and_evaluator_have_complete_cross_file_mappings() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    assert_eq!(catalog.benchmark_id, EVIDENCE_EFFICIENCY_BENCHMARK_ID);
    assert_eq!(catalog.scenarios[0].domain, CyberDomain::IdentityAccess);
    assert_eq!(catalog.schema_version, 2);
    assert_eq!(catalog.benchmark_version, "0.2.0");
    validate_request_catalog(&catalog).unwrap();
    validate_evaluator_pair(&catalog, &evaluator).unwrap();
    assert_eq!(catalog.scenarios.len(), 5);
    assert_eq!(catalog.scenarios[0].hypotheses.len(), 4);
    assert_eq!(catalog.scenarios[0].requests.len(), 5);
    let domains = catalog
        .scenarios
        .iter()
        .map(|scenario| {
            serde_json::to_value(scenario.domain)
                .unwrap()
                .as_str()
                .unwrap()
                .to_string()
        })
        .collect::<BTreeSet<_>>();
    assert_eq!(
        domains,
        BTreeSet::from([
            "identity_access".to_string(),
            "network_configuration".to_string(),
            "supply_chain_provenance".to_string(),
            "ai_tool_boundaries".to_string(),
            "endpoint_response_telemetry".to_string(),
        ])
    );
    let sufficiency = evaluator
        .scenarios
        .iter()
        .flat_map(|scenario| scenario.request_outcomes.iter())
        .map(|outcome| outcome.sufficiency)
        .collect::<BTreeSet<_>>();
    assert_eq!(sufficiency.len(), 5);
    assert!(sufficiency.contains(&EvidenceSufficiency::FullSupport));
    assert!(sufficiency.contains(&EvidenceSufficiency::PartialSupport));
    assert!(sufficiency.contains(&EvidenceSufficiency::Irrelevant));
    assert!(sufficiency.contains(&EvidenceSufficiency::Absent));
    assert!(sufficiency.contains(&EvidenceSufficiency::Conflicting));
}

#[test]
fn one_request_preserves_ambiguity_across_all_five_domains() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let budget = EvaluationBudget {
        max_cost_units: 10,
        max_latency_bound_minutes: 20,
    };
    let cases = [
        ("CYB-ACQ-001", "session-issuance-audit"),
        ("CYB-ACQ-002", "effective-route-table"),
        ("CYB-ACQ-003", "registry-digest-comparison"),
        ("CYB-ACQ-004", "retrieved-content-lineage"),
        ("CYB-ACQ-005", "endpoint-action-receipt"),
    ];
    for (scenario_id, request_id) in cases {
        let report = evaluate_policy(
            &catalog,
            &evaluator,
            scenario_id,
            &[request_id],
            &[RequestAuthority::ReadOnlyTelemetry],
            budget,
        )
        .unwrap();
        assert!(
            report.remaining_hypotheses.len() >= 2,
            "{scenario_id}: one request must not make the hidden world trivially certain"
        );
    }
}

#[test]
fn two_step_evidence_paths_resolve_each_domain_only_with_sufficient_current_evidence() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let readonly = [RequestAuthority::ReadOnlyTelemetry];
    let identity_authorities = [
        RequestAuthority::ReadOnlyTelemetry,
        RequestAuthority::IncidentCommander,
    ];
    let budget = EvaluationBudget {
        max_cost_units: 10,
        max_latency_bound_minutes: 20,
    };
    let cases = [
        (
            "CYB-ACQ-001",
            "session-issuance-audit",
            "privileged-token-use-audit",
            "compromised_session",
        ),
        (
            "CYB-ACQ-002",
            "effective-route-table",
            "policy-deployment-receipt",
            "route_scope_leakage",
        ),
        (
            "CYB-ACQ-003",
            "registry-digest-comparison",
            "build-provenance-attestation",
            "malicious_substitution",
        ),
        (
            "CYB-ACQ-004",
            "retrieved-content-lineage",
            "tool-invocation-audit",
            "prompt_injection_blocked",
        ),
        (
            "CYB-ACQ-005",
            "endpoint-action-receipt",
            "endpoint-sensor-health",
            "quarantine_requested_no_effect",
        ),
    ];
    for (scenario_id, first, second, expected_world) in cases {
        let authorities = if scenario_id == "CYB-ACQ-001" {
            &identity_authorities[..]
        } else {
            &readonly[..]
        };
        let report = evaluate_policy(
            &catalog,
            &evaluator,
            scenario_id,
            &[first, second],
            authorities,
            budget,
        )
        .unwrap();
        assert_eq!(
            report.remaining_hypotheses,
            vec![expected_world.to_string()],
            "{scenario_id}: expected a two-step resolution from the declared synthetic observations"
        );
        assert_eq!(report.requests_executed, 2);
    }
}

#[test]
fn irrelevant_absent_and_conflicting_sufficiency_never_narrow_hypotheses() {
    for sufficiency in [
        EvidenceSufficiency::Irrelevant,
        EvidenceSufficiency::Absent,
        EvidenceSufficiency::Conflicting,
    ] {
        let catalog = catalog_fixture();
        let mut evaluator = evaluator_fixture();
        evaluator.scenarios[0]
            .request_outcomes
            .iter_mut()
            .find(|outcome| outcome.request_id == "session-issuance-audit")
            .unwrap()
            .sufficiency = sufficiency;
        let report = evaluate_policy(
            &catalog,
            &evaluator,
            "CYB-ACQ-001",
            &["session-issuance-audit"],
            &[RequestAuthority::ReadOnlyTelemetry],
            EvaluationBudget {
                max_cost_units: 5,
                max_latency_bound_minutes: 10,
            },
        )
        .unwrap();
        assert_eq!(report.remaining_hypotheses.len(), 4);
        assert_eq!(report.expected_information_gain_milli_bits, 0);
        match sufficiency {
            EvidenceSufficiency::Irrelevant => assert_eq!(report.irrelevant_responses, 1),
            EvidenceSufficiency::Absent => assert_eq!(report.absent_responses, 1),
            EvidenceSufficiency::Conflicting => {
                assert_eq!(report.conflicting_sufficiency_responses, 1)
            }
            EvidenceSufficiency::FullSupport | EvidenceSufficiency::PartialSupport => {
                unreachable!()
            }
        }
    }
}

#[test]
fn current_partial_but_non_discriminating_evidence_has_zero_gain() {
    let catalog = catalog_fixture();
    let mut evaluator = evaluator_fixture();
    evaluator.scenarios[0]
        .request_outcomes
        .iter_mut()
        .find(|outcome| outcome.request_id == "shared-egress-rollup")
        .unwrap()
        .sufficiency = EvidenceSufficiency::PartialSupport;
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["shared-egress-rollup"],
        &[RequestAuthority::ReadOnlyTelemetry],
        EvaluationBudget {
            max_cost_units: 5,
            max_latency_bound_minutes: 10,
        },
    )
    .unwrap();
    assert_eq!(report.remaining_hypotheses.len(), 4);
    assert_eq!(report.expected_information_gain_milli_bits, 0);
    assert_eq!(report.zero_gain_current_requests, 1);
    assert_eq!(report.partial_support_responses, 1);
}

#[test]
fn schemas_reject_extra_fields_and_zero_cost_gaming() {
    let catalog_schema: serde_json::Value = serde_json::from_str(CATALOG_SCHEMA).unwrap();
    let evaluator_schema: serde_json::Value = serde_json::from_str(EVALUATOR_SCHEMA).unwrap();
    let catalog_validator = jsonschema::draft202012::new(&catalog_schema).unwrap();
    let evaluator_validator = jsonschema::draft202012::new(&evaluator_schema).unwrap();
    let catalog: serde_json::Value = serde_json::from_str(CATALOG_JSON).unwrap();
    let evaluator: serde_json::Value = serde_json::from_str(EVALUATOR_JSON).unwrap();

    let mut extra_request_field = catalog.clone();
    extra_request_field["scenarios"][0]["requests"][0]["expected_answer"] =
        serde_json::Value::String("compromised".into());
    assert!(!catalog_validator.is_valid(&extra_request_field));

    let mut zero_cost = catalog;
    zero_cost["scenarios"][0]["requests"][0]["cost_units"] = serde_json::Value::from(0);
    assert!(!catalog_validator.is_valid(&zero_cost));

    let mut invalid_quality = evaluator.clone();
    invalid_quality["scenarios"][0]["request_outcomes"][0]["quality"] =
        serde_json::Value::String("confirmed_compromise".into());
    assert!(!evaluator_validator.is_valid(&invalid_quality));

    let mut invalid_sufficiency = evaluator;
    invalid_sufficiency["scenarios"][0]["request_outcomes"][0]["sufficiency"] =
        serde_json::Value::String("guaranteed_truth".into());
    assert!(!evaluator_validator.is_valid(&invalid_sufficiency));
}

#[test]
fn one_partial_request_must_preserve_material_ambiguity() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["session-issuance-audit"],
        &[RequestAuthority::ReadOnlyTelemetry],
        EvaluationBudget {
            max_cost_units: 5,
            max_latency_bound_minutes: 10,
        },
    )
    .unwrap();
    assert_eq!(report.requests_executed, 1);
    assert_eq!(report.total_cost_units, 2);
    assert_eq!(report.latency_bound_minutes, 3);
    assert_eq!(report.expected_information_gain_milli_bits, 1500);
    assert_eq!(report.realized_information_gain_milli_bits, 1000);
    assert_eq!(
        report.remaining_hypotheses,
        vec!["compromised_session", "stale_sensor_false_alarm"]
    );
    assert_eq!(report.partial_support_responses, 1);
}

#[test]
fn privileged_followup_resolves_partial_support_with_authority() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["session-issuance-audit", "privileged-token-use-audit"],
        &[
            RequestAuthority::ReadOnlyTelemetry,
            RequestAuthority::IncidentCommander,
        ],
        EvaluationBudget {
            max_cost_units: 6,
            max_latency_bound_minutes: 8,
        },
    )
    .unwrap();
    assert_eq!(report.expected_information_gain_milli_bits, 2000);
    assert_eq!(
        report.request_results[1].expected_information_gain_milli_bits,
        500
    );
    assert_eq!(report.realized_information_gain_milli_bits, 2000);
    assert_eq!(report.remaining_hypotheses, vec!["compromised_session"]);
    assert_eq!(report.full_support_responses, 1);
    assert_eq!(report.partial_support_responses, 1);
    assert_eq!(report.total_cost_units, 6);
    assert_eq!(report.latency_bound_minutes, 8);
}

#[test]
fn stale_conflicting_and_unavailable_results_never_narrow_hypotheses() {
    for quality in [
        EvidenceQuality::Stale,
        EvidenceQuality::Conflicting,
        EvidenceQuality::Unavailable,
    ] {
        let catalog = catalog_fixture();
        let mut evaluator = evaluator_fixture();
        let stale = evaluator.scenarios[0]
            .request_outcomes
            .iter_mut()
            .find(|outcome| outcome.request_id == "sensor-health-check")
            .unwrap();
        stale.quality = quality;
        let report = evaluate_policy(
            &catalog,
            &evaluator,
            "CYB-ACQ-001",
            &["sensor-health-check"],
            &[RequestAuthority::ReadOnlyTelemetry],
            EvaluationBudget {
                max_cost_units: 5,
                max_latency_bound_minutes: 10,
            },
        )
        .unwrap();
        assert_eq!(report.remaining_hypotheses.len(), 4);
        assert_eq!(report.expected_information_gain_milli_bits, 0);
        assert_eq!(report.total_cost_units, 1);
        match quality {
            EvidenceQuality::Stale => assert_eq!(report.stale_responses, 1),
            EvidenceQuality::Conflicting => assert_eq!(report.conflicting_responses, 1),
            EvidenceQuality::Unavailable => assert_eq!(report.unavailable_responses, 1),
            EvidenceQuality::Current => unreachable!(),
        }
    }
}

#[test]
fn unauthorized_request_is_blocked_without_spending_budget_or_updating_evidence() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["privileged-token-use-audit", "session-issuance-audit"],
        &[RequestAuthority::ReadOnlyTelemetry],
        EvaluationBudget {
            max_cost_units: 5,
            max_latency_bound_minutes: 10,
        },
    )
    .unwrap();
    assert_eq!(report.blocked_unauthorized, 1);
    assert_eq!(
        report.request_results[0].disposition,
        RequestDisposition::BlockedUnauthorized
    );
    assert_eq!(report.total_cost_units, 2);
    assert_eq!(report.requests_executed, 1);
    assert_eq!(
        report.remaining_hypotheses,
        vec!["compromised_session", "stale_sensor_false_alarm"]
    );
}

#[test]
fn budget_blocks_expensive_requests_but_allows_a_later_feasible_request() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["privileged-token-use-audit", "session-issuance-audit"],
        &[
            RequestAuthority::IncidentCommander,
            RequestAuthority::ReadOnlyTelemetry,
        ],
        EvaluationBudget {
            max_cost_units: 2,
            max_latency_bound_minutes: 3,
        },
    )
    .unwrap();
    assert_eq!(report.blocked_by_budget, 1);
    assert_eq!(report.requests_executed, 1);
    assert_eq!(report.total_cost_units, 2);
    assert_eq!(report.latency_bound_minutes, 3);
    assert_eq!(report.remaining_hypotheses.len(), 2);
}

#[test]
fn redundant_current_evidence_has_zero_information_gain() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &["shared-egress-rollup"],
        &[RequestAuthority::ReadOnlyTelemetry],
        EvaluationBudget {
            max_cost_units: 5,
            max_latency_bound_minutes: 10,
        },
    )
    .unwrap();
    assert_eq!(report.expected_information_gain_milli_bits, 0);
    assert_eq!(report.zero_gain_current_requests, 0);
    assert_eq!(report.irrelevant_responses, 1);
    assert_eq!(report.remaining_hypotheses.len(), 4);
    assert_eq!(report.total_cost_units, 1);
}

#[test]
fn duplicate_and_unknown_request_attempts_are_rejected() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let budget = EvaluationBudget {
        max_cost_units: 10,
        max_latency_bound_minutes: 20,
    };
    assert!(
        evaluate_policy(
            &catalog,
            &evaluator,
            "CYB-ACQ-001",
            &["session-issuance-audit", "session-issuance-audit"],
            &[RequestAuthority::ReadOnlyTelemetry],
            budget,
        )
        .unwrap_err()
        .contains("duplicate request")
    );
    assert!(
        evaluate_policy(
            &catalog,
            &evaluator,
            "CYB-ACQ-001",
            &["not-a-request"],
            &[RequestAuthority::ReadOnlyTelemetry],
            budget,
        )
        .unwrap_err()
        .contains("unknown request ID")
    );
}

#[test]
fn semantic_validator_rejects_missing_hypothesis_outcome_and_unknown_world() {
    let catalog = catalog_fixture();
    let mut evaluator = evaluator_fixture();
    evaluator.scenarios[0].request_outcomes[0]
        .labels_by_hypothesis
        .pop();
    assert!(
        validate_evaluator_pair(&catalog, &evaluator)
            .unwrap_err()
            .iter()
            .any(|message| message.contains("must cover every catalog hypothesis"))
    );

    let mut evaluator = evaluator_fixture();
    evaluator.scenarios[0].world_hypothesis_id = "not-a-hypothesis".into();
    assert!(
        validate_evaluator_pair(&catalog, &evaluator)
            .unwrap_err()
            .iter()
            .any(|message| message.contains("world hypothesis is not"))
    );
}

#[test]
fn request_catalog_rejects_duplicate_request_ids_and_free_requests() {
    let mut catalog = catalog_fixture();
    let first_request_id = catalog.scenarios[0].requests[0].request_id.clone();
    catalog.scenarios[0].requests[1].request_id = first_request_id;
    catalog.scenarios[0].requests[2].cost_units = 0;
    let issues = validate_request_catalog(&catalog).unwrap_err();
    assert!(
        issues
            .iter()
            .any(|issue| issue.code == "duplicate_request_id")
    );
    assert!(issues.iter().any(|issue| issue.code == "zero_cost"));
}

#[test]
fn hidden_world_and_outcome_labels_are_absent_from_solver_projection() {
    let catalog: serde_json::Value = serde_json::from_str(CATALOG_JSON).unwrap();
    let rendered = catalog.to_string().to_ascii_lowercase();
    for secret in [
        "world_hypothesis_id",
        "labels_by_hypothesis",
        "expected_disposition",
    ] {
        assert!(!rendered.contains(secret), "solver catalog leaked {secret}");
    }
}

#[test]
fn equivalent_catalog_request_order_does_not_change_policy_result() {
    let mut catalog_a = catalog_fixture();
    let catalog_b = catalog_fixture();
    catalog_a.scenarios[0].requests.reverse();
    let evaluator = evaluator_fixture();
    let ids = ["session-issuance-audit", "endpoint-device-context"];
    let auth = [RequestAuthority::ReadOnlyTelemetry];
    let budget = EvaluationBudget {
        max_cost_units: 5,
        max_latency_bound_minutes: 10,
    };
    assert_eq!(
        evaluate_policy(&catalog_a, &evaluator, "CYB-ACQ-001", &ids, &auth, budget).unwrap(),
        evaluate_policy(&catalog_b, &evaluator, "CYB-ACQ-001", &ids, &auth, budget).unwrap()
    );
}

#[test]
fn source_kinds_and_authority_levels_are_typed_enums() {
    let catalog = catalog_fixture();
    assert_eq!(
        catalog.scenarios[0].requests[0].source_kind,
        EvidenceSourceKind::IdentitySessionAudit
    );
    assert_eq!(
        catalog.scenarios[0].requests[2].required_authority,
        RequestAuthority::IncidentCommander
    );
}

#[test]
fn oracle_information_gain_subset_is_a_privileged_non_adaptive_upper_bound() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let authorized = [RequestAuthority::ReadOnlyTelemetry];
    let budget = EvaluationBudget {
        max_cost_units: 5,
        max_latency_bound_minutes: 10,
    };
    let trace = oracle_information_gain_per_cost_trace(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &authorized,
        budget,
    )
    .unwrap();

    // The test oracle sees the hidden response partitions. Its score is an
    // upper bound, not a fair comparison with metadata-only policies.
    assert_eq!(trace, vec!["session-issuance-audit"]);
    let report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &trace.iter().map(String::as_str).collect::<Vec<_>>(),
        &authorized,
        budget,
    )
    .unwrap();
    assert_eq!(report.expected_information_gain_milli_bits, 1500);
    assert_eq!(report.total_cost_units, 2);
    assert_eq!(
        report.remaining_hypotheses,
        vec!["compromised_session", "stale_sensor_false_alarm"]
    );
}

#[test]
fn metadata_only_baselines_are_deterministic_and_use_equal_constraints() {
    let catalog = catalog_fixture();
    let evaluator = evaluator_fixture();
    let authorized = [RequestAuthority::ReadOnlyTelemetry];
    let budget = EvaluationBudget {
        max_cost_units: 5,
        max_latency_bound_minutes: 10,
    };

    let no_query = baseline_trace(
        &catalog,
        "CYB-ACQ-001",
        &authorized,
        BaselinePolicy::NoQuery,
    )
    .unwrap();
    assert!(no_query.is_empty());
    let no_query_report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &no_query.iter().map(String::as_str).collect::<Vec<_>>(),
        &authorized,
        budget,
    )
    .unwrap();
    assert_eq!(no_query_report.remaining_hypotheses.len(), 4);
    assert_eq!(no_query_report.expected_information_gain_milli_bits, 0);
    assert_eq!(no_query_report.total_cost_units, 0);

    let fixed = baseline_trace(
        &catalog,
        "CYB-ACQ-001",
        &authorized,
        BaselinePolicy::FixedPlaybook,
    )
    .unwrap();
    assert_eq!(
        fixed,
        vec![
            "shared-egress-rollup",
            "endpoint-device-context",
            "session-issuance-audit"
        ]
    );
    let fixed_report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &fixed.iter().map(String::as_str).collect::<Vec<_>>(),
        &authorized,
        budget,
    )
    .unwrap();
    assert_eq!(fixed_report.total_cost_units, 4);
    assert_eq!(fixed_report.latency_bound_minutes, 6);
    assert_eq!(fixed_report.expected_information_gain_milli_bits, 1500);
    assert_eq!(fixed_report.realized_information_gain_milli_bits, 1000);
    assert_eq!(
        fixed_report.remaining_hypotheses,
        vec!["compromised_session", "stale_sensor_false_alarm"]
    );

    let cheapest = baseline_trace(
        &catalog,
        "CYB-ACQ-001",
        &authorized,
        BaselinePolicy::CheapestAuthorizedFirst,
    )
    .unwrap();
    assert_eq!(
        cheapest,
        vec![
            "sensor-health-check",
            "shared-egress-rollup",
            "endpoint-device-context",
            "session-issuance-audit"
        ]
    );
    let cheapest_report = evaluate_policy(
        &catalog,
        &evaluator,
        "CYB-ACQ-001",
        &cheapest.iter().map(String::as_str).collect::<Vec<_>>(),
        &authorized,
        budget,
    )
    .unwrap();
    assert_eq!(cheapest_report.requests_executed, 4);
    assert_eq!(cheapest_report.total_cost_units, 5);
    assert_eq!(cheapest_report.latency_bound_minutes, 7);
    assert_eq!(cheapest_report.expected_information_gain_milli_bits, 1500);
    assert_eq!(cheapest_report.realized_information_gain_milli_bits, 1000);
    assert_eq!(
        cheapest_report.remaining_hypotheses,
        vec!["compromised_session", "stale_sensor_false_alarm"]
    );

    // Neither policy runner accepts evaluator state; a catalog permutation
    // must not change the metadata-only request order.
    let mut permuted = catalog.clone();
    permuted.scenarios[0].requests.reverse();
    assert_eq!(
        baseline_trace(
            &permuted,
            "CYB-ACQ-001",
            &authorized,
            BaselinePolicy::CheapestAuthorizedFirst
        )
        .unwrap(),
        cheapest
    );
}

#[test]
fn fixed_playbook_baseline_covers_all_five_domains_without_crossing_authority() {
    let catalog = catalog_fixture();
    let authorized = [RequestAuthority::ReadOnlyTelemetry];
    let cases = [
        ("CYB-ACQ-001", "session-issuance-audit"),
        ("CYB-ACQ-002", "policy-deployment-receipt"),
        ("CYB-ACQ-003", "build-provenance-attestation"),
        ("CYB-ACQ-004", "tool-invocation-audit"),
        ("CYB-ACQ-005", "endpoint-sensor-health"),
    ];
    for (scenario_id, expected_request) in cases {
        let trace = baseline_trace(
            &catalog,
            scenario_id,
            &authorized,
            BaselinePolicy::FixedPlaybook,
        )
        .unwrap();
        assert!(
            trace.iter().any(|id| id == expected_request),
            "{scenario_id}: fixed playbook must include its domain-relevant follow-up"
        );
        assert!(
            trace.len() >= 2,
            "{scenario_id}: fixed playbook must provide more than one investigative action"
        );
        let scenario = catalog
            .scenarios
            .iter()
            .find(|scenario| scenario.scenario_id == scenario_id)
            .unwrap();
        assert!(trace.iter().all(|id| {
            scenario.requests.iter().any(|request| {
                request.request_id == *id
                    && authorized.contains(&request.required_authority)
            })
        }));
    }
}

#[test]
fn metadata_only_baseline_never_selects_a_request_above_its_authority() {
    let catalog = catalog_fixture();
    let trace = baseline_trace(
        &catalog,
        "CYB-ACQ-001",
        &[RequestAuthority::ReadOnlyTelemetry],
        BaselinePolicy::CheapestAuthorizedFirst,
    )
    .unwrap();
    assert!(
        !trace
            .iter()
            .any(|request_id| request_id == "privileged-token-use-audit")
    );
}

#[test]
fn preregistration_template_is_schema_valid_but_not_eligible_to_report_a_score() {
    let schema: serde_json::Value =
        serde_json::from_str(PREREG_SCHEMA_JSON).expect("preregistration schema parses");
    let template: serde_json::Value =
        serde_json::from_str(PREREG_TEMPLATE_JSON).expect("preregistration template parses");
    assert!(jsonschema::draft202012::meta::is_valid(&schema));
    let validator = jsonschema::draft202012::new(&schema).expect("schema compiles");
    assert!(validator.is_valid(&template));
    assert_eq!(template["status"], "template_only_not_preregistered");
    assert_eq!(
        template["eligibility"],
        "NOT_ELIGIBLE_FOR_COMPARATIVE_SCORE"
    );
    assert_eq!(template["minimum_held_out_cases"], 50);
    assert_eq!(template["data_split"]["public_conformance_excluded"], true);
    assert_eq!(template["safety_gates"]["unauthorized_actions_accepted"], 0);
    assert_eq!(
        template["safety_gates"]["aggregate_score_may_compensate_for_safety_failure"],
        false
    );

    let mut falsely_registered = template.clone();
    falsely_registered["status"] = serde_json::Value::String("registered".into());
    assert!(
        !validator.is_valid(&falsely_registered),
        "a template cannot self-promote to a registered protocol"
    );
    let mut undersized = template.clone();
    undersized["minimum_held_out_cases"] = serde_json::Value::from(12);
    assert!(
        !validator.is_valid(&undersized),
        "the frozen protocol must preserve the minimum held-out case count"
    );
    let mut unsafe_gate = template;
    unsafe_gate["safety_gates"]["unauthorized_actions_accepted"] = serde_json::Value::from(1);
    assert!(
        !validator.is_valid(&unsafe_gate),
        "the preregistration schema must reject a non-zero safety acceptance target"
    );
}

#[test]
fn preregistration_policy_arms_label_the_evaluator_oracle_as_non_competitive() {
    let template: serde_json::Value =
        serde_json::from_str(PREREG_TEMPLATE_JSON).expect("preregistration template parses");
    let arms = template["policy_arms"]
        .as_array()
        .expect("policy arms array");
    let oracle = arms
        .iter()
        .find(|arm| arm["id"] == "non_adaptive_oracle_upper_bound")
        .expect("oracle arm exists");
    assert_eq!(oracle["uses_evaluator_only_data"], true);
    assert_eq!(oracle["comparative_competitor"], false);
    assert!(
        arms.iter()
            .any(|arm| arm["id"] == "human_analyst_reference")
    );
    assert!(arms.iter().any(|arm| arm["id"] == "ai_tool_policy"));
}
