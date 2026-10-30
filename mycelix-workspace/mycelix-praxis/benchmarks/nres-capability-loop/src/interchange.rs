//! NRES-006 portable evidence envelope exporter and fail-closed validator.
//!
//! This is a local JSON-LD profile using W3C PROV vocabulary. It is not a
//! complete PROV implementation, official HSEEP/IFRC/ISO/WHO schema, or
//! authentication of the underlying claims.

use crate::evaluate;
use serde_json::{json, Value};
use std::collections::{BTreeMap, BTreeSet};

const NS: &str = "urn:mycelix:praxis:nres:006:";
const PROV_NS: &str = "http://www.w3.org/ns/prov#";

fn node_id(kind: &str, part: &str) -> String {
    format!("{NS}{kind}:{part}")
}

fn reference(id: &str) -> Value {
    json!({ "@id": id })
}

fn field_str<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn evidence_id(cycle: &Value, raw_id: &str) -> Option<String> {
    cycle
        .get("evidence_records")?
        .as_array()?
        .iter()
        .find(|record| field_str(record, "id") == Some(raw_id))
        .map(|_| node_id("evidence", raw_id))
}

fn observation_id(cycle_id: &str, service: &str) -> String {
    node_id("service-observation", &format!("{cycle_id}:{service}"))
}

fn graph_relation_ids(node: &Value, property: &str) -> Vec<String> {
    let Some(value) = node.get(property) else {
        return Vec::new();
    };
    if let Some(items) = value.as_array() {
        return items
            .iter()
            .filter_map(|item| field_str(item, "@id").map(str::to_owned))
            .collect();
    }
    field_str(value, "@id")
        .map(|id| vec![id.to_owned()])
        .unwrap_or_default()
}

fn contains_forbidden_key(value: &Value, path: &str, errors: &mut Vec<String>) {
    const FORBIDDEN: &[&str] = &[
        "person_id",
        "learner_id",
        "natural_person_id",
        "political_affiliation",
        "ideology_score",
        "enlistment_propensity",
        "enlistment_interest",
        "loyalty_score",
        "military_suitability",
        "learner_suitability_score",
        "overall_resilience_score",
        "composite_score",
        "leaderboard_score",
    ];
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                let normalized = key.to_ascii_lowercase();
                if FORBIDDEN.contains(&normalized.as_str()) {
                    errors.push(format!("prohibited field at {path}.{key}"));
                }
                contains_forbidden_key(child, &format!("{path}.{key}"), errors);
            }
        }
        Value::Array(items) => {
            for (index, child) in items.iter().enumerate() {
                contains_forbidden_key(child, &format!("{path}[{index}]"), errors);
            }
        }
        _ => {}
    }
}

/// Validate the portable NRES-006 envelope without dereferencing remote contexts.
///
/// Unknown namespaced extension properties are intentionally retained. The
/// validator checks profile boundaries and graph closure; it does not check
/// legal authority, source truth, signatures, or institutional permission.
pub fn validate_evidence_envelope(envelope: &Value) -> Result<(), Vec<String>> {
    let mut errors = Vec::new();

    let context = envelope.get("@context");
    if context.and_then(|value| value.get("nres")).and_then(Value::as_str) != Some(NS) {
        errors.push("NRES namespace context is missing or incorrect".to_owned());
    }
    if context.and_then(|value| value.get("prov")).and_then(Value::as_str) != Some(PROV_NS) {
        errors.push("W3C PROV namespace context is missing or incorrect".to_owned());
    }
    if field_str(envelope, "schema_version") != Some("1.0.0") {
        errors.push("unsupported NRES-006 schema_version".to_owned());
    }
    if field_str(envelope, "@type") != Some("nres:EvidenceEnvelope") {
        errors.push("envelope @type must be nres:EvidenceEnvelope".to_owned());
    }

    let metadata = envelope.get("bundle_metadata").cloned().unwrap_or(Value::Null);
    if field_str(&metadata, "status") != Some("synthetic-demonstration") {
        errors.push("public conformance envelope must remain synthetic-demonstration".to_owned());
    }
    if field_str(&metadata, "evaluation_split") != Some("public-development-only") {
        errors.push("public fixture cannot be relabeled as pilot or confirmatory evidence".to_owned());
    }
    if metadata.get("no_real_world_effect_claim").and_then(Value::as_bool) != Some(true) {
        errors.push("envelope must disclaim real-world effect claims".to_owned());
    }

    let privacy = envelope.get("privacy").cloned().unwrap_or(Value::Null);
    if field_str(&privacy, "classification") != Some("public-synthetic") {
        errors.push("public fixture privacy classification is incorrect".to_owned());
    }
    if field_str(&privacy, "subject_identifiers") != Some("none-in-public-profile") {
        errors.push("public profile must exclude natural-person identifiers".to_owned());
    }
    if privacy.get("personal_data_present").and_then(Value::as_bool) != Some(false) {
        errors.push("public profile must declare personal_data_present=false".to_owned());
    }

    let integrity = envelope.get("integrity").cloned().unwrap_or(Value::Null);
    if field_str(&integrity, "digest_algorithm") != Some("SHA-256") {
        errors.push("integrity digest algorithm must be SHA-256".to_owned());
    }
    if field_str(&integrity, "signature_status") != Some("unsigned-demo") {
        errors.push("public fixture must remain explicitly unsigned-demo".to_owned());
    }
    match integrity.get("payload_sha256") {
        Some(Value::Null) => {}
        Some(Value::String(digest))
            if digest.len() == 64
                && digest.chars().all(|character| character.is_ascii_hexdigit()) => {}
        _ => errors.push("payload_sha256 must be null or a 64-character hexadecimal digest".to_owned()),
    }

    let graph = envelope
        .get("@graph")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    if graph.is_empty() {
        errors.push("provenance @graph must not be empty".to_owned());
    }
    let mut ids = BTreeSet::new();
    for node in &graph {
        let Some(id) = field_str(node, "@id") else {
            errors.push("every provenance graph node requires @id".to_owned());
            continue;
        };
        if !ids.insert(id.to_owned()) {
            errors.push(format!("duplicate provenance graph node id: {id}"));
        }
        if node.get("@type").is_none() {
            errors.push(format!("provenance graph node {id} requires @type"));
        }
        if field_str(node, "nres:recordKind").is_none() {
            errors.push(format!("provenance graph node {id} requires nres:recordKind"));
        }
    }

    for node in &graph {
        let Some(node_name) = field_str(node, "@id") else {
            continue;
        };
        for relation in [
            "prov:used",
            "prov:wasGeneratedBy",
            "prov:wasDerivedFrom",
            "prov:wasAssociatedWith",
        ] {
            for target in graph_relation_ids(node, relation) {
                if !ids.contains(&target) {
                    errors.push(format!("{node_name}: {relation} references missing graph node {target}"));
                }
            }
        }
    }

    // Closed corrective actions must link to actual evidence and a verification
    // record. The linked verifier node is a role placeholder in this public
    // fixture; it is not an identified person or institutional attestation.
    for node in &graph {
        if field_str(node, "nres:recordKind") != Some("corrective-action")
            || field_str(node, "nres:status") != Some("closed-verified")
        {
            continue;
        }
        let action_id = field_str(node, "nres:actionId").unwrap_or("unknown-action");
        if field_str(node, "nres:completionEvidenceId").is_none() {
            errors.push(format!("{action_id}: closed-verified action requires completion evidence"));
        }
        if node.get("nres:independentlyVerified").and_then(Value::as_bool) != Some(true) {
            errors.push(format!("{action_id}: closed-verified action requires independent verification"));
        }
        if field_str(node, "nres:retestResult") != Some("pass") {
            errors.push(format!("{action_id}: closed-verified action requires a passing retest"));
        }
        if node.get("nres:verificationRecord").and_then(|value| field_str(value, "@id")).is_none() {
            errors.push(format!("{action_id}: closed-verified action requires a verification record"));
        }
        if node.get("nres:retestObservation").and_then(|value| field_str(value, "@id")).is_none() {
            errors.push(format!("{action_id}: closed-verified action requires a retest observation"));
        }
    }

    let sources = envelope.get("source_references").and_then(Value::as_array);
    if sources.is_none_or(|items| items.is_empty()) {
        errors.push("source_references must be a non-empty array".to_owned());
    }
    let measures = envelope.get("measurements").and_then(Value::as_array);
    if measures.is_none() {
        errors.push("measurements must be an array".to_owned());
    }
    let crosswalks = envelope.get("framework_crosswalks").and_then(Value::as_array);
    if crosswalks.is_none_or(|items| items.is_empty()) {
        errors.push("framework_crosswalks must be a non-empty array".to_owned());
    }

    contains_forbidden_key(envelope, "$", &mut errors);

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Export NRES-005 synthetic exercise records as a JSON-LD/PROV evidence envelope.
///
/// The output is explicitly synthetic, public-development-only and unsigned.
/// It is suitable for schema/interoperability testing, not for operational data.
pub fn export_evidence_envelope(
    nres002: &Value,
    loop_fixture: &Value,
) -> Result<Value, Vec<String>> {
    let report = evaluate(nres002, loop_fixture)?;
    let scenario = loop_fixture
        .get("scenario_id")
        .and_then(Value::as_str)
        .ok_or_else(|| vec!["capability-loop scenario_id is missing".to_owned()])?;

    let source_002 = node_id("source", "nres-002");
    let source_005 = node_id("source", "nres-005");
    let scenario_entity = node_id("scenario", scenario);
    let fixture_agent = node_id("agent", "synthetic-fixture-origin");
    let verifier_agent = node_id("agent", "independent-verifier-role-placeholder");

    let mut graph = Vec::<Value>::new();
    graph.push(json!({
        "@id": source_002,
        "@type": ["prov:Entity", "nres:SourceFixture"],
        "nres:recordKind": "source-fixture",
        "nres:path": "examples/courses/civic-resilience-policy-benchmark-fixture-001.json",
        "nres:evidenceStatus": "synthetic",
        "nres:contentIdentifier": "PRAXIS-NRES-002-FIXTURE-001"
    }));
    graph.push(json!({
        "@id": source_005,
        "@type": ["prov:Entity", "nres:SourceFixture"],
        "nres:recordKind": "source-fixture",
        "nres:path": "examples/courses/civic-resilience-capability-loop-fixture-001.json",
        "nres:evidenceStatus": "synthetic",
        "nres:contentIdentifier": "PRAXIS-NRES-005-CAPABILITY-LOOP-001"
    }));
    graph.push(json!({
        "@id": scenario_entity,
        "@type": ["prov:Entity", "nres:Scenario"],
        "nres:recordKind": "scenario",
        "nres:scenarioId": scenario,
        "nres:label": nres002.pointer("/scenario/description"),
        "nres:syntheticOnly": true,
        "prov:wasDerivedFrom": [reference(&source_002), reference(&source_005)]
    }));
    graph.push(json!({
        "@id": fixture_agent,
        "@type": ["prov:Agent", "nres:RolePlaceholder"],
        "nres:recordKind": "role-placeholder",
        "nres:label": "Synthetic fixture origin; not a real person, organization or authority",
        "nres:subjectType": "non-person-placeholder"
    }));
    graph.push(json!({
        "@id": verifier_agent,
        "@type": ["prov:Agent", "nres:RolePlaceholder"],
        "nres:recordKind": "role-placeholder",
        "nres:label": "Independent-verifier role placeholder; no real individual is identified",
        "nres:subjectType": "non-person-placeholder"
    }));

    let cycles = loop_fixture
        .get("cycles")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    let mut activity_ids = BTreeMap::<String, String>::new();
    let mut evidence_ids = BTreeMap::<String, String>::new();
    let mut service_observation_ids = BTreeMap::<String, String>::new();

    for cycle in &cycles {
        let Some(cycle_id) = field_str(cycle, "cycle_id") else {
            return Err(vec!["cycle_id is missing during evidence export".to_owned()]);
        };
        let activity = node_id("activity", cycle_id);
        activity_ids.insert(cycle_id.to_owned(), activity.clone());
        graph.push(json!({
            "@id": activity,
            "@type": ["prov:Activity", "nres:ExerciseCycle"],
            "nres:recordKind": "exercise-cycle",
            "nres:cycleId": cycle_id,
            "nres:cycleOrder": cycle.get("cycle_order"),
            "nres:dataStatus": "illustrative-synthetic",
            "prov:used": [reference(&scenario_entity)],
            "prov:wasAssociatedWith": reference(&fixture_agent)
        }));

        for evidence in cycle.get("evidence_records").and_then(Value::as_array).into_iter().flatten() {
            let Some(raw_id) = field_str(evidence, "id") else {
                continue;
            };
            let id = node_id("evidence", raw_id);
            evidence_ids.insert(raw_id.to_owned(), id.clone());
            graph.push(json!({
                "@id": id,
                "@type": ["prov:Entity", "nres:EvidenceRecord"],
                "nres:recordKind": "evidence-record",
                "nres:evidenceId": raw_id,
                "nres:evidenceType": evidence.get("type"),
                "nres:claimLabel": evidence.get("claim_label"),
                "nres:independentlyReviewed": evidence.get("independently_reviewed"),
                "nres:syntheticOnly": true,
                "prov:wasGeneratedBy": reference(&activity)
            }));
        }

        for observation in cycle.get("objective_observations").and_then(Value::as_array).into_iter().flatten() {
            let Some(objective_id) = field_str(observation, "objective_id") else {
                continue;
            };
            let refs = observation
                .get("evidence_ids")
                .and_then(Value::as_array)
                .into_iter()
                .flatten()
                .filter_map(Value::as_str)
                .filter_map(|raw| evidence_ids.get(raw))
                .map(|id| reference(id))
                .collect::<Vec<_>>();
            let observation_node = node_id("objective-observation", &format!("{cycle_id}:{objective_id}"));
            graph.push(json!({
                "@id": observation_node,
                "@type": ["prov:Entity", "nres:ObjectiveObservation"],
                "nres:recordKind": "objective-observation",
                "nres:objectiveId": objective_id,
                "nres:acceptedObservation": observation.get("accepted_observation"),
                "nres:syntheticOnly": true,
                "prov:wasGeneratedBy": reference(&activity),
                "prov:wasDerivedFrom": refs
            }));
        }

        for observation in cycle.get("service_floor_observations").and_then(Value::as_array).into_iter().flatten() {
            let (Some(service), Some(raw_evidence_id)) = (
                field_str(observation, "service"),
                field_str(observation, "evidence_id"),
            ) else {
                continue;
            };
            let Some(raw_evidence) = evidence_ids.get(raw_evidence_id) else {
                return Err(vec![format!("{cycle_id}:{service}: evidence ID failed to resolve during export")]);
            };
            let id = node_id("service-observation", &format!("{cycle_id}:{service}"));
            service_observation_ids.insert(format!("{cycle_id}:{service}"), id.clone());
            let observed = observation.get("observed_percent").and_then(Value::as_f64).unwrap_or(-1.0);
            let required = observation.get("required_floor_percent").and_then(Value::as_f64).unwrap_or(101.0);
            graph.push(json!({
                "@id": id,
                "@type": ["prov:Entity", "nres:ServiceFloorObservation"],
                "nres:recordKind": "service-floor-observation",
                "nres:service": service,
                "nres:observedPercent": observed,
                "nres:requiredFloorPercent": required,
                "nres:passed": observed >= required,
                "nres:evidenceId": raw_evidence_id,
                "nres:syntheticOnly": true,
                "prov:wasGeneratedBy": reference(&activity),
                "prov:wasDerivedFrom": reference(raw_evidence)
            }));
        }
    }

    let improvement_activity = node_id("activity", "improvement-planning:CYCLE-02");
    let actions = loop_fixture
        .get("corrective_actions")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    let findings = actions
        .iter()
        .filter_map(|action| field_str(action, "action_id"))
        .map(|action_id| reference(&node_id("finding", action_id)))
        .collect::<Vec<_>>();
    graph.push(json!({
        "@id": improvement_activity,
        "@type": ["prov:Activity", "nres:ImprovementPlanning"],
        "nres:recordKind": "improvement-planning",
        "nres:syntheticOnly": true,
        "prov:used": findings,
        "prov:wasAssociatedWith": reference(&fixture_agent)
    }));

    for action in &actions {
        let Some(action_id) = field_str(action, "action_id") else {
            continue;
        };
        let finding = node_id("finding", action_id);
        let action_node = node_id("corrective-action", action_id);
        let opened_cycle = field_str(action, "opened_cycle_id").unwrap_or("UNKNOWN");
        let opened_activity = activity_ids.get(opened_cycle);
        if opened_activity.is_none() {
            return Err(vec![format!("{action_id}: opened cycle does not resolve during export")]);
        }
        graph.push(json!({
            "@id": finding,
            "@type": ["prov:Entity", "nres:AfterActionFinding"],
            "nres:recordKind": "after-action-finding",
            "nres:description": action.get("finding"),
            "nres:rootCauseHypothesis": action.get("root_cause_hypothesis"),
            "nres:syntheticOnly": true,
            "prov:wasGeneratedBy": reference(opened_activity.unwrap())
        }));

        let status = field_str(action, "status").unwrap_or("open");
        let completion_raw = field_str(action, "completion_evidence_id");
        let retest_cycle = field_str(action, "retest_cycle_id");
        let retest_service = field_str(action, "retest_service");
        let completion_node = completion_raw.and_then(|raw| evidence_ids.get(raw));
        let retest_observation = retest_cycle
            .zip(retest_service)
            .and_then(|(cycle, service)| service_observation_ids.get(&format!("{cycle}:{service}")));

        let mut derived = vec![reference(&finding)];
        if let Some(completion) = completion_node {
            derived.push(reference(completion));
        }
        let mut action_value = json!({
            "@id": action_node,
            "@type": ["prov:Entity", "nres:CorrectiveAction"],
            "nres:recordKind": "corrective-action",
            "nres:actionId": action_id,
            "nres:status": status,
            "nres:ownerRole": action.get("owner_role"),
            "nres:dueByCycleId": action.get("due_by_cycle_id"),
            "nres:retestResult": action.get("retest_result"),
            "nres:independentlyVerified": action.get("independently_verified"),
            "nres:syntheticOnly": true,
            "prov:wasGeneratedBy": reference(&improvement_activity),
            "prov:wasDerivedFrom": derived
        });
        if let Some(raw) = completion_raw {
            action_value["nres:completionEvidenceId"] = json!(raw);
        }
        if let Some(retest_cycle) = retest_cycle {
            if let Some(activity) = activity_ids.get(retest_cycle) {
                action_value["nres:retestActivity"] = reference(activity);
            }
        }
        if let Some(observation) = retest_observation {
            action_value["nres:retestObservation"] = reference(observation);
        }

        if status == "closed-verified" {
            let (Some(completion), Some(retest_observation)) = (completion_node, retest_observation) else {
                return Err(vec![format!("{action_id}: closed-verified action lacks resolvable evidence/retest during export")]);
            };
            let verify_activity = node_id("activity", &format!("verification:{action_id}"));
            let verification_record = node_id("verification", action_id);
            graph.push(json!({
                "@id": verify_activity,
                "@type": ["prov:Activity", "nres:IndependentVerification"],
                "nres:recordKind": "independent-verification-activity",
                "nres:syntheticOnly": true,
                "prov:used": [reference(completion), reference(retest_observation)],
                "prov:wasAssociatedWith": reference(&verifier_agent)
            }));
            graph.push(json!({
                "@id": verification_record,
                "@type": ["prov:Entity", "nres:VerificationRecord"],
                "nres:recordKind": "verification-record",
                "nres:actionId": action_id,
                "nres:result": "pass",
                "nres:syntheticOnly": true,
                "prov:wasGeneratedBy": reference(&verify_activity)
            }));
            action_value["nres:verificationRecord"] = reference(&verification_record);
        }
        graph.push(action_value);
    }

    let mut measurements = Vec::new();
    for cycle_metric in report.get("cycle_metrics").and_then(Value::as_array).into_iter().flatten() {
        let cycle_id = field_str(cycle_metric, "cycle_id").unwrap_or("UNKNOWN");
        measurements.push(json!({
            "measure_id": "M-OBJECTIVE-EVIDENCE",
            "cycle_id": cycle_id,
            "unit": "ratio",
            "result": cycle_metric.get("objective_evidence_coverage"),
            "evidence_ids": []
        }));
        measurements.push(json!({
            "measure_id": "M-SERVICE-FLOOR",
            "cycle_id": cycle_id,
            "unit": "per-service-percent-and-pass-fail",
            "result": cycle_metric.get("service_floors"),
            "evidence_ids": []
        }));
        measurements.push(json!({
            "measure_id": "M-RESOURCE-FEASIBILITY",
            "cycle_id": cycle_id,
            "unit": "fictional-points-and-responder-hours",
            "result": {
                "planning_budget": cycle_metric.get("planning_budget"),
                "responder_hours": cycle_metric.get("responder_hours")
            },
            "evidence_ids": []
        }));
        measurements.push(json!({
            "measure_id": "M-AUTHORITY-EVIDENCE",
            "cycle_id": cycle_id,
            "unit": "ratio",
            "result": cycle_metric.get("authority_evidence"),
            "evidence_ids": []
        }));
        measurements.push(json!({
            "measure_id": "M-ACCESSIBILITY-COVERAGE",
            "cycle_id": cycle_id,
            "unit": "ratio",
            "result": cycle_metric.get("accessibility_test_coverage"),
            "evidence_ids": []
        }));
    }
    let improvement = report.get("improvement_system").cloned().unwrap_or(Value::Null);
    measurements.push(json!({
        "measure_id": "M-ACTION-VERIFICATION",
        "cycle_id": "CYCLE-02",
        "unit": "ratio",
        "result": improvement.get("verified_actions"),
        "evidence_ids": ["EV-21", "EV-22"]
    }));
    measurements.push(json!({
        "measure_id": "M-REPEAT-DEFECT",
        "cycle_id": "CYCLE-02",
        "unit": "ratio",
        "result": improvement.get("retested_failure_recurrence"),
        "evidence_ids": ["EV-14"]
    }));

    let crosswalks = json!([
        {
            "framework": "W3C PROV-O / PROV-DM",
            "source_element": "prov:Entity, prov:Activity, prov:Agent, prov:used, prov:wasGeneratedBy, prov:wasDerivedFrom, prov:wasAssociatedWith",
            "target_element": "@graph provenance nodes and relation properties",
            "mapping_type": "exact-vocabulary",
            "scope_note": "Reuses PROV terms; does not claim complete PROV conformance or authenticate assertions."
        },
        {
            "framework": "W3C JSON-LD 1.1",
            "source_element": "JSON-LD context and @graph",
            "target_element": "@context and @graph",
            "mapping_type": "exact-vocabulary",
            "scope_note": "JSON-LD serialization profile; offline context mappings only."
        },
        {
            "framework": "FEMA HSEEP",
            "source_element": "Exercise objectives, evaluator observations, AAR/IP findings and corrective actions",
            "target_element": "objective observations, evidence records, after-action findings and corrective actions",
            "mapping_type": "conceptual",
            "scope_note": "Conceptual alignment only; not a native FEMA schema or official EEG/AAR-IP template."
        },
        {
            "framework": "IFRC Preparedness for Effective Response",
            "source_element": "Assessment, Prioritisation & Analysis, Work-Plan, Action & Accountability",
            "target_element": "measurements, findings, corrective actions, retests",
            "mapping_type": "conceptual",
            "scope_note": "Conceptual phase crosswalk; does not replace IFRC/National Society authority."
        },
        {
            "framework": "IFRC Emergency Operations Centres Guide 2026",
            "source_element": "EOC information management and readiness assessment",
            "target_element": "source references, evidence status, readiness measures",
            "mapping_type": "conceptual",
            "scope_note": "No official EOC tool integration or authorization is implied."
        },
        {
            "framework": "ISO 22301:2019",
            "source_element": "Business continuity monitoring, performance evaluation and continual improvement",
            "target_element": "service-floor measures, corrective actions, independent retests",
            "mapping_type": "conceptual",
            "scope_note": "No ISO clause-level mapping, conformity or certification is claimed."
        },
        {
            "framework": "WHO Public Health EOC Framework, second edition (2026)",
            "source_element": "Information systems/data standards; workforce, training, exercises, monitoring and evaluation",
            "target_element": "evidence envelope, measure catalog, training and exercise observations",
            "mapping_type": "conceptual",
            "scope_note": "Conceptual crosswalk only; not a WHO-endorsed health-data exchange."
        },
        {
            "framework": "OASIS Common Alerting Protocol v1.2",
            "source_element": "All-hazard emergency alerts and public warnings",
            "target_element": "out-of-scope for after-action evidence exchange",
            "mapping_type": "out-of-scope",
            "scope_note": "CAP is for alert messaging; an alert may be referenced as a source artifact but CAP is not the evidence-envelope schema."
        }
    ]);

    let envelope = json!({
        "@context": {
            "nres": NS,
            "prov": PROV_NS
        },
        "@id": node_id("bundle", scenario),
        "@type": "nres:EvidenceEnvelope",
        "schema_version": "1.0.0",
        "bundle_metadata": {
            "bundle_id": "PRAXIS-NRES-006-DEMO-001",
            "scenario_id": scenario,
            "status": "synthetic-demonstration",
            "evaluation_split": "public-development-only",
            "source_benchmark": "PRAXIS-NRES-005-CAPABILITY-LOOP-001",
            "generated_by": "praxis-nres-capability-loop-rust/0.1.0",
            "purpose": "Demonstrate portable exchange of synthetic exercise observations and corrective-action provenance.",
            "no_real_world_effect_claim": true
        },
        "source_references": [
            {
                "source_id": "NRES-002",
                "path": "examples/courses/civic-resilience-policy-benchmark-fixture-001.json",
                "media_type": "application/json",
                "evidence_status": "synthetic"
            },
            {
                "source_id": "NRES-005",
                "path": "examples/courses/civic-resilience-capability-loop-fixture-001.json",
                "media_type": "application/json",
                "evidence_status": "synthetic"
            }
        ],
        "@graph": graph,
        "measurements": measurements,
        "framework_crosswalks": crosswalks,
        "privacy": {
            "classification": "public-synthetic",
            "subject_identifiers": "none-in-public-profile",
            "default_purpose": "benchmark development and formative interoperability checks",
            "prohibited_inferences": [
                "political affiliation",
                "ideology",
                "loyalty",
                "enlistment interest",
                "personal military or coercive-service suitability"
            ],
            "personal_data_present": false
        },
        "integrity": {
            "digest_algorithm": "SHA-256",
            "digest_scope": "exact UTF-8 bytes of the exported envelope; no reserialization implied",
            "payload_sha256": null,
            "signature_status": "unsigned-demo",
            "signature_reference": null
        },
        "extensions": {
            "ext:reviewNote": "Unknown namespaced extension; lossless adapters must preserve this string.",
            "ext:opaquePayload": { "vendor": "synthetic-conformance", "revision": 2 }
        }
    });

    validate_evidence_envelope(&envelope)?;
    Ok(envelope)
}

#[cfg(test)]
mod tests {
    use super::{export_evidence_envelope, validate_evidence_envelope};
    use serde_json::{json, Value};

    const NRES002: &str = include_str!("../../../examples/courses/civic-resilience-policy-benchmark-fixture-001.json");
    const NRES005: &str = include_str!("../../../examples/courses/civic-resilience-capability-loop-fixture-001.json");

    fn envelope() -> Value {
        let nres002: Value = serde_json::from_str(NRES002).expect("NRES-002 fixture JSON");
        let nres005: Value = serde_json::from_str(NRES005).expect("NRES-005 fixture JSON");
        export_evidence_envelope(&nres002, &nres005).expect("exported envelope should validate")
    }

    #[test]
    fn exports_jsonld_prov_envelope_with_resolved_graph() {
        let bundle = envelope();
        assert_eq!(bundle.pointer("/@context/prov").and_then(Value::as_str), Some("http://www.w3.org/ns/prov#"));
        assert_eq!(bundle.pointer("/bundle_metadata/evaluation_split").and_then(Value::as_str), Some("public-development-only"));
        validate_evidence_envelope(&bundle).expect("all provenance references should resolve");
    }

    #[test]
    fn preserves_unknown_namespaced_extensions() {
        let bundle = envelope();
        let round_trip: Value = serde_json::from_str(&serde_json::to_string(&bundle).unwrap()).unwrap();
        assert_eq!(round_trip.pointer("/extensions/ext:reviewNote").and_then(Value::as_str), Some("Unknown namespaced extension; lossless adapters must preserve this string."));
        assert_eq!(round_trip.pointer("/extensions/ext:opaquePayload/revision").and_then(Value::as_u64), Some(2));
    }

    #[test]
    fn rejects_missing_provenance_reference() {
        let mut bundle = envelope();
        bundle["@graph"][0]["prov:wasDerivedFrom"] = json!([{ "@id": "urn:missing:entity" }]);
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn rejects_personal_and_political_fields() {
        let mut bundle = envelope();
        bundle["extensions"]["person_id"] = json!("synthetic-person");
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn rejects_aggregate_score_fields() {
        let mut bundle = envelope();
        bundle["measurements"][0]["composite_score"] = json!(100);
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn rejects_public_fixture_relabelled_as_confirmatory() {
        let mut bundle = envelope();
        bundle["bundle_metadata"]["evaluation_split"] = json!("confirmatory-held-out");
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn rejects_closed_action_without_verification_record() {
        let mut bundle = envelope();
        let action = bundle["@graph"].as_array_mut().unwrap().iter_mut()
            .find(|node| node.get("nres:recordKind").and_then(Value::as_str) == Some("corrective-action")
                && node.get("nres:status").and_then(Value::as_str) == Some("closed-verified"))
            .expect("fixture has closed corrective action");
        action.as_object_mut().unwrap().remove("nres:verificationRecord");
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn rejects_malformed_digest() {
        let mut bundle = envelope();
        bundle["integrity"]["payload_sha256"] = json!("not-a-digest");
        assert!(validate_evidence_envelope(&bundle).is_err());
    }

    #[test]
    fn preserves_non_compensable_service_failures() {
        let bundle = envelope();
        let failed = bundle["@graph"].as_array().unwrap().iter()
            .filter(|node| node.get("nres:recordKind").and_then(Value::as_str) == Some("service-floor-observation")
                && node.get("nres:passed").and_then(Value::as_bool) == Some(false))
            .count();
        assert!(failed >= 1);
        assert!(bundle.pointer("/integrity/signature_status").and_then(Value::as_str) == Some("unsigned-demo"));
    }
}
