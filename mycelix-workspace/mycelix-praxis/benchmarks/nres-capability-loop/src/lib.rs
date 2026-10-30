//! Deterministic evaluator for the public Praxis NRES-005 capability-loop fixture.
//!
//! This crate validates synthetic exercise records and derives separate measures.
//! It does not score people, infer political views, determine legal authority,
//! issue operational instructions, or estimate real-world readiness.

use serde_json::{json, Value};

pub mod interchange;
pub use interchange::{export_evidence_envelope, validate_evidence_envelope};

use std::collections::{BTreeMap, BTreeSet, HashMap};

fn str_field<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key)?.as_str()
}

fn num_field(value: &Value, key: &str) -> Option<f64> {
    value.get(key)?.as_f64()
}

fn count_field(value: &Value, key: &str) -> Option<u64> {
    value.get(key)?.as_u64()
}

fn array_field<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key)?.as_array()
}

fn ratio(numerator: usize, denominator: usize) -> Value {
    let value = if denominator == 0 {
        Value::Null
    } else {
        json!(numerator as f64 / denominator as f64)
    };
    json!({
        "numerator": numerator,
        "denominator": denominator,
        "value": value
    })
}

fn push_expected_count(
    root: &Value,
    path: &str,
    actual: usize,
    label: &str,
    errors: &mut Vec<String>,
) {
    match root.pointer(path).and_then(Value::as_u64) {
        Some(expected) if expected == actual as u64 => {}
        _ => errors.push(format!("expected derived metric mismatch: {label}")),
    }
}

fn evidence_ids(cycle: &Value) -> BTreeSet<String> {
    array_field(cycle, "evidence_records")
        .into_iter()
        .flatten()
        .filter_map(|record| str_field(record, "id").map(str::to_owned))
        .collect()
}

/// Validate the NRES-005 public fixture against the source NRES-002 service floors
/// and return an auditable report containing only disaggregated measures.
///
/// A successful report means that the fixture is internally consistent. It is
/// not a claim about actual preparedness, learning effectiveness or causality.
pub fn evaluate(nres002: &Value, loop_fixture: &Value) -> Result<Value, Vec<String>> {
    let mut errors = Vec::new();

    if str_field(loop_fixture, "benchmark_id")
        != Some("PRAXIS-NRES-005-CAPABILITY-LOOP-001")
    {
        errors.push("unexpected capability-loop benchmark_id".to_owned());
    }
    if str_field(loop_fixture, "schema_version") != Some("1.0.0") {
        errors.push("unsupported capability-loop schema_version".to_owned());
    }
    if str_field(loop_fixture, "status") != Some("synthetic-demonstration-for-review") {
        errors.push("capability-loop fixture must remain explicitly synthetic".to_owned());
    }
    if str_field(loop_fixture, "evaluation_split") != Some("public-development-only") {
        errors.push("capability-loop fixture must remain public development data".to_owned());
    }
    if loop_fixture.get("active_in_curriculum").and_then(Value::as_bool) != Some(false) {
        errors.push("capability-loop fixture must remain inactive".to_owned());
    }
    let source_scenario_id = nres002.pointer("/scenario/event_id").and_then(Value::as_str);
    if str_field(loop_fixture, "scenario_id") != source_scenario_id {
        errors.push("capability-loop scenario_id must match NRES-002".to_owned());
    }
    for forbidden in [
        "overall_resilience_score",
        "composite_score",
        "leaderboard_score",
        "learner_suitability_score",
    ] {
        if loop_fixture.get(forbidden).is_some() {
            errors.push(format!("forbidden aggregate or personal score field: {forbidden}"));
        }
    }

    let Some(source_floors) = nres002
        .pointer("/synthetic_constraints/minimum_service_floor_percent")
        .and_then(Value::as_object)
    else {
        return Err(vec!["NRES-002 minimum service floors are missing".to_owned()]);
    };
    let floors: BTreeMap<String, f64> = source_floors
        .iter()
        .filter_map(|(service, value)| value.as_f64().map(|floor| (service.clone(), floor)))
        .collect();
    if floors.len() != source_floors.len() || floors.is_empty() {
        errors.push("NRES-002 service floors must be numeric and non-empty".to_owned());
    }

    let mut objective_services = BTreeMap::new();
    let objectives = array_field(loop_fixture, "objectives").cloned().unwrap_or_default();
    for objective in &objectives {
        let id = str_field(objective, "id");
        let service = str_field(objective, "service");
        match (id, service) {
            (Some(id), Some(service)) => {
                if objective_services.insert(id.to_owned(), service.to_owned()).is_some() {
                    errors.push(format!("duplicate objective ID: {id}"));
                }
                if !floors.contains_key(service) {
                    errors.push(format!("objective {id} maps to unknown service {service}"));
                }
            }
            _ => errors.push("each objective needs a string id and service".to_owned()),
        }
    }
    if objective_services.is_empty() {
        errors.push("capability-loop objectives are missing".to_owned());
    }

    let mut cycle_values: HashMap<String, Value> = HashMap::new();
    let mut cycle_metrics = Vec::new();
    let mut floor_passes_by_cycle: HashMap<String, BTreeMap<String, bool>> = HashMap::new();
    let cycles = array_field(loop_fixture, "cycles").cloned().unwrap_or_default();
    if cycles.len() < 2 {
        errors.push("capability-loop requires at least two exercise cycles".to_owned());
    }

    for cycle in &cycles {
        let cycle_id = str_field(cycle, "cycle_id").unwrap_or("UNKNOWN").to_owned();
        if cycle_values.insert(cycle_id.clone(), cycle.clone()).is_some() {
            errors.push(format!("duplicate cycle ID: {cycle_id}"));
        }
        if str_field(cycle, "data_status") != Some("illustrative-synthetic") {
            errors.push(format!("{cycle_id}: public cycle data must be labeled synthetic"));
        }

        let evidence = evidence_ids(cycle);
        let scheduled: BTreeSet<String> = array_field(cycle, "scheduled_objective_ids")
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
            .map(str::to_owned)
            .collect();
        if scheduled.len() != objective_services.len()
            || objective_services.keys().any(|id| !scheduled.contains(id))
        {
            errors.push(format!("{cycle_id}: scheduled objectives must cover every objective exactly once"));
        }

        let observations = array_field(cycle, "objective_observations").cloned().unwrap_or_default();
        let mut observed_objectives = BTreeSet::new();
        let mut evidence_backed_objectives = 0usize;
        for observation in &observations {
            let Some(objective_id) = str_field(observation, "objective_id") else {
                errors.push(format!("{cycle_id}: objective observation is missing objective_id"));
                continue;
            };
            if !observed_objectives.insert(objective_id.to_owned()) {
                errors.push(format!("{cycle_id}: duplicate objective observation {objective_id}"));
            }
            if !objective_services.contains_key(objective_id) {
                errors.push(format!("{cycle_id}: unknown objective observation {objective_id}"));
            }
            let referenced = array_field(observation, "evidence_ids").cloned().unwrap_or_default();
            let accepted = observation.get("accepted_observation").and_then(Value::as_bool) == Some(true);
            if accepted {
                if referenced.is_empty() {
                    errors.push(format!("{cycle_id}: accepted observation {objective_id} has no evidence"));
                } else if referenced.iter().any(|id| id.as_str().is_none_or(|id| !evidence.contains(id))) {
                    errors.push(format!("{cycle_id}: observation {objective_id} references missing evidence"));
                } else {
                    evidence_backed_objectives += 1;
                }
            }
        }
        if observed_objectives != scheduled {
            errors.push(format!("{cycle_id}: objective observations must match scheduled objectives"));
        }

        let service_observations = array_field(cycle, "service_floor_observations")
            .cloned()
            .unwrap_or_default();
        let mut floor_results = BTreeMap::new();
        let mut floor_pass_count = 0usize;
        for observation in &service_observations {
            let Some(service) = str_field(observation, "service") else {
                errors.push(format!("{cycle_id}: service observation is missing service"));
                continue;
            };
            let Some(expected_floor) = floors.get(service) else {
                errors.push(format!("{cycle_id}: unexpected service {service}"));
                continue;
            };
            let observed = num_field(observation, "observed_percent");
            let required = num_field(observation, "required_floor_percent");
            if required != Some(*expected_floor) {
                errors.push(format!("{cycle_id}: {service} floor differs from frozen NRES-002"));
            }
            if observed.is_none_or(|value| !(0.0..=100.0).contains(&value)) {
                errors.push(format!("{cycle_id}: {service} observed percentage is invalid"));
                continue;
            }
            let evidence_id = str_field(observation, "evidence_id");
            if evidence_id.is_none_or(|id| !evidence.contains(id)) {
                errors.push(format!("{cycle_id}: {service} observation references missing evidence"));
            }
            let passed = observed.unwrap_or(-1.0) >= *expected_floor;
            if passed {
                floor_pass_count += 1;
            }
            if floor_results.insert(service.to_owned(), passed).is_some() {
                errors.push(format!("{cycle_id}: duplicate service-floor observation {service}"));
            }
        }
        if floor_results.len() != floors.len() {
            errors.push(format!("{cycle_id}: service-floor observations must cover all services"));
        }
        floor_passes_by_cycle.insert(cycle_id.clone(), floor_results);

        let resources = cycle.get("resources").cloned().unwrap_or(Value::Null);
        let budget_available = num_field(&resources, "planning_budget_available");
        let budget_committed = num_field(&resources, "planning_budget_committed");
        let responder_available = num_field(&resources, "responder_hours_available");
        let responder_committed = num_field(&resources, "responder_hours_committed");
        let source_budget = nres002.pointer("/synthetic_constraints/planning_budget/available").and_then(Value::as_f64);
        let source_hours = nres002.pointer("/synthetic_constraints/deployable_trained_responder_hours/available").and_then(Value::as_f64);
        if budget_available != source_budget || budget_committed.is_none_or(|value| value < 0.0 || value > budget_available.unwrap_or(0.0)) {
            errors.push(format!("{cycle_id}: planning budget commitment is outside frozen availability"));
        }
        if responder_available != source_hours || responder_committed.is_none_or(|value| value < 0.0 || value > responder_available.unwrap_or(0.0)) {
            errors.push(format!("{cycle_id}: responder-hour commitment is outside frozen availability"));
        }

        let authority = cycle.get("authority_evidence").cloned().unwrap_or(Value::Null);
        let authority_numerator = count_field(&authority, "decisions_with_role_basis_and_review_path").unwrap_or(0) as usize;
        let authority_denominator = count_field(&authority, "decisions_sampled").unwrap_or(0) as usize;
        if count_field(&authority, "decisions_with_role_basis_and_review_path").is_none()
            || count_field(&authority, "decisions_sampled").is_none()
            || authority_numerator > authority_denominator
        {
            errors.push(format!("{cycle_id}: authority evidence numerator/denominator is missing or invalid"));
        }
        let accessibility = cycle.get("accessibility").cloned().unwrap_or(Value::Null);
        let accessibility_specified = count_field(&accessibility, "requirements_specified").unwrap_or(0) as usize;
        let accessibility_tested = count_field(&accessibility, "requirements_tested").unwrap_or(0) as usize;
        let accessibility_passing = count_field(&accessibility, "requirements_passing").unwrap_or(0) as usize;
        if count_field(&accessibility, "requirements_specified").is_none()
            || count_field(&accessibility, "requirements_tested").is_none()
            || count_field(&accessibility, "requirements_passing").is_none()
            || accessibility_passing > accessibility_tested
            || accessibility_tested > accessibility_specified
        {
            errors.push(format!("{cycle_id}: accessibility counts are missing or inconsistent"));
        }

        cycle_metrics.push(json!({
            "cycle_id": cycle_id,
            "objective_evidence_coverage": ratio(evidence_backed_objectives, scheduled.len()),
            "service_floor_passes": ratio(floor_pass_count, floors.len()),
            "all_service_floors_pass": floor_pass_count == floors.len(),
            "service_floors": service_observations.iter().map(|item| json!({
                "service": item.get("service"),
                "observed_percent": item.get("observed_percent"),
                "required_floor_percent": item.get("required_floor_percent"),
                "passed": item.get("observed_percent").and_then(Value::as_f64).zip(item.get("required_floor_percent").and_then(Value::as_f64)).is_some_and(|(observed, required)| observed >= required)
            })).collect::<Vec<_>>(),
            "planning_budget": ratio(
                budget_committed.unwrap_or(0.0) as usize,
                budget_available.unwrap_or(0.0) as usize
            ),
            "responder_hours": ratio(
                responder_committed.unwrap_or(0.0) as usize,
                responder_available.unwrap_or(0.0) as usize
            ),
            "authority_evidence": ratio(authority_numerator, authority_denominator),
            "accessibility_test_coverage": ratio(accessibility_tested, accessibility_specified),
            "accessibility_requirements_passing": ratio(accessibility_passing, accessibility_specified)
        }));
    }

    let actions = array_field(loop_fixture, "corrective_actions").cloned().unwrap_or_default();
    let mut action_ids = BTreeSet::new();
    let mut verified_due_actions = 0usize;
    let mut due_actions = 0usize;
    for action in &actions {
        let action_id = str_field(action, "action_id").unwrap_or("UNKNOWN");
        if !action_ids.insert(action_id.to_owned()) {
            errors.push(format!("duplicate corrective-action ID: {action_id}"));
        }
        if str_field(action, "owner_role").is_none() {
            errors.push(format!("{action_id}: accountable owner role is missing"));
        }
        let status = str_field(action, "status").unwrap_or("");
        if !["open", "closed-verified", "blocked", "late", "not-tested", "reopened"].contains(&status) {
            errors.push(format!("{action_id}: unsupported corrective-action status"));
        }
        let opened_cycle_id = str_field(action, "opened_cycle_id").unwrap_or("");
        if !cycle_values.contains_key(opened_cycle_id) {
            errors.push(format!("{action_id}: opened cycle does not exist"));
        }
        let due_cycle_id = str_field(action, "due_by_cycle_id").unwrap_or("");
        if !cycle_values.contains_key(due_cycle_id) {
            errors.push(format!("{action_id}: due cycle does not exist"));
        }
        if due_cycle_id == "CYCLE-02" {
            due_actions += 1;
        }

        if status == "closed-verified" {
            let completion_id = str_field(action, "completion_evidence_id");
            let retest_id = str_field(action, "retest_cycle_id").unwrap_or("");
            let retest = cycle_values.get(retest_id);
            let independently_verified = action.get("independently_verified").and_then(Value::as_bool) == Some(true);
            let successful_retest = str_field(action, "retest_result") == Some("pass");
            if completion_id.is_none() || !independently_verified || !successful_retest || retest.is_none() {
                errors.push(format!("{action_id}: verified closure requires completion evidence, independent verification and successful retest"));
                continue;
            }
            let retest = retest.unwrap();
            let retest_evidence = evidence_ids(retest);
            if !retest_evidence.contains(completion_id.unwrap()) {
                errors.push(format!("{action_id}: completion evidence is absent from the retest cycle"));
            }
            let independently_reviewed = array_field(retest, "evidence_records")
                .into_iter()
                .flatten()
                .any(|record| {
                    str_field(record, "id") == completion_id
                        && record.get("independently_reviewed").and_then(Value::as_bool) == Some(true)
                });
            if !independently_reviewed {
                errors.push(format!("{action_id}: completion evidence must be independently reviewed"));
            }
            let objective_id = str_field(action, "retest_objective_id").unwrap_or("");
            let expected_service = objective_services.get(objective_id);
            let retest_service = str_field(action, "retest_service");
            if expected_service.map(String::as_str) != retest_service {
                errors.push(format!("{action_id}: retest objective/service mapping is inconsistent"));
            }
            if let Some(service) = retest_service {
                let passed = floor_passes_by_cycle
                    .get(retest_id)
                    .and_then(|results| results.get(service))
                    .copied()
                    .unwrap_or(false);
                if !passed {
                    errors.push(format!("{action_id}: closure retest must pass the linked service floor"));
                }
            }
            if due_cycle_id == "CYCLE-02" {
                verified_due_actions += 1;
            }
        } else if status == "open" && str_field(action, "retest_result") == Some("pass") {
            errors.push(format!("{action_id}: open action has an unreconciled passing retest"));
        }
    }

    let prior_failures: BTreeSet<String> = floor_passes_by_cycle
        .get("CYCLE-01")
        .into_iter()
        .flat_map(|results| results.iter())
        .filter_map(|(service, passed)| (!passed).then_some(service.clone()))
        .collect();
    let retested: BTreeMap<String, bool> = floor_passes_by_cycle
        .get("CYCLE-02")
        .cloned()
        .unwrap_or_default();
    let prior_retested: Vec<String> = prior_failures
        .iter()
        .filter(|service| retested.contains_key(*service))
        .cloned()
        .collect();
    let repeated_failures = prior_retested
        .iter()
        .filter(|service| !retested.get(*service).copied().unwrap_or(false))
        .count();

    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_01/objective_evidence_coverage/numerator",
        4,
        "CYCLE-01 objective evidence numerator",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_01/service_floors_passed/numerator",
        cycle_metrics.first().and_then(|m| m.pointer("/service_floor_passes/numerator")).and_then(Value::as_u64).unwrap_or(0) as usize,
        "CYCLE-01 service floors",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_02/service_floors_passed/numerator",
        cycle_metrics.get(1).and_then(|m| m.pointer("/service_floor_passes/numerator")).and_then(Value::as_u64).unwrap_or(0) as usize,
        "CYCLE-02 service floors",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_02/verified_corrective_actions/numerator",
        verified_due_actions,
        "CYCLE-02 verified actions",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_02/verified_corrective_actions/denominator",
        due_actions,
        "CYCLE-02 due actions",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_02/retested_failure_recurrence/numerator",
        repeated_failures,
        "CYCLE-02 repeat defects",
        &mut errors,
    );
    push_expected_count(
        loop_fixture,
        "/expected_derived_results/CYCLE_02/retested_failure_recurrence/denominator",
        prior_retested.len(),
        "CYCLE-02 retested prior failures",
        &mut errors,
    );

    if !array_field(loop_fixture, "interpretation_guardrails")
        .is_some_and(|items| items.iter().any(|item| item.as_str().is_some_and(|text| text.contains("single overall"))))
    {
        errors.push("capability-loop must prohibit a single compensating overall score".to_owned());
    }

    if !errors.is_empty() {
        return Err(errors);
    }

    let cycle2 = cycle_metrics.iter().find(|metric| metric.get("cycle_id").and_then(Value::as_str) == Some("CYCLE-02"));
    let all_floors_pass = cycle2
        .and_then(|metric| metric.get("all_service_floors_pass"))
        .and_then(Value::as_bool)
        .unwrap_or(false);

    Ok(json!({
        "benchmark_id": "PRAXIS-NRES-005-CAPABILITY-LOOP-001",
        "interpretation": "synthetic fixture consistency only; not real-world readiness or causal evidence",
        "cycle_metrics": cycle_metrics,
        "improvement_system": {
            "verified_actions": ratio(verified_due_actions, due_actions),
            "retested_failure_recurrence": ratio(repeated_failures, prior_retested.len()),
            "all_critical_service_floors_pass_at_retest": all_floors_pass
        },
    }))
}

#[cfg(test)]
mod tests {
    use super::evaluate;
    use serde_json::{json, Value};

    const NRES002: &str = include_str!("../../../examples/courses/civic-resilience-policy-benchmark-fixture-001.json");
    const NRES005: &str = include_str!("../../../examples/courses/civic-resilience-capability-loop-fixture-001.json");

    fn fixtures() -> (Value, Value) {
        (
            serde_json::from_str(NRES002).expect("NRES-002 fixture must be valid JSON"),
            serde_json::from_str(NRES005).expect("NRES-005 fixture must be valid JSON"),
        )
    }

    #[test]
    fn derives_separate_cycle_and_improvement_metrics() {
        let (nres002, loop_fixture) = fixtures();
        let report = evaluate(&nres002, &loop_fixture).expect("fixture should validate");
        assert_eq!(report.pointer("/cycle_metrics/0/service_floor_passes/numerator").and_then(Value::as_u64), Some(1));
        assert_eq!(report.pointer("/cycle_metrics/0/service_floor_passes/denominator").and_then(Value::as_u64), Some(4));
        assert_eq!(report.pointer("/cycle_metrics/1/service_floor_passes/numerator").and_then(Value::as_u64), Some(3));
        assert_eq!(report.pointer("/cycle_metrics/1/service_floor_passes/denominator").and_then(Value::as_u64), Some(4));
        assert_eq!(report.pointer("/improvement_system/verified_actions/numerator").and_then(Value::as_u64), Some(2));
        assert_eq!(report.pointer("/improvement_system/verified_actions/denominator").and_then(Value::as_u64), Some(3));
        assert_eq!(report.pointer("/improvement_system/retested_failure_recurrence/numerator").and_then(Value::as_u64), Some(1));
        assert_eq!(report.pointer("/improvement_system/retested_failure_recurrence/denominator").and_then(Value::as_u64), Some(3));
        assert_eq!(report.pointer("/improvement_system/all_critical_service_floors_pass_at_retest").and_then(Value::as_bool), Some(false));
        assert!(report.get("total_score").is_none());
        assert!(report.get("composite_score").is_none());
    }

    #[test]
    fn rejects_budget_overrun() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["cycles"][0]["resources"]["planning_budget_committed"] = json!(1001);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_service_floor_drift() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["cycles"][1]["service_floor_observations"][3]["required_floor_percent"] = json!(70);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_unverified_action_closure() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["corrective_actions"][0]["completion_evidence_id"] = Value::Null;
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_composite_score_injection() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["composite_score"] = json!(99);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_missing_observation_evidence() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["cycles"][0]["objective_observations"][0]["evidence_ids"] = json!(["MISSING-EVIDENCE"]);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_completion_evidence_without_independent_review() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["cycles"][1]["evidence_records"][4]["independently_reviewed"] = json!(false);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }

    #[test]
    fn rejects_action_closure_when_linked_service_floor_fails() {
        let (nres002, mut loop_fixture) = fixtures();
        loop_fixture["cycles"][1]["service_floor_observations"][0]["observed_percent"] = json!(60);
        assert!(evaluate(&nres002, &loop_fixture).is_err());
    }
}
