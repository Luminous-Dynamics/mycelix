//! Independent, effects-disabled evaluator for the South African payments profile.
//!
//! This tool validates the JSON Schema, validates the corpus against Draft 2020-12,
//! and applies profile-specific semantic invariants. It never calls a rail, provider,
//! bank, credential issuer, or external endpoint and cannot initiate an effect.

use jsonschema::draft202012;
use serde_json::{json, Value};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/south-african-payment-interop-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/south-african-payment-interop-v1.schema.json");

const EXPECTED_SCHEMA: &str = "mycelix.sa-payments-interop/v1";
const EXPECTED_CORPUS_ID: &str = "ZA-SA-PAYMENTS-SYNTHETIC-V1";

#[derive(Debug, Clone)]
struct Evaluation {
    errors: Vec<String>,
    incomplete: Vec<String>,
    scenario_count: usize,
    system_count: usize,
    source_count: usize,
    milestone_count: usize,
    allocation_count: usize,
}

impl Evaluation {
    fn new() -> Self {
        Self {
            errors: Vec::new(),
            incomplete: Vec::new(),
            scenario_count: 0,
            system_count: 0,
            source_count: 0,
            milestone_count: 0,
            allocation_count: 0,
        }
    }

    fn status(&self) -> &'static str {
        if !self.errors.is_empty() {
            "FAIL"
        } else if !self.incomplete.is_empty() {
            "INCOMPLETE"
        } else {
            "PASS"
        }
    }
}

fn string_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn is_leap_year(year: i32) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

fn valid_date(value: &str) -> bool {
    let mut parts = value.split('-');
    let Some(year) = parts.next().and_then(|v| v.parse::<i32>().ok()) else {
        return false;
    };
    let Some(month) = parts.next().and_then(|v| v.parse::<u32>().ok()) else {
        return false;
    };
    let Some(day) = parts.next().and_then(|v| v.parse::<u32>().ok()) else {
        return false;
    };
    if parts.next().is_some() || year < 1970 || !(1..=12).contains(&month) {
        return false;
    }
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => return false,
    };
    (1..=days).contains(&day)
}

fn date_le(left: &str, right: &str) -> bool {
    // YYYY-MM-DD is lexicographically ordered when both inputs are valid dates.
    valid_date(left) && valid_date(right) && left <= right
}

fn expected_for_category(category: &str) -> Option<(&'static str, &'static str)> {
    match category {
        "current_future_scope"
        | "prefunding_scope"
        | "rail_identity"
        | "provider_profile_missing"
        | "credential_issuer"
        | "credential_scope"
        | "credential_currentness"
        | "qr_alias_substitution"
        | "qr_interoperability_not_rail" => Some(("reject_before_dispatch", "deny")),

        "participant_eligibility"
        | "credential_authority_separation"
        | "activity_authorisation_not_banking_licence" => {
            Some(("require_authority_evidence", "deny"))
        }

        "cross_border_fx" => Some(("quarantine_or_reject", "deny")),
        "regional_settlement" | "status_finality" => {
            Some(("observe_without_finality", "hold_for_evidence"))
        }
        "iso_profile_drift" => {
            Some(("profile_requalification_required", "deny"))
        }
        "idempotency_exact_retry" => {
            Some(("resolve_existing_operation", "eligible_only_under_profile"))
        }
        "idempotency_conflict" => Some(("quarantine_or_reject", "deny")),
        "unknown_external_outcome" | "dependency_resilience" => {
            Some(("indeterminate_no_blind_retry", "hold_for_evidence"))
        }
        "return_reversal_lineage" => {
            Some(("append_successor_event", "hold_for_evidence"))
        }
        "privacy_minimisation" | "payment_data_purpose_limitation" => {
            Some(("privacy_reject_or_redact", "deny"))
        }
        "governance_transition_scope"
        | "announced_target_not_completion"
        | "regional_milestone_not_routing_rule"
        | "rail_adoption_metrics_not_service_guarantee" => {
            Some(("profile_requalification_required", "deny"))
        }
        _ => None,
    }
}

fn check_unique_ids(
    values: &[Value],
    key: &str,
    path: &str,
    errors: &mut Vec<String>,
) {
    let mut seen = BTreeSet::new();
    for (index, item) in values.iter().enumerate() {
        match string_at(item, key) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{key}: duplicate id {id}")),
            None => errors.push(format!("{path}[{index}].{key}: missing string id")),
        }
    }
}

fn semantic_evaluation(corpus: &Value) -> Evaluation {
    let mut result = Evaluation::new();

    let Some(as_of) = string_at(corpus, "as_of") else {
        result.errors.push("as_of: missing pinned evaluation date".into());
        return result;
    };
    if !valid_date(as_of) {
        result.errors.push(format!("as_of: invalid date {as_of}"));
    }

    if string_at(corpus, "schema") != Some(EXPECTED_SCHEMA) {
        result.errors.push("schema: unexpected profile identifier".into());
    }
    if string_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS_ID) {
        result.errors.push("corpus_id: unexpected corpus identity".into());
    }
    if string_at(corpus, "status") != Some("synthetic_reference_only") {
        result.errors.push("status: corpus must remain synthetic_reference_only".into());
    }
    if corpus.get("jurisdiction").and_then(Value::as_str) != Some("ZA") {
        result.errors.push("jurisdiction: expected ZA".into());
    }
    if corpus.get("external_effects_enabled").and_then(Value::as_bool) != Some(false) {
        result.errors.push("external_effects_enabled: must be false".into());
    }

    let sources = array_at(corpus, "source_register").cloned().unwrap_or_default();
    let systems = array_at(corpus, "systems").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();
    let milestones = array_at(corpus, "milestones").cloned().unwrap_or_default();
    let allocations = array_at(corpus, "function_allocation").cloned().unwrap_or_default();

    result.source_count = sources.len();
    result.system_count = systems.len();
    result.scenario_count = scenarios.len();
    result.milestone_count = milestones.len();
    result.allocation_count = allocations.len();

    if sources.is_empty() {
        result.errors.push("source_register: empty source register".into());
    }
    if systems.is_empty() {
        result.errors.push("systems: empty system register".into());
    }
    if scenarios.is_empty() {
        result.errors.push("scenarios: empty adversarial corpus".into());
    }

    check_unique_ids(&sources, "id", "source_register", &mut result.errors);
    check_unique_ids(&systems, "system_id", "systems", &mut result.errors);
    check_unique_ids(&scenarios, "id", "scenarios", &mut result.errors);
    check_unique_ids(&milestones, "id", "milestones", &mut result.errors);
    check_unique_ids(&allocations, "id", "function_allocation", &mut result.errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| string_at(source, "id").map(str::to_owned))
        .collect();

    for (index, source) in sources.iter().enumerate() {
        if let Some(observed) = string_at(source, "observed_on") {
            if !valid_date(observed) || !date_le(observed, as_of) {
                result.errors.push(format!(
                    "source_register[{index}].observed_on: invalid or after corpus as_of"
                ));
            }
        } else {
            result.errors.push(format!(
                "source_register[{index}].observed_on: missing observation date"
            ));
        }
        if let Some(published) = string_at(source, "published_on") {
            if !valid_date(published) || !date_le(published, as_of) {
                result.errors.push(format!(
                    "source_register[{index}].published_on: invalid or after corpus as_of"
                ));
            }
        }
    }

    for (index, system) in systems.iter().enumerate() {
        let Some(system_id) = string_at(system, "system_id") else {
            continue;
        };
        let refs = array_at(system, "evidence_refs").cloned().unwrap_or_default();
        if refs.is_empty() {
            result.errors.push(format!("systems[{index}]: no evidence_refs"));
        }
        for reference in refs.iter().filter_map(Value::as_str) {
            if !source_ids.contains(reference) {
                result.errors.push(format!(
                    "systems[{index}]: unresolved source reference {reference}"
                ));
            }
        }

        // These profile-specific controls prevent accidental promotion of programme
        // announcements into currently executable systems. Any change requires a
        // new dated source assertion and a reviewed evaluator update.
        let required_state = match system_id {
            "ZA-FUTURE-FPS" | "ZA-FUTURE-RTGS" => Some("planned_or_in_development"),
            "ZA-PEMKEY-PCE" => Some("programme_enabler"),
            "ZA-PAYINC-NPU" => Some("transitioning"),
            _ => None,
        };
        if let Some(required) = required_state {
            if string_at(system, "lifecycle_state") != Some(required) {
                result.errors.push(format!(
                    "systems[{index}]: {system_id} must remain {required} until new reviewed source evidence is admitted"
                ));
            }
        }
    }

    for (index, allocation) in allocations.iter().enumerate() {
        let Some(effective_on) = string_at(allocation, "effective_on") else {
            result.errors.push(format!("function_allocation[{index}]: missing effective_on"));
            continue;
        };
        if !valid_date(effective_on) || !date_le(effective_on, as_of) {
            result.errors.push(format!(
                "function_allocation[{index}]: effective_on is invalid or in the future"
            ));
        }
        for reference in array_at(allocation, "source_refs")
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
        {
            if !source_ids.contains(reference) {
                result.errors.push(format!(
                    "function_allocation[{index}]: unresolved source reference {reference}"
                ));
            }
        }
    }

    for (index, milestone) in milestones.iter().enumerate() {
        for reference in array_at(milestone, "source_refs")
            .into_iter()
            .flatten()
            .filter_map(Value::as_str)
        {
            if !source_ids.contains(reference) {
                result.errors.push(format!(
                    "milestones[{index}]: unresolved source reference {reference}"
                ));
            }
        }

        let Some(observed) = string_at(milestone, "observed_on") else {
            result.errors.push(format!("milestones[{index}]: missing observed_on"));
            continue;
        };
        if !valid_date(observed) || !date_le(observed, as_of) {
            result.errors.push(format!(
                "milestones[{index}]: observed_on is invalid or after as_of"
            ));
        }

        match string_at(milestone, "status") {
            Some("completed_at_source") => {
                if string_at(milestone, "effective_from").is_none()
                    || string_at(milestone, "effective_by").is_none()
                    || string_at(milestone, "outcome").is_none()
                {
                    result.errors.push(format!(
                        "milestones[{index}]: completed_at_source requires effective dates and a sourced outcome"
                    ));
                }
            }
            Some("announced_target") => {
                let start = string_at(milestone, "target_start");
                let end = string_at(milestone, "target_end");
                if start.is_none() || end.is_none() {
                    result.errors.push(format!(
                        "milestones[{index}]: announced_target requires a bounded target window"
                    ));
                    continue;
                }
                let (start, end) = (start.unwrap(), end.unwrap());
                if !valid_date(start) || !valid_date(end) || start > end {
                    result.errors.push(format!(
                        "milestones[{index}]: invalid target date interval"
                    ));
                } else if end <= as_of {
                    result.incomplete.push(format!(
                        "milestones[{index}]: announced target window elapsed; refresh against official completion evidence"
                    ));
                }
                if string_at(milestone, "outcome").is_some()
                    || string_at(milestone, "effective_by").is_some()
                {
                    result.errors.push(format!(
                        "milestones[{index}]: announced_target cannot contain completion fields"
                    ));
                }
            }
            _ => result.errors.push(format!(
                "milestones[{index}]: unknown or missing milestone status"
            )),
        }
    }

    for (index, scenario) in scenarios.iter().enumerate() {
        let Some(category) = string_at(scenario, "category") else {
            result.errors.push(format!("scenarios[{index}]: missing category"));
            continue;
        };
        let Some((expected_disposition, expected_dispatch)) = expected_for_category(category) else {
            result.errors.push(format!(
                "scenarios[{index}]: unknown category {category}; evaluator update required"
            ));
            continue;
        };
        if string_at(scenario, "expected_disposition") != Some(expected_disposition) {
            result.errors.push(format!(
                "scenarios[{index}]: {category} requires disposition {expected_disposition}"
            ));
        }
        if string_at(scenario, "dispatch_decision") != Some(expected_dispatch) {
            result.errors.push(format!(
                "scenarios[{index}]: {category} requires dispatch_decision {expected_dispatch}"
            ));
        }
        if array_at(scenario, "negative_controls").is_none_or(Vec::is_empty) {
            result.errors.push(format!(
                "scenarios[{index}]: negative_controls must be non-empty"
            ));
        }
        if array_at(scenario, "required_evidence").is_none_or(Vec::is_empty) {
            result.errors.push(format!(
                "scenarios[{index}]: required_evidence must be non-empty"
            ));
        }
    }

    result
}

fn schema_evaluation(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();

    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the JSON Schema meta-schema".into());
        return errors;
    }

    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => {
            errors.push(format!("cannot compile Draft 2020-12 schema: {error}"));
            return errors;
        }
    };

    for error in validator.iter_errors(corpus) {
        errors.push(format!(
            "schema validation error at {}: {error}",
            error.instance_path()
        ));
    }
    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Evaluation, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;

    let mut evaluation = semantic_evaluation(&corpus);
    evaluation.errors.extend(schema_evaluation(&schema, &corpus));
    Ok((evaluation, corpus, schema))
}

fn mutate_and_expect_failure(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let semantic = semantic_evaluation(&changed);
    let schema_errors = schema_evaluation(schema, &changed);
    if semantic.errors.is_empty() && schema_errors.is_empty() && semantic.incomplete.is_empty() {
        return Err(format!("self-test mutation was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let baseline = semantic_evaluation(corpus);
    let baseline_schema_errors = schema_evaluation(schema, corpus);
    if !baseline.errors.is_empty() || !baseline.incomplete.is_empty() || !baseline_schema_errors.is_empty() {
        return Err("self-test baseline corpus must validate before mutations".into());
    }

    let mut count = 0usize;

    mutate_and_expect_failure("external effects enabled", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_expect_failure("duplicate scenario identity", corpus, schema, |value| {
        let first = value["scenarios"][0]["id"].clone();
        value["scenarios"][1]["id"] = first;
    })?;
    count += 1;

    mutate_and_expect_failure("missing system source", corpus, schema, |value| {
        value["systems"][0]["evidence_refs"] = json!(["MISSING_SOURCE"]);
    })?;
    count += 1;

    mutate_and_expect_failure("future capability promoted to current", corpus, schema, |value| {
        value["systems"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|system| string_at(system, "system_id") == Some("ZA-FUTURE-FPS"))
            .unwrap()["lifecycle_state"] = json!("current");
    })?;
    count += 1;

    mutate_and_expect_failure("announced target forged as complete", corpus, schema, |value| {
        let milestone = value["milestones"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|milestone| string_at(milestone, "status") == Some("announced_target"))
            .unwrap();
        milestone["status"] = json!("completed_at_source");
    })?;
    count += 1;

    mutate_and_expect_failure("accepted status upgraded to finality", corpus, schema, |value| {
        let scenario = value["scenarios"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|scenario| string_at(scenario, "category") == Some("status_finality"))
            .unwrap();
        scenario["expected_disposition"] = json!("resolve_existing_operation");
        scenario["dispatch_decision"] = json!("eligible_only_under_profile");
    })?;
    count += 1;

    mutate_and_expect_failure("QR+ treated as settlement rail", corpus, schema, |value| {
        let scenario = value["scenarios"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|scenario| string_at(scenario, "category") == Some("qr_interoperability_not_rail"))
            .unwrap();
        scenario["expected_disposition"] = json!("observe_without_finality");
        scenario["dispatch_decision"] = json!("hold_for_evidence");
    })?;
    count += 1;

    mutate_and_expect_failure("payment history silently becomes credit score", corpus, schema, |value| {
        let scenario = value["scenarios"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|scenario| string_at(scenario, "category") == Some("payment_data_purpose_limitation"))
            .unwrap();
        scenario["expected_disposition"] = json!("resolve_existing_operation");
        scenario["dispatch_decision"] = json!("eligible_only_under_profile");
    })?;
    count += 1;

    mutate_and_expect_failure("regional amount target used as routing rule", corpus, schema, |value| {
        let scenario = value["scenarios"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|scenario| string_at(scenario, "category") == Some("regional_milestone_not_routing_rule"))
            .unwrap();
        scenario["expected_disposition"] = json!("resolve_existing_operation");
        scenario["dispatch_decision"] = json!("eligible_only_under_profile");
    })?;
    count += 1;

    mutate_and_expect_failure("unknown scenario category", corpus, schema, |value| {
        value["scenarios"][0]["category"] = json!("unknown_new_semantics");
    })?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut self_test = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => {
                let path = PathBuf::from(args.next().ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "--corpus requires a path"))?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| std::io::Error::new(std::io::ErrorKind::InvalidInput, "--schema requires a path"))?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("sa-payments-profile-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
                return Ok(());
            }
            other => return Err(std::io::Error::new(std::io::ErrorKind::InvalidInput, format!("unknown argument: {other}")).into()),
        }
    }

    let (evaluation, corpus, schema) = evaluate(&corpus_text, &schema_text)\n        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let mutation_count = if self_test {
        run_self_tests(&corpus, &schema)\n            .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?
    } else {
        0
    };

    let report = json!({
        "evaluator": "mycelix.sa-payments-profile-evaluator.v1",
        "status": evaluation.status(),
        "claim_ceiling": "synthetic profile, source-reference, schema, temporal and scenario-contract validation only",
        "external_effects_enabled": false,
        "counts": {
            "sources": evaluation.source_count,
            "systems": evaluation.system_count,
            "scenarios": evaluation.scenario_count,
            "milestones": evaluation.milestone_count,
            "function_allocations": evaluation.allocation_count,
            "negative_mutation_tests": mutation_count
        },
        "errors": evaluation.errors,
        "incomplete": evaluation.incomplete
    });
    println!("{}", serde_json::to_string_pretty(&report)?);

    if evaluation.status() != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_in_corpus_and_schema_validate() {
        let (evaluation, _, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("parse corpus");
        assert_eq!(evaluation.status(), "PASS", "{:?}", evaluation);
    }

    #[test]
    fn adversarial_mutations_are_rejected() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("parse corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("parse schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 10);
    }

    #[test]
    fn invalid_calendar_dates_are_rejected() {
        assert!(!valid_date("2026-02-29"));
        assert!(valid_date("2024-02-29"));
        assert!(!valid_date("2027-13-01"));
    }

    #[test]
    fn dates_are_compared_without_runtime_clock() {
        assert!(date_le("2026-09-02", "2026-11-03"));
        assert!(!date_le("2027-01-01", "2026-11-03"));
    }
}
