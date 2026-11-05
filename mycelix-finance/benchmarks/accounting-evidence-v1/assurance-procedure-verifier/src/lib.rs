#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

type AppResult<T> = Result<T, String>;

#[derive(Debug)]
struct Assessment {
    state: String,
    reasons: BTreeSet<String>,
}

pub fn verify(input_bytes: &[u8], schema_bytes: &[u8], oracle_bytes: &[u8], oracle_schema_bytes: &[u8]) -> Value {
    let mut errors = Vec::<String>::new();
    let input: Value = parse_json("candidate input", input_bytes, &mut errors);
    let schema: Value = parse_json("candidate schema", schema_bytes, &mut errors);
    let oracle: Value = parse_json("public oracle", oracle_bytes, &mut errors);
    let oracle_schema: Value = parse_json("oracle schema", oracle_schema_bytes, &mut errors);

    if errors.is_empty() {
        errors.extend(validate_instance("candidate input", &schema, &input));
        errors.extend(validate_instance("public oracle", &oracle_schema, &oracle));
    }

    let mut results = Vec::<Value>::new();
    if errors.is_empty() {
        let input_cases = input["cases"].as_array().cloned().unwrap_or_default();
        let oracle_cases = oracle["cases"].as_array().cloned().unwrap_or_default();
        let mut oracle_by_id = BTreeMap::<String, Value>::new();
        for row in &oracle_cases {
            let id = row["case_id"].as_str().unwrap_or("<missing>").to_owned();
            if oracle_by_id.insert(id.clone(), row.clone()).is_some() {
                errors.push(format!("duplicate oracle case_id: {id}"));
            }
        }
        let mut seen = BTreeSet::new();
        for case in &input_cases {
            let id = case["case_id"].as_str().unwrap_or("<missing>").to_owned();
            if !seen.insert(id.clone()) {
                errors.push(format!("duplicate candidate case_id: {id}"));
                continue;
            }
            match (oracle_by_id.get(&id), assess_case(case)) {
                (Some(reference), Ok(assessment)) => {
                    let expected = reference["expected_terminal_state"].as_str().unwrap_or("<missing>");
                    let blocks = assessment.state != "addressed_with_qualified_evidence"
                        && assessment.state != "not_applicable_with_authorized_rationale";
                    let expected_blocks = reference["blocks_unqualified_signoff"].as_bool().unwrap_or(true);
                    let passed = assessment.state == expected && blocks == expected_blocks;
                    results.push(json!({
                        "case_id": id,
                        "actual_terminal_state": assessment.state,
                        "expected_terminal_state": expected,
                        "passed": passed,
                        "blocks_unqualified_signoff": blocks,
                        "expected_blocks_unqualified_signoff": expected_blocks,
                        "reason_codes": assessment.reasons.into_iter().collect::<Vec<_>>()
                    }));
                }
                (None, _) => errors.push(format!("candidate case {id} has no public oracle row")),
                (_, Err(error)) => errors.push(format!("case {id}: {error}")),
            }
        }
        for id in oracle_by_id.keys() {
            if !seen.contains(id) {
                errors.push(format!("oracle case {id} has no candidate input"));
            }
        }
    }

    let passed = results.iter().filter(|row| row["passed"] == true).count();
    let state = if errors.is_empty() && !results.is_empty() && passed == results.len() { "PASS" } else { "FAIL" };
    let report = json!({
        "report_schema": "fin-assurance-procedure-report.v1",
        "benchmark_id": input.get("benchmark_id"),
        "dataset_version": input.get("dataset_version"),
        "status": state,
        "claim_scope": "public_synthetic_procedure_coverage_and_stand_back_only_not_audit_assurance",
        "digests": {
            "candidate_input_sha256": sha256_hex(input_bytes),
            "candidate_schema_sha256": sha256_hex(schema_bytes),
            "public_oracle_sha256": sha256_hex(oracle_bytes),
            "oracle_schema_sha256": sha256_hex(oracle_schema_bytes),
            "verifier_source_sha256": sha256_hex(include_bytes!("lib.rs"))
        },
        "case_count": results.len(),
        "passed_case_count": passed,
        "errors": errors,
        "cases": results
    });
    let report_schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(schema) => schema,
        Err(error) => return fail_report(report, format!("embedded report schema parse: {error}")),
    };
    let report_errors = validate_instance("generated report", &report_schema, &report);
    if report_errors.is_empty() {
        report
    } else {
        fail_report(report, report_errors.join("; "))
    }
}

fn fail_report(mut report: Value, error: String) -> Value {
    report["status"] = Value::String("FAIL".to_owned());
    if let Some(errors) = report["errors"].as_array_mut() {
        errors.push(Value::String(error));
    }
    report
}

fn parse_json(label: &str, bytes: &[u8], errors: &mut Vec<String>) -> Value {
    match serde_json::from_slice(bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{label} JSON: {error}"));
            Value::Null
        }
    }
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options().should_validate_formats(true).build(schema) {
        Ok(validator) => validator,
        Err(error) => return vec![format!("{label}: could not compile schema: {error}")],
    };
    validator.iter_errors(instance).map(|error| format!("{label}: {} at {}", error, error.instance_path())).collect()
}

fn assess_case(case: &Value) -> AppResult<Assessment> {
    let manifest = case.get("manifest").ok_or_else(|| "missing manifest".to_owned())?;
    let required_pairs = array(manifest, "required_pairs");
    let procedures = array(manifest, "procedures");
    let populations = array(manifest, "populations");
    let evidence = array(manifest, "evidence");
    if required_pairs.is_empty() {
        return Err("required_pairs must not be empty".to_owned());
    }
    let mut results = Vec::<(String, BTreeSet<String>)>::new();
    for pair in required_pairs {
        let pair_id = string(pair, "pair_id").ok_or_else(|| "pair missing pair_id".to_owned())?;
        let required = pair.get("required").and_then(Value::as_bool).unwrap_or(false);
        let assertion = string(pair, "assertion").unwrap_or_default();
        let purpose = string(pair, "purpose").unwrap_or_default();
        let min_origins = pair.get("min_independent_origins").and_then(Value::as_u64).unwrap_or(1) as usize;
        let mut reasons = BTreeSet::<String>::new();

        let related = procedures.iter().filter(|procedure| string(procedure, "pair_id") == Some(pair_id)).collect::<Vec<_>>();
        if !required {
            let na = related.iter().any(|procedure| {
                string(procedure, "execution_status") == Some("not_applicable")
                    && procedure.get("reviewer_authorized").and_then(Value::as_bool) == Some(true)
                    && string(procedure, "authorized_na_rationale").map(|s| !s.trim().is_empty()).unwrap_or(false)
            });
            let state = if na {
                reasons.insert("authorized_scoped_non_applicability".to_owned());
                "not_applicable_with_authorized_rationale"
            } else {
                reasons.insert("non_applicability_not_authorized".to_owned());
                "human_judgment_required"
            };
            results.push((state.to_owned(), reasons));
            continue;
        }

        let procedure = match related.iter().find(|procedure| string(procedure, "execution_status") != Some("not_applicable")) {
            Some(procedure) => *procedure,
            None => {
                reasons.insert("required_procedure_absent".to_owned());
                results.push(("procedure_not_performed".to_owned(), reasons));
                continue;
            }
        };
        if string(procedure, "execution_status") != Some("performed") {
            reasons.insert("required_procedure_not_performed".to_owned());
            results.push(("procedure_not_performed".to_owned(), reasons));
            continue;
        }
        if procedure.get("requires_human_judgment").and_then(Value::as_bool) == Some(true) {
            reasons.insert("professional_judgment_required".to_owned());
            results.push(("human_judgment_required".to_owned(), reasons));
            continue;
        }
        if procedure.get("stale_workpaper").and_then(Value::as_bool) == Some(true) {
            reasons.insert("workpaper_frontier_stale".to_owned());
            results.push(("exception_requires_resolution".to_owned(), reasons));
            continue;
        }
        if procedure.get("control_failed").and_then(Value::as_bool) == Some(true)
            && procedure.get("compensating_procedure_present").and_then(Value::as_bool) != Some(true) {
            reasons.insert("control_failure_without_compensating_procedure".to_owned());
            results.push(("procedure_not_performed".to_owned(), reasons));
            continue;
        }
        if procedure.get("exception_open").and_then(Value::as_bool) == Some(true) {
            reasons.insert("open_procedure_exception".to_owned());
            results.push(("exception_requires_resolution".to_owned(), reasons));
            continue;
        }

        if let Some(pop_id) = string(procedure, "population_id") {
            let population = populations.iter().find(|population| string(population, "population_id") == Some(pop_id));
            match population {
                None => {
                    reasons.insert("procedure_population_missing".to_owned());
                    results.push(("insufficient_evidence".to_owned(), reasons));
                    continue;
                }
                Some(population) => {
                    let expected = population.get("expected_count").and_then(Value::as_u64).unwrap_or(u64::MAX);
                    let observed = population.get("observed_count").and_then(Value::as_u64).unwrap_or(0);
                    let frame_complete = population.get("frame_complete").and_then(Value::as_bool) == Some(true);
                    if observed < expected {
                        reasons.insert("expected_population_incomplete".to_owned());
                        results.push(("insufficient_evidence".to_owned(), reasons));
                        continue;
                    }
                    if !frame_complete {
                        reasons.insert("sample_frame_omits_class".to_owned());
                        results.push(("insufficient_evidence".to_owned(), reasons));
                        continue;
                    }
                }
            }
        }

        let expected_denominator = string(procedure, "expected_denominator");
        let used_denominator = string(procedure, "used_denominator");
        if expected_denominator.is_some() && expected_denominator != used_denominator {
            reasons.insert("wrong_denominator_even_when_output_matches".to_owned());
            results.push(("exception_requires_resolution".to_owned(), reasons));
            continue;
        }
        if procedure.get("threshold_authorized").and_then(Value::as_bool) != Some(true) {
            reasons.insert("analytical_threshold_unapproved".to_owned());
            results.push(("insufficient_evidence".to_owned(), reasons));
            continue;
        }

        let refs = array(procedure, "evidence_refs");
        if refs.is_empty() {
            reasons.insert("procedure_has_no_evidence".to_owned());
            results.push(("insufficient_evidence".to_owned(), reasons));
            continue;
        }
        let mut origins = BTreeSet::<String>::new();
        let mut evidence_valid = true;
        let mut contradiction = false;
        for evidence_ref in refs {
            let evidence_id = evidence_ref.as_str().ok_or_else(|| "evidence_refs must contain strings".to_owned())?;
            let item = evidence.iter().find(|item| string(item, "evidence_id") == Some(evidence_id));
            let item = match item {
                Some(item) => *item,
                None => {
                    evidence_valid = false;
                    reasons.insert("evidence_reference_missing".to_owned());
                    continue;
                }
            };
            if item.get("reliability_status").and_then(Value::as_str) != Some("qualified") {
                evidence_valid = false;
                reasons.insert("evidence_reliability_not_qualified".to_owned());
            }
            if !array(item, "assertions_supported").iter().any(|value| value.as_str() == Some(assertion)) {
                evidence_valid = false;
                reasons.insert("evidence_does_not_support_required_assertion".to_owned());
            }
            if !array(item, "purposes_supported").iter().any(|value| value.as_str() == Some(purpose)) {
                evidence_valid = false;
                reasons.insert("evidence_does_not_support_intended_purpose".to_owned());
            }
            if string(item, "reporting_period") != string(manifest, "reporting_period") {
                evidence_valid = false;
                reasons.insert("evidence_period_stale_or_mismatched".to_owned());
            }
            if let Some(origin) = string(item, "origin_group") {
                origins.insert(origin.to_owned());
            }
            contradiction |= item.get("contradicts_required_claim").and_then(Value::as_bool) == Some(true);
        }
        if contradiction {
            reasons.insert("contradictory_evidence_unresolved".to_owned());
            results.push(("exception_requires_resolution".to_owned(), reasons));
            continue;
        }
        if origins.len() < min_origins {
            evidence_valid = false;
            reasons.insert("independence_origin_not_distinct".to_owned());
        }
        if !evidence_valid {
            results.push(("insufficient_evidence".to_owned(), reasons));
            continue;
        }
        if string(procedure, "outcome") != Some("supports") {
            reasons.insert("procedure_result_does_not_support_assertion".to_owned());
            results.push(("insufficient_evidence".to_owned(), reasons));
            continue;
        }
        reasons.insert("purpose_and_assertion_matched".to_owned());
        results.push(("addressed_with_qualified_evidence".to_owned(), reasons));
    }

    // A stand-back result is blocked when any required risk/assertion pair is unresolved.
    let precedence = [
        "human_judgment_required",
        "exception_requires_resolution",
        "procedure_not_performed",
        "insufficient_evidence",
    ];
    let mut states = results.iter().map(|(state, _)| state.clone()).collect::<Vec<_>>();
    states.sort();
    let mut state = "addressed_with_qualified_evidence";
    for candidate in precedence {
        if states.iter().any(|found| found.as_str() == candidate) {
            state = candidate;
            break;
        }
    }
    let mut all_reasons = BTreeSet::new();
    for (_, reasons) in results { all_reasons.extend(reasons); }
    Ok(Assessment { state: state.to_owned(), reasons: all_reasons })
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}
fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}
pub fn sha256_hex(bytes: &[u8]) -> String { hex::encode(Sha256::digest(bytes)) }

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;
    fn parse(source: &str) -> Value { serde_json::from_str(source).expect("valid fixture JSON") }

    #[test]
    fn all_public_reference_cases_match_the_oracle() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.0.0.json"));
        let oracle = parse(include_str!("../../public-assurance-oracle/fin_assurance_procedure_oracle_v1.0.0.json"));
        for case in candidate["cases"].as_array().expect("cases") {
            let id = case["case_id"].as_str().expect("case ID");
            let expected = oracle["cases"].as_array().expect("oracle cases").iter()
                .find(|row| row["case_id"] == id).expect("oracle row")["expected_terminal_state"].as_str().expect("state");
            assert_eq!(assess_case(case).expect("assessment").state, expected, "{id}");
        }
    }

    #[test]
    fn wrong_denominator_is_not_hidden_by_matching_output() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.0.0.json"));
        let case = candidate["cases"].as_array().expect("cases").iter().find(|row| row["case_id"] == "FAP-007").expect("case");
        assert_eq!(assess_case(case).expect("assessment").state, "exception_requires_resolution");
    }

    #[test]
    fn same_upstream_origin_is_not_independent_corroboration() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.0.0.json"));
        let case = candidate["cases"].as_array().expect("cases").iter().find(|row| row["case_id"] == "FAP-010").expect("case");
        assert_eq!(assess_case(case).expect("assessment").state, "insufficient_evidence");
    }

    #[test]
    fn non_applicability_requires_authorized_scoped_rationale() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.0.0.json"));
        let case = candidate["cases"].as_array().expect("cases").iter().find(|row| row["case_id"] == "FAP-016").expect("case");
        assert_eq!(assess_case(case).expect("assessment").state, "not_applicable_with_authorized_rationale");
    }

    #[test]
    fn a_late_source_stales_prior_workpaper() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.0.0.json"));
        let case = candidate["cases"].as_array().expect("cases").iter().find(|row| row["case_id"] == "FAP-017").expect("case");
        assert_eq!(assess_case(case).expect("assessment").state, "exception_requires_resolution");
    }
}
