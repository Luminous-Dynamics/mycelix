#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

type AppResult<T> = Result<T, String>;

#[derive(Debug)]
struct PairAssessment {
    pair_id: String,
    state: String,
    reasons: BTreeSet<String>,
}

#[derive(Debug)]
struct Assessment {
    state: String,
    reasons: BTreeSet<String>,
    pair_results: Vec<PairAssessment>,
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
                    let expected_pairs = reference["expected_pair_states"].as_array().cloned().unwrap_or_default();
                    let mut pair_checks = Vec::new();
                    let mut pair_match = assessment.pair_results.len() == expected_pairs.len();
                    for pair in &assessment.pair_results {
                        let expected_pair = expected_pairs.iter().find(|row| row["pair_id"] == pair.pair_id);
                        let expected_state = expected_pair.and_then(|row| row["terminal_state"].as_str()).unwrap_or("<missing>");
                        let pair_blocks = pair.state != "addressed_with_qualified_evidence"
                            && pair.state != "not_applicable_with_authorized_rationale";
                        let pair_passed = pair.state == expected_state;
                        pair_match &= pair_passed;
                        pair_checks.push(json!({
                            "pair_id": pair.pair_id,
                            "actual_terminal_state": pair.state,
                            "expected_terminal_state": expected_state,
                            "passed": pair_passed,
                            "blocks_unqualified_signoff": pair_blocks,
                            "reason_codes": pair.reasons.iter().cloned().collect::<Vec<_>>()
                        }));
                    }
                    let passed = assessment.state == expected && blocks == expected_blocks && pair_match;
                    results.push(json!({
                        "case_id": id,
                        "actual_terminal_state": assessment.state,
                        "expected_terminal_state": expected,
                        "passed": passed,
                        "blocks_unqualified_signoff": blocks,
                        "expected_blocks_unqualified_signoff": expected_blocks,
                        "pair_results": pair_checks,
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

    let known_pair_ids = required_pairs
        .iter()
        .filter_map(|pair| string(pair, "pair_id"))
        .collect::<BTreeSet<_>>();
    for procedure in procedures {
        let pair_id = string(procedure, "pair_id").unwrap_or_default();
        if !known_pair_ids.contains(pair_id) {
            return Err(format!("procedure references unknown risk/assertion pair: {pair_id}"));
        }
    }

    let mut pair_results = Vec::<PairAssessment>::new();
    for pair in required_pairs {
        let pair_id = string(pair, "pair_id").ok_or_else(|| "pair missing pair_id".to_owned())?;
        let required = pair.get("required").and_then(Value::as_bool).unwrap_or(false);
        let assertion = string(pair, "assertion").unwrap_or_default();
        let purpose = string(pair, "purpose").unwrap_or_default();
        let min_origins = pair.get("min_independent_origins").and_then(Value::as_u64).unwrap_or(1) as usize;
        let mut reasons = BTreeSet::<String>::new();

        let related = procedures
            .iter()
            .filter(|procedure| string(procedure, "pair_id") == Some(pair_id))
            .collect::<Vec<_>>();

        if !required {
            let authorized_na = related.iter().any(|procedure| {
                string(procedure, "execution_status") == Some("not_applicable")
                    && procedure.get("reviewer_authorized").and_then(Value::as_bool) == Some(true)
                    && string(procedure, "authorized_na_rationale")
                        .map(|value| !value.trim().is_empty())
                        .unwrap_or(false)
            });
            let state = if authorized_na {
                reasons.insert("authorized_scoped_non_applicability".to_owned());
                "not_applicable_with_authorized_rationale"
            } else {
                reasons.insert("non_applicability_not_authorized".to_owned());
                "human_judgment_required"
            };
            pair_results.push(pair_assessment(pair_id, state, reasons));
            continue;
        }

        let procedure = match related
            .iter()
            .find(|procedure| string(procedure, "execution_status") != Some("not_applicable"))
        {
            Some(procedure) => *procedure,
            None => {
                reasons.insert("required_procedure_absent".to_owned());
                pair_results.push(pair_assessment(pair_id, "procedure_not_performed", reasons));
                continue;
            }
        };

        if string(procedure, "execution_status") != Some("performed") {
            reasons.insert("required_procedure_not_performed".to_owned());
            pair_results.push(pair_assessment(pair_id, "procedure_not_performed", reasons));
            continue;
        }
        if string(procedure, "procedure_purpose") != Some(purpose) {
            reasons.insert("performed_procedure_purpose_mismatch".to_owned());
            pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
            continue;
        }
        if procedure.get("requires_human_judgment").and_then(Value::as_bool) == Some(true) {
            reasons.insert("professional_judgment_required".to_owned());
            pair_results.push(pair_assessment(pair_id, "human_judgment_required", reasons));
            continue;
        }
        if procedure.get("stale_workpaper").and_then(Value::as_bool) == Some(true) {
            reasons.insert("workpaper_frontier_stale".to_owned());
            pair_results.push(pair_assessment(pair_id, "exception_requires_resolution", reasons));
            continue;
        }
        if procedure.get("control_failed").and_then(Value::as_bool) == Some(true)
            && procedure.get("compensating_procedure_present").and_then(Value::as_bool) != Some(true)
        {
            reasons.insert("control_failure_without_compensating_procedure".to_owned());
            pair_results.push(pair_assessment(pair_id, "procedure_not_performed", reasons));
            continue;
        }
        if procedure.get("exception_open").and_then(Value::as_bool) == Some(true) {
            reasons.insert("open_procedure_exception".to_owned());
            pair_results.push(pair_assessment(pair_id, "exception_requires_resolution", reasons));
            continue;
        }

        if let Some(population_id) = string(procedure, "population_id") {
            let population = populations
                .iter()
                .find(|population| string(population, "population_id") == Some(population_id));
            match population {
                None => {
                    reasons.insert("procedure_population_missing".to_owned());
                    pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
                    continue;
                }
                Some(population) => {
                    let expected = population
                        .get("expected_count")
                        .and_then(Value::as_u64)
                        .unwrap_or(u64::MAX);
                    let observed = population
                        .get("observed_count")
                        .and_then(Value::as_u64)
                        .unwrap_or(0);
                    let frame_complete =
                        population.get("frame_complete").and_then(Value::as_bool) == Some(true);
                    if observed < expected {
                        reasons.insert("expected_population_incomplete".to_owned());
                        pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
                        continue;
                    }
                    if !frame_complete {
                        reasons.insert("sample_frame_omits_class".to_owned());
                        pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
                        continue;
                    }
                }
            }
        }

        let expected_denominator = string(procedure, "expected_denominator");
        let used_denominator = string(procedure, "used_denominator");
        if expected_denominator.is_some() && expected_denominator != used_denominator {
            reasons.insert("wrong_denominator_even_when_output_matches".to_owned());
            pair_results.push(pair_assessment(pair_id, "exception_requires_resolution", reasons));
            continue;
        }
        if procedure.get("threshold_authorized").and_then(Value::as_bool) != Some(true) {
            reasons.insert("analytical_threshold_unapproved".to_owned());
            pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
            continue;
        }
        if string(procedure, "procedure_type") == Some("analytical") {
            match procedure.get("within_tolerance").and_then(Value::as_bool) {
                Some(true) => {}
                Some(false) => {
                    reasons.insert("analytical_difference_above_threshold".to_owned());
                    pair_results.push(pair_assessment(pair_id, "exception_requires_resolution", reasons));
                    continue;
                }
                None => {
                    reasons.insert("analytical_result_tolerance_not_recorded".to_owned());
                    pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
                    continue;
                }
            }
        }

        let evidence_refs = array(procedure, "evidence_refs");
        if evidence_refs.is_empty() {
            reasons.insert("procedure_has_no_evidence".to_owned());
            pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
            continue;
        }

        let mut origins = BTreeSet::<String>::new();
        let mut evidence_valid = true;
        let mut contradiction = false;
        for evidence_ref in evidence_refs {
            let evidence_id = evidence_ref
                .as_str()
                .ok_or_else(|| "evidence_refs must contain strings".to_owned())?;
            let item = evidence
                .iter()
                .find(|item| string(item, "evidence_id") == Some(evidence_id));
            let item = match item {
                Some(item) => item,
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
            if !array(item, "assertions_supported")
                .iter()
                .any(|value| value.as_str() == Some(assertion))
            {
                evidence_valid = false;
                reasons.insert("evidence_does_not_support_required_assertion".to_owned());
            }
            if !array(item, "purposes_supported")
                .iter()
                .any(|value| value.as_str() == Some(purpose))
            {
                evidence_valid = false;
                reasons.insert("evidence_does_not_support_intended_purpose".to_owned());
            }
            if string(item, "reporting_entity") != string(manifest, "reporting_entity") {
                evidence_valid = false;
                reasons.insert("evidence_reporting_entity_mismatch".to_owned());
            }
            if string(item, "reporting_period") != string(manifest, "reporting_period") {
                evidence_valid = false;
                reasons.insert("evidence_period_stale_or_mismatched".to_owned());
            }
            if let Some(origin) = string(item, "origin_group") {
                origins.insert(origin.to_owned());
            }
            contradiction |= item
                .get("contradicts_required_claim")
                .and_then(Value::as_bool)
                == Some(true);
        }
        if contradiction {
            reasons.insert("contradictory_evidence_unresolved".to_owned());
            pair_results.push(pair_assessment(pair_id, "exception_requires_resolution", reasons));
            continue;
        }
        if origins.len() < min_origins {
            evidence_valid = false;
            reasons.insert("independence_origin_not_distinct".to_owned());
        }
        if !evidence_valid {
            pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
            continue;
        }
        if string(procedure, "outcome") != Some("supports") {
            reasons.insert("procedure_result_does_not_support_assertion".to_owned());
            pair_results.push(pair_assessment(pair_id, "insufficient_evidence", reasons));
            continue;
        }
        reasons.insert("purpose_and_assertion_matched".to_owned());
        pair_results.push(pair_assessment(
            pair_id,
            "addressed_with_qualified_evidence",
            reasons,
        ));
    }

    let states = pair_results
        .iter()
        .map(|result| result.state.as_str())
        .collect::<Vec<_>>();
    let precedence = [
        "human_judgment_required",
        "exception_requires_resolution",
        "procedure_not_performed",
        "insufficient_evidence",
        "addressed_with_explicit_limitation",
    ];
    let mut state = if !states.is_empty()
        && states.iter().all(|value| *value == "not_applicable_with_authorized_rationale")
    {
        "not_applicable_with_authorized_rationale"
    } else {
        "addressed_with_qualified_evidence"
    };
    for candidate in precedence {
        if states.iter().any(|found| *found == candidate) {
            state = candidate;
            break;
        }
    }

    let mut all_reasons = BTreeSet::new();
    for result in &pair_results {
        all_reasons.extend(result.reasons.iter().cloned());
    }
    Ok(Assessment {
        state: state.to_owned(),
        reasons: all_reasons,
        pair_results,
    })
}

fn pair_assessment(pair_id: &str, state: &str, reasons: BTreeSet<String>) -> PairAssessment {
    PairAssessment {
        pair_id: pair_id.to_owned(),
        state: state.to_owned(),
        reasons,
    }
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

    fn parse(source: &str) -> Value {
        serde_json::from_str(source).expect("valid fixture JSON")
    }

    fn case_by_id<'a>(candidate: &'a Value, id: &str) -> &'a Value {
        candidate["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .find(|row| row["case_id"] == id)
            .expect("case exists")
    }

    #[test]
    fn all_public_reference_cases_match_case_and_pair_oracles() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let oracle = parse(include_str!("../../public-assurance-oracle/fin_assurance_procedure_oracle_v1.1.0.json"));
        for case in candidate["cases"].as_array().expect("cases") {
            let id = case["case_id"].as_str().expect("case ID");
            let reference = oracle["cases"]
                .as_array()
                .expect("oracle cases")
                .iter()
                .find(|row| row["case_id"] == id)
                .expect("oracle row");
            let assessment = assess_case(case).expect("assessment");
            assert_eq!(assessment.state, reference["expected_terminal_state"].as_str().expect("expected state"), "{id}");
            let expected_pairs = reference["expected_pair_states"].as_array().expect("pair oracle");
            assert_eq!(assessment.pair_results.len(), expected_pairs.len(), "{id} pair count");
            for pair in &assessment.pair_results {
                let expected = expected_pairs
                    .iter()
                    .find(|row| row["pair_id"] == pair.pair_id)
                    .expect("pair oracle row");
                assert_eq!(pair.state, expected["terminal_state"].as_str().expect("pair state"), "{id}/{}", pair.pair_id);
            }
        }
    }

    #[test]
    fn wrong_denominator_is_not_hidden_by_matching_output() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FAP-007");
        assert_eq!(assess_case(case).expect("assessment").state, "exception_requires_resolution");
    }

    #[test]
    fn same_upstream_origin_is_not_independent_corroboration() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FAP-010");
        assert_eq!(assess_case(case).expect("assessment").state, "insufficient_evidence");
    }

    #[test]
    fn non_applicability_requires_authorized_scoped_rationale() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FAP-016");
        assert_eq!(
            assess_case(case).expect("assessment").state,
            "not_applicable_with_authorized_rationale"
        );
    }

    #[test]
    fn a_late_source_stales_prior_workpaper() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let case = case_by_id(&candidate, "FAP-017");
        assert_eq!(assess_case(case).expect("assessment").state, "exception_requires_resolution");
    }

    #[test]
    fn multi_pair_case_preserves_each_pair_disposition() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let assessment = assess_case(case_by_id(&candidate, "FAP-019")).expect("assessment");
        assert_eq!(assessment.state, "insufficient_evidence");
        assert_eq!(assessment.pair_results.len(), 2);
        assert_eq!(
            assessment.pair_results.iter().find(|row| row.pair_id == "pair-1").unwrap().state,
            "addressed_with_qualified_evidence"
        );
        assert_eq!(
            assessment.pair_results.iter().find(|row| row.pair_id == "pair-2").unwrap().state,
            "insufficient_evidence"
        );
    }

    #[test]
    fn procedure_purpose_must_match_required_pair_purpose() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let assessment = assess_case(case_by_id(&candidate, "FAP-020")).expect("assessment");
        assert_eq!(assessment.state, "insufficient_evidence");
        assert!(assessment.reasons.contains("performed_procedure_purpose_mismatch"));
    }

    #[test]
    fn evidence_must_be_bound_to_the_reporting_entity() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let assessment = assess_case(case_by_id(&candidate, "FAP-021")).expect("assessment");
        assert_eq!(assessment.state, "insufficient_evidence");
        assert!(assessment.reasons.contains("evidence_reporting_entity_mismatch"));
    }

    #[test]
    fn analytical_differences_above_threshold_remain_open() {
        let candidate = parse(include_str!("../../fin_assurance_procedure_candidates_v1.1.0.json"));
        let assessment = assess_case(case_by_id(&candidate, "FAP-022")).expect("assessment");
        assert_eq!(assessment.state, "exception_requires_resolution");
        assert!(assessment.reasons.contains("analytical_difference_above_threshold"));
    }
}
