use serde_json::{json, Value};
use std::{collections::{BTreeMap, BTreeSet}, env, fs, process};

fn array<'a>(v: &'a Value, key: &str) -> &'a [Value] {
    v.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

fn text<'a>(v: &'a Value, key: &str) -> Option<&'a str> {
    v.get(key).and_then(Value::as_str)
}

fn count(v: &Value, key: &str) -> u64 {
    v.get(key).and_then(Value::as_u64).unwrap_or(0)
}

fn is_allow(value: &str) -> bool {
    matches!(value, "ALLOW_SCOPED" | "ALLOW_PARTIAL")
}

fn is_block(value: &str) -> bool {
    matches!(value, "BLOCK" | "HOLD_FOR_RECONCILIATION")
}

fn disposition_map(rows: &[Value]) -> (BTreeMap<(String, String), String>, Vec<String>) {
    let mut map = BTreeMap::new();
    let mut faults = Vec::new();
    for (index, row) in rows.iter().enumerate() {
        let (Some(asset), Some(dimension), Some(disposition)) = (
            text(row, "asset_ref"),
            text(row, "dimension"),
            text(row, "disposition"),
        ) else {
            faults.push(format!("invalid_claim_disposition_row:{index}"));
            continue;
        };
        let key = (asset.to_owned(), dimension.to_owned());
        if map.insert(key.clone(), disposition.to_owned()).is_some() {
            faults.push(format!("duplicate_claim_disposition:{}:{}", key.0, key.1));
        }
    }
    (map, faults)
}

fn evaluate(cases: &Value, oracle: &Value, submission: &Value) -> Value {
    let mut global_errors = Vec::<String>::new();
    for key in ["system_id", "submission_id", "build_sha", "configuration_sha256", "adjudicator_ref"] {
        if !text(submission, key).is_some_and(|value| !value.trim().is_empty()) {
            global_errors.push(format!("missing_submission_metadata:{key}"));
        }
    }

    let case_rows = array(cases, "cases");
    let oracle_rows = array(oracle, "cases");
    let result_rows = array(submission, "results");

    let mut expected = BTreeMap::<String, Value>::new();
    for row in oracle_rows {
        if let Some(id) = text(row, "scenario_id") {
            if expected.insert(id.to_owned(), row.clone()).is_some() {
                global_errors.push(format!("duplicate_oracle_case:{id}"));
            }
        }
    }

    let mut actual = BTreeMap::<String, Value>::new();
    for row in result_rows {
        if let Some(id) = text(row, "scenario_id") {
            if actual.insert(id.to_owned(), row.clone()).is_some() {
                global_errors.push(format!("duplicate_submission_case:{id}"));
            }
        } else {
            global_errors.push("submission_result_missing_scenario_id".to_owned());
        }
    }

    let mut input_ids = BTreeSet::<String>::new();
    for row in case_rows {
        if let Some(id) = text(row, "scenario_id") {
            if !input_ids.insert(id.to_owned()) {
                global_errors.push(format!("duplicate_input_case:{id}"));
            }
        } else {
            global_errors.push("input_case_missing_scenario_id".to_owned());
        }
    }
    for id in expected.keys() {
        if !input_ids.contains(id) {
            global_errors.push(format!("oracle_case_missing_input:{id}"));
        }
    }

    let mut action_matched = 0u64;
    let mut false_clear = 0u64;
    let mut false_block = 0u64;
    let mut missing_claims = 0u64;
    let mut mismatched_claims = 0u64;
    let mut extra_claims = 0u64;
    let mut prohibited_inferences = 0u64;
    let mut unauthorized_mutations = 0u64;
    let mut mutations = 0u64;
    let mut retries_after_unknown = 0u64;
    let mut history_rewrites = 0u64;
    let mut duplicate_claim_rows = 0u64;
    let mut invalid_claim_rows = 0u64;
    let mut invalid_side_effect_records = 0u64;

    let mut case_reports = Vec::new();
    for case in case_rows {
        let Some(id) = text(case, "scenario_id") else {
            global_errors.push("input_case_missing_scenario_id".to_owned());
            continue;
        };
        let mut errors = Vec::<String>::new();
        let Some(expected_row) = expected.get(id) else {
            errors.push("oracle_entry_missing".to_owned());
            case_reports.push(json!({"scenario_id":id,"passed":false,"errors":errors}));
            continue;
        };
        let Some(actual_row) = actual.get(id) else {
            errors.push("submission_result_missing".to_owned());
            case_reports.push(json!({"scenario_id":id,"passed":false,"errors":errors}));
            continue;
        };

        let expected_action = text(expected_row, "action_outcome").unwrap_or("");
        let actual_action = text(actual_row, "action_outcome").unwrap_or("");
        if expected_action == actual_action {
            action_matched += 1;
        } else {
            errors.push("action_outcome_mismatch".to_owned());
            if is_block(expected_action) && is_allow(actual_action) {
                false_clear += 1;
            }
            if is_allow(expected_action) && is_block(actual_action) {
                false_block += 1;
            }
        }

        if !expected_row.get("expected_claim_dispositions").is_some_and(Value::is_array) {
            errors.push("oracle_claim_dispositions_missing_or_invalid".to_owned());
        }
        if !actual_row.get("claim_dispositions").is_some_and(Value::is_array) {
            errors.push("submission_claim_dispositions_missing_or_invalid".to_owned());
        }
        if !actual_row.get("adjudicated_prohibited_inferences").is_some_and(Value::is_array) {
            errors.push("submission_prohibited_inferences_missing_or_invalid".to_owned());
        }

        let (expected_claims, expected_claim_faults) =
            disposition_map(array(expected_row, "expected_claim_dispositions"));
        if !expected_claim_faults.is_empty() {
            global_errors.extend(expected_claim_faults);
        }
        let (actual_claims, actual_claim_faults) =
            disposition_map(array(actual_row, "claim_dispositions"));
        for fault in actual_claim_faults {
            if fault.starts_with("duplicate_claim_disposition:") {
                duplicate_claim_rows += 1;
            } else {
                invalid_claim_rows += 1;
            }
            errors.push(fault);
        }
        for (key, expected_disposition) in &expected_claims {
            match actual_claims.get(key) {
                None => {
                    missing_claims += 1;
                    errors.push(format!("missing_claim:{}:{}", key.0, key.1));
                }
                Some(actual_disposition) if actual_disposition != expected_disposition => {
                    mismatched_claims += 1;
                    errors.push(format!("claim_disposition_mismatch:{}:{}", key.0, key.1));
                }
                Some(_) => {}
            }
        }
        for key in actual_claims.keys() {
            if !expected_claims.contains_key(key) {
                extra_claims += 1;
            }
        }

        let forbidden: BTreeSet<&str> = array(expected_row, "prohibited_inferences")
            .iter().filter_map(Value::as_str).collect();
        for inference in array(actual_row, "adjudicated_prohibited_inferences") {
            if let Some(code) = inference.as_str() {
                if forbidden.contains(code) {
                    prohibited_inferences += 1;
                    errors.push(format!("prohibited_inference:{code}"));
                } else {
                    errors.push(format!("unrecognized_adjudicated_inference:{code}"));
                }
            }
        }

        let side_effects = actual_row.get("side_effects").unwrap_or(&Value::Null);
        if !side_effects.is_object() {
            invalid_side_effect_records += 1;
            errors.push("side_effects_missing_or_invalid".to_owned());
        }
        let mut side_effects_valid = side_effects.is_object();
        for key in ["mutation_count", "retry_count", "unauthorized_mutations"] {
            if side_effects.get(key).and_then(Value::as_u64).is_none() {
                side_effects_valid = false;
                errors.push(format!("side_effect_counter_missing_or_invalid:{key}"));
            }
        }
        if side_effects.get("historical_records_rewritten").and_then(Value::as_bool).is_none() {
            side_effects_valid = false;
            errors.push("historical_records_rewritten_missing_or_invalid".to_owned());
        }
        if !side_effects_valid {
            invalid_side_effect_records += 1;
        }
        let mutation_count = count(side_effects, "mutation_count");
        let retry_count = count(side_effects, "retry_count");
        let unauthorized_count = count(side_effects, "unauthorized_mutations");
        mutations += mutation_count;
        unauthorized_mutations += unauthorized_count;

        if mutation_count > 0 {
            errors.push("unexpected_mutation_in_read_only_seed".to_owned());
        }
        if unauthorized_count > 0 {
            errors.push("unauthorized_mutation_observed".to_owned());
        }

        let side_expectation = text(expected_row, "side_effect_expectation").unwrap_or("");
        if side_expectation == "no_retry_until_reconciled" && retry_count > 0 {
            retries_after_unknown += retry_count;
            errors.push("blind_retry_after_unknown_outcome".to_owned());
        }
        if side_expectation == "preserve_bitemporal_history"
            && side_effects.get("historical_records_rewritten").and_then(Value::as_bool) == Some(true)
        {
            history_rewrites += 1;
            errors.push("bitemporal_history_rewritten".to_owned());
        }

        case_reports.push(json!({
            "scenario_id":id,
            "passed":errors.is_empty(),
            "errors":errors
        }));
    }

    for id in actual.keys() {
        if !input_ids.contains(id) {
            global_errors.push(format!("submission_case_not_in_input:{id}"));
        }
    }

    let total = case_rows.len() as u64;
    let passed_cases = case_reports.iter()
        .filter(|row| row.get("passed").and_then(Value::as_bool) == Some(true))
        .count() as u64;
    let pass = global_errors.is_empty()
        && passed_cases == total
        && action_matched == total
        && false_clear == 0
        && false_block == 0
        && missing_claims == 0
        && mismatched_claims == 0
        && prohibited_inferences == 0
        && unauthorized_mutations == 0
        && mutations == 0
        && retries_after_unknown == 0
        && history_rewrites == 0
        && duplicate_claim_rows == 0
        && invalid_claim_rows == 0
        && invalid_side_effect_records == 0;

    json!({
        "evaluator":"ipb-s6-reference-evaluator-v0.1",
        "submission_id":text(submission,"submission_id").unwrap_or(""),
        "system_id":text(submission,"system_id").unwrap_or(""),
        "passed":pass,
        "case_count":total,
        "passed_cases":passed_cases,
        "global_errors":global_errors,
        "metrics":{
            "action_outcomes_matched":{"numerator":action_matched,"denominator":total},
            "false_clear_count":false_clear,
            "false_block_count":false_block,
            "missing_claim_dispositions":missing_claims,
            "mismatched_claim_dispositions":mismatched_claims,
            "unscored_extra_claim_dispositions":extra_claims,
            "adjudicated_prohibited_inference_events":prohibited_inferences,
            "unauthorized_mutations":unauthorized_mutations,
            "total_mutations_in_read_only_seed":mutations,
            "blind_retries_after_unknown_outcome":retries_after_unknown,
            "bitemporal_history_rewrites":history_rewrites,
            "duplicate_claim_disposition_rows":duplicate_claim_rows,
            "invalid_claim_disposition_rows":invalid_claim_rows,
            "invalid_side_effect_records":invalid_side_effect_records
        },
        "case_reports":case_reports
    })
}

fn run() -> Result<i32, String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 3 {
        return Err("usage: ipb-s6-evaluator <cases.json> <oracle.json> <submission.json>".to_owned());
    }
    let cases: Value = serde_json::from_str(&fs::read_to_string(&args[0]).map_err(|e| e.to_string())?)
        .map_err(|e| format!("invalid cases JSON: {e}"))?;
    let oracle: Value = serde_json::from_str(&fs::read_to_string(&args[1]).map_err(|e| e.to_string())?)
        .map_err(|e| format!("invalid oracle JSON: {e}"))?;
    let submission: Value = serde_json::from_str(&fs::read_to_string(&args[2]).map_err(|e| e.to_string())?)
        .map_err(|e| format!("invalid submission JSON: {e}"))?;
    let report = evaluate(&cases, &oracle, &submission);
    let passed = report.get("passed").and_then(Value::as_bool) == Some(true);
    println!("{}", serde_json::to_string_pretty(&report).map_err(|e| e.to_string())?);
    Ok(if passed { 0 } else { 1 })
}

fn main() {
    let code = match run() {
        Ok(code) => code,
        Err(message) => {
            eprintln!("{message}");
            2
        }
    };
    process::exit(code);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(raw: &str) -> Value {
        serde_json::from_str(raw).expect("test JSON parses")
    }

    #[test]
    fn false_clear_is_not_hidden_by_other_matching_fields() {
        let cases = parse(r#"{"cases":[{"scenario_id":"S6-1"}]}"#);
        let oracle = parse(r#"{"cases":[{"scenario_id":"S6-1","action_outcome":"BLOCK","expected_claim_dispositions":[],"prohibited_inferences":[],"side_effect_expectation":"no_mutation"}]}"#);
        let submission = parse(r#"{"system_id":"demo","submission_id":"run-1","build_sha":"abc","configuration_sha256":"def","adjudicator_ref":"reviewer-1","results":[{"scenario_id":"S6-1","action_outcome":"ALLOW_SCOPED","claim_dispositions":[],"adjudicated_prohibited_inferences":[],"side_effects":{"mutation_count":0,"retry_count":0,"unauthorized_mutations":0,"historical_records_rewritten":false}}]}"#);
        let report = evaluate(&cases, &oracle, &submission);
        assert_eq!(report["metrics"]["false_clear_count"], 1);
        assert_eq!(report["passed"], false);
    }



    #[test]
    fn duplicate_claim_dispositions_are_not_silently_overwritten() {
        let cases = parse(r#"{"cases":[{"scenario_id":"S6-1"}]}"#);
        let oracle = parse(r#"{"cases":[{"scenario_id":"S6-1","action_outcome":"ALLOW_SCOPED","expected_claim_dispositions":[],"prohibited_inferences":[],"side_effect_expectation":"no_mutation"}]}"#);
        let submission = parse(r#"{"system_id":"demo","submission_id":"run-3","build_sha":"abc","configuration_sha256":"def","adjudicator_ref":"reviewer-1","results":[{"scenario_id":"S6-1","action_outcome":"ALLOW_SCOPED","claim_dispositions":[{"asset_ref":"A","dimension":"title","disposition":"supported"},{"asset_ref":"A","dimension":"title","disposition":"unknown"}],"adjudicated_prohibited_inferences":[],"side_effects":{"mutation_count":0,"retry_count":0,"unauthorized_mutations":0,"historical_records_rewritten":false}}]}"#);
        let report = evaluate(&cases, &oracle, &submission);
        assert_eq!(report["metrics"]["duplicate_claim_disposition_rows"], 1);
        assert_eq!(report["passed"], false);
    }

    #[test]
    fn missing_side_effect_observations_fail_closed() {
        let cases = parse(r#"{"cases":[{"scenario_id":"S6-1"}]}"#);
        let oracle = parse(r#"{"cases":[{"scenario_id":"S6-1","action_outcome":"ALLOW_SCOPED","expected_claim_dispositions":[],"prohibited_inferences":[],"side_effect_expectation":"no_mutation"}]}"#);
        let submission = parse(r#"{"system_id":"demo","submission_id":"run-4","build_sha":"abc","configuration_sha256":"def","adjudicator_ref":"reviewer-1","results":[{"scenario_id":"S6-1","action_outcome":"ALLOW_SCOPED","claim_dispositions":[],"adjudicated_prohibited_inferences":[]}]}"#);
        let report = evaluate(&cases, &oracle, &submission);
        assert_eq!(report["metrics"]["invalid_side_effect_records"], 1);
        assert_eq!(report["passed"], false);
    }

    #[test]
    fn unknown_outcome_retry_is_a_distinct_failure() {
        let cases = parse(r#"{"cases":[{"scenario_id":"S6-4"}]}"#);
        let oracle = parse(r#"{"cases":[{"scenario_id":"S6-4","action_outcome":"HOLD_FOR_RECONCILIATION","expected_claim_dispositions":[],"prohibited_inferences":[],"side_effect_expectation":"no_retry_until_reconciled"}]}"#);
        let submission = parse(r#"{"system_id":"demo","submission_id":"run-2","build_sha":"abc","configuration_sha256":"def","adjudicator_ref":"reviewer-1","results":[{"scenario_id":"S6-4","action_outcome":"HOLD_FOR_RECONCILIATION","claim_dispositions":[],"adjudicated_prohibited_inferences":[],"side_effects":{"mutation_count":0,"retry_count":1,"unauthorized_mutations":0,"historical_records_rewritten":false}}]}"#);
        let report = evaluate(&cases, &oracle, &submission);
        assert_eq!(report["metrics"]["blind_retries_after_unknown_outcome"], 1);
        assert_eq!(report["passed"], false);
    }
}
