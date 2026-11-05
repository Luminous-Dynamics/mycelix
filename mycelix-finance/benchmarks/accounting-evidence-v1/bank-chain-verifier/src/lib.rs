#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde::Serialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

type AppResult<T> = Result<T, String>;

#[derive(Debug, Serialize)]
struct CaseResult {
    case_id: String,
    actual_disposition: String,
    expected_disposition: String,
    passed: bool,
    reason_codes: Vec<String>,
    economic_observation_count: usize,
}

#[derive(Debug)]
struct Assessment {
    disposition: String,
    reason_codes: BTreeSet<String>,
    economic_observation_count: usize,
}

pub fn verify(
    input_bytes: &[u8],
    input_schema_bytes: &[u8],
    oracle_bytes: &[u8],
    oracle_schema_bytes: &[u8],
) -> Value {
    let mut errors = Vec::<String>::new();
    let input: Value = match serde_json::from_slice(input_bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("candidate input JSON: {error}"));
            Value::Null
        }
    };
    let input_schema: Value = match serde_json::from_slice(input_schema_bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("candidate input schema JSON: {error}"));
            Value::Null
        }
    };
    let oracle: Value = match serde_json::from_slice(oracle_bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("public reference oracle JSON: {error}"));
            Value::Null
        }
    };
    let oracle_schema: Value = match serde_json::from_slice(oracle_schema_bytes) {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("oracle schema JSON: {error}"));
            Value::Null
        }
    };

    if errors.is_empty() {
        errors.extend(validate_instance("candidate input", &input_schema, &input));
        errors.extend(validate_instance("public reference oracle", &oracle_schema, &oracle));
    }

    let mut results = Vec::<CaseResult>::new();
    if errors.is_empty() {
        let input_cases = input.get("cases").and_then(Value::as_array);
        let oracle_cases = oracle.get("cases").and_then(Value::as_array);
        match (input_cases, oracle_cases) {
            (Some(input_cases), Some(oracle_cases)) => {
                let mut oracle_by_id = BTreeMap::<String, &Value>::new();
                for oracle_case in oracle_cases {
                    match string(oracle_case, "case_id") {
                        Some(id) => {
                            if oracle_by_id.insert(id.to_owned(), oracle_case).is_some() {
                                errors.push(format!("duplicate oracle case_id: {id}"));
                            }
                        }
                        None => errors.push("oracle case is missing case_id".to_owned()),
                    }
                }
                let mut seen = BTreeSet::new();
                for case in input_cases {
                    let id = string(case, "case_id").unwrap_or("<missing>").to_owned();
                    if !seen.insert(id.clone()) {
                        errors.push(format!("duplicate candidate case_id: {id}"));
                        continue;
                    }
                    let expected = oracle_by_id.get(&id);
                    let assessment = assess_case(case);
                    match (expected, assessment) {
                        (Some(expected), Ok(assessment)) => {
                            let expected_disposition =
                                string(expected, "expected_disposition").unwrap_or("<missing>").to_owned();
                            let passed = assessment.disposition == expected_disposition;
                            results.push(CaseResult {
                                case_id: id,
                                actual_disposition: assessment.disposition,
                                expected_disposition,
                                passed,
                                reason_codes: assessment.reason_codes.into_iter().collect(),
                                economic_observation_count: assessment.economic_observation_count,
                            });
                        }
                        (None, _) => errors.push(format!("candidate case {id} has no oracle record")),
                        (_, Err(error)) => errors.push(format!("case {id}: {error}")),
                    }
                }
                for oracle_id in oracle_by_id.keys() {
                    if !seen.contains(oracle_id) {
                        errors.push(format!("oracle case {oracle_id} has no candidate input"));
                    }
                }
            }
            _ => errors.push("candidate and oracle corpora must each contain a cases array".to_owned()),
        }
    }

    let passed_count = results.iter().filter(|case| case.passed).count();
    let status = if errors.is_empty() && !results.is_empty() && passed_count == results.len() {
        "PASS"
    } else {
        "FAIL"
    };
    let report = json!({
        "report_schema": "fin-bank-chain-verifier.report.v1",
        "benchmark_id": input.get("benchmark_id"),
        "dataset_version": input.get("dataset_version"),
        "status": status,
        "claim_scope": "public_synthetic_semantic_conformance_only_not_blind_evaluation",
        "digests": {
            "candidate_input_sha256": sha256_hex(input_bytes),
            "candidate_schema_sha256": sha256_hex(input_schema_bytes),
            "public_reference_oracle_sha256": sha256_hex(oracle_bytes),
            "oracle_schema_sha256": sha256_hex(oracle_schema_bytes),
            "verifier_library_sha256": sha256_hex(include_bytes!("lib.rs"))
        },
        "case_count": results.len(),
        "passed_case_count": passed_count,
        "errors": errors,
        "cases": results
    });
    let report_schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(schema) => schema,
        Err(error) => {
            let mut failed = report;
            failed["status"] = Value::String("FAIL".to_owned());
            failed["errors"].as_array_mut().map(|items| items.push(Value::String(format!("embedded report schema: {error}"))));
            return failed;
        }
    };
    let report_errors = validate_instance("generated verifier report", &report_schema, &report);
    if report_errors.is_empty() {
        report
    } else {
        let mut failed = report;
        failed["status"] = Value::String("FAIL".to_owned());
        if let Some(items) = failed["errors"].as_array_mut() {
            for error in report_errors {
                items.push(Value::String(error));
            }
        }
        failed
    }
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(validator) => validator,
        Err(error) => return vec![format!("{label}: could not compile schema: {error}")],
    };
    validator
        .iter_errors(instance)
        .map(|error| format!("{label}: {} at {}", error, error.instance_path()))
        .collect()
}

fn string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

fn amount(value: &Value) -> AppResult<i128> {
    let text = value.as_str().ok_or_else(|| "amount must be a canonical integer string".to_owned())?;
    if text.is_empty() || (text.len() > 1 && text.starts_with('0')) || !text.bytes().all(|b| b.is_ascii_digit()) {
        return Err(format!("non-canonical non-negative amount: {text}"));
    }
    let parsed = text.parse::<u128>().map_err(|error| format!("amount parse failed: {error}"))?;
    i128::try_from(parsed).map_err(|_| "amount exceeds i128 verifier range".to_owned())
}

fn field_amount(value: &Value, key: &str) -> AppResult<i128> {
    let field = value.get(key).ok_or_else(|| format!("missing amount field: {key}"))?;
    amount(field)
}

fn assess_case(case: &Value) -> AppResult<Assessment> {
    let manifest = case.get("manifest").ok_or_else(|| "case is missing manifest".to_owned())?;
    let mut findings = BTreeSet::<String>::new();
    let mut reasons = BTreeSet::<String>::new();

    let account = manifest.get("account").ok_or_else(|| "manifest is missing account".to_owned())?;
    let chain_account = string(account, "account_token").ok_or_else(|| "account token is missing".to_owned())?;
    let chain_entity = string(account, "entity_token").ok_or_else(|| "entity token is missing".to_owned())?;
    let chain_denomination = string(manifest, "denomination_profile_ref").ok_or_else(|| "denomination profile is missing".to_owned())?;

    let artifacts = array(manifest, "artifacts");
    let statements = array(manifest, "statements");
    let relations = array(manifest, "relations");
    let mut artifact_by_id = BTreeMap::<String, &Value>::new();
    for artifact in artifacts {
        let id = string(artifact, "artifact_id").ok_or_else(|| "artifact missing artifact_id".to_owned())?;
        if artifact_by_id.insert(id.to_owned(), artifact).is_some() {
            findings.insert("revision_conflict".to_owned());
            reasons.insert("duplicate_artifact_identity".to_owned());
        }
    }

    // A message identity with changed fixture bytes is a conflict, not an overwrite.
    let mut message_digest = BTreeMap::<String, String>::new();
    for artifact in artifacts {
        let message_id = string(artifact, "message_id").ok_or_else(|| "artifact missing message_id".to_owned())?;
        let digest = string(artifact, "fixture_digest_token").ok_or_else(|| "artifact missing fixture_digest_token".to_owned())?;
        if let Some(prior) = message_digest.insert(message_id.to_owned(), digest.to_owned()) {
            if prior != digest {
                findings.insert("revision_conflict".to_owned());
                reasons.insert("same_message_identity_changed_payload".to_owned());
            }
        }
    }

    let mut superseded_artifacts = BTreeSet::<String>::new();
    let mut has_explicit_revision = false;
    for relation in relations {
        let relation_type = string(relation, "relation_type").unwrap_or_default();
        if relation_type == "replaces" || relation_type == "corrects" {
            let from = string(relation, "from_artifact_id").unwrap_or_default();
            let to = string(relation, "to_artifact_id").unwrap_or_default();
            if artifact_by_id.contains_key(from) && artifact_by_id.contains_key(to) {
                superseded_artifacts.insert(to.to_owned());
                has_explicit_revision = true;
            }
        }
    }
    let active_statements = statements.iter().filter(|statement| {
        string(statement, "artifact_id")
            .map(|id| !superseded_artifacts.contains(id))
            .unwrap_or(false)
    }).collect::<Vec<_>>();

    // Verify identity, profile, pagination, booked-balance arithmetic and source authority.
    for statement in &active_statements {
        let artifact_id = string(statement, "artifact_id").unwrap_or_default();
        let artifact = match artifact_by_id.get(artifact_id) {
            Some(artifact) => *artifact,
            None => {
                findings.insert("profile_unsupported".to_owned());
                reasons.insert("statement_artifact_missing".to_owned());
                continue;
            }
        };
        if string(artifact, "message_definition") != Some("camt.053.001.14") {
            findings.insert("profile_unsupported".to_owned());
            reasons.insert("non_statement_message_used_as_booked_statement".to_owned());
        }
        if string(artifact, "account_token") != string(statement, "account_token") {
            findings.insert("account_lineage_unknown".to_owned());
            reasons.insert("artifact_statement_account_mismatch".to_owned());
        }
        if string(statement, "account_token") != Some(chain_account) ||
           string(statement, "entity_token") != Some(chain_entity) {
            findings.insert("account_lineage_unknown".to_owned());
            reasons.insert("account_entity_binding_conflict".to_owned());
        }
        if string(statement, "denomination_profile_ref") != Some(chain_denomination) {
            findings.insert("profile_unsupported".to_owned());
            reasons.insert("denomination_profile_not_pinned".to_owned());
        }
        for entry in array(statement, "entries") {
            if string(entry, "denomination_profile_ref") != Some(chain_denomination) {
                findings.insert("profile_unsupported".to_owned());
                reasons.insert("entry_denomination_profile_mismatch".to_owned());
            }
        }
        if artifact.get("sender_authenticated").and_then(Value::as_bool) != Some(true) {
            findings.insert("source_authentication_unqualified".to_owned());
            reasons.insert("transport_or_sender_authentication_unqualified".to_owned());
        }
        let expected_pages = statement.get("expected_page_count").and_then(Value::as_u64).unwrap_or(0) as usize;
        let received_pages = array(statement, "received_pages");
        let page_set = received_pages.iter().filter_map(Value::as_u64).collect::<BTreeSet<_>>();
        let pages_complete = expected_pages > 0
            && page_set.len() == expected_pages
            && (1..=expected_pages as u64).all(|page| page_set.contains(&page));
        if !pages_complete {
            findings.insert("gap_unknown".to_owned());
            reasons.insert("statement_pagination_incomplete".to_owned());
        }

        let opening = statement.get("opening_balance").ok_or_else(|| "statement opening balance missing".to_owned())?;
        let closing = statement.get("closing_balance").ok_or_else(|| "statement closing balance missing".to_owned())?;
        if string(opening, "balance_type") != Some("BOOKED") ||
           string(closing, "balance_type") != Some("BOOKED") {
            findings.insert("profile_unsupported".to_owned());
            reasons.insert("available_balance_not_booked_balance".to_owned());
        }
        let mut reconstructed = field_amount(opening, "amount_minor")?;
        for entry in array(statement, "entries") {
            let entry_amount = field_amount(entry, "amount_minor")?;
            reconstructed = match string(entry, "direction") {
                Some("CREDIT") => reconstructed.checked_add(entry_amount),
                Some("DEBIT") => reconstructed.checked_sub(entry_amount),
                _ => None,
            }.ok_or_else(|| "invalid direction or checked balance arithmetic overflow".to_owned())?;
        }
        let declared = field_amount(closing, "amount_minor")?;
        if reconstructed != declared {
            findings.insert("balance_conflict".to_owned());
            reasons.insert("statement_balance_reconstruction_mismatch".to_owned());
        }
    }

    // Each required period must have exactly one active CAMT.053 statement.
    let expected_scope = array(manifest, "expected_scope");
    let mut active_by_period = BTreeMap::<String, Vec<&Value>>::new();
    for statement in &active_statements {
        if let Some(period_id) = string(statement, "period_id") {
            active_by_period.entry(period_id.to_owned()).or_default().push(*statement);
        }
        if let Some(scope) = expected_scope.iter().find(|scope| {
            string(scope, "period_id") == string(statement, "period_id")
        }) {
            if string(statement, "cutoff_profile_ref") != string(scope, "cutoff_profile_ref") {
                findings.insert("profile_unsupported".to_owned());
                reasons.insert("incompatible_cutoff_semantics".to_owned());
            }
        }
    }
    for scope in expected_scope.iter().filter(|scope| scope.get("required").and_then(Value::as_bool) == Some(true)) {
        let period_id = string(scope, "period_id").unwrap_or_default();
        match active_by_period.get(period_id) {
            None => {
                findings.insert("gap_unknown".to_owned());
                reasons.insert("required_statement_period_missing".to_owned());
            }
            Some(rows) if rows.len() > 1 => {
                findings.insert("revision_conflict".to_owned());
                reasons.insert("competing_statement_versions_without_relation".to_owned());
            }
            Some(_) => {}
        }
    }

    // The expected source population is declared independently of files received.
    let expected_population = array(manifest, "expected_population_tokens").iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
    let observed_population = array(manifest, "observed_population_tokens").iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
    if !expected_population.is_subset(&observed_population) {
        findings.insert("gap_unknown".to_owned());
        reasons.insert("expected_source_population_not_observed".to_owned());
    }

    // Adjacent required periods compare only compatible booked balances and cutoff profiles.
    let mut scopes = expected_scope.iter().filter(|scope| scope.get("required").and_then(Value::as_bool) == Some(true)).collect::<Vec<_>>();
    scopes.sort_by_key(|scope| string(scope, "start_at").unwrap_or_default());
    for pair in scopes.windows(2) {
        let previous_id = string(pair[0], "period_id").unwrap_or_default();
        let next_id = string(pair[1], "period_id").unwrap_or_default();
        if string(pair[0], "cutoff_profile_ref") != string(pair[1], "cutoff_profile_ref") {
            findings.insert("profile_unsupported".to_owned());
            reasons.insert("incompatible_cutoff_semantics".to_owned());
            continue;
        }
        let previous = active_by_period.get(previous_id).and_then(|rows| rows.first());
        let next = active_by_period.get(next_id).and_then(|rows| rows.first());
        if let (Some(previous), Some(next)) = (previous, next) {
            let previous_closing = previous.get("closing_balance").ok_or_else(|| "previous closing balance missing".to_owned())?;
            let next_opening = next.get("opening_balance").ok_or_else(|| "next opening balance missing".to_owned())?;
            if string(previous_closing, "balance_type") != Some("BOOKED") ||
               string(next_opening, "balance_type") != Some("BOOKED") {
                findings.insert("profile_unsupported".to_owned());
                reasons.insert("carry_forward_not_booked_balance".to_owned());
            } else if string(previous, "denomination_profile_ref") != string(next, "denomination_profile_ref") {
                findings.insert("profile_unsupported".to_owned());
                reasons.insert("carry_forward_denomination_mismatch".to_owned());
            } else if field_amount(previous_closing, "amount_minor")? != field_amount(next_opening, "amount_minor")? {
                findings.insert("balance_conflict".to_owned());
                reasons.insert("opening_closing_booked_balance_mismatch".to_owned());
            }
        }
    }

    // A duplicated economic event across statements is one observation only when
    // its exact amount/semantics agree and an explicit bank-lineage relation exists.
    let mut event_rows = BTreeMap::<String, Vec<(&Value, &Value)>>::new();
    for statement in &active_statements {
        let artifact_id = string(statement, "artifact_id").unwrap_or_default();
        for entry in array(statement, "entries") {
            if let Some(event_id) = string(entry, "economic_event_ref") {
                event_rows.entry(event_id.to_owned()).or_default().push((statement, entry));
            }
        }
        let _ = artifact_id;
    }
    for (event_id, rows) in &event_rows {
        if rows.len() > 1 {
            let first = rows[0];
            for duplicate in rows.iter().skip(1) {
                let same = string(first.1, "amount_minor") == string(duplicate.1, "amount_minor")
                    && string(first.1, "direction") == string(duplicate.1, "direction")
                    && string(first.1, "denomination_profile_ref") == string(duplicate.1, "denomination_profile_ref")
                    && string(first.1, "booking_at") == string(duplicate.1, "booking_at")
                    && string(first.1, "value_date") == string(duplicate.1, "value_date");
                let from_artifact = string(first.0, "artifact_id").unwrap_or_default();
                let to_artifact = string(duplicate.0, "artifact_id").unwrap_or_default();
                let linked = relations.iter().any(|relation| {
                    string(relation, "relation_type") == Some("same_economic_observation")
                        && ((string(relation, "from_artifact_id") == Some(from_artifact)
                            && string(relation, "to_artifact_id") == Some(to_artifact))
                            || (string(relation, "from_artifact_id") == Some(to_artifact)
                                && string(relation, "to_artifact_id") == Some(from_artifact)))
                });
                if !same || !linked {
                    findings.insert("overlap_unresolved".to_owned());
                    reasons.insert(format!("overlap_entry_semantics_conflict:{event_id}"));
                }
            }
        }
    }

    // Notification artifacts do not become booked entries; a valid link is a relation.
    for artifact in artifacts {
        if string(artifact, "message_definition") == Some("camt.054.001.14") {
            let id = string(artifact, "artifact_id").unwrap_or_default();
            let linked = relations.iter().any(|relation| {
                string(relation, "relation_type") == Some("notification_for")
                    && string(relation, "from_artifact_id") == Some(id)
                    && artifact_by_id.get(string(relation, "to_artifact_id").unwrap_or_default())
                        .and_then(|target| string(target, "message_definition"))
                        == Some("camt.053.001.14")
            });
            if !linked {
                reasons.insert("unmatched_notification_not_booked_evidence".to_owned());
            }
        }
    }

    if has_explicit_revision || active_statements.iter().any(|statement| {
        array(statement, "entries").iter().any(|entry| string(entry, "entry_type") == Some("BANK_FEE"))
    }) {
        reasons.insert("explicit_adjustment_or_revision_preserved".to_owned());
    }

    let disposition = if findings.contains("revision_conflict") {
        "revision_conflict"
    } else if findings.contains("overlap_unresolved") {
        "overlap_unresolved"
    } else if findings.contains("balance_conflict") {
        "balance_conflict"
    } else if findings.contains("account_lineage_unknown") {
        "account_lineage_unknown"
    } else if findings.contains("profile_unsupported") {
        "profile_unsupported"
    } else if findings.contains("gap_unknown") {
        "gap_unknown"
    } else if findings.contains("source_authentication_unqualified") {
        "source_authentication_unqualified"
    } else if has_explicit_revision || active_statements.iter().any(|statement| {
        array(statement, "entries").iter().any(|entry| string(entry, "entry_type") == Some("BANK_FEE"))
    }) {
        "complete_with_explicit_adjustment"
    } else {
        "complete_contiguous"
    }.to_owned();

    let economic_observation_count = event_rows.len();
    Ok(Assessment {
        disposition,
        reason_codes: reasons,
        economic_observation_count,
    })
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn fixture(path: &str) -> Value {
        serde_json::from_str(path).expect("fixture JSON must parse")
    }

    #[test]
    fn all_public_reference_cases_match_the_oracle() {
        let candidate = fixture(include_str!("../../bank_statement_chain_candidates_v1.0.0.json"));
        let oracle = fixture(include_str!("../../public-reference-oracle/bank_statement_chain_oracle_v1.0.0.json"));
        let expected = oracle["cases"].as_array().expect("oracle cases");
        for case in candidate["cases"].as_array().expect("candidate cases") {
            let id = case["case_id"].as_str().expect("case id");
            let row = expected.iter().find(|row| row["case_id"] == id).expect("matching oracle row");
            let assessment = assess_case(case).expect("case assessment");
            assert_eq!(assessment.disposition, row["expected_disposition"].as_str().expect("expected disposition"), "{id}");
        }
    }

    #[test]
    fn exact_replay_does_not_increase_economic_observation_count() {
        let candidate = fixture(include_str!("../../bank_statement_chain_candidates_v1.0.0.json"));
        let cases = candidate["cases"].as_array().expect("candidate cases");
        let case = cases.iter().find(|case| case["case_id"] == "BSC-026").expect("BSC-026");
        let assessment = assess_case(case).expect("assessment");
        assert_eq!(assessment.disposition, "complete_contiguous");
        assert_eq!(assessment.economic_observation_count, 2);
    }

    #[test]
    fn missing_period_is_never_complete() {
        let candidate = fixture(include_str!("../../bank_statement_chain_candidates_v1.0.0.json"));
        let cases = candidate["cases"].as_array().expect("candidate cases");
        let case = cases.iter().find(|case| case["case_id"] == "BSC-010").expect("BSC-010");
        assert_eq!(assess_case(case).expect("assessment").disposition, "gap_unknown");
    }

    #[test]
    fn one_minor_unit_balance_mutation_is_detected() {
        let candidate = fixture(include_str!("../../bank_statement_chain_candidates_v1.0.0.json"));
        let cases = candidate["cases"].as_array().expect("candidate cases");
        let case = cases.iter().find(|case| case["case_id"] == "BSC-002").expect("BSC-002");
        assert_eq!(assess_case(case).expect("assessment").disposition, "balance_conflict");
    }

    #[test]
    fn arrival_order_does_not_change_chain_disposition() {
        let candidate = fixture(include_str!("../../bank_statement_chain_candidates_v1.0.0.json"));
        let cases = candidate["cases"].as_array().expect("candidate cases");
        let case = cases.iter().find(|case| case["case_id"] == "BSC-025").expect("BSC-025");
        assert_eq!(assess_case(case).expect("assessment").disposition, "complete_contiguous");
    }
}
