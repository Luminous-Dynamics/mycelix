#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const TASKS_DEFAULT: &str = "../tasks_v1.0.0.json";
const TASK_SCHEMA_DEFAULT: &str = "../task.schema.json";
const ORACLE_DEFAULT: &str = "../oracle_v1.0.0.json";
const ORACLE_SCHEMA_DEFAULT: &str = "../oracle.schema.json";
const RESPONSES_DEFAULT: &str = "../reference_responses_v1.0.0.json";
const RESPONSE_SCHEMA_DEFAULT: &str = "../response.schema.json";

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("fin-trust-s0-verifier: {error}");
        std::process::exit(2);
    }
}

fn run_cli() -> Result<(), Box<dyn Error>> {
    let mut tasks = PathBuf::from(TASKS_DEFAULT);
    let mut task_schema = PathBuf::from(TASK_SCHEMA_DEFAULT);
    let mut oracle = PathBuf::from(ORACLE_DEFAULT);
    let mut oracle_schema = PathBuf::from(ORACLE_SCHEMA_DEFAULT);
    let mut responses = PathBuf::from(RESPONSES_DEFAULT);
    let mut response_schema = PathBuf::from(RESPONSE_SCHEMA_DEFAULT);
    let mut output: Option<PathBuf> = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--tasks" => tasks = PathBuf::from(next_arg(&mut args, "--tasks")?),
            "--task-schema" => task_schema = PathBuf::from(next_arg(&mut args, "--task-schema")?),
            "--oracle" => oracle = PathBuf::from(next_arg(&mut args, "--oracle")?),
            "--oracle-schema" => oracle_schema = PathBuf::from(next_arg(&mut args, "--oracle-schema")?),
            "--responses" => responses = PathBuf::from(next_arg(&mut args, "--responses")?),
            "--response-schema" => response_schema = PathBuf::from(next_arg(&mut args, "--response-schema")?),
            "--out" => output = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--help" | "-h" => {
                println!("FIN-TRUST S0 verifier: --tasks --task-schema --oracle --oracle-schema --responses --response-schema --out");
                return Ok(());
            }
            unknown => return Err(format!("unknown argument: {unknown}; use --help").into()),
        }
    }

    let task_bytes = fs::read(tasks)?;
    let task_schema_bytes = fs::read(task_schema)?;
    let oracle_bytes = fs::read(oracle)?;
    let oracle_schema_bytes = fs::read(oracle_schema)?;
    let response_bytes = fs::read(responses)?;
    let response_schema_bytes = fs::read(response_schema)?;

    let report = verify(
        &task_bytes,
        &task_schema_bytes,
        &oracle_bytes,
        &oracle_schema_bytes,
        &response_bytes,
        &response_schema_bytes,
    );
    let rendered = serde_json::to_string_pretty(&report)?;
    match output {
        Some(path) => fs::write(path, format!("{rendered}\n"))?,
        None => println!("{rendered}"),
    }
    if report["status"] != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

fn next_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<String, Box<dyn Error>> {
    args.next()
        .ok_or_else(|| format!("{flag} requires a path").into())
}

fn verify(
    task_bytes: &[u8],
    task_schema_bytes: &[u8],
    oracle_bytes: &[u8],
    oracle_schema_bytes: &[u8],
    response_bytes: &[u8],
    response_schema_bytes: &[u8],
) -> Value {
    let parsed = (
        serde_json::from_slice::<Value>(task_bytes),
        serde_json::from_slice::<Value>(task_schema_bytes),
        serde_json::from_slice::<Value>(oracle_bytes),
        serde_json::from_slice::<Value>(oracle_schema_bytes),
        serde_json::from_slice::<Value>(response_bytes),
        serde_json::from_slice::<Value>(response_schema_bytes),
    );
    let (tasks, task_schema, oracle, oracle_schema, responses, response_schema) = match parsed {
        (Ok(a), Ok(b), Ok(c), Ok(d), Ok(e), Ok(f)) => (a, b, c, d, e, f),
        _ => {
            return json!({
                "report_schema": "fin-trust-s0-verifier.report.v1",
                "benchmark_id": "FIN-TRUST-BENCH",
                "dataset_version": "1.0.0",
                "status": "FAIL",
                "claim_scope": "public_synthetic_clarification_source_binding_journal_and_statement_conformance_only",
                "network_access_during_verification": false,
                "case_count": 0,
                "passed_case_count": 0,
                "errors": ["one or more input files are not valid JSON"],
                "cases": [],
                "digests": {}
            });
        }
    };

    let mut errors = Vec::<String>::new();
    errors.extend(validate_instance("tasks", &task_schema, &tasks));
    errors.extend(validate_instance("oracle", &oracle_schema, &oracle));
    errors.extend(validate_instance("responses", &response_schema, &responses));

    let mut results = Vec::<Value>::new();
    if errors.is_empty() {
        let task_rows = array(&tasks, "tasks");
        let oracle_rows = array(&oracle, "cases");
        let response_rows = array(&responses, "responses");
        let task_map = unique_index(task_rows, "task_id", "task", &mut errors);
        let oracle_map = unique_index(oracle_rows, "task_id", "oracle", &mut errors);
        let response_map = unique_index(response_rows, "task_id", "response", &mut errors);

        let task_ids = task_map.keys().cloned().collect::<BTreeSet<_>>();
        let oracle_ids = oracle_map.keys().cloned().collect::<BTreeSet<_>>();
        let response_ids = response_map.keys().cloned().collect::<BTreeSet<_>>();
        if task_ids != oracle_ids {
            errors.push("task IDs and oracle IDs differ".to_owned());
        }
        if task_ids != response_ids {
            errors.push("task IDs and response IDs differ".to_owned());
        }

        for task_id in &task_ids {
            match (task_map.get(task_id), oracle_map.get(task_id), response_map.get(task_id)) {
                (Some(task), Some(expected), Some(response)) => {
                    results.push(verify_case(task_id, task, expected, response));
                }
                _ => errors.push(format!("{task_id}: missing task, oracle or response")),
            }
        }
    }

    let passed = results
        .iter()
        .filter(|row| row["status"] == "PASS")
        .count();
    let status = if errors.is_empty() && !results.is_empty() && passed == results.len() {
        "PASS"
    } else {
        "FAIL"
    };
    let mut report = json!({
        "report_schema": "fin-trust-s0-verifier.report.v1",
        "benchmark_id": tasks.get("benchmark_id"),
        "dataset_version": tasks.get("dataset_version"),
        "status": status,
        "claim_scope": "public_synthetic_clarification_source_binding_journal_and_statement_conformance_only",
        "network_access_during_verification": false,
        "case_count": results.len(),
        "passed_case_count": passed,
        "errors": errors,
        "cases": results,
        "digests": {
            "tasks_sha256": sha256_hex(task_bytes),
            "task_schema_sha256": sha256_hex(task_schema_bytes),
            "oracle_sha256": sha256_hex(oracle_bytes),
            "oracle_schema_sha256": sha256_hex(oracle_schema_bytes),
            "responses_sha256": sha256_hex(response_bytes),
            "response_schema_sha256": sha256_hex(response_schema_bytes),
            "verifier_source_sha256": sha256_hex(include_bytes!("main.rs")),
            "report_schema_sha256": sha256_hex(include_bytes!("../report.schema.json"))
        }
    });
    let report_schema: Value = match serde_json::from_str(include_str!("../report.schema.json")) {
        Ok(value) => value,
        Err(error) => {
            report["status"] = Value::String("FAIL".to_owned());
            report["errors"].as_array_mut().expect("errors array")
                .push(Value::String(format!("embedded report schema JSON: {error}")));
            return report;
        }
    };
    let report_errors = validate_instance("report", &report_schema, &report);
    if !report_errors.is_empty() {
        report["status"] = Value::String("FAIL".to_owned());
        report["errors"].as_array_mut().expect("errors array")
            .push(Value::String(report_errors.join("; ")));
    }
    report
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options().should_validate_formats(true).build(schema) {
        Ok(value) => value,
        Err(error) => return vec![format!("{label}: cannot compile schema: {error}")],
    };
    validator.iter_errors(instance)
        .map(|error| format!("{label}: {} at {}", error, error.instance_path()))
        .collect()
}

fn unique_index<'a>(
    rows: &'a [Value],
    key: &str,
    label: &str,
    errors: &mut Vec<String>,
) -> BTreeMap<String, &'a Value> {
    let mut index = BTreeMap::new();
    for row in rows {
        let id = text(row, key).unwrap_or("<missing>").to_owned();
        if index.insert(id.clone(), row).is_some() {
            errors.push(format!("duplicate {label} ID: {id}"));
        }
    }
    index
}

fn verify_case(task_id: &str, task: &Value, expected: &Value, response: &Value) -> Value {
    let mut checks = Vec::<Value>::new();
    let mut reasons = Vec::<String>::new();

    check(
        &mut checks,
        &mut reasons,
        "task_identity",
        text(task, "task_id") == Some(task_id)
            && text(response, "task_id") == Some(task_id)
            && text(expected, "task_id") == Some(task_id),
        "task, oracle and response identities must match",
    );
    check(
        &mut checks,
        &mut reasons,
        "disposition_exact",
        text(response, "disposition") == text(expected, "disposition"),
        "response disposition must equal the adjudicated terminal disposition",
    );

    let required_questions = sorted_strings(array(expected, "required_question_ids"));
    let forbidden_questions = strings(array(expected, "forbidden_question_ids"));
    let actual_questions = strings(
        &array(response, "questions")
            .iter()
            .filter_map(|row| row.get("question_id"))
            .cloned()
            .collect::<Vec<_>>(),
    );
    check(
        &mut checks,
        &mut reasons,
        "minimum_clarification_set",
        actual_questions == required_questions,
        format!("actual={actual_questions:?} required={required_questions:?}"),
    );
    check(
        &mut checks,
        &mut reasons,
        "no_forbidden_or_redundant_questions",
        forbidden_questions.iter().all(|id| !actual_questions.contains(id)),
        format!("forbidden question IDs={forbidden_questions:?}"),
    );

    let task_unknowns = array(task, "known_unknowns")
        .iter()
        .filter_map(|row| text(row, "unknown_id").map(str::to_owned))
        .collect::<BTreeSet<_>>();
    let mut question_resolutions = BTreeSet::new();
    let mut question_ids_unique = BTreeSet::new();
    for question in array(response, "questions") {
        let qid = text(question, "question_id").unwrap_or("<missing>").to_owned();
        if !question_ids_unique.insert(qid.clone()) {
            reasons.push("duplicate_question_id".to_owned());
        }
        for unknown in strings(array(question, "resolves")) {
            if !task_unknowns.contains(&unknown) {
                reasons.push("question_resolves_unknown_not_in_task".to_owned());
            }
            question_resolutions.insert(unknown);
        }
    }
    check(
        &mut checks,
        &mut reasons,
        "questions_resolve_declared_unknowns",
        reasons.iter().all(|reason| reason != "duplicate_question_id" && reason != "question_resolves_unknown_not_in_task"),
        "clarification IDs must be unique and target declared task unknowns",
    );

    let expected_unresolved = sorted_strings(array(expected, "unresolved_ids"));
    let actual_unresolved = sorted_strings(array(response, "unresolved"));
    check(
        &mut checks,
        &mut reasons,
        "unresolved_decisions_preserved",
        actual_unresolved == expected_unresolved,
        format!("actual={actual_unresolved:?} expected={expected_unresolved:?}"),
    );
    check(
        &mut checks,
        &mut reasons,
        "no_complete_books_claim",
        bool_value(response, "claims_complete_books") == Some(false)
            && bool_value(expected, "must_not_claim_complete") == Some(true),
        "S0 tasks do not establish complete books",
    );

    let sources = array(task, "sources");
    let source_map = unique_source_index(sources, &mut reasons);
    let requirements = array(task, "requirements")
        .iter()
        .filter_map(|row| text(row, "requirement_id").map(str::to_owned))
        .collect::<BTreeSet<_>>();
    let mut bound_source_ids = BTreeSet::new();
    let mut bound_requirement_ids = BTreeSet::new();
    let mut qualified_source_ids = BTreeSet::new();

    for binding in array(response, "source_bindings") {
        let source_id = text(binding, "source_id").unwrap_or("<missing>").to_owned();
        let requirement_id = text(binding, "requirement_id").unwrap_or("<missing>").to_owned();
        bound_source_ids.insert(source_id.clone());
        bound_requirement_ids.insert(requirement_id.clone());

        let source = source_map.get(&source_id);
        let source_exists = source.is_some();
        let mut binding_valid = source_exists && requirements.contains(&requirement_id);
        if let Some(source) = source {
            let digest = text(source, "sha256").unwrap_or_default();
            let revision = text(source, "revision").unwrap_or_default();
            let locator = text(source.get("locator").unwrap_or(&Value::Null), "location").unwrap_or_default();
            binding_valid &= text(binding, "sha256") == Some(digest)
                && text(binding, "revision") == Some(revision)
                && text(binding, "locator") == Some(locator);
            let qualifies = bool_value(binding, "qualifies") == Some(true);
            if qualifies {
                let context = task.get("context").unwrap_or(&Value::Null);
                let entity_matches = text(source, "entity_id") == text(context, "entity_id");
                let currency_matches = text(source, "currency") == text(context, "currency");
                let recorded_by_frontier = text(source, "recorded_at")
                    .zip(text(context, "knowledge_at"))
                    .map(|(recorded, frontier)| recorded <= frontier)
                    .unwrap_or(false);
                let period_start = text(context, "period_start").unwrap_or_default();
                let period_end = text(context, "period_end").unwrap_or_default();
                let valid_from = text(source, "valid_from");
                let valid_to = text(source, "valid_to");
                let valid_time_matches = valid_from.map(|from| from <= period_end).unwrap_or(true)
                    && valid_to.map(|to| to >= period_start).unwrap_or(true);
                let authority_qualified = text(source, "authority") == Some("qualified_source");
                binding_valid &= entity_matches
                    && currency_matches
                    && recorded_by_frontier
                    && valid_time_matches
                    && authority_qualified;
                if binding_valid {
                    qualified_source_ids.insert(source_id.clone());
                } else {
                    reasons.push("unqualified_source_promoted_to_decision_grade".to_owned());
                }
            } else if bool_value(binding, "qualifies") != Some(false) {
                binding_valid = false;
            }
        }
        if !binding_valid {
            reasons.push(format!("source_binding_invalid:{source_id}:{requirement_id}"));
        }
    }

    let required_sources = strings(array(expected, "required_source_ids"))
        .into_iter()
        .collect::<BTreeSet<_>>();
    let missing_sources = required_sources.difference(&bound_source_ids).cloned().collect::<Vec<_>>();
    check(
        &mut checks,
        &mut reasons,
        "required_source_lineage_present",
        missing_sources.is_empty(),
        format!("missing source IDs={missing_sources:?}"),
    );
    let required_requirements = strings(array(expected, "required_binding_requirements"))
        .into_iter()
        .collect::<BTreeSet<_>>();
    let missing_requirements = required_requirements
        .difference(&bound_requirement_ids)
        .cloned()
        .collect::<Vec<_>>();
    check(
        &mut checks,
        &mut reasons,
        "required_assertion_binding_present",
        missing_requirements.is_empty(),
        format!("missing requirement IDs={missing_requirements:?}"),
    );
    check(
        &mut checks,
        &mut reasons,
        "source_bindings_are_valid",
        !reasons.iter().any(|reason| reason.starts_with("source_binding_invalid:") || reason == "unqualified_source_promoted_to_decision_grade"),
        "source digest, revision, locator, entity, currency, authority and temporal frontier must match",
    );

    let expected_journals = array(expected, "expected_journal_entries");
    let actual_journals = array(response, "journal_entries");
    let mut journal_valid = expected_journals.len() == actual_journals.len();
    for expected_journal in expected_journals {
        let id = text(expected_journal, "entry_id").unwrap_or("<missing>");
        let actual = actual_journals.iter().find(|row| text(row, "entry_id") == Some(id));
        let Some(actual) = actual else {
            journal_valid = false;
            reasons.push(format!("journal_entry_missing:{id}"));
            continue;
        };
        journal_valid &= text(actual, "entity_id") == text(expected_journal, "entity_id")
            && text(actual, "period") == text(expected_journal, "period")
            && text(actual, "currency") == text(expected_journal, "currency")
            && text(actual, "policy_id") == text(expected_journal, "policy_id");
        let actual_sources = strings(array(actual, "source_ids")).into_iter().collect::<BTreeSet<_>>();
        journal_valid &= actual_sources.iter().all(|source_id| qualified_source_ids.contains(source_id));
        let (debits, credits, lines_valid) = journal_totals(actual);
        let expected_debit = amount(expected_journal, "debit_minor");
        let expected_credit = amount(expected_journal, "credit_minor");
        journal_valid &= lines_valid
            && debits == expected_debit
            && credits == expected_credit
            && debits == credits;
        if !lines_valid || debits != expected_debit || credits != expected_credit || debits != credits {
            reasons.push(format!("journal_amount_or_balance_invalid:{id}"));
        }
        let context = task.get("context").unwrap_or(&Value::Null);
        journal_valid &= text(actual, "entity_id") == text(context, "entity_id")
            && text(actual, "period") == text(context, "period_end")
            && text(actual, "currency") == text(context, "currency")
            && text(actual, "policy_id") == text(context, "accounting_policy_id");
    }
    check(
        &mut checks,
        &mut reasons,
        "exact_double_entry_and_policy_binding",
        journal_valid,
        "journal count, entity, period, denomination, policy, qualified source lineage and exact debit/credit totals must match",
    );

    let expected_statement = sorted_statement_lines(array(expected, "expected_statement_lines"));
    let actual_statement = sorted_statement_lines(
        &array(response, "statement_reconstruction")
            .iter()
            .flat_map(|row| array(row, "lines").iter().cloned())
            .collect::<Vec<_>>(),
    );
    let statement_context_valid = array(response, "statement_reconstruction").iter().all(|row| {
        text(row, "entity_id") == text(task.get("context").unwrap_or(&Value::Null), "entity_id")
            && text(row, "period") == text(task.get("context").unwrap_or(&Value::Null), "period_end")
            && text(row, "currency") == text(task.get("context").unwrap_or(&Value::Null), "currency")
    });
    check(
        &mut checks,
        &mut reasons,
        "independent_statement_reconstruction",
        expected_statement == actual_statement && statement_context_valid,
        format!("actual statement lines={actual_statement:?}; expected={expected_statement:?}"),
    );

    let status = if reasons.is_empty() && checks.iter().all(|row| row["passed"] == true) {
        "PASS"
    } else {
        "FAIL"
    };
    json!({
        "task_id": task_id,
        "status": status,
        "actual_disposition": response.get("disposition"),
        "expected_disposition": expected.get("disposition"),
        "checks": checks,
        "reason_codes": reasons
    })
}

fn unique_source_index<'a>(sources: &'a [Value], reasons: &mut Vec<String>) -> BTreeMap<String, &'a Value> {
    let mut map = BTreeMap::new();
    for source in sources {
        let id = text(source, "source_id").unwrap_or("<missing>").to_owned();
        if map.insert(id.clone(), source).is_some() {
            reasons.push(format!("duplicate_source_id:{id}"));
        }
        let actual = source.get("content")
            .and_then(|content| serde_json::to_vec(content).ok())
            .map(|bytes| sha256_hex(&bytes));
        if actual.as_deref() != text(source, "sha256") {
            reasons.push(format!("source_content_digest_mismatch:{id}"));
        }
    }
    map
}

fn journal_totals(journal: &Value) -> (u128, u128, bool) {
    let mut debits = 0u128;
    let mut credits = 0u128;
    let mut valid = true;
    for line in array(journal, "lines") {
        let debit = amount(line, "debit_minor");
        let credit = amount(line, "credit_minor");
        valid &= debit.is_some() && credit.is_some();
        let debit = debit.unwrap_or(0);
        let credit = credit.unwrap_or(0);
        valid &= (debit == 0) ^ (credit == 0);
        debits = match debits.checked_add(debit) {
            Some(value) => value,
            None => { valid = false; debits }
        };
        credits = match credits.checked_add(credit) {
            Some(value) => value,
            None => { valid = false; credits }
        };
    }
    (debits, credits, valid && !array(journal, "lines").is_empty())
}

fn sorted_statement_lines(rows: &[Value]) -> Vec<(String, String)> {
    let mut result = rows.iter().map(|row| (
        text(row, "line_id").unwrap_or_default().to_owned(),
        text(row, "amount_minor").unwrap_or_default().to_owned()
    )).collect::<Vec<_>>();
    result.sort();
    result
}

fn check(checks: &mut Vec<Value>, reasons: &mut Vec<String>, invariant: &str, passed: bool, detail: impl Into<String>) {
    if !passed {
        reasons.push(invariant.to_owned());
    }
    checks.push(json!({
        "invariant": invariant,
        "passed": passed,
        "detail": detail.into()
    }));
}

fn array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
    value.get(key).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}

fn text<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn bool_value(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn strings(values: &[Value]) -> Vec<String> {
    values.iter().filter_map(Value::as_str).map(str::to_owned).collect()
}

fn sorted_strings(values: &[Value]) -> Vec<String> {
    let mut result = strings(values);
    result.sort();
    result
}

fn amount(value: &Value, key: &str) -> Option<u128> {
    let raw = text(value, key)?;
    if raw.is_empty() || (raw.len() > 1 && raw.starts_with('0')) || !raw.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    raw.parse::<u128>().ok()
}

fn sha256_hex(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (Value, Value, Value, Value, Value, Value) {
        (
            serde_json::from_str(include_str!("../../tasks_v1.0.0.json")).expect("tasks"),
            serde_json::from_str(include_str!("../../task.schema.json")).expect("task schema"),
            serde_json::from_str(include_str!("../../oracle_v1.0.0.json")).expect("oracle"),
            serde_json::from_str(include_str!("../../oracle.schema.json")).expect("oracle schema"),
            serde_json::from_str(include_str!("../../reference_responses_v1.0.0.json")).expect("responses"),
            serde_json::from_str(include_str!("../../response.schema.json")).expect("response schema"),
        )
    }

    fn report(data: &(Value, Value, Value, Value, Value, Value)) -> Value {
        let bytes = data.0.to_string();
        verify(bytes.as_bytes(), data.1.to_string().as_bytes(), data.2.to_string().as_bytes(),
            data.3.to_string().as_bytes(), data.4.to_string().as_bytes(), data.5.to_string().as_bytes())
    }

    #[test]
    fn public_s0_reference_responses_conform() {
        let data = fixture();
        let report = report(&data);
        assert_eq!(report["status"], "PASS", "{report:#}");
        assert_eq!(report["case_count"], 10);
        assert_eq!(report["passed_case_count"], 10);
    }

    #[test]
    fn wrong_disposition_is_rejected() {
        let mut data = fixture();
        data.4["responses"][0]["disposition"] = Value::String("deliver".to_owned());
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn redundant_clarification_is_rejected() {
        let mut data = fixture();
        data.4["responses"][0]["questions"].as_array_mut().unwrap().push(json!({
            "question_id":"Q-REDUNDANT",
            "text":"Can you confirm the invoice amount?",
            "resolves":["UNK-APPROVAL-001"],
            "target":"user"
        }));
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn one_minor_unit_journal_mutation_is_rejected() {
        let mut data = fixture();
        data.4["responses"][1]["journal_entries"][0]["lines"][0]["debit_minor"] =
            Value::String("125001".to_owned());
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn unqualified_source_cannot_be_promoted_into_a_journal() {
        let mut data = fixture();
        data.4["responses"][1]["source_bindings"][1]["qualifies"] = Value::Bool(false);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn future_recorded_source_cannot_be_qualified_at_historical_frontier() {
        let mut data = fixture();
        data.4["responses"][7]["source_bindings"][0]["qualifies"] = Value::Bool(true);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn cross_entity_source_cannot_be_qualified_for_the_requested_books() {
        let mut data = fixture();
        data.4["responses"][5]["source_bindings"][0]["qualifies"] = Value::Bool(true);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn changed_source_bytes_fail_digest_verification() {
        let mut data = fixture();
        data.0["tasks"][0]["sources"][0]["content"]["invoice_number"] =
            Value::String("MUTATED".to_owned());
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn statement_line_mutation_is_rejected() {
        let mut data = fixture();
        data.4["responses"][1]["statement_reconstruction"][0]["lines"][0]["amount_minor"] =
            Value::String("125001".to_owned());
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn no_response_can_claim_complete_books() {
        let mut data = fixture();
        data.4["responses"][8]["claims_complete_books"] = Value::Bool(true);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn repeated_verification_is_deterministic() {
        let data = fixture();
        assert_eq!(report(&data), report(&data));
    }
}
