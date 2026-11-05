#![forbid(unsafe_code)]

use jsonschema::draft202012;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const TASKS_DEFAULT: &str = "../tasks_v1.1.0.json";
const TASK_SCHEMA_DEFAULT: &str = "../task.schema.v1.1.0.json";
const ORACLE_DEFAULT: &str = "../oracle_v1.1.0.json";
const ORACLE_SCHEMA_DEFAULT: &str = "../oracle.schema.v1.1.0.json";
const RESPONSES_DEFAULT: &str = "../reference_responses_v1.1.0.json";
const RESPONSE_SCHEMA_DEFAULT: &str = "../response.schema.v1.1.0.json";

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("fin-trust-s1-verifier: {error}");
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
            "--oracle-schema" => {
                oracle_schema = PathBuf::from(next_arg(&mut args, "--oracle-schema")?)
            }
            "--responses" => responses = PathBuf::from(next_arg(&mut args, "--responses")?),
            "--response-schema" => {
                response_schema = PathBuf::from(next_arg(&mut args, "--response-schema")?)
            }
            "--out" => output = Some(PathBuf::from(next_arg(&mut args, "--out")?)),
            "--help" | "-h" => {
                println!(
                    "FIN-TRUST S1 verifier: --tasks --task-schema --oracle --oracle-schema --responses --response-schema --out"
                );
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
                "report_schema": "fin-trust-s1-verifier.report.v1.1.0",
                "benchmark_id": "FIN-TRUST-BENCH",
                "dataset_version": "1.1.0",
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
            match (
                task_map.get(task_id),
                oracle_map.get(task_id),
                response_map.get(task_id),
            ) {
                (Some(task), Some(expected), Some(response)) => {
                    results.push(verify_case(task_id, task, expected, response));
                }
                _ => errors.push(format!("{task_id}: missing task, oracle or response")),
            }
        }
    }

    let passed = results.iter().filter(|row| row["status"] == "PASS").count();
    let status = if errors.is_empty() && !results.is_empty() && passed == results.len() {
        "PASS"
    } else {
        "FAIL"
    };
    let mut report = json!({
        "report_schema": "fin-trust-s1-verifier.report.v1.1.0",
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
            report["errors"]
                .as_array_mut()
                .expect("errors array")
                .push(Value::String(format!(
                    "embedded report schema JSON: {error}"
                )));
            return report;
        }
    };
    let report_errors = validate_instance("report", &report_schema, &report);
    if !report_errors.is_empty() {
        report["status"] = Value::String("FAIL".to_owned());
        report["errors"]
            .as_array_mut()
            .expect("errors array")
            .push(Value::String(report_errors.join("; ")));
    }
    report
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Vec<String> {
    if let Err(error) = jsonschema::draft202012::meta::validate(schema) {
        return vec![format!("{label}: invalid Draft 2020-12 schema: {error}")];
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(value) => value,
        Err(error) => return vec![format!("{label}: cannot compile schema: {error}")],
    };
    validator
        .iter_errors(instance)
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
        "response disposition must equal the adjudicated task disposition",
    );

    let required_questions = sorted_strings(array(expected, "required_question_ids"));
    let forbidden_questions = strings(array(expected, "forbidden_question_ids"));
    let actual_question_rows = array(response, "questions");
    let actual_questions = actual_question_rows
        .iter()
        .filter_map(|row| text(row, "question_id").map(str::to_owned))
        .collect::<Vec<_>>();
    let mut actual_question_ids = actual_questions.clone();
    actual_question_ids.sort();
    check(
        &mut checks,
        &mut reasons,
        "minimum_clarification_set",
        actual_question_ids == required_questions,
        format!("actual={actual_question_ids:?} required={required_questions:?}"),
    );
    check(
        &mut checks,
        &mut reasons,
        "no_forbidden_or_redundant_questions",
        forbidden_questions
            .iter()
            .all(|id| !actual_question_ids.contains(id)),
        format!("forbidden question IDs={forbidden_questions:?}"),
    );

    let required_specs = array(expected, "required_question_specs");
    let mut question_specs_match = required_specs.len() == actual_question_rows.len();
    let mut seen_question_ids = BTreeSet::new();
    for question in actual_question_rows {
        let qid = text(question, "question_id").unwrap_or("<missing>");
        if !seen_question_ids.insert(qid.to_owned()) {
            question_specs_match = false;
        }
        let spec = required_specs
            .iter()
            .find(|row| text(row, "question_id") == Some(qid));
        match spec {
            Some(spec) => {
                question_specs_match &= text(question, "target") == text(spec, "target");
                question_specs_match &= sorted_strings(array(question, "resolves"))
                    == sorted_strings(array(spec, "resolves"));
            }
            None => question_specs_match = false,
        }
    }
    check(
        &mut checks,
        &mut reasons,
        "clarification_recipient_and_unknown_binding",
        question_specs_match,
        "each clarification must target the adjudicated recipient and resolve the adjudicated unknowns",
    );

    let task_unknowns = array(task, "known_unknowns")
        .iter()
        .filter_map(|row| text(row, "unknown_id").map(str::to_owned))
        .collect::<BTreeSet<_>>();
    let mut unknown_mapping_valid = true;
    for question in actual_question_rows {
        for unknown in strings(array(question, "resolves")) {
            if !task_unknowns.contains(&unknown) {
                unknown_mapping_valid = false;
            }
        }
    }
    check(
        &mut checks,
        &mut reasons,
        "questions_resolve_declared_unknowns",
        unknown_mapping_valid,
        "clarifications may resolve only unknowns declared by the task",
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
        "the bounded S0 evidence task cannot establish complete books",
    );

    let source_map = unique_source_index(array(task, "sources"), &mut reasons);
    let requirement_rows = array(task, "requirements");
    let mut requirement_map = BTreeMap::<String, &Value>::new();
    for requirement in requirement_rows {
        let id = text(requirement, "requirement_id")
            .unwrap_or("<missing>")
            .to_owned();
        if requirement_map.insert(id.clone(), requirement).is_some() {
            reasons.push(format!("duplicate_requirement_id:{id}"));
        }
    }

    let actual_bindings = array(response, "source_bindings");
    let expected_bindings = array(expected, "expected_evidence_bindings");
    let mut actual_binding_map = BTreeMap::<String, &Value>::new();
    let mut binding_contract_valid = actual_bindings.len() == expected_bindings.len();
    let mut bound_source_ids = BTreeSet::<String>::new();
    let mut bound_requirement_ids = BTreeSet::<String>::new();
    for binding in actual_bindings {
        let key = evidence_binding_key(binding);
        if key.contains("<missing>") || actual_binding_map.insert(key.clone(), binding).is_some() {
            binding_contract_valid = false;
            reasons.push(format!("duplicate_or_incomplete_evidence_binding:{key}"));
        }
        let source_id = text(binding, "source_id").unwrap_or("<missing>").to_owned();
        let requirement_id = text(binding, "requirement_id")
            .unwrap_or("<missing>")
            .to_owned();
        bound_source_ids.insert(source_id.clone());
        bound_requirement_ids.insert(requirement_id.clone());
        if !source_map.contains_key(&source_id) || !requirement_map.contains_key(&requirement_id) {
            binding_contract_valid = false;
            reasons.push(format!(
                "evidence_binding_unknown_source_or_requirement:{key}"
            ));
            continue;
        }
        let source = source_map.get(&source_id).expect("source checked above");
        binding_contract_valid &= text(binding, "revision") == text(source, "revision")
            && text(binding, "sha256") == text(source, "sha256")
            && text(binding, "locator")
                == text(source.get("locator").unwrap_or(&Value::Null), "location")
            && text(binding, "recorded_at") == text(source, "recorded_at");
    }
    for expected_binding in expected_bindings {
        let key = evidence_binding_key(expected_binding);
        match actual_binding_map.get(&key) {
            Some(actual) => {
                binding_contract_valid &= evidence_profile_equal(actual, expected_binding);
            }
            None => {
                binding_contract_valid = false;
                reasons.push(format!("expected_evidence_binding_missing:{key}"));
            }
        }
    }
    let required_sources = strings(array(expected, "required_source_ids"))
        .into_iter()
        .collect::<BTreeSet<_>>();
    let missing_sources = required_sources
        .difference(&bound_source_ids)
        .cloned()
        .collect::<Vec<_>>();
    check(
        &mut checks,
        &mut reasons,
        "required_source_lineage_present",
        missing_sources.is_empty(),
        format!("missing source IDs={missing_sources:?}"),
    );
    let required_binding_requirements = strings(array(expected, "required_binding_requirements"))
        .into_iter()
        .collect::<BTreeSet<_>>();
    let missing_binding_requirements = required_binding_requirements
        .difference(&bound_requirement_ids)
        .cloned()
        .collect::<Vec<_>>();
    check(
        &mut checks,
        &mut reasons,
        "required_assertion_bindings_present",
        missing_binding_requirements.is_empty(),
        format!("missing requirement IDs={missing_binding_requirements:?}"),
    );
    check(
        &mut checks,
        &mut reasons,
        "assertion_purpose_evidence_profile_matches_oracle",
        binding_contract_valid,
        "evidence class, polarity, purpose fit, origin, authentication, authority, time and transformation fields must match the v1.1 adjudication",
    );

    let actual_population = array(response, "population_assessments");
    let expected_population = array(expected, "expected_population_assessments");
    let population_match = population_assessments_equal(actual_population, expected_population);
    check(
        &mut checks,
        &mut reasons,
        "population_scope_and_control_totals_match",
        population_match,
        "population scope status, expected/observed counts and population reference must match",
    );

    let actual_requirement_assessments = array(response, "requirement_assessments");
    let expected_requirement_assessments = array(expected, "expected_requirement_assessments");
    let requirement_assessments_match = requirement_assessments_equal(
        actual_requirement_assessments,
        expected_requirement_assessments,
    );
    check(
        &mut checks,
        &mut reasons,
        "assertion_level_dispositions_match",
        requirement_assessments_match,
        "each task requirement must have the oracle disposition, reason codes and evidence references",
    );

    let mut decision_grade_source_ids = BTreeSet::<String>::new();
    let mut positive_disposition_guard = true;
    let mut assessment_ids = BTreeSet::new();
    for assessment in actual_requirement_assessments {
        let requirement_id = text(assessment, "requirement_id").unwrap_or("<missing>");
        if !assessment_ids.insert(requirement_id.to_owned()) {
            positive_disposition_guard = false;
            reasons.push(format!("duplicate_requirement_assessment:{requirement_id}"));
            continue;
        }
        let Some(requirement) = requirement_map.get(requirement_id) else {
            positive_disposition_guard = false;
            reasons.push(format!("requirement_assessment_unknown:{requirement_id}"));
            continue;
        };
        let evidence_refs = strings(array(assessment, "evidence_refs"));
        let mut selected = Vec::<&Value>::new();
        for evidence_id in &evidence_refs {
            let binding = actual_bindings.iter().find(|row| {
                text(row, "requirement_id") == Some(requirement_id)
                    && text(row, "evidence_id") == Some(evidence_id)
            });
            match binding {
                Some(binding) => selected.push(binding),
                None => {
                    positive_disposition_guard = false;
                    reasons.push(format!(
                        "assessment_references_unknown_evidence:{requirement_id}:{evidence_id}"
                    ));
                }
            }
        }
        if text(assessment, "disposition") == Some("supported_within_declared_scope") {
            let context = task.get("context").unwrap_or(&Value::Null);
            let obligations = strings(array(requirement, "required_obligations"));
            let mut obligation_ok = true;
            let has_purpose = selected
                .iter()
                .any(|row| bool_value(row, "purpose_fit") == Some(true));
            let has_auth = selected
                .iter()
                .any(|row| text(row, "source_authentication") == Some("qualified"));
            let has_authority = selected
                .iter()
                .any(|row| text(row, "source_authority") == Some("qualified"));
            let has_temporal = selected
                .iter()
                .any(|row| text(row, "temporal_fit") == Some("within_reporting_period"));
            let lineage_ok = !selected.is_empty()
                && selected.iter().all(|row| {
                    matches!(
                        text(row, "transformation_state"),
                        Some("original_preserved" | "derived_with_lineage")
                    )
                });
            for binding in &selected {
                let source_id = text(binding, "source_id").unwrap_or_default();
                let source = source_map.get(source_id).copied();
                let source = match source {
                    Some(source) => source,
                    None => {
                        obligation_ok = false;
                        continue;
                    }
                };
                let context_entity = text(context, "entity_id");
                let context_currency = text(context, "currency");
                let recorded_at = text(binding, "recorded_at");
                let knowledge_at = text(context, "knowledge_at");
                let valid_from = text(source, "valid_from");
                let valid_to = text(source, "valid_to");
                let period_start = text(context, "period_start").unwrap_or_default();
                let period_end = text(context, "period_end").unwrap_or_default();
                let temporal_known = recorded_at
                    .zip(knowledge_at)
                    .is_some_and(|(recorded, frontier)| recorded <= frontier);
                let valid_period = valid_from.map(|date| date <= period_end).unwrap_or(true)
                    && valid_to.map(|date| date >= period_start).unwrap_or(true);
                let support_grade = bool_value(binding, "purpose_fit") == Some(true)
                    && text(binding, "polarity") == Some("supports")
                    && text(binding, "source_authentication") == Some("qualified")
                    && text(binding, "source_authority") == Some("qualified")
                    && text(binding, "temporal_fit") == Some("within_reporting_period")
                    && matches!(
                        text(binding, "transformation_state"),
                        Some("original_preserved" | "derived_with_lineage")
                    )
                    && temporal_known
                    && valid_period
                    && text(source, "entity_id") == context_entity
                    && (context_currency.is_none() || text(source, "currency") == context_currency);
                if !support_grade {
                    obligation_ok = false;
                    reasons.push(format!("unsupported_evidence_used_for_supported_requirement:{requirement_id}:{source_id}"));
                } else {
                    decision_grade_source_ids.insert(source_id.to_owned());
                }
            }
            for obligation in obligations {
                let met = match obligation.as_str() {
                    "purpose_fit" => has_purpose,
                    "source_authentication" => has_auth,
                    "issuer_authority" => has_authority,
                    "temporal_coverage" => has_temporal,
                    "transformation_lineage" => lineage_ok,
                    "authorization" => selected.iter().any(|row| {
                        text(row, "evidence_class") == Some("approval_record")
                            && text(row, "source_authentication") == Some("qualified")
                            && text(row, "source_authority") == Some("qualified")
                            && text(row, "polarity") == Some("supports")
                            && bool_value(row, "purpose_fit") == Some(true)
                    }),
                    "independent_corroboration" => {
                        let origins = selected
                            .iter()
                            .filter(|row| {
                                text(row, "polarity") == Some("supports")
                                    && text(row, "source_authentication") == Some("qualified")
                                    && text(row, "source_authority") == Some("qualified")
                                    && bool_value(row, "purpose_fit") == Some(true)
                            })
                            .filter_map(|row| text(row, "origin_group_ref"))
                            .collect::<BTreeSet<_>>();
                        origins.len()
                            >= requirement
                                .get("min_independent_origins")
                                .and_then(Value::as_u64)
                                .unwrap_or(1) as usize
                    }
                    "population_completeness" => actual_population.iter().any(|row| {
                        text(row, "requirement_id") == Some(requirement_id)
                            && text(row, "scope_status") == Some("qualified")
                            && row.get("expected_count").and_then(Value::as_u64).is_some()
                            && row.get("expected_count") == row.get("observed_count")
                    }),
                    _ => false,
                };
                obligation_ok &= met;
                if !met {
                    reasons.push(format!(
                        "required_evidence_obligation_unmet:{requirement_id}:{obligation}"
                    ));
                }
            }
            if selected.is_empty() || !obligation_ok {
                positive_disposition_guard = false;
            }
        }
    }
    let task_requirement_ids = requirement_map.keys().cloned().collect::<BTreeSet<_>>();
    if assessment_ids != task_requirement_ids {
        positive_disposition_guard = false;
        reasons.push("requirement_assessment_population_mismatch".to_owned());
    }
    check(
        &mut checks,
        &mut reasons,
        "no_unsupported_assertion_upgrades",
        positive_disposition_guard,
        "supported dispositions require purpose-fit, qualified, in-period, current, lineage-preserved evidence and every declared obligation",
    );

    let expected_journals = array(expected, "expected_journal_entries");
    let actual_journals = array(response, "journal_entries");
    let mut journal_valid = expected_journals.len() == actual_journals.len();
    for expected_journal in expected_journals {
        let id = text(expected_journal, "entry_id").unwrap_or("<missing>");
        let actual = actual_journals
            .iter()
            .find(|row| text(row, "entry_id") == Some(id));
        let Some(actual) = actual else {
            journal_valid = false;
            reasons.push(format!("journal_entry_missing:{id}"));
            continue;
        };
        journal_valid &= text(actual, "entity_id") == text(expected_journal, "entity_id")
            && text(actual, "period") == text(expected_journal, "period")
            && text(actual, "currency") == text(expected_journal, "currency")
            && text(actual, "policy_id") == text(expected_journal, "policy_id");
        let actual_sources = strings(array(actual, "source_ids"))
            .into_iter()
            .collect::<BTreeSet<_>>();
        let expected_sources = strings(array(expected_journal, "source_ids"))
            .into_iter()
            .collect::<BTreeSet<_>>();
        journal_valid &= actual_sources == expected_sources
            && actual_sources
                .iter()
                .all(|source_id| decision_grade_source_ids.contains(source_id));
        let (debits, credits, lines_valid) = journal_totals(actual);
        let expected_debit = amount(expected_journal, "debit_minor");
        let expected_credit = amount(expected_journal, "credit_minor");
        journal_valid &= lines_valid
            && expected_debit == Some(debits)
            && expected_credit == Some(credits)
            && debits == credits;
        if !lines_valid
            || expected_debit != Some(debits)
            || expected_credit != Some(credits)
            || debits != credits
        {
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
        "journal count, exact source lineage, entity, period, currency, policy and debit/credit totals must match",
    );

    let expected_statement = sorted_statement_lines(array(expected, "expected_statement_lines"));
    let actual_statement = sorted_statement_lines(
        &array(response, "statement_reconstruction")
            .iter()
            .flat_map(|row| array(row, "lines").iter().cloned())
            .collect::<Vec<_>>(),
    );
    let context = task.get("context").unwrap_or(&Value::Null);
    let statement_context_valid = array(response, "statement_reconstruction")
        .iter()
        .all(|row| {
            text(row, "entity_id") == text(context, "entity_id")
                && text(row, "period") == text(context, "period_end")
                && text(row, "currency") == text(context, "currency")
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

fn evidence_binding_key(value: &Value) -> String {
    format!(
        "{}\0{}\0{}",
        text(value, "requirement_id").unwrap_or("<missing>"),
        text(value, "evidence_id").unwrap_or("<missing>"),
        text(value, "source_id").unwrap_or("<missing>")
    )
}

fn evidence_profile_equal(actual: &Value, expected: &Value) -> bool {
    let fields = [
        "requirement_id",
        "evidence_id",
        "source_id",
        "revision",
        "sha256",
        "locator",
        "recorded_at",
        "origin_group_ref",
        "evidence_class",
        "polarity",
        "source_authentication",
        "source_authority",
        "temporal_fit",
        "transformation_state",
    ];
    fields
        .iter()
        .all(|field| text(actual, field) == text(expected, field))
        && bool_value(actual, "purpose_fit") == bool_value(expected, "purpose_fit")
}

fn requirement_assessment_key(value: &Value) -> String {
    text(value, "requirement_id")
        .unwrap_or("<missing>")
        .to_owned()
}

fn requirement_assessments_equal(actual: &[Value], expected: &[Value]) -> bool {
    if actual.len() != expected.len() {
        return false;
    }
    let mut actual_map = BTreeMap::<String, &Value>::new();
    for row in actual {
        if actual_map
            .insert(requirement_assessment_key(row), row)
            .is_some()
        {
            return false;
        }
    }
    expected.iter().all(|row| {
        actual_map
            .get(&requirement_assessment_key(row))
            .is_some_and(|found| {
                text(found, "disposition") == text(row, "disposition")
                    && sorted_strings(array(found, "reason_codes"))
                        == sorted_strings(array(row, "reason_codes"))
                    && sorted_strings(array(found, "evidence_refs"))
                        == sorted_strings(array(row, "evidence_refs"))
            })
    })
}

fn population_assessments_equal(actual: &[Value], expected: &[Value]) -> bool {
    if actual.len() != expected.len() {
        return false;
    }
    let mut actual_map = BTreeMap::<String, &Value>::new();
    for row in actual {
        let id = text(row, "requirement_id")
            .unwrap_or("<missing>")
            .to_owned();
        if actual_map.insert(id, row).is_some() {
            return false;
        }
    }
    expected.iter().all(|row| {
        let id = text(row, "requirement_id").unwrap_or("<missing>");
        actual_map.get(id).is_some_and(|found| {
            text(found, "scope_status") == text(row, "scope_status")
                && found.get("expected_count") == row.get("expected_count")
                && found.get("observed_count") == row.get("observed_count")
                && found.get("population_ref") == row.get("population_ref")
        })
    })
}

fn unique_source_index<'a>(
    sources: &'a [Value],
    reasons: &mut Vec<String>,
) -> BTreeMap<String, &'a Value> {
    let mut map = BTreeMap::new();
    for source in sources {
        let id = text(source, "source_id").unwrap_or("<missing>").to_owned();
        if map.insert(id.clone(), source).is_some() {
            reasons.push(format!("duplicate_source_id:{id}"));
        }
        let actual = source
            .get("content")
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
            None => {
                valid = false;
                debits
            }
        };
        credits = match credits.checked_add(credit) {
            Some(value) => value,
            None => {
                valid = false;
                credits
            }
        };
    }
    (
        debits,
        credits,
        valid && !array(journal, "lines").is_empty(),
    )
}

fn sorted_statement_lines(rows: &[Value]) -> Vec<(String, String)> {
    let mut result = rows
        .iter()
        .map(|row| {
            (
                text(row, "line_id").unwrap_or_default().to_owned(),
                text(row, "amount_minor").unwrap_or_default().to_owned(),
            )
        })
        .collect::<Vec<_>>();
    result.sort();
    result
}

fn check(
    checks: &mut Vec<Value>,
    reasons: &mut Vec<String>,
    invariant: &str,
    passed: bool,
    detail: impl Into<String>,
) {
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
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

fn text<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn bool_value(value: &Value, key: &str) -> Option<bool> {
    value.get(key).and_then(Value::as_bool)
}

fn strings(values: &[Value]) -> Vec<String> {
    values
        .iter()
        .filter_map(Value::as_str)
        .map(str::to_owned)
        .collect()
}

fn sorted_strings(values: &[Value]) -> Vec<String> {
    let mut result = strings(values);
    result.sort();
    result
}

fn amount(value: &Value, key: &str) -> Option<u128> {
    let raw = text(value, key)?;
    if raw.is_empty()
        || (raw.len() > 1 && raw.starts_with('0'))
        || !raw.bytes().all(|byte| byte.is_ascii_digit())
    {
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
            serde_json::from_str(include_str!("../../tasks_v1.1.0.json")).expect("tasks"),
            serde_json::from_str(include_str!("../../task.schema.v1.1.0.json"))
                .expect("task schema"),
            serde_json::from_str(include_str!("../../oracle_v1.1.0.json")).expect("oracle"),
            serde_json::from_str(include_str!("../../oracle.schema.v1.1.0.json"))
                .expect("oracle schema"),
            serde_json::from_str(include_str!("../../reference_responses_v1.1.0.json"))
                .expect("responses"),
            serde_json::from_str(include_str!("../../response.schema.v1.1.0.json"))
                .expect("response schema"),
        )
    }

    fn report(data: &(Value, Value, Value, Value, Value, Value)) -> Value {
        let a = data.0.to_string();
        let b = data.1.to_string();
        let c = data.2.to_string();
        let d = data.3.to_string();
        let e = data.4.to_string();
        let f = data.5.to_string();
        verify(
            a.as_bytes(),
            b.as_bytes(),
            c.as_bytes(),
            d.as_bytes(),
            e.as_bytes(),
            f.as_bytes(),
        )
    }

    #[test]
    fn public_s0_v1_1_reference_responses_conform() {
        let data = fixture();
        let result = report(&data);
        assert_eq!(result["status"], "PASS", "{result:#}");
        assert_eq!(result["case_count"], 10);
        assert_eq!(result["passed_case_count"], 10);
    }

    #[test]
    fn wrong_clarification_recipient_is_rejected() {
        let mut data = fixture();
        data.4["responses"][0]["questions"][0]["target"] = Value::String("user".to_owned());
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn missing_assertion_purpose_profile_field_is_rejected() {
        let mut data = fixture();
        data.4["responses"][1]["source_bindings"][0]["purpose_fit"] = Value::Bool(false);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn a_forged_supported_disposition_cannot_upgrade_unknown_authentication() {
        let mut data = fixture();
        data.4["responses"][0]["requirement_assessments"][0]["disposition"] =
            Value::String("supported_within_declared_scope".to_owned());
        data.4["responses"][0]["requirement_assessments"][0]["reason_codes"] = json!([]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn mirrored_duplicate_origin_does_not_satisfy_independence() {
        let mut data = fixture();
        data.4["responses"][2]["requirement_assessments"][1]["disposition"] =
            Value::String("supported_within_declared_scope".to_owned());
        data.4["responses"][2]["requirement_assessments"][1]["reason_codes"] =
            json!(["independent_corroboration_met"]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn contradictory_authenticated_evidence_cannot_be_marked_supported() {
        let mut data = fixture();
        data.4["responses"][5]["requirement_assessments"][0]["disposition"] =
            Value::String("supported_within_declared_scope".to_owned());
        data.4["responses"][5]["requirement_assessments"][0]["reason_codes"] = json!([]);
        data.4["responses"][5]["requirement_assessments"][0]["evidence_refs"] =
            json!(["EV-INV-006"]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn future_recorded_evidence_cannot_support_historical_frontier() {
        let mut data = fixture();
        data.4["responses"][7]["requirement_assessments"][0]["disposition"] =
            Value::String("supported_within_declared_scope".to_owned());
        data.4["responses"][7]["requirement_assessments"][0]["reason_codes"] = json!([]);
        data.4["responses"][7]["requirement_assessments"][0]["evidence_refs"] =
            json!(["EV-CN-008"]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn a_population_gap_cannot_be_marked_complete() {
        let mut data = fixture();
        data.4["responses"][8]["population_assessments"][0]["scope_status"] =
            Value::String("qualified".to_owned());
        data.4["responses"][8]["population_assessments"][0]["observed_count"] = json!(12);
        data.4["responses"][8]["requirement_assessments"][0]["disposition"] =
            Value::String("supported_within_declared_scope".to_owned());
        data.4["responses"][8]["requirement_assessments"][0]["reason_codes"] = json!([]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn source_byte_mutation_fails_digest_verification() {
        let mut data = fixture();
        data.0["tasks"][0]["sources"][0]["content"]["invoice_number"] =
            Value::String("MUTATED".to_owned());
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
    fn journal_cannot_use_a_source_outside_its_supported_assertion_frontier() {
        let mut data = fixture();
        data.4["responses"][2]["journal_entries"][0]["source_ids"] = json!(["SRC-INV-003-B"]);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn no_response_can_claim_complete_books() {
        let mut data = fixture();
        data.4["responses"][9]["claims_complete_books"] = Value::Bool(true);
        assert_eq!(report(&data)["status"], "FAIL");
    }

    #[test]
    fn repeated_verification_is_deterministic() {
        let data = fixture();
        assert_eq!(report(&data), report(&data));
    }
}
