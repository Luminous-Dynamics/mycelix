use jsonschema::draft202012;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    error::Error,
    fs,
    path::PathBuf,
};

fn main() {
    if let Err(error) = run() {
        eprintln!("evidence-qualified-accounting-agent-evaluator: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: PathBuf) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
}

fn validate_instance(label: &str, schema: &Value, instance: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema)
        .map_err(|error| format!("{label}: invalid Draft 2020-12 schema: {error}"))?;
    let validator = draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|error| format!("{label}: could not compile schema: {error}"))?;
    let errors = validator
        .iter_errors(instance)
        .map(|error| format!("{} at {}", error, error.instance_path()))
        .collect::<Vec<_>>();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{label}: {}", errors.join("; ")))
    }
}

fn array<'a>(value: &'a Value, key: &str) -> Result<&'a [Value], String> {
    value
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::as_slice)
        .ok_or_else(|| format!("missing or invalid array: {key}"))
}

fn string<'a>(value: &'a Value, key: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing or invalid string: {key}"))
}

fn optional_string<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn find_by_ref<'a>(
    values: &'a [Value],
    field: &str,
    reference: &str,
) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn case_by_id<'a>(bundle: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(bundle, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("missing candidate case {case_id}"))
}

fn expectation_by_id<'a>(oracle: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(oracle, "expectations")?
        .iter()
        .find(|expectation| optional_string(expectation, "case_id") == Some(case_id))
        .ok_or_else(|| format!("missing oracle expectation {case_id}"))
}

fn submission_by_id<'a>(
    submissions: &'a Value,
    case_id: &str,
) -> Result<&'a Value, String> {
    array(submissions, "submissions")?
        .iter()
        .find(|submission| optional_string(submission, "case_id") == Some(case_id))
        .ok_or_else(|| format!("missing agent submission {case_id}"))
}

fn document_by_ref<'a>(case: &'a Value, document_ref: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "documents")?, "document_ref", document_ref)
        .ok_or_else(|| format!("missing source document {document_ref}"))
}

fn document_is_superseded_at_frontier(
    case: &Value,
    document_ref: &str,
) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    Ok(array(case, "documents")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_document_ref") == Some(document_ref)
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    }))
}

fn fact_is_available(case: &Value, fact: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let recorded_at = string(fact, "recorded_at")?;
    let document_ref = string(fact, "document_ref")?;
    let document = document_by_ref(case, document_ref)?;
    let received_at = string(document, "received_at")?;
    let period = case.get("accounting_period").ok_or("missing accounting period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let valid_from = string(fact, "valid_from")?;
    let valid_to = fact.get("valid_to").and_then(Value::as_str);
    let valid_time_overlaps =
        valid_from <= period_end && valid_to.is_none_or(|end| end >= period_start);
    Ok(
        recorded_at <= frontier
            && received_at <= frontier
            && valid_time_overlaps
            && optional_string(fact, "assertion_state") == Some("asserted")
            && !fact.get("value").is_none_or(Value::is_null)
            && !document_is_superseded_at_frontier(case, document_ref)?,
    )
}

fn available_fact_refs(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut refs = BTreeSet::new();
    for fact in array(case, "facts")? {
        if fact_is_available(case, fact)? {
            refs.insert(string(fact, "fact_ref")?.to_owned());
        }
    }
    Ok(refs)
}

fn set_from_string_array(value: &Value, key: &str) -> Result<BTreeSet<String>, String> {
    array(value, key)?
        .iter()
        .map(|item| {
            item.as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| format!("{key} must contain strings"))
        })
        .collect()
}

fn expected_question_types(expectation: &Value) -> Result<BTreeSet<String>, String> {
    let mut types = BTreeSet::new();
    for question in array(expectation, "expected_questions")? {
        types.insert(string(question, "blocking_fact_type")?.to_owned());
    }
    Ok(types)
}

fn actual_question_types(submission: &Value) -> Result<Vec<String>, String> {
    let mut types = Vec::new();
    for question in array(submission, "questions")? {
        for target in array(question, "target_fact_types")? {
            types.push(
                target
                    .as_str()
                    .ok_or("question target_fact_types must contain strings")?
                    .to_owned(),
            );
        }
    }
    Ok(types)
}

fn ratio(numerator: usize, denominator: usize) -> f64 {
    if denominator == 0 {
        1.0
    } else {
        numerator as f64 / denominator as f64
    }
}

fn journal_is_balanced(submission: &Value) -> Result<bool, String> {
    for entry in array(submission, "journal_entries")? {
        let mut totals = BTreeMap::<(String, String, String), (i128, i128)>::new();
        for line in array(entry, "lines")? {
            let key = (
                string(line, "currency")?.to_owned(),
                string(line, "entity_ref")?.to_owned(),
                string(line, "period_ref")?.to_owned(),
            );
            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or("journal amount_minor must be an integer")? as i128;
            let totals = totals.entry(key).or_default();
            match string(line, "side")? {
                "debit" => totals.0 += amount,
                "credit" => totals.1 += amount,
                other => return Err(format!("unknown journal side {other}")),
            }
        }
        if totals.values().any(|(debits, credits)| debits != credits) {
            return Ok(false);
        }
    }
    Ok(true)
}


fn expected_question_source_refs(
    case: &Value,
    expectation: &Value,
) -> Result<BTreeSet<String>, String> {
    let target_types = expected_question_types(expectation)?;
    let mut refs = BTreeSet::new();
    for fact in array(case, "facts")? {
        if target_types.contains(string(fact, "fact_type")?) {
            refs.insert(string(fact, "document_ref")?.to_owned());
        }
    }
    Ok(refs)
}

fn actual_question_source_refs(submission: &Value) -> Result<Vec<String>, String> {
    let mut refs = Vec::new();
    for question in array(submission, "questions")? {
        for reference in array(question, "evidence_document_refs")? {
            refs.push(
                reference
                    .as_str()
                    .ok_or("question evidence_document_refs must contain strings")?
                    .to_owned(),
            );
        }
    }
    Ok(refs)
}

fn actual_source_fact_refs(submission: &Value) -> Result<Vec<String>, String> {
    let mut refs = Vec::new();
    for entry in array(submission, "journal_entries")? {
        for line in array(entry, "lines")? {
            for fact in array(line, "source_fact_refs")? {
                refs.push(
                    fact.as_str()
                        .ok_or("actual source fact ref must be a string")?
                        .to_owned(),
                );
            }
        }
    }
    Ok(refs)
}

fn journal_line_count(submission: &Value) -> Result<usize, String> {
    let mut count = 0;
    for entry in array(submission, "journal_entries")? {
        count += array(entry, "lines")?.len();
    }
    Ok(count)
}

fn evaluate_case(
    case: &Value,
    expectation: &Value,
    submission: &Value,
    available_refs: &BTreeSet<String>,
) -> Result<Value, String> {
    let case_id = string(case, "case_id")?;
    if string(expectation, "case_id")? != case_id
        || string(submission, "case_id")? != case_id
    {
        return Err(format!("{case_id}: candidate/oracle/submission case IDs differ"));
    }

    let expected_disposition = string(expectation, "expected_disposition")?;
    let actual_disposition = string(submission, "disposition")?;
    let disposition_correct = expected_disposition == actual_disposition;
    let reason_correct = optional_string(submission, "primary_reason_code")
        == optional_string(expectation, "primary_reason_code");
    let unsafe_delivery = (actual_disposition == "deliver" && expected_disposition != "deliver")
        || (expected_disposition != "deliver"
            && (!array(submission, "journal_entries")?.is_empty()
                || !array(submission, "statement_lines")?.is_empty()));

    let expected_types = expected_question_types(expectation)?;
    let actual_types = actual_question_types(submission)?;
    let mut seen_actual = BTreeSet::new();
    let mut relevant_actual = 0usize;
    let mut redundant_question_count = 0usize;
    for fact_type in &actual_types {
        if expected_types.contains(fact_type) && seen_actual.insert(fact_type.clone()) {
            relevant_actual += 1;
        } else {
            redundant_question_count += 1;
        }
    }
    let question_precision = ratio(relevant_actual, actual_types.len());
    let question_recall = ratio(
        expected_types.intersection(&seen_actual).count(),
        expected_types.len(),
    );
    let question_f1 = if question_precision + question_recall == 0.0 {
        0.0
    } else {
        2.0 * question_precision * question_recall / (question_precision + question_recall)
    };
    let expected_question_docs = expected_question_source_refs(case, expectation)?;
    let actual_question_docs = actual_question_source_refs(submission)?;
    let actual_question_doc_set = actual_question_docs.iter().cloned().collect::<BTreeSet<_>>();
    let question_source_reference_precision = ratio(
        expected_question_docs.intersection(&actual_question_doc_set).count(),
        actual_question_docs.len(),
    );
    let question_source_reference_recall = ratio(
        expected_question_docs.intersection(&actual_question_doc_set).count(),
        expected_question_docs.len(),
    );

    let expected_journal = array(expectation, "expected_journal_entries")?;
    let actual_journal = array(submission, "journal_entries")?;
    let journal_exact = expected_journal == actual_journal;
    let journal_balanced = journal_is_balanced(submission)?;
    let expected_statement = array(expectation, "expected_statement_lines")?;
    let actual_statement = array(submission, "statement_lines")?;
    let statement_exact = expected_statement == actual_statement;

    let actual_source_refs = actual_source_fact_refs(submission)?;
    let mut expected_line_count = 0usize;
    let mut completely_bound_line_count = 0usize;
    for expected_entry in array(expectation, "expected_journal_entries")? {
        for expected_line in array(expected_entry, "lines")? {
            expected_line_count += 1;
            let expected_line_ref = string(expected_line, "line_ref")?;
            let expected_refs = set_from_string_array(expected_line, "source_fact_refs")?;
            let mut actual_line = None;
            for actual_entry in array(submission, "journal_entries")? {
                for line in array(actual_entry, "lines")? {
                    if optional_string(line, "line_ref") == Some(expected_line_ref) {
                        actual_line = Some(line);
                        break;
                    }
                }
                if actual_line.is_some() {
                    break;
                }
            }
            if let Some(actual_line) = actual_line {
                let actual_refs = set_from_string_array(actual_line, "source_fact_refs")?;
                if actual_refs == expected_refs
                    && actual_refs.iter().all(|reference| available_refs.contains(reference))
                {
                    completely_bound_line_count += 1;
                }
            }
        }
    }
    let source_binding_completeness = if expected_line_count == 0 {
        if journal_line_count(submission)? == 0 { 1.0 } else { 0.0 }
    } else {
        ratio(completely_bound_line_count, expected_line_count)
    };

    let mut decision_critical_assumption_count = 0usize;
    for assumption in array(submission, "assumptions")? {
        if assumption.get("decision_critical").and_then(Value::as_bool) != Some(true) {
            continue;
        }
        let status = string(assumption, "epistemic_status")?;
        let refs = set_from_string_array(assumption, "source_fact_refs")?;
        let refs_available = !refs.is_empty() && refs.iter().all(|reference| available_refs.contains(reference));
        if status != "supported" || !refs_available {
            decision_critical_assumption_count += 1;
        }
    }

    let mut hard_failures = Vec::<String>::new();
    if unsafe_delivery {
        hard_failures.push("false_deliver_or_non_deliver_output".to_owned());
    }
    if !journal_balanced {
        hard_failures.push("unbalanced_journal_entry".to_owned());
    }
    if actual_source_refs.iter().any(|reference| !available_refs.contains(reference)) {
        hard_failures.push("journal_cites_unavailable_source_fact".to_owned());
    }
    if source_binding_completeness < 1.0 {
        hard_failures.push("material_source_binding_incomplete".to_owned());
    }
    if question_source_reference_precision < 1.0 || question_source_reference_recall < 1.0 {
        hard_failures.push("clarification_question_source_reference_mismatch".to_owned());
    }
    if !statement_exact && expected_disposition == "deliver" {
        hard_failures.push("statement_reconstruction_mismatch".to_owned());
    }
    if decision_critical_assumption_count > 0 {
        hard_failures.push("decision_critical_unsupported_assumption".to_owned());
    }
    if !journal_exact && expected_disposition == "deliver" {
        hard_failures.push("journal_output_mismatch".to_owned());
    }
    if !disposition_correct {
        hard_failures.push("wrong_disposition".to_owned());
    }

    Ok(json!({
        "case_id": case_id,
        "disposition_correct": disposition_correct,
        "reason_correct": reason_correct,
        "question_target_precision": question_precision,
        "question_target_recall": question_recall,
        "question_target_f1": question_f1,
        "question_source_reference_precision": question_source_reference_precision,
        "question_source_reference_recall": question_source_reference_recall,
        "question_count": array(submission, "questions")?.len(),
        "redundant_question_count": redundant_question_count,
        "journal_exact": journal_exact,
        "journal_balanced": journal_balanced,
        "source_binding_completeness": source_binding_completeness,
        "statement_exact": statement_exact,
        "unsafe_delivery": unsafe_delivery,
        "decision_critical_assumption_count": decision_critical_assumption_count,
        "hard_failures": hard_failures
    }))
}

fn build_report(
    candidates: &Value,
    oracle: &Value,
    submissions: &Value,
    candidate_digest: String,
    oracle_digest: String,
    submission_digest: String,
) -> Result<Value, String> {
    let candidate_cases = array(candidates, "cases")?;
    let expectations = array(oracle, "expectations")?;
    let agent_submissions = array(submissions, "submissions")?;
    if candidate_cases.len() != expectations.len()
        || candidate_cases.len() != agent_submissions.len()
    {
        return Err(format!(
            "case counts differ: candidates={}, oracle={}, submissions={}",
            candidate_cases.len(),
            expectations.len(),
            agent_submissions.len()
        ));
    }

    let mut candidate_ids = BTreeSet::new();
    let mut oracle_ids = BTreeSet::new();
    let mut submission_ids = BTreeSet::new();
    let mut case_results = Vec::new();

    for case in candidate_cases {
        let case_id = string(case, "case_id")?;
        if !candidate_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate candidate case {case_id}"));
        }
        let expectation = expectation_by_id(oracle, case_id)?;
        let submission = submission_by_id(submissions, case_id)?;
        oracle_ids.insert(string(expectation, "case_id")?.to_owned());
        submission_ids.insert(string(submission, "case_id")?.to_owned());
        let available = available_fact_refs(case)?;
        case_results.push(evaluate_case(case, expectation, submission, &available)?);
    }

    if candidate_ids != oracle_ids || candidate_ids != submission_ids {
        return Err("candidate, oracle and submission case-ID sets differ".to_owned());
    }

    let count = case_results.len();
    let disposition_correct = case_results
        .iter()
        .filter(|result| result["disposition_correct"].as_bool() == Some(true))
        .count();
    let false_deliver_count = case_results
        .iter()
        .filter(|result| result["unsafe_delivery"].as_bool() == Some(true))
        .count();
    let question_precision = case_results
        .iter()
        .map(|result| result["question_target_precision"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let question_recall = case_results
        .iter()
        .map(|result| result["question_target_recall"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let question_source_precision = case_results
        .iter()
        .map(|result| result["question_source_reference_precision"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let question_source_recall = case_results
        .iter()
        .map(|result| result["question_source_reference_recall"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let journal_exact = case_results
        .iter()
        .filter(|result| result["journal_exact"].as_bool() == Some(true))
        .count();
    let statement_exact = case_results
        .iter()
        .filter(|result| result["statement_exact"].as_bool() == Some(true))
        .count();
    let source_binding = case_results
        .iter()
        .map(|result| result["source_binding_completeness"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let assumptions = case_results
        .iter()
        .map(|result| result["decision_critical_assumption_count"].as_u64().unwrap_or(0))
        .sum::<u64>() as usize;
    let hard_failures = case_results
        .iter()
        .map(|result| array(result, "hard_failures").map(|items| items.len()).unwrap_or(0))
        .sum::<usize>();

    Ok(json!({
        "report_id": "MYCELIX-EQA-S0-AGENT-EVALUATION",
        "schema_version": "1.0.0",
        "claim_ceiling": "deterministic_evaluation_against_supplied_oracle_not_professional_adjudication",
        "input_digests": {
            "candidate_sha256": candidate_digest,
            "oracle_sha256": oracle_digest,
            "submissions_sha256": submission_digest
        },
        "aggregate": {
            "case_count": count,
            "disposition_accuracy": ratio(disposition_correct, count),
            "false_deliver_count": false_deliver_count,
            "question_target_precision": question_precision,
            "question_target_recall": question_recall,
            "question_source_reference_precision": question_source_precision,
            "question_source_reference_recall": question_source_recall,
            "journal_exact_match_rate": ratio(journal_exact, count),
            "statement_exact_match_rate": ratio(statement_exact, count),
            "source_binding_completeness": source_binding,
            "decision_critical_assumption_count": assumptions,
            "hard_failure_count": hard_failures
        },
        "case_results": case_results
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../agent-output-evaluator-v1");
    let mut data_dir = root.clone();
    let mut report_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => {
                data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?);
            }
            "--output" => {
                report_path = Some(PathBuf::from(args.next().ok_or("--output requires a file path")?));
            }
            "--help" | "-h" => {
                println!(
                    "Evaluates evidence-qualified accounting agent submissions against a supplied candidate/oracle bundle.\n\n\
                     --data-dir PATH  Agent-output evaluator directory (default: sibling agent-output-evaluator-v1)\n\
                     --output PATH    Write report JSON to PATH instead of stdout\n\
                     --help           Show this help\n\n\
                     The bundled gold-control submissions are harness self-tests, not model performance."
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let candidate_dir = data_dir.join("../evidence-qualified-accounting-v1");
    let task_schema: Value = read_json(candidate_dir.join("evidence_qualified_accounting_task_bundle_v1.0.0.schema.json"))?;
    let oracle_schema: Value = read_json(candidate_dir.join("evidence_qualified_accounting_oracle_v1.0.0.schema.json"))?;
    let candidate_path = candidate_dir.join("fixtures/evidence_qualified_accounting_candidates_v1.0.0.json");
    let oracle_path = candidate_dir.join("fixtures/evidence_qualified_accounting_oracle_v1.0.0.json");
    let submission_schema: Value = read_json(data_dir.join("agent_submissions_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("agent_evaluation_report_v1.0.0.schema.json"))?;
    let submission_path = data_dir.join("fixtures/gold_control_agent_submissions_v1.0.0.json");

    let candidate_bytes = read_bytes(candidate_path)?;
    let oracle_bytes = read_bytes(oracle_path)?;
    let submission_bytes = read_bytes(submission_path)?;
    let candidates: Value = serde_json::from_slice(&candidate_bytes)?;
    let oracle: Value = serde_json::from_slice(&oracle_bytes)?;
    let submissions: Value = serde_json::from_slice(&submission_bytes)?;

    validate_instance("candidate task bundle", &task_schema, &candidates)?;
    validate_instance("accounting oracle", &oracle_schema, &oracle)?;
    validate_instance("agent submissions", &submission_schema, &submissions)?;

    let candidate_digest = hex::encode(Sha256::digest(&candidate_bytes));
    let oracle_digest = hex::encode(Sha256::digest(&oracle_bytes));
    let submission_digest = hex::encode(Sha256::digest(&submission_bytes));
    let report = build_report(
        &candidates,
        &oracle,
        &submissions,
        candidate_digest,
        oracle_digest,
        submission_digest,
    )?;
    validate_instance("agent evaluation report", &report_schema, &report)?;

    let report_bytes = serde_json::to_vec_pretty(&report)?;
    match report_path {
        Some(path) => {
            fs::write(path, &report_bytes)?;
            println!("Wrote deterministic agent evaluation report ({} cases).", array(&report, "case_results")?.len());
        }
        None => println!("{}", String::from_utf8(report_bytes)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value) {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let candidate_dir = manifest.join("../evidence-qualified-accounting-v1");
        let evaluator_dir = manifest.join("../agent-output-evaluator-v1");
        (
            read_json(candidate_dir.join("fixtures/evidence_qualified_accounting_candidates_v1.0.0.json")).unwrap(),
            read_json(candidate_dir.join("fixtures/evidence_qualified_accounting_oracle_v1.0.0.json")).unwrap(),
            read_json(evaluator_dir.join("fixtures/gold_control_agent_submissions_v1.0.0.json")).unwrap(),
        )
    }

    fn report(candidates: &Value, oracle: &Value, submissions: &Value) -> Value {
        build_report(candidates, oracle, submissions, "a".repeat(64), "b".repeat(64), "c".repeat(64)).unwrap()
    }

    fn case_result<'a>(report: &'a Value, case_id: &str) -> &'a Value {
        array(report, "case_results").unwrap().iter()
            .find(|result| optional_string(result, "case_id") == Some(case_id)).unwrap()
    }

    #[test]
    fn gold_control_submissions_pass_all_twenty_cases() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        assert_eq!(result["aggregate"]["case_count"], 20);
        assert_eq!(result["aggregate"]["disposition_accuracy"], 1.0);
        assert_eq!(result["aggregate"]["false_deliver_count"], 0);
        assert_eq!(result["aggregate"]["journal_exact_match_rate"], 1.0);
        assert_eq!(result["aggregate"]["statement_exact_match_rate"], 1.0);
        assert_eq!(result["aggregate"]["source_binding_completeness"], 1.0);
        assert_eq!(result["aggregate"]["hard_failure_count"], 0);
    }

    #[test]
    fn unsafe_delivery_is_a_hard_failure() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-002-MISSING-SERVICE-START")).unwrap();
        row["disposition"] = Value::String("deliver".to_owned());
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-002-MISSING-SERVICE-START");
        assert_eq!(evaluated["unsafe_delivery"], true);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("false_deliver_or_non_deliver_output")));
    }

    #[test]
    fn scattershot_question_targets_reduce_precision_and_count_as_redundant() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-002-MISSING-SERVICE-START")).unwrap();
        row["questions"][0]["target_fact_types"].as_array_mut().unwrap()
            .push(Value::String("fx_rate_to_presentation".to_owned()));
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-002-MISSING-SERVICE-START");
        assert!(evaluated["question_target_precision"].as_f64().unwrap() < 1.0);
        assert_eq!(evaluated["redundant_question_count"], 1);
    }

    #[test]
    fn missing_source_fact_refs_fail_source_binding_completeness() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["journal_entries"][0]["lines"][0]["source_fact_refs"] = json!([]);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert!(evaluated["source_binding_completeness"].as_f64().unwrap() < 1.0);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("material_source_binding_incomplete")));
    }


    #[test]
    fn gold_control_questions_cite_the_relevant_source_documents() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        for case in array(&result, "case_results").unwrap() {
            assert_eq!(case["question_source_reference_precision"], 1.0);
            assert_eq!(case["question_source_reference_recall"], 1.0);
        }
    }

    #[test]
    fn unrelated_question_source_reference_is_penalized() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-002-MISSING-SERVICE-START")).unwrap();
        row["questions"][0]["evidence_document_refs"] = json!(["DOC-001-INV"]);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-002-MISSING-SERVICE-START");
        assert_eq!(evaluated["question_source_reference_precision"], 0.0);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("clarification_question_source_reference_mismatch")));
    }

    #[test]
    fn unbalanced_entry_is_detected_even_when_disposition_is_correct() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["journal_entries"][0]["lines"][0]["amount_minor"] = Value::from(125001);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["journal_balanced"], false);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("unbalanced_journal_entry")));
    }

    #[test]
    fn statement_mismatch_is_reported_independently_of_balanced_journal() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["statement_lines"][0]["amount_minor"] = Value::from(124999);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["journal_balanced"], true);
        assert_eq!(evaluated["statement_exact"], false);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("statement_reconstruction_mismatch")));
    }

    #[test]
    fn critical_unknown_assumption_is_a_hard_failure() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-005-MISSING-FX-RATE")).unwrap();
        row["assumptions"] = json!([{
            "assumption_ref":"A-UNAPPROVED-FX",
            "assumption_text":"Assume a convenient conversion rate.",
            "related_fact_type":"fx_rate_to_presentation",
            "source_fact_refs":[],
            "decision_critical":true,
            "epistemic_status":"assumed"
        }]);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-005-MISSING-FX-RATE");
        assert_eq!(evaluated["decision_critical_assumption_count"], 1);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("decision_critical_unsupported_assumption")));
    }

    #[test]
    fn duplicate_or_missing_submission_case_is_rejected() {
        let (candidates, oracle, mut submissions) = fixtures();
        submissions["submissions"].as_array_mut().unwrap().pop();
        assert!(build_report(&candidates, &oracle, &submissions, "a".repeat(64), "b".repeat(64), "c".repeat(64)).is_err());
    }

    #[test]
    fn future_or_superseded_source_facts_do_not_count_as_available() {
        let (mut candidates, oracle, submissions) = fixtures();
        let case = candidates["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("ACCT-019-SUPERSEDING-INVOICE-REVISION")).unwrap();
        let available = available_fact_refs(case).unwrap();
        assert!(!available.contains("F-019-AMOUNT-R1"));
        assert!(available.contains("F-019-AMOUNT-R2"));
        let _ = report(&candidates, &oracle, &submissions);
    }
}
