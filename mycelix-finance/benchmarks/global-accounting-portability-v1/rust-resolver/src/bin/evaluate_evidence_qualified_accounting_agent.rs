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


fn oracle_source_fact_refs(
    case: &Value,
    expectation: &Value,
) -> Result<BTreeSet<String>, String> {
    let mut refs = array(expectation, "required_source_fact_refs")?
        .iter()
        .map(|value| {
            value
                .as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| "required_source_fact_refs must contain strings".to_owned())
        })
        .collect::<Result<BTreeSet<_>, _>>()?;
    if optional_string(expectation, "primary_reason_code") == Some("source_integrity_failed") {
        let failed_docs = array(case, "documents")?
            .iter()
            .filter(|document| optional_string(document, "integrity_state") == Some("synthetic_mismatch"))
            .filter_map(|document| optional_string(document, "document_ref"))
            .collect::<BTreeSet<_>>();
        for fact in array(case, "facts")? {
            if optional_string(fact, "document_ref").is_some_and(|reference| failed_docs.contains(reference)) {
                refs.insert(string(fact, "fact_ref")?.to_owned());
            }
        }
    }
    Ok(refs)
}

fn expected_fact_evidence_state(
    case: &Value,
    expectation: &Value,
    fact: &Value,
    available_refs: &BTreeSet<String>,
) -> Result<&'static str, String> {
    let source = document_by_ref(case, string(fact, "document_ref")?)?;
    if optional_string(source, "integrity_state") == Some("synthetic_mismatch") {
        return Ok("integrity_failed");
    }
    if optional_string(source, "integrity_state") == Some("synthetic_not_checked") {
        return Ok("integrity_unverified");
    }
    let frontier = string(case, "knowledge_at")?;
    if string(fact, "recorded_at")? > frontier || string(source, "received_at")? > frontier {
        return Ok("future");
    }
    if array(case, "documents")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_document_ref") == optional_string(fact, "document_ref")
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    }) {
        return Ok("superseded");
    }
    match optional_string(fact, "assertion_state") {
        Some("unknown") => return Ok("unknown"),
        Some("disputed") => return Ok("disputed"),
        Some("invalidated") => return Ok("invalidated"),
        _ => {}
    }

    let fact_type = string(fact, "fact_type")?;
    let reason = optional_string(expectation, "primary_reason_code").unwrap_or("");
    if (reason == "contradictory_service_period_evidence"
        && matches!(fact_type, "service_period_start" | "service_period_end"))
        || (reason == "contradictory_amount_evidence"
            && matches!(fact_type, "invoice_total_minor" | "purchase_order_total_minor"))
        || (reason == "source_revision_lineage_missing" && fact_type == "invoice_total_minor")
    {
        return Ok("contradicting");
    }

    let mut values = BTreeSet::new();
    for other in array(case, "facts")? {
        if optional_string(other, "fact_type") == Some(fact_type)
            && optional_string(other, "assertion_state") == Some("asserted")
            && !other.get("value").is_none_or(Value::is_null)
            && fact_is_available(case, other)?
        {
            values.insert(other.get("value").ok_or("asserted fact missing value")?.to_string());
        }
    }
    if values.len() > 1 {
        return Ok("contradicting");
    }
    if !available_refs.contains(string(fact, "fact_ref")?) {
        let period = case.get("accounting_period").ok_or("missing accounting period")?;
        let period_start = string(period, "start")?;
        let period_end = string(period, "end")?;
        let valid_from = string(fact, "valid_from")?;
        let valid_to = fact.get("valid_to").and_then(Value::as_str);
        if valid_from > period_end || valid_to.is_some_and(|end| end < period_start) {
            return Ok("outside_period");
        }
        return Ok("future");
    }
    Ok("asserted")
}

fn expected_trace_terminal_state(
    disposition: &str,
    fact_ref: &str,
    fact_type: &str,
    expected_refs: &BTreeSet<String>,
    expected_question_types: &BTreeSet<String>,
) -> Result<&'static str, String> {
    if expected_refs.contains(fact_ref) {
        return match disposition {
            "deliver" => Ok("carried_to_journal"),
            "clarify" => Ok("carried_to_question"),
            "abstain" => Ok("abstained"),
            "escalate" => Ok("carried_to_review"),
            other => Err(format!("unsupported oracle disposition {other}")),
        };
    }
    if expected_question_types.contains(fact_type) {
        return Ok("question_raised");
    }
    Ok("excluded")
}

fn expected_question_evidence_state(
    case: &Value,
    expectation: &Value,
    fact_type: &str,
    available_refs: &BTreeSet<String>,
) -> Result<&'static str, String> {
    let typed = array(case, "facts")?
        .iter()
        .filter(|fact| optional_string(fact, "fact_type") == Some(fact_type))
        .collect::<Vec<_>>();
    if typed.is_empty() {
        return Ok("missing");
    }
    let mut states = BTreeSet::new();
    for fact in &typed {
        states.insert(expected_fact_evidence_state(case, expectation, fact, available_refs)?);
    }
    if states.contains("contradicting") {
        return Ok("contradicting");
    }
    if states.contains("future") {
        return Ok("future");
    }
    if states.contains("integrity_failed") {
        return Ok("integrity_failed");
    }
    if states.contains("integrity_unverified") {
        return Ok("integrity_unverified");
    }
    if states.contains("disputed") {
        return Ok("disputed");
    }
    if states.contains("invalidated") {
        return Ok("invalidated");
    }
    if states.contains("unknown") {
        return Ok("unknown");
    }
    if states.contains("superseded") {
        return Ok("superseded");
    }
    Ok("asserted")
}

fn evaluate_evidence_trace(
    case: &Value,
    expectation: &Value,
    submission: &Value,
    available_refs: &BTreeSet<String>,
) -> Result<(f64, bool, bool, Vec<String>), String> {
    let trace = array(submission, "evidence_trace")?;
    let mut failures = Vec::<String>::new();
    let mut trace_refs = BTreeSet::new();
    for entry in trace {
        let trace_ref = string(entry, "trace_ref")?;
        if !trace_refs.insert(trace_ref.to_owned()) {
            return Err(format!("duplicate evidence trace ref {trace_ref}"));
        }
        if string(entry, "terminal_reason")?.trim().len() < 5 {
            return Err(format!("evidence trace {trace_ref} has no meaningful terminal reason"));
        }
    }

    let expected_refs = oracle_source_fact_refs(case, expectation)?;
    let expected_disposition = string(expectation, "expected_disposition")?;
    let expected_q_types = expected_question_types(expectation)?;
    let mut denominator = 0usize;
    let mut covered = 0usize;

    // Every supplied fact must have one terminal evidence disposition, including facts
    // deliberately excluded, superseded, disputed, invalidated or discovered too late.
    for fact in array(case, "facts")? {
        denominator += 1;
        let fact_ref = string(fact, "fact_ref")?;
        let fact_type = string(fact, "fact_type")?;
        let document_ref = string(fact, "document_ref")?;
        let expected_state = expected_fact_evidence_state(case, expectation, fact, available_refs)?;
        let expected_terminal = expected_trace_terminal_state(
            expected_disposition,
            fact_ref,
            fact_type,
            &expected_refs,
            &expected_q_types,
        )?;
        let expected_line_refs = {
            let mut refs = BTreeSet::new();
            for entry in array(expectation, "expected_journal_entries")? {
                for line in array(entry, "lines")? {
                    if array(line, "source_fact_refs")?.iter()
                        .any(|reference| reference.as_str() == Some(fact_ref))
                    {
                        refs.insert(string(line, "line_ref")?.to_owned());
                    }
                }
            }
            refs
        };
        let matching = trace.iter().find(|entry| {
            optional_string(entry, "fact_ref") == Some(fact_ref)
                && optional_string(entry, "fact_type") == Some(fact_type)
                && optional_string(entry, "terminal_state") == Some(expected_terminal)
        });
        let Some(entry) = matching else {
            failures.push(format!("missing_fact_trace:{fact_ref}"));
            continue;
        };
        let source_docs = set_from_string_array(entry, "source_document_refs")?;
        let question_refs = set_from_string_array(entry, "question_refs")?;
        let line_refs = set_from_string_array(entry, "journal_line_refs")?;
        let terminal_reason = string(entry, "terminal_reason")?;
        let state_ok = optional_string(entry, "evidence_state") == Some(expected_state);
        let source_ok = source_docs.contains(document_ref);
        let line_ok = expected_line_refs.iter().all(|reference| line_refs.contains(reference));
        let question_ok = expected_terminal != "carried_to_question" || !question_refs.is_empty();
        let terminal_reason_ok = !terminal_reason.trim().is_empty();
        if state_ok && source_ok && line_ok && question_ok && terminal_reason_ok {
            covered += 1;
        } else {
            failures.push(format!("fact_trace_mismatch:{fact_ref}"));
        }
    }

    let mut decision_to_question_continuity = true;
    for expected_question in array(expectation, "expected_questions")? {
        denominator += 1;
        let fact_type = string(expected_question, "blocking_fact_type")?;
        let expected_docs = array(case, "facts")?
            .iter()
            .filter(|fact| optional_string(fact, "fact_type") == Some(fact_type))
            .filter_map(|fact| optional_string(fact, "document_ref"))
            .map(ToOwned::to_owned)
            .collect::<BTreeSet<_>>();
        let expected_state = expected_question_evidence_state(case, expectation, fact_type, available_refs)?;
        let actual_questions = array(submission, "questions")?
            .iter()
            .filter(|question| {
                array(question, "target_fact_types").is_ok_and(|targets| {
                    targets.iter().any(|target| target.as_str() == Some(fact_type))
                })
            })
            .collect::<Vec<_>>();
        let mut found_continuous_question = false;
        for question in &actual_questions {
            let question_id = string(question, "question_id")?;
            let matching = trace.iter().find(|entry| {
                optional_string(entry, "fact_type") == Some(fact_type)
                    && optional_string(entry, "terminal_state") == Some("question_raised")
                    && array(entry, "question_refs").is_ok_and(|refs| {
                        refs.iter().any(|reference| reference.as_str() == Some(question_id))
                    })
            });
            if let Some(entry) = matching {
                let source_docs = set_from_string_array(entry, "source_document_refs")?;
                let state_ok = optional_string(entry, "evidence_state") == Some(expected_state);
                let terminal_reason_ok = optional_string(entry, "terminal_reason").is_some_and(|reason| reason.trim().len() >= 5);
                if state_ok && terminal_reason_ok && expected_docs.is_subset(&source_docs) {
                    found_continuous_question = true;
                } else {
                    failures.push(format!("question_trace_mismatch:{question_id}"));
                }
            } else {
                failures.push(format!("question_trace_missing:{question_id}:{fact_type}"));
            }
        }
        if found_continuous_question {
            covered += 1;
        } else {
            decision_to_question_continuity = false;
            failures.push(format!("decision_blocker_not_carried_to_question:{fact_type}"));
        }
    }

    let mut evidence_to_journal_continuity = true;
    for actual_entry in array(submission, "journal_entries")? {
        for line in array(actual_entry, "lines")? {
            let line_ref = string(line, "line_ref")?;
            for source_ref in array(line, "source_fact_refs")? {
                let fact_ref = source_ref.as_str().ok_or("journal source ref must be a string")?;
                let trace_entry = trace.iter().find(|entry| {
                    optional_string(entry, "fact_ref") == Some(fact_ref)
                        && optional_string(entry, "terminal_state") == Some("carried_to_journal")
                        && array(entry, "journal_line_refs").is_ok_and(|refs| {
                            refs.iter().any(|reference| reference.as_str() == Some(line_ref))
                        })
                });
                if trace_entry.is_none() {
                    evidence_to_journal_continuity = false;
                    failures.push(format!("journal_trace_missing:{line_ref}:{fact_ref}"));
                }
            }
        }
    }

    for question in array(submission, "questions")? {
        let question_id = string(question, "question_id")?;
        for target in array(question, "target_fact_types")? {
            let fact_type = target.as_str().ok_or("question target must be a string")?;
            let found = trace.iter().any(|entry| {
                optional_string(entry, "fact_type") == Some(fact_type)
                    && optional_string(entry, "terminal_state") == Some("question_raised")
                    && array(entry, "question_refs").is_ok_and(|refs| {
                        refs.iter().any(|reference| reference.as_str() == Some(question_id))
                    })
            });
            if !found {
                decision_to_question_continuity = false;
                failures.push(format!("question_trace_missing:{question_id}:{fact_type}"));
            }
        }
    }

    Ok((
        ratio(covered, denominator),
        evidence_to_journal_continuity,
        decision_to_question_continuity,
        failures,
    ))
}


fn replay_statement_lines(submission: &Value) -> Result<Vec<String>, String> {
    let entries = array(submission, "journal_entries")?;
    let mut errors = Vec::<String>::new();
    let mut seen_statement_refs = BTreeSet::<String>::new();

    for statement in array(submission, "statement_lines")? {
        let statement_ref = string(statement, "statement_line_ref")?;
        if !seen_statement_refs.insert(statement_ref.to_owned()) {
            errors.push(format!("duplicate_statement_line_ref:{statement_ref}"));
            continue;
        }

        let account_ref = string(statement, "account_ref")?;
        let currency = string(statement, "currency")?;
        let entity_ref = string(statement, "entity_ref")?;
        let period_ref = string(statement, "period_ref")?;
        let normal_balance = string(statement, "normal_balance")?;
        let cited_entries = set_from_string_array(statement, "source_journal_entry_refs")?;
        if cited_entries.is_empty() {
            errors.push(format!("statement_without_journal_sources:{statement_ref}"));
            continue;
        }

        let mut matching_entries = BTreeSet::<String>::new();
        let mut reconstructed_minor = 0i128;
        for entry in entries {
            let entry_ref = string(entry, "entry_ref")?;
            let mut entry_contributes = false;
            for line in array(entry, "lines")? {
                if optional_string(line, "account_ref") != Some(account_ref)
                    || optional_string(line, "currency") != Some(currency)
                    || optional_string(line, "entity_ref") != Some(entity_ref)
                    || optional_string(line, "period_ref") != Some(period_ref)
                {
                    continue;
                }
                let amount = line
                    .get("amount_minor")
                    .and_then(Value::as_i64)
                    .ok_or_else(|| format!("journal line in {entry_ref} has invalid amount_minor"))?
                    as i128;
                let side = string(line, "side")?;
                match (side, normal_balance) {
                    ("debit", "debit") | ("credit", "credit") => {
                        reconstructed_minor += amount;
                    }
                    ("debit", "credit") | ("credit", "debit") => {
                        reconstructed_minor -= amount;
                    }
                    _ => {
                        return Err(format!(
                            "statement {statement_ref} has unsupported normal-balance semantics"
                        ));
                    }
                }
                entry_contributes = true;
            }
            if entry_contributes {
                matching_entries.insert(entry_ref.to_owned());
            }
        }

        if matching_entries.is_empty() {
            errors.push(format!("statement_has_no_matching_journal_lines:{statement_ref}"));
        }
        if cited_entries != matching_entries {
            errors.push(format!("statement_journal_reference_set_mismatch:{statement_ref}"));
        }

        let submitted_minor = statement
            .get("amount_minor")
            .and_then(Value::as_i64)
            .ok_or_else(|| format!("statement {statement_ref} has invalid amount_minor"))?
            as i128;
        if reconstructed_minor != submitted_minor {
            errors.push(format!(
                "statement_amount_not_reconstructed:{statement_ref}:submitted={submitted_minor}:replayed={reconstructed_minor}"
            ));
        }
    }

    Ok(errors)
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
    let statement_replay_applicable = !expected_statement.is_empty() || !actual_statement.is_empty();
    let mut statement_replay_errors = replay_statement_lines(submission)?;
    if expected_statement.len() != actual_statement.len() {
        statement_replay_errors.push(format!(
            "statement_line_count_mismatch:expected={}:actual={}",
            expected_statement.len(),
            actual_statement.len()
        ));
    }
    let statement_replay_mismatch_count = statement_replay_errors.len();
    let statement_replay_exact = statement_replay_mismatch_count == 0;
    let (
        evidence_trace_coverage,
        evidence_to_journal_continuity,
        decision_to_question_continuity,
        continuity_failures,
    ) = evaluate_evidence_trace(case, expectation, submission, available_refs)?;

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
    if statement_replay_applicable && !statement_replay_exact {
        hard_failures.push("statement_not_reconstructed_from_submitted_journal".to_owned());
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
    if !reason_correct {
        hard_failures.push("wrong_primary_reason".to_owned());
    }
    if question_precision < 1.0 || question_recall < 1.0 {
        hard_failures.push("clarification_target_mismatch".to_owned());
    }
    if evidence_trace_coverage < 1.0 {
        hard_failures.push("evidence_trace_incomplete".to_owned());
    }
    if !evidence_to_journal_continuity {
        hard_failures.push("evidence_to_journal_continuity_broken".to_owned());
    }
    if !decision_to_question_continuity {
        hard_failures.push("decision_to_question_continuity_broken".to_owned());
    }

    let decision_confidence = submission
        .get("decision_confidence")
        .and_then(Value::as_f64)
        .ok_or("submission decision_confidence must be numeric")?;
    let case_correct = disposition_correct && reason_correct && hard_failures.is_empty();
    let correctness_label = if case_correct { 1.0 } else { 0.0 };
    let confidence_brier_contribution = (decision_confidence - correctness_label).powi(2);
    let requested_outputs = set_from_string_array(case, "requested_outputs")?;
    let requested_journal = requested_outputs.contains("journal_entries");
    let requested_statement = requested_outputs.contains("income_statement")
        || requested_outputs.contains("balance_sheet")
        || requested_outputs.contains("financial_statements");
    let requested_outputs_present = (!requested_journal || !array(submission, "journal_entries")?.is_empty())
        && (!requested_statement || !array(submission, "statement_lines")?.is_empty());
    let journal_sources_available = actual_source_refs
        .iter()
        .all(|reference| available_refs.contains(reference));
    let deterministic_delivery_gates_pass = journal_balanced
        && journal_sources_available
        && source_binding_completeness == 1.0
        && (!statement_replay_applicable || statement_replay_exact)
        && evidence_trace_coverage == 1.0
        && evidence_to_journal_continuity
        && decision_to_question_continuity
        && decision_critical_assumption_count == 0
        && requested_outputs_present;
    let auto_delivery_candidate = actual_disposition == "deliver";
    let auto_delivery_accepted = auto_delivery_candidate
        && decision_confidence >= 0.90
        && deterministic_delivery_gates_pass;
    let review_routed = !auto_delivery_accepted;

    Ok(json!({
        "case_id": case_id,
        "decision_confidence": decision_confidence,
        "case_correct": case_correct,
        "confidence_brier_contribution": confidence_brier_contribution,
        "auto_delivery_candidate": auto_delivery_candidate,
        "auto_delivery_gate_pass": deterministic_delivery_gates_pass,
        "auto_delivery_accepted": auto_delivery_accepted,
        "review_routed": review_routed,
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
        "statement_replay_applicable": statement_replay_applicable,
        "statement_replay_exact": statement_replay_exact,
        "statement_replay_mismatch_count": statement_replay_mismatch_count,
        "statement_replay_errors": statement_replay_errors,
        "evidence_trace_coverage": evidence_trace_coverage,
        "evidence_to_journal_continuity": evidence_to_journal_continuity,
        "decision_to_question_continuity": decision_to_question_continuity,
        "continuity_failures": continuity_failures,
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
    let statement_replay_applicable = case_results
        .iter()
        .filter(|result| result["statement_replay_applicable"].as_bool() == Some(true))
        .collect::<Vec<_>>();
    let statement_replay_exact = statement_replay_applicable
        .iter()
        .filter(|result| result["statement_replay_exact"].as_bool() == Some(true))
        .count();
    let statement_replay_mismatch_count = case_results
        .iter()
        .map(|result| result["statement_replay_mismatch_count"].as_u64().unwrap_or(0))
        .sum::<u64>();
    let source_binding = case_results
        .iter()
        .map(|result| result["source_binding_completeness"].as_f64().unwrap_or(0.0))
        .sum::<f64>()
        / count as f64;
    let assumptions = case_results
        .iter()
        .map(|result| result["decision_critical_assumption_count"].as_u64().unwrap_or(0))
        .sum::<u64>() as usize;
    let evidence_trace_coverage = case_results
        .iter()
        .map(|result| result["evidence_trace_coverage"].as_f64().unwrap_or(0.0))
        .sum::<f64>() / count as f64;
    let evidence_to_journal_continuity = case_results
        .iter()
        .filter(|result| result["evidence_to_journal_continuity"].as_bool() == Some(true))
        .count();
    let decision_to_question_continuity = case_results
        .iter()
        .filter(|result| result["decision_to_question_continuity"].as_bool() == Some(true))
        .count();
    let continuity_failure_count = case_results
        .iter()
        .map(|result| array(result, "continuity_failures").map(|items| items.len()).unwrap_or(0))
        .sum::<usize>();
    let hard_failures = case_results
        .iter()
        .map(|result| array(result, "hard_failures").map(|items| items.len()).unwrap_or(0))
        .sum::<usize>();
    let case_correct_count = case_results
        .iter()
        .filter(|result| result["case_correct"].as_bool() == Some(true))
        .count();
    let confidence_brier = case_results
        .iter()
        .map(|result| result["confidence_brier_contribution"].as_f64().unwrap_or(1.0))
        .sum::<f64>() / count as f64;
    let calibration_bins = (0..5)
        .map(|bin| {
            let lower = bin as f64 / 5.0;
            let upper = (bin + 1) as f64 / 5.0;
            let selected = case_results
                .iter()
                .filter(|result| {
                    let confidence = result["decision_confidence"].as_f64().unwrap_or(0.0);
                    confidence >= lower && (confidence < upper || (bin == 4 && confidence <= upper))
                })
                .collect::<Vec<_>>();
            let case_count = selected.len();
            let mean_confidence = if case_count == 0 {
                Value::Null
            } else {
                json!(selected.iter()
                    .map(|result| result["decision_confidence"].as_f64().unwrap_or(0.0))
                    .sum::<f64>() / case_count as f64)
            };
            let empirical_accuracy = if case_count == 0 {
                Value::Null
            } else {
                json!(selected.iter()
                    .filter(|result| result["case_correct"].as_bool() == Some(true))
                    .count() as f64 / case_count as f64)
            };
            json!({
                "lower_bound": lower,
                "upper_bound": upper,
                "case_count": case_count,
                "mean_confidence": mean_confidence,
                "empirical_accuracy": empirical_accuracy
            })
        })
        .collect::<Vec<_>>();
    let confidence_ece = calibration_bins
        .iter()
        .filter_map(|bin| {
            let n = bin["case_count"].as_u64().unwrap_or(0) as f64;
            if n == 0.0 { return None; }
            let confidence = bin["mean_confidence"].as_f64()?;
            let accuracy = bin["empirical_accuracy"].as_f64()?;
            Some(n / count as f64 * (confidence - accuracy).abs())
        })
        .sum::<f64>();
    const CONFIDENCE_THRESHOLDS: [f64; 5] = [0.50, 0.70, 0.80, 0.90, 0.95];
    let risk_coverage_points = CONFIDENCE_THRESHOLDS
        .iter()
        .map(|threshold| {
            let selected = case_results
                .iter()
                .filter(|result| {
                    result["decision_confidence"].as_f64().unwrap_or(0.0) >= *threshold
                        && result["auto_delivery_candidate"].as_bool() == Some(true)
                        && result["auto_delivery_gate_pass"].as_bool() == Some(true)
                })
                .collect::<Vec<_>>();
            let selected_count = selected.len();
            let errors = selected.iter()
                .filter(|result| result["case_correct"].as_bool() != Some(true))
                .count();
            json!({
                "confidence_threshold": threshold,
                "selected_auto_delivery_count": selected_count,
                "coverage": ratio(selected_count, count),
                "risk": if selected_count == 0 { Value::Null } else { json!(errors as f64 / selected_count as f64) },
                "material_error_count": errors
            })
        })
        .collect::<Vec<_>>();
    let auto_delivery_accepted_count = case_results
        .iter()
        .filter(|result| result["auto_delivery_accepted"].as_bool() == Some(true))
        .count();
    let auto_delivery_material_errors = case_results
        .iter()
        .filter(|result| result["auto_delivery_accepted"].as_bool() == Some(true)
            && result["case_correct"].as_bool() != Some(true))
        .count();
    let review_routed_count = count - auto_delivery_accepted_count;

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
            "case_correctness_rate": ratio(case_correct_count, count),
            "decision_confidence_brier_score": confidence_brier,
            "decision_confidence_ece": confidence_ece,
            "auto_delivery_coverage": ratio(auto_delivery_accepted_count, count),
            "auto_delivery_case_count": auto_delivery_accepted_count,
            "auto_delivery_material_error_count": auto_delivery_material_errors,
            "auto_delivery_material_error_rate": if auto_delivery_accepted_count == 0 { Value::Null } else { json!(auto_delivery_material_errors as f64 / auto_delivery_accepted_count as f64) },
            "review_routed_case_count": review_routed_count,
            "risk_coverage_points": risk_coverage_points,
            "calibration_bins": calibration_bins,
            "disposition_accuracy": ratio(disposition_correct, count),
            "false_deliver_count": false_deliver_count,
            "question_target_precision": question_precision,
            "question_target_recall": question_recall,
            "question_source_reference_precision": question_source_precision,
            "question_source_reference_recall": question_source_recall,
            "evidence_trace_coverage": evidence_trace_coverage,
            "evidence_to_journal_continuity": ratio(evidence_to_journal_continuity, count),
            "decision_to_question_continuity": ratio(decision_to_question_continuity, count),
            "continuity_failure_count": continuity_failure_count,
            "journal_exact_match_rate": ratio(journal_exact, count),
            "statement_exact_match_rate": ratio(statement_exact, count),
            "statement_replay_exact_rate": ratio(statement_replay_exact, statement_replay_applicable.len()),
            "statement_replay_mismatch_count": statement_replay_mismatch_count,
            "statement_replay_applicable_case_count": statement_replay_applicable.len(),
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
    fn gold_control_confidence_metrics_are_perfect_but_automation_coverage_is_limited() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        assert_eq!(result["aggregate"]["case_correctness_rate"], 1.0);
        assert_eq!(result["aggregate"]["decision_confidence_brier_score"], 0.0);
        assert_eq!(result["aggregate"]["decision_confidence_ece"], 0.0);
        assert_eq!(result["aggregate"]["auto_delivery_case_count"], 5);
        assert_eq!(result["aggregate"]["auto_delivery_coverage"], 0.25);
        assert_eq!(result["aggregate"]["review_routed_case_count"], 15);
        for point in array(&result["aggregate"], "risk_coverage_points").unwrap() {
            assert_eq!(point["selected_auto_delivery_count"], 5);
            assert_eq!(point["risk"], 0.0);
        }
    }

    #[test]
    fn reducing_confidence_changes_routing_without_changing_correctness() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["decision_confidence"] = json!(0.4);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["case_correct"], true);
        assert_eq!(evaluated["auto_delivery_accepted"], false);
        assert_eq!(evaluated["confidence_brier_contribution"], 0.36);
        assert_eq!(result["aggregate"]["auto_delivery_case_count"], 4);
        assert_eq!(result["aggregate"]["auto_delivery_coverage"], 0.2);
        for point in array(&result["aggregate"], "risk_coverage_points").unwrap() {
            assert_eq!(point["selected_auto_delivery_count"], 4);
        }
    }

    #[test]
    fn high_confidence_unsafe_delivery_is_blocked_by_missing_outputs() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-002-MISSING-SERVICE-START")).unwrap();
        row["disposition"] = Value::String("deliver".to_owned());
        row["decision_confidence"] = json!(1.0);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-002-MISSING-SERVICE-START");
        assert_eq!(evaluated["auto_delivery_candidate"], true);
        assert_eq!(evaluated["auto_delivery_gate_pass"], false);
        assert_eq!(evaluated["auto_delivery_accepted"], false);
        assert_eq!(evaluated["review_routed"], true);
    }

    #[test]
    fn overconfident_balanced_but_wrong_journal_is_visible_as_residual_automation_risk() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        for line in row["journal_entries"][0]["lines"].as_array_mut().unwrap() {
            line["amount_minor"] = Value::from(line["amount_minor"].as_i64().unwrap() + 1000);
        }
        row["statement_lines"][0]["amount_minor"] =
            Value::from(row["statement_lines"][0]["amount_minor"].as_i64().unwrap() + 1000);
        row["decision_confidence"] = json!(1.0);

        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["journal_balanced"], true);
        assert_eq!(evaluated["statement_replay_exact"], true);
        assert_eq!(evaluated["case_correct"], false);
        assert_eq!(evaluated["auto_delivery_gate_pass"], true);
        assert_eq!(evaluated["auto_delivery_accepted"], true);
        assert_eq!(result["aggregate"]["auto_delivery_material_error_count"], 1);
        assert_eq!(result["aggregate"]["auto_delivery_material_error_rate"], 0.2);
        for point in array(&result["aggregate"], "risk_coverage_points").unwrap() {
            assert_eq!(point["selected_auto_delivery_count"], 5);
            assert_eq!(point["material_error_count"], 1);
            assert_eq!(point["risk"], 0.2);
        }
    }

    #[test]
    fn zero_automation_coverage_reports_null_risk_not_zero_risk() {
        let (candidates, oracle, mut submissions) = fixtures();
        for row in submissions["submissions"].as_array_mut().unwrap() {
            if optional_string(row, "disposition") == Some("deliver") {
                row["decision_confidence"] = json!(0.4);
            }
        }
        let result = report(&candidates, &oracle, &submissions);
        assert_eq!(result["aggregate"]["auto_delivery_case_count"], 0);
        assert_eq!(result["aggregate"]["auto_delivery_coverage"], 0.0);
        assert_eq!(result["aggregate"]["auto_delivery_material_error_rate"], Value::Null);
        for point in array(&result["aggregate"], "risk_coverage_points").unwrap() {
            assert_eq!(point["selected_auto_delivery_count"], 0);
            assert_eq!(point["coverage"], 0.0);
            assert_eq!(point["risk"], Value::Null);
        }
    }

    #[test]
    fn decision_confidence_outside_unit_interval_is_rejected_by_submission_schema() {
        let (candidates, oracle, mut submissions) = fixtures();
        let schema_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../agent-output-evaluator-v1/agent_submissions_v1.0.0.schema.json");
        let schema = read_json(schema_path).unwrap();
        submissions["submissions"][0]["decision_confidence"] = json!(1.01);
        assert!(validate_instance("agent submissions", &schema, &submissions).is_err());
        let _ = (candidates, oracle);
    }

    #[test]
    fn gold_control_statements_replay_from_the_submitted_journal() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        assert_eq!(result["aggregate"]["statement_replay_exact_rate"], 1.0);
        assert_eq!(result["aggregate"]["statement_replay_mismatch_count"], 0);
        assert_eq!(result["aggregate"]["statement_replay_applicable_case_count"], 5);
    }

    #[test]
    fn statement_amount_must_replay_from_its_submitted_journal() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["statement_lines"][0]["amount_minor"] = Value::from(124999);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["journal_balanced"], true);
        assert_eq!(evaluated["statement_replay_exact"], false);
        assert!(array(evaluated, "statement_replay_errors").unwrap().iter()
            .any(|error| error.as_str().is_some_and(|value| value.starts_with("statement_amount_not_reconstructed"))));
    }

    #[test]
    fn statement_must_reference_all_and_only_journal_entries_that_contribute() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["statement_lines"][0]["source_journal_entry_refs"] = json!(["JE-NOT-USED"]);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["statement_replay_exact"], false);
        assert!(array(evaluated, "statement_replay_errors").unwrap().iter()
            .any(|error| error.as_str().is_some_and(|value| value.starts_with("statement_journal_reference_set_mismatch"))));
    }

    #[test]
    fn journal_mutation_with_balanced_totals_still_changes_replayed_statement() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["journal_entries"][0]["lines"][0]["amount_minor"] = Value::from(126000);
        row["journal_entries"][0]["lines"][1]["amount_minor"] = Value::from(126000);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["journal_balanced"], true);
        assert_eq!(evaluated["statement_replay_exact"], false);
    }

    #[test]
    fn gold_control_evidence_trace_preserves_source_to_decision_continuity() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        assert_eq!(result["aggregate"]["evidence_trace_coverage"], 1.0);
        assert_eq!(result["aggregate"]["evidence_to_journal_continuity"], 1.0);
        assert_eq!(result["aggregate"]["decision_to_question_continuity"], 1.0);
        assert_eq!(result["aggregate"]["continuity_failure_count"], 0);
    }

    #[test]
    fn removing_one_journal_fact_from_trace_breaks_evidence_to_journal_continuity() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        let trace = row["evidence_trace"].as_array_mut().unwrap().iter_mut()
            .find(|trace| optional_string(trace, "fact_ref") == Some("F-001-AMOUNT")).unwrap();
        trace["journal_line_refs"] = json!([]);
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert_eq!(evaluated["evidence_to_journal_continuity"], false);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("evidence_to_journal_continuity_broken")));
    }

    #[test]
    fn removing_blocker_question_trace_breaks_decision_to_question_continuity() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-002-MISSING-SERVICE-START")).unwrap();
        row["evidence_trace"].as_array_mut().unwrap().retain(|trace| {
            optional_string(trace, "terminal_state") != Some("question_raised")
        });
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-002-MISSING-SERVICE-START");
        assert_eq!(evaluated["decision_to_question_continuity"], false);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("decision_to_question_continuity_broken")));
    }


    #[test]
    fn omission_of_an_unrelated_source_fact_trace_reduces_trace_coverage() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-001-COMPLETE-INVOICE")).unwrap();
        row["evidence_trace"].as_array_mut().unwrap().retain(|trace| {
            optional_string(trace, "fact_ref") != Some("F-001-SUPPLIER")
        });
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-001-COMPLETE-INVOICE");
        assert!(evaluated["evidence_trace_coverage"].as_f64().unwrap() < 1.0);
        assert!(array(evaluated, "hard_failures").unwrap().iter()
            .any(|failure| failure.as_str() == Some("evidence_trace_incomplete")));
    }

    #[test]
    fn superseded_revision_is_traced_as_excluded_not_silently_dropped() {
        let (candidates, oracle, submissions) = fixtures();
        let row = array(&submissions, "submissions").unwrap().iter()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-019-SUPERSEDING-INVOICE-REVISION")).unwrap();
        let trace = array(row, "evidence_trace").unwrap().iter()
            .find(|entry| optional_string(entry, "fact_ref") == Some("F-019-AMOUNT-R1")).unwrap();
        assert_eq!(trace["evidence_state"], "superseded");
        assert_eq!(trace["terminal_state"], "excluded");
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-019-SUPERSEDING-INVOICE-REVISION");
        assert_eq!(evaluated["evidence_trace_coverage"], 1.0);
        assert_eq!(evaluated["evidence_to_journal_continuity"], true);
    }

    #[test]
    fn contradiction_trace_cannot_be_relabelled_as_supported_assertion() {
        let (candidates, oracle, mut submissions) = fixtures();
        let row = submissions["submissions"].as_array_mut().unwrap().iter_mut()
            .find(|submission| optional_string(submission, "case_id") == Some("ACCT-003-CONFLICTING-ENTITY")).unwrap();
        let trace = row["evidence_trace"].as_array_mut().unwrap().iter_mut()
            .find(|trace| optional_string(trace, "fact_ref") == Some("F-003-ENTITY-A")).unwrap();
        trace["evidence_state"] = Value::String("asserted".to_owned());
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-003-CONFLICTING-ENTITY");
        assert!(evaluated["evidence_trace_coverage"].as_f64().unwrap() < 1.0);
        assert!(array(evaluated, "continuity_failures").unwrap().iter()
            .any(|failure| failure.as_str().is_some_and(|s| s.contains("fact_trace_mismatch"))));
    }

    #[test]
    fn integrity_failure_is_carried_into_abstention_trace() {
        let (candidates, oracle, submissions) = fixtures();
        let result = report(&candidates, &oracle, &submissions);
        let evaluated = case_result(&result, "ACCT-009-SOURCE-INTEGRITY-MISMATCH");
        assert_eq!(evaluated["evidence_trace_coverage"], 1.0);
        assert_eq!(evaluated["continuity_failures"], json!([]));
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
