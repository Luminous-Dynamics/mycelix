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
        eprintln!("runner-observed-evidence-trace: {error}");
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

fn string_set(value: &Value, key: &str) -> Result<BTreeSet<String>, String> {
    array(value, key)?
        .iter()
        .map(|item| {
            item.as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| format!("{key} must contain strings"))
        })
        .collect()
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte))
}

fn digest(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn event_kind(event: &Value) -> Result<&str, String> {
    string(event, "event_kind")
}

fn event_actor(event: &Value) -> Result<&str, String> {
    string(event, "actor")
}

fn validate_trace(trace: &Value) -> Result<Value, String> {
    if optional_string(trace, "schema_version") != Some("1.0.0")
        || optional_string(trace, "trace_kind") != Some("runner_observed_evidence_trace")
    {
        return Err("unsupported observed trace version or kind".to_owned());
    }
    if optional_string(trace, "canonical_receipt_schema_version")
        != Some("luminous.benchmark-run-receipt.v1")
    {
        return Err("trace does not identify the canonical benchmark receipt version".to_owned());
    }

    let trace_id = string(trace, "trace_id")?;
    let events = array(trace, "events")?;
    if events.len() < 2 {
        return Err(format!("{trace_id}: trace requires at least two events"));
    }

    let mut event_by_id = BTreeMap::<String, &Value>::new();
    let mut source_opens = BTreeMap::<String, Vec<(String, String, u64)>>::new();
    let mut source_open_count = 0usize;
    let mut facts = BTreeMap::<String, (String, String, BTreeSet<String>, BTreeSet<String>, u64)>::new();
    let mut terminalized = BTreeMap::<String, (String, BTreeSet<String>, u64)>::new();
    let mut questions = BTreeMap::<String, (BTreeSet<String>, BTreeSet<String>, u64)>::new();
    let mut journal_lines = BTreeMap::<String, (BTreeSet<String>, u64)>::new();
    let mut statement_lines = BTreeMap::<String, (BTreeSet<String>, u64)>::new();
    let mut fact_observation_count = 0usize;
    let mut terminalization_count = 0usize;
    let mut human_override_count = 0usize;
    let mut disposition: Option<(String, u64)> = None;
    let mut terminal_disposition: Option<(String, u64)> = None;
    let mut previous_recorded_at: Option<String> = None;
    let mut previous_seq = 0u64;

    for event in events {
        let seq = event
            .get("seq")
            .and_then(Value::as_u64)
            .ok_or("trace event seq must be an unsigned integer")?;
        if seq != previous_seq + 1 {
            return Err(format!("{trace_id}: event sequence is not contiguous at {seq}"));
        }
        previous_seq = seq;

        let event_id = string(event, "event_id")?;
        if event_by_id.insert(event_id.to_owned(), event).is_some() {
            return Err(format!("{trace_id}: duplicate event ID {event_id}"));
        }

        let recorded_at = string(event, "recorded_at")?;
        if previous_recorded_at.is_some_and(|prior| recorded_at < prior.as_str()) {
            return Err(format!("{trace_id}/{event_id}: recorded_at regresses"));
        }
        previous_recorded_at = Some(recorded_at.to_owned());

        let kind = event_kind(event)?;
        let actor = event_actor(event)?;
        match kind {
            "source_opened" => {
                if !matches!(actor, "tool_adapter" | "runner") {
                    return Err(format!("{trace_id}/{event_id}: source_opened must be runner/tool-adapter observed"));
                }
                let document_ref = string(event, "source_document_ref")?;
                let revision = string(event, "source_revision")?;
                let source_sha = string(event, "source_sha256")?;
                if !is_sha256(source_sha) {
                    return Err(format!("{trace_id}/{event_id}: source digest is not canonical SHA-256"));
                }
                let prior = source_opens.entry(document_ref.to_owned()).or_default();
                if prior.iter().any(|(prior_revision, prior_digest, _)| {
                    prior_revision == revision && prior_digest != source_sha
                }) {
                    return Err(format!("{trace_id}/{event_id}: same source revision has different content digest"));
                }
                prior.push((revision.to_owned(), source_sha.to_owned(), seq));
                source_open_count += 1;
            }
            "tool_response" => {
                if !matches!(actor, "tool_adapter" | "runner") {
                    return Err(format!("{trace_id}/{event_id}: tool_response must be adapter/runner observed"));
                }
                if !is_sha256(string(event, "response_sha256")?) {
                    return Err(format!("{trace_id}/{event_id}: response digest is invalid"));
                }
                string(event, "tool_call_id")?;
            }
            "fact_observed" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: fact_observed actor must be candidate_agent"));
                }
                let fact_ref = string(event, "fact_ref")?;
                let fact_type = string(event, "fact_type")?;
                let state = string(event, "evidence_state")?;
                let value_digest = string(event, "fact_value_sha256")?;
                if !is_sha256(value_digest) {
                    return Err(format!("{trace_id}/{event_id}: fact value digest is invalid"));
                }
                if facts.contains_key(fact_ref) {
                    return Err(format!("{trace_id}/{event_id}: fact {fact_ref} is observed more than once; use fact_reclassified"));
                }
                let docs = string_set(event, "source_document_refs")?;
                let open_event_refs = string_set(event, "source_observation_event_refs")?;
                if docs.is_empty() || open_event_refs.is_empty() {
                    return Err(format!("{trace_id}/{event_id}: fact observation lacks source-document/open-event refs"));
                }
                let mut confirmed_docs = BTreeSet::new();
                for open_ref in &open_event_refs {
                    let open = event_by_id
                        .get(open_ref)
                        .ok_or_else(|| format!("{trace_id}/{event_id}: source observation {open_ref} is not a preceding event"))?;
                    if event_kind(open)? != "source_opened" {
                        return Err(format!("{trace_id}/{event_id}: source observation {open_ref} is not source_opened"));
                    }
                    let opened_doc = string(open, "source_document_ref")?;
                    let opened_sha = string(open, "source_sha256")?;
                    if !docs.contains(opened_doc) {
                        return Err(format!("{trace_id}/{event_id}: source-open event refers to an undeclared source document"));
                    }
                    if !source_opens.get(opened_doc).is_some_and(|opens| {
                        opens.iter().any(|(_, digest, open_seq)| digest.as_str() == opened_sha && *open_seq < seq)
                    }) {
                        return Err(format!("{trace_id}/{event_id}: source document was not opened before fact extraction"));
                    }
                    if !confirmed_docs.insert(opened_doc.to_owned()) {
                        return Err(format!("{trace_id}/{event_id}: more than one source-open revision is bound to the same document fact"));
                    }
                }
                if confirmed_docs.len() != docs.len() {
                    return Err(format!("{trace_id}/{event_id}: every fact source document must bind to exactly one preceding source-open event"));
                }
                if !docs.is_subset(&confirmed_docs) {
                    return Err(format!("{trace_id}/{event_id}: not every cited document has a prior source-open event"));
                }
                facts.insert(
                    fact_ref.to_owned(),
                    (fact_type.to_owned(), state.to_owned(), docs, open_event_refs, seq),
                );
                fact_observation_count += 1;
            }
            "fact_reclassified" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: fact_reclassified actor must be candidate_agent"));
                }
                let fact_ref = string(event, "fact_ref")?;
                let prior_state = string(event, "prior_evidence_state")?;
                let next_state = string(event, "evidence_state")?;
                let related = string_set(event, "related_event_refs")?;
                let previous = facts
                    .get_mut(fact_ref)
                    .ok_or_else(|| format!("{trace_id}/{event_id}: reclassified fact {fact_ref} was never observed"))?;
                if previous.1 != prior_state {
                    return Err(format!("{trace_id}/{event_id}: prior evidence state does not match the current state"));
                }
                if related.is_empty() {
                    return Err(format!("{trace_id}/{event_id}: reclassification must cite its cause event"));
                }
                if !related.iter().any(|reference| {
                    event_by_id.get(reference).is_some_and(|cause| {
                        cause.get("seq").and_then(Value::as_u64).is_some_and(|cause_seq| cause_seq < seq)
                    })
                }) {
                    return Err(format!("{trace_id}/{event_id}: reclassification must reference a preceding cause event"));
                }
                previous.1 = next_state.to_owned();
            }
            "contradiction_detected" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: contradiction event actor must be candidate_agent"));
                }
                let refs = string_set(event, "fact_refs")?;
                if refs.len() < 2 || refs.iter().any(|fact_ref| !facts.contains_key(fact_ref)) {
                    return Err(format!("{trace_id}/{event_id}: contradiction must connect at least two previously observed facts"));
                }
            }
            "question_emitted" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: question_emitted actor must be candidate_agent"));
                }
                let question_ref = string(event, "question_ref")?.to_owned();
                if questions.contains_key(&question_ref) {
                    return Err(format!("{trace_id}/{event_id}: duplicate question ref {question_ref}"));
                }
                if !is_sha256(string(event, "question_text_sha256")?) {
                    return Err(format!("{trace_id}/{event_id}: question text digest is invalid"));
                }
                let blockers = string_set(event, "blocker_fact_refs")?;
                let docs = string_set(event, "source_document_refs")?;
                let opens = string_set(event, "source_observation_event_refs")?;
                if blockers.is_empty() || docs.is_empty() || opens.is_empty() {
                    return Err(format!("{trace_id}/{event_id}: question lacks blocker/source lineage"));
                }
                let mut required_docs = BTreeSet::new();
                for fact_ref in &blockers {
                    let fact = facts
                        .get(fact_ref)
                        .ok_or_else(|| format!("{trace_id}/{event_id}: question blocker {fact_ref} was never observed"))?;
                    required_docs.extend(fact.2.clone());
                }
                if !required_docs.is_subset(&docs) {
                    return Err(format!("{trace_id}/{event_id}: question omits source documents for a blocker fact"));
                }
                let mut opened_question_docs = BTreeSet::new();
                for open_ref in &opens {
                    let open = event_by_id
                        .get(open_ref)
                        .ok_or_else(|| format!("{trace_id}/{event_id}: question source-open ref {open_ref} is not a preceding event"))?;
                    let opened_doc = string(open, "source_document_ref")?;
                    if event_kind(open)? != "source_opened" || !docs.contains(opened_doc) {
                        return Err(format!("{trace_id}/{event_id}: question source-open ref is inconsistent"));
                    }
                    opened_question_docs.insert(opened_doc.to_owned());
                }
                if !required_docs.is_subset(&opened_question_docs) {
                    return Err(format!("{trace_id}/{event_id}: each blocker source must have a preceding source-open observation"));
                }
                questions.insert(question_ref, (blockers, docs, seq));
            }
            "fact_terminalized" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: fact_terminalized actor must be candidate_agent"));
                }
                let fact_ref = string(event, "fact_ref")?;
                if !facts.contains_key(fact_ref) {
                    return Err(format!("{trace_id}/{event_id}: terminalized fact {fact_ref} was never observed"));
                }
                let state = string(event, "terminal_state")?.to_owned();
                let target_refs = string_set(event, "terminal_target_refs")?;
                string(event, "terminal_reason")?;
                if terminalized.insert(fact_ref.to_owned(), (state.clone(), target_refs.clone(), seq)).is_some() {
                    return Err(format!("{trace_id}/{event_id}: fact {fact_ref} is terminalized more than once"));
                }
                match state.as_str() {
                    "carried_to_question" | "carried_to_journal" | "carried_to_review" if target_refs.is_empty() => {
                        return Err(format!("{trace_id}/{event_id}: terminal state {state} requires a target"));
                    }
                    "abstained" | "excluded" => {}
                    other => return Err(format!("{trace_id}/{event_id}: unsupported terminal state {other}")),
                }
                terminalization_count += 1;
            }
            "journal_line_emitted" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: journal_line_emitted actor must be candidate_agent"));
                }
                let line_ref = string(event, "journal_line_ref")?.to_owned();
                if !is_sha256(string(event, "journal_line_sha256")?) {
                    return Err(format!("{trace_id}/{event_id}: journal-line digest is invalid"));
                }
                let fact_refs = string_set(event, "fact_refs")?;
                if fact_refs.is_empty() || fact_refs.iter().any(|fact_ref| !facts.contains_key(fact_ref)) {
                    return Err(format!("{trace_id}/{event_id}: journal line references unobserved facts"));
                }
                if journal_lines.insert(line_ref.clone(), (fact_refs, seq)).is_some() {
                    return Err(format!("{trace_id}/{event_id}: duplicate journal line ref {line_ref}"));
                }
            }
            "statement_line_emitted" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: statement_line_emitted actor must be candidate_agent"));
                }
                let line_ref = string(event, "statement_line_ref")?.to_owned();
                if !is_sha256(string(event, "statement_line_sha256")?) {
                    return Err(format!("{trace_id}/{event_id}: statement-line digest is invalid"));
                }
                let journal_refs = string_set(event, "journal_line_refs")?;
                if journal_refs.is_empty()
                    || journal_refs.iter().any(|reference| {
                        !journal_lines.get(reference).is_some_and(|(_, journal_seq)| *journal_seq < seq)
                    })
                {
                    return Err(format!("{trace_id}/{event_id}: statement line references a missing or future journal line"));
                }
                if statement_lines.insert(line_ref.clone(), (journal_refs, seq)).is_some() {
                    return Err(format!("{trace_id}/{event_id}: duplicate statement line ref {line_ref}"));
                }
            }
            "human_override" => {
                if actor != "human_reviewer" {
                    return Err(format!("{trace_id}/{event_id}: human_override actor must be human_reviewer"));
                }
                let refs = string_set(event, "fact_refs")?;
                if refs.is_empty() || refs.iter().any(|fact_ref| !facts.contains_key(fact_ref)) {
                    return Err(format!("{trace_id}/{event_id}: human override references unobserved facts"));
                }
                string(event, "reason_code")?;
                human_override_count += 1;
            }
            "disposition_emitted" => {
                if actor != "candidate_agent" {
                    return Err(format!("{trace_id}/{event_id}: disposition_emitted actor must be candidate_agent"));
                }
                if disposition.is_some() {
                    return Err(format!("{trace_id}/{event_id}: more than one final disposition"));
                }
                disposition = Some((string(event, "decision_disposition")?.to_owned(), seq));
                string(event, "reason_code")?;
            }
            "execution_terminal" => {
                if actor != "runner" {
                    return Err(format!("{trace_id}/{event_id}: execution_terminal actor must be runner"));
                }
                if terminal_disposition.is_some() {
                    return Err(format!("{trace_id}/{event_id}: more than one execution terminal"));
                }
                terminal_disposition = Some((string(event, "terminal_disposition")?.to_owned(), seq));
            }
            other => return Err(format!("{trace_id}/{event_id}: unknown event kind {other}")),
        }
    }

    let final_seq = events.len() as u64;
    let (terminal, terminal_seq) = terminal_disposition
        .as_ref()
        .ok_or_else(|| format!("{trace_id}: missing execution_terminal event"))?;
    if *terminal_seq != final_seq {
        return Err(format!("{trace_id}: execution_terminal must be the final event"));
    }

    for (fact_ref, (state, target_refs, terminal_seq)) in &terminalized {
        let fact = facts
            .get(fact_ref)
            .ok_or_else(|| format!("{trace_id}: terminalized fact {fact_ref} disappeared"))?;
        if *terminal_seq <= fact.4 {
            return Err(format!("{trace_id}: fact {fact_ref} was terminalized before observation"));
        }
        match state.as_str() {
            "carried_to_question" => {
                if !target_refs.iter().any(|target| {
                    questions.get(target).is_some_and(|(blockers, _, question_seq)| {
                        blockers.contains(fact_ref) && *question_seq < *terminal_seq
                    })
                }) {
                    return Err(format!("{trace_id}: fact {fact_ref} terminalized to a question that does not cite it"));
                }
            }
            "carried_to_journal" => {
                if !target_refs.iter().any(|target| {
                    journal_lines.get(target).is_some_and(|(refs, journal_seq)| {
                        refs.contains(fact_ref) && *journal_seq > *terminal_seq
                    })
                }) {
                    return Err(format!("{trace_id}: fact {fact_ref} terminalized to a journal line that does not cite it"));
                }
            }
            "carried_to_review" => {
                if !target_refs.iter().any(|target| {
                    event_by_id.get(target).is_some_and(|event| {
                        optional_string(event, "event_kind") == Some("human_override")
                            && event.get("seq").and_then(Value::as_u64).is_some_and(|review_seq| review_seq < *terminal_seq)
                            && string_set(event, "fact_refs").is_ok_and(|refs| refs.contains(fact_ref))
                    })
                }) {
                    return Err(format!("{trace_id}: fact {fact_ref} terminalized to review without a preceding matching review event"));
                }
            }
            "abstained" | "excluded" => {
                if !target_refs.is_empty() {
                    return Err(format!("{trace_id}: abstained/excluded fact {fact_ref} cannot point to journal/question output"));
                }
            }
            _ => unreachable!(),
        }
    }

    if terminalized.len() != facts.len() {
        let missing = facts
            .keys()
            .filter(|fact_ref| !terminalized.contains_key(*fact_ref))
            .cloned()
            .collect::<Vec<_>>();
        return Err(format!("{trace_id}: observed facts lack terminal dispositions: {}", missing.join(", ")));
    }

    for (question_ref, (blockers, _, _)) in &questions {
        for fact_ref in blockers {
            if !terminalized.get(fact_ref).is_some_and(|(state, targets, _)| {
                state == "carried_to_question" && targets.contains(question_ref)
            }) {
                return Err(format!("{trace_id}: blocker fact {fact_ref} is not terminalized to question {question_ref}"));
            }
        }
    }

    for (line_ref, (fact_refs, line_seq)) in &journal_lines {
        for fact_ref in fact_refs {
            let Some((state, targets, terminal_seq)) = terminalized.get(fact_ref) else {
                return Err(format!("{trace_id}/{line_ref}: source fact {fact_ref} is not terminalized"));
            };
            if state != "carried_to_journal" || !targets.contains(line_ref) || *terminal_seq >= *line_seq {
                return Err(format!("{trace_id}/{line_ref}: source fact {fact_ref} is not carried into this line"));
            }
        }
    }

    let (decision, decision_seq) = disposition
        .as_ref()
        .ok_or_else(|| format!("{trace_id}: missing disposition_emitted event"))?;
    if *decision_seq >= *terminal_seq {
        return Err(format!("{trace_id}: disposition must precede execution terminal"));
    }
    if terminalized.values().any(|(_, _, fact_terminal_seq)| fact_terminal_seq >= decision_seq) {
        return Err(format!("{trace_id}: final disposition precedes one or more fact terminal dispositions"));
    }
    if questions.values().any(|(_, _, question_seq)| question_seq >= decision_seq)
        || journal_lines.values().any(|(_, line_seq)| line_seq >= decision_seq)
        || statement_lines.values().any(|(_, line_seq)| line_seq >= decision_seq)
    {
        return Err(format!("{trace_id}: final disposition precedes a question, journal or statement output"));
    }
    let expected_terminal = match decision.as_str() {
        "deliver" => "completed",
        "clarify" => "needs_evidence",
        "abstain" => "abstained",
        "escalate" => "human_review",
        other => return Err(format!("{trace_id}: unknown disposition {other}")),
    };
    if terminal != expected_terminal {
        return Err(format!("{trace_id}: disposition {decision} conflicts with execution terminal {terminal}"));
    }
    match decision.as_str() {
        "clarify" if questions.is_empty() => {
            return Err(format!("{trace_id}: clarify disposition requires at least one clarification question"));
        }
        "clarify" | "abstain" | "escalate"
            if !journal_lines.is_empty() || !statement_lines.is_empty() =>
        {
            return Err(format!("{trace_id}: {decision} disposition cannot emit a journal or statement in this S0 contract"));
        }
        "escalate" if human_override_count == 0 => {
            return Err(format!("{trace_id}: escalate disposition requires a runner-observed human review event"));
        }
        _ => {}
    }

    Ok(json!({
        "trace_id": trace_id,
        "run_id": string(trace, "run_id")?,
        "task_id": string(trace, "task_id")?,
        "receipt_binding_checked": false,
        "receipt_binding_valid": null,
        "receipt_raw_sha256": null,
        "transcript_sha256": string(trace, "transcript_sha256")?,
        "source_open_count": source_open_count,
        "fact_observation_count": fact_observation_count,
        "fact_terminalization_count": terminalization_count,
        "question_count": questions.len(),
        "journal_line_count": journal_lines.len(),
        "statement_line_count": statement_lines.len(),
        "disposition": decision,
        "terminal_disposition": terminal,
        "unaccounted_fact_count": 0,
        "findings": if human_override_count > 0 { vec!["human_override_present"] } else { Vec::<&str>::new() }
    }))
}

fn validate_receipt_binding(
    trace: &Value,
    trace_bytes: &[u8],
    receipt: &Value,
    receipt_bytes: &[u8],
    validation_report: &Value,
    transcript_bytes: &[u8],
) -> Result<Value, String> {
    if optional_string(validation_report, "schema")
        != Some("luminous.benchmark-receipt-validation-result.v1")
        || validation_report.get("valid").and_then(Value::as_bool) != Some(true)
    {
        return Err("canonical benchmark receipt validation report is missing or invalid".to_owned());
    }
    if optional_string(receipt, "schema_version") != Some("luminous.benchmark-run-receipt.v1") {
        return Err("unsupported canonical benchmark receipt schema version".to_owned());
    }
    let receipt_run_id = string(receipt, "run_id")?;
    if receipt_run_id != string(trace, "run_id")? {
        return Err("runner-observed trace run_id differs from canonical receipt".to_owned());
    }
    let receipt_sha = digest(receipt_bytes);
    let validation_rows = array(validation_report, "receipts")?;
    let matching_report_rows = validation_rows
        .iter()
        .filter(|row| optional_string(row, "run_id") == Some(receipt_run_id))
        .collect::<Vec<_>>();
    if matching_report_rows.len() != 1 {
        return Err("canonical receipt validation report must contain exactly one matching run row".to_owned());
    }
    let report_row = matching_report_rows[0];
    if optional_string(report_row, "raw_file_sha256") != Some(receipt_sha.as_str()) {
        return Err("canonical receipt validation report does not bind exact receipt bytes".to_owned());
    }

    let transcript_digest = digest(transcript_bytes);
    let receipt_transcript = receipt
        .get("provenance")
        .and_then(|provenance| provenance.get("transcript_sha256"))
        .and_then(Value::as_str)
        .ok_or("canonical receipt is missing provenance.transcript_sha256")?;
    if receipt_transcript != transcript_digest
        || receipt_transcript != string(trace, "transcript_sha256")?
    {
        return Err("raw transcript bytes do not match the trace and canonical receipt digest".to_owned());
    }

    let task_id = string(trace, "task_id")?;
    let matching_tasks = array(receipt, "task_results")?
        .iter()
        .filter(|row| optional_string(row, "task_id") == Some(task_id))
        .collect::<Vec<_>>();
    if matching_tasks.len() != 1 {
        return Err(format!("canonical receipt must contain exactly one task result for {task_id}"));
    }
    let task = matching_tasks[0];
    let input_digest = string(task, "input_sha256")?;
    if input_digest != string(trace, "source_bundle_sha256")? {
        return Err("trace source bundle digest differs from canonical task input digest".to_owned());
    }
    let trace_digest = digest(trace_bytes);
    let output_digests = string_set(task, "output_sha256")?;
    if !output_digests.contains(&trace_digest) {
        return Err("runner-observed trace artifact digest is absent from canonical task output_sha256".to_owned());
    }

    Ok(json!({
        "receipt_binding_checked": true,
        "receipt_binding_valid": true,
        "receipt_raw_sha256": receipt_sha,
        "transcript_sha256": transcript_digest,
        "run_id": receipt_run_id,
        "task_id": task_id,
        "trace_sha256": trace_digest,
        "task_input_sha256": input_digest
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../agent-output-evaluator-v1");
    let mut schema_path = root.join("runner_observed_evidence_trace_v1.0.0.schema.json");
    let mut trace_path = root.join("fixtures/runner_observed_evidence_trace_synthetic_v1.0.0.json");
    let mut report_schema_path = root.join("runner_observed_evidence_trace_report_v1.0.0.schema.json");
    let mut receipt_path: Option<PathBuf> = None;
    let mut transcript_path: Option<PathBuf> = None;
    let mut receipt_validation_path: Option<PathBuf> = None;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--schema" => schema_path = PathBuf::from(args.next().ok_or("--schema requires a path")?),
            "--trace" => trace_path = PathBuf::from(args.next().ok_or("--trace requires a path")?),
            "--report-schema" => report_schema_path = PathBuf::from(args.next().ok_or("--report-schema requires a path")?),
            "--run-receipt" => receipt_path = Some(PathBuf::from(args.next().ok_or("--run-receipt requires a path")?)),
            "--transcript" => transcript_path = Some(PathBuf::from(args.next().ok_or("--transcript requires a path")?)),
            "--receipt-validation-report" => receipt_validation_path = Some(PathBuf::from(args.next().ok_or("--receipt-validation-report requires a path")?)),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--help" | "-h" => {
                println!(
                    "Validates a runner-observed evidence trace; optionally binds it to the canonical Symthaea run receipt.\n\n\
                     --trace PATH                     Trace artifact (default: bundled synthetic control)\n\
                     --schema PATH                    Draft 2020-12 trace schema\n\
                     --report-schema PATH             Validation report schema\n\
                     --run-receipt PATH               Canonical benchmark-run-receipt-v1 JSON\n\
                     --transcript PATH                Exact raw transcript bytes\n\
                     --receipt-validation-report PATH Successful canonical receipt validator report\n\
                     --output PATH                    Write report JSON"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}; use --help").into()),
        }
    }

    let receipt_args = [receipt_path.is_some(), transcript_path.is_some(), receipt_validation_path.is_some()];
    if receipt_args.iter().any(|present| *present) && !receipt_args.iter().all(|present| *present) {
        return Err("receipt-bound mode requires --run-receipt, --transcript and --receipt-validation-report together".into());
    }

    let schema: Value = read_json(schema_path)?;
    let report_schema: Value = read_json(report_schema_path)?;
    let trace_bytes = read_bytes(trace_path)?;
    let trace: Value = serde_json::from_slice(&trace_bytes)?;
    validate_instance("runner-observed trace", &schema, &trace)?;
    let mut report = validate_trace(&trace)?;
    report["report_id"] = Value::String("MYCELIX-RUNNER-OBSERVED-TRACE-REPORT".to_owned());
    report["schema_version"] = Value::String("1.0.0".to_owned());
    report["claim_ceiling"] = Value::String(
        "trace_structure_and_canonical_receipt_binding_not_model_identity_or_accounting_correctness".to_owned()
    );
    report["trace_sha256"] = Value::String(digest(&trace_bytes));

    if let (Some(receipt_path), Some(transcript_path), Some(validation_path)) =
        (receipt_path, transcript_path, receipt_validation_path)
    {
        let receipt_bytes = read_bytes(receipt_path)?;
        let transcript_bytes = read_bytes(transcript_path)?;
        let receipt_validation: Value = read_json(validation_path)?;
        let receipt: Value = serde_json::from_slice(&receipt_bytes)?;
        let binding = validate_receipt_binding(
            &trace,
            &trace_bytes,
            &receipt,
            &receipt_bytes,
            &receipt_validation,
            &transcript_bytes,
        )?;
        report["receipt_binding_checked"] = binding["receipt_binding_checked"].clone();
        report["receipt_binding_valid"] = binding["receipt_binding_valid"].clone();
        report["receipt_raw_sha256"] = binding["receipt_raw_sha256"].clone();
        report["transcript_sha256"] = binding["transcript_sha256"].clone();
    }

    validate_instance("runner-observed trace report", &report_schema, &report)?;
    let bytes = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &bytes)?;
            println!("Wrote runner-observed trace validation report for {}.", report["trace_id"]);
        }
        None => println!("{}", String::from_utf8(bytes)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> (Value, Value, Value, Vec<u8>) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../agent-output-evaluator-v1");
        let trace_path = root.join("fixtures/runner_observed_evidence_trace_synthetic_v1.0.0.json");
        let trace_bytes = read_bytes(trace_path).unwrap();
        let trace: Value = serde_json::from_slice(&trace_bytes).unwrap();
        let schema = read_json(root.join("runner_observed_evidence_trace_v1.0.0.schema.json")).unwrap();
        let report_schema = read_json(root.join("runner_observed_evidence_trace_report_v1.0.0.schema.json")).unwrap();
        (trace, schema, report_schema, trace_bytes)
    }

    #[test]
    fn positive_runner_observed_trace_passes_and_accounts_for_each_fact() {
        let (trace, schema, report_schema, bytes) = fixture();
        validate_instance("trace schema", &schema, &trace).unwrap();
        let mut report = validate_trace(&trace).unwrap();
        report["report_id"] = json!("TEST");
        report["schema_version"] = json!("1.0.0");
        report["claim_ceiling"] = json!("trace_structure_and_canonical_receipt_binding_not_model_identity_or_accounting_correctness");
        report["trace_sha256"] = json!(digest(&bytes));
        validate_instance("trace report", &report_schema, &report).unwrap();
        assert_eq!(report["source_open_count"], 2);
        assert_eq!(report["fact_observation_count"], 5);
        assert_eq!(report["fact_terminalization_count"], 5);
        assert_eq!(report["journal_line_count"], 2);
        assert_eq!(report["statement_line_count"], 1);
        assert_eq!(report["unaccounted_fact_count"], 0);
    }

    #[test]
    fn rejects_fact_observation_before_source_open() {
        let (mut trace, _, _, _) = fixture();
        let events = trace["events"].as_array_mut().unwrap();
        events.swap(0, 2);
        for (index, event) in events.iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
            event["recorded_at"] = json!(format!("2026-11-06T06:04:{:02}Z", index + 1));
        }
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_fact_without_terminal_disposition() {
        let (mut trace, _, _, _) = fixture();
        trace["events"].as_array_mut().unwrap().retain(|event| {
            optional_string(event, "fact_ref") != Some("F-001-SUPPLIER")
                || optional_string(event, "event_kind") != Some("fact_terminalized")
        });
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_source_digest_conflict_for_same_revision() {
        let (mut trace, _, _, _) = fixture();
        let first = trace["events"][0].clone();
        let mut second = first.clone();
        second["seq"] = json!(2);
        second["event_id"] = json!("EVT-DUPLICATE-OPEN");
        second["source_sha256"] = json!("d".repeat(64));
        trace["events"].as_array_mut().unwrap().insert(1, second);
        for (index, event) in trace["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
            event["recorded_at"] = json!(format!("2026-11-06T06:04:{:02}Z", index + 1));
        }
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_question_without_observed_blocker() {
        let (mut trace, _, _, _) = fixture();
        let mut event = trace["events"][2].clone();
        event["seq"] = json!(18);
        event["event_id"] = json!("EVT-QUESTION-INVALID");
        event["event_kind"] = json!("question_emitted");
        event["question_ref"] = json!("Q-UNKNOWN");
        event["question_text_sha256"] = json!("d".repeat(64));
        event["blocker_fact_refs"] = json!(["F-UNKNOWN"]);
        event["source_document_refs"] = json!(["DOC-001-INV"]);
        event["source_observation_event_refs"] = json!(["EVT-001-SOURCE-OPEN"]);
        event["fact_ref"] = Value::Null;
        event["fact_type"] = Value::Null;
        event["evidence_state"] = Value::Null;
        event["fact_value_sha256"] = Value::Null;
        event["terminal_state"] = Value::Null;
        event["terminal_target_refs"] = json!([]);
        event["terminal_reason"] = Value::Null;
        event["journal_line_ref"] = Value::Null;
        event["journal_line_sha256"] = Value::Null;
        event["statement_line_ref"] = Value::Null;
        event["statement_line_sha256"] = Value::Null;
        event["journal_line_refs"] = json!([]);
        event["decision_disposition"] = Value::Null;
        event["reason_code"] = Value::Null;
        event["terminal_disposition"] = Value::Null;
        event["response_sha256"] = Value::Null;
        trace["events"].as_array_mut().unwrap().insert(16, event);
        for (index, event) in trace["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
        }
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_journal_line_that_uses_an_unobserved_fact() {
        let (mut trace, _, _, _) = fixture();
        let line = trace["events"].as_array_mut().unwrap().iter_mut()
            .find(|event| optional_string(event, "event_kind") == Some("journal_line_emitted")).unwrap();
        line["fact_refs"] = json!(["F-NOT-OBSERVED"]);
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_statement_line_with_missing_journal_reference() {
        let (mut trace, _, _, _) = fixture();
        let line = trace["events"].as_array_mut().unwrap().iter_mut()
            .find(|event| optional_string(event, "event_kind") == Some("statement_line_emitted")).unwrap();
        line["journal_line_refs"] = json!(["JE-MISSING"]);
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_timestamp_regression() {
        let (mut trace, _, _, _) = fixture();
        trace["events"][4]["recorded_at"] = json!("2026-11-06T05:00:00Z");
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_reclassification_with_wrong_prior_state() {
        let (mut trace, _, _, _) = fixture();
        let mut event = trace["events"][2].clone();
        event["seq"] = json!(18);
        event["event_id"] = json!("EVT-RECLASSIFY-INVALID");
        event["event_kind"] = json!("fact_reclassified");
        event["fact_ref"] = json!("F-001-AMOUNT");
        event["prior_evidence_state"] = json!("disputed");
        event["evidence_state"] = json!("asserted");
        event["related_event_refs"] = json!(["EVT-001-SOURCE-OPEN"]);
        event["fact_type"] = Value::Null;
        event["fact_value_sha256"] = Value::Null;
        event["source_document_ref"] = Value::Null;
        event["source_revision"] = Value::Null;
        event["source_sha256"] = Value::Null;
        event["source_locator"] = Value::Null;
        event["source_document_refs"] = json!([]);
        event["source_observation_event_refs"] = json!([]);
        event["terminal_state"] = Value::Null;
        event["terminal_target_refs"] = json!([]);
        event["terminal_reason"] = Value::Null;
        event["question_ref"] = Value::Null;
        event["question_text_sha256"] = Value::Null;
        event["blocker_fact_refs"] = json!([]);
        event["journal_line_ref"] = Value::Null;
        event["journal_line_sha256"] = Value::Null;
        event["statement_line_ref"] = Value::Null;
        event["statement_line_sha256"] = Value::Null;
        event["journal_line_refs"] = json!([]);
        event["decision_disposition"] = Value::Null;
        event["reason_code"] = Value::Null;
        event["terminal_disposition"] = Value::Null;
        event["response_sha256"] = Value::Null;
        trace["events"].as_array_mut().unwrap().insert(16, event);
        for (index, event) in trace["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
        }
        assert!(validate_trace(&trace).is_err());
    }

    fn synthetic_receipt(trace: &Value, trace_bytes: &[u8], transcript: &[u8]) -> (Value, Value) {
        let trace_sha = digest(trace_bytes);
        let receipt = json!({
            "schema_version":"luminous.benchmark-run-receipt.v1",
            "run_id":string(trace,"run_id").unwrap(),
            "provenance":{"transcript_sha256":digest(transcript)},
            "task_results":[{
                "task_id":string(trace,"task_id").unwrap(),
                "input_sha256":string(trace,"source_bundle_sha256").unwrap(),
                "output_sha256":[trace_sha]
            }]
        });
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let validation_report = json!({
            "schema":"luminous.benchmark-receipt-validation-result.v1",
            "valid":true,
            "receipts":[{
                "run_id":string(trace,"run_id").unwrap(),
                "raw_file_sha256":digest(&receipt_bytes)
            }]
        });
        (receipt, validation_report)
    }


    #[test]
    fn rejects_fact_that_binds_multiple_revisions_of_same_document() {
        let (mut trace, _, _, _) = fixture();
        let duplicate_open = trace["events"][0].clone();
        let mut duplicate = duplicate_open;
        duplicate["seq"] = json!(18);
        duplicate["event_id"] = json!("EVT-REVISION-OPEN");
        duplicate["source_revision"] = json!("r2");
        duplicate["source_sha256"] = json!("d".repeat(64));
        trace["events"].as_array_mut().unwrap().insert(2, duplicate);
        let fact = trace["events"].as_array_mut().unwrap().iter_mut()
            .find(|event| optional_string(event, "event_kind") == Some("fact_observed")
                && optional_string(event, "fact_ref") == Some("F-001-AMOUNT")).unwrap();
        fact["source_observation_event_refs"] = json!(["EVT-001-SOURCE-OPEN","EVT-REVISION-OPEN"]);
        for (index, event) in trace["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
        }
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_question_blocker_without_fact_terminalized_to_that_question() {
        let (mut trace, _, _, _) = fixture();
        let mut fact = trace["events"][2].clone();
        fact["seq"] = json!(18);
        fact["event_id"] = json!("EVT-QUESTION-FACT");
        fact["event_kind"] = json!("question_emitted");
        fact["question_ref"] = json!("Q-001");
        fact["question_text_sha256"] = json!("d".repeat(64));
        fact["blocker_fact_refs"] = json!(["F-001-AMOUNT"]);
        fact["source_document_refs"] = json!(["DOC-001-INV"]);
        fact["source_observation_event_refs"] = json!(["EVT-001-SOURCE-OPEN"]);
        fact["fact_ref"] = Value::Null;
        fact["fact_type"] = Value::Null;
        fact["evidence_state"] = Value::Null;
        fact["fact_value_sha256"] = Value::Null;
        fact["terminal_state"] = Value::Null;
        fact["terminal_target_refs"] = json!([]);
        fact["terminal_reason"] = Value::Null;
        fact["journal_line_ref"] = Value::Null;
        fact["journal_line_sha256"] = Value::Null;
        fact["statement_line_ref"] = Value::Null;
        fact["statement_line_sha256"] = Value::Null;
        fact["journal_line_refs"] = json!([]);
        fact["decision_disposition"] = Value::Null;
        fact["reason_code"] = Value::Null;
        fact["terminal_disposition"] = Value::Null;
        fact["response_sha256"] = Value::Null;
        trace["events"].as_array_mut().unwrap().insert(16, fact);
        for (index, event) in trace["events"].as_array_mut().unwrap().iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
        }
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn rejects_final_disposition_before_the_last_fact_terminalization() {
        let (mut trace, _, _, _) = fixture();
        let events = trace["events"].as_array_mut().unwrap();
        let disposition = events.iter().position(|event| optional_string(event, "event_kind") == Some("disposition_emitted")).unwrap();
        let terminalization = events.iter().position(|event| optional_string(event, "event_kind") == Some("fact_terminalized")).unwrap();
        events.swap(disposition, terminalization);
        for (index, event) in events.iter_mut().enumerate() {
            event["seq"] = json!(index + 1);
        }
        assert!(validate_trace(&trace).is_err());
    }


    #[test]
    fn rejects_receipt_validation_report_with_wrong_schema_id() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (receipt, mut report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        report["schema"] = json!("unknown.receipt-validator.v9");
        assert!(validate_receipt_binding(
            &trace, &trace_bytes, &receipt, &receipt_bytes, &report, transcript
        ).is_err());
    }

    #[test]
    fn rejects_duplicate_matching_receipt_rows_in_validation_report() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (receipt, mut report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let row = report["receipts"][0].clone();
        report["receipts"].as_array_mut().unwrap().push(row);
        assert!(validate_receipt_binding(
            &trace, &trace_bytes, &receipt, &receipt_bytes, &report, transcript
        ).is_err());
    }

    #[test]
    fn rejects_duplicate_matching_task_rows_in_receipt() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (mut receipt, report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let task = receipt["task_results"][0].clone();
        receipt["task_results"].as_array_mut().unwrap().push(task);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let report = json!({"schema":"luminous.benchmark-receipt-validation-result.v1","valid":true,"receipts":[{
            "run_id":string(&receipt,"run_id").unwrap(),
            "raw_file_sha256":digest(&receipt_bytes)
        }]});
        assert!(validate_receipt_binding(
            &trace,&trace_bytes,&receipt,&receipt_bytes,&report,transcript
        ).is_err());
    }

    #[test]
    fn rejects_question_disposition_without_question_event() {
        let (mut trace, _, _, _) = fixture();
        let disposition = trace["events"].as_array_mut().unwrap().iter_mut()
            .find(|event| optional_string(event,"event_kind")==Some("disposition_emitted")).unwrap();
        disposition["decision_disposition"] = json!("clarify");
        disposition["reason_code"] = json!("missing_fact");
        let terminal = trace["events"].as_array_mut().unwrap().last_mut().unwrap();
        terminal["terminal_disposition"] = json!("needs_evidence");
        assert!(validate_trace(&trace).is_err());
    }

    #[test]
    fn binds_trace_to_canonical_receipt_task_output_and_transcript_digest() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"{\"events\":[],\"fixture\":\"synthetic\"}";
        let (receipt, report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let binding = validate_receipt_binding(
            &trace,
            &trace_bytes,
            &receipt,
            &receipt_bytes,
            &report,
            transcript,
        ).unwrap();
        assert_eq!(binding["receipt_binding_valid"], true);
    }

    #[test]
    fn rejects_transcript_bytes_that_do_not_match_receipt_digest() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (receipt, report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        assert!(validate_receipt_binding(
            &trace,&trace_bytes,&receipt,&receipt_bytes,&report,b"different transcript"
        ).is_err());
    }

    #[test]
    fn rejects_trace_digest_missing_from_task_output_artifacts() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (mut receipt, _initial_report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        receipt["task_results"][0]["output_sha256"] = json!([]);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let (receipt, report) = {
            let report = json!({"valid":true,"receipts":[{
                "run_id":string(&receipt,"run_id").unwrap(),
                "raw_file_sha256":digest(&receipt_bytes)
            }]});
            (receipt, report)
        };
        assert!(validate_receipt_binding(
            &trace,&trace_bytes,&receipt,&receipt_bytes,&report,transcript
        ).is_err());
    }

    #[test]
    fn rejects_canonical_receipt_report_for_different_receipt_bytes() {
        let (trace, _, _, trace_bytes) = fixture();
        let transcript = b"actual transcript";
        let (receipt, mut report) = synthetic_receipt(&trace, &trace_bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        report["receipts"][0]["raw_file_sha256"] = json!("d".repeat(64));
        assert!(validate_receipt_binding(
            &trace,&trace_bytes,&receipt,&receipt_bytes,&report,transcript
        ).is_err());
    }

    #[test]
    fn rejects_trace_source_bundle_digest_not_matching_task_input() {
        let (mut trace, _, _, trace_bytes) = fixture();
        trace["source_bundle_sha256"] = json!("d".repeat(64));
        let bytes = serde_json::to_vec(&trace).unwrap();
        let transcript = b"actual transcript";
        let (receipt, report) = synthetic_receipt(&trace, &bytes, transcript);
        let receipt_bytes = serde_json::to_vec(&receipt).unwrap();
        let mut bad_receipt = receipt.clone();
        bad_receipt["task_results"][0]["input_sha256"] = json!("e".repeat(64));
        let bad_bytes = serde_json::to_vec(&bad_receipt).unwrap();
        let report = json!({"schema":"luminous.benchmark-receipt-validation-result.v1","valid":true,"receipts":[{
            "run_id":string(&bad_receipt,"run_id").unwrap(),
            "raw_file_sha256":digest(&bad_bytes)
        }]});
        assert!(validate_receipt_binding(
            &trace,&bytes,&bad_receipt,&bad_bytes,&report,transcript
        ).is_err());
        let _ = report;
    }
}
