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
        eprintln!("accounting-policy-projection-audit: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: PathBuf) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
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

fn optional_i64(value: &Value, key: &str) -> Option<i64> {
    value.get(key).and_then(Value::as_i64)
}

fn optional_is_null(value: &Value, key: &str) -> bool {
    value.get(key).is_none_or(Value::is_null)
}

fn find_by_ref<'a>(items: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    items
        .iter()
        .find(|item| optional_string(item, field) == Some(reference))
}

fn graph_case_by_id<'a>(graph: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    find_by_ref(array(graph, "cases")?, "case_id", case_id)
        .ok_or_else(|| format!("unknown graph case {case_id}"))
}

fn event_by_ref<'a>(graph_case: &'a Value, event_ref: &str) -> Result<&'a Value, String> {
    find_by_ref(array(graph_case, "events")?, "event_ref", event_ref)
        .ok_or_else(|| format!("unknown graph event {event_ref}"))
}

fn assertion_by_ref<'a>(graph_case: &'a Value, assertion_ref: &str) -> Result<&'a Value, String> {
    find_by_ref(array(graph_case, "assertions")?, "assertion_ref", assertion_ref)
        .ok_or_else(|| format!("unknown graph assertion {assertion_ref}"))
}

fn source_by_ref<'a>(graph_case: &'a Value, source_ref: &str) -> Result<&'a Value, String> {
    find_by_ref(array(graph_case, "source_objects")?, "source_ref", source_ref)
        .ok_or_else(|| format!("unknown graph source {source_ref}"))
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

fn assertion_available(graph_case: &Value, assertion: &Value) -> Result<bool, String> {
    let frontier = string(graph_case, "knowledge_at")?;
    let source = source_by_ref(graph_case, string(assertion, "source_ref")?)?;
    let period = graph_case.get("period").ok_or("graph case missing period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let valid_from = string(assertion, "valid_from")?;
    let valid_to = assertion.get("valid_to").and_then(Value::as_str);
    let overlaps = valid_from <= period_end
        && valid_to.is_none_or(|end| end >= period_start);
    let superseded = array(graph_case, "source_objects")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_source_ref")
            == optional_string(source, "source_ref")
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    });
    Ok(string(assertion, "recorded_at")? <= frontier
        && string(source, "received_at")? <= frontier
        && overlaps
        && optional_string(source, "integrity_state") == Some("synthetic_match")
        && !superseded
        && optional_string(assertion, "epistemic_state") == Some("asserted")
        && !optional_is_null(assertion, "value"))
}

fn projection_decision_by_ref<'a>(
    projection: &'a Value,
    decision_ref: &str,
) -> Option<&'a Value> {
    array(projection, "event_decisions")
        .ok()?
        .iter()
        .find(|decision| optional_string(decision, "policy_decision_ref") == Some(decision_ref))
}

fn failure(failures: &mut BTreeSet<&'static str>, code: &'static str) {
    failures.insert(code);
}

fn valid_digest(value: Option<&str>) -> bool {
    value.is_some_and(|value| {
        value.len() == 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn evaluate_projection(
    projection: &Value,
    graph_case: &Value,
    actual_graph_sha256: &str,
) -> Result<Value, String> {
    let projection_id = string(projection, "projection_id")?;
    let mut failures = BTreeSet::<&'static str>::new();

    let graph_binding = projection.get("graph_binding").ok_or("projection missing graph_binding")?;
    let graph_binding_valid = optional_string(graph_binding, "graph_sha256")
        == Some(actual_graph_sha256)
        && optional_string(graph_binding, "graph_case_id") == optional_string(graph_case, "case_id");
    if !graph_binding_valid {
        failure(&mut failures, "graph_digest_mismatch");
    }

    let policy_binding = projection.get("policy_binding").ok_or("projection missing policy_binding")?;
    let policy_binding_valid = valid_digest(optional_string(policy_binding, "policy_sha256"));
    if !policy_binding_valid {
        failure(&mut failures, "policy_digest_invalid");
    }

    let projector_binding = projection.get("projector_binding").ok_or("projection missing projector_binding")?;
    let projector_binding_valid = valid_digest(optional_string(projector_binding, "projector_sha256"));
    if !projector_binding_valid {
        failure(&mut failures, "projector_digest_invalid");
    }

    let graph_events = array(graph_case, "events")?;
    let mut event_ids = BTreeSet::new();
    for event in graph_events {
        event_ids.insert(string(event, "event_ref")?.to_owned());
    }
    let decisions = array(projection, "event_decisions")?;
    let mut decisions_by_event = BTreeMap::<String, &Value>::new();
    let mut decision_refs = BTreeSet::<String>::new();
    let mut event_decision_coverage = true;
    let mut assertion_provenance_valid = true;

    for decision in decisions {
        let decision_ref = string(decision, "policy_decision_ref")?;
        if !decision_refs.insert(decision_ref.to_owned()) {
            failure(&mut failures, "duplicate_policy_decision_ref");
        }
        let event_ref = string(decision, "event_ref")?;
        if !event_ids.contains(event_ref) {
            failure(&mut failures, "orphan_event_decision");
            event_decision_coverage = false;
            continue;
        }
        if decisions_by_event.insert(event_ref.to_owned(), decision).is_some() {
            failure(&mut failures, "duplicate_event_decision");
            event_decision_coverage = false;
        }
        let event = event_by_ref(graph_case, event_ref)?;
        let event_assertions = array(event, "assertion_refs")?
            .iter()
            .map(|reference| {
                reference
                    .as_str()
                    .map(ToOwned::to_owned)
                    .ok_or_else(|| "event assertion ref must be a string".to_owned())
            })
            .collect::<Result<BTreeSet<_>, _>>()?;
        let decision_assertions = string_set(decision, "source_assertion_refs")?;
        if decision_assertions.is_empty() || !decision_assertions.is_subset(&event_assertions) {
            failure(&mut failures, "decision_assertion_not_attached_to_event");
            assertion_provenance_valid = false;
        }
        for assertion_ref in &decision_assertions {
            let assertion = assertion_by_ref(graph_case, assertion_ref)?;
            if !assertion_available(graph_case, assertion)? {
                failure(&mut failures, "decision_assertion_unavailable");
                assertion_provenance_valid = false;
            }
        }
        if optional_string(decision, "decision_state") == Some("recognized")
            && (optional_is_null(decision, "recognized_amount_minor")
                || optional_is_null(decision, "currency")
                || optional_is_null(decision, "period_ref"))
        {
            failure(&mut failures, "recognized_decision_missing_amount_or_period");
        }
    }

    for event_ref in &event_ids {
        if !decisions_by_event.contains_key(event_ref) {
            failure(&mut failures, "missing_event_decision");
            event_decision_coverage = false;
        }
    }
    if decisions_by_event.len() != event_ids.len() {
        event_decision_coverage = false;
    }

    let mut journal_lineage_valid = true;
    let mut journal_balanced = true;
    let mut journaled_decisions = BTreeSet::<String>::new();
    let mut journal_line_refs = BTreeSet::<String>::new();
    let mut decision_posted_assertions = BTreeMap::<String, BTreeSet<String>>::new();
    let mut decision_postings = BTreeMap::<String, BTreeMap<String, (i128, i128)>>::new();
    let projection_state = string(projection, "projection_state")?;
    let journal_entries = array(projection, "journal_entries")?;
    let mut total_line_count = 0usize;

    if projection_state != "complete_for_review" && !journal_entries.is_empty() {
        failure(&mut failures, "blocked_projection_has_journal");
    }

    let entity_ref = string(projection, "entity_ref")?;
    let period = projection.get("period").ok_or("projection missing period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;

    for entry in journal_entries {
        let mut balances = BTreeMap::<(String, String, String), (i128, i128)>::new();
        for line in array(entry, "lines")? {
            total_line_count += 1;
            let line_ref = string(line, "line_ref")?;
            if !journal_line_refs.insert(line_ref.to_owned()) {
                failure(&mut failures, "duplicate_journal_line_ref");
                journal_lineage_valid = false;
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            let Some(decision) = projection_decision_by_ref(projection, decision_ref) else {
                failure(&mut failures, "journal_policy_decision_missing");
                journal_lineage_valid = false;
                continue;
            };
            let decision_event_ref = string(decision, "event_ref")?;
            let line_event_refs = string_set(line, "event_refs")?;
            if line_event_refs.len() != 1 || !line_event_refs.contains(decision_event_ref) {
                failure(&mut failures, "journal_event_lineage_mismatch");
                journal_lineage_valid = false;
            }
            if optional_string(decision, "decision_state") != Some("recognized") {
                failure(&mut failures, "nonrecognized_decision_has_journal");
                journal_lineage_valid = false;
            }
            journaled_decisions.insert(decision_ref.to_owned());

            let line_assertions = string_set(line, "source_assertion_refs")?;
            let decision_assertions = string_set(decision, "source_assertion_refs")?;
            if line_assertions.is_empty() || !line_assertions.is_subset(&decision_assertions) {
                failure(&mut failures, "journal_assertion_lineage_mismatch");
                journal_lineage_valid = false;
            }
            decision_posted_assertions
                .entry(decision_ref.to_owned())
                .or_default()
                .extend(line_assertions.iter().cloned());
            for assertion_ref in &line_assertions {
                let assertion = assertion_by_ref(graph_case, assertion_ref)?;
                let event = event_by_ref(graph_case, decision_event_ref)?;
                let event_assertions = string_set(event, "assertion_refs")?;
                if !event_assertions.contains(assertion_ref) {
                    failure(&mut failures, "journal_assertion_lineage_mismatch");
                    journal_lineage_valid = false;
                }
                if !assertion_available(graph_case, assertion)? {
                    failure(&mut failures, "journal_source_assertion_unavailable");
                    journal_lineage_valid = false;
                }
            }

            let line_entity = string(line, "entity_ref")?;
            let line_period = string(line, "period_ref")?;
            let line_period_start = format!("{line_period}-01");
            if line_entity != entity_ref || line_period_start.as_str() < period_start || line_period_start.as_str() > period_end {
                failure(&mut failures, "journal_entity_period_mismatch");
                journal_lineage_valid = false;
            }
            let currency = string(line, "currency")?.to_owned();
            let decision_currency = optional_string(decision, "currency").unwrap_or("");
            let decision_period = optional_string(decision, "period_ref").unwrap_or("");
            if currency != decision_currency || line_period != decision_period {
                failure(&mut failures, "journal_decision_currency_period_mismatch");
                journal_lineage_valid = false;
            }
            let key = (currency.clone(), line_entity.to_owned(), line_period.to_owned());
            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or("journal line amount_minor must be an integer")? as i128;
            let decision_totals = decision_postings
                .entry(decision_ref.to_owned())
                .or_default()
                .entry(currency)
                .or_default();
            let totals = balances.entry(key).or_default();
            match string(line, "side")? {
                "debit" => {
                    totals.0 += amount;
                    decision_totals.0 += amount;
                }
                "credit" => {
                    totals.1 += amount;
                    decision_totals.1 += amount;
                }
                other => return Err(format!("invalid journal side {other}").into()),
            }
        }
        if balances.values().any(|(debit, credit)| debit != credit) {
            failure(&mut failures, "journal_entry_unbalanced");
            journal_balanced = false;
        }
    }

    for decision in decisions {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let state = string(decision, "decision_state")?;
        if projection_state == "complete_for_review"
            && matches!(state, "needs_review" | "indeterminate")
        {
            failure(&mut failures, "incomplete_decision_in_complete_projection");
        }
        let has_journal = journaled_decisions.contains(decision_ref);
        if state == "recognized" && !has_journal {
            failure(&mut failures, "recognized_decision_without_journal");
            journal_lineage_valid = false;
        }
        if state != "recognized" && has_journal {
            failure(&mut failures, "nonrecognized_decision_has_journal");
            journal_lineage_valid = false;
        }
        if state == "recognized" {
            let expected_amount = optional_i64(decision, "recognized_amount_minor").unwrap_or(i64::MIN) as i128;
            let expected_currency = optional_string(decision, "currency").unwrap_or("");
            let posted = decision_postings.get(decision_ref);
            let totals = posted.and_then(|map| map.get(expected_currency));
            let amount_matches = totals.is_some_and(|(debit, credit)| {
                *debit == expected_amount && *credit == expected_amount
            });
            let only_expected_currency = posted.is_some_and(|map| {
                map.len() == 1 && map.contains_key(expected_currency)
            });
            if !amount_matches || !only_expected_currency {
                failure(&mut failures, "recognized_amount_does_not_match_journal");
                journal_lineage_valid = false;
            }
            let expected_assertions = string_set(decision, "source_assertion_refs")?;
            let posted_assertions = decision_posted_assertions
                .get(decision_ref)
                .cloned()
                .unwrap_or_default();
            if !expected_assertions.is_subset(&posted_assertions) {
                failure(&mut failures, "journal_decision_evidence_closure_incomplete");
                journal_lineage_valid = false;
            }
        }
    }

    let mut assumption_refs = BTreeMap::<String, &Value>::new();
    for assumption in array(projection, "assumptions")? {
        let assumption_ref = string(assumption, "assumption_ref")?;
        if assumption_refs.insert(assumption_ref.to_owned(), assumption).is_some() {
            failure(&mut failures, "orphan_projection_assumption");
            assertion_provenance_valid = false;
        }
        for assertion_ref in string_set(assumption, "source_assertion_refs")? {
            let Ok(assertion) = assertion_by_ref(graph_case, &assertion_ref) else {
                failure(&mut failures, "assumption_assertion_reference_missing");
                assertion_provenance_valid = false;
                continue;
            };
            if !assertion_available(graph_case, assertion)? {
                failure(&mut failures, "assumption_assertion_unavailable");
                assertion_provenance_valid = false;
            }
        }
    }
    let mut referenced_assumptions = BTreeSet::<String>::new();
    for decision in decisions {
        let decision_assertions = string_set(decision, "source_assertion_refs")?;
        for assumption_ref in string_set(decision, "assumption_refs")? {
            referenced_assumptions.insert(assumption_ref.clone());
            let Some(assumption) = assumption_refs.get(&assumption_ref).copied() else {
                failure(&mut failures, "decision_assumption_reference_missing");
                assertion_provenance_valid = false;
                continue;
            };
            let assumption_assertions = string_set(assumption, "source_assertion_refs")?;
            if !assumption_assertions.is_subset(&decision_assertions) {
                failure(&mut failures, "assumption_assertion_reference_missing");
                assertion_provenance_valid = false;
            }
            let review_state = optional_string(assumption, "review_state");
            let approval_record = optional_string(assumption, "review_record_ref");
            if review_state == Some("approved_by_reviewer")
                && approval_record.is_none_or(str::is_empty)
            {
                failure(&mut failures, "review_approval_record_missing");
                assertion_provenance_valid = false;
            }
            if assumption.get("decision_critical").and_then(Value::as_bool) == Some(true) {
                let adequately_supported = review_state == Some("supported")
                    && !assumption_assertions.is_empty();
                let reviewer_approved = review_state == Some("approved_by_reviewer")
                    && approval_record.is_some_and(|reference| !reference.trim().is_empty());
                if !adequately_supported && !reviewer_approved {
                    failure(&mut failures, "decision_critical_unsupported_assumption");
                    assertion_provenance_valid = false;
                }
            }
            if assumption_assertions.is_empty()
                && optional_string(assumption, "review_state") == Some("supported")
            {
                failure(&mut failures, "assumption_assertion_reference_missing");
                assertion_provenance_valid = false;
            }
        }
    }
    for assumption_ref in assumption_refs.keys() {
        if !referenced_assumptions.contains(assumption_ref) {
            failure(&mut failures, "orphan_projection_assumption");
            assertion_provenance_valid = false;
        }
    }

    if projection_state != "complete_for_review" && total_line_count > 0 {
        failure(&mut failures, "blocked_projection_has_journal");
    }

    let mut unresolved_references_valid = true;
    let mut blocking_unresolved = false;
    for item in array(projection, "unresolved_items")? {
        blocking_unresolved |= item.get("blocking").and_then(Value::as_bool) == Some(true);
        let event_refs = string_set(item, "event_refs")?;
        let assertion_refs = string_set(item, "source_assertion_refs")?;
        for event_ref in &event_refs {
            if !event_ids.contains(event_ref) {
                failure(&mut failures, "unresolved_event_reference_missing");
                unresolved_references_valid = false;
            }
        }
        for assertion_ref in &assertion_refs {
            if assertion_by_ref(graph_case, assertion_ref).is_err() {
                failure(&mut failures, "unresolved_assertion_reference_missing");
                unresolved_references_valid = false;
            }
        }
    }
    if projection_state == "complete_for_review" && blocking_unresolved {
        failure(&mut failures, "blocking_unresolved_item_in_complete_projection");
    }

    let codes = failures.iter().map(|code| (*code).to_owned()).collect::<Vec<_>>();
    let status = if codes.is_empty() { "pass" } else { "reject" };
    Ok(json!({
        "projection_id": projection_id,
        "status": status,
        "graph_binding_valid": graph_binding_valid,
        "policy_binding_valid": policy_binding_valid,
        "projector_binding_valid": projector_binding_valid,
        "event_decision_coverage": event_decision_coverage,
        "assertion_provenance_valid": assertion_provenance_valid,
        "journal_lineage_valid": journal_lineage_valid,
        "journal_balanced": journal_balanced,
        "blocked_state_non_posting": projection_state == "complete_for_review" || journal_entries.is_empty(),
        "unresolved_references_valid": unresolved_references_valid,
        "failure_codes": codes
    }))
}

fn build_report(
    graph: &Value,
    projection_bundle: &Value,
    graph_sha256: String,
    projection_bundle_sha256: String,
) -> Result<Value, String> {
    let graph_case_id = string(projection_bundle, "graph_case_id")?;
    let graph_case = graph_case_by_id(graph, graph_case_id)?;
    let mut projection_ids = BTreeSet::new();
    let mut results = Vec::new();
    for projection in array(projection_bundle, "projections")? {
        let projection_id = string(projection, "projection_id")?;
        if !projection_ids.insert(projection_id.to_owned()) {
            return Err(format!("duplicate projection_id {projection_id}"));
        }
        results.push(evaluate_projection(projection, graph_case, &graph_sha256)?);
    }
    Ok(json!({
        "report_id": "MYCELIX-POLICY-PROJECTION-AUDIT-S0",
        "schema_version": "1.0.0",
        "claim_ceiling": "structural_binding_and_ledger_invariants_not_policy_correctness",
        "graph_sha256": graph_sha256,
        "projection_bundle_sha256": projection_bundle_sha256,
        "graph_case_id": graph_case_id,
        "projection_count": results.len(),
        "projection_results": results
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
    let mut data_dir = root;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a file path")?)),
            "--help" | "-h" => {
                println!(
                    "Audits policy-projection bindings, event coverage, evidence lineage and journal invariants.\n\n\
                     --data-dir PATH  Economic-event artifact directory (default: sibling economic-event-core-v1)\n\
                     --output PATH    Write audit report JSON to PATH\n\
                     --help           Show usage\n\n\
                     A pass verifies structural bindings and ledger invariants only; it does not certify policy correctness."
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let graph_schema: Value = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(data_dir.join("accounting_policy_projection_bundle_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("accounting_policy_projection_audit_report_v1.0.0.schema.json"))?;
    let graph_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
    let projection_path = data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json");
    let graph_bytes = read_bytes(graph_path)?;
    let projection_bytes = read_bytes(projection_path)?;
    let graph: Value = serde_json::from_slice(&graph_bytes)?;
    let projections: Value = serde_json::from_slice(&projection_bytes)?;

    validate_instance("economic event graph", &graph_schema, &graph)?;
    validate_instance("policy projection bundle", &projection_schema, &projections)?;

    let graph_digest = sha256(&graph_bytes);
    let projection_digest = sha256(&projection_bytes);
    let report = build_report(&graph, &projections, graph_digest, projection_digest)?;
    validate_instance("policy projection audit report", &report_schema, &report)?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!("Wrote projection audit for {} projections.", report["projection_count"]);
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, String) {
        let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        let graph_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
        let projection_path = data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json");
        let graph_bytes = read_bytes(graph_path).unwrap();
        (
            serde_json::from_slice(&graph_bytes).unwrap(),
            read_json(projection_path).unwrap(),
            sha256(&graph_bytes),
        )
    }

    fn test_projection() -> Value {
        let (_, bundle, _) = fixtures();
        bundle["projections"][0].clone()
    }

    fn report(graph: &Value, bundle: &Value, digest: &str) -> Value {
        build_report(graph, bundle, digest.to_owned(), "b".repeat(64)).unwrap()
    }

    fn failures(result: &Value) -> BTreeSet<String> {
        array(result, "failure_codes").unwrap()
            .iter()
            .filter_map(Value::as_str)
            .map(ToOwned::to_owned)
            .collect()
    }

    fn make_balanced_projection() -> Value {
        let mut projection = test_projection();
        projection["projection_state"] = Value::String("complete_for_review".to_owned());
        projection["unresolved_items"] = json!([]);
        projection["event_decisions"][0]["decision_state"] = Value::String("recognized".to_owned());
        projection["event_decisions"][0]["recognized_amount_minor"] = Value::from(125000);
        projection["event_decisions"][0]["currency"] = Value::String("USD".to_owned());
        projection["event_decisions"][0]["period_ref"] = Value::String("2026-10".to_owned());
        projection["event_decisions"][0]["treatment_rule_ref"] = Value::String("SYNTHETIC-STRUCTURAL-TEST-RULE".to_owned());
        projection["journal_entries"] = json!([{
            "journal_entry_ref":"SYN-JE-001",
            "entry_kind":"recognition",
            "lines":[
                {
                    "line_ref":"SYN-LINE-DR",
                    "account_ref":"SYN-EXPENSE",
                    "side":"debit",
                    "amount_minor":125000,
                    "currency":"USD",
                    "entity_ref":"ENTITY-ACME",
                    "period_ref":"2026-10",
                    "event_refs":["EV-001-INVOICE"],
                    "source_assertion_refs":["AS-001-AMOUNT","AS-001-SUPPLIER"],
                    "policy_decision_ref":"SYN-PROJECTION-A-DEC-001"
                },
                {
                    "line_ref":"SYN-LINE-CR",
                    "account_ref":"SYN-AP",
                    "side":"credit",
                    "amount_minor":125000,
                    "currency":"USD",
                    "entity_ref":"ENTITY-ACME",
                    "period_ref":"2026-10",
                    "event_refs":["EV-001-INVOICE"],
                    "source_assertion_refs":["AS-001-AMOUNT","AS-001-SUPPLIER"],
                    "policy_decision_ref":"SYN-PROJECTION-A-DEC-001"
                }
            ]
        }]);
        projection
    }

    #[test]
    fn both_review_only_projections_pass_the_independent_audit() {
        let (graph, bundle, digest) = fixtures();
        let result = report(&graph, &bundle, &digest);
        assert_eq!(result["projection_count"], 2);
        for projection in array(&result, "projection_results").unwrap() {
            assert_eq!(projection["status"], "pass");
            assert_eq!(projection["journal_balanced"], true);
            assert_eq!(projection["blocked_state_non_posting"], true);
            assert_eq!(array(projection, "failure_codes").unwrap().len(), 0);
        }
    }

    #[test]
    fn wrong_graph_digest_is_rejected() {
        let (graph, bundle, _) = fixtures();
        let projection = &bundle["projections"][0];
        let case = graph_case_by_id(&graph, string(&bundle, "graph_case_id").unwrap()).unwrap();
        let result = evaluate_projection(projection, case, &"f".repeat(64)).unwrap();
        assert!(failures(&result).contains("graph_digest_mismatch"));
    }

    #[test]
    fn duplicate_event_decision_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = test_projection();
        let duplicate = projection["event_decisions"][0].clone();
        projection["event_decisions"].as_array_mut().unwrap().push(duplicate);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("duplicate_event_decision"));
    }

    #[test]
    fn missing_event_decision_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = test_projection();
        projection["event_decisions"] = json!([]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("missing_event_decision"));
    }

    #[test]
    fn decision_assertion_not_attached_to_event_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = test_projection();
        projection["event_decisions"][0]["source_assertion_refs"] = json!(["AS-002-AMOUNT"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("decision_assertion_not_attached_to_event"));
    }

    #[test]
    fn unavailable_future_assertion_is_rejected() {
        let (mut graph, _, digest) = fixtures();
        let case = graph["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-001-INVOICE-WITHOUT-SERVICE-PROOF")).unwrap();
        case["knowledge_at"] = Value::String("2026-11-05T09:30:00Z".to_owned());
        let projection = test_projection();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("decision_assertion_unavailable"));
    }

    #[test]
    fn review_state_cannot_emit_a_balanced_journal() {
        let (graph, _, digest) = fixtures();
        let mut projection = test_projection();
        projection["journal_entries"] = make_balanced_projection()["journal_entries"].clone();
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("blocked_projection_has_journal"));
    }

    #[test]
    fn complete_projection_cannot_have_blocking_unresolved_items() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["unresolved_items"] = json!([{
            "item_ref":"SYN-BLOCK",
            "reason_code":"missing_acceptance",
            "description":"Service acceptance evidence has not been established.",
            "event_refs":["EV-001-INVOICE"],
            "source_assertion_refs":["AS-001-AMOUNT","AS-001-SUPPLIER"],
            "blocking":true
        }]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("blocking_unresolved_item_in_complete_projection"));
    }

    #[test]
    fn balanced_synthetic_complete_projection_passes_structural_audit() {
        let (graph, _, digest) = fixtures();
        let projection = make_balanced_projection();
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert_eq!(result["status"], "pass");
    }

    #[test]
    fn unbalanced_journal_entry_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][1]["amount_minor"] = Value::from(124999);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_entry_unbalanced"));
    }

    #[test]
    fn debit_and_credit_in_different_currencies_do_not_cancel() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][1]["currency"] = Value::String("EUR".to_owned());
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_entry_unbalanced"));
    }

    #[test]
    fn missing_policy_decision_ref_on_journal_line_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][0]["policy_decision_ref"] = Value::String("DEC-NOT-FOUND".to_owned());
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_policy_decision_missing"));
    }

    #[test]
    fn journal_line_cannot_bind_assertion_from_another_event() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][0]["source_assertion_refs"] = json!(["AS-002-AMOUNT"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_assertion_lineage_mismatch"));
    }

    #[test]
    fn nonrecognized_decision_cannot_have_journal_lines() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["event_decisions"][0]["decision_state"] = Value::String("deferred".to_owned());
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("nonrecognized_decision_has_journal"));
    }

    #[test]
    fn recognized_decision_must_have_a_journal() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"] = json!([]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("recognized_decision_without_journal"));
    }


    #[test]
    fn balanced_journal_with_wrong_recognized_amount_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][0]["amount_minor"] = Value::from(126000);
        projection["journal_entries"][0]["lines"][1]["amount_minor"] = Value::from(126000);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("recognized_amount_does_not_match_journal"));
        assert_eq!(result["journal_balanced"], true);
    }

    #[test]
    fn decision_source_evidence_must_all_reach_the_journal() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        for line in projection["journal_entries"][0]["lines"].as_array_mut().unwrap() {
            line["source_assertion_refs"] = json!(["AS-001-AMOUNT"]);
        }
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_decision_evidence_closure_incomplete"));
    }

    #[test]
    fn missing_decision_assumption_reference_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["event_decisions"][0]["assumption_refs"] = json!(["ASSUMPTION-MISSING"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("decision_assumption_reference_missing"));
    }

    #[test]
    fn orphan_projection_assumption_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["assumptions"] = json!([{
            "assumption_ref":"ASSUMPTION-ORPHAN",
            "assumption_kind":"timing",
            "description":"An unused assumption should not be hidden in the projection.",
            "value":"2026-10",
            "source_assertion_refs":["AS-001-AMOUNT"],
            "review_state":"supported",
            "decision_critical":false
        }]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("orphan_projection_assumption"));
    }

    #[test]
    fn unsupported_decision_critical_assumption_is_rejected() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["assumptions"] = json!([{
            "assumption_ref":"ASSUMPTION-FX",
            "assumption_kind":"currency",
            "description":"Assume a convenient foreign exchange rate.",
            "value":1.2,
            "source_assertion_refs":[],
            "review_state":"assumed",
            "decision_critical":true
        }]);
        projection["event_decisions"][0]["assumption_refs"] = json!(["ASSUMPTION-FX"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("decision_critical_unsupported_assumption"));
    }

    #[test]
    fn supported_assumption_must_cite_available_event_evidence() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["assumptions"] = json!([{
            "assumption_ref":"ASSUMPTION-SUPPORTED",
            "assumption_kind":"classification",
            "description":"Use a source-backed classification assumption.",
            "value":"EXPENSE",
            "source_assertion_refs":["AS-002-AMOUNT"],
            "review_state":"supported",
            "decision_critical":true
        }]);
        projection["event_decisions"][0]["assumption_refs"] = json!(["ASSUMPTION-SUPPORTED"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("assumption_assertion_reference_missing"));
    }

    #[test]
    fn decision_period_and_currency_must_match_journal_lines() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["event_decisions"][0]["period_ref"] = Value::String("2026-09".to_owned());
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_decision_currency_period_mismatch"));
    }


    #[test]
    fn reviewer_approved_critical_assumption_requires_record_reference() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["assumptions"] = json!([{
            "assumption_ref":"ASSUMPTION-REVIEWED",
            "assumption_kind":"classification",
            "description":"The reviewer approved this synthetic classification assumption.",
            "value":"EXPENSE",
            "source_assertion_refs":[],
            "review_state":"approved_by_reviewer",
            "decision_critical":true,
            "review_record_ref":null
        }]);
        projection["event_decisions"][0]["assumption_refs"] = json!(["ASSUMPTION-REVIEWED"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("review_approval_record_missing"));
    }

    #[test]
    fn complete_projection_cannot_hide_needs_review_event_decision() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["event_decisions"][0]["decision_state"] = Value::String("needs_review".to_owned());
        projection["journal_entries"] = json!([]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("incomplete_decision_in_complete_projection"));
    }

    #[test]
    fn unresolved_item_must_reference_real_graph_objects() {
        let (graph, _, digest) = fixtures();
        let mut projection = test_projection();
        projection["unresolved_items"][0]["event_refs"] = json!(["EV-MISSING"]);
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("unresolved_event_reference_missing"));
    }

    #[test]
    fn journal_entity_must_match_projection_entity() {
        let (graph, _, digest) = fixtures();
        let mut projection = make_balanced_projection();
        projection["journal_entries"][0]["lines"][0]["entity_ref"] = Value::String("ENTITY-OTHER".to_owned());
        let case = graph_case_by_id(&graph, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF").unwrap();
        let result = evaluate_projection(&projection, case, &digest).unwrap();
        assert!(failures(&result).contains("journal_entity_period_mismatch"));
    }
}
