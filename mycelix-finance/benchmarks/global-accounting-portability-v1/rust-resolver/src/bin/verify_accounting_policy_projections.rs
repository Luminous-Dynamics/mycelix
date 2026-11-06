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
        eprintln!("accounting-policy-projection-verifier: {error}");
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

fn find_by<'a>(values: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn event_by_ref<'a>(graph_case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by(array(graph_case, "events")?, "event_ref", reference)
        .ok_or_else(|| format!("unknown graph event reference {reference}"))
}

fn assertion_by_ref<'a>(graph_case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by(array(graph_case, "assertions")?, "assertion_ref", reference)
        .ok_or_else(|| format!("unknown graph assertion reference {reference}"))
}

fn set_from_array(value: &Value, key: &str) -> Result<BTreeSet<String>, String> {
    array(value, key)?
        .iter()
        .map(|item| {
            item.as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| format!("{key} must contain strings"))
        })
        .collect()
}

fn unique_refs(values: &[Value], field: &str, label: &str) -> Result<BTreeMap<String, &Value>, String> {
    let mut refs = BTreeMap::new();
    for value in values {
        let reference = string(value, field)?.to_owned();
        if refs.insert(reference.clone(), value).is_some() {
            return Err(format!("{label}: duplicate {field} {reference}"));
        }
    }
    Ok(refs)
}

fn validate_graph_binding(
    bundle: &Value,
    projection: &Value,
    graph_case: &Value,
    actual_graph_digest: &str,
) -> Result<(), String> {
    let case_id = string(bundle, "graph_case_id")?;
    let bundle_digest = string(bundle, "graph_sha256")?;
    let projection_id = string(projection, "projection_id")?;
    let binding = projection.get("graph_binding").ok_or("missing graph_binding")?;
    let projection_case_id = string(binding, "graph_case_id")?;
    let projection_digest = string(binding, "graph_sha256")?;

    if case_id != string(graph_case, "case_id")? || projection_case_id != case_id {
        return Err(format!("{projection_id}: graph case binding mismatch"));
    }
    if bundle_digest != actual_graph_digest || projection_digest != actual_graph_digest {
        return Err(format!(
            "{projection_id}: event graph digest mismatch; expected {actual_graph_digest}, bundle={bundle_digest}, projection={projection_digest}"
        ));
    }
    Ok(())
}

fn validate_event_decisions(
    projection: &Value,
    graph_case: &Value,
) -> Result<BTreeMap<String, &Value>, String> {
    let projection_id = string(projection, "projection_id")?;
    let events = unique_refs(array(graph_case, "events")?, "event_ref", "graph events")?;
    let decisions = array(projection, "event_decisions")?;
    let decision_refs = unique_refs(decisions, "policy_decision_ref", "policy decisions")?;
    let mut event_decision_refs = BTreeSet::new();

    for decision in decisions {
        let event_ref = string(decision, "event_ref")?;
        if !events.contains_key(event_ref) {
            return Err(format!("{projection_id}: decision references unknown event {event_ref}"));
        }
        if !event_decision_refs.insert(event_ref.to_owned()) {
            return Err(format!("{projection_id}: event {event_ref} has more than one policy decision"));
        }

        let event = events[event_ref];
        let event_assertions = set_from_array(event, "assertion_refs")?;
        let decision_assertions = set_from_array(decision, "source_assertion_refs")?;
        if decision_assertions.is_empty() || !decision_assertions.is_subset(&event_assertions) {
            return Err(format!(
                "{projection_id}: decision {} cites assertions not attached to event {event_ref}",
                string(decision, "policy_decision_ref")?
            ));
        }
        for assertion_ref in &decision_assertions {
            assertion_by_ref(graph_case, assertion_ref)?;
        }

        let state = string(decision, "decision_state")?;
        let amount_is_null = decision
            .get("recognized_amount_minor")
            .is_none_or(Value::is_null);
        let currency_is_null = decision.get("currency").is_none_or(Value::is_null);
        let period_is_null = decision.get("period_ref").is_none_or(Value::is_null);
        if state == "recognized" && (amount_is_null || currency_is_null || period_is_null) {
            return Err(format!(
                "{projection_id}: recognized decision {} requires amount, currency and period",
                string(decision, "policy_decision_ref")?
            ));
        }
        if matches!(state, "needs_review" | "indeterminate" | "not_recognized" | "deferred" | "excluded")
            && !(amount_is_null && currency_is_null && period_is_null)
        {
            return Err(format!(
                "{projection_id}: non-recognition/review decision {} must not carry a recognized amount",
                string(decision, "policy_decision_ref")?
            ));
        }

        let assumptions = set_from_array(decision, "assumption_refs")?;
        let known_assumptions = array(projection, "assumptions")?
            .iter()
            .filter_map(|assumption| optional_string(assumption, "assumption_ref"))
            .collect::<BTreeSet<_>>();
        if !assumptions.iter().all(|reference| known_assumptions.contains(reference.as_str())) {
            return Err(format!(
                "{projection_id}: decision {} references an unknown assumption",
                string(decision, "policy_decision_ref")?
            ));
        }
    }
    Ok(decision_refs)
}

fn validate_assumptions(projection: &Value, graph_case: &Value) -> Result<bool, String> {
    let projection_id = string(projection, "projection_id")?;
    let assumptions = array(projection, "assumptions")?;
    let refs = unique_refs(assumptions, "assumption_ref", "projection assumptions")?;
    for assumption in assumptions {
        for assertion_ref in array(assumption, "source_assertion_refs")? {
            let reference = assertion_ref
                .as_str()
                .ok_or("assumption source_assertion_refs must contain strings")?;
            assertion_by_ref(graph_case, reference)?;
        }
    }
    let has_unapproved_critical = assumptions.iter().any(|assumption| {
        assumption.get("decision_critical").and_then(Value::as_bool) == Some(true)
            && optional_string(assumption, "review_state") != Some("approved_by_reviewer")
    });
    if string(projection, "projection_state")? == "complete_for_review" && has_unapproved_critical {
        return Err(format!(
            "{projection_id}: complete_for_review cannot contain an unapproved decision-critical assumption"
        ));
    }
    let _ = refs;
    Ok(has_unapproved_critical)
}

fn validate_journal_entries(
    projection: &Value,
    graph_case: &Value,
    decision_refs: &BTreeMap<String, &Value>,
) -> Result<(usize, usize), String> {
    let projection_id = string(projection, "projection_id")?;
    let all_events = unique_refs(array(graph_case, "events")?, "event_ref", "graph events")?;
    let mut entry_refs = BTreeSet::new();
    let mut line_refs = BTreeSet::new();
    let mut total_lines = 0usize;

    for entry in array(projection, "journal_entries")? {
        let entry_ref = string(entry, "journal_entry_ref")?;
        if !entry_refs.insert(entry_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate journal entry {entry_ref}"));
        }
        let mut totals = BTreeMap::<(String, String, String), (i128, i128)>::new();

        for line in array(entry, "lines")? {
            total_lines += 1;
            let line_ref = string(line, "line_ref")?;
            if !line_refs.insert(line_ref.to_owned()) {
                return Err(format!("{projection_id}: duplicate journal line {line_ref}"));
            }

            let entity_ref = string(line, "entity_ref")?;
            if entity_ref != string(projection, "entity_ref")? {
                return Err(format!("{projection_id}/{line_ref}: line entity differs from projection entity"));
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            let decision = decision_refs
                .get(decision_ref)
                .ok_or_else(|| format!("{projection_id}/{line_ref}: unknown policy decision {decision_ref}"))?;
            if optional_string(decision, "decision_state") != Some("recognized") {
                return Err(format!("{projection_id}/{line_ref}: journal line is linked to a non-recognized decision"));
            }

            let line_event_refs = set_from_array(line, "event_refs")?;
            let decision_event_ref = string(decision, "event_ref")?;
            if !line_event_refs.contains(decision_event_ref) {
                return Err(format!("{projection_id}/{line_ref}: journal line omits its decision event"));
            }
            for event_ref in &line_event_refs {
                if !all_events.contains_key(event_ref) {
                    return Err(format!("{projection_id}/{line_ref}: unknown graph event {event_ref}"));
                }
            }

            let line_assertions = set_from_array(line, "source_assertion_refs")?;
            let decision_assertions = set_from_array(decision, "source_assertion_refs")?;
            if line_assertions.is_empty() || !line_assertions.is_subset(&decision_assertions) {
                return Err(format!("{projection_id}/{line_ref}: line assertions exceed or omit policy decision evidence"));
            }
            for assertion_ref in &line_assertions {
                assertion_by_ref(graph_case, assertion_ref)?;
            }

            let line_currency = string(line, "currency")?;
            let line_period = string(line, "period_ref")?;
            if optional_string(decision, "currency") != Some(line_currency)
                || optional_string(decision, "period_ref") != Some(line_period)
            {
                return Err(format!("{projection_id}/{line_ref}: line currency/period differs from its decision"));
            }

            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or_else(|| format!("{projection_id}/{line_ref}: amount_minor must be a signed integer"))?;
            if amount <= 0 {
                return Err(format!("{projection_id}/{line_ref}: amount_minor must be positive"));
            }
            let key = (
                line_currency.to_owned(),
                entity_ref.to_owned(),
                line_period.to_owned(),
            );
            let totals = totals.entry(key).or_default();
            match string(line, "side")? {
                "debit" => totals.0 += amount as i128,
                "credit" => totals.1 += amount as i128,
                other => return Err(format!("{projection_id}/{line_ref}: invalid journal side {other}")),
            }
        }

        if totals.values().any(|(debits, credits)| debits != credits) {
            return Err(format!("{projection_id}/{entry_ref}: journal entry is not balanced by currency/entity/period"));
        }
    }
    Ok((entry_refs.len(), total_lines))
}

fn validate_unresolved_items(projection: &Value, graph_case: &Value) -> Result<usize, String> {
    let projection_id = string(projection, "projection_id")?;
    let events = unique_refs(array(graph_case, "events")?, "event_ref", "graph events")?;
    let mut item_refs = BTreeSet::new();
    let mut blocking = 0usize;
    for item in array(projection, "unresolved_items")? {
        let item_ref = string(item, "item_ref")?;
        if !item_refs.insert(item_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate unresolved item {item_ref}"));
        }
        for event_ref in array(item, "event_refs")? {
            let reference = event_ref.as_str().ok_or("unresolved event ref must be a string")?;
            if !events.contains_key(reference) {
                return Err(format!("{projection_id}/{item_ref}: unknown event {reference}"));
            }
        }
        for assertion_ref in array(item, "source_assertion_refs")? {
            let reference = assertion_ref
                .as_str()
                .ok_or("unresolved source assertion ref must be a string")?;
            assertion_by_ref(graph_case, reference)?;
        }
        if item.get("blocking").and_then(Value::as_bool) == Some(true) {
            blocking += 1;
        }
    }
    if string(projection, "projection_state")? == "complete_for_review" && blocking > 0 {
        return Err(format!(
            "{projection_id}: complete_for_review cannot contain blocking unresolved items"
        ));
    }
    Ok(blocking)
}

fn validate_projection(
    bundle: &Value,
    projection: &Value,
    graph_case: &Value,
    actual_graph_digest: &str,
) -> Result<Value, String> {
    validate_graph_binding(bundle, projection, graph_case, actual_graph_digest)?;
    let decision_refs = validate_event_decisions(projection, graph_case)?;
    let has_unapproved_critical = validate_assumptions(projection, graph_case)?;
    let (entry_count, line_count) =
        validate_journal_entries(projection, graph_case, &decision_refs)?;
    let blocking_items = validate_unresolved_items(projection, graph_case)?;

    let state = string(projection, "projection_state")?;
    if state == "not_projected" && (entry_count > 0 || !array(projection, "event_decisions")?.is_empty()) {
        return Err(format!(
            "{}: not_projected must not contain decisions or journal entries",
            string(projection, "projection_id")?
        ));
    }
    if state == "complete_for_review" {
        for decision in array(projection, "event_decisions")? {
            if matches!(optional_string(decision, "decision_state"), Some("needs_review" | "indeterminate")) {
                return Err(format!(
                    "{}: complete_for_review contains unresolved event decision {}",
                    string(projection, "projection_id")?,
                    string(decision, "policy_decision_ref")?
                ));
            }
        }
    }

    Ok(json!({
        "projection_id": string(projection, "projection_id")?,
        "policy_profile_ref": string(projection.get("policy_binding").ok_or("missing policy_binding")?, "policy_profile_ref")?,
        "policy_version": string(projection.get("policy_binding").ok_or("missing policy_binding")?, "policy_version")?,
        "policy_sha256": string(projection.get("policy_binding").ok_or("missing policy_binding")?, "policy_sha256")?,
        "projector_version": string(projection.get("projector_binding").ok_or("missing projector_binding")?, "projector_version")?,
        "projector_sha256": string(projection.get("projector_binding").ok_or("missing projector_binding")?, "projector_sha256")?,
        "projection_state": state,
        "event_decision_count": array(projection, "event_decisions")?.len(),
        "journal_entry_count": entry_count,
        "journal_line_count": line_count,
        "blocking_unresolved_item_count": blocking_items,
        "unapproved_decision_critical_assumption_present": has_unapproved_critical,
        "validation_state": "structurally_and_provenance_valid_for_independent_review"
    }))
}

fn validate_bundle(
    bundle: &Value,
    graph: &Value,
    graph_digest: &str,
) -> Result<Value, String> {
    let graph_case_id = string(bundle, "graph_case_id")?;
    let graph_case = array(graph, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(graph_case_id))
        .ok_or_else(|| format!("unknown graph case {graph_case_id}"))?;

    if string(bundle, "graph_sha256")? != graph_digest {
        return Err(format!(
            "projection bundle graph digest differs from exact candidate bytes: expected {graph_digest}"
        ));
    }
    let projections = array(bundle, "projections")?;
    if projections.len() < 2 {
        return Err("review control bundle must include at least two policy projections".to_owned());
    }
    unique_refs(projections, "projection_id", "projections")?;

    let mut results = Vec::new();
    let mut policy_digests = BTreeSet::new();
    let mut projector_digests = BTreeSet::new();
    for projection in projections {
        policy_digests.insert(string(projection.get("policy_binding").ok_or("missing policy_binding")?, "policy_sha256")?.to_owned());
        projector_digests.insert(string(projection.get("projector_binding").ok_or("missing projector_binding")?, "projector_sha256")?.to_owned());
        results.push(validate_projection(bundle, projection, graph_case, graph_digest)?);
    }

    Ok(json!({
        "report_id": "MYCELIX-POLICY-PROJECTION-VERIFICATION-S0",
        "schema_version": "1.0.0",
        "claim_ceiling": "binding_provenance_and_ledger_invariants_only_not_accounting_policy_correctness",
        "graph_case_id": graph_case_id,
        "graph_sha256": graph_digest,
        "projection_count": projections.len(),
        "distinct_policy_digest_count": policy_digests.len(),
        "distinct_projector_digest_count": projector_digests.len(),
        "projections": results
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
                    "Verifies policy projection provenance, decision links and journal invariants against a frozen event graph.\n\n\
                     --data-dir PATH  Event-core artifact directory (default: sibling economic-event-core-v1)\n\
                     --output PATH    Write verification report JSON to PATH\n\
                     --help           Show usage\n\n\
                     This verifier does not implement GAAP/IFRS rules or authorize ledger posting."
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let graph_schema: Value = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let bundle_schema: Value = read_json(data_dir.join("accounting_policy_projection_bundle_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(data_dir.join("accounting_policy_projection_v1.0.0.schema.json"))?;
    let graph_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
    let bundle_path = data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json");
    let graph_bytes = read_bytes(graph_path)?;
    let graph: Value = serde_json::from_slice(&graph_bytes)?;
    let bundle: Value = read_json(bundle_path)?;

    validate_instance("economic event graph", &graph_schema, &graph)?;
    validate_instance("policy projection bundle", &bundle_schema, &bundle)?;
    for projection in array(&bundle, "projections")? {
        validate_instance("accounting policy projection", &projection_schema, projection)?;
    }

    let graph_digest = hex::encode(Sha256::digest(&graph_bytes));
    let report = validate_bundle(&bundle, &graph, &graph_digest)?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!(
                "Wrote verification report for {} policy projections.",
                report["projection_count"]
            );
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value, String) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        let graph_path = root.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
        let graph_bytes = fs::read(&graph_path).unwrap();
        let graph: Value = serde_json::from_slice(&graph_bytes).unwrap();
        let bundle: Value = read_json(root.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json")).unwrap();
        let graph_schema: Value = read_json(root.join("economic_event_graph_v1.0.0.schema.json")).unwrap();
        let projection_schema: Value = read_json(root.join("accounting_policy_projection_v1.0.0.schema.json")).unwrap();
        let digest = hex::encode(Sha256::digest(&graph_bytes));
        (graph, bundle, graph_schema, projection_schema, digest)
    }

    fn graph_case<'a>(graph: &'a Value, case_id: &str) -> &'a Value {
        array(graph, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some(case_id)).unwrap()
    }

    #[test]
    fn two_review_only_policy_projections_bind_to_the_same_exact_graph_bytes() {
        let (graph, bundle, graph_schema, projection_schema, digest) = fixtures();
        validate_instance("graph", &graph_schema, &graph).unwrap();
        for projection in array(&bundle, "projections").unwrap() {
            validate_instance("projection", &projection_schema, projection).unwrap();
        }
        assert_eq!(bundle["graph_sha256"], digest);
        let report = validate_bundle(&bundle, &graph, &digest).unwrap();
        assert_eq!(report["projection_count"], 2);
        assert_eq!(report["distinct_policy_digest_count"], 2);
        assert_eq!(report["distinct_projector_digest_count"], 2);
    }

    #[test]
    fn exact_graph_digest_mismatch_is_rejected() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["graph_sha256"] = Value::String("0".repeat(64));
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn unknown_event_decision_reference_is_rejected() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["event_ref"] = Value::String("EV-MISSING".to_owned());
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn source_assertion_must_belong_to_the_decided_event() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["source_assertion_refs"] = json!(["AS-002-AMOUNT"]);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn review_decision_cannot_claim_a_recognized_amount() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["recognized_amount_minor"] = Value::from(125000);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn recognized_decision_requires_amount_currency_and_period() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let decision = &mut bundle["projections"][0]["event_decisions"][0];
        decision["decision_state"] = Value::String("recognized".to_owned());
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn journal_line_must_reference_an_existing_decision() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let p = &mut bundle["projections"][0];
        p["projection_state"] = Value::String("complete_for_review".to_owned());
        p["event_decisions"][0]["decision_state"] = Value::String("recognized".to_owned());
        p["event_decisions"][0]["recognized_amount_minor"] = Value::from(125000);
        p["event_decisions"][0]["currency"] = Value::String("USD".to_owned());
        p["event_decisions"][0]["period_ref"] = Value::String("2026-10".to_owned());
        p["unresolved_items"][0]["blocking"] = Value::Bool(false);
        p["journal_entries"] = json!([{
            "journal_entry_ref":"JE-TEST",
            "entry_kind":"recognition",
            "lines":[
                {"line_ref":"JL-TEST-DR","account_ref":"EXP","side":"debit","amount_minor":125000,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"DEC-MISSING"},
                {"line_ref":"JL-TEST-CR","account_ref":"AP","side":"credit","amount_minor":125000,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"DEC-MISSING"}
            ]
        }]);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn unbalanced_journal_entry_is_rejected_even_if_projection_schema_is_valid() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let p = &mut bundle["projections"][0];
        p["projection_state"] = Value::String("complete_for_review".to_owned());
        p["event_decisions"][0]["decision_state"] = Value::String("recognized".to_owned());
        p["event_decisions"][0]["recognized_amount_minor"] = Value::from(125000);
        p["event_decisions"][0]["currency"] = Value::String("USD".to_owned());
        p["event_decisions"][0]["period_ref"] = Value::String("2026-10".to_owned());
        p["unresolved_items"][0]["blocking"] = Value::Bool(false);
        p["journal_entries"] = json!([{
            "journal_entry_ref":"JE-TEST",
            "entry_kind":"recognition",
            "lines":[
                {"line_ref":"JL-TEST-DR","account_ref":"EXP","side":"debit","amount_minor":125000,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"},
                {"line_ref":"JL-TEST-CR","account_ref":"AP","side":"credit","amount_minor":124999,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"}
            ]
        }]);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn journal_source_assertions_must_be_subset_of_decision_evidence() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let p = &mut bundle["projections"][0];
        p["projection_state"] = Value::String("complete_for_review".to_owned());
        p["event_decisions"][0]["decision_state"] = Value::String("recognized".to_owned());
        p["event_decisions"][0]["recognized_amount_minor"] = Value::from(125000);
        p["event_decisions"][0]["currency"] = Value::String("USD".to_owned());
        p["event_decisions"][0]["period_ref"] = Value::String("2026-10".to_owned());
        p["unresolved_items"][0]["blocking"] = Value::Bool(false);
        p["journal_entries"] = json!([{
            "journal_entry_ref":"JE-TEST",
            "entry_kind":"recognition",
            "lines":[
                {"line_ref":"JL-TEST-DR","account_ref":"EXP","side":"debit","amount_minor":125000,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-002-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"},
                {"line_ref":"JL-TEST-CR","account_ref":"AP","side":"credit","amount_minor":125000,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-002-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"}
            ]
        }]);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn blocking_unresolved_item_prevents_complete_for_review() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["projection_state"] = Value::String("complete_for_review".to_owned());
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn every_projection_decision_source_ref_must_exist_on_graph() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["source_assertion_refs"] = json!(["AS-MISSING"]);
        assert!(validate_bundle(&bundle, &graph, &digest).is_err());
    }

    #[test]
    fn multiple_projections_keep_the_same_graph_but_distinct_policy_and_projector_digests() {
        let (graph, bundle, _, _, digest) = fixtures();
        let report = validate_bundle(&bundle, &graph, &digest).unwrap();
        assert_eq!(report["graph_sha256"], digest);
        assert_eq!(report["distinct_policy_digest_count"], 2);
        assert_eq!(report["distinct_projector_digest_count"], 2);
    }
}
