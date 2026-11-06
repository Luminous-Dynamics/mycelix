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
        eprintln!("accounting-policy-projection-s0: {error}");
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

fn find_by_ref<'a>(values: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
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

fn graph_case<'a>(graph: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(graph, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("unknown graph case {case_id}"))
}

fn validate_projection(
    graph_case: &Value,
    projection: &Value,
    expected_graph_digest: &str,
) -> Result<Value, String> {
    let projection_id = string(projection, "projection_id")?;
    let graph_binding = projection.get("graph_binding").ok_or("missing graph_binding")?;
    let graph_case_id = string(graph_binding, "graph_case_id")?;
    let graph_digest = string(graph_binding, "graph_sha256")?;
    if graph_case_id != string(graph_case, "case_id")? || graph_digest != expected_graph_digest {
        return Err(format!("{projection_id}: projection graph binding does not match exact graph input"));
    }
    if optional_string(projection, "claim_ceiling")
        != Some("policy_specific_projection_output_requires_independent_review_not_universal_accounting_truth")
    {
        return Err(format!("{projection_id}: unexpected policy-projection claim ceiling"));
    }

    let graph_events = array(graph_case, "events")?;
    let graph_assertions = array(graph_case, "assertions")?;
    let event_map = graph_events
        .iter()
        .map(|event| Ok((string(event, "event_ref")?.to_owned(), event)))
        .collect::<Result<BTreeMap<_, _>, String>>()?;
    let assertion_map = graph_assertions
        .iter()
        .map(|assertion| Ok((string(assertion, "assertion_ref")?.to_owned(), assertion)))
        .collect::<Result<BTreeMap<_, _>, String>>()?;

    let policy_binding = projection.get("policy_binding").ok_or("missing policy_binding")?;
    let projector_binding = projection.get("projector_binding").ok_or("missing projector_binding")?;
    let policy_digest = string(policy_binding, "policy_sha256")?;
    let projector_digest = string(projector_binding, "projector_sha256")?;
    let entity_ref = string(projection, "entity_ref")?;
    let period = projection.get("period").ok_or("missing projection period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    if period_start > period_end {
        return Err(format!("{projection_id}: inverted projection period"));
    }

    let mut decision_refs = BTreeSet::new();
    let mut decisions_by_ref = BTreeMap::<String, (&Value, String, BTreeSet<String>)>::new();
    let mut event_decisions = BTreeSet::new();
    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let event_ref = string(decision, "event_ref")?;
        if !decision_refs.insert(decision_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate policy_decision_ref {decision_ref}"));
        }
        if !event_decisions.insert(event_ref.to_owned()) {
            return Err(format!("{projection_id}: event {event_ref} has more than one policy decision"));
        }
        let event = event_map
            .get(event_ref)
            .ok_or_else(|| format!("{projection_id}: decision references unknown event {event_ref}"))?;
        if string(event, "entity_ref")? != entity_ref {
            return Err(format!("{projection_id}/{event_ref}: event entity differs from projection entity"));
        }

        let decision_sources = set_from_array(decision, "source_assertion_refs")?;
        if decision_sources.is_empty() {
            return Err(format!("{projection_id}/{decision_ref}: policy decision has no source assertions"));
        }
        let event_sources = set_from_array(event, "assertion_refs")?;
        for source_ref in &decision_sources {
            if !assertion_map.contains_key(source_ref) {
                return Err(format!("{projection_id}/{decision_ref}: unknown source assertion {source_ref}"));
            }
            if !event_sources.contains(source_ref) {
                return Err(format!("{projection_id}/{decision_ref}: source assertion {source_ref} does not support event {event_ref}"));
            }
        }

        let decision_state = string(decision, "decision_state")?;
        if decision_state == "recognized" {
            let amount = decision
                .get("recognized_amount_minor")
                .and_then(Value::as_i64)
                .ok_or_else(|| format!("{projection_id}/{decision_ref}: recognized decision has no integer amount"))?;
            if amount <= 0 || value_is_null(decision, "currency") || value_is_null(decision, "period_ref") {
                return Err(format!("{projection_id}/{decision_ref}: recognized decision lacks positive measurement, currency or period"));
            }
        }
        decisions_by_ref.insert(
            decision_ref.to_owned(),
            (decision, event_ref.to_owned(), decision_sources),
        );
    }

    let allowed_currencies = [
        string(projection, "functional_currency")?,
        string(projection, "presentation_currency")?,
    ]
    .into_iter()
    .collect::<BTreeSet<_>>();

    let mut entry_refs = BTreeSet::new();
    let mut line_refs = BTreeSet::new();
    let mut journal_line_count = 0usize;
    let mut journals_balanced = true;
    for entry in array(projection, "journal_entries")? {
        let entry_ref = string(entry, "journal_entry_ref")?;
        if !entry_refs.insert(entry_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate journal entry ref {entry_ref}"));
        }
        let mut totals = BTreeMap::<(String, String, String), (i128, i128)>::new();
        for line in array(entry, "lines")? {
            journal_line_count += 1;
            let line_ref = string(line, "line_ref")?;
            if !line_refs.insert(line_ref.to_owned()) {
                return Err(format!("{projection_id}: duplicate journal line ref {line_ref}"));
            }
            if string(line, "entity_ref")? != entity_ref {
                return Err(format!("{projection_id}/{line_ref}: line entity differs from projection entity"));
            }
            let currency = string(line, "currency")?;
            if !allowed_currencies.contains(currency) {
                return Err(format!("{projection_id}/{line_ref}: line currency is not functional or presentation currency"));
            }
            let period_ref = string(line, "period_ref")?;
            let month_start = format!("{}-01", &period_ref);
            let month_end = format!("{}-31", &period_ref);
            if month_end.as_str() < period_start || month_start.as_str() > period_end {
                return Err(format!("{projection_id}/{line_ref}: journal period is outside projection period"));
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            let (decision, decision_event_ref, decision_sources) = decisions_by_ref
                .get(decision_ref)
                .ok_or_else(|| format!("{projection_id}/{line_ref}: no decision for policy_decision_ref {decision_ref}"))?;
            if optional_string(decision, "decision_state") != Some("recognized") {
                return Err(format!("{projection_id}/{line_ref}: journal line has no recognized policy decision"));
            }

            let line_event_refs = set_from_array(line, "event_refs")?;
            if line_event_refs.is_empty() || !line_event_refs.contains(decision_event_ref) {
                return Err(format!("{projection_id}/{line_ref}: journal line event refs do not include its decision event"));
            }
            for event_ref in &line_event_refs {
                let event = event_map
                    .get(event_ref)
                    .ok_or_else(|| format!("{projection_id}/{line_ref}: unknown event {event_ref}"))?;
                if string(event, "entity_ref")? != entity_ref {
                    return Err(format!("{projection_id}/{line_ref}: journal line references an event from another entity"));
                }
            }

            let line_sources = set_from_array(line, "source_assertion_refs")?;
            if line_sources.is_empty() || !line_sources.is_subset(decision_sources) {
                return Err(format!("{projection_id}/{line_ref}: journal line source refs are absent or exceed decision evidence"));
            }
            for assertion_ref in &line_sources {
                if !assertion_map.contains_key(assertion_ref) {
                    return Err(format!("{projection_id}/{line_ref}: unknown assertion {assertion_ref}"));
                }
            }

            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or_else(|| format!("{projection_id}/{line_ref}: amount_minor must be an integer"))?;
            if amount <= 0 {
                return Err(format!("{projection_id}/{line_ref}: amount_minor must be positive"));
            }
            let key = (
                currency.to_owned(),
                entity_ref.to_owned(),
                period_ref.to_owned(),
            );
            let group = totals.entry(key).or_default();
            match string(line, "side")? {
                "debit" => group.0 += amount as i128,
                "credit" => group.1 += amount as i128,
                other => return Err(format!("{projection_id}/{line_ref}: unknown side {other}")),
            }
        }
        if totals.values().any(|(debits, credits)| debits != credits) {
            journals_balanced = false;
            return Err(format!("{projection_id}/{entry_ref}: journal entry is not balanced per currency/entity/period"));
        }
    }

    if optional_string(projection, "projection_state") == Some("not_projected")
        && !array(projection, "journal_entries")?.is_empty()
    {
        return Err(format!("{projection_id}: not_projected output cannot contain journal entries"));
    }

    let mut unresolved_refs = BTreeSet::new();
    let mut blocking_unresolved = false;
    for unresolved in array(projection, "unresolved_items")? {
        let item_ref = string(unresolved, "item_ref")?;
        if !unresolved_refs.insert(item_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate unresolved item {item_ref}"));
        }
        let event_refs = set_from_array(unresolved, "event_refs")?;
        let assertion_refs = set_from_array(unresolved, "source_assertion_refs")?;
        if event_refs.is_empty() && assertion_refs.is_empty() {
            return Err(format!("{projection_id}/{item_ref}: unresolved item has no event or assertion locator"));
        }
        for event_ref in &event_refs {
            if !event_map.contains_key(event_ref) {
                return Err(format!("{projection_id}/{item_ref}: unknown event ref {event_ref}"));
            }
        }
        for assertion_ref in &assertion_refs {
            if !assertion_map.contains_key(assertion_ref) {
                return Err(format!("{projection_id}/{item_ref}: unknown assertion ref {assertion_ref}"));
            }
        }
        blocking_unresolved |= unresolved.get("blocking").and_then(Value::as_bool) == Some(true);
    }

    let findings = if blocking_unresolved {
        vec!["blocking_unresolved_item"]
    } else {
        Vec::<&str>::new()
    };
    Ok(json!({
        "projection_id": projection_id,
        "policy_sha256": policy_digest,
        "projector_sha256": projector_digest,
        "projection_state": string(projection, "projection_state")?,
        "event_decision_count": array(projection, "event_decisions")?.len(),
        "journal_entry_count": array(projection, "journal_entries")?.len(),
        "journal_line_count": journal_line_count,
        "unresolved_item_count": array(projection, "unresolved_items")?.len(),
        "journals_balanced": journals_balanced,
        "findings": findings
    }))
}

fn value_is_null(value: &Value, key: &str) -> bool {
    value.get(key).is_none_or(Value::is_null)
}

fn build_report(
    graph: &Value,
    graph_digest: &str,
    bundle: &Value,
) -> Result<Value, String> {
    if string(bundle, "graph_sha256")? != graph_digest {
        return Err("projection bundle graph_sha256 does not match exact candidate bytes".to_owned());
    }
    let case = graph_case(graph, string(bundle, "graph_case_id")?)?;
    let mut projection_ids = BTreeSet::new();
    let mut results = Vec::new();
    for projection in array(bundle, "projections")? {
        let projection_id = string(projection, "projection_id")?;
        if !projection_ids.insert(projection_id.to_owned()) {
            return Err(format!("duplicate projection ID {projection_id}"));
        }
        let graph_binding = projection.get("graph_binding").ok_or("projection missing graph_binding")?;
        if string(graph_binding, "graph_case_id")? != string(case, "case_id")?
            || string(graph_binding, "graph_sha256")? != graph_digest
        {
            return Err(format!("{projection_id}: graph binding mismatch"));
        }
        results.push(validate_projection(case, projection, graph_digest)?);
    }
    if results.is_empty() {
        return Err("projection bundle contains no projections".to_owned());
    }
    Ok(json!({
        "report_id": "MYCELIX-POLICY-PROJECTION-S0-REPORT",
        "schema_version": "1.0.0",
        "claim_ceiling": "projection_binding_and_ledger_invariant_validation_not_accounting_policy_correctness",
        "graph_sha256": graph_digest,
        "projection_count": results.len(),
        "all_graph_bindings_match": true,
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
                    "Validates policy projections against exact event-graph bytes and source lineage.\n\n\
                     --data-dir PATH  Economic-event artifact directory (default: sibling economic-event-core-v1)\n\
                     --output PATH    Write validation report JSON to PATH\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let graph_schema: Value = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(data_dir.join("accounting_policy_projection_v1.0.0.schema.json"))?;
    let bundle_schema: Value = read_json(data_dir.join("accounting_policy_projection_bundle_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("accounting_policy_projection_validation_report_v1.0.0.schema.json"))?;
    let graph_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
    let bundle_path = data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json");
    let graph_bytes = read_bytes(graph_path)?;
    let bundle_bytes = read_bytes(bundle_path)?;
    let graph: Value = serde_json::from_slice(&graph_bytes)?;
    let bundle: Value = serde_json::from_slice(&bundle_bytes)?;
    validate_instance("economic-event graph", &graph_schema, &graph)?;
    validate_instance("policy-projection bundle", &bundle_schema, &bundle)?;
    for projection in array(&bundle, "projections")? {
        validate_instance("accounting policy projection", &projection_schema, projection)?;
    }
    let graph_digest = hex::encode(Sha256::digest(&graph_bytes));
    let report = build_report(&graph, &graph_digest, &bundle)?;
    validate_instance("policy-projection report", &report_schema, &report)?;
    let bytes = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &bytes)?;
            println!("Wrote policy-projection validation report for {} projections.", report["projection_count"]);
        }
        None => println!("{}", String::from_utf8(bytes)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value, String) {
        let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        let graph_bytes = read_bytes(data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json")).unwrap();
        let graph: Value = serde_json::from_slice(&graph_bytes).unwrap();
        let bundle = read_json(data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json")).unwrap();
        let graph_schema = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json")).unwrap();
        let bundle_schema = read_json(data_dir.join("accounting_policy_projection_bundle_v1.0.0.schema.json")).unwrap();
        let digest = hex::encode(Sha256::digest(&graph_bytes));
        (graph, bundle, graph_schema, bundle_schema, digest)
    }

    #[test]
    fn two_review_only_policy_profiles_bind_to_the_same_exact_graph() {
        let (graph, bundle, graph_schema, bundle_schema, digest) = fixtures();
        validate_instance("graph", &graph_schema, &graph).unwrap();
        validate_instance("projection bundle", &bundle_schema, &bundle).unwrap();
        let report = build_report(&graph, &digest, &bundle).unwrap();
        assert_eq!(report["projection_count"], 2);
        assert_eq!(report["all_graph_bindings_match"], true);
        assert_eq!(report["projections"][0]["projection_state"], "needs_review");
        assert_eq!(report["projections"][0]["journal_line_count"], 0);
    }

    #[test]
    fn altered_graph_bytes_invalidate_projection_digest() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["graph_sha256"] = Value::String("a".repeat(64));
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn projection_with_wrong_graph_binding_is_rejected() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["graph_binding"]["graph_sha256"] = Value::String("b".repeat(64));
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn dangling_event_decision_reference_is_rejected() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["event_ref"] = Value::String("EV-MISSING".to_owned());
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn event_decision_cannot_cite_assertion_from_another_event() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["source_assertion_refs"] = json!(["AS-002-AMOUNT"]);
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn recognized_decision_requires_positive_measurement_and_currency_period() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let decision = &mut bundle["projections"][0]["event_decisions"][0];
        decision["decision_state"] = Value::String("recognized".to_owned());
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn not_projected_state_cannot_contain_journal_entries() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["projection_state"] = Value::String("not_projected".to_owned());
        bundle["projections"][0]["journal_entries"] = json!([{"journal_entry_ref":"J1","entry_kind":"recognition","lines":[]}]);
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn duplicate_policy_decision_refs_are_rejected() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        let decision = bundle["projections"][0]["event_decisions"][0].clone();
        bundle["projections"][0]["event_decisions"].as_array_mut().unwrap().push(decision);
        assert!(build_report(&graph, &digest, &bundle).is_err());
    }

    #[test]
    fn unresolved_items_require_live_event_or_assertion_refs() {
        let (graph, mut bundle, _, _, digest) = fixtures();
        bundle["projections"][0]["unresolved_items"][0]["event_refs"] = json!(["EV-MISSING"]);
        assert!(build_report(&graph, &bundle, &digest).is_err());
    }

    #[test]
    fn projection_example_policy_and_projector_digests_are_independent() {
        let (_, bundle, _, _, _) = fixtures();
        assert_ne!(
            bundle["projections"][0]["policy_binding"]["policy_sha256"],
            bundle["projections"][1]["policy_binding"]["policy_sha256"]
        );
        assert_ne!(
            bundle["projections"][0]["projector_binding"]["projector_sha256"],
            bundle["projections"][1]["projector_binding"]["projector_sha256"]
        );
        assert_eq!(
            bundle["projections"][0]["graph_binding"]["graph_sha256"],
            bundle["projections"][1]["graph_binding"]["graph_sha256"]
        );
    }
}
