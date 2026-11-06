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

fn is_null(value: &Value, key: &str) -> bool {
    value.get(key).is_none_or(Value::is_null)
}

fn find_by_ref<'a>(values: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn graph_case<'a>(graph: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(graph, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("unknown event graph case {case_id}"))
}

fn source_assertion<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "assertions")?, "assertion_ref", reference)
        .ok_or_else(|| format!("unknown graph assertion {reference}"))
}

fn economic_event<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "events")?, "event_ref", reference)
        .ok_or_else(|| format!("unknown graph event {reference}"))
}

fn reference_set(value: &Value, key: &str) -> Result<BTreeSet<String>, String> {
    array(value, key)?
        .iter()
        .map(|item| {
            item.as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| format!("{key} must contain strings"))
        })
        .collect()
}

fn assumption_by_ref<'a>(projection: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(projection, "assumptions")?, "assumption_ref", reference)
        .ok_or_else(|| format!("unknown projection assumption {reference}"))
}

fn source_is_available(case: &Value, source: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let source_ref = string(source, "source_ref")?;
    let superseded = array(case, "source_objects")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_source_ref") == Some(source_ref)
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    });
    Ok(string(source, "received_at")? <= frontier
        && optional_string(source, "integrity_state") == Some("synthetic_match")
        && !superseded)
}

fn assertion_is_available(case: &Value, assertion: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let source_ref = string(assertion, "source_ref")?;
    let source = find_by_ref(array(case, "source_objects")?, "source_ref", source_ref)
        .ok_or_else(|| format!("assertion references unknown source {source_ref}"))?;
    let period = case.get("period").ok_or("event graph case missing period")?;
    let valid_from = string(assertion, "valid_from")?;
    let valid_to = assertion.get("valid_to").and_then(Value::as_str);
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let overlap = valid_from <= period_end
        && valid_to.is_none_or(|end| end >= period_start);
    Ok(string(assertion, "recorded_at")? <= frontier
        && source_is_available(case, source)?
        && overlap
        && optional_string(assertion, "epistemic_state") == Some("asserted")
        && !is_null(assertion, "value"))
}

fn validate_projection(
    case: &Value,
    graph_digest: &str,
    bundle_case_id: &str,
    projection: &Value,
) -> Result<Value, String> {
    let projection_id = string(projection, "projection_id")?;
    let binding = projection.get("graph_binding").ok_or("projection missing graph_binding")?;
    if string(binding, "graph_case_id")? != bundle_case_id {
        return Err(format!("{projection_id}: projection graph_case_id differs from bundle"));
    }
    if string(binding, "graph_sha256")? != graph_digest {
        return Err(format!("{projection_id}: projection graph digest does not match exact candidate bytes"));
    }
    if string(projection, "entity_ref")? != string(case, "entity_scope_ref")? {
        return Err(format!("{projection_id}: projection entity differs from event graph scope"));
    }
    let period = projection.get("period").ok_or("projection period missing")?;
    let projection_start = string(period, "start")?;
    let projection_end = string(period, "end")?;
    if projection_start > projection_end {
        return Err(format!("{projection_id}: inverted projection period"));
    }
    let graph_period = case.get("period").ok_or("event graph period missing")?;
    if projection_start < string(graph_period, "start")?
        || projection_end > string(graph_period, "end")?
    {
        return Err(format!("{projection_id}: projection period is outside the event graph period"));
    }

    let mut assumption_refs = BTreeSet::new();
    let mut unresolved_critical_assumptions = 0usize;
    for assumption in array(projection, "assumptions")? {
        let reference = string(assumption, "assumption_ref")?;
        if !assumption_refs.insert(reference.to_owned()) {
            return Err(format!("{projection_id}: duplicate assumption {reference}"));
        }
        if assumption.get("decision_critical").and_then(Value::as_bool) == Some(true)
            && matches!(
                optional_string(assumption, "review_state"),
                Some("assumed" | "unknown" | "disputed")
            )
        {
            unresolved_critical_assumptions += 1;
        }
        for assertion_ref in array(assumption, "source_assertion_refs")? {
            source_assertion(
                case,
                assertion_ref.as_str().ok_or("assumption assertion ref must be string")?,
            )?;
        }
    }

    let mut decisions = BTreeMap::<String, &Value>::new();
    let mut event_decision_refs = BTreeSet::new();
    let mut recognized_count = 0usize;
    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let event_ref = string(decision, "event_ref")?;
        if decisions.contains_key(decision_ref) {
            return Err(format!("{projection_id}: duplicate policy decision {decision_ref}"));
        }
        if !event_decision_refs.insert(event_ref.to_owned()) {
            return Err(format!("{projection_id}: event {event_ref} has multiple decisions"));
        }
        let event = economic_event(case, event_ref)?;
        let event_assertions = reference_set(event, "assertion_refs")?;
        let decision_assertions = reference_set(decision, "source_assertion_refs")?;
        if decision_assertions.is_empty() || !decision_assertions.is_subset(&event_assertions) {
            return Err(format!("{projection_id}/{decision_ref}: decision source assertions are empty or not cited by its event"));
        }
        for assertion_ref in &decision_assertions {
            source_assertion(case, assertion_ref)?;
        }
        for assumption_ref in array(decision, "assumption_refs")? {
            let reference = assumption_ref.as_str().ok_or("decision assumption ref must be string")?;
            assumption_by_ref(projection, reference)?;
        }

        let amount_is_null = is_null(decision, "recognized_amount_minor");
        let currency_is_null = is_null(decision, "currency");
        if amount_is_null != currency_is_null {
            return Err(format!("{projection_id}/{decision_ref}: amount and currency must be both present or both null"));
        }
        match string(decision, "decision_state")? {
            "recognized" => {
                if amount_is_null || is_null(decision, "period_ref") {
                    return Err(format!("{projection_id}/{decision_ref}: recognized decision requires amount, currency and period"));
                }
                let amount = decision["recognized_amount_minor"]
                    .as_i64()
                    .ok_or("recognized_amount_minor must be integer minor units")?;
                if amount < 0 {
                    return Err(format!("{projection_id}/{decision_ref}: recognized amount cannot be negative"));
                }
                for assertion_ref in &decision_assertions {
                    let assertion = source_assertion(case, assertion_ref)?;
                    if !assertion_is_available(case, assertion)? {
                        return Err(format!("{projection_id}/{decision_ref}: recognized decision uses unavailable source assertion {assertion_ref}"));
                    }
                }
                recognized_count += 1;
            }
            _ if !amount_is_null => {
                return Err(format!("{projection_id}/{decision_ref}: non-recognized decision cannot carry a recognized amount"));
            }
            _ => {}
        }
        decisions.insert(decision_ref.to_owned(), decision);
    }

    let mut unresolved_refs = BTreeSet::new();
    let mut blocking_unresolved = 0usize;
    for item in array(projection, "unresolved_items")? {
        let item_ref = string(item, "item_ref")?;
        if !unresolved_refs.insert(item_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate unresolved item {item_ref}"));
        }
        if item.get("blocking").and_then(Value::as_bool) == Some(true) {
            blocking_unresolved += 1;
        }
        for event_ref in array(item, "event_refs")? {
            economic_event(case, event_ref.as_str().ok_or("unresolved event ref must be string")?)?;
        }
        for assertion_ref in array(item, "source_assertion_refs")? {
            source_assertion(case, assertion_ref.as_str().ok_or("unresolved assertion ref must be string")?)?;
        }
    }

    let mut journal_entry_refs = BTreeSet::new();
    let mut journal_line_refs = BTreeSet::new();
    let mut balanced_entries = 0usize;
    let mut debit_by_decision = BTreeMap::<String, i128>::new();
    let mut credit_by_decision = BTreeMap::<String, i128>::new();
    for entry in array(projection, "journal_entries")? {
        let entry_ref = string(entry, "journal_entry_ref")?;
        if !journal_entry_refs.insert(entry_ref.to_owned()) {
            return Err(format!("{projection_id}: duplicate journal entry {entry_ref}"));
        }
        let mut totals = BTreeMap::<(String, String, String), (i128, i128)>::new();
        for line in array(entry, "lines")? {
            let line_ref = string(line, "line_ref")?;
            if !journal_line_refs.insert(line_ref.to_owned()) {
                return Err(format!("{projection_id}: duplicate journal line {line_ref}"));
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            let decision = decisions
                .get(decision_ref)
                .copied()
                .ok_or_else(|| format!("{projection_id}/{line_ref}: unknown policy decision {decision_ref}"))?;
            if optional_string(decision, "decision_state") != Some("recognized") {
                return Err(format!("{projection_id}/{line_ref}: journal line references non-recognized decision"));
            }
            if string(line, "entity_ref")? != string(projection, "entity_ref")? {
                return Err(format!("{projection_id}/{line_ref}: journal entity differs from projection entity"));
            }
            let decision_event_ref = string(decision, "event_ref")?;
            let event_refs = reference_set(line, "event_refs")?;
            if event_refs.len() != 1 || !event_refs.contains(decision_event_ref) {
                return Err(format!("{projection_id}/{line_ref}: line must reference exactly the event bound to its policy decision"));
            }
            for event_ref in &event_refs {
                economic_event(case, event_ref)?;
            }
            let decision_assertions = reference_set(decision, "source_assertion_refs")?;
            let line_assertions = reference_set(line, "source_assertion_refs")?;
            if line_assertions.is_empty() || !line_assertions.is_subset(&decision_assertions) {
                return Err(format!("{projection_id}/{line_ref}: line assertions are not bound to its decision"));
            }
            for assertion_ref in &line_assertions {
                let assertion = source_assertion(case, assertion_ref)?;
                if !assertion_is_available(case, assertion)? {
                    return Err(format!("{projection_id}/{line_ref}: journal line cites assertion unavailable at graph knowledge frontier"));
                }
            }
            if string(line, "currency")? != string(decision, "currency")? {
                return Err(format!("{projection_id}/{line_ref}: line currency differs from policy decision"));
            }
            if string(line, "period_ref")? != string(decision, "period_ref")? {
                return Err(format!("{projection_id}/{line_ref}: line period differs from policy decision"));
            }
            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or("journal amount_minor must be an integer")? as i128;
            if amount <= 0 {
                return Err(format!("{projection_id}/{line_ref}: journal amount must be positive"));
            }
            let key = (
                string(line, "currency")?.to_owned(),
                string(line, "entity_ref")?.to_owned(),
                string(line, "period_ref")?.to_owned(),
            );
            let group = totals.entry(key).or_default();
            match string(line, "side")? {
                "debit" => {
                    group.0 += amount;
                    *debit_by_decision.entry(decision_ref.to_owned()).or_default() += amount;
                }
                "credit" => {
                    group.1 += amount;
                    *credit_by_decision.entry(decision_ref.to_owned()).or_default() += amount;
                }
                side => return Err(format!("{projection_id}/{line_ref}: invalid side {side}")),
            }
        }
        if totals.is_empty() || totals.values().any(|(debits, credits)| debits != credits) {
            return Err(format!("{projection_id}/{entry_ref}: journal entry is empty or unbalanced"));
        }
        balanced_entries += 1;
    }

    for (decision_ref, decision) in &decisions {
        if optional_string(decision, "decision_state") == Some("recognized") {
            let recognized_amount = decision["recognized_amount_minor"]
                .as_i64()
                .ok_or("recognized amount missing")? as i128;
            let debits = debit_by_decision.get(decision_ref).copied().unwrap_or(0);
            let credits = credit_by_decision.get(decision_ref).copied().unwrap_or(0);
            if debits != recognized_amount || credits != recognized_amount {
                return Err(format!("{projection_id}/{decision_ref}: recognized amount does not reconcile to linked journal debits and credits"));
            }
        }
    }

    match string(projection, "projection_state")? {
        "complete_for_review" if blocking_unresolved > 0 || unresolved_critical_assumptions > 0 => {
            return Err(format!("{projection_id}: complete_for_review has blocking unresolved items or critical assumptions"));
        }
        "needs_review" if blocking_unresolved == 0 && unresolved_critical_assumptions == 0 => {
            return Err(format!("{projection_id}: needs_review has no blocking unresolved item or critical assumption"));
        }
        "indeterminate" if array(projection, "unresolved_items")?.is_empty() => {
            return Err(format!("{projection_id}: indeterminate projection has no unresolved items"));
        }
        "not_projected" if !array(projection, "journal_entries")?.is_empty() => {
            return Err(format!("{projection_id}: not_projected cannot contain journal entries"));
        }
        "complete_for_review" | "needs_review" | "indeterminate" | "not_projected" => {}
        state => return Err(format!("{projection_id}: unknown projection state {state}")),
    }

    let policy = projection.get("policy_binding").ok_or("policy binding missing")?;
    let projector = projection.get("projector_binding").ok_or("projector binding missing")?;
    Ok(json!({
        "projection_id": projection_id,
        "policy_profile_ref": string(policy, "policy_profile_ref")?,
        "policy_sha256": string(policy, "policy_sha256")?,
        "projector_id": string(projector, "projector_id")?,
        "projector_sha256": string(projector, "projector_sha256")?,
        "projection_state": string(projection, "projection_state")?,
        "event_decision_count": decisions.len(),
        "recognized_decision_count": recognized_count,
        "journal_entry_count": journal_entry_refs.len(),
        "journal_line_count": journal_line_refs.len(),
        "balanced_entry_count": balanced_entries,
        "unresolved_item_count": array(projection, "unresolved_items")?.len(),
        "blocking_unresolved_count": blocking_unresolved,
        "assumption_count": array(projection, "assumptions")?.len(),
        "unresolved_critical_assumption_count": unresolved_critical_assumptions
    }))
}

fn build_report(
    graph: &Value,
    graph_digest: &str,
    bundle: &Value,
) -> Result<Value, String> {
    let graph_case_id = string(bundle, "graph_case_id")?;
    let case = graph_case(graph, graph_case_id)?;
    if string(bundle, "graph_sha256")? != graph_digest {
        return Err("projection bundle graph digest does not match exact graph bytes".to_owned());
    }
    let projections = array(bundle, "projections")?;
    if projections.len() < 2 {
        return Err("policy comparison bundle must contain at least two projections".to_owned());
    }
    let mut projection_ids = BTreeSet::new();
    let mut policy_hashes = BTreeSet::new();
    let mut projector_hashes = BTreeSet::new();
    let mut results = Vec::new();
    for projection in projections {
        let projection_id = string(projection, "projection_id")?;
        if !projection_ids.insert(projection_id.to_owned()) {
            return Err(format!("duplicate projection ID {projection_id}"));
        }
        let policy = projection.get("policy_binding").ok_or("policy binding missing")?;
        let projector = projection.get("projector_binding").ok_or("projector binding missing")?;
        policy_hashes.insert(string(policy, "policy_sha256")?.to_owned());
        projector_hashes.insert(string(projector, "projector_sha256")?.to_owned());
        results.push(validate_projection(case, graph_digest, graph_case_id, projection)?);
    }
    if policy_hashes.len() < 2 || projector_hashes.len() < 2 {
        return Err("review controls require distinct policy and projector digests over one graph".to_owned());
    }
    Ok(json!({
        "report_id": "MYCELIX-POLICY-PROJECTION-VALIDATION-S0",
        "schema_version": "1.0.0",
        "claim_ceiling": "graph_binding_provenance_and_journal_invariant_validation_not_accounting_policy_correctness",
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
                    "Validates accounting policy projection bundles against exact event-graph bytes.\n\n\
                     --data-dir PATH  Economic-event artifact directory (default: sibling economic-event-core-v1)\n\
                     --output PATH    Write validation report JSON to a file\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let graph_schema: Value = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let bundle_schema: Value = read_json(data_dir.join("accounting_policy_projection_bundle_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(data_dir.join("accounting_policy_projection_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("accounting_policy_projection_validation_report_v1.0.0.schema.json"))?;
    let graph_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
    let bundle_path = data_dir.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json");
    let graph_bytes = read_bytes(graph_path)?;
    let graph: Value = serde_json::from_slice(&graph_bytes)?;
    let bundle: Value = read_json(bundle_path)?;

    validate_instance("economic event graph", &graph_schema, &graph)?;
    validate_instance("policy projection bundle", &bundle_schema, &bundle)?;
    for projection in array(&bundle, "projections")? {
        validate_instance("policy projection", &projection_schema, projection)?;
    }
    let graph_digest = hex::encode(Sha256::digest(&graph_bytes));
    if string(&bundle, "graph_sha256")? != graph_digest {
        return Err("bundle digest does not match exact event-graph candidate bytes".into());
    }
    let report = build_report(&graph, &graph_digest, &bundle)?;
    validate_instance("policy projection validation report", &report_schema, &report)?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!("Wrote policy projection validation report for {} projections.", report["projection_count"]);
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        (
            read_json(root.join("fixtures/economic_event_graph_candidates_v1.0.0.json")).unwrap(),
            read_json(root.join("fixtures/accounting_policy_projection_review_controls_v1.0.0.json")).unwrap(),
            read_json(root.join("economic_event_graph_v1.0.0.schema.json")).unwrap(),
            read_json(root.join("accounting_policy_projection_bundle_v1.0.0.schema.json")).unwrap(),
        )
    }

    fn fixture_digest() -> String {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        hex::encode(Sha256::digest(
            fs::read(root.join("fixtures/economic_event_graph_candidates_v1.0.0.json")).unwrap(),
        ))
    }

    fn make_recognized_journal(projection: &mut Value, amount: i64) {
        projection["projection_state"] = Value::String("complete_for_review".to_owned());
        projection["unresolved_items"] = json!([]);
        projection["event_decisions"][0]["decision_state"] = Value::String("recognized".to_owned());
        projection["event_decisions"][0]["recognized_amount_minor"] = Value::from(amount);
        projection["event_decisions"][0]["currency"] = Value::String("USD".to_owned());
        projection["event_decisions"][0]["period_ref"] = Value::String("2026-10".to_owned());
        projection["journal_entries"] = json!([{
            "journal_entry_ref":"JE-TEST",
            "entry_kind":"recognition",
            "lines":[
                {"line_ref":"JL-TEST-DR","account_ref":"EXP","side":"debit","amount_minor":amount,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"},
                {"line_ref":"JL-TEST-CR","account_ref":"AP","side":"credit","amount_minor":amount,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"}
            ]
        }]);
    }

    #[test]
    fn two_policy_projections_bind_one_exact_graph_and_remain_review_only() {
        let (graph, bundle, graph_schema, bundle_schema) = fixtures();
        validate_instance("graph", &graph_schema, &graph).unwrap();
        validate_instance("projection bundle", &bundle_schema, &bundle).unwrap();
        let report = build_report(&graph, &fixture_digest(), &bundle).unwrap();
        assert_eq!(report["projection_count"], 2);
        assert_eq!(report["graph_sha256"], bundle["graph_sha256"]);
        assert_eq!(report["results"][0]["projection_state"], "needs_review");
        assert_eq!(report["results"][1]["projection_state"], "needs_review");
        assert_eq!(report["results"][0]["journal_entry_count"], 0);
        assert_eq!(report["results"][1]["journal_entry_count"], 0);
        assert_eq!(report["results"][0]["blocking_unresolved_count"], 1);
        assert_eq!(report["results"][1]["blocking_unresolved_count"], 1);
    }

    #[test]
    fn graph_digest_mismatch_is_rejected() {
        let (graph, mut bundle, _, _) = fixtures();
        bundle["graph_sha256"] = Value::String("0".repeat(64));
        assert!(build_report(&graph, &fixture_digest(), &bundle).is_err());
    }

    #[test]
    fn projection_cannot_reference_unknown_event() {
        let (graph, mut bundle, _, _) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["event_ref"] = Value::String("EV-MISSING".to_owned());
        assert!(build_report(&graph, &fixture_digest(), &bundle).is_err());
    }

    #[test]
    fn decision_source_assertions_must_belong_to_the_event() {
        let (graph, mut bundle, _, _) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["source_assertion_refs"] = json!(["AS-004-INVOICE-AMOUNT"]);
        assert!(validate_projection(
            &graph["cases"][0],
            &fixture_digest(),
            "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF",
            &bundle["projections"][0]
        ).is_err());
    }

    #[test]
    fn assumption_reference_must_resolve_in_projection_register() {
        let (graph, mut bundle, _, _) = fixtures();
        bundle["projections"][0]["event_decisions"][0]["assumption_refs"] = json!(["A-MISSING"]);
        assert!(validate_projection(
            &graph["cases"][0],
            &fixture_digest(),
            "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF",
            &bundle["projections"][0]
        ).is_err());
    }

    #[test]
    fn review_only_policy_decision_cannot_post_journal_lines() {
        let (graph, mut bundle, _, _) = fixtures();
        let projection = &mut bundle["projections"][0];
        projection["journal_entries"] = json!([{
            "journal_entry_ref":"JE-UNSAFE",
            "entry_kind":"recognition",
            "lines":[
                {"line_ref":"JL-UNSAFE-DR","account_ref":"EXP","side":"debit","amount_minor":100,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"},
                {"line_ref":"JL-UNSAFE-CR","account_ref":"AP","side":"credit","amount_minor":100,"currency":"USD","entity_ref":"ENTITY-ACME","period_ref":"2026-10","event_refs":["EV-001-INVOICE"],"source_assertion_refs":["AS-001-AMOUNT"],"policy_decision_ref":"SYN-PROJECTION-A-DEC-001"}
            ]
        }]);
        assert!(validate_projection(&graph["cases"][0], &fixture_digest(), "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", projection).is_err());
    }

    #[test]
    fn unbalanced_journal_entry_is_rejected() {
        let (graph, mut bundle, _, _) = fixtures();
        let projection = &mut bundle["projections"][0];
        make_recognized_journal(projection, 100);
        projection["journal_entries"][0]["lines"][1]["amount_minor"] = Value::from(90);
        assert!(validate_projection(&graph["cases"][0], &fixture_digest(), "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", projection).is_err());
    }

    #[test]
    fn recognized_amount_must_equal_linked_journal_debits_and_credits() {
        let (graph, mut bundle, _, _) = fixtures();
        let projection = &mut bundle["projections"][0];
        make_recognized_journal(projection, 100);
        projection["journal_entries"][0]["lines"][0]["amount_minor"] = Value::from(90);
        projection["journal_entries"][0]["lines"][1]["amount_minor"] = Value::from(90);
        assert!(validate_projection(&graph["cases"][0], &fixture_digest(), "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", projection).is_err());
    }

    #[test]
    fn recognized_decision_cannot_use_future_source_assertion() {
        let (mut graph, mut bundle, _, _) = fixtures();
        let projection = &mut bundle["projections"][0];
        make_recognized_journal(projection, 100);
        graph["cases"][0]["assertions"][0]["recorded_at"] = Value::String("2026-11-05T11:00:00Z".to_owned());
        let changed_graph_digest = "d".repeat(64);
        bundle["graph_sha256"] = Value::String(changed_graph_digest.clone());
        projection["graph_binding"]["graph_sha256"] = Value::String(changed_graph_digest.clone());
        assert!(validate_projection(&graph["cases"][0], &changed_graph_digest, "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", projection).is_err());
    }

    #[test]
    fn needs_review_requires_a_blocking_item_or_critical_assumption() {
        let (graph, mut bundle, _, _) = fixtures();
        let projection = &mut bundle["projections"][0];
        projection["unresolved_items"] = json!([]);
        assert!(validate_projection(&graph["cases"][0], &fixture_digest(), "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", projection).is_err());
    }

    #[test]
    fn projection_cannot_cross_graph_entity_scope() {
        let (graph, mut bundle, _, _) = fixtures();
        bundle["projections"][0]["entity_ref"] = Value::String("OTHER-ENTITY".to_owned());
        assert!(validate_projection(&graph["cases"][0], &fixture_digest(), "EVT-001-INVOICE-WITHOUT-SERVICE-PROOF", &bundle["projections"][0]).is_err());
    }
}
