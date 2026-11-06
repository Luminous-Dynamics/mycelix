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
        eprintln!("financial-evidence-drift-analyzer: {error}");
        std::process::exit(2);
    }
}

fn read_bytes(path: PathBuf) -> Result<Vec<u8>, Box<dyn Error>> {
    Ok(fs::read(path)?)
}

fn read_json(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&read_bytes(path)?)?)
}

fn digest(bytes: &[u8]) -> String {
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

fn find_ref<'a>(items: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    items
        .iter()
        .find(|item| optional_string(item, field) == Some(reference))
}

fn graph_case<'a>(graph: &'a Value, case_id: &str) -> Result<&'a Value, String> {
    array(graph, "cases")?
        .iter()
        .find(|case| optional_string(case, "case_id") == Some(case_id))
        .ok_or_else(|| format!("graph has no case {case_id}"))
}

fn parse_embedded_json(label: &str, request_case: &Value, field: &str) -> Result<(Vec<u8>, Value), String> {
    let raw = string(request_case, field)?.as_bytes().to_vec();
    let parsed = serde_json::from_slice(&raw)
        .map_err(|error| format!("{label}: embedded JSON parse failed: {error}"))?;
    Ok((raw, parsed))
}

fn maps_by_ref(items: &[Value], field: &str) -> Result<BTreeMap<String, Value>, String> {
    let mut result = BTreeMap::new();
    for item in items {
        let reference = string(item, field)?.to_owned();
        if result.insert(reference.clone(), item.clone()).is_some() {
            return Err(format!("duplicate {field} {reference}"));
        }
    }
    Ok(result)
}

fn changed_refs(
    baseline: &[Value],
    candidate: &[Value],
    field: &str,
    ignore_display_metadata: bool,
) -> Result<BTreeSet<String>, String> {
    let mut before = maps_by_ref(baseline, field)?;
    let mut after = maps_by_ref(candidate, field)?;
    let all = before
        .keys()
        .chain(after.keys())
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut changed = BTreeSet::new();

    for reference in all {
        let left = before.remove(&reference);
        let right = after.remove(&reference);
        let equal = match (left, right) {
            (None, None) => true,
            (Some(_), None) | (None, Some(_)) => false,
            (Some(mut left), Some(mut right)) => {
                if ignore_display_metadata {
                    if let Some(object) = left.as_object_mut() {
                        object.remove("display_metadata");
                    }
                    if let Some(object) = right.as_object_mut() {
                        object.remove("display_metadata");
                    }
                }
                left == right
            }
        };
        if !equal {
            changed.insert(reference);
        }
    }
    Ok(changed)
}

fn source_by_ref<'a>(case: &'a Value, source_ref: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "source_objects")?, "source_ref", source_ref)
        .ok_or_else(|| format!("unknown source {source_ref}"))
}

fn assertion_by_ref<'a>(case: &'a Value, assertion_ref: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "assertions")?, "assertion_ref", assertion_ref)
        .ok_or_else(|| format!("unknown assertion {assertion_ref}"))
}

fn event_by_ref<'a>(case: &'a Value, event_ref: &str) -> Result<&'a Value, String> {
    find_ref(array(case, "events")?, "event_ref", event_ref)
        .ok_or_else(|| format!("unknown event {event_ref}"))
}

fn source_is_superseded(case: &Value, source_ref: &str, frontier: &str) -> Result<bool, String> {
    Ok(array(case, "source_objects")?.iter().any(|source| {
        optional_string(source, "supersedes_source_ref") == Some(source_ref)
            && optional_string(source, "received_at").is_some_and(|received| received <= frontier)
    }))
}

fn assertion_available(case: &Value, assertion: &Value, frontier: &str) -> Result<bool, String> {
    let source = source_by_ref(case, string(assertion, "source_ref")?)?;
    let period = case.get("period").ok_or("graph case has no period")?;
    let start = string(period, "start")?;
    let end = string(period, "end")?;
    let valid_from = string(assertion, "valid_from")?;
    let valid_to = assertion.get("valid_to").and_then(Value::as_str);
    let valid_overlap = valid_from <= end && valid_to.is_none_or(|date| date >= start);
    Ok(string(assertion, "recorded_at")? <= frontier
        && string(source, "received_at")? <= frontier
        && optional_string(source, "integrity_state") == Some("synthetic_match")
        && optional_string(assertion, "epistemic_state") == Some("asserted")
        && !assertion.get("value").is_none_or(Value::is_null)
        && valid_overlap
        && !source_is_superseded(case, string(source, "source_ref")?, frontier)?)
}

fn assertion_refs_for_changed_sources(
    baseline: &Value,
    candidate: &Value,
    changed_source_refs: &BTreeSet<String>,
) -> Result<BTreeSet<String>, String> {
    let mut result = BTreeSet::new();
    for graph in [baseline, candidate] {
        for assertion in array(graph, "assertions")? {
            if optional_string(assertion, "source_ref")
                .is_some_and(|reference| changed_source_refs.contains(reference))
            {
                result.insert(string(assertion, "assertion_ref")?.to_owned());
            }
        }
    }
    Ok(result)
}

fn event_assertion_refs(case: &Value, event_ref: &str) -> Result<BTreeSet<String>, String> {
    let event = event_by_ref(case, event_ref)?;
    array(event, "assertion_refs")?
        .iter()
        .map(|reference| {
            reference
                .as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| "event assertion ref must be a string".to_owned())
        })
        .collect()
}

fn revision_family_unlinked(case: &Value) -> Result<bool, String> {
    let mut groups = BTreeMap::<(String, String, String), Vec<&Value>>::new();
    for source in array(case, "source_objects")? {
        let key = (
            string(source, "tenant_namespace")?.to_owned(),
            string(source, "external_id")?.to_owned(),
            string(source, "source_kind")?.to_owned(),
        );
        groups.entry(key).or_default().push(source);
    }

    for (_, mut group) in groups {
        let revisions = group
            .iter()
            .map(|source| optional_string(source, "revision").unwrap_or(""))
            .collect::<BTreeSet<_>>();
        if revisions.len() <= 1 {
            continue;
        }
        group.sort_by(|left, right| {
            optional_string(left, "received_at").cmp(&optional_string(right, "received_at"))
        });
        if optional_string(group[0], "supersedes_source_ref").is_some() {
            return Ok(true);
        }
        for index in 1..group.len() {
            let previous_ref = string(group[index - 1], "source_ref")?;
            if optional_string(group[index], "supersedes_source_ref") != Some(previous_ref) {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn cross_tenant_collision_preserved(case: &Value) -> Result<bool, String> {
    let sources = array(case, "source_objects")?;
    for left in sources {
        for right in sources {
            if optional_string(left, "source_ref") == optional_string(right, "source_ref") {
                continue;
            }
            if optional_string(left, "external_id") == optional_string(right, "external_id")
                && optional_string(left, "tenant_namespace")
                    != optional_string(right, "tenant_namespace")
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn new_evidence_available_at_candidate_frontier(
    baseline: &Value,
    candidate: &Value,
) -> Result<BTreeSet<String>, String> {
    let baseline_frontier = string(baseline, "knowledge_at")?;
    let candidate_frontier = string(candidate, "knowledge_at")?;
    let mut result = BTreeSet::new();

    if candidate_frontier <= baseline_frontier {
        return Ok(result);
    }

    for assertion in array(candidate, "assertions")? {
        let reference = string(assertion, "assertion_ref")?;
        let existed_before = find_ref(array(baseline, "assertions")?, "assertion_ref", reference);
        if existed_before.is_some() {
            continue;
        }
        if assertion_available(candidate, assertion, candidate_frontier)?
            && !assertion_available(candidate, assertion, baseline_frontier)?
        {
            result.insert(reference.to_owned());
        }
    }
    Ok(result)
}

fn decision_map<'a>(projection: &'a Value) -> Result<BTreeMap<String, &'a Value>, String> {
    let mut result = BTreeMap::new();
    for decision in array(projection, "event_decisions")? {
        let reference = string(decision, "policy_decision_ref")?.to_owned();
        if result.insert(reference.clone(), decision).is_some() {
            return Err(format!("duplicate policy decision {reference}"));
        }
    }
    Ok(result)
}

fn decision_refs_for_graph_change(
    graph: &Value,
    projection: &Value,
    changed_assertions: &BTreeSet<String>,
    changed_events: &BTreeSet<String>,
    changed_relations: &BTreeSet<String>,
    changed_sources: &BTreeSet<String>,
    all_due_to_policy_or_projector: bool,
    frontier_advanced: bool,
) -> Result<BTreeSet<String>, String> {
    let mut impacted = BTreeSet::new();
    let mut source_assertions = assertion_refs_for_changed_sources(graph, graph, changed_sources)?;
    source_assertions.extend(changed_assertions.iter().cloned());

    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let event_ref = string(decision, "event_ref")?;
        let decision_assertions = array(decision, "source_assertion_refs")?
            .iter()
            .filter_map(Value::as_str)
            .map(ToOwned::to_owned)
            .collect::<BTreeSet<String>>();

        let event_assertions = if find_ref(array(graph, "events")?, "event_ref", event_ref).is_some() {
            event_assertion_refs(graph, event_ref)?
                .into_iter()
                .collect::<BTreeSet<String>>()
        } else {
            BTreeSet::new()
        };

        let relation_touches_event = array(graph, "relations")?.iter().any(|relation| {
            changed_relations.contains(optional_string(relation, "relation_ref").unwrap_or(""))
                && (optional_string(relation, "from_ref") == Some(event_ref)
                    || optional_string(relation, "to_ref") == Some(event_ref))
        });

        if all_due_to_policy_or_projector
            || changed_events.contains(event_ref)
            || relation_touches_event
            || !decision_assertions.is_disjoint(&source_assertions)
            || !event_assertions.is_disjoint(&source_assertions)
            || (frontier_advanced
                && (!decision_assertions.is_disjoint(changed_assertions)
                    || !event_assertions.is_disjoint(changed_assertions)))
        {
            impacted.insert(decision_ref.to_owned());
        }
    }
    Ok(impacted)
}

fn projection_dependency_errors(graph: &Value, projection: &Value, case_id: &str) -> Result<Vec<String>, String> {
    let mut errors = Vec::new();
    if optional_string(projection, "projection_state") != Some("complete_for_review")
        && !array(projection, "journal_entries")?.is_empty()
    {
        errors.push("blocked_projection_contains_journal".to_owned());
    }

    let frontier = string(graph, "knowledge_at")?;
    let decisions = decision_map(projection)?;
    let mut decision_event_refs = BTreeSet::new();
    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let event_ref = string(decision, "event_ref")?;
        let event = match find_ref(array(graph, "events")?, "event_ref", event_ref) {
            Some(event) => event,
            None => {
                errors.push(format!("decision_event_missing:{decision_ref}:{event_ref}"));
                continue;
            }
        };
        decision_event_refs.insert(event_ref.to_owned());

        let event_assertions = event_assertion_refs(graph, event_ref)?;
        for assertion_reference in array(decision, "source_assertion_refs")? {
            let assertion_ref = assertion_reference
                .as_str()
                .ok_or("decision source assertion ref must be a string")?;
            if !event_assertions.contains(assertion_ref) {
                errors.push(format!("decision_assertion_not_attached:{decision_ref}:{assertion_ref}"));
                continue;
            }
            let assertion = assertion_by_ref(graph, assertion_ref)?;
            if optional_string(decision, "decision_state") == Some("recognized")
                && !assertion_available(graph, assertion, frontier)?
            {
                errors.push(format!("recognized_decision_uses_unavailable_assertion:{decision_ref}:{assertion_ref}"));
            }
        }
    }

    let mut entry_refs = BTreeSet::new();
    let mut line_refs = BTreeSet::new();
    let mut debit_credit = BTreeMap::<(String, String, String), (i128, i128)>::new();
    for entry in array(projection, "journal_entries")? {
        let entry_ref = string(entry, "journal_entry_ref")?;
        if !entry_refs.insert(entry_ref.to_owned()) {
            errors.push(format!("duplicate_journal_entry:{entry_ref}"));
        }
        for line in array(entry, "lines")? {
            let line_ref = string(line, "line_ref")?;
            if !line_refs.insert(line_ref.to_owned()) {
                errors.push(format!("duplicate_journal_line:{line_ref}"));
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            let decision = match decisions.get(decision_ref) {
                Some(decision) => *decision,
                None => {
                    errors.push(format!("journal_line_decision_missing:{line_ref}:{decision_ref}"));
                    continue;
                }
            };
            if optional_string(decision, "decision_state") != Some("recognized") {
                errors.push(format!("journal_line_not_recognized:{line_ref}"));
            }

            let decision_event = string(decision, "event_ref")?;
            let line_events = array(line, "event_refs")?
                .iter()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            if line_events.len() != 1 || !line_events.contains(decision_event) {
                errors.push(format!("journal_line_event_mismatch:{line_ref}"));
            }

            let decision_assertions = array(decision, "source_assertion_refs")?
                .iter()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            let line_assertions = array(line, "source_assertion_refs")?
                .iter()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            if line_assertions.is_empty() || !line_assertions.is_subset(&decision_assertions) {
                errors.push(format!("journal_line_assertion_mismatch:{line_ref}"));
            }

            if string(line, "entity_ref")? != string(projection, "entity_ref")?
                || string(line, "currency")? != string(decision, "currency")?
                || string(line, "period_ref")? != optional_string(decision, "period_ref").unwrap_or("")
            {
                errors.push(format!("journal_line_scope_mismatch:{line_ref}"));
            }

            let key = (
                string(line, "currency")?.to_owned(),
                string(line, "entity_ref")?.to_owned(),
                string(line, "period_ref")?.to_owned(),
            );
            let amount = line
                .get("amount_minor")
                .and_then(Value::as_i64)
                .ok_or("journal amount_minor must be integer")? as i128;
            let totals = debit_credit.entry(key).or_default();
            match string(line, "side")? {
                "debit" => totals.0 += amount,
                "credit" => totals.1 += amount,
                other => errors.push(format!("invalid_journal_side:{line_ref}:{other}")),
            }
        }
    }
    for (key, (debit, credit)) in debit_credit {
        if debit != credit {
            errors.push(format!(
                "journal_unbalanced:{}:{}:{}",
                key.0, key.1, key.2
            ));
        }
    }

    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        if optional_string(decision, "decision_state") == Some("recognized") {
            let has_line = array(projection, "journal_entries")?.iter().any(|entry| {
                array(entry, "lines").is_ok_and(|lines| {
                    lines.iter().any(|line| optional_string(line, "policy_decision_ref") == Some(decision_ref))
                })
            });
            if !has_line {
                errors.push(format!("recognized_decision_has_no_journal:{decision_ref}"));
            }
        }
    }

    for unresolved in array(projection, "unresolved_items")? {
        for reference in array(unresolved, "event_refs")? {
            let event_ref = reference.as_str().ok_or("unresolved event ref must be a string")?;
            if !decision_event_refs.contains(event_ref)
                && find_ref(array(graph, "events")?, "event_ref", event_ref).is_none()
            {
                errors.push(format!("unresolved_event_missing:{}", string(unresolved, "item_ref")?));
            }
        }
        for reference in array(unresolved, "source_assertion_refs")? {
            let assertion_ref = reference.as_str().ok_or("unresolved assertion ref must be a string")?;
            if find_ref(array(graph, "assertions")?, "assertion_ref", assertion_ref).is_none() {
                errors.push(format!("unresolved_assertion_missing:{}", string(unresolved, "item_ref")?));
            }
        }
        if optional_string(projection, "projection_state") == Some("complete_for_review")
            && unresolved.get("blocking").and_then(Value::as_bool) == Some(true)
        {
            errors.push(format!("complete_projection_has_blocking_item:{}", string(unresolved, "item_ref")?));
        }
    }

    if string(graph, "case_id")? != case_id {
        errors.push("graph_case_identity_mismatch".to_owned());
    }
    Ok(errors)
}

fn relation_affected_decision_refs(
    graph: &Value,
    changed_relation_refs: &BTreeSet<String>,
    projection: &Value,
) -> Result<BTreeSet<String>, String> {
    let mut impacted = BTreeSet::new();
    for relation in array(graph, "relations")? {
        if !changed_relation_refs.contains(string(relation, "relation_ref")?) {
            continue;
        }
        let endpoints = [
            optional_string(relation, "from_ref").unwrap_or(""),
            optional_string(relation, "to_ref").unwrap_or(""),
        ]
        .into_iter()
        .collect::<BTreeSet<_>>();
        let assertion_refs = array(relation, "source_assertion_refs")?
            .iter()
            .filter_map(Value::as_str)
            .collect::<BTreeSet<_>>();
        for decision in array(projection, "event_decisions")? {
            let event_ref = string(decision, "event_ref")?;
            let decision_assertions = array(decision, "source_assertion_refs")?
                .iter()
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            if endpoints.contains(event_ref)
                || !assertion_refs.is_disjoint(&decision_assertions)
            {
                impacted.insert(string(decision, "policy_decision_ref")?.to_owned());
            }
        }
    }
    Ok(impacted)
}

fn journal_impact_refs(
    baseline_projection: &Value,
    candidate_projection: &Value,
    impacted_decisions: &BTreeSet<String>,
    changed_events: &BTreeSet<String>,
    changed_assertions: &BTreeSet<String>,
) -> Result<(BTreeSet<String>, BTreeSet<String>), String> {
    let mut entries = BTreeSet::new();
    let mut lines = BTreeSet::new();
    for projection in [baseline_projection, candidate_projection] {
        for entry in array(projection, "journal_entries")? {
            let entry_ref = string(entry, "journal_entry_ref")?;
            let mut affected = false;
            for line in array(entry, "lines")? {
                let decision_ref = string(line, "policy_decision_ref")?;
                let event_refs = array(line, "event_refs")?
                    .iter()
                    .filter_map(Value::as_str)
                    .collect::<BTreeSet<_>>();
                let assertion_refs = array(line, "source_assertion_refs")?
                    .iter()
                    .filter_map(Value::as_str)
                    .collect::<BTreeSet<_>>();
                if impacted_decisions.contains(decision_ref)
                    || !event_refs.is_disjoint(changed_events)
                    || !assertion_refs.is_disjoint(changed_assertions)
                {
                    affected = true;
                    lines.insert(string(line, "line_ref")?.to_owned());
                }
            }
            if affected {
                entries.insert(entry_ref.to_owned());
            }
        }
    }
    Ok((entries, lines))
}

fn set_to_strings(values: &BTreeSet<String>) -> Vec<String> {
    values.iter().cloned().collect()
}

fn analyze_case(
    request_case: &Value,
    graph_schema: &Value,
    projection_schema: &Value,
) -> Result<Value, String> {
    let case_id = string(request_case, "case_id")?;
    let (baseline_graph_bytes, baseline_graph) =
        parse_embedded_json("baseline graph", request_case, "baseline_graph_json")?;
    let (candidate_graph_bytes, candidate_graph) =
        parse_embedded_json("candidate graph", request_case, "candidate_graph_json")?;
    let (baseline_projection_bytes, baseline_projection) =
        parse_embedded_json("baseline projection", request_case, "baseline_projection_json")?;
    let (candidate_projection_bytes, candidate_projection) =
        parse_embedded_json("candidate projection", request_case, "candidate_projection_json")?;

    validate_instance("baseline graph bundle", graph_schema, &baseline_graph)?;
    validate_instance("candidate graph bundle", graph_schema, &candidate_graph)?;
    validate_instance("baseline policy projection", projection_schema, &baseline_projection)?;
    validate_instance("candidate policy projection", projection_schema, &candidate_projection)?;

    let baseline_graph_case = graph_case(&baseline_graph, case_id)?;
    let candidate_graph_case = graph_case(&candidate_graph, case_id)?;
    let baseline_graph_hash = digest(&baseline_graph_bytes);
    let candidate_graph_hash = digest(&candidate_graph_bytes);
    let baseline_projection_hash = digest(&baseline_projection_bytes);
    let candidate_projection_hash = digest(&candidate_projection_bytes);

    let mut findings = BTreeSet::<String>::new();
    let mut projection_invalid = false;
    for (label, graph_hash, graph_case_value, projection) in [
        ("baseline", &baseline_graph_hash, baseline_graph_case, &baseline_projection),
        ("candidate", &candidate_graph_hash, candidate_graph_case, &candidate_projection),
    ] {
        if optional_string(projection.get("graph_binding").unwrap_or(&Value::Null), "graph_sha256")
            != Some(graph_hash.as_str())
            || optional_string(projection.get("graph_binding").unwrap_or(&Value::Null), "graph_case_id")
                != Some(case_id)
        {
            projection_invalid = true;
            findings.insert("projection_binding_invalid".to_owned());
            let _ = label;
        }
        let dependency_errors = projection_dependency_errors(graph_case_value, projection, case_id)?;
        if !dependency_errors.is_empty() {
            projection_invalid = true;
            findings.insert("projection_dependency_invalid".to_owned());
        }
    }

    let baseline_frontier = string(baseline_graph_case, "knowledge_at")?;
    let candidate_frontier = string(candidate_graph_case, "knowledge_at")?;
    let knowledge_frontier_changed = baseline_frontier != candidate_frontier;

    let changed_source_refs = changed_refs(
        array(baseline_graph_case, "source_objects")?,
        array(candidate_graph_case, "source_objects")?,
        "source_ref",
        false,
    )?;
    let semantic_source_refs = changed_refs(
        array(baseline_graph_case, "source_objects")?,
        array(candidate_graph_case, "source_objects")?,
        "source_ref",
        true,
    )?;
    let changed_assertion_refs = changed_refs(
        array(baseline_graph_case, "assertions")?,
        array(candidate_graph_case, "assertions")?,
        "assertion_ref",
        false,
    )?;
    let changed_event_refs = changed_refs(
        array(baseline_graph_case, "events")?,
        array(candidate_graph_case, "events")?,
        "event_ref",
        false,
    )?;
    let changed_relation_refs = changed_refs(
        array(baseline_graph_case, "relations")?,
        array(candidate_graph_case, "relations")?,
        "relation_ref",
        false,
    )?;

    if !semantic_source_refs.is_empty() {
        findings.insert("semantic_source_change".to_owned());
    } else if !changed_source_refs.is_empty() {
        findings.insert("nonsemantic_source_metadata_only".to_owned());
    }
    if !changed_assertion_refs.is_empty() {
        findings.insert("assertion_changed".to_owned());
    }
    if !changed_event_refs.is_empty() {
        findings.insert("event_changed".to_owned());
    }
    if !changed_relation_refs.is_empty() {
        findings.insert("relation_changed".to_owned());
    }

    let mut affected_assertions = changed_assertion_refs.clone();
    affected_assertions.extend(assertion_refs_for_changed_sources(
        baseline_graph_case,
        candidate_graph_case,
        &changed_source_refs,
    )?);

    let mut integrity_degraded = false;
    let mut assertion_review_required = false;
    for assertion in array(candidate_graph_case, "assertions")? {
        let reference = string(assertion, "assertion_ref")?;
        let before = find_ref(array(baseline_graph_case, "assertions")?, "assertion_ref", reference);
        let changed = changed_assertion_refs.contains(reference)
            || changed_source_refs.contains(optional_string(assertion, "source_ref").unwrap_or(""));
        if !changed {
            continue;
        }
        if matches!(
            optional_string(assertion, "epistemic_state"),
            Some("disputed" | "invalidated" | "unknown")
        ) {
            assertion_review_required = true;
            findings.insert("assertion_invalidated_or_disputed".to_owned());
        }
        let source = source_by_ref(candidate_graph_case, string(assertion, "source_ref")?)?;
        if optional_string(source, "integrity_state") != Some("synthetic_match") {
            integrity_degraded = true;
            findings.insert("integrity_state_degraded".to_owned());
        }
        if before.is_none()
            && string(assertion, "recorded_at")? > baseline_frontier
            && string(assertion, "recorded_at")? <= candidate_frontier
            && assertion_available(candidate_graph_case, assertion, candidate_frontier)?
        {
            findings.insert("late_evidence_available_at_new_frontier".to_owned());
        }
    }

    let policy_binding_changed = optional_string(
        baseline_projection.get("policy_binding").unwrap_or(&Value::Null),
        "policy_sha256",
    ) != optional_string(
        candidate_projection.get("policy_binding").unwrap_or(&Value::Null),
        "policy_sha256",
    );
    let projector_binding_changed = optional_string(
        baseline_projection.get("projector_binding").unwrap_or(&Value::Null),
        "projector_sha256",
    ) != optional_string(
        candidate_projection.get("projector_binding").unwrap_or(&Value::Null),
        "projector_sha256",
    );
    if policy_binding_changed {
        findings.insert("policy_digest_changed".to_owned());
    }
    if projector_binding_changed {
        findings.insert("projector_digest_changed".to_owned());
    }

    if revision_family_unlinked(candidate_graph_case)? {
        findings.insert("unlinked_source_revision_detected".to_owned());
    }
    if cross_tenant_collision_preserved(candidate_graph_case)? {
        findings.insert("cross_tenant_identity_preserved".to_owned());
    }

    let late_evidence = new_evidence_available_at_candidate_frontier(
        baseline_graph_case,
        candidate_graph_case,
    )?;
    if !late_evidence.is_empty() {
        findings.insert("late_evidence_available_at_new_frontier".to_owned());
        affected_assertions.extend(late_evidence);
    }

    let all_due_to_policy_or_projector = policy_binding_changed || projector_binding_changed;
    let mut impacted_decisions = decision_refs_for_graph_change(
        baseline_graph_case,
        &baseline_projection,
        &affected_assertions,
        &changed_event_refs,
        &changed_relation_refs,
        &semantic_source_refs,
        all_due_to_policy_or_projector,
        knowledge_frontier_changed,
    )?;
    impacted_decisions.extend(decision_refs_for_graph_change(
        candidate_graph_case,
        &candidate_projection,
        &affected_assertions,
        &changed_event_refs,
        &changed_relation_refs,
        &semantic_source_refs,
        all_due_to_policy_or_projector,
        knowledge_frontier_changed,
    )?);
    impacted_decisions.extend(relation_affected_decision_refs(
        baseline_graph_case,
        &changed_relation_refs,
        &baseline_projection,
    )?);
    impacted_decisions.extend(relation_affected_decision_refs(
        candidate_graph_case,
        &changed_relation_refs,
        &candidate_projection,
    )?);

    if revision_family_unlinked(candidate_graph_case)? {
        for decision in array(&baseline_projection, "event_decisions")?
            .iter()
            .chain(array(&candidate_projection, "event_decisions")?.iter())
        {
            let event = find_ref(
                array(candidate_graph_case, "events")?,
                "event_ref",
                optional_string(decision, "event_ref").unwrap_or(""),
            )
            .or_else(|| {
                find_ref(
                    array(baseline_graph_case, "events").unwrap_or(&[]),
                    "event_ref",
                    optional_string(decision, "event_ref").unwrap_or(""),
                )
            });
            let Some(event) = event else {
                continue;
            };
            let source_refs = array(event, "assertion_refs")?
                .iter()
                .filter_map(Value::as_str)
                .filter_map(|reference| {
                    find_ref(
                        array(candidate_graph_case, "assertions").unwrap_or(&[]),
                        "assertion_ref",
                        reference,
                    )
                    .or_else(|| {
                        find_ref(
                            array(baseline_graph_case, "assertions").unwrap_or(&[]),
                            "assertion_ref",
                            reference,
                        )
                    })
                })
                .filter_map(|assertion| optional_string(assertion, "source_ref"))
                .collect::<BTreeSet<_>>();
            let unlinked_in_same_family = array(candidate_graph_case, "source_objects")?
                .iter()
                .any(|source| {
                    source_refs.contains(optional_string(source, "source_ref").unwrap_or(""))
                        && array(candidate_graph_case, "source_objects")
                            .unwrap_or(&[])
                            .iter()
                            .any(|other| {
                                optional_string(other, "tenant_namespace")
                                    == optional_string(source, "tenant_namespace")
                                    && optional_string(other, "external_id")
                                        == optional_string(source, "external_id")
                                    && optional_string(other, "source_kind")
                                        == optional_string(source, "source_kind")
                                    && optional_string(other, "revision")
                                        != optional_string(source, "revision")
                            })
                });
            if unlinked_in_same_family {
                impacted_decisions.insert(string(decision, "policy_decision_ref")?.to_owned());
            }
        }
    }

    let (impacted_entries, impacted_lines) = journal_impact_refs(
        &baseline_projection,
        &candidate_projection,
        &impacted_decisions,
        &changed_event_refs,
        &affected_assertions,
    )?;

    let mut actual_impact_refs = BTreeSet::new();
    for reference in &impacted_decisions {
        actual_impact_refs.insert(format!("decision:{reference}"));
    }
    for reference in &impacted_entries {
        actual_impact_refs.insert(format!("journal_entry:{reference}"));
    }
    for reference in &impacted_lines {
        actual_impact_refs.insert(format!("journal_line:{reference}"));
    }
    let caller_reported = array(request_case, "caller_reported_affected_refs")?
        .iter()
        .map(|value| {
            value
                .as_str()
                .map(ToOwned::to_owned)
                .ok_or_else(|| "caller impact ref must be a string".to_owned())
        })
        .collect::<Result<BTreeSet<_>, _>>()?;
    let caller_impact_claim_matches = caller_reported == actual_impact_refs;
    if !caller_impact_claim_matches {
        if actual_impact_refs.is_superset(&caller_reported) {
            findings.insert("caller_impact_set_underreported".to_owned());
        } else {
            findings.insert("caller_impact_set_overreported".to_owned());
        }
    }

    let unlinked = revision_family_unlinked(candidate_graph_case)?;
    let has_new_late = !late_evidence.is_empty();
    let no_dependent_impact = impacted_decisions.is_empty()
        && impacted_entries.is_empty()
        && impacted_lines.is_empty()
        && !policy_binding_changed
        && !projector_binding_changed
        && !has_new_late;

    let classification = if projection_invalid {
        "projection_invalid"
    } else if assertion_review_required || integrity_degraded || unlinked {
        "review_required"
    } else if has_new_late && knowledge_frontier_changed {
        "historical_replay_required"
    } else if no_dependent_impact && changed_source_refs.is_empty()
        && changed_assertion_refs.is_empty()
        && changed_event_refs.is_empty()
        && changed_relation_refs.is_empty()
    {
        "no_material_impact"
    } else if no_dependent_impact {
        "no_material_impact"
    } else {
        "reproject_required"
    };

    Ok(json!({
        "case_id": case_id,
        "classification": classification,
        "baseline_graph_sha256": baseline_graph_hash,
        "candidate_graph_sha256": candidate_graph_hash,
        "baseline_projection_sha256": baseline_projection_hash,
        "candidate_projection_sha256": candidate_projection_hash,
        "changed_source_refs": set_to_strings(&changed_source_refs),
        "changed_assertion_refs": set_to_strings(&changed_assertion_refs),
        "changed_event_refs": set_to_strings(&changed_event_refs),
        "changed_relation_refs": set_to_strings(&changed_relation_refs),
        "impacted_decision_refs": set_to_strings(&impacted_decisions),
        "impacted_journal_entry_refs": set_to_strings(&impacted_entries),
        "impacted_journal_line_refs": set_to_strings(&impacted_lines),
        "policy_binding_changed": policy_binding_changed,
        "projector_binding_changed": projector_binding_changed,
        "knowledge_frontier_changed": knowledge_frontier_changed,
        "caller_impact_claim_matches": caller_impact_claim_matches,
        "finding_codes": findings.into_iter().collect::<Vec<_>>()
    }))
}

fn build_report(
    request: &Value,
    request_bytes: &[u8],
    request_schema: &Value,
    graph_schema: &Value,
    projection_schema: &Value,
    report_schema: &Value,
) -> Result<Value, String> {
    validate_instance("evidence drift request", request_schema, request)?;
    let mut case_ids = BTreeSet::new();
    let mut results = Vec::new();
    for request_case in array(request, "cases")? {
        let case_id = string(request_case, "case_id")?;
        if !case_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate drift case ID {case_id}"));
        }
        results.push(analyze_case(request_case, graph_schema, projection_schema)?);
    }

    let mut totals = BTreeMap::<String, usize>::new();
    for result in &results {
        let key = format!("{}_count", string(result, "classification")?);
        *totals.entry(key).or_default() += 1;
        if result["caller_impact_claim_matches"] == false {
            *totals.entry("underreported_impact_count".to_owned()).or_default() += 1;
        }
        for (field, total_key) in [
            ("changed_source_refs", "changed_source_count"),
            ("changed_assertion_refs", "changed_assertion_count"),
            ("changed_event_refs", "changed_event_count"),
            ("changed_relation_refs", "changed_relation_count"),
            ("impacted_decision_refs", "impacted_decision_count"),
            ("impacted_journal_entry_refs", "impacted_journal_entry_count"),
            ("impacted_journal_line_refs", "impacted_journal_line_count"),
        ] {
            *totals.entry(total_key.to_owned()).or_default() +=
                array(result, field)?.len();
        }
    }
    for key in [
        "no_material_impact_count",
        "review_required_count",
        "reproject_required_count",
        "historical_replay_required_count",
        "projection_invalid_count",
        "underreported_impact_count",
        "changed_source_count",
        "changed_assertion_count",
        "changed_event_count",
        "changed_relation_count",
        "impacted_decision_count",
        "impacted_journal_entry_count",
        "impacted_journal_line_count",
    ] {
        totals.entry(key.to_owned()).or_default();
    }

    let report = json!({
        "report_id": "MYCELIX-FINANCIAL-EVIDENCE-DRIFT-S0-REPORT",
        "schema_version": "1.0.0",
        "claim_ceiling": "deterministic_dependency_impact_report_not_accounting_reauthorization",
        "case_count": results.len(),
        "request_sha256": digest(request_bytes),
        "totals": totals,
        "case_results": results
    });
    validate_instance("evidence drift report", report_schema, &report)?;
    Ok(report)
}

fn run() -> Result<(), Box<dyn Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../evidence-drift-v1");
    let mut data_dir = root;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a directory")?),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--help" | "-h" => {
                println!(
                    "Analyzes deterministic financial evidence drift and dependent projection replay.\n\n\
                     --data-dir PATH  Evidence-drift artifact directory (default: sibling evidence-drift-v1)\n\
                     --output PATH    Write report JSON to PATH instead of stdout\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let request_schema: Value = read_json(data_dir.join("financial_evidence_drift_request_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("financial_evidence_drift_report_v1.0.0.schema.json"))?;
    let graph_dir = data_dir.join("../economic-event-core-v1");
    let graph_schema: Value = read_json(graph_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(graph_dir.join("accounting_policy_projection_v1.0.0.schema.json"))?;
    let request_path = data_dir.join("fixtures/financial_evidence_drift_candidates_v1.0.0.json");
    let request_bytes = read_bytes(request_path)?;
    let request: Value = serde_json::from_slice(&request_bytes)?;
    let report = build_report(
        &request,
        &request_bytes,
        &request_schema,
        &graph_schema,
        &projection_schema,
        &report_schema,
    )?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!("Wrote financial evidence drift report for {} cases.", report["case_count"]);
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value, Value, Value) {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let drift_dir = manifest.join("../evidence-drift-v1");
        let graph_dir = manifest.join("../economic-event-core-v1");
        (
            read_json(drift_dir.join("financial_evidence_drift_request_v1.0.0.schema.json")).unwrap(),
            read_json(drift_dir.join("financial_evidence_drift_report_v1.0.0.schema.json")).unwrap(),
            read_json(graph_dir.join("economic_event_graph_v1.0.0.schema.json")).unwrap(),
            read_json(graph_dir.join("accounting_policy_projection_v1.0.0.schema.json")).unwrap(),
            read_json(drift_dir.join("fixtures/financial_evidence_drift_candidates_v1.0.0.json")).unwrap(),
        )
    }

    fn result<'a>(report: &'a Value, case_id: &str) -> &'a Value {
        array(report, "case_results").unwrap().iter()
            .find(|item| optional_string(item, "case_id") == Some(case_id)).unwrap()
    }

    fn analyze_fixture(request_schema: &Value, report_schema: &Value, graph_schema: &Value, projection_schema: &Value, fixture: &Value) -> Value {
        build_report(fixture, b"synthetic fixture bytes", request_schema, graph_schema, projection_schema, report_schema).unwrap()
    }

    #[test]
    fn all_ten_synthetic_drift_cases_validate_and_classify() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        assert_eq!(report["case_count"], 10);
        assert_eq!(result(&report, "DRIFT-001-AMOUNT-CORRECTION")["classification"], "reproject_required");
        assert_eq!(result(&report, "DRIFT-002-METADATA-ONLY")["classification"], "no_material_impact");
        assert_eq!(result(&report, "DRIFT-003-ASSERTION-DISPUTED")["classification"], "review_required");
        assert_eq!(result(&report, "DRIFT-004-LATE-EVIDENCE")["classification"], "historical_replay_required");
        assert_eq!(result(&report, "DRIFT-005-POLICY-DIGEST-CHANGED")["classification"], "reproject_required");
        assert_eq!(result(&report, "DRIFT-006-PROJECTOR-DIGEST-CHANGED")["classification"], "reproject_required");
        assert_eq!(result(&report, "DRIFT-007-EVENT-RELATION-CHANGED")["classification"], "reproject_required");
        assert_eq!(result(&report, "DRIFT-008-CROSS-TENANT-ID-COLLISION")["classification"], "no_material_impact");
        assert_eq!(result(&report, "DRIFT-009-UNLINKED-REVISION")["classification"], "review_required");
        assert_eq!(result(&report, "DRIFT-010-UNDERREPORTED-IMPACT-SET")["caller_impact_claim_matches"], false);
    }

    #[test]
    fn benign_display_metadata_change_does_not_invalidate_projection() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-002-METADATA-ONLY");
        assert_eq!(result["classification"], "no_material_impact");
        assert_eq!(result["impacted_decision_refs"], json!([]));
        assert!(array(result, "finding_codes").unwrap().iter().any(|x| x == "nonsemantic_source_metadata_only"));
    }

    #[test]
    fn amount_correction_invalidates_exact_dependent_journal_lines() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-001-AMOUNT-CORRECTION");
        assert_eq!(result["impacted_decision_refs"], json!(["DEC-1"]));
        assert_eq!(result["impacted_journal_entry_refs"], json!(["JE-1"]));
        assert_eq!(result["impacted_journal_line_refs"], json!(["LINE-CR", "LINE-DR"]));
    }

    #[test]
    fn late_evidence_requires_historical_frontier_replay() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-004-LATE-EVIDENCE");
        assert_eq!(result["classification"], "historical_replay_required");
        assert_eq!(result["knowledge_frontier_changed"], true);
        assert!(array(result, "finding_codes").unwrap().iter().any(|x| x == "late_evidence_available_at_new_frontier"));
    }

    #[test]
    fn policy_digest_change_invalidates_all_dependent_decisions() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-005-POLICY-DIGEST-CHANGED");
        assert_eq!(result["policy_binding_changed"], true);
        assert_eq!(result["impacted_decision_refs"], json!(["DEC-1"]));
        assert_eq!(result["classification"], "reproject_required");
    }

    #[test]
    fn projector_digest_change_invalidates_all_dependent_decisions() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-006-PROJECTOR-DIGEST-CHANGED");
        assert_eq!(result["projector_binding_changed"], true);
        assert_eq!(result["impacted_decision_refs"], json!(["DEC-1"]));
        assert_eq!(result["classification"], "reproject_required");
    }

    #[test]
    fn cross_tenant_same_external_id_does_not_merge_dependencies() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-008-CROSS-TENANT-ID-COLLISION");
        assert_eq!(result["classification"], "no_material_impact");
        assert_eq!(result["impacted_decision_refs"], json!([]));
        assert!(array(result, "finding_codes").unwrap().iter().any(|x| x == "cross_tenant_identity_preserved"));
    }

    #[test]
    fn unlinked_revision_requires_review_and_invalidates_dependent_decisions() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-009-UNLINKED-REVISION");
        assert_eq!(result["classification"], "review_required");
        assert_eq!(result["impacted_decision_refs"], json!(["DEC-1"]));
        assert!(array(result, "finding_codes").unwrap().iter().any(|x| x == "unlinked_source_revision_detected"));
    }

    #[test]
    fn caller_underreporting_is_detected_not_used_to_limit_closure() {
        let (request_schema, report_schema, graph_schema, projection_schema, fixture) = fixtures();
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        let result = result(&report, "DRIFT-010-UNDERREPORTED-IMPACT-SET");
        assert_eq!(result["caller_impact_claim_matches"], false);
        assert_eq!(result["impacted_journal_line_refs"], json!(["LINE-CR", "LINE-DR"]));
        assert!(array(result, "finding_codes").unwrap().iter().any(|x| x == "caller_impact_set_underreported"));
    }

    #[test]
    fn bad_candidate_graph_binding_is_projection_invalid() {
        let (request_schema, report_schema, graph_schema, projection_schema, mut fixture) = fixtures();
        let row = fixture["cases"].as_array_mut().unwrap().iter_mut()
            .find(|item| optional_string(item, "case_id") == Some("DRIFT-001-AMOUNT-CORRECTION")).unwrap();
        let mut projection: Value = serde_json::from_str(string(row, "candidate_projection_json").unwrap()).unwrap();
        projection["graph_binding"]["graph_sha256"] = json!("b".repeat(64));
        row["candidate_projection_json"] = json!(serde_json::to_string(&projection).unwrap());
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        assert_eq!(result(&report, "DRIFT-001-AMOUNT-CORRECTION")["classification"], "projection_invalid");
    }

    #[test]
    fn unavailable_evidence_cannot_remain_as_recognized_projection_support() {
        let (request_schema, report_schema, graph_schema, projection_schema, mut fixture) = fixtures();
        let row = fixture["cases"].as_array_mut().unwrap().iter_mut()
            .find(|item| optional_string(item, "case_id") == Some("DRIFT-003-ASSERTION-DISPUTED")).unwrap();
        let mut projection: Value = serde_json::from_str(string(row, "candidate_projection_json").unwrap()).unwrap();
        projection["projection_state"] = json!("complete_for_review");
        projection["event_decisions"][0]["decision_state"] = json!("recognized");
        projection["event_decisions"][0]["recognized_amount_minor"] = json!(125000);
        projection["event_decisions"][0]["currency"] = json!("USD");
        projection["event_decisions"][0]["period_ref"] = json!("2026-10");
        projection["journal_entries"] = serde_json::from_str(string(row, "baseline_projection_json").unwrap()).unwrap()["journal_entries"].clone();
        row["candidate_projection_json"] = json!(serde_json::to_string(&projection).unwrap());
        let report = analyze_fixture(&request_schema, &report_schema, &graph_schema, &projection_schema, &fixture);
        assert_eq!(result(&report, "DRIFT-003-ASSERTION-DISPUTED")["classification"], "projection_invalid");
    }
}
