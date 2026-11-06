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

type DependencyMap = BTreeMap<String, BTreeSet<String>>;

fn main() {
    if let Err(error) = run() {
        eprintln!("financial-evidence-drift-s0: {error}");
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

fn tenant_root(namespace: &str) -> &str {
    namespace.split('/').next().unwrap_or(namespace)
}

fn canonical_json(value: &Value) -> Result<Vec<u8>, String> {
    serde_json::to_vec(value).map_err(|error| format!("canonical JSON serialization failed: {error}"))
}

fn canonical_sha256(value: &Value) -> Result<String, String> {
    Ok(hex::encode(Sha256::digest(canonical_json(value)?)))
}

fn add_dependency(dependencies: &mut DependencyMap, child: String, parent: String) {
    dependencies.entry(child).or_default().insert(parent);
}

fn node(kind: &str, reference: &str) -> String {
    format!("{kind}:{reference}")
}

fn values_by_ref<'a>(
    values: &'a [Value],
    id_field: &str,
) -> Result<BTreeMap<String, &'a Value>, String> {
    let mut mapped = BTreeMap::new();
    for value in values {
        let reference = value
            .get(id_field)
            .and_then(Value::as_str)
            .ok_or_else(|| format!("missing {id_field}"))?;
        if mapped.insert(reference.to_owned(), value).is_some() {
            return Err(format!("duplicate {id_field}: {reference}"));
        }
    }
    Ok(mapped)
}

fn changed_refs(
    baseline: &[Value],
    candidate: &[Value],
    id_field: &str,
    kind: &str,
) -> Result<BTreeSet<String>, String> {
    let before = values_by_ref(baseline, id_field)?;
    let after = values_by_ref(candidate, id_field)?;
    let all = before
        .keys()
        .chain(after.keys())
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut changed = BTreeSet::new();
    for reference in all {
        let differs = match (before.get(&reference), after.get(&reference)) {
            (Some(left), Some(right)) => left != right,
            (None, None) => false,
            _ => true,
        };
        if differs {
            changed.insert(node(kind, &reference));
        }
    }
    Ok(changed)
}

fn case_arrays<'a>(case: &'a Value, field: &str) -> Result<&'a [Value], String> {
    array(case, field)
}

fn collect_dependencies(
    case: &Value,
    projection: &Value,
    statement_lines: &[Value],
    dependencies: &mut DependencyMap,
) -> Result<(), String> {
    // Assertion -> source; event -> assertions; relation -> endpoints.
    let assertions = values_by_ref(case_arrays(case, "assertions")?, "assertion_ref")?;
    let sources = values_by_ref(case_arrays(case, "source_objects")?, "source_ref")?;
    let events = values_by_ref(case_arrays(case, "events")?, "event_ref")?;

    for (assertion_ref, assertion) in &assertions {
        let source_ref = string(assertion, "source_ref")?;
        if !sources.contains_key(source_ref) {
            return Err(format!("assertion {assertion_ref} references missing source {source_ref}"));
        }
        add_dependency(
            dependencies,
            node("assertion", assertion_ref),
            node("source", source_ref),
        );
    }

    for (event_ref, event) in &events {
        for assertion in array(event, "assertion_refs")? {
            let assertion_ref = assertion
                .as_str()
                .ok_or("event assertion_refs must contain strings")?;
            if !assertions.contains_key(assertion_ref) {
                return Err(format!("event {event_ref} references missing assertion {assertion_ref}"));
            }
            add_dependency(
                dependencies,
                node("event", event_ref),
                node("assertion", assertion_ref),
            );
        }
    }

    for relation in case_arrays(case, "relations")? {
        let relation_ref = string(relation, "relation_ref")?;
        let from_kind = string(relation, "from_kind")?;
        let from_ref = string(relation, "from_ref")?;
        let to_kind = string(relation, "to_kind")?;
        let to_ref = string(relation, "to_ref")?;
        let from_node = node(from_kind, from_ref);
        let to_node = node(to_kind, to_ref);
        add_dependency(
            dependencies,
            node("relation", relation_ref),
            from_node.clone(),
        );
        add_dependency(
            dependencies,
            node("relation", relation_ref),
            to_node.clone(),
        );
        // End-point events depend on the relation semantics as well.
        if from_kind == "event" {
            add_dependency(dependencies, from_node, node("relation", relation_ref));
        }
        if to_kind == "event" {
            add_dependency(dependencies, to_node, node("relation", relation_ref));
        }
    }

    let policy_binding = projection
        .get("policy_binding")
        .ok_or("projection is missing policy_binding")?;
    let projector_binding = projection
        .get("projector_binding")
        .ok_or("projection is missing projector_binding")?;
    let policy_id = string(policy_binding, "policy_profile_ref")?;
    let projector_id = string(projector_binding, "projector_id")?;
    let policy_node = node("policy", policy_id);
    let projector_node = node("projector", projector_id);
    dependencies.entry(policy_node.clone()).or_default();
    dependencies.entry(projector_node.clone()).or_default();

    let mut decision_refs = BTreeSet::new();
    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        if !decision_refs.insert(decision_ref.to_owned()) {
            return Err(format!("duplicate policy decision {decision_ref}"));
        }
        let event_ref = string(decision, "event_ref")?;
        if !events.contains_key(event_ref) {
            return Err(format!("policy decision {decision_ref} references missing event {event_ref}"));
        }
        let decision_node = node("decision", decision_ref);
        add_dependency(&mut *dependencies, decision_node.clone(), node("event", event_ref));
        add_dependency(&mut *dependencies, decision_node.clone(), policy_node.clone());
        add_dependency(&mut *dependencies, decision_node.clone(), projector_node.clone());
        for assertion in array(decision, "source_assertion_refs")? {
            let assertion_ref = assertion
                .as_str()
                .ok_or("decision source_assertion_refs must contain strings")?;
            if !assertions.contains_key(assertion_ref) {
                return Err(format!("decision {decision_ref} references missing assertion {assertion_ref}"));
            }
            add_dependency(
                dependencies,
                decision_node.clone(),
                node("assertion", assertion_ref),
            );
        }
    }

    let mut entry_refs = BTreeSet::new();
    let mut line_refs = BTreeSet::new();
    for entry in array(projection, "journal_entries")? {
        let entry_ref = string(entry, "journal_entry_ref")?;
        if !entry_refs.insert(entry_ref.to_owned()) {
            return Err(format!("duplicate journal entry {entry_ref}"));
        }
        let entry_node = node("journal_entry", entry_ref);
        for line in array(entry, "lines")? {
            let line_ref = string(line, "line_ref")?;
            if !line_refs.insert(line_ref.to_owned()) {
                return Err(format!("duplicate journal line {line_ref}"));
            }
            let decision_ref = string(line, "policy_decision_ref")?;
            if !decision_refs.contains(decision_ref) {
                return Err(format!("journal line {line_ref} references missing decision {decision_ref}"));
            }
            let line_node = node("journal_line", line_ref);
            add_dependency(
                dependencies,
                entry_node.clone(),
                node("decision", decision_ref),
            );
            add_dependency(
                dependencies,
                line_node.clone(),
                entry_node.clone(),
            );
            add_dependency(
                dependencies,
                line_node.clone(),
                node("decision", decision_ref),
            );
            for event in array(line, "event_refs")? {
                let event_ref = event.as_str().ok_or("journal event_refs must contain strings")?;
                if !events.contains_key(event_ref) {
                    return Err(format!("journal line {line_ref} references missing event {event_ref}"));
                }
                add_dependency(
                    dependencies,
                    line_node.clone(),
                    node("event", event_ref),
                );
            }
            for assertion in array(line, "source_assertion_refs")? {
                let assertion_ref = assertion
                    .as_str()
                    .ok_or("journal source_assertion_refs must contain strings")?;
                if !assertions.contains_key(assertion_ref) {
                    return Err(format!("journal line {line_ref} references missing assertion {assertion_ref}"));
                }
                add_dependency(
                    dependencies,
                    line_node.clone(),
                    node("assertion", assertion_ref),
                );
            }
        }
    }

    for statement in statement_lines {
        let statement_ref = string(statement, "statement_line_ref")?;
        let statement_node = node("statement_line", statement_ref);
        for entry in array(statement, "source_journal_entry_refs")? {
            let entry_ref = entry
                .as_str()
                .ok_or("statement source_journal_entry_refs must contain strings")?;
            if !entry_refs.contains(entry_ref) {
                return Err(format!("statement line {statement_ref} references missing journal entry {entry_ref}"));
            }
            add_dependency(
                dependencies,
                statement_node.clone(),
                node("journal_entry", entry_ref),
            );
        }
        for line in array(statement, "source_journal_line_refs")? {
            let line_ref = line
                .as_str()
                .ok_or("statement source_journal_line_refs must contain strings")?;
            if !line_refs.contains(line_ref) {
                return Err(format!("statement line {statement_ref} references missing journal line {line_ref}"));
            }
            add_dependency(
                dependencies,
                statement_node.clone(),
                node("journal_line", line_ref),
            );
        }
    }

    Ok(())
}

fn dependency_closure(
    changed: &BTreeSet<String>,
    dependencies: &DependencyMap,
) -> BTreeSet<String> {
    let mut dependents = BTreeMap::<String, BTreeSet<String>>::new();
    for (child, parents) in dependencies {
        for parent in parents {
            dependents
                .entry(parent.clone())
                .or_default()
                .insert(child.clone());
        }
    }
    let mut closure = changed.clone();
    let mut frontier = changed.iter().cloned().collect::<Vec<_>>();
    while let Some(current) = frontier.pop() {
        if let Some(children) = dependents.get(&current) {
            for child in children {
                if closure.insert(child.clone()) {
                    frontier.push(child.clone());
                }
            }
        }
    }
    closure
}

fn source_revision_unlinked(case: &Value) -> Result<bool, String> {
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
            .map(|source| string(source, "revision").unwrap_or_default())
            .collect::<BTreeSet<_>>();
        if revisions.len() <= 1 {
            continue;
        }
        group.sort_by(|left, right| optional_string(left, "received_at").cmp(&optional_string(right, "received_at")));
        if optional_string(group[0], "supersedes_source_ref").is_some() {
            return Ok(true);
        }
        for index in 1..group.len() {
            if optional_string(group[index], "supersedes_source_ref")
                != optional_string(group[index - 1], "source_ref")
            {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn assertion_available(case: &Value, assertion: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let source_ref = string(assertion, "source_ref")?;
    let source = values_by_ref(array(case, "source_objects")?, "source_ref")?
        .get(source_ref)
        .copied()
        .ok_or_else(|| format!("assertion references missing source {source_ref}"))?;
    let period = case.get("period").ok_or("case has no period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let valid_from = string(assertion, "valid_from")?;
    let valid_to = assertion.get("valid_to").and_then(Value::as_str);
    let superseded = array(case, "source_objects")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_source_ref") == Some(source_ref)
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    });
    Ok(
        string(source, "received_at")? <= frontier
            && string(assertion, "recorded_at")? <= frontier
            && valid_from <= period_end
            && valid_to.is_none_or(|end| end >= period_start)
            && optional_string(source, "integrity_state") == Some("synthetic_match")
            && optional_string(assertion, "epistemic_state") == Some("asserted")
            && assertion.get("value").is_some_and(|value| !value.is_null())
            && !superseded,
    )
}

fn newly_available_assertions(
    baseline: &Value,
    candidate: &Value,
) -> Result<bool, String> {
    let baseline_map = values_by_ref(array(baseline, "assertions")?, "assertion_ref")?;
    for assertion in array(candidate, "assertions")? {
        let reference = string(assertion, "assertion_ref")?;
        if !assertion_available(candidate, assertion)? {
            continue;
        }
        match baseline_map.get(reference) {
            Some(previous) if !assertion_available(baseline, previous)? => return Ok(true),
            None => {
                let baseline_frontier = string(baseline, "knowledge_at")?;
                let source_ref = string(assertion, "source_ref")?;
                let source = values_by_ref(array(candidate, "source_objects")?, "source_ref")?
                    .get(source_ref)
                    .copied()
                    .ok_or_else(|| format!("late assertion references missing source {source_ref}"))?;
                if string(assertion, "recorded_at")? > baseline_frontier
                    || string(source, "received_at")? > baseline_frontier
                {
                    return Ok(true);
                }
            }
            _ => {}
        }
    }
    Ok(false)
}

fn projection_references_invalid(
    case: &Value,
    projection: &Value,
) -> Result<bool, String> {
    let sources = values_by_ref(array(case, "source_objects")?, "source_ref")?;
    let assertions = values_by_ref(array(case, "assertions")?, "assertion_ref")?;
    let events = values_by_ref(array(case, "events")?, "event_ref")?;
    let mut decisions = BTreeMap::<String, (&Value, &Value)>::new();

    for decision in array(projection, "event_decisions")? {
        let decision_ref = string(decision, "policy_decision_ref")?;
        let event_ref = string(decision, "event_ref")?;
        let Some(event) = events.get(event_ref).copied() else {
            return Ok(true);
        };
        let event_assertions = array(event, "assertion_refs")?
            .iter()
            .filter_map(Value::as_str)
            .collect::<BTreeSet<_>>();
        for assertion_ref in array(decision, "source_assertion_refs")? {
            let assertion_ref = assertion_ref.as_str().ok_or("decision assertion ref must be string")?;
            let Some(assertion) = assertions.get(assertion_ref).copied() else {
                return Ok(true);
            };
            if !event_assertions.contains(assertion_ref) {
                return Ok(true);
            }
            let source_ref = string(assertion, "source_ref")?;
            let Some(source) = sources.get(source_ref).copied() else {
                return Ok(true);
            };
            for event_assertion_ref in &event_assertions {
                if let Some(event_assertion) = assertions.get(*event_assertion_ref) {
                    let event_source_ref = string(event_assertion, "source_ref")?;
                    if let Some(event_source) = sources.get(event_source_ref) {
                        if tenant_root(string(event_source, "tenant_namespace")?)
                            != tenant_root(string(source, "tenant_namespace")?)
                        {
                            return Ok(true);
                        }
                    }
                }
            }
        }
        decisions.insert(decision_ref.to_owned(), (decision, event));
    }

    for entry in array(projection, "journal_entries")? {
        for line in array(entry, "lines")? {
            let decision_ref = string(line, "policy_decision_ref")?;
            let Some((decision, event)) = decisions.get(decision_ref).copied() else {
                return Ok(true);
            };
            let decision_event_ref = string(decision, "event_ref")?;
            let line_events = array(line, "event_refs")?
                .iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
            if !line_events.contains(decision_event_ref) {
                return Ok(true);
            }
            let event_assertions = array(event, "assertion_refs")?
                .iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
            for assertion_ref in array(line, "source_assertion_refs")? {
                let assertion_ref = assertion_ref.as_str().ok_or("journal assertion ref must be string")?;
                if !assertions.contains_key(assertion_ref) || !event_assertions.contains(assertion_ref) {
                    return Ok(true);
                }
                let source_ref = string(assertions[assertion_ref], "source_ref")?;
                let source = sources.get(source_ref).ok_or("journal source missing")?;
                for event_assertion_ref in &event_assertions {
                    if let Some(event_assertion) = assertions.get(*event_assertion_ref) {
                        let event_source_ref = string(event_assertion, "source_ref")?;
                        if let Some(event_source) = sources.get(event_source_ref) {
                            if tenant_root(string(event_source, "tenant_namespace")?)
                                != tenant_root(string(source, "tenant_namespace")?)
                            {
                                return Ok(true);
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(false)
}

fn source_metadata_changed(pair: &Value) -> Result<bool, String> {
    Ok(array(pair, "baseline_source_metadata")?
        != array(pair, "candidate_source_metadata")?)
}

fn analyze_pair(pair: &Value) -> Result<Value, String> {
    let scenario_id = string(pair, "scenario_id")?;
    let baseline_graph = pair.get("baseline_graph_case").ok_or("missing baseline graph")?;
    let candidate_graph = pair.get("candidate_graph_case").ok_or("missing candidate graph")?;
    let baseline_projection = pair.get("baseline_projection").ok_or("missing baseline projection")?;
    let candidate_projection = pair.get("candidate_projection").ok_or("missing candidate projection")?;
    if string(baseline_graph, "case_id")? != string(candidate_graph, "case_id")? {
        return Err(format!("{scenario_id}: baseline and candidate graph case IDs differ"));
    }

    let baseline_hash = canonical_sha256(baseline_graph)?;
    let candidate_hash = canonical_sha256(candidate_graph)?;
    let mut changed = BTreeSet::<String>::new();

    changed.extend(changed_refs(
        array(baseline_graph, "source_objects")?,
        array(candidate_graph, "source_objects")?,
        "source_ref",
        "source",
    )?);
    changed.extend(changed_refs(
        array(baseline_graph, "assertions")?,
        array(candidate_graph, "assertions")?,
        "assertion_ref",
        "assertion",
    )?);
    changed.extend(changed_refs(
        array(baseline_graph, "events")?,
        array(candidate_graph, "events")?,
        "event_ref",
        "event",
    )?);
    changed.extend(changed_refs(
        array(baseline_graph, "relations")?,
        array(candidate_graph, "relations")?,
        "relation_ref",
        "relation",
    )?);

    if optional_string(baseline_graph, "knowledge_at")
        != optional_string(candidate_graph, "knowledge_at")
    {
        changed.insert(node("frontier", "knowledge_at"));
    }
    if baseline_graph.get("period") != candidate_graph.get("period") {
        changed.insert(node("scope", "period"));
    }
    if optional_string(baseline_graph, "entity_scope_ref")
        != optional_string(candidate_graph, "entity_scope_ref")
    {
        changed.insert(node("scope", "entity"));
    }

    let baseline_policy = baseline_projection.get("policy_binding").ok_or("baseline policy binding missing")?;
    let candidate_policy = candidate_projection.get("policy_binding").ok_or("candidate policy binding missing")?;
    let baseline_policy_id = string(baseline_policy, "policy_profile_ref")?;
    let candidate_policy_id = string(candidate_policy, "policy_profile_ref")?;
    let baseline_policy_node = node("policy", baseline_policy_id);
    let candidate_policy_node = node("policy", candidate_policy_id);
    if baseline_policy != candidate_policy || baseline_policy_node != candidate_policy_node {
        changed.insert(candidate_policy_node);
        changed.insert(baseline_policy_node);
    }

    let baseline_projector = baseline_projection.get("projector_binding").ok_or("baseline projector binding missing")?;
    let candidate_projector = candidate_projection.get("projector_binding").ok_or("candidate projector binding missing")?;
    let baseline_projector_id = string(baseline_projector, "projector_id")?;
    let candidate_projector_id = string(candidate_projector, "projector_id")?;
    let baseline_projector_node = node("projector", baseline_projector_id);
    let candidate_projector_node = node("projector", candidate_projector_id);
    if baseline_projector != candidate_projector || baseline_projector_node != candidate_projector_node {
        changed.insert(candidate_projector_node);
        changed.insert(baseline_projector_node);
    }

    changed.extend(changed_refs(
        array(baseline_projection, "event_decisions")?,
        array(candidate_projection, "event_decisions")?,
        "policy_decision_ref",
        "decision",
    )?);
    changed.extend(changed_refs(
        array(baseline_projection, "journal_entries")?,
        array(candidate_projection, "journal_entries")?,
        "journal_entry_ref",
        "journal_entry",
    )?);
    let baseline_lines = array(baseline_projection, "journal_entries")?
        .iter().flat_map(|entry| array(entry, "lines").unwrap_or(&[]).iter().cloned()).collect::<Vec<_>>();
    let candidate_lines = array(candidate_projection, "journal_entries")?
        .iter().flat_map(|entry| array(entry, "lines").unwrap_or(&[]).iter().cloned()).collect::<Vec<_>>();
    changed.extend(changed_refs(&baseline_lines, &candidate_lines, "line_ref", "journal_line")?);
    changed.extend(changed_refs(
        array(pair, "baseline_statement_lines")?,
        array(pair, "candidate_statement_lines")?,
        "statement_line_ref",
        "statement_line",
    )?);

    let mut dependencies = DependencyMap::new();
    collect_dependencies(
        baseline_graph,
        baseline_projection,
        array(pair, "baseline_statement_lines")?,
        &mut dependencies,
    )?;
    collect_dependencies(
        candidate_graph,
        candidate_projection,
        array(pair, "candidate_statement_lines")?,
        &mut dependencies,
    )?;
    let impacted = dependency_closure(&changed, &dependencies);

    let mut findings = BTreeSet::<String>::new();
    if changed.iter().any(|reference| reference.starts_with("source:") || reference.starts_with("assertion:")) {
        findings.insert("source_or_assertion_changed".to_owned());
    }
    if changed.iter().any(|reference| reference.starts_with("event:") || reference.starts_with("relation:")) {
        findings.insert("event_or_relation_changed".to_owned());
    }
    if changed.iter().any(|reference| reference.starts_with("policy:")) {
        findings.insert("policy_profile_changed".to_owned());
    }
    if changed.iter().any(|reference| reference.starts_with("projector:")) {
        findings.insert("projector_changed".to_owned());
    }
    if changed.contains(&node("frontier", "knowledge_at")) {
        findings.insert("knowledge_frontier_advanced".to_owned());
    }
    let late_evidence = newly_available_assertions(baseline_graph, candidate_graph)?;
    if late_evidence {
        findings.insert("late_evidence_became_available".to_owned());
    }

    let mut assertion_review = false;
    let mut superseded_source_reference = false;
    let referenced_assertions = array(candidate_projection, "event_decisions")?
        .iter()
        .flat_map(|decision| array(decision, "source_assertion_refs").unwrap_or(&[]).iter())
        .filter_map(Value::as_str)
        .collect::<BTreeSet<_>>();
    for entry in array(candidate_projection, "journal_entries")? {
        for line in array(entry, "lines")? {
            for reference in array(line, "source_assertion_refs")? {
                if let Some(reference) = reference.as_str() {
                    referenced_assertions.insert(reference);
                }
            }
        }
    }

    let candidate_assertions = values_by_ref(array(candidate_graph, "assertions")?, "assertion_ref")?;
    let candidate_sources = values_by_ref(array(candidate_graph, "source_objects")?, "source_ref")?;
    for assertion_ref in &referenced_assertions {
        if let Some(assertion) = candidate_assertions.get(assertion_ref) {
            if matches!(
                optional_string(assertion, "epistemic_state"),
                Some("disputed" | "invalidated")
            ) {
                assertion_review = true;
            }
            if let Some(source_ref) = optional_string(assertion, "source_ref") {
                if candidate_sources.get(source_ref).is_some_and(|source| {
                    optional_string(source, "integrity_state") == Some("synthetic_mismatch")
                }) {
                    findings.insert("source_integrity_failed".to_owned());
                    assertion_review = true;
                }
                if array(candidate_graph, "source_objects")?.iter().any(|source| {
                    optional_string(source, "supersedes_source_ref") == Some(source_ref)
                        && optional_string(source, "received_at")
                            .is_some_and(|received| received <= optional_string(candidate_graph, "knowledge_at").unwrap_or(""))
                }) {
                    superseded_source_reference = true;
                }
            }
        }
    }
    if assertion_review {
        findings.insert("assertion_disputed_or_invalidated".to_owned());
    }
    if superseded_source_reference {
        findings.insert("superseded_source_reference".to_owned());
    }

    let unlinked_revision = source_revision_unlinked(candidate_graph)?;
    if unlinked_revision {
        findings.insert("unlinked_source_revision".to_owned());
    }

    let invalid_projection = projection_references_invalid(candidate_graph, candidate_projection)?;
    if invalid_projection {
        findings.insert("projection_reference_invalid".to_owned());
    }

    let candidate_binding = candidate_projection.get("graph_binding").ok_or("candidate graph binding missing")?;
    let candidate_binding_hash = string(candidate_binding, "graph_sha256")?;
    let candidate_binding_case = string(candidate_binding, "graph_case_id")?;
    let stale_binding = candidate_binding_hash != candidate_hash
        || candidate_binding_case != string(candidate_graph, "case_id")?;
    if stale_binding {
        findings.insert("projection_graph_binding_stale".to_owned());
    }

    let mut undeclared = BTreeSet::new();
    if let Some(declared) = pair.get("caller_declared_impacted_refs").and_then(Value::as_array) {
        let declared = declared.iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
        undeclared.extend(
            impacted
                .iter()
                .filter(|reference| !declared.contains(reference.as_str()))
                .cloned(),
        );
    }
    let request_valid = undeclared.is_empty();
    if !request_valid {
        findings.insert("caller_impact_set_underreported".to_owned());
    }

    let scenario_kind = string(pair, "scenario_kind")?;
    let mut impact_class: Option<&str> = if invalid_projection {
        Some("projection_invalid")
    } else if assertion_review || unlinked_revision {
        Some("review_required")
    } else if late_evidence {
        Some("historical_replay_required")
    } else if !changed.is_empty() || stale_binding {
        Some("reproject_required")
    } else {
        Some("no_material_impact")
    };
    if !request_valid {
        impact_class = None;
    }
    if source_metadata_changed(pair)? && changed.is_empty() {
        findings.insert("nonsemantic_source_metadata_changed".to_owned());
    }

    // Scenario label is descriptive only; it never determines the impact class.
    let _ = scenario_kind;
    Ok(json!({
        "scenario_id": scenario_id,
        "scenario_kind": scenario_kind,
        "request_valid": request_valid,
        "impact_class": impact_class,
        "baseline_graph_sha256": baseline_hash,
        "candidate_graph_sha256": candidate_hash,
        "changed_node_refs": changed.into_iter().collect::<Vec<_>>(),
        "impacted_node_refs": impacted.into_iter().collect::<Vec<_>>(),
        "undeclared_impacted_node_refs": undeclared.into_iter().collect::<Vec<_>>(),
        "finding_codes": findings.into_iter().collect::<Vec<_>>()
    }))
}

fn build_report(request: &Value, request_sha256: String) -> Result<Value, String> {
    let mut seen = BTreeSet::new();
    let mut results = Vec::new();
    for pair in array(request, "pairs")? {
        let id = string(pair, "scenario_id")?;
        if !seen.insert(id.to_owned()) {
            return Err(format!("duplicate drift scenario {id}"));
        }
        results.push(analyze_pair(pair)?);
    }
    Ok(json!({
        "report_id": "MYCELIX-FIN-EVIDENCE-DRIFT-S0-REPORT",
        "schema_version": "1.0.0",
        "claim_ceiling": "deterministic_dependency_impact_assessment_not_professional_accounting_conclusion",
        "request_sha256": request_sha256,
        "pair_count": results.len(),
        "pair_results": results
    }))
}

fn run() -> Result<(), Box<dyn Error>> {
    let default_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../evidence-drift-v1");
    let mut data_dir = default_dir;
    let mut output_path: Option<PathBuf> = None;
    let mut args = env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--data-dir" => data_dir = PathBuf::from(args.next().ok_or("--data-dir requires a path")?),
            "--output" => output_path = Some(PathBuf::from(args.next().ok_or("--output requires a path")?)),
            "--help" | "-h" => {
                println!(
                    "Analyzes financial evidence change impact from typed dependency edges.\n\n\
                     --data-dir PATH  Evidence-drift package directory (default: sibling evidence-drift-v1)\n\
                     --output PATH    Write report JSON to PATH instead of stdout\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument {other}; use --help").into()),
        }
    }

    let event_dir = data_dir.join("../economic-event-core-v1");
    let request_schema: Value = read_json(data_dir.join("financial_evidence_drift_request_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("financial_evidence_drift_report_v1.0.0.schema.json"))?;
    let event_schema: Value = read_json(event_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let projection_schema: Value = read_json(event_dir.join("accounting_policy_projection_v1.0.0.schema.json"))?;
    let request_path = data_dir.join("fixtures/financial_evidence_drift_candidates_v1.0.0.json");
    let request_bytes = read_bytes(request_path)?;
    let request: Value = serde_json::from_slice(&request_bytes)?;
    validate_instance("financial evidence drift request", &request_schema, &request)?;

    // Revalidate the embedded graph cases and policy projections against their owning schemas.
    let case_schema = json!({
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$ref": "#/$defs/case",
        "$defs": event_schema["$defs"].clone()
    });
    for pair in array(&request, "pairs")? {
        validate_instance("baseline event graph case", &case_schema, &pair["baseline_graph_case"])?;
        validate_instance("candidate event graph case", &case_schema, &pair["candidate_graph_case"])?;
        validate_instance("baseline policy projection", &projection_schema, &pair["baseline_projection"])?;
        validate_instance("candidate policy projection", &projection_schema, &pair["candidate_projection"])?;
    }

    let request_sha256 = hex::encode(Sha256::digest(&request_bytes));
    let report = build_report(&request, request_sha256)?;
    validate_instance("financial evidence drift report", &report_schema, &report)?;
    let report_bytes = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &report_bytes)?;
            println!("Wrote financial evidence drift report for {} pairs.", report["pair_count"]);
        }
        None => println!("{}", String::from_utf8(report_bytes)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value, Value) {
        let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../evidence-drift-v1");
        (
            read_json(dir.join("fixtures/financial_evidence_drift_candidates_v1.0.0.json")).unwrap(),
            read_json(dir.join("financial_evidence_drift_request_v1.0.0.schema.json")).unwrap(),
            read_json(dir.join("financial_evidence_drift_report_v1.0.0.schema.json")).unwrap(),
        )
    }

    fn result<'a>(report: &'a Value, scenario_id: &str) -> &'a Value {
        array(report, "pair_results").unwrap().iter()
            .find(|entry| optional_string(entry, "scenario_id") == Some(scenario_id)).unwrap()
    }

    #[test]
    fn eleven_synthetic_drift_scenarios_produce_a_schema_valid_report() {
        let (request, schema, report_schema) = fixtures();
        validate_instance("request", &schema, &request).unwrap();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        validate_instance("report", &report_schema, &report).unwrap();
        assert_eq!(report["pair_count"], 11);
    }

    #[test]
    fn amount_correction_reprojects_transitive_decision_journal_and_statement_dependents() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-001-AMOUNT-CORRECTION");
        assert_eq!(row["impact_class"], "reproject_required");
        let impacted = set_from_string_array(row, "impacted_node_refs").unwrap();
        for reference in [
            "source:SRC-002-INV-R2",
            "assertion:AS-002-AMOUNT-R2",
            "event:EV-002-INVOICE",
            "decision:DEC-002-INVOICE",
            "journal_entry:JE-002-ACCRUAL",
            "journal_line:JL-002-DR",
            "statement_line:ST-002-EXP",
        ] {
            assert!(impacted.contains(reference), "missing impacted node {reference}");
        }
    }

    #[test]
    fn nonsemantic_source_metadata_change_does_not_trigger_reprojection() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-002-METADATA-ONLY");
        assert_eq!(row["impact_class"], "no_material_impact");
        assert!(array(row, "changed_node_refs").unwrap().is_empty());
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("nonsemantic_source_metadata_changed")));
    }

    #[test]
    fn assertion_invalidation_requires_review_and_invalidates_dependents() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-003-ASSERTION-INVALIDATION");
        assert_eq!(row["impact_class"], "review_required");
        assert!(array(row, "impacted_node_refs").unwrap().iter()
            .any(|value| value == "statement_line:ST-002-EXP"));
    }

    #[test]
    fn late_evidence_requires_historical_replay_without_rewriting_prior_frontier() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-004-LATE-EVIDENCE");
        assert_eq!(row["impact_class"], "historical_replay_required");
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("late_evidence_became_available")));
        let pair = array(&request, "pairs").unwrap().iter()
            .find(|pair| optional_string(pair, "scenario_id") == Some("DRIFT-004-LATE-EVIDENCE")).unwrap();
        let late = array(&pair["candidate_graph_case"], "assertions").unwrap().iter()
            .find(|assertion| optional_string(assertion, "assertion_ref") == Some("AS-002-LATE-SCOPE")).unwrap();
        assert!(!assertion_available(&pair["baseline_graph_case"], late).unwrap());
        assert!(assertion_available(&pair["candidate_graph_case"], late).unwrap());
    }

    #[test]
    fn policy_digest_change_invalidates_all_projection_decisions() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-005-POLICY-DIGEST-CHANGE");
        assert_eq!(row["impact_class"], "reproject_required");
        assert!(array(row, "impacted_node_refs").unwrap().iter()
            .any(|value| value == "statement_line:ST-002-EXP"));
    }

    #[test]
    fn projector_digest_change_invalidates_all_projection_decisions() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-006-PROJECTOR-DIGEST-CHANGE");
        assert_eq!(row["impact_class"], "reproject_required");
        assert!(array(row, "impacted_node_refs").unwrap().iter()
            .any(|value| value == "journal_entry:JE-002-ACCRUAL"));
    }

    #[test]
    fn event_relation_change_propagates_through_related_event_projection() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-007-EVENT-RELATION-CHANGE");
        assert_eq!(row["impact_class"], "reproject_required");
        assert!(array(row, "impacted_node_refs").unwrap().iter()
            .any(|value| value == "decision:DEC-002-INVOICE"));
    }

    #[test]
    fn foreign_tenant_assertion_cannot_be_substituted_into_projection() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-008-CROSS-TENANT-PROJECTION-REF");
        assert_eq!(row["impact_class"], "projection_invalid");
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("projection_reference_invalid")));
    }

    #[test]
    fn unlinked_source_revision_requires_review() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-009-UNLINKED-REVISION");
        assert_eq!(row["impact_class"], "review_required");
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("unlinked_source_revision")));
    }

    #[test]
    fn underreported_caller_impact_set_is_rejected() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-010-UNDERREPORTED-IMPACT");
        assert_eq!(row["request_valid"], false);
        assert_eq!(row["impact_class"], Value::Null);
        assert!(!array(row, "undeclared_impacted_node_refs").unwrap().is_empty());
    }

    #[test]
    fn source_integrity_change_requires_review_and_never_remains_qualified() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-011-SOURCE-INTEGRITY-CHANGE");
        assert_eq!(row["impact_class"], "review_required");
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("source_integrity_failed")));
    }

    #[test]
    fn stale_projection_graph_binding_is_detected_even_when_projection_is_balanced() {
        let (request, _, _) = fixtures();
        let report = build_report(&request, "a".repeat(64)).unwrap();
        let row = result(&report, "DRIFT-001-AMOUNT-CORRECTION");
        assert!(array(row, "finding_codes").unwrap().iter()
            .any(|code| code.as_str() == Some("projection_graph_binding_stale")));
    }

    #[test]
    fn changed_graph_hash_is_deterministic_for_the_same_case() {
        let (request, _, _) = fixtures();
        let pair = array(&request, "pairs").unwrap().first().unwrap();
        assert_eq!(
            canonical_sha256(&pair["baseline_graph_case"]).unwrap(),
            canonical_sha256(&pair["baseline_graph_case"]).unwrap()
        );
    }
}
