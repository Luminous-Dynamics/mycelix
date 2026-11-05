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
        eprintln!("economic-event-graph-s0: {error}");
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

fn value_is_null(value: &Value, key: &str) -> bool {
    value.get(key).is_none_or(Value::is_null)
}

fn find_by_ref<'a>(values: &'a [Value], field: &str, reference: &str) -> Option<&'a Value> {
    values
        .iter()
        .find(|value| optional_string(value, field) == Some(reference))
}

fn source_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "source_objects")?, "source_ref", reference)
        .ok_or_else(|| format!("unknown source reference {reference}"))
}

fn assertion_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "assertions")?, "assertion_ref", reference)
        .ok_or_else(|| format!("unknown assertion reference {reference}"))
}

fn event_by_ref<'a>(case: &'a Value, reference: &str) -> Result<&'a Value, String> {
    find_by_ref(array(case, "events")?, "event_ref", reference)
        .ok_or_else(|| format!("unknown event reference {reference}"))
}

fn reference_exists(case: &Value, kind: &str, reference: &str) -> Result<bool, String> {
    let values = match kind {
        "source" => array(case, "source_objects")?,
        "assertion" => array(case, "assertions")?,
        "event" => array(case, "events")?,
        other => return Err(format!("unsupported reference kind {other}")),
    };
    let field = match kind {
        "source" => "source_ref",
        "assertion" => "assertion_ref",
        "event" => "event_ref",
        _ => unreachable!(),
    };
    Ok(find_by_ref(values, field, reference).is_some())
}

fn source_is_superseded_at_frontier(case: &Value, source_ref: &str) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    Ok(array(case, "source_objects")?.iter().any(|candidate| {
        optional_string(candidate, "supersedes_source_ref") == Some(source_ref)
            && optional_string(candidate, "received_at").is_some_and(|received| received <= frontier)
    }))
}

fn source_is_available(case: &Value, source: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let source_ref = string(source, "source_ref")?;
    Ok(string(source, "received_at")? <= frontier
        && optional_string(source, "integrity_state") == Some("synthetic_match")
        && !source_is_superseded_at_frontier(case, source_ref)?)
}

fn assertion_is_available(case: &Value, assertion: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    let source = source_by_ref(case, string(assertion, "source_ref")?)?;
    let period = case.get("period").ok_or("missing period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;
    let valid_from = string(assertion, "valid_from")?;
    let valid_to = assertion.get("valid_to").and_then(Value::as_str);
    let valid_overlap = valid_from <= period_end
        && valid_to.is_none_or(|end| end >= period_start);
    Ok(string(assertion, "recorded_at")? <= frontier
        && valid_overlap
        && source_is_available(case, source)?
        && optional_string(assertion, "epistemic_state") == Some("asserted")
        && !value_is_null(assertion, "value"))
}

fn event_is_available(case: &Value, event: &Value) -> Result<bool, String> {
    let frontier = string(case, "knowledge_at")?;
    if string(event, "recorded_at")? > frontier {
        return Ok(false);
    }
    for assertion_ref in array(event, "assertion_refs")? {
        let reference = assertion_ref
            .as_str()
            .ok_or("event assertion_refs must contain strings")?;
        if !assertion_is_available(case, assertion_by_ref(case, reference)?)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn valid_reference_for_kind(case: &Value, kind: &str, reference: &str) -> Result<(), String> {
    if reference_exists(case, kind, reference)? {
        Ok(())
    } else {
        Err(format!("{kind} reference {reference} is not present"))
    }
}

fn validate_source_objects(case: &Value) -> Result<(usize, usize), String> {
    let case_id = string(case, "case_id")?;
    let mut refs = BTreeSet::new();
    let mut identities = BTreeSet::new();
    let mut revision_groups = BTreeMap::<(String, String, String), Vec<&Value>>::new();
    let mut integrity_mismatches = 0usize;
    let frontier = string(case, "knowledge_at")?;

    for source in array(case, "source_objects")? {
        let source_ref = string(source, "source_ref")?;
        if !refs.insert(source_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate source_ref {source_ref}"));
        }
        let namespace = string(source, "tenant_namespace")?.to_owned();
        let external_id = string(source, "external_id")?.to_owned();
        let revision = string(source, "revision")?.to_owned();
        let source_kind = string(source, "source_kind")?.to_owned();
        let identity = (
            namespace.clone(),
            external_id.clone(),
            revision.clone(),
            source_kind.clone(),
        );
        if !identities.insert(identity) {
            return Err(format!("{case_id}: duplicate source identity {namespace}/{external_id}/{revision}"));
        }
        revision_groups
            .entry((namespace, external_id, source_kind))
            .or_default()
            .push(source);

        let received_at = string(source, "received_at")?;
        if optional_string(source, "integrity_state") == Some("synthetic_mismatch")
            && received_at <= frontier
        {
            integrity_mismatches += 1;
        }
    }

    for source in array(case, "source_objects")? {
        let Some(previous_ref) = optional_string(source, "supersedes_source_ref") else {
            continue;
        };
        if optional_string(source, "supersedes_source_ref")
            == optional_string(source, "source_ref")
        {
            return Err(format!("{case_id}: source cannot supersede itself"));
        }
        let previous = source_by_ref(case, previous_ref)?;
        if string(previous, "tenant_namespace")? != string(source, "tenant_namespace")?
            || string(previous, "external_id")? != string(source, "external_id")?
            || string(previous, "source_kind")? != string(source, "source_kind")?
            || string(previous, "revision")? == string(source, "revision")?
            || string(source, "received_at")? <= string(previous, "received_at")?
        {
            return Err(format!("{case_id}: invalid source supersession identity or ordering"));
        }
    }

    let mut unlinked_revision_families = 0usize;
    for (_, mut group) in revision_groups {
        let revisions = group
            .iter()
            .map(|source| string(source, "revision").unwrap_or_default())
            .collect::<BTreeSet<_>>();
        if revisions.len() <= 1 {
            continue;
        }
        group.sort_by(|left, right| {
            optional_string(left, "received_at").cmp(&optional_string(right, "received_at"))
        });
        let mut linked = optional_string(group[0], "supersedes_source_ref").is_none();
        for index in 1..group.len() {
            let prior_ref = string(group[index - 1], "source_ref")?;
            if optional_string(group[index], "supersedes_source_ref") != Some(prior_ref) {
                linked = false;
            }
        }
        if !linked {
            unlinked_revision_families += 1;
        }
    }

    Ok((unlinked_revision_families, integrity_mismatches))
}

fn validate_assertions(case: &Value) -> Result<(usize, usize), String> {
    let case_id = string(case, "case_id")?;
    let mut refs = BTreeSet::new();
    let mut available_count = 0usize;
    let mut future_count = 0usize;
    let frontier = string(case, "knowledge_at")?;
    let period = case.get("period").ok_or("missing period")?;
    let period_start = string(period, "start")?;
    let period_end = string(period, "end")?;

    for assertion in array(case, "assertions")? {
        let assertion_ref = string(assertion, "assertion_ref")?;
        if !refs.insert(assertion_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate assertion_ref {assertion_ref}"));
        }
        let source = source_by_ref(case, string(assertion, "source_ref")?)?;
        let state = string(assertion, "epistemic_state")?;
        match state {
            "asserted" if value_is_null(assertion, "value") => {
                return Err(format!("{case_id}/{assertion_ref}: asserted value is null"));
            }
            "unknown" | "invalidated" if !value_is_null(assertion, "value") => {
                return Err(format!("{case_id}/{assertion_ref}: unknown/invalidated value must be null"));
            }
            _ => {}
        }
        let valid_from = string(assertion, "valid_from")?;
        let valid_to = assertion.get("valid_to").and_then(Value::as_str);
        if valid_to.is_some_and(|end| end < valid_from) {
            return Err(format!("{case_id}/{assertion_ref}: inverted valid-time interval"));
        }
        if assertion_is_available(case, assertion)? {
            available_count += 1;
        }
        if string(assertion, "recorded_at")? > frontier
            || string(source, "received_at")? > frontier
        {
            future_count += 1;
        }
        if state == "asserted"
            && valid_from > period_end
            && valid_to.is_none_or(|end| end < period_start)
        {
            // A valid assertion may be outside this period; it remains represented but is not available.
        }
    }
    Ok((available_count, future_count))
}

fn event_assertion_types(case: &Value, event: &Value) -> Result<BTreeSet<String>, String> {
    let mut types = BTreeSet::new();
    for reference in array(event, "assertion_refs")? {
        let assertion_ref = reference.as_str().ok_or("event assertion ref is not a string")?;
        let assertion = assertion_by_ref(case, assertion_ref)?;
        types.insert(string(assertion, "fact_type")?.to_owned());
    }
    Ok(types)
}


fn validate_event_amount_matches_assertion(
    case: &Value,
    event: &Value,
    expected_fact_type: &str,
) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let event_ref = string(event, "event_ref")?;
    let matches = array(event, "assertion_refs")?
        .iter()
        .map(|value| value.as_str().ok_or("event assertion ref is not a string"))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|reference| assertion_by_ref(case, reference))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .filter(|assertion| optional_string(assertion, "fact_type") == Some(expected_fact_type))
        .collect::<Vec<_>>();
    if matches.len() != 1 {
        return Err(format!(
            "{case_id}/{event_ref}: expected exactly one {expected_fact_type} assertion, found {}",
            matches.len()
        ));
    }
    let assertion = matches[0];
    let source_amount = assertion.get("value").and_then(Value::as_i64)
        .ok_or_else(|| format!("{case_id}/{event_ref}: {expected_fact_type} must be integer minor units"))?;
    let event_amount = event.get("amount_minor").and_then(Value::as_i64)
        .ok_or_else(|| format!("{case_id}/{event_ref}: event amount_minor must be integer minor units"))?;
    let source_currency = optional_string(assertion, "currency")
        .ok_or_else(|| format!("{case_id}/{event_ref}: {expected_fact_type} assertion has no currency"))?;
    let event_currency = string(event, "currency")?;
    if source_amount != event_amount || source_currency != event_currency {
        return Err(format!(
            "{case_id}/{event_ref}: event amount/currency {event_amount} {event_currency} does not match source assertion {source_amount} {source_currency}"
        ));
    }
    Ok(())
}

fn validate_event_date_matches_assertion(
    case: &Value,
    event: &Value,
    expected_fact_type: &str,
) -> Result<(), String> {
    let case_id = string(case, "case_id")?;
    let event_ref = string(event, "event_ref")?;
    let matches = array(event, "assertion_refs")?
        .iter()
        .map(|value| value.as_str().ok_or("event assertion ref is not a string"))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .map(|reference| assertion_by_ref(case, reference))
        .collect::<Result<Vec<_>, _>>()?
        .into_iter()
        .filter(|assertion| optional_string(assertion, "fact_type") == Some(expected_fact_type))
        .collect::<Vec<_>>();
    if matches.len() != 1 {
        return Err(format!(
            "{case_id}/{event_ref}: expected exactly one {expected_fact_type} date assertion, found {}",
            matches.len()
        ));
    }
    let asserted_date = string(matches[0], "value")?;
    let event_date = string(event, "occurred_at")?
        .get(..10)
        .ok_or_else(|| format!("{case_id}/{event_ref}: occurred_at does not contain a date prefix"))?;
    if asserted_date != event_date {
        return Err(format!(
            "{case_id}/{event_ref}: event occurred date {event_date} does not match {expected_fact_type} {asserted_date}"
        ));
    }
    Ok(())
}

fn validate_service_period_assertions(case: &Value, event: &Value) -> Result<(), String> {
    let refs = array(event, "assertion_refs")?
        .iter()
        .map(|value| value.as_str().ok_or("event assertion ref is not a string"))
        .collect::<Result<Vec<_>, _>>()?;
    let mut start: Option<&str> = None;
    let mut end: Option<&str> = None;
    for reference in refs {
        let assertion = assertion_by_ref(case, reference)?;
        match optional_string(assertion, "fact_type") {
            Some("service_period_start") => start = Some(string(assertion, "value")?),
            Some("service_period_end") => end = Some(string(assertion, "value")?),
            _ => {}
        }
    }
    if start.zip(end).is_some_and(|(start, end)| start > end) {
        return Err(format!(
            "{}/{}: service period start is after service period end",
            string(case, "case_id")?,
            string(event, "event_ref")?
        ));
    }
    Ok(())
}

fn validate_events(case: &Value) -> Result<(usize, usize), String> {
    let case_id = string(case, "case_id")?;
    let mut refs = BTreeSet::new();
    let mut available_count = 0usize;
    let mut future_count = 0usize;
    let frontier = string(case, "knowledge_at")?;

    for event in array(case, "events")? {
        let event_ref = string(event, "event_ref")?;
        if !refs.insert(event_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate event_ref {event_ref}"));
        }
        let assertion_refs = array(event, "assertion_refs")?;
        if assertion_refs.is_empty() {
            return Err(format!("{case_id}/{event_ref}: event has no supporting assertions"));
        }
        for reference in assertion_refs {
            let assertion_ref = reference.as_str().ok_or("event assertion ref is not a string")?;
            valid_reference_for_kind(case, "assertion", assertion_ref)?;
        }
        let amount_is_null = value_is_null(event, "amount_minor");
        let currency_is_null = value_is_null(event, "currency");
        if amount_is_null != currency_is_null {
            return Err(format!("{case_id}/{event_ref}: amount and currency must be both present or both null"));
        }

        let types = event_assertion_types(case, event)?;
        match string(event, "event_kind")? {
            "service_delivery" => {
                if !types.contains("service_acceptance_date") {
                    return Err(format!("{case_id}/{event_ref}: service_delivery lacks service acceptance assertion"));
                }
                validate_event_date_matches_assertion(case, event, "service_acceptance_date")?;
                validate_service_period_assertions(case, event)?;
            }
            "invoice_issued" => {
                if !types.contains("invoice_amount_minor") {
                    return Err(format!("{case_id}/{event_ref}: invoice_issued lacks invoice amount assertion"));
                }
                validate_event_amount_matches_assertion(case, event, "invoice_amount_minor")?;
            }
            "credit_issued" => {
                if !types.contains("credit_amount_minor") || !types.contains("original_invoice_id") {
                    return Err(format!("{case_id}/{event_ref}: credit_issued lacks credit amount or parent assertion"));
                }
                validate_event_amount_matches_assertion(case, event, "credit_amount_minor")?;
            }
            "settlement_made" | "settlement_received" => {
                if !types.contains("payment_amount_minor") || !types.contains("payment_date") {
                    return Err(format!("{case_id}/{event_ref}: settlement event lacks payment amount/date assertion"));
                }
                validate_event_amount_matches_assertion(case, event, "payment_amount_minor")?;
                validate_event_date_matches_assertion(case, event, "payment_date")?;
            }
            _ => {}
        }

        if event_is_available(case, event)? {
            available_count += 1;
        }
        if string(event, "recorded_at")? > frontier {
            future_count += 1;
        } else {
            for reference in assertion_refs {
                let assertion_ref = reference.as_str().ok_or("event assertion ref is not a string")?;
                let assertion = assertion_by_ref(case, assertion_ref)?;
                let source = source_by_ref(case, string(assertion, "source_ref")?)?;
                if string(source, "received_at")? > frontier
                    || string(assertion, "recorded_at")? > frontier
                {
                    future_count += 1;
                    break;
                }
            }
        }
    }
    Ok((available_count, future_count))
}

fn validate_relation_semantics(
    case: &Value,
    relation: &Value,
    from_kind: &str,
    to_kind: &str,
) -> Result<(), String> {
    let kind = string(relation, "relation_kind")?;
    let valid = match kind {
        "settles" | "reverses" | "allocates" | "corrects" | "causally_depends_on" => {
            from_kind == "event" && to_kind == "event"
        }
        "contradicts" => {
            (from_kind == "assertion" && to_kind == "assertion")
                || (from_kind == "event" && to_kind == "event")
        }
        "supersedes" | "duplicate_of" => {
            from_kind == to_kind && matches!(from_kind, "source" | "event")
        }
        "supports" => true,
        _ => false,
    };
    if !valid {
        return Err(format!(
            "{}: relation {kind} cannot connect {from_kind} to {to_kind}",
            string(case, "case_id")?
        ));
    }
    if kind == "settles" {
        let from = event_by_ref(case, string(relation, "from_ref")?)?;
        if !matches!(
            optional_string(from, "event_kind"),
            Some("settlement_made" | "settlement_received")
        ) {
            return Err(format!("{}: settles relation must originate at a settlement event", string(case, "case_id")?));
        }
    }
    if kind == "corrects" {
        let from = event_by_ref(case, string(relation, "from_ref")?)?;
        let to = event_by_ref(case, string(relation, "to_ref")?)?;
        if optional_string(from, "event_kind") != Some("credit_issued")
            || optional_string(to, "event_kind") != Some("invoice_issued")
        {
            return Err(format!("{}: corrects relation must link credit_issued to invoice_issued", string(case, "case_id")?));
        }
        let credit_parent_id = array(from, "assertion_refs")?
            .iter()
            .filter_map(Value::as_str)
            .map(|reference| assertion_by_ref(case, reference))
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .find(|assertion| optional_string(assertion, "fact_type") == Some("original_invoice_id"))
            .and_then(|assertion| assertion.get("value").and_then(Value::as_str))
            .ok_or("credit_issued event has no original_invoice_id value")?;
        let credit_parent_assertion = array(from, "assertion_refs")?
            .iter()
            .map(|reference| {
                let reference = reference.as_str().ok_or("credit event assertion ref is not a string")?;
                assertion_by_ref(case, reference)
            })
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .find(|assertion| optional_string(assertion, "fact_type") == Some("original_invoice_id"))
            .ok_or("credit event has no original_invoice_id assertion")?;
        let credit_source = source_by_ref(case, string(credit_parent_assertion, "source_ref")?)?;
        let mut parent_invoice_found = false;
        for reference in array(to, "assertion_refs")? {
            let reference = reference.as_str().ok_or("invoice event assertion ref is not a string")?;
            let assertion = assertion_by_ref(case, reference)?;
            if optional_string(assertion, "fact_type") != Some("invoice_amount_minor") {
                continue;
            }
            let source = source_by_ref(case, string(assertion, "source_ref")?)?;
            if optional_string(source, "external_id") == Some(credit_parent_id)
                && optional_string(source, "tenant_namespace")
                    == optional_string(credit_source, "tenant_namespace")
            {
                parent_invoice_found = true;
                break;
            }
        }
        if !parent_invoice_found {
            return Err(format!("{}: credit note parent invoice is not in the same tenant namespace and source identity", string(case, "case_id")?));
        }
    }
    Ok(())
}

fn graph_has_cycle(edges: &[(String, String)]) -> bool {
    let mut adjacency = BTreeMap::<String, Vec<String>>::new();
    for (from, to) in edges {
        adjacency.entry(from.clone()).or_default().push(to.clone());
    }
    fn visit(
        node: &str,
        adjacency: &BTreeMap<String, Vec<String>>,
        visiting: &mut BTreeSet<String>,
        visited: &mut BTreeSet<String>,
    ) -> bool {
        if visiting.contains(node) {
            return true;
        }
        if visited.contains(node) {
            return false;
        }
        visiting.insert(node.to_owned());
        if let Some(next) = adjacency.get(node) {
            for child in next {
                if visit(child, adjacency, visiting, visited) {
                    return true;
                }
            }
        }
        visiting.remove(node);
        visited.insert(node.to_owned());
        false
    }
    let mut visiting = BTreeSet::new();
    let mut visited = BTreeSet::new();
    adjacency
        .keys()
        .any(|node| visit(node, &adjacency, &mut visiting, &mut visited))
}

fn validate_relations(case: &Value) -> Result<(usize, usize), String> {
    let case_id = string(case, "case_id")?;
    let mut refs = BTreeSet::new();
    let mut contradiction_count = 0usize;
    let mut supersession_edges = Vec::<(String, String)>::new();

    for relation in array(case, "relations")? {
        let relation_ref = string(relation, "relation_ref")?;
        if !refs.insert(relation_ref.to_owned()) {
            return Err(format!("{case_id}: duplicate relation_ref {relation_ref}"));
        }
        let from_kind = string(relation, "from_kind")?;
        let from_ref = string(relation, "from_ref")?;
        let to_kind = string(relation, "to_kind")?;
        let to_ref = string(relation, "to_ref")?;
        valid_reference_for_kind(case, from_kind, from_ref)?;
        valid_reference_for_kind(case, to_kind, to_ref)?;
        if from_kind == to_kind && from_ref == to_ref {
            return Err(format!("{case_id}/{relation_ref}: self relation is not permitted"));
        }
        validate_relation_semantics(case, relation, from_kind, to_kind)?;

        for assertion_ref in array(relation, "source_assertion_refs")? {
            let reference = assertion_ref
                .as_str()
                .ok_or("relation source_assertion_refs must contain strings")?;
            valid_reference_for_kind(case, "assertion", reference)?;
        }
        if optional_string(relation, "relation_kind") == Some("contradicts") {
            contradiction_count += 1;
        }
        if optional_string(relation, "relation_kind") == Some("supersedes")
            && from_kind == "event"
        {
            supersession_edges.push((from_ref.to_owned(), to_ref.to_owned()));
        }
    }
    if graph_has_cycle(&supersession_edges) {
        return Err(format!("{case_id}: event supersession relation contains a cycle"));
    }
    Ok((refs.len(), contradiction_count))
}

fn analyze_case(case: &Value) -> Result<Value, String> {
    let (unlinked_revisions, integrity_mismatches) = validate_source_objects(case)?;
    let (available_assertions, future_assertions) = validate_assertions(case)?;
    let (available_events, future_events) = validate_events(case)?;
    let (relation_count, contradictions) = validate_relations(case)?;
    let source_count = array(case, "source_objects")?.len();
    let assertion_count = array(case, "assertions")?.len();
    let event_count = array(case, "events")?.len();

    let mut findings = BTreeSet::new();
    if future_assertions > 0 || future_events > 0 {
        findings.insert("future_evidence_not_available_at_frontier");
    }
    if unlinked_revisions > 0 {
        findings.insert("unlinked_source_revision_family");
    }
    if contradictions > 0 {
        findings.insert("contradictory_assertions_preserved");
    }
    if integrity_mismatches > 0 {
        findings.insert("source_integrity_mismatch_present");
    }

    Ok(json!({
        "case_id": string(case, "case_id")?,
        "source_object_count": source_count,
        "assertion_count": assertion_count,
        "available_assertion_count": available_assertions,
        "future_assertion_count": future_assertions,
        "event_count": event_count,
        "available_event_count": available_events,
        "future_event_count": future_events,
        "relation_count": relation_count,
        "unlinked_revision_family_count": unlinked_revisions,
        "contradiction_relation_count": contradictions,
        "integrity_mismatch_count": integrity_mismatches,
        "finding_codes": findings.into_iter().collect::<Vec<_>>()
    }))
}

fn build_report(candidates: &Value, candidate_digest: String) -> Result<Value, String> {
    let cases = array(candidates, "cases")?;
    let mut case_ids = BTreeSet::new();
    let mut case_results = Vec::new();
    let mut totals = BTreeMap::<String, usize>::new();

    for case in cases {
        let case_id = string(case, "case_id")?;
        if !case_ids.insert(case_id.to_owned()) {
            return Err(format!("duplicate event-graph case ID {case_id}"));
        }
        let result = analyze_case(case)?;
        for field in [
            "source_object_count",
            "assertion_count",
            "available_assertion_count",
            "future_assertion_count",
            "event_count",
            "available_event_count",
            "future_event_count",
            "relation_count",
            "unlinked_revision_family_count",
            "contradiction_relation_count",
            "integrity_mismatch_count",
        ] {
            *totals.entry(field.to_owned()).or_default() +=
                result[field].as_u64().unwrap_or(0) as usize;
        }
        case_results.push(result);
    }

    Ok(json!({
        "report_id": "MYCELIX-ECONOMIC-EVENT-CORE-S0-REPORT",
        "schema_version": "1.0.0",
        "claim_ceiling": "structural_and_temporal_graph_validation_not_accounting_policy_or_legal_conclusion",
        "candidate_sha256": candidate_digest,
        "case_count": case_results.len(),
        "totals": totals,
        "case_results": case_results
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
                    "Validates a bitemporal economic-event graph without accounting or legal conclusions.\n\n\
                     --data-dir PATH  Economic-event artifact directory (default: sibling economic-event-core-v1)\n\
                     --output PATH    Write validation report JSON to PATH\n\
                     --help           Show usage"
                );
                return Ok(());
            }
            other => return Err(format!("unknown argument: {other}; use --help").into()),
        }
    }

    let schema: Value = read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json"))?;
    let report_schema: Value = read_json(data_dir.join("economic_event_validation_report_v1.0.0.schema.json"))?;
    let candidate_path = data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json");
    let candidate_bytes = read_bytes(candidate_path)?;
    let candidates: Value = serde_json::from_slice(&candidate_bytes)?;
    validate_instance("economic-event graph", &schema, &candidates)?;
    let digest = hex::encode(Sha256::digest(&candidate_bytes));
    let report = build_report(&candidates, digest)?;
    validate_instance("economic-event validation report", &report_schema, &report)?;
    let output = serde_json::to_vec_pretty(&report)?;
    match output_path {
        Some(path) => {
            fs::write(path, &output)?;
            println!("Wrote economic-event validation report for {} cases.", report["case_count"]);
        }
        None => println!("{}", String::from_utf8(output)?),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixtures() -> (Value, Value) {
        let data_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../economic-event-core-v1");
        (
            read_json(data_dir.join("fixtures/economic_event_graph_candidates_v1.0.0.json")).unwrap(),
            read_json(data_dir.join("economic_event_graph_v1.0.0.schema.json")).unwrap(),
        )
    }


    #[test]
    fn invoice_event_amount_and_currency_must_match_its_source_assertion() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-001-INVOICE-WITHOUT-SERVICE-PROOF")).unwrap();
        case["events"][0]["amount_minor"] = Value::from(125001);
        assert!(validate_events(case).is_err());
    }

    #[test]
    fn invoice_event_currency_must_match_its_source_assertion() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-001-INVOICE-WITHOUT-SERVICE-PROOF")).unwrap();
        case["events"][0]["currency"] = Value::String("EUR".to_owned());
        assert!(validate_events(case).is_err());
    }

    #[test]
    fn service_delivery_occurred_date_must_match_acceptance_assertion() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-002-SERVICE-THEN-INVOICE")).unwrap();
        case["events"][0]["occurred_at"] = Value::String("2026-10-30T12:00:00Z".to_owned());
        assert!(validate_events(case).is_err());
    }

    #[test]
    fn settlement_event_amount_must_match_payment_assertion() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-003-SETTLEMENT-NOT-SECOND-EXPENSE")).unwrap();
        case["events"][1]["amount_minor"] = Value::from(97000);
        assert!(validate_events(case).is_err());
    }

    #[test]
    fn credit_note_correction_must_resolve_parent_in_same_tenant() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-004-CREDIT-NOTE-PARENT")).unwrap();
        case["source_objects"][0]["tenant_namespace"] = Value::String("tenant-other/ap".to_owned());
        assert!(validate_relations(case).is_err());
    }

    #[test]
    fn all_eight_graph_cases_pass_structural_and_temporal_validation() {
        let (bundle, schema) = fixtures();
        validate_instance("economic-event graph", &schema, &bundle).unwrap();
        let report = build_report(&bundle, "a".repeat(64)).unwrap();
        assert_eq!(report["case_count"], 8);
        assert_eq!(report["totals"]["source_object_count"], 16);
        assert_eq!(report["totals"]["assertion_count"], 21);
        assert_eq!(report["totals"]["event_count"], 13);
    }

    #[test]
    fn invoice_issuance_does_not_synthesize_service_delivery() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-001-INVOICE-WITHOUT-SERVICE-PROOF")).unwrap();
        assert_eq!(array(case, "events").unwrap().len(), 1);
        assert_eq!(optional_string(&array(case, "events").unwrap()[0], "event_kind"), Some("invoice_issued"));
    }

    #[test]
    fn settlement_is_a_separate_event_and_not_a_second_expense_event() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-003-SETTLEMENT-NOT-SECOND-EXPENSE")).unwrap();
        let kinds = array(case, "events").unwrap().iter()
            .map(|event| optional_string(event, "event_kind").unwrap())
            .collect::<BTreeSet<_>>();
        assert!(kinds.contains("invoice_issued"));
        assert!(kinds.contains("settlement_made"));
        assert!(!kinds.contains("accrual_recognized"));
        assert_eq!(array(case, "relations").unwrap()[0]["relation_kind"], "settles");
    }

    #[test]
    fn credit_note_is_explicitly_related_to_its_parent_invoice_event() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-004-CREDIT-NOTE-PARENT")).unwrap();
        let result = validate_relations(case).unwrap();
        assert_eq!(result.0, 1);
        assert_eq!(result.1, 0);
        assert_eq!(array(case, "relations").unwrap()[0]["relation_kind"], "corrects");
    }

    #[test]
    fn same_external_id_in_two_tenants_remains_two_source_objects() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-005-CROSS-TENANT-SAME-EXTERNAL-ID")).unwrap();
        assert_eq!(array(case, "source_objects").unwrap().len(), 2);
        assert_eq!(validate_source_objects(case).unwrap().0, 0);
    }

    #[test]
    fn future_received_source_and_assertion_are_not_available_at_prior_frontier() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-006-FUTURE-KNOWLEDGE-IS-RETAINED")).unwrap();
        assert_eq!(validate_assertions(case).unwrap(), (0, 1));
        assert_eq!(validate_events(case).unwrap(), (0, 1));
    }

    #[test]
    fn contradictory_assertions_are_preserved_as_a_relation() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-007-CONTRADICTORY-ASSERTIONS")).unwrap();
        assert_eq!(validate_relations(case).unwrap().1, 1);
        assert_eq!(array(case, "assertions").unwrap().len(), 2);
    }

    #[test]
    fn unlinked_revision_family_is_reported_not_silently_merged() {
        let (bundle, _) = fixtures();
        let case = array(&bundle, "cases").unwrap().iter()
            .find(|case| optional_string(case, "case_id") == Some("EVT-008-UNLINKED-SOURCE-REVISIONS")).unwrap();
        assert_eq!(validate_source_objects(case).unwrap().0, 1);
    }

    #[test]
    fn dangling_relation_endpoint_is_rejected() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-004-CREDIT-NOTE-PARENT")).unwrap();
        case["relations"][0]["to_ref"] = Value::String("EV-MISSING".to_owned());
        assert!(validate_relations(case).is_err());
    }

    #[test]
    fn source_supersession_cannot_cross_tenant_namespace() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-005-CROSS-TENANT-SAME-EXTERNAL-ID")).unwrap();
        case["source_objects"][1]["supersedes_source_ref"] = Value::String("SRC-005-A".to_owned());
        assert!(validate_source_objects(case).is_err());
    }

    #[test]
    fn event_supersession_cycle_is_rejected() {
        let (mut bundle, _) = fixtures();
        let case = bundle["cases"].as_array_mut().unwrap().iter_mut()
            .find(|case| optional_string(case, "case_id") == Some("EVT-004-CREDIT-NOTE-PARENT")).unwrap();
        case["relations"].as_array_mut().unwrap().push(json!({
            "relation_ref":"REL-CYCLE-1",
            "relation_kind":"supersedes",
            "from_kind":"event","from_ref":"EV-004-INVOICE",
            "to_kind":"event","to_ref":"EV-004-CREDIT",
            "recorded_at":"2026-11-05T10:00:00Z",
            "valid_from":"2026-10-01","valid_to":null,
            "source_assertion_refs":[]
        }));
        case["relations"][0]["relation_kind"] = Value::String("supersedes".to_owned());
        assert!(validate_relations(case).is_err());
    }
}
