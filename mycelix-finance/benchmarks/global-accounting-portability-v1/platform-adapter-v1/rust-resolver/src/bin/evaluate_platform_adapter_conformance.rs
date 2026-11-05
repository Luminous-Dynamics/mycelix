use jsonschema::draft202012;
use serde_json::Value;
use std::{collections::{BTreeMap, BTreeSet}, env, error::Error, fs, path::PathBuf};

fn main() {
    if let Err(error) = run() {
        eprintln!("platform-adapter-conformance: {error}");
        std::process::exit(2);
    }
}

fn read(path: PathBuf) -> Result<Value, Box<dyn Error>> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn validate(label: &str, schema: &Value, value: &Value) -> Result<(), String> {
    draft202012::meta::validate(schema).map_err(|e| format!("{label}: invalid schema: {e}"))?;
    let validator = draft202012::options().should_validate_formats(true)
        .build(schema).map_err(|e| format!("{label}: {e}"))?;
    let errors = validator.iter_errors(value)
        .map(|e| format!("{} at {}", e, e.instance_path())).collect::<Vec<_>>();
    if errors.is_empty() { Ok(()) } else { Err(format!("{label}: {}", errors.join("; "))) }
}

fn arr<'a>(v: &'a Value, key: &str) -> Result<&'a [Value], String> {
    v.get(key).and_then(Value::as_array).map(Vec::as_slice)
        .ok_or_else(|| format!("missing array {key}"))
}

fn s<'a>(v: &'a Value, key: &str) -> Result<&'a str, String> {
    v.get(key).and_then(Value::as_str).ok_or_else(|| format!("missing string {key}"))
}

fn set(values: &Value) -> Result<BTreeSet<String>, String> {
    values.as_array().ok_or("expected array")?.iter()
        .map(|v| v.as_str().map(str::to_owned).ok_or("expected string".to_owned()))
        .collect()
}

fn compare(case_id: &str, name: &str, actual: BTreeSet<String>, expected: &Value) -> Result<(), String> {
    let expected = set(expected)?;
    if actual == expected { Ok(()) }
    else { Err(format!("{case_id}: {name} expected {expected:?}, got {actual:?}")) }
}


fn has_capability(case: &Value, capability_id: &str, status: &str) -> Result<bool, String> {
    Ok(arr(case, "capability_coverage")?.iter().any(|item| {
        item.get("capability_id").and_then(Value::as_str) == Some(capability_id)
            && item.get("status").and_then(Value::as_str) == Some(status)
    }))
}

fn unmapped_paths(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut paths = BTreeSet::new();
    for mapping in arr(case, "field_mappings")? {
        if matches!(
            mapping.get("mapping_status").and_then(Value::as_str),
            Some("unmapped" | "unsupported" | "ambiguous")
        ) {
            paths.insert(s(mapping, "source_path")?.to_owned());
        }
    }
    Ok(paths)
}

fn incomplete_capabilities(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut capabilities = BTreeSet::new();
    for item in arr(case, "capability_coverage")? {
        if s(item, "status")? != "supported" {
            capabilities.insert(s(item, "capability_id")?.to_owned());
        }
    }
    Ok(capabilities)
}

fn future_relationship(case: &Value) -> Result<bool, String> {
    let frontier = s(case, "requested_knowledge_at")?;
    Ok(arr(case, "relationships")?.iter().any(|relationship| {
        relationship.get("recorded_at").and_then(Value::as_str)
            .is_some_and(|recorded| recorded > frontier)
    }))
}

fn duplicate_upstream_origin(case: &Value) -> Result<bool, String> {
    let mut counts = BTreeMap::<String, usize>::new();
    for relationship in arr(case, "relationships")? {
        let origins = arr(relationship, "upstream_origin_refs")?
            .iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
        for origin in origins {
            *counts.entry(origin.to_owned()).or_default() += 1;
        }
    }
    Ok(counts.values().any(|count| *count > 1))
}

fn namespace_collision(case: &Value) -> Result<bool, String> {
    let mut namespaces = BTreeMap::<String, String>::new();
    for object in arr(case, "objects")? {
        let external_id = s(object, "external_object_id")?;
        let namespace = s(object, "object_namespace")?;
        if let Some(previous) = namespaces.get(external_id) {
            if previous != namespace {
                return Ok(true);
            }
        } else {
            namespaces.insert(external_id.to_owned(), namespace.to_owned());
        }
    }
    Ok(false)
}

fn reason_codes(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut reasons = BTreeSet::new();
    let unmapped = unmapped_paths(case)?;
    let future = future_relationship(case)?;
    let duplicate_origin = duplicate_upstream_origin(case)?;
    let collision = namespace_collision(case)?;

    if has_capability(case, "actor_access_principals", "unsupported")? && !unmapped.is_empty() {
        reasons.insert("actor_access_principal_not_represented".to_owned());
    }
    if has_capability(case, "personal_data_classification", "supported")?
        && (has_capability(case, "data_transfer_relationships", "unknown")?
            || has_capability(case, "data_transfer_relationships", "unsupported")?)
    {
        reasons.insert("classification_is_not_transfer_authority".to_owned());
    }
    if has_capability(case, "data_lineage", "partial")?
        || has_capability(case, "data_lineage", "unsupported")?
    {
        reasons.insert("source_lineage_partial_or_truncated".to_owned());
    }
    if has_capability(case, "journal_posting", "unsupported")? {
        reasons.insert("journal_posting_capability_not_authorised_by_adapter".to_owned());
    }
    if future {
        reasons.insert("future_recorded_relationship_excluded".to_owned());
    }
    if s(case.get("source_snapshot").ok_or("missing source snapshot")?, "capture_state")? == "truncated" {
        reasons.insert("export_pagination_incomplete".to_owned());
    }
    if duplicate_origin {
        reasons.insert("correlated_origin_not_independent".to_owned());
    }
    if collision {
        reasons.insert("same_external_id_different_namespace".to_owned());
    }
    if unmapped.is_empty()
        && incomplete_capabilities(case)?.is_empty()
        && s(case.get("source_snapshot").ok_or("missing source snapshot")?, "capture_state")? == "complete"
        && !future && !duplicate_origin && !collision
    {
        reasons.insert("no_unresolved_mapping_loss".to_owned());
    }
    Ok(reasons)
}

fn adapter_state(case: &Value) -> Result<&'static str, String> {
    let reasons = reason_codes(case)?;
    let source = case.get("source_snapshot").ok_or("missing source snapshot")?;
    if reasons.contains("future_recorded_relationship_excluded")
        || reasons.contains("actor_access_principal_not_represented")
        || reasons.contains("classification_is_not_transfer_authority")
    {
        return Ok("indeterminate");
    }
    if reasons.contains("correlated_origin_not_independent") {
        return Ok("duplicate_origin_review");
    }
    if reasons.contains("same_external_id_different_namespace") {
        return Ok("identity_scope_preserved");
    }
    if has_capability(case, "journal_posting", "unsupported")?
        && has_capability(case, "journal_reconciliation", "supported")?
    {
        return Ok("evidence_only_read_only");
    }
    if s(source, "capture_state")? != "complete"
        || !incomplete_capabilities(case)?.is_empty()
        || !unmapped_paths(case)?.is_empty()
    {
        return Ok("mapping_partial");
    }
    Ok("mapping_complete")
}

fn run() -> Result<(), Box<dyn Error>> {
    let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../platform-adapter-v1");
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--data-dir" => root = PathBuf::from(args.next().ok_or("--data-dir requires a path")?),
            "--help" | "-h" => { println!("Validates synthetic adapter envelopes and the public mapping oracle; no vendor API or write-back."); return Ok(()); },
            other => return Err(format!("unknown argument: {other}").into())
        }
    }
    let schema = read(root.join("finance_platform_adapter_envelope_v1.0.0.schema.json"))?;
    let bundle_schema = read(root.join("adapter_conformance_bundle_v1.0.0.schema.json"))?;
    let oracle_schema = read(root.join("adapter_conformance_oracle_v1.0.0.schema.json"))?;
    let bundle = read(root.join("fixtures/adapter_conformance_candidates_v1.0.0.json"))?;
    let oracle = read(root.join("fixtures/adapter_conformance_oracle_v1.0.0.json"))?;
    validate("bundle", &bundle_schema, &bundle)?;
    validate("oracle", &oracle_schema, &oracle)?;

    let cases = arr(&bundle, "cases")?;
    let expected = arr(&oracle, "expectations")?;
    if cases.len() != expected.len() { return Err("candidate/oracle count mismatch".into()); }
    let mut ids = BTreeSet::new();
    for case in cases {
        validate("envelope", &schema, case)?;
        if !ids.insert(s(case, "case_id")?.to_owned()) { return Err("duplicate case id".into()); }
        let objects = arr(case, "objects")?.iter().map(|o| s(o, "object_ref").map(str::to_owned))
            .collect::<Result<BTreeSet<_>, _>>()?;
        for rel in arr(case, "relationships")? {
            for key in ["source_object_ref", "target_object_ref"] {
                if !objects.contains(s(rel, key)?) { return Err(format!("{}: unresolved {key}", s(case, "case_id")?)); }
            }
            if s(rel, "direction")? != "source_to_target" { return Err("relationship direction changed".into()); }
        }
        if !arr(case, "effects")?.is_empty() { return Err("adapter envelope contains an effect".into()); }
    }

    for exp in expected {
        let id = s(exp, "case_id")?;
        let case = cases.iter().find(|c| c.get("case_id").and_then(Value::as_str)==Some(id))
            .ok_or_else(|| format!("missing case {id}"))?;
        let frontier = s(case, "requested_knowledge_at")?;
        let observed = s(case.get("source_snapshot").ok_or("missing source snapshot")?, "observed_at")?;
        let day = observed.split('T').next().ok_or("invalid observed_at")?;
        let active = arr(case, "relationships")?.iter().filter(|r| {
            let recorded = r.get("recorded_at").and_then(Value::as_str).unwrap_or("");
            let start = r.get("valid_from").and_then(Value::as_str).unwrap_or("9999");
            let end = r.get("valid_to").and_then(Value::as_str);
            recorded <= frontier && start <= day && end.is_none_or(|x| x >= day)
        }).map(|r| s(r, "relationship_ref").unwrap().to_owned()).collect();
        compare(id, "active relationship refs", active, &exp["expected_active_relationship_refs"])?;

        let mut paths = BTreeSet::new();
        for m in arr(case, "field_mappings")? {
            if matches!(m.get("mapping_status").and_then(Value::as_str), Some("unmapped"|"unsupported"|"ambiguous")) {
                paths.insert(s(m, "source_path")?.to_owned());
            }
        }
        compare(id, "unmapped source paths", paths, &exp["expected_unmapped_source_paths"])?;

        let mut capabilities = BTreeSet::new();
        for c in arr(case, "capability_coverage")? {
            if s(c, "status")? != "supported" { capabilities.insert(s(c, "capability_id")?.to_owned()); }
        }
        compare(id, "incomplete capabilities", capabilities, &exp["expected_incomplete_capabilities"])?;

        let mut principals = BTreeSet::new();
        let mut origins = BTreeSet::new();
        for r in arr(case, "relationships")? {
            for x in arr(r, "access_principal_refs")? { principals.insert(x.as_str().ok_or("principal not string")?.to_owned()); }
            for x in arr(r, "upstream_origin_refs")? { origins.insert(x.as_str().ok_or("origin not string")?.to_owned()); }
        }
        compare(id, "access principal refs", principals, &exp["expected_access_principal_refs"])?;
        compare(id, "upstream origin refs", origins, &exp["expected_upstream_origin_refs"])?;
        compare(id, "reason codes", reason_codes(case)?, &exp["expected_reason_codes"])?;
        let actual_state = adapter_state(case)?;
        if s(exp, "expected_adapter_state")? != actual_state {
            return Err(format!("{id}: expected adapter state {}, got {actual_state}", s(exp, "expected_adapter_state")?));
        }
    }
    println!("PASS: {} synthetic envelopes validated and compared against the public source-mapping oracle; no vendor system was called or modified.", cases.len());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixtures() -> (Value, Value) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../platform-adapter-v1/fixtures");
        (read(root.join("adapter_conformance_candidates_v1.0.0.json")).unwrap(),
         read(root.join("adapter_conformance_oracle_v1.0.0.json")).unwrap())
    }
    #[test]
    fn ten_synthetic_cases_are_pinned_to_ten_oracle_rows() {
        let (c, o) = fixtures();
        assert_eq!(arr(&c, "cases").unwrap().len(), 10);
        assert_eq!(arr(&o, "expectations").unwrap().len(), 10);
    }
    #[test]
    fn future_recorded_fact_is_excluded_at_earlier_frontier() {
        let (c, _) = fixtures();
        let case = arr(&c, "cases").unwrap().iter().find(|x| s(x, "case_id").unwrap()=="TEMP-FUTURE-FACT").unwrap();
        let frontier = s(case, "requested_knowledge_at").unwrap();
        assert!(arr(case, "relationships").unwrap().iter().all(|r| s(r, "recorded_at").unwrap() <= frontier));
    }

    #[test]
    fn derived_adapter_states_and_reason_codes_match_every_oracle_row() {
        let (candidates, oracle) = fixtures();
        for expectation in arr(&oracle, "expectations").unwrap() {
            let id = s(expectation, "case_id").unwrap();
            let candidate = arr(&candidates, "cases").unwrap().iter()
                .find(|case| case.get("case_id").and_then(Value::as_str) == Some(id)).unwrap();
            compare(id, "reason codes", reason_codes(candidate).unwrap(), &expectation["expected_reason_codes"]).unwrap();
            assert_eq!(adapter_state(candidate).unwrap(), s(expectation, "expected_adapter_state").unwrap());
        }
    }

    #[test]
    fn no_adapter_fixture_contains_an_external_effect() {
        let (c, _) = fixtures();
        assert!(arr(&c, "cases").unwrap().iter().all(|x| arr(x, "effects").unwrap().is_empty()));
    }
}
