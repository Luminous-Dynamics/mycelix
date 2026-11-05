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


fn source_snapshot_known(case: &Value) -> Result<bool, String> {
    let frontier = s(case, "requested_knowledge_at")?;
    let source = case.get("source_snapshot").ok_or("missing source snapshot")?;
    Ok(s(source, "observed_at")? <= frontier)
}

fn has_capability(case: &Value, capability_id: &str, status: &str) -> Result<bool, String> {
    if !source_snapshot_known(case)? { return Ok(false); }
    let frontier = s(case, "requested_knowledge_at")?;
    Ok(arr(case, "capability_coverage")?.iter().any(|item| {
        item.get("capability_id").and_then(Value::as_str) == Some(capability_id)
            && item.get("status").and_then(Value::as_str) == Some(status)
            && item.get("observed_at").and_then(Value::as_str).is_some_and(|at| at <= frontier)
    }))
}

fn unmapped_paths(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut paths = BTreeSet::new();
    if !source_snapshot_known(case)? { return Ok(paths); }
    let frontier = s(case, "requested_knowledge_at")?;
    for mapping in arr(case, "field_mappings")? {
        let known = mapping.get("recorded_at").and_then(Value::as_str).is_some_and(|at| at <= frontier);
        if known && matches!(mapping.get("mapping_status").and_then(Value::as_str),
            Some("unmapped" | "unsupported" | "ambiguous")) {
            paths.insert(s(mapping, "source_path")?.to_owned());
        }
    }
    Ok(paths)
}

fn incomplete_capabilities(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut capabilities = BTreeSet::new();
    if !source_snapshot_known(case)? { return Ok(capabilities); }
    let frontier = s(case, "requested_knowledge_at")?;
    for item in arr(case, "capability_coverage")? {
        let known = item.get("observed_at").and_then(Value::as_str).is_some_and(|at| at <= frontier);
        if known && s(item, "status")? != "supported" {
            capabilities.insert(s(item, "capability_id")?.to_owned());
        }
    }
    Ok(capabilities)
}

fn relationship_visible(case: &Value, relationship: &Value) -> Result<bool, String> {
    if !source_snapshot_known(case)? { return Ok(false); }
    let frontier = s(case, "requested_knowledge_at")?;
    let source = case.get("source_snapshot").ok_or("missing source snapshot")?;
    let observed_day = s(source, "observed_at")?.split('T').next().ok_or("invalid observed_at")?;
    let recorded_at = s(relationship, "recorded_at")?;
    let valid_from = s(relationship, "valid_from")?;
    let valid_to = relationship.get("valid_to").and_then(Value::as_str);
    Ok(recorded_at <= frontier && valid_from <= observed_day
        && valid_to.is_none_or(|end| end >= observed_day))
}

fn active_relationship_refs(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut refs = BTreeSet::new();
    for relationship in arr(case, "relationships")? {
        if relationship_visible(case, relationship)? {
            refs.insert(s(relationship, "relationship_ref")?.to_owned());
        }
    }
    Ok(refs)
}

fn future_evidence(case: &Value) -> Result<bool, String> {
    let frontier = s(case, "requested_knowledge_at")?;
    let source = case.get("source_snapshot").ok_or("missing source snapshot")?;
    if s(source, "observed_at")? > frontier { return Ok(true); }
    for capability in arr(case, "capability_coverage")? {
        if s(capability, "observed_at")? > frontier { return Ok(true); }
    }
    for mapping in arr(case, "field_mappings")? {
        if s(mapping, "recorded_at")? > frontier { return Ok(true); }
    }
    for relationship in arr(case, "relationships")? {
        if s(relationship, "recorded_at")? > frontier { return Ok(true); }
    }
    Ok(false)
}

fn access_principal_refs(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut refs = BTreeSet::new();
    for relationship in arr(case, "relationships")? {
        if relationship_visible(case, relationship)? {
            for reference in arr(relationship, "access_principal_refs")? {
                refs.insert(reference.as_str().ok_or("principal not string")?.to_owned());
            }
        }
    }
    Ok(refs)
}

fn upstream_origin_refs(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut refs = BTreeSet::new();
    for relationship in arr(case, "relationships")? {
        if relationship_visible(case, relationship)? {
            for reference in arr(relationship, "upstream_origin_refs")? {
                refs.insert(reference.as_str().ok_or("origin not string")?.to_owned());
            }
        }
    }
    Ok(refs)
}

fn duplicate_upstream_origin(case: &Value) -> Result<bool, String> {
    let mut counts = BTreeMap::<String, usize>::new();
    for relationship in arr(case, "relationships")? {
        if !relationship_visible(case, relationship)? { continue; }
        let origins = arr(relationship, "upstream_origin_refs")?
            .iter().filter_map(Value::as_str).collect::<BTreeSet<_>>();
        for origin in origins { *counts.entry(origin.to_owned()).or_default() += 1; }
    }
    Ok(counts.values().any(|count| *count > 1))
}

fn namespace_collision(case: &Value) -> Result<bool, String> {
    if !source_snapshot_known(case)? { return Ok(false); }
    let mut namespaces = BTreeMap::<String, String>::new();
    for object in arr(case, "objects")? {
        let external_id = s(object, "external_object_id")?;
        let namespace = s(object, "object_namespace")?;
        if let Some(previous) = namespaces.get(external_id) {
            if previous != namespace { return Ok(true); }
        } else {
            namespaces.insert(external_id.to_owned(), namespace.to_owned());
        }
    }
    Ok(false)
}

fn reason_codes(case: &Value) -> Result<BTreeSet<String>, String> {
    let mut reasons = BTreeSet::new();
    let unmapped = unmapped_paths(case)?;
    let future = future_evidence(case)?;
    let duplicate_origin = duplicate_upstream_origin(case)?;
    let collision = namespace_collision(case)?;

    if future { reasons.insert("evidence_after_knowledge_frontier".to_owned()); }
    if has_capability(case, "actor_access_principals", "unsupported")? && !unmapped.is_empty() {
        reasons.insert("actor_access_principal_not_represented".to_owned());
    }
    if has_capability(case, "personal_data_classification", "supported")?
        && (has_capability(case, "data_transfer_relationships", "unknown")?
            || has_capability(case, "data_transfer_relationships", "unsupported")?) {
        reasons.insert("classification_is_not_transfer_authority".to_owned());
    }
    if has_capability(case, "data_lineage", "partial")?
        || has_capability(case, "data_lineage", "unsupported")? {
        reasons.insert("source_lineage_partial_or_truncated".to_owned());
    }
    if has_capability(case, "journal_posting", "unsupported")? {
        reasons.insert("journal_posting_capability_not_authorised_by_adapter".to_owned());
    }
    if s(case.get("source_snapshot").ok_or("missing source snapshot")?, "capture_state")? == "truncated" {
        reasons.insert("export_pagination_incomplete".to_owned());
    }
    if duplicate_origin { reasons.insert("correlated_origin_not_independent".to_owned()); }
    if collision { reasons.insert("same_external_id_different_namespace".to_owned()); }
    if unmapped.is_empty() && incomplete_capabilities(case)?.is_empty()
        && s(case.get("source_snapshot").ok_or("missing source snapshot")?, "capture_state")? == "complete"
        && !future && !duplicate_origin && !collision {
        reasons.insert("no_unresolved_mapping_loss".to_owned());
    }
    Ok(reasons)
}

fn adapter_state(case: &Value) -> Result<&'static str, String> {
    let reasons = reason_codes(case)?;
    let source = case.get("source_snapshot").ok_or("missing source snapshot")?;
    if reasons.contains("evidence_after_knowledge_frontier")
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


fn validate_case_references(case: &Value) -> Result<(), String> {
    let case_id = s(case, "case_id")?;
    let mut object_types = BTreeMap::<String, String>::new();
    for object in arr(case, "objects")? {
        let reference = s(object, "object_ref")?.to_owned();
        let object_type = s(object, "object_type")?.to_owned();
        if object_types.insert(reference.clone(), object_type).is_some() {
            return Err(format!("{case_id}: duplicate object_ref {reference}"));
        }
    }

    let mut relationship_ids = BTreeSet::new();
    for relationship in arr(case, "relationships")? {
        let relationship_id = s(relationship, "relationship_ref")?;
        if !relationship_ids.insert(relationship_id.to_owned()) {
            return Err(format!("{case_id}: duplicate relationship_ref {relationship_id}"));
        }
        if s(relationship, "direction")? != "source_to_target" {
            return Err(format!("{case_id}/{relationship_id}: relationship direction is not source_to_target"));
        }
        for endpoint in ["source_object_ref", "target_object_ref"] {
            let reference = s(relationship, endpoint)?;
            if !object_types.contains_key(reference) {
                return Err(format!("{case_id}/{relationship_id}: unresolved {endpoint} {reference}"));
            }
        }
        if let Some(activity) = relationship.get("processing_activity_ref").and_then(Value::as_str) {
            if object_types.get(activity).map(String::as_str) != Some("processing_activity") {
                return Err(format!("{case_id}/{relationship_id}: invalid processing_activity_ref {activity}"));
            }
        }
        for (field, expected_type) in [("purpose_refs", "purpose"), ("data_element_refs", "data_element")] {
            for value in arr(relationship, field)? {
                let reference = value.as_str().ok_or_else(|| format!("{case_id}/{relationship_id}: non-string {field}"))?;
                if object_types.get(reference).map(String::as_str) != Some(expected_type) {
                    return Err(format!("{case_id}/{relationship_id}: {reference} is not a {expected_type}"));
                }
            }
        }
        for value in arr(relationship, "access_principal_refs")? {
            let reference = value.as_str().ok_or_else(|| format!("{case_id}/{relationship_id}: non-string access principal"))?;
            if !matches!(object_types.get(reference).map(String::as_str), Some("person" | "service_account" | "organization")) {
                return Err(format!("{case_id}/{relationship_id}: {reference} is not an actor/access principal"));
            }
        }
    }

    let mut mapping_paths = BTreeSet::new();
    for mapping in arr(case, "field_mappings")? {
        let source_path = s(mapping, "source_path")?;
        if !mapping_paths.insert(source_path.to_owned()) {
            return Err(format!("{case_id}: duplicate field mapping source path {source_path}"));
        }
        let status = s(mapping, "mapping_status")?;
        let canonical = mapping.get("canonical_path").and_then(Value::as_str);
        let canonical_is_null = mapping.get("canonical_path").is_some_and(Value::is_null);
        let transformation = mapping.get("transformation_ref").and_then(Value::as_str);
        let transformation_is_null = mapping.get("transformation_ref").is_some_and(Value::is_null);
        let loss = mapping.get("loss_code").and_then(Value::as_str);
        let loss_is_null = mapping.get("loss_code").is_some_and(Value::is_null);
        match status {
            "mapped_exact" if canonical.is_none() || !transformation_is_null || !loss_is_null => {
                return Err(format!("{case_id}: exact mapping {source_path} must have a canonical path and no transformation/loss code"));
            }
            "mapped_normalized" if canonical.is_none() || transformation.is_none() || !loss_is_null => {
                return Err(format!("{case_id}: normalized mapping {source_path} requires a canonical path and transformation ref, with no loss code"));
            }
            "unmapped" | "unsupported" | "ambiguous" if !canonical_is_null || loss.is_none() => {
                return Err(format!("{case_id}: unresolved field {source_path} must have null canonical path and a loss code"));
            }
            _ => {}
        }
    }

    let mut capability_keys = BTreeSet::new();
    for capability in arr(case, "capability_coverage")? {
        let id = s(capability, "capability_id")?;
        let scope = capability.get("scope_ref").and_then(Value::as_str).unwrap_or("<global>");
        if !capability_keys.insert((id.to_owned(), scope.to_owned())) {
            return Err(format!("{case_id}: duplicate capability observation for {id}/{scope}"));
        }
    }
    Ok(())
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
        validate_case_references(case)?;
        if !arr(case, "effects")?.is_empty() { return Err("adapter envelope contains an effect".into()); }
    }

    for exp in expected {
        let id = s(exp, "case_id")?;
        let case = cases.iter().find(|c| c.get("case_id").and_then(Value::as_str)==Some(id))
            .ok_or_else(|| format!("missing case {id}"))?;
        compare(id, "active relationship refs", active_relationship_refs(case)?, &exp["expected_active_relationship_refs"])?;
        compare(id, "unmapped source paths", unmapped_paths(case)?, &exp["expected_unmapped_source_paths"])?;
        compare(id, "incomplete capabilities", incomplete_capabilities(case)?, &exp["expected_incomplete_capabilities"])?;
        compare(id, "access principal refs", access_principal_refs(case)?, &exp["expected_access_principal_refs"])?;
        compare(id, "upstream origin refs", upstream_origin_refs(case)?, &exp["expected_upstream_origin_refs"])?;
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
    fn future_recorded_relationship_is_excluded_at_earlier_frontier() {
        let (c, _) = fixtures();
        let case = arr(&c, "cases").unwrap().iter()
            .find(|x| s(x, "case_id").unwrap()=="TEMP-FUTURE-FACT").unwrap();
        assert!(active_relationship_refs(case).unwrap().is_empty());
        assert!(upstream_origin_refs(case).unwrap().is_empty());
        assert_eq!(adapter_state(case).unwrap(), "indeterminate");
        assert!(reason_codes(case).unwrap().contains("evidence_after_knowledge_frontier"));
    }

    #[test]
    fn future_capability_observation_is_not_visible_at_earlier_frontier() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap()=="BLACKLINE-RECON-NO-POSTING").unwrap();
        let index = arr(case, "capability_coverage").unwrap().iter()
            .position(|x| s(x, "capability_id").unwrap()=="journal_posting").unwrap();
        case["capability_coverage"][index]["observed_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        assert!(!incomplete_capabilities(case).unwrap().contains("journal_posting"));
        assert_eq!(adapter_state(case).unwrap(), "indeterminate");
    }

    #[test]
    fn future_field_mapping_is_not_visible_at_earlier_frontier() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap()=="BIGID-CLASSIFICATION-NOT-AUTHORITY").unwrap();
        let index = arr(case, "field_mappings").unwrap().len()-1;
        case["field_mappings"][index]["recorded_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        assert!(unmapped_paths(case).unwrap().is_empty());
        assert_eq!(adapter_state(case).unwrap(), "indeterminate");
    }

    #[test]
    fn future_source_snapshot_excludes_all_snapshot_relationships() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap()=="WORKIVA-DISCLOSURE-LINK").unwrap();
        case["source_snapshot"]["observed_at"] = Value::String("2026-11-05T10:45:00Z".to_owned());
        assert!(active_relationship_refs(case).unwrap().is_empty());
        assert!(upstream_origin_refs(case).unwrap().is_empty());
        assert_eq!(adapter_state(case).unwrap(), "indeterminate");
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
    fn invalid_purpose_reference_is_rejected() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "OT-DIRECTION-SUBPROCESSOR").unwrap();
        case["relationships"][0]["purpose_refs"][0] = Value::String("MISSING-PURPOSE".to_owned());
        assert!(validate_case_references(case).is_err());
    }

    #[test]
    fn dataset_cannot_be_used_as_an_access_principal() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "OT-DIRECTION-SUBPROCESSOR").unwrap();
        case["relationships"][0]["access_principal_refs"][0] = Value::String("OBJ-ELEMENT-EMAIL".to_owned());
        assert!(validate_case_references(case).is_err());
    }

    #[test]
    fn duplicate_object_refs_are_rejected() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "NAMESPACE-SCOPED-IDENTITY").unwrap();
        case["objects"][1]["object_ref"] = Value::String("OBJ-ORG-SA".to_owned());
        assert!(validate_case_references(case).is_err());
    }

    #[test]
    fn unresolved_mapping_without_loss_code_is_rejected() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "OT-ACCESS-PRINCIPAL-GAP").unwrap();
        case["field_mappings"][0]["loss_code"] = Value::Null;
        assert!(validate_case_references(case).is_err());
    }


    #[test]
    fn normalized_mapping_requires_transformation_reference() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "BIGID-CLASSIFICATION-NOT-AUTHORITY").unwrap();
        case["field_mappings"][0]["transformation_ref"] = Value::Null;
        assert!(validate_case_references(case).is_err());
    }

    #[test]
    fn successful_mapping_cannot_carry_a_loss_code() {
        let (mut c, _) = fixtures();
        let case = arr(&mut c, "cases").unwrap().iter_mut()
            .find(|x| s(x, "case_id").unwrap() == "OT-DIRECTION-SUBPROCESSOR").unwrap();
        case["field_mappings"][0]["loss_code"] = Value::String("silent_loss".to_owned());
        assert!(validate_case_references(case).is_err());
    }

    #[test]
    fn no_adapter_fixture_contains_an_external_effect() {
        let (c, _) = fixtures();
        assert!(arr(&c, "cases").unwrap().iter().all(|x| arr(x, "effects").unwrap().is_empty()));
    }
}
