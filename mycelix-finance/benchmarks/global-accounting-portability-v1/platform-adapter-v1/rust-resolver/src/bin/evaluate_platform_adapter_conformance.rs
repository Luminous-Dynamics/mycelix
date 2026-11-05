use jsonschema::draft202012;
use serde_json::Value;
use std::{collections::BTreeSet, env, error::Error, fs, path::PathBuf};

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
    arr(values, "unused").unwrap_or(&[]);
    values.as_array().ok_or("expected array")?.iter()
        .map(|v| v.as_str().map(str::to_owned).ok_or("expected string".to_owned()))
        .collect()
}

fn compare(case_id: &str, name: &str, actual: BTreeSet<String>, expected: &Value) -> Result<(), String> {
    let expected = set(expected)?;
    if actual == expected { Ok(()) }
    else { Err(format!("{case_id}: {name} expected {expected:?}, got {actual:?}")) }
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
    fn no_adapter_fixture_contains_an_external_effect() {
        let (c, _) = fixtures();
        assert!(arr(&c, "cases").unwrap().iter().all(|x| arr(x, "effects").unwrap().is_empty()));
    }
}
