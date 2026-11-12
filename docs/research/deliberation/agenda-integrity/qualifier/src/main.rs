use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{collections::{BTreeMap, BTreeSet}, env, fs, process::ExitCode};

const PROTOCOL_ID: &str = "DELIB-AGENDA-001";
const PROTOCOL_SCHEMA_ID: &str = "https://luminousdynamics.io/schemas/delib-agenda-protocol-v0.1.schema.json";
const FIXTURE_SCHEMA_ID: &str = "https://luminousdynamics.io/schemas/delib-agenda-fixtures-v0.1.schema.json";
const FIXTURE_COUNT: usize = 18;

#[derive(Default)]
struct Report {
    fixtures: usize,
    passed: usize,
    rejected: usize,
    indeterminate: usize,
    source_digest_count: usize,
    agenda_digest_count: usize,
    required_options: usize,
    represented_required_options: usize,
    required_constraints: usize,
    preserved_constraints: usize,
}

fn arr<'a>(v: &'a Value, k: &str) -> &'a [Value] {
    v.get(k).and_then(Value::as_array).map(Vec::as_slice).unwrap_or(&[])
}
fn s<'a>(v: &'a Value, k: &str) -> Option<&'a str> {
    v.get(k)?.as_str().filter(|x| !x.trim().is_empty())
}
fn b(v: &Value, k: &str) -> bool { v.get(k).and_then(Value::as_bool).unwrap_or(false) }
fn set(v: &Value) -> BTreeSet<String> {
    v.as_array().into_iter().flatten().filter_map(Value::as_str).map(str::to_owned).collect()
}
fn digest(v: &Value) -> Result<String, String> {
    let bytes = serde_json::to_vec(v).map_err(|e| e.to_string())?;
    Ok(format!("{:x}", Sha256::digest(bytes)))
}
fn schema_header(schema: &Value, id: &str, required: &[&str]) -> Vec<String> {
    let mut errors = Vec::new();
    if schema["$schema"].as_str() != Some("https://json-schema.org/draft/2020-12/schema") {
        errors.push("schema dialect must be draft 2020-12".to_string());
    }
    if schema["$id"].as_str() != Some(id) { errors.push("schema $id mismatch".to_string()); }
    if schema["type"].as_str() != Some("object") { errors.push("schema root must be object".to_string()); }
    if schema["additionalProperties"].as_bool() != Some(false) { errors.push("schema root must reject additional properties".to_string()); }
    let got = set(&schema["required"]);
    for field in required { if !got.contains(*field) { errors.push(format!("schema required field missing: {field}")); } }
    errors
}
fn validate_fixture(f: &Value) -> (Vec<String>, usize, usize, usize, usize) {
    let mut errors = Vec::new();
    let source = arr(f, "source_options");
    let presented = arr(f, "presented_options");
    let complete = b(f, "source_registry_complete");
    let cutoff = f["decision_cutoff_sequence"].as_u64().unwrap_or(0) as usize;
    let task_class = s(&f["task"], "task_class").unwrap_or("");

    let mut source_ids = BTreeSet::new();
    let mut required_ids = BTreeSet::new();
    let mut required_constraints = BTreeSet::new();
    let mut source_constraints_by_id: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for option in source {
        let id = s(option, "option_id").unwrap_or("");
        if id.is_empty() || !source_ids.insert(id.to_owned()) { errors.push("source option IDs must be unique and non-empty".into()); }
        if s(option, "origin_id").is_none() { errors.push(format!("source option {id} lacks origin provenance")); }
        let status = s(option, "status").unwrap_or("");
        if !["admitted", "candidate", "rejected"].contains(&status) {
            errors.push(format!("source option {id} has unknown admission status"));
        }
        if b(option, "required_in_agenda") {
            if status != "admitted" { errors.push(format!("required option {id} is not admitted")); }
            required_ids.insert(id.to_owned());
        }
        let mut option_constraints = BTreeSet::new();
        for c in arr(option, "constraints") {
            if let Some(c) = c.as_str() {
                option_constraints.insert(c.to_owned());
            } else { errors.push(format!("source option {id} has non-string constraint ID")); }
        }
        source_constraints_by_id.insert(id.to_owned(), option_constraints);
    }
    if source.is_empty() { errors.push("source option registry is empty".into()); }

    let mut presented_ids = BTreeSet::new();
    let mut represented_sources = BTreeSet::new();
    let mut preserved_constraints = BTreeSet::new();
    let mut preserved_constraints_by_source: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut appearances: BTreeMap<String, Vec<&Value>> = BTreeMap::new();

    for item in presented {
        let id = s(item, "option_id").unwrap_or("");
        if id.is_empty() || !presented_ids.insert(id.to_owned()) { errors.push("presented option IDs must be unique and non-empty".into()); }
        if s(item, "origin_status") != Some("source_recorded") { errors.push(format!("presented option {id} lacks admitted source provenance")); }
        let refs = set(&item["source_option_ids"]);
        if refs.is_empty() { errors.push(format!("presented option {id} has no source option reference")); }
        for source_id in &refs {
            if !source_ids.contains(source_id) {
                errors.push(format!("presented option {id} references unknown source option {source_id}"));
            } else if !source.iter().any(|source_option|
                source_option["option_id"].as_str() == Some(source_id.as_str())
                && source_option["status"].as_str() == Some("admitted")) {
                errors.push(format!("presented option {id} references non-admitted source option {source_id}"));
            }
            represented_sources.insert(source_id.clone());
            appearances.entry(source_id.clone()).or_default().push(item);
        }
        let item_constraints: BTreeSet<String> = arr(item, "preserved_constraint_ids")
            .iter().filter_map(Value::as_str).map(str::to_owned).collect();
        for c in &item_constraints { preserved_constraints.insert(c.clone()); }
        let allowed_for_item: BTreeSet<String> = refs.iter()
            .flat_map(|source_id| source_constraints_by_id.get(source_id).into_iter().flatten().cloned())
            .collect();
        for c in &item_constraints {
            if !allowed_for_item.contains(c) {
                errors.push(format!("presented option {id} carries constraint {c} that belongs to none of its source options"));
            }
        }
        for source_id in &refs {
            let allowed = source_constraints_by_id.get(source_id).cloned().unwrap_or_default();
            preserved_constraints_by_source.entry(source_id.clone()).or_default()
                .extend(item_constraints.intersection(&allowed).cloned());
        }
        if b(item, "is_default") {
            if !b(item, "default_declared") || s(item, "default_authority").is_none() {
                errors.push(format!("presented option {id} uses an undeclared or unauthorized default"));
            }
        } else if b(item, "default_declared") || !item["default_authority"].is_null() {
            errors.push(format!("presented option {id} declares default authority while not marked default"));
        }
        let seq = item["introduced_sequence"].as_u64().unwrap_or(0) as usize;
        if seq > cutoff && !(b(item, "late_mutation_authorized") && s(item, "late_mutation_authority").is_some()) {
            errors.push(format!("presented option {id} was introduced after the frozen agenda cutoff without an authority reference"));
        }
    }

    for source_id in &required_ids {
        if !represented_sources.contains(source_id) {
            errors.push(format!("required source option {source_id} is missing from presented agenda"));
        }
    }
    for (source_id, constraints) in &source_constraints_by_id {
        if !required_ids.contains(source_id) && !represented_sources.contains(source_id) { continue; }
        required_constraints.extend(constraints.iter().cloned());
        let retained = preserved_constraints_by_source.get(source_id).cloned().unwrap_or_default();
        for constraint in constraints {
            if !retained.contains(constraint) {
                errors.push(format!("required constraint {constraint} for source option {source_id} is absent or attributed to another option"));
            }
        }
    }

    for (source_id, items) in &appearances {
        if items.len() > 1 {
            let group = s(items[0], "split_group_id");
            let disclosed = items.iter().all(|item|
                s(item, "representation_kind") == Some("split")
                && group.is_some()
                && s(item, "split_group_id") == group
                && b(item, "split_disclosed"));
            if !disclosed {
                errors.push(format!("source option {source_id} is split across entries without one disclosed split group"));
            }
        }
    }
    for item in presented {
        let refs = set(&item["source_option_ids"]);
        match s(item, "representation_kind").unwrap_or("") {
            "single" if refs.len() != 1 => errors.push("single representation must reference exactly one source option".into()),
            "bundle" if refs.len() < 2 => errors.push("bundle must reference at least two source options".into()),
            "split" if s(item, "split_group_id").is_none() => errors.push("split representation requires split_group_id".into()),
            "single" | "bundle" | "split" => {},
            _ => errors.push("unknown representation_kind".into()),
        }
    }

    if task_class == "normative_pluralism" {
        if !f["task"]["gold_option"].is_null() { errors.push("normative-pluralism task must not define a gold policy option".into()); }
        if s(&f["agenda_intervention"], "semantic_relation") != Some("normative_pluralism") {
            errors.push("normative-pluralism task must declare normative_pluralism semantic relation".into());
        }
    } else if task_class == "verifiable" {
        let gold = s(&f["task"], "gold_option").unwrap_or("");
        if !source_ids.contains(gold) { errors.push("verifiable task gold option must be in source registry".into()); }
    } else {
        errors.push("unknown task_class".into());
    }

    let relation = s(&f["agenda_intervention"], "semantic_relation").unwrap_or("");
    if !["equivalent", "disclosed_tradeoff", "invalid_mutation", "normative_pluralism"].contains(&relation) {
        errors.push("unknown semantic_relation".into());
    }
    if relation == "disclosed_tradeoff" && !b(&f["agenda_intervention"], "tradeoff_disclosed") {
        errors.push("material trade-off must be explicitly disclosed".into());
    }
    if relation == "equivalent" && b(&f["agenda_intervention"], "tradeoff_disclosed") {
        errors.push("equivalent framing cannot simultaneously declare a material trade-off".into());
    }
    if (task_class == "normative_pluralism") != (relation == "normative_pluralism") {
        errors.push("normative-pluralism task class and semantic relation must agree".into());
    }
    if relation == "invalid_mutation" && errors.is_empty() {
        errors.push("agenda mutation is declared invalid but no structural invariant detects it".into());
    }

    if let Some(pair) = s(&f["agenda_intervention"], "matched_pair_id") {
        if s(&f["agenda_intervention"], "matched_material_id").is_none() {
            errors.push(format!("matched pair {pair} lacks matched_material_id"));
        }
    }
    if !complete && errors.is_empty() {
        return (Vec::new(), required_ids.len(), required_ids.intersection(&represented_sources).count(),
            required_constraints.len(), required_constraints.intersection(&preserved_constraints).count());
    }
    (errors, required_ids.len(), required_ids.intersection(&represented_sources).count(),
        required_constraints.len(), required_constraints.intersection(&preserved_constraints).count())
}
fn run(args: &[String]) -> Result<Report, String> {
    if args.len() != 4 { return Err("usage: qualifier PROTOCOL PROTOCOL_SCHEMA FIXTURES FIXTURES_SCHEMA".into()); }
    let protocol: Value = serde_json::from_str(&fs::read_to_string(&args[0]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let ps: Value = serde_json::from_str(&fs::read_to_string(&args[1]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let fixtures_doc: Value = serde_json::from_str(&fs::read_to_string(&args[2]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    let fschema: Value = serde_json::from_str(&fs::read_to_string(&args[3]).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;

    let mut se = schema_header(&ps, PROTOCOL_SCHEMA_ID, &["protocol_id","version","status","title","owning_issue","parent_issue","composition","semantic_relations","task_classes","transformation_families","metric_ids","qualification_gates","claim_ceiling"]);
    se.extend(schema_header(&fschema, FIXTURE_SCHEMA_ID, &["schema_version","status","protocol_id","fixture_count","fixtures"]));
    if !se.is_empty() { return Err(se.join("; ")); }
    if s(&protocol, "protocol_id") != Some(PROTOCOL_ID) || s(&protocol, "version") != Some("0.1.0")
        || s(&protocol, "status") != Some("research-proposal") { return Err("protocol identity/version/status mismatch".into()); }
    if s(&fixtures_doc, "protocol_id") != Some(PROTOCOL_ID)
        || s(&fixtures_doc, "schema_version") != Some("delib-agenda-fixtures-v0.1")
        || s(&fixtures_doc, "status") != Some("synthetic_fixture_design_unexecuted") { return Err("fixture document identity/status mismatch".into()); }
    let fixtures = arr(&fixtures_doc, "fixtures");
    if fixtures.len() != FIXTURE_COUNT || fixtures_doc["fixture_count"].as_u64() != Some(FIXTURE_COUNT as u64) {
        return Err("fixture count does not match frozen v0.1 corpus".into());
    }

    let mut report = Report { fixtures: fixtures.len(), ..Report::default() };
    let mut ids = BTreeSet::new();
    let mut pairs: BTreeMap<String, Vec<&Value>> = BTreeMap::new();
    for f in fixtures {
        let id = s(f, "fixture_id").unwrap_or("<missing>");
        if !ids.insert(id.to_owned()) { return Err(format!("duplicate fixture ID {id}")); }
        let (errors, req, represented, constraints, preserved) = validate_fixture(f);
        report.required_options += req;
        report.represented_required_options += represented;
        report.required_constraints += constraints;
        report.preserved_constraints += preserved;
        let complete = b(f, "source_registry_complete");
        let expected = s(&f["expected"], "disposition").unwrap_or("");
        let computed = if !errors.is_empty() { "reject" } else if !complete { "indeterminate" } else { "pass" };
        if expected != computed {
            return Err(format!("{id}: expected disposition {expected}, computed {computed}: {}", errors.join("; ")));
        }
        match computed {
            "pass" => report.passed += 1,
            "reject" => report.rejected += 1,
            "indeterminate" => report.indeterminate += 1,
            _ => return Err(format!("{id}: invalid computed disposition")),
        }
        if let Some(pair) = s(&f["agenda_intervention"], "matched_pair_id") {
            pairs.entry(pair.to_owned()).or_default().push(f);
        }
        println!("{id}: disposition={computed}; source_sha256={}; agenda_sha256={}",
            digest(&f["source_options"])?, digest(&f["presented_options"])?);
        report.source_digest_count += 1;
        report.agenda_digest_count += 1;
    }
    for (pair_id, members) in pairs {
        if members.len() != 2 { return Err(format!("matched pair {pair_id} must contain exactly two fixtures")); }
        let material_a = s(&members[0]["agenda_intervention"], "matched_material_id");
        let material_b = s(&members[1]["agenda_intervention"], "matched_material_id");
        if material_a.is_none() || material_a != material_b { return Err(format!("matched pair {pair_id} has mismatched material identity")); }
        if members[0]["source_options"] != members[1]["source_options"] {
            return Err(format!("matched pair {pair_id} does not hold source option universe constant"));
        }
        if members[0]["task"] != members[1]["task"] {
            return Err(format!("matched pair {pair_id} does not hold task/decision substrate constant"));
        }
        if members[0]["decision_cutoff_sequence"] != members[1]["decision_cutoff_sequence"] {
            return Err(format!("matched pair {pair_id} does not hold agenda cutoff constant"));
        }
    }
    Ok(report)
}
fn main() -> ExitCode {
    match run(&env::args().skip(1).collect::<Vec<_>>()) {
        Ok(r) => {
            let recall = if r.required_options == 0 { 1.0 } else { r.represented_required_options as f64 / r.required_options as f64 };
            let constraint = if r.required_constraints == 0 { 1.0 } else { r.preserved_constraints as f64 / r.required_constraints as f64 };
            println!("QUALIFIED: fixtures={} pass={} reject_controls={} indeterminate={} required_option_recall={:.4} constraint_retention={:.4} source_digests={} agenda_digests={}",
                r.fixtures, r.passed, r.rejected, r.indeterminate, recall, constraint, r.source_digest_count, r.agenda_digest_count);
            ExitCode::SUCCESS
        }
        Err(e) => { eprintln!("NOT_QUALIFIED: {e}"); ExitCode::FAILURE }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn corpus() -> Value {
        serde_json::from_str(include_str!("../../DELIB_AGENDA_FIXTURES_V0_1.json")).unwrap()
    }

    fn fixture(id: &str) -> Value {
        corpus()["fixtures"].as_array().unwrap().iter()
            .find(|f| f["fixture_id"].as_str() == Some(id)).unwrap().clone()
    }

    fn disposition(f: &Value) -> (&'static str, Vec<String>) {
        let (errors, _, _, _, _) = validate_fixture(f);
        let computed = if !errors.is_empty() { "reject" }
            else if !b(f, "source_registry_complete") { "indeterminate" }
            else { "pass" };
        (computed, errors)
    }

    #[test]
    fn every_frozen_fixture_matches_its_declared_oracle() {
        let doc = corpus();
        assert_eq!(arr(&doc, "fixtures").len(), FIXTURE_COUNT);
        for f in arr(&doc, "fixtures") {
            let (actual, errors) = disposition(f);
            assert_eq!(actual, s(&f["expected"], "disposition").unwrap(),
                "{}: {}", s(f, "fixture_id").unwrap(), errors.join("; "));
        }
    }

    #[test]
    fn complete_agenda_positive_control_passes() {
        assert_eq!(disposition(&fixture("AG-001")).0, "pass");
    }

    #[test]
    fn source_contributed_option_omission_is_rejected() {
        let (d, e) = disposition(&fixture("AG-005"));
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("required source option O-C")));
    }

    #[test]
    fn bundle_with_hidden_constraint_is_rejected() {
        let (d, e) = disposition(&fixture("AG-010"));
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("constraint") && x.contains("O-B")));
    }

    #[test]
    fn transparent_bundle_preserves_each_source_constraint() {
        let (d, e) = disposition(&fixture("AG-011"));
        assert_eq!(d, "pass", "{}", e.join("; "));
    }

    #[test]
    fn bundle_cannot_launder_one_options_constraint_as_another_options_constraint() {
        let mut f = fixture("AG-011");
        let bundle = f["presented_options"].as_array_mut().unwrap().iter_mut()
            .find(|x| x["representation_kind"] == "bundle").unwrap();
        bundle["source_option_ids"] = serde_json::json!(["O-A"]);
        let (d, e) = disposition(&f);
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("carries constraint") || x.contains("required source option O-B")));
    }

    #[test]
    fn unmarked_default_is_rejected() {
        let (d, e) = disposition(&fixture("AG-009"));
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("default")));
    }

    #[test]
    fn no_action_option_is_kept_in_the_option_universe() {
        let f = fixture("AG-008");
        assert!(arr(&f, "source_options").iter().any(|x| x["option_id"] == "O-NONE"));
        assert!(arr(&f, "presented_options").iter().any(|x| x["source_option_ids"].as_array().unwrap().iter().any(|v| v == "O-NONE")));
        assert_eq!(disposition(&f).0, "pass");
    }

    #[test]
    fn unauthorized_post_cutoff_option_is_rejected() {
        let (d, e) = disposition(&fixture("AG-013"));
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("after the frozen agenda cutoff")));
    }

    #[test]
    fn incomplete_source_registry_is_indeterminate_not_pass() {
        let (d, e) = disposition(&fixture("AG-016"));
        assert_eq!(d, "indeterminate", "{}", e.join("; "));
    }

    #[test]
    fn normative_pluralism_has_no_gold_policy_answer() {
        let f = fixture("AG-014");
        assert!(f["task"]["gold_option"].is_null());
        assert_eq!(disposition(&f).0, "pass");
    }

    #[test]
    fn disclosed_tradeoff_is_not_required_to_be_equivalent() {
        let (d, e) = disposition(&fixture("AG-015"));
        assert_eq!(d, "pass", "{}", e.join("; "));
    }

    #[test]
    fn undisclosed_split_is_rejected_but_disclosed_split_is_admitted() {
        assert_eq!(disposition(&fixture("AG-017")).0, "reject");
        let (d, e) = disposition(&fixture("AG-018"));
        assert_eq!(d, "pass", "{}", e.join("; "));
    }

    #[test]
    fn invalid_mutation_label_must_trigger_a_structural_failure() {
        let mut f = fixture("AG-001");
        f["agenda_intervention"]["semantic_relation"] = Value::String("invalid_mutation".into());
        let (d, e) = disposition(&f);
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("declared invalid")));
    }

    #[test]
    fn equivalent_pair_keeps_source_universe_and_task_constant() {
        let doc = corpus();
        let a = arr(&doc, "fixtures").iter().find(|f| f["fixture_id"] == "AG-002").unwrap();
        let b = arr(&doc, "fixtures").iter().find(|f| f["fixture_id"] == "AG-003").unwrap();
        assert_eq!(a["source_options"], b["source_options"]);
        assert_eq!(a["task"], b["task"]);
        assert_eq!(a["agenda_intervention"]["matched_material_id"], b["agenda_intervention"]["matched_material_id"]);
        assert_ne!(a["agenda_intervention"]["variant_label"], b["agenda_intervention"]["variant_label"]);
    }

    #[test]
    fn source_and_agenda_digests_are_deterministic() {
        let f = fixture("AG-001");
        assert_eq!(digest(&f["source_options"]).unwrap(), digest(&f["source_options"]).unwrap());
        assert_eq!(digest(&f["presented_options"]).unwrap(), digest(&f["presented_options"]).unwrap());
    }
    #[test]
    fn candidate_source_option_is_not_required_but_cannot_be_promoted() {
        let mut f = fixture("AG-001");
        assert_eq!(disposition(&f).0, "pass");
        f["presented_options"].as_array_mut().unwrap().push(serde_json::json!({
            "option_id":"CANDIDATE-PROMOTED",
            "source_option_ids":["O-CANDIDATE"],
            "display_position":4,
            "display_label":"Unadmitted candidate",
            "origin_status":"source_recorded",
            "introduced_sequence":3,
            "preserved_constraint_ids":["CANDIDATE-ONLY-CLAUSE"],
            "is_default":false,
            "default_declared":false,
            "default_authority":null,
            "representation_kind":"single",
            "split_group_id":null,
            "split_disclosed":false,
            "late_mutation_authorized":false,
            "late_mutation_authority":null
        }));
        let (d, e) = disposition(&f);
        assert_eq!(d, "reject");
        assert!(e.iter().any(|x| x.contains("non-admitted source option")));
    }

}
