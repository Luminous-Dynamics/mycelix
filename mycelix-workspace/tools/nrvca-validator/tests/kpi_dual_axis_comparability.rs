use serde_json::{json, Value};
use std::collections::BTreeSet;

const FIXTURE: &str = include_str!("../../../../fixtures/nrvca-kpi-dual-axis-comparability-synthetic-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("dual-axis KPI fixture parses")
}

fn text<'a>(value: &'a Value, field: &str) -> Result<&'a str, String> {
    value[field]
        .as_str()
        .filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("missing/non-empty {field}"))
}

fn list_strings(value: &Value) -> Result<Vec<String>, String> {
    value.as_array()
        .ok_or_else(|| "expected string array".to_owned())?
        .iter()
        .map(|v| v.as_str().map(str::to_owned).ok_or_else(|| "non-string array item".to_owned()))
        .collect()
}

fn fields_diff(left: &Value, right: &Value, fields: &[String]) -> (BTreeSet<String>, BTreeSet<String>) {
    let mut changed = BTreeSet::new();
    let mut unknown = BTreeSet::new();
    for field in fields {
        let a = left.get(field).unwrap_or(&Value::Null);
        let b = right.get(field).unwrap_or(&Value::Null);
        if a.is_null() || b.is_null() {
            unknown.insert(field.clone());
        } else if a != b {
            changed.insert(field.clone());
        }
    }
    (changed, unknown)
}

fn measurement_status(left: &Value, right: &Value, fields: &[String]) -> (String, BTreeSet<String>, BTreeSet<String>) {
    let (changed, unknown) = fields_diff(&left["measurement"], &right["measurement"], fields);
    let status = if !changed.is_empty() {
        "not_comparable"
    } else if !unknown.is_empty() {
        "indeterminate"
    } else {
        "comparable"
    };
    (status.to_owned(), changed, unknown)
}

fn target_status(left: &Value, right: &Value, fields: &[String]) -> (String, BTreeSet<String>, BTreeSet<String>) {
    let (changed, unknown) = fields_diff(&left["target"], &right["target"], fields);
    let status = if !changed.is_empty() {
        "changed_target_policy"
    } else if !unknown.is_empty() {
        "indeterminate"
    } else {
        "same_target_policy"
    };
    (status.to_owned(), changed, unknown)
}

fn naive_same_id_and_label(left: &Value, right: &Value) -> &'static str {
    if left["local_metric_id"] == right["local_metric_id"] && left["label"] == right["label"] {
        "comparable"
    } else {
        "not_comparable"
    }
}

fn validate(root: &Value) -> Result<(), String> {
    if root["schema_version"].as_str() != Some("nrvca-kpi-dual-axis-comparability-benchmark/0.1")
        || root["artifact_status"].as_str() != Some("synthetic_known_answer_benchmark") {
        return Err("fixture must remain an explicitly synthetic benchmark".to_owned());
    }
    if root["rules"]["labels_and_local_ids_are_not_measurement_semantics"].as_bool() != Some(true)
        || root["rules"]["source_vintage_is_provenance_not_measurement_semantics"].as_bool() != Some(true)
        || root["rules"]["target_changes_do_not_automatically_invalidate_actual_measurement_comparability"].as_bool() != Some(true)
        || root["rules"]["declared_fingerprint_is_untrusted_and_must_not_override_actual_fields"].as_bool() != Some(true) {
        return Err("dual-axis comparability policy weakened".to_owned());
    }
    let expected_semantic_fields = [
        "canonical_concept_id", "unit", "numerator_definition", "denominator_definition",
        "eligible_population", "clock_start", "deadline_rule", "category_rule",
        "geographic_scope", "aggregation", "inclusion_rule",
    ];
    let expected_target_fields = ["value", "unit", "target_population", "policy_vintage"];
    if list_strings(&root["semantic_fields"])? != expected_semantic_fields.iter().map(|s| s.to_string()).collect::<Vec<_>>()
        || list_strings(&root["target_fields"])? != expected_target_fields.iter().map(|s| s.to_string()).collect::<Vec<_>>() {
        return Err("semantic/target comparison field set changed without review".to_owned());
    }
    let cases = root["cases"].as_array().ok_or_else(|| "cases missing".to_owned())?;
    if cases.len() != 24 {
        return Err(format!("expected 24 known-answer pairs, got {}", cases.len()));
    }
    let mut ids = BTreeSet::new();
    for case in cases {
        let id = text(case, "case_id")?;
        if !ids.insert(id.to_owned()) {
            return Err(format!("duplicate benchmark case {id}"));
        }
        for side in ["left", "right"] {
            text(&case[side], "local_metric_id")?;
            text(&case[side], "label")?;
            text(&case[side], "source_vintage")?;
            for key in ["measurement", "target"] {
                if !case[side][key].is_object() {
                    return Err(format!("{id}: {side}.{key} must be an object"));
                }
            }
        }
        if !matches!(
            case["expected"]["measurement_comparability"].as_str(),
            Some("comparable" | "not_comparable" | "indeterminate")
        ) {
            return Err(format!("{id}: invalid expected measurement status"));
        }
        if !matches!(
            case["expected"]["target_comparability"].as_str(),
            Some("same_target_policy" | "changed_target_policy" | "indeterminate")
        ) {
            return Err(format!("{id}: invalid expected target status"));
        }
        if case["expected"]["reason_codes"].as_array().is_none_or(|v| v.is_empty()) {
            return Err(format!("{id}: expected reason codes missing"));
        }
    }
    if root["baseline"]["name"].as_str() != Some("same-local-id-and-display-label") {
        return Err("naive baseline definition changed".to_owned());
    }
    let mut aggregate = std::collections::BTreeMap::<String, i64>::new();
    aggregate.insert("total_pairs".to_owned(), cases.len() as i64);
    for item in cases {
        let measurement = text(&item["expected"], "measurement_comparability")?;
        let target = text(&item["expected"], "target_comparability")?;
        *aggregate.entry(format!("measurement_{measurement}")).or_default() += 1;
        *aggregate.entry(match target {
            "same_target_policy" => "target_same",
            "changed_target_policy" => "target_changed",
            "indeterminate" => "target_indeterminate",
            _ => return Err(format!("unexpected target status in {}", text(item, "case_id")?)),
        }.to_owned()).or_default() += 1;
        let naive = naive_same_id_and_label(&item["left"], &item["right"]);
        if naive == "comparable" && measurement != "comparable" {
            *aggregate.entry("naive_false_comparable".to_owned()).or_default() += 1;
        }
        if naive == "not_comparable" && measurement == "comparable" {
            *aggregate.entry("naive_false_incomparable".to_owned()).or_default() += 1;
        }
    }
    let summary = &root["expected_aggregate"];
    for (key, value) in aggregate {
        if summary[key.as_str()].as_i64() != Some(value) {
            return Err(format!("benchmark aggregate {key} mismatches pair-level decisions"));
        }
    }

    Ok(())
}

#[test]
fn all_24_pairs_match_separate_measurement_and_target_known_answers() {
    let root = fixture();
    validate(&root).expect("valid dual-axis benchmark");
    let semantic_fields = list_strings(&root["semantic_fields"]).unwrap();
    let target_fields = list_strings(&root["target_fields"]).unwrap();
    let mut comparable = 0_i64;
    let mut not_comparable = 0_i64;
    let mut indeterminate = 0_i64;
    let mut target_same = 0_i64;
    let mut target_changed = 0_i64;
    let mut target_unknown = 0_i64;
    let mut naive_false_comparable = 0_i64;
    let mut naive_false_incomparable = 0_i64;

    for case in root["cases"].as_array().unwrap() {
        let (m_status, changed, unknown) =
            measurement_status(&case["left"], &case["right"], &semantic_fields);
        let (t_status, target_changed_fields, target_unknown_fields) =
            target_status(&case["left"], &case["right"], &target_fields);
        assert_eq!(
            m_status,
            text(&case["expected"], "measurement_comparability").unwrap(),
            "measurement status for {} (changed={changed:?}, unknown={unknown:?})",
            text(case, "case_id").unwrap()
        );
        assert_eq!(
            t_status,
            text(&case["expected"], "target_comparability").unwrap(),
            "target status for {} (changed={target_changed_fields:?}, unknown={target_unknown_fields:?})",
            text(case, "case_id").unwrap()
        );
        assert_eq!(
            naive_same_id_and_label(&case["left"], &case["right"]),
            text(case, "naive_same_id_and_label_baseline").unwrap(),
            "baseline disposition for {}",
            text(case, "case_id").unwrap()
        );

        match m_status.as_str() {
            "comparable" => comparable += 1,
            "not_comparable" => not_comparable += 1,
            "indeterminate" => indeterminate += 1,
            _ => unreachable!(),
        }
        match t_status.as_str() {
            "same_target_policy" => target_same += 1,
            "changed_target_policy" => target_changed += 1,
            "indeterminate" => target_unknown += 1,
            _ => unreachable!(),
        }
        let baseline = naive_same_id_and_label(&case["left"], &case["right"]);
        if baseline == "comparable" && m_status != "comparable" {
            naive_false_comparable += 1;
        }
        if baseline == "not_comparable" && m_status == "comparable" {
            naive_false_incomparable += 1;
        }
    }

    assert_eq!(comparable, root["expected_aggregate"]["measurement_comparable"].as_i64().unwrap());
    assert_eq!(not_comparable, root["expected_aggregate"]["measurement_not_comparable"].as_i64().unwrap());
    assert_eq!(indeterminate, root["expected_aggregate"]["measurement_indeterminate"].as_i64().unwrap());
    assert_eq!(target_same, root["expected_aggregate"]["target_same"].as_i64().unwrap());
    assert_eq!(target_changed, root["expected_aggregate"]["target_changed"].as_i64().unwrap());
    assert_eq!(target_unknown, root["expected_aggregate"]["target_indeterminate"].as_i64().unwrap());
    assert_eq!(naive_false_comparable, root["expected_aggregate"]["naive_false_comparable"].as_i64().unwrap());
    assert_eq!(naive_false_incomparable, root["expected_aggregate"]["naive_false_incomparable"].as_i64().unwrap());
}

#[test]
fn target_change_preserves_measurement_comparability_but_not_policy_comparability() {
    let root = fixture();
    let item = root["cases"].as_array().unwrap().iter()
        .find(|c| c["case_id"] == "target-value-only").unwrap();
    let measurement = list_strings(&root["semantic_fields"]).unwrap();
    let targets = list_strings(&root["target_fields"]).unwrap();
    assert_eq!(measurement_status(&item["left"], &item["right"], &measurement).0, "comparable");
    assert_eq!(target_status(&item["left"], &item["right"], &targets).0, "changed_target_policy");
}

#[test]
fn wording_and_source_vintage_changes_do_not_create_false_semantic_drift() {
    let root = fixture();
    let fields = list_strings(&root["semantic_fields"]).unwrap();
    for id in ["wording-only-label-change", "source-vintage-only", "local-id-only"] {
        let item = root["cases"].as_array().unwrap().iter().find(|c| c["case_id"] == id).unwrap();
        assert_eq!(measurement_status(&item["left"], &item["right"], &fields).0, "comparable", "{id}");
    }
}

#[test]
fn critical_unknowns_abstain_unless_a_known_semantic_mismatch_exists() {
    let root = fixture();
    let fields = list_strings(&root["semantic_fields"]).unwrap();
    let deadline = root["cases"].as_array().unwrap().iter().find(|c| c["case_id"] == "deadline-unknown").unwrap();
    let denominator = root["cases"].as_array().unwrap().iter().find(|c| c["case_id"] == "denominator-unknown").unwrap();
    let hard_mismatch_and_unknown = root["cases"].as_array().unwrap().iter().find(|c| c["case_id"] == "fingerprint-lie").unwrap();
    assert_eq!(measurement_status(&deadline["left"], &deadline["right"], &fields).0, "indeterminate");
    assert_eq!(measurement_status(&denominator["left"], &denominator["right"], &fields).0, "indeterminate");
    assert_eq!(measurement_status(&hard_mismatch_and_unknown["left"], &hard_mismatch_and_unknown["right"], &fields).0, "not_comparable");
}

#[test]
fn actual_fields_not_labels_or_declared_fingerprints_decide_comparability() {
    let root = fixture();
    let fields = list_strings(&root["semantic_fields"]).unwrap();
    let item = root["cases"].as_array().unwrap().iter().find(|c| c["case_id"] == "fingerprint-lie").unwrap();
    assert_eq!(item["left"]["declared_fingerprint"], item["right"]["declared_fingerprint"]);
    assert_eq!(item["left"]["label"], item["right"]["label"]);
    assert_eq!(naive_same_id_and_label(&item["left"], &item["right"]), "comparable");
    assert_eq!(measurement_status(&item["left"], &item["right"], &fields).0, "not_comparable");
}

#[test]
fn malformed_or_incomplete_contracts_fail_closed() {
    let root = fixture();
    validate(&root).expect("base fixture valid");
    let mut changed_unit = root.clone();
    changed_unit["cases"][0]["right"]["measurement"]["unit"] = json!("hours");
    let fields = list_strings(&root["semantic_fields"]).unwrap();
    assert_eq!(measurement_status(&changed_unit["cases"][0]["left"], &changed_unit["cases"][0]["right"], &fields).0, "not_comparable");

    let mut unknown_denominator = root.clone();
    unknown_denominator["cases"][0]["right"]["measurement"]["denominator_definition"] = Value::Null;
    assert_eq!(measurement_status(&unknown_denominator["cases"][0]["left"], &unknown_denominator["cases"][0]["right"], &fields).0, "indeterminate");

    let mut missing_case_id = root.clone();
    missing_case_id["cases"][0]["case_id"] = json!("");
    assert!(validate(&missing_case_id).is_err());

    let mut false_counts = root.clone();
    false_counts["expected_aggregate"]["naive_false_comparable"] = json!(0);
    // The fixture's summary itself is a frozen audit contract. Recalculate the
    // benchmark metrics by rerunning the known-answer test when this mutation is made.
    assert!(validate(&false_counts).is_err());
}
