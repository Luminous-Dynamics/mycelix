use serde_json::{json, Value};
use std::collections::BTreeSet;

const FIXTURE: &str = include_str!("../../../../fixtures/nrvca-jw-service-kpi-semantic-drift-v0.1.json");
const EPS: f64 = 0.005;

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("service KPI fixture parses")
}

fn text<'a>(value: &'a Value, field: &str) -> Result<&'a str, String> {
    value[field].as_str().filter(|s| !s.trim().is_empty())
        .ok_or_else(|| format!("missing/non-empty {field}"))
}

fn num(value: &Value, field: &str) -> Result<f64, String> {
    value[field].as_f64().ok_or_else(|| format!("missing numeric {field}"))
}

fn by_id<'a>(values: &'a [Value], field: &str, key: &str) -> Option<&'a Value> {
    values.iter().find(|item| item[field].as_str() == Some(key))
}

fn semantic_signature(definition: &Value) -> Value {
    // Derive comparability from the actual fields instead of trusting a label
    // or a manually copied fingerprint. Unknown denominator/numerator details
    // remain part of the signature as declared evidence limitations.
    json!({
        "metric_id": definition["metric_id"],
        "eligibility": definition["eligibility"],
        "numerator_definition": definition["numerator_definition"],
        "denominator_definition": definition["denominator_definition"],
        "deadline_rule": definition["deadline_rule"],
        "deadline_hours": definition["deadline_hours"],
        "category_deadlines_hours": definition["category_deadlines_hours"],
        "unit": definition["unit"]
    })
}

fn validate(root: &Value) -> Result<(), String> {
    if root["schema_version"].as_str() != Some("nrvca-utility-service-kpi-vintage-bridge/0.1")
        || root["artifact_status"].as_str() != Some("official_reported_values_discovery_only_unpinned") {
        return Err("KPI vintage bridge must remain discovery-only and unpinned".to_owned());
    }
    if root["policy"]["same_label_does_not_imply_same_definition"].as_bool() != Some(true)
        || root["policy"]["do_not_compare_if_definition_fingerprint_differs"].as_bool() != Some(true)
        || root["policy"]["causal_effect_not_inferred_from_period_delta"].as_bool() != Some(true)
        || root["policy"]["automatic_dispatch_enabled"].as_bool() != Some(false) {
        return Err("semantic comparability or authority boundary weakened".to_owned());
    }

    let sources = root["sources"].as_array().ok_or_else(|| "sources missing".to_owned())?;
    let definitions = root["definitions"].as_array().ok_or_else(|| "definitions missing".to_owned())?;
    let observations = root["observations"].as_array().ok_or_else(|| "observations missing".to_owned())?;
    let comparisons = root["comparisons"].as_array().ok_or_else(|| "comparisons missing".to_owned())?;
    if sources.len() != 2 || definitions.len() != 4 || observations.len() != 4 || comparisons.len() != 2 {
        return Err("unexpected source/definition/observation/comparison coverage".to_owned());
    }

    let mut source_ids = BTreeSet::new();
    for source in sources {
        let id = text(source, "source_id")?;
        if !source_ids.insert(id.to_owned()) { return Err(format!("duplicate source ID {id}")); }
        text(source, "publisher")?;
        text(source, "url")?;
        text(source, "publication_id")?;
        text(source, "table")?;
        if source["source_bytes_acquired"].as_bool() != Some(false)
            || !source["sha256"].is_null()
            || source["extraction_status"].as_str() != Some("indexed_text_transcription_not_independently_reparsed") {
            return Err(format!("{id}: source bytes/digest status overclaimed"));
        }
    }

    let mut definition_ids = BTreeSet::new();
    for definition in definitions {
        let id = text(definition, "definition_id")?;
        if !definition_ids.insert(id.to_owned()) { return Err(format!("duplicate definition ID {id}")); }
        text(definition, "metric_id")?;
        text(definition, "label")?;
        text(definition, "deadline_rule")?;
        text(definition, "semantic_fingerprint")?;
        if definition["unit"].as_str() != Some("percent") {
            return Err(format!("{id}: unexpected unit"));
        }
        // The fingerprint is checked against the actual semantic fields; it is
        // not accepted as a caller-supplied label that can mask a changed rule.
        match (definition["metric_id"].as_str(), definition["definition_vintage"].as_str()) {
            (Some("JW-SS1"), Some("2023/24")) => {
                if definition["deadline_hours"].as_i64() != Some(12)
                    || definition["semantic_fingerprint"].as_str()
                        != Some("water_interruptions|within_12h_from_notification|planned_and_unplanned_inclusion_unknown|denominator_details_unknown")
                    || definition["eligibility"].as_str()
                        != Some("water-supply interruptions; planned/unplanned inclusion not disambiguated in the displayed label") {
                    return Err(format!("{id}: SS1 2023/24 semantic fingerprint does not match fields"));
                }
            }
            (Some("JW-SS1"), Some("2024/25")) => {
                if !definition["deadline_hours"].is_null()
                    || definition["category_deadlines_hours"]["A"].as_i64() != Some(12)
                    || definition["category_deadlines_hours"]["B"].as_i64() != Some(24)
                    || definition["category_deadlines_hours"]["C"].as_i64() != Some(72)
                    || definition["semantic_fingerprint"].as_str()
                        != Some("planned_water_interruptions|category_deadline_A12_B24_C72|denominator_details_unknown")
                    || definition["eligibility"].as_str()
                        != Some("planned water-supply interruptions by category A, B or C") {
                    return Err(format!("{id}: SS1 2024/25 category semantics do not match fingerprint"));
                }
            }
            (Some("JW-SS2"), Some("2023/24" | "2024/25")) => {
                if definition["deadline_hours"].as_i64() != Some(48)
                    || definition["semantic_fingerprint"].as_str()
                        != Some("fire_hydrant_repairs|within_48h_from_notification|denominator_details_unknown")
                    || definition["eligibility"].as_str()
                        != Some("fire hydrant repair notifications") {
                    return Err(format!("{id}: SS2 48-hour semantics do not match fingerprint"));
                }
            }
            _ => return Err(format!("{id}: unsupported metric definition vintage")),
        }
        if definition["numerator_definition"].as_str().is_none()
            || definition["denominator_definition"].as_str().is_none()
            || definition["missing_semantics"].as_array().is_none() {
            return Err(format!("{id}: numerator/denominator uncertainty must be explicit"));
        }
        if definition["deadline_hours"].is_null()
            && definition["category_deadlines_hours"].as_object().is_none() {
            return Err(format!("{id}: missing deadline semantics"));
        }
    }

    let mut observation_ids = BTreeSet::new();
    for observation in observations {
        let id = text(observation, "observation_id")?;
        if !observation_ids.insert(id.to_owned()) { return Err(format!("duplicate observation ID {id}")); }
        let source_id = text(observation, "source_id")?;
        let definition_id = text(observation, "definition_id")?;
        if !source_ids.contains(source_id) || !definition_ids.contains(definition_id) {
            return Err(format!("{id}: broken source/definition reference"));
        }
        for field in ["target_percent", "actual_percent", "reported_variance_percentage_points", "recomputed_variance_percentage_points"] {
            let value = num(observation, field)?;
            if !value.is_finite() || (field.ends_with("percent") && field != "actual_percent" && field != "target_percent" && value.abs() > 100.0) {
                return Err(format!("{id}: invalid {field}"));
            }
        }
        let target = num(observation, "target_percent")?;
        let actual = num(observation, "actual_percent")?;
        let reported_variance = num(observation, "reported_variance_percentage_points")?;
        let recomputed_variance = num(observation, "recomputed_variance_percentage_points")?;
        if !(0.0..=100.0).contains(&target) || !(0.0..=100.0).contains(&actual) {
            return Err(format!("{id}: percentage must be in [0, 100]"));
        }
        if (actual - target - recomputed_variance).abs() > EPS
            || (reported_variance - recomputed_variance).abs() > EPS {
            return Err(format!("{id}: target variance does not reconcile"));
        }
        if observation["value_semantics"].as_str().is_none_or(str::is_empty) {
            return Err(format!("{id}: evidence limitation must be explicit"));
        }
    }

    let mut comparison_ids = BTreeSet::new();
    for comparison in comparisons {
        let id = text(comparison, "comparison_id")?;
        if !comparison_ids.insert(id.to_owned()) { return Err(format!("duplicate comparison {id}")); }
        let earlier = by_id(observations, "observation_id", text(comparison, "earlier_observation_id")?)
            .ok_or_else(|| format!("{id}: earlier observation missing"))?;
        let later = by_id(observations, "observation_id", text(comparison, "later_observation_id")?)
            .ok_or_else(|| format!("{id}: later observation missing"))?;
        let earlier_def = by_id(definitions, "definition_id", text(earlier, "definition_id")?)
            .ok_or_else(|| format!("{id}: earlier definition missing"))?;
        let later_def = by_id(definitions, "definition_id", text(later, "definition_id")?)
            .ok_or_else(|| format!("{id}: later definition missing"))?;
        if earlier_def["metric_id"] != later_def["metric_id"] {
            return Err(format!("{id}: comparison crosses metric identity"));
        }
        let same_signature = semantic_signature(earlier_def) == semantic_signature(later_def);
        match comparison["comparability"].as_str() {
            Some("not_comparable") => {
                if same_signature || !comparison["actual_delta_percentage_points"].is_null() {
                    return Err(format!("{id}: non-comparable comparison must suppress numeric trend"));
                }
                if comparison["reason_codes"].as_array().is_none_or(|codes| codes.is_empty()) {
                    return Err(format!("{id}: non-comparable result requires reason codes"));
                }
            }
            Some("provisional_definition_comparable_denominator_unknown") => {
                if !same_signature {
                    return Err(format!("{id}: changed semantic fingerprint cannot support trend"));
                }
                let actual_delta = num(comparison, "actual_delta_percentage_points")?;
                let expected_delta = num(later, "actual_percent")? - num(earlier, "actual_percent")?;
                if (actual_delta - expected_delta).abs() > EPS {
                    return Err(format!("{id}: published actual delta does not reconcile"));
                }
                let early_gap = num(comparison, "target_gap_2023_24_percentage_points")?;
                let late_gap = num(comparison, "target_gap_2024_25_percentage_points")?;
                if (early_gap - num(earlier, "recomputed_variance_percentage_points")?).abs() > EPS
                    || (late_gap - num(later, "recomputed_variance_percentage_points")?).abs() > EPS {
                    return Err(format!("{id}: target gap does not match the source observations"));
                }
                if comparison["reason_codes"].as_array().is_none_or(|codes| {
                    !codes.iter().any(|x| x.as_str() == Some("raw_denominator_and_exclusion_rules_not_available"))
                }) {
                    return Err(format!("{id}: missing denominator uncertainty qualifier"));
                }
            }
            _ => return Err(format!("{id}: unknown comparability status")),
        }
    }

    if root["report_level_summary"]["achieved"].as_i64() != Some(3)
        || root["report_level_summary"]["total"].as_i64() != Some(13)
        || root["report_level_summary"]["source_id"].as_str() != Some("jw-annual-report-2024-25") {
        return Err("report's 3-of-13 result must remain source-attributed".to_owned());
    }
    Ok(())
}

#[test]
fn kpi_vintages_and_source_limitations_are_valid() {
    validate(&fixture()).expect("valid discovery-only KPI bridge");
}

#[test]
fn ss1_definition_drift_suppresses_year_on_year_delta() {
    let root = fixture();
    validate(&root).expect("valid KPI bridge");
    let comparison = by_id(root["comparisons"].as_array().unwrap(), "comparison_id", "jw-ss1-yoy-2023-24-to-2024-25").unwrap();
    assert_eq!(comparison["comparability"], "not_comparable");
    assert!(comparison["actual_delta_percentage_points"].is_null());
    assert!(comparison["reason_codes"].as_array().unwrap().iter().any(|r| r == "fixed_12h_deadline_replaced_by_category_deadlines_12_24_72h"));
}

#[test]
fn ss2_trend_and_target_gap_reconcile_but_remain_provisional() {
    let root = fixture();
    validate(&root).expect("valid KPI bridge");
    let comparison = by_id(root["comparisons"].as_array().unwrap(), "comparison_id", "jw-ss2-yoy-2023-24-to-2024-25").unwrap();
    assert_eq!(comparison["comparability"], "provisional_definition_comparable_denominator_unknown");
    assert!((num(comparison, "actual_delta_percentage_points").unwrap() - 2.95).abs() <= EPS);
    assert!((num(comparison, "target_gap_2024_25_percentage_points").unwrap() - (-8.94)).abs() <= EPS);
}

#[test]
fn source_hashes_targets_and_variances_fail_closed_when_mutated() {
    let mut source_hash = fixture();
    source_hash["sources"][0]["sha256"] = json!("abcdef0123456789");
    assert!(validate(&source_hash).is_err());

    let mut target = fixture();
    target["observations"][0]["target_percent"] = json!(80.0);
    assert!(validate(&target).is_err());

    let mut variance = fixture();
    variance["observations"][3]["reported_variance_percentage_points"] = json!(0.0);
    assert!(validate(&variance).is_err());
}

#[test]
fn semantic_signature_and_denominator_uncertainty_cannot_be_silently_ignored() {
    let mut relabel = fixture();
    relabel["comparisons"][0]["comparability"] = json!("provisional_definition_comparable_denominator_unknown");
    relabel["comparisons"][0]["actual_delta_percentage_points"] = json!(73.54);
    assert!(validate(&relabel).is_err());

    let mut same_label_new_deadline = fixture();
    same_label_new_deadline["definitions"][3]["deadline_hours"] = json!(24);
    assert!(validate(&same_label_new_deadline).is_err());

    let mut denominator_drift = fixture();
    denominator_drift["definitions"][3]["denominator_definition"] = json!("denominator changed: all hydrant assets rather than notifications");
    assert!(validate(&denominator_drift).is_err());

    let mut numerator_drift = fixture();
    numerator_drift["definitions"][3]["numerator_definition"] = json!("numerator changed: repairs merely assigned rather than completed");
    assert!(validate(&numerator_drift).is_err());

    let mut drop_uncertainty = fixture();
    drop_uncertainty["comparisons"][1]["reason_codes"] = json!(["same_published_48h_definition"]);
    assert!(validate(&drop_uncertainty).is_err());
}

#[test]
fn unknown_sources_duplicate_records_and_partial_report_counts_fail_closed() {
    let mut unknown_source = fixture();
    unknown_source["observations"][0]["source_id"] = json!("missing-source");
    assert!(validate(&unknown_source).is_err());

    let mut duplicate = fixture();
    duplicate["observations"][1]["observation_id"] = json!("jw-ss1-2023-24");
    assert!(validate(&duplicate).is_err());

    let mut false_summary = fixture();
    false_summary["report_level_summary"]["total"] = json!(2);
    assert!(validate(&false_summary).is_err());
}
