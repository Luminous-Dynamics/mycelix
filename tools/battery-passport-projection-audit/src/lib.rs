use jsonschema::{Validator, draft202012};
use serde::Serialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

const REGISTRY_ID: &str = "eu-battery-passport-data-point-guidance";

#[derive(Debug, Serialize)]
pub struct AuditReport {
    pub registry_id: String,
    pub registry_version: String,
    pub projection_version: String,
    pub projection_input_sha256: String,
    pub canonical_projection_sha256: String,
    pub registry_sha256: String,
    pub data_point_count: usize,
    pub outcome_counts: BTreeMap<String, usize>,
}

fn sha256(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn compile_schema(bytes: &[u8]) -> Result<Validator, String> {
    let schema: Value = serde_json::from_slice(bytes)
        .map_err(|error| format!("schema JSON is invalid: {error}"))?;
    if schema.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        return Err("schema must declare JSON Schema Draft 2020-12".to_owned());
    }
    if !draft202012::meta::is_valid(&schema) {
        return Err("schema fails Draft 2020-12 meta-schema validation".to_owned());
    }
    draft202012::options()
        .should_validate_formats(true)
        .build(&schema)
        .map_err(|error| format!("schema compilation failed: {error}"))
}

fn validate_instance(validator: &Validator, value: &Value, label: &str) -> Result<(), String> {
    let errors: Vec<String> = validator.iter_errors(value).map(|error| error.to_string()).collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{label} failed JSON Schema validation: {}", errors.join("; ")))
    }
}

fn required_string<'a>(value: &'a Value, path: &str) -> Result<&'a str, String> {
    value
        .pointer(path)
        .and_then(Value::as_str)
        .filter(|text| !text.trim().is_empty())
        .ok_or_else(|| format!("missing or empty field {path}"))
}

fn audience_can_view(audience: &str, disclosure: &str) -> bool {
    match disclosure {
        "PUBLIC" => true,
        "LEGITIMATE_INTEREST_RESTRICTED" => matches!(
            audience,
            "LEGITIMATE_INTEREST"
                | "NOTIFIED_BODY"
                | "MEMBER_STATE_AUTHORITY"
                | "EU_COMMISSION"
                | "RESPONSIBLE_OPERATOR_PREVIEW"
        ),
        "LEGITIMATE_INTEREST_AND_DESIGNATED_AUTHORITIES" => matches!(
            audience,
            "LEGITIMATE_INTEREST"
                | "NOTIFIED_BODY"
                | "MEMBER_STATE_AUTHORITY"
                | "EU_COMMISSION"
                | "RESPONSIBLE_OPERATOR_PREVIEW"
        ),
        "NOTIFIED_BODIES_MEMBER_STATE_AUTHORITIES_COMMISSION" => matches!(
            audience,
            "NOTIFIED_BODY" | "MEMBER_STATE_AUTHORITY" | "EU_COMMISSION"
        ),
        _ => false,
    }
}

fn object_map<'a>(registry: &'a Value) -> Result<BTreeMap<u64, &'a Value>, String> {
    let points = registry
        .get("data_points")
        .and_then(Value::as_array)
        .ok_or_else(|| "registry.data_points is not an array".to_owned())?;
    let mut map = BTreeMap::new();
    for point in points {
        let number = point
            .get("number")
            .and_then(Value::as_u64)
            .ok_or_else(|| "registry contains a data point without an integer number".to_owned())?;
        if map.insert(number, point).is_some() {
            return Err(format!("registry contains duplicate data point number {number}"));
        }
    }
    if registry
        .get("data_point_count")
        .and_then(Value::as_u64)
        != Some(map.len() as u64)
    {
        return Err("registry data_point_count does not match its data_points array".to_owned());
    }
    if map.len() != 71 || (1_u64..=71).any(|number| !map.contains_key(&number)) {
        return Err("registry must contain the exact complete data-point ID set 1..=71".to_owned());
    }
    if registry
        .get("complete_71_point_transcription")
        .and_then(Value::as_bool)
        != Some(true)
    {
        return Err("registry must declare a complete 71-point transcription".to_owned());
    }
    Ok(map)
}

fn check_predecessor_lineage(projection: &Value) -> Result<(), String> {
    let lifecycle = required_string(projection, "/battery/lifecycle_status")?;
    let predecessor = projection
        .pointer("/battery/predecessor_passport_ref")
        .and_then(Value::as_str)
        .filter(|value| !value.trim().is_empty());
    if matches!(lifecycle, "repurposed" | "remanufactured") && predecessor.is_none() {
        return Err(format!(
            "{lifecycle} battery requires a predecessor_passport_ref"
        ));
    }
    Ok(())
}

fn check_projection_semantics(registry: &Value, projection: &Value) -> Result<BTreeMap<String, usize>, String> {
    let registry_points = object_map(registry)?;
    let category = required_string(projection, "/battery/category")?;
    let audience = required_string(projection, "/target/audience")?;
    let effective_date = required_string(projection, "/target/effective_date")?;
    let first_use = projection
        .pointer("/battery/first_use_date")
        .and_then(Value::as_str);
    check_predecessor_lineage(projection)?;

    let submitted_points = projection
        .get("data_points")
        .and_then(Value::as_array)
        .ok_or_else(|| "projection.data_points is not an array".to_owned())?;
    let mut seen = BTreeSet::new();
    let mut counts = BTreeMap::new();

    for item in submitted_points {
        let number = item
            .get("number")
            .and_then(Value::as_u64)
            .ok_or_else(|| "projection data point has no integer number".to_owned())?;
        if !seen.insert(number) {
            return Err(format!("projection contains duplicate data point number {number}"));
        }
        let registry_point = registry_points
            .get(&number)
            .ok_or_else(|| format!("projection references unknown/unadmitted data point {number}"))?;
        let expected_applicability = registry_point
            .pointer(&format!("/applicability/{category}/status"))
            .and_then(Value::as_str)
            .ok_or_else(|| format!("registry has no {category} applicability for data point {number}"))?;
        let expected_disclosure = registry_point
            .get("disclosure")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("registry has no disclosure class for data point {number}"))?;
        let declared_applicability = required_string(item, "/applicability_status")?;
        let declared_disclosure = required_string(item, "/disclosure")?;
        let outcome = required_string(item, "/outcome")?;

        if declared_applicability != expected_applicability {
            return Err(format!(
                "data point {number} applicability mismatch: projection={declared_applicability}, registry={expected_applicability}"
            ));
        }
        if declared_disclosure != expected_disclosure {
            return Err(format!(
                "data point {number} disclosure mismatch: projection={declared_disclosure}, registry={expected_disclosure}"
            ));
        }

        if expected_applicability == "NOT_TO_BE_FILLED_DISPLAYED_AT_2027_02_18"
            && outcome != "NOT_REQUIRED_AT_EFFECTIVE_DATE"
        {
            return Err(format!(
                "data point {number} is not to be filled/displayed at the effective date and must not be treated as missing"
            ));
        }
        if outcome == "NOT_REQUIRED_AT_EFFECTIVE_DATE"
            && expected_applicability != "NOT_TO_BE_FILLED_DISPLAYED_AT_2027_02_18"
        {
            return Err(format!(
                "data point {number} is marked not-required without a matching effective-date rule"
            ));
        }

        if expected_applicability == "APPLICABLE_FROM_FIRST_USE" {
            match first_use {
                Some(date) if date <= effective_date => {
                    if outcome == "NOT_APPLICABLE" {
                        return Err(format!(
                            "data point {number} applies from first use and cannot be marked not-applicable after first use"
                        ));
                    }
                }
                Some(_) => {
                    if outcome != "NOT_APPLICABLE" {
                        return Err(format!(
                            "data point {number} is not applicable before the evidenced first-use date"
                        ));
                    }
                }
                None => {
                    if outcome != "CONDITIONAL_REVIEW_REQUIRED" {
                        return Err(format!(
                            "data point {number} has an unknown first-use date; applicability requires conditional review"
                        ));
                    }
                }
            }
        }

        if expected_applicability == "MANDATORY" && outcome == "NOT_APPLICABLE" {
            return Err(format!(
                "mandatory data point {number} cannot be marked not-applicable"
            ));
        }

        let audience_allowed = audience_can_view(audience, expected_disclosure);
        let conditional_without_payload = outcome == "CONDITIONAL_REVIEW_REQUIRED"
            && item.get("value").is_none_or(Value::is_null)
            && item.get("evidence")
                .and_then(Value::as_array)
                .is_none_or(Vec::is_empty);
        if !audience_allowed
            && outcome != "WITHHELD_BY_ACCESS_POLICY"
            && outcome != "NOT_REQUIRED_AT_EFFECTIVE_DATE"
            && !conditional_without_payload
        {
            return Err(format!(
                "data point {number} is restricted from audience {audience}; projection must withhold value/evidence payload"
            ));
        }
        if outcome == "READY_TO_PUBLISH" {
            let evidence = item
                .get("evidence")
                .and_then(Value::as_array)
                .ok_or_else(|| format!("ready data point {number} has no evidence array"))?;
            if evidence.is_empty() {
                return Err(format!("ready data point {number} has no evidence"));
            }
            for record in evidence {
                let class = required_string(record, "/evidence_class")?;
                if matches!(
                    class,
                    "synthetic_fixture" | "vendor_claim" | "unknown" | "model_prediction" | "simulation_output"
                ) {
                    return Err(format!(
                        "data point {number} cannot be READY_TO_PUBLISH from evidence class {class}"
                    ));
                }
                let digest = required_string(record, "/digest")?;
                if digest.len() != 64
                    || !digest.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
                {
                    return Err(format!("data point {number} has malformed evidence digest"));
                }
            }
            if item.get("value").is_none() {
                return Err(format!("ready data point {number} has no value"));
            }
        }

        *counts.entry(outcome.to_owned()).or_insert(0) += 1;
    }

    let summary = projection
        .get("summary")
        .ok_or_else(|| "projection.summary is missing".to_owned())?;
    for (outcome, key) in [
        ("READY_TO_PUBLISH", "ready_to_publish"),
        ("MISSING_REQUIRED_EVIDENCE", "missing_required_evidence"),
        ("BLOCKED_UNQUALIFIED_SOURCE", "blocked_unqualified_source"),
        ("WITHHELD_BY_ACCESS_POLICY", "withheld_by_access_policy"),
        ("NOT_REQUIRED_AT_EFFECTIVE_DATE", "not_required_at_effective_date"),
        ("NOT_APPLICABLE", "not_applicable"),
        ("CONDITIONAL_REVIEW_REQUIRED", "conditional_review_required"),
        ("OPERATOR_AUTHORIZATION_REQUIRED", "operator_authorization_required"),
    ] {
        let actual = summary.get(key).and_then(Value::as_u64);
        let expected = counts.get(outcome).copied().unwrap_or(0) as u64;
        if actual != Some(expected) {
            return Err(format!(
                "summary.{key} mismatch: declared={actual:?}, computed={expected}"
            ));
        }
    }
    Ok(counts)
}

pub fn audit_projection(
    registry_schema_bytes: &[u8],
    registry_bytes: &[u8],
    projection_schema_bytes: &[u8],
    projection_bytes: &[u8],
) -> Result<AuditReport, String> {
    let registry_schema = compile_schema(registry_schema_bytes)?;
    let projection_schema = compile_schema(projection_schema_bytes)?;
    let registry: Value = serde_json::from_slice(registry_bytes)
        .map_err(|error| format!("registry JSON is invalid: {error}"))?;
    let projection: Value = serde_json::from_slice(projection_bytes)
        .map_err(|error| format!("projection JSON is invalid: {error}"))?;
    validate_instance(&registry_schema, &registry, "registry")?;
    validate_instance(&projection_schema, &projection, "projection")?;

    if registry.get("registry_id").and_then(Value::as_str) != Some(REGISTRY_ID) {
        return Err("unexpected registry identity".to_owned());
    }
    let registry_digest = sha256(registry_bytes);
    if projection.pointer("/registry/registry_id").and_then(Value::as_str)
        != registry.get("registry_id").and_then(Value::as_str)
        || projection.pointer("/registry/registry_version").and_then(Value::as_str)
            != registry.get("registry_version").and_then(Value::as_str)
        || projection.pointer("/registry/guidance_version").and_then(Value::as_str)
            != registry.pointer("/source/guidance_version").and_then(Value::as_str)
    {
        return Err("projection registry/guidance identity does not match the supplied registry".to_owned());
    }
    if projection.pointer("/registry/source_registry_sha256").and_then(Value::as_str)
        != Some(registry_digest.as_str())
    {
        return Err("projection source_registry_sha256 does not match the exact registry bytes".to_owned());
    }
    let counts = check_projection_semantics(&registry, &projection)?;
    let data_point_count = projection
        .get("data_points")
        .and_then(Value::as_array)
        .map(Vec::len)
        .unwrap_or(0);
    let mut canonical_projection = projection.clone();
    if let Some(points) = canonical_projection
        .get_mut("data_points")
        .and_then(Value::as_array_mut)
    {
        points.sort_by_key(|point| point.get("number").and_then(Value::as_u64).unwrap_or(u64::MAX));
    }
    let canonical_projection_bytes = serde_json::to_vec(&canonical_projection)
        .map_err(|error| format!("cannot canonicalize projection: {error}"))?;
    Ok(AuditReport {
        registry_id: registry.get("registry_id").and_then(Value::as_str).unwrap_or_default().to_owned(),
        registry_version: registry.get("registry_version").and_then(Value::as_str).unwrap_or_default().to_owned(),
        projection_version: projection.get("projection_version").and_then(Value::as_str).unwrap_or_default().to_owned(),
        projection_input_sha256: sha256(projection_bytes),
        canonical_projection_sha256: sha256(&canonical_projection_bytes),
        registry_sha256: sha256(registry_bytes),
        data_point_count,
        outcome_counts: counts,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    const REGISTRY_SCHEMA: &[u8] = include_bytes!("../../../schemas/mobility/eu_battery_passport_guidance_registry_v0_1.schema.json");
    const PROJECTION_SCHEMA: &[u8] = include_bytes!("../../../schemas/mobility/battery_passport_projection_v0_1.schema.json");
    const REGISTRY: &[u8] = include_bytes!("../../../registries/eu_battery_passport_guidance_v3_2_complete_71_points.json");
    const EV_PUBLIC: &[u8] = include_bytes!("../../../fixtures/mobility/battery_passport_projection_ev_public_v0_1.json");
    const LMT_INTEREST: &[u8] = include_bytes!("../../../fixtures/mobility/battery_passport_projection_lmt_legitimate_interest_v0_1.json");
    const BESS_INTEREST: &[u8] = include_bytes!("../../../fixtures/mobility/battery_passport_projection_industrial_stationary_bess_v0_1.json");

    #[test]
    fn complete_71_point_registry_is_schema_valid_and_gap_free() {
        let registry_schema = compile_schema(REGISTRY_SCHEMA).expect("registry schema compiles");
        let registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        validate_instance(&registry_schema, &registry, "registry").expect("registry conforms");
        assert_eq!(registry["complete_71_point_transcription"], true);
        assert_eq!(registry["data_point_count"], 71);
        let ids: BTreeSet<u64> = registry["data_points"]
            .as_array().expect("registry points")
            .iter()
            .map(|point| point["number"].as_u64().expect("point number"))
            .collect();
        assert_eq!(ids, (1_u64..=71).collect());
    }

    #[test]
    fn removing_any_guidance_point_rejects_the_registry() {
        let registry_schema = compile_schema(REGISTRY_SCHEMA).expect("registry schema compiles");
        let mut registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        registry["data_points"].as_array_mut().unwrap()
            .retain(|point| point["number"] != 29);
        registry["data_point_count"] = json!(71);
        assert!(object_map(&registry).is_err());
    }

    #[test]
    fn category_fixtures_preserve_access_and_applicability() {
        for fixture in [EV_PUBLIC, LMT_INTEREST, BESS_INTEREST] {
            let report = audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, fixture)
                .expect("category projection passes schema and semantic audit");
            assert!(report.projection_input_sha256.len() == 64);
            assert!(report.canonical_projection_sha256.len() == 64);
            assert!(report.registry_sha256.len() == 64);
        }
    }

    #[test]
    fn public_audience_cannot_receive_restricted_soce() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        let soce = projection["data_points"]
            .as_array_mut().unwrap()
            .iter_mut()
            .find(|point| point["number"] == 61).unwrap();
        soce["outcome"] = json!("READY_TO_PUBLISH");
        soce["value"] = json!(98.4);
        soce["evidence"] = json!([{
            "evidence_ref":"synthetic:soce",
            "evidence_class":"synthetic_fixture",
            "digest":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "issuer_ref":"synthetic:issuer",
            "recorded_at":"2026-11-01T00:00:00Z"
        }]);
        projection["summary"]["withheld_by_access_policy"] = json!(2);
        projection["summary"]["ready_to_publish"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn synthetic_sources_cannot_be_promoted_to_ready_to_publish() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        let identity = projection["data_points"]
            .as_array_mut().unwrap()
            .iter_mut()
            .find(|point| point["number"] == 1).unwrap();
        identity["outcome"] = json!("READY_TO_PUBLISH");
        identity["value"] = json!("synthetic:physical-id");
        identity["evidence"] = json!([{
            "evidence_ref":"synthetic:identity",
            "evidence_class":"synthetic_fixture",
            "digest":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "issuer_ref":"synthetic:fixture",
            "recorded_at":"2026-11-01T00:00:00Z"
        }]);
        projection["summary"]["blocked_unqualified_source"] = json!(0);
        projection["summary"]["ready_to_publish"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn not_to_be_displayed_is_not_counted_as_missing_required_evidence() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        let capacity = projection["data_points"]
            .as_array_mut().unwrap()
            .iter_mut()
            .find(|point| point["number"] == 62).unwrap();
        capacity["outcome"] = json!("MISSING_REQUIRED_EVIDENCE");
        projection["summary"]["not_required_at_effective_date"] = json!(0);
        projection["summary"]["missing_required_evidence"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn remanufactured_battery_requires_predecessor_passport_link() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["battery"]["lifecycle_status"] = json!("remanufactured");
        projection["battery"]["predecessor_passport_ref"] = Value::Null;
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn mismatched_category_applicability_and_disclosure_are_rejected() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        let soce = projection["data_points"]
            .as_array_mut().unwrap()
            .iter_mut()
            .find(|point| point["number"] == 61).unwrap();
        soce["disclosure"] = json!("PUBLIC");
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn source_registry_digest_mismatch_is_rejected() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["registry"]["source_registry_sha256"] = json!(
            "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
        );
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn data_point_order_changes_raw_digest_but_not_canonical_projection_digest() {
        let original = audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, EV_PUBLIC)
            .expect("original EV projection");
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["data_points"].as_array_mut().unwrap().reverse();
        let permuted = serde_json::to_vec(&projection).unwrap();
        let replay = audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &permuted)
            .expect("permuted EV projection");
        assert_ne!(original.projection_input_sha256, replay.projection_input_sha256);
        assert_eq!(
            original.canonical_projection_sha256,
            replay.canonical_projection_sha256
        );
    }
    #[test]
    fn article_8_recycled_material_fields_remain_deferred_at_february_2027() {
        let registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        for number in [20, 21, 22, 23] {
            let point = registry["data_points"].as_array().unwrap()
                .iter().find(|point| point["number"] == number).unwrap();
            for category in ["EV", "LMT", "INDUSTRIAL"] {
                assert_eq!(
                    point["applicability"][category]["status"],
                    "NOT_TO_BE_FILLED_DISPLAYED_AT_2027_02_18"
                );
                assert!(point["applicability"][category]["condition"]
                    .as_str().unwrap().contains("Article 8"));
            }
        }
    }

    #[test]
    fn duplicate_material_and_capacity_summaries_are_explicitly_not_displayed() {
        let registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        for number in [16, 25] {
            let point = registry["data_points"].as_array().unwrap()
                .iter().find(|point| point["number"] == number).unwrap();
            for category in ["EV", "LMT", "INDUSTRIAL"] {
                assert_eq!(
                    point["applicability"][category]["status"],
                    "NOT_TO_BE_FILLED_DISPLAYED_AT_2027_02_18"
                );
                assert!(point["applicability"][category]["condition"]
                    .as_str().unwrap().contains("Repeated information"));
            }
        }
    }

    #[test]
    fn voltage_and_original_power_points_are_mandatory_public_fields() {
        let registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        for number in [24, 26, 27, 28, 29] {
            let point = registry["data_points"].as_array().unwrap()
                .iter().find(|point| point["number"] == number).unwrap();
            assert_eq!(point["disclosure"], "PUBLIC");
            for category in ["EV", "LMT", "INDUSTRIAL"] {
                assert_eq!(point["applicability"][category]["status"], "MANDATORY");
            }
        }
    }

    #[test]
    fn article_13_marking_fields_preserve_mandatory_and_conditional_semantics() {
        let registry: Value = serde_json::from_slice(REGISTRY).expect("registry JSON");
        let required_mark = registry["data_points"].as_array().unwrap()
            .iter().find(|point| point["number"] == 41).unwrap();
        let conditional_symbol = registry["data_points"].as_array().unwrap()
            .iter().find(|point| point["number"] == 42).unwrap();
        assert_eq!(required_mark["disclosure"], "PUBLIC");
        assert_eq!(conditional_symbol["disclosure"], "PUBLIC");
        for category in ["EV", "LMT", "INDUSTRIAL"] {
            assert_eq!(required_mark["applicability"][category]["status"], "MANDATORY");
            assert_eq!(conditional_symbol["applicability"][category]["status"], "CONDITIONAL");
            assert!(conditional_symbol["applicability"][category]["condition"]
                .as_str().unwrap().contains("cadmium or lead symbol"));
        }
    }

    #[test]
    fn unknown_first_use_date_requires_conditional_review_for_dynamic_points() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["battery"]["first_use_date"] = Value::Null;
        let cycles = projection["data_points"].as_array_mut().unwrap()
            .iter_mut().find(|point| point["number"] == 68).unwrap();
        cycles["outcome"] = json!("CONDITIONAL_REVIEW_REQUIRED");
        projection["summary"]["withheld_by_access_policy"] = json!(2);
        projection["summary"]["conditional_review_required"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_ok());
    }

    #[test]
    fn unknown_first_use_date_cannot_be_promoted_to_not_applicable() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["battery"]["first_use_date"] = Value::Null;
        let cycles = projection["data_points"].as_array_mut().unwrap()
            .iter_mut().find(|point| point["number"] == 68).unwrap();
        cycles["outcome"] = json!("NOT_APPLICABLE");
        projection["summary"]["withheld_by_access_policy"] = json!(2);
        projection["summary"]["not_applicable"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

    #[test]
    fn evidenced_future_first_use_keeps_dynamic_points_not_applicable() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["battery"]["first_use_date"] = json!("2027-03-01");
        let cycles = projection["data_points"].as_array_mut().unwrap()
            .iter_mut().find(|point| point["number"] == 68).unwrap();
        cycles["outcome"] = json!("NOT_APPLICABLE");
        projection["summary"]["withheld_by_access_policy"] = json!(2);
        projection["summary"]["not_applicable"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_ok());
    }

    #[test]
    fn restricted_conditional_review_may_expose_only_applicability_state() {
        let mut projection: Value = serde_json::from_slice(EV_PUBLIC).expect("EV fixture");
        projection["battery"]["first_use_date"] = Value::Null;
        let cycles = projection["data_points"].as_array_mut().unwrap()
            .iter_mut().find(|point| point["number"] == 68).unwrap();
        cycles["outcome"] = json!("CONDITIONAL_REVIEW_REQUIRED");
        cycles["value"] = json!(123);
        cycles["evidence"] = json!([{
            "evidence_ref":"synthetic:restricted-cycle-history",
            "evidence_class":"synthetic_fixture",
            "digest":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "issuer_ref":"synthetic:issuer",
            "recorded_at":"2026-11-01T00:00:00Z"
        }]);
        projection["summary"]["withheld_by_access_policy"] = json!(2);
        projection["summary"]["conditional_review_required"] = json!(1);
        let bytes = serde_json::to_vec(&projection).unwrap();
        assert!(audit_projection(REGISTRY_SCHEMA, REGISTRY, PROJECTION_SCHEMA, &bytes).is_err());
    }

}
