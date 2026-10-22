use serde_json::{json, Value};
use std::collections::BTreeSet;

const STRESS_FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json");
const EVIDENCE_MANIFEST: &str =
    include_str!("../../../../fixtures/nrvca-scenario-evidence-manifest-v0.1.json");

fn stress_fixture() -> Value {
    serde_json::from_str(STRESS_FIXTURE).expect("stress fixture parses")
}

fn evidence_manifest() -> Value {
    serde_json::from_str(EVIDENCE_MANIFEST).expect("evidence manifest parses")
}

fn strings(value: &Value) -> Result<Vec<String>, String> {
    value
        .as_array()
        .ok_or_else(|| "expected an array".to_owned())?
        .iter()
        .map(|item| {
            item.as_str()
                .map(str::to_owned)
                .ok_or_else(|| "expected array items to be strings".to_owned())
        })
        .collect()
}

fn is_nonempty(value: &Value) -> bool {
    match value {
        Value::Null => false,
        Value::String(s) => !s.trim().is_empty(),
        Value::Array(_) | Value::Object(_) => true,
        Value::Bool(_) | Value::Number(_) => true,
    }
}

fn is_sha256(value: &Value) -> bool {
    value.as_str().is_some_and(|digest| {
        digest.len() == 64 && digest.bytes().all(|byte| byte.is_ascii_hexdigit())
    })
}

fn source_by_id<'a>(manifest: &'a Value, source_id: &str) -> Option<&'a Value> {
    manifest["methodological_sources"]
        .as_array()?
        .iter()
        .find(|source| source["source_id"].as_str() == Some(source_id))
}

fn validate_definition(manifest: &Value, definition: &Value) -> Result<(), String> {
    let status = definition["evidence_status"]
        .as_str()
        .ok_or_else(|| "missing evidence_status".to_owned())?;
    let tier = manifest["evidence_tier_contract"]
        .get(status)
        .ok_or_else(|| format!("unrecognized evidence tier: {status}"))?;

    for field in strings(&tier["required_fields"])? {
        if !definition.get(&field).is_some_and(is_nonempty) {
            return Err(format!("{status} definition is missing required field {field}"));
        }
    }

    let refs = strings(&definition["source_refs"])?;
    let required_role = tier["required_source_role"].as_str();

    if status == "synthetic_assumption" && !refs.is_empty() {
        return Err("synthetic assumptions must not cite data sources as evidence".to_owned());
    }
    if matches!(status, "official_observation" | "operator_measurement" | "published_estimate" | "model_output" | "expert_judgment")
        && refs.is_empty()
    {
        return Err(format!("{status} requires at least one role-appropriate source reference"));
    }

    for source_id in refs {
        let source = source_by_id(manifest, &source_id)
            .ok_or_else(|| format!("dangling source reference: {source_id}"))?;
        if let Some(expected_role) = required_role {
            if source["role"].as_str() != Some(expected_role) {
                return Err(format!(
                    "source {source_id} has role {:?}, but {status} requires {expected_role}",
                    source["role"].as_str()
                ));
            }
        } else if source["role"].as_str() == Some("methodological_context_only") {
            return Err("methodology reference cannot be used as empirical evidence".to_owned());
        }
    }

    if matches!(status, "official_observation" | "operator_measurement" | "published_estimate")
        && !is_sha256(&definition["content_sha256"])
    {
        return Err(format!("{status} requires a syntactically valid SHA-256 content digest"));
    }
    if status == "model_output"
        && (!is_sha256(&definition["model_configuration_sha256"])
            || !is_sha256(&definition["input_manifest_sha256"]))
    {
        return Err("model_output requires configuration and input-manifest SHA-256 digests".to_owned());
    }

    Ok(())
}

fn validate_scenario(
    stress_fixture: &Value,
    manifest: &Value,
    scenario: &Value,
) -> Result<(), String> {
    let scenario_id = scenario["scenario_id"]
        .as_str()
        .ok_or_else(|| "scenario missing scenario_id".to_owned())?;
    let stressor_ids = strings(&scenario["stressors"])?;
    let stressor_set: BTreeSet<&str> = stressor_ids.iter().map(String::as_str).collect();

    if stressor_set.len() != stressor_ids.len() {
        return Err(format!("duplicate stressor in scenario {scenario_id}"));
    }

    let declared: BTreeSet<&str> = manifest["stressor_definitions"]
        .as_array()
        .ok_or_else(|| "stressor_definitions must be an array".to_owned())?
        .iter()
        .filter_map(|definition| definition["stressor_id"].as_str())
        .collect();

    for stressor in &stressor_set {
        if !declared.contains(stressor) {
            return Err(format!("scenario {scenario_id} uses undefined stressor {stressor}"));
        }
    }

    let conditions: Vec<&Value> = manifest["scenario_conditions"]
        .as_array()
        .ok_or_else(|| "scenario_conditions must be an array".to_owned())?
        .iter()
        .filter(|condition| condition["scenario_id"].as_str() == Some(scenario_id))
        .collect();
    if conditions.len() != 1 {
        return Err(format!(
            "scenario {scenario_id} needs exactly one history-condition record"
        ));
    }
    let history_tokens: BTreeSet<String> =
        strings(&conditions[0]["history_tokens"])?.into_iter().collect();
    if conditions[0]["history_evidence_status"].as_str() != Some("synthetic_assumption") {
        return Err(format!("scenario {scenario_id} has an unexpected history evidence tier"));
    }

    for rule in manifest["dependency_rules"]
        .as_array()
        .ok_or_else(|| "dependency_rules must be an array".to_owned())?
    {
        let rule_type = rule["rule_type"].as_str().unwrap_or_default();
        let trigger = rule["if_present"].as_str().unwrap_or_default();
        if !stressor_set.contains(trigger) {
            continue;
        }

        match rule_type {
            "stressor_dependency" => {
                for required in strings(&rule["requires_all"])? {
                    if !stressor_set.contains(required.as_str()) {
                        return Err(format!(
                            "scenario {scenario_id} violates dependency {}: missing {required}",
                            rule["rule_id"].as_str().unwrap_or("unnamed")
                        ));
                    }
                }
            }
            "temporal_dependency" => {
                for required in strings(&rule["requires_history_tokens"])? {
                    if !history_tokens.contains(&required) {
                        return Err(format!(
                            "scenario {scenario_id} violates temporal rule {}: missing history token {required}",
                            rule["rule_id"].as_str().unwrap_or("unnamed")
                        ));
                    }
                }
            }
            other => return Err(format!("unknown dependency rule type: {other}")),
        }
    }

    let declared_scenarios = stress_fixture["scenario_manifest"]
        .as_array()
        .ok_or_else(|| "scenario_manifest must be an array".to_owned())?;
    if !declared_scenarios.iter().any(|item| {
        item["scenario_id"].as_str() == Some(scenario_id)
    }) {
        return Err(format!("scenario condition refers to undeclared scenario {scenario_id}"));
    }

    Ok(())
}

fn validate_cross_fixture(stress: &Value, evidence: &Value) -> Result<(), String> {
    if evidence["status"].as_str() != Some("synthetic_only") {
        return Err("v0.1 scenario evidence manifest must remain explicitly synthetic".to_owned());
    }
    if stress["scoring_policy"]["scenario_weights_used"].as_bool() != Some(false) {
        return Err("scenario-neutral fixture must not use weights".to_owned());
    }

    let stressors: Vec<&Value> = stress["scenario_manifest"]
        .as_array()
        .ok_or_else(|| "scenario_manifest must be an array".to_owned())?
        .iter()
        .collect();
    let definitions: Vec<&Value> = evidence["stressor_definitions"]
        .as_array()
        .ok_or_else(|| "stressor_definitions must be an array".to_owned())?
        .iter()
        .collect();

    let mut used_ids = BTreeSet::new();
    for scenario in &stressors {
        validate_scenario(stress, evidence, scenario)?;
        for id in strings(&scenario["stressors"])? {
            used_ids.insert(id);
        }
    }

    let mut definition_ids = BTreeSet::new();
    for definition in &definitions {
        let id = definition["stressor_id"]
            .as_str()
            .ok_or_else(|| "stressor definition missing stressor_id".to_owned())?;
        if !definition_ids.insert(id.to_owned()) {
            return Err(format!("duplicate stressor definition {id}"));
        }
        validate_definition(evidence, definition)?;
        if definition["evidence_status"].as_str() != Some("synthetic_assumption")
            || definition["calibration_status"].as_str() != Some("not_calibrated")
        {
            return Err(format!("stressor {id} was promoted beyond the synthetic evidence ceiling"));
        }
    }
    if used_ids != definition_ids {
        return Err(format!(
            "stressor coverage mismatch: used {used_ids:?}, defined {definition_ids:?}"
        ));
    }

    let scenario_ids: BTreeSet<String> = stressors
        .iter()
        .map(|scenario| scenario["scenario_id"].as_str().unwrap().to_owned())
        .collect();
    let condition_ids: Vec<String> = evidence["scenario_conditions"]
        .as_array()
        .ok_or_else(|| "scenario_conditions must be an array".to_owned())?
        .iter()
        .map(|condition| {
            condition["scenario_id"]
                .as_str()
                .ok_or_else(|| "scenario condition missing scenario_id".to_owned())
                .map(str::to_owned)
        })
        .collect::<Result<_, _>>()?;
    let condition_set: BTreeSet<String> = condition_ids.iter().cloned().collect();
    if condition_set.len() != condition_ids.len() || condition_set != scenario_ids {
        return Err("scenario history conditions must cover each scenario exactly once".to_owned());
    }

    for strategy in stress["strategies"]
        .as_array()
        .ok_or_else(|| "strategies must be an array".to_owned())?
    {
        for row in strategy["scenario_results"]
            .as_array()
            .ok_or_else(|| "scenario_results must be an array".to_owned())?
        {
            if row["evidence_status"].as_str() != Some("synthetic_stress_assumption") {
                return Err("scenario outcome was promoted beyond synthetic stress assumption".to_owned());
            }
            if row.get("probability").is_some() {
                return Err("scenario-neutral outcome must not carry an implicit probability".to_owned());
            }
        }
    }

    Ok(())
}

#[test]
fn every_used_stressor_has_exactly_one_definition_and_each_scenario_has_one_history_record() {
    let stress = stress_fixture();
    let evidence = evidence_manifest();
    validate_cross_fixture(&stress, &evidence).expect("cross-fixture evidence contract should hold");
    assert_eq!(stress["scenario_manifest"].as_array().unwrap().len(), 5);
    assert_eq!(stress["strategies"].as_array().unwrap().len(), 3);
    assert_eq!(evidence["stressor_definitions"].as_array().unwrap().len(), 6);
    assert_eq!(evidence["scenario_conditions"].as_array().unwrap().len(), 5);
}

#[test]
fn method_references_are_not_stressor_measurements_and_all_frozen_outcomes_remain_synthetic() {
    let stress = stress_fixture();
    let evidence = evidence_manifest();
    for source in evidence["methodological_sources"].as_array().unwrap() {
        assert_eq!(source["role"], "methodological_context_only");
    }
    for definition in evidence["stressor_definitions"].as_array().unwrap() {
        assert_eq!(definition["evidence_status"], "synthetic_assumption");
        assert_eq!(definition["calibration_status"], "not_calibrated");
        assert!(definition["source_refs"].as_array().unwrap().is_empty());
    }
    for strategy in stress["strategies"].as_array().unwrap() {
        for outcome in strategy["scenario_results"].as_array().unwrap() {
            assert_eq!(outcome["evidence_status"], "synthetic_stress_assumption");
        }
    }
}

#[test]
fn dependency_rules_and_repeat_disruption_history_are_checked_as_data() {
    let stress = stress_fixture();
    let evidence = evidence_manifest();
    for scenario in stress["scenario_manifest"].as_array().unwrap() {
        validate_scenario(&stress, &evidence, scenario).expect("scenario dependencies are valid");
    }

    let prolonged = evidence["scenario_conditions"]
        .as_array().unwrap()
        .iter()
        .find(|condition| condition["scenario_id"] == "prolonged-repeat-disruption")
        .unwrap();
    let history: BTreeSet<String> = strings(&prolonged["history_tokens"]).unwrap().into_iter().collect();
    assert!(history.contains("prior_disruption"));
    assert!(history.contains("prior_reserve_drawdown"));
    assert!(history.contains("repeat_disruption"));
}

#[test]
fn unsupported_evidence_promotion_fails_closed() {
    let evidence = evidence_manifest();
    let mut promoted = evidence["stressor_definitions"][0].clone();
    promoted["evidence_status"] = json!("official_observation");
    assert!(validate_definition(&evidence, &promoted).is_err());
}

#[test]
fn empirical_claim_needs_a_real_role_and_well_formed_digest() {
    let mut evidence = evidence_manifest();
    let mut candidate = evidence["stressor_definitions"][0].clone();
    candidate["evidence_status"] = json!("official_observation");
    candidate["publisher_id"] = json!("test-publisher");
    candidate["source_artifact_uri"] = json!("https://example.invalid/source.csv");
    candidate["retrieved_at"] = json!("2026-10-22T00:00:00Z");
    candidate["valid_time"] = json!("2026-09");
    candidate["unit"] = json!("m3/day");
    candidate["geographic_coverage"] = json!("test-area");
    candidate["license_status"] = json!("test-only");
    candidate["content_sha256"] = json!("not-a-digest");
    candidate["source_refs"] = json!(["wuca_water_utility_dmdu_2026"]);

    assert!(validate_definition(&evidence, &candidate).is_err(),
        "a method guide cannot substantiate an observation");

    evidence["methodological_sources"].as_array_mut().unwrap().push(json!({
        "source_id": "test-data-source",
        "role": "empirical_data_source",
        "url": "https://example.invalid/source.csv"
    }));
    candidate["source_refs"] = json!(["test-data-source"]);
    assert!(validate_definition(&evidence, &candidate).is_err(),
        "syntactically invalid SHA-256 values must fail");
    candidate["content_sha256"] = json!("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    assert!(validate_definition(&evidence, &candidate).is_ok(),
        "the contract validates required provenance shape; digest-to-byte verification is a separate acquisition step");
}

#[test]
fn dangling_source_references_and_missing_scenario_definitions_are_rejected() {
    let evidence = evidence_manifest();
    let mut definition = evidence["stressor_definitions"][0].clone();
    definition["source_refs"] = json!(["source-that-does-not-exist"]);
    assert!(validate_definition(&evidence, &definition).is_err());

    let stress = stress_fixture();
    let mut incomplete = evidence.clone();
    incomplete["stressor_definitions"].as_array_mut().unwrap().remove(0);
    assert!(validate_cross_fixture(&stress, &incomplete).is_err());
}

#[test]
fn incompatible_combinations_and_missing_history_are_rejected() {
    let mut stress = stress_fixture();
    let evidence = evidence_manifest();

    let drought = stress["scenario_manifest"].as_array_mut().unwrap()
        .iter_mut()
        .find(|scenario| scenario["scenario_id"] == "seasonal-drought")
        .unwrap();
    drought["stressors"].as_array_mut().unwrap()
        .push(json!("powered_pressure_system_down"));
    assert!(validate_cross_fixture(&stress, &evidence).is_err(),
        "powered-system outage without its declared grid stressor must fail");

    let mut no_history = evidence.clone();
    let prolonged = no_history["scenario_conditions"].as_array_mut().unwrap()
        .iter_mut()
        .find(|condition| condition["scenario_id"] == "prolonged-repeat-disruption")
        .unwrap();
    prolonged["history_tokens"] = json!([]);
    let stress = stress_fixture();
    assert!(validate_cross_fixture(&stress, &no_history).is_err(),
        "reserve-depleted repeated disruption without prior-event history must fail");
}

#[test]
fn scenario_probabilities_are_not_inferred_from_case_counts_or_outcome_metadata() {
    let stress = stress_fixture();
    let evidence = evidence_manifest();
    validate_cross_fixture(&stress, &evidence).expect("frozen scenario-neutral contract");
    assert_eq!(evidence["probability_policy"]["scenario_neutral_metrics"], true);
    assert_eq!(evidence["probability_policy"]["probabilities_present"], false);
    assert!(stress["scenario_manifest"].as_array().unwrap()
        .iter().all(|scenario| scenario.get("probability").is_none()));
}
