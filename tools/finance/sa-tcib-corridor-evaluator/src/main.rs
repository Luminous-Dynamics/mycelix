#![forbid(unsafe_code)]

//! Effects-disabled, fail-closed SADC TCIB corridor qualification evaluator.
//! Only an explicitly synthetic test pair may reach the positive disposition.

use jsonschema::draft202012;
use serde_json::{Value, json};
use std::collections::BTreeSet;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

const DEFAULT_CORPUS: &str =
    include_str!("../../../../mycelix-finance/docs/sa-tcib-corridor-corpus-v1.json");
const DEFAULT_SCHEMA: &str =
    include_str!("../../../../mycelix-finance/docs/sa-tcib-corridor-corpus-v1.schema.json");
const EXPECTED_SCHEMA: &str = "mycelix.sa-tcib-corridor/v1";
const EXPECTED_CORPUS: &str = "ZA-SADC-TCIB-CORRIDOR-SYNTHETIC-V1";

fn str_at<'a>(value: &'a Value, key: &str) -> Option<&'a str> {
    value.get(key).and_then(Value::as_str)
}

fn u64_at(value: &Value, key: &str) -> Option<u64> {
    value.get(key).and_then(Value::as_u64)
}

fn array_at<'a>(value: &'a Value, key: &str) -> Option<&'a Vec<Value>> {
    value.get(key).and_then(Value::as_array)
}

fn schema_errors(schema: &Value, corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if !jsonschema::meta::is_valid(schema) {
        errors.push("schema is invalid against the JSON Schema meta-schema".into());
        return errors;
    }
    let validator = match draft202012::options()
        .should_validate_formats(true)
        .build(schema)
    {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("cannot compile Draft 2020-12 schema: {error}"));
            return errors;
        }
    };
    for error in validator.iter_errors(corpus) {
        errors.push(format!(
            "schema error at {}: {error}",
            error.instance_path()
        ));
    }
    errors
}

fn unique_ids(values: &[Value], field: &str, path: &str, errors: &mut Vec<String>) {
    let mut seen = BTreeSet::new();
    for (index, value) in values.iter().enumerate() {
        match str_at(value, field) {
            Some(id) if seen.insert(id.to_owned()) => {}
            Some(id) => errors.push(format!("{path}[{index}].{field}: duplicate {id}")),
            None => errors.push(format!("{path}[{index}].{field}: missing string")),
        }
    }
}

fn source_ids_resolve(refs: &[Value], ids: &BTreeSet<String>) -> bool {
    !refs.is_empty()
        && refs
            .iter()
            .all(|value| value.as_str().is_some_and(|id| ids.contains(id)))
}

fn derive_disposition(scenario: &Value, profile: &Value) -> &'static str {
    let route = &scenario["route"];
    let fx = &scenario["fx"];
    let c = &scenario["constraints"];

    if str_at(profile, "corridor_state") == Some("planned_programme_target") {
        return "planned_not_executable";
    }

    let origin_jurisdiction = str_at(route, "origin_jurisdiction").unwrap_or("");
    let destination_jurisdiction = str_at(route, "destination_jurisdiction").unwrap_or("");
    let origin_rail = str_at(route, "origin_rail").unwrap_or("");
    let destination_rail = str_at(route, "destination_rail").unwrap_or("");
    if matches!(origin_rail, "PayShap") || matches!(destination_rail, "PayShap") {
        if origin_jurisdiction != destination_jurisdiction {
            return "block_domestic_rail_cross_border";
        }
    }

    let origin_participant = str_at(route, "origin_participant").unwrap_or("");
    let destination_participant = str_at(route, "destination_participant").unwrap_or("");
    if matches!(origin_participant, "" | "unknown")
        || matches!(destination_participant, "" | "unknown")
        || matches!(origin_jurisdiction, "" | "unknown")
        || matches!(destination_jurisdiction, "" | "unknown")
        || str_at(c, "participant_pair_state") == Some("unknown")
    {
        return "block_unknown_counterparty";
    }
    if str_at(c, "participant_pair_state") == Some("one_side_only") {
        return "block_participant_pair_incomplete";
    }
    if str_at(c, "participant_pair_state") == Some("suspended") {
        return "block_corridor_unavailable";
    }

    if str_at(c, "message_profile_state") != Some("matched_synthetic")
        || str_at(route, "message_version_origin") != str_at(route, "message_version_destination")
    {
        return "block_message_profile_mismatch";
    }

    match str_at(c, "operating_calendar_state").unwrap_or("unknown") {
        "unknown" => return "block_operating_calendar_unknown",
        "cutoff_closed" => return "block_operating_window_closed",
        "qualified_synthetic" => {}
        _ => return "block_operating_calendar_unknown",
    }

    match str_at(c, "currency_pair_state").unwrap_or("unknown") {
        "rounding_profile_mismatch" => return "block_currency_arithmetic_profile_mismatch",
        "qualified_synthetic" => {}
        _ => return "block_currency_pair_unqualified",
    }

    if c["fx_quote_current"].as_bool() == Some(false) {
        return "block_expired_fx_quote";
    }
    if str_at(c, "fx_quote_state") != Some("executable_synthetic")
        || str_at(fx, "quote_state") != Some("executable_synthetic")
    {
        return "block_non_executable_fx";
    }
    if str_at(c, "fx_direction") != Some("origin_to_destination")
        || str_at(fx, "rate_direction") != Some("ZAR_per_USD")
    {
        return "block_fx_direction_mismatch";
    }
    if c["fx_fee_disclosed"].as_bool() != Some(true) {
        return "block_undisclosed_fx_cost";
    }

    match str_at(c, "funding_state").unwrap_or("unknown") {
        "unknown" => return "block_funding_unqualified",
        "unavailable" => return "block_funding_unavailable",
        "qualified_synthetic" => {}
        _ => return "block_funding_unqualified",
    }

    if str_at(c, "settlement_asset_state") != Some("qualified_synthetic") {
        return "block_settlement_asset_unknown";
    }
    if c["pvp_claimed"].as_bool() == Some(true)
        && str_at(c, "pvp_arrangement_evidence").is_none_or(str::is_empty)
    {
        return "block_unsupported_pvp_claim";
    }

    if str_at(c, "finality_claim") == Some("qualified_synthetic")
        && str_at(c, "legal_finality_evidence").is_none_or(str::is_empty)
    {
        return "block_unsupported_finality_claim";
    }

    let origin_settled = str_at(c, "origin_leg_state") == Some("settled_synthetic")
        && str_at(c, "origin_leg_receipt").is_some_and(|value| !value.is_empty());
    let destination_settled = str_at(c, "destination_leg_state") == Some("settled_synthetic")
        && str_at(c, "destination_leg_receipt").is_some_and(|value| !value.is_empty());
    if !origin_settled || !destination_settled {
        return "block_partial_or_unknown_settlement";
    }

    if str_at(c, "jurisdictional_compliance_state") != Some("qualified_synthetic") {
        return "block_jurisdictional_compliance_unknown";
    }
    if str_at(c, "screening_state") != Some("qualified_synthetic") {
        return "block_screening_unqualified";
    }
    if str_at(c, "fraud_status_state") != Some("origin_authenticated") {
        return "block_fraud_status_laundering";
    }
    if str_at(c, "data_audience_state") != Some("purpose_bound") {
        return "block_data_disclosure_out_of_scope";
    }

    let returned = u64_at(c, "returns_claimed_minor_units").unwrap_or(u64::MAX);
    if returned > 0
        && (str_at(c, "recourse_state") != Some("returned")
            || str_at(c, "independent_return_receipt").is_none_or(str::is_empty)
            || str_at(c, "independent_return_reconciliation").is_none_or(str::is_empty))
    {
        return "block_false_recovery_claim";
    }

    if str_at(profile, "corridor_state") == Some("synthetic_test_only")
        && str_at(profile, "profile_scope") == Some("effects_disabled_simulator_only")
    {
        "qualify_synthetic_test_only"
    } else {
        "block_participant_pair_incomplete"
    }
}

fn semantic_errors(corpus: &Value) -> Vec<String> {
    let mut errors = Vec::new();
    if str_at(corpus, "schema") != Some(EXPECTED_SCHEMA)
        || str_at(corpus, "corpus_id") != Some(EXPECTED_CORPUS)
        || str_at(corpus, "status") != Some("synthetic_reference_only")
        || corpus.get("external_effects_enabled").and_then(Value::as_bool) != Some(false)
    {
        errors.push("corpus identity/status/effects boundary mismatch".into());
    }
    let as_of = str_at(corpus, "as_of").unwrap_or("");
    let sources = array_at(corpus, "source_register").cloned().unwrap_or_default();
    let profiles = array_at(corpus, "corridor_profiles").cloned().unwrap_or_default();
    let scenarios = array_at(corpus, "scenarios").cloned().unwrap_or_default();

    unique_ids(&sources, "id", "source_register", &mut errors);
    unique_ids(&profiles, "profile_id", "corridor_profiles", &mut errors);
    unique_ids(&scenarios, "scenario_id", "scenarios", &mut errors);

    let source_ids: BTreeSet<String> = sources
        .iter()
        .filter_map(|source| str_at(source, "id").map(str::to_owned))
        .collect();
    let profile_map: std::collections::BTreeMap<String, Value> = profiles
        .iter()
        .filter_map(|profile| {
            str_at(profile, "profile_id").map(|id| (id.to_owned(), profile.clone()))
        })
        .collect();

    for (index, source) in sources.iter().enumerate() {
        if str_at(source, "observed_on").is_none_or(|date| date > as_of) {
            errors.push(format!("source_register[{index}]: invalid or future observation date"));
        }
    }

    for (index, profile) in profiles.iter().enumerate() {
        let refs = array_at(profile, "source_refs").cloned().unwrap_or_default();
        if !source_ids_resolve(&refs, &source_ids) {
            errors.push(format!("corridor_profiles[{index}]: missing or unresolved source refs"));
        }
        let state = str_at(profile, "corridor_state").unwrap_or("");
        if state == "planned_programme_target" {
            if profile.get("effective_from").is_some_and(|value| !value.is_null()) {
                errors.push(format!(
                    "corridor_profiles[{index}]: planned programme target cannot assert effective_from"
                ));
            }
            if profile.get("programme_target_amount_minor_units").is_none_or(Value::is_null) {
                errors.push(format!(
                    "corridor_profiles[{index}]: planned target amount must remain explicit"
                ));
            }
        }
        if state != "synthetic_test_only"
            && profile.get("programme_target_amount_minor_units").is_some_and(|v| !v.is_null())
            && state != "planned_programme_target"
        {
            errors.push(format!(
                "corridor_profiles[{index}]: programme target amount cannot be used as a live limit"
            ));
        }
    }

    for (index, scenario) in scenarios.iter().enumerate() {
        let path = format!("scenarios[{index}]");
        let profile_id = str_at(scenario, "profile_id").unwrap_or("");
        let Some(profile) = profile_map.get(profile_id) else {
            errors.push(format!("{path}: unresolved corridor profile {profile_id}"));
            continue;
        };
        if scenario.get("external_effects_enabled").and_then(Value::as_bool) != Some(false) {
            errors.push(format!("{path}: scenario external effects must remain disabled"));
        }
        if str_at(scenario, "status") != Some("synthetic_reference_only") {
            errors.push(format!("{path}: scenario must remain synthetic_reference_only"));
        }

        let route = &scenario["route"];
        if u64_at(route, "amount_minor_units").unwrap_or(0) == 0 {
            errors.push(format!("{path}: origin amount must be positive"));
        }
        let fx = &scenario["fx"];
        if u64_at(fx, "rate_numerator").unwrap_or(0) == 0
            || u64_at(fx, "rate_denominator").unwrap_or(0) == 0
        {
            errors.push(format!("{path}: FX rate numerator and denominator must be positive integers"));
        }
        if str_at(fx, "observed_at").is_none_or(str::is_empty)
            || str_at(fx, "expires_at").is_none_or(str::is_empty)
        {
            errors.push(format!("{path}: FX quote requires explicit observation and expiry"));
        }

        let required = array_at(scenario, "required_evidence").cloned().unwrap_or_default();
        let controls = array_at(scenario, "negative_controls").cloned().unwrap_or_default();
        if required.is_empty() || controls.is_empty() {
            errors.push(format!("{path}: required evidence and negative controls must be explicit"));
        }

        let actual = derive_disposition(scenario, profile);
        if str_at(scenario, "expected_disposition") != Some(actual) {
            errors.push(format!(
                "{path}: expected disposition does not match fail-closed corridor assessment ({actual})"
            ));
        }
        if actual == "qualify_synthetic_test_only"
            && (str_at(profile, "corridor_state") != Some("synthetic_test_only")
                || str_at(profile, "profile_scope") != Some("effects_disabled_simulator_only"))
        {
            errors.push(format!("{path}: positive qualification requires an explicit simulator-only profile"));
        }
    }

    errors
}

fn evaluate(corpus_text: &str, schema_text: &str) -> Result<(Vec<String>, Value, Value), String> {
    let corpus: Value = serde_json::from_str(corpus_text)
        .map_err(|error| format!("corpus JSON parse error: {error}"))?;
    let schema: Value = serde_json::from_str(schema_text)
        .map_err(|error| format!("schema JSON parse error: {error}"))?;
    let mut errors = semantic_errors(&corpus);
    errors.extend(schema_errors(&schema, &corpus));
    Ok((errors, corpus, schema))
}

fn mutate_and_reject(
    label: &str,
    corpus: &Value,
    schema: &Value,
    mutation: impl FnOnce(&mut Value),
) -> Result<(), String> {
    let mut changed = corpus.clone();
    mutation(&mut changed);
    let mut errors = semantic_errors(&changed);
    errors.extend(schema_errors(schema, &changed));
    if errors.is_empty() {
        return Err(format!("negative control was not rejected: {label}"));
    }
    Ok(())
}

fn run_self_tests(corpus: &Value, schema: &Value) -> Result<usize, String> {
    let baseline = semantic_errors(corpus);
    let mut structural = schema_errors(schema, corpus);
    if !baseline.is_empty() || !structural.is_empty() {
        structural.extend(baseline);
        return Err(format!("baseline corpus failed: {structural:?}"));
    }
    let mut count = 0usize;

    mutate_and_reject("enable external effects", corpus, schema, |value| {
        value["external_effects_enabled"] = json!(true);
    })?;
    count += 1;

    mutate_and_reject("planned programme becomes executable", corpus, schema, |value| {
        value["corridor_profiles"][1]["corridor_state"] = json!("synthetic_test_only");
        value["scenarios"][1]["profile_id"] = json!("SADC-TCIB-FUTURE-PROGRAMME-V1");
    })?;
    count += 1;

    mutate_and_reject("domestic PayShap used cross-border", corpus, schema, |value| {
        value["scenarios"][0]["route"]["origin_jurisdiction"] = json!("ZA");
        value["scenarios"][0]["route"]["destination_jurisdiction"] = json!("SYNTHETIC-SADC");
    })?;
    count += 1;

    mutate_and_reject("participant pair promoted from unknown", corpus, schema, |value| {
        value["scenarios"][3]["constraints"]["participant_pair_state"] = json!("qualified_synthetic");
        value["scenarios"][3]["route"]["destination_participant"] = json!("SYNTHETIC-PSP-SADC-B");
    })?;
    count += 1;

    mutate_and_reject("expired FX quote treated as current", corpus, schema, |value| {
        value["scenarios"][11]["constraints"]["fx_quote_current"] = json!(true);
        value["scenarios"][11]["expected_disposition"] = json!("qualify_synthetic_test_only");
    })?;
    count += 1;

    mutate_and_reject("FX direction reversed", corpus, schema, |value| {
        value["scenarios"][12]["constraints"]["fx_direction"] = json!("origin_to_destination");
    })?;
    count += 1;

    mutate_and_reject("funding unknown treated as available", corpus, schema, |value| {
        value["scenarios"][15]["constraints"]["funding_state"] = json!("qualified_synthetic");
    })?;
    count += 1;

    mutate_and_reject("PvP claim without evidence admitted", corpus, schema, |value| {
        value["scenarios"][18]["constraints"]["pvp_arrangement_evidence"] = json!("SYNTHETIC-PVP-CLAIM");
    })?;
    count += 1;

    mutate_and_reject("one-leg settlement promoted to finality", corpus, schema, |value| {
        value["scenarios"][19]["constraints"]["destination_leg_state"] = json!("settled_synthetic");
        value["scenarios"][19]["constraints"]["destination_leg_receipt"] = json!("SYNTHETIC-UNSUPPORTED-RECEIPT");
        value["scenarios"][19]["expected_disposition"] = json!("qualify_synthetic_test_only");
    })?;
    count += 1;

    mutate_and_reject("unqualified screening admitted", corpus, schema, |value| {
        value["scenarios"][21]["constraints"]["screening_state"] = json!("qualified_synthetic");
    })?;
    count += 1;

    mutate_and_reject("relay promotes fraud status", corpus, schema, |value| {
        value["scenarios"][22]["constraints"]["fraud_status_state"] = json!("origin_authenticated");
    })?;
    count += 1;

    mutate_and_reject("outside-audience data use admitted", corpus, schema, |value| {
        value["scenarios"][23]["constraints"]["data_audience_state"] = json!("purpose_bound");
    })?;
    count += 1;

    mutate_and_reject("hold counted as returned funds", corpus, schema, |value| {
        value["scenarios"][24]["constraints"]["recourse_state"] = json!("returned");
        value["scenarios"][24]["constraints"]["independent_return_receipt"] = json!("RETURN-RECEIPT");
        value["scenarios"][24]["constraints"]["independent_return_reconciliation"] = json!("LEDGER-RECONCILIATION");
    })?;
    count += 1;

    mutate_and_reject("positive synthetic qualification without destination receipt", corpus, schema, |value| {
        value["scenarios"][28]["constraints"]["destination_leg_receipt"] = Value::Null;
    })?;
    count += 1;

    Ok(count)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut corpus_text = DEFAULT_CORPUS.to_owned();
    let mut schema_text = DEFAULT_SCHEMA.to_owned();
    let mut self_test = false;
    let mut args = std::env::args().skip(1);

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--corpus" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--corpus requires a path",
                    )
                })?);
                corpus_text = fs::read_to_string(path)?;
            }
            "--schema" => {
                let path = PathBuf::from(args.next().ok_or_else(|| {
                    std::io::Error::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "--schema requires a path",
                    )
                })?);
                schema_text = fs::read_to_string(path)?;
            }
            "--self-test" => self_test = true,
            "--help" | "-h" => {
                println!("sa-tcib-corridor-evaluator [--corpus PATH] [--schema PATH] [--self-test]");
                return Ok(());
            }
            other => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("unknown argument: {other}"),
                )
                .into());
            }
        }
    }

    let (errors, corpus, schema) = evaluate(&corpus_text, &schema_text)
        .map_err(|message| std::io::Error::new(std::io::ErrorKind::InvalidData, message))?;
    let negative_controls = if self_test {
        run_self_tests(&corpus, &schema)?
    } else {
        0
    };
    let status = if errors.is_empty() { "PASS" } else { "FAIL" };
    let report = json!({
        "evaluator": "mycelix.sa-tcib-corridor-evaluator.v1",
        "status": status,
        "external_effects_enabled": false,
        "claim_ceiling": "source/profile/schema and synthetic corridor readiness conformance only",
        "counts": {
            "sources": array_at(&corpus, "source_register").map_or(0, Vec::len),
            "profiles": array_at(&corpus, "corridor_profiles").map_or(0, Vec::len),
            "scenarios": array_at(&corpus, "scenarios").map_or(0, Vec::len),
            "negative_controls": negative_controls
        },
        "errors": errors
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    if status != "PASS" {
        std::process::exit(1);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn schema_and_corridor_corpus_validate() {
        let (errors, corpus, _) = evaluate(DEFAULT_CORPUS, DEFAULT_SCHEMA).expect("evaluation");
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(array_at(&corpus, "source_register").unwrap().len(), 4);
        assert_eq!(array_at(&corpus, "corridor_profiles").unwrap().len(), 3);
        assert_eq!(array_at(&corpus, "scenarios").unwrap().len(), 30);
    }

    #[test]
    fn deterministic_negative_mutation_campaign_rejects_corridor_unsafe_states() {
        let corpus: Value = serde_json::from_str(DEFAULT_CORPUS).expect("corpus");
        let schema: Value = serde_json::from_str(DEFAULT_SCHEMA).expect("schema");
        assert_eq!(run_self_tests(&corpus, &schema).expect("self tests"), 14);
    }
}
