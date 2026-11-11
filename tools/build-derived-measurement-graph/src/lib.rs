//! Synthetic derivation graph qualifier.
//! Only the synthetic scalar test method is executed; real building methods remain blocked.

#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use std::fmt;

pub const PROFILE_JSON: &str = include_str!("../../../docs/metrology/BUILD_DERIVED_MEASUREMENT_PROFILE_V0_1.json");
pub const CORPUS_JSON: &str = include_str!("../../../docs/metrology/BUILD_DERIVED_MEASUREMENT_CORPUS_V0_1.json");
pub const PROFILE_SCHEMA_JSON: &str = include_str!("../../../docs/metrology/BUILD_DERIVED_MEASUREMENT_PROFILE_V0_1.schema.json");
pub const CORPUS_SCHEMA_JSON: &str = include_str!("../../../docs/metrology/BUILD_DERIVED_MEASUREMENT_CORPUS_V0_1.schema.json");

const PROFILE_ID: &str = "MYC-BUILD-DERIVED-GRAPH-V0.1";
const CORPUS_ID: &str = "BUILD-DERIVED-MEASUREMENT-GRAPH-CORPUS-V0.1";

const RECORD_FIELDS: &[&str] = &[
    "schema_version","derivation_id","claim_ceiling","calculation_system","output","method",
    "inputs","selection","conditions","uncertainty_model","source_resolution","invalidation","test_only",
];
const METHOD_FIELDS: &[&str] = &[
    "method_id","revision","output_channel_id","output_measurand_id","output_quantity_kind","output_unit",
    "calculation_kind","artifact_digest","runtime_id","method_reference","method_status","required_inputs","admission_note",
];
const METHOD_INPUT_FIELDS: &[&str] = &["role","ordinal","channel_id","measurand_id","unit"];
const TEST_METHOD_INPUT_FIELDS: &[&str] = &[
    "role","ordinal","channel_id","measurand_id","unit","firmware_revision","source_revision",
];
const CALCULATION_SYSTEM_FIELDS: &[&str] = &["system_id","revision","configuration_digest","runtime_id"];
const OUTPUT_FIELDS: &[&str] = &[
    "observation_id","channel_id","measurand_id","quantity_kind","unit","value","semantic_kind",
    "quality_state","space_id","geometry_revision","window",
];
const OUTPUT_WINDOW_FIELDS: &[&str] = &[
    "window_id","start_tick","end_tick","tick_unit","clock_domain_id","clock_epoch_id",
];
const METHOD_RECORD_FIELDS: &[&str] = &[
    "method_id","revision","artifact_digest","runtime_id","expression_kind","input_contract_digest",
    "source_method_reference","method_state",
];
const SELECTION_FIELDS: &[&str] = &[
    "ordered_input_observation_ids","time_alignment_policy","interpolation_policy","missingness_policy",
    "aggregation_window_id","expected_input_count",
];
const CONDITIONS_FIELDS: &[&str] = &["conditions_profile_id","conditions_digest","applicability_state"];
const UNCERTAINTY_FIELDS: &[&str] = &["model_id","revision","digest","state"];
const SOURCE_RESOLUTION_FIELDS: &[&str] = &["state","resolved_input_count","source_graph_digest"];
const INVALIDATION_FIELDS: &[&str] = &["state","receipt_id","graph_revision"];
const INPUT_FIELDS: &[&str] = &[
    "ordinal","role","observation_id","source_digest","source_revision","source_alias","instrument_id",
    "channel_id","channel_profile_id","measurand_id","unit","value","evidence_state","calibration_record_id",
    "calibration_state","installation_record_id","installation_state","firmware_revision","stream_id",
    "clock_domain_id","clock_epoch_id","timebase_profile_id","event_time_tick","sequence","rights_state",
    "parent_derivation_id","parent_graph_digest",
];
const EXPECTED_CASES: &[(&str,&str)] = &[
    ("DMG-001","SYNTHETIC_GRAPH_VALIDATION_ONLY"),
    ("DMG-002","MORE_EVIDENCE_REQUIRED"),
    ("DMG-003","MORE_EVIDENCE_REQUIRED"),
    ("DMG-004","MORE_EVIDENCE_REQUIRED"),
    ("DMG-005","NOT_COMPARABLE"),
    ("DMG-006","MORE_EVIDENCE_REQUIRED"),
    ("DMG-007","NOT_COMPARABLE"),
    ("DMG-008","EXPIRED_OR_INVALIDATED"),
    ("DMG-009","EXPIRED_OR_INVALIDATED"),
    ("DMG-010","EXPIRED_OR_INVALIDATED"),
    ("DMG-011","EXPIRED_OR_INVALIDATED"),
    ("DMG-012","EXPIRED_OR_INVALIDATED"),
    ("DMG-013","NOT_COMPARABLE"),
    ("DMG-014","NOT_COMPARABLE"),
    ("DMG-015","MORE_EVIDENCE_REQUIRED"),
    ("DMG-016","MORE_EVIDENCE_REQUIRED"),
    ("DMG-017","NOT_COMPARABLE"),
    ("DMG-018","MORE_EVIDENCE_REQUIRED"),
    ("DMG-019","EXPIRED_OR_INVALIDATED"),
    ("DMG-020","MORE_EVIDENCE_REQUIRED"),
    ("DMG-021","MORE_EVIDENCE_REQUIRED"),
    ("DMG-022","REJECTED_FOR_DECLARED_PURPOSE"),
    ("DMG-023","MORE_EVIDENCE_REQUIRED"),
    ("DMG-024","NOT_COMPARABLE"),
    ("DMG-025","NOT_COMPARABLE"),
    ("DMG-026","NOT_COMPARABLE"),
    ("DMG-027","MORE_EVIDENCE_REQUIRED"),
    ("DMG-028","MORE_EVIDENCE_REQUIRED"),
    ("DMG-029","NOT_COMPARABLE"),
    ("DMG-030","NOT_COMPARABLE"),
    ("DMG-031","MORE_EVIDENCE_REQUIRED"),
    ("DMG-032","MORE_EVIDENCE_REQUIRED"),
    ("DMG-033","NOT_COMPARABLE"),
    ("DMG-034","EXPIRED_OR_INVALIDATED"),
    ("DMG-035","MORE_EVIDENCE_REQUIRED"),
    ("DMG-036","REJECTED_CLAIM_PROMOTION"),
];

#[derive(Debug)]
struct UniqueJson(Value);
impl<'de> Deserialize<'de> for UniqueJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where D: Deserializer<'de> {
        deserializer.deserialize_any(UniqueJsonVisitor)
    }
}
struct UniqueJsonVisitor;
impl<'de> Visitor<'de> for UniqueJsonVisitor {
    type Value = UniqueJson;
    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a JSON value with no duplicate member names")
    }
    fn visit_bool<E>(self, v: bool) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::Bool(v)))
    }
    fn visit_i64<E>(self, v: i64) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::Number(v.into())))
    }
    fn visit_u64<E>(self, v: u64) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::Number(v.into())))
    }
    fn visit_f64<E>(self, v: f64) -> Result<Self::Value, E> where E: serde::de::Error {
        let n = serde_json::Number::from_f64(v).ok_or_else(|| E::custom("non-finite JSON number"))?;
        Ok(UniqueJson(Value::Number(n)))
    }
    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::String(v.to_owned())))
    }
    fn visit_string<E>(self, v: String) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::String(v)))
    }
    fn visit_unit<E>(self) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::Null))
    }
    fn visit_none<E>(self) -> Result<Self::Value, E> where E: serde::de::Error {
        Ok(UniqueJson(Value::Null))
    }
    fn visit_some<D>(self, d: D) -> Result<Self::Value, D::Error>
    where D: Deserializer<'de> { UniqueJson::deserialize(d) }
    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where A: SeqAccess<'de> {
        let mut values = Vec::new();
        while let Some(item) = seq.next_element::<UniqueJson>()? {
            values.push(item.0);
        }
        Ok(UniqueJson(Value::Array(values)))
    }
    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where A: MapAccess<'de> {
        let mut values = Map::new();
        let mut seen = BTreeSet::new();
        while let Some(key) = map.next_key::<String>()? {
            if !seen.insert(key.clone()) {
                return Err(serde::de::Error::custom(format!("duplicate JSON member {key:?}")));
            }
            values.insert(key, map.next_value::<UniqueJson>()?.0);
        }
        Ok(UniqueJson(Value::Object(values)))
    }
}
fn parse_unique(input: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(input);
    let value = UniqueJson::deserialize(&mut deserializer)
        .map_err(|error| format!("STRICT_JSON_REJECTED:{error}"))?;
    deserializer.end().map_err(|error| format!("JSON_TRAILING_DATA:{error}"))?;
    Ok(value.0)
}
fn text<'a>(value: &'a Value, key: &str) -> &'a str {
    value.get(key).and_then(Value::as_str).unwrap_or("")
}
fn finite_number(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64).filter(|n| n.is_finite())
}
fn boolean(value: &Value, key: &str) -> bool {
    value.get(key).and_then(Value::as_bool) == Some(true)
}
fn exact_keys(value: &Value, expected: &[&str]) -> bool {
    value.as_object().is_some_and(|object| {
        object.len() == expected.len() && expected.iter().all(|key| object.contains_key(*key))
    })
}
fn array_strings(value: &Value) -> Option<Vec<&str>> {
    value.as_array()?.iter().map(Value::as_str).collect()
}
fn is_sha256(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64 && hex.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    })
}
fn approximately_equal(left: f64, right: f64) -> bool {
    (left - right).abs() <= 1.0e-12 * left.abs().max(right.abs()).max(1.0)
}

pub fn classify_record(record: &Value, profile: &Value) -> &'static str {
    if !boolean(record, "test_only") || text(record, "claim_ceiling") != "synthetic_graph_validation_only" {
        return "REJECTED_CLAIM_PROMOTION";
    }
    let methods = profile.get("method_profiles").and_then(Value::as_array);
    let Some(method) = methods.and_then(|items| items.iter().find(|item| text(item, "method_id") == text(&record["method"], "method_id"))) else {
        return "NOT_COMPARABLE";
    };
    if text(&record["output"], "channel_id") != text(method, "output_channel_id")
        || text(&record["output"], "measurand_id") != text(method, "output_measurand_id")
        || text(&record["output"], "unit") != text(method, "output_unit")
    {
        return "NOT_COMPARABLE";
    }
    if text(method, "method_status") != "SYNTHETIC_TEST_ONLY" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["output"], "semantic_kind") != "derived" {
        return "NOT_COMPARABLE";
    }
    if text(&record["output"], "quality_state") != "valid" {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(&record["method"], "method_state") == "superseded" {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(&record["method"], "method_state") != "current" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["method"], "revision") != text(method, "revision")
        || text(&record["method"], "artifact_digest") != text(method, "artifact_digest")
        || text(&record["method"], "runtime_id") != text(method, "runtime_id")
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if !is_sha256(text(&record["method"], "artifact_digest"))
        || !is_sha256(text(&record["method"], "input_contract_digest"))
        || !is_sha256(text(&record["calculation_system"], "configuration_digest"))
        || text(&record["calculation_system"], "runtime_id") != text(&record["method"], "runtime_id")
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["calculation_system"], "system_id").is_empty()
        || text(&record["calculation_system"], "revision").is_empty()
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["output"], "geometry_revision").is_empty()
        || text(&record["output"], "space_id").is_empty()
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["selection"], "interpolation_policy") != "none"
        || text(&record["selection"], "missingness_policy") != "abstain"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["selection"], "time_alignment_policy") != "exact_same_clock_epoch_and_tick"
        || text(&record["selection"], "aggregation_window_id") != text(&record["output"]["window"], "window_id")
    {
        return "NOT_COMPARABLE";
    }
    if text(&record["conditions"], "applicability_state") != "within_declared_synthetic_envelope"
        || text(&record["conditions"], "conditions_profile_id").is_empty()
        || !is_sha256(text(&record["conditions"], "conditions_digest"))
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["uncertainty_model"], "state") != "synthetic_inputs_present_only"
        || !is_sha256(text(&record["uncertainty_model"], "digest"))
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(&record["invalidation"], "state") != "current"
        || text(&record["invalidation"], "receipt_id").is_empty()
        || text(&record["invalidation"], "graph_revision") != "1"
    {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(&record["source_resolution"], "state") != "all_exact_sources_resolved"
        || !is_sha256(text(&record["source_resolution"], "source_graph_digest"))
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    let inputs = record.get("inputs").and_then(Value::as_array);
    let Some(inputs) = inputs else { return "MORE_EVIDENCE_REQUIRED"; };
    let expected_inputs = method.get("required_inputs").and_then(Value::as_array);
    let Some(expected_inputs) = expected_inputs else { return "MORE_EVIDENCE_REQUIRED"; };
    if inputs.len() != expected_inputs.len()
        || record["selection"]["expected_input_count"].as_u64() != Some(expected_inputs.len() as u64)
        || record["source_resolution"]["resolved_input_count"].as_u64() != Some(inputs.len() as u64)
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    let ordered_ids = record["selection"]["ordered_input_observation_ids"].as_array();
    let Some(ordered_ids) = ordered_ids else { return "MORE_EVIDENCE_REQUIRED"; };
    if ordered_ids.len() != inputs.len() {
        return "NOT_COMPARABLE";
    }
    let mut input_ids = BTreeSet::new();
    let mut ordinals = BTreeSet::new();
    let mut values = Vec::new();
    let window = &record["output"]["window"];
    let window_start = window["start_tick"].as_i64();
    let window_end = window["end_tick"].as_i64();
    let mut common_clock = None::<(&str, &str)>;
    let mut common_tick = None::<i64>;
    for (index, (input, expected)) in inputs.iter().zip(expected_inputs).enumerate() {
        let observation_id = text(input, "observation_id");
        if observation_id.is_empty() || !input_ids.insert(observation_id.to_owned()) {
            return "NOT_COMPARABLE";
        }
        if ordered_ids[index].as_str() != Some(observation_id)
            || input["ordinal"].as_u64() != Some(index as u64)
            || !ordinals.insert(index)
        {
            return "NOT_COMPARABLE";
        }
        if text(input, "role") != text(expected, "role")
            || text(input, "channel_id") != text(expected, "channel_id")
            || text(input, "measurand_id") != text(expected, "measurand_id")
            || text(input, "unit") != text(expected, "unit")
        {
            return "NOT_COMPARABLE";
        }
        if let Some(firmware) = expected.get("firmware_revision").and_then(Value::as_str) {
            if text(input, "firmware_revision") != firmware {
                return "EXPIRED_OR_INVALIDATED";
            }
        }
        if let Some(source_revision) = expected.get("source_revision").and_then(Value::as_str) {
            if text(input, "source_revision") != source_revision {
                return "EXPIRED_OR_INVALIDATED";
            }
        }
        if text(input, "source_alias") != "exact" {
            return "NOT_COMPARABLE";
        }
        if !is_sha256(text(input, "source_digest")) || text(input, "source_revision").is_empty() {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if text(input, "evidence_state") != "current"
            || text(input, "calibration_state") != "current"
            || text(input, "installation_state") != "current"
        {
            return "EXPIRED_OR_INVALIDATED";
        }
        if text(input, "rights_state") != "authorized" {
            return "REJECTED_FOR_DECLARED_PURPOSE";
        }
        if text(input, "instrument_id").is_empty()
            || text(input, "calibration_record_id").is_empty()
            || text(input, "installation_record_id").is_empty()
            || text(input, "firmware_revision").is_empty()
            || text(input, "stream_id").is_empty()
            || text(input, "timebase_profile_id").is_empty()
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
        let clock = (text(input, "clock_domain_id"), text(input, "clock_epoch_id"));
        if clock.0.is_empty() || clock.1.is_empty() {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if common_clock.is_some_and(|previous| previous != clock) {
            return "NOT_COMPARABLE";
        }
        common_clock = Some(clock);
        let tick = input["event_time_tick"].as_i64();
        if tick.is_none() || input["sequence"].as_u64().unwrap_or(0) == 0 {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if common_tick.is_some_and(|previous| Some(previous) != tick) {
            return "NOT_COMPARABLE";
        }
        common_tick = tick;
        if tick.zip(window_start).zip(window_end).is_none_or(|((tick, start), end)| tick < start || tick > end) {
            return "NOT_COMPARABLE";
        }
        if text(input, "parent_derivation_id") == text(record, "derivation_id") {
            return "NOT_COMPARABLE";
        }
        if text(input, "parent_derivation_id") != "NOT_APPLICABLE"
            && (!is_sha256(text(input, "parent_graph_digest")) || text(input, "parent_derivation_id").is_empty())
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
        let Some(value) = finite_number(input, "value") else { return "MORE_EVIDENCE_REQUIRED"; };
        values.push(value);
    }
    if values.len() != 2 || text(&record["method"], "expression_kind") != "arithmetic_mean" {
        return "NOT_COMPARABLE";
    }
    let recomputed = values.iter().sum::<f64>() / values.len() as f64;
    let Some(declared) = finite_number(&record["output"], "value") else { return "MORE_EVIDENCE_REQUIRED"; };
    if !approximately_equal(recomputed, declared) {
        return "NOT_COMPARABLE";
    }
    "SYNTHETIC_GRAPH_VALIDATION_ONLY"
}

fn validate_profile(profile: &Value) -> Result<(), String> {
    if !exact_keys(profile, &[
        "schema_version","profile_id","base_channel_profile_id","classification","claim_ceiling",
        "real_physical_method_execution_enabled","method_profiles","invariants",
    ]) || text(profile, "schema_version") != "mycelix.derived-measurement-graph/0.1"
        || text(profile, "profile_id") != PROFILE_ID
        || text(profile, "base_channel_profile_id") != "MYC-BUILD-METROLOGY-PROFILE-V0.1"
        || text(profile, "classification") != "synthetic_graph_contract_with_blocked_physical_method_profiles"
        || text(profile, "claim_ceiling") != "synthetic_dependency_resolution_and_test_method_recomputation_only"
        || boolean(profile, "real_physical_method_execution_enabled")
    {
        return Err("DERIVED_PROFILE_IDENTITY_OR_CLAIM_CEILING_INVALID".into());
    }
    let methods = profile["method_profiles"].as_array().ok_or("METHOD_PROFILES_MISSING")?;
    if methods.len() != 3 {
        return Err("METHOD_PROFILE_COUNT_INVALID".into());
    }
    let expected_ids = [
        "synthetic.scalar_mean.v1",
        "thermal.mrt.six_direction_plane_radiant.v1",
        "thermal.useful_energy.flow_temperature_integral.v1",
    ];
    for (method, expected_id) in methods.iter().zip(expected_ids) {
        if !exact_keys(method, METHOD_FIELDS)
            || text(method, "method_id") != expected_id
            || !is_sha256(text(method, "artifact_digest"))
            || method["required_inputs"].as_array().is_none_or(|items| items.is_empty())
        {
            return Err(format!("METHOD_PROFILE_INVALID:{expected_id}"));
        }
    }
    let synthetic = &methods[0];
    if text(synthetic,"output_channel_id")!="synthetic_derived_scalar"
        ||text(synthetic,"output_measurand_id")!="synthetic.mean_scalar"
        ||text(synthetic,"output_quantity_kind")!="ratio"
        ||text(synthetic,"output_unit")!="1"
        ||text(synthetic,"calculation_kind")!="arithmetic_mean"
        ||text(synthetic,"method_status")!="SYNTHETIC_TEST_ONLY"
        ||text(synthetic,"runtime_id")!="luminous-derived-oracle-synthetic/1.0.0"
    {return Err("SYNTHETIC_METHOD_PROFILE_DRIFT".into());}
    let synthetic_inputs=synthetic["required_inputs"].as_array().ok_or("SYNTHETIC_METHOD_INPUTS_MISSING")?;
    if synthetic_inputs.len()!=2 {return Err("SYNTHETIC_METHOD_INPUT_COUNT_INVALID".into());}
    for (index,input) in synthetic_inputs.iter().enumerate() {
        if !exact_keys(input,TEST_METHOD_INPUT_FIELDS)
            ||text(input,"role")!="sample"
            ||input["ordinal"].as_u64()!=Some(index as u64)
            ||text(input,"channel_id")!="synthetic_scalar"
            ||text(input,"measurand_id")!="synthetic.scalar"
            ||text(input,"unit")!="1"
            ||text(input,"firmware_revision")!="FW-1"
            ||text(input,"source_revision")!="1"
        {return Err("SYNTHETIC_METHOD_INPUT_CONTRACT_DRIFT".into());}
    }
    let mrt=&methods[1];
    if text(mrt,"output_channel_id")!="mean_radiant_temperature"
        ||text(mrt,"output_unit")!="degC"
        ||text(mrt,"method_status")!="BLOCKED_PENDING_INPUT_CHANNEL_ADMISSION"
        ||array_strings(&mrt["required_inputs"]).is_some()
    {return Err("MRT_METHOD_PROFILE_DRIFT".into());}
    let mrt_inputs=mrt["required_inputs"].as_array().ok_or("MRT_METHOD_INPUTS_MISSING")?;
    let mrt_directions=["up","down","left","right","front","back"];
    if mrt_inputs.len()!=mrt_directions.len() {return Err("MRT_INPUT_COUNT_INVALID".into());}
    for (index,(input,direction)) in mrt_inputs.iter().zip(mrt_directions).enumerate() {
        if !exact_keys(input,METHOD_INPUT_FIELDS)||text(input,"role")!=direction
            ||input["ordinal"].as_u64()!=Some(index as u64)
            ||text(input,"channel_id")!="plane_radiant_temperature"
            ||text(input,"measurand_id")!="habitat.plane_radiant_temperature"
            ||text(input,"unit")!="degC"
        {return Err("MRT_INPUT_CONTRACT_DRIFT".into());}
    }
    let thermal=&methods[2];
    if text(thermal,"output_channel_id")!="useful_thermal_energy"
        ||text(thermal,"output_unit")!="kWh_th"
        ||text(thermal,"method_status")!="BLOCKED_PENDING_INPUT_CHANNEL_ADMISSION"
    {return Err("USEFUL_THERMAL_METHOD_PROFILE_DRIFT".into());}
    let thermal_inputs=thermal["required_inputs"].as_array().ok_or("THERMAL_METHOD_INPUTS_MISSING")?;
    let thermal_expected=[
        ("volume_flow","water_volume_flow","habitat.water_volume_flow","m3/s"),
        ("supply_temperature","water_temperature","habitat.water_temperature","degC"),
        ("return_temperature","water_temperature","habitat.water_temperature","degC"),
    ];
    if thermal_inputs.len()!=thermal_expected.len() {return Err("THERMAL_INPUT_COUNT_INVALID".into());}
    for (index,(input,(role,channel,measurand,unit))) in thermal_inputs.iter().zip(thermal_expected).enumerate() {
        if !exact_keys(input,METHOD_INPUT_FIELDS)||text(input,"role")!=role
            ||input["ordinal"].as_u64()!=Some(index as u64)
            ||text(input,"channel_id")!=channel||text(input,"measurand_id")!=measurand
            ||text(input,"unit")!=unit
        {return Err("THERMAL_INPUT_CONTRACT_DRIFT".into());}
    }
    if text(&methods[1], "method_status") != "BLOCKED_PENDING_INPUT_CHANNEL_ADMISSION"
        || text(&methods[2], "method_status") != "BLOCKED_PENDING_INPUT_CHANNEL_ADMISSION"
    {
        return Err("PHYSICAL_METHODS_PROMOTED".into());
    }
    if profile["invariants"].as_object().is_none_or(|object| object.values().any(|value| value.as_bool() != Some(true))) {
        return Err("DERIVED_INVARIANTS_WEAKENED".into());
    }
    Ok(())
}

pub fn validate_corpus(profile: &Value, corpus: &Value) -> Result<usize, String> {
    if !exact_keys(corpus, &[
        "schema_version","corpus_id","profile_id","classification","claim_ceiling","cases",
    ]) || text(corpus, "schema_version") != "mycelix.derived-measurement-graph-corpus/0.1"
        || text(corpus, "corpus_id") != CORPUS_ID
        || text(corpus, "profile_id") != PROFILE_ID
        || text(corpus, "classification") != "synthetic_dependency_graph_and_test_method_cases"
        || text(corpus, "claim_ceiling") != "synthetic_graph_validation_only"
    {
        return Err("DERIVED_CORPUS_IDENTITY_INVALID".into());
    }
    let cases = corpus["cases"].as_array().ok_or("DERIVED_CASES_MISSING")?;
    if cases.len() != EXPECTED_CASES.len() {
        return Err("DERIVED_CASE_COUNT_INVALID".into());
    }
    for (index, (case, (expected_id, expected_disposition))) in cases.iter().zip(EXPECTED_CASES).enumerate() {
        let record=&case["record"];
        let inputs=record["inputs"].as_array().ok_or("DERIVED_INPUTS_MISSING")?;
        let nested_ok=exact_keys(&record["calculation_system"],CALCULATION_SYSTEM_FIELDS)
            &&exact_keys(&record["output"],OUTPUT_FIELDS)
            &&exact_keys(&record["output"]["window"],OUTPUT_WINDOW_FIELDS)
            &&exact_keys(&record["method"],METHOD_RECORD_FIELDS)
            &&exact_keys(&record["selection"],SELECTION_FIELDS)
            &&exact_keys(&record["conditions"],CONDITIONS_FIELDS)
            &&exact_keys(&record["uncertainty_model"],UNCERTAINTY_FIELDS)
            &&exact_keys(&record["source_resolution"],SOURCE_RESOLUTION_FIELDS)
            &&exact_keys(&record["invalidation"],INVALIDATION_FIELDS)
            &&inputs.iter().all(|input|exact_keys(input,INPUT_FIELDS));
        if !exact_keys(case, &["case_id","description","record","expected_disposition"])
            || text(case, "case_id") != *expected_id
            || text(case, "description").len() < 20
            || text(case, "expected_disposition") != *expected_disposition
            || !exact_keys(record, RECORD_FIELDS)
            || !nested_ok
        {
            return Err(format!("DERIVED_CASE_SHAPE_OR_PIN_INVALID:{index}"));
        }
        let actual = classify_record(record, profile);
        if actual != *expected_disposition {
            return Err(format!("{expected_id}:EXPECTED_{expected_disposition},GOT_{actual}"));
        }
    }
    Ok(cases.len())
}

fn validate_profile_schema(schema: &Value, profile: &Value) -> Result<(), String> {
    if text(schema, "title") != "Mycelix Derived Measurement Graph Profile v0.1"
        || schema["type"] != "object" || schema["additionalProperties"] != false
        || !exact_keys(&schema["properties"], &[
            "schema_version","profile_id","base_channel_profile_id","classification","claim_ceiling",
            "real_physical_method_execution_enabled","method_profiles","invariants",
        ])
        || !schema["$defs"]["method"]["additionalProperties"].as_bool().is_some_and(|v| !v)
    {
        return Err("DERIVED_PROFILE_SCHEMA_INVALID".into());
    }
    let required = array_strings(&schema["required"]).ok_or("DERIVED_PROFILE_SCHEMA_REQUIRED_MISSING")?;
    let profile_keys = profile.as_object().ok_or("DERIVED_PROFILE_NOT_OBJECT")?.keys().map(String::as_str).collect::<BTreeSet<_>>();
    if required.iter().copied().collect::<BTreeSet<_>>() != profile_keys {
        return Err("DERIVED_PROFILE_SCHEMA_REQUIRED_DRIFT".into());
    }
    Ok(())
}

fn validate_corpus_schema(schema: &Value, corpus: &Value) -> Result<(), String> {
    if text(schema, "title") != "Mycelix Derived Measurement Graph Corpus v0.1"
        || schema["type"] != "object" || schema["additionalProperties"] != false
        || schema["properties"]["cases"]["minItems"] != EXPECTED_CASES.len()
        || schema["properties"]["cases"]["maxItems"] != EXPECTED_CASES.len()
        || schema["properties"]["cases"]["items"]["properties"]["record"]["$ref"] != "#/$defs/record"
        || schema["$defs"]["record"]["additionalProperties"] != false
        || schema["$defs"]["record"]["properties"]["inputs"]["minItems"] != 0
        || schema["$defs"]["record"]["properties"]["inputs"]["items"]["$ref"] != "#/$defs/input"
        || schema["$defs"]["input"]["additionalProperties"] != false
    {
        return Err("DERIVED_CORPUS_SCHEMA_INVALID".into());
    }
    let required = array_strings(&schema["required"]).ok_or("DERIVED_CORPUS_SCHEMA_REQUIRED_MISSING")?;
    let corpus_keys = corpus.as_object().ok_or("DERIVED_CORPUS_NOT_OBJECT")?.keys().map(String::as_str).collect::<BTreeSet<_>>();
    if required.iter().copied().collect::<BTreeSet<_>>() != corpus_keys {
        return Err("DERIVED_CORPUS_SCHEMA_REQUIRED_DRIFT".into());
    }
    Ok(())
}

pub fn validate_tracked() -> Result<usize, String> {
    let profile_schema = parse_unique(PROFILE_SCHEMA_JSON)?;
    let profile = parse_unique(PROFILE_JSON)?;
    validate_profile_schema(&profile_schema, &profile)?;
    validate_profile(&profile)?;
    let corpus_schema = parse_unique(CORPUS_SCHEMA_JSON)?;
    let corpus = parse_unique(CORPUS_JSON)?;
    validate_corpus_schema(&corpus_schema, &corpus)?;
    validate_corpus(&profile, &corpus)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracked_profile_and_all_36_known_answers_validate() {
        assert_eq!(validate_tracked().unwrap(), 36);
    }
    #[test]
    fn duplicate_json_members_are_rejected() {
        assert!(parse_unique(r#"{"a":1,"a":2}"#).is_err());
    }
    #[test]
    fn source_digests_alone_do_not_make_a_resolvable_graph() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][3]["record"], &profile), "MORE_EVIDENCE_REQUIRED");
    }
    #[test]
    fn revoked_parent_rights_reject_the_derived_claim() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][21]["record"], &profile), "REJECTED_FOR_DECLARED_PURPOSE");
    }
    #[test]
    fn derived_output_cannot_be_relabelled_as_raw() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][16]["record"], &profile), "NOT_COMPARABLE");
    }
    #[test]
    fn physical_methods_remain_blocked_until_input_channels_are_admitted() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][1]["record"], &profile), "MORE_EVIDENCE_REQUIRED");
        assert_eq!(classify_record(&corpus["cases"][2]["record"], &profile), "MORE_EVIDENCE_REQUIRED");
    }
    #[test]
    fn exact_synthetic_mean_is_recomputed_from_parent_values() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][0]["record"], &profile), "SYNTHETIC_GRAPH_VALIDATION_ONLY");
    }
    #[test]
    fn source_firmware_change_invalidates_exact_synthetic_method_input() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][11]["record"], &profile), "EXPIRED_OR_INVALIDATED");
    }
    #[test]
    fn cyclic_parent_graph_is_rejected() {
        let profile = parse_unique(PROFILE_JSON).unwrap();
        let corpus = parse_unique(CORPUS_JSON).unwrap();
        assert_eq!(classify_record(&corpus["cases"][23]["record"], &profile), "NOT_COMPARABLE");
    }
    #[test]
    fn changed_input_values_cannot_hide_behind_a_stale_output() {
        let mut profile = parse_unique(PROFILE_JSON).unwrap();
        let mut corpus = parse_unique(CORPUS_JSON).unwrap();
        corpus["cases"][0]["record"]["inputs"][1]["value"] = Value::from(8.0);
        assert_eq!(classify_record(&corpus["cases"][0]["record"], &profile), "NOT_COMPARABLE");
        profile["invariants"]["no_latest_alias"] = Value::Bool(false);
        assert!(validate_profile(&profile).is_err());
    }
}
