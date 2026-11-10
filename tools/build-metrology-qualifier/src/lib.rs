//! Strict synthetic-only admission oracle for building measurement profiles.
//! This does not qualify real sensors, calibration labs, buildings or physical outcomes.

#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROFILE_JSON: &str = include_str!("../../../docs/metrology/BUILDING_METROLOGY_PROFILE_V0_1.json");
pub const CORPUS_JSON: &str = include_str!("../../../docs/metrology/BUILDING_METROLOGY_CORPUS_V0_1.json");
pub const PROFILE_SCHEMA_JSON: &str = include_str!("../../../docs/metrology/BUILDING_METROLOGY_PROFILE_V0_1.schema.json");

const PROFILE_ID: &str = "MYC-BUILD-METROLOGY-PROFILE-V0.1";
const CORPUS_ID: &str = "MYC-BUILD-METROLOGY-CORPUS-V0.1";

const PROFILE_ROOT_FIELDS: &[&str] = &[
    "schema_version", "profile_id", "classification", "claim_ceiling", "owner",
    "owner_issue", "channels", "evidence_classes", "admission_states", "non_authority",
];
const CHANNEL_FIELDS: &[&str] = &[
    "id", "measurand_id", "quantity_kind", "unit", "raw_or_derived", "data_purpose", "status",
    "local_read_only_required", "calibration_or_reference_check_required",
    "installation_record_required", "uncertainty_budget_required", "response_time_required",
    "synchronized_timebase_required", "method_identity_required", "independent_assay_required",
    "acceptance_tolerances", "physical_thresholds",
];
const NON_AUTHORITY_FIELDS: &[&str] = &[
    "instrument_admission_grants_building_approval",
    "instrument_admission_grants_safety_approval",
    "instrument_admission_grants_device_control",
    "instrument_admission_proves_physical_outcome",
];
const MEASUREMENT_FIELDS: &[&str] = &[
    "profile_id", "instrument_identity_state", "instrument_id", "measurand_id", "quantity_kind",
    "unit", "raw_or_derived", "data_purpose", "firmware_profile_state", "acquisition_mode",
    "local_read_only", "calibration_state", "installation_state", "range_state", "uncertainty_state",
    "response_time_state", "timebase_state", "method_identity_state", "independent_assay_state",
    "rights_state", "reading_state", "serial_identity_reused",
];

/// Frozen v0.1 channel vocabulary. A profile and its corpus cannot redefine these
/// identities/units together and thereby certify a different physical measurand.
const EXPECTED_CHANNELS: &[(&str, &str, &str, &str, &str, &str, bool)] = &[
    ("air_temperature", "habitat.air_temperature", "temperature", "degC", "raw", "thermal_environment", false),
    ("mean_radiant_temperature", "habitat.mean_radiant_temperature", "temperature", "degC", "derived", "thermal_environment", false),
    ("relative_humidity", "habitat.relative_humidity", "ratio", "%RH", "raw", "thermal_environment", false),
    ("air_speed", "habitat.air_speed", "velocity", "m/s", "raw", "thermal_environment", false),
    ("co2_concentration", "habitat.co2_concentration", "mole_fraction", "ppm", "raw", "indoor_air_quality", false),
    ("pm25_mass_concentration", "habitat.pm25_mass_concentration", "mass_concentration", "ug/m3", "raw", "indoor_air_quality", false),
    ("outdoor_air_volume_flow", "habitat.outdoor_air_volume_flow", "volume_flow", "m3/s", "raw", "ventilation", false),
    ("water_volume_flow", "habitat.water_volume_flow", "volume_flow", "m3/s", "raw", "water_service", false),
    ("water_volume", "habitat.water_volume", "volume", "m3", "raw", "water_service", false),
    ("water_ph", "habitat.water_ph", "pH", "1", "raw", "water_quality_assay", true),
    ("water_conductivity", "habitat.water_conductivity", "conductivity", "uS/cm", "raw", "water_quality_assay", true),
    ("electrical_power", "habitat.electrical_power", "power", "W", "raw", "electrical_energy", false),
    ("electrical_energy", "habitat.electrical_energy", "energy", "kWh", "raw", "electrical_energy", false),
    ("useful_thermal_energy", "habitat.useful_thermal_energy", "thermal_energy", "kWh_th", "derived", "thermal_service", false),
];

const EXPECTED_CASES: &[(&str, &str)] = &[
    ("BM-001", "AdmittedForDeclaredMeasurementProfile"),
    ("BM-002", "NotComparable"),
    ("BM-003", "NotComparable"),
    ("BM-004", "ExpiredOrInvalidated"),
    ("BM-005", "MoreEvidenceRequired"),
    ("BM-006", "MoreEvidenceRequired"),
    ("BM-007", "RejectedForDeclaredPurpose"),
    ("BM-008", "ExpiredOrInvalidated"),
    ("BM-009", "MoreEvidenceRequired"),
    ("BM-010", "MoreEvidenceRequired"),
    ("BM-011", "NotComparable"),
    ("BM-012", "NotComparable"),
    ("BM-013", "NotComparable"),
    ("BM-014", "NotComparable"),
    ("BM-015", "NotComparable"),
    ("BM-016", "MoreEvidenceRequired"),
    ("BM-017", "NotComparable"),
    ("BM-018", "ExpiredOrInvalidated"),
    ("BM-019", "NotComparable"),
    ("BM-020", "NotComparable"),
    ("BM-021", "RejectedForDeclaredPurpose"),
    ("BM-022", "MoreEvidenceRequired"),
    ("BM-023", "MoreEvidenceRequired"),
    ("BM-024", "MoreEvidenceRequired"),
    ("BM-025", "RejectedForDeclaredPurpose"),
    ("BM-026", "ExpiredOrInvalidated"),
    ("BM-027", "ExpiredOrInvalidated"),
    ("BM-028", "MoreEvidenceRequired"),
    ("BM-029", "ExpiredOrInvalidated"),
    ("BM-030", "NotComparable"),
    ("BM-031", "NotComparable"),
    ("BM-032", "RejectedForDeclaredPurpose"),
    ("BM-033", "MoreEvidenceRequired"),
    ("BM-034", "MoreEvidenceRequired"),
    ("BM-035", "MoreEvidenceRequired"),
    ("BM-036", "AdmittedForDeclaredMeasurementProfile"),
];

/// A JSON value that rejects duplicate member names at every object depth.
#[derive(Debug)]
struct UniqueJson(Value);

impl<'de> Deserialize<'de> for UniqueJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(UniqueJsonVisitor)
    }
}

struct UniqueJsonVisitor;

impl<'de> Visitor<'de> for UniqueJsonVisitor {
    type Value = UniqueJson;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("a JSON value with no duplicate object member names")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::Number(value.into())))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::Number(value.into())))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
    {
        let number = serde_json::Number::from_f64(value)
            .ok_or_else(|| E::custom("non-finite JSON number"))?;
        Ok(UniqueJson(Value::Number(number)))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::String(value)))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: serde::de::Error,
     {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        UniqueJson::deserialize(deserializer)
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(value) = sequence.next_element::<UniqueJson>()? {
            values.push(value.0);
        }
        Ok(UniqueJson(Value::Array(values)))
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut values = Map::new();
        let mut seen = BTreeSet::new();
        while let Some(key) = map.next_key::<String>()? {
            if !seen.insert(key.clone()) {
                return Err(serde::de::Error::custom(format!(
                    "duplicate JSON object member {key:?}"
                )));
            }
            let value = map.next_value::<UniqueJson>()?;
            values.insert(key, value.0);
        }
        Ok(UniqueJson(Value::Object(values)))
    }
}

pub fn parse_unique_json(source: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(source);
    let value = UniqueJson::deserialize(&mut deserializer)
        .map_err(|error| format!("STRICT_JSON_REJECTED: {error}"))?;
    deserializer.end().map_err(|error| format!("STRICT_JSON_TRAILING_DATA: {error}"))?;
    Ok(value.0)
}

fn exact_keys(value: &Value, expected: &[&str]) -> bool {
    let Some(object) = value.as_object() else { return false; };
    object.len() == expected.len() && expected.iter().all(|key| object.contains_key(*key))
}

fn exact_string_array(value: &Value, expected: &[&str]) -> bool {
    value.as_array().is_some_and(|items| {
        items.len() == expected.len()
            && items.iter().zip(expected.iter()).all(|(item, expected)| {
                item.as_str() == Some(*expected)
            })
    })
}

fn schema_string_property(schema: &Value, property: &str) -> bool {
    let item = &schema["properties"][property];
    item["type"] == "string" && item["minLength"] == 1
}

fn validate_profile_schema(schema: &Value) -> Result<(), String> {
    if schema["$schema"] != "https://json-schema.org/draft/2020-12/schema"
        || schema["title"] != "Mycelix Building Metrology Profile v0.1"
        || schema["type"] != "object"
        || schema["additionalProperties"] != false
        || !exact_string_array(&schema["required"], PROFILE_ROOT_FIELDS)
        || !exact_keys(&schema["properties"], PROFILE_ROOT_FIELDS)
    {
        return Err("PROFILE_SCHEMA_ROOT_CONTRACT_INVALID".to_owned());
    }

    if schema["properties"]["schema_version"]["const"] != "mycelix.building-metrology-profile/0.1"
        || schema["properties"]["profile_id"]["const"] != PROFILE_ID
        || schema["properties"]["classification"]["const"] != "synthetic_measurement_requirements_only"
        || schema["properties"]["claim_ceiling"]["const"] != "instrument_profile_schema_and_physical_thresholds_not_set"
        || schema["properties"]["owner"]["const"] != "Mycelix"
        || schema["properties"]["owner_issue"]["const"]
            != "https://github.com/Luminous-Dynamics/mycelix/issues/6025"
    {
        return Err("PROFILE_SCHEMA_IDENTITY_CONSTRAINT_DRIFT".to_owned());
    }

    if schema["properties"]["evidence_classes"]["type"] != "array"
        || !exact_string_array(
            &schema["properties"]["evidence_classes"]["items"]["enum"],
            &[
                "provider_assertion",
                "documented_capability",
                "calibration_record",
                "independent_check",
                "measured_observation",
                "assay_result",
            ],
        )
        || schema["properties"]["evidence_classes"]["uniqueItems"] != true
        || schema["properties"]["admission_states"]["type"] != "array"
        || !exact_string_array(
            &schema["properties"]["admission_states"]["items"]["enum"],
            &[
                "AdmittedForDeclaredMeasurementProfile",
                "AdmittedWithDeclaredLimitations",
                "MoreEvidenceRequired",
                "NotComparable",
                "RejectedForDeclaredPurpose",
                "ExpiredOrInvalidated",
            ],
        )
        || schema["properties"]["admission_states"]["uniqueItems"] != true
    {
        return Err("PROFILE_SCHEMA_ENUM_CONTRACT_INVALID".to_owned());
    }

    let channel_array = &schema["properties"]["channels"];
    if channel_array["type"] != "array"
        || channel_array["minItems"] != EXPECTED_CHANNELS.len()
        || channel_array["maxItems"] != EXPECTED_CHANNELS.len()
        || channel_array["items"]["$ref"] != "#/$defs/channel"
    {
        return Err("PROFILE_SCHEMA_CHANNEL_ARRAY_INVALID".to_owned());
    }

    let channel_schema = &schema["$defs"]["channel"];
    if channel_schema["type"] != "object"
        || channel_schema["additionalProperties"] != false
        || !exact_string_array(&channel_schema["required"], CHANNEL_FIELDS)
        || !exact_keys(&channel_schema["properties"], CHANNEL_FIELDS)
    {
        return Err("PROFILE_SCHEMA_CHANNEL_CONTRACT_INVALID".to_owned());
    }

    for field in ["id", "measurand_id", "quantity_kind", "unit", "data_purpose"] {
        if !schema_string_property(channel_schema, field) {
            return Err(format!("PROFILE_SCHEMA_CHANNEL_STRING_INVALID:{field}"));
        }
    }
    if channel_schema["properties"]["raw_or_derived"]["enum"]
        != serde_json::json!(["raw", "derived"])
        || channel_schema["properties"]["status"]["const"] != "REQUIREMENTS_ONLY"
        || channel_schema["properties"]["local_read_only_required"]["const"] != true
        || channel_schema["properties"]["calibration_or_reference_check_required"]["const"] != true
        || channel_schema["properties"]["installation_record_required"]["const"] != true
        || channel_schema["properties"]["uncertainty_budget_required"]["const"] != true
        || channel_schema["properties"]["response_time_required"]["const"] != true
        || channel_schema["properties"]["synchronized_timebase_required"]["const"] != true
        || channel_schema["properties"]["method_identity_required"]["const"] != true
        || channel_schema["properties"]["independent_assay_required"]["type"] != "boolean"
        || channel_schema["properties"]["acceptance_tolerances"]["const"] != "NOT_SET_BY_THIS_PROFILE"
        || channel_schema["properties"]["physical_thresholds"]["const"] != "NOT_SET_BY_THIS_PROFILE"
    {
        return Err("PROFILE_SCHEMA_CHANNEL_CONSTRAINT_DRIFT".to_owned());
    }

    let authority_schema = &schema["properties"]["non_authority"];
    if authority_schema["type"] != "object"
        || authority_schema["additionalProperties"] != false
        || !exact_string_array(&authority_schema["required"], NON_AUTHORITY_FIELDS)
        || !exact_keys(&authority_schema["properties"], NON_AUTHORITY_FIELDS)
    {
        return Err("PROFILE_SCHEMA_AUTHORITY_CONTRACT_INVALID".to_owned());
    }
    for field in NON_AUTHORITY_FIELDS {
        if authority_schema["properties"][*field]["const"] != false {
            return Err(format!("PROFILE_SCHEMA_AUTHORITY_CONSTRAINT_DRIFT:{field}"));
        }
    }
    Ok(())
}

fn channel_expected<'a>(profile: &'a Value, id: &str) -> Option<&'a Value> {
    profile.get("channels")?.as_array()?.iter().find(|channel| {
        channel.get("id").and_then(Value::as_str) == Some(id)
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdmissionState {
    AdmittedForDeclaredMeasurementProfile,
    MoreEvidenceRequired,
    NotComparable,
    RejectedForDeclaredPurpose,
    ExpiredOrInvalidated,
}

impl AdmissionState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::AdmittedForDeclaredMeasurementProfile => "AdmittedForDeclaredMeasurementProfile",
            Self::MoreEvidenceRequired => "MoreEvidenceRequired",
            Self::NotComparable => "NotComparable",
            Self::RejectedForDeclaredPurpose => "RejectedForDeclaredPurpose",
            Self::ExpiredOrInvalidated => "ExpiredOrInvalidated",
        }
    }
}

fn text<'a>(value: &'a Value, key: &str) -> &'a str {
    value.get(key).and_then(Value::as_str).unwrap_or("")
}

fn bool_is(value: &Value, key: &str, expected: bool) -> bool {
    value.get(key).and_then(Value::as_bool) == Some(expected)
}

/// Classify one synthetic instrument evidence record against a profile channel.
/// The result is admission to a declared measurement profile only, not a physical pass.
pub fn classify_measurement(profile: &Value, measurement: &Value) -> AdmissionState {
    if validate_profile(profile).is_err()
        || !exact_keys(measurement, MEASUREMENT_FIELDS)
        || !measurement["local_read_only"].is_boolean()
        || !measurement["serial_identity_reused"].is_boolean()
        || MEASUREMENT_FIELDS.iter().filter(|field| **field != "local_read_only" && **field != "serial_identity_reused")
            .any(|field| measurement[*field].as_str().is_none())
    {
        return AdmissionState::MoreEvidenceRequired;
    }
    let id = text(measurement, "measurand_id");
    let Some(channel_id) = id.strip_prefix("habitat.") else {
        return AdmissionState::NotComparable;
    };
    let Some(channel) = channel_expected(profile, channel_id) else {
        return AdmissionState::NotComparable;
    };

    if text(measurement, "profile_id") != PROFILE_ID
        || text(measurement, "measurand_id") != text(channel, "measurand_id")
        || text(measurement, "quantity_kind") != text(channel, "quantity_kind")
        || text(measurement, "unit") != text(channel, "unit")
        || text(measurement, "raw_or_derived") != text(channel, "raw_or_derived")
        || text(measurement, "data_purpose") != text(channel, "data_purpose")
    {
        return AdmissionState::NotComparable;
    }

    if text(measurement, "firmware_profile_state") != "current"
        || matches!(text(measurement, "reading_state"), "stale" | "missing_as_zero" | "out_of_range" | "device_fault" | "conflicting")
    {
        return AdmissionState::ExpiredOrInvalidated;
    }

    if bool_is(channel, "local_read_only_required", true)
        && (text(measurement, "acquisition_mode") != "local"
            || !bool_is(measurement, "local_read_only", true))
    {
        return AdmissionState::RejectedForDeclaredPurpose;
    }

    if bool_is(measurement, "serial_identity_reused", true) {
        return AdmissionState::RejectedForDeclaredPurpose;
    }

    if text(measurement, "rights_state") == "revoked" {
        return AdmissionState::RejectedForDeclaredPurpose;
    }

    if text(measurement, "instrument_identity_state") != "established"
        || text(measurement, "instrument_id").is_empty()
        || text(measurement, "reading_state") != "current"
        || text(measurement, "rights_state") != "permitted_for_declared_purpose"
        || text(measurement, "calibration_state") != "current"
        || text(measurement, "installation_state") != "verified"
        || text(measurement, "range_state") != "documented"
        || text(measurement, "uncertainty_state") != "documented"
        || text(measurement, "response_time_state") != "documented"
        || text(measurement, "timebase_state") != "synchronized"
        || text(measurement, "method_identity_state") != "identified"
        || (bool_is(channel, "independent_assay_required", false)
            && text(measurement, "independent_assay_state") != "identified")
    {
        return AdmissionState::MoreEvidenceRequired;
    }

    AdmissionState::AdmittedForDeclaredMeasurementProfile
}

fn validate_profile(profile: &Value) -> Result<(), String> {
    if !exact_keys(profile, PROFILE_ROOT_FIELDS) {
        return Err("PROFILE_ROOT_FIELDS_INVALID".to_owned());
    }
    if text(profile, "schema_version") != "mycelix.building-metrology-profile/0.1"
        || text(profile, "profile_id") != PROFILE_ID
        || text(profile, "classification") != "synthetic_measurement_requirements_only"
        || text(profile, "claim_ceiling") != "instrument_profile_schema_and_synthetic_admission_logic_only"
        || text(profile, "owner") != "Mycelix"
        || text(profile, "owner_issue") != "https://github.com/Luminous-Dynamics/mycelix/issues/6025"
    {
        return Err("PROFILE_IDENTITY_OR_CLAIM_CEILING_INVALID".to_owned());
    }

    let channels = profile["channels"].as_array().ok_or("CHANNELS_MISSING")?;
    if channels.len() != EXPECTED_CHANNELS.len() {
        return Err("CHANNEL_COUNT_INVALID".to_owned());
    }
    for (channel, expected) in channels.iter().zip(EXPECTED_CHANNELS.iter()) {
        if !exact_keys(channel, CHANNEL_FIELDS) {
            return Err("CHANNEL_FIELDS_INVALID".to_owned());
        }
        let (id, measurand, kind, unit, raw_or_derived, purpose, requires_assay) = *expected;
        if text(channel, "id") != id
            || text(channel, "measurand_id") != measurand
            || text(channel, "quantity_kind") != kind
            || text(channel, "unit") != unit
            || text(channel, "raw_or_derived") != raw_or_derived
            || text(channel, "data_purpose") != purpose
            || text(channel, "status") != "REQUIREMENTS_ONLY"
            || !bool_is(channel, "local_read_only_required", true)
            || !bool_is(channel, "calibration_or_reference_check_required", true)
            || !bool_is(channel, "installation_record_required", true)
            || !bool_is(channel, "uncertainty_budget_required", true)
            || !bool_is(channel, "response_time_required", true)
            || !bool_is(channel, "synchronized_timebase_required", true)
            || !bool_is(channel, "method_identity_required", true)
            || !bool_is(channel, "independent_assay_required", requires_assay)
            || text(channel, "acceptance_tolerances") != "NOT_SET_BY_THIS_PROFILE"
            || text(channel, "physical_thresholds") != "NOT_SET_BY_THIS_PROFILE"
        {
            return Err(format!("CHANNEL_DEFINITION_DRIFT:{id}"));
        }
    }

    if !exact_string_array(
        &profile["evidence_classes"],
        &[
            "provider_assertion",
            "documented_capability",
            "calibration_record",
            "independent_check",
            "measured_observation",
            "assay_result",
        ],
    ) {
        return Err("EVIDENCE_CLASS_SET_INVALID".to_owned());
    }
    if !exact_string_array(
        &profile["admission_states"],
        &[
            "AdmittedForDeclaredMeasurementProfile",
            "AdmittedWithDeclaredLimitations",
            "MoreEvidenceRequired",
            "NotComparable",
            "RejectedForDeclaredPurpose",
            "ExpiredOrInvalidated",
        ],
    ) {
        return Err("ADMISSION_STATE_SET_INVALID".to_owned());
    }

    let non_authority = profile.get("non_authority").ok_or("NON_AUTHORITY_MISSING")?;
    if !exact_keys(non_authority, NON_AUTHORITY_FIELDS) {
        return Err("NON_AUTHORITY_FIELDS_INVALID".to_owned());
    }
    for key in NON_AUTHORITY_FIELDS {
        if !bool_is(non_authority, key, false) {
            return Err("NON_AUTHORITY_BOUNDARY_CHANGED".to_owned());
        }
    }
    Ok(())
}

pub fn validate_corpus(profile: &Value, corpus: &Value) -> Result<usize, String> {
    if !exact_keys(corpus, &[
        "schema_version", "corpus_id", "classification", "profile_id", "claim_ceiling", "cases"
    ]) {
        return Err("CORPUS_ROOT_FIELDS_INVALID".to_owned());
    }
    if text(corpus, "schema_version") != "mycelix.building-metrology-corpus/0.1"
        || text(corpus, "corpus_id") != CORPUS_ID
        || text(corpus, "classification") != "synthetic_known_answer_admission_fixtures"
        || text(corpus, "profile_id") != PROFILE_ID
        || text(corpus, "claim_ceiling") != "synthetic_instrument_admission_logic_only"
    {
        return Err("CORPUS_IDENTITY_OR_CLAIM_CEILING_INVALID".to_owned());
    }

    let cases = corpus["cases"].as_array().ok_or("CASES_MISSING")?;
    if cases.len() != EXPECTED_CASES.len() {
        return Err("CASE_COUNT_INVALID".to_owned());
    }

    for (case, (expected_id, expected_disposition)) in
        cases.iter().zip(EXPECTED_CASES.iter())
    {
        if !exact_keys(case, &["case_id", "description", "measurement", "expected_disposition"]) {
            return Err("CASE_FIELDS_INVALID".to_owned());
        }
        let id = text(case, "case_id");
        if id != *expected_id
            || text(case, "description").len() < 20
            || text(case, "expected_disposition") != *expected_disposition
        {
            return Err(format!("CASE_KNOWN_ANSWER_OR_ID_DRIFT:{expected_id}"));
        }

        let measurement = &case["measurement"];
        if !exact_keys(measurement, MEASUREMENT_FIELDS) {
            return Err(format!("{id}:MEASUREMENT_FIELDS_INVALID"));
        }
        let actual = classify_measurement(profile, measurement).as_str();
        if actual != *expected_disposition {
            return Err(format!("{id}:EXPECTED_{expected_disposition},GOT_{actual}"));
        }
    }
    Ok(cases.len())
}

pub fn validate_tracked() -> Result<usize, String> {
    let schema = parse_unique_json(PROFILE_SCHEMA_JSON)?;
    validate_profile_schema(&schema)?;
    let profile = parse_unique_json(PROFILE_JSON)?;
    let corpus = parse_unique_json(CORPUS_JSON)?;
    validate_profile(&profile)?;
    validate_corpus(&profile, &corpus)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracked_profile_and_36_known_answers_validate() {
        assert_eq!(validate_tracked().expect("tracked fixtures validate"), 36);
    }

    #[test]
    fn duplicate_root_json_key_is_rejected() {
        let changed = PROFILE_JSON.replacen(
            "\"profile_id\":",
            "\"profile_id\":\"MYC-BUILD-METROLOGY-PROFILE-V0.1\",\"profile_id\":",
            1,
        );
        assert_ne!(changed, PROFILE_JSON);
        assert!(parse_unique_json(&changed).is_err());
    }

    #[test]
    fn duplicate_nested_json_key_is_rejected() {
        let changed = PROFILE_JSON.replacen(
            "\"unit\": \"degC\"",
            "\"unit\": \"degC\", \"unit\": \"K\"",
            1,
        );
        assert_ne!(changed, PROFILE_JSON);
        assert!(parse_unique_json(&changed).is_err());
    }

    #[test]
    fn unknown_profile_root_field_is_rejected() {
        let changed = PROFILE_JSON.replacen(
            "\"profile_id\":",
            "\"unreviewed\":true,\"profile_id\":",
            1,
        );
        assert_ne!(changed, PROFILE_JSON);
        let profile = parse_unique_json(&changed).unwrap();
        assert!(validate_profile(&profile).is_err());
    }

    #[test]
    fn unknown_channel_field_is_rejected() {
        let changed = PROFILE_JSON.replacen(
            "\"measurand_id\":",
            "\"unreviewed_channel_field\":true,\"measurand_id\":",
            1,
        );
        assert_ne!(changed, PROFILE_JSON);
        let profile = parse_unique_json(&changed).unwrap();
        assert!(validate_profile(&profile).is_err());
    }

    #[test]
    fn profile_schema_matches_the_closed_rust_contract() {
        let schema = parse_unique_json(PROFILE_SCHEMA_JSON).expect("schema JSON parses without duplicate keys");
        validate_profile_schema(&schema).expect("schema matches exact Rust contract");
    }

    #[test]
    fn changing_a_channel_unit_is_rejected_even_if_fixture_is_edited_too() {
        let changed = PROFILE_JSON.replace(
            "\"unit\": \"degC\"",
            "\"unit\": \"K\"",
        );
        assert_ne!(changed, PROFILE_JSON);
        let profile = parse_unique_json(&changed).unwrap();
        assert!(validate_profile(&profile).is_err());
    }

    #[test]
    fn changing_a_channel_measurand_identity_is_rejected() {
        let changed = PROFILE_JSON.replace(
            "\"measurand_id\": \"habitat.air_temperature\"",
            "\"measurand_id\": \"habitat.generic_temperature\"",
        );
        assert_ne!(changed, PROFILE_JSON);
        let profile = parse_unique_json(&changed).unwrap();
        assert!(validate_profile(&profile).is_err());
    }

    #[test]
    fn weakening_a_known_answer_disposition_is_rejected() {
        let mut corpus = parse_unique_json(CORPUS_JSON).unwrap();
        corpus["cases"][6]["expected_disposition"] =
            serde_json::Value::String("AdmittedForDeclaredMeasurementProfile".to_owned());
        let profile = parse_unique_json(PROFILE_JSON).unwrap();
        assert!(validate_corpus(&profile, &corpus).is_err());
    }

    #[test]
    fn measurement_boolean_fields_must_be_booleans() {
        let mut measurement = parse_unique_json(CORPUS_JSON).unwrap()["cases"][0]["measurement"].clone();
        measurement["serial_identity_reused"] = serde_json::Value::String("false".to_owned());
        let profile = parse_unique_json(PROFILE_JSON).unwrap();
        assert_eq!(
            classify_measurement(&profile, &measurement),
            AdmissionState::MoreEvidenceRequired
        );
    }

    #[test]
    fn a_profile_cannot_promote_physical_tolerances() {
        let changed = PROFILE_JSON.replace(
            "\"acceptance_tolerances\": \"NOT_SET_BY_THIS_PROFILE\"",
            "\"acceptance_tolerances\": \"±0.1 degC\"",
        );
        assert_ne!(changed, PROFILE_JSON);
        let profile = parse_unique_json(&changed).unwrap();
        assert!(validate_profile(&profile).is_err());
    }
}
