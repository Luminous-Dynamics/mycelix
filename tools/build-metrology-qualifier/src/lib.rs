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

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Bool(value)))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Number(value.into())))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E> {
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

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::String(value.to_owned())))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::String(value)))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E> {
        Ok(UniqueJson(Value::Null))
    }

    fn visit_none<E>(self) -> Result<Self::Value, E> {
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
    if !exact_keys(profile, &[
        "schema_version","profile_id","classification","claim_ceiling","owner","owner_issue",
        "channels","evidence_classes","admission_states","non_authority"
    ]) {
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
    if channels.len() != 14 {
        return Err("CHANNEL_COUNT_INVALID".to_owned());
    }
    let mut ids = BTreeSet::new();
    for channel in channels {
        if !exact_keys(channel, &[
            "id","measurand_id","quantity_kind","unit","raw_or_derived","data_purpose","status",
            "local_read_only_required","calibration_or_reference_check_required",
            "installation_record_required","uncertainty_budget_required","response_time_required",
            "synchronized_timebase_required","method_identity_required","independent_assay_required",
            "acceptance_tolerances","physical_thresholds"
        ]) {
            return Err("CHANNEL_FIELDS_INVALID".to_owned());
        }
        let id = text(channel, "id");
        if id.is_empty() || !ids.insert(id.to_owned()) {
            return Err("CHANNEL_ID_DUPLICATE_OR_MISSING".to_owned());
        }
        if text(channel, "status") != "REQUIREMENTS_ONLY"
            || !bool_is(channel, "local_read_only_required", true)
            || text(channel, "acceptance_tolerances") != "NOT_SET_BY_THIS_PROFILE"
            || text(channel, "physical_thresholds") != "NOT_SET_BY_THIS_PROFILE"
        {
            return Err("CHANNEL_REQUIREMENT_OR_CLAIM_PROMOTED".to_owned());
        }
    }
    let non_authority = profile.get("non_authority").ok_or("NON_AUTHORITY_MISSING")?;
    if !exact_keys(non_authority, &[
        "instrument_admission_grants_building_approval","instrument_admission_grants_safety_approval",
        "instrument_admission_grants_device_control","instrument_admission_proves_physical_outcome"
    ]) {
        return Err("NON_AUTHORITY_FIELDS_INVALID".to_owned());
    }
    for key in [
        "instrument_admission_grants_building_approval",
        "instrument_admission_grants_safety_approval",
        "instrument_admission_grants_device_control",
        "instrument_admission_proves_physical_outcome"
    ] {
        if !bool_is(non_authority, key, false) {
            return Err("NON_AUTHORITY_BOUNDARY_CHANGED".to_owned());
        }
    }
    Ok(())
}

pub fn validate_corpus(profile: &Value, corpus: &Value) -> Result<usize, String> {
    if !exact_keys(corpus, &[
        "schema_version","corpus_id","classification","profile_id","claim_ceiling","cases"
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
    if cases.len() != 36 {
        return Err("CASE_COUNT_INVALID".to_owned());
    }
    let mut case_ids = BTreeSet::new();
    for case in cases {
        if !exact_keys(case, &["case_id","description","measurement","expected_disposition"]) {
            return Err("CASE_FIELDS_INVALID".to_owned());
        }
        let id = text(case, "case_id");
        if id.is_empty() || !case_ids.insert(id.to_owned()) {
            return Err("CASE_ID_DUPLICATE_OR_MISSING".to_owned());
        }
        let measurement = &case["measurement"];
        if !exact_keys(measurement, &[
            "profile_id","instrument_identity_state","instrument_id","measurand_id","quantity_kind",
            "unit","raw_or_derived","data_purpose","firmware_profile_state","acquisition_mode",
            "local_read_only","calibration_state","installation_state","range_state","uncertainty_state",
            "response_time_state","timebase_state","method_identity_state","independent_assay_state",
            "rights_state","reading_state","serial_identity_reused"
        ]) {
            return Err(format!("{id}:MEASUREMENT_FIELDS_INVALID"));
        }
        let actual = classify_measurement(profile, measurement).as_str();
        if actual != text(case, "expected_disposition") {
            return Err(format!("{id}:EXPECTED_{},GOT_{actual}", text(case, "expected_disposition")));
        }
    }
    Ok(cases.len())
}

pub fn validate_tracked() -> Result<usize, String> {
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
