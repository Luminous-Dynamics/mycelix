//! Synthetic-only condition-applicability oracle for building metrology.
//! It resolves sample conditions against versioned profiles; it does not qualify
//! physical instruments, validate corrections, or authorize building decisions.

#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use std::fmt;

pub const REGISTRY_JSON: &str = include_str!(
    "../../../docs/metrology/BUILDING_MEASUREMENT_CONDITION_PROFILES_V0_1.json"
);
pub const REGISTRY_SCHEMA_JSON: &str = include_str!(
    "../../../docs/metrology/BUILDING_MEASUREMENT_CONDITION_PROFILES_V0_1.schema.json"
);
pub const CORPUS_JSON: &str = include_str!(
    "../../../docs/metrology/BUILDING_MEASUREMENT_CONDITION_CORPUS_V0_1.json"
);
pub const CORPUS_SCHEMA_JSON: &str = include_str!(
    "../../../docs/metrology/BUILDING_MEASUREMENT_CONDITION_CORPUS_V0_1.schema.json"
);

const REGISTRY_ID: &str = "MYC-BUILD-MEASUREMENT-CONDITIONS-V0.1";
const REGISTRY_BLOB_SHA1: &str = "69b7fcbce60fde5b1aab9ee1088efe826d6b33ef";
const CORPUS_ID: &str = "MYC-BUILD-MEASUREMENT-CONDITIONS-CORPUS-V0.1";
const PROFILE_SCHEMA_VERSION: &str = "mycelix.building-measurement-condition-registry/0.1";
const CORPUS_SCHEMA_VERSION: &str = "mycelix.building.measurement-condition-corpus/0.1";
const CONDITION_RECORD_VERSION: &str = "mycelix.building.measurement-condition-applicability/0.1";
const PASSPORT_PROFILE_ID: &str = "MYC-BUILD-METROLOGY-PROFILE-V0.1";
const TIMEBASE_CONTRACT_ID: &str = "MYC-BUILD-STREAM-TIMEBASE-001";
const EXPECTED_CASE_COUNT: usize = 43;

const REGISTRY_FIELDS: &[&str] = &[
    "schema_version",
    "registry_id",
    "classification",
    "claim_ceiling",
    "profiles",
];
const PROFILE_FIELDS: &[&str] = &[
    "profile_id",
    "conditions_id",
    "revision",
    "channel_profile_id",
    "channel_id",
    "measurand_id",
    "firmware_revision",
    "instrument_configuration_digest",
    "profile_evidence_state",
    "valid_from_date",
    "valid_to_date",
    "variables",
    "spatial_requirements",
    "temporal_requirements",
    "correction_policy",
];
const VARIABLE_FIELDS: &[&str] = &[
    "variable_id",
    "quantity_kind",
    "unit",
    "value_kind",
    "minimum",
    "maximum",
    "allowed_values",
    "maximum_uncertainty",
    "required",
];
const SPATIAL_FIELDS: &[&str] = &[
    "geometry_revision",
    "location_context_id",
    "required_relation",
    "max_transfer_distance_m",
    "transfer_method_id",
    "transfer_validation_receipt_id",
];
const TEMPORAL_FIELDS: &[&str] = &[
    "timebase_contract_id",
    "require_full_sample_window",
    "minimum_coverage_fraction",
    "clock_domain_id",
    "boot_epoch_id",
    "tick_unit",
];
const CORRECTION_FIELDS: &[&str] = &[
    "state",
    "method_id",
    "method_revision",
    "method_digest",
    "validation_receipt_id",
    "must_apply_once",
];
const CORPUS_FIELDS: &[&str] = &[
    "schema_version",
    "corpus_id",
    "registry_id",
    "registry_blob_sha1",
    "classification",
    "claim_ceiling",
    "cases",
];
const CASE_FIELDS: &[&str] = &[
    "case_id",
    "description",
    "record",
    "expected_disposition",
];
const RECORD_FIELDS: &[&str] = &[
    "schema_version",
    "passport_binding",
    "registry_binding",
    "condition_observations",
    "correction_application",
];
const PASSPORT_BINDING_FIELDS: &[&str] = &[
    "passport_profile_id",
    "instrument_id",
    "channel_id",
    "measurand_id",
    "firmware_revision",
    "instrument_configuration_digest",
    "capability_conditions_id",
    "calibration_conditions_id",
    "accuracy_conditions_id",
    "drift_conditions_id",
    "sample_observation_id",
    "sample_digest",
    "observed_at_date",
    "sample_rights_state",
    "sample_geometry_revision",
    "sample_location_context_id",
    "sample_time_support",
];
const REGISTRY_BINDING_FIELDS: &[&str] = &[
    "registry_id",
    "registry_blob_sha1",
    "profile_id",
    "profile_revision",
    "conditions_id",
    "profile_evidence_state",
];
const OBSERVATION_FIELDS: &[&str] = &[
    "variable_id",
    "value_kind",
    "numeric_value",
    "category_value",
    "unit",
    "observation_id",
    "source_digest",
    "source_evidence_state",
    "source_instrument_id",
    "source_channel_id",
    "calibration_record_id",
    "calibration_state",
    "installation_record_id",
    "installation_state",
    "geometry_revision",
    "location_context_id",
    "spatial_relation",
    "transfer_method_id",
    "transfer_validation_receipt_id",
    "distance_m",
    "uncertainty_value",
    "uncertainty_unit",
    "evidence_class",
    "rights_state",
    "time_support",
];
const TIME_FIELDS: &[&str] = &[
    "timebase_contract_id",
    "stream_id",
    "clock_domain_id",
    "boot_epoch_id",
    "start_tick",
    "end_tick",
    "tick_unit",
    "coverage_fraction",
    "timebase_state",
    "timebase_receipt_id",
];
const CORRECTION_APPLICATION_FIELDS: &[&str] = &[
    "state",
    "method_id",
    "method_revision",
    "method_digest",
    "validation_receipt_id",
    "application_count",
    "source_conditions_digest",
    "output_sample_digest",
];

const EXPECTED_PROFILES: &[(&str, &str, &str, &str, &str, &str, usize)] = &[
    ("COND-AIR-TEMP-001", "SYNTH-ENV-01", "air_temperature", "habitat.air_temperature", "FW-SYNTH-1", "fixture-config-v1", 2),
    ("COND-OPTICAL-PM-001", "SYNTH-PM-ENV-01", "pm25_mass_concentration", "habitat.pm25_mass_concentration", "FW-SYNTH-1", "fixture-config-v1", 3),
    ("COND-GAS-001", "SYNTH-GAS-ENV-01", "co2_concentration", "habitat.co2_concentration", "FW-SYNTH-1", "fixture-config-v1", 3),
    ("COND-WATER-FLOW-001", "SYNTH-FLOW-ENV-01", "water_volume_flow", "habitat.water_volume_flow", "FW-SYNTH-1", "fixture-config-v1", 5),
    ("COND-ELECTRICAL-001", "SYNTH-ELEC-ENV-01", "electrical_power", "habitat.electrical_power", "FW-SYNTH-1", "fixture-config-v1", 4),
    ("COND-ACOUSTIC-001", "SYNTH-ACOUSTIC-ENV-01", "air_speed", "habitat.air_speed", "FW-SYNTH-1", "fixture-config-v1", 3),
];
const EXPECTED_CASES: &[(&str, &str)] = &[
    ("MC-001", "SYNTHETIC_CONDITION_CONTRACT_VALIDATION_ONLY"),
    ("MC-002", "NOT_COMPARABLE"),
    ("MC-003", "NOT_COMPARABLE"),
    ("MC-004", "MORE_EVIDENCE_REQUIRED"),
    ("MC-005", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("MC-006", "NOT_COMPARABLE"),
    ("MC-007", "MORE_EVIDENCE_REQUIRED"),
    ("MC-008", "MORE_EVIDENCE_REQUIRED"),
    ("MC-009", "NOT_COMPARABLE"),
    ("MC-010", "MORE_EVIDENCE_REQUIRED"),
    ("MC-011", "MORE_EVIDENCE_REQUIRED"),
    ("MC-012", "OUTSIDE_VALIDATED_ENVELOPE"),
    ("MC-013", "OUTSIDE_VALIDATED_ENVELOPE"),
    ("MC-014", "MORE_EVIDENCE_REQUIRED"),
    ("MC-015", "NOT_COMPARABLE"),
    ("MC-016", "MORE_EVIDENCE_REQUIRED"),
    ("MC-017", "NOT_COMPARABLE"),
    ("MC-018", "NOT_COMPARABLE"),
    ("MC-019", "NOT_COMPARABLE"),
    ("MC-020", "MORE_EVIDENCE_REQUIRED"),
    ("MC-021", "MORE_EVIDENCE_REQUIRED"),
    ("MC-022", "EXPIRED_OR_INVALIDATED"),
    ("MC-023", "EXPIRED_OR_INVALIDATED"),
    ("MC-024", "NOT_COMPARABLE"),
    ("MC-025", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("MC-026", "MORE_EVIDENCE_REQUIRED"),
    ("MC-027", "NOT_COMPARABLE"),
    ("MC-028", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("MC-029", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("MC-030", "MORE_EVIDENCE_REQUIRED"),
    ("MC-031", "OUTSIDE_VALIDATED_ENVELOPE"),
    ("MC-032", "NOT_COMPARABLE"),
    ("MC-033", "NOT_COMPARABLE"),
    ("MC-034", "NOT_COMPARABLE"),
    ("MC-035", "OUTSIDE_VALIDATED_ENVELOPE"),
    ("MC-036", "MORE_EVIDENCE_REQUIRED"),
    ("MC-037", "MORE_EVIDENCE_REQUIRED"),
    ("MC-038", "MORE_EVIDENCE_REQUIRED"),
    ("MC-039", "EXPIRED_OR_INVALIDATED"),
    ("MC-040", "EXPIRED_OR_INVALIDATED"),
    ("MC-041", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("MC-042", "MORE_EVIDENCE_REQUIRED"),
    ("MC-043", "MORE_EVIDENCE_REQUIRED"),
];

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
        formatter.write_str("a JSON value without duplicate object members")
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

fn parse_unique_json(source: &str) -> Result<Value, String> {
    let mut deserializer = serde_json::Deserializer::from_str(source);
    let value = UniqueJson::deserialize(&mut deserializer)
        .map_err(|error| format!("STRICT_JSON_REJECTED:{error}"))?;
    deserializer
        .end()
        .map_err(|error| format!("JSON_TRAILING_DATA:{error}"))?;
    Ok(value.0)
}

fn exact_keys(value: &Value, fields: &[&str]) -> bool {
    let Some(object) = value.as_object() else {
        return false;
    };
    object.len() == fields.len() && fields.iter().all(|field| object.contains_key(*field))
}

fn string_value<'a>(value: &'a Value, field: &str) -> &'a str {
    value.get(field).and_then(Value::as_str).unwrap_or("")
}

fn number_value(value: &Value, field: &str) -> Option<f64> {
    value.get(field).and_then(Value::as_f64).filter(|value| value.is_finite())
}

fn required_text(value: &Value, field: &str) -> bool {
    !string_value(value, field).trim().is_empty()
}

fn json_type_matches(value: &Value, type_name: &str) -> bool {
    match type_name {
        "object" => value.is_object(),
        "array" => value.is_array(),
        "string" => value.is_string(),
        "number" => value.as_f64().is_some_and(f64::is_finite),
        "integer" => value.as_i64().is_some()
            || value.as_u64().is_some()
            || value.as_f64().is_some_and(|number| number.is_finite() && number.fract() == 0.0),
        "boolean" => value.is_boolean(),
        "null" => value.is_null(),
        _ => false,
    }
}

fn resolve_reference<'a>(root: &'a Value, reference: &str) -> Option<&'a Value> {
    let pointer = reference.strip_prefix("#/")?;
    let mut current = root;
    for component in pointer.split('/') {
        let key = component.replace("~1", "/").replace("~0", "~");
        current = current.get(key)?;
    }
    Some(current)
}

fn date_is_valid(value: &str) -> bool {
    let Some((year, remainder)) = value.split_once('-') else {
        return false;
    };
    let Some((month, day)) = remainder.split_once('-') else {
        return false;
    };
    if year.len() != 4 || month.len() != 2 || day.len() != 2 {
        return false;
    }
    let (Ok(year), Ok(month), Ok(day)) = (
        year.parse::<u32>(),
        month.parse::<u32>(),
        day.parse::<u32>(),
    ) else {
        return false;
    };
    if !(1..=12).contains(&month) {
        return false;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    (1..=days).contains(&day)
}

fn schema_pattern_matches(pattern: &str, value: &str) -> bool {
    if pattern == "^MC-[0-9]{3}$" {
        let bytes = value.as_bytes();
        return bytes.len() == 6
            && bytes.starts_with(b"MC-")
            && bytes[3..].iter().all(u8::is_ascii_digit);
    }
    if pattern == "^[0-9a-f]{40}$" {
        let bytes = value.as_bytes();
        return bytes.len() == 40
            && bytes
                .iter()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(byte));
    }
    false
}

fn validate_json_schema(
    instance: &Value,
    schema: &Value,
    root: &Value,
    path: &str,
    errors: &mut Vec<String>,
    depth: usize,
) {
    if depth > 128 {
        errors.push(format!("{path}:SCHEMA_RECURSION_LIMIT"));
        return;
    }

    if let Some(reference) = schema.get("$ref").and_then(Value::as_str) {
        let Some(target) = resolve_reference(root, reference) else {
            errors.push(format!("{path}:SCHEMA_REFERENCE_UNRESOLVED:{reference}"));
            return;
        };
        validate_json_schema(instance, target, root, path, errors, depth + 1);
        return;
    }

    if let Some(expected) = schema.get("const") {
        if instance != expected {
            errors.push(format!("{path}:SCHEMA_CONST_MISMATCH"));
        }
    }
    if let Some(allowed) = schema.get("enum").and_then(Value::as_array) {
        if !allowed.iter().any(|candidate| candidate == instance) {
            errors.push(format!("{path}:SCHEMA_ENUM_MISMATCH"));
        }
    }
    if let Some(expected_type) = schema.get("type") {
        let matches = if let Some(type_name) = expected_type.as_str() {
            json_type_matches(instance, type_name)
        } else if let Some(type_names) = expected_type.as_array() {
            type_names.iter().any(|name| {
                name.as_str()
                    .is_some_and(|type_name| json_type_matches(instance, type_name))
            })
        } else {
            false
        };
        if !matches {
            errors.push(format!("{path}:SCHEMA_TYPE_MISMATCH"));
            return;
        }
    }

    if let Some(object) = instance.as_object() {
        if let Some(required) = schema.get("required").and_then(Value::as_array) {
            for field in required.iter().filter_map(Value::as_str) {
                if !object.contains_key(field) {
                    errors.push(format!("{path}:SCHEMA_REQUIRED_MISSING:{field}"));
                }
            }
        }
        let properties = schema.get("properties").and_then(Value::as_object);
        for (key, value) in object {
            if let Some(property_schema) = properties.and_then(|fields| fields.get(key)) {
                validate_json_schema(
                    value,
                    property_schema,
                    root,
                    &format!("{path}.{key}"),
                    errors,
                    depth + 1,
                );
            } else if schema.get("additionalProperties") == Some(&Value::Bool(false)) {
                errors.push(format!("{path}:SCHEMA_ADDITIONAL_PROPERTY:{key}"));
            } else if let Some(extra_schema) = schema.get("additionalProperties").filter(|value| value.is_object()) {
                validate_json_schema(
                    value,
                    extra_schema,
                    root,
                    &format!("{path}.{key}"),
                    errors,
                    depth + 1,
                );
            }
        }
    }

    if let Some(array) = instance.as_array() {
        if schema.get("minItems").and_then(Value::as_u64).is_some_and(|minimum| array.len() < minimum as usize) {
            errors.push(format!("{path}:SCHEMA_MIN_ITEMS"));
        }
        if schema.get("maxItems").and_then(Value::as_u64).is_some_and(|maximum| array.len() > maximum as usize) {
            errors.push(format!("{path}:SCHEMA_MAX_ITEMS"));
        }
        if schema.get("uniqueItems") == Some(&Value::Bool(true)) {
            let mut seen = BTreeSet::new();
            if array.iter().any(|item| !seen.insert(item.to_string())) {
                errors.push(format!("{path}:SCHEMA_DUPLICATE_ARRAY_ITEM"));
            }
        }
        if let Some(item_schema) = schema.get("items") {
            for (index, item) in array.iter().enumerate() {
                validate_json_schema(
                    item,
                    item_schema,
                    root,
                    &format!("{path}[{index}]"),
                    errors,
                    depth + 1,
                );
            }
        }
    }

    if let Some(value) = instance.as_str() {
        if schema.get("minLength").and_then(Value::as_u64).is_some_and(|minimum| value.chars().count() < minimum as usize) {
            errors.push(format!("{path}:SCHEMA_MIN_LENGTH"));
        }
        if schema.get("maxLength").and_then(Value::as_u64).is_some_and(|maximum| value.chars().count() > maximum as usize) {
            errors.push(format!("{path}:SCHEMA_MAX_LENGTH"));
        }
        if let Some(pattern) = schema.get("pattern").and_then(Value::as_str) {
            if !schema_pattern_matches(pattern, value) {
                errors.push(format!("{path}:SCHEMA_PATTERN_MISMATCH_OR_UNSUPPORTED"));
            }
        }
        if schema.get("format") == Some(&Value::String("date".to_owned())) && !date_is_valid(value) {
            errors.push(format!("{path}:SCHEMA_DATE_INVALID"));
        }
    }

    if let Some(value) = instance.as_f64().filter(|value| value.is_finite()) {
        if schema.get("minimum").and_then(Value::as_f64).is_some_and(|minimum| value < minimum) {
            errors.push(format!("{path}:SCHEMA_MINIMUM"));
        }
        if schema.get("maximum").and_then(Value::as_f64).is_some_and(|maximum| value > maximum) {
            errors.push(format!("{path}:SCHEMA_MAXIMUM"));
        }
    }
}

fn array_strings_equal(value: &Value, expected: &[&str]) -> bool {
    value.as_array().is_some_and(|values| {
        values.len() == expected.len()
            && values
                .iter()
                .zip(expected)
                .all(|(value, expected)| value.as_str() == Some(*expected))
    })
}

fn validate_schema_contracts(registry_schema: &Value, corpus_schema: &Value) -> Result<(), String> {
    if string_value(registry_schema, "title") != "Mycelix Building Measurement Condition Registry v0.1"
        || registry_schema["type"] != "object"
        || registry_schema["additionalProperties"] != false
        || !array_strings_equal(&registry_schema["required"], REGISTRY_FIELDS)
        || registry_schema["properties"]["profiles"]["minItems"] != 6
        || registry_schema["properties"]["profiles"]["maxItems"] != 6
        || registry_schema["properties"]["profiles"]["items"]["$ref"] != "#/$defs/profile"
    {
        return Err("REGISTRY_SCHEMA_CONTRACT_INVALID".to_owned());
    }
    if !array_strings_equal(
        &registry_schema["$defs"]["profile"]["required"],
        PROFILE_FIELDS,
    ) || registry_schema["$defs"]["profile"]["additionalProperties"] != false
    {
        return Err("REGISTRY_PROFILE_SCHEMA_INVALID".to_owned());
    }
    if string_value(corpus_schema, "title") != "Mycelix Measurement Condition Applicability Corpus v0.1"
        || corpus_schema["type"] != "object"
        || corpus_schema["additionalProperties"] != false
        || !array_strings_equal(&corpus_schema["required"], CORPUS_FIELDS)
        || corpus_schema["properties"]["cases"]["minItems"] != EXPECTED_CASE_COUNT
        || corpus_schema["properties"]["cases"]["maxItems"] != EXPECTED_CASE_COUNT
        || corpus_schema["properties"]["cases"]["items"]["$ref"] != "#/$defs/case"
    {
        return Err("CONDITION_CORPUS_SCHEMA_CONTRACT_INVALID".to_owned());
    }
    let case_schema = &corpus_schema["$defs"]["case"];
    if case_schema["additionalProperties"] != false
        || !array_strings_equal(&case_schema["required"], CASE_FIELDS)
        || case_schema["properties"]["record"]["additionalProperties"] != false
        || !array_strings_equal(&case_schema["properties"]["record"]["required"], RECORD_FIELDS)
    {
        return Err("CONDITION_CASE_SCHEMA_INVALID".to_owned());
    }
    Ok(())
}

#[derive(Clone, Copy)]
struct ExpectedVariable {
    id: &'static str,
    quantity_kind: &'static str,
    unit: &'static str,
    value_kind: &'static str,
    minimum: Option<f64>,
    maximum: Option<f64>,
    allowed: &'static [&'static str],
    maximum_uncertainty: Option<f64>,
}

const AIR_TEMP_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "ambient_temperature", quantity_kind: "temperature", unit: "degC", value_kind: "numeric", minimum: Some(10.0), maximum: Some(40.0), allowed: &[], maximum_uncertainty: Some(0.2) },
    ExpectedVariable { id: "ambient_relative_humidity", quantity_kind: "ratio", unit: "%RH", value_kind: "numeric", minimum: Some(20.0), maximum: Some(80.0), allowed: &[], maximum_uncertainty: Some(2.0) },
];
const PM_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "ambient_relative_humidity", quantity_kind: "ratio", unit: "%RH", value_kind: "numeric", minimum: Some(10.0), maximum: Some(70.0), allowed: &[], maximum_uncertainty: Some(2.0) },
    ExpectedVariable { id: "aerosol_challenge_class", quantity_kind: "aerosol_class", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_AEROSOL_A"], maximum_uncertainty: None },
    ExpectedVariable { id: "particle_size_profile", quantity_kind: "particle_size_distribution", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_PM_PROFILE_A"], maximum_uncertainty: None },
];
const GAS_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "ambient_temperature", quantity_kind: "temperature", unit: "degC", value_kind: "numeric", minimum: Some(10.0), maximum: Some(40.0), allowed: &[], maximum_uncertainty: Some(0.2) },
    ExpectedVariable { id: "ambient_relative_humidity", quantity_kind: "ratio", unit: "%RH", value_kind: "numeric", minimum: Some(20.0), maximum: Some(80.0), allowed: &[], maximum_uncertainty: Some(2.0) },
    ExpectedVariable { id: "interferent_profile", quantity_kind: "gas_interference_profile", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_GAS_MIX_A"], maximum_uncertainty: None },
];
const FLOW_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "fluid_identity", quantity_kind: "fluid_identity", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["water"], maximum_uncertainty: None },
    ExpectedVariable { id: "fluid_temperature", quantity_kind: "temperature", unit: "degC", value_kind: "numeric", minimum: Some(5.0), maximum: Some(40.0), allowed: &[], maximum_uncertainty: Some(0.2) },
    ExpectedVariable { id: "dynamic_viscosity", quantity_kind: "dynamic_viscosity", unit: "mPa.s", value_kind: "numeric", minimum: Some(0.5), maximum: Some(2.0), allowed: &[], maximum_uncertainty: Some(0.05) },
    ExpectedVariable { id: "flow_profile", quantity_kind: "flow_regime", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["fully_developed"], maximum_uncertainty: None },
    ExpectedVariable { id: "meter_installation_profile", quantity_kind: "installation_profile", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_STRAIGHT_RUN_A"], maximum_uncertainty: None },
];
const ELECTRIC_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "supply_frequency", quantity_kind: "frequency", unit: "Hz", value_kind: "numeric", minimum: Some(49.5), maximum: Some(50.5), allowed: &[], maximum_uncertainty: Some(0.01) },
    ExpectedVariable { id: "phase_configuration", quantity_kind: "phase_configuration", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["single_phase"], maximum_uncertainty: None },
    ExpectedVariable { id: "waveform_profile", quantity_kind: "waveform_profile", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_GRID_WAVEFORM_A"], maximum_uncertainty: None },
    ExpectedVariable { id: "power_factor", quantity_kind: "ratio", unit: "1", value_kind: "numeric", minimum: Some(0.7), maximum: Some(1.0), allowed: &[], maximum_uncertainty: Some(0.01) },
];
const ACOUSTIC_VARIABLES: &[ExpectedVariable] = &[
    ExpectedVariable { id: "microphone_orientation_error", quantity_kind: "angle", unit: "deg", value_kind: "numeric", minimum: Some(0.0), maximum: Some(10.0), allowed: &[], maximum_uncertainty: Some(1.0) },
    ExpectedVariable { id: "wind_speed", quantity_kind: "velocity", unit: "m/s", value_kind: "numeric", minimum: Some(0.0), maximum: Some(0.5), allowed: &[], maximum_uncertainty: Some(0.05) },
    ExpectedVariable { id: "background_noise_profile", quantity_kind: "acoustic_background_profile", unit: "1", value_kind: "categorical", minimum: None, maximum: None, allowed: &["SYNTH_BACKGROUND_A"], maximum_uncertainty: None },
];

fn expected_variables(profile_id: &str) -> Option<&'static [ExpectedVariable]> {
    match profile_id {
        "COND-AIR-TEMP-001" => Some(AIR_TEMP_VARIABLES),
        "COND-OPTICAL-PM-001" => Some(PM_VARIABLES),
        "COND-GAS-001" => Some(GAS_VARIABLES),
        "COND-WATER-FLOW-001" => Some(FLOW_VARIABLES),
        "COND-ELECTRICAL-001" => Some(ELECTRIC_VARIABLES),
        "COND-ACOUSTIC-001" => Some(ACOUSTIC_VARIABLES),
        _ => None,
    }
}

fn validate_profile(profile: &Value, index: usize) -> Result<(), String> {
    let Some((expected_id, conditions_id, channel_id, measurand_id, firmware, config, count)) =
        EXPECTED_PROFILES.get(index).copied()
    else {
        return Err("PROFILE_INDEX_OUT_OF_RANGE".to_owned());
    };
    if !exact_keys(profile, PROFILE_FIELDS)
        || string_value(profile, "profile_id") != expected_id
        || string_value(profile, "conditions_id") != conditions_id
        || string_value(profile, "channel_profile_id") != PASSPORT_PROFILE_ID
        || string_value(profile, "channel_id") != channel_id
        || string_value(profile, "measurand_id") != measurand_id
        || string_value(profile, "firmware_revision") != firmware
        || string_value(profile, "instrument_configuration_digest") != config
        || string_value(profile, "profile_evidence_state") != "synthetic_reference_only"
        || string_value(profile, "revision") != "1"
        || !date_is_valid(string_value(profile, "valid_from_date"))
        || !date_is_valid(string_value(profile, "valid_to_date"))
        || string_value(profile, "valid_from_date") > string_value(profile, "valid_to_date")
    {
        return Err(format!("PROFILE_IDENTITY_OR_SYNTHETIC_CEILING_INVALID:{expected_id}"));
    }

    let expected = expected_variables(expected_id).ok_or("PROFILE_VARIABLE_EXPECTATION_MISSING")?;
    let variables = profile["variables"].as_array().ok_or("PROFILE_VARIABLES_MISSING")?;
    if variables.len() != count || variables.len() != expected.len() {
        return Err(format!("PROFILE_VARIABLE_COUNT_INVALID:{expected_id}"));
    }
    for (variable, expected) in variables.iter().zip(expected) {
        if !exact_keys(variable, VARIABLE_FIELDS)
            || string_value(variable, "variable_id") != expected.id
            || string_value(variable, "quantity_kind") != expected.quantity_kind
            || string_value(variable, "unit") != expected.unit
            || string_value(variable, "value_kind") != expected.value_kind
            || variable["required"] != true
        {
            return Err(format!("PROFILE_VARIABLE_IDENTITY_INVALID:{}:{}", expected_id, expected.id));
        }
        match expected.value_kind {
            "numeric" => {
                if number_value(variable, "minimum") != expected.minimum
                    || number_value(variable, "maximum") != expected.maximum
                    || number_value(variable, "maximum_uncertainty") != expected.maximum_uncertainty
                    || !variable["allowed_values"].as_array().is_some_and(Vec::is_empty)
                {
                    return Err(format!("PROFILE_NUMERIC_ENVELOPE_DRIFT:{}:{}", expected_id, expected.id));
                }
            }
            "categorical" => {
                if !variable["minimum"].is_null()
                    || !variable["maximum"].is_null()
                    || !variable["maximum_uncertainty"].is_null()
                    || !array_strings_equal(
                        &variable["allowed_values"],
                        expected.allowed,
                    )
                {
                    return Err(format!("PROFILE_CATEGORY_ENVELOPE_DRIFT:{}:{}", expected_id, expected.id));
                }
            }
            _ => return Err("PROFILE_VALUE_KIND_UNSUPPORTED".to_owned()),
        }
    }

    let spatial = &profile["spatial_requirements"];
    if !exact_keys(spatial, SPATIAL_FIELDS)
        || string_value(spatial, "geometry_revision") != "GEOM-SYNTH-1"
        || string_value(spatial, "location_context_id") != "SENSOR-LOCAL-01"
        || string_value(spatial, "required_relation") != "colocated"
        || number_value(spatial, "max_transfer_distance_m") != Some(0.0)
        || string_value(spatial, "transfer_method_id") != "NOT_APPLICABLE"
        || string_value(spatial, "transfer_validation_receipt_id") != "NOT_APPLICABLE"
    {
        return Err(format!("PROFILE_SPATIAL_CONTRACT_DRIFT:{expected_id}"));
    }

    let temporal = &profile["temporal_requirements"];
    if !exact_keys(temporal, TEMPORAL_FIELDS)
        || string_value(temporal, "timebase_contract_id") != TIMEBASE_CONTRACT_ID
        || temporal["require_full_sample_window"] != true
        || number_value(temporal, "minimum_coverage_fraction") != Some(1.0)
        || string_value(temporal, "clock_domain_id") != "CLOCK-SYNTH-01"
        || string_value(temporal, "boot_epoch_id") != "BOOT-SYNTH-01"
        || string_value(temporal, "tick_unit") != "s"
    {
        return Err(format!("PROFILE_TEMPORAL_CONTRACT_DRIFT:{expected_id}"));
    }

    let correction = &profile["correction_policy"];
    if !exact_keys(correction, CORRECTION_FIELDS) {
        return Err(format!("PROFILE_CORRECTION_FIELDS_INVALID:{expected_id}"));
    }
    if expected_id == "COND-GAS-001" {
        if string_value(correction, "state") != "required"
            || string_value(correction, "method_id") != "SYNTH-GAS-COMP-01"
            || string_value(correction, "method_revision") != "1"
            || string_value(correction, "method_digest") != "fixture-gas-correction-v1"
            || string_value(correction, "validation_receipt_id") != "SYNTH-GAS-CORRECTION-VALIDATION-01"
            || correction["must_apply_once"] != true
        {
            return Err("PROFILE_GAS_CORRECTION_DRIFT".to_owned());
        }
    } else if string_value(correction, "state") != "not_required"
        || string_value(correction, "method_id") != "NOT_APPLICABLE"
        || string_value(correction, "method_revision") != "NOT_APPLICABLE"
        || string_value(correction, "method_digest") != "NOT_APPLICABLE"
        || string_value(correction, "validation_receipt_id") != "NOT_APPLICABLE"
        || correction["must_apply_once"] != false
    {
        return Err(format!("PROFILE_UNEXPECTED_CORRECTION:{expected_id}"));
    }

    Ok(())
}

fn array_strings_equal(value: &Value, expected: &[&str]) -> bool {
    value.as_array().is_some_and(|values| {
        values.len() == expected.len()
            && values
                .iter()
                .zip(expected)
                .all(|(value, expected)| value.as_str() == Some(*expected))
    })
}

pub fn classify_record(record: &Value) -> &'static str {
    if string_value(record, "schema_version") != CONDITION_RECORD_VERSION {
        return "NOT_COMPARABLE";
    }
    let passport = &record["passport_binding"];
    let binding = &record["registry_binding"];
    if string_value(passport, "passport_profile_id") != PASSPORT_PROFILE_ID
        || string_value(binding, "registry_id") != REGISTRY_ID
        || string_value(binding, "registry_blob_sha1") != REGISTRY_BLOB_SHA1
    {
        return "NOT_COMPARABLE";
    }
    let profile_id = string_value(binding, "profile_id");
    let Some((expected_id, conditions_id, channel_id, measurand_id, firmware, config, _)) =
        EXPECTED_PROFILES.iter().find(|item| item.0 == profile_id).copied()
    else {
        return "NOT_COMPARABLE";
    };
    let Some(profile_variables) = expected_variables(expected_id) else {
        return "NOT_COMPARABLE";
    };

    if string_value(binding, "profile_revision") != "1"
        || string_value(binding, "conditions_id") != conditions_id
        || string_value(passport, "capability_conditions_id") != conditions_id
        || string_value(passport, "calibration_conditions_id") != conditions_id
        || string_value(passport, "accuracy_conditions_id") != conditions_id
        || string_value(passport, "drift_conditions_id") != conditions_id
        || string_value(passport, "channel_id") != channel_id
        || string_value(passport, "measurand_id") != measurand_id
    {
        return "NOT_COMPARABLE";
    }
    if string_value(passport, "firmware_revision") != firmware {
        return "EXPIRED_OR_INVALIDATED";
    }
    if string_value(passport, "instrument_configuration_digest") != config {
        return "NOT_COMPARABLE";
    }
    if string_value(binding, "profile_evidence_state") == "revoked" {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if string_value(binding, "profile_evidence_state") != "synthetic_reference_only" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    let observed_date = string_value(passport, "observed_at_date");
    let valid = match profile_id {
        "COND-AIR-TEMP-001" | "COND-OPTICAL-PM-001" | "COND-GAS-001"
        | "COND-WATER-FLOW-001" | "COND-ELECTRICAL-001" | "COND-ACOUSTIC-001" => {
            date_is_valid(observed_date) && ("2026-01-01" <= observed_date && observed_date <= "2027-12-31")
        }
        _ => false,
    };
    if !valid {
        return "EXPIRED_OR_INVALIDATED";
    }
    if string_value(passport, "sample_rights_state") == "revoked" {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if string_value(passport, "sample_rights_state") != "synthetic_reference_only" {
        return "MORE_EVIDENCE_REQUIRED";
    }

    let sample_time = &passport["sample_time_support"];
    if string_value(sample_time, "timebase_contract_id") != TIMEBASE_CONTRACT_ID
        || string_value(sample_time, "clock_domain_id") != "CLOCK-SYNTH-01"
        || string_value(sample_time, "boot_epoch_id") != "BOOT-SYNTH-01"
        || string_value(sample_time, "tick_unit") != "s"
    {
        return "NOT_COMPARABLE";
    }
    if string_value(sample_time, "timebase_state") != "synthetic_reference_only"
        || !required_text(sample_time, "timebase_receipt_id")
        || number_value(sample_time, "coverage_fraction") != Some(1.0)
        || number_value(sample_time, "start_tick").is_none_or(|start| {
            start > number_value(sample_time, "end_tick").unwrap_or(f64::NEG_INFINITY)
        })
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    let observations = match record["condition_observations"].as_array() {
        Some(values) => values,
        None => return "MORE_EVIDENCE_REQUIRED",
    };
    let mut observed_ids = BTreeSet::new();
    for observation in observations {
        let id = string_value(observation, "variable_id");
        if id.is_empty() || !observed_ids.insert(id.to_owned()) {
            return "NOT_COMPARABLE";
        }
    }

    let sample_time = &passport["sample_time_support"];
    for expected in profile_variables {
        let Some(observation) = observations
            .iter()
            .find(|observation| string_value(observation, "variable_id") == expected.id)
        else {
            return "MORE_EVIDENCE_REQUIRED";
        };

        if string_value(observation, "source_evidence_state") == "revoked"
            || string_value(observation, "rights_state") == "revoked"
        {
            return "REJECTED_FOR_DECLARED_PURPOSE";
        }
        if string_value(observation, "calibration_state") == "expired"
            || string_value(observation, "installation_state") == "invalidated"
        {
            return "EXPIRED_OR_INVALIDATED";
        }
        if string_value(observation, "source_evidence_state") != "synthetic_reference_only"
            || string_value(observation, "calibration_state") != "synthetic_reference_only"
            || string_value(observation, "installation_state") != "synthetic_reference_only"
            || string_value(observation, "rights_state") != "synthetic_reference_only"
            || string_value(observation, "evidence_class") != "synthetic_reference_only"
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if string_value(observation, "value_kind") != expected.value_kind
            || string_value(observation, "unit") != expected.unit
        {
            return "NOT_COMPARABLE";
        }

        match expected.value_kind {
            "numeric" => {
                let Some(value) = number_value(observation, "numeric_value") else {
                    return "MORE_EVIDENCE_REQUIRED";
                };
                if value < expected.minimum.unwrap_or(f64::NEG_INFINITY)
                    || value > expected.maximum.unwrap_or(f64::INFINITY)
                {
                    return "OUTSIDE_VALIDATED_ENVELOPE";
                }
                if number_value(observation, "uncertainty_value")
                    .is_none_or(|uncertainty| uncertainty < 0.0 || uncertainty > expected.maximum_uncertainty.unwrap_or(f64::INFINITY))
                    || string_value(observation, "uncertainty_unit") != expected.unit
                {
                    return "MORE_EVIDENCE_REQUIRED";
                }
            }
            "categorical" => {
                if !expected
                    .allowed
                    .contains(&string_value(observation, "category_value"))
                {
                    return "NOT_COMPARABLE";
                }
            }
            _ => return "NOT_COMPARABLE",
        }

        if string_value(observation, "spatial_relation") == "same_zone_only" {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if string_value(observation, "geometry_revision") != "GEOM-SYNTH-1"
            || string_value(observation, "location_context_id") != "SENSOR-LOCAL-01"
        {
            return "NOT_COMPARABLE";
        }
        if string_value(observation, "spatial_relation") != "colocated" {
            return "NOT_COMPARABLE";
        }
        if string_value(observation, "transfer_method_id") != "NOT_APPLICABLE"
            || string_value(observation, "transfer_validation_receipt_id") != "NOT_APPLICABLE"
            || number_value(observation, "distance_m") != Some(0.0)
        {
            return "NOT_COMPARABLE";
        }

        let support = &observation["time_support"];
        if string_value(support, "timebase_contract_id") != TIMEBASE_CONTRACT_ID
            || string_value(sample_time, "timebase_contract_id") != TIMEBASE_CONTRACT_ID
            || string_value(support, "clock_domain_id") != "CLOCK-SYNTH-01"
            || string_value(support, "clock_domain_id") != string_value(sample_time, "clock_domain_id")
            || string_value(support, "boot_epoch_id") != "BOOT-SYNTH-01"
            || string_value(support, "boot_epoch_id") != string_value(sample_time, "boot_epoch_id")
            || string_value(support, "tick_unit") != "s"
            || string_value(support, "tick_unit") != string_value(sample_time, "tick_unit")
            || string_value(support, "stream_id") != string_value(sample_time, "stream_id")
        {
            return "NOT_COMPARABLE";
        }
        if string_value(support, "timebase_state") != "synthetic_reference_only"
            || !required_text(support, "timebase_receipt_id")
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if number_value(support, "coverage_fraction") != Some(1.0)
            || number_value(support, "start_tick").is_none_or(|start| start > number_value(sample_time, "start_tick").unwrap_or(f64::INFINITY))
            || number_value(support, "end_tick").is_none_or(|end| end < number_value(sample_time, "end_tick").unwrap_or(f64::NEG_INFINITY))
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
    }

    let policy = &record["correction_application"];
    let expected_correction = profile_id == "COND-GAS-001";
    if !expected_correction {
        if string_value(policy, "state") != "not_applied"
            || number_value(policy, "application_count") != Some(0.0)
        {
            return "REJECTED_FOR_DECLARED_PURPOSE";
        }
    } else {
        if string_value(policy, "state") != "applied" {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if string_value(policy, "method_id") != "SYNTH-GAS-COMP-01"
            || string_value(policy, "method_revision") != "1"
            || string_value(policy, "method_digest") != "fixture-gas-correction-v1"
        {
            return "NOT_COMPARABLE";
        }
        if string_value(policy, "validation_receipt_id") != "SYNTH-GAS-CORRECTION-VALIDATION-01"
            || !required_text(policy, "source_conditions_digest")
            || !required_text(policy, "output_sample_digest")
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if number_value(policy, "application_count") != Some(1.0) {
            return "REJECTED_FOR_DECLARED_PURPOSE";
        }
    }

    "SYNTHETIC_CONDITION_CONTRACT_VALIDATION_ONLY"
}

fn validate_registry(registry: &Value) -> Result<usize, String> {
    if !exact_keys(registry, REGISTRY_FIELDS)
        || string_value(registry, "schema_version") != PROFILE_SCHEMA_VERSION
        || string_value(registry, "registry_id") != REGISTRY_ID
        || string_value(registry, "classification") != "synthetic_condition_profiles_only"
        || string_value(registry, "claim_ceiling") != "profile_and_sample_condition_applicability_conformance_only"
    {
        return Err("REGISTRY_IDENTITY_OR_CLAIM_CEILING_INVALID".to_owned());
    }
    let profiles = registry["profiles"].as_array().ok_or("REGISTRY_PROFILES_MISSING")?;
    if profiles.len() != EXPECTED_PROFILES.len() {
        return Err("REGISTRY_PROFILE_COUNT_INVALID".to_owned());
    }
    for (index, profile) in profiles.iter().enumerate() {
        validate_profile(profile, index)?;
    }
    Ok(profiles.len())
}

fn validate_corpus(registry: &Value, corpus: &Value) -> Result<usize, String> {
    if !exact_keys(corpus, CORPUS_FIELDS)
        || string_value(corpus, "schema_version") != CORPUS_SCHEMA_VERSION
        || string_value(corpus, "corpus_id") != CORPUS_ID
        || string_value(corpus, "registry_id") != REGISTRY_ID
        || string_value(corpus, "registry_blob_sha1") != REGISTRY_BLOB_SHA1
        || string_value(corpus, "classification") != "synthetic_condition_applicability_cases"
        || string_value(corpus, "claim_ceiling") != "synthetic_environmental_and_influence_condition_applicability_only"
    {
        return Err("CORPUS_IDENTITY_OR_CLAIM_CEILING_INVALID".to_owned());
    }
    if validate_registry(registry)? != EXPECTED_PROFILES.len() {
        return Err("REGISTRY_NOT_VALIDATED".to_owned());
    }
    let cases = corpus["cases"].as_array().ok_or("CORPUS_CASES_MISSING")?;
    if cases.len() != EXPECTED_CASE_COUNT {
        return Err("CORPUS_CASE_COUNT_INVALID".to_owned());
    }
    for (index, (case, (expected_id, expected_disposition))) in
        cases.iter().zip(EXPECTED_CASES).enumerate()
    {
        if !exact_keys(case, CASE_FIELDS)
            || string_value(case, "case_id") != *expected_id
            || string_value(case, "description").chars().count() < 20
            || string_value(case, "expected_disposition") != *expected_disposition
        {
            return Err(format!("CASE_ID_OR_EXPECTED_DISPOSITION_DRIFT:{expected_id}"));
        }
        let record = &case["record"];
        if !exact_keys(record, RECORD_FIELDS)
            || !exact_keys(&record["passport_binding"], PASSPORT_BINDING_FIELDS)
            || !exact_keys(&record["registry_binding"], REGISTRY_BINDING_FIELDS)
            || !exact_keys(&record["correction_application"], CORRECTION_APPLICATION_FIELDS)
        {
            return Err(format!("CASE_RECORD_FIELDS_INVALID:{expected_id}"));
        }
        if record["condition_observations"]
            .as_array()
            .is_none_or(|observations| observations.iter().any(|observation| !exact_keys(observation, OBSERVATION_FIELDS) || !exact_keys(&observation["time_support"], TIME_FIELDS)))
        {
            return Err(format!("CASE_OBSERVATION_FIELDS_INVALID:{expected_id}"));
        }
        let actual = classify_record(record);
        if actual != *expected_disposition {
            return Err(format!("CASE_DISPOSITION_MISMATCH:{}:expected={expected_disposition}:actual={actual}", index + 1));
        }
    }
    Ok(cases.len())
}

pub fn validate_tracked() -> Result<(usize, usize), String> {
    let registry_schema = parse_unique_json(REGISTRY_SCHEMA_JSON)?;
    let corpus_schema = parse_unique_json(CORPUS_SCHEMA_JSON)?;
    validate_schema_contracts(&registry_schema, &corpus_schema)?;

    let registry = parse_unique_json(REGISTRY_JSON)?;
    let corpus = parse_unique_json(CORPUS_JSON)?;

    let mut registry_errors = Vec::new();
    validate_json_schema(
        &registry,
        &registry_schema,
        &registry_schema,
        "$registry",
        &mut registry_errors,
        0,
    );
    if !registry_errors.is_empty() {
        return Err(format!("REGISTRY_SCHEMA_VALIDATION_FAILED:{registry_errors:?}"));
    }

    let mut corpus_errors = Vec::new();
    validate_json_schema(
        &corpus,
        &corpus_schema,
        &corpus_schema,
        "$corpus",
        &mut corpus_errors,
        0,
    );
    if !corpus_errors.is_empty() {
        return Err(format!("CORPUS_SCHEMA_VALIDATION_FAILED:{corpus_errors:?}"));
    }

    let profile_count = validate_registry(&registry)?;
    let case_count = validate_corpus(&registry, &corpus)?;
    Ok((profile_count, case_count))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(case_id: &str) -> Value {
        let corpus = parse_unique_json(CORPUS_JSON).expect("valid corpus JSON");
        corpus["cases"]
            .as_array()
            .unwrap()
            .iter()
            .find(|case| string_value(case, "case_id") == case_id)
            .cloned()
            .expect("case exists")
    }

    #[test]
    fn tracked_registry_and_all_thirty_six_known_answers_validate() {
        assert_eq!(validate_tracked().expect("tracked validation"), (6, 43));
    }

    #[test]
    fn duplicate_json_members_are_rejected_at_nested_depth() {
        assert!(parse_unique_json(r#"{"outer":{"unit":"degC","unit":"K"}}"#).is_err());
    }

    #[test]
    fn missing_registry_digest_is_not_comparable() {
        assert_eq!(classify_record(&case("MC-002")["record"]), "NOT_COMPARABLE");
    }

    #[test]
    fn mismatched_registry_digest_is_not_comparable() {
        assert_eq!(classify_record(&case("MC-003")["record"]), "NOT_COMPARABLE");
    }

    #[test]
    fn outside_thermal_envelope_is_not_eligible() {
        assert_eq!(classify_record(&case("MC-012")["record"]), "OUTSIDE_VALIDATED_ENVELOPE");
    }

    #[test]
    fn stale_condition_evidence_requires_more_evidence() {
        assert_eq!(classify_record(&case("MC-010")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn same_zone_does_not_substitute_for_sensor_local_conditions() {
        assert_eq!(classify_record(&case("MC-016")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn condition_sample_must_cover_the_full_observation_window() {
        assert_eq!(classify_record(&case("MC-020")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn unresolved_timebase_cannot_support_applicability() {
        assert_eq!(classify_record(&case("MC-021")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn provider_asserted_condition_data_are_not_promoted_to_synthetic_verified_state() {
        assert_eq!(classify_record(&case("MC-030")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn profile_revocation_blocks_conditions_use() {
        assert_eq!(classify_record(&case("MC-005")["record"]), "REJECTED_FOR_DECLARED_PURPOSE");
    }

    #[test]
    fn condition_data_rights_revocation_blocks_use() {
        assert_eq!(classify_record(&case("MC-029")["record"]), "REJECTED_FOR_DECLARED_PURPOSE");
    }

    #[test]
    fn profile_revision_and_four_passport_condition_references_are_exact() {
        assert_eq!(classify_record(&case("MC-006")["record"]), "NOT_COMPARABLE");
    }

    #[test]
    fn configuration_and_firmware_are_separate_bindings() {
        assert_eq!(classify_record(&case("MC-023")["record"]), "EXPIRED_OR_INVALIDATED");
        assert_eq!(classify_record(&case("MC-024")["record"]), "NOT_COMPARABLE");
    }

    #[test]
    fn correction_must_match_exact_method_revision_digest_and_single_application() {
        assert_eq!(classify_record(&case("MC-026")["record"]), "MORE_EVIDENCE_REQUIRED");
        assert_eq!(classify_record(&case("MC-027")["record"]), "NOT_COMPARABLE");
        assert_eq!(classify_record(&case("MC-028")["record"]), "REJECTED_FOR_DECLARED_PURPOSE");
    }

    #[test]
    fn categorical_and_numeric_condition_domains_are_not_interchangeable() {
        assert_eq!(classify_record(&case("MC-032")["record"]), "NOT_COMPARABLE");
        assert_eq!(classify_record(&case("MC-034")["record"]), "NOT_COMPARABLE");
    }

    #[test]
    fn condition_order_does_not_change_the_disposition() {
        let mut positive = case("MC-001")["record"].clone();
        let original = classify_record(&positive);
        positive["condition_observations"]
            .as_array_mut()
            .unwrap()
            .reverse();
        assert_eq!(classify_record(&positive), original);
    }

    #[test]
    fn only_synthetic_reference_records_can_reach_review_eligibility() {
        let mut positive = case("MC-001")["record"].clone();
        assert_eq!(
            classify_record(&positive),
            "SYNTHETIC_CONDITION_CONTRACT_VALIDATION_ONLY"
        );
        positive["registry_binding"]["profile_evidence_state"] =
            Value::String("independently_characterized".to_owned());
        assert_eq!(classify_record(&positive), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn passport_sample_timebase_must_be_resolved() {
        assert_eq!(classify_record(&case("MC-037")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn passport_sample_rights_state_is_checked_independently() {
        assert_eq!(classify_record(&case("MC-038")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn expired_condition_calibration_invalidates_applicability() {
        assert_eq!(classify_record(&case("MC-039")["record"]), "EXPIRED_OR_INVALIDATED");
    }

    #[test]
    fn invalidated_condition_installation_invalidates_applicability() {
        assert_eq!(classify_record(&case("MC-040")["record"]), "EXPIRED_OR_INVALIDATED");
    }

    #[test]
    fn revoked_condition_source_is_rejected() {
        assert_eq!(classify_record(&case("MC-041")["record"]), "REJECTED_FOR_DECLARED_PURPOSE");
    }

    #[test]
    fn unknown_condition_data_rights_require_more_evidence() {
        assert_eq!(classify_record(&case("MC-042")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn required_correction_needs_an_exact_validation_receipt() {
        assert_eq!(classify_record(&case("MC-043")["record"]), "MORE_EVIDENCE_REQUIRED");
    }

    #[test]
    fn unknown_fields_are_rejected_by_the_closed_schema() {
        let schema = parse_unique_json(CORPUS_SCHEMA_JSON).expect("schema");
        let mut corpus = parse_unique_json(CORPUS_JSON).expect("corpus");
        corpus["cases"][0]["record"]["surprise"] = Value::Bool(true);
        let mut errors = Vec::new();
        validate_json_schema(&corpus, &schema, &schema, "$", &mut errors, 0);
        assert!(errors.iter().any(|error| error.contains("SCHEMA_ADDITIONAL_PROPERTY")));
    }
}
