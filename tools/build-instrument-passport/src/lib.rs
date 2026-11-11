//! Synthetic-only building instrument passport applicability checker.
//!
//! This crate verifies a purpose-specific evidence chain. It does not calibrate
//! instruments, validate laboratories, approve installations, or authorize use.

#![forbid(unsafe_code)]

use serde::de::{MapAccess, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer};
use serde_json::{Map, Value};
use std::collections::BTreeSet;
use std::fmt;

pub const PASSPORT_SCHEMA_JSON: &str =
    include_str!("../../../docs/metrology/BUILDING_INSTRUMENT_PASSPORT_SCHEMA_V0_1.json");
pub const CORPUS_SCHEMA_JSON: &str =
    include_str!("../../../docs/metrology/BUILDING_INSTRUMENT_PASSPORT_CASES_V0_1.schema.json");
pub const CORPUS_JSON: &str =
    include_str!("../../../docs/metrology/BUILDING_INSTRUMENT_PASSPORT_CASES_V0_1.json");

const PROFILE_ID: &str = "MYC-BUILD-METROLOGY-PROFILE-V0.1";
const PASSPORT_VERSION: &str = "mycelix.building-instrument-passport/0.1";
const CORPUS_ID: &str = "BUILDING-INSTRUMENT-PASSPORT-CORPUS-V0.1";
const EXPECTED_CASE_COUNT: usize = 59;

const ROOT_FIELDS: &[&str] = &[
    "passport_schema_version",
    "profile_id",
    "instrument",
    "capability",
    "calibration",
    "installation",
    "sample",
    "uncertainty_budget",
    "purpose_profile",
    "claim_requested",
    "independent_assay",
];
const INSTRUMENT_FIELDS: &[&str] = &[
    "instrument_id",
    "manufacturer",
    "model",
    "serial_id",
    "hardware_revision",
    "firmware_revision",
    "protocol_profile_id",
    "protocol_profile_digest",
    "protocol_profile_state",
    "local_read_only_supported",
    "protocol_profile_verification_receipt_id",
    "protocol_profile_verifier_id",
    "channel_id",
    "channel_profile_id",
    "identity_state",
    "serial_identity_reused",
    "lifecycle_state",
];
const CAPABILITY_FIELDS: &[&str] = &[
    "measurement_kind",
    "operating_range",
    "resolution",
    "accuracy",
    "response_time_seconds",
    "verification_state",
    "repeatability",
    "drift",
    "cross_sensitivity_state",
];
const OPERATING_RANGE_FIELDS: &[&str] = &["minimum","maximum","unit","conditions_id"];
const RESOLUTION_FIELDS: &[&str] = &["value","unit"];
const ACCURACY_FIELDS: &[&str] = &["value","unit","basis","conditions_id","evidence_class","source_record_id"];
const REPEATABILITY_FIELDS: &[&str] = &["value","unit","method_id","evidence_class"];
const DRIFT_FIELDS: &[&str] = &["value","unit","period_seconds","conditions_id"];
const CALIBRATION_UNCERTAINTY_FIELDS: &[&str] = &["value","unit","coverage_factor","method_id","correlation_model_id"];
const POSITION_FIELDS: &[&str] = &["x","y","z","frame_id"];
const CALIBRATION_FIELDS: &[&str] = &[
    "record_id",
    "instrument_id",
    "channel_id",
    "channel_profile_id",
    "firmware_revision",
    "certificate_digest",
    "certificate_state",
    "calibration_state",
    "lab_id",
    "accreditation_id",
    "accreditation_scope_id",
    "accreditation_scope_state",
    "method_id",
    "method_revision",
    "measurand_id",
    "unit",
    "range_minimum",
    "range_maximum",
    "calibration_date",
    "due_date",
    "reference_standard_ids",
    "traceability_state",
    "as_found_state",
    "as_left_state",
    "prior_data_impact_state",
    "uncertainty",
];
const INSTALLATION_FIELDS: &[&str] = &[
    "record_id",
    "asset_id",
    "building_id",
    "room_zone",
    "geometry_revision",
    "position",
    "height_m",
    "orientation_degrees",
    "mounting_method",
    "shielding_state",
    "enclosure_state",
    "location_context_id",
    "installation_state",
    "verifier_id",
    "verifier_competence_state",
    "installed_at",
    "relocation_state",
];
const SAMPLE_FIELDS: &[&str] = &[
    "observation_id",
    "instrument_id",
    "channel_id",
    "channel_profile_id",
    "firmware_revision",
    "value",
    "unit",
    "observed_at_date",
    "qc_state",
    "event_time_seconds",
    "acquisition_time_seconds",
    "time_unit",
    "timezone",
    "clock_source",
    "synchronization_uncertainty_seconds",
    "sequence",
    "processing_mode",
    "processing_method_id",
    "processing_method_revision",
    "raw_source_digest",
    "calibration_record_id",
    "installation_record_id",
    "rights_state",
    "acquisition_mode",
    "local_read_only",
    "protocol_profile_digest",
    "read_only_session_receipt_id",
];
const UNCERTAINTY_FIELDS: &[&str] = &[
    "state",
    "combination_method_id",
    "correlation_model_id",
    "coverage_factor",
    "expanded_uncertainty_value",
    "unit",
    "components",
];
const UNCERTAINTY_COMPONENT_FIELDS: &[&str] = &[
    "component_id",
    "value",
    "unit",
    "component_kind",
    "correlation_group",
    "source_record_id",
];
const PURPOSE_FIELDS: &[&str] = &[
    "purpose_id",
    "channel_id",
    "measurand_id",
    "unit",
    "acceptable_minimum",
    "acceptable_maximum",
    "maximum_expanded_uncertainty",
    "maximum_response_time_seconds",
    "maximum_time_sync_uncertainty_seconds",
    "required_geometry_revision",
    "required_location_context_id",
    "classification",
];
const INDEPENDENT_ASSAY_FIELDS: &[&str] = &[
    "state",
    "assay_id",
    "laboratory_id",
    "accreditation_scope_id",
    "method_id",
    "method_revision",
    "sample_id",
    "sample_digest",
    "measurand_id",
    "value",
    "unit",
    "certificate_digest",
    "certificate_state",
    "accreditation_scope_state",
    "rights_state",
];
const CASE_FIELDS: &[&str] = &[
    "case_id",
    "description",
    "record",
    "expected_disposition",
];

#[derive(Clone, Copy)]
struct Channel {
    id: &'static str,
    measurand: &'static str,
    _quantity_kind: &'static str,
    unit: &'static str,
    raw_or_derived: &'static str,
    independent_assay: bool,
}

const CHANNELS: &[Channel] = &[
    Channel { id: "air_temperature", measurand: "habitat.air_temperature", _quantity_kind: "temperature", unit: "degC", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "mean_radiant_temperature", measurand: "habitat.mean_radiant_temperature", _quantity_kind: "temperature", unit: "degC", raw_or_derived: "derived", independent_assay: false },
    Channel { id: "relative_humidity", measurand: "habitat.relative_humidity", _quantity_kind: "ratio", unit: "%RH", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "air_speed", measurand: "habitat.air_speed", _quantity_kind: "velocity", unit: "m/s", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "co2_concentration", measurand: "habitat.co2_concentration", _quantity_kind: "mole_fraction", unit: "ppm", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "pm25_mass_concentration", measurand: "habitat.pm25_mass_concentration", _quantity_kind: "mass_concentration", unit: "ug/m3", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "outdoor_air_volume_flow", measurand: "habitat.outdoor_air_volume_flow", _quantity_kind: "volume_flow", unit: "m3/s", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "water_volume_flow", measurand: "habitat.water_volume_flow", _quantity_kind: "volume_flow", unit: "m3/s", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "water_volume", measurand: "habitat.water_volume", _quantity_kind: "volume", unit: "m3", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "water_ph", measurand: "habitat.water_ph", _quantity_kind: "pH", unit: "1", raw_or_derived: "raw", independent_assay: true },
    Channel { id: "water_conductivity", measurand: "habitat.water_conductivity", _quantity_kind: "conductivity", unit: "uS/cm", raw_or_derived: "raw", independent_assay: true },
    Channel { id: "electrical_power", measurand: "habitat.electrical_power", _quantity_kind: "power", unit: "W", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "electrical_energy", measurand: "habitat.electrical_energy", _quantity_kind: "energy", unit: "kWh", raw_or_derived: "raw", independent_assay: false },
    Channel { id: "useful_thermal_energy", measurand: "habitat.useful_thermal_energy", _quantity_kind: "thermal_energy", unit: "kWh_th", raw_or_derived: "derived", independent_assay: false },
];

const EXPECTED_CASES: &[(&str, &str)] = &[
    ("BIP-001", "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"),
    ("BIP-002", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-003", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-004", "NOT_COMPARABLE"),
    ("BIP-005", "NOT_COMPARABLE"),
    ("BIP-006", "EXPIRED_OR_INVALIDATED"),
    ("BIP-007", "EXPIRED_OR_INVALIDATED"),
    ("BIP-008", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-009", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-010", "NOT_COMPARABLE"),
    ("BIP-011", "EXPIRED_OR_INVALIDATED"),
    ("BIP-012", "EXPIRED_OR_INVALIDATED"),
    ("BIP-013", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-014", "EXPIRED_OR_INVALIDATED"),
    ("BIP-015", "NOT_COMPARABLE"),
    ("BIP-016", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-017", "EXPIRED_OR_INVALIDATED"),
    ("BIP-018", "NOT_COMPARABLE"),
    ("BIP-019", "NOT_COMPARABLE"),
    ("BIP-020", "NOT_COMPARABLE"),
    ("BIP-021", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-022", "EXPIRED_OR_INVALIDATED"),
    ("BIP-023", "EXPIRED_OR_INVALIDATED"),
    ("BIP-024", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-025", "EXPIRED_OR_INVALIDATED"),
    ("BIP-026", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-027", "NOT_COMPARABLE"),
    ("BIP-028", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-029", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-030", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-031", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-032", "NOT_COMPARABLE"),
    ("BIP-033", "REJECTED_CLAIM_PROMOTION"),
    ("BIP-034", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-035", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-036", "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"),
    ("BIP-037", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-038", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-039", "NOT_COMPARABLE"),
    ("BIP-040", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-041", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-042", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-043", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-044", "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"),
    ("BIP-045", "NOT_COMPARABLE"),
    ("BIP-046", "EXPIRED_OR_INVALIDATED"),
    ("BIP-047", "EXPIRED_OR_INVALIDATED"),
    ("BIP-048", "NOT_COMPARABLE"),
    ("BIP-049", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-050", "REJECTED_FOR_DECLARED_PURPOSE"),
    ("BIP-051", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-052", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-053", "NOT_COMPARABLE"),
    ("BIP-054", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-055", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-056", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-057", "MORE_EVIDENCE_REQUIRED"),
    ("BIP-058", "NOT_COMPARABLE"),
    ("BIP-059", "EXPIRED_OR_INVALIDATED"),
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
    let parsed = UniqueJson::deserialize(&mut deserializer)
        .map_err(|error| format!("STRICT_JSON_REJECTED:{error}"))?;
    deserializer
        .end()
        .map_err(|error| format!("JSON_TRAILING_DATA:{error}"))?;
    Ok(parsed.0)
}

fn text<'a>(value: &'a Value, key: &str) -> &'a str {
    value.get(key).and_then(Value::as_str).unwrap_or("")
}

fn number(value: &Value, key: &str) -> Option<f64> {
    value.get(key).and_then(Value::as_f64).filter(|v| v.is_finite())
}

fn exact_keys(value: &Value, expected: &[&str]) -> bool {
    let Some(object) = value.as_object() else {
        return false;
    };
    object.len() == expected.len() && expected.iter().all(|key| object.contains_key(*key))
}

fn string_array_equals(value: &Value, expected: &[&str]) -> bool {
    value.as_array().is_some_and(|array| {
        array.len() == expected.len()
            && array
                .iter()
                .zip(expected)
                .all(|(actual, wanted)| actual.as_str() == Some(*wanted))
    })
}

fn non_empty_text(value: &Value, key: &str) -> bool {
    !text(value, key).trim().is_empty()
}

fn valid_iso_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 10 || bytes[4] != b'-' || bytes[7] != b'-' {
        return false;
    }
    if !bytes.iter().enumerate().all(|(index, byte)| {
        index == 4 || index == 7 || byte.is_ascii_digit()
    }) {
        return false;
    }
    let Ok(year) = value[0..4].parse::<u32>() else {
        return false;
    };
    let Ok(month) = value[5..7].parse::<u32>() else {
        return false;
    };
    let Ok(day) = value[8..10].parse::<u32>() else {
        return false;
    };
    if month == 0 || month > 12 || day == 0 {
        return false;
    }
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    day <= days_in_month
}


fn channel_by_id(id: &str) -> Option<&'static Channel> {
    CHANNELS.iter().find(|channel| channel.id == id)
}

pub fn classify_passport(record: &Value) -> &'static str {
    if text(record, "passport_schema_version") != PASSPORT_VERSION
        || text(record, "profile_id") != PROFILE_ID
    {
        return "NOT_COMPARABLE";
    }
    if text(record, "claim_requested") == "universal_indoor_air_quality" {
        return "REJECTED_CLAIM_PROMOTION";
    }
    if text(record, "claim_requested") != "declared_measurement"
        || text(&record["purpose_profile"], "classification") != "synthetic_measurement_purpose_only"
    {
        return "NOT_COMPARABLE";
    }

    let instrument = &record["instrument"];
    let capability = &record["capability"];
    let calibration = &record["calibration"];
    let installation = &record["installation"];
    let sample = &record["sample"];
    let uncertainty = &record["uncertainty_budget"];
    let purpose = &record["purpose_profile"];

    let Some(channel) = channel_by_id(text(instrument, "channel_id")) else {
        return "NOT_COMPARABLE";
    };

    if instrument["serial_identity_reused"].as_bool() == Some(true) {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if matches!(text(instrument, "lifecycle_state"), "retired" | "replaced" | "moved") {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(instrument, "lifecycle_state") != "installed" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(instrument, "identity_state") != "established" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    // v0.1 supports an absolute accuracy statement only. Percentage bases
    // require a separate, range-aware conversion model before applicability.
    if text(&capability["accuracy"], "basis") != "absolute" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if !non_empty_text(instrument, "protocol_profile_id")
        || !non_empty_text(instrument, "protocol_profile_digest")
        || !non_empty_text(instrument, "protocol_profile_verification_receipt_id")
        || !non_empty_text(instrument, "protocol_profile_verifier_id")
        || text(instrument, "protocol_profile_state") != "independently_verified"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if instrument["local_read_only_supported"].as_bool() != Some(true)
        || text(sample, "acquisition_mode") != "local"
        || sample["local_read_only"].as_bool() != Some(true)
    {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if !non_empty_text(sample, "read_only_session_receipt_id") {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(sample, "protocol_profile_digest") != text(instrument, "protocol_profile_digest") {
        return "NOT_COMPARABLE";
    }

    if text(instrument, "channel_profile_id") != PROFILE_ID
        || text(calibration, "channel_profile_id") != PROFILE_ID
        || text(sample, "channel_profile_id") != PROFILE_ID
    {
        return "NOT_COMPARABLE";
    }

    if text(instrument, "instrument_id") != text(calibration, "instrument_id")
        || text(instrument, "instrument_id") != text(sample, "instrument_id")
        || text(instrument, "channel_id") != text(calibration, "channel_id")
        || text(instrument, "channel_id") != text(sample, "channel_id")
        || text(instrument, "channel_id") != text(purpose, "channel_id")
        || text(instrument, "firmware_revision") != text(calibration, "firmware_revision")
        || text(instrument, "firmware_revision") != text(sample, "firmware_revision")
    {
        return "EXPIRED_OR_INVALIDATED";
    }

    let independent_assay = &record["independent_assay"];
    if channel.independent_assay {
        if text(independent_assay, "state") == "revoked"
            || text(independent_assay, "certificate_state") == "revoked"
            || text(independent_assay, "rights_state") == "revoked"
        {
            return "REJECTED_FOR_DECLARED_PURPOSE";
        }
        if text(independent_assay, "state") != "identified" {
            return "MORE_EVIDENCE_REQUIRED";
        }
        if text(independent_assay, "sample_id") != text(sample, "observation_id")
            || text(independent_assay, "sample_digest") != text(sample, "raw_source_digest")
        {
            return "NOT_COMPARABLE";
        }
        if text(independent_assay, "measurand_id") != channel.measurand
            || text(independent_assay, "unit") != channel.unit
        {
            return "NOT_COMPARABLE";
        }
        if text(independent_assay, "certificate_state") != "authenticated"
            || text(independent_assay, "accreditation_scope_state") != "covers_measurand_method"
            || text(independent_assay, "rights_state") != "authorized"
            || !non_empty_text(independent_assay, "assay_id")
            || !non_empty_text(independent_assay, "laboratory_id")
            || !non_empty_text(independent_assay, "accreditation_scope_id")
            || !non_empty_text(independent_assay, "method_id")
            || !non_empty_text(independent_assay, "method_revision")
            || !non_empty_text(independent_assay, "certificate_digest")
            || number(independent_assay, "value").is_none()
        {
            return "MORE_EVIDENCE_REQUIRED";
        }
    } else if text(independent_assay, "state") != "not_required" {
        return "NOT_COMPARABLE";
    }

    if text(calibration, "measurand_id") != channel.measurand
        || text(calibration, "unit") != channel.unit
        || text(purpose, "measurand_id") != channel.measurand
        || text(purpose, "unit") != channel.unit
        || text(capability, "measurement_kind") != channel.raw_or_derived
        || text(&capability["operating_range"], "unit") != channel.unit
        || text(&capability["resolution"], "unit") != channel.unit
        || text(&capability["accuracy"], "unit") != channel.unit
        || text(&capability["repeatability"], "unit") != channel.unit
        || text(&capability["drift"], "unit") != channel.unit
        || text(sample, "unit") != channel.unit
        || text(uncertainty, "unit") != channel.unit
    {
        return "NOT_COMPARABLE";
    }

    let Some(range_minimum) = number(&capability["operating_range"], "minimum") else {
        return "NOT_COMPARABLE";
    };
    let Some(range_maximum) = number(&capability["operating_range"], "maximum") else {
        return "NOT_COMPARABLE";
    };
    let Some(resolution) = number(&capability["resolution"], "value") else {
        return "NOT_COMPARABLE";
    };
    let Some(accuracy) = number(&capability["accuracy"], "value") else {
        return "NOT_COMPARABLE";
    };
    let Some(response_time) = number(capability, "response_time_seconds") else {
        return "NOT_COMPARABLE";
    };
    let Some(repeatability) = number(&capability["repeatability"], "value") else {
        return "NOT_COMPARABLE";
    };
    let Some(drift) = number(&capability["drift"], "value") else {
        return "NOT_COMPARABLE";
    };
    let Some(drift_period) = number(&capability["drift"], "period_seconds") else {
        return "NOT_COMPARABLE";
    };
    if range_minimum >= range_maximum || resolution <= 0.0 || accuracy < 0.0
        || response_time < 0.0 || repeatability < 0.0 || drift < 0.0 || drift_period <= 0.0
    {
        return "NOT_COMPARABLE";
    }

    if text(capability, "verification_state") != "independently_verified"
        || !matches!(
            text(&capability["accuracy"], "evidence_class"),
            "independent_type_test" | "independent_characterization"
        )
        || !non_empty_text(&capability["accuracy"], "source_record_id")
        || !matches!(
            text(&capability["repeatability"], "evidence_class"),
            "independent_type_test" | "independent_characterization"
        )
        || !non_empty_text(&capability["repeatability"], "method_id")
        || text(capability, "cross_sensitivity_state") != "characterized_or_not_applicable"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    if matches!(
        text(calibration, "certificate_state"),
        "expired" | "revoked"
    ) || matches!(
        text(calibration, "calibration_state"),
        "expired" | "out_of_tolerance"
    ) {
        return "EXPIRED_OR_INVALIDATED";
    }

    if text(calibration, "certificate_state") != "authenticated"
        || text(calibration, "calibration_state") != "current"
        || text(calibration, "accreditation_scope_state") != "covers_measurand_range_method"
        || text(calibration, "traceability_state") != "documented"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    if text(calibration, "as_found_state") == "out_of_tolerance"
        && text(calibration, "prior_data_impact_state") != "reviewed"
    {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(calibration, "as_left_state") == "out_of_tolerance" {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(calibration, "as_found_state") == "unknown"
        || text(calibration, "as_left_state") == "unknown"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    let calibration_date = text(calibration, "calibration_date");
    let calibration_due_date = text(calibration, "due_date");
    let sample_date = text(sample, "observed_at_date");
    if !valid_iso_date(calibration_date)
        || !valid_iso_date(calibration_due_date)
        || !valid_iso_date(sample_date)
        || calibration_date > calibration_due_date
    {
        return "NOT_COMPARABLE";
    }
    if sample_date < calibration_date || sample_date > calibration_due_date {
        return "EXPIRED_OR_INVALIDATED";
    }

    if !non_empty_text(calibration, "certificate_digest")
        || !non_empty_text(calibration, "method_id")
        || !non_empty_text(calibration, "method_revision")
        || calibration["reference_standard_ids"]
            .as_array()
            .is_none_or(|ids| ids.is_empty())
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    if !non_empty_text(&calibration["uncertainty"], "method_id")
        || !non_empty_text(&calibration["uncertainty"], "correlation_model_id")
        || text(&calibration["uncertainty"], "unit") != channel.unit
        || number(&calibration["uncertainty"], "value").is_none_or(|value| value < 0.0)
        || number(&calibration["uncertainty"], "coverage_factor").is_none_or(|factor| factor <= 0.0)
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    let Some(sample_value) = number(sample, "value") else {
        return "EXPIRED_OR_INVALIDATED";
    };
    let Some(calibration_minimum) = number(calibration, "range_minimum") else {
        return "MORE_EVIDENCE_REQUIRED";
    };
    let Some(calibration_maximum) = number(calibration, "range_maximum") else {
        return "MORE_EVIDENCE_REQUIRED";
    };
    if calibration_minimum >= calibration_maximum
        || calibration_minimum < range_minimum
        || calibration_maximum > range_maximum
    {
        return "NOT_COMPARABLE";
    }
    if calibration_minimum > sample_value
        || calibration_maximum < sample_value
        || sample_value < range_minimum
        || sample_value > range_maximum
    {
        return "EXPIRED_OR_INVALIDATED";
    }

    if matches!(text(installation, "installation_state"), "invalidated")
        || text(installation, "relocation_state") == "relocated_unverified"
    {
        return "EXPIRED_OR_INVALIDATED";
    }
    if text(installation, "installation_state") != "verified"
        || text(installation, "verifier_competence_state") != "documented"
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(installation, "geometry_revision") != text(purpose, "required_geometry_revision")
        || text(installation, "location_context_id") != text(purpose, "required_location_context_id")
    {
        return "NOT_COMPARABLE";
    }

    if text(sample, "qc_state") == "warming_up" {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if text(sample, "qc_state") != "valid" {
        return "EXPIRED_OR_INVALIDATED";
    }

    let Some(purpose_minimum) = number(purpose, "acceptable_minimum") else {
        return "NOT_COMPARABLE";
    };
    let Some(purpose_maximum) = number(purpose, "acceptable_maximum") else {
        return "NOT_COMPARABLE";
    };
    if sample_value < purpose_minimum || sample_value > purpose_maximum {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }

    if text(sample, "time_unit") != "s"
        || !non_empty_text(sample, "timezone")
        || !non_empty_text(sample, "clock_source")
    {
        return "NOT_COMPARABLE";
    }
    let Some(event_time) = number(sample, "event_time_seconds") else {
        return "NOT_COMPARABLE";
    };
    let Some(acquisition_time) = number(sample, "acquisition_time_seconds") else {
        return "NOT_COMPARABLE";
    };
    if acquisition_time < event_time {
        return "NOT_COMPARABLE";
    }
    let Some(sync_uncertainty) = number(sample, "synchronization_uncertainty_seconds") else {
        return "NOT_COMPARABLE";
    };
    let Some(max_sync_uncertainty) = number(purpose, "maximum_time_sync_uncertainty_seconds") else {
        return "NOT_COMPARABLE";
    };
    if sync_uncertainty > max_sync_uncertainty {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if sample["sequence"].as_u64().is_none_or(|sequence| sequence == 0) {
        return "EXPIRED_OR_INVALIDATED";
    }

    if text(sample, "processing_mode") != channel.raw_or_derived {
        return "NOT_COMPARABLE";
    }
    if channel.raw_or_derived == "derived"
        && (!non_empty_text(sample, "processing_method_id")
            || text(sample, "processing_method_id") == "NOT_APPLICABLE"
            || !non_empty_text(sample, "processing_method_revision")
            || text(sample, "processing_method_revision") == "NOT_APPLICABLE"
            || !non_empty_text(sample, "raw_source_digest"))
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    if !non_empty_text(sample, "raw_source_digest")
        || text(sample, "calibration_record_id") != text(calibration, "record_id")
        || text(sample, "installation_record_id") != text(installation, "record_id")
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    if text(sample, "rights_state") == "revoked" {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }
    if text(sample, "rights_state") != "authorized" {
        return "MORE_EVIDENCE_REQUIRED";
    }

    if text(uncertainty, "state") != "complete"
        || !non_empty_text(uncertainty, "combination_method_id")
        || !non_empty_text(uncertainty, "correlation_model_id")
        || number(uncertainty, "coverage_factor").is_none_or(|factor| factor <= 0.0)
        || number(uncertainty, "expanded_uncertainty_value").is_none_or(|value| value < 0.0)
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    let Some(components) = uncertainty["components"].as_array() else {
        return "MORE_EVIDENCE_REQUIRED";
    };
    if components.is_empty()
        || components.iter().any(|component| {
            !exact_keys(component, UNCERTAINTY_COMPONENT_FIELDS)
                || number(component, "value").is_none_or(|value| value < 0.0)
                || text(component, "unit") != channel.unit
                || !non_empty_text(component, "correlation_group")
                || !non_empty_text(component, "source_record_id")
        })
    {
        return "MORE_EVIDENCE_REQUIRED";
    }
    let calibration_component_count = components.iter().filter(|component| {
        text(component, "component_kind") == "calibration"
            && text(component, "source_record_id") == text(calibration, "record_id")
    }).count();
    let Some(calibration_uncertainty_value) = number(&calibration["uncertainty"], "value") else {
        return "MORE_EVIDENCE_REQUIRED";
    };
    if calibration_component_count != 1
        || components.iter().filter(|component| {
            text(component, "component_kind") == "calibration"
                && text(component, "source_record_id") == text(calibration, "record_id")
        }).any(|component| {
            text(component, "unit") != channel.unit
                || number(component, "value") != Some(calibration_uncertainty_value)
        })
    {
        return "MORE_EVIDENCE_REQUIRED";
    }

    let Some(maximum_uncertainty) = number(purpose, "maximum_expanded_uncertainty") else {
        return "NOT_COMPARABLE";
    };
    let Some(maximum_response_time) = number(purpose, "maximum_response_time_seconds") else {
        return "NOT_COMPARABLE";
    };
    let Some(expanded_uncertainty) = number(uncertainty, "expanded_uncertainty_value") else {
        return "MORE_EVIDENCE_REQUIRED";
    };
    if expanded_uncertainty > maximum_uncertainty || response_time > maximum_response_time {
        return "REJECTED_FOR_DECLARED_PURPOSE";
    }

    // Even a complete synthetic record only reaches review eligibility.
    // The purpose owner and independent instrument reviewer retain admission authority.
    let _independent_assay_required = channel.independent_assay;
    "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"
}

fn validate_passport_shape(record: &Value) -> Result<(), String> {
    if !exact_keys(record, ROOT_FIELDS) {
        return Err("PASSPORT_ROOT_FIELDS_INVALID".to_owned());
    }
    let instrument = &record["instrument"];
    let capability = &record["capability"];
    let calibration = &record["calibration"];
    let installation = &record["installation"];
    let sample = &record["sample"];
    let uncertainty = &record["uncertainty_budget"];
    let purpose = &record["purpose_profile"];

    if !exact_keys(instrument, INSTRUMENT_FIELDS)
        || !exact_keys(capability, CAPABILITY_FIELDS)
        || !exact_keys(calibration, CALIBRATION_FIELDS)
        || !exact_keys(installation, INSTALLATION_FIELDS)
        || !exact_keys(sample, SAMPLE_FIELDS)
        || !exact_keys(uncertainty, UNCERTAINTY_FIELDS)
        || !exact_keys(purpose, PURPOSE_FIELDS)
        || !exact_keys(&record["independent_assay"], INDEPENDENT_ASSAY_FIELDS)
        || !exact_keys(&capability["operating_range"], &["minimum", "maximum", "unit", "conditions_id"])
        || !exact_keys(&capability["resolution"], &["value", "unit"])
        || !exact_keys(
            &capability["accuracy"],
            &["value", "unit", "basis", "conditions_id", "evidence_class", "source_record_id"],
        )
        || !exact_keys(
            &capability["repeatability"],
            &["value", "unit", "method_id", "evidence_class"],
        )
        || !exact_keys(
            &capability["drift"],
            &["value", "unit", "period_seconds", "conditions_id"],
        )
        || !exact_keys(
            &calibration["uncertainty"],
            &["value", "unit", "coverage_factor", "method_id", "correlation_model_id"],
        )
        || !exact_keys(
            &installation["position"],
            &["x", "y", "z", "frame_id"],
        )
    {
        return Err("PASSPORT_NESTED_FIELDS_INVALID".to_owned());
    }

    let Some(components) = uncertainty["components"].as_array() else {
        return Err("UNCERTAINTY_COMPONENTS_INVALID".to_owned());
    };
    for component in components {
        if !exact_keys(component, UNCERTAINTY_COMPONENT_FIELDS) {
            return Err("UNCERTAINTY_COMPONENT_FIELDS_INVALID".to_owned());
        }
    }

    for (object, keys) in [
        (&instrument["instrument_id"], "instrument_id"),
        (&instrument["manufacturer"], "manufacturer"),
        (&instrument["model"], "model"),
        (&instrument["serial_id"], "serial_id"),
        (&instrument["hardware_revision"], "hardware_revision"),
        (&instrument["firmware_revision"], "firmware_revision"),
        (&instrument["protocol_profile_id"], "protocol_profile_id"),
        (&sample["observation_id"], "observation_id"),
    ] {
        if object.as_str().is_none_or(|value| value.trim().is_empty()) {
            return Err(format!("PASSPORT_REQUIRED_TEXT_EMPTY:{keys}"));
        }
    }
    Ok(())
}

fn schema_object_matches(schema: &Value, fields: &[&str]) -> bool {
    schema["type"] == "object"
        && schema["additionalProperties"] == false
        && string_array_equals(&schema["required"], fields)
        && schema["properties"].as_object().is_some_and(|properties| {
            properties.len() == fields.len()
                && fields.iter().all(|field| properties.contains_key(*field))
        })
}

fn json_schema_type_matches(instance: &Value, type_name: &str) -> bool {
    match type_name {
        "object" => instance.is_object(),
        "array" => instance.is_array(),
        "string" => instance.is_string(),
        "number" => instance.as_f64().is_some_and(f64::is_finite),
        "integer" => instance.as_i64().is_some()
            || instance.as_u64().is_some()
            || instance.as_f64().is_some_and(|value| value.is_finite() && value.fract() == 0.0),
        "boolean" => instance.is_boolean(),
        "null" => instance.is_null(),
        _ => false,
    }
}

fn resolve_local_schema_reference<'a>(schema_root: &'a Value, reference: &str) -> Option<&'a Value> {
    let mut current = schema_root;
    let pointer = reference.strip_prefix("#/")?;
    for encoded_key in pointer.split('/') {
        let key = encoded_key.replace("~1", "/").replace("~0", "~");
        current = current.get(key)?;
    }
    Some(current)
}

fn schema_pattern_matches(pattern: &str, value: &str) -> bool {
    // The checked-in schema currently uses only the BIP-NNN case identifier pattern.
    if pattern == "^BIP-[0-9]{3}$" {
        let bytes = value.as_bytes();
        return bytes.len() == 7
            && &bytes[0..4] == b"BIP-"
            && bytes[4..7].iter().all(|byte| byte.is_ascii_digit());
    }
    false
}

fn validate_json_schema(
    instance: &Value,
    schema: &Value,
    schema_root: &Value,
    path: &str,
    errors: &mut Vec<String>,
    depth: usize,
) {
    if depth > 128 {
        errors.push(format!("{path}:SCHEMA_RECURSION_LIMIT"));
        return;
    }

    if let Some(reference) = schema.get("$ref").and_then(Value::as_str) {
        let Some(target) = resolve_local_schema_reference(schema_root, reference) else {
            errors.push(format!("{path}:SCHEMA_REFERENCE_UNRESOLVED:{reference}"));
            return;
        };
        validate_json_schema(instance, target, schema_root, path, errors, depth + 1);
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
            json_schema_type_matches(instance, type_name)
        } else if let Some(type_names) = expected_type.as_array() {
            type_names.iter().any(|name| {
                name.as_str()
                    .is_some_and(|type_name| json_schema_type_matches(instance, type_name))
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
            if let Some(property_schema) = properties.and_then(|map| map.get(key)) {
                validate_json_schema(
                    value,
                    property_schema,
                    schema_root,
                    &format!("{path}.{key}"),
                    errors,
                    depth + 1,
                );
            } else if schema.get("additionalProperties") == Some(&Value::Bool(false)) {
                errors.push(format!("{path}:SCHEMA_ADDITIONAL_PROPERTY:{key}"));
            } else if let Some(additional_schema) = schema
                .get("additionalProperties")
                .filter(|value| value.is_object())
            {
                validate_json_schema(
                    value,
                    additional_schema,
                    schema_root,
                    &format!("{path}.{key}"),
                    errors,
                    depth + 1,
                );
            }
        }
    }

    if let Some(array) = instance.as_array() {
        if schema
            .get("minItems")
            .and_then(Value::as_u64)
            .is_some_and(|minimum| array.len() < minimum as usize)
        {
            errors.push(format!("{path}:SCHEMA_MIN_ITEMS"));
        }
        if schema
            .get("maxItems")
            .and_then(Value::as_u64)
            .is_some_and(|maximum| array.len() > maximum as usize)
        {
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
                    schema_root,
                    &format!("{path}[{index}]"),
                    errors,
                    depth + 1,
                );
            }
        }
    }

    if let Some(value) = instance.as_str() {
        if schema
            .get("minLength")
            .and_then(Value::as_u64)
            .is_some_and(|minimum| value.chars().count() < minimum as usize)
        {
            errors.push(format!("{path}:SCHEMA_MIN_LENGTH"));
        }
        if schema
            .get("maxLength")
            .and_then(Value::as_u64)
            .is_some_and(|maximum| value.chars().count() > maximum as usize)
        {
            errors.push(format!("{path}:SCHEMA_MAX_LENGTH"));
        }
        if let Some(pattern) = schema.get("pattern").and_then(Value::as_str) {
            if !schema_pattern_matches(pattern, value) {
                errors.push(format!("{path}:SCHEMA_PATTERN_MISMATCH_OR_UNSUPPORTED"));
            }
        }
        // Draft 2020-12 treats format as annotation unless format-assertion is enabled.
        // Calendar-date semantics are checked by the purpose classifier below.
    }

    if let Some(value) = instance.as_f64().filter(|number| number.is_finite()) {
        if schema
            .get("minimum")
            .and_then(Value::as_f64)
            .is_some_and(|minimum| value < minimum)
        {
            errors.push(format!("{path}:SCHEMA_MINIMUM"));
        }
        if schema
            .get("maximum")
            .and_then(Value::as_f64)
            .is_some_and(|maximum| value > maximum)
        {
            errors.push(format!("{path}:SCHEMA_MAXIMUM"));
        }
    }
}

fn validate_schema_contracts(passport_schema: &Value, corpus_schema: &Value) -> Result<(), String> {
    if text(passport_schema, "title") != "Mycelix Building Instrument Passport v0.1"
        || passport_schema["type"] != "object"
        || passport_schema["additionalProperties"] != false
        || !string_array_equals(&passport_schema["required"], ROOT_FIELDS)
    {
        return Err("PASSPORT_SCHEMA_ROOT_INVALID".to_owned());
    }
    for (field, definition) in [
        ("instrument", "instrument"),
        ("capability", "capability"),
        ("calibration", "calibration"),
        ("installation", "installation"),
        ("sample", "sample"),
        ("uncertainty_budget", "uncertainty_budget"),
        ("purpose_profile", "purpose_profile"),
    ] {
        let expected_reference = format!("#/$defs/{definition}");
        if passport_schema["properties"][field]["$ref"].as_str()
            != Some(expected_reference.as_str())
        {
            return Err("PASSPORT_SCHEMA_REFERENCE_INVALID".to_owned());
        }
    }
    let required_definitions: &[(&str, &[&str])] = &[
        ("instrument", INSTRUMENT_FIELDS),
        ("capability", CAPABILITY_FIELDS),
        ("calibration", CALIBRATION_FIELDS),
        ("installation", INSTALLATION_FIELDS),
        ("sample", SAMPLE_FIELDS),
        ("uncertainty_budget", UNCERTAINTY_FIELDS),
        ("purpose_profile", PURPOSE_FIELDS),
        ("independent_assay", INDEPENDENT_ASSAY_FIELDS),
    ];
    for (name, fields) in required_definitions {
        if !schema_object_matches(&passport_schema["$defs"][*name], fields) {
            return Err(format!("PASSPORT_SCHEMA_OBJECT_DRIFT:{name}"));
        }
    }
    let nested_objects: &[(&str, &Value, &[&str])] = &[
        ("operating_range", &passport_schema["$defs"]["capability"]["properties"]["operating_range"], OPERATING_RANGE_FIELDS),
        ("resolution", &passport_schema["$defs"]["capability"]["properties"]["resolution"], RESOLUTION_FIELDS),
        ("accuracy", &passport_schema["$defs"]["capability"]["properties"]["accuracy"], ACCURACY_FIELDS),
        ("repeatability", &passport_schema["$defs"]["capability"]["properties"]["repeatability"], REPEATABILITY_FIELDS),
        ("drift", &passport_schema["$defs"]["capability"]["properties"]["drift"], DRIFT_FIELDS),
        ("position", &passport_schema["$defs"]["installation"]["properties"]["position"], POSITION_FIELDS),
        ("calibration_uncertainty", &passport_schema["$defs"]["calibration"]["properties"]["uncertainty"], CALIBRATION_UNCERTAINTY_FIELDS),
        ("uncertainty_component", &passport_schema["$defs"]["uncertainty_budget"]["properties"]["components"]["items"], UNCERTAINTY_COMPONENT_FIELDS),
    ];
    for (name, schema, fields) in nested_objects {
        if !schema_object_matches(schema, fields) {
            return Err(format!("PASSPORT_SCHEMA_NESTED_OBJECT_DRIFT:{name}"));
        }
    }
    if passport_schema["properties"]["independent_assay"]["$ref"] != "#/$defs/independent_assay"
        || passport_schema["$defs"].get("independent_assay").is_none()
    {
        return Err("INDEPENDENT_ASSAY_SCHEMA_INVALID".to_owned());
    }
    if passport_schema["$defs"]["sample"]["properties"]["observed_at_date"]["format"] != "date" {
        return Err("SAMPLE_DATE_SCHEMA_INVALID".to_owned());
    }
    if text(corpus_schema, "title") != "Mycelix Building Instrument Passport Corpus v0.1"
        || corpus_schema["type"] != "object"
        || corpus_schema["additionalProperties"] != false
        || !string_array_equals(
            &corpus_schema["required"],
            &["schema_version", "corpus_id", "classification", "profile_id", "claim_ceiling", "cases"],
        )
        || corpus_schema["properties"]["cases"]["minItems"] != EXPECTED_CASE_COUNT
        || corpus_schema["properties"]["cases"]["maxItems"] != EXPECTED_CASE_COUNT
        || corpus_schema["properties"]["cases"]["items"]["properties"]["record"]["$ref"]
            != "#/$defs/passport"
        || !schema_object_matches(&corpus_schema["$defs"]["passport"], ROOT_FIELDS)
        || corpus_schema["$defs"]["passport"].get("$id").is_some()
        || corpus_schema["$defs"]["passport"].get("$schema").is_some()
        || corpus_schema["$defs"]["passport"]["properties"]["independent_assay"]["$ref"] != "#/$defs/independent_assay"
        || corpus_schema["$defs"]["passport"]["properties"]["instrument"]["$ref"] != "#/$defs/instrument"
        || corpus_schema["$defs"]["passport"]["properties"]["capability"]["$ref"] != "#/$defs/capability"
        || corpus_schema["$defs"]["passport"]["properties"]["calibration"]["$ref"] != "#/$defs/calibration"
        || corpus_schema["$defs"]["passport"]["properties"]["installation"]["$ref"] != "#/$defs/installation"
        || corpus_schema["$defs"]["passport"]["properties"]["sample"]["$ref"] != "#/$defs/sample"
        || corpus_schema["$defs"]["passport"]["properties"]["uncertainty_budget"]["$ref"] != "#/$defs/uncertainty_budget"
        || corpus_schema["$defs"]["passport"]["properties"]["purpose_profile"]["$ref"] != "#/$defs/purpose_profile"
        || corpus_schema["$defs"]["sample"]["properties"]["observed_at_date"].get("format").is_some()
        || !schema_object_matches(&corpus_schema["$defs"]["instrument"], INSTRUMENT_FIELDS)
        || !schema_object_matches(&corpus_schema["$defs"]["sample"], SAMPLE_FIELDS)
    {
        return Err("PASSPORT_CORPUS_SCHEMA_INVALID".to_owned());
    }
    for definition in [
        "passport",
        "instrument",
        "capability",
        "calibration",
        "installation",
        "sample",
        "uncertainty_budget",
        "purpose_profile",
        "independent_assay",
    ] {
        if corpus_schema["$defs"].get(definition).is_none() {
            return Err(format!("PASSPORT_CORPUS_SCHEMA_DEF_MISSING:{definition}"));
        }
    }
    Ok(())
}

pub fn validate_corpus(corpus: &Value) -> Result<usize, String> {
    if !exact_keys(
        corpus,
        &["schema_version", "corpus_id", "classification", "profile_id", "claim_ceiling", "cases"],
    ) || text(corpus, "schema_version") != "mycelix.building-instrument-passport-corpus/0.1"
        || text(corpus, "corpus_id") != CORPUS_ID
        || text(corpus, "classification") != "synthetic_passport_applicability_cases"
        || text(corpus, "profile_id") != PROFILE_ID
        || text(corpus, "claim_ceiling") != "synthetic_instrument_passport_applicability_only"
    {
        return Err("PASSPORT_CORPUS_IDENTITY_INVALID".to_owned());
    }

    let cases = corpus["cases"]
        .as_array()
        .ok_or_else(|| "PASSPORT_CASES_MISSING".to_owned())?;
    if cases.len() != EXPECTED_CASE_COUNT {
        return Err("PASSPORT_CASE_COUNT_INVALID".to_owned());
    }

    for (index, (case, (expected_id, expected_disposition))) in
        cases.iter().zip(EXPECTED_CASES).enumerate()
    {
        if !exact_keys(case, CASE_FIELDS)
            || text(case, "case_id") != *expected_id
            || text(case, "description").len() < 20
            || text(case, "expected_disposition") != *expected_disposition
        {
            return Err(format!("PASSPORT_CASE_IDENTITY_OR_EXPECTED_DRIFT:{expected_id}"));
        }
        validate_passport_shape(&case["record"])?;
        let actual = classify_passport(&case["record"]);
        if actual != *expected_disposition {
            return Err(format!(
                "PASSPORT_CASE_DISPOSITION_MISMATCH:{}:expected={expected_disposition}:actual={actual}",
                index + 1
            ));
        }
    }
    Ok(cases.len())
}

pub fn validate_tracked() -> Result<usize, String> {
    let passport_schema = parse_unique_json(PASSPORT_SCHEMA_JSON)?;
    let corpus_schema = parse_unique_json(CORPUS_SCHEMA_JSON)?;
    validate_schema_contracts(&passport_schema, &corpus_schema)?;
    let corpus = parse_unique_json(CORPUS_JSON)?;

    let mut schema_errors = Vec::new();
    validate_json_schema(
        &corpus,
        &corpus_schema,
        &corpus_schema,
        "$corpus",
        &mut schema_errors,
        0,
    );
    if let Some(cases) = corpus.get("cases").and_then(Value::as_array) {
        for (index, case) in cases.iter().enumerate() {
            if let Some(record) = case.get("record") {
                let case_id = text(case, "case_id");
                validate_json_schema(
                    record,
                    &passport_schema,
                    &passport_schema,
                    &format!("$corpus.cases[{index}:{case_id}].record"),
                    &mut schema_errors,
                    0,
                );
            }
        }
    }
    if !schema_errors.is_empty() {
        schema_errors.sort();
        return Err(format!(
            "PASSPORT_JSON_SCHEMA_VALIDATION_FAILED:{}",
            schema_errors.join("|")
        ));
    }

    validate_corpus(&corpus)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn case(id: &str) -> Value {
        let corpus = parse_unique_json(CORPUS_JSON).expect("tracked corpus is valid JSON");
        corpus["cases"]
            .as_array()
            .expect("cases array")
            .iter()
            .find(|case| text(case, "case_id") == id)
            .expect("known case")
            .clone()
    }

    #[test]
    fn json_schema_validation_accepts_the_full_tracked_corpus_and_passports() {
        assert_eq!(validate_tracked().expect("all 55 schemas and cases"), 55);
    }

    #[test]
    fn json_schema_validation_rejects_unknown_nested_properties() {
        let passport_schema = parse_unique_json(PASSPORT_SCHEMA_JSON).expect("schema");
        let mut record = case("BIP-001")["record"].clone();
        record["sample"]["unreviewed_value"] = Value::from(42);
        let mut errors = Vec::new();
        validate_json_schema(
            &record,
            &passport_schema,
            &passport_schema,
            "$record",
            &mut errors,
            0,
        );
        assert!(errors.iter().any(|error| error.contains("SCHEMA_ADDITIONAL_PROPERTY")));
    }

    #[test]
    fn json_schema_validation_accepts_explicit_null_missing_protocol_evidence() {
        let passport_schema = parse_unique_json(PASSPORT_SCHEMA_JSON).expect("schema");
        for case_id in ["BIP-052", "BIP-054"] {
            let record = case(case_id)["record"].clone();
            let mut errors = Vec::new();
            validate_json_schema(
                &record,
                &passport_schema,
                &passport_schema,
                "$record",
                &mut errors,
                0,
            );
            assert!(errors.is_empty(), "{case_id}: {errors:?}");
        }
    }

    #[test]
    fn tracked_passport_corpus_matches_all_55_known_answers() {
        assert_eq!(validate_tracked().expect("tracked passport validation"), 55);
    }

    #[test]
    fn protocol_profile_digest_and_local_only_fields_are_schema_pinned() {
        let passport_schema = parse_unique_json(PASSPORT_SCHEMA_JSON).expect("passport schema");
        let instrument = &passport_schema["$defs"]["instrument"];
        let sample = &passport_schema["$defs"]["sample"];
        assert!(instrument["required"].as_array().is_some_and(|fields| {
            fields.iter().any(|field| field == "protocol_profile_digest")
                && fields.iter().any(|field| field == "protocol_profile_state")
                && fields.iter().any(|field| field == "local_read_only_supported")
        }));
        assert!(sample["required"].as_array().is_some_and(|fields| {
            fields.iter().any(|field| field == "acquisition_mode")
                && fields.iter().any(|field| field == "local_read_only")
        }));
        assert_eq!(validate_tracked().expect("55-case passport corpus"), 55);
    }

    #[test]
    fn duplicate_json_object_members_are_rejected_at_nested_depth() {
        assert!(parse_unique_json(r#"{"outer":{"unit":"m","unit":"ft"}}"#).is_err());
    }

    #[test]
    fn unknown_passport_fields_are_rejected() {
        let mut record = case("BIP-001")["record"].clone();
        record["instrument"]["unreviewed_vendor_score"] = Value::from(99);
        assert!(validate_passport_shape(&record).is_err());
    }

    #[test]
    fn remote_acquisition_is_rejected_for_local_read_only_channels() {
        assert_eq!(
            classify_passport(&case("BIP-049")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
    }

    #[test]
    fn write_capable_protocol_is_rejected_for_local_read_only_channels() {
        assert_eq!(
            classify_passport(&case("BIP-050")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
    }

    #[test]
    fn provider_asserted_protocol_profile_is_not_independent_evidence() {
        assert_eq!(
            classify_passport(&case("BIP-051")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn protocol_verification_requires_named_verifier_and_receipt() {
        assert_eq!(
            classify_passport(&case("BIP-052")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn sample_must_bind_exact_protocol_digest() {
        assert_eq!(
            classify_passport(&case("BIP-053")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn local_read_only_session_requires_receipt() {
        assert_eq!(
            classify_passport(&case("BIP-054")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn unsupported_percentage_accuracy_basis_requires_more_evidence() {
        assert_eq!(
            classify_passport(&case("BIP-055")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn provider_assertion_is_not_independent_capability_evidence() {
        assert_eq!(
            classify_passport(&case("BIP-003")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn calibration_scope_must_cover_the_exact_method_and_range() {
        assert_eq!(
            classify_passport(&case("BIP-009")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn certificate_measurand_mismatch_is_not_comparable() {
        assert_eq!(
            classify_passport(&case("BIP-010")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn firmware_change_invalidates_the_sample_binding() {
        assert_eq!(
            classify_passport(&case("BIP-017")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
    }

    #[test]
    fn sensor_relocation_requires_reverification() {
        assert_eq!(
            classify_passport(&case("BIP-014")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
    }

    #[test]
    fn reused_serial_identity_is_rejected() {
        assert_eq!(
            classify_passport(&case("BIP-016")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
    }

    #[test]
    fn acquisition_time_cannot_precede_event_time() {
        assert_eq!(
            classify_passport(&case("BIP-039")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn derived_observations_need_method_and_source_lineage() {
        assert_eq!(
            classify_passport(&case("BIP-028")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
        assert_eq!(
            classify_passport(&case("BIP-034")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn uncertainty_components_preserve_correlation_groups() {
        assert_eq!(
            classify_passport(&case("BIP-040")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn stale_and_faulted_measurements_are_never_numeric_passes() {
        assert_eq!(
            classify_passport(&case("BIP-022")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
        assert_eq!(
            classify_passport(&case("BIP-023")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
    }

    #[test]
    fn missing_data_cannot_be_encoded_as_zero() {
        assert_eq!(
            classify_passport(&case("BIP-025")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
    }

    #[test]
    fn rights_revocation_blocks_declared_purpose_use() {
        assert_eq!(
            classify_passport(&case("BIP-035")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
    }

    #[test]
    fn co2_sensor_does_not_prove_universal_indoor_air_quality() {
        assert_eq!(
            classify_passport(&case("BIP-033")["record"]),
            "REJECTED_CLAIM_PROMOTION"
        );
    }

    #[test]
    fn proposed_v02_channel_is_not_admitted_by_v01_profile() {
        assert_eq!(
            classify_passport(&case("BIP-032")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn purpose_specific_response_and_uncertainty_limits_are_separate() {
        assert_eq!(
            classify_passport(&case("BIP-030")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
        assert_eq!(
            classify_passport(&case("BIP-042")["record"]),
            "REJECTED_FOR_DECLARED_PURPOSE"
        );
    }

    #[test]
    fn water_quality_channel_requires_independent_assay() {
        assert_eq!(
            classify_passport(&case("BIP-043")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
        assert_eq!(
            classify_passport(&case("BIP-044")["record"]),
            "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"
        );
    }

    #[test]
    fn independent_assay_must_match_the_exact_physical_sample() {
        assert_eq!(
            classify_passport(&case("BIP-045")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn calibration_date_is_checked_against_the_sample_observation_date() {
        assert_eq!(
            classify_passport(&case("BIP-046")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
        assert_eq!(
            classify_passport(&case("BIP-047")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
        assert_eq!(
            classify_passport(&case("BIP-048")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn calibration_uncertainty_must_be_present_in_the_exact_sample_budget() {
        assert_eq!(
            classify_passport(&case("BIP-056")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
        assert_eq!(
            classify_passport(&case("BIP-057")["record"]),
            "MORE_EVIDENCE_REQUIRED"
        );
    }

    #[test]
    fn calibration_range_must_fit_the_instrument_capability_envelope() {
        assert_eq!(
            classify_passport(&case("BIP-058")["record"]),
            "NOT_COMPARABLE"
        );
    }

    #[test]
    fn retired_instrument_cannot_support_a_current_sample() {
        assert_eq!(
            classify_passport(&case("BIP-059")["record"]),
            "EXPIRED_OR_INVALIDATED"
        );
    }

    #[test]
    fn embedded_passport_schema_cannot_shadow_the_corpus_reference_scope() {
        let passport_schema = parse_unique_json(PASSPORT_SCHEMA_JSON).expect("passport schema");
        let mut corpus_schema = parse_unique_json(CORPUS_SCHEMA_JSON).expect("corpus schema");
        corpus_schema["$defs"]["passport"]["$id"] =
            Value::String("https://example.invalid/passport.json".to_owned());
        assert!(validate_schema_contracts(&passport_schema, &corpus_schema).is_err());
    }

    #[test]
    fn even_a_complete_record_is_review_eligible_not_admitted() {
        assert_eq!(
            classify_passport(&case("BIP-036")["record"]),
            "ELIGIBLE_FOR_INDEPENDENT_ADMISSION_REVIEW_ONLY"
        );
    }
}
