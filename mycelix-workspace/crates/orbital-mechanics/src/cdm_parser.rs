// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Legacy project-prefixed CDM key/value parser.
//!
//! This parser handles Mycelix's historical flattened representation, where object
//! fields use `OBJECT1_*` and `OBJECT2_*` prefixes. It is **not** a standards-native
//! CCSDS 508.0-B-1 KVN parser: CCSDS KVN uses `OBJECT = OBJECT1` / `OBJECT = OBJECT2`
//! blocks and unprefixed object fields. Standards-native parsing is tracked in issue #6369.
//!
//! The legacy input is line-oriented `KEY = VALUE` data with `COMMENT` lines.

use chrono::{DateTime, NaiveDateTime, Utc};

use crate::cdm::{
    CdmCovariance, CdmObjectData, CdmObjectMetadata, CdmRefFrame, CdmStateVector,
    ConjunctionDataMessage, Maneuverable, PcMethod,
};

/// Error type for CDM parsing
#[derive(Debug, thiserror::Error)]
pub enum CdmParseError {
    #[error("Missing required field: {0}")]
    MissingField(String),
    #[error("Invalid value for {field}: {value}")]
    InvalidValue { field: String, value: String },
    #[error("Date parse error for {field}: {value}")]
    DateParseError { field: String, value: String },
    #[error("Unsupported CDM version: {0}")]
    UnsupportedVersion(String),
    #[error("Empty input")]
    EmptyInput,
    #[error("Incomplete covariance for {object}: CR_R present but missing elements: {missing}")]
    IncompleteCovariance { object: String, missing: String },
    #[error("CREATION_DATE ({creation}) is after TCA ({tca}) — CDM is malformed")]
    CreationAfterTca { creation: String, tca: String },
    #[error("Object1 and Object2 must use the same REF_FRAME; got {object1} and {object2}")]
    MismatchedReferenceFrames { object1: String, object2: String },
    #[error("Unsupported REF_FRAME value for {field}: {value}")]
    UnsupportedReferenceFrame { field: String, value: String },
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CdmParseReport {
    /// Optional source keywords that were absent and projected to legacy placeholders.
    pub absent_optional_fields: Vec<String>,
}

impl CdmParseReport {
    pub fn has_projection_loss(&self) -> bool {
        !self.absent_optional_fields.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct ParsedCdmKvn {
    pub message: ConjunctionDataMessage,
    pub report: CdmParseReport,
}

fn project_optional_number(
    value: Option<f64>,
    field: &str,
    report: &mut CdmParseReport,
) -> f64 {
    match value {
        Some(value) => value,
        None => {
            report.absent_optional_fields.push(field.to_string());
            0.0
        }
    }
}

fn parse_finite_f64(field: &str, raw: &str) -> Result<f64, CdmParseError> {
    let value = strip_units(raw).to_string();
    let parsed = value.parse::<f64>().map_err(|_| CdmParseError::InvalidValue {
        field: field.to_string(),
        value: value.clone(),
    })?;
    if !parsed.is_finite() {
        return Err(CdmParseError::InvalidValue { field: field.to_string(), value });
    }
    Ok(parsed)
}

/// Parse the legacy Mycelix prefixed CDM representation.
///
/// This compatibility API discards the parse report. New consumers that need to
/// distinguish absent optional values from numeric zero must use
/// `parse_cdm_kvn_with_report`. Neither API parses standards-native CCSDS KVN.
pub fn parse_cdm_kvn(input: &str) -> Result<ConjunctionDataMessage, CdmParseError> {
    parse_cdm_kvn_with_report(input).map(|parsed| parsed.message)
}

/// Parse the legacy prefixed representation and report absent optional fields.
/// The embedded version marker is not proof of CCSDS KVN conformance. This API
/// does not parse standards-native KVN object blocks, XML, JSON, or TraCSS CDM v2.1.
pub fn parse_cdm_kvn_with_report(input: &str) -> Result<ParsedCdmKvn, CdmParseError> {
    if input.trim().is_empty() {
        return Err(CdmParseError::EmptyInput);
    }

    let lines: Vec<(&str, &str)> = input
        .lines()
        .filter_map(|line| {
            let line = line.trim();
            if line.is_empty() || line.starts_with("COMMENT") {
                return None;
            }
            let mut parts = line.splitn(2, '=');
            let key = parts.next()?.trim();
            let value = parts.next()?.trim();
            Some((key, value))
        })
        .collect();

    let get = |key: &str| -> Result<String, CdmParseError> {
        lines
            .iter()
            .find(|(k, _)| *k == key)
            .map(|(_, v)| strip_units(v).to_string())
            .ok_or_else(|| CdmParseError::MissingField(key.to_string()))
    };

    let get_f64 = |key: &str| -> Result<f64, CdmParseError> {
        let raw = lines
            .iter()
            .find(|(found, _)| *found == key)
            .map(|(_, value)| *value)
            .ok_or_else(|| CdmParseError::MissingField(key.to_string()))?;
        parse_finite_f64(key, raw)
    };

    let get_opt_f64 = |key: &str| -> Result<Option<f64>, CdmParseError> {
        match lines.iter().find(|(k, _)| *k == key) {
            Some((_, raw)) => parse_finite_f64(key, raw).map(Some),
            None => Ok(None),
        }
    };

    let mut report = CdmParseReport::default();

    // --- Header ---
    let version = get("CCSDS_CDM_VERS")?;
    if version != "1.0" {
        return Err(CdmParseError::UnsupportedVersion(version));
    }

    let creation_date = parse_cdm_datetime(&get("CREATION_DATE")?, "CREATION_DATE")?;
    let originator = get("ORIGINATOR")?;
    let message_id = get("MESSAGE_ID")?;

    // --- Relative Metadata ---
    let tca = parse_cdm_datetime(&get("TCA")?, "TCA")?;
    let miss_distance = get_f64("MISS_DISTANCE")?;
    let relative_speed = project_optional_number(
        get_opt_f64("RELATIVE_SPEED")?,
        "RELATIVE_SPEED",
        &mut report,
    );

    let relative_position_r = project_optional_number(
        get_opt_f64("RELATIVE_POSITION_R")?,
        "RELATIVE_POSITION_R",
        &mut report,
    );
    let relative_position_t = project_optional_number(
        get_opt_f64("RELATIVE_POSITION_T")?,
        "RELATIVE_POSITION_T",
        &mut report,
    );
    let relative_position_n = project_optional_number(
        get_opt_f64("RELATIVE_POSITION_N")?,
        "RELATIVE_POSITION_N",
        &mut report,
    );
    let relative_velocity_r = project_optional_number(
        get_opt_f64("RELATIVE_VELOCITY_R")?,
        "RELATIVE_VELOCITY_R",
        &mut report,
    );
    let relative_velocity_t = project_optional_number(
        get_opt_f64("RELATIVE_VELOCITY_T")?,
        "RELATIVE_VELOCITY_T",
        &mut report,
    );
    let relative_velocity_n = project_optional_number(
        get_opt_f64("RELATIVE_VELOCITY_N")?,
        "RELATIVE_VELOCITY_N",
        &mut report,
    );

    // --- Freshness validation: CREATION_DATE must be <= TCA ---
    if creation_date > tca {
        return Err(CdmParseError::CreationAfterTca {
            creation: creation_date.to_rfc3339(),
            tca: tca.to_rfc3339(),
        });
    }

    // --- Probability ---
    let collision_probability = project_optional_number(
        get_opt_f64("COLLISION_PROBABILITY")?,
        "COLLISION_PROBABILITY",
        &mut report,
    );
    if !(0.0..=1.0).contains(&collision_probability) {
        return Err(CdmParseError::InvalidValue {
            field: "COLLISION_PROBABILITY".to_string(),
            value: collision_probability.to_string(),
        });
    }
    let collision_probability_method = match lines
        .iter()
        .find(|(key, _)| *key == "COLLISION_PROBABILITY_METHOD")
    {
        Some((_, value)) => PcMethod::from_str_permissive(strip_units(value)),
        None => {
            report
                .absent_optional_fields
                .push("COLLISION_PROBABILITY_METHOD".to_string());
            PcMethod::Other("UNSPECIFIED".to_string())
        }
    };

    // --- Object Data ---
    let object1 = parse_object_block(&lines, "OBJECT1", &mut report)?;
    let object2 = parse_object_block(&lines, "OBJECT2", &mut report)?;
    if object1.metadata.ref_frame != object2.metadata.ref_frame {
        return Err(CdmParseError::MismatchedReferenceFrames {
            object1: object1.metadata.ref_frame.to_string(),
            object2: object2.metadata.ref_frame.to_string(),
        });
    }

    // --- Optional fields ---
    let hard_body_radius = get_opt_f64("HARD_BODY_RADIUS")?;

    Ok(ParsedCdmKvn {
        message: ConjunctionDataMessage {
            ccsds_cdm_vers: version,
            creation_date,
            originator,
            message_id,
            tca,
            miss_distance,
            relative_speed,
            relative_position_r,
            relative_position_t,
            relative_position_n,
            relative_velocity_r,
            relative_velocity_t,
            relative_velocity_n,
            collision_probability,
            collision_probability_method,
            object1,
            object2,
            screening_entry_time: None,
            screening_data_source: None,
            hard_body_radius,
        },
        report,
    })
}

/// Parse an object data block (metadata + state vector + optional covariance).
fn parse_object_block(
    lines: &[(&str, &str)],
    prefix: &str,
    report: &mut CdmParseReport,
) -> Result<CdmObjectData, CdmParseError> {
    let get = |suffix: &str| -> Result<String, CdmParseError> {
        let key = format!("{}_{}", prefix, suffix);
        lines
            .iter()
            .find(|(k, _)| *k == key)
            .map(|(_, v)| strip_units(v).to_string())
            .ok_or(CdmParseError::MissingField(key))
    };

    let get_f64 = |suffix: &str| -> Result<f64, CdmParseError> {
        let key = format!("{}_{}", prefix, suffix);
        let raw = lines
            .iter()
            .find(|(found, _)| *found == key)
            .map(|(_, value)| *value)
            .ok_or_else(|| CdmParseError::MissingField(key.clone()))?;
        parse_finite_f64(&key, raw)
    };

    let get_opt_f64 = |suffix: &str| -> Result<Option<f64>, CdmParseError> {
        let key = format!("{}_{}", prefix, suffix);
        match lines.iter().find(|(k, _)| *k == key) {
            Some((_, raw)) => parse_finite_f64(&key, raw).map(Some),
            None => Ok(None),
        }
    };

    // Required metadata fields are not silently defaulted. An absent international
    // designator must be represented by the explicit source value "UNKNOWN".
    let object_type_key = format!("{}_OBJECT_TYPE", prefix);
    let object_type = match lines.iter().find(|(key, _)| *key == object_type_key) {
        Some((_, value)) => strip_units(value).to_string(),
        None => {
            report.absent_optional_fields.push(object_type_key);
            "UNKNOWN".to_string()
        }
    };
    let maneuverable_key = format!("{}_MANEUVERABLE", prefix);
    let maneuverable = parse_maneuverable(&get("MANEUVERABLE")?, &maneuverable_key)?;

    let metadata = CdmObjectMetadata {
        object_designator: get("OBJECT_DESIGNATOR")?,
        catalog_name: get("CATALOG_NAME")?,
        object_name: get("OBJECT_NAME")?,
        international_designator: get("INTERNATIONAL_DESIGNATOR")?,
        object_type,
        operator_organization: get("OPERATOR_ORGANIZATION").ok(),
        operator_phone: get("OPERATOR_PHONE").ok(),
        operator_email: get("OPERATOR_EMAIL").ok(),
        ephemeris_name: get("EPHEMERIS_NAME")?,
        covariance_method: get("COVARIANCE_METHOD")?,
        maneuverable,
        ref_frame: parse_ref_frame(
            &get("REF_FRAME")?,
            &format!("{}_REF_FRAME", prefix),
        )?,
    };

    // State vector
    let state_vector = CdmStateVector {
        x: get_f64("X")?,
        y: get_f64("Y")?,
        z: get_f64("Z")?,
        x_dot: get_f64("X_DOT")?,
        y_dot: get_f64("Y_DOT")?,
        z_dot: get_f64("Z_DOT")?,
    };

    // If any covariance term is present, all 21 terms are required. Malformed values
    // are rejected rather than being interpreted as an absent covariance.
    const COV_ELEMENTS: &[&str] = &[
        "CR_R",
        "CT_R",
        "CT_T",
        "CN_R",
        "CN_T",
        "CN_N",
        "CRDOT_R",
        "CRDOT_T",
        "CRDOT_N",
        "CRDOT_RDOT",
        "CTDOT_R",
        "CTDOT_T",
        "CTDOT_N",
        "CTDOT_RDOT",
        "CTDOT_TDOT",
        "CNDOT_R",
        "CNDOT_T",
        "CNDOT_N",
        "CNDOT_RDOT",
        "CNDOT_TDOT",
        "CNDOT_NDOT",
    ];
    let has_any_covariance = COV_ELEMENTS.iter().any(|suffix| {
        let key = format!("{}_{}", prefix, suffix);
        lines.iter().any(|(found, _)| *found == key)
    });
    let covariance = if has_any_covariance {
        let mut missing = Vec::new();
        for elem in COV_ELEMENTS {
            if get_opt_f64(elem)?.is_none() {
                missing.push(*elem);
            }
        }

        if !missing.is_empty() {
            return Err(CdmParseError::IncompleteCovariance {
                object: prefix.to_string(),
                missing: missing.join(", "),
            });
        }

        Some(CdmCovariance {
            cr_r: get_f64("CR_R")?,
            ct_r: get_f64("CT_R")?,
            ct_t: get_f64("CT_T")?,
            cn_r: get_f64("CN_R")?,
            cn_t: get_f64("CN_T")?,
            cn_n: get_f64("CN_N")?,
            crdot_r: get_f64("CRDOT_R")?,
            crdot_t: get_f64("CRDOT_T")?,
            crdot_n: get_f64("CRDOT_N")?,
            crdot_rdot: get_f64("CRDOT_RDOT")?,
            ctdot_r: get_f64("CTDOT_R")?,
            ctdot_t: get_f64("CTDOT_T")?,
            ctdot_n: get_f64("CTDOT_N")?,
            ctdot_rdot: get_f64("CTDOT_RDOT")?,
            ctdot_tdot: get_f64("CTDOT_TDOT")?,
            cndot_r: get_f64("CNDOT_R")?,
            cndot_t: get_f64("CNDOT_T")?,
            cndot_n: get_f64("CNDOT_N")?,
            cndot_rdot: get_f64("CNDOT_RDOT")?,
            cndot_tdot: get_f64("CNDOT_TDOT")?,
            cndot_ndot: get_f64("CNDOT_NDOT")?,
        })
    } else {
        None
    };

    Ok(CdmObjectData {
        metadata,
        state_vector,
        covariance,
    })
}

/// Parse a CDM datetime string.
/// Supports ISO 8601 format: `2024-01-15T12:00:00.000`
fn parse_cdm_datetime(s: &str, field: &str) -> Result<DateTime<Utc>, CdmParseError> {
    // Try full ISO 8601 with fractional seconds
    if let Ok(ndt) = NaiveDateTime::parse_from_str(s, "%Y-%m-%dT%H:%M:%S%.f") {
        return Ok(ndt.and_utc());
    }
    // Try without fractional seconds
    if let Ok(ndt) = NaiveDateTime::parse_from_str(s, "%Y-%m-%dT%H:%M:%S") {
        return Ok(ndt.and_utc());
    }
    // Try DOY format: YYYY-DDDTHH:MM:SS.SSS
    if s.len() >= 17 && s.chars().nth(4) == Some('-') && s.chars().nth(8) == Some('T') {
        if let (Ok(year), Ok(doy)) = (s[..4].parse::<i32>(), s[5..8].parse::<u32>()) {
            let time_part = &s[9..];
            let base = chrono::NaiveDate::from_yo_opt(year, doy).and_then(|d| {
                chrono::NaiveTime::parse_from_str(time_part, "%H:%M:%S%.f")
                    .ok()
                    .or_else(|| chrono::NaiveTime::parse_from_str(time_part, "%H:%M:%S").ok())
                    .map(|t| d.and_time(t))
            });
            if let Some(ndt) = base {
                return Ok(ndt.and_utc());
            }
        }
    }
    Err(CdmParseError::DateParseError {
        field: field.to_string(),
        value: s.to_string(),
    })
}

/// Strip unit annotations from KVN values, e.g. `"1.234 [km]"` → `"1.234"`
fn strip_units(value: &str) -> &str {
    if let Some(pos) = value.find('[') {
        value[..pos].trim()
    } else {
        value.trim()
    }
}

fn parse_maneuverable(s: &str, field: &str) -> Result<Maneuverable, CdmParseError> {
    match s.to_uppercase().as_str() {
        "YES" => Ok(Maneuverable::Yes),
        "NO" => Ok(Maneuverable::No),
        "N/A" => Ok(Maneuverable::Unknown),
        _ => Err(CdmParseError::InvalidValue {
            field: field.to_string(),
            value: s.to_string(),
        }),
    }
}

/// CCSDS 508.0-B-1 CDM v1.0 permits GCRF, EME2000 and ITRF.
fn parse_ref_frame(s: &str, field: &str) -> Result<CdmRefFrame, CdmParseError> {
    match s.to_uppercase().as_str() {
        "EME2000" => Ok(CdmRefFrame::EME2000),
        "GCRF" => Ok(CdmRefFrame::GCRF),
        "ITRF" => Ok(CdmRefFrame::ITRF),
        _ => Err(CdmParseError::UnsupportedReferenceFrame {
            field: field.to_string(),
            value: s.to_string(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_CDM: &str = r#"CCSDS_CDM_VERS = 1.0
CREATION_DATE = 2024-01-15T12:00:00.000
ORIGINATOR = MYCELIX-SPACE
MESSAGE_ID = CDM-25544-99999-20240115T180000

TCA = 2024-01-15T18:00:00.000
MISS_DISTANCE = 0.500000 [km]
RELATIVE_SPEED = 14500.000000 [m/s]
RELATIVE_POSITION_R = 200.000000 [m]
RELATIVE_POSITION_T = 300.000000 [m]
RELATIVE_POSITION_N = 100.000000 [m]
RELATIVE_VELOCITY_R = 5000.000000 [m/s]
RELATIVE_VELOCITY_T = 12000.000000 [m/s]
RELATIVE_VELOCITY_N = 3000.000000 [m/s]

COLLISION_PROBABILITY = 1.000000e-05
COLLISION_PROBABILITY_METHOD = ALFANO-2005

COMMENT Object 1 (Primary)
OBJECT1_OBJECT_DESIGNATOR = 25544
OBJECT1_CATALOG_NAME = SATCAT
OBJECT1_OBJECT_NAME = ISS (ZARYA)
OBJECT1_INTERNATIONAL_DESIGNATOR = 1998-067A
OBJECT1_OBJECT_TYPE = PAYLOAD
OBJECT1_EPHEMERIS_NAME = MYCELIX-SPACE
OBJECT1_COVARIANCE_METHOD = CALCULATED
OBJECT1_MANEUVERABLE = YES
OBJECT1_REF_FRAME = EME2000
OBJECT1_X = 6800.000000 [km]
OBJECT1_Y = 100.000000 [km]
OBJECT1_Z = 50.000000 [km]
OBJECT1_X_DOT = 0.100000000 [km/s]
OBJECT1_Y_DOT = 7.660000000 [km/s]
OBJECT1_Z_DOT = 0.050000000 [km/s]

COMMENT Object 2 (Secondary)
OBJECT2_OBJECT_DESIGNATOR = 99999
OBJECT2_CATALOG_NAME = SATCAT
OBJECT2_OBJECT_NAME = COSMOS 1408 DEB
OBJECT2_INTERNATIONAL_DESIGNATOR = 2021-101A
OBJECT2_OBJECT_TYPE = DEBRIS
OBJECT2_EPHEMERIS_NAME = MYCELIX-SPACE
OBJECT2_COVARIANCE_METHOD = CALCULATED
OBJECT2_MANEUVERABLE = NO
OBJECT2_REF_FRAME = EME2000
OBJECT2_X = 6800.500000 [km]
OBJECT2_Y = 100.000000 [km]
OBJECT2_Z = 50.000000 [km]
OBJECT2_X_DOT = 0.100000000 [km/s]
OBJECT2_Y_DOT = 7.660000000 [km/s]
OBJECT2_Z_DOT = 0.050000000 [km/s]
"#;

    #[test]
    fn test_parse_reference_cdm() {
        let cdm = parse_cdm_kvn(SAMPLE_CDM).unwrap();

        assert_eq!(cdm.ccsds_cdm_vers, "1.0");
        assert_eq!(cdm.originator, "MYCELIX-SPACE");
        assert_eq!(cdm.message_id, "CDM-25544-99999-20240115T180000");
        assert!((cdm.miss_distance - 0.5).abs() < 0.001);
        assert!((cdm.relative_speed - 14500.0).abs() < 0.1);
        assert!((cdm.collision_probability - 1e-5).abs() < 1e-10);
        assert_eq!(cdm.collision_probability_method, PcMethod::Alfano2005);

        // Object 1
        assert_eq!(cdm.object1.metadata.object_designator, "25544");
        assert_eq!(cdm.object1.metadata.object_name, "ISS (ZARYA)");
        assert_eq!(cdm.object1.metadata.maneuverable, Maneuverable::Yes);
        assert_eq!(cdm.object1.metadata.ref_frame, CdmRefFrame::EME2000);
        assert!((cdm.object1.state_vector.x - 6800.0).abs() < 0.001);

        // Object 2
        assert_eq!(cdm.object2.metadata.object_designator, "99999");
        assert_eq!(cdm.object2.metadata.object_name, "COSMOS 1408 DEB");
        assert_eq!(cdm.object2.metadata.maneuverable, Maneuverable::No);
    }

    #[test]
    fn test_roundtrip_generate_parse() {
        // Generate KVN from code, then parse it back
        let cdm_original = parse_cdm_kvn(SAMPLE_CDM).unwrap();
        let kvn_output = cdm_original.to_kvn();
        let cdm_roundtrip = parse_cdm_kvn(&kvn_output).unwrap();

        assert_eq!(cdm_original.originator, cdm_roundtrip.originator);
        assert!((cdm_original.miss_distance - cdm_roundtrip.miss_distance).abs() < 0.001);
        assert!(
            (cdm_original.collision_probability - cdm_roundtrip.collision_probability).abs() < 1e-8
        );
        assert_eq!(
            cdm_original.object1.metadata.object_name,
            cdm_roundtrip.object1.metadata.object_name
        );
        assert_eq!(
            cdm_original.object2.metadata.object_name,
            cdm_roundtrip.object2.metadata.object_name
        );
    }

    #[test]
    fn test_missing_required_field() {
        let incomplete = "CCSDS_CDM_VERS = 1.0\nORIGINATOR = TEST\n";
        let err = parse_cdm_kvn(incomplete);
        assert!(err.is_err());
        match err.unwrap_err() {
            CdmParseError::MissingField(field) => {
                assert_eq!(field, "CREATION_DATE");
            }
            other => panic!("Expected MissingField, got {:?}", other),
        }
    }

    #[test]
    fn test_empty_input() {
        let err = parse_cdm_kvn("").unwrap_err();
        assert!(matches!(err, CdmParseError::EmptyInput));
    }

    #[test]
    fn test_unsupported_version() {
        let bad_version = "CCSDS_CDM_VERS = 2.0\nCREATION_DATE = 2024-01-01T00:00:00\n";
        let err = parse_cdm_kvn(bad_version).unwrap_err();
        assert!(matches!(err, CdmParseError::UnsupportedVersion(_)));
    }

    #[test]
    fn test_strip_units() {
        assert_eq!(strip_units("1.234 [km]"), "1.234");
        assert_eq!(strip_units("1.234e-05 [km**2]"), "1.234e-05");
        assert_eq!(strip_units("ALFANO-2005"), "ALFANO-2005");
        assert_eq!(strip_units("  42.0  "), "42.0");
    }

    #[test]
    fn test_parse_datetime_iso() {
        let dt = parse_cdm_datetime("2024-01-15T18:30:45.123", "test").unwrap();
        assert_eq!(dt.year(), 2024);
        assert_eq!(dt.month(), 1);
        assert_eq!(dt.day(), 15);
        assert_eq!(dt.hour(), 18);
    }

    #[test]
    fn test_parse_datetime_doy() {
        // Day-of-year format: 2024-015T12:00:00.000 (Jan 15)
        let dt = parse_cdm_datetime("2024-015T12:00:00.000", "test").unwrap();
        assert_eq!(dt.year(), 2024);
        assert_eq!(dt.month(), 1);
        assert_eq!(dt.day(), 15);
    }

    use chrono::{Datelike, Timelike};

    /// Helper: generate all 21 covariance KVN lines for a given object prefix.
    fn full_covariance_kvn(prefix: &str) -> String {
        format!(
            r#"{pfx}_CR_R = 1.0e-04 [km**2]
{pfx}_CT_R = 2.0e-05 [km**2]
{pfx}_CT_T = 5.0e-04 [km**2]
{pfx}_CN_R = 1.0e-06 [km**2]
{pfx}_CN_T = 2.0e-06 [km**2]
{pfx}_CN_N = 3.0e-04 [km**2]
{pfx}_CRDOT_R = 1.0e-07 [km**2/s]
{pfx}_CRDOT_T = 2.0e-07 [km**2/s]
{pfx}_CRDOT_N = 3.0e-07 [km**2/s]
{pfx}_CRDOT_RDOT = 4.0e-08 [(km/s)**2]
{pfx}_CTDOT_R = 5.0e-07 [km**2/s]
{pfx}_CTDOT_T = 6.0e-07 [km**2/s]
{pfx}_CTDOT_N = 7.0e-07 [km**2/s]
{pfx}_CTDOT_RDOT = 8.0e-08 [(km/s)**2]
{pfx}_CTDOT_TDOT = 9.0e-08 [(km/s)**2]
{pfx}_CNDOT_R = 1.0e-07 [km**2/s]
{pfx}_CNDOT_T = 1.1e-07 [km**2/s]
{pfx}_CNDOT_N = 1.2e-07 [km**2/s]
{pfx}_CNDOT_RDOT = 1.3e-08 [(km/s)**2]
{pfx}_CNDOT_TDOT = 1.4e-08 [(km/s)**2]
{pfx}_CNDOT_NDOT = 1.5e-08 [(km/s)**2]
"#,
            pfx = prefix
        )
    }

    #[test]
    fn test_covariance_parsing_complete() {
        let cdm_with_cov = format!(
            r#"CCSDS_CDM_VERS = 1.0
CREATION_DATE = 2024-01-15T12:00:00.000
ORIGINATOR = TEST
MESSAGE_ID = TEST-001

TCA = 2024-01-15T18:00:00.000
MISS_DISTANCE = 1.0 [km]
RELATIVE_SPEED = 10000.0 [m/s]

COLLISION_PROBABILITY = 1.0e-06
COLLISION_PROBABILITY_METHOD = ALFANO-2005

OBJECT1_OBJECT_DESIGNATOR = 11111
OBJECT1_CATALOG_NAME = SATCAT
OBJECT1_OBJECT_NAME = SAT-A
OBJECT1_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT1_EPHEMERIS_NAME = NONE
OBJECT1_COVARIANCE_METHOD = CALCULATED
OBJECT1_REF_FRAME = EME2000
OBJECT1_MANEUVERABLE = YES
OBJECT1_X = 7000.0 [km]
OBJECT1_Y = 0.0 [km]
OBJECT1_Z = 0.0 [km]
OBJECT1_X_DOT = 0.0 [km/s]
OBJECT1_Y_DOT = 7.5 [km/s]
OBJECT1_Z_DOT = 0.0 [km/s]
{cov1}
OBJECT2_OBJECT_DESIGNATOR = 22222
OBJECT2_CATALOG_NAME = SATCAT
OBJECT2_OBJECT_NAME = DEBRIS-B
OBJECT2_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT2_EPHEMERIS_NAME = NONE
OBJECT2_COVARIANCE_METHOD = CALCULATED
OBJECT2_REF_FRAME = EME2000
OBJECT2_MANEUVERABLE = NO
OBJECT2_X = 7000.5 [km]
OBJECT2_Y = 0.0 [km]
OBJECT2_Z = 0.0 [km]
OBJECT2_X_DOT = 0.0 [km/s]
OBJECT2_Y_DOT = -7.5 [km/s]
OBJECT2_Z_DOT = 0.0 [km/s]
"#,
            cov1 = full_covariance_kvn("OBJECT1"),
        );

        let cdm = parse_cdm_kvn(&cdm_with_cov).unwrap();

        // Object 1 should have full covariance
        let cov1 = cdm.object1.covariance.as_ref().unwrap();
        assert!((cov1.cr_r - 1.0e-4).abs() < 1e-10);
        assert!((cov1.ct_r - 2.0e-5).abs() < 1e-10);
        assert!((cov1.ct_t - 5.0e-4).abs() < 1e-10);
        assert!((cov1.cndot_ndot - 1.5e-8).abs() < 1e-14);

        // Object 2 should NOT have covariance (no CR_R)
        assert!(cdm.object2.covariance.is_none());
    }

    #[test]
    fn test_incomplete_covariance_rejected() {
        // Only 3 of 21 covariance elements — must be rejected
        let cdm_partial_cov = r#"CCSDS_CDM_VERS = 1.0
CREATION_DATE = 2024-01-15T12:00:00.000
ORIGINATOR = TEST
MESSAGE_ID = TEST-001

TCA = 2024-01-15T18:00:00.000
MISS_DISTANCE = 1.0 [km]
RELATIVE_SPEED = 10000.0 [m/s]

COLLISION_PROBABILITY = 1.0e-06
COLLISION_PROBABILITY_METHOD = ALFANO-2005

OBJECT1_OBJECT_DESIGNATOR = 11111
OBJECT1_CATALOG_NAME = SATCAT
OBJECT1_OBJECT_NAME = SAT-A
OBJECT1_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT1_EPHEMERIS_NAME = NONE
OBJECT1_COVARIANCE_METHOD = CALCULATED
OBJECT1_REF_FRAME = EME2000
OBJECT1_MANEUVERABLE = YES
OBJECT1_X = 7000.0 [km]
OBJECT1_Y = 0.0 [km]
OBJECT1_Z = 0.0 [km]
OBJECT1_X_DOT = 0.0 [km/s]
OBJECT1_Y_DOT = 7.5 [km/s]
OBJECT1_Z_DOT = 0.0 [km/s]
OBJECT1_CR_R = 1.0e-04 [km**2]
OBJECT1_CT_R = 2.0e-05 [km**2]
OBJECT1_CT_T = 5.0e-04 [km**2]

OBJECT2_OBJECT_DESIGNATOR = 22222
OBJECT2_CATALOG_NAME = SATCAT
OBJECT2_OBJECT_NAME = DEBRIS-B
OBJECT2_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT2_EPHEMERIS_NAME = NONE
OBJECT2_COVARIANCE_METHOD = CALCULATED
OBJECT2_REF_FRAME = EME2000
OBJECT2_MANEUVERABLE = NO
OBJECT2_X = 7000.5 [km]
OBJECT2_Y = 0.0 [km]
OBJECT2_Z = 0.0 [km]
OBJECT2_X_DOT = 0.0 [km/s]
OBJECT2_Y_DOT = -7.5 [km/s]
OBJECT2_Z_DOT = 0.0 [km/s]
"#;

        let err = parse_cdm_kvn(cdm_partial_cov).unwrap_err();
        match err {
            CdmParseError::IncompleteCovariance { object, missing } => {
                assert_eq!(object, "OBJECT1");
                // Should list the 18 missing elements
                assert!(missing.contains("CN_R"));
                assert!(missing.contains("CNDOT_NDOT"));
                assert!(!missing.contains("CR_R")); // CR_R is present
                assert!(!missing.contains("CT_R")); // CT_R is present
                assert!(!missing.contains("CT_T")); // CT_T is present
            }
            other => panic!("Expected IncompleteCovariance, got {:?}", other),
        }
    }

    #[test]
    fn test_creation_after_tca_rejected() {
        // CREATION_DATE is 2024-01-16 but TCA is 2024-01-15 — malformed
        let stale_cdm = r#"CCSDS_CDM_VERS = 1.0
CREATION_DATE = 2024-01-16T12:00:00.000
ORIGINATOR = TEST
MESSAGE_ID = TEST-001

TCA = 2024-01-15T18:00:00.000
MISS_DISTANCE = 1.0 [km]
RELATIVE_SPEED = 10000.0 [m/s]

COLLISION_PROBABILITY = 1.0e-06
COLLISION_PROBABILITY_METHOD = FOSTER-1992

OBJECT1_OBJECT_DESIGNATOR = 11111
OBJECT1_CATALOG_NAME = SATCAT
OBJECT1_OBJECT_NAME = SAT-A
OBJECT1_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT1_EPHEMERIS_NAME = NONE
OBJECT1_COVARIANCE_METHOD = CALCULATED
OBJECT1_REF_FRAME = EME2000
OBJECT1_MANEUVERABLE = YES
OBJECT1_X = 7000.0 [km]
OBJECT1_Y = 0.0 [km]
OBJECT1_Z = 0.0 [km]
OBJECT1_X_DOT = 0.0 [km/s]
OBJECT1_Y_DOT = 7.5 [km/s]
OBJECT1_Z_DOT = 0.0 [km/s]

OBJECT2_OBJECT_DESIGNATOR = 22222
OBJECT2_CATALOG_NAME = SATCAT
OBJECT2_OBJECT_NAME = DEBRIS-B
OBJECT2_INTERNATIONAL_DESIGNATOR = UNKNOWN
OBJECT2_EPHEMERIS_NAME = NONE
OBJECT2_COVARIANCE_METHOD = CALCULATED
OBJECT2_REF_FRAME = EME2000
OBJECT2_MANEUVERABLE = NO
OBJECT2_X = 7000.5 [km]
OBJECT2_Y = 0.0 [km]
OBJECT2_Z = 0.0 [km]
OBJECT2_X_DOT = 0.0 [km/s]
OBJECT2_Y_DOT = -7.5 [km/s]
OBJECT2_Z_DOT = 0.0 [km/s]
"#;

        let err = parse_cdm_kvn(stale_cdm).unwrap_err();
        match err {
            CdmParseError::CreationAfterTca { .. } => {} // expected
            other => panic!("Expected CreationAfterTca, got {:?}", other),
        }
    }

    #[test]
    fn optional_fields_are_reported_instead_of_inferred_from_zero() {
        let minimal = SAMPLE_CDM
            .lines()
            .filter(|line| !line.starts_with("RELATIVE_SPEED = "))
            .filter(|line| !line.starts_with("RELATIVE_POSITION_"))
            .filter(|line| !line.starts_with("RELATIVE_VELOCITY_"))
            .filter(|line| !line.starts_with("COLLISION_PROBABILITY = "))
            .filter(|line| !line.starts_with("COLLISION_PROBABILITY_METHOD = "))
            .collect::<Vec<_>>()
            .join("\n");
        let parsed = parse_cdm_kvn_with_report(&minimal).unwrap();
        assert_eq!(parsed.message.relative_speed, 0.0);
        assert!(parsed.report.has_projection_loss());
        for key in [
            "RELATIVE_SPEED",
            "RELATIVE_POSITION_R",
            "RELATIVE_POSITION_T",
            "RELATIVE_POSITION_N",
            "RELATIVE_VELOCITY_R",
            "RELATIVE_VELOCITY_T",
            "RELATIVE_VELOCITY_N",
            "COLLISION_PROBABILITY",
            "COLLISION_PROBABILITY_METHOD",
        ] {
            assert!(
                parsed
                    .report
                    .absent_optional_fields
                    .iter()
                    .any(|field| field == key),
                "missing {key}"
            );
        }
    }

    #[test]
    fn required_reference_frame_is_not_defaulted() {
        let missing = SAMPLE_CDM.replace("OBJECT1_REF_FRAME = EME2000\n", "");
        assert!(matches!(
            parse_cdm_kvn(&missing),
            Err(CdmParseError::MissingField(field)) if field == "OBJECT1_REF_FRAME"
        ));
    }

    #[test]
    fn unsupported_reference_frame_is_not_coerced() {
        let unsupported =
            SAMPLE_CDM.replace("OBJECT1_REF_FRAME = EME2000", "OBJECT1_REF_FRAME = TEME");
        assert!(matches!(
            parse_cdm_kvn(&unsupported),
            Err(CdmParseError::UnsupportedReferenceFrame { field, value })
                if field == "OBJECT1_REF_FRAME" && value == "TEME"
        ));
    }

    #[test]
    fn mismatched_object_reference_frames_are_rejected() {
        let mismatched =
            SAMPLE_CDM.replace("OBJECT2_REF_FRAME = EME2000", "OBJECT2_REF_FRAME = GCRF");
        assert!(matches!(
            parse_cdm_kvn(&mismatched),
            Err(CdmParseError::MismatchedReferenceFrames { .. })
        ));
    }

    #[test]
    fn malformed_optional_numeric_is_not_treated_as_absent() {
        let malformed = SAMPLE_CDM.replace(
            "RELATIVE_SPEED = 14500.000000 [m/s]",
            "RELATIVE_SPEED = not-a-number [m/s]"
        );
        assert!(matches!(
            parse_cdm_kvn_with_report(&malformed),
            Err(CdmParseError::InvalidValue { field, .. }) if field == "RELATIVE_SPEED"
        ));
    }

    #[test]
    fn missing_required_ephemeris_metadata_is_rejected() {
        let missing = SAMPLE_CDM.replace("OBJECT1_EPHEMERIS_NAME = MYCELIX-SPACE\n", "");
        assert!(matches!(
            parse_cdm_kvn(&missing),
            Err(CdmParseError::MissingField(field)) if field == "OBJECT1_EPHEMERIS_NAME"
        ));
    }

    #[test]
    fn non_finite_state_vector_value_is_rejected() {
        let invalid = SAMPLE_CDM.replace(
            "OBJECT1_X = 6800.000000 [km]",
            "OBJECT1_X = NaN [km]",
        );
        assert!(matches!(
            parse_cdm_kvn(&invalid),
            Err(CdmParseError::InvalidValue { field, .. }) if field == "OBJECT1_X"
        ));
    }

    #[test]
    fn invalid_maneuverable_value_is_rejected() {
        let invalid = SAMPLE_CDM.replace(
            "OBJECT1_MANEUVERABLE = YES",
            "OBJECT1_MANEUVERABLE = MAYBE",
        );
        assert!(matches!(
            parse_cdm_kvn(&invalid),
            Err(CdmParseError::InvalidValue { field, .. }) if field == "OBJECT1_MANEUVERABLE"
        ));
    }

    #[test]
    fn malformed_first_covariance_term_is_not_treated_as_absent_covariance() {
        let bad_cov = format!(
            "{}\n{}",
            SAMPLE_CDM,
            full_covariance_kvn("OBJECT1").replacen("1.0e-04", "NaN", 1)
        );
        assert!(matches!(
            parse_cdm_kvn(&bad_cov),
            Err(CdmParseError::InvalidValue { field, .. }) if field == "OBJECT1_CR_R"
        ));
    }

    #[test]
    fn test_pc_method_known_variants() {
        assert_eq!(
            PcMethod::from_str_permissive("ALFANO-2005"),
            PcMethod::Alfano2005
        );
        assert_eq!(
            PcMethod::from_str_permissive("FOSTER-1992"),
            PcMethod::Foster1992
        );
        assert_eq!(
            PcMethod::from_str_permissive("PATERA-2001"),
            PcMethod::Patera2001
        );
        assert_eq!(
            PcMethod::from_str_permissive("MCKINLEY-2006"),
            PcMethod::McKinley2006
        );
        assert_eq!(
            PcMethod::from_str_permissive("CHAN-2008"),
            PcMethod::Chan2008
        );
        assert_eq!(
            PcMethod::from_str_permissive("MONTE-CARLO"),
            PcMethod::MonteCarlo
        );
        assert_eq!(
            PcMethod::from_str_permissive("MonteCarlo"),
            PcMethod::MonteCarlo
        );
    }

    #[test]
    fn test_pc_method_unknown_becomes_other() {
        let m = PcMethod::from_str_permissive("CUSTOM-METHOD-2026");
        assert_eq!(m, PcMethod::Other("CUSTOM-METHOD-2026".to_string()));
        assert_eq!(m.to_string(), "CUSTOM-METHOD-2026");
    }

    #[test]
    fn test_pc_method_parsed_from_cdm() {
        let cdm = parse_cdm_kvn(SAMPLE_CDM).unwrap();
        assert_eq!(cdm.collision_probability_method, PcMethod::Alfano2005);
    }
}
