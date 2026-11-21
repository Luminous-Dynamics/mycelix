//! Audit the Terra Preta field greenhouse-gas observation fixture.
//!
//! This is a structural and semantic record audit only. It does not integrate
//! fluxes through a season, estimate treatment effects, or qualify field data.
//! The v0.1 CLI accepts explicitly synthetic fixture envelopes so that the
//! first executable increment cannot be mistaken for empirical qualification.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const INTERPRETATION_BOUNDARY: &str =
    "Synthetic observation-record validation only; not flux integration, treatment-effect estimation, instrument qualification, field validation, product certification, regulatory approval, or carbon-credit verification.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureEnvelope {
    fixture_status: String,
    fixture_version: String,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    schema_version: String,
    observation_id: String,
    evidence_class: String,
    study_id: String,
    site_id: String,
    block_id: String,
    experimental_unit_id: String,
    plot_or_vessel_id: String,
    treatment_assignment_id: String,
    material_batch_id: Option<String>,
    soil_source_id: String,
    crop_id: Option<String>,
    management_event: Option<String>,
    observed_at: String,
    interval_start: String,
    interval_end: String,
    gas_species: String,
    mass_basis: String,
    flux_value: f64,
    flux_unit: String,
    sign_convention: String,
    method: Method,
    instrument: Instrument,
    raw_artifact: RawArtifact,
    uncertainty: Uncertainty,
    detection_limit: Option<DetectionLimit>,
    qc_state: String,
    #[serde(default)]
    qc_reasons: Vec<String>,
    missingness_reason: Option<String>,
    source_record_uri: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Method {
    method_id: String,
    method_version: String,
    chamber_type: String,
    chamber_area_m2: f64,
    chamber_volume_m3: f64,
    closure_duration_s: f64,
    flux_calculation_version: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Instrument {
    instrument_id: String,
    calibration_state: String,
    calibration_reference: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawArtifact {
    sha256: String,
    artifact_kind: String,
    source_uri: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Uncertainty {
    status: String,
    standard_uncertainty: Option<f64>,
    unit: Option<String>,
    method: Option<String>,
    reason_not_available: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DetectionLimit {
    value: f64,
    unit: String,
    basis: String,
    censoring_method: Option<String>,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    fixture_status: &'static str,
    fixture_version: String,
    input_sha256: String,
    observation_count: usize,
    gas_species_counts: BTreeMap<String, usize>,
    evidence_class_counts: BTreeMap<String, usize>,
    qc_state_counts: BTreeMap<String, usize>,
    observations: Vec<ObservationSummary>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct ObservationSummary {
    observation_id: String,
    gas_species: String,
    mass_basis: String,
    flux_value: f64,
    flux_unit: String,
    evidence_class: String,
    experimental_unit_id: String,
    qc_state: String,
    uncertainty_status: String,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn nonblank(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.trim() != value {
        return Err(format!("{field} must be non-empty with no surrounding whitespace"));
    }
    Ok(())
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn require_keys(value: &serde_json::Value, keys: &[&str], field: &str) -> Result<(), String> {
    let object = value
        .as_object()
        .ok_or_else(|| format!("{field} must be a JSON object"))?;
    for key in keys {
        if !object.contains_key(*key) {
            return Err(format!("{field} is missing required key {key}"));
        }
    }
    Ok(())
}

fn finite_positive(value: f64, field: &str) -> Result<(), String> {
    if !value.is_finite() || value <= 0.0 {
        return Err(format!("{field} must be finite and greater than zero"));
    }
    Ok(())
}

fn finite_nonnegative(value: f64, field: &str) -> Result<(), String> {
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{field} must be finite and non-negative"));
    }
    Ok(())
}

fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    // Howard Hinnant's civil-calendar transform; output is days from 1970-01-01.
    let year = year - i64::from(month <= 2);
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month = i64::from(month);
    let day = i64::from(day);
    let day_of_year = (153 * (month + if month > 2 { -3 } else { 9 }) + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

fn parse_rfc3339_seconds(value: &str) -> Result<f64, String> {
    let bytes = value.as_bytes();
    if bytes.len() < 20
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || !value.is_ascii()
    {
        return Err(format!("invalid RFC3339 timestamp: {value}"));
    }

    let parse = |start: usize, end: usize, field: &str| -> Result<u32, String> {
        value[start..end]
            .parse::<u32>()
            .map_err(|_| format!("invalid {field} in timestamp {value}"))
    };
    let year = parse(0, 4, "year")?;
    let month = parse(5, 7, "month")?;
    let day = parse(8, 10, "day")?;
    let hour = parse(11, 13, "hour")?;
    let minute = parse(14, 16, "minute")?;
    let second = parse(17, 19, "second")?;

    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return Err(format!("invalid month in timestamp {value}")),
    };
    if day == 0 || day > days_in_month || hour > 23 || minute > 59 || second > 59 {
        return Err(format!("out-of-range date/time in timestamp {value}"));
    }

    let mut position = 19;
    let mut fraction = 0.0;
    if bytes.get(position) == Some(&b'.') {
        position += 1;
        let start = position;
        while bytes.get(position).is_some_and(|byte| byte.is_ascii_digit()) {
            position += 1;
        }
        if position == start {
            return Err(format!("empty fractional seconds in timestamp {value}"));
        }
        let digits = &value[start..position];
        fraction = format!("0.{digits}")
            .parse::<f64>()
            .map_err(|_| format!("invalid fractional seconds in timestamp {value}"))?;
    }

    let offset_seconds: i64 = match bytes.get(position).copied() {
        Some(b'Z') if position + 1 == bytes.len() => 0,
        Some(sign @ (b'+' | b'-')) if position + 6 == bytes.len() => {
            if bytes[position + 3] != b':' {
                return Err(format!("invalid timezone offset in timestamp {value}"));
            }
            let offset_hour = value[position + 1..position + 3]
                .parse::<i64>()
                .map_err(|_| format!("invalid timezone hour in timestamp {value}"))?;
            let offset_minute = value[position + 4..position + 6]
                .parse::<i64>()
                .map_err(|_| format!("invalid timezone minute in timestamp {value}"))?;
            if offset_hour > 23 || offset_minute > 59 {
                return Err(format!("out-of-range timezone offset in timestamp {value}"));
            }
            let magnitude = offset_hour * 3_600 + offset_minute * 60;
            if sign == b'+' { magnitude } else { -magnitude }
        }
        _ => return Err(format!("timestamp must end with Z or ±HH:MM: {value}")),
    };

    let local_seconds = days_from_civil(i64::from(year), month, day) * 86_400
        + i64::from(hour) * 3_600
        + i64::from(minute) * 60
        + i64::from(second);
    Ok((local_seconds - offset_seconds) as f64 + fraction)
}

const N2O_N_MASS_FRACTION: f64 = 28.0 / 44.0;

/// Convert between N2O mass and nitrogen mass contained in N2O.
///
/// This is a stoichiometric basis conversion only. It does not change the
/// observation's source value, method, or uncertainty provenance.
fn convert_n2o_mass_basis(value: f64, from: &str, to: &str) -> Result<f64, String> {
    if !value.is_finite() {
        return Err("N2O mass-basis conversion requires a finite value".to_string());
    }
    match (from, to) {
        ("N2O", "N2O") | ("N2O_N", "N2O_N") => Ok(value),
        ("N2O", "N2O_N") => Ok(value * N2O_N_MASS_FRACTION),
        ("N2O_N", "N2O") => Ok(value / N2O_N_MASS_FRACTION),
        _ => Err(format!("unsupported N2O mass-basis conversion: {from} -> {to}")),
    }
}

#[derive(Clone, Copy, Debug)]
struct FluxPoint {
    elapsed_seconds: f64,
    flux_mg_m2_h: f64,
}

/// Integrate explicitly supplied, time-stamped flux-rate points by trapezoid.
///
/// Each point must already belong to the same gas, mass basis, experimental
/// unit, treatment, and declared time series. This helper integrates only
/// between the first and last supplied point; it does not infer schedule
/// completeness or fill an unobserved leading/trailing interval.
fn integrate_flux_points_trapezoidal(
    points: &[FluxPoint],
    maximum_gap_seconds: f64,
) -> Result<f64, String> {
    if points.len() < 2 {
        return Err("at least two flux points are required".to_string());
    }
    finite_positive(maximum_gap_seconds, "maximum_gap_seconds")?;
    let mut integrated_mg_m2 = 0.0;
    for (index, point) in points.iter().enumerate() {
        if !point.elapsed_seconds.is_finite() || !point.flux_mg_m2_h.is_finite() {
            return Err(format!("flux point {index} contains a non-finite value"));
        }
        if index == 0 {
            continue;
        }
        let previous = points[index - 1];
        let duration_seconds = point.elapsed_seconds - previous.elapsed_seconds;
        if duration_seconds <= 0.0 {
            return Err("flux point timestamps must be strictly increasing".to_string());
        }
        if duration_seconds > maximum_gap_seconds {
            return Err(format!(
                "flux point gap {duration_seconds}s exceeds declared maximum {maximum_gap_seconds}s"
            ));
        }
        let mean_flux = (previous.flux_mg_m2_h + point.flux_mg_m2_h) / 2.0;
        integrated_mg_m2 += mean_flux * (duration_seconds / 3_600.0);
        if !integrated_mg_m2.is_finite() {
            return Err("integrated flux is non-finite".to_string());
        }
    }
    Ok(integrated_mg_m2)
}

fn require_uri_scheme(value: &str, field: &str) -> Result<(), String> {
    let Some((scheme, rest)) = value.split_once(':') else {
        return Err(format!("{field} must be an absolute URI"));
    };
    if scheme.is_empty()
        || !scheme.as_bytes()[0].is_ascii_alphabetic()
        || !scheme
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"+.-".contains(&byte))
        || rest.is_empty()
    {
        return Err(format!("{field} must be an absolute URI"));
    }
    Ok(())
}

fn validate_required_shape(payload: &serde_json::Value) -> Result<(), String> {
    require_keys(payload, &["fixture_status", "fixture_version", "observations"], "root")?;
    let observations = payload["observations"]
        .as_array()
        .ok_or_else(|| "observations must be an array".to_string())?;

    const OBSERVATION_KEYS: &[&str] = &[
        "schema_version",
        "observation_id",
        "evidence_class",
        "study_id",
        "site_id",
        "block_id",
        "experimental_unit_id",
        "plot_or_vessel_id",
        "treatment_assignment_id",
        "material_batch_id",
        "soil_source_id",
        "observed_at",
        "interval_start",
        "interval_end",
        "gas_species",
        "mass_basis",
        "flux_value",
        "flux_unit",
        "sign_convention",
        "method",
        "instrument",
        "raw_artifact",
        "uncertainty",
        "qc_state",
    ];
    for (index, observation) in observations.iter().enumerate() {
        let label = format!("observations[{index}]");
        require_keys(observation, OBSERVATION_KEYS, &label)?;
        require_keys(
            &observation["method"],
            &[
                "method_id",
                "method_version",
                "chamber_type",
                "chamber_area_m2",
                "chamber_volume_m3",
                "closure_duration_s",
            ],
            &format!("{label}.method"),
        )?;
        require_keys(
            &observation["instrument"],
            &["instrument_id", "calibration_state"],
            &format!("{label}.instrument"),
        )?;
        require_keys(
            &observation["raw_artifact"],
            &["sha256", "artifact_kind"],
            &format!("{label}.raw_artifact"),
        )?;
        require_keys(
            &observation["uncertainty"],
            &["status"],
            &format!("{label}.uncertainty"),
        )?;
        if let Some(detection_limit) = observation.get("detection_limit") {
            if !detection_limit.is_null() {
                require_keys(
                    detection_limit,
                    &["value", "unit", "basis"],
                    &format!("{label}.detection_limit"),
                )?;
            }
        }
    }
    Ok(())
}

fn validate_observation(observation: &Observation, index: usize) -> Result<Vec<String>, String> {
    let label = format!("observations[{index}]");
    let text_fields = [
        (&observation.schema_version, "schema_version"),
        (&observation.observation_id, "observation_id"),
        (&observation.study_id, "study_id"),
        (&observation.site_id, "site_id"),
        (&observation.block_id, "block_id"),
        (&observation.experimental_unit_id, "experimental_unit_id"),
        (&observation.plot_or_vessel_id, "plot_or_vessel_id"),
        (&observation.treatment_assignment_id, "treatment_assignment_id"),
        (&observation.soil_source_id, "soil_source_id"),
        (&observation.gas_species, "gas_species"),
        (&observation.mass_basis, "mass_basis"),
        (&observation.flux_unit, "flux_unit"),
        (&observation.sign_convention, "sign_convention"),
        (&observation.method.method_id, "method.method_id"),
        (&observation.method.method_version, "method.method_version"),
        (&observation.method.chamber_type, "method.chamber_type"),
        (&observation.instrument.instrument_id, "instrument.instrument_id"),
        (&observation.instrument.calibration_state, "instrument.calibration_state"),
        (&observation.raw_artifact.artifact_kind, "raw_artifact.artifact_kind"),
        (&observation.uncertainty.status, "uncertainty.status"),
        (&observation.qc_state, "qc_state"),
    ];
    for (value, field) in text_fields {
        nonblank(value, &format!("{label}.{field}"))?;
    }

    if observation.schema_version != "field-ghg-observation-v0.1" {
        return Err(format!("{label}.schema_version is unsupported"));
    }
    if !matches!(
        observation.evidence_class.as_str(),
        "laboratory_incubation" | "pot_mesocosm" | "field_plot" | "field_farm"
    ) {
        return Err(format!("{label}.evidence_class is unsupported"));
    }
    if observation.sign_convention != "positive_to_atmosphere_negative_from_atmosphere" {
        return Err(format!("{label}.sign_convention is unsupported"));
    }
    if !matches!(
        observation.method.chamber_type.as_str(),
        "static_non_vented" | "static_vented" | "automated" | "flow_through" | "other"
    ) {
        return Err(format!("{label}.method.chamber_type is unsupported"));
    }
    if !matches!(
        observation.instrument.calibration_state.as_str(),
        "in_date" | "out_of_date" | "unknown" | "not_applicable"
    ) {
        return Err(format!("{label}.instrument.calibration_state is unsupported"));
    }
    if !matches!(
        observation.raw_artifact.artifact_kind.as_str(),
        "raw_chromatogram" | "raw_concentration_series" | "instrument_export" | "lab_report" | "other"
    ) {
        return Err(format!("{label}.raw_artifact.artifact_kind is unsupported"));
    }
    if !matches!(
        observation.qc_state.as_str(),
        "pending" | "pass" | "flagged" | "rejected"
    ) {
        return Err(format!("{label}.qc_state is unsupported"));
    }
    if !valid_sha256(&observation.raw_artifact.sha256) {
        return Err(format!("{label}.raw_artifact.sha256 must be 64 lowercase hexadecimal characters"));
    }

    let expected_unit = match (
        observation.gas_species.as_str(),
        observation.mass_basis.as_str(),
    ) {
        ("N2O", "N2O") => "mg_N2O_m2_h",
        ("N2O", "N2O_N") => "mg_N2O_N_m2_h",
        ("CH4", "CH4") => "mg_CH4_m2_h",
        ("CO2", "CO2") => "mg_CO2_m2_h",
        _ => return Err(format!("{label} has an incompatible gas_species / mass_basis pair")),
    };
    if observation.flux_unit != expected_unit {
        return Err(format!(
            "{label}.flux_unit must be {expected_unit} for the declared gas and mass basis"
        ));
    }

    finite_positive(observation.method.chamber_area_m2, &format!("{label}.method.chamber_area_m2"))?;
    finite_positive(observation.method.chamber_volume_m3, &format!("{label}.method.chamber_volume_m3"))?;
    finite_positive(observation.method.closure_duration_s, &format!("{label}.method.closure_duration_s"))?;
    if !observation.flux_value.is_finite() {
        return Err(format!("{label}.flux_value must be finite"));
    }

    let start = parse_rfc3339_seconds(&observation.interval_start)?;
    let observed = parse_rfc3339_seconds(&observation.observed_at)?;
    let end = parse_rfc3339_seconds(&observation.interval_end)?;
    if start >= end || observed < start || observed > end {
        return Err(format!("{label} requires interval_start <= observed_at <= interval_end"));
    }

    match observation.uncertainty.status.as_str() {
        "reported" => {
            let value = observation.uncertainty.standard_uncertainty.ok_or_else(|| {
                format!("{label}.uncertainty.standard_uncertainty is required when status=reported")
            })?;
            finite_nonnegative(value, &format!("{label}.uncertainty.standard_uncertainty"))?;
            let unit = observation.uncertainty.unit.as_deref().ok_or_else(|| {
                format!("{label}.uncertainty.unit is required when status=reported")
            })?;
            if unit != observation.flux_unit {
                return Err(format!("{label}.uncertainty.unit must match flux_unit"));
            }
        }
        "not_available" => {
            if observation.uncertainty.standard_uncertainty.is_some()
                || observation.uncertainty.unit.is_some()
            {
                return Err(format!("{label} must not supply an uncertainty value/unit when status=not_available"));
            }
            nonblank(
                observation.uncertainty.reason_not_available.as_deref().unwrap_or(""),
                &format!("{label}.uncertainty.reason_not_available"),
            )?;
        }
        _ => return Err(format!("{label}.uncertainty.status is unsupported")),
    }

    if let Some(limit) = &observation.detection_limit {
        finite_nonnegative(limit.value, &format!("{label}.detection_limit.value"))?;
        nonblank(&limit.unit, &format!("{label}.detection_limit.unit"))?;
        if limit.unit != observation.flux_unit {
            return Err(format!("{label}.detection_limit.unit must match flux_unit"));
        }
        if !matches!(limit.basis.as_str(), "gas_concentration" | "flux" | "mass") {
            return Err(format!("{label}.detection_limit.basis is unsupported"));
        }
    }

    for (value, field) in [
        (observation.material_batch_id.as_deref(), "material_batch_id"),
        (observation.crop_id.as_deref(), "crop_id"),
        (observation.management_event.as_deref(), "management_event"),
        (observation.missingness_reason.as_deref(), "missingness_reason"),
        (observation.uncertainty.method.as_deref(), "uncertainty.method"),
        (observation.instrument.calibration_reference.as_deref(), "instrument.calibration_reference"),
        (observation.method.flux_calculation_version.as_deref(), "method.flux_calculation_version"),
    ] {
        if let Some(value) = value {
            if !value.is_empty() && value.trim() != value {
                return Err(format!("{label}.{field} must not have surrounding whitespace"));
            }
            if field == "material_batch_id" || field == "uncertainty.method" {
                nonblank(value, &format!("{label}.{field}"))?;
            }
        }
    }
    if let Some(limit) = &observation.detection_limit {
        if let Some(method) = limit.censoring_method.as_deref() {
            if !method.is_empty() && method.trim() != method {
                return Err(format!("{label}.detection_limit.censoring_method must not have surrounding whitespace"));
            }
        }
    }
    for (value, field) in [
        (observation.raw_artifact.source_uri.as_deref(), "raw_artifact.source_uri"),
        (observation.source_record_uri.as_deref(), "source_record_uri"),
    ] {
        if let Some(value) = value {
            require_uri_scheme(value, &format!("{label}.{field}"))?;
        }
    }

    let mut warnings = Vec::new();
    if observation.qc_state != "pass" {
        warnings.push(format!(
            "{label} ({}) has qc_state={}",
            observation.observation_id, observation.qc_state
        ));
    }
    if observation.uncertainty.status != "reported" {
        warnings.push(format!(
            "{label} ({}) has unavailable uncertainty",
            observation.observation_id
        ));
    }
    if !matches!(
        observation.instrument.calibration_state.as_str(),
        "in_date" | "not_applicable"
    ) {
        warnings.push(format!(
            "{label} ({}) has calibration_state={}",
            observation.observation_id, observation.instrument.calibration_state
        ));
    }
    for reason in &observation.qc_reasons {
        nonblank(reason, &format!("{label}.qc_reasons[]"))?;
    }
    Ok(warnings)
}

fn audit_bytes(bytes: &[u8]) -> Result<AuditReport, String> {
    let payload: serde_json::Value =
        serde_json::from_slice(bytes).map_err(|error| format!("invalid JSON: {error}"))?;
    validate_required_shape(&payload)?;
    let envelope: FixtureEnvelope =
        serde_json::from_value(payload).map_err(|error| format!("invalid fixture shape: {error}"))?;

    if envelope.fixture_status != FIXTURE_STATUS {
        return Err("only explicitly synthetic fixture envelopes are accepted by v0.1".to_string());
    }
    nonblank(&envelope.fixture_version, "fixture_version")?;
    if envelope.observations.is_empty() {
        return Err("observations must contain at least one record".to_string());
    }

    let mut ids = HashSet::new();
    let mut gas_species_counts = BTreeMap::new();
    let mut evidence_class_counts = BTreeMap::new();
    let mut qc_state_counts = BTreeMap::new();
    let mut warnings = Vec::new();
    let mut summaries = Vec::new();
    let mut all_qualified = true;

    for (index, observation) in envelope.observations.iter().enumerate() {
        if !ids.insert(observation.observation_id.clone()) {
            return Err(format!("duplicate observation_id: {}", observation.observation_id));
        }
        warnings.extend(validate_observation(observation, index)?);
        *gas_species_counts.entry(observation.gas_species.clone()).or_insert(0) += 1;
        *evidence_class_counts.entry(observation.evidence_class.clone()).or_insert(0) += 1;
        *qc_state_counts.entry(observation.qc_state.clone()).or_insert(0) += 1;

        if observation.qc_state != "pass"
            || observation.uncertainty.status != "reported"
            || !matches!(
                observation.instrument.calibration_state.as_str(),
                "in_date" | "not_applicable"
            )
        {
            all_qualified = false;
        }

        summaries.push(ObservationSummary {
            observation_id: observation.observation_id.clone(),
            gas_species: observation.gas_species.clone(),
            mass_basis: observation.mass_basis.clone(),
            flux_value: observation.flux_value,
            flux_unit: observation.flux_unit.clone(),
            evidence_class: observation.evidence_class.clone(),
            experimental_unit_id: observation.experimental_unit_id.clone(),
            qc_state: observation.qc_state.clone(),
            uncertainty_status: observation.uncertainty.status.clone(),
        });
    }

    warnings.sort();
    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status: if all_qualified {
            "synthetic_fixture_records_validated_only"
        } else {
            "synthetic_fixture_unqualified_due_to_qc_or_uncertainty"
        },
        fixture_status: FIXTURE_STATUS,
        fixture_version: envelope.fixture_version,
        input_sha256: sha256_hex(bytes),
        observation_count: summaries.len(),
        gas_species_counts,
        evidence_class_counts,
        qc_state_counts,
        observations: summaries,
        warnings,
        interpretation_boundary: INTERPRETATION_BOUNDARY,
    })
}

fn write_new_file(path: &Path, content: &[u8]) -> io::Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(content)?;
    file.write_all(b"\n")?;
    file.sync_all()
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <synthetic-fixture.json> <new-report.json>",
            args.first().map(String::as_str).unwrap_or("field-ghg-audit")
        )
        .into());
    }

    let input_path = Path::new(&args[1]);
    let output_path = Path::new(&args[2]);
    if output_path.exists() {
        return Err(format!("refusing to overwrite existing report: {}", output_path.display()).into());
    }

    let bytes = fs::read(input_path)?;
    let report = audit_bytes(&bytes).map_err(io::Error::other)?;
    let output = serde_json::to_vec_pretty(&report)?;
    write_new_file(output_path, &output)?;
    println!(
        "wrote {} observations; status={}; input_sha256={}",
        report.observation_count, report.status, report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("field GHG audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn fixture_value() -> Value {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-field-ghg-observations.json"
        ))
        .expect("committed synthetic fixture must parse")
    }

    fn audit_value(value: &Value) -> Result<AuditReport, String> {
        audit_bytes(value.to_string().as_bytes())
    }

    #[test]
    fn committed_fixture_is_structurally_valid_but_not_qualified() {
        let report = audit_value(&fixture_value()).expect("fixture should validate");
        assert_eq!(report.observation_count, 3);
        assert_eq!(report.status, "synthetic_fixture_unqualified_due_to_qc_or_uncertainty");
        assert_eq!(report.gas_species_counts.get("N2O"), Some(&1));
        assert_eq!(report.gas_species_counts.get("CH4"), Some(&1));
    }

    #[test]
    fn valid_n2o_n_basis_is_accepted() {
        let mut value = fixture_value();
        let observation = &mut value["observations"][0];
        observation["mass_basis"] = Value::String("N2O_N".to_string());
        observation["flux_unit"] = Value::String("mg_N2O_N_m2_h".to_string());
        observation["uncertainty"]["unit"] = Value::String("mg_N2O_N_m2_h".to_string());
        let report = audit_value(&value).expect("matching N2O-N basis must pass");
        assert_eq!(report.observation_count, 3);
    }

    #[test]
    fn mismatched_n2o_n_unit_is_rejected() {
        let mut value = fixture_value();
        value["observations"][0]["mass_basis"] = Value::String("N2O_N".to_string());
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn placeholder_digest_is_rejected() {
        let mut value = fixture_value();
        value["observations"][0]["raw_artifact"]["sha256"] =
            Value::String("placeholder".to_string());
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn duplicate_observation_id_is_rejected() {
        let mut value = fixture_value();
        value["observations"][1]["observation_id"] =
            value["observations"][0]["observation_id"].clone();
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn observed_timestamp_outside_chamber_interval_is_rejected() {
        let mut value = fixture_value();
        value["observations"][0]["observed_at"] =
            Value::String("2026-11-20T09:01:00Z".to_string());
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn incorrect_sign_convention_is_rejected() {
        let mut value = fixture_value();
        value["observations"][0]["sign_convention"] =
            Value::String("negative_to_atmosphere".to_string());
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn unavailable_uncertainty_requires_reason_and_cannot_include_value() {
        let mut value = fixture_value();
        let uncertainty = &mut value["observations"][0]["uncertainty"];
        uncertainty["status"] = Value::String("not_available".to_string());
        uncertainty["standard_uncertainty"] = Value::Null;
        uncertainty["unit"] = Value::Null;
        uncertainty["reason_not_available"] = Value::Null;
        assert!(audit_value(&value).is_err());
        uncertainty["reason_not_available"] =
            Value::String("not measured in synthetic negative control".to_string());
        let report = audit_value(&value).expect("explicit unavailable status should remain auditable");
        assert_eq!(
            report.status,
            "synthetic_fixture_unqualified_due_to_qc_or_uncertainty"
        );
    }

    #[test]
    fn unknown_root_key_is_rejected() {
        let mut value = fixture_value();
        value["unreviewed_override"] = Value::Bool(true);
        assert!(audit_value(&value).is_err());
    }

    #[test]
    fn timestamp_offsets_are_compared_in_utc() {
        let start = parse_rfc3339_seconds("2026-11-20T10:00:00+02:00").unwrap();
        let observed = parse_rfc3339_seconds("2026-11-20T08:30:00Z").unwrap();
        let end = parse_rfc3339_seconds("2026-11-20T09:00:00Z").unwrap();
        assert!(start < observed && observed < end);
    }

    #[test]
    fn invalid_calendar_date_is_rejected() {
        assert!(parse_rfc3339_seconds("2026-02-30T09:00:00Z").is_err());
        assert!(parse_rfc3339_seconds("2026-11-20T25:00:00Z").is_err());
    }

    #[test]
    fn n2o_to_n2o_n_mass_basis_conversion_is_stoichiometric() {
        let nitrogen_mass = convert_n2o_mass_basis(44.0, "N2O", "N2O_N").unwrap();
        assert!((nitrogen_mass - 28.0).abs() < 1e-12);
        let n2o_mass = convert_n2o_mass_basis(28.0, "N2O_N", "N2O").unwrap();
        assert!((n2o_mass - 44.0).abs() < 1e-12);
    }

    #[test]
    fn n2o_mass_basis_conversion_preserves_negative_flux_sign() {
        let nitrogen_mass = convert_n2o_mass_basis(-44.0, "N2O", "N2O_N").unwrap();
        assert!((nitrogen_mass + 28.0).abs() < 1e-12);
    }

    #[test]
    fn unrelated_gas_mass_basis_conversion_is_rejected() {
        assert!(convert_n2o_mass_basis(1.0, "CH4", "N2O_N").is_err());
    }

    #[test]
    fn trapezoidal_flux_integration_matches_hand_calculation() {
        let points = [
            FluxPoint {
                elapsed_seconds: 0.0,
                flux_mg_m2_h: 1.0,
            },
            FluxPoint {
                elapsed_seconds: 3_600.0,
                flux_mg_m2_h: 3.0,
            },
        ];
        let integrated = integrate_flux_points_trapezoidal(&points, 3_600.0).unwrap();
        assert!((integrated - 2.0).abs() < 1e-12);
    }

    #[test]
    fn trapezoidal_flux_integration_rejects_excessive_gap() {
        let points = [
            FluxPoint {
                elapsed_seconds: 0.0,
                flux_mg_m2_h: 1.0,
            },
            FluxPoint {
                elapsed_seconds: 7_200.0,
                flux_mg_m2_h: 3.0,
            },
        ];
        assert!(integrate_flux_points_trapezoidal(&points, 3_600.0).is_err());
    }

    #[test]
    fn trapezoidal_flux_integration_rejects_nonmonotonic_time() {
        let points = [
            FluxPoint {
                elapsed_seconds: 3_600.0,
                flux_mg_m2_h: 1.0,
            },
            FluxPoint {
                elapsed_seconds: 0.0,
                flux_mg_m2_h: 3.0,
            },
        ];
        assert!(integrate_flux_points_trapezoidal(&points, 7_200.0).is_err());
    }

    #[test]
    fn trapezoidal_flux_integration_preserves_net_negative_uptake() {
        let points = [
            FluxPoint {
                elapsed_seconds: 0.0,
                flux_mg_m2_h: -2.0,
            },
            FluxPoint {
                elapsed_seconds: 3_600.0,
                flux_mg_m2_h: -2.0,
            },
        ];
        let integrated = integrate_flux_points_trapezoidal(&points, 3_600.0).unwrap();
        assert!((integrated + 2.0).abs() < 1e-12);
    }

    #[test]
    fn malformed_timezone_is_rejected() {
        assert!(parse_rfc3339_seconds("2026-11-20T09:00:00+25:00").is_err());
        assert!(parse_rfc3339_seconds("2026-11-20T09:00:00+02").is_err());
    }
}
