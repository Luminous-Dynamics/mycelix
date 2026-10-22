// Copyright (C) 2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural safety gate for scoped landscape measurement authorization records.
//!
//! This gate does not authenticate signatures, resolve Mycelix authority records,
//! establish land title, or decide whether a permission is currently effective.
//! Hosts must resolve evidence digests, verify signatures/authority, check the
//! time window, and re-evaluate revocation before permitting real measurements.

use chrono::DateTime;
use serde_json::Value;

pub const LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA: &str =
    "mycelix.landscape-measurement-authorization.v0";
pub const LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/landscape-measurement-authorization-v0.schema.json";

const CONSENT_STATES: &[&str] = &["granted", "denied", "withdrawn", "expired", "unknown"];
const ACCESS_STATES: &[&str] = &["authorized", "denied", "revoked", "expired", "unknown"];
const PUBLICATION_CONSENT_STATES: &[&str] =
    &["not-requested", "granted", "denied", "withdrawn", "unknown"];
const PUBLICATION_POLICIES: &[&str] = &["private", "controlled", "embargoed", "public"];
const COORDINATE_DISCLOSURE: &[&str] = &["none", "coarse", "exact"];
const ACTIVITIES: &[&str] = &[
    "measure-site",
    "install-sensor",
    "collect-observation",
    "maintain-sensor",
];
const EVIDENCE_CLASSES: &[&str] = &["synthetic", "asserted", "signed"];
const VERIFICATION_STATES: &[&str] = &["not-verified", "signature-verified", "authority-resolved"];
const ALLOWED_FIELDS: &[&str] = &[
    "schema",
    "schema_uri",
    "authorization_id",
    "created_at",
    "project_snapshot_sha256",
    "site_ref_sha256",
    "metric_id",
    "unit",
    "activity",
    "participant_ref",
    "consent_status",
    "consent_evidence_sha256",
    "site_access_status",
    "site_access_evidence_sha256",
    "publication_consent_status",
    "publication_consent_evidence_sha256",
    "publication_policy",
    "coordinate_disclosure",
    "valid_from",
    "valid_until",
    "evidence_class",
    "verification_status",
    "statement",
];

/// Validate record shape and high-consequence status/digest consistency.
///
/// Ok(()) means only that this record is structurally consistent. It is not
/// a decision to enter a site, install equipment, collect observations, or
/// publish data. In particular, verification_status is an input claim and is
/// not authenticated by this function.
pub fn validate_measurement_authorization(record: &Value) -> Result<(), Vec<String>> {
    let mut errors = Vec::new();
    let Some(object) = record.as_object() else {
        return Err(vec!["$: expected a JSON object".to_owned()]);
    };
    for field in object.keys() {
        if !ALLOWED_FIELDS.contains(&field.as_str()) {
            errors.push(format!(
                "$.{field}: unknown property is not allowed by authorization v0"
            ));
        }
    }

    if record.get("schema").and_then(Value::as_str)
        != Some(LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA)
    {
        errors.push(format!(
            "$.schema: expected {LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA}"
        ));
    }
    if record.get("schema_uri").and_then(Value::as_str)
        != Some(LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA_URI)
    {
        errors.push(format!(
            "$.schema_uri: expected {LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA_URI}"
        ));
    }

    for field in [
        "authorization_id",
        "created_at",
        "metric_id",
        "unit",
        "activity",
        "participant_ref",
        "valid_from",
        "valid_until",
        "statement",
    ] {
        require_nonblank(record, field, "$", &mut errors);
    }
    let parsed_times: std::collections::BTreeMap<&str, DateTime<chrono::FixedOffset>> =
        ["created_at", "valid_from", "valid_until"]
            .into_iter()
            .filter_map(|field| {
                let raw = record.get(field).and_then(Value::as_str)?;
                match DateTime::parse_from_rfc3339(raw) {
                    Ok(parsed) => Some((field, parsed)),
                    Err(_) => {
                        errors.push(format!("$.{field}: expected a valid RFC 3339 timestamp"));
                        None
                    }
                }
            })
            .collect();
    if let (Some(start), Some(end)) = (
        parsed_times.get("valid_from"),
        parsed_times.get("valid_until"),
    ) {
        if end <= start {
            errors.push("$.valid_until: must be strictly later than valid_from".to_owned());
        }
    }

    for field in ["project_snapshot_sha256", "site_ref_sha256"] {
        if !record
            .get(field)
            .and_then(Value::as_str)
            .is_some_and(is_sha256)
        {
            errors.push(format!(
                "$.{field}: expected 64 lowercase hexadecimal characters"
            ));
        }
    }

    let consent_status = record
        .get("consent_status")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !CONSENT_STATES.contains(&consent_status) {
        errors.push("$.consent_status: unsupported consent state".to_owned());
    }
    validate_evidence_binding(
        record,
        "consent_status",
        "granted",
        "consent_evidence_sha256",
        "$",
        &mut errors,
    );

    let access_status = record
        .get("site_access_status")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !ACCESS_STATES.contains(&access_status) {
        errors.push("$.site_access_status: unsupported site-access state".to_owned());
    }
    validate_evidence_binding(
        record,
        "site_access_status",
        "authorized",
        "site_access_evidence_sha256",
        "$",
        &mut errors,
    );

    let publication_consent = record
        .get("publication_consent_status")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !PUBLICATION_CONSENT_STATES.contains(&publication_consent) {
        errors
            .push("$.publication_consent_status: unsupported publication-consent state".to_owned());
    }
    validate_evidence_binding(
        record,
        "publication_consent_status",
        "granted",
        "publication_consent_evidence_sha256",
        "$",
        &mut errors,
    );

    let publication_policy = record
        .get("publication_policy")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !PUBLICATION_POLICIES.contains(&publication_policy) {
        errors.push("$.publication_policy: unsupported policy".to_owned());
    }
    let coordinate_disclosure = record
        .get("coordinate_disclosure")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !COORDINATE_DISCLOSURE.contains(&coordinate_disclosure) {
        errors.push("$.coordinate_disclosure: unsupported precision policy".to_owned());
    }
    if publication_policy == "public" {
        if publication_consent != "granted" {
            errors.push(
                "$.publication_consent_status: public policy requires separate granted publication consent"
                    .to_owned(),
            );
        }
        if coordinate_disclosure == "exact" {
            errors.push(
                "$.coordinate_disclosure: public policy cannot disclose exact coordinates"
                    .to_owned(),
            );
        }
    }

    let activity = record.get("activity").and_then(Value::as_str).unwrap_or("");
    if !ACTIVITIES.contains(&activity) {
        errors.push("$.activity: unsupported measurement activity".to_owned());
    }
    let evidence_class = record
        .get("evidence_class")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !EVIDENCE_CLASSES.contains(&evidence_class) {
        errors.push("$.evidence_class: unsupported evidence class".to_owned());
    }
    let verification_status = record
        .get("verification_status")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !VERIFICATION_STATES.contains(&verification_status) {
        errors.push("$.verification_status: unsupported verification state".to_owned());
    }
    if evidence_class == "synthetic" && verification_status != "not-verified" {
        errors
            .push("$.verification_status: synthetic examples must remain not-verified".to_owned());
    }
    if evidence_class != "signed" && verification_status != "not-verified" {
        errors.push(
            "$.verification_status: a non-signed record cannot claim signature or authority resolution"
                .to_owned(),
        );
    }

    let mut coordinate_paths = Vec::new();
    collect_structured_coordinate_paths(record, "$", &mut coordinate_paths);
    if !coordinate_paths.is_empty() {
        errors.push(format!(
            "$: structured coordinates are forbidden; use an opaque site_ref_sha256: {}",
            coordinate_paths.join(", ")
        ));
    }

    if errors.is_empty() {
        Ok(())
    } else {
        errors.sort();
        Err(errors)
    }
}

fn validate_evidence_binding(
    record: &Value,
    status_field: &str,
    positive_state: &str,
    digest_field: &str,
    path: &str,
    errors: &mut Vec<String>,
) {
    let status = record
        .get(status_field)
        .and_then(Value::as_str)
        .unwrap_or("");
    let digest = record.get(digest_field);
    if status == positive_state {
        if !digest.and_then(Value::as_str).is_some_and(is_sha256) {
            errors.push(format!(
                "{path}.{digest_field}: {positive_state} requires a valid 64-character lowercase SHA-256 reference"
            ));
        }
    } else if !digest.is_some_and(Value::is_null) {
        errors.push(format!(
            "{path}.{digest_field}: non-{positive_state} status must use null, not claim positive evidence"
        ));
    }
}

fn require_nonblank(record: &Value, field: &str, path: &str, errors: &mut Vec<String>) {
    if !record
        .get(field)
        .and_then(Value::as_str)
        .is_some_and(|text| !text.trim().is_empty())
    {
        errors.push(format!("{path}.{field}: required non-empty string"));
    }
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn collect_structured_coordinate_paths(value: &Value, path: &str, out: &mut Vec<String>) {
    match value {
        Value::Object(object) => {
            for (key, child) in object {
                let normalized = key.to_ascii_lowercase();
                if matches!(
                    normalized.as_str(),
                    "latitude"
                        | "longitude"
                        | "lat"
                        | "lon"
                        | "lng"
                        | "coordinates"
                        | "coordinate_pair"
                ) {
                    out.push(format!("{path}.{key}"));
                } else {
                    collect_structured_coordinate_paths(child, &format!("{path}.{key}"), out);
                }
            }
        }
        Value::Array(values) => {
            for (index, child) in values.iter().enumerate() {
                collect_structured_coordinate_paths(child, &format!("{path}[{index}]"), out);
            }
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fixture() -> Value {
        serde_json::from_str(include_str!(
            "../../../fixtures/landscape-measurement-authorization-v0.public.example.json"
        ))
        .expect("authorization fixture must be valid JSON")
    }

    #[test]
    fn versioned_synthetic_example_passes_structural_gate_only() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../schemas/landscape-measurement-authorization-v0.schema.json"
        ))
        .expect("authorization schema JSON");
        let record = fixture();
        assert_eq!(
            schema["$id"],
            LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA_URI
        );
        assert_eq!(record["schema"], LANDSCAPE_MEASUREMENT_AUTHORIZATION_SCHEMA);
        assert_eq!(record["evidence_class"], "synthetic");
        assert_eq!(record["verification_status"], "not-verified");
        assert!(validate_measurement_authorization(&record).is_ok());
    }

    #[test]
    fn consent_and_access_positive_states_require_their_own_evidence() {
        let mut consent_missing = fixture();
        consent_missing["consent_evidence_sha256"] = Value::Null;
        assert!(
            validate_measurement_authorization(&consent_missing)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("consent_evidence_sha256"))
        );

        let mut access_missing = fixture();
        access_missing["site_access_evidence_sha256"] = Value::Null;
        assert!(
            validate_measurement_authorization(&access_missing)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("site_access_evidence_sha256"))
        );
    }

    #[test]
    fn denied_or_unknown_states_cannot_claim_positive_evidence() {
        let mut denied = fixture();
        denied["consent_status"] = json!("denied");
        denied["consent_evidence_sha256"] = json!("c".repeat(64));
        assert!(
            validate_measurement_authorization(&denied)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("non-granted status must use null"))
        );

        let mut unknown = fixture();
        unknown["site_access_status"] = json!("unknown");
        unknown["site_access_evidence_sha256"] = Value::Null;
        assert!(validate_measurement_authorization(&unknown).is_ok());
    }

    #[test]
    fn publication_consent_is_separate_from_measurement_consent() {
        let mut record = fixture();
        record["consent_status"] = json!("granted");
        record["publication_consent_status"] = json!("not-requested");
        record["publication_consent_evidence_sha256"] = Value::Null;
        record["publication_policy"] = json!("public");
        assert!(
            validate_measurement_authorization(&record)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("separate granted publication consent"))
        );
    }

    #[test]
    fn public_policy_rejects_exact_coordinate_disclosure() {
        let mut record = fixture();
        record["publication_consent_status"] = json!("granted");
        record["publication_consent_evidence_sha256"] = json!("e".repeat(64));
        record["publication_policy"] = json!("public");
        record["coordinate_disclosure"] = json!("exact");
        assert!(
            validate_measurement_authorization(&record)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("cannot disclose exact coordinates"))
        );
    }

    #[test]
    fn synthetic_record_cannot_claim_authority_verification() {
        let mut record = fixture();
        record["verification_status"] = json!("authority-resolved");
        assert!(
            validate_measurement_authorization(&record)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("synthetic examples must remain not-verified"))
        );
    }

    #[test]
    fn validity_window_requires_valid_timestamps_and_positive_duration() {
        let mut invalid_timestamp = fixture();
        invalid_timestamp["valid_from"] = json!("not-rfc3339");
        assert!(
            validate_measurement_authorization(&invalid_timestamp)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("valid RFC 3339 timestamp"))
        );

        let mut reversed_window = fixture();
        reversed_window["valid_from"] = json!("2026-12-31T00:00:00Z");
        reversed_window["valid_until"] = json!("2026-10-21T00:00:00Z");
        assert!(
            validate_measurement_authorization(&reversed_window)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("strictly later than valid_from"))
        );
    }

    #[test]
    fn undeclared_authorization_properties_are_rejected() {
        let mut record = fixture();
        record["grant_everything"] = json!(true);
        assert!(
            validate_measurement_authorization(&record)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("unknown property is not allowed"))
        );
    }

    #[test]
    fn raw_coordinate_fields_are_rejected() {
        let mut record = fixture();
        record["site_location"] = json!({"latitude": -26.2, "longitude": 28.0});
        assert!(
            validate_measurement_authorization(&record)
                .unwrap_err()
                .iter()
                .any(|error| error.contains("structured coordinates are forbidden"))
        );
    }
}
