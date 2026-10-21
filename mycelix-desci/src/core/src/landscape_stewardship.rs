// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Runtime safety gate for the Mycelix landscape stewardship profile.
//!
//! Run the versioned JSON Schema validator before this gate. This function is
//! intentionally narrower than JSON Schema: it enforces high-consequence
//! publication, provenance, evidence-class and structured-location rules at
//! the application boundary. It does not verify signatures, land title,
//! consent records, digest-to-source byte equality, or scientific truth.

use serde_json::Value;

pub const LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA: &str =
    "mycelix.landscape-stewardship-profile.v0";
pub const LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI: &str =
    "https://luminousdynamics.io/schemas/landscape-stewardship-profile-v0.schema.json";

const VALID_SCOPES: &[&str] = &["site-parcel", "district", "watershed", "bioregion"];
const VALID_EVIDENCE_CLASSES: &[&str] =
    &["observed", "curated", "synthetic", "modelled", "assumed", "missing"];
const VALID_RECORD_TYPES: &[&str] = &[
    "project-version-review",
    "scenario-review",
    "field-observation",
    "stewardship-commitment",
    "consent-change",
    "decision-withdrawal",
];

/// Apply non-negotiable policy gates to a parsed stewardship profile.
///
/// This is not a replacement for validating the full JSON Schema and must not
/// be used as evidence that a signature, authority reference, consent record,
/// measurement, or model result is authentic.
pub fn validate_profile_safety_gate(profile: &Value) -> Result<(), Vec<String>> {
    let mut errors = Vec::new();
    let null_value = Value::Null;
    if !profile.is_object() {
        return Err(vec!["$: expected a JSON object".to_string()]);
    }

    if profile.get("schema").and_then(Value::as_str)
        != Some(LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA)
    {
        errors.push(format!("$.schema: expected {LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA}"));
    }
    if profile.get("schema_uri").and_then(Value::as_str)
        != Some(LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI)
    {
        errors.push(format!(
            "$.schema_uri: expected {LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI}"
        ));
    }

    for field in ["record_id", "created_at", "project_id", "statement"] {
        require_nonblank(profile, field, "$", &mut errors);
    }

    let record_type = profile.get("record_type").and_then(Value::as_str).unwrap_or("");
    if !VALID_RECORD_TYPES.contains(&record_type) {
        errors.push("$.record_type: unsupported record type".to_string());
    }
    let evidence_class = profile
        .get("evidence_class")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !VALID_EVIDENCE_CLASSES.contains(&evidence_class) {
        errors.push("$.evidence_class: unsupported evidence class".to_string());
    }

    let snapshot = match profile.get("project_snapshot") {
        Some(value) if value.is_object() => value,
        _ => {
            errors.push("$.project_snapshot: expected an object".to_string());
            &null_value
        }
    };
    if snapshot.get("system").and_then(Value::as_str) != Some("sol-atlas") {
        errors.push("$.project_snapshot.system: expected sol-atlas".to_string());
    }
    if snapshot.get("schema").and_then(Value::as_str) != Some("luminous.landscape-design.v0") {
        errors.push("$.project_snapshot.schema: unsupported design snapshot schema".to_string());
    }
    if snapshot.get("schema_uri").and_then(Value::as_str)
        != Some("https://luminousdynamics.io/schemas/landscape-design-v0.schema.json")
    {
        errors.push("$.project_snapshot.schema_uri: unsupported design schema URI".to_string());
    }
    if !snapshot
        .get("sha256")
        .and_then(Value::as_str)
        .is_some_and(is_sha256)
    {
        errors.push("$.project_snapshot.sha256: expected 64 lowercase hexadecimal characters".to_string());
    }
    let scope = snapshot.get("scope").and_then(Value::as_str).unwrap_or("");
    if !VALID_SCOPES.contains(&scope) {
        errors.push("$.project_snapshot.scope: unsupported project scope".to_string());
    }
    if snapshot.get("coordinate_frame_kind").and_then(Value::as_str)
        != Some("LOCAL_CARTESIAN_DRAWING_UNITS")
        || snapshot.get("georeferenced").and_then(Value::as_bool) != Some(false)
    {
        errors.push(
            "$.project_snapshot: v0 only accepts the explicitly non-georeferenced local drawing frame"
                .to_string(),
        );
    }

    let geometry_status = snapshot
        .get("geometry_status")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !matches!(geometry_status, "schematic-design-intent" | "synthetic-benchmark") {
        errors.push("$.project_snapshot.geometry_status: unsupported v0 geometry status".to_string());
    }
    if geometry_status == "synthetic-benchmark"
        && !matches!(evidence_class, "synthetic" | "missing")
    {
        errors.push(
            "$.evidence_class: a synthetic benchmark snapshot cannot be represented as observed or curated evidence"
                .to_string(),
        );
    }

    let actor = match profile.get("actor_authority") {
        Some(value) if value.is_object() => value,
        _ => {
            errors.push("$.actor_authority: expected an object".to_string());
            &Value::Null
        }
    };
    for field in ["actor_id", "role", "authority_kind", "consent_reference"] {
        require_nonblank(actor, field, "$.actor_authority", &mut errors);
    }
    let authority_kind = actor
        .get("authority_kind")
        .and_then(Value::as_str)
        .unwrap_or("");
    if authority_kind != "not-claimed"
        && !has_nonblank(actor.get("authority_reference"))
    {
        errors.push(
            "$.actor_authority.authority_reference: required when an authority basis is claimed"
                .to_string(),
        );
    }
    let publication_consent = actor
        .get("publication_consent")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !matches!(
        publication_consent,
        "not-requested" | "granted" | "denied" | "withdrawn"
    ) {
        errors.push("$.actor_authority.publication_consent: unsupported consent state".to_string());
    }

    let access = match profile.get("access_policy") {
        Some(value) if value.is_object() => value,
        _ => {
            errors.push("$.access_policy: expected an object".to_string());
            &Value::Null
        }
    };
    let visibility = access.get("visibility").and_then(Value::as_str).unwrap_or("");
    if !matches!(visibility, "private" | "controlled" | "embargoed" | "public") {
        errors.push("$.access_policy.visibility: unsupported visibility".to_string());
    }
    let coordinate_disclosure = access
        .get("coordinate_disclosure")
        .and_then(Value::as_str)
        .unwrap_or("");
    if !matches!(coordinate_disclosure, "none" | "coarse" | "exact") {
        errors.push("$.access_policy.coordinate_disclosure: unsupported precision policy".to_string());
    }
    if access.get("sensitive_site_data_included").and_then(Value::as_bool) != Some(false) {
        errors.push(
            "$.access_policy.sensitive_site_data_included: v0 profiles must not embed sensitive site data"
                .to_string(),
        );
    }

    if visibility == "public" {
        if publication_consent != "granted" {
            errors.push(
                "$.actor_authority.publication_consent: public visibility requires granted consent"
                    .to_string(),
            );
        }
        if !has_nonblank(actor.get("publication_authority_reference")) {
            errors.push(
                "$.actor_authority.publication_authority_reference: public visibility requires a publication-authority reference"
                    .to_string(),
            );
        }
        if coordinate_disclosure == "exact" {
            errors.push(
                "$.access_policy.coordinate_disclosure: public visibility cannot disclose exact coordinates"
                    .to_string(),
            );
        }
    }

    let mut coordinate_paths = Vec::new();
    collect_structured_coordinate_paths(profile, "$", &mut coordinate_paths);
    if !coordinate_paths.is_empty() {
        errors.push(format!(
            "$: structured coordinate fields are forbidden in this profile: {}",
            coordinate_paths.join(", ")
        ));
    }

    match record_type {
        "scenario-review" => {
            let scenario = profile.get("scenario_reference");
            if !scenario.is_some_and(Value::is_object) {
                errors.push("$.scenario_reference: scenario-review requires an object".to_string());
            } else if let Some(scenario) = scenario {
                if scenario.get("system").and_then(Value::as_str) != Some("symtropy") {
                    errors.push("$.scenario_reference.system: expected symtropy".to_string());
                }
                if !scenario
                    .get("manifest_sha256")
                    .and_then(Value::as_str)
                    .is_some_and(is_sha256)
                {
                    errors.push("$.scenario_reference.manifest_sha256: invalid SHA-256".to_string());
                }
                require_nonblank(scenario, "model_id", "$.scenario_reference", &mut errors);
                require_nonblank(scenario, "model_version", "$.scenario_reference", &mut errors);
                let purpose = scenario.get("purpose").and_then(Value::as_str).unwrap_or("");
                if !matches!(purpose, "schema-qualification" | "synthetic-benchmark") {
                    errors.push(
                        "$.scenario_reference.purpose: v0 cannot authorize analytical or scientific simulation claims"
                            .to_string(),
                    );
                }
                if purpose == "synthetic-benchmark" {
                    if scenario.get("evidence_class").and_then(Value::as_str) != Some("synthetic") {
                        errors.push(
                            "$.scenario_reference.evidence_class: synthetic benchmark results must remain synthetic"
                                .to_string(),
                        );
                    }
                    if !scenario
                        .get("result_sha256")
                        .and_then(Value::as_str)
                        .is_some_and(is_sha256)
                    {
                        errors.push(
                            "$.scenario_reference.result_sha256: synthetic benchmark requires an exact result digest"
                                .to_string(),
                        );
                    }
                }
            }
        }
        "field-observation" => {
            if evidence_class != "observed" {
                errors.push("$.evidence_class: field-observation requires observed evidence".to_string());
            }
            let observation = profile.get("observation");
            if !observation.is_some_and(Value::is_object) {
                errors.push("$.observation: field-observation requires an observation object".to_string());
            } else if let Some(observation) = observation {
                for field in [
                    "metric_id",
                    "unit",
                    "source_reference",
                    "method_reference",
                    "observed_at",
                    "uncertainty",
                ] {
                    require_nonblank(observation, field, "$.observation", &mut errors);
                }
            }
        }
        "stewardship-commitment" => {
            let commitment = profile.get("commitment");
            if !commitment.is_some_and(Value::is_object) {
                errors.push("$.commitment: stewardship-commitment requires an object".to_string());
            } else if let Some(commitment) = commitment {
                if commitment.get("status").and_then(Value::as_str) == Some("accepted")
                    && !has_nonblank(commitment.get("acceptance_reference"))
                {
                    errors.push(
                        "$.commitment.acceptance_reference: accepted commitments require an acceptance reference"
                            .to_string(),
                    );
                }
            }
        }
        "consent-change" => {
            if let Some(change) = profile.get("consent_change") {
                let status = change.get("status").and_then(Value::as_str).unwrap_or("");
                if matches!(status, "withdrawn" | "expired") && visibility == "public" {
                    errors.push(
                        "$.access_policy.visibility: withdrawn or expired consent transitions cannot be published as public payloads"
                            .to_string(),
                    );
                }
            } else {
                errors.push("$.consent_change: consent-change requires an object".to_string());
            }
        }
        "decision-withdrawal" => {
            require_nonblank(profile, "supersedes_record_id", "$", &mut errors);
            require_nonblank(profile, "withdrawal_reason", "$", &mut errors);
        }
        _ => {}
    }

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

fn require_nonblank(
    object: &Value,
    key: &str,
    path: &str,
    errors: &mut Vec<String>,
) {
    if !has_nonblank(object.get(key)) {
        errors.push(format!("{path}.{key}: required non-empty string"));
    }
}

fn has_nonblank(value: Option<&Value>) -> bool {
    value
        .and_then(Value::as_str)
        .is_some_and(|text| !text.trim().is_empty())
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
                    "latitude" | "longitude" | "lat" | "lon" | "lng" | "coordinates" | "coordinate_pair"
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

    const SNAPSHOT_SHA256: &str =
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    fn valid_public_profile() -> Value {
        json!({
            "schema": LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA,
            "schema_uri": LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI,
            "record_id": "landscape-record-demo-001",
            "record_type": "project-version-review",
            "created_at": "2026-10-21T00:00:00Z",
            "project_id": "project-demo",
            "project_snapshot": {
                "system": "sol-atlas",
                "schema": "luminous.landscape-design.v0",
                "schema_uri": "https://luminousdynamics.io/schemas/landscape-design-v0.schema.json",
                "sha256": SNAPSHOT_SHA256,
                "scope": "site-parcel",
                "coordinate_frame_kind": "LOCAL_CARTESIAN_DRAWING_UNITS",
                "georeferenced": false,
                "geometry_status": "schematic-design-intent"
            },
            "actor_authority": {
                "actor_id": "did:example:123",
                "role": "designer",
                "authority_kind": "project-role",
                "authority_reference": "authority-record:demo",
                "consent_reference": "consent-record:demo",
                "publication_consent": "granted",
                "publication_authority_reference": "publication-record:demo"
            },
            "access_policy": {
                "visibility": "public",
                "coordinate_disclosure": "coarse",
                "sensitive_site_data_included": false
            },
            "evidence_class": "curated",
            "statement": "Synthetic conformance fixture only; not an actual decision or authority record."
        })
    }

    #[test]
    fn schema_identity_is_versioned_and_fixture_passes_safety_gate() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../schemas/landscape-stewardship-profile-v0.schema.json"
        ))
        .unwrap();
        let profile = valid_public_profile();
        assert_eq!(schema["$id"], LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI);
        assert_eq!(schema["properties"]["schema"]["const"], LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA);
        assert_eq!(schema["properties"]["schema_uri"]["const"], LANDSCAPE_STEWARDSHIP_PROFILE_SCHEMA_URI);
        assert!(validate_profile_safety_gate(&profile).is_ok());
    }

    #[test]
    fn public_profile_requires_consent_and_publication_authority() {
        let mut profile = valid_public_profile();
        profile["actor_authority"]["publication_consent"] = json!("withdrawn");
        profile["actor_authority"].as_object_mut().unwrap().remove("publication_authority_reference");
        let errors = validate_profile_safety_gate(&profile).unwrap_err();
        assert!(errors.iter().any(|error| error.contains("public visibility requires granted consent")));
        assert!(errors.iter().any(|error| error.contains("publication-authority reference")));
    }

    #[test]
    fn exact_coordinates_are_blocked_from_public_and_private_payloads() {
        let mut profile = valid_public_profile();
        profile["access_policy"]["coordinate_disclosure"] = json!("exact");
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("public visibility cannot disclose exact coordinates")
        }));

        profile["access_policy"]["visibility"] = json!("private");
        profile["site_location"] = json!({"latitude": -26.2, "longitude": 28.0});
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("structured coordinate fields are forbidden")
        }));
    }

    #[test]
    fn synthetic_geometry_cannot_be_attested_as_observed_evidence() {
        let mut profile = valid_public_profile();
        profile["project_snapshot"]["geometry_status"] = json!("synthetic-benchmark");
        profile["evidence_class"] = json!("observed");
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("synthetic benchmark snapshot cannot be represented")
        }));
    }

    #[test]
    fn accepted_commitment_requires_an_acceptance_reference() {
        let mut profile = valid_public_profile();
        profile["record_type"] = json!("stewardship-commitment");
        profile["commitment"] = json!({
            "commitment_id": "maintenance-demo-001",
            "obligation": "Inspect the demonstration rain garden each month.",
            "responsible_party_reference": "role:maintainer-demo",
            "cadence": "monthly",
            "trigger": "Monthly schedule or visible damage after a storm.",
            "status": "accepted",
            "review_at": "2027-01-01T00:00:00Z",
            "resource_status": "unknown"
        });
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("accepted commitments require an acceptance reference")
        }));
        profile["commitment"]["acceptance_reference"] = json!("acceptance-record:demo");
        assert!(validate_profile_safety_gate(&profile).is_ok());
    }

    #[test]
    fn consent_withdrawal_cannot_remain_public() {
        let mut profile = valid_public_profile();
        profile["record_type"] = json!("consent-change");
        profile["consent_change"] = json!({
            "consent_reference": "consent-record:demo",
            "scope": "publication",
            "status": "withdrawn",
            "effective_at": "2026-10-21T01:00:00Z",
            "recording_authority_reference": "authority-record:demo",
            "supersedes_consent_reference": "consent-record:demo-v0"
        });
        profile["actor_authority"]["publication_consent"] = json!("withdrawn");
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("withdrawn or expired consent transitions")
        }));
    }

    #[test]
    fn scenario_review_rejects_scientific_claims_for_schematic_v0() {
        let mut profile = valid_public_profile();
        profile["record_type"] = json!("scenario-review");
        profile["scenario_reference"] = json!({
            "system": "symtropy",
            "manifest_sha256": SNAPSHOT_SHA256,
            "model_id": "landscape-scenario-v0",
            "model_version": "0.1.0",
            "purpose": "scientific-simulation",
            "evidence_class": "modelled"
        });
        assert!(validate_profile_safety_gate(&profile).unwrap_err().iter().any(|error| {
            error.contains("cannot authorize analytical or scientific simulation claims")
        }));
    }
}
