// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Runtime invariants for Mycelix passive-design provenance v2.
//!
//! This module intentionally does not replace the JSON Schema validator.
//! It mirrors the semantic acceptance boundary that must hold when an
//! application path accepts or promotes a provenance record:
//!
//! - exactly one claim for each mandatory passive field;
//! - field-specific value typing and non-negative numeric bounds;
//! - fail-closed status/value/source compatibility;
//! - Validated requires ValidationRecord;
//! - positive passive-score eligibility cannot outrun mandatory evidence.
//!
//! The JSON Schema remains the structural authority. This runtime validator
//! prevents an application path from accepting a record with semantics weaker
//! than the schema contract.

use serde_json::{Map, Value};

const MANDATORY_FIELDS: [&str; 5] = [
    "moving_solid_components",
    "mechanical_joints",
    "active_power_w",
    "commanded_actuators",
    "requires_external_control",
];

const ALLOWED_FIELDS: [&str; 8] = [
    "moving_solid_components",
    "mechanical_joints",
    "active_power_w",
    "commanded_actuators",
    "requires_external_control",
    "uses_fluid_motion",
    "uses_distributed_deformation",
    "uses_phase_change",
];

const ALLOWED_STATUSES: [&str; 6] = [
    "Unobserved",
    "Declared",
    "Simulated",
    "Verified",
    "Measured",
    "Validated",
];

const ALLOWED_SOURCE_KINDS: [&str; 6] = [
    "DesignDeclaration",
    "SimulationDeclaration",
    "ManufacturingDeclaration",
    "VerificationRecord",
    "MeasurementRecord",
    "ValidationRecord",
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PassiveProvenanceValidationError {
    RootMustBeObject,
    MissingTopLevelField(&'static str),
    InvalidSchemaVersion,
    EmptyArtifactDigest,
    EvidenceClaimsMustBeArray,
    EvidenceClaimMustBeObject { index: usize },
    MissingClaimField { index: usize, field: &'static str },
    UnknownClaimProperty { index: usize, property: String },
    UnknownClaimField { index: usize, field: String },
    UnknownClaimStatus { index: usize, status: String },
    UnknownClaimSourceKind { index: usize, source_kind: String },
    EmptyEvidenceDigest { index: usize },
    DuplicateMandatoryClaim { field: String },
    UnobservedClaimCarriesValue { index: usize },
    ObservedClaimMissingValue { index: usize, status: String },
    StatusSourceMismatch {
        index: usize,
        status: String,
        source_kind: String,
    },
    InvalidCountValue { index: usize, field: String },
    InvalidPowerValue { index: usize },
    InvalidBooleanValue { index: usize, field: String },
    DecisionMustBeObject,
    EligibilityMustBeBoolean,
    EligibilityOutrunsEvidence { field: &'static str },
}

pub fn validate_passive_design_provenance_v2(
    record: &Value,
) -> Result<(), PassiveProvenanceValidationError> {
    let root = record
        .as_object()
        .ok_or(PassiveProvenanceValidationError::RootMustBeObject)?;

    for field in [
        "schema_version",
        "artifact_digest",
        "passive_contract",
        "evidence_claims",
        "lineage",
        "decision",
    ] {
        if !root.contains_key(field) {
            return Err(PassiveProvenanceValidationError::MissingTopLevelField(field));
        }
    }

    match root.get("schema_version") {
        Some(Value::String(version)) if version == "passive-design-provenance-v2" => {}
        _ => return Err(PassiveProvenanceValidationError::InvalidSchemaVersion),
    }

    match root.get("artifact_digest") {
        Some(Value::String(value)) if !value.is_empty() => {}
        _ => return Err(PassiveProvenanceValidationError::EmptyArtifactDigest),
    }

    let claims = root
        .get("evidence_claims")
        .and_then(Value::as_array)
        .ok_or(PassiveProvenanceValidationError::EvidenceClaimsMustBeArray)?;

    let mut mandatory_seen = [false; MANDATORY_FIELDS.len()];
    for (index, claim) in claims.iter().enumerate() {
        validate_claim(index, claim, &mut mandatory_seen)?;
    }

    for (index, field) in MANDATORY_FIELDS.iter().enumerate() {
        if !mandatory_seen[index] {
            return Err(PassiveProvenanceValidationError::MissingClaimField {
                index: claims.len(),
                field,
            });
        }
    }

    let decision = root
        .get("decision")
        .and_then(Value::as_object)
        .ok_or(PassiveProvenanceValidationError::DecisionMustBeObject)?;

    let eligible = decision
        .get("eligible_for_passive_scoring")
        .and_then(Value::as_bool)
        .ok_or(PassiveProvenanceValidationError::EligibilityMustBeBoolean)?;

    if eligible {
        for field in MANDATORY_FIELDS {
            let claim = claims
                .iter()
                .find(|claim| claim.get("field").and_then(Value::as_str) == Some(field))
                .expect("mandatory claim existence checked above");
            if claim
                .get("status")
                .and_then(Value::as_str)
                .is_none_or(|status| status == "Unobserved")
            {
                return Err(
                    PassiveProvenanceValidationError::EligibilityOutrunsEvidence { field },
                );
            }
        }
    }

    Ok(())
}

fn validate_claim(
    index: usize,
    claim: &Value,
    mandatory_seen: &mut [bool; MANDATORY_FIELDS.len()],
) -> Result<(), PassiveProvenanceValidationError> {
    let object = claim
        .as_object()
        .ok_or(PassiveProvenanceValidationError::EvidenceClaimMustBeObject {
            index,
        })?;

    for property in object.keys() {
        if ![
            "field",
            "status",
            "source_kind",
            "evidence_digest",
            "value",
        ]
        .contains(&property.as_str())
        {
            return Err(PassiveProvenanceValidationError::UnknownClaimProperty {
                index,
                property: property.clone(),
            });
        }
    }

    let field = required_string(object, index, "field", |field| {
        ALLOWED_FIELDS.contains(&field)
    })?;

    let status = required_string(object, index, "status", |status| {
        ALLOWED_STATUSES.contains(&status)
    })?;

    let source_kind = required_string(object, index, "source_kind", |source| {
        ALLOWED_SOURCE_KINDS.contains(&source)
    })?;

    match object.get("evidence_digest") {
        Some(Value::String(value)) if !value.is_empty() => {}
        _ => return Err(PassiveProvenanceValidationError::EmptyEvidenceDigest { index }),
    }

    if let Some(mandatory_index) = MANDATORY_FIELDS.iter().position(|value| *value == field) {
        if mandatory_seen[mandatory_index] {
            return Err(
                PassiveProvenanceValidationError::DuplicateMandatoryClaim {
                    field: field.to_string(),
                },
            );
        }
        mandatory_seen[mandatory_index] = true;
    }

    let has_value = object.contains_key("value");
    if status == "Unobserved" {
        if has_value {
            return Err(
                PassiveProvenanceValidationError::UnobservedClaimCarriesValue { index },
            );
        }
    } else if !has_value {
        return Err(PassiveProvenanceValidationError::ObservedClaimMissingValue {
            index,
            status: status.to_string(),
        });
    }

    let source_is_allowed = match status {
        "Declared" => matches!(
            source_kind,
            "DesignDeclaration" | "ManufacturingDeclaration"
        ),
        "Simulated" => source_kind == "SimulationDeclaration",
        "Verified" => source_kind == "VerificationRecord",
        "Measured" => source_kind == "MeasurementRecord",
        "Validated" => source_kind == "ValidationRecord",
        "Unobserved" => true,
        _ => false,
    };

    if !source_is_allowed {
        return Err(PassiveProvenanceValidationError::StatusSourceMismatch {
            index,
            status: status.to_string(),
            source_kind: source_kind.to_string(),
        });
    }

    if let Some(value) = object.get("value") {
        match field {
            "moving_solid_components" | "mechanical_joints" | "commanded_actuators" => {
                if !is_non_negative_integer(value) {
                    return Err(PassiveProvenanceValidationError::InvalidCountValue {
                        index,
                        field: field.to_string(),
                    });
                }
            }
            "active_power_w" => {
                if !is_non_negative_number(value) {
                    return Err(PassiveProvenanceValidationError::InvalidPowerValue { index });
                }
            }
            "requires_external_control"
            | "uses_fluid_motion"
            | "uses_distributed_deformation"
            | "uses_phase_change" => {
                if !value.is_boolean() {
                    return Err(PassiveProvenanceValidationError::InvalidBooleanValue {
                        index,
                        field: field.to_string(),
                    });
                }
            }
            _ => {}
        }
    }

    Ok(())
}

fn required_string<'a, F>(
    object: &'a Map<String, Value>,
    index: usize,
    field_name: &'static str,
    allowed: F,
) -> Result<&'a str, PassiveProvenanceValidationError>
where
    F: Fn(&str) -> bool,
{
    let value = object
        .get(field_name)
        .and_then(Value::as_str)
        .ok_or(PassiveProvenanceValidationError::MissingClaimField {
            index,
            field: field_name,
        })?;

    if !allowed(value) {
        return Err(match field_name {
            "field" => PassiveProvenanceValidationError::UnknownClaimField {
                index,
                field: value.to_string(),
            },
            "status" => PassiveProvenanceValidationError::UnknownClaimStatus {
                index,
                status: value.to_string(),
            },
            "source_kind" => PassiveProvenanceValidationError::UnknownClaimSourceKind {
                index,
                source_kind: value.to_string(),
            },
            _ => unreachable!("required_string only handles claim discriminator fields"),
        });
    }

    Ok(value)
}

fn is_non_negative_integer(value: &Value) -> bool {
    if value.as_u64().is_some() {
        return true;
    }
    value
        .as_f64()
        .is_some_and(|value| value.is_finite() && value >= 0.0 && value.fract() == 0.0)
}

fn is_non_negative_number(value: &Value) -> bool {
    value
        .as_f64()
        .is_some_and(|value| value.is_finite() && value >= 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn valid_record() -> Value {
        json!({
            "schema_version": "passive-design-provenance-v2",
            "artifact_digest": "artifact-1",
            "passive_contract": {
                "contract_digest": "contract-1",
                "input": "Fluidic",
                "output": "Thermal",
                "mechanism": "Geometry"
            },
            "evidence_claims": [
                {
                    "field": "moving_solid_components",
                    "status": "Declared",
                    "source_kind": "DesignDeclaration",
                    "evidence_digest": "e1",
                    "value": 0
                },
                {
                    "field": "mechanical_joints",
                    "status": "Declared",
                    "source_kind": "DesignDeclaration",
                    "evidence_digest": "e2",
                    "value": 0
                },
                {
                    "field": "active_power_w",
                    "status": "Declared",
                    "source_kind": "ManufacturingDeclaration",
                    "evidence_digest": "e3",
                    "value": 0.0
                },
                {
                    "field": "commanded_actuators",
                    "status": "Declared",
                    "source_kind": "DesignDeclaration",
                    "evidence_digest": "e4",
                    "value": 0
                },
                {
                    "field": "requires_external_control",
                    "status": "Declared",
                    "source_kind": "DesignDeclaration",
                    "evidence_digest": "e5",
                    "value": false
                }
            ],
            "lineage": {
                "parent_artifact_digests": [],
                "transformation": "initial"
            },
            "decision": {
                "eligible_for_passive_scoring": true,
                "reasons": ["all mandatory fields observed"]
            }
        })
    }

    #[test]
    fn valid_record_passes() {
        assert!(validate_passive_design_provenance_v2(&valid_record()).is_ok());
    }

    #[test]
    fn committed_schema_contains_runtime_invariants() {
        let schema: Value = serde_json::from_str(include_str!(
            "../../../docs/design/passive-design-provenance-v2.schema.json"
        ))
        .expect("passive provenance schema must remain valid JSON");

        assert_eq!(
            schema["$schema"],
            "https://json-schema.org/draft/2020-12/schema"
        );
        assert_eq!(
            schema["properties"]["schema_version"]["const"],
            "passive-design-provenance-v2"
        );
        assert_eq!(schema["properties"]["evidence_claims"]["minItems"], 5);
        assert!(
            schema["$defs"]["evidence_claim"]["properties"]["value"]
                .get("anyOf")
                .is_some(),
            "schema value union must be inclusive"
        );
        assert!(
            schema["$defs"]["evidence_claim"]["properties"]["value"]
                .get("oneOf")
                .is_none(),
            "schema must not use overlapping oneOf numeric types"
        );

        let claim_rules = schema["properties"]["evidence_claims"]["allOf"]
            .as_array()
            .expect("evidence_claims must expose occurrence rules");

        for field in MANDATORY_FIELDS {
            let matching = claim_rules.iter().any(|rule| {
                rule["contains"]["properties"]["field"]["const"] == field
                    && rule["minContains"] == 1
                    && rule["maxContains"] == 1
            });
            assert!(matching, "schema lost exact-one rule for {field}");
        }

        let evidence_claim = &schema["$defs"]["evidence_claim"];
        let source_kinds = evidence_claim["properties"]["source_kind"]["enum"]
            .as_array()
            .expect("source_kind must remain an enum");
        assert!(source_kinds.iter().any(|value| value == "ValidationRecord"));

        let status_rules = evidence_claim["allOf"]
            .as_array()
            .expect("evidence_claim must expose status/source rules");

        let expected_sources = [
            ("Declared", "DesignDeclaration"),
            ("Simulated", "SimulationDeclaration"),
            ("Verified", "VerificationRecord"),
            ("Measured", "MeasurementRecord"),
            ("Validated", "ValidationRecord"),
        ];

        for (status, source_kind) in expected_sources {
            let matching = status_rules.iter().any(|rule| {
                rule["if"]["properties"]["status"]["const"] == status
                    && rule["then"]["properties"]["source_kind"]["const"] == source_kind
            });
            assert!(matching, "schema lost {status} -> {source_kind} mapping");
        }

        let eligibility_gate = schema["allOf"]
            .as_array()
            .expect("schema must expose positive-eligibility gate")
            .iter()
            .any(|rule| {
                rule["if"]["properties"]["decision"]["properties"]
                    ["eligible_for_passive_scoring"]["const"]
                    == true
            });
        assert!(eligibility_gate, "schema lost positive eligibility guard");
    }

    #[test]
    fn duplicate_mandatory_claim_fails_closed() {
        let mut record = valid_record();
        let first = record["evidence_claims"][0].clone();
        record["evidence_claims"].as_array_mut().unwrap().push(first);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::DuplicateMandatoryClaim { field })
                if field == "moving_solid_components"
        ));
    }

    #[test]
    fn missing_mandatory_claim_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"]
            .as_array_mut()
            .unwrap()
            .retain(|claim| claim["field"] != "mechanical_joints");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::MissingClaimField {
                field: "mechanical_joints",
                ..
            })
        ));
    }

    #[test]
    fn integral_json_number_is_valid_count() {
        let mut record = valid_record();
        record["evidence_claims"][0]["value"] = json!(1.0);
        assert!(validate_passive_design_provenance_v2(&record).is_ok());
    }

    #[test]
    fn wrong_count_type_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["value"] = json!(0.5);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::InvalidCountValue { .. })
        ));
    }

    #[test]
    fn negative_numeric_value_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["value"] = json!(-1);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::InvalidCountValue { .. })
        ));

        record["evidence_claims"][0]["value"] = json!(1);
        record["evidence_claims"][2]["value"] = json!(-0.1);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::InvalidPowerValue { .. })
        ));
    }

    #[test]
    fn wrong_boolean_type_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][4]["value"] = json!(0);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::InvalidBooleanValue { .. })
        ));
    }

    #[test]
    fn unobserved_value_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["status"] = json!("Unobserved");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::UnobservedClaimCarriesValue { .. })
        ));
    }

    #[test]
    fn stronger_status_without_value_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0].as_object_mut().unwrap().remove("value");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::ObservedClaimMissingValue { .. })
        ));
    }

    #[test]
    fn status_source_mismatch_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["status"] = json!("Verified");
        record["evidence_claims"][0]["source_kind"] = json!("DesignDeclaration");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::StatusSourceMismatch { .. })
        ));
    }

    #[test]
    fn validated_requires_validation_record() {
        let mut record = valid_record();
        record["evidence_claims"][4]["status"] = json!("Validated");
        record["evidence_claims"][4]["source_kind"] = json!("MeasurementRecord");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::StatusSourceMismatch { .. })
        ));

        record["evidence_claims"][4]["source_kind"] = json!("ValidationRecord");
        assert!(validate_passive_design_provenance_v2(&record).is_ok());
    }

    #[test]
    fn eligibility_cannot_outpace_unobserved_claim() {
        let mut record = valid_record();
        record["evidence_claims"][0]["status"] = json!("Unobserved");
        record["evidence_claims"][0].as_object_mut().unwrap().remove("value");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::EligibilityOutrunsEvidence {
                field: "moving_solid_components"
            })
        ));
    }

    #[test]
    fn optional_mechanism_claims_can_be_repeated() {
        let mut record = valid_record();
        let claim = json!({
            "field": "uses_fluid_motion",
            "status": "Declared",
            "source_kind": "DesignDeclaration",
            "evidence_digest": "fluid-1",
            "value": true
        });
        record["evidence_claims"].as_array_mut().unwrap().push(claim.clone());
        record["evidence_claims"].as_array_mut().unwrap().push(claim);
        assert!(validate_passive_design_provenance_v2(&record).is_ok());
    }

    #[test]
    fn extra_claim_property_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["unexpected"] = json!(true);
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::UnknownClaimProperty { .. })
        ));
    }

    #[test]
    fn wrong_source_kind_name_fails_closed() {
        let mut record = valid_record();
        record["evidence_claims"][0]["source_kind"] = json!("Measurement");
        assert!(matches!(
            validate_passive_design_provenance_v2(&record),
            Err(PassiveProvenanceValidationError::UnknownClaimSourceKind { .. })
        ));
    }
}
