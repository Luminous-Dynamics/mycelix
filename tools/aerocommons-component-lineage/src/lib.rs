//! A bounded, synthetic AeroCommons build-packet verifier.
//!
//! A successful result means only that declared records are structurally
//! complete and consistently linked. It does not qualify a physical part.

use aero_evidence_types::{
    validate_evidence, AeroEvidenceV1, ArtifactRef, ClaimKind, EvidenceLifecycle,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

pub const PACKET_SCHEMA: &str = "AeroComponentBuildPacketV1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContentArtifactV1 {
    pub reference: ArtifactRef,
    pub sha256: String,
    pub byte_length: u64,
    pub media_type: String,
    pub format_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationValidityV1 {
    pub evidence: ArtifactRef,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
    pub used_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentBuildPacketV1 {
    pub schema: String,
    pub packet_id: String,
    pub component: ArtifactRef,
    pub physical_unit: ArtifactRef,
    pub configuration: ArtifactRef,
    pub design_definition: ContentArtifactV1,
    pub requirements: Vec<ArtifactRef>,
    pub process_plan: ContentArtifactV1,
    pub material_batch: ArtifactRef,
    pub machine: ArtifactRef,
    pub tooling: Vec<ArtifactRef>,
    pub build_evidence: AeroEvidenceV1,
    pub inspection_plan: ArtifactRef,
    pub inspection_instrument: ArtifactRef,
    pub inspection_calibration: AeroEvidenceV1,
    pub inspection_calibration_validity: CalibrationValidityV1,
    pub inspection_result: ContentArtifactV1,
    pub inspection_evidence: AeroEvidenceV1,
    pub test_plan: ArtifactRef,
    pub test_instrument: ArtifactRef,
    pub test_calibration: AeroEvidenceV1,
    pub test_calibration_validity: CalibrationValidityV1,
    pub test_result: ContentArtifactV1,
    pub test_evidence: AeroEvidenceV1,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum PacketDisposition {
    StructurallyComplete,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationReport {
    pub disposition: PacketDisposition,
    pub packet_id: String,
    pub evidence_records: usize,
    pub content_artifacts: usize,
    pub required_references: usize,
    pub artifact_payloads_verified: bool,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum PacketError {
    #[error("packet schema is not AeroComponentBuildPacketV1")]
    WrongSchema,
    #[error("packet, component, or configuration identity is empty or malformed")]
    InvalidIdentity,
    #[error("required list is empty: {0}")]
    EmptyRequiredList(&'static str),
    #[error("duplicate identity: {0}")]
    DuplicateIdentity(String),
    #[error("content artifact is invalid: {0}")]
    InvalidContentArtifact(String),
    #[error("content artifact has unexpected format or reference kind: {0}")]
    WrongContentArtifactKind(String),
    #[error("evidence is invalid: {0}")]
    InvalidEvidence(String),
    #[error("evidence has an unexpected claim kind: {0}")]
    WrongClaimKind(String),
    #[error("evidence is not active: {0}")]
    EvidenceNotActive(String),
    #[error("evidence subject does not match the expected subject: {0}")]
    WrongEvidenceSubject(String),
    #[error("evidence configuration does not match the packet: {0}")]
    WrongEvidenceConfiguration(String),
    #[error("evidence is missing required reference {reference_id} ({reference_kind}): {evidence_id}")]
    MissingEvidenceReference {
        evidence_id: String,
        reference_id: String,
        reference_kind: String,
    },
    #[error("evidence method does not match the required method: {0}")]
    WrongEvidenceMethod(String),
    #[error("calibration evidence is not valid at the linked measurement time: {0}")]
    CalibrationWindowInvalid(String),
    #[error("test event precedes the inspection event")]
    NonMonotonicTestTime,
    #[error("artifact byte length does not match declared length: {0}")]
    ArtifactByteLengthMismatch(String),
    #[error("artifact bytes do not match declared SHA-256: {0}")]
    ArtifactDigestMismatch(String),
    #[error("invalid packet JSON: {0}")]
    InvalidPacketJson(String),
    #[error("unknown field in {scope}: {field}")]
    UnknownField { scope: String, field: String },
    #[error("malformed JSON value at {0}")]
    MalformedPacketField(String),
    #[error("missing payload for artifact: {0}")]
    MissingArtifactPayload(String),
    #[error("duplicate payload for artifact: {0}")]
    DuplicateArtifactPayload(String),
    #[error("unexpected artifact payload: {0}")]
    UnexpectedArtifactPayload(String),
}

fn check_object_fields(
    value: &serde_json::Value,
    allowed: &[&str],
    scope: &str,
) -> Result<(), PacketError> {
    let object = value
        .as_object()
        .ok_or_else(|| PacketError::MalformedPacketField(scope.to_owned()))?;
    for field in object.keys() {
        if !allowed.contains(&field.as_str()) {
            return Err(PacketError::UnknownField {
                scope: scope.to_owned(),
                field: field.clone(),
            });
        }
    }
    Ok(())
}

fn check_ref_value(value: &serde_json::Value, scope: &str) -> Result<(), PacketError> {
    check_object_fields(value, &["id", "kind"], scope)?;
    check_required_fields(value, &["id", "kind"], scope)
}

fn check_required_fields(
    value: &serde_json::Value,
    required: &[&str],
    scope: &str,
) -> Result<(), PacketError> {
    let object = value
        .as_object()
        .ok_or_else(|| PacketError::MalformedPacketField(scope.to_owned()))?;
    for field in required {
        if !object.contains_key(*field) {
            return Err(PacketError::MalformedPacketField(format!("{scope}.{field}")));
        }
    }
    Ok(())
}

fn check_ref_value(value: &serde_json::Value, scope: &str) -> Result<(), PacketError> {
    check_object_fields(value, &["id", "kind"], scope)?;
    check_required_fields(value, &["id", "kind"], scope)
}

fn check_ref_array(value: &serde_json::Value, scope: &str) -> Result<(), PacketError> {
    let array = value
        .as_array()
        .ok_or_else(|| PacketError::MalformedPacketField(scope.to_owned()))?;
    for (index, item) in array.iter().enumerate() {
        check_ref_value(item, &format!("{scope}[{index}]"))?;
    }
    Ok(())
}

fn check_content_value(value: &serde_json::Value, scope: &str) -> Result<(), PacketError> {
    check_object_fields(
        value,
        &["reference", "sha256", "byte_length", "media_type", "format_id"],
        scope,
    )?;
    check_required_fields(value, &["reference", "sha256", "byte_length", "media_type", "format_id"], scope)?;
    let reference = value
        .get("reference")
        .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.reference")))?;
    check_ref_value(reference, &format!("{scope}.reference"))
}

fn check_calibration_validity_value(
    value: &serde_json::Value,
    scope: &str,
) -> Result<(), PacketError> {
    check_object_fields(
        value,
        &["evidence", "valid_from_unix_ms", "valid_until_unix_ms", "used_at_unix_ms"],
        scope,
    )?;
    check_required_fields(value, &["evidence", "valid_from_unix_ms", "valid_until_unix_ms", "used_at_unix_ms"], scope)?;
    let evidence = value
        .get("evidence")
        .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.evidence")))?;
    check_ref_value(evidence, &format!("{scope}.evidence"))
}

fn check_evidence_value(value: &serde_json::Value, scope: &str) -> Result<(), PacketError> {
    check_object_fields(
        value,
        &[
            "evidence_id",
            "schema",
            "claim",
            "claim_kind",
            "configuration",
            "subject",
            "basis",
            "method",
            "inputs",
            "toolchain",
            "result",
            "uncertainty",
            "validity_domain",
            "producer",
            "review",
            "lifecycle",
            "provenance",
        ],
        scope,
    )?;
    check_required_fields(
        value,
        &[
            "evidence_id", "schema", "claim", "claim_kind", "configuration", "subject",
            "basis", "method", "inputs", "toolchain", "result", "uncertainty",
            "validity_domain", "producer", "review", "lifecycle", "provenance",
        ],
        scope,
    )?;
    for field in ["configuration", "subject"] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.{field}")))?;
        check_ref_value(nested, &format!("{scope}.{field}"))?;
    }
    if let Some(method) = value.get("method") {
        if !method.is_null() {
            check_ref_value(method, &format!("{scope}.method"))?;
        }
    }
    for field in ["basis", "inputs", "toolchain", "review", "provenance"] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.{field}")))?;
        check_ref_array(nested, &format!("{scope}.{field}"))?;
    }

    if let Some(uncertainty) = value.get("uncertainty") {
        if !uncertainty.is_null() {
            check_object_fields(
                uncertainty,
                &["representation", "value", "unit"],
                &format!("{scope}.uncertainty"),
            )?;
            check_required_fields(
                uncertainty,
                &["representation", "value", "unit"],
                &format!("{scope}.uncertainty"),
            )?;
        }
    }
    let validity = value
        .get("validity_domain")
        .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.validity_domain")))?;
    check_object_fields(validity, &["conditions"], &format!("{scope}.validity_domain"))?;
    check_required_fields(validity, &["conditions"], &format!("{scope}.validity_domain"))?;
    let conditions = validity
        .get("conditions")
        .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.validity_domain.conditions")))?;
    if !conditions.is_object() {
        return Err(PacketError::MalformedPacketField(format!(
            "{scope}.validity_domain.conditions"
        )));
    }
    let lifecycle = value
        .get("lifecycle")
        .ok_or_else(|| PacketError::MalformedPacketField(format!("{scope}.lifecycle")))?;
    check_object_fields(
        lifecycle,
        &["state", "reason", "predecessor"],
        &format!("{scope}.lifecycle"),
    )?;
    check_required_fields(
        lifecycle,
        &["state", "reason", "predecessor"],
        &format!("{scope}.lifecycle"),
    )
}

fn check_packet_json_contract(value: &serde_json::Value) -> Result<(), PacketError> {
    const ROOT_FIELDS: &[&str] = &[
        "schema", "packet_id", "component", "physical_unit", "configuration",
        "design_definition", "requirements", "process_plan", "material_batch",
        "machine", "tooling", "build_evidence", "inspection_plan",
        "inspection_instrument", "inspection_calibration",
        "inspection_calibration_validity", "inspection_result",
        "inspection_evidence", "test_plan", "test_instrument",
        "test_calibration", "test_calibration_validity", "test_result",
        "test_evidence",
    ];
    check_object_fields(value, ROOT_FIELDS, "packet")?;
    check_required_fields(value, ROOT_FIELDS, "packet")?;

    for field in [
        "component", "physical_unit", "configuration", "material_batch", "machine",
        "inspection_plan", "inspection_instrument", "test_plan", "test_instrument",
    ] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("packet.{field}")))?;
        check_ref_value(nested, &format!("packet.{field}"))?;
    }
    for field in ["requirements", "tooling"] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("packet.{field}")))?;
        check_ref_array(nested, &format!("packet.{field}"))?;
    }
    for field in ["design_definition", "process_plan", "inspection_result", "test_result"] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("packet.{field}")))?;
        check_content_value(nested, &format!("packet.{field}"))?;
    }
    for field in [
        "build_evidence", "inspection_calibration", "inspection_evidence",
        "test_calibration", "test_evidence",
    ] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("packet.{field}")))?;
        check_evidence_value(nested, &format!("packet.{field}"))?;
    }
    for field in ["inspection_calibration_validity", "test_calibration_validity"] {
        let nested = value
            .get(field)
            .ok_or_else(|| PacketError::MalformedPacketField(format!("packet.{field}")))?;
        check_calibration_validity_value(nested, &format!("packet.{field}"))?;
    }
    Ok(())
}

/// Strict JSON admission: rejects unknown fields, including nested fields in
/// the upstream AeroEvidenceV1 type which intentionally remains reusable.
pub fn parse_packet_json(bytes: &[u8]) -> Result<ComponentBuildPacketV1, PacketError> {
    let value: serde_json::Value = serde_json::from_slice(bytes)
        .map_err(|error| PacketError::InvalidPacketJson(error.to_string()))?;
    check_packet_json_contract(&value)?;
    serde_json::from_value(value)
        .map_err(|error| PacketError::InvalidPacketJson(error.to_string()))
}

fn valid_ref(reference: &ArtifactRef) -> bool {
    !reference.id.trim().is_empty() && !reference.kind.trim().is_empty()
}

fn require_member(
    references: &[ArtifactRef],
    evidence_id: &str,
    required: &ArtifactRef,
) -> Result<(), PacketError> {
    if references.iter().any(|item| item == required) {
        Ok(())
    } else {
        Err(PacketError::MissingEvidenceReference {
            evidence_id: evidence_id.to_owned(),
            reference_id: required.id.clone(),
            reference_kind: required.kind.clone(),
        })
    }
}

fn require_input(evidence: &AeroEvidenceV1, required: &ArtifactRef) -> Result<(), PacketError> {
    require_member(&evidence.inputs, &evidence.evidence_id, required)
}

fn require_basis(evidence: &AeroEvidenceV1, required: &ArtifactRef) -> Result<(), PacketError> {
    require_member(&evidence.basis, &evidence.evidence_id, required)
}

fn require_toolchain(evidence: &AeroEvidenceV1, required: &ArtifactRef) -> Result<(), PacketError> {
    require_member(&evidence.toolchain, &evidence.evidence_id, required)
}

/// Verify an artifact's declared length and SHA-256 against its exact bytes.
///
/// This is separate from packet structure validation because artifact payloads
/// are stored outside the packet and must be fetched by an explicit caller.
pub fn verify_artifact_bytes(
    artifact: &ContentArtifactV1,
    bytes: &[u8],
) -> Result<(), PacketError> {
    if bytes.len() as u64 != artifact.byte_length {
        return Err(PacketError::ArtifactByteLengthMismatch(
            artifact.reference.id.clone(),
        ));
    }
    let digest = Sha256::digest(bytes);
    let actual = digest.iter().map(|byte| format!("{byte:02x}")).collect::<String>();
    if actual != artifact.sha256 || artifact.reference.id != format!("sha256:{actual}") {
        return Err(PacketError::ArtifactDigestMismatch(
            artifact.reference.id.clone(),
        ));
    }
    Ok(())
}

fn validate_content_artifact(
    artifact: &ContentArtifactV1,
    expected_kind: &str,
    expected_format: &str,
    label: &str,
) -> Result<(), PacketError> {
    let digest_is_valid = artifact.sha256.len() == 64
        && artifact
            .sha256
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase());

    if !valid_ref(&artifact.reference)
        || !digest_is_valid
        || artifact.byte_length == 0
        || artifact.media_type.trim().is_empty()
        || artifact.format_id.trim().is_empty()
        || artifact.reference.id != format!("sha256:{}", artifact.sha256)
    {
        return Err(PacketError::InvalidContentArtifact(label.to_owned()));
    }

    if artifact.reference.kind != expected_kind || artifact.format_id != expected_format {
        return Err(PacketError::WrongContentArtifactKind(label.to_owned()));
    }
    Ok(())
}

fn validate_evidence_record(
    evidence: &AeroEvidenceV1,
    expected_kind: ClaimKind,
    expected_subject: &ArtifactRef,
    expected_configuration: Option<&ArtifactRef>,
) -> Result<(), PacketError> {
    validate_evidence(evidence)
        .map_err(|error| PacketError::InvalidEvidence(format!("{}: {error}", evidence.evidence_id)))?;

    if evidence.claim_kind != expected_kind {
        return Err(PacketError::WrongClaimKind(evidence.evidence_id.clone()));
    }
    if evidence.lifecycle.state != EvidenceLifecycle::Active {
        return Err(PacketError::EvidenceNotActive(evidence.evidence_id.clone()));
    }
    if evidence.subject != *expected_subject {
        return Err(PacketError::WrongEvidenceSubject(evidence.evidence_id.clone()));
    }
    if let Some(configuration) = expected_configuration {
        if evidence.configuration != *configuration {
            return Err(PacketError::WrongEvidenceConfiguration(
                evidence.evidence_id.clone(),
            ));
        }
    }
    Ok(())
}

fn require_method(
    evidence: &AeroEvidenceV1,
    expected: &ArtifactRef,
) -> Result<(), PacketError> {
    if evidence.method.as_ref() == Some(expected) {
        Ok(())
    } else {
        Err(PacketError::WrongEvidenceMethod(evidence.evidence_id.clone()))
    }
}

fn ensure_unique(values: &[ArtifactRef]) -> Result<(), PacketError> {
    let mut seen = std::collections::BTreeSet::new();
    for value in values {
        if !valid_ref(value) || !seen.insert((value.id.clone(), value.kind.clone())) {
            return Err(PacketError::DuplicateIdentity(value.id.clone()));
        }
    }
    Ok(())
}

fn evidence_ref(evidence: &AeroEvidenceV1) -> ArtifactRef {
    ArtifactRef {
        id: evidence.evidence_id.clone(),
        kind: "evidence".to_owned(),
    }
}

fn validate_calibration_window(
    window: &CalibrationValidityV1,
    evidence: &AeroEvidenceV1,
) -> Result<(), PacketError> {
    if window.evidence != evidence_ref(evidence)
        || window.valid_from_unix_ms >= window.valid_until_unix_ms
        || window.used_at_unix_ms < window.valid_from_unix_ms
        || window.used_at_unix_ms >= window.valid_until_unix_ms
    {
        return Err(PacketError::CalibrationWindowInvalid(
            evidence.evidence_id.clone(),
        ));
    }
    Ok(())
}

/// Validate the packet's structural identity and evidence joins.
///
/// This function never decides whether the part passed inspection or testing.
pub fn validate_packet(
    packet: &ComponentBuildPacketV1,
) -> Result<ValidationReport, PacketError> {
    if packet.schema != PACKET_SCHEMA {
        return Err(PacketError::WrongSchema);
    }
    if packet.packet_id.trim().is_empty()
        || !valid_ref(&packet.component)
        || !valid_ref(&packet.physical_unit)
        || !valid_ref(&packet.configuration)
        || packet.component.kind != "component"
        || packet.physical_unit.kind != "physical_unit"
        || packet.physical_unit.id == packet.component.id
        || packet.configuration.kind != "configuration"
        || !valid_ref(&packet.material_batch)
        || packet.material_batch.kind != "material_batch"
        || !valid_ref(&packet.machine)
        || packet.machine.kind != "machine"
        || !valid_ref(&packet.inspection_plan)
        || !valid_ref(&packet.inspection_instrument)
        || !valid_ref(&packet.test_plan)
        || !valid_ref(&packet.test_instrument)
    {
        return Err(PacketError::InvalidIdentity);
    }
    if packet.requirements.is_empty() {
        return Err(PacketError::EmptyRequiredList("requirements"));
    }
    if packet.tooling.is_empty() {
        return Err(PacketError::EmptyRequiredList("tooling"));
    }
    ensure_unique(&packet.requirements)?;
    ensure_unique(&packet.tooling)?;
    if packet.requirements.iter().any(|item| item.kind != "requirement")
        || packet.tooling.iter().any(|item| item.kind != "tooling")
    {
        return Err(PacketError::InvalidIdentity);
    }

    validate_content_artifact(
        &packet.design_definition,
        "design_definition",
        "ISO_10303_242:2025",
        "design_definition",
    )?;
    validate_content_artifact(
        &packet.process_plan,
        "manufacturing_process_plan",
        "AERO_PROCESS_PLAN_V1",
        "process_plan",
    )?;
    validate_content_artifact(
        &packet.inspection_result,
        "qif_result",
        "ISO_23952:2020",
        "inspection_result",
    )?;
    validate_content_artifact(
        &packet.test_result,
        "test_result",
        "AERO_TEST_RESULT_V1",
        "test_result",
    )?;

    let evidence_records = [
        &packet.build_evidence,
        &packet.inspection_calibration,
        &packet.inspection_evidence,
        &packet.test_calibration,
        &packet.test_evidence,
    ];
    let mut evidence_ids = std::collections::BTreeSet::new();
    for evidence in evidence_records {
        if !evidence_ids.insert(evidence.evidence_id.clone()) {
            return Err(PacketError::DuplicateIdentity(evidence.evidence_id.clone()));
        }
    }

    validate_evidence_record(
        &packet.build_evidence,
        ClaimKind::Observation,
        &packet.physical_unit,
        Some(&packet.configuration),
    )?;
    require_method(&packet.build_evidence, &packet.process_plan.reference)?;
    require_input(&packet.build_evidence, &packet.component)?;
    require_input(&packet.build_evidence, &packet.design_definition.reference)?;
    require_input(&packet.build_evidence, &packet.process_plan.reference)?;
    require_input(&packet.build_evidence, &packet.material_batch)?;
    for requirement in &packet.requirements {
        if require_basis(&packet.build_evidence, requirement).is_err() {
            return Err(PacketError::MissingEvidenceReference {
                evidence_id: packet.build_evidence.evidence_id.clone(),
                reference_id: requirement.id.clone(),
                reference_kind: requirement.kind.clone(),
            });
        }
    }
    require_toolchain(&packet.build_evidence, &packet.machine)?;
    for tooling in &packet.tooling {
        require_toolchain(&packet.build_evidence, tooling)?;
    }

    validate_evidence_record(
        &packet.inspection_calibration,
        ClaimKind::Qualification,
        &packet.inspection_instrument,
        None,
    )?;
    validate_calibration_window(
        &packet.inspection_calibration_validity,
        &packet.inspection_calibration,
    )?;
    validate_evidence_record(
        &packet.inspection_evidence,
        ClaimKind::Inspection,
        &packet.physical_unit,
        Some(&packet.configuration),
    )?;
    require_method(&packet.inspection_evidence, &packet.inspection_plan)?;
    for required in [
        evidence_ref(&packet.build_evidence),
        packet.inspection_plan.clone(),
        packet.inspection_instrument.clone(),
        evidence_ref(&packet.inspection_calibration),
        packet.inspection_result.reference.clone(),
    ] {
        require_input(&packet.inspection_evidence, &required)?;
    }

    validate_evidence_record(
        &packet.test_calibration,
        ClaimKind::Qualification,
        &packet.test_instrument,
        None,
    )?;
    validate_calibration_window(&packet.test_calibration_validity, &packet.test_calibration)?;
    if packet.test_calibration_validity.used_at_unix_ms
        < packet.inspection_calibration_validity.used_at_unix_ms
    {
        return Err(PacketError::NonMonotonicTestTime);
    }
    validate_evidence_record(
        &packet.test_evidence,
        ClaimKind::Test,
        &packet.physical_unit,
        Some(&packet.configuration),
    )?;
    require_method(&packet.test_evidence, &packet.test_plan)?;
    for required in [
        evidence_ref(&packet.build_evidence),
        evidence_ref(&packet.inspection_evidence),
        packet.test_plan.clone(),
        packet.test_instrument.clone(),
        evidence_ref(&packet.test_calibration),
        packet.test_result.reference.clone(),
    ] {
        require_input(&packet.test_evidence, &required)?;
    }

    let required_references = packet.requirements.len() + packet.tooling.len() + 16;
    Ok(ValidationReport {
        disposition: PacketDisposition::StructurallyComplete,
        packet_id: packet.packet_id.clone(),
        evidence_records: evidence_records.len(),
        content_artifacts: 4,
        required_references,
        artifact_payloads_verified: false,
    })
}

/// Validate the packet and verify the exact bytes for all four referenced artifacts.
///
/// The caller is responsible for obtaining payloads from an authorized source.
/// This function verifies byte length and SHA-256, not source authenticity,
/// licensing, or the truth of the engineering content.
pub fn verify_packet_artifact_payloads(
    packet: &ComponentBuildPacketV1,
    payloads: &[(&ContentArtifactV1, &[u8])],
) -> Result<ValidationReport, PacketError> {
    let mut report = validate_packet(packet)?;
    let expected = [
        &packet.design_definition,
        &packet.process_plan,
        &packet.inspection_result,
        &packet.test_result,
    ];

    if payloads.len() != expected.len() {
        return Err(PacketError::MissingArtifactPayload(
            "packet must contain exactly four artifact payloads".to_owned(),
        ));
    }

    let mut seen = std::collections::BTreeSet::new();
    for (provided_artifact, bytes) in payloads {
        let key = (
            provided_artifact.reference.id.clone(),
            provided_artifact.reference.kind.clone(),
        );
        if !seen.insert(key.clone()) {
            return Err(PacketError::DuplicateArtifactPayload(key.0));
        }
        let Some(expected_artifact) = expected.iter().find(|item| {
            item.reference == provided_artifact.reference
        }) else {
            return Err(PacketError::UnexpectedArtifactPayload(key.0));
        };
        verify_artifact_bytes(expected_artifact, bytes)?;
    }

    for artifact in expected {
        if !seen.contains(&(
            artifact.reference.id.clone(),
            artifact.reference.kind.clone(),
        )) {
            return Err(PacketError::MissingArtifactPayload(
                artifact.reference.id.clone(),
            ));
        }
    }

    report.artifact_payloads_verified = true;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use aero_evidence_types::{LifecycleEvent, ValidityDomain};
    use serde_json::json;

    fn reference(id: &str, kind: &str) -> ArtifactRef {
        ArtifactRef { id: id.into(), kind: kind.into() }
    }

    fn content(seed: u64, kind: &str, format: &str, media: &str) -> ContentArtifactV1 {
        let sha256 = format!("{:064x}", seed);
        ContentArtifactV1 {
            reference: reference(&format!("sha256:{sha256}"), kind),
            sha256,
            byte_length: 128,
            media_type: media.into(),
            format_id: format.into(),
        }
    }

    fn evidence(
        id: &str,
        claim_kind: ClaimKind,
        subject: ArtifactRef,
        configuration: ArtifactRef,
        method: Option<ArtifactRef>,
        inputs: Vec<ArtifactRef>,
        basis: Vec<ArtifactRef>,
        toolchain: Vec<ArtifactRef>,
    ) -> AeroEvidenceV1 {
        AeroEvidenceV1 {
            evidence_id: id.into(),
            schema: "AeroEvidenceV1".into(),
            claim: format!("Synthetic evidence record {id}"),
            claim_kind,
            configuration,
            subject,
            basis,
            method,
            inputs,
            toolchain,
            result: json!({"synthetic": true, "value": 1}),
            uncertainty: None,
            validity_domain: ValidityDomain::default(),
            producer: "synthetic-lab-agent".into(),
            review: vec![],
            lifecycle: LifecycleEvent {
                state: EvidenceLifecycle::Active,
                reason: None,
                predecessor: None,
            },
            provenance: vec![],
        }
    }

    fn fixture() -> ComponentBuildPacketV1 {
        let component = reference("component:coupon-001", "component");
        let physical_unit = reference("physical-unit:coupon-001-serial-0001", "physical_unit");
        let configuration = reference("configuration:coupon-001-r1", "configuration");
        let design = content(1, "design_definition", "ISO_10303_242:2025", "application/step");
        let process = content(2, "manufacturing_process_plan", "AERO_PROCESS_PLAN_V1", "application/json");
        let inspection_result = content(3, "qif_result", "ISO_23952:2020", "application/xml");
        let test_result = content(4, "test_result", "AERO_TEST_RESULT_V1", "application/json");
        let requirement_a = reference("requirement:dimension-a", "requirement");
        let requirement_b = reference("requirement:surface-finish", "requirement");
        let material = reference("material-batch:al-6061-lot-01", "material_batch");
        let machine = reference("machine:mill-01", "machine");
        let tooling = reference("tooling:fixture-01", "tooling");
        let inspection_plan = reference("inspection-plan:coupon-v1", "inspection_plan");
        let inspection_instrument = reference("instrument:cmm-01", "instrument");
        let test_plan = reference("test-plan:coupon-v1", "test_plan");
        let test_instrument = reference("instrument:load-cell-01", "instrument");

        let build = evidence(
            "evidence:build-01",
            ClaimKind::Observation,
            physical_unit.clone(),
            configuration.clone(),
            Some(process.reference.clone()),
            vec![component.clone(), design.reference.clone(), process.reference.clone(), material.clone()],
            vec![requirement_a.clone(), requirement_b.clone()],
            vec![machine.clone(), tooling.clone()],
        );
        let inspection_calibration = evidence(
            "evidence:cal-inspection-01",
            ClaimKind::Qualification,
            inspection_instrument.clone(),
            reference("calibration-profile:cmm-01", "calibration_profile"),
            Some(reference("method:calibration-check-v1", "method")),
            vec![],
            vec![reference("requirement:calibration-traceability", "requirement")],
            vec![reference("lab:metrology-01", "laboratory")],
        );
        let inspection = evidence(
            "evidence:inspection-01",
            ClaimKind::Inspection,
            physical_unit.clone(),
            configuration.clone(),
            Some(inspection_plan.clone()),
            vec![
                evidence_ref(&build),
                inspection_plan.clone(),
                inspection_instrument.clone(),
                evidence_ref(&inspection_calibration),
                inspection_result.reference.clone(),
            ],
            vec![],
            vec![inspection_instrument.clone()],
        );
        let test_calibration = evidence(
            "evidence:cal-test-01",
            ClaimKind::Qualification,
            test_instrument.clone(),
            reference("calibration-profile:load-cell-01", "calibration_profile"),
            Some(reference("method:calibration-check-v1", "method")),
            vec![],
            vec![reference("requirement:calibration-traceability", "requirement")],
            vec![reference("lab:metrology-01", "laboratory")],
        );
        let test = evidence(
            "evidence:test-01",
            ClaimKind::Test,
            physical_unit.clone(),
            configuration.clone(),
            Some(test_plan.clone()),
            vec![
                evidence_ref(&build),
                evidence_ref(&inspection),
                test_plan.clone(),
                test_instrument.clone(),
                evidence_ref(&test_calibration),
                test_result.reference.clone(),
            ],
            vec![],
            vec![test_instrument.clone()],
        );

        ComponentBuildPacketV1 {
            schema: PACKET_SCHEMA.into(),
            packet_id: "packet:coupon-001-r1".into(),
            component,
            physical_unit,
            configuration,
            design_definition: design,
            requirements: vec![requirement_a, requirement_b],
            process_plan: process,
            material_batch: material,
            machine,
            tooling: vec![tooling],
            build_evidence: build,
            inspection_plan,
            inspection_instrument,
            inspection_calibration_validity: CalibrationValidityV1 {
                evidence: evidence_ref(&inspection_calibration),
                valid_from_unix_ms: 1_800_000_000_000,
                valid_until_unix_ms: 1_900_000_000_000,
                used_at_unix_ms: 1_850_000_000_000,
            },
            inspection_calibration,
            inspection_result,
            inspection_evidence: inspection,
            test_plan,
            test_instrument,
            test_calibration_validity: CalibrationValidityV1 {
                evidence: evidence_ref(&test_calibration),
                valid_from_unix_ms: 1_800_000_000_000,
                valid_until_unix_ms: 1_900_000_000_000,
                used_at_unix_ms: 1_850_000_010_000,
            },
            test_calibration,
            test_result,
            test_evidence: test,
        }
    }

    #[test]
    fn complete_synthetic_packet_is_structurally_complete() {
        let report = validate_packet(&fixture()).expect("valid fixture");
        assert_eq!(report.disposition, PacketDisposition::StructurallyComplete);
        assert_eq!(report.evidence_records, 5);
        assert_eq!(report.content_artifacts, 4);
    }

    #[test]
    fn wrong_configuration_is_rejected() {
        let mut packet = fixture();
        packet.inspection_evidence.configuration =
            reference("configuration:other-revision", "configuration");
        assert!(matches!(validate_packet(&packet), Err(PacketError::WrongEvidenceConfiguration(_))));
    }

    #[test]
    fn missing_material_lineage_is_rejected() {
        let mut packet = fixture();
        packet.build_evidence.inputs.retain(|item| item != &packet.material_batch);
        assert!(matches!(validate_packet(&packet), Err(PacketError::MissingEvidenceReference { .. })));
    }

    #[test]
    fn a_reference_in_the_wrong_evidence_role_is_rejected() {
        let mut packet = fixture();
        packet.build_evidence.inputs.retain(|item| item != &packet.design_definition.reference);
        packet.build_evidence.basis.push(packet.design_definition.reference.clone());
        assert!(matches!(validate_packet(&packet), Err(PacketError::MissingEvidenceReference { .. })));
    }

    #[test]
    fn stale_calibration_is_rejected() {
        let mut packet = fixture();
        packet.inspection_calibration.lifecycle.state = EvidenceLifecycle::Superseded;
        packet.inspection_calibration.lifecycle.predecessor = Some("evidence:old-cal".into());
        assert!(matches!(validate_packet(&packet), Err(PacketError::EvidenceNotActive(_))));
    }

    #[test]
    fn malformed_or_mismatched_digest_is_rejected() {
        let mut packet = fixture();
        packet.design_definition.sha256 = "not-a-digest".into();
        assert!(matches!(validate_packet(&packet), Err(PacketError::InvalidContentArtifact(_))));
    }

    #[test]
    fn zero_length_artifact_is_rejected() {
        let mut packet = fixture();
        packet.inspection_result.byte_length = 0;
        assert!(matches!(validate_packet(&packet), Err(PacketError::InvalidContentArtifact(_))));
    }

    #[test]
    fn wrong_instrument_calibration_subject_is_rejected() {
        let mut packet = fixture();
        packet.test_calibration.subject = reference("instrument:other", "instrument");
        assert!(matches!(validate_packet(&packet), Err(PacketError::WrongEvidenceSubject(_))));
    }

    #[test]
    fn duplicate_evidence_identity_is_rejected() {
        let mut packet = fixture();
        packet.test_evidence.evidence_id = packet.build_evidence.evidence_id.clone();
        assert!(matches!(validate_packet(&packet), Err(PacketError::DuplicateIdentity(_))));
    }

    #[test]
    fn wrong_method_and_missing_qif_result_are_rejected() {
        let mut packet = fixture();
        packet.inspection_evidence.method = Some(reference("inspection-plan:other", "inspection_plan"));
        assert!(matches!(validate_packet(&packet), Err(PacketError::WrongEvidenceMethod(_))));

        let mut packet = fixture();
        packet.inspection_evidence.inputs.retain(|item| item != &packet.inspection_result.reference);
        assert!(matches!(validate_packet(&packet), Err(PacketError::MissingEvidenceReference { .. })));
    }

    #[test]
    fn order_permutation_does_not_change_disposition_or_counts() {
        let packet = fixture();
        let mut permuted = packet.clone();
        permuted.requirements.reverse();
        permuted.tooling.reverse();
        permuted.build_evidence.inputs.reverse();
        permuted.build_evidence.basis.reverse();
        permuted.build_evidence.toolchain.reverse();
        permuted.inspection_evidence.inputs.reverse();
        permuted.test_evidence.inputs.reverse();
        assert_eq!(validate_packet(&packet), validate_packet(&permuted));
    }

    #[test]
    fn unknown_packet_fields_are_rejected() {
        let packet = fixture();
        let mut value = serde_json::to_value(packet).expect("serialize fixture");
        value.as_object_mut().expect("object").insert(
            "hidden_release_authority".into(),
            json!({"authority": "invented"}),
        );
        assert!(serde_json::from_value::<ComponentBuildPacketV1>(value).is_err());
    }

    #[test]
    fn structural_success_is_not_an_acceptance_or_release_state() {
        let report = validate_packet(&fixture()).expect("valid fixture");
        assert_eq!(report.disposition, PacketDisposition::StructurallyComplete);
    }

    #[test]
    fn calibration_must_cover_actual_measurement_time() {
        let mut packet = fixture();
        packet.inspection_calibration_validity.used_at_unix_ms =
            packet.inspection_calibration_validity.valid_until_unix_ms;
        assert!(matches!(validate_packet(&packet), Err(PacketError::CalibrationWindowInvalid(_))));
    }

    #[test]
    fn test_event_must_follow_inspection_event() {
        let mut packet = fixture();
        packet.test_calibration_validity.used_at_unix_ms =
            packet.inspection_calibration_validity.used_at_unix_ms - 1;
        assert!(matches!(validate_packet(&packet), Err(PacketError::NonMonotonicTestTime)));
    }

    #[test]
    fn physical_unit_is_distinct_from_component_definition() {
        let mut packet = fixture();
        packet.physical_unit = packet.component.clone();
        assert!(matches!(validate_packet(&packet), Err(PacketError::InvalidIdentity)));
    }

    #[test]
    fn artifact_byte_verification_checks_length_and_sha256() {
        let bytes = b"synthetic AP242 payload";
        let digest = Sha256::digest(bytes)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        let artifact = ContentArtifactV1 {
            reference: reference(&format!("sha256:{digest}"), "design_definition"),
            sha256: digest,
            byte_length: bytes.len() as u64,
            media_type: "application/step".into(),
            format_id: "ISO_10303_242:2025".into(),
        };
        assert_eq!(verify_artifact_bytes(&artifact, bytes), Ok(()));
        assert!(matches!(
            verify_artifact_bytes(&artifact, b"changed AP242 payload"),
            Err(PacketError::ArtifactDigestMismatch(_))
        ));
        assert!(matches!(
            verify_artifact_bytes(&artifact, b"short"),
            Err(PacketError::ArtifactByteLengthMismatch(_))
        ));
    }

    fn set_artifact_payload(artifact: &mut ContentArtifactV1, bytes: &[u8]) {
        let digest = Sha256::digest(bytes)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        artifact.sha256 = digest.clone();
        artifact.reference.id = format!("sha256:{digest}");
        artifact.byte_length = bytes.len() as u64;
    }

    #[test]
    fn whole_packet_artifact_verification_checks_every_payload() {
        let mut packet = fixture();
        let design = b"synthetic STEP AP242 document";
        let process = b"synthetic process plan";
        let inspection = b"synthetic QIF result";
        let test = b"synthetic test result";
        set_artifact_payload(&mut packet.design_definition, design);
        set_artifact_payload(&mut packet.process_plan, process);
        set_artifact_payload(&mut packet.inspection_result, inspection);
        set_artifact_payload(&mut packet.test_result, test);

        let payloads = [
            (&packet.design_definition, design.as_slice()),
            (&packet.process_plan, process.as_slice()),
            (&packet.inspection_result, inspection.as_slice()),
            (&packet.test_result, test.as_slice()),
        ];
        let report = verify_packet_artifact_payloads(&packet, &payloads).expect("verified bytes");
        assert!(report.artifact_payloads_verified);
        assert_eq!(report.disposition, PacketDisposition::StructurallyComplete);

        let mutated = b"synthetic QIF resulT";
        let bad_payloads = [
            (&packet.design_definition, design.as_slice()),
            (&packet.process_plan, process.as_slice()),
            (&packet.inspection_result, mutated.as_slice()),
            (&packet.test_result, test.as_slice()),
        ];
        assert!(matches!(
            verify_packet_artifact_payloads(&packet, &bad_payloads),
            Err(PacketError::ArtifactDigestMismatch(_))
        ));
    }

    #[test]
    fn whole_packet_artifact_verification_rejects_missing_and_duplicate_payloads() {
        let mut packet = fixture();
        let design = b"design";
        let process = b"process";
        let inspection = b"inspection";
        let test = b"test";
        set_artifact_payload(&mut packet.design_definition, design);
        set_artifact_payload(&mut packet.process_plan, process);
        set_artifact_payload(&mut packet.inspection_result, inspection);
        set_artifact_payload(&mut packet.test_result, test);
        let payloads = [
            (&packet.design_definition, design.as_slice()),
            (&packet.process_plan, process.as_slice()),
            (&packet.inspection_result, inspection.as_slice()),
            (&packet.test_result, test.as_slice()),
        ];
        assert!(matches!(
            verify_packet_artifact_payloads(&packet, &payloads[..3]),
            Err(PacketError::MissingArtifactPayload(_))
        ));
        let duplicated = [
            (&packet.design_definition, design.as_slice()),
            (&packet.process_plan, process.as_slice()),
            (&packet.inspection_result, inspection.as_slice()),
            (&packet.inspection_result, inspection.as_slice()),
        ];
        assert!(matches!(
            verify_packet_artifact_payloads(&packet, &duplicated),
            Err(PacketError::DuplicateArtifactPayload(_))
        ));
    }

    #[test]
    fn strict_parser_rejects_unknown_evidence_fields() {
        let mut value: serde_json::Value =
            serde_json::from_slice(include_bytes!("../fixtures/nonflight_coupon_packet_v1.json"))
                .expect("fixture JSON");
        value["build_evidence"]["hidden_authority"] = json!(true);
        let bytes = serde_json::to_vec(&value).expect("serialize mutated fixture");
        assert!(matches!(
            parse_packet_json(&bytes),
            Err(PacketError::UnknownField { .. })
        ));
    }

    #[test]
    fn strict_parser_requires_optional_evidence_fields_to_be_explicit() {
        let mut value: serde_json::Value =
            serde_json::from_slice(include_bytes!("../fixtures/nonflight_coupon_packet_v1.json"))
                .expect("fixture JSON");
        value["build_evidence"].as_object_mut().unwrap().remove("method");
        let bytes = serde_json::to_vec(&value).expect("serialize mutated fixture");
        assert!(matches!(
            parse_packet_json(&bytes),
            Err(PacketError::MalformedPacketField(_))
        ));
    }

    #[test]
    fn strict_parser_rejects_unknown_artifact_reference_fields() {
        let mut value: serde_json::Value =
            serde_json::from_slice(include_bytes!("../fixtures/nonflight_coupon_packet_v1.json"))
                .expect("fixture JSON");
        value["physical_unit"]["release_status"] = json!("approved");
        let bytes = serde_json::to_vec(&value).expect("serialize mutated fixture");
        assert!(matches!(
            parse_packet_json(&bytes),
            Err(PacketError::UnknownField { .. })
        ));
    }

    #[test]
    fn checked_in_fixture_replays() {
        let packet = parse_packet_json(include_bytes!("../fixtures/nonflight_coupon_packet_v1.json"))
            .expect("fixture JSON");
        assert_eq!(
            validate_packet(&packet).expect("valid fixture").disposition,
            PacketDisposition::StructurallyComplete
        );
        let payloads = [
            (&packet.design_definition, include_bytes!("../fixtures/artifacts/design.step").as_slice()),
            (&packet.process_plan, include_bytes!("../fixtures/artifacts/process-plan.json").as_slice()),
            (&packet.inspection_result, include_bytes!("../fixtures/artifacts/inspection.qif").as_slice()),
            (&packet.test_result, include_bytes!("../fixtures/artifacts/test-result.json").as_slice()),
        ];
        let report = verify_packet_artifact_payloads(&packet, &payloads)
            .expect("all checked-in fixture bytes match their SHA-256 references");
        assert!(report.artifact_payloads_verified);
    }
}
