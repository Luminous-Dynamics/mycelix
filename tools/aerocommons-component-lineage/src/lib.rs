//! A bounded, synthetic AeroCommons build-packet verifier.
//!
//! A successful result means only that declared records are structurally
//! complete and consistently linked. It does not qualify a physical part.

use aero_evidence_types::{
    validate_evidence, AeroEvidenceV1, ArtifactRef, ClaimKind, EvidenceLifecycle,
};
use serde::{Deserialize, Serialize};
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentBuildPacketV1 {
    pub schema: String,
    pub packet_id: String,
    pub component: ArtifactRef,
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
    pub inspection_result: ContentArtifactV1,
    pub inspection_evidence: AeroEvidenceV1,
    pub test_plan: ArtifactRef,
    pub test_instrument: ArtifactRef,
    pub test_calibration: AeroEvidenceV1,
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
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum PacketError {
    #[error("packet schema must be {PACKET_SCHEMA}")]
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
}

fn valid_ref(reference: &ArtifactRef) -> bool {
    !reference.id.trim().is_empty() && !reference.kind.trim().is_empty()
}

fn contains_ref(evidence: &AeroEvidenceV1, required: &ArtifactRef) -> bool {
    evidence.inputs.iter().any(|item| item == required)
        || evidence.basis.iter().any(|item| item == required)
        || evidence.toolchain.iter().any(|item| item == required)
}

fn require_ref(
    evidence: &AeroEvidenceV1,
    required: &ArtifactRef,
) -> Result<(), PacketError> {
    if contains_ref(evidence, required) {
        Ok(())
    } else {
        Err(PacketError::MissingEvidenceReference {
            evidence_id: evidence.evidence_id.clone(),
            reference_id: required.id.clone(),
            reference_kind: required.kind.clone(),
        })
    }
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
        || !valid_ref(&packet.configuration)
        || packet.component.kind != "component"
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
        &packet.component,
        Some(&packet.configuration),
    )?;
    require_ref(&packet.build_evidence, &packet.design_definition.reference)?;
    require_ref(&packet.build_evidence, &packet.process_plan.reference)?;
    require_ref(&packet.build_evidence, &packet.material_batch)?;
    for requirement in &packet.requirements {
        if !packet.build_evidence.basis.iter().any(|item| item == requirement) {
            return Err(PacketError::MissingEvidenceReference {
                evidence_id: packet.build_evidence.evidence_id.clone(),
                reference_id: requirement.id.clone(),
                reference_kind: requirement.kind.clone(),
            });
        }
    }
    require_ref(&packet.build_evidence, &packet.machine)?;
    for tooling in &packet.tooling {
        require_ref(&packet.build_evidence, tooling)?;
    }

    validate_evidence_record(
        &packet.inspection_calibration,
        ClaimKind::Qualification,
        &packet.inspection_instrument,
        None,
    )?;
    validate_evidence_record(
        &packet.inspection_evidence,
        ClaimKind::Inspection,
        &packet.component,
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
        require_ref(&packet.inspection_evidence, &required)?;
    }

    validate_evidence_record(
        &packet.test_calibration,
        ClaimKind::Qualification,
        &packet.test_instrument,
        None,
    )?;
    validate_evidence_record(
        &packet.test_evidence,
        ClaimKind::Test,
        &packet.component,
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
        require_ref(&packet.test_evidence, &required)?;
    }

    let required_references = packet.requirements.len() + packet.tooling.len() + 12;
    Ok(ValidationReport {
        disposition: PacketDisposition::StructurallyComplete,
        packet_id: packet.packet_id.clone(),
        evidence_records: evidence_records.len(),
        content_artifacts: 4,
        required_references,
    })
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
            component.clone(),
            configuration.clone(),
            Some(process.reference.clone()),
            vec![design.reference.clone(), process.reference.clone(), material.clone()],
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
            component.clone(),
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
            component.clone(),
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
            inspection_calibration,
            inspection_result,
            inspection_evidence: inspection,
            test_plan,
            test_instrument,
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
}
