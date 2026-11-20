//! QIF 3.0 external-document xId resolution for AeroCommons.
//!
//! This crate verifies a normalized cross-document trace graph. It is not a
//! QIF XML parser, AP242 parser, GD&T evaluator, or inspection acceptance tool.
//! QIF local IDs, external-document references, external xIds and STEP UUIDs
//! remain separate identifier namespaces.

use aero_adapter_run_receipt::{verify_artifact_bytes, ContentArtifactV1, ReceiptError};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};
use thiserror::Error;

pub const TRACE_SCHEMA: &str = "AeroQifExternalReferenceTraceV1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasurementStatus {
    Passed,
    Failed,
    NotEvaluated,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlanObjectKind {
    CharacteristicItem,
    FeatureItem,
    DatumDefinition,
    Other,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StepEntityKind {
    Characteristic,
    GeometricFeature,
    Datum,
    ProductDefinition,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExternalQifDocumentReferenceV1 {
    /// The id of this ExternalQIFDocumentReference in the Results document.
    pub local_reference_id: u32,
    /// The QPId identifying the external QIF Plan document.
    pub plan_qpid: String,
    /// Exact pinned Plan bytes. URI is descriptive and never used as identity.
    pub plan_artifact: ContentArtifactV1,
    pub uri: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CharacteristicMeasurementReferenceV1 {
    pub measurement_id: u32,
    /// Local QIF ID selecting ExternalQIFDocumentReference in this Results doc.
    pub external_document_reference_id: u32,
    /// External QIF object id (QIFReferenceType xId) inside the selected Plan.
    pub xid: u32,
    pub status: MeasurementStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResultsDocumentV1 {
    pub qpid: String,
    pub artifact: ContentArtifactV1,
    pub external_documents: Vec<ExternalQifDocumentReferenceV1>,
    pub measurements: Vec<CharacteristicMeasurementReferenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCharacteristicObjectV1 {
    /// The QIF object id within this Plan document, selected by external xId.
    pub object_id: u32,
    pub kind: PlanObjectKind,
    pub characteristic_id: String,
    /// STEP persistent UUID reached through the Plan object's external-ID map.
    pub step_uuid: String,
    pub step_entity_kind: StepEntityKind,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifPlanDocumentV1 {
    pub qpid: String,
    pub artifact: ContentArtifactV1,
    pub step_document: ContentArtifactV1,
    pub objects: Vec<PlanCharacteristicObjectV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StepEntityV1 {
    pub uuid: String,
    pub kind: StepEntityKind,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StepDocumentV1 {
    pub artifact: ContentArtifactV1,
    pub entities: Vec<StepEntityV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifExternalReferenceTraceCaseV1 {
    pub schema: String,
    pub benchmark_id: String,
    pub case_id: String,
    pub results: ResultsDocumentV1,
    pub plans: Vec<QifPlanDocumentV1>,
    pub step_documents: Vec<StepDocumentV1>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedMeasurementTraceV1 {
    pub measurement_id: String,
    pub status: MeasurementStatus,
    pub results_qpid: String,
    pub external_document_reference_id: u32,
    pub plan_qpid: String,
    pub plan_artifact: aero_adapter_run_receipt::ContentArtifactV1,
    pub plan_object_id: u32,
    pub characteristic_id: String,
    pub step_document: ContentArtifactV1,
    pub step_uuid: String,
    pub step_entity_kind: StepEntityKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceDisposition {
    AllMeasurementReferencesResolved,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TraceVerificationReport {
    pub disposition: TraceDisposition,
    pub resolved_measurements: Vec<ResolvedMeasurementTraceV1>,
    pub checked_artifacts: usize,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum TraceError {
    #[error("invalid JSON: {0}")]
    InvalidJson(String),
    #[error("malformed object at {0}")]
    MalformedObject(String),
    #[error("unknown field {field} at {scope}")]
    UnknownField { scope: String, field: String },
    #[error("missing field {field} at {scope}")]
    MissingField { scope: String, field: String },
    #[error("wrong trace schema")]
    WrongSchema,
    #[error("empty identity at {0}")]
    InvalidIdentity(String),
    #[error("invalid QPId or STEP UUID at {0}")]
    InvalidUuid(String),
    #[error("zero identifier at {0}")]
    ZeroIdentifier(String),
    #[error("duplicate identifier at {0}")]
    DuplicateIdentifier(String),
    #[error("missing external document reference {0}")]
    MissingExternalDocumentReference(u32),
    #[error("missing QIF Plan document {0}")]
    MissingPlan(String),
    #[error("external Plan identity or content does not match reference {0}")]
    PlanReferenceMismatch(u32),
    #[error("missing Plan object xId {xid} in QPId {qpid}")]
    MissingExternalObject { qpid: String, xid: u32 },
    #[error("xId {xid} in QPId {qpid} resolved to a non-CharacteristicItem")]
    WrongExternalObjectKind { qpid: String, xid: u32 },
    #[error("missing STEP document for Plan {0}")]
    MissingStepDocument(String),
    #[error("STEP document artifact does not exactly match Plan reference for {0}")]
    StepDocumentReferenceMismatch(String),
    #[error("Results document contains no CharacteristicMeasurement references")]
    EmptyMeasurementSet,
    #[error("missing STEP UUID {uuid} in document {document}")]
    MissingStepEntity { document: String, uuid: String },
    #[error("STEP UUID {uuid} has a kind mismatch in document {document}")]
    StepEntityKindMismatch { document: String, uuid: String },
    #[error("invalid content artifact {0}")]
    InvalidArtifact(String),
    #[error("duplicate artifact reference {0}")]
    DuplicateArtifact(String),
    #[error("artifact byte verification failed: {0}")]
    ArtifactVerification(ReceiptError),
    #[error("artifact payload set is incomplete or contains extra entries")]
    ArtifactPayloadSetMismatch,
    #[error("duplicate artifact payload {0}")]
    DuplicateArtifactPayload(String),
    #[error("unexpected artifact payload {0}")]
    UnexpectedArtifactPayload(String),
}

pub fn parse_trace_case_json(bytes: &[u8]) -> Result<QifExternalReferenceTraceCaseV1, TraceError> {
    let value: Value = serde_json::from_slice(bytes)
        .map_err(|error| TraceError::InvalidJson(error.to_string()))?;
    const ROOT: &[&str] = &["schema", "benchmark_id", "case_id", "results", "plans", "step_documents"];
    check_fields(&value, "case", ROOT, ROOT)?;

    let results = value.get("results")
        .ok_or_else(|| TraceError::MissingField { scope: "case".into(), field: "results".into() })?;
    check_fields(
        results,
        "case.results",
        &["qpid", "artifact", "external_documents", "measurements"],
        &["qpid", "artifact", "external_documents", "measurements"],
    )?;
    check_content_artifact(
        results.get("artifact").ok_or_else(|| TraceError::MissingField {
            scope: "case.results".into(), field: "artifact".into()
        })?,
        "case.results.artifact",
    )?;

    let external_documents = results.get("external_documents").and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.results.external_documents".into()))?;
    for (index, item) in external_documents.iter().enumerate() {
        let scope = format!("case.results.external_documents[{index}]");
        check_fields(item, &scope, &["local_reference_id", "plan_qpid", "plan_artifact", "uri"],
            &["local_reference_id", "plan_qpid", "plan_artifact", "uri"])?;
        check_content_artifact(
            item.get("plan_artifact").ok_or_else(|| TraceError::MissingField {
                scope: scope.clone(), field: "plan_artifact".into()
            })?,
            &format!("{scope}.plan_artifact"),
        )?;
    }

    let measurements = results.get("measurements").and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.results.measurements".into()))?;
    for (index, item) in measurements.iter().enumerate() {
        let scope = format!("case.results.measurements[{index}]");
        check_fields(item, &scope,
            &["measurement_id", "external_document_reference_id", "xid", "status"],
            &["measurement_id", "external_document_reference_id", "xid", "status"])?;
    }

    let plans = value.get("plans").and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.plans".into()))?;
    for (index, plan) in plans.iter().enumerate() {
        let scope = format!("case.plans[{index}]");
        check_fields(plan, &scope, &["qpid", "artifact", "step_document", "objects"],
            &["qpid", "artifact", "step_document", "objects"])?;
        check_content_artifact(plan.get("artifact").ok_or_else(|| TraceError::MissingField {
            scope: scope.clone(), field: "artifact".into()
        })?, &format!("{scope}.artifact"))?;
        check_content_artifact(plan.get("step_document").ok_or_else(|| TraceError::MissingField {
            scope: scope.clone(), field: "step_document".into()
        })?, &format!("{scope}.step_document"))?;

        let objects = plan.get("objects").and_then(Value::as_array)
            .ok_or_else(|| TraceError::MalformedObject(format!("{scope}.objects")))?;
        for (object_index, object) in objects.iter().enumerate() {
            let object_scope = format!("{scope}.objects[{object_index}]");
            check_fields(object, &object_scope,
                &["object_id", "kind", "characteristic_id", "step_uuid", "step_entity_kind"],
                &["object_id", "kind", "characteristic_id", "step_uuid", "step_entity_kind"])?;
        }
    }

    let step_documents = value.get("step_documents").and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.step_documents".into()))?;
    for (index, document) in step_documents.iter().enumerate() {
        let scope = format!("case.step_documents[{index}]");
        check_fields(document, &scope, &["artifact", "entities"], &["artifact", "entities"])?;
        check_content_artifact(document.get("artifact").ok_or_else(|| TraceError::MissingField {
            scope: scope.clone(), field: "artifact".into()
        })?, &format!("{scope}.artifact"))?;
        let entities = document.get("entities").and_then(Value::as_array)
            .ok_or_else(|| TraceError::MalformedObject(format!("{scope}.entities")))?;
        for (entity_index, entity) in entities.iter().enumerate() {
            let entity_scope = format!("{scope}.entities[{entity_index}]");
            check_fields(entity, &entity_scope, &["uuid", "kind"], &["uuid", "kind"])?;
        }
    }

    let case: QifExternalReferenceTraceCaseV1 = serde_json::from_value(value)
        .map_err(|error| TraceError::InvalidJson(error.to_string()))?;
    validate_trace_case(&case)?;
    Ok(case)
}

fn check_fields(value: &Value, scope: &str, allowed: &[&str], required: &[&str]) -> Result<(), TraceError> {
    let object = value.as_object().ok_or_else(|| TraceError::MalformedObject(scope.into()))?;
    for key in object.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(TraceError::UnknownField { scope: scope.into(), field: key.clone() });
        }
    }
    for field in required {
        if !object.contains_key(*field) {
            return Err(TraceError::MissingField { scope: scope.into(), field: (*field).into() });
        }
    }
    Ok(())
}

fn check_content_artifact(value: &Value, scope: &str) -> Result<(), TraceError> {
    check_fields(value, scope,
        &["reference", "sha256", "byte_length", "media_type", "format_id"],
        &["reference", "sha256", "byte_length", "media_type", "format_id"])?;
    let reference = value.get("reference")
        .ok_or_else(|| TraceError::MissingField { scope: scope.into(), field: "reference".into() })?;
    check_fields(reference, &format!("{scope}.reference"), &["id", "kind"], &["id", "kind"])
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn valid_uuid(value: &str) -> bool {
    if value.len() != 36 {
        return false;
    }
    value.bytes().enumerate().all(|(index, byte)| {
        if matches!(index, 8 | 13 | 18 | 23) {
            byte == b'-'
        } else {
            byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase()
        }
    })
}

fn validate_content_artifact(artifact: &ContentArtifactV1) -> Result<(), TraceError> {
    if artifact.reference.id.trim().is_empty()
        || artifact.reference.kind.trim().is_empty()
        || !valid_sha256(&artifact.sha256)
        || artifact.reference.id != format!("sha256:{}", artifact.sha256)
        || artifact.byte_length == 0
        || artifact.media_type.trim().is_empty()
        || artifact.format_id.trim().is_empty()
    {
        return Err(TraceError::InvalidArtifact(artifact.reference.id.clone()));
    }
    Ok(())
}

pub fn validate_trace_case(case: &QifExternalReferenceTraceCaseV1) -> Result<(), TraceError> {
    if case.schema != TRACE_SCHEMA {
        return Err(TraceError::WrongSchema);
    }
    if case.benchmark_id.trim().is_empty() || case.case_id.trim().is_empty() {
        return Err(TraceError::InvalidIdentity("benchmark_id/case_id".into()));
    }
    if !valid_uuid(&case.results.qpid) {
        return Err(TraceError::InvalidUuid("results.qpid".into()));
    }
    validate_content_artifact(&case.results.artifact)?;
    if case.results.artifact.reference.kind != "qif_results"
        || case.results.artifact.format_id != "QIF_3.0"
    {
        return Err(TraceError::InvalidArtifact(case.results.artifact.reference.id.clone()));
    }
    if case.results.measurements.is_empty() {
        return Err(TraceError::EmptyMeasurementSet);
    }

    let mut all_artifacts: BTreeSet<(String, String)> = BTreeSet::new();
    let mut add_artifact = |artifact: &ContentArtifactV1| -> Result<(), TraceError> {
        validate_content_artifact(artifact)?;
        let key = (artifact.reference.id.clone(), artifact.reference.kind.clone());
        if !all_artifacts.insert(key.clone()) {
            return Err(TraceError::DuplicateArtifact(key.0));
        }
        Ok(())
    };
    add_artifact(&case.results.artifact)?;

    let mut qpid_set = BTreeSet::new();
    if !qpid_set.insert(case.results.qpid.clone()) {
        return Err(TraceError::DuplicateIdentifier(case.results.qpid.clone()));
    }

    let mut refs_by_id = BTreeMap::new();
    for external in &case.results.external_documents {
        if external.local_reference_id == 0 {
            return Err(TraceError::ZeroIdentifier("external_documents.local_reference_id".into()));
        }
        if !valid_uuid(&external.plan_qpid) {
            return Err(TraceError::InvalidUuid("external_documents.plan_qpid".into()));
        }
        if refs_by_id.insert(external.local_reference_id, external).is_some() {
            return Err(TraceError::DuplicateIdentifier(format!("external ref {}", external.local_reference_id)));
        }
        validate_content_artifact(&external.plan_artifact)?;
        if external.plan_artifact.reference.kind != "qif_plan"
            || external.plan_artifact.format_id != "QIF_3.0"
        {
            return Err(TraceError::InvalidArtifact(external.plan_artifact.reference.id.clone()));
        }
    }

    let mut plans_by_qpid = BTreeMap::new();
    for plan in &case.plans {
        if !valid_uuid(&plan.qpid) {
            return Err(TraceError::InvalidUuid("plans.qpid".into()));
        }
        if qpid_set.contains(&plan.qpid) {
            return Err(TraceError::DuplicateIdentifier(plan.qpid.clone()));
        }
        qpid_set.insert(plan.qpid.clone());
        if plans_by_qpid.insert(plan.qpid.clone(), plan).is_some() {
            return Err(TraceError::DuplicateIdentifier(plan.qpid.clone()));
        }
        add_artifact(&plan.artifact)?;
        if plan.artifact.reference.kind != "qif_plan" || plan.artifact.format_id != "QIF_3.0" {
            return Err(TraceError::InvalidArtifact(plan.artifact.reference.id.clone()));
        }
        validate_content_artifact(&plan.step_document)?;
        if plan.step_document.reference.kind != "step_file"
            || !plan.step_document.format_id.starts_with("ISO_10303_242:")
        {
            return Err(TraceError::InvalidArtifact(plan.step_document.reference.id.clone()));
        }
    }

    for external in refs_by_id.values() {
        let plan = plans_by_qpid.get(&external.plan_qpid)
            .ok_or_else(|| TraceError::MissingPlan(external.plan_qpid.clone()))?;
        if external.plan_artifact != plan.artifact {
            return Err(TraceError::PlanReferenceMismatch(external.local_reference_id));
        }
    }

    let mut step_by_artifact = BTreeMap::new();
    for document in &case.step_documents {
        add_artifact(&document.artifact)?;
        if document.artifact.reference.kind != "step_file"
            || !document.artifact.format_id.starts_with("ISO_10303_242:")
        {
            return Err(TraceError::InvalidArtifact(document.artifact.reference.id.clone()));
        }
        let step_key = (document.artifact.reference.id.clone(), document.artifact.reference.kind.clone());
        if step_by_artifact.insert(step_key, document).is_some() {
            return Err(TraceError::DuplicateArtifact(document.artifact.reference.id.clone()));
        }
        let mut uuids = BTreeSet::new();
        for entity in &document.entities {
            if !valid_uuid(&entity.uuid) {
                return Err(TraceError::InvalidUuid(format!("step entity {}", entity.uuid)));
            }
            if !uuids.insert(entity.uuid.clone()) {
                return Err(TraceError::DuplicateIdentifier(format!("STEP UUID {}", entity.uuid)));
            }
        }
    }

    for plan in &case.plans {
        let step_key = (plan.step_document.reference.id.clone(), plan.step_document.reference.kind.clone());
        let Some(step_document) = step_by_artifact.get(&step_key) else {
            return Err(TraceError::MissingStepDocument(plan.qpid.clone()));
        };
        if step_document.artifact != plan.step_document {
            return Err(TraceError::StepDocumentReferenceMismatch(plan.qpid.clone()));
        }
        let mut object_ids = BTreeSet::new();
        for object in &plan.objects {
            if object.object_id == 0 {
                return Err(TraceError::ZeroIdentifier(format!("Plan object in {}", plan.qpid)));
            }
            if !object_ids.insert(object.object_id) {
                return Err(TraceError::DuplicateIdentifier(format!("Plan object {} in {}", object.object_id, plan.qpid)));
            }
            if object.characteristic_id.trim().is_empty() || !valid_uuid(&object.step_uuid) {
                return Err(TraceError::InvalidIdentity(format!("Plan object {}", object.object_id)));
            }
        }
    }

    let mut measurement_ids = BTreeSet::new();
    for measurement in &case.results.measurements {
        if measurement.measurement_id == 0 {
            return Err(TraceError::ZeroIdentifier("measurement_id".into()));
        }
        if !measurement_ids.insert(measurement.measurement_id) {
            return Err(TraceError::DuplicateIdentifier(format!("measurement {}", measurement.measurement_id)));
        }
        if measurement.external_document_reference_id == 0 || measurement.xid == 0 {
            return Err(TraceError::ZeroIdentifier(format!("measurement {}", measurement.measurement_id)));
        }

        let external = refs_by_id.get(&measurement.external_document_reference_id)
            .ok_or(TraceError::MissingExternalDocumentReference(measurement.external_document_reference_id))?;
        let plan = plans_by_qpid.get(&external.plan_qpid)
            .ok_or_else(|| TraceError::MissingPlan(external.plan_qpid.clone()))?;
        if external.plan_artifact != plan.artifact {
            return Err(TraceError::PlanReferenceMismatch(external.local_reference_id));
        }
        let object = plan.objects.iter().find(|item| item.object_id == measurement.xid)
            .ok_or_else(|| TraceError::MissingExternalObject { qpid: plan.qpid.clone(), xid: measurement.xid })?;
        if object.kind != PlanObjectKind::CharacteristicItem {
            return Err(TraceError::WrongExternalObjectKind { qpid: plan.qpid.clone(), xid: measurement.xid });
        }

        let step_key = (plan.step_document.reference.id.clone(), plan.step_document.reference.kind.clone());
        let step_document = step_by_artifact.get(&step_key)
            .ok_or_else(|| TraceError::MissingStepDocument(plan.qpid.clone()))?;
        let entity = step_document.entities.iter().find(|item| item.uuid == object.step_uuid)
            .ok_or_else(|| TraceError::MissingStepEntity {
                document: step_document.artifact.reference.id.clone(),
                uuid: object.step_uuid.clone(),
            })?;
        if entity.kind != object.step_entity_kind {
            return Err(TraceError::StepEntityKindMismatch {
                document: step_document.artifact.reference.id.clone(),
                uuid: object.step_uuid.clone(),
            });
        }

    }
    Ok(())
}

/// Verify exact bytes for Results, every referenced Plan, and every STEP source,
/// then resolve all measurements. The supplied payload set must be exact.
pub fn verify_and_resolve_trace_case(
    case: &QifExternalReferenceTraceCaseV1,
    payloads: &[(&aero_evidence_types::ArtifactRef, &[u8])],
) -> Result<TraceVerificationReport, TraceError> {
    validate_trace_case(case)?;
    let mut expected: Vec<&ContentArtifactV1> = vec![&case.results.artifact];
    expected.extend(case.plans.iter().map(|plan| &plan.artifact));
    expected.extend(case.step_documents.iter().map(|document| &document.artifact));
    if payloads.len() != expected.len() {
        return Err(TraceError::ArtifactPayloadSetMismatch);
    }
    let mut seen = BTreeSet::new();
    for (reference, bytes) in payloads {
        let key = (reference.id.clone(), reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(TraceError::DuplicateArtifactPayload(key.0));
        }
        let artifact = expected.iter().copied().find(|item| {
            item.reference.id == reference.id && item.reference.kind == reference.kind
        }).ok_or_else(|| TraceError::UnexpectedArtifactPayload(key.0))?;
        verify_artifact_bytes(artifact, bytes).map_err(TraceError::ArtifactVerification)?;
    }
    for artifact in &expected {
        if !seen.contains(&(artifact.reference.id.clone(), artifact.reference.kind.clone())) {
            return Err(TraceError::ArtifactPayloadSetMismatch);
        }
    }

    let report = resolve_without_bytes(case)?;
    Ok(TraceVerificationReport {
        disposition: report.0,
        resolved_measurements: report.1,
        checked_artifacts: expected.len(),
    })
}

fn resolve_without_bytes(
    case: &QifExternalReferenceTraceCaseV1,
) -> Result<(TraceDisposition, Vec<ResolvedMeasurementTraceV1>), TraceError> {
    // Reuse the same checked graph resolver by requiring all structural
    // invariants here, but do not duplicate byte verification.
    validate_trace_case(case)?;
    let refs: BTreeMap<_, _> = case.results.external_documents.iter()
        .map(|item| (item.local_reference_id, item))
        .collect();
    let plans: BTreeMap<_, _> = case.plans.iter().map(|plan| (plan.qpid.clone(), plan)).collect();
    let steps: BTreeMap<_, _> = case.step_documents.iter()
        .map(|document| ((document.artifact.reference.id.clone(), document.artifact.reference.kind.clone()), document)).collect();
    let mut resolved = Vec::new();
    for measurement in &case.results.measurements {
        let external = refs[&measurement.external_document_reference_id];
        let plan = plans[&external.plan_qpid];
        let object = plan.objects.iter().find(|item| item.object_id == measurement.xid)
            .expect("validated Plan xId");
        let step_key = (plan.step_document.reference.id.clone(), plan.step_document.reference.kind.clone());
        let step = steps[&step_key];
        let entity = step.entities.iter().find(|item| item.uuid == object.step_uuid)
            .expect("validated STEP UUID");
        resolved.push(ResolvedMeasurementTraceV1 {
            measurement_id: measurement.measurement_id.clone(),
            status: measurement.status,
            results_qpid: case.results.qpid.clone(),
            external_document_reference_id: external.local_reference_id,
            plan_qpid: plan.qpid.clone(),
            plan_artifact: plan.artifact.clone(),
            plan_object_id: object.object_id,
            characteristic_id: object.characteristic_id.clone(),
            step_document: step.artifact.clone(),
            step_uuid: object.step_uuid.clone(),
            step_entity_kind: entity.kind,
        });
    }
    resolved.sort_by(|left, right| left.measurement_id.cmp(&right.measurement_id));
    Ok((TraceDisposition::AllMeasurementReferencesResolved, resolved))
}

#[cfg(test)]
mod tests {
    use super::*;
    use aero_adapter_run_receipt::ContentArtifactV1;
    use aero_evidence_types::ArtifactRef;
    use serde_json::json;
    use sha2::{Digest, Sha256};

    fn artifact(kind: &str, format: &str, media: &str, bytes: &[u8]) -> ContentArtifactV1 {
        let digest = Sha256::digest(bytes);
        let sha256 = digest.iter().map(|byte| format!("{byte:02x}")).collect::<String>();
        ContentArtifactV1 {
            reference: ArtifactRef { id: format!("sha256:{sha256}"), kind: kind.into() },
            sha256,
            byte_length: bytes.len() as u64,
            media_type: media.into(),
            format_id: format.into(),
        }
    }

    fn fixture() -> (QifExternalReferenceTraceCaseV1, Vec<(ArtifactRef, Vec<u8>)>) {
        let results_bytes = b"<QIFDocument kind=\"Results\">synthetic</QIFDocument>";
        let plan_bytes = b"<QIFDocument kind=\"Plan\">synthetic</QIFDocument>";
        let step_bytes = b"ISO-10303-21; synthetic STEP source";
        let results_artifact = artifact("qif_results", "QIF_3.0", "application/xml", results_bytes);
        let plan_artifact = artifact("qif_plan", "QIF_3.0", "application/xml", plan_bytes);
        let step_artifact = artifact("step_file", "ISO_10303_242:2025", "application/step", step_bytes);
        let case = QifExternalReferenceTraceCaseV1 {
            schema: TRACE_SCHEMA.into(),
            benchmark_id: "AERO-QIF-XID-001".into(),
            case_id: "synthetic-test8-001".into(),
            results: ResultsDocumentV1 {
                qpid: "10000000-0000-4000-8000-000000000001".into(),
                artifact: results_artifact.clone(),
                external_documents: vec![
                    ExternalQifDocumentReferenceV1 {
                        local_reference_id: 7,
                        plan_qpid: "20000000-0000-4000-8000-000000000002".into(),
                        plan_artifact: plan_artifact.clone(),
                        uri: Some("synthetic://plan-a".into()),
                    },
                    ExternalQifDocumentReferenceV1 {
                        local_reference_id: 8,
                        plan_qpid: "30000000-0000-4000-8000-000000000003".into(),
                        plan_artifact: artifact("qif_plan", "QIF_3.0", "application/xml", b"<PlanB/>"),
                        uri: Some("synthetic://plan-b".into()),
                    },
                ],
                measurements: vec![
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: "measurement:pass".into(),
                        external_document_reference_id: 7,
                        xid: 301,
                        status: MeasurementStatus::Passed,
                    },
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: "measurement:fail".into(),
                        external_document_reference_id: 7,
                        xid: 302,
                        status: MeasurementStatus::Failed,
                    },
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: "measurement:second-plan".into(),
                        external_document_reference_id: 8,
                        xid: 301,
                        status: MeasurementStatus::NotEvaluated,
                    },
                ],
            },
            plans: vec![
                QifPlanDocumentV1 {
                    qpid: "20000000-0000-4000-8000-000000000002".into(),
                    artifact: plan_artifact.clone(),
                    step_document: step_artifact.clone(),
                    objects: vec![
                        PlanCharacteristicObjectV1 {
                            object_id: 301,
                            kind: PlanObjectKind::CharacteristicItem,
                            characteristic_id: "characteristic:hole-dia".into(),
                            step_uuid: "40000000-0000-4000-8000-000000000004".into(),
                            step_entity_kind: StepEntityKind::Characteristic,
                        },
                        PlanCharacteristicObjectV1 {
                            object_id: 302,
                            kind: PlanObjectKind::CharacteristicItem,
                            characteristic_id: "characteristic:surface".into(),
                            step_uuid: "50000000-0000-4000-8000-000000000005".into(),
                            step_entity_kind: StepEntityKind::GeometricFeature,
                        },
                    ],
                },
                QifPlanDocumentV1 {
                    qpid: "30000000-0000-4000-8000-000000000003".into(),
                    artifact: artifact("qif_plan", "QIF_3.0", "application/xml", b"<PlanB/>"),
                    step_document: step_artifact.clone(),
                    objects: vec![
                        PlanCharacteristicObjectV1 {
                            object_id: 301,
                            kind: PlanObjectKind::CharacteristicItem,
                            characteristic_id: "characteristic:flatness".into(),
                            step_uuid: "60000000-0000-4000-8000-000000000006".into(),
                            step_entity_kind: StepEntityKind::Datum,
                        },
                    ],
                },
            ],
            step_documents: vec![
                StepDocumentV1 {
                    artifact: step_artifact.clone(),
                    entities: vec![
                        StepEntityV1 {
                            uuid: "40000000-0000-4000-8000-000000000004".into(),
                            kind: StepEntityKind::Characteristic,
                        },
                        StepEntityV1 {
                            uuid: "50000000-0000-4000-8000-000000000005".into(),
                            kind: StepEntityKind::GeometricFeature,
                        },
                        StepEntityV1 {
                            uuid: "60000000-0000-4000-8000-000000000006".into(),
                            kind: StepEntityKind::Datum,
                        },
                    ],
                },
            ],
        };
        let payloads = vec![
            (results_artifact.reference.clone(), results_bytes.to_vec()),
            (plan_artifact.reference.clone(), plan_bytes.to_vec()),
            (case.plans[1].artifact.reference.clone(), b"<PlanB/>".to_vec()),
            (step_artifact.reference.clone(), step_bytes.to_vec()),
        ];
        (case, payloads)
    }

    fn refs_and_bytes(payloads: &[(ArtifactRef, Vec<u8>)]) -> Vec<(&ArtifactRef, &[u8])> {
        payloads.iter().map(|(reference, bytes)| (reference, bytes.as_slice())).collect()
    }

    #[test]
    fn checked_in_normalized_test8_fixture_replays_exact_bytes() {
        let case = parse_trace_case_json(
            include_bytes!("../fixtures/test8-normalized-trace-v1.json")
        ).expect("checked-in normalized trace fixture parses");
        let payloads = [
            (&case.results.artifact.reference, include_bytes!("../fixtures/results.qif").as_slice()),
            (&case.plans[0].artifact.reference, include_bytes!("../fixtures/plan-a.qif").as_slice()),
            (&case.plans[1].artifact.reference, include_bytes!("../fixtures/plan-b.qif").as_slice()),
            (&case.step_documents[0].artifact.reference, include_bytes!("../fixtures/source.step").as_slice()),
        ];
        let report = verify_and_resolve_trace_case(&case, &payloads)
            .expect("all checked-in source bytes and trace links match");
        assert_eq!(report.resolved_measurements.len(), 3);
        assert_eq!(report.checked_artifacts, 4);
        assert_eq!(
            report.resolved_measurements.iter()
                .find(|item| item.measurement_id == 902)
                .expect("failed measurement retained").status,
            MeasurementStatus::Failed
        );
    }

    #[test]
    fn external_plan_qpid_mismatch_is_rejected() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].plan_qpid = "70000000-0000-4000-8000-000000000007".into();
        assert!(matches!(validate_trace_case(&case), Err(TraceError::MissingPlan(_))));
    }

    #[test]
    fn plan_step_artifact_must_match_exact_step_document_metadata() {
        let (mut case, _) = fixture();
        case.plans[0].step_document.byte_length += 1;
        assert!(matches!(validate_trace_case(&case), Err(TraceError::StepDocumentReferenceMismatch(_))));
    }

    #[test]
    fn resolves_two_external_plans_without_xid_namespace_collision() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        assert_eq!(report.disposition, TraceDisposition::AllMeasurementReferencesResolved);
        assert_eq!(report.resolved_measurements.len(), 3);
        assert_eq!(
            report.resolved_measurements.iter()
                .find(|item| item.measurement_id == 901)
                .expect("passed measurement retained").status,
            MeasurementStatus::Passed
        );
        assert_eq!(
            report.resolved_measurements.iter()
                .find(|item| item.measurement_id == 902)
                .expect("failed measurement retained").status,
            MeasurementStatus::Failed
        );
        assert_eq!(report.resolved_measurements[2].plan_object_id, 301);
        assert_eq!(report.resolved_measurements[2].plan_qpid, "30000000-0000-4000-8000-000000000003");
        assert_eq!(report.checked_artifacts, 4);
    }

    #[test]
    fn failed_measurements_remain_traceable() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let failed = report.resolved_measurements.iter().find(|item| item.status == MeasurementStatus::Failed).unwrap();
        assert_eq!(failed.characteristic_id, "characteristic:surface");
        assert_eq!(failed.step_uuid, "50000000-0000-4000-8000-000000000005");
    }

    #[test]
    fn local_reference_id_and_external_xid_are_not_collapsed() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].local_reference_id = 301;
        case.results.measurements[0].external_document_reference_id = 301;
        case.results.measurements[0].xid = 301;
        assert!(validate_trace_case(&case).is_ok());
    }

    #[test]
    fn same_xid_in_two_plans_resolves_within_exact_plan_namespace() {
        let (case, _) = fixture();
        let report = resolve_without_bytes(&case).unwrap();
        let first = report.1.iter().find(|item| item.external_document_reference_id == 7).unwrap();
        let second = report.1.iter().find(|item| item.external_document_reference_id == 8).unwrap();
        assert_ne!(first.step_uuid, second.step_uuid);
    }

    #[test]
    fn wrong_local_external_reference_is_rejected() {
        let (mut case, _) = fixture();
        case.results.measurements[0].external_document_reference_id = 999;
        assert_eq!(validate_trace_case(&case), Err(TraceError::MissingExternalDocumentReference(999)));
    }

    #[test]
    fn external_plan_qpid_and_artifact_must_match_exactly() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].plan_artifact.byte_length += 1;
        assert_eq!(validate_trace_case(&case), Err(TraceError::PlanReferenceMismatch(7)));
    }

    #[test]
    fn missing_external_xid_is_rejected() {
        let (mut case, _) = fixture();
        case.results.measurements[0].xid = 999;
        assert!(matches!(validate_trace_case(&case), Err(TraceError::MissingExternalObject { xid: 999, .. })));
    }

    #[test]
    fn xid_resolving_to_wrong_qif_object_kind_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].objects[0].kind = PlanObjectKind::DatumDefinition;
        assert!(matches!(validate_trace_case(&case), Err(TraceError::WrongExternalObjectKind { xid: 301, .. })));
    }

    #[test]
    fn duplicate_plan_object_id_is_rejected_within_document() {
        let (mut case, _) = fixture();
        case.plans[0].objects[1].object_id = 301;
        assert!(matches!(validate_trace_case(&case), Err(TraceError::DuplicateIdentifier(_))));
    }

    #[test]
    fn duplicate_step_uuid_is_rejected_within_step_document() {
        let (mut case, _) = fixture();
        case.step_documents[0].entities[1].uuid = case.step_documents[0].entities[0].uuid.clone();
        assert!(matches!(validate_trace_case(&case), Err(TraceError::DuplicateIdentifier(_))));
    }

    #[test]
    fn step_uuid_does_not_accept_an_xid_integer() {
        let (mut case, _) = fixture();
        case.plans[0].objects[0].step_uuid = "301".into();
        assert!(matches!(validate_trace_case(&case), Err(TraceError::InvalidUuid(_))));
    }

    #[test]
    fn missing_step_mapping_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].objects[0].step_uuid = "70000000-0000-4000-8000-000000000007".into();
        assert!(matches!(validate_trace_case(&case), Err(TraceError::MissingStepEntity { .. })));
    }

    #[test]
    fn step_entity_kind_mismatch_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].objects[0].step_entity_kind = StepEntityKind::Datum;
        assert!(matches!(validate_trace_case(&case), Err(TraceError::StepEntityKindMismatch { .. })));
    }

    #[test]
    fn field_order_is_irrelevant_after_normalized_parse() {
        let (case, _) = fixture();
        let mut value = serde_json::to_value(&case).unwrap();
        value["results"]["measurements"].as_array_mut().unwrap().reverse();
        let bytes = serde_json::to_vec(&value).unwrap();
        let parsed = parse_trace_case_json(&bytes).unwrap();
        let base = resolve_without_bytes(&case).unwrap();
        let reversed = resolve_without_bytes(&parsed).unwrap();
        assert_eq!(base, reversed);
    }

    #[test]
    fn unknown_nested_artifact_ref_field_is_rejected() {
        let (case, _) = fixture();
        let mut value = serde_json::to_value(&case).unwrap();
        value["plans"][0]["artifact"]["reference"]["trust"] = json!("trusted");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(parse_trace_case_json(&bytes), Err(TraceError::UnknownField { .. })));
    }

    #[test]
    fn exact_payload_bytes_are_required() {
        let (case, mut payloads) = fixture();
        payloads[0].1.push(b'X');
        assert!(matches!(
            verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)),
            Err(TraceError::ArtifactVerification(ReceiptError::ArtifactByteLengthMismatch(_)))
        ));
    }

    #[test]
    fn missing_payload_is_rejected() {
        let (case, mut payloads) = fixture();
        payloads.pop();
        assert_eq!(
            verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)),
            Err(TraceError::ArtifactPayloadSetMismatch)
        );
    }
}
