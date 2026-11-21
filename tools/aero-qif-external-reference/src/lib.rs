//! QIF 3.0 external-document xId resolution for AeroCommons.
//!
//! This crate verifies a normalized cross-document trace graph. It is not a
//! QIF XML parser, AP242 parser, GD&T evaluator, or inspection acceptance tool.
//! QIF local IDs, external-document references, external xIds and STEP UUIDs
//! remain separate identifier namespaces.

use aero_adapter_run_receipt::{ContentArtifactV1, ReceiptError, verify_artifact_bytes};
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
    #[serde(default)]
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
    /// IDs of MeasurementResultsSet objects in this Results document.
    pub result_set_ids: Vec<u32>,
    pub external_documents: Vec<ExternalQifDocumentReferenceV1>,
    pub measurements: Vec<CharacteristicMeasurementReferenceV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OtherPlanObjectV1 {
    /// ID of another QIF object type in the Plan, used to distinguish wrong-type xId.
    pub object_id: u32,
    pub object_kind: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCharacteristicItemV1 {
    /// QIF CharacteristicItem id selected by CharacteristicItemId/@xId.
    pub object_id: u32,
    /// CharacteristicItem/CharacteristicNominalId, local to this Plan.
    pub characteristic_nominal_id: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlanCharacteristicNominalV1 {
    /// QIF CharacteristicNominal id, local to this Plan.
    pub object_id: u32,
    pub characteristic_id: String,
    /// EntityExternalIds/Id values referencing DigitalModel/Entities/Entity/@id.
    pub entity_external_ids: Vec<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DigitalModelExternalEntityV1 {
    /// QIF Entity/@id within this Plan's Product/Part/DefinitionExternal/DigitalModel.
    pub entity_id: u32,
    /// Entity/EntityId: persistent UUID of the source STEP entity.
    pub step_uuid: String,
    pub step_entity_kind: StepEntityKind,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QifPlanDocumentV1 {
    pub qpid: String,
    pub artifact: ContentArtifactV1,
    pub step_document: ContentArtifactV1,
    /// Subset of Product/Part/DefinitionExternal/DigitalModel/Entities.
    pub digital_model_entities: Vec<DigitalModelExternalEntityV1>,
    pub other_objects: Vec<OtherPlanObjectV1>,
    pub characteristic_items: Vec<PlanCharacteristicItemV1>,
    pub characteristic_nominals: Vec<PlanCharacteristicNominalV1>,
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
pub struct ResolvedStepEntityV1 {
    pub entity_external_id: u32,
    pub step_uuid: String,
    pub step_entity_kind: StepEntityKind,
    pub step_document: ContentArtifactV1,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedMeasurementTraceV1 {
    pub measurement_id: u32,
    pub result_set_id: u32,
    pub status: MeasurementStatus,
    pub results_qpid: String,
    pub external_document_reference_id: u32,
    pub plan_qpid: String,
    pub plan_artifact: ContentArtifactV1,
    pub plan_characteristic_item_id: u32,
    pub characteristic_nominal_id: u32,
    pub characteristic_id: String,
    pub step_entities: Vec<ResolvedStepEntityV1>,
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
    #[error("missing Plan CharacteristicItem xId {xid} in QPId {qpid}")]
    MissingExternalObject { qpid: String, xid: u32 },
    #[error("xId {xid} in QPId {qpid} resolves to wrong QIF object type {object_kind}")]
    WrongExternalObjectKind {
        qpid: String,
        xid: u32,
        object_kind: String,
    },
    #[error("MeasurementResultsSet {0} is not declared in the Results document")]
    MissingResultSet(u32),
    #[error("CharacteristicNominal {nominal_id} missing in Plan {qpid}")]
    MissingCharacteristicNominal { qpid: String, nominal_id: u32 },
    #[error("CharacteristicNominal {nominal_id} has no EntityExternalIds")]
    MissingEntityExternalIds { nominal_id: u32 },
    #[error("EntityExternalId {entity_id} missing in DigitalModel map for Plan {qpid}")]
    MissingDigitalModelEntity { qpid: String, entity_id: u32 },
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
    const ROOT: &[&str] = &[
        "schema",
        "benchmark_id",
        "case_id",
        "results",
        "plans",
        "step_documents",
    ];
    check_fields(&value, "case", ROOT, ROOT)?;

    let results = value
        .get("results")
        .ok_or_else(|| TraceError::MissingField {
            scope: "case".into(),
            field: "results".into(),
        })?;
    check_fields(
        results,
        "case.results",
        &[
            "qpid",
            "artifact",
            "result_set_ids",
            "external_documents",
            "measurements",
        ],
        &[
            "qpid",
            "artifact",
            "result_set_ids",
            "external_documents",
            "measurements",
        ],
    )?;
    check_content_artifact(
        results
            .get("artifact")
            .ok_or_else(|| TraceError::MissingField {
                scope: "case.results".into(),
                field: "artifact".into(),
            })?,
        "case.results.artifact",
    )?;

    let result_set_ids = results
        .get("result_set_ids")
        .and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.results.result_set_ids".into()))?;
    if result_set_ids.iter().any(|id| {
        id.as_u64()
            .filter(|value| *value > 0 && *value <= u32::MAX as u64)
            .is_none()
    }) {
        return Err(TraceError::MalformedObject(
            "case.results.result_set_ids".into(),
        ));
    }

    let external_documents = results
        .get("external_documents")
        .and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.results.external_documents".into()))?;
    for (index, item) in external_documents.iter().enumerate() {
        let scope = format!("case.results.external_documents[{index}]");
        check_fields(
            item,
            &scope,
            &["local_reference_id", "plan_qpid", "plan_artifact", "uri"],
            &["local_reference_id", "plan_qpid", "plan_artifact"],
        )?;
        check_content_artifact(
            item.get("plan_artifact")
                .ok_or_else(|| TraceError::MissingField {
                    scope: scope.clone(),
                    field: "plan_artifact".into(),
                })?,
            &format!("{scope}.plan_artifact"),
        )?;
    }

    let measurements = results
        .get("measurements")
        .and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.results.measurements".into()))?;
    for (index, item) in measurements.iter().enumerate() {
        let scope = format!("case.results.measurements[{index}]");
        check_fields(
            item,
            &scope,
            &[
                "measurement_id",
                "result_set_id",
                "external_document_reference_id",
                "xid",
                "status",
            ],
            &[
                "measurement_id",
                "result_set_id",
                "external_document_reference_id",
                "xid",
                "status",
            ],
        )?;
    }

    let plans = value
        .get("plans")
        .and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.plans".into()))?;
    for (index, plan) in plans.iter().enumerate() {
        let scope = format!("case.plans[{index}]");
        check_fields(
            plan,
            &scope,
            &[
                "qpid",
                "artifact",
                "step_document",
                "digital_model_entities",
                "other_objects",
                "characteristic_items",
                "characteristic_nominals",
            ],
            &[
                "qpid",
                "artifact",
                "step_document",
                "digital_model_entities",
                "other_objects",
                "characteristic_items",
                "characteristic_nominals",
            ],
        )?;
        check_content_artifact(
            plan.get("artifact")
                .ok_or_else(|| TraceError::MissingField {
                    scope: scope.clone(),
                    field: "artifact".into(),
                })?,
            &format!("{scope}.artifact"),
        )?;
        check_content_artifact(
            plan.get("step_document")
                .ok_or_else(|| TraceError::MissingField {
                    scope: scope.clone(),
                    field: "step_document".into(),
                })?,
            &format!("{scope}.step_document"),
        )?;

        let digital_entities = plan
            .get("digital_model_entities")
            .and_then(Value::as_array)
            .ok_or_else(|| {
                TraceError::MalformedObject(format!("{scope}.digital_model_entities"))
            })?;
        for (entity_index, entity) in digital_entities.iter().enumerate() {
            let entity_scope = format!("{scope}.digital_model_entities[{entity_index}]");
            check_fields(
                entity,
                &entity_scope,
                &["entity_id", "step_uuid", "step_entity_kind"],
                &["entity_id", "step_uuid", "step_entity_kind"],
            )?;
        }
        let other_objects = plan
            .get("other_objects")
            .and_then(Value::as_array)
            .ok_or_else(|| TraceError::MalformedObject(format!("{scope}.other_objects")))?;
        for (object_index, object) in other_objects.iter().enumerate() {
            let object_scope = format!("{scope}.other_objects[{object_index}]");
            check_fields(
                object,
                &object_scope,
                &["object_id", "object_kind"],
                &["object_id", "object_kind"],
            )?;
        }
        let items = plan
            .get("characteristic_items")
            .and_then(Value::as_array)
            .ok_or_else(|| TraceError::MalformedObject(format!("{scope}.characteristic_items")))?;
        for (item_index, item) in items.iter().enumerate() {
            let item_scope = format!("{scope}.characteristic_items[{item_index}]");
            check_fields(
                item,
                &item_scope,
                &["object_id", "characteristic_nominal_id"],
                &["object_id", "characteristic_nominal_id"],
            )?;
        }
        let nominals = plan
            .get("characteristic_nominals")
            .and_then(Value::as_array)
            .ok_or_else(|| {
                TraceError::MalformedObject(format!("{scope}.characteristic_nominals"))
            })?;
        for (nominal_index, nominal) in nominals.iter().enumerate() {
            let nominal_scope = format!("{scope}.characteristic_nominals[{nominal_index}]");
            check_fields(
                nominal,
                &nominal_scope,
                &["object_id", "characteristic_id", "entity_external_ids"],
                &["object_id", "characteristic_id", "entity_external_ids"],
            )?;
            let ids = nominal
                .get("entity_external_ids")
                .and_then(Value::as_array)
                .ok_or_else(|| {
                    TraceError::MalformedObject(format!("{nominal_scope}.entity_external_ids"))
                })?;
            if ids.iter().any(|id| {
                id.as_u64()
                    .filter(|value| *value > 0 && *value <= u32::MAX as u64)
                    .is_none()
            }) {
                return Err(TraceError::MalformedObject(format!(
                    "{nominal_scope}.entity_external_ids"
                )));
            }
        }
    }

    let step_documents = value
        .get("step_documents")
        .and_then(Value::as_array)
        .ok_or_else(|| TraceError::MalformedObject("case.step_documents".into()))?;
    for (index, document) in step_documents.iter().enumerate() {
        let scope = format!("case.step_documents[{index}]");
        check_fields(
            document,
            &scope,
            &["artifact", "entities"],
            &["artifact", "entities"],
        )?;
        check_content_artifact(
            document
                .get("artifact")
                .ok_or_else(|| TraceError::MissingField {
                    scope: scope.clone(),
                    field: "artifact".into(),
                })?,
            &format!("{scope}.artifact"),
        )?;
        let entities = document
            .get("entities")
            .and_then(Value::as_array)
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

fn check_fields(
    value: &Value,
    scope: &str,
    allowed: &[&str],
    required: &[&str],
) -> Result<(), TraceError> {
    let object = value
        .as_object()
        .ok_or_else(|| TraceError::MalformedObject(scope.into()))?;
    for key in object.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(TraceError::UnknownField {
                scope: scope.into(),
                field: key.clone(),
            });
        }
    }
    for field in required {
        if !object.contains_key(*field) {
            return Err(TraceError::MissingField {
                scope: scope.into(),
                field: (*field).into(),
            });
        }
    }
    Ok(())
}

fn check_content_artifact(value: &Value, scope: &str) -> Result<(), TraceError> {
    check_fields(
        value,
        scope,
        &[
            "reference",
            "sha256",
            "byte_length",
            "media_type",
            "format_id",
        ],
        &[
            "reference",
            "sha256",
            "byte_length",
            "media_type",
            "format_id",
        ],
    )?;
    let reference = value
        .get("reference")
        .ok_or_else(|| TraceError::MissingField {
            scope: scope.into(),
            field: "reference".into(),
        })?;
    check_fields(
        reference,
        &format!("{scope}.reference"),
        &["id", "kind"],
        &["id", "kind"],
    )
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn valid_uuid(value: &str) -> bool {
    if value.len() != 36 {
        return false;
    }
    value.bytes().enumerate().all(|(index, byte)| {
        if matches!(index, 8 | 13 | 18 | 23) {
            byte == b'-'
        } else {
            byte.is_ascii_hexdigit()
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

fn normalized_uuid(value: &str) -> String {
    value.to_ascii_lowercase()
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
        return Err(TraceError::InvalidArtifact(
            case.results.artifact.reference.id.clone(),
        ));
    }
    if case.results.measurements.is_empty() {
        return Err(TraceError::EmptyMeasurementSet);
    }

    let mut all_artifacts: BTreeSet<(String, String)> = BTreeSet::new();
    let mut add_artifact = |artifact: &ContentArtifactV1| -> Result<(), TraceError> {
        validate_content_artifact(artifact)?;
        let key = (
            artifact.reference.id.clone(),
            artifact.reference.kind.clone(),
        );
        if !all_artifacts.insert(key.clone()) {
            return Err(TraceError::DuplicateArtifact(key.0));
        }
        Ok(())
    };
    add_artifact(&case.results.artifact)?;

    let results_qpid_key = normalized_uuid(&case.results.qpid);
    let mut qpid_set = BTreeSet::new();
    qpid_set.insert(results_qpid_key);

    let mut results_local_ids = BTreeSet::new();
    let mut result_set_ids = BTreeSet::new();
    for result_set_id in &case.results.result_set_ids {
        if *result_set_id == 0 {
            return Err(TraceError::ZeroIdentifier(
                "MeasurementResultsSet id".into(),
            ));
        }
        if !result_set_ids.insert(*result_set_id) || !results_local_ids.insert(*result_set_id) {
            return Err(TraceError::DuplicateIdentifier(format!(
                "Results QIF id {}",
                result_set_id
            )));
        }
    }
    if result_set_ids.is_empty() {
        return Err(TraceError::InvalidIdentity("results.result_set_ids".into()));
    }

    let mut refs_by_id = BTreeMap::new();
    for external in &case.results.external_documents {
        if external.local_reference_id == 0 {
            return Err(TraceError::ZeroIdentifier(
                "external_documents.local_reference_id".into(),
            ));
        }
        if !valid_uuid(&external.plan_qpid) {
            return Err(TraceError::InvalidUuid(
                "external_documents.plan_qpid".into(),
            ));
        }
        if !results_local_ids.insert(external.local_reference_id) {
            return Err(TraceError::DuplicateIdentifier(format!(
                "Results QIF id {}",
                external.local_reference_id
            )));
        }
        if refs_by_id
            .insert(external.local_reference_id, external)
            .is_some()
        {
            return Err(TraceError::DuplicateIdentifier(format!(
                "external ref {}",
                external.local_reference_id
            )));
        }
        validate_content_artifact(&external.plan_artifact)?;
        if external.plan_artifact.reference.kind != "qif_plan"
            || external.plan_artifact.format_id != "QIF_3.0"
        {
            return Err(TraceError::InvalidArtifact(
                external.plan_artifact.reference.id.clone(),
            ));
        }
    }

    let mut plans_by_qpid = BTreeMap::new();
    for plan in &case.plans {
        if !valid_uuid(&plan.qpid) {
            return Err(TraceError::InvalidUuid("plans.qpid".into()));
        }
        let plan_key = normalized_uuid(&plan.qpid);
        if !qpid_set.insert(plan_key.clone()) {
            return Err(TraceError::DuplicateIdentifier(plan.qpid.clone()));
        }
        if plans_by_qpid.insert(plan_key, plan).is_some() {
            return Err(TraceError::DuplicateIdentifier(plan.qpid.clone()));
        }
        add_artifact(&plan.artifact)?;
        if plan.artifact.reference.kind != "qif_plan" || plan.artifact.format_id != "QIF_3.0" {
            return Err(TraceError::InvalidArtifact(
                plan.artifact.reference.id.clone(),
            ));
        }
        validate_content_artifact(&plan.step_document)?;
        if plan.step_document.reference.kind != "step_file"
            || !plan.step_document.format_id.starts_with("ISO_10303_242:")
        {
            return Err(TraceError::InvalidArtifact(
                plan.step_document.reference.id.clone(),
            ));
        }
    }

    for external in refs_by_id.values() {
        let plan = plans_by_qpid
            .get(&normalized_uuid(&external.plan_qpid))
            .ok_or_else(|| TraceError::MissingPlan(external.plan_qpid.clone()))?;
        if external.plan_artifact != plan.artifact {
            return Err(TraceError::PlanReferenceMismatch(
                external.local_reference_id,
            ));
        }
    }

    let mut step_by_artifact = BTreeMap::new();
    for document in &case.step_documents {
        add_artifact(&document.artifact)?;
        if document.artifact.reference.kind != "step_file"
            || !document.artifact.format_id.starts_with("ISO_10303_242:")
        {
            return Err(TraceError::InvalidArtifact(
                document.artifact.reference.id.clone(),
            ));
        }
        let step_key = (
            document.artifact.reference.id.clone(),
            document.artifact.reference.kind.clone(),
        );
        if step_by_artifact.insert(step_key, document).is_some() {
            return Err(TraceError::DuplicateArtifact(
                document.artifact.reference.id.clone(),
            ));
        }
        let mut uuids = BTreeSet::new();
        for entity in &document.entities {
            if !valid_uuid(&entity.uuid) {
                return Err(TraceError::InvalidUuid(format!(
                    "STEP entity {}",
                    entity.uuid
                )));
            }
            if !uuids.insert(normalized_uuid(&entity.uuid)) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "STEP UUID {}",
                    entity.uuid
                )));
            }
        }
    }

    for plan in &case.plans {
        let step_key = (
            plan.step_document.reference.id.clone(),
            plan.step_document.reference.kind.clone(),
        );
        let Some(step_document) = step_by_artifact.get(&step_key) else {
            return Err(TraceError::MissingStepDocument(plan.qpid.clone()));
        };
        if step_document.artifact != plan.step_document {
            return Err(TraceError::StepDocumentReferenceMismatch(plan.qpid.clone()));
        }

        // QIF item IDs, nominal IDs and DigitalModel Entity IDs share one
        // document-local identifier namespace and must not collide.
        let mut local_ids = BTreeSet::new();
        let mut digital_entities = BTreeMap::new();
        let mut mapped_step_uuids = BTreeSet::new();
        for mapping in &plan.digital_model_entities {
            if mapping.entity_id == 0 {
                return Err(TraceError::ZeroIdentifier(format!(
                    "DigitalModel Entity in {}",
                    plan.qpid
                )));
            }
            if !local_ids.insert(mapping.entity_id) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "QIF Plan id {} in {}",
                    mapping.entity_id, plan.qpid
                )));
            }
            if !valid_uuid(&mapping.step_uuid) {
                return Err(TraceError::InvalidUuid(format!(
                    "DigitalModel EntityId {}",
                    mapping.step_uuid
                )));
            }
            if !mapped_step_uuids.insert(normalized_uuid(&mapping.step_uuid)) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "DigitalModel STEP UUID {}",
                    mapping.step_uuid
                )));
            }
            digital_entities.insert(mapping.entity_id, mapping);
        }

        for other in &plan.other_objects {
            if other.object_id == 0 || other.object_kind.trim().is_empty() {
                return Err(TraceError::InvalidIdentity(format!(
                    "other Plan object {}",
                    other.object_id
                )));
            }
            if !local_ids.insert(other.object_id) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "QIF Plan id {} in {}",
                    other.object_id, plan.qpid
                )));
            }
        }

        let mut nominal_ids = BTreeMap::new();
        for nominal in &plan.characteristic_nominals {
            if nominal.object_id == 0 {
                return Err(TraceError::ZeroIdentifier(format!(
                    "CharacteristicNominal in {}",
                    plan.qpid
                )));
            }
            if !local_ids.insert(nominal.object_id) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "QIF Plan id {} in {}",
                    nominal.object_id, plan.qpid
                )));
            }
            if nominal.characteristic_id.trim().is_empty() {
                return Err(TraceError::InvalidIdentity(format!(
                    "CharacteristicNominal {}",
                    nominal.object_id
                )));
            }
            if nominal.entity_external_ids.is_empty() {
                return Err(TraceError::MissingEntityExternalIds {
                    nominal_id: nominal.object_id,
                });
            }
            let mut external_ids = BTreeSet::new();
            for entity_id in &nominal.entity_external_ids {
                if *entity_id == 0 {
                    return Err(TraceError::ZeroIdentifier(format!(
                        "EntityExternalIds in nominal {}",
                        nominal.object_id
                    )));
                }
                if !external_ids.insert(*entity_id) {
                    return Err(TraceError::DuplicateIdentifier(format!(
                        "EntityExternalId {} in nominal {}",
                        entity_id, nominal.object_id
                    )));
                }
                if !digital_entities.contains_key(entity_id) {
                    return Err(TraceError::MissingDigitalModelEntity {
                        qpid: plan.qpid.clone(),
                        entity_id: *entity_id,
                    });
                }
            }
            if nominal_ids.insert(nominal.object_id, nominal).is_some() {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "CharacteristicNominal {}",
                    nominal.object_id
                )));
            }
        }

        let mut characteristic_items = BTreeMap::new();
        for item in &plan.characteristic_items {
            if item.object_id == 0 || item.characteristic_nominal_id == 0 {
                return Err(TraceError::ZeroIdentifier(format!(
                    "CharacteristicItem in {}",
                    plan.qpid
                )));
            }
            if !local_ids.insert(item.object_id) {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "QIF Plan id {} in {}",
                    item.object_id, plan.qpid
                )));
            }
            if !nominal_ids.contains_key(&item.characteristic_nominal_id) {
                return Err(TraceError::MissingCharacteristicNominal {
                    qpid: plan.qpid.clone(),
                    nominal_id: item.characteristic_nominal_id,
                });
            }
            if characteristic_items.insert(item.object_id, item).is_some() {
                return Err(TraceError::DuplicateIdentifier(format!(
                    "CharacteristicItem {}",
                    item.object_id
                )));
            }
        }
    }

    let mut measurement_ids = BTreeSet::new();
    for measurement in &case.results.measurements {
        if measurement.measurement_id == 0 || measurement.result_set_id == 0 {
            return Err(TraceError::ZeroIdentifier(
                "measurement_id/result_set_id".into(),
            ));
        }
        if !measurement_ids.insert(measurement.measurement_id)
            || !results_local_ids.insert(measurement.measurement_id)
        {
            return Err(TraceError::DuplicateIdentifier(format!(
                "Results QIF id {}",
                measurement.measurement_id
            )));
        }
        if !result_set_ids.contains(&measurement.result_set_id) {
            return Err(TraceError::MissingResultSet(measurement.result_set_id));
        }
        if measurement.external_document_reference_id == 0 || measurement.xid == 0 {
            return Err(TraceError::ZeroIdentifier(format!(
                "measurement {}",
                measurement.measurement_id
            )));
        }

        let external = refs_by_id
            .get(&measurement.external_document_reference_id)
            .ok_or(TraceError::MissingExternalDocumentReference(
                measurement.external_document_reference_id,
            ))?;
        let plan = plans_by_qpid
            .get(&normalized_uuid(&external.plan_qpid))
            .ok_or_else(|| TraceError::MissingPlan(external.plan_qpid.clone()))?;
        if external.plan_artifact != plan.artifact {
            return Err(TraceError::PlanReferenceMismatch(
                external.local_reference_id,
            ));
        }
        let item = match plan
            .characteristic_items
            .iter()
            .find(|item| item.object_id == measurement.xid)
        {
            Some(item) => item,
            None => {
                if let Some(other) = plan
                    .other_objects
                    .iter()
                    .find(|item| item.object_id == measurement.xid)
                {
                    return Err(TraceError::WrongExternalObjectKind {
                        qpid: plan.qpid.clone(),
                        xid: measurement.xid,
                        object_kind: other.object_kind.clone(),
                    });
                }
                return Err(TraceError::MissingExternalObject {
                    qpid: plan.qpid.clone(),
                    xid: measurement.xid,
                });
            }
        };
        let nominal = plan
            .characteristic_nominals
            .iter()
            .find(|nominal| nominal.object_id == item.characteristic_nominal_id)
            .ok_or_else(|| TraceError::MissingCharacteristicNominal {
                qpid: plan.qpid.clone(),
                nominal_id: item.characteristic_nominal_id,
            })?;
        for entity_external_id in &nominal.entity_external_ids {
            let mapping = plan
                .digital_model_entities
                .iter()
                .find(|mapping| mapping.entity_id == *entity_external_id)
                .ok_or_else(|| TraceError::MissingDigitalModelEntity {
                    qpid: plan.qpid.clone(),
                    entity_id: *entity_external_id,
                })?;
            let step_key = (
                plan.step_document.reference.id.clone(),
                plan.step_document.reference.kind.clone(),
            );
            let step_document = step_by_artifact
                .get(&step_key)
                .ok_or_else(|| TraceError::MissingStepDocument(plan.qpid.clone()))?;
            let step_entity = step_document
                .entities
                .iter()
                .find(|entity| normalized_uuid(&entity.uuid) == normalized_uuid(&mapping.step_uuid))
                .ok_or_else(|| TraceError::MissingStepEntity {
                    document: step_document.artifact.reference.id.clone(),
                    uuid: mapping.step_uuid.clone(),
                })?;
            if step_entity.kind != mapping.step_entity_kind {
                return Err(TraceError::StepEntityKindMismatch {
                    document: step_document.artifact.reference.id.clone(),
                    uuid: mapping.step_uuid.clone(),
                });
            }
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
    expected.extend(
        case.step_documents
            .iter()
            .map(|document| &document.artifact),
    );
    if payloads.len() != expected.len() {
        return Err(TraceError::ArtifactPayloadSetMismatch);
    }
    let mut seen = BTreeSet::new();
    for (reference, bytes) in payloads {
        let key = (reference.id.clone(), reference.kind.clone());
        if !seen.insert(key.clone()) {
            return Err(TraceError::DuplicateArtifactPayload(key.0));
        }
        let artifact = expected
            .iter()
            .copied()
            .find(|item| item.reference.id == reference.id && item.reference.kind == reference.kind)
            .ok_or_else(|| TraceError::UnexpectedArtifactPayload(key.0))?;
        verify_artifact_bytes(artifact, bytes).map_err(TraceError::ArtifactVerification)?;
    }
    for artifact in &expected {
        if !seen.contains(&(
            artifact.reference.id.clone(),
            artifact.reference.kind.clone(),
        )) {
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
    validate_trace_case(case)?;
    let refs: BTreeMap<_, _> = case
        .results
        .external_documents
        .iter()
        .map(|item| (item.local_reference_id, item))
        .collect();
    let plans: BTreeMap<_, _> = case
        .plans
        .iter()
        .map(|plan| (normalized_uuid(&plan.qpid), plan))
        .collect();
    let steps: BTreeMap<_, _> = case
        .step_documents
        .iter()
        .map(|document| {
            (
                (
                    document.artifact.reference.id.clone(),
                    document.artifact.reference.kind.clone(),
                ),
                document,
            )
        })
        .collect();

    let mut resolved = Vec::new();
    for measurement in &case.results.measurements {
        let external = refs[&measurement.external_document_reference_id];
        let plan = plans[&normalized_uuid(&external.plan_qpid)];
        let item = plan
            .characteristic_items
            .iter()
            .find(|item| item.object_id == measurement.xid)
            .expect("validated CharacteristicItem xId");
        let nominal = plan
            .characteristic_nominals
            .iter()
            .find(|nominal| nominal.object_id == item.characteristic_nominal_id)
            .expect("validated CharacteristicNominalId");
        let step_key = (
            plan.step_document.reference.id.clone(),
            plan.step_document.reference.kind.clone(),
        );
        let step_document = steps[&step_key];

        let mut step_entities = Vec::new();
        for entity_external_id in &nominal.entity_external_ids {
            let mapping = plan
                .digital_model_entities
                .iter()
                .find(|mapping| mapping.entity_id == *entity_external_id)
                .expect("validated EntityExternalId");
            let step_entity = step_document
                .entities
                .iter()
                .find(|entity| normalized_uuid(&entity.uuid) == normalized_uuid(&mapping.step_uuid))
                .expect("validated STEP UUID mapping");
            step_entities.push(ResolvedStepEntityV1 {
                entity_external_id: *entity_external_id,
                step_uuid: step_entity.uuid.clone(),
                step_entity_kind: step_entity.kind,
                step_document: step_document.artifact.clone(),
            });
        }
        step_entities.sort_by(|left, right| left.entity_external_id.cmp(&right.entity_external_id));

        resolved.push(ResolvedMeasurementTraceV1 {
            measurement_id: measurement.measurement_id.clone(),
            result_set_id: measurement.result_set_id.clone(),
            status: measurement.status,
            results_qpid: case.results.qpid.clone(),
            external_document_reference_id: external.local_reference_id,
            plan_qpid: plan.qpid.clone(),
            plan_artifact: plan.artifact.clone(),
            plan_characteristic_item_id: item.object_id,
            characteristic_nominal_id: nominal.object_id,
            characteristic_id: nominal.characteristic_id.clone(),
            step_entities,
        });
    }
    resolved.sort_by(|left, right| {
        (&left.result_set_id, &left.measurement_id)
            .cmp(&(&right.result_set_id, &right.measurement_id))
    });
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
        let sha256 = digest
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        ContentArtifactV1 {
            reference: ArtifactRef {
                id: format!("sha256:{sha256}"),
                kind: kind.into(),
            },
            sha256,
            byte_length: bytes.len() as u64,
            media_type: media.into(),
            format_id: format.into(),
        }
    }

    fn fixture() -> (QifExternalReferenceTraceCaseV1, Vec<(ArtifactRef, Vec<u8>)>) {
        let results_bytes = b"<QIFDocument kind=\"Results\">synthetic</QIFDocument>";
        let plan_a_bytes = b"<QIFDocument kind=\"Plan\">synthetic A</QIFDocument>";
        let plan_b_bytes = b"<QIFDocument kind=\"Plan\">synthetic B</QIFDocument>";
        let step_bytes = b"ISO-10303-21; synthetic STEP source";
        let results_artifact = artifact("qif_results", "QIF_3.0", "application/xml", results_bytes);
        let plan_a_artifact = artifact("qif_plan", "QIF_3.0", "application/xml", plan_a_bytes);
        let plan_b_artifact = artifact("qif_plan", "QIF_3.0", "application/xml", plan_b_bytes);
        let step_artifact = artifact(
            "step_file",
            "ISO_10303_242:2025",
            "application/step",
            step_bytes,
        );

        let plan_a = QifPlanDocumentV1 {
            qpid: "20000000-0000-4000-8000-000000000002".into(),
            artifact: plan_a_artifact.clone(),
            step_document: step_artifact.clone(),
            digital_model_entities: vec![
                DigitalModelExternalEntityV1 {
                    entity_id: 9991,
                    step_uuid: "40000000-0000-4000-8000-000000000004".into(),
                    step_entity_kind: StepEntityKind::Characteristic,
                },
                DigitalModelExternalEntityV1 {
                    entity_id: 9992,
                    step_uuid: "50000000-0000-4000-8000-000000000005".into(),
                    step_entity_kind: StepEntityKind::GeometricFeature,
                },
                DigitalModelExternalEntityV1 {
                    entity_id: 9993,
                    step_uuid: "70000000-0000-4000-8000-000000000007".into(),
                    step_entity_kind: StepEntityKind::Datum,
                },
            ],
            other_objects: vec![OtherPlanObjectV1 {
                object_id: 304,
                object_kind: "FeatureItem".into(),
            }],
            characteristic_items: vec![
                PlanCharacteristicItemV1 {
                    object_id: 301,
                    characteristic_nominal_id: 401,
                },
                PlanCharacteristicItemV1 {
                    object_id: 302,
                    characteristic_nominal_id: 402,
                },
                PlanCharacteristicItemV1 {
                    object_id: 303,
                    characteristic_nominal_id: 403,
                },
            ],
            characteristic_nominals: vec![
                PlanCharacteristicNominalV1 {
                    object_id: 401,
                    characteristic_id: "characteristic:hole-dia".into(),
                    entity_external_ids: vec![9991],
                },
                PlanCharacteristicNominalV1 {
                    object_id: 402,
                    characteristic_id: "characteristic:surface".into(),
                    entity_external_ids: vec![9992],
                },
                PlanCharacteristicNominalV1 {
                    object_id: 403,
                    characteristic_id: "characteristic:composite".into(),
                    entity_external_ids: vec![9991, 9993],
                },
            ],
        };

        let plan_b = QifPlanDocumentV1 {
            qpid: "30000000-0000-4000-8000-000000000003".into(),
            artifact: plan_b_artifact.clone(),
            step_document: step_artifact.clone(),
            digital_model_entities: vec![DigitalModelExternalEntityV1 {
                entity_id: 9991,
                step_uuid: "60000000-0000-4000-8000-000000000006".into(),
                step_entity_kind: StepEntityKind::Datum,
            }],
            other_objects: Vec::new(),
            characteristic_items: vec![PlanCharacteristicItemV1 {
                object_id: 301,
                characteristic_nominal_id: 401,
            }],
            characteristic_nominals: vec![PlanCharacteristicNominalV1 {
                object_id: 401,
                characteristic_id: "characteristic:flatness".into(),
                entity_external_ids: vec![9991],
            }],
        };

        let case = QifExternalReferenceTraceCaseV1 {
            schema: TRACE_SCHEMA.into(),
            benchmark_id: "AERO-QIF-XID-001".into(),
            case_id: "synthetic-test8-001".into(),
            results: ResultsDocumentV1 {
                qpid: "10000000-0000-4000-8000-000000000001".into(),
                artifact: results_artifact.clone(),
                result_set_ids: vec![701, 702, 703],
                external_documents: vec![
                    ExternalQifDocumentReferenceV1 {
                        local_reference_id: 7,
                        plan_qpid: plan_a.qpid.clone(),
                        plan_artifact: plan_a_artifact.clone(),
                        uri: Some("synthetic://plan-a".into()),
                    },
                    ExternalQifDocumentReferenceV1 {
                        local_reference_id: 8,
                        plan_qpid: plan_b.qpid.clone(),
                        plan_artifact: plan_b_artifact.clone(),
                        uri: Some("synthetic://plan-b".into()),
                    },
                ],
                measurements: vec![
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: 901,
                        result_set_id: 701,
                        external_document_reference_id: 7,
                        xid: 301,
                        status: MeasurementStatus::Passed,
                    },
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: 902,
                        result_set_id: 701,
                        external_document_reference_id: 7,
                        xid: 302,
                        status: MeasurementStatus::Failed,
                    },
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: 903,
                        result_set_id: 702,
                        external_document_reference_id: 8,
                        xid: 301,
                        status: MeasurementStatus::NotEvaluated,
                    },
                    CharacteristicMeasurementReferenceV1 {
                        measurement_id: 904,
                        result_set_id: 703,
                        external_document_reference_id: 7,
                        xid: 303,
                        status: MeasurementStatus::Unknown,
                    },
                ],
            },
            plans: vec![plan_a, plan_b],
            step_documents: vec![StepDocumentV1 {
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
                    StepEntityV1 {
                        uuid: "70000000-0000-4000-8000-000000000007".into(),
                        kind: StepEntityKind::Datum,
                    },
                ],
            }],
        };
        let payloads = vec![
            (results_artifact.reference.clone(), results_bytes.to_vec()),
            (plan_a_artifact.reference.clone(), plan_a_bytes.to_vec()),
            (plan_b_artifact.reference.clone(), plan_b_bytes.to_vec()),
            (step_artifact.reference.clone(), step_bytes.to_vec()),
        ];
        (case, payloads)
    }

    fn refs_and_bytes(payloads: &[(ArtifactRef, Vec<u8>)]) -> Vec<(&ArtifactRef, &[u8])> {
        payloads
            .iter()
            .map(|(reference, bytes)| (reference, bytes.as_slice()))
            .collect()
    }

    fn resolved<'a>(
        report: &'a TraceVerificationReport,
        measurement_id: u32,
    ) -> &'a ResolvedMeasurementTraceV1 {
        report
            .resolved_measurements
            .iter()
            .find(|item| item.measurement_id == measurement_id)
            .expect("measurement trace exists")
    }

    #[test]
    fn normalized_test8_fixture_replays_exact_payload_bytes() {
        let case =
            parse_trace_case_json(include_bytes!("../fixtures/test8-normalized-trace-v1.json"))
                .expect("checked-in normalized trace fixture parses");
        let payloads = [
            (
                &case.results.artifact.reference,
                include_bytes!("../fixtures/results.qif").as_slice(),
            ),
            (
                &case.plans[0].artifact.reference,
                include_bytes!("../fixtures/plan-a.qif").as_slice(),
            ),
            (
                &case.plans[1].artifact.reference,
                include_bytes!("../fixtures/plan-b.qif").as_slice(),
            ),
            (
                &case.step_documents[0].artifact.reference,
                include_bytes!("../fixtures/source.step").as_slice(),
            ),
        ];
        let report = verify_and_resolve_trace_case(&case, &payloads)
            .expect("all checked-in Results, Plan and STEP bytes and links match");
        assert_eq!(report.resolved_measurements.len(), 4);
        assert_eq!(report.checked_artifacts, 4);
        assert_eq!(resolved(&report, 902).status, MeasurementStatus::Failed);
        assert_eq!(resolved(&report, 904).step_entities.len(), 2);
    }

    #[test]
    fn resolves_xid_through_item_nominal_external_id_and_step_uuid() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let trace = resolved(&report, 901);
        assert_eq!(trace.plan_characteristic_item_id, 301);
        assert_eq!(trace.characteristic_nominal_id, 401);
        assert_eq!(trace.characteristic_id, "characteristic:hole-dia");
        assert_eq!(trace.step_entities.len(), 1);
        assert_eq!(trace.step_entities[0].entity_external_id, 9991);
        assert_eq!(
            trace.step_entities[0].step_uuid,
            "40000000-0000-4000-8000-000000000004"
        );
    }

    #[test]
    fn failed_measurement_remains_traceable_to_its_nominal_and_step_entity() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let trace = resolved(&report, "measurement:fail");
        assert_eq!(trace.status, MeasurementStatus::Failed);
        assert_eq!(trace.characteristic_id, "characteristic:surface");
        assert_eq!(
            trace.step_entities[0].step_uuid,
            "50000000-0000-4000-8000-000000000005"
        );
    }

    #[test]
    fn same_xid_in_two_plans_resolves_in_the_selected_plan_namespace() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let first = resolved(&report, "measurement:pass");
        let second = resolved(&report, 903);
        assert_eq!(first.plan_characteristic_item_id, 301);
        assert_eq!(second.plan_characteristic_item_id, 301);
        assert_ne!(first.plan_qpid, second.plan_qpid);
        assert_ne!(
            first.step_entities[0].step_uuid,
            second.step_entities[0].step_uuid
        );
    }

    #[test]
    fn local_external_reference_id_is_not_collapsed_into_xid() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].local_reference_id = 301;
        case.results.measurements[0].external_document_reference_id = 301;
        case.results.measurements[0].xid = 301;
        assert!(validate_trace_case(&case).is_ok());
    }

    #[test]
    fn multiple_entity_external_ids_preserve_all_step_mappings() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let trace = resolved(&report, "measurement:multi-entity");
        assert_eq!(trace.step_entities.len(), 2);
        assert_eq!(
            trace
                .step_entities
                .iter()
                .map(|item| item.entity_external_id)
                .collect::<Vec<_>>(),
            vec![9991, 9993]
        );
    }

    #[test]
    fn omitted_external_qif_uri_is_admitted() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].uri = None;
        let bytes = serde_json::to_vec(&case).unwrap();
        let parsed = parse_trace_case_json(&bytes).unwrap();
        assert_eq!(parsed.results.external_documents[0].uri, None);
        assert!(validate_trace_case(&parsed).is_ok());
    }

    #[test]
    fn results_document_qif_ids_are_unique_across_result_sets_refs_and_measurements() {
        let (mut case, _) = fixture();
        case.results.measurements[0].measurement_id = 701;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::DuplicateIdentifier(_))
        ));
    }

    #[test]
    fn wrong_type_external_xid_is_not_reported_as_missing() {
        let (mut case, _) = fixture();
        case.results.measurements[0].xid = 304;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::WrongExternalObjectKind {
                xid: 304,
                object_kind,
                ..
            }) if object_kind == "FeatureItem"
        ));
    }

    #[test]
    fn missing_measurement_result_set_is_rejected() {
        let (mut case, _) = fixture();
        case.results.measurements[0].result_set_id = 799;
        assert_eq!(
            validate_trace_case(&case),
            Err(TraceError::MissingResultSet(799))
        );
    }

    #[test]
    fn wrong_local_external_reference_is_rejected() {
        let (mut case, _) = fixture();
        case.results.measurements[0].external_document_reference_id = 999;
        assert_eq!(
            validate_trace_case(&case),
            Err(TraceError::MissingExternalDocumentReference(999))
        );
    }

    #[test]
    fn missing_external_plan_qpid_is_rejected() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].plan_qpid =
            "70000000-0000-4000-8000-000000000007".into();
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::MissingPlan(_))
        ));
    }

    #[test]
    fn exact_plan_artifact_must_match_external_reference() {
        let (mut case, _) = fixture();
        case.results.external_documents[0].plan_artifact.byte_length += 1;
        assert_eq!(
            validate_trace_case(&case),
            Err(TraceError::PlanReferenceMismatch(7))
        );
    }

    #[test]
    fn missing_xid_in_the_selected_plan_is_rejected() {
        let (mut case, _) = fixture();
        case.results.measurements[0].xid = 999;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::MissingExternalObject { xid: 999, .. })
        ));
    }

    #[test]
    fn missing_characteristic_nominal_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].characteristic_items[0].characteristic_nominal_id = 777;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::MissingCharacteristicNominal {
                nominal_id: 777,
                ..
            })
        ));
    }

    #[test]
    fn missing_entity_external_id_mapping_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].characteristic_nominals[0].entity_external_ids = vec![12345];
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::MissingDigitalModelEntity {
                entity_id: 12345,
                ..
            })
        ));
    }

    #[test]
    fn nominal_without_entity_external_ids_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].characteristic_nominals[0]
            .entity_external_ids
            .clear();
        assert_eq!(
            validate_trace_case(&case),
            Err(TraceError::MissingEntityExternalIds { nominal_id: 401 })
        );
    }

    #[test]
    fn duplicate_qif_ids_across_plan_tables_are_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].characteristic_items[0].object_id = 401;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::DuplicateIdentifier(_))
        ));
    }

    #[test]
    fn duplicate_step_uuid_is_rejected() {
        let (mut case, _) = fixture();
        case.step_documents[0].entities[1].uuid = case.step_documents[0].entities[0].uuid.clone();
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::DuplicateIdentifier(_))
        ));
    }

    #[test]
    fn step_uuid_does_not_accept_an_xid_integer() {
        let (mut case, _) = fixture();
        case.plans[0].digital_model_entities[0].step_uuid = "301".into();
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::InvalidUuid(_))
        ));
    }

    #[test]
    fn missing_step_mapping_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].digital_model_entities[0].step_uuid =
            "80000000-0000-4000-8000-000000000008".into();
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::MissingStepEntity { .. })
        ));
    }

    #[test]
    fn step_entity_kind_mismatch_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[0].digital_model_entities[0].step_entity_kind = StepEntityKind::Datum;
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::StepEntityKindMismatch { .. })
        ));
    }

    #[test]
    fn qpid_and_step_uuid_comparison_is_case_insensitive() {
        let (mut case, _) = fixture();
        case.plans[0].qpid = case.plans[0].qpid.to_ascii_uppercase();
        case.results.external_documents[0].plan_qpid = case.results.external_documents[0]
            .plan_qpid
            .to_ascii_uppercase();
        case.plans[0].digital_model_entities[0].step_uuid = case.plans[0].digital_model_entities[0]
            .step_uuid
            .to_ascii_uppercase();
        case.step_documents[0].entities[0].uuid =
            case.step_documents[0].entities[0].uuid.to_ascii_uppercase();
        assert!(validate_trace_case(&case).is_ok());
    }

    #[test]
    fn duplicate_qpid_with_case_variation_is_rejected() {
        let (mut case, _) = fixture();
        case.plans[1].qpid = case.plans[0].qpid.to_ascii_uppercase();
        assert!(matches!(
            validate_trace_case(&case),
            Err(TraceError::DuplicateIdentifier(_))
        ));
    }

    #[test]
    fn result_set_and_measurement_order_do_not_change_normalized_trace() {
        let (case, _) = fixture();
        let original = resolve_without_bytes(&case).unwrap();
        let mut permuted = case.clone();
        permuted.results.measurements.reverse();
        let reversed = resolve_without_bytes(&permuted).unwrap();
        assert_eq!(original, reversed);
    }

    #[test]
    fn unknown_nested_artifact_field_is_rejected() {
        let (case, _) = fixture();
        let mut value = serde_json::to_value(&case).unwrap();
        value["plans"][0]["artifact"]["reference"]["trust"] = json!("trusted");
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(matches!(
            parse_trace_case_json(&bytes),
            Err(TraceError::UnknownField { .. })
        ));
    }

    #[test]
    fn exact_payload_bytes_are_required() {
        let (case, mut payloads) = fixture();
        payloads[0].1.push(b'X');
        assert!(matches!(
            verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)),
            Err(TraceError::ArtifactVerification(
                ReceiptError::ArtifactByteLengthMismatch(_)
            ))
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

    #[test]
    fn result_set_identity_is_preserved() {
        let (case, payloads) = fixture();
        let report = verify_and_resolve_trace_case(&case, &refs_and_bytes(&payloads)).unwrap();
        let trace = resolved(&report, "measurement:second-plan");
        assert_eq!(trace.result_set_id, "result-set:2");
    }
}
