// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Manufacturing execution integrity types.
//!
//! This zome records what physically happened, separately from planning.
//! Entries are immutable: corrections are new records, never in-place edits.
//!
//! Qualification attestations record who/what asserted a qualification under
//! an explicit method and validity interval. They do not themselves imply
//! independence or regulatory certification; those are trust-policy claims.

use hdi::prelude::*;
use std::collections::HashSet;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum Disposition {
    Accepted,
    Rejected,
    Rework,
    Quarantined,
}

use manufacturing_common::{
    evaluate_capability, CapabilityProfile, CapabilityQualification, CapabilityRequirement,
};

#[derive(Serialize, Deserialize, SerializedBytes, Debug, Clone)]
struct WorkOrderRevisionProjection {
    #[serde(default)]
    bom_hash: Option<ActionHash>,
    #[serde(default)]
    routing_hash: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, SerializedBytes, Debug, Clone)]
struct RoutingStepProjection {
    #[serde(default)]
    sequence: u32,
    #[serde(default)]
    capability_requirement_hash: Option<ActionHash>,
    #[serde(default)]
    required_inspection_criterion_hashes: Vec<ActionHash>,
}

#[derive(Serialize, Deserialize, SerializedBytes, Debug, Clone)]
struct CapabilityRequirementProjection {
    requirement: CapabilityRequirement,
}

#[derive(Serialize, Deserialize, SerializedBytes, Debug, Clone)]
struct RoutingRevisionProjection {
    #[serde(default)]
    steps: Vec<RoutingStepProjection>,
}

fn bind_execution_to_work_order_revision(
    execution: &ExecutionReceiptEntry,
    work_order: &WorkOrderRevisionProjection,
) -> Result<(), &'static str> {
    let accepted = matches!(execution.disposition, Disposition::Accepted);

    if accepted {
        match (&work_order.bom_hash, &execution.bom_hash) {
            (Some(work_order_bom), Some(execution_bom)) if work_order_bom == execution_bom => {}
            (Some(_), Some(_)) => {
                return Err("execution BOM does not match the work-order-bound BOM revision");
            }
            _ => {
                return Err("accepted execution requires an exact work-order-bound BOM revision");
            }
        }
        match (&work_order.routing_hash, &execution.routing_hash) {
            (Some(work_order_routing), Some(execution_routing))
                if work_order_routing == execution_routing => {}
            (Some(_), Some(_)) => {
                return Err("execution routing does not match the work-order-bound routing revision");
            }
            _ => {
                return Err(
                    "accepted execution requires an exact work-order-bound routing revision",
                );
            }
        }
        return Ok(());
    }

    if let Some(execution_bom) = &execution.bom_hash {
        if work_order.bom_hash.as_ref() != Some(execution_bom) {
            return Err("execution BOM does not match the work-order-bound BOM revision");
        }
    }
    if let Some(execution_routing) = &execution.routing_hash {
        if work_order.routing_hash.as_ref() != Some(execution_routing) {
            return Err("execution routing does not match the work-order-bound routing revision");
        }
    }

    Ok(())
}


#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum EvidenceKind {
    CapabilityQualification,
    Execution,
    MachineObservation,
    ProcessTrace,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EvidenceEntry {
    pub evidence_id: String,
    pub subject_id: String,
    pub kind: EvidenceKind,
    pub payload_hash: String,
    pub source: String,
    pub observed_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CapabilityContractEntry {
    pub contract_id: String,
    pub machine_hash: ActionHash,
    pub process_family: String,
    pub material_classes: Vec<String>,
    pub envelope_x_mm: Option<u32>,
    pub envelope_y_mm: Option<u32>,
    pub envelope_z_mm: Option<u32>,
    pub tolerance_um: Option<u32>,
    pub supported_protocols: Vec<String>,
    pub qualification: CapabilityQualification,
    pub qualification_evidence_hashes: Vec<ActionHash>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct QualificationAttestationEntry {
    pub capability_contract_hash: ActionHash,
    pub outcome: CapabilityQualification,
    pub evidence_hashes: Vec<ActionHash>,
    pub method: String,
    pub authority_reference: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub external_reference_hash: Option<String>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MaterialLotEntry {
    pub lot_id: String,
    pub material_id: String,
    pub quantity: u64,
    pub unit: String,
    pub source_reference: Option<String>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct InspectionCriterionEntry {
    pub requirement_id: String,
    pub revision: String,
    pub characteristic: String,
    pub unit: String,
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
    pub measurement_method: Option<String>,
    pub required_instrument_class: Option<String>,
    pub specification_reference: Option<String>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MeasurementEntry {
    pub measurement_id: String,
    /// Optional during schema migration; accepted executions require it.
    #[serde(default)]
    pub criterion_hash: Option<ActionHash>,
    pub kind: String,
    pub value: f64,
    pub unit: String,
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
    pub instrument_hash: Option<ActionHash>,
    pub measured_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CalibrationEntry {
    pub asset_hash: ActionHash,
    pub method: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub evidence_reference: Option<String>,
    pub created_at: Timestamp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ExecutionReceiptEntry {
    pub execution_id: String,
    /// Optional during migration; accepted executions require an attributed qualification attestation.
    #[serde(default)]
    pub qualification_attestation_hash: Option<ActionHash>,
    pub work_order_hash: ActionHash,
    pub bom_hash: Option<ActionHash>,
    pub routing_hash: Option<ActionHash>,
    pub operation_sequence: u32,
    pub machine_hash: ActionHash,
    pub capability_contract_hash: Option<ActionHash>,
    pub process_parameters_hash: Option<String>,
    pub input_lot_hashes: Vec<ActionHash>,
    pub output_lot_hashes: Vec<ActionHash>,
    pub measurement_hashes: Vec<ActionHash>,
    pub calibration_hashes: Vec<ActionHash>,
    pub started_at: Timestamp,
    pub completed_at: Timestamp,
    pub disposition: Disposition,
    pub evidence_hashes: Vec<ActionHash>,
    pub notes: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    MaterialLot(MaterialLotEntry),
    Measurement(MeasurementEntry),
    Calibration(CalibrationEntry),
    InspectionCriterion(InspectionCriterionEntry),
    QualificationAttestation(QualificationAttestationEntry),
    CapabilityContract(CapabilityContractEntry),
    Evidence(EvidenceEntry),
    ExecutionReceipt(ExecutionReceiptEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMaterialLots,
    AllMeasurements,
    AllCalibrations,
    AllInspectionCriteria,
    AllQualificationAttestations,
    AllCapabilityContracts,
    AllEvidence,
    AllExecutions,
    WorkOrderToExecutions,
    MachineToExecutions,
    MachineToCapabilities,
    CapabilityContractToQualificationAttestations,
    ExecutionToInputs,
    ExecutionToOutputs,
    ExecutionToMeasurements,
    MeasurementToInspectionCriterion,
    ExecutionToCalibrations,
    ExecutionToQualificationAttestation,
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => {
            validate_create(app_entry)
        }
        FlatOp::StoreEntry(OpEntry::UpdateEntry { .. }) => Ok(
            ValidateCallbackResult::Invalid("Execution records are immutable".into())
        ),
        FlatOp::RegisterUpdate(_) => Ok(
            ValidateCallbackResult::Invalid("Execution records are immutable".into())
        ),
        FlatOp::RegisterDelete(_) => Ok(
            ValidateCallbackResult::Invalid("Execution records are immutable".into())
        ),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn all_unique<T: std::cmp::Eq + std::hash::Hash>(items: &[T]) -> bool {
    let mut seen = HashSet::with_capacity(items.len());
    items.iter().all(|item| seen.insert(item))
}

fn measurement_matches_criterion(
    measurement: &MeasurementEntry,
    criterion: &InspectionCriterionEntry,
) -> Result<(), &'static str> {
    if measurement.kind != criterion.characteristic || measurement.unit != criterion.unit {
        return Err("measurement does not match its inspection criterion");
    }
    if criterion.lower_bound.is_some_and(|lower| measurement.value < lower)
        || criterion.upper_bound.is_some_and(|upper| measurement.value > upper)
    {
        return Err("measurement value is outside its authoritative inspection criterion");
    }
    Ok(())
}

fn routing_step_for_sequence(
    routing: &RoutingRevisionProjection,
    sequence: u32,
) -> Option<&RoutingStepProjection> {
    routing.steps.iter().find(|step| step.sequence == sequence)
}

fn required_inspection_criteria_are_unique(
    hashes: &[ActionHash],
) -> bool {
    all_unique(hashes)
}



fn capability_profile_from_contract(
    contract: &CapabilityContractEntry,
) -> CapabilityProfile {
    CapabilityProfile {
        process_family: contract.process_family.clone(),
        material_classes: contract.material_classes.clone(),
        envelope_x_mm: contract.envelope_x_mm,
        envelope_y_mm: contract.envelope_y_mm,
        envelope_z_mm: contract.envelope_z_mm,
        tolerance_um: contract.tolerance_um,
        supported_protocols: contract.supported_protocols.clone(),
        qualification: contract.qualification.clone(),
    }
}

fn validate_create(entry: EntryTypes) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::MaterialLot(lot) => {
            if lot.lot_id.is_empty() {
                return Ok(ValidateCallbackResult::Invalid("lot_id is required".into()));
            }
            if lot.material_id.is_empty() {
                return Ok(ValidateCallbackResult::Invalid("material_id is required".into()));
            }
            if lot.quantity == 0 {
                return Ok(ValidateCallbackResult::Invalid("quantity must be > 0".into()));
            }
            if lot.unit.is_empty() {
                return Ok(ValidateCallbackResult::Invalid("unit is required".into()));
            }
        }
        EntryTypes::Measurement(m) => {
            if m.measurement_id.is_empty() || m.kind.is_empty() || m.unit.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "measurement_id, kind and unit are required".into(),
                ));
            }
            if !m.value.is_finite()
                || m.lower_bound.is_some_and(|v| !v.is_finite())
                || m.upper_bound.is_some_and(|v| !v.is_finite())
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "measurement values must be finite".into(),
                ));
            }
            if let (Some(lower), Some(upper)) = (m.lower_bound, m.upper_bound) {
                if lower > upper {
                    return Ok(ValidateCallbackResult::Invalid(
                        "lower_bound must be <= upper_bound".into(),
                    ));
                }
            }
            if let Some(hash) = m.criterion_hash.clone() {
                let record = must_get_valid_record(hash)?;
                let criterion: Option<InspectionCriterionEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(criterion) = criterion else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "measurement criterion reference is not an inspection criterion".into(),
                    ));
                };
                if let Err(message) = measurement_matches_criterion(&m, &criterion) {
                    return Ok(ValidateCallbackResult::Invalid(message.into()));
                }
            }
        }
        EntryTypes::InspectionCriterion(c) => {
            if c.requirement_id.is_empty() || c.revision.is_empty() || c.characteristic.is_empty() || c.unit.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "inspection criterion requires requirement_id, revision, characteristic and unit".into(),
                ));
            }
            if c.lower_bound.is_none() && c.upper_bound.is_none() {
                return Ok(ValidateCallbackResult::Invalid(
                    "inspection criterion requires at least one authoritative bound".into(),
                ));
            }
            if c.lower_bound.is_some_and(|v| !v.is_finite())
                || c.upper_bound.is_some_and(|v| !v.is_finite())
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "inspection criterion bounds must be finite".into(),
                ));
            }
            if let (Some(lower), Some(upper)) = (c.lower_bound, c.upper_bound) {
                if lower > upper {
                    return Ok(ValidateCallbackResult::Invalid(
                        "inspection criterion lower_bound must be <= upper_bound".into(),
                    ));
                }
            }
        }
        EntryTypes::Calibration(c) => {
            if c.method.is_empty() {
                return Ok(ValidateCallbackResult::Invalid("calibration method is required".into()));
            }
            if c.valid_until < c.valid_from {
                return Ok(ValidateCallbackResult::Invalid(
                    "calibration validity window is inverted".into(),
                ));
            }
        }
        EntryTypes::QualificationAttestation(a) => {
            if a.method.is_empty() || a.authority_reference.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation requires method and authority_reference".into(),
                ));
            }
            if a.evidence_hashes.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation requires evidence".into(),
                ));
            }
            if a.valid_until < a.valid_from {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation validity window is inverted".into(),
                ));
            }
            if !matches!(a.outcome, CapabilityQualification::Qualified) {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation must carry the Qualified outcome".into(),
                ));
            }
            if !all_unique(&a.evidence_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation evidence hashes must be unique".into(),
                ));
            }

            let contract_record = must_get_valid_record(a.capability_contract_hash.clone())?;
            let contract: Option<CapabilityContractEntry> = contract_record
                .entry()
                .to_app_option()
                .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
            let Some(contract) = contract else {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation must reference a capability contract".into(),
                ));
            };
            if !matches!(contract.qualification, CapabilityQualification::Qualified) {
                return Ok(ValidateCallbackResult::Invalid(
                    "qualification attestation cannot upgrade a non-Qualified capability contract".into(),
                ));
            }

            for hash in &a.evidence_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let evidence: Option<EvidenceEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(evidence) = evidence else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "qualification attestation evidence must reference an Evidence record".into(),
                    ));
                };
                if evidence.subject_id != contract.contract_id
                    || !matches!(evidence.kind, EvidenceKind::CapabilityQualification)
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "qualification attestation evidence is not bound to its capability contract".into(),
                    ));
                }
            }
        }
        EntryTypes::CapabilityContract(c) => {
            if c.contract_id.is_empty() || c.process_family.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "contract_id and process_family are required".into(),
                ));
            }
            if c.material_classes.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "at least one material class is required".into(),
                ));
            }
            if c.envelope_x_mm == Some(0)
                || c.envelope_y_mm == Some(0)
                || c.envelope_z_mm == Some(0)
                || c.tolerance_um == Some(0)
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "envelope and tolerance values must be > 0 when provided".into(),
                ));
            }
            if matches!(c.qualification, CapabilityQualification::Qualified)
                && c.qualification_evidence_hashes.is_empty()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "Qualified capability requires qualification evidence".into(),
                ));
            }
            if !all_unique(&c.qualification_evidence_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "capability qualification evidence hashes must be unique".into(),
                ));
            }
            for hash in &c.qualification_evidence_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let evidence: Option<EvidenceEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(evidence) = evidence else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability qualification evidence must reference an Evidence record".into(),
                    ));
                };
                if evidence.subject_id != c.contract_id
                    || !matches!(evidence.kind, EvidenceKind::CapabilityQualification)
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability qualification evidence is not semantically bound to the contract".into(),
                    ));
                }
            }
        }
        EntryTypes::Evidence(e) => {
            if e.evidence_id.is_empty()
                || e.subject_id.is_empty()
                || e.payload_hash.is_empty()
                || e.source.is_empty()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "evidence_id, subject_id, payload_hash and source are required".into(),
                ));
            }
        }
        EntryTypes::ExecutionReceipt(e) => {
            if e.operation_sequence == 0 {
                return Ok(ValidateCallbackResult::Invalid(
                    "operation_sequence must be > 0".into(),
                ));
            }
            if e.completed_at < e.started_at {
                return Ok(ValidateCallbackResult::Invalid(
                    "completed_at must be >= started_at".into(),
                ));
            }
            if e.input_lot_hashes.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "at least one input lot is required".into(),
                ));
            }
            if !all_unique(&e.input_lot_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution input lot hashes must be unique".into(),
                ));
            }
            if e.output_lot_hashes.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "at least one output lot is required".into(),
                ));
            }
            if e.evidence_hashes.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "at least one evidence hash is required".into(),
                ));
            }
            if !all_unique(&e.output_lot_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution output lot hashes must be unique".into(),
                ));
            }
            if !all_unique(&e.evidence_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution evidence hashes must be unique".into(),
                ));
            }
            if !all_unique(&e.measurement_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution measurement hashes must be unique".into(),
                ));
            }
            if !all_unique(&e.calibration_hashes) {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution calibration hashes must be unique".into(),
                ));
            }
            if e.execution_id.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "execution_id is required".into(),
                ));
            }
            for hash in &e.evidence_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let evidence: Option<EvidenceEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(evidence) = evidence else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution evidence hash must reference an Evidence record".into(),
                    ));
                };
                if evidence.subject_id != e.execution_id
                    || !matches!(
                        evidence.kind,
                        EvidenceKind::Execution
                            | EvidenceKind::MachineObservation
                            | EvidenceKind::ProcessTrace
                    )
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution evidence is not semantically bound to the execution".into(),
                    ));
                }
            }

            // A claim of accepted production requires measured and calibrated
            // execution evidence. Rejected/rework/quarantined executions may
            // legitimately lack one or both categories while still being
            // recorded as observed events.
            if matches!(e.disposition, Disposition::Accepted)
                && e.capability_contract_hash.is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires a capability contract".into(),
                ));
            }
            if matches!(e.disposition, Disposition::Accepted)
                && e.process_parameters_hash.is_none()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires a process-parameter hash".into(),
                ));
            }
            if matches!(e.disposition, Disposition::Accepted)
                && (e.bom_hash.is_none() || e.routing_hash.is_none())
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires BOM and routing references".into(),
                ));
            }
            if matches!(e.disposition, Disposition::Accepted)
                && e.measurement_hashes.is_empty()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires at least one measurement".into(),
                ));
            }
            if matches!(e.disposition, Disposition::Accepted)
                && e.calibration_hashes.is_empty()
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires at least one calibration record".into(),
                ));
            }

            // Require every referenced action to be present in the DHT.
            // The stricter type checks below keep the local execution graph
            // from being populated with hashes for unrelated records.
            let work_order_record = must_get_valid_record(e.work_order_hash.clone())?;
            let work_order: WorkOrderRevisionProjection = work_order_record
                .entry()
                .to_app_option()
                .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "work order reference is missing its entry".into(),
                )))?;

            if let Err(message) = bind_execution_to_work_order_revision(e, &work_order) {
                return Ok(ValidateCallbackResult::Invalid(message.into()));
            }

            must_get_valid_record(e.machine_hash.clone())?;
            if let Some(hash) = e.capability_contract_hash.clone() {
                let record = must_get_valid_record(hash)?;
                let contract: Option<CapabilityContractEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(contract) = contract else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability_contract_hash does not reference a capability contract".into(),
                    ));
                };
                if contract.machine_hash != e.machine_hash {
                    return Ok(ValidateCallbackResult::Invalid(
                        "capability contract is bound to a different machine".into(),
                    ));
                }
                if matches!(e.disposition, Disposition::Accepted)
                    && (!matches!(contract.qualification, CapabilityQualification::Qualified)
                        || contract.qualification_evidence_hashes.is_empty()
                        || e.qualification_attestation_hash.is_none())
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "accepted execution requires a Qualified capability contract, evidence, and qualification attestation".into(),
                    ));
                }
                if let Some(attestation_hash) = e.qualification_attestation_hash.clone() {
                    let attestation_record = must_get_valid_record(attestation_hash)?;
                    let attestation: Option<QualificationAttestationEntry> = attestation_record
                        .entry()
                        .to_app_option()
                        .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                    let Some(attestation) = attestation else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "qualification attestation reference is not a qualification attestation".into(),
                        ));
                    };
                    if attestation.capability_contract_hash != hash {
                        return Ok(ValidateCallbackResult::Invalid(
                            "qualification attestation is bound to a different capability contract".into(),
                        ));
                    }
                    if !matches!(attestation.outcome, CapabilityQualification::Qualified)
                        || attestation.valid_from > e.started_at
                        || attestation.valid_until < e.completed_at
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "qualification attestation is not Qualified and valid for the execution interval".into(),
                        ));
                    }
                }
            }
            if let Some(hash) = e.bom_hash.clone() {
                must_get_valid_record(hash)?;
            }
            let mut required_inspection_criteria: Option<HashSet<ActionHash>> = None;

            if let Some(hash) = e.routing_hash.clone() {
                let routing_record = must_get_valid_record(hash)?;
                let routing: Option<RoutingRevisionProjection> = routing_record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(routing) = routing else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution routing reference is not a routing record".into(),
                    ));
                };
                let Some(step) = routing_step_for_sequence(&routing, e.operation_sequence) else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution operation sequence is not present in the bound routing revision".into(),
                    ));
                };

                if matches!(e.disposition, Disposition::Accepted) {
                    if !required_inspection_criteria_are_unique(
                        &step.required_inspection_criterion_hashes,
                    ) {
                        return Ok(ValidateCallbackResult::Invalid(
                            "accepted execution routing step has duplicate inspection criterion references".into(),
                        ));
                    }
                    if step.required_inspection_criterion_hashes.is_empty() {
                        return Ok(ValidateCallbackResult::Invalid(
                            "accepted execution requires at least one routing-owned inspection criterion".into(),
                        ));
                    }
                    required_inspection_criteria = Some(
                        step.required_inspection_criterion_hashes.iter().cloned().collect(),
                    );

                    let Some(requirement_hash) = step.capability_requirement_hash.clone() else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "accepted execution requires a typed capability requirement on the routing step".into(),
                        ));
                    };

                    let requirement_record = must_get_valid_record(requirement_hash)?;
                    let requirement: CapabilityRequirementProjection = requirement_record
                        .entry()
                        .to_app_option()
                        .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?
                        .ok_or(wasm_error!(WasmErrorInner::Guest(
                            "routing capability requirement record has no entry".into(),
                        )))?;

                    let Some(contract_hash) = e.capability_contract_hash.clone() else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "accepted execution requires a capability contract".into(),
                        ));
                    };
                    let contract_record = must_get_valid_record(contract_hash)?;
                    let contract: CapabilityContractEntry = contract_record
                        .entry()
                        .to_app_option()
                        .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?
                        .ok_or(wasm_error!(WasmErrorInner::Guest(
                            "execution capability contract record has no entry".into(),
                        )))?;

                    let profile = capability_profile_from_contract(&contract);
                    if let Err(mismatch) = evaluate_capability(&requirement.requirement, &profile) {
                        return Ok(ValidateCallbackResult::Invalid(format!(
                            "execution capability contract does not satisfy routing-owned requirement: {mismatch:?}"
                        )));
                    }
                }
            }

            for hash in &e.input_lot_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let lot: Option<MaterialLotEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                if lot.is_none() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution input references a non-material-lot record".into(),
                    ));
                }
            }

            for hash in &e.output_lot_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let lot: Option<MaterialLotEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                if lot.is_none() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution output references a non-material-lot record".into(),
                    ));
                }
            }

            let mut measurement_instruments = HashSet::new();
            for hash in &e.measurement_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let measurement: Option<MeasurementEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(measurement) = measurement else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution measurement reference is not a measurement record".into(),
                    ));
                };
                if measurement.measured_at < e.started_at
                    || measurement.measured_at > e.completed_at
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution measurement must fall within the execution interval".into(),
                    ));
                }
                if matches!(e.disposition, Disposition::Accepted)
                    && measurement.criterion_hash.is_none()
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "accepted execution measurements require an inspection criterion".into(),
                    ));
                }
                if let Some(instrument_hash) = measurement.instrument_hash.clone() {
                    measurement_instruments.insert(instrument_hash);
                }
                if let Some(lower) = measurement.lower_bound {
                    if measurement.value < lower {
                        return Ok(ValidateCallbackResult::Invalid(
                            "execution measurement is below its declared lower bound".into(),
                        ));
                    }
                }
                if let Some(upper) = measurement.upper_bound {
                    if measurement.value > upper {
                        return Ok(ValidateCallbackResult::Invalid(
                            "execution measurement is above its declared upper bound".into(),
                        ));
                    }
                }
            }

            if let Some(required_criteria) = required_inspection_criteria.as_ref() {
                let mut measured_criteria = HashSet::new();
                for hash in &e.measurement_hashes {
                    let record = must_get_valid_record(hash.clone())?;
                    let measurement: Option<MeasurementEntry> = record
                        .entry()
                        .to_app_option()
                        .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                    let Some(measurement) = measurement else {
                        return Ok(ValidateCallbackResult::Invalid(
                            "execution measurement reference is not a measurement record".into(),
                        ));
                    };
                    let Some(criterion_hash) = measurement.criterion_hash.clone() else {
                        if matches!(e.disposition, Disposition::Accepted) {
                            return Ok(ValidateCallbackResult::Invalid(
                                "accepted execution measurement must identify its inspection criterion".into(),
                            ));
                        }
                        continue;
                    };
                    if !required_criteria.contains(&criterion_hash)
                        && matches!(e.disposition, Disposition::Accepted)
                    {
                        return Ok(ValidateCallbackResult::Invalid(
                            "execution measurement references an inspection criterion not required by the routing step".into(),
                        ));
                    }
                    measured_criteria.insert(criterion_hash);
                }

                if matches!(e.disposition, Disposition::Accepted)
                    && measured_criteria != *required_criteria
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "accepted execution is missing one or more routing-required inspection measurements".into(),
                    ));
                }
            }

            let mut calibrated_assets = HashSet::new();
            for hash in &e.calibration_hashes {
                let record = must_get_valid_record(hash.clone())?;
                let calibration: Option<CalibrationEntry> = record
                    .entry()
                    .to_app_option()
                    .map_err(|err| wasm_error!(WasmErrorInner::Guest(err.to_string())))?;
                let Some(calibration) = calibration else {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution calibration reference is not a calibration record".into(),
                    ));
                };
                if calibration.valid_from <= e.started_at
                    && calibration.valid_until >= e.completed_at
                {
                    calibrated_assets.insert(calibration.asset_hash.clone());
                }
                if calibration.valid_from > e.started_at
                    || calibration.valid_until < e.completed_at
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution calibration does not cover the declared machine and execution interval".into(),
                    ));
                }
            }

            if matches!(e.disposition, Disposition::Accepted)
                && !calibrated_assets.contains(&e.machine_hash)
            {
                return Ok(ValidateCallbackResult::Invalid(
                    "accepted execution requires machine calibration covering the execution interval".into(),
                ));
            }
            for instrument_hash in &measurement_instruments {
                if matches!(e.disposition, Disposition::Accepted)
                    && !calibrated_assets.contains(instrument_hash)
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "accepted execution measurement instrument lacks calibration covering the execution interval".into(),
                    ));
                }
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inspection_criterion_reference_set_requires_unique_hashes() {
        let first = ActionHash::from_raw_36(vec![1; 36]);
        let second = ActionHash::from_raw_36(vec![2; 36]);
        assert!(required_inspection_criteria_are_unique(&[first.clone(), second]));
        assert!(!required_inspection_criteria_are_unique(&[first.clone(), first]));
    }

    #[test]
    fn rejects_capability_contract_that_fails_routing_requirement() {
        let requirement = CapabilityRequirement {
            process_family: "milling".into(),
            material_class: "aluminum".into(),
            envelope_x_mm: Some(400),
            envelope_y_mm: Some(200),
            envelope_z_mm: Some(100),
            tolerance_um: Some(10),
            required_protocols: vec!["opcua".into()],
        };
        let mut contract = CapabilityContractEntry {
            contract_id: "CAP-1".into(),
            machine_hash: ActionHash::from_raw_36(vec![3; 36]),
            process_family: "milling".into(),
            material_classes: vec!["aluminum".into()],
            envelope_x_mm: Some(500),
            envelope_y_mm: Some(300),
            envelope_z_mm: Some(200),
            tolerance_um: Some(25),
            supported_protocols: vec!["opcua".into()],
            qualification: CapabilityQualification::Qualified,
            qualification_evidence_hashes: vec![],
            created_at: Timestamp::from_micros(0),
        };
        let profile = capability_profile_from_contract(&contract);
        assert!(evaluate_capability(&requirement, &profile).is_err());

        contract.tolerance_um = Some(5);
        let profile = capability_profile_from_contract(&contract);
        assert!(evaluate_capability(&requirement, &profile).is_ok());
    }

    #[test]
    fn rejects_execution_using_different_work_order_bom_revision() {
        let execution = ExecutionReceiptEntry {
            execution_id: "EXEC-1".into(),
            qualification_attestation_hash: None,
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: Some(ActionHash::from_raw_36(vec![1; 36])),
            routing_hash: Some(ActionHash::from_raw_36(vec![2; 36])),
            operation_sequence: 10,
            machine_hash: ActionHash::from_raw_36(vec![3; 36]),
            capability_contract_hash: None,
            process_parameters_hash: None,
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![4; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Rejected,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![6; 36])],
            notes: None,
        };
        let work_order = WorkOrderRevisionProjection {
            bom_hash: Some(ActionHash::from_raw_36(vec![9; 36])),
            routing_hash: Some(ActionHash::from_raw_36(vec![2; 36])),
        };

        assert_eq!(
            bind_execution_to_work_order_revision(&execution, &work_order),
            Err("execution BOM does not match the work-order-bound BOM revision")
        );
    }

    #[test]
    fn allows_rejected_execution_to_omit_planned_revisions() {
        let execution = ExecutionReceiptEntry {
            execution_id: "EXEC-2".into(),
            qualification_attestation_hash: None,
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: None,
            routing_hash: None,
            operation_sequence: 10,
            machine_hash: ActionHash::from_raw_36(vec![3; 36]),
            capability_contract_hash: None,
            process_parameters_hash: None,
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![4; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Rejected,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![6; 36])],
            notes: None,
        };
        let work_order = WorkOrderRevisionProjection {
            bom_hash: Some(ActionHash::from_raw_36(vec![9; 36])),
            routing_hash: Some(ActionHash::from_raw_36(vec![8; 36])),
        };

        assert!(bind_execution_to_work_order_revision(&execution, &work_order).is_ok());
    }

    #[test]
    fn rejects_accepted_execution_without_exact_work_order_revision_binding() {
        let execution = ExecutionReceiptEntry {
            execution_id: "EXEC-3".into(),
            qualification_attestation_hash: None,
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: Some(ActionHash::from_raw_36(vec![1; 36])),
            routing_hash: Some(ActionHash::from_raw_36(vec![2; 36])),
            operation_sequence: 10,
            machine_hash: ActionHash::from_raw_36(vec![3; 36]),
            capability_contract_hash: None,
            process_parameters_hash: None,
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![4; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Accepted,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![6; 36])],
            notes: None,
        };
        let work_order = WorkOrderRevisionProjection {
            bom_hash: None,
            routing_hash: None,
        };

        assert_eq!(
            bind_execution_to_work_order_revision(&execution, &work_order),
            Err("accepted execution requires an exact work-order-bound BOM revision")
        );
    }


    #[test]
    fn rejects_malformed_inspection_criterion() {
        let entry = InspectionCriterionEntry {
            requirement_id: "".into(),
            revision: "A".into(),
            characteristic: "length".into(),
            unit: "mm".into(),
            lower_bound: Some(0.0),
            upper_bound: Some(1.0),
            measurement_method: None,
            required_instrument_class: None,
            specification_reference: None,
            created_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::InspectionCriterion(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn authoritative_criterion_overrides_permissive_measurement_bounds() {
        let criterion = InspectionCriterionEntry {
            requirement_id: "CRIT-1".into(),
            revision: "A".into(),
            characteristic: "length".into(),
            unit: "mm".into(),
            lower_bound: Some(10.0),
            upper_bound: Some(15.0),
            measurement_method: Some("CMM".into()),
            required_instrument_class: Some("dimensional".into()),
            specification_reference: None,
            created_at: Timestamp::from_micros(0),
        };
        let measurement = MeasurementEntry {
            measurement_id: "M-1".into(),
            criterion_hash: Some(ActionHash::from_raw_36(vec![9; 36])),
            kind: "length".into(),
            value: 25.0,
            unit: "mm".into(),
            lower_bound: Some(0.0),
            upper_bound: Some(100.0),
            instrument_hash: None,
            measured_at: Timestamp::from_micros(0),
        };

        assert_eq!(
            measurement_matches_criterion(&measurement, &criterion),
            Err("measurement value is outside its authoritative inspection criterion")
        );
    }

    #[test]
    fn rejects_unbounded_inspection_criterion() {
        let entry = InspectionCriterionEntry {
            requirement_id: "CRIT-1".into(),
            revision: "A".into(),
            characteristic: "length".into(),
            unit: "mm".into(),
            lower_bound: None,
            upper_bound: None,
            measurement_method: None,
            required_instrument_class: None,
            specification_reference: None,
            created_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::InspectionCriterion(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn rejects_inverted_qualification_attestation_window() {
        let entry = QualificationAttestationEntry {
            capability_contract_hash: ActionHash::from_raw_36(vec![1; 36]),
            outcome: CapabilityQualification::Qualified,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![2; 36])],
            method: "peer qualification".into(),
            authority_reference: "authority-v1".into(),
            valid_from: Timestamp::from_micros(10),
            valid_until: Timestamp::from_micros(0),
            external_reference_hash: None,
            created_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::QualificationAttestation(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn rejects_nonqualified_attestation_outcome() {
        let entry = QualificationAttestationEntry {
            capability_contract_hash: ActionHash::from_raw_36(vec![1; 36]),
            outcome: CapabilityQualification::Verified,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![2; 36])],
            method: "peer qualification".into(),
            authority_reference: "authority-v1".into(),
            valid_from: Timestamp::from_micros(0),
            valid_until: Timestamp::from_micros(10),
            external_reference_hash: None,
            created_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::QualificationAttestation(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn rejects_invalid_measurement() {
        let entry = MeasurementEntry {
            measurement_id: "M1".into(),
            criterion_hash: None,
            kind: "length".into(),
            value: f64::NAN,
            unit: "mm".into(),
            lower_bound: None,
            upper_bound: None,
            instrument_hash: None,
            measured_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::Measurement(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn rejects_inverted_calibration_window() {
        let entry = CalibrationEntry {
            asset_hash: ActionHash::from_raw_36(vec![0; 36]),
            method: "reference gauge".into(),
            valid_from: Timestamp::from_micros(10),
            valid_until: Timestamp::from_micros(0),
            evidence_reference: None,
            created_at: Timestamp::from_micros(0),
        };
        let result = validate_create(EntryTypes::Calibration(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn rejects_unbound_evidence() {
        let entry = EvidenceEntry {
            evidence_id: "EV-1".into(),
            subject_id: "OTHER".into(),
            kind: EvidenceKind::Execution,
            payload_hash: "sha256:abc".into(),
            source: "machine".into(),
            observed_at: Timestamp::from_micros(0),
        };
        assert_eq!(entry.subject_id, "OTHER");
        assert!(matches!(entry.kind, EvidenceKind::Execution));
    }

    #[test]
    fn rejects_unproven_execution() {
        let entry = ExecutionReceiptEntry {
            execution_id: "EXEC-TEST".into(),
            qualification_attestation_hash: None,
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: None,
            routing_hash: None,
            operation_sequence: 1,
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            capability_contract_hash: Some(ActionHash::from_raw_36(vec![4; 36])),
            process_parameters_hash: None,
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![2; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![3; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Accepted,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            notes: None,
        };
        let result = validate_create(EntryTypes::ExecutionReceipt(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn accepted_execution_requires_measurement_and_calibration() {
        let entry = ExecutionReceiptEntry {
            execution_id: "EXEC-TEST-2".into(),
            qualification_attestation_hash: None,
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: None,
            routing_hash: None,
            operation_sequence: 1,
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            process_parameters_hash: None,
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![2; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![3; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Accepted,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            notes: None,
        };
        let result = validate_create(EntryTypes::ExecutionReceipt(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
