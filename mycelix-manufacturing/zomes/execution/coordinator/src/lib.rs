// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Manufacturing execution coordinator.
//!
//! Execution is intentionally separate from planning: a work order describes
//! intent; an execution receipt describes an observed production event.

use hdk::prelude::*;
use manufacturing_execution_integrity::*;

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateMaterialLotInput {
    pub lot_id: String,
    pub material_id: String,
    pub quantity: u64,
    pub unit: String,
    pub source_reference: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateInspectionCriterionInput {
    pub requirement_id: String,
    pub revision: String,
    pub characteristic: String,
    pub unit: String,
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
    pub measurement_method: Option<String>,
    pub required_instrument_class: Option<String>,
    pub specification_reference: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateMeasurementInput {
    pub measurement_id: String,
    pub criterion_hash: Option<ActionHash>,
    pub kind: String,
    pub value: f64,
    pub unit: String,
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
    pub instrument_hash: Option<ActionHash>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateQualificationAttestationInput {
    pub capability_contract_hash: ActionHash,
    pub outcome: CapabilityQualification,
    pub evidence_hashes: Vec<ActionHash>,
    pub method: String,
    pub authority_reference: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub external_reference_hash: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateCalibrationInput {
    pub asset_hash: ActionHash,
    pub method: String,
    pub valid_from: Timestamp,
    pub valid_until: Timestamp,
    pub evidence_reference: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateEvidenceInput {
    pub evidence_id: String,
    pub subject_id: String,
    pub kind: EvidenceKind,
    pub payload_hash: String,
    pub source: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateCapabilityContractInput {
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
}

#[derive(Serialize, Deserialize, Debug)]
pub struct CreateExecutionInput {
    pub execution_id: String,
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

fn link_from(anchor: &str, link_type: LinkTypes, target: ActionHash) -> ExternResult<()> {
    let path = Path::from(anchor).typed(link_type)?;
    path.ensure()?;
    create_link(path.path_entry_hash()?, target, link_type, ())?;
    Ok(())
}

#[hdk_extern]
pub fn record_material_lot(input: CreateMaterialLotInput) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::MaterialLot(MaterialLotEntry {
        lot_id: input.lot_id,
        material_id: input.material_id,
        quantity: input.quantity,
        unit: input.unit,
        source_reference: input.source_reference,
        created_at: sys_time()?,
    }))?;

    link_from("all_material_lots", LinkTypes::AllMaterialLots, hash.clone())?;
    Ok(hash)
}

#[hdk_extern]
pub fn record_inspection_criterion(
    input: CreateInspectionCriterionInput,
) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::InspectionCriterion(
        InspectionCriterionEntry {
            requirement_id: input.requirement_id,
            revision: input.revision,
            characteristic: input.characteristic,
            unit: input.unit,
            lower_bound: input.lower_bound,
            upper_bound: input.upper_bound,
            measurement_method: input.measurement_method,
            required_instrument_class: input.required_instrument_class,
            specification_reference: input.specification_reference,
            created_at: sys_time()?,
        },
    ))?;

    link_from(
        "all_inspection_criteria",
        LinkTypes::AllInspectionCriteria,
        hash.clone(),
    )?;
    Ok(hash)
}

#[hdk_extern]
pub fn record_measurement(input: CreateMeasurementInput) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::Measurement(MeasurementEntry {
        measurement_id: input.measurement_id,
        criterion_hash: input.criterion_hash,
        kind: input.kind,
        value: input.value,
        unit: input.unit,
        lower_bound: input.lower_bound,
        upper_bound: input.upper_bound,
        instrument_hash: input.instrument_hash,
        measured_at: sys_time()?,
    }))?;

    link_from("all_measurements", LinkTypes::AllMeasurements, hash.clone())?;
    if let Some(criterion_hash) = input.criterion_hash {
        create_link(
            hash.clone(),
            criterion_hash,
            LinkTypes::MeasurementToInspectionCriterion,
            (),
        )?;
    }
    Ok(hash)
}

#[hdk_extern]
pub fn record_qualification_attestation(
    input: CreateQualificationAttestationInput,
) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::QualificationAttestation(
        QualificationAttestationEntry {
            capability_contract_hash: input.capability_contract_hash,
            outcome: input.outcome,
            evidence_hashes: input.evidence_hashes,
            method: input.method,
            authority_reference: input.authority_reference,
            valid_from: input.valid_from,
            valid_until: input.valid_until,
            external_reference_hash: input.external_reference_hash,
            created_at: sys_time()?,
        },
    ))?;

    link_from(
        "all_qualification_attestations",
        LinkTypes::AllQualificationAttestations,
        hash.clone(),
    )?;
    create_link(
        input.capability_contract_hash,
        hash.clone(),
        LinkTypes::CapabilityContractToQualificationAttestations,
        (),
    )?;
    Ok(hash)
}

#[hdk_extern]
pub fn record_calibration(input: CreateCalibrationInput) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::Calibration(CalibrationEntry {
        asset_hash: input.asset_hash,
        method: input.method,
        valid_from: input.valid_from,
        valid_until: input.valid_until,
        evidence_reference: input.evidence_reference,
        created_at: sys_time()?,
    }))?;

    link_from("all_calibrations", LinkTypes::AllCalibrations, hash.clone())?;
    Ok(hash)
}

#[hdk_extern]
pub fn record_evidence(input: CreateEvidenceInput) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::Evidence(EvidenceEntry {
        evidence_id: input.evidence_id,
        subject_id: input.subject_id,
        kind: input.kind,
        payload_hash: input.payload_hash,
        source: input.source,
        observed_at: sys_time()?,
    }))?;

    link_from("all_evidence", LinkTypes::AllEvidence, hash.clone())?;
    Ok(hash)
}

#[hdk_extern]
pub fn record_capability_contract(
    input: CreateCapabilityContractInput,
) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::CapabilityContract(CapabilityContractEntry {
        contract_id: input.contract_id,
        machine_hash: input.machine_hash.clone(),
        process_family: input.process_family,
        material_classes: input.material_classes,
        envelope_x_mm: input.envelope_x_mm,
        envelope_y_mm: input.envelope_y_mm,
        envelope_z_mm: input.envelope_z_mm,
        tolerance_um: input.tolerance_um,
        supported_protocols: input.supported_protocols,
        qualification: input.qualification,
        qualification_evidence_hashes: input.qualification_evidence_hashes,
        created_at: sys_time()?,
    }))?;

    link_from(
        "all_capability_contracts",
        LinkTypes::AllCapabilityContracts,
        hash.clone(),
    )?;
    create_link(
        input.machine_hash,
        hash.clone(),
        LinkTypes::MachineToCapabilities,
        (),
    )?;

    Ok(hash)
}

#[hdk_extern]
pub fn record_execution(input: CreateExecutionInput) -> ExternResult<ActionHash> {
    let hash = create_entry(EntryTypes::ExecutionReceipt(ExecutionReceiptEntry {
        execution_id: input.execution_id.clone(),
        qualification_attestation_hash: input.qualification_attestation_hash.clone(),
        work_order_hash: input.work_order_hash.clone(),
        bom_hash: input.bom_hash,
        routing_hash: input.routing_hash,
        operation_sequence: input.operation_sequence,
        machine_hash: input.machine_hash.clone(),
        capability_contract_hash: input.capability_contract_hash,
        process_parameters_hash: input.process_parameters_hash,
        input_lot_hashes: input.input_lot_hashes.clone(),
        output_lot_hashes: input.output_lot_hashes.clone(),
        measurement_hashes: input.measurement_hashes.clone(),
        calibration_hashes: input.calibration_hashes.clone(),
        started_at: input.started_at,
        completed_at: input.completed_at,
        disposition: input.disposition,
        evidence_hashes: input.evidence_hashes,
        notes: input.notes,
    }))?;

    link_from("all_executions", LinkTypes::AllExecutions, hash.clone())?;

    create_link(
        input.work_order_hash,
        hash.clone(),
        LinkTypes::WorkOrderToExecutions,
        (),
    )?;
    create_link(
        input.machine_hash,
        hash.clone(),
        LinkTypes::MachineToExecutions,
        (),
    )?;

    for lot in input.input_lot_hashes {
        create_link(hash.clone(), lot, LinkTypes::ExecutionToInputs, ())?;
    }
    for lot in input.output_lot_hashes {
        create_link(hash.clone(), lot, LinkTypes::ExecutionToOutputs, ())?;
    }
    for measurement in input.measurement_hashes {
        create_link(
            hash.clone(),
            measurement,
            LinkTypes::ExecutionToMeasurements,
            (),
        )?;
    }
    if let Some(attestation_hash) = input.qualification_attestation_hash {
        create_link(
            hash.clone(),
            attestation_hash,
            LinkTypes::ExecutionToQualificationAttestation,
            (),
        )?;
    }
    for calibration in input.calibration_hashes {
        create_link(
            hash.clone(),
            calibration,
            LinkTypes::ExecutionToCalibrations,
            (),
        )?;
    }

    Ok(hash)
}

/// Get a qualification attestation by action hash.
#[hdk_extern]
pub fn get_qualification_attestation(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// List qualification attestations bound to a specific capability contract.
#[hdk_extern]
pub fn list_qualification_attestations_for_contract(
    capability_contract_hash: ActionHash,
) -> ExternResult<Vec<Link>> {
    get_links(
        GetLinksInputBuilder::try_new(
            capability_contract_hash,
            LinkTypes::CapabilityContractToQualificationAttestations,
        )?
        .build(),
    )
}

#[hdk_extern]
pub fn get_execution(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// Get an inspection criterion by action hash.
#[hdk_extern]
pub fn get_inspection_criterion(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// Get a capability contract by action hash.
#[hdk_extern]
pub fn get_capability_contract(hash: ActionHash) -> ExternResult<Option<Record>> {
    get(hash, GetOptions::default())
}

/// List all capability contracts from the immutable execution registry.
#[hdk_extern]
pub fn list_capability_contracts(_: ()) -> ExternResult<Vec<Link>> {
    let path = Path::from("all_capability_contracts")
        .typed(LinkTypes::AllCapabilityContracts)?;
    get_links(
        GetLinksInputBuilder::try_new(
            path.path_entry_hash()?,
            LinkTypes::AllCapabilityContracts,
        )?
        .build(),
    )
}

#[hdk_extern]
pub fn list_executions(_: ()) -> ExternResult<Vec<Link>> {
    let path = Path::from("all_executions").typed(LinkTypes::AllExecutions)?;
    get_links(GetLinksInputBuilder::try_new(path.path_entry_hash()?, LinkTypes::AllExecutions)?.build())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[test]
    fn test_measurement_input_serde_with_criterion() {
        let input = CreateMeasurementInput {
            measurement_id: "M-1".into(),
            criterion_hash: Some(ActionHash::from_raw_36(vec![7; 36])),
            kind: "length".into(),
            value: 12.5,
            unit: "mm".into(),
            lower_bound: Some(10.0),
            upper_bound: Some(15.0),
            instrument_hash: None,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateMeasurementInput = serde_json::from_str(&json).unwrap();
        assert!(back.criterion_hash.is_some());
        assert_eq!(back.value, 12.5);
    }

    #[test]
    fn test_inspection_criterion_input_serde() {
        let input = CreateInspectionCriterionInput {
            requirement_id: "INSPECT-001".into(),
            revision: "A".into(),
            characteristic: "length".into(),
            unit: "mm".into(),
            lower_bound: Some(10.0),
            upper_bound: Some(15.0),
            measurement_method: Some("CMM".into()),
            required_instrument_class: Some("dimensional".into()),
            specification_reference: Some("spec-sha256:abc".into()),
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateInspectionCriterionInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.requirement_id, "INSPECT-001");
        assert_eq!(back.revision, "A");
    }

    #[test]
    fn test_execution_input_serde() {
        let input = CreateExecutionInput {
            execution_id: "EXEC-TEST".into(),
            work_order_hash: ActionHash::from_raw_36(vec![0; 36]),
            bom_hash: None,
            routing_hash: None,
            operation_sequence: 1,
            machine_hash: ActionHash::from_raw_36(vec![1; 36]),
            capability_contract_hash: Some(ActionHash::from_raw_36(vec![4; 36])),
            process_parameters_hash: Some("recipe-sha256".into()),
            input_lot_hashes: vec![ActionHash::from_raw_36(vec![2; 36])],
            output_lot_hashes: vec![ActionHash::from_raw_36(vec![3; 36])],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(10),
            disposition: Disposition::Accepted,
            evidence_hashes: vec![ActionHash::from_raw_36(vec![5; 36])],
            notes: None,
        };
        let json = serde_json::to_string(&input).unwrap();
        let back: CreateExecutionInput = serde_json::from_str(&json).unwrap();
        assert_eq!(back.operation_sequence, 1);
        assert_eq!(back.disposition, Disposition::Accepted);
    }
}
