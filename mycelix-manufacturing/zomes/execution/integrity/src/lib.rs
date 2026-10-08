// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Manufacturing execution integrity types.
//!
//! This zome records what physically happened, separately from planning.
//! Entries are immutable: corrections are new records, never in-place edits.

use hdi::prelude::*;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum Disposition {
    Accepted,
    Rejected,
    Rework,
    Quarantined,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum CapabilityQualification {
    Declared,
    Observed,
    Verified,
    Qualified,
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
pub struct MeasurementEntry {
    pub measurement_id: String,
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
    CapabilityContract(CapabilityContractEntry),
    ExecutionReceipt(ExecutionReceiptEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMaterialLots,
    AllMeasurements,
    AllCalibrations,
    AllCapabilityContracts,
    AllExecutions,
    WorkOrderToExecutions,
    MachineToExecutions,
    MachineToCapabilities,
    ExecutionToInputs,
    ExecutionToOutputs,
    ExecutionToMeasurements,
    ExecutionToCalibrations,
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
            for hash in &c.qualification_evidence_hashes {
                must_get_valid_record(hash.clone())?;
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
            for hash in &e.evidence_hashes {
                must_get_valid_record(hash.clone())?;
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
            must_get_valid_record(e.work_order_hash.clone())?;
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
                        || contract.qualification_evidence_hashes.is_empty())
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "accepted execution requires a Qualified capability contract with evidence".into(),
                    ));
                }
            }
            if let Some(hash) = e.bom_hash.clone() {
                must_get_valid_record(hash)?;
            }
            if let Some(hash) = e.routing_hash.clone() {
                must_get_valid_record(hash)?;
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
                if calibration.asset_hash != e.machine_hash
                    || calibration.valid_from > e.started_at
                    || calibration.valid_until < e.completed_at
                {
                    return Ok(ValidateCallbackResult::Invalid(
                        "execution calibration does not cover the declared machine and execution interval".into(),
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
    fn rejects_invalid_measurement() {
        let entry = MeasurementEntry {
            measurement_id: "M1".into(),
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
    fn rejects_unproven_execution() {
        let entry = ExecutionReceiptEntry {
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
