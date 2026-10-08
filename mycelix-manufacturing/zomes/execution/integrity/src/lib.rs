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
    pub process_parameters_hash: Option<String>,
    pub input_lot_hashes: Vec<ActionHash>,
    pub output_lot_hashes: Vec<ActionHash>,
    pub measurement_hashes: Vec<ActionHash>,
    pub calibration_hashes: Vec<ActionHash>,
    pub started_at: Timestamp,
    pub completed_at: Timestamp,
    pub disposition: Disposition,
    pub evidence_references: Vec<String>,
    pub notes: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    MaterialLot(MaterialLotEntry),
    Measurement(MeasurementEntry),
    Calibration(CalibrationEntry),
    ExecutionReceipt(ExecutionReceiptEntry),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMaterialLots,
    AllMeasurements,
    AllCalibrations,
    AllExecutions,
    WorkOrderToExecutions,
    MachineToExecutions,
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
            if e.evidence_references.is_empty() {
                return Ok(ValidateCallbackResult::Invalid(
                    "at least one evidence reference is required".into(),
                ));
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
            process_parameters_hash: None,
            input_lot_hashes: vec![],
            output_lot_hashes: vec![],
            measurement_hashes: vec![],
            calibration_hashes: vec![],
            started_at: Timestamp::from_micros(0),
            completed_at: Timestamp::from_micros(1),
            disposition: Disposition::Accepted,
            evidence_references: vec![],
            notes: None,
        };
        let result = validate_create(EntryTypes::ExecutionReceipt(entry)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
