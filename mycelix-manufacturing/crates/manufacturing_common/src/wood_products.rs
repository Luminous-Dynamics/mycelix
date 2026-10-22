// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Evidence-aware timber, lumber, and process-material contracts.
//!
//! This module is a shared data contract, not a certification scheme. Evidence
//! states describe what a record claims about its inputs; they do not prove a
//! physical measurement, species claim, grade or source is accurate. A
//! qualification claim still requires checks by the applicable authority and
//! rules outside this module.

use hdi::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Evidence state attached to a material or process claim.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WoodEvidenceState {
    Unknown,
    Estimated,
    Observed,
    Inspected,
    Qualified,
}

/// The semantic quantity represented by a measurement.
///
/// A measurement kind fixes the unit expected by this contract. This prevents
/// dimension and volume values from being silently compared as if equivalent.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WoodMeasurementKind {
    LogLength,
    LogDiameter,
    LogVolume,
    BoardLength,
    BoardWidth,
    BoardThickness,
    BoardVolume,
    DefectExtent,
    DryMass,
    MoistureContent,
    PieceCount,
    EnergyUse,
    Other,
}

/// Base units accepted by the wood-measurement contract.
///
/// Values and absolute uncertainty use the same integer unit: µm for dimensions,
/// mm³ for volume, g for dry mass, basis points of percent for moisture, pieces
/// for counts, and Wh for energy.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WoodMeasurementUnit {
    Micrometer,
    CubicMillimeter,
    GramDryMass,
    MoistureBasisPoint,
    Piece,
    WattHour,
    Other,
}

/// Calibration metadata is evidence about the instrument, not a grade certificate.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WoodCalibrationStatus {
    Current,
    Expired,
    NotApplicable,
    Unknown,
}

/// A typed, traceable observation about a wood batch.
///
/// value and absolute_uncertainty are integer quantities in the same base
/// unit selected by unit. This record preserves measurement provenance but
/// does not prove that the observation is true or independently qualify lumber.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct WoodMeasurementEvidence {
    pub measurement_id: String,
    pub batch_id: String,
    pub kind: WoodMeasurementKind,
    pub value: u64,
    pub unit: WoodMeasurementUnit,
    pub absolute_uncertainty: u64,
    pub method: String,
    pub evidence_state: WoodEvidenceState,
    pub instrument_id: Option<String>,
    pub calibration_status: WoodCalibrationStatus,
    pub source_refs: Vec<String>,
    pub uncertainty_note: Option<String>,
    pub recorded_at: Timestamp,
}

impl WoodMeasurementEvidence {
    fn expected_unit(kind: WoodMeasurementKind) -> WoodMeasurementUnit {
        match kind {
            WoodMeasurementKind::LogLength
            | WoodMeasurementKind::LogDiameter
            | WoodMeasurementKind::BoardLength
            | WoodMeasurementKind::BoardWidth
            | WoodMeasurementKind::BoardThickness
            | WoodMeasurementKind::DefectExtent => WoodMeasurementUnit::Micrometer,
            WoodMeasurementKind::LogVolume | WoodMeasurementKind::BoardVolume => {
                WoodMeasurementUnit::CubicMillimeter
            }
            WoodMeasurementKind::DryMass => WoodMeasurementUnit::GramDryMass,
            WoodMeasurementKind::MoistureContent => WoodMeasurementUnit::MoistureBasisPoint,
            WoodMeasurementKind::PieceCount => WoodMeasurementUnit::Piece,
            WoodMeasurementKind::EnergyUse => WoodMeasurementUnit::WattHour,
            WoodMeasurementKind::Other => WoodMeasurementUnit::Other,
        }
    }

    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        for (field, value) in [
            ("measurement_id", self.measurement_id.as_str()),
            ("batch_id", self.batch_id.as_str()),
            ("method", self.method.as_str()),
        ] {
            if value.trim().is_empty() {
                errors.push(format!("{field} is required"));
            }
        }
        if self.measurement_id.len() > 200 {
            errors.push("measurement_id must be <= 200 characters".to_owned());
        }
        if self.batch_id.len() > 200 {
            errors.push("batch_id must be <= 200 characters".to_owned());
        }
        if self.unit != Self::expected_unit(self.kind) {
            errors.push(format!(
                "unit {:?} does not match measurement kind {:?}; expected {:?}",
                self.unit,
                self.kind,
                Self::expected_unit(self.kind)
            ));
        }
        if self.instrument_id.as_ref().is_some_and(|value| value.trim().is_empty()) {
            errors.push("instrument_id must be omitted rather than blank".to_owned());
        }
        if self.calibration_status == WoodCalibrationStatus::Current
            && self.instrument_id.is_none()
        {
            errors.push("current calibration requires an instrument_id".to_owned());
        }
        if self.instrument_id.is_some()
            && matches!(
                self.calibration_status,
                WoodCalibrationStatus::Expired | WoodCalibrationStatus::Unknown
            )
            && self.uncertainty_note.as_deref().is_none_or(|note| note.trim().is_empty())
        {
            errors.push(
                "expired or unknown instrument calibration requires an uncertainty note"
                    .to_owned(),
            );
        }

        let mut sources = BTreeSet::new();
        for source in &self.source_refs {
            if source.trim().is_empty() {
                errors.push("source references must not be blank".to_owned());
            }
            if !sources.insert(source) {
                errors.push(format!("duplicate source reference: {source}"));
            }
        }
        match self.evidence_state {
            WoodEvidenceState::Unknown | WoodEvidenceState::Estimated => {
                if self.uncertainty_note.as_deref().is_none_or(|note| note.trim().is_empty()) {
                    errors.push(
                        "unknown or estimated measurements require an uncertainty note"
                            .to_owned(),
                    );
                }
            }
            WoodEvidenceState::Observed
            | WoodEvidenceState::Inspected
            | WoodEvidenceState::Qualified => {
                if self.source_refs.is_empty() {
                    errors.push(
                        "observed, inspected, and qualified measurements require a source reference"
                            .to_owned(),
                    );
                }
            }
        }
        if self.uncertainty_note.as_deref().is_some_and(|note| note.trim().is_empty()) {
            errors.push("uncertainty_note must be omitted rather than blank".to_owned());
        }

        if errors.is_empty() { Ok(()) } else { Err(errors) }
    }
}

/// Broad material classification without a jurisdiction-specific grading rule.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum WoodMaterialKind {
    StandingTimber,
    Roundwood,
    GreenLumber,
    DriedLumber,
    GradedLumber,
    ReclaimedWood,
    Chips,
    Sawdust,
    Bark,
    Offcuts,
    RecoveredFiber,
    DisposalWaste,
    DeclaredProcessLoss,
    Other,
}

/// A traceable material lot or product batch.
///
/// Quantity retains an explicit unit; dry mass is in grams when present.
/// Moisture content uses basis points of percentage (1,200 = 12.00%). Wood
/// moisture content may exceed 100% on a dry-mass basis, so no 100% ceiling is
/// imposed here. A grade claim is free text and is not validated as a standard.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct WoodBatchClaim {
    pub batch_id: String,
    /// Outputs from a process should receive new IDs and cite their parent lots.
    pub parent_batch_ids: Vec<String>,
    pub material_kind: WoodMaterialKind,
    pub quantity: u64,
    pub quantity_unit: String,
    pub dry_mass_grams: Option<u64>,
    pub moisture_content_basis_points: Option<u32>,
    pub species_claim: Option<String>,
    pub grade_claim: Option<String>,
    pub evidence_state: WoodEvidenceState,
    /// Source records, inspection artefacts or signed-attestation references.
    pub source_refs: Vec<String>,
    pub uncertainty_note: Option<String>,
    pub recorded_at: Timestamp,
}

impl WoodBatchClaim {
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        if self.batch_id.trim().is_empty() {
            errors.push("batch_id is required".to_owned());
        }
        if self.batch_id.len() > 200 {
            errors.push("batch_id must be <= 200 characters".to_owned());
        }
        if self.quantity == 0 {
            errors.push("quantity must be > 0".to_owned());
        }
        if self.quantity_unit.trim().is_empty() {
            errors.push("quantity_unit is required".to_owned());
        }
        if self.dry_mass_grams == Some(0) {
            errors.push("dry_mass_grams, when present, must be > 0".to_owned());
        }
        if self.species_claim.as_ref().is_some_and(|value| value.trim().is_empty()) {
            errors.push("species_claim must be omitted rather than blank".to_owned());
        }
        if self.grade_claim.as_ref().is_some_and(|value| value.trim().is_empty()) {
            errors.push("grade_claim must be omitted rather than blank".to_owned());
        }

        let mut parent_ids = BTreeSet::new();
        for parent_id in &self.parent_batch_ids {
            if parent_id.trim().is_empty() {
                errors.push("parent batch IDs must not be empty".to_owned());
            }
            if parent_id == &self.batch_id {
                errors.push("a batch cannot list itself as a parent".to_owned());
            }
            if !parent_ids.insert(parent_id) {
                errors.push(format!("duplicate parent batch ID: {parent_id}"));
            }
        }

        let mut sources = BTreeSet::new();
        for source in &self.source_refs {
            if source.trim().is_empty() {
                errors.push("source references must not be blank".to_owned());
            }
            if !sources.insert(source) {
                errors.push(format!("duplicate source reference: {source}"));
            }
        }

        match self.evidence_state {
            WoodEvidenceState::Unknown | WoodEvidenceState::Estimated => {
                if self.uncertainty_note.as_deref().is_none_or(|note| note.trim().is_empty()) {
                    errors.push(
                        "unknown or estimated material claims require an uncertainty note".to_owned(),
                    );
                }
            }
            WoodEvidenceState::Observed
            | WoodEvidenceState::Inspected
            | WoodEvidenceState::Qualified => {
                if self.source_refs.is_empty() {
                    errors.push(
                        "observed, inspected, and qualified claims require a source reference"
                            .to_owned(),
                    );
                }
            }
        }

        if self.uncertainty_note.as_deref().is_some_and(|note| note.trim().is_empty()) {
            errors.push("uncertainty_note must be omitted rather than blank".to_owned());
        }
        if errors.is_empty() { Ok(()) } else { Err(errors) }
    }
}

/// An input lot's declared oven-dry / absolute dry mass.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WoodMassInput {
    pub batch_id: String,
    pub dry_mass_grams: u64,
}

/// One output stream from a process, including co-products, waste, and loss.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WoodMassOutput {
    pub stream_id: String,
    pub material_kind: WoodMaterialKind,
    pub dry_mass_grams: u64,
}

/// Result of checking dry-solid accounting for one process run.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct DryMassBalanceReport {
    pub process_run_id: String,
    pub input_dry_mass_grams: u64,
    pub accounted_output_dry_mass_grams: u64,
    pub balanced: bool,
}

/// Immutable material-flow record for sawing, drying, finishing, recycling or
/// other wood processing. It balances declared dry solids but does not prove
/// that every physical stream was measured or that none was omitted.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct WoodProcessLedger {
    pub process_run_id: String,
    pub inputs: Vec<WoodMassInput>,
    pub outputs: Vec<WoodMassOutput>,
    pub evidence_state: WoodEvidenceState,
    pub source_refs: Vec<String>,
    pub uncertainty_note: Option<String>,
    pub recorded_at: Timestamp,
}

impl WoodProcessLedger {
    /// Validate identities, checked integer sums and exact dry-mass closure.
    ///
    /// Water removed during drying is outside dry mass. Any additional solid
    /// inputs must be listed as input batches; residuals and process losses must
    /// be output streams. No implicit rounding tolerance is used.
    pub fn validate_mass_balance(&self) -> Result<DryMassBalanceReport, String> {
        if self.process_run_id.trim().is_empty() {
            return Err("process_run_id is required".to_owned());
        }
        if self.inputs.is_empty() {
            return Err("at least one input batch is required".to_owned());
        }
        if self.outputs.is_empty() {
            return Err("at least one output stream is required".to_owned());
        }
        if self.source_refs.iter().any(|source| source.trim().is_empty()) {
            return Err("source references must not contain blanks".to_owned());
        }
        if matches!(self.evidence_state, WoodEvidenceState::Unknown | WoodEvidenceState::Estimated)
            && self.uncertainty_note.as_deref().is_none_or(|note| note.trim().is_empty())
        {
            return Err("unknown or estimated process ledgers require an uncertainty note".to_owned());
        }
        if matches!(
            self.evidence_state,
            WoodEvidenceState::Observed | WoodEvidenceState::Inspected | WoodEvidenceState::Qualified
        ) && self.source_refs.is_empty()
        {
            return Err("observed, inspected, and qualified process ledgers require source references".to_owned());
        }

        let mut input_ids = BTreeSet::new();
        let mut input_mass = 0_u64;
        for input in &self.inputs {
            if input.batch_id.trim().is_empty() {
                return Err("input batch IDs must not be empty".to_owned());
            }
            if input.dry_mass_grams == 0 {
                return Err(format!("input {} dry mass must be > 0", input.batch_id));
            }
            if !input_ids.insert(&input.batch_id) {
                return Err(format!("duplicate input batch ID: {}", input.batch_id));
            }
            input_mass = input_mass.checked_add(input.dry_mass_grams)
                .ok_or_else(|| "input dry-mass total overflow".to_owned())?;
        }

        let mut output_ids = BTreeSet::new();
        let mut output_mass = 0_u64;
        for output in &self.outputs {
            if output.stream_id.trim().is_empty() {
                return Err("output stream IDs must not be empty".to_owned());
            }
            if output.dry_mass_grams == 0 {
                return Err(format!("output {} dry mass must be > 0", output.stream_id));
            }
            if !output_ids.insert(&output.stream_id) {
                return Err(format!("duplicate output stream ID: {}", output.stream_id));
            }
            output_mass = output_mass.checked_add(output.dry_mass_grams)
                .ok_or_else(|| "output dry-mass total overflow".to_owned())?;
        }

        if input_mass != output_mass {
            return Err(format!(
                "dry-mass imbalance in {}: inputs={} g, accounted outputs={} g",
                self.process_run_id, input_mass, output_mass
            ));
        }

        Ok(DryMassBalanceReport {
            process_run_id: self.process_run_id.clone(),
            input_dry_mass_grams: input_mass,
            accounted_output_dry_mass_grams: output_mass,
            balanced: true,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_timestamp() -> Timestamp {
        Timestamp::from_micros(1_700_000_000_000_000)
    }

    fn measured_dimension() -> WoodMeasurementEvidence {
        WoodMeasurementEvidence {
            measurement_id: "measurement-board-001-thickness".to_owned(),
            batch_id: "batch-lumber-001".to_owned(),
            kind: WoodMeasurementKind::BoardThickness,
            value: 25_000,
            unit: WoodMeasurementUnit::Micrometer,
            absolute_uncertainty: 200,
            method: "digital_caliper".to_owned(),
            evidence_state: WoodEvidenceState::Observed,
            instrument_id: Some("caliper-007".to_owned()),
            calibration_status: WoodCalibrationStatus::Current,
            source_refs: vec![
                "calibration-certificate-007".to_owned(),
                "measurement-record-001".to_owned(),
            ],
            uncertainty_note: None,
            recorded_at: test_timestamp(),
        }
    }

    fn observed_batch() -> WoodBatchClaim {
        WoodBatchClaim {
            batch_id: "batch-lumber-001".to_owned(),
            parent_batch_ids: vec!["batch-roundwood-001".to_owned()],
            material_kind: WoodMaterialKind::DriedLumber,
            quantity: 120,
            quantity_unit: "board_foot".to_owned(),
            dry_mass_grams: Some(620_000),
            moisture_content_basis_points: Some(1_200),
            species_claim: Some("species-claim-001".to_owned()),
            grade_claim: Some("grade-claim-not-a-certification".to_owned()),
            evidence_state: WoodEvidenceState::Observed,
            source_refs: vec!["inspection-record-001".to_owned()],
            uncertainty_note: None,
            recorded_at: test_timestamp(),
        }
    }

    fn balanced_ledger() -> WoodProcessLedger {
        WoodProcessLedger {
            process_run_id: "saw-run-001".to_owned(),
            inputs: vec![WoodMassInput {
                batch_id: "roundwood-001".to_owned(),
                dry_mass_grams: 1_000_000,
            }],
            outputs: vec![
                WoodMassOutput { stream_id: "lumber-001".to_owned(), material_kind: WoodMaterialKind::GradedLumber, dry_mass_grams: 620_000 },
                WoodMassOutput { stream_id: "chips-001".to_owned(), material_kind: WoodMaterialKind::Chips, dry_mass_grams: 220_000 },
                WoodMassOutput { stream_id: "sawdust-001".to_owned(), material_kind: WoodMaterialKind::Sawdust, dry_mass_grams: 80_000 },
                WoodMassOutput { stream_id: "bark-001".to_owned(), material_kind: WoodMaterialKind::Bark, dry_mass_grams: 30_000 },
                WoodMassOutput { stream_id: "loss-001".to_owned(), material_kind: WoodMaterialKind::DeclaredProcessLoss, dry_mass_grams: 50_000 },
            ],
            evidence_state: WoodEvidenceState::Observed,
            source_refs: vec!["scale-record-001".to_owned()],
            uncertainty_note: None,
            recorded_at: test_timestamp(),
        }
    }

    #[test]
    fn valid_batch_preserves_evidence_and_parent_lineage() {
        let batch = observed_batch();
        assert!(batch.validate().is_ok());
        let encoded = serde_json::to_string(&batch).expect("batch serializes");
        let decoded: WoodBatchClaim = serde_json::from_str(&encoded).expect("batch deserializes");
        assert_eq!(decoded, batch);
        assert_eq!(decoded.parent_batch_ids, vec!["batch-roundwood-001"]);
    }

    #[test]
    fn unknown_or_estimated_claims_require_uncertainty_notes() {
        let mut batch = observed_batch();
        batch.evidence_state = WoodEvidenceState::Estimated;
        batch.source_refs.clear();
        assert!(batch.validate().expect_err("uncertainty must be explicit")
            .iter().any(|error| error.contains("uncertainty note")));
    }

    #[test]
    fn self_parent_and_duplicate_parent_ids_are_rejected() {
        let mut batch = observed_batch();
        batch.parent_batch_ids = vec![
            "batch-lumber-001".to_owned(),
            "batch-roundwood-001".to_owned(),
            "batch-roundwood-001".to_owned(),
        ];
        let errors = batch.validate().expect_err("invalid lineage must fail");
        assert!(errors.iter().any(|error| error.contains("itself as a parent")));
        assert!(errors.iter().any(|error| error.contains("duplicate parent")));
    }

    #[test]
    fn measurement_preserves_units_uncertainty_and_provenance() {
        let evidence = measured_dimension();
        assert!(evidence.validate().is_ok());
        let encoded = serde_json::to_string(&evidence).expect("measurement serializes");
        let decoded: WoodMeasurementEvidence =
            serde_json::from_str(&encoded).expect("measurement deserializes");
        assert_eq!(decoded, evidence);
        assert_eq!(decoded.absolute_uncertainty, 200);
        assert_eq!(decoded.source_refs.len(), 2);
    }

    #[test]
    fn measurement_kind_unit_mismatch_is_rejected() {
        let mut evidence = measured_dimension();
        evidence.unit = WoodMeasurementUnit::CubicMillimeter;
        let errors = evidence.validate().expect_err("length cannot be expressed as volume");
        assert!(errors.iter().any(|error| error.contains("does not match measurement kind")));
    }

    #[test]
    fn current_calibration_requires_instrument_identity() {
        let mut evidence = measured_dimension();
        evidence.instrument_id = None;
        assert!(
            evidence.validate().expect_err("current calibration must identify an instrument")
                .iter().any(|error| error.contains("current calibration requires"))
        );
    }

    #[test]
    fn uncalibrated_instrument_requires_uncertainty_note() {
        let mut evidence = measured_dimension();
        evidence.calibration_status = WoodCalibrationStatus::Expired;
        evidence.uncertainty_note = None;
        assert!(
            evidence.validate().expect_err("expired calibration must be disclosed")
                .iter().any(|error| error.contains("uncertainty note"))
        );
    }

    #[test]
    fn measured_evidence_requires_source_reference() {
        let mut evidence = measured_dimension();
        evidence.source_refs.clear();
        assert!(
            evidence.validate().expect_err("measurements must be traceable")
                .iter().any(|error| error.contains("source reference"))
        );
    }

    #[test]
    fn estimated_measurement_requires_uncertainty_note() {
        let mut evidence = measured_dimension();
        evidence.evidence_state = WoodEvidenceState::Estimated;
        evidence.uncertainty_note = None;
        assert!(
            evidence.validate().expect_err("estimate must disclose uncertainty")
                .iter().any(|error| error.contains("uncertainty note"))
        );
    }

    #[test]
    fn ledger_accounts_for_products_coproducts_and_loss() {
        let report = balanced_ledger().validate_mass_balance().expect("dry-mass ledger closes");
        assert_eq!(report.input_dry_mass_grams, 1_000_000);
        assert_eq!(report.accounted_output_dry_mass_grams, 1_000_000);
        assert!(report.balanced);
    }

    #[test]
    fn unexplained_mass_creation_is_rejected() {
        let mut ledger = balanced_ledger();
        ledger.outputs[0].dry_mass_grams = 620_001;
        let error = ledger.validate_mass_balance().expect_err("one gram of mass creation must fail");
        assert!(error.contains("dry-mass imbalance"));
    }

    #[test]
    fn missing_residual_mass_is_not_silently_rounded_away() {
        let mut ledger = balanced_ledger();
        ledger.outputs.retain(|output| output.stream_id != "bark-001");
        let error = ledger.validate_mass_balance().expect_err("missing stream must fail closure");
        assert!(error.contains("dry-mass imbalance"));
    }

    #[test]
    fn integer_overflow_is_rejected() {
        let mut ledger = balanced_ledger();
        ledger.inputs = vec![
            WoodMassInput { batch_id: "a".to_owned(), dry_mass_grams: u64::MAX },
            WoodMassInput { batch_id: "b".to_owned(), dry_mass_grams: 1 },
        ];
        let error = ledger.validate_mass_balance().expect_err("overflow must fail closed");
        assert!(error.contains("overflow"));
    }

    #[test]
    fn measured_process_requires_source_references() {
        let mut ledger = balanced_ledger();
        ledger.source_refs.clear();
        assert!(ledger.validate_mass_balance().expect_err("evidence needs sources")
            .contains("source references"));
    }

    #[test]
    fn duplicate_output_stream_ids_are_rejected() {
        let mut ledger = balanced_ledger();
        ledger.outputs[1].stream_id = ledger.outputs[0].stream_id.clone();
        assert!(ledger.validate_mass_balance().expect_err("stream IDs must be unique")
            .contains("duplicate output stream ID"));
    }
}
