// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Preregistered, non-authorizing evidence contracts for hospitality waste measurements.
//!
//! This module distinguishes physical observations, derived estimates, model imputations,
//! and explicit missingness. A valid assessment proves consistency of the submitted
//! evidence bundle, not that the underlying physical measurement actually occurred.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ReferenceId, ScopeRef};
use sha2::{Digest, Sha256};

pub const MAX_RATE_BASIS_POINTS: u16 = 10_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeasurementWindow {
    pub start_unix_ms: u64,
    pub end_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlannedWasteSlot {
    /// The measurement window for this slot, bounded by the protocol window.
    pub observation_window: MeasurementWindow,
    /// The target/event time must match any later measurement record exactly.
    pub target_at_unix_ms: u64,
    /// The waste category is fixed before collecting outcomes.
    pub category: ReferenceId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WasteMeasurementProtocol {
    pub protocol_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub window: MeasurementWindow,
    /// Registration must precede the registered measurement window.
    pub registered_at_unix_ms: u64,
    /// Each key binds one preregistered slot to its target time and category.
    /// Every planned slot must be represented by exactly one outcome.
    pub planned_slots: BTreeMap<ReferenceId, PlannedWasteSlot>,
    pub minimum_planned_slots: u32,
    pub maximum_missing_bps: u16,
    pub maximum_imputed_bps: u16,
    pub maximum_unobserved_bps: u16,
    pub minimum_direct_weighing_bps: u16,
    pub policy_digest: Digest32,
}

impl WasteMeasurementProtocol {
    pub fn validate(&self) -> Result<(), WasteMeasurementError> {
        if self.window.start_unix_ms >= self.window.end_unix_ms {
            return Err(WasteMeasurementError::InvalidWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.window.start_unix_ms
        {
            return Err(WasteMeasurementError::ProtocolNotPreregistered);
        }
        if self.minimum_planned_slots == 0
            || self.planned_slots.len() < self.minimum_planned_slots as usize
            || self.planned_slots.len() > u32::MAX as usize
        {
            return Err(WasteMeasurementError::InvalidPlannedSlotCount);
        }
        let mut slots_by_category: BTreeMap<ReferenceId, Vec<&PlannedWasteSlot>> = BTreeMap::new();
        for slot in self.planned_slots.values() {
            if slot.observation_window.start_unix_ms < self.window.start_unix_ms
                || slot.observation_window.start_unix_ms >= slot.observation_window.end_unix_ms
                || slot.observation_window.end_unix_ms > self.window.end_unix_ms
                || slot.target_at_unix_ms < slot.observation_window.start_unix_ms
                || slot.target_at_unix_ms >= slot.observation_window.end_unix_ms
            {
                return Err(WasteMeasurementError::InvalidPlannedSlot);
            }
            slots_by_category
                .entry(slot.category.clone())
                .or_default()
                .push(slot);
        }
        for slots in slots_by_category.values_mut() {
            slots.sort_by_key(|slot| slot.observation_window.start_unix_ms);
            if slots.windows(2).any(|pair| {
                pair[0].observation_window.end_unix_ms > pair[1].observation_window.start_unix_ms
            }) {
                return Err(WasteMeasurementError::InvalidPlannedSlot);
            }
        }
        if [
            self.maximum_missing_bps,
            self.maximum_imputed_bps,
            self.maximum_unobserved_bps,
            self.minimum_direct_weighing_bps,
        ]
        .into_iter()
        .any(|value| value > MAX_RATE_BASIS_POINTS)
        {
            return Err(WasteMeasurementError::InvalidThreshold);
        }
        if is_zero(&self.policy_digest) {
            return Err(WasteMeasurementError::ZeroDigest);
        }
        Ok(())
    }
}

/// Evidence about how a waste mass was obtained. Only `DirectWeighing` counts as
/// directly weighed. Count-derived and visual estimates are kept separate, and a
/// model output is always classified as imputed rather than physical ground truth.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MassEvidence {
    DirectWeighing {
        gross_grams: u64,
        tare_grams: u64,
        scale_id: ReferenceId,
        calibration_digest: Digest32,
        calibration_valid_from_unix_ms: u64,
        calibration_valid_until_unix_ms: u64,
    },
    CountDerived {
        count: u64,
        grams_per_unit: u64,
        conversion_digest: Digest32,
    },
    VisualEstimate {
        estimated_grams: u64,
        method_digest: Digest32,
    },
    ModelImputed {
        estimated_grams: u64,
        model_lineage: ReferenceId,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WasteMeasurementRecord {
    pub measurement_id: ReferenceId,
    pub slot_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub category: ReferenceId,
    pub reason: ReferenceId,
    /// Target/event time in the preregistered evaluation window.
    pub target_at_unix_ms: u64,
    /// Present only for physical observations; model-imputed values must use None.
    pub observed_at_unix_ms: Option<u64>,
    pub recorded_at_unix_ms: u64,
    /// Digest of the measurement record or model-output evidence.
    pub record_evidence_digest: Digest32,
    pub mass: MassEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MissingMeasurementRecord {
    pub slot_id: ReferenceId,
    pub source_system: ReferenceId,
    pub scope: ScopeRef,
    pub recorded_at_unix_ms: u64,
    pub reason: ReferenceId,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WasteSlotOutcome {
    Measured(WasteMeasurementRecord),
    NotObserved(MissingMeasurementRecord),
}

impl WasteSlotOutcome {
    pub fn slot_id(&self) -> &ReferenceId {
        match self {
            Self::Measured(record) => &record.slot_id,
            Self::NotObserved(record) => &record.slot_id,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WasteMeasurementError {
    InvalidWindow,
    ProtocolNotPreregistered,
    InvalidPlannedSlotCount,
    InvalidPlannedSlot,
    WrongSlotTarget,
    WrongWasteCategory,
    InvalidThreshold,
    ZeroDigest,
    EvaluationBeforeWindowCloses,
    DuplicateSlotDisposition,
    UnplannedSlot,
    UnaccountedSlots(Vec<ReferenceId>),
    WrongSourceSystem,
    WrongScope,
    ObservationOutsideWindow,
    MissingObservedTimestamp,
    UnexpectedObservedTimestamp,
    RecordedBeforeObservation,
    EvidenceAfterAssessment,
    ImputationBeforeSlotClosed,
    InvalidEvidenceTimestamp,
    MissingRecordEvidenceDigest,
    GrossBelowTare,
    InvalidCalibrationWindow,
    CalibrationNotValidAtObservation,
    InvalidDerivedMassEvidence,
    DuplicateMeasurementId,
    MassOverflow,
    RateArithmeticOverflow,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WasteMeasurementGateFailure {
    MissingRateExceeded { observed_bps: u16, maximum_bps: u16 },
    ImputedRateExceeded { observed_bps: u16, maximum_bps: u16 },
    UnobservedRateExceeded { observed_bps: u16, maximum_bps: u16 },
    DirectWeighingBelowMinimum { observed_bps: u16, minimum_bps: u16 },
}

/// Structurally validated measurement coverage. This assessment does not authenticate
/// instruments or prove that the submitted physical observations are true.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WasteMeasurementCoverageAssessment {
    pub protocol_id: ReferenceId,
    pub policy_digest: Digest32,
    pub evidence_digest: Digest32,
    pub assessed_at_unix_ms: u64,
    pub planned_slots: u32,
    pub directly_weighed_slots: u32,
    pub count_derived_slots: u32,
    pub visual_estimate_slots: u32,
    pub model_imputed_slots: u32,
    pub not_observed_slots: u32,
    /// Rates use basis points (10,000 = 100%).
    pub missing_rate_bps: u16,
    pub imputed_rate_bps: u16,
    pub unobserved_rate_bps: u16,
    pub direct_weighing_rate_bps: u16,
    pub non_imputed_observation_rate_bps: u16,
    pub directly_weighed_mass_grams: u64,
    pub count_derived_mass_grams: u64,
    pub visually_estimated_mass_grams: u64,
    pub model_imputed_mass_grams: u64,
    /// True only when all preregistered coverage thresholds pass. It is not a claim
    /// that physical waste was completely captured or that a business outcome improved.
    pub passes_registered_coverage_gates: bool,
    pub gate_failures: Vec<WasteMeasurementGateFailure>,
}

/// Validate the preregistered slot denominator, evidence, mass values, times, and
/// coverage thresholds. Missing observations must be explicitly represented in
/// `outcomes`; they are never silently dropped from the denominator.
pub fn assess_waste_measurement_coverage(
    protocol: &WasteMeasurementProtocol,
    outcomes: &[WasteSlotOutcome],
    assessed_at_unix_ms: u64,
) -> Result<WasteMeasurementCoverageAssessment, WasteMeasurementError> {
    protocol.validate()?;
    if assessed_at_unix_ms < protocol.window.end_unix_ms || assessed_at_unix_ms == 0 {
        return Err(WasteMeasurementError::EvaluationBeforeWindowCloses);
    }

    let mut outcome_slots = BTreeSet::new();
    let mut measurement_ids = BTreeSet::new();
    let mut canonical_outcomes: BTreeMap<ReferenceId, &WasteSlotOutcome> = BTreeMap::new();

    let mut direct_count = 0_u64;
    let mut count_derived_count = 0_u64;
    let mut visual_count = 0_u64;
    let mut imputed_count = 0_u64;
    let mut missing_count = 0_u64;

    let mut direct_mass = 0_u64;
    let mut count_derived_mass = 0_u64;
    let mut visual_mass = 0_u64;
    let mut imputed_mass = 0_u64;

    for outcome in outcomes {
        let slot_id = outcome.slot_id().clone();
        if !protocol.planned_slots.contains_key(&slot_id) {
            return Err(WasteMeasurementError::UnplannedSlot);
        }
        if !outcome_slots.insert(slot_id.clone()) {
            return Err(WasteMeasurementError::DuplicateSlotDisposition);
        }
        canonical_outcomes.insert(slot_id, outcome);

        match outcome {
            WasteSlotOutcome::Measured(record) => {
                if record.source_system != protocol.source_system {
                    return Err(WasteMeasurementError::WrongSourceSystem);
                }
                if record.scope != protocol.scope {
                    return Err(WasteMeasurementError::WrongScope);
                }
                let planned_slot = protocol
                    .planned_slots
                    .get(&record.slot_id)
                    .ok_or(WasteMeasurementError::UnplannedSlot)?;
                if record.target_at_unix_ms != planned_slot.target_at_unix_ms {
                    return Err(WasteMeasurementError::WrongSlotTarget);
                }
                if record.category != planned_slot.category {
                    return Err(WasteMeasurementError::WrongWasteCategory);
                }
                if record.recorded_at_unix_ms < protocol.registered_at_unix_ms {
                    return Err(WasteMeasurementError::InvalidEvidenceTimestamp);
                }
                if record.recorded_at_unix_ms > assessed_at_unix_ms {
                    return Err(WasteMeasurementError::EvidenceAfterAssessment);
                }
                if is_zero(&record.record_evidence_digest) {
                    return Err(WasteMeasurementError::MissingRecordEvidenceDigest);
                }
                if !measurement_ids.insert(record.measurement_id.clone()) {
                    return Err(WasteMeasurementError::DuplicateMeasurementId);
                }

                match &record.mass {
                    MassEvidence::DirectWeighing {
                        gross_grams,
                        tare_grams,
                        scale_id: _,
                        calibration_digest,
                        calibration_valid_from_unix_ms,
                        calibration_valid_until_unix_ms,
                    } => {
                        let observed_at = physical_observation_time(record, planned_slot)?;
                        if gross_grams < tare_grams {
                            return Err(WasteMeasurementError::GrossBelowTare);
                        }
                        if is_zero(calibration_digest) {
                            return Err(WasteMeasurementError::ZeroDigest);
                        }
                        if calibration_valid_from_unix_ms >= calibration_valid_until_unix_ms {
                            return Err(WasteMeasurementError::InvalidCalibrationWindow);
                        }
                        if observed_at < *calibration_valid_from_unix_ms
                            || observed_at >= *calibration_valid_until_unix_ms
                        {
                            return Err(WasteMeasurementError::CalibrationNotValidAtObservation);
                        }
                        let net = *gross_grams - *tare_grams;
                        direct_mass = direct_mass
                            .checked_add(net)
                            .ok_or(WasteMeasurementError::MassOverflow)?;
                        direct_count += 1;
                    }
                    MassEvidence::CountDerived {
                        count,
                        grams_per_unit,
                        conversion_digest,
                    } => {
                        physical_observation_time(record, planned_slot)?;
                        if is_zero(conversion_digest) {
                            return Err(WasteMeasurementError::ZeroDigest);
                        }
                        if *count > 0 && *grams_per_unit == 0 {
                            return Err(WasteMeasurementError::InvalidDerivedMassEvidence);
                        }
                        let mass = (*count)
                            .checked_mul(*grams_per_unit)
                            .ok_or(WasteMeasurementError::MassOverflow)?;
                        count_derived_mass = count_derived_mass
                            .checked_add(mass)
                            .ok_or(WasteMeasurementError::MassOverflow)?;
                        count_derived_count += 1;
                    }
                    MassEvidence::VisualEstimate {
                        estimated_grams,
                        method_digest,
                    } => {
                        physical_observation_time(record, planned_slot)?;
                        if is_zero(method_digest) {
                            return Err(WasteMeasurementError::ZeroDigest);
                        }
                        visual_mass = visual_mass
                            .checked_add(*estimated_grams)
                            .ok_or(WasteMeasurementError::MassOverflow)?;
                        visual_count += 1;
                    }
                    MassEvidence::ModelImputed {
                        estimated_grams,
                        model_lineage: _,
                    } => {
                        if record.observed_at_unix_ms.is_some() {
                            return Err(WasteMeasurementError::UnexpectedObservedTimestamp);
                        }
                        // A value recorded before the slot closes is a forecast, not a
                        // retrospective imputation for an unobserved outcome.
                        if record.recorded_at_unix_ms < planned_slot.observation_window.end_unix_ms
                        {
                            return Err(WasteMeasurementError::ImputationBeforeSlotClosed);
                        }
                        imputed_mass = imputed_mass
                            .checked_add(*estimated_grams)
                            .ok_or(WasteMeasurementError::MassOverflow)?;
                        imputed_count += 1;
                    }
                }
            }
            WasteSlotOutcome::NotObserved(record) => {
                if record.source_system != protocol.source_system {
                    return Err(WasteMeasurementError::WrongSourceSystem);
                }
                if record.scope != protocol.scope {
                    return Err(WasteMeasurementError::WrongScope);
                }
                let planned_slot = protocol
                    .planned_slots
                    .get(&record.slot_id)
                    .ok_or(WasteMeasurementError::UnplannedSlot)?;
                if record.recorded_at_unix_ms < planned_slot.observation_window.end_unix_ms {
                    return Err(WasteMeasurementError::InvalidEvidenceTimestamp);
                }
                if record.recorded_at_unix_ms > assessed_at_unix_ms {
                    return Err(WasteMeasurementError::EvidenceAfterAssessment);
                }
                missing_count += 1;
            }
        }
    }

    if outcome_slots.len() != protocol.planned_slots.len() {
        let unaccounted = protocol
            .planned_slots
            .keys()
            .filter(|slot_id| !outcome_slots.contains(*slot_id))
            .cloned()
            .collect::<Vec<_>>();
        return Err(WasteMeasurementError::UnaccountedSlots(unaccounted));
    }

    let planned = u64::try_from(protocol.planned_slots.len())
        .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?;
    let directly_observed = direct_count
        .checked_add(count_derived_count)
        .and_then(|value| value.checked_add(visual_count))
        .ok_or(WasteMeasurementError::RateArithmeticOverflow)?;
    let imputed_plus_missing = imputed_count
        .checked_add(missing_count)
        .ok_or(WasteMeasurementError::RateArithmeticOverflow)?;

    let missing_rate_bps = rate_bps_ceil(missing_count, planned)?;
    let imputed_rate_bps = rate_bps_ceil(imputed_count, planned)?;
    let unobserved_rate_bps = rate_bps_ceil(imputed_plus_missing, planned)?;
    let direct_weighing_rate_bps = rate_bps_floor(direct_count, planned)?;
    let non_imputed_observation_rate_bps = rate_bps_floor(directly_observed, planned)?;

    let mut gate_failures = Vec::new();
    if missing_rate_bps > protocol.maximum_missing_bps {
        gate_failures.push(WasteMeasurementGateFailure::MissingRateExceeded {
            observed_bps: missing_rate_bps,
            maximum_bps: protocol.maximum_missing_bps,
        });
    }
    if imputed_rate_bps > protocol.maximum_imputed_bps {
        gate_failures.push(WasteMeasurementGateFailure::ImputedRateExceeded {
            observed_bps: imputed_rate_bps,
            maximum_bps: protocol.maximum_imputed_bps,
        });
    }
    if unobserved_rate_bps > protocol.maximum_unobserved_bps {
        gate_failures.push(WasteMeasurementGateFailure::UnobservedRateExceeded {
            observed_bps: unobserved_rate_bps,
            maximum_bps: protocol.maximum_unobserved_bps,
        });
    }
    if direct_weighing_rate_bps < protocol.minimum_direct_weighing_bps {
        gate_failures.push(WasteMeasurementGateFailure::DirectWeighingBelowMinimum {
            observed_bps: direct_weighing_rate_bps,
            minimum_bps: protocol.minimum_direct_weighing_bps,
        });
    }

    let evidence_digest =
        measurement_evidence_digest(protocol, &canonical_outcomes, assessed_at_unix_ms);

    Ok(WasteMeasurementCoverageAssessment {
        protocol_id: protocol.protocol_id.clone(),
        policy_digest: protocol.policy_digest,
        evidence_digest,
        assessed_at_unix_ms,
        planned_slots: u32::try_from(planned)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        directly_weighed_slots: u32::try_from(direct_count)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        count_derived_slots: u32::try_from(count_derived_count)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        visual_estimate_slots: u32::try_from(visual_count)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        model_imputed_slots: u32::try_from(imputed_count)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        not_observed_slots: u32::try_from(missing_count)
            .map_err(|_| WasteMeasurementError::RateArithmeticOverflow)?,
        missing_rate_bps,
        imputed_rate_bps,
        unobserved_rate_bps,
        direct_weighing_rate_bps,
        non_imputed_observation_rate_bps,
        directly_weighed_mass_grams: direct_mass,
        count_derived_mass_grams: count_derived_mass,
        visually_estimated_mass_grams: visual_mass,
        model_imputed_mass_grams: imputed_mass,
        passes_registered_coverage_gates: gate_failures.is_empty(),
        gate_failures,
    })
}

fn physical_observation_time(
    record: &WasteMeasurementRecord,
    planned_slot: &PlannedWasteSlot,
) -> Result<u64, WasteMeasurementError> {
    let observed_at = record
        .observed_at_unix_ms
        .ok_or(WasteMeasurementError::MissingObservedTimestamp)?;
    if observed_at < planned_slot.observation_window.start_unix_ms
        || observed_at >= planned_slot.observation_window.end_unix_ms
    {
        return Err(WasteMeasurementError::ObservationOutsideWindow);
    }
    if record.recorded_at_unix_ms < observed_at {
        return Err(WasteMeasurementError::RecordedBeforeObservation);
    }
    Ok(observed_at)
}

fn rate_bps_ceil(numerator: u64, denominator: u64) -> Result<u16, WasteMeasurementError> {
    if denominator == 0 || numerator > denominator {
        return Err(WasteMeasurementError::RateArithmeticOverflow);
    }
    let numerator = u128::from(numerator) * u128::from(MAX_RATE_BASIS_POINTS);
    let denominator = u128::from(denominator);
    let rate = numerator.div_ceil(denominator);
    u16::try_from(rate).map_err(|_| WasteMeasurementError::RateArithmeticOverflow)
}

fn rate_bps_floor(numerator: u64, denominator: u64) -> Result<u16, WasteMeasurementError> {
    if denominator == 0 || numerator > denominator {
        return Err(WasteMeasurementError::RateArithmeticOverflow);
    }
    let rate = u128::from(numerator) * u128::from(MAX_RATE_BASIS_POINTS) / u128::from(denominator);
    u16::try_from(rate).map_err(|_| WasteMeasurementError::RateArithmeticOverflow)
}

fn is_zero(digest: &Digest32) -> bool {
    digest == &Digest32([0; 32])
}

fn hash_text(hasher: &mut Sha256, value: &str) {
    hash_u64(hasher, value.len() as u64);
    hasher.update(value.as_bytes());
}

fn hash_u64(hasher: &mut Sha256, value: u64) {
    hasher.update(value.to_be_bytes());
}

fn hash_digest(hasher: &mut Sha256, value: &Digest32) {
    hasher.update(value.0);
}

fn hash_ref(hasher: &mut Sha256, value: &ReferenceId) {
    hash_text(hasher, value.as_str());
}

fn hash_scope(hasher: &mut Sha256, value: &ScopeRef) {
    hash_ref(hasher, &value.0);
}

fn measurement_evidence_digest(
    protocol: &WasteMeasurementProtocol,
    outcomes: &BTreeMap<ReferenceId, &WasteSlotOutcome>,
    assessed_at_unix_ms: u64,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_text(
        &mut hasher,
        "mycelix:hospitality-waste-measurement-evidence:v1",
    );
    hash_ref(&mut hasher, &protocol.protocol_id);
    hash_ref(&mut hasher, &protocol.source_system);
    hash_scope(&mut hasher, &protocol.scope);
    hash_u64(&mut hasher, protocol.window.start_unix_ms);
    hash_u64(&mut hasher, protocol.window.end_unix_ms);
    hash_u64(&mut hasher, protocol.registered_at_unix_ms);
    hash_u64(&mut hasher, u64::from(protocol.minimum_planned_slots));
    hash_u64(&mut hasher, u64::from(protocol.maximum_missing_bps));
    hash_u64(&mut hasher, u64::from(protocol.maximum_imputed_bps));
    hash_u64(&mut hasher, u64::from(protocol.maximum_unobserved_bps));
    hash_u64(&mut hasher, u64::from(protocol.minimum_direct_weighing_bps));
    hash_digest(&mut hasher, &protocol.policy_digest);
    hash_u64(&mut hasher, protocol.planned_slots.len() as u64);
    for (slot_id, definition) in &protocol.planned_slots {
        hash_ref(&mut hasher, slot_id);
        hash_u64(&mut hasher, definition.observation_window.start_unix_ms);
        hash_u64(&mut hasher, definition.observation_window.end_unix_ms);
        hash_u64(&mut hasher, definition.target_at_unix_ms);
        hash_ref(&mut hasher, &definition.category);
    }
    hash_u64(&mut hasher, assessed_at_unix_ms);
    hash_u64(&mut hasher, outcomes.len() as u64);

    for (slot_id, outcome) in outcomes {
        hash_ref(&mut hasher, slot_id);
        match outcome {
            WasteSlotOutcome::NotObserved(record) => {
                hasher.update([0]);
                hash_ref(&mut hasher, &record.source_system);
                hash_scope(&mut hasher, &record.scope);
                hash_u64(&mut hasher, record.recorded_at_unix_ms);
                hash_ref(&mut hasher, &record.reason);
            }
            WasteSlotOutcome::Measured(record) => {
                hasher.update([1]);
                hash_ref(&mut hasher, &record.measurement_id);
                hash_ref(&mut hasher, &record.source_system);
                hash_scope(&mut hasher, &record.scope);
                hash_ref(&mut hasher, &record.category);
                hash_ref(&mut hasher, &record.reason);
                hash_u64(&mut hasher, record.target_at_unix_ms);
                match record.observed_at_unix_ms {
                    Some(observed_at) => {
                        hasher.update([1]);
                        hash_u64(&mut hasher, observed_at);
                    }
                    None => {
                        hasher.update([0]);
                    }
                }
                hash_u64(&mut hasher, record.recorded_at_unix_ms);
                hash_digest(&mut hasher, &record.record_evidence_digest);
                match &record.mass {
                    MassEvidence::DirectWeighing {
                        gross_grams,
                        tare_grams,
                        scale_id,
                        calibration_digest,
                        calibration_valid_from_unix_ms,
                        calibration_valid_until_unix_ms,
                    } => {
                        hasher.update([0]);
                        hash_u64(&mut hasher, *gross_grams);
                        hash_u64(&mut hasher, *tare_grams);
                        hash_ref(&mut hasher, scale_id);
                        hash_digest(&mut hasher, calibration_digest);
                        hash_u64(&mut hasher, *calibration_valid_from_unix_ms);
                        hash_u64(&mut hasher, *calibration_valid_until_unix_ms);
                    }
                    MassEvidence::CountDerived {
                        count,
                        grams_per_unit,
                        conversion_digest,
                    } => {
                        hasher.update([1]);
                        hash_u64(&mut hasher, *count);
                        hash_u64(&mut hasher, *grams_per_unit);
                        hash_digest(&mut hasher, conversion_digest);
                    }
                    MassEvidence::VisualEstimate {
                        estimated_grams,
                        method_digest,
                    } => {
                        hasher.update([2]);
                        hash_u64(&mut hasher, *estimated_grams);
                        hash_digest(&mut hasher, method_digest);
                    }
                    MassEvidence::ModelImputed {
                        estimated_grams,
                        model_lineage,
                    } => {
                        hasher.update([3]);
                        hash_u64(&mut hasher, *estimated_grams);
                        hash_ref(&mut hasher, model_lineage);
                    }
                }
            }
        }
    }

    let bytes: [u8; 32] = hasher.finalize().into();
    Digest32(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ref_id(value: &str) -> ReferenceId {
        ReferenceId::new(value).expect("test reference is canonical")
    }

    fn scope(value: &str) -> ScopeRef {
        ScopeRef(ref_id(value))
    }

    fn digest(value: u8) -> Digest32 {
        Digest32::repeat(value)
    }

    fn protocol() -> WasteMeasurementProtocol {
        WasteMeasurementProtocol {
            protocol_id: ref_id("protocol:waste:test:v1"),
            source_system: ref_id("source:pos:test"),
            scope: scope("scope:restaurant:one"),
            window: MeasurementWindow {
                start_unix_ms: 2_000,
                end_unix_ms: 6_000,
            },
            registered_at_unix_ms: 1_000,
            planned_slots: BTreeMap::from([
                (
                    ref_id("slot:one"),
                    PlannedWasteSlot {
                        observation_window: MeasurementWindow {
                            start_unix_ms: 2_900,
                            end_unix_ms: 3_100,
                        },
                        target_at_unix_ms: 3_000,
                        category: ref_id("waste:prep"),
                    },
                ),
                (
                    ref_id("slot:two"),
                    PlannedWasteSlot {
                        observation_window: MeasurementWindow {
                            start_unix_ms: 3_400,
                            end_unix_ms: 3_600,
                        },
                        target_at_unix_ms: 3_500,
                        category: ref_id("waste:prep"),
                    },
                ),
                (
                    ref_id("slot:three"),
                    PlannedWasteSlot {
                        observation_window: MeasurementWindow {
                            start_unix_ms: 3_900,
                            end_unix_ms: 4_100,
                        },
                        target_at_unix_ms: 4_000,
                        category: ref_id("waste:unsold"),
                    },
                ),
                (
                    ref_id("slot:four"),
                    PlannedWasteSlot {
                        observation_window: MeasurementWindow {
                            start_unix_ms: 4_900,
                            end_unix_ms: 5_100,
                        },
                        target_at_unix_ms: 5_000,
                        category: ref_id("waste:other"),
                    },
                ),
            ]),
            minimum_planned_slots: 4,
            maximum_missing_bps: 2_500,
            maximum_imputed_bps: 2_500,
            maximum_unobserved_bps: 5_000,
            minimum_direct_weighing_bps: 5_000,
            policy_digest: digest(1),
        }
    }

    fn weighed(
        slot: &str,
        measurement: &str,
        gross: u64,
        tare: u64,
        target_at_unix_ms: u64,
    ) -> WasteSlotOutcome {
        WasteSlotOutcome::Measured(WasteMeasurementRecord {
            measurement_id: ref_id(measurement),
            slot_id: ref_id(slot),
            source_system: ref_id("source:pos:test"),
            scope: scope("scope:restaurant:one"),
            category: ref_id("waste:prep"),
            reason: ref_id("reason:trim"),
            target_at_unix_ms,
            observed_at_unix_ms: Some(target_at_unix_ms),
            recorded_at_unix_ms: target_at_unix_ms + 100,
            record_evidence_digest: digest(2),
            mass: MassEvidence::DirectWeighing {
                gross_grams: gross,
                tare_grams: tare,
                scale_id: ref_id("scale:kitchen:one"),
                calibration_digest: digest(3),
                calibration_valid_from_unix_ms: 1_500,
                calibration_valid_until_unix_ms: 5_000,
            },
        })
    }

    fn outcomes() -> Vec<WasteSlotOutcome> {
        vec![
            weighed("slot:one", "measurement:one", 1_500, 500, 3_000),
            weighed("slot:two", "measurement:two", 1_800, 600, 3_500),
            WasteSlotOutcome::Measured(WasteMeasurementRecord {
                measurement_id: ref_id("measurement:three"),
                slot_id: ref_id("slot:three"),
                source_system: ref_id("source:pos:test"),
                scope: scope("scope:restaurant:one"),
                category: ref_id("waste:unsold"),
                reason: ref_id("reason:expired"),
                target_at_unix_ms: 4_000,
                observed_at_unix_ms: None,
                recorded_at_unix_ms: 4_100,
                record_evidence_digest: digest(4),
                mass: MassEvidence::ModelImputed {
                    estimated_grams: 900,
                    model_lineage: ref_id("model:waste:v1"),
                },
            }),
            WasteSlotOutcome::NotObserved(MissingMeasurementRecord {
                slot_id: ref_id("slot:four"),
                source_system: ref_id("source:pos:test"),
                scope: scope("scope:restaurant:one"),
                recorded_at_unix_ms: 6_100,
                reason: ref_id("reason:operator-unavailable"),
            }),
        ]
    }

    #[test]
    fn rejects_overlapping_slots_for_the_same_waste_category() {
        let mut value = protocol();
        value.planned_slots.insert(
            ref_id("slot:overlap"),
            PlannedWasteSlot {
                observation_window: MeasurementWindow {
                    start_unix_ms: 3_000,
                    end_unix_ms: 3_200,
                },
                target_at_unix_ms: 3_100,
                category: ref_id("waste:prep"),
            },
        );
        assert_eq!(
            value.validate(),
            Err(WasteMeasurementError::InvalidPlannedSlot)
        );
    }

    #[test]
    fn accepts_preregistered_denominator_and_separates_imputed_values() {
        let result = assess_waste_measurement_coverage(&protocol(), &outcomes(), 7_000).unwrap();

        assert_eq!(result.planned_slots, 4);
        assert_eq!(result.directly_weighed_slots, 2);
        assert_eq!(result.model_imputed_slots, 1);
        assert_eq!(result.not_observed_slots, 1);
        assert_eq!(result.direct_weighing_rate_bps, 5_000);
        assert_eq!(result.non_imputed_observation_rate_bps, 5_000);
        assert_eq!(result.missing_rate_bps, 2_500);
        assert_eq!(result.imputed_rate_bps, 2_500);
        assert_eq!(result.unobserved_rate_bps, 5_000);
        assert_eq!(result.directly_weighed_mass_grams, 2_200);
        assert_eq!(result.model_imputed_mass_grams, 900);
        assert!(result.passes_registered_coverage_gates);
    }

    #[test]
    fn rejects_target_time_changed_after_preregistration() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.target_at_unix_ms = 3_001;
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::WrongSlotTarget)
        );
    }

    #[test]
    fn rejects_waste_category_changed_after_preregistration() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.category = ref_id("waste:other");
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::WrongWasteCategory)
        );
    }

    #[test]
    fn model_imputation_cannot_claim_a_physical_observation_time() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            record.observed_at_unix_ms = Some(4_000);
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::UnexpectedObservedTimestamp)
        );
    }

    #[test]
    fn model_imputation_cannot_be_recorded_before_slot_closes() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            // The slot closes at 4_100. A model value written before then is a
            // forecast, and must not enter the outcome-imputation path.
            record.recorded_at_unix_ms = 4_099;
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::ImputationBeforeSlotClosed)
        );
    }

    #[test]
    fn direct_weighing_requires_an_observation_timestamp() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.observed_at_unix_ms = None;
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::MissingObservedTimestamp)
        );
    }

    #[test]
    fn count_derived_values_remain_separate_from_imputed_mass() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            record.observed_at_unix_ms = Some(4_000);
            record.record_evidence_digest = digest(9);
            record.mass = MassEvidence::CountDerived {
                count: 3,
                grams_per_unit: 100,
                conversion_digest: digest(8),
            };
        }
        let result = assess_waste_measurement_coverage(&protocol(), &values, 7_000).unwrap();
        assert_eq!(result.count_derived_slots, 1);
        assert_eq!(result.count_derived_mass_grams, 300);
        assert_eq!(result.non_imputed_observation_rate_bps, 7_500);
        assert_eq!(result.model_imputed_slots, 0);
        assert_eq!(result.model_imputed_mass_grams, 0);
    }

    #[test]
    fn visual_estimate_must_be_inside_its_preregistered_slot_window() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            // 4_500 is inside the overall protocol window [2_000, 6_000), but
            // outside this slot's narrower preregistered window [3_900, 4_100).
            record.observed_at_unix_ms = Some(4_500);
            record.recorded_at_unix_ms = 4_600;
            record.mass = MassEvidence::VisualEstimate {
                estimated_grams: 700,
                method_digest: digest(8),
            };
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::ObservationOutsideWindow)
        );
    }

    #[test]
    fn visual_estimates_remain_separate_from_direct_weighing() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            record.observed_at_unix_ms = Some(4_000);
            record.record_evidence_digest = digest(9);
            record.mass = MassEvidence::VisualEstimate {
                estimated_grams: 700,
                method_digest: digest(8),
            };
        }
        let result = assess_waste_measurement_coverage(&protocol(), &values, 7_000).unwrap();
        assert_eq!(result.visual_estimate_slots, 1);
        assert_eq!(result.visually_estimated_mass_grams, 700);
        assert_eq!(result.directly_weighed_slots, 2);
    }

    #[test]
    fn conservative_basis_point_rounding_is_directional() {
        assert_eq!(rate_bps_ceil(1, 3), Ok(3_334));
        assert_eq!(rate_bps_floor(1, 3), Ok(3_333));
    }

    #[test]
    fn rejects_mass_aggregation_overflow() {
        let mut values = outcomes();
        values[0] = weighed("slot:one", "measurement:one", u64::MAX, 0, 3_000);
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::MassOverflow)
        );
    }

    #[test]
    fn rejects_zero_unit_mass_for_positive_count() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[2] {
            record.observed_at_unix_ms = Some(4_000);
            record.mass = MassEvidence::CountDerived {
                count: 1,
                grams_per_unit: 0,
                conversion_digest: digest(8),
            };
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::InvalidDerivedMassEvidence)
        );
    }

    #[test]
    fn digest_is_deterministic_and_binds_measurement_changes() {
        let protocol = protocol();
        let first = assess_waste_measurement_coverage(&protocol, &outcomes(), 7_000).unwrap();
        let second = assess_waste_measurement_coverage(&protocol, &outcomes(), 7_000).unwrap();
        assert_eq!(first.evidence_digest, second.evidence_digest);

        let mut changed = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut changed[0] {
            record.mass = MassEvidence::DirectWeighing {
                gross_grams: 1_600,
                tare_grams: 500,
                scale_id: ref_id("scale:kitchen:one"),
                calibration_digest: digest(3),
                calibration_valid_from_unix_ms: 1_500,
                calibration_valid_until_unix_ms: 5_000,
            };
        }
        let altered = assess_waste_measurement_coverage(&protocol, &changed, 7_000).unwrap();
        assert_ne!(first.evidence_digest, altered.evidence_digest);
    }

    #[test]
    fn rejects_evaluation_before_window_closes() {
        let result = assess_waste_measurement_coverage(&protocol(), &outcomes(), 5_999);
        assert_eq!(
            result,
            Err(WasteMeasurementError::EvaluationBeforeWindowCloses)
        );
    }

    #[test]
    fn rejects_protocol_registered_after_window_starts() {
        let mut value = protocol();
        value.registered_at_unix_ms = value.window.start_unix_ms;
        assert_eq!(
            value.validate(),
            Err(WasteMeasurementError::ProtocolNotPreregistered)
        );
    }

    #[test]
    fn missing_slot_cannot_be_declared_before_its_window_closes() {
        let mut values = outcomes();
        if let WasteSlotOutcome::NotObserved(record) = &mut values[3] {
            record.recorded_at_unix_ms = 4_999;
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::InvalidEvidenceTimestamp)
        );
    }

    #[test]
    fn rejects_unaccounted_planned_slot() {
        let mut values = outcomes();
        values.pop();
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::UnaccountedSlots(vec![ref_id(
                "slot:four"
            )]))
        );
    }

    #[test]
    fn rejects_duplicate_slot_disposition() {
        let mut values = outcomes();
        values.push(values[0].clone());
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::DuplicateSlotDisposition)
        );
    }

    #[test]
    fn rejects_unplanned_slot() {
        let mut values = outcomes();
        values[3] = WasteSlotOutcome::NotObserved(MissingMeasurementRecord {
            slot_id: ref_id("slot:unknown"),
            source_system: ref_id("source:pos:test"),
            scope: scope("scope:restaurant:one"),
            recorded_at_unix_ms: 6_100,
            reason: ref_id("reason:missing"),
        });
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::UnplannedSlot)
        );
    }

    #[test]
    fn rejects_direct_weighing_gross_below_tare() {
        let mut values = outcomes();
        values[0] = weighed("slot:one", "measurement:one", 400, 500, 3_000);
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::GrossBelowTare)
        );
    }

    #[test]
    fn rejects_expired_calibration() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.mass = MassEvidence::DirectWeighing {
                gross_grams: 1_500,
                tare_grams: 500,
                scale_id: ref_id("scale:kitchen:one"),
                calibration_digest: digest(3),
                calibration_valid_from_unix_ms: 1_500,
                calibration_valid_until_unix_ms: 3_000,
            };
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::CalibrationNotValidAtObservation)
        );
    }

    #[test]
    fn rejects_missing_calibration_evidence() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.mass = MassEvidence::DirectWeighing {
                gross_grams: 1_500,
                tare_grams: 500,
                scale_id: ref_id("scale:kitchen:one"),
                calibration_digest: Digest32([0; 32]),
                calibration_valid_from_unix_ms: 1_500,
                calibration_valid_until_unix_ms: 5_000,
            };
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::ZeroDigest)
        );
    }

    #[test]
    fn rejects_observation_outside_its_slot_window_even_inside_protocol_window() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.observed_at_unix_ms = Some(3_300);
            record.recorded_at_unix_ms = 3_400;
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::ObservationOutsideWindow)
        );
    }

    #[test]
    fn rejects_observation_outside_window() {
        let mut values = outcomes();
        if let WasteSlotOutcome::Measured(record) = &mut values[0] {
            record.observed_at_unix_ms = Some(protocol().window.end_unix_ms);
        }
        assert_eq!(
            assess_waste_measurement_coverage(&protocol(), &values, 7_000),
            Err(WasteMeasurementError::ObservationOutsideWindow)
        );
    }

    #[test]
    fn coverage_gates_fail_without_required_direct_weighing() {
        let mut value = protocol();
        value.minimum_direct_weighing_bps = 7_500;
        let result = assess_waste_measurement_coverage(&value, &outcomes(), 7_000).unwrap();
        assert!(!result.passes_registered_coverage_gates);
        assert_eq!(result.direct_weighing_rate_bps, 5_000);
        assert!(result.gate_failures.iter().any(|failure| matches!(
            failure,
            WasteMeasurementGateFailure::DirectWeighingBelowMinimum { .. }
        )));
    }
}
