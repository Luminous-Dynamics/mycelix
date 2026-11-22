//! Pure assurance kernel for energy production claims, meter observations,
//! offer fills and payment settlement.
//!
//! This crate deliberately has no Holochain, networking, bank or cryptographic
//! dependency. Callers must authenticate the evidence and execute these rules
//! inside the authoritative integrity/transaction boundary.

#![forbid(unsafe_code)]

use std::collections::BTreeMap;

pub const MICROS_PER_HOUR: u128 = 3_600_000_000;
pub const WH_PER_KWH: u128 = 1_000;
pub const BASIS_POINTS: u128 = 10_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EnergyAssuranceError {
    MissingReference,
    InvalidInterval,
    InvalidCalibration,
    ClockQualityInsufficient,
    MeterTamperSuspected,
    MeterRegisterDecreased,
    MeterSequenceReplayed,
    CapacityExceeded,
    InvalidTolerance,
    ArithmeticOverflow,
    InvalidEnergyQuantity,
    InvalidOffer,
    OfferUnavailable,
    OfferVersionConflict { expected: u64, actual: u64 },
    OfferOverfilled,
    SelfTrade,
    IdempotencyKeyConflict,
    InvalidSettlementTransition,
    MissingSettlementEvidence,
    InsufficientManualAttestors,
    ProducerCannotVerifyOwnClaim,
    ClaimNotMeterDerived,
    InvalidProductionClaim,
    ClaimDigestMismatch,
    AttestationEvidenceNotInClaim,
    DuplicateEvidenceDigest,
    VerificationEvidenceMissing,
    ConflictedVerifier,
    UncertaintyTooHigh,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockQuality {
    Synchronized,
    BoundedDrift,
    Unsynchronized,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeterIntegrity {
    IntactUnderDeviceModel,
    TamperSuspected,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeterObservation {
    pub observation_id: String,
    pub meter_id: String,
    pub sequence: u64,
    pub previous_sequence: Option<u64>,
    pub cumulative_register_wh: u128,
    pub previous_register_wh: u128,
    pub interval_start_micros: i64,
    pub interval_end_micros: i64,
    pub calibration_valid_from_micros: i64,
    pub calibration_valid_until_micros: i64,
    pub clock_quality: ClockQuality,
    pub clock_drift_micros: u64,
    pub maximum_clock_drift_micros: u64,
    pub integrity: MeterIntegrity,
}

impl MeterObservation {
    pub fn interval_energy_wh(&self) -> Result<u128, EnergyAssuranceError> {
        if self.observation_id.is_empty() || self.meter_id.is_empty() {
            return Err(EnergyAssuranceError::MissingReference);
        }
        if self.sequence == 0 {
            return Err(EnergyAssuranceError::MeterSequenceReplayed);
        }
        if self.interval_end_micros <= self.interval_start_micros {
            return Err(EnergyAssuranceError::InvalidInterval);
        }
        if self.calibration_valid_from_micros > self.interval_start_micros
            || self.calibration_valid_until_micros < self.interval_end_micros
        {
            return Err(EnergyAssuranceError::InvalidCalibration);
        }
        if !matches!(
            self.clock_quality,
            ClockQuality::Synchronized | ClockQuality::BoundedDrift
        ) || self.clock_drift_micros > self.maximum_clock_drift_micros
        {
            return Err(EnergyAssuranceError::ClockQualityInsufficient);
        }
        if self.integrity != MeterIntegrity::IntactUnderDeviceModel {
            return Err(EnergyAssuranceError::MeterTamperSuspected);
        }
        if let Some(previous) = self.previous_sequence {
            if self.sequence <= previous {
                return Err(EnergyAssuranceError::MeterSequenceReplayed);
            }
        }
        if self.cumulative_register_wh < self.previous_register_wh {
            return Err(EnergyAssuranceError::MeterRegisterDecreased);
        }
        self.cumulative_register_wh
            .checked_sub(self.previous_register_wh)
            .ok_or(EnergyAssuranceError::MeterRegisterDecreased)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapacityBound {
    pub rated_capacity_w: u128,
    pub tolerance_bps: u16,
}

impl CapacityBound {
    pub fn validate_observation(
        self,
        observation: &MeterObservation,
    ) -> Result<u128, EnergyAssuranceError> {
        if self.rated_capacity_w == 0 {
            return Err(EnergyAssuranceError::InvalidEnergyQuantity);
        }
        if self.tolerance_bps as u128 > BASIS_POINTS {
            return Err(EnergyAssuranceError::InvalidTolerance);
        }
        let energy_wh = observation.interval_energy_wh()?;
        let duration_micros = observation
            .interval_end_micros
            .checked_sub(observation.interval_start_micros)
            .ok_or(EnergyAssuranceError::InvalidInterval)? as u128;
        let theoretical_wh = self
            .rated_capacity_w
            .checked_mul(duration_micros)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?
            / MICROS_PER_HOUR;
        let tolerated_wh = theoretical_wh
            .checked_mul(BASIS_POINTS + self.tolerance_bps as u128)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?
            / BASIS_POINTS;
        if energy_wh > tolerated_wh {
            return Err(EnergyAssuranceError::CapacityExceeded);
        }
        Ok(energy_wh)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProductionSource {
    ProducerAssertion,
    MeterDerived,
    Simulation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProductionClaim {
    pub claim_id: String,
    pub claim_digest: [u8; 32],
    pub producer_id: String,
    pub project_id: String,
    pub energy_wh: u128,
    pub source: ProductionSource,
    pub evidence_digests: Vec<[u8; 32]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerificationResult {
    VerifiedForScope,
    Rejected,
    Contested,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementUncertainty {
    Low,
    Moderate,
    High,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifierAttestation {
    pub attestation_id: String,
    pub claim_id: String,
    pub claim_digest: [u8; 32],
    pub verifier_id: String,
    pub independently_qualified: bool,
    pub conflicts_declared: bool,
    pub uncertainty: MeasurementUncertainty,
    pub observation_digests: Vec<[u8; 32]>,
    pub scope: String,
    pub result: VerificationResult,
}

impl VerifierAttestation {
    pub fn validate_against(&self, claim: &ProductionClaim) -> Result<(), EnergyAssuranceError> {
        if self.attestation_id.is_empty()
            || self.claim_id.is_empty()
            || self.scope.is_empty()
            || self.verifier_id.is_empty()
            || self.claim_id != claim.claim_id
        {
            return Err(EnergyAssuranceError::MissingReference);
        }
        if claim.producer_id.is_empty() || claim.project_id.is_empty() || claim.energy_wh == 0 {
            return Err(EnergyAssuranceError::InvalidProductionClaim);
        }
        if self.claim_digest != claim.claim_digest {
            return Err(EnergyAssuranceError::ClaimDigestMismatch);
        }
        if self.verifier_id == claim.producer_id {
            return Err(EnergyAssuranceError::ProducerCannotVerifyOwnClaim);
        }
        if !self.independently_qualified {
            return Err(EnergyAssuranceError::VerificationEvidenceMissing);
        }
        if self.conflicts_declared {
            return Err(EnergyAssuranceError::ConflictedVerifier);
        }
        if claim.source == ProductionSource::Simulation
            && self.result == VerificationResult::VerifiedForScope
        {
            return Err(EnergyAssuranceError::ClaimNotMeterDerived);
        }
        if self.result == VerificationResult::VerifiedForScope {
            if claim.source != ProductionSource::MeterDerived {
                return Err(EnergyAssuranceError::ClaimNotMeterDerived);
            }
            if self.observation_digests.is_empty() || claim.evidence_digests.is_empty() {
                return Err(EnergyAssuranceError::VerificationEvidenceMissing);
            }
            let unique_observations: std::collections::BTreeSet<_> =
                self.observation_digests.iter().collect();
            let unique_claim_evidence: std::collections::BTreeSet<_> =
                claim.evidence_digests.iter().collect();
            if unique_observations.len() != self.observation_digests.len()
                || unique_claim_evidence.len() != claim.evidence_digests.len()
            {
                return Err(EnergyAssuranceError::DuplicateEvidenceDigest);
            }
            if self
                .observation_digests
                .iter()
                .any(|digest| !unique_claim_evidence.contains(digest))
            {
                return Err(EnergyAssuranceError::AttestationEvidenceNotInClaim);
            }
            if matches!(
                self.uncertainty,
                MeasurementUncertainty::High | MeasurementUncertainty::Unknown
            ) {
                return Err(EnergyAssuranceError::UncertaintyTooHigh);
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnergyOffer {
    pub offer_id: String,
    pub seller_id: String,
    pub total_quantity_wh: u128,
    pub filled_quantity_wh: u128,
    pub price_per_kwh_minor: u128,
    pub currency: String,
    pub available_from_micros: i64,
    pub available_until_micros: i64,
    pub revision: u64,
}

impl EnergyOffer {
    pub fn validate(&self) -> Result<(), EnergyAssuranceError> {
        if self.offer_id.is_empty() || self.seller_id.is_empty() || self.currency.is_empty() {
            return Err(EnergyAssuranceError::MissingReference);
        }
        if self.total_quantity_wh == 0
            || self.filled_quantity_wh > self.total_quantity_wh
            || self.available_until_micros <= self.available_from_micros
        {
            return Err(EnergyAssuranceError::InvalidOffer);
        }
        Ok(())
    }

    pub fn remaining_quantity_wh(&self) -> Result<u128, EnergyAssuranceError> {
        self.total_quantity_wh
            .checked_sub(self.filled_quantity_wh)
            .ok_or(EnergyAssuranceError::OfferOverfilled)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnergyFill {
    pub fill_id: String,
    pub offer_id: String,
    pub buyer_id: String,
    pub quantity_wh: u128,
    pub total_price_minor: u128,
    pub currency: String,
    pub resulting_offer_revision: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FillDisposition {
    New(EnergyFill),
    ExactReplay(EnergyFill),
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ConsumedFillIntent {
    request_digest: [u8; 32],
    fill: EnergyFill,
}

#[derive(Debug, Clone, Default)]
pub struct EnergyOfferLedger {
    offers: BTreeMap<String, EnergyOffer>,
    consumed_intents: BTreeMap<String, ConsumedFillIntent>,
}

impl EnergyOfferLedger {
    pub fn insert_offer(&mut self, offer: EnergyOffer) -> Result<(), EnergyAssuranceError> {
        offer.validate()?;
        if self.offers.contains_key(&offer.offer_id) {
            return Err(EnergyAssuranceError::InvalidOffer);
        }
        self.offers.insert(offer.offer_id.clone(), offer);
        Ok(())
    }

    pub fn reserve_fill(
        &mut self,
        offer_id: &str,
        buyer_id: &str,
        quantity_wh: u128,
        expected_revision: u64,
        now_micros: i64,
        idempotency_key: &str,
        request_digest: [u8; 32],
    ) -> Result<FillDisposition, EnergyAssuranceError> {
        if idempotency_key.is_empty() || offer_id.is_empty() || buyer_id.is_empty() {
            return Err(EnergyAssuranceError::MissingReference);
        }
        if let Some(existing) = self.consumed_intents.get(idempotency_key) {
            if existing.request_digest == request_digest {
                return Ok(FillDisposition::ExactReplay(existing.fill.clone()));
            }
            return Err(EnergyAssuranceError::IdempotencyKeyConflict);
        }
        if quantity_wh == 0 {
            return Err(EnergyAssuranceError::InvalidEnergyQuantity);
        }

        let offer = self
            .offers
            .get_mut(offer_id)
            .ok_or(EnergyAssuranceError::InvalidOffer)?;
        offer.validate()?;
        if buyer_id == offer.seller_id {
            return Err(EnergyAssuranceError::SelfTrade);
        }
        if now_micros < offer.available_from_micros || now_micros >= offer.available_until_micros {
            return Err(EnergyAssuranceError::OfferUnavailable);
        }
        if expected_revision != offer.revision {
            return Err(EnergyAssuranceError::OfferVersionConflict {
                expected: expected_revision,
                actual: offer.revision,
            });
        }
        let remaining = offer.remaining_quantity_wh()?;
        if quantity_wh > remaining {
            return Err(EnergyAssuranceError::OfferOverfilled);
        }

        // Minor units are rounded up so a fractional minor-unit amount cannot be
        // undercharged. The policy must be disclosed to the buyer before approval.
        let numerator = quantity_wh
            .checked_mul(offer.price_per_kwh_minor)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?;
        let total_price_minor = numerator
            .checked_add(WH_PER_KWH - 1)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?
            / WH_PER_KWH;

        let new_filled = offer
            .filled_quantity_wh
            .checked_add(quantity_wh)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?;
        let new_revision = offer
            .revision
            .checked_add(1)
            .ok_or(EnergyAssuranceError::ArithmeticOverflow)?;

        let fill = EnergyFill {
            fill_id: idempotency_key.to_string(),
            offer_id: offer.offer_id.clone(),
            buyer_id: buyer_id.to_string(),
            quantity_wh,
            total_price_minor,
            currency: offer.currency.clone(),
            resulting_offer_revision: new_revision,
        };

        // All validation and arithmetic complete before mutation.
        offer.filled_quantity_wh = new_filled;
        offer.revision = new_revision;
        self.consumed_intents.insert(
            idempotency_key.to_string(),
            ConsumedFillIntent {
                request_digest,
                fill: fill.clone(),
            },
        );
        Ok(FillDisposition::New(fill))
    }

    pub fn offer(&self, offer_id: &str) -> Option<&EnergyOffer> {
        self.offers.get(offer_id)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RailOutcome {
    PaidFinal {
        rail_reference: String,
        finality_policy_ref: String,
    },
    Rejected {
        rail_reference: String,
        reason: String,
    },
    Indeterminate {
        reconciliation_reference: Option<String>,
    },
    UnsupportedCurrency,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettlementState {
    Prepared,
    Submitted,
    Indeterminate,
    PaidFinal,
    Rejected,
    ManualAttested,
    Compensated,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManualSettlementAttestation {
    pub evidence_reference: String,
    pub attestor_ids: Vec<String>,
    pub buyer_id: String,
    pub seller_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TradeSettlement {
    pub trade_id: String,
    pub buyer_id: String,
    pub seller_id: String,
    pub amount_minor: u128,
    pub currency: String,
    state: SettlementState,
    rail_reference: Option<String>,
    finality_policy_ref: Option<String>,
    reconciliation_reference: Option<String>,
}

impl TradeSettlement {
    pub fn new(
        trade_id: String,
        buyer_id: String,
        seller_id: String,
        amount_minor: u128,
        currency: String,
    ) -> Result<Self, EnergyAssuranceError> {
        if trade_id.is_empty() || buyer_id.is_empty() || seller_id.is_empty() || currency.is_empty()
        {
            return Err(EnergyAssuranceError::MissingReference);
        }
        if buyer_id == seller_id || amount_minor == 0 {
            return Err(EnergyAssuranceError::InvalidOffer);
        }
        Ok(Self {
            trade_id,
            buyer_id,
            seller_id,
            amount_minor,
            currency,
            state: SettlementState::Prepared,
            rail_reference: None,
            finality_policy_ref: None,
            reconciliation_reference: None,
        })
    }

    pub fn state(&self) -> SettlementState {
        self.state
    }

    pub fn mark_submitted(&mut self) -> Result<(), EnergyAssuranceError> {
        if self.state != SettlementState::Prepared {
            return Err(EnergyAssuranceError::InvalidSettlementTransition);
        }
        self.state = SettlementState::Submitted;
        Ok(())
    }

    pub fn apply_rail_outcome(&mut self, outcome: RailOutcome) -> Result<(), EnergyAssuranceError> {
        if !matches!(
            self.state,
            SettlementState::Submitted | SettlementState::Indeterminate
        ) {
            return Err(EnergyAssuranceError::InvalidSettlementTransition);
        }
        match outcome {
            RailOutcome::PaidFinal {
                rail_reference,
                finality_policy_ref,
            } => {
                if rail_reference.is_empty() || finality_policy_ref.is_empty() {
                    return Err(EnergyAssuranceError::MissingSettlementEvidence);
                }
                self.rail_reference = Some(rail_reference);
                self.finality_policy_ref = Some(finality_policy_ref);
                self.state = SettlementState::PaidFinal;
            }
            RailOutcome::Rejected {
                rail_reference,
                reason,
            } => {
                if rail_reference.is_empty() || reason.is_empty() {
                    return Err(EnergyAssuranceError::MissingSettlementEvidence);
                }
                self.rail_reference = Some(rail_reference);
                self.reconciliation_reference = Some(reason);
                self.state = SettlementState::Rejected;
            }
            RailOutcome::Indeterminate {
                reconciliation_reference,
            } => {
                self.reconciliation_reference = reconciliation_reference;
                self.state = SettlementState::Indeterminate;
            }
            RailOutcome::UnsupportedCurrency => {
                self.state = SettlementState::Rejected;
                self.reconciliation_reference =
                    Some("unsupported rail; separate manual process required".to_string());
            }
        }
        Ok(())
    }

    /// A manual process is explicitly distinguished from rail-final payment.
    /// It can never transition the state to PaidFinal.
    pub fn record_manual_attestation(
        &mut self,
        attestation: ManualSettlementAttestation,
    ) -> Result<(), EnergyAssuranceError> {
        if !matches!(
            self.state,
            SettlementState::Prepared | SettlementState::Rejected
        ) {
            return Err(EnergyAssuranceError::InvalidSettlementTransition);
        }
        if attestation.evidence_reference.is_empty() {
            return Err(EnergyAssuranceError::MissingSettlementEvidence);
        }
        let unique: std::collections::BTreeSet<String> =
            attestation.attestor_ids.iter().cloned().collect();
        if unique.len() < 2
            || attestation.attestor_ids.len() < 2
            || unique.contains(&attestation.buyer_id)
            || unique.contains(&attestation.seller_id)
        {
            return Err(EnergyAssuranceError::InsufficientManualAttestors);
        }
        self.reconciliation_reference = Some(attestation.evidence_reference);
        self.state = SettlementState::ManualAttested;
        Ok(())
    }

    pub fn record_compensation(
        &mut self,
        reconciliation_reference: String,
    ) -> Result<(), EnergyAssuranceError> {
        if self.state != SettlementState::Rejected || reconciliation_reference.is_empty() {
            return Err(EnergyAssuranceError::InvalidSettlementTransition);
        }
        self.reconciliation_reference = Some(reconciliation_reference);
        self.state = SettlementState::Compensated;
        Ok(())
    }

    pub fn rail_reference(&self) -> Option<&str> {
        self.rail_reference.as_deref()
    }

    pub fn finality_policy_ref(&self) -> Option<&str> {
        self.finality_policy_ref.as_deref()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn meter_observation() -> MeterObservation {
        MeterObservation {
            observation_id: "obs-001".into(),
            meter_id: "meter-001".into(),
            sequence: 2,
            previous_sequence: Some(1),
            cumulative_register_wh: 1_200_000,
            previous_register_wh: 0,
            interval_start_micros: 0,
            interval_end_micros: 3_600_000_000,
            calibration_valid_from_micros: 0,
            calibration_valid_until_micros: 7_200_000_000,
            clock_quality: ClockQuality::Synchronized,
            clock_drift_micros: 10_000,
            maximum_clock_drift_micros: 100_000,
            integrity: MeterIntegrity::IntactUnderDeviceModel,
        }
    }

    fn offer() -> EnergyOffer {
        EnergyOffer {
            offer_id: "offer-001".into(),
            seller_id: "seller-001".into(),
            total_quantity_wh: 1_000_000,
            filled_quantity_wh: 0,
            price_per_kwh_minor: 250,
            currency: "ZAR".into(),
            available_from_micros: 0,
            available_until_micros: 10_000_000_000,
            revision: 1,
        }
    }

    #[test]
    fn valid_meter_observation_returns_interval_energy() {
        assert_eq!(meter_observation().interval_energy_wh().unwrap(), 1_200_000);
    }

    #[test]
    fn invalid_or_overlapping_meter_interval_is_rejected() {
        let mut meter = meter_observation();
        meter.interval_end_micros = meter.interval_start_micros;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::InvalidInterval)
        );
    }

    #[test]
    fn expired_calibration_is_rejected() {
        let mut meter = meter_observation();
        meter.calibration_valid_until_micros = meter.interval_end_micros - 1;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::InvalidCalibration)
        );
    }

    #[test]
    fn meter_sequence_replay_is_rejected() {
        let mut meter = meter_observation();
        meter.sequence = 1;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::MeterSequenceReplayed)
        );
    }

    #[test]
    fn cumulative_register_decrease_is_rejected() {
        let mut meter = meter_observation();
        meter.cumulative_register_wh = 10;
        meter.previous_register_wh = 20;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::MeterRegisterDecreased)
        );
    }

    #[test]
    fn unknown_clock_quality_is_not_verified() {
        let mut meter = meter_observation();
        meter.clock_quality = ClockQuality::Unknown;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::ClockQualityInsufficient)
        );
    }

    #[test]
    fn tamper_suspected_meter_is_not_verified() {
        let mut meter = meter_observation();
        meter.integrity = MeterIntegrity::TamperSuspected;
        assert_eq!(
            meter.interval_energy_wh(),
            Err(EnergyAssuranceError::MeterTamperSuspected)
        );
    }

    #[test]
    fn rated_capacity_bounds_energy_by_interval_duration() {
        let bound = CapacityBound {
            rated_capacity_w: 1_000,
            tolerance_bps: 0,
        };
        assert_eq!(
            bound.validate_observation(&meter_observation()),
            Err(EnergyAssuranceError::CapacityExceeded)
        );
    }

    #[test]
    fn capacity_bound_accepts_plausible_output_with_tolerance() {
        let mut meter = meter_observation();
        meter.cumulative_register_wh = 900;
        let bound = CapacityBound {
            rated_capacity_w: 1_000,
            tolerance_bps: 0,
        };
        assert_eq!(bound.validate_observation(&meter).unwrap(), 900);
    }

    #[test]
    fn simulation_cannot_be_attested_as_verified_production() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(), claim_digest: [2; 32],
            producer_id: "producer-001".into(), project_id: "project-001".into(), energy_wh: 1000,
            source: ProductionSource::Simulation, evidence_digests: vec![[1; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(), claim_id: claim.claim_id.clone(),
            claim_digest: [2; 32], verifier_id: "inspector-001".into(),
            independently_qualified: true, conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Low, observation_digests: vec![[3; 32]],
            scope: "interval-output".into(), result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(attestation.validate_against(&claim), Err(EnergyAssuranceError::ClaimNotMeterDerived));
    }

    #[test]
    fn attestation_must_bind_exact_claim_digest() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(), claim_digest: [1; 32],
            producer_id: "producer-001".into(), project_id: "project-001".into(),
            energy_wh: 1000, source: ProductionSource::MeterDerived,
            evidence_digests: vec![[3; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(), claim_id: claim.claim_id.clone(),
            claim_digest: [9; 32], verifier_id: "inspector-001".into(),
            independently_qualified: true, conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Low, observation_digests: vec![[3; 32]],
            scope: "interval-output".into(), result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(
            attestation.validate_against(&claim),
            Err(EnergyAssuranceError::ClaimDigestMismatch)
        );
    }

    #[test]
    fn attestation_cannot_reference_observations_outside_claim_evidence() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(), claim_digest: [2; 32],
            producer_id: "producer-001".into(), project_id: "project-001".into(),
            energy_wh: 1000, source: ProductionSource::MeterDerived,
            evidence_digests: vec![[3; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(), claim_id: claim.claim_id.clone(),
            claim_digest: [2; 32], verifier_id: "inspector-001".into(),
            independently_qualified: true, conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Low, observation_digests: vec![[4; 32]],
            scope: "interval-output".into(), result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(
            attestation.validate_against(&claim),
            Err(EnergyAssuranceError::AttestationEvidenceNotInClaim)
        );
    }

    #[test]
    fn duplicate_evidence_digest_is_rejected() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(),
            claim_digest: [2; 32],
            producer_id: "producer-001".into(),
            project_id: "project-001".into(),
            energy_wh: 1000,
            source: ProductionSource::MeterDerived,
            evidence_digests: vec![[3; 32], [3; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(),
            claim_id: claim.claim_id.clone(),
            claim_digest: [2; 32],
            verifier_id: "inspector-001".into(),
            independently_qualified: true,
            conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Low,
            observation_digests: vec![[3; 32]],
            scope: "interval-output".into(),
            result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(
            attestation.validate_against(&claim),
            Err(EnergyAssuranceError::DuplicateEvidenceDigest)
        );
    }

    #[test]
    fn producer_cannot_verify_own_claim() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(), claim_digest: [2; 32],
            producer_id: "producer-001".into(), project_id: "project-001".into(),
            energy_wh: 1000,
            source: ProductionSource::MeterDerived, evidence_digests: vec![[1; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(), claim_id: claim.claim_id.clone(),
            claim_digest: [2; 32], verifier_id: claim.producer_id.clone(),
            independently_qualified: true, conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Low, observation_digests: vec![[3; 32]],
            scope: "interval-output".into(), result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(attestation.validate_against(&claim), Err(EnergyAssuranceError::ProducerCannotVerifyOwnClaim));
    }

    #[test]
    fn valid_independent_meter_attestation_passes_scope_checks() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(), claim_digest: [2; 32],
            producer_id: "producer-001".into(), project_id: "project-001".into(),
            energy_wh: 1000,
            source: ProductionSource::MeterDerived, evidence_digests: vec![[1; 32]],
        };
        let attestation = VerifierAttestation {
            attestation_id: "attest-001".into(), claim_id: claim.claim_id.clone(),
            claim_digest: [2; 32], verifier_id: "inspector-001".into(),
            independently_qualified: true, conflicts_declared: false,
            uncertainty: MeasurementUncertainty::Moderate, observation_digests: vec![[3; 32]],
            scope: "interval-output".into(), result: VerificationResult::VerifiedForScope,
        };
        assert_eq!(attestation.validate_against(&claim), Ok(()));
    }

    #[test]
    fn energy_offer_rejects_oversell_without_mutating_offer() {
        let mut ledger = EnergyOfferLedger::default();
        ledger.insert_offer(offer()).unwrap();
        let result = ledger.reserve_fill(
            "offer-001",
            "buyer-001",
            1_000_001,
            1,
            1,
            "fill-001",
            [1; 32],
        );
        assert_eq!(result, Err(EnergyAssuranceError::OfferOverfilled));
        assert_eq!(ledger.offer("offer-001").unwrap().filled_quantity_wh, 0);
        assert_eq!(ledger.offer("offer-001").unwrap().revision, 1);
    }

    #[test]
    fn stale_offer_revision_is_rejected() {
        let mut ledger = EnergyOfferLedger::default();
        ledger.insert_offer(offer()).unwrap();
        let result = ledger.reserve_fill("offer-001", "buyer-001", 100, 0, 1, "fill-001", [1; 32]);
        assert_eq!(
            result,
            Err(EnergyAssuranceError::OfferVersionConflict {
                expected: 0,
                actual: 1
            })
        );
    }

    #[test]
    fn exact_fill_retry_returns_original_receipt_without_double_consumption() {
        let mut ledger = EnergyOfferLedger::default();
        ledger.insert_offer(offer()).unwrap();
        let first = ledger
            .reserve_fill("offer-001", "buyer-001", 400_000, 1, 1, "fill-001", [1; 32])
            .unwrap();
        assert!(matches!(first, FillDisposition::New(_)));
        let retry = ledger
            .reserve_fill("offer-001", "buyer-001", 400_000, 1, 1, "fill-001", [1; 32])
            .unwrap();
        assert!(matches!(retry, FillDisposition::ExactReplay(_)));
        assert_eq!(
            ledger.offer("offer-001").unwrap().filled_quantity_wh,
            400_000
        );
    }

    #[test]
    fn reused_fill_key_with_different_payload_is_rejected() {
        let mut ledger = EnergyOfferLedger::default();
        ledger.insert_offer(offer()).unwrap();
        ledger
            .reserve_fill("offer-001", "buyer-001", 100, 1, 1, "fill-001", [1; 32])
            .unwrap();
        assert_eq!(
            ledger.reserve_fill("offer-001", "buyer-001", 101, 1, 1, "fill-001", [2; 32]),
            Err(EnergyAssuranceError::IdempotencyKeyConflict)
        );
    }

    #[test]
    fn fractional_minor_currency_amount_rounds_up_transparently() {
        let mut ledger = EnergyOfferLedger::default();
        ledger.insert_offer(offer()).unwrap();
        let result = ledger
            .reserve_fill("offer-001", "buyer-001", 1, 1, 1, "fill-001", [1; 32])
            .unwrap();
        let FillDisposition::New(fill) = result else {
            panic!("new fill expected")
        };
        assert_eq!(fill.total_price_minor, 1);
    }

    #[test]
    fn unsupported_currency_does_not_become_paid_final_from_reference_string() {
        let mut settlement = TradeSettlement::new(
            "trade-001".into(),
            "buyer-001".into(),
            "seller-001".into(),
            100,
            "USD".into(),
        )
        .unwrap();
        settlement.mark_submitted().unwrap();
        settlement
            .apply_rail_outcome(RailOutcome::UnsupportedCurrency)
            .unwrap();
        assert_eq!(settlement.state(), SettlementState::Rejected);
        assert!(settlement.rail_reference().is_none());
    }

    #[test]
    fn unknown_external_payout_is_not_paid_final() {
        let mut settlement = TradeSettlement::new(
            "trade-001".into(),
            "buyer-001".into(),
            "seller-001".into(),
            100,
            "ZAR".into(),
        )
        .unwrap();
        settlement.mark_submitted().unwrap();
        settlement
            .apply_rail_outcome(RailOutcome::Indeterminate {
                reconciliation_reference: None,
            })
            .unwrap();
        assert_eq!(settlement.state(), SettlementState::Indeterminate);
        assert!(settlement.rail_reference().is_none());
    }

    #[test]
    fn only_external_rail_finality_can_mark_paid_final() {
        let mut settlement = TradeSettlement::new(
            "trade-001".into(), "buyer-001".into(), "seller-001".into(), 100, "ZAR".into()
        ).unwrap();
        settlement.mark_submitted().unwrap();
        settlement
            .apply_rail_outcome(RailOutcome::PaidFinal {
                rail_reference: "bank-rail-123".into(),
                finality_policy_ref: "rail-finality-policy/1".into(),
            })
            .unwrap();
        assert_eq!(settlement.state(), SettlementState::PaidFinal);
        assert_eq!(settlement.rail_reference(), Some("bank-rail-123"));
        assert_eq!(settlement.apply_rail_outcome(RailOutcome::PaidFinal {
            rail_reference: "duplicate".into(), finality_policy_ref: "policy".into()
        }), Err(EnergyAssuranceError::InvalidSettlementTransition));
    }

    #[test]
    fn manual_attestation_remains_distinct_from_paid_final() {
        let mut settlement = TradeSettlement::new(
            "trade-001".into(), "buyer-001".into(), "seller-001".into(), 100, "USD".into()
        ).unwrap();
        settlement
            .record_manual_attestation(ManualSettlementAttestation {
                evidence_reference: "manual-evidence-001".into(),
                attestor_ids: vec!["inspector-1".into(), "auditor-2".into()],
                buyer_id: "buyer-001".into(),
                seller_id: "seller-001".into(),
            })
            .unwrap();
        assert_eq!(settlement.state(), SettlementState::ManualAttested);
        assert_ne!(settlement.state(), SettlementState::PaidFinal);
    }

    #[test]
    fn buyer_or_seller_cannot_be_manual_attestor() {
        let mut settlement = TradeSettlement::new(
            "trade-001".into(), "buyer-001".into(), "seller-001".into(), 100, "USD".into()
        ).unwrap();
        let result = settlement.record_manual_attestation(ManualSettlementAttestation {
            evidence_reference: "manual-evidence-001".into(),
            attestor_ids: vec!["buyer-001".into(), "auditor-2".into()],
            buyer_id: "buyer-001".into(),
            seller_id: "seller-001".into(),
        });
        assert_eq!(
            result,
            Err(EnergyAssuranceError::InsufficientManualAttestors)
        );
    }
}
