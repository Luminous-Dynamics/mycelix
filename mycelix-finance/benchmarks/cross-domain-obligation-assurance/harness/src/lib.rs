//! Dependency-free policy kernel for the PCOE benchmark.
//!
//! This is a deterministic in-memory reference model. It does not authenticate
//! real signatures, validate legal authority, prove physical metrology, call a
//! bank, or provide distributed atomicity. Production adapters must enforce
//! these invariants at their authoritative commit boundaries.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

pub type Digest = [u8; 32];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KernelError {
    MissingReference,
    InvalidAmount,
    Unauthorized,
    AuthorityExpired,
    AuthorityRevoked,
    AuthorityStateUnknown,
    ScopeExceeded,
    CapabilityConsumed,
    HumanDispositionRequired,
    InvalidInterval,
    InvalidCalibration,
    InvalidMeterSequence,
    InvalidBoundary,
    InvalidTolerance,
    CapacityExceeded,
    ArithmeticOverflow,
    SimulationIsNotMeasurement,
    InvalidClaim,
    ClaimDigestMismatch,
    EvidenceDigestMismatch,
    DuplicateEvidence,
    ProducerCannotVerify,
    VerifierNotQualified,
    VerifierConflict,
    InsufficientWitnesses,
    DuplicateWitness,
    CommonCauseFailureDomain,
    ContractDigestMismatch,
    MilestoneDigestMismatch,
    InsufficientEvidence,
    DuplicateOffer,
    OfferNotFound,
    OfferRevisionConflict { expected: u64, actual: u64 },
    OfferOverfilled,
    IdempotencyConflict,
    PaymentInFlight,
    ReconciliationRequired,
    InvalidPaymentTransition,
    UnauthenticatedReceipt,
    ReceiptMismatch,
    DuplicateExternalReceipt,
    InvalidFinalityPolicy,
    InvalidManualAttestation,
    DuplicateCollateral,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityGrant {
    pub grant_id: String,
    pub principal_id: String,
    pub task_id: String,
    pub audience_id: String,
    pub actor_id: String,
    pub action_class: String,
    pub resource_id: String,
    pub max_amount: u128,
    pub not_before: u64,
    pub expires_at: u64,
    pub revocation_epoch: u64,
    pub requires_human_disposition: bool,
    pub one_time: bool,
    pub human_approver_ids: BTreeSet<String>,
    pub max_human_approval_age_secs: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ActionRequest {
    pub principal_id: String,
    pub task_id: String,
    pub audience_id: String,
    pub request_digest: Digest,
    pub actor_id: String,
    pub action_class: String,
    pub resource_id: String,
    pub amount: u128,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HumanDisposition {
    pub decision_maker_id: String,
    pub decision_reference: String,
    pub request_digest: Digest,
    /// Test seam set only after a separate human authentication channel is verified.
    pub independent_authentication_verified: bool,
    pub approved_at: u64,
}

pub fn authorize(
    grant: &AuthorityGrant,
    request: &ActionRequest,
    now: u64,
    observed_revocation_epoch: u64,
    human_disposition: Option<&HumanDisposition>,
) -> Result<(), KernelError> {
    if grant.grant_id.is_empty()
        || grant.principal_id.is_empty()
        || grant.task_id.is_empty()
        || grant.audience_id.is_empty()
        || grant.actor_id.is_empty()
        || grant.action_class.is_empty()
        || grant.resource_id.is_empty()
        || request.principal_id.is_empty()
        || request.task_id.is_empty()
        || request.audience_id.is_empty()
        || request.actor_id.is_empty()
        || request.action_class.is_empty()
        || request.resource_id.is_empty()
    {
        return Err(KernelError::MissingReference);
    }
    if grant.not_before >= grant.expires_at {
        return Err(KernelError::AuthorityExpired);
    }
    if now < grant.not_before || now >= grant.expires_at {
        return Err(KernelError::AuthorityExpired);
    }
    if observed_revocation_epoch > grant.revocation_epoch {
        return Err(KernelError::AuthorityRevoked);
    }
    if observed_revocation_epoch < grant.revocation_epoch {
        return Err(KernelError::AuthorityStateUnknown);
    }
    if grant.principal_id != request.principal_id
        || grant.task_id != request.task_id
        || grant.audience_id != request.audience_id
        || grant.actor_id != request.actor_id
        || grant.action_class != request.action_class
        || grant.resource_id != request.resource_id
    {
        return Err(KernelError::Unauthorized);
    }
    if request.amount == 0 {
        return Err(KernelError::InvalidAmount);
    }
    if request.amount > grant.max_amount {
        return Err(KernelError::ScopeExceeded);
    }
    if grant.requires_human_disposition || request.action_class.starts_with("BR3") {
        let disposition = human_disposition.ok_or(KernelError::HumanDispositionRequired)?;
        if disposition.decision_maker_id.is_empty()
            || disposition.decision_reference.is_empty()
            || disposition.decision_maker_id == request.actor_id
            || disposition.request_digest != request.request_digest
            || !grant
                .human_approver_ids
                .contains(&disposition.decision_maker_id)
            || grant.max_human_approval_age_secs == 0
            || disposition.approved_at > now
            || now - disposition.approved_at > grant.max_human_approval_age_secs
            || !disposition.independent_authentication_verified
        {
            return Err(KernelError::HumanDispositionRequired);
        }
    }
    Ok(())
}

#[derive(Debug, Clone, Default)]
pub struct AuthorityConsumptionLedger {
    consumed_grants: BTreeMap<String, Digest>,
}

impl AuthorityConsumptionLedger {
    /// Enforces one-time grants. Call atomically with the authorized side effect
    /// in a production integrity/transaction boundary.
    pub fn consume(
        &mut self,
        grant: &AuthorityGrant,
        request: &ActionRequest,
    ) -> Result<(), KernelError> {
        if grant.grant_id.is_empty() || request.request_digest == [0; 32] {
            return Err(KernelError::MissingReference);
        }
        if !grant.one_time {
            return Ok(());
        }
        if let Some(previous_digest) = self.consumed_grants.get(&grant.grant_id) {
            if previous_digest == &request.request_digest {
                return Err(KernelError::CapabilityConsumed);
            }
            return Err(KernelError::IdempotencyConflict);
        }
        self.consumed_grants
            .insert(grant.grant_id.clone(), request.request_digest);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClaimSource {
    ProducerAssertion,
    MeterDerived,
    Simulation,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MeasurementBoundary {
    PvDcOutput,
    InverterAcOutput,
    SiteNetGeneration,
    GridExport,
    GridImport,
    StorageCharge,
    StorageDischarge,
    SiteLoad,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MeterObservation {
    pub observation_id: String,
    pub producer_id: String,
    pub meter_id: String,
    pub sequence: u64,
    pub previous_sequence: Option<u64>,
    pub interval_start_micros: i64,
    pub interval_end_micros: i64,
    pub calibration_valid_from_micros: i64,
    pub calibration_valid_until_micros: i64,
    pub cumulative_register_wh: u128,
    pub previous_register_wh: u128,
    pub rated_capacity_w: u128,
    pub tolerance_bps: u16,
    pub boundary: MeasurementBoundary,
    pub raw_payload_digest: Digest,
    pub source: ClaimSource,
}

impl MeterObservation {
    pub fn interval_energy_wh(
        &self,
        expected_boundary: MeasurementBoundary,
        maximum_tolerance_bps: u16,
    ) -> Result<u128, KernelError> {
        if self.observation_id.is_empty() || self.producer_id.is_empty() || self.meter_id.is_empty()
        {
            return Err(KernelError::MissingReference);
        }
        if self.source == ClaimSource::Simulation {
            return Err(KernelError::SimulationIsNotMeasurement);
        }
        if self.source != ClaimSource::MeterDerived {
            return Err(KernelError::InvalidClaim);
        }
        if self.sequence == 0
            || self
                .previous_sequence
                .is_some_and(|previous| self.sequence <= previous)
        {
            return Err(KernelError::InvalidMeterSequence);
        }
        if self.interval_start_micros < 0 || self.interval_end_micros <= self.interval_start_micros
        {
            return Err(KernelError::InvalidInterval);
        }
        if self.calibration_valid_from_micros > self.interval_start_micros
            || self.calibration_valid_until_micros < self.interval_end_micros
        {
            return Err(KernelError::InvalidCalibration);
        }
        if self.boundary != expected_boundary {
            return Err(KernelError::InvalidBoundary);
        }
        if maximum_tolerance_bps > 10_000 || self.tolerance_bps > maximum_tolerance_bps {
            return Err(KernelError::InvalidTolerance);
        }
        if self.raw_payload_digest == [0; 32] {
            return Err(KernelError::MissingReference);
        }
        if self.rated_capacity_w == 0 {
            return Err(KernelError::InvalidAmount);
        }
        if self.cumulative_register_wh < self.previous_register_wh {
            return Err(KernelError::InvalidInterval);
        }
        let energy_wh = self
            .cumulative_register_wh
            .checked_sub(self.previous_register_wh)
            .ok_or(KernelError::InvalidInterval)?;
        let duration_micros = self
            .interval_end_micros
            .checked_sub(self.interval_start_micros)
            .ok_or(KernelError::InvalidInterval)? as u128;
        let max_numerator = self
            .rated_capacity_w
            .checked_mul(duration_micros)
            .and_then(|value| value.checked_mul(10_000u128 + self.tolerance_bps as u128))
            .ok_or(KernelError::ArithmeticOverflow)?;
        let max_denominator = 3_600_000_000u128
            .checked_mul(10_000)
            .ok_or(KernelError::ArithmeticOverflow)?;
        let maximum_energy_wh = max_numerator / max_denominator;
        if energy_wh > maximum_energy_wh {
            return Err(KernelError::CapacityExceeded);
        }
        Ok(energy_wh)
    }
}

#[derive(Debug, Clone, Default)]
pub struct MeterSequenceLedger {
    consumed: BTreeSet<(String, MeasurementBoundary, u64)>,
}

impl MeterSequenceLedger {
    /// Consumes a meter sequence at the authoritative boundary.
    /// Production implementations must persist this uniqueness constraint atomically.
    pub fn consume(&mut self, observation: &MeterObservation) -> Result<(), KernelError> {
        if observation.meter_id.is_empty() || observation.sequence == 0 {
            return Err(KernelError::MissingReference);
        }
        let identity = (
            observation.meter_id.clone(),
            observation.boundary,
            observation.sequence,
        );
        if !self.consumed.insert(identity) {
            return Err(KernelError::InvalidMeterSequence);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProductionClaim {
    pub claim_id: String,
    pub producer_id: String,
    pub claim_digest: Digest,
    pub energy_wh: u128,
    pub source: ClaimSource,
    pub evidence_digests: Vec<Digest>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifierAttestation {
    pub attestation_id: String,
    pub verifier_id: String,
    pub claim_id: String,
    pub claim_digest: Digest,
    pub evidence_digests: Vec<Digest>,
    pub witnesses: Vec<VerifierWitness>,
    pub scope: String,
    pub qualified: bool,
    pub conflicts_declared: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifierWitness {
    pub verifier_id: String,
    /// Material common-cause failure domains declared by the verifier.
    /// Examples: controlling legal entity, operator, source feed, device admin,
    /// calibration provider, key custodian, and cloud/control plane.
    pub failure_domains: BTreeSet<String>,
}

pub fn validate_witness_independence(
    producer_id: &str,
    witnesses: &[VerifierWitness],
) -> Result<(), KernelError> {
    if producer_id.is_empty() || witnesses.len() < 2 {
        return Err(KernelError::InsufficientWitnesses);
    }
    let mut verifier_ids = BTreeSet::new();
    for witness in witnesses {
        if witness.verifier_id.is_empty() || witness.verifier_id == producer_id {
            return Err(KernelError::ProducerCannotVerify);
        }
        if witness.failure_domains.is_empty() {
            return Err(KernelError::InsufficientWitnesses);
        }
        if !verifier_ids.insert(witness.verifier_id.as_str()) {
            return Err(KernelError::DuplicateWitness);
        }
    }
    for left in 0..witnesses.len() {
        for right in (left + 1)..witnesses.len() {
            if !witnesses[left]
                .failure_domains
                .is_disjoint(&witnesses[right].failure_domains)
            {
                return Err(KernelError::CommonCauseFailureDomain);
            }
        }
    }
    Ok(())
}

pub fn validate_attestation(
    claim: &ProductionClaim,
    attestation: &VerifierAttestation,
) -> Result<(), KernelError> {
    if claim.claim_id.is_empty()
        || claim.producer_id.is_empty()
        || attestation.attestation_id.is_empty()
        || attestation.verifier_id.is_empty()
        || attestation.scope.is_empty()
        || claim.energy_wh == 0
    {
        return Err(KernelError::InvalidClaim);
    }
    if claim.source != ClaimSource::MeterDerived {
        return Err(KernelError::SimulationIsNotMeasurement);
    }
    if attestation.verifier_id == claim.producer_id {
        return Err(KernelError::ProducerCannotVerify);
    }
    if !attestation.qualified {
        return Err(KernelError::VerifierNotQualified);
    }
    validate_witness_independence(&claim.producer_id, &attestation.witnesses)?;
    if !attestation
        .witnesses
        .iter()
        .any(|witness| witness.verifier_id == attestation.verifier_id)
    {
        return Err(KernelError::InsufficientWitnesses);
    }
    if attestation.conflicts_declared {
        return Err(KernelError::VerifierConflict);
    }
    if attestation.claim_id != claim.claim_id || attestation.claim_digest != claim.claim_digest {
        return Err(KernelError::ClaimDigestMismatch);
    }
    let claim_evidence: BTreeSet<_> = claim.evidence_digests.iter().collect();
    let attested_evidence: BTreeSet<_> = attestation.evidence_digests.iter().collect();
    if claim_evidence.len() != claim.evidence_digests.len()
        || attested_evidence.len() != attestation.evidence_digests.len()
    {
        return Err(KernelError::DuplicateEvidence);
    }
    if claim_evidence.is_empty() || claim_evidence != attested_evidence {
        return Err(KernelError::EvidenceDigestMismatch);
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AcceptanceDecision {
    Accepted,
    PartiallyAccepted,
    Rejected,
    Disputed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MilestoneAcceptance {
    pub obligation_id: String,
    pub contract_digest: Digest,
    pub milestone_digest: Digest,
    pub evidence_digests: Vec<Digest>,
    pub decision: AcceptanceDecision,
    pub decision_maker: String,
    pub authority_reference: String,
}

pub fn validate_acceptance(
    acceptance: &MilestoneAcceptance,
    expected_obligation_id: &str,
    expected_contract_digest: Digest,
    expected_milestone_digest: Digest,
    required_evidence_digests: &[Digest],
) -> Result<(), KernelError> {
    if acceptance.obligation_id.is_empty()
        || acceptance.decision_maker.is_empty()
        || acceptance.authority_reference.is_empty()
    {
        return Err(KernelError::MissingReference);
    }
    if acceptance.obligation_id != expected_obligation_id {
        return Err(KernelError::Unauthorized);
    }
    if acceptance.contract_digest != expected_contract_digest {
        return Err(KernelError::ContractDigestMismatch);
    }
    if acceptance.milestone_digest != expected_milestone_digest {
        return Err(KernelError::MilestoneDigestMismatch);
    }
    let supplied: BTreeSet<_> = acceptance.evidence_digests.iter().collect();
    let required: BTreeSet<_> = required_evidence_digests.iter().collect();
    if supplied.len() != acceptance.evidence_digests.len()
        || required.len() != required_evidence_digests.len()
        || supplied != required
    {
        return Err(KernelError::InsufficientEvidence);
    }
    Ok(())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OfferState {
    pub offer_id: String,
    pub remaining_wh: u128,
    pub revision: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FillReceipt {
    pub offer_id: String,
    pub idempotency_key: String,
    pub request_digest: Digest,
    pub quantity_wh: u128,
    pub remaining_wh: u128,
    pub resulting_revision: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FillDisposition {
    Applied(FillReceipt),
    ExactReplay(FillReceipt),
}

#[derive(Debug, Clone, Default)]
pub struct OfferLedger {
    offers: BTreeMap<String, OfferState>,
    consumed: BTreeMap<String, (Digest, FillReceipt)>,
}

impl OfferLedger {
    pub fn create_offer(&mut self, offer_id: &str, amount_wh: u128) -> Result<(), KernelError> {
        if offer_id.is_empty() || amount_wh == 0 {
            return Err(KernelError::InvalidAmount);
        }
        if self.offers.contains_key(offer_id) {
            return Err(KernelError::DuplicateOffer);
        }
        self.offers.insert(
            offer_id.to_owned(),
            OfferState {
                offer_id: offer_id.to_owned(),
                remaining_wh: amount_wh,
                revision: 0,
            },
        );
        Ok(())
    }

    pub fn fill(
        &mut self,
        offer_id: &str,
        expected_revision: u64,
        quantity_wh: u128,
        idempotency_key: &str,
        request_digest: Digest,
    ) -> Result<FillDisposition, KernelError> {
        if idempotency_key.is_empty() || offer_id.is_empty() {
            return Err(KernelError::MissingReference);
        }
        if let Some((existing_digest, receipt)) = self.consumed.get(idempotency_key) {
            if existing_digest == &request_digest
                && receipt.offer_id == offer_id
                && receipt.quantity_wh == quantity_wh
                && receipt.idempotency_key == idempotency_key
                && receipt.resulting_revision.checked_sub(1) == Some(expected_revision)
            {
                return Ok(FillDisposition::ExactReplay(receipt.clone()));
            }
            return Err(KernelError::IdempotencyConflict);
        }
        if quantity_wh == 0 {
            return Err(KernelError::InvalidAmount);
        }
        let offer = self
            .offers
            .get_mut(offer_id)
            .ok_or(KernelError::OfferNotFound)?;
        if offer.revision != expected_revision {
            return Err(KernelError::OfferRevisionConflict {
                expected: expected_revision,
                actual: offer.revision,
            });
        }
        if quantity_wh > offer.remaining_wh {
            return Err(KernelError::OfferOverfilled);
        }
        let remaining_wh = offer
            .remaining_wh
            .checked_sub(quantity_wh)
            .ok_or(KernelError::ArithmeticOverflow)?;
        let resulting_revision = offer
            .revision
            .checked_add(1)
            .ok_or(KernelError::ArithmeticOverflow)?;
        offer.remaining_wh = remaining_wh;
        offer.revision = resulting_revision;
        let receipt = FillReceipt {
            offer_id: offer_id.to_owned(),
            idempotency_key: idempotency_key.to_owned(),
            request_digest,
            quantity_wh,
            remaining_wh,
            resulting_revision,
        };
        self.consumed.insert(
            idempotency_key.to_owned(),
            (request_digest, receipt.clone()),
        );
        Ok(FillDisposition::Applied(receipt))
    }

    pub fn offer(&self, offer_id: &str) -> Option<&OfferState> {
        self.offers.get(offer_id)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PaymentRequest {
    pub obligation_id: String,
    pub payer_id: String,
    pub payee_id: String,
    pub amount_minor: u128,
    pub currency: String,
    pub idempotency_key: String,
    pub request_digest: Digest,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RailReceipt {
    pub external_receipt_id: String,
    pub obligation_id: String,
    pub payer_id: String,
    pub payee_id: String,
    pub amount_minor: u128,
    pub currency: String,
    pub idempotency_key: String,
    pub request_digest: Digest,
    pub finality_policy_reference: String,
    /// Set only by an upstream adapter after authenticating the receipt.
    pub authentication_verified: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PaymentStatus {
    Submitted,
    Indeterminate,
    PaidFinal {
        external_receipt_id: String,
        finality_policy_reference: String,
    },
    Rejected {
        external_receipt_id: String,
    },
    ManualAttested {
        attestor_a: String,
        attestor_b: String,
        evidence_reference: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BeginPayment {
    NewIntent,
    InFlight,
    ReconciliationRequired,
    ExactReplay(PaymentStatus),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RailOutcome {
    PaidFinal(RailReceipt),
    Rejected(RailReceipt),
    Indeterminate,
}

#[derive(Debug, Clone, Default)]
pub struct PaymentLedger {
    intents: BTreeMap<String, (PaymentRequest, PaymentStatus)>,
    consumed_receipts: BTreeSet<String>,
    side_effect_attempts: u64,
}

impl PaymentLedger {
    pub fn begin(&mut self, request: PaymentRequest) -> Result<BeginPayment, KernelError> {
        if request.obligation_id.is_empty()
            || request.payer_id.is_empty()
            || request.payee_id.is_empty()
            || request.idempotency_key.is_empty()
            || request.currency.len() != 3
            || !request
                .currency
                .bytes()
                .all(|byte| byte.is_ascii_uppercase())
        {
            return Err(KernelError::MissingReference);
        }
        if request.amount_minor == 0 || request.payer_id == request.payee_id {
            return Err(KernelError::InvalidAmount);
        }
        if let Some((existing_request, status)) = self.intents.get(&request.idempotency_key) {
            if existing_request.request_digest != request.request_digest
                || existing_request != &request
            {
                return Err(KernelError::IdempotencyConflict);
            }
            return Ok(match status {
                PaymentStatus::Submitted => BeginPayment::InFlight,
                PaymentStatus::Indeterminate => BeginPayment::ReconciliationRequired,
                terminal => BeginPayment::ExactReplay(terminal.clone()),
            });
        }
        self.side_effect_attempts = self
            .side_effect_attempts
            .checked_add(1)
            .ok_or(KernelError::ArithmeticOverflow)?;
        self.intents.insert(
            request.idempotency_key.clone(),
            (request, PaymentStatus::Submitted),
        );
        Ok(BeginPayment::NewIntent)
    }

    pub fn reconcile(
        &mut self,
        idempotency_key: &str,
        outcome: RailOutcome,
    ) -> Result<PaymentStatus, KernelError> {
        match outcome {
            RailOutcome::Indeterminate => {
                let (_, status) = self
                    .intents
                    .get_mut(idempotency_key)
                    .ok_or(KernelError::InvalidPaymentTransition)?;
                if !matches!(
                    status,
                    PaymentStatus::Submitted | PaymentStatus::Indeterminate
                ) {
                    return Err(KernelError::InvalidPaymentTransition);
                }
                *status = PaymentStatus::Indeterminate;
                Ok(status.clone())
            }
            RailOutcome::PaidFinal(receipt) => {
                self.record_rail_receipt(idempotency_key, receipt, true)
            }
            RailOutcome::Rejected(receipt) => {
                self.record_rail_receipt(idempotency_key, receipt, false)
            }
        }
    }

    fn record_rail_receipt(
        &mut self,
        idempotency_key: &str,
        receipt: RailReceipt,
        is_paid: bool,
    ) -> Result<PaymentStatus, KernelError> {
        let (request, current_status) = self
            .intents
            .get(idempotency_key)
            .ok_or(KernelError::InvalidPaymentTransition)?;
        if !matches!(
            current_status,
            PaymentStatus::Submitted | PaymentStatus::Indeterminate
        ) {
            return Err(KernelError::InvalidPaymentTransition);
        }
        if !receipt.authentication_verified {
            return Err(KernelError::UnauthenticatedReceipt);
        }
        if receipt.external_receipt_id.is_empty() || receipt.finality_policy_reference.is_empty() {
            return Err(KernelError::InvalidFinalityPolicy);
        }
        if receipt.obligation_id != request.obligation_id
            || receipt.payer_id != request.payer_id
            || receipt.payee_id != request.payee_id
            || receipt.amount_minor != request.amount_minor
            || receipt.currency != request.currency
            || receipt.idempotency_key != request.idempotency_key
            || receipt.request_digest != request.request_digest
        {
            return Err(KernelError::ReceiptMismatch);
        }
        if self
            .consumed_receipts
            .contains(&receipt.external_receipt_id)
        {
            return Err(KernelError::DuplicateExternalReceipt);
        }
        self.consumed_receipts
            .insert(receipt.external_receipt_id.clone());
        let status = if is_paid {
            PaymentStatus::PaidFinal {
                external_receipt_id: receipt.external_receipt_id,
                finality_policy_reference: receipt.finality_policy_reference,
            }
        } else {
            PaymentStatus::Rejected {
                external_receipt_id: receipt.external_receipt_id,
            }
        };
        self.intents
            .get_mut(idempotency_key)
            .ok_or(KernelError::InvalidPaymentTransition)?
            .1 = status.clone();
        Ok(status)
    }

    pub fn record_manual_attestation(
        &mut self,
        idempotency_key: &str,
        attestor_a: &str,
        attestor_b: &str,
        payer_id: &str,
        payee_id: &str,
        evidence_reference: &str,
    ) -> Result<PaymentStatus, KernelError> {
        let (_, status) = self
            .intents
            .get_mut(idempotency_key)
            .ok_or(KernelError::InvalidPaymentTransition)?;
        if *status != PaymentStatus::Indeterminate
            || attestor_a.is_empty()
            || attestor_b.is_empty()
            || attestor_a == attestor_b
            || attestor_a == payer_id
            || attestor_a == payee_id
            || attestor_b == payer_id
            || attestor_b == payee_id
            || evidence_reference.is_empty()
        {
            return Err(KernelError::InvalidManualAttestation);
        }
        *status = PaymentStatus::ManualAttested {
            attestor_a: attestor_a.to_owned(),
            attestor_b: attestor_b.to_owned(),
            evidence_reference: evidence_reference.to_owned(),
        };
        Ok(status.clone())
    }

    pub fn status(&self, idempotency_key: &str) -> Option<&PaymentStatus> {
        self.intents.get(idempotency_key).map(|entry| &entry.1)
    }

    pub fn side_effect_attempts(&self) -> u64 {
        self.side_effect_attempts
    }
}

#[derive(Debug, Clone, Default)]
pub struct CollateralLedger {
    pledged_assets: BTreeMap<String, String>,
}

impl CollateralLedger {
    pub fn pledge(&mut self, asset_id: &str, obligation_id: &str) -> Result<(), KernelError> {
        if asset_id.is_empty() || obligation_id.is_empty() {
            return Err(KernelError::MissingReference);
        }
        if self.pledged_assets.contains_key(asset_id) {
            return Err(KernelError::DuplicateCollateral);
        }
        self.pledged_assets
            .insert(asset_id.to_owned(), obligation_id.to_owned());
        Ok(())
    }

    pub fn pledged_to(&self, asset_id: &str) -> Option<&str> {
        self.pledged_assets.get(asset_id).map(String::as_str)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoverageStatus {
    ImplementedInKernelTests,
    PartialOrRequiresAdapter,
    NotYetImplemented,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioCoverage {
    pub id: &'static str,
    pub status: CoverageStatus,
}

pub const SCENARIO_COVERAGE: [ScenarioCoverage; 39] = [
    ScenarioCoverage {
        id: "AUTH-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AUTH-002",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AUTH-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AUTH-004",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AUTH-005",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AUTH-006",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "EVID-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EVID-002",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EVID-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EVID-004",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EVID-005",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "EVID-006",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "VER-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "VER-002",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "VER-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "VER-004",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "VER-005",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "ACC-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "ACC-002",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "ACC-003",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "OFFER-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "COLL-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-002",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "PAY-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-004",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-005",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-006",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "PAY-007",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-008",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "PAY-009",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "DIST-001",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "DIST-002",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "DIST-003",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "PRIV-001",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "HRM-001",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MODEL-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "MODEL-002",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "RECON-001",
        status: CoverageStatus::NotYetImplemented,
    },
];

pub const AGENT_SCENARIO_COVERAGE: [ScenarioCoverage; 10] = [
    ScenarioCoverage {
        id: "AGENT-001",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AGENT-002",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AGENT-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AGENT-004",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AGENT-005",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AGENT-006",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AGENT-007",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "AGENT-008",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "AGENT-009",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "AGENT-010",
        status: CoverageStatus::NotYetImplemented,
    },
];

pub const TRANSPARENCY_METROLOGY_SCENARIO_COVERAGE: [ScenarioCoverage; 10] = [
    ScenarioCoverage {
        id: "TSP-001",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "TSP-002",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "TSP-003",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "TSP-004",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "TSP-005",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MET-001",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MET-002",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MET-003",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MET-004",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "MARKET-001",
        status: CoverageStatus::NotYetImplemented,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(value: u8) -> Digest {
        [value; 32]
    }

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            grant_id: "grant-001".into(),
            principal_id: "community-board".into(),
            task_id: "procure-solar-001".into(),
            audience_id: "mycelix-finance".into(),
            actor_id: "procurement-agent".into(),
            action_class: "BR2_PURCHASE".into(),
            resource_id: "solar-project-001".into(),
            max_amount: 1_000_000,
            not_before: 100,
            expires_at: 500,
            revocation_epoch: 7,
            requires_human_disposition: false,
            one_time: true,
            human_approver_ids: BTreeSet::new(),
            max_human_approval_age_secs: 300,
        }
    }

    fn request() -> ActionRequest {
        ActionRequest {
            principal_id: "community-board".into(),
            task_id: "procure-solar-001".into(),
            audience_id: "mycelix-finance".into(),
            request_digest: digest(9),
            actor_id: "procurement-agent".into(),
            action_class: "BR2_PURCHASE".into(),
            resource_id: "solar-project-001".into(),
            amount: 500_000,
        }
    }

    fn meter() -> MeterObservation {
        MeterObservation {
            observation_id: "obs-001".into(),
            producer_id: "producer-001".into(),
            meter_id: "meter-001".into(),
            sequence: 2,
            previous_sequence: Some(1),
            interval_start_micros: 0,
            interval_end_micros: 3_600_000_000,
            calibration_valid_from_micros: 0,
            calibration_valid_until_micros: 7_200_000_000,
            cumulative_register_wh: 900_000,
            previous_register_wh: 0,
            rated_capacity_w: 1_000_000,
            tolerance_bps: 0,
            boundary: MeasurementBoundary::InverterAcOutput,
            raw_payload_digest: digest(1),
            source: ClaimSource::MeterDerived,
        }
    }

    fn payment_request(key: &str, amount: u128, digest_value: u8) -> PaymentRequest {
        PaymentRequest {
            obligation_id: "obligation-001".into(),
            payer_id: "community".into(),
            payee_id: "installer".into(),
            amount_minor: amount,
            currency: "ZAR".into(),
            idempotency_key: key.into(),
            request_digest: digest(digest_value),
        }
    }

    fn receipt(request: &PaymentRequest, receipt_id: &str) -> RailReceipt {
        RailReceipt {
            external_receipt_id: receipt_id.into(),
            obligation_id: request.obligation_id.clone(),
            payer_id: request.payer_id.clone(),
            payee_id: request.payee_id.clone(),
            amount_minor: request.amount_minor,
            currency: request.currency.clone(),
            idempotency_key: request.idempotency_key.clone(),
            request_digest: request.request_digest,
            finality_policy_reference: "rail-finality/v1".into(),
            authentication_verified: true,
        }
    }

    #[test]
    fn valid_authority_is_accepted() {
        assert_eq!(authorize(&grant(), &request(), 200, 7, None), Ok(()));
    }

    #[test]
    fn authority_expiry_scope_revocation_and_br3_fail_closed() {
        assert_eq!(
            authorize(&grant(), &request(), 500, 7, None),
            Err(KernelError::AuthorityExpired)
        );
        assert_eq!(
            authorize(&grant(), &request(), 200, 8, None),
            Err(KernelError::AuthorityRevoked)
        );
        assert_eq!(
            authorize(&grant(), &request(), 200, 6, None),
            Err(KernelError::AuthorityStateUnknown)
        );
        let mut oversized = request();
        oversized.amount += 1;
        assert_eq!(
            authorize(&grant(), &oversized, 200, 7, None),
            Err(KernelError::ScopeExceeded)
        );
        let mut wrong_audience = request();
        wrong_audience.audience_id = "other-service".into();
        assert_eq!(
            authorize(&grant(), &wrong_audience, 200, 7, None),
            Err(KernelError::Unauthorized)
        );
        let mut wrong_task = request();
        wrong_task.task_id = "different-task".into();
        assert_eq!(
            authorize(&grant(), &wrong_task, 200, 7, None),
            Err(KernelError::Unauthorized)
        );
        let mut br3 = grant();
        br3.action_class = "BR3_ASSET_TRANSFER".into();
        let mut br3_request = request();
        br3_request.action_class = "BR3_ASSET_TRANSFER".into();
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, None),
            Err(KernelError::HumanDispositionRequired)
        );
        br3.human_approver_ids.insert("board-member".into());
        br3.max_human_approval_age_secs = 30;
        let approval = HumanDisposition {
            decision_maker_id: "board-member".into(),
            decision_reference: "human-decision-001".into(),
            request_digest: br3_request.request_digest,
            independent_authentication_verified: true,
            approved_at: 190,
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&approval)),
            Ok(())
        );
        let agent_approval = HumanDisposition {
            decision_maker_id: br3_request.actor_id.clone(),
            ..approval.clone()
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&agent_approval)),
            Err(KernelError::HumanDispositionRequired)
        );
        let unknown_approver = HumanDisposition {
            decision_maker_id: "unlisted-person".into(),
            ..approval.clone()
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&unknown_approver)),
            Err(KernelError::HumanDispositionRequired)
        );
        let stale_approval = HumanDisposition {
            approved_at: 100,
            ..approval.clone()
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&stale_approval)),
            Err(KernelError::HumanDispositionRequired)
        );
        let future_approval = HumanDisposition {
            approved_at: 201,
            ..approval.clone()
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&future_approval)),
            Err(KernelError::HumanDispositionRequired)
        );
        let mismatched_approval = HumanDisposition {
            request_digest: digest(99),
            ..approval
        };
        assert_eq!(
            authorize(&br3, &br3_request, 200, 7, Some(&mismatched_approval)),
            Err(KernelError::HumanDispositionRequired)
        );
    }

    #[test]
    fn one_time_authority_grant_cannot_be_reused() {
        let grant = grant();
        let request = request();
        let mut ledger = AuthorityConsumptionLedger::default();
        assert_eq!(ledger.consume(&grant, &request), Ok(()));
        assert_eq!(
            ledger.consume(&grant, &request),
            Err(KernelError::CapabilityConsumed)
        );
        let mut changed = request.clone();
        changed.request_digest = digest(44);
        assert_eq!(
            ledger.consume(&grant, &changed),
            Err(KernelError::IdempotencyConflict)
        );
    }

    #[test]
    fn simulations_and_producer_assertions_are_not_metered_facts() {
        let mut observation = meter();
        observation.source = ClaimSource::Simulation;
        assert_eq!(
            observation.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::SimulationIsNotMeasurement)
        );
        observation.source = ClaimSource::ProducerAssertion;
        assert_eq!(
            observation.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::InvalidClaim)
        );
    }

    #[test]
    fn meter_interval_calibration_sequence_and_capacity_are_checked() {
        assert_eq!(
            meter().interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Ok(900_000)
        );
        let mut invalid = meter();
        invalid.calibration_valid_until_micros = invalid.interval_end_micros - 1;
        assert_eq!(
            invalid.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::InvalidCalibration)
        );
        let mut invalid = meter();
        invalid.sequence = 1;
        assert_eq!(
            invalid.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::InvalidMeterSequence)
        );
        let mut invalid = meter();
        invalid.cumulative_register_wh = 1_000_001;
        assert_eq!(
            invalid.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::CapacityExceeded)
        );
        let mut invalid = meter();
        invalid.boundary = MeasurementBoundary::GridExport;
        assert_eq!(
            invalid.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::InvalidBoundary)
        );
        let mut invalid = meter();
        invalid.tolerance_bps = 10;
        assert_eq!(
            invalid.interval_energy_wh(MeasurementBoundary::InverterAcOutput, 0),
            Err(KernelError::InvalidTolerance)
        );
    }

    #[test]
    fn meter_sequence_identity_is_consumed_once_across_claims() {
        let observation = meter();
        let mut ledger = MeterSequenceLedger::default();
        assert_eq!(ledger.consume(&observation), Ok(()));
        assert_eq!(
            ledger.consume(&observation),
            Err(KernelError::InvalidMeterSequence)
        );
        let mut next = observation.clone();
        next.sequence += 1;
        assert_eq!(ledger.consume(&next), Ok(()));
    }

    #[test]
    fn verifier_attestation_binds_exact_claim_and_evidence_digests() {
        let claim = ProductionClaim {
            claim_id: "claim-001".into(),
            producer_id: "producer-001".into(),
            claim_digest: digest(9),
            energy_wh: 900_000,
            source: ClaimSource::MeterDerived,
            evidence_digests: vec![digest(1), digest(2)],
        };
        let valid = VerifierAttestation {
            attestation_id: "att-001".into(),
            verifier_id: "verifier-001".into(),
            claim_id: claim.claim_id.clone(),
            claim_digest: claim.claim_digest,
            evidence_digests: claim.evidence_digests.clone(),
            witnesses: vec![
                VerifierWitness {
                    verifier_id: "verifier-001".into(),
                    failure_domains: BTreeSet::from([
                        "legal-entity-a".to_owned(),
                        "meter-admin-a".to_owned(),
                    ]),
                },
                VerifierWitness {
                    verifier_id: "verifier-002".into(),
                    failure_domains: BTreeSet::from([
                        "legal-entity-b".to_owned(),
                        "meter-admin-b".to_owned(),
                    ]),
                },
            ],
            scope: "metered inverter AC output".into(),
            qualified: true,
            conflicts_declared: false,
        };
        assert_eq!(validate_attestation(&claim, &valid), Ok(()));
        let mut tampered = valid.clone();
        tampered.claim_digest = digest(8);
        assert_eq!(
            validate_attestation(&claim, &tampered),
            Err(KernelError::ClaimDigestMismatch)
        );
        let mut wrong_evidence = valid.clone();
        wrong_evidence.evidence_digests = vec![digest(1)];
        assert_eq!(
            validate_attestation(&claim, &wrong_evidence),
            Err(KernelError::EvidenceDigestMismatch)
        );
    }

    #[test]
    fn producer_cannot_verify_and_distinct_ids_do_not_hide_common_causes() {
        let producer_witness = VerifierWitness {
            verifier_id: "producer-001".into(),
            failure_domains: BTreeSet::from(["operator-a".to_owned()]),
        };
        assert_eq!(
            validate_witness_independence("producer-001", &[producer_witness]),
            Err(KernelError::InsufficientWitnesses)
        );
        let mut first_domains = BTreeSet::new();
        first_domains.insert("operator-a".to_owned());
        first_domains.insert("cloud-a".to_owned());
        let mut second_domains = BTreeSet::new();
        second_domains.insert("operator-b".to_owned());
        second_domains.insert("cloud-a".to_owned());
        let witnesses = [
            VerifierWitness {
                verifier_id: "verifier-a".into(),
                failure_domains: first_domains,
            },
            VerifierWitness {
                verifier_id: "verifier-b".into(),
                failure_domains: second_domains,
            },
        ];
        assert_eq!(
            validate_witness_independence("producer-001", &witnesses),
            Err(KernelError::CommonCauseFailureDomain)
        );
    }

    #[test]
    fn two_non_overlapping_witness_failure_domains_are_accepted() {
        let mut first = BTreeSet::new();
        first.insert("legal-entity-a".to_owned());
        first.insert("device-admin-a".to_owned());
        let mut second = BTreeSet::new();
        second.insert("legal-entity-b".to_owned());
        second.insert("device-admin-b".to_owned());
        let witnesses = [
            VerifierWitness {
                verifier_id: "verifier-a".into(),
                failure_domains: first,
            },
            VerifierWitness {
                verifier_id: "verifier-b".into(),
                failure_domains: second,
            },
        ];
        assert_eq!(
            validate_witness_independence("producer-001", &witnesses),
            Ok(())
        );
    }

    #[test]
    fn acceptance_is_bound_to_the_exact_contract_milestone_and_evidence() {
        let acceptance = MilestoneAcceptance {
            obligation_id: "obligation-001".into(),
            contract_digest: digest(3),
            milestone_digest: digest(4),
            evidence_digests: vec![digest(1), digest(2)],
            decision: AcceptanceDecision::Accepted,
            decision_maker: "buyer-board".into(),
            authority_reference: "mandate-001".into(),
        };
        assert_eq!(
            validate_acceptance(
                &acceptance,
                "obligation-001",
                digest(3),
                digest(4),
                &[digest(1), digest(2)]
            ),
            Ok(())
        );
        assert_eq!(
            validate_acceptance(
                &acceptance,
                "obligation-001",
                digest(8),
                digest(4),
                &[digest(1), digest(2)]
            ),
            Err(KernelError::ContractDigestMismatch)
        );
    }

    #[test]
    fn offer_fills_conserve_quantity_and_reject_stale_or_oversized_fills() {
        let mut ledger = OfferLedger::default();
        ledger.create_offer("offer-001", 1_000).unwrap();
        let first = ledger
            .fill("offer-001", 0, 600, "fill-1", digest(1))
            .unwrap();
        assert!(matches!(first, FillDisposition::Applied(_)));
        assert_eq!(
            ledger.fill("offer-001", 0, 500, "fill-2", digest(2)),
            Err(KernelError::OfferRevisionConflict {
                expected: 0,
                actual: 1
            })
        );
        assert_eq!(ledger.offer("offer-001").unwrap().remaining_wh, 400);
        assert_eq!(
            ledger.fill("offer-001", 1, 401, "fill-3", digest(3)),
            Err(KernelError::OfferOverfilled)
        );
    }

    #[test]
    fn exact_offer_retry_returns_receipt_without_consuming_more_quantity() {
        let mut ledger = OfferLedger::default();
        ledger.create_offer("offer-001", 1_000).unwrap();
        let first = ledger
            .fill("offer-001", 0, 400, "fill-1", digest(1))
            .unwrap();
        let replay = ledger
            .fill("offer-001", 0, 400, "fill-1", digest(1))
            .unwrap();
        assert_eq!(first, replay);
        assert_eq!(ledger.offer("offer-001").unwrap().remaining_wh, 600);
        assert_eq!(
            ledger.fill("offer-001", 0, 400, "fill-1", digest(9)),
            Err(KernelError::IdempotencyConflict)
        );
    }

    #[test]
    fn collateral_identity_can_only_be_pledged_once() {
        let mut ledger = CollateralLedger::default();
        ledger.pledge("asset-001", "loan-001").unwrap();
        assert_eq!(
            ledger.pledge("asset-001", "loan-002"),
            Err(KernelError::DuplicateCollateral)
        );
        assert_eq!(ledger.pledged_to("asset-001"), Some("loan-001"));
    }

    #[test]
    fn unknown_payment_outcome_blocks_retry_until_reconciliation() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        assert_eq!(ledger.begin(request.clone()), Ok(BeginPayment::NewIntent));
        ledger
            .reconcile("key-1", RailOutcome::Indeterminate)
            .unwrap();
        assert_eq!(
            ledger.begin(request),
            Ok(BeginPayment::ReconciliationRequired)
        );
        assert_eq!(ledger.side_effect_attempts(), 1);
    }

    #[test]
    fn final_payment_requires_authenticated_exact_receipt_and_replay_is_safe() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        ledger.begin(request.clone()).unwrap();
        let final_status = ledger
            .reconcile(
                "key-1",
                RailOutcome::PaidFinal(receipt(&request, "rail-receipt-1")),
            )
            .unwrap();
        assert!(matches!(final_status, PaymentStatus::PaidFinal { .. }));
        assert_eq!(
            ledger.begin(request),
            Ok(BeginPayment::ExactReplay(final_status))
        );
        assert_eq!(ledger.side_effect_attempts(), 1);
    }

    #[test]
    fn payment_receipt_mismatch_or_missing_authentication_never_becomes_paid() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        ledger.begin(request.clone()).unwrap();
        let mut bad_receipt = receipt(&request, "receipt-1");
        bad_receipt.amount_minor += 1;
        assert_eq!(
            ledger.reconcile("key-1", RailOutcome::PaidFinal(bad_receipt)),
            Err(KernelError::ReceiptMismatch)
        );
        let mut unauthenticated = receipt(&request, "receipt-2");
        unauthenticated.authentication_verified = false;
        assert_eq!(
            ledger.reconcile("key-1", RailOutcome::PaidFinal(unauthenticated)),
            Err(KernelError::UnauthenticatedReceipt)
        );
        assert_eq!(ledger.status("key-1"), Some(&PaymentStatus::Submitted));
    }

    #[test]
    fn manual_attestation_is_not_paid_final() {
        let mut ledger = PaymentLedger::default();
        ledger.begin(payment_request("key-1", 10_000, 1)).unwrap();
        ledger
            .reconcile("key-1", RailOutcome::Indeterminate)
            .unwrap();
        let manual = ledger
            .record_manual_attestation(
                "key-1",
                "observer-a",
                "observer-b",
                "community",
                "installer",
                "manual-evidence-1",
            )
            .unwrap();
        assert!(matches!(manual, PaymentStatus::ManualAttested { .. }));
        assert!(!matches!(manual, PaymentStatus::PaidFinal { .. }));
    }

    #[test]
    fn external_receipt_cannot_settle_two_intents() {
        let mut ledger = PaymentLedger::default();
        let first = payment_request("key-1", 10_000, 1);
        ledger.begin(first.clone()).unwrap();
        ledger
            .reconcile(
                "key-1",
                RailOutcome::PaidFinal(receipt(&first, "shared-receipt")),
            )
            .unwrap();
        let second = payment_request("key-2", 10_000, 2);
        ledger.begin(second.clone()).unwrap();
        assert_eq!(
            ledger.reconcile(
                "key-2",
                RailOutcome::PaidFinal(receipt(&second, "shared-receipt"))
            ),
            Err(KernelError::DuplicateExternalReceipt)
        );
        assert_eq!(ledger.status("key-2"), Some(&PaymentStatus::Submitted));
    }

    #[test]
    fn corpus_scenario_inventory_is_unique_and_linked() {
        const CORPUS: &str = include_str!("../../pcoe-scenarios.v0.1.json");
        let mut ids = BTreeSet::new();
        for item in SCENARIO_COVERAGE {
            assert!(
                ids.insert(item.id),
                "duplicate scenario mapping: {}",
                item.id
            );
            let needle = format!("\"id\": \"{}\"", item.id);
            assert!(
                CORPUS.contains(&needle),
                "scenario missing from corpus: {}",
                item.id
            );
        }
        assert_eq!(ids.len(), 39);
        assert_eq!(CORPUS.matches("\"id\":").count(), 39);
    }

    #[test]
    fn agent_authority_addendum_inventory_is_unique_and_linked() {
        const CORPUS: &str = include_str!("../../pcoe-agent-authority-scenarios.v0.1.json");
        let mut ids = BTreeSet::new();
        for item in AGENT_SCENARIO_COVERAGE {
            assert!(
                ids.insert(item.id),
                "duplicate agent scenario mapping: {}",
                item.id
            );
            let needle = format!("\"id\": \"{}\"", item.id);
            assert!(
                CORPUS.contains(&needle),
                "agent scenario missing from corpus: {}",
                item.id
            );
        }
        assert_eq!(ids.len(), 10);
        assert_eq!(CORPUS.matches("\"id\":").count(), 10);
    }

    #[test]
    fn transparency_metrology_market_inventory_is_unique_and_linked() {
        const CORPUS: &str = include_str!("../../pcoe-transparency-metrology-scenarios.v0.1.json");
        let mut ids = BTreeSet::new();
        for item in TRANSPARENCY_METROLOGY_SCENARIO_COVERAGE {
            assert!(
                ids.insert(item.id),
                "duplicate transparency/metrology scenario mapping: {}",
                item.id
            );
            let needle = format!("\"id\": \"{}\"", item.id);
            assert!(
                CORPUS.contains(&needle),
                "scenario missing from transparency/metrology corpus: {}",
                item.id
            );
        }
        assert_eq!(ids.len(), 10);
        assert_eq!(CORPUS.matches("\"id\":").count(), 10);
    }

    #[test]
    fn coverage_status_does_not_claim_full_corpus_qualification() {
        let implemented = SCENARIO_COVERAGE
            .iter()
            .filter(|item| item.status == CoverageStatus::ImplementedInKernelTests)
            .count();
        let partial = SCENARIO_COVERAGE
            .iter()
            .filter(|item| item.status == CoverageStatus::PartialOrRequiresAdapter)
            .count();
        let outstanding = SCENARIO_COVERAGE
            .iter()
            .filter(|item| item.status == CoverageStatus::NotYetImplemented)
            .count();
        assert_eq!(implemented + partial + outstanding, 39);
        assert!(implemented > 0);
        assert!(outstanding > 0);
    }
}
