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
    DelegationExpansion,
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
    InvalidReversal,
    InvalidPaymentHistory,
    BrokenEventChain,
    UnverifiedEventDigest,
    InvalidManualAttestation,
    DuplicateCollateral,
    DuplicateGeneration,
    OverlappingGenerationInterval,
    IneligibleGenerationEvidence,
    InvalidEacDenomination,
    RegistryProductMismatch,
    EacOverIssuance,
    DuplicateCertificate,
    CertificateNotFound,
    CertificateNotOwned,
    CertificateAlreadyRetired,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorityGrant {
    pub grant_id: String,
    pub parent_grant_id: Option<String>,
    pub delegated_by_actor_id: Option<String>,
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

pub fn validate_delegation(
    parent: &AuthorityGrant,
    child: &AuthorityGrant,
) -> Result<(), KernelError> {
    if parent.grant_id.is_empty()
        || child.grant_id.is_empty()
        || child.grant_id == parent.grant_id
        || child.parent_grant_id.as_deref() != Some(parent.grant_id.as_str())
        || child.delegated_by_actor_id.as_deref() != Some(parent.actor_id.as_str())
        || child.actor_id.is_empty()
        || child.actor_id == parent.actor_id
    {
        return Err(KernelError::DelegationExpansion);
    }
    if child.principal_id != parent.principal_id
        || child.task_id != parent.task_id
        || child.audience_id != parent.audience_id
        || child.action_class != parent.action_class
        || child.resource_id != parent.resource_id
        || child.max_amount == 0
        || child.max_amount > parent.max_amount
        || child.not_before < parent.not_before
        || child.expires_at > parent.expires_at
        || child.not_before >= child.expires_at
        || child.revocation_epoch != parent.revocation_epoch
        || (parent.requires_human_disposition && !child.requires_human_disposition)
        || (parent.one_time && !child.one_time)
        || child.max_human_approval_age_secs > parent.max_human_approval_age_secs
        || !child
            .human_approver_ids
            .is_subset(&parent.human_approver_ids)
    {
        return Err(KernelError::DelegationExpansion);
    }
    Ok(())
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
    /// Checks policy before consuming a grant. A production adapter must make
    /// authorization, grant consumption and the side effect one atomic operation.
    pub fn authorize_and_consume(
        &mut self,
        grant: &AuthorityGrant,
        request: &ActionRequest,
        now: u64,
        observed_revocation_epoch: u64,
        human_disposition: Option<&HumanDisposition>,
    ) -> Result<(), KernelError> {
        authorize(
            grant,
            request,
            now,
            observed_revocation_epoch,
            human_disposition,
        )?;
        self.consume(grant, request)
    }

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
pub struct ReversalReceipt {
    pub external_reversal_id: String,
    pub original_payment_receipt_id: String,
    pub obligation_id: String,
    pub payer_id: String,
    pub payee_id: String,
    pub amount_minor: u128,
    pub currency: String,
    pub request_digest: Digest,
    pub reason_reference: String,
    /// Test seam set only after upstream rail-authentication verification.
    pub authentication_verified: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PaymentAuditEvent {
    IntentSubmitted {
        request_digest: Digest,
    },
    OutcomeIndeterminate,
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
    ReversalRecorded {
        original_receipt_id: String,
        reversal_receipt_id: String,
        reason_reference: String,
    },
}

/// Reconstructs a payment disposition from the original request and ordered
/// append-only events. It does not trust the producer's stored aggregate status.
pub fn replay_payment_history(
    request: &PaymentRequest,
    events: &[PaymentAuditEvent],
) -> Result<PaymentStatus, KernelError> {
    if request.obligation_id.is_empty()
        || request.payer_id.is_empty()
        || request.payee_id.is_empty()
        || request.idempotency_key.is_empty()
        || request.amount_minor == 0
        || request.payer_id == request.payee_id
        || events.is_empty()
    {
        return Err(KernelError::InvalidPaymentHistory);
    }

    let mut status: Option<PaymentStatus> = None;
    for (index, event) in events.iter().enumerate() {
        match event {
            PaymentAuditEvent::IntentSubmitted { request_digest } => {
                if index != 0 || status.is_some() || request_digest != &request.request_digest {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::Submitted);
            }
            PaymentAuditEvent::OutcomeIndeterminate => {
                if !matches!(
                    status.as_ref(),
                    Some(PaymentStatus::Submitted | PaymentStatus::Indeterminate)
                ) {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::Indeterminate);
            }
            PaymentAuditEvent::PaidFinal {
                external_receipt_id,
                finality_policy_reference,
            } => {
                if !matches!(
                    status.as_ref(),
                    Some(PaymentStatus::Submitted | PaymentStatus::Indeterminate)
                ) || external_receipt_id.is_empty()
                    || finality_policy_reference.is_empty()
                {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::PaidFinal {
                    external_receipt_id: external_receipt_id.clone(),
                    finality_policy_reference: finality_policy_reference.clone(),
                });
            }
            PaymentAuditEvent::Rejected {
                external_receipt_id,
            } => {
                if !matches!(
                    status.as_ref(),
                    Some(PaymentStatus::Submitted | PaymentStatus::Indeterminate)
                ) || external_receipt_id.is_empty()
                {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::Rejected {
                    external_receipt_id: external_receipt_id.clone(),
                });
            }
            PaymentAuditEvent::ManualAttested {
                attestor_a,
                attestor_b,
                evidence_reference,
            } => {
                if status != Some(PaymentStatus::Indeterminate)
                    || attestor_a.is_empty()
                    || attestor_b.is_empty()
                    || attestor_a == attestor_b
                    || attestor_a == &request.payer_id
                    || attestor_a == &request.payee_id
                    || attestor_b == &request.payer_id
                    || attestor_b == &request.payee_id
                    || evidence_reference.is_empty()
                {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::ManualAttested {
                    attestor_a: attestor_a.clone(),
                    attestor_b: attestor_b.clone(),
                    evidence_reference: evidence_reference.clone(),
                });
            }
            PaymentAuditEvent::ReversalRecorded {
                original_receipt_id,
                reversal_receipt_id,
                reason_reference,
            } => {
                let expected_original = match &status {
                    Some(PaymentStatus::PaidFinal {
                        external_receipt_id,
                        ..
                    }) => external_receipt_id,
                    _ => return Err(KernelError::InvalidPaymentHistory),
                };
                if original_receipt_id != expected_original
                    || reversal_receipt_id.is_empty()
                    || reversal_receipt_id == original_receipt_id
                    || reason_reference.is_empty()
                {
                    return Err(KernelError::InvalidPaymentHistory);
                }
                status = Some(PaymentStatus::Reversed {
                    original_receipt_id: original_receipt_id.clone(),
                    reversal_receipt_id: reversal_receipt_id.clone(),
                    reason_reference: reason_reference.clone(),
                });
            }
        }
    }
    status.ok_or(KernelError::InvalidPaymentHistory)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PaymentEventEnvelope {
    pub sequence: u64,
    pub previous_event_digest: Option<Digest>,
    pub event_digest: Digest,
    pub event: PaymentAuditEvent,
    /// Test seam set only after an upstream canonicalization/signature verifier
    /// authenticates the event payload and its issuer.
    pub digest_and_signature_verified: bool,
}

/// Validates a linear event chain and then reconstructs the payment disposition.
/// The final digest must be anchored by an independent checkpoint, e.g. a
/// transparency receipt or separately controlled witness. This function does
/// not compute cryptographic hashes or authenticate signatures itself.
pub fn replay_payment_event_chain(
    request: &PaymentRequest,
    envelopes: &[PaymentEventEnvelope],
    independently_anchored_final_digest: Digest,
) -> Result<PaymentStatus, KernelError> {
    if envelopes.is_empty() || independently_anchored_final_digest == [0; 32] {
        return Err(KernelError::BrokenEventChain);
    }
    let mut seen_digests = BTreeSet::new();
    let mut payload_events = Vec::with_capacity(envelopes.len());
    for (index, envelope) in envelopes.iter().enumerate() {
        let expected_sequence = (index as u64)
            .checked_add(1)
            .ok_or(KernelError::ArithmeticOverflow)?;
        if envelope.sequence != expected_sequence || envelope.event_digest == [0; 32] {
            return Err(KernelError::BrokenEventChain);
        }
        if !envelope.digest_and_signature_verified {
            return Err(KernelError::UnverifiedEventDigest);
        }
        let expected_previous = if index == 0 {
            None
        } else {
            Some(envelopes[index - 1].event_digest)
        };
        if envelope.previous_event_digest != expected_previous {
            return Err(KernelError::BrokenEventChain);
        }
        if !seen_digests.insert(envelope.event_digest) {
            return Err(KernelError::BrokenEventChain);
        }
        payload_events.push(envelope.event.clone());
    }
    if envelopes.last().map(|envelope| envelope.event_digest)
        != Some(independently_anchored_final_digest)
    {
        return Err(KernelError::BrokenEventChain);
    }
    replay_payment_history(request, &payload_events)
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
    Reversed {
        original_receipt_id: String,
        reversal_receipt_id: String,
        reason_reference: String,
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
    events: BTreeMap<String, Vec<PaymentAuditEvent>>,
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
        let idempotency_key = request.idempotency_key.clone();
        let request_digest = request.request_digest;
        self.intents
            .insert(idempotency_key.clone(), (request, PaymentStatus::Submitted));
        self.events
            .entry(idempotency_key)
            .or_default()
            .push(PaymentAuditEvent::IntentSubmitted { request_digest });
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
                let current = status.clone();
                self.events
                    .entry(idempotency_key.to_owned())
                    .or_default()
                    .push(PaymentAuditEvent::OutcomeIndeterminate);
                Ok(current)
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
        let audit_event = match &status {
            PaymentStatus::PaidFinal {
                external_receipt_id,
                finality_policy_reference,
            } => PaymentAuditEvent::PaidFinal {
                external_receipt_id: external_receipt_id.clone(),
                finality_policy_reference: finality_policy_reference.clone(),
            },
            PaymentStatus::Rejected {
                external_receipt_id,
            } => PaymentAuditEvent::Rejected {
                external_receipt_id: external_receipt_id.clone(),
            },
            _ => return Err(KernelError::InvalidPaymentTransition),
        };
        self.events
            .entry(idempotency_key.to_owned())
            .or_default()
            .push(audit_event);
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
        let current = status.clone();
        self.events
            .entry(idempotency_key.to_owned())
            .or_default()
            .push(PaymentAuditEvent::ManualAttested {
                attestor_a: attestor_a.to_owned(),
                attestor_b: attestor_b.to_owned(),
                evidence_reference: evidence_reference.to_owned(),
            });
        Ok(current)
    }

    /// Adds a fully authenticated reversal event while retaining the original
    /// PaidFinal event. Partial reversals require a rail-specific policy and
    /// are intentionally outside this reference slice.
    pub fn record_reversal(
        &mut self,
        idempotency_key: &str,
        reversal: ReversalReceipt,
    ) -> Result<PaymentStatus, KernelError> {
        let (request, current_status) = self
            .intents
            .get(idempotency_key)
            .ok_or(KernelError::InvalidReversal)?;
        let original_receipt_id = match current_status {
            PaymentStatus::PaidFinal {
                external_receipt_id,
                ..
            } => external_receipt_id.clone(),
            _ => return Err(KernelError::InvalidReversal),
        };
        if !reversal.authentication_verified {
            return Err(KernelError::UnauthenticatedReceipt);
        }
        if reversal.external_reversal_id.is_empty()
            || reversal.original_payment_receipt_id.is_empty()
            || reversal.reason_reference.is_empty()
        {
            return Err(KernelError::MissingReference);
        }
        if reversal.original_payment_receipt_id != original_receipt_id
            || reversal.obligation_id != request.obligation_id
            || reversal.payer_id != request.payer_id
            || reversal.payee_id != request.payee_id
            || reversal.amount_minor != request.amount_minor
            || reversal.currency != request.currency
            || reversal.request_digest != request.request_digest
        {
            return Err(KernelError::ReceiptMismatch);
        }
        if self
            .consumed_receipts
            .contains(&reversal.external_reversal_id)
        {
            return Err(KernelError::DuplicateExternalReceipt);
        }
        self.consumed_receipts
            .insert(reversal.external_reversal_id.clone());
        let status = PaymentStatus::Reversed {
            original_receipt_id: original_receipt_id.clone(),
            reversal_receipt_id: reversal.external_reversal_id.clone(),
            reason_reference: reversal.reason_reference.clone(),
        };
        self.intents
            .get_mut(idempotency_key)
            .ok_or(KernelError::InvalidReversal)?
            .1 = status.clone();
        self.events
            .entry(idempotency_key.to_owned())
            .or_default()
            .push(PaymentAuditEvent::ReversalRecorded {
                original_receipt_id,
                reversal_receipt_id: reversal.external_reversal_id,
                reason_reference: reversal.reason_reference,
            });
        Ok(status)
    }

    pub fn events_for(&self, idempotency_key: &str) -> Option<&[PaymentAuditEvent]> {
        self.events.get(idempotency_key).map(Vec::as_slice)
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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedGeneration {
    pub generation_id: String,
    pub registry_id: String,
    pub product_code_reference: String,
    pub installation_id: String,
    pub producer_id: String,
    pub meter_id: String,
    pub interval_start_micros: i64,
    pub interval_end_micros: i64,
    pub eligible_energy_wh: u128,
    pub boundary: MeasurementBoundary,
    pub source: ClaimSource,
    pub evidence_digest: Digest,
    pub verifier_attestation_digest: Digest,
    pub verifier_id: String,
    /// Upstream product-code, verifier and evidence validation test seam.
    pub independently_qualified: bool,
    pub conflicts_declared: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EacState {
    Held,
    Retired {
        claimant_id: String,
        retirement_reference: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnergyAttributeCertificate {
    pub certificate_id: String,
    pub registry_id: String,
    pub product_code_reference: String,
    pub generation_id: String,
    pub quantity_wh: u128,
    pub denomination_wh: u128,
    pub owner_id: String,
    pub state: EacState,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct GenerationIssuance {
    generation: VerifiedGeneration,
    issued_wh: u128,
}

#[derive(Debug, Clone, Default)]
pub struct EnergyAttributeLedger {
    generations: BTreeMap<String, GenerationIssuance>,
    certificates: BTreeMap<String, EnergyAttributeCertificate>,
}

impl EnergyAttributeLedger {
    /// Registers one generation interval after an upstream verifier and the
    /// applicable product code establish eligibility at the exact boundary.
    /// The in-memory ledger prevents duplicate/overlapping local issuance only;
    /// accredited registries remain authoritative for legal certificate status.
    pub fn register_generation(
        &mut self,
        generation: VerifiedGeneration,
        expected_product_boundary: MeasurementBoundary,
    ) -> Result<(), KernelError> {
        if generation.generation_id.is_empty()
            || generation.registry_id.is_empty()
            || generation.product_code_reference.is_empty()
            || generation.installation_id.is_empty()
            || generation.producer_id.is_empty()
            || generation.meter_id.is_empty()
            || generation.verifier_id.is_empty()
            || generation.evidence_digest == [0; 32]
            || generation.verifier_attestation_digest == [0; 32]
            || generation.eligible_energy_wh == 0
        {
            return Err(KernelError::MissingReference);
        }
        if generation.source != ClaimSource::MeterDerived
            || generation.verifier_id == generation.producer_id
            || !generation.independently_qualified
            || generation.conflicts_declared
        {
            return Err(KernelError::IneligibleGenerationEvidence);
        }
        if generation.boundary != expected_product_boundary {
            return Err(KernelError::InvalidBoundary);
        }
        if generation.interval_start_micros < 0
            || generation.interval_end_micros <= generation.interval_start_micros
        {
            return Err(KernelError::InvalidInterval);
        }
        if self.generations.contains_key(&generation.generation_id) {
            return Err(KernelError::DuplicateGeneration);
        }
        let overlaps = self.generations.values().any(|existing| {
            existing.generation.meter_id == generation.meter_id
                && existing.generation.boundary == generation.boundary
                && generation.interval_start_micros < existing.generation.interval_end_micros
                && existing.generation.interval_start_micros < generation.interval_end_micros
        });
        if overlaps {
            return Err(KernelError::OverlappingGenerationInterval);
        }
        self.generations.insert(
            generation.generation_id.clone(),
            GenerationIssuance {
                generation,
                issued_wh: 0,
            },
        );
        Ok(())
    }

    /// Issues EAC units only against registered eligible generation. Denomination
    /// and registry/product-code identity are explicit and versioned by the caller.
    pub fn issue(
        &mut self,
        certificate_id: &str,
        registry_id: &str,
        product_code_reference: &str,
        generation_id: &str,
        quantity_wh: u128,
        denomination_wh: u128,
        owner_id: &str,
    ) -> Result<EnergyAttributeCertificate, KernelError> {
        if certificate_id.is_empty()
            || registry_id.is_empty()
            || product_code_reference.is_empty()
            || generation_id.is_empty()
            || owner_id.is_empty()
        {
            return Err(KernelError::MissingReference);
        }
        if self.certificates.contains_key(certificate_id) {
            return Err(KernelError::DuplicateCertificate);
        }
        if denomination_wh == 0 || quantity_wh == 0 || quantity_wh % denomination_wh != 0 {
            return Err(KernelError::InvalidEacDenomination);
        }
        let issuance = self
            .generations
            .get_mut(generation_id)
            .ok_or(KernelError::InvalidClaim)?;
        if issuance.generation.registry_id != registry_id
            || issuance.generation.product_code_reference != product_code_reference
        {
            return Err(KernelError::RegistryProductMismatch);
        }
        let issued_after = issuance
            .issued_wh
            .checked_add(quantity_wh)
            .ok_or(KernelError::ArithmeticOverflow)?;
        if issued_after > issuance.generation.eligible_energy_wh {
            return Err(KernelError::EacOverIssuance);
        }
        let certificate = EnergyAttributeCertificate {
            certificate_id: certificate_id.to_owned(),
            registry_id: registry_id.to_owned(),
            product_code_reference: product_code_reference.to_owned(),
            generation_id: generation_id.to_owned(),
            quantity_wh,
            denomination_wh,
            owner_id: owner_id.to_owned(),
            state: EacState::Held,
        };
        issuance.issued_wh = issued_after;
        self.certificates
            .insert(certificate_id.to_owned(), certificate.clone());
        Ok(certificate)
    }

    /// Changes certificate ownership in this local projection. Production
    /// integrations must reconcile the authoritative accredited registry receipt.
    pub fn transfer(
        &mut self,
        certificate_id: &str,
        current_owner_id: &str,
        new_owner_id: &str,
    ) -> Result<EnergyAttributeCertificate, KernelError> {
        let certificate = self
            .certificates
            .get_mut(certificate_id)
            .ok_or(KernelError::CertificateNotFound)?;
        if certificate.owner_id != current_owner_id || new_owner_id.is_empty() {
            return Err(KernelError::CertificateNotOwned);
        }
        if certificate.state != EacState::Held {
            return Err(KernelError::CertificateAlreadyRetired);
        }
        certificate.owner_id = new_owner_id.to_owned();
        Ok(certificate.clone())
    }

    /// Retirement is terminal in the reference projection: the attribute unit
    /// cannot be transferred or retired again for another claim.
    pub fn retire(
        &mut self,
        certificate_id: &str,
        owner_id: &str,
        claimant_id: &str,
        retirement_reference: &str,
    ) -> Result<EnergyAttributeCertificate, KernelError> {
        let certificate = self
            .certificates
            .get_mut(certificate_id)
            .ok_or(KernelError::CertificateNotFound)?;
        if certificate.owner_id != owner_id
            || claimant_id.is_empty()
            || retirement_reference.is_empty()
        {
            return Err(KernelError::CertificateNotOwned);
        }
        if certificate.state != EacState::Held {
            return Err(KernelError::CertificateAlreadyRetired);
        }
        certificate.state = EacState::Retired {
            claimant_id: claimant_id.to_owned(),
            retirement_reference: retirement_reference.to_owned(),
        };
        Ok(certificate.clone())
    }

    pub fn certificate(&self, certificate_id: &str) -> Option<&EnergyAttributeCertificate> {
        self.certificates.get(certificate_id)
    }

    pub fn issued_wh(&self, generation_id: &str) -> Option<u128> {
        self.generations
            .get(generation_id)
            .map(|issuance| issuance.issued_wh)
    }

    pub fn remaining_eligible_wh(&self, generation_id: &str) -> Option<u128> {
        self.generations.get(generation_id).and_then(|issuance| {
            issuance
                .generation
                .eligible_energy_wh
                .checked_sub(issuance.issued_wh)
        })
    }
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
        status: CoverageStatus::PartialOrRequiresAdapter,
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
        status: CoverageStatus::ImplementedInKernelTests,
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

pub const ENERGY_ATTRIBUTE_SCENARIO_COVERAGE: [ScenarioCoverage; 12] = [
    ScenarioCoverage {
        id: "EAC-001",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EAC-002",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EAC-003",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EAC-004",
        status: CoverageStatus::ImplementedInKernelTests,
    },
    ScenarioCoverage {
        id: "EAC-005",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "EAC-006",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "EAC-007",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "EAC-008",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "EAC-009",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "EAC-010",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
    ScenarioCoverage {
        id: "EAC-011",
        status: CoverageStatus::NotYetImplemented,
    },
    ScenarioCoverage {
        id: "EAC-012",
        status: CoverageStatus::PartialOrRequiresAdapter,
    },
];

pub const DPP_SCENARIO_COVERAGE: [ScenarioCoverage; 12] = [
    ScenarioCoverage { id: "DPP-001", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-002", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-003", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-004", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-005", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-006", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-007", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-008", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-009", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-010", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-011", status: CoverageStatus::NotYetImplemented },
    ScenarioCoverage { id: "DPP-012", status: CoverageStatus::NotYetImplemented },
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
            parent_grant_id: None,
            delegated_by_actor_id: None,
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
    fn delegation_is_attenuating_across_scope_amount_time_and_controls() {
        let parent = grant();
        let mut child = parent.clone();
        child.grant_id = "grant-child".into();
        child.parent_grant_id = Some(parent.grant_id.clone());
        child.delegated_by_actor_id = Some(parent.actor_id.clone());
        child.actor_id = "sub-agent".into();
        child.max_amount = 400_000;
        child.expires_at = 400;
        child.not_before = 150;
        assert_eq!(validate_delegation(&parent, &child), Ok(()));

        let mut expanded = child.clone();
        expanded.max_amount = parent.max_amount + 1;
        assert_eq!(
            validate_delegation(&parent, &expanded),
            Err(KernelError::DelegationExpansion)
        );

        let mut longer_lived = child.clone();
        longer_lived.expires_at = parent.expires_at + 1;
        assert_eq!(
            validate_delegation(&parent, &longer_lived),
            Err(KernelError::DelegationExpansion)
        );

        let mut different_task = child.clone();
        different_task.task_id = "other-task".into();
        assert_eq!(
            validate_delegation(&parent, &different_task),
            Err(KernelError::DelegationExpansion)
        );

        let mut wrong_delegator = child.clone();
        wrong_delegator.delegated_by_actor_id = Some("other-agent".into());
        assert_eq!(
            validate_delegation(&parent, &wrong_delegator),
            Err(KernelError::DelegationExpansion)
        );

        let mut missing_parent = child;
        missing_parent.parent_grant_id = None;
        assert_eq!(
            validate_delegation(&parent, &missing_parent),
            Err(KernelError::DelegationExpansion)
        );
    }

    #[test]
    fn rejected_authorization_does_not_consume_one_time_grant() {
        let grant = grant();
        let mut oversized = request();
        oversized.amount = grant.max_amount + 1;
        let mut ledger = AuthorityConsumptionLedger::default();
        assert_eq!(
            ledger.authorize_and_consume(&grant, &oversized, 200, 7, None),
            Err(KernelError::ScopeExceeded)
        );
        assert_eq!(
            ledger.authorize_and_consume(&grant, &request(), 200, 7, None),
            Ok(())
        );
        assert_eq!(
            ledger.authorize_and_consume(&grant, &request(), 200, 7, None),
            Err(KernelError::CapabilityConsumed)
        );
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
        oversized.amount = grant().max_amount + 1;
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
        let first_receipt = match first {
            FillDisposition::Applied(receipt) => receipt,
            other => panic!("first fill should apply, got {other:?}"),
        };
        let replay_receipt = match replay {
            FillDisposition::ExactReplay(receipt) => receipt,
            other => panic!("retry should return the prior receipt, got {other:?}"),
        };
        assert_eq!(first_receipt, replay_receipt);
        assert_eq!(ledger.offer("offer-001").unwrap().remaining_wh, 600);
        assert_eq!(
            ledger.fill("offer-001", 0, 400, "fill-1", digest(9)),
            Err(KernelError::IdempotencyConflict)
        );
        assert_eq!(
            ledger.fill("offer-001", 1, 400, "fill-1", digest(1)),
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
    fn post_finality_reversal_appends_compensating_event_without_erasing_paid_receipt() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        ledger.begin(request.clone()).unwrap();
        let paid = ledger
            .reconcile(
                "key-1",
                RailOutcome::PaidFinal(receipt(&request, "rail-payment-1")),
            )
            .unwrap();
        assert!(matches!(paid, PaymentStatus::PaidFinal { .. }));
        let reversed = ledger
            .record_reversal(
                "key-1",
                ReversalReceipt {
                    external_reversal_id: "rail-reversal-1".into(),
                    original_payment_receipt_id: "rail-payment-1".into(),
                    obligation_id: request.obligation_id.clone(),
                    payer_id: request.payer_id.clone(),
                    payee_id: request.payee_id.clone(),
                    amount_minor: request.amount_minor,
                    currency: request.currency.clone(),
                    request_digest: request.request_digest,
                    reason_reference: "rail-chargeback-1".into(),
                    authentication_verified: true,
                },
            )
            .unwrap();
        assert!(matches!(reversed, PaymentStatus::Reversed { .. }));
        let events = ledger.events_for("key-1").unwrap();
        assert_eq!(events.len(), 3);
        assert!(matches!(
            events[0],
            PaymentAuditEvent::IntentSubmitted { .. }
        ));
        assert!(matches!(events[1], PaymentAuditEvent::PaidFinal { .. }));
        assert!(matches!(
            events[2],
            PaymentAuditEvent::ReversalRecorded { .. }
        ));
        assert_eq!(
            ledger.begin(request),
            Ok(BeginPayment::ExactReplay(reversed))
        );
    }

    #[test]
    fn independent_replay_reconstructs_paid_then_reversed_payment_history() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        ledger.begin(request.clone()).unwrap();
        ledger
            .reconcile(
                "key-1",
                RailOutcome::PaidFinal(receipt(&request, "rail-payment-1")),
            )
            .unwrap();
        ledger
            .record_reversal(
                "key-1",
                ReversalReceipt {
                    external_reversal_id: "rail-reversal-1".into(),
                    original_payment_receipt_id: "rail-payment-1".into(),
                    obligation_id: request.obligation_id.clone(),
                    payer_id: request.payer_id.clone(),
                    payee_id: request.payee_id.clone(),
                    amount_minor: request.amount_minor,
                    currency: request.currency.clone(),
                    request_digest: request.request_digest,
                    reason_reference: "chargeback-reason-1".into(),
                    authentication_verified: true,
                },
            )
            .unwrap();
        let reconstructed =
            replay_payment_history(&request, ledger.events_for("key-1").unwrap()).unwrap();
        assert_eq!(Some(&reconstructed), ledger.status("key-1"));
    }

    #[test]
    fn anchored_payment_event_chain_replays_and_detects_tampering_or_truncation() {
        let request = payment_request("key-1", 10_000, 1);
        let envelopes = vec![
            PaymentEventEnvelope {
                sequence: 1,
                previous_event_digest: None,
                event_digest: digest(10),
                event: PaymentAuditEvent::IntentSubmitted {
                    request_digest: request.request_digest,
                },
                digest_and_signature_verified: true,
            },
            PaymentEventEnvelope {
                sequence: 2,
                previous_event_digest: Some(digest(10)),
                event_digest: digest(11),
                event: PaymentAuditEvent::PaidFinal {
                    external_receipt_id: "payment-1".into(),
                    finality_policy_reference: "rail-finality/v1".into(),
                },
                digest_and_signature_verified: true,
            },
            PaymentEventEnvelope {
                sequence: 3,
                previous_event_digest: Some(digest(11)),
                event_digest: digest(12),
                event: PaymentAuditEvent::ReversalRecorded {
                    original_receipt_id: "payment-1".into(),
                    reversal_receipt_id: "reversal-1".into(),
                    reason_reference: "chargeback-1".into(),
                },
                digest_and_signature_verified: true,
            },
        ];
        let replayed = replay_payment_event_chain(&request, &envelopes, digest(12)).unwrap();
        assert!(matches!(replayed, PaymentStatus::Reversed { .. }));

        let mut broken_link = envelopes.clone();
        broken_link[2].previous_event_digest = Some(digest(99));
        assert_eq!(
            replay_payment_event_chain(&request, &broken_link, digest(12)),
            Err(KernelError::BrokenEventChain)
        );

        let mut unverified = envelopes.clone();
        unverified[1].digest_and_signature_verified = false;
        assert_eq!(
            replay_payment_event_chain(&request, &unverified, digest(12)),
            Err(KernelError::UnverifiedEventDigest)
        );

        let mut duplicate_digest = envelopes.clone();
        duplicate_digest[2].event_digest = digest(11);
        assert_eq!(
            replay_payment_event_chain(&request, &duplicate_digest, digest(11)),
            Err(KernelError::BrokenEventChain)
        );

        assert_eq!(
            replay_payment_event_chain(&request, &envelopes[..2], digest(12)),
            Err(KernelError::BrokenEventChain)
        );
    }

    #[test]
    fn independent_replay_rejects_impossible_or_mutated_payment_histories() {
        let request = payment_request("key-1", 10_000, 1);
        let missing_start = vec![PaymentAuditEvent::PaidFinal {
            external_receipt_id: "receipt-1".into(),
            finality_policy_reference: "rail-finality/v1".into(),
        }];
        assert_eq!(
            replay_payment_history(&request, &missing_start),
            Err(KernelError::InvalidPaymentHistory)
        );
        let wrong_digest = vec![PaymentAuditEvent::IntentSubmitted {
            request_digest: digest(99),
        }];
        assert_eq!(
            replay_payment_history(&request, &wrong_digest),
            Err(KernelError::InvalidPaymentHistory)
        );
        let reversal_before_payment = vec![
            PaymentAuditEvent::IntentSubmitted {
                request_digest: request.request_digest,
            },
            PaymentAuditEvent::OutcomeIndeterminate,
            PaymentAuditEvent::ReversalRecorded {
                original_receipt_id: "payment-1".into(),
                reversal_receipt_id: "reversal-1".into(),
                reason_reference: "reason-1".into(),
            },
        ];
        assert_eq!(
            replay_payment_history(&request, &reversal_before_payment),
            Err(KernelError::InvalidPaymentHistory)
        );
        let manual_promoted_to_paid = vec![
            PaymentAuditEvent::IntentSubmitted {
                request_digest: request.request_digest,
            },
            PaymentAuditEvent::OutcomeIndeterminate,
            PaymentAuditEvent::ManualAttested {
                attestor_a: "observer-a".into(),
                attestor_b: "observer-b".into(),
                evidence_reference: "manual-evidence".into(),
            },
            PaymentAuditEvent::PaidFinal {
                external_receipt_id: "payment-1".into(),
                finality_policy_reference: "rail-finality/v1".into(),
            },
        ];
        assert_eq!(
            replay_payment_history(&request, &manual_promoted_to_paid),
            Err(KernelError::InvalidPaymentHistory)
        );
    }

    #[test]
    fn reversal_requires_exact_authenticated_final_receipt_and_cannot_repeat() {
        let mut ledger = PaymentLedger::default();
        let request = payment_request("key-1", 10_000, 1);
        ledger.begin(request.clone()).unwrap();
        ledger
            .reconcile(
                "key-1",
                RailOutcome::PaidFinal(receipt(&request, "rail-payment-1")),
            )
            .unwrap();
        let mut reversal = ReversalReceipt {
            external_reversal_id: "rail-reversal-1".into(),
            original_payment_receipt_id: "rail-payment-1".into(),
            obligation_id: request.obligation_id.clone(),
            payer_id: request.payer_id.clone(),
            payee_id: request.payee_id.clone(),
            amount_minor: request.amount_minor + 1,
            currency: request.currency.clone(),
            request_digest: request.request_digest,
            reason_reference: "rail-chargeback-1".into(),
            authentication_verified: true,
        };
        assert_eq!(
            ledger.record_reversal("key-1", reversal.clone()),
            Err(KernelError::ReceiptMismatch)
        );
        reversal.authentication_verified = false;
        reversal.amount_minor = request.amount_minor;
        assert_eq!(
            ledger.record_reversal("key-1", reversal.clone()),
            Err(KernelError::UnauthenticatedReceipt)
        );
        reversal.authentication_verified = true;
        ledger.record_reversal("key-1", reversal.clone()).unwrap();
        assert_eq!(
            ledger.record_reversal("key-1", reversal),
            Err(KernelError::InvalidReversal)
        );
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

    fn verified_generation() -> VerifiedGeneration {
        VerifiedGeneration {
            generation_id: "generation-001".into(),
            registry_id: "accredited-registry-1".into(),
            product_code_reference: "irec-e-product-code/v1".into(),
            installation_id: "site-001".into(),
            producer_id: "producer-001".into(),
            meter_id: "meter-001".into(),
            interval_start_micros: 1_000,
            interval_end_micros: 3_601_000,
            eligible_energy_wh: 2_500_000,
            boundary: MeasurementBoundary::InverterAcOutput,
            source: ClaimSource::MeterDerived,
            evidence_digest: digest(11),
            verifier_attestation_digest: digest(12),
            verifier_id: "independent-verifier".into(),
            independently_qualified: true,
            conflicts_declared: false,
        }
    }

    #[test]
    fn eac_issuance_is_denominated_and_cannot_exceed_verified_generation() {
        let mut ledger = EnergyAttributeLedger::default();
        ledger
            .register_generation(verified_generation(), MeasurementBoundary::InverterAcOutput)
            .unwrap();
        let issued = ledger
            .issue(
                "eac-001",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                2_000_000,
                1_000_000,
                "producer-account",
            )
            .unwrap();
        assert_eq!(issued.quantity_wh, 2_000_000);
        assert_eq!(
            ledger.remaining_eligible_wh("generation-001"),
            Some(500_000)
        );
        assert_eq!(
            ledger.issue(
                "eac-002",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                1_000_000,
                1_000_000,
                "producer-account",
            ),
            Err(KernelError::EacOverIssuance)
        );
        assert_eq!(
            ledger.issue(
                "eac-003",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                500_000,
                1_000_000,
                "producer-account",
            ),
            Err(KernelError::InvalidEacDenomination)
        );
        assert_eq!(
            ledger.issue(
                "eac-004",
                "different-registry",
                "irec-e-product-code/v1",
                "generation-001",
                1_000_000,
                1_000_000,
                "producer-account",
            ),
            Err(KernelError::RegistryProductMismatch)
        );
    }

    #[test]
    fn eac_ownership_transfer_and_retirement_are_single_use() {
        let mut ledger = EnergyAttributeLedger::default();
        ledger
            .register_generation(verified_generation(), MeasurementBoundary::InverterAcOutput)
            .unwrap();
        ledger
            .issue(
                "eac-001",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                1_000_000,
                1_000_000,
                "producer-account",
            )
            .unwrap();
        ledger
            .transfer("eac-001", "producer-account", "buyer-account")
            .unwrap();
        let retired = ledger
            .retire(
                "eac-001",
                "buyer-account",
                "buyer-claim-001",
                "registry-retirement-001",
            )
            .unwrap();
        assert_eq!(
            retired.state,
            EacState::Retired {
                claimant_id: "buyer-claim-001".into(),
                retirement_reference: "registry-retirement-001".into(),
            }
        );
        assert_eq!(
            ledger.transfer("eac-001", "buyer-account", "second-buyer"),
            Err(KernelError::CertificateAlreadyRetired)
        );
        assert_eq!(
            ledger.retire("eac-001", "buyer-account", "second-claim", "retirement-2"),
            Err(KernelError::CertificateAlreadyRetired)
        );
    }

    #[test]
    fn capacity_tolerance_floor_is_conservative_and_monotonic_for_one_million_cases() {
        const MICROS_PER_HOUR: u128 = 3_600_000_000;
        const BASIS_POINTS: u128 = 10_000;
        for sample in 0u128..1_000_000 {
            let rated_capacity_w = sample % 2_000_000 + 1;
            let duration_micros = (sample * 97 + 29) % 86_400_000_000 + 1;
            let tolerance_bps = sample % 1_001;
            let raw_capacity_time = rated_capacity_w
                .checked_mul(duration_micros)
                .expect("bounded property inputs");
            let theoretical_wh = raw_capacity_time / MICROS_PER_HOUR;
            let tolerance_multiplier = BASIS_POINTS + tolerance_bps;
            let tolerated_wh = theoretical_wh
                .checked_mul(tolerance_multiplier)
                .expect("bounded tolerance product")
                / BASIS_POINTS;
            let conservative_scaled = tolerated_wh
                .checked_mul(MICROS_PER_HOUR)
                .and_then(|value| value.checked_mul(BASIS_POINTS))
                .expect("bounded conservative product");
            let exact_scaled_bound = raw_capacity_time
                .checked_mul(tolerance_multiplier)
                .expect("bounded exact product");
            assert!(
                conservative_scaled <= exact_scaled_bound,
                "rounding must not inflate physical capacity at sample {sample}"
            );
            let next_tolerance_wh = theoretical_wh
                .checked_mul(tolerance_multiplier + 1)
                .expect("bounded next tolerance product")
                / BASIS_POINTS;
            assert!(
                next_tolerance_wh >= tolerated_wh,
                "increasing tolerance cannot lower the integer capacity bound at sample {sample}"
            );
        }
    }

    #[test]
    fn offer_ledger_conserves_quantity_under_deterministic_adversarial_schedules() {
        const INITIAL_WH: u128 = 1_000_000;
        for seed in 0u64..256 {
            let mut ledger = OfferLedger::default();
            ledger.create_offer("property-offer", INITIAL_WH).unwrap();
            let mut random = seed.wrapping_add(1);
            let mut filled_wh = 0u128;
            let mut successful_fills = 0u64;
            for index in 0u64..32 {
                random = random
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                let quantity_wh = (random % 200_000 + 1) as u128;
                let current = ledger.offer("property-offer").unwrap().clone();
                let key = format!("fill-{seed}-{index}");
                let request_digest = digest((seed.wrapping_add(index) % 251) as u8);
                if quantity_wh <= current.remaining_wh {
                    let applied = ledger
                        .fill(
                            "property-offer",
                            current.revision,
                            quantity_wh,
                            &key,
                            request_digest,
                        )
                        .unwrap();
                    let receipt = match applied {
                        FillDisposition::Applied(receipt) => receipt,
                        other => panic!("new fill must apply, got {other:?}"),
                    };
                    filled_wh += quantity_wh;
                    successful_fills += 1;
                    let replay = ledger
                        .fill(
                            "property-offer",
                            current.revision,
                            quantity_wh,
                            &key,
                            request_digest,
                        )
                        .unwrap();
                    let replay_receipt = match replay {
                        FillDisposition::ExactReplay(receipt) => receipt,
                        other => panic!("exact fill retry must replay, got {other:?}"),
                    };
                    assert_eq!(receipt, replay_receipt);
                } else {
                    assert_eq!(
                        ledger.fill(
                            "property-offer",
                            current.revision,
                            quantity_wh,
                            &key,
                            request_digest,
                        ),
                        Err(KernelError::OfferOverfilled)
                    );
                }
                let after = ledger.offer("property-offer").unwrap();
                assert_eq!(filled_wh + after.remaining_wh, INITIAL_WH);
                assert_eq!(after.revision, successful_fills);
            }
        }
    }

    #[test]
    fn eac_generation_requires_independent_meter_derived_evidence_and_exact_boundary() {
        let mut ledger = EnergyAttributeLedger::default();
        let mut simulated = verified_generation();
        simulated.source = ClaimSource::Simulation;
        assert_eq!(
            ledger.register_generation(simulated, MeasurementBoundary::InverterAcOutput),
            Err(KernelError::IneligibleGenerationEvidence)
        );
        let mut self_verified = verified_generation();
        self_verified.verifier_id = self_verified.producer_id.clone();
        assert_eq!(
            ledger.register_generation(self_verified, MeasurementBoundary::InverterAcOutput),
            Err(KernelError::IneligibleGenerationEvidence)
        );
        let wrong_boundary = verified_generation();
        assert_eq!(
            ledger.register_generation(wrong_boundary, MeasurementBoundary::GridExport),
            Err(KernelError::InvalidBoundary)
        );
    }

    #[test]
    fn eac_overlapping_generation_intervals_and_duplicate_certificates_reject() {
        let mut ledger = EnergyAttributeLedger::default();
        ledger
            .register_generation(verified_generation(), MeasurementBoundary::InverterAcOutput)
            .unwrap();
        let mut overlapping = verified_generation();
        overlapping.generation_id = "generation-002".into();
        overlapping.interval_start_micros += 1_000;
        overlapping.interval_end_micros += 1_000;
        assert_eq!(
            ledger.register_generation(overlapping, MeasurementBoundary::InverterAcOutput),
            Err(KernelError::OverlappingGenerationInterval)
        );
        ledger
            .issue(
                "eac-001",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                1_000_000,
                1_000_000,
                "producer-account",
            )
            .unwrap();
        assert_eq!(
            ledger.issue(
                "eac-001",
                "accredited-registry-1",
                "irec-e-product-code/v1",
                "generation-001",
                1_000_000,
                1_000_000,
                "producer-account",
            ),
            Err(KernelError::DuplicateCertificate)
        );
    }

    #[test]
    fn eac_ledger_conserves_eligible_generation_under_deterministic_schedules() {
        const ELIGIBLE_WH: u128 = 20_000_000;
        const DENOMINATION_WH: u128 = 1_000_000;
        for seed in 0u64..128 {
            let mut ledger = EnergyAttributeLedger::default();
            let mut generation = verified_generation();
            generation.generation_id = format!("generation-{seed}");
            generation.meter_id = format!("meter-{seed}");
            generation.interval_start_micros = (seed as i64) * 10_000_000_000;
            generation.interval_end_micros = generation.interval_start_micros + 3_600_000_000;
            generation.eligible_energy_wh = ELIGIBLE_WH;
            ledger
                .register_generation(generation, MeasurementBoundary::InverterAcOutput)
                .unwrap();
            let generation_id = format!("generation-{seed}");
            let mut random = seed.wrapping_add(17);
            let mut issued = 0u128;
            for index in 0u64..32 {
                random = random
                    .wrapping_mul(2_862_933_555_777_941_757)
                    .wrapping_add(3_037_000_493);
                let quantity_wh = ((random % 4) as u128 + 1) * DENOMINATION_WH;
                let certificate_id = format!("eac-{seed}-{index}");
                if issued + quantity_wh <= ELIGIBLE_WH {
                    ledger
                        .issue(
                            &certificate_id,
                            "accredited-registry-1",
                            "irec-e-product-code/v1",
                            &generation_id,
                            quantity_wh,
                            DENOMINATION_WH,
                            "producer-account",
                        )
                        .unwrap();
                    issued += quantity_wh;
                    ledger
                        .transfer(&certificate_id, "producer-account", "buyer-account")
                        .unwrap();
                    ledger
                        .retire(
                            &certificate_id,
                            "buyer-account",
                            "buyer-claim",
                            &format!("retirement-{seed}-{index}"),
                        )
                        .unwrap();
                } else {
                    assert_eq!(
                        ledger.issue(
                            &certificate_id,
                            "accredited-registry-1",
                            "irec-e-product-code/v1",
                            &generation_id,
                            quantity_wh,
                            DENOMINATION_WH,
                            "producer-account",
                        ),
                        Err(KernelError::EacOverIssuance)
                    );
                }
                assert_eq!(ledger.issued_wh(&generation_id), Some(issued));
                assert_eq!(
                    ledger.remaining_eligible_wh(&generation_id),
                    Some(ELIGIBLE_WH - issued)
                );
                assert_eq!(
                    ledger.issued_wh(&generation_id).unwrap()
                        + ledger.remaining_eligible_wh(&generation_id).unwrap(),
                    ELIGIBLE_WH
                );
            }
        }
    }

    #[test]
    fn energy_attribute_scenario_inventory_is_unique_and_linked() {
        const CORPUS: &str = include_str!("../../pcoe-energy-attribute-scenarios.v0.1.json");
        let mut ids = BTreeSet::new();
        for item in ENERGY_ATTRIBUTE_SCENARIO_COVERAGE {
            assert!(
                ids.insert(item.id),
                "duplicate energy-attribute scenario mapping: {}",
                item.id
            );
            let needle = format!("\"id\": \"{}\"", item.id);
            assert!(
                CORPUS.contains(&needle),
                "scenario missing from energy-attribute corpus: {}",
                item.id
            );
        }
        assert_eq!(ids.len(), 12);
        assert_eq!(CORPUS.matches("\"id\":").count(), 12);
    }

    #[test]
    fn product_passport_scenario_inventory_is_unique_and_linked() {
        const CORPUS: &str = include_str!("../../pcoe-product-passport-scenarios.v0.1.json");
        let mut ids = BTreeSet::new();
        for item in DPP_SCENARIO_COVERAGE {
            assert!(
                ids.insert(item.id),
                "duplicate product-passport scenario mapping: {}",
                item.id
            );
            let needle = format!("\"id\": \"{}\"", item.id);
            assert!(
                CORPUS.contains(&needle),
                "scenario missing from product-passport corpus: {}",
                item.id
            );
            assert_eq!(item.status, CoverageStatus::NotYetImplemented);
        }
        assert_eq!(ids.len(), 12);
        assert_eq!(CORPUS.matches("\"id\":").count(), 12);
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
