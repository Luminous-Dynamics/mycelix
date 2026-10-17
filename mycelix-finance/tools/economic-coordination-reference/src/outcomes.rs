//! Append-only procurement outcome evidence and deterministic replay.
//!
//! This is a bounded reference ledger, not a cryptographic verifier, distributed
//! database, signature checker, supplier rating engine, or purchasing authority.
//! A source claim ID is the idempotency key supplied by an upstream adapter.
//! Distinct IDs for the same real-world fact cannot be deduplicated without
//! stronger source semantics and independent evidence checks.

use std::collections::{BTreeMap, BTreeSet};

use super::{EvidenceOrigin, EvidenceStatus};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum IssuerRole {
    ForecastService,
    Supplier,
    Carrier,
    Buyer,
    IndependentVerifier,
    LedgerOperator,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum OutcomeState {
    NoTrustedEvidence,
    Forecasted,
    Committed,
    InTransit,
    Received,
    PartiallyAccepted,
    Accepted,
    Rejected,
    Returned,
    Disputed,
    Finalized,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimeInterval {
    pub earliest_at: u64,
    pub latest_at: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OutcomeEventKind {
    ForecastIssued {
        model_digest: String,
        data_cutoff_at: u64,
        predicted_delivery_at: u64,
        delivery_interval: Option<TimeInterval>,
    },
    CommitmentAuthorized {
        quantity: u32,
        due_at: u64,
        authorized_landed_cost_minor: u64,
        currency: String,
        authorization_receipt_id: String,
    },
    ShipmentReported {
        quantity: u32,
        shipment_evidence_ref: String,
    },
    ReceiptObserved {
        quantity: u32,
        receipt_evidence_ref: String,
    },
    AcceptanceDecided {
        accepted_quantity: u32,
        rejected_quantity: u32,
        decision_evidence_ref: String,
    },
    ReturnObserved {
        quantity: u32,
        return_evidence_ref: String,
    },
    DisputeOpened {
        reason_code: String,
        dispute_evidence_ref: String,
    },
    DisputeResolved {
        dispute_event_id: String,
        resolution_evidence_ref: String,
    },
    OutcomeFinalized {
        net_accepted_quantity: u32,
        rejected_quantity: u32,
        realized_landed_cost_minor: u64,
        currency: String,
        accepted_at: Option<u64>,
        final_outcome_receipt_id: String,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutcomeEvent {
    /// Stable ledger identity for this event.
    pub event_id: String,
    /// Upstream idempotency key. Exact retries must reuse it.
    pub claim_id: String,
    pub case_id: String,
    pub commitment_id: String,
    pub issuer_role: IssuerRole,
    /// When the represented fact or forecast was created/effective.
    pub occurred_at: u64,
    /// When this ledger/adapter observed the claim. Can differ from occurred_at.
    pub observed_at: u64,
    pub evidence_status: EvidenceStatus,
    pub origin: EvidenceOrigin,
    pub source_uri: String,
    pub source_payload_digest: String,
    /// Corrections append a replacement event and point to the prior event ID.
    pub supersedes_event_id: Option<String>,
    pub kind: OutcomeEventKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AppendDisposition {
    Inserted,
    IdempotentReplay,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutcomeLedgerError {
    InvalidEvent,
    EventIdentityConflict,
    ClaimIdentityConflict,
    MissingSupersededEvent,
    SupersessionCrossesCase,
    SupersessionChangesEventFamily,
    EventAlreadySuperseded,
    InvalidDisputeReference,
    DuplicateActiveCommitment,
    DuplicateActiveFinalization,
    OutcomeQuantityMismatch,
    FinalizationMismatch,
    ArithmeticOverflow,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ForecastSummary {
    pub forecast_event_id: String,
    pub model_digest: String,
    pub data_cutoff_at: u64,
    pub predicted_delivery_at: u64,
    pub delivery_interval: Option<TimeInterval>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OutcomeCaseReport {
    pub case_id: String,
    pub commitment_id: String,
    pub state: OutcomeState,
    pub trusted_event_count: u64,
    pub excluded_or_unverified_event_count: u64,
    pub active_event_count: u64,
    /// Historical events replaced by later corrections; still present in the ledger.
    pub superseded_event_count: u64,
    pub open_dispute_event_ids: Vec<String>,
    pub pre_commitment_forecast: Option<ForecastSummary>,
    pub committed_quantity: Option<u32>,
    pub due_at: Option<u64>,
    pub authorized_landed_cost_minor: Option<u64>,
    pub currency: Option<String>,
    pub shipped_quantity: u64,
    pub received_quantity: u64,
    pub gross_accepted_quantity: u64,
    pub rejected_quantity: u64,
    pub returned_quantity: u64,
    pub net_accepted_quantity: u64,
    pub finalization_eligible_for_metrics: bool,
    /// None while there is no trusted, undisputed finalization.
    pub on_time_in_full: Option<bool>,
    /// actual acceptance time minus forecast point in seconds; positive is late.
    pub forecast_delivery_error_secs: Option<i128>,
    /// Whether actual accepted time fell inside the pre-commitment forecast interval.
    pub forecast_interval_covered: Option<bool>,
    /// realized landed cost minus authorized landed cost.
    pub landed_cost_variance_minor: Option<i128>,
    pub realized_landed_cost_minor: Option<u64>,
    pub final_accepted_at: Option<u64>,
}

/// Serial append-only reference book. This does not establish distributed
/// concurrency safety, durable storage, identity, signatures or physical truth.
#[derive(Default)]
pub struct OutcomeLedger {
    events: BTreeMap<String, OutcomeEvent>,
    claim_ids: BTreeMap<String, String>,
    /// Superseded event ID -> active replacement event ID.
    superseded_by: BTreeMap<String, String>,
}

fn non_empty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn event_family(kind: &OutcomeEventKind) -> u8 {
    match kind {
        OutcomeEventKind::ForecastIssued { .. } => 0,
        OutcomeEventKind::CommitmentAuthorized { .. } => 1,
        OutcomeEventKind::ShipmentReported { .. } => 2,
        OutcomeEventKind::ReceiptObserved { .. } => 3,
        OutcomeEventKind::AcceptanceDecided { .. } => 4,
        OutcomeEventKind::ReturnObserved { .. } => 5,
        OutcomeEventKind::DisputeOpened { .. } => 6,
        OutcomeEventKind::DisputeResolved { .. } => 7,
        OutcomeEventKind::OutcomeFinalized { .. } => 8,
    }
}

fn same_event_except_id(left: &OutcomeEvent, right: &OutcomeEvent) -> bool {
    let mut normalized_left = left.clone();
    let mut normalized_right = right.clone();
    normalized_left.event_id.clear();
    normalized_right.event_id.clear();
    normalized_left == normalized_right
}

fn trusted(event: &OutcomeEvent) -> bool {
    event.evidence_status == EvidenceStatus::Verified
        && event.origin == EvidenceOrigin::LiveDomain
}

fn role_allowed(role: IssuerRole, kind: &OutcomeEventKind) -> bool {
    match kind {
        OutcomeEventKind::ForecastIssued { .. } => matches!(
            role,
            IssuerRole::ForecastService | IssuerRole::Buyer | IssuerRole::LedgerOperator
        ),
        OutcomeEventKind::CommitmentAuthorized { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::LedgerOperator)
        }
        OutcomeEventKind::ShipmentReported { .. } => {
            matches!(role, IssuerRole::Supplier | IssuerRole::Carrier)
        }
        OutcomeEventKind::ReceiptObserved { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::IndependentVerifier)
        }
        OutcomeEventKind::AcceptanceDecided { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::IndependentVerifier)
        }
        OutcomeEventKind::ReturnObserved { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::IndependentVerifier)
        }
        OutcomeEventKind::DisputeOpened { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::Supplier)
        }
        OutcomeEventKind::DisputeResolved { .. } => role == IssuerRole::IndependentVerifier,
        OutcomeEventKind::OutcomeFinalized { .. } => {
            matches!(role, IssuerRole::Buyer | IssuerRole::IndependentVerifier)
        }
    }
}

fn validate_event(event: &OutcomeEvent) -> Result<(), OutcomeLedgerError> {
    if !non_empty(&event.event_id)
        || !non_empty(&event.claim_id)
        || !non_empty(&event.case_id)
        || !non_empty(&event.commitment_id)
        || !non_empty(&event.source_uri)
        || !non_empty(&event.source_payload_digest)
        || event.origin == EvidenceOrigin::Unknown && event.evidence_status == EvidenceStatus::Verified
        || event.observed_at < event.occurred_at
        || !role_allowed(event.issuer_role, &event.kind)
    {
        return Err(OutcomeLedgerError::InvalidEvent);
    }

    let valid = match &event.kind {
        OutcomeEventKind::ForecastIssued {
            model_digest,
            data_cutoff_at,
            predicted_delivery_at,
            delivery_interval,
        } => {
            non_empty(model_digest)
                && *data_cutoff_at <= event.occurred_at
                && delivery_interval.map_or(true, |interval| {
                    interval.earliest_at <= *predicted_delivery_at
                        && *predicted_delivery_at <= interval.latest_at
                })
        }
        OutcomeEventKind::CommitmentAuthorized {
            quantity,
            due_at,
            currency,
            authorization_receipt_id,
            ..
        } => {
            *quantity > 0
                && *due_at >= event.occurred_at
                && non_empty(currency)
                && non_empty(authorization_receipt_id)
        }
        OutcomeEventKind::ShipmentReported {
            quantity,
            shipment_evidence_ref,
        } => *quantity > 0 && non_empty(shipment_evidence_ref),
        OutcomeEventKind::ReceiptObserved {
            quantity,
            receipt_evidence_ref,
        } => *quantity > 0 && non_empty(receipt_evidence_ref),
        OutcomeEventKind::AcceptanceDecided {
            accepted_quantity,
            rejected_quantity,
            decision_evidence_ref,
        } => {
            accepted_quantity.checked_add(*rejected_quantity).unwrap_or(0) > 0
                && accepted_quantity.checked_add(*rejected_quantity).is_some()
                && non_empty(decision_evidence_ref)
        }
        OutcomeEventKind::ReturnObserved {
            quantity,
            return_evidence_ref,
        } => *quantity > 0 && non_empty(return_evidence_ref),
        OutcomeEventKind::DisputeOpened {
            reason_code,
            dispute_evidence_ref,
        } => non_empty(reason_code) && non_empty(dispute_evidence_ref),
        OutcomeEventKind::DisputeResolved {
            dispute_event_id,
            resolution_evidence_ref,
        } => non_empty(dispute_event_id) && non_empty(resolution_evidence_ref),
        OutcomeEventKind::OutcomeFinalized {
            net_accepted_quantity,
            rejected_quantity,
            currency,
            accepted_at,
            final_outcome_receipt_id,
            ..
        } => {
            net_accepted_quantity.checked_add(*rejected_quantity).is_some()
                && non_empty(currency)
                && non_empty(final_outcome_receipt_id)
                && (*net_accepted_quantity == 0 || accepted_at.is_some())
                && accepted_at.map_or(true, |at| at <= event.observed_at)
        }
    };

    if !valid {
        return Err(OutcomeLedgerError::InvalidEvent);
    }
    Ok(())
}

impl OutcomeLedger {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn event_count(&self) -> usize {
        self.events.len()
    }

    pub fn get_event(&self, event_id: &str) -> Option<&OutcomeEvent> {
        self.events.get(event_id)
    }

    pub fn append(
        &mut self,
        event: OutcomeEvent,
    ) -> Result<AppendDisposition, OutcomeLedgerError> {
        validate_event(&event)?;

        if let Some(existing) = self.events.get(&event.event_id) {
            return if existing == &event {
                Ok(AppendDisposition::IdempotentReplay)
            } else {
                Err(OutcomeLedgerError::EventIdentityConflict)
            };
        }

        if let Some(existing_id) = self.claim_ids.get(&event.claim_id) {
            let existing = self.events.get(existing_id)
                .ok_or(OutcomeLedgerError::ClaimIdentityConflict)?;
            return if same_event_except_id(existing, &event) {
                Ok(AppendDisposition::IdempotentReplay)
            } else {
                Err(OutcomeLedgerError::ClaimIdentityConflict)
            };
        }

        if let Some(target_id) = &event.supersedes_event_id {
            let target = self.events.get(target_id)
                .ok_or(OutcomeLedgerError::MissingSupersededEvent)?;
            if target.case_id != event.case_id || target.commitment_id != event.commitment_id {
                return Err(OutcomeLedgerError::SupersessionCrossesCase);
            }
            if event_family(&target.kind) != event_family(&event.kind) {
                return Err(OutcomeLedgerError::SupersessionChangesEventFamily);
            }
            if event.observed_at < target.observed_at {
                return Err(OutcomeLedgerError::InvalidEvent);
            }
            if self.superseded_by.contains_key(target_id) {
                return Err(OutcomeLedgerError::EventAlreadySuperseded);
            }
        }

        if let OutcomeEventKind::DisputeResolved { dispute_event_id, .. } = &event.kind {
            let dispute = self.events.get(dispute_event_id)
                .ok_or(OutcomeLedgerError::InvalidDisputeReference)?;
            if dispute.case_id != event.case_id
                || dispute.commitment_id != event.commitment_id
                || !matches!(&dispute.kind, OutcomeEventKind::DisputeOpened { .. })
                || self.superseded_by.contains_key(dispute_event_id)
                || self.events.values().any(|existing| {
                    !self.superseded_by.contains_key(&existing.event_id)
                        && matches!(
                            &existing.kind,
                            OutcomeEventKind::DisputeResolved { dispute_event_id: resolved_id, .. }
                                if resolved_id == dispute_event_id
                        )
                })
            {
                return Err(OutcomeLedgerError::InvalidDisputeReference);
            }
        }

        if let Some(target_id) = &event.supersedes_event_id {
            self.superseded_by.insert(target_id.clone(), event.event_id.clone());
        }
        self.claim_ids.insert(event.claim_id.clone(), event.event_id.clone());
        self.events.insert(event.event_id.clone(), event);
        Ok(AppendDisposition::Inserted)
    }

    pub fn active_events(&self, case_id: &str) -> Vec<&OutcomeEvent> {
        let mut active: Vec<&OutcomeEvent> = self.events.values()
            .filter(|event| {
                event.case_id == case_id && !self.superseded_by.contains_key(&event.event_id)
            })
            .collect();
        active.sort_by(|left, right| {
            left.occurred_at.cmp(&right.occurred_at)
                .then_with(|| left.event_id.cmp(&right.event_id))
        });
        active
    }

    pub fn report_case(
        &self,
        case_id: &str,
        commitment_id: &str,
    ) -> Result<OutcomeCaseReport, OutcomeLedgerError> {
        if !non_empty(case_id) || !non_empty(commitment_id) {
            return Err(OutcomeLedgerError::InvalidEvent);
        }
        let active_all: Vec<&OutcomeEvent> = self.active_events(case_id).into_iter()
            .filter(|event| event.commitment_id == commitment_id)
            .collect();
        let superseded_event_count = self.events.values().filter(|event| {
            event.case_id == case_id
                && event.commitment_id == commitment_id
                && self.superseded_by.contains_key(&event.event_id)
        }).count() as u64;
        let untrusted_count = active_all.iter().filter(|event| !trusted(event)).count() as u64;
        let events: Vec<&OutcomeEvent> = active_all.into_iter().filter(|event| trusted(event)).collect();
        let mut commitments: Vec<&OutcomeEvent> = events.iter().copied()
            .filter(|event| matches!(&event.kind, OutcomeEventKind::CommitmentAuthorized { .. }))
            .collect();
        commitments.sort_by(|left, right| {
            left.occurred_at.cmp(&right.occurred_at).then_with(|| left.event_id.cmp(&right.event_id))
        });
        if commitments.len() > 1 {
            return Err(OutcomeLedgerError::DuplicateActiveCommitment);
        }
        let commitment_event = commitments.first().copied();
        let committed_quantity = commitment_event.and_then(|event| {
            if let OutcomeEventKind::CommitmentAuthorized { quantity, .. } = &event.kind {
                Some(*quantity)
            } else { None }
        });
        let due_at = commitment_event.and_then(|event| {
            if let OutcomeEventKind::CommitmentAuthorized { due_at, .. } = &event.kind {
                Some(*due_at)
            } else { None }
        });
        let authorized_landed_cost_minor = commitment_event.and_then(|event| {
            if let OutcomeEventKind::CommitmentAuthorized { authorized_landed_cost_minor, .. } = &event.kind {
                Some(*authorized_landed_cost_minor)
            } else { None }
        });
        let currency = commitment_event.and_then(|event| {
            if let OutcomeEventKind::CommitmentAuthorized { currency, .. } = &event.kind {
                Some(currency.clone())
            } else { None }
        });

        let pre_commitment_forecast = commitment_event.and_then(|commitment| {
            events.iter().copied()
                .filter(|event| event.occurred_at <= commitment.occurred_at)
                .filter_map(|event| match &event.kind {
                    OutcomeEventKind::ForecastIssued { model_digest, data_cutoff_at, predicted_delivery_at, delivery_interval } =>
                        Some((event, ForecastSummary {
                            forecast_event_id: event.event_id.clone(),
                            model_digest: model_digest.clone(),
                            data_cutoff_at: *data_cutoff_at,
                            predicted_delivery_at: *predicted_delivery_at,
                            delivery_interval: *delivery_interval,
                        })),
                    _ => None,
                })
                .max_by(|(left, _), (right, _)| left.occurred_at.cmp(&right.occurred_at).then_with(|| left.event_id.cmp(&right.event_id)))
                .map(|(_, summary)| summary)
        });

        let sum_quantity = |kind_match: fn(&OutcomeEventKind) -> Option<u32>| -> Result<u64, OutcomeLedgerError> {
            events.iter().try_fold(0u64, |total, event| {
                let quantity = kind_match(&event.kind).unwrap_or(0);
                total.checked_add(u64::from(quantity)).ok_or(OutcomeLedgerError::ArithmeticOverflow)
            })
        };
        let shipped_quantity = sum_quantity(|kind| match kind {
            OutcomeEventKind::ShipmentReported { quantity, .. } => Some(*quantity),
            _ => None,
        })?;
        let received_quantity = sum_quantity(|kind| match kind {
            OutcomeEventKind::ReceiptObserved { quantity, .. } => Some(*quantity),
            _ => None,
        })?;
        let mut gross_accepted_quantity = sum_quantity(|kind| match kind {
            OutcomeEventKind::AcceptanceDecided { accepted_quantity, .. } => Some(*accepted_quantity),
            _ => None,
        })?;
        let mut rejected_quantity = sum_quantity(|kind| match kind {
            OutcomeEventKind::AcceptanceDecided { rejected_quantity, .. } => Some(*rejected_quantity),
            _ => None,
        })?;
        let mut returned_quantity = sum_quantity(|kind| match kind {
            OutcomeEventKind::ReturnObserved { quantity, .. } => Some(*quantity),
            _ => None,
        })?;
        let has_outcome_detail = events.iter().any(|event| matches!(&event.kind,
            OutcomeEventKind::AcceptanceDecided { .. } | OutcomeEventKind::ReturnObserved { .. }));
        let inspected_quantity = gross_accepted_quantity
            .checked_add(rejected_quantity)
            .ok_or(OutcomeLedgerError::ArithmeticOverflow)?;
        if returned_quantity > gross_accepted_quantity
            || (has_outcome_detail && inspected_quantity > received_quantity)
            || committed_quantity.map_or(false, |committed| {
                inspected_quantity > u64::from(committed)
            })
        {
            return Err(OutcomeLedgerError::OutcomeQuantityMismatch);
        }
        let mut net_accepted_quantity = gross_accepted_quantity - returned_quantity;

        let mut resolved_dispute_ids = BTreeSet::new();
        for event in &events {
            if let OutcomeEventKind::DisputeResolved { dispute_event_id, .. } = &event.kind {
                resolved_dispute_ids.insert(dispute_event_id.as_str());
            }
        }
        let mut open_dispute_event_ids: Vec<String> = events.iter().filter_map(|event| {
            if matches!(&event.kind, OutcomeEventKind::DisputeOpened { .. })
                && !resolved_dispute_ids.contains(event.event_id.as_str())
            {
                Some(event.event_id.clone())
            } else { None }
        }).collect();
        open_dispute_event_ids.sort();

        let mut finalizations: Vec<&OutcomeEvent> = events.iter().copied()
            .filter(|event| matches!(&event.kind, OutcomeEventKind::OutcomeFinalized { .. }))
            .collect();
        finalizations.sort_by(|left, right| {
            left.occurred_at.cmp(&right.occurred_at).then_with(|| left.event_id.cmp(&right.event_id))
        });
        if finalizations.len() > 1 {
            return Err(OutcomeLedgerError::DuplicateActiveFinalization);
        }
        let finalization = finalizations.first().copied();
        if finalization.is_some() && commitment_event.is_none() {
            return Err(OutcomeLedgerError::FinalizationMismatch);
        }
        if let Some(event) = finalization {
            if let OutcomeEventKind::OutcomeFinalized {
                net_accepted_quantity: finalized_accepted,
                rejected_quantity: finalized_rejected,
                currency: final_currency,
                ..
            } = &event.kind {
                if has_outcome_detail && (u64::from(*finalized_accepted) != net_accepted_quantity
                    || u64::from(*finalized_rejected) != rejected_quantity)
                {
                    return Err(OutcomeLedgerError::FinalizationMismatch);
                }
                if !has_outcome_detail {
                    // When detailed acceptance/return events are not available,
                    // an independently evidenced finalization is the outcome summary.
                    gross_accepted_quantity = u64::from(*finalized_accepted);
                    rejected_quantity = u64::from(*finalized_rejected);
                    returned_quantity = 0;
                    net_accepted_quantity = u64::from(*finalized_accepted);
                }
                if currency.as_deref() != Some(final_currency.as_str()) {
                    return Err(OutcomeLedgerError::FinalizationMismatch);
                }
                if let Some(committed) = committed_quantity {
                    if *finalized_accepted > committed
                        || u64::from(*finalized_accepted) + u64::from(*finalized_rejected) > u64::from(committed)
                    {
                        return Err(OutcomeLedgerError::OutcomeQuantityMismatch);
                    }
                }
            }
        }
        let disputed = !open_dispute_event_ids.is_empty();
        let state = if disputed {
            OutcomeState::Disputed
        } else if finalization.is_some() {
            OutcomeState::Finalized
        } else if returned_quantity > 0 {
            OutcomeState::Returned
        } else if net_accepted_quantity > 0 && committed_quantity.map_or(false, |quantity| net_accepted_quantity < u64::from(quantity)) {
            OutcomeState::PartiallyAccepted
        } else if net_accepted_quantity > 0 {
            OutcomeState::Accepted
        } else if rejected_quantity > 0 {
            OutcomeState::Rejected
        } else if received_quantity > 0 {
            OutcomeState::Received
        } else if shipped_quantity > 0 {
            OutcomeState::InTransit
        } else if commitment_event.is_some() {
            OutcomeState::Committed
        } else if pre_commitment_forecast.is_some() || events.iter().any(|event| matches!(&event.kind, OutcomeEventKind::ForecastIssued { .. })) {
            OutcomeState::Forecasted
        } else {
            OutcomeState::NoTrustedEvidence
        };

        let mut on_time_in_full = None;
        let mut forecast_delivery_error_secs = None;
        let mut forecast_interval_covered = None;
        let mut landed_cost_variance_minor = None;
        let mut realized_landed_cost_minor = None;
        let mut final_accepted_at = None;
        let finalization_eligible_for_metrics = finalization.is_some() && !disputed;
        if finalization_eligible_for_metrics {
            if let Some(event) = finalization {
                if let OutcomeEventKind::OutcomeFinalized {
                    net_accepted_quantity,
                    rejected_quantity: finalized_rejected,
                    realized_landed_cost_minor: realized_cost,
                    accepted_at,
                    ..
                } = &event.kind {
                    final_accepted_at = *accepted_at;
                    realized_landed_cost_minor = Some(*realized_cost);
                    if let Some(expected_cost) = authorized_landed_cost_minor {
                        landed_cost_variance_minor = Some(i128::from(*realized_cost) - i128::from(expected_cost));
                    }
                    if let (Some(committed), Some(due), Some(accepted_at)) =
                        (committed_quantity, due_at, *accepted_at)
                    {
                        on_time_in_full = Some(*net_accepted_quantity == committed
                            && *finalized_rejected == 0
                            && accepted_at <= due);
                    } else if committed_quantity.is_some() && due_at.is_some() {
                        on_time_in_full = Some(false);
                    }
                    if let (Some(forecast), Some(accepted_at)) = (&pre_commitment_forecast, *accepted_at) {
                        forecast_delivery_error_secs =
                            Some(i128::from(accepted_at) - i128::from(forecast.predicted_delivery_at));
                        forecast_interval_covered = forecast.delivery_interval.map(|interval| {
                            interval.earliest_at <= accepted_at && accepted_at <= interval.latest_at
                        });
                    }
                }
            }
        }

        Ok(OutcomeCaseReport {
            case_id: case_id.to_owned(),
            commitment_id: commitment_id.to_owned(),
            state,
            trusted_event_count: events.len() as u64,
            excluded_or_unverified_event_count: untrusted_count,
            active_event_count: (events.len() as u64).checked_add(untrusted_count)
                .ok_or(OutcomeLedgerError::ArithmeticOverflow)?,
            superseded_event_count,
            open_dispute_event_ids,
            pre_commitment_forecast,
            committed_quantity,
            due_at,
            authorized_landed_cost_minor,
            currency,
            shipped_quantity,
            received_quantity,
            gross_accepted_quantity,
            rejected_quantity,
            returned_quantity,
            net_accepted_quantity,
            finalization_eligible_for_metrics,
            on_time_in_full,
            forecast_delivery_error_secs,
            forecast_interval_covered,
            landed_cost_variance_minor,
            realized_landed_cost_minor,
            final_accepted_at,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EvidenceOrigin, EvidenceStatus};

    fn event(
        event_id: &str,
        claim_id: &str,
        role: IssuerRole,
        occurred_at: u64,
        observed_at: u64,
        kind: OutcomeEventKind,
    ) -> OutcomeEvent {
        OutcomeEvent {
            event_id: event_id.to_owned(),
            claim_id: claim_id.to_owned(),
            case_id: "case-1".to_owned(),
            commitment_id: "commitment-1".to_owned(),
            issuer_role: role,
            occurred_at,
            observed_at,
            evidence_status: EvidenceStatus::Verified,
            origin: EvidenceOrigin::LiveDomain,
            source_uri: format!("https://example.invalid/{claim_id}"),
            source_payload_digest: format!("digest-{claim_id}"),
            supersedes_event_id: None,
            kind,
        }
    }

    fn basic_finalized_ledger() -> OutcomeLedger {
        let mut ledger = OutcomeLedger::new();
        ledger.append(event(
            "forecast-1", "claim-forecast-1", IssuerRole::ForecastService, 10, 10,
            OutcomeEventKind::ForecastIssued {
                model_digest: "model-v1".to_owned(), data_cutoff_at: 9,
                predicted_delivery_at: 100,
                delivery_interval: Some(TimeInterval { earliest_at: 90, latest_at: 120 }),
            },
        )).unwrap();
        ledger.append(event(
            "commit-1", "claim-commit-1", IssuerRole::Buyer, 20, 20,
            OutcomeEventKind::CommitmentAuthorized {
                quantity: 5, due_at: 110, authorized_landed_cost_minor: 500,
                currency: "USD".to_owned(), authorization_receipt_id: "authorization-1".to_owned(),
            },
        )).unwrap();
        ledger.append(event(
            "ship-1", "claim-ship-1", IssuerRole::Carrier, 80, 81,
            OutcomeEventKind::ShipmentReported { quantity: 5, shipment_evidence_ref: "tracking-1".to_owned() },
        )).unwrap();
        ledger.append(event(
            "receipt-1", "claim-receipt-1", IssuerRole::Buyer, 105, 106,
            OutcomeEventKind::ReceiptObserved { quantity: 5, receipt_evidence_ref: "receiving-1".to_owned() },
        )).unwrap();
        ledger.append(event(
            "accept-1", "claim-accept-1", IssuerRole::IndependentVerifier, 106, 107,
            OutcomeEventKind::AcceptanceDecided {
                accepted_quantity: 5, rejected_quantity: 0, decision_evidence_ref: "inspection-1".to_owned(),
            },
        )).unwrap();
        ledger.append(event(
            "final-1", "claim-final-1", IssuerRole::IndependentVerifier, 108, 108,
            OutcomeEventKind::OutcomeFinalized {
                net_accepted_quantity: 5, rejected_quantity: 0, realized_landed_cost_minor: 525,
                currency: "USD".to_owned(), accepted_at: Some(106), final_outcome_receipt_id: "final-receipt-1".to_owned(),
            },
        )).unwrap();
        ledger
    }

    #[test]
    fn finalized_case_computes_otif_forecast_error_and_cost_variance() {
        let report = basic_finalized_ledger().report_case("case-1", "commitment-1").unwrap();
        assert_eq!(report.state, OutcomeState::Finalized);
        assert_eq!(report.on_time_in_full, Some(true));
        assert_eq!(report.forecast_delivery_error_secs, Some(6));
        assert_eq!(report.forecast_interval_covered, Some(true));
        assert_eq!(report.landed_cost_variance_minor, Some(25));
        assert_eq!(report.net_accepted_quantity, 5);
    }

    #[test]
    fn finalization_summary_supplies_quantities_when_detailed_acceptance_events_are_missing() {
        let mut ledger = OutcomeLedger::new();
        ledger.append(event(
            "commit-summary",
            "claim-commit-summary",
            IssuerRole::Buyer,
            20,
            20,
            OutcomeEventKind::CommitmentAuthorized {
                quantity: 5,
                due_at: 110,
                authorized_landed_cost_minor: 500,
                currency: "USD".to_owned(),
                authorization_receipt_id: "auth-summary".to_owned(),
            },
        )).unwrap();
        ledger.append(event(
            "final-summary",
            "claim-final-summary",
            IssuerRole::IndependentVerifier,
            120,
            120,
            OutcomeEventKind::OutcomeFinalized {
                net_accepted_quantity: 2,
                rejected_quantity: 3,
                realized_landed_cost_minor: 525,
                currency: "USD".to_owned(),
                accepted_at: Some(115),
                final_outcome_receipt_id: "summary-outcome".to_owned(),
            },
        )).unwrap();
        let report = ledger.report_case("case-1", "commitment-1").unwrap();
        assert_eq!(report.state, OutcomeState::Finalized);
        assert_eq!(report.net_accepted_quantity, 2);
        assert_eq!(report.rejected_quantity, 3);
        assert_eq!(report.on_time_in_full, Some(false));
    }

    #[test]
    fn exact_retry_is_idempotent_and_claim_id_conflict_fails_closed() {
        let mut ledger = basic_finalized_ledger();
        let original = ledger.get_event("receipt-1").unwrap().clone();
        assert_eq!(ledger.append(original.clone()), Ok(AppendDisposition::IdempotentReplay));
        let mut conflict = original;
        conflict.kind = OutcomeEventKind::ReceiptObserved {
            quantity: 4, receipt_evidence_ref: "other-receipt".to_owned(),
        };
        assert_eq!(ledger.append(conflict), Err(OutcomeLedgerError::EventIdentityConflict));

        let mut reused = ledger.get_event("receipt-1").unwrap().clone();
        reused.event_id = "receipt-2".to_owned();
        assert_eq!(ledger.append(reused), Ok(AppendDisposition::IdempotentReplay));
        assert_eq!(ledger.event_count(), 6);
    }

    #[test]
    fn disputed_finalization_does_not_publish_otif_or_calibration_metrics() {
        let mut ledger = basic_finalized_ledger();
        ledger.append(event(
            "dispute-1", "claim-dispute-1", IssuerRole::Supplier, 109, 109,
            OutcomeEventKind::DisputeOpened { reason_code: "quantity-contested".to_owned(), dispute_evidence_ref: "dispute-evidence".to_owned() },
        )).unwrap();
        let report = ledger.report_case("case-1", "commitment-1").unwrap();
        assert_eq!(report.state, OutcomeState::Disputed);
        assert_eq!(report.on_time_in_full, None);
        assert_eq!(report.forecast_delivery_error_secs, None);
        assert_eq!(report.forecast_interval_covered, None);
    }

    #[test]
    fn unknown_or_synthetic_outcomes_are_visible_but_not_scored_as_truth() {
        let mut ledger = OutcomeLedger::new();
        let mut simulated = event(
            "fake-accept", "claim-sim", IssuerRole::IndependentVerifier, 20, 20,
            OutcomeEventKind::AcceptanceDecided {
                accepted_quantity: 5, rejected_quantity: 0, decision_evidence_ref: "simulated".to_owned(),
            },
        );
        simulated.origin = EvidenceOrigin::SyntheticSimulation;
        ledger.append(simulated).unwrap();
        let report = ledger.report_case("case-1", "commitment-1").unwrap();
        assert_eq!(report.state, OutcomeState::NoTrustedEvidence);
        assert_eq!(report.excluded_or_unverified_event_count, 1);
        assert_eq!(report.net_accepted_quantity, 0);
    }

    #[test]
    fn correction_supersedes_without_deleting_original_event() {
        let mut ledger = basic_finalized_ledger();
        let mut correction = event(
            "receipt-corrected", "claim-receipt-corrected", IssuerRole::Buyer, 105, 120,
            OutcomeEventKind::ReceiptObserved { quantity: 4, receipt_evidence_ref: "receiving-correction-1".to_owned() },
        );
        correction.supersedes_event_id = Some("receipt-1".to_owned());
        ledger.append(correction).unwrap();
        assert!(ledger.get_event("receipt-1").is_some());
        let report = ledger.report_case("case-1", "commitment-1");
        assert_eq!(report, Err(OutcomeLedgerError::OutcomeQuantityMismatch));
        assert_eq!(ledger.event_count(), 7);
    }

    #[test]
    fn late_arriving_fact_replays_in_occurred_time_order() {
        let mut ledger = OutcomeLedger::new();
        ledger.append(event(
            "receipt-late", "claim-receipt-late", IssuerRole::Buyer, 105, 130,
            OutcomeEventKind::ReceiptObserved { quantity: 1, receipt_evidence_ref: "receipt-late-1".to_owned() },
        )).unwrap();
        ledger.append(event(
            "commit-late", "claim-commit-late", IssuerRole::Buyer, 20, 131,
            OutcomeEventKind::CommitmentAuthorized {
                quantity: 1, due_at: 110, authorized_landed_cost_minor: 100,
                currency: "USD".to_owned(), authorization_receipt_id: "auth-late-1".to_owned(),
            },
        )).unwrap();
        ledger.append(event(
            "accept-late", "claim-accept-late", IssuerRole::IndependentVerifier, 106, 132,
            OutcomeEventKind::AcceptanceDecided { accepted_quantity: 1, rejected_quantity: 0, decision_evidence_ref: "accept-late-1".to_owned() },
        )).unwrap();
        assert_eq!(ledger.report_case("case-1", "commitment-1").unwrap().net_accepted_quantity, 1);
    }

    #[test]
    fn incompatible_correction_and_cross_case_dispute_resolution_are_rejected() {
        let mut ledger = basic_finalized_ledger();
        let mut wrong_family = event(
            "wrong-correction", "claim-wrong-correction", IssuerRole::Buyer, 110, 110,
            OutcomeEventKind::ShipmentReported { quantity: 2, shipment_evidence_ref: "corrected-shipment".to_owned() },
        );
        wrong_family.supersedes_event_id = Some("receipt-1".to_owned());
        assert_eq!(ledger.append(wrong_family), Err(OutcomeLedgerError::SupersessionChangesEventFamily));

        let mut cross_case = event(
            "other-case-dispute", "claim-other-dispute", IssuerRole::Supplier, 109, 109,
            OutcomeEventKind::DisputeOpened { reason_code: "claim".to_owned(), dispute_evidence_ref: "claim-doc".to_owned() },
        );
        cross_case.case_id = "case-2".to_owned();
        ledger.append(cross_case).unwrap();
        let mut resolution = event(
            "bad-resolution", "claim-bad-resolution", IssuerRole::IndependentVerifier, 120, 120,
            OutcomeEventKind::DisputeResolved { dispute_event_id: "other-case-dispute".to_owned(), resolution_evidence_ref: "decision".to_owned() },
        );
        assert_eq!(ledger.append(resolution.clone()), Err(OutcomeLedgerError::InvalidDisputeReference));
        resolution.case_id = "case-2".to_owned();
        assert_eq!(ledger.append(resolution), Ok(AppendDisposition::Inserted));
    }

    #[test]
    fn same_claim_id_with_different_payload_is_rejected_even_if_event_id_changes() {
        let mut ledger = basic_finalized_ledger();
        let mut conflicting = ledger.get_event("receipt-1").unwrap().clone();
        conflicting.event_id = "different-event-id".to_owned();
        conflicting.kind = OutcomeEventKind::ReceiptObserved {
            quantity: 4,
            receipt_evidence_ref: "changed-receipt".to_owned(),
        };
        assert_eq!(
            ledger.append(conflicting),
            Err(OutcomeLedgerError::ClaimIdentityConflict)
        );
    }

    #[test]
    fn replay_is_independent_of_ingestion_order() {
        let first = basic_finalized_ledger();
        let events: Vec<OutcomeEvent> = first.events.values().cloned().collect();
        let mut second = OutcomeLedger::new();
        for event in events.into_iter().rev() {
            second.append(event).unwrap();
        }
        assert_eq!(
            first.report_case("case-1", "commitment-1").unwrap(),
            second.report_case("case-1", "commitment-1").unwrap()
        );
    }

    #[test]
    fn a_valid_correction_is_append_only_and_replays_replacement() {
        let mut ledger = basic_finalized_ledger();
        let mut correction = event(
            "receipt-corrected",
            "claim-receipt-corrected",
            IssuerRole::Buyer,
            105,
            120,
            OutcomeEventKind::ReceiptObserved {
                quantity: 5,
                receipt_evidence_ref: "receiving-correction-1".to_owned(),
            },
        );
        correction.supersedes_event_id = Some("receipt-1".to_owned());
        ledger.append(correction).unwrap();
        assert!(ledger.get_event("receipt-1").is_some());
        assert_eq!(
            ledger.report_case("case-1", "commitment-1").unwrap().superseded_event_count,
            1
        );
        assert_eq!(ledger.active_events("case-1").iter().filter(|e| {
            matches!(&e.kind, OutcomeEventKind::ReceiptObserved { .. })
        }).count(), 1);
        let report = ledger.report_case("case-1", "commitment-1").unwrap();
        assert_eq!(report.state, OutcomeState::Finalized);
        assert_eq!(report.received_quantity, 5);
        assert_eq!(report.on_time_in_full, Some(true));
    }

    #[test]
    fn future_observation_and_invalid_forecast_interval_are_rejected() {
        let mut ledger = OutcomeLedger::new();
        let future = event(
            "future-1", "claim-future", IssuerRole::Supplier, 50, 49,
            OutcomeEventKind::ShipmentReported { quantity: 1, shipment_evidence_ref: "future-shipment".to_owned() },
        );
        assert_eq!(ledger.append(future), Err(OutcomeLedgerError::InvalidEvent));
        let invalid_interval = event(
            "forecast-bad", "claim-forecast-bad", IssuerRole::ForecastService, 10, 10,
            OutcomeEventKind::ForecastIssued {
                model_digest: "model".to_owned(), data_cutoff_at: 9,
                predicted_delivery_at: 100,
                delivery_interval: Some(TimeInterval { earliest_at: 110, latest_at: 120 }),
            },
        );
        assert_eq!(ledger.append(invalid_interval), Err(OutcomeLedgerError::InvalidEvent));
    }
}
