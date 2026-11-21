//! Provider-neutral queue delivery binding and worker handoff semantics.
//!
//! A broker delivery is transport evidence only. This module checks that a
//! delivery matches the exact command envelope and that the command remains
//! inside its bounded validity interval. It does not authorize execution,
//! verify the decision receipt signature, install a fence, or prove an effect.
//! The effect owner must still verify the Allow receipt and current generation
//! atomically with the protected mutation.

use crate::receipts::{EffectOwnerId, LogicalOperationId};
use crate::{ClockWindow, Digest, MAX_IDENTIFIER_BYTES};

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct QueueIdentifier(String);

impl QueueIdentifier {
    pub fn new(value: impl Into<String>) -> Result<Self, QueueDeliveryError> {
        let value = value.into();
        if value.trim().is_empty() {
            return Err(QueueDeliveryError::EmptyIdentifier);
        }
        if value.len() > MAX_IDENTIFIER_BYTES {
            return Err(QueueDeliveryError::IdentifierTooLong);
        }
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QueueDeliveryError {
    EmptyIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    InvalidGeneration,
    InvalidValidityWindow,
}

fn append_identifier(bytes: &mut Vec<u8>, value: &str) -> Result<(), QueueDeliveryError> {
    let len = u16::try_from(value.len()).map_err(|_| QueueDeliveryError::IdentifierTooLong)?;
    bytes.extend_from_slice(&len.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QueueCommandBinding {
    broker_id: QueueIdentifier,
    destination_id: QueueIdentifier,
    message_key: QueueIdentifier,
    payload_digest: Digest,
    request_digest: Digest,
    decision_receipt_digest: Digest,
    logical_operation_id: LogicalOperationId,
    effect_owner_id: EffectOwnerId,
    effect_fence_generation: u64,
    clock_profile_id: QueueIdentifier,
    issued_at_ms: u64,
    not_before_ms: u64,
    expires_at_ms: u64,
    max_clock_uncertainty_ms: u64,
}

impl QueueCommandBinding {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        broker_id: QueueIdentifier,
        destination_id: QueueIdentifier,
        message_key: QueueIdentifier,
        payload_digest: Digest,
        request_digest: Digest,
        decision_receipt_digest: Digest,
        logical_operation_id: LogicalOperationId,
        effect_owner_id: EffectOwnerId,
        effect_fence_generation: u64,
        clock_profile_id: QueueIdentifier,
        issued_at_ms: u64,
        not_before_ms: u64,
        expires_at_ms: u64,
        max_clock_uncertainty_ms: u64,
    ) -> Result<Self, QueueDeliveryError> {
        if [payload_digest, request_digest, decision_receipt_digest]
            .iter()
            .any(|digest| *digest == [0; 32])
        {
            return Err(QueueDeliveryError::ZeroDigest);
        }
        if effect_fence_generation == 0 {
            return Err(QueueDeliveryError::InvalidGeneration);
        }
        if issued_at_ms > not_before_ms || not_before_ms >= expires_at_ms {
            return Err(QueueDeliveryError::InvalidValidityWindow);
        }

        Ok(Self {
            broker_id,
            destination_id,
            message_key,
            payload_digest,
            request_digest,
            decision_receipt_digest,
            logical_operation_id,
            effect_owner_id,
            effect_fence_generation,
            clock_profile_id,
            issued_at_ms,
            not_before_ms,
            expires_at_ms,
            max_clock_uncertainty_ms,
        })
    }

    pub fn effect_owner_id(&self) -> &EffectOwnerId {
        &self.effect_owner_id
    }

    pub fn effect_fence_generation(&self) -> u64 {
        self.effect_fence_generation
    }

    pub fn logical_operation_id(&self) -> &LogicalOperationId {
        &self.logical_operation_id
    }

    /// Canonical envelope bytes. This method does not hash or authenticate them.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, QueueDeliveryError> {
        let mut bytes = b"MYC-DIR-QUEUE-COMMAND-BINDING\0\x01".to_vec();
        append_identifier(&mut bytes, self.broker_id.as_str())?;
        append_identifier(&mut bytes, self.destination_id.as_str())?;
        append_identifier(&mut bytes, self.message_key.as_str())?;
        bytes.extend_from_slice(&self.payload_digest);
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.decision_receipt_digest);
        append_identifier(&mut bytes, self.logical_operation_id.as_str())?;
        append_identifier(&mut bytes, self.effect_owner_id.as_str())?;
        bytes.extend_from_slice(&self.effect_fence_generation.to_be_bytes());
        append_identifier(&mut bytes, self.clock_profile_id.as_str())?;
        bytes.extend_from_slice(&self.issued_at_ms.to_be_bytes());
        bytes.extend_from_slice(&self.not_before_ms.to_be_bytes());
        bytes.extend_from_slice(&self.expires_at_ms.to_be_bytes());
        bytes.extend_from_slice(&self.max_clock_uncertainty_ms.to_be_bytes());
        Ok(bytes)
    }
}

/// Observation of one broker delivery. Sequence and delivery-count fields are
/// transport metadata; neither is an authorization epoch or an effect receipt.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QueueDeliveryObservation {
    broker_id: QueueIdentifier,
    destination_id: QueueIdentifier,
    message_key: QueueIdentifier,
    payload_digest: Digest,
    delivery_attempt_id: QueueIdentifier,
    broker_sequence: Option<u64>,
    redelivery_count: Option<u32>,
}

impl QueueDeliveryObservation {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        broker_id: QueueIdentifier,
        destination_id: QueueIdentifier,
        message_key: QueueIdentifier,
        payload_digest: Digest,
        delivery_attempt_id: QueueIdentifier,
        broker_sequence: Option<u64>,
        redelivery_count: Option<u32>,
    ) -> Result<Self, QueueDeliveryError> {
        if payload_digest == [0; 32] {
            return Err(QueueDeliveryError::ZeroDigest);
        }
        Ok(Self {
            broker_id,
            destination_id,
            message_key,
            payload_digest,
            delivery_attempt_id,
            broker_sequence,
            redelivery_count,
        })
    }

    /// Canonical transport-observation bytes; not proof of worker execution.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, QueueDeliveryError> {
        let mut bytes = b"MYC-DIR-QUEUE-DELIVERY-OBSERVATION\0\x01".to_vec();
        append_identifier(&mut bytes, self.broker_id.as_str())?;
        append_identifier(&mut bytes, self.destination_id.as_str())?;
        append_identifier(&mut bytes, self.message_key.as_str())?;
        bytes.extend_from_slice(&self.payload_digest);
        append_identifier(&mut bytes, self.delivery_attempt_id.as_str())?;
        match self.broker_sequence {
            Some(sequence) => {
                bytes.push(1);
                bytes.extend_from_slice(&sequence.to_be_bytes());
            }
            None => bytes.push(0),
        }
        match self.redelivery_count {
            Some(count) => {
                bytes.push(1);
                bytes.extend_from_slice(&count.to_be_bytes());
            }
            None => bytes.push(0),
        }
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueueDeliveryAssessment {
    /// The delivery matches and is time-valid. The effect owner must still
    /// authenticate the exact Allow and atomically check its current fence.
    EligibleForOwnerRevalidation,
    BrokerMismatch,
    DestinationMismatch,
    MessageKeyMismatch,
    PayloadMismatch,
    ClockProfileMismatch,
    ClockUncertaintyExceeded,
    NotYetValid,
    ExpiredOrBoundaryOverlap,
}

pub fn assess_queue_delivery(
    command: &QueueCommandBinding,
    delivery: &QueueDeliveryObservation,
    clock: &ClockWindow,
) -> QueueDeliveryAssessment {
    if delivery.broker_id != command.broker_id {
        return QueueDeliveryAssessment::BrokerMismatch;
    }
    if delivery.destination_id != command.destination_id {
        return QueueDeliveryAssessment::DestinationMismatch;
    }
    if delivery.message_key != command.message_key {
        return QueueDeliveryAssessment::MessageKeyMismatch;
    }
    if delivery.payload_digest != command.payload_digest {
        return QueueDeliveryAssessment::PayloadMismatch;
    }
    if clock.profile_id() != command.clock_profile_id.as_str() {
        return QueueDeliveryAssessment::ClockProfileMismatch;
    }
    if clock.uncertainty_ms() > command.max_clock_uncertainty_ms {
        return QueueDeliveryAssessment::ClockUncertaintyExceeded;
    }
    if clock.latest_ms() < command.not_before_ms {
        return QueueDeliveryAssessment::NotYetValid;
    }
    if clock.earliest_ms() < command.issued_at_ms
        || clock.earliest_ms() < command.not_before_ms
        || clock.latest_ms() >= command.expires_at_ms
    {
        return QueueDeliveryAssessment::ExpiredOrBoundaryOverlap;
    }

    QueueDeliveryAssessment::EligibleForOwnerRevalidation
}

/// A durable work disposition controls broker acknowledgement independently
/// from the underlying effect outcome. Unknown outcomes may be acknowledged
/// only after a durable reconciliation task has been committed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DurableQueueDisposition {
    Unstarted,
    AttemptUnknown,
    UnknownWithDurableReconciliationTask,
    Committed,
    ConfirmedNoEffect,
    RejectedBeforeAttempt,
}

pub fn broker_ack_permitted(disposition: DurableQueueDisposition) -> bool {
    matches!(
        disposition,
        DurableQueueDisposition::UnknownWithDurableReconciliationTask
            | DurableQueueDisposition::Committed
            | DurableQueueDisposition::ConfirmedNoEffect
            | DurableQueueDisposition::RejectedBeforeAttempt
    )
}

/// A fresh physical attempt is permitted only for a never-started operation
/// or after the exact earlier attempt has been reconciled as no-effect.
pub fn new_physical_attempt_permitted(disposition: DurableQueueDisposition) -> bool {
    matches!(
        disposition,
        DurableQueueDisposition::Unstarted | DurableQueueDisposition::ConfirmedNoEffect
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::receipts::{EffectOwnerId, LogicalOperationId};
    use crate::{ClockProfileId, ClockWindow};

    fn qid(value: &str) -> QueueIdentifier {
        QueueIdentifier::new(value).unwrap()
    }

    fn clock(earliest: u64, latest: u64) -> ClockWindow {
        ClockWindow::new(
            ClockProfileId::new("trusted-utc-v1").unwrap(),
            earliest,
            latest,
        )
        .unwrap()
    }

    fn command() -> QueueCommandBinding {
        QueueCommandBinding::new(
            qid("nats-prod-a"),
            qid("jobs.execute"),
            qid("job-381"),
            [1; 32],
            [2; 32],
            [3; 32],
            LogicalOperationId::new("invoice-post-381").unwrap(),
            EffectOwnerId::new("ledger-db-a").unwrap(),
            7,
            qid("trusted-utc-v1"),
            100,
            110,
            200,
            10,
        )
        .unwrap()
    }

    fn delivery() -> QueueDeliveryObservation {
        QueueDeliveryObservation::new(
            qid("nats-prod-a"),
            qid("jobs.execute"),
            qid("job-381"),
            [1; 32],
            qid("delivery-attempt-1"),
            Some(91),
            Some(1),
        )
        .unwrap()
    }

    #[test]
    fn exact_delivery_is_only_eligible_for_owner_revalidation() {
        assert_eq!(
            assess_queue_delivery(&command(), &delivery(), &clock(120, 125)),
            QueueDeliveryAssessment::EligibleForOwnerRevalidation
        );
    }

    #[test]
    fn broker_substitution_is_rejected() {
        let mut observed = delivery();
        observed.broker_id = qid("attacker-broker");
        assert_eq!(
            assess_queue_delivery(&command(), &observed, &clock(120, 125)),
            QueueDeliveryAssessment::BrokerMismatch
        );
    }

    #[test]
    fn destination_substitution_is_rejected() {
        let mut observed = delivery();
        observed.destination_id = qid("jobs.admin");
        assert_eq!(
            assess_queue_delivery(&command(), &observed, &clock(120, 125)),
            QueueDeliveryAssessment::DestinationMismatch
        );
    }

    #[test]
    fn message_key_substitution_is_rejected() {
        let mut observed = delivery();
        observed.message_key = qid("job-382");
        assert_eq!(
            assess_queue_delivery(&command(), &observed, &clock(120, 125)),
            QueueDeliveryAssessment::MessageKeyMismatch
        );
    }

    #[test]
    fn payload_substitution_is_rejected() {
        let mut observed = delivery();
        observed.payload_digest = [9; 32];
        assert_eq!(
            assess_queue_delivery(&command(), &observed, &clock(120, 125)),
            QueueDeliveryAssessment::PayloadMismatch
        );
    }

    #[test]
    fn worker_rejects_clock_profile_substitution() {
        assert_eq!(
            assess_queue_delivery(&command(), &delivery(), &ClockWindow::new(
                ClockProfileId::new("local-wall-clock").unwrap(), 120, 125
            ).unwrap()),
            QueueDeliveryAssessment::ClockProfileMismatch
        );
    }

    #[test]
    fn worker_rejects_excessive_clock_uncertainty() {
        assert_eq!(
            assess_queue_delivery(&command(), &delivery(), &clock(120, 140)),
            QueueDeliveryAssessment::ClockUncertaintyExceeded
        );
    }

    #[test]
    fn worker_rejects_delivery_before_not_before() {
        assert_eq!(
            assess_queue_delivery(&command(), &delivery(), &clock(100, 105)),
            QueueDeliveryAssessment::NotYetValid
        );
    }

    #[test]
    fn worker_rejects_expiry_overlap() {
        assert_eq!(
            assess_queue_delivery(&command(), &delivery(), &clock(195, 205)),
            QueueDeliveryAssessment::ExpiredOrBoundaryOverlap
        );
    }

    #[test]
    fn broker_redelivery_count_does_not_change_authorization_assessment() {
        let mut first = delivery();
        let mut redelivered = delivery();
        first.redelivery_count = Some(1);
        redelivered.redelivery_count = Some(99);
        assert_eq!(
            assess_queue_delivery(&command(), &first, &clock(120, 125)),
            assess_queue_delivery(&command(), &redelivered, &clock(120, 125))
        );
    }

    #[test]
    fn unknown_outcome_cannot_be_acknowledged_or_retried() {
        assert!(!broker_ack_permitted(DurableQueueDisposition::AttemptUnknown));
        assert!(!new_physical_attempt_permitted(DurableQueueDisposition::AttemptUnknown));
    }

    #[test]
    fn unknown_can_be_acknowledged_only_after_durable_reconciliation_handoff() {
        assert!(broker_ack_permitted(
            DurableQueueDisposition::UnknownWithDurableReconciliationTask
        ));
        assert!(!new_physical_attempt_permitted(
            DurableQueueDisposition::UnknownWithDurableReconciliationTask
        ));
    }

    #[test]
    fn retry_requires_confirmed_no_effect_and_commit_is_terminal() {
        assert!(new_physical_attempt_permitted(
            DurableQueueDisposition::ConfirmedNoEffect
        ));
        assert!(!new_physical_attempt_permitted(DurableQueueDisposition::Committed));
        assert!(broker_ack_permitted(DurableQueueDisposition::Committed));
        assert!(broker_ack_permitted(DurableQueueDisposition::RejectedBeforeAttempt));
    }

    #[test]
    fn canonical_bytes_bind_operation_owner_and_generation() {
        let base = command();
        let other = QueueCommandBinding::new(
            qid("nats-prod-a"),
            qid("jobs.execute"),
            qid("job-381"),
            [1; 32],
            [2; 32],
            [3; 32],
            LogicalOperationId::new("invoice-post-381").unwrap(),
            EffectOwnerId::new("ledger-db-b").unwrap(),
            8,
            qid("trusted-utc-v1"),
            100, 110, 200, 10
        ).unwrap();
        assert_ne!(base.canonical_bytes().unwrap(), other.canonical_bytes().unwrap());
    }
}
