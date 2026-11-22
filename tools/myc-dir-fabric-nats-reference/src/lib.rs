//! Dependency-free safety contract for NATS JetStream delivery.
//!
//! This is a reference model, not a NATS client. Broker publication and
//! acknowledgement evidence never authorize or prove a protected effect.

pub type Digest = [u8; 32];

const MAX_IDENTIFIER_BYTES: usize = 512;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NatsContractError {
    InvalidIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    ZeroOwnerEpoch,
    ZeroResourceGeneration,
    ZeroSequence,
    ZeroDeliveryCount,
    WrongStream,
    UnsupportedAckPolicy,
    ZeroAckWait,
    ZeroMaxAckPending,
    ZeroMaxDeliver,
    ZeroDuplicateWindow,
}

fn check_identifier(value: &str) -> Result<(), NatsContractError> {
    if value.trim().is_empty() {
        return Err(NatsContractError::InvalidIdentifier);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(NatsContractError::IdentifierTooLong);
    }
    Ok(())
}

fn digest_is_nonzero(value: &Digest) -> bool {
    value.iter().any(|byte| *byte != 0)
}

fn append_identifier(bytes: &mut Vec<u8>, value: &str) -> Result<(), NatsContractError> {
    check_identifier(value)?;
    let length = u16::try_from(value.len()).map_err(|_| NatsContractError::IdentifierTooLong)?;
    bytes.extend_from_slice(&length.to_be_bytes());
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NatsCommandBindingInput {
    pub account_id: String,
    pub stream_name: String,
    pub subject: String,
    pub consumer_name: String,
    /// Application supplied Nats-Msg-Id. This is transport identity only.
    pub nats_message_id: String,
    pub payload_digest: Digest,
    pub request_digest: Digest,
    pub decision_receipt_digest: Digest,
    pub logical_operation_id: String,
    pub idempotency_digest: Digest,
    pub effect_owner_id: String,
    pub owner_epoch: u64,
    pub resource_generation: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NatsCommandBinding {
    account_id: String,
    stream_name: String,
    subject: String,
    consumer_name: String,
    nats_message_id: String,
    payload_digest: Digest,
    request_digest: Digest,
    decision_receipt_digest: Digest,
    logical_operation_id: String,
    idempotency_digest: Digest,
    effect_owner_id: String,
    owner_epoch: u64,
    resource_generation: u64,
}

impl NatsCommandBinding {
    pub fn new(input: NatsCommandBindingInput) -> Result<Self, NatsContractError> {
        for value in [
            input.account_id.as_str(),
            input.stream_name.as_str(),
            input.subject.as_str(),
            input.consumer_name.as_str(),
            input.nats_message_id.as_str(),
            input.logical_operation_id.as_str(),
            input.effect_owner_id.as_str(),
        ] {
            check_identifier(value)?;
        }
        if [
            input.payload_digest,
            input.request_digest,
            input.decision_receipt_digest,
            input.idempotency_digest,
        ]
        .iter()
        .any(|digest| !digest_is_nonzero(digest))
        {
            return Err(NatsContractError::ZeroDigest);
        }
        if input.owner_epoch == 0 {
            return Err(NatsContractError::ZeroOwnerEpoch);
        }
        if input.resource_generation == 0 {
            return Err(NatsContractError::ZeroResourceGeneration);
        }

        Ok(Self {
            account_id: input.account_id,
            stream_name: input.stream_name,
            subject: input.subject,
            consumer_name: input.consumer_name,
            nats_message_id: input.nats_message_id,
            payload_digest: input.payload_digest,
            request_digest: input.request_digest,
            decision_receipt_digest: input.decision_receipt_digest,
            logical_operation_id: input.logical_operation_id,
            idempotency_digest: input.idempotency_digest,
            effect_owner_id: input.effect_owner_id,
            owner_epoch: input.owner_epoch,
            resource_generation: input.resource_generation,
        })
    }

    pub fn account_id(&self) -> &str {
        &self.account_id
    }
    pub fn stream_name(&self) -> &str {
        &self.stream_name
    }
    pub fn subject(&self) -> &str {
        &self.subject
    }
    pub fn consumer_name(&self) -> &str {
        &self.consumer_name
    }
    pub fn nats_message_id(&self) -> &str {
        &self.nats_message_id
    }
    pub fn logical_operation_id(&self) -> &str {
        &self.logical_operation_id
    }
    pub fn idempotency_digest(&self) -> &Digest {
        &self.idempotency_digest
    }
    pub fn effect_owner_id(&self) -> &str {
        &self.effect_owner_id
    }
    pub fn payload_digest(&self) -> &Digest {
        &self.payload_digest
    }
    pub fn owner_position(&self) -> (u64, u64) {
        (self.owner_epoch, self.resource_generation)
    }

    /// Versioned, length-prefixed canonical bytes. Hashing/authentication is
    /// intentionally external to this dependency-free reference crate.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, NatsContractError> {
        let mut bytes = b"MYC-DIR-NATS-JETSTREAM-COMMAND\0\x01".to_vec();
        for value in [
            self.account_id.as_str(),
            self.stream_name.as_str(),
            self.subject.as_str(),
            self.consumer_name.as_str(),
            self.nats_message_id.as_str(),
        ] {
            append_identifier(&mut bytes, value)?;
        }
        bytes.extend_from_slice(&self.payload_digest);
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.decision_receipt_digest);
        append_identifier(&mut bytes, &self.logical_operation_id)?;
        bytes.extend_from_slice(&self.idempotency_digest);
        append_identifier(&mut bytes, &self.effect_owner_id)?;
        bytes.extend_from_slice(&self.owner_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.resource_generation.to_be_bytes());
        Ok(bytes)
    }

    /// The same protected logical effect may be republished with a new
    /// Nats-Msg-Id, renewed decision receipt or newer owner position. The
    /// immutable operation/request/effect-owner/idempotency identity remains
    /// stable; authority and fence freshness are revalidated separately.
    pub fn same_logical_effect(&self, other: &Self) -> bool {
        self.logical_operation_id == other.logical_operation_id
            && self.idempotency_digest == other.idempotency_digest
            && self.effect_owner_id == other.effect_owner_id
            && self.payload_digest == other.payload_digest
            && self.request_digest == other.request_digest
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JetStreamDeliveryObservation {
    pub account_id: String,
    pub stream_name: String,
    pub subject: String,
    pub consumer_name: String,
    pub nats_message_id: String,
    pub payload_digest: Digest,
    pub stream_sequence: u64,
    pub consumer_sequence: u64,
    pub delivery_count: u32,
}

impl JetStreamDeliveryObservation {
    pub fn validate(&self) -> Result<(), NatsContractError> {
        for value in [
            self.account_id.as_str(),
            self.stream_name.as_str(),
            self.subject.as_str(),
            self.consumer_name.as_str(),
            self.nats_message_id.as_str(),
        ] {
            check_identifier(value)?;
        }
        if !digest_is_nonzero(&self.payload_digest) {
            return Err(NatsContractError::ZeroDigest);
        }
        if self.stream_sequence == 0 || self.consumer_sequence == 0 {
            return Err(NatsContractError::ZeroSequence);
        }
        if self.delivery_count == 0 {
            return Err(NatsContractError::ZeroDeliveryCount);
        }
        Ok(())
    }

    pub fn is_redelivery(&self) -> bool {
        self.delivery_count > 1
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeliveryAssessment {
    EligibleForOwnerRevalidation,
    RejectAccount,
    RejectStream,
    RejectSubject,
    RejectConsumer,
    RejectMessageId,
    RejectPayload,
    RejectInvalidObservation,
}

/// Matching transport metadata permits only owner revalidation, never direct
/// execution.
pub fn assess_delivery(
    binding: &NatsCommandBinding,
    observation: &JetStreamDeliveryObservation,
) -> DeliveryAssessment {
    if observation.validate().is_err() {
        return DeliveryAssessment::RejectInvalidObservation;
    }
    if observation.account_id != binding.account_id {
        return DeliveryAssessment::RejectAccount;
    }
    if observation.stream_name != binding.stream_name {
        return DeliveryAssessment::RejectStream;
    }
    if observation.subject != binding.subject {
        return DeliveryAssessment::RejectSubject;
    }
    if observation.consumer_name != binding.consumer_name {
        return DeliveryAssessment::RejectConsumer;
    }
    if observation.nats_message_id != binding.nats_message_id {
        return DeliveryAssessment::RejectMessageId;
    }
    if observation.payload_digest != binding.payload_digest {
        return DeliveryAssessment::RejectPayload;
    }
    DeliveryAssessment::EligibleForOwnerRevalidation
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AckPolicy {
    Explicit,
    All,
    None,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct JetStreamStreamProfile {
    pub duplicate_window_ms: u64,
}

impl JetStreamStreamProfile {
    pub fn validate(&self) -> Result<(), NatsContractError> {
        if self.duplicate_window_ms == 0 {
            return Err(NatsContractError::ZeroDuplicateWindow);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct JetStreamConsumerProfile {
    pub ack_policy: AckPolicy,
    pub ack_wait_ms: u64,
    pub max_ack_pending: u64,
    pub max_deliver: Option<u32>,
}

impl JetStreamConsumerProfile {
    pub fn validate(&self) -> Result<(), NatsContractError> {
        if self.ack_policy != AckPolicy::Explicit {
            return Err(NatsContractError::UnsupportedAckPolicy);
        }
        if self.ack_wait_ms == 0 {
            return Err(NatsContractError::ZeroAckWait);
        }
        if self.max_ack_pending == 0 {
            return Err(NatsContractError::ZeroMaxAckPending);
        }
        if self.max_deliver == Some(0) {
            return Err(NatsContractError::ZeroMaxDeliver);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AckMode {
    Ack,
    AckSync,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DurableOwnerDisposition {
    EffectCommitted {
        logical_operation_id: String,
        effect_owner_id: String,
        idempotency_digest: Digest,
        effect_receipt_digest: Digest,
    },
    TerminalRejected {
        logical_operation_id: String,
        effect_owner_id: String,
        idempotency_digest: Digest,
        rejection_receipt_digest: Digest,
    },
    TerminalNoEffect {
        logical_operation_id: String,
        effect_owner_id: String,
        idempotency_digest: Digest,
        attempt_id: String,
        terminal_at_effect_owner: bool,
        outcome_receipt_digest: Digest,
    },
    ReconciliationHandoffDurable {
        logical_operation_id: String,
        effect_owner_id: String,
        idempotency_digest: Digest,
        attempt_id: String,
        handoff_receipt_digest: Digest,
    },
    InFlight,
    Unknown,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AckPlan {
    DoNotAcknowledge,
    Acknowledge {
        mode: AckMode,
        disposition_digest: Digest,
    },
}

pub fn plan_acknowledgement(
    binding: &NatsCommandBinding,
    disposition: &DurableOwnerDisposition,
    mode: AckMode,
) -> AckPlan {
    let (operation_id, effect_owner_id, idempotency_digest, receipt_digest) = match disposition {
        DurableOwnerDisposition::EffectCommitted {
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            effect_receipt_digest,
        } => (
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            effect_receipt_digest,
        ),
        DurableOwnerDisposition::TerminalRejected {
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            rejection_receipt_digest,
        } => (
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            rejection_receipt_digest,
        ),
        DurableOwnerDisposition::TerminalNoEffect {
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            attempt_id,
            terminal_at_effect_owner,
            outcome_receipt_digest,
        } => {
            if check_identifier(attempt_id).is_err() || !terminal_at_effect_owner {
                return AckPlan::DoNotAcknowledge;
            }
            (
                logical_operation_id,
                effect_owner_id,
                idempotency_digest,
                outcome_receipt_digest,
            )
        }
        DurableOwnerDisposition::ReconciliationHandoffDurable {
            logical_operation_id,
            effect_owner_id,
            idempotency_digest,
            attempt_id,
            handoff_receipt_digest,
        } => {
            if check_identifier(attempt_id).is_err() {
                return AckPlan::DoNotAcknowledge;
            }
            (
                logical_operation_id,
                effect_owner_id,
                idempotency_digest,
                handoff_receipt_digest,
            )
        }
        DurableOwnerDisposition::InFlight | DurableOwnerDisposition::Unknown => {
            return AckPlan::DoNotAcknowledge
        }
    };

    if operation_id != &binding.logical_operation_id
        || effect_owner_id != &binding.effect_owner_id
        || idempotency_digest != &binding.idempotency_digest
        || !digest_is_nonzero(receipt_digest)
    {
        return AckPlan::DoNotAcknowledge;
    }
    AckPlan::Acknowledge { mode, disposition_digest: *digest }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JetStreamAckConfirmation {
    pub stream_name: String,
    pub consumer_name: String,
    pub stream_sequence: u64,
    pub ack_received_by_broker: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BrokerAckAssessment {
    Confirmed,
    NotConfirmed,
    WrongStream,
    WrongConsumer,
    WrongSequence,
    InvalidConfirmation,
}

/// Confirms only the broker's receipt of an ACK for the exact stream,
/// consumer and stream sequence; this is not evidence of an external effect.
pub fn assess_ack_confirmation(
    expected_stream: &str,
    expected_consumer: &str,
    expected_stream_sequence: u64,
    confirmation: &JetStreamAckConfirmation,
) -> BrokerAckAssessment {
    if check_identifier(expected_stream).is_err()
        || check_identifier(expected_consumer).is_err()
        || check_identifier(&confirmation.stream_name).is_err()
        || check_identifier(&confirmation.consumer_name).is_err()
        || confirmation.stream_sequence == 0
        || expected_stream_sequence == 0
    {
        return BrokerAckAssessment::InvalidConfirmation;
    }
    if confirmation.stream_name != expected_stream {
        return BrokerAckAssessment::WrongStream;
    }
    if confirmation.consumer_name != expected_consumer {
        return BrokerAckAssessment::WrongConsumer;
    }
    if confirmation.stream_sequence != expected_stream_sequence {
        return BrokerAckAssessment::WrongSequence;
    }
    if confirmation.ack_received_by_broker {
        BrokerAckAssessment::Confirmed
    } else {
        BrokerAckAssessment::NotConfirmed
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JetStreamPublishAck {
    pub stream_name: String,
    pub stream_sequence: u64,
    pub duplicate: bool,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PublishAssessment {
    StoredByBroker,
    BrokerDeduplicated,
}

pub fn assess_publish_ack(
    expected_stream: &str,
    ack: &JetStreamPublishAck,
) -> Result<PublishAssessment, NatsContractError> {
    check_identifier(expected_stream)?;
    check_identifier(&ack.stream_name)?;
    if ack.stream_name != expected_stream {
        return Err(NatsContractError::WrongStream);
    }
    if ack.stream_sequence == 0 {
        return Err(NatsContractError::ZeroSequence);
    }
    Ok(if ack.duplicate {
        PublishAssessment::BrokerDeduplicated
    } else {
        PublishAssessment::StoredByBroker
    })
}

pub fn is_consistent_redelivery(
    previous: &JetStreamDeliveryObservation,
    next: &JetStreamDeliveryObservation,
) -> bool {
    previous.validate().is_ok()
        && next.validate().is_ok()
        && previous.account_id == next.account_id
        && previous.stream_name == next.stream_name
        && previous.subject == next.subject
        && previous.consumer_name == next.consumer_name
        && previous.nats_message_id == next.nats_message_id
        && previous.payload_digest == next.payload_digest
        && previous.stream_sequence == next.stream_sequence
        && next.delivery_count > previous.delivery_count
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest {
        [byte; 32]
    }

    fn binding(message_id: &str) -> NatsCommandBinding {
        NatsCommandBinding::new(NatsCommandBindingInput {
            account_id: "account-a".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.command.resource-a".into(),
            consumer_name: "effect-workers".into(),
            nats_message_id: message_id.into(),
            payload_digest: digest(1),
            request_digest: digest(2),
            decision_receipt_digest: digest(3),
            logical_operation_id: "logical-op-a".into(),
            idempotency_digest: digest(4),
            effect_owner_id: "postgres-owner-a".into(),
            owner_epoch: 3,
            resource_generation: 9,
        })
        .unwrap()
    }

    fn delivery(count: u32) -> JetStreamDeliveryObservation {
        JetStreamDeliveryObservation {
            account_id: "account-a".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.command.resource-a".into(),
            consumer_name: "effect-workers".into(),
            nats_message_id: "publish-123".into(),
            payload_digest: digest(1),
            stream_sequence: 100,
            consumer_sequence: 200 + u64::from(count),
            delivery_count: count,
        }
    }

    fn consumer_profile() -> JetStreamConsumerProfile {
        JetStreamConsumerProfile {
            ack_policy: AckPolicy::Explicit,
            ack_wait_ms: 30_000,
            max_ack_pending: 256,
            max_deliver: Some(20),
        }
    }

    #[test]
    fn exact_delivery_is_only_eligible_for_owner_revalidation() {
        assert_eq!(
            assess_delivery(&binding("publish-123"), &delivery(1)),
            DeliveryAssessment::EligibleForOwnerRevalidation
        );
    }
    #[test]
    fn account_substitution_is_rejected() {
        let mut item = delivery(1);
        item.account_id = "other".into();
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectAccount
        );
    }
    #[test]
    fn stream_substitution_is_rejected() {
        let mut item = delivery(1);
        item.stream_name = "OTHER".into();
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectStream
        );
    }
    #[test]
    fn subject_substitution_is_rejected() {
        let mut item = delivery(1);
        item.subject = "mycelix.admin".into();
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectSubject
        );
    }
    #[test]
    fn consumer_substitution_is_rejected() {
        let mut item = delivery(1);
        item.consumer_name = "admin".into();
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectConsumer
        );
    }
    #[test]
    fn message_id_substitution_is_rejected() {
        let mut item = delivery(1);
        item.nats_message_id = "other".into();
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectMessageId
        );
    }
    #[test]
    fn payload_substitution_is_rejected() {
        let mut item = delivery(1);
        item.payload_digest = digest(99);
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectPayload
        );
    }
    #[test]
    fn invalid_delivery_sequence_is_rejected() {
        let mut item = delivery(1);
        item.stream_sequence = 0;
        assert_eq!(
            assess_delivery(&binding("publish-123"), &item),
            DeliveryAssessment::RejectInvalidObservation
        );
    }
    #[test]
    fn explicit_ack_policy_is_required() {
        assert_eq!(consumer_profile().validate(), Ok(()));
        let bad = JetStreamConsumerProfile {
            ack_policy: AckPolicy::All,
            ..consumer_profile()
        };
        assert_eq!(
            bad.validate(),
            Err(NatsContractError::UnsupportedAckPolicy)
        );
    }
    #[test]
    fn stream_duplicate_window_is_pinned_separately() {
        assert_eq!(
            JetStreamStreamProfile {
                duplicate_window_ms: 120_000
            }
            .validate(),
            Ok(())
        );
        assert_eq!(
            JetStreamStreamProfile {
                duplicate_window_ms: 0
            }
            .validate(),
            Err(NatsContractError::ZeroDuplicateWindow)
        );
    }
    #[test]
    fn unknown_and_inflight_outcomes_cannot_be_acked() {
        let b = binding("publish-123");
        assert_eq!(
            plan_acknowledgement(&b, &DurableOwnerDisposition::Unknown, AckMode::AckSync),
            AckPlan::DoNotAcknowledge
        );
        assert_eq!(
            plan_acknowledgement(&b, &DurableOwnerDisposition::InFlight, AckMode::Ack),
            AckPlan::DoNotAcknowledge
        );
    }
    #[test]
    fn terminal_no_effect_requires_owner_terminal_evidence() {
        let b = binding("publish-123");
        let disposition = DurableOwnerDisposition::TerminalNoEffect {
            logical_operation_id: "logical-op-a".into(),
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: digest(4),
            attempt_id: "attempt-1".into(),
            terminal_at_effect_owner: false,
            outcome_receipt_digest: digest(5),
        };
        assert_eq!(
            plan_acknowledgement(&b, &disposition, AckMode::Ack),
            AckPlan::DoNotAcknowledge
        );
    }
    #[test]
    fn durable_reconciliation_handoff_can_be_acked() {
        let b = binding("publish-123");
        let disposition = DurableOwnerDisposition::ReconciliationHandoffDurable {
            logical_operation_id: "logical-op-a".into(),
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: digest(4),
            attempt_id: "attempt-1".into(),
            handoff_receipt_digest: digest(5),
        };
        assert!(matches!(
            plan_acknowledgement(&b, &disposition, AckMode::AckSync),
            AckPlan::Acknowledge { .. }
        ));
    }
    #[test]
    fn ack_confirmation_requires_exact_stream_consumer_and_sequence() {
        let confirmation = JetStreamAckConfirmation {
            stream_name: "MYCELIX_COMMANDS".into(),
            consumer_name: "effect-workers".into(),
            stream_sequence: 100,
            ack_received_by_broker: true,
        };
        assert_eq!(
            assess_ack_confirmation("MYCELIX_COMMANDS", "effect-workers", 100, &confirmation),
            BrokerAckAssessment::Confirmed
        );
        assert_eq!(
            assess_ack_confirmation("MYCELIX_COMMANDS", "other", 100, &confirmation),
            BrokerAckAssessment::WrongConsumer
        );
        assert_eq!(
            assess_ack_confirmation("MYCELIX_COMMANDS", "effect-workers", 101, &confirmation),
            BrokerAckAssessment::WrongSequence
        );
    }
    #[test]
    fn puback_is_transport_only_and_duplicate_is_explicit() {
        assert_eq!(
            assess_publish_ack(
                "MYCELIX_COMMANDS",
                &JetStreamPublishAck {
                    stream_name: "MYCELIX_COMMANDS".into(),
                    stream_sequence: 100,
                    duplicate: true,
                }
            ),
            Ok(PublishAssessment::BrokerDeduplicated)
        );
    }
    #[test]
    fn republish_with_new_message_id_preserves_logical_effect_identity() {
        assert!(binding("publish-123").same_logical_effect(&binding("publish-456")));
    }
    #[test]
    fn same_logical_effect_survives_owner_recovery_and_new_decision() {
        let original = binding("publish-123");
        let mut recovered_input = NatsCommandBindingInput {
            account_id: "account-a".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.command.resource-a".into(),
            consumer_name: "effect-workers".into(),
            nats_message_id: "publish-after-recovery".into(),
            payload_digest: digest(1),
            request_digest: digest(2),
            decision_receipt_digest: digest(99),
            logical_operation_id: "logical-op-a".into(),
            idempotency_digest: digest(4),
            effect_owner_id: "postgres-owner-a".into(),
            owner_epoch: 4,
            resource_generation: 1,
        };
        // A higher owner epoch with a resource generation reset is not a
        // valid recovery position; the identity comparison intentionally does
        // not validate recovery. The owner-state recovery gate does that.
        recovered_input.resource_generation = 19;
        let recovered = NatsCommandBinding::new(recovered_input).unwrap();
        assert!(original.same_logical_effect(&recovered));
    }

    #[test]
    fn acknowledgement_disposition_must_match_exact_effect_owner() {
        let b = binding("publish-123");
        let disposition = DurableOwnerDisposition::EffectCommitted {
            logical_operation_id: "logical-op-a".into(),
            effect_owner_id: "different-owner".into(),
            idempotency_digest: digest(4),
            effect_receipt_digest: digest(5),
        };
        assert_eq!(
            plan_acknowledgement(&b, &disposition, AckMode::Ack),
            AckPlan::DoNotAcknowledge
        );
    }

    #[test]
    fn acknowledgement_disposition_must_match_idempotency_commitment() {
        let b = binding("publish-123");
        let disposition = DurableOwnerDisposition::EffectCommitted {
            logical_operation_id: "logical-op-a".into(),
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: digest(99),
            effect_receipt_digest: digest(5),
        };
        assert_eq!(plan_acknowledgement(&b, &disposition, AckMode::Ack), AckPlan::DoNotAcknowledge);
    }

    #[test]
    fn canonical_binding_changes_when_message_id_changes() {
        assert_ne!(
            binding("publish-123").canonical_bytes().unwrap(),
            binding("publish-456").canonical_bytes().unwrap()
        );
    }
    #[test]
    fn redelivery_preserves_stream_message_and_increments_count() {
        assert!(is_consistent_redelivery(&delivery(1), &delivery(2)));
        let mut changed = delivery(2);
        changed.stream_sequence += 1;
        assert!(!is_consistent_redelivery(&delivery(1), &changed));
    }
}
