//! Pinned async-nats JetStream adapter for Mycelix Directory Fabric.
//!
//! This crate is the concrete transport adapter. It does not implement an
//! effect owner, authorize a command, or claim that broker ACKs prove effects.
//! Every received command must still be revalidated by the worker and then by
//! the resource owner at the atomic mutation boundary.

use std::fmt;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::Duration;

use async_nats::jetstream::{
    self,
    consumer::{self, AckPolicy},
    context::traits::Publisher,
    message::{Acker, PublishMessage},
    stream::{DiscardPolicy, RetentionPolicy, StorageType},
    AckKind,
};
use bytes::Bytes;
use futures_util::StreamExt;
use myc_dir_fabric_nats_reference::{
    assess_ack_confirmation, assess_delivery, plan_acknowledgement, AckMode, AckPlan,
    BrokerAckAssessment, DurableOwnerDisposition, JetStreamAckConfirmation,
    JetStreamDeliveryObservation, NatsCommandBinding, NatsCommandBindingInput,
    NatsContractError,
};
use sha2::{Digest as ShaDigest, Sha256};

const WIRE_VERSION: &str = "1";
const HEADER_VERSION: &str = "Mycelix-Wire-Version";
const HEADER_PUBLICATION_ID: &str = "Mycelix-Publication-Id";
const HEADER_LOGICAL_OPERATION: &str = "Mycelix-Logical-Operation-Id";
const HEADER_REQUEST_DIGEST: &str = "Mycelix-Request-SHA256";
const HEADER_DECISION_DIGEST: &str = "Mycelix-Decision-SHA256";
const HEADER_IDEMPOTENCY_DIGEST: &str = "Mycelix-Idempotency-SHA256";
const HEADER_EFFECT_OWNER: &str = "Mycelix-Effect-Owner";
const HEADER_OWNER_EPOCH: &str = "Mycelix-Owner-Epoch";
const HEADER_RESOURCE_GENERATION: &str = "Mycelix-Resource-Generation";
const HEADER_PAYLOAD_DIGEST: &str = "Mycelix-Payload-SHA256";
const HEADER_BINDING_DIGEST: &str = "Mycelix-Binding-SHA256";

pub type AdapterResult<T> = std::result::Result<T, AdapterError>;

#[derive(Debug)]
pub enum AdapterError {
    InvalidConfiguration(&'static str),
    NotInitialized,
    Nats(String),
    MissingHeader(&'static str),
    InvalidHeader(&'static str),
    InvalidWireVersion,
    PayloadDigestMismatch,
    BindingDigestMismatch,
    Binding(NatsContractError),
    AccountScopeMismatch,
    JetStreamDomainMismatch,
    AccountHashMismatch,
    StreamConfigurationMismatch(&'static str),
    ConsumerConfigurationMismatch(&'static str),
    AckNotPermitted,
    OwnerDispositionVerification(String),
    VerifiedDispositionBindingMismatch,
}

impl fmt::Display for AdapterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(field) => write!(f, "invalid adapter configuration: {field}"),
            Self::NotInitialized => write!(f, "JetStream stream/consumer configuration has not been verified"),
            Self::Nats(error) => write!(f, "NATS JetStream error: {error}"),
            Self::MissingHeader(header) => write!(f, "missing required command header {header}"),
            Self::InvalidHeader(header) => write!(f, "invalid command header {header}"),
            Self::InvalidWireVersion => write!(f, "unsupported Mycelix command wire version"),
            Self::PayloadDigestMismatch => write!(f, "command payload SHA-256 does not match its binding"),
            Self::BindingDigestMismatch => write!(f, "canonical command binding digest does not match its header"),
            Self::Binding(error) => write!(f, "command binding rejected: {error:?}"),
            Self::AccountScopeMismatch => write!(f, "JetStream delivery does not match the configured account, stream, subject or consumer"),
            Self::JetStreamDomainMismatch => write!(f, "JetStream domain differs from the configured domain"),
            Self::AccountHashMismatch => write!(f, "JetStream account hash differs from the configured account hash"),
            Self::StreamConfigurationMismatch(field) => write!(f, "JetStream stream configuration mismatch: {field}"),
            Self::ConsumerConfigurationMismatch(field) => write!(f, "JetStream consumer configuration mismatch: {field}"),
            Self::AckNotPermitted => write!(f, "Mycelix durable owner disposition does not permit ACK"),
            Self::OwnerDispositionVerification(error) => write!(f, "effect-owner disposition verification failed: {error}"),
            Self::VerifiedDispositionBindingMismatch => write!(f, "verified owner disposition belongs to a different command binding"),
        }
    }
}

impl std::error::Error for AdapterError {}

fn sha256(bytes: &[u8]) -> [u8; 32] {
    Sha256::digest(bytes).into()
}

fn hex_encode(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

fn hex_decode_32(value: &str, header: &'static str) -> AdapterResult<[u8; 32]> {
    if value.len() != 64 {
        return Err(AdapterError::InvalidHeader(header));
    }
    let mut output = [0_u8; 32];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        let high = (pair[0] as char)
            .to_digit(16)
            .ok_or(AdapterError::InvalidHeader(header))?;
        let low = (pair[1] as char)
            .to_digit(16)
            .ok_or(AdapterError::InvalidHeader(header))?;
        output[index] = ((high << 4) | low) as u8;
    }
    Ok(output)
}

fn header_text(
    headers: &async_nats::HeaderMap,
    name: &'static str,
) -> AdapterResult<String> {
    let mut values = headers.get_all(name);
    let first = values.next().ok_or(AdapterError::MissingHeader(name))?;
    if values.next().is_some() {
        return Err(AdapterError::InvalidHeader(name));
    }
    Ok(first.as_ref().to_owned())
}

fn header_digest(
    headers: &async_nats::HeaderMap,
    name: &'static str,
) -> AdapterResult<[u8; 32]> {
    let value = header_text(headers, name)?;
    hex_decode_32(&value, name)
}

fn parse_u64_header(
    headers: &async_nats::HeaderMap,
    name: &'static str,
) -> AdapterResult<u64> {
    header_text(headers, name)?
        .parse::<u64>()
        .map_err(|_| AdapterError::InvalidHeader(name))
}

#[derive(Clone, Debug)]
pub struct JetStreamAdapterConfig {
    /// Stable application label for the NATS account authenticated by this
    /// connection. Production deployments should pin expected_account_hash.
    pub account_id: String,
    pub stream_name: String,
    pub subject: String,
    pub durable_consumer: String,
    /// Exact stream-level duplicate tracking window.
    pub duplicate_window: Duration,
    pub expected_stream_replicas: usize,
    /// Exact durable stream retention policy. Limits keeps acknowledged
    /// messages available for independent audit/replay until configured limits.
    pub expected_retention: RetentionPolicy,
    /// Exact finite capacity. DiscardNew makes saturation visible to publishers
    /// instead of deleting old unprocessed work.
    pub max_messages: i64,
    pub max_bytes: i64,
    /// Exact maximum message age; the decision's own validity is still checked
    /// independently at execution time.
    pub max_age: Duration,
    /// Optional JetStream domain pin. If configured, every delivery must
    /// expose the same domain in its ADR-15 acknowledgement metadata.
    pub expected_domain: Option<String>,
    /// Optional account-hash pin from the authenticated NATS account. When
    /// supplied, delivery metadata must include and match it.
    pub expected_account_hash: Option<String>,
    pub ack_wait: Duration,
    pub max_ack_pending: i64,
    pub max_deliver: i64,
    pub fetch_timeout: Duration,
}

impl JetStreamAdapterConfig {
    pub fn validate(&self) -> AdapterResult<()> {
        for (field, value) in [
            ("account_id", self.account_id.as_str()),
            ("stream_name", self.stream_name.as_str()),
            ("subject", self.subject.as_str()),
            ("durable_consumer", self.durable_consumer.as_str()),
        ] {
            if value.trim().is_empty() || value.len() > 512 {
                return Err(AdapterError::InvalidConfiguration(field));
            }
        }
        if self.duplicate_window.is_zero() {
            return Err(AdapterError::InvalidConfiguration("duplicate_window"));
        }
        if self.expected_stream_replicas == 0 || self.expected_stream_replicas > 5 {
            return Err(AdapterError::InvalidConfiguration("expected_stream_replicas"));
        }
        if self.max_messages <= 0 {
            return Err(AdapterError::InvalidConfiguration("max_messages"));
        }
        if self.max_bytes <= 0 {
            return Err(AdapterError::InvalidConfiguration("max_bytes"));
        }
        if self.max_age.is_zero() {
            return Err(AdapterError::InvalidConfiguration("max_age"));
        }
        if self.ack_wait.is_zero() {
            return Err(AdapterError::InvalidConfiguration("ack_wait"));
        }
        if self.max_ack_pending <= 0 {
            return Err(AdapterError::InvalidConfiguration("max_ack_pending"));
        }
        if self.max_deliver <= 0 {
            return Err(AdapterError::InvalidConfiguration("max_deliver"));
        }
        if self.fetch_timeout.is_zero() {
            return Err(AdapterError::InvalidConfiguration("fetch_timeout"));
        }
        if self.expected_domain.as_deref().is_some_and(str::is_empty) {
            return Err(AdapterError::InvalidConfiguration("expected_domain"));
        }
        if self
            .expected_account_hash
            .as_deref()
            .is_some_and(str::is_empty)
        {
            return Err(AdapterError::InvalidConfiguration("expected_account_hash"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PublishEvidence {
    pub stream: String,
    pub sequence: u64,
    pub duplicate: bool,
    pub domain: String,
}

/// External trust boundary for durable effect-owner receipts. Production
/// implementations must authenticate the owner signature/proof and compare
/// the receipt's operation, owner, idempotency, scope and terminal disposition
/// with this exact immutable command binding.
pub trait OwnerDispositionVerifier {
    fn verify_durable_disposition(
        &self,
        binding: &NatsCommandBinding,
        disposition: &DurableOwnerDisposition,
    ) -> Result<(), String>;
}

/// Proof capability created only after an OwnerDispositionVerifier accepts
/// the exact command binding and durable owner disposition.
pub struct VerifiedOwnerDisposition {
    binding_digest: [u8; 32],
    disposition: DurableOwnerDisposition,
}

impl VerifiedOwnerDisposition {
    pub fn verify(
        binding: &NatsCommandBinding,
        disposition: DurableOwnerDisposition,
        verifier: &impl OwnerDispositionVerifier,
    ) -> AdapterResult<Self> {
        if matches!(
            plan_acknowledgement(binding, &disposition, AckMode::Ack),
            AckPlan::DoNotAcknowledge
        ) {
            return Err(AdapterError::AckNotPermitted);
        }
        verifier
            .verify_durable_disposition(binding, &disposition)
            .map_err(AdapterError::OwnerDispositionVerification)?;
        let binding_bytes = binding
            .canonical_bytes()
            .map_err(AdapterError::Binding)?;
        Ok(Self {
            binding_digest: sha256(&binding_bytes),
            disposition,
        })
    }

    fn is_for_binding(&self, binding: &NatsCommandBinding) -> AdapterResult<bool> {
        let bytes = binding.canonical_bytes().map_err(AdapterError::Binding)?;
        Ok(self.binding_digest == sha256(&bytes))
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AckExecution {
    pub stream: String,
    pub consumer: String,
    pub stream_sequence: u64,
    pub mode: AckMode,
    /// True only when double_ack received the broker confirmation response.
    pub broker_confirmed: bool,
    pub disposition_digest: [u8; 32],
    pub assessment: BrokerAckAssessment,
}

pub struct ReceivedDelivery {
    pub observation: JetStreamDeliveryObservation,
    pub binding: NatsCommandBinding,
    pub payload: Bytes,
    acker: Acker,
}

impl ReceivedDelivery {
    /// Extends JetStream's AckWait timer only. It does not renew a Mycelix
    /// decision, worker claim, owner epoch, or resource generation.
    pub async fn progress(&self) -> AdapterResult<()> {
        self.acker
            .ack_with(AckKind::Progress)
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))
    }

    /// A broker ACK is permitted only after the independent Mycelix effect
    /// owner has produced an exact durable disposition. Ack sends the ACK;
    /// AckSync waits for the broker to confirm receipt of that ACK.
    pub async fn acknowledge(
        self,
        verified: &VerifiedOwnerDisposition,
        mode: AckMode,
    ) -> AdapterResult<AckExecution> {
        if !verified.is_for_binding(&self.binding)? {
            return Err(AdapterError::VerifiedDispositionBindingMismatch);
        }
        let plan = plan_acknowledgement(&self.binding, &verified.disposition, mode);
        let AckPlan::Acknowledge { disposition_digest, .. } = plan else {
            return Err(AdapterError::AckNotPermitted);
        };

        let broker_confirmed = match mode {
            AckMode::Ack => {
                self.acker
                    .ack()
                    .await
                    .map_err(|error| AdapterError::Nats(error.to_string()))?;
                false
            }
            AckMode::AckSync => {
                self.acker
                    .double_ack()
                    .await
                    .map_err(|error| AdapterError::Nats(error.to_string()))?;
                true
            }
        };

        let confirmation = JetStreamAckConfirmation {
            stream_name: self.observation.stream_name.clone(),
            consumer_name: self.observation.consumer_name.clone(),
            stream_sequence: self.observation.stream_sequence,
            ack_received_by_broker: broker_confirmed,
        };
        let assessment = assess_ack_confirmation(
            &self.observation.stream_name,
            &self.observation.consumer_name,
            self.observation.stream_sequence,
            &confirmation,
        );

        Ok(AckExecution {
            stream: self.observation.stream_name,
            consumer: self.observation.consumer_name,
            stream_sequence: self.observation.stream_sequence,
            mode,
            broker_confirmed,
            disposition_digest,
            assessment,
        })
    }
}

#[derive(Clone)]
pub struct JetStreamAdapter {
    context: jetstream::Context,
    config: JetStreamAdapterConfig,
    server_config_verified: Arc<AtomicBool>,
}

impl JetStreamAdapter {
    pub fn new(
        context: jetstream::Context,
        config: JetStreamAdapterConfig,
    ) -> AdapterResult<Self> {
        config.validate()?;
        Ok(Self {
            context,
            config,
            server_config_verified: Arc::new(AtomicBool::new(false)),
        })
    }

    pub async fn connect(
        server_url: &str,
        config: JetStreamAdapterConfig,
    ) -> AdapterResult<Self> {
        let client = async_nats::connect(server_url)
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;
        Self::new(jetstream::new(client), config)
    }

    pub fn config(&self) -> &JetStreamAdapterConfig {
        &self.config
    }

    /// Fetches server-side stream/consumer configuration and rejects drift.
    /// Existing stream or consumer settings are never silently rewritten.
    pub async fn verify_server_configuration(&self) -> AdapterResult<()> {
        let stream = self
            .context
            .get_stream(&self.config.stream_name)
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;
        let stream_info = stream
            .get_info()
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        if !stream_info
            .config
            .subjects
            .iter()
            .any(|subject| subject == &self.config.subject)
        {
            return Err(AdapterError::StreamConfigurationMismatch("subjects"));
        }
        if stream_info.config.duplicate_window != self.config.duplicate_window {
            return Err(AdapterError::StreamConfigurationMismatch("duplicate_window"));
        }
        if stream_info.config.num_replicas != self.config.expected_stream_replicas {
            return Err(AdapterError::StreamConfigurationMismatch("num_replicas"));
        }
        if stream_info.config.storage != StorageType::File {
            return Err(AdapterError::StreamConfigurationMismatch("storage"));
        }
        if stream_info.config.no_ack {
            return Err(AdapterError::StreamConfigurationMismatch("no_ack"));
        }
        if stream_info.config.discard != DiscardPolicy::New {
            return Err(AdapterError::StreamConfigurationMismatch("discard"));
        }
        if stream_info.config.retention != self.config.expected_retention {
            return Err(AdapterError::StreamConfigurationMismatch("retention"));
        }
        if stream_info.config.max_messages != self.config.max_messages {
            return Err(AdapterError::StreamConfigurationMismatch("max_messages"));
        }
        if stream_info.config.max_bytes != self.config.max_bytes {
            return Err(AdapterError::StreamConfigurationMismatch("max_bytes"));
        }
        if stream_info.config.max_age != self.config.max_age {
            return Err(AdapterError::StreamConfigurationMismatch("max_age"));
        }

        let mut consumer = stream
            .get_or_create_consumer(
                &self.config.durable_consumer,
                consumer::pull::Config {
                    durable_name: Some(self.config.durable_consumer.clone()),
                    ack_policy: AckPolicy::Explicit,
                    ack_wait: self.config.ack_wait,
                    max_ack_pending: self.config.max_ack_pending,
                    max_deliver: self.config.max_deliver,
                    filter_subject: self.config.subject.clone(),
                    ..Default::default()
                },
            )
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        let consumer_info = consumer
            .info()
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        if consumer_info.stream_name != self.config.stream_name
            || consumer_info.name != self.config.durable_consumer
        {
            return Err(AdapterError::ConsumerConfigurationMismatch("identity"));
        }
        if consumer_info.config.durable_name.as_deref()
            != Some(self.config.durable_consumer.as_str())
        {
            return Err(AdapterError::ConsumerConfigurationMismatch("durable_name"));
        }
        if consumer_info.config.ack_policy != AckPolicy::Explicit {
            return Err(AdapterError::ConsumerConfigurationMismatch("ack_policy"));
        }
        if consumer_info.config.ack_wait != self.config.ack_wait {
            return Err(AdapterError::ConsumerConfigurationMismatch("ack_wait"));
        }
        if consumer_info.config.max_ack_pending != self.config.max_ack_pending {
            return Err(AdapterError::ConsumerConfigurationMismatch("max_ack_pending"));
        }
        if consumer_info.config.max_deliver != self.config.max_deliver {
            return Err(AdapterError::ConsumerConfigurationMismatch("max_deliver"));
        }
        if consumer_info.config.filter_subject != self.config.subject {
            return Err(AdapterError::ConsumerConfigurationMismatch("filter_subject"));
        }

        self.server_config_verified.store(true, Ordering::Release);
        Ok(())
    }

    fn require_server_configuration_verified(&self) -> AdapterResult<()> {
        if self.server_config_verified.load(Ordering::Acquire) {
            Ok(())
        } else {
            Err(AdapterError::NotInitialized)
        }
    }

    /// Publishes a raw command payload with immutable authority/effect metadata
    /// in application headers and Nats-Msg-Id in its reserved broker header.
    /// The payload digest is recomputed here rather than accepted from a caller.
    pub async fn publish(
        &self,
        mut input: NatsCommandBindingInput,
        payload: &[u8],
    ) -> AdapterResult<PublishEvidence> {
        self.require_server_configuration_verified()?;
        if input.account_id != self.config.account_id
            || input.stream_name != self.config.stream_name
            || input.subject != self.config.subject
            || input.consumer_name != self.config.durable_consumer
        {
            return Err(AdapterError::AccountScopeMismatch);
        }

        input.payload_digest = sha256(payload);
        let binding = NatsCommandBinding::new(input.clone()).map_err(AdapterError::Binding)?;
        let binding_digest = sha256(
            &binding
                .canonical_bytes()
                .map_err(AdapterError::Binding)?,
        );

        let message = PublishMessage::build()
            .payload(Bytes::copy_from_slice(payload))
            .message_id(&input.nats_message_id)
            .header(HEADER_VERSION, WIRE_VERSION)
            .header(HEADER_PUBLICATION_ID, &input.nats_message_id)
            .header(HEADER_LOGICAL_OPERATION, &input.logical_operation_id)
            .header(HEADER_REQUEST_DIGEST, hex_encode(&input.request_digest))
            .header(HEADER_DECISION_DIGEST, hex_encode(&input.decision_receipt_digest))
            .header(HEADER_IDEMPOTENCY_DIGEST, hex_encode(&input.idempotency_digest))
            .header(HEADER_EFFECT_OWNER, &input.effect_owner_id)
            .header(HEADER_OWNER_EPOCH, input.owner_epoch.to_string())
            .header(
                HEADER_RESOURCE_GENERATION,
                input.resource_generation.to_string(),
            )
            .header(HEADER_PAYLOAD_DIGEST, hex_encode(&input.payload_digest))
            .header(HEADER_BINDING_DIGEST, hex_encode(&binding_digest))
            .outbound_message(input.subject.as_str());

        let ack = self
            .context
            .publish_message(message)
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        if ack.stream != self.config.stream_name || ack.sequence == 0 {
            return Err(AdapterError::StreamConfigurationMismatch("publish_ack"));
        }

        Ok(PublishEvidence {
            stream: ack.stream,
            sequence: ack.sequence,
            duplicate: ack.duplicate,
            domain: ack.domain,
        })
    }

    /// Fetches at most one delivery from the configured durable pull consumer.
    /// An absent message is a normal None result; no implicit ACK is sent.
    pub async fn fetch_one(&self) -> AdapterResult<Option<ReceivedDelivery>> {
        self.require_server_configuration_verified()?;
        let stream = self
            .context
            .get_stream(&self.config.stream_name)
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        let consumer = stream
            .get_or_create_consumer(
                &self.config.durable_consumer,
                consumer::pull::Config {
                    durable_name: Some(self.config.durable_consumer.clone()),
                    ack_policy: AckPolicy::Explicit,
                    ack_wait: self.config.ack_wait,
                    max_ack_pending: self.config.max_ack_pending,
                    max_deliver: self.config.max_deliver,
                    filter_subject: self.config.subject.clone(),
                    ..Default::default()
                },
            )
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        let mut messages = consumer
            .fetch()
            .max_messages(1)
            .expires(self.config.fetch_timeout)
            .messages()
            .await
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        let Some(message) = messages.next().await else {
            return Ok(None);
        };
        let message = message.map_err(|error| AdapterError::Nats(error.to_string()))?;

        let info = message
            .info()
            .map_err(|error| AdapterError::Nats(error.to_string()))?;

        if info.stream != self.config.stream_name
            || info.consumer != self.config.durable_consumer
        {
            return Err(AdapterError::AccountScopeMismatch);
        }
        if self
            .config
            .expected_domain
            .as_deref()
            .is_some_and(|expected| info.domain != Some(expected))
        {
            return Err(AdapterError::JetStreamDomainMismatch);
        }
        if self
            .config
            .expected_account_hash
            .as_deref()
            .is_some_and(|expected| info.acc_hash != Some(expected))
        {
            return Err(AdapterError::AccountHashMismatch);
        }

        let stream_name = info.stream.to_owned();
        let consumer_name = info.consumer.to_owned();
        let stream_sequence = info.stream_sequence;
        let consumer_sequence = info.consumer_sequence;
        let delivery_count = u32::try_from(info.delivered)
            .map_err(|_| AdapterError::InvalidHeader("delivery_count"))?;
        if delivery_count == 0 {
            return Err(AdapterError::InvalidHeader("delivery_count"));
        }

        let (raw_message, acker) = message.split();
        let headers = raw_message
            .headers
            .as_ref()
            .ok_or(AdapterError::MissingHeader(HEADER_VERSION))?;

        if header_text(headers, HEADER_VERSION)? != WIRE_VERSION {
            return Err(AdapterError::InvalidWireVersion);
        }

        let nats_message_id = header_text(headers, HEADER_PUBLICATION_ID)?;
        let request_digest = header_digest(headers, HEADER_REQUEST_DIGEST)?;
        let decision_receipt_digest = header_digest(headers, HEADER_DECISION_DIGEST)?;
        let idempotency_digest = header_digest(headers, HEADER_IDEMPOTENCY_DIGEST)?;
        let effect_owner_id = header_text(headers, HEADER_EFFECT_OWNER)?;
        let owner_epoch = parse_u64_header(headers, HEADER_OWNER_EPOCH)?;
        let resource_generation = parse_u64_header(headers, HEADER_RESOURCE_GENERATION)?;
        let declared_payload_digest = header_digest(headers, HEADER_PAYLOAD_DIGEST)?;
        let declared_binding_digest = header_digest(headers, HEADER_BINDING_DIGEST)?;

        let payload = raw_message.payload;
        let actual_payload_digest = sha256(&payload);
        if declared_payload_digest != actual_payload_digest {
            return Err(AdapterError::PayloadDigestMismatch);
        }

        let input = NatsCommandBindingInput {
            account_id: self.config.account_id.clone(),
            stream_name: stream_name.clone(),
            subject: raw_message.subject.to_string(),
            consumer_name: consumer_name.clone(),
            nats_message_id: nats_message_id.clone(),
            payload_digest: actual_payload_digest,
            request_digest,
            decision_receipt_digest,
            logical_operation_id: header_text(headers, HEADER_LOGICAL_OPERATION)?,
            idempotency_digest,
            effect_owner_id,
            owner_epoch,
            resource_generation,
        };
        let binding = NatsCommandBinding::new(input).map_err(AdapterError::Binding)?;
        let computed_binding_digest = sha256(
            &binding
                .canonical_bytes()
                .map_err(AdapterError::Binding)?,
        );
        if computed_binding_digest != declared_binding_digest {
            return Err(AdapterError::BindingDigestMismatch);
        }

        let observation = JetStreamDeliveryObservation {
            account_id: self.config.account_id.clone(),
            stream_name,
            subject: raw_message.subject.to_string(),
            consumer_name,
            nats_message_id,
            payload_digest: actual_payload_digest,
            stream_sequence,
            consumer_sequence,
            delivery_count,
        };
        if assess_delivery(&binding, &observation)
            != myc_dir_fabric_nats_reference::DeliveryAssessment::EligibleForOwnerRevalidation
        {
            return Err(AdapterError::AccountScopeMismatch);
        }

        Ok(Some(ReceivedDelivery {
            observation,
            binding,
            payload,
            acker,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> JetStreamAdapterConfig {
        JetStreamAdapterConfig {
            account_id: "default".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.commands.>".into(),
            durable_consumer: "mycelix-workers".into(),
            duplicate_window: Duration::from_secs(120),
            expected_stream_replicas: 1,
            expected_retention: RetentionPolicy::Limits,
            max_messages: 10_000,
            max_bytes: 32 * 1024 * 1024,
            max_age: Duration::from_secs(7 * 24 * 60 * 60),
            expected_domain: Some("MYCELIX".into()),
            expected_account_hash: None,
            ack_wait: Duration::from_secs(30),
            max_ack_pending: 256,
            max_deliver: 10,
            fetch_timeout: Duration::from_secs(5),
        }
    }

    #[test]
    fn valid_pinned_configuration_is_accepted() {
        assert!(config().validate().is_ok());
    }

    #[test]
    fn empty_account_identity_is_rejected() {
        let mut value = config();
        value.account_id.clear();
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("account_id"))
        ));
    }

    #[test]
    fn zero_duplicate_window_is_rejected() {
        let mut value = config();
        value.duplicate_window = Duration::ZERO;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("duplicate_window"))
        ));
    }

    #[test]
    fn zero_replicas_is_rejected() {
        let mut value = config();
        value.expected_stream_replicas = 0;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("expected_stream_replicas"))
        ));
    }

    #[test]
    fn unsupported_replica_count_is_rejected() {
        let mut value = config();
        value.expected_stream_replicas = 6;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("expected_stream_replicas"))
        ));
    }

    #[test]
    fn zero_max_messages_is_rejected() {
        let mut value = config();
        value.max_messages = 0;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("max_messages"))
        ));
    }

    #[test]
    fn zero_max_bytes_is_rejected() {
        let mut value = config();
        value.max_bytes = 0;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("max_bytes"))
        ));
    }

    #[test]
    fn zero_max_age_is_rejected() {
        let mut value = config();
        value.max_age = Duration::ZERO;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("max_age"))
        ));
    }

    #[test]
    fn zero_ack_wait_is_rejected() {
        let mut value = config();
        value.ack_wait = Duration::ZERO;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("ack_wait"))
        ));
    }

    #[test]
    fn zero_ack_pending_is_rejected() {
        let mut value = config();
        value.max_ack_pending = 0;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("max_ack_pending"))
        ));
    }

    #[test]
    fn zero_max_deliver_is_rejected() {
        let mut value = config();
        value.max_deliver = 0;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("max_deliver"))
        ));
    }

    #[test]
    fn zero_fetch_timeout_is_rejected() {
        let mut value = config();
        value.fetch_timeout = Duration::ZERO;
        assert!(matches!(
            value.validate(),
            Err(AdapterError::InvalidConfiguration("fetch_timeout"))
        ));
    }

    #[test]
    fn hex_digest_round_trips_without_loss() {
        let expected = [0xabu8; 32];
        let encoded = hex_encode(&expected);
        assert_eq!(hex_decode_32(&encoded, HEADER_PAYLOAD_DIGEST).unwrap(), expected);
    }

    #[test]
    fn malformed_digest_length_is_rejected() {
        assert!(matches!(
            hex_decode_32("abc", HEADER_PAYLOAD_DIGEST),
            Err(AdapterError::InvalidHeader(HEADER_PAYLOAD_DIGEST))
        ));
    }

    #[test]
    fn non_hex_digest_is_rejected() {
        assert!(matches!(
            hex_decode_32(&"gg".repeat(32), HEADER_PAYLOAD_DIGEST),
            Err(AdapterError::InvalidHeader(HEADER_PAYLOAD_DIGEST))
        ));
    }

    struct AcceptOwnerDisposition;
    impl OwnerDispositionVerifier for AcceptOwnerDisposition {
        fn verify_durable_disposition(
            &self,
            _binding: &NatsCommandBinding,
            _disposition: &DurableOwnerDisposition,
        ) -> Result<(), String> {
            Ok(())
        }
    }

    struct RejectOwnerDisposition;
    impl OwnerDispositionVerifier for RejectOwnerDisposition {
        fn verify_durable_disposition(
            &self,
            _binding: &NatsCommandBinding,
            _disposition: &DurableOwnerDisposition,
        ) -> Result<(), String> {
            Err("receipt signature invalid".into())
        }
    }

    fn test_binding(message_id: &str) -> NatsCommandBinding {
        NatsCommandBinding::new(NatsCommandBindingInput {
            account_id: "default".into(),
            stream_name: "MYCELIX_COMMANDS".into(),
            subject: "mycelix.commands.test".into(),
            consumer_name: "myc-workers".into(),
            nats_message_id: message_id.into(),
            payload_digest: [1; 32],
            request_digest: [2; 32],
            decision_receipt_digest: [3; 32],
            logical_operation_id: "logical-operation-a".into(),
            idempotency_digest: [4; 32],
            effect_owner_id: "postgres-owner-a".into(),
            owner_epoch: 2,
            resource_generation: 5,
        }).unwrap()
    }

    fn test_disposition() -> DurableOwnerDisposition {
        DurableOwnerDisposition::EffectCommitted {
            logical_operation_id: "logical-operation-a".into(),
            effect_owner_id: "postgres-owner-a".into(),
            idempotency_digest: [4; 32],
            effect_receipt_digest: [9; 32],
        }
    }

    #[test]
    fn verified_owner_disposition_requires_external_verifier() {
        let result = VerifiedOwnerDisposition::verify(
            &test_binding("message-a"),
            test_disposition(),
            &RejectOwnerDisposition,
        );
        assert!(matches!(result, Err(AdapterError::OwnerDispositionVerification(_))));
    }

    #[test]
    fn verified_owner_disposition_is_bound_to_exact_command() {
        let first = test_binding("message-a");
        let second = test_binding("message-b");
        let verified = VerifiedOwnerDisposition::verify(
            &first,
            test_disposition(),
            &AcceptOwnerDisposition,
        ).unwrap();
        assert!(verified.is_for_binding(&first).unwrap());
        assert!(!verified.is_for_binding(&second).unwrap());
    }

    #[test]
    fn duplicate_authority_header_is_rejected() {
        let mut headers = async_nats::HeaderMap::new();
        headers.append(HEADER_OWNER_EPOCH, "7");
        headers.append(HEADER_OWNER_EPOCH, "8");
        assert!(matches!(
            header_text(&headers, HEADER_OWNER_EPOCH),
            Err(AdapterError::InvalidHeader(HEADER_OWNER_EPOCH))
        ));
    }
}
