//! Transport-neutral contracts for Pulse's future multi-network adapters.
//!
//! This crate does not implement radio drivers, encryption, signature
//! verification, or a network stack. The application must construct
//! OpaqueEnvelope only after its existing end-to-end protection pipeline,
//! evaluate a candidate before sending, and record recipient delivery only
//! after Pulse's trusted verifier has validated the recipient receipt.
//!
//! Adapters receive opaque bytes and cannot claim recipient delivery through
//! their send result. Link and gateway acknowledgements remain pending states.

use std::collections::BTreeSet;
use std::fmt;

/// Bounded retry/deadline lifecycle primitive; durable storage is supplied by the caller.
pub mod delivery_state;
/// Storage-neutral journal contract and in-memory reference model; not durable.
pub mod delivery_journal;

const MAX_IDENTIFIER_BYTES: usize = 64;
const MAX_MESSAGE_ID_BYTES: usize = 128;
const MAX_RECIPIENT_ID_BYTES: usize = 512;
const MAX_RECEIPT_BYTES: usize = 16 * 1024;
/// Contract-level cap for one serialized encrypted Pulse envelope.
/// Large attachments should be chunked/referenced above this layer.
pub const MAX_OPAQUE_ENVELOPE_BYTES: usize = 16 * 1024 * 1024;

/// Monotonic process-local time used only for freshness and elapsed durations.
/// Never serialize it as a cross-device message expiry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MonotonicMillis(u64);

impl MonotonicMillis {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    pub const fn as_millis(self) -> u64 {
        self.0
    }
}

/// Unix-epoch milliseconds used for message expiry across queues and devices.
/// Callers must account for wall-clock uncertainty in their expiry policy.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct UnixMillis(u64);

impl UnixMillis {
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    pub const fn as_millis(self) -> u64 {
        self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum IdentifierError {
    Empty,
    TooLong,
    InvalidCharacter,
}

impl fmt::Display for IdentifierError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => f.write_str("identifier must not be empty"),
            Self::TooLong => f.write_str("identifier exceeds its maximum byte length"),
            Self::InvalidCharacter => f.write_str(
                "identifier may contain only lowercase ASCII letters, digits, '.', '_' or '-'",
            ),
        }
    }
}

impl std::error::Error for IdentifierError {}

fn validate_identifier(value: &str) -> Result<(), IdentifierError> {
    if value.is_empty() {
        return Err(IdentifierError::Empty);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(IdentifierError::TooLong);
    }
    if !value.bytes().all(|byte| {
        byte.is_ascii_lowercase()
            || byte.is_ascii_digit()
            || matches!(byte, b'.' | b'_' | b'-')
    }) {
        return Err(IdentifierError::InvalidCharacter);
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransportId(String);

impl TransportId {
    pub fn new(value: impl Into<String>) -> Result<Self, IdentifierError> {
        let value = value.into();
        validate_identifier(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FailureDomainId(String);

impl FailureDomainId {
    pub fn new(value: impl Into<String>) -> Result<Self, IdentifierError> {
        let value = value.into();
        validate_identifier(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TransportProfileId(String);

impl TransportProfileId {
    pub fn new(value: impl Into<String>) -> Result<Self, IdentifierError> {
        let value = value.into();
        validate_identifier(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Version of the transport-neutral Pulse encrypted-envelope contract.
/// This is distinct from the link-specific frame/profile used by an adapter.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EnvelopeProfileId(String);

impl EnvelopeProfileId {
    pub fn new(value: impl Into<String>) -> Result<Self, IdentifierError> {
        let value = value.into();
        validate_identifier(&value)?;
        Ok(Self(value))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransportMode {
    Online,
    Opportunistic,
    StoreAndForward,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Send,
    Receive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PolicyDecision {
    Allowed,
    Denied,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkState {
    Available,
    Unavailable,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkObservation {
    pub state: LinkState,
    pub observed_at_monotonic: MonotonicMillis,
    pub valid_for_millis: u64,
}

impl LinkObservation {
    pub fn effective_state(self, now_monotonic: MonotonicMillis) -> LinkState {
        if self.state == LinkState::Unknown
            || now_monotonic.as_millis() < self.observed_at_monotonic.as_millis()
            || self.valid_for_millis == 0
            || now_monotonic.as_millis() - self.observed_at_monotonic.as_millis()
                >= self.valid_for_millis
        {
            LinkState::Unknown
        } else {
            self.state
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransportCapabilities {
    can_send: bool,
    can_receive: bool,
    max_envelope_bytes: usize,
    mode: TransportMode,
    transport_profiles: Vec<TransportProfileId>,
    envelope_profiles: Vec<EnvelopeProfileId>,
}

impl TransportCapabilities {
    pub fn new(
        can_send: bool,
        can_receive: bool,
        max_envelope_bytes: usize,
        mode: TransportMode,
        transport_profiles: Vec<TransportProfileId>,
        envelope_profiles: Vec<EnvelopeProfileId>,
    ) -> Result<Self, CapabilityError> {
        if !can_send && !can_receive {
            return Err(CapabilityError::NoDirectionSupported);
        }
        if max_envelope_bytes == 0 {
            return Err(CapabilityError::ZeroPayloadBudget);
        }
        if max_envelope_bytes > MAX_OPAQUE_ENVELOPE_BYTES {
            return Err(CapabilityError::PayloadBudgetExceedsContractLimit);
        }
        if transport_profiles.is_empty() {
            return Err(CapabilityError::NoTransportProfilesDeclared);
        }
        if envelope_profiles.is_empty() {
            return Err(CapabilityError::NoEnvelopeProfilesDeclared);
        }
        let unique_transport_count = transport_profiles.iter().collect::<BTreeSet<_>>().len();
        if unique_transport_count != transport_profiles.len() {
            return Err(CapabilityError::DuplicateTransportProfile);
        }
        let unique_envelope_count = envelope_profiles.iter().collect::<BTreeSet<_>>().len();
        if unique_envelope_count != envelope_profiles.len() {
            return Err(CapabilityError::DuplicateEnvelopeProfile);
        }
        Ok(Self {
            can_send,
            can_receive,
            max_envelope_bytes,
            mode,
            transport_profiles,
            envelope_profiles,
        })
    }

    pub fn can_send(&self) -> bool {
        self.can_send
    }

    pub fn can_receive(&self) -> bool {
        self.can_receive
    }

    pub fn max_envelope_bytes(&self) -> usize {
        self.max_envelope_bytes
    }

    pub fn mode(&self) -> TransportMode {
        self.mode
    }

    pub fn supports_transport_profile(&self, profile_id: &TransportProfileId) -> bool {
        self.transport_profiles
            .iter()
            .any(|profile| profile == profile_id)
    }

    pub fn supports_envelope_profile(&self, profile_id: &EnvelopeProfileId) -> bool {
        self.envelope_profiles
            .iter()
            .any(|profile| profile == profile_id)
    }

    pub fn transport_profiles(&self) -> &[TransportProfileId] {
        &self.transport_profiles
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapabilityError {
    NoDirectionSupported,
    ZeroPayloadBudget,
    PayloadBudgetExceedsContractLimit,
    NoTransportProfilesDeclared,
    NoEnvelopeProfilesDeclared,
    DuplicateTransportProfile,
    DuplicateEnvelopeProfile,
}

impl fmt::Display for CapabilityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDirectionSupported => f.write_str("adapter must support send or receive"),
            Self::ZeroPayloadBudget => f.write_str("max envelope size must be greater than zero"),
            Self::PayloadBudgetExceedsContractLimit => {
                f.write_str("adapter payload budget exceeds the contract-level envelope limit")
            }
            Self::NoTransportProfilesDeclared => {
                f.write_str("at least one transport profile must be declared")
            }
            Self::NoEnvelopeProfilesDeclared => {
                f.write_str("at least one envelope profile must be declared")
            }
            Self::DuplicateTransportProfile => {
                f.write_str("transport profile IDs must be unique")
            }
            Self::DuplicateEnvelopeProfile => {
                f.write_str("envelope profile IDs must be unique")
            }
        }
    }
}

impl std::error::Error for CapabilityError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AdapterDescriptor {
    id: TransportId,
    failure_domain_id: FailureDomainId,
    capabilities: TransportCapabilities,
}

impl AdapterDescriptor {
    pub fn new(
        id: TransportId,
        failure_domain_id: FailureDomainId,
        capabilities: TransportCapabilities,
    ) -> Self {
        Self {
            id,
            failure_domain_id,
            capabilities,
        }
    }

    pub fn id(&self) -> &TransportId {
        &self.id
    }

    pub fn failure_domain_id(&self) -> &FailureDomainId {
        &self.failure_domain_id
    }

    pub fn capabilities(&self) -> &TransportCapabilities {
        &self.capabilities
    }
}

/// A length-bounded logical envelope containing bytes opaque to the adapter.
///
/// The envelope profile identifies the transport-neutral Pulse message format.
/// It is separate from transport framing (for example, QUIC streams versus a
/// Bluetooth GATT frame). Construction validates metadata only; it does not
/// prove that the payload is encrypted or signed. Callers must pass bytes
/// produced by Pulse's existing end-to-end cryptographic pipeline.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OpaqueEnvelope {
    logical_message_id: String,
    envelope_profile_id: EnvelopeProfileId,
    payload: Vec<u8>,
    expires_at_unix_millis: UnixMillis,
}

impl OpaqueEnvelope {
    pub fn new(
        logical_message_id: impl Into<String>,
        envelope_profile_id: EnvelopeProfileId,
        payload: Vec<u8>,
        expires_at_unix_millis: UnixMillis,
    ) -> Result<Self, EnvelopeError> {
        let logical_message_id = logical_message_id.into();
        if logical_message_id.trim().is_empty()
            || logical_message_id.len() > MAX_MESSAGE_ID_BYTES
            || logical_message_id.chars().any(char::is_control)
        {
            return Err(EnvelopeError::InvalidLogicalMessageId);
        }
        if payload.is_empty() {
            return Err(EnvelopeError::EmptyPayload);
        }
        if payload.len() > MAX_OPAQUE_ENVELOPE_BYTES {
            return Err(EnvelopeError::PayloadTooLarge);
        }
        Ok(Self {
            logical_message_id,
            envelope_profile_id,
            payload,
            expires_at_unix_millis,
        })
    }

    pub fn logical_message_id(&self) -> &str {
        &self.logical_message_id
    }

    pub fn envelope_profile_id(&self) -> &EnvelopeProfileId {
        &self.envelope_profile_id
    }

    /// Returns opaque payload bytes. No plaintext representation is provided.
    pub fn payload(&self) -> &[u8] {
        &self.payload
    }

    pub fn payload_len(&self) -> usize {
        self.payload.len()
    }

    pub fn expires_at_unix_millis(&self) -> UnixMillis {
        self.expires_at_unix_millis
    }

    pub fn is_expired(&self, now_unix_millis: UnixMillis) -> bool {
        now_unix_millis >= self.expires_at_unix_millis
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnvelopeError {
    InvalidLogicalMessageId,
    EmptyPayload,
    PayloadTooLarge,
}

impl fmt::Display for EnvelopeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLogicalMessageId => f.write_str("logical message ID is invalid"),
            Self::EmptyPayload => f.write_str("opaque envelope payload must not be empty"),
            Self::PayloadTooLarge => {
                f.write_str("opaque envelope payload exceeds the contract-level size limit")
            }
        }
    }
}

impl std::error::Error for EnvelopeError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RoutePolicy {
    pub direction: Direction,
    pub required_envelope_profile_id: EnvelopeProfileId,
    /// If omitted, the adapter's first declared transport profile is selected.
    pub required_transport_profile_id: Option<TransportProfileId>,
    pub require_store_and_forward: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IneligibilityReason {
    UserConsentDenied,
    UserConsentUnknown,
    RegulatoryPermissionDenied,
    RegulatoryPermissionUnknown,
    PrivacyPolicyDenied,
    PrivacyPolicyUnknown,
    EnvelopeExpired,
    EnvelopeProfileMismatch,
    EnvelopeProfileUnsupported,
    ProfileUnsupported,
    SendUnsupported,
    ReceiveUnsupported,
    PayloadTooLarge,
    StoreAndForwardRequired,
    LinkUnavailable,
    LinkAvailabilityUnknown,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RouteCandidate<'a> {
    pub descriptor: &'a AdapterDescriptor,
    pub observation: LinkObservation,
    pub user_consent: PolicyDecision,
    pub regulatory_permission: PolicyDecision,
    pub privacy_permission: PolicyDecision,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EligibleRoute {
    pub transport_id: TransportId,
    pub transport_profile_id: TransportProfileId,
    pub failure_domain_id: FailureDomainId,
    pub mode: TransportMode,
    pub max_envelope_bytes: usize,
}

pub fn evaluate_route(
    candidate: &RouteCandidate<'_>,
    policy: &RoutePolicy,
    envelope: &OpaqueEnvelope,
    now_unix_millis: UnixMillis,
    now_monotonic: MonotonicMillis,
) -> Result<EligibleRoute, IneligibilityReason> {
    if candidate.user_consent != PolicyDecision::Allowed {
        return Err(match candidate.user_consent {
            PolicyDecision::Denied => IneligibilityReason::UserConsentDenied,
            PolicyDecision::Unknown => IneligibilityReason::UserConsentUnknown,
            PolicyDecision::Allowed => unreachable!("allowed handled by the condition"),
        });
    }
    if candidate.regulatory_permission != PolicyDecision::Allowed {
        return Err(match candidate.regulatory_permission {
            PolicyDecision::Denied => IneligibilityReason::RegulatoryPermissionDenied,
            PolicyDecision::Unknown => IneligibilityReason::RegulatoryPermissionUnknown,
            PolicyDecision::Allowed => unreachable!("allowed handled by the condition"),
        });
    }
    if candidate.privacy_permission != PolicyDecision::Allowed {
        return Err(match candidate.privacy_permission {
            PolicyDecision::Denied => IneligibilityReason::PrivacyPolicyDenied,
            PolicyDecision::Unknown => IneligibilityReason::PrivacyPolicyUnknown,
            PolicyDecision::Allowed => unreachable!("allowed handled by the condition"),
        });
    }
    if envelope.is_expired(now_unix_millis) {
        return Err(IneligibilityReason::EnvelopeExpired);
    }
    if envelope.envelope_profile_id() != &policy.required_envelope_profile_id {
        return Err(IneligibilityReason::EnvelopeProfileMismatch);
    }

    let capabilities = candidate.descriptor.capabilities();
    if !capabilities.supports_envelope_profile(&policy.required_envelope_profile_id) {
        return Err(IneligibilityReason::EnvelopeProfileUnsupported);
    }
    let transport_profile_id = match policy.required_transport_profile_id.as_ref() {
        Some(required) if capabilities.supports_transport_profile(required) => required.clone(),
        Some(_) => return Err(IneligibilityReason::ProfileUnsupported),
        None => capabilities
            .transport_profiles()
            .first()
            .expect("capabilities guarantee at least one transport profile")
            .clone(),
    };
    match policy.direction {
        Direction::Send if !capabilities.can_send() => {
            return Err(IneligibilityReason::SendUnsupported);
        }
        Direction::Receive if !capabilities.can_receive() => {
            return Err(IneligibilityReason::ReceiveUnsupported);
        }
        Direction::Send | Direction::Receive => {}
    }
    if envelope.payload_len() > capabilities.max_envelope_bytes() {
        return Err(IneligibilityReason::PayloadTooLarge);
    }
    if policy.require_store_and_forward && capabilities.mode() != TransportMode::StoreAndForward {
        return Err(IneligibilityReason::StoreAndForwardRequired);
    }

    match candidate.observation.effective_state(now_monotonic) {
        LinkState::Available => {}
        LinkState::Unavailable => return Err(IneligibilityReason::LinkUnavailable),
        LinkState::Unknown => return Err(IneligibilityReason::LinkAvailabilityUnknown),
    }

    Ok(EligibleRoute {
        transport_id: candidate.descriptor.id().clone(),
        transport_profile_id,
        failure_domain_id: candidate.descriptor.failure_domain_id().clone(),
        mode: capabilities.mode(),
        max_envelope_bytes: capabilities.max_envelope_bytes(),
    })
}

/// Counts distinct declared failure domains, not adapter instances.
pub fn distinct_failure_domain_count(routes: &[EligibleRoute]) -> usize {
    routes
        .iter()
        .map(|route| &route.failure_domain_id)
        .collect::<BTreeSet<_>>()
        .len()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdapterSendOutcome {
    AcceptedByAdapter,
    LinkLayerAcknowledged,
    GatewayAcknowledged,
    TemporarilyUnavailable,
    PermanentlyRejected,
    OutcomeUnknown,
}

/// One opaque frame accepted from a transport adapter.
///
/// The receive timestamp is monotonic and process-local; envelope expiry uses
/// the separate UnixMillis type. The caller must authenticate/decrypt/validate
/// the frame using Pulse's existing cryptographic pipeline before application
/// processing or replay-ledger insertion.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ReceivedOpaqueEnvelope {
    envelope: OpaqueEnvelope,
    received_at_monotonic: MonotonicMillis,
}

impl ReceivedOpaqueEnvelope {
    pub fn new(
        envelope: OpaqueEnvelope,
        received_at_monotonic: MonotonicMillis,
    ) -> Self {
        Self {
            envelope,
            received_at_monotonic,
        }
    }

    pub fn envelope(&self) -> &OpaqueEnvelope {
        &self.envelope
    }

    pub fn received_at_monotonic(&self) -> MonotonicMillis {
        self.received_at_monotonic
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdapterReceiveOutcome {
    Frame(ReceivedOpaqueEnvelope),
    NoFrameAvailable,
    TemporarilyUnavailable,
    PermanentlyRejected,
    OutcomeUnknown,
}

/// An adapter can report only what its own layer observed. It cannot claim
/// recipient delivery; that requires Pulse's separate receipt verifier.
pub trait TransportAdapter {
    fn descriptor(&self) -> &AdapterDescriptor;

    fn observe(&mut self, now_monotonic: MonotonicMillis) -> LinkObservation;

    fn send_opaque(&mut self, envelope: &OpaqueEnvelope) -> AdapterSendOutcome;

    /// Non-blocking receive poll. Incoming bytes remain untrusted until the
    /// application-level cryptographic verifier accepts the envelope.
    fn receive_opaque(&mut self, now_monotonic: MonotonicMillis) -> AdapterReceiveOutcome;
}

impl From<AdapterSendOutcome> for DeliveryEvidenceStage {
    fn from(outcome: AdapterSendOutcome) -> Self {
        match outcome {
            AdapterSendOutcome::AcceptedByAdapter => Self::AcceptedByAdapter,
            AdapterSendOutcome::LinkLayerAcknowledged => Self::LinkLayerAcknowledged,
            AdapterSendOutcome::GatewayAcknowledged => Self::GatewayAcknowledged,
            AdapterSendOutcome::TemporarilyUnavailable => Self::TemporarilyUnavailable,
            AdapterSendOutcome::PermanentlyRejected => Self::FailedPermanently,
            AdapterSendOutcome::OutcomeUnknown => Self::OutcomeUnknown,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeliveryEvidenceStage {
    NotAttempted,
    LocallyQueued,
    AcceptedByAdapter,
    LinkLayerAcknowledged,
    GatewayAcknowledged,
    TemporarilyUnavailable,
    FailedPermanently,
    Expired,
    OutcomeUnknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserVisibleDeliveryState {
    Queued,
    Pending,
    Delivered,
    Failed,
    Expired,
    Unknown,
}

/// Errors returned by the trusted recipient-receipt verification boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReceiptVerificationError {
    InvalidContext,
    InvalidReceipt,
}

impl fmt::Display for ReceiptVerificationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidContext => f.write_str("receipt context is empty, oversized, or invalid"),
            Self::InvalidReceipt => {
                f.write_str("recipient receipt failed authentication or binding checks")
            }
        }
    }
}

impl std::error::Error for ReceiptVerificationError {}

/// Security-critical integration point implemented by Pulse's trusted
/// cryptographic receipt-verification module, never by a transport adapter.
///
/// Implementations MUST verify the receipt's authenticating signature/MAC,
/// logical message ID, intended recipient identity, exact envelope commitment,
/// and any required protocol or freshness fields. The trait is deliberately
/// explicit because this crate has no dependency on Pulse's crypto implementation.
pub trait TrustedRecipientReceiptVerifier {
    fn verify(
        &self,
        encoded_receipt: &[u8],
        expected_logical_message_id: &str,
        expected_recipient_id: &str,
        expected_envelope_commitment: &[u8; 32],
    ) -> Result<(), ReceiptVerificationError>;
}

/// Opaque capability issued only after the configured trusted verifier accepts
/// a receipt for one specific message and recipient. Its binding fields are
/// private, so callers cannot construct or retarget a token with a struct
/// literal from outside this crate.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifiedRecipientReceipt {
    logical_message_id: String,
    recipient_id: String,
    envelope_commitment: [u8; 32],
}

impl VerifiedRecipientReceipt {
    pub fn verify<V: TrustedRecipientReceiptVerifier>(
        verifier: &V,
        encoded_receipt: &[u8],
        expected_logical_message_id: &str,
        expected_recipient_id: &str,
        expected_envelope_commitment: [u8; 32],
    ) -> Result<Self, ReceiptVerificationError> {
        if !valid_receipt_binding(expected_logical_message_id, MAX_MESSAGE_ID_BYTES)
            || !valid_receipt_binding(expected_recipient_id, MAX_RECIPIENT_ID_BYTES)
        {
            return Err(ReceiptVerificationError::InvalidContext);
        }
        if encoded_receipt.is_empty() || encoded_receipt.len() > MAX_RECEIPT_BYTES {
            return Err(ReceiptVerificationError::InvalidReceipt);
        }
        verifier.verify(
            encoded_receipt,
            expected_logical_message_id,
            expected_recipient_id,
            &expected_envelope_commitment,
        )?;
        Ok(Self {
            logical_message_id: expected_logical_message_id.to_owned(),
            recipient_id: expected_recipient_id.to_owned(),
            envelope_commitment: expected_envelope_commitment,
        })
    }

    fn matches(
        &self,
        logical_message_id: &str,
        recipient_id: &str,
        envelope_commitment: &[u8; 32],
    ) -> bool {
        self.logical_message_id == logical_message_id
            && self.recipient_id == recipient_id
            && &self.envelope_commitment == envelope_commitment
    }
}

fn valid_receipt_binding(value: &str, maximum_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= maximum_bytes
        && !value.chars().any(char::is_control)
}

/// Converts transport-layer evidence to user-visible state. No transport
/// outcome maps to Delivered. A separately verified token must match the
/// current logical message ID, recipient, and immutable envelope commitment.
pub fn user_visible_delivery_state(
    stage: DeliveryEvidenceStage,
    logical_message_id: &str,
    recipient_id: &str,
    envelope_commitment: &[u8; 32],
    verified_receipt: Option<&VerifiedRecipientReceipt>,
) -> UserVisibleDeliveryState {
    if verified_receipt.is_some_and(|receipt| {
        receipt.matches(logical_message_id, recipient_id, envelope_commitment)
    }) {
        return UserVisibleDeliveryState::Delivered;
    }
    match stage {
        DeliveryEvidenceStage::LocallyQueued => UserVisibleDeliveryState::Queued,
        DeliveryEvidenceStage::AcceptedByAdapter
        | DeliveryEvidenceStage::LinkLayerAcknowledged
        | DeliveryEvidenceStage::GatewayAcknowledged
        | DeliveryEvidenceStage::TemporarilyUnavailable => UserVisibleDeliveryState::Pending,
        DeliveryEvidenceStage::FailedPermanently => UserVisibleDeliveryState::Failed,
        DeliveryEvidenceStage::Expired => UserVisibleDeliveryState::Expired,
        DeliveryEvidenceStage::NotAttempted | DeliveryEvidenceStage::OutcomeUnknown => {
            UserVisibleDeliveryState::Unknown
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn envelope_profile() -> EnvelopeProfileId {
        EnvelopeProfileId::new("pulse-envelope-v1").unwrap()
    }

    fn transport_profile() -> TransportProfileId {
        TransportProfileId::new("test-frame-v1").unwrap()
    }

    fn descriptor_for_transport_profile(
        id: &str,
        domain: &str,
        max_bytes: usize,
        mode: TransportMode,
        transport_profile_id: &str,
    ) -> AdapterDescriptor {
        AdapterDescriptor::new(
            TransportId::new(id).unwrap(),
            FailureDomainId::new(domain).unwrap(),
            TransportCapabilities::new(
                true,
                true,
                max_bytes,
                mode,
                vec![TransportProfileId::new(transport_profile_id).unwrap()],
                vec![envelope_profile()],
            )
            .unwrap(),
        )
    }

    fn descriptor(
        id: &str,
        domain: &str,
        max_bytes: usize,
        mode: TransportMode,
    ) -> AdapterDescriptor {
        descriptor_for_transport_profile(
            id,
            domain,
            max_bytes,
            mode,
            "test-frame-v1",
        )
    }

    fn envelope(bytes: usize, expires_at_unix_millis: UnixMillis) -> OpaqueEnvelope {
        OpaqueEnvelope::new(
            "msg-0001",
            envelope_profile(),
            vec![0xA5; bytes],
            expires_at_unix_millis,
        )
        .unwrap()
    }

    fn candidate<'a>(
        descriptor: &'a AdapterDescriptor,
        observation: LinkObservation,
    ) -> RouteCandidate<'a> {
        RouteCandidate {
            descriptor,
            observation,
            user_consent: PolicyDecision::Allowed,
            regulatory_permission: PolicyDecision::Allowed,
            privacy_permission: PolicyDecision::Allowed,
        }
    }

    fn available_observation() -> LinkObservation {
        LinkObservation {
            state: LinkState::Available,
            observed_at_monotonic: MonotonicMillis::new(1_000),
            valid_for_millis: 10_000,
        }
    }

    fn policy() -> RoutePolicy {
        RoutePolicy {
            direction: Direction::Send,
            required_envelope_profile_id: envelope_profile(),
            required_transport_profile_id: None,
            require_store_and_forward: false,
        }
    }

    #[test]
    fn eligible_route_preserves_failure_domain_and_profile_limits() {
        let descriptor = descriptor(
            "wifi-direct",
            "local-wifi",
            2048,
            TransportMode::Opportunistic,
        );
        let route = evaluate_route(
            &candidate(&descriptor, available_observation()),
            &policy(),
            &envelope(512, UnixMillis::new(20_000)),
            UnixMillis::new(1_500),
            MonotonicMillis::new(1_500),
        )
        .unwrap();

        assert_eq!(route.transport_id.as_str(), "wifi-direct");
        assert_eq!(route.failure_domain_id.as_str(), "local-wifi");
        assert_eq!(route.max_envelope_bytes, 2048);
    }

    #[test]
    fn consent_regulatory_and_privacy_unknowns_fail_closed() {
        let descriptor = descriptor(
            "bluetooth-le",
            "nearby-radio",
            1024,
            TransportMode::Opportunistic,
        );
        let base = candidate(&descriptor, available_observation());

        let mut no_consent = base.clone();
        no_consent.user_consent = PolicyDecision::Unknown;
        assert_eq!(
            evaluate_route(
                &no_consent,
                &policy(),
                &envelope(100, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::UserConsentUnknown)
        );

        let mut no_regulatory_proof = base.clone();
        no_regulatory_proof.regulatory_permission = PolicyDecision::Unknown;
        assert_eq!(
            evaluate_route(
                &no_regulatory_proof,
                &policy(),
                &envelope(100, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::RegulatoryPermissionUnknown)
        );

        let mut unknown_privacy = base;
        unknown_privacy.privacy_permission = PolicyDecision::Unknown;
        assert_eq!(
            evaluate_route(
                &unknown_privacy,
                &policy(),
                &envelope(100, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::PrivacyPolicyUnknown)
        );
    }

    #[test]
    fn stale_future_and_unknown_link_observations_are_not_eligible() {
        let descriptor = descriptor("ip-path", "home-uplink", 4096, TransportMode::Online);
        let envelope = envelope(100, UnixMillis::new(20_000));
        let policy = policy();

        let stale = LinkObservation {
            state: LinkState::Available,
            observed_at_monotonic: MonotonicMillis::new(1_000),
            valid_for_millis: 100,
        };
        assert_eq!(
            evaluate_route(
                &candidate(&descriptor, stale),
                &policy,
                &envelope,
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_100),
            ),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );

        let future = LinkObservation {
            state: LinkState::Available,
            observed_at_monotonic: MonotonicMillis::new(2_000),
            valid_for_millis: 500,
        };
        assert_eq!(
            evaluate_route(
                &candidate(&descriptor, future),
                &policy,
                &envelope,
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );

        let unknown = LinkObservation {
            state: LinkState::Unknown,
            observed_at_monotonic: MonotonicMillis::new(1_000),
            valid_for_millis: 500,
        };
        assert_eq!(
            evaluate_route(
                &candidate(&descriptor, unknown),
                &policy,
                &envelope,
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_100),
            ),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );
    }

    #[test]
    fn one_envelope_can_cross_distinct_transport_profiles() {
        let quic = descriptor_for_transport_profile(
            "quic-adapter",
            "wan-uplink",
            4096,
            TransportMode::Online,
            "quic-stream-v1",
        );
        let bluetooth = descriptor_for_transport_profile(
            "bluetooth-adapter",
            "nearby-radio",
            1024,
            TransportMode::Opportunistic,
            "bluetooth-gatt-v1",
        );
        let envelope = envelope(100, UnixMillis::new(20_000));
        let policy = policy();

        let quic_route = evaluate_route(
            &candidate(&quic, available_observation()),
            &policy,
            &envelope,
            UnixMillis::new(1_500),
            MonotonicMillis::new(1_500),
        )
        .unwrap();
        let bluetooth_route = evaluate_route(
            &candidate(&bluetooth, available_observation()),
            &policy,
            &envelope,
            UnixMillis::new(1_500),
            MonotonicMillis::new(1_500),
        )
        .unwrap();

        assert_eq!(quic_route.transport_profile_id.as_str(), "quic-stream-v1");
        assert_eq!(
            bluetooth_route.transport_profile_id.as_str(),
            "bluetooth-gatt-v1"
        );
        assert_eq!(envelope.envelope_profile_id().as_str(), "pulse-envelope-v1");
    }

    #[test]
    fn envelope_profile_and_transport_profile_failures_are_distinct() {
        let descriptor = descriptor(
            "quic-adapter",
            "wan-uplink",
            4096,
            TransportMode::Online,
        );
        let incompatible_envelope = OpaqueEnvelope::new(
            "msg-0001",
            EnvelopeProfileId::new("pulse-envelope-v2").unwrap(),
            vec![0xA5; 100],
            UnixMillis::new(20_000),
        )
        .unwrap();
        assert_eq!(
            evaluate_route(
                &candidate(&descriptor, available_observation()),
                &policy(),
                &incompatible_envelope,
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::EnvelopeProfileMismatch)
        );

        let mut unsupported_envelope_policy = policy();
        unsupported_envelope_policy.required_envelope_profile_id =
            EnvelopeProfileId::new("pulse-envelope-v2").unwrap();
        assert_eq!(
            evaluate_route(
                &candidate(&descriptor, available_observation()),
                &unsupported_envelope_policy,
                &incompatible_envelope,
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::EnvelopeProfileUnsupported)
        );
    }

    #[test]
    fn wall_clock_expiry_and_monotonic_freshness_use_separate_types() {
        let observation = LinkObservation {
            state: LinkState::Available,
            observed_at_monotonic: MonotonicMillis::new(1_000),
            valid_for_millis: 5_000,
        };
        let envelope = envelope(100, UnixMillis::new(20_000));

        assert_eq!(
            observation.effective_state(MonotonicMillis::new(1_500)),
            LinkState::Available
        );
        assert_eq!(
            observation.effective_state(MonotonicMillis::new(6_000)),
            LinkState::Unknown
        );
        assert!(!envelope.is_expired(UnixMillis::new(19_999)));
        assert!(envelope.is_expired(UnixMillis::new(20_000)));
    }

    #[test]
    fn payload_profile_direction_and_expiry_are_hard_gates() {
        let small = descriptor(
            "lpwan-adapter",
            "lpwan-gateway",
            80,
            TransportMode::StoreAndForward,
        );
        assert_eq!(
            evaluate_route(
                &candidate(&small, available_observation()),
                &policy(),
                &envelope(81, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::PayloadTooLarge)
        );
        assert_eq!(
            evaluate_route(
                &candidate(&small, available_observation()),
                &policy(),
                &envelope(50, UnixMillis::new(20_000)),
                UnixMillis::new(20_000),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::EnvelopeExpired)
        );

        let mut unsupported_transport_policy = policy();
        unsupported_transport_policy.required_transport_profile_id =
            Some(TransportProfileId::new("bluetooth-gatt-v1").unwrap());
        assert_eq!(
            evaluate_route(
                &candidate(&small, available_observation()),
                &unsupported_transport_policy,
                &envelope(50, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::ProfileUnsupported)
        );

        let receive_only = AdapterDescriptor::new(
            TransportId::new("receive-only").unwrap(),
            FailureDomainId::new("receive-only-domain").unwrap(),
            TransportCapabilities::new(
                false,
                true,
                1000,
                TransportMode::Online,
                vec![transport_profile()],
                vec![envelope_profile()],
            )
            .unwrap(),
        );
        assert_eq!(
            evaluate_route(
                &candidate(&receive_only, available_observation()),
                &policy(),
                &envelope(50, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::SendUnsupported)
        );

        let mut store_forward_policy = policy();
        store_forward_policy.require_store_and_forward = true;
        let online = descriptor("online-only", "internet-uplink", 4096, TransportMode::Online);
        assert_eq!(
            evaluate_route(
                &candidate(&online, available_observation()),
                &store_forward_policy,
                &envelope(50, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::StoreAndForwardRequired)
        );
    }

    #[test]
    fn unknown_and_forbidden_routes_are_never_promoted_by_quality() {
        let descriptor = descriptor(
            "high-success-but-forbidden",
            "carrier-core",
            4096,
            TransportMode::Online,
        );
        let mut candidate = candidate(&descriptor, available_observation());
        candidate.privacy_permission = PolicyDecision::Denied;
        assert_eq!(
            evaluate_route(
                &candidate,
                &policy(),
                &envelope(100, UnixMillis::new(20_000)),
                UnixMillis::new(1_500),
                MonotonicMillis::new(1_500),
            ),
            Err(IneligibilityReason::PrivacyPolicyDenied)
        );
    }

    #[test]
    fn same_failure_domain_is_not_counted_as_independent_redundancy() {
        let first = descriptor("cellular-a", "carrier-core-a", 4096, TransportMode::Online);
        let second = descriptor("cellular-b", "carrier-core-a", 4096, TransportMode::Online);
        let third = descriptor(
            "community-relay",
            "relay-network-b",
            4096,
            TransportMode::StoreAndForward,
        );
        let routes = [first, second, third]
            .iter()
            .map(|descriptor| {
                evaluate_route(
                    &candidate(descriptor, available_observation()),
                    &policy(),
                    &envelope(100, UnixMillis::new(20_000)),
                    UnixMillis::new(1_500),
                    MonotonicMillis::new(1_500),
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        assert_eq!(distinct_failure_domain_count(&routes), 2);
    }

    struct StubTransportAdapter {
        descriptor: AdapterDescriptor,
        observation: LinkObservation,
        send_outcome: AdapterSendOutcome,
        receive_outcome: AdapterReceiveOutcome,
    }

    impl TransportAdapter for StubTransportAdapter {
        fn descriptor(&self) -> &AdapterDescriptor {
            &self.descriptor
        }

        fn observe(&mut self, _now_monotonic: MonotonicMillis) -> LinkObservation {
            self.observation
        }

        fn send_opaque(&mut self, _envelope: &OpaqueEnvelope) -> AdapterSendOutcome {
            self.send_outcome
        }

        fn receive_opaque(&mut self, _now_monotonic: MonotonicMillis) -> AdapterReceiveOutcome {
            self.receive_outcome.clone()
        }
    }

    #[test]
    fn adapter_contract_has_symmetric_opaque_send_and_receive() {
        let descriptor = descriptor(
            "loopback-test",
            "test-only-domain",
            1024,
            TransportMode::Opportunistic,
        );
        let incoming = ReceivedOpaqueEnvelope::new(
            envelope(16, UnixMillis::new(10_000)),
            MonotonicMillis::new(3_000),
        );
        let mut adapter = StubTransportAdapter {
            descriptor,
            observation: available_observation(),
            send_outcome: AdapterSendOutcome::AcceptedByAdapter,
            receive_outcome: AdapterReceiveOutcome::Frame(incoming.clone()),
        };

        assert_eq!(
            adapter.send_opaque(&envelope(16, UnixMillis::new(10_000))),
            AdapterSendOutcome::AcceptedByAdapter
        );
        assert_eq!(
            adapter.receive_opaque(MonotonicMillis::new(3_100)),
            AdapterReceiveOutcome::Frame(incoming.clone())
        );
        if let AdapterReceiveOutcome::Frame(frame) =
            adapter.receive_opaque(MonotonicMillis::new(3_200))
        {
            assert_eq!(
                frame.received_at_monotonic(),
                MonotonicMillis::new(3_000)
            );
            assert_eq!(frame.envelope().logical_message_id(), "msg-0001");
        } else {
            panic!("stub must return its queued opaque frame");
        }
    }

    #[test]
    fn adapter_outcomes_map_without_fabricating_recipient_delivery() {
        let cases = [
            (
                AdapterSendOutcome::AcceptedByAdapter,
                DeliveryEvidenceStage::AcceptedByAdapter,
            ),
            (
                AdapterSendOutcome::LinkLayerAcknowledged,
                DeliveryEvidenceStage::LinkLayerAcknowledged,
            ),
            (
                AdapterSendOutcome::GatewayAcknowledged,
                DeliveryEvidenceStage::GatewayAcknowledged,
            ),
            (
                AdapterSendOutcome::TemporarilyUnavailable,
                DeliveryEvidenceStage::TemporarilyUnavailable,
            ),
            (
                AdapterSendOutcome::PermanentlyRejected,
                DeliveryEvidenceStage::FailedPermanently,
            ),
            (
                AdapterSendOutcome::OutcomeUnknown,
                DeliveryEvidenceStage::OutcomeUnknown,
            ),
        ];
        for (outcome, expected) in cases {
            assert_eq!(DeliveryEvidenceStage::from(outcome), expected);
            assert_ne!(
                user_visible_delivery_state(expected, "msg-1", "recipient-1", &[0; 32], None),
                UserVisibleDeliveryState::Delivered
            );
        }
    }

    struct TestReceiptVerifier;

    impl TrustedRecipientReceiptVerifier for TestReceiptVerifier {
        fn verify(
            &self,
            encoded_receipt: &[u8],
            expected_logical_message_id: &str,
            expected_recipient_id: &str,
            expected_envelope_commitment: &[u8; 32],
        ) -> Result<(), ReceiptVerificationError> {
            if encoded_receipt == b"valid-test-receipt"
                && expected_logical_message_id == "msg-1"
                && expected_recipient_id == "recipient-1"
                && expected_envelope_commitment == &[0; 32]
            {
                Ok(())
            } else {
                Err(ReceiptVerificationError::InvalidReceipt)
            }
        }
    }

    #[test]
    fn only_verified_receipt_bound_to_exact_message_and_recipient_maps_to_delivered() {
        let verifier = TestReceiptVerifier;
        let receipt = VerifiedRecipientReceipt::verify(
            &verifier,
            b"valid-test-receipt",
            "msg-1",
            "recipient-1",
            [0; 32],
        )
        .unwrap();
        assert_eq!(
            user_visible_delivery_state(
                DeliveryEvidenceStage::GatewayAcknowledged,
                "msg-1",
                "recipient-1",
                &[0; 32],
                Some(&receipt),
            ),
            UserVisibleDeliveryState::Delivered
        );
        assert_eq!(
            user_visible_delivery_state(
                DeliveryEvidenceStage::GatewayAcknowledged,
                "msg-2",
                "recipient-1",
                &[0; 32],
                Some(&receipt),
            ),
            UserVisibleDeliveryState::Pending
        );
        assert_eq!(
            user_visible_delivery_state(
                DeliveryEvidenceStage::GatewayAcknowledged,
                "msg-1",
                "recipient-2",
                Some(&receipt),
            ),
            UserVisibleDeliveryState::Pending
        );
    }

    #[test]
    fn invalid_or_empty_receipt_cannot_issue_verified_token() {
        let verifier = TestReceiptVerifier;
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                b"untrusted-receipt",
                "msg-1",
                "recipient-1",
                [0; 32],
            ),
            Err(ReceiptVerificationError::InvalidReceipt)
        );
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                b"",
                "msg-1",
                "recipient-1",
                [0; 32],
            ),
            Err(ReceiptVerificationError::InvalidContext)
        );
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                &vec![0_u8; MAX_RECEIPT_BYTES + 1],
                "msg-1",
                "recipient-1",
                [0; 32],
            ),
            Err(ReceiptVerificationError::InvalidReceipt)
        );
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                b"valid-test-receipt",
                "",
                "recipient-1",
                [0; 32],
            ),
            Err(ReceiptVerificationError::InvalidContext)
        );
    }

    #[test]
    fn non_receipt_stages_never_map_to_delivered_without_verified_token() {
        for stage in [
            DeliveryEvidenceStage::NotAttempted,
            DeliveryEvidenceStage::LocallyQueued,
            DeliveryEvidenceStage::AcceptedByAdapter,
            DeliveryEvidenceStage::LinkLayerAcknowledged,
            DeliveryEvidenceStage::GatewayAcknowledged,
            DeliveryEvidenceStage::TemporarilyUnavailable,
            DeliveryEvidenceStage::FailedPermanently,
            DeliveryEvidenceStage::Expired,
            DeliveryEvidenceStage::OutcomeUnknown,
        ] {
            assert_ne!(
                user_visible_delivery_state(stage, "msg-1", "recipient-1", &[0; 32], None),
                UserVisibleDeliveryState::Delivered
            );
        }
    }

    #[test]
    fn constructors_reject_ambiguous_or_unbounded_metadata() {
        assert_eq!(TransportId::new(""), Err(IdentifierError::Empty));
        assert_eq!(TransportId::new("MixedCase"), Err(IdentifierError::InvalidCharacter));
        assert_eq!(
            OpaqueEnvelope::new(
                "msg-1",
                envelope_profile(),
                Vec::new(),
                UnixMillis::new(100),
            ),
            Err(EnvelopeError::EmptyPayload)
        );
        assert_eq!(
            OpaqueEnvelope::new(
                "msg-1",
                envelope_profile(),
                vec![0; MAX_OPAQUE_ENVELOPE_BYTES + 1],
                UnixMillis::new(100),
            ),
            Err(EnvelopeError::PayloadTooLarge)
        );
        assert_eq!(
            TransportCapabilities::new(
                true,
                true,
                MAX_OPAQUE_ENVELOPE_BYTES + 1,
                TransportMode::Online,
                vec![transport_profile()],
                vec![envelope_profile()],
            ),
            Err(CapabilityError::PayloadBudgetExceedsContractLimit)
        );
        assert_eq!(
            TransportCapabilities::new(
                false,
                false,
                1000,
                TransportMode::Online,
                vec![transport_profile()],
                vec![envelope_profile()],
            ),
            Err(CapabilityError::NoDirectionSupported)
        );
        assert_eq!(
            TransportCapabilities::new(
                true,
                true,
                1000,
                TransportMode::Online,
                vec![],
                vec![envelope_profile()],
            ),
            Err(CapabilityError::NoTransportProfilesDeclared)
        );
        assert_eq!(
            TransportCapabilities::new(
                true,
                true,
                1000,
                TransportMode::Online,
                vec![transport_profile()],
                vec![],
            ),
            Err(CapabilityError::NoEnvelopeProfilesDeclared)
        );
    }
}
