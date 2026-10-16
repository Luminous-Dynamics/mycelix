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

const MAX_IDENTIFIER_BYTES: usize = 64;
const MAX_MESSAGE_ID_BYTES: usize = 128;

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
    profiles: Vec<TransportProfileId>,
}

impl TransportCapabilities {
    pub fn new(
        can_send: bool,
        can_receive: bool,
        max_envelope_bytes: usize,
        mode: TransportMode,
        profiles: Vec<TransportProfileId>,
    ) -> Result<Self, CapabilityError> {
        if !can_send && !can_receive {
            return Err(CapabilityError::NoDirectionSupported);
        }
        if max_envelope_bytes == 0 {
            return Err(CapabilityError::ZeroPayloadBudget);
        }
        if profiles.is_empty() {
            return Err(CapabilityError::NoProfilesDeclared);
        }
        let unique_count = profiles.iter().collect::<BTreeSet<_>>().len();
        if unique_count != profiles.len() {
            return Err(CapabilityError::DuplicateProfile);
        }
        Ok(Self {
            can_send,
            can_receive,
            max_envelope_bytes,
            mode,
            profiles,
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

    pub fn supports_profile(&self, profile_id: &TransportProfileId) -> bool {
        self.profiles.iter().any(|profile| profile == profile_id)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CapabilityError {
    NoDirectionSupported,
    ZeroPayloadBudget,
    NoProfilesDeclared,
    DuplicateProfile,
}

impl fmt::Display for CapabilityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NoDirectionSupported => f.write_str("adapter must support send or receive"),
            Self::ZeroPayloadBudget => f.write_str("max envelope size must be greater than zero"),
            Self::NoProfilesDeclared => {
                f.write_str("at least one transport profile must be declared")
            }
            Self::DuplicateProfile => f.write_str("transport profile IDs must be unique"),
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
/// The profile ID describes the transport framing/adapter profile, not an
/// encryption suite. Construction validates basic metadata only. It does not
/// prove that the payload is encrypted or signed; the caller must pass bytes
/// produced by Pulse's existing cryptographic pipeline.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct OpaqueEnvelope {
    logical_message_id: String,
    profile_id: TransportProfileId,
    payload: Vec<u8>,
    expires_at_unix_millis: UnixMillis,
}

impl OpaqueEnvelope {
    pub fn new(
        logical_message_id: impl Into<String>,
        profile_id: TransportProfileId,
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
        Ok(Self {
            logical_message_id,
            profile_id,
            payload,
            expires_at_unix_millis,
        })
    }

    pub fn logical_message_id(&self) -> &str {
        &self.logical_message_id
    }

    pub fn profile_id(&self) -> &TransportProfileId {
        &self.profile_id
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
}

impl fmt::Display for EnvelopeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidLogicalMessageId => f.write_str("logical message ID is invalid"),
            Self::EmptyPayload => f.write_str("opaque envelope payload must not be empty"),
        }
    }
}

impl std::error::Error for EnvelopeError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RoutePolicy {
    pub direction: Direction,
    pub required_profile_id: TransportProfileId,
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
    if envelope.profile_id() != &policy.required_profile_id {
        return Err(IneligibilityReason::EnvelopeProfileMismatch);
    }

    let capabilities = candidate.descriptor.capabilities();
    if !capabilities.supports_profile(&policy.required_profile_id) {
        return Err(IneligibilityReason::ProfileUnsupported);
    }
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

/// An adapter can report only what its own layer observed. It cannot claim
/// recipient delivery; that requires Pulse's separate receipt verifier.
pub trait TransportAdapter {
    fn descriptor(&self) -> &AdapterDescriptor;

    fn observe(&mut self, now_monotonic: MonotonicMillis) -> LinkObservation;

    fn send_opaque(&mut self, envelope: &OpaqueEnvelope) -> AdapterSendOutcome;
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
            Self::InvalidReceipt => f.write_str("recipient receipt failed authentication or binding checks"),
        }
    }
}

impl std::error::Error for ReceiptVerificationError {}

/// Security-critical integration point implemented by Pulse's trusted
/// cryptographic receipt-verification module, never by a transport adapter.
///
/// Implementations MUST verify the receipt's authenticating signature/MAC,
/// logical message ID, intended recipient identity, and any required protocol
/// or freshness fields. The trait is deliberately explicit because this
/// crate has no dependency on Pulse's crypto implementation.
pub trait TrustedRecipientReceiptVerifier {
    fn verify(
        &self,
        encoded_receipt: &[u8],
        expected_logical_message_id: &str,
        expected_recipient_id: &str,
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
}

impl VerifiedRecipientReceipt {
    pub fn verify<V: TrustedRecipientReceiptVerifier>(
        verifier: &V,
        encoded_receipt: &[u8],
        expected_logical_message_id: &str,
        expected_recipient_id: &str,
    ) -> Result<Self, ReceiptVerificationError> {
        if !valid_receipt_binding(expected_logical_message_id, MAX_MESSAGE_ID_BYTES)
            || !valid_receipt_binding(expected_recipient_id, 512)
            || encoded_receipt.is_empty()
        {
            return Err(ReceiptVerificationError::InvalidContext);
        }
        verifier.verify(
            encoded_receipt,
            expected_logical_message_id,
            expected_recipient_id,
        )?;
        Ok(Self {
            logical_message_id: expected_logical_message_id.to_owned(),
            recipient_id: expected_recipient_id.to_owned(),
        })
    }

    fn matches(&self, logical_message_id: &str, recipient_id: &str) -> bool {
        self.logical_message_id == logical_message_id && self.recipient_id == recipient_id
    }
}

fn valid_receipt_binding(value: &str, maximum_bytes: usize) -> bool {
    !value.trim().is_empty()
        && value.len() <= maximum_bytes
        && !value.chars().any(char::is_control)
}

/// Converts transport-layer evidence to user-visible state. No transport
/// outcome maps to Delivered. A separately verified token must match both the
/// current logical message ID and the intended recipient.
pub fn user_visible_delivery_state(
    stage: DeliveryEvidenceStage,
    logical_message_id: &str,
    recipient_id: &str,
    verified_receipt: Option<&VerifiedRecipientReceipt>,
) -> UserVisibleDeliveryState {
    if verified_receipt.is_some_and(|receipt| {
        receipt.matches(logical_message_id, recipient_id)
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

    fn profile() -> TransportProfileId {
        TransportProfileId::new("quic-envelope-v1").unwrap()
    }

    fn descriptor(
        id: &str,
        domain: &str,
        max_bytes: usize,
        mode: TransportMode,
    ) -> AdapterDescriptor {
        AdapterDescriptor::new(
            TransportId::new(id).unwrap(),
            FailureDomainId::new(domain).unwrap(),
            TransportCapabilities::new(true, true, max_bytes, mode, vec![profile()]).unwrap(),
        )
    }

    fn envelope(bytes: usize, expires_at_unix_millis: UnixMillis) -> OpaqueEnvelope {
        OpaqueEnvelope::new(
            "msg-0001",
            profile(),
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
            required_profile_id: profile(),
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
            evaluate_route(&no_consent, &policy(), &envelope(100, UnixMillis::new(20_000)), UnixMillis::new(1_500), MonotonicMillis::new(1_500)),
            Err(IneligibilityReason::UserConsentUnknown)
        );

        let mut no_regulatory_proof = base.clone();
        no_regulatory_proof.regulatory_permission = PolicyDecision::Unknown;
        assert_eq!(
            evaluate_route(&no_regulatory_proof, &policy(), &envelope(100, UnixMillis::new(20_000)), UnixMillis::new(1_500), MonotonicMillis::new(1_500)),
            Err(IneligibilityReason::RegulatoryPermissionUnknown)
        );

        let mut unknown_privacy = base;
        unknown_privacy.privacy_permission = PolicyDecision::Unknown;
        assert_eq!(
            evaluate_route(&unknown_privacy, &policy(), &envelope(100, UnixMillis::new(20_000)), UnixMillis::new(1_500), MonotonicMillis::new(1_500)),
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
            evaluate_route(&candidate(&descriptor, stale), &policy, &envelope, UnixMillis::new(1_500), MonotonicMillis::new(1_100)),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );

        let future = LinkObservation {
            state: LinkState::Available,
            observed_at_monotonic: MonotonicMillis::new(2_000),
            valid_for_millis: 500,
        };
        assert_eq!(
            evaluate_route(&candidate(&descriptor, future), &policy, &envelope, UnixMillis::new(1_500), MonotonicMillis::new(1_500)),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );

        let unknown = LinkObservation {
            state: LinkState::Unknown,
            observed_at_monotonic: 1_000,
            valid_for_millis: 500,
        };
        assert_eq!(
            evaluate_route(&candidate(&descriptor, unknown), &policy, &envelope, UnixMillis::new(1_500), MonotonicMillis::new(1_100)),
            Err(IneligibilityReason::LinkAvailabilityUnknown)
        );
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
                1_500,
            ),
            Err(IneligibilityReason::PayloadTooLarge)
        );
        assert_eq!(
            evaluate_route(
                &candidate(&small, available_observation()),
                &policy(),
                &envelope(50, UnixMillis::new(20_000)),
                20_000,
            ),
            Err(IneligibilityReason::EnvelopeExpired)
        );

        let mut unsupported_profile_policy = policy();
        unsupported_profile_policy.required_profile_id =
            TransportProfileId::new("bluetooth-gatt-v1").unwrap();
        assert_eq!(
            evaluate_route(
                &candidate(&small, available_observation()),
                &unsupported_profile_policy,
                &envelope(50, UnixMillis::new(20_000)),
                1_500,
            ),
            Err(IneligibilityReason::EnvelopeProfileMismatch)
        );

        let receive_only = AdapterDescriptor::new(
            TransportId::new("receive-only").unwrap(),
            FailureDomainId::new("receive-only-domain").unwrap(),
            TransportCapabilities::new(
                false,
                true,
                1000,
                TransportMode::Online,
                vec![profile()],
            )
            .unwrap(),
        );
        assert_eq!(
            evaluate_route(
                &candidate(&receive_only, available_observation()),
                &policy(),
                &envelope(50, UnixMillis::new(20_000)),
                1_500,
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
                1_500,
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
            evaluate_route(&candidate, &policy(), &envelope(100, UnixMillis::new(20_000)), UnixMillis::new(1_500), MonotonicMillis::new(1_500)),
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
                    1_500,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        assert_eq!(distinct_failure_domain_count(&routes), 2);
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
                user_visible_delivery_state(expected, "msg-1", "recipient-1", None),
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
        ) -> Result<(), ReceiptVerificationError> {
            if encoded_receipt == b"valid-test-receipt"
                && expected_logical_message_id == "msg-1"
                && expected_recipient_id == "recipient-1"
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
        )
        .unwrap();
        assert_eq!(
            user_visible_delivery_state(
                DeliveryEvidenceStage::GatewayAcknowledged,
                "msg-1",
                "recipient-1",
                Some(&receipt),
            ),
            UserVisibleDeliveryState::Delivered
        );
        assert_eq!(
            user_visible_delivery_state(
                DeliveryEvidenceStage::GatewayAcknowledged,
                "msg-2",
                "recipient-1",
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
            ),
            Err(ReceiptVerificationError::InvalidReceipt)
        );
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                b"",
                "msg-1",
                "recipient-1",
            ),
            Err(ReceiptVerificationError::InvalidContext)
        );
        assert_eq!(
            VerifiedRecipientReceipt::verify(
                &verifier,
                b"valid-test-receipt",
                "",
                "recipient-1",
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
                user_visible_delivery_state(stage, "msg-1", "recipient-1", None),
                UserVisibleDeliveryState::Delivered
            );
        }
    }

    #[test]
    fn constructors_reject_ambiguous_or_unbounded_metadata() {
        assert_eq!(TransportId::new(""), Err(IdentifierError::Empty));
        assert_eq!(TransportId::new("MixedCase"), Err(IdentifierError::InvalidCharacter));
        assert_eq!(
            OpaqueEnvelope::new("msg-1", profile(), Vec::new(), 100),
            Err(EnvelopeError::EmptyPayload)
        );
        assert_eq!(
            TransportCapabilities::new(false, false, 1000, TransportMode::Online, vec![profile()]),
            Err(CapabilityError::NoDirectionSupported)
        );
        assert_eq!(
            TransportCapabilities::new(true, true, 1000, TransportMode::Online, vec![]),
            Err(CapabilityError::NoProfilesDeclared)
        );
    }
}
