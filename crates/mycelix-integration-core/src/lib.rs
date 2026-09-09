//! Provider-neutral semantic kernel for the Mycelix integration plane.
//!
//! This crate intentionally contains no network client, HTTP server, Holochain,
//! database, secret-manager, vendor SDK, or async-runtime dependency. It defines
//! only the semantic objects that higher layers may persist, verify, project,
//! reconcile, or execute.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fmt;

const MAX_TOKEN_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationError {
    Empty,
    TooLong { max: usize, actual: usize },
    ContainsControl,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty => write!(f, "value must not be empty"),
            Self::TooLong { max, actual } => {
                write!(f, "value is too long: {actual} bytes, maximum is {max}")
            }
            Self::ContainsControl => write!(f, "value must not contain control characters"),
        }
    }
}

impl std::error::Error for ValidationError {}

fn validate_token(value: &str) -> Result<(), ValidationError> {
    if value.is_empty() {
        return Err(ValidationError::Empty);
    }
    if value.len() > MAX_TOKEN_LEN {
        return Err(ValidationError::TooLong {
            max: MAX_TOKEN_LEN,
            actual: value.len(),
        });
    }
    if value.chars().any(char::is_control) {
        return Err(ValidationError::ContainsControl);
    }
    Ok(())
}

macro_rules! id_type {
    ($name:ident) => {
        #[derive(
            Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ValidationError> {
                let value = value.into();
                validate_token(&value)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

id_type!(IntegrationEventId);
id_type!(IntegrationCommandId);
id_type!(ExternalSystemId);
id_type!(ConnectorInstanceId);
id_type!(ExternalObjectType);
id_type!(ExternalActorType);
id_type!(ExternalOpaqueId);
id_type!(ExternalEventType);
id_type!(ExternalOperationKind);
id_type!(ExternalRejectionCode);
id_type!(MycelixSubjectRef);
id_type!(DomainObjectRef);
id_type!(MappingProfileId);
id_type!(SchemaVersion);
id_type!(SemanticProfileId);
id_type!(ProjectionProfileId);
id_type!(VerificationProfileId);
id_type!(CorrelationId);
id_type!(CausationId);
id_type!(IdempotencyKey);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DigestAlgorithm {
    Sha256,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ContentCommitment {
    pub algorithm: DigestAlgorithm,
    pub digest: [u8; 32],
}

impl ContentCommitment {
    pub fn sha256(bytes: &[u8]) -> Self {
        let digest: [u8; 32] = Sha256::digest(bytes).into();
        Self {
            algorithm: DigestAlgorithm::Sha256,
            digest,
        }
    }

    pub fn matches(&self, bytes: &[u8]) -> bool {
        match self.algorithm {
            DigestAlgorithm::Sha256 => self == &Self::sha256(bytes),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalObjectRef {
    pub system: ExternalSystemId,
    pub object_type: ExternalObjectType,
    pub external_id: ExternalOpaqueId,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalActorRef {
    pub system: ExternalSystemId,
    pub actor_type: ExternalActorType,
    pub external_id: ExternalOpaqueId,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalIdentityBinding {
    pub external_actor: ExternalActorRef,
    pub mycelix_subject: MycelixSubjectRef,
    pub mapping_profile: MappingProfileId,
    pub evidence: ContentCommitment,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalObjectBinding {
    pub external_object: ExternalObjectRef,
    pub domain_object: DomainObjectRef,
    pub mapping_profile: MappingProfileId,
    pub evidence: ContentCommitment,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerificationEvidence {
    pub profile: VerificationProfileId,
    pub evidence_commitment: ContentCommitment,
    pub verified_at_ms: i64,
    pub expires_at_ms: Option<i64>,
}

impl VerificationEvidence {
    pub fn valid_at(&self, at_ms: i64) -> bool {
        at_ms >= self.verified_at_ms
            && self
                .expires_at_ms
                .is_none_or(|expires_at_ms| at_ms <= expires_at_ms)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExternalTrustState {
    Observed,
    Authenticated,
    Verified,
    Reconciled,
    CrossAttested,
    Finalized,
    Rejected,
}

impl ExternalTrustState {
    pub fn allows_transition_to(self, next: Self) -> bool {
        use ExternalTrustState::{
            Authenticated, CrossAttested, Finalized, Observed, Reconciled, Rejected, Verified,
        };

        matches!(
            (self, next),
            (Observed, Authenticated)
                | (Observed, Verified)
                | (Observed, Rejected)
                | (Authenticated, Verified)
                | (Authenticated, Rejected)
                | (Verified, Reconciled)
                | (Verified, Rejected)
                | (Reconciled, CrossAttested)
                | (Reconciled, Finalized)
                | (Reconciled, Rejected)
                | (CrossAttested, Finalized)
                | (CrossAttested, Rejected)
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationEventEnvelope {
    pub event_id: IntegrationEventId,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub external_event_type: ExternalEventType,
    pub subject: Option<ExternalObjectRef>,
    pub schema_version: SchemaVersion,
    pub mapping_profile: MappingProfileId,
    pub semantic_profile: SemanticProfileId,
    pub observed_at_ms: i64,
    pub content_commitment: ContentCommitment,
    pub correlation_id: Option<CorrelationId>,
    pub causation_id: Option<CausationId>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationEvent<P> {
    pub envelope: IntegrationEventEnvelope,
    pub payload: P,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectionContext {
    pub mapping_profile: MappingProfileId,
    pub schema_version: SchemaVersion,
    pub binding_set_commitment: Option<ContentCommitment>,
    pub policy_context_commitment: Option<ContentCommitment>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProjectionResult<C> {
    pub proposed_command: C,
    pub source_event_commitment: ContentCommitment,
    pub projection_profile: ProjectionProfileId,
}

/// Pure semantic projection boundary.
///
/// Implementations should depend only on the supplied event and explicit
/// `ProjectionContext`. Wall-clock time, randomness, network I/O, hidden DB
/// lookups, and mutable global configuration do not belong in this interface.
pub trait IntegrationProjection {
    type Payload;
    type DomainCommand;
    type Error;

    fn project(
        &self,
        event: &IntegrationEvent<Self::Payload>,
        context: &ProjectionContext,
    ) -> Result<ProjectionResult<Self::DomainCommand>, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SideEffectClass {
    ReadOnly,
    Reversible,
    Compensatable,
    Irreversible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RetryPolicyClass {
    BoundedNormalRetry,
    IdempotentOnly,
    DurableSaga,
    ReconcileBeforeReexecution,
}

impl SideEffectClass {
    pub fn required_retry_policy(self) -> RetryPolicyClass {
        match self {
            Self::ReadOnly => RetryPolicyClass::BoundedNormalRetry,
            Self::Reversible => RetryPolicyClass::IdempotentOnly,
            Self::Compensatable => RetryPolicyClass::DurableSaga,
            Self::Irreversible => RetryPolicyClass::ReconcileBeforeReexecution,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntegrationCommand<C> {
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub operation_kind: ExternalOperationKind,
    pub target: Option<ExternalObjectRef>,
    pub side_effect_class: SideEffectClass,
    pub idempotency_key: Option<IdempotencyKey>,
    pub semantic_profile: SemanticProfileId,
    pub payload: C,
}

/// Exact external-operation identity known to the integration plane.
///
/// `provider_operation` is optional because ambiguity can exist before the
/// provider returns a durable operation identifier.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalOperationRef {
    pub command_id: IntegrationCommandId,
    pub connector_instance: ConnectorInstanceId,
    pub provider_operation: Option<ExternalOpaqueId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReconciliationStrategy {
    ExactOperation,
    IdempotencyKey,
    ObjectLookup,
    CursorScan,
    ManualReview,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationHint {
    pub strategy: ReconciliationStrategy,
    pub object: Option<ExternalObjectRef>,
    pub idempotency_key: Option<IdempotencyKey>,
    pub earliest_retry_at_ms: Option<i64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalReceipt {
    pub operation: ExternalOperationRef,
    pub provider_receipt: Option<ExternalOpaqueId>,
    pub receipt_commitment: ContentCommitment,
    pub confirmed_at_ms: i64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalRejection {
    pub code: ExternalRejectionCode,
    pub detail_commitment: Option<ContentCommitment>,
    pub rejected_at_ms: i64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", content = "data", rename_all = "snake_case")]
pub enum ExternalExecutionOutcome {
    Confirmed(ExternalReceipt),
    Rejected {
        reason: ExternalRejection,
    },
    Ambiguous {
        operation: ExternalOperationRef,
        reconciliation_hint: ReconciliationHint,
    },
}

impl ExternalExecutionOutcome {
    pub fn requires_reconciliation_before_reexecution(
        &self,
        side_effect_class: SideEffectClass,
    ) -> bool {
        matches!(self, Self::Ambiguous { .. })
            && matches!(side_effect_class, SideEffectClass::Irreversible)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconcileCursor {
    pub connector_instance: ConnectorInstanceId,
    pub cursor: ExternalOpaqueId,
    pub checkpoint_commitment: ContentCommitment,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReconciliationDisposition {
    ConfirmsEffect,
    ConfirmsNoEffect,
    StillAmbiguous,
    Superseded,
    Rejected,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationResult {
    pub operation: ExternalOperationRef,
    pub disposition: ReconciliationDisposition,
    pub evidence: ContentCommitment,
    pub reconciled_at_ms: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InboundStage {
    Received,
    Authenticated,
    Decoded,
    Normalized,
    Persisted,
    Projected,
    DomainAccepted,
    Reconciled,
    Finalized,
    Rejected,
    Duplicate,
    DomainRejected,
}

impl InboundStage {
    pub fn allows_transition_to(self, next: Self) -> bool {
        use InboundStage::{
            Authenticated, Decoded, DomainAccepted, DomainRejected, Duplicate, Finalized,
            Normalized, Persisted, Projected, Received, Reconciled, Rejected,
        };

        matches!(
            (self, next),
            (Received, Authenticated)
                | (Received, Rejected)
                | (Authenticated, Duplicate)
                | (Authenticated, Decoded)
                | (Authenticated, Rejected)
                | (Decoded, Normalized)
                | (Decoded, Rejected)
                | (Normalized, Persisted)
                | (Normalized, Rejected)
                | (Persisted, Projected)
                | (Persisted, Rejected)
                | (Projected, DomainAccepted)
                | (Projected, DomainRejected)
                | (DomainAccepted, Reconciled)
                | (DomainAccepted, Finalized)
                | (Reconciled, Finalized)
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OutboundStage {
    Proposed,
    AuthorityChecked,
    Approved,
    OutboxCommitted,
    Executing,
    Confirmed,
    Rejected,
    Ambiguous,
    Reconciled,
    Finalized,
}

impl OutboundStage {
    pub fn allows_transition_to(self, next: Self) -> bool {
        use OutboundStage::{
            Ambiguous, Approved, AuthorityChecked, Confirmed, Executing, Finalized,
            OutboxCommitted, Proposed, Reconciled, Rejected,
        };

        matches!(
            (self, next),
            (Proposed, AuthorityChecked)
                | (AuthorityChecked, Approved)
                | (AuthorityChecked, Rejected)
                | (Approved, OutboxCommitted)
                | (OutboxCommitted, Executing)
                | (Executing, Confirmed)
                | (Executing, Rejected)
                | (Executing, Ambiguous)
                | (Confirmed, Reconciled)
                | (Confirmed, Finalized)
                | (Ambiguous, Reconciled)
                | (Reconciled, Finalized)
        )
    }
}

pub trait CanonicalEncodeV1 {
    fn canonical_preimage_v1(&self) -> Vec<u8>;

    fn canonical_commitment_v1(&self) -> ContentCommitment {
        ContentCommitment::sha256(&self.canonical_preimage_v1())
    }
}

fn push_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_be_bytes());
    out.extend_from_slice(bytes);
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    push_bytes(out, value.as_bytes());
}

fn push_optional_str(out: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => {
            out.push(1);
            push_str(out, value);
        }
        None => out.push(0),
    }
}

fn push_commitment(out: &mut Vec<u8>, commitment: &ContentCommitment) {
    match commitment.algorithm {
        DigestAlgorithm::Sha256 => out.push(1),
    }
    out.extend_from_slice(&commitment.digest);
}

impl CanonicalEncodeV1 for ExternalObjectRef {
    fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut out = b"MYCELIX-INTEGRATION-EXTERNAL-OBJECT\0V1\0".to_vec();
        push_str(&mut out, self.system.as_str());
        push_str(&mut out, self.object_type.as_str());
        push_str(&mut out, self.external_id.as_str());
        out
    }
}

impl CanonicalEncodeV1 for IntegrationEventEnvelope {
    fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut out = b"MYCELIX-INTEGRATION-EVENT\0V1\0".to_vec();
        push_str(&mut out, self.event_id.as_str());
        push_str(&mut out, self.connector_instance.as_str());
        push_str(&mut out, self.system.as_str());
        push_str(&mut out, self.external_event_type.as_str());

        match &self.subject {
            Some(subject) => {
                out.push(1);
                push_bytes(&mut out, &subject.canonical_preimage_v1());
            }
            None => out.push(0),
        }

        push_str(&mut out, self.schema_version.as_str());
        push_str(&mut out, self.mapping_profile.as_str());
        push_str(&mut out, self.semantic_profile.as_str());
        out.extend_from_slice(&self.observed_at_ms.to_be_bytes());
        push_commitment(&mut out, &self.content_commitment);
        push_optional_str(&mut out, self.correlation_id.as_ref().map(CorrelationId::as_str));
        push_optional_str(&mut out, self.causation_id.as_ref().map(CausationId::as_str));
        out
    }
}

impl<C> CanonicalEncodeV1 for IntegrationCommand<C>
where
    C: CanonicalEncodeV1,
{
    fn canonical_preimage_v1(&self) -> Vec<u8> {
        let mut out = b"MYCELIX-INTEGRATION-COMMAND\0V1\0".to_vec();
        push_str(&mut out, self.command_id.as_str());
        push_str(&mut out, self.connector_instance.as_str());
        push_str(&mut out, self.system.as_str());
        push_str(&mut out, self.operation_kind.as_str());

        match &self.target {
            Some(target) => {
                out.push(1);
                push_bytes(&mut out, &target.canonical_preimage_v1());
            }
            None => out.push(0),
        }

        out.push(match self.side_effect_class {
            SideEffectClass::ReadOnly => 0,
            SideEffectClass::Reversible => 1,
            SideEffectClass::Compensatable => 2,
            SideEffectClass::Irreversible => 3,
        });
        push_optional_str(&mut out, self.idempotency_key.as_ref().map(IdempotencyKey::as_str));
        push_str(&mut out, self.semantic_profile.as_str());
        push_bytes(&mut out, &self.payload.canonical_preimage_v1());
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        match constructor(value.to_owned()) {
            Ok(value) => value,
            Err(error) => panic!("test fixture id must be valid: {error}"),
        }
    }

    fn system() -> ExternalSystemId {
        id("stripe", ExternalSystemId::new)
    }

    fn connector() -> ConnectorInstanceId {
        id("stripe-prod-eu-1", ConnectorInstanceId::new)
    }

    fn subject() -> ExternalObjectRef {
        ExternalObjectRef {
            system: system(),
            object_type: id("payment_intent", ExternalObjectType::new),
            external_id: id("pi_123", ExternalOpaqueId::new),
        }
    }

    fn envelope() -> IntegrationEventEnvelope {
        IntegrationEventEnvelope {
            event_id: id("evt_123", IntegrationEventId::new),
            connector_instance: connector(),
            system: system(),
            external_event_type: id("payment.succeeded", ExternalEventType::new),
            subject: Some(subject()),
            schema_version: id("stripe-2026-08-15", SchemaVersion::new),
            mapping_profile: id("stripe-payment-v1", MappingProfileId::new),
            semantic_profile: id("payments.external-observation@1", SemanticProfileId::new),
            observed_at_ms: 1_789_000_000_000,
            content_commitment: ContentCommitment::sha256(b"raw-provider-body"),
            correlation_id: Some(id("corr-1", CorrelationId::new)),
            causation_id: None,
        }
    }

    #[test]
    fn ids_reject_empty_and_control_characters() {
        assert_eq!(ExternalSystemId::new(""), Err(ValidationError::Empty));
        assert_eq!(
            ExternalSystemId::new("bad\nvalue"),
            Err(ValidationError::ContainsControl)
        );
    }

    #[test]
    fn content_commitment_checks_exact_bytes() {
        let commitment = ContentCommitment::sha256(b"abc");
        assert!(commitment.matches(b"abc"));
        assert!(!commitment.matches(b"abd"));
    }

    #[test]
    fn canonical_event_commitment_is_deterministic_and_semantic() {
        let first = envelope();
        let second = envelope();
        assert_eq!(first.canonical_commitment_v1(), second.canonical_commitment_v1());

        let mut changed = envelope();
        changed.mapping_profile = id("stripe-payment-v2", MappingProfileId::new);
        assert_ne!(
            first.canonical_commitment_v1(),
            changed.canonical_commitment_v1()
        );
    }

    #[test]
    fn trust_state_cannot_move_backwards_or_skip_reconciliation_after_verified() {
        assert!(ExternalTrustState::Observed.allows_transition_to(ExternalTrustState::Verified));
        assert!(
            ExternalTrustState::Verified.allows_transition_to(ExternalTrustState::Reconciled)
        );
        assert!(!ExternalTrustState::Verified.allows_transition_to(ExternalTrustState::Observed));
        assert!(!ExternalTrustState::Verified.allows_transition_to(ExternalTrustState::Finalized));
    }

    #[test]
    fn irreversible_ambiguous_execution_requires_reconciliation() {
        let outcome = ExternalExecutionOutcome::Ambiguous {
            operation: ExternalOperationRef {
                command_id: id("cmd-1", IntegrationCommandId::new),
                connector_instance: connector(),
                provider_operation: None,
            },
            reconciliation_hint: ReconciliationHint {
                strategy: ReconciliationStrategy::IdempotencyKey,
                object: Some(subject()),
                idempotency_key: Some(id("idem-1", IdempotencyKey::new)),
                earliest_retry_at_ms: None,
            },
        };

        assert!(outcome.requires_reconciliation_before_reexecution(SideEffectClass::Irreversible));
        assert!(!outcome.requires_reconciliation_before_reexecution(SideEffectClass::ReadOnly));
    }

    #[test]
    fn side_effect_class_selects_minimum_retry_policy() {
        assert_eq!(
            SideEffectClass::ReadOnly.required_retry_policy(),
            RetryPolicyClass::BoundedNormalRetry
        );
        assert_eq!(
            SideEffectClass::Compensatable.required_retry_policy(),
            RetryPolicyClass::DurableSaga
        );
        assert_eq!(
            SideEffectClass::Irreversible.required_retry_policy(),
            RetryPolicyClass::ReconcileBeforeReexecution
        );
    }

    #[test]
    fn inbound_state_machine_forbids_observation_to_consequence_shortcut() {
        assert!(InboundStage::Received.allows_transition_to(InboundStage::Authenticated));
        assert!(!InboundStage::Received.allows_transition_to(InboundStage::DomainAccepted));
        assert!(!InboundStage::Authenticated.allows_transition_to(InboundStage::Projected));
        assert!(InboundStage::Projected.allows_transition_to(InboundStage::DomainRejected));
    }

    #[test]
    fn outbound_state_machine_requires_durable_outbox_before_execution() {
        assert!(OutboundStage::Approved.allows_transition_to(OutboundStage::OutboxCommitted));
        assert!(!OutboundStage::Approved.allows_transition_to(OutboundStage::Executing));
        assert!(OutboundStage::OutboxCommitted.allows_transition_to(OutboundStage::Executing));
        assert!(OutboundStage::Executing.allows_transition_to(OutboundStage::Ambiguous));
        assert!(!OutboundStage::Ambiguous.allows_transition_to(OutboundStage::Executing));
        assert!(OutboundStage::Ambiguous.allows_transition_to(OutboundStage::Reconciled));
    }

    #[test]
    fn verification_evidence_has_explicit_validity_window() {
        let evidence = VerificationEvidence {
            profile: id("stripe-webhook-signature-v1", VerificationProfileId::new),
            evidence_commitment: ContentCommitment::sha256(b"verification receipt"),
            verified_at_ms: 100,
            expires_at_ms: Some(200),
        };

        assert!(!evidence.valid_at(99));
        assert!(evidence.valid_at(100));
        assert!(evidence.valid_at(200));
        assert!(!evidence.valid_at(201));
    }

    #[test]
    fn ambiguous_outcome_round_trips_through_serde() {
        let outcome = ExternalExecutionOutcome::Ambiguous {
            operation: ExternalOperationRef {
                command_id: id("cmd-2", IntegrationCommandId::new),
                connector_instance: connector(),
                provider_operation: Some(id("provider-op-9", ExternalOpaqueId::new)),
            },
            reconciliation_hint: ReconciliationHint {
                strategy: ReconciliationStrategy::ExactOperation,
                object: None,
                idempotency_key: None,
                earliest_retry_at_ms: Some(500),
            },
        };

        let json = match serde_json::to_string(&outcome) {
            Ok(json) => json,
            Err(error) => panic!("serialization must succeed: {error}"),
        };
        let decoded: ExternalExecutionOutcome = match serde_json::from_str(&json) {
            Ok(value) => value,
            Err(error) => panic!("deserialization must succeed: {error}"),
        };
        assert_eq!(outcome, decoded);
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct DemoPayload {
        amount_minor: u64,
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct DemoCommand {
        amount_minor: u64,
    }

    struct DemoProjection;

    impl IntegrationProjection for DemoProjection {
        type Payload = DemoPayload;
        type DomainCommand = DemoCommand;
        type Error = ();

        fn project(
            &self,
            event: &IntegrationEvent<Self::Payload>,
            _context: &ProjectionContext,
        ) -> Result<ProjectionResult<Self::DomainCommand>, Self::Error> {
            Ok(ProjectionResult {
                proposed_command: DemoCommand {
                    amount_minor: event.payload.amount_minor,
                },
                source_event_commitment: event.envelope.canonical_commitment_v1(),
                projection_profile: id("demo-projection-v1", ProjectionProfileId::new),
            })
        }
    }

    #[test]
    fn projection_contract_is_replayable_for_same_explicit_inputs() {
        let projection = DemoProjection;
        let event = IntegrationEvent {
            envelope: envelope(),
            payload: DemoPayload { amount_minor: 5000 },
        };
        let context = ProjectionContext {
            mapping_profile: id("stripe-payment-v1", MappingProfileId::new),
            schema_version: id("stripe-2026-08-15", SchemaVersion::new),
            binding_set_commitment: None,
            policy_context_commitment: None,
        };

        let first = projection.project(&event, &context);
        let second = projection.project(&event, &context);
        assert_eq!(first, second);
    }
}
