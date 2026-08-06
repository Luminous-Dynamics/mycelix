// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Append-only scientific event kernel.
//!
//! This module is intentionally independent from the legacy mutable
//! [`crate::claims::DesciClaim`] model. It establishes the invariants needed for
//! the architectural refoundation without forcing an unsafe flag-day migration:
//!
//! - research objects are distinct from atomic claims;
//! - evidence and attestations are typed rather than represented as counters;
//! - every state transition is an immutable, signed event;
//! - signed bytes use an explicit, versioned canonical codec;
//! - event streams use optimistic concurrency, idempotency, and hash chaining;
//! - query state is a deterministic, disposable projection;
//! - scientific maturity counts qualified independent sources, not event volume.

use crate::transactional_file::persist_json_vec_transaction;
use crate::{Error, Result};
use async_trait::async_trait;
use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap, HashSet};
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

/// Stable protocol identifier embedded into every signed event.
pub const SCIENTIFIC_EVENT_PROTOCOL: &str = "mycelix-desci";
/// Version of the public event protocol.
pub const SCIENTIFIC_EVENT_PROTOCOL_VERSION: u16 = 1;
/// Canonical codec identifier. The implementation is defined below, not by a
/// Rust serializer's enum layout.
pub const SCIENTIFIC_EVENT_CODEC: &str = "mycelix-canonical-binary-v1";
/// Schema version for the authoritative event envelope.
pub const SCIENTIFIC_EVENT_SCHEMA_VERSION: u16 = 3;
/// Oldest schema version still accepted for replay. Schema v2 is the first
/// explicit canonical-codec envelope and remains verifiable after the v3
/// migration additions.
pub const MIN_SUPPORTED_SCIENTIFIC_EVENT_SCHEMA_VERSION: u16 = 2;
/// Default explanatory assessment policy.
pub const DEFAULT_EVIDENCE_POLICY_ID: &str = "mycelix-evidence-policy";
pub const DEFAULT_EVIDENCE_POLICY_VERSION: &str = "1.0.0";
/// Client clocks may be slightly ahead, but cannot place authoritative events
/// arbitrarily into the future.
pub const MAX_EVENT_FUTURE_SKEW_SECONDS: i64 = 300;
/// Defensive bound for one durable claim-stream file. Large evidence belongs
/// in content-addressed artifact storage, not inline event JSON.
pub const MAX_EVENT_STREAM_FILE_BYTES: u64 = 64 * 1024 * 1024;

macro_rules! uuid_id {
    ($name:ident) => {
        #[derive(
            Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub Uuid);

        impl $name {
            pub fn new() -> Self {
                Self(Uuid::new_v4())
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                self.0.fmt(f)
            }
        }
    };
}

uuid_id!(ResearchObjectId);
uuid_id!(ClaimId);
uuid_id!(ArtifactId);
uuid_id!(AttestationId);
uuid_id!(ScientificEventId);

fn validate_string_identifier(value: impl Into<String>, label: &str) -> Result<String> {
    let value = value.into();
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(Error::Validation(format!("{label} cannot be empty")));
    }
    if trimmed.len() > 512 {
        return Err(Error::Validation(format!(
            "{label} cannot exceed 512 bytes"
        )));
    }
    if trimmed.chars().any(char::is_control) {
        return Err(Error::Validation(format!(
            "{label} cannot contain control characters"
        )));
    }
    Ok(trimmed.to_string())
}

fn validate_canonical_identifier(value: &str, label: &str) -> Result<()> {
    let normalized = validate_string_identifier(value.to_string(), label)?;
    if normalized != value {
        return Err(Error::Validation(format!(
            "{label} must not contain leading or trailing whitespace"
        )));
    }
    Ok(())
}

/// Stable authenticated actor identifier (for example a DID or service ID).
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ActorId(String);

impl ActorId {
    pub fn new(value: impl Into<String>) -> Result<Self> {
        Ok(Self(validate_string_identifier(value, "actor identifier")?))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<()> {
        validate_canonical_identifier(&self.0, "actor identifier")
    }
}

impl fmt::Display for ActorId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// Organization under whose authority an actor is acting for an event.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct OrganizationId(String);

impl OrganizationId {
    pub fn new(value: impl Into<String>) -> Result<Self> {
        Ok(Self(validate_string_identifier(
            value,
            "organization identifier",
        )?))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<()> {
        validate_canonical_identifier(&self.0, "organization identifier")
    }
}

impl fmt::Display for OrganizationId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// BLAKE3 digest used for event-chain and artifact integrity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ContentHash(pub [u8; 32]);

impl ContentHash {
    pub fn digest(bytes: &[u8]) -> Self {
        Self(*blake3::hash(bytes).as_bytes())
    }

    pub fn to_hex(self) -> String {
        hex::encode(self.0)
    }
}

impl fmt::Display for ContentHash {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

/// Research-object container. This is not itself an atomic truth claim.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResearchObject {
    pub id: ResearchObjectId,
    pub title: String,
    pub object_type: ResearchObjectType,
    pub persistent_identifier: Option<String>,
}

impl ResearchObject {
    pub fn validate(&self) -> Result<()> {
        if self.title.trim().is_empty() {
            return Err(Error::Validation(
                "research object title cannot be empty".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResearchObjectType {
    Manuscript,
    Dataset,
    Software,
    Workflow,
    Protocol,
    Model,
    Other(String),
}

/// One scoped proposition that can accumulate evidence and attestations.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AtomicClaim {
    pub id: ClaimId,
    pub research_object_id: ResearchObjectId,
    pub statement: String,
    pub scope: Option<String>,
}

impl AtomicClaim {
    pub fn validate(&self) -> Result<()> {
        if self.statement.trim().is_empty() {
            return Err(Error::Validation(
                "atomic claim statement cannot be empty".to_string(),
            ));
        }
        Ok(())
    }
}

/// Content-addressed evidence referenced by a claim.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceArtifact {
    pub id: ArtifactId,
    pub content_hash: ContentHash,
    pub media_type: String,
    pub locator: String,
    pub license: Option<String>,
    pub availability: ArtifactAvailability,
}

impl EvidenceArtifact {
    pub fn validate(&self) -> Result<()> {
        if self.media_type.trim().is_empty() {
            return Err(Error::Validation(
                "artifact media type cannot be empty".to_string(),
            ));
        }
        if self.locator.trim().is_empty() {
            return Err(Error::Validation(
                "artifact locator cannot be empty".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArtifactAvailability {
    Public,
    Controlled,
    Embargoed,
    Unavailable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceOutcome {
    Supports,
    DoesNotSupport,
    Inconclusive,
}

/// Typed scientific attestation. The signer and acting organization are
/// carried by the event envelope and validated by the governed append path.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Attestation {
    pub id: AttestationId,
    pub claim_id: ClaimId,
    pub kind: AttestationKind,
    pub evidence_ids: Vec<ArtifactId>,
    pub statement: Option<String>,
    /// Versioned protocol, preregistration, or method package used for a
    /// reproduction or replication. Required for those attestation kinds.
    pub protocol_reference: Option<String>,
}

impl Attestation {
    pub fn validate(&self) -> Result<()> {
        let mut unique = HashSet::new();
        if !self.evidence_ids.iter().all(|id| unique.insert(*id)) {
            return Err(Error::Validation(
                "attestation contains duplicate evidence identifiers".to_string(),
            ));
        }

        if matches!(
            self.kind,
            AttestationKind::ComputationalReproduction { .. }
                | AttestationKind::IndependentReplication { .. }
        ) {
            if self.evidence_ids.is_empty() {
                return Err(Error::Validation(
                    "reproduction and replication attestations require evidence".to_string(),
                ));
            }
            if self
                .protocol_reference
                .as_deref()
                .map(str::trim)
                .filter(|value| !value.is_empty())
                .is_none()
            {
                return Err(Error::Validation(
                    "reproduction and replication attestations require a protocol reference"
                        .to_string(),
                ));
            }
        }
        Ok(())
    }

    fn uniqueness_family(&self) -> u8 {
        match &self.kind {
            AttestationKind::Review => 1,
            AttestationKind::ComputationalReproduction { .. } => 2,
            AttestationKind::IndependentReplication { .. } => 3,
            AttestationKind::Critique => 4,
            AttestationKind::ConflictDisclosure => 5,
            AttestationKind::Correction => 6,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AttestationKind {
    Review,
    ComputationalReproduction {
        outcome: EvidenceOutcome,
    },
    IndependentReplication {
        outcome: EvidenceOutcome,
    },
    Critique,
    ConflictDisclosure,
    /// Legacy compatibility. New corrections should use
    /// [`ScientificEventPayload::AttestationCorrected`].
    Correction,
}

/// Metadata preserved when a mutable legacy claim is imported. None of these
/// historical assertions contribute to canonical evidence maturity.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyImportMetadata {
    pub source_system: String,
    pub source_record_hash: ContentHash,
    pub source_locator: Option<String>,
    /// Historical creator string from the mutable record. This is preserved
    /// for attribution display only and is not an authenticated actor binding.
    pub legacy_creator: Option<String>,
    pub legacy_tier: String,
    pub legacy_verification_count: usize,
    pub legacy_provenance_count: usize,
    pub omitted_fields: Vec<String>,
}

impl LegacyImportMetadata {
    pub fn validate(&self) -> Result<()> {
        validate_canonical_identifier(&self.source_system, "legacy source system")?;
        validate_canonical_identifier(&self.legacy_tier, "legacy tier")?;
        if let Some(locator) = &self.source_locator {
            validate_canonical_identifier(locator, "legacy source locator")?;
        }
        if let Some(creator) = &self.legacy_creator {
            validate_string_identifier(creator.clone(), "legacy creator")?;
        }
        let mut unique = BTreeSet::new();
        for field in &self.omitted_fields {
            let field = validate_string_identifier(field.clone(), "omitted legacy field")?;
            if !unique.insert(field) {
                return Err(Error::Validation(
                    "legacy import metadata contains duplicate omitted fields".to_string(),
                ));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ScientificEventPayload {
    ClaimProposed {
        research_object: ResearchObject,
        claim: AtomicClaim,
    },
    /// Explicitly imports historical mutable state without treating old tiers,
    /// verification blobs, or aggregate scores as validated evidence.
    LegacyClaimImported {
        research_object: ResearchObject,
        claim: AtomicClaim,
        import: LegacyImportMetadata,
    },
    EvidenceAttached {
        claim_id: ClaimId,
        artifact: EvidenceArtifact,
    },
    AttestationRecorded {
        attestation: Attestation,
    },
    AttestationCorrected {
        claim_id: ClaimId,
        attestation_id: AttestationId,
        replacement: Attestation,
        reason: String,
    },
    AttestationWithdrawn {
        claim_id: ClaimId,
        attestation_id: AttestationId,
        reason: String,
    },
    ClaimSuperseded {
        claim_id: ClaimId,
        replacement_claim_id: ClaimId,
        reason: String,
    },
    ClaimRetracted {
        claim_id: ClaimId,
        reason: String,
    },
}

impl ScientificEventPayload {
    pub fn claim_id(&self) -> ClaimId {
        match self {
            Self::ClaimProposed { claim, .. } | Self::LegacyClaimImported { claim, .. } => claim.id,
            Self::EvidenceAttached { claim_id, .. }
            | Self::AttestationCorrected { claim_id, .. }
            | Self::AttestationWithdrawn { claim_id, .. }
            | Self::ClaimSuperseded { claim_id, .. }
            | Self::ClaimRetracted { claim_id, .. } => *claim_id,
            Self::AttestationRecorded { attestation } => attestation.claim_id,
        }
    }

    pub fn validate(&self) -> Result<()> {
        match self {
            Self::ClaimProposed {
                research_object,
                claim,
            }
            | Self::LegacyClaimImported {
                research_object,
                claim,
                ..
            } => {
                research_object.validate()?;
                claim.validate()?;
                if claim.research_object_id != research_object.id {
                    return Err(Error::Validation(
                        "claim research_object_id does not match the proposed research object"
                            .to_string(),
                    ));
                }
                if let Self::LegacyClaimImported { import, .. } = self {
                    import.validate()?;
                }
            }
            Self::EvidenceAttached { artifact, .. } => artifact.validate()?,
            Self::AttestationRecorded { attestation } => attestation.validate()?,
            Self::AttestationCorrected {
                claim_id,
                attestation_id,
                replacement,
                reason,
            } => {
                replacement.validate()?;
                if replacement.claim_id != *claim_id {
                    return Err(Error::Validation(
                        "attestation correction must remain in the same claim stream".to_string(),
                    ));
                }
                if replacement.id != *attestation_id {
                    return Err(Error::Validation(
                        "attestation correction must preserve the attestation identifier"
                            .to_string(),
                    ));
                }
                if reason.trim().is_empty() {
                    return Err(Error::Validation(
                        "attestation correction reason cannot be empty".to_string(),
                    ));
                }
            }
            Self::AttestationWithdrawn { reason, .. } if reason.trim().is_empty() => {
                return Err(Error::Validation(
                    "attestation withdrawal reason cannot be empty".to_string(),
                ));
            }
            Self::AttestationWithdrawn { .. } => {}
            Self::ClaimSuperseded {
                claim_id,
                replacement_claim_id,
                reason,
            } => {
                if claim_id == replacement_claim_id {
                    return Err(Error::Validation(
                        "a claim cannot supersede itself".to_string(),
                    ));
                }
                if reason.trim().is_empty() {
                    return Err(Error::Validation(
                        "supersession reason cannot be empty".to_string(),
                    ));
                }
            }
            Self::ClaimRetracted { reason, .. } if reason.trim().is_empty() => {
                return Err(Error::Validation(
                    "retraction reason cannot be empty".to_string(),
                ));
            }
            Self::ClaimRetracted { .. } => {}
        }
        Ok(())
    }

    fn is_post_terminal_housekeeping(&self) -> bool {
        matches!(
            self,
            Self::AttestationCorrected { .. } | Self::AttestationWithdrawn { .. }
        )
    }
}

/// Unsigned canonical event envelope. Its explicitly encoded bytes are signed.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScientificEventEnvelope {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub event_id: ScientificEventId,
    pub stream_id: ClaimId,
    pub sequence: u64,
    pub previous_hash: Option<ContentHash>,
    pub actor: ActorId,
    pub acting_organization: Option<OrganizationId>,
    pub occurred_at: DateTime<Utc>,
    pub idempotency_key: Option<String>,
    pub payload: ScientificEventPayload,
}

impl ScientificEventEnvelope {
    pub fn genesis(
        actor: ActorId,
        occurred_at: DateTime<Utc>,
        payload: ScientificEventPayload,
    ) -> Result<Self> {
        payload.validate()?;
        Ok(Self {
            protocol: SCIENTIFIC_EVENT_PROTOCOL.to_string(),
            protocol_version: SCIENTIFIC_EVENT_PROTOCOL_VERSION,
            codec: SCIENTIFIC_EVENT_CODEC.to_string(),
            schema_version: SCIENTIFIC_EVENT_SCHEMA_VERSION,
            event_id: ScientificEventId::new(),
            stream_id: payload.claim_id(),
            sequence: 0,
            previous_hash: None,
            actor,
            acting_organization: None,
            occurred_at,
            idempotency_key: None,
            payload,
        })
    }

    pub fn next(
        previous: &SignedScientificEvent,
        actor: ActorId,
        occurred_at: DateTime<Utc>,
        payload: ScientificEventPayload,
    ) -> Result<Self> {
        payload.validate()?;
        if payload.claim_id() != previous.envelope.stream_id {
            return Err(Error::Validation(
                "event payload belongs to a different claim stream".to_string(),
            ));
        }
        Ok(Self {
            protocol: SCIENTIFIC_EVENT_PROTOCOL.to_string(),
            protocol_version: SCIENTIFIC_EVENT_PROTOCOL_VERSION,
            codec: SCIENTIFIC_EVENT_CODEC.to_string(),
            schema_version: SCIENTIFIC_EVENT_SCHEMA_VERSION,
            event_id: ScientificEventId::new(),
            stream_id: previous.envelope.stream_id,
            sequence: previous.envelope.sequence + 1,
            previous_hash: Some(previous.event_hash()?),
            actor,
            acting_organization: None,
            occurred_at,
            idempotency_key: None,
            payload,
        })
    }

    pub fn with_acting_organization(mut self, organization: OrganizationId) -> Self {
        self.acting_organization = Some(organization);
        self
    }

    pub fn with_idempotency_key(mut self, key: impl Into<String>) -> Result<Self> {
        let key = validate_string_identifier(key, "idempotency key")?;
        self.idempotency_key = Some(key);
        Ok(self)
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut encoder = CanonicalEncoder::new(b"MYCELIX-DESCI-EVENT\0");
        encoder.string(&self.protocol)?;
        encoder.u16(self.protocol_version);
        encoder.string(&self.codec)?;
        encoder.u16(self.schema_version);
        encoder.uuid(self.event_id.0);
        encoder.uuid(self.stream_id.0);
        encoder.u64(self.sequence);
        encoder.option_hash(self.previous_hash);
        encoder.string(self.actor.as_str())?;
        encoder.option_string(
            self.acting_organization
                .as_ref()
                .map(OrganizationId::as_str),
        )?;
        encoder.datetime(&self.occurred_at);
        encoder.option_string(self.idempotency_key.as_deref())?;
        encoder.payload(&self.payload)?;
        Ok(encoder.finish())
    }

    fn validate(&self) -> Result<()> {
        if self.protocol != SCIENTIFIC_EVENT_PROTOCOL {
            return Err(Error::Validation(format!(
                "unsupported scientific event protocol: {}",
                self.protocol
            )));
        }
        if self.protocol_version != SCIENTIFIC_EVENT_PROTOCOL_VERSION {
            return Err(Error::Validation(format!(
                "unsupported scientific event protocol version: {}",
                self.protocol_version
            )));
        }
        if self.codec != SCIENTIFIC_EVENT_CODEC {
            return Err(Error::Validation(format!(
                "unsupported scientific event codec: {}",
                self.codec
            )));
        }
        if !(MIN_SUPPORTED_SCIENTIFIC_EVENT_SCHEMA_VERSION..=SCIENTIFIC_EVENT_SCHEMA_VERSION)
            .contains(&self.schema_version)
        {
            return Err(Error::Validation(format!(
                "unsupported scientific event schema version: {}",
                self.schema_version
            )));
        }
        if self.schema_version < 3
            && matches!(
                &self.payload,
                ScientificEventPayload::LegacyClaimImported { .. }
            )
        {
            return Err(Error::Validation(
                "legacy_claim_imported requires scientific event schema v3".to_string(),
            ));
        }
        self.actor.validate()?;
        if let Some(organization) = &self.acting_organization {
            organization.validate()?;
        }
        if self.stream_id != self.payload.claim_id() {
            return Err(Error::Validation(
                "event stream does not match payload claim identifier".to_string(),
            ));
        }
        if self.sequence == 0 && self.previous_hash.is_some() {
            return Err(Error::Validation(
                "genesis event cannot have a previous hash".to_string(),
            ));
        }
        if self.sequence > 0 && self.previous_hash.is_none() {
            return Err(Error::Validation(
                "non-genesis event must have a previous hash".to_string(),
            ));
        }
        if let Some(key) = &self.idempotency_key {
            validate_string_identifier(key.clone(), "idempotency key")?;
        }
        self.payload.validate()
    }
}

/// Signed canonical event. Signature verification proves integrity; actor/key
/// authorization is deliberately a separate governed append concern.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SignedScientificEvent {
    pub envelope: ScientificEventEnvelope,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedScientificEvent {
    pub fn sign(envelope: ScientificEventEnvelope, signing_key: &SigningKey) -> Result<Self> {
        envelope.validate()?;
        let signature = signing_key
            .sign(&envelope.signing_bytes()?)
            .to_bytes()
            .to_vec();
        Ok(Self {
            envelope,
            signer_public_key: signing_key.verifying_key().to_bytes(),
            signature,
        })
    }

    pub fn verify(&self) -> Result<()> {
        self.envelope.validate()?;
        let verifying_key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        verifying_key
            .verify_strict(&self.envelope.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn event_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-EVENT\0");
        let signing_bytes = self.envelope.signing_bytes()?;
        push_len(&mut bytes, signing_bytes.len())?;
        bytes.extend_from_slice(&signing_bytes);
        bytes.extend_from_slice(&self.signer_public_key);
        push_len(&mut bytes, self.signature.len())?;
        bytes.extend_from_slice(&self.signature);
        Ok(ContentHash::digest(&bytes))
    }
}

/// Server-side receipt for a committed event. `received_at` is authoritative
/// ordering metadata; `occurred_at` remains the signer's claimed event time.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AppendReceipt {
    pub stream_id: ClaimId,
    pub sequence: u64,
    pub event_id: ScientificEventId,
    pub event_hash: ContentHash,
    pub received_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StreamHead {
    pub sequence: u64,
    pub event_id: ScientificEventId,
    pub event_hash: ContentHash,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventPage {
    pub events: Vec<SignedScientificEvent>,
    pub next_sequence: Option<u64>,
}

/// Append-only event-log capability with optimistic concurrency and lookup
/// methods needed for replay, migration, and idempotent command handling.
#[async_trait]
pub trait ScientificEventLog: Send + Sync {
    async fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt>;

    async fn append(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
    ) -> Result<AppendReceipt> {
        self.append_at(expected_sequence, event, Utc::now()).await
    }

    async fn head(&self, claim_id: ClaimId) -> Result<Option<StreamHead>>;

    async fn read(&self, claim_id: ClaimId, from_sequence: u64, limit: usize) -> Result<EventPage>;

    async fn stream(&self, claim_id: ClaimId) -> Result<Vec<SignedScientificEvent>>;

    /// Deterministically list every known claim stream for replay and audit.
    async fn stream_ids(&self) -> Result<Vec<ClaimId>>;

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>>;

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>>;
}

#[async_trait]
impl<T> ScientificEventLog for Arc<T>
where
    T: ScientificEventLog + ?Sized,
{
    async fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        (**self)
            .append_at(expected_sequence, event, received_at)
            .await
    }

    async fn head(&self, claim_id: ClaimId) -> Result<Option<StreamHead>> {
        (**self).head(claim_id).await
    }

    async fn read(&self, claim_id: ClaimId, from_sequence: u64, limit: usize) -> Result<EventPage> {
        (**self).read(claim_id, from_sequence, limit).await
    }

    async fn stream(&self, claim_id: ClaimId) -> Result<Vec<SignedScientificEvent>> {
        (**self).stream(claim_id).await
    }

    async fn stream_ids(&self) -> Result<Vec<ClaimId>> {
        (**self).stream_ids().await
    }

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>> {
        (**self).event_by_id(event_id).await
    }

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>> {
        (**self).event_by_hash(hash).await
    }
}

#[derive(Debug, Clone, Default)]
struct EventLogState {
    streams: HashMap<ClaimId, Vec<SignedScientificEvent>>,
    event_ids: HashMap<ScientificEventId, ClaimId>,
    event_hashes: HashMap<ContentHash, ScientificEventId>,
    idempotency_keys: HashMap<(ActorId, String), ScientificEventId>,
}

impl EventLogState {
    fn append(
        &mut self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        validate_event_time(&event, &received_at)?;
        event.verify()?;

        if event.envelope.sequence != expected_sequence {
            return Err(Error::Storage(format!(
                "optimistic concurrency failure: expected sequence {}, event carries {}",
                expected_sequence, event.envelope.sequence
            )));
        }
        if self.event_ids.contains_key(&event.envelope.event_id) {
            return Err(Error::Storage("duplicate scientific event id".to_string()));
        }
        if let Some(key) = &event.envelope.idempotency_key {
            let lookup = (event.envelope.actor.clone(), key.clone());
            if self.idempotency_keys.contains_key(&lookup) {
                return Err(Error::Storage(
                    "duplicate scientific event idempotency key".to_string(),
                ));
            }
        }

        let stream = self.streams.entry(event.envelope.stream_id).or_default();
        let actual_sequence = stream.len() as u64;
        if actual_sequence != expected_sequence {
            return Err(Error::Storage(format!(
                "optimistic concurrency failure: stream is at sequence {}, caller expected {}",
                actual_sequence, expected_sequence
            )));
        }

        validate_append_to_stream(stream, &event)?;
        let mut candidate = stream.clone();
        candidate.push(event.clone());
        ClaimProjection::rebuild(&candidate)?;

        let event_hash = event.event_hash()?;
        let receipt = AppendReceipt {
            stream_id: event.envelope.stream_id,
            sequence: event.envelope.sequence,
            event_id: event.envelope.event_id,
            event_hash,
            received_at,
        };
        self.event_ids
            .insert(event.envelope.event_id, event.envelope.stream_id);
        self.event_hashes
            .insert(event_hash, event.envelope.event_id);
        if let Some(key) = &event.envelope.idempotency_key {
            self.idempotency_keys.insert(
                (event.envelope.actor.clone(), key.clone()),
                event.envelope.event_id,
            );
        }
        stream.push(event);
        Ok(receipt)
    }

    fn rebuild_indexes(&mut self) -> Result<()> {
        self.event_ids.clear();
        self.event_hashes.clear();
        self.idempotency_keys.clear();
        for (claim_id, stream) in &self.streams {
            ClaimProjection::rebuild(stream)?;
            for event in stream {
                if self
                    .event_ids
                    .insert(event.envelope.event_id, *claim_id)
                    .is_some()
                {
                    return Err(Error::Storage(
                        "duplicate scientific event id during replay".to_string(),
                    ));
                }
                let hash = event.event_hash()?;
                if self
                    .event_hashes
                    .insert(hash, event.envelope.event_id)
                    .is_some()
                {
                    return Err(Error::Storage(
                        "duplicate scientific event hash during replay".to_string(),
                    ));
                }
                if let Some(key) = &event.envelope.idempotency_key {
                    if self
                        .idempotency_keys
                        .insert(
                            (event.envelope.actor.clone(), key.clone()),
                            event.envelope.event_id,
                        )
                        .is_some()
                    {
                        return Err(Error::Storage(
                            "duplicate idempotency key during replay".to_string(),
                        ));
                    }
                }
            }
        }
        Ok(())
    }

    fn event_by_id(&self, event_id: ScientificEventId) -> Option<SignedScientificEvent> {
        let claim_id = self.event_ids.get(&event_id)?;
        self.streams
            .get(claim_id)?
            .iter()
            .find(|event| event.envelope.event_id == event_id)
            .cloned()
    }
}

/// Deterministic in-memory event log for tests, simulation, and local development.
#[derive(Debug, Clone, Default)]
pub struct MemoryScientificEventLog {
    state: Arc<RwLock<EventLogState>>,
}

impl MemoryScientificEventLog {
    pub fn new() -> Self {
        Self::default()
    }
}

#[async_trait]
impl ScientificEventLog for MemoryScientificEventLog {
    async fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        self.state
            .write()
            .await
            .append(expected_sequence, event, received_at)
    }

    async fn head(&self, claim_id: ClaimId) -> Result<Option<StreamHead>> {
        let state = self.state.read().await;
        state
            .streams
            .get(&claim_id)
            .and_then(|stream| stream.last())
            .map(|event| {
                Ok(StreamHead {
                    sequence: event.envelope.sequence,
                    event_id: event.envelope.event_id,
                    event_hash: event.event_hash()?,
                })
            })
            .transpose()
    }

    async fn read(&self, claim_id: ClaimId, from_sequence: u64, limit: usize) -> Result<EventPage> {
        let state = self.state.read().await;
        let stream = state.streams.get(&claim_id).cloned().unwrap_or_default();
        page_events(&stream, from_sequence, limit)
    }

    async fn stream(&self, claim_id: ClaimId) -> Result<Vec<SignedScientificEvent>> {
        Ok(self
            .state
            .read()
            .await
            .streams
            .get(&claim_id)
            .cloned()
            .unwrap_or_default())
    }

    async fn stream_ids(&self) -> Result<Vec<ClaimId>> {
        let mut ids = self
            .state
            .read()
            .await
            .streams
            .keys()
            .copied()
            .collect::<Vec<_>>();
        ids.sort();
        Ok(ids)
    }

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>> {
        Ok(self.state.read().await.event_by_id(event_id))
    }

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>> {
        let state = self.state.read().await;
        let event_id = match state.event_hashes.get(&hash) {
            Some(event_id) => *event_id,
            None => return Ok(None),
        };
        Ok(state.event_by_id(event_id))
    }
}

/// Single-process durable reference adapter. Each stream is atomically rewritten
/// to a content-verifiable JSON file; startup rejects corrupted or discontinuous
/// streams instead of silently projecting partial history.
#[derive(Debug, Clone)]
pub struct FileScientificEventLog {
    root: Arc<PathBuf>,
    state: Arc<RwLock<EventLogState>>,
}

impl FileScientificEventLog {
    pub async fn open(path: impl AsRef<Path>) -> Result<Self> {
        let root = path.as_ref().to_path_buf();
        fs::create_dir_all(&root)?;
        let mut state = EventLogState::default();

        for entry in fs::read_dir(&root)? {
            let entry = entry?;
            let path = entry.path();
            let file_type = entry.file_type()?;
            if file_type.is_symlink() {
                return Err(Error::Storage(format!(
                    "scientific event log refuses symbolic links: {}",
                    path.display()
                )));
            }
            if !file_type.is_file()
                || path.extension().and_then(|value| value.to_str()) != Some("json")
            {
                continue;
            }
            let metadata = entry.metadata()?;
            if metadata.len() > MAX_EVENT_STREAM_FILE_BYTES {
                return Err(Error::Storage(format!(
                    "scientific event stream exceeds {} bytes: {}",
                    MAX_EVENT_STREAM_FILE_BYTES,
                    path.display()
                )));
            }
            let bytes = fs::read(&path)?;
            let stream: Vec<SignedScientificEvent> = serde_json::from_slice(&bytes)?;
            let first = stream.first().ok_or_else(|| {
                Error::Storage(format!(
                    "empty scientific event stream file: {}",
                    path.display()
                ))
            })?;
            let claim_id = first.envelope.stream_id;
            let expected_name = format!("{}.events.json", claim_id.0);
            if entry.file_name().to_string_lossy() != expected_name {
                return Err(Error::Storage(format!(
                    "scientific event stream filename does not match stream id: {}",
                    path.display()
                )));
            }
            if state.streams.insert(claim_id, stream).is_some() {
                return Err(Error::Storage(format!(
                    "duplicate scientific event stream for claim {claim_id}"
                )));
            }
        }
        state.rebuild_indexes()?;

        Ok(Self {
            root: Arc::new(root),
            state: Arc::new(RwLock::new(state)),
        })
    }

    fn stream_path(&self, claim_id: ClaimId) -> PathBuf {
        self.root.join(format!("{}.events.json", claim_id.0))
    }

    fn persist_stream(
        &self,
        claim_id: ClaimId,
        expected: &[SignedScientificEvent],
        next: &[SignedScientificEvent],
    ) -> Result<()> {
        persist_json_vec_transaction(
            &self.stream_path(claim_id),
            expected,
            next,
            MAX_EVENT_STREAM_FILE_BYTES,
            "scientific event stream",
        )
    }
}

#[async_trait]
impl ScientificEventLog for FileScientificEventLog {
    async fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        let claim_id = event.envelope.stream_id;
        let mut state = self.state.write().await;
        let before = state.clone();
        let receipt = state.append(expected_sequence, event, received_at)?;
        let expected_stream = before.streams.get(&claim_id).cloned().unwrap_or_default();
        let stream = state.streams.get(&claim_id).cloned().unwrap_or_default();
        if let Err(error) = self.persist_stream(claim_id, &expected_stream, &stream) {
            *state = before;
            return Err(error);
        }
        Ok(receipt)
    }

    async fn head(&self, claim_id: ClaimId) -> Result<Option<StreamHead>> {
        let state = self.state.read().await;
        state
            .streams
            .get(&claim_id)
            .and_then(|stream| stream.last())
            .map(|event| {
                Ok(StreamHead {
                    sequence: event.envelope.sequence,
                    event_id: event.envelope.event_id,
                    event_hash: event.event_hash()?,
                })
            })
            .transpose()
    }

    async fn read(&self, claim_id: ClaimId, from_sequence: u64, limit: usize) -> Result<EventPage> {
        let state = self.state.read().await;
        let stream = state.streams.get(&claim_id).cloned().unwrap_or_default();
        page_events(&stream, from_sequence, limit)
    }

    async fn stream(&self, claim_id: ClaimId) -> Result<Vec<SignedScientificEvent>> {
        Ok(self
            .state
            .read()
            .await
            .streams
            .get(&claim_id)
            .cloned()
            .unwrap_or_default())
    }

    async fn stream_ids(&self) -> Result<Vec<ClaimId>> {
        let mut ids = self
            .state
            .read()
            .await
            .streams
            .keys()
            .copied()
            .collect::<Vec<_>>();
        ids.sort();
        Ok(ids)
    }

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>> {
        Ok(self.state.read().await.event_by_id(event_id))
    }

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>> {
        let state = self.state.read().await;
        let event_id = match state.event_hashes.get(&hash) {
            Some(event_id) => *event_id,
            None => return Ok(None),
        };
        Ok(state.event_by_id(event_id))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClaimLifecycle {
    Active,
    Superseded {
        replacement_claim_id: ClaimId,
        reason: String,
    },
    Retracted {
        reason: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceMaturity {
    Proposed,
    ArtifactBacked,
    Reviewed,
    ComputationallyReproduced,
    IndependentlyReplicated,
    Contested,
    Superseded,
    Retracted,
}

/// Independent dimensions used by assessment policies; no confidence scalar.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceProfile {
    pub artifact_count: usize,
    pub public_artifact_count: usize,
    pub controlled_artifact_count: usize,
    pub review_count: usize,
    pub supportive_reproduction_count: usize,
    pub supportive_independent_replication_count: usize,
    pub non_supporting_result_count: usize,
    pub inconclusive_result_count: usize,
    pub critique_count: usize,
    pub conflict_disclosure_count: usize,
    pub correction_count: usize,
    pub withdrawn_attestation_count: usize,
}

/// Versioned and explainable interpretation of the factual evidence profile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceAssessment {
    pub policy_id: String,
    pub policy_version: String,
    pub maturity: EvidenceMaturity,
    pub contested: bool,
    pub reasons: Vec<String>,
}

impl EvidenceAssessment {
    pub fn derive(profile: &EvidenceProfile, lifecycle: &ClaimLifecycle) -> Self {
        let mut reasons = Vec::new();
        let maturity = match lifecycle {
            ClaimLifecycle::Retracted { .. } => {
                reasons.push("claim has an active retraction".to_string());
                EvidenceMaturity::Retracted
            }
            ClaimLifecycle::Superseded { .. } => {
                reasons.push("claim has been superseded".to_string());
                EvidenceMaturity::Superseded
            }
            ClaimLifecycle::Active if profile.supportive_independent_replication_count > 0 => {
                reasons.push(format!(
                    "{} qualified independent replication source(s)",
                    profile.supportive_independent_replication_count
                ));
                EvidenceMaturity::IndependentlyReplicated
            }
            ClaimLifecycle::Active if profile.supportive_reproduction_count > 0 => {
                reasons.push(format!(
                    "{} qualified computational reproduction source(s)",
                    profile.supportive_reproduction_count
                ));
                EvidenceMaturity::ComputationallyReproduced
            }
            ClaimLifecycle::Active if profile.review_count > 0 => {
                reasons.push(format!(
                    "{} unique active reviewer(s)",
                    profile.review_count
                ));
                EvidenceMaturity::Reviewed
            }
            ClaimLifecycle::Active if profile.artifact_count > 0 => {
                reasons.push(format!("{} attached artifact(s)", profile.artifact_count));
                EvidenceMaturity::ArtifactBacked
            }
            ClaimLifecycle::Active
                if profile.non_supporting_result_count > 0 || profile.critique_count > 0 =>
            {
                reasons.push("active non-supporting evidence or critique".to_string());
                EvidenceMaturity::Contested
            }
            ClaimLifecycle::Active => {
                reasons.push("proposal has no qualifying external assessment".to_string());
                EvidenceMaturity::Proposed
            }
        };
        let contested = profile.non_supporting_result_count > 0 || profile.critique_count > 0;
        if contested && !matches!(maturity, EvidenceMaturity::Contested) {
            reasons.push("maturity coexists with active contestation".to_string());
        }
        Self {
            policy_id: DEFAULT_EVIDENCE_POLICY_ID.to_string(),
            policy_version: DEFAULT_EVIDENCE_POLICY_VERSION.to_string(),
            maturity,
            contested,
            reasons,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttestationStatus {
    Active,
    Withdrawn { reason: String },
}

/// Projection record retaining the identity context needed to qualify unique
/// and organizationally independent evidence sources.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedAttestation {
    pub attestation: Attestation,
    pub actor: ActorId,
    pub acting_organization: Option<OrganizationId>,
    pub status: AttestationStatus,
    pub revision_count: u32,
    pub last_reason: Option<String>,
}

impl RecordedAttestation {
    pub fn is_active(&self) -> bool {
        matches!(self.status, AttestationStatus::Active)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClaimOrigin {
    Native,
    LegacyImport { metadata: LegacyImportMetadata },
}

impl ClaimOrigin {
    pub fn is_legacy_unassessed(&self) -> bool {
        matches!(self, Self::LegacyImport { .. })
    }
}

/// Rebuildable query view derived only from a verified event stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimProjection {
    pub origin: ClaimOrigin,
    pub research_object: ResearchObject,
    pub claim: AtomicClaim,
    pub creator: ActorId,
    pub creator_organization: Option<OrganizationId>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub lifecycle: ClaimLifecycle,
    pub evidence_profile: EvidenceProfile,
    pub evidence: Vec<EvidenceArtifact>,
    pub attestations: Vec<RecordedAttestation>,
    pub event_count: u64,
    pub last_event_hash: ContentHash,
}

impl ClaimProjection {
    pub fn rebuild(events: &[SignedScientificEvent]) -> Result<Self> {
        let first = events
            .first()
            .ok_or_else(|| Error::Validation("cannot project an empty event stream".to_string()))?;
        first.verify()?;

        let (origin, research_object, claim) = match &first.envelope.payload {
            ScientificEventPayload::ClaimProposed {
                research_object,
                claim,
            } if first.envelope.sequence == 0 => {
                (ClaimOrigin::Native, research_object.clone(), claim.clone())
            }
            ScientificEventPayload::LegacyClaimImported {
                research_object,
                claim,
                import,
            } if first.envelope.sequence == 0 => (
                ClaimOrigin::LegacyImport {
                    metadata: import.clone(),
                },
                research_object.clone(),
                claim.clone(),
            ),
            _ => {
                return Err(Error::Validation(
                    "scientific event stream must begin with claim_proposed or legacy_claim_imported"
                        .to_string(),
                ));
            }
        };

        let mut projection = Self {
            origin,
            research_object,
            claim,
            creator: first.envelope.actor.clone(),
            creator_organization: first.envelope.acting_organization.clone(),
            created_at: first.envelope.occurred_at.clone(),
            updated_at: first.envelope.occurred_at.clone(),
            lifecycle: ClaimLifecycle::Active,
            evidence_profile: EvidenceProfile::default(),
            evidence: Vec::new(),
            attestations: Vec::new(),
            event_count: 1,
            last_event_hash: first.event_hash()?,
        };

        let mut evidence_ids = HashSet::new();
        let mut attestation_ids = HashSet::new();

        for (index, event) in events.iter().enumerate().skip(1) {
            event.verify()?;
            let expected_sequence = index as u64;
            if event.envelope.stream_id != projection.claim.id
                || event.envelope.sequence != expected_sequence
                || event.envelope.previous_hash != Some(projection.last_event_hash)
            {
                return Err(Error::Validation(
                    "invalid scientific event stream ordering or hash chain".to_string(),
                ));
            }

            if !matches!(projection.lifecycle, ClaimLifecycle::Active)
                && !event.envelope.payload.is_post_terminal_housekeeping()
            {
                return Err(Error::Validation(
                    "claim content cannot change after supersession or retraction".to_string(),
                ));
            }

            match &event.envelope.payload {
                ScientificEventPayload::ClaimProposed { .. }
                | ScientificEventPayload::LegacyClaimImported { .. } => {
                    return Err(Error::Validation(
                        "claim genesis events can appear only once".to_string(),
                    ));
                }
                ScientificEventPayload::EvidenceAttached { artifact, .. } => {
                    if !evidence_ids.insert(artifact.id) {
                        return Err(Error::Validation(
                            "duplicate evidence artifact id in stream".to_string(),
                        ));
                    }
                    projection.evidence.push(artifact.clone());
                }
                ScientificEventPayload::AttestationRecorded { attestation } => {
                    if !attestation_ids.insert(attestation.id) {
                        return Err(Error::Validation(
                            "duplicate attestation id in stream".to_string(),
                        ));
                    }
                    validate_attestation_evidence(attestation, &evidence_ids)?;
                    projection.ensure_unique_active_attestation(
                        &event.envelope.actor,
                        event.envelope.acting_organization.as_ref(),
                        attestation,
                        None,
                    )?;
                    projection.attestations.push(RecordedAttestation {
                        attestation: attestation.clone(),
                        actor: event.envelope.actor.clone(),
                        acting_organization: event.envelope.acting_organization.clone(),
                        status: AttestationStatus::Active,
                        revision_count: 0,
                        last_reason: None,
                    });
                }
                ScientificEventPayload::AttestationCorrected {
                    attestation_id,
                    replacement,
                    reason,
                    ..
                } => {
                    validate_attestation_evidence(replacement, &evidence_ids)?;
                    let index = projection
                        .attestations
                        .iter()
                        .position(|record| record.attestation.id == *attestation_id)
                        .ok_or_else(|| {
                            Error::Validation("cannot correct an unknown attestation".to_string())
                        })?;
                    let actor = projection.attestations[index].actor.clone();
                    let organization = projection.attestations[index].acting_organization.clone();
                    projection.ensure_unique_active_attestation(
                        &actor,
                        organization.as_ref(),
                        replacement,
                        Some(*attestation_id),
                    )?;
                    let record = &mut projection.attestations[index];
                    if !record.is_active() {
                        return Err(Error::Validation(
                            "cannot correct a withdrawn attestation".to_string(),
                        ));
                    }
                    record.attestation = replacement.clone();
                    record.revision_count += 1;
                    record.last_reason = Some(reason.clone());
                }
                ScientificEventPayload::AttestationWithdrawn {
                    attestation_id,
                    reason,
                    ..
                } => {
                    let record = projection
                        .attestations
                        .iter_mut()
                        .find(|record| record.attestation.id == *attestation_id)
                        .ok_or_else(|| {
                            Error::Validation("cannot withdraw an unknown attestation".to_string())
                        })?;
                    if !record.is_active() {
                        return Err(Error::Validation(
                            "attestation is already withdrawn".to_string(),
                        ));
                    }
                    record.status = AttestationStatus::Withdrawn {
                        reason: reason.clone(),
                    };
                    record.last_reason = Some(reason.clone());
                }
                ScientificEventPayload::ClaimSuperseded {
                    replacement_claim_id,
                    reason,
                    ..
                } => {
                    projection.lifecycle = ClaimLifecycle::Superseded {
                        replacement_claim_id: *replacement_claim_id,
                        reason: reason.clone(),
                    };
                }
                ScientificEventPayload::ClaimRetracted { reason, .. } => {
                    projection.lifecycle = ClaimLifecycle::Retracted {
                        reason: reason.clone(),
                    };
                }
            }

            if event.envelope.occurred_at > projection.updated_at {
                projection.updated_at = event.envelope.occurred_at.clone();
            }
            projection.event_count += 1;
            projection.last_event_hash = event.event_hash()?;
            projection.recompute_evidence_profile();
        }

        projection.recompute_evidence_profile();
        Ok(projection)
    }

    pub fn maturity(&self) -> EvidenceMaturity {
        self.assessment().maturity
    }

    pub fn assessment(&self) -> EvidenceAssessment {
        let mut assessment = EvidenceAssessment::derive(&self.evidence_profile, &self.lifecycle);
        if self.origin.is_legacy_unassessed() {
            assessment.reasons.insert(
                0,
                "legacy record imported as unassessed history; historical tiers and counts do not qualify as evidence"
                    .to_string(),
            );
        }
        assessment
    }

    pub fn attestation(&self, id: AttestationId) -> Option<&RecordedAttestation> {
        self.attestations
            .iter()
            .find(|record| record.attestation.id == id)
    }

    fn ensure_unique_active_attestation(
        &self,
        actor: &ActorId,
        _organization: Option<&OrganizationId>,
        candidate: &Attestation,
        replacing: Option<AttestationId>,
    ) -> Result<()> {
        let duplicate = self.attestations.iter().any(|record| {
            record.is_active()
                && Some(record.attestation.id) != replacing
                && &record.actor == actor
                && record.attestation.uniqueness_family() == candidate.uniqueness_family()
                && record.attestation.protocol_reference == candidate.protocol_reference
        });
        if duplicate {
            return Err(Error::Validation(
                "actor already has an active attestation for this kind and protocol".to_string(),
            ));
        }
        Ok(())
    }

    fn recompute_evidence_profile(&mut self) {
        let mut profile = EvidenceProfile::default();
        profile.artifact_count = self.evidence.len();
        for artifact in &self.evidence {
            match artifact.availability {
                ArtifactAvailability::Public => profile.public_artifact_count += 1,
                ArtifactAvailability::Controlled => profile.controlled_artifact_count += 1,
                ArtifactAvailability::Embargoed | ArtifactAvailability::Unavailable => {}
            }
        }

        let mut reviewers = BTreeSet::new();
        let mut reproductions = BTreeSet::new();
        let mut independent_replications = BTreeSet::new();
        let mut non_supporting = BTreeSet::new();
        let mut inconclusive = BTreeSet::new();
        let mut critiques = BTreeSet::new();
        let mut conflicts = BTreeSet::new();

        for record in &self.attestations {
            // Revision/correction history is a permanent audit-trail fact
            // about this attestation, independent of whether it is
            // currently active -- must be counted even for records that
            // are later withdrawn, so it accumulates before the
            // active-only `continue` below.
            profile.correction_count += record.revision_count as usize;
            if !record.is_active() {
                profile.withdrawn_attestation_count += 1;
                continue;
            }
            let source = source_key(&record.actor, record.acting_organization.as_ref());
            match &record.attestation.kind {
                AttestationKind::Review => {
                    let actor_is_creator = record.actor == self.creator;
                    let organization_is_creator = record.acting_organization.is_some()
                        && record.acting_organization.as_ref()
                            == self.creator_organization.as_ref();
                    if !actor_is_creator && !organization_is_creator {
                        reviewers.insert(record.actor.clone());
                    }
                }
                AttestationKind::ComputationalReproduction { outcome } => match outcome {
                    EvidenceOutcome::Supports => {
                        reproductions.insert(source);
                    }
                    EvidenceOutcome::DoesNotSupport => {
                        non_supporting.insert(source);
                    }
                    EvidenceOutcome::Inconclusive => {
                        inconclusive.insert(source);
                    }
                },
                AttestationKind::IndependentReplication { outcome } => {
                    let actor_is_creator = record.actor == self.creator;
                    let organization_is_creator = record.acting_organization.is_some()
                        && record.acting_organization.as_ref()
                            == self.creator_organization.as_ref();
                    match outcome {
                        EvidenceOutcome::Supports
                            if !actor_is_creator && !organization_is_creator =>
                        {
                            independent_replications.insert(source);
                        }
                        EvidenceOutcome::Supports => {}
                        EvidenceOutcome::DoesNotSupport => {
                            non_supporting.insert(source);
                        }
                        EvidenceOutcome::Inconclusive => {
                            inconclusive.insert(source);
                        }
                    }
                }
                AttestationKind::Critique => {
                    critiques.insert(record.actor.clone());
                }
                AttestationKind::ConflictDisclosure => {
                    conflicts.insert(record.actor.clone());
                }
                AttestationKind::Correction => profile.correction_count += 1,
            }
        }

        profile.review_count = reviewers.len();
        profile.supportive_reproduction_count = reproductions.len();
        profile.supportive_independent_replication_count = independent_replications.len();
        profile.non_supporting_result_count = non_supporting.len();
        profile.inconclusive_result_count = inconclusive.len();
        profile.critique_count = critiques.len();
        profile.conflict_disclosure_count = conflicts.len();
        self.evidence_profile = profile;
    }
}

fn source_key(actor: &ActorId, organization: Option<&OrganizationId>) -> String {
    match organization {
        Some(organization) => format!("org:{}", organization.as_str()),
        None => format!("actor:{}", actor.as_str()),
    }
}

fn validate_attestation_evidence(
    attestation: &Attestation,
    evidence_ids: &HashSet<ArtifactId>,
) -> Result<()> {
    if attestation
        .evidence_ids
        .iter()
        .any(|id| !evidence_ids.contains(id))
    {
        return Err(Error::Validation(
            "attestation references evidence not yet attached to the claim".to_string(),
        ));
    }
    Ok(())
}

fn validate_event_time(event: &SignedScientificEvent, received_at: &DateTime<Utc>) -> Result<()> {
    let latest_allowed = received_at.clone() + Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS);
    if event.envelope.occurred_at > latest_allowed {
        return Err(Error::Validation(
            "scientific event occurred_at exceeds allowed future clock skew".to_string(),
        ));
    }
    Ok(())
}

fn validate_append_to_stream(
    stream: &[SignedScientificEvent],
    event: &SignedScientificEvent,
) -> Result<()> {
    match stream.last() {
        None => {
            if event.envelope.previous_hash.is_some() {
                return Err(Error::Storage(
                    "genesis event cannot reference a previous event".to_string(),
                ));
            }
            if !matches!(
                &event.envelope.payload,
                ScientificEventPayload::ClaimProposed { .. }
                    | ScientificEventPayload::LegacyClaimImported { .. }
            ) {
                return Err(Error::Storage(
                    "scientific event stream must begin with claim_proposed or legacy_claim_imported"
                        .to_string(),
                ));
            }
        }
        Some(previous) => {
            let expected_previous_hash = previous.event_hash()?;
            if event.envelope.previous_hash != Some(expected_previous_hash) {
                return Err(Error::Storage(
                    "scientific event hash chain is discontinuous".to_string(),
                ));
            }
        }
    }
    Ok(())
}

fn page_events(
    stream: &[SignedScientificEvent],
    from_sequence: u64,
    limit: usize,
) -> Result<EventPage> {
    if limit == 0 || limit > 10_000 {
        return Err(Error::Validation(
            "event page limit must be between 1 and 10000".to_string(),
        ));
    }
    let start = usize::try_from(from_sequence)
        .map_err(|_| Error::Validation("event sequence exceeds platform limits".to_string()))?;
    if start >= stream.len() {
        return Ok(EventPage {
            events: Vec::new(),
            next_sequence: None,
        });
    }
    let end = start.saturating_add(limit).min(stream.len());
    Ok(EventPage {
        events: stream[start..end].to_vec(),
        next_sequence: (end < stream.len()).then_some(end as u64),
    })
}

fn push_len(bytes: &mut Vec<u8>, len: usize) -> Result<()> {
    let len = u32::try_from(len)
        .map_err(|_| Error::SerializationError("canonical field exceeds u32 length".to_string()))?;
    bytes.extend_from_slice(&len.to_be_bytes());
    Ok(())
}

struct CanonicalEncoder {
    bytes: Vec<u8>,
}

impl CanonicalEncoder {
    fn new(domain: &[u8]) -> Self {
        Self {
            bytes: domain.to_vec(),
        }
    }

    fn finish(self) -> Vec<u8> {
        self.bytes
    }

    fn u8(&mut self, value: u8) {
        self.bytes.push(value);
    }

    fn u16(&mut self, value: u16) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn i64(&mut self, value: i64) {
        self.bytes.extend_from_slice(&value.to_be_bytes());
    }

    fn uuid(&mut self, value: Uuid) {
        self.bytes.extend_from_slice(value.as_bytes());
    }

    fn hash(&mut self, value: ContentHash) {
        self.bytes.extend_from_slice(&value.0);
    }

    fn string(&mut self, value: &str) -> Result<()> {
        push_len(&mut self.bytes, value.len())?;
        self.bytes.extend_from_slice(value.as_bytes());
        Ok(())
    }

    fn option_string(&mut self, value: Option<&str>) -> Result<()> {
        match value {
            Some(value) => {
                self.u8(1);
                self.string(value)?;
            }
            None => self.u8(0),
        }
        Ok(())
    }

    fn option_hash(&mut self, value: Option<ContentHash>) {
        match value {
            Some(value) => {
                self.u8(1);
                self.hash(value);
            }
            None => self.u8(0),
        }
    }

    fn datetime(&mut self, value: &DateTime<Utc>) {
        self.i64(value.timestamp());
        self.u32(value.timestamp_subsec_nanos());
    }

    fn research_object(&mut self, object: &ResearchObject) -> Result<()> {
        self.uuid(object.id.0);
        self.string(&object.title)?;
        match &object.object_type {
            ResearchObjectType::Manuscript => self.u8(1),
            ResearchObjectType::Dataset => self.u8(2),
            ResearchObjectType::Software => self.u8(3),
            ResearchObjectType::Workflow => self.u8(4),
            ResearchObjectType::Protocol => self.u8(5),
            ResearchObjectType::Model => self.u8(6),
            ResearchObjectType::Other(value) => {
                self.u8(255);
                self.string(value)?;
            }
        }
        self.option_string(object.persistent_identifier.as_deref())
    }

    fn claim(&mut self, claim: &AtomicClaim) -> Result<()> {
        self.uuid(claim.id.0);
        self.uuid(claim.research_object_id.0);
        self.string(&claim.statement)?;
        self.option_string(claim.scope.as_deref())
    }

    fn artifact(&mut self, artifact: &EvidenceArtifact) -> Result<()> {
        self.uuid(artifact.id.0);
        self.hash(artifact.content_hash);
        self.string(&artifact.media_type)?;
        self.string(&artifact.locator)?;
        self.option_string(artifact.license.as_deref())?;
        self.u8(match artifact.availability {
            ArtifactAvailability::Public => 1,
            ArtifactAvailability::Controlled => 2,
            ArtifactAvailability::Embargoed => 3,
            ArtifactAvailability::Unavailable => 4,
        });
        Ok(())
    }

    fn outcome(&mut self, outcome: EvidenceOutcome) {
        self.u8(match outcome {
            EvidenceOutcome::Supports => 1,
            EvidenceOutcome::DoesNotSupport => 2,
            EvidenceOutcome::Inconclusive => 3,
        });
    }

    fn attestation(&mut self, attestation: &Attestation) -> Result<()> {
        self.uuid(attestation.id.0);
        self.uuid(attestation.claim_id.0);
        match &attestation.kind {
            AttestationKind::Review => self.u8(1),
            AttestationKind::ComputationalReproduction { outcome } => {
                self.u8(2);
                self.outcome(*outcome);
            }
            AttestationKind::IndependentReplication { outcome } => {
                self.u8(3);
                self.outcome(*outcome);
            }
            AttestationKind::Critique => self.u8(4),
            AttestationKind::ConflictDisclosure => self.u8(5),
            AttestationKind::Correction => self.u8(6),
        }
        push_len(&mut self.bytes, attestation.evidence_ids.len())?;
        for id in &attestation.evidence_ids {
            self.uuid(id.0);
        }
        self.option_string(attestation.statement.as_deref())?;
        self.option_string(attestation.protocol_reference.as_deref())
    }

    fn payload(&mut self, payload: &ScientificEventPayload) -> Result<()> {
        match payload {
            ScientificEventPayload::ClaimProposed {
                research_object,
                claim,
            } => {
                self.u16(1);
                self.research_object(research_object)?;
                self.claim(claim)?;
            }
            ScientificEventPayload::LegacyClaimImported {
                research_object,
                claim,
                import,
            } => {
                self.u16(8);
                self.research_object(research_object)?;
                self.claim(claim)?;
                self.string(&import.source_system)?;
                self.hash(import.source_record_hash);
                self.option_string(import.source_locator.as_deref())?;
                self.option_string(import.legacy_creator.as_deref())?;
                self.string(&import.legacy_tier)?;
                self.u64(import.legacy_verification_count as u64);
                self.u64(import.legacy_provenance_count as u64);
                push_len(&mut self.bytes, import.omitted_fields.len())?;
                for field in &import.omitted_fields {
                    self.string(field)?;
                }
            }
            ScientificEventPayload::EvidenceAttached { claim_id, artifact } => {
                self.u16(2);
                self.uuid(claim_id.0);
                self.artifact(artifact)?;
            }
            ScientificEventPayload::AttestationRecorded { attestation } => {
                self.u16(3);
                self.attestation(attestation)?;
            }
            ScientificEventPayload::AttestationCorrected {
                claim_id,
                attestation_id,
                replacement,
                reason,
            } => {
                self.u16(4);
                self.uuid(claim_id.0);
                self.uuid(attestation_id.0);
                self.attestation(replacement)?;
                self.string(reason)?;
            }
            ScientificEventPayload::AttestationWithdrawn {
                claim_id,
                attestation_id,
                reason,
            } => {
                self.u16(5);
                self.uuid(claim_id.0);
                self.uuid(attestation_id.0);
                self.string(reason)?;
            }
            ScientificEventPayload::ClaimSuperseded {
                claim_id,
                replacement_claim_id,
                reason,
            } => {
                self.u16(6);
                self.uuid(claim_id.0);
                self.uuid(replacement_claim_id.0);
                self.string(reason)?;
            }
            ScientificEventPayload::ClaimRetracted { claim_id, reason } => {
                self.u16(7);
                self.uuid(claim_id.0);
                self.string(reason)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;
    use tempfile::tempdir;

    fn actor(value: &str) -> ActorId {
        ActorId::new(value).unwrap()
    }

    fn organization(value: &str) -> OrganizationId {
        OrganizationId::new(value).unwrap()
    }

    fn key(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }

    fn proposed_payload(claim_id: ClaimId, object_id: ResearchObjectId) -> ScientificEventPayload {
        ScientificEventPayload::ClaimProposed {
            research_object: ResearchObject {
                id: object_id,
                title: "A reproducible result".to_string(),
                object_type: ResearchObjectType::Manuscript,
                persistent_identifier: Some("doi:10.0000/example".to_string()),
            },
            claim: AtomicClaim {
                id: claim_id,
                research_object_id: object_id,
                statement: "Treatment X changes outcome Y under condition Z".to_string(),
                scope: Some("Registered population and endpoint".to_string()),
            },
        }
    }

    fn artifact(id: ArtifactId) -> EvidenceArtifact {
        EvidenceArtifact {
            id,
            content_hash: ContentHash::digest(b"dataset"),
            media_type: "application/parquet".to_string(),
            locator: "ipfs://bafy-test".to_string(),
            license: Some("CC-BY-4.0".to_string()),
            availability: ArtifactAvailability::Public,
        }
    }

    #[test]
    fn canonical_bytes_bind_actor_payload_and_protocol() {
        let envelope = ScientificEventEnvelope::genesis(
            actor("did:key:alice"),
            Utc.timestamp_opt(1_700_000_000, 123).unwrap(),
            proposed_payload(ClaimId::new(), ResearchObjectId::new()),
        )
        .unwrap()
        .with_acting_organization(organization("ror:alice-lab"))
        .with_idempotency_key("claim-create-1")
        .unwrap();
        let mut signed = SignedScientificEvent::sign(envelope, &key(7)).unwrap();
        signed.verify().unwrap();
        let bytes = signed.envelope.signing_bytes().unwrap();
        assert!(bytes.starts_with(b"MYCELIX-DESCI-EVENT\0"));

        signed.envelope.actor = actor("did:key:mallory");
        assert!(signed.verify().is_err());
    }

    #[tokio::test]
    async fn event_log_enforces_sequence_hash_chain_and_idempotency() {
        let log = MemoryScientificEventLog::new();
        let claim_id = ClaimId::new();
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let genesis = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                now.clone(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap()
            .with_idempotency_key("create-claim")
            .unwrap(),
            &key(1),
        )
        .unwrap();
        log.append_at(0, genesis.clone(), now.clone())
            .await
            .unwrap();

        let artifact_event = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &genesis,
                actor("did:key:alice"),
                now.clone() + Duration::seconds(10),
                ScientificEventPayload::EvidenceAttached {
                    claim_id,
                    artifact: artifact(ArtifactId::new()),
                },
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();

        assert!(
            log.append_at(
                0,
                artifact_event.clone(),
                now.clone() + Duration::seconds(10)
            )
            .await
            .is_err()
        );
        let receipt = log
            .append_at(
                1,
                artifact_event.clone(),
                now.clone() + Duration::seconds(10),
            )
            .await
            .unwrap();
        assert_eq!(receipt.sequence, 1);
        assert_eq!(log.head(claim_id).await.unwrap().unwrap().sequence, 1);
        assert!(
            log.event_by_hash(receipt.event_hash)
                .await
                .unwrap()
                .is_some()
        );
    }

    #[test]
    fn independent_replication_counts_unique_non_creator_sources() {
        let claim_id = ClaimId::new();
        let creator_org = organization("ror:creator-lab");
        let genesis = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                Utc.timestamp_opt(1_700_000_000, 0).unwrap(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap()
            .with_acting_organization(creator_org.clone()),
            &key(1),
        )
        .unwrap();

        let artifact_id = ArtifactId::new();
        let evidence = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &genesis,
                actor("did:key:alice"),
                Utc.timestamp_opt(1_700_000_010, 0).unwrap(),
                ScientificEventPayload::EvidenceAttached {
                    claim_id,
                    artifact: artifact(artifact_id),
                },
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();

        let self_replication = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &evidence,
                actor("did:key:alice"),
                Utc.timestamp_opt(1_700_000_020, 0).unwrap(),
                ScientificEventPayload::AttestationRecorded {
                    attestation: Attestation {
                        id: AttestationId::new(),
                        claim_id,
                        kind: AttestationKind::IndependentReplication {
                            outcome: EvidenceOutcome::Supports,
                        },
                        evidence_ids: vec![artifact_id],
                        statement: Some("Author rerun".to_string()),
                        protocol_reference: Some("protocol:v1".to_string()),
                    },
                },
            )
            .unwrap()
            .with_acting_organization(creator_org),
            &key(1),
        )
        .unwrap();

        let external = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &self_replication,
                actor("did:key:bob"),
                Utc.timestamp_opt(1_700_000_030, 0).unwrap(),
                ScientificEventPayload::AttestationRecorded {
                    attestation: Attestation {
                        id: AttestationId::new(),
                        claim_id,
                        kind: AttestationKind::IndependentReplication {
                            outcome: EvidenceOutcome::Supports,
                        },
                        evidence_ids: vec![artifact_id],
                        statement: Some("Independent preregistered replication".to_string()),
                        protocol_reference: Some("protocol:v1".to_string()),
                    },
                },
            )
            .unwrap()
            .with_acting_organization(organization("ror:external-lab")),
            &key(2),
        )
        .unwrap();

        let projection =
            ClaimProjection::rebuild(&[genesis, evidence, self_replication, external]).unwrap();
        assert_eq!(
            projection
                .evidence_profile
                .supportive_independent_replication_count,
            1
        );
        assert_eq!(
            projection.maturity(),
            EvidenceMaturity::IndependentlyReplicated
        );
    }

    #[test]
    fn attestation_correction_and_withdrawal_recompute_projection() {
        let claim_id = ClaimId::new();
        let genesis = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                Utc.timestamp_opt(1_700_000_000, 0).unwrap(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();
        let artifact_id = ArtifactId::new();
        let evidence = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &genesis,
                actor("did:key:alice"),
                Utc.timestamp_opt(1_700_000_010, 0).unwrap(),
                ScientificEventPayload::EvidenceAttached {
                    claim_id,
                    artifact: artifact(artifact_id),
                },
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();
        let attestation_id = AttestationId::new();
        let recorded = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &evidence,
                actor("did:key:bob"),
                Utc.timestamp_opt(1_700_000_020, 0).unwrap(),
                ScientificEventPayload::AttestationRecorded {
                    attestation: Attestation {
                        id: attestation_id,
                        claim_id,
                        kind: AttestationKind::ComputationalReproduction {
                            outcome: EvidenceOutcome::Supports,
                        },
                        evidence_ids: vec![artifact_id],
                        statement: None,
                        protocol_reference: Some("workflow:v1".to_string()),
                    },
                },
            )
            .unwrap(),
            &key(2),
        )
        .unwrap();
        let corrected = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &recorded,
                actor("did:key:bob"),
                Utc.timestamp_opt(1_700_000_030, 0).unwrap(),
                ScientificEventPayload::AttestationCorrected {
                    claim_id,
                    attestation_id,
                    replacement: Attestation {
                        id: attestation_id,
                        claim_id,
                        kind: AttestationKind::ComputationalReproduction {
                            outcome: EvidenceOutcome::Inconclusive,
                        },
                        evidence_ids: vec![artifact_id],
                        statement: Some("Environment mismatch".to_string()),
                        protocol_reference: Some("workflow:v1".to_string()),
                    },
                    reason: "Corrected result classification".to_string(),
                },
            )
            .unwrap(),
            &key(2),
        )
        .unwrap();
        let withdrawn = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &corrected,
                actor("did:key:bob"),
                Utc.timestamp_opt(1_700_000_040, 0).unwrap(),
                ScientificEventPayload::AttestationWithdrawn {
                    claim_id,
                    attestation_id,
                    reason: "Protocol package was incomplete".to_string(),
                },
            )
            .unwrap(),
            &key(2),
        )
        .unwrap();

        let projection =
            ClaimProjection::rebuild(&[genesis, evidence, recorded, corrected, withdrawn]).unwrap();
        assert_eq!(projection.evidence_profile.supportive_reproduction_count, 0);
        assert_eq!(projection.evidence_profile.inconclusive_result_count, 0);
        assert_eq!(projection.evidence_profile.withdrawn_attestation_count, 1);
        assert_eq!(projection.evidence_profile.correction_count, 1);
    }

    #[tokio::test]
    async fn file_log_replays_byte_equivalent_projection() {
        let directory = tempdir().unwrap();
        let claim_id = ClaimId::new();
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let genesis = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                now.clone(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();

        let original = FileScientificEventLog::open(directory.path())
            .await
            .unwrap();
        original.append_at(0, genesis, now.clone()).await.unwrap();
        let before = ClaimProjection::rebuild(&original.stream(claim_id).await.unwrap()).unwrap();
        drop(original);

        let reopened = FileScientificEventLog::open(directory.path())
            .await
            .unwrap();
        let after = ClaimProjection::rebuild(&reopened.stream(claim_id).await.unwrap()).unwrap();
        assert_eq!(before, after);
    }

    #[tokio::test]
    async fn file_log_rejects_a_stale_second_process_writer() {
        let directory = tempdir().unwrap();
        let claim_id = ClaimId::new();
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let first = FileScientificEventLog::open(directory.path())
            .await
            .unwrap();
        let stale = FileScientificEventLog::open(directory.path())
            .await
            .unwrap();
        let accepted = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                now.clone(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();
        first.append_at(0, accepted, now.clone()).await.unwrap();

        let conflicting = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:bob"),
                now.clone(),
                proposed_payload(claim_id, ResearchObjectId::new()),
            )
            .unwrap(),
            &key(2),
        )
        .unwrap();
        let error = stale.append_at(0, conflicting, now).await.unwrap_err();
        assert!(error.to_string().contains("changed on disk"));
    }

    #[tokio::test]
    async fn file_log_rejects_stream_filename_mismatch() {
        let directory = tempdir().unwrap();
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                now,
                proposed_payload(ClaimId::new(), ResearchObjectId::new()),
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();
        std::fs::write(
            directory.path().join("wrong.events.json"),
            serde_json::to_vec(&vec![event]).unwrap(),
        )
        .unwrap();
        assert!(
            FileScientificEventLog::open(directory.path())
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn future_dated_events_fail_closed() {
        let log = MemoryScientificEventLog::new();
        let received = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                actor("did:key:alice"),
                received + Duration::minutes(10),
                proposed_payload(ClaimId::new(), ResearchObjectId::new()),
            )
            .unwrap(),
            &key(1),
        )
        .unwrap();
        assert!(log.append_at(0, event, received).await.is_err());
    }

    #[test]
    fn schema_v2_native_events_remain_verifiable() {
        let mut envelope = ScientificEventEnvelope::genesis(
            actor("did:key:alice"),
            Utc.timestamp_opt(1_700_000_000, 0).unwrap(),
            proposed_payload(ClaimId::new(), ResearchObjectId::new()),
        )
        .unwrap();
        envelope.schema_version = 2;
        let event = SignedScientificEvent::sign(envelope, &key(1)).unwrap();
        event.verify().unwrap();
    }

    #[test]
    fn deserialized_actor_identifiers_must_be_canonical() {
        let mut envelope = ScientificEventEnvelope::genesis(
            actor("did:key:alice"),
            Utc.timestamp_opt(1_700_000_000, 0).unwrap(),
            proposed_payload(ClaimId::new(), ResearchObjectId::new()),
        )
        .unwrap();
        envelope.actor = serde_json::from_str(r#"" did:key:alice ""#).unwrap();
        assert!(SignedScientificEvent::sign(envelope, &key(1)).is_err());
    }

    #[test]
    fn legacy_import_requires_schema_v3() {
        let claim_id = ClaimId::new();
        let object_id = ResearchObjectId::new();
        let mut envelope = ScientificEventEnvelope::genesis(
            actor("did:key:migration-service"),
            Utc.timestamp_opt(1_700_000_000, 0).unwrap(),
            ScientificEventPayload::LegacyClaimImported {
                research_object: ResearchObject {
                    id: object_id,
                    title: "Legacy record".to_string(),
                    object_type: ResearchObjectType::Other("legacy_claim_record".to_string()),
                    persistent_identifier: None,
                },
                claim: AtomicClaim {
                    id: claim_id,
                    research_object_id: object_id,
                    statement: "Historical statement".to_string(),
                    scope: None,
                },
                import: LegacyImportMetadata {
                    source_system: "legacy-json".to_string(),
                    source_record_hash: ContentHash::digest(b"legacy"),
                    source_locator: Some("legacy.json".to_string()),
                    legacy_creator: Some("historical-author@example.org".to_string()),
                    legacy_tier: "E4".to_string(),
                    legacy_verification_count: 12,
                    legacy_provenance_count: 3,
                    omitted_fields: vec!["legacy_verification_material".to_string()],
                },
            },
        )
        .unwrap();
        envelope.schema_version = 2;
        assert!(SignedScientificEvent::sign(envelope, &key(8)).is_err());
    }
}
