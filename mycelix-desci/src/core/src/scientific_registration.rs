// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Claim-independent append-only authority for preregistrations and protocols.
//!
//! Registrations are neither claims nor evidence. Drafts remain mutable
//! workspace state. Canonical authority begins only when a registration is
//! recorded. Pre-execution changes are immutable amendments; post-start changes
//! are explicit deviations and can never rewrite what was preregistered.

use crate::scientific_events::{
    ActorId, ContentHash, OrganizationId, MAX_EVENT_FUTURE_SKEW_SECONDS,
};
use crate::{Error, Result};
use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::sync::RwLock;
use uuid::Uuid;

pub const SCIENTIFIC_REGISTRATION_PROTOCOL: &str = "mycelix-desci-registration";
pub const SCIENTIFIC_REGISTRATION_PROTOCOL_VERSION: u16 = 1;
pub const SCIENTIFIC_REGISTRATION_SCHEMA_VERSION: u16 = 1;
pub const SCIENTIFIC_REGISTRATION_CODEC: &str = "mycelix-canonical-binary-v1";

const MAX_TITLE_BYTES: usize = 1024;
const MAX_TEXT_BYTES: usize = 4096;
const MAX_IDEMPOTENCY_KEY_BYTES: usize = 512;

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

uuid_id!(ResearchRegistrationId);
uuid_id!(ScientificRegistrationEventId);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegistrationKind {
    Preregistration,
    RegisteredReport,
    Protocol,
    AnalysisPlan,
    Other(String),
}

impl RegistrationKind {
    fn validate(&self) -> Result<()> {
        if let Self::Other(value) = self {
            validate_text(value, "registration kind", 256)?;
        }
        Ok(())
    }
}

/// Content-addressed protocol/analysis-plan document for one immutable version.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegistrationProtocolDocument {
    pub content_hash: ContentHash,
    pub media_type: String,
    pub locator: String,
}

impl RegistrationProtocolDocument {
    pub fn validate(&self) -> Result<()> {
        validate_text(&self.media_type, "registration protocol media type", 256)?;
        validate_text(&self.locator, "registration protocol locator", MAX_TEXT_BYTES)
    }
}

/// Exact local reference to one immutable registration version.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegistrationReference {
    pub registration_id: ResearchRegistrationId,
    pub version: u32,
    pub version_event_hash: ContentHash,
}

impl RegistrationReference {
    pub fn validate(&self) -> Result<()> {
        if self.version == 0 {
            return Err(Error::Validation(
                "registration reference version must be at least one".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ScientificRegistrationPayload {
    RegistrationCreated {
        registration_id: ResearchRegistrationId,
        title: String,
        kind: RegistrationKind,
        protocol: RegistrationProtocolDocument,
    },
    RegistrationAmended {
        registration_id: ResearchRegistrationId,
        predecessor_version: u32,
        predecessor_version_hash: ContentHash,
        protocol: RegistrationProtocolDocument,
        reason: String,
    },
    RegistrationSealed {
        registration_id: ResearchRegistrationId,
        version: u32,
    },
    ExecutionStarted {
        registration_id: ResearchRegistrationId,
        sealed_version: u32,
    },
    ProtocolDeviationRecorded {
        registration_id: ResearchRegistrationId,
        sealed_version: u32,
        reason: String,
        addendum: Option<RegistrationProtocolDocument>,
    },
    /// Lifecycle fact only; this event does not assert that an outcome supports
    /// a hypothesis or that evidence is scientifically valid.
    ExecutionCompleted {
        registration_id: ResearchRegistrationId,
        sealed_version: u32,
    },
    RegistrationWithdrawn {
        registration_id: ResearchRegistrationId,
        reason: String,
    },
}

impl ScientificRegistrationPayload {
    pub fn registration_id(&self) -> ResearchRegistrationId {
        match self {
            Self::RegistrationCreated { registration_id, .. }
            | Self::RegistrationAmended { registration_id, .. }
            | Self::RegistrationSealed { registration_id, .. }
            | Self::ExecutionStarted { registration_id, .. }
            | Self::ProtocolDeviationRecorded { registration_id, .. }
            | Self::ExecutionCompleted { registration_id, .. }
            | Self::RegistrationWithdrawn { registration_id, .. } => *registration_id,
        }
    }

    pub fn validate(&self) -> Result<()> {
        match self {
            Self::RegistrationCreated {
                title,
                kind,
                protocol,
                ..
            } => {
                validate_text(title, "registration title", MAX_TITLE_BYTES)?;
                kind.validate()?;
                protocol.validate()?;
            }
            Self::RegistrationAmended {
                predecessor_version,
                protocol,
                reason,
                ..
            } => {
                if *predecessor_version == 0 {
                    return Err(Error::Validation(
                        "registration amendment predecessor version must be at least one"
                            .to_string(),
                    ));
                }
                protocol.validate()?;
                validate_text(reason, "registration amendment reason", MAX_TEXT_BYTES)?;
            }
            Self::RegistrationSealed { version, .. } => require_version(*version)?,
            Self::ExecutionStarted { sealed_version, .. }
            | Self::ExecutionCompleted { sealed_version, .. } => {
                require_version(*sealed_version)?;
            }
            Self::ProtocolDeviationRecorded {
                sealed_version,
                reason,
                addendum,
                ..
            } => {
                require_version(*sealed_version)?;
                validate_text(reason, "protocol deviation reason", MAX_TEXT_BYTES)?;
                if let Some(addendum) = addendum {
                    addendum.validate()?;
                }
            }
            Self::RegistrationWithdrawn { reason, .. } => {
                validate_text(reason, "registration withdrawal reason", MAX_TEXT_BYTES)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScientificRegistrationEnvelope {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub event_id: ScientificRegistrationEventId,
    pub stream_id: ResearchRegistrationId,
    pub sequence: u64,
    pub previous_hash: Option<ContentHash>,
    pub actor: ActorId,
    pub acting_organization: Option<OrganizationId>,
    pub occurred_at: DateTime<Utc>,
    pub idempotency_key: Option<String>,
    pub payload: ScientificRegistrationPayload,
}

impl ScientificRegistrationEnvelope {
    pub fn genesis(
        actor: ActorId,
        occurred_at: DateTime<Utc>,
        payload: ScientificRegistrationPayload,
    ) -> Result<Self> {
        payload.validate()?;
        if !matches!(
            &payload,
            ScientificRegistrationPayload::RegistrationCreated { .. }
        ) {
            return Err(Error::Validation(
                "registration stream genesis must be registration_created".to_string(),
            ));
        }
        Ok(Self {
            protocol: SCIENTIFIC_REGISTRATION_PROTOCOL.to_string(),
            protocol_version: SCIENTIFIC_REGISTRATION_PROTOCOL_VERSION,
            codec: SCIENTIFIC_REGISTRATION_CODEC.to_string(),
            schema_version: SCIENTIFIC_REGISTRATION_SCHEMA_VERSION,
            event_id: ScientificRegistrationEventId::new(),
            stream_id: payload.registration_id(),
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
        previous: &SignedScientificRegistrationEvent,
        actor: ActorId,
        occurred_at: DateTime<Utc>,
        payload: ScientificRegistrationPayload,
    ) -> Result<Self> {
        payload.validate()?;
        if payload.registration_id() != previous.envelope.stream_id {
            return Err(Error::Validation(
                "registration payload belongs to a different stream".to_string(),
            ));
        }
        Ok(Self {
            protocol: SCIENTIFIC_REGISTRATION_PROTOCOL.to_string(),
            protocol_version: SCIENTIFIC_REGISTRATION_PROTOCOL_VERSION,
            codec: SCIENTIFIC_REGISTRATION_CODEC.to_string(),
            schema_version: SCIENTIFIC_REGISTRATION_SCHEMA_VERSION,
            event_id: ScientificRegistrationEventId::new(),
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
        let key = key.into();
        validate_text(&key, "registration idempotency key", MAX_IDEMPOTENCY_KEY_BYTES)?;
        self.idempotency_key = Some(key);
        Ok(self)
    }

    pub fn validate(&self) -> Result<()> {
        if self.protocol != SCIENTIFIC_REGISTRATION_PROTOCOL
            || self.protocol_version != SCIENTIFIC_REGISTRATION_PROTOCOL_VERSION
            || self.codec != SCIENTIFIC_REGISTRATION_CODEC
            || self.schema_version != SCIENTIFIC_REGISTRATION_SCHEMA_VERSION
        {
            return Err(Error::Validation(
                "unsupported scientific registration envelope version".to_string(),
            ));
        }
        self.actor.validate()?;
        if let Some(organization) = &self.acting_organization {
            organization.validate()?;
        }
        if self.stream_id != self.payload.registration_id() {
            return Err(Error::Validation(
                "registration stream id does not match payload".to_string(),
            ));
        }
        if self.sequence == 0 && self.previous_hash.is_some() {
            return Err(Error::Validation(
                "registration genesis cannot have a previous hash".to_string(),
            ));
        }
        if self.sequence > 0 && self.previous_hash.is_none() {
            return Err(Error::Validation(
                "non-genesis registration events require a previous hash".to_string(),
            ));
        }
        if let Some(key) = &self.idempotency_key {
            validate_text(key, "registration idempotency key", MAX_IDEMPOTENCY_KEY_BYTES)?;
        }
        self.payload.validate()
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut encoder = CanonicalEncoder::new(b"MYCELIX-DESCI-REGISTRATION-EVENT\0");
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
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedScientificRegistrationEvent {
    pub envelope: ScientificRegistrationEnvelope,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedScientificRegistrationEvent {
    pub fn sign(envelope: ScientificRegistrationEnvelope, key: &SigningKey) -> Result<Self> {
        envelope.validate()?;
        let signature = key.sign(&envelope.signing_bytes()?).to_bytes().to_vec();
        Ok(Self {
            envelope,
            signer_public_key: key.verifying_key().to_bytes(),
            signature,
        })
    }

    pub fn verify(&self) -> Result<()> {
        self.envelope.validate()?;
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify_strict(&self.envelope.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn event_hash(&self) -> Result<ContentHash> {
        self.verify()?;
        let signing_bytes = self.envelope.signing_bytes()?;
        let mut bytes = Vec::with_capacity(signing_bytes.len() + 128);
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-REGISTRATION-EVENT\0");
        push_len(&mut bytes, signing_bytes.len())?;
        bytes.extend_from_slice(&signing_bytes);
        bytes.extend_from_slice(&self.signer_public_key);
        push_len(&mut bytes, self.signature.len())?;
        bytes.extend_from_slice(&self.signature);
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistrationAppendReceipt {
    pub stream_id: ResearchRegistrationId,
    pub sequence: u64,
    pub event_id: ScientificRegistrationEventId,
    pub event_hash: ContentHash,
    pub received_at: DateTime<Utc>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RegistrationVersionRecord {
    pub version: u32,
    pub protocol: RegistrationProtocolDocument,
    pub source_event_id: ScientificRegistrationEventId,
    pub source_event_hash: ContentHash,
}

impl RegistrationVersionRecord {
    pub fn reference(&self, registration_id: ResearchRegistrationId) -> RegistrationReference {
        RegistrationReference {
            registration_id,
            version: self.version,
            version_event_hash: self.source_event_hash,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegistrationStatus {
    Registered,
    Sealed { version: u32 },
    Executing { sealed_version: u32 },
    Completed { sealed_version: u32 },
    Withdrawn { reason: String },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordedProtocolDeviation {
    pub event_id: ScientificRegistrationEventId,
    pub sealed_version: u32,
    pub reason: String,
    pub addendum: Option<RegistrationProtocolDocument>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ScientificRegistrationProjection {
    pub registration_id: ResearchRegistrationId,
    pub title: String,
    pub kind: RegistrationKind,
    pub creator: ActorId,
    pub creator_organization: Option<OrganizationId>,
    pub versions: Vec<RegistrationVersionRecord>,
    pub status: RegistrationStatus,
    pub deviations: Vec<RecordedProtocolDeviation>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub event_count: usize,
    pub last_event_hash: ContentHash,
}

impl ScientificRegistrationProjection {
    pub fn replay(events: &[SignedScientificRegistrationEvent]) -> Result<Self> {
        let first = events
            .first()
            .ok_or_else(|| Error::Validation("registration stream is empty".to_string()))?;
        first.verify()?;
        if first.envelope.sequence != 0 || first.envelope.previous_hash.is_some() {
            return Err(Error::Validation(
                "registration stream must begin at sequence zero".to_string(),
            ));
        }
        let first_hash = first.event_hash()?;
        let (registration_id, title, kind, protocol) = match &first.envelope.payload {
            ScientificRegistrationPayload::RegistrationCreated {
                registration_id,
                title,
                kind,
                protocol,
            } => (*registration_id, title.clone(), kind.clone(), protocol.clone()),
            _ => {
                return Err(Error::Validation(
                    "registration stream genesis must be registration_created".to_string(),
                ));
            }
        };

        let mut projection = Self {
            registration_id,
            title,
            kind,
            creator: first.envelope.actor.clone(),
            creator_organization: first.envelope.acting_organization.clone(),
            versions: vec![RegistrationVersionRecord {
                version: 1,
                protocol,
                source_event_id: first.envelope.event_id,
                source_event_hash: first_hash,
            }],
            status: RegistrationStatus::Registered,
            deviations: Vec::new(),
            created_at: first.envelope.occurred_at.clone(),
            updated_at: first.envelope.occurred_at.clone(),
            event_count: 1,
            last_event_hash: first_hash,
        };

        for (index, event) in events.iter().enumerate().skip(1) {
            event.verify()?;
            if event.envelope.stream_id != registration_id
                || event.envelope.sequence != index as u64
                || event.envelope.previous_hash != Some(projection.last_event_hash)
            {
                return Err(Error::Validation(
                    "registration event stream is discontinuous".to_string(),
                ));
            }
            projection.apply(event)?;
        }
        Ok(projection)
    }

    pub fn current_version(&self) -> &RegistrationVersionRecord {
        self.versions
            .last()
            .expect("registration projection always has a genesis version")
    }

    pub fn current_reference(&self) -> RegistrationReference {
        self.current_version().reference(self.registration_id)
    }

    fn apply(&mut self, event: &SignedScientificRegistrationEvent) -> Result<()> {
        match &event.envelope.payload {
            ScientificRegistrationPayload::RegistrationCreated { .. } => {
                return Err(Error::Validation(
                    "registration_created can appear only once".to_string(),
                ));
            }
            ScientificRegistrationPayload::RegistrationAmended {
                predecessor_version,
                predecessor_version_hash,
                protocol,
                ..
            } => {
                if !matches!(
                    &self.status,
                    RegistrationStatus::Registered | RegistrationStatus::Sealed { .. }
                ) {
                    return Err(Error::Validation(
                        "registration amendments are forbidden after execution starts"
                            .to_string(),
                    ));
                }
                let current_version = self.current_version().version;
                let current_hash = self.current_version().source_event_hash;
                if *predecessor_version != current_version
                    || *predecessor_version_hash != current_hash
                {
                    return Err(Error::Validation(
                        "registration amendment predecessor does not match current version"
                            .to_string(),
                    ));
                }
                self.versions.push(RegistrationVersionRecord {
                    version: current_version + 1,
                    protocol: protocol.clone(),
                    source_event_id: event.envelope.event_id,
                    source_event_hash: event.event_hash()?,
                });
                self.status = RegistrationStatus::Registered;
            }
            ScientificRegistrationPayload::RegistrationSealed { version, .. } => {
                if !matches!(&self.status, RegistrationStatus::Registered) {
                    return Err(Error::Validation(
                        "only a registered version can be sealed".to_string(),
                    ));
                }
                if *version != self.current_version().version {
                    return Err(Error::Validation(
                        "only the current registration version can be sealed".to_string(),
                    ));
                }
                self.status = RegistrationStatus::Sealed { version: *version };
            }
            ScientificRegistrationPayload::ExecutionStarted { sealed_version, .. } => {
                match &self.status {
                    RegistrationStatus::Sealed { version } if version == sealed_version => {}
                    _ => {
                        return Err(Error::Validation(
                            "execution requires the exact currently sealed registration version"
                                .to_string(),
                        ));
                    }
                }
                self.status = RegistrationStatus::Executing {
                    sealed_version: *sealed_version,
                };
            }
            ScientificRegistrationPayload::ProtocolDeviationRecorded {
                sealed_version,
                reason,
                addendum,
                ..
            } => {
                match &self.status {
                    RegistrationStatus::Executing {
                        sealed_version: active,
                    } if active == sealed_version => {}
                    _ => {
                        return Err(Error::Validation(
                            "protocol deviations can be recorded only during execution"
                                .to_string(),
                        ));
                    }
                }
                self.deviations.push(RecordedProtocolDeviation {
                    event_id: event.envelope.event_id,
                    sealed_version: *sealed_version,
                    reason: reason.clone(),
                    addendum: addendum.clone(),
                });
            }
            ScientificRegistrationPayload::ExecutionCompleted { sealed_version, .. } => {
                match &self.status {
                    RegistrationStatus::Executing {
                        sealed_version: active,
                    } if active == sealed_version => {}
                    _ => {
                        return Err(Error::Validation(
                            "execution completion requires an active matching execution"
                                .to_string(),
                        ));
                    }
                }
                self.status = RegistrationStatus::Completed {
                    sealed_version: *sealed_version,
                };
            }
            ScientificRegistrationPayload::RegistrationWithdrawn { reason, .. } => {
                if matches!(
                    &self.status,
                    RegistrationStatus::Completed { .. } | RegistrationStatus::Withdrawn { .. }
                ) {
                    return Err(Error::Validation(
                        "completed or already withdrawn registration cannot be withdrawn"
                            .to_string(),
                    ));
                }
                self.status = RegistrationStatus::Withdrawn {
                    reason: reason.clone(),
                };
            }
        }

        if event.envelope.occurred_at > self.updated_at {
            self.updated_at = event.envelope.occurred_at.clone();
        }
        self.event_count += 1;
        self.last_event_hash = event.event_hash()?;
        Ok(())
    }
}

#[derive(Debug, Default)]
struct RegistrationLogState {
    streams: HashMap<ResearchRegistrationId, Vec<SignedScientificRegistrationEvent>>,
    event_ids: HashSet<ScientificRegistrationEventId>,
    idempotency: HashMap<(ActorId, String), RegistrationAppendReceipt>,
}

/// In-memory authority backend used for deterministic tests and adapters.
/// Invalid lifecycle transitions are rejected before they enter the stream.
#[derive(Debug, Default)]
pub struct MemoryScientificRegistrationLog {
    state: RwLock<RegistrationLogState>,
}

impl MemoryScientificRegistrationLog {
    pub fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificRegistrationEvent,
        received_at: DateTime<Utc>,
    ) -> Result<RegistrationAppendReceipt> {
        event.verify()?;
        if event.envelope.occurred_at
            > received_at.clone() + Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::Validation(
                "registration event occurred_at exceeds allowed future clock skew".to_string(),
            ));
        }
        let event_hash = event.event_hash()?;
        let mut state = self
            .state
            .write()
            .map_err(|_| Error::Storage("registration log lock poisoned".to_string()))?;

        if let Some(key) = &event.envelope.idempotency_key {
            let idempotency_key = (event.envelope.actor.clone(), key.clone());
            if let Some(existing) = state.idempotency.get(&idempotency_key) {
                if existing.event_hash == event_hash {
                    return Ok(existing.clone());
                }
                return Err(Error::Storage(
                    "registration idempotency key reused for different event bytes".to_string(),
                ));
            }
        }

        if state.event_ids.contains(&event.envelope.event_id) {
            return Err(Error::Storage(
                "duplicate scientific registration event id".to_string(),
            ));
        }
        if event.envelope.sequence != expected_sequence {
            return Err(Error::Storage(format!(
                "registration optimistic concurrency failure: event sequence {} != expected {}",
                event.envelope.sequence, expected_sequence
            )));
        }

        let mut candidate = state
            .streams
            .get(&event.envelope.stream_id)
            .cloned()
            .unwrap_or_default();
        if candidate.len() as u64 != expected_sequence {
            return Err(Error::Storage(format!(
                "registration optimistic concurrency failure: stream length {} != expected {}",
                candidate.len(), expected_sequence
            )));
        }
        candidate.push(event.clone());
        ScientificRegistrationProjection::replay(&candidate)?;

        let receipt = RegistrationAppendReceipt {
            stream_id: event.envelope.stream_id,
            sequence: event.envelope.sequence,
            event_id: event.envelope.event_id,
            event_hash,
            received_at,
        };
        state.streams.insert(event.envelope.stream_id, candidate);
        state.event_ids.insert(event.envelope.event_id);
        if let Some(key) = &event.envelope.idempotency_key {
            state
                .idempotency
                .insert((event.envelope.actor.clone(), key.clone()), receipt.clone());
        }
        Ok(receipt)
    }

    pub fn append(
        &self,
        expected_sequence: u64,
        event: SignedScientificRegistrationEvent,
    ) -> Result<RegistrationAppendReceipt> {
        self.append_at(expected_sequence, event, Utc::now())
    }

    pub fn stream(
        &self,
        registration_id: ResearchRegistrationId,
    ) -> Result<Vec<SignedScientificRegistrationEvent>> {
        let state = self
            .state
            .read()
            .map_err(|_| Error::Storage("registration log lock poisoned".to_string()))?;
        Ok(state
            .streams
            .get(&registration_id)
            .cloned()
            .unwrap_or_default())
    }
}

fn require_version(version: u32) -> Result<()> {
    if version == 0 {
        return Err(Error::Validation(
            "registration version must be at least one".to_string(),
        ));
    }
    Ok(())
}

fn validate_text(value: &str, label: &str, max_bytes: usize) -> Result<()> {
    if value.trim().is_empty() {
        return Err(Error::Validation(format!("{label} cannot be empty")));
    }
    if value.trim() != value {
        return Err(Error::Validation(format!(
            "{label} cannot contain leading or trailing whitespace"
        )));
    }
    if value.len() > max_bytes {
        return Err(Error::Validation(format!(
            "{label} cannot exceed {max_bytes} bytes"
        )));
    }
    if value.chars().any(char::is_control) {
        return Err(Error::Validation(format!(
            "{label} cannot contain control characters"
        )));
    }
    Ok(())
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

    fn kind(&mut self, kind: &RegistrationKind) -> Result<()> {
        match kind {
            RegistrationKind::Preregistration => self.u8(1),
            RegistrationKind::RegisteredReport => self.u8(2),
            RegistrationKind::Protocol => self.u8(3),
            RegistrationKind::AnalysisPlan => self.u8(4),
            RegistrationKind::Other(value) => {
                self.u8(255);
                self.string(value)?;
            }
        }
        Ok(())
    }

    fn protocol_document(&mut self, protocol: &RegistrationProtocolDocument) -> Result<()> {
        self.hash(protocol.content_hash);
        self.string(&protocol.media_type)?;
        self.string(&protocol.locator)
    }

    fn option_protocol_document(
        &mut self,
        protocol: Option<&RegistrationProtocolDocument>,
    ) -> Result<()> {
        match protocol {
            Some(protocol) => {
                self.u8(1);
                self.protocol_document(protocol)?;
            }
            None => self.u8(0),
        }
        Ok(())
    }

    fn payload(&mut self, payload: &ScientificRegistrationPayload) -> Result<()> {
        match payload {
            ScientificRegistrationPayload::RegistrationCreated {
                registration_id,
                title,
                kind,
                protocol,
            } => {
                self.u8(1);
                self.uuid(registration_id.0);
                self.string(title)?;
                self.kind(kind)?;
                self.protocol_document(protocol)?;
            }
            ScientificRegistrationPayload::RegistrationAmended {
                registration_id,
                predecessor_version,
                predecessor_version_hash,
                protocol,
                reason,
            } => {
                self.u8(2);
                self.uuid(registration_id.0);
                self.u32(*predecessor_version);
                self.hash(*predecessor_version_hash);
                self.protocol_document(protocol)?;
                self.string(reason)?;
            }
            ScientificRegistrationPayload::RegistrationSealed {
                registration_id,
                version,
            } => {
                self.u8(3);
                self.uuid(registration_id.0);
                self.u32(*version);
            }
            ScientificRegistrationPayload::ExecutionStarted {
                registration_id,
                sealed_version,
            } => {
                self.u8(4);
                self.uuid(registration_id.0);
                self.u32(*sealed_version);
            }
            ScientificRegistrationPayload::ProtocolDeviationRecorded {
                registration_id,
                sealed_version,
                reason,
                addendum,
            } => {
                self.u8(5);
                self.uuid(registration_id.0);
                self.u32(*sealed_version);
                self.string(reason)?;
                self.option_protocol_document(addendum.as_ref())?;
            }
            ScientificRegistrationPayload::ExecutionCompleted {
                registration_id,
                sealed_version,
            } => {
                self.u8(6);
                self.uuid(registration_id.0);
                self.u32(*sealed_version);
            }
            ScientificRegistrationPayload::RegistrationWithdrawn {
                registration_id,
                reason,
            } => {
                self.u8(7);
                self.uuid(registration_id.0);
                self.string(reason)?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn actor() -> ActorId {
        ActorId::new("did:mycelix:researcher-1").unwrap()
    }

    fn key() -> SigningKey {
        SigningKey::from_bytes(&[7u8; 32])
    }

    fn document(label: &str) -> RegistrationProtocolDocument {
        RegistrationProtocolDocument {
            content_hash: ContentHash::digest(label.as_bytes()),
            media_type: "application/json".to_string(),
            locator: format!("cas:{label}"),
        }
    }

    fn created(
        id: ResearchRegistrationId,
        at: DateTime<Utc>,
    ) -> SignedScientificRegistrationEvent {
        let envelope = ScientificRegistrationEnvelope::genesis(
            actor(),
            at,
            ScientificRegistrationPayload::RegistrationCreated {
                registration_id: id,
                title: "Preregistered experiment".to_string(),
                kind: RegistrationKind::Preregistration,
                protocol: document("v1"),
            },
        )
        .unwrap();
        SignedScientificRegistrationEvent::sign(envelope, &key()).unwrap()
    }

    fn next(
        previous: &SignedScientificRegistrationEvent,
        at: DateTime<Utc>,
        payload: ScientificRegistrationPayload,
    ) -> SignedScientificRegistrationEvent {
        let envelope = ScientificRegistrationEnvelope::next(previous, actor(), at, payload).unwrap();
        SignedScientificRegistrationEvent::sign(envelope, &key()).unwrap()
    }

    #[test]
    fn registration_can_be_sealed_then_started() {
        let id = ResearchRegistrationId::new();
        let t0 = Utc::now();
        let e0 = created(id, t0.clone());
        let e1 = next(
            &e0,
            t0.clone() + Duration::seconds(1),
            ScientificRegistrationPayload::RegistrationSealed {
                registration_id: id,
                version: 1,
            },
        );
        let e2 = next(
            &e1,
            t0 + Duration::seconds(2),
            ScientificRegistrationPayload::ExecutionStarted {
                registration_id: id,
                sealed_version: 1,
            },
        );

        let projection = ScientificRegistrationProjection::replay(&[e0, e1, e2]).unwrap();
        assert_eq!(
            projection.status,
            RegistrationStatus::Executing { sealed_version: 1 }
        );
        assert_eq!(projection.current_reference().version, 1);
    }

    #[test]
    fn amendment_after_seal_requires_resealing_new_version() {
        let id = ResearchRegistrationId::new();
        let t0 = Utc::now();
        let e0 = created(id, t0.clone());
        let v1_hash = e0.event_hash().unwrap();
        let e1 = next(
            &e0,
            t0.clone() + Duration::seconds(1),
            ScientificRegistrationPayload::RegistrationSealed {
                registration_id: id,
                version: 1,
            },
        );
        let e2 = next(
            &e1,
            t0 + Duration::seconds(2),
            ScientificRegistrationPayload::RegistrationAmended {
                registration_id: id,
                predecessor_version: 1,
                predecessor_version_hash: v1_hash,
                protocol: document("v2"),
                reason: "Corrected sample-size calculation before execution".to_string(),
            },
        );
        let projection = ScientificRegistrationProjection::replay(&[e0, e1, e2]).unwrap();
        assert_eq!(projection.status, RegistrationStatus::Registered);
        assert_eq!(projection.current_reference().version, 2);
    }

    #[test]
    fn amendment_is_rejected_at_append_boundary_after_execution_starts() {
        let id = ResearchRegistrationId::new();
        let t0 = Utc::now();
        let e0 = created(id, t0.clone());
        let v1_hash = e0.event_hash().unwrap();
        let e1 = next(
            &e0,
            t0.clone() + Duration::seconds(1),
            ScientificRegistrationPayload::RegistrationSealed {
                registration_id: id,
                version: 1,
            },
        );
        let e2 = next(
            &e1,
            t0.clone() + Duration::seconds(2),
            ScientificRegistrationPayload::ExecutionStarted {
                registration_id: id,
                sealed_version: 1,
            },
        );
        let e3 = next(
            &e2,
            t0.clone() + Duration::seconds(3),
            ScientificRegistrationPayload::RegistrationAmended {
                registration_id: id,
                predecessor_version: 1,
                predecessor_version_hash: v1_hash,
                protocol: document("post-hoc"),
                reason: "Change made after execution started".to_string(),
            },
        );

        let log = MemoryScientificRegistrationLog::default();
        log.append_at(0, e0, t0.clone()).unwrap();
        log.append_at(1, e1, t0.clone() + Duration::seconds(1))
            .unwrap();
        log.append_at(2, e2, t0.clone() + Duration::seconds(2))
            .unwrap();
        let error = log
            .append_at(3, e3, t0 + Duration::seconds(3))
            .expect_err("post-start amendment must be rejected before commit");
        assert!(error.to_string().contains("forbidden after execution starts"));
        assert_eq!(log.stream(id).unwrap().len(), 3);
    }

    #[test]
    fn post_start_deviation_is_explicit_and_preserves_sealed_version() {
        let id = ResearchRegistrationId::new();
        let t0 = Utc::now();
        let e0 = created(id, t0.clone());
        let e1 = next(
            &e0,
            t0.clone() + Duration::seconds(1),
            ScientificRegistrationPayload::RegistrationSealed {
                registration_id: id,
                version: 1,
            },
        );
        let e2 = next(
            &e1,
            t0.clone() + Duration::seconds(2),
            ScientificRegistrationPayload::ExecutionStarted {
                registration_id: id,
                sealed_version: 1,
            },
        );
        let e3 = next(
            &e2,
            t0 + Duration::seconds(3),
            ScientificRegistrationPayload::ProtocolDeviationRecorded {
                registration_id: id,
                sealed_version: 1,
                reason: "Instrument failure required an alternate acquisition path".to_string(),
                addendum: Some(document("deviation-1")),
            },
        );

        let projection = ScientificRegistrationProjection::replay(&[e0, e1, e2, e3]).unwrap();
        assert_eq!(projection.current_reference().version, 1);
        assert_eq!(projection.deviations.len(), 1);
        assert_eq!(
            projection.status,
            RegistrationStatus::Executing { sealed_version: 1 }
        );
    }

    #[test]
    fn memory_log_is_idempotent_only_for_identical_signed_event() {
        let id = ResearchRegistrationId::new();
        let t0 = Utc::now();
        let envelope = ScientificRegistrationEnvelope::genesis(
            actor(),
            t0.clone(),
            ScientificRegistrationPayload::RegistrationCreated {
                registration_id: id,
                title: "Idempotent registration".to_string(),
                kind: RegistrationKind::Preregistration,
                protocol: document("idempotent"),
            },
        )
        .unwrap()
        .with_idempotency_key("register-once")
        .unwrap();
        let event = SignedScientificRegistrationEvent::sign(envelope, &key()).unwrap();
        let log = MemoryScientificRegistrationLog::default();

        let first = log.append_at(0, event.clone(), t0.clone()).unwrap();
        let replay = log
            .append_at(0, event, t0 + Duration::seconds(1))
            .unwrap();
        assert_eq!(first.event_hash, replay.event_hash);
        assert_eq!(log.stream(id).unwrap().len(), 1);
    }
}
