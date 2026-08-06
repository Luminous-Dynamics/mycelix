// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Threshold governance for the scientific credential registry.
//!
//! Credential events are powerful: they can authorize keys, grant roles, and
//! revoke administrators. A single administrator signature is therefore an
//! insufficient production authority boundary. This module adds a separate,
//! append-only governance journal with proposals, unique-actor approvals,
//! delayed activation, emergency cancellation, governed acceptance-service key
//! rotation, and externally publishable transparency checkpoints.
//!
//! The credential event being proposed is signed before governance begins and
//! is bound to the credential-registry head at proposal time. Approval events
//! do not move the credential head. Execution fails closed if another
//! credential mutation has changed that head, making parallel stale proposals
//! harmless rather than ambiguous.

use crate::authority_epoch::DatabaseEpochPromotionIntent;
use crate::postgres_authority::PostgresAuthorityStore;
use crate::scientific_credentials::{
    PreparedScientificCredentialAppend, RecordedScientificCredentialEvent,
    ScientificCredentialAppendReceipt, ScientificCredentialEventId, ScientificCredentialRegistry,
    ScientificCredentialRegistryProjection, SignedScientificCredentialEvent,
};
use crate::scientific_events::{
    ActorId, ContentHash, MAX_EVENT_FUTURE_SKEW_SECONDS, OrganizationId,
};
use crate::scientific_governance::{ScientificIdentityResolver, ScientificRole};
use crate::scientific_governance_quorum::{
    CredentialGovernanceApprovalRule, CredentialGovernanceRiskPolicy, CredentialGovernanceRiskTier,
};
use crate::transactional_file::persist_json_vec_transaction;
use crate::{Error, Result};
use chrono::{DateTime, Duration, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::{Mutex, RwLock};
use uuid::Uuid;

pub const CREDENTIAL_GOVERNANCE_PROTOCOL: &str = "mycelix-desci-credential-governance";
pub const CREDENTIAL_GOVERNANCE_PROTOCOL_VERSION: u16 = 1;
pub const CREDENTIAL_GOVERNANCE_SCHEMA_VERSION: u16 = 1;
pub const CREDENTIAL_GOVERNANCE_CODEC: &str = "mycelix-canonical-binary-v1";
pub const MAX_CREDENTIAL_GOVERNANCE_FILE_BYTES: u64 = 16 * 1024 * 1024;
pub const MAX_CREDENTIAL_GOVERNANCE_PENDING_BYTES: u64 = 1024 * 1024;
const MAX_REASON_BYTES: usize = 4096;
const MAX_IDEMPOTENCY_KEY_BYTES: usize = 512;
const MAX_ACTIVATION_DELAY_SECONDS: u64 = 30 * 24 * 60 * 60;
const MAX_PROPOSAL_TTL_SECONDS: u64 = 90 * 24 * 60 * 60;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CredentialGovernanceEventId(pub Uuid);

impl CredentialGovernanceEventId {
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for CredentialGovernanceEventId {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct CredentialGovernanceProposalId(pub Uuid);

impl CredentialGovernanceProposalId {
    pub fn new() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for CredentialGovernanceProposalId {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialGovernancePolicy {
    pub approval_threshold: u16,
    pub activation_delay_seconds: u64,
    pub proposal_ttl_seconds: u64,
    #[serde(default = "default_true")]
    pub proposer_counts_as_approval: bool,
    #[serde(default = "default_true")]
    pub emergency_cancellation_enabled: bool,
}

impl CredentialGovernancePolicy {
    pub fn validate(&self) -> Result<()> {
        if self.approval_threshold < 2 {
            return Err(Error::Validation(
                "credential governance approval threshold must be at least two".to_string(),
            ));
        }
        if self.activation_delay_seconds == 0
            || self.activation_delay_seconds > MAX_ACTIVATION_DELAY_SECONDS
        {
            return Err(Error::Validation(format!(
                "credential governance activation delay must be between 1 and {MAX_ACTIVATION_DELAY_SECONDS} seconds"
            )));
        }
        if self.proposal_ttl_seconds <= self.activation_delay_seconds
            || self.proposal_ttl_seconds > MAX_PROPOSAL_TTL_SECONDS
        {
            return Err(Error::Validation(format!(
                "credential governance proposal TTL must exceed the activation delay and be at most {MAX_PROPOSAL_TTL_SECONDS} seconds"
            )));
        }
        Ok(())
    }
}

fn default_true() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorizedCredentialAcceptanceKey {
    pub public_key: [u8; 32],
    pub valid_from: DateTime<Utc>,
    pub valid_until: Option<DateTime<Utc>>,
    pub revoked_at: Option<DateTime<Utc>>,
}

impl AuthorizedCredentialAcceptanceKey {
    pub fn validate(&self) -> Result<()> {
        VerifyingKey::from_bytes(&self.public_key).map_err(|error| {
            Error::Crypto(format!("invalid acceptance service public key: {error}"))
        })?;
        if self
            .valid_until
            .as_ref()
            .is_some_and(|until| until <= &self.valid_from)
        {
            return Err(Error::Validation(
                "acceptance service valid_until must be after valid_from".to_string(),
            ));
        }
        if self
            .revoked_at
            .as_ref()
            .is_some_and(|revoked| revoked < &self.valid_from)
        {
            return Err(Error::Validation(
                "acceptance service key cannot be revoked before valid_from".to_string(),
            ));
        }
        Ok(())
    }

    pub fn is_active_at(&self, at: &DateTime<Utc>) -> bool {
        at >= &self.valid_from
            && self.valid_until.as_ref().is_none_or(|until| at < until)
            && self.revoked_at.as_ref().is_none_or(|revoked| at < revoked)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorizedCredentialTransparencyWitness {
    pub actor: ActorId,
    pub organization: OrganizationId,
    pub public_key: [u8; 32],
    pub valid_from: DateTime<Utc>,
    pub valid_until: Option<DateTime<Utc>>,
    pub revoked_at: Option<DateTime<Utc>>,
}

impl AuthorizedCredentialTransparencyWitness {
    pub fn validate(&self) -> Result<()> {
        self.actor.validate()?;
        self.organization.validate()?;
        VerifyingKey::from_bytes(&self.public_key).map_err(|error| {
            Error::Crypto(format!("invalid transparency witness public key: {error}"))
        })?;
        if self
            .valid_until
            .as_ref()
            .is_some_and(|until| until <= &self.valid_from)
        {
            return Err(Error::Validation(
                "transparency witness valid_until must be after valid_from".to_string(),
            ));
        }
        if self
            .revoked_at
            .as_ref()
            .is_some_and(|revoked| revoked < &self.valid_from)
        {
            return Err(Error::Validation(
                "transparency witness cannot be revoked before valid_from".to_string(),
            ));
        }
        Ok(())
    }

    pub fn is_active_at(&self, at: &DateTime<Utc>) -> bool {
        at >= &self.valid_from
            && self.valid_until.as_ref().is_none_or(|until| at < until)
            && self.revoked_at.as_ref().is_none_or(|revoked| at < revoked)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TransparencyWitnessCompromiseInterval {
    pub compromised_from: DateTime<Utc>,
    pub restored_at: Option<DateTime<Utc>>,
    pub reason: String,
}

impl TransparencyWitnessCompromiseInterval {
    pub fn validate(&self) -> Result<()> {
        validate_reason(&self.reason)?;
        if self
            .restored_at
            .as_ref()
            .is_some_and(|restored| restored <= &self.compromised_from)
        {
            return Err(Error::Validation(
                "transparency witness restored_at must be after compromised_from".to_string(),
            ));
        }
        Ok(())
    }

    pub fn validate_at(&self, recorded_at: &DateTime<Utc>) -> Result<()> {
        self.validate()?;
        if &self.compromised_from > recorded_at
            || self
                .restored_at
                .as_ref()
                .is_some_and(|restored| restored > recorded_at)
        {
            return Err(Error::Validation(
                "transparency witness compromise intervals must be historical at execution time"
                    .to_string(),
            ));
        }
        Ok(())
    }

    pub fn contains(&self, at: &DateTime<Utc>) -> bool {
        at >= &self.compromised_from
            && self
                .restored_at
                .as_ref()
                .is_none_or(|restored| at < restored)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedCredentialTransparencyWitness {
    pub checkpoint_hash: ContentHash,
    pub actor: ActorId,
    pub organization: OrganizationId,
    pub witnessed_at: DateTime<Utc>,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedCredentialTransparencyWitness {
    pub fn sign(
        checkpoint_hash: ContentHash,
        actor: ActorId,
        organization: OrganizationId,
        witnessed_at: DateTime<Utc>,
        key: &SigningKey,
    ) -> Result<Self> {
        actor.validate()?;
        organization.validate()?;
        let mut witness = Self {
            checkpoint_hash,
            actor,
            organization,
            witnessed_at,
            signer_public_key: key.verifying_key().to_bytes(),
            signature: Vec::new(),
        };
        witness.signature = key.sign(&witness.signing_bytes()?).to_bytes().to_vec();
        Ok(witness)
    }

    fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.actor.validate()?;
        self.organization.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CREDENTIAL-CHECKPOINT-WITNESS\0");
        bytes.extend_from_slice(&self.checkpoint_hash.0);
        push_string(&mut bytes, self.actor.as_str())?;
        push_string(&mut bytes, self.organization.as_str())?;
        push_datetime(&mut bytes, &self.witnessed_at);
        bytes.extend_from_slice(&self.signer_public_key);
        Ok(bytes)
    }

    pub fn verify(&self) -> Result<()> {
        let key = VerifyingKey::from_bytes(&self.signer_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify_strict(&self.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type", deny_unknown_fields)]
pub enum CredentialGovernanceAction {
    AppendCredentialEvent {
        event: SignedScientificCredentialEvent,
    },
    AuthorizeAcceptanceServiceKey {
        key: AuthorizedCredentialAcceptanceKey,
    },
    RevokeAcceptanceServiceKey {
        public_key: [u8; 32],
        revoked_at: DateTime<Utc>,
        reason: String,
    },
    UpdateGovernancePolicy {
        policy: CredentialGovernancePolicy,
    },
    UpdateGovernanceRiskPolicy {
        policy: CredentialGovernanceRiskPolicy,
    },
    AuthorizeTransparencyWitness {
        witness: AuthorizedCredentialTransparencyWitness,
    },
    RevokeTransparencyWitness {
        actor: ActorId,
        revoked_at: DateTime<Utc>,
        reason: String,
    },
    RecordTransparencyWitnessCompromise {
        actor: ActorId,
        compromised_from: DateTime<Utc>,
        restored_at: Option<DateTime<Utc>>,
        reason: String,
    },
    AuthorizeDatabaseEpochPromotion {
        intent: DatabaseEpochPromotionIntent,
    },
}

impl CredentialGovernanceAction {
    pub fn validate(&self) -> Result<()> {
        match self {
            Self::AppendCredentialEvent { event } => {
                event.verify()?;
                if event.envelope.sequence == 0 {
                    return Err(Error::Validation(
                        "credential governance cannot propose registry genesis".to_string(),
                    ));
                }
            }
            Self::AuthorizeAcceptanceServiceKey { key } => {
                key.validate()?;
                if key.revoked_at.is_some() {
                    return Err(Error::Validation(
                        "new acceptance service keys cannot already be revoked".to_string(),
                    ));
                }
            }
            Self::RevokeAcceptanceServiceKey {
                public_key, reason, ..
            } => {
                VerifyingKey::from_bytes(public_key).map_err(|error| {
                    Error::Crypto(format!("invalid acceptance service public key: {error}"))
                })?;
                validate_reason(reason)?;
            }
            Self::UpdateGovernancePolicy { policy } => policy.validate()?,
            Self::UpdateGovernanceRiskPolicy { policy } => policy.validate()?,
            Self::AuthorizeTransparencyWitness { witness } => {
                witness.validate()?;
                if witness.revoked_at.is_some() {
                    return Err(Error::Validation(
                        "new transparency witnesses cannot already be revoked".to_string(),
                    ));
                }
            }
            Self::RevokeTransparencyWitness { actor, reason, .. } => {
                actor.validate()?;
                validate_reason(reason)?;
            }
            Self::RecordTransparencyWitnessCompromise {
                actor,
                compromised_from,
                restored_at,
                reason,
            } => {
                actor.validate()?;
                TransparencyWitnessCompromiseInterval {
                    compromised_from: compromised_from.clone(),
                    restored_at: restored_at.clone(),
                    reason: reason.clone(),
                }
                .validate()?;
            }
            Self::AuthorizeDatabaseEpochPromotion { intent } => intent.validate()?,
        }
        Ok(())
    }

    fn code(&self) -> u8 {
        match self {
            Self::AppendCredentialEvent { .. } => 1,
            Self::AuthorizeAcceptanceServiceKey { .. } => 2,
            Self::RevokeAcceptanceServiceKey { .. } => 3,
            Self::UpdateGovernancePolicy { .. } => 4,
            Self::UpdateGovernanceRiskPolicy { .. } => 5,
            Self::AuthorizeTransparencyWitness { .. } => 6,
            Self::RevokeTransparencyWitness { .. } => 7,
            Self::RecordTransparencyWitnessCompromise { .. } => 8,
            Self::AuthorizeDatabaseEpochPromotion { .. } => 9,
        }
    }

    pub fn action_hash(&self) -> Result<ContentHash> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CREDENTIAL-GOVERNANCE-ACTION\0");
        push_action(&mut bytes, self)?;
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialTransparencyCheckpoint {
    pub registry_event_count: u64,
    pub registry_head: ContentHash,
    pub registry_merkle_root: ContentHash,
    pub governance_event_count: u64,
    pub governance_head: ContentHash,
    pub governance_merkle_root: ContentHash,
    pub issued_at: DateTime<Utc>,
}

impl CredentialTransparencyCheckpoint {
    pub fn checkpoint_hash(&self) -> ContentHash {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CREDENTIAL-TRANSPARENCY-CHECKPOINT\0");
        bytes.extend_from_slice(&self.registry_event_count.to_be_bytes());
        bytes.extend_from_slice(&self.registry_head.0);
        bytes.extend_from_slice(&self.registry_merkle_root.0);
        bytes.extend_from_slice(&self.governance_event_count.to_be_bytes());
        bytes.extend_from_slice(&self.governance_head.0);
        bytes.extend_from_slice(&self.governance_merkle_root.0);
        bytes.extend_from_slice(&self.issued_at.timestamp().to_be_bytes());
        bytes.extend_from_slice(&self.issued_at.timestamp_subsec_nanos().to_be_bytes());
        ContentHash::digest(&bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type", deny_unknown_fields)]
pub enum CredentialGovernancePayload {
    PolicyInitialized {
        policy: CredentialGovernancePolicy,
        initial_acceptance_service_keys: Vec<AuthorizedCredentialAcceptanceKey>,
    },
    ProposalOpened {
        proposal_id: CredentialGovernanceProposalId,
        base_registry_head: ContentHash,
        action: CredentialGovernanceAction,
        activation_not_before: DateTime<Utc>,
        expires_at: DateTime<Utc>,
        reason: String,
    },
    ProposalApproved {
        proposal_id: CredentialGovernanceProposalId,
    },
    ProposalCancelled {
        proposal_id: CredentialGovernanceProposalId,
        reason: String,
    },
    ProposalExecuted {
        proposal_id: CredentialGovernanceProposalId,
    },
    TransparencyCheckpointPublished {
        checkpoint: CredentialTransparencyCheckpoint,
    },
    TransparencyCheckpointWitnessed {
        witness: SignedCredentialTransparencyWitness,
    },
}

impl CredentialGovernancePayload {
    fn validate(&self) -> Result<()> {
        match self {
            Self::PolicyInitialized {
                policy,
                initial_acceptance_service_keys,
            } => {
                policy.validate()?;
                if initial_acceptance_service_keys.is_empty() {
                    return Err(Error::Validation(
                        "credential governance requires at least one acceptance service key"
                            .to_string(),
                    ));
                }
                let mut keys = BTreeSet::new();
                for key in initial_acceptance_service_keys {
                    key.validate()?;
                    if !keys.insert(key.public_key) {
                        return Err(Error::Validation(
                            "credential governance initial acceptance keys contain duplicates"
                                .to_string(),
                        ));
                    }
                }
            }
            Self::ProposalOpened {
                action,
                activation_not_before,
                expires_at,
                reason,
                ..
            } => {
                action.validate()?;
                validate_reason(reason)?;
                if expires_at <= activation_not_before {
                    return Err(Error::Validation(
                        "credential governance proposal must expire after activation".to_string(),
                    ));
                }
            }
            Self::ProposalApproved { .. } | Self::ProposalExecuted { .. } => {}
            Self::ProposalCancelled { reason, .. } => validate_reason(reason)?,
            Self::TransparencyCheckpointPublished { checkpoint } => {
                if checkpoint.registry_event_count == 0 || checkpoint.governance_event_count == 0 {
                    return Err(Error::Validation(
                        "transparency checkpoints require non-empty registry and governance histories"
                            .to_string(),
                    ));
                }
            }
            Self::TransparencyCheckpointWitnessed { witness } => witness.verify()?,
        }
        Ok(())
    }

    fn code(&self) -> u8 {
        match self {
            Self::PolicyInitialized { .. } => 1,
            Self::ProposalOpened { .. } => 2,
            Self::ProposalApproved { .. } => 3,
            Self::ProposalCancelled { .. } => 4,
            Self::ProposalExecuted { .. } => 5,
            Self::TransparencyCheckpointPublished { .. } => 6,
            Self::TransparencyCheckpointWitnessed { .. } => 7,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialGovernanceEnvelope {
    pub protocol: String,
    pub protocol_version: u16,
    pub codec: String,
    pub schema_version: u16,
    pub event_id: CredentialGovernanceEventId,
    pub sequence: u64,
    pub previous_hash: Option<ContentHash>,
    pub actor: ActorId,
    pub occurred_at: DateTime<Utc>,
    pub idempotency_key: String,
    pub payload: CredentialGovernancePayload,
}

impl CredentialGovernanceEnvelope {
    pub fn new(
        sequence: u64,
        previous_hash: Option<ContentHash>,
        actor: ActorId,
        occurred_at: DateTime<Utc>,
        idempotency_key: impl Into<String>,
        payload: CredentialGovernancePayload,
    ) -> Result<Self> {
        let envelope = Self {
            protocol: CREDENTIAL_GOVERNANCE_PROTOCOL.to_string(),
            protocol_version: CREDENTIAL_GOVERNANCE_PROTOCOL_VERSION,
            codec: CREDENTIAL_GOVERNANCE_CODEC.to_string(),
            schema_version: CREDENTIAL_GOVERNANCE_SCHEMA_VERSION,
            event_id: CredentialGovernanceEventId::new(),
            sequence,
            previous_hash,
            actor,
            occurred_at,
            idempotency_key: idempotency_key.into(),
            payload,
        };
        envelope.validate()?;
        Ok(envelope)
    }

    fn validate(&self) -> Result<()> {
        if self.protocol != CREDENTIAL_GOVERNANCE_PROTOCOL
            || self.protocol_version != CREDENTIAL_GOVERNANCE_PROTOCOL_VERSION
            || self.codec != CREDENTIAL_GOVERNANCE_CODEC
            || self.schema_version != CREDENTIAL_GOVERNANCE_SCHEMA_VERSION
        {
            return Err(Error::Validation(
                "unsupported credential governance protocol".to_string(),
            ));
        }
        self.actor.validate()?;
        if self.sequence == 0 && self.previous_hash.is_some() {
            return Err(Error::Validation(
                "credential governance genesis cannot have a previous hash".to_string(),
            ));
        }
        if self.sequence > 0 && self.previous_hash.is_none() {
            return Err(Error::Validation(
                "non-genesis credential governance event requires a previous hash".to_string(),
            ));
        }
        if self.idempotency_key.trim().is_empty()
            || self.idempotency_key.len() > MAX_IDEMPOTENCY_KEY_BYTES
            || self.idempotency_key.chars().any(char::is_control)
        {
            return Err(Error::Validation(
                "credential governance idempotency key is invalid".to_string(),
            ));
        }
        self.payload.validate()
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CREDENTIAL-GOVERNANCE-EVENT\0");
        push_string(&mut bytes, &self.protocol)?;
        push_u16(&mut bytes, self.protocol_version);
        push_string(&mut bytes, &self.codec)?;
        push_u16(&mut bytes, self.schema_version);
        push_uuid(&mut bytes, self.event_id.0);
        push_u64(&mut bytes, self.sequence);
        push_option_hash(&mut bytes, self.previous_hash);
        push_string(&mut bytes, self.actor.as_str())?;
        push_datetime(&mut bytes, &self.occurred_at);
        push_string(&mut bytes, &self.idempotency_key)?;
        push_payload(&mut bytes, &self.payload)?;
        Ok(bytes)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedCredentialGovernanceEvent {
    pub envelope: CredentialGovernanceEnvelope,
    pub signer_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedCredentialGovernanceEvent {
    pub fn sign(envelope: CredentialGovernanceEnvelope, key: &SigningKey) -> Result<Self> {
        envelope.validate()?;
        Ok(Self {
            signature: key.sign(&envelope.signing_bytes()?).to_bytes().to_vec(),
            signer_public_key: key.verifying_key().to_bytes(),
            envelope,
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
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-CREDENTIAL-GOVERNANCE-EVENT\0");
        let signing = self.envelope.signing_bytes()?;
        push_len(&mut bytes, signing.len())?;
        bytes.extend_from_slice(&signing);
        bytes.extend_from_slice(&self.signer_public_key);
        push_len(&mut bytes, self.signature.len())?;
        bytes.extend_from_slice(&self.signature);
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RecordedCredentialGovernanceEvent {
    pub event: SignedCredentialGovernanceEvent,
    pub received_at: DateTime<Utc>,
    pub acceptance_service_public_key: [u8; 32],
    pub acceptance_signature: Vec<u8>,
}

impl RecordedCredentialGovernanceEvent {
    fn sign_acceptance(
        event: SignedCredentialGovernanceEvent,
        received_at: DateTime<Utc>,
        service_key: &SigningKey,
    ) -> Result<Self> {
        if event.signer_public_key == service_key.verifying_key().to_bytes() {
            return Err(Error::VerificationFailed(
                "governance actor and acceptance service must use distinct keys".to_string(),
            ));
        }
        let mut recorded = Self {
            event,
            received_at,
            acceptance_service_public_key: service_key.verifying_key().to_bytes(),
            acceptance_signature: Vec::new(),
        };
        recorded.acceptance_signature = service_key
            .sign(&recorded.acceptance_signing_bytes()?)
            .to_bytes()
            .to_vec();
        Ok(recorded)
    }

    fn acceptance_signing_bytes(&self) -> Result<Vec<u8>> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-CREDENTIAL-GOVERNANCE-ACCEPTANCE\0");
        bytes.extend_from_slice(&self.event.event_hash()?.0);
        push_datetime(&mut bytes, &self.received_at);
        Ok(bytes)
    }

    fn verify_acceptance_signature(&self) -> Result<()> {
        self.event.verify()?;
        if self.event.signer_public_key == self.acceptance_service_public_key {
            return Err(Error::VerificationFailed(
                "governance actor and acceptance service must use distinct keys".to_string(),
            ));
        }
        let key = VerifyingKey::from_bytes(&self.acceptance_service_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.acceptance_signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        key.verify_strict(&self.acceptance_signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn record_hash(&self) -> Result<ContentHash> {
        self.verify_acceptance_signature()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-RECORDED-CREDENTIAL-GOVERNANCE-EVENT\0");
        let acceptance = self.acceptance_signing_bytes()?;
        push_len(&mut bytes, acceptance.len())?;
        bytes.extend_from_slice(&acceptance);
        bytes.extend_from_slice(&self.acceptance_service_public_key);
        push_len(&mut bytes, self.acceptance_signature.len())?;
        bytes.extend_from_slice(&self.acceptance_signature);
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CredentialGovernanceProposalStatus {
    Pending,
    Cancelled,
    Executed,
    Expired,
    Stale,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialGovernanceProposal {
    pub proposal_id: CredentialGovernanceProposalId,
    pub proposer: ActorId,
    pub base_registry_head: ContentHash,
    pub action: CredentialGovernanceAction,
    pub action_hash: ContentHash,
    pub risk_tier: CredentialGovernanceRiskTier,
    pub required_approvals: u16,
    pub required_distinct_organizations: u16,
    pub opened_at: DateTime<Utc>,
    pub activation_not_before: DateTime<Utc>,
    pub expires_at: DateTime<Utc>,
    pub reason: String,
    pub approvals: BTreeSet<ActorId>,
    pub status: CredentialGovernanceProposalStatus,
    pub cancelled_by: Option<ActorId>,
    pub executed_at: Option<DateTime<Utc>>,
}

impl CredentialGovernanceProposal {
    pub fn effective_status(
        &self,
        now: &DateTime<Utc>,
        current_registry_head: Option<ContentHash>,
    ) -> CredentialGovernanceProposalStatus {
        if self.status != CredentialGovernanceProposalStatus::Pending {
            return self.status;
        }
        if now > &self.expires_at {
            return CredentialGovernanceProposalStatus::Expired;
        }
        if current_registry_head != Some(self.base_registry_head) {
            return CredentialGovernanceProposalStatus::Stale;
        }
        CredentialGovernanceProposalStatus::Pending
    }
}

fn proposal_approval_counts(
    proposal: &CredentialGovernanceProposal,
    registry: &ScientificCredentialRegistryProjection,
    at: &DateTime<Utc>,
) -> (usize, usize) {
    let mut active_actors = 0usize;
    let mut organizations = BTreeSet::new();
    for approver in &proposal.approvals {
        let Some(profile) = registry.resolve(approver) else {
            continue;
        };
        if !profile.has_role(ScientificRole::RegistryAdmin)
            || !profile
                .authorized_keys
                .iter()
                .any(|key| key.is_active_at(at))
        {
            continue;
        }
        active_actors += 1;
        // Each actor contributes at most one organization to diversity. The
        // canonical lowest identifier is deterministic and prevents a
        // multi-affiliated actor from satisfying several diversity slots.
        if let Some(organization) = profile.organizations.iter().next() {
            organizations.insert(organization.clone());
        }
    }
    (active_actors, organizations.len())
}

fn legacy_approval_rule(policy: &CredentialGovernancePolicy) -> CredentialGovernanceApprovalRule {
    CredentialGovernanceApprovalRule {
        approval_threshold: policy.approval_threshold,
        minimum_distinct_organizations: 0,
        activation_delay_seconds: policy.activation_delay_seconds,
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialGovernanceProjection {
    pub policy: Option<CredentialGovernancePolicy>,
    pub risk_policy: Option<CredentialGovernanceRiskPolicy>,
    pub proposals: BTreeMap<CredentialGovernanceProposalId, CredentialGovernanceProposal>,
    pub acceptance_service_keys: BTreeMap<[u8; 32], AuthorizedCredentialAcceptanceKey>,
    pub transparency_witnesses: BTreeMap<ActorId, AuthorizedCredentialTransparencyWitness>,
    pub transparency_witness_compromises:
        BTreeMap<ActorId, Vec<TransparencyWitnessCompromiseInterval>>,
    pub checkpoints: Vec<CredentialTransparencyCheckpoint>,
    pub checkpoint_witnesses:
        BTreeMap<ContentHash, BTreeMap<ActorId, SignedCredentialTransparencyWitness>>,
    pub head_hash: Option<ContentHash>,
    pub event_count: u64,
}

impl CredentialGovernanceProjection {
    pub fn active_acceptance_key(&self, key: &[u8; 32], at: &DateTime<Utc>) -> bool {
        self.acceptance_service_keys
            .get(key)
            .is_some_and(|authorized| authorized.is_active_at(at))
    }

    pub fn continuing_acceptance_key_count(&self, at: &DateTime<Utc>) -> usize {
        self.acceptance_service_keys
            .values()
            .filter(|key| {
                key.is_active_at(at) && key.valid_until.is_none() && key.revoked_at.is_none()
            })
            .count()
    }

    pub fn active_transparency_witness(
        &self,
        actor: &ActorId,
        key: &[u8; 32],
        at: &DateTime<Utc>,
    ) -> Option<&AuthorizedCredentialTransparencyWitness> {
        self.transparency_witnesses.get(actor).filter(|witness| {
            &witness.public_key == key
                && witness.is_active_at(at)
                && !self
                    .transparency_witness_compromises
                    .get(actor)
                    .is_some_and(|intervals| intervals.iter().any(|interval| interval.contains(at)))
        })
    }

    pub fn checkpoint_witness_organization_count(&self, checkpoint_hash: ContentHash) -> usize {
        self.checkpoint_witnesses
            .get(&checkpoint_hash)
            .map(|witnesses| {
                witnesses
                    .values()
                    .filter(|witness| {
                        !self
                            .transparency_witness_compromises
                            .get(&witness.actor)
                            .is_some_and(|intervals| {
                                intervals
                                    .iter()
                                    .any(|interval| interval.contains(&witness.witnessed_at))
                            })
                    })
                    .map(|witness| witness.organization.clone())
                    .collect::<BTreeSet<_>>()
                    .len()
            })
            .unwrap_or(0)
    }

    pub fn pending_proposals(&self) -> usize {
        self.proposals
            .values()
            .filter(|proposal| proposal.status == CredentialGovernanceProposalStatus::Pending)
            .count()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialGovernanceApprovalStatus {
    pub risk_tier: CredentialGovernanceRiskTier,
    pub required_approvals: u16,
    pub active_approvals: usize,
    pub required_distinct_organizations: u16,
    pub active_distinct_organizations: usize,
    pub activation_delay_elapsed: bool,
    pub registry_head_current: bool,
    pub executable: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialGovernanceAppendReceipt {
    pub event_id: CredentialGovernanceEventId,
    pub sequence: u64,
    pub event_hash: ContentHash,
    pub received_at: DateTime<Utc>,
    pub governance_revision: ContentHash,
    pub credential_receipt: Option<ScientificCredentialAppendReceipt>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CredentialGovernanceSummary {
    pub initialized: bool,
    pub event_count: usize,
    pub pending_proposals: usize,
    pub approval_threshold: Option<u16>,
    pub activation_delay_seconds: Option<u64>,
    pub risk_policy_configured: bool,
    pub routine_approval_threshold: Option<u16>,
    pub sensitive_approval_threshold: Option<u16>,
    pub critical_approval_threshold: Option<u16>,
    pub active_transparency_witnesses: usize,
    pub currently_compromised_transparency_witnesses: usize,
    pub witnessed_checkpoints: usize,
    pub latest_checkpoint_hash: Option<ContentHash>,
    pub latest_checkpoint_witness_organizations: usize,
    pub active_acceptance_service_keys: usize,
    pub continuing_acceptance_service_keys: usize,
    pub head_hash: Option<ContentHash>,
    pub durable: bool,
    pub acceptance_signing_configured: bool,
    pub current_acceptance_signing_key_active: bool,
    pub pending_execution_recovery: bool,
}

#[derive(Clone)]
pub struct ScientificCredentialGovernance {
    registry: Arc<ScientificCredentialRegistry>,
    state: Arc<RwLock<CredentialGovernanceState>>,
    persistence_path: Option<PathBuf>,
    pending_execution_path: Option<PathBuf>,
    bootstrap_acceptance_keys: BTreeSet<[u8; 32]>,
    acceptance_signing_key: Option<Arc<SigningKey>>,
    mutation_lock: Arc<Mutex<()>>,
    postgres_store: Option<Arc<PostgresAuthorityStore>>,
}

#[derive(Debug, Clone)]
struct CredentialGovernanceState {
    events: Vec<RecordedCredentialGovernanceEvent>,
    projection: CredentialGovernanceProjection,
    event_ids: HashSet<CredentialGovernanceEventId>,
    idempotency_keys: HashMap<(ActorId, String), CredentialGovernanceEventId>,
}

impl CredentialGovernanceState {
    fn from_events(
        events: Vec<RecordedCredentialGovernanceEvent>,
        projection: CredentialGovernanceProjection,
    ) -> Self {
        let event_ids = events
            .iter()
            .map(|recorded| recorded.event.envelope.event_id)
            .collect();
        let idempotency_keys = events
            .iter()
            .map(|recorded| {
                (
                    (
                        recorded.event.envelope.actor.clone(),
                        recorded.event.envelope.idempotency_key.clone(),
                    ),
                    recorded.event.envelope.event_id,
                )
            })
            .collect();
        Self {
            events,
            projection,
            event_ids,
            idempotency_keys,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PendingCredentialGovernanceExecution {
    expected_sequence: u64,
    recorded_event: RecordedCredentialGovernanceEvent,
}

impl ScientificCredentialGovernance {
    pub async fn memory(
        registry: Arc<ScientificCredentialRegistry>,
        bootstrap_acceptance_keys: BTreeSet<[u8; 32]>,
        acceptance_signing_key: Option<Arc<SigningKey>>,
    ) -> Result<Self> {
        let governance = Self {
            registry,
            state: Arc::new(RwLock::new(CredentialGovernanceState {
                events: Vec::new(),
                projection: CredentialGovernanceProjection::default(),
                event_ids: HashSet::new(),
                idempotency_keys: HashMap::new(),
            })),
            persistence_path: None,
            pending_execution_path: None,
            bootstrap_acceptance_keys,
            acceptance_signing_key,
            mutation_lock: Arc::new(Mutex::new(())),
            postgres_store: None,
        };
        Ok(governance)
    }

    pub async fn open_file(
        registry: Arc<ScientificCredentialRegistry>,
        path: impl Into<PathBuf>,
        bootstrap_acceptance_keys: BTreeSet<[u8; 32]>,
        acceptance_signing_key: Option<Arc<SigningKey>>,
    ) -> Result<Self> {
        let path = path.into();
        let pending_execution_path = path.with_extension("pending-execution.json");
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent)?;
        }
        let events: Vec<RecordedCredentialGovernanceEvent> = read_json_file(
            &path,
            MAX_CREDENTIAL_GOVERNANCE_FILE_BYTES,
            "credential governance journal",
        )?
        .unwrap_or_default();
        let projection =
            rebuild_projection(&events, registry.as_ref(), &bootstrap_acceptance_keys).await?;
        let event_ids = events
            .iter()
            .map(|recorded| recorded.event.envelope.event_id)
            .collect();
        let idempotency_keys = events
            .iter()
            .map(|recorded| {
                (
                    (
                        recorded.event.envelope.actor.clone(),
                        recorded.event.envelope.idempotency_key.clone(),
                    ),
                    recorded.event.envelope.event_id,
                )
            })
            .collect();
        let governance = Self {
            registry,
            state: Arc::new(RwLock::new(CredentialGovernanceState {
                events,
                projection,
                event_ids,
                idempotency_keys,
            })),
            persistence_path: Some(path),
            pending_execution_path: Some(pending_execution_path),
            bootstrap_acceptance_keys,
            acceptance_signing_key,
            mutation_lock: Arc::new(Mutex::new(())),
            postgres_store: None,
        };
        governance.reconcile_pending_execution().await?;
        if governance.state.read().await.projection.policy.is_some() {
            governance.registry.require_threshold_governance()?;
        }
        Ok(governance)
    }

    pub async fn open_postgres(
        registry: Arc<ScientificCredentialRegistry>,
        store: Arc<PostgresAuthorityStore>,
        bootstrap_acceptance_keys: BTreeSet<[u8; 32]>,
        acceptance_signing_key: Option<Arc<SigningKey>>,
    ) -> Result<Self> {
        registry.synchronize().await?;
        let events = store.governance_records().await?;
        let projection =
            rebuild_projection(&events, registry.as_ref(), &bootstrap_acceptance_keys).await?;
        let governance = Self {
            registry,
            state: Arc::new(RwLock::new(CredentialGovernanceState::from_events(
                events, projection,
            ))),
            persistence_path: None,
            pending_execution_path: None,
            bootstrap_acceptance_keys,
            acceptance_signing_key,
            mutation_lock: Arc::new(Mutex::new(())),
            postgres_store: Some(store),
        };
        if governance.state.read().await.projection.policy.is_some() {
            governance.registry.require_threshold_governance()?;
        }
        Ok(governance)
    }

    pub async fn synchronize(&self) -> Result<()> {
        let _mutation = self.mutation_lock.lock().await;
        self.synchronize_locked().await
    }

    async fn synchronize_locked(&self) -> Result<()> {
        let Some(store) = &self.postgres_store else {
            return Ok(());
        };
        self.registry.synchronize().await?;
        let events = store.governance_records().await?;
        let projection = rebuild_projection(
            &events,
            self.registry.as_ref(),
            &self.bootstrap_acceptance_keys,
        )
        .await?;
        *self.state.write().await = CredentialGovernanceState::from_events(events, projection);
        Ok(())
    }

    async fn existing_append_receipt(
        &self,
        event: &SignedCredentialGovernanceEvent,
    ) -> Result<Option<CredentialGovernanceAppendReceipt>> {
        let (recorded, proposal_action) = {
            let state = self.state.read().await;
            let Some(recorded) = state
                .events
                .iter()
                .find(|recorded| recorded.event.envelope.event_id == event.envelope.event_id)
                .cloned()
            else {
                return Ok(None);
            };
            if recorded.event != *event {
                return Err(Error::Storage(
                    "credential governance event id is already bound to different signed bytes"
                        .to_string(),
                ));
            }
            let proposal_action = match &recorded.event.envelope.payload {
                CredentialGovernancePayload::ProposalExecuted { proposal_id } => state
                    .projection
                    .proposals
                    .get(proposal_id)
                    .map(|proposal| proposal.action.clone()),
                _ => None,
            };
            (recorded, proposal_action)
        };
        let credential_receipt = match proposal_action {
            Some(CredentialGovernanceAction::AppendCredentialEvent { event }) => {
                let recorded_credential = self
                    .registry
                    .event(event.envelope.event_id)
                    .await
                    .ok_or_else(|| {
                        Error::VerificationFailed(
                            "executed governance event is missing its credential record"
                                .to_string(),
                        )
                    })?;
                if recorded_credential.event != event {
                    return Err(Error::VerificationFailed(
                        "executed governance proposal does not match the committed credential event"
                            .to_string(),
                    ));
                }
                Some(ScientificCredentialAppendReceipt {
                    event_id: recorded_credential.event.envelope.event_id,
                    sequence: recorded_credential.event.envelope.sequence,
                    event_hash: recorded_credential.event.event_hash()?,
                    received_at: recorded_credential.received_at.clone(),
                    registry_revision: recorded_credential.record_hash()?,
                })
            }
            _ => None,
        };
        Ok(Some(CredentialGovernanceAppendReceipt {
            event_id: recorded.event.envelope.event_id,
            sequence: recorded.event.envelope.sequence,
            event_hash: recorded.event.event_hash()?,
            received_at: recorded.received_at.clone(),
            governance_revision: recorded.record_hash()?,
            credential_receipt,
        }))
    }

    pub async fn append(
        &self,
        expected_sequence: u64,
        event: SignedCredentialGovernanceEvent,
    ) -> Result<CredentialGovernanceAppendReceipt> {
        let _mutation = self.mutation_lock.lock().await;
        self.synchronize_locked().await?;
        if let Some(receipt) = self.existing_append_receipt(&event).await? {
            return Ok(receipt);
        }
        if matches!(
            event.envelope.payload,
            CredentialGovernancePayload::ProposalExecuted { .. }
        ) {
            return Err(Error::Validation(
                "proposal execution must use the atomic execute operation".to_string(),
            ));
        }
        self.append_internal(expected_sequence, event, None).await
    }

    pub async fn execute(
        &self,
        expected_sequence: u64,
        event: SignedCredentialGovernanceEvent,
    ) -> Result<CredentialGovernanceAppendReceipt> {
        let _mutation = self.mutation_lock.lock().await;
        self.synchronize_locked().await?;
        if let Some(receipt) = self.existing_append_receipt(&event).await? {
            return Ok(receipt);
        }
        let proposal_id = match &event.envelope.payload {
            CredentialGovernancePayload::ProposalExecuted { proposal_id } => *proposal_id,
            _ => {
                return Err(Error::Validation(
                    "credential governance execute requires proposal_executed payload".to_string(),
                ));
            }
        };
        event.verify()?;
        let now = Utc::now();
        let (proposal, current_head) = {
            let state = self.state.read().await;
            self.validate_common(&state, expected_sequence, &event, &now)
                .await?;
            if state.projection.policy.is_none() {
                return Err(Error::VerificationFailed(
                    "credential governance is not initialized".to_string(),
                ));
            }
            let proposal = state
                .projection
                .proposals
                .get(&proposal_id)
                .cloned()
                .ok_or_else(|| {
                    Error::NotFound("credential governance proposal not found".to_string())
                })?;
            (proposal, state.projection.head_hash)
        };
        if proposal.status != CredentialGovernanceProposalStatus::Pending {
            return Err(Error::Validation(
                "credential governance proposal is not pending".to_string(),
            ));
        }
        if now < proposal.activation_not_before {
            return Err(Error::VerificationFailed(
                "credential governance activation delay has not elapsed".to_string(),
            ));
        }
        if now > proposal.expires_at {
            return Err(Error::VerificationFailed(
                "credential governance proposal has expired".to_string(),
            ));
        }
        let registry_projection = self.registry.projection().await;
        let (active_approval_count, active_organization_count) =
            proposal_approval_counts(&proposal, &registry_projection, &now);
        if active_approval_count < usize::from(proposal.required_approvals)
            || active_organization_count < usize::from(proposal.required_distinct_organizations)
        {
            return Err(Error::VerificationFailed(format!(
                "credential governance proposal has {active_approval_count}/{} active approvals and {active_organization_count}/{} distinct organizations",
                proposal.required_approvals, proposal.required_distinct_organizations
            )));
        }
        if registry_projection.head_hash != Some(proposal.base_registry_head) {
            return Err(Error::Storage(
                "credential governance proposal is stale because the registry head changed"
                    .to_string(),
            ));
        }
        if event.envelope.previous_hash != current_head {
            return Err(Error::Storage(
                "credential governance execution does not extend the current governance head"
                    .to_string(),
            ));
        }

        let recorded_event = self.prepare_recorded_event(event, now).await?;
        if let Some(store) = &self.postgres_store {
            // Hold the registry mutation boundary across prepare, SQL commit,
            // and in-memory apply so health/read replicas cannot refresh the
            // projection between those phases.
            let _registry_mutation = self.registry.lock_mutation().await;
            let prepared_credential: Option<PreparedScientificCredentialAppend> =
                match &proposal.action {
                    CredentialGovernanceAction::AppendCredentialEvent { event } => Some(
                        self.registry
                            .prepare_governance_approved_at(
                                event.envelope.sequence,
                                event.clone(),
                                recorded_event.received_at.clone(),
                            )
                            .await?,
                    ),
                    CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { .. }
                    | CredentialGovernanceAction::RevokeAcceptanceServiceKey { .. }
                    | CredentialGovernanceAction::UpdateGovernancePolicy { .. }
                    | CredentialGovernanceAction::UpdateGovernanceRiskPolicy { .. }
                    | CredentialGovernanceAction::AuthorizeTransparencyWitness { .. }
                    | CredentialGovernanceAction::RevokeTransparencyWitness { .. }
                    | CredentialGovernanceAction::RecordTransparencyWitnessCompromise { .. }
                    | CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { .. } => None,
                };
            let preview_registry = match &prepared_credential {
                Some(prepared) => Some(self.registry.preview_with_prepared(prepared).await?),
                None => None,
            };
            let validation_registry = preview_registry.as_ref().unwrap_or(self.registry.as_ref());
            let mut candidate = self.state.read().await.events.clone();
            candidate.push(recorded_event.clone());
            rebuild_projection(
                &candidate,
                validation_registry,
                &self.bootstrap_acceptance_keys,
            )
            .await?;
            store
                .commit_governance_execution(
                    registry_projection.event_count,
                    prepared_credential
                        .as_ref()
                        .map(|prepared| &prepared.recorded),
                    expected_sequence,
                    &recorded_event,
                )
                .await?;
            let credential_receipt = prepared_credential
                .as_ref()
                .map(|prepared| prepared.receipt.clone());
            if let Some(prepared) = prepared_credential {
                self.registry.apply_prepared(prepared).await?;
            }
            return self
                .append_recorded_internal(
                    expected_sequence,
                    recorded_event,
                    credential_receipt,
                    false,
                )
                .await;
        }

        if !matches!(
            &proposal.action,
            CredentialGovernanceAction::AppendCredentialEvent { .. }
        ) {
            let mut candidate = self.state.read().await.events.clone();
            candidate.push(recorded_event.clone());
            rebuild_projection(
                &candidate,
                self.registry.as_ref(),
                &self.bootstrap_acceptance_keys,
            )
            .await?;
        }
        self.persist_pending_execution(expected_sequence, &recorded_event)?;
        let credential_receipt = match &proposal.action {
            CredentialGovernanceAction::AppendCredentialEvent { event } => Some(
                self.registry
                    .append_governance_approved(event.envelope.sequence, event.clone())
                    .await?,
            ),
            CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { .. }
            | CredentialGovernanceAction::RevokeAcceptanceServiceKey { .. }
            | CredentialGovernanceAction::UpdateGovernancePolicy { .. }
            | CredentialGovernanceAction::UpdateGovernanceRiskPolicy { .. }
            | CredentialGovernanceAction::AuthorizeTransparencyWitness { .. }
            | CredentialGovernanceAction::RevokeTransparencyWitness { .. }
            | CredentialGovernanceAction::RecordTransparencyWitnessCompromise { .. }
            | CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { .. } => None,
        };
        let result = self
            .append_recorded_internal(expected_sequence, recorded_event, credential_receipt, true)
            .await;
        if result.is_ok() {
            self.remove_pending_execution()?;
        }
        result
    }

    async fn append_internal(
        &self,
        expected_sequence: u64,
        event: SignedCredentialGovernanceEvent,
        credential_receipt: Option<ScientificCredentialAppendReceipt>,
    ) -> Result<CredentialGovernanceAppendReceipt> {
        self.append_internal_at(expected_sequence, event, credential_receipt, Utc::now())
            .await
    }

    async fn append_internal_at(
        &self,
        expected_sequence: u64,
        event: SignedCredentialGovernanceEvent,
        credential_receipt: Option<ScientificCredentialAppendReceipt>,
        received_at: DateTime<Utc>,
    ) -> Result<CredentialGovernanceAppendReceipt> {
        let recorded = self.prepare_recorded_event(event, received_at).await?;
        self.append_recorded_internal(expected_sequence, recorded, credential_receipt, true)
            .await
    }

    async fn prepare_recorded_event(
        &self,
        event: SignedCredentialGovernanceEvent,
        received_at: DateTime<Utc>,
    ) -> Result<RecordedCredentialGovernanceEvent> {
        event.verify()?;
        let acceptance_key = self.acceptance_signing_key.as_ref().ok_or_else(|| {
            Error::VerificationFailed(
                "credential governance acceptance signing is not configured".to_string(),
            )
        })?;
        let public_key = acceptance_key.verifying_key().to_bytes();
        let state = self.state.read().await;
        if state.projection.policy.is_none() {
            if !self.bootstrap_acceptance_keys.contains(&public_key) {
                return Err(Error::VerificationFailed(
                    "credential governance genesis requires a bootstrap-trusted acceptance key"
                        .to_string(),
                ));
            }
        } else if !state
            .projection
            .active_acceptance_key(&public_key, &received_at)
        {
            return Err(Error::VerificationFailed(
                "configured acceptance service key is not active in threshold governance"
                    .to_string(),
            ));
        }
        drop(state);
        RecordedCredentialGovernanceEvent::sign_acceptance(event, received_at, acceptance_key)
    }

    async fn append_recorded_internal(
        &self,
        expected_sequence: u64,
        recorded: RecordedCredentialGovernanceEvent,
        credential_receipt: Option<ScientificCredentialAppendReceipt>,
        persist: bool,
    ) -> Result<CredentialGovernanceAppendReceipt> {
        recorded.verify_acceptance_signature()?;
        let event = recorded.event.clone();
        let received_at = recorded.received_at.clone();
        let mut state = self.state.write().await;
        self.validate_common(&state, expected_sequence, &event, &received_at)
            .await?;
        if let CredentialGovernancePayload::ProposalOpened {
            base_registry_head,
            action,
            ..
        } = &event.envelope.payload
        {
            let registry_projection = self.registry.projection().await;
            if registry_projection.head_hash != Some(*base_registry_head) {
                return Err(Error::Storage(
                    "credential governance proposal does not bind the current registry head"
                        .to_string(),
                ));
            }
            if let CredentialGovernanceAction::AppendCredentialEvent { event: proposed } = action {
                if proposed.envelope.previous_hash != Some(*base_registry_head)
                    || proposed.envelope.sequence != registry_projection.event_count
                {
                    return Err(Error::Storage(
                        "proposed credential event does not extend the current registry head"
                            .to_string(),
                    ));
                }
            }
        }
        if state.projection.policy.is_none() {
            if !self
                .bootstrap_acceptance_keys
                .contains(&recorded.acceptance_service_public_key)
            {
                return Err(Error::VerificationFailed(
                    "credential governance genesis requires a bootstrap-trusted acceptance key"
                        .to_string(),
                ));
            }
        } else if !state
            .projection
            .active_acceptance_key(&recorded.acceptance_service_public_key, &received_at)
        {
            return Err(Error::VerificationFailed(
                "recorded acceptance service key is not active in threshold governance".to_string(),
            ));
        }
        let event_hash = event.event_hash()?;
        let governance_revision = recorded.record_hash()?;
        let mut next_events = state.events.clone();
        next_events.push(recorded.clone());
        let next_projection = rebuild_projection(
            &next_events,
            self.registry.as_ref(),
            &self.bootstrap_acceptance_keys,
        )
        .await?;
        if persist {
            if let Some(store) = &self.postgres_store {
                store
                    .append_governance_record(expected_sequence, &recorded)
                    .await?;
            } else if let Some(path) = &self.persistence_path {
                persist_json_vec_transaction(
                    path,
                    &state.events,
                    &next_events,
                    MAX_CREDENTIAL_GOVERNANCE_FILE_BYTES,
                    "scientific credential governance journal",
                )?;
            }
        }
        state.events = next_events;
        state.projection = next_projection;
        state.event_ids.insert(event.envelope.event_id);
        state.idempotency_keys.insert(
            (
                event.envelope.actor.clone(),
                event.envelope.idempotency_key.clone(),
            ),
            event.envelope.event_id,
        );
        if matches!(
            event.envelope.payload,
            CredentialGovernancePayload::PolicyInitialized { .. }
        ) {
            self.registry.require_threshold_governance()?;
        }
        Ok(CredentialGovernanceAppendReceipt {
            event_id: event.envelope.event_id,
            sequence: event.envelope.sequence,
            event_hash,
            received_at,
            governance_revision,
            credential_receipt,
        })
    }

    async fn validate_common(
        &self,
        state: &CredentialGovernanceState,
        expected_sequence: u64,
        event: &SignedCredentialGovernanceEvent,
        received_at: &DateTime<Utc>,
    ) -> Result<()> {
        if event.envelope.occurred_at
            > received_at.clone() + Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "credential governance event is beyond the allowed future clock skew".to_string(),
            ));
        }
        if expected_sequence != state.events.len() as u64
            || event.envelope.sequence != expected_sequence
        {
            return Err(Error::Storage(format!(
                "credential governance optimistic concurrency failure: expected sequence {}, received event sequence {}",
                state.events.len(),
                event.envelope.sequence
            )));
        }
        if event.envelope.previous_hash != state.projection.head_hash {
            return Err(Error::Storage(
                "credential governance event does not extend the current head".to_string(),
            ));
        }
        if state.event_ids.contains(&event.envelope.event_id) {
            return Err(Error::Storage(
                "duplicate credential governance event id".to_string(),
            ));
        }
        if let Some(existing) = state.idempotency_keys.get(&(
            event.envelope.actor.clone(),
            event.envelope.idempotency_key.clone(),
        )) {
            return Err(Error::Storage(format!(
                "credential governance idempotency key already belongs to event {}",
                existing.0
            )));
        }
        if let CredentialGovernancePayload::TransparencyCheckpointWitnessed { witness } =
            &event.envelope.payload
        {
            witness.verify()?;
            if witness.actor != event.envelope.actor
                || witness.signer_public_key != event.signer_public_key
                || witness.witnessed_at != event.envelope.occurred_at
            {
                return Err(Error::VerificationFailed(
                    "checkpoint witness payload and governance envelope signer do not match"
                        .to_string(),
                ));
            }
            let authorized = state
                .projection
                .active_transparency_witness(
                    &witness.actor,
                    &witness.signer_public_key,
                    received_at,
                )
                .ok_or_else(|| {
                    Error::VerificationFailed(
                        "checkpoint witness key is not governed and active".to_string(),
                    )
                })?;
            if authorized.organization != witness.organization {
                return Err(Error::VerificationFailed(
                    "checkpoint witness organization does not match governed authority".to_string(),
                ));
            }
            return Ok(());
        }

        let actor = self
            .registry
            .resolve(&event.envelope.actor, received_at)
            .await?
            .ok_or_else(|| {
                Error::VerificationFailed(
                    "credential governance signer is not a registered actor".to_string(),
                )
            })?;
        if !actor.has_role(ScientificRole::RegistryAdmin)
            || !actor.authorizes_key(&event.signer_public_key, received_at)
        {
            return Err(Error::VerificationFailed(
                "credential governance mutation requires an active registry administrator key"
                    .to_string(),
            ));
        }
        Ok(())
    }

    async fn reconcile_pending_execution(&self) -> Result<()> {
        let _mutation = self.mutation_lock.lock().await;
        let Some(path) = &self.pending_execution_path else {
            return Ok(());
        };
        let Some(pending): Option<PendingCredentialGovernanceExecution> = read_json_file(
            path,
            MAX_CREDENTIAL_GOVERNANCE_PENDING_BYTES,
            "credential governance pending execution",
        )?
        else {
            return Ok(());
        };
        pending.recorded_event.verify_acceptance_signature()?;
        let execution_event = &pending.recorded_event.event;
        let proposal_id = match &execution_event.envelope.payload {
            CredentialGovernancePayload::ProposalExecuted { proposal_id } => *proposal_id,
            _ => {
                return Err(Error::Storage(
                    "pending credential governance execution has the wrong payload".to_string(),
                ));
            }
        };
        if let Some(committed) = self
            .state
            .read()
            .await
            .events
            .iter()
            .find(|recorded| recorded.event.envelope.event_id == execution_event.envelope.event_id)
            .cloned()
        {
            if committed.record_hash()? != pending.recorded_event.record_hash()? {
                return Err(Error::VerificationFailed(
                    "pending governance execution conflicts with the committed record".to_string(),
                ));
            }
            return self.remove_pending_execution();
        }
        let proposal = self
            .state
            .read()
            .await
            .projection
            .proposals
            .get(&proposal_id)
            .cloned()
            .ok_or_else(|| {
                Error::Storage("pending execution references an unknown proposal".to_string())
            })?;
        let credential_receipt = match proposal.action {
            CredentialGovernanceAction::AppendCredentialEvent { event } => {
                let Some(recorded) = self.registry.event(event.envelope.event_id).await else {
                    self.remove_pending_execution()?;
                    return Ok(());
                };
                if recorded.event.event_hash()? != event.event_hash()? {
                    return Err(Error::VerificationFailed(
                        "pending execution credential event conflicts with the committed event"
                            .to_string(),
                    ));
                }
                Some(ScientificCredentialAppendReceipt {
                    event_id: recorded.event.envelope.event_id,
                    sequence: recorded.event.envelope.sequence,
                    event_hash: recorded.event.event_hash()?,
                    received_at: recorded.received_at.clone(),
                    registry_revision: recorded.record_hash()?,
                })
            }
            CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { .. }
            | CredentialGovernanceAction::RevokeAcceptanceServiceKey { .. }
            | CredentialGovernanceAction::UpdateGovernancePolicy { .. }
            | CredentialGovernanceAction::UpdateGovernanceRiskPolicy { .. }
            | CredentialGovernanceAction::AuthorizeTransparencyWitness { .. }
            | CredentialGovernanceAction::RevokeTransparencyWitness { .. }
            | CredentialGovernanceAction::RecordTransparencyWitnessCompromise { .. }
            | CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { .. } => None,
        };
        self.append_recorded_internal(
            pending.expected_sequence,
            pending.recorded_event,
            credential_receipt,
            true,
        )
        .await?;
        self.remove_pending_execution()
    }

    fn persist_pending_execution(
        &self,
        expected_sequence: u64,
        recorded_event: &RecordedCredentialGovernanceEvent,
    ) -> Result<()> {
        let Some(path) = &self.pending_execution_path else {
            return Ok(());
        };
        persist_json(
            path,
            &PendingCredentialGovernanceExecution {
                expected_sequence,
                recorded_event: recorded_event.clone(),
            },
            MAX_CREDENTIAL_GOVERNANCE_PENDING_BYTES,
        )
    }

    fn remove_pending_execution(&self) -> Result<()> {
        let Some(path) = &self.pending_execution_path else {
            return Ok(());
        };
        if path.exists() {
            fs::remove_file(path)?;
            if let Some(parent) = path
                .parent()
                .filter(|parent| !parent.as_os_str().is_empty())
            {
                File::open(parent)?.sync_all()?;
            }
        }
        Ok(())
    }

    pub async fn events(&self) -> Vec<RecordedCredentialGovernanceEvent> {
        self.state.read().await.events.clone()
    }

    pub async fn projection(&self) -> CredentialGovernanceProjection {
        self.state.read().await.projection.clone()
    }

    /// Current credential-registry head used to bind a new threshold proposal.
    pub async fn credential_registry_head(&self) -> Result<Option<ContentHash>> {
        Ok(self.registry.projection().await.head_hash)
    }

    pub async fn event(
        &self,
        event_id: CredentialGovernanceEventId,
    ) -> Option<RecordedCredentialGovernanceEvent> {
        self.state
            .read()
            .await
            .events
            .iter()
            .find(|recorded| recorded.event.envelope.event_id == event_id)
            .cloned()
    }

    pub async fn proposal(
        &self,
        proposal_id: CredentialGovernanceProposalId,
    ) -> Option<CredentialGovernanceProposal> {
        self.state
            .read()
            .await
            .projection
            .proposals
            .get(&proposal_id)
            .cloned()
    }

    pub async fn approval_status(
        &self,
        proposal_id: CredentialGovernanceProposalId,
        at: DateTime<Utc>,
    ) -> Result<CredentialGovernanceApprovalStatus> {
        let proposal = self.proposal(proposal_id).await.ok_or_else(|| {
            Error::NotFound("credential governance proposal not found".to_string())
        })?;
        let registry = self.registry.projection_at(&at).await?;
        let (active_approvals, active_distinct_organizations) =
            proposal_approval_counts(&proposal, &registry, &at);
        let activation_delay_elapsed =
            at >= proposal.activation_not_before && at <= proposal.expires_at;
        let registry_head_current = registry.head_hash == Some(proposal.base_registry_head);
        let executable = proposal.status == CredentialGovernanceProposalStatus::Pending
            && activation_delay_elapsed
            && registry_head_current
            && active_approvals >= usize::from(proposal.required_approvals)
            && active_distinct_organizations
                >= usize::from(proposal.required_distinct_organizations);
        Ok(CredentialGovernanceApprovalStatus {
            risk_tier: proposal.risk_tier,
            required_approvals: proposal.required_approvals,
            active_approvals,
            required_distinct_organizations: proposal.required_distinct_organizations,
            active_distinct_organizations,
            activation_delay_elapsed,
            registry_head_current,
            executable,
        })
    }

    pub async fn summary(&self) -> CredentialGovernanceSummary {
        let state = self.state.read().await;
        let now = Utc::now();
        CredentialGovernanceSummary {
            initialized: state.projection.policy.is_some(),
            event_count: state.events.len(),
            pending_proposals: state.projection.pending_proposals(),
            approval_threshold: state
                .projection
                .policy
                .as_ref()
                .map(|policy| policy.approval_threshold),
            activation_delay_seconds: state
                .projection
                .policy
                .as_ref()
                .map(|policy| policy.activation_delay_seconds),
            risk_policy_configured: state.projection.risk_policy.is_some(),
            routine_approval_threshold: state
                .projection
                .risk_policy
                .as_ref()
                .map(|policy| policy.routine.approval_threshold),
            sensitive_approval_threshold: state
                .projection
                .risk_policy
                .as_ref()
                .map(|policy| policy.sensitive.approval_threshold),
            critical_approval_threshold: state
                .projection
                .risk_policy
                .as_ref()
                .map(|policy| policy.critical.approval_threshold),
            active_transparency_witnesses: state
                .projection
                .transparency_witnesses
                .values()
                .filter(|witness| witness.is_active_at(&now))
                .count(),
            currently_compromised_transparency_witnesses: state
                .projection
                .transparency_witness_compromises
                .values()
                .filter(|intervals| intervals.iter().any(|interval| interval.contains(&now)))
                .count(),
            witnessed_checkpoints: state
                .projection
                .checkpoint_witnesses
                .values()
                .filter(|witnesses| !witnesses.is_empty())
                .count(),
            latest_checkpoint_hash: state
                .projection
                .checkpoints
                .last()
                .map(CredentialTransparencyCheckpoint::checkpoint_hash),
            latest_checkpoint_witness_organizations: state
                .projection
                .checkpoints
                .last()
                .map(|checkpoint| {
                    state
                        .projection
                        .checkpoint_witness_organization_count(checkpoint.checkpoint_hash())
                })
                .unwrap_or(0),
            active_acceptance_service_keys: state
                .projection
                .acceptance_service_keys
                .values()
                .filter(|key| key.is_active_at(&now))
                .count(),
            continuing_acceptance_service_keys: state
                .projection
                .continuing_acceptance_key_count(&now),
            head_hash: state.projection.head_hash,
            durable: self.persistence_path.is_some() || self.postgres_store.is_some(),
            acceptance_signing_configured: self.acceptance_signing_key.is_some(),
            current_acceptance_signing_key_active: self
                .acceptance_signing_key
                .as_ref()
                .is_some_and(|key| {
                    state
                        .projection
                        .active_acceptance_key(&key.verifying_key().to_bytes(), &now)
                }),
            pending_execution_recovery: self
                .pending_execution_path
                .as_ref()
                .is_some_and(|path| path.exists()),
        }
    }

    pub async fn checkpoint_candidate(&self) -> Result<CredentialTransparencyCheckpoint> {
        let governance_events = self.events().await;
        let registry_events = self.registry.events().await;
        let registry_head = registry_events
            .last()
            .ok_or_else(|| Error::Validation("credential registry is empty".to_string()))?
            .record_hash()?;
        let governance_head = governance_events
            .last()
            .ok_or_else(|| Error::Validation("credential governance is empty".to_string()))?
            .record_hash()?;
        let registry_hashes = registry_events
            .iter()
            .map(RecordedScientificCredentialEvent::record_hash)
            .collect::<Result<Vec<_>>>()?;
        let governance_hashes = governance_events
            .iter()
            .map(RecordedCredentialGovernanceEvent::record_hash)
            .collect::<Result<Vec<_>>>()?;
        Ok(CredentialTransparencyCheckpoint {
            registry_event_count: registry_events.len() as u64,
            registry_head,
            registry_merkle_root: merkle_root(&registry_hashes)?,
            governance_event_count: governance_events.len() as u64,
            governance_head,
            governance_merkle_root: merkle_root(&governance_hashes)?,
            issued_at: Utc::now(),
        })
    }
}

async fn rebuild_projection(
    events: &[RecordedCredentialGovernanceEvent],
    registry: &ScientificCredentialRegistry,
    bootstrap_acceptance_keys: &BTreeSet<[u8; 32]>,
) -> Result<CredentialGovernanceProjection> {
    let mut projection = CredentialGovernanceProjection::default();
    let mut event_ids = HashSet::new();
    let mut idempotency_keys = HashSet::new();
    for (index, recorded) in events.iter().enumerate() {
        recorded.verify_acceptance_signature()?;
        let event = &recorded.event;
        if event.envelope.sequence != index as u64 {
            return Err(Error::VerificationFailed(
                "credential governance sequence is discontinuous".to_string(),
            ));
        }
        if index > 0 && recorded.received_at < events[index - 1].received_at {
            return Err(Error::VerificationFailed(
                "credential governance acceptance timestamps move backward".to_string(),
            ));
        }
        if event.envelope.occurred_at
            > recorded.received_at.clone() + Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS)
        {
            return Err(Error::VerificationFailed(
                "credential governance event exceeds future clock skew at receipt".to_string(),
            ));
        }
        if event.envelope.previous_hash != projection.head_hash {
            return Err(Error::VerificationFailed(
                "credential governance hash chain is discontinuous".to_string(),
            ));
        }
        if !event_ids.insert(event.envelope.event_id) {
            return Err(Error::VerificationFailed(
                "credential governance contains a duplicate event id".to_string(),
            ));
        }
        if !idempotency_keys.insert((
            event.envelope.actor.clone(),
            event.envelope.idempotency_key.clone(),
        )) {
            return Err(Error::VerificationFailed(
                "credential governance contains a duplicate actor idempotency key".to_string(),
            ));
        }
        if let CredentialGovernancePayload::TransparencyCheckpointWitnessed { witness } =
            &event.envelope.payload
        {
            witness.verify()?;
            if witness.actor != event.envelope.actor
                || witness.signer_public_key != event.signer_public_key
                || witness.witnessed_at != event.envelope.occurred_at
            {
                return Err(Error::VerificationFailed(
                    "checkpoint witness payload and governance envelope signer do not match"
                        .to_string(),
                ));
            }
            let authorized = projection
                .active_transparency_witness(
                    &witness.actor,
                    &witness.signer_public_key,
                    &recorded.received_at,
                )
                .ok_or_else(|| {
                    Error::VerificationFailed(
                        "checkpoint witness lacked governed authority at receipt time".to_string(),
                    )
                })?;
            if authorized.organization != witness.organization {
                return Err(Error::VerificationFailed(
                    "checkpoint witness organization does not match governed authority".to_string(),
                ));
            }
        } else {
            let actor = registry
                .resolve(&event.envelope.actor, &recorded.received_at)
                .await?
                .ok_or_else(|| {
                    Error::VerificationFailed(
                        "credential governance signer was not registered at receipt time"
                            .to_string(),
                    )
                })?;
            if !actor.has_role(ScientificRole::RegistryAdmin)
                || !actor.authorizes_key(&event.signer_public_key, &recorded.received_at)
            {
                return Err(Error::VerificationFailed(
                    "credential governance signer lacked administrator authority at receipt time"
                        .to_string(),
                ));
            }
        }
        if projection.policy.is_none() {
            if index != 0
                || !matches!(
                    event.envelope.payload,
                    CredentialGovernancePayload::PolicyInitialized { .. }
                )
                || !bootstrap_acceptance_keys.contains(&recorded.acceptance_service_public_key)
            {
                return Err(Error::VerificationFailed(
                    "credential governance genesis requires a bootstrap-trusted acceptance key"
                        .to_string(),
                ));
            }
        } else if !projection.active_acceptance_key(
            &recorded.acceptance_service_public_key,
            &recorded.received_at,
        ) {
            return Err(Error::VerificationFailed(
                "credential governance record uses an unauthorized acceptance service key"
                    .to_string(),
            ));
        }

        apply_governance_transition(&mut projection, recorded, registry, events).await?;
        projection.head_hash = Some(recorded.record_hash()?);
        projection.event_count += 1;
    }
    Ok(projection)
}

async fn apply_governance_transition(
    projection: &mut CredentialGovernanceProjection,
    recorded: &RecordedCredentialGovernanceEvent,
    registry: &ScientificCredentialRegistry,
    prior_events: &[RecordedCredentialGovernanceEvent],
) -> Result<()> {
    let event = &recorded.event;
    match &event.envelope.payload {
        CredentialGovernancePayload::PolicyInitialized {
            policy,
            initial_acceptance_service_keys,
        } => {
            if projection.policy.is_some() || projection.event_count != 0 {
                return Err(Error::Validation(
                    "credential governance policy can only be initialized once".to_string(),
                ));
            }
            let registry_projection = registry.projection_at(&recorded.received_at).await?;
            for key in initial_acceptance_service_keys {
                if registry_projection.actors.values().any(|profile| {
                    profile
                        .authorized_keys
                        .iter()
                        .any(|actor_key| actor_key.public_key == key.public_key)
                }) {
                    return Err(Error::VerificationFailed(
                        "credential governance acceptance keys cannot also be scientific actor keys"
                            .to_string(),
                    ));
                }
            }
            if registry_projection.active_admin_count(&recorded.received_at)
                < usize::from(policy.approval_threshold)
            {
                return Err(Error::VerificationFailed(
                    "credential governance threshold exceeds active registry administrators"
                        .to_string(),
                ));
            }
            let mut includes_current_acceptance_key = false;
            for key in initial_acceptance_service_keys {
                if key.public_key == recorded.acceptance_service_public_key
                    && key.is_active_at(&recorded.received_at)
                {
                    includes_current_acceptance_key = true;
                }
                projection
                    .acceptance_service_keys
                    .insert(key.public_key, key.clone());
            }
            if !includes_current_acceptance_key {
                return Err(Error::VerificationFailed(
                    "governance genesis must authorize its acceptance service key".to_string(),
                ));
            }
            if projection.continuing_acceptance_key_count(&recorded.received_at) == 0 {
                return Err(Error::VerificationFailed(
                    "credential governance requires a non-expiring acceptance-service recovery key"
                        .to_string(),
                ));
            }
            projection.policy = Some(policy.clone());
        }
        CredentialGovernancePayload::ProposalOpened {
            proposal_id,
            base_registry_head,
            action,
            activation_not_before,
            expires_at,
            reason,
        } => {
            let policy = projection.policy.as_ref().ok_or_else(|| {
                Error::VerificationFailed("credential governance policy is absent".to_string())
            })?;
            if projection.proposals.contains_key(proposal_id) {
                return Err(Error::Validation(
                    "credential governance proposal id already exists".to_string(),
                ));
            }
            if let CredentialGovernanceAction::AppendCredentialEvent { event: proposed } = action {
                if proposed.envelope.actor != event.envelope.actor {
                    return Err(Error::VerificationFailed(
                        "credential proposal actor must sign the proposed credential event"
                            .to_string(),
                    ));
                }
                let registry_events = registry.events().await;
                let base_position = registry_events
                    .iter()
                    .position(|recorded| recorded.record_hash().ok() == Some(*base_registry_head))
                    .ok_or_else(|| {
                        Error::VerificationFailed(
                            "credential proposal base registry head is absent from local history"
                                .to_string(),
                        )
                    })?;
                if proposed.envelope.previous_hash != Some(*base_registry_head)
                    || proposed.envelope.sequence != (base_position + 1) as u64
                {
                    return Err(Error::Storage(
                        "proposed credential event does not extend the bound registry head"
                            .to_string(),
                    ));
                }
            }
            let risk_tier = action.risk_tier();
            let approval_rule = projection
                .risk_policy
                .as_ref()
                .map(|risk_policy| risk_policy.rule(risk_tier).clone())
                .unwrap_or_else(|| legacy_approval_rule(policy));
            let minimum_activation = recorded.received_at.clone()
                + Duration::seconds(approval_rule.activation_delay_seconds as i64);
            let maximum_expiry = recorded.received_at.clone()
                + Duration::seconds(policy.proposal_ttl_seconds as i64);
            // Client times are signed intent, but the accepted proposal uses
            // server-derived bounds. A client cannot shorten the activation
            // delay or extend the TTL, and ordinary network latency cannot make
            // an otherwise valid proposal fail by a few microseconds.
            let effective_activation =
                std::cmp::max(activation_not_before.clone(), minimum_activation);
            let effective_expiry = std::cmp::min(expires_at.clone(), maximum_expiry);
            if effective_expiry <= effective_activation {
                return Err(Error::VerificationFailed(
                    "proposal has no executable interval after server policy bounds".to_string(),
                ));
            }
            let mut approvals = BTreeSet::new();
            if policy.proposer_counts_as_approval {
                approvals.insert(event.envelope.actor.clone());
            }
            projection.proposals.insert(
                *proposal_id,
                CredentialGovernanceProposal {
                    proposal_id: *proposal_id,
                    proposer: event.envelope.actor.clone(),
                    base_registry_head: *base_registry_head,
                    action: action.clone(),
                    action_hash: action.action_hash()?,
                    risk_tier,
                    required_approvals: approval_rule.approval_threshold,
                    required_distinct_organizations: approval_rule.minimum_distinct_organizations,
                    opened_at: recorded.received_at.clone(),
                    activation_not_before: effective_activation,
                    expires_at: effective_expiry,
                    reason: reason.clone(),
                    approvals,
                    status: CredentialGovernanceProposalStatus::Pending,
                    cancelled_by: None,
                    executed_at: None,
                },
            );
        }
        CredentialGovernancePayload::ProposalApproved { proposal_id } => {
            let bound_head = projection
                .proposals
                .get(proposal_id)
                .ok_or_else(|| {
                    Error::NotFound("credential governance proposal not found".to_string())
                })?
                .base_registry_head;
            let registry_at_approval = registry.projection_at(&recorded.received_at).await?;
            if registry_at_approval.head_hash != Some(bound_head) {
                return Err(Error::VerificationFailed(
                    "stale credential governance proposal cannot receive new approvals".to_string(),
                ));
            }
            let proposal = projection.proposals.get_mut(proposal_id).ok_or_else(|| {
                Error::NotFound("credential governance proposal not found".to_string())
            })?;
            if proposal.status != CredentialGovernanceProposalStatus::Pending {
                return Err(Error::Validation(
                    "only pending credential governance proposals can be approved".to_string(),
                ));
            }
            if recorded.received_at > proposal.expires_at {
                return Err(Error::VerificationFailed(
                    "expired credential governance proposal cannot be approved".to_string(),
                ));
            }
            if !proposal.approvals.insert(event.envelope.actor.clone()) {
                return Err(Error::Validation(
                    "registry administrator already approved this proposal".to_string(),
                ));
            }
        }
        CredentialGovernancePayload::ProposalCancelled { proposal_id, .. } => {
            let policy = projection.policy.as_ref().ok_or_else(|| {
                Error::VerificationFailed("credential governance policy is absent".to_string())
            })?;
            let proposal = projection.proposals.get_mut(proposal_id).ok_or_else(|| {
                Error::NotFound("credential governance proposal not found".to_string())
            })?;
            if proposal.status != CredentialGovernanceProposalStatus::Pending {
                return Err(Error::Validation(
                    "only pending credential governance proposals can be cancelled".to_string(),
                ));
            }
            if !policy.emergency_cancellation_enabled && event.envelope.actor != proposal.proposer {
                return Err(Error::VerificationFailed(
                    "emergency cancellation is disabled and only the proposer may cancel"
                        .to_string(),
                ));
            }
            proposal.status = CredentialGovernanceProposalStatus::Cancelled;
            proposal.cancelled_by = Some(event.envelope.actor.clone());
        }
        CredentialGovernancePayload::ProposalExecuted { proposal_id } => {
            let proposal = projection
                .proposals
                .get(proposal_id)
                .cloned()
                .ok_or_else(|| {
                    Error::NotFound("credential governance proposal not found".to_string())
                })?;
            let registry_at_execution = registry.projection_at(&recorded.received_at).await?;
            let (active_approval_count, active_organization_count) =
                proposal_approval_counts(&proposal, &registry_at_execution, &recorded.received_at);
            if proposal.status != CredentialGovernanceProposalStatus::Pending
                || recorded.received_at < proposal.activation_not_before
                || recorded.received_at > proposal.expires_at
                || active_approval_count < usize::from(proposal.required_approvals)
                || active_organization_count < usize::from(proposal.required_distinct_organizations)
            {
                return Err(Error::VerificationFailed(
                    "credential governance proposal is not executable with the currently active approvals"
                        .to_string(),
                ));
            }
            match &proposal.action {
                CredentialGovernanceAction::AppendCredentialEvent { event: proposed } => {
                    let registry_events = registry.events().await;
                    let committed_position = registry_events
                        .iter()
                        .position(|candidate| {
                            candidate.event.envelope.event_id == proposed.envelope.event_id
                        })
                        .ok_or_else(|| {
                            Error::VerificationFailed(
                                "executed credential proposal is absent from the credential registry"
                                    .to_string(),
                            )
                        })?;
                    let committed = &registry_events[committed_position];
                    if committed.event.event_hash()? != proposed.event_hash()?
                        || committed_position as u64 != proposed.envelope.sequence
                        || committed.event.envelope.previous_hash
                            != Some(proposal.base_registry_head)
                        || committed.received_at < recorded.received_at
                    {
                        return Err(Error::VerificationFailed(
                            "executed credential proposal does not match the temporally bound registry successor"
                                .to_string(),
                        ));
                    }
                }
                CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { key } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    if projection
                        .acceptance_service_keys
                        .contains_key(&key.public_key)
                    {
                        return Err(Error::Validation(
                            "acceptance service key is already governed".to_string(),
                        ));
                    }
                    if registry_at_execution.actors.values().any(|profile| {
                        profile
                            .authorized_keys
                            .iter()
                            .any(|actor_key| actor_key.public_key == key.public_key)
                    }) {
                        return Err(Error::VerificationFailed(
                            "acceptance service key cannot also be a scientific actor key"
                                .to_string(),
                        ));
                    }
                    let mut effective_key = key.clone();
                    effective_key.valid_from = std::cmp::max(
                        effective_key.valid_from.clone(),
                        recorded.received_at.clone(),
                    );
                    effective_key.validate()?;
                    projection
                        .acceptance_service_keys
                        .insert(effective_key.public_key, effective_key);
                }
                CredentialGovernanceAction::RevokeAcceptanceServiceKey {
                    public_key,
                    revoked_at,
                    ..
                } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    let existing = projection
                        .acceptance_service_keys
                        .get(public_key)
                        .cloned()
                        .ok_or_else(|| {
                            Error::NotFound("governed acceptance service key not found".to_string())
                        })?;
                    let effective_revoked_at =
                        std::cmp::max(revoked_at.clone(), recorded.received_at.clone());
                    if effective_revoked_at < existing.valid_from.clone()
                        || existing
                            .revoked_at
                            .as_ref()
                            .is_some_and(|current| current <= &effective_revoked_at)
                    {
                        return Err(Error::Validation(
                            "acceptance service key revocation time is invalid".to_string(),
                        ));
                    }
                    let another_key_survives = projection.acceptance_service_keys.iter().any(
                        |(candidate_key, candidate)| {
                            candidate_key != public_key
                                && candidate.is_active_at(&effective_revoked_at)
                        },
                    );
                    if !another_key_survives {
                        return Err(Error::VerificationFailed(
                            "credential governance cannot schedule revocation of its final active acceptance service key"
                                .to_string(),
                        ));
                    }
                    projection
                        .acceptance_service_keys
                        .get_mut(public_key)
                        .expect("acceptance key existence checked above")
                        .revoked_at = Some(effective_revoked_at);
                }
                CredentialGovernanceAction::UpdateGovernancePolicy { policy: new_policy } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    if registry_at_execution.active_admin_count(&recorded.received_at)
                        < usize::from(new_policy.approval_threshold)
                    {
                        return Err(Error::VerificationFailed(
                            "updated governance threshold exceeds active registry administrators"
                                .to_string(),
                        ));
                    }
                    projection.policy = Some(new_policy.clone());
                }
                CredentialGovernanceAction::UpdateGovernanceRiskPolicy { policy: new_policy } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    new_policy.validate()?;
                    let base_policy = projection.policy.as_ref().ok_or_else(|| {
                        Error::VerificationFailed(
                            "credential governance policy is absent".to_string(),
                        )
                    })?;
                    if new_policy.critical.activation_delay_seconds
                        >= base_policy.proposal_ttl_seconds
                    {
                        return Err(Error::Validation(
                            "critical governance delay must be shorter than the proposal TTL"
                                .to_string(),
                        ));
                    }
                    let active_admins =
                        registry_at_execution.active_admin_count(&recorded.received_at);
                    let active_admin_organizations = registry_at_execution
                        .actors
                        .values()
                        .filter(|profile| {
                            profile.has_role(ScientificRole::RegistryAdmin)
                                && profile
                                    .authorized_keys
                                    .iter()
                                    .any(|key| key.is_active_at(&recorded.received_at))
                        })
                        .filter_map(|profile| profile.organizations.iter().next().cloned())
                        .collect::<BTreeSet<_>>()
                        .len();
                    if active_admins < usize::from(new_policy.critical.approval_threshold)
                        || active_admin_organizations
                            < usize::from(new_policy.critical.minimum_distinct_organizations)
                    {
                        return Err(Error::VerificationFailed(
                            "risk policy exceeds the active administrator or organization set"
                                .to_string(),
                        ));
                    }
                    projection.risk_policy = Some(new_policy.clone());
                }
                CredentialGovernanceAction::AuthorizeTransparencyWitness { witness } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    if projection
                        .transparency_witnesses
                        .contains_key(&witness.actor)
                    {
                        return Err(Error::Validation(
                            "transparency witness actor is already governed".to_string(),
                        ));
                    }
                    if projection
                        .transparency_witnesses
                        .values()
                        .any(|existing| existing.public_key == witness.public_key)
                    {
                        return Err(Error::VerificationFailed(
                            "transparency witness key has already been assigned".to_string(),
                        ));
                    }
                    if projection
                        .acceptance_service_keys
                        .contains_key(&witness.public_key)
                        || registry_at_execution.actors.values().any(|profile| {
                            profile
                                .authorized_keys
                                .iter()
                                .any(|key| key.public_key == witness.public_key)
                        })
                    {
                        return Err(Error::VerificationFailed(
                            "transparency witness key must be distinct from actor and service keys"
                                .to_string(),
                        ));
                    }
                    let mut effective = witness.clone();
                    effective.valid_from =
                        std::cmp::max(effective.valid_from.clone(), recorded.received_at.clone());
                    effective.validate()?;
                    projection
                        .transparency_witnesses
                        .insert(effective.actor.clone(), effective);
                }
                CredentialGovernanceAction::RevokeTransparencyWitness {
                    actor, revoked_at, ..
                } => {
                    if registry_at_execution.head_hash != Some(proposal.base_registry_head) {
                        return Err(Error::VerificationFailed(
                            "credential governance proposal was stale at execution time"
                                .to_string(),
                        ));
                    }
                    let witness = projection
                        .transparency_witnesses
                        .get_mut(actor)
                        .ok_or_else(|| {
                            Error::NotFound("governed transparency witness not found".to_string())
                        })?;
                    let effective_revoked_at =
                        std::cmp::max(revoked_at.clone(), recorded.received_at.clone());
                    if effective_revoked_at < witness.valid_from.clone()
                        || witness
                            .revoked_at
                            .as_ref()
                            .is_some_and(|current| current <= &effective_revoked_at)
                    {
                        return Err(Error::Validation(
                            "transparency witness revocation time is invalid".to_string(),
                        ));
                    }
                    witness.revoked_at = Some(effective_revoked_at);
                }
                CredentialGovernanceAction::RecordTransparencyWitnessCompromise {
                    actor,
                    compromised_from,
                    restored_at,
                    reason,
                } => {
                    if !projection.transparency_witnesses.contains_key(actor) {
                        return Err(Error::NotFound(
                            "governed transparency witness not found".to_string(),
                        ));
                    }
                    let interval = TransparencyWitnessCompromiseInterval {
                        compromised_from: compromised_from.clone(),
                        restored_at: restored_at.clone(),
                        reason: reason.clone(),
                    };
                    interval.validate_at(&recorded.received_at)?;
                    let intervals = projection
                        .transparency_witness_compromises
                        .entry(actor.clone())
                        .or_default();
                    if intervals
                        .iter()
                        .any(|existing| intervals_overlap(existing, &interval))
                    {
                        return Err(Error::Validation(
                            "transparency witness compromise intervals cannot overlap".to_string(),
                        ));
                    }
                    intervals.push(interval);
                    intervals.sort_by_key(|interval| interval.compromised_from);
                }
                CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { .. } => {
                    // Execution authorizes a later database-epoch certificate.
                    // The SQL epoch store verifies the executed proposal and
                    // commits the actual state commitment under an exclusive
                    // authority barrier.
                }
            }
            if projection.continuing_acceptance_key_count(&recorded.received_at) == 0 {
                return Err(Error::VerificationFailed(
                    "credential governance transition would remove the final non-expiring acceptance-service recovery key"
                        .to_string(),
                ));
            }
            let proposal = projection
                .proposals
                .get_mut(proposal_id)
                .expect("proposal existence checked above");
            proposal.status = CredentialGovernanceProposalStatus::Executed;
            proposal.executed_at = Some(recorded.received_at.clone());
        }
        CredentialGovernancePayload::TransparencyCheckpointPublished { checkpoint } => {
            if checkpoint.governance_event_count != event.envelope.sequence
                || checkpoint.governance_head
                    != event.envelope.previous_hash.ok_or_else(|| {
                        Error::VerificationFailed(
                            "checkpoint requires a previous governance head".to_string(),
                        )
                    })?
                || checkpoint.issued_at != event.envelope.occurred_at
            {
                return Err(Error::VerificationFailed(
                    "transparency checkpoint does not bind the signed governance prefix"
                        .to_string(),
                ));
            }
            let registry_events = registry.events().await;
            let checkpoint_registry_len = usize::try_from(checkpoint.registry_event_count)
                .map_err(|_| {
                    Error::VerificationFailed(
                        "transparency checkpoint registry count exceeds this implementation"
                            .to_string(),
                    )
                })?;
            if checkpoint_registry_len == 0 || checkpoint_registry_len > registry_events.len() {
                return Err(Error::VerificationFailed(
                    "transparency checkpoint registry prefix is unavailable".to_string(),
                ));
            }
            let registry_prefix = &registry_events[..checkpoint_registry_len];
            if registry_prefix
                .last()
                .map(RecordedScientificCredentialEvent::record_hash)
                .transpose()?
                != Some(checkpoint.registry_head)
                || registry_prefix
                    .last()
                    .is_some_and(|last| last.received_at > recorded.received_at)
            {
                return Err(Error::VerificationFailed(
                    "transparency checkpoint does not bind the registry prefix known at publication"
                        .to_string(),
                ));
            }
            let registry_hashes = registry_prefix
                .iter()
                .map(RecordedScientificCredentialEvent::record_hash)
                .collect::<Result<Vec<_>>>()?;
            let governance_hashes = prior_events[..event.envelope.sequence as usize]
                .iter()
                .map(RecordedCredentialGovernanceEvent::record_hash)
                .collect::<Result<Vec<_>>>()?;
            if merkle_root(&registry_hashes)? != checkpoint.registry_merkle_root
                || merkle_root(&governance_hashes)? != checkpoint.governance_merkle_root
            {
                return Err(Error::VerificationFailed(
                    "transparency checkpoint Merkle root does not match local history".to_string(),
                ));
            }
            let checkpoint_hash = checkpoint.checkpoint_hash();
            if projection
                .checkpoints
                .iter()
                .any(|existing| existing.checkpoint_hash() == checkpoint_hash)
            {
                return Err(Error::Validation(
                    "transparency checkpoint hash has already been published".to_string(),
                ));
            }
            projection.checkpoints.push(checkpoint.clone());
        }
        CredentialGovernancePayload::TransparencyCheckpointWitnessed { witness } => {
            witness.verify()?;
            let checkpoint = projection
                .checkpoints
                .iter()
                .find(|checkpoint| checkpoint.checkpoint_hash() == witness.checkpoint_hash)
                .ok_or_else(|| {
                    Error::NotFound(
                        "witnessed transparency checkpoint is not present in governance history"
                            .to_string(),
                    )
                })?;
            if witness.witnessed_at < checkpoint.issued_at
                || witness.witnessed_at
                    > recorded.received_at.clone()
                        + Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS)
            {
                return Err(Error::VerificationFailed(
                    "transparency witness timestamp is outside the accepted checkpoint interval"
                        .to_string(),
                ));
            }
            let authorized = projection
                .active_transparency_witness(
                    &witness.actor,
                    &witness.signer_public_key,
                    &recorded.received_at,
                )
                .ok_or_else(|| {
                    Error::VerificationFailed(
                        "transparency witness was not governed and active at receipt time"
                            .to_string(),
                    )
                })?;
            if authorized.organization != witness.organization {
                return Err(Error::VerificationFailed(
                    "transparency witness organization does not match governed authority"
                        .to_string(),
                ));
            }
            let witnesses = projection
                .checkpoint_witnesses
                .entry(witness.checkpoint_hash)
                .or_default();
            if witnesses
                .insert(witness.actor.clone(), witness.clone())
                .is_some()
            {
                return Err(Error::Validation(
                    "transparency witness actor has already witnessed this checkpoint".to_string(),
                ));
            }
        }
    }
    Ok(())
}

fn merkle_root(leaves: &[ContentHash]) -> Result<ContentHash> {
    if leaves.is_empty() {
        return Err(Error::Validation(
            "cannot calculate a transparency root for empty history".to_string(),
        ));
    }
    let mut level = leaves.to_vec();
    while level.len() > 1 {
        let mut next = Vec::with_capacity(level.len().div_ceil(2));
        for pair in level.chunks(2) {
            let left = pair[0];
            let right = *pair.get(1).unwrap_or(&left);
            let mut bytes = Vec::new();
            bytes.extend_from_slice(b"MYCELIX-DESCI-TRANSPARENCY-MERKLE-NODE\0");
            bytes.extend_from_slice(&left.0);
            bytes.extend_from_slice(&right.0);
            next.push(ContentHash::digest(&bytes));
        }
        level = next;
    }
    Ok(level[0])
}

fn intervals_overlap(
    left: &TransparencyWitnessCompromiseInterval,
    right: &TransparencyWitnessCompromiseInterval,
) -> bool {
    let left_ends_before_right = left
        .restored_at
        .as_ref()
        .is_some_and(|end| end <= &right.compromised_from);
    let right_ends_before_left = right
        .restored_at
        .as_ref()
        .is_some_and(|end| end <= &left.compromised_from);
    !(left_ends_before_right || right_ends_before_left)
}

fn validate_reason(reason: &str) -> Result<()> {
    if reason.trim().is_empty() || reason.len() > MAX_REASON_BYTES {
        return Err(Error::Validation(
            "credential governance reason is empty or exceeds its size limit".to_string(),
        ));
    }
    Ok(())
}

fn read_json_file<T: for<'de> Deserialize<'de>>(
    path: &Path,
    maximum_bytes: u64,
    label: &str,
) -> Result<Option<T>> {
    if !path.exists() {
        return Ok(None);
    }
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(Error::Storage(format!(
            "{label} must be a regular non-symlink file: {}",
            path.display()
        )));
    }
    if metadata.len() > maximum_bytes {
        return Err(Error::Storage(format!(
            "{label} exceeds {maximum_bytes} bytes"
        )));
    }
    let bytes = fs::read(path)?;
    if bytes.is_empty() {
        Ok(None)
    } else {
        Ok(Some(serde_json::from_slice(&bytes)?))
    }
}

fn persist_json<T: Serialize>(path: &Path, value: &T, maximum_bytes: u64) -> Result<()> {
    let bytes = serde_json::to_vec_pretty(value)?;
    if bytes.len() as u64 > maximum_bytes {
        return Err(Error::Storage(format!(
            "credential governance persistence exceeds {maximum_bytes} bytes"
        )));
    }
    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    fs::create_dir_all(parent)?;
    let temporary = parent.join(format!(
        ".{}.{}.tmp",
        path.file_name()
            .and_then(|name| name.to_str())
            .unwrap_or("credential-governance"),
        Uuid::new_v4()
    ));
    let result = (|| -> Result<()> {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)?;
        file.write_all(&bytes)?;
        file.sync_all()?;
        fs::rename(&temporary, path)?;
        File::open(parent)?.sync_all()?;
        Ok(())
    })();
    if result.is_err() {
        let _ = fs::remove_file(&temporary);
    }
    result
}

fn push_payload(bytes: &mut Vec<u8>, payload: &CredentialGovernancePayload) -> Result<()> {
    bytes.push(payload.code());
    match payload {
        CredentialGovernancePayload::PolicyInitialized {
            policy,
            initial_acceptance_service_keys,
        } => {
            push_u16(bytes, policy.approval_threshold);
            push_u64(bytes, policy.activation_delay_seconds);
            push_u64(bytes, policy.proposal_ttl_seconds);
            bytes.push(u8::from(policy.proposer_counts_as_approval));
            bytes.push(u8::from(policy.emergency_cancellation_enabled));
            push_len(bytes, initial_acceptance_service_keys.len())?;
            for key in initial_acceptance_service_keys {
                push_service_key(bytes, key);
            }
        }
        CredentialGovernancePayload::ProposalOpened {
            proposal_id,
            base_registry_head,
            action,
            activation_not_before,
            expires_at,
            reason,
        } => {
            push_uuid(bytes, proposal_id.0);
            push_hash(bytes, *base_registry_head);
            push_action(bytes, action)?;
            push_datetime(bytes, activation_not_before);
            push_datetime(bytes, expires_at);
            push_string(bytes, reason)?;
        }
        CredentialGovernancePayload::ProposalApproved { proposal_id }
        | CredentialGovernancePayload::ProposalExecuted { proposal_id } => {
            push_uuid(bytes, proposal_id.0);
        }
        CredentialGovernancePayload::ProposalCancelled {
            proposal_id,
            reason,
        } => {
            push_uuid(bytes, proposal_id.0);
            push_string(bytes, reason)?;
        }
        CredentialGovernancePayload::TransparencyCheckpointPublished { checkpoint } => {
            push_u64(bytes, checkpoint.registry_event_count);
            push_hash(bytes, checkpoint.registry_head);
            push_hash(bytes, checkpoint.registry_merkle_root);
            push_u64(bytes, checkpoint.governance_event_count);
            push_hash(bytes, checkpoint.governance_head);
            push_hash(bytes, checkpoint.governance_merkle_root);
            push_datetime(bytes, &checkpoint.issued_at);
        }
        CredentialGovernancePayload::TransparencyCheckpointWitnessed { witness } => {
            push_hash(bytes, witness.checkpoint_hash);
            push_string(bytes, witness.actor.as_str())?;
            push_string(bytes, witness.organization.as_str())?;
            push_datetime(bytes, &witness.witnessed_at);
            bytes.extend_from_slice(&witness.signer_public_key);
            push_len(bytes, witness.signature.len())?;
            bytes.extend_from_slice(&witness.signature);
        }
    }
    Ok(())
}

fn push_action(bytes: &mut Vec<u8>, action: &CredentialGovernanceAction) -> Result<()> {
    bytes.push(action.code());
    match action {
        CredentialGovernanceAction::AppendCredentialEvent { event } => {
            bytes.extend_from_slice(&event.event_hash()?.0);
        }
        CredentialGovernanceAction::AuthorizeAcceptanceServiceKey { key } => {
            push_service_key(bytes, key);
        }
        CredentialGovernanceAction::RevokeAcceptanceServiceKey {
            public_key,
            revoked_at,
            reason,
        } => {
            bytes.extend_from_slice(public_key);
            push_datetime(bytes, revoked_at);
            push_string(bytes, reason)?;
        }
        CredentialGovernanceAction::UpdateGovernancePolicy { policy } => {
            push_u16(bytes, policy.approval_threshold);
            push_u64(bytes, policy.activation_delay_seconds);
            push_u64(bytes, policy.proposal_ttl_seconds);
            bytes.push(u8::from(policy.proposer_counts_as_approval));
            bytes.push(u8::from(policy.emergency_cancellation_enabled));
        }
        CredentialGovernanceAction::UpdateGovernanceRiskPolicy { policy } => {
            push_risk_policy(bytes, policy);
        }
        CredentialGovernanceAction::AuthorizeTransparencyWitness { witness } => {
            push_transparency_witness_authority(bytes, witness)?;
        }
        CredentialGovernanceAction::RevokeTransparencyWitness {
            actor,
            revoked_at,
            reason,
        } => {
            push_string(bytes, actor.as_str())?;
            push_datetime(bytes, revoked_at);
            push_string(bytes, reason)?;
        }
        CredentialGovernanceAction::RecordTransparencyWitnessCompromise {
            actor,
            compromised_from,
            restored_at,
            reason,
        } => {
            push_string(bytes, actor.as_str())?;
            push_datetime(bytes, compromised_from);
            push_option_datetime(bytes, restored_at.as_ref());
            push_string(bytes, reason)?;
        }
        CredentialGovernanceAction::AuthorizeDatabaseEpochPromotion { intent } => {
            bytes.extend_from_slice(&intent.intent_hash()?.0);
        }
    }
    Ok(())
}

fn push_risk_policy(bytes: &mut Vec<u8>, policy: &CredentialGovernanceRiskPolicy) {
    for rule in [&policy.routine, &policy.sensitive, &policy.critical] {
        push_u16(bytes, rule.approval_threshold);
        push_u16(bytes, rule.minimum_distinct_organizations);
        push_u64(bytes, rule.activation_delay_seconds);
    }
}

fn push_transparency_witness_authority(
    bytes: &mut Vec<u8>,
    witness: &AuthorizedCredentialTransparencyWitness,
) -> Result<()> {
    push_string(bytes, witness.actor.as_str())?;
    push_string(bytes, witness.organization.as_str())?;
    bytes.extend_from_slice(&witness.public_key);
    push_datetime(bytes, &witness.valid_from);
    push_option_datetime(bytes, witness.valid_until.as_ref());
    push_option_datetime(bytes, witness.revoked_at.as_ref());
    Ok(())
}

fn push_service_key(bytes: &mut Vec<u8>, key: &AuthorizedCredentialAcceptanceKey) {
    bytes.extend_from_slice(&key.public_key);
    push_datetime(bytes, &key.valid_from);
    push_option_datetime(bytes, key.valid_until.as_ref());
    push_option_datetime(bytes, key.revoked_at.as_ref());
}

fn push_len(bytes: &mut Vec<u8>, len: usize) -> Result<()> {
    let len = u32::try_from(len)
        .map_err(|_| Error::Validation("canonical field exceeds u32 length".to_string()))?;
    bytes.extend_from_slice(&len.to_be_bytes());
    Ok(())
}

fn push_string(bytes: &mut Vec<u8>, value: &str) -> Result<()> {
    push_len(bytes, value.len())?;
    bytes.extend_from_slice(value.as_bytes());
    Ok(())
}

fn push_u16(bytes: &mut Vec<u8>, value: u16) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u64(bytes: &mut Vec<u8>, value: u64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_uuid(bytes: &mut Vec<u8>, value: Uuid) {
    bytes.extend_from_slice(value.as_bytes());
}

fn push_hash(bytes: &mut Vec<u8>, value: ContentHash) {
    bytes.extend_from_slice(&value.0);
}

fn push_option_hash(bytes: &mut Vec<u8>, value: Option<ContentHash>) {
    match value {
        Some(hash) => {
            bytes.push(1);
            push_hash(bytes, hash);
        }
        None => bytes.push(0),
    }
}

fn push_datetime(bytes: &mut Vec<u8>, value: &DateTime<Utc>) {
    bytes.extend_from_slice(&value.timestamp().to_be_bytes());
    bytes.extend_from_slice(&value.timestamp_subsec_nanos().to_be_bytes());
}

fn push_option_datetime(bytes: &mut Vec<u8>, value: Option<&DateTime<Utc>>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_datetime(bytes, value);
        }
        None => bytes.push(0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scientific_credentials::{
        ScientificCredentialEnvelope, ScientificCredentialPayload,
    };
    use crate::scientific_governance::{AuthorizedActorKey, ResolvedScientificActor};

    fn key(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }

    fn actor(value: &str) -> ActorId {
        ActorId::new(value).unwrap()
    }

    fn admin_profile(
        id: ActorId,
        signing_key: &SigningKey,
        now: &DateTime<Utc>,
    ) -> ResolvedScientificActor {
        let organization = if id.as_str().contains("admin-one") {
            OrganizationId::new("org:admin-one").unwrap()
        } else {
            OrganizationId::new("org:admin-two").unwrap()
        };
        ResolvedScientificActor {
            actor: id,
            authorized_keys: vec![AuthorizedActorKey {
                public_key: signing_key.verifying_key().to_bytes(),
                valid_from: now.clone() - Duration::seconds(1),
                valid_until: None,
                revoked_at: None,
            }],
            organizations: BTreeSet::from([organization]),
            roles: BTreeSet::from([ScientificRole::RegistryAdmin]),
            authority_revision: None,
        }
    }

    async fn registry_with_two_admins() -> (
        Arc<ScientificCredentialRegistry>,
        SigningKey,
        SigningKey,
        Arc<SigningKey>,
    ) {
        let admin_one = key(1);
        let admin_two = key(2);
        let service = Arc::new(key(9));
        let registry = Arc::new(ScientificCredentialRegistry::memory(
            BTreeSet::from([admin_one.verifying_key().to_bytes()]),
            BTreeSet::from([service.verifying_key().to_bytes()]),
            Some(service.clone()),
        ));
        let now = Utc::now();
        let admin_one_id = actor("did:key:admin-one");
        let genesis = SignedScientificCredentialEvent::sign(
            ScientificCredentialEnvelope::genesis(
                admin_one_id.clone(),
                now.clone(),
                admin_profile(admin_one_id.clone(), &admin_one, &now),
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        registry.append(0, genesis).await.unwrap();
        let previous = registry.events().await.pop().unwrap();
        let register = SignedScientificCredentialEvent::sign(
            ScientificCredentialEnvelope::successor(
                &previous,
                admin_one_id,
                now.clone(),
                "register-second-admin",
                ScientificCredentialPayload::ActorRegistered {
                    profile: admin_profile(actor("did:key:admin-two"), &admin_two, &now),
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        registry.append(1, register).await.unwrap();
        (registry, admin_one, admin_two, service)
    }

    #[tokio::test]
    async fn governance_requires_two_administrators() {
        let (registry, admin_one, _, service) = registry_with_two_admins().await;
        let governance = ScientificCredentialGovernance::memory(
            registry.clone(),
            BTreeSet::from([service.verifying_key().to_bytes()]),
            Some(service.clone()),
        )
        .await
        .unwrap();
        let now = Utc::now();
        let policy_event = SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                0,
                None,
                actor("did:key:admin-one"),
                now.clone(),
                "initialize-threshold-governance",
                CredentialGovernancePayload::PolicyInitialized {
                    policy: CredentialGovernancePolicy {
                        approval_threshold: 2,
                        activation_delay_seconds: 1,
                        proposal_ttl_seconds: 60,
                        proposer_counts_as_approval: true,
                        emergency_cancellation_enabled: true,
                    },
                    initial_acceptance_service_keys: vec![AuthorizedCredentialAcceptanceKey {
                        public_key: service.verifying_key().to_bytes(),
                        valid_from: now - Duration::seconds(1),
                        valid_until: None,
                        revoked_at: None,
                    }],
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        governance.append(0, policy_event).await.unwrap();
        assert!(governance.summary().await.initialized);
        assert!(registry.threshold_governance_required());
    }

    #[tokio::test]
    async fn identical_governance_retry_returns_the_original_acceptance_receipt() {
        let (registry, admin_one, _, service) = registry_with_two_admins().await;
        let governance = ScientificCredentialGovernance::memory(
            registry,
            BTreeSet::from([service.verifying_key().to_bytes()]),
            Some(service.clone()),
        )
        .await
        .unwrap();
        let now = Utc::now();
        let event = SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                0,
                None,
                actor("did:key:admin-one"),
                now.clone(),
                "retry-threshold-governance-genesis",
                CredentialGovernancePayload::PolicyInitialized {
                    policy: CredentialGovernancePolicy {
                        approval_threshold: 2,
                        activation_delay_seconds: 1,
                        proposal_ttl_seconds: 60,
                        proposer_counts_as_approval: true,
                        emergency_cancellation_enabled: true,
                    },
                    initial_acceptance_service_keys: vec![AuthorizedCredentialAcceptanceKey {
                        public_key: service.verifying_key().to_bytes(),
                        valid_from: now - Duration::seconds(1),
                        valid_until: None,
                        revoked_at: None,
                    }],
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        let original = governance.append(0, event.clone()).await.unwrap();
        let retried = governance.append(1, event).await.unwrap();
        assert_eq!(retried, original);
        assert_eq!(governance.events().await.len(), 1);
    }

    #[tokio::test]
    async fn governance_requires_non_expiring_acceptance_recovery_key() {
        let (registry, admin_one, _, service) = registry_with_two_admins().await;
        let governance = ScientificCredentialGovernance::memory(
            registry,
            BTreeSet::from([service.verifying_key().to_bytes()]),
            Some(service.clone()),
        )
        .await
        .unwrap();
        let now = Utc::now();
        let event = SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                0,
                None,
                actor("did:key:admin-one"),
                now.clone(),
                "expiring-only-governance-key",
                CredentialGovernancePayload::PolicyInitialized {
                    policy: CredentialGovernancePolicy {
                        approval_threshold: 2,
                        activation_delay_seconds: 1,
                        proposal_ttl_seconds: 60,
                        proposer_counts_as_approval: true,
                        emergency_cancellation_enabled: true,
                    },
                    initial_acceptance_service_keys: vec![AuthorizedCredentialAcceptanceKey {
                        public_key: service.verifying_key().to_bytes(),
                        valid_from: now.clone() - Duration::seconds(1),
                        valid_until: Some(now + Duration::hours(1)),
                        revoked_at: None,
                    }],
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        assert!(governance.append(0, event).await.is_err());
    }

    #[test]
    fn service_key_validity_is_fail_closed() {
        let now = Utc::now();
        let key = AuthorizedCredentialAcceptanceKey {
            public_key: [4; 32],
            valid_from: now.clone(),
            valid_until: Some(now.clone()),
            revoked_at: None,
        };
        assert!(key.validate().is_err());
    }

    #[test]
    fn action_hash_binds_the_proposed_event() {
        let signing_key = key(3);
        let now = Utc::now();
        let envelope = ScientificCredentialEnvelope::genesis(
            actor("did:key:temporary"),
            now.clone(),
            admin_profile(actor("did:key:temporary"), &signing_key, &now),
        )
        .unwrap();
        let event = SignedScientificCredentialEvent::sign(envelope, &signing_key).unwrap();
        let action = CredentialGovernanceAction::AppendCredentialEvent { event };
        assert_ne!(action.action_hash().unwrap(), ContentHash([0; 32]));
    }

    async fn initialized_governance() -> (
        Arc<ScientificCredentialRegistry>,
        ScientificCredentialGovernance,
        SigningKey,
        SigningKey,
        Arc<SigningKey>,
    ) {
        let (registry, admin_one, admin_two, service) = registry_with_two_admins().await;
        let governance = ScientificCredentialGovernance::memory(
            registry.clone(),
            BTreeSet::from([service.verifying_key().to_bytes()]),
            Some(service.clone()),
        )
        .await
        .unwrap();
        let now = Utc::now();
        let policy_event = SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                0,
                None,
                actor("did:key:admin-one"),
                now.clone(),
                "initialize-threshold-governance",
                CredentialGovernancePayload::PolicyInitialized {
                    policy: CredentialGovernancePolicy {
                        approval_threshold: 2,
                        activation_delay_seconds: 1,
                        proposal_ttl_seconds: 120,
                        proposer_counts_as_approval: true,
                        emergency_cancellation_enabled: true,
                    },
                    initial_acceptance_service_keys: vec![AuthorizedCredentialAcceptanceKey {
                        public_key: service.verifying_key().to_bytes(),
                        valid_from: now - Duration::seconds(1),
                        valid_until: None,
                        revoked_at: None,
                    }],
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        governance.append(0, policy_event).await.unwrap();
        (registry, governance, admin_one, admin_two, service)
    }

    fn signed_governance_event(
        governance_projection: &CredentialGovernanceProjection,
        sequence: u64,
        actor_id: ActorId,
        idempotency_key: &str,
        payload: CredentialGovernancePayload,
        signing_key: &SigningKey,
    ) -> SignedCredentialGovernanceEvent {
        SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                sequence,
                governance_projection.head_hash,
                actor_id,
                Utc::now(),
                idempotency_key,
                payload,
            )
            .unwrap(),
            signing_key,
        )
        .unwrap()
    }

    #[tokio::test]
    async fn unique_admin_approvals_and_delay_gate_execution() {
        let (registry, governance, admin_one, admin_two, _) = initialized_governance().await;
        let registry_events = registry.events().await;
        let previous = registry_events.last().unwrap();
        let proposed = SignedScientificCredentialEvent::sign(
            ScientificCredentialEnvelope::successor(
                previous,
                actor("did:key:admin-one"),
                Utc::now(),
                "governed-register-reviewer",
                ScientificCredentialPayload::ActorRegistered {
                    profile: ResolvedScientificActor {
                        actor: actor("did:key:reviewer"),
                        authorized_keys: vec![AuthorizedActorKey {
                            public_key: key(5).verifying_key().to_bytes(),
                            valid_from: Utc::now() - Duration::seconds(1),
                            valid_until: None,
                            revoked_at: None,
                        }],
                        organizations: BTreeSet::new(),
                        roles: BTreeSet::from([ScientificRole::Reviewer]),
                        authority_revision: None,
                    },
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        let proposal_id = CredentialGovernanceProposalId::new();
        let now = Utc::now();
        let open = signed_governance_event(
            &governance.projection().await,
            1,
            actor("did:key:admin-one"),
            "open-register-reviewer",
            CredentialGovernancePayload::ProposalOpened {
                proposal_id,
                base_registry_head: previous.record_hash().unwrap(),
                action: CredentialGovernanceAction::AppendCredentialEvent { event: proposed },
                // Wide enough that setup (several signed+verified async
                // appends) can't eat the whole window under heavy
                // concurrent build load before the early-execute assertion
                // below runs -- measured 3.7s elapsed on a loaded box
                // against the previous 2s margin, which made this test
                // genuinely flaky (not a bug in the delay-gate itself).
                activation_not_before: now.clone() + Duration::seconds(20),
                expires_at: now + Duration::seconds(120),
                reason: "add an independent reviewer through threshold governance".to_string(),
            },
            &admin_one,
        );
        governance.append(1, open).await.unwrap();
        let duplicate = signed_governance_event(
            &governance.projection().await,
            2,
            actor("did:key:admin-one"),
            "duplicate-proposer-approval",
            CredentialGovernancePayload::ProposalApproved { proposal_id },
            &admin_one,
        );
        assert!(governance.append(2, duplicate).await.is_err());
        let approve = signed_governance_event(
            &governance.projection().await,
            2,
            actor("did:key:admin-two"),
            "second-admin-approval",
            CredentialGovernancePayload::ProposalApproved { proposal_id },
            &admin_two,
        );
        governance.append(2, approve).await.unwrap();
        let early_execute = signed_governance_event(
            &governance.projection().await,
            3,
            actor("did:key:admin-two"),
            "early-execute",
            CredentialGovernancePayload::ProposalExecuted { proposal_id },
            &admin_two,
        );
        assert!(governance.execute(3, early_execute).await.is_err());
        tokio::time::sleep(std::time::Duration::from_millis(20_100)).await;
        let execute = signed_governance_event(
            &governance.projection().await,
            3,
            actor("did:key:admin-two"),
            "execute-register-reviewer",
            CredentialGovernancePayload::ProposalExecuted { proposal_id },
            &admin_two,
        );
        governance.execute(3, execute).await.unwrap();
        assert!(
            registry
                .projection()
                .await
                .resolve(&actor("did:key:reviewer"))
                .is_some()
        );
    }

    #[tokio::test]
    async fn published_checkpoint_survives_later_governance_growth() {
        let (registry, governance, admin_one, _, _) = initialized_governance().await;
        let checkpoint = governance.checkpoint_candidate().await.unwrap();
        let checkpoint_event = SignedCredentialGovernanceEvent::sign(
            CredentialGovernanceEnvelope::new(
                1,
                governance.projection().await.head_hash,
                actor("did:key:admin-one"),
                checkpoint.issued_at.clone(),
                "publish-first-transparency-checkpoint",
                CredentialGovernancePayload::TransparencyCheckpointPublished {
                    checkpoint: checkpoint.clone(),
                },
            )
            .unwrap(),
            &admin_one,
        )
        .unwrap();
        governance.append(1, checkpoint_event).await.unwrap();

        let base_registry_head = registry.projection().await.head_hash.unwrap();
        let now = Utc::now();
        let proposal = signed_governance_event(
            &governance.projection().await,
            2,
            actor("did:key:admin-one"),
            "open-service-key-proposal-after-checkpoint",
            CredentialGovernancePayload::ProposalOpened {
                proposal_id: CredentialGovernanceProposalId::new(),
                base_registry_head,
                action: CredentialGovernanceAction::AuthorizeAcceptanceServiceKey {
                    key: AuthorizedCredentialAcceptanceKey {
                        public_key: key(10).verifying_key().to_bytes(),
                        valid_from: now.clone(),
                        valid_until: None,
                        revoked_at: None,
                    },
                },
                activation_not_before: now.clone() + Duration::seconds(2),
                expires_at: now + Duration::seconds(60),
                reason: "prove historical checkpoints survive later journal growth".to_string(),
            },
            &admin_one,
        );
        governance.append(2, proposal).await.unwrap();
        assert_eq!(governance.projection().await.checkpoints, vec![checkpoint]);
    }

    #[tokio::test]
    async fn parallel_proposal_becomes_stale_after_first_execution() {
        let (registry, governance, admin_one, admin_two, _) = initialized_governance().await;
        let previous = registry.events().await.pop().unwrap();
        let base = previous.record_hash().unwrap();
        let mut proposal_ids = Vec::new();
        for (offset, target) in [(6_u8, "did:key:first"), (7_u8, "did:key:second")] {
            let proposed = SignedScientificCredentialEvent::sign(
                ScientificCredentialEnvelope::successor(
                    &previous,
                    actor("did:key:admin-one"),
                    Utc::now(),
                    format!("register-{target}"),
                    ScientificCredentialPayload::ActorRegistered {
                        profile: ResolvedScientificActor {
                            actor: actor(target),
                            authorized_keys: vec![AuthorizedActorKey {
                                public_key: key(offset).verifying_key().to_bytes(),
                                valid_from: Utc::now() - Duration::seconds(1),
                                valid_until: None,
                                revoked_at: None,
                            }],
                            organizations: BTreeSet::new(),
                            roles: BTreeSet::from([ScientificRole::Contributor]),
                            authority_revision: None,
                        },
                    },
                )
                .unwrap(),
                &admin_one,
            )
            .unwrap();
            let proposal_id = CredentialGovernanceProposalId::new();
            let sequence = governance.summary().await.event_count as u64;
            let now = Utc::now();
            let open = signed_governance_event(
                &governance.projection().await,
                sequence,
                actor("did:key:admin-one"),
                &format!("open-{target}"),
                CredentialGovernancePayload::ProposalOpened {
                    proposal_id,
                    base_registry_head: base,
                    action: CredentialGovernanceAction::AppendCredentialEvent { event: proposed },
                    activation_not_before: now.clone() + Duration::seconds(2),
                    expires_at: now + Duration::seconds(60),
                    reason: format!("parallel proposal for {target}"),
                },
                &admin_one,
            );
            governance.append(sequence, open).await.unwrap();
            let sequence = governance.summary().await.event_count as u64;
            let approve = signed_governance_event(
                &governance.projection().await,
                sequence,
                actor("did:key:admin-two"),
                &format!("approve-{target}"),
                CredentialGovernancePayload::ProposalApproved { proposal_id },
                &admin_two,
            );
            governance.append(sequence, approve).await.unwrap();
            proposal_ids.push(proposal_id);
        }
        tokio::time::sleep(std::time::Duration::from_millis(2100)).await;
        let sequence = governance.summary().await.event_count as u64;
        let execute_first = signed_governance_event(
            &governance.projection().await,
            sequence,
            actor("did:key:admin-two"),
            "execute-first-parallel",
            CredentialGovernancePayload::ProposalExecuted {
                proposal_id: proposal_ids[0],
            },
            &admin_two,
        );
        governance.execute(sequence, execute_first).await.unwrap();
        let sequence = governance.summary().await.event_count as u64;
        let execute_second = signed_governance_event(
            &governance.projection().await,
            sequence,
            actor("did:key:admin-two"),
            "execute-second-parallel",
            CredentialGovernancePayload::ProposalExecuted {
                proposal_id: proposal_ids[1],
            },
            &admin_two,
        );
        assert!(governance.execute(sequence, execute_second).await.is_err());
    }

    #[test]
    fn approval_diversity_counts_organizations_not_accounts() {
        let now = Utc::now();
        let shared = OrganizationId::new("org:shared").unwrap();
        let admin_one = actor("did:key:diversity-one");
        let admin_two = actor("did:key:diversity-two");
        let key_one = key(21);
        let key_two = key(22);
        let mut registry = ScientificCredentialRegistryProjection::default();
        for (id, signing_key) in [(admin_one.clone(), &key_one), (admin_two.clone(), &key_two)] {
            registry.actors.insert(
                id.clone(),
                ResolvedScientificActor {
                    actor: id,
                    authorized_keys: vec![AuthorizedActorKey {
                        public_key: signing_key.verifying_key().to_bytes(),
                        valid_from: now.clone() - Duration::seconds(1),
                        valid_until: None,
                        revoked_at: None,
                    }],
                    organizations: BTreeSet::from([shared.clone()]),
                    roles: BTreeSet::from([ScientificRole::RegistryAdmin]),
                    authority_revision: None,
                },
            );
        }
        let proposal = CredentialGovernanceProposal {
            proposal_id: CredentialGovernanceProposalId::new(),
            proposer: admin_one.clone(),
            base_registry_head: ContentHash([7; 32]),
            action: CredentialGovernanceAction::UpdateGovernancePolicy {
                policy: CredentialGovernancePolicy {
                    approval_threshold: 2,
                    activation_delay_seconds: 60,
                    proposal_ttl_seconds: 3_600,
                    proposer_counts_as_approval: true,
                    emergency_cancellation_enabled: true,
                },
            },
            action_hash: ContentHash([8; 32]),
            risk_tier: CredentialGovernanceRiskTier::Critical,
            required_approvals: 2,
            required_distinct_organizations: 2,
            opened_at: now.clone(),
            activation_not_before: now.clone(),
            expires_at: now.clone() + Duration::hours(1),
            reason: "test organization diversity".to_string(),
            approvals: BTreeSet::from([admin_one, admin_two]),
            status: CredentialGovernanceProposalStatus::Pending,
            cancelled_by: None,
            executed_at: None,
        };
        assert_eq!(proposal_approval_counts(&proposal, &registry, &now), (2, 1));
    }

    #[test]
    fn witness_signature_binds_checkpoint_actor_organization_and_time() {
        let witness_key = key(23);
        let now = Utc::now();
        let witness = SignedCredentialTransparencyWitness::sign(
            ContentHash([11; 32]),
            actor("did:key:witness-one"),
            OrganizationId::new("org:witness-one").unwrap(),
            now,
            &witness_key,
        )
        .unwrap();
        assert!(witness.verify().is_ok());
        let mut tampered = witness;
        tampered.organization = OrganizationId::new("org:witness-two").unwrap();
        assert!(tampered.verify().is_err());
    }

    #[test]
    fn compromise_interval_cannot_pre_authorize_future_restoration() {
        let recorded_at = Utc::now();
        let interval = TransparencyWitnessCompromiseInterval {
            compromised_from: recorded_at - chrono::Duration::hours(1),
            restored_at: Some(recorded_at + chrono::Duration::hours(1)),
            reason: "planned future restoration is not forensic evidence".to_string(),
        };
        assert!(interval.validate_at(&recorded_at).is_err());
    }
}
