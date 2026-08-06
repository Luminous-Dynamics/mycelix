// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Durable receipt-time authority evidence for canonical scientific events.
//!
//! Event signatures prove who controlled the event key. Governance additionally
//! decides whether that key was bound to the named actor and whether that actor
//! was authorized for the scientific action at receipt time. This module makes
//! that acceptance decision independently auditable without re-evaluating old
//! events against today's roles or key registry.

use crate::scientific_events::{
    ActorId, ClaimId, ContentHash, MAX_EVENT_FUTURE_SKEW_SECONDS, OrganizationId,
    ScientificEventId, ScientificEventLog, SignedScientificEvent,
};
use crate::scientific_governance::{AuthorizedActorKey, ScientificAction, ScientificRole};
use crate::{Error, Result};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use ed25519_dalek::{Signature, Signer, SigningKey, VerifyingKey};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::RwLock;

pub const AUTHORITY_RECEIPT_PROTOCOL: &str = "mycelix-desci-authority-receipt";
pub const AUTHORITY_RECEIPT_PROTOCOL_VERSION: u16 = 2;
pub const MIN_SUPPORTED_AUTHORITY_RECEIPT_PROTOCOL_VERSION: u16 = 1;
pub const MAX_AUTHORITY_RECEIPT_FILE_BYTES: u64 = 1024 * 1024;
const MAX_POLICY_ID_BYTES: usize = 256;
const MAX_POLICY_VERSION_BYTES: usize = 64;
const MAX_DECISION_REASON_BYTES: usize = 4096;

/// Receipt-time evidence captured from the identity resolver and authorization
/// policy. Later key rotation or role changes do not rewrite this history.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScientificAuthorityReceipt {
    pub protocol: String,
    pub protocol_version: u16,
    pub event_id: ScientificEventId,
    pub event_hash: ContentHash,
    pub stream_id: ClaimId,
    pub sequence: u64,
    pub received_at: DateTime<Utc>,
    pub actor: ActorId,
    pub acting_organization: Option<OrganizationId>,
    pub event_signer_public_key: [u8; 32],
    pub authorized_key: AuthorizedActorKey,
    pub authorized_organizations: BTreeSet<OrganizationId>,
    pub authorized_roles: BTreeSet<ScientificRole>,
    /// Exact credential-registry revision used to resolve the actor. Legacy
    /// v1 receipts predate the event-sourced registry and leave this unset.
    #[serde(default)]
    pub credential_registry_revision: Option<ContentHash>,
    pub action: ScientificAction,
    pub policy_id: String,
    pub policy_version: String,
    pub decision_reason: String,
    pub authority_snapshot_hash: ContentHash,
    pub previous_receipt_hash: Option<ContentHash>,
    /// When authority receipts begin after pre-v0.5 events, this anchors the
    /// first receipt to the preceding event without claiming those older events
    /// had receipt-time authority evidence.
    pub legacy_cutover_anchor: Option<ContentHash>,
}

impl ScientificAuthorityReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        event: &SignedScientificEvent,
        received_at: DateTime<Utc>,
        authorized_key: AuthorizedActorKey,
        authorized_organizations: BTreeSet<OrganizationId>,
        authorized_roles: BTreeSet<ScientificRole>,
        credential_registry_revision: Option<ContentHash>,
        action: ScientificAction,
        policy_id: impl Into<String>,
        policy_version: impl Into<String>,
        decision_reason: impl Into<String>,
        previous_receipt_hash: Option<ContentHash>,
        legacy_cutover_anchor: Option<ContentHash>,
    ) -> Result<Self> {
        let protocol_version = if credential_registry_revision.is_some() {
            AUTHORITY_RECEIPT_PROTOCOL_VERSION
        } else {
            MIN_SUPPORTED_AUTHORITY_RECEIPT_PROTOCOL_VERSION
        };
        let mut receipt = Self {
            protocol: AUTHORITY_RECEIPT_PROTOCOL.to_string(),
            protocol_version,
            event_id: event.envelope.event_id,
            event_hash: event.event_hash()?,
            stream_id: event.envelope.stream_id,
            sequence: event.envelope.sequence,
            received_at,
            actor: event.envelope.actor.clone(),
            acting_organization: event.envelope.acting_organization.clone(),
            event_signer_public_key: event.signer_public_key,
            authorized_key,
            authorized_organizations,
            authorized_roles,
            credential_registry_revision,
            action,
            policy_id: policy_id.into(),
            policy_version: policy_version.into(),
            decision_reason: decision_reason.into(),
            authority_snapshot_hash: ContentHash([0; 32]),
            previous_receipt_hash,
            legacy_cutover_anchor,
        };
        receipt.authority_snapshot_hash = receipt.compute_authority_snapshot_hash()?;
        receipt.validate_for_event(event)?;
        Ok(receipt)
    }

    pub fn signing_bytes(&self) -> Result<Vec<u8>> {
        self.validate_common()?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-RECEIPT\0");
        push_string(&mut bytes, &self.protocol)?;
        push_u16(&mut bytes, self.protocol_version);
        push_uuid(&mut bytes, self.event_id.0);
        push_hash(&mut bytes, self.event_hash);
        push_uuid(&mut bytes, self.stream_id.0);
        push_u64(&mut bytes, self.sequence);
        push_datetime(&mut bytes, &self.received_at);
        push_string(&mut bytes, self.actor.as_str())?;
        push_option_string(
            &mut bytes,
            self.acting_organization
                .as_ref()
                .map(OrganizationId::as_str),
        )?;
        bytes.extend_from_slice(&self.event_signer_public_key);
        push_authorized_key(&mut bytes, &self.authorized_key);
        push_organizations(&mut bytes, &self.authorized_organizations)?;
        push_roles(&mut bytes, &self.authorized_roles)?;
        if self.protocol_version >= 2 {
            push_option_hash(&mut bytes, self.credential_registry_revision);
        }
        bytes.push(self.action.code());
        push_string(&mut bytes, &self.policy_id)?;
        push_string(&mut bytes, &self.policy_version)?;
        push_string(&mut bytes, &self.decision_reason)?;
        push_hash(&mut bytes, self.authority_snapshot_hash);
        push_option_hash(&mut bytes, self.previous_receipt_hash);
        push_option_hash(&mut bytes, self.legacy_cutover_anchor);
        Ok(bytes)
    }

    pub fn compute_authority_snapshot_hash(&self) -> Result<ContentHash> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-AUTHORITY-SNAPSHOT\0");
        push_string(&mut bytes, self.actor.as_str())?;
        push_authorized_key(&mut bytes, &self.authorized_key);
        push_organizations(&mut bytes, &self.authorized_organizations)?;
        push_roles(&mut bytes, &self.authorized_roles)?;
        if self.protocol_version >= 2 {
            push_option_hash(&mut bytes, self.credential_registry_revision);
        }
        Ok(ContentHash::digest(&bytes))
    }

    pub fn validate_for_event(&self, event: &SignedScientificEvent) -> Result<()> {
        self.validate_common()?;
        event.verify()?;
        let latest_allowed =
            self.received_at.clone() + chrono::Duration::seconds(MAX_EVENT_FUTURE_SKEW_SECONDS);
        if event.envelope.occurred_at > latest_allowed {
            return Err(Error::VerificationFailed(
                "authority receipt records an event beyond the allowed future clock skew"
                    .to_string(),
            ));
        }
        if self.event_id != event.envelope.event_id
            || self.event_hash != event.event_hash()?
            || self.stream_id != event.envelope.stream_id
            || self.sequence != event.envelope.sequence
            || self.actor != event.envelope.actor
            || self.acting_organization != event.envelope.acting_organization
            || self.event_signer_public_key != event.signer_public_key
        {
            return Err(Error::VerificationFailed(
                "authority receipt does not bind exactly to the scientific event".to_string(),
            ));
        }
        if self.action != ScientificAction::from_payload(&event.envelope.payload) {
            return Err(Error::VerificationFailed(
                "authority receipt action does not match event payload".to_string(),
            ));
        }
        if self.authorized_key.public_key != event.signer_public_key
            || !self.authorized_key.is_active_at(&self.received_at)
        {
            return Err(Error::VerificationFailed(
                "authority receipt key was not active at receipt time".to_string(),
            ));
        }
        if let Some(organization) = &self.acting_organization {
            if !self.authorized_organizations.contains(organization) {
                return Err(Error::VerificationFailed(
                    "authority receipt organization is absent from the actor snapshot".to_string(),
                ));
            }
        }
        if self.authority_snapshot_hash != self.compute_authority_snapshot_hash()? {
            return Err(Error::VerificationFailed(
                "authority receipt snapshot hash is invalid".to_string(),
            ));
        }
        if self.sequence == 0 {
            if self.previous_receipt_hash.is_some() || self.legacy_cutover_anchor.is_some() {
                return Err(Error::VerificationFailed(
                    "genesis authority receipt cannot have a previous receipt or cutover anchor"
                        .to_string(),
                ));
            }
        } else if self.previous_receipt_hash.is_none() {
            if self.legacy_cutover_anchor != event.envelope.previous_hash {
                return Err(Error::VerificationFailed(
                    "first post-legacy authority receipt must anchor to the preceding event hash"
                        .to_string(),
                ));
            }
        } else if self.legacy_cutover_anchor.is_some() {
            return Err(Error::VerificationFailed(
                "continued authority receipt chains cannot carry a legacy cutover anchor"
                    .to_string(),
            ));
        }
        Ok(())
    }

    fn validate_common(&self) -> Result<()> {
        if self.protocol != AUTHORITY_RECEIPT_PROTOCOL
            || !(MIN_SUPPORTED_AUTHORITY_RECEIPT_PROTOCOL_VERSION
                ..=AUTHORITY_RECEIPT_PROTOCOL_VERSION)
                .contains(&self.protocol_version)
        {
            return Err(Error::Validation(
                "unsupported scientific authority receipt protocol".to_string(),
            ));
        }
        if self.protocol_version == 1 && self.credential_registry_revision.is_some() {
            return Err(Error::Validation(
                "authority receipt v1 cannot carry a credential registry revision".to_string(),
            ));
        }
        if self.protocol_version >= 2 && self.credential_registry_revision.is_none() {
            return Err(Error::Validation(
                "authority receipt v2 requires a credential registry revision".to_string(),
            ));
        }
        self.actor.validate()?;
        if let Some(organization) = &self.acting_organization {
            organization.validate()?;
        }
        for organization in &self.authorized_organizations {
            organization.validate()?;
        }
        self.authorized_key.validate()?;
        if !self.authorized_key.is_active_at(&self.received_at) {
            return Err(Error::Validation(
                "authority receipt key was not active at receipt time".to_string(),
            ));
        }
        if let Some(organization) = &self.acting_organization {
            if !self.authorized_organizations.contains(organization) {
                return Err(Error::Validation(
                    "authority receipt acting organization is absent from the authority snapshot"
                        .to_string(),
                ));
            }
        }
        if self.authorized_roles.is_empty() {
            return Err(Error::Validation(
                "authority receipt requires at least one authorized role".to_string(),
            ));
        }
        if self.policy_id.trim().is_empty()
            || self.policy_version.trim().is_empty()
            || self.decision_reason.trim().is_empty()
        {
            return Err(Error::Validation(
                "authority receipt policy and decision fields cannot be empty".to_string(),
            ));
        }
        if self.policy_id.len() > MAX_POLICY_ID_BYTES
            || self.policy_version.len() > MAX_POLICY_VERSION_BYTES
            || self.decision_reason.len() > MAX_DECISION_REASON_BYTES
        {
            return Err(Error::Validation(
                "authority receipt policy or decision field exceeds its size limit".to_string(),
            ));
        }
        if self.authority_snapshot_hash != self.compute_authority_snapshot_hash()? {
            return Err(Error::Validation(
                "authority receipt snapshot hash is invalid".to_string(),
            ));
        }
        if self.sequence == 0 {
            if self.previous_receipt_hash.is_some() || self.legacy_cutover_anchor.is_some() {
                return Err(Error::Validation(
                    "genesis authority receipt cannot have chain predecessors".to_string(),
                ));
            }
        } else if self.previous_receipt_hash.is_some() == self.legacy_cutover_anchor.is_some() {
            return Err(Error::Validation(
                "non-genesis authority receipt requires exactly one chain predecessor".to_string(),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SignedScientificAuthorityReceipt {
    pub receipt: ScientificAuthorityReceipt,
    pub service_public_key: [u8; 32],
    pub signature: Vec<u8>,
}

impl SignedScientificAuthorityReceipt {
    pub fn sign(receipt: ScientificAuthorityReceipt, signing_key: &SigningKey) -> Result<Self> {
        let signature = signing_key
            .sign(&receipt.signing_bytes()?)
            .to_bytes()
            .to_vec();
        Ok(Self {
            receipt,
            service_public_key: signing_key.verifying_key().to_bytes(),
            signature,
        })
    }

    pub fn verify(&self, trusted_service_keys: &BTreeSet<[u8; 32]>) -> Result<()> {
        if self.service_public_key == self.receipt.event_signer_public_key {
            return Err(Error::VerificationFailed(
                "authority receipts require a service key distinct from the event signer"
                    .to_string(),
            ));
        }
        if !trusted_service_keys.contains(&self.service_public_key) {
            return Err(Error::VerificationFailed(
                "authority receipt service key is not trusted".to_string(),
            ));
        }
        let verifying_key = VerifyingKey::from_bytes(&self.service_public_key)
            .map_err(|error| Error::Crypto(error.to_string()))?;
        let signature = Signature::try_from(self.signature.as_slice())
            .map_err(|error| Error::Crypto(error.to_string()))?;
        verifying_key
            .verify_strict(&self.receipt.signing_bytes()?, &signature)
            .map_err(|error| Error::VerificationFailed(error.to_string()))
    }

    pub fn verify_for_event(
        &self,
        event: &SignedScientificEvent,
        trusted_service_keys: &BTreeSet<[u8; 32]>,
    ) -> Result<()> {
        self.verify(trusted_service_keys)?;
        self.receipt.validate_for_event(event)
    }

    pub fn receipt_hash(&self) -> Result<ContentHash> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"MYCELIX-DESCI-SIGNED-AUTHORITY-RECEIPT\0");
        let signing_bytes = self.receipt.signing_bytes()?;
        push_len(&mut bytes, signing_bytes.len())?;
        bytes.extend_from_slice(&signing_bytes);
        bytes.extend_from_slice(&self.service_public_key);
        push_len(&mut bytes, self.signature.len())?;
        bytes.extend_from_slice(&self.signature);
        Ok(ContentHash::digest(&bytes))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthorityAttestationStatus {
    ReceiptAttested,
    PendingReconciliation,
    LegacyUnattested,
    UnsafeUnattested,
    Unknown,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityAuditSummary {
    pub committed_receipts: usize,
    pub pending_receipts: usize,
    pub unattested_events: usize,
    pub legacy_unattested_events: usize,
    pub unsafe_unattested_events: usize,
    pub receipt_chains: usize,
    pub last_reconciled_at: Option<DateTime<Utc>>,
}

#[async_trait]
pub trait ScientificAuthorityAuditStore: Send + Sync {
    async fn prepare(&self, receipt: SignedScientificAuthorityReceipt) -> Result<()>;
    async fn commit(&self, event_id: ScientificEventId) -> Result<()>;
    async fn abort(&self, event_id: ScientificEventId) -> Result<()>;
    async fn receipt(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>>;
    async fn status(&self, event_id: ScientificEventId) -> Result<AuthorityAttestationStatus>;
    async fn stream_head(
        &self,
        claim_id: ClaimId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>>;
    async fn stream_receipts(
        &self,
        claim_id: ClaimId,
    ) -> Result<Vec<SignedScientificAuthorityReceipt>>;
    async fn reconcile(&self, event_log: &dyn ScientificEventLog) -> Result<AuthorityAuditSummary>;
    async fn summary(&self) -> Result<AuthorityAuditSummary>;
}

#[async_trait]
impl<T> ScientificAuthorityAuditStore for Arc<T>
where
    T: ScientificAuthorityAuditStore + ?Sized,
{
    async fn prepare(&self, receipt: SignedScientificAuthorityReceipt) -> Result<()> {
        (**self).prepare(receipt).await
    }

    async fn commit(&self, event_id: ScientificEventId) -> Result<()> {
        (**self).commit(event_id).await
    }

    async fn abort(&self, event_id: ScientificEventId) -> Result<()> {
        (**self).abort(event_id).await
    }

    async fn receipt(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        (**self).receipt(event_id).await
    }

    async fn status(&self, event_id: ScientificEventId) -> Result<AuthorityAttestationStatus> {
        (**self).status(event_id).await
    }

    async fn stream_head(
        &self,
        claim_id: ClaimId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        (**self).stream_head(claim_id).await
    }

    async fn stream_receipts(
        &self,
        claim_id: ClaimId,
    ) -> Result<Vec<SignedScientificAuthorityReceipt>> {
        (**self).stream_receipts(claim_id).await
    }

    async fn reconcile(&self, event_log: &dyn ScientificEventLog) -> Result<AuthorityAuditSummary> {
        (**self).reconcile(event_log).await
    }

    async fn summary(&self) -> Result<AuthorityAuditSummary> {
        (**self).summary().await
    }
}

#[derive(Debug, Clone, Default)]
struct AuthorityAuditState {
    committed: HashMap<ScientificEventId, SignedScientificAuthorityReceipt>,
    pending: HashMap<ScientificEventId, SignedScientificAuthorityReceipt>,
    legacy_unattested_events: BTreeSet<ScientificEventId>,
    unsafe_unattested_events: BTreeSet<ScientificEventId>,
    last_reconciled_at: Option<DateTime<Utc>>,
}

impl AuthorityAuditState {
    fn summary(&self) -> AuthorityAuditSummary {
        AuthorityAuditSummary {
            committed_receipts: self.committed.len(),
            pending_receipts: self.pending.len(),
            unattested_events: self.legacy_unattested_events.len()
                + self.unsafe_unattested_events.len(),
            legacy_unattested_events: self.legacy_unattested_events.len(),
            unsafe_unattested_events: self.unsafe_unattested_events.len(),
            receipt_chains: self
                .committed
                .values()
                .map(|receipt| receipt.receipt.stream_id)
                .collect::<BTreeSet<_>>()
                .len(),
            last_reconciled_at: self.last_reconciled_at.clone(),
        }
    }

    fn status(&self, event_id: ScientificEventId) -> AuthorityAttestationStatus {
        if self.committed.contains_key(&event_id) {
            AuthorityAttestationStatus::ReceiptAttested
        } else if self.pending.contains_key(&event_id) {
            AuthorityAttestationStatus::PendingReconciliation
        } else if self.legacy_unattested_events.contains(&event_id) {
            AuthorityAttestationStatus::LegacyUnattested
        } else if self.unsafe_unattested_events.contains(&event_id) {
            AuthorityAttestationStatus::UnsafeUnattested
        } else {
            AuthorityAttestationStatus::Unknown
        }
    }

    fn stream_head(&self, claim_id: ClaimId) -> Option<SignedScientificAuthorityReceipt> {
        self.committed
            .values()
            .chain(self.pending.values())
            .filter(|receipt| receipt.receipt.stream_id == claim_id)
            .max_by_key(|receipt| receipt.receipt.sequence)
            .cloned()
    }

    fn stream_receipts(&self, claim_id: ClaimId) -> Vec<SignedScientificAuthorityReceipt> {
        let mut receipts = self
            .committed
            .values()
            .filter(|receipt| receipt.receipt.stream_id == claim_id)
            .cloned()
            .collect::<Vec<_>>();
        receipts.sort_by_key(|receipt| receipt.receipt.sequence);
        receipts
    }
}

#[derive(Debug, Clone)]
pub struct MemoryScientificAuthorityAuditStore {
    trusted_service_keys: Arc<BTreeSet<[u8; 32]>>,
    state: Arc<RwLock<AuthorityAuditState>>,
}

impl MemoryScientificAuthorityAuditStore {
    pub fn new(trusted_service_keys: BTreeSet<[u8; 32]>) -> Self {
        Self {
            trusted_service_keys: Arc::new(trusted_service_keys),
            state: Arc::new(RwLock::new(AuthorityAuditState::default())),
        }
    }
}

#[async_trait]
impl ScientificAuthorityAuditStore for MemoryScientificAuthorityAuditStore {
    async fn prepare(&self, receipt: SignedScientificAuthorityReceipt) -> Result<()> {
        receipt.verify(&self.trusted_service_keys)?;
        let event_id = receipt.receipt.event_id;
        let mut state = self.state.write().await;
        if state.committed.contains_key(&event_id) || state.pending.contains_key(&event_id) {
            return Err(Error::Storage(
                "duplicate scientific authority receipt event id".to_string(),
            ));
        }
        validate_prepare_chain(&state, &receipt)?;
        state.pending.insert(event_id, receipt);
        Ok(())
    }

    async fn commit(&self, event_id: ScientificEventId) -> Result<()> {
        let mut state = self.state.write().await;
        let receipt = state.pending.remove(&event_id).ok_or_else(|| {
            Error::Storage("scientific authority receipt was not prepared".to_string())
        })?;
        state.committed.insert(event_id, receipt);
        state.legacy_unattested_events.remove(&event_id);
        state.unsafe_unattested_events.remove(&event_id);
        Ok(())
    }

    async fn abort(&self, event_id: ScientificEventId) -> Result<()> {
        self.state.write().await.pending.remove(&event_id);
        Ok(())
    }

    async fn receipt(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.committed.get(&event_id).cloned())
    }

    async fn status(&self, event_id: ScientificEventId) -> Result<AuthorityAttestationStatus> {
        Ok(self.state.read().await.status(event_id))
    }

    async fn stream_head(
        &self,
        claim_id: ClaimId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.stream_head(claim_id))
    }

    async fn stream_receipts(
        &self,
        claim_id: ClaimId,
    ) -> Result<Vec<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.stream_receipts(claim_id))
    }

    async fn reconcile(&self, event_log: &dyn ScientificEventLog) -> Result<AuthorityAuditSummary> {
        let events = collect_events(event_log).await?;
        let mut state = self.state.write().await;
        let pending_ids = state.pending.keys().copied().collect::<Vec<_>>();
        for event_id in pending_ids {
            if let Some(event) = events.get(&event_id) {
                let receipt = state
                    .pending
                    .remove(&event_id)
                    .expect("pending receipt exists");
                receipt.verify_for_event(event, &self.trusted_service_keys)?;
                state.committed.insert(event_id, receipt);
            } else {
                state.pending.remove(&event_id);
            }
        }
        reconcile_state(&mut state, &events, &self.trusted_service_keys)?;
        Ok(state.summary())
    }

    async fn summary(&self) -> Result<AuthorityAuditSummary> {
        Ok(self.state.read().await.summary())
    }
}

#[derive(Debug, Clone)]
pub struct FileScientificAuthorityAuditStore {
    root: Arc<PathBuf>,
    trusted_service_keys: Arc<BTreeSet<[u8; 32]>>,
    state: Arc<RwLock<AuthorityAuditState>>,
}

impl FileScientificAuthorityAuditStore {
    pub async fn open(
        path: impl AsRef<Path>,
        trusted_service_keys: BTreeSet<[u8; 32]>,
    ) -> Result<Self> {
        let root = path.as_ref().to_path_buf();
        if let Ok(metadata) = fs::symlink_metadata(&root) {
            if metadata.file_type().is_symlink() {
                return Err(Error::Storage(format!(
                    "scientific authority audit root must not be a symbolic link: {}",
                    root.display()
                )));
            }
        }
        fs::create_dir_all(&root)?;
        let mut state = AuthorityAuditState::default();
        for entry in fs::read_dir(&root)? {
            let entry = entry?;
            let path = entry.path();
            let file_type = entry.file_type()?;
            if file_type.is_symlink() {
                return Err(Error::Storage(format!(
                    "scientific authority audit refuses symbolic links: {}",
                    path.display()
                )));
            }
            if !file_type.is_file() {
                continue;
            }
            let name = entry.file_name().to_string_lossy().to_string();
            let pending = name.starts_with(".pending.") && name.ends_with(".authority.json");
            let committed = !name.starts_with('.') && name.ends_with(".authority.json");
            if !pending && !committed {
                continue;
            }
            let metadata = entry.metadata()?;
            if metadata.len() > MAX_AUTHORITY_RECEIPT_FILE_BYTES {
                return Err(Error::Storage(format!(
                    "scientific authority receipt exceeds {} bytes: {}",
                    MAX_AUTHORITY_RECEIPT_FILE_BYTES,
                    path.display()
                )));
            }
            let receipt: SignedScientificAuthorityReceipt =
                serde_json::from_slice(&fs::read(&path)?)?;
            receipt.verify(&trusted_service_keys)?;
            let expected_name = if pending {
                format!(".pending.{}.authority.json", receipt.receipt.event_id.0)
            } else {
                final_receipt_name(&receipt)
            };
            if name != expected_name {
                return Err(Error::Storage(format!(
                    "scientific authority receipt filename does not match its content: {}",
                    path.display()
                )));
            }
            let target = if pending {
                &mut state.pending
            } else {
                &mut state.committed
            };
            if target.insert(receipt.receipt.event_id, receipt).is_some() {
                return Err(Error::Storage(
                    "duplicate scientific authority receipt event id".to_string(),
                ));
            }
        }
        if state
            .pending
            .keys()
            .any(|event_id| state.committed.contains_key(event_id))
        {
            return Err(Error::Storage(
                "authority receipt exists as both pending and committed".to_string(),
            ));
        }
        Ok(Self {
            root: Arc::new(root),
            trusted_service_keys: Arc::new(trusted_service_keys),
            state: Arc::new(RwLock::new(state)),
        })
    }

    fn pending_path(&self, event_id: ScientificEventId) -> PathBuf {
        self.root
            .join(format!(".pending.{}.authority.json", event_id.0))
    }

    fn final_path(&self, receipt: &SignedScientificAuthorityReceipt) -> PathBuf {
        self.root.join(final_receipt_name(receipt))
    }

    fn write_new(&self, path: &Path, receipt: &SignedScientificAuthorityReceipt) -> Result<()> {
        let bytes = serde_json::to_vec_pretty(receipt)?;
        if bytes.len() as u64 > MAX_AUTHORITY_RECEIPT_FILE_BYTES {
            return Err(Error::Storage(
                "scientific authority receipt exceeds file-size limit".to_string(),
            ));
        }
        let mut file = OpenOptions::new().create_new(true).write(true).open(path)?;
        file.write_all(&bytes)?;
        file.sync_all()?;
        sync_directory(self.root.as_ref());
        Ok(())
    }

    fn commit_file(&self, receipt: &SignedScientificAuthorityReceipt) -> Result<()> {
        let pending = self.pending_path(receipt.receipt.event_id);
        let target = self.final_path(receipt);
        if target.exists() {
            return Err(Error::Storage(
                "scientific authority receipt target already exists".to_string(),
            ));
        }
        fs::rename(&pending, &target)?;
        sync_directory(self.root.as_ref());
        Ok(())
    }
}

#[async_trait]
impl ScientificAuthorityAuditStore for FileScientificAuthorityAuditStore {
    async fn prepare(&self, receipt: SignedScientificAuthorityReceipt) -> Result<()> {
        receipt.verify(&self.trusted_service_keys)?;
        let event_id = receipt.receipt.event_id;
        let mut state = self.state.write().await;
        if state.committed.contains_key(&event_id) || state.pending.contains_key(&event_id) {
            return Err(Error::Storage(
                "duplicate scientific authority receipt event id".to_string(),
            ));
        }
        validate_prepare_chain(&state, &receipt)?;
        let path = self.pending_path(event_id);
        self.write_new(&path, &receipt)?;
        state.pending.insert(event_id, receipt);
        Ok(())
    }

    async fn commit(&self, event_id: ScientificEventId) -> Result<()> {
        let mut state = self.state.write().await;
        let receipt = state.pending.get(&event_id).cloned().ok_or_else(|| {
            Error::Storage("scientific authority receipt was not prepared".to_string())
        })?;
        self.commit_file(&receipt)?;
        state.pending.remove(&event_id);
        state.committed.insert(event_id, receipt);
        state.legacy_unattested_events.remove(&event_id);
        state.unsafe_unattested_events.remove(&event_id);
        Ok(())
    }

    async fn abort(&self, event_id: ScientificEventId) -> Result<()> {
        let mut state = self.state.write().await;
        if state.pending.remove(&event_id).is_some() {
            let path = self.pending_path(event_id);
            match fs::remove_file(&path) {
                Ok(()) => sync_directory(self.root.as_ref()),
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(error.into()),
            }
        }
        Ok(())
    }

    async fn receipt(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.committed.get(&event_id).cloned())
    }

    async fn status(&self, event_id: ScientificEventId) -> Result<AuthorityAttestationStatus> {
        Ok(self.state.read().await.status(event_id))
    }

    async fn stream_head(
        &self,
        claim_id: ClaimId,
    ) -> Result<Option<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.stream_head(claim_id))
    }

    async fn stream_receipts(
        &self,
        claim_id: ClaimId,
    ) -> Result<Vec<SignedScientificAuthorityReceipt>> {
        Ok(self.state.read().await.stream_receipts(claim_id))
    }

    async fn reconcile(&self, event_log: &dyn ScientificEventLog) -> Result<AuthorityAuditSummary> {
        let events = collect_events(event_log).await?;
        let mut state = self.state.write().await;
        let pending_ids = state.pending.keys().copied().collect::<Vec<_>>();
        for event_id in pending_ids {
            if let Some(event) = events.get(&event_id) {
                let receipt = state
                    .pending
                    .get(&event_id)
                    .cloned()
                    .expect("pending exists");
                receipt.verify_for_event(event, &self.trusted_service_keys)?;
                self.commit_file(&receipt)?;
                state.pending.remove(&event_id);
                state.committed.insert(event_id, receipt);
            } else {
                state.pending.remove(&event_id);
                let pending_path = self.pending_path(event_id);
                match fs::remove_file(&pending_path) {
                    Ok(()) => sync_directory(self.root.as_ref()),
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                    Err(error) => return Err(error.into()),
                }
            }
        }
        reconcile_state(&mut state, &events, &self.trusted_service_keys)?;
        Ok(state.summary())
    }

    async fn summary(&self) -> Result<AuthorityAuditSummary> {
        Ok(self.state.read().await.summary())
    }
}

fn validate_prepare_chain(
    state: &AuthorityAuditState,
    receipt: &SignedScientificAuthorityReceipt,
) -> Result<()> {
    if state
        .committed
        .values()
        .chain(state.pending.values())
        .any(|existing| {
            existing.receipt.stream_id == receipt.receipt.stream_id
                && existing.receipt.sequence == receipt.receipt.sequence
        })
    {
        return Err(Error::Storage(
            "duplicate scientific authority receipt stream sequence".to_string(),
        ));
    }

    match state.stream_head(receipt.receipt.stream_id) {
        Some(previous) => {
            if receipt.receipt.sequence != previous.receipt.sequence + 1
                || receipt.receipt.previous_receipt_hash != Some(previous.receipt_hash()?)
                || receipt.receipt.legacy_cutover_anchor.is_some()
            {
                return Err(Error::Storage(
                    "scientific authority receipt does not extend the current receipt chain"
                        .to_string(),
                ));
            }
        }
        None if receipt.receipt.sequence == 0 => {
            if receipt.receipt.previous_receipt_hash.is_some()
                || receipt.receipt.legacy_cutover_anchor.is_some()
            {
                return Err(Error::Storage(
                    "genesis authority receipt has invalid chain fields".to_string(),
                ));
            }
        }
        None => {
            if receipt.receipt.previous_receipt_hash.is_some()
                || receipt.receipt.legacy_cutover_anchor.is_none()
            {
                return Err(Error::Storage(
                    "first post-legacy authority receipt requires a cutover anchor".to_string(),
                ));
            }
        }
    }
    Ok(())
}

async fn collect_events(
    event_log: &dyn ScientificEventLog,
) -> Result<HashMap<ScientificEventId, SignedScientificEvent>> {
    let mut events = HashMap::new();
    for stream_id in event_log.stream_ids().await? {
        for event in event_log.stream(stream_id).await? {
            if events.insert(event.envelope.event_id, event).is_some() {
                return Err(Error::Storage(
                    "duplicate scientific event id during authority reconciliation".to_string(),
                ));
            }
        }
    }
    Ok(events)
}

fn reconcile_state(
    state: &mut AuthorityAuditState,
    events: &HashMap<ScientificEventId, SignedScientificEvent>,
    trusted_service_keys: &BTreeSet<[u8; 32]>,
) -> Result<()> {
    for (event_id, receipt) in &state.committed {
        let event = events.get(event_id).ok_or_else(|| {
            Error::Storage(format!(
                "committed authority receipt has no scientific event: {event_id}"
            ))
        })?;
        receipt.verify_for_event(event, trusted_service_keys)?;
    }

    let mut by_stream: BTreeMap<ClaimId, Vec<&SignedScientificAuthorityReceipt>> = BTreeMap::new();
    for receipt in state.committed.values() {
        by_stream
            .entry(receipt.receipt.stream_id)
            .or_default()
            .push(receipt);
    }
    for receipts in by_stream.values_mut() {
        receipts.sort_by_key(|receipt| receipt.receipt.sequence);
        for (index, receipt) in receipts.iter().enumerate() {
            if index == 0 {
                if receipt.receipt.sequence == 0 {
                    if receipt.receipt.previous_receipt_hash.is_some()
                        || receipt.receipt.legacy_cutover_anchor.is_some()
                    {
                        return Err(Error::Storage(
                            "invalid genesis authority receipt chain".to_string(),
                        ));
                    }
                } else if receipt.receipt.previous_receipt_hash.is_some()
                    || receipt.receipt.legacy_cutover_anchor.is_none()
                {
                    return Err(Error::Storage(
                        "authority receipt chain must explicitly mark its legacy cutover"
                            .to_string(),
                    ));
                }
                continue;
            }
            let previous = receipts[index - 1];
            if receipt.receipt.sequence != previous.receipt.sequence + 1
                || receipt.receipt.previous_receipt_hash != Some(previous.receipt_hash()?)
                || receipt.receipt.legacy_cutover_anchor.is_some()
            {
                return Err(Error::Storage(
                    "scientific authority receipt chain is discontinuous".to_string(),
                ));
            }
        }
    }

    let first_receipt_sequence = by_stream
        .iter()
        .filter_map(|(stream_id, receipts)| {
            receipts
                .first()
                .map(|receipt| (*stream_id, receipt.receipt.sequence))
        })
        .collect::<HashMap<_, _>>();
    drop(by_stream);
    state.legacy_unattested_events.clear();
    state.unsafe_unattested_events.clear();
    for (event_id, event) in events {
        if state.committed.contains_key(event_id) {
            continue;
        }
        match first_receipt_sequence.get(&event.envelope.stream_id) {
            None => {
                state.legacy_unattested_events.insert(*event_id);
            }
            Some(first_sequence) if event.envelope.sequence < *first_sequence => {
                state.legacy_unattested_events.insert(*event_id);
            }
            Some(_) => {
                state.unsafe_unattested_events.insert(*event_id);
            }
        }
    }
    state.last_reconciled_at = Some(Utc::now());
    Ok(())
}

fn final_receipt_name(receipt: &SignedScientificAuthorityReceipt) -> String {
    format!(
        "{}.{:020}.{}.authority.json",
        receipt.receipt.stream_id.0, receipt.receipt.sequence, receipt.receipt.event_id.0
    )
}

fn sync_directory(path: &Path) {
    if let Ok(directory) = File::open(path) {
        let _ = directory.sync_all();
    }
}

fn push_len(bytes: &mut Vec<u8>, len: usize) -> Result<()> {
    let len = u32::try_from(len)
        .map_err(|_| Error::Validation("canonical receipt field is too large".to_string()))?;
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

fn push_i64(bytes: &mut Vec<u8>, value: i64) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_be_bytes());
}

fn push_uuid(bytes: &mut Vec<u8>, value: uuid::Uuid) {
    bytes.extend_from_slice(value.as_bytes());
}

fn push_hash(bytes: &mut Vec<u8>, value: ContentHash) {
    bytes.extend_from_slice(&value.0);
}

fn push_option_hash(bytes: &mut Vec<u8>, value: Option<ContentHash>) {
    match value {
        Some(value) => {
            bytes.push(1);
            push_hash(bytes, value);
        }
        None => bytes.push(0),
    }
}

fn push_datetime(bytes: &mut Vec<u8>, value: &DateTime<Utc>) {
    push_i64(bytes, value.timestamp());
    push_u32(bytes, value.timestamp_subsec_nanos());
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

fn push_option_string(bytes: &mut Vec<u8>, value: Option<&str>) -> Result<()> {
    match value {
        Some(value) => {
            bytes.push(1);
            push_string(bytes, value)?;
        }
        None => bytes.push(0),
    }
    Ok(())
}

fn push_authorized_key(bytes: &mut Vec<u8>, key: &AuthorizedActorKey) {
    bytes.extend_from_slice(&key.public_key);
    push_datetime(bytes, &key.valid_from);
    push_option_datetime(bytes, key.valid_until.as_ref());
    push_option_datetime(bytes, key.revoked_at.as_ref());
}

fn push_organizations(bytes: &mut Vec<u8>, organizations: &BTreeSet<OrganizationId>) -> Result<()> {
    push_len(bytes, organizations.len())?;
    for organization in organizations {
        push_string(bytes, organization.as_str())?;
    }
    Ok(())
}

fn push_roles(bytes: &mut Vec<u8>, roles: &BTreeSet<ScientificRole>) -> Result<()> {
    push_len(bytes, roles.len())?;
    for role in roles {
        bytes.push(role.code());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scientific_events::{
        ArtifactAvailability, ArtifactId, AtomicClaim, EvidenceArtifact, FileScientificEventLog,
        MemoryScientificEventLog, ResearchObject, ResearchObjectId, ResearchObjectType,
        ScientificEventEnvelope, ScientificEventPayload,
    };
    use chrono::TimeZone;

    fn event(signing_key: &SigningKey) -> SignedScientificEvent {
        let object_id = ResearchObjectId::new();
        let claim_id = ClaimId::new();
        let payload = ScientificEventPayload::ClaimProposed {
            research_object: ResearchObject {
                id: object_id,
                title: "Receipt test".to_string(),
                object_type: ResearchObjectType::Manuscript,
                persistent_identifier: None,
            },
            claim: AtomicClaim {
                id: claim_id,
                research_object_id: object_id,
                statement: "Receipt evidence is independently verifiable".to_string(),
                scope: None,
            },
        };
        SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                ActorId::new("did:key:receipt-author").unwrap(),
                Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 0).unwrap(),
                payload,
            )
            .unwrap(),
            signing_key,
        )
        .unwrap()
    }

    fn signed_receipt_with_chain(
        event: &SignedScientificEvent,
        receipt_key: &SigningKey,
        previous_receipt_hash: Option<ContentHash>,
        legacy_cutover_anchor: Option<ContentHash>,
    ) -> SignedScientificAuthorityReceipt {
        let received_at = Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 5).unwrap();
        let receipt = ScientificAuthorityReceipt::new(
            event,
            received_at.clone(),
            AuthorizedActorKey {
                public_key: event.signer_public_key,
                valid_from: received_at - chrono::Duration::minutes(1),
                valid_until: None,
                revoked_at: None,
            },
            BTreeSet::new(),
            BTreeSet::from([ScientificRole::Contributor]),
            None,
            ScientificAction::from_payload(&event.envelope.payload),
            "default-policy",
            "1.0.0",
            "actor was authorized at receipt time",
            previous_receipt_hash,
            legacy_cutover_anchor,
        )
        .unwrap();
        SignedScientificAuthorityReceipt::sign(receipt, receipt_key).unwrap()
    }

    fn signed_receipt(
        event: &SignedScientificEvent,
        receipt_key: &SigningKey,
    ) -> SignedScientificAuthorityReceipt {
        signed_receipt_with_chain(event, receipt_key, None, None)
    }

    fn evidence_event(
        previous: &SignedScientificEvent,
        signing_key: &SigningKey,
        marker: u8,
        second: u32,
    ) -> SignedScientificEvent {
        let artifact = EvidenceArtifact {
            id: ArtifactId::new(),
            content_hash: ContentHash::digest(&[marker]),
            media_type: "application/json".to_string(),
            locator: format!("ipfs://artifact-{marker}"),
            license: Some("CC0-1.0".to_string()),
            availability: ArtifactAvailability::Public,
        };
        SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                previous,
                previous.envelope.actor.clone(),
                Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, second).unwrap(),
                ScientificEventPayload::EvidenceAttached {
                    claim_id: previous.envelope.stream_id,
                    artifact,
                },
            )
            .unwrap(),
            signing_key,
        )
        .unwrap()
    }

    #[test]
    fn schema_v2_receipt_signature_binds_credential_registry_revision() {
        let event_key = SigningKey::from_bytes(&[7; 32]);
        let receipt_key = SigningKey::from_bytes(&[9; 32]);
        let event = event(&event_key);
        let received_at = Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 5).unwrap();
        let revision = ContentHash::digest(b"credential-registry-revision-a");
        let receipt = ScientificAuthorityReceipt::new(
            &event,
            received_at.clone(),
            AuthorizedActorKey {
                public_key: event.signer_public_key,
                valid_from: received_at - chrono::Duration::minutes(1),
                valid_until: None,
                revoked_at: None,
            },
            BTreeSet::new(),
            BTreeSet::from([ScientificRole::Contributor]),
            Some(revision),
            ScientificAction::from_payload(&event.envelope.payload),
            "default-policy",
            "1.0.0",
            "actor was authorized under a precise credential revision",
            None,
            None,
        )
        .unwrap();
        let mut signed = SignedScientificAuthorityReceipt::sign(receipt, &receipt_key).unwrap();
        let trusted = BTreeSet::from([receipt_key.verifying_key().to_bytes()]);
        signed.verify_for_event(&event, &trusted).unwrap();
        signed.receipt.credential_registry_revision =
            Some(ContentHash::digest(b"credential-registry-revision-b"));
        assert!(signed.verify_for_event(&event, &trusted).is_err());
    }

    #[tokio::test]
    async fn pending_receipt_is_finalized_when_event_exists() {
        let event_key = SigningKey::from_bytes(&[7; 32]);
        let receipt_key = SigningKey::from_bytes(&[9; 32]);
        let event = event(&event_key);
        let trusted = BTreeSet::from([receipt_key.verifying_key().to_bytes()]);
        let audit = MemoryScientificAuthorityAuditStore::new(trusted);
        let log = MemoryScientificEventLog::new();
        let receipt = signed_receipt(&event, &receipt_key);
        audit.prepare(receipt).await.unwrap();
        log.append_at(
            0,
            event,
            Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 1).unwrap(),
        )
        .await
        .unwrap();
        let summary = audit.reconcile(&log).await.unwrap();
        assert_eq!(summary.committed_receipts, 1);
        assert_eq!(summary.pending_receipts, 0);
        assert_eq!(summary.unattested_events, 0);
    }

    #[tokio::test]
    async fn file_journal_recovers_pending_receipt_after_event_commit() {
        let directory = tempfile::tempdir().unwrap();
        let event_key = SigningKey::from_bytes(&[13; 32]);
        let receipt_key = SigningKey::from_bytes(&[14; 32]);
        let trusted = BTreeSet::from([receipt_key.verifying_key().to_bytes()]);
        let event = event(&event_key);
        let event_id = event.envelope.event_id;
        let event_log = FileScientificEventLog::open(directory.path().join("events"))
            .await
            .unwrap();
        let audit_path = directory.path().join("authority");
        let audit = FileScientificAuthorityAuditStore::open(&audit_path, trusted.clone())
            .await
            .unwrap();
        audit
            .prepare(signed_receipt(&event, &receipt_key))
            .await
            .unwrap();
        event_log
            .append_at(
                0,
                event,
                Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 1).unwrap(),
            )
            .await
            .unwrap();
        drop(audit);

        let reopened = FileScientificAuthorityAuditStore::open(&audit_path, trusted)
            .await
            .unwrap();
        let summary = reopened.reconcile(&event_log).await.unwrap();
        assert_eq!(summary.pending_receipts, 0);
        assert_eq!(summary.committed_receipts, 1);
        assert_eq!(
            reopened.status(event_id).await.unwrap(),
            AuthorityAttestationStatus::ReceiptAttested
        );
    }

    #[tokio::test]
    async fn unattested_history_is_reported_not_laundered() {
        let event_key = SigningKey::from_bytes(&[11; 32]);
        let receipt_key = SigningKey::from_bytes(&[12; 32]);
        let event = event(&event_key);
        let event_id = event.envelope.event_id;
        let log = MemoryScientificEventLog::new();
        log.append_at(
            0,
            event,
            Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 1).unwrap(),
        )
        .await
        .unwrap();
        let audit = MemoryScientificAuthorityAuditStore::new(BTreeSet::from([receipt_key
            .verifying_key()
            .to_bytes()]));
        let summary = audit.reconcile(&log).await.unwrap();
        assert_eq!(summary.unattested_events, 1);
        assert_eq!(summary.legacy_unattested_events, 1);
        assert_eq!(summary.unsafe_unattested_events, 0);
        assert_eq!(summary.committed_receipts, 0);
        assert_eq!(
            audit.status(event_id).await.unwrap(),
            AuthorityAttestationStatus::LegacyUnattested
        );
    }

    #[tokio::test]
    async fn missing_receipt_after_cutover_is_unsafe_not_waivable_legacy() {
        let event_key = SigningKey::from_bytes(&[21; 32]);
        let receipt_key = SigningKey::from_bytes(&[22; 32]);
        let genesis = event(&event_key);
        let first = evidence_event(&genesis, &event_key, 1, 1);
        let second = evidence_event(&first, &event_key, 2, 2);
        let genesis_id = genesis.envelope.event_id;
        let second_id = second.envelope.event_id;
        let log = MemoryScientificEventLog::new();
        log.append_at(
            0,
            genesis,
            Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 1).unwrap(),
        )
        .await
        .unwrap();
        log.append_at(
            1,
            first.clone(),
            Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 2).unwrap(),
        )
        .await
        .unwrap();
        log.append_at(
            2,
            second,
            Utc.with_ymd_and_hms(2026, 8, 4, 12, 0, 3).unwrap(),
        )
        .await
        .unwrap();

        let audit = MemoryScientificAuthorityAuditStore::new(BTreeSet::from([receipt_key
            .verifying_key()
            .to_bytes()]));
        let receipt =
            signed_receipt_with_chain(&first, &receipt_key, None, first.envelope.previous_hash);
        audit.prepare(receipt).await.unwrap();
        audit.commit(first.envelope.event_id).await.unwrap();
        let summary = audit.reconcile(&log).await.unwrap();

        assert_eq!(summary.legacy_unattested_events, 1);
        assert_eq!(summary.unsafe_unattested_events, 1);
        assert_eq!(
            audit.status(genesis_id).await.unwrap(),
            AuthorityAttestationStatus::LegacyUnattested
        );
        assert_eq!(
            audit.status(second_id).await.unwrap(),
            AuthorityAttestationStatus::UnsafeUnattested
        );
    }

    #[test]
    fn receipt_service_key_must_be_distinct_from_event_signer() {
        let event_key = SigningKey::from_bytes(&[31; 32]);
        let event = event(&event_key);
        let receipt = signed_receipt(&event, &event_key);
        let trusted = BTreeSet::from([event_key.verifying_key().to_bytes()]);
        assert!(receipt.verify(&trusted).is_err());
    }

    #[tokio::test]
    async fn receipt_chain_rejects_parallel_forks_at_the_same_sequence() {
        let event_key = SigningKey::from_bytes(&[41; 32]);
        let receipt_key = SigningKey::from_bytes(&[42; 32]);
        let genesis = event(&event_key);
        let receipt_a = signed_receipt(&genesis, &receipt_key);
        let mut receipt_b = signed_receipt(&genesis, &receipt_key);
        receipt_b.receipt.event_id = ScientificEventId::new();
        receipt_b =
            SignedScientificAuthorityReceipt::sign(receipt_b.receipt, &receipt_key).unwrap();
        let audit = MemoryScientificAuthorityAuditStore::new(BTreeSet::from([receipt_key
            .verifying_key()
            .to_bytes()]));
        audit.prepare(receipt_a).await.unwrap();
        assert!(audit.prepare(receipt_b).await.is_err());
    }
}
