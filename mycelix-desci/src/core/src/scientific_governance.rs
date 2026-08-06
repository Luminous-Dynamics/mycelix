// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Identity binding and scientific authorization for canonical event streams.
//!
//! Cryptographic signature verification proves that a key signed an event. It
//! does not prove that the key is authorized for the actor named by the event,
//! or that the actor may perform the requested scientific action. This module
//! supplies that missing authority boundary and wraps any [`ScientificEventLog`]
//! with fail-closed identity and policy checks.

use crate::scientific_authority_audit::{
    AuthorityAttestationStatus, ScientificAuthorityAuditStore, ScientificAuthorityReceipt,
    SignedScientificAuthorityReceipt,
};
use crate::scientific_events::{
    ActorId, AppendReceipt, AttestationKind, ClaimProjection, ContentHash, EventPage,
    OrganizationId, ScientificEventId, ScientificEventLog, ScientificEventPayload,
    SignedScientificEvent, StreamHead,
};
use crate::{Error, Result};
use async_trait::async_trait;
use chrono::{DateTime, Utc};
use ed25519_dalek::SigningKey;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeSet, HashMap};
use std::sync::Arc;
use tokio::sync::RwLock;

/// A public key authorization with explicit validity and revocation bounds.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuthorizedActorKey {
    pub public_key: [u8; 32],
    pub valid_from: DateTime<Utc>,
    pub valid_until: Option<DateTime<Utc>>,
    pub revoked_at: Option<DateTime<Utc>>,
}

impl AuthorizedActorKey {
    pub fn validate(&self) -> Result<()> {
        if self
            .valid_until
            .as_ref()
            .is_some_and(|valid_until| valid_until <= &self.valid_from)
        {
            return Err(Error::Validation(
                "authorized key valid_until must be after valid_from".to_string(),
            ));
        }
        if self
            .revoked_at
            .as_ref()
            .is_some_and(|revoked_at| revoked_at < &self.valid_from)
        {
            return Err(Error::Validation(
                "authorized key cannot be revoked before valid_from".to_string(),
            ));
        }
        Ok(())
    }

    pub fn is_active_at(&self, at: &DateTime<Utc>) -> bool {
        if at < &self.valid_from {
            return false;
        }
        if self
            .valid_until
            .as_ref()
            .is_some_and(|valid_until| at >= valid_until)
        {
            return false;
        }
        if self
            .revoked_at
            .as_ref()
            .is_some_and(|revoked_at| at >= revoked_at)
        {
            return false;
        }
        true
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScientificRole {
    Contributor,
    Reviewer,
    Editor,
    Institution,
    Service,
    /// Offline or tightly governed authority allowed to import historical
    /// mutable records as explicitly unassessed legacy events.
    MigrationService,
    /// Administrator of the append-only scientific credential registry.
    RegistryAdmin,
}

impl ScientificRole {
    pub const fn code(self) -> u8 {
        match self {
            Self::Contributor => 1,
            Self::Reviewer => 2,
            Self::Editor => 3,
            Self::Institution => 4,
            Self::Service => 5,
            Self::MigrationService => 6,
            Self::RegistryAdmin => 7,
        }
    }
}

/// Resolved durable identity context used by authorization policy.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResolvedScientificActor {
    pub actor: ActorId,
    pub authorized_keys: Vec<AuthorizedActorKey>,
    pub organizations: BTreeSet<OrganizationId>,
    pub roles: BTreeSet<ScientificRole>,
    /// Hash of the credential-registry revision used to resolve this actor.
    /// Legacy startup profiles leave this unset.
    #[serde(default)]
    pub authority_revision: Option<ContentHash>,
}

impl ResolvedScientificActor {
    pub fn validate(&self) -> Result<()> {
        self.actor.validate()?;
        for organization in &self.organizations {
            organization.validate()?;
        }
        if self.authorized_keys.is_empty() {
            return Err(Error::Validation(
                "resolved scientific actor requires at least one authorized key".to_string(),
            ));
        }
        if self.roles.is_empty() {
            return Err(Error::Validation(
                "resolved scientific actor requires at least one scientific role".to_string(),
            ));
        }
        let mut unique_keys = BTreeSet::new();
        for key in &self.authorized_keys {
            key.validate()?;
            if !unique_keys.insert(key.public_key) {
                return Err(Error::Validation(
                    "resolved scientific actor contains a duplicate authorized key".to_string(),
                ));
            }
        }
        Ok(())
    }

    pub fn active_key(&self, key: &[u8; 32], at: &DateTime<Utc>) -> Option<&AuthorizedActorKey> {
        self.authorized_keys
            .iter()
            .find(|authorized| &authorized.public_key == key && authorized.is_active_at(at))
    }

    pub fn authorizes_key(&self, key: &[u8; 32], at: &DateTime<Utc>) -> bool {
        self.active_key(key, at).is_some()
    }

    pub fn has_role(&self, role: ScientificRole) -> bool {
        self.roles.contains(&role)
    }

    pub fn has_any_role(&self, roles: &[ScientificRole]) -> bool {
        roles.iter().any(|role| self.has_role(*role))
    }
}

#[async_trait]
pub trait ScientificIdentityResolver: Send + Sync {
    async fn resolve(
        &self,
        actor: &ActorId,
        at: &DateTime<Utc>,
    ) -> Result<Option<ResolvedScientificActor>>;
}

#[async_trait]
impl<T: ScientificIdentityResolver + ?Sized> ScientificIdentityResolver for Arc<T> {
    async fn resolve(
        &self,
        actor: &ActorId,
        at: &DateTime<Utc>,
    ) -> Result<Option<ResolvedScientificActor>> {
        (**self).resolve(actor, at).await
    }
}

/// Deterministic resolver for tests, local deployments, and identity-adapter
/// conformance. Production resolvers may bind DIDs, JWT subjects, ORCID OAuth,
/// institutional credentials, and key-status registries into the same result.
#[derive(Debug, Clone, Default)]
pub struct MemoryScientificIdentityResolver {
    actors: Arc<RwLock<HashMap<ActorId, ResolvedScientificActor>>>,
}

impl MemoryScientificIdentityResolver {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn register(&self, actor: ResolvedScientificActor) -> Result<()> {
        actor.validate()?;
        let mut actors = self.actors.write().await;
        if actors.contains_key(&actor.actor) {
            return Err(Error::Validation(format!(
                "scientific actor is already registered: {}",
                actor.actor
            )));
        }
        actors.insert(actor.actor.clone(), actor);
        Ok(())
    }

    pub async fn revoke_key(
        &self,
        actor: &ActorId,
        public_key: [u8; 32],
        revoked_at: DateTime<Utc>,
    ) -> Result<()> {
        let mut actors = self.actors.write().await;
        let actor = actors
            .get_mut(actor)
            .ok_or_else(|| Error::NotFound("scientific actor not found".to_string()))?;
        let key = actor
            .authorized_keys
            .iter_mut()
            .find(|key| key.public_key == public_key)
            .ok_or_else(|| Error::NotFound("authorized actor key not found".to_string()))?;
        if revoked_at < key.valid_from {
            return Err(Error::Validation(
                "authorized key cannot be revoked before valid_from".to_string(),
            ));
        }
        key.revoked_at = Some(revoked_at);
        Ok(())
    }
}

#[async_trait]
impl ScientificIdentityResolver for MemoryScientificIdentityResolver {
    async fn resolve(
        &self,
        actor: &ActorId,
        _at: &DateTime<Utc>,
    ) -> Result<Option<ResolvedScientificActor>> {
        Ok(self.actors.read().await.get(actor).cloned())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScientificAction {
    ProposeClaim,
    ImportLegacyClaim,
    AttachEvidence,
    RecordAttestation,
    CorrectAttestation,
    WithdrawAttestation,
    SupersedeClaim,
    RetractClaim,
}

impl ScientificAction {
    pub const fn code(self) -> u8 {
        match self {
            Self::ProposeClaim => 1,
            Self::ImportLegacyClaim => 2,
            Self::AttachEvidence => 3,
            Self::RecordAttestation => 4,
            Self::CorrectAttestation => 5,
            Self::WithdrawAttestation => 6,
            Self::SupersedeClaim => 7,
            Self::RetractClaim => 8,
        }
    }

    pub fn from_payload(payload: &ScientificEventPayload) -> Self {
        match payload {
            ScientificEventPayload::ClaimProposed { .. } => Self::ProposeClaim,
            ScientificEventPayload::LegacyClaimImported { .. } => Self::ImportLegacyClaim,
            ScientificEventPayload::EvidenceAttached { .. } => Self::AttachEvidence,
            ScientificEventPayload::AttestationRecorded { .. } => Self::RecordAttestation,
            ScientificEventPayload::AttestationCorrected { .. } => Self::CorrectAttestation,
            ScientificEventPayload::AttestationWithdrawn { .. } => Self::WithdrawAttestation,
            ScientificEventPayload::ClaimSuperseded { .. } => Self::SupersedeClaim,
            ScientificEventPayload::ClaimRetracted { .. } => Self::RetractClaim,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizationDecision {
    pub allowed: bool,
    pub reason: String,
}

impl AuthorizationDecision {
    pub fn allow(reason: impl Into<String>) -> Self {
        Self {
            allowed: true,
            reason: reason.into(),
        }
    }

    pub fn deny(reason: impl Into<String>) -> Self {
        Self {
            allowed: false,
            reason: reason.into(),
        }
    }
}

pub trait ScientificAuthorizationPolicy: Send + Sync {
    fn policy_id(&self) -> &'static str;
    fn policy_version(&self) -> &'static str;

    fn authorize(
        &self,
        actor: &ResolvedScientificActor,
        event: &SignedScientificEvent,
        current: Option<&ClaimProjection>,
    ) -> AuthorizationDecision;
}

/// Conservative default policy. It intentionally distinguishes cryptographic
/// validity from authority and never treats self-asserted independent evidence
/// as independent.
#[derive(Debug, Clone, Default)]
pub struct DefaultScientificAuthorizationPolicy;

impl DefaultScientificAuthorizationPolicy {
    fn is_editor(actor: &ResolvedScientificActor) -> bool {
        actor.has_any_role(&[ScientificRole::Editor, ScientificRole::Institution])
    }
}

impl ScientificAuthorizationPolicy for DefaultScientificAuthorizationPolicy {
    fn policy_id(&self) -> &'static str {
        "mycelix-default-scientific-authorization"
    }

    fn policy_version(&self) -> &'static str {
        "1.0.0"
    }

    fn authorize(
        &self,
        actor: &ResolvedScientificActor,
        event: &SignedScientificEvent,
        current: Option<&ClaimProjection>,
    ) -> AuthorizationDecision {
        let action = ScientificAction::from_payload(&event.envelope.payload);
        match (&event.envelope.payload, current) {
            (ScientificEventPayload::ClaimProposed { .. }, None) => {
                if actor.has_any_role(&[
                    ScientificRole::Contributor,
                    ScientificRole::Editor,
                    ScientificRole::Institution,
                ]) {
                    AuthorizationDecision::allow("authenticated contributor may propose a claim")
                } else {
                    AuthorizationDecision::deny("actor lacks a claim-proposal role")
                }
            }
            (ScientificEventPayload::ClaimProposed { .. }, Some(_)) => {
                AuthorizationDecision::deny("claim_proposed is valid only for an empty stream")
            }
            (ScientificEventPayload::LegacyClaimImported { .. }, None) => {
                if actor.has_role(ScientificRole::MigrationService) {
                    AuthorizationDecision::allow(
                        "designated migration service may import unassessed legacy history",
                    )
                } else {
                    AuthorizationDecision::deny("legacy imports require the migration_service role")
                }
            }
            (ScientificEventPayload::LegacyClaimImported { .. }, Some(_)) => {
                AuthorizationDecision::deny(
                    "legacy_claim_imported is valid only for an empty stream",
                )
            }
            (_, None) => {
                AuthorizationDecision::deny(format!("{action:?} requires an existing claim stream"))
            }
            (ScientificEventPayload::EvidenceAttached { .. }, Some(projection)) => {
                if actor.actor == projection.creator || Self::is_editor(actor) {
                    AuthorizationDecision::allow("claim owner or editor may attach evidence")
                } else {
                    AuthorizationDecision::deny(
                        "only the claim owner or governed editor may attach evidence",
                    )
                }
            }
            (ScientificEventPayload::AttestationRecorded { attestation }, Some(projection)) => {
                if !actor.has_any_role(&[
                    ScientificRole::Reviewer,
                    ScientificRole::Contributor,
                    ScientificRole::Institution,
                ]) {
                    return AuthorizationDecision::deny(
                        "actor lacks a scientific attestation role",
                    );
                }
                if matches!(
                    &attestation.kind,
                    AttestationKind::IndependentReplication { .. }
                ) {
                    if actor.actor == projection.creator {
                        return AuthorizationDecision::deny(
                            "claim creator cannot authorize their own independent replication",
                        );
                    }
                    if event.envelope.acting_organization.is_some()
                        && event.envelope.acting_organization.as_ref()
                            == projection.creator_organization.as_ref()
                    {
                        return AuthorizationDecision::deny(
                            "creator organization cannot count as an independent replication",
                        );
                    }
                }
                AuthorizationDecision::allow("qualified actor may submit an attestation")
            }
            (
                ScientificEventPayload::AttestationCorrected { attestation_id, .. }
                | ScientificEventPayload::AttestationWithdrawn { attestation_id, .. },
                Some(projection),
            ) => match projection.attestation(*attestation_id) {
                Some(record) if record.actor == actor.actor => AuthorizationDecision::allow(
                    "attestation author may correct or withdraw their own attestation",
                ),
                Some(_) if Self::is_editor(actor) => AuthorizationDecision::allow(
                    "governed editor may correct or withdraw an attestation",
                ),
                Some(_) => {
                    AuthorizationDecision::deny("actor cannot modify another actor's attestation")
                }
                None => AuthorizationDecision::deny("attestation does not exist"),
            },
            (
                ScientificEventPayload::ClaimSuperseded { .. }
                | ScientificEventPayload::ClaimRetracted { .. },
                Some(projection),
            ) => {
                if actor.actor == projection.creator || Self::is_editor(actor) {
                    AuthorizationDecision::allow(
                        "claim owner or governed editor may change claim lifecycle",
                    )
                } else {
                    AuthorizationDecision::deny(
                        "actor cannot supersede or retract another actor's claim",
                    )
                }
            }
        }
    }
}

/// Optional storage capability that commits a canonical scientific event, its
/// signed authority receipt, and any publication record as one durable unit.
/// Multi-process production backends should implement this rather than relying
/// on the file journal's prepare/append/finalize recovery sequence.
#[async_trait]
pub trait AtomicScientificCommitStore: Send + Sync {
    async fn commit_scientific_event(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        receipt: SignedScientificAuthorityReceipt,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt>;
}

#[async_trait]
impl<T> AtomicScientificCommitStore for Arc<T>
where
    T: AtomicScientificCommitStore + ?Sized,
{
    async fn commit_scientific_event(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        receipt: SignedScientificAuthorityReceipt,
        received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        (**self)
            .commit_scientific_event(expected_sequence, event, receipt, received_at)
            .await
    }
}

/// Authoritative wrapper that binds event keys to durable actors, validates
/// acting-organization membership, applies scientific authorization, and only
/// then delegates to the append-only store.
#[derive(Clone)]
pub struct GovernedScientificEventLog<L, R, P> {
    inner: L,
    resolver: R,
    policy: P,
    authority_audit: Arc<dyn ScientificAuthorityAuditStore>,
    receipt_signing_key: Option<Arc<SigningKey>>,
    atomic_committer: Option<Arc<dyn AtomicScientificCommitStore>>,
}

impl<L, R, P> GovernedScientificEventLog<L, R, P> {
    pub fn new(
        inner: L,
        resolver: R,
        policy: P,
        authority_audit: Arc<dyn ScientificAuthorityAuditStore>,
        receipt_signing_key: Option<Arc<SigningKey>>,
    ) -> Self {
        Self {
            inner,
            resolver,
            policy,
            authority_audit,
            receipt_signing_key,
            atomic_committer: None,
        }
    }

    pub fn with_atomic_committer(
        mut self,
        atomic_committer: Arc<dyn AtomicScientificCommitStore>,
    ) -> Self {
        self.atomic_committer = Some(atomic_committer);
        self
    }

    pub fn inner(&self) -> &L {
        &self.inner
    }
}

#[async_trait]
impl<L, R, P> ScientificEventLog for GovernedScientificEventLog<L, R, P>
where
    L: ScientificEventLog,
    R: ScientificIdentityResolver,
    P: ScientificAuthorizationPolicy,
{
    async fn append_at(
        &self,
        expected_sequence: u64,
        event: SignedScientificEvent,
        _received_at: DateTime<Utc>,
    ) -> Result<AppendReceipt> {
        // Receipt time is an observation made by the governed service. Never
        // trust a library caller to backdate authority evaluation through the
        // storage-oriented `append_at` trait method.
        let received_at = Utc::now();
        event.verify()?;

        // An exact replay of an already committed signed event is a read of
        // the prior result, not a new scientific mutation. Handle it before
        // current-role evaluation so later key rotation or revocation cannot
        // turn a safely retryable request into a second write or a misleading
        // authorization failure.
        if let Some(existing) = self.inner.event_by_id(event.envelope.event_id).await? {
            if existing != event {
                return Err(Error::Storage(
                    "duplicate scientific event id identifies different signed bytes".to_string(),
                ));
            }
            if self.authority_audit.status(event.envelope.event_id).await?
                != AuthorityAttestationStatus::ReceiptAttested
            {
                return Err(Error::Storage(
                    "existing scientific event is not backed by a committed authority receipt"
                        .to_string(),
                ));
            }
            let authority_receipt = self
                .authority_audit
                .receipt(event.envelope.event_id)
                .await?
                .ok_or_else(|| {
                    Error::Storage(
                        "committed authority status has no retrievable receipt".to_string(),
                    )
                })?;
            return Ok(AppendReceipt {
                stream_id: event.envelope.stream_id,
                sequence: event.envelope.sequence,
                event_id: event.envelope.event_id,
                event_hash: event.event_hash()?,
                received_at: authority_receipt.receipt.received_at,
            });
        }

        let resolved = self
            .resolver
            .resolve(&event.envelope.actor, &received_at)
            .await?
            .ok_or_else(|| {
                Error::VerificationFailed("scientific actor is unresolved".to_string())
            })?;

        if resolved.actor != event.envelope.actor {
            return Err(Error::VerificationFailed(
                "identity resolver returned a mismatched actor".to_string(),
            ));
        }
        let authorized_key = resolved
            .active_key(&event.signer_public_key, &received_at)
            .cloned()
            .ok_or_else(|| {
                Error::VerificationFailed(
                    "event signing key is not active for the claimed actor".to_string(),
                )
            })?;
        if let Some(organization) = &event.envelope.acting_organization {
            if !resolved.organizations.contains(organization) {
                return Err(Error::VerificationFailed(
                    "actor is not authorized for the claimed acting organization".to_string(),
                ));
            }
        }

        let stream = self.inner.stream(event.envelope.stream_id).await?;
        let current = if stream.is_empty() {
            None
        } else {
            Some(ClaimProjection::rebuild(&stream)?)
        };
        let decision = self.policy.authorize(&resolved, &event, current.as_ref());
        if !decision.allowed {
            return Err(Error::VerificationFailed(format!(
                "scientific action denied: {}",
                decision.reason
            )));
        }

        let receipt_signing_key = self.receipt_signing_key.as_ref().ok_or_else(|| {
            Error::VerificationFailed(
                "scientific authority receipt signing is not configured".to_string(),
            )
        })?;
        let prior_receipt = self
            .authority_audit
            .stream_head(event.envelope.stream_id)
            .await?;
        let (previous_receipt_hash, legacy_cutover_anchor) = match prior_receipt {
            Some(prior) => {
                if prior.receipt.sequence + 1 != event.envelope.sequence {
                    return Err(Error::VerificationFailed(
                        "authority receipt chain does not immediately precede the event"
                            .to_string(),
                    ));
                }
                (Some(prior.receipt_hash()?), None)
            }
            None if event.envelope.sequence == 0 => (None, None),
            None => (None, event.envelope.previous_hash),
        };
        let authority_receipt = ScientificAuthorityReceipt::new(
            &event,
            received_at.clone(),
            authorized_key,
            resolved.organizations.clone(),
            resolved.roles.clone(),
            resolved.authority_revision,
            ScientificAction::from_payload(&event.envelope.payload),
            self.policy.policy_id(),
            self.policy.policy_version(),
            decision.reason.clone(),
            previous_receipt_hash,
            legacy_cutover_anchor,
        )?;
        let signed_receipt = SignedScientificAuthorityReceipt::sign(
            authority_receipt,
            receipt_signing_key.as_ref(),
        )?;
        if let Some(committer) = &self.atomic_committer {
            return committer
                .commit_scientific_event(expected_sequence, event, signed_receipt, received_at)
                .await;
        }

        let event_id = event.envelope.event_id;
        self.authority_audit.prepare(signed_receipt).await?;
        let append_result = self
            .inner
            .append_at(expected_sequence, event, received_at)
            .await;
        let append_receipt = match append_result {
            Ok(receipt) => receipt,
            Err(error) => {
                let _ = self.authority_audit.abort(event_id).await;
                return Err(error);
            }
        };
        self.authority_audit
            .commit(event_id)
            .await
            .map_err(|error| {
                Error::Storage(format!(
                "scientific event committed but authority receipt finalization is pending: {error}"
            ))
            })?;
        Ok(append_receipt)
    }

    async fn head(
        &self,
        claim_id: crate::scientific_events::ClaimId,
    ) -> Result<Option<StreamHead>> {
        self.inner.head(claim_id).await
    }

    async fn read(
        &self,
        claim_id: crate::scientific_events::ClaimId,
        from_sequence: u64,
        limit: usize,
    ) -> Result<EventPage> {
        self.inner.read(claim_id, from_sequence, limit).await
    }

    async fn stream(
        &self,
        claim_id: crate::scientific_events::ClaimId,
    ) -> Result<Vec<SignedScientificEvent>> {
        self.inner.stream(claim_id).await
    }

    async fn stream_ids(&self) -> Result<Vec<crate::scientific_events::ClaimId>> {
        self.inner.stream_ids().await
    }

    async fn event_by_id(
        &self,
        event_id: ScientificEventId,
    ) -> Result<Option<SignedScientificEvent>> {
        self.inner.event_by_id(event_id).await
    }

    async fn event_by_hash(&self, hash: ContentHash) -> Result<Option<SignedScientificEvent>> {
        self.inner.event_by_hash(hash).await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scientific_authority_audit::MemoryScientificAuthorityAuditStore;
    use crate::scientific_events::{
        AtomicClaim, ClaimId, MemoryScientificEventLog, ResearchObject, ResearchObjectId,
        ResearchObjectType, ScientificEventEnvelope,
    };
    use chrono::{Duration, TimeZone};
    use ed25519_dalek::SigningKey;
    use std::sync::Arc;

    fn actor(value: &str) -> ActorId {
        ActorId::new(value).unwrap()
    }

    fn key(seed: u8) -> SigningKey {
        SigningKey::from_bytes(&[seed; 32])
    }

    fn governed(
        resolver: MemoryScientificIdentityResolver,
    ) -> GovernedScientificEventLog<
        MemoryScientificEventLog,
        MemoryScientificIdentityResolver,
        DefaultScientificAuthorizationPolicy,
    > {
        let receipt_key = Arc::new(key(250));
        let audit = Arc::new(MemoryScientificAuthorityAuditStore::new(BTreeSet::from([
            receipt_key.verifying_key().to_bytes(),
        ])));
        GovernedScientificEventLog::new(
            MemoryScientificEventLog::new(),
            resolver,
            DefaultScientificAuthorizationPolicy,
            audit,
            Some(receipt_key),
        )
    }

    fn proposed(claim_id: ClaimId) -> ScientificEventPayload {
        let object_id = ResearchObjectId::new();
        ScientificEventPayload::ClaimProposed {
            research_object: ResearchObject {
                id: object_id,
                title: "Bound identity test".to_string(),
                object_type: ResearchObjectType::Manuscript,
                persistent_identifier: None,
            },
            claim: AtomicClaim {
                id: claim_id,
                research_object_id: object_id,
                statement: "A signed claim".to_string(),
                scope: None,
            },
        }
    }

    fn profile(
        actor_id: ActorId,
        signing_key: &SigningKey,
        role: ScientificRole,
        now: &DateTime<Utc>,
    ) -> ResolvedScientificActor {
        ResolvedScientificActor {
            actor: actor_id,
            authorized_keys: vec![AuthorizedActorKey {
                public_key: signing_key.verifying_key().to_bytes(),
                valid_from: now.clone() - Duration::days(1),
                valid_until: None,
                revoked_at: None,
            }],
            organizations: BTreeSet::new(),
            roles: BTreeSet::from([role]),
            authority_revision: None,
        }
    }

    #[tokio::test]
    async fn valid_signature_with_unbound_key_is_rejected() {
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let alice = actor("did:key:alice");
        let alice_key = key(1);
        let mallory_key = key(9);
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                alice.clone(),
                &alice_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        let log = governed(resolver);
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(alice, now.clone(), proposed(ClaimId::new())).unwrap(),
            &mallory_key,
        )
        .unwrap();
        assert!(log.append_at(0, event, now.clone()).await.is_err());
    }

    #[tokio::test]
    async fn revoked_key_cannot_be_revived_by_backdating_receipt_time() {
        let now = Utc::now();
        let alice = actor("did:key:alice");
        let alice_key = key(1);
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                alice.clone(),
                &alice_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        resolver
            .revoke_key(
                &alice,
                alice_key.verifying_key().to_bytes(),
                now.clone() - Duration::seconds(1),
            )
            .await
            .unwrap();
        let log = governed(resolver);
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                alice,
                now.clone() - Duration::minutes(1),
                proposed(ClaimId::new()),
            )
            .unwrap(),
            &alice_key,
        )
        .unwrap();
        let forged_pre_revocation_receipt_time = now - Duration::hours(1);
        assert!(
            log.append_at(0, event, forged_pre_revocation_receipt_time)
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn arbitrary_actor_cannot_retract_another_claim() {
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let alice = actor("did:key:alice");
        let bob = actor("did:key:bob");
        let alice_key = key(1);
        let bob_key = key(2);
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                alice.clone(),
                &alice_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        resolver
            .register(profile(
                bob.clone(),
                &bob_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        let log = governed(resolver);
        let claim_id = ClaimId::new();
        let genesis = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(alice, now.clone(), proposed(claim_id)).unwrap(),
            &alice_key,
        )
        .unwrap();
        log.append_at(0, genesis.clone(), now.clone())
            .await
            .unwrap();

        let retraction = SignedScientificEvent::sign(
            ScientificEventEnvelope::next(
                &genesis,
                bob,
                now.clone() + Duration::seconds(1),
                ScientificEventPayload::ClaimRetracted {
                    claim_id,
                    reason: "Unauthorized".to_string(),
                },
            )
            .unwrap(),
            &bob_key,
        )
        .unwrap();
        assert!(
            log.append_at(1, retraction, now.clone() + Duration::seconds(1))
                .await
                .is_err()
        );
    }

    #[tokio::test]
    async fn successful_append_finalizes_receipt_time_authority_evidence() {
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let alice = actor("did:key:alice");
        let alice_key = key(17);
        let receipt_key = Arc::new(key(18));
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                alice.clone(),
                &alice_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        let audit = Arc::new(MemoryScientificAuthorityAuditStore::new(BTreeSet::from([
            receipt_key.verifying_key().to_bytes(),
        ])));
        let log = GovernedScientificEventLog::new(
            MemoryScientificEventLog::new(),
            resolver,
            DefaultScientificAuthorizationPolicy,
            audit.clone(),
            Some(receipt_key),
        );
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(alice, now.clone(), proposed(ClaimId::new())).unwrap(),
            &alice_key,
        )
        .unwrap();
        let event_id = event.envelope.event_id;

        log.append_at(0, event, now).await.unwrap();

        assert_eq!(
            audit.status(event_id).await.unwrap(),
            AuthorityAttestationStatus::ReceiptAttested
        );
        assert!(audit.receipt(event_id).await.unwrap().is_some());
    }

    #[tokio::test]
    async fn exact_signed_event_retry_returns_the_original_append_receipt() {
        let now = Utc::now();
        let alice = actor("did:key:retry-alice");
        let alice_key = key(19);
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                alice.clone(),
                &alice_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        let log = governed(resolver);
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(alice, now, proposed(ClaimId::new())).unwrap(),
            &alice_key,
        )
        .unwrap();

        let first = log.append_at(0, event.clone(), Utc::now()).await.unwrap();
        let retried = log.append_at(0, event, Utc::now()).await.unwrap();

        assert_eq!(retried, first);
        assert_eq!(log.stream(first.stream_id).await.unwrap().len(), 1);
    }

    #[tokio::test]
    async fn legacy_import_requires_migration_service_role() {
        let now = Utc.timestamp_opt(1_700_000_000, 0).unwrap();
        let importer = actor("did:key:importer");
        let importer_key = key(5);
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(profile(
                importer.clone(),
                &importer_key,
                ScientificRole::Contributor,
                &now,
            ))
            .await
            .unwrap();
        let log = governed(resolver);
        let claim_id = ClaimId::new();
        let object_id = ResearchObjectId::new();
        let event = SignedScientificEvent::sign(
            ScientificEventEnvelope::genesis(
                importer,
                now.clone(),
                ScientificEventPayload::LegacyClaimImported {
                    research_object: ResearchObject {
                        id: object_id,
                        title: "Legacy".to_string(),
                        object_type: ResearchObjectType::Other("legacy_claim_record".to_string()),
                        persistent_identifier: None,
                    },
                    claim: AtomicClaim {
                        id: claim_id,
                        research_object_id: object_id,
                        statement: "Historical claim".to_string(),
                        scope: None,
                    },
                    import: crate::scientific_events::LegacyImportMetadata {
                        source_system: "legacy-json".to_string(),
                        source_record_hash: ContentHash::digest(b"legacy"),
                        source_locator: None,
                        legacy_creator: Some("historical-author@example.org".to_string()),
                        legacy_tier: "E4".to_string(),
                        legacy_verification_count: 5,
                        legacy_provenance_count: 1,
                        omitted_fields: vec![],
                    },
                },
            )
            .unwrap(),
            &importer_key,
        )
        .unwrap();
        assert!(log.append_at(0, event, now).await.is_err());
    }
}
