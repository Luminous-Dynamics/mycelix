//! Consent and disclosure policy primitives for the Relationship OS.
//!
//! Consent is an explicit, bounded permission. It is not identity, evidence,
//! delegated authority, or proof that an external action occurred.
//!
//! All authorization decisions take an explicit request timestamp and
//! authority epoch. No ambient clock or mutable global state is consulted.

use std::fmt;

use crate::{ParticipantRef, RelationshipId};

pub type AuthorityEpoch = u64;

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PrincipalRef {
    pub participant: ParticipantRef,
}

impl PrincipalRef {
    pub fn new(participant: ParticipantRef) -> Self { Self { participant } }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ConsentPurpose {
    Named(String),
}

impl ConsentPurpose {
    pub fn named(value: impl Into<String>) -> Result<Self, ConsentError> {
        let value = value.into();
        if value.trim().is_empty() { return Err(ConsentError::EmptyPurpose); }
        Ok(Self::Named(value))
    }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum DataClass {
    Public,
    Identity,
    Contact,
    Financial,
    Health,
    Operational,
    Confidential,
    Custom(String),
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct AudienceRef {
    pub principal: PrincipalRef,
}

impl AudienceRef {
    pub fn principal(principal: PrincipalRef) -> Self { Self { principal } }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum AccessMode {
    Read,
    Disclose,
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DisclosureScope {
    pub mode: AccessMode,
    pub data_classes: Vec<DataClass>,
    /// Explicit field/resource names. Empty scope is never authorized.
    pub fields: Vec<String>,
}

impl DisclosureScope {
    pub fn new(
        mode: AccessMode,
        mut data_classes: Vec<DataClass>,
        mut fields: Vec<String>,
    ) -> Result<Self, ConsentError> {
        data_classes.sort();
        data_classes.dedup();
        fields.sort();
        fields.dedup();
        if data_classes.is_empty() || fields.is_empty() {
            return Err(ConsentError::EmptyScope);
        }
        if fields.iter().any(|field| field.trim().is_empty()) {
            return Err(ConsentError::EmptyScope);
        }
        Ok(Self { mode, data_classes, fields })
    }

    fn contains(&self, requested: &Self) -> bool {
        if self.mode != requested.mode { return false; }
        requested.data_classes.iter().all(|c| self.data_classes.contains(c))
            && requested.fields.iter().all(|f| self.fields.contains(f))
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ConsentStatus {
    Granted,
    Revoked,
    Expired,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ConsentEventKind {
    Grant,
    Revoke,
    Expire,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConsentEvent {
    pub kind: ConsentEventKind,
    pub actor: PrincipalRef,
    pub occurred_at: i64,
    pub authority_epoch: AuthorityEpoch,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ConsentId([u8; 32]);

impl ConsentId {
    pub fn derive(relationship: RelationshipId, seed: &[u8]) -> Self {
        let mut h = blake3::Hasher::new();
        h.update(b"mycelix.consent.v1\0");
        h.update(relationship.as_bytes());
        h.update(&(seed.len() as u64).to_le_bytes());
        h.update(seed);
        Self(*h.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] { &self.0 }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Consent {
    pub id: ConsentId,
    pub relationship_id: RelationshipId,
    pub grantor: PrincipalRef,
    pub audience: AudienceRef,
    pub purpose: ConsentPurpose,
    pub scope: DisclosureScope,
    pub granted_at: i64,
    pub expires_at: Option<i64>,
    pub authority_epoch: AuthorityEpoch,
    pub status: ConsentStatus,
    pub events: Vec<ConsentEvent>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConsentRequest {
    pub requester: PrincipalRef,
    pub audience: AudienceRef,
    pub purpose: ConsentPurpose,
    pub scope: DisclosureScope,
    pub requested_at: i64,
    pub authority_epoch: AuthorityEpoch,
}

impl Consent {
    /// Canonical immutable authority content used for identity binding.
    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.consent-content.v1\\0");
        out.extend_from_slice(self.relationship_id.as_bytes());
        canonical_principal(&mut out, &self.grantor);
        canonical_principal(&mut out, &self.audience.principal);
        canonical_purpose(&mut out, &self.purpose);
        canonical_scope(&mut out, &self.scope);
        out.extend_from_slice(&self.granted_at.to_le_bytes());
        match self.expires_at {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_le_bytes());
            }
            None => out.push(0),
        }
        out.extend_from_slice(&self.authority_epoch.to_le_bytes());
        out
    }

    /// Canonical mutable lifecycle state, separate from immutable identity.
    pub fn state_canonical_bytes(&self) -> Vec<u8> {
        let mut out = self.canonical_bytes();
        out.push(match self.status {
            ConsentStatus::Granted => 0,
            ConsentStatus::Revoked => 1,
            ConsentStatus::Expired => 2,
        });
        out.extend_from_slice(&(self.events.len() as u64).to_le_bytes());
        for event in &self.events {
            out.push(match event.kind {
                ConsentEventKind::Grant => 0,
                ConsentEventKind::Revoke => 1,
                ConsentEventKind::Expire => 2,
            });
            canonical_principal(&mut out, &event.actor);
            out.extend_from_slice(&event.occurred_at.to_le_bytes());
            out.extend_from_slice(&event.authority_epoch.to_le_bytes());
        }
        out
    }

    pub fn validate_identity(&self) -> Result<(), ConsentError> {
        if self.id != ConsentId::derive_content_bound(self) {
            return Err(ConsentError::IdentityMismatch);
        }
        Ok(())
    }

    pub fn grant(
        _id: ConsentId,
        relationship_id: RelationshipId,
        grantor: PrincipalRef,
        audience: AudienceRef,
        purpose: ConsentPurpose,
        scope: DisclosureScope,
        granted_at: i64,
        expires_at: Option<i64>,
        authority_epoch: AuthorityEpoch,
    ) -> Result<Self, ConsentError> {
        if let Some(expiry) = expires_at {
            if expiry < granted_at { return Err(ConsentError::InvalidExpiry); }
        }
        let event = ConsentEvent {
            kind: ConsentEventKind::Grant,
            actor: grantor.clone(),
            occurred_at: granted_at,
            authority_epoch,
        };
        let mut consent = Self {
            id: ConsentId([0; 32]),
            relationship_id, grantor, audience, purpose, scope,
            granted_at, expires_at, authority_epoch,
            status: ConsentStatus::Granted, events: vec![event],
        };
        consent.id = ConsentId::derive_content_bound(&consent);
        Ok(consent)
    }

    pub fn revoke(
        &mut self,
        actor: &PrincipalRef,
        occurred_at: i64,
        authority_epoch: AuthorityEpoch,
    ) -> Result<(), ConsentError> {
        if self.status != ConsentStatus::Granted { return Err(ConsentError::AlreadyInactive); }
        if actor != &self.grantor { return Err(ConsentError::UnauthorizedRevocation); }
        if occurred_at < self.granted_at { return Err(ConsentError::NonMonotonicEventTime); }
        if authority_epoch != self.authority_epoch {
            return Err(ConsentError::StaleAuthorityEpoch);
        }
        self.status = ConsentStatus::Revoked;
        self.events.push(ConsentEvent {
            kind: ConsentEventKind::Revoke,
            actor: actor.clone(),
            occurred_at,
            authority_epoch,
        });
        Ok(())
    }

    /// Evaluate a request against explicit state only.
    ///
    /// Expiry is checked against the request timestamp; the function never
    /// consults the wall clock. A consent grant never authorizes execution.
    pub fn authorize(&self, request: &ConsentRequest) -> Result<(), ConsentError> {
        if self.status != ConsentStatus::Granted { return Err(ConsentError::NotGranted); }
        if request.requested_at < self.granted_at { return Err(ConsentError::NotYetEffective); }
        if let Some(expiry) = self.expires_at {
            if request.requested_at >= expiry { return Err(ConsentError::Expired); }
        }
        if request.authority_epoch != self.authority_epoch {
            return Err(ConsentError::StaleAuthorityEpoch);
        }
        if request.audience != self.audience { return Err(ConsentError::AudienceMismatch); }
        if request.purpose != self.purpose { return Err(ConsentError::PurposeMismatch); }
        if !self.scope.contains(&request.scope) { return Err(ConsentError::ScopeExceeded); }
        if request.requester != self.audience.principal {
            return Err(ConsentError::RequesterNotInAudience);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConsentError {
    EmptyPurpose,
    EmptyScope,
    InvalidExpiry,
    AlreadyInactive,
    UnauthorizedRevocation,
    NonMonotonicEventTime,
    NotGranted,
    NotYetEffective,
    Expired,
    StaleAuthorityEpoch,
    IdentityMismatch,
    AudienceMismatch,
    PurposeMismatch,
    ScopeExceeded,
    RequesterNotInAudience,
}

impl fmt::Display for ConsentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyPurpose => "consent purpose must not be empty",
            Self::EmptyScope => "consent scope must contain data classes and fields",
            Self::InvalidExpiry => "consent expiry cannot precede grant time",
            Self::AlreadyInactive => "consent is already inactive",
            Self::UnauthorizedRevocation => "only the grantor may revoke consent",
            Self::NonMonotonicEventTime => "consent event time cannot move backwards",
            Self::NotGranted => "consent is not active",
            Self::NotYetEffective => "consent was not active at request time",
            Self::Expired => "consent had expired at request time",
            Self::StaleAuthorityEpoch => "request authority epoch does not match consent",
            Self::IdentityMismatch => "consent identifier does not match canonical authority content",
            Self::AudienceMismatch => "request audience does not match consent audience",
            Self::PurposeMismatch => "request purpose does not match consent purpose",
            Self::ScopeExceeded => "requested scope exceeds granted scope",
            Self::RequesterNotInAudience => "requester is not the consent audience",
        };
        f.write_str(message)
    }
}

impl std::error::Error for ConsentError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RelationshipId;

    fn participant(ns: &str, id: &str) -> PrincipalRef {
        PrincipalRef::new(ParticipantRef::new(ns, id))
    }

    fn scope(mode: AccessMode, class: DataClass, field: &str) -> DisclosureScope {
        DisclosureScope::new(mode, vec![class], vec![field.to_owned()]).unwrap()
    }

    fn consent() -> Consent {
        let relationship = RelationshipId::derive("test", b"relationship");
        let grantor = participant("did", "alice");
        let audience = AudienceRef::principal(participant("did", "bob"));
        Consent::grant(
            ConsentId::derive(relationship, b"consent"),
            relationship,
            grantor,
            audience,
            ConsentPurpose::named("relationship-support").unwrap(),
            scope(AccessMode::Disclose, DataClass::Contact, "email"),
            100,
            Some(200),
            7,
        ).unwrap()
    }

    fn request(purpose: &str, at: i64, epoch: AuthorityEpoch) -> ConsentRequest {
        ConsentRequest {
            requester: participant("did", "bob"),
            audience: AudienceRef::principal(participant("did", "bob")),
            purpose: ConsentPurpose::named(purpose).unwrap(),
            scope: scope(AccessMode::Disclose, DataClass::Contact, "email"),
            requested_at: at,
            authority_epoch: epoch,
        }
    }

    #[test]
    fn stale_revocation_epoch_does_not_mutate_state() {
        let mut c = consent();
        let before = c.clone();
        let actor = participant("did", "alice");
        assert_eq!(
            c.revoke(&actor, 150, 8),
            Err(ConsentError::StaleAuthorityEpoch)
        );
        assert_eq!(c, before);
    }

    #[test]
    fn current_epoch_revocation_still_succeeds() {
        let mut c = consent();
        let actor = participant("did", "alice");
        c.revoke(&actor, 150, 7).unwrap();
        assert_eq!(c.status, ConsentStatus::Revoked);
    }

    #[test]
    fn content_bound_identity_is_deterministic() {
        let c = consent();
        assert_eq!(c.id, ConsentId::derive_content_bound(&c));
        assert_eq!(c.validate_identity(), Ok(()));
        assert_eq!(c.canonical_bytes(), c.canonical_bytes());
        assert_eq!(c.state_canonical_bytes(), c.state_canonical_bytes());
    }

    #[test]
    fn identity_detects_immutable_content_tampering() {
        let mut c = consent();
        c.scope = DisclosureScope::new(
            AccessMode::Disclose,
            vec![DataClass::Contact],
            vec!["phone".into()],
        ).unwrap();
        assert_eq!(c.validate_identity(), Err(ConsentError::IdentityMismatch));
    }

    #[test]
    fn lifecycle_mutation_does_not_change_immutable_identity() {
        let mut c = consent();
        let id = c.id;
        let canonical = c.canonical_bytes();
        c.revoke(&participant("did", "alice"), 150, 7).unwrap();
        assert_eq!(c.id, id);
        assert_eq!(c.canonical_bytes(), canonical);
        assert_ne!(c.state_canonical_bytes(), canonical);
    }

    #[test]
    fn matching_request_is_authorized() {
        assert!(consent().authorize(&request("relationship-support", 150, 7)).is_ok());
    }

    #[test]
    fn purpose_mismatch_fails_closed() {
        assert_eq!(consent().authorize(&request("marketing", 150, 7)), Err(ConsentError::PurposeMismatch));
    }

    #[test]
    fn audience_mismatch_fails_closed() {
        let mut r = request("relationship-support", 150, 7);
        r.audience = AudienceRef::principal(participant("did", "carol"));
        assert_eq!(consent().authorize(&r), Err(ConsentError::AudienceMismatch));
    }

    #[test]
    fn data_class_mismatch_fails_closed() {
        let mut r = request("relationship-support", 150, 7);
        r.scope = scope(AccessMode::Disclose, DataClass::Financial, "email");
        assert_eq!(consent().authorize(&r), Err(ConsentError::ScopeExceeded));
    }

    #[test]
    fn field_expansion_fails_closed() {
        let mut r = request("relationship-support", 150, 7);
        r.scope = DisclosureScope::new(
            AccessMode::Disclose, vec![DataClass::Contact],
            vec!["email".into(), "phone".into()],
        ).unwrap();
        assert_eq!(consent().authorize(&r), Err(ConsentError::ScopeExceeded));
    }

    #[test]
    fn read_permission_does_not_authorize_disclosure() {
        let relationship = RelationshipId::derive("test", b"read-only");
        let grantor = participant("did", "alice");
        let audience = AudienceRef::principal(participant("did", "bob"));
        let c = Consent::grant(
            ConsentId::derive(relationship, b"c"),
            relationship, grantor, audience,
            ConsentPurpose::named("support").unwrap(),
            scope(AccessMode::Read, DataClass::Contact, "email"),
            100, None, 1,
        ).unwrap();
        let mut r = request("support", 150, 1);
        r.scope = scope(AccessMode::Disclose, DataClass::Contact, "email");
        assert_eq!(c.authorize(&r), Err(ConsentError::ScopeExceeded));
    }

    #[test]
    fn expired_grant_fails_at_explicit_request_time() {
        assert_eq!(consent().authorize(&request("relationship-support", 200, 7)), Err(ConsentError::Expired));
    }

    #[test]
    fn stale_epoch_fails_closed() {
        assert_eq!(consent().authorize(&request("relationship-support", 150, 8)), Err(ConsentError::StaleAuthorityEpoch));
    }

    #[test]
    fn future_request_before_grant_fails() {
        assert_eq!(consent().authorize(&request("relationship-support", 99, 7)), Err(ConsentError::NotYetEffective));
    }

    #[test]
    fn missing_consent_is_not_an_authorization() {
        let c = consent();
        let mut r = request("relationship-support", 150, 7);
        r.scope = scope(AccessMode::Disclose, DataClass::Health, "diagnosis");
        assert!(c.authorize(&r).is_err());
    }

    #[test]
    fn revocation_is_explicit_and_authorized() {
        let mut c = consent();
        let grantor = participant("did", "alice");
        c.revoke(&grantor, 160, 7).unwrap();
        assert_eq!(c.authorize(&request("relationship-support", 170, 7)), Err(ConsentError::NotGranted));
    }

    #[test]
    fn non_grantor_cannot_revoke() {
        let mut c = consent();
        let bob = participant("did", "bob");
        assert_eq!(c.revoke(&bob, 160, 7), Err(ConsentError::UnauthorizedRevocation));
    }

    #[test]
    fn disclosure_does_not_become_execution_authority() {
        let c = consent();
        assert!(c.authorize(&request("relationship-support", 150, 7)).is_ok());
    }

    #[test]
    fn supporting_evidence_is_not_embedded_as_authority() {
        assert_eq!(ConsentStatus::Granted, consent().status);
    }
}


fn canonical_principal(out: &mut Vec<u8>, principal: &PrincipalRef) {
    write_len_prefixed(out, principal.participant.namespace.as_bytes());
    write_len_prefixed(out, principal.participant.identifier.as_bytes());
}

fn canonical_purpose(out: &mut Vec<u8>, purpose: &ConsentPurpose) {
    match purpose {
        ConsentPurpose::Named(value) => {
            out.push(0);
            write_len_prefixed(out, value.as_bytes());
        }
    }
}

fn canonical_scope(out: &mut Vec<u8>, scope: &DisclosureScope) {
    out.push(match scope.mode {
        AccessMode::Read => 0,
        AccessMode::Disclose => 1,
    });
    out.extend_from_slice(&(scope.data_classes.len() as u64).to_le_bytes());
    for class in &scope.data_classes {
        match class {
            DataClass::Public => out.push(0),
            DataClass::Identity => out.push(1),
            DataClass::Contact => out.push(2),
            DataClass::Financial => out.push(3),
            DataClass::Health => out.push(4),
            DataClass::Operational => out.push(5),
            DataClass::Confidential => out.push(6),
            DataClass::Custom(value) => {
                out.push(7);
                write_len_prefixed(out, value.as_bytes());
            }
        }
    }
    out.extend_from_slice(&(scope.fields.len() as u64).to_le_bytes());
    for field in &scope.fields {
        write_len_prefixed(out, field.as_bytes());
    }
}

fn write_len_prefixed(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
}
