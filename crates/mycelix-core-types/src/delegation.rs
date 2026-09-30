//! Delegated authority primitives for the Relationship OS.
//!
//! This layer is intentionally distinct from consent. Consent governs bounded
//! information access/disclosure; delegated authority governs bounded actions.
//! The result of authorization is not proof that an external action occurred.

use std::fmt;

use crate::{AuthorityEpoch, ParticipantRef, RelationshipId};

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct PrincipalRef {
    pub participant: ParticipantRef,
}

impl PrincipalRef {
    pub fn new(participant: ParticipantRef) -> Self { Self { participant } }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ActionScope {
    Named(String),
}

impl ActionScope {
    pub fn named(value: impl Into<String>) -> Result<Self, DelegationError> {
        let value = value.into();
        if value.trim().is_empty() { return Err(DelegationError::EmptyScope); }
        Ok(Self::Named(value))
    }

    fn contains(&self, requested: &Self) -> bool { self == requested }
}

#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ResourceScope {
    Named(String),
}

impl ResourceScope {
    pub fn named(value: impl Into<String>) -> Result<Self, DelegationError> {
        let value = value.into();
        if value.trim().is_empty() { return Err(DelegationError::EmptyScope); }
        Ok(Self::Named(value))
    }

    fn contains(&self, requested: &Self) -> bool { self == requested }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum DelegationMode {
    Propose,
    Execute,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum DelegationStatus {
    Active,
    Revoked,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum DelegationEventKind {
    Grant,
    Revoke,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DelegationEvent {
    pub kind: DelegationEventKind,
    pub actor: PrincipalRef,
    pub occurred_at: i64,
    pub authority_epoch: AuthorityEpoch,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DelegationId([u8; 32]);

impl DelegationId {
    pub fn derive(relationship: RelationshipId, seed: &[u8]) -> Self {
        let mut h = blake3::Hasher::new();
        h.update(b"mycelix.delegation.v1\0");
        h.update(relationship.as_bytes());
        h.update(&(seed.len() as u64).to_le_bytes());
        h.update(seed);
        Self(*h.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] { &self.0 }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Delegation {
    pub id: DelegationId,
    pub relationship_id: RelationshipId,
    pub principal: PrincipalRef,
    pub delegate: PrincipalRef,
    pub action: ActionScope,
    pub resource: ResourceScope,
    pub mode: DelegationMode,
    pub granted_at: i64,
    pub expires_at: Option<i64>,
    pub authority_epoch: AuthorityEpoch,
    pub status: DelegationStatus,
    pub parent: Option<DelegationId>,
    pub events: Vec<DelegationEvent>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExecutionRequest {
    pub principal: PrincipalRef,
    pub delegate: PrincipalRef,
    pub action: ActionScope,
    pub resource: ResourceScope,
    pub mode: DelegationMode,
    pub requested_at: i64,
    pub authority_epoch: AuthorityEpoch,
    pub parent_active: bool,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ExecutionDecision {
    AuthorizedProposal,
    AuthorizedExecution,
}

impl Delegation {
    pub fn grant(
        id: DelegationId,
        relationship_id: RelationshipId,
        principal: PrincipalRef,
        delegate: PrincipalRef,
        action: ActionScope,
        resource: ResourceScope,
        mode: DelegationMode,
        granted_at: i64,
        expires_at: Option<i64>,
        authority_epoch: AuthorityEpoch,
        parent: Option<DelegationId>,
    ) -> Result<Self, DelegationError> {
        if principal == delegate { return Err(DelegationError::SelfDelegation); }
        if let Some(expiry) = expires_at {
            if expiry < granted_at { return Err(DelegationError::InvalidExpiry); }
        }
        let event = DelegationEvent {
            kind: DelegationEventKind::Grant,
            actor: principal.clone(),
            occurred_at: granted_at,
            authority_epoch,
        };
        Ok(Self {
            id, relationship_id, principal, delegate, action, resource, mode,
            granted_at, expires_at, authority_epoch,
            status: DelegationStatus::Active, parent, events: vec![event],
        })
    }

    pub fn revoke(
        &mut self,
        actor: &PrincipalRef,
        occurred_at: i64,
        authority_epoch: AuthorityEpoch,
    ) -> Result<(), DelegationError> {
        if self.status != DelegationStatus::Active { return Err(DelegationError::AlreadyInactive); }
        if actor != &self.principal { return Err(DelegationError::UnauthorizedRevocation); }
        if occurred_at < self.granted_at { return Err(DelegationError::NonMonotonicEventTime); }
        self.status = DelegationStatus::Revoked;
        self.events.push(DelegationEvent {
            kind: DelegationEventKind::Revoke,
            actor: actor.clone(),
            occurred_at,
            authority_epoch,
        });
        Ok(())
    }

    pub fn authorize(&self, request: &ExecutionRequest) -> Result<ExecutionDecision, DelegationError> {
        if self.status != DelegationStatus::Active { return Err(DelegationError::NotActive); }
        if request.requested_at < self.granted_at { return Err(DelegationError::NotYetEffective); }
        if let Some(expiry) = self.expires_at {
            if request.requested_at >= expiry { return Err(DelegationError::Expired); }
        }
        if request.authority_epoch != self.authority_epoch {
            return Err(DelegationError::StaleAuthorityEpoch);
        }
        if request.principal != self.principal { return Err(DelegationError::PrincipalMismatch); }
        if request.delegate != self.delegate { return Err(DelegationError::DelegateMismatch); }
        if !self.action.contains(&request.action) { return Err(DelegationError::ActionExceeded); }
        if !self.resource.contains(&request.resource) { return Err(DelegationError::ResourceExceeded); }
        if request.mode != self.mode { return Err(DelegationError::ModeMismatch); }
        if self.parent.is_some() && !request.parent_active {
            return Err(DelegationError::ParentInactive);
        }

        Ok(match request.mode {
            DelegationMode::Propose => ExecutionDecision::AuthorizedProposal,
            DelegationMode::Execute => ExecutionDecision::AuthorizedExecution,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DelegationError {
    EmptyScope,
    SelfDelegation,
    InvalidExpiry,
    AlreadyInactive,
    UnauthorizedRevocation,
    NonMonotonicEventTime,
    NotActive,
    NotYetEffective,
    Expired,
    StaleAuthorityEpoch,
    PrincipalMismatch,
    DelegateMismatch,
    ActionExceeded,
    ResourceExceeded,
    ModeMismatch,
    ParentInactive,
}

impl fmt::Display for DelegationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyScope => "delegation scope must not be empty",
            Self::SelfDelegation => "principal cannot delegate to itself",
            Self::InvalidExpiry => "delegation expiry cannot precede grant time",
            Self::AlreadyInactive => "delegation is already inactive",
            Self::UnauthorizedRevocation => "only the principal may revoke delegation",
            Self::NonMonotonicEventTime => "delegation event time cannot move backwards",
            Self::NotActive => "delegation is not active",
            Self::NotYetEffective => "delegation was not active at request time",
            Self::Expired => "delegation had expired at request time",
            Self::StaleAuthorityEpoch => "request authority epoch does not match delegation",
            Self::PrincipalMismatch => "request principal does not match delegation",
            Self::DelegateMismatch => "request delegate does not match delegation",
            Self::ActionExceeded => "requested action exceeds delegation",
            Self::ResourceExceeded => "requested resource exceeds delegation",
            Self::ModeMismatch => "request mode does not match delegation",
            Self::ParentInactive => "parent delegation is not active",
        };
        f.write_str(message)
    }
}

impl std::error::Error for DelegationError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(id: &str) -> PrincipalRef { PrincipalRef::new(ParticipantRef::new("did", id)) }

    fn delegation(mode: DelegationMode) -> Delegation {
        let relationship = RelationshipId::derive("test", b"relationship");
        Delegation::grant(
            DelegationId::derive(relationship, b"delegation"),
            relationship,
            p("alice"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            mode, 100, Some(200), 4, None,
        ).unwrap()
    }

    fn request(mode: DelegationMode, at: i64, epoch: AuthorityEpoch) -> ExecutionRequest {
        ExecutionRequest {
            principal: p("alice"), delegate: p("agent"),
            action: ActionScope::named("create-opportunity").unwrap(),
            resource: ResourceScope::named("acme:opportunity:7").unwrap(),
            mode, requested_at: at, authority_epoch: epoch, parent_active: true,
        }
    }

    #[test]
    fn proposal_and_execution_are_distinct_decisions() {
        assert_eq!(delegation(DelegationMode::Propose).authorize(&request(DelegationMode::Propose, 150, 4)),
            Ok(ExecutionDecision::AuthorizedProposal));
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 4)),
            Ok(ExecutionDecision::AuthorizedExecution));
    }

    #[test]
    fn action_escalation_fails_closed() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.action = ActionScope::named("delete-opportunity").unwrap();
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r), Err(DelegationError::ActionExceeded));
    }

    #[test]
    fn resource_escalation_fails_closed() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.resource = ResourceScope::named("acme:opportunity:8").unwrap();
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r), Err(DelegationError::ResourceExceeded));
    }

    #[test]
    fn stale_epoch_fails_closed() {
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 5)),
            Err(DelegationError::StaleAuthorityEpoch));
    }

    #[test]
    fn expired_delegation_fails() {
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 200, 4)),
            Err(DelegationError::Expired));
    }

    #[test]
    fn wrong_delegate_fails() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.delegate = p("other-agent");
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r), Err(DelegationError::DelegateMismatch));
    }

    #[test]
    fn revoked_delegation_fails() {
        let mut d = delegation(DelegationMode::Execute);
        d.revoke(&p("alice"), 160, 4).unwrap();
        assert_eq!(d.authorize(&request(DelegationMode::Execute, 170, 4)), Err(DelegationError::NotActive));
    }

    #[test]
    fn parent_revocation_blocks_child() {
        let relationship = RelationshipId::derive("test", b"relationship");
        let parent = DelegationId::derive(relationship, b"parent");
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("alice"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, None, 4, Some(parent),
        ).unwrap();
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.parent_active = false;
        assert_eq!(child.authorize(&r), Err(DelegationError::ParentInactive));
    }

    #[test]
    fn capability_presence_is_not_semantic_authority() {
        // A Holochain capability is an underlying call-security mechanism.
        // This pure type has no ambient capability lookup, so absence/presence
        // of a runtime token cannot silently authorize this delegation.
        assert!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 4)).is_ok());
    }

    #[test]
    fn external_success_is_not_implied() {
        // Authorization returns a decision only; no external execution result exists here.
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 4)),
            Ok(ExecutionDecision::AuthorizedExecution));
    }
}
