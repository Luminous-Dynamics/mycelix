//! Delegated authority primitives for the Relationship OS.
//!
//! This layer is intentionally distinct from consent. Consent governs bounded
//! information access/disclosure; delegated authority governs bounded actions.
//! Authorization requires an addressable delegation context when a parent exists.
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

impl DelegationMode {
    fn is_contained_by(self, parent: Self) -> bool {
        matches!(
            (parent, self),
            (Self::Propose, Self::Propose) | (Self::Execute, Self::Propose | Self::Execute)
        )
    }
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
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum ExecutionDecision {
    AuthorizedProposal,
    AuthorizedExecution,
}

/// Addressable validation dependencies for delegated authority.
///
/// The context is deliberately supplied by the caller instead of being fetched
/// from ambient mutable state. A validator can therefore deterministically
/// retrieve the exact parent records it used for a decision.
pub struct DelegationContext<'a> {
    delegations: &'a [Delegation],
}

impl<'a> DelegationContext<'a> {
    pub fn from_slice(delegations: &'a [Delegation]) -> Result<Self, DelegationError> {
        for (index, delegation) in delegations.iter().enumerate() {
            if delegations[index + 1..].iter().any(|other| other.id == delegation.id) {
                return Err(DelegationError::DuplicateDelegationId { id: delegation.id });
            }
        }
        Ok(Self { delegations })
    }

    fn get(&self, id: DelegationId) -> Result<&Delegation, DelegationError> {
        self.delegations
            .iter()
            .find(|delegation| delegation.id == id)
            .ok_or(DelegationError::MissingParent { id })
    }
}

impl Delegation {
    pub const MAX_CHAIN_DEPTH: usize = 64;

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

    pub fn authorize(
        &self,
        request: &ExecutionRequest,
        context: &DelegationContext<'_>,
    ) -> Result<ExecutionDecision, DelegationError> {
        self.validate_request(request)?;
        if self.parent.is_some() {
            self.validate_parent_chain(context, request)?;
        }

        Ok(match request.mode {
            DelegationMode::Propose => ExecutionDecision::AuthorizedProposal,
            DelegationMode::Execute => ExecutionDecision::AuthorizedExecution,
        })
    }

    fn validate_request(&self, request: &ExecutionRequest) -> Result<(), DelegationError> {
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
        Ok(())
    }

    fn validate_parent_chain(
        &self,
        context: &DelegationContext<'_>,
        request: &ExecutionRequest,
    ) -> Result<(), DelegationError> {
        let mut child = self;
        let mut next = self.parent;
        let mut visited = Vec::new();
        let mut depth = 0usize;

        while let Some(parent_id) = next {
            if depth >= Self::MAX_CHAIN_DEPTH {
                return Err(DelegationError::DelegationChainTooDeep);
            }
            if visited.contains(&parent_id) || parent_id == child.id {
                return Err(DelegationError::DelegationCycle { id: parent_id });
            }
            visited.push(parent_id);

            let parent = context.get(parent_id)?;
            if parent.status != DelegationStatus::Active {
                return Err(DelegationError::ParentInactive { id: parent.id });
            }
            if parent.relationship_id != child.relationship_id {
                return Err(DelegationError::ParentRelationshipMismatch);
            }
            if parent.delegate != child.principal {
                return Err(DelegationError::ParentPrincipalMismatch);
            }
            if parent.authority_epoch != child.authority_epoch {
                return Err(DelegationError::ParentEpochMismatch);
            }
            if !parent.action.contains(&child.action) {
                return Err(DelegationError::ParentActionExceeded);
            }
            if !parent.resource.contains(&child.resource) {
                return Err(DelegationError::ParentResourceExceeded);
            }
            if !child.mode.is_contained_by(parent.mode) {
                return Err(DelegationError::ParentModeExceeded);
            }
            if child.granted_at < parent.granted_at {
                return Err(DelegationError::ParentNotYetEffective);
            }
            if let Some(parent_expiry) = parent.expires_at {
                if child.granted_at >= parent_expiry {
                    return Err(DelegationError::ParentExpired);
                }
                if child.expires_at.is_none_or(|child_expiry| child_expiry > parent_expiry) {
                    return Err(DelegationError::ChildOutlivesParent);
                }
                if request.requested_at >= parent_expiry {
                    return Err(DelegationError::ParentExpired);
                }
            }

            child = parent;
            next = parent.parent;
            depth += 1;
        }

        Ok(())
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
    DuplicateDelegationId { id: DelegationId },
    MissingParent { id: DelegationId },
    ParentInactive { id: DelegationId },
    ParentRelationshipMismatch,
    ParentPrincipalMismatch,
    ParentEpochMismatch,
    ParentActionExceeded,
    ParentResourceExceeded,
    ParentModeExceeded,
    ParentNotYetEffective,
    ParentExpired,
    ChildOutlivesParent,
    DelegationCycle { id: DelegationId },
    DelegationChainTooDeep,
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
            Self::DuplicateDelegationId { .. } => "delegation context contains duplicate identifiers",
            Self::MissingParent { .. } => "parent delegation is missing from validation context",
            Self::ParentInactive { .. } => "parent delegation is not active",
            Self::ParentRelationshipMismatch => "parent and child belong to different relationships",
            Self::ParentPrincipalMismatch => "child principal is not the parent delegate",
            Self::ParentEpochMismatch => "parent and child authority epochs differ",
            Self::ParentActionExceeded => "child action exceeds parent delegation",
            Self::ParentResourceExceeded => "child resource exceeds parent delegation",
            Self::ParentModeExceeded => "child mode exceeds parent delegation",
            Self::ParentNotYetEffective => "child delegation predates its parent",
            Self::ParentExpired => "parent delegation had expired at the requested time",
            Self::ChildOutlivesParent => "child delegation outlives its parent",
            Self::DelegationCycle { .. } => "delegation parent chain contains a cycle",
            Self::DelegationChainTooDeep => "delegation parent chain exceeds the deterministic depth limit",
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
            mode, requested_at: at, authority_epoch: epoch,
        }
    }

    fn context<'a>(delegations: &'a [Delegation]) -> DelegationContext<'a> {
        DelegationContext::from_slice(delegations).unwrap()
    }

    #[test]
    fn proposal_and_execution_are_distinct_decisions() {
        let d = delegation(DelegationMode::Propose);
        assert_eq!(d.authorize(&request(DelegationMode::Propose, 150, 4), &context(&[])),
            Ok(ExecutionDecision::AuthorizedProposal));
        let d = delegation(DelegationMode::Execute);
        assert_eq!(d.authorize(&request(DelegationMode::Execute, 150, 4), &context(&[])),
            Ok(ExecutionDecision::AuthorizedExecution));
    }

    #[test]
    fn action_escalation_fails_closed() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.action = ActionScope::named("delete-opportunity").unwrap();
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r, &context(&[])), Err(DelegationError::ActionExceeded));
    }

    #[test]
    fn resource_escalation_fails_closed() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.resource = ResourceScope::named("acme:opportunity:8").unwrap();
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r, &context(&[])), Err(DelegationError::ResourceExceeded));
    }

    #[test]
    fn stale_epoch_fails_closed() {
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 5), &context(&[])),
            Err(DelegationError::StaleAuthorityEpoch));
    }

    #[test]
    fn expired_delegation_fails() {
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 200, 4), &context(&[])),
            Err(DelegationError::Expired));
    }

    #[test]
    fn wrong_delegate_fails() {
        let mut r = request(DelegationMode::Execute, 150, 4);
        r.delegate = p("other-agent");
        assert_eq!(delegation(DelegationMode::Execute).authorize(&r, &context(&[])), Err(DelegationError::DelegateMismatch));
    }

    #[test]
    fn revoked_delegation_fails() {
        let mut d = delegation(DelegationMode::Execute);
        d.revoke(&p("alice"), 160, 4).unwrap();
        assert_eq!(d.authorize(&request(DelegationMode::Execute, 170, 4), &context(&[])), Err(DelegationError::NotActive));
    }

    #[test]
    fn missing_parent_is_not_caller_assertable() {
        let relationship = RelationshipId::derive("test", b"relationship");
        let parent = DelegationId::derive(relationship, b"parent");
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("alice"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, None, 4, Some(parent),
        ).unwrap();
        assert_eq!(
            child.authorize(&request(DelegationMode::Execute, 150, 4), &context(&[])),
            Err(DelegationError::MissingParent { id: parent })
        );
    }

    #[test]
    fn valid_parent_chain_authorizes() {
        let relationship = RelationshipId::derive("test", b"chain");
        let parent = Delegation::grant(
            DelegationId::derive(relationship, b"parent"),
            relationship, p("alice"), p("broker"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, Some(300), 4, None,
        ).unwrap();
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("broker"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 150, Some(250), 4, Some(parent.id),
        ).unwrap();
        let req = ExecutionRequest {
            principal: p("broker"), delegate: p("agent"),
            action: ActionScope::named("create-opportunity").unwrap(),
            resource: ResourceScope::named("acme:opportunity:7").unwrap(),
            mode: DelegationMode::Execute, requested_at: 200, authority_epoch: 4,
        };
        assert_eq!(child.authorize(&req, &context(&[parent])),
            Ok(ExecutionDecision::AuthorizedExecution));
    }

    #[test]
    fn revoked_parent_blocks_child() {
        let relationship = RelationshipId::derive("test", b"revoked-parent");
        let mut parent = Delegation::grant(
            DelegationId::derive(relationship, b"parent"),
            relationship, p("alice"), p("broker"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, None, 4, None,
        ).unwrap();
        let parent_id = parent.id;
        parent.revoke(&p("alice"), 160, 4).unwrap();
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("broker"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 150, None, 4, Some(parent_id),
        ).unwrap();
        assert_eq!(child.authorize(
            &ExecutionRequest {
                principal: p("broker"), delegate: p("agent"),
                action: ActionScope::named("create-opportunity").unwrap(),
                resource: ResourceScope::named("acme:opportunity:7").unwrap(),
                mode: DelegationMode::Execute, requested_at: 155, authority_epoch: 4,
            },
            &context(&[parent]),
        ), Err(DelegationError::ParentInactive { id: parent_id }));
    }

    #[test]
    fn parent_scope_and_epoch_escalation_fail_closed() {
        let relationship = RelationshipId::derive("test", b"scope");
        let parent = Delegation::grant(
            DelegationId::derive(relationship, b"parent"),
            relationship, p("alice"), p("broker"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Propose, 100, Some(300), 4, None,
        ).unwrap();
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("broker"), p("agent"),
            ActionScope::named("delete-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 150, None, 4, Some(parent.id),
        ).unwrap();
        let req = ExecutionRequest {
            principal: p("broker"), delegate: p("agent"),
            action: ActionScope::named("delete-opportunity").unwrap(),
            resource: ResourceScope::named("acme:opportunity:7").unwrap(),
            mode: DelegationMode::Execute, requested_at: 200, authority_epoch: 4,
        };
        assert_eq!(child.authorize(&req, &context(&[parent])),
            Err(DelegationError::ParentActionExceeded));
    }

    #[test]
    fn parent_chain_cycle_is_rejected() {
        let relationship = RelationshipId::derive("test", b"cycle");
        let a_id = DelegationId::derive(relationship, b"a");
        let b_id = DelegationId::derive(relationship, b"b");
        let a = Delegation::grant(
            a_id, relationship, p("root"), p("broker"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, None, 4, Some(b_id),
        ).unwrap();
        let b = Delegation::grant(
            b_id, relationship, p("broker"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 110, None, 4, Some(a_id),
        ).unwrap();
        let req = ExecutionRequest {
            principal: p("root"), delegate: p("broker"),
            action: ActionScope::named("create-opportunity").unwrap(),
            resource: ResourceScope::named("acme:opportunity:7").unwrap(),
            mode: DelegationMode::Execute, requested_at: 120, authority_epoch: 4,
        };
        assert_eq!(a.authorize(&req, &context(&[b, a])),
            Err(DelegationError::ParentPrincipalMismatch));
    }

    #[test]
    fn duplicate_context_ids_are_rejected() {
        let d = delegation(DelegationMode::Execute);
        assert_eq!(
            DelegationContext::from_slice(&[d.clone(), d]),
            Err(DelegationError::DuplicateDelegationId { id: d.id })
        );
    }

    #[test]
    fn child_cannot_outlive_parent() {
        let relationship = RelationshipId::derive("test", b"expiry");
        let parent = Delegation::grant(
            DelegationId::derive(relationship, b"parent"),
            relationship, p("alice"), p("broker"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 100, Some(200), 4, None,
        ).unwrap();
        let child = Delegation::grant(
            DelegationId::derive(relationship, b"child"),
            relationship, p("broker"), p("agent"),
            ActionScope::named("create-opportunity").unwrap(),
            ResourceScope::named("acme:opportunity:7").unwrap(),
            DelegationMode::Execute, 150, Some(250), 4, Some(parent.id),
        ).unwrap();
        let req = ExecutionRequest {
            principal: p("broker"), delegate: p("agent"),
            action: ActionScope::named("create-opportunity").unwrap(),
            resource: ResourceScope::named("acme:opportunity:7").unwrap(),
            mode: DelegationMode::Execute, requested_at: 180, authority_epoch: 4,
        };
        assert_eq!(child.authorize(&req, &context(&[parent])),
            Err(DelegationError::ChildOutlivesParent));
    }

    #[test]
    fn capability_presence_is_not_semantic_authority() {
        assert!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 4), &context(&[])).is_ok());
    }

    #[test]
    fn external_success_is_not_implied() {
        assert_eq!(delegation(DelegationMode::Execute).authorize(&request(DelegationMode::Execute, 150, 4), &context(&[])),
            Ok(ExecutionDecision::AuthorizedExecution));
    }
}
