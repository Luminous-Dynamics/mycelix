//! Deterministic HMP-001 authorization decision kernel.
//!
//! This module evaluates an already-authenticated, already-fetched permission snapshot.
//! It does not verify signatures, fetch current Holochain authority/revocation state, or
//! enforce access in any coordinator read path. Callers MUST obtain and verify those inputs
//! before using this decision. Unknown or contradictory inputs never authorize protected use.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Operation {
    Read,
    Display,
    Cite,
    Index,
    Retrieve,
    Summarize,
    Translate,
    Adapt,
    Train,
    Redistribute,
    Export,
    Preserve,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessClass {
    PublicSource,
    CommunityControlled,
    Restricted,
    ReferenceOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PermissionStatus {
    Granted,
    Pending,
    Denied,
    Withdrawn,
    NotRequested,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityStatus {
    Verified,
    Unverified,
    Disputed,
    NotRequired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RevocationStatus {
    Active,
    Revoked,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecisionKind {
    Allow,
    Deny,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecisionReason {
    ActorNotAuthenticated,
    ActorMismatch,
    PermissionNotGranted,
    AuthorityUnverified,
    AuthorityDisputed,
    AuthorityReferenceMissing,
    RevocationUnknown,
    PermissionRevoked,
    GrantMissing,
    RecordMismatch,
    GranteeMismatch,
    PurposeMismatch,
    PermissionReferenceMissing,
    PermissionBasisMissing,
    GrantNotYetValid,
    GrantExpired,
    ConflictingOperationRules,
    OperationExplicitlyDenied,
    OperationNotGranted,
    AuthorizedByScopedGrant,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PermissionGrant {
    pub record_id: String,
    pub grantee_id: String,
    pub purpose: String,
    pub permission_reference: String,
    pub permission_basis: String,
    pub authority_reference: Option<String>,
    pub allowed_operations: Vec<Operation>,
    pub denied_operations: Vec<Operation>,
    pub valid_from_micros: i64,
    pub expires_at_micros: Option<i64>,
}

#[derive(Debug, Clone, Copy)]
pub struct AuthorizationRequest<'a> {
    /// Caller-supplied actor identity.
    pub actor_id: &'a str,
    /// Identity independently authenticated by the caller's auth layer.
    pub authenticated_actor_id: Option<&'a str>,
    pub record_id: &'a str,
    pub purpose: &'a str,
    pub operation: Operation,
    pub now_micros: i64,
    pub access_class: AccessClass,
    pub permission_status: PermissionStatus,
    pub authority_status: AuthorityStatus,
    pub revocation_status: RevocationStatus,
    /// Snapshot must already be checked against a signed, versioned grant.
    pub grant: Option<&'a PermissionGrant>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorizationDecision {
    pub kind: DecisionKind,
    pub reason: DecisionReason,
}

impl AuthorizationDecision {
    const fn allow(reason: DecisionReason) -> Self {
        Self { kind: DecisionKind::Allow, reason }
    }

    const fn deny(reason: DecisionReason) -> Self {
        Self { kind: DecisionKind::Deny, reason }
    }

    const fn indeterminate(reason: DecisionReason) -> Self {
        Self { kind: DecisionKind::Indeterminate, reason }
    }
}

fn requires_community_authority(access_class: AccessClass) -> bool {
    matches!(
        access_class,
        AccessClass::CommunityControlled | AccessClass::Restricted
    )
}

/// Evaluate one operation against one frozen permission snapshot.
///
/// Fail-closed order is intentional: authentication, permission state, authority,
/// revocation, grant binding, temporal validity, operation conflicts, explicit denial,
/// and finally explicit allow. No trust/reputation or consensus score participates.
pub fn authorize(request: AuthorizationRequest<'_>) -> AuthorizationDecision {
    let Some(authenticated_actor_id) = request.authenticated_actor_id else {
        return AuthorizationDecision::indeterminate(DecisionReason::ActorNotAuthenticated);
    };
    if request.actor_id.trim().is_empty() || request.record_id.trim().is_empty() {
        return AuthorizationDecision::indeterminate(DecisionReason::ActorNotAuthenticated);
    }
    if authenticated_actor_id != request.actor_id {
        return AuthorizationDecision::deny(DecisionReason::ActorMismatch);
    }

    if request.permission_status != PermissionStatus::Granted {
        return AuthorizationDecision::deny(DecisionReason::PermissionNotGranted);
    }

    if requires_community_authority(request.access_class) {
        match request.authority_status {
            AuthorityStatus::Verified => {}
            AuthorityStatus::Disputed => {
                return AuthorizationDecision::indeterminate(DecisionReason::AuthorityDisputed);
            }
            AuthorityStatus::Unverified | AuthorityStatus::NotRequired => {
                return AuthorizationDecision::indeterminate(DecisionReason::AuthorityUnverified);
            }
        }
    }

    if request.revocation_status == RevocationStatus::Unknown {
        return AuthorizationDecision::indeterminate(DecisionReason::RevocationUnknown);
    }
    if request.revocation_status == RevocationStatus::Revoked {
        return AuthorizationDecision::deny(DecisionReason::PermissionRevoked);
    }

    let Some(grant) = request.grant else {
        return AuthorizationDecision::indeterminate(DecisionReason::GrantMissing);
    };

    if grant.record_id != request.record_id {
        return AuthorizationDecision::deny(DecisionReason::RecordMismatch);
    }
    if grant.grantee_id != request.actor_id {
        return AuthorizationDecision::deny(DecisionReason::GranteeMismatch);
    }
    if request.purpose.trim().is_empty() || grant.purpose.trim().is_empty()
        || grant.purpose != request.purpose
    {
        return AuthorizationDecision::deny(DecisionReason::PurposeMismatch);
    }
    if grant.permission_reference.trim().is_empty() {
        return AuthorizationDecision::indeterminate(DecisionReason::PermissionReferenceMissing);
    }
    if grant.permission_basis.trim().is_empty() {
        return AuthorizationDecision::indeterminate(DecisionReason::PermissionBasisMissing);
    }
    if requires_community_authority(request.access_class)
        && grant.authority_reference.as_deref().map_or(true, str::is_empty)
    {
        return AuthorizationDecision::indeterminate(DecisionReason::AuthorityReferenceMissing);
    }
    if request.now_micros < grant.valid_from_micros {
        return AuthorizationDecision::deny(DecisionReason::GrantNotYetValid);
    }
    if grant.expires_at_micros.is_some_and(|expiry| request.now_micros >= expiry) {
        return AuthorizationDecision::deny(DecisionReason::GrantExpired);
    }

    // A reference-only record permits citation of its existence, not content access,
    // retrieval, transformation, training or redistribution—even if a malformed grant
    // accidentally lists one of those operations.
    if request.access_class == AccessClass::ReferenceOnly && request.operation != Operation::Cite {
        return AuthorizationDecision::deny(DecisionReason::OperationNotGranted);
    }

    let explicitly_allowed = grant.allowed_operations.contains(&request.operation);
    let explicitly_denied = grant.denied_operations.contains(&request.operation);

    if explicitly_allowed && explicitly_denied {
        return AuthorizationDecision::indeterminate(DecisionReason::ConflictingOperationRules);
    }
    if explicitly_denied {
        return AuthorizationDecision::deny(DecisionReason::OperationExplicitlyDenied);
    }
    if !explicitly_allowed {
        return AuthorizationDecision::deny(DecisionReason::OperationNotGranted);
    }

    AuthorizationDecision::allow(DecisionReason::AuthorizedByScopedGrant)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_grant() -> PermissionGrant {
        PermissionGrant {
            record_id: "record-1".into(),
            grantee_id: "agent-1".into(),
            purpose: "research-display".into(),
            permission_reference: "signed-grant-1".into(),
            permission_basis: "explicit scoped grant".into(),
            authority_reference: Some("community-authority-1".into()),
            allowed_operations: vec![Operation::Cite, Operation::Display],
            denied_operations: vec![
                Operation::Read,
                Operation::Retrieve,
                Operation::Summarize,
                Operation::Translate,
                Operation::Train,
                Operation::Redistribute,
                Operation::Export,
            ],
            valid_from_micros: 10,
            expires_at_micros: Some(100),
        }
    }

    fn request<'a>(
        grant: Option<&'a PermissionGrant>,
        operation: Operation,
    ) -> AuthorizationRequest<'a> {
        AuthorizationRequest {
            actor_id: "agent-1",
            authenticated_actor_id: Some("agent-1"),
            record_id: "record-1",
            purpose: "research-display",
            operation,
            now_micros: 50,
            access_class: AccessClass::CommunityControlled,
            permission_status: PermissionStatus::Granted,
            authority_status: AuthorityStatus::Verified,
            revocation_status: RevocationStatus::Active,
            grant,
        }
    }

    #[test]
    fn scoped_citation_and_display_are_allowed() {
        let grant = valid_grant();
        assert_eq!(authorize(request(Some(&grant), Operation::Cite)).kind, DecisionKind::Allow);
        assert_eq!(authorize(request(Some(&grant), Operation::Display)).kind, DecisionKind::Allow);
    }

    #[test]
    fn citation_only_style_grant_does_not_allow_training_or_retrieval() {
        let grant = valid_grant();
        assert_eq!(authorize(request(Some(&grant), Operation::Train)).kind, DecisionKind::Deny);
        assert_eq!(authorize(request(Some(&grant), Operation::Retrieve)).kind, DecisionKind::Deny);
    }

    #[test]
    fn explicit_denial_overrides_other_allowed_operations() {
        let grant = valid_grant();
        let decision = authorize(request(Some(&grant), Operation::Translate));
        assert_eq!(decision.kind, DecisionKind::Deny);
        assert_eq!(decision.reason, DecisionReason::OperationExplicitlyDenied);
    }

    #[test]
    fn revoked_permission_never_authorizes() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.revocation_status = RevocationStatus::Revoked;
        assert_eq!(authorize(req).kind, DecisionKind::Deny);
    }

    #[test]
    fn unknown_revocation_state_is_indeterminate() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.revocation_status = RevocationStatus::Unknown;
        assert_eq!(authorize(req).kind, DecisionKind::Indeterminate);
    }

    #[test]
    fn unauthenticated_actor_is_indeterminate() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.authenticated_actor_id = None;
        assert_eq!(authorize(req).kind, DecisionKind::Indeterminate);
    }

    #[test]
    fn actor_mismatch_is_denied() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.authenticated_actor_id = Some("agent-2");
        assert_eq!(authorize(req).kind, DecisionKind::Deny);
    }

    #[test]
    fn pending_permission_is_denied() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.permission_status = PermissionStatus::Pending;
        assert_eq!(authorize(req).kind, DecisionKind::Deny);
    }

    #[test]
    fn disputed_community_authority_is_indeterminate() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.authority_status = AuthorityStatus::Disputed;
        assert_eq!(authorize(req).kind, DecisionKind::Indeterminate);
    }

    #[test]
    fn community_authority_reference_is_required() {
        let mut grant = valid_grant();
        grant.authority_reference = None;
        assert_eq!(
            authorize(request(Some(&grant), Operation::Cite)).kind,
            DecisionKind::Indeterminate
        );
    }

    #[test]
    fn grant_for_different_record_or_actor_is_denied() {
        let mut grant = valid_grant();
        grant.record_id = "record-2".into();
        assert_eq!(authorize(request(Some(&grant), Operation::Cite)).reason, DecisionReason::RecordMismatch);

        let mut grant = valid_grant();
        grant.grantee_id = "agent-2".into();
        assert_eq!(authorize(request(Some(&grant), Operation::Cite)).reason, DecisionReason::GranteeMismatch);
    }

    #[test]
    fn purpose_mismatch_is_denied() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.purpose = "model-training";
        assert_eq!(authorize(req).reason, DecisionReason::PurposeMismatch);
    }

    #[test]
    fn expired_and_not_yet_valid_grants_are_denied() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.now_micros = 100;
        assert_eq!(authorize(req).reason, DecisionReason::GrantExpired);

        let mut req = request(Some(&grant), Operation::Cite);
        req.now_micros = 9;
        assert_eq!(authorize(req).reason, DecisionReason::GrantNotYetValid);
    }

    #[test]
    fn conflicting_allowed_and_denied_rules_are_indeterminate() {
        let mut grant = valid_grant();
        grant.denied_operations.push(Operation::Cite);
        assert_eq!(
            authorize(request(Some(&grant), Operation::Cite)).reason,
            DecisionReason::ConflictingOperationRules
        );
    }

    #[test]
    fn reference_only_never_allows_content_operations() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Read);
        req.access_class = AccessClass::ReferenceOnly;
        assert_eq!(authorize(req).kind, DecisionKind::Deny);
    }

    #[test]
    fn reference_only_citation_can_be_allowed_by_scoped_grant() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.access_class = AccessClass::ReferenceOnly;
        assert_eq!(authorize(req).kind, DecisionKind::Allow);
    }

    #[test]
    fn empty_purpose_is_denied() {
        let grant = valid_grant();
        let mut req = request(Some(&grant), Operation::Cite);
        req.purpose = "";
        assert_eq!(authorize(req).reason, DecisionReason::PurposeMismatch);
    }

    #[test]
    fn missing_grant_is_indeterminate() {
        assert_eq!(
            authorize(request(None, Operation::Cite)).kind,
            DecisionKind::Indeterminate
        );
    }
}
