//! Reference authorization semantics for plural Mycelix Craft guilds.
//!
//! This crate separates presentation labels from authority. It does not verify
//! signatures, charter adoption, grantor eligibility, Holochain provenance, or
//! federation membership. Those checks belong to the owning integrity and
//! coordinator layers before trusted assignments reach this evaluator.
//!
//! This is a fail-closed policy core, not a complete identity or governance system.

#![forbid(unsafe_code)]

use std::collections::HashSet;

/// An explicit scope. Guild and federation scopes never inherit into one another.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub enum ScopeV1 {
    /// A domain inside one guild.
    Guild { guild_id: String, domain_id: String },
    /// A domain inside one named federation.
    Federation { federation_id: String, domain_id: String },
}

impl ScopeV1 {
    fn validate(&self) -> Result<(), AssignmentErrorV1> {
        match self {
            Self::Guild { guild_id, domain_id } => {
                require_non_empty(guild_id)?;
                require_non_empty(domain_id)
            }
            Self::Federation { federation_id, domain_id } => {
                require_non_empty(federation_id)?;
                require_non_empty(domain_id)
            }
        }
    }
}

/// Actions are permissions. Titles and labels are not.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum GovernanceActionV1 {
    /// Submit a proposal for a domain decision.
    Propose,
    /// Facilitate a process without acquiring its decision rights.
    Facilitate,
    /// Record an already-made decision.
    RecordDecision,
    /// Admit a member under the guild charter.
    AdmitMember,
    /// Suspend a member under due process.
    SuspendMember,
    /// Amend a local guild charter.
    AmendGuildCharter,
    /// Elect or recall a delegate.
    ManageDelegates,
    /// Set treasury policy within the named domain.
    SetTreasuryPolicy,
    /// Authorize a disbursement within an explicitly delegated limit.
    AuthorizeDisbursement,
    /// Create a federation agreement.
    CreateFederationAgreement,
    /// Amend a federation agreement.
    AmendFederationAgreement,
}

/// Membership status is independent of identity and governance assignments.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MembershipStatusV1 {
    /// Member in good standing under the local membership contract.
    Active,
    /// Member temporarily on leave.
    OnLeave,
    /// Member suspended under the applicable process.
    Suspended,
    /// Membership has ended.
    Exited,
    /// Identity is verified, but the principal is not a member of this guild.
    NotMember,
    /// Membership state could not be established from the trusted membership owner.
    Unknown,
}

/// Charter-selected eligibility rule for one governance assignment.
///
/// A broader policy permits a verified external principal to receive only the
/// exact actions and scope in this assignment. It does not create membership,
/// professional assessment authority, or federation inheritance.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PrincipalEligibilityPolicyV1 {
    /// Only an active member may exercise the assignment.
    ActiveMemberRequired,
    /// Active members, members on leave, and verified non-members may exercise
    /// the exact assignment. Suspended, exited, and unknown states are denied.
    VerifiedPrincipalNotSuspendedOrExited,
}

/// Exact adopted charter-policy reference that authorized this assignment's
/// eligibility rule. The host verifies adoption and grantor authority upstream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CharterPolicyRefV1 {
    /// Stable charter policy identifier.
    pub policy_id: String,
    /// Monotonically versioned adopted policy revision.
    pub revision: u64,
    /// Digest of the exact adopted policy document.
    pub digest: [u8; 32],
}

impl CharterPolicyRefV1 {
    fn validate(&self) -> Result<(), AssignmentErrorV1> {
        require_non_empty(&self.policy_id)?;
        if self.revision == 0 || self.digest.iter().all(|byte| *byte == 0) {
            return Err(AssignmentErrorV1::InvalidCharterPolicyReference);
        }
        Ok(())
    }
}

/// A time-bounded assignment of explicit actions in one exact scope.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GovernanceAssignmentV1 {
    /// Stable identifier for this assignment record.
    pub assignment_id: String,
    /// Person or organization receiving the assignment.
    pub subject_id: String,
    /// Exact domain in which the assignment has effect.
    pub scope: ScopeV1,
    /// Explicit action set. No action is inferred from display_label.
    pub actions: Vec<GovernanceActionV1>,
    /// Charter-selected principal eligibility rule for this assignment.
    pub principal_eligibility: PrincipalEligibilityPolicyV1,
    /// Exact adopted charter policy authorizing this eligibility rule.
    pub charter_policy: CharterPolicyRefV1,
    /// Identity that granted the assignment; its authority is verified upstream.
    pub grantor_id: String,
    /// Optional culturally or locally chosen display label (for example, Elder).
    pub display_label: Option<String>,
    /// Inclusive start timestamp (Unix seconds).
    pub effective_at: i64,
    /// Exclusive expiry timestamp (Unix seconds).
    pub expires_at: i64,
    /// Timestamp at which revocation takes effect, if any.
    pub revoked_at: Option<i64>,
    /// Actions for which this assignment holder has a declared conflict of interest.
    pub conflicts_of_interest: Vec<GovernanceActionV1>,
}

impl GovernanceAssignmentV1 {
    /// Validate local invariants. This does not authenticate the assignment issuer.
    pub fn validate(&self) -> Result<(), AssignmentErrorV1> {
        require_non_empty(&self.assignment_id)?;
        require_non_empty(&self.subject_id)?;
        require_non_empty(&self.grantor_id)?;
        if self.grantor_id == self.subject_id {
            return Err(AssignmentErrorV1::SelfGrant);
        }
        self.scope.validate()?;
        self.charter_policy.validate()?;
        if self.actions.is_empty() {
            return Err(AssignmentErrorV1::EmptyActionSet);
        }
        if self.expires_at <= self.effective_at {
            return Err(AssignmentErrorV1::InvalidTimeRange);
        }
        if let Some(revoked_at) = self.revoked_at {
            if revoked_at < self.effective_at {
                return Err(AssignmentErrorV1::InvalidRevocationTime);
            }
        }

        let mut actions = HashSet::new();
        for action in &self.actions {
            if !actions.insert(*action) {
                return Err(AssignmentErrorV1::DuplicateAction);
            }
        }

        let mut conflicts = HashSet::new();
        for action in &self.conflicts_of_interest {
            if !actions.contains(action) {
                return Err(AssignmentErrorV1::ConflictNotGranted);
            }
            if !conflicts.insert(*action) {
                return Err(AssignmentErrorV1::DuplicateConflict);
            }
        }
        Ok(())
    }
}

/// Public presentation metadata. This type is intentionally not accepted by
/// authorize; founder provenance and honorifics cannot grant rights.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemberPresentationV1 {
    /// Member identity.
    pub member_id: String,
    /// Optional locally selected honorific or display alias.
    pub honorific: Option<String>,
    /// Historical provenance only; not an authorization signal.
    pub founder_provenance: bool,
}

/// Request to check one action against one exact domain.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorizationRequestV1 {
    /// Requesting identity (member or external principal).
    pub principal_id: String,
    /// True only when the trusted host has authenticated the principal identity.
    pub principal_identity_verified: bool,
    /// Membership status from the trusted membership owner; NotMember is distinct
    /// from Unknown and must only be asserted after an authoritative lookup.
    pub membership_status: MembershipStatusV1,
    /// Requested action.
    pub action: GovernanceActionV1,
    /// Exact requested scope.
    pub scope: ScopeV1,
    /// Evaluation time in Unix seconds.
    pub at: i64,
}

/// Structural errors in one assignment record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssignmentErrorV1 {
    /// A required identifier was blank.
    EmptyField,
    /// A grantor and subject may not self-authorize through this contract.
    SelfGrant,
    /// No permission actions were declared.
    EmptyActionSet,
    /// Expiry must be strictly after the effective time.
    InvalidTimeRange,
    /// Revocation may not predate the assignment's effective time.
    InvalidRevocationTime,
    /// The same action was declared more than once.
    DuplicateAction,
    /// A conflict was declared for an action not present in the assignment.
    ConflictNotGranted,
    /// The same conflict was declared more than once.
    DuplicateConflict,
    /// The adopted charter policy reference is incomplete or malformed.
    InvalidCharterPolicyReference,
}

/// Why an authorization request was denied.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenialReasonV1 {
    /// The request itself was malformed.
    InvalidRequest,
    /// The principal identity was not authenticated by the trusted host.
    PrincipalIdentityUnverified,
    /// Membership state is unknown; the evaluator cannot safely infer non-membership.
    MembershipStateUnknown,
    /// The principal does not satisfy the matching assignment's charter policy.
    MembershipNotEligible,
    /// The supplied authority snapshot contains a duplicate assignment ID.
    DuplicateAssignmentId,
    /// A matching assignment violates a local structural invariant.
    InvalidAssignment,
    /// A matching active assignment declares a conflict for this action.
    ConflictOfInterest,
    /// No current assignment grants this exact action in this exact scope.
    NoMatchingAssignment,
}

/// Authorization outcome. A successful result identifies the assignment used.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AuthorizationDecisionV1 {
    /// The action is permitted by this exact assignment.
    Authorized { assignment_id: String },
    /// The action is denied.
    Denied(DenialReasonV1),
}

/// Evaluate a request against an already authenticated authority snapshot.
///
/// Exact scope equality is required. There is no parent/child or federation-to-guild
/// permission inheritance. A matching conflict declaration takes precedence over
/// a potentially overlapping permission grant.
pub fn authorize(
    request: &AuthorizationRequestV1,
    assignments: &[GovernanceAssignmentV1],
) -> AuthorizationDecisionV1 {
    if request.principal_id.trim().is_empty() || request.scope.validate().is_err() {
        return AuthorizationDecisionV1::Denied(DenialReasonV1::InvalidRequest);
    }
    if !request.principal_identity_verified {
        return AuthorizationDecisionV1::Denied(DenialReasonV1::PrincipalIdentityUnverified);
    }
    match request.membership_status {
        MembershipStatusV1::Unknown => {
            return AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipStateUnknown);
        }
        MembershipStatusV1::Suspended | MembershipStatusV1::Exited => {
            return AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible);
        }
        MembershipStatusV1::Active | MembershipStatusV1::OnLeave | MembershipStatusV1::NotMember => {}
    }

    // Treat the supplied records as one authority snapshot. Duplicate stable IDs
    // make that snapshot ambiguous, so no record from it is used.
    let mut seen_ids = HashSet::new();
    for assignment in assignments {
        if !seen_ids.insert(assignment.assignment_id.as_str()) {
            return AuthorizationDecisionV1::Denied(DenialReasonV1::DuplicateAssignmentId);
        }
    }

    let mut conflict_found = false;
    let mut membership_blocked = false;
    let mut allowed_by: Option<&GovernanceAssignmentV1> = None;

    for assignment in assignments {
        if assignment.subject_id != request.principal_id || assignment.scope != request.scope {
            continue;
        }
        if assignment.validate().is_err() {
            return AuthorizationDecisionV1::Denied(DenialReasonV1::InvalidAssignment);
        }

        // Inclusive start, exclusive expiry; revocation takes effect at its timestamp.
        if request.at < assignment.effective_at || request.at >= assignment.expires_at {
            continue;
        }
        if let Some(revoked_at) = assignment.revoked_at {
            if request.at >= revoked_at {
                continue;
            }
        }

        if assignment.actions.contains(&request.action) {
            if !principal_eligibility_allows(
                assignment.principal_eligibility,
                request.membership_status,
            ) {
                membership_blocked = true;
                continue;
            }
            if assignment.conflicts_of_interest.contains(&request.action) {
                conflict_found = true;
            } else if allowed_by.is_none() {
                allowed_by = Some(assignment);
            }
        }
    }

    if conflict_found {
        return AuthorizationDecisionV1::Denied(DenialReasonV1::ConflictOfInterest);
    }
    match allowed_by {
        Some(assignment) => AuthorizationDecisionV1::Authorized {
            assignment_id: assignment.assignment_id.clone(),
        },
        None if membership_blocked => {
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible)
        }
        None => AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment),
    }
}

fn principal_eligibility_allows(
    policy: PrincipalEligibilityPolicyV1,
    status: MembershipStatusV1,
) -> bool {
    match policy {
        PrincipalEligibilityPolicyV1::ActiveMemberRequired => status == MembershipStatusV1::Active,
        PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited => matches!(
            status,
            MembershipStatusV1::Active | MembershipStatusV1::OnLeave | MembershipStatusV1::NotMember
        ),
    }
}

fn require_non_empty(value: &str) -> Result<(), AssignmentErrorV1> {
    if value.trim().is_empty() {
        Err(AssignmentErrorV1::EmptyField)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn guild_scope(guild: &str, domain: &str) -> ScopeV1 {
        ScopeV1::Guild {
            guild_id: guild.to_owned(),
            domain_id: domain.to_owned(),
        }
    }

    fn assignment(
        actions: Vec<GovernanceActionV1>,
        scope: ScopeV1,
    ) -> GovernanceAssignmentV1 {
        GovernanceAssignmentV1 {
            assignment_id: "assignment-1".to_owned(),
            subject_id: "member-1".to_owned(),
            scope,
            actions,
            principal_eligibility: PrincipalEligibilityPolicyV1::ActiveMemberRequired,
            charter_policy: CharterPolicyRefV1 {
                policy_id: "charter:default-governance".to_owned(),
                revision: 1,
                digest: [5; 32],
            },
            grantor_id: "charter-authority".to_owned(),
            display_label: None,
            effective_at: 10,
            expires_at: 30,
            revoked_at: None,
            conflicts_of_interest: Vec::new(),
        }
    }

    fn request(action: GovernanceActionV1, scope: ScopeV1) -> AuthorizationRequestV1 {
        AuthorizationRequestV1 {
            principal_id: "member-1".to_owned(),
            principal_identity_verified: true,
            membership_status: MembershipStatusV1::Active,
            action,
            scope,
            at: 20,
        }
    }

    #[test]
    fn exact_scope_assignment_authorizes_declared_action() {
        let a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        assert_eq!(
            authorize(&request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings")), &[a]),
            AuthorizationDecisionV1::Authorized { assignment_id: "assignment-1".to_owned() }
        );
    }

    #[test]
    fn scope_mismatch_does_not_authorize() {
        let a = assignment(vec![GovernanceActionV1::AdmitMember], guild_scope("guild-a", "membership"));
        assert_eq!(
            authorize(&request(GovernanceActionV1::AdmitMember, guild_scope("guild-b", "membership")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn federation_assignment_does_not_inherit_into_guild_scope() {
        let a = assignment(
            vec![GovernanceActionV1::CreateFederationAgreement],
            ScopeV1::Federation { federation_id: "federation-a".to_owned(), domain_id: "federation-charter".to_owned() },
        );
        assert_eq!(
            authorize(&request(GovernanceActionV1::CreateFederationAgreement, guild_scope("guild-a", "charter")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn facilitator_title_does_not_grant_treasury_actions() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.display_label = Some("Elder".to_owned());
        assert_eq!(
            authorize(&request(GovernanceActionV1::AuthorizeDisbursement, guild_scope("guild-a", "meetings")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn founder_provenance_and_honorific_are_not_permissions() {
        let presentation = MemberPresentationV1 {
            member_id: "member-1".to_owned(),
            honorific: Some("Elder".to_owned()),
            founder_provenance: true,
        };
        assert!(presentation.founder_provenance);
        assert_eq!(
            authorize(&request(GovernanceActionV1::AmendGuildCharter, guild_scope("guild-a", "constitution")), &[]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn suspended_members_cannot_use_a_valid_assignment() {
        let a = assignment(vec![GovernanceActionV1::AdmitMember], guild_scope("guild-a", "membership"));
        let mut r = request(GovernanceActionV1::AdmitMember, guild_scope("guild-a", "membership"));
        r.membership_status = MembershipStatusV1::Suspended;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible)
        );
    }

    #[test]
    fn assignment_requires_a_pinned_adopted_charter_policy_reference() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.charter_policy.digest = [0; 32];
        assert_eq!(
            a.validate(),
            Err(AssignmentErrorV1::InvalidCharterPolicyReference)
        );
    }

    #[test]
    fn non_member_is_denied_by_active_member_only_policy() {
        let a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.membership_status = MembershipStatusV1::NotMember;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible)
        );
    }

    #[test]
    fn charter_explicitly_authorized_external_facilitator_is_allowed() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.principal_eligibility =
            PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited;
        a.subject_id = "external-facilitator".to_owned();
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.principal_id = "external-facilitator".to_owned();
        r.membership_status = MembershipStatusV1::NotMember;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Authorized {
                assignment_id: "assignment-1".to_owned()
            }
        );
    }

    #[test]
    fn external_facilitator_receives_only_the_explicit_action() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.principal_eligibility =
            PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited;
        a.subject_id = "external-facilitator".to_owned();
        let mut r = request(GovernanceActionV1::AuthorizeDisbursement, guild_scope("guild-a", "meetings"));
        r.principal_id = "external-facilitator".to_owned();
        r.membership_status = MembershipStatusV1::NotMember;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn on_leave_is_denied_by_member_only_policy() {
        let a = assignment(vec![GovernanceActionV1::Propose], guild_scope("guild-a", "projects"));
        let mut r = request(GovernanceActionV1::Propose, guild_scope("guild-a", "projects"));
        r.membership_status = MembershipStatusV1::OnLeave;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible)
        );
    }

    #[test]
    fn on_leave_can_use_explicit_non_suspended_principal_policy() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.principal_eligibility =
            PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited;
        a.subject_id = "member-1".to_owned();
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.membership_status = MembershipStatusV1::OnLeave;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Authorized {
                assignment_id: "assignment-1".to_owned()
            }
        );
    }

    #[test]
    fn exited_principal_is_denied_even_by_broader_policy() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.principal_eligibility =
            PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited;
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.membership_status = MembershipStatusV1::Exited;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipNotEligible)
        );
    }

    #[test]
    fn unknown_membership_state_fails_closed() {
        let mut a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        a.principal_eligibility =
            PrincipalEligibilityPolicyV1::VerifiedPrincipalNotSuspendedOrExited;
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.membership_status = MembershipStatusV1::Unknown;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::MembershipStateUnknown)
        );
    }

    #[test]
    fn unauthenticated_principal_fails_closed() {
        let a = assignment(vec![GovernanceActionV1::Facilitate], guild_scope("guild-a", "meetings"));
        let mut r = request(GovernanceActionV1::Facilitate, guild_scope("guild-a", "meetings"));
        r.principal_identity_verified = false;
        assert_eq!(
            authorize(&r, &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::PrincipalIdentityUnverified)
        );
    }

    #[test]
    fn expiry_is_exclusive() {
        let mut a = assignment(vec![GovernanceActionV1::RecordDecision], guild_scope("guild-a", "records"));
        a.expires_at = 20;
        assert_eq!(
            authorize(&request(GovernanceActionV1::RecordDecision, guild_scope("guild-a", "records")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn revocation_takes_effect_at_its_timestamp() {
        let mut a = assignment(vec![GovernanceActionV1::SetTreasuryPolicy], guild_scope("guild-a", "treasury"));
        a.revoked_at = Some(20);
        assert_eq!(
            authorize(&request(GovernanceActionV1::SetTreasuryPolicy, guild_scope("guild-a", "treasury")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::NoMatchingAssignment)
        );
    }

    #[test]
    fn conflict_of_interest_overrides_an_overlapping_grant() {
        let mut conflicted = assignment(vec![GovernanceActionV1::AuthorizeDisbursement], guild_scope("guild-a", "treasury"));
        conflicted.assignment_id = "conflicted".to_owned();
        conflicted.conflicts_of_interest = vec![GovernanceActionV1::AuthorizeDisbursement];
        let clean = GovernanceAssignmentV1 { assignment_id: "clean".to_owned(), conflicts_of_interest: Vec::new(), ..conflicted.clone() };
        assert_eq!(
            authorize(&request(GovernanceActionV1::AuthorizeDisbursement, guild_scope("guild-a", "treasury")), &[clean, conflicted]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::ConflictOfInterest)
        );
    }

    #[test]
    fn duplicate_assignment_ids_fail_closed() {
        let a = assignment(vec![GovernanceActionV1::Propose], guild_scope("guild-a", "projects"));
        assert_eq!(
            authorize(&request(GovernanceActionV1::Propose, guild_scope("guild-a", "projects")), &[a.clone(), a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::DuplicateAssignmentId)
        );
    }

    #[test]
    fn self_grant_is_invalid() {
        let mut a = assignment(vec![GovernanceActionV1::Propose], guild_scope("guild-a", "projects"));
        a.grantor_id = a.subject_id.clone();
        assert_eq!(a.validate(), Err(AssignmentErrorV1::SelfGrant));
    }

    #[test]
    fn empty_action_sets_are_invalid() {
        let a = assignment(Vec::new(), guild_scope("guild-a", "projects"));
        assert_eq!(a.validate(), Err(AssignmentErrorV1::EmptyActionSet));
    }

    #[test]
    fn invalid_matching_record_denies_the_snapshot() {
        let mut a = assignment(vec![GovernanceActionV1::Propose], guild_scope("guild-a", "projects"));
        a.expires_at = a.effective_at;
        assert_eq!(
            authorize(&request(GovernanceActionV1::Propose, guild_scope("guild-a", "projects")), &[a]),
            AuthorizationDecisionV1::Denied(DenialReasonV1::InvalidAssignment)
        );
    }
}
