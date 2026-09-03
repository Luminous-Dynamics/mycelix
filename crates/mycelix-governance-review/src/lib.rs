// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Governed review boundary between advisory analysis and binding outcomes.
//!
//! An advisory signal may request review. It does not itself alter a tally,
//! revoke a right, change ballot weight, or block finalization indefinitely.
//! Procedural interventions require a pre-existing rulebook-bound review policy
//! plus an explicit institutional authority grant.

use mycelix_institutional_core::{
    evaluate_authority, AdvisorySignalId, AuthorityDecision, AuthorityGrant, AuthorityGrantId,
    AuthorityRequirement, CapabilityId, Digest32, EvidenceRef, InstitutionId, JurisdictionId,
    RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-review-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_CODE_BYTES: usize = 256;

macro_rules! id_type {
    ($name:ident, $field:literal) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ReviewError> {
                let value = value.into();
                validate_text(&value, $field, MAX_ID_BYTES)?;
                Ok(Self(value))
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

id_type!(ReviewPolicyId, "review_policy_id");
id_type!(ReviewRequestId, "review_request_id");
id_type!(ReviewDispositionId, "review_disposition_id");
id_type!(ProposalRef, "proposal_ref");

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewTriggerKind {
    EthicsConcern,
    ConcentrationWarning,
    NarrowMargin,
    CoordinationAnomaly,
    SafetyConcern,
    Custom(String),
}

impl ReviewTriggerKind {
    fn validate(&self) -> Result<(), ReviewError> {
        if let Self::Custom(code) = self {
            validate_text(code, "review_trigger.custom", MAX_CODE_BYTES)?;
        }
        Ok(())
    }
}

/// Pre-committed procedural policy adopted by the institution.
///
/// Advisory systems do not choose this policy at run time. They may only emit a
/// trigger type that the already-adopted rulebook/policy recognizes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReviewPolicy {
    pub protocol_version: String,
    pub id: ReviewPolicyId,
    pub policy_digest: Digest32,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    /// Authority capability required to authorize a procedural intervention.
    pub reviewer_capability: CapabilityId,
    pub allowed_triggers: Vec<ReviewTriggerKind>,
    /// Hard ceiling for any cooling period authorized under this policy.
    pub max_cooling_period_ms: u64,
    /// Hard ceiling for an independent review deadline.
    pub max_review_period_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Institutional grant/decision that adopted this procedural policy.
    pub adopted_by: AuthorityGrantId,
    pub policy_proof_ref: String,
}

impl ReviewPolicy {
    pub fn validate(&self) -> Result<(), ReviewError> {
        require_protocol(&self.protocol_version)?;
        require_digest(self.policy_digest, "review_policy.digest")?;
        self.rulebook.validate().map_err(|_| ReviewError::InvalidRulebook)?;
        if self.allowed_triggers.is_empty() {
            return Err(ReviewError::NoAllowedTriggers);
        }
        for trigger in &self.allowed_triggers {
            trigger.validate()?;
        }
        if self.max_cooling_period_ms == 0 || self.max_review_period_ms == 0 {
            return Err(ReviewError::ZeroProcedureLimit);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(ReviewError::InvalidTimeRange("review policy"));
        }
        validate_text(
            &self.policy_proof_ref,
            "review_policy.policy_proof_ref",
            MAX_ID_BYTES,
        )
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }
}

/// Non-binding request produced from an advisory finding.
///
/// The request records *why review was requested* and the exact tally/outcome it
/// refers to. Merely creating this object does not change governance state.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdvisoryReviewRequest {
    pub protocol_version: String,
    pub id: ReviewRequestId,
    pub proposal: ProposalRef,
    pub binding_tally_digest: Digest32,
    pub advisory_signal_id: AdvisorySignalId,
    pub trigger: ReviewTriggerKind,
    pub explanation_ref: String,
    pub requested_at_ms: u64,
    pub expires_at_ms: u64,
}

impl AdvisoryReviewRequest {
    pub fn validate(&self) -> Result<(), ReviewError> {
        require_protocol(&self.protocol_version)?;
        require_digest(self.binding_tally_digest, "review_request.tally_digest")?;
        self.trigger.validate()?;
        validate_text(
            &self.explanation_ref,
            "review_request.explanation_ref",
            MAX_ID_BYTES,
        )?;
        if self.requested_at_ms == 0 || self.expires_at_ms <= self.requested_at_ms {
            return Err(ReviewError::InvalidTimeRange("review request"));
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.requested_at_ms <= now_ms && now_ms < self.expires_at_ms
    }
}

/// Procedures an authorized reviewer may invoke.
///
/// There is intentionally no `ChangeBallotWeight`, `RewriteTally`,
/// `RemoveVoterRight`, or `RejectProposalBecauseModelSaidSo` variant.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewDisposition {
    /// Record the advisory finding and leave finalization unchanged.
    DisclosureOnly,
    /// Time-bounded pause. The original binding tally remains immutable.
    CoolingPeriod { until_ms: u64 },
    /// Route to an institution/forum for independent review by an explicit deadline.
    IndependentReview {
        forum: InstitutionId,
        deadline_ms: u64,
    },
    /// Require a new ballot under a separately referenced, pre-existing policy.
    /// The original tally remains part of history; it is never rewritten.
    SecondBallot {
        successor_proposal: ProposalRef,
        starts_after_ms: u64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizedReviewDisposition {
    pub protocol_version: String,
    pub id: ReviewDispositionId,
    pub request_id: ReviewRequestId,
    pub proposal: ProposalRef,
    pub original_binding_tally_digest: Digest32,
    pub advisory_signal_id: AdvisorySignalId,
    pub policy_id: ReviewPolicyId,
    pub policy_digest: Digest32,
    pub disposition: ReviewDisposition,
    pub authorized_by: AuthorityGrantId,
    pub authorized_at_ms: u64,
    pub expires_at_ms: u64,
    pub authorization_proof_ref: String,
}

impl AuthorizedReviewDisposition {
    pub fn validate(&self) -> Result<(), ReviewError> {
        require_protocol(&self.protocol_version)?;
        require_digest(
            self.original_binding_tally_digest,
            "review_disposition.tally_digest",
        )?;
        require_digest(self.policy_digest, "review_disposition.policy_digest")?;
        if self.authorized_at_ms == 0 || self.expires_at_ms <= self.authorized_at_ms {
            return Err(ReviewError::InvalidTimeRange("review disposition"));
        }
        validate_text(
            &self.authorization_proof_ref,
            "review_disposition.authorization_proof_ref",
            MAX_ID_BYTES,
        )
    }
}

/// Authorize a procedural response to an advisory review request.
///
/// The advisory finding alone is insufficient. This function first validates
/// the institution's pre-committed review policy, then requires an explicit
/// authority grant with the policy's reviewer capability under the same
/// institution/jurisdiction/rulebook.
pub fn authorize_review_disposition(
    policy: &ReviewPolicy,
    request: &AdvisoryReviewRequest,
    reviewer_grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
    disposition_id: ReviewDispositionId,
    disposition: ReviewDisposition,
    authorization_proof_ref: String,
    now_ms: u64,
) -> Result<AuthorizedReviewDisposition, ReviewError> {
    policy.validate()?;
    request.validate()?;
    if !policy.is_active_at(now_ms) {
        return Err(ReviewError::PolicyInactive);
    }
    if !request.is_active_at(now_ms) {
        return Err(ReviewError::RequestInactive);
    }
    if !policy.allowed_triggers.contains(&request.trigger) {
        return Err(ReviewError::TriggerNotAuthorized);
    }

    let authority_requirement = AuthorityRequirement {
        institution: policy.institution.clone(),
        jurisdiction: policy.jurisdiction.clone(),
        required_capabilities: vec![policy.reviewer_capability.clone()],
        accepted_roles: vec![],
        evidence: vec![],
        rulebook: policy.rulebook.clone(),
    };

    let granted = match evaluate_authority(
        reviewer_grant,
        &authority_requirement,
        authority_evidence,
        now_ms,
    ) {
        AuthorityDecision::Allow(granted) => granted,
        AuthorityDecision::Deny(denial) => {
            return Err(ReviewError::ReviewerNotAuthorized(denial.reason_code));
        }
        AuthorityDecision::NeedsEvidence(_) => {
            return Err(ReviewError::ReviewerNeedsEvidence);
        }
    };

    validate_disposition(policy, request, &disposition, now_ms)?;
    validate_text(
        &authorization_proof_ref,
        "review_disposition.authorization_proof_ref",
        MAX_ID_BYTES,
    )?;

    let expires_at_ms = match &disposition {
        ReviewDisposition::DisclosureOnly => request.expires_at_ms,
        ReviewDisposition::CoolingPeriod { until_ms } => *until_ms,
        ReviewDisposition::IndependentReview { deadline_ms, .. } => *deadline_ms,
        ReviewDisposition::SecondBallot {
            starts_after_ms, ..
        } => *starts_after_ms,
    }
    .min(request.expires_at_ms)
    .min(policy.valid_until_ms);

    if expires_at_ms <= now_ms {
        return Err(ReviewError::DispositionAlreadyExpired);
    }

    Ok(AuthorizedReviewDisposition {
        protocol_version: PROTOCOL_VERSION.into(),
        id: disposition_id,
        request_id: request.id.clone(),
        proposal: request.proposal.clone(),
        original_binding_tally_digest: request.binding_tally_digest,
        advisory_signal_id: request.advisory_signal_id.clone(),
        policy_id: policy.id.clone(),
        policy_digest: policy.policy_digest,
        disposition,
        authorized_by: granted.grant_id,
        authorized_at_ms: now_ms,
        expires_at_ms,
        authorization_proof_ref,
    })
}

fn validate_disposition(
    policy: &ReviewPolicy,
    request: &AdvisoryReviewRequest,
    disposition: &ReviewDisposition,
    now_ms: u64,
) -> Result<(), ReviewError> {
    match disposition {
        ReviewDisposition::DisclosureOnly => Ok(()),
        ReviewDisposition::CoolingPeriod { until_ms } => {
            if *until_ms <= now_ms {
                return Err(ReviewError::InvalidTimeRange("cooling period"));
            }
            let duration = until_ms
                .checked_sub(now_ms)
                .ok_or(ReviewError::TimeOverflow)?;
            if duration > policy.max_cooling_period_ms {
                return Err(ReviewError::CoolingPeriodTooLong);
            }
            if *until_ms > request.expires_at_ms {
                return Err(ReviewError::ProcedureOutlivesRequest);
            }
            Ok(())
        }
        ReviewDisposition::IndependentReview { forum, deadline_ms } => {
            validate_text(forum.as_str(), "review_disposition.forum", MAX_ID_BYTES)?;
            if *deadline_ms <= now_ms {
                return Err(ReviewError::InvalidTimeRange("independent review"));
            }
            let duration = deadline_ms
                .checked_sub(now_ms)
                .ok_or(ReviewError::TimeOverflow)?;
            if duration > policy.max_review_period_ms {
                return Err(ReviewError::ReviewPeriodTooLong);
            }
            if *deadline_ms > request.expires_at_ms {
                return Err(ReviewError::ProcedureOutlivesRequest);
            }
            Ok(())
        }
        ReviewDisposition::SecondBallot {
            successor_proposal,
            starts_after_ms,
        } => {
            validate_text(
                successor_proposal.as_str(),
                "review_disposition.successor_proposal",
                MAX_ID_BYTES,
            )?;
            if successor_proposal == &request.proposal {
                return Err(ReviewError::SuccessorMustBeDistinct);
            }
            if *starts_after_ms < now_ms {
                return Err(ReviewError::InvalidTimeRange("second ballot"));
            }
            if *starts_after_ms > request.expires_at_ms {
                return Err(ReviewError::ProcedureOutlivesRequest);
            }
            Ok(())
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReviewError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    ZeroDigest(&'static str),
    InvalidRulebook,
    NoAllowedTriggers,
    ZeroProcedureLimit,
    InvalidTimeRange(&'static str),
    PolicyInactive,
    RequestInactive,
    TriggerNotAuthorized,
    ReviewerNotAuthorized(String),
    ReviewerNeedsEvidence,
    CoolingPeriodTooLong,
    ReviewPeriodTooLong,
    ProcedureOutlivesRequest,
    SuccessorMustBeDistinct,
    DispositionAlreadyExpired,
    TimeOverflow,
}

impl fmt::Display for ReviewError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be zero"),
            Self::InvalidRulebook => write!(f, "invalid rulebook reference"),
            Self::NoAllowedTriggers => write!(f, "review policy has no allowed triggers"),
            Self::ZeroProcedureLimit => write!(f, "review procedure limits must be positive"),
            Self::InvalidTimeRange(field) => write!(f, "invalid time range for {field}"),
            Self::PolicyInactive => write!(f, "review policy is inactive"),
            Self::RequestInactive => write!(f, "review request is inactive"),
            Self::TriggerNotAuthorized => write!(f, "advisory trigger is not authorized by review policy"),
            Self::ReviewerNotAuthorized(reason) => write!(f, "reviewer is not authorized: {reason}"),
            Self::ReviewerNeedsEvidence => write!(f, "reviewer authority needs additional evidence"),
            Self::CoolingPeriodTooLong => write!(f, "cooling period exceeds policy limit"),
            Self::ReviewPeriodTooLong => write!(f, "independent review exceeds policy limit"),
            Self::ProcedureOutlivesRequest => write!(f, "review procedure outlives its review request"),
            Self::SuccessorMustBeDistinct => write!(f, "second ballot must use a distinct successor proposal"),
            Self::DispositionAlreadyExpired => write!(f, "authorized disposition would already be expired"),
            Self::TimeOverflow => write!(f, "timestamp arithmetic overflow"),
        }
    }
}

impl std::error::Error for ReviewError {}

fn require_protocol(version: &str) -> Result<(), ReviewError> {
    if version == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(ReviewError::WrongProtocolVersion)
    }
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), ReviewError> {
    if digest.is_zero() {
        Err(ReviewError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

fn validate_text(value: &str, field: &'static str, max: usize) -> Result<(), ReviewError> {
    if value.trim().is_empty() {
        return Err(ReviewError::Empty(field));
    }
    if value.len() > max {
        return Err(ReviewError::TooLong(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{
        AuthorityGrantId, AuthoritySourceKind, AuthoritySourceRef, CapabilityId, PrincipalId,
        RoleId, RulebookId,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn institution() -> InstitutionId {
        InstitutionId::new("institution:community").unwrap()
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:community:v1").unwrap(),
            version: "1.0.0".into(),
            digest: digest(1),
        }
    }

    fn policy() -> ReviewPolicy {
        ReviewPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ReviewPolicyId::new("review-policy:v1").unwrap(),
            policy_digest: digest(2),
            institution: institution(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            rulebook: rulebook(),
            reviewer_capability: CapabilityId::new("capability:governance-review").unwrap(),
            allowed_triggers: vec![
                ReviewTriggerKind::EthicsConcern,
                ReviewTriggerKind::ConcentrationWarning,
            ],
            max_cooling_period_ms: 72 * 60 * 60 * 1000,
            max_review_period_ms: 14 * 24 * 60 * 60 * 1000,
            valid_from_ms: 1_000,
            valid_until_ms: 50_000_000_000,
            adopted_by: AuthorityGrantId::new("grant:policy-adoption").unwrap(),
            policy_proof_ref: "proof:policy-adoption".into(),
        }
    }

    fn request() -> AdvisoryReviewRequest {
        AdvisoryReviewRequest {
            protocol_version: PROTOCOL_VERSION.into(),
            id: ReviewRequestId::new("review-request:1").unwrap(),
            proposal: ProposalRef::new("MIP-200").unwrap(),
            binding_tally_digest: digest(3),
            advisory_signal_id: AdvisorySignalId::new("advisory:ethics:1").unwrap(),
            trigger: ReviewTriggerKind::EthicsConcern,
            explanation_ref: "artifact:ethics-analysis".into(),
            requested_at_ms: 5_000,
            expires_at_ms: 20_000_000,
        }
    }

    fn reviewer_grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: mycelix_institutional_core::PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:reviewer").unwrap(),
            holder: PrincipalId::new("did:example:reviewer").unwrap(),
            institution: institution(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:local").unwrap()),
            roles: vec![RoleId::new("role:reviewer").unwrap()],
            capabilities: vec![CapabilityId::new("capability:governance-review").unwrap()],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "decision:appoint-reviewer".into(),
                proof_ref: "proof:appointment".into(),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 30_000_000,
            delegated_from: None,
            grant_proof_ref: "proof:reviewer-grant".into(),
        }
    }

    #[test]
    fn advisory_request_is_non_binding_by_itself() {
        // A request validates, but there is no `blocks_advancement()` or tally
        // mutation method on it. Authority is required to produce a disposition.
        assert!(request().validate().is_ok());
    }

    #[test]
    fn authorized_reviewer_can_create_bounded_cooling_period() {
        let now = 10_000;
        let result = authorize_review_disposition(
            &policy(),
            &request(),
            &reviewer_grant(),
            &[],
            ReviewDispositionId::new("disposition:1").unwrap(),
            ReviewDisposition::CoolingPeriod {
                until_ms: now + 60_000,
            },
            "proof:review-authorization".into(),
            now,
        )
        .unwrap();
        assert_eq!(result.original_binding_tally_digest, digest(3));
        assert_eq!(result.authorized_by.as_str(), "grant:reviewer");
    }

    #[test]
    fn wrong_capability_cannot_turn_advisory_into_procedure() {
        let mut grant = reviewer_grant();
        grant.capabilities = vec![CapabilityId::new("capability:unrelated").unwrap()];
        assert!(matches!(
            authorize_review_disposition(
                &policy(),
                &request(),
                &grant,
                &[],
                ReviewDispositionId::new("disposition:2").unwrap(),
                ReviewDisposition::DisclosureOnly,
                "proof:review-authorization".into(),
                10_000,
            ),
            Err(ReviewError::ReviewerNotAuthorized(_))
        ));
    }

    #[test]
    fn model_signal_cannot_create_unbounded_cooling_period() {
        let now = 10_000;
        let too_long = policy().max_cooling_period_ms + 1;
        assert_eq!(
            authorize_review_disposition(
                &policy(),
                &request(),
                &reviewer_grant(),
                &[],
                ReviewDispositionId::new("disposition:3").unwrap(),
                ReviewDisposition::CoolingPeriod {
                    until_ms: now + too_long,
                },
                "proof:review-authorization".into(),
                now,
            )
            .unwrap_err(),
            ReviewError::CoolingPeriodTooLong
        );
    }

    #[test]
    fn unrecognized_trigger_cannot_invoke_review_policy() {
        let mut request = request();
        request.trigger = ReviewTriggerKind::NarrowMargin;
        assert_eq!(
            authorize_review_disposition(
                &policy(),
                &request,
                &reviewer_grant(),
                &[],
                ReviewDispositionId::new("disposition:4").unwrap(),
                ReviewDisposition::DisclosureOnly,
                "proof:review-authorization".into(),
                10_000,
            )
            .unwrap_err(),
            ReviewError::TriggerNotAuthorized
        );
    }

    #[test]
    fn second_ballot_preserves_original_tally_identity() {
        let result = authorize_review_disposition(
            &policy(),
            &request(),
            &reviewer_grant(),
            &[],
            ReviewDispositionId::new("disposition:5").unwrap(),
            ReviewDisposition::SecondBallot {
                successor_proposal: ProposalRef::new("MIP-200-R1").unwrap(),
                starts_after_ms: 20_000,
            },
            "proof:review-authorization".into(),
            10_000,
        )
        .unwrap();
        assert_eq!(result.original_binding_tally_digest, digest(3));
        assert_eq!(result.proposal.as_str(), "MIP-200");
    }

    #[test]
    fn wire_round_trip_preserves_review_authority() {
        let policy = policy();
        let json = serde_json::to_string(&policy).unwrap();
        let decoded: ReviewPolicy = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, policy);
    }
}
