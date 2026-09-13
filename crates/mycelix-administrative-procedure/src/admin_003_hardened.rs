// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Public ADMIN-003 review-lineage hardening.
//!
//! The semantic ADMIN-003 module keeps core review records intentionally small.
//! This facade adds two proof-carrying facts needed by the public path:
//! monotonic stay-transition history and immutable provenance for the original
//! decision authority when independent review is required.

use super::admin_003;
use mycelix_institutional_core::{
    Appeal, AuthorityGrant, AuthorityGrantId, EvidenceRef, PrincipalId,
};
use std::fmt;

pub use admin_003::{StayDirective, StayDirectiveKind, StayState};

/// Opaque challenge token retaining the exact original decision-authority
/// provenance that would otherwise disappear behind the thinner Challenge type.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedChallenge {
    inner: admin_003::QualifiedChallenge,
    original_decider: PrincipalId,
    original_authority_grant_id: AuthorityGrantId,
    require_independent_reviewer: bool,
}

impl QualifiedChallenge {
    pub fn challenge(&self) -> &mycelix_institutional_core::Challenge {
        self.inner.challenge()
    }

    pub fn original_decision_id(&self) -> &mycelix_institutional_core::DecisionId {
        self.inner.original_decision_id()
    }

    pub fn original_decider(&self) -> &PrincipalId {
        &self.original_decider
    }

    pub fn original_authority_grant_id(&self) -> &AuthorityGrantId {
        &self.original_authority_grant_id
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_challenge(
    reviewable: admin_003::QualifiedReviewableDecision,
    submission: admin_003::ChallengeSubmission,
) -> Result<QualifiedChallenge, AdministrativeReviewHardeningError> {
    let original_decider = reviewable.issued().decider().clone();
    let original_authority_grant_id = reviewable.issued().authority_grant_id().clone();
    let require_independent_reviewer = reviewable.policy().require_independent_reviewer;
    let inner = admin_003::qualify_challenge(reviewable, submission)?;
    Ok(QualifiedChallenge {
        inner,
        original_decider,
        original_authority_grant_id,
        require_independent_reviewer,
    })
}

/// Opaque appeal-review token carrying original-authority provenance plus
/// monotonic stay-transition history.
///
/// These fields are deliberately not serialized. Persisted review state must be
/// rebuilt from the exact decision/challenge/appeal/stay lineage instead of
/// accepting a deserialized current-state snapshot as sufficient proof.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAppealReview {
    inner: admin_003::QualifiedAppealReview,
    original_decider: PrincipalId,
    original_authority_grant_id: AuthorityGrantId,
    require_independent_reviewer: bool,
    last_stay_transition_ms: Option<u64>,
    active_stay_imposed_at_ms: Option<u64>,
}

impl QualifiedAppealReview {
    pub fn appeal(&self) -> &Appeal {
        self.inner.appeal()
    }

    pub fn stay_state(&self) -> StayState {
        self.inner.stay_state()
    }

    pub fn original_decider(&self) -> &PrincipalId {
        &self.original_decider
    }

    pub fn original_authority_grant_id(&self) -> &AuthorityGrantId {
        &self.original_authority_grant_id
    }

    pub fn last_stay_transition_ms(&self) -> Option<u64> {
        self.last_stay_transition_ms
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_appeal(
    challenge: QualifiedChallenge,
    appeal: Appeal,
) -> Result<QualifiedAppealReview, AdministrativeReviewHardeningError> {
    let original_decider = challenge.original_decider.clone();
    let original_authority_grant_id = challenge.original_authority_grant_id.clone();
    let require_independent_reviewer = challenge.require_independent_reviewer;
    let inner = admin_003::qualify_appeal(challenge.inner, appeal)?;
    Ok(QualifiedAppealReview {
        inner,
        original_decider,
        original_authority_grant_id,
        require_independent_reviewer,
        last_stay_transition_ms: None,
        active_stay_imposed_at_ms: None,
    })
}

/// Apply a stay directive only when its event time is monotonic with the exact
/// previously qualified stay history.
///
/// This closes a subtle state-snapshot ambiguity: `StayedUntil(t)` alone does
/// not reveal when the stay was imposed, so a backdated lift could otherwise
/// appear valid as long as it followed the appeal filing.
pub fn apply_stay_directive(
    mut review: QualifiedAppealReview,
    directive: StayDirective,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedAppealReview, AdministrativeReviewHardeningError> {
    let event_at_ms = directive.issued_at_ms;
    let kind = directive.kind;

    if let Some(previous_at_ms) = review.last_stay_transition_ms {
        if event_at_ms < previous_at_ms {
            return Err(AdministrativeReviewHardeningError::StayTimeRegression);
        }
    }
    if matches!(kind, StayDirectiveKind::Lift) {
        let imposed_at_ms = review
            .active_stay_imposed_at_ms
            .ok_or(AdministrativeReviewHardeningError::NoQualifiedActiveStayHistory)?;
        if event_at_ms < imposed_at_ms {
            return Err(AdministrativeReviewHardeningError::StayTimeRegression);
        }
    }

    review.inner = admin_003::apply_stay_directive(
        review.inner,
        directive,
        grant,
        authority_evidence,
    )?;
    review.last_stay_transition_ms = Some(event_at_ms);
    review.active_stay_imposed_at_ms = match kind {
        StayDirectiveKind::Impose => Some(event_at_ms),
        StayDirectiveKind::Lift => None,
    };
    Ok(review)
}

pub fn qualify_review_disposition(
    review: QualifiedAppealReview,
    disposition: admin_003::AdministrativeReviewDisposition,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<admin_003::QualifiedReviewDisposition, AdministrativeReviewHardeningError> {
    if review.require_independent_reviewer && grant.id == review.original_authority_grant_id {
        return Err(AdministrativeReviewHardeningError::ReviewerGrantNotIndependent);
    }
    admin_003::qualify_review_disposition(review.inner, disposition, grant, authority_evidence)
        .map_err(Into::into)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdministrativeReviewHardeningError {
    Review(admin_003::AdministrativeReviewError),
    StayTimeRegression,
    NoQualifiedActiveStayHistory,
    ReviewerGrantNotIndependent,
}

impl From<admin_003::AdministrativeReviewError> for AdministrativeReviewHardeningError {
    fn from(value: admin_003::AdministrativeReviewError) -> Self {
        Self::Review(value)
    }
}

impl fmt::Display for AdministrativeReviewHardeningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Review(error) => write!(f, "{error}"),
            Self::StayTimeRegression => {
                write!(f, "stay directive time regresses behind qualified stay history")
            }
            Self::NoQualifiedActiveStayHistory => {
                write!(f, "stay lift lacks a qualified active-stay history")
            }
            Self::ReviewerGrantNotIndependent => write!(
                f,
                "independent review cannot reuse the original decision authority grant"
            ),
        }
    }
}

impl std::error::Error for AdministrativeReviewHardeningError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hardening_errors_are_explicit() {
        assert_eq!(
            AdministrativeReviewHardeningError::StayTimeRegression.to_string(),
            "stay directive time regresses behind qualified stay history"
        );
        assert_eq!(
            AdministrativeReviewHardeningError::ReviewerGrantNotIndependent.to_string(),
            "independent review cannot reuse the original decision authority grant"
        );
    }
}
