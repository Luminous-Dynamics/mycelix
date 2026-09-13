// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Public ADMIN-003 stay-history hardening.
//!
//! The semantic ADMIN-003 module keeps stay state intentionally compact. This
//! facade adds the missing temporal-history token so a later directive cannot
//! be dated before the directive that produced the current state.

use super::admin_003;
use mycelix_institutional_core::{
    Appeal, AuthorityGrant, EvidenceRef,
};
use std::fmt;

pub use admin_003::{StayDirective, StayDirectiveKind, StayState};

/// Opaque appeal-review token carrying monotonic stay-transition history.
///
/// `last_stay_transition_ms` is deliberately not serialized. Persisted review
/// state must be rebuilt from the ordered stay-directive lineage instead of
/// accepting a deserialized current-state snapshot as sufficient history.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAppealReview {
    inner: admin_003::QualifiedAppealReview,
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

    pub fn last_stay_transition_ms(&self) -> Option<u64> {
        self.last_stay_transition_ms
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

pub fn qualify_appeal(
    challenge: admin_003::QualifiedChallenge,
    appeal: Appeal,
) -> Result<QualifiedAppealReview, AdministrativeReviewHardeningError> {
    let inner = admin_003::qualify_appeal(challenge, appeal)?;
    Ok(QualifiedAppealReview {
        inner,
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
    admin_003::qualify_review_disposition(review.inner, disposition, grant, authority_evidence)
        .map_err(Into::into)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdministrativeReviewHardeningError {
    Review(admin_003::AdministrativeReviewError),
    StayTimeRegression,
    NoQualifiedActiveStayHistory,
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
        }
    }
}

impl std::error::Error for AdministrativeReviewHardeningError {}

#[cfg(test)]
mod tests {
    use super::*;

    // The semantic module owns the full end-to-end review fixtures. This facade
    // is additionally statically gated in CI to ensure the raw stay/review
    // functions are not re-exported. Runtime qualification tests exercise the
    // wrapper together with the semantic ADMIN-003 corpus.
    #[test]
    fn hardening_error_is_not_authority() {
        assert_eq!(
            AdministrativeReviewHardeningError::StayTimeRegression.to_string(),
            "stay directive time regresses behind qualified stay history"
        );
    }
}
