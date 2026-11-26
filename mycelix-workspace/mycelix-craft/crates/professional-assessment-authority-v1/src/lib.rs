//! Independent, scoped authority for professional capability assessment.
//!
//! This policy core is independent of guild membership and governance office.
//! It evaluates an already-authenticated snapshot of assessment authorities.
//! The host must verify issuer/grantor eligibility, signatures, key binding,
//! authority provenance, status freshness and jurisdictional recognition before
//! admitting records to that snapshot.
//!
//! Authorization to assess is not proof that an assessment is correct, that work
//! is safe, or that a statutory licence exists.

#![forbid(unsafe_code)]

use std::collections::HashSet;

/// Exact professional scope. All fields must match; no taxonomy or jurisdiction
/// inheritance is implied by this reference evaluator.
#[derive(Clone, Debug, Eq, PartialEq, Hash)]
pub struct AssessmentScopeV1 {
    /// Capability or occupational scope identifier from a pinned taxonomy/profile.
    pub capability_scope_id: String,
    /// Assessment method or procedure identifier.
    pub method_id: String,
    /// Jurisdiction in which the authority is intended to apply.
    pub jurisdiction_id: String,
    /// Standards or assessment profile identifier.
    pub standards_profile_id: String,
    /// Exact standards/profile revision.
    pub standards_revision: String,
    /// Digest of the exact standards/profile document.
    pub standards_digest: [u8; 32],
}

impl AssessmentScopeV1 {
    fn validate(&self) -> Result<(), AuthorityErrorV1> {
        require_non_empty(&self.capability_scope_id)?;
        require_non_empty(&self.method_id)?;
        require_non_empty(&self.jurisdiction_id)?;
        require_non_empty(&self.standards_profile_id)?;
        require_non_empty(&self.standards_revision)?;
        require_digest(&self.standards_digest)
    }
}

/// Lifecycle status supplied by the authority's trusted status source.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthorityStatusV1 {
    /// May authorize assessments inside its exact scope and validity window.
    Active,
    /// Temporarily unavailable for new assessments.
    Suspended,
    /// Withdrawn by the authority owner.
    Revoked,
}

/// Versioned permission for one assessor to use one assessment scope.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProfessionalAssessmentAuthorityV1 {
    /// Stable authority identifier.
    pub authority_id: String,
    /// Monotonically increasing authority revision.
    pub revision: u64,
    /// Assessor identity receiving this authority.
    pub assessor_id: String,
    /// Organization or jurisdictional body that issued the authority.
    pub issuer_id: String,
    /// Identity that granted the authority; eligibility is verified upstream.
    pub grantor_id: String,
    /// Exact authorized professional assessment scope.
    pub scope: AssessmentScopeV1,
    /// Current status from the trusted authority snapshot.
    pub status: AuthorityStatusV1,
    /// Inclusive start timestamp (Unix seconds).
    pub effective_at: i64,
    /// Exclusive expiry timestamp (Unix seconds).
    pub expires_at: i64,
    /// Timestamp at which revocation takes effect, if any.
    pub revoked_at: Option<i64>,
    /// Parties with whom this assessor has a declared conflict for this authority.
    pub conflict_party_ids: Vec<String>,
    /// Digest of the signed/authoritative source document.
    pub source_document_digest: [u8; 32],
}

impl ProfessionalAssessmentAuthorityV1 {
    /// Validate the authority's local structure.
    ///
    /// This does not authenticate the issuer, grantor, signature or source status.
    pub fn validate(&self) -> Result<(), AuthorityErrorV1> {
        require_non_empty(&self.authority_id)?;
        require_non_empty(&self.assessor_id)?;
        require_non_empty(&self.issuer_id)?;
        require_non_empty(&self.grantor_id)?;

        if self.grantor_id == self.assessor_id {
            return Err(AuthorityErrorV1::SelfGrant);
        }
        if self.revision == 0 {
            return Err(AuthorityErrorV1::InvalidRevision);
        }
        self.scope.validate()?;
        if self.expires_at <= self.effective_at {
            return Err(AuthorityErrorV1::InvalidTimeRange);
        }
        if let Some(revoked_at) = self.revoked_at {
            if revoked_at < self.effective_at {
                return Err(AuthorityErrorV1::InvalidRevocationTime);
            }
        }
        require_digest(&self.source_document_digest)?;

        let mut parties = HashSet::new();
        for party_id in &self.conflict_party_ids {
            require_non_empty(party_id)?;
            if !parties.insert(party_id.as_str()) {
                return Err(AuthorityErrorV1::DuplicateConflictParty);
            }
        }
        Ok(())
    }
}

/// Request to authorize one assessment.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssessmentAuthorizationRequestV1 {
    /// Identity of the person conducting the assessment.
    pub assessor_id: String,
    /// Person whose capability is being assessed.
    pub subject_id: String,
    /// Other parties whose involvement is relevant to conflict screening.
    /// This list must include the subject.
    pub involved_party_ids: Vec<String>,
    /// Exact requested assessment scope.
    pub scope: AssessmentScopeV1,
    /// Evaluation time (Unix seconds).
    pub at: i64,
}

impl AssessmentAuthorizationRequestV1 {
    fn validate(&self) -> Result<(), DenialReasonV1> {
        if !is_non_empty_unpadded(&self.assessor_id)
            || !is_non_empty_unpadded(&self.subject_id)
            || self.scope.validate().is_err()
        {
            return Err(DenialReasonV1::InvalidRequest);
        }
        if self.assessor_id == self.subject_id {
            return Err(DenialReasonV1::SelfAssessment);
        }

        let mut parties = HashSet::new();
        for party_id in &self.involved_party_ids {
            if !is_non_empty_unpadded(party_id) || !parties.insert(party_id.as_str()) {
                return Err(DenialReasonV1::InvalidRequest);
            }
        }
        if !parties.contains(self.subject_id.as_str()) {
            return Err(DenialReasonV1::InvalidRequest);
        }
        Ok(())
    }
}

/// Structural errors in a single authority record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthorityErrorV1 {
    /// Required identifier is blank.
    EmptyField,
    /// A grantor cannot grant assessment authority to themself.
    SelfGrant,
    /// Authority revision must be greater than zero.
    InvalidRevision,
    /// Expiry must be strictly after the effective time.
    InvalidTimeRange,
    /// Revocation may not predate the effective time.
    InvalidRevocationTime,
    /// A required source or standards digest is missing.
    MissingDigest,
    /// Conflict party identifier is duplicated.
    DuplicateConflictParty,
}

/// Why an assessment request was denied.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DenialReasonV1 {
    /// The request is malformed or incomplete.
    InvalidRequest,
    /// A person may not independently assess their own capability.
    SelfAssessment,
    /// The authority snapshot contains duplicate stable identifiers.
    DuplicateAuthorityId,
    /// A matching authority record violates a local invariant.
    InvalidAuthority,
    /// A matching authority is suspended or revoked.
    AuthorityNotActive,
    /// A matching authority is outside its effective/expiry/revocation window.
    AuthorityOutsideValidity,
    /// The assessor or assessment scope is not authorized.
    NoMatchingAuthority,
    /// A declared conflict intersects the assessment parties.
    ConflictOfInterest,
}

/// Authorization result. It identifies the exact authority used.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AssessmentAuthorizationDecisionV1 {
    /// The assessor may perform this assessment under this authority revision.
    Authorized {
        authority_id: String,
        authority_revision: u64,
    },
    /// No authority permits the requested assessment.
    Denied(DenialReasonV1),
}

/// Authorize assessment using a trusted, current authority snapshot.
///
/// No membership status, guild role, founder provenance, honorific, reputation,
/// seniority, economic balance or federation office is accepted as an input.
/// External assessors can therefore be authorized without joining a guild.
///
/// Exact scope equality is required. The host remains responsible for atomic
/// attestation/replay reservation and for verifying all issuer-side trust inputs.
pub fn authorize_assessment(
    request: &AssessmentAuthorizationRequestV1,
    authorities: &[ProfessionalAssessmentAuthorityV1],
) -> AssessmentAuthorizationDecisionV1 {
    if let Err(reason) = request.validate() {
        return AssessmentAuthorizationDecisionV1::Denied(reason);
    }

    let mut seen_ids = HashSet::new();
    for authority in authorities {
        if !seen_ids.insert(authority.authority_id.as_str()) {
            return AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::DuplicateAuthorityId,
            );
        }
    }

    let mut conflict_found = false;
    let mut inactive_found = false;
    let mut outside_validity_found = false;
    let mut authorized_by: Option<&ProfessionalAssessmentAuthorityV1> = None;
    let involved: HashSet<&str> = request
        .involved_party_ids
        .iter()
        .map(String::as_str)
        .collect();

    for authority in authorities {
        if authority.assessor_id != request.assessor_id || authority.scope != request.scope {
            continue;
        }
        if authority.validate().is_err() {
            return AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::InvalidAuthority,
            );
        }
        if authority.status != AuthorityStatusV1::Active {
            inactive_found = true;
            continue;
        }
        if request.at < authority.effective_at || request.at >= authority.expires_at {
            outside_validity_found = true;
            continue;
        }
        if authority
            .revoked_at
            .map(|revoked_at| request.at >= revoked_at)
            .unwrap_or(false)
        {
            outside_validity_found = true;
            continue;
        }

        if authority
            .conflict_party_ids
            .iter()
            .any(|party_id| involved.contains(party_id.as_str()))
        {
            conflict_found = true;
        } else if authorized_by.is_none() {
            authorized_by = Some(authority);
        }
    }

    if conflict_found {
        return AssessmentAuthorizationDecisionV1::Denied(
            DenialReasonV1::ConflictOfInterest,
        );
    }

    match authorized_by {
        Some(authority) => AssessmentAuthorizationDecisionV1::Authorized {
            authority_id: authority.authority_id.clone(),
            authority_revision: authority.revision,
        },
        None if inactive_found => AssessmentAuthorizationDecisionV1::Denied(
            DenialReasonV1::AuthorityNotActive,
        ),
        None if outside_validity_found => AssessmentAuthorizationDecisionV1::Denied(
            DenialReasonV1::AuthorityOutsideValidity,
        ),
        None => AssessmentAuthorizationDecisionV1::Denied(
            DenialReasonV1::NoMatchingAuthority,
        ),
    }
}

fn is_non_empty_unpadded(value: &str) -> bool {
    !value.is_empty() && value.trim() == value
}

fn require_non_empty(value: &str) -> Result<(), AuthorityErrorV1> {
    if !is_non_empty_unpadded(value) {
        Err(AuthorityErrorV1::EmptyField)
    } else {
        Ok(())
    }
}

fn require_digest(value: &[u8; 32]) -> Result<(), AuthorityErrorV1> {
    if value.iter().all(|byte| *byte == 0) {
        Err(AuthorityErrorV1::MissingDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scope() -> AssessmentScopeV1 {
        AssessmentScopeV1 {
            capability_scope_id: "capability:electrical-inspection".to_owned(),
            method_id: "method:inspection-v1".to_owned(),
            jurisdiction_id: "ZA-GP".to_owned(),
            standards_profile_id: "standard:installation-safety".to_owned(),
            standards_revision: "2026.1".to_owned(),
            standards_digest: [3; 32],
        }
    }

    fn authority() -> ProfessionalAssessmentAuthorityV1 {
        ProfessionalAssessmentAuthorityV1 {
            authority_id: "authority-1".to_owned(),
            revision: 1,
            assessor_id: "external-assessor".to_owned(),
            issuer_id: "professional-body".to_owned(),
            grantor_id: "authorized-issuer".to_owned(),
            scope: scope(),
            status: AuthorityStatusV1::Active,
            effective_at: 10,
            expires_at: 30,
            revoked_at: None,
            conflict_party_ids: Vec::new(),
            source_document_digest: [9; 32],
        }
    }

    fn request() -> AssessmentAuthorizationRequestV1 {
        AssessmentAuthorizationRequestV1 {
            assessor_id: "external-assessor".to_owned(),
            subject_id: "worker".to_owned(),
            involved_party_ids: vec!["worker".to_owned(), "guild".to_owned()],
            scope: scope(),
            at: 20,
        }
    }

    #[test]
    fn external_assessor_is_authorized_without_guild_membership_input() {
        assert_eq!(
            authorize_assessment(&request(), &[authority()]),
            AssessmentAuthorizationDecisionV1::Authorized {
                authority_id: "authority-1".to_owned(),
                authority_revision: 1,
            }
        );
    }

    #[test]
    fn assessor_without_independent_authority_is_not_authorized() {
        assert_eq!(
            authorize_assessment(&request(), &[]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );
    }

    #[test]
    fn wrong_capability_scope_does_not_authorize() {
        let mut a = authority();
        a.scope.capability_scope_id = "capability:machinery-repair".to_owned();
        assert_eq!(
            authorize_assessment(&request(), &[a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );
    }

    #[test]
    fn method_jurisdiction_and_standard_revision_are_exact() {
        let mut a = authority();
        a.scope.method_id = "method:visual-check".to_owned();
        assert_eq!(
            authorize_assessment(&request(), &[a.clone()]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );

        a.scope = scope();
        a.scope.jurisdiction_id = "ZA-WC".to_owned();
        assert_eq!(
            authorize_assessment(&request(), &[a.clone()]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );

        a.scope = scope();
        a.scope.standards_revision = "2025.4".to_owned();
        assert_eq!(
            authorize_assessment(&request(), &[a.clone()]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );

        a.scope = scope();
        a.scope.standards_digest = [4; 32];
        assert_eq!(
            authorize_assessment(&request(), &[a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::NoMatchingAuthority
            )
        );
    }

    #[test]
    fn suspended_authority_cannot_authorize() {
        let mut a = authority();
        a.status = AuthorityStatusV1::Suspended;
        assert_eq!(
            authorize_assessment(&request(), &[a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::AuthorityNotActive
            )
        );
    }

    #[test]
    fn expiry_is_exclusive() {
        let mut a = authority();
        a.expires_at = 20;
        assert_eq!(
            authorize_assessment(&request(), &[a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::AuthorityOutsideValidity
            )
        );
    }

    #[test]
    fn revocation_takes_effect_at_its_timestamp() {
        let mut a = authority();
        a.revoked_at = Some(20);
        assert_eq!(
            authorize_assessment(&request(), &[a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::AuthorityOutsideValidity
            )
        );
    }

    #[test]
    fn conflict_of_interest_denies_even_when_another_authority_matches() {
        let clean = authority();
        let mut conflicted = authority();
        conflicted.authority_id = "authority-2".to_owned();
        conflicted.conflict_party_ids = vec!["guild".to_owned()];
        assert_eq!(
            authorize_assessment(&request(), &[clean, conflicted]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::ConflictOfInterest
            )
        );
    }

    #[test]
    fn assessor_cannot_grant_authority_to_themself() {
        let mut a = authority();
        a.grantor_id = a.assessor_id.clone();
        assert_eq!(a.validate(), Err(AuthorityErrorV1::SelfGrant));
    }

    #[test]
    fn duplicate_authority_ids_fail_closed() {
        let a = authority();
        assert_eq!(
            authorize_assessment(&request(), &[a.clone(), a]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::DuplicateAuthorityId
            )
        );
    }

    #[test]
    fn self_assessment_is_denied() {
        let mut r = request();
        r.assessor_id = "worker".to_owned();
        assert_eq!(
            authorize_assessment(&r, &[authority()]),
            AssessmentAuthorizationDecisionV1::Denied(
                DenialReasonV1::SelfAssessment
            )
        );
    }
}
