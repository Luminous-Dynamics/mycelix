// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! ADMIN-003: immutable administrative review, stay, finality, and remedy semantics.
//!
//! ADMIN-003 is additive above ADMIN-002. It never rewrites the original
//! administrative decision and never turns a review/finality record into an
//! external-effect capability.

use super::{AdministrativeCaseState, IssuedAdministrativeDecision, ProcedureProfileId};
use mycelix_institutional_core::{
    Appeal, AppealId, AuthorityDecision, AuthorityGrant, AuthorityGrantId, AuthorityRequirement,
    CapabilityId, Challenge, DecisionId, Digest32, EvidenceRef, InstitutionId, JurisdictionId,
    PROTOCOL_VERSION as INSTITUTIONAL_PROTOCOL_VERSION, PrincipalId, Remedy, RoleId, RulebookRef,
    evaluate_authority,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const ADMIN_003_PROTOCOL_VERSION: &str = "mycelix-administrative-review-v0.1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_TEXT_BYTES: usize = 8192;
const MAX_REASONS: usize = 64;
const MAX_EVIDENCE: usize = 256;
const MAX_REMEDY_TYPES: usize = 32;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeReviewPolicy {
    pub protocol_version: String,
    pub procedure_profile: ProcedureProfileId,
    pub policy_ref: String,
    pub policy_digest: Digest32,
    pub policy_digest_profile: String,
    /// Exact source-decision institutional context to which this review profile applies.
    pub source_institution: InstitutionId,
    pub source_jurisdiction: Option<JurisdictionId>,
    pub source_rulebook: RulebookRef,
    /// Competent administrative review forum and its own authority context.
    pub review_forum: InstitutionId,
    pub review_jurisdiction: Option<JurisdictionId>,
    pub review_rulebook: RulebookRef,
    pub review_capability: CapabilityId,
    pub stay_capability: CapabilityId,
    pub remedy_capability: CapabilityId,
    pub accepted_review_roles: Vec<RoleId>,
    pub challenge_window_ms: u64,
    pub appeal_window_ms: u64,
    /// Minimum delay after a review disposition before administrative finality may close.
    pub finality_delay_ms: u64,
    pub require_independent_reviewer: bool,
    pub allowed_remedy_types: Vec<String>,
}

impl AdministrativeReviewPolicy {
    pub fn validate(&self) -> Result<(), AdministrativeReviewError> {
        require_admin3_protocol(&self.protocol_version)?;
        validate_ref(&self.policy_ref)?;
        validate_profile(&self.policy_digest_profile)?;
        if self.policy_digest.is_zero() {
            return Err(AdministrativeReviewError::ZeroPolicyDigest);
        }
        validate_id(self.procedure_profile.as_str())?;
        validate_id(self.source_institution.as_str())?;
        if let Some(jurisdiction) = &self.source_jurisdiction {
            validate_id(jurisdiction.as_str())?;
        }
        self.source_rulebook
            .validate()
            .map_err(|_| AdministrativeReviewError::InvalidRulebook)?;
        validate_id(self.review_forum.as_str())?;
        if let Some(jurisdiction) = &self.review_jurisdiction {
            validate_id(jurisdiction.as_str())?;
        }
        self.review_rulebook
            .validate()
            .map_err(|_| AdministrativeReviewError::InvalidRulebook)?;
        validate_id(self.review_capability.as_str())?;
        validate_id(self.stay_capability.as_str())?;
        validate_id(self.remedy_capability.as_str())?;
        if self.challenge_window_ms == 0 || self.appeal_window_ms == 0 {
            return Err(AdministrativeReviewError::InvalidReviewWindow);
        }
        let mut roles = BTreeSet::new();
        for role in &self.accepted_review_roles {
            validate_id(role.as_str())?;
            if !roles.insert(role.as_str()) {
                return Err(AdministrativeReviewError::DuplicateReviewRole);
            }
        }
        if self.allowed_remedy_types.len() > MAX_REMEDY_TYPES {
            return Err(AdministrativeReviewError::TooManyRemedyTypes);
        }
        let mut remedies = BTreeSet::new();
        for remedy in &self.allowed_remedy_types {
            validate_code(remedy)?;
            if !remedies.insert(remedy.as_str()) {
                return Err(AdministrativeReviewError::DuplicateRemedyType);
            }
        }
        Ok(())
    }
}

/// Admissibility metadata that institutional-core `Challenge` deliberately does
/// not try to encode. Proof authenticity/standing interpretation belongs to a
/// runtime/provider; this pure layer binds those references to the exact filing.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChallengeSubmission {
    pub protocol_version: String,
    pub challenge: Challenge,
    pub standing_proof_ref: String,
    pub service_proof_ref: String,
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedReviewableDecision {
    issued: IssuedAdministrativeDecision,
    policy: AdministrativeReviewPolicy,
}

impl QualifiedReviewableDecision {
    pub fn issued(&self) -> &IssuedAdministrativeDecision {
        &self.issued
    }

    pub fn policy(&self) -> &AdministrativeReviewPolicy {
        &self.policy
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedChallenge {
    reviewable: QualifiedReviewableDecision,
    submission: ChallengeSubmission,
}

impl QualifiedChallenge {
    pub fn challenge(&self) -> &Challenge {
        &self.submission.challenge
    }

    pub fn original_decision_id(&self) -> &DecisionId {
        &self.reviewable.issued.decision().id
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StayDirectiveKind {
    Impose,
    Lift,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StayDirective {
    pub protocol_version: String,
    pub appeal_id: AppealId,
    pub decision_id: DecisionId,
    pub kind: StayDirectiveKind,
    pub authorized_by: AuthorityGrantId,
    pub issued_at_ms: u64,
    /// Required for `Impose`; forbidden for `Lift`.
    pub expires_at_ms: Option<u64>,
    pub proof_ref: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StayState {
    NotStayed,
    StayedUntil(u64),
    LiftedAt(u64),
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAppealReview {
    challenge: QualifiedChallenge,
    appeal: Appeal,
    stay_state: StayState,
}

impl QualifiedAppealReview {
    pub fn appeal(&self) -> &Appeal {
        &self.appeal
    }

    pub fn stay_state(&self) -> StayState {
        self.stay_state
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReviewDispositionOutcome {
    Affirm,
    Reverse,
    Vacate,
    Remand,
    Modify,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeReviewDisposition {
    pub protocol_version: String,
    pub appeal_id: AppealId,
    pub decision_id: DecisionId,
    pub reviewer: PrincipalId,
    pub authority_grant_id: AuthorityGrantId,
    pub outcome: ReviewDispositionOutcome,
    pub reasons: Vec<String>,
    pub evidence: Vec<EvidenceRef>,
    pub decided_at_ms: u64,
    pub disposition_ref: String,
    pub proof_ref: String,
    /// Exact remedy types this disposition authorizes, if any.
    pub authorized_remedy_types: Vec<String>,
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedReviewDisposition {
    review: QualifiedAppealReview,
    disposition: AdministrativeReviewDisposition,
}

impl QualifiedReviewDisposition {
    pub fn disposition(&self) -> &AdministrativeReviewDisposition {
        &self.disposition
    }

    pub fn stay_state(&self) -> StayState {
        self.review.stay_state
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdministrativeFinalityReceipt {
    pub protocol_version: String,
    pub decision_id: DecisionId,
    pub appeal_id: AppealId,
    pub disposition_ref: String,
    pub finalized_at_ms: u64,
    pub proof_ref: String,
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedAdministrativeFinality {
    disposition: QualifiedReviewDisposition,
    receipt: AdministrativeFinalityReceipt,
}

impl QualifiedAdministrativeFinality {
    pub fn decision_id(&self) -> &DecisionId {
        &self.receipt.decision_id
    }

    pub fn finalized_at_ms(&self) -> u64 {
        self.receipt.finalized_at_ms
    }

    pub fn disposition(&self) -> &AdministrativeReviewDisposition {
        &self.disposition.disposition
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Court/judicial finality is intentionally *recorded*, not created, by this
/// administrative kernel. The provider must independently establish court
/// competence and proof authenticity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalJudicialFinalityReference {
    pub protocol_version: String,
    pub decision_id: DecisionId,
    pub administrative_disposition_ref: String,
    pub court_forum_ref: String,
    pub judgment_ref: String,
    pub judgment_digest: Digest32,
    pub judgment_digest_profile: String,
    pub finalized_at_ms: u64,
    pub proof_ref: String,
}

#[derive(Debug, PartialEq, Eq)]
pub struct RecordedExternalJudicialFinality {
    reference: ExternalJudicialFinalityReference,
}

impl RecordedExternalJudicialFinality {
    pub fn reference(&self) -> &ExternalJudicialFinalityReference {
        &self.reference
    }

    pub fn grants_authority(&self) -> bool {
        false
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedRemedy {
    remedy: Remedy,
    authorizer: PrincipalId,
}

impl QualifiedRemedy {
    pub fn remedy(&self) -> &Remedy {
        &self.remedy
    }

    pub fn authorizer(&self) -> &PrincipalId {
        &self.authorizer
    }

    pub fn grants_external_effect_authority(&self) -> bool {
        false
    }
}

/// Capture one issued ADMIN-002 decision into an immutable review domain.
pub fn qualify_reviewable_decision(
    issued: IssuedAdministrativeDecision,
    policy: AdministrativeReviewPolicy,
) -> Result<QualifiedReviewableDecision, AdministrativeReviewError> {
    policy.validate()?;
    let case = issued.successor_case();
    match &case.state {
        AdministrativeCaseState::DecisionIssued {
            decision_id,
            issued_at_ms,
        } => {
            if decision_id != &issued.decision().id
                || *issued_at_ms != issued.decision().decided_at_ms
            {
                return Err(AdministrativeReviewError::IssuedDecisionMismatch);
            }
        }
        _ => return Err(AdministrativeReviewError::DecisionNotIssued),
    }
    if policy.procedure_profile != case.procedure_profile
        || policy.source_institution != case.institution
        || policy.source_jurisdiction != case.jurisdiction
        || policy.source_rulebook != case.rulebook
    {
        return Err(AdministrativeReviewError::ReviewPolicyContextMismatch);
    }
    Ok(QualifiedReviewableDecision { issued, policy })
}

pub fn qualify_challenge(
    reviewable: QualifiedReviewableDecision,
    submission: ChallengeSubmission,
) -> Result<QualifiedChallenge, AdministrativeReviewError> {
    require_admin3_protocol(&submission.protocol_version)?;
    validate_ref(&submission.standing_proof_ref)?;
    validate_ref(&submission.service_proof_ref)?;
    validate_challenge(&submission.challenge)?;

    let challenge = &submission.challenge;
    let decision = reviewable.issued.decision();
    if challenge.decision_id != decision.id {
        return Err(AdministrativeReviewError::DecisionMismatch);
    }
    if challenge.filed_at_ms < decision.decided_at_ms {
        return Err(AdministrativeReviewError::ChallengeBeforeDecision);
    }
    let deadline = checked_deadline(
        decision.decided_at_ms,
        reviewable.policy.challenge_window_ms,
    )?;
    if challenge.filed_at_ms > deadline {
        return Err(AdministrativeReviewError::ChallengeUntimely);
    }
    validate_evidence_cut(&challenge.evidence, challenge.filed_at_ms)?;

    Ok(QualifiedChallenge {
        reviewable,
        submission,
    })
}

pub fn qualify_appeal(
    challenge: QualifiedChallenge,
    appeal: Appeal,
) -> Result<QualifiedAppealReview, AdministrativeReviewError> {
    validate_appeal(&appeal)?;
    let filed_challenge = &challenge.submission.challenge;
    let policy = &challenge.reviewable.policy;
    if appeal.challenge_id != filed_challenge.id {
        return Err(AdministrativeReviewError::ChallengeMismatch);
    }
    // v0.1 deliberately requires the same principal. Representation/guardian
    // authority belongs to a later profile/provider layer rather than an
    // untyped alternate-appellant shortcut.
    if appeal.appellant != filed_challenge.challenger {
        return Err(AdministrativeReviewError::AppellantMismatch);
    }
    if appeal.forum != policy.review_forum || appeal.rulebook != policy.review_rulebook {
        return Err(AdministrativeReviewError::ReviewForumMismatch);
    }
    if appeal.filed_at_ms < filed_challenge.filed_at_ms {
        return Err(AdministrativeReviewError::AppealBeforeChallenge);
    }
    let deadline = checked_deadline(filed_challenge.filed_at_ms, policy.appeal_window_ms)?;
    if appeal.filed_at_ms > deadline {
        return Err(AdministrativeReviewError::AppealUntimely);
    }

    Ok(QualifiedAppealReview {
        challenge,
        appeal,
        stay_state: StayState::NotStayed,
    })
}

pub fn apply_stay_directive(
    mut review: QualifiedAppealReview,
    directive: StayDirective,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedAppealReview, AdministrativeReviewError> {
    validate_stay_directive(&directive)?;
    let decision = review.challenge.reviewable.issued.decision();
    if directive.appeal_id != review.appeal.id || directive.decision_id != decision.id {
        return Err(AdministrativeReviewError::StayScopeMismatch);
    }
    if directive.authorized_by != grant.id {
        return Err(AdministrativeReviewError::AuthorityGrantMismatch);
    }
    if directive.issued_at_ms < review.appeal.filed_at_ms {
        return Err(AdministrativeReviewError::StayBeforeAppeal);
    }
    qualify_authority(
        grant,
        &review.challenge.reviewable.policy,
        &review.challenge.reviewable.policy.stay_capability,
        authority_evidence,
        directive.issued_at_ms,
    )?;

    review.stay_state = match directive.kind {
        StayDirectiveKind::Impose => {
            let expires = directive
                .expires_at_ms
                .ok_or(AdministrativeReviewError::InvalidStayLifetime)?;
            match review.stay_state {
                StayState::NotStayed => StayState::StayedUntil(expires),
                StayState::LiftedAt(lifted_at) if directive.issued_at_ms >= lifted_at => {
                    StayState::StayedUntil(expires)
                }
                StayState::StayedUntil(previous_until)
                    if directive.issued_at_ms >= previous_until =>
                {
                    StayState::StayedUntil(expires)
                }
                StayState::StayedUntil(_) | StayState::LiftedAt(_) => {
                    return Err(AdministrativeReviewError::StayAlreadyActive);
                }
            }
        }
        StayDirectiveKind::Lift => match review.stay_state {
            StayState::StayedUntil(until) if directive.issued_at_ms <= until => {
                StayState::LiftedAt(directive.issued_at_ms)
            }
            _ => return Err(AdministrativeReviewError::StayNotActive),
        },
    };

    Ok(review)
}

pub fn qualify_review_disposition(
    review: QualifiedAppealReview,
    disposition: AdministrativeReviewDisposition,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedReviewDisposition, AdministrativeReviewError> {
    validate_disposition(&disposition)?;
    let policy = &review.challenge.reviewable.policy;
    let original = review.challenge.reviewable.issued.decision();
    if disposition.appeal_id != review.appeal.id || disposition.decision_id != original.id {
        return Err(AdministrativeReviewError::DispositionScopeMismatch);
    }
    if disposition.authority_grant_id != grant.id || disposition.reviewer != grant.holder {
        return Err(AdministrativeReviewError::AuthorityGrantMismatch);
    }
    if disposition.decided_at_ms < review.appeal.filed_at_ms {
        return Err(AdministrativeReviewError::DispositionBeforeAppeal);
    }
    if policy.require_independent_reviewer
        && (disposition.reviewer == *review.challenge.reviewable.issued.decider()
            || disposition.reviewer == review.appeal.appellant)
    {
        return Err(AdministrativeReviewError::ReviewerNotIndependent);
    }
    validate_evidence_cut(&disposition.evidence, disposition.decided_at_ms)?;
    qualify_authority(
        grant,
        policy,
        &policy.review_capability,
        authority_evidence,
        disposition.decided_at_ms,
    )?;

    let allowed = exact_code_set(&policy.allowed_remedy_types)?;
    let authorized = exact_code_set(&disposition.authorized_remedy_types)?;
    if !authorized.is_subset(&allowed) {
        return Err(AdministrativeReviewError::UnauthorizedRemedyType);
    }
    if disposition.outcome == ReviewDispositionOutcome::Affirm && !authorized.is_empty() {
        return Err(AdministrativeReviewError::AffirmCannotAuthorizeRemedy);
    }

    Ok(QualifiedReviewDisposition {
        review,
        disposition,
    })
}

pub fn qualify_administrative_finality(
    disposition: QualifiedReviewDisposition,
    receipt: AdministrativeFinalityReceipt,
) -> Result<QualifiedAdministrativeFinality, AdministrativeReviewError> {
    require_admin3_protocol(&receipt.protocol_version)?;
    validate_ref(&receipt.disposition_ref)?;
    validate_ref(&receipt.proof_ref)?;
    let reviewed = &disposition.disposition;
    if receipt.decision_id != reviewed.decision_id
        || receipt.appeal_id != reviewed.appeal_id
        || receipt.disposition_ref != reviewed.disposition_ref
    {
        return Err(AdministrativeReviewError::FinalityScopeMismatch);
    }
    let earliest = checked_deadline(
        reviewed.decided_at_ms,
        disposition
            .review
            .challenge
            .reviewable
            .policy
            .finality_delay_ms,
    )?;
    if receipt.finalized_at_ms < earliest {
        return Err(AdministrativeReviewError::FinalityTooEarly);
    }

    Ok(QualifiedAdministrativeFinality {
        disposition,
        receipt,
    })
}

pub fn record_external_judicial_finality(
    administrative: &QualifiedAdministrativeFinality,
    reference: ExternalJudicialFinalityReference,
) -> Result<RecordedExternalJudicialFinality, AdministrativeReviewError> {
    require_admin3_protocol(&reference.protocol_version)?;
    validate_ref(&reference.administrative_disposition_ref)?;
    validate_ref(&reference.court_forum_ref)?;
    validate_ref(&reference.judgment_ref)?;
    validate_ref(&reference.proof_ref)?;
    validate_profile(&reference.judgment_digest_profile)?;
    if reference.judgment_digest.is_zero() {
        return Err(AdministrativeReviewError::ZeroJudgmentDigest);
    }
    if reference.decision_id != *administrative.decision_id()
        || reference.administrative_disposition_ref != administrative.disposition().disposition_ref
    {
        return Err(AdministrativeReviewError::JudicialFinalityScopeMismatch);
    }
    if reference.finalized_at_ms < administrative.finalized_at_ms() {
        return Err(AdministrativeReviewError::JudicialFinalityBeforeAdministrativeFinality);
    }
    Ok(RecordedExternalJudicialFinality { reference })
}

pub fn qualify_remedy(
    administrative: &QualifiedAdministrativeFinality,
    remedy: Remedy,
    grant: &AuthorityGrant,
    authority_evidence: &[EvidenceRef],
) -> Result<QualifiedRemedy, AdministrativeReviewError> {
    validate_remedy(&remedy)?;
    let policy = &administrative
        .disposition
        .review
        .challenge
        .reviewable
        .policy;
    if remedy.decision_id != *administrative.decision_id() {
        return Err(AdministrativeReviewError::DecisionMismatch);
    }
    if remedy.authorized_by != grant.id {
        return Err(AdministrativeReviewError::AuthorityGrantMismatch);
    }
    if remedy.issued_at_ms < administrative.finalized_at_ms() {
        return Err(AdministrativeReviewError::RemedyBeforeFinality);
    }
    if !administrative
        .disposition()
        .authorized_remedy_types
        .iter()
        .any(|kind| kind == &remedy.remedy_type)
    {
        return Err(AdministrativeReviewError::UnauthorizedRemedyType);
    }
    qualify_authority(
        grant,
        policy,
        &policy.remedy_capability,
        authority_evidence,
        remedy.issued_at_ms,
    )?;
    Ok(QualifiedRemedy {
        remedy,
        authorizer: grant.holder.clone(),
    })
}

fn validate_challenge(challenge: &Challenge) -> Result<(), AdministrativeReviewError> {
    require_institutional_protocol(&challenge.protocol_version)?;
    validate_id(challenge.id.as_str())?;
    validate_id(challenge.decision_id.as_str())?;
    validate_id(challenge.challenger.as_str())?;
    validate_code(&challenge.grounds_code)?;
    if challenge.filed_at_ms == 0 {
        return Err(AdministrativeReviewError::InvalidChallengeTime);
    }
    Ok(())
}

fn validate_appeal(appeal: &Appeal) -> Result<(), AdministrativeReviewError> {
    require_institutional_protocol(&appeal.protocol_version)?;
    validate_id(appeal.id.as_str())?;
    validate_id(appeal.challenge_id.as_str())?;
    validate_id(appeal.appellant.as_str())?;
    validate_id(appeal.forum.as_str())?;
    appeal
        .rulebook
        .validate()
        .map_err(|_| AdministrativeReviewError::InvalidRulebook)?;
    if appeal.filed_at_ms == 0 {
        return Err(AdministrativeReviewError::InvalidAppealTime);
    }
    Ok(())
}

fn validate_stay_directive(directive: &StayDirective) -> Result<(), AdministrativeReviewError> {
    require_admin3_protocol(&directive.protocol_version)?;
    validate_id(directive.appeal_id.as_str())?;
    validate_id(directive.decision_id.as_str())?;
    validate_id(directive.authorized_by.as_str())?;
    validate_ref(&directive.proof_ref)?;
    if directive.issued_at_ms == 0 {
        return Err(AdministrativeReviewError::InvalidStayTime);
    }
    match directive.kind {
        StayDirectiveKind::Impose => match directive.expires_at_ms {
            Some(expires) if expires > directive.issued_at_ms => Ok(()),
            _ => Err(AdministrativeReviewError::InvalidStayLifetime),
        },
        StayDirectiveKind::Lift => {
            if directive.expires_at_ms.is_some() {
                Err(AdministrativeReviewError::InvalidStayLifetime)
            } else {
                Ok(())
            }
        }
    }
}

fn validate_disposition(
    disposition: &AdministrativeReviewDisposition,
) -> Result<(), AdministrativeReviewError> {
    require_admin3_protocol(&disposition.protocol_version)?;
    validate_id(disposition.appeal_id.as_str())?;
    validate_id(disposition.decision_id.as_str())?;
    validate_id(disposition.reviewer.as_str())?;
    validate_id(disposition.authority_grant_id.as_str())?;
    if disposition.reasons.is_empty() || disposition.reasons.len() > MAX_REASONS {
        return Err(AdministrativeReviewError::InvalidReasons);
    }
    for reason in &disposition.reasons {
        validate_text(reason)?;
    }
    if disposition.evidence.len() > MAX_EVIDENCE {
        return Err(AdministrativeReviewError::TooMuchEvidence);
    }
    if disposition.decided_at_ms == 0 {
        return Err(AdministrativeReviewError::InvalidDispositionTime);
    }
    validate_ref(&disposition.disposition_ref)?;
    validate_ref(&disposition.proof_ref)?;
    if disposition.authorized_remedy_types.len() > MAX_REMEDY_TYPES {
        return Err(AdministrativeReviewError::TooManyRemedyTypes);
    }
    exact_code_set(&disposition.authorized_remedy_types)?;
    Ok(())
}

fn validate_remedy(remedy: &Remedy) -> Result<(), AdministrativeReviewError> {
    require_institutional_protocol(&remedy.protocol_version)?;
    validate_id(remedy.id.as_str())?;
    validate_id(remedy.decision_id.as_str())?;
    validate_code(&remedy.remedy_type)?;
    validate_text(&remedy.description)?;
    validate_id(remedy.authorized_by.as_str())?;
    if remedy.issued_at_ms == 0 {
        return Err(AdministrativeReviewError::InvalidRemedyTime);
    }
    if let Some(expires) = remedy.expires_at_ms {
        if expires <= remedy.issued_at_ms {
            return Err(AdministrativeReviewError::InvalidRemedyLifetime);
        }
    }
    Ok(())
}

fn qualify_authority(
    grant: &AuthorityGrant,
    policy: &AdministrativeReviewPolicy,
    capability: &CapabilityId,
    evidence: &[EvidenceRef],
    at_ms: u64,
) -> Result<(), AdministrativeReviewError> {
    grant
        .validate()
        .map_err(|_| AdministrativeReviewError::InvalidAuthorityGrant)?;
    let requirement = AuthorityRequirement {
        institution: policy.review_forum.clone(),
        jurisdiction: policy.review_jurisdiction.clone(),
        required_capabilities: vec![capability.clone()],
        accepted_roles: policy.accepted_review_roles.clone(),
        evidence: vec![],
        rulebook: policy.review_rulebook.clone(),
    };
    match evaluate_authority(grant, &requirement, evidence, at_ms) {
        AuthorityDecision::Allow(allowed) => {
            if allowed.grant_id != grant.id || allowed.holder != grant.holder {
                Err(AdministrativeReviewError::AuthorityResultMismatch)
            } else {
                Ok(())
            }
        }
        AuthorityDecision::NeedsEvidence(_) => {
            Err(AdministrativeReviewError::MissingAuthorityEvidence)
        }
        AuthorityDecision::Deny(_) => Err(AdministrativeReviewError::ReviewAuthorityDenied),
    }
}

fn validate_evidence_cut(
    evidence: &[EvidenceRef],
    no_later_than_ms: u64,
) -> Result<(), AdministrativeReviewError> {
    if evidence.len() > MAX_EVIDENCE {
        return Err(AdministrativeReviewError::TooMuchEvidence);
    }
    let mut ids = BTreeSet::new();
    for item in evidence {
        item.validate()
            .map_err(|_| AdministrativeReviewError::InvalidEvidence)?;
        if item.observed_at_ms > no_later_than_ms {
            return Err(AdministrativeReviewError::EvidenceFromFuture);
        }
        if !ids.insert(item.id.as_str()) {
            return Err(AdministrativeReviewError::DuplicateEvidenceIdentity);
        }
    }
    Ok(())
}

fn exact_code_set(values: &[String]) -> Result<BTreeSet<&str>, AdministrativeReviewError> {
    let mut set = BTreeSet::new();
    for value in values {
        validate_code(value)?;
        if !set.insert(value.as_str()) {
            return Err(AdministrativeReviewError::DuplicateRemedyType);
        }
    }
    Ok(set)
}

fn checked_deadline(start_ms: u64, window_ms: u64) -> Result<u64, AdministrativeReviewError> {
    start_ms
        .checked_add(window_ms)
        .ok_or(AdministrativeReviewError::TimeOverflow)
}

fn require_admin3_protocol(value: &str) -> Result<(), AdministrativeReviewError> {
    if value == ADMIN_003_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(AdministrativeReviewError::WrongProtocolVersion)
    }
}

fn require_institutional_protocol(value: &str) -> Result<(), AdministrativeReviewError> {
    if value == INSTITUTIONAL_PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(AdministrativeReviewError::WrongInstitutionalProtocolVersion)
    }
}

fn validate_id(value: &str) -> Result<(), AdministrativeReviewError> {
    validate_bounded(
        value,
        MAX_REF_BYTES,
        AdministrativeReviewError::InvalidIdentifier,
    )
}

fn validate_ref(value: &str) -> Result<(), AdministrativeReviewError> {
    validate_bounded(
        value,
        MAX_REF_BYTES,
        AdministrativeReviewError::InvalidReference,
    )
}

fn validate_code(value: &str) -> Result<(), AdministrativeReviewError> {
    validate_bounded(value, 256, AdministrativeReviewError::InvalidCode)
}

fn validate_text(value: &str) -> Result<(), AdministrativeReviewError> {
    validate_bounded(
        value,
        MAX_TEXT_BYTES,
        AdministrativeReviewError::InvalidText,
    )
}

fn validate_bounded(
    value: &str,
    max: usize,
    error: AdministrativeReviewError,
) -> Result<(), AdministrativeReviewError> {
    if value.trim().is_empty() || value.len() > max {
        Err(error)
    } else {
        Ok(())
    }
}

fn validate_profile(value: &str) -> Result<(), AdministrativeReviewError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(AdministrativeReviewError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AdministrativeReviewError {
    WrongProtocolVersion,
    WrongInstitutionalProtocolVersion,
    InvalidIdentifier,
    InvalidReference,
    InvalidCode,
    InvalidText,
    InvalidProfile,
    ZeroPolicyDigest,
    ZeroJudgmentDigest,
    InvalidRulebook,
    InvalidReviewWindow,
    DuplicateReviewRole,
    TooManyRemedyTypes,
    DuplicateRemedyType,
    DecisionNotIssued,
    IssuedDecisionMismatch,
    ReviewPolicyContextMismatch,
    DecisionMismatch,
    InvalidChallengeTime,
    ChallengeBeforeDecision,
    ChallengeUntimely,
    InvalidAppealTime,
    ChallengeMismatch,
    AppellantMismatch,
    ReviewForumMismatch,
    AppealBeforeChallenge,
    AppealUntimely,
    InvalidStayTime,
    InvalidStayLifetime,
    StayScopeMismatch,
    StayBeforeAppeal,
    StayAlreadyActive,
    StayNotActive,
    InvalidDispositionTime,
    DispositionScopeMismatch,
    DispositionBeforeAppeal,
    ReviewerNotIndependent,
    InvalidReasons,
    TooMuchEvidence,
    InvalidEvidence,
    DuplicateEvidenceIdentity,
    EvidenceFromFuture,
    UnauthorizedRemedyType,
    AffirmCannotAuthorizeRemedy,
    FinalityScopeMismatch,
    FinalityTooEarly,
    JudicialFinalityScopeMismatch,
    JudicialFinalityBeforeAdministrativeFinality,
    InvalidRemedyTime,
    InvalidRemedyLifetime,
    RemedyBeforeFinality,
    InvalidAuthorityGrant,
    AuthorityGrantMismatch,
    MissingAuthorityEvidence,
    ReviewAuthorityDenied,
    AuthorityResultMismatch,
    TimeOverflow,
}

impl fmt::Display for AdministrativeReviewError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong ADMIN-003 protocol version",
            Self::WrongInstitutionalProtocolVersion => "wrong institutional-core protocol version",
            Self::InvalidIdentifier => "invalid administrative-review identifier",
            Self::InvalidReference => "invalid administrative-review reference",
            Self::InvalidCode => "invalid administrative-review code",
            Self::InvalidText => "invalid administrative-review text",
            Self::InvalidProfile => "invalid administrative-review digest profile",
            Self::ZeroPolicyDigest => "review policy digest must not be zero",
            Self::ZeroJudgmentDigest => "judicial judgment digest must not be zero",
            Self::InvalidRulebook => "invalid administrative-review rulebook",
            Self::InvalidReviewWindow => "challenge/appeal review window must be non-zero",
            Self::DuplicateReviewRole => "duplicate accepted review role",
            Self::TooManyRemedyTypes => "too many remedy types",
            Self::DuplicateRemedyType => "duplicate remedy type",
            Self::DecisionNotIssued => "review requires an issued administrative decision",
            Self::IssuedDecisionMismatch => "issued case and decision identity disagree",
            Self::ReviewPolicyContextMismatch => "review policy does not match source case context",
            Self::DecisionMismatch => "review object targets another decision",
            Self::InvalidChallengeTime => "invalid challenge filing time",
            Self::ChallengeBeforeDecision => "challenge predates the challenged decision",
            Self::ChallengeUntimely => "challenge was filed after its exact window",
            Self::InvalidAppealTime => "invalid appeal filing time",
            Self::ChallengeMismatch => "appeal targets another challenge",
            Self::AppellantMismatch => "v0.1 appeal appellant must equal the challenger",
            Self::ReviewForumMismatch => "appeal forum/rulebook does not match review policy",
            Self::AppealBeforeChallenge => "appeal predates its challenge",
            Self::AppealUntimely => "appeal was filed after its exact window",
            Self::InvalidStayTime => "invalid stay directive time",
            Self::InvalidStayLifetime => "invalid stay directive lifetime",
            Self::StayScopeMismatch => "stay directive targets another appeal or decision",
            Self::StayBeforeAppeal => "stay directive predates the appeal",
            Self::StayAlreadyActive => "a non-expired stay is already active",
            Self::StayNotActive => "no active stay exists to lift",
            Self::InvalidDispositionTime => "invalid review disposition time",
            Self::DispositionScopeMismatch => "review disposition targets another appeal/decision",
            Self::DispositionBeforeAppeal => "review disposition predates the appeal",
            Self::ReviewerNotIndependent => "reviewer is not independent under the policy",
            Self::InvalidReasons => "review disposition requires bounded non-empty reasons",
            Self::TooMuchEvidence => "administrative-review evidence exceeds the v0.1 bound",
            Self::InvalidEvidence => "invalid administrative-review evidence",
            Self::DuplicateEvidenceIdentity => "duplicate administrative-review evidence identity",
            Self::EvidenceFromFuture => "review evidence was observed after the event it supports",
            Self::UnauthorizedRemedyType => {
                "review/remedy type is not authorized by policy/disposition"
            }
            Self::AffirmCannotAuthorizeRemedy => {
                "affirmance cannot authorize a corrective remedy in v0.1"
            }
            Self::FinalityScopeMismatch => "administrative-finality receipt targets another review",
            Self::FinalityTooEarly => "administrative finality closed before the configured delay",
            Self::JudicialFinalityScopeMismatch => {
                "judicial-finality reference targets another decision/review"
            }
            Self::JudicialFinalityBeforeAdministrativeFinality => {
                "judicial finality predates administrative finality"
            }
            Self::InvalidRemedyTime => "invalid remedy issuance time",
            Self::InvalidRemedyLifetime => "invalid remedy lifetime",
            Self::RemedyBeforeFinality => "remedy predates administrative finality",
            Self::InvalidAuthorityGrant => "invalid review authority grant",
            Self::AuthorityGrantMismatch => {
                "review object authority grant does not match supplied grant"
            }
            Self::MissingAuthorityEvidence => "review authority requires missing evidence",
            Self::ReviewAuthorityDenied => "institutional authority denied the review action",
            Self::AuthorityResultMismatch => {
                "authority evaluator returned mismatched grant identity"
            }
            Self::TimeOverflow => "administrative-review time arithmetic overflow",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for AdministrativeReviewError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        EvidenceClosureReceipt, NoticeReceipt, ProceduralCompletenessPolicy, ReasonsRequirement,
        ResponseMode, ResponseModeRequirement, ResponseOpportunityReceipt,
        qualify_administrative_decision, qualify_case_lineage, qualify_procedural_completeness,
    };
    use mycelix_institutional_core::{
        AuthoritySourceKind, AuthoritySourceRef, Decision, EvidenceId, RulebookId,
    };

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn p(value: &str) -> PrincipalId {
        PrincipalId::new(value).unwrap()
    }

    fn source_institution() -> InstitutionId {
        InstitutionId::new("institution:city").unwrap()
    }

    fn review_forum() -> InstitutionId {
        InstitutionId::new("institution:admin-appeals-board").unwrap()
    }

    fn jurisdiction() -> JurisdictionId {
        JurisdictionId::new("jurisdiction:city").unwrap()
    }

    fn source_rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administration:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(7),
        }
    }

    fn review_rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:administrative-review:v1").unwrap(),
            version: "1.0.0".into(),
            digest: d(8),
        }
    }

    fn review_policy() -> AdministrativeReviewPolicy {
        AdministrativeReviewPolicy {
            protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
            procedure_profile: ProcedureProfileId::new("procedure:permit:v1").unwrap(),
            policy_ref: "policy:review:permit:v1".into(),
            policy_digest: d(21),
            policy_digest_profile: "admin-review-policy-v1-blake3".into(),
            source_institution: source_institution(),
            source_jurisdiction: Some(jurisdiction()),
            source_rulebook: source_rulebook(),
            review_forum: review_forum(),
            review_jurisdiction: Some(jurisdiction()),
            review_rulebook: review_rulebook(),
            review_capability: CapabilityId::new("administration.review").unwrap(),
            stay_capability: CapabilityId::new("administration.stay").unwrap(),
            remedy_capability: CapabilityId::new("administration.remedy").unwrap(),
            accepted_review_roles: vec![RoleId::new("role:appeals-officer").unwrap()],
            challenge_window_ms: 10_000,
            appeal_window_ms: 10_000,
            finality_delay_ms: 1_000,
            require_independent_reviewer: true,
            allowed_remedy_types: vec!["remand".into(), "vacate".into()],
        }
    }

    fn review_grant(capabilities: Vec<CapabilityId>) -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:reviewer:1").unwrap(),
            holder: p("did:example:reviewer"),
            institution: review_forum(),
            jurisdiction: Some(jurisdiction()),
            roles: vec![RoleId::new("role:appeals-officer").unwrap()],
            capabilities,
            rulebook: review_rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "credential:reviewer:1".into(),
                proof_ref: "proof:reviewer:1".into(),
            }],
            issued_at_ms: 1_000,
            expires_at_ms: 100_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant:reviewer:1".into(),
        }
    }

    fn issued_decision() -> IssuedAdministrativeDecision {
        use crate::{
            AdministrativeCase, AdministrativeCaseId, AdministrativeCaseState,
            AdministrativeDecisionEnvelope, AdministrativeDecisionPolicy, PROTOCOL_VERSION,
            PreDecisionState, PreDecisionTransition,
        };
        use mycelix_institutional_core::{DecisionId, EvidenceRequirement};

        let case_id = AdministrativeCaseId::new("case:1").unwrap();
        let profile = ProcedureProfileId::new("procedure:permit:v1").unwrap();
        let filed = AdministrativeCase {
            protocol_version: PROTOCOL_VERSION.into(),
            id: case_id.clone(),
            institution: source_institution(),
            jurisdiction: Some(jurisdiction()),
            rulebook: source_rulebook(),
            procedure_profile: profile.clone(),
            subject_ref: "permit:parcel:42".into(),
            filed_by: p("did:example:applicant"),
            filed_at_ms: 1_000,
            state: AdministrativeCaseState::Filed,
        };
        let transitions = vec![
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: case_id.clone(),
                next_state: PreDecisionState::EvidenceOpen {
                    opened_at_ms: 2_000,
                },
            },
            PreDecisionTransition {
                protocol_version: PROTOCOL_VERSION.into(),
                case_id: case_id.clone(),
                next_state: PreDecisionState::ReadyForDecision { ready_at_ms: 3_000 },
            },
        ];
        let decision_policy = AdministrativeDecisionPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            procedure_profile: profile.clone(),
            required_capability: CapabilityId::new("administration.decide").unwrap(),
            accepted_roles: vec![RoleId::new("role:permit-officer").unwrap()],
            authority_evidence: Vec::<EvidenceRequirement>::new(),
        };
        let lineage = qualify_case_lineage(filed, &transitions, decision_policy).unwrap();
        let completeness = ProceduralCompletenessPolicy {
            protocol_version: crate::ADMIN_002_PROTOCOL_VERSION.into(),
            procedure_profile: profile,
            policy_ref: "policy:permit:v1".into(),
            policy_digest: d(9),
            policy_digest_profile: "permit-policy-v1-blake3".into(),
            required_notice_recipients: vec![p("did:example:applicant")],
            required_response_recipients: vec![p("did:example:applicant")],
            response_mode: ResponseModeRequirement::Written,
            min_response_window_ms: 500,
            reasons: ReasonsRequirement::AtLeastOne,
        };
        let notice = NoticeReceipt {
            protocol_version: crate::ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id.clone(),
            recipient: p("did:example:applicant"),
            served_at_ms: 1_500,
            content_digest: d(10),
            proof_ref: "proof:notice:1".into(),
        };
        let response = ResponseOpportunityReceipt {
            protocol_version: crate::ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id.clone(),
            recipient: p("did:example:applicant"),
            mode: ResponseMode::Written,
            opened_at_ms: 1_600,
            closed_at_ms: 2_500,
            proof_ref: "proof:response-window:1".into(),
        };
        let closure = EvidenceClosureReceipt {
            protocol_version: crate::ADMIN_002_PROTOCOL_VERSION.into(),
            case_id: case_id.clone(),
            closed_at_ms: 2_800,
            decision_evidence: vec![],
            proof_ref: "proof:evidence-closure:1".into(),
        };
        let complete =
            qualify_procedural_completeness(lineage, completeness, &[notice], &[response], closure)
                .unwrap();
        let grant = AuthorityGrant {
            protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:permit-officer:1").unwrap(),
            holder: p("did:example:officer"),
            institution: source_institution(),
            jurisdiction: Some(jurisdiction()),
            roles: vec![RoleId::new("role:permit-officer").unwrap()],
            capabilities: vec![CapabilityId::new("administration.decide").unwrap()],
            rulebook: source_rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Credential,
                reference: "credential:permit-officer:1".into(),
                proof_ref: "proof:permit-officer:1".into(),
            }],
            issued_at_ms: 500,
            expires_at_ms: 100_000,
            delegated_from: None,
            grant_proof_ref: "proof:grant:permit-officer:1".into(),
        };
        let envelope = AdministrativeDecisionEnvelope {
            protocol_version: PROTOCOL_VERSION.into(),
            case_id,
            jurisdiction: Some(jurisdiction()),
            decider: p("did:example:officer"),
            authority_grant_id: grant.id.clone(),
            decision: Decision {
                protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                id: DecisionId::new("decision:1").unwrap(),
                institution: source_institution(),
                rulebook: source_rulebook(),
                subject_ref: "permit:parcel:42".into(),
                outcome_code: "granted".into(),
                reasons: vec!["requirements satisfied".into()],
                evidence: vec![],
                advisory_inputs: vec![],
                decided_at_ms: 4_000,
                decision_proof_ref: "proof:decision:1".into(),
            },
        };
        let qualified = qualify_administrative_decision(complete, envelope, &grant, &[]).unwrap();
        crate::issue_qualified_decision(qualified).unwrap()
    }

    fn challenge(reviewable: QualifiedReviewableDecision) -> QualifiedChallenge {
        qualify_challenge(
            reviewable,
            ChallengeSubmission {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                challenge: Challenge {
                    protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                    id: mycelix_institutional_core::ChallengeId::new("challenge:1").unwrap(),
                    decision_id: DecisionId::new("decision:1").unwrap(),
                    challenger: p("did:example:applicant"),
                    grounds_code: "procedure.error".into(),
                    evidence: vec![],
                    filed_at_ms: 5_000,
                },
                standing_proof_ref: "proof:standing:1".into(),
                service_proof_ref: "proof:challenge-service:1".into(),
            },
        )
        .unwrap()
    }

    fn appeal(challenge: QualifiedChallenge) -> QualifiedAppealReview {
        qualify_appeal(
            challenge,
            Appeal {
                protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                id: AppealId::new("appeal:1").unwrap(),
                challenge_id: mycelix_institutional_core::ChallengeId::new("challenge:1").unwrap(),
                appellant: p("did:example:applicant"),
                forum: review_forum(),
                rulebook: review_rulebook(),
                filed_at_ms: 6_000,
            },
        )
        .unwrap()
    }

    #[test]
    fn exact_review_lineage_reaches_administrative_finality() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let disposition = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:reviewer"),
                authority_grant_id: grant.id.clone(),
                outcome: ReviewDispositionOutcome::Remand,
                reasons: vec!["record requires reconsideration".into()],
                evidence: vec![],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:1".into(),
                proof_ref: "proof:review-disposition:1".into(),
                authorized_remedy_types: vec!["remand".into()],
            },
            &grant,
            &[],
        )
        .unwrap();
        let finality = qualify_administrative_finality(
            disposition,
            AdministrativeFinalityReceipt {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                disposition_ref: "review-disposition:1".into(),
                finalized_at_ms: 8_000,
                proof_ref: "proof:admin-finality:1".into(),
            },
        )
        .unwrap();
        assert_eq!(finality.decision_id().as_str(), "decision:1");
        assert!(!finality.grants_external_effect_authority());
    }

    #[test]
    fn untimely_challenge_fails_closed() {
        let mut policy = review_policy();
        policy.challenge_window_ms = 100;
        let reviewable = qualify_reviewable_decision(issued_decision(), policy).unwrap();
        let error = qualify_challenge(
            reviewable,
            ChallengeSubmission {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                challenge: Challenge {
                    protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                    id: mycelix_institutional_core::ChallengeId::new("challenge:late").unwrap(),
                    decision_id: DecisionId::new("decision:1").unwrap(),
                    challenger: p("did:example:applicant"),
                    grounds_code: "procedure.error".into(),
                    evidence: vec![],
                    filed_at_ms: 5_000,
                },
                standing_proof_ref: "proof:standing:late".into(),
                service_proof_ref: "proof:service:late".into(),
            },
        )
        .unwrap_err();
        assert_eq!(error, AdministrativeReviewError::ChallengeUntimely);
    }

    #[test]
    fn original_decider_cannot_review_when_independence_required() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let mut grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        grant.holder = p("did:example:officer");
        let error = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:officer"),
                authority_grant_id: grant.id.clone(),
                outcome: ReviewDispositionOutcome::Affirm,
                reasons: vec!["affirmed".into()],
                evidence: vec![],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:self".into(),
                proof_ref: "proof:self-review".into(),
                authorized_remedy_types: vec![],
            },
            &grant,
            &[],
        )
        .unwrap_err();
        assert_eq!(error, AdministrativeReviewError::ReviewerNotIndependent);
    }

    #[test]
    fn stay_requires_separate_stay_capability() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let error = apply_stay_directive(
            review,
            StayDirective {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                kind: StayDirectiveKind::Impose,
                authorized_by: grant.id.clone(),
                issued_at_ms: 6_500,
                expires_at_ms: Some(9_000),
                proof_ref: "proof:stay:1".into(),
            },
            &grant,
            &[],
        )
        .unwrap_err();
        assert_eq!(error, AdministrativeReviewError::ReviewAuthorityDenied);
    }

    #[test]
    fn affirm_cannot_smuggle_corrective_remedy() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let error = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:reviewer"),
                authority_grant_id: grant.id.clone(),
                outcome: ReviewDispositionOutcome::Affirm,
                reasons: vec!["affirmed".into()],
                evidence: vec![],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:affirm".into(),
                proof_ref: "proof:affirm".into(),
                authorized_remedy_types: vec!["vacate".into()],
            },
            &grant,
            &[],
        )
        .unwrap_err();
        assert_eq!(
            error,
            AdministrativeReviewError::AffirmCannotAuthorizeRemedy
        );
    }

    #[test]
    fn remedy_requires_exact_disposition_authorization_and_capability() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let reviewer_grant =
            review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let disposition = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:reviewer"),
                authority_grant_id: reviewer_grant.id.clone(),
                outcome: ReviewDispositionOutcome::Vacate,
                reasons: vec!["material procedural defect".into()],
                evidence: vec![],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:vacate".into(),
                proof_ref: "proof:vacate".into(),
                authorized_remedy_types: vec!["vacate".into()],
            },
            &reviewer_grant,
            &[],
        )
        .unwrap();
        let finality = qualify_administrative_finality(
            disposition,
            AdministrativeFinalityReceipt {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                disposition_ref: "review-disposition:vacate".into(),
                finalized_at_ms: 8_000,
                proof_ref: "proof:admin-finality:vacate".into(),
            },
        )
        .unwrap();
        let remedy_grant = review_grant(vec![CapabilityId::new("administration.remedy").unwrap()]);
        let qualified = qualify_remedy(
            &finality,
            Remedy {
                protocol_version: INSTITUTIONAL_PROTOCOL_VERSION.into(),
                id: mycelix_institutional_core::RemedyId::new("remedy:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                remedy_type: "vacate".into(),
                description: "vacate original permit decision".into(),
                authorized_by: remedy_grant.id.clone(),
                issued_at_ms: 8_100,
                expires_at_ms: None,
            },
            &remedy_grant,
            &[],
        )
        .unwrap();
        assert_eq!(qualified.remedy().remedy_type, "vacate");
        assert!(!qualified.grants_external_effect_authority());
    }

    #[test]
    fn judicial_finality_is_recorded_not_minted_as_authority() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let disposition = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:reviewer"),
                authority_grant_id: grant.id.clone(),
                outcome: ReviewDispositionOutcome::Affirm,
                reasons: vec!["affirmed".into()],
                evidence: vec![],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:final".into(),
                proof_ref: "proof:review-final".into(),
                authorized_remedy_types: vec![],
            },
            &grant,
            &[],
        )
        .unwrap();
        let finality = qualify_administrative_finality(
            disposition,
            AdministrativeFinalityReceipt {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                disposition_ref: "review-disposition:final".into(),
                finalized_at_ms: 8_000,
                proof_ref: "proof:admin-finality:final".into(),
            },
        )
        .unwrap();
        let recorded = record_external_judicial_finality(
            &finality,
            ExternalJudicialFinalityReference {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                administrative_disposition_ref: "review-disposition:final".into(),
                court_forum_ref: "court:superior:1".into(),
                judgment_ref: "judgment:123".into(),
                judgment_digest: d(31),
                judgment_digest_profile: "court-judgment-v1-sha256".into(),
                finalized_at_ms: 9_000,
                proof_ref: "proof:court-judgment:123".into(),
            },
        )
        .unwrap();
        assert!(!recorded.grants_authority());
        assert!(!recorded.grants_external_effect_authority());
    }

    #[test]
    fn future_review_evidence_fails_closed() {
        let reviewable = qualify_reviewable_decision(issued_decision(), review_policy()).unwrap();
        let review = appeal(challenge(reviewable));
        let grant = review_grant(vec![CapabilityId::new("administration.review").unwrap()]);
        let error = qualify_review_disposition(
            review,
            AdministrativeReviewDisposition {
                protocol_version: ADMIN_003_PROTOCOL_VERSION.into(),
                appeal_id: AppealId::new("appeal:1").unwrap(),
                decision_id: DecisionId::new("decision:1").unwrap(),
                reviewer: p("did:example:reviewer"),
                authority_grant_id: grant.id.clone(),
                outcome: ReviewDispositionOutcome::Affirm,
                reasons: vec!["affirmed".into()],
                evidence: vec![EvidenceRef {
                    id: EvidenceId::new("evidence:future").unwrap(),
                    evidence_type: "review.record".into(),
                    issuer: None,
                    digest: Some(d(44)),
                    observed_at_ms: 7_001,
                    proof_ref: Some("proof:evidence:future".into()),
                }],
                decided_at_ms: 7_000,
                disposition_ref: "review-disposition:future".into(),
                proof_ref: "proof:future".into(),
                authorized_remedy_types: vec![],
            },
            &grant,
            &[],
        )
        .unwrap_err();
        assert_eq!(error, AdministrativeReviewError::EvidenceFromFuture);
    }
}
