// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Constitution-rooted historical operational authority at one exact causal position.
//!
//! This is the historical sibling of `mycelix-authority-operational-freshness` (#117).
//! It preserves the same already-current operational policy context and the same fresh
//! challenge/source/witness coverage qualification, but it never converts the covered
//! authority-state lineage into live current freshness.
//!
//! Instead, the caller supplies one already-signed causal coordinate
//! `(generation, transition digest)`. The covered lineage is projected through #429 and
//! the selected exact state must pass #446's historical `Active` gate. Success therefore
//! means that one exact operational authority subject was historically Active at the
//! signer-committed causal coordinate under the exact current constitution-rooted policy
//! context and fresh complete-source evidence used by this qualification.
//!
//! The result is deliberately non-deserializable and exposes no current-freshness or
//! current-operational conversion.

#![forbid(unsafe_code)]

use mycelix_authority_causal_active::{
    qualify_active_causal_authority_at, QualifiedActiveCausalAuthority,
};
use mycelix_authority_causal_projection::project_authority_state_at_causal_position;
use mycelix_authority_freshness::AuthoritySubjectRef;
use mycelix_authority_operational_context::QualifiedOperationalPolicyContext;
use mycelix_authority_state_coverage::{
    VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    qualify_context_bound_coverage, VerifiedCoverageChallenge, VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::VerifiedAuthorityStateTransition;
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-historical-operational-v0.1";
pub const AUTHORITY_IDENTITY_PROFILE: &str =
    "mycelix-authority-historical-operational-v1-blake3-framed";
pub const EVIDENCE_IDENTITY_PROFILE: &str =
    "mycelix-authority-historical-operational-evidence-v1-blake3-framed";

const DOMAIN_AUTHORITY: &[u8] = b"mycelix/authority/historical-operational/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority/historical-operational-evidence/v1";

/// Non-deserializable constitution-rooted positive historical authority for one exact
/// operational subject and one exact signer-committed causal authority-state coordinate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedHistoricalOperationalAuthority {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    operational_context_digest: Digest32,
    operational_context_profile: String,
    subject: AuthoritySubjectRef,
    target_generation: u64,
    selected_transition_digest: Digest32,
    causal_projection_digest: Digest32,
    active_causal_authority_digest: Digest32,
    full_lineage_digest: Digest32,
    current_head_generation: u64,
    current_head_transition_digest: Digest32,
    context_coverage_digest: Digest32,
    context_coverage_profile: String,
    authority_digest: Digest32,
    authority_profile: String,
    evidence_digest: Digest32,
    evidence_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedHistoricalOperationalAuthority {
    pub fn root_qualification_digest(&self) -> Digest32 {
        self.root_qualification_digest
    }

    pub fn root_qualification_profile(&self) -> &str {
        &self.root_qualification_profile
    }

    pub fn operational_context_digest(&self) -> Digest32 {
        self.operational_context_digest
    }

    pub fn operational_context_profile(&self) -> &str {
        &self.operational_context_profile
    }

    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn target_generation(&self) -> u64 {
        self.target_generation
    }

    pub fn selected_transition_digest(&self) -> Digest32 {
        self.selected_transition_digest
    }

    pub fn causal_projection_digest(&self) -> Digest32 {
        self.causal_projection_digest
    }

    pub fn active_causal_authority_digest(&self) -> Digest32 {
        self.active_causal_authority_digest
    }

    pub fn full_lineage_digest(&self) -> Digest32 {
        self.full_lineage_digest
    }

    pub fn current_head_generation(&self) -> u64 {
        self.current_head_generation
    }

    pub fn current_head_transition_digest(&self) -> Digest32 {
        self.current_head_transition_digest
    }

    pub fn context_coverage_digest(&self) -> Digest32 {
        self.context_coverage_digest
    }

    pub fn context_coverage_profile(&self) -> &str {
        &self.context_coverage_profile
    }

    pub fn authority_digest(&self) -> Digest32 {
        self.authority_digest
    }

    pub fn authority_profile(&self) -> &str {
        &self.authority_profile
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        &self.evidence_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

/// Qualify one exact historical operational authority through the same current
/// constitution-rooted policy context and challenge/source/witness coverage used by #117,
/// but select the authority state by the exact signed causal coordinate rather than by the
/// current head.
#[allow(clippy::too_many_arguments)]
pub fn qualify_historical_operational_authority(
    context: &QualifiedOperationalPolicyContext,
    challenge_receipt: &VerifiedCoverageChallenge,
    source_receipt: &VerifiedAuthoritySourceHead,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    trust_bindings: &[VerifiedWitnessTrustBinding],
    transition_receipts: &[VerifiedAuthorityStateTransition],
    target_generation: u64,
    target_transition_digest: Digest32,
    now_ms: u64,
) -> Result<QualifiedHistoricalOperationalAuthority, HistoricalOperationalAuthorityError> {
    if now_ms == 0 {
        return Err(HistoricalOperationalAuthorityError::InvalidVerificationTime);
    }
    context
        .validate_current_at(now_ms)
        .map_err(|_| HistoricalOperationalAuthorityError::OperationalContextNotCurrent)?;

    let subject = context.target_subject();
    let challenge = &challenge_receipt.challenge;
    if &challenge.subject != subject {
        return Err(HistoricalOperationalAuthorityError::ChallengeSubjectMismatch);
    }
    if challenge.context_policy_digest != context.context_policy_digest()
        || challenge.coverage_policy_digest != context.coverage_policy_digest()
    {
        return Err(HistoricalOperationalAuthorityError::ChallengePolicyContextMismatch);
    }

    let context_coverage = qualify_context_bound_coverage(
        context.context_receipt(),
        challenge_receipt,
        context.coverage_receipt(),
        source_receipt,
        witness_receipts,
        trust_bindings,
        now_ms,
    )
    .map_err(|_| HistoricalOperationalAuthorityError::CoverageQualificationDenied)?;

    let state_coverage = context_coverage.to_state_source_coverage();
    let projection = project_authority_state_at_causal_position(
        subject,
        transition_receipts,
        &state_coverage,
        target_generation,
        target_transition_digest,
        now_ms,
    )
    .map_err(|_| HistoricalOperationalAuthorityError::CausalProjectionDenied)?;

    let active = qualify_active_causal_authority_at(&projection, now_ms)
        .map_err(|_| HistoricalOperationalAuthorityError::HistoricalActiveAuthorityDenied)?;
    validate_exact_binding(subject, target_generation, target_transition_digest, &active)?;

    let verified_at_ms = active.verified_at_ms().max(context.verified_at_ms());
    let lease_until_ms = active.lease_until_ms().min(context.valid_until_ms());
    if verified_at_ms > now_ms || lease_until_ms <= now_ms {
        return Err(HistoricalOperationalAuthorityError::QualificationWindowInvalid);
    }

    let authority_digest = historical_operational_authority_digest(
        context.bootstrap_root_digest(),
        context.qualification_digest(),
        active.active_authority_digest(),
        target_generation,
        target_transition_digest,
    );
    let evidence_digest = historical_operational_evidence_digest(
        authority_digest,
        context_coverage.context_coverage_digest(),
        active.causal_projection_digest(),
    );

    Ok(QualifiedHistoricalOperationalAuthority {
        root_qualification_digest: context.bootstrap_root_digest(),
        root_qualification_profile: context.bootstrap_root_profile().into(),
        operational_context_digest: context.qualification_digest(),
        operational_context_profile: context.qualification_profile().into(),
        subject: subject.clone(),
        target_generation,
        selected_transition_digest: active.selected_transition_digest(),
        causal_projection_digest: active.causal_projection_digest(),
        active_causal_authority_digest: active.active_authority_digest(),
        full_lineage_digest: active.full_lineage_digest(),
        current_head_generation: active.current_head_generation(),
        current_head_transition_digest: active.current_head_transition_digest(),
        context_coverage_digest: context_coverage.context_coverage_digest(),
        context_coverage_profile: context_coverage.context_coverage_profile().into(),
        authority_digest,
        authority_profile: AUTHORITY_IDENTITY_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_IDENTITY_PROFILE.into(),
        verified_at_ms,
        lease_until_ms,
    })
}

fn validate_exact_binding(
    subject: &AuthoritySubjectRef,
    target_generation: u64,
    target_transition_digest: Digest32,
    active: &QualifiedActiveCausalAuthority,
) -> Result<(), HistoricalOperationalAuthorityError> {
    if active.subject() != subject {
        return Err(HistoricalOperationalAuthorityError::QualifiedSubjectMismatch);
    }
    if active.generation() != target_generation {
        return Err(HistoricalOperationalAuthorityError::QualifiedGenerationMismatch);
    }
    if active.selected_transition_digest() != target_transition_digest {
        return Err(HistoricalOperationalAuthorityError::QualifiedTransitionMismatch);
    }
    Ok(())
}

fn historical_operational_authority_digest(
    root_digest: Digest32,
    operational_context_digest: Digest32,
    active_causal_authority_digest: Digest32,
    target_generation: u64,
    target_transition_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_AUTHORITY);
    frame(&mut hasher, AUTHORITY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &operational_context_digest.0);
    frame(&mut hasher, &active_causal_authority_digest.0);
    frame(&mut hasher, &target_generation.to_le_bytes());
    frame(&mut hasher, &target_transition_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn historical_operational_evidence_digest(
    authority_digest: Digest32,
    context_coverage_digest: Digest32,
    causal_projection_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &authority_digest.0);
    frame(&mut hasher, &context_coverage_digest.0);
    frame(&mut hasher, &causal_projection_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HistoricalOperationalAuthorityError {
    InvalidVerificationTime,
    OperationalContextNotCurrent,
    ChallengeSubjectMismatch,
    ChallengePolicyContextMismatch,
    CoverageQualificationDenied,
    CausalProjectionDenied,
    HistoricalActiveAuthorityDenied,
    QualifiedSubjectMismatch,
    QualifiedGenerationMismatch,
    QualifiedTransitionMismatch,
    QualificationWindowInvalid,
}

impl fmt::Display for HistoricalOperationalAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidVerificationTime => "invalid historical operational authority verification time",
            Self::OperationalContextNotCurrent => "operational policy context is not currently usable",
            Self::ChallengeSubjectMismatch => "historical authority probe belongs to another operational subject",
            Self::ChallengePolicyContextMismatch => "historical authority probe belongs to another operational policy context",
            Self::CoverageQualificationDenied => "historical operational challenge/source/witness coverage qualification denied",
            Self::CausalProjectionDenied => "exact causal historical authority-state projection denied",
            Self::HistoricalActiveAuthorityDenied => "selected exact historical state is not positive Active authority",
            Self::QualifiedSubjectMismatch => "historical positive authority belongs to another subject",
            Self::QualifiedGenerationMismatch => "historical positive authority belongs to another generation",
            Self::QualifiedTransitionMismatch => "historical positive authority belongs to another transition",
            Self::QualificationWindowInvalid => "historical operational authority evidence is stale or not yet valid",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for HistoricalOperationalAuthorityError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn stable_authority_identity_commits_root_context_and_exact_causal_authority() {
        let baseline = historical_operational_authority_digest(d(1), d(2), d(3), 7, d(4));
        assert_eq!(
            baseline,
            historical_operational_authority_digest(d(1), d(2), d(3), 7, d(4))
        );
        assert_ne!(
            baseline,
            historical_operational_authority_digest(d(9), d(2), d(3), 7, d(4))
        );
        assert_ne!(
            baseline,
            historical_operational_authority_digest(d(1), d(8), d(3), 7, d(4))
        );
        assert_ne!(
            baseline,
            historical_operational_authority_digest(d(1), d(2), d(6), 7, d(4))
        );
        assert_ne!(
            baseline,
            historical_operational_authority_digest(d(1), d(2), d(3), 8, d(4))
        );
        assert_ne!(
            baseline,
            historical_operational_authority_digest(d(1), d(2), d(3), 7, d(5))
        );
    }

    #[test]
    fn fresh_coverage_instance_changes_evidence_not_stable_authority() {
        let authority = historical_operational_authority_digest(d(1), d(2), d(3), 7, d(4));
        let evidence_a = historical_operational_evidence_digest(authority, d(5), d(6));
        let evidence_b = historical_operational_evidence_digest(authority, d(7), d(6));
        assert_ne!(evidence_a, evidence_b);
        assert_eq!(
            authority,
            historical_operational_authority_digest(d(1), d(2), d(3), 7, d(4))
        );
    }

    #[test]
    fn positive_type_is_not_deserializable_by_derivation() {
        let source = include_str!("lib.rs");
        let before = source
            .split("pub struct QualifiedHistoricalOperationalAuthority")
            .next()
            .unwrap();
        let derive = before.rsplit("#[derive(").next().unwrap_or_default();
        assert!(!derive.contains("Deserialize"));
    }
}
