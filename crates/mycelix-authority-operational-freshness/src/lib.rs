// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Covered current freshness for ordinary operational authority subjects.
//!
//! This layer composes an already-current operational policy context with one
//! fresh challenge-bound source/witness coverage proof and one complete covered
//! append-only authority-state lineage. It produces current freshness for exactly
//! one operational authority subject without selecting a "latest" record.
//!
//! Stable authority identity and dynamic evidence identity are deliberately
//! separate. Re-probing the same current policy + same authority generation may
//! refresh evidence without inventing a new authority domain.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectRef, CurrentAuthorityFreshness,
    VerifiedAuthorityFreshness,
};
use mycelix_authority_operational_context::QualifiedOperationalPolicyContext;
use mycelix_authority_state_coverage::{
    VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    qualify_context_bound_coverage, VerifiedCoverageChallenge, VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::{
    project_current_authority_state, VerifiedAuthorityStateTransition,
};
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-operational-freshness-v0.1";
pub const AUTHORITY_IDENTITY_PROFILE: &str =
    "mycelix-authority-operational-current-v1-blake3-framed";
pub const EVIDENCE_IDENTITY_PROFILE: &str =
    "mycelix-authority-operational-current-evidence-v1-blake3-framed";
pub const SET_IDENTITY_PROFILE: &str =
    "mycelix-authority-operational-current-set-v1-blake3-framed";

const DOMAIN_AUTHORITY: &[u8] = b"mycelix/authority/operational-current/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/authority/operational-current-evidence/v1";
const DOMAIN_SET: &[u8] = b"mycelix/authority/operational-current-set/v1";
const MAX_OPERATIONAL_SUBJECTS: usize = 64;

/// Non-deserializable proof that one exact operational authority subject is
/// currently covered under one exact current operational policy context.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedOperationalSubjectFreshness {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    operational_context_digest: Digest32,
    operational_context_profile: String,
    subject: AuthoritySubjectRef,
    state_projection_digest: Digest32,
    state_projection_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    /// Stable current authority qualification. Excludes fresh challenge/coverage
    /// instance identity so same-generation re-verification does not mint a new
    /// authority domain.
    authority_digest: Digest32,
    authority_profile: String,
    /// Exact challenge/source/witness proof instance used for this qualification.
    evidence_digest: Digest32,
    evidence_profile: String,
    context_coverage_digest: Digest32,
    context_coverage_profile: String,
}

impl QualifiedOperationalSubjectFreshness {
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

    pub fn state_projection_digest(&self) -> Digest32 {
        self.state_projection_digest
    }

    pub fn state_projection_profile(&self) -> &str {
        &self.state_projection_profile
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

    pub fn context_coverage_digest(&self) -> Digest32 {
        self.context_coverage_digest
    }

    pub fn context_coverage_profile(&self) -> &str {
        &self.context_coverage_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.current_freshness.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.current_freshness.lease_until_ms
    }

    /// Adapter projection only. Runtime callers must first obtain this
    /// non-deserializable qualified value from the pure composition kernel.
    pub fn to_verified_freshness(&self) -> VerifiedAuthorityFreshness {
        self.current_freshness.clone()
    }
}

/// Stable closed-set qualification over exact operational subjects. Individual
/// subjects may use different operational policy contexts, but all must descend
/// from one exact bootstrap-root epoch.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedOperationalFreshnessSet {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    current: CurrentAuthorityFreshness,
    authority_digest: Digest32,
    authority_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedOperationalFreshnessSet {
    pub fn root_qualification_digest(&self) -> Digest32 {
        self.root_qualification_digest
    }

    pub fn root_qualification_profile(&self) -> &str {
        &self.root_qualification_profile
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.current.freshness_digest
    }

    pub fn freshness_profile(&self) -> &str {
        &self.current.freshness_profile
    }

    pub fn authority_digest(&self) -> Digest32 {
        self.authority_digest
    }

    pub fn authority_profile(&self) -> &str {
        &self.authority_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

/// Qualify one exact operational subject through current policy context, fresh
/// challenge-bound source coverage and complete current state projection.
#[allow(clippy::too_many_arguments)]
pub fn qualify_operational_subject_freshness(
    context: &QualifiedOperationalPolicyContext,
    challenge_receipt: &VerifiedCoverageChallenge,
    source_receipt: &VerifiedAuthoritySourceHead,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    trust_bindings: &[VerifiedWitnessTrustBinding],
    transition_receipts: &[VerifiedAuthorityStateTransition],
    now_ms: u64,
) -> Result<QualifiedOperationalSubjectFreshness, OperationalFreshnessError> {
    context
        .validate_current_at(now_ms)
        .map_err(|_| OperationalFreshnessError::OperationalContextNotCurrent)?;

    let subject = context.target_subject();
    let challenge = &challenge_receipt.challenge;
    if &challenge.subject != subject {
        return Err(OperationalFreshnessError::ChallengeSubjectMismatch);
    }
    if challenge.context_policy_digest != context.context_policy_digest()
        || challenge.coverage_policy_digest != context.coverage_policy_digest()
    {
        return Err(OperationalFreshnessError::ChallengePolicyContextMismatch);
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
    .map_err(|_| OperationalFreshnessError::CoverageQualificationDenied)?;

    let state_coverage = context_coverage.to_state_source_coverage();
    let projection = project_current_authority_state(
        subject,
        transition_receipts,
        &state_coverage,
        now_ms,
    )
    .map_err(|_| OperationalFreshnessError::StateProjectionDenied)?;

    let mut freshness = projection
        .to_current_freshness_receipt()
        .map_err(|_| OperationalFreshnessError::CurrentFreshnessProjectionDenied)?;
    freshness.verified_at_ms = freshness.verified_at_ms.max(context.verified_at_ms());
    freshness.lease_until_ms = freshness.lease_until_ms.min(context.valid_until_ms());

    let snapshot_digest = freshness
        .snapshot
        .identity_digest()
        .map_err(|_| OperationalFreshnessError::CurrentFreshnessProjectionDenied)?;
    let authority_digest = operational_authority_digest(
        context.qualification_digest(),
        projection.projection_digest(),
        snapshot_digest,
    );
    let evidence_digest = operational_evidence_digest(
        authority_digest,
        context_coverage.context_coverage_digest(),
    );
    freshness.verification_ref = format!(
        "operational-freshness-evidence:{EVIDENCE_IDENTITY_PROFILE}:{}",
        hex_digest(evidence_digest)
    );
    freshness
        .validate_at(now_ms)
        .map_err(|_| OperationalFreshnessError::CurrentFreshnessProjectionDenied)?;

    Ok(QualifiedOperationalSubjectFreshness {
        root_qualification_digest: context.bootstrap_root_digest(),
        root_qualification_profile: context.bootstrap_root_profile().into(),
        operational_context_digest: context.qualification_digest(),
        operational_context_profile: context.qualification_profile().into(),
        subject: subject.clone(),
        state_projection_digest: projection.projection_digest(),
        state_projection_profile: projection.projection_profile().into(),
        current_freshness: freshness,
        authority_digest,
        authority_profile: AUTHORITY_IDENTITY_PROFILE.into(),
        evidence_digest,
        evidence_profile: EVIDENCE_IDENTITY_PROFILE.into(),
        context_coverage_digest: context_coverage.context_coverage_digest(),
        context_coverage_profile: context_coverage.context_coverage_profile().into(),
    })
}

/// Compose an exact required set of operational current-authority proofs.
///
/// #74 independently rechecks exact subject coverage and Active current state.
/// The stable set identity additionally commits every per-subject stable authority
/// digest in canonical subject-identity order, so a policy-context change cannot
/// hide behind an unchanged freshness snapshot bundle.
pub fn qualify_operational_freshness_set(
    required_subjects: &[AuthoritySubjectRef],
    qualified_subjects: &[QualifiedOperationalSubjectFreshness],
    now_ms: u64,
) -> Result<QualifiedOperationalFreshnessSet, OperationalFreshnessError> {
    if required_subjects.is_empty() || required_subjects.len() > MAX_OPERATIONAL_SUBJECTS {
        return Err(OperationalFreshnessError::InvalidRequiredSubjectCount);
    }
    if qualified_subjects.len() != required_subjects.len() {
        return Err(OperationalFreshnessError::QualifiedSetSizeMismatch);
    }

    let first = qualified_subjects
        .first()
        .ok_or(OperationalFreshnessError::QualifiedSetSizeMismatch)?;
    let root_digest = first.root_qualification_digest;
    let root_profile = first.root_qualification_profile.as_str();

    let mut freshness_receipts = Vec::with_capacity(qualified_subjects.len());
    let mut authority_items = Vec::<(Digest32, Digest32)>::with_capacity(qualified_subjects.len());
    for qualified in qualified_subjects {
        if qualified.root_qualification_digest != root_digest
            || qualified.root_qualification_profile != root_profile
        {
            return Err(OperationalFreshnessError::BootstrapRootMismatch);
        }
        let receipt = qualified.to_verified_freshness();
        receipt
            .validate_at(now_ms)
            .map_err(|_| OperationalFreshnessError::CurrentFreshnessProjectionDenied)?;
        if receipt.snapshot.subject != qualified.subject {
            return Err(OperationalFreshnessError::QualifiedSubjectMismatch);
        }
        let subject_digest = qualified
            .subject
            .identity_digest()
            .map_err(|_| OperationalFreshnessError::QualifiedSubjectMismatch)?;
        authority_items.push((subject_digest, qualified.authority_digest));
        freshness_receipts.push(receipt);
    }

    let current = qualify_current_freshness(required_subjects, &freshness_receipts, now_ms)
        .map_err(|_| OperationalFreshnessError::ClosedSetFreshnessDenied)?;

    authority_items.sort_by(|left, right| left.0.0.cmp(&right.0.0));
    for pair in authority_items.windows(2) {
        if pair[0].0 == pair[1].0 {
            return Err(OperationalFreshnessError::DuplicateQualifiedSubject);
        }
    }
    let authority_digest = operational_set_digest(
        root_digest,
        current.freshness_digest,
        &authority_items,
    );

    Ok(QualifiedOperationalFreshnessSet {
        root_qualification_digest: root_digest,
        root_qualification_profile: root_profile.into(),
        verified_at_ms: current.verified_at_ms,
        lease_until_ms: current.lease_until_ms,
        current,
        authority_digest,
        authority_profile: SET_IDENTITY_PROFILE.into(),
    })
}

fn operational_authority_digest(
    context_digest: Digest32,
    state_projection_digest: Digest32,
    snapshot_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_AUTHORITY);
    frame(&mut hasher, AUTHORITY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &context_digest.0);
    frame(&mut hasher, &state_projection_digest.0);
    frame(&mut hasher, &snapshot_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn operational_evidence_digest(
    authority_digest: Digest32,
    context_coverage_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &authority_digest.0);
    frame(&mut hasher, &context_coverage_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn operational_set_digest(
    root_digest: Digest32,
    freshness_digest: Digest32,
    authority_items: &[(Digest32, Digest32)],
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SET);
    frame(&mut hasher, SET_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &freshness_digest.0);
    frame(&mut hasher, &(authority_items.len() as u64).to_le_bytes());
    for (subject_digest, authority_digest) in authority_items {
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, &authority_digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OperationalFreshnessError {
    OperationalContextNotCurrent,
    ChallengeSubjectMismatch,
    ChallengePolicyContextMismatch,
    CoverageQualificationDenied,
    StateProjectionDenied,
    CurrentFreshnessProjectionDenied,
    InvalidRequiredSubjectCount,
    QualifiedSetSizeMismatch,
    BootstrapRootMismatch,
    QualifiedSubjectMismatch,
    DuplicateQualifiedSubject,
    ClosedSetFreshnessDenied,
}

impl fmt::Display for OperationalFreshnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::OperationalContextNotCurrent => "operational policy context is not currently usable",
            Self::ChallengeSubjectMismatch => "freshness probe belongs to another operational subject",
            Self::ChallengePolicyContextMismatch => "freshness probe belongs to another operational policy context",
            Self::CoverageQualificationDenied => "operational challenge/source/witness coverage qualification denied",
            Self::StateProjectionDenied => "covered operational authority-state projection denied",
            Self::CurrentFreshnessProjectionDenied => "current operational freshness projection denied",
            Self::InvalidRequiredSubjectCount => "invalid required operational subject count",
            Self::QualifiedSetSizeMismatch => "qualified operational subject count does not match requirements",
            Self::BootstrapRootMismatch => "operational subject proofs belong to different bootstrap-root epochs",
            Self::QualifiedSubjectMismatch => "qualified operational subject does not match its current freshness receipt",
            Self::DuplicateQualifiedSubject => "duplicate qualified operational subject",
            Self::ClosedSetFreshnessDenied => "closed-set operational freshness qualification denied",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for OperationalFreshnessError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject(byte: u8) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test:authority".into(),
            subject_id: format!("grant:{byte}"),
            identity: ProfiledDigest {
                digest: d(byte),
                profile: "grant-v1-blake3".into(),
            },
        }
    }

    #[test]
    fn fresh_probe_changes_evidence_not_stable_authority_identity() {
        let authority_a = operational_authority_digest(d(1), d(2), d(3));
        let authority_b = operational_authority_digest(d(1), d(2), d(3));
        assert_eq!(authority_a, authority_b);

        let evidence_a = operational_evidence_digest(authority_a, d(4));
        let evidence_b = operational_evidence_digest(authority_b, d(5));
        assert_ne!(evidence_a, evidence_b);
    }

    #[test]
    fn policy_or_generation_change_changes_stable_authority_identity() {
        let baseline = operational_authority_digest(d(1), d(2), d(3));
        assert_ne!(baseline, operational_authority_digest(d(9), d(2), d(3)));
        assert_ne!(baseline, operational_authority_digest(d(1), d(8), d(3)));
        assert_ne!(baseline, operational_authority_digest(d(1), d(2), d(7)));
    }

    #[test]
    fn set_identity_is_independent_of_input_order_after_canonicalization() {
        let a = subject(1).identity_digest().unwrap();
        let b = subject(2).identity_digest().unwrap();
        let mut first = vec![(a, d(11)), (b, d(12))];
        let mut second = vec![(b, d(12)), (a, d(11))];
        first.sort_by(|left, right| left.0.0.cmp(&right.0.0));
        second.sort_by(|left, right| left.0.0.cmp(&right.0.0));
        assert_eq!(
            operational_set_digest(d(20), d(21), &first),
            operational_set_digest(d(20), d(21), &second)
        );
    }
}
