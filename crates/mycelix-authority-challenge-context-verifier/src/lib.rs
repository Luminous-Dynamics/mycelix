// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure composition boundary for current authority-state challenge context.
//!
//! This crate does not discover policy, verify signatures, persist state, or mint
//! freshness. It only composes independently verified semantic policy/issuer
//! receipts with one exact current freshness bundle.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef,
    CurrentAuthorityFreshness, ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, CoverageMode, VerifiedAuthorityCoveragePolicy,
    POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-challenge-context-v0.1";
pub const ISSUER_GRANT_RECEIPT_PROTOCOL: &str =
    "mycelix-authority-state-challenge-issuer-grant-v0.1";
pub const WITNESS_TRUST_RECEIPT_PROTOCOL: &str =
    "mycelix-authority-state-witness-trust-authority-v0.1";
pub const CONTEXT_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-current-challenge-context-v1-blake3-framed";
pub const REQUIRED_CHALLENGE_CAPABILITY: &str = "authority_state.challenge_issue";

const DOMAIN_CONTEXT: &[u8] = b"mycelix/authority-state/current-challenge-context/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Independently verified proof that one exact institutional AuthorityGrant gives
/// one principal the bounded capability to issue authority-state challenges.
///
/// The receipt does not manufacture grant semantics. `grant_subject` must be the
/// exact canonical grant identity supplied by the grant-verification boundary and
/// is independently required to be current/Active through authority freshness.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedChallengeIssuerGrant {
    pub protocol_version: String,
    pub grant_subject: AuthoritySubjectRef,
    pub issuer_did: String,
    pub institution_ref: String,
    pub jurisdiction_ref: Option<String>,
    pub rulebook_ref: String,
    pub verified_capability: String,
    pub authority_ref: String,
    pub grant_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Independently verified semantic authority for the exact witness-trust policy
/// named by the context policy. Required only in WitnessQuorum mode.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedWitnessTrustPolicyAuthority {
    pub protocol_version: String,
    pub policy: ProfiledDigest,
    pub institution_ref: String,
    pub jurisdiction_ref: Option<String>,
    pub rulebook_ref: String,
    pub authority_ref: String,
    pub policy_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable result. A positive value can only be constructed by
/// requalifying all exact semantic receipts against one exact current freshness set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedChallengeContext {
    protocol: String,
    subject: AuthoritySubjectRef,
    context_policy_digest: Digest32,
    context_policy_profile: String,
    coverage_policy_digest: Digest32,
    coverage_policy_profile: String,
    witness_trust_policy: Option<ProfiledDigest>,
    issuer_grant_subject_digest: Digest32,
    issuer_grant_subject_profile: String,
    freshness_digest: Digest32,
    freshness_profile: String,
    max_challenge_lifetime_ms: u64,
    context_valid_until_ms: u64,
    challenge_issuer_did: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
    context_digest: Digest32,
    context_profile: String,
}

impl QualifiedChallengeContext {
    pub fn protocol(&self) -> &str { &self.protocol }
    pub fn subject(&self) -> &AuthoritySubjectRef { &self.subject }
    pub fn context_policy_digest(&self) -> Digest32 { self.context_policy_digest }
    pub fn context_policy_profile(&self) -> &str { &self.context_policy_profile }
    pub fn coverage_policy_digest(&self) -> Digest32 { self.coverage_policy_digest }
    pub fn coverage_policy_profile(&self) -> &str { &self.coverage_policy_profile }
    pub fn max_challenge_lifetime_ms(&self) -> u64 { self.max_challenge_lifetime_ms }
    pub fn context_valid_until_ms(&self) -> u64 { self.context_valid_until_ms }
    pub fn challenge_issuer_did(&self) -> &str { &self.challenge_issuer_did }
    pub fn verification_ref(&self) -> &str { &self.verification_ref }
    pub fn verified_at_ms(&self) -> u64 { self.verified_at_ms }
    pub fn valid_until_ms(&self) -> u64 { self.valid_until_ms }
    pub fn freshness_digest(&self) -> Digest32 { self.freshness_digest }
    pub fn freshness_profile(&self) -> &str { &self.freshness_profile }
    pub fn context_digest(&self) -> Digest32 { self.context_digest }
    pub fn context_profile(&self) -> &str { &self.context_profile }
}

#[allow(clippy::too_many_arguments)]
pub fn qualify_challenge_context(
    subject: &AuthoritySubjectRef,
    context_receipt: &VerifiedCoverageTrustContextPolicy,
    coverage_receipt: &VerifiedAuthorityCoveragePolicy,
    witness_trust_receipt: Option<&VerifiedWitnessTrustPolicyAuthority>,
    issuer_receipt: &VerifiedChallengeIssuerGrant,
    freshness_receipts: &[VerifiedAuthorityFreshness],
    now_ms: u64,
) -> Result<QualifiedChallengeContext, ChallengeContextError> {
    if now_ms == 0 {
        return Err(ChallengeContextError::InvalidVerificationTime);
    }
    subject
        .validate()
        .map_err(|_| ChallengeContextError::InvalidSubject)?;

    let context = verify_context_receipt(context_receipt, now_ms)?;
    let coverage = verify_coverage_receipt(coverage_receipt, now_ms)?;
    let context_digest = context.identity_digest().map_err(|_| ChallengeContextError::InvalidContextPolicy)?;
    let coverage_digest = coverage.identity_digest().map_err(|_| ChallengeContextError::InvalidCoveragePolicy)?;

    if context.coverage_policy.digest != coverage_digest
        || context.coverage_policy.profile != POLICY_IDENTITY_PROFILE
    {
        return Err(ChallengeContextError::CoveragePolicyMismatch);
    }

    let witness_policy = verify_witness_mode(context, coverage, witness_trust_receipt, now_ms)?;
    verify_issuer(issuer_receipt, context, now_ms)?;

    let coverage_subject = policy_subject(
        AuthoritySubjectKind::AuthorityCoveragePolicy,
        &coverage.namespace,
        &coverage.policy_id,
        coverage_digest,
        POLICY_IDENTITY_PROFILE,
    )?;
    let context_subject = policy_subject(
        AuthoritySubjectKind::CoverageTrustContextPolicy,
        &context.institution_ref,
        &context.context_policy_id,
        context_digest,
        CONTEXT_POLICY_PROFILE,
    )?;

    let mut required = vec![
        coverage_subject,
        context_subject,
        issuer_receipt.grant_subject.clone(),
    ];
    if let Some(policy) = &witness_policy {
        required.push(policy_subject(
            AuthoritySubjectKind::WitnessTrustPolicy,
            &context.institution_ref,
            &witness_policy_subject_id(policy),
            policy.digest,
            &policy.profile,
        )?);
    }

    let current = qualify_current_freshness(&required, freshness_receipts, now_ms)
        .map_err(|_| ChallengeContextError::CurrentFreshnessDenied)?;

    let issuer_subject_digest = issuer_receipt
        .grant_subject
        .identity_digest()
        .map_err(|_| ChallengeContextError::InvalidIssuerGrant)?;

    let mut verified_at_ms = current
        .verified_at_ms
        .max(context_receipt.verified_at_ms)
        .max(coverage_receipt.verified_at_ms)
        .max(issuer_receipt.verified_at_ms);
    let mut valid_until_ms = current
        .lease_until_ms
        .min(context.valid_until_ms)
        .min(coverage.valid_until_ms)
        .min(issuer_receipt.valid_until_ms);
    if let Some(receipt) = witness_trust_receipt {
        verified_at_ms = verified_at_ms.max(receipt.verified_at_ms);
        valid_until_ms = valid_until_ms.min(receipt.valid_until_ms);
    }
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(ChallengeContextError::ContextNotCurrentlyUsable);
    }

    let digest = current_context_digest(
        subject,
        context_digest,
        coverage_digest,
        witness_policy.as_ref(),
        issuer_subject_digest,
        current.freshness_digest,
        &issuer_receipt.issuer_did,
    )?;
    let verification_ref = format!(
        "current-challenge-context:{}:{}",
        CONTEXT_IDENTITY_PROFILE,
        hex_digest(digest)
    );

    Ok(QualifiedChallengeContext {
        protocol: PROTOCOL_VERSION.into(),
        subject: subject.clone(),
        context_policy_digest: context_digest,
        context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
        coverage_policy_digest: coverage_digest,
        coverage_policy_profile: POLICY_IDENTITY_PROFILE.into(),
        witness_trust_policy: witness_policy,
        issuer_grant_subject_digest: issuer_subject_digest,
        issuer_grant_subject_profile: mycelix_authority_freshness::SUBJECT_IDENTITY_PROFILE.into(),
        freshness_digest: current.freshness_digest,
        freshness_profile: current.freshness_profile,
        max_challenge_lifetime_ms: context.max_challenge_lifetime_ms,
        context_valid_until_ms: context.valid_until_ms,
        challenge_issuer_did: issuer_receipt.issuer_did.clone(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
        context_digest: digest,
        context_profile: CONTEXT_IDENTITY_PROFILE.into(),
    })
}

fn verify_context_receipt<'a>(
    receipt: &'a VerifiedCoverageTrustContextPolicy,
    now_ms: u64,
) -> Result<&'a CoverageTrustContextPolicy, ChallengeContextError> {
    let policy = &receipt.policy;
    policy.validate().map_err(|_| ChallengeContextError::InvalidContextPolicy)?;
    if !policy.active_at(now_ms)
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_authority_ref != policy.authority_ref
        || receipt.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(ChallengeContextError::ContextPolicyVerificationMismatch);
    }
    require_ref(&receipt.policy_record_ref)?;
    require_ref(&receipt.verification_ref)?;
    Ok(policy)
}

fn verify_coverage_receipt<'a>(
    receipt: &'a VerifiedAuthorityCoveragePolicy,
    now_ms: u64,
) -> Result<&'a AuthorityCoveragePolicy, ChallengeContextError> {
    let policy = &receipt.policy;
    policy.validate().map_err(|_| ChallengeContextError::InvalidCoveragePolicy)?;
    if !policy.is_active_at(now_ms)
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_authority_ref != policy.authority_ref
        || receipt.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(ChallengeContextError::CoveragePolicyVerificationMismatch);
    }
    require_ref(&receipt.policy_record_ref)?;
    require_ref(&receipt.verification_ref)?;
    Ok(policy)
}

fn verify_witness_mode(
    context: &CoverageTrustContextPolicy,
    coverage: &AuthorityCoveragePolicy,
    receipt: Option<&VerifiedWitnessTrustPolicyAuthority>,
    now_ms: u64,
) -> Result<Option<ProfiledDigest>, ChallengeContextError> {
    match (&coverage.mode, &context.witness_trust_policy, receipt) {
        (CoverageMode::DirectSource, None, None) => Ok(None),
        (CoverageMode::DirectSource, _, _) => Err(ChallengeContextError::UnexpectedWitnessTrustPolicy),
        (CoverageMode::WitnessQuorum { .. }, Some(expected), Some(actual)) => {
            if actual.protocol_version != WITNESS_TRUST_RECEIPT_PROTOCOL
                || &actual.policy != expected
                || actual.institution_ref != context.institution_ref
                || actual.jurisdiction_ref != context.jurisdiction_ref
                || actual.rulebook_ref != context.rulebook_ref
                || actual.verified_at_ms == 0
                || actual.verified_at_ms > now_ms
                || actual.valid_until_ms <= now_ms
            {
                return Err(ChallengeContextError::WitnessTrustAuthorityMismatch);
            }
            for value in [
                actual.authority_ref.as_str(),
                actual.policy_proof_ref.as_str(),
                actual.verification_ref.as_str(),
            ] {
                require_ref(value)?;
            }
            Ok(Some(actual.policy.clone()))
        }
        (CoverageMode::WitnessQuorum { .. }, _, _) => Err(ChallengeContextError::MissingWitnessTrustPolicy),
    }
}

fn verify_issuer(
    receipt: &VerifiedChallengeIssuerGrant,
    context: &CoverageTrustContextPolicy,
    now_ms: u64,
) -> Result<(), ChallengeContextError> {
    if receipt.protocol_version != ISSUER_GRANT_RECEIPT_PROTOCOL
        || receipt.grant_subject.kind != AuthoritySubjectKind::AuthorityGrant
        || receipt.issuer_did.trim().is_empty()
        || receipt.institution_ref != context.institution_ref
        || receipt.jurisdiction_ref != context.jurisdiction_ref
        || receipt.rulebook_ref != context.rulebook_ref
        || receipt.verified_capability != REQUIRED_CHALLENGE_CAPABILITY
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.valid_until_ms <= now_ms
    {
        return Err(ChallengeContextError::InvalidIssuerGrant);
    }
    receipt.grant_subject.validate().map_err(|_| ChallengeContextError::InvalidIssuerGrant)?;
    for value in [
        receipt.issuer_did.as_str(),
        receipt.authority_ref.as_str(),
        receipt.grant_proof_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    Ok(())
}

fn policy_subject(
    kind: AuthoritySubjectKind,
    namespace: &str,
    subject_id: &str,
    digest: Digest32,
    profile: &str,
) -> Result<AuthoritySubjectRef, ChallengeContextError> {
    let subject = AuthoritySubjectRef {
        kind,
        namespace: namespace.into(),
        subject_id: subject_id.into(),
        identity: ProfiledDigest { digest, profile: profile.into() },
    };
    subject.validate().map_err(|_| ChallengeContextError::InvalidPolicyFreshnessSubject)?;
    Ok(subject)
}

fn witness_policy_subject_id(policy: &ProfiledDigest) -> String {
    format!("witness-trust-policy:{}", hex_digest(policy.digest))
}

fn current_context_digest(
    subject: &AuthoritySubjectRef,
    context_policy_digest: Digest32,
    coverage_policy_digest: Digest32,
    witness_policy: Option<&ProfiledDigest>,
    issuer_subject_digest: Digest32,
    freshness_digest: Digest32,
    issuer_did: &str,
) -> Result<Digest32, ChallengeContextError> {
    let subject_digest = subject.identity_digest().map_err(|_| ChallengeContextError::InvalidSubject)?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CONTEXT);
    frame(&mut hasher, CONTEXT_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &subject_digest.0);
    frame(&mut hasher, &context_policy_digest.0);
    frame(&mut hasher, &coverage_policy_digest.0);
    match witness_policy {
        Some(policy) => {
            frame(&mut hasher, &[1]);
            frame(&mut hasher, policy.profile.as_bytes());
            frame(&mut hasher, &policy.digest.0);
        }
        None => frame(&mut hasher, &[0]),
    }
    frame(&mut hasher, &issuer_subject_digest.0);
    frame(&mut hasher, &freshness_digest.0);
    frame(&mut hasher, issuer_did.as_bytes());
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

fn require_ref(value: &str) -> Result<(), ChallengeContextError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ChallengeContextError::InvalidReference)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn hex_digest(digest: Digest32) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(64);
    for byte in digest.0 {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChallengeContextError {
    InvalidReference,
    InvalidSubject,
    InvalidVerificationTime,
    InvalidContextPolicy,
    InvalidCoveragePolicy,
    ContextPolicyVerificationMismatch,
    CoveragePolicyVerificationMismatch,
    CoveragePolicyMismatch,
    UnexpectedWitnessTrustPolicy,
    MissingWitnessTrustPolicy,
    WitnessTrustAuthorityMismatch,
    InvalidIssuerGrant,
    InvalidPolicyFreshnessSubject,
    CurrentFreshnessDenied,
    ContextNotCurrentlyUsable,
}

impl fmt::Display for ChallengeContextError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidReference => "invalid challenge-context reference",
            Self::InvalidSubject => "invalid challenge subject",
            Self::InvalidVerificationTime => "invalid challenge-context verification time",
            Self::InvalidContextPolicy => "invalid coverage trust-context policy",
            Self::InvalidCoveragePolicy => "invalid authority coverage policy",
            Self::ContextPolicyVerificationMismatch => "context-policy verification mismatch",
            Self::CoveragePolicyVerificationMismatch => "coverage-policy verification mismatch",
            Self::CoveragePolicyMismatch => "context references a different coverage policy",
            Self::UnexpectedWitnessTrustPolicy => "witness-trust authority supplied for DirectSource mode",
            Self::MissingWitnessTrustPolicy => "WitnessQuorum requires exact witness-trust authority",
            Self::WitnessTrustAuthorityMismatch => "witness-trust authority does not match current context",
            Self::InvalidIssuerGrant => "challenge issuer grant is invalid or out of scope",
            Self::InvalidPolicyFreshnessSubject => "cannot construct exact policy freshness subject",
            Self::CurrentFreshnessDenied => "required challenge authority is not currently active",
            Self::ContextNotCurrentlyUsable => "qualified challenge context is stale or expired",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for ChallengeContextError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, AuthorityFreshnessState, VerifiedAuthorityFreshness,
        PROTOCOL_VERSION as FRESHNESS_PROTOCOL,
    };

    fn d(byte: u8) -> Digest32 { Digest32([byte; 32]) }

    #[test]
    fn new_policy_subject_kinds_are_distinct() {
        let coverage = policy_subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            "ns",
            "coverage",
            d(1),
            "policy-v1",
        ).unwrap();
        let context = policy_subject(
            AuthoritySubjectKind::CoverageTrustContextPolicy,
            "ns",
            "context",
            d(1),
            "policy-v1",
        ).unwrap();
        let witness = policy_subject(
            AuthoritySubjectKind::WitnessTrustPolicy,
            "ns",
            "witness",
            d(1),
            "policy-v1",
        ).unwrap();
        assert_ne!(coverage.identity_digest().unwrap(), context.identity_digest().unwrap());
        assert_ne!(context.identity_digest().unwrap(), witness.identity_digest().unwrap());
    }

    #[test]
    fn inactive_freshness_cannot_be_qualified() {
        let policy = policy_subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            "ns",
            "coverage",
            d(1),
            "policy-v1",
        ).unwrap();
        let receipt = VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL.into(),
                subject: policy.clone(),
                generation: 2,
                state: AuthorityFreshnessState::Revoked,
                effective_at_ms: 10,
                status_record_ref: "status:revoked".into(),
            },
            authoritative_source_ref: "source:1".into(),
            verification_ref: "verify:1".into(),
            verified_at_ms: 20,
            lease_until_ms: 40,
        };
        assert!(qualify_current_freshness(&[policy], &[receipt], 30).is_err());
    }
}
