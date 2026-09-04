// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Institution-bound trust context for authority-state coverage.
//!
//! `mycelix-authority-state-coverage` qualifies a fresh source head and optional
//! independent witness quorum. This crate closes two deliberately deferred trust
//! bindings before runtime provisioning:
//!
//! 1. the fresh challenge is bound to the exact coverage-policy identity; and
//! 2. every witness observer/trust-domain classification is bound to one exact
//!    institution-adopted witness-trust policy and exact observation identity.

use mycelix_authority_freshness::{AuthoritySubjectRef, ProfiledDigest};
use mycelix_authority_state_coverage::{
    qualify_authority_state_coverage, AuthorityCoverageError, CoverageMode,
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
    COVERAGE_IDENTITY_PROFILE, POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_source::VerifiedAuthorityStateCoverage;
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-coverage-context-v0.1";
pub const CONTEXT_POLICY_PROFILE: &str =
    "mycelix-authority-state-coverage-context-policy-v1-blake3-framed";
pub const CHALLENGE_PROFILE: &str =
    "mycelix-authority-state-coverage-challenge-v1-blake3-framed";
pub const WITNESS_TRUST_BINDING_PROFILE: &str =
    "mycelix-authority-state-witness-trust-binding-v1-blake3-framed";
pub const CONTEXT_COVERAGE_PROFILE: &str =
    "mycelix-authority-state-context-coverage-v1-blake3-framed";

const DOMAIN_CONTEXT: &[u8] = b"mycelix/authority-state/coverage-context/v1";
const DOMAIN_CHALLENGE: &[u8] = b"mycelix/authority-state/coverage-challenge/v1";
const DOMAIN_WITNESS_TRUST: &[u8] = b"mycelix/authority-state/witness-trust/v1";
const DOMAIN_CONTEXT_COVERAGE: &[u8] = b"mycelix/authority-state/context-coverage/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_TRUST_BINDINGS: usize = 64;

/// Institution-adopted context specifying which coverage policy and, when
/// witnesses are required, which witness-trust policy may qualify coverage.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoverageTrustContextPolicy {
    pub protocol_version: String,
    pub context_policy_id: String,
    pub institution_ref: String,
    pub jurisdiction_ref: Option<String>,
    pub rulebook_ref: String,
    pub coverage_policy: ProfiledDigest,
    /// Required for WitnessQuorum and forbidden for DirectSource.
    pub witness_trust_policy: Option<ProfiledDigest>,
    /// Logical verifier/trust-framework identity that owns witness classification.
    pub witness_trust_verifier_ref: Option<String>,
    pub max_challenge_lifetime_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub authority_ref: String,
    pub policy_proof_ref: String,
}

impl CoverageTrustContextPolicy {
    pub fn validate(&self) -> Result<(), CoverageContextError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoverageContextError::WrongProtocolVersion);
        }
        for value in [
            self.context_policy_id.as_str(),
            self.institution_ref.as_str(),
            self.rulebook_ref.as_str(),
            self.authority_ref.as_str(),
            self.policy_proof_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        if let Some(value) = &self.jurisdiction_ref {
            require_ref(value)?;
        }
        self.coverage_policy
            .validate()
            .map_err(|_| CoverageContextError::InvalidCoveragePolicyIdentity)?;
        if self.coverage_policy.profile != POLICY_IDENTITY_PROFILE {
            return Err(CoverageContextError::WrongCoveragePolicyProfile);
        }
        match (
            &self.witness_trust_policy,
            &self.witness_trust_verifier_ref,
        ) {
            (None, None) => {}
            (Some(policy), Some(verifier_ref)) => {
                policy
                    .validate()
                    .map_err(|_| CoverageContextError::InvalidWitnessTrustPolicy)?;
                require_profile(&policy.profile)?;
                require_ref(verifier_ref)?;
            }
            _ => return Err(CoverageContextError::IncompleteWitnessTrustContext),
        }
        if self.max_challenge_lifetime_ms == 0
            || self.valid_from_ms == 0
            || self.valid_until_ms <= self.valid_from_ms
        {
            return Err(CoverageContextError::InvalidContextLifetime);
        }
        Ok(())
    }

    pub fn active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, CoverageContextError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CONTEXT);
        frame(&mut hasher, CONTEXT_POLICY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.context_policy_id.as_bytes());
        frame(&mut hasher, self.institution_ref.as_bytes());
        frame_optional_string(&mut hasher, self.jurisdiction_ref.as_deref());
        frame(&mut hasher, self.rulebook_ref.as_bytes());
        frame_profiled_digest(&mut hasher, &self.coverage_policy);
        frame_optional_profiled_digest(&mut hasher, self.witness_trust_policy.as_ref());
        frame_optional_string(&mut hasher, self.witness_trust_verifier_ref.as_deref());
        frame(&mut hasher, &self.max_challenge_lifetime_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        frame(&mut hasher, self.authority_ref.as_bytes());
        frame(&mut hasher, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCoverageTrustContextPolicy {
    pub policy: CoverageTrustContextPolicy,
    pub policy_record_ref: String,
    pub verified_authority_ref: String,
    pub verified_policy_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Fresh unpredictable challenge scoped to one exact trust context, one exact
/// coverage policy, and one exact authority subject.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoverageChallenge {
    pub protocol_version: String,
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
    pub coverage_policy_digest: Digest32,
    pub coverage_policy_profile: String,
    pub subject: AuthoritySubjectRef,
    /// Commitment to runtime-generated unpredictable random material.
    pub nonce_digest: Digest32,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    pub challenge_issuer_ref: String,
}

impl CoverageChallenge {
    pub fn validate(&self) -> Result<(), CoverageContextError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoverageContextError::WrongProtocolVersion);
        }
        if self.context_policy_digest.is_zero()
            || self.coverage_policy_digest.is_zero()
            || self.nonce_digest.is_zero()
        {
            return Err(CoverageContextError::ZeroDigest);
        }
        require_profile(&self.context_policy_profile)?;
        require_profile(&self.coverage_policy_profile)?;
        if self.context_policy_profile != CONTEXT_POLICY_PROFILE
            || self.coverage_policy_profile != POLICY_IDENTITY_PROFILE
        {
            return Err(CoverageContextError::WrongChallengeProfile);
        }
        self.subject
            .validate()
            .map_err(|_| CoverageContextError::InvalidSubject)?;
        require_ref(&self.challenge_issuer_ref)?;
        if self.issued_at_ms == 0 || self.expires_at_ms <= self.issued_at_ms {
            return Err(CoverageContextError::InvalidChallengeLifetime);
        }
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, CoverageContextError> {
        self.validate()?;
        let subject_digest = self
            .subject
            .identity_digest()
            .map_err(|_| CoverageContextError::InvalidSubject)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_CHALLENGE);
        frame(&mut hasher, CHALLENGE_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, &self.context_policy_digest.0);
        frame(&mut hasher, self.context_policy_profile.as_bytes());
        frame(&mut hasher, &self.coverage_policy_digest.0);
        frame(&mut hasher, self.coverage_policy_profile.as_bytes());
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, &self.nonce_digest.0);
        frame(&mut hasher, &self.issued_at_ms.to_le_bytes());
        frame(&mut hasher, &self.expires_at_ms.to_le_bytes());
        frame(&mut hasher, self.challenge_issuer_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCoverageChallenge {
    pub challenge: CoverageChallenge,
    /// Proof that nonce material came from the runtime-approved freshness source.
    pub verified_randomness_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Exact binding between one witness observation and one institution-adopted
/// witness-trust policy/classification decision.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WitnessTrustBinding {
    pub protocol_version: String,
    pub context_policy_digest: Digest32,
    pub context_policy_profile: String,
    pub witness_trust_policy: ProfiledDigest,
    pub challenge_digest: Digest32,
    pub witness_observation_digest: Digest32,
    pub observer_id: String,
    pub trust_domain_id: String,
    pub trust_domain_ref: String,
    pub classification_proof_ref: String,
}

impl WitnessTrustBinding {
    pub fn validate(&self) -> Result<(), CoverageContextError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(CoverageContextError::WrongProtocolVersion);
        }
        if self.context_policy_digest.is_zero()
            || self.challenge_digest.is_zero()
            || self.witness_observation_digest.is_zero()
        {
            return Err(CoverageContextError::ZeroDigest);
        }
        if self.context_policy_profile != CONTEXT_POLICY_PROFILE {
            return Err(CoverageContextError::WrongContextPolicyProfile);
        }
        self.witness_trust_policy
            .validate()
            .map_err(|_| CoverageContextError::InvalidWitnessTrustPolicy)?;
        require_profile(&self.witness_trust_policy.profile)?;
        for value in [
            self.observer_id.as_str(),
            self.trust_domain_id.as_str(),
            self.trust_domain_ref.as_str(),
            self.classification_proof_ref.as_str(),
        ] {
            require_ref(value)?;
        }
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, CoverageContextError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_WITNESS_TRUST);
        frame(&mut hasher, WITNESS_TRUST_BINDING_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, &self.context_policy_digest.0);
        frame(&mut hasher, self.context_policy_profile.as_bytes());
        frame_profiled_digest(&mut hasher, &self.witness_trust_policy);
        frame(&mut hasher, &self.challenge_digest.0);
        frame(&mut hasher, &self.witness_observation_digest.0);
        frame(&mut hasher, self.observer_id.as_bytes());
        frame(&mut hasher, self.trust_domain_id.as_bytes());
        frame(&mut hasher, self.trust_domain_ref.as_bytes());
        frame(&mut hasher, self.classification_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedWitnessTrustBinding {
    pub binding: WitnessTrustBinding,
    pub verified_classification_proof_ref: String,
    pub verified_trust_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable result. Only the qualifier can construct current context-
/// bound coverage from the exact current evidence set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedContextBoundCoverage {
    state_source_coverage: VerifiedAuthorityStateCoverage,
    context_policy_digest: Digest32,
    context_policy_profile: String,
    challenge_digest: Digest32,
    challenge_profile: String,
    base_coverage_digest: Digest32,
    base_coverage_profile: String,
    trust_binding_count: u16,
    context_coverage_digest: Digest32,
    context_coverage_profile: String,
}

impl QualifiedContextBoundCoverage {
    pub fn state_source_coverage(&self) -> &VerifiedAuthorityStateCoverage {
        &self.state_source_coverage
    }

    pub fn context_policy_digest(&self) -> Digest32 {
        self.context_policy_digest
    }

    pub fn challenge_digest(&self) -> Digest32 {
        self.challenge_digest
    }

    pub fn context_coverage_digest(&self) -> Digest32 {
        self.context_coverage_digest
    }

    pub fn context_coverage_profile(&self) -> &str {
        &self.context_coverage_profile
    }

    pub fn trust_binding_count(&self) -> u16 {
        self.trust_binding_count
    }

    pub fn to_state_source_coverage(&self) -> VerifiedAuthorityStateCoverage {
        self.state_source_coverage.clone()
    }
}

#[allow(clippy::too_many_arguments)]
pub fn qualify_context_bound_coverage(
    context_receipt: &VerifiedCoverageTrustContextPolicy,
    challenge_receipt: &VerifiedCoverageChallenge,
    coverage_policy_receipt: &VerifiedAuthorityCoveragePolicy,
    source_receipt: &VerifiedAuthoritySourceHead,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    trust_bindings: &[VerifiedWitnessTrustBinding],
    now_ms: u64,
) -> Result<QualifiedContextBoundCoverage, CoverageContextError> {
    if now_ms == 0 {
        return Err(CoverageContextError::InvalidVerificationTime);
    }

    let context = verify_context_policy(context_receipt, now_ms)?;
    let context_digest = context.identity_digest()?;
    let coverage_policy = &coverage_policy_receipt.policy;
    coverage_policy
        .validate()
        .map_err(CoverageContextError::BaseCoverage)?;
    let coverage_policy_digest = coverage_policy
        .identity_digest()
        .map_err(CoverageContextError::BaseCoverage)?;
    if context.coverage_policy.digest != coverage_policy_digest
        || context.coverage_policy.profile != POLICY_IDENTITY_PROFILE
    {
        return Err(CoverageContextError::CoveragePolicyMismatch);
    }

    match (&coverage_policy.mode, &context.witness_trust_policy) {
        (CoverageMode::DirectSource, None) => {
            if !trust_bindings.is_empty() {
                return Err(CoverageContextError::UnexpectedTrustBindings);
            }
        }
        (CoverageMode::DirectSource, Some(_)) => {
            return Err(CoverageContextError::UnexpectedWitnessTrustPolicy);
        }
        (CoverageMode::WitnessQuorum { .. }, Some(_)) => {}
        (CoverageMode::WitnessQuorum { .. }, None) => {
            return Err(CoverageContextError::MissingWitnessTrustPolicy);
        }
    }

    let challenge = verify_challenge(
        challenge_receipt,
        context,
        context_digest,
        coverage_policy_digest,
        now_ms,
    )?;
    let challenge_digest = challenge.identity_digest()?;

    if source_receipt.attestation.subject != challenge.subject {
        return Err(CoverageContextError::ChallengeSubjectMismatch);
    }

    let trust_binding_digests = verify_trust_bindings(
        context,
        context_digest,
        challenge_digest,
        witness_receipts,
        trust_bindings,
        now_ms,
    )?;

    let base = qualify_authority_state_coverage(
        coverage_policy_receipt,
        source_receipt,
        witness_receipts,
        challenge_digest,
        now_ms,
    )
    .map_err(CoverageContextError::BaseCoverage)?;

    let mut verified_at_ms = base.verified_at_ms().max(context_receipt.verified_at_ms);
    verified_at_ms = verified_at_ms.max(challenge_receipt.verified_at_ms);
    let mut lease_until_ms = base
        .lease_until_ms()
        .min(context.valid_until_ms)
        .min(challenge.expires_at_ms);
    for receipt in trust_bindings {
        verified_at_ms = verified_at_ms.max(receipt.verified_at_ms);
        lease_until_ms = lease_until_ms.min(receipt.valid_until_ms);
    }
    if verified_at_ms > now_ms || lease_until_ms <= now_ms {
        return Err(CoverageContextError::ContextCoverageExpired);
    }

    let context_coverage_digest = context_coverage_digest(
        context_digest,
        challenge_digest,
        base.coverage_digest(),
        &trust_binding_digests,
    );

    let mut state_source_coverage = base.to_state_source_coverage();
    state_source_coverage.coverage_proof_ref = context_coverage_ref(context_coverage_digest);
    state_source_coverage.verification_ref =
        context_coverage_verification_ref(context_coverage_digest);
    state_source_coverage.verified_at_ms = verified_at_ms;
    state_source_coverage.lease_until_ms = lease_until_ms;
    state_source_coverage
        .validate_at(now_ms)
        .map_err(|_| CoverageContextError::InvalidProjectedCoverage)?;

    Ok(QualifiedContextBoundCoverage {
        state_source_coverage,
        context_policy_digest: context_digest,
        context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
        challenge_digest,
        challenge_profile: CHALLENGE_PROFILE.into(),
        base_coverage_digest: base.coverage_digest(),
        base_coverage_profile: COVERAGE_IDENTITY_PROFILE.into(),
        trust_binding_count: trust_binding_digests.len() as u16,
        context_coverage_digest,
        context_coverage_profile: CONTEXT_COVERAGE_PROFILE.into(),
    })
}

fn verify_context_policy<'a>(
    receipt: &'a VerifiedCoverageTrustContextPolicy,
    now_ms: u64,
) -> Result<&'a CoverageTrustContextPolicy, CoverageContextError> {
    let policy = &receipt.policy;
    policy.validate()?;
    if !policy.active_at(now_ms) {
        return Err(CoverageContextError::ContextPolicyInactive);
    }
    for value in [
        receipt.policy_record_ref.as_str(),
        receipt.verified_authority_ref.as_str(),
        receipt.verified_policy_proof_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if receipt.verified_authority_ref != policy.authority_ref {
        return Err(CoverageContextError::ContextAuthorityMismatch);
    }
    if receipt.verified_policy_proof_ref != policy.policy_proof_ref {
        return Err(CoverageContextError::ContextProofMismatch);
    }
    if receipt.verified_at_ms == 0 || receipt.verified_at_ms > now_ms {
        return Err(CoverageContextError::InvalidVerificationTime);
    }
    Ok(policy)
}

fn verify_challenge<'a>(
    receipt: &'a VerifiedCoverageChallenge,
    context: &CoverageTrustContextPolicy,
    context_digest: Digest32,
    coverage_policy_digest: Digest32,
    now_ms: u64,
) -> Result<&'a CoverageChallenge, CoverageContextError> {
    let challenge = &receipt.challenge;
    challenge.validate()?;
    require_ref(&receipt.verified_randomness_ref)?;
    require_ref(&receipt.verification_ref)?;
    if challenge.context_policy_digest != context_digest
        || challenge.context_policy_profile != CONTEXT_POLICY_PROFILE
        || challenge.coverage_policy_digest != coverage_policy_digest
        || challenge.coverage_policy_profile != POLICY_IDENTITY_PROFILE
    {
        return Err(CoverageContextError::ChallengeContextMismatch);
    }
    if challenge.issued_at_ms < context.valid_from_ms
        || challenge.issued_at_ms >= context.valid_until_ms
        || challenge.expires_at_ms > context.valid_until_ms
        || challenge
            .expires_at_ms
            .saturating_sub(challenge.issued_at_ms)
            > context.max_challenge_lifetime_ms
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < challenge.issued_at_ms
        || now_ms >= challenge.expires_at_ms
    {
        return Err(CoverageContextError::InvalidChallengeWindow);
    }
    Ok(challenge)
}

fn verify_trust_bindings(
    context: &CoverageTrustContextPolicy,
    context_digest: Digest32,
    challenge_digest: Digest32,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    trust_bindings: &[VerifiedWitnessTrustBinding],
    now_ms: u64,
) -> Result<Vec<Digest32>, CoverageContextError> {
    if witness_receipts.len() > MAX_TRUST_BINDINGS || trust_bindings.len() > MAX_TRUST_BINDINGS {
        return Err(CoverageContextError::TooManyTrustBindings);
    }

    let Some(expected_trust_policy) = context.witness_trust_policy.as_ref() else {
        if witness_receipts.is_empty() && trust_bindings.is_empty() {
            return Ok(Vec::new());
        }
        return Err(CoverageContextError::UnexpectedTrustBindings);
    };
    let expected_verifier = context
        .witness_trust_verifier_ref
        .as_ref()
        .ok_or(CoverageContextError::IncompleteWitnessTrustContext)?;

    let mut witnesses_by_observer = BTreeMap::new();
    for witness in witness_receipts {
        let observer_id = witness.observation.observer_id.clone();
        if witnesses_by_observer.insert(observer_id, witness).is_some() {
            return Err(CoverageContextError::DuplicateWitnessObserver);
        }
    }

    let mut bindings_by_observer = BTreeMap::new();
    for receipt in trust_bindings {
        let binding = &receipt.binding;
        binding.validate()?;
        if binding.context_policy_digest != context_digest
            || binding.context_policy_profile != CONTEXT_POLICY_PROFILE
            || &binding.witness_trust_policy != expected_trust_policy
            || binding.challenge_digest != challenge_digest
        {
            return Err(CoverageContextError::TrustBindingContextMismatch);
        }
        if receipt.verified_trust_verifier_ref != *expected_verifier {
            return Err(CoverageContextError::TrustVerifierMismatch);
        }
        if receipt.verified_classification_proof_ref != binding.classification_proof_ref {
            return Err(CoverageContextError::TrustClassificationProofMismatch);
        }
        require_ref(&receipt.verification_ref)?;
        if receipt.verified_at_ms == 0
            || receipt.verified_at_ms > now_ms
            || receipt.valid_until_ms <= now_ms
        {
            return Err(CoverageContextError::TrustBindingExpired);
        }
        if bindings_by_observer
            .insert(binding.observer_id.clone(), receipt)
            .is_some()
        {
            return Err(CoverageContextError::DuplicateTrustBinding);
        }
    }

    if witnesses_by_observer.len() != bindings_by_observer.len() {
        return Err(CoverageContextError::TrustBindingSetMismatch);
    }

    let mut digests = Vec::with_capacity(witnesses_by_observer.len());
    let mut seen_domains = BTreeSet::new();
    for (observer_id, witness) in witnesses_by_observer {
        let receipt = bindings_by_observer
            .get(&observer_id)
            .ok_or(CoverageContextError::MissingTrustBinding)?;
        let binding = &receipt.binding;
        let observation_digest = witness
            .observation
            .identity_digest()
            .map_err(CoverageContextError::BaseCoverage)?;
        if binding.witness_observation_digest != observation_digest
            || binding.observer_id != witness.observation.observer_id
            || binding.trust_domain_id != witness.observation.trust_domain_id
            || binding.trust_domain_ref != witness.verified_trust_domain_ref
        {
            return Err(CoverageContextError::TrustBindingObservationMismatch);
        }
        // A domain may naturally classify several observers. This set exists only
        // to make the classification inputs explicit; #94 enforces concentration.
        seen_domains.insert(binding.trust_domain_id.clone());
        digests.push(binding.identity_digest()?);
    }
    let _ = seen_domains;
    Ok(digests)
}

fn context_coverage_digest(
    context_policy_digest: Digest32,
    challenge_digest: Digest32,
    base_coverage_digest: Digest32,
    trust_binding_digests: &[Digest32],
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CONTEXT_COVERAGE);
    frame(&mut hasher, CONTEXT_COVERAGE_PROFILE.as_bytes());
    frame(&mut hasher, &context_policy_digest.0);
    frame(&mut hasher, &challenge_digest.0);
    frame(&mut hasher, &base_coverage_digest.0);
    frame(&mut hasher, &(trust_binding_digests.len() as u64).to_le_bytes());
    for digest in trust_binding_digests {
        frame(&mut hasher, &digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn context_coverage_ref(digest: Digest32) -> String {
    format!(
        "coverage-context:{}:{}",
        CONTEXT_COVERAGE_PROFILE,
        hex_digest(digest)
    )
}

fn context_coverage_verification_ref(digest: Digest32) -> String {
    format!("coverage-context-verification:{}", hex_digest(digest))
}

fn require_ref(value: &str) -> Result<(), CoverageContextError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(CoverageContextError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), CoverageContextError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(CoverageContextError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn frame_optional_string(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        None => frame(hasher, &[0]),
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, value.as_bytes());
        }
    }
}

fn frame_profiled_digest(hasher: &mut blake3::Hasher, value: &ProfiledDigest) {
    frame(hasher, value.profile.as_bytes());
    frame(hasher, &value.digest.0);
}

fn frame_optional_profiled_digest(hasher: &mut blake3::Hasher, value: Option<&ProfiledDigest>) {
    match value {
        None => frame(hasher, &[0]),
        Some(value) => {
            frame(hasher, &[1]);
            frame_profiled_digest(hasher, value);
        }
    }
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

#[derive(Debug, PartialEq, Eq)]
pub enum CoverageContextError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidProfile,
    ZeroDigest,
    InvalidSubject,
    InvalidCoveragePolicyIdentity,
    WrongCoveragePolicyProfile,
    InvalidWitnessTrustPolicy,
    IncompleteWitnessTrustContext,
    InvalidContextLifetime,
    WrongChallengeProfile,
    InvalidChallengeLifetime,
    WrongContextPolicyProfile,
    InvalidVerificationTime,
    ContextPolicyInactive,
    ContextAuthorityMismatch,
    ContextProofMismatch,
    CoveragePolicyMismatch,
    UnexpectedTrustBindings,
    UnexpectedWitnessTrustPolicy,
    MissingWitnessTrustPolicy,
    ChallengeContextMismatch,
    ChallengeSubjectMismatch,
    InvalidChallengeWindow,
    TooManyTrustBindings,
    DuplicateWitnessObserver,
    DuplicateTrustBinding,
    TrustBindingContextMismatch,
    TrustVerifierMismatch,
    TrustClassificationProofMismatch,
    TrustBindingExpired,
    TrustBindingSetMismatch,
    MissingTrustBinding,
    TrustBindingObservationMismatch,
    ContextCoverageExpired,
    InvalidProjectedCoverage,
    BaseCoverage(AuthorityCoverageError),
}

impl fmt::Display for CoverageContextError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong authority coverage-context protocol"),
            Self::InvalidReference => write!(f, "invalid authority coverage-context reference"),
            Self::InvalidProfile => write!(f, "invalid authority coverage-context profile"),
            Self::ZeroDigest => write!(f, "authority coverage-context digest must not be zero"),
            Self::InvalidSubject => write!(f, "invalid authority coverage subject"),
            Self::InvalidCoveragePolicyIdentity => write!(f, "invalid coverage-policy identity"),
            Self::WrongCoveragePolicyProfile => write!(f, "wrong coverage-policy profile"),
            Self::InvalidWitnessTrustPolicy => write!(f, "invalid witness-trust policy identity"),
            Self::IncompleteWitnessTrustContext => write!(f, "incomplete witness-trust context"),
            Self::InvalidContextLifetime => write!(f, "invalid coverage-context lifetime"),
            Self::WrongChallengeProfile => write!(f, "wrong coverage challenge profile"),
            Self::InvalidChallengeLifetime => write!(f, "invalid coverage challenge lifetime"),
            Self::WrongContextPolicyProfile => write!(f, "wrong context-policy profile"),
            Self::InvalidVerificationTime => write!(f, "invalid coverage-context verification time"),
            Self::ContextPolicyInactive => write!(f, "coverage trust-context policy is inactive"),
            Self::ContextAuthorityMismatch => write!(f, "coverage trust-context authority mismatch"),
            Self::ContextProofMismatch => write!(f, "coverage trust-context proof mismatch"),
            Self::CoveragePolicyMismatch => write!(f, "coverage policy does not match trust context"),
            Self::UnexpectedTrustBindings => write!(f, "unexpected witness trust bindings"),
            Self::UnexpectedWitnessTrustPolicy => write!(f, "DirectSource forbids witness trust policy"),
            Self::MissingWitnessTrustPolicy => write!(f, "WitnessQuorum requires witness trust policy"),
            Self::ChallengeContextMismatch => write!(f, "challenge is bound to another context/policy"),
            Self::ChallengeSubjectMismatch => write!(f, "challenge subject does not match source head"),
            Self::InvalidChallengeWindow => write!(f, "challenge is stale, future, or out of policy bounds"),
            Self::TooManyTrustBindings => write!(f, "witness trust-binding fan-in exceeds v0.1 bound"),
            Self::DuplicateWitnessObserver => write!(f, "duplicate witness observer in context input"),
            Self::DuplicateTrustBinding => write!(f, "duplicate trust binding for observer"),
            Self::TrustBindingContextMismatch => write!(f, "witness trust binding belongs to another context"),
            Self::TrustVerifierMismatch => write!(f, "witness trust verifier does not match adopted context"),
            Self::TrustClassificationProofMismatch => write!(f, "witness trust classification proof mismatch"),
            Self::TrustBindingExpired => write!(f, "witness trust binding is stale or expired"),
            Self::TrustBindingSetMismatch => write!(f, "witness and trust-binding sets differ"),
            Self::MissingTrustBinding => write!(f, "witness is missing an exact trust binding"),
            Self::TrustBindingObservationMismatch => write!(f, "trust binding does not match exact witness observation"),
            Self::ContextCoverageExpired => write!(f, "context-bound coverage lease is expired"),
            Self::InvalidProjectedCoverage => write!(f, "context-bound state-source coverage is invalid"),
            Self::BaseCoverage(error) => write!(f, "base coverage qualification failed: {error}"),
        }
    }
}

impl std::error::Error for CoverageContextError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{AuthoritySubjectKind, ProfiledDigest};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test".into(),
            subject_id: "grant:test".into(),
            identity: ProfiledDigest {
                digest: d(1),
                profile: "grant-v1-blake3".into(),
            },
        }
    }

    fn context(trust: Option<ProfiledDigest>) -> CoverageTrustContextPolicy {
        CoverageTrustContextPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            context_policy_id: "coverage-context:test".into(),
            institution_ref: "institution:test".into(),
            jurisdiction_ref: None,
            rulebook_ref: "rulebook:test@1".into(),
            coverage_policy: ProfiledDigest {
                digest: d(2),
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
            witness_trust_verifier_ref: trust.as_ref().map(|_| "trust-verifier:test".into()),
            witness_trust_policy: trust,
            max_challenge_lifetime_ms: 500,
            valid_from_ms: 100,
            valid_until_ms: 2_000,
            authority_ref: "authority:test".into(),
            policy_proof_ref: "proof:test".into(),
        }
    }

    #[test]
    fn challenge_identity_changes_with_context_policy() {
        let a = context(None);
        let mut b = a.clone();
        b.rulebook_ref = "rulebook:test@2".into();
        let challenge_a = CoverageChallenge {
            protocol_version: PROTOCOL_VERSION.into(),
            context_policy_digest: a.identity_digest().unwrap(),
            context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
            coverage_policy_digest: a.coverage_policy.digest,
            coverage_policy_profile: POLICY_IDENTITY_PROFILE.into(),
            subject: subject(),
            nonce_digest: d(9),
            issued_at_ms: 500,
            expires_at_ms: 800,
            challenge_issuer_ref: "challenge-issuer:test".into(),
        };
        let mut challenge_b = challenge_a.clone();
        challenge_b.context_policy_digest = b.identity_digest().unwrap();
        assert_ne!(
            challenge_a.identity_digest().unwrap(),
            challenge_b.identity_digest().unwrap()
        );
    }

    #[test]
    fn trust_policy_changes_context_identity() {
        let a = context(Some(ProfiledDigest {
            digest: d(7),
            profile: "witness-trust-v1".into(),
        }));
        let b = context(Some(ProfiledDigest {
            digest: d(8),
            profile: "witness-trust-v1".into(),
        }));
        assert_ne!(a.identity_digest().unwrap(), b.identity_digest().unwrap());
    }

    #[test]
    fn observation_rebinding_changes_trust_binding_identity() {
        let trust = ProfiledDigest {
            digest: d(7),
            profile: "witness-trust-v1".into(),
        };
        let context = context(Some(trust.clone()));
        let context_digest = context.identity_digest().unwrap();
        let a = WitnessTrustBinding {
            protocol_version: PROTOCOL_VERSION.into(),
            context_policy_digest: context_digest,
            context_policy_profile: CONTEXT_POLICY_PROFILE.into(),
            witness_trust_policy: trust,
            challenge_digest: d(10),
            witness_observation_digest: d(11),
            observer_id: "observer:a".into(),
            trust_domain_id: "domain:a".into(),
            trust_domain_ref: "domain-proof:a".into(),
            classification_proof_ref: "classification:a".into(),
        };
        let mut b = a.clone();
        b.witness_observation_digest = d(12);
        assert_ne!(a.identity_digest().unwrap(), b.identity_digest().unwrap());
    }
}
