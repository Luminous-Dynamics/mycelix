// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fresh authoritative-head coverage qualification.
//!
//! A complete transition prefix is not proof that no later revocation exists.
//! This crate qualifies one fresh challenge-bound head response from the exact
//! configured logical authority source and, when required by institutional
//! policy, an independently verified witness quorum over that exact same head.
//!
//! It does not persist state, generate randomness, verify signatures itself, or
//! decide institutional policy. Those facts must be independently verified by
//! the host before entering this pure boundary.

use mycelix_authority_freshness::{
    AuthoritySubjectKind, AuthoritySubjectRef, ProfiledDigest as FreshnessProfiledDigest,
};
use mycelix_authority_state_source::VerifiedAuthorityStateCoverage;
use mycelix_institutional_core::Digest32;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-state-coverage-v0.1";
pub const POLICY_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-coverage-policy-v1-blake3-framed";
pub const SOURCE_HEAD_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-source-head-v1-blake3-framed";
pub const WITNESS_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-head-witness-v1-blake3-framed";
pub const COVERAGE_IDENTITY_PROFILE: &str =
    "mycelix-authority-state-coverage-v1-blake3-framed";

const DOMAIN_POLICY: &[u8] = b"mycelix/authority-state/coverage-policy/v1";
const DOMAIN_SOURCE_HEAD: &[u8] = b"mycelix/authority-state/source-head/v1";
const DOMAIN_WITNESS: &[u8] = b"mycelix/authority-state/head-witness/v1";
const DOMAIN_COVERAGE: &[u8] = b"mycelix/authority-state/coverage/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_WITNESSES: usize = 64;
const MAX_TRUST_DOMAINS: usize = 32;
const MAX_ALLOWED_SUBJECT_KINDS: usize = 16;
const MAX_HEAD_GENERATION: u64 = 256;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CoverageMode {
    /// The configured logical source's fresh signed/challenge-bound head is the
    /// complete authority for coverage. This optimizes liveness but concentrates
    /// source-compromise risk.
    DirectSource,
    /// The source head must additionally be observed by independently verified
    /// witnesses. Distinct observer IDs alone are not independence.
    WitnessQuorum {
        min_witnesses: u16,
        min_trust_domains: u16,
        max_per_trust_domain: u16,
    },
}

impl CoverageMode {
    fn validate(&self) -> Result<(), AuthorityCoverageError> {
        match self {
            Self::DirectSource => Ok(()),
            Self::WitnessQuorum {
                min_witnesses,
                min_trust_domains,
                max_per_trust_domain,
            } => {
                if *min_witnesses == 0
                    || *min_witnesses as usize > MAX_WITNESSES
                    || *min_trust_domains == 0
                    || *min_trust_domains as usize > MAX_TRUST_DOMAINS
                    || *max_per_trust_domain == 0
                    || *min_trust_domains > *min_witnesses
                {
                    return Err(AuthorityCoverageError::InvalidWitnessPolicy);
                }
                let maximum_witnesses =
                    (MAX_TRUST_DOMAINS as u32) * u32::from(*max_per_trust_domain);
                if u32::from(*min_witnesses) > maximum_witnesses {
                    return Err(AuthorityCoverageError::ImpossibleWitnessPolicy);
                }
                Ok(())
            }
        }
    }

    fn code(&self) -> u8 {
        match self {
            Self::DirectSource => 1,
            Self::WitnessQuorum { .. } => 2,
        }
    }
}

/// Institution-adopted policy describing how one logical authority-state source
/// may establish coverage for subjects in one exact namespace.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityCoveragePolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub namespace: String,
    pub allowed_subject_kinds: Vec<AuthoritySubjectKind>,
    pub authoritative_source_ref: String,
    /// Exact verification identity/key/profile of the configured source.
    pub source_identity: FreshnessProfiledDigest,
    pub mode: CoverageMode,
    /// Maximum age of the source response at qualification time.
    pub max_source_age_ms: u64,
    /// Maximum age of an accepted witness observation at qualification time.
    pub max_witness_age_ms: u64,
    /// Maximum reusable lease issued by this pure qualifier.
    pub max_coverage_lease_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Institutional/rulebook authority that adopted this exact coverage policy.
    pub authority_ref: String,
    pub policy_proof_ref: String,
}

impl AuthorityCoveragePolicy {
    pub fn validate(&self) -> Result<(), AuthorityCoverageError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AuthorityCoverageError::WrongProtocolVersion);
        }
        require_ref(&self.policy_id)?;
        require_ref(&self.namespace)?;
        require_ref(&self.authoritative_source_ref)?;
        require_ref(&self.authority_ref)?;
        require_ref(&self.policy_proof_ref)?;
        self.source_identity
            .validate()
            .map_err(|_| AuthorityCoverageError::InvalidSourceIdentity)?;
        require_profile(&self.source_identity.profile)?;
        self.mode.validate()?;
        if self.allowed_subject_kinds.is_empty()
            || self.allowed_subject_kinds.len() > MAX_ALLOWED_SUBJECT_KINDS
        {
            return Err(AuthorityCoverageError::InvalidAllowedSubjectKinds);
        }
        canonical_subject_kind_codes(&self.allowed_subject_kinds)?;
        if self.max_source_age_ms == 0
            || self.max_coverage_lease_ms == 0
            || matches!(self.mode, CoverageMode::WitnessQuorum { .. })
                && self.max_witness_age_ms == 0
        {
            return Err(AuthorityCoverageError::InvalidCoverageWindow);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(AuthorityCoverageError::InvalidPolicyLifetime);
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, AuthorityCoverageError> {
        self.validate()?;
        let subject_codes = canonical_subject_kind_codes(&self.allowed_subject_kinds)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_POLICY);
        frame(&mut hasher, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.policy_id.as_bytes());
        frame(&mut hasher, self.namespace.as_bytes());
        frame(&mut hasher, &(subject_codes.len() as u64).to_le_bytes());
        for code in subject_codes {
            frame(&mut hasher, &[code]);
        }
        frame(&mut hasher, self.authoritative_source_ref.as_bytes());
        frame(&mut hasher, self.source_identity.profile.as_bytes());
        frame(&mut hasher, &self.source_identity.digest.0);
        frame(&mut hasher, &[self.mode.code()]);
        match &self.mode {
            CoverageMode::DirectSource => {}
            CoverageMode::WitnessQuorum {
                min_witnesses,
                min_trust_domains,
                max_per_trust_domain,
            } => {
                frame(&mut hasher, &min_witnesses.to_le_bytes());
                frame(&mut hasher, &min_trust_domains.to_le_bytes());
                frame(&mut hasher, &max_per_trust_domain.to_le_bytes());
            }
        }
        frame(&mut hasher, &self.max_source_age_ms.to_le_bytes());
        frame(&mut hasher, &self.max_witness_age_ms.to_le_bytes());
        frame(&mut hasher, &self.max_coverage_lease_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        frame(&mut hasher, self.authority_ref.as_bytes());
        frame(&mut hasher, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// Host verification of the exact institution-adopted coverage policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityCoveragePolicy {
    pub policy: AuthorityCoveragePolicy,
    pub policy_record_ref: String,
    pub verified_policy_proof_ref: String,
    pub verified_authority_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Fresh challenge-bound head statement from the configured logical source.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthoritySourceHeadAttestation {
    pub protocol_version: String,
    pub subject: AuthoritySubjectRef,
    pub authoritative_source_ref: String,
    pub source_identity: FreshnessProfiledDigest,
    pub head_generation: u64,
    pub head_transition_digest: Digest32,
    pub head_status_record_ref: String,
    /// Head of the source's own append-only publication transcript/source chain.
    pub source_chain_head_ref: String,
    /// Must come from a fresh unpredictable verifier challenge in runtime.
    pub challenge_digest: Digest32,
    pub responded_at_ms: u64,
    pub expires_at_ms: u64,
    pub source_proof_ref: String,
}

impl AuthoritySourceHeadAttestation {
    pub fn validate(&self) -> Result<(), AuthorityCoverageError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AuthorityCoverageError::WrongProtocolVersion);
        }
        self.subject
            .validate()
            .map_err(|_| AuthorityCoverageError::InvalidSubject)?;
        require_ref(&self.authoritative_source_ref)?;
        self.source_identity
            .validate()
            .map_err(|_| AuthorityCoverageError::InvalidSourceIdentity)?;
        require_profile(&self.source_identity.profile)?;
        if self.head_generation == 0
            || self.head_generation > MAX_HEAD_GENERATION
            || self.head_transition_digest.is_zero()
        {
            return Err(AuthorityCoverageError::InvalidHeadIdentity);
        }
        require_ref(&self.head_status_record_ref)?;
        require_ref(&self.source_chain_head_ref)?;
        if self.challenge_digest.is_zero() {
            return Err(AuthorityCoverageError::ZeroChallenge);
        }
        if self.responded_at_ms == 0 || self.expires_at_ms <= self.responded_at_ms {
            return Err(AuthorityCoverageError::InvalidSourceAttestationLifetime);
        }
        require_ref(&self.source_proof_ref)
    }

    pub fn identity_digest(&self) -> Result<Digest32, AuthorityCoverageError> {
        self.validate()?;
        let subject_digest = self
            .subject
            .identity_digest()
            .map_err(|_| AuthorityCoverageError::InvalidSubject)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_SOURCE_HEAD);
        frame(&mut hasher, SOURCE_HEAD_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, self.authoritative_source_ref.as_bytes());
        frame(&mut hasher, self.source_identity.profile.as_bytes());
        frame(&mut hasher, &self.source_identity.digest.0);
        frame(&mut hasher, &self.head_generation.to_le_bytes());
        frame(&mut hasher, &self.head_transition_digest.0);
        frame(&mut hasher, self.head_status_record_ref.as_bytes());
        frame(&mut hasher, self.source_chain_head_ref.as_bytes());
        frame(&mut hasher, &self.challenge_digest.0);
        frame(&mut hasher, &self.responded_at_ms.to_le_bytes());
        frame(&mut hasher, &self.expires_at_ms.to_le_bytes());
        frame(&mut hasher, self.source_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthoritySourceHead {
    pub attestation: AuthoritySourceHeadAttestation,
    pub verified_source_identity: FreshnessProfiledDigest,
    pub verified_source_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// One independently observed copy of the exact source head attestation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityHeadWitnessObservation {
    pub protocol_version: String,
    pub subject: AuthoritySubjectRef,
    pub authoritative_source_ref: String,
    pub source_head_digest: Digest32,
    pub head_generation: u64,
    pub head_transition_digest: Digest32,
    pub head_status_record_ref: String,
    pub challenge_digest: Digest32,
    pub observer_id: String,
    /// Independence class verified by the host; not caller self-description.
    pub trust_domain_id: String,
    pub observed_at_ms: u64,
    pub expires_at_ms: u64,
    pub observation_proof_ref: String,
}

impl AuthorityHeadWitnessObservation {
    pub fn validate(&self) -> Result<(), AuthorityCoverageError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(AuthorityCoverageError::WrongProtocolVersion);
        }
        self.subject
            .validate()
            .map_err(|_| AuthorityCoverageError::InvalidSubject)?;
        require_ref(&self.authoritative_source_ref)?;
        if self.source_head_digest.is_zero()
            || self.head_transition_digest.is_zero()
            || self.challenge_digest.is_zero()
            || self.head_generation == 0
            || self.head_generation > MAX_HEAD_GENERATION
        {
            return Err(AuthorityCoverageError::InvalidWitnessHeadIdentity);
        }
        require_ref(&self.head_status_record_ref)?;
        require_ref(&self.observer_id)?;
        require_ref(&self.trust_domain_id)?;
        require_ref(&self.observation_proof_ref)?;
        if self.observed_at_ms == 0 || self.expires_at_ms <= self.observed_at_ms {
            return Err(AuthorityCoverageError::InvalidWitnessLifetime);
        }
        Ok(())
    }

    pub fn identity_digest(&self) -> Result<Digest32, AuthorityCoverageError> {
        self.validate()?;
        let subject_digest = self
            .subject
            .identity_digest()
            .map_err(|_| AuthorityCoverageError::InvalidSubject)?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_WITNESS);
        frame(&mut hasher, WITNESS_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, &subject_digest.0);
        frame(&mut hasher, self.authoritative_source_ref.as_bytes());
        frame(&mut hasher, &self.source_head_digest.0);
        frame(&mut hasher, &self.head_generation.to_le_bytes());
        frame(&mut hasher, &self.head_transition_digest.0);
        frame(&mut hasher, self.head_status_record_ref.as_bytes());
        frame(&mut hasher, &self.challenge_digest.0);
        frame(&mut hasher, self.observer_id.as_bytes());
        frame(&mut hasher, self.trust_domain_id.as_bytes());
        frame(&mut hasher, &self.observed_at_ms.to_le_bytes());
        frame(&mut hasher, &self.expires_at_ms.to_le_bytes());
        frame(&mut hasher, self.observation_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedAuthorityHeadWitness {
    pub observation: AuthorityHeadWitnessObservation,
    pub verified_observation_proof_ref: String,
    pub verified_observer_id: String,
    pub verified_trust_domain_id: String,
    pub verified_trust_domain_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Non-forgeable result of exact current source-head coverage qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedAuthorityStateCoverage {
    subject: AuthoritySubjectRef,
    authoritative_source_ref: String,
    head_generation: u64,
    head_transition_digest: Digest32,
    head_status_record_ref: String,
    challenge_digest: Digest32,
    policy_digest: Digest32,
    policy_profile: String,
    source_head_digest: Digest32,
    source_head_profile: String,
    witness_count: u16,
    trust_domain_count: u16,
    coverage_digest: Digest32,
    coverage_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedAuthorityStateCoverage {
    pub fn challenge_digest(&self) -> Digest32 {
        self.challenge_digest
    }

    pub fn policy_digest(&self) -> Digest32 {
        self.policy_digest
    }

    pub fn policy_profile(&self) -> &str {
        &self.policy_profile
    }

    pub fn source_head_digest(&self) -> Digest32 {
        self.source_head_digest
    }

    pub fn source_head_profile(&self) -> &str {
        &self.source_head_profile
    }

    pub fn witness_count(&self) -> u16 {
        self.witness_count
    }

    pub fn trust_domain_count(&self) -> u16 {
        self.trust_domain_count
    }

    pub fn coverage_digest(&self) -> Digest32 {
        self.coverage_digest
    }

    pub fn coverage_profile(&self) -> &str {
        &self.coverage_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }

    pub fn to_state_source_coverage(&self) -> VerifiedAuthorityStateCoverage {
        VerifiedAuthorityStateCoverage {
            subject: self.subject.clone(),
            authoritative_source_ref: self.authoritative_source_ref.clone(),
            head_generation: self.head_generation,
            head_transition_digest: self.head_transition_digest,
            head_status_record_ref: self.head_status_record_ref.clone(),
            coverage_proof_ref: coverage_ref(self.coverage_digest),
            verification_ref: coverage_verification_ref(self.coverage_digest),
            verified_at_ms: self.verified_at_ms,
            lease_until_ms: self.lease_until_ms,
        }
    }
}

pub fn qualify_authority_state_coverage(
    policy_receipt: &VerifiedAuthorityCoveragePolicy,
    source_receipt: &VerifiedAuthoritySourceHead,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    challenge_digest: Digest32,
    now_ms: u64,
) -> Result<QualifiedAuthorityStateCoverage, AuthorityCoverageError> {
    if now_ms == 0 {
        return Err(AuthorityCoverageError::InvalidVerificationTime);
    }
    if challenge_digest.is_zero() {
        return Err(AuthorityCoverageError::ZeroChallenge);
    }
    let policy = verify_policy(policy_receipt, now_ms)?;
    let policy_digest = policy.identity_digest()?;
    let source = verify_source(source_receipt, policy, challenge_digest, now_ms)?;
    let source_head_digest = source.identity_digest()?;

    let mut canonical_witnesses = BTreeMap::<String, WitnessQualified>::new();
    let mut domain_counts = BTreeMap::<String, u16>::new();
    let mut verified_at_ms = policy_receipt
        .verified_at_ms
        .max(source_receipt.verified_at_ms);
    let mut lease_until_ms = source
        .expires_at_ms
        .min(policy.valid_until_ms)
        .min(now_ms.saturating_add(policy.max_coverage_lease_ms));

    match &policy.mode {
        CoverageMode::DirectSource => {
            if !witness_receipts.is_empty() {
                return Err(AuthorityCoverageError::UnexpectedWitnessEvidence);
            }
        }
        CoverageMode::WitnessQuorum {
            min_witnesses,
            min_trust_domains,
            max_per_trust_domain,
        } => {
            if witness_receipts.len() > MAX_WITNESSES {
                return Err(AuthorityCoverageError::TooManyWitnesses);
            }
            for receipt in witness_receipts {
                let qualified = verify_witness(
                    receipt,
                    source,
                    source_head_digest,
                    challenge_digest,
                    policy,
                    now_ms,
                )?;
                match canonical_witnesses.get(&qualified.observer_id) {
                    Some(existing) if existing != &qualified => {
                        return Err(AuthorityCoverageError::AmbiguousObserver);
                    }
                    Some(_) => {}
                    None => {
                        let count = domain_counts
                            .entry(qualified.trust_domain_id.clone())
                            .or_insert(0);
                        *count = count.saturating_add(1);
                        if *count > *max_per_trust_domain {
                            return Err(AuthorityCoverageError::TrustDomainConcentration);
                        }
                        verified_at_ms = verified_at_ms.max(qualified.verified_at_ms);
                        lease_until_ms = lease_until_ms.min(qualified.expires_at_ms);
                        canonical_witnesses.insert(qualified.observer_id.clone(), qualified);
                    }
                }
            }

            if canonical_witnesses.len() < usize::from(*min_witnesses) {
                return Err(AuthorityCoverageError::InsufficientWitnesses);
            }
            if domain_counts.len() < usize::from(*min_trust_domains) {
                return Err(AuthorityCoverageError::InsufficientTrustDomains);
            }
        }
    }

    if lease_until_ms <= now_ms || verified_at_ms > now_ms {
        return Err(AuthorityCoverageError::CoverageExpired);
    }

    let witness_digests = canonical_witnesses
        .values()
        .map(|value| value.identity_digest)
        .collect::<Vec<_>>();
    let coverage_digest = coverage_digest(
        policy_digest,
        source_head_digest,
        challenge_digest,
        &witness_digests,
    );

    Ok(QualifiedAuthorityStateCoverage {
        subject: source.subject.clone(),
        authoritative_source_ref: source.authoritative_source_ref.clone(),
        head_generation: source.head_generation,
        head_transition_digest: source.head_transition_digest,
        head_status_record_ref: source.head_status_record_ref.clone(),
        challenge_digest,
        policy_digest,
        policy_profile: POLICY_IDENTITY_PROFILE.into(),
        source_head_digest,
        source_head_profile: SOURCE_HEAD_IDENTITY_PROFILE.into(),
        witness_count: canonical_witnesses.len() as u16,
        trust_domain_count: domain_counts.len() as u16,
        coverage_digest,
        coverage_profile: COVERAGE_IDENTITY_PROFILE.into(),
        verified_at_ms,
        lease_until_ms,
    })
}

fn verify_policy<'a>(
    receipt: &'a VerifiedAuthorityCoveragePolicy,
    now_ms: u64,
) -> Result<&'a AuthorityCoveragePolicy, AuthorityCoverageError> {
    let policy = &receipt.policy;
    policy.validate()?;
    if !policy.is_active_at(now_ms) {
        return Err(AuthorityCoverageError::PolicyInactive);
    }
    for value in [
        receipt.policy_record_ref.as_str(),
        receipt.verified_policy_proof_ref.as_str(),
        receipt.verified_authority_ref.as_str(),
        receipt.verification_ref.as_str(),
    ] {
        require_ref(value)?;
    }
    if receipt.verified_policy_proof_ref != policy.policy_proof_ref {
        return Err(AuthorityCoverageError::PolicyProofMismatch);
    }
    if receipt.verified_authority_ref != policy.authority_ref {
        return Err(AuthorityCoverageError::PolicyAuthorityMismatch);
    }
    if receipt.verified_at_ms == 0 || receipt.verified_at_ms > now_ms {
        return Err(AuthorityCoverageError::InvalidVerificationTime);
    }
    Ok(policy)
}

fn verify_source<'a>(
    receipt: &'a VerifiedAuthoritySourceHead,
    policy: &AuthorityCoveragePolicy,
    challenge_digest: Digest32,
    now_ms: u64,
) -> Result<&'a AuthoritySourceHeadAttestation, AuthorityCoverageError> {
    let source = &receipt.attestation;
    source.validate()?;
    if source.subject.namespace != policy.namespace
        || !policy
            .allowed_subject_kinds
            .iter()
            .any(|kind| kind == &source.subject.kind)
    {
        return Err(AuthorityCoverageError::SubjectOutsidePolicy);
    }
    if source.authoritative_source_ref != policy.authoritative_source_ref {
        return Err(AuthorityCoverageError::SourceMismatch);
    }
    if source.source_identity != policy.source_identity
        || receipt.verified_source_identity != policy.source_identity
    {
        return Err(AuthorityCoverageError::SourceIdentityMismatch);
    }
    if source.challenge_digest != challenge_digest {
        return Err(AuthorityCoverageError::ChallengeMismatch);
    }
    if receipt.verified_source_proof_ref != source.source_proof_ref {
        return Err(AuthorityCoverageError::SourceProofMismatch);
    }
    require_ref(&receipt.verification_ref)?;
    if source.responded_at_ms < policy.valid_from_ms
        || source.responded_at_ms >= policy.valid_until_ms
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < source.responded_at_ms
        || now_ms >= source.expires_at_ms
        || now_ms.saturating_sub(source.responded_at_ms) > policy.max_source_age_ms
    {
        return Err(AuthorityCoverageError::StaleSourceHead);
    }
    Ok(source)
}

fn verify_witness(
    receipt: &VerifiedAuthorityHeadWitness,
    source: &AuthoritySourceHeadAttestation,
    source_head_digest: Digest32,
    challenge_digest: Digest32,
    policy: &AuthorityCoveragePolicy,
    now_ms: u64,
) -> Result<WitnessQualified, AuthorityCoverageError> {
    let witness = &receipt.observation;
    witness.validate()?;
    if witness.subject != source.subject
        || witness.authoritative_source_ref != source.authoritative_source_ref
        || witness.source_head_digest != source_head_digest
        || witness.head_generation != source.head_generation
        || witness.head_transition_digest != source.head_transition_digest
        || witness.head_status_record_ref != source.head_status_record_ref
        || witness.challenge_digest != challenge_digest
    {
        return Err(AuthorityCoverageError::WitnessHeadMismatch);
    }
    if receipt.verified_observation_proof_ref != witness.observation_proof_ref {
        return Err(AuthorityCoverageError::WitnessProofMismatch);
    }
    require_ref(&receipt.verified_observer_id)?;
    require_ref(&receipt.verified_trust_domain_id)?;
    require_ref(&receipt.verified_trust_domain_ref)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.verified_observer_id != witness.observer_id {
        return Err(AuthorityCoverageError::WitnessObserverMismatch);
    }
    if receipt.verified_trust_domain_id != witness.trust_domain_id {
        return Err(AuthorityCoverageError::WitnessTrustDomainMismatch);
    }
    if receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < witness.observed_at_ms
        || witness.observed_at_ms < source.responded_at_ms
        || now_ms >= witness.expires_at_ms
        || now_ms.saturating_sub(witness.observed_at_ms) > policy.max_witness_age_ms
    {
        return Err(AuthorityCoverageError::StaleWitness);
    }

    Ok(WitnessQualified {
        observer_id: witness.observer_id.clone(),
        trust_domain_id: witness.trust_domain_id.clone(),
        identity_digest: witness.identity_digest()?,
        verified_at_ms: receipt.verified_at_ms,
        expires_at_ms: witness.expires_at_ms,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct WitnessQualified {
    observer_id: String,
    trust_domain_id: String,
    identity_digest: Digest32,
    verified_at_ms: u64,
    expires_at_ms: u64,
}

fn coverage_digest(
    policy_digest: Digest32,
    source_head_digest: Digest32,
    challenge_digest: Digest32,
    witness_digests: &[Digest32],
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_COVERAGE);
    frame(&mut hasher, COVERAGE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &policy_digest.0);
    frame(&mut hasher, &source_head_digest.0);
    frame(&mut hasher, &challenge_digest.0);
    frame(&mut hasher, &(witness_digests.len() as u64).to_le_bytes());
    for digest in witness_digests {
        frame(&mut hasher, &digest.0);
    }
    Digest32(*hasher.finalize().as_bytes())
}

fn coverage_ref(digest: Digest32) -> String {
    format!("coverage:{}:{}", COVERAGE_IDENTITY_PROFILE, hex_digest(digest))
}

fn coverage_verification_ref(digest: Digest32) -> String {
    format!("coverage-verification:{}", hex_digest(digest))
}

fn canonical_subject_kind_codes(
    kinds: &[AuthoritySubjectKind],
) -> Result<Vec<u8>, AuthorityCoverageError> {
    let mut codes = kinds.iter().map(subject_kind_code).collect::<Vec<_>>();
    codes.sort_unstable();
    let mut seen = BTreeSet::new();
    for code in &codes {
        if !seen.insert(*code) {
            return Err(AuthorityCoverageError::DuplicateSubjectKind);
        }
    }
    Ok(codes)
}

fn subject_kind_code(kind: &AuthoritySubjectKind) -> u8 {
    match kind {
        AuthoritySubjectKind::AuthorityGrant => 1,
        AuthoritySubjectKind::SigningPolicy => 2,
        AuthoritySubjectKind::ThresholdAuthorization => 3,
        AuthoritySubjectKind::ExecutorDesignation => 4,
        AuthoritySubjectKind::EffectSafetyPolicy => 5,
        AuthoritySubjectKind::Delegation => 6,
    }
}

fn require_ref(value: &str) -> Result<(), AuthorityCoverageError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(AuthorityCoverageError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), AuthorityCoverageError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(AuthorityCoverageError::InvalidProfile)
    } else {
        Ok(())
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

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AuthorityCoverageError {
    WrongProtocolVersion,
    InvalidReference,
    InvalidProfile,
    InvalidSourceIdentity,
    InvalidAllowedSubjectKinds,
    DuplicateSubjectKind,
    InvalidWitnessPolicy,
    ImpossibleWitnessPolicy,
    InvalidCoverageWindow,
    InvalidPolicyLifetime,
    InvalidSubject,
    InvalidHeadIdentity,
    ZeroChallenge,
    InvalidSourceAttestationLifetime,
    InvalidWitnessHeadIdentity,
    InvalidWitnessLifetime,
    InvalidVerificationTime,
    PolicyInactive,
    PolicyProofMismatch,
    PolicyAuthorityMismatch,
    SubjectOutsidePolicy,
    SourceMismatch,
    SourceIdentityMismatch,
    ChallengeMismatch,
    SourceProofMismatch,
    StaleSourceHead,
    UnexpectedWitnessEvidence,
    TooManyWitnesses,
    WitnessHeadMismatch,
    WitnessProofMismatch,
    WitnessObserverMismatch,
    WitnessTrustDomainMismatch,
    StaleWitness,
    AmbiguousObserver,
    TrustDomainConcentration,
    InsufficientWitnesses,
    InsufficientTrustDomains,
    CoverageExpired,
}

impl fmt::Display for AuthorityCoverageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::WrongProtocolVersion => "wrong authority-state coverage protocol version",
            Self::InvalidReference => "invalid authority-state coverage reference",
            Self::InvalidProfile => "invalid authority-state coverage profile",
            Self::InvalidSourceIdentity => "invalid authority-state source identity",
            Self::InvalidAllowedSubjectKinds => "invalid allowed authority subject kinds",
            Self::DuplicateSubjectKind => "duplicate authority subject kind",
            Self::InvalidWitnessPolicy => "invalid authority coverage witness policy",
            Self::ImpossibleWitnessPolicy => "authority coverage witness policy is impossible",
            Self::InvalidCoverageWindow => "invalid authority coverage reuse window",
            Self::InvalidPolicyLifetime => "invalid authority coverage policy lifetime",
            Self::InvalidSubject => "invalid authority coverage subject",
            Self::InvalidHeadIdentity => "invalid authority source head identity",
            Self::ZeroChallenge => "authority coverage challenge must be non-zero",
            Self::InvalidSourceAttestationLifetime => "invalid source head attestation lifetime",
            Self::InvalidWitnessHeadIdentity => "invalid witness head identity",
            Self::InvalidWitnessLifetime => "invalid witness observation lifetime",
            Self::InvalidVerificationTime => "invalid authority coverage verification time",
            Self::PolicyInactive => "authority coverage policy is inactive",
            Self::PolicyProofMismatch => "verified coverage policy proof mismatch",
            Self::PolicyAuthorityMismatch => "verified coverage policy authority mismatch",
            Self::SubjectOutsidePolicy => "authority subject is outside coverage policy",
            Self::SourceMismatch => "authority coverage source mismatch",
            Self::SourceIdentityMismatch => "authority coverage source identity mismatch",
            Self::ChallengeMismatch => "authority coverage challenge mismatch",
            Self::SourceProofMismatch => "verified source head proof mismatch",
            Self::StaleSourceHead => "authority source head is stale or unverified",
            Self::UnexpectedWitnessEvidence => "witness evidence supplied for direct-source policy",
            Self::TooManyWitnesses => "authority coverage witness fan-in exceeds v0.1 bound",
            Self::WitnessHeadMismatch => "witness observed a different authority source head",
            Self::WitnessProofMismatch => "verified witness proof mismatch",
            Self::WitnessObserverMismatch => "verified witness observer identity mismatch",
            Self::WitnessTrustDomainMismatch => "verified witness trust-domain identity mismatch",
            Self::StaleWitness => "authority head witness is stale or unverified",
            Self::AmbiguousObserver => "same observer supplied conflicting coverage evidence",
            Self::TrustDomainConcentration => "too many witnesses come from one trust domain",
            Self::InsufficientWitnesses => "authority coverage witness quorum is insufficient",
            Self::InsufficientTrustDomains => "authority coverage trust-domain diversity is insufficient",
            Self::CoverageExpired => "qualified authority source coverage is expired",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for AuthorityCoverageError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn subject() -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind: AuthoritySubjectKind::AuthorityGrant,
            namespace: "institution:test".into(),
            subject_id: "grant:test".into(),
            identity: FreshnessProfiledDigest {
                digest: d(1),
                profile: "grant-v1-blake3".into(),
            },
        }
    }

    fn policy(mode: CoverageMode) -> AuthorityCoveragePolicy {
        AuthorityCoveragePolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "coverage-policy:test".into(),
            namespace: "institution:test".into(),
            allowed_subject_kinds: vec![AuthoritySubjectKind::AuthorityGrant],
            authoritative_source_ref: "authority-source:test".into(),
            source_identity: FreshnessProfiledDigest {
                digest: d(2),
                profile: "source-key-v1-blake3".into(),
            },
            mode,
            max_source_age_ms: 100,
            max_witness_age_ms: 100,
            max_coverage_lease_ms: 100,
            valid_from_ms: 1,
            valid_until_ms: 5_000,
            authority_ref: "rulebook-authority:test".into(),
            policy_proof_ref: "policy-proof:test".into(),
        }
    }

    fn policy_receipt(mode: CoverageMode) -> VerifiedAuthorityCoveragePolicy {
        let policy = policy(mode);
        VerifiedAuthorityCoveragePolicy {
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            verified_authority_ref: policy.authority_ref.clone(),
            policy_record_ref: "coverage-policy-record:test".into(),
            verification_ref: "coverage-policy-verification:test".into(),
            verified_at_ms: 950,
            policy,
        }
    }

    fn source(challenge: Digest32) -> AuthoritySourceHeadAttestation {
        AuthoritySourceHeadAttestation {
            protocol_version: PROTOCOL_VERSION.into(),
            subject: subject(),
            authoritative_source_ref: "authority-source:test".into(),
            source_identity: FreshnessProfiledDigest {
                digest: d(2),
                profile: "source-key-v1-blake3".into(),
            },
            head_generation: 3,
            head_transition_digest: d(3),
            head_status_record_ref: "status:3".into(),
            source_chain_head_ref: "source-chain-head:99".into(),
            challenge_digest: challenge,
            responded_at_ms: 920,
            expires_at_ms: 1_050,
            source_proof_ref: "source-proof:test".into(),
        }
    }

    fn source_receipt(challenge: Digest32) -> VerifiedAuthoritySourceHead {
        let attestation = source(challenge);
        VerifiedAuthoritySourceHead {
            verified_source_identity: attestation.source_identity.clone(),
            verified_source_proof_ref: attestation.source_proof_ref.clone(),
            verification_ref: "source-verification:test".into(),
            verified_at_ms: 940,
            attestation,
        }
    }

    fn witness(
        challenge: Digest32,
        source_head_digest: Digest32,
        observer: &str,
        domain: &str,
    ) -> VerifiedAuthorityHeadWitness {
        let observation = AuthorityHeadWitnessObservation {
            protocol_version: PROTOCOL_VERSION.into(),
            subject: subject(),
            authoritative_source_ref: "authority-source:test".into(),
            source_head_digest,
            head_generation: 3,
            head_transition_digest: d(3),
            head_status_record_ref: "status:3".into(),
            challenge_digest: challenge,
            observer_id: observer.into(),
            trust_domain_id: domain.into(),
            observed_at_ms: 930,
            expires_at_ms: 1_050,
            observation_proof_ref: format!("witness-proof:{observer}"),
        };
        VerifiedAuthorityHeadWitness {
            verified_observation_proof_ref: observation.observation_proof_ref.clone(),
            verified_observer_id: observation.observer_id.clone(),
            verified_trust_domain_id: observation.trust_domain_id.clone(),
            verified_trust_domain_ref: format!("trust-domain-proof:{domain}"),
            verification_ref: format!("witness-verification:{observer}"),
            verified_at_ms: 945,
            observation,
        }
    }

    #[test]
    fn direct_source_qualifies_exact_challenge_head() {
        let challenge = d(9);
        let coverage = qualify_authority_state_coverage(
            &policy_receipt(CoverageMode::DirectSource),
            &source_receipt(challenge),
            &[],
            challenge,
            1_000,
        )
        .unwrap();
        let projected = coverage.to_state_source_coverage();
        assert_eq!(projected.head_generation, 3);
        assert_eq!(projected.head_transition_digest, d(3));
    }

    #[test]
    fn challenge_replay_under_another_challenge_denies() {
        let source_challenge = d(9);
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(CoverageMode::DirectSource),
                &source_receipt(source_challenge),
                &[],
                d(8),
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::ChallengeMismatch
        );
    }

    #[test]
    fn witness_quorum_requires_independent_trust_domains() {
        let challenge = d(9);
        let source = source_receipt(challenge);
        let source_digest = source.attestation.identity_digest().unwrap();
        let mode = CoverageMode::WitnessQuorum {
            min_witnesses: 3,
            min_trust_domains: 2,
            max_per_trust_domain: 2,
        };
        let witnesses = vec![
            witness(challenge, source_digest, "observer:a", "domain:1"),
            witness(challenge, source_digest, "observer:b", "domain:1"),
            witness(challenge, source_digest, "observer:c", "domain:2"),
        ];
        let coverage = qualify_authority_state_coverage(
            &policy_receipt(mode),
            &source,
            &witnesses,
            challenge,
            1_000,
        )
        .unwrap();
        assert_eq!(coverage.witness_count(), 3);
        assert_eq!(coverage.trust_domain_count(), 2);
    }

    #[test]
    fn many_observers_in_one_domain_cannot_fake_independence() {
        let challenge = d(9);
        let source = source_receipt(challenge);
        let source_digest = source.attestation.identity_digest().unwrap();
        let mode = CoverageMode::WitnessQuorum {
            min_witnesses: 3,
            min_trust_domains: 2,
            max_per_trust_domain: 2,
        };
        let witnesses = vec![
            witness(challenge, source_digest, "observer:a", "domain:1"),
            witness(challenge, source_digest, "observer:b", "domain:1"),
            witness(challenge, source_digest, "observer:c", "domain:1"),
        ];
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(mode),
                &source,
                &witnesses,
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::TrustDomainConcentration
        );
    }

    #[test]
    fn witness_for_another_head_denies() {
        let challenge = d(9);
        let source = source_receipt(challenge);
        let source_digest = source.attestation.identity_digest().unwrap();
        let mode = CoverageMode::WitnessQuorum {
            min_witnesses: 1,
            min_trust_domains: 1,
            max_per_trust_domain: 1,
        };
        let mut wrong = witness(challenge, source_digest, "observer:a", "domain:1");
        wrong.observation.head_transition_digest = d(44);
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(mode),
                &source,
                &[wrong],
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::WitnessHeadMismatch
        );
    }

    #[test]
    fn same_observer_conflicting_trust_domain_denies() {
        let challenge = d(9);
        let source = source_receipt(challenge);
        let source_digest = source.attestation.identity_digest().unwrap();
        let mode = CoverageMode::WitnessQuorum {
            min_witnesses: 1,
            min_trust_domains: 1,
            max_per_trust_domain: 2,
        };
        let witnesses = vec![
            witness(challenge, source_digest, "observer:a", "domain:1"),
            witness(challenge, source_digest, "observer:a", "domain:2"),
        ];
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(mode),
                &source,
                &witnesses,
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::AmbiguousObserver
        );
    }

    #[test]
    fn verified_trust_domain_must_match_observation() {
        let challenge = d(9);
        let source = source_receipt(challenge);
        let source_digest = source.attestation.identity_digest().unwrap();
        let mode = CoverageMode::WitnessQuorum {
            min_witnesses: 1,
            min_trust_domains: 1,
            max_per_trust_domain: 1,
        };
        let mut witness = witness(challenge, source_digest, "observer:a", "domain:1");
        witness.verified_trust_domain_id = "domain:other".into();
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(mode),
                &source,
                &[witness],
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::WitnessTrustDomainMismatch
        );
    }

    #[test]
    fn source_identity_change_denies_under_same_logical_source_ref() {
        let challenge = d(9);
        let mut source = source_receipt(challenge);
        source.attestation.source_identity.digest = d(99);
        source.verified_source_identity.digest = d(99);
        assert_eq!(
            qualify_authority_state_coverage(
                &policy_receipt(CoverageMode::DirectSource),
                &source,
                &[],
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::SourceIdentityMismatch
        );
    }

    #[test]
    fn source_response_before_policy_lifetime_denies() {
        let challenge = d(9);
        let mut policy = policy_receipt(CoverageMode::DirectSource);
        policy.policy.valid_from_ms = 925;
        assert_eq!(
            qualify_authority_state_coverage(
                &policy,
                &source_receipt(challenge),
                &[],
                challenge,
                1_000,
            )
            .unwrap_err(),
            AuthorityCoverageError::StaleSourceHead
        );
    }
}
