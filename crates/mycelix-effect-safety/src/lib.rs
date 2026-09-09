// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure qualification for one exact effect-safety policy and the exact adapter
//! release/mechanisms that policy requires.
//!
//! Policy semantics, immutable record authenticity, institutional adoption,
//! generation currentness and adapter enforcement evidence are deliberately
//! separate facts. This crate composes them without claiming that deserializable
//! proof receipts prove their own runtime origin and without creating an
//! execution attempt or external effect.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef,
    CurrentAuthorityFreshness, ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_execution_action_digest::ACTIONS_DIGEST_PROFILE_V1;
use mycelix_institutional_core::{Digest32, InstitutionId, JurisdictionId, RulebookRef};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-effect-safety-v0.1";
pub const POLICY_RECORD_PROOF_PROTOCOL: &str = "mycelix-effect-safety-policy-record-proof-v0.1";
pub const POLICY_ADOPTION_PROOF_PROTOCOL: &str =
    "mycelix-effect-safety-policy-adoption-proof-v0.1";
pub const ADAPTER_QUALIFICATION_PROTOCOL: &str = "mycelix-effect-adapter-qualification-v0.1";
pub const POLICY_IDENTITY_PROFILE: &str =
    "mycelix-effect-safety-policy-v1-blake3-framed";
pub const CURRENT_QUALIFICATION_PROFILE: &str =
    "mycelix-effect-safety-current-qualification-v1-blake3-framed";
pub const EVIDENCE_PROFILE: &str =
    "mycelix-effect-safety-evidence-v1-blake3-framed";
pub const MAX_ADAPTER_ATTESTATION_LEASE_MS: u64 = 5_000;
pub const MAX_ATTEMPT_LEASE_MS: u64 = 60_000;

const DOMAIN_POLICY: &[u8] = b"mycelix/effect-safety/policy/v1";
const DOMAIN_CURRENT: &[u8] = b"mycelix/effect-safety/current/v1";
const DOMAIN_EVIDENCE: &[u8] = b"mycelix/effect-safety/evidence/v1";
const MAX_TEXT_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Semantic policy for one exact authorized action identity and one exact adapter
/// release. This is transportable policy data, not authority merely because it
/// deserializes.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EffectSafetyPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub action_class: String,
    pub actions_digest: Digest32,
    pub actions_digest_profile: String,
    pub adapter_profile: String,
    pub adapter_release_digest: Digest32,
    pub adapter_release_profile: String,
    pub automatic_effects_allowed: bool,
    pub requires_idempotency: bool,
    pub requires_attempt_fencing: bool,
    pub requires_precondition_fence: bool,
    pub requires_compensation: bool,
    /// Maximum lease a later attempt-admission theorem may grant from this policy.
    /// This is not itself an attempt lease.
    pub max_attempt_lease_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    /// Institutional authority expected to have adopted this exact policy.
    pub authority_ref: String,
    /// Immutable adoption proof identity expected by this exact policy.
    pub policy_proof_ref: String,
}

impl EffectSafetyPolicy {
    pub fn validate(&self) -> Result<(), EffectSafetyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(EffectSafetyError::WrongProtocolVersion);
        }
        require_text(&self.policy_id)?;
        require_text(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_text(jurisdiction.as_str())?;
        }
        self.rulebook
            .validate()
            .map_err(|_| EffectSafetyError::InvalidRulebook)?;
        require_text(&self.action_class)?;
        require_digest(self.actions_digest)?;
        if self.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1 {
            return Err(EffectSafetyError::UnsupportedActionsDigestProfile);
        }
        require_profile(&self.adapter_profile)?;
        require_digest(self.adapter_release_digest)?;
        require_profile(&self.adapter_release_profile)?;
        require_text(&self.authority_ref)?;
        require_text(&self.policy_proof_ref)?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(EffectSafetyError::InvalidPolicyWindow);
        }
        if self.max_attempt_lease_ms == 0 || self.max_attempt_lease_ms > MAX_ATTEMPT_LEASE_MS {
            return Err(EffectSafetyError::InvalidAttemptLease);
        }
        // Automatic external effects are never allowed under a policy that omits
        // the two minimum replay/race controls. Stronger adapters may add more.
        if self.automatic_effects_allowed
            && (!self.requires_idempotency || !self.requires_attempt_fencing)
        {
            return Err(EffectSafetyError::UnsafeAutomaticEffectPolicy);
        }
        Ok(())
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, EffectSafetyError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_POLICY);
        frame(&mut hasher, POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.policy_id.as_bytes());
        frame(&mut hasher, self.institution.as_str().as_bytes());
        frame_optional_text(
            &mut hasher,
            self.jurisdiction.as_ref().map(|value| value.as_str()),
        );
        frame(&mut hasher, self.rulebook.id.as_str().as_bytes());
        frame(&mut hasher, self.rulebook.version.as_bytes());
        frame(&mut hasher, &self.rulebook.digest.0);
        frame(&mut hasher, self.action_class.as_bytes());
        frame(&mut hasher, &self.actions_digest.0);
        frame(&mut hasher, self.actions_digest_profile.as_bytes());
        frame(&mut hasher, self.adapter_profile.as_bytes());
        frame(&mut hasher, &self.adapter_release_digest.0);
        frame(&mut hasher, self.adapter_release_profile.as_bytes());
        frame_bool(&mut hasher, self.automatic_effects_allowed);
        frame_bool(&mut hasher, self.requires_idempotency);
        frame_bool(&mut hasher, self.requires_attempt_fencing);
        frame_bool(&mut hasher, self.requires_precondition_fence);
        frame_bool(&mut hasher, self.requires_compensation);
        frame(&mut hasher, &self.max_attempt_lease_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        frame(&mut hasher, self.authority_ref.as_bytes());
        frame(&mut hasher, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }

    pub fn subject_ref(&self) -> Result<AuthoritySubjectRef, EffectSafetyError> {
        Ok(AuthoritySubjectRef {
            kind: AuthoritySubjectKind::EffectSafetyPolicy,
            namespace: self.institution.as_str().into(),
            subject_id: self.policy_id.clone(),
            identity: ProfiledDigest {
                digest: self.identity_digest()?,
                profile: POLICY_IDENTITY_PROFILE.into(),
            },
        })
    }
}

/// Evidence-shaped authentication of one immutable policy record.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedEffectSafetyPolicyRecordProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub policy_record_ref: String,
    pub record_proof_ref: String,
    pub record_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Evidence-shaped verification that the institution/rulebook authority adopted
/// the exact policy identity under the exact proof reference committed by it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedEffectSafetyPolicyAdoptionProof {
    pub protocol_version: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub verified_authority_ref: String,
    pub verified_policy_proof_ref: String,
    pub authority_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Evidence that one exact adapter release currently exposes the enforcement
/// mechanisms required by a policy. Deserialization does not prove this receipt
/// came from a native adapter attestor.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedEffectAdapterQualification {
    pub protocol_version: String,
    pub adapter_profile: String,
    pub adapter_release_digest: Digest32,
    pub adapter_release_profile: String,
    pub deployment_ref: String,
    pub enforcement_ref: String,
    pub exact_action_digest_enforced: bool,
    pub supports_idempotency: bool,
    pub supports_attempt_fencing: bool,
    pub supports_precondition_fence: bool,
    pub supports_compensation: bool,
    pub external_effects_enabled: bool,
    pub adapter_verifier_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
}

impl VerifiedEffectAdapterQualification {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), EffectSafetyError> {
        if self.protocol_version != ADAPTER_QUALIFICATION_PROTOCOL {
            return Err(EffectSafetyError::WrongAdapterProtocol);
        }
        require_profile(&self.adapter_profile)?;
        require_digest(self.adapter_release_digest)?;
        require_profile(&self.adapter_release_profile)?;
        for value in [
            self.deployment_ref.as_str(),
            self.enforcement_ref.as_str(),
            self.adapter_verifier_ref.as_str(),
            self.verification_ref.as_str(),
        ] {
            require_text(value)?;
        }
        validate_window(self.verified_at_ms, self.valid_until_ms, now_ms)?;
        if self.valid_until_ms - self.verified_at_ms > MAX_ADAPTER_ATTESTATION_LEASE_MS {
            return Err(EffectSafetyError::AdapterLeaseTooLong);
        }
        Ok(())
    }
}

/// Non-deserializable result of exact semantic/current/mechanism qualification.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedEffectSafetyPolicy {
    policy: EffectSafetyPolicy,
    policy_record_ref: String,
    subject: AuthoritySubjectRef,
    current_freshness_digest: Digest32,
    current_freshness_profile: String,
    adapter_deployment_ref: String,
    adapter_enforcement_ref: String,
    qualification_digest: Digest32,
    evidence_digest: Digest32,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedEffectSafetyPolicy {
    pub fn policy(&self) -> &EffectSafetyPolicy {
        &self.policy
    }

    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }

    pub fn policy_record_ref(&self) -> &str {
        &self.policy_record_ref
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        CURRENT_QUALIFICATION_PROFILE
    }

    pub fn evidence_digest(&self) -> Digest32 {
        self.evidence_digest
    }

    pub fn evidence_profile(&self) -> &str {
        EVIDENCE_PROFILE
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn adapter_deployment_ref(&self) -> &str {
        &self.adapter_deployment_ref
    }

    pub fn adapter_enforcement_ref(&self) -> &str {
        &self.adapter_enforcement_ref
    }

    pub fn automatic_effects_allowed(&self) -> bool {
        self.policy.automatic_effects_allowed
    }

    pub const fn mechanism_requirements_satisfied_here(&self) -> bool {
        true
    }

    pub const fn current_policy_semantics_bound_here(&self) -> bool {
        true
    }

    pub const fn record_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn adoption_proof_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn freshness_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn adapter_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn attempt_fenced_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Qualify an exact policy identity against independent record/adoption proofs,
/// exact generation-bound currentness and exact adapter enforcement evidence.
#[allow(clippy::too_many_arguments)]
pub fn qualify_effect_safety_policy(
    policy: &EffectSafetyPolicy,
    policy_record_ref: &str,
    record_proof: &VerifiedEffectSafetyPolicyRecordProof,
    adoption_proof: &VerifiedEffectSafetyPolicyAdoptionProof,
    freshness: &VerifiedAuthorityFreshness,
    adapter: &VerifiedEffectAdapterQualification,
    now_ms: u64,
) -> Result<QualifiedEffectSafetyPolicy, EffectSafetyError> {
    policy.validate()?;
    if !policy.is_active_at(now_ms) {
        return Err(EffectSafetyError::PolicyNotActive);
    }
    require_text(policy_record_ref)?;
    let policy_digest = policy.identity_digest()?;
    verify_record_proof(record_proof, policy_digest, policy_record_ref, now_ms)?;
    verify_adoption_proof(adoption_proof, policy, policy_digest, now_ms)?;

    let subject = policy.subject_ref()?;
    if freshness.snapshot.effective_at_ms < policy.valid_from_ms {
        return Err(EffectSafetyError::FreshnessPredatesPolicy);
    }
    let current = qualify_current_freshness(
        std::slice::from_ref(&subject),
        std::slice::from_ref(freshness),
        now_ms,
    )
    .map_err(|error| EffectSafetyError::Freshness(error.to_string()))?;

    adapter.validate_at(now_ms)?;
    verify_adapter(policy, adapter)?;

    let verified_at_ms = record_proof
        .verified_at_ms
        .max(adoption_proof.verified_at_ms)
        .max(current.verified_at_ms)
        .max(adapter.verified_at_ms);
    let valid_until_ms = policy
        .valid_until_ms
        .min(record_proof.valid_until_ms)
        .min(adoption_proof.valid_until_ms)
        .min(current.lease_until_ms)
        .min(adapter.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(EffectSafetyError::NoUsableQualificationWindow);
    }

    let qualification_digest = qualification_digest(policy_digest, &current);
    let evidence_digest = evidence_digest(
        qualification_digest,
        policy_record_ref,
        record_proof,
        adoption_proof,
        adapter,
        verified_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedEffectSafetyPolicy {
        policy: policy.clone(),
        policy_record_ref: policy_record_ref.into(),
        subject,
        current_freshness_digest: current.freshness_digest,
        current_freshness_profile: current.freshness_profile,
        adapter_deployment_ref: adapter.deployment_ref.clone(),
        adapter_enforcement_ref: adapter.enforcement_ref.clone(),
        qualification_digest,
        evidence_digest,
        verified_at_ms,
        valid_until_ms,
    })
}

fn verify_record_proof(
    proof: &VerifiedEffectSafetyPolicyRecordProof,
    policy_digest: Digest32,
    policy_record_ref: &str,
    now_ms: u64,
) -> Result<(), EffectSafetyError> {
    if proof.protocol_version != POLICY_RECORD_PROOF_PROTOCOL {
        return Err(EffectSafetyError::WrongRecordProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != POLICY_IDENTITY_PROFILE {
        return Err(EffectSafetyError::RecordPolicyIdentityMismatch);
    }
    require_profile(&proof.policy_profile)?;
    for value in [
        proof.policy_record_ref.as_str(),
        proof.record_proof_ref.as_str(),
        proof.record_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_text(value)?;
    }
    if proof.policy_record_ref != policy_record_ref {
        return Err(EffectSafetyError::PolicyRecordRefMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn verify_adoption_proof(
    proof: &VerifiedEffectSafetyPolicyAdoptionProof,
    policy: &EffectSafetyPolicy,
    policy_digest: Digest32,
    now_ms: u64,
) -> Result<(), EffectSafetyError> {
    if proof.protocol_version != POLICY_ADOPTION_PROOF_PROTOCOL {
        return Err(EffectSafetyError::WrongAdoptionProofProtocol);
    }
    if proof.policy_digest != policy_digest || proof.policy_profile != POLICY_IDENTITY_PROFILE {
        return Err(EffectSafetyError::AdoptionPolicyIdentityMismatch);
    }
    require_profile(&proof.policy_profile)?;
    for value in [
        proof.verified_authority_ref.as_str(),
        proof.verified_policy_proof_ref.as_str(),
        proof.authority_verifier_ref.as_str(),
        proof.verification_ref.as_str(),
    ] {
        require_text(value)?;
    }
    if proof.verified_authority_ref != policy.authority_ref {
        return Err(EffectSafetyError::AdoptionAuthorityMismatch);
    }
    if proof.verified_policy_proof_ref != policy.policy_proof_ref {
        return Err(EffectSafetyError::AdoptionProofRefMismatch);
    }
    validate_window(proof.verified_at_ms, proof.valid_until_ms, now_ms)
}

fn verify_adapter(
    policy: &EffectSafetyPolicy,
    adapter: &VerifiedEffectAdapterQualification,
) -> Result<(), EffectSafetyError> {
    if adapter.adapter_profile != policy.adapter_profile {
        return Err(EffectSafetyError::AdapterProfileMismatch);
    }
    if adapter.adapter_release_digest != policy.adapter_release_digest
        || adapter.adapter_release_profile != policy.adapter_release_profile
    {
        return Err(EffectSafetyError::AdapterReleaseMismatch);
    }
    if !adapter.exact_action_digest_enforced {
        return Err(EffectSafetyError::ActionDigestNotEnforced);
    }
    if policy.requires_idempotency && !adapter.supports_idempotency {
        return Err(EffectSafetyError::MissingIdempotency);
    }
    if policy.requires_attempt_fencing && !adapter.supports_attempt_fencing {
        return Err(EffectSafetyError::MissingAttemptFencing);
    }
    if policy.requires_precondition_fence && !adapter.supports_precondition_fence {
        return Err(EffectSafetyError::MissingPreconditionFence);
    }
    if policy.requires_compensation && !adapter.supports_compensation {
        return Err(EffectSafetyError::MissingCompensation);
    }
    if policy.automatic_effects_allowed
        && (!adapter.external_effects_enabled
            || !adapter.supports_idempotency
            || !adapter.supports_attempt_fencing)
    {
        return Err(EffectSafetyError::AutomaticEffectsUnavailable);
    }
    Ok(())
}

fn qualification_digest(policy_digest: Digest32, current: &CurrentAuthorityFreshness) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENT);
    frame(&mut hasher, CURRENT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &policy_digest.0);
    frame(&mut hasher, current.freshness_profile.as_bytes());
    frame(&mut hasher, &current.freshness_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

#[allow(clippy::too_many_arguments)]
fn evidence_digest(
    qualification_digest: Digest32,
    policy_record_ref: &str,
    record: &VerifiedEffectSafetyPolicyRecordProof,
    adoption: &VerifiedEffectSafetyPolicyAdoptionProof,
    adapter: &VerifiedEffectAdapterQualification,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_EVIDENCE);
    frame(&mut hasher, EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, &qualification_digest.0);
    frame(&mut hasher, policy_record_ref.as_bytes());
    frame(&mut hasher, record.record_proof_ref.as_bytes());
    frame(&mut hasher, record.record_verifier_ref.as_bytes());
    frame(&mut hasher, record.verification_ref.as_bytes());
    frame(&mut hasher, &record.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &record.valid_until_ms.to_le_bytes());
    frame(&mut hasher, adoption.authority_verifier_ref.as_bytes());
    frame(&mut hasher, adoption.verification_ref.as_bytes());
    frame(&mut hasher, &adoption.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &adoption.valid_until_ms.to_le_bytes());
    frame(&mut hasher, adapter.deployment_ref.as_bytes());
    frame(&mut hasher, adapter.enforcement_ref.as_bytes());
    frame(&mut hasher, adapter.adapter_verifier_ref.as_bytes());
    frame(&mut hasher, adapter.verification_ref.as_bytes());
    frame(&mut hasher, &adapter.verified_at_ms.to_le_bytes());
    frame(&mut hasher, &adapter.valid_until_ms.to_le_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

fn require_text(value: &str) -> Result<(), EffectSafetyError> {
    if value.trim().is_empty() || value.len() > MAX_TEXT_BYTES {
        Err(EffectSafetyError::InvalidText)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), EffectSafetyError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(EffectSafetyError::InvalidProfile)
    } else {
        Ok(())
    }
}

fn require_digest(value: Digest32) -> Result<(), EffectSafetyError> {
    if value.is_zero() {
        Err(EffectSafetyError::ZeroDigest)
    } else {
        Ok(())
    }
}

fn validate_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
    now_ms: u64,
) -> Result<(), EffectSafetyError> {
    if now_ms == 0
        || verified_at_ms == 0
        || verified_at_ms > now_ms
        || valid_until_ms <= now_ms
        || valid_until_ms < verified_at_ms
    {
        Err(EffectSafetyError::InvalidVerificationWindow)
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn frame_bool(hasher: &mut blake3::Hasher, value: bool) {
    frame(hasher, &[u8::from(value)]);
}

fn frame_optional_text(hasher: &mut blake3::Hasher, value: Option<&str>) {
    match value {
        Some(value) => {
            frame(hasher, &[1]);
            frame(hasher, value.as_bytes());
        }
        None => frame(hasher, &[0]),
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EffectSafetyError {
    WrongProtocolVersion,
    InvalidText,
    InvalidProfile,
    ZeroDigest,
    InvalidRulebook,
    UnsupportedActionsDigestProfile,
    InvalidPolicyWindow,
    InvalidAttemptLease,
    UnsafeAutomaticEffectPolicy,
    PolicyNotActive,
    WrongRecordProofProtocol,
    RecordPolicyIdentityMismatch,
    PolicyRecordRefMismatch,
    WrongAdoptionProofProtocol,
    AdoptionPolicyIdentityMismatch,
    AdoptionAuthorityMismatch,
    AdoptionProofRefMismatch,
    FreshnessPredatesPolicy,
    Freshness(String),
    WrongAdapterProtocol,
    InvalidVerificationWindow,
    AdapterLeaseTooLong,
    AdapterProfileMismatch,
    AdapterReleaseMismatch,
    ActionDigestNotEnforced,
    MissingIdempotency,
    MissingAttemptFencing,
    MissingPreconditionFence,
    MissingCompensation,
    AutomaticEffectsUnavailable,
    NoUsableQualificationWindow,
}

impl fmt::Display for EffectSafetyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong effect-safety protocol version"),
            Self::InvalidText => write!(f, "invalid effect-safety identifier/reference"),
            Self::InvalidProfile => write!(f, "invalid effect-safety profile token"),
            Self::ZeroDigest => write!(f, "effect-safety digest must not be zero"),
            Self::InvalidRulebook => write!(f, "invalid effect-safety rulebook"),
            Self::UnsupportedActionsDigestProfile => write!(f, "effect-safety policy uses unsupported action digest profile"),
            Self::InvalidPolicyWindow => write!(f, "invalid effect-safety policy validity window"),
            Self::InvalidAttemptLease => write!(f, "invalid effect-safety maximum attempt lease"),
            Self::UnsafeAutomaticEffectPolicy => write!(f, "automatic effects require policy-level idempotency and attempt fencing"),
            Self::PolicyNotActive => write!(f, "effect-safety policy is not semantically active"),
            Self::WrongRecordProofProtocol => write!(f, "wrong effect-safety record-proof protocol"),
            Self::RecordPolicyIdentityMismatch => write!(f, "effect-safety record proof targets another policy identity"),
            Self::PolicyRecordRefMismatch => write!(f, "effect-safety record proof targets another policy record"),
            Self::WrongAdoptionProofProtocol => write!(f, "wrong effect-safety adoption-proof protocol"),
            Self::AdoptionPolicyIdentityMismatch => write!(f, "effect-safety adoption proof targets another policy identity"),
            Self::AdoptionAuthorityMismatch => write!(f, "effect-safety adoption proof targets another institutional authority"),
            Self::AdoptionProofRefMismatch => write!(f, "effect-safety adoption proof reference does not match policy"),
            Self::FreshnessPredatesPolicy => write!(f, "effect-safety current-state fact predates the policy validity epoch"),
            Self::Freshness(error) => write!(f, "effect-safety currentness failed: {error}"),
            Self::WrongAdapterProtocol => write!(f, "wrong effect-adapter qualification protocol"),
            Self::InvalidVerificationWindow => write!(f, "invalid effect-safety verification window"),
            Self::AdapterLeaseTooLong => write!(f, "effect-adapter attestation exceeds the five-second reuse ceiling"),
            Self::AdapterProfileMismatch => write!(f, "effect adapter profile does not match policy"),
            Self::AdapterReleaseMismatch => write!(f, "effect adapter release identity does not match policy"),
            Self::ActionDigestNotEnforced => write!(f, "effect adapter does not enforce the exact authorized action digest"),
            Self::MissingIdempotency => write!(f, "effect adapter lacks policy-required idempotency"),
            Self::MissingAttemptFencing => write!(f, "effect adapter lacks policy-required attempt fencing"),
            Self::MissingPreconditionFence => write!(f, "effect adapter lacks policy-required precondition fencing"),
            Self::MissingCompensation => write!(f, "effect adapter lacks policy-required compensation support"),
            Self::AutomaticEffectsUnavailable => write!(f, "adapter cannot currently support policy-authorized automatic effects"),
            Self::NoUsableQualificationWindow => write!(f, "effect-safety evidence has no usable intersected validity window"),
        }
    }
}

impl std::error::Error for EffectSafetyError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, AuthorityFreshnessState,
        PROTOCOL_VERSION as FRESHNESS_PROTOCOL,
    };
    use mycelix_institutional_core::{RulebookId, RulebookRef};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn policy() -> EffectSafetyPolicy {
        EffectSafetyPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "effect-safety:cooling-center:v1".into(),
            institution: InstitutionId::new("municipality:johannesburg").unwrap(),
            jurisdiction: Some(JurisdictionId::new("Johannesburg").unwrap()),
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:heat-response").unwrap(),
                version: "1".into(),
                digest: d(1),
            },
            action_class: "open_cooling_center".into(),
            actions_digest: d(2),
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
            adapter_profile: "mycelix.municipal.cooling-center.v1".into(),
            adapter_release_digest: d(3),
            adapter_release_profile: "mycelix-adapter-release-v1-blake3".into(),
            automatic_effects_allowed: true,
            requires_idempotency: true,
            requires_attempt_fencing: true,
            requires_precondition_fence: true,
            requires_compensation: false,
            max_attempt_lease_ms: 10_000,
            valid_from_ms: 100,
            valid_until_ms: 1_000,
            authority_ref: "institutional-authority:jhb:emergency:v1".into(),
            policy_proof_ref: "policy-adoption:jhb:cooling:v1".into(),
        }
    }

    fn record(policy: &EffectSafetyPolicy) -> VerifiedEffectSafetyPolicyRecordProof {
        VerifiedEffectSafetyPolicyRecordProof {
            protocol_version: POLICY_RECORD_PROOF_PROTOCOL.into(),
            policy_digest: policy.identity_digest().unwrap(),
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            policy_record_ref: "policy-record:cooling:v1".into(),
            record_proof_ref: "record-proof:cooling:v1".into(),
            record_verifier_ref: "record-verifier:1".into(),
            verification_ref: "record-verification:1".into(),
            verified_at_ms: 150,
            valid_until_ms: 900,
        }
    }

    fn adoption(policy: &EffectSafetyPolicy) -> VerifiedEffectSafetyPolicyAdoptionProof {
        VerifiedEffectSafetyPolicyAdoptionProof {
            protocol_version: POLICY_ADOPTION_PROOF_PROTOCOL.into(),
            policy_digest: policy.identity_digest().unwrap(),
            policy_profile: POLICY_IDENTITY_PROFILE.into(),
            verified_authority_ref: policy.authority_ref.clone(),
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            authority_verifier_ref: "authority-verifier:1".into(),
            verification_ref: "adoption-verification:1".into(),
            verified_at_ms: 160,
            valid_until_ms: 800,
        }
    }

    fn freshness(policy: &EffectSafetyPolicy) -> VerifiedAuthorityFreshness {
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL.into(),
                subject: policy.subject_ref().unwrap(),
                generation: 1,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: 100,
                status_record_ref: "effect-safety-status:1".into(),
            },
            authoritative_source_ref: "effect-safety-source:jhb".into(),
            verification_ref: "freshness-verification:1".into(),
            verified_at_ms: 170,
            lease_until_ms: 700,
        }
    }

    fn adapter(policy: &EffectSafetyPolicy) -> VerifiedEffectAdapterQualification {
        VerifiedEffectAdapterQualification {
            protocol_version: ADAPTER_QUALIFICATION_PROTOCOL.into(),
            adapter_profile: policy.adapter_profile.clone(),
            adapter_release_digest: policy.adapter_release_digest,
            adapter_release_profile: policy.adapter_release_profile.clone(),
            deployment_ref: "adapter-deployment:jhb:cooling:1".into(),
            enforcement_ref: "adapter-enforcement:jhb:cooling:1".into(),
            exact_action_digest_enforced: true,
            supports_idempotency: true,
            supports_attempt_fencing: true,
            supports_precondition_fence: true,
            supports_compensation: false,
            external_effects_enabled: true,
            adapter_verifier_ref: "adapter-verifier:1".into(),
            verification_ref: "adapter-verification:1".into(),
            verified_at_ms: 190,
            valid_until_ms: 600,
        }
    }

    fn qualify(
        policy: &EffectSafetyPolicy,
        record: &VerifiedEffectSafetyPolicyRecordProof,
        adoption: &VerifiedEffectSafetyPolicyAdoptionProof,
        freshness: &VerifiedAuthorityFreshness,
        adapter: &VerifiedEffectAdapterQualification,
    ) -> Result<QualifiedEffectSafetyPolicy, EffectSafetyError> {
        qualify_effect_safety_policy(
            policy,
            "policy-record:cooling:v1",
            record,
            adoption,
            freshness,
            adapter,
            200,
        )
    }

    #[test]
    fn exact_policy_currentness_and_adapter_mechanisms_qualify() {
        let policy = policy();
        let qualified = qualify(
            &policy,
            &record(&policy),
            &adoption(&policy),
            &freshness(&policy),
            &adapter(&policy),
        )
        .unwrap();
        assert_eq!(qualified.subject(), &policy.subject_ref().unwrap());
        assert_eq!(qualified.valid_until_ms(), 600);
        assert!(qualified.mechanism_requirements_satisfied_here());
        assert!(qualified.current_policy_semantics_bound_here());
        assert!(!qualified.record_proof_origin_verified_here());
        assert!(!qualified.adoption_proof_origin_verified_here());
        assert!(!qualified.freshness_origin_verified_here());
        assert!(!qualified.adapter_origin_verified_here());
        assert!(!qualified.attempt_fenced_here());
        assert!(!qualified.grants_execution_authority());
    }

    #[test]
    fn record_identity_substitution_denies() {
        let policy = policy();
        let mut proof = record(&policy);
        proof.policy_digest = d(99);
        assert_eq!(
            qualify(
                &policy,
                &proof,
                &adoption(&policy),
                &freshness(&policy),
                &adapter(&policy),
            )
            .unwrap_err(),
            EffectSafetyError::RecordPolicyIdentityMismatch
        );
    }

    #[test]
    fn adoption_authority_substitution_denies() {
        let policy = policy();
        let mut proof = adoption(&policy);
        proof.verified_authority_ref = "institutional-authority:other".into();
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &proof,
                &freshness(&policy),
                &adapter(&policy),
            )
            .unwrap_err(),
            EffectSafetyError::AdoptionAuthorityMismatch
        );
    }

    #[test]
    fn revoked_policy_freshness_denies() {
        let policy = policy();
        let mut fresh = freshness(&policy);
        fresh.snapshot.state = AuthorityFreshnessState::Revoked;
        assert!(matches!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &fresh,
                &adapter(&policy),
            ),
            Err(EffectSafetyError::Freshness(_))
        ));
    }

    #[test]
    fn wrong_freshness_subject_denies() {
        let policy = policy();
        let mut fresh = freshness(&policy);
        fresh.snapshot.subject.subject_id = "effect-safety:other".into();
        assert!(matches!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &fresh,
                &adapter(&policy),
            ),
            Err(EffectSafetyError::Freshness(_))
        ));
    }

    #[test]
    fn adapter_release_substitution_denies() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.adapter_release_digest = d(77);
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::AdapterReleaseMismatch
        );
    }

    #[test]
    fn adapter_must_enforce_exact_authorized_action_digest() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.exact_action_digest_enforced = false;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::ActionDigestNotEnforced
        );
    }

    #[test]
    fn required_mechanisms_fail_closed() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.supports_idempotency = false;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::MissingIdempotency
        );

        let mut attestation = adapter(&policy);
        attestation.supports_attempt_fencing = false;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::MissingAttemptFencing
        );

        let mut attestation = adapter(&policy);
        attestation.supports_precondition_fence = false;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::MissingPreconditionFence
        );
    }

    #[test]
    fn automatic_effect_requires_currently_enabled_adapter() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.external_effects_enabled = false;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::AutomaticEffectsUnavailable
        );
    }

    #[test]
    fn adapter_attestation_reuse_is_hard_capped() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.valid_until_ms = attestation.verified_at_ms + MAX_ADAPTER_ATTESTATION_LEASE_MS + 1;
        assert_eq!(
            qualify(
                &policy,
                &record(&policy),
                &adoption(&policy),
                &freshness(&policy),
                &attestation,
            )
            .unwrap_err(),
            EffectSafetyError::AdapterLeaseTooLong
        );
    }

    #[test]
    fn qualification_lease_is_intersection_not_policy_lifetime() {
        let policy = policy();
        let mut attestation = adapter(&policy);
        attestation.valid_until_ms = 450;
        let qualified = qualify(
            &policy,
            &record(&policy),
            &adoption(&policy),
            &freshness(&policy),
            &attestation,
        )
        .unwrap();
        assert_eq!(qualified.valid_until_ms(), 450);
        assert!(qualified.valid_until_ms() < policy.valid_until_ms);
    }
}
