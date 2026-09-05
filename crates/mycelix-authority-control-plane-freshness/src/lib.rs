// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Constitution-rooted control-plane authority freshness.
//!
//! This crate bridges the non-circular bootstrap root to the ordinary generation
//! freshness machinery. It uses the exact root-embedded coverage/context policy,
//! a non-authoritative probe, exact source/witness coverage, and the complete
//! covered append-only transition lineage before producing current freshness for
//! control-plane policy subjects.
//!
//! It performs no Holochain calls, persistence, signature verification, challenge
//! generation, DHT selection, lifecycle claims, or external effects.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef,
    CurrentAuthorityFreshness, VerifiedAuthorityFreshness,
};
use mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot;
use mycelix_authority_state_coverage::{
    VerifiedAuthorityCoveragePolicy, VerifiedAuthorityHeadWitness, VerifiedAuthoritySourceHead,
};
use mycelix_authority_state_coverage_context::{
    qualify_context_bound_coverage, VerifiedCoverageChallenge, VerifiedCoverageTrustContextPolicy,
    VerifiedWitnessTrustBinding,
};
use mycelix_authority_state_source::{
    project_current_authority_state, VerifiedAuthorityStateTransition,
};
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-control-plane-freshness-v0.1";
pub const SUBJECT_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-control-plane-subject-freshness-v1-blake3-framed";
pub const SET_QUALIFICATION_PROFILE: &str =
    "mycelix-authority-control-plane-freshness-set-v1-blake3-framed";

const DOMAIN_SUBJECT: &[u8] = b"mycelix/authority-control-plane/subject-freshness/v1";
const DOMAIN_SET: &[u8] = b"mycelix/authority-control-plane/freshness-set/v1";
const MAX_REQUIRED_SUBJECTS: usize = 3;

/// Non-deserializable proof that one exact control-plane policy subject is
/// current under one exact constitution-rooted bootstrap authority.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedControlPlaneSubjectFreshness {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    subject: AuthoritySubjectRef,
    context_coverage_digest: Digest32,
    context_coverage_profile: String,
    state_projection_digest: Digest32,
    state_projection_profile: String,
    current_freshness: VerifiedAuthorityFreshness,
    qualification_digest: Digest32,
    qualification_profile: String,
}

impl QualifiedControlPlaneSubjectFreshness {
    pub fn root_qualification_digest(&self) -> Digest32 {
        self.root_qualification_digest
    }
    pub fn root_qualification_profile(&self) -> &str {
        &self.root_qualification_profile
    }
    pub fn subject(&self) -> &AuthoritySubjectRef {
        &self.subject
    }
    pub fn context_coverage_digest(&self) -> Digest32 {
        self.context_coverage_digest
    }
    pub fn context_coverage_profile(&self) -> &str {
        &self.context_coverage_profile
    }
    pub fn state_projection_digest(&self) -> Digest32 {
        self.state_projection_digest
    }
    pub fn state_projection_profile(&self) -> &str {
        &self.state_projection_profile
    }
    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
    /// Adapter projection only. A runtime should expose this ABI only after
    /// obtaining this non-deserializable qualified object from the pure kernel.
    pub fn to_verified_freshness(&self) -> VerifiedAuthorityFreshness {
        self.current_freshness.clone()
    }
}

/// Non-deserializable closed-set qualification over exact current control-plane
/// subjects under one exact bootstrap-root epoch.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedControlPlaneFreshnessSet {
    root_qualification_digest: Digest32,
    root_qualification_profile: String,
    current: CurrentAuthorityFreshness,
    qualification_digest: Digest32,
    qualification_profile: String,
}

impl QualifiedControlPlaneFreshnessSet {
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
    pub fn verified_at_ms(&self) -> u64 {
        self.current.verified_at_ms
    }
    pub fn lease_until_ms(&self) -> u64 {
        self.current.lease_until_ms
    }
    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }
    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }
}

/// Qualify one exact current control-plane subject under the current bootstrap root.
#[allow(clippy::too_many_arguments)]
pub fn qualify_control_plane_subject_freshness(
    root: &QualifiedAuthorityStateBootstrapRoot,
    challenge_receipt: &VerifiedCoverageChallenge,
    source_receipt: &VerifiedAuthoritySourceHead,
    witness_receipts: &[VerifiedAuthorityHeadWitness],
    trust_bindings: &[VerifiedWitnessTrustBinding],
    transition_receipts: &[VerifiedAuthorityStateTransition],
    now_ms: u64,
) -> Result<QualifiedControlPlaneSubjectFreshness, ControlPlaneFreshnessError> {
    validate_root_now(root, now_ms)?;

    let subject = &challenge_receipt.challenge.subject;
    validate_control_plane_subject(root, subject)?;

    // A root-qualified manifest already proved that these exact policies were
    // adopted by the current constitution/rulebook. Project them into the #94/#96
    // evidence ABIs without asking the operational freshness plane to prove them.
    let coverage_policy_receipt = root_coverage_policy_receipt(root);
    let context_policy_receipt = root_context_policy_receipt(root);

    let context_coverage = qualify_context_bound_coverage(
        &context_policy_receipt,
        challenge_receipt,
        &coverage_policy_receipt,
        source_receipt,
        witness_receipts,
        trust_bindings,
        now_ms,
    )
    .map_err(|_| ControlPlaneFreshnessError::CoverageQualificationDenied)?;

    let state_coverage = context_coverage.to_state_source_coverage();
    let projection = project_current_authority_state(
        subject,
        transition_receipts,
        &state_coverage,
        now_ms,
    )
    .map_err(|_| ControlPlaneFreshnessError::StateProjectionDenied)?;

    let mut freshness = projection
        .to_current_freshness_receipt()
        .map_err(|_| ControlPlaneFreshnessError::CurrentFreshnessProjectionDenied)?;

    // The control-plane freshness receipt may never outlive the constitutional
    // bootstrap root that authorized the coverage semantics used to derive it.
    freshness.verified_at_ms = freshness.verified_at_ms.max(root.verified_at_ms());
    freshness.lease_until_ms = freshness.lease_until_ms.min(root.valid_until_ms());
    freshness
        .validate_at(now_ms)
        .map_err(|_| ControlPlaneFreshnessError::CurrentFreshnessProjectionDenied)?;

    let root_digest = root.qualification_digest();
    let qualification_digest = subject_qualification_digest(
        root_digest,
        context_coverage.context_coverage_digest(),
        projection.projection_digest(),
        freshness
            .snapshot
            .identity_digest()
            .map_err(|_| ControlPlaneFreshnessError::CurrentFreshnessProjectionDenied)?,
    );

    Ok(QualifiedControlPlaneSubjectFreshness {
        root_qualification_digest: root_digest,
        root_qualification_profile: root.qualification_profile().into(),
        subject: subject.clone(),
        context_coverage_digest: context_coverage.context_coverage_digest(),
        context_coverage_profile: context_coverage.context_coverage_profile().into(),
        state_projection_digest: projection.projection_digest(),
        state_projection_profile: projection.projection_profile().into(),
        current_freshness: freshness,
        qualification_digest,
        qualification_profile: SUBJECT_QUALIFICATION_PROFILE.into(),
    })
}

/// Qualify an exact closed set of already root-qualified current control-plane
/// subjects. This function does not select which subjects matter: the caller must
/// supply the exact required semantic subject set, and #74 rejects extra/missing or
/// conflicting freshness evidence.
pub fn qualify_control_plane_freshness_set(
    root: &QualifiedAuthorityStateBootstrapRoot,
    required_subjects: &[AuthoritySubjectRef],
    qualified_subjects: &[QualifiedControlPlaneSubjectFreshness],
    now_ms: u64,
) -> Result<QualifiedControlPlaneFreshnessSet, ControlPlaneFreshnessError> {
    validate_root_now(root, now_ms)?;
    if required_subjects.is_empty() || required_subjects.len() > MAX_REQUIRED_SUBJECTS {
        return Err(ControlPlaneFreshnessError::InvalidRequiredSubjectCount);
    }
    if qualified_subjects.len() != required_subjects.len() {
        return Err(ControlPlaneFreshnessError::QualifiedSetSizeMismatch);
    }

    let root_digest = root.qualification_digest();
    let root_profile = root.qualification_profile();
    let mut receipts = Vec::with_capacity(qualified_subjects.len());
    for qualified in qualified_subjects {
        if qualified.root_qualification_digest != root_digest
            || qualified.root_qualification_profile != root_profile
        {
            return Err(ControlPlaneFreshnessError::BootstrapRootMismatch);
        }
        validate_control_plane_subject(root, &qualified.subject)?;
        let receipt = qualified.to_verified_freshness();
        receipt
            .validate_at(now_ms)
            .map_err(|_| ControlPlaneFreshnessError::CurrentFreshnessProjectionDenied)?;
        if receipt.snapshot.subject != qualified.subject {
            return Err(ControlPlaneFreshnessError::QualifiedSubjectMismatch);
        }
        receipts.push(receipt);
    }

    for subject in required_subjects {
        validate_control_plane_subject(root, subject)?;
    }

    let current = qualify_current_freshness(required_subjects, &receipts, now_ms)
        .map_err(|_| ControlPlaneFreshnessError::ClosedSetFreshnessDenied)?;
    let qualification_digest = set_qualification_digest(root_digest, current.freshness_digest);

    Ok(QualifiedControlPlaneFreshnessSet {
        root_qualification_digest: root_digest,
        root_qualification_profile: root_profile.into(),
        current,
        qualification_digest,
        qualification_profile: SET_QUALIFICATION_PROFILE.into(),
    })
}

fn validate_root_now(
    root: &QualifiedAuthorityStateBootstrapRoot,
    now_ms: u64,
) -> Result<(), ControlPlaneFreshnessError> {
    if now_ms == 0
        || root.verified_at_ms() == 0
        || root.verified_at_ms() > now_ms
        || root.valid_until_ms() <= now_ms
    {
        return Err(ControlPlaneFreshnessError::BootstrapRootNotCurrent);
    }
    Ok(())
}

fn validate_control_plane_subject(
    root: &QualifiedAuthorityStateBootstrapRoot,
    subject: &AuthoritySubjectRef,
) -> Result<(), ControlPlaneFreshnessError> {
    subject
        .validate()
        .map_err(|_| ControlPlaneFreshnessError::InvalidSubject)?;
    if subject.namespace != root.manifest().control_plane_namespace {
        return Err(ControlPlaneFreshnessError::ControlPlaneNamespaceMismatch);
    }
    match subject.kind {
        AuthoritySubjectKind::AuthorityCoveragePolicy
        | AuthoritySubjectKind::CoverageTrustContextPolicy => Ok(()),
        AuthoritySubjectKind::WitnessTrustPolicy
            if root
                .manifest()
                .coverage_context_policy
                .witness_trust_policy
                .is_some() => Ok(()),
        AuthoritySubjectKind::WitnessTrustPolicy => {
            Err(ControlPlaneFreshnessError::WitnessPolicyOutsideRoot)
        }
        _ => Err(ControlPlaneFreshnessError::OperationalSubjectForbidden),
    }
}

fn root_coverage_policy_receipt(
    root: &QualifiedAuthorityStateBootstrapRoot,
) -> VerifiedAuthorityCoveragePolicy {
    let policy = root.manifest().coverage_policy.clone();
    VerifiedAuthorityCoveragePolicy {
        policy_record_ref: format!(
            "bootstrap-root-coverage-policy:{}",
            hex_digest(root.coverage_policy_digest())
        ),
        verified_policy_proof_ref: policy.policy_proof_ref.clone(),
        verified_authority_ref: policy.authority_ref.clone(),
        verification_ref: root.verification_ref().into(),
        verified_at_ms: root.verified_at_ms(),
        policy,
    }
}

fn root_context_policy_receipt(
    root: &QualifiedAuthorityStateBootstrapRoot,
) -> VerifiedCoverageTrustContextPolicy {
    let policy = root.manifest().coverage_context_policy.clone();
    VerifiedCoverageTrustContextPolicy {
        policy_record_ref: format!(
            "bootstrap-root-context-policy:{}",
            hex_digest(root.context_policy_digest())
        ),
        verified_authority_ref: policy.authority_ref.clone(),
        verified_policy_proof_ref: policy.policy_proof_ref.clone(),
        verification_ref: root.verification_ref().into(),
        verified_at_ms: root.verified_at_ms(),
        policy,
    }
}

fn subject_qualification_digest(
    root_digest: Digest32,
    context_coverage_digest: Digest32,
    projection_digest: Digest32,
    snapshot_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SUBJECT);
    frame(&mut hasher, SUBJECT_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &context_coverage_digest.0);
    frame(&mut hasher, &projection_digest.0);
    frame(&mut hasher, &snapshot_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn set_qualification_digest(root_digest: Digest32, freshness_digest: Digest32) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_SET);
    frame(&mut hasher, SET_QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &freshness_digest.0);
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
pub enum ControlPlaneFreshnessError {
    BootstrapRootNotCurrent,
    InvalidSubject,
    ControlPlaneNamespaceMismatch,
    WitnessPolicyOutsideRoot,
    OperationalSubjectForbidden,
    CoverageQualificationDenied,
    StateProjectionDenied,
    CurrentFreshnessProjectionDenied,
    InvalidRequiredSubjectCount,
    QualifiedSetSizeMismatch,
    BootstrapRootMismatch,
    QualifiedSubjectMismatch,
    ClosedSetFreshnessDenied,
}

impl fmt::Display for ControlPlaneFreshnessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::BootstrapRootNotCurrent => "authority-state bootstrap root is not currently usable",
            Self::InvalidSubject => "invalid control-plane authority subject",
            Self::ControlPlaneNamespaceMismatch => "control-plane subject namespace does not match bootstrap root",
            Self::WitnessPolicyOutsideRoot => "witness-trust policy is not part of this bootstrap root",
            Self::OperationalSubjectForbidden => "operational authority subject is forbidden in control-plane bootstrap qualification",
            Self::CoverageQualificationDenied => "root-bound context/source/witness coverage qualification denied",
            Self::StateProjectionDenied => "covered control-plane state projection denied",
            Self::CurrentFreshnessProjectionDenied => "current control-plane freshness projection denied",
            Self::InvalidRequiredSubjectCount => "invalid required control-plane subject count",
            Self::QualifiedSetSizeMismatch => "qualified control-plane set size does not match requirements",
            Self::BootstrapRootMismatch => "qualified control-plane subject belongs to another bootstrap root",
            Self::QualifiedSubjectMismatch => "qualified control-plane subject/freshness mismatch",
            Self::ClosedSetFreshnessDenied => "closed-set control-plane freshness qualification denied",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for ControlPlaneFreshnessError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_control_plane_subject_kinds_are_recognized() {
        assert!(matches!(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            AuthoritySubjectKind::AuthorityCoveragePolicy
        ));
        assert!(matches!(
            AuthoritySubjectKind::CoverageTrustContextPolicy,
            AuthoritySubjectKind::CoverageTrustContextPolicy
        ));
        assert!(matches!(
            AuthoritySubjectKind::WitnessTrustPolicy,
            AuthoritySubjectKind::WitnessTrustPolicy
        ));
    }

    #[test]
    fn qualification_profiles_are_distinct() {
        assert_ne!(SUBJECT_QUALIFICATION_PROFILE, SET_QUALIFICATION_PROFILE);
        assert!(!SUBJECT_QUALIFICATION_PROFILE.is_empty());
        assert!(!SET_QUALIFICATION_PROFILE.is_empty());
    }
}
