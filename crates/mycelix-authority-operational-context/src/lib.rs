// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Current operational authority-policy context.
//!
//! The authority-state bootstrap root and control-plane freshness layer establish
//! whether policy objects are current. This crate joins those currentness proofs
//! to the exact semantic coverage/context policies for one exact operational
//! authority subject.
//!
//! It deliberately does not issue probes, inspect DHT state, project transition
//! history, verify signatures, persist state, grant execution authority, or
//! interpret probe authorship as institutional authority.

use mycelix_authority_control_plane_freshness::QualifiedControlPlaneSubjectFreshness;
use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef, CurrentAuthorityFreshness,
    ProfiledDigest, VerifiedAuthorityFreshness,
};
use mycelix_authority_state_bootstrap_root::QualifiedAuthorityStateBootstrapRoot;
use mycelix_authority_state_coverage::{
    AuthorityCoveragePolicy, CoverageMode, VerifiedAuthorityCoveragePolicy,
    POLICY_IDENTITY_PROFILE,
};
use mycelix_authority_state_coverage_context::{
    CoverageTrustContextPolicy, VerifiedCoverageTrustContextPolicy, CONTEXT_POLICY_PROFILE,
};
use mycelix_institutional_core::Digest32;
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-operational-context-v0.1";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-operational-context-v1-blake3-framed";

const DOMAIN_QUALIFICATION: &[u8] = b"mycelix/authority/operational-context/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_POLICY_SUBJECTS: usize = 3;

/// Non-deserializable proof that one exact semantic operational policy context is
/// current for one exact operational authority subject.
///
/// Probe authorship is intentionally absent. A probe author supplies randomness
/// provenance; it does not become an institutional authority holder by doing so.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedOperationalPolicyContext {
    protocol_version: String,
    target_subject: AuthoritySubjectRef,
    bootstrap_root_digest: Digest32,
    bootstrap_root_profile: String,
    control_plane_namespace: String,
    coverage_receipt: VerifiedAuthorityCoveragePolicy,
    context_receipt: VerifiedCoverageTrustContextPolicy,
    coverage_policy_digest: Digest32,
    context_policy_digest: Digest32,
    witness_trust_policy: Option<ProfiledDigest>,
    policy_subjects: Vec<AuthoritySubjectRef>,
    current_policies: CurrentAuthorityFreshness,
    qualification_digest: Digest32,
    qualification_profile: String,
    verification_ref: String,
    verified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedOperationalPolicyContext {
    pub fn protocol_version(&self) -> &str {
        &self.protocol_version
    }

    pub fn target_subject(&self) -> &AuthoritySubjectRef {
        &self.target_subject
    }

    pub fn bootstrap_root_digest(&self) -> Digest32 {
        self.bootstrap_root_digest
    }

    pub fn bootstrap_root_profile(&self) -> &str {
        &self.bootstrap_root_profile
    }

    pub fn control_plane_namespace(&self) -> &str {
        &self.control_plane_namespace
    }

    pub fn coverage_receipt(&self) -> &VerifiedAuthorityCoveragePolicy {
        &self.coverage_receipt
    }

    pub fn context_receipt(&self) -> &VerifiedCoverageTrustContextPolicy {
        &self.context_receipt
    }

    pub fn coverage_policy_digest(&self) -> Digest32 {
        self.coverage_policy_digest
    }

    pub fn context_policy_digest(&self) -> Digest32 {
        self.context_policy_digest
    }

    pub fn witness_trust_policy(&self) -> Option<&ProfiledDigest> {
        self.witness_trust_policy.as_ref()
    }

    pub fn policy_subjects(&self) -> &[AuthoritySubjectRef] {
        &self.policy_subjects
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.current_policies.freshness_digest
    }

    pub fn freshness_profile(&self) -> &str {
        &self.current_policies.freshness_profile
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn verification_ref(&self) -> &str {
        &self.verification_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn validate_current_at(&self, now_ms: u64) -> Result<(), OperationalContextError> {
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.current_policies.lease_until_ms <= now_ms
        {
            return Err(OperationalContextError::ContextNotCurrentlyUsable);
        }
        Ok(())
    }
}

/// Qualify the exact current operational coverage/context policy set for one
/// target authority subject.
///
/// `policy_freshness` must contain the exact #115 control-plane freshness proofs
/// for the semantic policies supplied here. Neither the semantic receipts nor the
/// freshness provider gets to choose which policy subjects are required.
pub fn qualify_operational_policy_context(
    root: &QualifiedAuthorityStateBootstrapRoot,
    target_subject: &AuthoritySubjectRef,
    context_receipt: &VerifiedCoverageTrustContextPolicy,
    coverage_receipt: &VerifiedAuthorityCoveragePolicy,
    policy_freshness: &[QualifiedControlPlaneSubjectFreshness],
    now_ms: u64,
) -> Result<QualifiedOperationalPolicyContext, OperationalContextError> {
    validate_root(root, now_ms)?;
    validate_operational_subject(target_subject)?;

    let context = verify_context_receipt(context_receipt, root, now_ms)?;
    let coverage = verify_coverage_receipt(coverage_receipt, now_ms)?;
    let coverage_digest = coverage
        .identity_digest()
        .map_err(|_| OperationalContextError::InvalidCoveragePolicy)?;
    let context_digest = context
        .identity_digest()
        .map_err(|_| OperationalContextError::InvalidContextPolicy)?;

    if context.coverage_policy.digest != coverage_digest
        || context.coverage_policy.profile != POLICY_IDENTITY_PROFILE
    {
        return Err(OperationalContextError::CoveragePolicyBindingMismatch);
    }
    if coverage.namespace != target_subject.namespace
        || !coverage
            .allowed_subject_kinds
            .iter()
            .any(|kind| kind == &target_subject.kind)
    {
        return Err(OperationalContextError::TargetOutsideCoveragePolicy);
    }

    let expected_subjects = expected_policy_subjects(
        root,
        coverage,
        coverage_digest,
        context,
        context_digest,
    )?;
    if expected_subjects.len() > MAX_POLICY_SUBJECTS
        || policy_freshness.len() != expected_subjects.len()
    {
        return Err(OperationalContextError::PolicyFreshnessSetSizeMismatch);
    }

    let root_digest = root.qualification_digest();
    let root_profile = root.qualification_profile();
    let mut freshness_receipts = Vec::<VerifiedAuthorityFreshness>::with_capacity(policy_freshness.len());
    for qualified in policy_freshness {
        if qualified.root_qualification_digest() != root_digest
            || qualified.root_qualification_profile() != root_profile
        {
            return Err(OperationalContextError::BootstrapRootMismatch);
        }
        let receipt = qualified.to_verified_freshness();
        receipt
            .validate_at(now_ms)
            .map_err(|_| OperationalContextError::PolicyFreshnessDenied)?;
        if &receipt.snapshot.subject != qualified.subject() {
            return Err(OperationalContextError::PolicyFreshnessSubjectMismatch);
        }
        freshness_receipts.push(receipt);
    }

    let current_policies = qualify_current_freshness(
        &expected_subjects,
        &freshness_receipts,
        now_ms,
    )
    .map_err(|_| OperationalContextError::PolicyFreshnessDenied)?;

    let verified_at_ms = current_policies
        .verified_at_ms
        .max(root.verified_at_ms())
        .max(context_receipt.verified_at_ms)
        .max(coverage_receipt.verified_at_ms);
    let valid_until_ms = current_policies
        .lease_until_ms
        .min(root.valid_until_ms())
        .min(context.valid_until_ms)
        .min(coverage.valid_until_ms);
    if verified_at_ms > now_ms || valid_until_ms <= now_ms {
        return Err(OperationalContextError::ContextNotCurrentlyUsable);
    }

    let target_digest = target_subject
        .identity_digest()
        .map_err(|_| OperationalContextError::InvalidTargetSubject)?;
    let qualification_digest = qualification_digest(
        root_digest,
        target_digest,
        coverage_digest,
        context_digest,
        context.witness_trust_policy.as_ref(),
        current_policies.freshness_digest,
    );
    let verification_ref = format!(
        "operational-policy-context:{QUALIFICATION_PROFILE}:{}",
        hex_digest(qualification_digest)
    );

    Ok(QualifiedOperationalPolicyContext {
        protocol_version: PROTOCOL_VERSION.into(),
        target_subject: target_subject.clone(),
        bootstrap_root_digest: root_digest,
        bootstrap_root_profile: root_profile.into(),
        control_plane_namespace: root.manifest().control_plane_namespace.clone(),
        coverage_receipt: coverage_receipt.clone(),
        context_receipt: context_receipt.clone(),
        coverage_policy_digest: coverage_digest,
        context_policy_digest: context_digest,
        witness_trust_policy: context.witness_trust_policy.clone(),
        policy_subjects: expected_subjects,
        current_policies,
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        verification_ref,
        verified_at_ms,
        valid_until_ms,
    })
}

fn validate_root(
    root: &QualifiedAuthorityStateBootstrapRoot,
    now_ms: u64,
) -> Result<(), OperationalContextError> {
    if now_ms == 0
        || root.verified_at_ms() == 0
        || root.verified_at_ms() > now_ms
        || root.valid_until_ms() <= now_ms
    {
        return Err(OperationalContextError::BootstrapRootNotCurrent);
    }
    Ok(())
}

fn validate_operational_subject(subject: &AuthoritySubjectRef) -> Result<(), OperationalContextError> {
    subject
        .validate()
        .map_err(|_| OperationalContextError::InvalidTargetSubject)?;
    match subject.kind {
        AuthoritySubjectKind::AuthorityGrant
        | AuthoritySubjectKind::SigningPolicy
        | AuthoritySubjectKind::ThresholdAuthorization
        | AuthoritySubjectKind::ExecutorDesignation
        | AuthoritySubjectKind::EffectSafetyPolicy
        | AuthoritySubjectKind::Delegation => Ok(()),
        AuthoritySubjectKind::AuthorityCoveragePolicy
        | AuthoritySubjectKind::CoverageTrustContextPolicy
        | AuthoritySubjectKind::WitnessTrustPolicy => {
            Err(OperationalContextError::ControlPlaneTargetForbidden)
        }
    }
}

fn verify_context_receipt<'a>(
    receipt: &'a VerifiedCoverageTrustContextPolicy,
    root: &QualifiedAuthorityStateBootstrapRoot,
    now_ms: u64,
) -> Result<&'a CoverageTrustContextPolicy, OperationalContextError> {
    let policy = &receipt.policy;
    policy
        .validate()
        .map_err(|_| OperationalContextError::InvalidContextPolicy)?;
    if !policy.active_at(now_ms)
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < policy.valid_from_ms
        || receipt.verified_authority_ref != policy.authority_ref
        || receipt.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(OperationalContextError::ContextPolicyVerificationMismatch);
    }
    require_ref(&receipt.policy_record_ref)?;
    require_ref(&receipt.verification_ref)?;

    if policy.institution_ref != root.manifest().institution_id
        || policy.rulebook_ref != root.manifest().rulebook_ref
    {
        return Err(OperationalContextError::InstitutionRulebookMismatch);
    }
    Ok(policy)
}

fn verify_coverage_receipt<'a>(
    receipt: &'a VerifiedAuthorityCoveragePolicy,
    now_ms: u64,
) -> Result<&'a AuthorityCoveragePolicy, OperationalContextError> {
    let policy = &receipt.policy;
    policy
        .validate()
        .map_err(|_| OperationalContextError::InvalidCoveragePolicy)?;
    if !policy.is_active_at(now_ms)
        || receipt.verified_at_ms == 0
        || receipt.verified_at_ms > now_ms
        || receipt.verified_at_ms < policy.valid_from_ms
        || receipt.verified_authority_ref != policy.authority_ref
        || receipt.verified_policy_proof_ref != policy.policy_proof_ref
    {
        return Err(OperationalContextError::CoveragePolicyVerificationMismatch);
    }
    require_ref(&receipt.policy_record_ref)?;
    require_ref(&receipt.verification_ref)?;
    Ok(policy)
}

fn expected_policy_subjects(
    root: &QualifiedAuthorityStateBootstrapRoot,
    coverage: &AuthorityCoveragePolicy,
    coverage_digest: Digest32,
    context: &CoverageTrustContextPolicy,
    context_digest: Digest32,
) -> Result<Vec<AuthoritySubjectRef>, OperationalContextError> {
    let namespace = root.manifest().control_plane_namespace.as_str();
    let mut expected = vec![
        policy_subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            namespace,
            &coverage.policy_id,
            coverage_digest,
            POLICY_IDENTITY_PROFILE,
        )?,
        policy_subject(
            AuthoritySubjectKind::CoverageTrustContextPolicy,
            namespace,
            &context.context_policy_id,
            context_digest,
            CONTEXT_POLICY_PROFILE,
        )?,
    ];

    match (
        &coverage.mode,
        &context.witness_trust_policy,
        &context.witness_trust_verifier_ref,
    ) {
        (CoverageMode::DirectSource, None, None) => {}
        (CoverageMode::DirectSource, _, _) => {
            return Err(OperationalContextError::UnexpectedWitnessTrustPolicy);
        }
        (CoverageMode::WitnessQuorum { .. }, Some(policy), Some(verifier_ref)) => {
            require_ref(verifier_ref)?;
            expected.push(policy_subject(
                AuthoritySubjectKind::WitnessTrustPolicy,
                namespace,
                &witness_policy_subject_id(policy),
                policy.digest,
                &policy.profile,
            )?);
        }
        (CoverageMode::WitnessQuorum { .. }, _, _) => {
            return Err(OperationalContextError::MissingWitnessTrustPolicy);
        }
    }

    Ok(expected)
}

fn policy_subject(
    kind: AuthoritySubjectKind,
    registry_namespace: &str,
    subject_id: &str,
    digest: Digest32,
    profile: &str,
) -> Result<AuthoritySubjectRef, OperationalContextError> {
    let subject = AuthoritySubjectRef {
        kind,
        namespace: registry_namespace.into(),
        subject_id: subject_id.into(),
        identity: ProfiledDigest {
            digest,
            profile: profile.into(),
        },
    };
    subject
        .validate()
        .map_err(|_| OperationalContextError::InvalidPolicyFreshnessSubject)?;
    Ok(subject)
}

fn witness_policy_subject_id(policy: &ProfiledDigest) -> String {
    format!("witness-trust-policy:{}", hex_digest(policy.digest))
}

fn qualification_digest(
    root_digest: Digest32,
    target_digest: Digest32,
    coverage_digest: Digest32,
    context_digest: Digest32,
    witness_policy: Option<&ProfiledDigest>,
    current_policy_freshness_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_QUALIFICATION);
    frame(&mut hasher, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut hasher, &root_digest.0);
    frame(&mut hasher, &target_digest.0);
    frame(&mut hasher, &coverage_digest.0);
    frame(&mut hasher, &context_digest.0);
    match witness_policy {
        None => frame(&mut hasher, &[0]),
        Some(policy) => {
            frame(&mut hasher, &[1]);
            frame(&mut hasher, policy.profile.as_bytes());
            frame(&mut hasher, &policy.digest.0);
        }
    }
    frame(&mut hasher, &current_policy_freshness_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn require_ref(value: &str) -> Result<(), OperationalContextError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(OperationalContextError::InvalidReference)
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
    let mut output = String::with_capacity(64);
    for byte in digest.0 {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum OperationalContextError {
    BootstrapRootNotCurrent,
    InvalidTargetSubject,
    ControlPlaneTargetForbidden,
    InvalidContextPolicy,
    InvalidCoveragePolicy,
    ContextPolicyVerificationMismatch,
    CoveragePolicyVerificationMismatch,
    InstitutionRulebookMismatch,
    CoveragePolicyBindingMismatch,
    TargetOutsideCoveragePolicy,
    UnexpectedWitnessTrustPolicy,
    MissingWitnessTrustPolicy,
    InvalidPolicyFreshnessSubject,
    PolicyFreshnessSetSizeMismatch,
    BootstrapRootMismatch,
    PolicyFreshnessSubjectMismatch,
    PolicyFreshnessDenied,
    ContextNotCurrentlyUsable,
    InvalidReference,
}

impl fmt::Display for OperationalContextError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::BootstrapRootNotCurrent => "authority-state bootstrap root is not current",
            Self::InvalidTargetSubject => "invalid operational authority target subject",
            Self::ControlPlaneTargetForbidden => "control-plane policy subjects require bootstrap qualification, not operational context",
            Self::InvalidContextPolicy => "invalid operational coverage trust-context policy",
            Self::InvalidCoveragePolicy => "invalid operational authority coverage policy",
            Self::ContextPolicyVerificationMismatch => "verified context policy receipt does not exactly match semantic policy authority",
            Self::CoveragePolicyVerificationMismatch => "verified coverage policy receipt does not exactly match semantic policy authority",
            Self::InstitutionRulebookMismatch => "operational context belongs to another institution or rulebook",
            Self::CoveragePolicyBindingMismatch => "context policy does not bind the exact coverage policy",
            Self::TargetOutsideCoveragePolicy => "target authority subject is outside the operational coverage policy",
            Self::UnexpectedWitnessTrustPolicy => "direct-source operational coverage forbids witness trust policy",
            Self::MissingWitnessTrustPolicy => "witness-quorum operational coverage requires witness trust policy and verifier",
            Self::InvalidPolicyFreshnessSubject => "invalid control-plane freshness subject for operational policy",
            Self::PolicyFreshnessSetSizeMismatch => "current policy freshness set size does not match exact semantic requirements",
            Self::BootstrapRootMismatch => "policy freshness proof belongs to another bootstrap-root epoch",
            Self::PolicyFreshnessSubjectMismatch => "policy freshness proof subject does not match its qualified subject",
            Self::PolicyFreshnessDenied => "exact current operational policy freshness qualification denied",
            Self::ContextNotCurrentlyUsable => "operational policy context is stale or outside its current lease",
            Self::InvalidReference => "invalid operational policy context reference",
        };
        write!(f, "{message}")
    }
}

impl std::error::Error for OperationalContextError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn target(kind: AuthoritySubjectKind) -> AuthoritySubjectRef {
        AuthoritySubjectRef {
            kind,
            namespace: "institution:test:authority".into(),
            subject_id: "target:test".into(),
            identity: ProfiledDigest {
                digest: d(1),
                profile: "target-v1-blake3".into(),
            },
        }
    }

    #[test]
    fn policy_registry_namespace_is_not_covered_namespace() {
        let subject = policy_subject(
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            "authority-control:test",
            "coverage-policy:test",
            d(2),
            POLICY_IDENTITY_PROFILE,
        )
        .unwrap();
        assert_eq!(subject.namespace, "authority-control:test");
        assert_ne!(subject.namespace, "institution:test:authority");
    }

    #[test]
    fn operational_subjects_exclude_control_plane_policy_classes() {
        for kind in [
            AuthoritySubjectKind::AuthorityGrant,
            AuthoritySubjectKind::SigningPolicy,
            AuthoritySubjectKind::ThresholdAuthorization,
            AuthoritySubjectKind::ExecutorDesignation,
            AuthoritySubjectKind::EffectSafetyPolicy,
            AuthoritySubjectKind::Delegation,
        ] {
            assert!(validate_operational_subject(&target(kind)).is_ok());
        }
        for kind in [
            AuthoritySubjectKind::AuthorityCoveragePolicy,
            AuthoritySubjectKind::CoverageTrustContextPolicy,
            AuthoritySubjectKind::WitnessTrustPolicy,
        ] {
            assert_eq!(
                validate_operational_subject(&target(kind)).unwrap_err(),
                OperationalContextError::ControlPlaneTargetForbidden
            );
        }
    }

    #[test]
    fn witness_policy_subject_is_bound_to_exact_digest_and_registry_namespace() {
        let policy = ProfiledDigest {
            digest: d(9),
            profile: "witness-policy-v1-blake3".into(),
        };
        let subject = policy_subject(
            AuthoritySubjectKind::WitnessTrustPolicy,
            "authority-control:test",
            &witness_policy_subject_id(&policy),
            policy.digest,
            &policy.profile,
        )
        .unwrap();
        assert_eq!(subject.namespace, "authority-control:test");
        assert_eq!(subject.identity, policy);
        assert!(subject.subject_id.ends_with(&hex_digest(d(9))));
    }

    #[test]
    fn qualification_identity_changes_with_target_subject() {
        let a = target(AuthoritySubjectKind::AuthorityGrant)
            .identity_digest()
            .unwrap();
        let b = target(AuthoritySubjectKind::Delegation)
            .identity_digest()
            .unwrap();
        assert_ne!(a, b);
        let qa = qualification_digest(d(2), a, d(3), d(4), None, d(5));
        let qb = qualification_digest(d(2), b, d(3), d(4), None, d(5));
        assert_ne!(qa, qb);
    }
}
