// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Generation-bound current executor authority for Mycelix governance.
//!
//! Semantic executor qualification and current revocation state are separate facts.
//! This crate re-runs the exact lineage-bound executor qualifier, derives stable
//! identities for the exact grant and threshold authorization used by that call,
//! and then requires one closed set of current generation-bound freshness facts.
//! No caller-provided `current=true`, timestamp ordering, or legacy delegation
//! string can substitute for those exact current authority subjects.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef, FreshnessError,
    ProfiledDigest as FreshnessProfiledDigest, VerifiedAuthorityFreshness,
    BUNDLE_IDENTITY_PROFILE,
};
use mycelix_authority_identity::{
    authority_grant_identity, AuthorityIdentityError, AUTHORITY_GRANT_IDENTITY_PROFILE,
};
use mycelix_governance_authority::{ProposalId, SignatureAlgorithm};
use mycelix_governance_executor_designation::{
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_governance_executor_lineage::{
    qualify_lineage_bound_executor_authority, DelegationLineageEvidence, ExecutorLineageError,
    EXECUTOR_LINEAGE_AUTHORITY_PROFILE,
};
use mycelix_institutional_core::{
    AuthorityGrantId, Digest32, InstitutionId, PrincipalId, RulebookRef,
};
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-current-executor-authority-v0.1";
pub const THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE: &str =
    "mycelix-governance-threshold-authorization-v1-blake3-framed-semantic";
pub const CURRENT_EXECUTOR_AUTHORITY_PROFILE: &str =
    "mycelix-governance-current-executor-authority-v1-blake3-framed";

const DOMAIN_THRESHOLD: &[u8] = b"mycelix/governance/threshold-authorization/identity/v1";
const DOMAIN_CURRENT_EXECUTOR: &[u8] = b"mycelix/governance/current-executor-authority/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedCurrentExecutorAuthority {
    protocol_version: String,
    proposal_id: ProposalId,
    executor_principal: PrincipalId,
    authority_grant_id: AuthorityGrantId,
    authority_grant_identity_digest: Digest32,
    authority_grant_identity_profile: String,
    threshold_authorization_ref: String,
    threshold_authorization_identity_digest: Digest32,
    threshold_authorization_identity_profile: String,
    executor_semantic_authority_digest: Digest32,
    executor_semantic_authority_profile: String,
    institution: InstitutionId,
    rulebook: RulebookRef,
    freshness_digest: Digest32,
    freshness_profile: String,
    current_authority_digest: Digest32,
    current_authority_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedCurrentExecutorAuthority {
    pub fn proposal_id(&self) -> &ProposalId {
        &self.proposal_id
    }

    pub fn executor_principal(&self) -> &PrincipalId {
        &self.executor_principal
    }

    pub fn authority_grant_id(&self) -> &AuthorityGrantId {
        &self.authority_grant_id
    }

    pub fn authority_grant_identity_digest(&self) -> Digest32 {
        self.authority_grant_identity_digest
    }

    pub fn threshold_authorization_ref(&self) -> &str {
        &self.threshold_authorization_ref
    }

    pub fn threshold_authorization_identity_digest(&self) -> Digest32 {
        self.threshold_authorization_identity_digest
    }

    pub fn executor_semantic_authority_digest(&self) -> Digest32 {
        self.executor_semantic_authority_digest
    }

    pub fn executor_semantic_authority_profile(&self) -> &str {
        &self.executor_semantic_authority_profile
    }

    pub fn institution(&self) -> &InstitutionId {
        &self.institution
    }

    pub fn rulebook(&self) -> &RulebookRef {
        &self.rulebook
    }

    pub fn freshness_digest(&self) -> Digest32 {
        self.freshness_digest
    }

    pub fn freshness_profile(&self) -> &str {
        &self.freshness_profile
    }

    pub fn current_authority_digest(&self) -> Digest32 {
        self.current_authority_digest
    }

    pub fn current_authority_profile(&self) -> &str {
        &self.current_authority_profile
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

/// Reconstruct exact executor semantics and then qualify the exact current
/// revocation generations for the same authority objects.
pub fn qualify_current_executor_authority(
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    grant_freshness: &VerifiedAuthorityFreshness,
    threshold_freshness: &VerifiedAuthorityFreshness,
    executor_freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedCurrentExecutorAuthority, CurrentExecutorAuthorityError> {
    if now_ms == 0 {
        return Err(CurrentExecutorAuthorityError::InvalidVerificationTime);
    }

    // Re-run the full exact semantic qualifier. This ensures the current layer
    // cannot combine freshness for object A with a previously qualified executor
    // authority derived from object B.
    let executor = qualify_lineage_bound_executor_authority(
        threshold,
        grant_receipt,
        designation_receipt,
        lineage_evidence,
        now_ms,
    )
    .map_err(CurrentExecutorAuthorityError::ExecutorLineage)?;

    let grant = &grant_receipt.grant;
    let grant_identity =
        authority_grant_identity(grant).map_err(CurrentExecutorAuthorityError::GrantIdentity)?;
    if grant_identity.profile != AUTHORITY_GRANT_IDENTITY_PROFILE
        || executor.authority_grant_id() != &grant.id
        || executor.authority_grant_identity_digest() != grant_identity.digest
    {
        return Err(CurrentExecutorAuthorityError::GrantIdentityMismatch);
    }

    if executor.proposal_id() != &threshold.authorization.proposal_id
        || executor.executor_principal() != &designation_receipt.designation.executor
    {
        return Err(CurrentExecutorAuthorityError::ExecutorSemanticMismatch);
    }

    let threshold_identity = threshold_authorization_identity(threshold)?;
    let institution = threshold.authorization.institution.clone();
    let namespace = institution.as_str().to_string();

    let grant_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::AuthorityGrant,
        namespace: namespace.clone(),
        subject_id: grant.id.as_str().to_string(),
        identity: FreshnessProfiledDigest {
            digest: grant_identity.digest,
            profile: grant_identity.profile.clone(),
        },
    };
    let threshold_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::ThresholdAuthorization,
        namespace: namespace.clone(),
        subject_id: threshold.threshold_authorization_ref.clone(),
        identity: FreshnessProfiledDigest {
            digest: threshold_identity,
            profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.into(),
        },
    };
    let executor_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::ExecutorDesignation,
        namespace,
        subject_id: designation_receipt.designation_record_ref.clone(),
        identity: FreshnessProfiledDigest {
            digest: executor.authority_digest(),
            profile: executor.authority_profile().to_string(),
        },
    };

    if grant_freshness.snapshot.subject != grant_subject {
        return Err(CurrentExecutorAuthorityError::GrantFreshnessMismatch);
    }
    if threshold_freshness.snapshot.subject != threshold_subject {
        return Err(CurrentExecutorAuthorityError::ThresholdFreshnessMismatch);
    }
    if executor_freshness.snapshot.subject != executor_subject {
        return Err(CurrentExecutorAuthorityError::ExecutorFreshnessMismatch);
    }

    // Current-state facts cannot predate the immutable semantic object they
    // govern. A later effective time is valid and represents subsequent
    // activation/re-authorization of the same exact object generation.
    if grant_freshness.snapshot.effective_at_ms < grant.issued_at_ms {
        return Err(CurrentExecutorAuthorityError::FreshnessPredatesAuthority);
    }
    if threshold_freshness.snapshot.effective_at_ms < threshold.authorization.signed_at_ms {
        return Err(CurrentExecutorAuthorityError::FreshnessPredatesAuthority);
    }
    if executor_freshness.snapshot.effective_at_ms < designation_receipt.designation.issued_at_ms {
        return Err(CurrentExecutorAuthorityError::FreshnessPredatesAuthority);
    }

    let freshness = qualify_current_freshness(
        &[grant_subject, threshold_subject, executor_subject],
        &[
            grant_freshness.clone(),
            threshold_freshness.clone(),
            executor_freshness.clone(),
        ],
        now_ms,
    )
    .map_err(CurrentExecutorAuthorityError::Freshness)?;
    if freshness.freshness_profile != BUNDLE_IDENTITY_PROFILE {
        return Err(CurrentExecutorAuthorityError::FreshnessProfileMismatch);
    }

    let verified_at_ms = freshness.verified_at_ms.max(executor.verified_at_ms());
    let lease_until_ms = freshness.lease_until_ms.min(executor.lease_until_ms());
    if verified_at_ms > now_ms || lease_until_ms <= now_ms {
        return Err(CurrentExecutorAuthorityError::CurrentAuthorityExpired);
    }

    let current_authority_digest = current_executor_digest(
        grant_identity.digest,
        threshold_identity,
        executor.authority_digest(),
        freshness.freshness_digest,
    );

    Ok(QualifiedCurrentExecutorAuthority {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: threshold.authorization.proposal_id.clone(),
        executor_principal: designation_receipt.designation.executor.clone(),
        authority_grant_id: grant.id.clone(),
        authority_grant_identity_digest: grant_identity.digest,
        authority_grant_identity_profile: grant_identity.profile,
        threshold_authorization_ref: threshold.threshold_authorization_ref.clone(),
        threshold_authorization_identity_digest: threshold_identity,
        threshold_authorization_identity_profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.into(),
        executor_semantic_authority_digest: executor.authority_digest(),
        executor_semantic_authority_profile: executor.authority_profile().to_string(),
        institution,
        rulebook: threshold.authorization.rulebook.clone(),
        freshness_digest: freshness.freshness_digest,
        freshness_profile: freshness.freshness_profile,
        current_authority_digest,
        current_authority_profile: CURRENT_EXECUTOR_AUTHORITY_PROFILE.into(),
        verified_at_ms,
        lease_until_ms,
    })
}

/// Stable semantic identity of the exact threshold authorization consumed by
/// executor qualification. Dynamic verifier invocation refs/timestamps are
/// deliberately excluded; immutable policy/signature provenance remains bound.
pub fn threshold_authorization_identity(
    threshold: &VerifiedThresholdAuthorization,
) -> Result<Digest32, CurrentExecutorAuthorityError> {
    require_ref(
        &threshold.threshold_authorization_ref,
        "threshold authorization ref",
    )?;
    require_profile(&threshold.actions_digest_profile)?;
    let authority = &threshold.authorization;
    if authority.actions_digest.is_zero()
        || authority.signing_policy_digest.is_zero()
        || authority.committee_key_digest.is_zero()
        || authority.epoch == 0
        || authority.min_signers == 0
        || authority.member_count == 0
        || authority.signer_count == 0
        || authority.signed_at_ms == 0
        || authority.valid_until_ms <= authority.signed_at_ms
    {
        return Err(CurrentExecutorAuthorityError::InvalidThresholdIdentity);
    }
    for value in [
        authority.proposal_id.as_str(),
        authority.institution.as_str(),
        authority.governing_body.as_str(),
        authority.signing_policy_id.as_str(),
        authority.committee_id.as_str(),
        authority.signature_id.as_str(),
        authority.signature_ref.as_str(),
        authority.policy_record_ref.as_str(),
    ] {
        require_ref(value, "threshold identity field")?;
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_THRESHOLD);
    frame(
        &mut hasher,
        THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, authority.protocol_version.as_bytes());
    frame(&mut hasher, threshold.threshold_authorization_ref.as_bytes());
    frame(&mut hasher, authority.proposal_id.as_str().as_bytes());
    frame(&mut hasher, authority.institution.as_str().as_bytes());
    frame_optional_text(
        &mut hasher,
        authority.jurisdiction.as_ref().map(|value| value.as_str()),
    );
    frame(&mut hasher, authority.rulebook.id.as_str().as_bytes());
    frame(&mut hasher, authority.rulebook.version.as_bytes());
    frame(&mut hasher, &authority.rulebook.digest.0);
    frame(&mut hasher, authority.governing_body.as_str().as_bytes());
    frame(&mut hasher, authority.action_class.as_bytes());
    frame(&mut hasher, &authority.actions_digest.0);
    frame(&mut hasher, threshold.actions_digest_profile.as_bytes());
    frame(&mut hasher, authority.signing_policy_id.as_str().as_bytes());
    frame(&mut hasher, &authority.signing_policy_digest.0);
    frame(&mut hasher, authority.committee_id.as_str().as_bytes());
    frame(&mut hasher, &authority.committee_key_digest.0);
    frame(&mut hasher, &authority.epoch.to_le_bytes());
    frame(&mut hasher, &authority.min_signers.to_le_bytes());
    frame(&mut hasher, &authority.member_count.to_le_bytes());
    frame(&mut hasher, &authority.signer_count.to_le_bytes());
    frame(&mut hasher, &[signature_algorithm_code(&authority.algorithm)]);
    frame(&mut hasher, authority.signature_id.as_str().as_bytes());
    frame(&mut hasher, authority.signature_ref.as_bytes());
    frame(&mut hasher, authority.policy_record_ref.as_bytes());
    frame(&mut hasher, &authority.signed_at_ms.to_le_bytes());
    frame(&mut hasher, &authority.valid_until_ms.to_le_bytes());
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

fn current_executor_digest(
    grant_identity: Digest32,
    threshold_identity: Digest32,
    executor_semantic_identity: Digest32,
    freshness_digest: Digest32,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENT_EXECUTOR);
    frame(&mut hasher, CURRENT_EXECUTOR_AUTHORITY_PROFILE.as_bytes());
    frame(&mut hasher, AUTHORITY_GRANT_IDENTITY_PROFILE.as_bytes());
    frame(
        &mut hasher,
        THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, EXECUTOR_LINEAGE_AUTHORITY_PROFILE.as_bytes());
    frame(&mut hasher, BUNDLE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &grant_identity.0);
    frame(&mut hasher, &threshold_identity.0);
    frame(&mut hasher, &executor_semantic_identity.0);
    frame(&mut hasher, &freshness_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn signature_algorithm_code(algorithm: &SignatureAlgorithm) -> u8 {
    match algorithm {
        SignatureAlgorithm::EcdsaSecp256k1 => 1,
        SignatureAlgorithm::MlDsa65 => 2,
        SignatureAlgorithm::HybridEcdsaMlDsa65 => 3,
    }
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

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

fn require_ref(value: &str, _field: &str) -> Result<(), CurrentExecutorAuthorityError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(CurrentExecutorAuthorityError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), CurrentExecutorAuthorityError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(CurrentExecutorAuthorityError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum CurrentExecutorAuthorityError {
    InvalidVerificationTime,
    InvalidReference,
    InvalidProfile,
    InvalidThresholdIdentity,
    GrantIdentity(AuthorityIdentityError),
    GrantIdentityMismatch,
    ExecutorSemanticMismatch,
    ExecutorLineage(ExecutorLineageError),
    GrantFreshnessMismatch,
    ThresholdFreshnessMismatch,
    ExecutorFreshnessMismatch,
    FreshnessPredatesAuthority,
    Freshness(FreshnessError),
    FreshnessProfileMismatch,
    CurrentAuthorityExpired,
}

impl fmt::Display for CurrentExecutorAuthorityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVerificationTime => write!(f, "invalid current-executor verification time"),
            Self::InvalidReference => write!(f, "invalid current-executor reference"),
            Self::InvalidProfile => write!(f, "invalid current-executor digest profile"),
            Self::InvalidThresholdIdentity => write!(f, "invalid threshold-authorization semantic identity"),
            Self::GrantIdentity(error) => write!(f, "cannot canonicalize current executor grant: {error}"),
            Self::GrantIdentityMismatch => write!(f, "current executor grant does not match lineage-bound executor authority"),
            Self::ExecutorSemanticMismatch => write!(f, "executor semantics do not match exact threshold/designation inputs"),
            Self::ExecutorLineage(error) => write!(f, "lineage-bound executor qualification failed: {error}"),
            Self::GrantFreshnessMismatch => write!(f, "grant freshness does not bind the exact canonical executor grant"),
            Self::ThresholdFreshnessMismatch => write!(f, "threshold freshness does not bind the exact threshold authorization"),
            Self::ExecutorFreshnessMismatch => write!(f, "executor freshness does not bind the exact lineage-bound executor authority"),
            Self::FreshnessPredatesAuthority => write!(f, "current freshness state predates the immutable authority it governs"),
            Self::Freshness(error) => write!(f, "current executor freshness qualification failed: {error}"),
            Self::FreshnessProfileMismatch => write!(f, "unexpected current executor freshness profile"),
            Self::CurrentAuthorityExpired => write!(f, "current executor authority is stale or expired"),
        }
    }
}

impl std::error::Error for CurrentExecutorAuthorityError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn generation_bundle_changes_current_executor_identity() {
        let a = current_executor_digest(d(1), d(2), d(3), d(4));
        let b = current_executor_digest(d(1), d(2), d(3), d(5));
        assert_ne!(a, b);
    }

    #[test]
    fn semantic_executor_change_changes_current_executor_identity() {
        let a = current_executor_digest(d(1), d(2), d(3), d(4));
        let b = current_executor_digest(d(1), d(2), d(9), d(4));
        assert_ne!(a, b);
    }

    #[test]
    fn threshold_change_changes_current_executor_identity() {
        let a = current_executor_digest(d(1), d(2), d(3), d(4));
        let b = current_executor_digest(d(1), d(8), d(3), d(4));
        assert_ne!(a, b);
    }
}
