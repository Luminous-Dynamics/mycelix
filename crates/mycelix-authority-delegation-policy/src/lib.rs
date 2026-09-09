// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure institution-bound authority-to-delegate qualification.
//!
//! Possessing a capability is not the same as possessing authority to delegate
//! it. This crate qualifies one explicit delegation policy against one exact,
//! currently fresh parent AuthorityGrant and one exact, currently fresh policy
//! generation. A later delegation-lineage kernel consumes the resulting bounded
//! authority instead of trusting an opaque policy digest echo.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthorityFreshnessState, AuthoritySubjectKind,
    AuthoritySubjectRef, FreshnessError, ProfiledDigest as FreshnessProfiledDigest,
    VerifiedAuthorityFreshness, BUNDLE_IDENTITY_PROFILE,
};
use mycelix_authority_identity::{
    authority_grant_identity, AuthorityIdentityError, CanonicalAuthorityIdentity,
    AUTHORITY_GRANT_IDENTITY_PROFILE,
};
use mycelix_institutional_core::{
    AuthorityGrant, AuthorityGrantId, AuthoritySourceKind, AuthoritySourceRef, CapabilityId,
    Digest32, InstitutionId, JurisdictionId, PrincipalId, RoleId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-delegation-policy-v0.1";
pub const DELEGATION_POLICY_IDENTITY_PROFILE: &str =
    "mycelix-authority-delegation-policy-v1-blake3-framed-semantic";
pub const CURRENT_DELEGATION_AUTHORITY_PROFILE: &str =
    "mycelix-authority-delegation-current-v1-blake3-framed";

const DOMAIN_POLICY: &[u8] = b"mycelix/authority-delegation-policy/v1";
const DOMAIN_CURRENT_AUTHORITY: &[u8] = b"mycelix/authority-delegation-current/v1";
const MAX_ID_BYTES: usize = 512;
const MAX_REF_BYTES: usize = 2048;
const MAX_DELEGATES: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DelegateScope {
    /// Explicitly permits any otherwise-valid principal.
    AnyPrincipal,
    /// Only these exact principals may receive a child delegation.
    AllowList(Vec<PrincipalId>),
}

impl DelegateScope {
    pub fn validate(&self) -> Result<(), DelegationPolicyError> {
        match self {
            Self::AnyPrincipal => Ok(()),
            Self::AllowList(values) => {
                if values.is_empty() || values.len() > MAX_DELEGATES {
                    return Err(DelegationPolicyError::InvalidDelegateScope);
                }
                canonical_principals(values)?;
                Ok(())
            }
        }
    }

    fn allows(&self, principal: &PrincipalId) -> bool {
        match self {
            Self::AnyPrincipal => true,
            Self::AllowList(values) => values.iter().any(|value| value == principal),
        }
    }
}

/// Immutable policy defining which parts of one exact parent grant may be
/// delegated, to whom, for how long, and whether onward re-delegation is allowed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DelegationPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub parent_grant_id: AuthorityGrantId,
    pub parent_grant_digest: Digest32,
    pub parent_generation: u64,
    pub delegator: PrincipalId,
    pub delegable_roles: Vec<RoleId>,
    pub delegable_capabilities: Vec<CapabilityId>,
    pub delegate_scope: DelegateScope,
    pub allow_redelegation: bool,
    pub max_child_lifetime_ms: u64,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
    pub source: AuthoritySourceRef,
    pub policy_proof_ref: String,
}

impl DelegationPolicy {
    pub fn validate(&self) -> Result<(), DelegationPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(DelegationPolicyError::WrongProtocolVersion);
        }
        require_id(&self.policy_id)?;
        require_id(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_id(jurisdiction.as_str())?;
        }
        require_id(self.parent_grant_id.as_str())?;
        require_id(self.delegator.as_str())?;
        if self.parent_grant_digest.is_zero() {
            return Err(DelegationPolicyError::ZeroParentGrantDigest);
        }
        if self.parent_generation == 0 {
            return Err(DelegationPolicyError::ZeroParentGeneration);
        }
        if self.delegable_capabilities.is_empty() {
            return Err(DelegationPolicyError::NoDelegableCapabilities);
        }
        canonical_roles(&self.delegable_roles)?;
        canonical_capabilities(&self.delegable_capabilities)?;
        self.delegate_scope.validate()?;
        if self.max_child_lifetime_ms == 0 {
            return Err(DelegationPolicyError::ZeroChildLifetimeLimit);
        }
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(DelegationPolicyError::InvalidPolicyLifetime);
        }
        self.rulebook
            .validate()
            .map_err(|_| DelegationPolicyError::InvalidRulebook)?;
        self.source
            .validate()
            .map_err(|_| DelegationPolicyError::InvalidAuthoritySource)?;
        if matches!(&self.source.kind, AuthoritySourceKind::Delegation) {
            return Err(DelegationPolicyError::CircularDelegationPolicySource);
        }
        require_ref(&self.policy_proof_ref)
    }

    pub fn is_active_at(&self, now_ms: u64) -> bool {
        self.valid_from_ms <= now_ms && now_ms < self.valid_until_ms
    }

    pub fn identity_digest(&self) -> Result<Digest32, DelegationPolicyError> {
        self.validate()?;
        let roles = canonical_roles(&self.delegable_roles)?;
        let capabilities = canonical_capabilities(&self.delegable_capabilities)?;
        let delegates = canonical_delegate_scope(&self.delegate_scope)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_POLICY);
        frame(&mut hasher, DELEGATION_POLICY_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, AUTHORITY_GRANT_IDENTITY_PROFILE.as_bytes());
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
        frame(&mut hasher, self.parent_grant_id.as_str().as_bytes());
        frame(&mut hasher, &self.parent_grant_digest.0);
        frame(&mut hasher, &self.parent_generation.to_le_bytes());
        frame(&mut hasher, self.delegator.as_str().as_bytes());
        frame(&mut hasher, &(roles.len() as u64).to_le_bytes());
        for role in roles {
            frame(&mut hasher, role.as_bytes());
        }
        frame(&mut hasher, &(capabilities.len() as u64).to_le_bytes());
        for capability in capabilities {
            frame(&mut hasher, capability.as_bytes());
        }
        match delegates {
            CanonicalDelegateScope::Any => frame(&mut hasher, &[0]),
            CanonicalDelegateScope::AllowList(values) => {
                frame(&mut hasher, &[1]);
                frame(&mut hasher, &(values.len() as u64).to_le_bytes());
                for value in values {
                    frame(&mut hasher, value.as_bytes());
                }
            }
        }
        frame(&mut hasher, &[u8::from(self.allow_redelegation)]);
        frame(&mut hasher, &self.max_child_lifetime_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_from_ms.to_le_bytes());
        frame(&mut hasher, &self.valid_until_ms.to_le_bytes());
        frame(&mut hasher, &[source_kind_code(&self.source.kind)]);
        frame(&mut hasher, self.source.reference.as_bytes());
        frame(&mut hasher, self.source.proof_ref.as_bytes());
        frame(&mut hasher, self.policy_proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DelegationAuthorityRef {
    pub policy_id: String,
    pub policy_digest: Digest32,
    pub policy_profile: String,
    pub parent_generation: u64,
    pub policy_generation: u64,
    pub current_authority_digest: Digest32,
    pub current_authority_profile: String,
}

impl DelegationAuthorityRef {
    pub fn validate(&self) -> Result<(), DelegationPolicyError> {
        require_id(&self.policy_id)?;
        if self.policy_digest.is_zero() || self.current_authority_digest.is_zero() {
            return Err(DelegationPolicyError::ZeroPolicyDigest);
        }
        if self.policy_profile != DELEGATION_POLICY_IDENTITY_PROFILE {
            return Err(DelegationPolicyError::WrongPolicyIdentityProfile);
        }
        if self.current_authority_profile != CURRENT_DELEGATION_AUTHORITY_PROFILE {
            return Err(DelegationPolicyError::WrongCurrentAuthorityProfile);
        }
        if self.parent_generation == 0 || self.policy_generation == 0 {
            return Err(DelegationPolicyError::ZeroCurrentAuthorityGeneration);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedDelegationParentGrant {
    pub grant: AuthorityGrant,
    pub grant_record_ref: String,
    pub verified_grant_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub freshness: VerifiedAuthorityFreshness,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedDelegationPolicy {
    pub policy: DelegationPolicy,
    pub policy_record_ref: String,
    pub verified_policy_proof_ref: String,
    pub verified_source_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub freshness: VerifiedAuthorityFreshness,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedDelegationAuthority {
    protocol_version: String,
    authority_ref: DelegationAuthorityRef,
    parent_grant_id: AuthorityGrantId,
    parent_grant_digest: Digest32,
    parent_generation: u64,
    delegator: PrincipalId,
    institution: InstitutionId,
    jurisdiction: Option<JurisdictionId>,
    rulebook: RulebookRef,
    delegable_roles: Vec<RoleId>,
    delegable_capabilities: Vec<CapabilityId>,
    delegate_scope: DelegateScope,
    allow_redelegation: bool,
    max_child_lifetime_ms: u64,
    valid_from_ms: u64,
    valid_until_ms: u64,
    policy_record_ref: String,
    policy_verification_ref: String,
    parent_verification_ref: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedDelegationAuthority {
    pub fn protocol_version(&self) -> &str {
        &self.protocol_version
    }

    pub fn authority_ref(&self) -> &DelegationAuthorityRef {
        &self.authority_ref
    }

    pub fn parent_grant_id(&self) -> &AuthorityGrantId {
        &self.parent_grant_id
    }

    pub fn parent_grant_digest(&self) -> Digest32 {
        self.parent_grant_digest
    }

    pub fn parent_generation(&self) -> u64 {
        self.parent_generation
    }

    pub fn delegator(&self) -> &PrincipalId {
        &self.delegator
    }

    pub fn institution(&self) -> &InstitutionId {
        &self.institution
    }

    pub fn jurisdiction(&self) -> Option<&JurisdictionId> {
        self.jurisdiction.as_ref()
    }

    pub fn rulebook(&self) -> &RulebookRef {
        &self.rulebook
    }

    pub fn allow_redelegation(&self) -> bool {
        self.allow_redelegation
    }

    pub fn policy_record_ref(&self) -> &str {
        &self.policy_record_ref
    }

    pub fn policy_verification_ref(&self) -> &str {
        &self.policy_verification_ref
    }

    pub fn parent_verification_ref(&self) -> &str {
        &self.parent_verification_ref
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }

    pub fn validate_delegation_scope(
        &self,
        delegate: &PrincipalId,
        roles: &[RoleId],
        capabilities: &[CapabilityId],
        issued_at_ms: u64,
        expires_at_ms: u64,
        now_ms: u64,
    ) -> Result<(), DelegationPolicyError> {
        if now_ms == 0 || now_ms < self.verified_at_ms || now_ms >= self.lease_until_ms {
            return Err(DelegationPolicyError::PolicyLeaseExpired);
        }
        require_id(delegate.as_str())?;
        let roles = canonical_roles(roles)?;
        let capabilities = canonical_capabilities(capabilities)?;
        if capabilities.is_empty() {
            return Err(DelegationPolicyError::NoDelegableCapabilities);
        }
        if !self.delegate_scope.allows(delegate) {
            return Err(DelegationPolicyError::DelegateNotAllowed);
        }
        let allowed_roles = role_set(&self.delegable_roles);
        if !roles.iter().all(|role| allowed_roles.contains(role)) {
            return Err(DelegationPolicyError::RoleOutsidePolicy);
        }
        let allowed_capabilities = capability_set(&self.delegable_capabilities);
        if !capabilities
            .iter()
            .all(|capability| allowed_capabilities.contains(capability))
        {
            return Err(DelegationPolicyError::CapabilityOutsidePolicy);
        }
        if issued_at_ms == 0 || issued_at_ms > now_ms || expires_at_ms <= issued_at_ms {
            return Err(DelegationPolicyError::InvalidChildLifetime);
        }
        if now_ms >= expires_at_ms {
            return Err(DelegationPolicyError::ChildAlreadyExpired);
        }
        if issued_at_ms < self.valid_from_ms || expires_at_ms > self.valid_until_ms {
            return Err(DelegationPolicyError::ChildOutsidePolicyLifetime);
        }
        if expires_at_ms - issued_at_ms > self.max_child_lifetime_ms {
            return Err(DelegationPolicyError::ChildLifetimeTooLong);
        }
        Ok(())
    }
}

pub fn qualify_delegation_authority(
    parent_receipt: &VerifiedDelegationParentGrant,
    policy_receipt: &VerifiedDelegationPolicy,
    now_ms: u64,
) -> Result<QualifiedDelegationAuthority, DelegationPolicyError> {
    if now_ms == 0 {
        return Err(DelegationPolicyError::InvalidVerificationTime);
    }
    let (parent_identity, parent_subject) = verify_parent(parent_receipt, now_ms)?;
    let policy = &policy_receipt.policy;
    policy.validate()?;
    verify_policy_receipt(policy_receipt, now_ms)?;
    if !policy.is_active_at(now_ms) {
        return Err(DelegationPolicyError::PolicyInactive);
    }

    if policy.parent_grant_id != parent_receipt.grant.id {
        return Err(DelegationPolicyError::ParentGrantMismatch);
    }
    if policy.parent_grant_digest != parent_identity.digest {
        return Err(DelegationPolicyError::ParentGrantDigestMismatch);
    }
    if policy.parent_generation != parent_receipt.freshness.snapshot.generation {
        return Err(DelegationPolicyError::ParentGenerationMismatch);
    }
    if policy.institution != parent_receipt.grant.institution {
        return Err(DelegationPolicyError::InstitutionMismatch);
    }
    if policy.jurisdiction != parent_receipt.grant.jurisdiction {
        return Err(DelegationPolicyError::JurisdictionMismatch);
    }
    if policy.rulebook != parent_receipt.grant.rulebook {
        return Err(DelegationPolicyError::RulebookMismatch);
    }
    if policy.delegator != parent_receipt.grant.holder {
        return Err(DelegationPolicyError::DelegatorMismatch);
    }

    let parent_roles = role_set(&parent_receipt.grant.roles);
    let policy_roles = role_set(&policy.delegable_roles);
    if !policy_roles.is_subset(&parent_roles) {
        return Err(DelegationPolicyError::RoleExpansion);
    }
    let parent_capabilities = capability_set(&parent_receipt.grant.capabilities);
    let policy_capabilities = capability_set(&policy.delegable_capabilities);
    if !policy_capabilities.is_subset(&parent_capabilities) {
        return Err(DelegationPolicyError::CapabilityExpansion);
    }
    if policy.valid_from_ms < parent_receipt.grant.issued_at_ms
        || policy.valid_until_ms > parent_receipt.grant.expires_at_ms
    {
        return Err(DelegationPolicyError::PolicyExceedsParentLifetime);
    }
    if parent_receipt.freshness.snapshot.effective_at_ms > policy.valid_from_ms {
        return Err(DelegationPolicyError::ParentGenerationNotEffectiveAtPolicyStart);
    }

    let policy_digest = policy.identity_digest()?;
    let policy_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::Delegation,
        namespace: policy.institution.as_str().to_string(),
        subject_id: policy.policy_id.clone(),
        identity: FreshnessProfiledDigest {
            digest: policy_digest,
            profile: DELEGATION_POLICY_IDENTITY_PROFILE.into(),
        },
    };
    if policy_receipt.freshness.snapshot.subject != policy_subject {
        return Err(DelegationPolicyError::PolicyFreshnessIdentityMismatch);
    }
    if policy_receipt.freshness.snapshot.effective_at_ms < policy.valid_from_ms {
        return Err(DelegationPolicyError::PolicyFreshnessPredatesPolicy);
    }

    let current = qualify_current_freshness(
        &[parent_subject, policy_subject],
        &[
            parent_receipt.freshness.clone(),
            policy_receipt.freshness.clone(),
        ],
        now_ms,
    )
    .map_err(DelegationPolicyError::Freshness)?;
    if current.freshness_profile != BUNDLE_IDENTITY_PROFILE {
        return Err(DelegationPolicyError::WrongFreshnessProfile);
    }

    let policy_generation = policy_receipt.freshness.snapshot.generation;
    let current_authority_digest = current_authority_digest(policy_digest, current.freshness_digest);
    let authority_ref = DelegationAuthorityRef {
        policy_id: policy.policy_id.clone(),
        policy_digest,
        policy_profile: DELEGATION_POLICY_IDENTITY_PROFILE.into(),
        parent_generation: policy.parent_generation,
        policy_generation,
        current_authority_digest,
        current_authority_profile: CURRENT_DELEGATION_AUTHORITY_PROFILE.into(),
    };
    authority_ref.validate()?;

    let lease_until_ms = current
        .lease_until_ms
        .min(policy.valid_until_ms)
        .min(parent_receipt.grant.expires_at_ms);
    if lease_until_ms <= now_ms {
        return Err(DelegationPolicyError::PolicyLeaseExpired);
    }

    Ok(QualifiedDelegationAuthority {
        protocol_version: PROTOCOL_VERSION.into(),
        authority_ref,
        parent_grant_id: parent_receipt.grant.id.clone(),
        parent_grant_digest: parent_identity.digest,
        parent_generation: policy.parent_generation,
        delegator: policy.delegator.clone(),
        institution: policy.institution.clone(),
        jurisdiction: policy.jurisdiction.clone(),
        rulebook: policy.rulebook.clone(),
        delegable_roles: canonical_roles_owned(&policy.delegable_roles)?,
        delegable_capabilities: canonical_capabilities_owned(&policy.delegable_capabilities)?,
        delegate_scope: canonical_delegate_scope_owned(&policy.delegate_scope)?,
        allow_redelegation: policy.allow_redelegation,
        max_child_lifetime_ms: policy.max_child_lifetime_ms,
        valid_from_ms: policy.valid_from_ms,
        valid_until_ms: policy.valid_until_ms,
        policy_record_ref: policy_receipt.policy_record_ref.clone(),
        policy_verification_ref: policy_receipt.verification_ref.clone(),
        parent_verification_ref: parent_receipt.verification_ref.clone(),
        verified_at_ms: current
            .verified_at_ms
            .max(parent_receipt.verified_at_ms)
            .max(policy_receipt.verified_at_ms),
        lease_until_ms,
    })
}

fn verify_parent(
    receipt: &VerifiedDelegationParentGrant,
    now_ms: u64,
) -> Result<(CanonicalAuthorityIdentity, AuthoritySubjectRef), DelegationPolicyError> {
    receipt
        .grant
        .validate()
        .map_err(|_| DelegationPolicyError::InvalidParentGrant)?;
    if !receipt.grant.is_active_at(now_ms) {
        return Err(DelegationPolicyError::ParentGrantInactive);
    }
    require_ref(&receipt.grant_record_ref)?;
    require_ref(&receipt.verified_grant_proof_ref)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.verified_grant_proof_ref != receipt.grant.grant_proof_ref {
        return Err(DelegationPolicyError::ParentGrantProofMismatch);
    }
    if receipt.verified_at_ms < receipt.grant.issued_at_ms || receipt.verified_at_ms > now_ms {
        return Err(DelegationPolicyError::InvalidVerificationTime);
    }
    if receipt.freshness.snapshot.effective_at_ms < receipt.grant.issued_at_ms {
        return Err(DelegationPolicyError::ParentFreshnessPredatesGrant);
    }

    let identity =
        authority_grant_identity(&receipt.grant).map_err(DelegationPolicyError::ParentIdentity)?;
    if identity.profile != AUTHORITY_GRANT_IDENTITY_PROFILE {
        return Err(DelegationPolicyError::WrongParentIdentityProfile);
    }
    let expected_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::AuthorityGrant,
        namespace: receipt.grant.institution.as_str().to_string(),
        subject_id: receipt.grant.id.as_str().to_string(),
        identity: FreshnessProfiledDigest {
            digest: identity.digest,
            profile: identity.profile.clone(),
        },
    };
    if receipt.freshness.snapshot.subject != expected_subject {
        return Err(DelegationPolicyError::ParentFreshnessIdentityMismatch);
    }
    Ok((identity, expected_subject))
}

fn verify_policy_receipt(
    receipt: &VerifiedDelegationPolicy,
    now_ms: u64,
) -> Result<(), DelegationPolicyError> {
    require_ref(&receipt.policy_record_ref)?;
    require_ref(&receipt.verified_policy_proof_ref)?;
    require_ref(&receipt.verified_source_proof_ref)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.verified_policy_proof_ref != receipt.policy.policy_proof_ref {
        return Err(DelegationPolicyError::PolicyProofMismatch);
    }
    if receipt.verified_source_proof_ref != receipt.policy.source.proof_ref {
        return Err(DelegationPolicyError::SourceProofMismatch);
    }
    if receipt.verified_at_ms < receipt.policy.valid_from_ms || receipt.verified_at_ms > now_ms {
        return Err(DelegationPolicyError::InvalidVerificationTime);
    }
    receipt
        .freshness
        .validate_at(now_ms)
        .map_err(DelegationPolicyError::Freshness)
}

fn current_authority_digest(policy_digest: Digest32, freshness_digest: Digest32) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENT_AUTHORITY);
    frame(&mut hasher, CURRENT_DELEGATION_AUTHORITY_PROFILE.as_bytes());
    frame(&mut hasher, DELEGATION_POLICY_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, BUNDLE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &policy_digest.0);
    frame(&mut hasher, &freshness_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn canonical_roles(roles: &[RoleId]) -> Result<Vec<&str>, DelegationPolicyError> {
    let mut values = Vec::with_capacity(roles.len());
    for role in roles {
        require_id(role.as_str()).map_err(|_| DelegationPolicyError::InvalidRoleId)?;
        values.push(role.as_str());
    }
    let unique = values.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != values.len() {
        return Err(DelegationPolicyError::DuplicateRole);
    }
    Ok(unique.into_iter().collect())
}

fn canonical_capabilities(
    capabilities: &[CapabilityId],
) -> Result<Vec<&str>, DelegationPolicyError> {
    let mut values = Vec::with_capacity(capabilities.len());
    for capability in capabilities {
        require_id(capability.as_str()).map_err(|_| DelegationPolicyError::InvalidCapabilityId)?;
        values.push(capability.as_str());
    }
    let unique = values.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != values.len() {
        return Err(DelegationPolicyError::DuplicateCapability);
    }
    Ok(unique.into_iter().collect())
}

fn canonical_principals(
    principals: &[PrincipalId],
) -> Result<Vec<&str>, DelegationPolicyError> {
    let mut values = Vec::with_capacity(principals.len());
    for principal in principals {
        require_id(principal.as_str()).map_err(|_| DelegationPolicyError::InvalidPrincipalId)?;
        values.push(principal.as_str());
    }
    let unique = values.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != values.len() {
        return Err(DelegationPolicyError::DuplicateDelegate);
    }
    Ok(unique.into_iter().collect())
}

fn canonical_roles_owned(roles: &[RoleId]) -> Result<Vec<RoleId>, DelegationPolicyError> {
    canonical_roles(roles)?
        .into_iter()
        .map(|value| RoleId::new(value).map_err(|_| DelegationPolicyError::InvalidRoleId))
        .collect()
}

fn canonical_capabilities_owned(
    capabilities: &[CapabilityId],
) -> Result<Vec<CapabilityId>, DelegationPolicyError> {
    canonical_capabilities(capabilities)?
        .into_iter()
        .map(|value| CapabilityId::new(value).map_err(|_| DelegationPolicyError::InvalidCapabilityId))
        .collect()
}

enum CanonicalDelegateScope<'a> {
    Any,
    AllowList(Vec<&'a str>),
}

fn canonical_delegate_scope(
    scope: &DelegateScope,
) -> Result<CanonicalDelegateScope<'_>, DelegationPolicyError> {
    match scope {
        DelegateScope::AnyPrincipal => Ok(CanonicalDelegateScope::Any),
        DelegateScope::AllowList(values) => {
            if values.is_empty() || values.len() > MAX_DELEGATES {
                return Err(DelegationPolicyError::InvalidDelegateScope);
            }
            Ok(CanonicalDelegateScope::AllowList(canonical_principals(values)?))
        }
    }
}

fn canonical_delegate_scope_owned(
    scope: &DelegateScope,
) -> Result<DelegateScope, DelegationPolicyError> {
    match canonical_delegate_scope(scope)? {
        CanonicalDelegateScope::Any => Ok(DelegateScope::AnyPrincipal),
        CanonicalDelegateScope::AllowList(values) => Ok(DelegateScope::AllowList(
            values
                .into_iter()
                .map(|value| PrincipalId::new(value).map_err(|_| DelegationPolicyError::InvalidPrincipalId))
                .collect::<Result<Vec<_>, _>>()?,
        )),
    }
}

fn role_set(roles: &[RoleId]) -> BTreeSet<&str> {
    roles.iter().map(|role| role.as_str()).collect()
}

fn capability_set(capabilities: &[CapabilityId]) -> BTreeSet<&str> {
    capabilities.iter().map(|capability| capability.as_str()).collect()
}

fn source_kind_code(kind: &AuthoritySourceKind) -> u8 {
    match kind {
        AuthoritySourceKind::Credential => 1,
        AuthoritySourceKind::Consent => 2,
        AuthoritySourceKind::Agreement => 3,
        AuthoritySourceKind::GovernanceDecision => 4,
        AuthoritySourceKind::Delegation => 5,
        AuthoritySourceKind::EmergencyMandate => 6,
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

fn require_id(value: &str) -> Result<(), DelegationPolicyError> {
    if value.trim().is_empty() || value.len() > MAX_ID_BYTES {
        Err(DelegationPolicyError::InvalidIdentifier)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), DelegationPolicyError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(DelegationPolicyError::InvalidReference)
    } else {
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum DelegationPolicyError {
    WrongProtocolVersion,
    InvalidIdentifier,
    InvalidReference,
    InvalidRulebook,
    InvalidAuthoritySource,
    CircularDelegationPolicySource,
    ZeroParentGrantDigest,
    ZeroPolicyDigest,
    WrongPolicyIdentityProfile,
    WrongCurrentAuthorityProfile,
    ZeroParentGeneration,
    ZeroCurrentAuthorityGeneration,
    NoDelegableCapabilities,
    InvalidRoleId,
    InvalidCapabilityId,
    InvalidPrincipalId,
    DuplicateRole,
    DuplicateCapability,
    DuplicateDelegate,
    InvalidDelegateScope,
    ZeroChildLifetimeLimit,
    InvalidPolicyLifetime,
    InvalidParentGrant,
    ParentGrantInactive,
    ParentGrantProofMismatch,
    ParentIdentity(AuthorityIdentityError),
    WrongParentIdentityProfile,
    ParentFreshnessIdentityMismatch,
    ParentFreshnessPredatesGrant,
    PolicyFreshnessIdentityMismatch,
    PolicyFreshnessPredatesPolicy,
    WrongFreshnessProfile,
    Freshness(FreshnessError),
    PolicyProofMismatch,
    SourceProofMismatch,
    PolicyInactive,
    ParentGrantMismatch,
    ParentGrantDigestMismatch,
    ParentGenerationMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    DelegatorMismatch,
    RoleExpansion,
    CapabilityExpansion,
    PolicyExceedsParentLifetime,
    ParentGenerationNotEffectiveAtPolicyStart,
    InvalidVerificationTime,
    PolicyLeaseExpired,
    DelegateNotAllowed,
    RoleOutsidePolicy,
    CapabilityOutsidePolicy,
    InvalidChildLifetime,
    ChildAlreadyExpired,
    ChildOutsidePolicyLifetime,
    ChildLifetimeTooLong,
}

impl fmt::Display for DelegationPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong delegation-policy protocol version"),
            Self::InvalidIdentifier => write!(f, "invalid delegation-policy identifier"),
            Self::InvalidReference => write!(f, "invalid delegation-policy reference"),
            Self::InvalidRulebook => write!(f, "invalid delegation-policy rulebook"),
            Self::InvalidAuthoritySource => write!(f, "invalid delegation-policy authority source"),
            Self::CircularDelegationPolicySource => write!(f, "delegation cannot directly authorize its own delegation policy"),
            Self::ZeroParentGrantDigest => write!(f, "parent grant digest must not be zero"),
            Self::ZeroPolicyDigest => write!(f, "delegation/current authority digest must not be zero"),
            Self::WrongPolicyIdentityProfile => write!(f, "wrong delegation policy identity profile"),
            Self::WrongCurrentAuthorityProfile => write!(f, "wrong current delegation-authority profile"),
            Self::ZeroParentGeneration => write!(f, "parent generation must not be zero"),
            Self::ZeroCurrentAuthorityGeneration => write!(f, "current delegation-authority generation must not be zero"),
            Self::NoDelegableCapabilities => write!(f, "delegation policy must permit at least one capability"),
            Self::InvalidRoleId => write!(f, "delegation policy contains an invalid role id"),
            Self::InvalidCapabilityId => write!(f, "delegation policy contains an invalid capability id"),
            Self::InvalidPrincipalId => write!(f, "delegation policy contains an invalid principal id"),
            Self::DuplicateRole => write!(f, "delegation policy contains a duplicate role"),
            Self::DuplicateCapability => write!(f, "delegation policy contains a duplicate capability"),
            Self::DuplicateDelegate => write!(f, "delegation policy contains a duplicate delegate"),
            Self::InvalidDelegateScope => write!(f, "invalid delegation policy delegate scope"),
            Self::ZeroChildLifetimeLimit => write!(f, "child lifetime limit must be non-zero"),
            Self::InvalidPolicyLifetime => write!(f, "invalid delegation policy lifetime"),
            Self::InvalidParentGrant => write!(f, "invalid delegation parent grant"),
            Self::ParentGrantInactive => write!(f, "delegation parent grant is inactive"),
            Self::ParentGrantProofMismatch => write!(f, "verified parent grant proof mismatch"),
            Self::ParentIdentity(error) => write!(f, "cannot canonicalize parent grant: {error}"),
            Self::WrongParentIdentityProfile => write!(f, "wrong canonical parent grant profile"),
            Self::ParentFreshnessIdentityMismatch => write!(f, "parent freshness does not bind exact grant identity"),
            Self::ParentFreshnessPredatesGrant => write!(f, "parent freshness state predates parent grant"),
            Self::PolicyFreshnessIdentityMismatch => write!(f, "policy freshness does not bind exact policy identity"),
            Self::PolicyFreshnessPredatesPolicy => write!(f, "policy freshness state predates policy validity"),
            Self::WrongFreshnessProfile => write!(f, "wrong current-authority freshness profile"),
            Self::Freshness(error) => write!(f, "current authority freshness qualification failed: {error}"),
            Self::PolicyProofMismatch => write!(f, "verified delegation policy proof mismatch"),
            Self::SourceProofMismatch => write!(f, "verified delegation policy source proof mismatch"),
            Self::PolicyInactive => write!(f, "delegation policy is inactive"),
            Self::ParentGrantMismatch => write!(f, "delegation policy names a different parent grant"),
            Self::ParentGrantDigestMismatch => write!(f, "delegation policy names a different parent grant digest"),
            Self::ParentGenerationMismatch => write!(f, "delegation policy names a stale/wrong parent generation"),
            Self::InstitutionMismatch => write!(f, "delegation policy institution mismatch"),
            Self::JurisdictionMismatch => write!(f, "delegation policy jurisdiction mismatch"),
            Self::RulebookMismatch => write!(f, "delegation policy rulebook mismatch"),
            Self::DelegatorMismatch => write!(f, "delegation policy delegator is not parent holder"),
            Self::RoleExpansion => write!(f, "delegation policy expands parent roles"),
            Self::CapabilityExpansion => write!(f, "delegation policy expands parent capabilities"),
            Self::PolicyExceedsParentLifetime => write!(f, "delegation policy exceeds parent grant lifetime"),
            Self::ParentGenerationNotEffectiveAtPolicyStart => write!(f, "parent generation was not effective when policy became valid"),
            Self::InvalidVerificationTime => write!(f, "invalid delegation policy verification time"),
            Self::PolicyLeaseExpired => write!(f, "delegation policy/current-parent verification lease is expired"),
            Self::DelegateNotAllowed => write!(f, "delegate is outside delegation policy scope"),
            Self::RoleOutsidePolicy => write!(f, "child role is outside delegation policy scope"),
            Self::CapabilityOutsidePolicy => write!(f, "child capability is outside delegation policy scope"),
            Self::InvalidChildLifetime => write!(f, "invalid child delegation lifetime"),
            Self::ChildAlreadyExpired => write!(f, "child delegation is already expired"),
            Self::ChildOutsidePolicyLifetime => write!(f, "child delegation lies outside policy lifetime"),
            Self::ChildLifetimeTooLong => write!(f, "child delegation exceeds policy lifetime ceiling"),
        }
    }
}

impl std::error::Error for DelegationPolicyError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, PROTOCOL_VERSION as FRESHNESS_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::{RulebookId, PROTOCOL_VERSION as CORE_PROTOCOL_VERSION};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn rulebook() -> RulebookRef {
        RulebookRef {
            id: RulebookId::new("rulebook:test").unwrap(),
            version: "1".into(),
            digest: d(1),
        }
    }

    fn parent() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: CORE_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:root").unwrap(),
            holder: PrincipalId::new("did:example:root").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            roles: vec![RoleId::new("role:executor").unwrap(), RoleId::new("role:reviewer").unwrap()],
            capabilities: vec![CapabilityId::new("governance.execute").unwrap(), CapabilityId::new("governance.read").unwrap()],
            rulebook: rulebook(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "decision:root".into(),
                proof_ref: "proof:root-source".into(),
            }],
            issued_at_ms: 10,
            expires_at_ms: 200,
            delegated_from: None,
            grant_proof_ref: "proof:root-grant".into(),
        }
    }

    fn parent_freshness(grant: &AuthorityGrant, generation: u64) -> VerifiedAuthorityFreshness {
        let identity = authority_grant_identity(grant).unwrap();
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
                subject: AuthoritySubjectRef {
                    kind: AuthoritySubjectKind::AuthorityGrant,
                    namespace: grant.institution.as_str().into(),
                    subject_id: grant.id.as_str().into(),
                    identity: FreshnessProfiledDigest { digest: identity.digest, profile: identity.profile },
                },
                generation,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: grant.issued_at_ms,
                status_record_ref: format!("status:{}:g{generation}", grant.id.as_str()),
            },
            authoritative_source_ref: "authority-state:parent".into(),
            verification_ref: "freshness-verification:parent".into(),
            verified_at_ms: 90,
            lease_until_ms: 150,
        }
    }

    fn parent_receipt(generation: u64) -> VerifiedDelegationParentGrant {
        let grant = parent();
        VerifiedDelegationParentGrant {
            verified_grant_proof_ref: grant.grant_proof_ref.clone(),
            grant_record_ref: "grant-record:root".into(),
            verification_ref: "grant-verification:root".into(),
            verified_at_ms: 90,
            freshness: parent_freshness(&grant, generation),
            grant,
        }
    }

    fn policy() -> DelegationPolicy {
        let grant = parent();
        let identity = authority_grant_identity(&grant).unwrap();
        DelegationPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "delegation-policy:root:1".into(),
            institution: grant.institution.clone(),
            jurisdiction: grant.jurisdiction.clone(),
            rulebook: grant.rulebook.clone(),
            parent_grant_id: grant.id.clone(),
            parent_grant_digest: identity.digest,
            parent_generation: 1,
            delegator: grant.holder.clone(),
            delegable_roles: vec![RoleId::new("role:executor").unwrap()],
            delegable_capabilities: vec![CapabilityId::new("governance.execute").unwrap()],
            delegate_scope: DelegateScope::AllowList(vec![PrincipalId::new("did:example:child").unwrap()]),
            allow_redelegation: false,
            max_child_lifetime_ms: 80,
            valid_from_ms: 20,
            valid_until_ms: 180,
            source: AuthoritySourceRef {
                kind: AuthoritySourceKind::GovernanceDecision,
                reference: "decision:delegation-policy".into(),
                proof_ref: "proof:delegation-policy-source".into(),
            },
            policy_proof_ref: "proof:delegation-policy".into(),
        }
    }

    fn policy_freshness(policy: &DelegationPolicy, generation: u64) -> VerifiedAuthorityFreshness {
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
                subject: AuthoritySubjectRef {
                    kind: AuthoritySubjectKind::Delegation,
                    namespace: policy.institution.as_str().into(),
                    subject_id: policy.policy_id.clone(),
                    identity: FreshnessProfiledDigest { digest: policy.identity_digest().unwrap(), profile: DELEGATION_POLICY_IDENTITY_PROFILE.into() },
                },
                generation,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: policy.valid_from_ms,
                status_record_ref: format!("status:{}:g{generation}", policy.policy_id),
            },
            authoritative_source_ref: "authority-state:delegation-policy".into(),
            verification_ref: "freshness-verification:delegation-policy".into(),
            verified_at_ms: 90,
            lease_until_ms: 150,
        }
    }

    fn policy_receipt_with_generation(policy: DelegationPolicy, generation: u64) -> VerifiedDelegationPolicy {
        VerifiedDelegationPolicy {
            verified_policy_proof_ref: policy.policy_proof_ref.clone(),
            verified_source_proof_ref: policy.source.proof_ref.clone(),
            policy_record_ref: "policy-record:1".into(),
            verification_ref: "policy-verification:1".into(),
            verified_at_ms: 90,
            freshness: policy_freshness(&policy, generation),
            policy,
        }
    }

    fn policy_receipt(policy: DelegationPolicy) -> VerifiedDelegationPolicy {
        policy_receipt_with_generation(policy, 1)
    }

    fn qualify() -> QualifiedDelegationAuthority {
        qualify_delegation_authority(&parent_receipt(1), &policy_receipt(policy()), 100).unwrap()
    }

    #[test]
    fn exact_policy_qualifies_against_current_parent() {
        let authority = qualify();
        assert_eq!(authority.parent_generation(), 1);
        assert_eq!(authority.authority_ref().policy_generation, 1);
        assert!(!authority.authority_ref().current_authority_digest.is_zero());
        assert!(!authority.allow_redelegation());
    }

    #[test]
    fn policy_cannot_expand_parent_capabilities() {
        let mut value = policy();
        value.delegable_capabilities.push(CapabilityId::new("treasury.admin").unwrap());
        assert_eq!(qualify_delegation_authority(&parent_receipt(1), &policy_receipt(value), 100).unwrap_err(), DelegationPolicyError::CapabilityExpansion);
    }

    #[test]
    fn stale_parent_generation_is_denied() {
        assert_eq!(qualify_delegation_authority(&parent_receipt(2), &policy_receipt(policy()), 100).unwrap_err(), DelegationPolicyError::ParentGenerationMismatch);
    }

    #[test]
    fn revoked_policy_denies_current_authority() {
        let mut receipt = policy_receipt(policy());
        receipt.freshness.snapshot.state = AuthorityFreshnessState::Revoked;
        assert_eq!(qualify_delegation_authority(&parent_receipt(1), &receipt, 100).unwrap_err(), DelegationPolicyError::Freshness(FreshnessError::SubjectNotActive));
    }

    #[test]
    fn policy_generation_changes_current_authority_identity() {
        let first = qualify();
        let second = qualify_delegation_authority(&parent_receipt(1), &policy_receipt_with_generation(policy(), 2), 100).unwrap();
        assert_ne!(first.authority_ref(), second.authority_ref());
        assert_eq!(first.authority_ref().policy_digest, second.authority_ref().policy_digest);
    }

    #[test]
    fn source_proof_is_independent_from_policy_proof() {
        let mut receipt = policy_receipt(policy());
        receipt.verified_source_proof_ref = "proof:wrong-source".into();
        assert_eq!(qualify_delegation_authority(&parent_receipt(1), &receipt, 100).unwrap_err(), DelegationPolicyError::SourceProofMismatch);
    }

    #[test]
    fn delegation_cannot_directly_bootstrap_its_own_policy() {
        let mut value = policy();
        value.source.kind = AuthoritySourceKind::Delegation;
        assert_eq!(value.validate().unwrap_err(), DelegationPolicyError::CircularDelegationPolicySource);
    }

    #[test]
    fn set_reordering_preserves_policy_identity() {
        let mut first = policy();
        first.delegable_roles.push(RoleId::new("role:reviewer").unwrap());
        first.delegable_capabilities.push(CapabilityId::new("governance.read").unwrap());
        let mut second = first.clone();
        second.delegable_roles.reverse();
        second.delegable_capabilities.reverse();
        assert_eq!(first.identity_digest().unwrap(), second.identity_digest().unwrap());
    }

    #[test]
    fn redelegation_permission_is_identity_bearing() {
        let first = policy();
        let mut second = first.clone();
        second.allow_redelegation = true;
        assert_ne!(first.identity_digest().unwrap(), second.identity_digest().unwrap());
    }

    #[test]
    fn later_reverification_same_generation_preserves_authority_identity() {
        let first = qualify();
        let mut parent = parent_receipt(1);
        parent.verification_ref = "grant-verification:later".into();
        parent.verified_at_ms = 99;
        parent.freshness.verification_ref = "freshness:later-parent".into();
        parent.freshness.verified_at_ms = 99;
        let mut policy_receipt = policy_receipt(policy());
        policy_receipt.verification_ref = "policy-verification:later".into();
        policy_receipt.verified_at_ms = 99;
        policy_receipt.freshness.verification_ref = "freshness:later-policy".into();
        policy_receipt.freshness.verified_at_ms = 99;
        let second = qualify_delegation_authority(&parent, &policy_receipt, 100).unwrap();
        assert_eq!(first.authority_ref(), second.authority_ref());
    }

    #[test]
    fn delegate_and_child_scope_are_enforced() {
        let authority = qualify();
        assert_eq!(authority.validate_delegation_scope(&PrincipalId::new("did:example:other").unwrap(), &[RoleId::new("role:executor").unwrap()], &[CapabilityId::new("governance.execute").unwrap()], 100, 150, 100).unwrap_err(), DelegationPolicyError::DelegateNotAllowed);
        assert_eq!(authority.validate_delegation_scope(&PrincipalId::new("did:example:child").unwrap(), &[RoleId::new("role:executor").unwrap()], &[CapabilityId::new("governance.read").unwrap()], 100, 150, 100).unwrap_err(), DelegationPolicyError::CapabilityOutsidePolicy);
    }

    #[test]
    fn child_lifetime_is_bounded() {
        let authority = qualify();
        assert_eq!(authority.validate_delegation_scope(&PrincipalId::new("did:example:child").unwrap(), &[RoleId::new("role:executor").unwrap()], &[CapabilityId::new("governance.execute").unwrap()], 90, 175, 100).unwrap_err(), DelegationPolicyError::ChildLifetimeTooLong);
    }
}
