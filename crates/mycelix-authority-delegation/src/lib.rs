// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed delegation attenuation and complete-lineage qualification.
//!
//! A child `AuthorityGrant` is not valid merely because `delegated_from` names a
//! parent ID. This crate binds exact canonical parent/child grant identities,
//! exact current generations, an independently verified authority-to-delegate
//! binding, exact delegated roles/capabilities, proof provenance, and complete
//! root-to-target lineage without choosing by record recency or input order.

use mycelix_authority_freshness::{
    qualify_current_freshness, AuthoritySubjectKind, AuthoritySubjectRef,
    FreshnessError, ProfiledDigest as FreshnessProfiledDigest,
    VerifiedAuthorityFreshness, BUNDLE_IDENTITY_PROFILE,
};
use mycelix_authority_identity::{
    authority_grant_identity, AuthorityIdentityError, CanonicalAuthorityIdentity,
    AUTHORITY_GRANT_IDENTITY_PROFILE,
};
use mycelix_institutional_core::{
    AuthorityGrant, AuthorityGrantId, AuthoritySourceKind, CapabilityId, Digest32, InstitutionId,
    JurisdictionId, PrincipalId, RoleId, RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-delegation-v0.1";
pub const DELEGATION_IDENTITY_PROFILE: &str =
    "mycelix-authority-delegation-v1-blake3-framed-semantic";
pub const CURRENT_EDGE_PROFILE: &str =
    "mycelix-authority-delegation-current-edge-v1-blake3-framed";
pub const LINEAGE_PROFILE: &str =
    "mycelix-authority-delegation-lineage-v1-blake3-framed";

const DOMAIN_DELEGATION: &[u8] = b"mycelix/authority-delegation/attestation/v1";
const DOMAIN_CURRENT_EDGE: &[u8] = b"mycelix/authority-delegation/current-edge/v1";
const DOMAIN_LINEAGE: &[u8] = b"mycelix/authority-delegation/lineage/v1";
const MAX_REF_BYTES: usize = 2048;
const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_LINEAGE_DEPTH: usize = 16;

/// Exact immutable institutional authority that permits this delegation.
///
/// This is intentionally separate from `proof_ref`: a signature/proof can be
/// cryptographically valid without proving that the signer/institution was
/// authorized to delegate the parent's capabilities.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DelegationAuthorityBinding {
    pub authority_ref: String,
    pub authority_digest: Digest32,
    pub authority_profile: String,
}

impl DelegationAuthorityBinding {
    pub fn validate(&self) -> Result<(), DelegationError> {
        require_ref(&self.authority_ref)?;
        if self.authority_digest.is_zero() {
            return Err(DelegationError::ZeroDelegationAuthorityDigest);
        }
        require_profile(&self.authority_profile)
    }
}

/// Institutionally attested attenuation from one exact parent grant to one exact
/// child grant at exact authority generations.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DelegationAttestation {
    pub protocol_version: String,
    pub delegation_id: String,
    pub parent_grant_id: AuthorityGrantId,
    pub child_grant_id: AuthorityGrantId,
    pub parent_grant_digest: Digest32,
    pub child_grant_digest: Digest32,
    pub parent_generation: u64,
    pub child_generation: u64,
    pub delegator: PrincipalId,
    pub delegate: PrincipalId,
    pub institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    /// Exact roles delegated to the child. Order has no semantic meaning.
    pub roles: Vec<RoleId>,
    /// Exact capabilities delegated to the child. Order has no semantic meaning.
    pub capabilities: Vec<CapabilityId>,
    /// Independent institutional decision/policy that makes delegation lawful.
    pub delegation_authority: DelegationAuthorityBinding,
    pub issued_at_ms: u64,
    pub expires_at_ms: u64,
    /// Cryptographic/institutional attestation over this exact delegation object.
    pub proof_ref: String,
}

impl DelegationAttestation {
    pub fn validate(&self) -> Result<(), DelegationError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(DelegationError::WrongProtocolVersion);
        }
        require_ref(&self.delegation_id)?;
        require_id(self.parent_grant_id.as_str())?;
        require_id(self.child_grant_id.as_str())?;
        require_id(self.delegator.as_str())?;
        require_id(self.delegate.as_str())?;
        require_id(self.institution.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_id(jurisdiction.as_str())?;
        }
        if self.parent_grant_id == self.child_grant_id {
            return Err(DelegationError::SelfDelegation);
        }
        if self.parent_grant_digest.is_zero() || self.child_grant_digest.is_zero() {
            return Err(DelegationError::ZeroGrantDigest);
        }
        if self.parent_generation == 0 || self.child_generation == 0 {
            return Err(DelegationError::ZeroGeneration);
        }
        self.rulebook
            .validate()
            .map_err(|_| DelegationError::InvalidRulebook)?;
        if self.capabilities.is_empty() {
            return Err(DelegationError::NoDelegatedCapabilities);
        }
        canonical_roles(&self.roles)?;
        canonical_capabilities(&self.capabilities)?;
        self.delegation_authority.validate()?;
        if self.issued_at_ms == 0 || self.expires_at_ms <= self.issued_at_ms {
            return Err(DelegationError::InvalidDelegationLifetime);
        }
        require_ref(&self.proof_ref)
    }

    pub fn identity_digest(&self) -> Result<Digest32, DelegationError> {
        self.validate()?;
        let roles = canonical_roles(&self.roles)?;
        let capabilities = canonical_capabilities(&self.capabilities)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_DELEGATION);
        frame(&mut hasher, DELEGATION_IDENTITY_PROFILE.as_bytes());
        frame(&mut hasher, self.protocol_version.as_bytes());
        frame(&mut hasher, self.delegation_id.as_bytes());
        frame(&mut hasher, self.parent_grant_id.as_str().as_bytes());
        frame(&mut hasher, self.child_grant_id.as_str().as_bytes());
        frame(&mut hasher, &self.parent_grant_digest.0);
        frame(&mut hasher, &self.child_grant_digest.0);
        frame(&mut hasher, &self.parent_generation.to_le_bytes());
        frame(&mut hasher, &self.child_generation.to_le_bytes());
        frame(&mut hasher, self.delegator.as_str().as_bytes());
        frame(&mut hasher, self.delegate.as_str().as_bytes());
        frame(&mut hasher, self.institution.as_str().as_bytes());
        frame_optional_text(
            &mut hasher,
            self.jurisdiction.as_ref().map(|value| value.as_str()),
        );
        frame(&mut hasher, self.rulebook.id.as_str().as_bytes());
        frame(&mut hasher, self.rulebook.version.as_bytes());
        frame(&mut hasher, &self.rulebook.digest.0);
        frame(&mut hasher, &(roles.len() as u64).to_le_bytes());
        for role in roles {
            frame(&mut hasher, role.as_bytes());
        }
        frame(&mut hasher, &(capabilities.len() as u64).to_le_bytes());
        for capability in capabilities {
            frame(&mut hasher, capability.as_bytes());
        }
        frame(
            &mut hasher,
            self.delegation_authority.authority_ref.as_bytes(),
        );
        frame(
            &mut hasher,
            self.delegation_authority.authority_profile.as_bytes(),
        );
        frame(
            &mut hasher,
            &self.delegation_authority.authority_digest.0,
        );
        frame(&mut hasher, &self.issued_at_ms.to_le_bytes());
        frame(&mut hasher, &self.expires_at_ms.to_le_bytes());
        frame(&mut hasher, self.proof_ref.as_bytes());
        Ok(Digest32(*hasher.finalize().as_bytes()))
    }
}

/// One exact grant plus independent proof verification and current freshness.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedGrantForDelegation {
    pub grant: AuthorityGrant,
    pub grant_record_ref: String,
    pub verified_grant_proof_ref: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub freshness: VerifiedAuthorityFreshness,
}

/// Host-verified institutional delegation attestation.
///
/// The host independently verifies both the exact proof and the exact immutable
/// authority-to-delegate binding. Merely possessing a valid parent grant does not
/// imply that its holder may delegate it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedDelegationAttestation {
    pub attestation: DelegationAttestation,
    pub delegation_record_ref: String,
    pub verified_proof_ref: String,
    pub verified_delegation_authority_ref: String,
    pub verified_delegation_authority_digest: Digest32,
    pub verified_delegation_authority_profile: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
}

/// Qualified current parent->child delegation edge.
///
/// Fields are private and this type is not deserializable. Downstream code must
/// obtain it from `qualify_delegation_edge` rather than reconstructing authority
/// from application data.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedDelegationEdge {
    protocol_version: String,
    delegation_id: String,
    delegation_identity_digest: Digest32,
    parent_grant_id: AuthorityGrantId,
    child_grant_id: AuthorityGrantId,
    parent_grant_identity_digest: Digest32,
    child_grant_identity_digest: Digest32,
    parent_generation: u64,
    child_generation: u64,
    delegator: PrincipalId,
    delegate: PrincipalId,
    roles: Vec<RoleId>,
    capabilities: Vec<CapabilityId>,
    freshness_digest: Digest32,
    freshness_profile: String,
    current_edge_digest: Digest32,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedDelegationEdge {
    pub fn delegation_id(&self) -> &str {
        &self.delegation_id
    }

    pub fn parent_grant_id(&self) -> &AuthorityGrantId {
        &self.parent_grant_id
    }

    pub fn child_grant_id(&self) -> &AuthorityGrantId {
        &self.child_grant_id
    }

    pub fn parent_generation(&self) -> u64 {
        self.parent_generation
    }

    pub fn child_generation(&self) -> u64 {
        self.child_generation
    }

    pub fn current_edge_digest(&self) -> Digest32 {
        self.current_edge_digest
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

/// Deterministically reconstructed complete root->target delegation chain.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedDelegationLineage {
    protocol_version: String,
    root_grant_id: AuthorityGrantId,
    target_grant_id: AuthorityGrantId,
    depth: u8,
    lineage_digest: Digest32,
    lineage_profile: String,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedDelegationLineage {
    pub fn root_grant_id(&self) -> &AuthorityGrantId {
        &self.root_grant_id
    }

    pub fn target_grant_id(&self) -> &AuthorityGrantId {
        &self.target_grant_id
    }

    pub fn depth(&self) -> u8 {
        self.depth
    }

    pub fn lineage_digest(&self) -> Digest32 {
        self.lineage_digest
    }

    pub fn lineage_profile(&self) -> &str {
        &self.lineage_profile
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

pub fn qualify_delegation_edge(
    parent_receipt: &VerifiedGrantForDelegation,
    child_receipt: &VerifiedGrantForDelegation,
    delegation_receipt: &VerifiedDelegationAttestation,
    delegation_freshness: &VerifiedAuthorityFreshness,
    now_ms: u64,
) -> Result<QualifiedDelegationEdge, DelegationError> {
    let (parent_identity, parent_subject) = verify_grant_receipt(parent_receipt, now_ms)?;
    let (child_identity, child_subject) = verify_grant_receipt(child_receipt, now_ms)?;

    let parent = &parent_receipt.grant;
    let child = &child_receipt.grant;
    let attestation = &delegation_receipt.attestation;
    attestation.validate()?;
    verify_delegation_receipt(delegation_receipt, now_ms)?;

    if child.delegated_from.as_ref() != Some(&parent.id) {
        return Err(DelegationError::ParentGrantMismatch);
    }
    if attestation.parent_grant_id != parent.id || attestation.child_grant_id != child.id {
        return Err(DelegationError::GrantIdentityMismatch);
    }
    if attestation.parent_grant_digest != parent_identity.digest
        || attestation.child_grant_digest != child_identity.digest
    {
        return Err(DelegationError::GrantDigestMismatch);
    }

    if parent.institution != child.institution
        || attestation.institution != parent.institution
    {
        return Err(DelegationError::InstitutionMismatch);
    }
    if parent.jurisdiction != child.jurisdiction
        || attestation.jurisdiction != parent.jurisdiction
    {
        return Err(DelegationError::JurisdictionMismatch);
    }
    if parent.rulebook != child.rulebook || attestation.rulebook != parent.rulebook {
        return Err(DelegationError::RulebookMismatch);
    }
    if attestation.delegator != parent.holder || attestation.delegate != child.holder {
        return Err(DelegationError::PrincipalMismatch);
    }

    let parent_roles = role_set(&parent.roles);
    let child_roles = role_set(&child.roles);
    let delegated_roles = role_set(&attestation.roles);
    if !child_roles.is_subset(&parent_roles) {
        return Err(DelegationError::RoleExpansion);
    }
    if child_roles != delegated_roles {
        return Err(DelegationError::DelegatedRoleMismatch);
    }

    let parent_capabilities = capability_set(&parent.capabilities);
    let child_capabilities = capability_set(&child.capabilities);
    let delegated_capabilities = capability_set(&attestation.capabilities);
    if !child_capabilities.is_subset(&parent_capabilities) {
        return Err(DelegationError::CapabilityExpansion);
    }
    if child_capabilities != delegated_capabilities {
        return Err(DelegationError::DelegatedCapabilityMismatch);
    }

    let delegation_sources = child
        .sources
        .iter()
        .filter(|source| matches!(&source.kind, AuthoritySourceKind::Delegation))
        .collect::<Vec<_>>();
    if delegation_sources.len() != 1 {
        return Err(DelegationError::AmbiguousDelegationSource);
    }
    let source = delegation_sources[0];
    if source.reference != attestation.delegation_id || source.proof_ref != attestation.proof_ref {
        return Err(DelegationError::DelegationSourceMismatch);
    }

    if attestation.issued_at_ms < parent.issued_at_ms
        || attestation.expires_at_ms > parent.expires_at_ms
        || child.issued_at_ms < attestation.issued_at_ms
        || child.expires_at_ms > attestation.expires_at_ms
        || child.expires_at_ms > parent.expires_at_ms
    {
        return Err(DelegationError::LifetimeExpansion);
    }

    if parent_receipt.freshness.snapshot.effective_at_ms > attestation.issued_at_ms {
        return Err(DelegationError::ParentGenerationNotEffectiveAtDelegation);
    }
    if attestation.parent_generation != parent_receipt.freshness.snapshot.generation
        || attestation.child_generation != child_receipt.freshness.snapshot.generation
    {
        return Err(DelegationError::GenerationMismatch);
    }

    let delegation_identity_digest = attestation.identity_digest()?;
    let delegation_subject = AuthoritySubjectRef {
        kind: AuthoritySubjectKind::Delegation,
        namespace: attestation.institution.as_str().to_string(),
        subject_id: attestation.delegation_id.clone(),
        identity: FreshnessProfiledDigest {
            digest: delegation_identity_digest,
            profile: DELEGATION_IDENTITY_PROFILE.into(),
        },
    };
    if delegation_freshness.snapshot.subject != delegation_subject {
        return Err(DelegationError::DelegationFreshnessIdentityMismatch);
    }
    if delegation_freshness.snapshot.effective_at_ms < attestation.issued_at_ms {
        return Err(DelegationError::DelegationFreshnessPredatesAttestation);
    }

    let current_freshness = qualify_current_freshness(
        &[parent_subject, child_subject, delegation_subject],
        &[
            parent_receipt.freshness.clone(),
            child_receipt.freshness.clone(),
            delegation_freshness.clone(),
        ],
        now_ms,
    )
    .map_err(DelegationError::Freshness)?;
    if current_freshness.freshness_profile != BUNDLE_IDENTITY_PROFILE {
        return Err(DelegationError::FreshnessProfileMismatch);
    }

    let current_edge_digest = current_edge_digest(
        delegation_identity_digest,
        current_freshness.freshness_digest,
    );
    let roles = canonical_roles_owned(&attestation.roles)?;
    let capabilities = canonical_capabilities_owned(&attestation.capabilities)?;

    let lease_until_ms = current_freshness
        .lease_until_ms
        .min(attestation.expires_at_ms)
        .min(parent.expires_at_ms)
        .min(child.expires_at_ms);
    if lease_until_ms <= now_ms {
        return Err(DelegationError::LineageLeaseExpired);
    }

    Ok(QualifiedDelegationEdge {
        protocol_version: PROTOCOL_VERSION.into(),
        delegation_id: attestation.delegation_id.clone(),
        delegation_identity_digest,
        parent_grant_id: parent.id.clone(),
        child_grant_id: child.id.clone(),
        parent_grant_identity_digest: parent_identity.digest,
        child_grant_identity_digest: child_identity.digest,
        parent_generation: attestation.parent_generation,
        child_generation: attestation.child_generation,
        delegator: parent.holder.clone(),
        delegate: child.holder.clone(),
        roles,
        capabilities,
        freshness_digest: current_freshness.freshness_digest,
        freshness_profile: current_freshness.freshness_profile,
        current_edge_digest,
        verified_at_ms: current_freshness
            .verified_at_ms
            .max(parent_receipt.verified_at_ms)
            .max(child_receipt.verified_at_ms)
            .max(delegation_receipt.verified_at_ms),
        lease_until_ms,
    })
}

/// Reconstruct a complete delegated root->target chain from already-qualified
/// edges. Input order is irrelevant. Every supplied edge must belong to the one
/// exact chain; cycles, missing parents, duplicate children, mixed generations,
/// excess depth, and unrelated extras fail closed.
pub fn qualify_complete_delegation_lineage(
    root: &AuthorityGrant,
    target: &AuthorityGrant,
    edges: &[QualifiedDelegationEdge],
    now_ms: u64,
) -> Result<QualifiedDelegationLineage, DelegationError> {
    if now_ms == 0 {
        return Err(DelegationError::InvalidVerificationTime);
    }
    root.validate()
        .map_err(|_| DelegationError::InvalidParentGrant)?;
    target
        .validate()
        .map_err(|_| DelegationError::InvalidChildGrant)?;
    if root.delegated_from.is_some() {
        return Err(DelegationError::RootGrantIsDelegated);
    }
    if root.id == target.id || edges.is_empty() {
        return Err(DelegationError::EmptyDelegationLineage);
    }
    if edges.len() > MAX_LINEAGE_DEPTH {
        return Err(DelegationError::LineageTooDeep);
    }

    let root_identity = authority_grant_identity(root).map_err(DelegationError::ParentIdentity)?;
    let target_identity =
        authority_grant_identity(target).map_err(DelegationError::ChildIdentity)?;

    let mut by_child = BTreeMap::<AuthorityGrantId, &QualifiedDelegationEdge>::new();
    for edge in edges {
        validate_qualified_edge(edge, now_ms)?;
        if by_child.insert(edge.child_grant_id.clone(), edge).is_some() {
            return Err(DelegationError::DuplicateLineageChild);
        }
    }

    let mut current_id = target.id.clone();
    let mut expected_identity = target_identity.digest;
    let mut expected_child_generation: Option<u64> = None;
    let mut seen = BTreeSet::<AuthorityGrantId>::new();
    let mut reverse = Vec::<&QualifiedDelegationEdge>::new();

    while current_id != root.id {
        if !seen.insert(current_id.clone()) {
            return Err(DelegationError::DelegationCycle);
        }
        if reverse.len() >= MAX_LINEAGE_DEPTH {
            return Err(DelegationError::LineageTooDeep);
        }
        let edge = by_child
            .get(&current_id)
            .copied()
            .ok_or(DelegationError::MissingLineageEdge)?;
        if edge.child_grant_identity_digest != expected_identity {
            return Err(DelegationError::LineageGrantIdentityMismatch);
        }
        if let Some(expected_generation) = expected_child_generation {
            if edge.child_generation != expected_generation {
                return Err(DelegationError::LineageGenerationMismatch);
            }
        }
        reverse.push(edge);
        current_id = edge.parent_grant_id.clone();
        expected_identity = edge.parent_grant_identity_digest;
        expected_child_generation = Some(edge.parent_generation);
    }

    if expected_identity != root_identity.digest {
        return Err(DelegationError::LineageGrantIdentityMismatch);
    }
    if reverse.len() != edges.len() {
        return Err(DelegationError::UnexpectedLineageEdge);
    }

    reverse.reverse();
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_LINEAGE);
    frame(&mut hasher, LINEAGE_PROFILE.as_bytes());
    frame(&mut hasher, AUTHORITY_GRANT_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, root.id.as_str().as_bytes());
    frame(&mut hasher, &root_identity.digest.0);
    frame(&mut hasher, target.id.as_str().as_bytes());
    frame(&mut hasher, &target_identity.digest.0);
    frame(&mut hasher, &(reverse.len() as u64).to_le_bytes());

    let mut verified_at_ms = 0u64;
    let mut lease_until_ms = u64::MAX;
    for edge in &reverse {
        frame(&mut hasher, &edge.current_edge_digest.0);
        verified_at_ms = verified_at_ms.max(edge.verified_at_ms);
        lease_until_ms = lease_until_ms.min(edge.lease_until_ms);
    }
    if lease_until_ms <= now_ms {
        return Err(DelegationError::LineageLeaseExpired);
    }

    Ok(QualifiedDelegationLineage {
        protocol_version: PROTOCOL_VERSION.into(),
        root_grant_id: root.id.clone(),
        target_grant_id: target.id.clone(),
        depth: reverse.len() as u8,
        lineage_digest: Digest32(*hasher.finalize().as_bytes()),
        lineage_profile: LINEAGE_PROFILE.into(),
        verified_at_ms,
        lease_until_ms,
    })
}

fn verify_grant_receipt(
    receipt: &VerifiedGrantForDelegation,
    now_ms: u64,
) -> Result<(CanonicalAuthorityIdentity, AuthoritySubjectRef), DelegationError> {
    receipt
        .grant
        .validate()
        .map_err(|_| DelegationError::InvalidGrantReceipt)?;
    if !receipt.grant.is_active_at(now_ms) {
        return Err(DelegationError::GrantInactive);
    }
    require_ref(&receipt.grant_record_ref)?;
    require_ref(&receipt.verified_grant_proof_ref)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.verified_grant_proof_ref != receipt.grant.grant_proof_ref {
        return Err(DelegationError::GrantProofMismatch);
    }
    if receipt.verified_at_ms < receipt.grant.issued_at_ms || receipt.verified_at_ms > now_ms {
        return Err(DelegationError::InvalidVerificationTime);
    }
    receipt
        .freshness
        .validate_at(now_ms)
        .map_err(DelegationError::Freshness)?;
    if receipt.freshness.snapshot.effective_at_ms < receipt.grant.issued_at_ms {
        return Err(DelegationError::GrantFreshnessPredatesGrant);
    }

    let identity =
        authority_grant_identity(&receipt.grant).map_err(DelegationError::GrantIdentity)?;
    if identity.profile != AUTHORITY_GRANT_IDENTITY_PROFILE {
        return Err(DelegationError::GrantIdentityProfileMismatch);
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
        return Err(DelegationError::GrantFreshnessIdentityMismatch);
    }
    Ok((identity, expected_subject))
}

fn verify_delegation_receipt(
    receipt: &VerifiedDelegationAttestation,
    now_ms: u64,
) -> Result<(), DelegationError> {
    require_ref(&receipt.delegation_record_ref)?;
    require_ref(&receipt.verified_proof_ref)?;
    require_ref(&receipt.verified_delegation_authority_ref)?;
    require_profile(&receipt.verified_delegation_authority_profile)?;
    require_ref(&receipt.verification_ref)?;
    if receipt.verified_proof_ref != receipt.attestation.proof_ref {
        return Err(DelegationError::DelegationProofMismatch);
    }
    if receipt.verified_delegation_authority_ref
        != receipt.attestation.delegation_authority.authority_ref
        || receipt.verified_delegation_authority_digest
            != receipt.attestation.delegation_authority.authority_digest
        || receipt.verified_delegation_authority_profile
            != receipt.attestation.delegation_authority.authority_profile
    {
        return Err(DelegationError::DelegationAuthorityMismatch);
    }
    if receipt.verified_delegation_authority_digest.is_zero() {
        return Err(DelegationError::ZeroDelegationAuthorityDigest);
    }
    if receipt.verified_at_ms < receipt.attestation.issued_at_ms
        || receipt.verified_at_ms > now_ms
    {
        return Err(DelegationError::InvalidVerificationTime);
    }
    Ok(())
}

fn validate_qualified_edge(
    edge: &QualifiedDelegationEdge,
    now_ms: u64,
) -> Result<(), DelegationError> {
    if edge.protocol_version != PROTOCOL_VERSION
        || edge.parent_grant_id == edge.child_grant_id
        || edge.parent_grant_identity_digest.is_zero()
        || edge.child_grant_identity_digest.is_zero()
        || edge.delegation_identity_digest.is_zero()
        || edge.freshness_digest.is_zero()
        || edge.current_edge_digest.is_zero()
        || edge.parent_generation == 0
        || edge.child_generation == 0
        || edge.capabilities.is_empty()
        || edge.verified_at_ms == 0
        || edge.verified_at_ms > now_ms
        || edge.lease_until_ms <= now_ms
    {
        return Err(DelegationError::InvalidQualifiedEdge);
    }
    if edge.freshness_profile != BUNDLE_IDENTITY_PROFILE {
        return Err(DelegationError::FreshnessProfileMismatch);
    }
    if edge.current_edge_digest
        != current_edge_digest(edge.delegation_identity_digest, edge.freshness_digest)
    {
        return Err(DelegationError::InvalidQualifiedEdge);
    }
    canonical_roles(&edge.roles)?;
    canonical_capabilities(&edge.capabilities)?;
    require_ref(&edge.delegation_id)?;
    Ok(())
}

fn current_edge_digest(delegation_identity: Digest32, freshness_digest: Digest32) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENT_EDGE);
    frame(&mut hasher, CURRENT_EDGE_PROFILE.as_bytes());
    frame(&mut hasher, BUNDLE_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, &delegation_identity.0);
    frame(&mut hasher, &freshness_digest.0);
    Digest32(*hasher.finalize().as_bytes())
}

fn canonical_roles(roles: &[RoleId]) -> Result<Vec<&str>, DelegationError> {
    let mut values = Vec::with_capacity(roles.len());
    for role in roles {
        require_id(role.as_str()).map_err(|_| DelegationError::InvalidDelegatedRole)?;
        values.push(role.as_str());
    }
    let set = values.iter().copied().collect::<BTreeSet<_>>();
    if set.len() != values.len() {
        return Err(DelegationError::DuplicateDelegatedRole);
    }
    Ok(set.into_iter().collect())
}

fn canonical_capabilities(
    capabilities: &[CapabilityId],
) -> Result<Vec<&str>, DelegationError> {
    let mut values = Vec::with_capacity(capabilities.len());
    for capability in capabilities {
        require_id(capability.as_str())
            .map_err(|_| DelegationError::InvalidDelegatedCapability)?;
        values.push(capability.as_str());
    }
    let set = values.iter().copied().collect::<BTreeSet<_>>();
    if set.len() != values.len() {
        return Err(DelegationError::DuplicateDelegatedCapability);
    }
    Ok(set.into_iter().collect())
}

fn canonical_roles_owned(roles: &[RoleId]) -> Result<Vec<RoleId>, DelegationError> {
    canonical_roles(roles)?
        .into_iter()
        .map(|value| RoleId::new(value).map_err(|_| DelegationError::InvalidDelegatedRole))
        .collect()
}

fn canonical_capabilities_owned(
    capabilities: &[CapabilityId],
) -> Result<Vec<CapabilityId>, DelegationError> {
    canonical_capabilities(capabilities)?
        .into_iter()
        .map(|value| {
            CapabilityId::new(value).map_err(|_| DelegationError::InvalidDelegatedCapability)
        })
        .collect()
}

fn role_set(roles: &[RoleId]) -> BTreeSet<&str> {
    roles.iter().map(|role| role.as_str()).collect()
}

fn capability_set(capabilities: &[CapabilityId]) -> BTreeSet<&str> {
    capabilities
        .iter()
        .map(|capability| capability.as_str())
        .collect()
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

fn require_id(value: &str) -> Result<(), DelegationError> {
    if value.trim().is_empty() || value.len() > MAX_ID_BYTES {
        Err(DelegationError::InvalidIdentifier)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), DelegationError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(DelegationError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), DelegationError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(DelegationError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum DelegationError {
    WrongProtocolVersion,
    InvalidIdentifier,
    InvalidReference,
    InvalidProfile,
    InvalidRulebook,
    SelfDelegation,
    ZeroGrantDigest,
    ZeroDelegationAuthorityDigest,
    ZeroGeneration,
    NoDelegatedCapabilities,
    InvalidDelegatedRole,
    InvalidDelegatedCapability,
    DuplicateDelegatedRole,
    DuplicateDelegatedCapability,
    InvalidDelegationLifetime,
    InvalidParentGrant,
    InvalidChildGrant,
    InvalidGrantReceipt,
    GrantInactive,
    GrantProofMismatch,
    GrantIdentity(AuthorityIdentityError),
    ParentIdentity(AuthorityIdentityError),
    ChildIdentity(AuthorityIdentityError),
    GrantIdentityProfileMismatch,
    GrantFreshnessIdentityMismatch,
    GrantFreshnessPredatesGrant,
    DelegationFreshnessIdentityMismatch,
    DelegationFreshnessPredatesAttestation,
    FreshnessProfileMismatch,
    Freshness(FreshnessError),
    DelegationProofMismatch,
    DelegationAuthorityMismatch,
    ParentGrantMismatch,
    GrantIdentityMismatch,
    GrantDigestMismatch,
    InstitutionMismatch,
    JurisdictionMismatch,
    RulebookMismatch,
    PrincipalMismatch,
    RoleExpansion,
    CapabilityExpansion,
    DelegatedRoleMismatch,
    DelegatedCapabilityMismatch,
    AmbiguousDelegationSource,
    DelegationSourceMismatch,
    LifetimeExpansion,
    ParentGenerationNotEffectiveAtDelegation,
    GenerationMismatch,
    InvalidVerificationTime,
    RootGrantIsDelegated,
    EmptyDelegationLineage,
    LineageTooDeep,
    DuplicateLineageChild,
    DelegationCycle,
    MissingLineageEdge,
    UnexpectedLineageEdge,
    LineageGrantIdentityMismatch,
    LineageGenerationMismatch,
    InvalidQualifiedEdge,
    LineageLeaseExpired,
}

impl fmt::Display for DelegationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong authority-delegation protocol version"),
            Self::InvalidIdentifier => write!(f, "invalid authority-delegation identifier"),
            Self::InvalidReference => write!(f, "invalid authority-delegation reference"),
            Self::InvalidProfile => write!(f, "invalid authority-delegation digest profile"),
            Self::InvalidRulebook => write!(f, "invalid delegation rulebook"),
            Self::SelfDelegation => write!(f, "delegation parent and child grant must differ"),
            Self::ZeroGrantDigest => write!(f, "delegation grant identity digest must not be zero"),
            Self::ZeroDelegationAuthorityDigest => {
                write!(f, "delegation authority digest must not be zero")
            }
            Self::ZeroGeneration => write!(f, "delegation generation must not be zero"),
            Self::NoDelegatedCapabilities => write!(f, "delegation must contain capabilities"),
            Self::InvalidDelegatedRole => write!(f, "delegation contains an invalid role id"),
            Self::InvalidDelegatedCapability => {
                write!(f, "delegation contains an invalid capability id")
            }
            Self::DuplicateDelegatedRole => write!(f, "delegation contains a duplicate role"),
            Self::DuplicateDelegatedCapability => {
                write!(f, "delegation contains a duplicate capability")
            }
            Self::InvalidDelegationLifetime => write!(f, "invalid delegation lifetime"),
            Self::InvalidParentGrant => write!(f, "invalid root/parent authority grant"),
            Self::InvalidChildGrant => write!(f, "invalid child/target authority grant"),
            Self::InvalidGrantReceipt => write!(f, "invalid verified grant receipt"),
            Self::GrantInactive => write!(f, "grant is not currently active"),
            Self::GrantProofMismatch => write!(f, "verified grant proof does not match grant"),
            Self::GrantIdentity(error) => write!(f, "cannot canonicalize grant identity: {error}"),
            Self::ParentIdentity(error) => write!(f, "cannot canonicalize root grant identity: {error}"),
            Self::ChildIdentity(error) => write!(f, "cannot canonicalize target grant identity: {error}"),
            Self::GrantIdentityProfileMismatch => {
                write!(f, "unexpected canonical grant identity profile")
            }
            Self::GrantFreshnessIdentityMismatch => {
                write!(f, "grant freshness does not bind the exact canonical grant")
            }
            Self::GrantFreshnessPredatesGrant => {
                write!(f, "grant freshness state predates the immutable grant")
            }
            Self::DelegationFreshnessIdentityMismatch => {
                write!(f, "delegation freshness does not bind the exact delegation attestation")
            }
            Self::DelegationFreshnessPredatesAttestation => {
                write!(f, "delegation freshness state predates its attestation")
            }
            Self::FreshnessProfileMismatch => {
                write!(f, "unexpected current-authority freshness profile")
            }
            Self::Freshness(error) => write!(f, "authority freshness qualification failed: {error}"),
            Self::DelegationProofMismatch => {
                write!(f, "verified delegation proof does not match attestation")
            }
            Self::DelegationAuthorityMismatch => {
                write!(f, "verified delegation authority does not match attestation")
            }
            Self::ParentGrantMismatch => {
                write!(f, "child delegated_from does not name the exact parent grant")
            }
            Self::GrantIdentityMismatch => {
                write!(f, "delegation names different parent/child grant IDs")
            }
            Self::GrantDigestMismatch => {
                write!(f, "delegation names different canonical grant digests")
            }
            Self::InstitutionMismatch => write!(f, "delegation institution mismatch"),
            Self::JurisdictionMismatch => write!(f, "delegation jurisdiction mismatch"),
            Self::RulebookMismatch => write!(f, "delegation rulebook mismatch"),
            Self::PrincipalMismatch => {
                write!(f, "delegator/delegate does not match parent/child holder")
            }
            Self::RoleExpansion => write!(f, "delegated child expands parent roles"),
            Self::CapabilityExpansion => write!(f, "delegated child expands parent capabilities"),
            Self::DelegatedRoleMismatch => write!(f, "child roles differ from exact delegated roles"),
            Self::DelegatedCapabilityMismatch => {
                write!(f, "child capabilities differ from exact delegated capabilities")
            }
            Self::AmbiguousDelegationSource => {
                write!(f, "child grant must contain exactly one delegation authority source")
            }
            Self::DelegationSourceMismatch => {
                write!(f, "child delegation source does not bind exact attestation/proof")
            }
            Self::LifetimeExpansion => {
                write!(f, "delegation or child grant extends parent authority lifetime")
            }
            Self::ParentGenerationNotEffectiveAtDelegation => {
                write!(f, "attested parent generation was not effective when delegation was issued")
            }
            Self::GenerationMismatch => {
                write!(f, "delegation does not bind current parent/child generations")
            }
            Self::InvalidVerificationTime => write!(f, "invalid delegation verification time"),
            Self::RootGrantIsDelegated => write!(f, "lineage root must be a non-delegated grant"),
            Self::EmptyDelegationLineage => {
                write!(f, "delegated lineage must contain at least one edge")
            }
            Self::LineageTooDeep => write!(f, "delegation lineage exceeds v0.1 maximum depth"),
            Self::DuplicateLineageChild => {
                write!(f, "multiple delegation edges claim the same child grant")
            }
            Self::DelegationCycle => write!(f, "delegation lineage contains a cycle"),
            Self::MissingLineageEdge => write!(f, "delegation lineage is missing a parent edge"),
            Self::UnexpectedLineageEdge => {
                write!(f, "delegation input contains an unrelated/extraneous edge")
            }
            Self::LineageGrantIdentityMismatch => {
                write!(f, "adjacent delegation edges bind inconsistent grant identities")
            }
            Self::LineageGenerationMismatch => {
                write!(f, "adjacent delegation edges bind different generations of one grant")
            }
            Self::InvalidQualifiedEdge => write!(f, "invalid qualified delegation edge"),
            Self::LineageLeaseExpired => write!(f, "delegation lineage freshness lease is expired"),
        }
    }
}

impl std::error::Error for DelegationError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_freshness::{
        AuthorityFreshnessSnapshot, AuthorityFreshnessState,
        PROTOCOL_VERSION as FRESHNESS_PROTOCOL_VERSION,
    };
    use mycelix_institutional_core::{
        AuthoritySourceRef, RulebookId, PROTOCOL_VERSION as CORE_PROTOCOL_VERSION,
    };

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

    fn root_grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: CORE_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:root").unwrap(),
            holder: PrincipalId::new("did:example:root").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            roles: vec![
                RoleId::new("role:executor").unwrap(),
                RoleId::new("role:reviewer").unwrap(),
            ],
            capabilities: vec![
                CapabilityId::new("governance.execute").unwrap(),
                CapabilityId::new("governance.read").unwrap(),
            ],
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

    fn child_grant(parent: &AuthorityGrant, id: &str, holder: &str) -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: CORE_PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new(id).unwrap(),
            holder: PrincipalId::new(holder).unwrap(),
            institution: parent.institution.clone(),
            jurisdiction: parent.jurisdiction.clone(),
            roles: vec![RoleId::new("role:executor").unwrap()],
            capabilities: vec![CapabilityId::new("governance.execute").unwrap()],
            rulebook: parent.rulebook.clone(),
            sources: vec![AuthoritySourceRef {
                kind: AuthoritySourceKind::Delegation,
                reference: format!("delegation:{id}"),
                proof_ref: format!("proof:delegation:{id}"),
            }],
            issued_at_ms: parent.issued_at_ms + 20,
            expires_at_ms: parent.expires_at_ms - 20,
            delegated_from: Some(parent.id.clone()),
            grant_proof_ref: format!("proof:grant:{id}"),
        }
    }

    fn grant_freshness(grant: &AuthorityGrant, generation: u64) -> VerifiedAuthorityFreshness {
        let identity = authority_grant_identity(grant).unwrap();
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
                subject: AuthoritySubjectRef {
                    kind: AuthoritySubjectKind::AuthorityGrant,
                    namespace: grant.institution.as_str().into(),
                    subject_id: grant.id.as_str().into(),
                    identity: FreshnessProfiledDigest {
                        digest: identity.digest,
                        profile: identity.profile,
                    },
                },
                generation,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: grant.issued_at_ms,
                status_record_ref: format!("status:{}:g{generation}", grant.id.as_str()),
            },
            authoritative_source_ref: "authority-state:1".into(),
            verification_ref: "freshness-verification:1".into(),
            verified_at_ms: 100,
            lease_until_ms: 150,
        }
    }

    fn grant_receipt(grant: AuthorityGrant, generation: u64) -> VerifiedGrantForDelegation {
        let proof = grant.grant_proof_ref.clone();
        VerifiedGrantForDelegation {
            grant: grant.clone(),
            grant_record_ref: format!("grant-record:{}", grant.id.as_str()),
            verified_grant_proof_ref: proof,
            verification_ref: format!("grant-verification:{}", grant.id.as_str()),
            verified_at_ms: 100,
            freshness: grant_freshness(&grant, generation),
        }
    }

    fn delegation_authority() -> DelegationAuthorityBinding {
        DelegationAuthorityBinding {
            authority_ref: "delegation-authority:rulebook:test:1".into(),
            authority_digest: d(9),
            authority_profile: "mycelix-delegation-policy-v1-blake3".into(),
        }
    }

    fn attestation(
        parent: &AuthorityGrant,
        child: &AuthorityGrant,
        parent_generation: u64,
        child_generation: u64,
    ) -> DelegationAttestation {
        let parent_identity = authority_grant_identity(parent).unwrap();
        let child_identity = authority_grant_identity(child).unwrap();
        DelegationAttestation {
            protocol_version: PROTOCOL_VERSION.into(),
            delegation_id: format!("delegation:{}", child.id.as_str()),
            parent_grant_id: parent.id.clone(),
            child_grant_id: child.id.clone(),
            parent_grant_digest: parent_identity.digest,
            child_grant_digest: child_identity.digest,
            parent_generation,
            child_generation,
            delegator: parent.holder.clone(),
            delegate: child.holder.clone(),
            institution: parent.institution.clone(),
            jurisdiction: parent.jurisdiction.clone(),
            rulebook: parent.rulebook.clone(),
            roles: child.roles.clone(),
            capabilities: child.capabilities.clone(),
            delegation_authority: delegation_authority(),
            issued_at_ms: child.issued_at_ms,
            expires_at_ms: child.expires_at_ms,
            proof_ref: format!("proof:delegation:{}", child.id.as_str()),
        }
    }

    fn delegation_receipt(attestation: DelegationAttestation) -> VerifiedDelegationAttestation {
        VerifiedDelegationAttestation {
            verified_proof_ref: attestation.proof_ref.clone(),
            verified_delegation_authority_ref: attestation
                .delegation_authority
                .authority_ref
                .clone(),
            verified_delegation_authority_digest: attestation
                .delegation_authority
                .authority_digest,
            verified_delegation_authority_profile: attestation
                .delegation_authority
                .authority_profile
                .clone(),
            delegation_record_ref: format!("delegation-record:{}", attestation.delegation_id),
            verification_ref: format!("delegation-verification:{}", attestation.delegation_id),
            verified_at_ms: 100,
            attestation,
        }
    }

    fn delegation_freshness(attestation: &DelegationAttestation) -> VerifiedAuthorityFreshness {
        VerifiedAuthorityFreshness {
            snapshot: AuthorityFreshnessSnapshot {
                protocol_version: FRESHNESS_PROTOCOL_VERSION.into(),
                subject: AuthoritySubjectRef {
                    kind: AuthoritySubjectKind::Delegation,
                    namespace: attestation.institution.as_str().into(),
                    subject_id: attestation.delegation_id.clone(),
                    identity: FreshnessProfiledDigest {
                        digest: attestation.identity_digest().unwrap(),
                        profile: DELEGATION_IDENTITY_PROFILE.into(),
                    },
                },
                generation: 1,
                state: AuthorityFreshnessState::Active,
                effective_at_ms: attestation.issued_at_ms,
                status_record_ref: format!("status:{}:g1", attestation.delegation_id),
            },
            authoritative_source_ref: "delegation-state:1".into(),
            verification_ref: "delegation-freshness:1".into(),
            verified_at_ms: 100,
            lease_until_ms: 150,
        }
    }

    fn qualify(parent: &AuthorityGrant, child: &AuthorityGrant) -> QualifiedDelegationEdge {
        let att = attestation(parent, child, 1, 1);
        qualify_delegation_edge(
            &grant_receipt(parent.clone(), 1),
            &grant_receipt(child.clone(), 1),
            &delegation_receipt(att.clone()),
            &delegation_freshness(&att),
            120,
        )
        .unwrap()
    }

    #[test]
    fn exact_attenuated_delegation_qualifies() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let edge = qualify(&parent, &child);
        assert_eq!(edge.parent_grant_id().as_str(), "grant:root");
        assert_eq!(edge.child_grant_id().as_str(), "grant:child");
        assert!(!edge.current_edge_digest().is_zero());
    }

    #[test]
    fn child_cannot_expand_parent_capabilities() {
        let parent = root_grant();
        let mut child = child_grant(&parent, "grant:child", "did:example:child");
        child
            .capabilities
            .push(CapabilityId::new("treasury.admin").unwrap());
        let att = attestation(&parent, &child, 1, 1);
        assert_eq!(
            qualify_delegation_edge(
                &grant_receipt(parent.clone(), 1),
                &grant_receipt(child.clone(), 1),
                &delegation_receipt(att.clone()),
                &delegation_freshness(&att),
                120,
            )
            .unwrap_err(),
            DelegationError::CapabilityExpansion
        );
    }

    #[test]
    fn delegation_requires_independent_authority_to_delegate() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let att = attestation(&parent, &child, 1, 1);
        let mut receipt = delegation_receipt(att.clone());
        receipt.verified_delegation_authority_digest = d(8);
        assert_eq!(
            qualify_delegation_edge(
                &grant_receipt(parent.clone(), 1),
                &grant_receipt(child.clone(), 1),
                &receipt,
                &delegation_freshness(&att),
                120,
            )
            .unwrap_err(),
            DelegationError::DelegationAuthorityMismatch
        );
    }

    #[test]
    fn stale_parent_generation_cannot_be_replayed() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let att = attestation(&parent, &child, 1, 1);
        assert_eq!(
            qualify_delegation_edge(
                &grant_receipt(parent.clone(), 2),
                &grant_receipt(child.clone(), 1),
                &delegation_receipt(att.clone()),
                &delegation_freshness(&att),
                120,
            )
            .unwrap_err(),
            DelegationError::GenerationMismatch
        );
    }

    #[test]
    fn delegation_proof_cannot_be_replayed_to_changed_child() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let mut changed = child.clone();
        changed.holder = PrincipalId::new("did:example:other").unwrap();
        let att = attestation(&parent, &child, 1, 1);
        assert_eq!(
            qualify_delegation_edge(
                &grant_receipt(parent.clone(), 1),
                &grant_receipt(changed.clone(), 1),
                &delegation_receipt(att.clone()),
                &delegation_freshness(&att),
                120,
            )
            .unwrap_err(),
            DelegationError::GrantDigestMismatch
        );
    }

    #[test]
    fn freshness_cannot_predate_grant_or_delegation() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let att = attestation(&parent, &child, 1, 1);

        let mut parent_receipt = grant_receipt(parent.clone(), 1);
        parent_receipt.freshness.snapshot.effective_at_ms = parent.issued_at_ms - 1;
        assert_eq!(
            qualify_delegation_edge(
                &parent_receipt,
                &grant_receipt(child.clone(), 1),
                &delegation_receipt(att.clone()),
                &delegation_freshness(&att),
                120,
            )
            .unwrap_err(),
            DelegationError::GrantFreshnessPredatesGrant
        );

        let mut delegation_state = delegation_freshness(&att);
        delegation_state.snapshot.effective_at_ms = att.issued_at_ms - 1;
        assert_eq!(
            qualify_delegation_edge(
                &grant_receipt(parent.clone(), 1),
                &grant_receipt(child.clone(), 1),
                &delegation_receipt(att.clone()),
                &delegation_state,
                120,
            )
            .unwrap_err(),
            DelegationError::DelegationFreshnessPredatesAttestation
        );
    }

    #[test]
    fn same_generation_reverification_preserves_current_edge_identity() {
        let parent = root_grant();
        let child = child_grant(&parent, "grant:child", "did:example:child");
        let first = qualify(&parent, &child);

        let att = attestation(&parent, &child, 1, 1);
        let mut parent_receipt = grant_receipt(parent.clone(), 1);
        parent_receipt.freshness.verification_ref = "freshness:later-parent".into();
        parent_receipt.freshness.verified_at_ms = 119;
        let mut child_receipt = grant_receipt(child.clone(), 1);
        child_receipt.freshness.verification_ref = "freshness:later-child".into();
        child_receipt.freshness.verified_at_ms = 119;
        let mut delegation_state = delegation_freshness(&att);
        delegation_state.verification_ref = "freshness:later-delegation".into();
        delegation_state.verified_at_ms = 119;

        let second = qualify_delegation_edge(
            &parent_receipt,
            &child_receipt,
            &delegation_receipt(att),
            &delegation_state,
            120,
        )
        .unwrap();
        assert_eq!(first.current_edge_digest(), second.current_edge_digest());
    }

    #[test]
    fn complete_lineage_is_input_order_independent() {
        let root = root_grant();
        let mid = child_grant(&root, "grant:mid", "did:example:mid");
        let mut leaf = child_grant(&mid, "grant:leaf", "did:example:leaf");
        leaf.expires_at_ms = 150;

        let root_to_mid = qualify(&root, &mid);
        let mid_to_leaf = qualify(&mid, &leaf);
        let first = qualify_complete_delegation_lineage(
            &root,
            &leaf,
            &[root_to_mid.clone(), mid_to_leaf.clone()],
            120,
        )
        .unwrap();
        let second = qualify_complete_delegation_lineage(
            &root,
            &leaf,
            &[mid_to_leaf, root_to_mid],
            120,
        )
        .unwrap();
        assert_eq!(first.depth(), 2);
        assert_eq!(first.lineage_digest(), second.lineage_digest());
        assert_eq!(first.lineage_profile(), LINEAGE_PROFILE);
    }

    #[test]
    fn mixed_intermediate_generations_fail_closed() {
        let root = root_grant();
        let mid = child_grant(&root, "grant:mid", "did:example:mid");
        let leaf = child_grant(&mid, "grant:leaf", "did:example:leaf");
        let root_to_mid = qualify(&root, &mid);
        let mut mid_to_leaf = qualify(&mid, &leaf);
        mid_to_leaf.parent_generation = 2;
        assert_eq!(
            qualify_complete_delegation_lineage(&root, &leaf, &[root_to_mid, mid_to_leaf], 120)
                .unwrap_err(),
            DelegationError::LineageGenerationMismatch
        );
    }

    #[test]
    fn unrelated_extra_edge_fails_closed() {
        let root = root_grant();
        let child = child_grant(&root, "grant:child", "did:example:child");
        let other = child_grant(&root, "grant:other", "did:example:other");
        let child_edge = qualify(&root, &child);
        let other_edge = qualify(&root, &other);
        assert_eq!(
            qualify_complete_delegation_lineage(
                &root,
                &child,
                &[child_edge, other_edge],
                120,
            )
            .unwrap_err(),
            DelegationError::UnexpectedLineageEdge
        );
    }
}
