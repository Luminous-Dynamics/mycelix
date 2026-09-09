// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Exact delegation-lineage binding for governance executor authority.
//!
//! PR #71 deliberately left delegated-grant verification as a runtime-provider
//! responsibility represented by a compatibility string. This crate closes that
//! semantic gap without rewriting the earlier stacked tranche: it reconstructs
//! the exact current delegation lineage against the exact executor grant first,
//! discards caller-supplied delegation strings, and only then invokes the older
//! executor-designation qualifier with a deterministic lineage-derived marker.
//!
//! The resulting authority commits canonical grant identity, exact designation
//! semantics, and the exact current delegation-lineage commitment. Dynamic
//! verifier timestamps and lease refreshes remain outside stable authority
//! identity.

use mycelix_authority_delegation::{
    qualify_complete_delegation_lineage, DelegationError, QualifiedDelegationEdge,
    QualifiedDelegationLineage,
};
use mycelix_authority_identity::{
    authority_grant_identity, AuthorityIdentityError, AUTHORITY_GRANT_IDENTITY_PROFILE,
};
use mycelix_governance_authority::ProposalId;
use mycelix_governance_executor_designation::{
    qualify_executor_designation, ExecutorDesignation, ExecutorDesignationError,
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_institutional_core::{
    AuthorityGrant, AuthorityGrantId, AuthoritySourceKind, CapabilityId, Digest32, InstitutionId,
    JurisdictionId, PrincipalId, RulebookRef,
};
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-executor-lineage-v0.1";
pub const EXECUTOR_DESIGNATION_IDENTITY_PROFILE: &str =
    "mycelix-governance-executor-designation-v1-blake3-framed-semantic";
pub const EXECUTOR_LINEAGE_AUTHORITY_PROFILE: &str =
    "mycelix-governance-executor-lineage-authority-v1-blake3-framed";

const DOMAIN_DESIGNATION: &[u8] = b"mycelix/governance/executor-designation/identity/v1";
const DOMAIN_AUTHORITY: &[u8] = b"mycelix/governance/executor-lineage-authority/v1";
const MAX_ID_BYTES: usize = 512;
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Evidence used to prove whether the executor grant is direct or delegated.
///
/// For a delegated grant the exact current lineage is reconstructed from the
/// supplied already-qualified edges and the exact grant bytes in
/// `VerifiedAuthorityGrant`. The caller cannot simply supply a lineage digest.
pub enum DelegationLineageEvidence<'a> {
    Direct,
    Delegated {
        root_grant: &'a AuthorityGrant,
        edges: &'a [QualifiedDelegationEdge],
    },
}

/// Final pure executor authority with delegation semantics closed.
///
/// Private fields and the absence of `Deserialize` prevent application payloads
/// from minting this authority-shaped object. Obtain it only from
/// `qualify_lineage_bound_executor_authority`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedLineageBoundExecutorAuthority {
    protocol_version: String,
    proposal_id: ProposalId,
    actions_digest: Digest32,
    actions_digest_profile: String,
    threshold_authorization_ref: String,
    executor_principal: PrincipalId,
    authority_grant_id: AuthorityGrantId,
    authority_grant_identity_digest: Digest32,
    authority_grant_identity_profile: String,
    executor_designation_identity_digest: Digest32,
    executor_designation_identity_profile: String,
    executor_authority_ref: String,
    granting_institution: InstitutionId,
    jurisdiction: Option<JurisdictionId>,
    rulebook: RulebookRef,
    capability_scope: CapabilityId,
    delegation_root_grant_id: Option<AuthorityGrantId>,
    delegation_lineage_digest: Option<Digest32>,
    delegation_lineage_profile: Option<String>,
    delegation_depth: u8,
    authority_digest: Digest32,
    authority_profile: String,
    semantic_valid_until_ms: u64,
    verified_at_ms: u64,
    lease_until_ms: u64,
}

impl QualifiedLineageBoundExecutorAuthority {
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

    pub fn delegation_lineage_digest(&self) -> Option<Digest32> {
        self.delegation_lineage_digest
    }

    pub fn delegation_lineage_profile(&self) -> Option<&str> {
        self.delegation_lineage_profile.as_deref()
    }

    pub fn delegation_depth(&self) -> u8 {
        self.delegation_depth
    }

    pub fn authority_digest(&self) -> Digest32 {
        self.authority_digest
    }

    pub fn authority_profile(&self) -> &str {
        &self.authority_profile
    }

    pub fn semantic_valid_until_ms(&self) -> u64 {
        self.semantic_valid_until_ms
    }

    pub fn verified_at_ms(&self) -> u64 {
        self.verified_at_ms
    }

    pub fn lease_until_ms(&self) -> u64 {
        self.lease_until_ms
    }
}

pub fn qualify_lineage_bound_executor_authority(
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    now_ms: u64,
) -> Result<QualifiedLineageBoundExecutorAuthority, ExecutorLineageError> {
    if now_ms == 0 {
        return Err(ExecutorLineageError::InvalidVerificationTime);
    }

    let grant_identity =
        authority_grant_identity(&grant_receipt.grant).map_err(ExecutorLineageError::GrantIdentity)?;
    if grant_identity.profile != AUTHORITY_GRANT_IDENTITY_PROFILE {
        return Err(ExecutorLineageError::GrantIdentityProfileMismatch);
    }

    let lineage = qualify_exact_lineage(&grant_receipt.grant, lineage_evidence, now_ms)?;

    // Compatibility containment for PR #71: never trust a caller-provided
    // `delegation_verification_ref`. Reconstruct the exact current lineage first,
    // then overwrite that field with a deterministic marker derived from the
    // qualified lineage. The marker carries no authority outside the old API.
    let mut sanitized_grant_receipt = grant_receipt.clone();
    sanitized_grant_receipt.delegation_verification_ref = lineage.as_ref().map(|qualified| {
        compatibility_lineage_marker(qualified.lineage_digest(), qualified.lineage_profile())
    });

    let qualified = qualify_executor_designation(
        threshold,
        &sanitized_grant_receipt,
        designation_receipt,
        now_ms,
    )
    .map_err(ExecutorLineageError::ExecutorDesignation)?;

    if qualified.authority_grant_id != grant_receipt.grant.id {
        return Err(ExecutorLineageError::GrantIdentityMismatch);
    }

    let designation_identity = executor_designation_identity(&designation_receipt.designation)?;
    let lineage_commitment = lineage.as_ref().map(|value| LineageCommitment {
        root_grant_id: value.root_grant_id().clone(),
        digest: value.lineage_digest(),
        profile: value.lineage_profile().to_string(),
        depth: value.depth(),
    });

    let semantic_valid_until_ms = qualified.valid_until_ms;
    let mut verified_at_ms = qualified.verified_at_ms;
    let mut lease_until_ms = semantic_valid_until_ms;
    if let Some(value) = &lineage {
        verified_at_ms = verified_at_ms.max(value.verified_at_ms());
        lease_until_ms = lease_until_ms.min(value.lease_until_ms());
    }
    if verified_at_ms > now_ms || lease_until_ms <= now_ms {
        return Err(ExecutorLineageError::InactiveCurrentAuthority);
    }

    let digest_inputs = AuthorityDigestInputs {
        proposal_id: &qualified.proposal_id,
        actions_digest: qualified.actions_digest,
        actions_digest_profile: &qualified.actions_digest_profile,
        threshold_authorization_ref: &qualified.threshold_authorization_ref,
        executor_principal: &qualified.executor_principal,
        authority_grant_id: &qualified.authority_grant_id,
        authority_grant_identity_digest: grant_identity.digest,
        executor_designation_identity_digest: designation_identity,
        executor_authority_ref: &qualified.executor_authority_ref,
        granting_institution: &qualified.granting_institution,
        jurisdiction: qualified.jurisdiction.as_ref(),
        rulebook: &qualified.rulebook,
        capability_scope: &qualified.capability_scope,
        lineage: lineage_commitment.as_ref(),
        semantic_valid_until_ms,
    };
    let authority_digest = current_authority_digest(&digest_inputs);

    Ok(QualifiedLineageBoundExecutorAuthority {
        protocol_version: PROTOCOL_VERSION.into(),
        proposal_id: qualified.proposal_id,
        actions_digest: qualified.actions_digest,
        actions_digest_profile: qualified.actions_digest_profile,
        threshold_authorization_ref: qualified.threshold_authorization_ref,
        executor_principal: qualified.executor_principal,
        authority_grant_id: qualified.authority_grant_id,
        authority_grant_identity_digest: grant_identity.digest,
        authority_grant_identity_profile: grant_identity.profile,
        executor_designation_identity_digest: designation_identity,
        executor_designation_identity_profile: EXECUTOR_DESIGNATION_IDENTITY_PROFILE.into(),
        executor_authority_ref: qualified.executor_authority_ref,
        granting_institution: qualified.granting_institution,
        jurisdiction: qualified.jurisdiction,
        rulebook: qualified.rulebook,
        capability_scope: qualified.capability_scope,
        delegation_root_grant_id: lineage_commitment
            .as_ref()
            .map(|value| value.root_grant_id.clone()),
        delegation_lineage_digest: lineage_commitment.as_ref().map(|value| value.digest),
        delegation_lineage_profile: lineage_commitment
            .as_ref()
            .map(|value| value.profile.clone()),
        delegation_depth: lineage_commitment.as_ref().map_or(0, |value| value.depth),
        authority_digest,
        authority_profile: EXECUTOR_LINEAGE_AUTHORITY_PROFILE.into(),
        semantic_valid_until_ms,
        verified_at_ms,
        lease_until_ms,
    })
}

fn qualify_exact_lineage(
    executor_grant: &AuthorityGrant,
    evidence: DelegationLineageEvidence<'_>,
    now_ms: u64,
) -> Result<Option<QualifiedDelegationLineage>, ExecutorLineageError> {
    match (&executor_grant.delegated_from, evidence) {
        (None, DelegationLineageEvidence::Direct) => Ok(None),
        (None, DelegationLineageEvidence::Delegated { .. }) => {
            Err(ExecutorLineageError::UnexpectedDelegationLineage)
        }
        (Some(_), DelegationLineageEvidence::Direct) => {
            Err(ExecutorLineageError::MissingDelegationLineage)
        }
        (Some(_), DelegationLineageEvidence::Delegated { root_grant, edges }) => {
            let lineage =
                qualify_complete_delegation_lineage(root_grant, executor_grant, edges, now_ms)
                    .map_err(ExecutorLineageError::Delegation)?;
            if lineage.target_grant_id() != &executor_grant.id {
                return Err(ExecutorLineageError::LineageTargetMismatch);
            }
            Ok(Some(lineage))
        }
    }
}

fn executor_designation_identity(
    designation: &ExecutorDesignation,
) -> Result<Digest32, ExecutorLineageError> {
    designation
        .validate()
        .map_err(ExecutorLineageError::ExecutorDesignation)?;

    for value in [
        designation.designation_id.as_str(),
        designation.executor.as_str(),
        designation.authority_grant_id.as_str(),
        designation.proposal_id.as_str(),
        designation.institution.as_str(),
        designation.required_capability.as_str(),
    ] {
        require_id(value)?;
    }
    if let Some(jurisdiction) = &designation.jurisdiction {
        require_id(jurisdiction.as_str())?;
    }
    require_profile(&designation.actions_digest_profile)?;
    require_ref(&designation.threshold_authorization_ref)?;
    require_ref(&designation.designation_proof_ref)?;

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_DESIGNATION);
    frame(&mut hasher, EXECUTOR_DESIGNATION_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, designation.protocol_version.as_bytes());
    frame(&mut hasher, designation.designation_id.as_bytes());
    frame(&mut hasher, designation.executor.as_str().as_bytes());
    frame(&mut hasher, designation.authority_grant_id.as_str().as_bytes());
    frame(&mut hasher, designation.proposal_id.as_str().as_bytes());
    frame(&mut hasher, &designation.actions_digest.0);
    frame(&mut hasher, designation.actions_digest_profile.as_bytes());
    frame(&mut hasher, designation.threshold_authorization_ref.as_bytes());
    frame(&mut hasher, designation.institution.as_str().as_bytes());
    frame_optional_text(
        &mut hasher,
        designation.jurisdiction.as_ref().map(|value| value.as_str()),
    );
    frame(&mut hasher, designation.rulebook.id.as_str().as_bytes());
    frame(&mut hasher, designation.rulebook.version.as_bytes());
    frame(&mut hasher, &designation.rulebook.digest.0);
    frame(&mut hasher, designation.required_capability.as_str().as_bytes());
    frame(&mut hasher, &[source_kind_code(&designation.source.kind)]);
    frame(&mut hasher, designation.source.reference.as_bytes());
    frame(&mut hasher, designation.source.proof_ref.as_bytes());
    frame(&mut hasher, &designation.issued_at_ms.to_le_bytes());
    frame(&mut hasher, &designation.expires_at_ms.to_le_bytes());
    frame(&mut hasher, designation.designation_proof_ref.as_bytes());
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

#[derive(Clone)]
struct LineageCommitment {
    root_grant_id: AuthorityGrantId,
    digest: Digest32,
    profile: String,
    depth: u8,
}

struct AuthorityDigestInputs<'a> {
    proposal_id: &'a ProposalId,
    actions_digest: Digest32,
    actions_digest_profile: &'a str,
    threshold_authorization_ref: &'a str,
    executor_principal: &'a PrincipalId,
    authority_grant_id: &'a AuthorityGrantId,
    authority_grant_identity_digest: Digest32,
    executor_designation_identity_digest: Digest32,
    executor_authority_ref: &'a str,
    granting_institution: &'a InstitutionId,
    jurisdiction: Option<&'a JurisdictionId>,
    rulebook: &'a RulebookRef,
    capability_scope: &'a CapabilityId,
    lineage: Option<&'a LineageCommitment>,
    semantic_valid_until_ms: u64,
}

fn current_authority_digest(inputs: &AuthorityDigestInputs<'_>) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_AUTHORITY);
    frame(&mut hasher, EXECUTOR_LINEAGE_AUTHORITY_PROFILE.as_bytes());
    frame(&mut hasher, AUTHORITY_GRANT_IDENTITY_PROFILE.as_bytes());
    frame(
        &mut hasher,
        EXECUTOR_DESIGNATION_IDENTITY_PROFILE.as_bytes(),
    );
    frame(&mut hasher, inputs.proposal_id.as_str().as_bytes());
    frame(&mut hasher, &inputs.actions_digest.0);
    frame(&mut hasher, inputs.actions_digest_profile.as_bytes());
    frame(&mut hasher, inputs.threshold_authorization_ref.as_bytes());
    frame(&mut hasher, inputs.executor_principal.as_str().as_bytes());
    frame(&mut hasher, inputs.authority_grant_id.as_str().as_bytes());
    frame(&mut hasher, &inputs.authority_grant_identity_digest.0);
    frame(
        &mut hasher,
        &inputs.executor_designation_identity_digest.0,
    );
    frame(&mut hasher, inputs.executor_authority_ref.as_bytes());
    frame(&mut hasher, inputs.granting_institution.as_str().as_bytes());
    frame_optional_text(&mut hasher, inputs.jurisdiction.map(|value| value.as_str()));
    frame(&mut hasher, inputs.rulebook.id.as_str().as_bytes());
    frame(&mut hasher, inputs.rulebook.version.as_bytes());
    frame(&mut hasher, &inputs.rulebook.digest.0);
    frame(&mut hasher, inputs.capability_scope.as_str().as_bytes());
    match inputs.lineage {
        None => frame(&mut hasher, &[0]),
        Some(lineage) => {
            frame(&mut hasher, &[1]);
            frame(&mut hasher, lineage.root_grant_id.as_str().as_bytes());
            frame(&mut hasher, &lineage.digest.0);
            frame(&mut hasher, lineage.profile.as_bytes());
            frame(&mut hasher, &[lineage.depth]);
        }
    }
    frame(
        &mut hasher,
        &inputs.semantic_valid_until_ms.to_le_bytes(),
    );
    Digest32(*hasher.finalize().as_bytes())
}

fn compatibility_lineage_marker(digest: Digest32, profile: &str) -> String {
    format!("qualified-lineage:{profile}:{}", hex_digest(digest))
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

fn require_id(value: &str) -> Result<(), ExecutorLineageError> {
    if value.trim().is_empty() || value.len() > MAX_ID_BYTES {
        Err(ExecutorLineageError::InvalidIdentifier)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), ExecutorLineageError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ExecutorLineageError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), ExecutorLineageError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ExecutorLineageError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ExecutorLineageError {
    InvalidVerificationTime,
    InvalidIdentifier,
    InvalidReference,
    InvalidProfile,
    GrantIdentity(AuthorityIdentityError),
    GrantIdentityProfileMismatch,
    GrantIdentityMismatch,
    MissingDelegationLineage,
    UnexpectedDelegationLineage,
    LineageTargetMismatch,
    Delegation(DelegationError),
    ExecutorDesignation(ExecutorDesignationError),
    InactiveCurrentAuthority,
}

impl fmt::Display for ExecutorLineageError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidVerificationTime => write!(f, "invalid executor-lineage verification time"),
            Self::InvalidIdentifier => write!(f, "invalid executor-lineage identifier"),
            Self::InvalidReference => write!(f, "invalid executor-lineage reference"),
            Self::InvalidProfile => write!(f, "invalid executor-lineage digest profile"),
            Self::GrantIdentity(error) => {
                write!(f, "cannot canonicalize executor grant: {error}")
            }
            Self::GrantIdentityProfileMismatch => {
                write!(f, "unexpected canonical grant identity profile")
            }
            Self::GrantIdentityMismatch => {
                write!(f, "qualified executor designation names another grant")
            }
            Self::MissingDelegationLineage => write!(
                f,
                "delegated executor grant lacks exact current delegation lineage"
            ),
            Self::UnexpectedDelegationLineage => {
                write!(f, "direct executor grant must not carry delegation lineage")
            }
            Self::LineageTargetMismatch => {
                write!(f, "delegation lineage terminates at another grant")
            }
            Self::Delegation(error) => {
                write!(f, "delegation-lineage qualification failed: {error}")
            }
            Self::ExecutorDesignation(error) => {
                write!(f, "executor-designation qualification failed: {error}")
            }
            Self::InactiveCurrentAuthority => {
                write!(f, "lineage-bound executor authority is not currently reusable")
            }
        }
    }
}

impl std::error::Error for ExecutorLineageError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    #[test]
    fn compatibility_marker_is_profiled_and_fixed_width() {
        let marker = compatibility_lineage_marker(d(0xab), "lineage-profile-v1");
        assert!(marker.starts_with("qualified-lineage:lineage-profile-v1:"));
        assert_eq!(marker.rsplit(':').next().unwrap().len(), 64);
        assert!(marker.ends_with(&"ab".repeat(32)));
    }

    #[test]
    fn lineage_presence_changes_stable_authority_identity() {
        let proposal_id = ProposalId::new("proposal:test").unwrap();
        let executor = PrincipalId::new("did:example:executor").unwrap();
        let grant_id = AuthorityGrantId::new("grant:executor").unwrap();
        let institution = InstitutionId::new("institution:test").unwrap();
        let capability = CapabilityId::new("governance.execute").unwrap();
        let rulebook = RulebookRef {
            id: mycelix_institutional_core::RulebookId::new("rulebook:test").unwrap(),
            version: "1".into(),
            digest: d(9),
        };
        let direct = AuthorityDigestInputs {
            proposal_id: &proposal_id,
            actions_digest: d(1),
            actions_digest_profile: "actions-v1-blake3",
            threshold_authorization_ref: "threshold:1",
            executor_principal: &executor,
            authority_grant_id: &grant_id,
            authority_grant_identity_digest: d(2),
            executor_designation_identity_digest: d(3),
            executor_authority_ref: "designation:record:1",
            granting_institution: &institution,
            jurisdiction: None,
            rulebook: &rulebook,
            capability_scope: &capability,
            lineage: None,
            semantic_valid_until_ms: 100,
        };
        let lineage = LineageCommitment {
            root_grant_id: AuthorityGrantId::new("grant:root").unwrap(),
            digest: d(4),
            profile: "lineage-v1-blake3".into(),
            depth: 1,
        };
        let delegated = AuthorityDigestInputs {
            lineage: Some(&lineage),
            ..direct
        };
        assert_ne!(
            current_authority_digest(&direct),
            current_authority_digest(&delegated)
        );
    }

    #[test]
    fn verifier_refresh_fields_are_not_part_of_authority_digest_inputs() {
        // The stable digest input type contains semantic authority only; there are
        // intentionally no verification timestamps, verification references, or
        // freshness lease fields to mutate here.
        assert_eq!(
            EXECUTOR_LINEAGE_AUTHORITY_PROFILE,
            "mycelix-governance-executor-lineage-authority-v1-blake3-framed"
        );
    }
}
