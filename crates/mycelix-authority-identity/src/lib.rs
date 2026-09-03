// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical semantic identities for institutional authority objects.
//!
//! The institutional core intentionally stays wire-neutral. This crate defines
//! cryptographic semantic identities over those objects so freshness, delegation,
//! executor, and audit adapters do not invent incompatible same-ID hashing rules.

use mycelix_institutional_core::{
    AuthorityGrant, AuthoritySourceKind, Digest32, ValidationError,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const AUTHORITY_GRANT_IDENTITY_PROFILE: &str =
    "mycelix-authority-grant-v1-blake3-framed-semantic";
const DOMAIN_AUTHORITY_GRANT: &[u8] = b"mycelix/authority-identity/grant/v1";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CanonicalAuthorityIdentity {
    pub digest: Digest32,
    pub profile: String,
}

/// Return the canonical semantic identity of one institutional AuthorityGrant.
///
/// Roles, capabilities, and authority sources are semantically set-like in the
/// institutional evaluator, so their serialized order does not affect identity.
/// Duplicate set members are rejected rather than silently increasing apparent
/// authority or creating multiple byte representations of the same semantics.
pub fn authority_grant_identity(
    grant: &AuthorityGrant,
) -> Result<CanonicalAuthorityIdentity, AuthorityIdentityError> {
    grant
        .validate()
        .map_err(AuthorityIdentityError::InvalidAuthorityGrant)?;

    let roles = canonical_text_set(
        grant.roles.iter().map(|role| role.as_str()),
        AuthorityIdentityError::DuplicateRole,
    )?;
    let capabilities = canonical_text_set(
        grant.capabilities.iter().map(|capability| capability.as_str()),
        AuthorityIdentityError::DuplicateCapability,
    )?;

    let mut sources = grant
        .sources
        .iter()
        .map(|source| {
            (
                source_kind_code(&source.kind),
                source.reference.as_str(),
                source.proof_ref.as_str(),
            )
        })
        .collect::<Vec<_>>();
    sources.sort_unstable();
    if sources.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(AuthorityIdentityError::DuplicateAuthoritySource);
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_AUTHORITY_GRANT);
    frame(&mut hasher, AUTHORITY_GRANT_IDENTITY_PROFILE.as_bytes());
    frame(&mut hasher, grant.protocol_version.as_bytes());
    frame(&mut hasher, grant.id.as_str().as_bytes());
    frame(&mut hasher, grant.holder.as_str().as_bytes());
    frame(&mut hasher, grant.institution.as_str().as_bytes());
    frame_optional_text(
        &mut hasher,
        grant.jurisdiction.as_ref().map(|value| value.as_str()),
    );

    frame(&mut hasher, &(roles.len() as u64).to_le_bytes());
    for role in roles {
        frame(&mut hasher, role.as_bytes());
    }

    frame(
        &mut hasher,
        &(capabilities.len() as u64).to_le_bytes(),
    );
    for capability in capabilities {
        frame(&mut hasher, capability.as_bytes());
    }

    frame(&mut hasher, grant.rulebook.id.as_str().as_bytes());
    frame(&mut hasher, grant.rulebook.version.as_bytes());
    frame(&mut hasher, &grant.rulebook.digest.0);

    frame(&mut hasher, &(sources.len() as u64).to_le_bytes());
    for (kind, reference, proof_ref) in sources {
        frame(&mut hasher, &[kind]);
        frame(&mut hasher, reference.as_bytes());
        frame(&mut hasher, proof_ref.as_bytes());
    }

    frame(&mut hasher, &grant.issued_at_ms.to_le_bytes());
    frame(&mut hasher, &grant.expires_at_ms.to_le_bytes());
    frame_optional_text(
        &mut hasher,
        grant.delegated_from.as_ref().map(|value| value.as_str()),
    );
    frame(&mut hasher, grant.grant_proof_ref.as_bytes());

    Ok(CanonicalAuthorityIdentity {
        digest: Digest32(*hasher.finalize().as_bytes()),
        profile: AUTHORITY_GRANT_IDENTITY_PROFILE.into(),
    })
}

fn canonical_text_set<'a>(
    values: impl Iterator<Item = &'a str>,
    duplicate_error: AuthorityIdentityError,
) -> Result<Vec<&'a str>, AuthorityIdentityError> {
    let values = values.collect::<Vec<_>>();
    let unique = values.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != values.len() {
        return Err(duplicate_error);
    }
    Ok(unique.into_iter().collect())
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

#[derive(Debug)]
pub enum AuthorityIdentityError {
    InvalidAuthorityGrant(ValidationError),
    DuplicateRole,
    DuplicateCapability,
    DuplicateAuthoritySource,
}

impl PartialEq for AuthorityIdentityError {
    fn eq(&self, other: &Self) -> bool {
        matches!(
            (self, other),
            (Self::DuplicateRole, Self::DuplicateRole)
                | (Self::DuplicateCapability, Self::DuplicateCapability)
                | (
                    Self::DuplicateAuthoritySource,
                    Self::DuplicateAuthoritySource
                )
        ) || matches!(
            (self, other),
            (Self::InvalidAuthorityGrant(_), Self::InvalidAuthorityGrant(_))
        )
    }
}

impl Eq for AuthorityIdentityError {}

impl fmt::Display for AuthorityIdentityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAuthorityGrant(error) => write!(f, "invalid authority grant: {error}"),
            Self::DuplicateRole => write!(f, "authority grant contains a duplicate role"),
            Self::DuplicateCapability => {
                write!(f, "authority grant contains a duplicate capability")
            }
            Self::DuplicateAuthoritySource => {
                write!(f, "authority grant contains a duplicate authority source")
            }
        }
    }
}

impl std::error::Error for AuthorityIdentityError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{
        AuthorityGrantId, AuthoritySourceRef, CapabilityId, InstitutionId, PrincipalId, RoleId,
        RulebookId, RulebookRef, PROTOCOL_VERSION,
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

    fn grant() -> AuthorityGrant {
        AuthorityGrant {
            protocol_version: PROTOCOL_VERSION.into(),
            id: AuthorityGrantId::new("grant:child").unwrap(),
            holder: PrincipalId::new("did:example:holder").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            roles: vec![
                RoleId::new("role:operator").unwrap(),
                RoleId::new("role:reviewer").unwrap(),
            ],
            capabilities: vec![
                CapabilityId::new("governance.execute").unwrap(),
                CapabilityId::new("governance.read").unwrap(),
            ],
            rulebook: rulebook(),
            sources: vec![
                AuthoritySourceRef {
                    kind: AuthoritySourceKind::GovernanceDecision,
                    reference: "decision:1".into(),
                    proof_ref: "proof:decision:1".into(),
                },
                AuthoritySourceRef {
                    kind: AuthoritySourceKind::Agreement,
                    reference: "agreement:1".into(),
                    proof_ref: "proof:agreement:1".into(),
                },
            ],
            issued_at_ms: 10,
            expires_at_ms: 100,
            delegated_from: None,
            grant_proof_ref: "proof:grant:1".into(),
        }
    }

    #[test]
    fn reordered_set_like_fields_preserve_semantic_identity() {
        let first = grant();
        let mut second = first.clone();
        second.roles.reverse();
        second.capabilities.reverse();
        second.sources.reverse();
        assert_eq!(
            authority_grant_identity(&first).unwrap(),
            authority_grant_identity(&second).unwrap()
        );
    }

    #[test]
    fn same_id_changed_authority_changes_identity() {
        let first = grant();
        let mut second = first.clone();
        second.holder = PrincipalId::new("did:example:other").unwrap();
        assert_ne!(
            authority_grant_identity(&first).unwrap().digest,
            authority_grant_identity(&second).unwrap().digest
        );

        let mut third = first.clone();
        third.capabilities.pop();
        assert_ne!(
            authority_grant_identity(&first).unwrap().digest,
            authority_grant_identity(&third).unwrap().digest
        );
    }

    #[test]
    fn delegation_parent_is_part_of_identity() {
        let first = grant();
        let mut delegated = first.clone();
        delegated.delegated_from = Some(AuthorityGrantId::new("grant:parent").unwrap());
        assert_ne!(
            authority_grant_identity(&first).unwrap().digest,
            authority_grant_identity(&delegated).unwrap().digest
        );
    }

    #[test]
    fn rulebook_and_proof_provenance_are_part_of_identity() {
        let first = grant();
        let mut changed_rulebook = first.clone();
        changed_rulebook.rulebook.digest = d(9);
        assert_ne!(
            authority_grant_identity(&first).unwrap().digest,
            authority_grant_identity(&changed_rulebook).unwrap().digest
        );

        let mut changed_proof = first.clone();
        changed_proof.grant_proof_ref = "proof:grant:2".into();
        assert_ne!(
            authority_grant_identity(&first).unwrap().digest,
            authority_grant_identity(&changed_proof).unwrap().digest
        );
    }

    #[test]
    fn duplicate_set_members_are_rejected() {
        let mut duplicate_capability = grant();
        duplicate_capability
            .capabilities
            .push(CapabilityId::new("governance.execute").unwrap());
        assert_eq!(
            authority_grant_identity(&duplicate_capability).unwrap_err(),
            AuthorityIdentityError::DuplicateCapability
        );

        let mut duplicate_role = grant();
        duplicate_role
            .roles
            .push(RoleId::new("role:operator").unwrap());
        assert_eq!(
            authority_grant_identity(&duplicate_role).unwrap_err(),
            AuthorityIdentityError::DuplicateRole
        );

        let mut duplicate_source = grant();
        duplicate_source.sources.push(duplicate_source.sources[0].clone());
        assert_eq!(
            authority_grant_identity(&duplicate_source).unwrap_err(),
            AuthorityIdentityError::DuplicateAuthoritySource
        );
    }
}
