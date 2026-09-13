#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Pure FIN-SAFE-030 TEND authority policy.
//!
//! This crate owns no Holochain entry types, callbacks, links, externs, or
//! persistence authority. It defines only the immutable policy model that a
//! DNA-bound integrity zome can later consume.

use serde::{Deserialize, Serialize};

pub const TEND_AUTHORITY_SCHEMA_VERSION_V1: u16 = 1;
pub const MAX_TEND_AUTHORITY_DIDS_PER_ROLE: usize = 64;
pub const MAX_TEND_AUTHORITY_DID_LEN: usize = 512;
pub const MAX_TEND_AUTHORITY_ROOT_ID_LEN: usize = 128;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TendAuthorityConfig {
    pub enabled: bool,
    pub root: Option<TendAuthorityRootV1>,
}

impl TendAuthorityConfig {
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            root: None,
        }
    }

    pub fn validate(&self) -> Result<(), TendAuthorityError> {
        match (self.enabled, self.root.as_ref()) {
            (false, None) => Ok(()),
            (false, Some(_)) => Err(TendAuthorityError::DisabledConfigCarriesRoot),
            (true, None) => Err(TendAuthorityError::EnabledConfigMissingRoot),
            (true, Some(root)) => root.validate(),
        }
    }

    pub fn require_enabled(&self) -> Result<&TendAuthorityRootV1, TendAuthorityError> {
        self.validate()?;
        if !self.enabled {
            return Err(TendAuthorityError::AuthorityDisabled);
        }
        self.root
            .as_ref()
            .ok_or(TendAuthorityError::EnabledConfigMissingRoot)
    }

    pub fn authorize_oracle(&self, author_did: &str) -> Result<(), TendAuthorityError> {
        self.require_enabled()?.authorize_oracle(author_did)
    }

    pub fn authorize_governance(&self, author_did: &str) -> Result<(), TendAuthorityError> {
        self.require_enabled()?.authorize_governance(author_did)
    }
}

impl Default for TendAuthorityConfig {
    fn default() -> Self {
        Self::disabled()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TendAuthorityRootV1 {
    pub schema_version: u16,
    pub root_id: String,
    pub policy_version: u64,
    pub allow_role_overlap: bool,
    pub oracle_authority_dids: Vec<String>,
    pub governance_authority_dids: Vec<String>,
}

impl TendAuthorityRootV1 {
    pub fn validate(&self) -> Result<(), TendAuthorityError> {
        if self.schema_version != TEND_AUTHORITY_SCHEMA_VERSION_V1 {
            return Err(TendAuthorityError::UnsupportedSchemaVersion {
                actual: self.schema_version,
            });
        }
        validate_root_id(&self.root_id)?;
        if self.policy_version == 0 {
            return Err(TendAuthorityError::ZeroPolicyVersion);
        }
        validate_role_dids(AuthorityRole::Oracle, &self.oracle_authority_dids)?;
        validate_role_dids(AuthorityRole::Governance, &self.governance_authority_dids)?;

        if !self.allow_role_overlap
            && self
                .oracle_authority_dids
                .iter()
                .any(|did| self.governance_authority_dids.binary_search(did).is_ok())
        {
            return Err(TendAuthorityError::RoleOverlapNotAllowed);
        }

        Ok(())
    }

    pub fn authorize_oracle(&self, author_did: &str) -> Result<(), TendAuthorityError> {
        self.validate()?;
        validate_author_did(author_did)?;
        if self
            .oracle_authority_dids
            .binary_search_by(|candidate| candidate.as_str().cmp(author_did))
            .is_ok()
        {
            Ok(())
        } else {
            Err(TendAuthorityError::UnauthorizedOracleAuthor)
        }
    }

    pub fn authorize_governance(&self, author_did: &str) -> Result<(), TendAuthorityError> {
        self.validate()?;
        validate_author_did(author_did)?;
        if self
            .governance_authority_dids
            .binary_search_by(|candidate| candidate.as_str().cmp(author_did))
            .is_ok()
        {
            Ok(())
        } else {
            Err(TendAuthorityError::UnauthorizedGovernanceAuthor)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AuthorityRole {
    Oracle,
    Governance,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TendAuthorityError {
    DisabledConfigCarriesRoot,
    EnabledConfigMissingRoot,
    AuthorityDisabled,
    UnsupportedSchemaVersion { actual: u16 },
    EmptyRootId,
    NonCanonicalRootId,
    RootIdTooLong,
    ZeroPolicyVersion,
    EmptyAuthorityRole { role: &'static str },
    TooManyAuthorities { role: &'static str },
    InvalidAuthorityDid { role: &'static str },
    NonCanonicalAuthorityOrdering { role: &'static str },
    RoleOverlapNotAllowed,
    InvalidAuthorDid,
    UnauthorizedOracleAuthor,
    UnauthorizedGovernanceAuthor,
}

fn role_name(role: AuthorityRole) -> &'static str {
    match role {
        AuthorityRole::Oracle => "oracle",
        AuthorityRole::Governance => "governance",
    }
}

fn validate_root_id(root_id: &str) -> Result<(), TendAuthorityError> {
    if root_id.is_empty() {
        return Err(TendAuthorityError::EmptyRootId);
    }
    if root_id.len() > MAX_TEND_AUTHORITY_ROOT_ID_LEN {
        return Err(TendAuthorityError::RootIdTooLong);
    }
    if root_id.trim() != root_id
        || !root_id
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b':' | b'-'))
    {
        return Err(TendAuthorityError::NonCanonicalRootId);
    }
    Ok(())
}

fn validate_role_dids(role: AuthorityRole, dids: &[String]) -> Result<(), TendAuthorityError> {
    let role = role_name(role);
    if dids.is_empty() {
        return Err(TendAuthorityError::EmptyAuthorityRole { role });
    }
    if dids.len() > MAX_TEND_AUTHORITY_DIDS_PER_ROLE {
        return Err(TendAuthorityError::TooManyAuthorities { role });
    }
    for did in dids {
        if !is_canonical_mycelix_did(did) {
            return Err(TendAuthorityError::InvalidAuthorityDid { role });
        }
    }
    if dids.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(TendAuthorityError::NonCanonicalAuthorityOrdering { role });
    }
    Ok(())
}

fn validate_author_did(author_did: &str) -> Result<(), TendAuthorityError> {
    if is_canonical_mycelix_did(author_did) {
        Ok(())
    } else {
        Err(TendAuthorityError::InvalidAuthorDid)
    }
}

fn is_canonical_mycelix_did(did: &str) -> bool {
    const PREFIX: &str = "did:mycelix:";
    if did.len() > MAX_TEND_AUTHORITY_DID_LEN || !did.starts_with(PREFIX) {
        return false;
    }
    let suffix = &did[PREFIX.len()..];
    !suffix.is_empty()
        && did.trim() == did
        && did.is_ascii()
        && suffix.bytes().all(|byte| !byte.is_ascii_whitespace())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_root() -> TendAuthorityRootV1 {
        TendAuthorityRootV1 {
            schema_version: TEND_AUTHORITY_SCHEMA_VERSION_V1,
            root_id: "tend-authority:local:v1".into(),
            policy_version: 1,
            allow_role_overlap: false,
            oracle_authority_dids: vec!["did:mycelix:oracle-a".into()],
            governance_authority_dids: vec!["did:mycelix:governance-a".into()],
        }
    }

    fn enabled(root: TendAuthorityRootV1) -> TendAuthorityConfig {
        TendAuthorityConfig {
            enabled: true,
            root: Some(root),
        }
    }

    #[test]
    fn disabled_config_is_canonical_and_fail_closed() {
        let config = TendAuthorityConfig::disabled();
        assert_eq!(config.validate(), Ok(()));
        assert_eq!(config.require_enabled(), Err(TendAuthorityError::AuthorityDisabled));
        assert_eq!(
            config.authorize_oracle("did:mycelix:oracle-a"),
            Err(TendAuthorityError::AuthorityDisabled)
        );
    }

    #[test]
    fn disabled_config_cannot_smuggle_a_root() {
        let config = TendAuthorityConfig {
            enabled: false,
            root: Some(valid_root()),
        };
        assert_eq!(
            config.validate(),
            Err(TendAuthorityError::DisabledConfigCarriesRoot)
        );
    }

    #[test]
    fn enabled_config_requires_a_root() {
        let config = TendAuthorityConfig {
            enabled: true,
            root: None,
        };
        assert_eq!(
            config.validate(),
            Err(TendAuthorityError::EnabledConfigMissingRoot)
        );
    }

    #[test]
    fn valid_root_authorizes_exact_roles_only() {
        let config = enabled(valid_root());
        assert_eq!(config.validate(), Ok(()));
        assert_eq!(config.authorize_oracle("did:mycelix:oracle-a"), Ok(()));
        assert_eq!(
            config.authorize_oracle("did:mycelix:governance-a"),
            Err(TendAuthorityError::UnauthorizedOracleAuthor)
        );
        assert_eq!(
            config.authorize_governance("did:mycelix:governance-a"),
            Ok(())
        );
        assert_eq!(
            config.authorize_governance("did:mycelix:oracle-a"),
            Err(TendAuthorityError::UnauthorizedGovernanceAuthor)
        );
    }

    #[test]
    fn malformed_versions_and_root_ids_fail_closed() {
        let mut root = valid_root();
        root.schema_version = 2;
        assert_eq!(
            root.validate(),
            Err(TendAuthorityError::UnsupportedSchemaVersion { actual: 2 })
        );

        let mut root = valid_root();
        root.policy_version = 0;
        assert_eq!(root.validate(), Err(TendAuthorityError::ZeroPolicyVersion));

        for invalid in ["", " leading", "trailing ", "has/slash", "has space"] {
            let mut root = valid_root();
            root.root_id = invalid.into();
            assert!(root.validate().is_err(), "root id {invalid:?} must fail");
        }
    }

    #[test]
    fn roles_must_be_nonempty_bounded_sorted_unique_and_canonical() {
        let mut root = valid_root();
        root.oracle_authority_dids.clear();
        assert_eq!(
            root.validate(),
            Err(TendAuthorityError::EmptyAuthorityRole { role: "oracle" })
        );

        let mut root = valid_root();
        root.oracle_authority_dids = (0..=MAX_TEND_AUTHORITY_DIDS_PER_ROLE)
            .map(|index| format!("did:mycelix:oracle-{index:03}"))
            .collect();
        assert_eq!(
            root.validate(),
            Err(TendAuthorityError::TooManyAuthorities { role: "oracle" })
        );

        for dids in [
            vec!["did:mycelix:b".into(), "did:mycelix:a".into()],
            vec!["did:mycelix:a".into(), "did:mycelix:a".into()],
        ] {
            let mut root = valid_root();
            root.oracle_authority_dids = dids;
            assert_eq!(
                root.validate(),
                Err(TendAuthorityError::NonCanonicalAuthorityOrdering { role: "oracle" })
            );
        }

        for invalid in ["oracle-a", "did:mycelix:", " did:mycelix:a", "did:mycelix:a b"] {
            let mut root = valid_root();
            root.oracle_authority_dids = vec![invalid.into()];
            assert_eq!(
                root.validate(),
                Err(TendAuthorityError::InvalidAuthorityDid { role: "oracle" })
            );
        }
    }

    #[test]
    fn role_overlap_requires_explicit_policy() {
        let mut root = valid_root();
        root.oracle_authority_dids = vec!["did:mycelix:shared".into()];
        root.governance_authority_dids = vec!["did:mycelix:shared".into()];
        assert_eq!(root.validate(), Err(TendAuthorityError::RoleOverlapNotAllowed));

        root.allow_role_overlap = true;
        assert_eq!(root.validate(), Ok(()));
        let config = enabled(root);
        assert_eq!(config.authorize_oracle("did:mycelix:shared"), Ok(()));
        assert_eq!(config.authorize_governance("did:mycelix:shared"), Ok(()));
    }

    #[test]
    fn malformed_and_unknown_action_authors_fail_closed() {
        let config = enabled(valid_root());
        assert_eq!(
            config.authorize_oracle("not-a-did"),
            Err(TendAuthorityError::InvalidAuthorDid)
        );
        assert_eq!(
            config.authorize_oracle("did:mycelix:unknown"),
            Err(TendAuthorityError::UnauthorizedOracleAuthor)
        );
    }

    #[test]
    fn serde_round_trip_preserves_authority_exactly() {
        let config = enabled(valid_root());
        let encoded = serde_json::to_vec(&config).expect("serialize authority config");
        let decoded: TendAuthorityConfig =
            serde_json::from_slice(&encoded).expect("deserialize authority config");
        assert_eq!(decoded, config);
        assert_eq!(decoded.validate(), Ok(()));
    }
}
