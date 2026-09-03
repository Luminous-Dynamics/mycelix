// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! DNA-bound constitutional genesis contracts.
//!
//! This crate is intentionally transport-neutral. Holochain adapters obtain the
//! manifest from DNA properties; other runtimes may bind the same manifest to an
//! equivalent immutable network/genesis identifier.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-constitution-genesis-v0.1";
const MAX_ID_BYTES: usize = 512;
const MAX_PROFILE_BYTES: usize = 128;
const MAX_BOOTSTRAP_AUTHORITIES: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Digest32(pub [u8; 32]);

impl Digest32 {
    pub fn is_zero(&self) -> bool {
        self.0.iter().all(|byte| *byte == 0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionalGenesisManifest {
    pub protocol_version: String,
    pub institution_id: String,
    pub constitutional_rulebook_id: String,
    pub constitutional_rulebook_version: String,
    pub constitutional_rulebook_digest: Digest32,
    pub constitutional_rulebook_digest_profile: String,

    /// Digest of the exact genesis charter bytes/profile. The charter entry may
    /// be materialized later, but it cannot claim constitutional authority unless
    /// it matches this DNA-bound commitment.
    pub genesis_charter_digest: Digest32,
    pub genesis_charter_digest_profile: String,

    /// Digest of the complete bootstrap parameter set. Individual parameters are
    /// not authoritative merely because they were written first.
    pub genesis_parameter_manifest_digest: Digest32,
    pub genesis_parameter_manifest_profile: String,

    /// Capability/policy identifier for the mechanism authorized to ratify later
    /// constitutional versions. This points to a governance process, not a person.
    pub amendment_authority_policy_id: String,
    pub amendment_authority_policy_digest: Digest32,
    pub amendment_authority_policy_profile: String,

    /// Optional, tightly bounded authorities allowed to materialize the committed
    /// genesis charter/parameter set. They receive no post-genesis amendment power.
    pub bootstrap_authorities: Vec<String>,

    /// Human/network-readable epoch. A new incompatible constitutional root should
    /// form a distinct DNA/network rather than mutate this value at runtime.
    pub genesis_epoch: u64,
}

impl ConstitutionalGenesisManifest {
    pub fn validate(&self) -> Result<(), GenesisError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(GenesisError::WrongProtocolVersion);
        }
        validate_text(&self.institution_id, "institution_id", MAX_ID_BYTES)?;
        validate_text(
            &self.constitutional_rulebook_id,
            "constitutional_rulebook_id",
            MAX_ID_BYTES,
        )?;
        validate_text(
            &self.constitutional_rulebook_version,
            "constitutional_rulebook_version",
            MAX_PROFILE_BYTES,
        )?;
        validate_profile(
            &self.constitutional_rulebook_digest_profile,
            "constitutional_rulebook_digest_profile",
        )?;
        validate_profile(
            &self.genesis_charter_digest_profile,
            "genesis_charter_digest_profile",
        )?;
        validate_profile(
            &self.genesis_parameter_manifest_profile,
            "genesis_parameter_manifest_profile",
        )?;
        validate_text(
            &self.amendment_authority_policy_id,
            "amendment_authority_policy_id",
            MAX_ID_BYTES,
        )?;
        validate_profile(
            &self.amendment_authority_policy_profile,
            "amendment_authority_policy_profile",
        )?;
        require_digest(
            self.constitutional_rulebook_digest,
            "constitutional_rulebook_digest",
        )?;
        require_digest(self.genesis_charter_digest, "genesis_charter_digest")?;
        require_digest(
            self.genesis_parameter_manifest_digest,
            "genesis_parameter_manifest_digest",
        )?;
        require_digest(
            self.amendment_authority_policy_digest,
            "amendment_authority_policy_digest",
        )?;
        if self.genesis_epoch == 0 {
            return Err(GenesisError::ZeroGenesisEpoch);
        }
        if self.bootstrap_authorities.len() > MAX_BOOTSTRAP_AUTHORITIES {
            return Err(GenesisError::TooManyBootstrapAuthorities);
        }
        let mut unique = BTreeSet::new();
        for authority in &self.bootstrap_authorities {
            validate_text(authority, "bootstrap_authority", MAX_ID_BYTES)?;
            if !authority.starts_with("did:") {
                return Err(GenesisError::InvalidBootstrapAuthority);
            }
            if !unique.insert(authority) {
                return Err(GenesisError::DuplicateBootstrapAuthority);
            }
        }
        Ok(())
    }

    pub fn bootstrap_can_materialize(&self, author_did: &str) -> bool {
        self.bootstrap_authorities
            .iter()
            .any(|authority| authority == author_did)
    }
}

/// A materialized charter is constitutionally authoritative only if it matches
/// the DNA-bound genesis commitment or is a verified descendant of it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConstitutionalVersionRef {
    pub institution_id: String,
    pub version: u64,
    pub charter_digest: Digest32,
    pub charter_digest_profile: String,
    pub rulebook_digest: Digest32,
    pub parent_charter_digest: Option<Digest32>,
    pub ratification_receipt_ref: Option<String>,
}

impl ConstitutionalVersionRef {
    pub fn validate_genesis_against(
        &self,
        manifest: &ConstitutionalGenesisManifest,
    ) -> Result<(), GenesisError> {
        manifest.validate()?;
        if self.version != 1 {
            return Err(GenesisError::NotGenesisVersion);
        }
        if self.institution_id != manifest.institution_id {
            return Err(GenesisError::InstitutionMismatch);
        }
        if self.charter_digest != manifest.genesis_charter_digest
            || self.charter_digest_profile != manifest.genesis_charter_digest_profile
        {
            return Err(GenesisError::GenesisCharterMismatch);
        }
        if self.rulebook_digest != manifest.constitutional_rulebook_digest {
            return Err(GenesisError::RulebookMismatch);
        }
        if self.parent_charter_digest.is_some() || self.ratification_receipt_ref.is_some() {
            return Err(GenesisError::GenesisCannotHaveParentOrRatification);
        }
        Ok(())
    }

    pub fn validate_successor_of(
        &self,
        previous: &ConstitutionalVersionRef,
    ) -> Result<(), GenesisError> {
        if self.institution_id != previous.institution_id {
            return Err(GenesisError::InstitutionMismatch);
        }
        if self.version != previous.version.saturating_add(1) {
            return Err(GenesisError::NonSequentialVersion);
        }
        if self.parent_charter_digest != Some(previous.charter_digest) {
            return Err(GenesisError::ParentDigestMismatch);
        }
        require_digest(self.charter_digest, "successor.charter_digest")?;
        require_digest(self.rulebook_digest, "successor.rulebook_digest")?;
        validate_profile(
            &self.charter_digest_profile,
            "successor.charter_digest_profile",
        )?;
        let receipt = self
            .ratification_receipt_ref
            .as_deref()
            .ok_or(GenesisError::MissingRatificationReceipt)?;
        validate_text(receipt, "ratification_receipt_ref", MAX_ID_BYTES)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GenesisError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    ZeroDigest(&'static str),
    ZeroGenesisEpoch,
    TooManyBootstrapAuthorities,
    InvalidBootstrapAuthority,
    DuplicateBootstrapAuthority,
    NotGenesisVersion,
    InstitutionMismatch,
    GenesisCharterMismatch,
    RulebookMismatch,
    GenesisCannotHaveParentOrRatification,
    NonSequentialVersion,
    ParentDigestMismatch,
    MissingRatificationReceipt,
}

impl fmt::Display for GenesisError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong constitutional genesis protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a canonical profile token"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::ZeroGenesisEpoch => write!(f, "genesis epoch must be non-zero"),
            Self::TooManyBootstrapAuthorities => write!(f, "too many bootstrap authorities"),
            Self::InvalidBootstrapAuthority => write!(f, "bootstrap authority must be a DID"),
            Self::DuplicateBootstrapAuthority => write!(f, "duplicate bootstrap authority"),
            Self::NotGenesisVersion => write!(f, "constitutional genesis version must be 1"),
            Self::InstitutionMismatch => write!(f, "constitutional institution mismatch"),
            Self::GenesisCharterMismatch => write!(f, "materialized charter does not match DNA-bound genesis charter"),
            Self::RulebookMismatch => write!(f, "constitutional rulebook mismatch"),
            Self::GenesisCannotHaveParentOrRatification => write!(f, "genesis version cannot have a parent or ratification receipt"),
            Self::NonSequentialVersion => write!(f, "constitutional versions must increment by exactly one"),
            Self::ParentDigestMismatch => write!(f, "constitutional successor parent digest mismatch"),
            Self::MissingRatificationReceipt => write!(f, "constitutional successor requires a ratification receipt"),
        }
    }
}

impl std::error::Error for GenesisError {}

fn validate_text(value: &str, field: &'static str, max: usize) -> Result<(), GenesisError> {
    if value.trim().is_empty() {
        return Err(GenesisError::Empty(field));
    }
    if value.len() > max {
        return Err(GenesisError::TooLong(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), GenesisError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(GenesisError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(digest: Digest32, field: &'static str) -> Result<(), GenesisError> {
    if digest.is_zero() {
        Err(GenesisError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn manifest() -> ConstitutionalGenesisManifest {
        ConstitutionalGenesisManifest {
            protocol_version: PROTOCOL_VERSION.into(),
            institution_id: "institution:mycelix:test".into(),
            constitutional_rulebook_id: "rulebook:constitution".into(),
            constitutional_rulebook_version: "1".into(),
            constitutional_rulebook_digest: digest(1),
            constitutional_rulebook_digest_profile: "mycelix-rulebook-v1".into(),
            genesis_charter_digest: digest(2),
            genesis_charter_digest_profile: "mycelix-charter-v1".into(),
            genesis_parameter_manifest_digest: digest(3),
            genesis_parameter_manifest_profile: "mycelix-constitution-params-v1".into(),
            amendment_authority_policy_id: "policy:constitutional-amendment:v1".into(),
            amendment_authority_policy_digest: digest(4),
            amendment_authority_policy_profile: "mycelix-authority-policy-v1".into(),
            bootstrap_authorities: vec!["did:mycelix:bootstrap".into()],
            genesis_epoch: 1,
        }
    }

    #[test]
    fn valid_manifest_passes() {
        assert!(manifest().validate().is_ok());
    }

    #[test]
    fn zero_digest_is_rejected() {
        let mut manifest = manifest();
        manifest.genesis_charter_digest = Digest32([0; 32]);
        assert!(matches!(
            manifest.validate(),
            Err(GenesisError::ZeroDigest("genesis_charter_digest"))
        ));
    }

    #[test]
    fn bootstrap_authority_has_no_implicit_successor_power() {
        let manifest = manifest();
        assert!(manifest.bootstrap_can_materialize("did:mycelix:bootstrap"));
        let genesis = ConstitutionalVersionRef {
            institution_id: manifest.institution_id.clone(),
            version: 1,
            charter_digest: manifest.genesis_charter_digest,
            charter_digest_profile: manifest.genesis_charter_digest_profile.clone(),
            rulebook_digest: manifest.constitutional_rulebook_digest,
            parent_charter_digest: None,
            ratification_receipt_ref: None,
        };
        let successor = ConstitutionalVersionRef {
            institution_id: manifest.institution_id.clone(),
            version: 2,
            charter_digest: digest(9),
            charter_digest_profile: "mycelix-charter-v1".into(),
            rulebook_digest: digest(8),
            parent_charter_digest: Some(genesis.charter_digest),
            ratification_receipt_ref: None,
        };
        assert_eq!(
            successor.validate_successor_of(&genesis),
            Err(GenesisError::MissingRatificationReceipt)
        );
    }

    #[test]
    fn genesis_must_match_exact_dna_commitment() {
        let manifest = manifest();
        let mut genesis = ConstitutionalVersionRef {
            institution_id: manifest.institution_id.clone(),
            version: 1,
            charter_digest: manifest.genesis_charter_digest,
            charter_digest_profile: manifest.genesis_charter_digest_profile.clone(),
            rulebook_digest: manifest.constitutional_rulebook_digest,
            parent_charter_digest: None,
            ratification_receipt_ref: None,
        };
        assert!(genesis.validate_genesis_against(&manifest).is_ok());
        genesis.charter_digest = digest(99);
        assert_eq!(
            genesis.validate_genesis_against(&manifest),
            Err(GenesisError::GenesisCharterMismatch)
        );
    }
}
