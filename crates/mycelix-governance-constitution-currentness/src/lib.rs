// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure transport and validation contract for leased constitutional currentness.
//!
//! This crate intentionally contains no Holochain calls and grants no authority by
//! deserialization. It defines the canonical evidence-shaped transport used between
//! the designated currentness verifier and direct local consumers, plus the exact
//! digest/reference/lease rules consumers must independently recompute.

use mycelix_governance_constitution::{ConstitutionStatement, Digest32, STATEMENT_PROFILE};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-current-constitution-leased-v0.1";
pub const CURRENTNESS_EVIDENCE_PROFILE: &str =
    "mycelix-governance-current-constitution-evidence-v1-blake3-framed";
pub const GENESIS_CURRENTNESS_LEASE_BASIS: &str =
    "dna-immutable-genesis-amendments-disabled-local-reuse-v1";
pub const GENESIS_CURRENTNESS_REUSE_MS: u64 = 30_000;

const DOMAIN_CURRENTNESS_EVIDENCE: &[u8] =
    b"mycelix/governance/current-constitution-evidence/v1";
const MAX_DNA_HASH_BYTES: usize = 1024;

/// Transport projection of one bounded current-constitution verification.
///
/// This type is deliberately deserializable because it crosses a zome/API boundary.
/// Deserialization does not make it positive authority. A direct consumer must obtain
/// it from the designated verifier and call [`LeasedVerifiedCurrentConstitution::validate_at`]
/// before use.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct LeasedVerifiedCurrentConstitution {
    pub protocol: String,
    pub dna_hash: String,
    pub statement: ConstitutionStatement,
    pub statement_digest: Digest32,
    pub currentness_evidence_digest: Digest32,
    pub currentness_evidence_profile: String,
    pub verified_transition_count: u64,
    pub legacy_constitution_authoritative: bool,
    pub lease_basis: String,
    pub verification_ref: String,
    pub verified_at_ms: u64,
    pub valid_until_ms: u64,
    pub genesis_currentness_by_amendments_disabled: bool,
    pub transition_currentness_supported: bool,
    pub candidate_discovery_used_for_positive_currentness: bool,
}

impl LeasedVerifiedCurrentConstitution {
    /// Construct the canonical genesis-only transport after the host has independently
    /// established that amendments are disabled for the binding DNA authority.
    pub fn new_genesis(
        dna_hash: impl Into<String>,
        statement: ConstitutionStatement,
        verified_at_ms: u64,
        valid_until_ms: u64,
    ) -> Result<Self, CurrentnessContractError> {
        let dna_hash = dna_hash.into();
        validate_dna_hash(&dna_hash)?;
        validate_genesis_statement(&statement)?;
        validate_lease_window(verified_at_ms, valid_until_ms)?;

        let statement_digest = statement
            .digest()
            .map_err(|error| CurrentnessContractError::InvalidStatement(error.to_string()))?;
        let currentness_evidence_digest = currentness_evidence_digest(
            &dna_hash,
            statement_digest,
            GENESIS_CURRENTNESS_LEASE_BASIS,
            verified_at_ms,
            valid_until_ms,
        );
        let verification_ref = currentness_verification_ref(currentness_evidence_digest);

        let value = Self {
            protocol: PROTOCOL_VERSION.into(),
            dna_hash,
            statement,
            statement_digest,
            currentness_evidence_digest,
            currentness_evidence_profile: CURRENTNESS_EVIDENCE_PROFILE.into(),
            verified_transition_count: 0,
            legacy_constitution_authoritative: false,
            lease_basis: GENESIS_CURRENTNESS_LEASE_BASIS.into(),
            verification_ref,
            verified_at_ms,
            valid_until_ms,
            genesis_currentness_by_amendments_disabled: true,
            transition_currentness_supported: false,
            candidate_discovery_used_for_positive_currentness: false,
        };
        value.validate_at(verified_at_ms)?;
        Ok(value)
    }

    /// Recompute every derived identity/mode/lease property at the consumer's
    /// observation time. This validates evidence shape only; it does not prove the
    /// caller obtained the value from the designated verifier.
    pub fn validate_at(&self, now_ms: u64) -> Result<(), CurrentnessContractError> {
        if self.protocol != PROTOCOL_VERSION {
            return Err(CurrentnessContractError::WrongProtocol);
        }
        validate_dna_hash(&self.dna_hash)?;
        validate_genesis_statement(&self.statement)?;

        let statement_digest = self
            .statement
            .digest()
            .map_err(|error| CurrentnessContractError::InvalidStatement(error.to_string()))?;
        if statement_digest != self.statement_digest {
            return Err(CurrentnessContractError::StatementDigestMismatch);
        }

        if self.currentness_evidence_profile != CURRENTNESS_EVIDENCE_PROFILE {
            return Err(CurrentnessContractError::WrongEvidenceProfile);
        }
        if self.verified_transition_count != 0
            || self.legacy_constitution_authoritative
            || !self.genesis_currentness_by_amendments_disabled
            || self.transition_currentness_supported
            || self.candidate_discovery_used_for_positive_currentness
        {
            return Err(CurrentnessContractError::UnsupportedCurrentnessMode);
        }
        if self.lease_basis != GENESIS_CURRENTNESS_LEASE_BASIS {
            return Err(CurrentnessContractError::WrongLeaseBasis);
        }

        validate_lease_window(self.verified_at_ms, self.valid_until_ms)?;
        if now_ms == 0 || self.verified_at_ms > now_ms {
            return Err(CurrentnessContractError::FutureDatedEvidence);
        }
        if self.valid_until_ms <= now_ms {
            return Err(CurrentnessContractError::ExpiredEvidence);
        }

        let expected_digest = currentness_evidence_digest(
            &self.dna_hash,
            self.statement_digest,
            &self.lease_basis,
            self.verified_at_ms,
            self.valid_until_ms,
        );
        if expected_digest != self.currentness_evidence_digest
            || self.currentness_evidence_digest.is_zero()
        {
            return Err(CurrentnessContractError::EvidenceDigestMismatch);
        }
        if self.verification_ref != currentness_verification_ref(expected_digest) {
            return Err(CurrentnessContractError::VerificationRefMismatch);
        }
        Ok(())
    }
}

pub fn currentness_evidence_digest(
    dna_hash: &str,
    statement_digest: Digest32,
    lease_basis: &str,
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_CURRENTNESS_EVIDENCE);
    frame(&mut hasher, CURRENTNESS_EVIDENCE_PROFILE.as_bytes());
    frame(&mut hasher, dna_hash.as_bytes());
    frame(&mut hasher, STATEMENT_PROFILE.as_bytes());
    frame(&mut hasher, &statement_digest.0);
    frame(&mut hasher, lease_basis.as_bytes());
    frame(&mut hasher, &verified_at_ms.to_le_bytes());
    frame(&mut hasher, &valid_until_ms.to_le_bytes());
    Digest32(*hasher.finalize().as_bytes())
}

pub fn currentness_verification_ref(digest: Digest32) -> String {
    format!(
        "constitution-currentness-evidence:{CURRENTNESS_EVIDENCE_PROFILE}:{}",
        digest.to_hex()
    )
}

fn validate_genesis_statement(
    statement: &ConstitutionStatement,
) -> Result<(), CurrentnessContractError> {
    statement
        .validate()
        .map_err(|error| CurrentnessContractError::InvalidStatement(error.to_string()))?;
    if statement.version != 1 || statement.parent_statement_digest.is_some() {
        return Err(CurrentnessContractError::NotGenesisStatement);
    }
    Ok(())
}

fn validate_dna_hash(value: &str) -> Result<(), CurrentnessContractError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_DNA_HASH_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        return Err(CurrentnessContractError::InvalidDnaHash);
    }
    Ok(())
}

fn validate_lease_window(
    verified_at_ms: u64,
    valid_until_ms: u64,
) -> Result<(), CurrentnessContractError> {
    if verified_at_ms == 0 || valid_until_ms <= verified_at_ms {
        return Err(CurrentnessContractError::InvalidLeaseWindow);
    }
    let width = valid_until_ms
        .checked_sub(verified_at_ms)
        .ok_or(CurrentnessContractError::InvalidLeaseWindow)?;
    if width > GENESIS_CURRENTNESS_REUSE_MS {
        return Err(CurrentnessContractError::LeaseExceedsGenesisReuseCap);
    }
    Ok(())
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CurrentnessContractError {
    WrongProtocol,
    InvalidDnaHash,
    InvalidStatement(String),
    NotGenesisStatement,
    StatementDigestMismatch,
    WrongEvidenceProfile,
    UnsupportedCurrentnessMode,
    WrongLeaseBasis,
    InvalidLeaseWindow,
    LeaseExceedsGenesisReuseCap,
    FutureDatedEvidence,
    ExpiredEvidence,
    EvidenceDigestMismatch,
    VerificationRefMismatch,
}

impl fmt::Display for CurrentnessContractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong leased constitutional-currentness protocol"),
            Self::InvalidDnaHash => write!(f, "invalid constitutional DNA identity"),
            Self::InvalidStatement(error) => {
                write!(f, "invalid constitutional statement: {error}")
            }
            Self::NotGenesisStatement => write!(
                f,
                "genesis-only currentness transport contains a non-genesis statement"
            ),
            Self::StatementDigestMismatch => write!(f, "constitutional statement digest mismatch"),
            Self::WrongEvidenceProfile => {
                write!(f, "wrong constitutional-currentness evidence profile")
            }
            Self::UnsupportedCurrentnessMode => write!(
                f,
                "unsupported amendment/candidate constitutional-currentness mode"
            ),
            Self::WrongLeaseBasis => {
                write!(f, "wrong genesis constitutional-currentness lease basis")
            }
            Self::InvalidLeaseWindow => {
                write!(f, "invalid constitutional-currentness lease window")
            }
            Self::LeaseExceedsGenesisReuseCap => write!(
                f,
                "constitutional-currentness lease exceeds the genesis reuse cap"
            ),
            Self::FutureDatedEvidence => {
                write!(f, "constitutional-currentness evidence is future-dated")
            }
            Self::ExpiredEvidence => write!(f, "constitutional-currentness evidence is expired"),
            Self::EvidenceDigestMismatch => {
                write!(f, "constitutional-currentness evidence digest mismatch")
            }
            Self::VerificationRefMismatch => write!(
                f,
                "constitutional-currentness verification reference mismatch"
            ),
        }
    }
}

impl std::error::Error for CurrentnessContractError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_constitution::{
        ConstitutionGenesisManifest, ConstitutionId, InstitutionId, NetworkId, ProfiledDigest,
        RulebookId, PROTOCOL_VERSION as CONSTITUTION_PROTOCOL_VERSION,
    };

    fn digest(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn pd(byte: u8, profile: &str) -> ProfiledDigest {
        ProfiledDigest {
            digest: digest(byte),
            profile: profile.into(),
        }
    }

    fn statement() -> ConstitutionStatement {
        ConstitutionGenesisManifest {
            protocol_version: CONSTITUTION_PROTOCOL_VERSION.into(),
            network_id: NetworkId::new("network:mycelix:test").unwrap(),
            institution_id: InstitutionId::new("institution:mycelix:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:mycelix:test").unwrap(),
            rulebook_id: RulebookId::new("rulebook:constitution:v1").unwrap(),
            rulebook_version: "1".into(),
            rulebook: pd(1, "mycelix-rulebook-v1"),
            charter: pd(2, "mycelix-charter-v1"),
            parameters: pd(3, "mycelix-parameters-v1"),
            amendment_policy: pd(4, "mycelix-amendment-policy-v1"),
            binding_vote_profile: "mycelix-binding-vote-v2".into(),
            threshold_authority_profile: "mycelix-threshold-authority-v1".into(),
            effective_from_ms: 1_000,
        }
        .genesis_statement()
        .unwrap()
    }

    fn valid() -> LeasedVerifiedCurrentConstitution {
        LeasedVerifiedCurrentConstitution::new_genesis(
            "uhC0kCurrentnessTest",
            statement(),
            10_000,
            40_000,
        )
        .unwrap()
    }

    #[test]
    fn canonical_genesis_transport_validates() {
        valid().validate_at(20_000).unwrap();
    }

    #[test]
    fn refreshed_observation_changes_evidence_not_statement_identity() {
        let first = valid();
        let second = LeasedVerifiedCurrentConstitution::new_genesis(
            first.dna_hash.clone(),
            first.statement.clone(),
            10_001,
            40_001,
        )
        .unwrap();
        assert_eq!(first.statement_digest, second.statement_digest);
        assert_ne!(first.currentness_evidence_digest, second.currentness_evidence_digest);
        assert_ne!(first.verification_ref, second.verification_ref);
    }

    #[test]
    fn tampered_evidence_digest_is_rejected() {
        let mut value = valid();
        value.currentness_evidence_digest = digest(99);
        assert_eq!(
            value.validate_at(20_000).unwrap_err(),
            CurrentnessContractError::EvidenceDigestMismatch
        );
    }

    #[test]
    fn tampered_reference_is_rejected() {
        let mut value = valid();
        value.verification_ref.push_str(":forged");
        assert_eq!(
            value.validate_at(20_000).unwrap_err(),
            CurrentnessContractError::VerificationRefMismatch
        );
    }

    #[test]
    fn widened_lease_is_rejected_even_with_recomputed_identity() {
        assert_eq!(
            LeasedVerifiedCurrentConstitution::new_genesis(
                "uhC0kCurrentnessTest",
                statement(),
                10_000,
                40_001,
            )
            .unwrap_err(),
            CurrentnessContractError::LeaseExceedsGenesisReuseCap
        );
    }

    #[test]
    fn transition_mode_is_rejected() {
        let mut value = valid();
        value.verified_transition_count = 1;
        assert_eq!(
            value.validate_at(20_000).unwrap_err(),
            CurrentnessContractError::UnsupportedCurrentnessMode
        );
    }

    #[test]
    fn non_genesis_statement_is_rejected() {
        let mut later = statement();
        later.version = 2;
        later.parent_statement_digest = Some(digest(9));
        assert_eq!(
            LeasedVerifiedCurrentConstitution::new_genesis(
                "uhC0kCurrentnessTest",
                later,
                10_000,
                40_000,
            )
            .unwrap_err(),
            CurrentnessContractError::NotGenesisStatement
        );
    }

    #[test]
    fn stale_and_future_dated_evidence_are_rejected() {
        let value = valid();
        assert_eq!(
            value.validate_at(40_000).unwrap_err(),
            CurrentnessContractError::ExpiredEvidence
        );
        assert_eq!(
            value.validate_at(9_999).unwrap_err(),
            CurrentnessContractError::FutureDatedEvidence
        );
    }

    #[test]
    fn serde_round_trip_does_not_change_validation_identity() {
        let value = valid();
        let json = serde_json::to_string(&value).unwrap();
        let decoded: LeasedVerifiedCurrentConstitution = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, value);
        decoded.validate_at(20_000).unwrap();
    }
}
