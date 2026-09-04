// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure runtime-provider projection for current governance executor authority.
//!
//! The lifecycle v0.1 wire format already carries an opaque
//! `executor_authority_ref`. This crate gives that field a strict meaning without
//! changing the lifecycle protocol: it is a deterministic reference to the exact
//! generation-bound current executor authority qualified by
//! `mycelix-governance-current-executor-authority`.
//!
//! A runtime provider must construct this receipt from exact semantic inputs and
//! authoritative freshness evidence. Callers do not get to choose the authority
//! ref, current digest, or validity horizon.

use mycelix_authority_freshness::{VerifiedAuthorityFreshness, BUNDLE_IDENTITY_PROFILE};
use mycelix_authority_identity::AUTHORITY_GRANT_IDENTITY_PROFILE;
use mycelix_governance_authority::ProposalId;
use mycelix_governance_current_executor_authority::{
    qualify_current_executor_authority, CurrentExecutorAuthorityError,
    THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE, CURRENT_EXECUTOR_AUTHORITY_PROFILE,
};
use mycelix_governance_executor_designation::{
    VerifiedAuthorityGrant, VerifiedExecutorDesignation, VerifiedThresholdAuthorization,
};
use mycelix_governance_executor_lineage::{
    DelegationLineageEvidence, EXECUTOR_LINEAGE_AUTHORITY_PROFILE,
};
use mycelix_institutional_core::{
    AuthorityGrantId, CapabilityId, Digest32, InstitutionId, JurisdictionId, PrincipalId,
    RulebookRef,
};
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-executor-authority-provider-v0.1";
pub const EXECUTOR_AUTHORITY_REF_SCHEME: &str = "current-executor";

const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Exact wire projection consumed by the lifecycle composition verifier.
///
/// This type is deserializable because it is a provider ABI, not authority by
/// itself. A consumer must obtain it by directly calling the authoritative
/// provider and must validate it before use.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentExecutorAuthorityProjection {
    pub proposal_id: ProposalId,
    pub actions_digest: Digest32,
    pub actions_digest_profile: String,
    pub threshold_authorization_ref: String,
    pub threshold_authorization_identity_digest: Digest32,
    pub threshold_authorization_identity_profile: String,
    pub executor_principal: PrincipalId,
    /// Deterministic alias of `current_executor_authority_digest/profile`.
    pub executor_authority_ref: String,
    pub current_executor_authority_digest: Digest32,
    pub current_executor_authority_profile: String,
    pub semantic_executor_authority_digest: Digest32,
    pub semantic_executor_authority_profile: String,
    pub authority_grant_id: AuthorityGrantId,
    pub authority_grant_identity_digest: Digest32,
    pub authority_grant_identity_profile: String,
    pub granting_institution: InstitutionId,
    pub jurisdiction: Option<JurisdictionId>,
    pub rulebook: RulebookRef,
    pub capability_scope: CapabilityId,
    pub freshness_digest: Digest32,
    pub freshness_profile: String,
    /// Semantic validity ceiling before current-state leases are applied.
    pub semantic_valid_until_ms: u64,
}

impl CurrentExecutorAuthorityProjection {
    pub fn validate(&self) -> Result<(), ExecutorProviderContractError> {
        if self.actions_digest.is_zero()
            || self.threshold_authorization_identity_digest.is_zero()
            || self.current_executor_authority_digest.is_zero()
            || self.semantic_executor_authority_digest.is_zero()
            || self.authority_grant_identity_digest.is_zero()
            || self.freshness_digest.is_zero()
        {
            return Err(ExecutorProviderContractError::ZeroDigest);
        }
        require_profile(&self.actions_digest_profile)?;
        require_ref(&self.threshold_authorization_ref)?;
        require_ref(&self.executor_authority_ref)?;
        require_id(self.proposal_id.as_str())?;
        require_id(self.executor_principal.as_str())?;
        require_id(self.authority_grant_id.as_str())?;
        require_id(self.granting_institution.as_str())?;
        require_id(self.capability_scope.as_str())?;
        if let Some(jurisdiction) = &self.jurisdiction {
            require_id(jurisdiction.as_str())?;
        }
        self.rulebook
            .validate()
            .map_err(|_| ExecutorProviderContractError::InvalidRulebook)?;

        if self.threshold_authorization_identity_profile
            != THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE
        {
            return Err(ExecutorProviderContractError::WrongThresholdIdentityProfile);
        }
        if self.current_executor_authority_profile != CURRENT_EXECUTOR_AUTHORITY_PROFILE {
            return Err(ExecutorProviderContractError::WrongCurrentAuthorityProfile);
        }
        if self.semantic_executor_authority_profile != EXECUTOR_LINEAGE_AUTHORITY_PROFILE {
            return Err(ExecutorProviderContractError::WrongSemanticAuthorityProfile);
        }
        if self.authority_grant_identity_profile != AUTHORITY_GRANT_IDENTITY_PROFILE {
            return Err(ExecutorProviderContractError::WrongGrantIdentityProfile);
        }
        if self.freshness_profile != BUNDLE_IDENTITY_PROFILE {
            return Err(ExecutorProviderContractError::WrongFreshnessProfile);
        }
        if self.semantic_valid_until_ms == 0 {
            return Err(ExecutorProviderContractError::InvalidValidityHorizon);
        }

        let expected = executor_authority_ref(
            self.current_executor_authority_digest,
            &self.current_executor_authority_profile,
        )?;
        if self.executor_authority_ref != expected {
            return Err(ExecutorProviderContractError::ExecutorAuthorityRefMismatch);
        }
        Ok(())
    }
}

/// Runtime-provider receipt. Dynamic verification metadata is intentionally
/// outside the stable executor authority reference.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedCurrentExecutorAuthorityReceipt {
    pub protocol: String,
    pub projection: CurrentExecutorAuthorityProjection,
    /// Provider-owned trace/content reference for this verification invocation.
    pub verification_ref: String,
    pub verified_at_ms: u64,
    /// Reuse ceiling. At/after this instant the provider must be called again.
    pub valid_until_ms: u64,
}

impl VerifiedCurrentExecutorAuthorityReceipt {
    pub fn validate_at(&self, now_ms: u64) -> Result<(), ExecutorProviderContractError> {
        if self.protocol != PROTOCOL_VERSION {
            return Err(ExecutorProviderContractError::WrongProtocolVersion);
        }
        self.projection.validate()?;
        require_ref(&self.verification_ref)?;
        if now_ms == 0
            || self.verified_at_ms == 0
            || self.verified_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms > self.projection.semantic_valid_until_ms
        {
            return Err(ExecutorProviderContractError::InvalidVerificationWindow);
        }
        Ok(())
    }
}

/// Re-run exact current executor qualification and project it into the runtime
/// provider ABI expected by the lifecycle verifier.
#[allow(clippy::too_many_arguments)]
pub fn qualify_executor_provider_receipt(
    threshold: &VerifiedThresholdAuthorization,
    grant_receipt: &VerifiedAuthorityGrant,
    designation_receipt: &VerifiedExecutorDesignation,
    lineage_evidence: DelegationLineageEvidence<'_>,
    grant_freshness: &VerifiedAuthorityFreshness,
    threshold_freshness: &VerifiedAuthorityFreshness,
    executor_freshness: &VerifiedAuthorityFreshness,
    provider_verification_ref: impl Into<String>,
    now_ms: u64,
) -> Result<VerifiedCurrentExecutorAuthorityReceipt, ExecutorProviderContractError> {
    let current = qualify_current_executor_authority(
        threshold,
        grant_receipt,
        designation_receipt,
        lineage_evidence,
        grant_freshness,
        threshold_freshness,
        executor_freshness,
        now_ms,
    )
    .map_err(ExecutorProviderContractError::CurrentAuthority)?;

    let provider_verification_ref = provider_verification_ref.into();
    require_ref(&provider_verification_ref)?;

    let authority_ref = executor_authority_ref(
        current.current_authority_digest(),
        current.current_authority_profile(),
    )?;

    let semantic_valid_until_ms = designation_receipt
        .designation
        .expires_at_ms
        .min(grant_receipt.grant.expires_at_ms)
        .min(threshold.authorization.valid_until_ms);
    if semantic_valid_until_ms == 0 || current.lease_until_ms() > semantic_valid_until_ms {
        return Err(ExecutorProviderContractError::InvalidValidityHorizon);
    }

    let projection = CurrentExecutorAuthorityProjection {
        proposal_id: threshold.authorization.proposal_id.clone(),
        actions_digest: threshold.authorization.actions_digest,
        actions_digest_profile: threshold.actions_digest_profile.clone(),
        threshold_authorization_ref: current.threshold_authorization_ref().to_string(),
        threshold_authorization_identity_digest: current
            .threshold_authorization_identity_digest(),
        threshold_authorization_identity_profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.into(),
        executor_principal: current.executor_principal().clone(),
        executor_authority_ref: authority_ref,
        current_executor_authority_digest: current.current_authority_digest(),
        current_executor_authority_profile: current.current_authority_profile().to_string(),
        semantic_executor_authority_digest: current.executor_semantic_authority_digest(),
        semantic_executor_authority_profile: current
            .executor_semantic_authority_profile()
            .to_string(),
        authority_grant_id: current.authority_grant_id().clone(),
        authority_grant_identity_digest: current.authority_grant_identity_digest(),
        authority_grant_identity_profile: AUTHORITY_GRANT_IDENTITY_PROFILE.into(),
        granting_institution: current.institution().clone(),
        jurisdiction: threshold.authorization.jurisdiction.clone(),
        rulebook: current.rulebook().clone(),
        capability_scope: designation_receipt.designation.required_capability.clone(),
        freshness_digest: current.freshness_digest(),
        freshness_profile: current.freshness_profile().to_string(),
        semantic_valid_until_ms,
    };
    projection.validate()?;

    let receipt = VerifiedCurrentExecutorAuthorityReceipt {
        protocol: PROTOCOL_VERSION.into(),
        projection,
        verification_ref: provider_verification_ref,
        // The provider has just performed qualification for `now_ms`. Internal
        // evidence may have earlier verification timestamps, but this receipt is
        // the provider invocation result at this exact time.
        verified_at_ms: now_ms,
        valid_until_ms: current.lease_until_ms(),
    };
    receipt.validate_at(now_ms)?;
    Ok(receipt)
}

/// Stable lifecycle-facing alias for one exact current executor authority.
///
/// This deliberately excludes provider verification timestamps and lease
/// horizons. Same-generation re-verification therefore preserves the execution
/// domain; semantic/current-generation changes do not.
pub fn executor_authority_ref(
    digest: Digest32,
    profile: &str,
) -> Result<String, ExecutorProviderContractError> {
    if digest.is_zero() {
        return Err(ExecutorProviderContractError::ZeroDigest);
    }
    if profile != CURRENT_EXECUTOR_AUTHORITY_PROFILE {
        return Err(ExecutorProviderContractError::WrongCurrentAuthorityProfile);
    }
    let value = format!(
        "{EXECUTOR_AUTHORITY_REF_SCHEME}:{profile}:{}",
        hex_digest(digest)
    );
    require_ref(&value)?;
    Ok(value)
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

fn require_id(value: &str) -> Result<(), ExecutorProviderContractError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ExecutorProviderContractError::InvalidIdentifier)
    } else {
        Ok(())
    }
}

fn require_ref(value: &str) -> Result<(), ExecutorProviderContractError> {
    if value.trim().is_empty() || value.len() > MAX_REF_BYTES {
        Err(ExecutorProviderContractError::InvalidReference)
    } else {
        Ok(())
    }
}

fn require_profile(value: &str) -> Result<(), ExecutorProviderContractError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        Err(ExecutorProviderContractError::InvalidProfile)
    } else {
        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ExecutorProviderContractError {
    WrongProtocolVersion,
    InvalidIdentifier,
    InvalidReference,
    InvalidProfile,
    InvalidRulebook,
    ZeroDigest,
    WrongThresholdIdentityProfile,
    WrongCurrentAuthorityProfile,
    WrongSemanticAuthorityProfile,
    WrongGrantIdentityProfile,
    WrongFreshnessProfile,
    InvalidValidityHorizon,
    ExecutorAuthorityRefMismatch,
    InvalidVerificationWindow,
    CurrentAuthority(CurrentExecutorAuthorityError),
}

impl fmt::Display for ExecutorProviderContractError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong executor-provider protocol version"),
            Self::InvalidIdentifier => write!(f, "invalid executor-provider identifier"),
            Self::InvalidReference => write!(f, "invalid executor-provider reference"),
            Self::InvalidProfile => write!(f, "invalid executor-provider digest profile"),
            Self::InvalidRulebook => write!(f, "invalid executor-provider rulebook"),
            Self::ZeroDigest => write!(f, "executor-provider digest must not be zero"),
            Self::WrongThresholdIdentityProfile => {
                write!(f, "wrong threshold-authorization identity profile")
            }
            Self::WrongCurrentAuthorityProfile => {
                write!(f, "wrong current-executor authority profile")
            }
            Self::WrongSemanticAuthorityProfile => {
                write!(f, "wrong semantic executor-authority profile")
            }
            Self::WrongGrantIdentityProfile => write!(f, "wrong authority-grant identity profile"),
            Self::WrongFreshnessProfile => write!(f, "wrong authority-freshness bundle profile"),
            Self::InvalidValidityHorizon => write!(f, "invalid executor-provider validity horizon"),
            Self::ExecutorAuthorityRefMismatch => {
                write!(f, "executor authority ref does not match current authority digest/profile")
            }
            Self::InvalidVerificationWindow => write!(f, "invalid executor-provider verification window"),
            Self::CurrentAuthority(error) => write!(f, "current executor authority failed: {error}"),
        }
    }
}

impl std::error::Error for ExecutorProviderContractError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_institutional_core::{RulebookId, RulebookRef};

    fn d(byte: u8) -> Digest32 {
        Digest32([byte; 32])
    }

    fn projection() -> CurrentExecutorAuthorityProjection {
        CurrentExecutorAuthorityProjection {
            proposal_id: ProposalId::new("proposal:test").unwrap(),
            actions_digest: d(1),
            actions_digest_profile: "actions-v1-blake3".into(),
            threshold_authorization_ref: "threshold:1".into(),
            threshold_authorization_identity_digest: d(2),
            threshold_authorization_identity_profile: THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE.into(),
            executor_principal: PrincipalId::new("did:example:executor").unwrap(),
            executor_authority_ref: executor_authority_ref(
                d(3),
                CURRENT_EXECUTOR_AUTHORITY_PROFILE,
            )
            .unwrap(),
            current_executor_authority_digest: d(3),
            current_executor_authority_profile: CURRENT_EXECUTOR_AUTHORITY_PROFILE.into(),
            semantic_executor_authority_digest: d(4),
            semantic_executor_authority_profile: EXECUTOR_LINEAGE_AUTHORITY_PROFILE.into(),
            authority_grant_id: AuthorityGrantId::new("grant:executor").unwrap(),
            authority_grant_identity_digest: d(5),
            authority_grant_identity_profile: AUTHORITY_GRANT_IDENTITY_PROFILE.into(),
            granting_institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:test").unwrap(),
                version: "1".into(),
                digest: d(6),
            },
            capability_scope: CapabilityId::new("governance.execute").unwrap(),
            freshness_digest: d(7),
            freshness_profile: BUNDLE_IDENTITY_PROFILE.into(),
            semantic_valid_until_ms: 1_000,
        }
    }

    #[test]
    fn authority_ref_is_stable_for_same_current_identity() {
        let a = executor_authority_ref(d(9), CURRENT_EXECUTOR_AUTHORITY_PROFILE).unwrap();
        let b = executor_authority_ref(d(9), CURRENT_EXECUTOR_AUTHORITY_PROFILE).unwrap();
        assert_eq!(a, b);
        assert!(a.starts_with("current-executor:mycelix-governance-current-executor-authority-v1-blake3-framed:"));
    }

    #[test]
    fn authority_ref_changes_when_generation_bound_identity_changes() {
        let a = executor_authority_ref(d(9), CURRENT_EXECUTOR_AUTHORITY_PROFILE).unwrap();
        let b = executor_authority_ref(d(10), CURRENT_EXECUTOR_AUTHORITY_PROFILE).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn forged_authority_ref_is_rejected() {
        let mut value = projection();
        value.executor_authority_ref = "current-executor:forged".into();
        assert_eq!(
            value.validate().unwrap_err(),
            ExecutorProviderContractError::ExecutorAuthorityRefMismatch
        );
    }

    #[test]
    fn dynamic_provider_refresh_does_not_change_authority_ref() {
        let projection = projection();
        let older = VerifiedCurrentExecutorAuthorityReceipt {
            protocol: PROTOCOL_VERSION.into(),
            projection: projection.clone(),
            verification_ref: "provider-run:1".into(),
            verified_at_ms: 100,
            valid_until_ms: 500,
        };
        let newer = VerifiedCurrentExecutorAuthorityReceipt {
            protocol: PROTOCOL_VERSION.into(),
            projection,
            verification_ref: "provider-run:2".into(),
            verified_at_ms: 200,
            valid_until_ms: 700,
        };
        assert_eq!(
            older.projection.executor_authority_ref,
            newer.projection.executor_authority_ref
        );
        older.validate_at(250).unwrap();
        newer.validate_at(250).unwrap();
    }
}
