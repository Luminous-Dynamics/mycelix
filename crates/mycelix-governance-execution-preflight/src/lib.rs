// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure fail-closed execution preflight for binding Mycelix governance.
//!
//! This crate deliberately performs no Holochain calls and accepts no Phi,
//! reputation, stake, model score, or advisory signal. A host must first resolve
//! verified proposal authority, verified current constitution, and a verified
//! binding tally; this kernel then proves those facts describe the same exact
//! proposal/action/institution/rulebook before execution may continue.

use mycelix_governance_authority::ProposalAuthorityContext;
use mycelix_governance_constitution::{ConstitutionStatement, Digest32 as ConstitutionDigest32};
use mycelix_institutional_core::Digest32 as InstitutionalDigest32;
use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-governance-execution-preflight-v0.1";
pub const ACTIONS_DIGEST_PROFILE_V1: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";
const MAX_REF_BYTES: usize = 2048;
const MAX_PROFILE_BYTES: usize = 128;

/// Host-produced reference to a binding tally that was obtained through the
/// binding-voting verified endpoint. This is evidence-shaped data, not authority
/// merely because it deserializes; adapters MUST construct it only after the
/// verified tally path succeeds.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct VerifiedBindingTallyRef {
    pub proposal_id: String,
    pub tally_action_ref: String,
    pub tally_digest: InstitutionalDigest32,
    pub tally_digest_profile: String,
    pub proposal_authority_binding_ref: String,
    pub verified_at_ms: u64,
    pub verification_receipt_ref: String,
}

impl VerifiedBindingTallyRef {
    pub fn validate(&self) -> Result<(), PreflightError> {
        validate_text(&self.proposal_id, "tally.proposal_id", MAX_REF_BYTES)?;
        validate_text(
            &self.tally_action_ref,
            "tally.tally_action_ref",
            MAX_REF_BYTES,
        )?;
        require_digest(self.tally_digest, "tally.tally_digest")?;
        validate_profile(
            &self.tally_digest_profile,
            "tally.tally_digest_profile",
        )?;
        validate_text(
            &self.proposal_authority_binding_ref,
            "tally.proposal_authority_binding_ref",
            MAX_REF_BYTES,
        )?;
        validate_text(
            &self.verification_receipt_ref,
            "tally.verification_receipt_ref",
            MAX_REF_BYTES,
        )?;
        if self.verified_at_ms == 0 {
            return Err(PreflightError::ZeroTimestamp("tally.verified_at_ms"));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionPreflightInput {
    pub protocol_version: String,
    pub proposal_id: String,
    pub proposal_authority_binding_ref: String,
    pub authority: ProposalAuthorityContext,
    pub actions_digest: InstitutionalDigest32,
    pub actions_digest_profile: String,
    pub constitution: ConstitutionStatement,
    pub expected_constitution_statement_digest: ConstitutionDigest32,
    pub binding_tally: VerifiedBindingTallyRef,
    pub checked_at_ms: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExecutionPreflightPermit {
    pub proposal_id: String,
    pub proposal_authority_binding_ref: String,
    pub constitution_statement_digest: ConstitutionDigest32,
    pub tally_action_ref: String,
    pub checked_at_ms: u64,
}

pub fn authorize_execution_preflight(
    input: &ExecutionPreflightInput,
) -> Result<ExecutionPreflightPermit, PreflightError> {
    if input.protocol_version != PROTOCOL_VERSION {
        return Err(PreflightError::WrongProtocolVersion);
    }
    validate_text(&input.proposal_id, "proposal_id", MAX_REF_BYTES)?;
    validate_text(
        &input.proposal_authority_binding_ref,
        "proposal_authority_binding_ref",
        MAX_REF_BYTES,
    )?;
    if input.checked_at_ms == 0 {
        return Err(PreflightError::ZeroTimestamp("checked_at_ms"));
    }

    input
        .authority
        .validate()
        .map_err(|_| PreflightError::InvalidProposalAuthority)?;
    input
        .constitution
        .validate()
        .map_err(|_| PreflightError::InvalidConstitution)?;
    input.binding_tally.validate()?;
    require_digest(input.actions_digest, "actions_digest")?;
    validate_profile(&input.actions_digest_profile, "actions_digest_profile")?;

    if input.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1 {
        return Err(PreflightError::UnsupportedActionsDigestProfile);
    }
    if input.authority.proposal_id.as_str() != input.proposal_id {
        return Err(PreflightError::ProposalMismatch);
    }
    if !input.authority.is_active_at(input.checked_at_ms) {
        return Err(PreflightError::ProposalAuthorityInactive);
    }
    if input.authority.actions_digest != input.actions_digest {
        return Err(PreflightError::ActionsDigestMismatch);
    }

    let computed_constitution_digest = input
        .constitution
        .digest()
        .map_err(|_| PreflightError::InvalidConstitution)?;
    if computed_constitution_digest != input.expected_constitution_statement_digest {
        return Err(PreflightError::ConstitutionDigestMismatch);
    }
    if input.constitution.effective_from_ms > input.checked_at_ms {
        return Err(PreflightError::ConstitutionNotYetEffective);
    }

    if input.authority.institution.as_str() != input.constitution.institution_id.as_str() {
        return Err(PreflightError::InstitutionMismatch);
    }
    if input.authority.rulebook.id.as_str() != input.constitution.rulebook_id.as_str()
        || input.authority.rulebook.version != input.constitution.rulebook_version
        || input.authority.rulebook.digest.0 != input.constitution.rulebook.digest.0
    {
        return Err(PreflightError::RulebookMismatch);
    }

    if input.binding_tally.proposal_id != input.proposal_id {
        return Err(PreflightError::TallyProposalMismatch);
    }
    if input.binding_tally.proposal_authority_binding_ref
        != input.proposal_authority_binding_ref
    {
        return Err(PreflightError::TallyAuthorityBindingMismatch);
    }
    if input.binding_tally.verified_at_ms > input.checked_at_ms {
        return Err(PreflightError::TallyVerificationFromFuture);
    }

    Ok(ExecutionPreflightPermit {
        proposal_id: input.proposal_id.clone(),
        proposal_authority_binding_ref: input.proposal_authority_binding_ref.clone(),
        constitution_statement_digest: computed_constitution_digest,
        tally_action_ref: input.binding_tally.tally_action_ref.clone(),
        checked_at_ms: input.checked_at_ms,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PreflightError {
    WrongProtocolVersion,
    Empty(&'static str),
    TooLong(&'static str),
    InvalidProfile(&'static str),
    ZeroDigest(&'static str),
    ZeroTimestamp(&'static str),
    InvalidProposalAuthority,
    InvalidConstitution,
    UnsupportedActionsDigestProfile,
    ProposalMismatch,
    ProposalAuthorityInactive,
    ActionsDigestMismatch,
    ConstitutionDigestMismatch,
    ConstitutionNotYetEffective,
    InstitutionMismatch,
    RulebookMismatch,
    TallyProposalMismatch,
    TallyAuthorityBindingMismatch,
    TallyVerificationFromFuture,
}

impl fmt::Display for PreflightError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocolVersion => write!(f, "wrong execution-preflight protocol version"),
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::TooLong(field) => write!(f, "{field} exceeds maximum length"),
            Self::InvalidProfile(field) => write!(f, "{field} is not a canonical profile token"),
            Self::ZeroDigest(field) => write!(f, "{field} must not be the zero digest"),
            Self::ZeroTimestamp(field) => write!(f, "{field} must be non-zero"),
            Self::InvalidProposalAuthority => write!(f, "proposal authority context is invalid"),
            Self::InvalidConstitution => write!(f, "current constitution is invalid"),
            Self::UnsupportedActionsDigestProfile => write!(f, "unsupported execution action digest profile"),
            Self::ProposalMismatch => write!(f, "proposal authority targets another proposal"),
            Self::ProposalAuthorityInactive => write!(f, "proposal authority is not active at preflight time"),
            Self::ActionsDigestMismatch => write!(f, "execution action digest differs from proposal authority"),
            Self::ConstitutionDigestMismatch => write!(f, "verified constitution digest does not match statement"),
            Self::ConstitutionNotYetEffective => write!(f, "constitution is not yet effective"),
            Self::InstitutionMismatch => write!(f, "proposal institution differs from current constitution"),
            Self::RulebookMismatch => write!(f, "proposal rulebook differs from current constitution"),
            Self::TallyProposalMismatch => write!(f, "verified binding tally targets another proposal"),
            Self::TallyAuthorityBindingMismatch => write!(f, "binding tally was verified under another proposal authority binding"),
            Self::TallyVerificationFromFuture => write!(f, "binding tally verification timestamp is in the future"),
        }
    }
}

impl std::error::Error for PreflightError {}

fn validate_text(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), PreflightError> {
    if value.trim().is_empty() {
        return Err(PreflightError::Empty(field));
    }
    if value.len() > max_bytes {
        return Err(PreflightError::TooLong(field));
    }
    Ok(())
}

fn validate_profile(value: &str, field: &'static str) -> Result<(), PreflightError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_PROFILE_BYTES
        || !bytes.iter().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(*byte, b'.' | b'_' | b'/' | b'-' | b':')
        })
    {
        return Err(PreflightError::InvalidProfile(field));
    }
    Ok(())
}

fn require_digest(
    digest: InstitutionalDigest32,
    field: &'static str,
) -> Result<(), PreflightError> {
    if digest.is_zero() {
        Err(PreflightError::ZeroDigest(field))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_authority::{GovernanceBodyId, ProposalId, SigningPolicyId, PROTOCOL_VERSION as AUTHORITY_PROTOCOL};
    use mycelix_governance_constitution::{
        ConstitutionId, InstitutionId as ConstitutionInstitutionId, NetworkId, ProfiledDigest,
        RulebookId as ConstitutionRulebookId, PROTOCOL_VERSION as CONSTITUTION_PROTOCOL,
    };
    use mycelix_institutional_core::{
        InstitutionId, RulebookId, RulebookRef,
    };

    fn idigest(byte: u8) -> InstitutionalDigest32 {
        InstitutionalDigest32([byte; 32])
    }

    fn cdigest(byte: u8) -> ConstitutionDigest32 {
        ConstitutionDigest32([byte; 32])
    }

    fn constitution() -> ConstitutionStatement {
        ConstitutionStatement {
            protocol_version: CONSTITUTION_PROTOCOL.into(),
            network_id: NetworkId::new("network:test").unwrap(),
            institution_id: ConstitutionInstitutionId::new("institution:test").unwrap(),
            constitution_id: ConstitutionId::new("constitution:test").unwrap(),
            version: 1,
            parent_statement_digest: None,
            rulebook_id: ConstitutionRulebookId::new("rulebook:test").unwrap(),
            rulebook_version: "1".into(),
            rulebook: ProfiledDigest { digest: cdigest(7), profile: "mycelix-rulebook-v1".into() },
            charter: ProfiledDigest { digest: cdigest(8), profile: "mycelix-charter-v1".into() },
            parameters: ProfiledDigest { digest: cdigest(9), profile: "mycelix-parameters-v1".into() },
            amendment_policy: ProfiledDigest { digest: cdigest(10), profile: "mycelix-amendment-policy-v1".into() },
            binding_vote_profile: "mycelix-binding-vote-v2".into(),
            threshold_authority_profile: "mycelix-threshold-authority-v1".into(),
            effective_from_ms: 1_000,
        }
    }

    fn authority() -> ProposalAuthorityContext {
        ProposalAuthorityContext {
            protocol_version: AUTHORITY_PROTOCOL.into(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            institution: InstitutionId::new("institution:test").unwrap(),
            jurisdiction: None,
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:test").unwrap(),
                version: "1".into(),
                digest: idigest(7),
            },
            governing_body: GovernanceBodyId::new("body:root").unwrap(),
            action_class: "protocol".into(),
            actions_digest: idigest(11),
            signing_policy_id: SigningPolicyId::new("policy:signing:v1").unwrap(),
            signing_policy_digest: idigest(12),
            created_at_ms: 1_000,
            expires_at_ms: 20_000,
        }
    }

    fn input() -> ExecutionPreflightInput {
        let constitution = constitution();
        ExecutionPreflightInput {
            protocol_version: PROTOCOL_VERSION.into(),
            proposal_id: "MIP-42".into(),
            proposal_authority_binding_ref: "uhCkk-authority".into(),
            authority: authority(),
            actions_digest: idigest(11),
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
            expected_constitution_statement_digest: constitution.digest().unwrap(),
            constitution,
            binding_tally: VerifiedBindingTallyRef {
                proposal_id: "MIP-42".into(),
                tally_action_ref: "uhCkk-tally".into(),
                tally_digest: idigest(13),
                tally_digest_profile: "mycelix-binding-tally-v1".into(),
                proposal_authority_binding_ref: "uhCkk-authority".into(),
                verified_at_ms: 4_000,
                verification_receipt_ref: "receipt:tally:v1".into(),
            },
            checked_at_ms: 5_000,
        }
    }

    #[test]
    fn exact_current_authority_allows_preflight() {
        let permit = authorize_execution_preflight(&input()).unwrap();
        assert_eq!(permit.proposal_id, "MIP-42");
    }

    #[test]
    fn stale_rulebook_denies_execution() {
        let mut input = input();
        input.authority.rulebook.digest = idigest(99);
        assert_eq!(authorize_execution_preflight(&input).unwrap_err(), PreflightError::RulebookMismatch);
    }

    #[test]
    fn action_plan_rebinding_denies_execution() {
        let mut input = input();
        input.actions_digest = idigest(99);
        assert_eq!(authorize_execution_preflight(&input).unwrap_err(), PreflightError::ActionsDigestMismatch);
    }

    #[test]
    fn tally_from_another_authority_binding_denies_execution() {
        let mut input = input();
        input.binding_tally.proposal_authority_binding_ref = "uhCkk-other".into();
        assert_eq!(authorize_execution_preflight(&input).unwrap_err(), PreflightError::TallyAuthorityBindingMismatch);
    }

    #[test]
    fn future_constitution_denies_execution() {
        let mut input = input();
        input.constitution.effective_from_ms = 9_000;
        input.expected_constitution_statement_digest = input.constitution.digest().unwrap();
        assert_eq!(authorize_execution_preflight(&input).unwrap_err(), PreflightError::ConstitutionNotYetEffective);
    }

    #[test]
    fn advisory_inputs_are_absent_by_type() {
        // Regression test by construction: the preflight input has no score,
        // reputation, Phi, stake, or model-advice field. A successful permit is
        // therefore derivable only from explicit institutional/cryptographic facts.
        assert!(authorize_execution_preflight(&input()).is_ok());
    }
}
