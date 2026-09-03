// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integrity rules for proposal institutional authority context.
//!
//! The existing `Proposal` wire schema is intentionally left untouched. New
//! binding-governance flows attach an immutable authority context to a proposal
//! through this zome instead.

use hdi::prelude::*;
use mycelix_governance_authority::ProposalAuthorityContext;

pub const ACTIONS_DIGEST_PROFILE_V1: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Holochain persistence wrapper around the wire-neutral authority context.
///
/// `proposal_action_hash` records the proposal version observed when this
/// binding was created. Consumers MUST still recompute the current proposal's
/// action digest; this lets stale Draft-era bindings remain auditable without
/// becoming valid after the proposal content changes.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProposalAuthorityBinding {
    pub id: String,
    pub proposal_action_hash: ActionHash,
    pub proposal_author: String,
    pub context: ProposalAuthorityContext,
    /// Explicit profile for the `context.actions_digest` bytes.
    pub actions_digest_profile: String,
    /// Explicit profile/algorithm identifier for the signing-policy digest.
    /// The signing-policy registry must require an exact match before execution.
    pub signing_policy_digest_profile: String,
    pub created_at: Timestamp,
}

impl ProposalAuthorityBinding {
    pub fn validate_structure(&self) -> Result<(), String> {
        if self.id.is_empty() || self.id.len() > 512 {
            return Err("Authority binding id must be 1-512 bytes".into());
        }
        if !self.proposal_author.starts_with("did:mycelix:") {
            return Err("Proposal authority binding requires a did:mycelix author".into());
        }
        self.context
            .validate()
            .map_err(|e| format!("Invalid proposal authority context: {e}"))?;
        if !self.context.proposal_id.as_str().starts_with("MIP-") {
            return Err("Authority context proposal id must start with MIP-".into());
        }
        if self.actions_digest_profile != ACTIONS_DIGEST_PROFILE_V1 {
            return Err("Unsupported actions digest profile".into());
        }
        validate_profile_token(
            &self.signing_policy_digest_profile,
            "signing policy digest profile",
        )?;

        let micros = self.created_at.as_micros();
        if micros <= 0 {
            return Err("Authority binding timestamp must be positive".into());
        }
        let created_ms = micros as u64 / 1_000;
        // Holochain timestamps are microsecond precision while the portable
        // governance contract intentionally uses milliseconds.
        if created_ms != self.context.created_at_ms {
            return Err("Binding timestamp does not match authority context".into());
        }
        Ok(())
    }
}

fn validate_profile_token(value: &str, field: &str) -> Result<(), String> {
    let bytes = value.as_bytes();
    if bytes.is_empty() || bytes.len() > 128 {
        return Err(format!("{field} must be 1-128 ASCII bytes"));
    }
    if !bytes.iter().all(|b| {
        b.is_ascii_lowercase()
            || b.is_ascii_digit()
            || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
    }) {
        return Err(format!("{field} contains unsupported characters"));
    }
    Ok(())
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ProposalAuthorityBinding(ProposalAuthorityBinding),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// `proposal-authority:<MIP-id>` anchor -> immutable bindings.
    ProposalToAuthorityContext,
}

fn validate_create_binding(
    action: Create,
    binding: ProposalAuthorityBinding,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = binding.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }

    let expected_author = format!("did:mycelix:{}", action.author);
    if binding.proposal_author != expected_author {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Proposal authority context must be authored by proposal author: expected {expected_author}"
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ProposalAuthorityBinding(binding) => {
                    validate_create_binding(action, binding)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ProposalAuthorityBinding(_) => Ok(ValidateCallbackResult::Invalid(
                    "Proposal authority bindings are immutable; create a new Draft-era binding instead"
                        .into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Proposal authority links are append-only".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Proposal authority entries are append-only".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_governance_authority::{
        GovernanceBodyId, ProposalId, SigningPolicyId, PROTOCOL_VERSION,
    };
    use mycelix_governance_authority::ProposalAuthorityContext;
    use mycelix_institutional_core::{
        Digest32, InstitutionId, JurisdictionId, RulebookId, RulebookRef,
    };

    fn sample_context() -> ProposalAuthorityContext {
        ProposalAuthorityContext {
            protocol_version: PROTOCOL_VERSION.into(),
            proposal_id: ProposalId::new("MIP-42").unwrap(),
            institution: InstitutionId::new("institution:example").unwrap(),
            jurisdiction: Some(JurisdictionId::new("jurisdiction:za").unwrap()),
            rulebook: RulebookRef {
                id: RulebookId::new("rulebook:example").unwrap(),
                version: "1".into(),
                digest: Digest32([7; 32]),
            },
            governing_body: GovernanceBodyId::new("body:council").unwrap(),
            action_class: "standard".into(),
            actions_digest: Digest32([8; 32]),
            signing_policy_id: SigningPolicyId::new("policy:signing:v1").unwrap(),
            signing_policy_digest: Digest32([9; 32]),
            created_at_ms: 2_000,
            expires_at_ms: 20_000,
        }
    }

    #[test]
    fn binding_requires_explicit_digest_profiles() {
        let binding = ProposalAuthorityBinding {
            id: "pac:MIP-42:1".into(),
            proposal_action_hash: ActionHash::from_raw_39(vec![0; 39]),
            proposal_author: "did:mycelix:alice".into(),
            context: sample_context(),
            actions_digest_profile: ACTIONS_DIGEST_PROFILE_V1.into(),
            signing_policy_digest_profile: "mycelix-signing-policy-v1-blake3".into(),
            created_at: Timestamp::from_micros(2_000_000),
        };
        assert!(binding.validate_structure().is_ok());
    }

    #[test]
    fn unsupported_actions_profile_fails_closed() {
        let mut binding = ProposalAuthorityBinding {
            id: "pac:MIP-42:1".into(),
            proposal_action_hash: ActionHash::from_raw_39(vec![0; 39]),
            proposal_author: "did:mycelix:alice".into(),
            context: sample_context(),
            actions_digest_profile: "json-whatever".into(),
            signing_policy_digest_profile: "mycelix-signing-policy-v1-blake3".into(),
            created_at: Timestamp::from_micros(2_000_000),
        };
        assert!(binding.validate_structure().is_err());
        binding.actions_digest_profile = ACTIONS_DIGEST_PROFILE_V1.into();
        assert!(binding.validate_structure().is_ok());
    }
}
