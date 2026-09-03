// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Integrity rules for proposal constitutional-epoch bindings.
//!
//! These entries do not become authority merely because they exist. The
//! coordinator's verified read re-resolves proposal authority, the binding
//! election, and the current verified constitution. Integrity only supplies the
//! append-only provenance and timing facts that must remain immutable.

use hdi::prelude::*;
use mycelix_governance_constitution::Digest32;

const MAX_ID_BYTES: usize = 512;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Immutable statement that one proposal/election crossed into binding
/// governance under one exact constitutional statement digest.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProposalConstitutionEpochBinding {
    pub id: String,
    pub proposal_id: String,
    pub proposal_authority_binding: ActionHash,
    pub election_configuration: ActionHash,
    pub constitution_statement_digest: Digest32,
    pub constitution_version: u64,
    pub ballot_opens_at_ms: u64,
    pub bound_by: String,
    pub bound_at: Timestamp,
}

impl ProposalConstitutionEpochBinding {
    pub fn validate_structure(&self) -> Result<(), String> {
        validate_text(&self.id, "epoch binding id")?;
        validate_text(&self.proposal_id, "proposal id")?;
        if !self.proposal_id.starts_with("MIP-") {
            return Err("Proposal constitutional epoch requires an MIP proposal id".into());
        }
        if self.constitution_statement_digest.is_zero() {
            return Err("Constitution statement digest must not be zero".into());
        }
        if self.constitution_version == 0 {
            return Err("Constitution version must be non-zero".into());
        }
        if self.ballot_opens_at_ms == 0 {
            return Err("Ballot opening time must be non-zero".into());
        }
        require_mycelix_did(&self.bound_by, "epoch binding publisher")?;
        let bound_at_ms = timestamp_ms(self.bound_at, "epoch bound_at")?;
        if bound_at_ms > self.ballot_opens_at_ms {
            return Err("Constitutional epoch must be bound no later than ballot opening".into());
        }
        Ok(())
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ProposalConstitutionEpoch(ProposalConstitutionEpochBinding),
}

#[hdk_link_types]
pub enum LinkTypes {
    ProposalToConstitutionEpoch,
}

fn validate_create_epoch(
    action: Create,
    entry: ProposalConstitutionEpochBinding,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = entry.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let expected_author = format!("did:mycelix:{}", action.author);
    if entry.bound_by != expected_author {
        return Ok(ValidateCallbackResult::Invalid(
            "Epoch binding bound_by must equal the committing agent".into(),
        ));
    }
    if timestamp_ms(action.timestamp, "epoch action timestamp")?
        != timestamp_ms(entry.bound_at, "epoch bound_at")?
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Epoch binding timestamp must equal the create-action timestamp".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ProposalConstitutionEpoch(entry) => validate_create_epoch(action, entry),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ProposalConstitutionEpoch(_) => Ok(ValidateCallbackResult::Invalid(
                    "Proposal constitutional epoch bindings are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Proposal constitutional epoch links are append-only".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Proposal constitutional epoch entries are append-only".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_text(value: &str, field: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{field} must not be empty"));
    }
    if value.len() > MAX_ID_BYTES {
        return Err(format!("{field} exceeds {MAX_ID_BYTES} bytes"));
    }
    Ok(())
}

fn require_mycelix_did(value: &str, field: &str) -> Result<(), String> {
    validate_text(value, field)?;
    if !value.starts_with("did:mycelix:") {
        return Err(format!("{field} must be a did:mycelix identifier"));
    }
    Ok(())
}

fn timestamp_ms(timestamp: Timestamp, field: &str) -> Result<u64, String> {
    let micros = timestamp.as_micros();
    if micros <= 0 {
        return Err(format!("{field} must be positive"));
    }
    Ok(micros as u64 / 1_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(ms: i64) -> Timestamp {
        Timestamp::from_micros(ms * 1_000)
    }

    #[test]
    fn epoch_must_precede_ballot_open() {
        let binding = ProposalConstitutionEpochBinding {
            id: "epoch:MIP-42".into(),
            proposal_id: "MIP-42".into(),
            proposal_authority_binding: ActionHash::from_raw_36(vec![1; 36]),
            election_configuration: ActionHash::from_raw_36(vec![2; 36]),
            constitution_statement_digest: Digest32([3; 32]),
            constitution_version: 1,
            ballot_opens_at_ms: 5_000,
            bound_by: "did:mycelix:test".into(),
            bound_at: ts(5_001),
        };
        assert!(binding.validate_structure().is_err());
    }

    #[test]
    fn zero_constitution_digest_is_rejected() {
        let binding = ProposalConstitutionEpochBinding {
            id: "epoch:MIP-42".into(),
            proposal_id: "MIP-42".into(),
            proposal_authority_binding: ActionHash::from_raw_36(vec![1; 36]),
            election_configuration: ActionHash::from_raw_36(vec![2; 36]),
            constitution_statement_digest: Digest32::ZERO,
            constitution_version: 1,
            ballot_opens_at_ms: 5_000,
            bound_by: "did:mycelix:test".into(),
            bound_at: ts(4_000),
        };
        assert!(binding.validate_structure().is_err());
    }
}
