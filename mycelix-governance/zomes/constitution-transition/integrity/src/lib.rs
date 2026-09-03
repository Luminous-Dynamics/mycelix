// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only candidate records for constitutional transitions.
//!
//! A candidate is NOT authority by existence. The coordinator re-verifies its
//! binding tally and threshold authorization every time it projects the current
//! constitution.

use hdi::prelude::*;
use mycelix_governance_constitution::{
    ConstitutionStatement, ConstitutionTransitionAuthorization,
};

const MAX_ID_BYTES: usize = 512;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ConstitutionTransitionCandidate {
    pub id: String,
    pub child: ConstitutionStatement,
    pub authorization: ConstitutionTransitionAuthorization,
    pub submitted_by: String,
    pub submitted_at: Timestamp,
}

impl ConstitutionTransitionCandidate {
    pub fn validate_structure(&self) -> Result<(), String> {
        if self.id.trim().is_empty() || self.id.len() > MAX_ID_BYTES {
            return Err("Transition candidate id must be 1-512 bytes".into());
        }
        if !self.submitted_by.starts_with("did:mycelix:") {
            return Err("Transition candidate submitter must be a did:mycelix identifier".into());
        }
        if self.submitted_at.as_micros() <= 0 {
            return Err("Transition candidate timestamp must be positive".into());
        }
        self.child
            .validate()
            .map_err(|e| format!("Invalid child constitution statement: {e}"))?;
        self.authorization
            .validate()
            .map_err(|e| format!("Invalid constitutional transition authorization: {e}"))?;
        let child_digest = self
            .child
            .digest()
            .map_err(|e| format!("Cannot digest child constitution: {e}"))?;
        if child_digest != self.authorization.to_statement_digest {
            return Err("Transition candidate child does not match authorization target".into());
        }
        Ok(())
    }
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ConstitutionTransitionCandidate(ConstitutionTransitionCandidate),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllTransitionCandidates,
}

fn validate_create_candidate(
    action: Create,
    candidate: ConstitutionTransitionCandidate,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(error) = candidate.validate_structure() {
        return Ok(ValidateCallbackResult::Invalid(error));
    }
    let author_did = format!("did:mycelix:{}", action.author);
    if candidate.submitted_by != author_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Transition candidate submitted_by must equal the committing agent".into(),
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
                EntryTypes::ConstitutionTransitionCandidate(candidate) => {
                    validate_create_candidate(action, candidate)
                }
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ConstitutionTransitionCandidate(_) => Ok(
                    ValidateCallbackResult::Invalid(
                        "Constitutional transition candidates are append-only".into(),
                    ),
                ),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Constitutional transition candidates are append-only".into(),
        )),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Constitutional transition links are append-only".into(),
        )),
        FlatOp::RegisterCreateLink { .. }
        | FlatOp::StoreRecord(_)
        | FlatOp::RegisterAgentActivity(_)
        | FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
    }
}
