// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Holochain integrity adapter for Hearth membership authority proofs.
//!
//! This crate gathers deterministic source-chain evidence with HDI host calls,
//! decodes canonical Kinship membership records, and delegates the actual
//! freshness decision to `hearth-authority-proof`.
//!
//! It intentionally does not contain Care-specific policy.

use hdi::prelude::*;
use hdk::prelude::ChainFilter;
use hearth_authority_proof::{
    MembershipAuthorityError, MembershipRevision, require_fresh_active_membership,
};
use hearth_kinship_integrity::HearthMembership;
use hearth_types::MemberRole;
use std::fmt;

const KINSHIP_INTEGRITY_ZOME: &str = "hearth_kinship_integrity";
// Published historical EntryTypes order in hearth_kinship_integrity:
// Hearth = 0, HearthMembership = 1. This is a wire-compatibility constant.
const KINSHIP_MEMBERSHIP_ENTRY_INDEX: u8 = 1;

/// Structural failure while converting Holochain/Kinship evidence into the
/// pure membership authority kernel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MembershipEvidenceError {
    TransitionAuthorMismatch,
    ClaimedRecordAuthorMismatch,
    ClaimedRecordNotAppEntry,
    ClaimedRecordWrongEntryType,
    ClaimedRecordMissingEntry,
    ClaimedRecordDecode(String),
    ActivityRecordMissingEntry(ActionHash),
    ActivityRecordDecode {
        action_hash: ActionHash,
        message: String,
    },
    Authority(MembershipAuthorityError),
}

impl fmt::Display for MembershipEvidenceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TransitionAuthorMismatch => {
                write!(f, "transition action author does not match claimed actor")
            }
            Self::ClaimedRecordAuthorMismatch => {
                write!(f, "claimed membership action was not authored by the actor")
            }
            Self::ClaimedRecordNotAppEntry => {
                write!(f, "claimed membership action is not an application entry")
            }
            Self::ClaimedRecordWrongEntryType => write!(
                f,
                "claimed membership action is not hearth_kinship_integrity::HearthMembership"
            ),
            Self::ClaimedRecordMissingEntry => {
                write!(f, "claimed membership record is missing its entry")
            }
            Self::ClaimedRecordDecode(message) => {
                write!(f, "failed to decode claimed HearthMembership: {message}")
            }
            Self::ActivityRecordMissingEntry(action_hash) => {
                write!(f, "membership activity record {action_hash:?} is missing its entry")
            }
            Self::ActivityRecordDecode {
                action_hash,
                message,
            } => write!(
                f,
                "failed to decode HearthMembership activity record {action_hash:?}: {message}"
            ),
            Self::Authority(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for MembershipEvidenceError {}

impl From<MembershipAuthorityError> for MembershipEvidenceError {
    fn from(value: MembershipAuthorityError) -> Self {
        Self::Authority(value)
    }
}

fn decode_membership_record(
    record: &Record,
    action_hash: &ActionHash,
    claimed: bool,
) -> Result<HearthMembership, MembershipEvidenceError> {
    match record.entry().to_app_option::<HearthMembership>() {
        Ok(Some(membership)) => Ok(membership),
        Ok(None) if claimed => Err(MembershipEvidenceError::ClaimedRecordMissingEntry),
        Ok(None) => Err(MembershipEvidenceError::ActivityRecordMissingEntry(
            action_hash.clone(),
        )),
        Err(error) if claimed => Err(MembershipEvidenceError::ClaimedRecordDecode(
            error.to_string(),
        )),
        Err(error) => Err(MembershipEvidenceError::ActivityRecordDecode {
            action_hash: action_hash.clone(),
            message: error.to_string(),
        }),
    }
}

fn reduce_membership(
    action_hash: ActionHash,
    membership: HearthMembership,
) -> MembershipRevision {
    MembershipRevision::new(
        action_hash,
        membership.hearth_hash,
        membership.agent,
        membership.role,
        membership.status,
    )
}

/// Bind a claimed membership AppEntryDef to the canonical Kinship membership
/// entry type without hard-coding the Kinship zome's numeric position.
fn is_kinship_membership_entry_def(
    entry_def: &AppEntryDef,
    integrity_zome_names: &[ZomeName],
) -> bool {
    let zome_name = integrity_zome_names.get(entry_def.zome_index.0 as usize);
    zome_name == Some(&ZomeName::new(KINSHIP_INTEGRITY_ZOME))
        && entry_def.entry_index.0 == KINSHIP_MEMBERSHIP_ENTRY_INDEX
}

/// Prove that `claimed_membership_hash` is the actor's latest matching Active
/// HearthMembership revision before `transition_action`.
///
/// Holochain host failures and unresolved dependencies propagate through
/// `ExternResult` so callers cannot accidentally strengthen missing evidence
/// into authority. Structural/semantic proof failures are returned separately
/// as `MembershipEvidenceError` and should normally become
/// `ValidateCallbackResult::Invalid` at the calling integrity boundary.
pub fn require_fresh_active_membership_from_chain(
    transition_action: &Create,
    actor: &AgentPubKey,
    hearth_hash: &ActionHash,
    claimed_membership_hash: &ActionHash,
) -> ExternResult<Result<MemberRole, MembershipEvidenceError>> {
    if &transition_action.author != actor {
        return Ok(Err(MembershipEvidenceError::TransitionAuthorMismatch));
    }

    let claimed_record = must_get_valid_record(claimed_membership_hash.clone())?;
    if claimed_record.action().author() != actor {
        return Ok(Err(MembershipEvidenceError::ClaimedRecordAuthorMismatch));
    }

    // Derive the exact AppEntryDef from the validated claim, then prove that it
    // is the canonical Kinship membership type. Structural Serde compatibility
    // from another zome is not membership authority.
    let claimed_entry_def = match claimed_record.action().app_entry_def() {
        Some(entry_def) => entry_def.clone(),
        None => return Ok(Err(MembershipEvidenceError::ClaimedRecordNotAppEntry)),
    };
    let dna = dna_info()?;
    if !is_kinship_membership_entry_def(&claimed_entry_def, &dna.zome_names) {
        return Ok(Err(MembershipEvidenceError::ClaimedRecordWrongEntryType));
    }

    let claimed_membership = match decode_membership_record(
        &claimed_record,
        claimed_membership_hash,
        true,
    ) {
        Ok(membership) => membership,
        Err(error) => return Ok(Err(error)),
    };
    let claimed_revision = reduce_membership(
        claimed_membership_hash.clone(),
        claimed_membership,
    );

    let previous_action = transition_action
        .prev_action()
        .cloned()
        .expect("a Create action always has a previous action");

    // `until_hash` is inclusive. The returned activity is ordered newest to
    // oldest, which is exactly the order expected by the pure proof kernel.
    let activity = must_get_agent_activity(
        actor.clone(),
        ChainFilter::until_hash(previous_action, claimed_membership_hash.clone()),
    )?;

    let mut revisions = Vec::new();
    for activity_item in activity {
        let activity_action = &activity_item.action.hashed.content;
        if activity_action.app_entry_def() != Some(&claimed_entry_def) {
            continue;
        }

        let action_hash = activity_item.action.as_hash().clone();
        let record = must_get_valid_record(action_hash.clone())?;
        let membership = match decode_membership_record(&record, &action_hash, false) {
            Ok(membership) => membership,
            Err(error) => return Ok(Err(error)),
        };
        revisions.push(reduce_membership(action_hash, membership));
    }

    Ok(require_fresh_active_membership(
        actor,
        hearth_hash,
        &claimed_revision,
        revisions.iter(),
    )
    .map_err(MembershipEvidenceError::from))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn app_entry_def(zome_index: u8, entry_index: u8) -> AppEntryDef {
        AppEntryDef {
            zome_index: zome_index.into(),
            entry_index: entry_index.into(),
            visibility: EntryVisibility::Public,
        }
    }

    fn zomes() -> Vec<ZomeName> {
        vec![
            ZomeName::new("unrelated_integrity"),
            ZomeName::new(KINSHIP_INTEGRITY_ZOME),
            ZomeName::new("hearth_care_transitions_integrity"),
        ]
    }

    #[test]
    fn adapter_errors_preserve_authority_error_identity() {
        let error = MembershipEvidenceError::from(
            MembershipAuthorityError::ClaimedHearthMismatch,
        );
        assert_eq!(
            error,
            MembershipEvidenceError::Authority(
                MembershipAuthorityError::ClaimedHearthMismatch
            )
        );
    }

    #[test]
    fn membership_type_provenance_accepts_exact_kinship_entry() {
        assert!(is_kinship_membership_entry_def(
            &app_entry_def(1, KINSHIP_MEMBERSHIP_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn membership_type_provenance_rejects_serde_lookalike_in_other_zome() {
        assert!(!is_kinship_membership_entry_def(
            &app_entry_def(0, KINSHIP_MEMBERSHIP_ENTRY_INDEX),
            &zomes(),
        ));
    }

    #[test]
    fn membership_type_provenance_rejects_other_kinship_entry() {
        assert!(!is_kinship_membership_entry_def(
            &app_entry_def(1, 0),
            &zomes(),
        ));
    }

    #[test]
    fn membership_type_provenance_rejects_out_of_range_zome_index() {
        assert!(!is_kinship_membership_entry_def(
            &app_entry_def(9, KINSHIP_MEMBERSHIP_ENTRY_INDEX),
            &zomes(),
        ));
    }
}
