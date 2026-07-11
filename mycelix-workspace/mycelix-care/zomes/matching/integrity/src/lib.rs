// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Matching Integrity Zome
//! Defines entry types and validation for intelligent care matching.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Status of a care match
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MatchStatus {
    Suggested,
    Accepted,
    Declined,
    Completed,
}

/// Factors contributing to a match score
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct MatchFactors {
    /// How close the provider is to the requester (0.0 - 1.0)
    pub proximity_score: f32,
    /// How well the provider's skills align with the request (0.0 - 1.0)
    pub skill_alignment: f32,
    /// How compatible the schedules are (0.0 - 1.0)
    pub schedule_compatibility: f32,
    /// Trust/reputation score between the agents (0.0 - 1.0)
    pub trust_score: f32,
}

/// A match between a service offer and a service request
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CareMatch {
    /// Hash of the ServiceOffer
    pub offer_hash: ActionHash,
    /// Hash of the ServiceRequest
    pub request_hash: ActionHash,
    /// Provider agent
    pub provider: AgentPubKey,
    /// Requester agent
    pub requester: AgentPubKey,
    /// Overall match score (0.0 - 1.0)
    pub score: f32,
    /// Breakdown of contributing factors
    pub factors: MatchFactors,
    /// Current status of the match
    pub status: MatchStatus,
    /// When the match was suggested
    pub created_at: Timestamp,
    /// When the match status was last updated
    pub updated_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    CareMatch(CareMatch),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Request to suggested matches
    RequestToMatch,
    /// Offer to suggested matches
    OfferToMatch,
    /// Agent to matches they are involved in (as provider)
    AgentProviderMatches,
    /// Agent to matches they are involved in (as requester)
    AgentRequesterMatches,
    /// All pending matches
    AllPendingMatches,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::CareMatch(care_match) => validate_create_match(action, care_match),
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                original_action_hash: _,
                original_entry_hash: _,
            } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink {
            link_type: _,
            original_action,
            base_address: _,
            target_address: _,
            tag: _,
            action,
        } => {
            // Previously accepted unconditionally regardless of author --
            // the coordinator never calls delete_link here, so this is
            // pure hardening (zero functional impact). Found + fixed
            // 2026-07-09 during the P0 author-binding pass.
            if action.author != original_action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original link creator can delete a link".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 7th confirmed instance of this exact
            // bug pattern this pass. Found + fixed 2026-07-09 during the P0
            // author-binding pass. Route through the same per-type
            // validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(action, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
            // Also previously fully permissive. The coordinator never
            // calls delete_entry here, so this is pure hardening, zero
            // functional impact.
            let original = must_get_action(action.deletes_address.clone())?;
            if action.author != *original.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete an entry".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
    }
}

/// Shared per-entry-type update validation, called from BOTH the
/// StoreEntry (OpEntry::UpdateEntry) and RegisterUpdate DHT-op
/// perspectives so they agree.
fn validate_update_entry_type(
    action: Update,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::CareMatch(care_match) => validate_update_match(action, care_match),
    }
}

/// No author-binding possible on `provider`/`requester`: the coordinator's
/// suggest_match is explicitly "manual matching by an organizer or
/// system" (see its doc comment) -- a legitimate third party (not
/// necessarily either named agent) proposes the match, so both fields
/// name third parties, not the committer. Reviewed 2026-07-09 during the
/// P0 author-binding pass; case (b). Note this does mean any agent can
/// currently suggest arbitrary matches (spam/clutter risk, not a
/// privilege-escalation one -- matches only become consequential once
/// accept_match is called, which IS gated to the named provider/requester
/// in validate_update_match below).
fn validate_create_match(
    _action: Create,
    care_match: CareMatch,
) -> ExternResult<ValidateCallbackResult> {
    if care_match.score < 0.0 || care_match.score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Match score must be between 0.0 and 1.0".into(),
        ));
    }
    if care_match.factors.proximity_score < 0.0 || care_match.factors.proximity_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Proximity score must be between 0.0 and 1.0".into(),
        ));
    }
    if care_match.factors.skill_alignment < 0.0 || care_match.factors.skill_alignment > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Skill alignment must be between 0.0 and 1.0".into(),
        ));
    }
    if care_match.factors.schedule_compatibility < 0.0
        || care_match.factors.schedule_compatibility > 1.0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule compatibility must be between 0.0 and 1.0".into(),
        ));
    }
    if care_match.factors.trust_score < 0.0 || care_match.factors.trust_score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Trust score must be between 0.0 and 1.0".into(),
        ));
    }
    if care_match.provider == care_match.requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider and requester cannot be the same agent".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a CareMatch update (the coordinator's accept_match/
/// decline_match both route through update_match_status, which already
/// checks the caller is the named provider or requester).
///
/// Previously this had NO author check and never fetched the original --
/// any agent could rewrite ANY field (including provider/requester/score)
/// under the guise of "updating status". Hardened 2026-07-09 during the
/// P0 author-binding pass to mirror update_match_status's own
/// authorization (belt-and-suspenders against a modified coordinator) and
/// restrict content to status/updated_at.
fn validate_update_match(
    action: Update,
    care_match: CareMatch,
) -> ExternResult<ValidateCallbackResult> {
    if care_match.score < 0.0 || care_match.score > 1.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Match score must be between 0.0 and 1.0".into(),
        ));
    }

    if action.author != care_match.provider && action.author != care_match.requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the provider or requester can update a match".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: CareMatch = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original match not found".into()
        )))?;

    if care_match.offer_hash != original.offer_hash
        || care_match.request_hash != original.request_hash
        || care_match.provider != original.provider
        || care_match.requester != original.requester
        || care_match.score != original.score
        || care_match.factors != original.factors
        || care_match.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/updated_at can change on a match update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn update_action(author: AgentPubKey) -> Update {
        Update {
            author,
            timestamp: Timestamp::from_micros(1),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            original_action_address: ActionHash::from_raw_36(vec![9u8; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_match(provider: AgentPubKey, requester: AgentPubKey) -> CareMatch {
        CareMatch {
            offer_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            request_hash: ActionHash::from_raw_36(vec![0u8; 36]),
            provider,
            requester,
            score: 0.8,
            factors: MatchFactors {
                proximity_score: 0.8,
                skill_alignment: 0.8,
                schedule_compatibility: 0.8,
                trust_score: 0.8,
            },
            status: MatchStatus::Suggested,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn update_match_rejects_uninvolved_agent() {
        let m = valid_match(me(), other_agent());
        let uninvolved = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let result = validate_update_match(update_action(uninvolved), m).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
