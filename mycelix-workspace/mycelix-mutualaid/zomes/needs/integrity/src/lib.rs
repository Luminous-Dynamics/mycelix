// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Needs Integrity Zome
//!
//! This zome defines entry types and validation rules for needs matching
//! in the Mycelix Mutual Aid hApp. Supports needs, offers, matches, and fulfillments.

use hdi::prelude::*;
use mutualaid_common::*;

/// Entry types for the needs zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// A need from a member
    #[entry_type(visibility = "public")]
    Need(Need),
    /// An offer from a member
    #[entry_type(visibility = "public")]
    Offer(Offer),
    /// A match between need and offer
    #[entry_type(visibility = "public")]
    Match(Match),
    /// Fulfillment record
    #[entry_type(visibility = "public")]
    Fulfillment(Fulfillment),
}

/// Link types for the needs zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Link from agent to their needs
    AgentToNeeds,
    /// Link from agent to their offers
    AgentToOffers,
    /// Link from category anchor to needs
    CategoryToNeeds,
    /// Link from category anchor to offers
    CategoryToOffers,
    /// Link from need to its matches
    NeedToMatches,
    /// Link from offer to its matches
    OfferToMatches,
    /// Link from match to fulfillment
    MatchToFulfillment,
    /// Link for all needs discovery
    AllNeeds,
    /// Link for all offers discovery
    AllOffers,
    /// Link for emergency needs
    EmergencyNeeds,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => validate_create_entry(action, app_entry),
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                action,
                ..
            } => validate_update_entry_type(action, original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address,
            target_address,
            tag,
            ..
        } => validate_create_link(link_type, base_address, target_address, tag),
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap): this
        // coordinator never calls delete_link, so there's nothing to
        // harden against either way.
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            let _ = link_type;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally via the catch-all `_` arm) -- the 31st
            // confirmed instance of this exact bug pattern this pass.
            // Found + fixed 2026-07-09 during the P0 author-binding pass.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(
                action.clone(),
                action.original_action_address,
                app_entry,
            ),
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
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

/// Validate entry creation
fn validate_create_entry(
    action: Create,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Need(need) => {
            // Author-binding: create_need already derives requester
            // from agent_info(), so this is belt-and-suspenders. Found +
            // fixed 2026-07-09 during the P0 author-binding pass.
            if need.requester != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Need requester must correspond to the committing agent".into(),
                ));
            }
            validate_need(need)
        }
        EntryTypes::Offer(offer) => {
            if offer.offerer != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Offer offerer must correspond to the committing agent".into(),
                ));
            }
            validate_offer(offer)
        }
        // Match.requester/offerer are NOT bound directly to the
        // committer: propose_match derives them from the ALREADY-
        // validated Need/Offer records (need.requester/offer.offerer),
        // not from caller input -- safety is transitive through those
        // earlier-bound records, the same pattern used in
        // mycelix-housing/membership's Member.agent. propose_match may
        // legitimately be called by a third-party matchmaker, not
        // either participant.
        EntryTypes::Match(m) => validate_match(m),
        EntryTypes::Fulfillment(fulfillment) => validate_create_fulfillment(action, fulfillment),
    }
}

/// **Real, fixable check**: fulfill_match always sets exactly one of
/// requester_confirmed/offerer_confirmed to true, matching whichever
/// role the calling agent has in the referenced Match -- verified here
/// via a cross-entry must_get on match_hash, closing the gap where a
/// modified coordinator could otherwise forge a false
/// participant-confirmation claim (e.g. claiming the requester
/// confirmed when they never called fulfill_match at all).
fn validate_create_fulfillment(
    action: Create,
    fulfillment: Fulfillment,
) -> ExternResult<ValidateCallbackResult> {
    let match_record = must_get_valid_record(fulfillment.match_hash.clone())?;
    let m: Match = match_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Referenced match not found".into()
        )))?;

    let is_requester = action.author == m.requester;
    let is_offerer = action.author == m.offerer;

    if fulfillment.requester_confirmed != is_requester
        || fulfillment.offerer_confirmed != is_offerer
    {
        return Ok(ValidateCallbackResult::Invalid(
            "requester_confirmed/offerer_confirmed must reflect the committing agent's actual role in the match".into(),
        ));
    }
    if !is_requester && !is_offerer {
        return Ok(ValidateCallbackResult::Invalid(
            "Only a match participant can record a fulfillment".into(),
        ));
    }

    validate_fulfillment(fulfillment)
}

/// Only Need/Offer/Match have live coordinator update paths.
/// Fulfillment has none (confirmed via grep for `update_entry`) and is
/// made immutable. Reviewed 2026-07-09 during the P0 author-binding
/// pass: previously all 4 entry types routed updates through the same
/// create-shaped validator with no comparison to the original.
fn validate_update_entry_type(
    action: Update,
    original_action_hash: ActionHash,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::Need(need) => validate_update_need(action, original_action_hash, need),
        EntryTypes::Offer(offer) => validate_update_offer(action, original_action_hash, offer),
        EntryTypes::Match(m) => validate_update_match(action, original_action_hash, m),
        EntryTypes::Fulfillment(_) => Ok(ValidateCallbackResult::Invalid(
            "Fulfillment records are immutable".into(),
        )),
    }
}

/// **Real authorization fix**: withdraw_need's own client-side check
/// ("only the requester can withdraw") is now enforced at the integrity
/// level for the Withdrawn transition specifically. Other status
/// transitions (e.g. fulfill_match setting Fulfilled) have no author
/// requirement -- fulfill_match may legitimately be called by either
/// match participant, not necessarily the need's own requester. Content
/// is otherwise restricted to status only.
fn validate_update_need(
    action: Update,
    original_action_hash: ActionHash,
    need: Need,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Need = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original need not found".into()
        )))?;

    if need.id != original.id
        || need.requester != original.requester
        || need.category != original.category
        || need.title != original.title
        || need.description != original.description
        || need.urgency != original.urgency
        || need.emergency != original.emergency
        || need.quantity != original.quantity
        || need.location != original.location
        || need.needed_by != original.needed_by
        || need.reciprocity_offers != original.reciprocity_offers
        || need.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on a need update".into(),
        ));
    }

    if need.status == NeedStatus::Withdrawn && action.author != original.requester {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the requester can withdraw their need".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Same pattern as validate_update_need: withdraw_offer's "only the
/// offerer can withdraw" check is enforced for the Withdrawn
/// transition; fulfill_match's Completed transition has no author
/// requirement (either participant may complete it).
fn validate_update_offer(
    action: Update,
    original_action_hash: ActionHash,
    offer: Offer,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Offer = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original offer not found".into()
        )))?;

    if offer.id != original.id
        || offer.offerer != original.offerer
        || offer.category != original.category
        || offer.title != original.title
        || offer.description != original.description
        || offer.quantity != original.quantity
        || offer.condition != original.condition
        || offer.location != original.location
        || offer.available_until != original.available_until
        || offer.asking_for != original.asking_for
        || offer.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status can change on an offer update".into(),
        ));
    }

    if offer.status == OfferStatus::Withdrawn && action.author != original.offerer {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the offerer can withdraw their offer".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// **Real authorization fix**: accept_match/schedule_handoff/
/// fulfill_match ALL share the same client-side "only participants" check
/// -- uniformly enforced here at the integrity level. Content restricted
/// to status/scheduled_handoff/handoff_location (the union of fields
/// those three flows actually change).
fn validate_update_match(
    action: Update,
    original_action_hash: ActionHash,
    m: Match,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: Match = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original match not found".into()
        )))?;

    if action.author != original.requester && action.author != original.offerer {
        return Ok(ValidateCallbackResult::Invalid(
            "Only a match participant can update it".into(),
        ));
    }

    if m.id != original.id
        || m.need_hash != original.need_hash
        || m.offer_hash != original.offer_hash
        || m.requester != original.requester
        || m.offerer != original.offerer
        || m.quantity != original.quantity
        || m.notes != original.notes
        || m.matched_at != original.matched_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/scheduled_handoff/handoff_location can change on a match update".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a need
fn validate_need(need: Need) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if need.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Need ID cannot be empty".to_string(),
        ));
    }

    // Title must not be empty
    if need.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Need title cannot be empty".to_string(),
        ));
    }

    // Title length limit
    if need.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Need title cannot exceed 200 characters".to_string(),
        ));
    }

    // Description length limit
    if need.description.len() > 3000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Need description cannot exceed 3000 characters".to_string(),
        ));
    }

    // Reciprocity offers limit
    if need.reciprocity_offers.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 10 reciprocity offers".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate an offer
fn validate_offer(offer: Offer) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if offer.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer ID cannot be empty".to_string(),
        ));
    }

    // Title must not be empty
    if offer.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title cannot be empty".to_string(),
        ));
    }

    // Title length limit
    if offer.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title cannot exceed 200 characters".to_string(),
        ));
    }

    // Description length limit
    if offer.description.len() > 3000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer description cannot exceed 3000 characters".to_string(),
        ));
    }

    // Asking for limit
    if offer.asking_for.len() > 10 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 10 asking for items".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a match
fn validate_match(m: Match) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if m.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Match ID cannot be empty".to_string(),
        ));
    }

    // Requester and offerer must be different
    if m.requester == m.offerer {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester and offerer must be different".to_string(),
        ));
    }

    // Notes length limit
    if m.notes.len() > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Match notes cannot exceed 1000 characters".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a fulfillment
fn validate_fulfillment(fulfillment: Fulfillment) -> ExternResult<ValidateCallbackResult> {
    // Notes length limit
    if fulfillment.notes.len() > 1000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Fulfillment notes cannot exceed 1000 characters".to_string(),
        ));
    }

    // Gratitude message length limit
    if let Some(ref msg) = fulfillment.gratitude_message {
        if msg.len() > 500 {
            return Ok(ValidateCallbackResult::Invalid(
                "Gratitude message cannot exceed 500 characters".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate link creation
fn validate_create_link(
    link_type: LinkTypes,
    _base_address: AnyLinkableHash,
    _target_address: AnyLinkableHash,
    _tag: LinkTag,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::AgentToNeeds => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AgentToOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CategoryToNeeds => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CategoryToOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::NeedToMatches => Ok(ValidateCallbackResult::Valid),
        LinkTypes::OfferToMatches => Ok(ValidateCallbackResult::Valid),
        LinkTypes::MatchToFulfillment => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllNeeds => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::EmergencyNeeds => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn create_action(author: AgentPubKey) -> Create {
        Create {
            author,
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
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

    fn valid_need(requester: AgentPubKey) -> Need {
        Need {
            id: "n-1".into(),
            requester,
            category: NeedCategory::Food,
            title: "Groceries".into(),
            description: "".into(),
            urgency: UrgencyLevel::Low,
            emergency: false,
            quantity: None,
            location: LocationConstraint::ToBeArranged,
            needed_by: None,
            reciprocity_offers: vec![],
            status: NeedStatus::Open,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_need_valid_when_committer_is_requester() {
        let author = me();
        let need = valid_need(author.clone());
        let result = validate_create_entry(create_action(author), EntryTypes::Need(need)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_need_forgery_rejected() {
        let need = valid_need(other_agent());
        let result = validate_create_entry(create_action(me()), EntryTypes::Need(need)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_offer(offerer: AgentPubKey) -> Offer {
        Offer {
            id: "o-1".into(),
            offerer,
            category: NeedCategory::Food,
            title: "Spare veggies".into(),
            description: "".into(),
            quantity: None,
            condition: None,
            location: LocationConstraint::ToBeArranged,
            available_until: None,
            asking_for: vec![],
            status: OfferStatus::Available,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_offer_valid_when_committer_is_offerer() {
        let author = me();
        let offer = valid_offer(author.clone());
        let result =
            validate_create_entry(create_action(author), EntryTypes::Offer(offer)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_offer_forgery_rejected() {
        let offer = valid_offer(other_agent());
        let result = validate_create_entry(create_action(me()), EntryTypes::Offer(offer)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn match_requires_distinct_participants() {
        let m = Match {
            id: "m-1".into(),
            need_hash: ActionHash::from_raw_36(vec![2u8; 36]),
            offer_hash: ActionHash::from_raw_36(vec![3u8; 36]),
            requester: me(),
            offerer: me(),
            quantity: None,
            notes: "".into(),
            status: MatchStatus::Proposed,
            matched_at: Timestamp::from_micros(0),
            scheduled_handoff: None,
            handoff_location: None,
        };
        let result = validate_match(m).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_create_fulfillment / validate_update_{need,offer,match} all
    // call must_get_valid_record, which requires a live HDI host and can't
    // run in a plain unit test -- matching the established pattern from
    // circles' validate_update_credit_line (also untested directly for the
    // same reason). Correctness there is verified via cargo check plus the
    // code-review reasoning in the commit message.
}
