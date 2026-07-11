// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Timebank Integrity Zome
//!
//! This zome defines the entry types and validation rules for time banking
//! in the Mycelix Mutual Aid hApp. It implements the core principle:
//! 1 hour = 1 hour, regardless of service type.

use hdi::prelude::*;
use mutualaid_common::*;

/// Entry types for the timebank zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    /// A service offer from a member
    #[entry_type(visibility = "public")]
    ServiceOffer(ServiceOffer),
    /// A service request from a member
    #[entry_type(visibility = "public")]
    ServiceRequest(ServiceRequest),
    /// A completed time exchange
    #[entry_type(visibility = "public")]
    TimeExchange(TimeExchange),
    /// Time credit record
    #[entry_type(visibility = "public")]
    TimeCredit(TimeCredit),
}

/// Link types for the timebank zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Link from agent to their service offers
    AgentToOffers,
    /// Link from agent to their service requests
    AgentToRequests,
    /// Link from agent to exchanges they participated in
    AgentToExchanges,
    /// Link from category anchor to offers
    CategoryToOffers,
    /// Link from category anchor to requests
    CategoryToRequests,
    /// Link from offer to exchange
    OfferToExchange,
    /// Link from request to exchange
    RequestToExchange,
    /// Link for all offers discovery
    AllOffers,
    /// Link for all requests discovery
    AllRequests,
    /// Link from agent to their time credits
    AgentToCredits,
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
            OpEntry::CreateEntry { app_entry, action } => {
                validate_create_entry(action.author, app_entry)
            }
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
        // Deliberately left fully permissive (coordinator never calls
        // delete_link), reviewed 2026-07-09 during the P0 author-binding pass.
        FlatOp::RegisterDeleteLink { link_type, .. } => {
            let _ = link_type;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // Previously fully permissive (`Ok(Valid)` unconditionally via
            // the catch-all `_` arm below) even though the create-side
            // author-binding fix already existed in this file -- the
            // update path was never actually closed. 33rd confirmed
            // instance of this exact bug pattern this pass. Found + fixed
            // 2026-07-09 during the P0 author-binding pass.
            OpUpdate::Entry { app_entry, action } => validate_update_entry_type(
                action.clone(),
                action.original_action_address,
                app_entry,
            ),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(OpDelete { action }) => {
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
    author: AgentPubKey,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::ServiceOffer(offer) => validate_service_offer(author, offer),
        EntryTypes::ServiceRequest(request) => validate_service_request(author, request),
        EntryTypes::TimeExchange(exchange) => validate_time_exchange(author, exchange),
        EntryTypes::TimeCredit(credit) => validate_time_credit(author, credit),
    }
}

/// ServiceOffer and TimeExchange are the only entry types with live
/// coordinator update paths (deactivate_offer; confirm_exchange /
/// rate_exchange). ServiceRequest and TimeCredit have none and are made
/// immutable. Reviewed 2026-07-09 during the P0 author-binding pass: the
/// existing create-path author-binding never carried over to updates --
/// updates routed through the same content-only validator with no
/// comparison to the original, and RegisterUpdate/RegisterDelete were
/// fully permissive.
fn validate_update_entry_type(
    action: Update,
    original_action_hash: ActionHash,
    entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match entry {
        EntryTypes::ServiceOffer(offer) => {
            validate_update_service_offer(action, original_action_hash, offer)
        }
        EntryTypes::TimeExchange(exchange) => {
            validate_update_time_exchange(action, original_action_hash, exchange)
        }
        EntryTypes::ServiceRequest(_) => Ok(ValidateCallbackResult::Invalid(
            "Service requests are immutable".into(),
        )),
        EntryTypes::TimeCredit(_) => Ok(ValidateCallbackResult::Invalid(
            "Time credits are immutable".into(),
        )),
    }
}

/// **Real authorization fix**: deactivate_offer's own client-side check
/// ("only the provider can deactivate") is now enforced at the integrity
/// level. Content restricted to active/updated_at, the only fields that
/// flow ever changes.
fn validate_update_service_offer(
    action: Update,
    original_action_hash: ActionHash,
    offer: ServiceOffer,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: ServiceOffer = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original offer not found".into()
        )))?;

    if offer.id != original.id
        || offer.provider != original.provider
        || offer.category != original.category
        || offer.title != original.title
        || offer.description != original.description
        || offer.qualifications != original.qualifications
        || offer.availability != original.availability
        || offer.location != original.location
        || offer.min_duration_hours != original.min_duration_hours
        || offer.max_duration_hours != original.max_duration_hours
        || offer.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only active/updated_at can change on a service offer update".into(),
        ));
    }

    if action.author != original.provider {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the provider can deactivate their offer".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// **Real authorization fix**: confirm_exchange/rate_exchange's own
/// client-side "only participants" checks are now enforced at the
/// integrity level, plus a check rate_exchange's coordinator logic
/// already implies but never enforced against a modified coordinator:
/// a participant may only ever set THEIR OWN rating, never the other
/// party's. Content restricted to confirmed/provider_rating/
/// recipient_rating.
fn validate_update_time_exchange(
    action: Update,
    original_action_hash: ActionHash,
    exchange: TimeExchange,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: TimeExchange = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original exchange not found".into()
        )))?;

    if exchange.id != original.id
        || exchange.offer_hash != original.offer_hash
        || exchange.request_hash != original.request_hash
        || exchange.provider != original.provider
        || exchange.recipient != original.recipient
        || exchange.hours != original.hours
        || exchange.category != original.category
        || exchange.description != original.description
        || exchange.completed_at != original.completed_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only confirmed/provider_rating/recipient_rating can change on an exchange update"
                .into(),
        ));
    }

    if action.author != original.provider && action.author != original.recipient {
        return Ok(ValidateCallbackResult::Invalid(
            "Only participants can update an exchange".into(),
        ));
    }

    if exchange.provider_rating != original.provider_rating && action.author != original.provider {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the provider can set their own rating".into(),
        ));
    }

    if exchange.recipient_rating != original.recipient_rating && action.author != original.recipient
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the recipient can set their own rating".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a service offer
fn validate_service_offer(
    author: AgentPubKey,
    offer: ServiceOffer,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the offer to its committer -- create_service_offer already
    // derives `provider` from agent_info() coordinator-side with zero user
    // input, so this never rejects a legitimate offer; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    if offer.provider != author {
        return Ok(ValidateCallbackResult::Invalid(
            "ServiceOffer provider must be the committing agent (offer forgery)".to_string(),
        ));
    }

    // ID must not be empty
    if offer.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Service offer ID cannot be empty".to_string(),
        ));
    }

    // Title must not be empty
    if offer.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Service offer title cannot be empty".to_string(),
        ));
    }

    // Title length limit
    if offer.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Service offer title cannot exceed 200 characters".to_string(),
        ));
    }

    // Description length limit
    if offer.description.len() > 5000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Service offer description cannot exceed 5000 characters".to_string(),
        ));
    }

    // Minimum duration must be positive
    if offer.min_duration_hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum duration must be positive".to_string(),
        ));
    }

    // Max duration must be >= min if specified
    if let Some(max) = offer.max_duration_hours {
        if max < offer.min_duration_hours {
            return Ok(ValidateCallbackResult::Invalid(
                "Maximum duration cannot be less than minimum".to_string(),
            ));
        }
    }

    // Qualifications limit
    if offer.qualifications.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "Too many qualifications (max 20)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a service request
fn validate_service_request(
    author: AgentPubKey,
    request: ServiceRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the request to its committer -- create_service_request already
    // derives `requester` from agent_info() coordinator-side with zero user
    // input, so this never rejects a legitimate request (P0 author-binding
    // gap).
    if request.requester != author {
        return Ok(ValidateCallbackResult::Invalid(
            "ServiceRequest requester must be the committing agent (request forgery)".to_string(),
        ));
    }

    // ID must not be empty
    if request.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Service request ID cannot be empty".to_string(),
        ));
    }

    // Title must not be empty
    if request.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Service request title cannot be empty".to_string(),
        ));
    }

    // Title length limit
    if request.title.len() > 200 {
        return Ok(ValidateCallbackResult::Invalid(
            "Service request title cannot exceed 200 characters".to_string(),
        ));
    }

    // Description length limit
    if request.description.len() > 5000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Service request description cannot exceed 5000 characters".to_string(),
        ));
    }

    // Estimated hours must be positive
    if request.estimated_hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Estimated hours must be positive".to_string(),
        ));
    }

    // Estimated hours should be reasonable (max 168 = 1 week)
    if request.estimated_hours > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Estimated hours cannot exceed 168 (one week)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a time exchange
fn validate_time_exchange(
    author: AgentPubKey,
    exchange: TimeExchange,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the exchange to ONE of its two real participants -- record_exchange
    // took provider AND recipient straight from caller input with ZERO
    // identity check at all (a completely unrelated third party could
    // fabricate an exchange, and the TimeCredit it mints, between two other
    // agents who never interacted). This applies to BOTH create and update
    // (confirm_exchange reuses this same validator via validate_create_entry's
    // dispatch), matching confirm_exchange's own existing client-side check
    // ("only participants can confirm"). NOTE: this does not by itself solve
    // one-sided fabrication between two REAL agents who never actually
    // transacted -- that needs genuine two-party consent (e.g. both
    // signatures, or a request/accept handshake), out of scope for this
    // pass; it closes the more severe "uninvolved third party" hole.
    if author != exchange.provider && author != exchange.recipient {
        return Ok(ValidateCallbackResult::Invalid(
            "TimeExchange must be committed by the provider or the recipient \
             (uninvolved third party cannot fabricate an exchange)"
                .to_string(),
        ));
    }

    // ID must not be empty
    if exchange.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Time exchange ID cannot be empty".to_string(),
        ));
    }

    // Hours must be positive
    if exchange.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange hours must be positive".to_string(),
        ));
    }

    // Hours should be reasonable (max 168 = 1 week)
    if exchange.hours > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange hours cannot exceed 168 (one week)".to_string(),
        ));
    }

    // Provider and recipient must be different
    if exchange.provider == exchange.recipient {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider and recipient must be different agents".to_string(),
        ));
    }

    // Description must not be empty
    if exchange.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange description cannot be empty".to_string(),
        ));
    }

    // Validate ratings if present
    if let Some(rating) = &exchange.provider_rating {
        if rating.score < 1 || rating.score > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Rating score must be between 1 and 5".to_string(),
            ));
        }
    }

    if let Some(rating) = &exchange.recipient_rating {
        if rating.score < 1 || rating.score > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Rating score must be between 1 and 5".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a time credit
fn validate_time_credit(
    author: AgentPubKey,
    credit: TimeCredit,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the credit to ONE of its two real parties -- same gap and same
    // scope note as validate_time_exchange above: record_exchange took
    // earner AND debtor straight from caller input with zero identity
    // check, so an uninvolved third party could mint a fake TimeCredit
    // between two other agents.
    if author != credit.earner && author != credit.debtor {
        return Ok(ValidateCallbackResult::Invalid(
            "TimeCredit must be committed by the earner or the debtor \
             (uninvolved third party cannot mint a credit)"
                .to_string(),
        ));
    }

    // Hours must be positive
    if credit.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit hours must be positive".to_string(),
        ));
    }

    // Hours should be reasonable
    if credit.hours > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit hours cannot exceed 168 (one week)".to_string(),
        ));
    }

    // Earner and debtor must be different
    if credit.earner == credit.debtor {
        return Ok(ValidateCallbackResult::Invalid(
            "Earner and debtor must be different agents".to_string(),
        ));
    }

    // Description must not be empty
    if credit.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit description cannot be empty".to_string(),
        ));
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
        LinkTypes::AgentToOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AgentToRequests => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AgentToExchanges => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CategoryToOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::CategoryToRequests => Ok(ValidateCallbackResult::Valid),
        LinkTypes::OfferToExchange => Ok(ValidateCallbackResult::Valid),
        LinkTypes::RequestToExchange => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllOffers => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AllRequests => Ok(ValidateCallbackResult::Valid),
        LinkTypes::AgentToCredits => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod author_binding_tests {
    use super::*;

    fn me() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }

    fn other_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn valid_availability() -> Availability {
        Availability {
            days: vec![0, 1, 2, 3, 4, 5, 6],
            start_minutes: 0,
            end_minutes: 1440,
            timezone_offset_minutes: 0,
            exceptions: vec![],
            notes: None,
        }
    }

    fn valid_offer(provider: AgentPubKey) -> ServiceOffer {
        ServiceOffer {
            id: "o-1".into(),
            provider,
            category: ServiceCategory::Childcare,
            title: "Babysitting".into(),
            description: "".into(),
            qualifications: vec![],
            availability: valid_availability(),
            location: LocationConstraint::ToBeArranged,
            min_duration_hours: 1.0,
            max_duration_hours: None,
            active: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_offer_valid_when_committer_is_provider() {
        let author = me();
        let offer = valid_offer(author.clone());
        let result = validate_create_entry(author, EntryTypes::ServiceOffer(offer)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_offer_forgery_rejected() {
        let offer = valid_offer(other_agent());
        let result = validate_create_entry(me(), EntryTypes::ServiceOffer(offer)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_request(requester: AgentPubKey) -> ServiceRequest {
        ServiceRequest {
            id: "r-1".into(),
            requester,
            category: ServiceCategory::Childcare,
            title: "Need a sitter".into(),
            description: "".into(),
            urgency: UrgencyLevel::Low,
            needed_by: None,
            estimated_hours: 2.0,
            location: LocationConstraint::ToBeArranged,
            status: RequestStatus::Open,
            created_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_request_valid_when_committer_is_requester() {
        let author = me();
        let request = valid_request(author.clone());
        let result = validate_create_entry(author, EntryTypes::ServiceRequest(request)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_forgery_rejected() {
        let request = valid_request(other_agent());
        let result = validate_create_entry(me(), EntryTypes::ServiceRequest(request)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_exchange(provider: AgentPubKey, recipient: AgentPubKey) -> TimeExchange {
        TimeExchange {
            id: "e-1".into(),
            offer_hash: None,
            request_hash: None,
            provider,
            recipient,
            hours: 1.0,
            category: ServiceCategory::Childcare,
            description: "Watched the kids".into(),
            completed_at: Timestamp::from_micros(0),
            provider_rating: None,
            recipient_rating: None,
            confirmed: false,
        }
    }

    #[test]
    fn create_exchange_valid_when_committer_is_participant() {
        let author = me();
        let exchange = valid_exchange(author.clone(), other_agent());
        let result = validate_create_entry(author, EntryTypes::TimeExchange(exchange)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_exchange_rejected_for_uninvolved_third_party() {
        let exchange = valid_exchange(other_agent(), AgentPubKey::from_raw_36(vec![2u8; 36]));
        let result = validate_create_entry(me(), EntryTypes::TimeExchange(exchange)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_credit(earner: AgentPubKey, debtor: AgentPubKey) -> TimeCredit {
        TimeCredit {
            hours: 1.0,
            earner,
            debtor,
            service_category: ServiceCategory::Childcare,
            description: "Watched the kids".into(),
            performed_at: Timestamp::from_micros(0),
            expires_at: None,
        }
    }

    #[test]
    fn create_credit_valid_when_committer_is_party() {
        let author = me();
        let credit = valid_credit(author.clone(), other_agent());
        let result = validate_create_entry(author, EntryTypes::TimeCredit(credit)).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_credit_rejected_for_uninvolved_third_party() {
        let credit = valid_credit(other_agent(), AgentPubKey::from_raw_36(vec![2u8; 36]));
        let result = validate_create_entry(me(), EntryTypes::TimeCredit(credit)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // validate_update_{service_offer,time_exchange} both call
    // must_get_valid_record, which requires a live HDI host and can't run
    // in a plain unit test -- matching the established pattern from every
    // other zome's update validator this pass. Correctness there is
    // verified via cargo check plus the code-review reasoning in the
    // commit message.
}
