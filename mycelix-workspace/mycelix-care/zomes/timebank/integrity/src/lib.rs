// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Timebank Integrity Zome
//! Defines entry types and validation for service offers, requests, exchanges, and time credits.

use hdi::prelude::*;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Categories of care services available in the timebank
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ServiceCategory {
    Childcare,
    Eldercare,
    PetCare,
    Cooking,
    Cleaning,
    Gardening,
    Tutoring,
    TechSupport,
    Transportation,
    Companionship,
    HealthSupport,
    HomeRepair,
    LegalAdvice,
    Counseling,
    ArtMusic,
    LanguageHelp,
    Administrative,
    Other(String),
}

impl ServiceCategory {
    /// Return a canonical string key for anchor-based discovery
    pub fn anchor_key(&self) -> String {
        match self {
            ServiceCategory::Childcare => "childcare".to_string(),
            ServiceCategory::Eldercare => "eldercare".to_string(),
            ServiceCategory::PetCare => "petcare".to_string(),
            ServiceCategory::Cooking => "cooking".to_string(),
            ServiceCategory::Cleaning => "cleaning".to_string(),
            ServiceCategory::Gardening => "gardening".to_string(),
            ServiceCategory::Tutoring => "tutoring".to_string(),
            ServiceCategory::TechSupport => "techsupport".to_string(),
            ServiceCategory::Transportation => "transportation".to_string(),
            ServiceCategory::Companionship => "companionship".to_string(),
            ServiceCategory::HealthSupport => "healthsupport".to_string(),
            ServiceCategory::HomeRepair => "homerepair".to_string(),
            ServiceCategory::LegalAdvice => "legaladvice".to_string(),
            ServiceCategory::Counseling => "counseling".to_string(),
            ServiceCategory::ArtMusic => "artmusic".to_string(),
            ServiceCategory::LanguageHelp => "languagehelp".to_string(),
            ServiceCategory::Administrative => "administrative".to_string(),
            ServiceCategory::Other(s) => format!("other_{}", s.to_lowercase().replace(' ', "_")),
        }
    }
}

/// Urgency level for service requests
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum UrgencyLevel {
    Low,
    Medium,
    High,
    Critical,
}

/// A service offer posted by a provider
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ServiceOffer {
    /// The agent offering the service
    pub provider: AgentPubKey,
    /// Category of service
    pub category: ServiceCategory,
    /// Short title for the offer
    pub title: String,
    /// Detailed description of what is offered
    pub description: String,
    /// Maximum hours available per week
    pub hours_available: f32,
    /// Availability description (e.g. "weekday mornings", "flexible")
    pub availability: String,
    /// Location or area served
    pub location: String,
    /// Skills or qualifications relevant to this offer
    pub skills_required: Vec<String>,
    /// Whether this offer is currently active
    pub active: bool,
    /// When the offer was created
    pub created_at: Timestamp,
    /// When the offer was last updated
    pub updated_at: Timestamp,
}

/// A service request posted by someone needing help
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ServiceRequest {
    /// The agent requesting help
    pub requester: AgentPubKey,
    /// Category of service needed
    pub category: ServiceCategory,
    /// Short title for the request
    pub title: String,
    /// Detailed description of what is needed
    pub description: String,
    /// Estimated hours needed
    pub hours_needed: f32,
    /// Preferred schedule
    pub preferred_schedule: String,
    /// Location where service is needed
    pub location: String,
    /// How urgent the request is
    pub urgency: UrgencyLevel,
    /// Whether this request is still open
    pub open: bool,
    /// When the request was created
    pub created_at: Timestamp,
    /// When the request was last updated
    pub updated_at: Timestamp,
}

/// A completed exchange of service between two agents
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TimeExchange {
    /// The service offer that was fulfilled
    pub offer_id: ActionHash,
    /// The service request that was fulfilled
    pub request_id: ActionHash,
    /// The provider who gave the service
    pub provider: AgentPubKey,
    /// The recipient who received the service
    pub recipient: AgentPubKey,
    /// Number of hours exchanged
    pub hours: f32,
    /// Category of service exchanged
    pub category: ServiceCategory,
    /// When the exchange was completed
    pub completed_at: Timestamp,
    /// Provider's rating of the recipient (1-5)
    pub rating_provider: Option<u8>,
    /// Recipient's rating of the provider (1-5)
    pub rating_recipient: Option<u8>,
    /// Optional notes about the exchange
    pub notes: String,
}

/// Time credit balance for an agent
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TimeCredit {
    /// The agent this credit belongs to
    pub agent: AgentPubKey,
    /// Current balance in hours
    pub balance: f64,
    /// Total hours earned through providing services
    pub total_earned: f64,
    /// Total hours spent receiving services
    pub total_spent: f64,
    /// Last updated timestamp
    pub updated_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    ServiceOffer(ServiceOffer),
    ServiceRequest(ServiceRequest),
    TimeExchange(TimeExchange),
    TimeCredit(TimeCredit),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Agent to their service offers
    AgentToOffer,
    /// Agent to their service requests
    AgentToRequest,
    /// Category anchor to offers in that category
    CategoryToOffer,
    /// Category anchor to requests in that category
    CategoryToRequest,
    /// All active offers anchor
    AllActiveOffers,
    /// All open requests anchor
    AllOpenRequests,
    /// Agent to their completed exchanges
    AgentToExchange,
    /// Agent to their time credit record
    AgentToCredit,
    /// Offer to exchanges that fulfilled it
    OfferToExchange,
    /// Request to exchanges that fulfilled it
    RequestToExchange,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// HDI 0.7 single validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ServiceOffer(offer) => validate_create_offer(action, offer),
                EntryTypes::ServiceRequest(request) => validate_create_request(action, request),
                EntryTypes::TimeExchange(exchange) => validate_create_exchange(action, exchange),
                EntryTypes::TimeCredit(credit) => validate_create_credit(action, credit),
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
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => match link_type {
            LinkTypes::AgentToOffer => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToRequest => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CategoryToOffer => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CategoryToRequest => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllActiveOffers => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AllOpenRequests => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToExchange => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AgentToCredit => Ok(ValidateCallbackResult::Valid),
            LinkTypes::OfferToExchange => Ok(ValidateCallbackResult::Valid),
            LinkTypes::RequestToExchange => Ok(ValidateCallbackResult::Valid),
        },
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
            // unconditionally) -- the 8th confirmed instance of this exact
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
        EntryTypes::ServiceOffer(_) => Ok(ValidateCallbackResult::Invalid(
            "Service offers are immutable (re-post to change details)".into(),
        )),
        EntryTypes::ServiceRequest(_) => Ok(ValidateCallbackResult::Invalid(
            "Service requests are immutable (re-post to change details)".into(),
        )),
        EntryTypes::TimeExchange(exchange) => validate_update_exchange(action, exchange),
        EntryTypes::TimeCredit(credit) => validate_update_credit(action, credit),
    }
}

fn validate_create_offer(
    action: Create,
    offer: ServiceOffer,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's create_service_offer now derives
    // `provider` from agent_info() rather than trusting caller input, so
    // this is belt-and-suspenders against a modified coordinator forging
    // a victim agent as provider. Found + fixed 2026-07-09 during the P0
    // author-binding pass.
    if offer.provider != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer provider must correspond to the committing agent".into(),
        ));
    }

    if offer.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title cannot be empty".into(),
        ));
    }
    if offer.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title must be 256 characters or fewer".into(),
        ));
    }
    if offer.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer description cannot be empty".into(),
        ));
    }
    if offer.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer description must be 4096 characters or fewer".into(),
        ));
    }
    if offer.hours_available <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours available must be positive".into(),
        ));
    }
    if offer.hours_available > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours available cannot exceed 168 per week".into(),
        ));
    }
    if offer.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Location cannot be empty".into(),
        ));
    }
    if offer.location.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Location must be 512 characters or fewer".into(),
        ));
    }
    if offer.availability.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Availability must be 512 characters or fewer".into(),
        ));
    }
    if offer.skills_required.len() > 20 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot list more than 20 skills".into(),
        ));
    }
    for skill in &offer.skills_required {
        if skill.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each skill must be 128 characters or fewer".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_request(
    action: Create,
    request: ServiceRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's create_service_request now derives
    // `requester` from agent_info() rather than trusting caller input, so
    // this is belt-and-suspenders against a modified coordinator forging
    // a victim agent as requester. Found + fixed 2026-07-09 during the P0
    // author-binding pass.
    if request.requester != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Request requester must correspond to the committing agent".into(),
        ));
    }

    if request.title.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title cannot be empty".into(),
        ));
    }
    if request.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title must be 256 characters or fewer".into(),
        ));
    }
    if request.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request description cannot be empty".into(),
        ));
    }
    if request.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Request description must be 4096 characters or fewer".into(),
        ));
    }
    if request.hours_needed <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours needed must be positive".into(),
        ));
    }
    if request.hours_needed > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours needed cannot exceed 168".into(),
        ));
    }
    if request.location.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Location cannot be empty".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_exchange(
    action: Create,
    exchange: TimeExchange,
) -> ExternResult<ValidateCallbackResult> {
    // Author-binding: the coordinator's complete_exchange now requires the
    // caller be the provider or recipient before creating this entry, but
    // that's bypassable by a modified coordinator -- the integrity
    // validator is the real security boundary. Without this, ANY agent
    // could record an exchange between two arbitrary victims and directly
    // manipulate BOTH of their TimeCredit balances (a financial-forgery
    // vector, not just an identity one). Found + fixed 2026-07-09 during
    // the P0 author-binding pass.
    //
    // KNOWN LIMITATION, not fully fixed here: this only requires the
    // caller be ONE of the two parties, not both -- see the matching note
    // on complete_exchange in the coordinator. True mutual confirmation
    // would need a Holochain countersigning session.
    if exchange.provider != action.author && exchange.recipient != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange must be committed by the provider or recipient".into(),
        ));
    }

    if exchange.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange hours must be positive".into(),
        ));
    }
    if exchange.hours > 168.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange hours cannot exceed 168".into(),
        ));
    }
    if exchange.provider == exchange.recipient {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider and recipient cannot be the same agent".into(),
        ));
    }
    // Ratings are only ever added via a later update (rate_exchange); a
    // freshly created exchange cannot already claim one.
    if exchange.rating_provider.is_some() || exchange.rating_recipient.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New exchanges cannot have ratings set".into(),
        ));
    }
    if let Some(rating) = exchange.rating_provider {
        if rating < 1 || rating > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Provider rating must be 1-5".into(),
            ));
        }
    }
    if let Some(rating) = exchange.rating_recipient {
        if rating < 1 || rating > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Recipient rating must be 1-5".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

/// No author-binding: see the rationale on validate_update_credit above --
/// `agent` is routinely credited by the OTHER exchange participant via
/// complete_exchange. get_or_create_credit also self-initializes a
/// zero-balance record for the caller's own DID with no issue. Reviewed
/// 2026-07-09 during the P0 author-binding pass.
fn validate_create_credit(
    _action: Create,
    credit: TimeCredit,
) -> ExternResult<ValidateCallbackResult> {
    if credit.total_earned < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total earned cannot be negative".into(),
        ));
    }
    if credit.total_spent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total spent cannot be negative".into(),
        ));
    }
    // The coordinator's get_or_create_credit only ever creates a fresh
    // record with a one-time 5.0-hour starter bonus and zero history --
    // check for exactly that shape rather than the general
    // balance == total_earned - total_spent invariant (which the starter
    // bonus deliberately breaks; that invariant is instead checked as
    // "stays constant across updates" in validate_update_credit below).
    if credit.total_earned != 0.0 || credit.total_spent != 0.0 || credit.balance != 5.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "New credit records must start at the 5.0-hour starter balance with zero history"
                .into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a TimeExchange update (the coordinator's rate_exchange lets
/// the provider set rating_provider or the recipient set rating_recipient).
///
/// Previously had no author check and never fetched the original -- any
/// agent could rewrite provider/recipient/hours/category under the guise
/// of "rating". Hardened 2026-07-09 during the P0 author-binding pass:
/// the committing agent must be the provider or recipient, and only the
/// rating field belonging to that role may change.
fn validate_update_exchange(
    action: Update,
    exchange: TimeExchange,
) -> ExternResult<ValidateCallbackResult> {
    if let Some(rating) = exchange.rating_provider {
        if rating < 1 || rating > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Provider rating must be 1-5".into(),
            ));
        }
    }
    if let Some(rating) = exchange.rating_recipient {
        if rating < 1 || rating > 5 {
            return Ok(ValidateCallbackResult::Invalid(
                "Recipient rating must be 1-5".into(),
            ));
        }
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: TimeExchange = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original exchange not found".into()
        )))?;

    if exchange.offer_id != original.offer_id
        || exchange.request_id != original.request_id
        || exchange.provider != original.provider
        || exchange.recipient != original.recipient
        || exchange.hours != original.hours
        || exchange.category != original.category
        || exchange.completed_at != original.completed_at
        || exchange.notes != original.notes
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only rating_provider/rating_recipient can change on an exchange update".into(),
        ));
    }

    if action.author == exchange.provider {
        if exchange.rating_recipient != original.rating_recipient {
            return Ok(ValidateCallbackResult::Invalid(
                "Only the provider's own rating can be set by the provider".into(),
            ));
        }
    } else if action.author == exchange.recipient {
        if exchange.rating_provider != original.rating_provider {
            return Ok(ValidateCallbackResult::Invalid(
                "Only the recipient's own rating can be set by the recipient".into(),
            ));
        }
    } else {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the provider or recipient can rate an exchange".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate a TimeCredit update (the coordinator's update_agent_credit,
/// called from complete_exchange, credits the provider and debits the
/// recipient of an exchange).
///
/// No author requirement: TimeCredit legitimately gets updated by
/// whichever of the two exchange participants called complete_exchange
/// (now restricted to the provider or recipient at the coordinator/
/// integrity level for TimeExchange creation, see validate_create_exchange
/// below) -- meaning the committing agent is routinely the OTHER party to
/// this specific credit record, not its own `agent`. This mirrors the
/// report_reputation-class gap elsewhere in this pass, but is narrower and
/// already mitigated by the exchange-participant check. Content integrity
/// is enforced instead: `agent` is immutable, updated_at must advance, and
/// balance must stay arithmetically consistent with total_earned -
/// total_spent.
fn validate_update_credit(
    action: Update,
    credit: TimeCredit,
) -> ExternResult<ValidateCallbackResult> {
    if credit.total_earned < 0.0 || credit.total_spent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total earned/spent cannot be negative".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original: TimeCredit = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original credit record not found".into()
        )))?;

    if credit.agent != original.agent {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit agent cannot be changed".into(),
        ));
    }
    if credit.updated_at <= original.updated_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit updated_at must advance".into(),
        ));
    }
    if credit.total_earned < original.total_earned || credit.total_spent < original.total_spent {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit total_earned/total_spent cannot decrease".into(),
        ));
    }
    // balance - total_earned + total_spent is a per-agent constant offset
    // (their one-time starter bonus) -- it must never drift across
    // updates, even though the coordinator's update_agent_credit already
    // maintains this by construction (balance and the relevant total move
    // by exactly the same `hours` together).
    let original_offset = original.balance - original.total_earned + original.total_spent;
    let new_offset = credit.balance - credit.total_earned + credit.total_spent;
    if (new_offset - original_offset).abs() > f64::EPSILON {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit balance must move in lockstep with total_earned/total_spent".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
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

    fn valid_offer(provider: AgentPubKey) -> ServiceOffer {
        ServiceOffer {
            provider,
            category: ServiceCategory::Tutoring,
            title: "Math tutoring".into(),
            description: "Algebra help".into(),
            hours_available: 5.0,
            availability: "weekends".into(),
            location: "Downtown".into(),
            skills_required: vec![],
            active: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_offer_valid_when_provider_matches_committer() {
        let o = valid_offer(me());
        let result = validate_create_offer(create_action(me()), o).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_offer_forgery_rejected() {
        let o = valid_offer(me());
        let result = validate_create_offer(create_action(other_agent()), o).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_request(requester: AgentPubKey) -> ServiceRequest {
        ServiceRequest {
            requester,
            category: ServiceCategory::Tutoring,
            title: "Need math help".into(),
            description: "Algebra homework".into(),
            hours_needed: 3.0,
            preferred_schedule: "evenings".into(),
            location: "Downtown".into(),
            urgency: UrgencyLevel::Medium,
            open: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    #[test]
    fn create_request_valid_when_requester_matches_committer() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(me()), r).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_request_forgery_rejected() {
        let r = valid_request(me());
        let result = validate_create_request(create_action(other_agent()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    fn valid_exchange(provider: AgentPubKey, recipient: AgentPubKey) -> TimeExchange {
        TimeExchange {
            offer_id: ActionHash::from_raw_36(vec![0u8; 36]),
            request_id: ActionHash::from_raw_36(vec![0u8; 36]),
            provider,
            recipient,
            hours: 2.0,
            category: ServiceCategory::Tutoring,
            completed_at: Timestamp::from_micros(0),
            rating_provider: None,
            rating_recipient: None,
            notes: String::new(),
        }
    }

    #[test]
    fn create_exchange_valid_when_committer_is_provider() {
        let e = valid_exchange(me(), other_agent());
        let result = validate_create_exchange(create_action(me()), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_exchange_valid_when_committer_is_recipient() {
        let e = valid_exchange(other_agent(), me());
        let result = validate_create_exchange(create_action(me()), e).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_exchange_rejected_for_uninvolved_agent() {
        let third_party = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let e = valid_exchange(other_agent(), third_party);
        let result = validate_create_exchange(create_action(me()), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_exchange_rejects_preset_ratings() {
        let mut e = valid_exchange(me(), other_agent());
        e.rating_provider = Some(5);
        let result = validate_create_exchange(create_action(me()), e).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn create_credit_valid_at_starter_balance() {
        let credit = TimeCredit {
            agent: me(),
            balance: 5.0,
            total_earned: 0.0,
            total_spent: 0.0,
            updated_at: Timestamp::from_micros(0),
        };
        let result = validate_create_credit(create_action(me()), credit).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_credit_rejects_nonstarter_shape() {
        let credit = TimeCredit {
            agent: me(),
            balance: 1000.0,
            total_earned: 995.0,
            total_spent: 0.0,
            updated_at: Timestamp::from_micros(0),
        };
        let result = validate_create_credit(create_action(me()), credit).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn update_entry_type_rejects_offer_update() {
        let o = valid_offer(me());
        let action = Update {
            author: me(),
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
        };
        let result = validate_update_entry_type(action, EntryTypes::ServiceOffer(o)).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
