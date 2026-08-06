// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Timebank Integrity Zome
//! Defines entry types and validation for service offers, requests, exchanges, and time credits.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match};

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
                original_action_hash,
                original_entry_hash: _,
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ServiceOffer(_) => Ok(ValidateCallbackResult::Invalid(
                    "Service offers are immutable (re-post to change details)".into(),
                )),
                EntryTypes::ServiceRequest(_) => Ok(ValidateCallbackResult::Invalid(
                    "Service requests are immutable (re-post to change details)".into(),
                )),
                EntryTypes::TimeExchange(exchange) => {
                    validate_update_exchange(action, exchange, original_action_hash)
                }
                EntryTypes::TimeCredit(credit) => {
                    validate_update_credit(action, credit, original_action_hash)
                }
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag,
            action: _,
        } => match link_type {
            LinkTypes::AgentToOffer => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToOffer link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AgentToRequest => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToRequest link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::CategoryToOffer => {
                if tag.0.len() > 512 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CategoryToOffer link tag too long (max 512 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::CategoryToRequest => {
                if tag.0.len() > 512 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "CategoryToRequest link tag too long (max 512 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AllActiveOffers => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllActiveOffers link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AllOpenRequests => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllOpenRequests link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AgentToExchange => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToExchange link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::AgentToCredit => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToCredit link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::OfferToExchange => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "OfferToExchange link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::RequestToExchange => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "RequestToExchange link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
        },
        FlatOp::RegisterDeleteLink { action, .. } => {
            let original_action = must_get_action(action.link_add_address.clone())?;
            Ok(check_link_author_match(
                original_action.action().author(),
                &action.author,
            ))
        }
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action, .. }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

fn validate_service_category_other(category: &ServiceCategory) -> Result<(), String> {
    if let ServiceCategory::Other(s) = category {
        if s.trim().is_empty() {
            return Err("Custom service category label cannot be empty".to_string());
        }
        if s.len() > 128 {
            return Err(
                "Custom service category label must be 128 characters or fewer".to_string(),
            );
        }
    }
    Ok(())
}

fn validate_create_offer(
    action: Create,
    offer: ServiceOffer,
) -> ExternResult<ValidateCallbackResult> {
    if offer.provider != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Service offer provider must match the committing agent".into(),
        ));
    }
    if offer.title.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title cannot be empty".into(),
        ));
    }
    if offer.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer title must be 256 characters or fewer".into(),
        ));
    }
    if offer.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer description cannot be empty".into(),
        ));
    }
    if offer.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Offer description must be 4096 characters or fewer".into(),
        ));
    }
    if !offer.hours_available.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours available must be a finite number".into(),
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
    if offer.location.trim().is_empty() {
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
        if skill.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Skill name cannot be empty".into(),
            ));
        }
        if skill.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Each skill must be 128 characters or fewer".into(),
            ));
        }
    }
    if let Err(msg) = validate_service_category_other(&offer.category) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_request(
    action: Create,
    request: ServiceRequest,
) -> ExternResult<ValidateCallbackResult> {
    if request.requester != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Service request requester must match the committing agent".into(),
        ));
    }
    if request.title.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title cannot be empty".into(),
        ));
    }
    if request.title.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Request title must be 256 characters or fewer".into(),
        ));
    }
    if request.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Request description cannot be empty".into(),
        ));
    }
    if request.description.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Request description must be 4096 characters or fewer".into(),
        ));
    }
    if !request.hours_needed.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours needed must be a finite number".into(),
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
    if request.location.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Location cannot be empty".into(),
        ));
    }
    if request.location.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Location must be 512 characters or fewer".into(),
        ));
    }
    if request.preferred_schedule.len() > 512 {
        return Ok(ValidateCallbackResult::Invalid(
            "Preferred schedule must be 512 characters or fewer".into(),
        ));
    }
    if let Err(msg) = validate_service_category_other(&request.category) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_exchange(
    action: Create,
    exchange: TimeExchange,
) -> ExternResult<ValidateCallbackResult> {
    if exchange.provider != action.author && exchange.recipient != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange must be committed by the provider or recipient".into(),
        ));
    }
    if exchange.rating_provider.is_some() || exchange.rating_recipient.is_some() {
        return Ok(ValidateCallbackResult::Invalid(
            "New exchanges cannot have ratings set".into(),
        ));
    }
    if !exchange.hours.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours must be a finite number".into(),
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
    if let Some(rating) = exchange.rating_provider {
        if !(1..=5).contains(&rating) {
            return Ok(ValidateCallbackResult::Invalid(
                "Provider rating must be 1-5".into(),
            ));
        }
    }
    if let Some(rating) = exchange.rating_recipient {
        if !(1..=5).contains(&rating) {
            return Ok(ValidateCallbackResult::Invalid(
                "Recipient rating must be 1-5".into(),
            ));
        }
    }
    if exchange.notes.len() > 4096 {
        return Ok(ValidateCallbackResult::Invalid(
            "Notes must be 4096 characters or fewer".into(),
        ));
    }
    if let Err(msg) = validate_service_category_other(&exchange.category) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_credit(
    _action: Create,
    credit: TimeCredit,
) -> ExternResult<ValidateCallbackResult> {
    if !credit.balance.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Balance must be a finite number".into(),
        ));
    }
    if !credit.total_earned.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Total earned must be a finite number".into(),
        ));
    }
    if credit.total_earned < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total earned cannot be negative".into(),
        ));
    }
    if !credit.total_spent.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Total spent must be a finite number".into(),
        ));
    }
    if credit.total_spent < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Total spent cannot be negative".into(),
        ));
    }
    // The coordinator's get_or_create_credit only ever creates a fresh
    // record with a one-time 5.0-hour starter bonus and zero history.
    if credit.total_earned != 0.0 || credit.total_spent != 0.0 || credit.balance != 5.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "New credit records must start at the 5.0-hour starter balance with zero history"
                .into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Pure logic for a `TimeExchange` update, factored out of
/// [`validate_update_exchange`] so it can be unit-tested without a live HDI
/// (`must_get_valid_record` has no test mock in this codebase). The
/// coordinator's `rate_exchange` lets the provider set `rating_provider` or
/// the recipient set `rating_recipient` -- nothing else may change.
fn validate_exchange_transition(
    original: &TimeExchange,
    exchange: &TimeExchange,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    if let Some(rating) = exchange.rating_provider
        && !(1..=5).contains(&rating)
    {
        return ValidateCallbackResult::Invalid("Provider rating must be 1-5".into());
    }
    if let Some(rating) = exchange.rating_recipient
        && !(1..=5).contains(&rating)
    {
        return ValidateCallbackResult::Invalid("Recipient rating must be 1-5".into());
    }
    if exchange.offer_id != original.offer_id
        || exchange.request_id != original.request_id
        || exchange.provider != original.provider
        || exchange.recipient != original.recipient
        || exchange.hours != original.hours
        || exchange.category != original.category
        || exchange.completed_at != original.completed_at
        || exchange.notes != original.notes
    {
        return ValidateCallbackResult::Invalid(
            "Only rating_provider/rating_recipient can change on an exchange update".into(),
        );
    }
    if *author == exchange.provider {
        if exchange.rating_recipient != original.rating_recipient {
            return ValidateCallbackResult::Invalid(
                "Only the provider's own rating can be set by the provider".into(),
            );
        }
    } else if *author == exchange.recipient {
        if exchange.rating_provider != original.rating_provider {
            return ValidateCallbackResult::Invalid(
                "Only the recipient's own rating can be set by the recipient".into(),
            );
        }
    } else {
        return ValidateCallbackResult::Invalid(
            "Only the provider or recipient can rate an exchange".into(),
        );
    }
    ValidateCallbackResult::Valid
}

fn validate_update_exchange(
    action: Update,
    exchange: TimeExchange,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: TimeExchange = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original TimeExchange entry not found".to_string()
        )))?;
    Ok(validate_exchange_transition(
        &original,
        &exchange,
        &action.author,
    ))
}

/// Pure logic for a `TimeCredit` update, factored out of
/// [`validate_update_credit`] so it can be unit-tested without a live HDI.
/// No author requirement: TimeCredit legitimately gets updated by whichever
/// of the two exchange participants called `complete_exchange` (already
/// restricted to the provider or recipient at `validate_create_exchange`).
/// Content integrity is enforced instead.
fn validate_credit_transition(
    original: &TimeCredit,
    credit: &TimeCredit,
) -> ValidateCallbackResult {
    if credit.total_earned < 0.0 || credit.total_spent < 0.0 {
        return ValidateCallbackResult::Invalid("Total earned/spent cannot be negative".into());
    }
    if credit.agent != original.agent {
        return ValidateCallbackResult::Invalid("Credit agent cannot be changed".into());
    }
    if credit.updated_at <= original.updated_at {
        return ValidateCallbackResult::Invalid("Credit updated_at must advance".into());
    }
    if credit.total_earned < original.total_earned || credit.total_spent < original.total_spent {
        return ValidateCallbackResult::Invalid(
            "Credit total_earned/total_spent cannot decrease".into(),
        );
    }
    let original_offset = original.balance - original.total_earned + original.total_spent;
    let new_offset = credit.balance - credit.total_earned + credit.total_spent;
    if (new_offset - original_offset).abs() > f64::EPSILON {
        return ValidateCallbackResult::Invalid(
            "Credit balance must move in lockstep with total_earned/total_spent".into(),
        );
    }
    ValidateCallbackResult::Valid
}

fn validate_update_credit(
    _action: Update,
    credit: TimeCredit,
    original_action_hash: ActionHash,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: TimeCredit = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original TimeCredit entry not found".to_string()
        )))?;
    Ok(validate_credit_transition(&original, &credit))
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────

    fn fake_create() -> Create {
        Create {
            author: agent_a(),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex(0),
                ZomeIndex(0),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: EntryRateWeight::default(),
        }
    }

    fn agent_a() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![1u8; 36])
    }

    fn agent_b() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![2u8; 36])
    }

    fn valid_offer() -> ServiceOffer {
        ServiceOffer {
            provider: agent_a(),
            category: ServiceCategory::Childcare,
            title: "Babysitting".to_string(),
            description: "I can watch your kids.".to_string(),
            hours_available: 10.0,
            availability: "Weekday mornings".to_string(),
            location: "Downtown".to_string(),
            skills_required: vec!["CPR certified".to_string()],
            active: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    fn valid_request() -> ServiceRequest {
        ServiceRequest {
            requester: agent_a(),
            category: ServiceCategory::Eldercare,
            title: "Need help with grandma".to_string(),
            description: "Looking for afternoon companionship.".to_string(),
            hours_needed: 4.0,
            preferred_schedule: "Afternoons".to_string(),
            location: "East side".to_string(),
            urgency: UrgencyLevel::Medium,
            open: true,
            created_at: Timestamp::from_micros(0),
            updated_at: Timestamp::from_micros(0),
        }
    }

    fn valid_exchange() -> TimeExchange {
        // No ratings by default: this represents both the create-time shape
        // (ratings must be unset on create) and the "original" baseline for
        // update-transition tests (rate_exchange sets exactly one rating).
        TimeExchange {
            offer_id: ActionHash::from_raw_36(vec![10u8; 36]),
            request_id: ActionHash::from_raw_36(vec![11u8; 36]),
            provider: agent_a(),
            recipient: agent_b(),
            hours: 2.0,
            category: ServiceCategory::Tutoring,
            completed_at: Timestamp::from_micros(1000),
            rating_provider: None,
            rating_recipient: None,
            notes: "Great session".to_string(),
        }
    }

    fn valid_credit() -> TimeCredit {
        TimeCredit {
            agent: agent_a(),
            balance: 10.0,
            total_earned: 12.0,
            total_spent: 2.0,
            updated_at: Timestamp::from_micros(0),
        }
    }

    fn assert_valid(result: ExternResult<ValidateCallbackResult>) {
        match result {
            Ok(ValidateCallbackResult::Valid) => {}
            Ok(ValidateCallbackResult::Invalid(msg)) => {
                panic!("Expected Valid, got Invalid: {msg}")
            }
            other => panic!("Expected Valid, got {other:?}"),
        }
    }

    fn assert_invalid(result: ExternResult<ValidateCallbackResult>, expected_substr: &str) {
        match result {
            Ok(ValidateCallbackResult::Invalid(msg)) => {
                assert!(
                    msg.contains(expected_substr),
                    "Expected Invalid message containing '{expected_substr}', got: '{msg}'"
                );
            }
            Ok(ValidateCallbackResult::Valid) => {
                panic!("Expected Invalid containing '{expected_substr}', got Valid")
            }
            other => panic!("Expected Invalid, got {other:?}"),
        }
    }

    // ── Serde roundtrip tests ───────────────────────────────────────────

    #[test]
    fn serde_roundtrip_service_category() {
        let cats = vec![
            ServiceCategory::Childcare,
            ServiceCategory::Eldercare,
            ServiceCategory::PetCare,
            ServiceCategory::Cooking,
            ServiceCategory::Cleaning,
            ServiceCategory::Gardening,
            ServiceCategory::Tutoring,
            ServiceCategory::TechSupport,
            ServiceCategory::Transportation,
            ServiceCategory::Companionship,
            ServiceCategory::HealthSupport,
            ServiceCategory::HomeRepair,
            ServiceCategory::LegalAdvice,
            ServiceCategory::Counseling,
            ServiceCategory::ArtMusic,
            ServiceCategory::LanguageHelp,
            ServiceCategory::Administrative,
            ServiceCategory::Other("Custom Thing".to_string()),
        ];
        for cat in &cats {
            let json = serde_json::to_string(cat).unwrap();
            let back: ServiceCategory = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, cat);
        }
    }

    #[test]
    fn serde_roundtrip_urgency_level() {
        let levels = vec![
            UrgencyLevel::Low,
            UrgencyLevel::Medium,
            UrgencyLevel::High,
            UrgencyLevel::Critical,
        ];
        for level in &levels {
            let json = serde_json::to_string(level).unwrap();
            let back: UrgencyLevel = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, level);
        }
    }

    #[test]
    fn serde_roundtrip_service_offer() {
        let offer = valid_offer();
        let json = serde_json::to_string(&offer).unwrap();
        let back: ServiceOffer = serde_json::from_str(&json).unwrap();
        assert_eq!(back, offer);
    }

    #[test]
    fn serde_roundtrip_service_request() {
        let req = valid_request();
        let json = serde_json::to_string(&req).unwrap();
        let back: ServiceRequest = serde_json::from_str(&json).unwrap();
        assert_eq!(back, req);
    }

    #[test]
    fn serde_roundtrip_time_exchange() {
        let ex = valid_exchange();
        let json = serde_json::to_string(&ex).unwrap();
        let back: TimeExchange = serde_json::from_str(&json).unwrap();
        assert_eq!(back, ex);
    }

    #[test]
    fn serde_roundtrip_time_credit() {
        let credit = valid_credit();
        let json = serde_json::to_string(&credit).unwrap();
        let back: TimeCredit = serde_json::from_str(&json).unwrap();
        assert_eq!(back, credit);
    }

    // ── ServiceCategory::anchor_key tests ───────────────────────────────

    #[test]
    fn anchor_key_known_categories() {
        assert_eq!(ServiceCategory::Childcare.anchor_key(), "childcare");
        assert_eq!(ServiceCategory::Eldercare.anchor_key(), "eldercare");
        assert_eq!(ServiceCategory::PetCare.anchor_key(), "petcare");
        assert_eq!(ServiceCategory::Cooking.anchor_key(), "cooking");
        assert_eq!(ServiceCategory::Cleaning.anchor_key(), "cleaning");
        assert_eq!(ServiceCategory::Gardening.anchor_key(), "gardening");
        assert_eq!(ServiceCategory::Tutoring.anchor_key(), "tutoring");
        assert_eq!(ServiceCategory::TechSupport.anchor_key(), "techsupport");
        assert_eq!(
            ServiceCategory::Transportation.anchor_key(),
            "transportation"
        );
        assert_eq!(ServiceCategory::Companionship.anchor_key(), "companionship");
        assert_eq!(ServiceCategory::HealthSupport.anchor_key(), "healthsupport");
        assert_eq!(ServiceCategory::HomeRepair.anchor_key(), "homerepair");
        assert_eq!(ServiceCategory::LegalAdvice.anchor_key(), "legaladvice");
        assert_eq!(ServiceCategory::Counseling.anchor_key(), "counseling");
        assert_eq!(ServiceCategory::ArtMusic.anchor_key(), "artmusic");
        assert_eq!(ServiceCategory::LanguageHelp.anchor_key(), "languagehelp");
        assert_eq!(
            ServiceCategory::Administrative.anchor_key(),
            "administrative"
        );
    }

    #[test]
    fn anchor_key_other_lowercases_and_replaces_spaces() {
        let cat = ServiceCategory::Other("Dog Walking Service".to_string());
        assert_eq!(cat.anchor_key(), "other_dog_walking_service");
    }

    #[test]
    fn anchor_key_other_empty_string() {
        let cat = ServiceCategory::Other(String::new());
        assert_eq!(cat.anchor_key(), "other_");
    }

    // ── validate_create_offer tests ─────────────────────────────────────

    #[test]
    fn create_offer_valid() {
        assert_valid(validate_create_offer(fake_create(), valid_offer()));
    }

    #[test]
    fn create_offer_empty_title() {
        let mut o = valid_offer();
        o.title = String::new();
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Offer title cannot be empty",
        );
    }

    #[test]
    fn create_offer_title_too_long() {
        let mut o = valid_offer();
        o.title = "a".repeat(257);
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Offer title must be 256 characters or fewer",
        );
    }

    #[test]
    fn create_offer_title_at_boundary() {
        let mut o = valid_offer();
        o.title = "a".repeat(256);
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_empty_description() {
        let mut o = valid_offer();
        o.description = String::new();
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Offer description cannot be empty",
        );
    }

    #[test]
    fn create_offer_description_too_long() {
        let mut o = valid_offer();
        o.description = "b".repeat(4097);
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Offer description must be 4096 characters or fewer",
        );
    }

    #[test]
    fn create_offer_description_at_boundary() {
        let mut o = valid_offer();
        o.description = "b".repeat(4096);
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_zero_hours() {
        let mut o = valid_offer();
        o.hours_available = 0.0;
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Hours available must be positive",
        );
    }

    #[test]
    fn create_offer_negative_hours() {
        let mut o = valid_offer();
        o.hours_available = -1.0;
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Hours available must be positive",
        );
    }

    #[test]
    fn create_offer_hours_exceed_week() {
        let mut o = valid_offer();
        o.hours_available = 168.1;
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Hours available cannot exceed 168 per week",
        );
    }

    #[test]
    fn create_offer_hours_at_max_boundary() {
        let mut o = valid_offer();
        o.hours_available = 168.0;
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_hours_just_above_zero() {
        let mut o = valid_offer();
        o.hours_available = 0.01;
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_empty_location() {
        let mut o = valid_offer();
        o.location = String::new();
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Location cannot be empty",
        );
    }

    #[test]
    fn create_offer_location_too_long() {
        let mut o = valid_offer();
        o.location = "x".repeat(513);
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Location must be 512 characters or fewer",
        );
    }

    #[test]
    fn create_offer_location_at_boundary() {
        let mut o = valid_offer();
        o.location = "x".repeat(512);
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_availability_too_long() {
        let mut o = valid_offer();
        o.availability = "y".repeat(513);
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Availability must be 512 characters or fewer",
        );
    }

    #[test]
    fn create_offer_availability_at_boundary() {
        let mut o = valid_offer();
        o.availability = "y".repeat(512);
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_too_many_skills() {
        let mut o = valid_offer();
        o.skills_required = (0..21).map(|i| format!("skill_{i}")).collect();
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Cannot list more than 20 skills",
        );
    }

    #[test]
    fn create_offer_skills_at_max_count() {
        let mut o = valid_offer();
        o.skills_required = (0..20).map(|i| format!("skill_{i}")).collect();
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_skill_too_long() {
        let mut o = valid_offer();
        o.skills_required = vec!["z".repeat(129)];
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Each skill must be 128 characters or fewer",
        );
    }

    #[test]
    fn create_offer_skill_at_boundary() {
        let mut o = valid_offer();
        o.skills_required = vec!["z".repeat(128)];
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_empty_skills_list() {
        let mut o = valid_offer();
        o.skills_required = vec![];
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_empty_availability_ok() {
        let mut o = valid_offer();
        o.availability = String::new();
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_with_other_category() {
        let mut o = valid_offer();
        o.category = ServiceCategory::Other("Custom".to_string());
        assert_valid(validate_create_offer(fake_create(), o));
    }

    // ── validate_create_request tests ───────────────────────────────────

    #[test]
    fn create_request_valid() {
        assert_valid(validate_create_request(fake_create(), valid_request()));
    }

    #[test]
    fn create_request_empty_title() {
        let mut r = valid_request();
        r.title = String::new();
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Request title cannot be empty",
        );
    }

    #[test]
    fn create_request_title_too_long() {
        let mut r = valid_request();
        r.title = "t".repeat(257);
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Request title must be 256 characters or fewer",
        );
    }

    #[test]
    fn create_request_title_at_boundary() {
        let mut r = valid_request();
        r.title = "t".repeat(256);
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_empty_description() {
        let mut r = valid_request();
        r.description = String::new();
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Request description cannot be empty",
        );
    }

    #[test]
    fn create_request_description_too_long() {
        let mut r = valid_request();
        r.description = "d".repeat(4097);
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Request description must be 4096 characters or fewer",
        );
    }

    #[test]
    fn create_request_description_at_boundary() {
        let mut r = valid_request();
        r.description = "d".repeat(4096);
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_zero_hours() {
        let mut r = valid_request();
        r.hours_needed = 0.0;
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Hours needed must be positive",
        );
    }

    #[test]
    fn create_request_negative_hours() {
        let mut r = valid_request();
        r.hours_needed = -5.0;
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Hours needed must be positive",
        );
    }

    #[test]
    fn create_request_hours_exceed_168() {
        let mut r = valid_request();
        r.hours_needed = 169.0;
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Hours needed cannot exceed 168",
        );
    }

    #[test]
    fn create_request_hours_at_max_boundary() {
        let mut r = valid_request();
        r.hours_needed = 168.0;
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_hours_just_above_zero() {
        let mut r = valid_request();
        r.hours_needed = 0.001;
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_empty_location() {
        let mut r = valid_request();
        r.location = String::new();
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Location cannot be empty",
        );
    }

    #[test]
    fn create_request_all_urgency_levels_valid() {
        for urgency in [
            UrgencyLevel::Low,
            UrgencyLevel::Medium,
            UrgencyLevel::High,
            UrgencyLevel::Critical,
        ] {
            let mut r = valid_request();
            r.urgency = urgency;
            assert_valid(validate_create_request(fake_create(), r));
        }
    }

    // ── validate_create_exchange tests ──────────────────────────────────

    #[test]
    fn create_exchange_valid() {
        assert_valid(validate_create_exchange(fake_create(), valid_exchange()));
    }

    #[test]
    fn create_exchange_valid_no_ratings() {
        let mut e = valid_exchange();
        e.rating_provider = None;
        e.rating_recipient = None;
        assert_valid(validate_create_exchange(fake_create(), e));
    }

    #[test]
    fn create_exchange_zero_hours() {
        let mut e = valid_exchange();
        e.hours = 0.0;
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Exchange hours must be positive",
        );
    }

    #[test]
    fn create_exchange_negative_hours() {
        let mut e = valid_exchange();
        e.hours = -2.0;
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Exchange hours must be positive",
        );
    }

    #[test]
    fn create_exchange_hours_exceed_168() {
        let mut e = valid_exchange();
        e.hours = 168.5;
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Exchange hours cannot exceed 168",
        );
    }

    #[test]
    fn create_exchange_hours_at_max_boundary() {
        let mut e = valid_exchange();
        e.hours = 168.0;
        assert_valid(validate_create_exchange(fake_create(), e));
    }

    #[test]
    fn create_exchange_same_provider_and_recipient() {
        let mut e = valid_exchange();
        e.recipient = e.provider.clone();
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Provider and recipient cannot be the same agent",
        );
    }

    #[test]
    fn create_exchange_with_provider_rating_rejected() {
        // Ratings can only be set later via rate_exchange (an update), never on create.
        let mut e = valid_exchange();
        e.rating_provider = Some(3);
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "New exchanges cannot have ratings set",
        );
    }

    #[test]
    fn create_exchange_with_recipient_rating_rejected() {
        let mut e = valid_exchange();
        e.rating_recipient = Some(3);
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "New exchanges cannot have ratings set",
        );
    }

    #[test]
    fn create_exchange_empty_notes_ok() {
        let mut e = valid_exchange();
        e.notes = String::new();
        assert_valid(validate_create_exchange(fake_create(), e));
    }

    // ── validate_create_credit tests ────────────────────────────────────

    #[test]
    fn create_credit_zero_balances_rejected() {
        // No longer matches the required 5.0-hour starter shape.
        let c = TimeCredit {
            agent: agent_a(),
            balance: 0.0,
            total_earned: 0.0,
            total_spent: 0.0,
            updated_at: Timestamp::from_micros(0),
        };
        assert_invalid(
            validate_create_credit(fake_create(), c),
            "must start at the 5.0-hour starter balance",
        );
    }

    #[test]
    fn create_credit_starter_shape_valid() {
        let c = TimeCredit {
            agent: agent_a(),
            balance: 5.0,
            total_earned: 0.0,
            total_spent: 0.0,
            updated_at: Timestamp::from_micros(0),
        };
        assert_valid(validate_create_credit(fake_create(), c));
    }

    #[test]
    fn create_credit_non_starter_balance_rejected() {
        // valid_credit() represents an already-established credit record
        // (post-update), not the create-time starter shape.
        assert_invalid(
            validate_create_credit(fake_create(), valid_credit()),
            "must start at the 5.0-hour starter balance",
        );
    }

    #[test]
    fn create_credit_negative_earned() {
        let mut c = valid_credit();
        c.total_earned = -0.01;
        assert_invalid(
            validate_create_credit(fake_create(), c),
            "Total earned cannot be negative",
        );
    }

    #[test]
    fn create_credit_negative_spent() {
        let mut c = valid_credit();
        c.total_spent = -1.0;
        assert_invalid(
            validate_create_credit(fake_create(), c),
            "Total spent cannot be negative",
        );
    }

    // ── validate_credit_transition (pure update logic) tests ───────────

    #[test]
    fn update_credit_lockstep_valid() {
        let original = TimeCredit {
            agent: agent_a(),
            balance: 5.0,
            total_earned: 0.0,
            total_spent: 0.0,
            updated_at: Timestamp::from_micros(0),
        };
        let mut credit = original.clone();
        credit.balance = 10.0;
        credit.total_earned = 5.0;
        credit.updated_at = Timestamp::from_micros(1);
        let result = validate_credit_transition(&original, &credit);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_credit_agent_change_rejected() {
        let original = valid_credit();
        let mut credit = original.clone();
        credit.agent = agent_b();
        credit.updated_at = Timestamp::from_micros(1);
        let result = validate_credit_transition(&original, &credit);
        assert_invalid(Ok(result), "Credit agent cannot be changed");
    }

    #[test]
    fn update_credit_stale_timestamp_rejected() {
        let original = valid_credit();
        let credit = original.clone();
        let result = validate_credit_transition(&original, &credit);
        assert_invalid(Ok(result), "Credit updated_at must advance");
    }

    #[test]
    fn update_credit_decrease_rejected() {
        let original = valid_credit();
        let mut credit = original.clone();
        credit.total_earned -= 1.0;
        credit.updated_at = Timestamp::from_micros(1);
        let result = validate_credit_transition(&original, &credit);
        assert_invalid(Ok(result), "cannot decrease");
    }

    #[test]
    fn update_credit_offset_drift_rejected() {
        let original = valid_credit();
        let mut credit = original.clone();
        credit.balance += 100.0; // total_earned/total_spent unchanged
        credit.updated_at = Timestamp::from_micros(1);
        let result = validate_credit_transition(&original, &credit);
        assert_invalid(Ok(result), "must move in lockstep");
    }

    // ── validate_exchange_transition (pure update logic) tests ─────────

    #[test]
    fn update_exchange_provider_sets_own_rating_valid() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_provider = Some(5);
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_exchange_recipient_sets_own_rating_valid() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_recipient = Some(4);
        let result = validate_exchange_transition(&original, &exchange, &agent_b());
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn update_exchange_provider_rating_zero_rejected() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_provider = Some(0);
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(Ok(result), "Provider rating must be 1-5");
    }

    #[test]
    fn update_exchange_provider_rating_six_rejected() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_provider = Some(6);
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(Ok(result), "Provider rating must be 1-5");
    }

    #[test]
    fn update_exchange_recipient_rating_zero_rejected() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_recipient = Some(0);
        let result = validate_exchange_transition(&original, &exchange, &agent_b());
        assert_invalid(Ok(result), "Recipient rating must be 1-5");
    }

    #[test]
    fn update_exchange_provider_cannot_set_recipient_rating() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_recipient = Some(3);
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(Ok(result), "Only the provider's own rating");
    }

    #[test]
    fn update_exchange_recipient_cannot_set_provider_rating() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_provider = Some(3);
        let result = validate_exchange_transition(&original, &exchange, &agent_b());
        assert_invalid(Ok(result), "Only the recipient's own rating");
    }

    #[test]
    fn update_exchange_non_participant_rejected() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.rating_provider = Some(5);
        let forger = AgentPubKey::from_raw_36(vec![0xef; 36]);
        let result = validate_exchange_transition(&original, &exchange, &forger);
        assert_invalid(Ok(result), "Only the provider or recipient");
    }

    #[test]
    fn update_exchange_rejects_hours_change() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.hours = 99.0;
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(
            Ok(result),
            "Only rating_provider/rating_recipient can change",
        );
    }

    #[test]
    fn update_exchange_rejects_provider_change() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.provider = agent_b();
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(
            Ok(result),
            "Only rating_provider/rating_recipient can change",
        );
    }

    #[test]
    fn update_exchange_rejects_notes_change() {
        let original = valid_exchange();
        let mut exchange = original.clone();
        exchange.notes = "changed".to_string();
        let result = validate_exchange_transition(&original, &exchange, &agent_a());
        assert_invalid(
            Ok(result),
            "Only rating_provider/rating_recipient can change",
        );
    }

    // ── Create request max-length boundary tests ──────────────────────

    #[test]
    fn create_request_location_at_max_length() {
        let mut r = valid_request();
        r.location = "x".repeat(512);
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_location_over_max_length() {
        let mut r = valid_request();
        r.location = "x".repeat(513);
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Location must be 512 characters or fewer",
        );
    }

    #[test]
    fn create_request_preferred_schedule_at_max_length() {
        let mut r = valid_request();
        r.preferred_schedule = "s".repeat(512);
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_preferred_schedule_over_max_length() {
        let mut r = valid_request();
        r.preferred_schedule = "s".repeat(513);
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Preferred schedule must be 512 characters or fewer",
        );
    }

    // ── Create exchange max-length boundary tests ─────────────────────

    #[test]
    fn create_exchange_notes_at_max_length() {
        let mut e = valid_exchange();
        e.notes = "n".repeat(4096);
        assert_valid(validate_create_exchange(fake_create(), e));
    }

    #[test]
    fn create_exchange_notes_over_max_length() {
        let mut e = valid_exchange();
        e.notes = "n".repeat(4097);
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Notes must be 4096 characters or fewer",
        );
    }

    // ============================================================================
    // Link Tag Validation Tests
    // ============================================================================

    fn validate_create_link_tag(
        link_type: LinkTypes,
        tag_bytes: Vec<u8>,
    ) -> ExternResult<ValidateCallbackResult> {
        let tag = LinkTag(tag_bytes);
        match link_type {
            LinkTypes::AgentToOffer
            | LinkTypes::AgentToRequest
            | LinkTypes::AllActiveOffers
            | LinkTypes::AllOpenRequests
            | LinkTypes::AgentToExchange
            | LinkTypes::AgentToCredit
            | LinkTypes::OfferToExchange
            | LinkTypes::RequestToExchange => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(format!(
                        "{:?} link tag too long (max 256 bytes)",
                        link_type
                    )));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::CategoryToOffer | LinkTypes::CategoryToRequest => {
                if tag.0.len() > 512 {
                    return Ok(ValidateCallbackResult::Invalid(format!(
                        "{:?} link tag too long (max 512 bytes)",
                        link_type
                    )));
                }
                Ok(ValidateCallbackResult::Valid)
            }
        }
    }

    #[test]
    fn link_tag_agent_to_offer_at_limit() {
        assert_valid(validate_create_link_tag(
            LinkTypes::AgentToOffer,
            vec![0u8; 256],
        ));
    }

    #[test]
    fn link_tag_agent_to_offer_over_limit() {
        let result = validate_create_link_tag(LinkTypes::AgentToOffer, vec![0u8; 257]).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn link_tag_category_to_offer_at_limit() {
        assert_valid(validate_create_link_tag(
            LinkTypes::CategoryToOffer,
            vec![0u8; 512],
        ));
    }

    #[test]
    fn link_tag_category_to_offer_over_limit() {
        let result = validate_create_link_tag(LinkTypes::CategoryToOffer, vec![0u8; 513]).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn link_tag_category_to_request_at_limit() {
        assert_valid(validate_create_link_tag(
            LinkTypes::CategoryToRequest,
            vec![0u8; 512],
        ));
    }

    #[test]
    fn link_tag_category_to_request_over_limit() {
        let result =
            validate_create_link_tag(LinkTypes::CategoryToRequest, vec![0u8; 513]).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn link_tag_empty_tag_valid() {
        assert_valid(validate_create_link_tag(LinkTypes::AgentToOffer, vec![]));
    }

    // ── ServiceCategory::Other string length validation tests ─────────

    #[test]
    fn create_offer_other_category_at_limit() {
        let mut o = valid_offer();
        o.category = ServiceCategory::Other("a".repeat(128));
        assert_valid(validate_create_offer(fake_create(), o));
    }

    #[test]
    fn create_offer_other_category_too_long() {
        let mut o = valid_offer();
        o.category = ServiceCategory::Other("a".repeat(129));
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Custom service category label must be 128 characters or fewer",
        );
    }

    #[test]
    fn create_offer_other_category_empty() {
        let mut o = valid_offer();
        o.category = ServiceCategory::Other("".to_string());
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Custom service category label cannot be empty",
        );
    }

    #[test]
    fn create_offer_other_category_whitespace_only() {
        let mut o = valid_offer();
        o.category = ServiceCategory::Other("   ".to_string());
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Custom service category label cannot be empty",
        );
    }

    #[test]
    fn create_request_other_category_at_limit() {
        let mut r = valid_request();
        r.category = ServiceCategory::Other("a".repeat(128));
        assert_valid(validate_create_request(fake_create(), r));
    }

    #[test]
    fn create_request_other_category_too_long() {
        let mut r = valid_request();
        r.category = ServiceCategory::Other("a".repeat(129));
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Custom service category label must be 128 characters or fewer",
        );
    }

    #[test]
    fn create_request_other_category_empty() {
        let mut r = valid_request();
        r.category = ServiceCategory::Other("".to_string());
        assert_invalid(
            validate_create_request(fake_create(), r),
            "Custom service category label cannot be empty",
        );
    }

    #[test]
    fn create_exchange_other_category_at_limit() {
        let mut e = valid_exchange();
        e.category = ServiceCategory::Other("a".repeat(128));
        assert_valid(validate_create_exchange(fake_create(), e));
    }

    #[test]
    fn create_exchange_other_category_too_long() {
        let mut e = valid_exchange();
        e.category = ServiceCategory::Other("a".repeat(129));
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Custom service category label must be 128 characters or fewer",
        );
    }

    #[test]
    fn create_exchange_other_category_empty() {
        let mut e = valid_exchange();
        e.category = ServiceCategory::Other("".to_string());
        assert_invalid(
            validate_create_exchange(fake_create(), e),
            "Custom service category label cannot be empty",
        );
    }

    // ── Empty skill validation tests ─────────────────────────────────

    #[test]
    fn create_offer_empty_skill_name() {
        let mut o = valid_offer();
        o.skills_required = vec!["".to_string()];
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Skill name cannot be empty",
        );
    }

    #[test]
    fn create_offer_whitespace_skill_name() {
        let mut o = valid_offer();
        o.skills_required = vec!["   ".to_string()];
        assert_invalid(
            validate_create_offer(fake_create(), o),
            "Skill name cannot be empty",
        );
    }

    // validate_update_offer/validate_update_request no longer exist: the
    // dispatcher now rejects all ServiceOffer/ServiceRequest updates
    // outright (immutable; re-post to change details), matching the
    // standalone mycelix-care/zomes/timebank fix -- confirmed no live
    // coordinator update flow exists for either entry type.
}
