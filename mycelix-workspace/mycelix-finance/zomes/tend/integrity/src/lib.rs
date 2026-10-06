#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! TEND (Time Exchange) Integrity Zome
//!
//! Implements Commons Charter Article II, Section 2 - Time Exchange Module
//!
//! Core Principles:
//! - 1 TEND = 1 hour of service (all labor valued equally)
//! - Mutual credit: members can go negative (debt) or positive (credit)
//! - Balance limits: ±40 TEND (prevents excessive debt/credit)
//! - No interest: time doesn't compound, debt doesn't grow
//! - Community-scoped: each Local DAO has its own TEND ledger
//!
//! Constitutional Reference: Commons Charter v1.0, Article II, Section 2
//! MIP Template Reference: MIP-C-042

use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};
pub use mycelix_finance_types::{
    Currency, HEARTH_MAX_MEMBERS, HEARTH_TEND_CREDIT_LIMIT, TendLimitTier,
};

// =============================================================================
// CONSTANTS (Per MIP-C-042 Template)
// =============================================================================

/// Balance limit (both positive and negative)
pub const BALANCE_LIMIT: i32 = 40;

/// 1 TEND = 1 hour of service (in minutes for precision)
pub const TEND_UNIT_MINUTES: u32 = 60;

/// Maximum service duration in one transaction (8 hours)
pub const MAX_SERVICE_HOURS: u32 = 8;

/// Minimum service duration (15 minutes)
pub const MIN_SERVICE_MINUTES: u32 = 15;

/// Elevated balance limit (Stressed state)
pub const BALANCE_LIMIT_ELEVATED: i32 = 60;

/// High balance limit (Critical state)
pub const BALANCE_LIMIT_HIGH: i32 = 80;

/// Emergency balance limit (Failing state)
pub const BALANCE_LIMIT_EMERGENCY: i32 = 120;

/// Apprentice balance limit
pub const APPRENTICE_BALANCE_LIMIT: i32 = 10;

// String length limits — prevent DHT bloat attacks
const MAX_DID_LEN: usize = 256;
const MAX_ID_LEN: usize = 256;
const MAX_TITLE_LEN: usize = 200;
const MAX_DESCRIPTION_LEN: usize = 2000;
const MAX_AVAILABILITY_LEN: usize = 1024;
const MAX_RESOLUTION_LEN: usize = 4096;
const MAX_CULTURAL_ALIAS_LEN: usize = 64;

// =============================================================================
// ENTRY TYPES
// =============================================================================

/// A Time Exchange transaction
///
/// When Alice provides 2 hours of service to Bob:
/// - Alice earns +2 TEND (credit)
/// - Bob spends -2 TEND (debt)
///
/// This is MUTUAL CREDIT: the total TEND in the system is always zero.
/// One person's credit is another's debt.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TendExchange {
    /// Unique identifier
    pub id: String,

    /// DID of the service provider (earns TEND)
    pub provider_did: String,

    /// DID of the service receiver (spends TEND)
    pub receiver_did: String,

    /// Amount of TEND exchanged (in hours, e.g., 2.5 = 2h 30m)
    pub hours: f32,

    /// Description of the service provided
    pub service_description: String,

    /// Category of service
    pub service_category: ServiceCategory,

    /// Cultural alias used (if any) - e.g., "CARE", "HOURS"
    pub cultural_alias: Option<String>,

    /// The DAO/community where this exchange occurred
    pub dao_did: String,

    /// When the exchange was recorded
    pub timestamp: Timestamp,

    /// Status of the exchange
    pub status: ExchangeStatus,

    /// Optional: when the service was actually performed (if different from recorded)
    pub service_date: Option<Timestamp>,
}

/// Categories of service that can be exchanged
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ServiceCategory {
    /// Childcare, eldercare, pet care
    CareWork,
    /// Home repairs, maintenance
    HomeServices,
    /// Cooking, meal prep
    FoodServices,
    /// Driving, moving help
    Transportation,
    /// Tutoring, teaching
    Education,
    /// General help, errands
    GeneralAssistance,
    /// Administrative, paperwork
    Administrative,
    /// Creative work (art, music, writing)
    Creative,
    /// Tech support, computer help
    TechSupport,
    /// Health and wellness (non-medical)
    Wellness,
    /// Gardening, landscaping
    Gardening,
    /// Custom category defined by DAO
    Custom(String),
}

/// Status of an exchange
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ExchangeStatus {
    /// Proposed by provider, awaiting receiver confirmation
    Proposed,
    /// Confirmed by both parties
    Confirmed,
    /// Disputed by one party
    Disputed,
    /// Cancelled before confirmation
    Cancelled,
    /// Resolved after dispute
    Resolved,
}

impl ExchangeStatus {
    /// Valid status transitions. Terminal states (Confirmed, Cancelled, Resolved) cannot revert.
    pub fn can_transition_to(&self, new: &ExchangeStatus) -> bool {
        matches!(
            (self, new),
            (ExchangeStatus::Proposed, ExchangeStatus::Confirmed)
                | (ExchangeStatus::Proposed, ExchangeStatus::Disputed)
                | (ExchangeStatus::Proposed, ExchangeStatus::Cancelled)
                | (ExchangeStatus::Disputed, ExchangeStatus::Resolved)
                | (ExchangeStatus::Disputed, ExchangeStatus::Cancelled)
        )
    }
}

/// Member's TEND balance within a DAO
///
/// Balance can be positive (credit - you've provided more than received)
/// or negative (debt - you've received more than provided).
/// Both are limited to ±40 TEND.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TendBalance {
    /// DID of the member
    pub member_did: String,

    /// The DAO/community
    pub dao_did: String,

    /// Current balance (can be negative)
    pub balance: i32, // Using i32 for negative support

    /// Total hours provided (lifetime)
    pub total_provided: f32,

    /// Total hours received (lifetime)
    pub total_received: f32,

    /// Number of exchanges participated in
    pub exchange_count: u32,

    /// Last activity timestamp
    pub last_activity: Timestamp,
}

/// Service listing - offering services to the community
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ServiceListing {
    /// Unique identifier
    pub id: String,

    /// DID of the service provider
    pub provider_did: String,

    /// The DAO/community
    pub dao_did: String,

    /// Title of the service
    pub title: String,

    /// Description
    pub description: String,

    /// Category
    pub category: ServiceCategory,

    /// Estimated hours (optional)
    pub estimated_hours: Option<f32>,

    /// Availability notes
    pub availability: Option<String>,

    /// Whether the listing is active
    pub active: bool,

    /// When listed
    pub created: Timestamp,
}

/// Service request - requesting services from the community
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ServiceRequest {
    /// Unique identifier
    pub id: String,

    /// DID of the requester
    pub requester_did: String,

    /// The DAO/community
    pub dao_did: String,

    /// Title of the request
    pub title: String,

    /// Description of what's needed
    pub description: String,

    /// Category
    pub category: ServiceCategory,

    /// Estimated hours needed
    pub estimated_hours: Option<f32>,

    /// Urgency level
    pub urgency: Urgency,

    /// Whether the request is still open
    pub open: bool,

    /// When requested
    pub created: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum Urgency {
    /// Flexible timing
    Flexible,
    /// Within a week
    SoonPreferred,
    /// Within a few days
    Urgent,
    /// Immediate need
    Emergency,
}

/// Quality rating for a completed exchange
///
/// After an exchange is confirmed, either party can rate the experience.
/// Ratings feed into MYCEL reputation scores and help the community
/// identify reliable service providers.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct QualityRating {
    /// ID of the exchange being rated
    pub exchange_id: String,

    /// DID of the member submitting the rating
    pub rater_did: String,

    /// DID of the service provider being rated
    pub provider_did: String,

    /// Rating from 1-5 stars
    pub rating: u8,

    /// Optional comment explaining the rating
    pub comment: Option<String>,

    /// When the rating was submitted
    pub timestamp: Timestamp,
}

// =============================================================================
// DISPUTE RESOLUTION
// =============================================================================

/// Stage of a dispute case, following the three-tier resolution process.
///
/// Disputes escalate through stages with increasing community involvement:
/// 1. DirectNegotiation - 72 hours for the two parties to resolve directly
/// 2. MediationPanel - 3 random members with MYCEL > 0.5, 7 day window
/// 3. GovernanceVote - Full community vote, final and binding
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum DisputeStage {
    /// 72 hours between parties to resolve directly
    DirectNegotiation,
    /// 3 random members with MYCEL > 0.5, 7 days to mediate
    MediationPanel,
    /// Final, binding community governance vote
    GovernanceVote,
}

/// A dispute case for a contested exchange
///
/// When an exchange is disputed, a DisputeCase tracks the resolution
/// process through the three-tier escalation system defined in the
/// Commons Charter.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DisputeCase {
    /// Unique identifier for the dispute
    pub id: String,

    /// ID of the disputed exchange
    pub exchange_id: String,

    /// DID of the member who filed the dispute
    pub complainant_did: String,

    /// DID of the member the dispute is against
    pub respondent_did: String,

    /// Current stage of the dispute
    pub stage: DisputeStage,

    /// Description of the dispute
    pub description: String,

    /// DIDs of assigned mediators (populated in MediationPanel stage)
    pub mediator_dids: Vec<String>,

    /// Resolution outcome (populated when resolved)
    pub resolution: Option<String>,

    /// When the dispute was opened
    pub opened_at: Timestamp,

    /// When the dispute was escalated to the next stage (if applicable)
    pub escalated_at: Option<Timestamp>,

    /// When the dispute was resolved (if applicable)
    pub resolved_at: Option<Timestamp>,
}

// =============================================================================
// ENTRY & LINK TYPE ENUMS
// =============================================================================

/// Oracle state for counter-cyclical TEND limit adjustments.
///
/// Updated by the metabolic oracle (or governance) to signal network health.
/// The TEND coordinator reads this to determine dynamic balance limits.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct OracleState {
    /// Network vitality score (0-100)
    pub vitality: u32,
    /// Total thermodynamic yield in the constellation (Loop 3)
    pub total_yield_kwh: f32,
    /// Current TEND limit tier (derived from vitality)
    pub tier: TendLimitTier,
    /// When this state was last updated
    pub updated_at: Timestamp,
}

impl OracleState {
    /// Derive the limit tier from a vitality score
    pub fn tier_from_vitality(vitality: u32) -> TendLimitTier {
        TendLimitTier::from_vitality(vitality)
    }
}

// =============================================================================
// HEARTH-SCOPED TEND (Phase 2: lightweight sub-ledgers for family units)
// =============================================================================

/// Hearth-scoped TEND balance (smaller limit than DAO: ±20 instead of ±40).
///
/// Hearths are intimate groups (2-50 members) that don't need full DAO
/// overhead. Their TEND operates with tighter limits and simpler governance.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HearthTendBalance {
    /// DID of the member
    pub member_did: String,
    /// DID of the hearth (family unit)
    pub hearth_did: String,
    /// Current balance (limited to ±HEARTH_TEND_CREDIT_LIMIT)
    pub balance: i32,
    /// Total hours provided within this hearth
    pub total_provided: f32,
    /// Total hours received within this hearth
    pub total_received: f32,
    /// Number of exchanges within this hearth
    pub exchange_count: u32,
    /// Last activity timestamp
    pub last_activity: Timestamp,
}

// =============================================================================
// CULTURAL ALIASES (Phase 3: community-named currencies)
// =============================================================================

/// A registered cultural alias for a community's currency.
///
/// Communities can name their local TEND/SAP however they want:
/// "Cuidado" (care), "Ubuntu Hours", "Horas", "Water Credits".
/// The alias is display-only — the underlying mutual credit physics are unchanged.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CurrencyAliasEntry {
    /// The community/DAO that registered this alias
    pub dao_did: String,
    /// The human-readable alias name (e.g., "CARE", "HORAS", "UBUNTU")
    pub alias_name: String,
    /// Which base currency this aliases
    pub base_currency: Currency,
    /// Optional short display symbol (e.g., "C", "H")
    pub display_symbol: Option<String>,
    /// Optional description of cultural meaning
    pub description: Option<String>,
    /// When this alias was registered
    pub created_at: Timestamp,
}

/// A pending balance adjustment for crash recovery in confirm_exchange.
///
/// Written BEFORE the two balance updates so that if a crash occurs between
/// the provider update and the receiver update, a governance agent can call
/// `recover_pending_adjustments` to complete the interrupted operation and
/// restore the zero-sum invariant.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PendingBalanceAdjustment {
    /// The exchange this adjustment belongs to
    pub exchange_id: String,
    /// Exact ActionHash of the original TendExchange create action.
    pub exchange_action_hash: ActionHash,
    /// DID of the service provider (gains hours)
    pub provider_did: String,
    /// DID of the service receiver (spends hours)
    pub receiver_did: String,
    /// Amount of TEND-hours being exchanged
    pub hours: f64,
    /// The currency/DAO scope
    pub currency_id: String,
    /// Whether the provider's balance has been updated
    pub provider_completed: bool,
    /// Whether the receiver's balance has been updated
    pub receiver_completed: bool,
    /// When this pending adjustment was created
    pub created_at: Timestamp,
}

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

/// Status of a bilateral settlement (two-phase commit pattern).
///
/// Settlements transition: Pending -> Completed (if treasury transfer succeeds)
///                         Pending -> Failed (if treasury transfer fails)
/// No other transitions are valid.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum SettlementStatus {
    /// Phase 1: Settlement created, awaiting treasury SAP transfer
    Pending,
    /// Phase 2 success: Treasury transfer completed, bilateral balance zeroed
    Completed,
    /// Phase 2 failure: Treasury transfer failed, bilateral balance unchanged
    Failed,
}

/// A bilateral settlement record (two-phase commit for TEND clearing).
///
/// When settling inter-DAO TEND imbalances, the settlement is created in
/// Pending status BEFORE the treasury SAP transfer. Only after the transfer
/// succeeds is the bilateral balance zeroed and the settlement marked Completed.
/// If the transfer fails, the settlement is marked Failed and the bilateral
/// balance remains unchanged -- no debt is lost.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BilateralSettlement {
    /// Unique identifier for this settlement
    pub id: String,
    /// DID of the debtor DAO (the DAO that owes TEND-hours)
    pub debtor_dao_did: String,
    /// DID of the creditor DAO (the DAO that is owed TEND-hours)
    pub creditor_dao_did: String,
    /// Amount of TEND-hours to settle (always positive)
    pub amount: i32,
    /// Current status of the settlement
    pub status: SettlementStatus,
    /// When the settlement was created (Phase 1)
    pub created_at: Timestamp,
    /// When the settlement completed or failed (Phase 2), if applicable
    pub completed_at: Option<Timestamp>,
}

/// Bilateral balance between two DAOs for inter-community TEND clearing.
///
/// When a member from DAO-A provides service to a member of DAO-B,
/// the exchange is recorded locally in each DAO's zero-sum ledger,
/// but the inter-DAO imbalance is tracked here.
///
/// Bilateral balances are settled quarterly via SAP transfer from
/// the debtor DAO's commons pool to the creditor DAO's commons pool.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct BilateralBalance {
    /// DID of one DAO (the "left" side, alphabetically first)
    pub dao_a_did: String,
    /// DID of the other DAO (the "right" side)
    pub dao_b_did: String,
    /// Net balance in TEND-hours. Positive = DAO-A is owed by DAO-B.
    pub net_balance: i32,
    /// Running total of exchanges crossing this bilateral pair
    pub total_exchanges: u32,
    /// Last time this balance was settled (or created)
    pub last_settled_at: Timestamp,
    /// Last time this balance was updated
    pub last_updated_at: Timestamp,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    TendExchange(TendExchange),
    TendBalance(TendBalance),
    ServiceListing(ServiceListing),
    ServiceRequest(ServiceRequest),
    Anchor(Anchor),
    QualityRating(QualityRating),
    DisputeCase(DisputeCase),
    OracleState(OracleState),
    BilateralBalance(BilateralBalance),
    BilateralSettlement(BilateralSettlement),
    HearthTendBalance(HearthTendBalance),
    CurrencyAliasEntry(CurrencyAliasEntry),
    PendingBalanceAdjustment(PendingBalanceAdjustment),
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Link from provider DID to exchanges where they provided
    ProviderToExchanges,
    /// Link from receiver DID to exchanges where they received
    ReceiverToExchanges,
    /// Link from member DID to their balance
    MemberToBalance,
    /// Reverse index from member DID to balances across DAOs
    MemberToDaoBalances,
    /// Reverse index from member DID to other DAO-scoped activity
    MemberToDaoActivity,
    /// Link from DAO to all exchanges in that community
    DaoToExchanges,
    /// Link from DAO to service listings
    DaoToListings,
    /// Link from DAO to service requests
    DaoToRequests,
    /// Link from provider to their listings
    ProviderToListings,
    /// Link from category anchor to listings
    CategoryToListings,
    /// Link from exchange ID to exchange entry (for lookup by ID)
    ExchangeIdToExchange,
    /// Link type for anchor/path infrastructure
    AnchorLinks,
    /// Link from exchange to quality ratings
    ExchangeToRating,
    /// Link from member DID to disputes they are involved in
    MemberToDisputes,
    /// Link from exchange to its dispute case
    ExchangeToDispute,
    /// Link from DAO-pair anchor to bilateral balance
    DaoToBilateralBalance,
    /// Link from settlement registry anchor to settlement entries
    SettlementRegistry,
    /// Link from governance_agents anchor to authorized agent pubkeys
    GovernanceAgents,
    /// Link from hearth DID to hearth TEND balances
    HearthToBalances,
    /// Link from member DID to hearth TEND balances
    MemberToHearthBalance,
    /// Link from DAO to its registered currency alias
    DaoToAlias,
    /// Link from pending balance adjustment to its exchange entry
    PendingAdjustmentToExchange,
}

// =============================================================================
// VALIDATION
// =============================================================================

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => {
                match app_entry {
                    EntryTypes::TendExchange(exchange) => {
                        validate_create_exchange(EntryCreationAction::Create(action), exchange)
                    }
                    EntryTypes::TendBalance(balance) => {
                        validate_create_balance(EntryCreationAction::Create(action), balance)
                    }
                    EntryTypes::ServiceListing(listing) => {
                        validate_create_listing(EntryCreationAction::Create(action), listing)
                    }
                    EntryTypes::ServiceRequest(request) => {
                        validate_create_request(EntryCreationAction::Create(action), request)
                    }
                    EntryTypes::QualityRating(rating) => {
                        validate_create_quality_rating(EntryCreationAction::Create(action), rating)
                    }
                    EntryTypes::DisputeCase(dispute) => {
                        validate_create_dispute_case(EntryCreationAction::Create(action), dispute)
                    }
                    EntryTypes::OracleState(state) => {
                        validate_create_oracle_state(EntryCreationAction::Create(action), state)
                    }
                    EntryTypes::BilateralBalance(bal) => {
                        validate_create_bilateral_balance(EntryCreationAction::Create(action), bal)
                    }
                    EntryTypes::BilateralSettlement(settlement) => {
                        validate_create_bilateral_settlement(
                            EntryCreationAction::Create(action),
                            settlement,
                        )
                    }
                    EntryTypes::HearthTendBalance(bal) => validate_create_hearth_balance(bal),
                    EntryTypes::CurrencyAliasEntry(alias) => validate_create_currency_alias(alias),
                    EntryTypes::PendingBalanceAdjustment(adj) => {
                        validate_create_pending_balance_adjustment(
                            EntryCreationAction::Create(action),
                            adj,
                        )
                    }
                    // Anchors are always valid (just hash placeholders)
                    EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                }
            }
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => {
                match app_entry {
                    EntryTypes::TendExchange(exchange) => {
                        validate_update_exchange(action, exchange)
                    }
                    EntryTypes::TendBalance(balance) => validate_update_balance(action, balance),
                    EntryTypes::ServiceListing(listing) => validate_update_listing(action, listing),
                    EntryTypes::ServiceRequest(request) => validate_update_request(action, request),
                    EntryTypes::QualityRating(_) => {
                        // Ratings are immutable once created
                        Ok(ValidateCallbackResult::Invalid(
                            "Quality ratings cannot be updated".into(),
                        ))
                    }
                    EntryTypes::DisputeCase(dispute) => {
                        validate_update_dispute_case(action, dispute)
                    }
                    EntryTypes::OracleState(state) => validate_update_oracle_state(action, state)
                    EntryTypes::BilateralBalance(balance) => {
                        validate_update_bilateral_balance(action, balance)
                    }
                    EntryTypes::BilateralSettlement(settlement) => {
                        validate_update_bilateral_settlement(action, settlement)
                    }
                    EntryTypes::HearthTendBalance(bal) => validate_update_hearth_balance(action, bal),
                    EntryTypes::CurrencyAliasEntry(_) => {
                        // Aliases can be updated (e.g., change display name)
                        Ok(ValidateCallbackResult::Valid)
                    }
                    EntryTypes::PendingBalanceAdjustment(adj) => {
                        validate_update_pending_balance_adjustment(action, adj)
                    }
                    // Anchors cannot be updated
                    EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                        "Anchors cannot be updated".into(),
                    )),
                }
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::ProviderToExchanges => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ReceiverToExchanges => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToBalance => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToDaoBalances => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToDaoActivity => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DaoToExchanges => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DaoToListings => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DaoToRequests => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ProviderToListings => Ok(ValidateCallbackResult::Valid),
            LinkTypes::CategoryToListings => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ExchangeIdToExchange => Ok(ValidateCallbackResult::Valid),
            LinkTypes::AnchorLinks => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ExchangeToRating => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToDisputes => Ok(ValidateCallbackResult::Valid),
            LinkTypes::ExchangeToDispute => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DaoToBilateralBalance => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SettlementRegistry => Ok(ValidateCallbackResult::Valid),
            LinkTypes::GovernanceAgents => Ok(ValidateCallbackResult::Valid),
            LinkTypes::HearthToBalances => Ok(ValidateCallbackResult::Valid),
            LinkTypes::MemberToHearthBalance => Ok(ValidateCallbackResult::Valid),
            LinkTypes::DaoToAlias => Ok(ValidateCallbackResult::Valid),
            LinkTypes::PendingAdjustmentToExchange => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_exchange(
    _action: EntryCreationAction,
    exchange: TendExchange,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if exchange.provider_did.len() > MAX_DID_LEN
        || exchange.receiver_did.len() > MAX_DID_LEN
        || exchange.dao_did.len() > MAX_DID_LEN
    {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if exchange.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange ID exceeds maximum length".into(),
        ));
    }
    if let Some(ref alias) = exchange.cultural_alias {
        if alias.len() > MAX_CULTURAL_ALIAS_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Cultural alias exceeds maximum length".into(),
            ));
        }
    }

    // Validate DIDs
    if !exchange.provider_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider must be a valid DID".into(),
        ));
    }
    if !exchange.receiver_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver must be a valid DID".into(),
        ));
    }
    if !exchange.dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO must be a valid DID".into(),
        ));
    }

    // Cannot exchange with yourself
    if exchange.provider_did == exchange.receiver_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot exchange time with yourself".into(),
        ));
    }

    // Hours must be finite, positive, and within limits
    if !exchange.hours.is_finite() || exchange.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours must be a finite positive number".into(),
        ));
    }

    let minutes = (exchange.hours * 60.0) as u32;
    if minutes < MIN_SERVICE_MINUTES {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Minimum service duration is {} minutes",
            MIN_SERVICE_MINUTES
        )));
    }
    if exchange.hours > MAX_SERVICE_HOURS as f32 {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Maximum service duration is {} hours per exchange",
            MAX_SERVICE_HOURS
        )));
    }

    // Description required
    if exchange.service_description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Service description is required".into(),
        ));
    }
    if exchange.service_description.len() > 2000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Service description too long (max 2000 chars)".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_exchange(
    action: Update,
    exchange: TendExchange,
) -> ExternResult<ValidateCallbackResult> {
    // Only status can change. Core exchange data is immutable.
    if !exchange.hours.is_finite() || exchange.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Hours must be a finite positive number".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original = original_record
        .entry()
        .to_app_option::<TendExchange>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "TendExchange predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "TendExchange update predecessor is not a TendExchange entry".into()
            ))
        })?;

    if original.status != exchange.status
        && !original.status.can_transition_to(&exchange.status)
    {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid exchange status transition: {:?} → {:?}",
            original.status, exchange.status
        )));
    }

    if original.provider_did != exchange.provider_did
        || original.receiver_did != exchange.receiver_did
        || original.hours != exchange.hours
        || original.service_description != exchange.service_description
        || original.service_category != exchange.service_category
        || original.cultural_alias != exchange.cultural_alias
        || original.dao_did != exchange.dao_did
        || original.timestamp != exchange.timestamp
        || original.service_date != exchange.service_date
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot change immutable exchange terms on an existing exchange".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_balance(
    _action: EntryCreationAction,
    balance: TendBalance,
) -> ExternResult<ValidateCallbackResult> {
    // A newly created balance is only an initialization shell. Value and
    // history must be introduced by the validated settlement update protocol.
    if balance.balance != 0
        || balance.total_provided != 0.0
        || balance.total_received != 0.0
        || balance.exchange_count != 0
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Initial TendBalance must be zero-valued and have no exchange history".into(),
        ));
    }
    // String length checks — prevent DHT bloat
    if balance.member_did.len() > MAX_DID_LEN || balance.dao_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }

    if !balance.member_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Member must be a valid DID".into(),
        ));
    }
    if !balance.dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO must be a valid DID".into(),
        ));
    }

    // Float fields must be finite
    if !balance.total_provided.is_finite() || !balance.total_received.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "total_provided and total_received must be finite numbers".into(),
        ));
    }

    // Balance must be within the constitutional maximum (Emergency tier ±120).
    // The coordinator enforces the tighter dynamic limit based on oracle state.
    if balance.balance.abs() > BALANCE_LIMIT_EMERGENCY {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Balance exceeds constitutional maximum of ±{}",
            BALANCE_LIMIT_EMERGENCY
        )));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_balance(
    action: Update,
    balance: TendBalance,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<TendBalance>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "TendBalance predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "TendBalance update predecessor is not a TendBalance".into()
            ))
        })?;

    if let ValidateCallbackResult::Invalid(msg) =
        validate_balance_state_transition(&original, &balance)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_balance_state_transition(
    original: &TendBalance,
    updated: &TendBalance,
) -> ValidateCallbackResult {
    // Identity and accounting domain are immutable.
    if original.member_did != updated.member_did {
        return ValidateCallbackResult::Invalid(
            "TendBalance member_did is immutable across updates".into(),
        );
    }
    if original.dao_did != updated.dao_did {
        return ValidateCallbackResult::Invalid(
            "TendBalance dao_did is immutable across updates".into(),
        );
    }

    // Historical counters are append-only.
    if updated.total_provided < original.total_provided {
        return ValidateCallbackResult::Invalid(
            "TendBalance total_provided cannot decrease".into(),
        );
    }
    if updated.total_received < original.total_received {
        return ValidateCallbackResult::Invalid(
            "TendBalance total_received cannot decrease".into(),
        );
    }
    if updated.exchange_count < original.exchange_count {
        return ValidateCallbackResult::Invalid(
            "TendBalance exchange_count cannot decrease".into(),
        );
    }
    if updated.last_activity < original.last_activity {
        return ValidateCallbackResult::Invalid(
            "TendBalance last_activity cannot move backwards".into(),
        );
    }

    if !updated.total_provided.is_finite() || !updated.total_received.is_finite() {
        return ValidateCallbackResult::Invalid(
            "total_provided and total_received must be finite numbers".into(),
        );
    }

    if updated.balance.abs() > BALANCE_LIMIT_EMERGENCY {
        return ValidateCallbackResult::Invalid(format!(
            "Balance would exceed constitutional maximum of ±{}",
            BALANCE_LIMIT_EMERGENCY
        ));
    }

    ValidateCallbackResult::Valid
}

fn validate_create_listing(
    action: EntryCreationAction,
    listing: ServiceListing,
) -> ExternResult<ValidateCallbackResult> {
    // Bind to the committing agent. `create_listing`/`create_request`
    // (tend/coordinator:2171/:2303) already derive this field from
    // `agent_info()`, so this enforces at the DHT level what the coordinator
    // already does (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance Class-A).
    //
    // Create-only: `validate_update_servicelisting` exists but no coordinator path
    // updates one today, so it is deliberately left unbound rather than guessing at
    // a future edit flow's authority model.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) = require_did_is_author(
        "ServiceListing",
        "provider_did",
        &listing.provider_did,
        &author_did,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // String length checks — prevent DHT bloat
    if listing.provider_did.len() > MAX_DID_LEN || listing.dao_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if listing.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Listing ID exceeds maximum length".into(),
        ));
    }
    if listing.description.len() > MAX_DESCRIPTION_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Description exceeds maximum length of 2000".into(),
        ));
    }
    if let Some(ref avail) = listing.availability {
        if avail.len() > MAX_AVAILABILITY_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Availability exceeds maximum length".into(),
            ));
        }
    }

    if !listing.provider_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider must be a valid DID".into(),
        ));
    }
    if !listing.dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO must be a valid DID".into(),
        ));
    }
    if listing.title.is_empty() || listing.title.len() > MAX_TITLE_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Title must be 1-200 chars".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_listing(
    _action: Update,
    _listing: ServiceListing,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_request(
    action: EntryCreationAction,
    request: ServiceRequest,
) -> ExternResult<ValidateCallbackResult> {
    // Bind to the committing agent. `create_listing`/`create_request`
    // (tend/coordinator:2171/:2303) already derive this field from
    // `agent_info()`, so this enforces at the DHT level what the coordinator
    // already does (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance Class-A).
    //
    // Create-only: `validate_update_servicerequest` exists but no coordinator path
    // updates one today, so it is deliberately left unbound rather than guessing at
    // a future edit flow's authority model.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) = require_did_is_author(
        "ServiceRequest",
        "requester_did",
        &request.requester_did,
        &author_did,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    // String length checks — prevent DHT bloat
    if request.requester_did.len() > MAX_DID_LEN || request.dao_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if request.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Request ID exceeds maximum length".into(),
        ));
    }
    if request.description.len() > MAX_DESCRIPTION_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Description exceeds maximum length of 2000".into(),
        ));
    }

    if !request.requester_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Requester must be a valid DID".into(),
        ));
    }
    if !request.dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO must be a valid DID".into(),
        ));
    }
    if request.title.is_empty() || request.title.len() > MAX_TITLE_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Title must be 1-200 chars".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_request(
    _action: Update,
    _request: ServiceRequest,
) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_quality_rating(
    action: EntryCreationAction,
    rating: QualityRating,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the rating to its committer. `rate_exchange` (tend/coordinator:1305)
    // never checks the caller, so before this any agent could submit ratings as
    // anyone — inflating their own reputation or smearing a provider under a
    // third party's name (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance
    // Class-A, `tend:1059`). Ratings are immutable, so create is the only path.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) =
        require_did_is_author("QualityRating", "rater_did", &rating.rater_did, &author_did)
    {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    // String length checks — prevent DHT bloat
    if rating.rater_did.len() > MAX_DID_LEN || rating.provider_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if rating.exchange_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange ID exceeds maximum length".into(),
        ));
    }

    // Validate rater DID
    if !rating.rater_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Rater must be a valid DID".into(),
        ));
    }

    // Validate provider DID
    if !rating.provider_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider must be a valid DID".into(),
        ));
    }

    // Cannot rate yourself
    if rating.rater_did == rating.provider_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot rate yourself".into(),
        ));
    }

    // Rating must be 1-5 stars
    if rating.rating < 1 || rating.rating > 5 {
        return Ok(ValidateCallbackResult::Invalid(
            "Rating must be between 1 and 5".into(),
        ));
    }

    // Exchange ID must not be empty
    if rating.exchange_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange ID is required".into(),
        ));
    }

    // Comment length check (if provided)
    if let Some(ref comment) = rating.comment {
        if comment.len() > 2000 {
            return Ok(ValidateCallbackResult::Invalid(
                "Comment too long (max 2000 chars)".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_dispute_case(
    action: EntryCreationAction,
    dispute: DisputeCase,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the dispute to its committer. `open_dispute` (tend/coordinator:1440)
    // never checks the caller, so before this any agent could open a dispute in
    // someone else's name (MYCELIX_AUTHOR_BINDING_TRIAGE_2026-07-09.md, finance
    // Class-A, `tend:1122`).
    //
    // Create only. `validate_update_dispute_case` is deliberately left unbound:
    // no coordinator path updates a DisputeCase today, and a future resolution
    // flow would be driven by a mediator — i.e. a third party — which makes it a
    // Class-D authority question, not an author bind.
    let author_did = did_for_author(action.author());
    if let ValidateCallbackResult::Invalid(msg) = require_did_is_author(
        "DisputeCase",
        "complainant_did",
        &dispute.complainant_did,
        &author_did,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }
    // String length checks — prevent DHT bloat
    if dispute.complainant_did.len() > MAX_DID_LEN || dispute.respondent_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if dispute.id.len() > MAX_ID_LEN || dispute.exchange_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "ID exceeds maximum length".into(),
        ));
    }
    if let Some(ref resolution) = dispute.resolution {
        if resolution.len() > MAX_RESOLUTION_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Resolution exceeds maximum length".into(),
            ));
        }
    }
    for mediator_did in &dispute.mediator_dids {
        if mediator_did.len() > MAX_DID_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Mediator DID exceeds maximum length".into(),
            ));
        }
    }

    // Validate complainant DID
    if !dispute.complainant_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Complainant must be a valid DID".into(),
        ));
    }

    // Validate respondent DID
    if !dispute.respondent_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Respondent must be a valid DID".into(),
        ));
    }

    // Cannot dispute with yourself
    if dispute.complainant_did == dispute.respondent_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot file a dispute against yourself".into(),
        ));
    }

    // Validate mediator DIDs (if any are present)
    for mediator_did in &dispute.mediator_dids {
        if !mediator_did.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "All mediator DIDs must be valid".into(),
            ));
        }
    }

    // Description is required
    if dispute.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute description is required".into(),
        ));
    }
    if dispute.description.len() > 5000 {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute description too long (max 5000 chars)".into(),
        ));
    }

    // Exchange ID must not be empty
    if dispute.exchange_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Exchange ID is required".into(),
        ));
    }

    // Dispute ID must not be empty
    if dispute.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute ID is required".into(),
        ));
    }

    // New disputes must start at DirectNegotiation stage
    if dispute.stage != DisputeStage::DirectNegotiation {
        return Ok(ValidateCallbackResult::Invalid(
            "New disputes must start at DirectNegotiation stage".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_bilateral_settlement(
    action: EntryCreationAction,
    settlement: BilateralSettlement,
) -> ExternResult<ValidateCallbackResult> {
    // String length checks — prevent DHT bloat
    if settlement.debtor_dao_did.len() > MAX_DID_LEN
        || settlement.creditor_dao_did.len() > MAX_DID_LEN
    {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if settlement.id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement ID exceeds maximum length".into(),
        ));
    }

    // Amount must be positive
    if settlement.amount <= 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement amount must be positive".into(),
        ));
    }

    // DIDs must be valid
    if !settlement.debtor_dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Debtor DAO DID must be a valid DID".into(),
        ));
    }
    if !settlement.creditor_dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Creditor DAO DID must be a valid DID".into(),
        ));
    }

    // New settlements must start in Pending status
    if settlement.status != SettlementStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New settlements must start in Pending status".into(),
        ));
    }

    // ID must not be empty
    if settlement.id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement ID is required".into(),
        ));
    }

    if let ValidateCallbackResult::Invalid(msg) = validate_action_timestamp(
        "BilateralSettlement created_at",
        action.timestamp(),
        &settlement.created_at,
    ) {
        return Ok(ValidateCallbackResult::Invalid(msg));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_oracle_state_shape(state: &OracleState) -> ValidateCallbackResult {
    if state.vitality > 100 {
        return ValidateCallbackResult::Invalid("Vitality must be 0-100".into());
    }
    if !state.total_yield_kwh.is_finite() || state.total_yield_kwh < 0.0 {
        return ValidateCallbackResult::Invalid(
            "Total oracle yield must be finite and non-negative".into(),
        );
    }
    if state.tier != OracleState::tier_from_vitality(state.vitality) {
        return ValidateCallbackResult::Invalid(
            "OracleState tier must equal the tier derived from vitality".into(),
        );
    }
    ValidateCallbackResult::Valid
}

fn validate_action_timestamp(
    field: &str,
    action_timestamp: &Timestamp,
    entry_timestamp: &Timestamp,
) -> ValidateCallbackResult {
    if action_timestamp != entry_timestamp {
        return ValidateCallbackResult::Invalid(format!(
            "{field} must equal the Holochain action timestamp"
        ));
    }
    ValidateCallbackResult::Valid
}

fn validate_oracle_state_timestamp(
    action_timestamp: &Timestamp,
    state_timestamp: &Timestamp,
) -> ValidateCallbackResult {
    validate_action_timestamp(
        "OracleState updated_at",
        action_timestamp,
        state_timestamp,
    )
}

fn validate_create_oracle_state(
    action: EntryCreationAction,
    state: OracleState,
) -> ExternResult<ValidateCallbackResult> {
    let shape = validate_oracle_state_shape(&state);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(validate_oracle_state_timestamp(action.timestamp(), &state.updated_at))
}

fn validate_update_oracle_state(
    action: Update,
    state: OracleState,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<OracleState>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "OracleState predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "OracleState update predecessor is not an OracleState".into()
            ))
        })?;

    let shape = validate_oracle_state_shape(&state);
    if !matches!(shape, ValidateCallbackResult::Valid) {
        return Ok(shape);
    }
    Ok(validate_oracle_state_transition(&original, &state, &action.timestamp))
}

fn validate_oracle_state_transition(
    original: &OracleState,
    updated: &OracleState,
    action_timestamp: &Timestamp,
) -> ValidateCallbackResult {
    if updated.tier != OracleState::tier_from_vitality(updated.vitality) {
        return ValidateCallbackResult::Invalid(
            "OracleState tier must equal the tier derived from vitality".into(),
        );
    }
    if updated.updated_at < original.updated_at {
        return ValidateCallbackResult::Invalid(
            "OracleState updated_at cannot move backwards".into(),
        );
    }
    if !updated.total_yield_kwh.is_finite() || updated.total_yield_kwh < 0.0 {
        return ValidateCallbackResult::Invalid(
            "Total oracle yield must be finite and non-negative".into(),
        );
    }
    if updated.updated_at != *action_timestamp {
        return ValidateCallbackResult::Invalid(
            "OracleState updated_at must equal the Holochain action timestamp".into(),
        );
    }
    ValidateCallbackResult::Valid
}

fn validate_create_bilateral_balance(
    action: EntryCreationAction,
    bal: BilateralBalance,
) -> ExternResult<ValidateCallbackResult> {
    if bal.dao_a_did.len() > MAX_DID_LEN || bal.dao_b_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if !bal.dao_a_did.starts_with("did:") || !bal.dao_b_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO DIDs must be valid".into(),
        ));
    }
    if bal.dao_a_did >= bal.dao_b_did {
        return Ok(ValidateCallbackResult::Invalid(
            "dao_a_did must be alphabetically before dao_b_did (canonical ordering)".into(),
        ));
    }
    if bal.last_settled_at != *action.timestamp()
        || bal.last_updated_at != *action.timestamp()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "New BilateralBalance timestamps must equal the Holochain action timestamp".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_bilateral_balance(
    action: Update,
    balance: BilateralBalance,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<BilateralBalance>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "BilateralBalance predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "BilateralBalance update predecessor is not a BilateralBalance".into()
            ))
        })?;

    Ok(validate_bilateral_balance_state_transition(
        &original,
        &balance,
        &action.timestamp,
    ))
}

fn validate_bilateral_balance_state_transition(
    original: &BilateralBalance,
    updated: &BilateralBalance,
    action_timestamp: &Timestamp,
) -> ValidateCallbackResult {
    if updated.dao_a_did != original.dao_a_did {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance dao_a_did is immutable across updates".into(),
        );
    }
    if updated.dao_b_did != original.dao_b_did {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance dao_b_did is immutable across updates".into(),
        );
    }
    if updated.dao_a_did >= updated.dao_b_did {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance DAO identifiers must retain canonical ordering".into(),
        );
    }
    if !updated.dao_a_did.starts_with("did:")
        || !updated.dao_b_did.starts_with("did:")
    {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance DAO identifiers must remain valid DIDs".into(),
        );
    }
    if updated.total_exchanges < original.total_exchanges {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance total_exchanges cannot decrease".into(),
        );
    }
    if updated.last_settled_at < original.last_settled_at {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance last_settled_at cannot move backwards".into(),
        );
    }
    if updated.last_updated_at < original.last_updated_at {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance last_updated_at cannot move backwards".into(),
        );
    }
    if updated.last_updated_at != *action_timestamp {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance last_updated_at must equal the Holochain action timestamp".into(),
        );
    }
    if updated.last_settled_at > *action_timestamp {
        return ValidateCallbackResult::Invalid(
            "BilateralBalance last_settled_at cannot be in the future relative to the action"
                .into(),
        );
    }
    if updated.last_settled_at != original.last_settled_at
        && updated.last_settled_at != *action_timestamp
    {
        return ValidateCallbackResult::Invalid(
            "Changed BilateralBalance last_settled_at must equal the Holochain action timestamp"
                .into(),
        );
    }

    ValidateCallbackResult::Valid
}

fn validate_update_bilateral_settlement(
    action: Update,
    settlement: BilateralSettlement,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<BilateralSettlement>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "BilateralSettlement predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "BilateralSettlement update predecessor is not a BilateralSettlement".into()
            ))
        })?;

    Ok(validate_bilateral_settlement_transition(
        &original,
        &settlement,
        &action.timestamp,
    ))
}

fn validate_bilateral_settlement_transition(
    original: &BilateralSettlement,
    updated: &BilateralSettlement,
    action_timestamp: &Timestamp,
) -> ValidateCallbackResult {
    if original.status != SettlementStatus::Pending {
        return ValidateCallbackResult::Invalid(
            "Only a Pending settlement may transition to a terminal state".into(),
        );
    }

    match updated.status {
        SettlementStatus::Completed | SettlementStatus::Failed => {}
        SettlementStatus::Pending => {
            return ValidateCallbackResult::Invalid(
                "BilateralSettlement updates must use a terminal status".into(),
            );
        }
    }

    if updated.id != original.id
        || updated.debtor_dao_did != original.debtor_dao_did
        || updated.creditor_dao_did != original.creditor_dao_did
        || updated.amount != original.amount
        || updated.created_at != original.created_at
    {
        return ValidateCallbackResult::Invalid(
            "BilateralSettlement terms are immutable after creation".into(),
        );
    }

    if updated.amount <= 0 {
        return ValidateCallbackResult::Invalid(
            "Settlement amount must remain positive".into(),
        );
    }

    if !updated.debtor_dao_did.starts_with("did:")
        || !updated.creditor_dao_did.starts_with("did:")
    {
        return ValidateCallbackResult::Invalid(
            "Settlement DAO DIDs must remain valid".into(),
        );
    }

    let Some(completed_at) = updated.completed_at else {
        return ValidateCallbackResult::Invalid(
            "Terminal BilateralSettlement state requires completed_at".into(),
        );
    };

    if completed_at < original.created_at {
        return ValidateCallbackResult::Invalid(
            "completed_at cannot precede created_at".into(),
        );
    }

    if completed_at != *action_timestamp {
        return ValidateCallbackResult::Invalid(
            "BilateralSettlement completed_at must equal the Holochain action timestamp"
                .into(),
        );
    }

    ValidateCallbackResult::Valid
}

fn validate_create_hearth_balance(bal: HearthTendBalance) -> ExternResult<ValidateCallbackResult> {
    if bal.member_did.len() > MAX_DID_LEN || bal.hearth_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if !bal.member_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Member must be a valid DID".into(),
        ));
    }
    if !bal.hearth_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Hearth must be a valid DID".into(),
        ));
    }
    // Hearth credit limit is tighter than DAO
    if bal.balance.abs() > HEARTH_TEND_CREDIT_LIMIT {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Hearth TEND balance exceeds limit of ±{}",
            HEARTH_TEND_CREDIT_LIMIT
        )));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_hearth_balance(
    action: Update,
    bal: HearthTendBalance,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<HearthTendBalance>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "HearthTendBalance predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "HearthTendBalance update predecessor is not a HearthTendBalance".into()
            ))
        })?;

    Ok(validate_hearth_balance_state_transition(&original, &bal))
}

fn validate_hearth_balance_state_transition(
    original: &HearthTendBalance,
    updated: &HearthTendBalance,
) -> ValidateCallbackResult {
    if original.member_did != updated.member_did {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance member_did is immutable across updates".into(),
        );
    }
    if original.hearth_did != updated.hearth_did {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance hearth_did is immutable across updates".into(),
        );
    }
    if updated.total_provided < original.total_provided {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance total_provided cannot decrease".into(),
        );
    }
    if updated.total_received < original.total_received {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance total_received cannot decrease".into(),
        );
    }
    if updated.exchange_count < original.exchange_count {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance exchange_count cannot decrease".into(),
        );
    }
    if updated.last_activity < original.last_activity {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance last_activity cannot move backwards".into(),
        );
    }
    if updated.balance.abs() > HEARTH_TEND_CREDIT_LIMIT {
        return ValidateCallbackResult::Invalid(format!(
            "Hearth TEND balance would exceed limit of ±{}",
            HEARTH_TEND_CREDIT_LIMIT
        ));
    }
    if !updated.total_provided.is_finite() || !updated.total_received.is_finite() {
        return ValidateCallbackResult::Invalid(
            "HearthTendBalance totals must be finite numbers".into(),
        );
    }

    ValidateCallbackResult::Valid
}

fn validate_create_currency_alias(
    alias: CurrencyAliasEntry,
) -> ExternResult<ValidateCallbackResult> {
    if alias.dao_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if !alias.dao_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DAO must be a valid DID".into(),
        ));
    }
    if alias.alias_name.is_empty() || alias.alias_name.len() > MAX_CULTURAL_ALIAS_LEN {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Alias name must be 1-{} characters",
            MAX_CULTURAL_ALIAS_LEN
        )));
    }
    if let Some(ref sym) = alias.display_symbol {
        if sym.len() > 6 {
            return Ok(ValidateCallbackResult::Invalid(
                "Display symbol must be 1-6 characters".into(),
            ));
        }
    }
    if let Some(ref desc) = alias.description {
        if desc.len() > MAX_DESCRIPTION_LEN {
            return Ok(ValidateCallbackResult::Invalid(
                "Description exceeds maximum length".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_pending_balance_adjustment(
    action: EntryCreationAction,
    adj: PendingBalanceAdjustment,
) -> ExternResult<ValidateCallbackResult> {
    if !adj.hours.is_finite() || adj.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "PendingBalanceAdjustment hours must be finite and positive".into(),
        ));
    }

    if adj.provider_completed || adj.receiver_completed {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claims must start with both completion flags false".into(),
        ));
    }

    if adj.provider_did.len() > MAX_DID_LEN || adj.receiver_did.len() > MAX_DID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "DID exceeds maximum length".into(),
        ));
    }
    if adj.exchange_id.len() > MAX_ID_LEN || adj.currency_id.len() > MAX_ID_LEN {
        return Ok(ValidateCallbackResult::Invalid(
            "Identifier exceeds maximum length".into(),
        ));
    }

    if !adj.provider_did.starts_with("did:")
        || !adj.receiver_did.starts_with("did:")
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider and receiver must be valid DIDs".into(),
        ));
    }

    let exchange_record = must_get_valid_record(adj.exchange_action_hash.clone())?;
    if !matches!(exchange_record.action(), Action::Create(_)) {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claim must target the original TendExchange create action".into(),
        ));
    }

    let exchange = exchange_record
        .entry()
        .to_app_option::<TendExchange>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Settlement claim exchange deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Settlement claim target is not a TendExchange entry".into()
            ))
        })?;

    if exchange.status != ExchangeStatus::Proposed {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claim target must be a Proposed exchange".into(),
        ));
    }

    if exchange.id != adj.exchange_id
        || exchange.provider_did != adj.provider_did
        || exchange.receiver_did != adj.receiver_did
        || exchange.hours != adj.hours as f32
        || exchange.dao_did != adj.currency_id
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claim terms do not exactly match the exchange".into(),
        ));
    }

    let claim_author_did = did_for_author(action.author());
    if claim_author_did != adj.receiver_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the exchange receiver may create its settlement claim".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_pending_balance_adjustment(
    action: Update,
    adj: PendingBalanceAdjustment,
) -> ExternResult<ValidateCallbackResult> {
    if !adj.hours.is_finite() || adj.hours <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "PendingBalanceAdjustment hours must be finite and positive".into(),
        ));
    }
    if adj.provider_completed && !adj.receiver_completed {
        // This is the only intermediate state produced by the settlement protocol.
    } else if !adj.provider_completed && adj.receiver_completed {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver completion cannot precede provider completion".into(),
        ));
    }

    let original_record = must_get_valid_record(action.original_action_address)?;
    let original = original_record
        .entry()
        .to_app_option::<PendingBalanceAdjustment>()
        .map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "PendingBalanceAdjustment predecessor deserialization error: {:?}",
                e
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "PendingBalanceAdjustment predecessor is not a PendingBalanceAdjustment".into()
            ))
        })?;

    if original.exchange_id != adj.exchange_id
        || original.exchange_action_hash != adj.exchange_action_hash
        || original.provider_did != adj.provider_did
        || original.receiver_did != adj.receiver_did
        || original.hours != adj.hours
        || original.currency_id != adj.currency_id
        || original.created_at != adj.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claim binding fields are immutable".into(),
        ));
    }

    if original.provider_completed && !adj.provider_completed {
        return Ok(ValidateCallbackResult::Invalid(
            "Provider settlement completion cannot be reverted".into(),
        ));
    }
    if original.receiver_completed && !adj.receiver_completed {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver settlement completion cannot be reverted".into(),
        ));
    }
    if adj.receiver_completed && !adj.provider_completed {
        return Ok(ValidateCallbackResult::Invalid(
            "Receiver completion requires provider completion".into(),
        ));
    }

    let original_author = original_record.action().author();
    if action.author() != original_author {
        return Ok(ValidateCallbackResult::Invalid(
            "Settlement claim may only be updated by its original author".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_dispute_case(
    _action: Update,
    dispute: DisputeCase,
) -> ExternResult<ValidateCallbackResult> {
    // Validate core DID fields remain valid
    if !dispute.complainant_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Complainant must be a valid DID".into(),
        ));
    }
    if !dispute.respondent_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Respondent must be a valid DID".into(),
        ));
    }

    // Validate mediator DIDs
    for mediator_did in &dispute.mediator_dids {
        if !mediator_did.starts_with("did:") {
            return Ok(ValidateCallbackResult::Invalid(
                "All mediator DIDs must be valid".into(),
            ));
        }
    }

    // MediationPanel stage requires exactly 3 mediators
    if dispute.stage == DisputeStage::MediationPanel && dispute.mediator_dids.len() != 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "MediationPanel stage requires exactly 3 mediators".into(),
        ));
    }

    // Description must remain non-empty
    if dispute.description.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Dispute description is required".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn ts(micros: i64) -> Timestamp {
        Timestamp::from_micros(micros)
    }

    fn make_create() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(1_000_000),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    /// DID of the agent `make_create()`/`make_update()` attribute actions to.
    /// Fixtures meant to be VALID must use this — quality ratings and dispute
    /// cases bind their reporter DID to the committing agent.
    fn test_author_did() -> String {
        format!("did:mycelix:{}", AgentPubKey::from_raw_36(vec![0; 36]))
    }

    fn make_update() -> Update {
        Update {
            author: AgentPubKey::from_raw_36(vec![0; 36]),
            timestamp: ts(2_000_000),
            action_seq: 1,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            original_action_address: ActionHash::from_raw_36(vec![0; 36]),
            original_entry_address: EntryHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    fn valid_listing() -> ServiceListing {
        ServiceListing {
            id: "listing:test:001".into(),
            provider_did: test_author_did(),
            dao_did: "did:mycelix:dao".into(),
            title: "Garden help".into(),
            description: "Weeding and planting".into(),
            category: ServiceCategory::HomeServices,
            estimated_hours: Some(2.0),
            availability: Some("weekends".into()),
            active: true,
            created: ts(1_000_000),
        }
    }

    fn valid_request() -> ServiceRequest {
        ServiceRequest {
            id: "request:test:001".into(),
            requester_did: test_author_did(),
            dao_did: "did:mycelix:dao".into(),
            title: "Need a ride".into(),
            description: "Airport pickup".into(),
            category: ServiceCategory::Transportation,
            estimated_hours: Some(1.5),
            urgency: Urgency::Flexible,
            open: true,
            created: ts(1_000_000),
        }
    }

    fn valid_exchange() -> TendExchange {
        TendExchange {
            id: "exch:001".into(),
            provider_did: "did:mycelix:alice".into(),
            receiver_did: "did:mycelix:bob".into(),
            hours: 2.0,
            service_description: "Tutoring session".into(),
            service_category: ServiceCategory::Education,
            cultural_alias: None,
            dao_did: "did:mycelix:dao1".into(),
            timestamp: ts(1_000_000),
            status: ExchangeStatus::Proposed,
            service_date: None,
        }
    }

    fn valid_initial_balance() -> TendBalance {
        TendBalance {
            member_did: "did:mycelix:alice".into(),
            dao_did: "did:mycelix:dao1".into(),
            balance: 0,
            total_provided: 0.0,
            total_received: 0.0,
            exchange_count: 0,
            last_activity: ts(1_000_000),
        }
    }

    fn valid_balance() -> TendBalance {
        TendBalance {
            member_did: "did:mycelix:alice".into(),
            dao_did: "did:mycelix:dao1".into(),
            balance: 5,
            total_provided: 10.0,
            total_received: 5.0,
            exchange_count: 3,
            last_activity: ts(1_000_000),
        }
    }

    #[test]
    fn balance_state_transition_accepts_identity_and_monotonic_history() {
        let original = valid_balance();
        let mut updated = original.clone();
        updated.balance = 7;
        updated.total_provided = 12.0;
        updated.total_received = 5.0;
        updated.exchange_count = 4;
        updated.last_activity = ts(2_000_000);

        assert!(matches!(
            validate_balance_state_transition(&original, &updated),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn balance_state_transition_rejects_member_identity_swap() {
        let original = valid_balance();
        let mut updated = original.clone();
        updated.member_did = "did:mycelix:attacker".into();

        assert!(matches!(
            validate_balance_state_transition(&original, &updated),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn balance_state_transition_rejects_dao_identity_swap() {
        let original = valid_balance();
        let mut updated = original.clone();
        updated.dao_did = "did:mycelix:other-dao".into();

        assert!(matches!(
            validate_balance_state_transition(&original, &updated),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn balance_state_transition_rejects_history_regression() {
        let original = valid_balance();

        let mut provided_regresses = original.clone();
        provided_regresses.total_provided = original.total_provided - 0.5;
        assert!(matches!(
            validate_balance_state_transition(&original, &provided_regresses),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut received_regresses = original.clone();
        received_regresses.total_received = original.total_received - 0.5;
        assert!(matches!(
            validate_balance_state_transition(&original, &received_regresses),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut count_regresses = original.clone();
        count_regresses.exchange_count = original.exchange_count - 1;
        assert!(matches!(
            validate_balance_state_transition(&original, &count_regresses),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut time_regresses = original.clone();
        time_regresses.last_activity = ts(500_000);
        assert!(matches!(
            validate_balance_state_transition(&original, &time_regresses),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    fn valid_rating() -> QualityRating {
        QualityRating {
            exchange_id: "exch:001".into(),
            rater_did: test_author_did(),
            provider_did: "did:mycelix:alice".into(),
            rating: 4,
            comment: None,
            timestamp: ts(2_000_000),
        }
    }

    fn valid_dispute() -> DisputeCase {
        DisputeCase {
            id: "dispute:001".into(),
            exchange_id: "exch:001".into(),
            complainant_did: test_author_did(),
            respondent_did: "did:mycelix:alice".into(),
            stage: DisputeStage::DirectNegotiation,
            description: "Service not as described".into(),
            mediator_dids: vec![],
            resolution: None,
            opened_at: ts(2_000_000),
            escalated_at: None,
            resolved_at: None,
        }
    }

    fn valid_bilateral_settlement() -> BilateralSettlement {
        BilateralSettlement {
            id: "settle:001".into(),
            debtor_dao_did: "did:mycelix:dao-a".into(),
            creditor_dao_did: "did:mycelix:dao-b".into(),
            amount: 10,
            status: SettlementStatus::Pending,
            created_at: ts(1_000_000),
            completed_at: None,
        }
    }

    #[test]
    fn bilateral_settlement_transition_accepts_pending_to_completed() {
        let original = valid_bilateral_settlement();
        let mut updated = original.clone();
        updated.status = SettlementStatus::Completed;
        updated.completed_at = Some(ts(2_000_000));

        assert!(matches!(
            validate_bilateral_settlement_transition(
                &original,
                &updated,
                &updated.completed_at.unwrap_or(ts(0)),
            ),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn bilateral_settlement_transition_rejects_term_swaps() {
        let original = valid_bilateral_settlement();

        for mutate in [
            |x: &mut BilateralSettlement| x.id = "attacker".into(),
            |x: &mut BilateralSettlement| x.debtor_dao_did = "did:mycelix:attacker".into(),
            |x: &mut BilateralSettlement| x.creditor_dao_did = "did:mycelix:attacker".into(),
            |x: &mut BilateralSettlement| x.amount = 99,
            |x: &mut BilateralSettlement| x.created_at = ts(2_000_000),
        ] {
            let mut updated = original.clone();
            mutate(&mut updated);
            updated.status = SettlementStatus::Completed;
            updated.completed_at = Some(ts(2_000_000));

            assert!(matches!(
                validate_bilateral_settlement_transition(
                    &original,
                    &updated,
                    &updated.completed_at.unwrap_or(ts(0)),
                ),
                ValidateCallbackResult::Invalid(_)
            ));
        }
    }

    #[test]
    fn bilateral_settlement_transition_rejects_invalid_terminal_state() {
        let original = valid_bilateral_settlement();

        let mut missing_time = original.clone();
        missing_time.status = SettlementStatus::Completed;
        assert!(matches!(
            validate_bilateral_settlement_transition(&original, &missing_time, &ts(2_000_000)),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut backwards_time = original.clone();
        backwards_time.status = SettlementStatus::Failed;
        backwards_time.completed_at = Some(ts(500_000));
        assert!(matches!(
            validate_bilateral_settlement_transition(&original, &backwards_time, &ts(500_000)),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn bilateral_settlement_transition_rejects_repeated_terminal_transition() {
        let mut original = valid_bilateral_settlement();
        original.status = SettlementStatus::Completed;
        original.completed_at = Some(ts(2_000_000));

        let mut updated = original.clone();
        updated.completed_at = Some(ts(3_000_000));

        assert!(matches!(
            validate_bilateral_settlement_transition(
                &original,
                &updated,
                &updated.completed_at.unwrap_or(ts(0)),
            ),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn bilateral_balance_create_requires_action_timestamps() {
        let action_time = ts(2_000_000);
        let mut action = make_create();
        action.timestamp = action_time;
        let mut balance = valid_bilateral_balance();
        balance.last_settled_at = action_time;
        balance.last_updated_at = action_time;

        assert!(matches!(
            validate_create_bilateral_balance(
                EntryCreationAction::Create(action),
                balance,
            ),
            Ok(ValidateCallbackResult::Valid)
        ));

        let mut future_balance = valid_bilateral_balance();
        future_balance.last_settled_at = ts(2_000_001);
        future_balance.last_updated_at = action_time;
        let mut future_action = make_create();
        future_action.timestamp = action_time;
        assert!(matches!(
            validate_create_bilateral_balance(
                EntryCreationAction::Create(future_action),
                future_balance,
            ),
            Ok(ValidateCallbackResult::Invalid(_))
        ));
    }

    #[test]
    fn bilateral_settlement_create_requires_action_timestamp() {
        let action_time = ts(2_000_000);
        let mut action = make_create();
        action.timestamp = action_time;
        let mut settlement = valid_settlement();
        settlement.created_at = action_time;

        assert!(matches!(
            validate_create_bilateral_settlement(
                EntryCreationAction::Create(action),
                settlement,
            ),
            Ok(ValidateCallbackResult::Valid)
        ));

        let mut future = valid_settlement();
        future.created_at = ts(2_000_001);
        let mut future_action = make_create();
        future_action.timestamp = action_time;
        assert!(matches!(
            validate_create_bilateral_settlement(
                EntryCreationAction::Create(future_action),
                future,
            ),
            Ok(ValidateCallbackResult::Invalid(_))
        ));
    }

    #[test]
    fn bilateral_settlement_completion_requires_action_timestamp() {
        let original = valid_settlement();
        let mut updated = original.clone();
        updated.status = SettlementStatus::Completed;

        for completed_at in [ts(1_999_999), ts(2_000_001)] {
            updated.completed_at = Some(completed_at);
            assert!(matches!(
                validate_bilateral_settlement_transition(
                    &original,
                    &updated,
                    &ts(2_000_000),
                ),
                ValidateCallbackResult::Invalid(_)
            ));
        }

        updated.completed_at = Some(ts(2_000_000));
        assert!(matches!(
            validate_bilateral_settlement_transition(
                &original,
                &updated,
                &ts(2_000_000),
            ),
            ValidateCallbackResult::Valid
        ));
    }

    fn valid_hearth_balance() -> HearthTendBalance {
        HearthTendBalance {
            member_did: "did:mycelix:alice".into(),
            hearth_did: "did:mycelix:hearth1".into(),
            balance: 5,
            total_provided: 10.0,
            total_received: 5.0,
            exchange_count: 2,
            last_activity: ts(1_000_000),
        }
    }

    #[test]
    fn hearth_balance_transition_accepts_identity_and_monotonic_history() {
        let original = valid_hearth_balance();
        let mut updated = original.clone();
        updated.balance = 7;
        updated.total_provided = 12.0;
        updated.total_received = 5.0;
        updated.exchange_count = 3;
        updated.last_activity = ts(2_000_000);

        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &updated),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn hearth_balance_transition_rejects_identity_swaps() {
        let original = valid_hearth_balance();

        let mut member_swap = original.clone();
        member_swap.member_did = "did:mycelix:attacker".into();
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &member_swap),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut hearth_swap = original.clone();
        hearth_swap.hearth_did = "did:mycelix:other-hearth".into();
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &hearth_swap),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn hearth_balance_transition_rejects_history_regression() {
        let original = valid_hearth_balance();

        let mut provided = original.clone();
        provided.total_provided = 9.0;
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &provided),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut received = original.clone();
        received.total_received = 4.0;
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &received),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut count = original.clone();
        count.exchange_count = 1;
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &count),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut time = original.clone();
        time.last_activity = ts(500_000);
        assert!(matches!(
            validate_hearth_balance_state_transition(&original, &time),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    fn valid_oracle_state() -> OracleState {
        OracleState {
            vitality: 50,
            total_yield_kwh: 100.0,
            tier: OracleState::tier_from_vitality(50),
            updated_at: ts(1_000_000),
        }
    }

    #[test]
    fn oracle_state_timestamp_accepts_exact_action_time() {
        let action_time = ts(2_000_000);
        assert!(matches!(
            validate_oracle_state_timestamp(&action_time, &action_time),
            ValidateCallbackResult::Valid
        ));

        let mut state = valid_oracle_state();
        state.updated_at = action_time;
        let mut action = make_create();
        action.timestamp = action_time;
        assert!(matches!(
            validate_create_oracle_state(EntryCreationAction::Create(action), state),
            Ok(ValidateCallbackResult::Valid)
        ));
    }

    #[test]
    fn oracle_state_timestamp_rejects_past_or_future_time() {
        let action_time = ts(2_000_000);
        for state_time in [ts(1_999_999), ts(2_000_001)] {
            assert!(matches!(
                validate_oracle_state_timestamp(&action_time, &state_time),
                ValidateCallbackResult::Invalid(_)
            ));

            let mut state = valid_oracle_state();
            state.updated_at = state_time;
            let mut action = make_create();
            action.timestamp = action_time;
            assert!(matches!(
                validate_create_oracle_state(EntryCreationAction::Create(action), state),
                Ok(ValidateCallbackResult::Invalid(_))
            ));
        }
    }

    #[test]
    fn oracle_state_transition_accepts_derived_tier_and_monotonic_time() {
        let original = valid_oracle_state();
        let mut updated = original.clone();
        updated.vitality = 20;
        updated.total_yield_kwh = 120.0;
        updated.tier = OracleState::tier_from_vitality(updated.vitality);
        updated.updated_at = ts(2_000_000);

        assert!(matches!(
            validate_oracle_state_transition(&original, &updated, &updated.updated_at),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn oracle_state_transition_rejects_tier_mismatch() {
        let original = valid_oracle_state();
        let mut updated = original.clone();
        updated.vitality = 10;
        updated.updated_at = ts(2_000_000);

        assert!(matches!(
            validate_oracle_state_transition(&original, &updated, &updated.updated_at),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn oracle_state_transition_rejects_time_regression() {
        let original = valid_oracle_state();
        let mut updated = original.clone();
        updated.updated_at = ts(500_000);

        assert!(matches!(
            validate_oracle_state_transition(&original, &updated, &updated.updated_at),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn oracle_state_create_rejects_invalid_yield() {
        let mut invalid_yield = valid_oracle_state();
        invalid_yield.total_yield_kwh = -1.0;

        assert!(matches!(
            validate_oracle_state_shape(&invalid_yield),
            Ok(ValidateCallbackResult::Invalid(_))
        ));
    }

    fn valid_bilateral_balance() -> BilateralBalance {
        BilateralBalance {
            dao_a_did: "did:mycelix:dao-a".into(),
            dao_b_did: "did:mycelix:dao-b".into(),
            net_balance: 10,
            total_exchanges: 3,
            last_settled_at: ts(1_000_000),
            last_updated_at: ts(2_000_000),
        }
    }

    #[test]
    fn bilateral_balance_transition_accepts_monotonic_update() {
        let original = valid_bilateral_balance();
        let mut updated = original.clone();
        updated.net_balance = 5;
        updated.total_exchanges = 4;
        updated.last_settled_at = ts(3_000_000);
        updated.last_updated_at = ts(3_000_000);

        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &updated,
                &updated.last_updated_at,
            ),
            ValidateCallbackResult::Valid
        ));
    }

    #[test]
    fn bilateral_balance_transition_rejects_identity_swaps() {
        let original = valid_bilateral_balance();

        let mut swap_a = original.clone();
        swap_a.dao_a_did = "did:mycelix:attacker".into();
        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &swap_a,
                &swap_a.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut swap_b = original.clone();
        swap_b.dao_b_did = "did:mycelix:attacker".into();
        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &swap_b,
                &swap_b.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn bilateral_balance_transition_rejects_history_regression() {
        let original = valid_bilateral_balance();

        let mut exchanges = original.clone();
        exchanges.total_exchanges = 2;
        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &exchanges,
                &exchanges.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut settled = original.clone();
        settled.last_settled_at = ts(500_000);
        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &settled,
                &settled.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut updated = original.clone();
        updated.last_updated_at = ts(1_500_000);
        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &updated,
                &updated.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn bilateral_balance_transition_rejects_noncanonical_dao_order() {
        let original = valid_bilateral_balance();
        let mut updated = original.clone();
        updated.dao_a_did = "did:mycelix:zeta".into();
        updated.dao_b_did = "did:mycelix:alpha".into();

        assert!(matches!(
            validate_bilateral_balance_state_transition(
                &original,
                &updated,
                &updated.last_updated_at,
            ),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    fn valid_settlement() -> BilateralSettlement {
        BilateralSettlement {
            id: "settle:001".into(),
            debtor_dao_did: "did:mycelix:dao_a".into(),
            creditor_dao_did: "did:mycelix:dao_b".into(),
            amount: 10,
            status: SettlementStatus::Pending,
            created_at: ts(1_000_000),
            completed_at: None,
        }
    }

    fn valid_alias() -> CurrencyAliasEntry {
        CurrencyAliasEntry {
            dao_did: "did:mycelix:dao1".into(),
            alias_name: "CARE".into(),
            base_currency: Currency::Tend,
            display_symbol: Some("C".into()),
            description: Some("Community care hours".into()),
            created_at: ts(1_000_000),
        }
    }

    // ---- Exchange creation ----

    #[test]
    fn test_exchange_create_valid() {
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), valid_exchange())
                .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_exchange_rejects_invalid_provider_did() {
        let mut ex = valid_exchange();
        ex.provider_did = "not-a-did".into();
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_invalid_receiver_did() {
        let mut ex = valid_exchange();
        ex.receiver_did = "bad".into();
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_self_exchange() {
        let mut ex = valid_exchange();
        ex.receiver_did = ex.provider_did.clone();
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_nan_hours() {
        let mut ex = valid_exchange();
        ex.hours = f32::NAN;
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_zero_hours() {
        let mut ex = valid_exchange();
        ex.hours = 0.0;
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_negative_hours() {
        let mut ex = valid_exchange();
        ex.hours = -1.0;
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_infinity_hours() {
        let mut ex = valid_exchange();
        ex.hours = f32::INFINITY;
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_exceeding_max_hours() {
        let mut ex = valid_exchange();
        ex.hours = 9.0; // MAX_SERVICE_HOURS is 8
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_below_min_minutes() {
        let mut ex = valid_exchange();
        ex.hours = 0.1; // 6 minutes, MIN_SERVICE_MINUTES is 15
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_empty_description() {
        let mut ex = valid_exchange();
        ex.service_description = "".into();
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_overlong_description() {
        let mut ex = valid_exchange();
        ex.service_description = "x".repeat(2001);
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_exchange_rejects_overlong_cultural_alias() {
        let mut ex = valid_exchange();
        ex.cultural_alias = Some("x".repeat(65)); // MAX_CULTURAL_ALIAS_LEN is 64
        let result =
            validate_create_exchange(EntryCreationAction::Create(make_create()), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Exchange update ----

    #[test]
    fn test_exchange_update_rejects_nan_hours() {
        let mut ex = valid_exchange();
        ex.hours = f32::NAN;
        let result = validate_update_exchange(make_update(), ex).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Balance creation ----

    #[test]
    fn test_balance_rejects_preseeded_value_or_history() {
        let mut bal = valid_initial_balance();
        bal.balance = 1;
        assert!(matches!(
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));

        let mut historic = valid_initial_balance();
        historic.total_provided = 1.0;
        historic.exchange_count = 1;
        assert!(matches!(
            validate_create_balance(EntryCreationAction::Create(make_create()), historic).unwrap(),
            ValidateCallbackResult::Invalid(_)
        ));
    }

    #[test]
    fn test_balance_create_valid() {
        let result =
            validate_create_balance(
                EntryCreationAction::Create(make_create()),
                valid_initial_balance(),
            )
            .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_balance_rejects_invalid_member_did() {
        let mut bal = valid_balance();
        bal.member_did = "nope".into();
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_rejects_invalid_dao_did() {
        let mut bal = valid_balance();
        bal.dao_did = "nope".into();
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_rejects_nan_total_provided() {
        let mut bal = valid_balance();
        bal.total_provided = f32::NAN;
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_rejects_inf_total_received() {
        let mut bal = valid_balance();
        bal.total_received = f32::INFINITY;
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_rejects_exceeding_emergency_limit() {
        let mut bal = valid_balance();
        bal.balance = 121; // BALANCE_LIMIT_EMERGENCY is 120
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_rejects_negative_exceeding_emergency_limit() {
        let mut bal = valid_balance();
        bal.balance = -121;
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_allows_emergency_limit_exactly() {
        let mut bal = valid_balance();
        bal.balance = 120;
        let result =
            validate_create_balance(EntryCreationAction::Create(make_create()), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- Balance update ----

    #[test]
    fn test_balance_update_rejects_nan_provided() {
        let mut bal = valid_balance();
        bal.total_provided = f32::NAN;
        let result = validate_update_balance(make_update(), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_balance_update_rejects_over_limit() {
        let mut bal = valid_balance();
        bal.balance = 121;
        let result = validate_update_balance(make_update(), bal).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Quality rating ----

    #[test]
    fn test_rating_create_valid() {
        let result = validate_create_quality_rating(
            EntryCreationAction::Create(make_create()),
            valid_rating(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_rating_rejects_self_rating() {
        let mut r = valid_rating();
        r.provider_did = r.rater_did.clone();
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_rating_rejects_zero_score() {
        let mut r = valid_rating();
        r.rating = 0;
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_rating_rejects_above_five() {
        let mut r = valid_rating();
        r.rating = 6;
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_rating_rejects_empty_exchange_id() {
        let mut r = valid_rating();
        r.exchange_id = "".into();
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_rating_rejects_invalid_rater_did() {
        let mut r = valid_rating();
        r.rater_did = "bad".into();
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Dispute creation ----

    #[test]
    fn test_dispute_create_valid() {
        let result = validate_create_dispute_case(
            EntryCreationAction::Create(make_create()),
            valid_dispute(),
        )
        .unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_dispute_rejects_self_dispute() {
        let mut d = valid_dispute();
        d.respondent_did = d.complainant_did.clone();
        let result =
            validate_create_dispute_case(EntryCreationAction::Create(make_create()), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_dispute_rejects_empty_description() {
        let mut d = valid_dispute();
        d.description = "".into();
        let result =
            validate_create_dispute_case(EntryCreationAction::Create(make_create()), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_dispute_rejects_empty_exchange_id() {
        let mut d = valid_dispute();
        d.exchange_id = "".into();
        let result =
            validate_create_dispute_case(EntryCreationAction::Create(make_create()), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_dispute_must_start_at_direct_negotiation() {
        let mut d = valid_dispute();
        d.stage = DisputeStage::MediationPanel;
        d.mediator_dids = vec![
            "did:mycelix:m1".into(),
            "did:mycelix:m2".into(),
            "did:mycelix:m3".into(),
        ];
        let result =
            validate_create_dispute_case(EntryCreationAction::Create(make_create()), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Dispute update ----

    #[test]
    fn test_dispute_update_mediation_requires_3_mediators() {
        let mut d = valid_dispute();
        d.stage = DisputeStage::MediationPanel;
        d.mediator_dids = vec!["did:mycelix:m1".into(), "did:mycelix:m2".into()];
        let result = validate_update_dispute_case(make_update(), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_dispute_update_mediation_valid_with_3() {
        let mut d = valid_dispute();
        d.stage = DisputeStage::MediationPanel;
        d.mediator_dids = vec![
            "did:mycelix:m1".into(),
            "did:mycelix:m2".into(),
            "did:mycelix:m3".into(),
        ];
        let result = validate_update_dispute_case(make_update(), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_dispute_update_rejects_invalid_mediator_did() {
        let mut d = valid_dispute();
        d.stage = DisputeStage::MediationPanel;
        d.mediator_dids = vec![
            "did:mycelix:m1".into(),
            "bad-did".into(),
            "did:mycelix:m3".into(),
        ];
        let result = validate_update_dispute_case(make_update(), d).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Hearth balance ----

    #[test]
    fn test_hearth_balance_create_valid() {
        let result = validate_create_hearth_balance(valid_hearth_balance()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_hearth_balance_rejects_invalid_member_did() {
        let mut hb = valid_hearth_balance();
        hb.member_did = "nope".into();
        let result = validate_create_hearth_balance(hb).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_hearth_balance_rejects_invalid_hearth_did() {
        let mut hb = valid_hearth_balance();
        hb.hearth_did = "nope".into();
        let result = validate_create_hearth_balance(hb).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_hearth_balance_rejects_exceeding_credit_limit() {
        let mut hb = valid_hearth_balance();
        hb.balance = 21; // HEARTH_TEND_CREDIT_LIMIT is 20
        let result = validate_create_hearth_balance(hb).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_hearth_balance_update_rejects_exceeding_limit() {
        let mut hb = valid_hearth_balance();
        hb.balance = -21;
        let result = validate_update_hearth_balance(hb).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ---- Bilateral settlement ----

    #[test]
    fn test_settlement_create_valid() {
        let result = validate_create_bilateral_settlement(
            EntryCreationAction::Create(make_create()),
            valid_settlement(),
        ).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_settlement_rejects_zero_amount() {
        let mut s = valid_settlement();
        s.amount = 0;
        let result = validate_create_bilateral_settlement(
            EntryCreationAction::Create(make_create()),
            s,
        ).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_settlement_rejects_non_pending_status() {
        let mut s = valid_settlement();
        s.status = SettlementStatus::Completed;
        let result = validate_create_bilateral_settlement(
            EntryCreationAction::Create(make_create()),
            s,
        ).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_settlement_update_rejects_pending_status() {
        let mut s = valid_settlement();
        s.status = SettlementStatus::Pending;
        let result = validate_update_bilateral_settlement(s).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_settlement_update_allows_completed() {
        let mut s = valid_settlement();
        s.status = SettlementStatus::Completed;
        let result = validate_update_bilateral_settlement(s).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    // ---- Currency alias ----

    #[test]
    fn test_alias_create_valid() {
        let result = validate_create_currency_alias(valid_alias()).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_alias_rejects_empty_name() {
        let mut a = valid_alias();
        a.alias_name = "".into();
        let result = validate_create_currency_alias(a).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_alias_rejects_overlong_symbol() {
        let mut a = valid_alias();
        a.display_symbol = Some("TOOLONG!".into()); // > 6 chars
        let result = validate_create_currency_alias(a).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_alias_rejects_invalid_dao_did() {
        let mut a = valid_alias();
        a.dao_did = "nope".into();
        let result = validate_create_currency_alias(a).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_quality_rating_forged_rater_is_rejected() {
        let mut r = valid_rating();
        r.rater_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_quality_rating(EntryCreationAction::Create(make_create()), r).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged rater_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_dispute_forged_complainant_is_rejected() {
        let mut d = valid_dispute();
        d.complainant_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_dispute_case(EntryCreationAction::Create(make_create()), d).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged complainant_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_listing_forged_provider_is_rejected() {
        let mut l = valid_listing();
        l.provider_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_listing(EntryCreationAction::Create(make_create()), l).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged provider_did must be rejected, got {other:?}"),
        }
    }

    #[test]
    fn test_request_forged_requester_is_rejected() {
        let mut r = valid_request();
        r.requester_did = "did:mycelix:uhCAkSomeoneElse".into();
        let result =
            validate_create_request(EntryCreationAction::Create(make_create()), r).unwrap();
        match result {
            ValidateCallbackResult::Invalid(msg) => {
                assert!(msg.contains("forgery"), "got: {msg}")
            }
            other => panic!("forged requester_did must be rejected, got {other:?}"),
        }
    }
}
