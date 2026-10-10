// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! P2P Grid Trading Integrity Zome
use hdi::prelude::*;
use holo_hash::blake2b_256;

/// Anchor entry for deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EnergyProduction {
    pub id: String,
    pub producer_did: String,
    pub project_id: String,
    pub amount_kwh: f64,
    pub timestamp: Timestamp,
    pub period_hours: f64,
    pub meter_reading: Option<f64>,
    pub verified: bool,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct EnergyConsumption {
    pub id: String,
    pub consumer_did: String,
    pub amount_kwh: f64,
    pub timestamp: Timestamp,
    pub period_hours: f64,
    pub meter_reading: Option<f64>,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct TradeOffer {
    pub id: String,
    pub seller_did: String,
    pub project_id: Option<String>,
    pub amount_kwh: f64,
    pub price_per_kwh: f64,
    pub currency: String,
    pub available_from: Timestamp,
    pub available_until: Timestamp,
    pub status: OfferStatus,
    pub created: Timestamp,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum OfferStatus {
    Active,
    PartiallyFilled,
    Filled,
    Expired,
    Cancelled,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Trade {
    pub id: String,
    pub offer_id: String,
    pub seller_did: String,
    pub buyer_did: String,
    pub amount_kwh: f64,
    pub price_per_kwh: f64,
    pub total_price: f64,
    pub currency: String,
    pub executed: Timestamp,
    pub settled: bool,
    pub payment_reference: Option<String>,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    EnergyProduction(EnergyProduction),
    EnergyConsumption(EnergyConsumption),
    TradeOffer(TradeOffer),
    Trade(Trade),
}

#[hdk_link_types]
pub enum LinkTypes {
    ProducerToProduction,
    ConsumerToConsumption,
    SellerToOffers,
    ActiveOffers,
    OfferToTrades,
    BuyerToTrades,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Validate a new app entry and the action that publishes it. Shared by
/// StoreEntry and StoreRecord so the entry and action authorities enforce the
/// same structural and identity invariants.
fn validate_create_app_entry(
    app_entry: EntryTypes,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::EnergyProduction(production) => {
            validate_create_energy_production(EntryCreationAction::Create(action), production)
        }
        EntryTypes::EnergyConsumption(consumption) => {
            validate_create_energy_consumption(EntryCreationAction::Create(action), consumption)
        }
        EntryTypes::TradeOffer(offer) => {
            validate_create_trade_offer(EntryCreationAction::Create(action), offer)
        }
        EntryTypes::Trade(trade) => {
            validate_create_trade(EntryCreationAction::Create(action), trade)
        }
    }
}

/// Validate an app-entry update through every DHT operation that carries it.
fn validate_update_app_entry(
    app_entry: EntryTypes,
    action: Update,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::EnergyProduction(_) => Ok(ValidateCallbackResult::Invalid(
            "Production records cannot be updated".into(),
        )),
        EntryTypes::EnergyConsumption(_) => Ok(ValidateCallbackResult::Invalid(
            "Consumption records cannot be updated".into(),
        )),
        EntryTypes::TradeOffer(offer) => validate_update_trade_offer(action, offer),
        EntryTypes::Trade(trade) => validate_update_trade(action, trade),
    }
}

/// Main validation callback. Holochain sends each authored action through
/// multiple DHT operations, including StoreRecord, StoreEntry, and
/// RegisterUpdate. Enforce the same entry invariants on every path carrying
/// the app entry, rather than letting the action authority accept a record
/// that the entry authority rejects.
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => {
                validate_create_app_entry(app_entry, action)
            }
            OpEntry::UpdateEntry { app_entry, action, .. } => {
                validate_update_app_entry(app_entry, action)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(record) => match record {
            OpRecord::CreateEntry { app_entry, action } => {
                validate_create_app_entry(app_entry, action)
            }
            OpRecord::UpdateEntry { app_entry, action, .. } => {
                validate_update_app_entry(app_entry, action)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => match update {
            OpUpdate::Entry { app_entry, action } => {
                validate_update_app_entry(app_entry, action)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            action,
            ..
        } => {
            let target_action_hash = match target_address.clone().into_action_hash() {
                Some(hash) => hash,
                None => {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Grid index links must target an action hash".into(),
                    ));
                }
            };
            let target_record = must_get_valid_record(target_action_hash)?;
            // AppEntry bytes encode the concrete entry struct, not the
            // EntryTypes enum wrapper. Decode the concrete type expected by
            // this link kind, then pass the in-memory enum to the pure rule.
            let target_entry = match &link_type {
                LinkTypes::ProducerToProduction => match target_record
                    .entry()
                    .to_app_option::<EnergyProduction>()
                    .ok()
                    .flatten()
                {
                    Some(entry) => EntryTypes::EnergyProduction(entry),
                    None => {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Producer index must target an energy-production entry".into(),
                        ));
                    }
                },
                LinkTypes::ConsumerToConsumption => match target_record
                    .entry()
                    .to_app_option::<EnergyConsumption>()
                    .ok()
                    .flatten()
                {
                    Some(entry) => EntryTypes::EnergyConsumption(entry),
                    None => {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Consumer index must target an energy-consumption entry".into(),
                        ));
                    }
                },
                LinkTypes::SellerToOffers | LinkTypes::ActiveOffers => match target_record
                    .entry()
                    .to_app_option::<TradeOffer>()
                    .ok()
                    .flatten()
                {
                    Some(entry) => EntryTypes::TradeOffer(entry),
                    None => {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Offer index must target a trade-offer entry".into(),
                        ));
                    }
                },
                LinkTypes::OfferToTrades | LinkTypes::BuyerToTrades => match target_record
                    .entry()
                    .to_app_option::<Trade>()
                    .ok()
                    .flatten()
                {
                    Some(entry) => EntryTypes::Trade(entry),
                    None => {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Trade index must target a trade entry".into(),
                        ));
                    }
                },
            };
            result_from_validation_error(grid_link_validation_error(
                &base_address,
                &target_entry,
                &link_type,
                &action.author,
            ))
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

/// Reproduce the Energy hApp's anchor-key encoding without depending on the
/// coordinator/shared crate. Keep byte-for-byte parity with
/// mycelix_energy_shared::anchors::anchor_hash.
fn energy_anchor_hash(value: &str) -> AnyLinkableHash {
    let digest = blake2b_256(value.as_bytes());
    let bytes: Vec<u8> = digest.iter().copied().chain([0u8; 4]).collect();
    AnyLinkableHash::from(EntryHash::from_raw_36(bytes))
}

fn offer_day_bucket(timestamp: Timestamp) -> i64 {
    const MICROS_PER_DAY: i64 = 86_400 * 1_000_000;
    timestamp.as_micros().div_euclid(MICROS_PER_DAY)
}

fn grid_link_validation_error(
    base_address: &AnyLinkableHash,
    target_entry: &EntryTypes,
    link_type: &LinkTypes,
    link_author: &AgentPubKey,
) -> Option<&'static str> {
    let invalid = Some("Grid link base, target, or author does not match its entry");

    match (link_type, target_entry) {
        (LinkTypes::ProducerToProduction, EntryTypes::EnergyProduction(production)) => {
            if !did_matches_agent(&production.producer_did, link_author)
                || base_address != &energy_anchor_hash(&production.producer_did)
            {
                return invalid;
            }
        }
        (LinkTypes::ConsumerToConsumption, EntryTypes::EnergyConsumption(consumption)) => {
            if !did_matches_agent(&consumption.consumer_did, link_author)
                || base_address != &energy_anchor_hash(&consumption.consumer_did)
            {
                return invalid;
            }
        }
        (LinkTypes::SellerToOffers, EntryTypes::TradeOffer(offer)) => {
            if !did_matches_agent(&offer.seller_did, link_author)
                || base_address != &energy_anchor_hash(&offer.seller_did)
            {
                return invalid;
            }
        }
        (LinkTypes::ActiveOffers, EntryTypes::TradeOffer(offer)) => {
            let expected_anchor = energy_anchor_hash(&format!(
                "active_energy_offers:{}",
                offer_day_bucket(offer.created)
            ));
            if !did_matches_agent(&offer.seller_did, link_author)
                || base_address != &expected_anchor
            {
                return invalid;
            }
        }
        (LinkTypes::OfferToTrades, EntryTypes::Trade(trade)) => {
            if !did_matches_agent(&trade.buyer_did, link_author)
                || base_address != &energy_anchor_hash(&trade.offer_id)
            {
                return invalid;
            }
        }
        (LinkTypes::BuyerToTrades, EntryTypes::Trade(trade)) => {
            if !did_matches_agent(&trade.buyer_did, link_author)
                || base_address != &energy_anchor_hash(&trade.buyer_did)
            {
                return invalid;
            }
        }
        _ => return Some("Grid index link must target the expected entry type"),
    }

    None
}

fn valid_nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_did(value: &str) -> bool {
    value.starts_with("did:")
        && value.len() > 4
        && !value.chars().any(char::is_whitespace)
}

/// Parse the repository's canonical DID representation and bind it to the
/// Holochain agent key that authored an entry action. A syntactically valid
/// but unrelated DID is not proof of ownership.
fn agent_pub_key_from_did(did: &str) -> Option<AgentPubKey> {
    let agent_str = did.strip_prefix("did:mycelix:")?;
    AgentPubKey::try_from(agent_str).ok()
}

fn did_matches_agent(did: &str, author: &AgentPubKey) -> bool {
    agent_pub_key_from_did(did).as_ref() == Some(author)
}

fn production_validation_error(production: &EnergyProduction) -> Option<&'static str> {
    if !valid_nonempty(&production.id) || !valid_nonempty(&production.project_id) {
        return Some("Production id and project id must be non-empty");
    }
    if !valid_did(&production.producer_did) {
        return Some("Producer must be a valid DID");
    }
    // Verification is an attestation, not a user-controlled input flag. The
    // legacy verification update is rejected below, so never let a new record
    // mint verified production directly while #4944 specifies append-only attestations.
    if production.verified {
        return Some("New production records cannot claim verified status");
    }
    if !production.amount_kwh.is_finite() || production.amount_kwh <= 0.0 {
        return Some("Production amount must be finite and positive");
    }
    if !production.period_hours.is_finite() || production.period_hours <= 0.0 {
        return Some("Production period must be finite and positive");
    }
    if matches!(production.meter_reading, Some(value) if !value.is_finite() || value < 0.0) {
        return Some("Meter reading must be finite and non-negative");
    }
    None
}

fn production_creation_author_validation_error(
    production: &EnergyProduction,
    author: &AgentPubKey,
) -> Option<&'static str> {
    if let Some(error) = production_validation_error(production) {
        return Some(error);
    }
    if !did_matches_agent(&production.producer_did, author) {
        return Some("Producer DID must identify the agent authoring the production record");
    }
    None
}

fn consumption_validation_error(consumption: &EnergyConsumption) -> Option<&'static str> {
    if !valid_nonempty(&consumption.id) {
        return Some("Consumption id must be non-empty");
    }
    if !valid_did(&consumption.consumer_did) {
        return Some("Consumer must be a valid DID");
    }
    if !consumption.amount_kwh.is_finite() || consumption.amount_kwh <= 0.0 {
        return Some("Consumption amount must be finite and positive");
    }
    if !consumption.period_hours.is_finite() || consumption.period_hours <= 0.0 {
        return Some("Consumption period must be finite and positive");
    }
    if matches!(consumption.meter_reading, Some(value) if !value.is_finite() || value < 0.0) {
        return Some("Meter reading must be finite and non-negative");
    }
    None
}

fn consumption_creation_author_validation_error(
    consumption: &EnergyConsumption,
    author: &AgentPubKey,
) -> Option<&'static str> {
    if let Some(error) = consumption_validation_error(consumption) {
        return Some(error);
    }
    if !did_matches_agent(&consumption.consumer_did, author) {
        return Some("Consumer DID must identify the agent authoring the consumption record");
    }
    None
}

fn offer_validation_error(offer: &TradeOffer) -> Option<&'static str> {
    if !valid_nonempty(&offer.id) {
        return Some("Offer id must be non-empty");
    }
    if !valid_did(&offer.seller_did) {
        return Some("Seller must be a valid DID");
    }
    if matches!(offer.project_id.as_deref(), Some(project_id) if !valid_nonempty(project_id)) {
        return Some("Project id must be non-empty when present");
    }
    if !offer.amount_kwh.is_finite() || offer.amount_kwh < 0.0 {
        return Some("Offer amount must be finite and non-negative");
    }
    // A filled offer has no quantity remaining. All other states with a
    // remaining quantity must have a strictly positive finite amount.
    if offer.status == OfferStatus::Filled {
        if offer.amount_kwh != 0.0 {
            return Some("Filled offers must have zero remaining quantity");
        }
    } else if offer.amount_kwh <= 0.0 {
        return Some("Non-filled offers must have positive remaining quantity");
    }
    if !offer.price_per_kwh.is_finite() || offer.price_per_kwh < 0.0 {
        return Some("Price must be finite and non-negative");
    }
    if !valid_nonempty(&offer.currency) {
        return Some("Currency must be non-empty");
    }
    if offer.available_from.as_micros() > offer.available_until.as_micros() {
        return Some("Offer availability window is invalid");
    }
    None
}

fn trade_validation_error(trade: &Trade) -> Option<&'static str> {
    if !valid_nonempty(&trade.id) || !valid_nonempty(&trade.offer_id) {
        return Some("Trade id and offer id must be non-empty");
    }
    if !valid_did(&trade.seller_did) || !valid_did(&trade.buyer_did) {
        return Some("Parties must be valid DIDs");
    }
    if trade.seller_did == trade.buyer_did {
        return Some("Cannot trade with yourself");
    }
    if !trade.amount_kwh.is_finite() || trade.amount_kwh <= 0.0 {
        return Some("Trade amount must be finite and positive");
    }
    if !trade.price_per_kwh.is_finite() || trade.price_per_kwh < 0.0 {
        return Some("Trade price must be finite and non-negative");
    }
    if !trade.total_price.is_finite() || trade.total_price < 0.0 {
        return Some("Trade total price must be finite and non-negative");
    }
    let expected_total = trade.amount_kwh * trade.price_per_kwh;
    let tolerance = expected_total.abs().max(1.0) * 1.0e-9;
    if !expected_total.is_finite() || (trade.total_price - expected_total).abs() > tolerance {
        return Some("Trade total price must equal amount multiplied by unit price");
    }
    if !valid_nonempty(&trade.currency) {
        return Some("Currency must be non-empty");
    }
    match (trade.settled, trade.payment_reference.as_deref()) {
        (false, None) => {}
        (false, Some(_)) => {
            return Some("Unsettled trades must not contain a payment reference");
        }
        (true, Some(reference)) if valid_nonempty(reference) => {}
        (true, _) => {
            return Some("Settled trades require a non-empty payment reference");
        }
    }
    None
}

/// A new trade must begin in the unsettled state. A caller cannot bypass the
/// settlement transition by creating a record that already claims success.
fn trade_creation_validation_error(trade: &Trade) -> Option<&'static str> {
    if trade.settled || trade.payment_reference.is_some() {
        return Some("New trades must start unsettled without a payment reference");
    }
    trade_validation_error(trade)
}

fn trade_creation_author_validation_error(
    trade: &Trade,
    author: &AgentPubKey,
) -> Option<&'static str> {
    if !did_matches_agent(&trade.buyer_did, author) {
        return Some("Buyer DID must identify the agent authoring the trade");
    }
    trade_creation_validation_error(trade)
}

/// Only the original trade author may record settlement. This is an
/// authorization boundary in addition to the monotonic state transition.
fn trade_update_author_validation_error(
    update_author: &AgentPubKey,
    original_author: &AgentPubKey,
) -> Option<&'static str> {
    if update_author != original_author {
        return Some("Only the original trade author may update settlement state");
    }
    None
}

/// Trade terms are immutable after creation. The only permitted update is a
/// one-way unsettled -> settled transition with a non-empty reference.
/// This is a structural invariant, not proof that an external payment really
/// occurred; rail-authenticated receipts remain a separate trust requirement.
fn trade_update_validation_error(
    previous: &Trade,
    updated: &Trade,
) -> Option<&'static str> {
    if let Some(error) = trade_validation_error(updated) {
        return Some(error);
    }
    if previous.settled {
        return Some("Settled trades cannot be updated");
    }
    if previous.payment_reference.is_some() {
        return Some("Original unsettled trade contains a payment reference");
    }
    if previous.id != updated.id
        || previous.offer_id != updated.offer_id
        || previous.seller_did != updated.seller_did
        || previous.buyer_did != updated.buyer_did
        || previous.amount_kwh != updated.amount_kwh
        || previous.price_per_kwh != updated.price_per_kwh
        || previous.total_price != updated.total_price
        || previous.currency != updated.currency
        || previous.executed != updated.executed
    {
        return Some("Trade terms are immutable after creation");
    }
    if !updated.settled {
        return Some("Trade updates must perform the unsettled-to-settled transition");
    }
    None
}

fn result_from_validation_error(
    error: Option<&'static str>,
) -> ExternResult<ValidateCallbackResult> {
    Ok(match error {
        Some(message) => ValidateCallbackResult::Invalid(message.into()),
        None => ValidateCallbackResult::Valid,
    })
}

fn validate_create_energy_production(
    action: EntryCreationAction,
    production: EnergyProduction,
) -> ExternResult<ValidateCallbackResult> {
    result_from_validation_error(production_creation_author_validation_error(
        &production,
        action.author(),
    ))
}

fn validate_create_energy_consumption(
    action: EntryCreationAction,
    consumption: EnergyConsumption,
) -> ExternResult<ValidateCallbackResult> {
    result_from_validation_error(consumption_creation_author_validation_error(
        &consumption,
        action.author(),
    ))
}

fn validate_create_trade_offer(
    action: EntryCreationAction,
    offer: TradeOffer,
) -> ExternResult<ValidateCallbackResult> {
    result_from_validation_error(offer_creation_author_validation_error(
        &offer,
        action.author(),
    ))
}

/// Offer identity and publication terms are immutable. The coordinator may
/// update the price without changing quantity/status, cancel an active offer,
/// or reduce remaining quantity after a fill. No other transition is valid.
fn offer_creation_author_validation_error(
    offer: &TradeOffer,
    author: &AgentPubKey,
) -> Option<&'static str> {
    if let Some(error) = offer_validation_error(offer) {
        return Some(error);
    }
    if !did_matches_agent(&offer.seller_did, author) {
        return Some("Seller DID must identify the agent authoring the offer");
    }
    None
}

fn offer_update_validation_error(
    previous: &TradeOffer,
    updated: &TradeOffer,
) -> Option<&'static str> {
    if let Some(error) = offer_validation_error(updated) {
        return Some(error);
    }
    if previous.id != updated.id
        || previous.seller_did != updated.seller_did
        || previous.project_id != updated.project_id
        || previous.currency != updated.currency
        || previous.available_from != updated.available_from
        || previous.available_until != updated.available_until
        || previous.created != updated.created
    {
        return Some("Offer identity and availability terms are immutable");
    }

    if !matches!(previous.status, OfferStatus::Active | OfferStatus::PartiallyFilled) {
        return Some("Terminal offers cannot be updated");
    }

    // Price changes are permitted only as a price-only edit, preserving the
    // current status and amount. This prevents repricing from being combined
    // with a fill or status transition in one update.
    if updated.price_per_kwh != previous.price_per_kwh {
        if updated.amount_kwh != previous.amount_kwh || updated.status != previous.status {
            return Some("Price changes cannot modify offer quantity or status");
        }
        return None;
    }

    // Cancellation preserves terms and quantity, and only Active -> Cancelled
    // is supported by the coordinator's current cancellation endpoint.
    if previous.status == OfferStatus::Active
        && updated.status == OfferStatus::Cancelled
        && updated.amount_kwh == previous.amount_kwh
    {
        return None;
    }

    // A fill must strictly reduce quantity. The status must agree exactly with
    // the remaining amount, so a no-op / precision-lost fill cannot be accepted.
    if updated.amount_kwh >= previous.amount_kwh {
        return Some("Offer quantity must strictly decrease after a fill");
    }
    let expected_status = if updated.amount_kwh == 0.0 {
        OfferStatus::Filled
    } else {
        OfferStatus::PartiallyFilled
    };
    if updated.status != expected_status {
        return Some("Offer status must match the remaining quantity after a fill");
    }
    None
}

fn validate_update_trade_offer(
    action: Update,
    offer: TradeOffer,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    if &action.author != original_record.action().author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Only the original offer author may update offer state".into(),
        ));
    }
    let original_offer = original_record
        .entry()
        .to_app_option::<TradeOffer>()
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Original offer entry could not be decoded".into()
        )))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Original offer entry is unavailable".into()
            ))
        })?;
    result_from_validation_error(offer_update_validation_error(&original_offer, &offer))
}

fn validate_create_trade(
    action: EntryCreationAction,
    trade: Trade,
) -> ExternResult<ValidateCallbackResult> {
    result_from_validation_error(trade_creation_author_validation_error(
        &trade,
        action.author(),
    ))
}

fn validate_update_trade(
    action: Update,
    trade: Trade,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    if let Some(error) =
        trade_update_author_validation_error(&action.author, original_record.action().author())
    {
        return Ok(ValidateCallbackResult::Invalid(error.into()));
    }
    let original_trade = original_record
        .entry()
        .to_app_option::<Trade>()
        .map_err(|_| wasm_error!(WasmErrorInner::Guest(
            "Original trade entry could not be decoded".into()
        )))?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Original trade entry is unavailable".into()
            ))
        })?;
    result_from_validation_error(trade_update_validation_error(&original_trade, &trade))
}

#[cfg(test)]
mod tests {
    use super::*;

    // =========================================================================
    // OfferStatus Enum Tests
    // =========================================================================

    #[test]
    fn test_offer_status_variants() {
        let statuses = vec![
            OfferStatus::Active,
            OfferStatus::PartiallyFilled,
            OfferStatus::Filled,
            OfferStatus::Expired,
            OfferStatus::Cancelled,
        ];
        assert_eq!(statuses.len(), 5);
    }

    #[test]
    fn test_offer_status_equality() {
        assert_eq!(OfferStatus::Active, OfferStatus::Active);
        assert_ne!(OfferStatus::Active, OfferStatus::Cancelled);
    }

    // =========================================================================
    // EnergyProduction Validation Tests
    // =========================================================================

    fn create_test_timestamp() -> Timestamp {
        Timestamp::from_micros(1704067200000000) // 2024-01-01 00:00:00 UTC
    }

    fn valid_energy_production() -> EnergyProduction {
        EnergyProduction {
            id: "prod:did:test:producer1:123456".to_string(),
            producer_did: "did:test:producer1".to_string(),
            project_id: "project:solar_farm_1".to_string(),
            amount_kwh: 100.5,
            timestamp: create_test_timestamp(),
            period_hours: 24.0,
            meter_reading: Some(12500.75),
            verified: false,
        }
    }

    #[test]
    fn test_energy_production_valid_did() {
        let production = valid_energy_production();
        assert!(production.producer_did.starts_with("did:"));
    }

    #[test]
    fn test_energy_production_positive_amount() {
        let production = valid_energy_production();
        assert!(production.amount_kwh > 0.0);
    }

    #[test]
    fn test_energy_production_invalid_did_format() {
        let production = EnergyProduction {
            producer_did: "invalid_producer".to_string(),
            ..valid_energy_production()
        };
        // Validation should fail for non-DID format
        assert!(!production.producer_did.starts_with("did:"));
    }

    #[test]
    fn test_energy_production_zero_amount_invalid() {
        let production = EnergyProduction {
            amount_kwh: 0.0,
            ..valid_energy_production()
        };
        assert!(production.amount_kwh <= 0.0);
    }

    #[test]
    fn test_energy_production_negative_amount_invalid() {
        let production = EnergyProduction {
            amount_kwh: -50.0,
            ..valid_energy_production()
        };
        assert!(production.amount_kwh <= 0.0);
    }

    #[test]
    fn test_energy_production_with_meter_reading() {
        let production = valid_energy_production();
        assert!(production.meter_reading.is_some());
        assert_eq!(production.meter_reading.unwrap(), 12500.75);
    }

    #[test]
    fn test_energy_production_without_meter_reading() {
        let production = EnergyProduction {
            meter_reading: None,
            ..valid_energy_production()
        };
        assert!(production.meter_reading.is_none());
    }

    #[test]
    fn test_energy_production_unverified_by_default() {
        let production = valid_energy_production();
        assert!(!production.verified);
    }

    // =========================================================================
    // EnergyConsumption Validation Tests
    // =========================================================================

    fn valid_energy_consumption() -> EnergyConsumption {
        EnergyConsumption {
            id: "cons:did:test:consumer1:123456".to_string(),
            consumer_did: "did:test:consumer1".to_string(),
            amount_kwh: 75.25,
            timestamp: create_test_timestamp(),
            period_hours: 24.0,
            meter_reading: Some(8500.50),
        }
    }

    #[test]
    fn test_energy_consumption_valid_did() {
        let consumption = valid_energy_consumption();
        assert!(consumption.consumer_did.starts_with("did:"));
    }

    #[test]
    fn test_energy_consumption_positive_amount() {
        let consumption = valid_energy_consumption();
        assert!(consumption.amount_kwh > 0.0);
    }

    #[test]
    fn test_energy_consumption_invalid_did_format() {
        let consumption = EnergyConsumption {
            consumer_did: "consumer123".to_string(),
            ..valid_energy_consumption()
        };
        assert!(!consumption.consumer_did.starts_with("did:"));
    }

    #[test]
    fn test_energy_consumption_zero_amount_invalid() {
        let consumption = EnergyConsumption {
            amount_kwh: 0.0,
            ..valid_energy_consumption()
        };
        assert!(consumption.amount_kwh <= 0.0);
    }

    #[test]
    fn test_energy_consumption_negative_amount_invalid() {
        let consumption = EnergyConsumption {
            amount_kwh: -25.0,
            ..valid_energy_consumption()
        };
        assert!(consumption.amount_kwh <= 0.0);
    }

    // =========================================================================
    // TradeOffer Validation Tests
    // =========================================================================

    fn valid_trade_offer() -> TradeOffer {
        TradeOffer {
            id: "offer:did:test:seller1:123456".to_string(),
            seller_did: "did:test:seller1".to_string(),
            project_id: Some("project:solar_farm_1".to_string()),
            amount_kwh: 500.0,
            price_per_kwh: 0.15,
            currency: "USD".to_string(),
            available_from: create_test_timestamp(),
            available_until: Timestamp::from_micros(1704153600000000), // +24 hours
            status: OfferStatus::Active,
            created: create_test_timestamp(),
        }
    }

    #[test]
    fn test_trade_offer_valid_seller_did() {
        let offer = valid_trade_offer();
        assert!(offer.seller_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_offer_positive_amount() {
        let offer = valid_trade_offer();
        assert!(offer.amount_kwh > 0.0);
    }

    #[test]
    fn test_trade_offer_non_negative_price() {
        let offer = valid_trade_offer();
        assert!(offer.price_per_kwh >= 0.0);
    }

    #[test]
    fn test_trade_offer_zero_price_valid() {
        // Free energy offers should be allowed
        let offer = TradeOffer {
            price_per_kwh: 0.0,
            ..valid_trade_offer()
        };
        assert!(offer.price_per_kwh >= 0.0);
    }

    #[test]
    fn test_trade_offer_negative_price_invalid() {
        let offer = TradeOffer {
            price_per_kwh: -0.05,
            ..valid_trade_offer()
        };
        assert!(offer.price_per_kwh < 0.0);
    }

    #[test]
    fn test_trade_offer_invalid_seller_did() {
        let offer = TradeOffer {
            seller_did: "seller123".to_string(),
            ..valid_trade_offer()
        };
        assert!(!offer.seller_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_offer_zero_amount_invalid() {
        let offer = TradeOffer {
            amount_kwh: 0.0,
            ..valid_trade_offer()
        };
        assert!(offer.amount_kwh <= 0.0);
    }

    #[test]
    fn test_trade_offer_with_project_id() {
        let offer = valid_trade_offer();
        assert!(offer.project_id.is_some());
    }

    #[test]
    fn test_trade_offer_without_project_id() {
        let offer = TradeOffer {
            project_id: None,
            ..valid_trade_offer()
        };
        assert!(offer.project_id.is_none());
    }

    #[test]
    fn test_trade_offer_status_transitions() {
        // Test valid status values
        let statuses = [
            OfferStatus::Active,
            OfferStatus::PartiallyFilled,
            OfferStatus::Filled,
            OfferStatus::Expired,
            OfferStatus::Cancelled,
        ];

        for status in statuses {
            let offer = TradeOffer {
                status,
                ..valid_trade_offer()
            };
            // Each status should be cloneable and comparable
            assert_eq!(offer.status.clone(), offer.status);
        }
    }

    // =========================================================================
    // Trade Validation Tests
    // =========================================================================

    fn valid_trade() -> Trade {
        Trade {
            id: "trade:offer1:123456".to_string(),
            offer_id: "offer:did:test:seller1:123456".to_string(),
            seller_did: "did:test:seller1".to_string(),
            buyer_did: "did:test:buyer1".to_string(),
            amount_kwh: 100.0,
            price_per_kwh: 0.15,
            total_price: 15.0,
            currency: "USD".to_string(),
            executed: create_test_timestamp(),
            settled: false,
            payment_reference: None,
        }
    }

    #[test]
    fn test_trade_valid_seller_did() {
        let trade = valid_trade();
        assert!(trade.seller_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_valid_buyer_did() {
        let trade = valid_trade();
        assert!(trade.buyer_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_different_seller_and_buyer() {
        let trade = valid_trade();
        assert_ne!(trade.seller_did, trade.buyer_did);
    }

    #[test]
    fn test_trade_same_seller_buyer_invalid() {
        let trade = Trade {
            buyer_did: "did:test:seller1".to_string(),
            ..valid_trade()
        };
        // Self-trade should be invalid
        assert_eq!(trade.seller_did, trade.buyer_did);
    }

    #[test]
    fn test_trade_invalid_seller_did() {
        let trade = Trade {
            seller_did: "seller123".to_string(),
            ..valid_trade()
        };
        assert!(!trade.seller_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_invalid_buyer_did() {
        let trade = Trade {
            buyer_did: "buyer456".to_string(),
            ..valid_trade()
        };
        assert!(!trade.buyer_did.starts_with("did:"));
    }

    #[test]
    fn test_trade_total_price_calculation() {
        let trade = valid_trade();
        let expected_total = trade.amount_kwh * trade.price_per_kwh;
        assert!((trade.total_price - expected_total).abs() < 0.001);
    }

    #[test]
    fn test_trade_unsettled_by_default() {
        let trade = valid_trade();
        assert!(!trade.settled);
        assert!(trade.payment_reference.is_none());
    }

    #[test]
    fn test_trade_settled_with_reference() {
        let trade = Trade {
            settled: true,
            payment_reference: Some("PAY-2024-001234".to_string()),
            ..valid_trade()
        };
        assert!(trade.settled);
        assert!(trade.payment_reference.is_some());
    }

    // =========================================================================
    // Anchor Tests
    // =========================================================================

    #[test]
    fn test_anchor_creation() {
        let anchor = Anchor("active_energy_offers".to_string());
        assert_eq!(anchor.0, "active_energy_offers");
    }

    #[test]
    fn test_anchor_equality() {
        let anchor1 = Anchor("test_anchor".to_string());
        let anchor2 = Anchor("test_anchor".to_string());
        let anchor3 = Anchor("other_anchor".to_string());

        assert_eq!(anchor1, anchor2);
        assert_ne!(anchor1, anchor3);
    }

    // =========================================================================
    // Edge Case Tests
    // =========================================================================

    #[test]
    fn test_very_large_kwh_amount() {
        let production = EnergyProduction {
            amount_kwh: 1_000_000_000.0, // 1 billion kWh
            ..valid_energy_production()
        };
        assert!(production.amount_kwh > 0.0);
    }

    #[test]
    fn test_very_small_kwh_amount() {
        let production = EnergyProduction {
            amount_kwh: 0.001, // 1 Wh
            ..valid_energy_production()
        };
        assert!(production.amount_kwh > 0.0);
    }

    #[test]
    fn test_very_high_price_per_kwh() {
        let offer = TradeOffer {
            price_per_kwh: 10000.0, // Very high price
            ..valid_trade_offer()
        };
        assert!(offer.price_per_kwh >= 0.0);
    }

    #[test]
    fn test_fractional_kwh_amounts() {
        let trade = Trade {
            amount_kwh: 0.123456789,
            price_per_kwh: 0.15,
            total_price: 0.123456789 * 0.15,
            ..valid_trade()
        };
        assert!(trade.amount_kwh > 0.0);
    }

    #[test]
    fn test_various_currency_codes() {
        let currencies = vec!["USD", "EUR", "GBP", "BTC", "ETH", "SOLAR"];
        for currency in currencies {
            let offer = TradeOffer {
                currency: currency.to_string(),
                ..valid_trade_offer()
            };
            assert!(!offer.currency.is_empty());
        }
    }

    #[test]
    fn test_long_period_hours() {
        let production = EnergyProduction {
            period_hours: 8760.0, // Full year
            ..valid_energy_production()
        };
        assert!(production.period_hours > 0.0);
    }

    #[test]
    fn test_fractional_period_hours() {
        let consumption = EnergyConsumption {
            period_hours: 0.5, // 30 minutes
            ..valid_energy_consumption()
        };
        assert!(consumption.period_hours > 0.0);
    }
}


#[cfg(test)]
mod strict_validation_regression_tests {
    use super::*;

    fn timestamp() -> Timestamp {
        Timestamp::from_micros(1_700_000_000_000_000)
    }

    fn valid_production() -> EnergyProduction {
        EnergyProduction {
            id: "prod-1".into(),
            producer_did: "did:test:producer".into(),
            project_id: "solar-1".into(),
            amount_kwh: 10.0,
            timestamp: timestamp(),
            period_hours: 1.0,
            meter_reading: Some(100.0),
            verified: false,
        }
    }

    fn valid_offer() -> TradeOffer {
        TradeOffer {
            id: "offer-1".into(),
            seller_did: "did:test:seller".into(),
            project_id: Some("solar-1".into()),
            amount_kwh: 10.0,
            price_per_kwh: 0.12,
            currency: "USD".into(),
            available_from: timestamp(),
            available_until: Timestamp::from_micros(1_700_003_600_000_000),
            status: OfferStatus::Active,
            created: timestamp(),
        }
    }

    fn valid_trade() -> Trade {
        Trade {
            id: "trade-1".into(),
            offer_id: "offer-1".into(),
            seller_did: "did:test:seller".into(),
            buyer_did: "did:test:buyer".into(),
            amount_kwh: 2.0,
            price_per_kwh: 0.12,
            total_price: 0.24,
            currency: "USD".into(),
            executed: timestamp(),
            settled: false,
            payment_reference: None,
        }
    }

    #[test]
    fn production_rejects_non_finite_values_and_invalid_intervals() {
        let mut production = valid_production();
        production.amount_kwh = f64::NAN;
        assert!(production_validation_error(&production).is_some());

        let mut production = valid_production();
        production.period_hours = f64::INFINITY;
        assert!(production_validation_error(&production).is_some());

        let mut production = valid_production();
        production.meter_reading = Some(-1.0);
        assert!(production_validation_error(&production).is_some());

        let mut production = valid_production();
        production.verified = true;
        assert_eq!(
            production_validation_error(&production),
            Some("New production records cannot claim verified status")
        );
    }

    #[test]
    fn grid_link_validation_binds_target_type_anchor_and_author() {
        let seller = AgentPubKey::from_raw_32(vec![7u8; 32]);
        let seller_did = format!("did:mycelix:{seller}");
        let other = AgentPubKey::from_raw_32(vec![8u8; 32]);
        let timestamp = Timestamp::from_micros(1_700_000_000_000_000);
        let offer = TradeOffer {
            seller_did: seller_did.clone(),
            created: timestamp,
            ..valid_offer()
        };
        let offer_entry = EntryTypes::TradeOffer(offer.clone());

        let production = EnergyProduction {
            producer_did: seller_did.clone(),
            ..valid_production()
        };
        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&seller_did),
                &EntryTypes::EnergyProduction(production.clone()),
                &LinkTypes::ProducerToProduction,
                &seller,
            ),
            None
        );
        assert!(grid_link_validation_error(
            &energy_anchor_hash("did:mycelix:someone-else"),
            &EntryTypes::EnergyProduction(production),
            &LinkTypes::ProducerToProduction,
            &seller,
        )
        .is_some());

        let consumption = EnergyConsumption {
            id: "consumption-link-test".into(),
            consumer_did: seller_did.clone(),
            amount_kwh: 1.0,
            timestamp,
            period_hours: 1.0,
            meter_reading: None,
        };
        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&seller_did),
                &EntryTypes::EnergyConsumption(consumption.clone()),
                &LinkTypes::ConsumerToConsumption,
                &seller,
            ),
            None
        );
        assert!(grid_link_validation_error(
            &energy_anchor_hash(&seller_did),
            &EntryTypes::EnergyConsumption(consumption),
            &LinkTypes::ConsumerToConsumption,
            &other,
        )
        .is_some());

        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&seller_did),
                &offer_entry,
                &LinkTypes::SellerToOffers,
                &seller,
            ),
            None
        );
        assert!(grid_link_validation_error(
            &energy_anchor_hash("did:mycelix:someone-else"),
            &offer_entry,
            &LinkTypes::SellerToOffers,
            &seller,
        )
        .is_some());
        assert!(grid_link_validation_error(
            &energy_anchor_hash(&seller_did),
            &offer_entry,
            &LinkTypes::SellerToOffers,
            &other,
        )
        .is_some());

        let shard = format!("active_energy_offers:{}", offer_day_bucket(timestamp));
        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&shard),
                &offer_entry,
                &LinkTypes::ActiveOffers,
                &seller,
            ),
            None
        );
        assert!(grid_link_validation_error(
            &energy_anchor_hash("active_energy_offers:0"),
            &offer_entry,
            &LinkTypes::ActiveOffers,
            &seller,
        )
        .is_some());

        let buyer = AgentPubKey::from_raw_32(vec![9u8; 32]);
        let buyer_did = format!("did:mycelix:{buyer}");
        let trade = Trade {
            buyer_did: buyer_did.clone(),
            offer_id: offer.id.clone(),
            ..valid_trade()
        };
        let trade_entry = EntryTypes::Trade(trade.clone());
        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&trade.offer_id),
                &trade_entry,
                &LinkTypes::OfferToTrades,
                &buyer,
            ),
            None
        );
        assert_eq!(
            grid_link_validation_error(
                &energy_anchor_hash(&buyer_did),
                &trade_entry,
                &LinkTypes::BuyerToTrades,
                &buyer,
            ),
            None
        );
        assert!(grid_link_validation_error(
            &energy_anchor_hash(&trade.offer_id),
            &EntryTypes::TradeOffer(offer),
            &LinkTypes::OfferToTrades,
            &buyer,
        )
        .is_some());
    }

    #[test]
    fn production_and_consumption_identity_must_match_the_author() {
        let author = AgentPubKey::from_raw_32(vec![7u8; 32]);
        let other = AgentPubKey::from_raw_32(vec![8u8; 32]);
        let author_did = format!("did:mycelix:{author}");
        let other_did = format!("did:mycelix:{other}");

        let owned_production = EnergyProduction {
            producer_did: author_did.clone(),
            ..valid_production()
        };
        assert_eq!(
            production_creation_author_validation_error(&owned_production, &author),
            None
        );
        let impersonated_production = EnergyProduction {
            producer_did: other_did.clone(),
            ..valid_production()
        };
        assert_eq!(
            production_creation_author_validation_error(&impersonated_production, &author),
            Some("Producer DID must identify the agent authoring the production record")
        );

        let owned_consumption = EnergyConsumption {
            id: "consumption-1".into(),
            consumer_did: author_did,
            amount_kwh: 2.0,
            timestamp: timestamp(),
            period_hours: 1.0,
            meter_reading: None,
        };
        assert_eq!(
            consumption_creation_author_validation_error(&owned_consumption, &author),
            None
        );
        let impersonated_consumption = EnergyConsumption {
            consumer_did: other_did,
            ..owned_consumption
        };
        assert_eq!(
            consumption_creation_author_validation_error(&impersonated_consumption, &author),
            Some("Consumer DID must identify the agent authoring the consumption record")
        );
    }

    #[test]
    fn offers_reject_bad_price_and_reversed_availability_window() {
        let mut offer = valid_offer();
        offer.price_per_kwh = f64::INFINITY;
        assert!(offer_validation_error(&offer).is_some());

        let mut offer = valid_offer();
        offer.available_from = Timestamp::from_micros(1_700_010_000_000_000);
        assert!(offer_validation_error(&offer).is_some());
    }

    #[test]
    fn fully_filled_offer_may_have_zero_remaining_quantity_only() {
        let mut offer = valid_offer();
        offer.status = OfferStatus::Filled;
        offer.amount_kwh = 0.0;
        assert_eq!(offer_validation_error(&offer), None);

        offer.amount_kwh = 1.0;
        assert!(offer_validation_error(&offer).is_some());
    }

    #[test]
    fn trades_reject_invalid_amounts_and_inconsistent_totals() {
        let mut trade = valid_trade();
        trade.amount_kwh = f64::NAN;
        assert!(trade_validation_error(&trade).is_some());

        let mut trade = valid_trade();
        trade.total_price = 99.0;
        assert!(trade_validation_error(&trade).is_some());
    }

    #[test]
    fn valid_energy_records_remain_accepted() {
        assert_eq!(production_validation_error(&valid_production()), None);
        assert_eq!(offer_validation_error(&valid_offer()), None);
        assert_eq!(trade_validation_error(&valid_trade()), None);
        assert_eq!(trade_creation_validation_error(&valid_trade()), None);
    }

    #[test]
    fn new_trades_cannot_claim_settlement_or_attach_a_premature_reference() {
        let mut settled_at_creation = valid_trade();
        settled_at_creation.settled = true;
        settled_at_creation.payment_reference = Some("manual-reference".into());
        assert_eq!(
            trade_creation_validation_error(&settled_at_creation),
            Some("New trades must start unsettled without a payment reference")
        );

        let mut reference_before_settlement = valid_trade();
        reference_before_settlement.payment_reference = Some("premature-reference".into());
        assert_eq!(
            trade_creation_validation_error(&reference_before_settlement),
            Some("New trades must start unsettled without a payment reference")
        );
    }

    #[test]
    fn trade_updates_only_allow_immutable_terms_and_one_way_settlement() {
        let previous = valid_trade();
        let settled = Trade {
            settled: true,
            payment_reference: Some("external:receipt-1".into()),
            ..previous.clone()
        };
        assert_eq!(trade_update_validation_error(&previous, &settled), None);

        let changed_terms = Trade {
            amount_kwh: 3.0,
            total_price: 0.36,
            ..settled.clone()
        };
        assert_eq!(
            trade_update_validation_error(&previous, &changed_terms),
            Some("Trade terms are immutable after creation")
        );

        let rolled_back = valid_trade();
        assert_eq!(
            trade_update_validation_error(&settled, &rolled_back),
            Some("Settled trades cannot be updated")
        );
    }

    #[test]
    fn offer_updates_preserve_identity_and_only_allow_valid_transitions() {
        let previous = valid_offer();

        let price_only = TradeOffer {
            price_per_kwh: 0.15,
            ..previous.clone()
        };
        assert_eq!(offer_update_validation_error(&previous, &price_only), None);

        let partial_fill = TradeOffer {
            amount_kwh: 4.0,
            status: OfferStatus::PartiallyFilled,
            ..previous.clone()
        };
        assert_eq!(offer_update_validation_error(&previous, &partial_fill), None);

        let filled = TradeOffer {
            amount_kwh: 0.0,
            status: OfferStatus::Filled,
            ..previous.clone()
        };
        assert_eq!(offer_update_validation_error(&previous, &filled), None);

        let cancelled = TradeOffer {
            status: OfferStatus::Cancelled,
            ..previous.clone()
        };
        assert_eq!(offer_update_validation_error(&previous, &cancelled), None);
    }

    #[test]
    fn offer_updates_reject_term_changes_quantity_increases_and_noop_fills() {
        let previous = valid_offer();

        let changed_seller = TradeOffer {
            seller_did: "did:test:attacker".into(),
            ..previous.clone()
        };
        assert_eq!(
            offer_update_validation_error(&previous, &changed_seller),
            Some("Offer identity and availability terms are immutable")
        );

        let increased_quantity = TradeOffer {
            amount_kwh: 11.0,
            ..previous.clone()
        };
        assert_eq!(
            offer_update_validation_error(&previous, &increased_quantity),
            Some("Offer quantity must strictly decrease after a fill")
        );

        let no_op_fill = TradeOffer {
            status: OfferStatus::PartiallyFilled,
            ..previous.clone()
        };
        assert_eq!(
            offer_update_validation_error(&previous, &no_op_fill),
            Some("Offer quantity must strictly decrease after a fill")
        );

        let combined_price_and_fill = TradeOffer {
            amount_kwh: 4.0,
            price_per_kwh: 0.15,
            status: OfferStatus::PartiallyFilled,
            ..previous.clone()
        };
        assert_eq!(
            offer_update_validation_error(&previous, &combined_price_and_fill),
            Some("Price changes cannot modify offer quantity or status")
        );
    }

    #[test]
    fn terminal_offers_cannot_be_reopened_or_repriced() {
        let previous = TradeOffer {
            status: OfferStatus::Filled,
            amount_kwh: 0.0,
            ..valid_offer()
        };
        let reopened = TradeOffer {
            status: OfferStatus::Active,
            amount_kwh: 10.0,
            ..previous.clone()
        };
        assert_eq!(
            offer_update_validation_error(&previous, &reopened),
            Some("Terminal offers cannot be updated")
        );
    }

    #[test]
    fn offer_and_trade_create_claims_must_match_the_author_key() {
        let author = AgentPubKey::from_raw_32(vec![7u8; 32]);
        let other_author = AgentPubKey::from_raw_32(vec![8u8; 32]);
        let author_did = format!("did:mycelix:{author}");
        let other_did = format!("did:mycelix:{other_author}");

        assert!(did_matches_agent(&author_did, &author));
        assert!(!did_matches_agent(&other_did, &author));
        assert!(!did_matches_agent("did:test:seller", &author));
        assert!(!did_matches_agent("did:mycelix:not-a-public-key", &author));

        let owned_offer = TradeOffer {
            seller_did: author_did.clone(),
            ..valid_offer()
        };
        assert_eq!(offer_creation_author_validation_error(&owned_offer, &author), None);

        let owned_trade = Trade {
            buyer_did: author_did,
            ..valid_trade()
        };
        assert_eq!(trade_creation_author_validation_error(&owned_trade, &author), None);

        let mut offer = valid_offer();
        offer.seller_did = other_did.clone();
        assert_eq!(
            offer_creation_author_validation_error(&offer, &author),
            Some("Seller DID must identify the agent authoring the offer")
        );

        let mut trade = valid_trade();
        trade.buyer_did = other_did;
        assert_eq!(
            trade_creation_author_validation_error(&trade, &author),
            Some("Buyer DID must identify the agent authoring the trade")
        );
    }

    #[test]
    fn trade_settlement_updates_require_original_author() {
        let original_author = AgentPubKey::from_raw_32(vec![7u8; 32]);
        let other_author = AgentPubKey::from_raw_32(vec![8u8; 32]);
        assert_eq!(
            trade_update_author_validation_error(&original_author, &original_author),
            None
        );
        assert_eq!(
            trade_update_author_validation_error(&other_author, &original_author),
            Some("Only the original trade author may update settlement state")
        );
    }

    #[test]
    fn settled_trades_require_non_empty_references() {
        let mut trade = valid_trade();
        trade.settled = true;
        assert_eq!(
            trade_validation_error(&trade),
            Some("Settled trades require a non-empty payment reference")
        );

        trade.payment_reference = Some("   ".into());
        assert_eq!(
            trade_validation_error(&trade),
            Some("Settled trades require a non-empty payment reference")
        );
    }
}
