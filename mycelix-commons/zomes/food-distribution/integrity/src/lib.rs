// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Food Distribution Integrity Zome
//! Entry types and validation for local food markets, listings, and orders.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{check_author_match, check_link_author_match, did_for_author};

pub mod food_lot_reconciliation;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

// ============================================================================
// MARKET
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MarketType {
    Farmers,
    CSA,
    FoodBank,
    CoOp,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Market {
    pub id: String,
    pub name: String,
    pub location_lat: f64,
    pub location_lon: f64,
    pub market_type: MarketType,
    pub steward: AgentPubKey,
    pub schedule: String,
}

// ============================================================================
// LISTING
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ListingStatus {
    Available,
    Reserved,
    Sold,
    Expired,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Listing {
    pub market_hash: ActionHash,
    pub producer: AgentPubKey,
    pub product_name: String,
    pub quantity_kg: f64,
    pub price_per_kg: f64,
    pub available_from: u64,
    pub status: ListingStatus,
    pub allergen_flags: Vec<String>,
    pub organic: bool,
    pub cultural_markers: Vec<String>,
}

// ============================================================================
// ORDER
// ============================================================================

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum OrderStatus {
    Pending,
    Confirmed,
    Fulfilled,
    Cancelled,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Order {
    pub listing_hash: ActionHash,
    pub buyer: AgentPubKey,
    pub quantity_kg: f64,
    pub status: OrderStatus,
}

/// Immutable public supplier/producer-side lot declaration.
/// The committing agent is bound to lot.producer_ref in the integrity callback.
/// This proves authorship of a declaration only, not legal producer status.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ProducerLotRecordV1 {
    pub lot: food_lot_reconciliation::FoodLotProvenanceV1,
    /// Typed author identity, checked against the Holochain Create action signer.
    pub producer_author: AgentPubKey,
    /// Revision N must reference the signed action for revision N-1.
    pub previous_revision: Option<ActionHash>,
}

// ============================================================================
// ENTRY & LINK TYPE REGISTRATION
// ============================================================================

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Anchor(Anchor),
    Market(Market),
    Listing(Listing),
    Order(Order),
    ProducerLot(ProducerLotRecordV1),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMarkets,
    MarketToListing,
    ProducerToListing,
    ProducerToLot,
    BuyerToOrder,
    ListingToOrder,
    GeoIndex,
}

// ============================================================================
// VALIDATION
// ============================================================================

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::Market(m) => validate_market(m),
                EntryTypes::Listing(l) => validate_listing(l),
                EntryTypes::Order(o) => validate_order(o),
                EntryTypes::ProducerLot(record) => {
                    validate_producer_lot(record, EntryCreationAction::Create(action))
                },
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::ProducerLot(_) => Ok(ValidateCallbackResult::Invalid(
                    "Producer lot declarations are immutable; publish a linked next revision"
                        .into(),
                )),
                EntryTypes::Market(m) => validate_market(m),
                EntryTypes::Listing(l) => validate_listing(l),
                EntryTypes::Order(o) => validate_order(o),
                _ => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, tag, .. } => match link_type {
            LinkTypes::AllMarkets => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AllMarkets link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::MarketToListing => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "MarketToListing link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::ProducerToListing => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "ProducerToListing link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::ProducerToLot => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "ProducerToLot link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::BuyerToOrder => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "BuyerToOrder link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::ListingToOrder => {
                if tag.0.len() > 256 {
                    return Ok(ValidateCallbackResult::Invalid(
                        "ListingToOrder link tag too long (max 256 bytes)".into(),
                    ));
                }
                Ok(ValidateCallbackResult::Valid)
            }
            LinkTypes::GeoIndex => Ok(ValidateCallbackResult::Valid),
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
            if matches!(
                &update,
                OpUpdate::Entry {
                    app_entry: EntryTypes::ProducerLot(_),
                    ..
                }
            ) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Producer lot declarations are immutable; publish a linked next revision"
                        .into(),
                ));
            }
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
            let original_record = must_get_valid_record(action.deletes_address.clone())?;
            let is_producer_lot = original_record
                .entry()
                .to_app_option::<ProducerLotRecordV1>()
                .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
                .is_some();
            if is_producer_lot {
                return Ok(ValidateCallbackResult::Invalid(
                    "Producer lot declarations cannot be deleted; supersede them with a linked revision".into(),
                ));
            }
            let original = must_get_action(action.deletes_address.clone())?;
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

fn validate_producer_lot_fields(
    lot: &food_lot_reconciliation::FoodLotProvenanceV1,
    action_author: &AgentPubKey,
) -> Result<(), String> {
    if lot.schema_version != food_lot_reconciliation::FOOD_LOT_RECONCILIATION_SCHEMA_VERSION {
        return Err("Unsupported producer lot schema version".into());
    }
    let suffix = lot
        .lot_ref
        .strip_prefix("mycelix:food-lot:")
        .ok_or_else(|| "lot_ref must use the canonical mycelix:food-lot namespace".to_string())?;
    if suffix.is_empty()
        || suffix.len() > 128
        || !suffix.as_bytes()[0].is_ascii_lowercase() && !suffix.as_bytes()[0].is_ascii_digit()
        || !suffix.bytes().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(byte, b'.' | b'_' | b':' | b'-')
        })
    {
        return Err("lot_ref suffix is empty or outside the canonical identifier alphabet".into());
    }
    if lot.revision == 0 {
        return Err("Producer lot revision must be positive".into());
    }
    if lot.product_ref.trim().is_empty() || lot.product_ref.len() > 512 {
        return Err("product_ref must be non-empty and at most 512 bytes".into());
    }
    if lot.producer_ref != did_for_author(action_author) {
        return Err(
            "producer_ref must equal the canonical DID of the signed Holochain action author"
                .into(),
        );
    }
    if food_lot_reconciliation::validate_food_lot_quantity(&lot.quantity).is_err() {
        return Err(
            "Producer lot quantity must be positive, dimensionally valid and exactly representable"
                .into(),
        );
    }
    if lot.evidence.source_ref.trim().is_empty() || lot.evidence.source_ref.len() > 512 {
        return Err("Evidence source_ref must be non-empty and at most 512 bytes".into());
    }
    match lot.evidence.evidence_class.as_str() {
        "Declared" | "Attested" | "Modelled" => {}
        "Measured" => {
            if lot
                .evidence
                .calibration_ref
                .as_deref()
                .is_none_or(|value| value.trim().is_empty())
            {
                return Err("Measured lot evidence requires a calibration reference".into());
            }
        }
        "IndependentlyAssayed" | "IndependentlyQualified" => {
            if lot
                .evidence
                .method_ref
                .as_deref()
                .is_none_or(|value| value.trim().is_empty())
            {
                return Err("Independent lot evidence requires a method reference".into());
            }
        }
        _ => return Err("Unsupported producer lot evidence class".into()),
    }
    if lot.claim_ceiling != food_lot_reconciliation::FOOD_LOT_PROVENANCE_CLAIM_CEILING {
        return Err(
            "Producer lot claim ceiling does not match the fixed source-declaration contract"
                .into(),
        );
    }
    if lot
        .harvested_at
        .zip(lot.packed_at)
        .is_some_and(|(harvested, packed)| packed < harvested)
    {
        return Err("packed_at cannot precede harvested_at".into());
    }
    let origin_time = lot.packed_at.or(lot.harvested_at);
    if origin_time.is_some_and(|origin| {
        lot.best_before_at
            .is_some_and(|best_before| best_before < origin)
            || lot.use_by_at.is_some_and(|use_by| use_by < origin)
    }) {
        return Err(
            "best-before/use-by dates cannot precede the known harvest/packing time".into(),
        );
    }
    for optional_ref in [
        lot.supplier_lot_code.as_deref(),
        lot.listing_ref.as_deref(),
        lot.origin_ref.as_deref(),
        lot.handling_ref.as_deref(),
        lot.evidence.method_ref.as_deref(),
        lot.evidence.calibration_ref.as_deref(),
    ] {
        if optional_ref.is_some_and(|value| value.trim().is_empty() || value.len() > 512) {
            return Err("Optional producer lot references must be non-empty and at most 512 bytes when present".into());
        }
    }
    Ok(())
}

fn validate_producer_lot_record_author(
    record: &ProducerLotRecordV1,
    action_author: &AgentPubKey,
) -> Result<(), String> {
    if record.producer_author != *action_author {
        return Err(
            "producer_author must equal the cryptographic author of the Holochain Create action"
                .into(),
        );
    }
    validate_producer_lot_fields(&record.lot, action_author)
}

fn validate_producer_lot_revision_fields(
    current: &ProducerLotRecordV1,
    current_author: &AgentPubKey,
    previous: Option<(&ProducerLotRecordV1, &AgentPubKey)>,
) -> Result<(), String> {
    match (&current.previous_revision, previous) {
        (None, None) if current.lot.revision == 1 => Ok(()),
        (Some(_), _) if current.lot.revision == 1 => {
            Err("The first producer lot revision cannot name a predecessor".into())
        }
        (None, _) => Err(
            "Every producer lot revision after revision 1 must link its exact predecessor action"
                .into(),
        ),
        (Some(_), None) => Err(
            "Producer lot predecessor action does not contain a canonical producer lot record"
                .into(),
        ),
        (Some(_), Some((previous_record, previous_author))) => {
            if previous_author != current_author
                || previous_record.producer_author != current.producer_author
                || previous_record.lot.producer_ref != current.lot.producer_ref
            {
                return Err(
                    "Producer lot revision predecessor must have the same signed author"
                        .into(),
                );
            }
            if previous_record.lot.lot_ref != current.lot.lot_ref
                || previous_record.lot.product_ref != current.lot.product_ref
            {
                return Err(
                    "Producer lot revision must continue the same canonical lot and product"
                        .into(),
                );
            }
            if previous_record.lot.revision.checked_add(1) != Some(current.lot.revision) {
                return Err(
                    "Producer lot revision must increment its predecessor by exactly one"
                        .into(),
                );
            }
            Ok(())
        }
    }
}

fn validate_producer_lot(
    record: ProducerLotRecordV1,
    action: EntryCreationAction,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(message) = validate_producer_lot_record_author(&record, action.author()) {
        return Ok(ValidateCallbackResult::Invalid(message));
    }
    let previous = match record.previous_revision.as_ref() {
        None => None,
        Some(previous_hash) => {
            let previous_record = must_get_valid_record(previous_hash.clone())?;
            let previous_author = previous_record.action().author().clone();
            let previous_entry = previous_record
                .entry()
                .to_app_option::<ProducerLotRecordV1>()
                .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?;
            previous_entry.map(|entry| (entry, previous_author))
        }
    };
    let previous_ref = previous
        .as_ref()
        .map(|(entry, author)| (entry, author));
    match validate_producer_lot_revision_fields(&record, action.author(), previous_ref) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(message) => Ok(ValidateCallbackResult::Invalid(message)),
    }
}

fn validate_market(m: Market) -> ExternResult<ValidateCallbackResult> {
    if m.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Market ID cannot be empty".into(),
        ));
    }
    if m.id.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Market ID must be 256 characters or fewer".into(),
        ));
    }
    if m.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Market name cannot be empty".into(),
        ));
    }
    if m.name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Market name must be 256 characters or fewer".into(),
        ));
    }
    if m.schedule.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Schedule must be 256 characters or fewer".into(),
        ));
    }
    if !m.location_lat.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be a finite number".into(),
        ));
    }
    if m.location_lat < -90.0 || m.location_lat > 90.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Latitude must be between -90 and 90".into(),
        ));
    }
    if !m.location_lon.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be a finite number".into(),
        ));
    }
    if m.location_lon < -180.0 || m.location_lon > 180.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Longitude must be between -180 and 180".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_listing(l: Listing) -> ExternResult<ValidateCallbackResult> {
    if l.product_name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Product name cannot be empty".into(),
        ));
    }
    if l.product_name.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Product name must be 256 characters or fewer".into(),
        ));
    }
    if !l.quantity_kg.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be a finite number".into(),
        ));
    }
    if l.quantity_kg <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be positive".into(),
        ));
    }
    if !l.price_per_kg.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Price must be a finite number".into(),
        ));
    }
    if l.price_per_kg < 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Price cannot be negative".into(),
        ));
    }
    // Allergen flags validation
    if l.allergen_flags.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 50 allergen flags".into(),
        ));
    }
    for flag in &l.allergen_flags {
        if flag.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Allergen flag cannot be empty".into(),
            ));
        }
        if flag.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Allergen flag too long (max 128 chars)".into(),
            ));
        }
    }
    // Cultural markers validation
    if l.cultural_markers.len() > 50 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot have more than 50 cultural markers".into(),
        ));
    }
    for marker in &l.cultural_markers {
        if marker.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Cultural marker cannot be empty".into(),
            ));
        }
        if marker.len() > 128 {
            return Ok(ValidateCallbackResult::Invalid(
                "Cultural marker too long (max 128 chars)".into(),
            ));
        }
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_order(o: Order) -> ExternResult<ValidateCallbackResult> {
    if !o.quantity_kg.is_finite() {
        return Ok(ValidateCallbackResult::Invalid(
            "Quantity must be a finite number".into(),
        ));
    }
    if o.quantity_kg <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Order quantity must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────

    fn fake_agent() -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![0u8; 36])
    }
    fn fake_action_hash() -> ActionHash {
        ActionHash::from_raw_36(vec![0u8; 36])
    }

    fn valid_market() -> Market {
        Market {
            id: "mkt-1".into(),
            name: "Richardson Farmers Market".into(),
            location_lat: 32.95,
            location_lon: -96.73,
            market_type: MarketType::Farmers,
            steward: fake_agent(),
            schedule: "Saturdays 8am-1pm".into(),
        }
    }

    fn valid_listing() -> Listing {
        Listing {
            market_hash: fake_action_hash(),
            producer: fake_agent(),
            product_name: "Heirloom Tomatoes".into(),
            quantity_kg: 10.0,
            price_per_kg: 5.50,
            available_from: 1700000000,
            status: ListingStatus::Available,
            allergen_flags: vec![],
            organic: false,
            cultural_markers: vec![],
        }
    }

    fn valid_order() -> Order {
        Order {
            listing_hash: fake_action_hash(),
            buyer: fake_agent(),
            quantity_kg: 5.0,
            status: OrderStatus::Pending,
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
                    "Expected Invalid containing '{expected_substr}', got: '{msg}'"
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
    fn serde_roundtrip_market_type() {
        let types = vec![
            MarketType::Farmers,
            MarketType::CSA,
            MarketType::FoodBank,
            MarketType::CoOp,
        ];
        for t in &types {
            let json = serde_json::to_string(t).unwrap();
            let back: MarketType = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, t);
        }
    }

    #[test]
    fn serde_roundtrip_listing_status() {
        let statuses = vec![
            ListingStatus::Available,
            ListingStatus::Reserved,
            ListingStatus::Sold,
            ListingStatus::Expired,
        ];
        for s in &statuses {
            let json = serde_json::to_string(s).unwrap();
            let back: ListingStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, s);
        }
    }

    #[test]
    fn serde_roundtrip_order_status() {
        let statuses = vec![
            OrderStatus::Pending,
            OrderStatus::Confirmed,
            OrderStatus::Fulfilled,
            OrderStatus::Cancelled,
        ];
        for s in &statuses {
            let json = serde_json::to_string(s).unwrap();
            let back: OrderStatus = serde_json::from_str(&json).unwrap();
            assert_eq!(&back, s);
        }
    }

    #[test]
    fn serde_roundtrip_market() {
        let m = valid_market();
        let json = serde_json::to_string(&m).unwrap();
        let back: Market = serde_json::from_str(&json).unwrap();
        assert_eq!(back, m);
    }

    #[test]
    fn serde_roundtrip_listing() {
        let l = valid_listing();
        let json = serde_json::to_string(&l).unwrap();
        let back: Listing = serde_json::from_str(&json).unwrap();
        assert_eq!(back, l);
    }

    #[test]
    fn serde_roundtrip_order() {
        let o = valid_order();
        let json = serde_json::to_string(&o).unwrap();
        let back: Order = serde_json::from_str(&json).unwrap();
        assert_eq!(back, o);
    }

    // ── validate_market: id ─────────────────────────────────────────────

    #[test]
    fn valid_market_passes() {
        assert_valid(validate_market(valid_market()));
    }

    #[test]
    fn market_empty_id_rejected() {
        let mut m = valid_market();
        m.id = String::new();
        assert_invalid(validate_market(m), "Market ID cannot be empty");
    }

    #[test]
    fn market_whitespace_id_rejected() {
        let mut m = valid_market();
        m.id = " ".into();
        assert_invalid(validate_market(m), "Market ID cannot be empty");
    }

    // ── validate_market: name ───────────────────────────────────────────

    #[test]
    fn market_empty_name_rejected() {
        let mut m = valid_market();
        m.name = String::new();
        assert_invalid(validate_market(m), "Market name cannot be empty");
    }

    #[test]
    fn market_whitespace_name_rejected() {
        let mut m = valid_market();
        m.name = "  ".into();
        assert_invalid(validate_market(m), "Market name cannot be empty");
    }

    // ── validate_market: latitude ───────────────────────────────────────

    #[test]
    fn market_lat_too_low_rejected() {
        let mut m = valid_market();
        m.location_lat = -91.0;
        assert_invalid(validate_market(m), "Latitude must be between -90 and 90");
    }

    #[test]
    fn market_lat_too_high_rejected() {
        let mut m = valid_market();
        m.location_lat = 91.0;
        assert_invalid(validate_market(m), "Latitude must be between -90 and 90");
    }

    #[test]
    fn market_lat_at_boundary_neg90_valid() {
        let mut m = valid_market();
        m.location_lat = -90.0;
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_lat_at_boundary_pos90_valid() {
        let mut m = valid_market();
        m.location_lat = 90.0;
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_lat_zero_valid() {
        let mut m = valid_market();
        m.location_lat = 0.0;
        assert_valid(validate_market(m));
    }

    // ── validate_market: longitude ──────────────────────────────────────

    #[test]
    fn market_lon_too_low_rejected() {
        let mut m = valid_market();
        m.location_lon = -181.0;
        assert_invalid(validate_market(m), "Longitude must be between -180 and 180");
    }

    #[test]
    fn market_lon_too_high_rejected() {
        let mut m = valid_market();
        m.location_lon = 181.0;
        assert_invalid(validate_market(m), "Longitude must be between -180 and 180");
    }

    #[test]
    fn market_lon_at_boundary_neg180_valid() {
        let mut m = valid_market();
        m.location_lon = -180.0;
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_lon_at_boundary_pos180_valid() {
        let mut m = valid_market();
        m.location_lon = 180.0;
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_lon_zero_valid() {
        let mut m = valid_market();
        m.location_lon = 0.0;
        assert_valid(validate_market(m));
    }

    // ── validate_market: type variants ──────────────────────────────────

    #[test]
    fn market_all_types_valid() {
        for mt in [
            MarketType::Farmers,
            MarketType::CSA,
            MarketType::FoodBank,
            MarketType::CoOp,
        ] {
            let mut m = valid_market();
            m.market_type = mt;
            assert_valid(validate_market(m));
        }
    }

    // ── validate_market: combined invalid ───────────────────────────────

    #[test]
    fn market_empty_id_with_empty_name_rejects_id_first() {
        let mut m = valid_market();
        m.id = String::new();
        m.name = String::new();
        assert_invalid(validate_market(m), "Market ID cannot be empty");
    }

    #[test]
    fn market_empty_name_with_invalid_lat_rejects_name_first() {
        let mut m = valid_market();
        m.name = String::new();
        m.location_lat = 91.0;
        assert_invalid(validate_market(m), "Market name cannot be empty");
    }

    // ── validate_market: schedule is not validated ───────────────────────

    #[test]
    fn market_empty_schedule_accepted() {
        let mut m = valid_market();
        m.schedule = String::new();
        assert_valid(validate_market(m));
    }

    // ── validate_listing: product_name ──────────────────────────────────

    #[test]
    fn valid_listing_passes() {
        assert_valid(validate_listing(valid_listing()));
    }

    #[test]
    fn listing_empty_product_rejected() {
        let mut l = valid_listing();
        l.product_name = String::new();
        assert_invalid(validate_listing(l), "Product name cannot be empty");
    }

    #[test]
    fn listing_whitespace_product_rejected() {
        let mut l = valid_listing();
        l.product_name = " ".into();
        assert_invalid(validate_listing(l), "Product name cannot be empty");
    }

    // ── validate_listing: quantity_kg ────────────────────────────────────

    #[test]
    fn listing_zero_quantity_rejected() {
        let mut l = valid_listing();
        l.quantity_kg = 0.0;
        assert_invalid(validate_listing(l), "Quantity must be positive");
    }

    #[test]
    fn listing_negative_quantity_rejected() {
        let mut l = valid_listing();
        l.quantity_kg = -10.0;
        assert_invalid(validate_listing(l), "Quantity must be positive");
    }

    #[test]
    fn listing_barely_positive_quantity_valid() {
        let mut l = valid_listing();
        l.quantity_kg = 0.001;
        assert_valid(validate_listing(l));
    }

    #[test]
    fn listing_large_quantity_valid() {
        let mut l = valid_listing();
        l.quantity_kg = 99999.0;
        assert_valid(validate_listing(l));
    }

    // ── validate_listing: price_per_kg ──────────────────────────────────

    #[test]
    fn listing_negative_price_rejected() {
        let mut l = valid_listing();
        l.price_per_kg = -1.0;
        assert_invalid(validate_listing(l), "Price cannot be negative");
    }

    #[test]
    fn listing_barely_negative_price_rejected() {
        let mut l = valid_listing();
        l.price_per_kg = -0.001;
        assert_invalid(validate_listing(l), "Price cannot be negative");
    }

    #[test]
    fn listing_zero_price_valid() {
        let mut l = valid_listing();
        l.price_per_kg = 0.0;
        assert_valid(validate_listing(l));
    }

    #[test]
    fn listing_large_price_valid() {
        let mut l = valid_listing();
        l.price_per_kg = 9999.0;
        assert_valid(validate_listing(l));
    }

    // ── validate_listing: status variants ───────────────────────────────

    #[test]
    fn listing_all_statuses_valid() {
        for status in [
            ListingStatus::Available,
            ListingStatus::Reserved,
            ListingStatus::Sold,
            ListingStatus::Expired,
        ] {
            let mut l = valid_listing();
            l.status = status;
            assert_valid(validate_listing(l));
        }
    }

    // ── validate_listing: combined invalid ──────────────────────────────

    #[test]
    fn listing_empty_product_with_zero_quantity_rejects_product_first() {
        let mut l = valid_listing();
        l.product_name = String::new();
        l.quantity_kg = 0.0;
        assert_invalid(validate_listing(l), "Product name cannot be empty");
    }

    // ── validate_order: quantity_kg ─────────────────────────────────────

    #[test]
    fn valid_order_passes() {
        assert_valid(validate_order(valid_order()));
    }

    #[test]
    fn order_zero_quantity_rejected() {
        let mut o = valid_order();
        o.quantity_kg = 0.0;
        assert_invalid(validate_order(o), "Order quantity must be positive");
    }

    #[test]
    fn order_negative_quantity_rejected() {
        let mut o = valid_order();
        o.quantity_kg = -5.0;
        assert_invalid(validate_order(o), "Order quantity must be positive");
    }

    #[test]
    fn order_barely_positive_quantity_valid() {
        let mut o = valid_order();
        o.quantity_kg = 0.001;
        assert_valid(validate_order(o));
    }

    #[test]
    fn order_large_quantity_valid() {
        let mut o = valid_order();
        o.quantity_kg = 99999.0;
        assert_valid(validate_order(o));
    }

    #[test]
    fn order_barely_negative_quantity_rejected() {
        let mut o = valid_order();
        o.quantity_kg = -0.001;
        assert_invalid(validate_order(o), "Order quantity must be positive");
    }

    // ── validate_order: status variants ─────────────────────────────────

    #[test]
    fn order_all_statuses_valid() {
        for status in [
            OrderStatus::Pending,
            OrderStatus::Confirmed,
            OrderStatus::Fulfilled,
            OrderStatus::Cancelled,
        ] {
            let mut o = valid_order();
            o.status = status;
            assert_valid(validate_order(o));
        }
    }

    // ── Anchor test ─────────────────────────────────────────────────────

    #[test]
    fn serde_roundtrip_anchor() {
        let a = Anchor("all_markets".to_string());
        let json = serde_json::to_string(&a).unwrap();
        let back: Anchor = serde_json::from_str(&json).unwrap();
        assert_eq!(back, a);
    }

    // ── Link tag length validation tests ────────────────────────────────

    fn validate_link_tag(link_type: &LinkTypes, tag_len: usize) -> ValidateCallbackResult {
        let tag = LinkTag(vec![0u8; tag_len]);
        let max = match link_type {
            LinkTypes::AllMarkets
            | LinkTypes::MarketToListing
            | LinkTypes::ProducerToListing
            | LinkTypes::ProducerToLot
            | LinkTypes::BuyerToOrder
            | LinkTypes::ListingToOrder
            | LinkTypes::GeoIndex => 256,
        };
        let name = match link_type {
            LinkTypes::AllMarkets => "AllMarkets",
            LinkTypes::MarketToListing => "MarketToListing",
            LinkTypes::ProducerToListing => "ProducerToListing",
            LinkTypes::ProducerToLot => "ProducerToLot",
            LinkTypes::BuyerToOrder => "BuyerToOrder",
            LinkTypes::ListingToOrder => "ListingToOrder",
            LinkTypes::GeoIndex => "GeoIndex",
        };
        if tag.0.len() > max {
            ValidateCallbackResult::Invalid(format!(
                "{} link tag too long (max {} bytes)",
                name, max
            ))
        } else {
            ValidateCallbackResult::Valid
        }
    }

    #[test]
    fn test_link_all_markets_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::AllMarkets, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_all_markets_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::AllMarkets, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_market_to_listing_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::MarketToListing, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_market_to_listing_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::MarketToListing, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_producer_to_listing_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::ProducerToListing, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_producer_to_listing_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::ProducerToListing, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_buyer_to_order_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::BuyerToOrder, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_buyer_to_order_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::BuyerToOrder, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_link_listing_to_order_tag_at_max_accepted() {
        let result = validate_link_tag(&LinkTypes::ListingToOrder, 256);
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_link_listing_to_order_tag_over_max_rejected() {
        let result = validate_link_tag(&LinkTypes::ListingToOrder, 257);
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // ── NaN/Infinity bypass hardening tests ────────────────────────────

    #[test]
    fn market_nan_lat_rejected() {
        let mut m = valid_market();
        m.location_lat = f64::NAN;
        assert_invalid(validate_market(m), "Latitude must be a finite number");
    }

    #[test]
    fn market_infinity_lat_rejected() {
        let mut m = valid_market();
        m.location_lat = f64::INFINITY;
        assert_invalid(validate_market(m), "Latitude must be a finite number");
    }

    #[test]
    fn market_nan_lon_rejected() {
        let mut m = valid_market();
        m.location_lon = f64::NAN;
        assert_invalid(validate_market(m), "Longitude must be a finite number");
    }

    #[test]
    fn market_infinity_lon_rejected() {
        let mut m = valid_market();
        m.location_lon = f64::INFINITY;
        assert_invalid(validate_market(m), "Longitude must be a finite number");
    }

    #[test]
    fn listing_nan_quantity_rejected() {
        let mut l = valid_listing();
        l.quantity_kg = f64::NAN;
        assert_invalid(validate_listing(l), "Quantity must be a finite number");
    }

    #[test]
    fn listing_infinity_quantity_rejected() {
        let mut l = valid_listing();
        l.quantity_kg = f64::INFINITY;
        assert_invalid(validate_listing(l), "Quantity must be a finite number");
    }

    #[test]
    fn listing_nan_price_rejected() {
        let mut l = valid_listing();
        l.price_per_kg = f64::NAN;
        assert_invalid(validate_listing(l), "Price must be a finite number");
    }

    #[test]
    fn listing_infinity_price_rejected() {
        let mut l = valid_listing();
        l.price_per_kg = f64::INFINITY;
        assert_invalid(validate_listing(l), "Price must be a finite number");
    }

    #[test]
    fn order_nan_quantity_rejected() {
        let mut o = valid_order();
        o.quantity_kg = f64::NAN;
        assert_invalid(validate_order(o), "Quantity must be a finite number");
    }

    #[test]
    fn order_infinity_quantity_rejected() {
        let mut o = valid_order();
        o.quantity_kg = f64::INFINITY;
        assert_invalid(validate_order(o), "Quantity must be a finite number");
    }

    // ── Serde roundtrip: Listing with new fields ────────────────────────

    #[test]
    fn serde_roundtrip_listing_with_new_fields() {
        let l = Listing {
            market_hash: fake_action_hash(),
            producer: fake_agent(),
            product_name: "Organic Kale".into(),
            quantity_kg: 5.0,
            price_per_kg: 3.00,
            available_from: 1700000000,
            status: ListingStatus::Available,
            allergen_flags: vec!["gluten".into(), "soy".into()],
            organic: true,
            cultural_markers: vec!["halal".into(), "kosher".into()],
        };
        let json = serde_json::to_string(&l).unwrap();
        let back: Listing = serde_json::from_str(&json).unwrap();
        assert_eq!(back, l);
    }

    #[test]
    fn serde_roundtrip_listing_with_allergens_and_cultural_markers() {
        let l = Listing {
            market_hash: fake_action_hash(),
            producer: fake_agent(),
            product_name: "Tamales".into(),
            quantity_kg: 20.0,
            price_per_kg: 12.00,
            available_from: 1700000000,
            status: ListingStatus::Available,
            allergen_flags: vec!["corn".into(), "dairy".into(), "lard".into()],
            organic: false,
            cultural_markers: vec!["mexican".into(), "traditional".into()],
        };
        let json = serde_json::to_string(&l).unwrap();
        let back: Listing = serde_json::from_str(&json).unwrap();
        assert_eq!(back.allergen_flags, vec!["corn", "dairy", "lard"]);
        assert_eq!(back.cultural_markers, vec!["mexican", "traditional"]);
        assert!(!back.organic);
    }

    // ── Allergen flag validation tests ──────────────────────────────────

    #[test]
    fn listing_with_allergens_valid() {
        let mut l = valid_listing();
        l.allergen_flags = vec!["gluten".into(), "dairy".into(), "nuts".into()];
        assert_valid(validate_listing(l));
    }

    #[test]
    fn listing_too_many_allergens_rejected() {
        let mut l = valid_listing();
        l.allergen_flags = (0..51).map(|i| format!("allergen_{i}")).collect();
        assert_invalid(
            validate_listing(l),
            "Cannot have more than 50 allergen flags",
        );
    }

    #[test]
    fn listing_empty_allergen_flag_rejected() {
        let mut l = valid_listing();
        l.allergen_flags = vec!["gluten".into(), "".into()];
        assert_invalid(validate_listing(l), "Allergen flag cannot be empty");
    }

    #[test]
    fn listing_allergen_flag_too_long_rejected() {
        let mut l = valid_listing();
        l.allergen_flags = vec!["x".repeat(129)];
        assert_invalid(
            validate_listing(l),
            "Allergen flag too long (max 128 chars)",
        );
    }

    #[test]
    fn listing_exactly_50_allergens_accepted() {
        let mut l = valid_listing();
        l.allergen_flags = (0..50).map(|i| format!("allergen_{i}")).collect();
        assert_valid(validate_listing(l));
    }

    // ── Organic flag validation tests ───────────────────────────────────

    #[test]
    fn listing_organic_true_valid() {
        let mut l = valid_listing();
        l.organic = true;
        assert_valid(validate_listing(l));
    }

    // ── Cultural markers validation tests ───────────────────────────────

    #[test]
    fn listing_cultural_markers_valid() {
        let mut l = valid_listing();
        l.cultural_markers = vec!["halal".into(), "kosher".into(), "vegan".into()];
        assert_valid(validate_listing(l));
    }

    #[test]
    fn listing_too_many_cultural_markers_rejected() {
        let mut l = valid_listing();
        l.cultural_markers = (0..51).map(|i| format!("marker_{i}")).collect();
        assert_invalid(
            validate_listing(l),
            "Cannot have more than 50 cultural markers",
        );
    }

    #[test]
    fn listing_empty_cultural_marker_rejected() {
        let mut l = valid_listing();
        l.cultural_markers = vec!["halal".into(), " ".into()];
        assert_invalid(validate_listing(l), "Cultural marker cannot be empty");
    }

    #[test]
    fn listing_cultural_marker_too_long_rejected() {
        let mut l = valid_listing();
        l.cultural_markers = vec!["x".repeat(129)];
        assert_invalid(
            validate_listing(l),
            "Cultural marker too long (max 128 chars)",
        );
    }

    // ── String length limit boundary tests ────────────────────────────────

    #[test]
    fn market_id_at_limit_accepted() {
        let mut m = valid_market();
        m.id = "A".repeat(64);
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_id_over_limit_rejected() {
        let mut m = valid_market();
        m.id = "A".repeat(257);
        assert_invalid(
            validate_market(m),
            "Market ID must be 256 characters or fewer",
        );
    }

    #[test]
    fn market_name_at_limit_accepted() {
        let mut m = valid_market();
        m.name = "A".repeat(256);
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_name_over_limit_rejected() {
        let mut m = valid_market();
        m.name = "A".repeat(257);
        assert_invalid(
            validate_market(m),
            "Market name must be 256 characters or fewer",
        );
    }

    #[test]
    fn market_schedule_at_limit_accepted() {
        let mut m = valid_market();
        m.schedule = "S".repeat(256);
        assert_valid(validate_market(m));
    }

    #[test]
    fn market_schedule_over_limit_rejected() {
        let mut m = valid_market();
        m.schedule = "S".repeat(257);
        assert_invalid(
            validate_market(m),
            "Schedule must be 256 characters or fewer",
        );
    }

    #[test]
    fn listing_product_name_at_limit_accepted() {
        let mut l = valid_listing();
        l.product_name = "P".repeat(256);
        assert_valid(validate_listing(l));
    }

    #[test]
    fn listing_product_name_over_limit_rejected() {
        let mut l = valid_listing();
        l.product_name = "P".repeat(257);
        assert_invalid(
            validate_listing(l),
            "Product name must be 256 characters or fewer",
        );
    }
    fn valid_producer_lot(author: &AgentPubKey) -> food_lot_reconciliation::FoodLotProvenanceV1 {
        food_lot_reconciliation::FoodLotProvenanceV1 {
            schema_version: 1,
            lot_ref: "mycelix:food-lot:rice-lot-2026-001".to_owned(),
            revision: 1,
            product_ref: "product:rice-brown".to_owned(),
            producer_ref: did_for_author(author),
            supplier_lot_code: Some("SUPPLIER-RICE-001".to_owned()),
            listing_ref: None,
            origin_ref: None,
            quantity: food_lot_reconciliation::FoodLotQuantityV1 {
                dimension: "mass".to_owned(),
                value: 10_000,
                decimal_places: 0,
                unit: "g".to_owned(),
            },
            harvested_at: None,
            packed_at: None,
            best_before_at: None,
            use_by_at: None,
            handling_ref: None,
            evidence: food_lot_reconciliation::FoodLotEvidenceV1 {
                source_ref: "producer-declaration:rice-lot-001".to_owned(),
                evidence_class: "Declared".to_owned(),
                method_ref: None,
                calibration_ref: None,
            },
            claim_ceiling: food_lot_reconciliation::FOOD_LOT_PROVENANCE_CLAIM_CEILING.to_owned(),
        }
    }

    fn producer_lot_revision_record(
        author: &AgentPubKey,
        revision: u32,
        previous_revision: Option<ActionHash>,
    ) -> ProducerLotRecordV1 {
        let mut lot = valid_producer_lot(author);
        lot.revision = revision;
        ProducerLotRecordV1 {
            lot,
            producer_author: author.clone(),
            previous_revision,
        }
    }

    #[test]
    fn producer_lot_revision_one_requires_no_predecessor() {
        let author = fake_agent();
        let first = producer_lot_revision_record(&author, 1, None);
        assert!(validate_producer_lot_revision_fields(&first, &author, None).is_ok());

        let forged_first = producer_lot_revision_record(&author, 1, Some(fake_action_hash()));
        assert!(validate_producer_lot_revision_fields(&forged_first, &author, None).is_err());
    }

    #[test]
    fn producer_lot_revision_two_requires_exact_same_author_lot_product_predecessor() {
        let author = fake_agent();
        let first = producer_lot_revision_record(&author, 1, None);
        let second = producer_lot_revision_record(&author, 2, Some(fake_action_hash()));
        assert!(
            validate_producer_lot_revision_fields(&second, &author, Some((&first, &author)))
                .is_ok()
        );
    }

    #[test]
    fn producer_lot_revision_rejects_revision_gaps_and_author_changes() {
        let author = fake_agent();
        let other = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let first = producer_lot_revision_record(&author, 1, None);
        let skipped = producer_lot_revision_record(&author, 3, Some(fake_action_hash()));
        assert!(
            validate_producer_lot_revision_fields(&skipped, &author, Some((&first, &author)))
                .is_err()
        );

        let second = producer_lot_revision_record(&author, 2, Some(fake_action_hash()));
        assert!(
            validate_producer_lot_revision_fields(&second, &author, Some((&first, &other)))
                .is_err()
        );
    }

    #[test]
    fn producer_lot_revision_rejects_cross_lot_and_cross_product_predecessors() {
        let author = fake_agent();
        let first = producer_lot_revision_record(&author, 1, None);
        let mut other_lot = producer_lot_revision_record(&author, 1, None);
        other_lot.lot.lot_ref = "mycelix:food-lot:another-lot".to_owned();
        let mut other_product = producer_lot_revision_record(&author, 1, None);
        other_product.lot.product_ref = "product:other-rice".to_owned();
        let second = producer_lot_revision_record(&author, 2, Some(fake_action_hash()));
        assert!(
            validate_producer_lot_revision_fields(&second, &author, Some((&other_lot, &author)))
                .is_err()
        );
        assert!(
            validate_producer_lot_revision_fields(&second, &author, Some((&other_product, &author)))
                .is_err()
        );
        assert!(
            validate_producer_lot_revision_fields(&second, &author, Some((&first, &author)))
                .is_ok()
        );
    }

    #[test]
    fn producer_lot_typed_author_must_equal_signed_action_author() {
        let author = fake_agent();
        let other = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let record = ProducerLotRecordV1 {
            lot: valid_producer_lot(&author),
            producer_author: author.clone(),
            previous_revision: None,
        };
        assert!(validate_producer_lot_record_author(&record, &author).is_ok());

        let forged = ProducerLotRecordV1 {
            producer_author: other,
            ..record
        };
        assert!(validate_producer_lot_record_author(&forged, &author).is_err());
    }

    #[test]
    fn producer_lot_requires_exact_signed_author_did_binding() {
        let author = fake_agent();
        let other = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let valid = valid_producer_lot(&author);
        assert!(validate_producer_lot_fields(&valid, &author).is_ok());

        let forged = valid_producer_lot(&other);
        assert!(validate_producer_lot_fields(&forged, &author).is_err());
    }

    #[test]
    fn producer_lot_requires_positive_exact_quantity_and_supported_schema() {
        let author = fake_agent();
        let mut lot = valid_producer_lot(&author);
        lot.quantity.value = 0;
        assert!(validate_producer_lot_fields(&lot, &author).is_err());

        let mut lot = valid_producer_lot(&author);
        lot.quantity.dimension = "volume".to_owned();
        assert!(validate_producer_lot_fields(&lot, &author).is_err());

        let mut lot = valid_producer_lot(&author);
        lot.schema_version = 2;
        assert!(validate_producer_lot_fields(&lot, &author).is_err());
    }

    #[test]
    fn producer_lot_rejects_claim_ceiling_and_evidence_class_forgery() {
        let author = fake_agent();
        let mut lot = valid_producer_lot(&author);
        lot.claim_ceiling = "food is safe".to_owned();
        assert!(validate_producer_lot_fields(&lot, &author).is_err());

        let mut lot = valid_producer_lot(&author);
        lot.evidence.evidence_class = "IndependentlyQualified".to_owned();
        assert!(validate_producer_lot_fields(&lot, &author).is_err());

        let mut lot = valid_producer_lot(&author);
        lot.evidence.evidence_class = "Measured".to_owned();
        assert!(validate_producer_lot_fields(&lot, &author).is_err());
    }

    #[test]
    fn producer_lot_rejects_invalid_identity_and_date_order() {
        let author = fake_agent();
        let mut lot = valid_producer_lot(&author);
        lot.lot_ref = "listing:only-not-a-lot".to_owned();
        assert!(validate_producer_lot_fields(&lot, &author).is_err());

        let mut lot = valid_producer_lot(&author);
        lot.harvested_at = Some(200);
        lot.packed_at = Some(100);
        assert!(validate_producer_lot_fields(&lot, &author).is_err());
    }

    #[test]
    fn same_supplier_lot_code_from_different_agents_has_distinct_signed_identity() {
        let first_author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let second_author = AgentPubKey::from_raw_36(vec![2u8; 36]);
        let first = valid_producer_lot(&first_author);
        let second = valid_producer_lot(&second_author);
        assert_eq!(first.supplier_lot_code, second.supplier_lot_code);
        assert_ne!(first.producer_ref, second.producer_ref);
        assert!(validate_producer_lot_fields(&first, &first_author).is_ok());
        assert!(validate_producer_lot_fields(&second, &second_author).is_ok());
    }
}
