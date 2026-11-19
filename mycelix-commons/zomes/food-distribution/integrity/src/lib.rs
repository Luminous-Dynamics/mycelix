// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Food Distribution Integrity Zome
//! Entry types and validation for local food markets, listings, and orders.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{
    check_author_match, check_link_author_match, did_for_author, require_did_is_author,
};

pub mod food_lot_reconciliation;

/// Producer-authored lot provenance stored as an immutable, signed Holochain
/// entry. Later revisions must point to the exact previous revision action.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct FoodLotProvenanceEntryV1 {
    pub provenance: food_lot_reconciliation::FoodLotProvenanceV1,
    pub previous_revision_action: Option<ActionHash>,
}

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
    /// Append-only producer lot provenance; appended to preserve existing entry indices.
    FoodLotProvenance(FoodLotProvenanceEntryV1),
}

#[hdk_link_types]
pub enum LinkTypes {
    AllMarkets,
    MarketToListing,
    ProducerToListing,
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
                EntryTypes::FoodLotProvenance(entry) => validate_create_food_lot_provenance(
                    EntryCreationAction::Create(action),
                    entry,
                ),
            },
            OpEntry::UpdateEntry { app_entry, action, .. } => match app_entry {
                EntryTypes::Market(m) => validate_market(m),
                EntryTypes::Listing(l) => validate_listing(l),
                EntryTypes::Order(o) => validate_order(o),
                EntryTypes::FoodLotProvenance(_) => Ok(ValidateCallbackResult::Invalid(
                    "Food lot provenance revisions are immutable; create a linked successor revision".into(),
                )),
                _ => {
                    let _ = action;
                    Ok(ValidateCallbackResult::Valid)
                }
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
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_valid_record(action.original_action_address.clone())?;
            if matches!(
                original.entry().to_app_option::<EntryTypes>()?,
                Some(EntryTypes::FoodLotProvenance(_))
            ) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Food lot provenance entries are immutable and cannot be updated".into(),
                ));
            }
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "update",
            ))
        }
        FlatOp::RegisterDelete(OpDelete { action, .. }) => {
            let original = must_get_valid_record(action.deletes_address.clone())?;
            if matches!(
                original.entry().to_app_option::<EntryTypes>()?,
                Some(EntryTypes::FoodLotProvenance(_))
            ) {
                return Ok(ValidateCallbackResult::Invalid(
                    "Food lot provenance entries are immutable and cannot be deleted".into(),
                ));
            }
            Ok(check_author_match(
                original.action().author(),
                &action.author,
                "delete",
            ))
        }
    }
}

const FOOD_LOT_PROVENANCE_SCHEMA_VERSION: u32 = 1;
const FOOD_LOT_PROVENANCE_CLAIM_CEILING: &str = "Supplier/producer lot identity and source-declared quantity only; not proof of household receipt, freshness, food safety, allergen status or suitability.";

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn validate_food_lot_quantity(quantity: &food_lot_reconciliation::FoodLotQuantityV1) -> bool {
    if quantity.value == 0 || quantity.decimal_places > 6 {
        return false;
    }
    match (quantity.dimension.as_str(), quantity.unit.as_str()) {
        ("mass", "mg" | "g" | "kg")
        | ("volume", "mL" | "L")
        | ("count", "count" | "piece") => true,
        _ => false,
    }
}

/// Pure field and author-binding checks. The DID is supplied by the signed
/// Create action, never accepted from an untrusted coordinator request.
fn validate_food_lot_provenance_declaration(
    lot: &food_lot_reconciliation::FoodLotProvenanceV1,
    author_did: &str,
) -> ValidateCallbackResult {
    if lot.schema_version != FOOD_LOT_PROVENANCE_SCHEMA_VERSION {
        return ValidateCallbackResult::Invalid("Unsupported FoodLotProvenanceV1 schema version".into());
    }
    let lot_suffix = lot.lot_ref.strip_prefix("mycelix:food-lot:");
    if lot.lot_ref.len() > 150
        || lot_suffix.is_none_or(|suffix| {
            suffix.is_empty()
                || suffix.len() > 128
                || !(suffix.as_bytes()[0].is_ascii_lowercase()
                    || suffix.as_bytes()[0].is_ascii_digit())
                || !suffix.bytes().all(|byte| {
                    byte.is_ascii_lowercase()
                        || byte.is_ascii_digit()
                        || matches!(byte, b'.' | b'_' | b':' | b'-')
                })
        })
        || !nonempty(&lot.product_ref)
        || lot.product_ref.len() > 512
        || !nonempty(&lot.producer_ref)
        || lot.producer_ref.len() > 512
        || lot.revision == 0
    {
        return ValidateCallbackResult::Invalid(
            "Invalid food lot identity, product, producer or revision".into(),
        );
    }
    if lot.supplier_lot_code.as_deref().is_some_and(|value| value.is_empty() || value.len() > 128)
        || lot.listing_ref.as_deref().is_some_and(|value| value.is_empty() || value.len() > 512)
        || lot.origin_ref.as_deref().is_some_and(|value| value.is_empty() || value.len() > 512)
        || lot.handling_ref.as_deref().is_some_and(|value| value.is_empty() || value.len() > 512)
    {
        return ValidateCallbackResult::Invalid("Optional food lot references must be non-empty and within schema limits".into());
    }
    if !validate_food_lot_quantity(&lot.quantity) {
        return ValidateCallbackResult::Invalid("Food lot quantity must be positive, exact and dimensionally valid".into());
    }
    let source = &lot.evidence.source_ref;
    if !nonempty(source) || source.len() > 512 {
        return ValidateCallbackResult::Invalid("Food lot evidence source_ref is required".into());
    }
    let evidence_class = lot.evidence.evidence_class.as_str();
    if !matches!(
        evidence_class,
        "Declared" | "Attested" | "Modelled" | "Measured" | "IndependentlyAssayed" | "IndependentlyQualified"
    ) {
        return ValidateCallbackResult::Invalid("Unsupported food lot evidence class".into());
    }
    if evidence_class == "Measured"
        && lot.evidence.calibration_ref.as_deref().is_none_or(|value| !nonempty(value))
    {
        return ValidateCallbackResult::Invalid("Measured food lot evidence requires calibration_ref".into());
    }
    if matches!(evidence_class, "IndependentlyAssayed" | "IndependentlyQualified")
        && lot.evidence.method_ref.as_deref().is_none_or(|value| !nonempty(value))
    {
        return ValidateCallbackResult::Invalid("Independent food lot evidence requires method_ref".into());
    }
    if lot.evidence.method_ref.as_deref().is_some_and(|value| value.is_empty() || value.len() > 512)
        || lot.evidence.calibration_ref.as_deref().is_some_and(|value| value.is_empty() || value.len() > 512)
    {
        return ValidateCallbackResult::Invalid("Optional evidence references must not be empty strings".into());
    }
    if [
        lot.harvested_at,
        lot.packed_at,
        lot.best_before_at,
        lot.use_by_at,
    ]
    .into_iter()
    .flatten()
    .any(|timestamp| timestamp == 0)
    {
        return ValidateCallbackResult::Invalid(
            "Known food lot timestamps must be positive epoch values".into(),
        );
    }
    let source_date = lot.packed_at.or(lot.harvested_at);
    if let Some(source_date) = source_date {
        if lot.best_before_at.is_some_and(|end| end < source_date)
            || lot.use_by_at.is_some_and(|end| end < source_date)
        {
            return ValidateCallbackResult::Invalid("Best-before/use-by dates cannot precede known harvest/packing time".into());
        }
    }
    if lot.harvested_at.zip(lot.packed_at).is_some_and(|(harvested, packed)| packed < harvested) {
        return ValidateCallbackResult::Invalid("packed_at cannot precede harvested_at".into());
    }
    if lot.claim_ceiling != FOOD_LOT_PROVENANCE_CLAIM_CEILING {
        return ValidateCallbackResult::Invalid("Food lot provenance claim ceiling mismatch".into());
    }
    if let ValidateCallbackResult::Invalid(message) = require_did_is_author(
        "FoodLotProvenanceV1",
        "producer_ref",
        &lot.producer_ref,
        author_did,
    ) {
        return ValidateCallbackResult::Invalid(message);
    }
    ValidateCallbackResult::Valid
}

/// Check the append-only revision relation without performing any Holochain I/O.
fn validate_food_lot_revision_successor(
    current: &food_lot_reconciliation::FoodLotProvenanceV1,
    previous: Option<&food_lot_reconciliation::FoodLotProvenanceV1>,
) -> ValidateCallbackResult {
    match (current.revision, previous) {
        (1, None) => ValidateCallbackResult::Valid,
        (1, Some(_)) => ValidateCallbackResult::Invalid("Initial food lot revision must not have a predecessor".into()),
        (_, None) => ValidateCallbackResult::Invalid("Food lot revisions after revision 1 require a predecessor action".into()),
        (revision, Some(previous)) => {
            let Some(expected_revision) = previous.revision.checked_add(1) else {
                return ValidateCallbackResult::Invalid("Food lot predecessor revision overflow".into());
            };
            if revision != expected_revision
                || current.lot_ref != previous.lot_ref
                || current.product_ref != previous.product_ref
                || current.producer_ref != previous.producer_ref
            {
                return ValidateCallbackResult::Invalid(
                    "Food lot successor must increment the exact predecessor revision and preserve lot, product and producer identity".into(),
                );
            }
            ValidateCallbackResult::Valid
        }
    }
}

fn validate_create_food_lot_provenance(
    action: EntryCreationAction,
    entry: FoodLotProvenanceEntryV1,
) -> ExternResult<ValidateCallbackResult> {
    let author_did = did_for_author(action.author());
    let field_report = validate_food_lot_provenance_declaration(&entry.provenance, &author_did);
    if matches!(field_report, ValidateCallbackResult::Invalid(_)) {
        return Ok(field_report);
    }
    match (&entry.previous_revision_action, entry.provenance.revision) {
        (None, 1) => Ok(validate_food_lot_revision_successor(&entry.provenance, None)),
        (None, _) => Ok(ValidateCallbackResult::Invalid(
            "Food lot revision after 1 must reference its predecessor action".into(),
        )),
        (Some(_), 1) => Ok(ValidateCallbackResult::Invalid(
            "Initial food lot revision cannot have a predecessor action".into(),
        )),
        (Some(previous_action), _) => {
            let previous_record = must_get_valid_record(previous_action.clone())?;
            let previous_entry = match previous_record.entry().to_app_option::<EntryTypes>()? {
                Some(EntryTypes::FoodLotProvenance(previous_entry)) => previous_entry,
                _ => {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Food lot predecessor action is not a FoodLotProvenanceEntryV1 record".into(),
                    ));
                }
            };
            Ok(validate_food_lot_revision_successor(
                &entry.provenance,
                Some(&previous_entry.provenance),
            ))
        }
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
            | LinkTypes::BuyerToOrder
            | LinkTypes::ListingToOrder
            | LinkTypes::GeoIndex => 256,
        };
        let name = match link_type {
            LinkTypes::AllMarkets => "AllMarkets",
            LinkTypes::MarketToListing => "MarketToListing",
            LinkTypes::ProducerToListing => "ProducerToListing",
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
}


#[cfg(test)]
mod food_lot_provenance_tests {
    use super::*;
    use food_lot_reconciliation::{
        FoodLotEvidenceV1, FoodLotProvenanceV1, FoodLotQuantityV1,
    };

    fn valid_lot() -> FoodLotProvenanceV1 {
        FoodLotProvenanceV1 {
            schema_version: 1,
            lot_ref: "mycelix:food-lot:rice-2026-01".to_owned(),
            revision: 1,
            product_ref: "mycelix:product:rice-brown".to_owned(),
            producer_ref: "did:mycelix:producer-01".to_owned(),
            supplier_lot_code: Some("RICE-2026-01".to_owned()),
            listing_ref: None,
            origin_ref: Some("origin:cooperative-01".to_owned()),
            quantity: FoodLotQuantityV1 {
                dimension: "mass".to_owned(),
                value: 25,
                decimal_places: 0,
                unit: "kg".to_owned(),
            },
            harvested_at: Some(1_760_000_000),
            packed_at: Some(1_760_100_000),
            best_before_at: Some(1_800_000_000),
            use_by_at: None,
            handling_ref: None,
            evidence: FoodLotEvidenceV1 {
                source_ref: "source:producer-record-01".to_owned(),
                evidence_class: "Declared".to_owned(),
                method_ref: None,
                calibration_ref: None,
            },
            claim_ceiling: FOOD_LOT_PROVENANCE_CLAIM_CEILING.to_owned(),
        }
    }

    fn assert_invalid(result: ValidateCallbackResult) {
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn producer_did_must_match_the_signed_author() {
        let mut lot = valid_lot();
        lot.producer_ref = "did:mycelix:actual-author".to_owned();
        assert_eq!(
            validate_food_lot_provenance_declaration(&lot, "did:mycelix:actual-author"),
            ValidateCallbackResult::Valid
        );
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            "did:mycelix:attacker",
        ));
    }

    #[test]
    fn initial_lot_revision_has_no_predecessor() {
        let lot = valid_lot();
        assert_eq!(
            validate_food_lot_revision_successor(&lot, None),
            ValidateCallbackResult::Valid
        );
        assert_invalid(validate_food_lot_revision_successor(&lot, Some(&lot)));
    }

    #[test]
    fn successor_revision_requires_exact_previous_lot_product_and_producer() {
        let previous = valid_lot();
        let mut successor = previous.clone();
        successor.revision = 2;
        assert_eq!(
            validate_food_lot_revision_successor(&successor, Some(&previous)),
            ValidateCallbackResult::Valid
        );

        let mut forked_revision = successor.clone();
        forked_revision.revision = 3;
        assert_invalid(validate_food_lot_revision_successor(
            &forked_revision,
            Some(&previous),
        ));

        let mut changed_product = successor.clone();
        changed_product.product_ref = "mycelix:product:other".to_owned();
        assert_invalid(validate_food_lot_revision_successor(
            &changed_product,
            Some(&previous),
        ));

        let mut changed_producer = successor.clone();
        changed_producer.producer_ref = "did:mycelix:other-producer".to_owned();
        assert_invalid(validate_food_lot_revision_successor(
            &changed_producer,
            Some(&previous),
        ));
    }

    #[test]
    fn later_revision_without_predecessor_is_rejected() {
        let mut successor = valid_lot();
        successor.revision = 2;
        assert_invalid(validate_food_lot_revision_successor(&successor, None));
    }

    #[test]
    fn lot_quantity_and_dates_are_dimensionally_and_temporally_validated() {
        let mut lot = valid_lot();
        lot.quantity.dimension = "mass".to_owned();
        lot.quantity.unit = "L".to_owned();
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            &lot.producer_ref,
        ));

        let mut lot = valid_lot();
        lot.packed_at = Some(1_759_000_000);
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            &lot.producer_ref,
        ));

        let mut lot = valid_lot();
        lot.harvested_at = Some(0);
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            &lot.producer_ref,
        ));
    }

    #[test]
    fn entry_schema_and_integrity_challenge_manifest_are_versioned() {
        let schema: serde_json::Value = serde_json::from_str(include_str!(
            "../schemas/FoodLotProvenanceEntryV1.schema.json"
        ))
        .expect("FoodLotProvenanceEntryV1 schema must parse");
        let fixtures: serde_json::Value = serde_json::from_str(include_str!(
            "../schemas/fixtures/food-lot-provenance-entry-v1.json"
        ))
        .expect("food lot provenance fixture bundle must parse");

        assert_eq!(
            schema["$id"],
            "https://luminousdynamics.io/schemas/food/FoodLotProvenanceEntryV1.schema.json"
        );
        assert_eq!(
            schema["properties"]["provenance"]["$ref"],
            "https://luminousdynamics.io/schemas/food/FoodLotProvenanceV1.schema.json"
        );
        assert_eq!(fixtures["status"], "synthetic_integrity_contract_only");
        let scenarios = fixtures["scenarios"].as_array().unwrap();
        assert_eq!(scenarios.len(), 9);
        let mut ids = std::collections::BTreeSet::new();
        for scenario in scenarios {
            assert!(ids.insert(scenario["scenario_id"].as_str().unwrap()));
        }
    }

    #[test]
    fn measured_and_independent_evidence_have_required_supporting_refs() {
        let mut lot = valid_lot();
        lot.evidence.evidence_class = "Measured".to_owned();
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            &lot.producer_ref,
        ));

        lot.evidence.calibration_ref = Some("calibration:scale-01".to_owned());
        assert_eq!(
            validate_food_lot_provenance_declaration(&lot, &lot.producer_ref),
            ValidateCallbackResult::Valid
        );

        lot.evidence.evidence_class = "IndependentlyAssayed".to_owned();
        lot.evidence.method_ref = None;
        assert_invalid(validate_food_lot_provenance_declaration(
            &lot,
            &lot.producer_ref,
        ));
    }
}
