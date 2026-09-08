// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Bridge Integrity Zome
//!
//! Defines entry types and validation for cross-hApp climate verification.
//! Uses HDI 0.7.0-dev.1 with FlatOp validation pattern.

use hdi::prelude::*;
use mycelix_bridge_entry_types::{
    check_author_match, check_link_author_match, did_for_author, require_did_is_author,
    CrossClusterNotification,
};

/// Anchor entry for creating deterministic link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

/// Purpose of a climate query
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum QueryPurpose {
    /// Verify the authenticity of a carbon credit
    CreditVerification,
    /// Audit a carbon footprint
    FootprintAudit,
    /// Due diligence on a climate project
    ProjectDueDiligence,
}

/// Status of a query
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum QueryStatus {
    /// Query submitted, awaiting processing
    Pending,
    /// Query is being processed
    Processing,
    /// Query completed with results
    Completed,
    /// Query failed
    Failed,
}

/// Result type for climate queries
#[hdk_entry_helper]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum VerificationResult {
    /// Verification passed
    Verified,
    /// Verification failed
    Failed,
    /// Inconclusive - needs more information
    Inconclusive,
    /// Target not found
    NotFound,
}

/// A cross-hApp query for climate verification
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ClimateQuery {
    /// Unique query identifier
    pub query_id: String,
    /// Purpose of the query
    pub purpose: QueryPurpose,
    /// DID of the requester
    pub requester_did: String,
    /// Target ID (credit ID, footprint ID, or project ID)
    pub target_id: String,
    /// Optional additional parameters as JSON
    pub parameters: Option<String>,
    /// Query status
    pub status: QueryStatus,
    /// Timestamp when query was created
    pub created_at: i64,
}

/// Result of a climate query
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ClimateResult {
    /// ID of the associated query
    pub query_id: String,
    /// Verification result
    pub result: VerificationResult,
    /// Detailed result data as JSON
    pub data: Option<String>,
    /// DID of the responder
    pub responder_did: String,
    /// Timestamp when result was created
    pub responded_at: i64,
    /// Digital signature of the result
    pub signature: Option<String>,
}

/// Cross-hApp marketplace listing for carbon credits
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MarketplaceListing {
    /// Listing ID
    pub listing_id: String,
    /// Credit ID being listed
    pub credit_id: String,
    /// Credit action hash from the carbon zome
    pub credit_action_hash: String,
    /// Project ID the credit belongs to
    pub project_id: String,
    /// Seller DID
    pub seller_did: String,
    /// Price per tonne in base currency units
    pub price_per_tonne: u64,
    /// Currency code (e.g., "USD", "EUR")
    pub currency: String,
    /// Minimum purchase in tonnes
    pub min_purchase: f64,
    /// Available tonnes for sale
    pub available_tonnes: f64,
    /// Listing expiry (Unix timestamp)
    pub expires_at: i64,
    /// Whether listing is active
    pub is_active: bool,
    /// Timestamp when listing was created
    pub created_at: i64,
}

/// Entry types for the bridge zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    ClimateQuery(ClimateQuery),
    #[entry_type(visibility = "public")]
    ClimateResult(ClimateResult),
    #[entry_type(visibility = "public")]
    MarketplaceListing(MarketplaceListing),
    Notification(CrossClusterNotification),
}

/// Link types for the bridge zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all queries
    AnchorToQueries,
    /// Query to its result
    QueryToResult,
    /// Requester to their queries
    RequesterToQueries,
    /// Anchor to marketplace listings
    AnchorToListings,
    /// Seller to their listings
    SellerToListings,
    /// Credit to listings
    CreditToListings,
    AgentToNotification,
    AllNotifications,
    NotificationSubscription,
}

/// Validate DIDs have proper format
fn validate_did(did: &str) -> ExternResult<ValidateCallbackResult> {
    if did.is_empty() {
        return Ok(ValidateCallbackResult::Invalid("DID cannot be empty".to_string()));
    }
    if !did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "DID must start with 'did:' prefix".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

/// Bind a self-reported actor DID to the Holochain author that committed the entry.
///
/// This is intentionally used only where the committer is unambiguously the actor:
/// query requester, result responder, and marketplace seller. It must not be reused
/// for on-behalf-of or shared-state subjects without a domain-specific witness rule.
fn validate_actor_binding(
    entry: &str,
    field: &str,
    did: &str,
    author: &AgentPubKey,
) -> ValidateCallbackResult {
    let author_did = did_for_author(author);
    require_did_is_author(entry, field, did, &author_did)
}

/// Validate a ClimateQuery entry
fn validate_climate_query(query: &ClimateQuery) -> ExternResult<ValidateCallbackResult> {
    // Validate query ID
    if query.query_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query ID cannot be empty".to_string(),
        ));
    }

    // Validate requester DID
    let did_result = validate_did(&query.requester_did)?;
    if let ValidateCallbackResult::Invalid(_) = did_result {
        return Ok(did_result);
    }

    // Validate target ID
    if query.target_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Target ID cannot be empty".to_string(),
        ));
    }

    // Validate parameters JSON if present
    if let Some(ref params) = query.parameters {
        if !params.is_empty() {
            if serde_json::from_str::<serde_json::Value>(params).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Parameters must be valid JSON".to_string(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_climate_query_for_author(
    query: &ClimateQuery,
    author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_climate_query(query)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }
    Ok(validate_actor_binding(
        "ClimateQuery",
        "requester_did",
        &query.requester_did,
        author,
    ))
}

/// Validate a ClimateResult entry
fn validate_climate_result(result: &ClimateResult) -> ExternResult<ValidateCallbackResult> {
    // Validate query ID
    if result.query_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Query ID cannot be empty".to_string(),
        ));
    }

    // Validate responder DID
    let did_result = validate_did(&result.responder_did)?;
    if let ValidateCallbackResult::Invalid(_) = did_result {
        return Ok(did_result);
    }

    // Validate data JSON if present
    if let Some(ref data) = result.data {
        if !data.is_empty() {
            if serde_json::from_str::<serde_json::Value>(data).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Data must be valid JSON".to_string(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_climate_result_for_author(
    result: &ClimateResult,
    author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_climate_result(result)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }
    Ok(validate_actor_binding(
        "ClimateResult",
        "responder_did",
        &result.responder_did,
        author,
    ))
}

/// Validate a MarketplaceListing entry
fn validate_marketplace_listing(listing: &MarketplaceListing) -> ExternResult<ValidateCallbackResult> {
    // Validate listing ID
    if listing.listing_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Listing ID cannot be empty".to_string(),
        ));
    }

    // Validate credit ID
    if listing.credit_id.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit ID cannot be empty".to_string(),
        ));
    }

    // Validate credit action hash
    if listing.credit_action_hash.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Credit action hash cannot be empty".to_string(),
        ));
    }

    // Validate seller DID
    let did_result = validate_did(&listing.seller_did)?;
    if let ValidateCallbackResult::Invalid(_) = did_result {
        return Ok(did_result);
    }

    // Validate price
    if listing.price_per_tonne == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Price must be greater than zero".to_string(),
        ));
    }

    // Validate currency
    if listing.currency.is_empty() || listing.currency.len() != 3 {
        return Ok(ValidateCallbackResult::Invalid(
            "Currency must be a 3-letter code".to_string(),
        ));
    }

    // Validate min purchase
    if listing.min_purchase <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum purchase must be positive".to_string(),
        ));
    }

    // Validate available tonnes
    if listing.available_tonnes <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Available tonnes must be positive".to_string(),
        ));
    }

    // Min purchase cannot exceed available
    if listing.min_purchase > listing.available_tonnes {
        return Ok(ValidateCallbackResult::Invalid(
            "Minimum purchase cannot exceed available tonnes".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_marketplace_listing_for_author(
    listing: &MarketplaceListing,
    author: &AgentPubKey,
) -> ExternResult<ValidateCallbackResult> {
    let fields = validate_marketplace_listing(listing)?;
    if let ValidateCallbackResult::Invalid(_) = fields {
        return Ok(fields);
    }
    Ok(validate_actor_binding(
        "MarketplaceListing",
        "seller_did",
        &listing.seller_did,
        author,
    ))
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::ClimateQuery(query) => {
                    validate_climate_query_for_author(&query, &action.author)
                }
                EntryTypes::ClimateResult(result) => {
                    validate_climate_result_for_author(&result, &action.author)
                }
                EntryTypes::MarketplaceListing(listing) => {
                    validate_marketplace_listing_for_author(&listing, &action.author)
                }
                EntryTypes::Notification(n) => {
                    mycelix_bridge_entry_types::validate_notification(&n)
                        .map(|()| ValidateCallbackResult::Valid)
                        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e)))
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Invalid(
                    "Anchors cannot be updated".to_string(),
                )),
                EntryTypes::ClimateQuery(query) => {
                    validate_climate_query_for_author(&query, &action.author)
                }
                EntryTypes::ClimateResult(_) => Ok(ValidateCallbackResult::Invalid(
                    "Climate results cannot be updated once submitted".to_string(),
                )),
                EntryTypes::MarketplaceListing(listing) => {
                    validate_marketplace_listing_for_author(&listing, &action.author)
                }
                EntryTypes::Notification(_) => Ok(ValidateCallbackResult::Invalid(
                    "Notifications cannot be updated".to_string(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToQueries
            | LinkTypes::QueryToResult
            | LinkTypes::RequesterToQueries
            | LinkTypes::AnchorToListings
            | LinkTypes::SellerToListings
            | LinkTypes::CreditToListings
            | LinkTypes::AgentToNotification
            | LinkTypes::AllNotifications
            | LinkTypes::NotificationSubscription => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { link_type, action, .. } => match link_type {
            LinkTypes::QueryToResult => Ok(ValidateCallbackResult::Invalid(
                "Query-to-result links cannot be deleted".to_string(),
            )),
            _ => {
                let original_action = must_get_action(action.link_add_address.clone())?;
                Ok(check_link_author_match(
                    original_action.action().author(),
                    &action.author,
                ))
            }
        },
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn valid_query() -> ClimateQuery {
        ClimateQuery {
            query_id: "query:1".into(),
            purpose: QueryPurpose::ProjectDueDiligence,
            requester_did: "did:mycelix:requester-1".into(),
            target_id: "project:1".into(),
            parameters: Some(r#"{"depth":"standard"}"#.into()),
            status: QueryStatus::Pending,
            created_at: 1_788_825_600,
        }
    }

    fn valid_result() -> ClimateResult {
        ClimateResult {
            query_id: "query:1".into(),
            result: VerificationResult::Verified,
            data: Some(r#"{"evidence":"artifact:1"}"#.into()),
            responder_did: "did:mycelix:verifier-1".into(),
            responded_at: 1_788_825_700,
            signature: None,
        }
    }

    fn valid_listing() -> MarketplaceListing {
        MarketplaceListing {
            listing_id: "listing:1".into(),
            credit_id: "credit:1".into(),
            credit_action_hash: "uhCAkexample".into(),
            project_id: "project:1".into(),
            seller_did: "did:mycelix:seller-1".into(),
            price_per_tonne: 1_000,
            currency: "USD".into(),
            min_purchase: 1.0,
            available_tonnes: 10.0,
            expires_at: 1_800_000_000,
            is_active: true,
            created_at: 1_788_825_600,
        }
    }

    fn assert_valid(result: ExternResult<ValidateCallbackResult>) {
        assert!(matches!(
            result.expect("validator should execute"),
            ValidateCallbackResult::Valid
        ));
    }

    fn assert_invalid_contains(result: ExternResult<ValidateCallbackResult>, needle: &str) {
        match result.expect("validator should execute") {
            ValidateCallbackResult::Invalid(reason) => assert!(
                reason.contains(needle),
                "expected rejection containing {needle:?}, got {reason:?}"
            ),
            other => panic!("expected invalid result, got {other:?}"),
        }
    }

    #[test]
    fn production_query_validator_accepts_valid_query() {
        assert_valid(validate_climate_query(&valid_query()));
    }

    #[test]
    fn production_query_validator_rejects_invalid_requester_identity() {
        let mut query = valid_query();
        query.requester_did = "requester-1".into();
        assert_invalid_contains(validate_climate_query(&query), "did:");
    }

    #[test]
    fn production_query_validator_rejects_invalid_parameters_json() {
        let mut query = valid_query();
        query.parameters = Some("{".into());
        assert_invalid_contains(validate_climate_query(&query), "valid JSON");
    }

    #[test]
    fn production_result_validator_accepts_valid_result() {
        assert_valid(validate_climate_result(&valid_result()));
    }

    #[test]
    fn production_result_validator_rejects_invalid_responder_identity() {
        let mut result = valid_result();
        result.responder_did = "verifier-1".into();
        assert_invalid_contains(validate_climate_result(&result), "did:");
    }

    #[test]
    fn production_result_validator_rejects_invalid_data_json() {
        let mut result = valid_result();
        result.data = Some("[".into());
        assert_invalid_contains(validate_climate_result(&result), "valid JSON");
    }

    #[test]
    fn production_listing_validator_accepts_valid_listing() {
        assert_valid(validate_marketplace_listing(&valid_listing()));
    }

    #[test]
    fn production_listing_validator_rejects_zero_price() {
        let mut listing = valid_listing();
        listing.price_per_tonne = 0;
        assert_invalid_contains(validate_marketplace_listing(&listing), "Price");
    }

    #[test]
    fn production_listing_validator_rejects_oversized_minimum_purchase() {
        let mut listing = valid_listing();
        listing.min_purchase = listing.available_tonnes + 1.0;
        assert_invalid_contains(validate_marketplace_listing(&listing), "cannot exceed");
    }

    #[test]
    fn author_binding_accepts_query_requester_that_matches_committer() {
        let author = fake_agent(1);
        let mut query = valid_query();
        query.requester_did = did_for_author(&author);
        assert_valid(validate_climate_query_for_author(&query, &author));
    }

    #[test]
    fn author_binding_rejects_forged_query_requester() {
        let author = fake_agent(1);
        let victim = fake_agent(2);
        let mut query = valid_query();
        query.requester_did = did_for_author(&victim);
        assert_invalid_contains(
            validate_climate_query_for_author(&query, &author),
            "committing agent",
        );
    }

    #[test]
    fn author_binding_rejects_forged_result_responder() {
        let author = fake_agent(3);
        let victim = fake_agent(4);
        let mut result = valid_result();
        result.responder_did = did_for_author(&victim);
        assert_invalid_contains(
            validate_climate_result_for_author(&result, &author),
            "committing agent",
        );
    }

    #[test]
    fn author_binding_rejects_forged_marketplace_seller() {
        let author = fake_agent(5);
        let victim = fake_agent(6);
        let mut listing = valid_listing();
        listing.seller_did = did_for_author(&victim);
        assert_invalid_contains(
            validate_marketplace_listing_for_author(&listing, &author),
            "committing agent",
        );
    }
}
