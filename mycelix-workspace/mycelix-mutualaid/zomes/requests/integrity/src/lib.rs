// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Requests Integrity Zome - Aid requests and offers for mutual aid coordination
//!
//! This zome defines the data structures and validation rules for aid requests
//! and offers within the Mycelix mutual aid network.

use hdi::prelude::*;

/// Anchor entry type for string-based link bases
#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct Anchor(pub String);

impl Anchor {
    pub fn new(value: impl Into<String>) -> Self {
        Anchor(value.into())
    }
}

/// Type of aid being requested
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestType {
    Financial,
    Housing,
    Food,
    Medical,
    Childcare,
    Transportation,
    Legal,
    Other(String),
}

/// Urgency level for aid requests
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Urgency {
    Critical,
    High,
    Medium,
    Low,
}

/// Status of an aid request
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestStatus {
    Open,
    PartiallyFulfilled,
    Fulfilled,
    Cancelled,
}

/// Status of an aid offer
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OfferStatus {
    Pending,
    Accepted,
    Completed,
    Withdrawn,
}

/// An aid request from a community member
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AidRequest {
    /// Unique identifier for this request
    pub id: String,
    /// DID of the person requesting aid
    pub requester_did: String,
    /// Type of aid being requested
    pub request_type: RequestType,
    /// Detailed description of the need
    pub description: String,
    /// Urgency level
    pub urgency: Urgency,
    /// Optional location (for local aid)
    pub location: Option<String>,
    /// Amount needed (if applicable, in smallest currency unit)
    pub amount_needed: Option<u64>,
    /// Amount already fulfilled
    pub fulfilled_amount: u64,
    /// Current status of the request
    pub status: RequestStatus,
    /// Timestamp when request was created
    pub created_at: Timestamp,
    /// Timestamp when request was last updated
    pub updated_at: Timestamp,
}

/// An offer to fulfill an aid request
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AidOffer {
    /// Unique identifier for this offer
    pub id: String,
    /// Reference to the aid request being fulfilled
    pub request_id: String,
    /// DID of the person offering aid
    pub offerer_did: String,
    /// Amount being offered (if applicable)
    pub amount: Option<u64>,
    /// Message from the offerer
    pub message: String,
    /// Current status of the offer
    pub status: OfferStatus,
    /// Timestamp when offer was created
    pub created_at: Timestamp,
    /// Timestamp when offer was last updated
    pub updated_at: Timestamp,
}

/// All entry types for this zome
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
    #[entry_type(visibility = "public")]
    AidRequest(AidRequest),
    #[entry_type(visibility = "public")]
    AidOffer(AidOffer),
}

/// Link types for connecting entries
#[hdk_link_types]
pub enum LinkTypes {
    /// Anchor to all requests
    AnchorToRequest,
    /// Anchor to requests by type
    TypeToRequest,
    /// Anchor to requests by status
    StatusToRequest,
    /// Anchor to requests by urgency
    UrgencyToRequest,
    /// Request to its offers
    RequestToOffer,
    /// Requester DID to their requests
    RequesterToRequest,
    /// Offerer DID to their offers
    OffererToOffer,
}

/// Validation errors for requests zome
#[derive(Debug)]
pub enum RequestsError {
    InvalidDid(String),
    InvalidId(String),
    NegativeAmount,
    FulfilledExceedsNeeded,
    EmptyDescription,
    EmptyRequestId,
}

impl std::fmt::Display for RequestsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidDid(s) => write!(f, "Invalid DID format: {}", s),
            Self::InvalidId(s) => write!(f, "Invalid ID format: {}", s),
            Self::NegativeAmount => write!(f, "Amount cannot be negative"),
            Self::FulfilledExceedsNeeded => write!(f, "Fulfilled amount exceeds needed amount"),
            Self::EmptyDescription => write!(f, "Description cannot be empty"),
            Self::EmptyRequestId => write!(f, "Request ID cannot be empty"),
        }
    }
}

/// Validate that a DID has a valid format
fn validate_did(did: &str) -> ExternResult<()> {
    if did.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            RequestsError::InvalidDid("DID cannot be empty".to_string()).to_string()
        )));
    }
    // Basic DID format check: did:method:identifier
    if !did.starts_with("did:") || did.split(':').count() < 3 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            RequestsError::InvalidDid(format!("Invalid DID format: {}", did)).to_string()
        )));
    }
    Ok(())
}

/// Validate that an ID is non-empty
fn validate_id(id: &str, field_name: &str) -> ExternResult<()> {
    if id.is_empty() {
        return Err(wasm_error!(WasmErrorInner::Guest(
            RequestsError::InvalidId(format!("{} cannot be empty", field_name)).to_string()
        )));
    }
    Ok(())
}

/// requester_did is NOT bound to the committer -- see the zome-wide
/// disclosed-gap note on validate_update_entry_type above. Case (d).
fn validate_aid_request(request: &AidRequest) -> ExternResult<ValidateCallbackResult> {
    // Validate requester DID
    validate_did(&request.requester_did)?;

    // Validate ID
    validate_id(&request.id, "Request ID")?;

    // Validate description is not empty
    if request.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            RequestsError::EmptyDescription.to_string(),
        ));
    }

    // Validate fulfilled amount doesn't exceed needed amount
    if let Some(needed) = request.amount_needed {
        if request.fulfilled_amount > needed {
            return Ok(ValidateCallbackResult::Invalid(
                RequestsError::FulfilledExceedsNeeded.to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// offerer_did is NOT bound to the committer -- see the zome-wide
/// disclosed-gap note on validate_update_entry_type above. Case (d).
fn validate_aid_offer(offer: &AidOffer) -> ExternResult<ValidateCallbackResult> {
    // Validate offerer DID
    validate_did(&offer.offerer_did)?;

    // Validate IDs
    validate_id(&offer.id, "Offer ID")?;
    validate_id(&offer.request_id, "Request ID")?;

    Ok(ValidateCallbackResult::Valid)
}

/// Genesis self-check callback
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, .. } => match app_entry {
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
                EntryTypes::AidRequest(request) => validate_aid_request(&request),
                EntryTypes::AidOffer(offer) => validate_aid_offer(&offer),
            },
            OpEntry::UpdateEntry {
                app_entry,
                original_action_hash,
                ..
            } => validate_update_entry_type(original_action_hash, app_entry),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToRequest
            | LinkTypes::TypeToRequest
            | LinkTypes::StatusToRequest
            | LinkTypes::UrgencyToRequest
            | LinkTypes::RequestToOffer
            | LinkTypes::RequesterToRequest
            | LinkTypes::OffererToOffer => Ok(ValidateCallbackResult::Valid),
        },
        // Deliberately left fully permissive for ALL link types (reviewed
        // 2026-07-09 during the P0 author-binding pass, not a gap):
        // update_request_status deletes/recreates StatusToRequest links
        // on every status transition, and the coordinator's own doc
        // comments for cancel_request/withdraw_offer ("only by the
        // requester"/"only by the offerer") describe an authorization
        // intent that was never actually implemented in code (see the
        // zome-wide disclosed-gap note below) -- so there's no reliable
        // notion of "the real requester" to restrict link deletion to
        // even if we wanted to.
        FlatOp::RegisterDeleteLink { link_type, .. } => match link_type {
            LinkTypes::AnchorToRequest
            | LinkTypes::TypeToRequest
            | LinkTypes::StatusToRequest
            | LinkTypes::UrgencyToRequest
            | LinkTypes::RequestToOffer
            | LinkTypes::RequesterToRequest
            | LinkTypes::OffererToOffer => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(_) | FlatOp::RegisterAgentActivity(_) => {
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterUpdate(op_update) => match op_update {
            // This DHT op was previously left fully permissive (`Ok(Valid)`
            // unconditionally) -- the 28th confirmed instance of this
            // exact bug pattern this pass. Found + fixed 2026-07-09
            // during the P0 author-binding pass. Route through the same
            // per-type validators as the StoreEntry perspective.
            OpUpdate::Entry { app_entry, action } => {
                validate_update_entry_type(action.original_action_address, app_entry)
            }
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

/// **Zome-wide disclosed, NOT-fixed gap**: `AidRequest.requester_did`/
/// `AidOffer.offerer_did` are free-form String DIDs asserted by the
/// caller with NO local convention anywhere in this coordinator for
/// deriving or verifying a DID string from the committing agent's real
/// `action.author` (same gap as mycelix-mutualaid/bridge, fixed just
/// before this zome). Notably, the coordinator's OWN doc comments for
/// `cancel_request`/`withdraw_offer` claim "only by the
/// requester"/"only by the offerer" restrictions that were NEVER
/// actually implemented in code -- both functions just call
/// `update_request_status`/`update_offer_status` directly, which have
/// zero caller-identity checks of any kind. Reviewed 2026-07-09 during
/// the P0 author-binding pass; case (d), needs real
/// call-provenance/capability-grant infrastructure. Content is still
/// restricted below (status/fulfilled_amount/updated_at for requests;
/// status/updated_at for offers) to at least prevent unrelated-field
/// tampering, matching the pattern established for mycelix-mutualaid/bridge.
fn validate_update_entry_type(
    original_action_hash: ActionHash,
    app_entry: EntryTypes,
) -> ExternResult<ValidateCallbackResult> {
    match app_entry {
        EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
        EntryTypes::AidRequest(request) => {
            validate_update_aid_request(original_action_hash, request)
        }
        EntryTypes::AidOffer(offer) => validate_update_aid_offer(original_action_hash, offer),
    }
}

fn validate_update_aid_request(
    original_action_hash: ActionHash,
    request: AidRequest,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: AidRequest = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original request not found".into()
        )))?;

    if request.id != original.id
        || request.requester_did != original.requester_did
        || request.request_type != original.request_type
        || request.description != original.description
        || request.urgency != original.urgency
        || request.location != original.location
        || request.amount_needed != original.amount_needed
        || request.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/fulfilled_amount/updated_at can change on a request update".into(),
        ));
    }

    validate_aid_request(&request)
}

fn validate_update_aid_offer(
    original_action_hash: ActionHash,
    offer: AidOffer,
) -> ExternResult<ValidateCallbackResult> {
    let original_record = must_get_valid_record(original_action_hash)?;
    let original: AidOffer = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Original offer not found".into()
        )))?;

    if offer.id != original.id
        || offer.request_id != original.request_id
        || offer.offerer_did != original.offerer_did
        || offer.amount != original.amount
        || offer.message != original.message
        || offer.created_at != original.created_at
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Only status/updated_at can change on an offer update".into(),
        ));
    }

    validate_aid_offer(&offer)
}
