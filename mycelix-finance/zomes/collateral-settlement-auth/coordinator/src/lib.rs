#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-011 coordinator for DNA-rooted collateral attestations and settlement.
//!
//! Authoritative settlement accepts exact action hashes only. It reloads the V2
//! deposit request plus price/custody attestations with Holochain validity
//! semantics, obtains their real action authors/timestamps, and derives the
//! request/evidence-bound settlement from the DNA trust root.

use collateral_deposit_v2_integrity::{
    CollateralDepositRequestV2Entry, MAX_CREATE_TIMESTAMP_SKEW_MICROS,
    UnitEntryTypes as DepositUnitEntryTypes,
};
use collateral_settlement_auth_integrity::{
    CustodyAttestationV1Entry, EntryTypes, PriceAttestationV1Entry,
    UnitEntryTypes as AuthUnitEntryTypes, load_collateral_auth_config,
};
use finance_collateral_auth::{
    CollateralSettlementTrustRootV1, CustodyAttestationV1, PriceAttestationV1,
};
use finance_collateral_request_binding::{
    AuthenticatedBoundCollateralSettlementIntentV1, BoundCollateralDepositRequest,
    derive_authenticated_bound_settlement_intent_from_valid_actions,
};
use finance_collateral_settlement::{CustodyEvidenceEnvelope, PriceEvidenceEnvelope};
use hdk::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedPriceAttestationV1 {
    pub attestation_action_reference: String,
    pub action_author_did: String,
    pub action_timestamp_micros: i64,
    pub attestation: PriceAttestationV1,
    pub evidence: PriceEvidenceEnvelope,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedCustodyAttestationV1 {
    pub attestation_action_reference: String,
    pub action_author_did: String,
    pub action_timestamp_micros: i64,
    pub attestation: CustodyAttestationV1,
    pub evidence: CustodyEvidenceEnvelope,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeriveAuthenticatedCollateralSettlementV1Input {
    pub request_action_hash: ActionHash,
    pub price_attestation_action_hash: ActionHash,
    pub custody_attestation_action_hash: ActionHash,
}

#[hdk_extern]
pub fn create_price_attestation_v1(attestation: PriceAttestationV1) -> ExternResult<Record> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral price attestation is disabled or unconfigured: {error:?}"
        )))
    })?;
    let info = agent_info()?;
    let author_did = did_for_author(&info.agent_initial_pubkey);
    attestation
        .to_evidence_from_valid_action(root, &author_did, sys_time()?.as_micros())
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Price attestation is not authorized by the Finance DNA trust root: {error:?}"
            )))
        })?;

    let action_hash = create_entry(&EntryTypes::PriceAttestationV1(PriceAttestationV1Entry {
        attestation,
    }))?;
    must_get_valid_record(action_hash)
}

#[hdk_extern]
pub fn create_custody_attestation_v1(attestation: CustodyAttestationV1) -> ExternResult<Record> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral custody attestation is disabled or unconfigured: {error:?}"
        )))
    })?;
    let info = agent_info()?;
    let author_did = did_for_author(&info.agent_initial_pubkey);
    attestation
        .to_evidence_from_valid_action(root, &author_did, sys_time()?.as_micros())
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Custody attestation is not authorized by the Finance DNA trust root: {error:?}"
            )))
        })?;

    let action_hash = create_entry(&EntryTypes::CustodyAttestationV1(
        CustodyAttestationV1Entry { attestation },
    ))?;
    must_get_valid_record(action_hash)
}

/// Project one exact valid price-attestation Create action into FIN-SAFE-006 evidence.
///
/// This is a convenience/audit read. The returned object is not a portable
/// authority token; persistent issuance must reload the exact referenced action.
#[hdk_extern]
pub fn get_verified_price_attestation_v1(
    action_hash: ActionHash,
) -> ExternResult<VerifiedPriceAttestationV1> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral settlement trust root is disabled or invalid: {error:?}"
        )))
    })?;
    load_price_attestation(action_hash, root)
}

/// Project one exact valid custody-attestation Create action into FIN-SAFE-006 evidence.
#[hdk_extern]
pub fn get_verified_custody_attestation_v1(
    action_hash: ActionHash,
) -> ExternResult<VerifiedCustodyAttestationV1> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral settlement trust root is disabled or invalid: {error:?}"
        )))
    })?;
    load_custody_attestation(action_hash, root)
}

/// Derive one issuance-eligible settlement authorization from exactly three
/// already-valid Holochain Create actions:
///
/// - V2 collateral request;
/// - price attestation;
/// - custody attestation.
///
/// No economic terms, provider IDs, evidence payloads, rates, or policy fields
/// are accepted from the caller. Decision time is the current conductor time.
#[hdk_extern]
pub fn derive_authenticated_collateral_settlement_v1(
    input: DeriveAuthenticatedCollateralSettlementV1Input,
) -> ExternResult<AuthenticatedBoundCollateralSettlementIntentV1> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral settlement is disabled or lacks a valid DNA trust root: {error:?}"
        )))
    })?;

    let bound_request = load_bound_request(input.request_action_hash)?;
    let price = load_price_attestation(input.price_attestation_action_hash, root)?;
    let custody = load_custody_attestation(input.custody_attestation_action_hash, root)?;
    let evaluated_at_micros = sys_time()?.as_micros();

    derive_authenticated_bound_settlement_intent_from_valid_actions(
        bound_request,
        root,
        price.attestation_action_reference,
        price.action_author_did,
        price.action_timestamp_micros,
        price.attestation,
        custody.attestation_action_reference,
        custody.action_author_did,
        custody.action_timestamp_micros,
        custody.attestation,
        evaluated_at_micros,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Authenticated collateral settlement derivation failed: {error:?}"
        )))
    })
}

fn load_bound_request(action_hash: ActionHash) -> ExternResult<BoundCollateralDepositRequest> {
    let record = must_get_valid_record(action_hash.clone())?;
    require_create_action(&record, "collateral request")?;
    require_exact_app_entry_type(
        &record,
        AppEntryDef::try_from(DepositUnitEntryTypes::CollateralDepositRequestV2)?,
        "V2 collateral request",
    )?;
    let entry = record
        .entry()
        .to_app_option::<CollateralDepositRequestV2Entry>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode V2 collateral request: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Request action does not contain a V2 collateral request".into()
            ))
        })?;
    let request = entry.to_model().map_err(|message| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid V2 collateral request entry: {message}"
        )))
    })?;
    request
        .validate_action_timestamp(
            record.action().timestamp().as_micros(),
            MAX_CREATE_TIMESTAMP_SKEW_MICROS,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "V2 collateral request timestamp binding failed: {error:?}"
            )))
        })?;
    let author_did = did_for_author(record.action().author());
    if request.depositor_did != author_did {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "V2 collateral request depositor DID does not match action author: expected {}, got {}",
            author_did, request.depositor_did
        ))));
    }

    let bound = BoundCollateralDepositRequest {
        request_action_reference: action_hash.to_string(),
        request,
    };
    bound.validate().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Bound V2 collateral request failed validation: {error:?}"
        )))
    })?;
    Ok(bound)
}

fn load_price_attestation(
    action_hash: ActionHash,
    root: &CollateralSettlementTrustRootV1,
) -> ExternResult<VerifiedPriceAttestationV1> {
    let record = must_get_valid_record(action_hash.clone())?;
    require_create_action(&record, "price attestation")?;
    require_exact_app_entry_type(
        &record,
        AppEntryDef::try_from(AuthUnitEntryTypes::PriceAttestationV1)?,
        "FIN-SAFE-011 price attestation",
    )?;
    let entry = record
        .entry()
        .to_app_option::<PriceAttestationV1Entry>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode FIN-SAFE-011 price attestation: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Action does not contain a FIN-SAFE-011 price attestation".into()
            ))
        })?;
    let action_author_did = did_for_author(record.action().author());
    let action_timestamp_micros = record.action().timestamp().as_micros();
    let evidence = entry
        .attestation
        .to_evidence_from_valid_action(root, &action_author_did, action_timestamp_micros)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Exact price attestation action failed FIN-SAFE-011 authentication: {error:?}"
            )))
        })?;

    Ok(VerifiedPriceAttestationV1 {
        attestation_action_reference: action_hash.to_string(),
        action_author_did,
        action_timestamp_micros,
        attestation: entry.attestation,
        evidence,
    })
}

fn load_custody_attestation(
    action_hash: ActionHash,
    root: &CollateralSettlementTrustRootV1,
) -> ExternResult<VerifiedCustodyAttestationV1> {
    let record = must_get_valid_record(action_hash.clone())?;
    require_create_action(&record, "custody attestation")?;
    require_exact_app_entry_type(
        &record,
        AppEntryDef::try_from(AuthUnitEntryTypes::CustodyAttestationV1)?,
        "FIN-SAFE-011 custody attestation",
    )?;
    let entry = record
        .entry()
        .to_app_option::<CustodyAttestationV1Entry>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode FIN-SAFE-011 custody attestation: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Action does not contain a FIN-SAFE-011 custody attestation".into()
            ))
        })?;
    let action_author_did = did_for_author(record.action().author());
    let action_timestamp_micros = record.action().timestamp().as_micros();
    let evidence = entry
        .attestation
        .to_evidence_from_valid_action(root, &action_author_did, action_timestamp_micros)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Exact custody attestation action failed FIN-SAFE-011 authentication: {error:?}"
            )))
        })?;

    Ok(VerifiedCustodyAttestationV1 {
        attestation_action_reference: action_hash.to_string(),
        action_author_did,
        action_timestamp_micros,
        attestation: entry.attestation,
        evidence,
    })
}

fn require_create_action(record: &Record, label: &str) -> ExternResult<()> {
    if record.action().action_type() != ActionType::Create {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "FIN-SAFE-011 {label} evidence must reference an exact Create action"
        ))));
    }
    Ok(())
}

fn require_exact_app_entry_type(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> ExternResult<()> {
    match record.action().app_entry_def() {
        Some(actual) if actual == &expected => Ok(()),
        Some(actual) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} action has the wrong app entry definition: expected {expected:?}, got {actual:?}"
        )))),
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} action is not an application entry"
        )))),
    }
}
