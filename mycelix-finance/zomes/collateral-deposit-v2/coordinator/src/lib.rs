#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-008 request-only collateral deposit coordinator.
//!
//! This zome cannot price collateral, confirm custody, mint SAP, settle, redeem,
//! or mutate a request. It only creates immutable V2 deposit intent records and
//! reads them by exact action hash / the caller's own source chain.

use collateral_deposit_v2_integrity::{
    CollateralDepositRequestV2Entry, EntryTypes, UnitEntryTypes,
};
use finance_collateral_deposit::{
    CollateralDepositRequestV2, COLLATERAL_DEPOSIT_NONCE_BYTES,
};
use hdk::prelude::*;
use mycelix_finance_shared::verify_caller_is_did;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct CreateCollateralDepositRequestV2Input {
    pub depositor_did: String,
    pub collateral_asset_id: String,
    pub collateral_amount: u64,
    /// Persist this nonce client-side and reuse it when retrying the same logical
    /// request. Use `generate_collateral_deposit_request_nonce` to obtain it from
    /// the conductor's CSPRNG if the client does not already have secure entropy.
    pub request_nonce: Vec<u8>,
}

/// Generate a 32-byte conductor-backed nonce for one logical V2 request.
///
/// This does not persist the nonce. The caller must persist it before submission
/// if crash-safe retry identity is required.
#[hdk_extern]
pub fn generate_collateral_deposit_request_nonce(_: ()) -> ExternResult<Vec<u8>> {
    Ok(random_bytes(COLLATERAL_DEPOSIT_NONCE_BYTES as u32)?.to_vec())
}

#[hdk_extern]
pub fn create_collateral_deposit_request_v2(
    input: CreateCollateralDepositRequestV2Input,
) -> ExternResult<Record> {
    verify_caller_is_did(&input.depositor_did)?;

    let request_nonce: [u8; COLLATERAL_DEPOSIT_NONCE_BYTES] = input
        .request_nonce
        .as_slice()
        .try_into()
        .map_err(|_| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "request_nonce must contain exactly {} bytes",
                COLLATERAL_DEPOSIT_NONCE_BYTES
            )))
        })?;

    let now = sys_time()?;
    let request = CollateralDepositRequestV2::new(
        input.depositor_did,
        input.collateral_asset_id,
        input.collateral_amount,
        request_nonce,
        now.as_micros(),
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Invalid V2 collateral deposit request: {error:?}"
        )))
    })?;
    let entry = CollateralDepositRequestV2Entry::from(request);

    // Same logical request + same persisted nonce is idempotent on this source
    // chain. This deliberately does NOT claim global exactly-once behavior under
    // concurrent authorship across multiple agents/cells.
    if let Some(existing) = find_my_request_by_id(&entry.deposit_id)? {
        let existing_entry = decode_request(&existing)?;
        if existing_entry == entry {
            return Ok(existing);
        }
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Deposit ID already exists on this source chain with different request terms".into()
        )));
    }

    let action_hash = create_entry(&EntryTypes::CollateralDepositRequestV2(entry))?;
    get(action_hash, GetOptions::default())?.ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(
            "V2 collateral deposit request was not readable after creation".into()
        ))
    })
}

/// Read one V2 request by its exact Create action hash.
#[hdk_extern]
pub fn get_collateral_deposit_request_v2(
    action_hash: ActionHash,
) -> ExternResult<Option<Record>> {
    match get(action_hash, GetOptions::default())? {
        Some(record) => {
            decode_request(&record)?;
            Ok(Some(record))
        }
        None => Ok(None),
    }
}

/// Return V2 requests authored on the current agent's source chain.
#[hdk_extern]
pub fn get_my_collateral_deposit_requests_v2(_: ()) -> ExternResult<Vec<Record>> {
    query(request_query_filter()?)
}

fn find_my_request_by_id(deposit_id: &str) -> ExternResult<Option<Record>> {
    for record in query(request_query_filter()?)? {
        let entry = decode_request(&record)?;
        if entry.deposit_id == deposit_id {
            return Ok(Some(record));
        }
    }
    Ok(None)
}

fn request_query_filter() -> ExternResult<ChainQueryFilter> {
    Ok(ChainQueryFilter::new()
        .entry_type(EntryType::App(AppEntryDef::try_from(
            UnitEntryTypes::CollateralDepositRequestV2,
        )?))
        .include_entries(true))
}

fn decode_request(record: &Record) -> ExternResult<CollateralDepositRequestV2Entry> {
    record
        .entry()
        .to_app_option::<CollateralDepositRequestV2Entry>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode V2 collateral deposit request: {error:?}"
            )))
        })?
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(
                "Action does not contain a V2 collateral deposit request".into()
            ))
        })
}
