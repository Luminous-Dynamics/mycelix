#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-010 coordinator for staged collateral SAP issuance provenance.
//!
//! This zome has no balance mutation capability. It creates only three immutable
//! zero-effect stages: mint authorization, V2 collateral mint, and issuance
//! receipt. The integrity zome independently reconstructs every authoritative
//! reference before any stage is accepted.

use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry, CollateralSapMintAuthorizationV2Entry,
    CollateralSapMintRecordV2Entry, EntryTypes, UnitEntryTypes,
};
use collateral_settlement_auth_integrity::load_collateral_auth_config;
use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2, CollateralSapMintAuthorizationRecordV2,
    CollateralSapMintRecordV2Compact,
};
use finance_collateral_request_binding::AuthenticatedBoundCollateralSettlementIntentV1;
use finance_sap_conservation::AuthenticatedCollateralSapMintAuthorizationV2;
use hdk::prelude::*;

/// Persist a compact mint authorization from an authenticated FIN-SAFE-011
/// settlement object. The object supplied here is only a construction aid; DHT
/// integrity reloads its exact action references and re-derives authority.
#[hdk_extern]
pub fn create_collateral_sap_mint_authorization_v2(
    authenticated_settlement: AuthenticatedBoundCollateralSettlementIntentV1,
) -> ExternResult<Record> {
    let config = load_collateral_auth_config()?;
    let root = config.require_enabled_root().map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral issuance is disabled or lacks a valid DNA trust root: {error:?}"
        )))
    })?;

    let authorization =
        AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(
            authenticated_settlement,
            root,
        )
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Authenticated collateral mint authorization failed: {error:?}"
            )))
        })?;
    let compact = CollateralSapMintAuthorizationRecordV2::from_authenticated(&authorization, root)
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Compact collateral mint authorization construction failed: {error:?}"
            )))
        })?;

    create_and_reload(EntryTypes::CollateralSapMintAuthorizationV2(
        CollateralSapMintAuthorizationV2Entry { record: compact },
    ))
}

/// Persist the exact V2 mint facts authorized by one valid authorization action.
/// This has zero SAP balance effect.
#[hdk_extern]
pub fn create_collateral_sap_mint_record_v2(
    authorization_action_hash: ActionHash,
) -> ExternResult<Record> {
    let authorization = load_authorization(authorization_action_hash.clone())?;
    let compact = CollateralSapMintRecordV2Compact::from_authorization_record(
        authorization_action_hash.to_string(),
        &authorization,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "V2 collateral mint construction failed: {error:?}"
        )))
    })?;

    create_and_reload(EntryTypes::CollateralSapMintRecordV2(
        CollateralSapMintRecordV2Entry { record: compact },
    ))
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CreateCollateralSapIssuanceReceiptV2Input {
    pub authorization_action_hash: ActionHash,
    pub mint_action_hash: ActionHash,
}

/// Persist a receipt joining one exact authorization action and one exact V2
/// mint action. Receipt creation still has zero SAP balance effect.
#[hdk_extern]
pub fn create_collateral_sap_issuance_receipt_v2(
    input: CreateCollateralSapIssuanceReceiptV2Input,
) -> ExternResult<Record> {
    let authorization = load_authorization(input.authorization_action_hash.clone())?;
    let mint = load_mint(input.mint_action_hash.clone())?;
    let compact = CollateralSapIssuanceReceiptRecordV2::from_records(
        input.authorization_action_hash.to_string(),
        &authorization,
        input.mint_action_hash.to_string(),
        &mint,
    )
    .map_err(|error| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Collateral issuance receipt construction failed: {error:?}"
        )))
    })?;

    create_and_reload(EntryTypes::CollateralSapIssuanceReceiptV2(
        CollateralSapIssuanceReceiptV2Entry { record: compact },
    ))
}

/// Exact-action audit read for any FIN-SAFE-010 provenance stage. The entry type
/// is checked by the specific stage loader rather than inferred from bytes.
#[hdk_extern]
pub fn get_collateral_sap_mint_authorization_v2(
    action_hash: ActionHash,
) -> ExternResult<CollateralSapMintAuthorizationRecordV2> {
    load_authorization(action_hash)
}

#[hdk_extern]
pub fn get_collateral_sap_mint_record_v2(
    action_hash: ActionHash,
) -> ExternResult<CollateralSapMintRecordV2Compact> {
    load_mint(action_hash)
}

#[hdk_extern]
pub fn get_collateral_sap_issuance_receipt_v2(
    action_hash: ActionHash,
) -> ExternResult<CollateralSapIssuanceReceiptRecordV2> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::CollateralSapIssuanceReceiptV2)?,
        "FIN-SAFE-010 issuance receipt",
    )?;
    decode_entry::<CollateralSapIssuanceReceiptV2Entry>(&record, "FIN-SAFE-010 issuance receipt")
        .map(|entry| entry.record)
}

fn load_authorization(
    action_hash: ActionHash,
) -> ExternResult<CollateralSapMintAuthorizationRecordV2> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::CollateralSapMintAuthorizationV2)?,
        "FIN-SAFE-010 mint authorization",
    )?;
    decode_entry::<CollateralSapMintAuthorizationV2Entry>(
        &record,
        "FIN-SAFE-010 mint authorization",
    )
    .map(|entry| entry.record)
}

fn load_mint(action_hash: ActionHash) -> ExternResult<CollateralSapMintRecordV2Compact> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::CollateralSapMintRecordV2)?,
        "FIN-SAFE-010 V2 collateral mint",
    )?;
    decode_entry::<CollateralSapMintRecordV2Entry>(&record, "FIN-SAFE-010 V2 collateral mint")
        .map(|entry| entry.record)
}

fn create_and_reload(entry: EntryTypes) -> ExternResult<Record> {
    let action_hash = create_entry(&entry)?;
    must_get_valid_record(action_hash)
}

fn require_exact_create_entry(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> ExternResult<()> {
    if record.action().action_type() != ActionType::Create {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} reference is not an exact Create action"
        ))));
    }
    match record.action().app_entry_def() {
        Some(actual) if actual == &expected => Ok(()),
        Some(actual) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has the wrong app entry definition: expected {expected:?}, got {actual:?}"
        )))),
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} is not an application entry"
        )))),
    }
}

fn decode_entry<T>(record: &Record, label: &str) -> ExternResult<T>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError>,
{
    record
        .entry()
        .to_app_option::<T>()
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Failed to decode {label}: {error:?}"
            )))
        })?
        .ok_or_else(|| wasm_error!(WasmErrorInner::Guest(format!("{label} entry is missing"))))
}
