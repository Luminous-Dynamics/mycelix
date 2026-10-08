#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-014 coordinator for the disabled/non-canonical owner-authored SAP V2
//! collateral-claim journal.
//!
//! This API has no raw credit surface. A collateral claim accepts exactly one
//! FIN-SAFE-010 issuance-receipt `ActionHash`; member identity and economic facts
//! are reconstructed from the caller and the exact valid receipt.

use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry,
    UnitEntryTypes as CollateralIssuanceUnitEntryTypes,
};
use finance_collateral_issuance_persistence::CollateralSapIssuanceReceiptRecordV2;
use finance_sap_account_v2::{SapAccountOpenedV2, SapCollateralClaimV2};
use hdk::prelude::*;
use mycelix_bridge_entry_types::did_for_author;
use sap_account_v2_integrity::{
    load_sap_account_v2_config, EntryTypes, SapAccountOpenedV2Entry,
    SapCollateralClaimV2Entry, UnitEntryTypes,
};

/// Create a zero-economic-effect owner-authored account opening marker.
///
/// Multiple physical opening markers are harmless and do not create SAP.
#[hdk_extern]
pub fn open_sap_account_v2(_: ()) -> ExternResult<Record> {
    require_protocol_enabled()?;
    let member_did = current_author_did()?;
    let opened = SapAccountOpenedV2::new(member_did).map_err(guest_error)?;
    create_and_reload(EntryTypes::SapAccountOpenedV2(SapAccountOpenedV2Entry {
        opened,
    }))
}

/// Claim one exact FIN-SAFE-010 issuance receipt into the caller's V2 account
/// journal. The caller cannot provide member DID, amount, mint ID, or deposit ID.
#[hdk_extern]
pub fn claim_collateral_issuance_v2(
    issuance_receipt_action_hash: ActionHash,
) -> ExternResult<Record> {
    require_protocol_enabled()?;
    let member_did = current_author_did()?;
    let receipt = load_exact_issuance_receipt(issuance_receipt_action_hash.clone())?;
    if receipt.recipient_did != member_did {
        return Err(wasm_error!(WasmErrorInner::Guest(format!(
            "FIN-SAFE-010 issuance receipt belongs to {}, not caller {}",
            receipt.recipient_did, member_did
        ))));
    }

    let claim = SapCollateralClaimV2::new(
        member_did,
        issuance_receipt_action_hash.to_string(),
    )
    .map_err(guest_error)?;

    create_and_reload(EntryTypes::SapCollateralClaimV2(
        SapCollateralClaimV2Entry { claim },
    ))
}

#[hdk_extern]
pub fn get_sap_account_opened_v2(
    action_hash: ActionHash,
) -> ExternResult<SapAccountOpenedV2> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::SapAccountOpenedV2)?,
        "FIN-SAFE-014 SAP account opening",
    )?;
    decode_entry::<SapAccountOpenedV2Entry>(&record, "FIN-SAFE-014 SAP account opening")
        .map(|entry| entry.opened)
}

#[hdk_extern]
pub fn get_sap_collateral_claim_v2(
    action_hash: ActionHash,
) -> ExternResult<SapCollateralClaimV2> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::SapCollateralClaimV2)?,
        "FIN-SAFE-014 SAP collateral claim",
    )?;
    decode_entry::<SapCollateralClaimV2Entry>(&record, "FIN-SAFE-014 SAP collateral claim")
        .map(|entry| entry.claim)
}

/// Convenience/audit projection of the exact receipt referenced by one valid
/// claim action. This does not compute a canonical account balance.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedSapCollateralClaimV2 {
    pub claim_action_reference: String,
    pub claim_author_did: String,
    pub claim: SapCollateralClaimV2,
    pub receipt: CollateralSapIssuanceReceiptRecordV2,
}

#[hdk_extern]
pub fn get_verified_sap_collateral_claim_v2(
    claim_action_hash: ActionHash,
) -> ExternResult<VerifiedSapCollateralClaimV2> {
    let record = must_get_valid_record(claim_action_hash.clone())?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::SapCollateralClaimV2)?,
        "FIN-SAFE-014 SAP collateral claim",
    )?;
    let entry = decode_entry::<SapCollateralClaimV2Entry>(
        &record,
        "FIN-SAFE-014 SAP collateral claim",
    )?;
    let author_did = did_for_author(record.action().author());
    if author_did != entry.claim.member_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "SAP collateral claim author does not match member DID".into()
        )));
    }
    let receipt_hash = ActionHash::try_from(entry.claim.issuance_receipt_action_reference.clone())
        .map_err(|error| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid issuance receipt action reference: {error:?}"
            )))
        })?;
    let receipt = load_exact_issuance_receipt(receipt_hash)?;
    if receipt.recipient_did != entry.claim.member_did {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Issuance receipt recipient does not match claim owner".into()
        )));
    }

    Ok(VerifiedSapCollateralClaimV2 {
        claim_action_reference: claim_action_hash.to_string(),
        claim_author_did: author_did,
        claim: entry.claim,
        receipt,
    })
}

fn require_protocol_enabled() -> ExternResult<()> {
    load_sap_account_v2_config()?
        .require_enabled()
        .map_err(guest_error)
}

fn current_author_did() -> ExternResult<String> {
    Ok(did_for_author(&agent_info()?.agent_initial_pubkey))
}

fn load_exact_issuance_receipt(
    action_hash: ActionHash,
) -> ExternResult<CollateralSapIssuanceReceiptRecordV2> {
    let record = must_get_valid_record(action_hash)?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(
            CollateralIssuanceUnitEntryTypes::CollateralSapIssuanceReceiptV2,
        )?,
        "FIN-SAFE-010 issuance receipt",
    )?;
    decode_entry::<CollateralSapIssuanceReceiptV2Entry>(
        &record,
        "FIN-SAFE-010 issuance receipt",
    )
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
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
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
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "{label} entry is missing"
            )))
        })
}

fn guest_error(error: impl core::fmt::Debug) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{error:?}")))
}
