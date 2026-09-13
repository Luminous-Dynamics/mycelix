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

use finance_collateral_issuance_persistence::CollateralSapIssuanceReceiptRecordV2;
use finance_holochain_contracts::{
    CollateralSapIssuanceReceiptV2Entry, SapAccountOpenedV2Entry, SapCollateralClaimV2Entry,
};
use finance_sap_account_v2::{SapAccountOpenedV2, SapCollateralClaimV2};
use hdk::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

const COLLATERAL_ISSUANCE_INTEGRITY_ZOME: &str = "collateral_issuance_v2_integrity";
const COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX: u8 = 2;
use sap_account_v2_integrity::{EntryTypes, UnitEntryTypes, load_sap_account_v2_config};

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

    let claim = SapCollateralClaimV2::new(member_did, issuance_receipt_action_hash.to_string())
        .map_err(guest_error)?;

    create_and_reload(EntryTypes::SapCollateralClaimV2(
        SapCollateralClaimV2Entry { claim },
    ))
}

#[hdk_extern]
pub fn get_sap_account_opened_v2(action_hash: ActionHash) -> ExternResult<SapAccountOpenedV2> {
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
pub fn get_sap_collateral_claim_v2(action_hash: ActionHash) -> ExternResult<SapCollateralClaimV2> {
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
    let entry =
        decode_entry::<SapCollateralClaimV2Entry>(&record, "FIN-SAFE-014 SAP collateral claim")?;
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
        foreign_public_entry_def(
            COLLATERAL_ISSUANCE_INTEGRITY_ZOME,
            COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX,
        )?,
        "FIN-SAFE-010 issuance receipt",
    )?;
    decode_entry::<CollateralSapIssuanceReceiptV2Entry>(&record, "FIN-SAFE-010 issuance receipt")
        .map(|entry| entry.record)
}

fn foreign_public_entry_def(zome_name: &str, entry_index: u8) -> ExternResult<AppEntryDef> {
    let info = dna_info()?;
    let zome_position = info
        .zome_names
        .iter()
        .position(|name| name.to_string() == zome_name)
        .ok_or_else(|| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Required integrity zome {zome_name:?} is absent from this DNA"
            )))
        })?;
    let zome_index = u8::try_from(zome_position).map_err(|_| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Integrity zome index for {zome_name:?} exceeds u8"
        )))
    })?;
    Ok(AppEntryDef::new(
        EntryDefIndex::from(entry_index),
        ZomeIndex::from(zome_index),
        EntryVisibility::Public,
    ))
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
    match record.action().entry_type() {
        Some(EntryType::App(actual)) if actual == &expected => Ok(()),
        Some(EntryType::App(actual)) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        )))),
        Some(actual) => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} is not an application entry: got {actual:?}"
        )))),
        None => Err(wasm_error!(WasmErrorInner::Guest(format!(
            "{label} has no entry type"
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

fn guest_error(error: impl core::fmt::Debug) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(format!("{error:?}")))
}
