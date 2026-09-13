#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-016 disabled/non-canonical SAP V2 transfer-spend integrity zome.
//!
//! V1 spends only exact owner-authored FIN-SAFE-014 collateral claims. Each input
//! claim is reloaded with Holochain validity semantics, its FIN-SAFE-010 receipt is
//! reloaded, the economic note is reconstructed, and the pure FIN-SAFE-015/016
//! transfer theorem is rerun. No caller-supplied note amount/owner payload is
//! authoritative.

use finance_holochain_contracts::{
    CollateralSapIssuanceReceiptV2Entry, FinanceSapTransferV2DnaProperties,
    SapCollateralClaimV2Entry, SapTransferSpendV2Entry,
};
use finance_sap_account_v2::ValidatedCollateralClaimV2;
use finance_sap_transfer_v2::{SapTransferSpendRecordV2, SapTransferV2Config};
use hdi::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

const SAP_ACCOUNT_V2_INTEGRITY_ZOME: &str = "sap_account_v2_integrity";
const SAP_COLLATERAL_CLAIM_ENTRY_INDEX: u8 = 1;
const COLLATERAL_ISSUANCE_INTEGRITY_ZOME: &str = "collateral_issuance_v2_integrity";
const COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX: u8 = 2;

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    SapTransferSpendV2(SapTransferSpendV2Entry),
}

/// No V1 global spend/input index is authoritative. Conflict projection consumes
/// exact spend actions supplied/discovered by callers and never treats a link as
/// uniqueness or latest-state consensus.
#[hdk_link_types]
pub enum LinkTypes {
    ReservedSpendIndex,
}

pub fn load_sap_transfer_v2_config() -> ExternResult<SapTransferV2Config> {
    Ok(FinanceSapTransferV2DnaProperties::try_from_dna_properties()?.sap_transfer_v2)
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    let config = load_sap_transfer_v2_config()?;
    if let Err(error) = config.validate() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid FIN-SAFE-016 SAP transfer V2 configuration: {error:?}"
        )));
    }

    for (zome, index, label) in [
        (
            SAP_ACCOUNT_V2_INTEGRITY_ZOME,
            SAP_COLLATERAL_CLAIM_ENTRY_INDEX,
            "FIN-SAFE-014 collateral claim",
        ),
        (
            COLLATERAL_ISSUANCE_INTEGRITY_ZOME,
            COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX,
            "FIN-SAFE-010 issuance receipt",
        ),
    ] {
        if let Err(message) = foreign_public_entry_def(zome, index) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "{label} dependency is invalid: {message}"
            )));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::SapTransferSpendV2(entry) => validate_spend_create(action, entry),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-016 transfer-spend entries are immutable".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::SapTransferSpendV2(entry) => validate_spend_create(action, entry),
            },
            OpRecord::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-016 transfer-spend entries are immutable".into(),
            )),
            OpRecord::CreateLink { .. } | OpRecord::DeleteLink { .. } => {
                Ok(ValidateCallbackResult::Invalid(
                    "FIN-SAFE-016 v1 does not authorize transfer links".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-016 transfer-spend entries are immutable".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-016 transfer-spend entries cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-016 v1 does not authorize transfer links".into(),
            ))
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_spend_create(
    action: Create,
    entry: SapTransferSpendV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    let config = load_sap_transfer_v2_config()?;
    if let Err(error) = config.require_enabled() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "SAP transfer V2 spends are disabled by DNA policy: {error:?}"
        )));
    }

    if let Err(error) = entry.spend.validate_shape() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid FIN-SAFE-016 transfer-spend shape: {error:?}"
        )));
    }

    let author_did = did_for_author(&action.author);
    if author_did != entry.spend.transfer.sender_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer-spend action author does not match transfer sender DID".into(),
        ));
    }

    let mut claims = Vec::with_capacity(entry.spend.input_claims.len());
    for input in &entry.spend.input_claims {
        match load_exact_validated_collateral_claim(&input.claim_action_reference) {
            Ok(claim) => claims.push(claim),
            Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
        }
    }

    match entry.spend.validate_against_claims(&claims) {
        Ok(_) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "FIN-SAFE-016 transfer-spend theorem failed: {error:?}"
        ))),
    }
}

pub fn load_exact_validated_collateral_claim(
    reference: &str,
) -> Result<ValidatedCollateralClaimV2, String> {
    let claim_hash = parse_action_reference(reference, "collateral claim")?;
    let claim_record = must_get_valid_record(claim_hash.clone()).map_err(|error| {
        format!("Unable to load exact FIN-SAFE-014 collateral claim: {error:?}")
    })?;
    require_exact_create_entry(
        &claim_record,
        foreign_public_entry_def(
            SAP_ACCOUNT_V2_INTEGRITY_ZOME,
            SAP_COLLATERAL_CLAIM_ENTRY_INDEX,
        )?,
        "FIN-SAFE-014 collateral claim",
    )?;
    let claim_entry =
        decode_entry::<SapCollateralClaimV2Entry>(&claim_record, "FIN-SAFE-014 collateral claim")?;
    let claim_author_did = did_for_author(claim_record.action().author());

    let receipt =
        load_exact_issuance_receipt(&claim_entry.claim.issuance_receipt_action_reference)?;

    ValidatedCollateralClaimV2::from_valid_receipt(
        claim_hash.to_string(),
        claim_author_did,
        &claim_entry.claim,
        &receipt,
    )
    .map_err(|error| format!("FIN-SAFE-014 collateral claim reconstruction failed: {error:?}"))
}

fn load_exact_issuance_receipt(
    reference: &str,
) -> Result<finance_collateral_issuance_persistence::CollateralSapIssuanceReceiptRecordV2, String> {
    let receipt_hash = parse_action_reference(reference, "issuance receipt")?;
    let record = must_get_valid_record(receipt_hash).map_err(|error| {
        format!("Unable to load exact FIN-SAFE-010 issuance receipt: {error:?}")
    })?;
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

fn parse_action_reference(reference: &str, label: &str) -> Result<ActionHash, String> {
    ActionHash::try_from(reference.to_string())
        .map_err(|error| format!("Invalid {label} action reference: {error:?}"))
}

fn foreign_public_entry_def(zome_name: &str, entry_index: u8) -> Result<AppEntryDef, String> {
    let info = dna_info().map_err(|error| format!("Unable to read DNA zome order: {error:?}"))?;
    let zome_position = info
        .zome_names
        .iter()
        .position(|name| name.to_string() == zome_name)
        .ok_or_else(|| format!("Required integrity zome {zome_name:?} is absent from this DNA"))?;
    let zome_index = u8::try_from(zome_position)
        .map_err(|_| format!("Integrity zome index for {zome_name:?} exceeds u8"))?;

    Ok(AppEntryDef::new(
        EntryDefIndex::from(entry_index),
        ZomeIndex::from(zome_index),
        EntryVisibility::Public,
    ))
}

fn require_exact_create_entry(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> Result<(), String> {
    if record.action().action_type() != ActionType::Create {
        return Err(format!("{label} reference is not an exact Create action"));
    }
    match record.action().entry_type() {
        Some(EntryType::App(actual)) if actual == &expected => Ok(()),
        Some(EntryType::App(actual)) => Err(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        )),
        Some(actual) => Err(format!(
            "{label} is not an application entry: got {actual:?}"
        )),
        None => Err(format!("{label} has no entry type")),
    }
}

fn decode_entry<T>(record: &Record, label: &str) -> Result<T, String>
where
    T: TryFrom<SerializedBytes, Error = SerializedBytesError>,
{
    record
        .entry()
        .to_app_option::<T>()
        .map_err(|error| format!("Failed to decode {label}: {error:?}"))?
        .ok_or_else(|| format!("{label} entry is missing"))
}
