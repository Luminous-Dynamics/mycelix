#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-014 disabled/non-canonical owner-authored SAP V2 claim journal.
//!
//! This integrity zome persists only immutable account-opening and collateral-claim
//! facts. It does not persist or mutate a scalar SAP balance. Collateral claims are
//! authorized by exact valid FIN-SAFE-010 issuance-receipt actions; downstream
//! projection de-duplicates their economic effect by canonical `mint_id`.

use collateral_issuance_v2_integrity::CollateralSapIssuanceReceiptV2Entry;
use finance_sap_account_v2::{SapAccountOpenedV2, SapAccountV2Config, SapCollateralClaimV2};
use hdi::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

const COLLATERAL_ISSUANCE_INTEGRITY_ZOME: &str = "collateral_issuance_v2_integrity";
/// FIN-SAFE-010 local entry order:
/// 0 authorization, 1 mint, 2 issuance receipt.
const COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX: u8 = 2;

#[dna_properties]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FinanceSapAccountV2DnaProperties {
    pub sap_account_v2: SapAccountV2Config,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapAccountOpenedV2Entry {
    pub opened: SapAccountOpenedV2,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct SapCollateralClaimV2Entry {
    pub claim: SapCollateralClaimV2,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    SapAccountOpenedV2(SapAccountOpenedV2Entry),
    SapCollateralClaimV2(SapCollateralClaimV2Entry),
}

/// V1 deliberately has no global claim/opening index. The account is an
/// owner-authored immutable journal, not a globally mutable balance singleton.
#[hdk_link_types]
pub enum LinkTypes {
    ReservedAccountIndex,
}

pub fn load_sap_account_v2_config() -> ExternResult<SapAccountV2Config> {
    Ok(FinanceSapAccountV2DnaProperties::try_from_dna_properties()?.sap_account_v2)
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    let config = load_sap_account_v2_config()?;
    if let Err(error) = config.validate() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid FIN-SAFE-014 SAP account V2 configuration: {error:?}"
        )));
    }

    // Preflight the exact foreign FIN-SAFE-010 receipt identity even while the
    // shipped protocol is disabled. A malformed DNA should fail at genesis rather
    // than discovering the missing dependency during a future enablement.
    if let Err(message) = foreign_public_entry_def(
        COLLATERAL_ISSUANCE_INTEGRITY_ZOME,
        COLLATERAL_ISSUANCE_RECEIPT_ENTRY_INDEX,
    ) {
        return Ok(ValidateCallbackResult::Invalid(message));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::SapAccountOpenedV2(entry) => validate_open_create(action, entry),
                EntryTypes::SapCollateralClaimV2(entry) => validate_claim_create(action, entry),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-014 SAP account V2 journal entries are immutable".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::SapAccountOpenedV2(entry) => validate_open_create(action, entry),
                EntryTypes::SapCollateralClaimV2(entry) => validate_claim_create(action, entry),
            },
            OpRecord::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-014 SAP account V2 journal entries are immutable".into(),
            )),
            OpRecord::CreateLink { .. } | OpRecord::DeleteLink { .. } => {
                Ok(ValidateCallbackResult::Invalid(
                    "FIN-SAFE-014 v1 does not authorize account links".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-014 SAP account V2 journal entries are immutable".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-014 SAP account V2 journal entries cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-014 v1 does not authorize account links".into(),
            ))
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_open_create(
    action: Create,
    entry: SapAccountOpenedV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    let config = load_sap_account_v2_config()?;
    if let Err(error) = config.require_enabled() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "SAP account V2 opening is disabled by DNA policy: {error:?}"
        )));
    }

    let author_did = did_for_author(&action.author);
    match entry.opened.validate_author(&author_did) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(error) => Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid owner-authored SAP account V2 opening: {error:?}"
        ))),
    }
}

fn validate_claim_create(
    action: Create,
    entry: SapCollateralClaimV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    let config = load_sap_account_v2_config()?;
    if let Err(error) = config.require_enabled() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "SAP account V2 collateral claims are disabled by DNA policy: {error:?}"
        )));
    }

    if let Err(error) = entry.claim.validate_shape() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid SAP collateral claim shape: {error:?}"
        )));
    }

    let author_did = did_for_author(&action.author);
    if author_did != entry.claim.member_did {
        return Ok(ValidateCallbackResult::Invalid(
            "SAP collateral claim action author does not match member DID".into(),
        ));
    }

    let receipt = match load_exact_issuance_receipt(&entry.claim.issuance_receipt_action_reference)
    {
        Ok(receipt) => receipt,
        Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
    };

    if receipt.recipient_did != entry.claim.member_did {
        return Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-010 issuance receipt recipient does not match SAP claim owner".into(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn load_exact_issuance_receipt(
    reference: &str,
) -> Result<finance_collateral_issuance_persistence::CollateralSapIssuanceReceiptRecordV2, String> {
    let action_hash = ActionHash::try_from(reference.to_string())
        .map_err(|error| format!("Invalid issuance receipt action reference: {error:?}"))?;
    let record = must_get_valid_record(action_hash).map_err(|error| {
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
    match record.action().app_entry_def() {
        Some(actual) if actual == &expected => Ok(()),
        Some(actual) => Err(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        )),
        None => Err(format!("{label} is not an application entry")),
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
