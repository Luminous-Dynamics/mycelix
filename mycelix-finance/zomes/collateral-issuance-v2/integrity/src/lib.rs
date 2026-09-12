#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-010 zero-effect collateral SAP issuance provenance integrity.
//!
//! This zome persists only authorization/mint/receipt facts. None of these entry
//! types can mutate a SAP balance. Authority is reconstructed by loading the exact
//! V2 request + FIN-SAFE-011 price/custody actions and the DNA trust root.

use collateral_deposit_v2_integrity::{
    CollateralDepositRequestV2Entry, MAX_CREATE_TIMESTAMP_SKEW_MICROS,
};
use collateral_settlement_auth_integrity::{
    CustodyAttestationV1Entry, PriceAttestationV1Entry, load_collateral_auth_config,
};
use finance_collateral_auth::{
    CollateralSettlementTrustRootV1, CustodyAttestationV1, PriceAttestationV1,
};
use finance_collateral_issuance_persistence::{
    CollateralSapIssuanceReceiptRecordV2, CollateralSapMintAuthorizationRecordV2,
    CollateralSapMintRecordV2Compact,
};
use finance_collateral_request_binding::{
    BoundCollateralDepositRequest, derive_authenticated_bound_settlement_intent_from_valid_actions,
};
use finance_sap_conservation::AuthenticatedCollateralSapMintAuthorizationV2;
use hdi::prelude::*;
use mycelix_bridge_entry_types::did_for_author;

/// A committed authorization should immediately follow the decision that produced
/// its `authorized_at_micros`. This protects the persisted decision time from
/// arbitrary backdating/forward-dating while allowing normal conductor latency.
pub const MAX_AUTHORIZATION_PERSIST_SKEW_MICROS: i64 = 5_000_000;

const DEPOSIT_INTEGRITY_ZOME: &str = "collateral_deposit_v2_integrity";
const SETTLEMENT_AUTH_INTEGRITY_ZOME: &str = "collateral_settlement_auth_integrity";
const DEPOSIT_REQUEST_ENTRY_INDEX: u8 = 0;
const PRICE_ATTESTATION_ENTRY_INDEX: u8 = 0;
const CUSTODY_ATTESTATION_ENTRY_INDEX: u8 = 1;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapMintAuthorizationV2Entry {
    pub record: CollateralSapMintAuthorizationRecordV2,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapMintRecordV2Entry {
    pub record: CollateralSapMintRecordV2Compact,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CollateralSapIssuanceReceiptV2Entry {
    pub record: CollateralSapIssuanceReceiptRecordV2,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CollateralSapMintAuthorizationV2(CollateralSapMintAuthorizationV2Entry),
    CollateralSapMintRecordV2(CollateralSapMintRecordV2Entry),
    CollateralSapIssuanceReceiptV2(CollateralSapIssuanceReceiptV2Entry),
}

#[hdk_link_types]
pub enum LinkTypes {
    ReservedIssuanceIndex,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    // FIN-SAFE-011 owns the DNA trust-root shape. A disabled root is a valid DNA
    // configuration, but all issuance creates will fail closed until enabled.
    let config = load_collateral_auth_config()?;
    if let Err(error) = config.validate() {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid collateral issuance trust-root configuration: {error:?}"
        )));
    }

    // Foreign entry identity is resolved from the actual ordered integrity-zome
    // list in this DNA. Missing/renamed required zomes therefore fail closed.
    for (zome, entry_index, label) in [
        (
            DEPOSIT_INTEGRITY_ZOME,
            DEPOSIT_REQUEST_ENTRY_INDEX,
            "V2 collateral request",
        ),
        (
            SETTLEMENT_AUTH_INTEGRITY_ZOME,
            PRICE_ATTESTATION_ENTRY_INDEX,
            "FIN-SAFE-011 price attestation",
        ),
        (
            SETTLEMENT_AUTH_INTEGRITY_ZOME,
            CUSTODY_ATTESTATION_ENTRY_INDEX,
            "FIN-SAFE-011 custody attestation",
        ),
    ] {
        if let Err(message) = foreign_public_entry_def(zome, entry_index) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Unable to resolve {label} entry identity: {message}"
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
                EntryTypes::CollateralSapMintAuthorizationV2(entry) => {
                    validate_authorization_create(action, entry)
                }
                EntryTypes::CollateralSapMintRecordV2(entry) => validate_mint_create(action, entry),
                EntryTypes::CollateralSapIssuanceReceiptV2(entry) => {
                    validate_receipt_create(action, entry)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-010 issuance provenance entries are immutable".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CollateralSapMintAuthorizationV2(entry) => {
                    validate_authorization_create(action, entry)
                }
                EntryTypes::CollateralSapMintRecordV2(entry) => validate_mint_create(action, entry),
                EntryTypes::CollateralSapIssuanceReceiptV2(entry) => {
                    validate_receipt_create(action, entry)
                }
            },
            OpRecord::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-010 issuance provenance entries are immutable".into(),
            )),
            OpRecord::CreateLink { .. } | OpRecord::DeleteLink { .. } => {
                Ok(ValidateCallbackResult::Invalid(
                    "FIN-SAFE-010 v1 does not authorize issuance links".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-010 issuance provenance entries are immutable".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "FIN-SAFE-010 issuance provenance entries cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-010 v1 does not authorize issuance links".into(),
            ))
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_authorization_create(
    action: Create,
    entry: CollateralSapMintAuthorizationV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    match validate_authorization_model(action.timestamp.as_micros(), &entry.record) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(message) => Ok(ValidateCallbackResult::Invalid(message)),
    }
}

fn validate_mint_create(
    _action: Create,
    entry: CollateralSapMintRecordV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    match validate_mint_model(&entry.record) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(message) => Ok(ValidateCallbackResult::Invalid(message)),
    }
}

fn validate_receipt_create(
    _action: Create,
    entry: CollateralSapIssuanceReceiptV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    match validate_receipt_model(&entry.record) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(message) => Ok(ValidateCallbackResult::Invalid(message)),
    }
}

fn validate_authorization_model(
    action_timestamp_micros: i64,
    compact: &CollateralSapMintAuthorizationRecordV2,
) -> Result<(), String> {
    compact
        .validate_shape()
        .map_err(|error| format!("Invalid compact mint authorization shape: {error:?}"))?;
    bind_authorization_time(action_timestamp_micros, compact.authorized_at_micros)?;

    let config = load_collateral_auth_config().map_err(|error| format!("DNA config: {error:?}"))?;
    let root = config
        .require_enabled_root()
        .map_err(|error| format!("Collateral issuance disabled or unconfigured: {error:?}"))?;
    if compact.trust_root_id != root.root_id || compact.trust_root_version != root.root_version {
        return Err("Mint authorization trust-root reference does not match Finance DNA".into());
    }

    let authenticated = reconstruct_authenticated_authorization(compact, root)?;
    compact
        .validate_against_authenticated(&authenticated, root)
        .map_err(|error| {
            format!("Mint authorization facts do not match reconstructed authority: {error:?}")
        })
}

fn validate_mint_model(compact: &CollateralSapMintRecordV2Compact) -> Result<(), String> {
    let (authorization_action_timestamp, authorization) =
        load_authorization_record(&compact.authorization_action_reference)?;
    validate_authorization_model(authorization_action_timestamp, &authorization)?;
    compact
        .validate_against_authorization(&authorization)
        .map_err(|error| {
            format!("V2 collateral mint does not match exact authorization: {error:?}")
        })
}

fn validate_receipt_model(compact: &CollateralSapIssuanceReceiptRecordV2) -> Result<(), String> {
    let (authorization_action_timestamp, authorization) =
        load_authorization_record(&compact.authorization_action_reference)?;
    validate_authorization_model(authorization_action_timestamp, &authorization)?;

    let mint = load_mint_record(&compact.mint_action_reference)?;
    validate_mint_model(&mint)?;
    compact
        .validate_against_records(&authorization, &mint)
        .map_err(|error| {
            format!("Issuance receipt does not match exact authorization/mint actions: {error:?}")
        })
}

fn reconstruct_authenticated_authorization(
    compact: &CollateralSapMintAuthorizationRecordV2,
    root: &CollateralSettlementTrustRootV1,
) -> Result<AuthenticatedCollateralSapMintAuthorizationV2, String> {
    let bound_request = load_bound_request(&compact.request_action_reference)?;
    let (price, price_author, price_timestamp) =
        load_price_attestation(&compact.price_attestation_action_reference, root)?;
    let (custody, custody_author, custody_timestamp) =
        load_custody_attestation(&compact.custody_attestation_action_reference, root)?;

    let settlement = derive_authenticated_bound_settlement_intent_from_valid_actions(
        bound_request,
        root,
        compact.price_attestation_action_reference.clone(),
        price_author,
        price_timestamp,
        price,
        compact.custody_attestation_action_reference.clone(),
        custody_author,
        custody_timestamp,
        custody,
        compact.authorized_at_micros,
    )
    .map_err(|error| format!("Failed to reconstruct authenticated settlement: {error:?}"))?;

    AuthenticatedCollateralSapMintAuthorizationV2::from_authenticated_settlement(settlement, root)
        .map_err(|error| {
            format!("Failed to reconstruct authenticated mint authorization: {error:?}")
        })
}

fn load_bound_request(reference: &str) -> Result<BoundCollateralDepositRequest, String> {
    let action_hash = parse_action_reference(reference)?;
    let record = must_get_valid_record(action_hash.clone())
        .map_err(|error| format!("Unable to load exact V2 request action: {error:?}"))?;
    require_exact_create_entry(
        &record,
        foreign_public_entry_def(DEPOSIT_INTEGRITY_ZOME, DEPOSIT_REQUEST_ENTRY_INDEX)?,
        "V2 collateral request",
    )?;
    let entry = decode_entry::<CollateralDepositRequestV2Entry>(&record, "V2 collateral request")?;
    let request = entry.to_model()?;
    request
        .validate_action_timestamp(
            record.action().timestamp().as_micros(),
            MAX_CREATE_TIMESTAMP_SKEW_MICROS,
        )
        .map_err(|error| format!("V2 request timestamp binding failed: {error:?}"))?;
    let author_did = did_for_author(record.action().author());
    if request.depositor_did != author_did {
        return Err("V2 request depositor DID does not match action author".into());
    }
    let bound = BoundCollateralDepositRequest {
        request_action_reference: action_hash.to_string(),
        request,
    };
    bound
        .validate()
        .map_err(|error| format!("V2 request binding failed: {error:?}"))?;
    Ok(bound)
}

fn load_price_attestation(
    reference: &str,
    root: &CollateralSettlementTrustRootV1,
) -> Result<(PriceAttestationV1, String, i64), String> {
    let action_hash = parse_action_reference(reference)?;
    let record = must_get_valid_record(action_hash)
        .map_err(|error| format!("Unable to load exact price attestation: {error:?}"))?;
    require_exact_create_entry(
        &record,
        foreign_public_entry_def(
            SETTLEMENT_AUTH_INTEGRITY_ZOME,
            PRICE_ATTESTATION_ENTRY_INDEX,
        )?,
        "FIN-SAFE-011 price attestation",
    )?;
    let entry = decode_entry::<PriceAttestationV1Entry>(&record, "FIN-SAFE-011 price attestation")?;
    let author_did = did_for_author(record.action().author());
    let timestamp = record.action().timestamp().as_micros();
    entry
        .attestation
        .to_evidence_from_valid_action(root, &author_did, timestamp)
        .map_err(|error| format!("Price attestation authentication failed: {error:?}"))?;
    Ok((entry.attestation, author_did, timestamp))
}

fn load_custody_attestation(
    reference: &str,
    root: &CollateralSettlementTrustRootV1,
) -> Result<(CustodyAttestationV1, String, i64), String> {
    let action_hash = parse_action_reference(reference)?;
    let record = must_get_valid_record(action_hash)
        .map_err(|error| format!("Unable to load exact custody attestation: {error:?}"))?;
    require_exact_create_entry(
        &record,
        foreign_public_entry_def(
            SETTLEMENT_AUTH_INTEGRITY_ZOME,
            CUSTODY_ATTESTATION_ENTRY_INDEX,
        )?,
        "FIN-SAFE-011 custody attestation",
    )?;
    let entry =
        decode_entry::<CustodyAttestationV1Entry>(&record, "FIN-SAFE-011 custody attestation")?;
    let author_did = did_for_author(record.action().author());
    let timestamp = record.action().timestamp().as_micros();
    entry
        .attestation
        .to_evidence_from_valid_action(root, &author_did, timestamp)
        .map_err(|error| format!("Custody attestation authentication failed: {error:?}"))?;
    Ok((entry.attestation, author_did, timestamp))
}

fn load_authorization_record(
    reference: &str,
) -> Result<(i64, CollateralSapMintAuthorizationRecordV2), String> {
    let action_hash = parse_action_reference(reference)?;
    let record = must_get_valid_record(action_hash)
        .map_err(|error| format!("Unable to load exact mint authorization action: {error:?}"))?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::CollateralSapMintAuthorizationV2)
            .map_err(|error| format!("Authorization entry definition: {error:?}"))?,
        "FIN-SAFE-010 mint authorization",
    )?;
    let entry = decode_entry::<CollateralSapMintAuthorizationV2Entry>(
        &record,
        "FIN-SAFE-010 mint authorization",
    )?;
    Ok((record.action().timestamp().as_micros(), entry.record))
}

fn load_mint_record(reference: &str) -> Result<CollateralSapMintRecordV2Compact, String> {
    let action_hash = parse_action_reference(reference)?;
    let record = must_get_valid_record(action_hash)
        .map_err(|error| format!("Unable to load exact V2 collateral mint action: {error:?}"))?;
    require_exact_create_entry(
        &record,
        AppEntryDef::try_from(UnitEntryTypes::CollateralSapMintRecordV2)
            .map_err(|error| format!("Mint entry definition: {error:?}"))?,
        "FIN-SAFE-010 V2 collateral mint",
    )?;
    let entry =
        decode_entry::<CollateralSapMintRecordV2Entry>(&record, "FIN-SAFE-010 V2 collateral mint")?;
    Ok(entry.record)
}

fn parse_action_reference(reference: &str) -> Result<ActionHash, String> {
    ActionHash::try_from(reference.to_string())
        .map_err(|error| format!("Invalid action reference {reference:?}: {error:?}"))
}

/// Resolve a public app-entry definition from the DNA's actual ordered integrity
/// zome list. `dna_info().zome_names` is derived from `DnaDef.integrity_zomes`, so
/// this exactly matches the zome index encoded in an application's Create action.
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
        return Err(format!("{label} reference is not a Create action"));
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

fn bind_authorization_time(action_timestamp: i64, authorized_at: i64) -> Result<(), String> {
    if action_timestamp < authorized_at {
        return Err("Mint authorization action predates its decision time".into());
    }
    let delta = action_timestamp as i128 - authorized_at as i128;
    if delta > MAX_AUTHORIZATION_PERSIST_SKEW_MICROS as i128 {
        return Err("Mint authorization was persisted too long after its decision time".into());
    }
    Ok(())
}
