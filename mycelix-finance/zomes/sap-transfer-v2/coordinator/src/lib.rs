#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-016 thin coordinator for the disabled SAP V2 transfer-spend lane.
//!
//! Callers supply recipient, amount, and exact owner-authored collateral-claim
//! action hashes. Authoritative note payloads are never accepted from callers.

use collateral_issuance_v2_integrity::{
    CollateralSapIssuanceReceiptV2Entry,
    UnitEntryTypes as CollateralIssuanceUnitEntryTypes,
};
use finance_sap_account_v2::ValidatedCollateralClaimV2;
use finance_sap_transfer_v2::SapTransferSpendRecordV2;
use finance_sap_value_notes::{
    classify_note_consumption, SapNoteConsumptionStateV2, SapTransferSpendObservationV2,
    SapTransferV2, SapValueNoteV2, MAX_TRANSFER_INPUTS,
};
use hdk::prelude::*;
use mycelix_bridge_entry_types::did_for_author;
use sap_account_v2_integrity::{
    SapCollateralClaimV2Entry, UnitEntryTypes as SapAccountUnitEntryTypes,
};
use sap_transfer_v2_integrity::{
    load_sap_transfer_v2_config, EntryTypes, SapTransferSpendV2Entry,
    UnitEntryTypes as SapTransferUnitEntryTypes,
};

pub const MAX_EXPLICIT_SPEND_OBSERVATIONS: usize = 1024;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CreateSapTransferV2Input {
    pub recipient_did: String,
    pub transfer_amount: u64,
    pub input_claim_action_hashes: Vec<ActionHash>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerifiedSapTransferSpendV2 {
    pub action_reference: String,
    pub action_author_did: String,
    pub spend: SapTransferSpendRecordV2,
    pub observation: SapTransferSpendObservationV2,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ClassifySapNoteConsumptionV2Input {
    pub note_id: String,
    pub spend_action_hashes: Vec<ActionHash>,
}

/// Classification scope is encoded in the response so `Unspent` cannot be
/// mistaken for network-complete proof that no other spend action exists.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SapSpendObservationScopeV2 {
    ExplicitActionSetOnly,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SapNoteConsumptionAuditV2 {
    pub note_id: String,
    pub supplied_spend_action_count: u32,
    pub scope: SapSpendObservationScopeV2,
    pub state: SapNoteConsumptionStateV2,
}

#[hdk_extern]
pub fn create_sap_transfer_v2(input: CreateSapTransferV2Input) -> ExternResult<Record> {
    load_sap_transfer_v2_config()?
        .require_enabled()
        .map_err(|error| guest(format!(
            "SAP transfer V2 is disabled by DNA policy: {error:?}"
        )))?;

    if input.input_claim_action_hashes.is_empty() {
        return Err(guest("SAP transfer V2 requires at least one input claim"));
    }
    if input.input_claim_action_hashes.len() > MAX_TRANSFER_INPUTS {
        return Err(guest(format!(
            "SAP transfer V2 exceeds the maximum of {MAX_TRANSFER_INPUTS} inputs"
        )));
    }

    let info = agent_info()?;
    let sender_did = did_for_author(&info.agent_initial_pubkey);

    let mut claims = Vec::with_capacity(input.input_claim_action_hashes.len());
    for claim_hash in input.input_claim_action_hashes {
        claims.push(load_exact_validated_collateral_claim(claim_hash)?);
    }

    let notes = notes_from_claims(&claims)?;
    let transfer = SapTransferV2::derive(
        sender_did,
        input.recipient_did,
        &notes,
        input.transfer_amount,
    )
    .map_err(|error| guest(format!("Unable to derive canonical SAP transfer: {error:?}")))?;
    let spend = SapTransferSpendRecordV2::from_validated_claims(transfer, &claims)
        .map_err(|error| guest(format!("Unable to derive FIN-SAFE-016 spend record: {error:?}")))?;

    let action_hash = create_entry(&EntryTypes::SapTransferSpendV2(SapTransferSpendV2Entry {
        spend,
    }))?;
    must_get_valid_record(action_hash)
}

#[hdk_extern]
pub fn get_verified_sap_transfer_v2(
    action_hash: ActionHash,
) -> ExternResult<VerifiedSapTransferSpendV2> {
    load_exact_verified_spend(action_hash)
}

/// Classify only the exact spend actions supplied by the caller.
///
/// The returned scope is permanently `ExplicitActionSetOnly` in V1. This helper
/// never claims global discovery, finality, or proactive double-spend prevention.
#[hdk_extern]
pub fn classify_sap_note_consumption_v2(
    input: ClassifySapNoteConsumptionV2Input,
) -> ExternResult<SapNoteConsumptionAuditV2> {
    if input.spend_action_hashes.len() > MAX_EXPLICIT_SPEND_OBSERVATIONS {
        return Err(guest(format!(
            "Explicit spend audit exceeds the maximum of {MAX_EXPLICIT_SPEND_OBSERVATIONS} actions"
        )));
    }

    let supplied_spend_action_count = u32::try_from(input.spend_action_hashes.len())
        .map_err(|_| guest("Explicit spend audit count exceeds u32"))?;
    let mut observations = Vec::with_capacity(input.spend_action_hashes.len());
    for action_hash in input.spend_action_hashes {
        observations.push(load_exact_verified_spend(action_hash)?.observation);
    }
    let state = classify_note_consumption(&input.note_id, &observations)
        .map_err(|error| guest(format!("SAP note-consumption classification failed: {error:?}")))?;

    Ok(SapNoteConsumptionAuditV2 {
        note_id: input.note_id,
        supplied_spend_action_count,
        scope: SapSpendObservationScopeV2::ExplicitActionSetOnly,
        state,
    })
}

fn load_exact_verified_spend(
    action_hash: ActionHash,
) -> ExternResult<VerifiedSapTransferSpendV2> {
    let record = must_get_valid_record(action_hash.clone())?;
    require_create_action(&record, "FIN-SAFE-016 transfer spend")?;
    require_exact_app_entry_type(
        &record,
        AppEntryDef::try_from(SapTransferUnitEntryTypes::SapTransferSpendV2)?,
        "FIN-SAFE-016 transfer spend",
    )?;
    let entry = record
        .entry()
        .to_app_option::<SapTransferSpendV2Entry>()
        .map_err(|error| guest(format!("Failed to decode FIN-SAFE-016 transfer spend: {error:?}")))?
        .ok_or_else(|| guest("FIN-SAFE-016 transfer spend entry is missing"))?;

    let mut claims = Vec::with_capacity(entry.spend.input_claims.len());
    for input in &entry.spend.input_claims {
        let claim_hash = ActionHash::try_from(input.claim_action_reference.clone())
            .map_err(|error| guest(format!("Invalid collateral-claim action reference: {error:?}")))?;
        claims.push(load_exact_validated_collateral_claim(claim_hash)?);
    }

    let action_author_did = did_for_author(record.action().author());
    let observation = entry
        .spend
        .to_fork_detection_observation(
            action_hash.to_string(),
            action_author_did.clone(),
            &claims,
        )
        .map_err(|error| guest(format!("FIN-SAFE-016 spend reconstruction failed: {error:?}")))?;

    Ok(VerifiedSapTransferSpendV2 {
        action_reference: action_hash.to_string(),
        action_author_did,
        spend: entry.spend,
        observation,
    })
}

fn load_exact_validated_collateral_claim(
    claim_hash: ActionHash,
) -> ExternResult<ValidatedCollateralClaimV2> {
    let claim_record = must_get_valid_record(claim_hash.clone())?;
    require_create_action(&claim_record, "FIN-SAFE-014 collateral claim")?;
    require_exact_app_entry_type(
        &claim_record,
        AppEntryDef::try_from(SapAccountUnitEntryTypes::SapCollateralClaimV2)?,
        "FIN-SAFE-014 collateral claim",
    )?;
    let claim_entry = claim_record
        .entry()
        .to_app_option::<SapCollateralClaimV2Entry>()
        .map_err(|error| guest(format!("Failed to decode FIN-SAFE-014 collateral claim: {error:?}")))?
        .ok_or_else(|| guest("FIN-SAFE-014 collateral claim entry is missing"))?;
    let claim_author_did = did_for_author(claim_record.action().author());

    let receipt_hash = ActionHash::try_from(
        claim_entry.claim.issuance_receipt_action_reference.clone(),
    )
    .map_err(|error| guest(format!("Invalid issuance-receipt action reference: {error:?}")))?;
    let receipt_record = must_get_valid_record(receipt_hash)?;
    require_create_action(&receipt_record, "FIN-SAFE-010 issuance receipt")?;
    require_exact_app_entry_type(
        &receipt_record,
        AppEntryDef::try_from(
            CollateralIssuanceUnitEntryTypes::CollateralSapIssuanceReceiptV2,
        )?,
        "FIN-SAFE-010 issuance receipt",
    )?;
    let receipt_entry = receipt_record
        .entry()
        .to_app_option::<CollateralSapIssuanceReceiptV2Entry>()
        .map_err(|error| guest(format!("Failed to decode FIN-SAFE-010 issuance receipt: {error:?}")))?
        .ok_or_else(|| guest("FIN-SAFE-010 issuance receipt entry is missing"))?;

    ValidatedCollateralClaimV2::from_valid_receipt(
        claim_hash.to_string(),
        claim_author_did,
        &claim_entry.claim,
        &receipt_entry.record,
    )
    .map_err(|error| guest(format!("FIN-SAFE-014 claim reconstruction failed: {error:?}")))
}

fn notes_from_claims(
    claims: &[ValidatedCollateralClaimV2],
) -> ExternResult<Vec<SapValueNoteV2>> {
    claims
        .iter()
        .map(|claim| {
            SapValueNoteV2::from_collateral_claim(claim)
                .map_err(|error| guest(format!("Unable to derive SAP value note from claim: {error:?}")))
        })
        .collect()
}

fn require_create_action(record: &Record, label: &str) -> ExternResult<()> {
    if record.action().action_type() == ActionType::Create {
        Ok(())
    } else {
        Err(guest(format!("{label} is not an exact Create action")))
    }
}

fn require_exact_app_entry_type(
    record: &Record,
    expected: AppEntryDef,
    label: &str,
) -> ExternResult<()> {
    match record.action().app_entry_def() {
        Some(actual) if actual == &expected => Ok(()),
        Some(actual) => Err(guest(format!(
            "{label} has wrong app entry definition: expected {expected:?}, got {actual:?}"
        ))),
        None => Err(guest(format!("{label} is not an application entry"))),
    }
}

fn guest(message: impl Into<String>) -> WasmError {
    wasm_error!(WasmErrorInner::Guest(message.into()))
}
