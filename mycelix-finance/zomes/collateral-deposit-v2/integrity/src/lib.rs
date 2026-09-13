#![deny(unsafe_code)]
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! FIN-SAFE-008 collateral deposit request integrity zome.
//!
//! V2 requests are immutable intent records. They carry no caller-authoritative
//! price, minted amount, settlement status, or custody claim.

use finance_holochain_contracts::{
    CollateralDepositRequestV2Entry, MAX_CREATE_TIMESTAMP_SKEW_MICROS,
};
use hdi::prelude::*;
use mycelix_bridge_entry_types::{did_for_author, require_did_is_author};

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    CollateralDepositRequestV2(CollateralDepositRequestV2Entry),
}

/// Reserved deliberately: FIN-SAFE-008 v1 uses exact action hashes and local
/// source-chain queries rather than a globally mutable/poisonable request index.
#[hdk_link_types]
pub enum LinkTypes {
    ReservedRequestIndex,
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

fn validate_request_create(
    action: Create,
    entry: CollateralDepositRequestV2Entry,
) -> ExternResult<ValidateCallbackResult> {
    let request = match entry.to_model() {
        Ok(request) => request,
        Err(message) => return Ok(ValidateCallbackResult::Invalid(message)),
    };

    if let Err(error) = request.validate_action_timestamp(
        action.timestamp.as_micros(),
        MAX_CREATE_TIMESTAMP_SKEW_MICROS,
    ) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid V2 collateral deposit request: {error:?}"
        )));
    }

    let author_did = did_for_author(&action.author);
    if let ValidateCallbackResult::Invalid(message) = require_did_is_author(
        "CollateralDepositRequestV2",
        "depositor_did",
        &request.depositor_did,
        &author_did,
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
                EntryTypes::CollateralDepositRequestV2(entry) => {
                    validate_request_create(action, entry)
                }
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "V2 collateral deposit requests are immutable and cannot be updated".into(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::StoreRecord(store_record) => match store_record {
            OpRecord::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::CollateralDepositRequestV2(entry) => {
                    validate_request_create(action, entry)
                }
            },
            OpRecord::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "V2 collateral deposit requests are immutable and cannot be updated".into(),
            )),
            OpRecord::CreateLink { .. } | OpRecord::DeleteLink { .. } => {
                Ok(ValidateCallbackResult::Invalid(
                    "FIN-SAFE-008 v1 does not authorize collateral request links".into(),
                ))
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "V2 collateral deposit requests are immutable and cannot be updated".into(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "V2 collateral deposit requests are immutable and cannot be deleted".into(),
        )),
        FlatOp::RegisterCreateLink { .. } | FlatOp::RegisterDeleteLink { .. } => {
            Ok(ValidateCallbackResult::Invalid(
                "FIN-SAFE-008 v1 does not authorize collateral request links".into(),
            ))
        }
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use finance_collateral_deposit::{COLLATERAL_DEPOSIT_NONCE_BYTES, CollateralDepositRequestV2};

    fn action(author_byte: u8, timestamp_micros: i64) -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![author_byte; 36]),
            timestamp: Timestamp::from_micros(timestamp_micros),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0; 36]),
            entry_type: EntryType::CapClaim,
            entry_hash: EntryHash::from_raw_36(vec![0; 36]),
            weight: Default::default(),
        }
    }

    fn author_did(author_byte: u8) -> String {
        did_for_author(&AgentPubKey::from_raw_36(vec![author_byte; 36]))
    }

    fn valid_entry(author_byte: u8, created_at_micros: i64) -> CollateralDepositRequestV2Entry {
        CollateralDepositRequestV2::new(
            author_did(author_byte),
            "ETH".into(),
            100,
            [7; COLLATERAL_DEPOSIT_NONCE_BYTES],
            created_at_micros,
        )
        .expect("valid request")
        .into()
    }

    #[test]
    fn exact_author_and_timestamp_request_is_valid() {
        let result = validate_request_create(action(1, 1_000_000), valid_entry(1, 1_000_000))
            .expect("validation result");
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn forged_depositor_is_rejected() {
        let result = validate_request_create(action(1, 1_000_000), valid_entry(2, 1_000_000))
            .expect("validation result");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn tampered_identity_is_rejected() {
        let mut entry = valid_entry(1, 1_000_000);
        entry.deposit_id.push('x');
        let result =
            validate_request_create(action(1, 1_000_000), entry).expect("validation result");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn malformed_nonce_is_rejected() {
        let mut entry = valid_entry(1, 1_000_000);
        entry.request_nonce.pop();
        let result =
            validate_request_create(action(1, 1_000_000), entry).expect("validation result");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn claimed_time_outside_action_policy_is_rejected() {
        let entry = valid_entry(1, 1_000_000);
        let result = validate_request_create(
            action(1, 1_000_000 + MAX_CREATE_TIMESTAMP_SKEW_MICROS + 1),
            entry,
        )
        .expect("validation result");
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
