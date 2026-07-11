// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root//! Property Transfer Integrity Zome
use hdi::prelude::*;

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Transfer {
    pub id: String,
    pub property_id: String,
    pub from_did: String,
    pub to_did: String,
    pub transfer_type: TransferType,
    pub price: Option<f64>,
    pub currency: Option<String>,
    pub conditions: Vec<TransferCondition>,
    pub status: TransferStatus,
    pub initiated: Timestamp,
    pub completed: Option<Timestamp>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TransferType {
    Sale,
    Gift,
    Inheritance,
    CourtOrder,
    Exchange,
    Other,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct TransferCondition {
    pub condition_type: ConditionType,
    pub description: String,
    pub satisfied: bool,
    pub verified_by: Option<String>,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum ConditionType {
    PaymentReceived,
    InspectionComplete,
    TitleClear,
    DocumentsSigned,
    TaxesPaid,
    Custom(String),
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum TransferStatus {
    Initiated,
    AwaitingAcceptance,
    InEscrow,
    ConditionsPending,
    Completed,
    Cancelled,
    Disputed,
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Escrow {
    pub id: String,
    pub transfer_id: String,
    pub escrow_agent_did: Option<String>,
    pub amount: f64,
    pub currency: String,
    pub funded: bool,
    pub release_conditions: Vec<String>,
    pub created: Timestamp,
    pub released: Option<Timestamp>,
}

/// Anchor entry for deterministic link bases from strings
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Transfer(Transfer),
    Escrow(Escrow),
    #[entry_type(visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    PropertyToTransfers,
    SellerToTransfers,
    BuyerToTransfers,
    TransferToEscrow,
}

/// Genesis self-check
#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback using FlatOp pattern
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Transfer(transfer) => {
                    validate_create_transfer(EntryCreationAction::Create(action), transfer)
                }
                EntryTypes::Escrow(escrow) => {
                    validate_create_escrow(EntryCreationAction::Create(action), escrow)
                }
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Transfer(transfer) => validate_update_transfer(action, transfer),
                EntryTypes::Escrow(escrow) => validate_update_escrow(action, escrow),
                EntryTypes::Anchor(_) => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { link_type, .. } => match link_type {
            LinkTypes::PropertyToTransfers => Ok(ValidateCallbackResult::Valid),
            LinkTypes::SellerToTransfers => Ok(ValidateCallbackResult::Valid),
            LinkTypes::BuyerToTransfers => Ok(ValidateCallbackResult::Valid),
            LinkTypes::TransferToEscrow => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_transfer(
    action: EntryCreationAction,
    transfer: Transfer,
) -> ExternResult<ValidateCallbackResult> {
    // Bind the transfer to its committer -- initiate_transfer already
    // derives `from_did` from agent_info() coordinator-side with zero user
    // input, so this never rejects a legitimate initiation; it's the real
    // DHT-level enforcement a modified coordinator could otherwise bypass
    // (P0 author-binding gap).
    let expected_seller = format!("did:mycelix:{}", action.author());
    if transfer.from_did != expected_seller {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer seller (from_did) must be the committing agent (forgery)".to_string(),
        ));
    }

    if !transfer.from_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Seller must be a valid DID".into(),
        ));
    }
    if !transfer.to_did.starts_with("did:") {
        return Ok(ValidateCallbackResult::Invalid(
            "Buyer must be a valid DID".into(),
        ));
    }
    if transfer.from_did == transfer.to_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot transfer to yourself".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_transfer(
    action: Update,
    _transfer: Transfer,
) -> ExternResult<ValidateCallbackResult> {
    // Every status-changing coordinator function (cancel/accept/dispute/
    // complete/satisfy_condition/add_condition) updates the same Transfer
    // entry; bind the update to a party of the ORIGINAL (pre-update)
    // transfer so an unrelated third agent can't forge a status change --
    // otherwise a modified coordinator could bypass every one of those
    // functions' own requester checks. Using the original (not the new)
    // from_did/to_did also stops a party from renaming themselves out of
    // the transfer in the same update that forges the status change.
    let original_record = must_get_valid_record(action.original_action_address.clone())?;
    let original_transfer: Transfer = original_record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(e))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid original transfer entry".to_string()
        )))?;
    let committer_did = format!("did:mycelix:{}", action.author);
    if original_transfer.from_did != committer_did && original_transfer.to_did != committer_did {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer update must be committed by a party to the transfer".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_escrow(
    _action: EntryCreationAction,
    escrow: Escrow,
) -> ExternResult<ValidateCallbackResult> {
    // Deliberately NOT author-bound: escrow_agent_did names a third-party
    // escrow holder, not necessarily the committing agent, and the
    // coordinator doesn't establish who (seller, buyer, or an appointed
    // agent) is expected to call create_escrow. No clear self-referential
    // identity field to bind without a design decision this pass doesn't
    // make.
    if escrow.amount <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Escrow amount must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_update_escrow(_action: Update, escrow: Escrow) -> ExternResult<ValidateCallbackResult> {
    // Same rationale as validate_create_escrow above.
    if escrow.amount <= 0.0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Escrow amount must be positive".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_action() -> Create {
        Create {
            author: AgentPubKey::from_raw_36(vec![0u8; 36]),
            timestamp: Timestamp::from_micros(0),
            action_seq: 0,
            prev_action: ActionHash::from_raw_36(vec![0u8; 36]),
            entry_type: EntryType::App(AppEntryDef::new(
                EntryDefIndex::from(0),
                0.into(),
                EntryVisibility::Public,
            )),
            entry_hash: EntryHash::from_raw_36(vec![0u8; 36]),
            weight: Default::default(),
        }
    }

    fn test_author_did() -> String {
        format!("did:mycelix:{}", test_action().author)
    }

    fn valid_transfer(from_did: String) -> Transfer {
        Transfer {
            id: "transfer:test:0".to_string(),
            property_id: "property:test:0".to_string(),
            from_did,
            to_did: "did:mycelix:buyer".to_string(),
            transfer_type: TransferType::Sale,
            price: Some(100.0),
            currency: Some("USD".to_string()),
            conditions: vec![],
            status: TransferStatus::Initiated,
            initiated: Timestamp::from_micros(0),
            completed: None,
        }
    }

    #[test]
    fn test_create_transfer_valid() {
        let transfer = valid_transfer(test_author_did());
        let result =
            validate_create_transfer(EntryCreationAction::Create(test_action()), transfer).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_transfer_seller_forgery_rejected() {
        // from_did claims test_author_did(), but the committing action's
        // author is a different agent.
        let mut forged_action = test_action();
        forged_action.author = AgentPubKey::from_raw_36(vec![1u8; 36]);
        let transfer = valid_transfer(test_author_did());
        let result =
            validate_create_transfer(EntryCreationAction::Create(forged_action), transfer).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_transfer_self_transfer_rejected() {
        let mut transfer = valid_transfer(test_author_did());
        transfer.to_did = test_author_did();
        let result =
            validate_create_transfer(EntryCreationAction::Create(test_action()), transfer).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_create_escrow_valid() {
        let escrow = Escrow {
            id: "escrow:test:0".to_string(),
            transfer_id: "transfer:test:0".to_string(),
            escrow_agent_did: None,
            amount: 100.0,
            currency: "USD".to_string(),
            funded: false,
            release_conditions: vec![],
            created: Timestamp::from_micros(0),
            released: None,
        };
        let result =
            validate_create_escrow(EntryCreationAction::Create(test_action()), escrow).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_create_escrow_negative_amount_rejected() {
        let escrow = Escrow {
            id: "escrow:test:0".to_string(),
            transfer_id: "transfer:test:0".to_string(),
            escrow_agent_did: None,
            amount: -1.0,
            currency: "USD".to_string(),
            funded: false,
            release_conditions: vec![],
            created: Timestamp::from_micros(0),
            released: None,
        };
        let result =
            validate_create_escrow(EntryCreationAction::Create(test_action()), escrow).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }
}
