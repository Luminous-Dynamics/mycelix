// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

mod conflicts;
pub use conflicts::*;

/// Transaction entry - represents a purchase in the marketplace
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Transaction {
    /// Buyer agent
    pub buyer: AgentPubKey,

    /// Seller agent
    pub seller: AgentPubKey,

    /// Listing being purchased
    pub listing_hash: ActionHash,

    /// Quantity purchased
    pub quantity: u32,

    /// Total price in cents (quantity * unit_price)
    pub total_price_cents: u64,

    /// Current status
    pub status: TransactionStatus,

    /// Creation timestamp
    pub created_at: Timestamp,

    /// Last update timestamp
    pub updated_at: Timestamp,

    /// Delivery tracking (optional)
    pub tracking_info: Option<String>,

    /// Epistemic classification
    /// Transactions are N1 (communal) agreements between buyer-seller
    pub epistemic: EpistemicClassification,
}

/// Transaction lifecycle states
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum TransactionStatus {
    /// Buyer created, awaiting seller confirmation
    Pending,

    /// Seller confirmed, payment/arrangement made
    Confirmed,

    /// Seller marked as shipped
    Shipped,

    /// Buyer confirmed delivery
    Delivered,

    /// Transaction completed successfully (triggers MATL update)
    Completed,

    /// Disputed by either party
    Disputed,

    /// Cancelled before completion
    Cancelled,
}

/// Epistemic classification
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct EpistemicClassification {
    pub empirical: EmpiricalLevel,
    pub normative: NormativeLevel,
    pub materiality: MaterialityLevel,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum EmpiricalLevel {
    E0Null,
    E1Testimonial,
    E2PrivateVerify,
    E3Cryptographic,
    E4PublicRepro,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum NormativeLevel {
    N0Personal,
    N1Communal,
    N2Network,
    N3Axiomatic,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum MaterialityLevel {
    M0Ephemeral,
    M1Temporal,
    M2Persistent,
    M3Foundational,
}

#[hdk_link_types]
pub enum LinkTypes {
    /// Buyer -> Transactions
    BuyerToTransactions,

    /// Seller -> Transactions
    SellerToTransactions,

    /// Listing -> Transactions
    ListingToTransactions,

    /// Stable Transaction root -> party conflict approvals
    TransactionToConflictApprovals,

    /// Stable Transaction root -> final conflict resolutions
    TransactionToConflictResolutions,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Transaction(Transaction),
    TransactionConflictApproval(TransactionConflictApproval),
    TransactionConflictResolution(TransactionConflictResolutionEntry),
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::Transaction(transaction) => {
                    validate_create_transaction(&transaction, &action)
                }
                EntryTypes::TransactionConflictApproval(approval) => {
                    validate_create_conflict_approval(&approval, &action)
                }
                EntryTypes::TransactionConflictResolution(resolution) => {
                    validate_create_conflict_resolution(&resolution, &action)
                }
            },
            OpEntry::UpdateEntry {
                app_entry, action, ..
            } => match app_entry {
                EntryTypes::Transaction(transaction) => {
                    validate_update_transaction(&transaction, &action)
                }
                EntryTypes::TransactionConflictApproval(_) => Ok(ValidateCallbackResult::Invalid(
                    "Transaction conflict approvals are immutable".into(),
                )),
                EntryTypes::TransactionConflictResolution(_) => Ok(ValidateCallbackResult::Invalid(
                    "Transaction conflict resolutions are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update_entry) => match update_entry {
            OpUpdate::Entry { app_entry, action } => match app_entry {
                EntryTypes::Transaction(transaction) => {
                    validate_update_transaction(&transaction, &action)
                }
                EntryTypes::TransactionConflictApproval(_) => Ok(ValidateCallbackResult::Invalid(
                    "Transaction conflict approvals are immutable".into(),
                )),
                EntryTypes::TransactionConflictResolution(_) => Ok(ValidateCallbackResult::Invalid(
                    "Transaction conflict resolutions are immutable".into(),
                )),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Transactions and conflict authority records are permanent audit evidence and cannot be deleted".into(),
        )),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_transaction_data(transaction: &Transaction) -> Result<(), String> {
    if transaction.quantity == 0 {
        return Err("Quantity must be at least 1".into());
    }
    if transaction.total_price_cents == 0 {
        return Err("Total price must be greater than zero".into());
    }
    if transaction.buyer == transaction.seller {
        return Err("Buyer and seller must be different agents".into());
    }
    if transaction.updated_at < transaction.created_at {
        return Err("Updated timestamp cannot be earlier than creation timestamp".into());
    }
    if transaction.epistemic.normative != NormativeLevel::N1Communal {
        return Err("Transactions must be N1 (communal agreement)".into());
    }
    Ok(())
}

fn validate_transaction(transaction: &Transaction) -> ExternResult<ValidateCallbackResult> {
    match validate_transaction_data(transaction) {
        Ok(()) => Ok(ValidateCallbackResult::Valid),
        Err(reason) => Ok(ValidateCallbackResult::Invalid(reason)),
    }
}

fn validate_create_transaction(
    transaction: &Transaction,
    action: &Create,
) -> ExternResult<ValidateCallbackResult> {
    if let Err(reason) = validate_create_transaction_fields(transaction, &action.author) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_transaction_fields(
    transaction: &Transaction,
    author: &AgentPubKey,
) -> Result<(), String> {
    validate_transaction_data(transaction)?;

    if author != &transaction.buyer {
        return Err("Transaction creation must be authored by the buyer".into());
    }
    if transaction.status != TransactionStatus::Pending {
        return Err("New transactions must start in Pending status".into());
    }
    if transaction.tracking_info.is_some() {
        return Err("New transactions cannot contain tracking information".into());
    }
    if transaction.epistemic.empirical != EmpiricalLevel::E1Testimonial {
        return Err("New transactions must start at E1 testimonial evidence".into());
    }
    if transaction.epistemic.materiality != MaterialityLevel::M1Temporal {
        return Err("New transactions must start at M1 temporal materiality".into());
    }

    Ok(())
}

fn validate_update_transaction(
    transaction: &Transaction,
    action: &Update,
) -> ExternResult<ValidateCallbackResult> {
    let original_entry = must_get_entry(action.original_entry_address.clone())?;
    let original_transaction = match original_entry.as_content() {
        Entry::App(app_entry) => Transaction::try_from(app_entry.clone().into_sb()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Could not decode the transaction being updated: {e:?}"
            )))
        })?,
        _ => {
            return Ok(ValidateCallbackResult::Invalid(
                "Transaction updates must reference an application entry".into(),
            ));
        }
    };

    if let Err(reason) = validate_transaction_update_fields(
        &original_transaction,
        transaction,
        &action.author,
    ) {
        return Ok(ValidateCallbackResult::Invalid(reason));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_transaction_update_fields(
    original: &Transaction,
    updated: &Transaction,
    updater: &AgentPubKey,
) -> Result<(), String> {
    validate_transaction_data(updated)?;

    if updater != &original.buyer && updater != &original.seller {
        return Err("Only the buyer or seller may update a transaction".into());
    }

    if updated.buyer != original.buyer
        || updated.seller != original.seller
        || updated.listing_hash != original.listing_hash
        || updated.quantity != original.quantity
        || updated.total_price_cents != original.total_price_cents
        || updated.created_at != original.created_at
    {
        return Err("Transaction parties, listing, quantity, price, and creation time are immutable".into());
    }

    if updated.updated_at < original.updated_at {
        return Err("Transaction update timestamp cannot move backwards".into());
    }

    if updated.epistemic.empirical != original.epistemic.empirical
        || updated.epistemic.normative != original.epistemic.normative
    {
        return Err("Transaction empirical and normative classifications are immutable".into());
    }

    if updated.epistemic.materiality != original.epistemic.materiality {
        return Err(
            "Transaction materiality is immutable; Finance settlement is represented by its authoritative external record".into(),
        );
    }

    validate_status_transition(&original.status, &updated.status, updater, original)?;
    validate_tracking_transition(original, updated)?;

    Ok(())
}

fn validate_status_transition(
    from: &TransactionStatus,
    to: &TransactionStatus,
    updater: &AgentPubKey,
    transaction: &Transaction,
) -> Result<(), String> {
    use TransactionStatus::*;

    let is_buyer = updater == &transaction.buyer;
    let is_seller = updater == &transaction.seller;
    let is_party = is_buyer || is_seller;

    let allowed = match (from, to) {
        (Pending, Confirmed) => is_seller,
        (Pending, Cancelled | Disputed) => is_party,
        (Confirmed, Shipped) => is_seller,
        (Confirmed, Cancelled | Disputed) => is_party,
        (Shipped, Delivered) => is_buyer,
        (Shipped, Disputed) => is_party,
        // Economic completion cannot be represented by a locally forgeable
        // status update. Finance settlement remains an external authoritative
        // record while Marketplace lifecycle state remains Delivered.
        (Delivered, Disputed) => is_party,
        _ => false,
    };

    if !allowed {
        return Err(format!(
            "Illegal or unauthorized transaction transition: {from:?} -> {to:?}"
        ));
    }

    Ok(())
}

fn validate_tracking_transition(
    original: &Transaction,
    updated: &Transaction,
) -> Result<(), String> {
    if let Some(tracking) = &updated.tracking_info {
        if tracking.trim().is_empty() {
            return Err("Tracking information cannot be empty".into());
        }
        if tracking.len() > 256 {
            return Err("Tracking information cannot exceed 256 characters".into());
        }
    }

    match (&original.status, &updated.status) {
        (TransactionStatus::Confirmed, TransactionStatus::Shipped) => Ok(()),
        _ if original.tracking_info == updated.tracking_info => Ok(()),
        _ => Err("Tracking information may only change when a transaction is marked shipped".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_agent(byte: u8) -> AgentPubKey {
        AgentPubKey::from_raw_36(vec![byte; 36])
    }

    fn valid_transaction() -> Transaction {
        Transaction {
            buyer: mock_agent(1),
            seller: mock_agent(2),
            listing_hash: ActionHash::from_raw_36(vec![3u8; 36]),
            quantity: 1,
            total_price_cents: 1999,
            status: TransactionStatus::Pending,
            created_at: Timestamp::from_micros(1000000),
            updated_at: Timestamp::from_micros(1000000),
            tracking_info: None,
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E1Testimonial,
                normative: NormativeLevel::N1Communal,
                materiality: MaterialityLevel::M1Temporal,
            },
        }
    }

    #[test]
    fn test_validate_transaction_valid() {
        let tx = valid_transaction();
        let result = validate_transaction(&tx).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_transaction_zero_quantity() {
        let tx = Transaction { quantity: 0, ..valid_transaction() };
        let result = validate_transaction(&tx).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_transaction_zero_price() {
        let tx = Transaction { total_price_cents: 0, ..valid_transaction() };
        let result = validate_transaction(&tx).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_transaction_same_buyer_seller() {
        let tx = Transaction { buyer: mock_agent(1), seller: mock_agent(1), ..valid_transaction() };
        let result = validate_transaction(&tx).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_validate_transaction_wrong_normative_level() {
        let tx = Transaction {
            epistemic: EpistemicClassification {
                empirical: EmpiricalLevel::E1Testimonial,
                normative: NormativeLevel::N0Personal,
                materiality: MaterialityLevel::M1Temporal,
            },
            ..valid_transaction()
        };
        let result = validate_transaction(&tx).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    #[test]
    fn test_all_transaction_statuses() {
        let statuses = vec![
            TransactionStatus::Pending, TransactionStatus::Confirmed,
            TransactionStatus::Shipped, TransactionStatus::Delivered,
            TransactionStatus::Completed, TransactionStatus::Disputed,
            TransactionStatus::Cancelled,
        ];
        assert_eq!(statuses.len(), 7);
    }

    #[test]
    fn test_transaction_serde_roundtrip() {
        let tx = valid_transaction();
        let json = serde_json::to_string(&tx).unwrap();
        let parsed: Transaction = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.quantity, tx.quantity);
        assert_eq!(parsed.total_price_cents, tx.total_price_cents);
    }

    #[test]
    fn test_validate_transaction_large_quantity() {
        let tx = Transaction { quantity: 1_000_000, total_price_cents: 1_000_000_000, ..valid_transaction() };
        let result = validate_transaction(&tx).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn test_validate_transaction_min_valid() {
        let tx = Transaction { quantity: 1, total_price_cents: 1, ..valid_transaction() };
        let result = validate_transaction(&tx).unwrap();
        assert_eq!(result, ValidateCallbackResult::Valid);
    }

    #[test]
    fn create_must_be_authored_by_buyer() {
        let tx = valid_transaction();
        assert!(validate_create_transaction_fields(&tx, &tx.buyer).is_ok());
        let error = validate_create_transaction_fields(&tx, &tx.seller).unwrap_err();
        assert!(error.contains("authored by the buyer"));
    }

    #[test]
    fn create_must_start_pending_without_tracking() {
        let mut tx = valid_transaction();
        tx.status = TransactionStatus::Confirmed;
        assert!(validate_create_transaction_fields(&tx, &tx.buyer).is_err());

        let mut tx = valid_transaction();
        tx.tracking_info = Some("tracking".into());
        assert!(validate_create_transaction_fields(&tx, &tx.buyer).is_err());
    }

    #[test]
    fn seller_can_confirm_pending_transaction() {
        let original = valid_transaction();
        let mut updated = original.clone();
        updated.status = TransactionStatus::Confirmed;
        updated.updated_at = Timestamp::from_micros(2_000_000);

        assert!(validate_transaction_update_fields(
            &original,
            &updated,
            &original.seller,
        )
        .is_ok());
    }

    #[test]
    fn buyer_cannot_confirm_pending_transaction() {
        let original = valid_transaction();
        let mut updated = original.clone();
        updated.status = TransactionStatus::Confirmed;
        updated.updated_at = Timestamp::from_micros(2_000_000);

        let error = validate_transaction_update_fields(&original, &updated, &original.buyer)
            .unwrap_err();
        assert!(error.contains("Illegal or unauthorized"));
    }

    #[test]
    fn buyer_can_confirm_delivery_but_not_forge_completion() {
        let mut shipped = valid_transaction();
        shipped.status = TransactionStatus::Shipped;
        shipped.tracking_info = Some("carrier-123".into());

        let mut delivered = shipped.clone();
        delivered.status = TransactionStatus::Delivered;
        delivered.updated_at = Timestamp::from_micros(2_000_000);
        assert!(validate_transaction_update_fields(&shipped, &delivered, &shipped.buyer).is_ok());

        let mut completed = delivered.clone();
        completed.status = TransactionStatus::Completed;
        completed.updated_at = Timestamp::from_micros(3_000_000);
        let error = validate_transaction_update_fields(&delivered, &completed, &delivered.buyer)
            .unwrap_err();
        assert!(error.contains("Illegal or unauthorized"));
    }

    #[test]
    fn transaction_core_terms_are_immutable() {
        let original = valid_transaction();
        let mut updated = original.clone();
        updated.status = TransactionStatus::Confirmed;
        updated.total_price_cents += 1;
        updated.updated_at = Timestamp::from_micros(2_000_000);

        let error = validate_transaction_update_fields(&original, &updated, &original.seller)
            .unwrap_err();
        assert!(error.contains("immutable"));
    }

    #[test]
    fn terminal_and_skipped_transitions_are_rejected() {
        let original = valid_transaction();
        let mut skipped = original.clone();
        skipped.status = TransactionStatus::Shipped;
        skipped.updated_at = Timestamp::from_micros(2_000_000);
        assert!(validate_transaction_update_fields(&original, &skipped, &original.seller).is_err());

        let mut completed = valid_transaction();
        completed.status = TransactionStatus::Completed;
        completed.epistemic.materiality = MaterialityLevel::M2Persistent;
        let mut resurrected = completed.clone();
        resurrected.status = TransactionStatus::Pending;
        resurrected.updated_at = Timestamp::from_micros(2_000_000);
        assert!(validate_transaction_update_fields(&completed, &resurrected, &completed.buyer).is_err());
    }

    #[test]
    fn tracking_only_changes_when_shipped() {
        let mut confirmed = valid_transaction();
        confirmed.status = TransactionStatus::Confirmed;
        let mut shipped = confirmed.clone();
        shipped.status = TransactionStatus::Shipped;
        shipped.tracking_info = Some("carrier-123".into());
        shipped.updated_at = Timestamp::from_micros(2_000_000);
        assert!(validate_transaction_update_fields(&confirmed, &shipped, &confirmed.seller).is_ok());

        let mut delivered = shipped.clone();
        delivered.status = TransactionStatus::Delivered;
        delivered.tracking_info = Some("tampered".into());
        delivered.updated_at = Timestamp::from_micros(3_000_000);
        assert!(validate_transaction_update_fields(&shipped, &delivered, &shipped.buyer).is_err());
    }

}
