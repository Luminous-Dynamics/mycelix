// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Balances Integrity Zome
//!
//! Tracks credit/debit balances for listeners and artists.
//! Listeners can pre-fund accounts, artists can request cashouts.
//! All on Holochain until cashout - then settles on-chain.

use hdi::prelude::*;

/// Listener account - tracks pre-funded balance
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ListenerAccount {
    /// Owner's agent pub key
    pub owner: AgentPubKey,
    /// Ethereum address for deposits/refunds
    pub eth_address: String,
    /// Current balance (in wei)
    pub balance: u64,
    /// Total deposited all-time
    pub total_deposited: u64,
    /// Total spent on plays
    pub total_spent: u64,
    /// Account creation timestamp
    pub created_at: Timestamp,
    /// Last activity timestamp
    pub updated_at: Timestamp,
}

/// Artist account - tracks earnings awaiting cashout
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct ArtistAccount {
    /// Owner's agent pub key
    pub owner: AgentPubKey,
    /// Ethereum address for payouts
    pub eth_address: String,
    /// Pending earnings (not yet cashed out)
    pub pending_balance: u64,
    /// Total earned all-time
    pub total_earned: u64,
    /// Total cashed out
    pub total_cashed_out: u64,
    /// Account creation timestamp
    pub created_at: Timestamp,
    /// Last activity timestamp
    pub updated_at: Timestamp,
}

/// Deposit record - when listener funds their account
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Deposit {
    /// Listener's agent pub key
    pub listener: AgentPubKey,
    /// Amount deposited (in wei)
    pub amount: u64,
    /// On-chain transaction hash
    pub tx_hash: String,
    /// Block number of deposit
    pub block_number: u64,
    /// Timestamp
    pub deposited_at: Timestamp,
    /// Verification status
    pub verified: bool,
}

/// Cashout request - artist requesting payout
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct CashoutRequest {
    /// Artist's agent pub key
    pub artist: AgentPubKey,
    /// Amount to cash out (in wei)
    pub amount: u64,
    /// Destination Ethereum address
    pub eth_address: String,
    /// Request timestamp
    pub requested_at: Timestamp,
    /// Status
    pub status: CashoutStatus,
    /// Transaction hash (if completed)
    pub tx_hash: Option<String>,
    /// Completion timestamp
    pub completed_at: Option<Timestamp>,
}

/// Cashout status
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum CashoutStatus {
    /// Request submitted
    Pending,
    /// Being processed
    Processing,
    /// Completed successfully
    Completed,
    /// Failed (will be retried)
    Failed,
    /// Cancelled by artist
    Cancelled,
}

/// Transfer record - internal transfer between accounts
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Transfer {
    /// From (listener agent)
    pub from: AgentPubKey,
    /// To (artist agent)
    pub to: AgentPubKey,
    /// Amount transferred
    pub amount: u64,
    /// Reason (play settlement, tip, etc.)
    pub reason: TransferReason,
    /// Reference (settlement batch hash, etc.)
    pub reference: Option<ActionHash>,
    /// Timestamp
    pub transferred_at: Timestamp,
}

/// Transfer reason
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub enum TransferReason {
    /// Play settlement batch
    PlaySettlement,
    /// Direct tip from listener
    Tip,
    /// Patronage payment
    Patronage,
    /// Download purchase
    Download,
    /// NFT access
    NftAccess,
}

/// Link types
#[hdk_link_types]
pub enum LinkTypes {
    /// Agent -> Their listener account
    AgentToListenerAccount,
    /// Agent -> Their artist account
    AgentToArtistAccount,
    /// Agent -> Their deposits
    AgentToDeposits,
    /// Agent -> Their cashout requests
    AgentToCashouts,
    /// Agent -> Transfers (as sender or recipient)
    AgentToTransfers,
}

/// Entry types
#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    ListenerAccount(ListenerAccount),
    ArtistAccount(ArtistAccount),
    Deposit(Deposit),
    CashoutRequest(CashoutRequest),
    Transfer(Transfer),
}

fn valid_eth_address(address: &str) -> bool {
    address.starts_with("0x")
        && address.len() == 42
        && address[2..].bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn listener_starts_at_zero(account: &ListenerAccount) -> bool {
    account.balance == 0 && account.total_deposited == 0 && account.total_spent == 0
}

fn artist_starts_at_zero(account: &ArtistAccount) -> bool {
    account.pending_balance == 0 && account.total_earned == 0 && account.total_cashed_out == 0
}

/// Validation
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::ListenerAccount(account) => validate_listener_account(account, action),
                EntryTypes::ArtistAccount(account) => validate_artist_account(account, action),
                EntryTypes::Deposit(deposit) => validate_deposit(deposit, action),
                EntryTypes::CashoutRequest(cashout) => validate_cashout(cashout, action),
                EntryTypes::Transfer(transfer) => validate_transfer(transfer, action),
            },
            OpEntry::UpdateEntry { .. } => Ok(ValidateCallbackResult::Invalid(
                "Economic entries are immutable until an authorized settlement protocol is active"
                    .to_string(),
            )),
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Invalid(
            "Economic entry updates are disabled until an authorized settlement protocol is active"
                .to_string(),
        )),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Invalid(
            "Economic ledger entries cannot be deleted".to_string(),
        )),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Economic ledger links cannot be deleted".to_string(),
        )),
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address,
            tag: _,
            action,
        } => validate_create_link(link_type, target_address, action),
        _ => Ok(ValidateCallbackResult::Valid),
    }
}

fn validate_create_link(
    link_type: LinkTypes,
    target_address: AnyLinkableHash,
    action: CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    let Some(target_hash) = target_address.into_action_hash() else {
        return Ok(ValidateCallbackResult::Invalid(
            "Economic index targets must be action hashes".to_string(),
        ));
    };
    let record = must_get_valid_record(target_hash)?;
    if record.action().author() != &action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Only an economic entry author may index that entry".to_string(),
        ));
    }

    let owner_matches = match link_type {
        LinkTypes::AgentToListenerAccount => record
            .entry()
            .to_app_option::<ListenerAccount>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .is_some_and(|entry| entry.owner == action.author),
        LinkTypes::AgentToArtistAccount => record
            .entry()
            .to_app_option::<ArtistAccount>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .is_some_and(|entry| entry.owner == action.author),
        LinkTypes::AgentToDeposits => record
            .entry()
            .to_app_option::<Deposit>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .is_some_and(|entry| entry.listener == action.author),
        LinkTypes::AgentToCashouts => record
            .entry()
            .to_app_option::<CashoutRequest>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .is_some_and(|entry| entry.artist == action.author),
        LinkTypes::AgentToTransfers => record
            .entry()
            .to_app_option::<Transfer>()
            .map_err(|error| wasm_error!(WasmErrorInner::Guest(error.to_string())))?
            .is_some_and(|entry| entry.from == action.author || entry.to == action.author),
    };

    if !owner_matches {
        return Ok(ValidateCallbackResult::Invalid(
            "Economic index type and entry owner must match the link author".to_string(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

fn validate_listener_account(
    account: ListenerAccount,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Owner must match author
    if account.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Account owner must match action author".to_string(),
        ));
    }

    if !listener_starts_at_zero(&account) {
        return Ok(ValidateCallbackResult::Invalid(
            "New listener accounts must start with zero balances and totals".to_string(),
        ));
    }

    // Must have valid Ethereum address (basic check)
    if !valid_eth_address(&account.eth_address) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid Ethereum address format".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_artist_account(
    account: ArtistAccount,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Owner must match author
    if account.owner != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Account owner must match action author".to_string(),
        ));
    }

    if !artist_starts_at_zero(&account) {
        return Ok(ValidateCallbackResult::Invalid(
            "New artist accounts must start with zero balances and totals".to_string(),
        ));
    }

    // Must have valid Ethereum address
    if !valid_eth_address(&account.eth_address) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid Ethereum address format".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_deposit(deposit: Deposit, action: Create) -> ExternResult<ValidateCallbackResult> {
    if deposit.listener != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Deposit listener must match action author".to_string(),
        ));
    }

    if deposit.verified {
        return Ok(ValidateCallbackResult::Invalid(
            "New deposits must be unverified".to_string(),
        ));
    }
    // Deposit must have a transaction hash
    if deposit.tx_hash.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Deposit must have a transaction hash".to_string(),
        ));
    }

    // Amount must be positive
    if deposit.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Deposit amount must be greater than 0".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_cashout(
    cashout: CashoutRequest,
    action: Create,
) -> ExternResult<ValidateCallbackResult> {
    // Artist must match author
    if cashout.artist != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Cashout artist must match action author".to_string(),
        ));
    }

    // Amount must be positive
    if cashout.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Cashout amount must be greater than 0".to_string(),
        ));
    }

    if !valid_eth_address(&cashout.eth_address) {
        return Ok(ValidateCallbackResult::Invalid(
            "Invalid cashout Ethereum address format".to_string(),
        ));
    }

    // New cashouts must be pending
    if cashout.status != CashoutStatus::Pending {
        return Ok(ValidateCallbackResult::Invalid(
            "New cashout requests must have Pending status".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_transfer(transfer: Transfer, action: Create) -> ExternResult<ValidateCallbackResult> {
    if transfer.from != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer sender must match action author".to_string(),
        ));
    }
    // Amount must be positive
    if transfer.amount == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer amount must be greater than 0".to_string(),
        ));
    }

    // From and to must be different
    if transfer.from == transfer.to {
        return Ok(ValidateCallbackResult::Invalid(
            "Cannot transfer to self".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

#[cfg(test)]
mod tests {
    use super::valid_eth_address;

    #[test]
    fn ethereum_address_validation_rejects_shape_only_impostors() {
        assert!(valid_eth_address(
            "0x1234567890abcdef1234567890abcdef12345678"
        ));
        assert!(!valid_eth_address(
            "0xzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"
        ));
        assert!(!valid_eth_address("0x1234"));
    }
}
