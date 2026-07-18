// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Balances Coordinator Zome
//!
//! Manages listener and artist accounts, deposits, cashouts, and transfers.
//! Everything happens on Holochain until cashout - minimizing on-chain costs.
//! Holochain 0.6 compatible (hdk 0.6)

use balances_integrity::*;
use hdk::prelude::*;
use mycelix_bridge_common::{CivicRequirement, civic_requirement_proposal, gate_civic};

fn require_consciousness(requirement: &CivicRequirement, action_name: &str) -> ExternResult<()> {
    gate_civic("music_bridge", requirement, action_name).map(|_| ())
}

/// Helper to ensure a path exists and return its entry hash
fn ensure_path(path: Path, link_type: LinkTypes) -> ExternResult<EntryHash> {
    let typed = path.typed(link_type)?;
    typed.ensure()?;
    typed.path_entry_hash()
}

/// Create or get listener account
#[hdk_extern]
pub fn get_or_create_listener_account(eth_address: String) -> ExternResult<ListenerAccount> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    // Check if account exists
    if let Some(account) = get_listener_account(my_agent.clone())? {
        return Ok(account);
    }

    // Create new account
    let now = sys_time()?;
    let account = ListenerAccount {
        owner: my_agent.clone(),
        eth_address,
        balance: 0,
        total_deposited: 0,
        total_spent: 0,
        created_at: now,
        updated_at: now,
    };

    let action_hash = create_entry(&EntryTypes::ListenerAccount(account.clone()))?;

    // Link to agent
    let account_path = Path::from(format!("listener_account/{}", my_agent));
    let account_hash = ensure_path(account_path, LinkTypes::AgentToListenerAccount)?;
    create_link(
        account_hash,
        action_hash,
        LinkTypes::AgentToListenerAccount,
        (),
    )?;

    Ok(account)
}

/// Get listener account
fn get_listener_account(agent: AgentPubKey) -> ExternResult<Option<ListenerAccount>> {
    let account_path = Path::from(format!("listener_account/{}", agent));
    let typed_path = account_path.typed(LinkTypes::AgentToListenerAccount)?;
    let filter = LinkTypeFilter::try_from(LinkTypes::AgentToListenerAccount)?;
    let links = get_links(
        LinkQuery::new(typed_path.path_entry_hash()?, filter),
        GetStrategy::default(),
    )?;

    for link in links.iter().rev() {
        if let Some(action_hash) = link.target.clone().into_action_hash() {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                let author_matches = record.action().author() == &agent;
                let account: Option<ListenerAccount> =
                    record.entry().to_app_option().map_err(|e| wasm_error!(e))?;
                if let Some(account) = account {
                    if author_matches && account.owner == agent {
                        return Ok(Some(account));
                    }
                }
            }
        }
    }

    Ok(None)
}

/// Create or get artist account
#[hdk_extern]
pub fn get_or_create_artist_account(eth_address: String) -> ExternResult<ArtistAccount> {
    let my_agent = agent_info()?.agent_initial_pubkey;

    // Check if account exists
    if let Some(account) = get_artist_account(my_agent.clone())? {
        return Ok(account);
    }

    // Create new account
    let now = sys_time()?;
    let account = ArtistAccount {
        owner: my_agent.clone(),
        eth_address,
        pending_balance: 0,
        total_earned: 0,
        total_cashed_out: 0,
        created_at: now,
        updated_at: now,
    };

    let action_hash = create_entry(&EntryTypes::ArtistAccount(account.clone()))?;

    // Link to agent
    let account_path = Path::from(format!("artist_account/{}", my_agent));
    let account_hash = ensure_path(account_path, LinkTypes::AgentToArtistAccount)?;
    create_link(
        account_hash,
        action_hash,
        LinkTypes::AgentToArtistAccount,
        (),
    )?;

    Ok(account)
}

/// Get artist account
fn get_artist_account(agent: AgentPubKey) -> ExternResult<Option<ArtistAccount>> {
    let account_path = Path::from(format!("artist_account/{}", agent));
    let typed_path = account_path.typed(LinkTypes::AgentToArtistAccount)?;
    let filter = LinkTypeFilter::try_from(LinkTypes::AgentToArtistAccount)?;
    let links = get_links(
        LinkQuery::new(typed_path.path_entry_hash()?, filter),
        GetStrategy::default(),
    )?;

    for link in links.iter().rev() {
        if let Some(action_hash) = link.target.clone().into_action_hash() {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                let author_matches = record.action().author() == &agent;
                let account: Option<ArtistAccount> =
                    record.entry().to_app_option().map_err(|e| wasm_error!(e))?;
                if let Some(account) = account {
                    if author_matches && account.owner == agent {
                        return Ok(Some(account));
                    }
                }
            }
        }
    }

    Ok(None)
}

/// Record an unverified deposit claim. This never credits a balance.
#[hdk_extern]
pub fn record_deposit(input: RecordDepositInput) -> ExternResult<ActionHash> {
    require_consciousness(&civic_requirement_proposal(), "record_deposit")?;
    let my_agent = agent_info()?.agent_initial_pubkey;

    let deposit = Deposit {
        listener: my_agent.clone(),
        amount: input.amount,
        tx_hash: input.tx_hash,
        block_number: input.block_number,
        deposited_at: sys_time()?,
        verified: false, // Will be verified by oracle
    };

    let action_hash = create_entry(&EntryTypes::Deposit(deposit))?;

    // Link to agent
    let deposits_path = Path::from(format!("deposits/{}", my_agent));
    let deposits_hash = ensure_path(deposits_path, LinkTypes::AgentToDeposits)?;
    create_link(
        deposits_hash,
        action_hash.clone(),
        LinkTypes::AgentToDeposits,
        (),
    )?;

    // Balance is NOT updated here — deposit must be verified by a trusted oracle first.
    // Use verify_deposit() to confirm and credit the balance.

    Ok(action_hash)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RecordDepositInput {
    pub amount: u64,
    pub tx_hash: String,
    pub block_number: u64,
}

/// Deposit verification is fail-closed until oracle authorization, unique
/// transaction consumption, and atomic balance crediting are enforced.
#[hdk_extern]
pub fn verify_deposit(_deposit_hash: ActionHash) -> ExternResult<ActionHash> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Deposit verification unavailable: an authenticated oracle policy and idempotent credit protocol are required"
            .to_string()
    )))
}

/// Request a cashout (artist)
#[hdk_extern]
pub fn request_cashout(_amount: u64) -> ExternResult<ActionHash> {
    Err(wasm_error!(WasmErrorInner::Guest(
        "Cashout unavailable: balance reservation, processor authorization, and idempotent completion are not active"
            .to_string()
    )))
}

/// Execute transfer from listener to artist (internal, called by plays zome)
#[hdk_extern]
pub fn execute_transfer(input: ExecuteTransferInput) -> ExternResult<ActionHash> {
    let caller = agent_info()?.agent_initial_pubkey;
    if input.from != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Transfer sender must match the authenticated caller".to_string()
        )));
    }
    Err(wasm_error!(WasmErrorInner::Guest(
        "Transfers unavailable: atomic debit, credit, replay protection, and settlement authorization are not active"
            .to_string()
    )))
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ExecuteTransferInput {
    pub from: AgentPubKey,
    pub to: AgentPubKey,
    pub amount: u64,
    pub reason: TransferReason,
    pub reference: Option<ActionHash>,
}

/// Get my listener account balance
#[hdk_extern]
pub fn get_my_listener_balance(_: ()) -> ExternResult<Option<ListenerAccount>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    get_listener_account(my_agent)
}

/// Get my artist account balance
#[hdk_extern]
pub fn get_my_artist_balance(_: ()) -> ExternResult<Option<ArtistAccount>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    get_artist_account(my_agent)
}

/// Get my cashout history
#[hdk_extern]
pub fn get_my_cashouts(_: ()) -> ExternResult<Vec<CashoutRequest>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let cashouts_path = Path::from(format!("cashouts/{}", my_agent));
    let typed_path = cashouts_path.typed(LinkTypes::AgentToCashouts)?;
    let filter = LinkTypeFilter::try_from(LinkTypes::AgentToCashouts)?;
    let links = get_links(
        LinkQuery::new(typed_path.path_entry_hash()?, filter),
        GetStrategy::default(),
    )?;

    let mut cashouts = Vec::new();
    for link in links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                if let Some(cashout) = record
                    .entry()
                    .to_app_option::<CashoutRequest>()
                    .map_err(|e| wasm_error!(e))?
                {
                    if record.action().author() == &my_agent && cashout.artist == my_agent {
                        cashouts.push(cashout);
                    }
                }
            }
        }
    }

    Ok(cashouts)
}

/// Get my transfer history
#[hdk_extern]
pub fn get_my_transfers(_: ()) -> ExternResult<Vec<Transfer>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let transfers_path = Path::from(format!("transfers/{}", my_agent));
    let typed_path = transfers_path.typed(LinkTypes::AgentToTransfers)?;
    let filter = LinkTypeFilter::try_from(LinkTypes::AgentToTransfers)?;
    let links = get_links(
        LinkQuery::new(typed_path.path_entry_hash()?, filter),
        GetStrategy::default(),
    )?;

    let mut transfers = Vec::new();
    for link in links {
        if let Some(action_hash) = link.target.into_action_hash() {
            if let Some(record) = get(action_hash, GetOptions::default())? {
                if let Some(transfer) = record
                    .entry()
                    .to_app_option::<Transfer>()
                    .map_err(|e| wasm_error!(e))?
                {
                    if record.action().author() == &transfer.from
                        && (transfer.from == my_agent || transfer.to == my_agent)
                    {
                        transfers.push(transfer);
                    }
                }
            }
        }
    }

    Ok(transfers)
}
