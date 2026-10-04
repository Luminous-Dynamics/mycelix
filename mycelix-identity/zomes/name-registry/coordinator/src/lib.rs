// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdk::prelude::*;
use mycelix_bridge_common::{
    GovernanceEligibility, civic_requirement_basic, civic_requirement_voting,
};
use name_registry_integrity::*;

use mycelix_zome_helpers as _;
/// Helper to get an anchor entry hash
fn anchor_hash(anchor_str: &str) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))
}

/// Helper to ensure an anchor entry exists and return its hash
fn ensure_anchor(anchor_str: &str) -> ExternResult<EntryHash> {
    create_entry(&EntryTypes::Anchor(Anchor(anchor_str.to_string())))?;
    anchor_hash(anchor_str)
}

/// Register a mesh name (Participant+).
#[hdk_extern]
pub fn register_name(entry: MeshNameEntry) -> ExternResult<Record> {
    let _eligibility = mycelix_zome_helpers::require_civic(
        "identity_bridge",
        &civic_requirement_basic(),
        "register_name",
    )?;

    let action_hash = create_entry(&EntryTypes::MeshNameEntry(entry.clone()))?;
    let agent = agent_info()?.agent_initial_pubkey;

    // Create anchor for name resolution
    let anchor_str = format!("mesh_name/{}", entry.segments.join("/"));
    let name_anchor = ensure_anchor(&anchor_str)?;
    create_link(name_anchor, action_hash.clone(), LinkTypes::NamePath, ())?;

    // Link from agent
    create_link(agent, action_hash.clone(), LinkTypes::AgentToNames, ())?;

    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Record not found".into())
    ))?;
    Ok(record)
}

/// Resolve a mesh name.
///
/// NamePath is an append-only index and its DHT traversal order is not an
/// authority clock. Return exactly one currently-unexpired registration; when
/// more than one valid registration claims the name, fail closed instead of
/// arbitrarily selecting whichever link the network returned first.
#[hdk_extern]
pub fn resolve_name(canonical: String) -> ExternResult<Option<MeshNameEntry>> {
    let segments: Vec<&str> = canonical
        .strip_prefix("mycelix://")
        .unwrap_or(&canonical)
        .trim_matches('/')
        .split('/')
        .filter(|segment| !segment.is_empty())
        .collect();

    if segments.is_empty() || segments.len() > 5 {
        return Ok(None);
    }

    let expected_canonical = format!("mycelix://{}", segments.join("/"));
    if expected_canonical != canonical.trim_end_matches('/') {
        return Ok(None);
    }

    let anchor_str = format!("mesh_name/{}", segments.join("/"));
    let links = get_links(
        LinkQuery::try_new(anchor_hash(&anchor_str)?, LinkTypes::NamePath)?,
        GetStrategy::default(),
    )?;

    let now = sys_time()?;
    let mut candidates = Vec::new();
    for link in links {
        if let Some(target) = link.target.into_action_hash() {
            if let Some(record) = get(target, GetOptions::default())? {
                if let Some(entry) = record
                    .entry()
                    .to_app_option::<MeshNameEntry>()
                    .ok()
                    .flatten()
                {
                    if entry.canonical == expected_canonical && entry.expires_at > now.as_micros() as u64 {
                        candidates.push(entry);
                    }
                }
            }
        }
    }

    match candidates.len() {
        0 => Ok(None),
        1 => Ok(candidates.into_iter().next()),
        _ => Err(wasm_error!(WasmErrorInner::Guest(
            "Ambiguous active mesh-name registrations; refusing nondeterministic resolution".into(),
        ))),
    }
}

fn current_name_owner(
    name_hash: &ActionHash,
) -> ExternResult<(AgentPubKey, Option<ActionHash>, MeshNameEntry)> {
    let name_record = get(name_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Name not found".into())))?;
    let name = name_record
        .entry()
        .to_app_option::<MeshNameEntry>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Name hash does not reference a MeshNameEntry".into(),
        )))?;

    let links = get_links(
        LinkQuery::try_new(name_hash.clone(), LinkTypes::NameToTransfers)?,
        GetStrategy::default(),
    )?;

    let mut transfers: Vec<(ActionHash, Record, NameTransfer)> = Vec::new();
    for link in links {
        let hash = link
            .target
            .into_action_hash()
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "NameToTransfers target must be an ActionHash".into(),
            )))?;
        if let Some(record) = get(hash.clone(), GetOptions::default())? {
            if let Some(transfer) = record
                .entry()
                .to_app_option::<NameTransfer>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if transfer.name_hash == *name_hash {
                    transfers.push((hash, record, transfer));
                }
            }
        }
    }

    if transfers.is_empty() {
        return Ok((name_record.action().author().clone(), None, name));
    }

    let child_hashes: std::collections::HashSet<ActionHash> = transfers
        .iter()
        .filter_map(|(_, _, transfer)| transfer.previous_transfer_hash.clone())
        .collect();

    let tips: Vec<_> = transfers
        .iter()
        .filter(|(hash, _, _)| !child_hashes.contains(hash))
        .collect();

    if tips.len() != 1 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Ambiguous name ownership history; refusing nondeterministic transfer resolution".into(),
        )));
    }

    let (tip_hash, tip_record, tip) = tips[0];

    // Walk the explicit ownership chain back to the original registration.
    // This verifies that every successor was authorized by the owner established
    // by its predecessor rather than merely trusting a root-index link.
    let mut seen = std::collections::HashSet::new();
    let mut current_hash = tip_hash.clone();
    let mut current_record = tip_record.clone();
    let mut current = tip.clone();

    loop {
        if !seen.insert(current_hash.clone()) {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Cyclic name ownership history".into(),
            )));
        }

        match current.previous_transfer_hash.clone() {
            None => {
                if *current_record.action().author() != *name_record.action().author() {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "First transfer is not authorized by the original name owner".into(),
                    )));
                }
                break;
            }
            Some(previous_hash) => {
                let previous_record = get(previous_hash.clone(), GetOptions::default())?
                    .ok_or(wasm_error!(WasmErrorInner::Guest(
                        "Previous name transfer not found".into(),
                    )))?;
                let previous = previous_record
                    .entry()
                    .to_app_option::<NameTransfer>()
                    .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                    .ok_or(wasm_error!(WasmErrorInner::Guest(
                        "Previous transfer hash does not reference a NameTransfer".into(),
                    )))?;
                if previous.name_hash != *name_hash
                    || previous.new_owner != *current_record.action().author()
                {
                    return Err(wasm_error!(WasmErrorInner::Guest(
                        "Name ownership chain contains an unauthorized successor".into(),
                    )));
                }
                current_hash = previous_hash;
                current_record = previous_record;
                current = previous;
            }
        }
    }

    Ok((tip.new_owner.clone(), Some(tip_hash.clone()), name))
}

/// Transfer name ownership (owner only, Citizen+).
#[hdk_extern]
pub fn transfer_name(transfer: NameTransfer) -> ExternResult<Record> {
    let _eligibility = mycelix_zome_helpers::require_civic(
        "identity_bridge",
        &civic_requirement_voting(),
        "transfer_name",
    )?;

    let caller = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?;
    let (owner, previous_transfer_hash, name) = current_name_owner(&transfer.name_hash)?;

    if name.expires_at <= now.as_micros() as u64 {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Expired names cannot be transferred".into()
        )));
    }
    if owner != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only the current owner can transfer".into()
        )));
    }
    if transfer.new_owner == caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Name transfer must specify a different owner".into()
        )));
    }

    // The previous state is derived from canonical ownership history rather
    // than trusted from caller input. This makes transfer succession ergonomic
    // while preserving an explicit, auditable chain for integrity validation.
    let transfer = NameTransfer {
        name_hash: transfer.name_hash,
        previous_transfer_hash,
        new_owner: transfer.new_owner,
        timestamp_us: now.as_micros() as u64,
    };

    let action_hash = create_entry(&EntryTypes::NameTransfer(transfer.clone()))?;
    create_link(
        transfer.name_hash,
        action_hash.clone(),
        LinkTypes::NameToTransfers,
        (),
    )?;

    // Link new owner
    create_link(
        transfer.new_owner,
        action_hash.clone(),
        LinkTypes::AgentToNames,
        (),
    )?;

    let record = get(action_hash, GetOptions::default())?.ok_or(wasm_error!(
        WasmErrorInner::Guest("Record not found".into())
    ))?;
    Ok(record)
}

/// Renew a name (extend expiry by 1 year). Owner only.
#[hdk_extern]
pub fn renew_name(name_hash: ActionHash) -> ExternResult<Record> {
    let _eligibility = mycelix_zome_helpers::require_civic(
        "identity_bridge",
        &civic_requirement_basic(),
        "renew_name",
    )?;

    let record = get(name_hash.clone(), GetOptions::default())?
        .ok_or(wasm_error!(WasmErrorInner::Guest("Name not found".into())))?;
    let owner = record.action().author().clone();
    let caller = agent_info()?.agent_initial_pubkey;
    if owner != caller {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Only owner can renew".into()
        )));
    }

    if let Some(mut entry) = record
        .entry()
        .to_app_option::<MeshNameEntry>()
        .ok()
        .flatten()
    {
        let one_year_us = 365 * 24 * 3600 * 1_000_000u64;
        entry.expires_at = entry.expires_at.saturating_add(one_year_us);
        let new_hash = update_entry(name_hash, &entry)?;
        let updated = get(new_hash, GetOptions::default())?.ok_or(wasm_error!(
            WasmErrorInner::Guest("Updated record not found".into())
        ))?;
        Ok(updated)
    } else {
        Err(wasm_error!(WasmErrorInner::Guest("Invalid entry".into())))
    }
}
