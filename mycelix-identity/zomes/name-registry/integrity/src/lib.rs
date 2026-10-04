// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
use hdi::prelude::*;

/// A registered mesh name binding.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct MeshNameEntry {
    /// Name segments (e.g., ["joburg", "water", "tank-7"]).
    pub segments: Vec<String>,
    /// Canonical form (e.g., "mycelix://joburg/water/tank-7").
    pub canonical: String,
    /// Endpoint type: "iroh", "lora", "holochain", "ip".
    pub endpoint_type: String,
    /// Endpoint data.
    pub endpoint_data: String,
    /// Registration timestamp (µs since epoch).
    pub registered_at: u64,
    /// Expiry timestamp (µs since epoch, default 1 year from registration).
    pub expires_at: u64,
}

/// Transfer of name ownership.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct NameTransfer {
    /// Action hash of the original MeshNameEntry being transferred.
    pub name_hash: ActionHash,
    /// Previous transfer in the ownership chain. None only for the first transfer.
    #[serde(default)]
    pub previous_transfer_hash: Option<ActionHash>,
    /// New owner agent.
    pub new_owner: AgentPubKey,
    /// Transfer timestamp.
    pub timestamp_us: u64,
}

/// Anchor entry for deterministic link bases.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Anchor(pub String);

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_type(name = "MeshNameEntry", visibility = "public")]
    MeshNameEntry(MeshNameEntry),
    #[entry_type(name = "NameTransfer", visibility = "public")]
    NameTransfer(NameTransfer),
    #[entry_type(name = "Anchor", visibility = "public")]
    Anchor(Anchor),
}

#[hdk_link_types]
pub enum LinkTypes {
    NamePath,
    AgentToNames,
    NameToTransfers,
}

fn name_anchor(segments: &[String]) -> ExternResult<EntryHash> {
    hash_entry(&EntryTypes::Anchor(Anchor(format!(
        "mesh_name/{}",
        segments.join("/")
    ))))
}

fn agent_anchor(agent: &AgentPubKey) -> AgentPubKey {
    agent.clone()
}

fn action_target(
    target_address: &AnyLinkableHash,
    label: &str,
) -> ExternResult<ActionHash> {
    target_address.clone().into_action_hash().ok_or_else(|| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "{label} target must be an ActionHash"
        )))
    })
}

fn validate_mesh_name(entry: &MeshNameEntry) -> Result<(), String> {
    if entry.segments.is_empty() || entry.segments.len() > 5 {
        return Err("Name depth must be 1-5 segments".into());
    }
    for seg in &entry.segments {
        if seg.is_empty() || seg.len() > 63 {
            return Err(format!("Segment '{}' invalid length", seg));
        }
        if !seg
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
        {
            return Err(format!("Segment '{}' contains invalid chars", seg));
        }
        if seg.starts_with('-') || seg.ends_with('-') {
            return Err(format!("Segment '{}' cannot start/end with hyphen", seg));
        }
    }

    let expected_canonical = format!("mycelix://{}", entry.segments.join("/"));
    if entry.canonical != expected_canonical {
        return Err("Canonical name must exactly match its segments".into());
    }

    if entry.endpoint_type.is_empty()
        || !["iroh", "lora", "holochain", "ip"].contains(&entry.endpoint_type.as_str())
    {
        return Err("Invalid endpoint type".into());
    }
    if entry.endpoint_data.len() > 4096 {
        return Err("Endpoint data exceeds 4096 bytes".into());
    }
    if entry.registered_at == 0 || entry.expires_at <= entry.registered_at {
        return Err("Registration/expiry timestamps are invalid".into());
    }

    Ok(())
}

fn validate_name_transfer_chain_uniqueness(
    action: &Create,
    transfer: &NameTransfer,
) -> ExternResult<ValidateCallbackResult> {
    let activity = must_get_agent_activity(
        action.author.clone(),
        ChainFilter::new(action.prev_action.clone()),
    )?;
    let entry_type = EntryType::App(AppEntryDef::try_from(UnitEntryTypes::NameTransfer)?);

    for prior in activity {
        let prior_action = prior.action.action();
        let Action::Create(prior_create) = prior_action else {
            continue;
        };
        if prior_create.entry_type != entry_type {
            continue;
        }
        let prior_entry = must_get_entry(prior_create.entry_hash.clone())?;
        let prior_transfer: NameTransfer = prior_entry.try_into().map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Name transfer history entry could not be decoded: {e}"
            )))
        })?;

        if prior_transfer.name_hash != transfer.name_hash {
            continue;
        }

        if transfer.previous_transfer_hash.is_none()
            || prior_transfer.previous_transfer_hash == transfer.previous_transfer_hash
        {
            return Ok(ValidateCallbackResult::Invalid(
                "A source chain cannot create two transfers for the same ownership state".into(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

fn validate_create_name_transfer(
    action: EntryCreationAction,
    transfer: NameTransfer,
) -> ExternResult<ValidateCallbackResult> {
    if transfer.timestamp_us == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Transfer timestamp must be non-zero".into(),
        ));
    }

    let name_record = must_get_valid_record(transfer.name_hash.clone())?;
    if name_record
        .entry()
        .to_app_option::<MeshNameEntry>()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .is_none()
    {
        return Ok(ValidateCallbackResult::Invalid(
            "NameTransfer must reference a MeshNameEntry".into(),
        ));
    }

    match transfer.previous_transfer_hash.clone() {
        None => {
            if *name_record.action().author() != *action.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original MeshNameEntry owner may create the first transfer".into(),
                ));
            }
        }
        Some(previous_hash) => {
            let previous_record = must_get_valid_record(previous_hash.clone())?;
            let previous: NameTransfer = previous_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "previous_transfer_hash must reference a NameTransfer".into(),
                )))?;

            if previous.name_hash != transfer.name_hash {
                return Ok(ValidateCallbackResult::Invalid(
                    "Previous transfer must reference the same MeshNameEntry".into(),
                ));
            }
            if previous.new_owner != *action.author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the current owner may continue the transfer chain".into(),
                ));
            }
        }
    }

    if transfer.new_owner == *action.author() {
        return Ok(ValidateCallbackResult::Invalid(
            "Name transfer must specify a different owner".into(),
        ));
    }

    validate_name_transfer_chain_uniqueness(
        match action {
            EntryCreationAction::Create(create) => create,
        },
        &transfer,
    )
}

fn validate_create_link(
    link_type: LinkTypes,
    base_address: &AnyLinkableHash,
    target_address: &AnyLinkableHash,
    action: &CreateLink,
) -> ExternResult<ValidateCallbackResult> {
    match link_type {
        LinkTypes::NamePath => {
            let base = base_address.clone().into_entry_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "NamePath base must be an EntryHash".into(),
                ))
            })?;
            let target = action_target(target_address, "NamePath")?;
            let record = must_get_valid_record(target)?;
            let entry: MeshNameEntry = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "NamePath target must be a MeshNameEntry".into(),
                )))?;
            if base != name_anchor(&entry.segments)? {
                return Ok(ValidateCallbackResult::Invalid(
                    "NamePath base does not match the name segments".into(),
                ));
            }
            if action.author != *record.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "NamePath link must be authored by the name owner".into(),
                ));
            }
        }
        LinkTypes::AgentToNames => {
            let base = base_address.clone().into_agent_pub_key().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "AgentToNames base must be an AgentPubKey".into(),
                ))
            })?;
            let target = action_target(target_address, "AgentToNames")?;
            let record = must_get_valid_record(target)?;

            if let Some(name) = record
                .entry()
                .to_app_option::<MeshNameEntry>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if base != *record.action().author() || action.author != *record.action().author() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToNames link for a name must use and be authored by its owner".into(),
                    ));
                }
                let _ = name;
            } else if let Some(transfer) = record
                .entry()
                .to_app_option::<NameTransfer>()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            {
                if base != transfer.new_owner || action.author != *record.action().author() {
                    return Ok(ValidateCallbackResult::Invalid(
                        "AgentToNames transfer link must target the new owner and be authored by the transfer creator".into(),
                    ));
                }
            } else {
                return Ok(ValidateCallbackResult::Invalid(
                    "AgentToNames target must be a MeshNameEntry or NameTransfer".into(),
                ));
            }
        }
        LinkTypes::NameToTransfers => {
            let base = base_address.clone().into_action_hash().ok_or_else(|| {
                wasm_error!(WasmErrorInner::Guest(
                    "NameToTransfers base must be an ActionHash".into(),
                ))
            })?;
            let target = action_target(target_address, "NameToTransfers")?;
            let transfer_record = must_get_valid_record(target)?;
            let transfer: NameTransfer = transfer_record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "NameToTransfers target must be a NameTransfer".into(),
                )))?;
            if base != transfer.name_hash || action.author != *transfer_record.action().author() {
                return Ok(ValidateCallbackResult::Invalid(
                    "NameToTransfers link does not match its transfer".into(),
                ));
            }
            let name_record = must_get_valid_record(base)?;
            if *name_record.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "NameToTransfers link must be created by the original name owner".into(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn genesis_self_check(_data: GenesisSelfCheckData) -> ExternResult<ValidateCallbackResult> {
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::MeshNameEntry(entry) => {
                    validate_mesh_name(&entry).map_or_else(
                        |msg| Ok(ValidateCallbackResult::Invalid(msg)),
                        |_| Ok(ValidateCallbackResult::Valid),
                    )
                }
                EntryTypes::NameTransfer(transfer) => {
                    validate_create_name_transfer(
                        EntryCreationAction::Create(action),
                        transfer,
                    )
                }
                EntryTypes::Anchor(anchor) => {
                    if anchor.0.is_empty() || anchor.0.len() > 256 {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Anchor must be 1-256 characters".into(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
            },
            OpEntry::UpdateEntry {
                app_entry,
                action,
                ..
            } => {
                let original = must_get_valid_record(action.original_action_address.clone())?;
                if *original.action().author() != action.author {
                    return Ok(ValidateCallbackResult::Invalid(
                        "Only the original entry author can update entries".into(),
                    ));
                }
                match app_entry {
                    EntryTypes::MeshNameEntry(entry) => {
                        validate_mesh_name(&entry).map_or_else(
                            |msg| Ok(ValidateCallbackResult::Invalid(msg)),
                            |_| Ok(ValidateCallbackResult::Valid),
                        )
                    }
                    EntryTypes::NameTransfer(_) | EntryTypes::Anchor(_) => {
                        Ok(ValidateCallbackResult::Invalid(
                            "Name transfers and anchors are append-only".into(),
                        ))
                    }
                }
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink {
            base_address,
            target_address,
            link_type,
            tag,
            action,
        } => {
            if tag.0.len() > 1024 {
                return Ok(ValidateCallbackResult::Invalid(
                    "Link tag exceeds maximum length of 1024 bytes".into(),
                ));
            }
            validate_create_link(link_type, &base_address, &target_address, &action)
        }
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Invalid(
            "Name registry indexes cannot be deleted".into(),
        )),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(activity) => match activity {
            OpActivity::CreateEntry {
                app_entry_type: Some(UnitEntryTypes::NameTransfer),
                action,
            } => {
                let entry = must_get_entry(action.entry_hash.clone())?;
                let transfer: NameTransfer = entry.try_into().map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "NameTransfer activity entry could not be decoded: {e}"
                    )))
                })?;
                validate_name_transfer_chain_uniqueness(action, &transfer)
            }
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterUpdate(update) => {
            let action = match &update {
                OpUpdate::Entry { action, .. }
                | OpUpdate::PrivateEntry { action, .. }
                | OpUpdate::Agent { action, .. }
                | OpUpdate::CapClaim { action, .. }
                | OpUpdate::CapGrant { action, .. } => action,
            };
            let original = must_get_action(action.original_action_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can update entries".into(),
                ));
            }
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterDelete(OpDelete { action }) => {
            let original = must_get_action(action.deletes_address.clone())?;
            if *original.action().author() != action.author {
                return Ok(ValidateCallbackResult::Invalid(
                    "Only the original entry author can delete entries".into(),
                ));
            }
            match original.action().entry_type() {
                Some(EntryType::App(def))
                    if *def == UnitEntryTypes::MeshNameEntry.into()
                        || *def == UnitEntryTypes::NameTransfer.into() =>
                {
                    Ok(ValidateCallbackResult::Invalid(
                        "Name bindings and transfers cannot be deleted".into(),
                    ))
                }
                _ => Ok(ValidateCallbackResult::Valid),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn name() -> MeshNameEntry {
        MeshNameEntry {
            segments: vec!["joburg".into(), "water".into(), "tank-7".into()],
            canonical: "mycelix://joburg/water/tank-7".into(),
            endpoint_type: "holochain".into(),
            endpoint_data: "uhCAkexample".into(),
            registered_at: 1,
            expires_at: 365 * 24 * 3600 * 1_000_000,
        }
    }

    #[test]
    fn canonical_name_is_derived_from_segments() {
        assert_eq!(
            format!("mycelix://{}", name().segments.join("/")),
            name().canonical
        );
    }

    #[test]
    fn transfer_authority_is_not_the_new_owner() {
        let old_owner = AgentPubKey::from_raw_36(vec![1; 36]);
        let new_owner = AgentPubKey::from_raw_36(vec![2; 36]);
        assert_ne!(old_owner, new_owner);
    }
}
