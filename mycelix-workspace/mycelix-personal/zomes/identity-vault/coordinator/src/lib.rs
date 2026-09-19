// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Identity Vault Coordinator Zome
//!
//! CRUD operations for the agent's private identity data.
//! All entries are stored on the source chain (private by default).

use hdk::prelude::*;
use identity_vault_integrity::*;

getrandom::register_custom_getrandom!(my_custom_getrandom);

pub fn my_custom_getrandom(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    let bytes = random_bytes(buf.len() as u32).map_err(|_| getrandom::Error::UNSUPPORTED)?;
    buf.copy_from_slice(bytes.as_ref());
    Ok(())
}

use mycelix_zkp_core::consciousness::{CivicTier, verify_consciousness_tier};
use personal_leptos_types::{
    MasterKeyView, MutationReceiptView, ProfileEvidenceView, ProfileView,
};

#[derive(Serialize, Deserialize, Debug)]
pub struct SubmitTierProofInput {
    pub tier: String,
    pub proof_bytes: Vec<u8>,
    pub score_commitment: [u8; 32],
    pub proof_epoch_secs: u64,
}

/// Submit a ZKP proof for tier membership.
/// Verifies the STARK proof on-chain (using backend-winterfell).
#[hdk_extern]
pub fn submit_tier_proof(input: SubmitTierProofInput) -> ExternResult<ActionHash> {
    let civic_tier = match input.tier.as_str() {
        "Participant" => CivicTier::Participant,
        "Citizen" => CivicTier::Citizen,
        "Steward" => CivicTier::Steward,
        "Guardian" => CivicTier::Guardian,
        _ => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Unsupported tier".into()
            )));
        }
    };

    let current_time_secs = sys_time()?.as_micros() / 1_000_000;

    // ON-CHAIN STARK VERIFICATION (Vector 3)
    let is_valid = verify_consciousness_tier(
        &input.proof_bytes,
        &civic_tier,
        &input.score_commitment,
        input.proof_epoch_secs,
        current_time_secs as u64,
    )
    .map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "ZKP Verification Error: {:?}",
            e
        )))
    })?;

    if !is_valid {
        return Err(wasm_error!(WasmErrorInner::Guest(
            "Invalid ZKP proof for tier".into()
        )));
    }

    // Proof is valid, commit the membership record
    let entry = TierMembershipProof {
        tier: input.tier,
        proof_bytes: input.proof_bytes,
        committed_at: sys_time()?,
    };

    let action_hash = create_entry(&EntryTypes::TierMembershipProof(entry))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToProof, ())?;

    Ok(action_hash)
}

/// Create or update the agent's profile.
///
/// Stores the profile on the source chain and creates a link from the
/// agent's pubkey for retrieval. If a profile already exists, it is updated.
#[hdk_extern]
pub fn set_profile(profile: Profile) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::Profile(profile.clone()))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToProfile, ())?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created profile".into()
    )))
}

#[hdk_extern]
pub fn set_profile_view(profile: ProfileView) -> ExternResult<MutationReceiptView> {
    let record = set_profile(Profile {
        display_name: profile.display_name,
        avatar: profile.avatar,
        bio: profile.bio,
        metadata: profile.metadata,
        updated_at: sys_time()?,
        mineralized_at: None,
    })?;
    Ok(MutationReceiptView {
        action_hash: record.action_address().to_string(),
    })
}

fn should_replace_action_seq(current: Option<u32>, candidate: u32) -> bool {
    current.map(|seq| candidate > seq).unwrap_or(true)
}

fn latest_linked_record(links: Vec<Link>) -> ExternResult<Option<Record>> {
    let mut latest: Option<Record> = None;

    for link in links {
        let target = ActionHash::try_from(link.target).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        let Some(record) = get(target, GetOptions::default())? else {
            continue;
        };

        let current_seq = latest.as_ref().map(|record| record.action().action_seq());
        let candidate_seq = record.action().action_seq();
        if should_replace_action_seq(current_seq, candidate_seq) {
            latest = Some(record);
        }
    }

    Ok(latest)
}

/// Get the agent's current profile.
///
/// `get_links` is treated as an unordered candidate set. Current state is
/// selected from self-authored links by the target Record's source-chain action
/// sequence, not by vector position.
#[hdk_extern]
pub fn get_my_profile(_: ()) -> ExternResult<Option<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent.clone(), LinkTypes::AgentToProfile)?.author(agent),
        GetStrategy::Local,
    )?;
    latest_linked_record(links)
}

fn profile_view_from_record(record: &Record) -> ExternResult<ProfileView> {
    let profile: Profile = record
        .entry()
        .to_app_option()
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        .ok_or(wasm_error!(WasmErrorInner::Guest(
            "Invalid profile entry".into()
        )))?;

    Ok(ProfileView {
        display_name: profile.display_name,
        avatar: profile.avatar,
        bio: profile.bio,
        metadata: profile.metadata,
        updated_at: profile.updated_at.as_micros(),
    })
}

#[hdk_extern]
pub fn get_my_profile_view(_: ()) -> ExternResult<Option<ProfileView>> {
    get_my_profile(())?
        .as_ref()
        .map(profile_view_from_record)
        .transpose()
}

/// Get the current profile together with the exact source-chain action that
/// produced the returned read-model value.
///
/// This is additive to `get_my_profile_view`: existing callers retain the old
/// payload contract while evidence-aware callers can correlate a mutation
/// receipt with the action actually observed by the read model.
#[hdk_extern]
pub fn get_my_profile_evidence_view(_: ()) -> ExternResult<Option<ProfileEvidenceView>> {
    get_my_profile(())?
        .map(|record| {
            let profile = profile_view_from_record(&record)?;
            Ok(ProfileEvidenceView {
                action_hash: record.action_address().to_string(),
                profile,
            })
        })
        .transpose()
}

/// Register a master key for this agent.
#[hdk_extern]
pub fn register_key(key: MasterKey) -> ExternResult<Record> {
    let action_hash = create_entry(&EntryTypes::MasterKey(key))?;
    let agent = agent_info()?.agent_initial_pubkey;
    create_link(agent, action_hash.clone(), LinkTypes::AgentToKeys, ())?;
    get(action_hash, GetOptions::default())?.ok_or(wasm_error!(WasmErrorInner::Guest(
        "Could not retrieve created key".into()
    )))
}

/// List all registered keys for this agent.
#[hdk_extern]
pub fn get_my_keys(_: ()) -> ExternResult<Vec<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent.clone(), LinkTypes::AgentToKeys)?.author(agent),
        GetStrategy::Local,
    )?;
    let mut records = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        if let Some(record) = get(target, GetOptions::default())? {
            records.push(record);
        }
    }
    Ok(records)
}

#[hdk_extern]
pub fn get_my_keys_view(_: ()) -> ExternResult<Vec<MasterKeyView>> {
    let records = get_my_keys(())?;
    let mut keys = Vec::new();
    for record in records {
        if let Some(key) = record
            .entry()
            .to_app_option::<MasterKey>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
        {
            keys.push(MasterKeyView {
                label: key.label,
                purpose: key.purpose,
                public_key_hex: key.public_key_hex,
                active: key.active,
                created_at: key.created_at.as_micros(),
            });
        }
    }
    Ok(keys)
}

/// Selective disclosure: return profile fields filtered by requested scope.
///
/// Used by personal_bridge to fulfill cross-cluster identity queries
/// without revealing the full profile.
#[hdk_extern]
pub fn disclose_profile(fields: Vec<String>) -> ExternResult<String> {
    let profile_record = get_my_profile(())?;
    let profile = match profile_record {
        Some(record) => {
            let p: Profile = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Invalid profile entry".into()
                )))?;
            p
        }
        None => return Ok("{}".into()),
    };

    let mut disclosed = serde_json::Map::new();
    for field in &fields {
        match field.as_str() {
            "display_name" => {
                disclosed.insert(
                    "display_name".into(),
                    serde_json::Value::String(profile.display_name.clone()),
                );
            }
            "bio" => {
                if let Some(ref bio) = profile.bio {
                    disclosed.insert("bio".into(), serde_json::Value::String(bio.clone()));
                }
            }
            "avatar" => {
                if let Some(ref avatar) = profile.avatar {
                    disclosed.insert("avatar".into(), serde_json::Value::String(avatar.clone()));
                }
            }
            _ => {}
        }
    }

    serde_json::to_string(&disclosed)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Serialization error: {}", e))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_entry_type_exists() {
        let _variant = UnitEntryTypes::Profile;
    }

    #[test]
    fn master_key_entry_type_exists() {
        let _variant = UnitEntryTypes::MasterKey;
    }

    #[test]
    fn link_types_exist() {
        let _profile = LinkTypes::AgentToProfile;
        let _keys = LinkTypes::AgentToKeys;
    }

    #[test]
    fn source_chain_sequence_is_the_current_record_order() {
        assert!(should_replace_action_seq(None, 4));
        assert!(should_replace_action_seq(Some(4), 5));
        assert!(!should_replace_action_seq(Some(5), 5));
        assert!(!should_replace_action_seq(Some(6), 5));
    }
}
