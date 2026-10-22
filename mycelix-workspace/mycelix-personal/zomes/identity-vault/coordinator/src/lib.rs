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
use personal_leptos_types::{MasterKeyView, ProfileView};

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
pub fn set_profile_view(profile: ProfileView) -> ExternResult<Record> {
    set_profile(Profile {
        display_name: profile.display_name,
        avatar: profile.avatar,
        bio: profile.bio,
        metadata: profile.metadata,
        updated_at: sys_time()?,
        mineralized_at: None,
    })
}

/// Get the agent's current profile.
#[hdk_extern]
pub fn get_my_profile(_: ()) -> ExternResult<Option<Record>> {
    let agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToProfile)?,
        GetStrategy::Local,
    )?;
    if let Some(link) = links.last() {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
            wasm_error!(WasmErrorInner::Guest(format!(
                "Invalid link target: {:?}",
                e
            )))
        })?;
        get(target, GetOptions::default())
    } else {
        Ok(None)
    }
}

#[hdk_extern]
pub fn get_my_profile_view(_: ()) -> ExternResult<Option<ProfileView>> {
    let profile_record = get_my_profile(())?;
    match profile_record {
        Some(record) => {
            let profile: Profile = record
                .entry()
                .to_app_option()
                .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
                .ok_or(wasm_error!(WasmErrorInner::Guest(
                    "Invalid profile entry".into()
                )))?;

            Ok(Some(ProfileView {
                display_name: profile.display_name,
                avatar: profile.avatar,
                bio: profile.bio,
                metadata: profile.metadata,
                updated_at: profile.updated_at.as_micros(),
            }))
        }
        None => Ok(None),
    }
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
        LinkQuery::try_new(agent, LinkTypes::AgentToKeys)?,
        GetStrategy::Local,
    )?;
    let mut records = Vec::new();
    for link in links {
        let target = ActionHash::try_from(link.target.clone()).map_err(|e| {
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

const ALLOWED_PROFILE_DISCLOSURE_FIELDS: &[&str] = &["display_name", "bio", "avatar"];

/// Project only an explicit, validated field set. Unknown, duplicate, and empty
/// scopes are rejected rather than silently broadened or collapsed to an empty object.
fn project_profile_fields(
    profile: &Profile,
    fields: &[String],
) -> Result<serde_json::Map<String, serde_json::Value>, &'static str> {
    if fields.is_empty() {
        return Err("Identity disclosure scope must contain at least one field");
    }

    for (index, field) in fields.iter().enumerate() {
        if !ALLOWED_PROFILE_DISCLOSURE_FIELDS.contains(&field.as_str()) {
            return Err("Identity disclosure scope contains an unsupported field");
        }
        if fields[..index].iter().any(|prior| prior == field) {
            return Err("Identity disclosure scope contains a duplicate field");
        }
    }

    let mut disclosed = serde_json::Map::new();
    for field in fields {
        match field.as_str() {
            "display_name" => {
                disclosed.insert(
                    "display_name".into(),
                    serde_json::Value::String(profile.display_name.clone()),
                );
            }
            "bio" => {
                disclosed.insert(
                    "bio".into(),
                    profile
                        .bio
                        .clone()
                        .map(serde_json::Value::String)
                        .unwrap_or(serde_json::Value::Null),
                );
            }
            "avatar" => {
                disclosed.insert(
                    "avatar".into(),
                    profile
                        .avatar
                        .clone()
                        .map(serde_json::Value::String)
                        .unwrap_or(serde_json::Value::Null),
                );
            }
            _ => return Err("Identity disclosure scope contains an unsupported field"),
        }
    }
    Ok(disclosed)
}

/// Legacy field projection only; this does not produce a cryptographic proof,
/// bind a verifier/nonce, or establish permission to disclose.
///
/// Optional requested fields are returned as JSON null when no value is stored,
/// keeping output shape dependent on the requested scope rather than profile
/// population. A missing profile is an error, not an empty JSON object.
#[hdk_extern]
pub fn disclose_profile(fields: Vec<String>) -> ExternResult<String> {
    let profile_record = get_my_profile(())?;
    let profile = match profile_record {
        Some(record) => record
            .entry()
            .to_app_option::<Profile>()
            .map_err(|e| wasm_error!(WasmErrorInner::Guest(e.to_string())))?
            .ok_or(wasm_error!(WasmErrorInner::Guest(
                "Invalid profile entry".into()
            )))?,
        None => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "No profile is available for identity disclosure".into()
            )));
        }
    };

    let disclosed = project_profile_fields(&profile, &fields)
        .map_err(|reason| wasm_error!(WasmErrorInner::Guest(reason.into())))?;

    serde_json::to_string(&disclosed)
        .map_err(|e| wasm_error!(WasmErrorInner::Guest(format!("Serialization error: {}", e))))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disclosure_scope_rejects_empty_field_sets() {
        let profile = Profile {
            display_name: "Ada".into(),
            avatar: None,
            bio: None,
            metadata: std::collections::HashMap::new(),
            updated_at: Timestamp::from_micros(1),
            mineralized_at: None,
        };
        assert_eq!(
            project_profile_fields(&profile, &[]).unwrap_err(),
            "Identity disclosure scope must contain at least one field"
        );
    }

    #[test]
    fn disclosure_scope_rejects_unknown_and_case_variant_fields() {
        let profile = Profile {
            display_name: "Ada".into(),
            avatar: None,
            bio: None,
            metadata: std::collections::HashMap::new(),
            updated_at: Timestamp::from_micros(1),
            mineralized_at: None,
        };
        for field in ["email", "metadata", "Display_Name"] {
            assert_eq!(
                project_profile_fields(&profile, &[field.into()]).unwrap_err(),
                "Identity disclosure scope contains an unsupported field"
            );
        }
    }

    #[test]
    fn disclosure_scope_rejects_duplicate_fields() {
        let profile = Profile {
            display_name: "Ada".into(),
            avatar: None,
            bio: None,
            metadata: std::collections::HashMap::new(),
            updated_at: Timestamp::from_micros(1),
            mineralized_at: None,
        };
        assert_eq!(
            project_profile_fields(&profile, &["bio".into(), "bio".into()]).unwrap_err(),
            "Identity disclosure scope contains a duplicate field"
        );
    }

    #[test]
    fn disclosure_projection_contains_only_requested_keys_and_stable_nulls() {
        let profile = Profile {
            display_name: "Ada".into(),
            avatar: None,
            bio: None,
            metadata: std::collections::HashMap::new(),
            updated_at: Timestamp::from_micros(1),
            mineralized_at: None,
        };
        let projected = project_profile_fields(&profile, &["bio".into(), "avatar".into()])
            .expect("known, distinct fields should be projected");
        assert_eq!(projected.len(), 2);
        assert_eq!(projected.get("bio"), Some(&serde_json::Value::Null));
        assert_eq!(projected.get("avatar"), Some(&serde_json::Value::Null));
        assert!(!projected.contains_key("display_name"));
        assert!(!projected.contains_key("metadata"));
    }

    #[test]
    fn disclosure_projection_returns_only_the_explicit_display_name() {
        let profile = Profile {
            display_name: "Ada".into(),
            avatar: Some("private-avatar".into()),
            bio: Some("private-bio".into()),
            metadata: std::collections::HashMap::new(),
            updated_at: Timestamp::from_micros(1),
            mineralized_at: None,
        };
        let projected = project_profile_fields(&profile, &["display_name".into()])
            .expect("known field should be projected");
        assert_eq!(projected.len(), 1);
        assert_eq!(
            projected.get("display_name"),
            Some(&serde_json::Value::String("Ada".into()))
        );
    }


    #[test]
    fn profile_entry_type_exists() {
        // Verify the entry type enum compiles and matches
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
}
