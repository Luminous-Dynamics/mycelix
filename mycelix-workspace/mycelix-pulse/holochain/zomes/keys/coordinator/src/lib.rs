// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Keys Coordinator Zome
//!
//! Pre-key bundle management for end-to-end encryption in Mycelix Mail.
//!
//! # Overview
//!
//! This zome implements the Signal Protocol-style pre-key bundle system with
//! post-quantum cryptography support (Kyber + Dilithium). Key bundles enable
//! asynchronous message encryption when the recipient is offline.
//!
//! # Key Bundle Structure
//!
//! Each bundle contains:
//! - **Identity Key**: Long-term public key (Dilithium for post-quantum)
//! - **Signed Pre-Key**: Medium-term key signed by identity key
//! - **One-Time Pre-Keys**: Ephemeral keys for forward secrecy (Kyber)
//!
//! # Key Management Functions
//!
//! - [`publish_pre_key_bundle`] - Publish a new key bundle to the DHT
//! - [`get_pre_key_bundle`] - Retrieve an agent's current key bundle
//! - [`consume_pre_key`] - Use a one-time pre-key for encryption
//! - [`rotate_keys`] - Rotate keys with audit trail
//! - [`needs_refresh`] - Check if bundle needs refresh
//!
//! # Example Usage
//!
//! ```ignore
//! // Publish a pre-key bundle
//! let bundle = PreKeyBundle {
//!     identity_public_key: identity_key,
//!     signed_pre_key: signed_key,
//!     signed_pre_key_signature: signature,
//!     one_time_pre_keys: otpks,
//!     created_at: now,
//!     expires_at: now + 30_days,
//! };
//! let hash = publish_pre_key_bundle(bundle)?;
//!
//! // Get someone's bundle for encryption
//! let their_bundle = get_pre_key_bundle(recipient_agent)?;
//!
//! // Consume a one-time key
//! let otpk = consume_pre_key(ConsumePreKeyInput {
//!     bundle_hash: their_bundle_hash,
//!     key_id: 0,
//! })?;
//! ```
//!
//! # Security Considerations
//!
//! - One-time pre-keys provide forward secrecy
//! - Bundle expiration enforces key rotation
//! - Key rotation creates audit trail
//! - Post-quantum algorithms protect against future threats

use hdk::prelude::*;
use keys_integrity::*;

// ==================== BUNDLE MANAGEMENT ====================

/// Publish pre-key bundle
#[hdk_extern]
pub fn publish_pre_key_bundle(mut bundle: PreKeyBundle) -> ExternResult<ActionHash> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    bundle.signed_pre_key_signature = sign_raw(my_agent.clone(), pre_key_signing_content(&bundle))?
        .0
        .to_vec();
    let action_hash = create_entry(EntryTypes::PreKeyBundle(bundle))?;

    // Link from agent to bundle
    create_link(my_agent, action_hash.clone(), LinkTypes::AgentToBundle, ())?;

    Ok(action_hash)
}

/// Publish an immutable Pulse V2 bundle. The coordinator derives its key ID
/// and replaces any client-supplied signature with the active agent signature.
#[hdk_extern]
pub fn publish_hybrid_key_bundle_v2(mut bundle: HybridKeyBundleV2) -> ExternResult<ActionHash> {
    let agent = agent_info()?.agent_initial_pubkey;
    bundle.version = HYBRID_KEY_BUNDLE_V2;
    bundle.suite = HYBRID_SUITE_V2.to_string();
    bundle.key_id = hybrid_key_id(&bundle);
    bundle.agent_signature = sign_raw(agent.clone(), hybrid_key_signing_content(&bundle))?
        .0
        .to_vec();
    let action_hash = create_entry(EntryTypes::HybridKeyBundleV2(bundle))?;
    create_link(
        agent,
        action_hash.clone(),
        LinkTypes::AgentToBundle,
        LinkTag::new("hybrid-v2"),
    )?;
    Ok(action_hash)
}

/// Return the newest published V2 bundle. Callers still enforce state and
/// expiry for new sends rather than treating presence as capability.
#[hdk_extern]
pub fn get_hybrid_key_bundle_v2(agent: AgentPubKey) -> ExternResult<Option<HybridKeyBundleV2>> {
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToBundle)?.tag_prefix(LinkTag::new("hybrid-v2")),
        GetStrategy::default(),
    )?;
    let mut bundles = Vec::new();
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                if let Some(bundle) = record
                    .entry()
                    .to_app_option::<HybridKeyBundleV2>()
                    .map_err(|e| wasm_error!(e))?
                {
                    bundles.push(bundle);
                }
            }
        }
    }
    bundles.sort_by_key(|bundle| bundle.created_at);
    Ok(bundles.pop())
}

#[hdk_extern]
pub fn get_my_hybrid_key_bundle_v2(_: ()) -> ExternResult<Option<HybridKeyBundleV2>> {
    get_hybrid_key_bundle_v2(agent_info()?.agent_initial_pubkey)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct HybridKeyLookupV2 {
    pub agent: AgentPubKey,
    pub key_id: [u8; 32],
}

/// Resolve the exact historical sender key referenced by an envelope. Retired
/// keys remain verifiable; selecting keys for a new send still requires Active.
#[hdk_extern]
pub fn get_hybrid_key_bundle_by_id_v2(
    input: HybridKeyLookupV2,
) -> ExternResult<Option<HybridKeyBundleV2>> {
    let links = get_links(
        LinkQuery::try_new(input.agent, LinkTypes::AgentToBundle)?
            .tag_prefix(LinkTag::new("hybrid-v2")),
        GetStrategy::default(),
    )?;
    for link in links {
        if let Some(hash) = link.target.into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                if let Some(bundle) = record
                    .entry()
                    .to_app_option::<HybridKeyBundleV2>()
                    .map_err(|error| wasm_error!(error))?
                {
                    if bundle.key_id == input.key_id {
                        return Ok(Some(bundle));
                    }
                }
            }
        }
    }
    Ok(None)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct HybridSendContextV2 {
    pub sender_agent_raw: Vec<u8>,
    pub recipient_agent_raw: Vec<u8>,
    pub sender_bundle: HybridKeyBundleV2,
    pub recipient_bundle: HybridKeyBundleV2,
}

/// Resolve the exact raw agent bytes and active bundle needed to construct the
/// canonical V2 AAD. This avoids client-side assumptions about AgentPubKey
/// string encodings.
#[hdk_extern]
pub fn resolve_hybrid_send_context_v2(
    recipient: AgentPubKey,
) -> ExternResult<Option<HybridSendContextV2>> {
    let sender = agent_info()?.agent_initial_pubkey;
    let Some(sender_bundle) = get_hybrid_key_bundle_v2(sender.clone())? else {
        return Ok(None);
    };
    let Some(bundle) = get_hybrid_key_bundle_v2(recipient.clone())? else {
        return Ok(None);
    };
    if sender_bundle.state != HybridKeyStateV2::Active
        || sender_bundle.expires_at <= sys_time()?.as_micros() as u64
        || bundle.state != HybridKeyStateV2::Active
        || bundle.expires_at <= sys_time()?.as_micros() as u64
    {
        return Ok(None);
    }
    Ok(Some(HybridSendContextV2 {
        sender_agent_raw: sender.get_raw_39().to_vec(),
        recipient_agent_raw: recipient.get_raw_39().to_vec(),
        sender_bundle,
        recipient_bundle: bundle,
    }))
}

/// Get pre-key bundle for an agent
#[hdk_extern]
pub fn get_pre_key_bundle(agent: AgentPubKey) -> ExternResult<Option<PreKeyBundle>> {
    let links = get_links(
        LinkQuery::try_new(agent, LinkTypes::AgentToBundle)?,
        GetStrategy::default(),
    )?;

    // Get the most recent bundle
    let mut bundles: Vec<(u64, PreKeyBundle)> = Vec::new();
    let now = sys_time()?.as_micros() as u64;

    for link in links {
        if let Some(hash) = link.target.clone().into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                if let Some(bundle) = record
                    .entry()
                    .to_app_option::<PreKeyBundle>()
                    .map_err(|e| wasm_error!(e))?
                {
                    // Only include non-expired bundles
                    if bundle.expires_at > now {
                        bundles.push((bundle.created_at, bundle));
                    }
                }
            }
        }
    }

    // Return the most recent bundle
    bundles.sort_by(|a, b| b.0.cmp(&a.0));
    Ok(bundles.into_iter().next().map(|(_, b)| b))
}

/// Get my current bundle
#[hdk_extern]
pub fn get_my_bundle(_: ()) -> ExternResult<Option<PreKeyBundle>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    get_pre_key_bundle(my_agent)
}

/// Consume a one-time pre-key
#[hdk_extern]
pub fn consume_pre_key(input: ConsumePreKeyInput) -> ExternResult<Option<Vec<u8>>> {
    // Get the bundle
    if let Some(record) = get(input.bundle_hash.clone(), GetOptions::default())? {
        if let Some(mut bundle) = record
            .entry()
            .to_app_option::<PreKeyBundle>()
            .map_err(|e| wasm_error!(e))?
        {
            // Find the requested key
            for otpk in &mut bundle.one_time_pre_keys {
                if otpk.key_id == input.key_id && !otpk.used {
                    let key = otpk.public_key.clone();

                    // Mark as used
                    otpk.used = true;

                    // Update the bundle
                    update_entry(input.bundle_hash.clone(), EntryTypes::PreKeyBundle(bundle))?;

                    // Record the usage
                    let used = UsedPreKey {
                        key_id: input.key_id,
                        bundle_hash: input.bundle_hash,
                        used_at: sys_time()?.as_micros() as u64,
                        used_by: agent_info()?.agent_initial_pubkey,
                    };
                    create_entry(EntryTypes::UsedPreKey(used))?;

                    return Ok(Some(key));
                }
            }
        }
    }

    Ok(None)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConsumePreKeyInput {
    pub bundle_hash: ActionHash,
    pub key_id: u32,
}

/// Get available one-time pre-key count
#[hdk_extern]
pub fn get_available_pre_key_count(agent: AgentPubKey) -> ExternResult<u32> {
    if let Some(bundle) = get_pre_key_bundle(agent)? {
        let count = bundle.one_time_pre_keys.iter().filter(|k| !k.used).count();
        Ok(count as u32)
    } else {
        Ok(0)
    }
}

// ==================== KEY ROTATION ====================

/// Rotate keys (publish new bundle)
#[hdk_extern]
pub fn rotate_keys(input: RotateKeysInput) -> ExternResult<ActionHash> {
    // Get old bundle hash
    let my_agent = agent_info()?.agent_initial_pubkey;
    let old_bundle_hash = get_my_bundle_hash()?;

    // Publish new bundle
    let new_bundle_hash = publish_pre_key_bundle(input.new_bundle)?;

    // Record rotation
    if let Some(old_hash) = old_bundle_hash {
        let rotation = KeyRotation {
            old_bundle_hash: old_hash,
            new_bundle_hash: new_bundle_hash.clone(),
            rotated_at: sys_time()?.as_micros() as u64,
            reason: input.reason,
        };
        let rotation_hash = create_entry(EntryTypes::KeyRotation(rotation))?;

        create_link(my_agent, rotation_hash, LinkTypes::KeyRotations, ())?;
    }

    Ok(new_bundle_hash)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RotateKeysInput {
    pub new_bundle: PreKeyBundle,
    pub reason: RotationReason,
}

/// Get key rotation history
#[hdk_extern]
pub fn get_rotation_history(_: ()) -> ExternResult<Vec<KeyRotation>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(my_agent, LinkTypes::KeyRotations)?,
        GetStrategy::default(),
    )?;

    let mut rotations = Vec::new();
    for link in links {
        if let Some(hash) = link.target.clone().into_action_hash() {
            if let Some(record) = get(hash, GetOptions::default())? {
                if let Some(rotation) = record
                    .entry()
                    .to_app_option::<KeyRotation>()
                    .map_err(|e| wasm_error!(e))?
                {
                    rotations.push(rotation);
                }
            }
        }
    }

    rotations.sort_by(|a, b| b.rotated_at.cmp(&a.rotated_at));

    Ok(rotations)
}

// ==================== BUNDLE REFRESH ====================

/// Check if bundle needs refresh
#[hdk_extern]
pub fn needs_refresh(_: ()) -> ExternResult<BundleStatus> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let now = sys_time()?.as_micros() as u64;

    if let Some(bundle) = get_pre_key_bundle(my_agent)? {
        let available_keys = bundle.one_time_pre_keys.iter().filter(|k| !k.used).count();

        // Check expiration (warn if expires within 24 hours)
        let one_day = 24 * 60 * 60 * 1_000_000; // microseconds
        if bundle.expires_at < now + one_day {
            return Ok(BundleStatus::ExpiringSoon);
        }

        // Check if running low on one-time keys
        if available_keys < 10 {
            return Ok(BundleStatus::LowOnKeys(available_keys as u32));
        }

        Ok(BundleStatus::Ok)
    } else {
        Ok(BundleStatus::NoBundle)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BundleStatus {
    Ok,
    NoBundle,
    ExpiringSoon,
    Expired,
    LowOnKeys(u32),
}

// ==================== HELPERS ====================

fn get_my_bundle_hash() -> ExternResult<Option<ActionHash>> {
    let my_agent = agent_info()?.agent_initial_pubkey;
    let links = get_links(
        LinkQuery::try_new(my_agent, LinkTypes::AgentToBundle)?,
        GetStrategy::default(),
    )?;

    // Get most recent
    let mut bundles: Vec<(u64, ActionHash)> = Vec::new();
    for link in links {
        if let Some(hash) = link.target.clone().into_action_hash() {
            if let Some(record) = get(hash.clone(), GetOptions::default())? {
                if let Some(bundle) = record
                    .entry()
                    .to_app_option::<PreKeyBundle>()
                    .map_err(|e| wasm_error!(e))?
                {
                    bundles.push((bundle.created_at, hash));
                }
            }
        }
    }

    bundles.sort_by(|a, b| b.0.cmp(&a.0));
    Ok(bundles.into_iter().next().map(|(_, h)| h))
}
