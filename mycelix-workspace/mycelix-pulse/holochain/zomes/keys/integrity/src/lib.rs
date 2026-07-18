// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Keys Integrity Zome
//!
//! Pre-key bundles for E2E encryption key exchange.

use hdi::prelude::*;
use sha2::{Digest, Sha256};

pub const HYBRID_KEY_BUNDLE_V2: u16 = 2;
pub const HYBRID_SUITE_V2: &str = "x25519+ml-kem-768-hkdf-sha256-aes256gcm-agent-ed25519+ml-dsa-65";
pub const ML_KEM_768_PUBLIC_KEY_BYTES: usize = 1184;
pub const ML_DSA_65_PUBLIC_KEY_BYTES: usize = 1952;
const HYBRID_KEY_ID_DOMAIN: &[u8] = b"mycelix-pulse/key-bundle/v2\0";
const HYBRID_KEY_SIGNATURE_DOMAIN: &[u8] = b"mycelix-pulse/key-bundle-signature/v2\0";

/// Pre-key bundle for X3DH key exchange
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct PreKeyBundle {
    /// Long-term identity key (public)
    pub identity_key: Vec<u8>,
    /// Signed pre-key (public)
    pub signed_pre_key: Vec<u8>,
    /// Signed pre-key ID
    pub signed_pre_key_id: u32,
    /// Signature of signed pre-key
    pub signed_pre_key_signature: Vec<u8>,
    /// One-time pre-keys (public)
    pub one_time_pre_keys: Vec<OneTimePreKey>,
    /// When the bundle was created
    pub created_at: u64,
    /// When the bundle expires
    pub expires_at: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct OneTimePreKey {
    pub key_id: u32,
    pub public_key: Vec<u8>,
    pub used: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum HybridKeyStateV2 {
    Active,
    Retired,
    RevokedCompromised,
    Lost,
}

/// Agent-bound static keys used by the Pulse V2 envelope. Secret material is
/// device-local and never enters this record.
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct HybridKeyBundleV2 {
    pub version: u16,
    pub suite: String,
    pub key_id: [u8; 32],
    pub x25519_public_key: [u8; 32],
    pub ml_kem_768_public_key: Vec<u8>,
    pub ml_dsa_65_public_key: Vec<u8>,
    pub state: HybridKeyStateV2,
    pub created_at: u64,
    pub expires_at: u64,
    /// Active Holochain agent signature over `hybrid_key_signing_content`.
    pub agent_signature: Vec<u8>,
}

fn put_bytes(out: &mut Vec<u8>, value: &[u8]) {
    out.extend_from_slice(&(value.len() as u32).to_be_bytes());
    out.extend_from_slice(value);
}

pub fn hybrid_key_id(bundle: &HybridKeyBundleV2) -> [u8; 32] {
    let mut transcript = Vec::with_capacity(
        128 + bundle.ml_kem_768_public_key.len() + bundle.ml_dsa_65_public_key.len(),
    );
    transcript.extend_from_slice(HYBRID_KEY_ID_DOMAIN);
    put_bytes(&mut transcript, bundle.suite.as_bytes());
    transcript.extend_from_slice(&bundle.x25519_public_key);
    put_bytes(&mut transcript, &bundle.ml_kem_768_public_key);
    put_bytes(&mut transcript, &bundle.ml_dsa_65_public_key);
    Sha256::digest(&transcript).into()
}

pub fn hybrid_key_signing_content(bundle: &HybridKeyBundleV2) -> Vec<u8> {
    let mut content = Vec::with_capacity(
        192 + bundle.ml_kem_768_public_key.len() + bundle.ml_dsa_65_public_key.len(),
    );
    content.extend_from_slice(HYBRID_KEY_SIGNATURE_DOMAIN);
    content.extend_from_slice(&bundle.version.to_be_bytes());
    put_bytes(&mut content, bundle.suite.as_bytes());
    content.extend_from_slice(&bundle.key_id);
    content.extend_from_slice(&bundle.x25519_public_key);
    put_bytes(&mut content, &bundle.ml_kem_768_public_key);
    put_bytes(&mut content, &bundle.ml_dsa_65_public_key);
    content.push(match bundle.state {
        HybridKeyStateV2::Active => 1,
        HybridKeyStateV2::Retired => 2,
        HybridKeyStateV2::RevokedCompromised => 3,
        HybridKeyStateV2::Lost => 4,
    });
    content.extend_from_slice(&bundle.created_at.to_be_bytes());
    content.extend_from_slice(&bundle.expires_at.to_be_bytes());
    content
}

/// Used pre-key record (to prevent reuse)
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct UsedPreKey {
    pub key_id: u32,
    pub bundle_hash: ActionHash,
    pub used_at: u64,
    pub used_by: AgentPubKey,
}

/// Key rotation record
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct KeyRotation {
    pub old_bundle_hash: ActionHash,
    pub new_bundle_hash: ActionHash,
    pub rotated_at: u64,
    pub reason: RotationReason,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum RotationReason {
    Scheduled,
    Compromised,
    PreKeysExhausted,
    Manual,
}

#[hdk_entry_types]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    PreKeyBundle(PreKeyBundle),
    HybridKeyBundleV2(HybridKeyBundleV2),
    UsedPreKey(UsedPreKey),
    KeyRotation(KeyRotation),
}

fn validate_create_hybrid_key_bundle_v2(
    action: Create,
    bundle: HybridKeyBundleV2,
) -> ExternResult<ValidateCallbackResult> {
    if bundle.version != HYBRID_KEY_BUNDLE_V2 || bundle.suite != HYBRID_SUITE_V2 {
        return Ok(ValidateCallbackResult::Invalid(
            "Unsupported hybrid key bundle version or suite".into(),
        ));
    }
    if bundle.key_id != hybrid_key_id(&bundle) {
        return Ok(ValidateCallbackResult::Invalid(
            "Hybrid key ID does not match its public-key transcript".into(),
        ));
    }
    if bundle.ml_kem_768_public_key.len() != ML_KEM_768_PUBLIC_KEY_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "ML-KEM-768 public key must be 1184 bytes".into(),
        ));
    }
    if bundle.ml_dsa_65_public_key.len() != ML_DSA_65_PUBLIC_KEY_BYTES {
        return Ok(ValidateCallbackResult::Invalid(
            "ML-DSA-65 public key must be 1952 bytes".into(),
        ));
    }
    if bundle.expires_at <= bundle.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Expiration must be after creation time".into(),
        ));
    }
    if bundle.agent_signature.len() != 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Agent signature must be 64 bytes".into(),
        ));
    }
    let mut signature = [0; 64];
    signature.copy_from_slice(&bundle.agent_signature);
    if !verify_signature_raw(
        action.author,
        Signature(signature),
        hybrid_key_signing_content(&bundle),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "Hybrid key bundle agent signature verification failed".into(),
        ));
    }
    Ok(ValidateCallbackResult::Valid)
}

#[hdk_link_types]
pub enum LinkTypes {
    AgentToBundle,
    BundleToUsedKeys,
    KeyRotations,
}

pub fn pre_key_signing_content(bundle: &PreKeyBundle) -> Vec<u8> {
    let mut content = Vec::with_capacity(128);
    content.extend_from_slice(b"mycelix-pulse/pre-key/v1\0");
    content.extend_from_slice(&bundle.identity_key);
    content.extend_from_slice(&bundle.signed_pre_key);
    content.extend_from_slice(&bundle.signed_pre_key_id.to_be_bytes());
    content.extend_from_slice(&bundle.created_at.to_be_bytes());
    content.extend_from_slice(&bundle.expires_at.to_be_bytes());
    content
}

/// Validate pre-key bundle
fn validate_create_pre_key_bundle(
    action: Create,
    bundle: PreKeyBundle,
) -> ExternResult<ValidateCallbackResult> {
    // Validate identity key length (32 bytes for X25519)
    if bundle.identity_key.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Identity key must be 32 bytes".to_string(),
        ));
    }

    // Validate signed pre-key length
    if bundle.signed_pre_key.len() != 32 {
        return Ok(ValidateCallbackResult::Invalid(
            "Signed pre-key must be 32 bytes".to_string(),
        ));
    }

    // Validate signature length (64 bytes for Ed25519)
    if bundle.signed_pre_key_signature.len() != 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Signature must be 64 bytes".to_string(),
        ));
    }
    let mut signature = [0u8; 64];
    signature.copy_from_slice(&bundle.signed_pre_key_signature);
    if !verify_signature_raw(
        action.author,
        Signature(signature),
        pre_key_signing_content(&bundle),
    )? {
        return Ok(ValidateCallbackResult::Invalid(
            "Signed pre-key signature verification failed".to_string(),
        ));
    }

    // Validate at least some one-time pre-keys
    if bundle.one_time_pre_keys.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Bundle must have at least one one-time pre-key".to_string(),
        ));
    }

    // Validate one-time pre-key lengths
    for otpk in &bundle.one_time_pre_keys {
        if otpk.public_key.len() != 32 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "One-time pre-key {} must be 32 bytes",
                otpk.key_id
            )));
        }
    }

    // Validate expiration is after creation
    if bundle.expires_at <= bundle.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Expiration must be after creation time".to_string(),
        ));
    }

    // Note: Expiration check against current time is done in coordinator zome
    // since sys_time() is not available in integrity zomes

    Ok(ValidateCallbackResult::Valid)
}

/// Validate used pre-key record -- bind to its committer. consume_pre_key already derives
/// used_by from agent_info() coordinator-side with zero user input (P0 author-binding gap).
/// Note: consume_pre_key legitimately calls update_entry on the OTHER agent's PreKeyBundle
/// (X3DH protocol -- the consumer marks the bundle owner's one-time key used) -- that
/// cross-agent update path is a real, deliberate exception and is NOT touched here.
fn validate_create_used_pre_key(
    action: Create,
    used: UsedPreKey,
) -> ExternResult<ValidateCallbackResult> {
    // Basic validation
    if used.key_id == 0 {
        return Ok(ValidateCallbackResult::Invalid(
            "Key ID cannot be 0".to_string(),
        ));
    }

    if used.used_by != action.author {
        return Ok(ValidateCallbackResult::Invalid(
            "UsedPreKey must be recorded by the consuming agent (used_by forgery)".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validate key rotation
fn validate_create_key_rotation(
    _action: Create,
    rotation: KeyRotation,
) -> ExternResult<ValidateCallbackResult> {
    // Old and new bundles must be different
    if rotation.old_bundle_hash == rotation.new_bundle_hash {
        return Ok(ValidateCallbackResult::Invalid(
            "Old and new bundle hashes must be different".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation dispatcher
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(store_entry) => match store_entry {
            OpEntry::CreateEntry { app_entry, action } => match app_entry {
                EntryTypes::PreKeyBundle(bundle) => validate_create_pre_key_bundle(action, bundle),
                EntryTypes::HybridKeyBundleV2(bundle) => {
                    validate_create_hybrid_key_bundle_v2(action, bundle)
                }
                EntryTypes::UsedPreKey(used) => validate_create_used_pre_key(action, used),
                EntryTypes::KeyRotation(rotation) => validate_create_key_rotation(action, rotation),
            },
            OpEntry::UpdateEntry { app_entry, .. } => match app_entry {
                EntryTypes::PreKeyBundle(bundle) => {
                    // Updates are allowed (for marking keys as used)
                    if bundle.identity_key.len() != 32 {
                        return Ok(ValidateCallbackResult::Invalid(
                            "Identity key must be 32 bytes".to_string(),
                        ));
                    }
                    Ok(ValidateCallbackResult::Valid)
                }
                EntryTypes::HybridKeyBundleV2(_) => Ok(ValidateCallbackResult::Invalid(
                    "Hybrid V2 bundles are immutable; publish a successor bundle".into(),
                )),
                _ => Ok(ValidateCallbackResult::Valid),
            },
            _ => Ok(ValidateCallbackResult::Valid),
        },
        FlatOp::RegisterCreateLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDeleteLink { .. } => Ok(ValidateCallbackResult::Valid),
        FlatOp::StoreRecord(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterAgentActivity(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterUpdate(_) => Ok(ValidateCallbackResult::Valid),
        FlatOp::RegisterDelete(_) => Ok(ValidateCallbackResult::Valid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bundle() -> HybridKeyBundleV2 {
        let mut value = HybridKeyBundleV2 {
            version: HYBRID_KEY_BUNDLE_V2,
            suite: HYBRID_SUITE_V2.into(),
            key_id: [0; 32],
            x25519_public_key: [1; 32],
            ml_kem_768_public_key: vec![2; ML_KEM_768_PUBLIC_KEY_BYTES],
            ml_dsa_65_public_key: vec![3; ML_DSA_65_PUBLIC_KEY_BYTES],
            state: HybridKeyStateV2::Active,
            created_at: 10,
            expires_at: 20,
            agent_signature: vec![0; 64],
        };
        value.key_id = hybrid_key_id(&value);
        value
    }

    #[test]
    fn key_id_binds_both_pq_and_classical_public_keys() {
        let original = bundle();
        let id = original.key_id;
        let mut changed = original.clone();
        changed.x25519_public_key[0] ^= 1;
        assert_ne!(id, hybrid_key_id(&changed));
        let mut changed = original.clone();
        changed.ml_kem_768_public_key[0] ^= 1;
        assert_ne!(id, hybrid_key_id(&changed));
        let mut changed = original;
        changed.ml_dsa_65_public_key[0] ^= 1;
        assert_ne!(id, hybrid_key_id(&changed));
    }

    #[test]
    fn signature_transcript_binds_state_and_expiry() {
        let original = bundle();
        let transcript = hybrid_key_signing_content(&original);
        let mut changed = original.clone();
        changed.state = HybridKeyStateV2::Retired;
        assert_ne!(transcript, hybrid_key_signing_content(&changed));
        let mut changed = original;
        changed.expires_at += 1;
        assert_ne!(transcript, hybrid_key_signing_content(&changed));
    }
}
