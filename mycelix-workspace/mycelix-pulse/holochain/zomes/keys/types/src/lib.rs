// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Keys types — plain data shared between the `keys` zome and other zomes
//! that need to inspect a `HybridKeyBundleV2` (e.g. `mail_messages_integrity`
//! verifying a sender's ML-DSA-65 signature against their published bundle).
//!
//! Deliberately has **no** `#[hdk_entry_types]`/`#[hdk_link_types]`/
//! `#[hdk_extern]` — those macros generate globally-named WASM exports
//! (`entry_defs`, `validate`, `__num_entry_types`, ...) that collide at link
//! time if two zome crates both define them and get linked into the same
//! cdylib. This crate is `rlib`-only and never built as its own zome —
//! `keys_integrity` depends on it for its own entry registration, and other
//! integrity zomes may depend on it directly for read-only type access
//! without pulling in `keys_integrity`'s zome surface.

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
