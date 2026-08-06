// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Client-side encryption for Mycelix Pulse.
//!
//! This module now uses a real shared-secret flow for the active web app path:
//! X25519 key agreement in Rust/WASM, HKDF-SHA256 for key derivation, and
//! AES-256-GCM via Web Crypto for payload encryption.

use base64::{Engine as _, engine::general_purpose::STANDARD as BASE64};
use hkdf::Hkdf;
use sha2::Sha256;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use x25519_dalek::{PublicKey, StaticSecret};

const LOCAL_IDENTITY_SECRET_KEY: &str = "mycelix_pulse_identity_secret_b64";
const LOCAL_IDENTITY_PUBLIC_KEY: &str = "mycelix_pulse_identity_public_b64";
const HKDF_INFO_SUBJECT: &[u8] = b"mycelix-pulse-v1-subject";
const HKDF_INFO_BODY: &[u8] = b"mycelix-pulse-v1-body";

pub struct MessageCrypto {
    pub ephemeral_pubkey: Vec<u8>,
    pub nonce: [u8; 24],
    pub subject_key: [u8; 32],
    pub body_key: [u8; 32],
}

/// Generate cryptographically secure random bytes.
pub fn generate_nonce(len: usize) -> Vec<u8> {
    let mut buf = vec![0u8; len];
    let crypto = web_sys::window()
        .unwrap()
        .crypto()
        .expect("crypto API required");
    crypto
        .get_random_values_with_u8_array(&mut buf)
        .expect("RNG failed");
    buf
}

/// Ensure a local identity keypair exists and return the public key bytes.
pub fn ensure_local_identity_keypair() -> Result<Vec<u8>, String> {
    if let Some(public_key) = load_local_identity_public_key() {
        return Ok(public_key.to_vec());
    }

    let secret_bytes = generate_nonce(32);
    let secret_array: [u8; 32] = secret_bytes
        .as_slice()
        .try_into()
        .map_err(|_| "Could not generate 32-byte identity key".to_string())?;

    let secret = StaticSecret::from(secret_array);
    let public = PublicKey::from(&secret);

    store_local_identity(&secret.to_bytes(), &public.to_bytes())?;

    Ok(public.to_bytes().to_vec())
}

pub fn local_identity_public_key() -> Option<Vec<u8>> {
    load_local_identity_public_key().map(|pk| pk.to_vec())
}

pub fn generate_public_key_bytes() -> Vec<u8> {
    let secret_bytes = generate_nonce(32);
    let secret_array: [u8; 32] = secret_bytes
        .as_slice()
        .try_into()
        .expect("32-byte key generation failed");
    let secret = StaticSecret::from(secret_array);
    let public = PublicKey::from(&secret);
    public.to_bytes().to_vec()
}

pub fn decode_transport_text(bytes: &[u8]) -> Option<String> {
    String::from_utf8(bytes.to_vec()).ok()
}

pub fn derive_message_crypto(recipient_pubkey: &[u8]) -> Result<MessageCrypto, String> {
    let recipient_array: [u8; 32] = recipient_pubkey
        .try_into()
        .map_err(|_| "Recipient public key must be 32 bytes".to_string())?;

    let ephemeral_secret_bytes = generate_nonce(32);
    let ephemeral_secret_array: [u8; 32] = ephemeral_secret_bytes
        .as_slice()
        .try_into()
        .map_err(|_| "Could not create ephemeral secret".to_string())?;
    let ephemeral_secret = StaticSecret::from(ephemeral_secret_array);
    let ephemeral_public = PublicKey::from(&ephemeral_secret);
    let recipient_public = PublicKey::from(recipient_array);
    let shared_secret = ephemeral_secret.diffie_hellman(&recipient_public);

    let mut subject_key = [0u8; 32];
    let mut body_key = [0u8; 32];
    let hk = Hkdf::<Sha256>::new(Some(ephemeral_public.as_bytes()), shared_secret.as_bytes());
    hk.expand(HKDF_INFO_SUBJECT, &mut subject_key)
        .map_err(|_| "Could not derive subject key".to_string())?;
    hk.expand(HKDF_INFO_BODY, &mut body_key)
        .map_err(|_| "Could not derive body key".to_string())?;

    let mut nonce = [0u8; 24];
    nonce.copy_from_slice(&generate_nonce(24));

    Ok(MessageCrypto {
        ephemeral_pubkey: ephemeral_public.to_bytes().to_vec(),
        nonce,
        subject_key,
        body_key,
    })
}

pub fn derive_message_crypto_for_local_recipient(
    ephemeral_pubkey: &[u8],
    nonce: &[u8],
) -> Result<MessageCrypto, String> {
    let secret = load_local_identity_secret_key()
        .ok_or_else(|| "No local identity secret available".to_string())?;
    let ephemeral_array: [u8; 32] = ephemeral_pubkey
        .try_into()
        .map_err(|_| "Ephemeral public key must be 32 bytes".to_string())?;
    let recipient_secret = StaticSecret::from(secret);
    let shared_secret = recipient_secret.diffie_hellman(&PublicKey::from(ephemeral_array));
    let mut subject_key = [0u8; 32];
    let mut body_key = [0u8; 32];
    let hk = Hkdf::<Sha256>::new(Some(ephemeral_pubkey), shared_secret.as_bytes());
    hk.expand(HKDF_INFO_SUBJECT, &mut subject_key)
        .map_err(|_| "Could not derive subject key".to_string())?;
    hk.expand(HKDF_INFO_BODY, &mut body_key)
        .map_err(|_| "Could not derive body key".to_string())?;

    let nonce_array: [u8; 24] = nonce
        .try_into()
        .map_err(|_| "Nonce must be 24 bytes".to_string())?;

    Ok(MessageCrypto {
        ephemeral_pubkey: ephemeral_pubkey.to_vec(),
        nonce: nonce_array,
        subject_key,
        body_key,
    })
}

fn web_crypto() -> Result<web_sys::SubtleCrypto, String> {
    web_sys::window()
        .ok_or_else(|| "window unavailable".to_string())?
        .crypto()
        .map_err(|e| format!("Web Crypto unavailable: {e:?}"))
        .map(|crypto| crypto.subtle())
}

async fn import_aes_gcm_key(key: &[u8; 32], usage: &str) -> Result<web_sys::CryptoKey, String> {
    let subtle = web_crypto()?;
    let key_bytes = js_sys::Uint8Array::from(key.as_slice());
    let usages = js_sys::Array::new();
    usages.push(&JsValue::from_str(usage));
    let key_data: &js_sys::Object = key_bytes.unchecked_ref::<js_sys::Object>();
    let promise = subtle
        .import_key_with_str("raw", key_data, "AES-GCM", false, usages.as_ref())
        .map_err(|e| format!("AES-GCM key import failed: {e:?}"))?;
    JsFuture::from(promise)
        .await
        .map_err(|e| format!("AES-GCM key import failed: {e:?}"))?
        .dyn_into::<web_sys::CryptoKey>()
        .map_err(|e| format!("AES-GCM key result was invalid: {e:?}"))
}

fn aes_gcm_params(nonce: &[u8; 24]) -> Result<js_sys::Object, String> {
    let params = js_sys::Object::new();
    js_sys::Reflect::set(
        &params,
        &JsValue::from_str("name"),
        &JsValue::from_str("AES-GCM"),
    )
    .map_err(|e| format!("Could not set AES-GCM algorithm name: {e:?}"))?;
    let iv = js_sys::Uint8Array::from(&nonce[..12]);
    js_sys::Reflect::set(&params, &JsValue::from_str("iv"), iv.as_ref())
        .map_err(|e| format!("Could not set AES-GCM IV: {e:?}"))?;
    Ok(params)
}

pub async fn encrypt_with_key(
    plaintext: &[u8],
    key: &[u8; 32],
    nonce: &[u8; 24],
) -> Result<Vec<u8>, String> {
    let subtle = web_crypto()?;
    let crypto_key = import_aes_gcm_key(key, "encrypt").await?;
    let params = aes_gcm_params(nonce)?;
    let promise = subtle
        .encrypt_with_object_and_u8_array(&params, &crypto_key, plaintext)
        .map_err(|e| format!("AES-GCM encrypt setup failed: {e:?}"))?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|e| format!("AES-GCM encrypt failed: {e:?}"))?;
    Ok(js_sys::Uint8Array::new(&result).to_vec())
}

pub async fn decrypt_with_key(
    ciphertext: &[u8],
    key: &[u8; 32],
    nonce: &[u8; 24],
) -> Result<Vec<u8>, String> {
    let subtle = web_crypto()?;
    let crypto_key = import_aes_gcm_key(key, "decrypt").await?;
    let params = aes_gcm_params(nonce)?;
    let promise = subtle
        .decrypt_with_object_and_u8_array(&params, &crypto_key, ciphertext)
        .map_err(|e| format!("AES-GCM decrypt setup failed: {e:?}"))?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|e| format!("AES-GCM decrypt failed: {e:?}"))?;
    Ok(js_sys::Uint8Array::new(&result).to_vec())
}

fn store_local_identity(secret: &[u8; 32], public: &[u8; 32]) -> Result<(), String> {
    let storage = web_sys::window()
        .and_then(|w| w.local_storage().ok().flatten())
        .ok_or_else(|| "localStorage unavailable".to_string())?;
    storage
        .set_item(LOCAL_IDENTITY_SECRET_KEY, &base64_encode(secret))
        .map_err(|e| format!("Could not store identity secret: {e:?}"))?;
    storage
        .set_item(LOCAL_IDENTITY_PUBLIC_KEY, &base64_encode(public))
        .map_err(|e| format!("Could not store identity public key: {e:?}"))?;
    Ok(())
}

fn load_local_identity_secret_key() -> Option<[u8; 32]> {
    let storage = web_sys::window()?.local_storage().ok().flatten()?;
    let secret = storage.get_item(LOCAL_IDENTITY_SECRET_KEY).ok().flatten()?;
    let secret_bytes = base64_decode(&secret);
    secret_bytes.try_into().ok()
}

fn load_local_identity_public_key() -> Option<[u8; 32]> {
    let storage = web_sys::window()?.local_storage().ok().flatten()?;
    let public = storage.get_item(LOCAL_IDENTITY_PUBLIC_KEY).ok().flatten()?;
    let public_bytes = base64_decode(&public);
    public_bytes.try_into().ok()
}

fn base64_encode(data: &[u8]) -> String {
    BASE64.encode(data)
}

fn base64_decode(s: &str) -> Vec<u8> {
    BASE64.decode(s).unwrap_or_default()
}
