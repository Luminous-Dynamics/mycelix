// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Experimental authenticated WebAuthn-PRF identity-seed envelope.
//!
//! This module is deliberately not called by the production identity path.
//! It demonstrates the next composition step after PRF-to-key derivation:
//! AES-256-GCM through Web Crypto with canonical, versioned additional
//! authenticated data (AAD).
//!
//! A caller must pass an independently expected RP ID, credential ID, stored
//! identity public key, and matching envelope salt. Do not reconstruct the
//! trust context only from untrusted envelope metadata. A real caller must
//! first validate the WebAuthn assertion's origin, RP ID, challenge,
//! user-verification policy, selected credential ID and PRF extension result.
//!
//! The output is a 32-byte Ed25519 seed only after AEAD authentication and
//! public-key binding both succeed. No browser credential ceremony or storage
//! read/write is performed here.

use super::webauthn_prf_envelope::{
    derive_prf_envelope_key, PrfEnvelopeDerivationError, PrfEnvelopeKey,
    WebAuthnPrfEnvelopeContext,
};
use base64::{Engine as _, engine::general_purpose::STANDARD};
use ed25519_dalek::SigningKey;
use js_sys::Uint8Array;
use serde::{Deserialize, Serialize};
use std::fmt;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{AesGcmParams, CryptoKey, SubtleCrypto};
use zeroize::{Zeroize, Zeroizing};

pub const PRF_WRAPPED_ENVELOPE_VERSION: u8 = 1;
pub const PRF_WRAPPED_CIPHERTEXT_LEN: usize = 48;
pub const AES_GCM_NONCE_LEN: usize = 12;

const KEY_PROFILE: &str = "WEBAUTHN-PRF-HKDF-SHA256-V1";
const CIPHER_PROFILE: &str = "AES-256-GCM";
const AAD_DOMAIN: &[u8] = b"LUMINOUS-DYNAMICS/MYCELIX/WEBAUTHN-PRF-ENVELOPE-AAD/V1\0";
const PAYLOAD_PURPOSE: &[u8] = b"mycelix-ed25519-seed-wrap-v1";

/// Serializable envelope metadata and ciphertext.
///
/// The structure deliberately contains only public metadata and ciphertext.
/// It does not implement Debug to discourage accidental diagnostic logging of
/// credential identifiers and stable correlating metadata.
#[derive(Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrfWrappedIdentityEnvelopeV1 {
    pub format_version: u8,
    pub key_profile: String,
    pub cipher_profile: String,
    pub relying_party_id: String,
    pub credential_id_b64: String,
    pub envelope_salt_b64: String,
    pub nonce_b64: String,
    pub identity_public_key_b64: String,
    pub ciphertext_b64: String,
}

/// Non-sensitive failure categories. Cryptographic failures do not expose
/// raw browser exceptions, PRF bytes, plaintext or key material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrfEnvelopeAeadError {
    Derivation(PrfEnvelopeDerivationError),
    BrowserCryptoUnavailable,
    RandomNonceFailed,
    KeyImportFailed,
    EncryptionFailed,
    DecryptionFailed,
    InvalidEnvelopeMetadata,
    ContextMismatch,
    IdentityPublicKeyMismatch,
    InvalidPlaintextLength,
    ContextLengthOverflow,
}

impl fmt::Display for PrfEnvelopeAeadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Derivation(error) => write!(f, "PRF key derivation failed: {error}"),
            Self::BrowserCryptoUnavailable => f.write_str("Web Crypto is unavailable"),
            Self::RandomNonceFailed => f.write_str("Could not generate an envelope nonce"),
            Self::KeyImportFailed => f.write_str("Could not import the derived AES key"),
            Self::EncryptionFailed => f.write_str("Envelope encryption failed"),
            Self::DecryptionFailed => f.write_str("Envelope authentication or decryption failed"),
            Self::InvalidEnvelopeMetadata => f.write_str("Envelope metadata is malformed or unsupported"),
            Self::ContextMismatch => f.write_str("Envelope does not match the expected identity context"),
            Self::IdentityPublicKeyMismatch => f.write_str("Seed does not match the expected identity public key"),
            Self::InvalidPlaintextLength => f.write_str("Authenticated identity seed has an invalid length"),
            Self::ContextLengthOverflow => f.write_str("Authenticated envelope context length is not representable"),
        }
    }
}

impl std::error::Error for PrfEnvelopeAeadError {}

impl From<PrfEnvelopeDerivationError> for PrfEnvelopeAeadError {
    fn from(value: PrfEnvelopeDerivationError) -> Self {
        Self::Derivation(value)
    }
}

/// Encrypt one 32-byte Ed25519 seed using a verified WebAuthn PRF output.
///
/// The supplied context must contain the actual credential ID, RP ID, fresh
/// PRF evaluation salt and public key expected by the application. The
/// function checks that the seed derives that public key, consumes and
/// zeroizes the Rust-owned PRF output through the derivation module, uses a
/// fresh 96-bit nonce from Web Crypto, and returns an AEAD envelope whose
/// metadata is authenticated as AAD.
///
/// This is an experimental building block only. It does not establish that
/// the caller validated a WebAuthn assertion or that the key is hardware-backed.
pub async fn encrypt_seed_with_prf(
    seed: &[u8; 32],
    prf_output: Vec<u8>,
    context: &WebAuthnPrfEnvelopeContext<'_>,
) -> Result<PrfWrappedIdentityEnvelopeV1, PrfEnvelopeAeadError> {
    let mut prf_output = Zeroizing::new(prf_output);
    if !seed_matches_public_key(seed, context.identity_public_key) {
        return Err(PrfEnvelopeAeadError::IdentityPublicKeyMismatch);
    }

    let owned_prf_output = std::mem::take(&mut *prf_output);
    let derived_key = derive_prf_envelope_key(owned_prf_output, context)?;
    let mut nonce = random_nonce()?;
    let mut aad = canonical_aad(context, &nonce)?;
    let crypto_key = import_non_extractable_aes_key(&derived_key).await?;
    let subtle = subtle()?;

    let params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut nonce);
    params.set_additional_data_u8_slice(&mut aad);
    let encrypted = subtle
        .encrypt_with_object_and_u8_array(&params, &crypto_key, seed)
        .map_err(|_| PrfEnvelopeAeadError::EncryptionFailed)?;
    let encrypted = JsFuture::from(encrypted)
        .await
        .map_err(|_| PrfEnvelopeAeadError::EncryptionFailed)?;
    let ciphertext = Uint8Array::new(&encrypted).to_vec();
    if ciphertext.len() != PRF_WRAPPED_CIPHERTEXT_LEN {
        return Err(PrfEnvelopeAeadError::EncryptionFailed);
    }

    Ok(PrfWrappedIdentityEnvelopeV1 {
        format_version: PRF_WRAPPED_ENVELOPE_VERSION,
        key_profile: KEY_PROFILE.to_string(),
        cipher_profile: CIPHER_PROFILE.to_string(),
        relying_party_id: context.relying_party_id.to_string(),
        credential_id_b64: base64_encode(context.credential_id),
        envelope_salt_b64: base64_encode(context.envelope_salt),
        nonce_b64: base64_encode(&nonce),
        identity_public_key_b64: base64_encode(context.identity_public_key),
        ciphertext_b64: base64_encode(&ciphertext),
    })
}

/// Decrypt an envelope only when all metadata matches the caller's expected
/// context, AEAD authentication succeeds, and the recovered seed derives the
/// expected Ed25519 public key.
///
/// The caller should source the expected RP ID from the application origin,
/// the credential ID from the successful WebAuthn assertion, and the public
/// key from a trusted current identity anchor—not trust these fields merely
/// because they appear in the envelope.
pub async fn decrypt_seed_with_prf(
    envelope: &PrfWrappedIdentityEnvelopeV1,
    prf_output: Vec<u8>,
    expected_context: &WebAuthnPrfEnvelopeContext<'_>,
) -> Result<Zeroizing<[u8; 32]>, PrfEnvelopeAeadError> {
    let mut prf_output = Zeroizing::new(prf_output);
    let (nonce, ciphertext) = validate_expected_context(envelope, expected_context)?;
    let owned_prf_output = std::mem::take(&mut *prf_output);
    let derived_key = derive_prf_envelope_key(owned_prf_output, expected_context)?;
    let mut aad = canonical_aad(expected_context, &nonce)?;
    let crypto_key = import_non_extractable_aes_key(&derived_key).await?;
    let subtle = subtle()?;

    let mut nonce_for_webcrypto = nonce;
    let params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut nonce_for_webcrypto);
    params.set_additional_data_u8_slice(&mut aad);
    let ciphertext = Uint8Array::from(ciphertext.as_slice());
    let decrypted = subtle
        .decrypt_with_object_and_u8_array(&params, &crypto_key, &ciphertext)
        .map_err(|_| PrfEnvelopeAeadError::DecryptionFailed)?;
    let decrypted = JsFuture::from(decrypted)
        .await
        .map_err(|_| PrfEnvelopeAeadError::DecryptionFailed)?;
    let mut plaintext = Zeroizing::new(Uint8Array::new(&decrypted).to_vec());
    if plaintext.len() != 32 {
        return Err(PrfEnvelopeAeadError::InvalidPlaintextLength);
    }

    let mut seed_bytes = [0u8; 32];
    seed_bytes.copy_from_slice(&plaintext);
    plaintext.zeroize();
    let seed = Zeroizing::new(seed_bytes);
    if !seed_matches_public_key(&seed, expected_context.identity_public_key) {
        return Err(PrfEnvelopeAeadError::IdentityPublicKeyMismatch);
    }
    Ok(seed)
}

fn validate_expected_context(
    envelope: &PrfWrappedIdentityEnvelopeV1,
    expected: &WebAuthnPrfEnvelopeContext<'_>,
) -> Result<([u8; AES_GCM_NONCE_LEN], Vec<u8>), PrfEnvelopeAeadError> {
    if envelope.format_version != PRF_WRAPPED_ENVELOPE_VERSION
        || envelope.key_profile != KEY_PROFILE
        || envelope.cipher_profile != CIPHER_PROFILE
        || envelope.relying_party_id != expected.relying_party_id
    {
        return Err(PrfEnvelopeAeadError::InvalidEnvelopeMetadata);
    }

    let credential_id = base64_decode(&envelope.credential_id_b64)?;
    let salt = base64_decode(&envelope.envelope_salt_b64)?;
    let public_key = base64_decode(&envelope.identity_public_key_b64)?;
    let nonce = base64_decode(&envelope.nonce_b64)?;
    let ciphertext = base64_decode(&envelope.ciphertext_b64)?;

    if credential_id != expected.credential_id
        || salt != expected.envelope_salt
        || public_key != expected.identity_public_key
    {
        return Err(PrfEnvelopeAeadError::ContextMismatch);
    }
    if nonce.len() != AES_GCM_NONCE_LEN
        || ciphertext.len() != PRF_WRAPPED_CIPHERTEXT_LEN
        || credential_id.is_empty()
        || credential_id.len() > super::webauthn_prf_envelope::MAX_CREDENTIAL_ID_LEN
        || salt.len() != super::webauthn_prf_envelope::ENVELOPE_SALT_LEN
        || public_key.len() != 32
    {
        return Err(PrfEnvelopeAeadError::InvalidEnvelopeMetadata);
    }

    let mut nonce_array = [0u8; AES_GCM_NONCE_LEN];
    nonce_array.copy_from_slice(&nonce);
    Ok((nonce_array, ciphertext))
}

fn canonical_aad(
    context: &WebAuthnPrfEnvelopeContext<'_>,
    nonce: &[u8; AES_GCM_NONCE_LEN],
) -> Result<Vec<u8>, PrfEnvelopeAeadError> {
    let mut aad = Vec::with_capacity(
        AAD_DOMAIN.len()
            + context.relying_party_id.len()
            + context.credential_id.len()
            + context.envelope_salt.len()
            + nonce.len()
            + context.identity_public_key.len()
            + KEY_PROFILE.len()
            + CIPHER_PROFILE.len()
            + PAYLOAD_PURPOSE.len()
            + 32,
    );
    aad.extend_from_slice(AAD_DOMAIN);
    aad.push(PRF_WRAPPED_ENVELOPE_VERSION);
    append_field(&mut aad, KEY_PROFILE.as_bytes())?;
    append_field(&mut aad, CIPHER_PROFILE.as_bytes())?;
    append_field(&mut aad, context.relying_party_id.as_bytes())?;
    append_field(&mut aad, context.credential_id)?;
    append_field(&mut aad, context.envelope_salt)?;
    append_field(&mut aad, nonce)?;
    append_field(&mut aad, context.identity_public_key)?;
    append_field(&mut aad, PAYLOAD_PURPOSE)?;
    Ok(aad)
}

fn append_field(target: &mut Vec<u8>, value: &[u8]) -> Result<(), PrfEnvelopeAeadError> {
    let length =
        u32::try_from(value.len()).map_err(|_| PrfEnvelopeAeadError::ContextLengthOverflow)?;
    target.extend_from_slice(&length.to_be_bytes());
    target.extend_from_slice(value);
    Ok(())
}

fn seed_matches_public_key(seed: &[u8; 32], public_key: &[u8]) -> bool {
    if public_key.len() != 32 {
        return false;
    }
    let signing_key = SigningKey::from_bytes(seed);
    signing_key.verifying_key().as_bytes() == public_key
}

fn random_nonce() -> Result<[u8; AES_GCM_NONCE_LEN], PrfEnvelopeAeadError> {
    let window = web_sys::window().ok_or(PrfEnvelopeAeadError::BrowserCryptoUnavailable)?;
    let crypto = window
        .crypto()
        .map_err(|_| PrfEnvelopeAeadError::BrowserCryptoUnavailable)?;
    let mut nonce = [0u8; AES_GCM_NONCE_LEN];
    crypto
        .get_random_values_with_u8_array(&mut nonce)
        .map_err(|_| PrfEnvelopeAeadError::RandomNonceFailed)?;
    Ok(nonce)
}

fn subtle() -> Result<SubtleCrypto, PrfEnvelopeAeadError> {
    let window = web_sys::window().ok_or(PrfEnvelopeAeadError::BrowserCryptoUnavailable)?;
    let crypto = window
        .crypto()
        .map_err(|_| PrfEnvelopeAeadError::BrowserCryptoUnavailable)?;
    Ok(crypto.subtle())
}

async fn import_non_extractable_aes_key(
    key: &PrfEnvelopeKey,
) -> Result<CryptoKey, PrfEnvelopeAeadError> {
    let subtle = subtle()?;
    let raw_key = Uint8Array::from(key.expose_for_crypto_import().as_slice());
    let usages = js_sys::Array::of2(
        &JsValue::from_str("encrypt"),
        &JsValue::from_str("decrypt"),
    );
    let import_promise = match subtle.import_key_with_str(
        "raw",
        &raw_key,
        "AES-GCM",
        false,
        &usages,
    ) {
        Ok(promise) => promise,
        Err(_) => {
            raw_key.fill(0);
            return Err(PrfEnvelopeAeadError::KeyImportFailed);
        }
    };
    let imported = JsFuture::from(import_promise).await;
    // Best-effort clearing of the JavaScript typed-array copy after import.
    raw_key.fill(0);
    let imported = imported.map_err(|_| PrfEnvelopeAeadError::KeyImportFailed)?;
    Ok(imported.unchecked_into())
}

fn base64_encode(bytes: &[u8]) -> String {
    STANDARD.encode(bytes)
}

fn base64_decode(value: &str) -> Result<Vec<u8>, PrfEnvelopeAeadError> {
    STANDARD
        .decode(value)
        .map_err(|_| PrfEnvelopeAeadError::InvalidEnvelopeMetadata)
}

#[cfg(all(test, target_arch = "wasm32"))]
mod browser_tests {
    use super::*;
    use wasm_bindgen_test::wasm_bindgen_test;

    wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

    fn fixture() -> ([u8; 32], Vec<u8>, [u8; 32], [u8; 32], [u8; 4]) {
        let seed = [7u8; 32];
        let public_key = SigningKey::from_bytes(&seed).verifying_key().to_bytes();
        (seed, vec![0x42; 32], [0x24; 32], public_key, [1, 2, 3, 4])
    }

    fn context<'a>(
        rp_id: &'a str,
        credential_id: &'a [u8],
        salt: &'a [u8],
        public_key: &'a [u8],
    ) -> WebAuthnPrfEnvelopeContext<'a> {
        WebAuthnPrfEnvelopeContext {
            relying_party_id: rp_id,
            credential_id,
            envelope_salt: salt,
            identity_public_key: public_key,
        }
    }

    async fn encrypted_fixture() -> (
        [u8; 32],
        Vec<u8>,
        [u8; 32],
        [u8; 32],
        [u8; 4],
        PrfWrappedIdentityEnvelopeV1,
    ) {
        let (seed, prf, salt, public_key, credential_id) = fixture();
        let context = context("localhost", &credential_id, &salt, &public_key);
        let envelope = encrypt_seed_with_prf(&seed, prf.clone(), &context)
            .await
            .expect("synthetic PRF output encrypts in Web Crypto");
        (seed, prf, salt, public_key, credential_id, envelope)
    }

    #[wasm_bindgen_test]
    async fn round_trip_authenticates_metadata_and_does_not_store_plain_seed() {
        let (seed, prf, salt, public_key, credential_id, envelope) = encrypted_fixture().await;
        let json = serde_json::to_string(&envelope).unwrap();
        let seed_b64 = base64_encode(&seed);
        assert!(!json.contains(&seed_b64));
        assert_eq!(envelope.format_version, PRF_WRAPPED_ENVELOPE_VERSION);

        let context = context("localhost", &credential_id, &salt, &public_key);
        let recovered = decrypt_seed_with_prf(&envelope, prf, &context)
            .await
            .expect("correct PRF/context decrypts");
        assert_eq!(&*recovered, &seed);
    }

    #[wasm_bindgen_test]
    async fn wrong_prf_output_fails_authentication() {
        let (_, _, salt, public_key, credential_id, envelope) = encrypted_fixture().await;
        let wrong_prf = vec![0x43; 32];
        let context = context("localhost", &credential_id, &salt, &public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, wrong_prf, &context).await.unwrap_err(),
            PrfEnvelopeAeadError::DecryptionFailed
        );
    }

    #[wasm_bindgen_test]
    async fn tampered_ciphertext_fails_authentication() {
        let (_, prf, salt, public_key, credential_id, mut envelope) = encrypted_fixture().await;
        let mut ciphertext = base64_decode(&envelope.ciphertext_b64).unwrap();
        ciphertext[0] ^= 0x80;
        envelope.ciphertext_b64 = base64_encode(&ciphertext);
        let context = context("localhost", &credential_id, &salt, &public_key);
        assert!(decrypt_seed_with_prf(&envelope, prf, &context).await.is_err());
    }

    #[wasm_bindgen_test]
    async fn changed_rp_id_credential_salt_or_identity_key_is_rejected() {
        let (_, prf, salt, public_key, credential_id, envelope) = encrypted_fixture().await;
        let wrong_rp = context("other.localhost", &credential_id, &salt, &public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, prf.clone(), &wrong_rp).await.unwrap_err(),
            PrfEnvelopeAeadError::InvalidEnvelopeMetadata
        );

        let wrong_credential = [9, 8, 7, 6];
        let wrong_id = context("localhost", &wrong_credential, &salt, &public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, prf.clone(), &wrong_id).await.unwrap_err(),
            PrfEnvelopeAeadError::ContextMismatch
        );

        let wrong_salt = [0x25; 32];
        let wrong_salt_ctx = context("localhost", &credential_id, &wrong_salt, &public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, prf.clone(), &wrong_salt_ctx).await.unwrap_err(),
            PrfEnvelopeAeadError::ContextMismatch
        );

        let other_seed = [8u8; 32];
        let wrong_public_key = SigningKey::from_bytes(&other_seed).verifying_key().to_bytes();
        let wrong_key_ctx = context("localhost", &credential_id, &salt, &wrong_public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, prf, &wrong_key_ctx).await.unwrap_err(),
            PrfEnvelopeAeadError::ContextMismatch
        );
    }

    #[wasm_bindgen_test]
    async fn unknown_profile_metadata_is_rejected_before_decrypt() {
        let (_, prf, salt, public_key, credential_id, mut envelope) = encrypted_fixture().await;
        envelope.format_version = PRF_WRAPPED_ENVELOPE_VERSION + 1;
        let context = context("localhost", &credential_id, &salt, &public_key);
        assert_eq!(
            decrypt_seed_with_prf(&envelope, prf, &context).await.unwrap_err(),
            PrfEnvelopeAeadError::InvalidEnvelopeMetadata
        );
    }
}
