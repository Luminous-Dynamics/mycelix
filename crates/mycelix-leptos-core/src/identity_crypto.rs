// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Passphrase-based encryption for secrets kept in the browser at rest.
//!
//! Backs step 2 of [`local_identity`](crate::local_identity)'s recoverable-
//! identity roadmap: wrapping the local Ed25519 secret key with a
//! passphrase-derived key before it touches `localStorage`, instead of
//! storing the raw key bytes.
//!
//! Uses the real Web Crypto API (`SubtleCrypto`), not a hand-rolled or
//! JS-`eval`'d KDF/cipher:
//! - **PBKDF2-HMAC-SHA256**, [`PBKDF2_ITERATIONS`] rounds — OWASP's 2023
//!   minimum recommendation for PBKDF2-SHA256.
//! - **AES-256-GCM** for the actual secret encryption (authenticated —
//!   tampering or a wrong key/passphrase both fail closed).
//!
//! A wrong passphrase and a corrupted ciphertext are deliberately
//! indistinguishable from the caller's perspective ([`decrypt`] returns the
//! same generic error either way) — that's what `AES-GCM` gives you for
//! free, and collapsing the distinction avoids leaking a passphrase-guessing
//! oracle.

use js_sys::Uint8Array;
use wasm_bindgen::{JsCast, JsValue};
use wasm_bindgen_futures::JsFuture;
use web_sys::{AesDerivedKeyParams, AesGcmParams, CryptoKey, Pbkdf2Params, SubtleCrypto};

/// OWASP's 2023 minimum recommendation for PBKDF2-HMAC-SHA256.
///
/// Deliberately much lower under `cfg(test)`: `cargo test`/`wasm-pack test`
/// never builds with `cfg(test)` unset, so this never affects production —
/// but at the real iteration count, this module's browser test suite (each
/// test does 1-2 full PBKDF2 derivations) is slow enough to risk tripping
/// the headless-browser test runner's timeout. The security-relevant
/// property under test is "does the derived key round-trip / reject a wrong
/// passphrase / reject tampering", which doesn't depend on the iteration
/// count.
#[cfg(not(test))]
pub const PBKDF2_ITERATIONS: u32 = 600_000;
#[cfg(test)]
pub const PBKDF2_ITERATIONS: u32 = 1_000;
/// Random salt length for PBKDF2, in bytes.
pub const SALT_LEN: usize = 16;
/// AES-GCM's standard (and most efficient) nonce length, in bytes.
pub const IV_LEN: usize = 12;

/// A passphrase-encrypted secret, ready to persist.
///
/// `salt` and `iv` are not secret — they just need to be unique per
/// encryption and are stored alongside `ciphertext`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EncryptedBlob {
    pub salt: Vec<u8>,
    pub iv: Vec<u8>,
    /// AES-GCM ciphertext, with the authentication tag appended (this is
    /// `SubtleCrypto.encrypt`'s standard output shape for AES-GCM).
    pub ciphertext: Vec<u8>,
}

fn subtle() -> Result<SubtleCrypto, String> {
    Ok(web_sys::window()
        .ok_or("no global `window` (not running in a browser)")?
        .crypto()
        .map_err(|_| "Web Crypto API unavailable".to_string())?
        .subtle())
}

fn random_bytes(len: usize) -> Result<Vec<u8>, String> {
    let mut buf = vec![0u8; len];
    web_sys::window()
        .ok_or("no global `window` (not running in a browser)")?
        .crypto()
        .map_err(|_| "Web Crypto API unavailable".to_string())?
        .get_random_values_with_u8_array(&mut buf)
        .map_err(|e| format!("crypto.getRandomValues failed: {e:?}"))?;
    Ok(buf)
}

/// PBKDF2(passphrase, salt) -> non-extractable AES-256-GCM `CryptoKey`.
///
/// Non-extractable by design: once derived, the raw AES key bytes never
/// need to leave `SubtleCrypto`'s internal (non-JS-readable) key storage.
async fn derive_aes_key(passphrase: &str, salt: &[u8]) -> Result<CryptoKey, String> {
    let subtle = subtle()?;

    let key_material = Uint8Array::from(passphrase.as_bytes());
    let derive_key_usage = js_sys::Array::of1(&JsValue::from_str("deriveKey"));
    let import_promise = subtle
        .import_key_with_str("raw", &key_material, "PBKDF2", false, &derive_key_usage)
        .map_err(|e| format!("importKey failed: {e:?}"))?;
    let base_key: CryptoKey = JsFuture::from(import_promise)
        .await
        .map_err(|e| format!("importKey rejected: {e:?}"))?
        .unchecked_into();

    let pbkdf2_params = Pbkdf2Params::new(
        "PBKDF2",
        &JsValue::from_str("SHA-256"),
        PBKDF2_ITERATIONS,
        &Uint8Array::from(salt),
    );
    let derived_key_type = AesDerivedKeyParams::new("AES-GCM", 256);
    let encrypt_decrypt_usages =
        js_sys::Array::of2(&JsValue::from_str("encrypt"), &JsValue::from_str("decrypt"));

    let derive_promise = subtle
        .derive_key_with_object_and_object(
            &pbkdf2_params,
            &base_key,
            &derived_key_type,
            false,
            &encrypt_decrypt_usages,
        )
        .map_err(|e| format!("deriveKey failed: {e:?}"))?;

    Ok(JsFuture::from(derive_promise)
        .await
        .map_err(|e| format!("deriveKey rejected: {e:?}"))?
        .unchecked_into())
}

/// Encrypt `plaintext` under a key derived from `passphrase`.
///
/// Generates a fresh random salt and IV each call — safe to call repeatedly
/// with the same passphrase for different secrets, or to re-encrypt the
/// same secret (e.g. on passphrase change).
pub async fn encrypt(passphrase: &str, plaintext: &[u8]) -> Result<EncryptedBlob, String> {
    let salt = random_bytes(SALT_LEN)?;
    let mut iv = random_bytes(IV_LEN)?;
    let aes_key = derive_aes_key(passphrase, &salt).await?;

    let subtle = subtle()?;
    let gcm_params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut iv);
    let promise = subtle
        .encrypt_with_object_and_u8_array(&gcm_params, &aes_key, plaintext)
        .map_err(|e| format!("encrypt failed: {e:?}"))?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|e| format!("encrypt rejected: {e:?}"))?;
    let ciphertext = Uint8Array::new(&result).to_vec();

    Ok(EncryptedBlob {
        salt,
        iv,
        ciphertext,
    })
}

/// Decrypt a blob produced by [`encrypt`] with `passphrase`.
///
/// Fails closed with a generic error on a wrong passphrase *or* a corrupted
/// blob — the two aren't distinguished on purpose (see module docs).
pub async fn decrypt(passphrase: &str, blob: &EncryptedBlob) -> Result<Vec<u8>, String> {
    let aes_key = derive_aes_key(passphrase, &blob.salt).await?;

    let subtle = subtle()?;
    let mut iv = blob.iv.clone();
    let gcm_params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut iv);
    let promise = subtle
        .decrypt_with_object_and_u8_array(&gcm_params, &aes_key, &blob.ciphertext)
        .map_err(|_| "Decryption failed (wrong passphrase?)".to_string())?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|_| "Decryption failed (wrong passphrase?)".to_string())?;

    Ok(Uint8Array::new(&result).to_vec())
}

#[cfg(test)]
mod tests {
    use super::*;

    wasm_bindgen_test::wasm_bindgen_test_configure!(run_in_browser);

    #[wasm_bindgen_test::wasm_bindgen_test]
    async fn round_trips_with_the_correct_passphrase() {
        let secret = b"32-byte-ed25519-secret-key-seed";
        let blob = encrypt("correct horse battery staple", secret)
            .await
            .expect("encrypt should succeed");

        let recovered = decrypt("correct horse battery staple", &blob)
            .await
            .expect("decrypt with the right passphrase should succeed");

        assert_eq!(recovered, secret);
    }

    #[wasm_bindgen_test::wasm_bindgen_test]
    async fn rejects_the_wrong_passphrase() {
        let secret = b"32-byte-ed25519-secret-key-seed";
        let blob = encrypt("correct horse battery staple", secret)
            .await
            .expect("encrypt should succeed");

        let result = decrypt("wrong passphrase entirely", &blob).await;

        assert!(result.is_err(), "wrong passphrase must not decrypt");
    }

    #[wasm_bindgen_test::wasm_bindgen_test]
    async fn rejects_a_tampered_ciphertext() {
        let secret = b"32-byte-ed25519-secret-key-seed";
        let mut blob = encrypt("correct horse battery staple", secret)
            .await
            .expect("encrypt should succeed");
        // Flip a bit — AES-GCM's authentication tag must catch this even
        // though the passphrase (and thus the derived key) is correct.
        blob.ciphertext[0] ^= 0xFF;

        let result = decrypt("correct horse battery staple", &blob).await;

        assert!(result.is_err(), "tampered ciphertext must fail auth");
    }

    #[wasm_bindgen_test::wasm_bindgen_test]
    async fn each_encryption_uses_a_fresh_salt_and_iv() {
        let secret = b"32-byte-ed25519-secret-key-seed";
        let blob_a = encrypt("same passphrase", secret)
            .await
            .expect("encrypt should succeed");
        let blob_b = encrypt("same passphrase", secret)
            .await
            .expect("encrypt should succeed");

        assert_ne!(blob_a.salt, blob_b.salt, "salts must not repeat");
        assert_ne!(blob_a.iv, blob_b.iv, "IVs must not repeat");
        assert_ne!(
            blob_a.ciphertext, blob_b.ciphertext,
            "ciphertext must differ when salt/IV differ, even for identical plaintext"
        );
        // (Round-trip correctness under a matching passphrase is already
        // covered by `round_trips_with_the_correct_passphrase` — not
        // re-checked here, to avoid running 4 PBKDF2 derivations worth of
        // work in one headless-browser test.)
    }
}
