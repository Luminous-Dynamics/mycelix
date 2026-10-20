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
//! - **PBKDF2-HMAC-SHA256**, [`PBKDF2_ITERATIONS`] rounds — currently
//!   aligned with OWASP's 600,000-iteration PBKDF2-SHA256 floor for FIPS-oriented
//!   deployments. OWASP prefers Argon2id when compatible; this module uses Web
//!   Crypto's native PBKDF2 primitive and treats an Argon2id migration as a
//!   separately benchmarked/versioned design decision.
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

/// Production PBKDF2-HMAC-SHA256 floor, aligned with current OWASP guidance
/// for FIPS-oriented deployments. This is not a claim that PBKDF2 is the
/// strongest available KDF for every deployment.
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
/// The frozen iteration count used to interpret legacy three-field envelopes.
/// AES-GCM's standard nonce length, in bytes.
pub const IV_LEN: usize = 12;

pub const LEGACY_PBKDF2_ITERATIONS: u32 = 600_000;
/// New envelopes explicitly identify the format and cryptographic parameters.
pub const CURRENT_ENVELOPE_VERSION: u8 = 2;
/// Upper bound for attacker-controlled envelope work factors.
///
/// Keep the initial ceiling close to the production floor so a corrupted local
/// envelope cannot impose an excessive synchronous-looking wait on the user.
/// Raising this cap requires reference-device latency data and a reviewed format
/// policy change; it must not be increased merely to make an outlier pass.
pub const MAX_PBKDF2_ITERATIONS: u32 = 1_200_000;
pub const KDF_IDENTIFIER: &str = "PBKDF2-HMAC-SHA256";
pub const CIPHER_IDENTIFIER: &str = "AES-256-GCM";

/// Authenticated key-derivation policy recorded in a versioned envelope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyDerivationFunction {
    Pbkdf2HmacSha256,
}

impl KeyDerivationFunction {
    pub fn identifier(self) -> &'static str {
        match self {
            Self::Pbkdf2HmacSha256 => KDF_IDENTIFIER,
        }
    }

    pub fn from_identifier(identifier: &str) -> Option<Self> {
        match identifier {
            KDF_IDENTIFIER => Some(Self::Pbkdf2HmacSha256),
            _ => None,
        }
    }
}

/// Authenticated cipher policy recorded in a versioned envelope.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IdentityCipher {
    Aes256Gcm,
}

impl IdentityCipher {
    pub fn identifier(self) -> &'static str {
        match self {
            Self::Aes256Gcm => CIPHER_IDENTIFIER,
        }
    }

    pub fn from_identifier(identifier: &str) -> Option<Self> {
        match identifier {
            CIPHER_IDENTIFIER => Some(Self::Aes256Gcm),
            _ => None,
        }
    }
}

/// A passphrase-encrypted secret, ready to persist.
///
/// The v2 header is authenticated as AES-GCM additional authenticated data.
/// v1 is reserved for legacy envelopes that had no header and used the
/// frozen legacy PBKDF2 work factor.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EncryptedBlob {
    pub format_version: u8,
    pub kdf: KeyDerivationFunction,
    pub iterations: u32,
    pub cipher: IdentityCipher,
    pub salt: Vec<u8>,
    pub iv: Vec<u8>,
    /// AES-GCM ciphertext, with the authentication tag appended.
    pub ciphertext: Vec<u8>,
}

fn minimum_supported_iterations() -> u32 {
    #[cfg(test)]
    {
        1_000
    }
    #[cfg(not(test))]
    {
        LEGACY_PBKDF2_ITERATIONS
    }
}

fn canonical_authenticated_header(
    format_version: u8,
    kdf: KeyDerivationFunction,
    iterations: u32,
    cipher: IdentityCipher,
) -> Vec<u8> {
    format!(
        "mycelix-identity-envelope|version={format_version}|kdf={}|iterations={iterations}|cipher={}",
        kdf.identifier(),
        cipher.identifier()
    )
    .into_bytes()
}

impl EncryptedBlob {
    /// Validate all untrusted envelope parameters before doing expensive work.
    pub fn validate(&self) -> Result<(), String> {
        if self.salt.len() != SALT_LEN {
            return Err("Invalid encrypted identity salt length.".into());
        }
        if self.iv.len() != IV_LEN {
            return Err("Invalid encrypted identity nonce length.".into());
        }
        if self.ciphertext.len() < 16 {
            return Err("Encrypted identity payload is truncated.".into());
        }

        match self.format_version {
            1 => {
                if self.iterations != LEGACY_PBKDF2_ITERATIONS {
                    return Err("Unsupported legacy identity work factor.".into());
                }
            }
            CURRENT_ENVELOPE_VERSION => {
                if !(minimum_supported_iterations()..=MAX_PBKDF2_ITERATIONS)
                    .contains(&self.iterations)
                {
                    return Err("Encrypted identity work factor is outside policy limits.".into());
                }
            }
            _ => return Err("Unsupported encrypted identity envelope version.".into()),
        }
        if self.kdf != KeyDerivationFunction::Pbkdf2HmacSha256 {
            return Err("Unsupported encrypted identity KDF.".into());
        }
        if self.cipher != IdentityCipher::Aes256Gcm {
            return Err("Unsupported encrypted identity cipher.".into());
        }
        Ok(())
    }

    fn authenticated_header(&self) -> Option<Vec<u8>> {
        (self.format_version >= CURRENT_ENVELOPE_VERSION).then(|| {
            canonical_authenticated_header(
                self.format_version,
                self.kdf,
                self.iterations,
                self.cipher,
            )
        })
    }
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
async fn derive_aes_key(
    passphrase: &str,
    salt: &[u8],
    iterations: u32,
) -> Result<CryptoKey, String> {
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
        iterations,
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
    let iterations = PBKDF2_ITERATIONS;
    let kdf = KeyDerivationFunction::Pbkdf2HmacSha256;
    let cipher = IdentityCipher::Aes256Gcm;
    let aes_key = derive_aes_key(passphrase, &salt, iterations).await?;

    let mut additional_data =
        canonical_authenticated_header(CURRENT_ENVELOPE_VERSION, kdf, iterations, cipher);
    let subtle = subtle()?;
    let gcm_params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut iv);
    gcm_params.set_additional_data_u8_slice(&mut additional_data);
    let promise = subtle
        .encrypt_with_object_and_u8_array(&gcm_params, &aes_key, plaintext)
        .map_err(|e| format!("encrypt failed: {e:?}"))?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|e| format!("encrypt rejected: {e:?}"))?;
    let ciphertext = Uint8Array::new(&result).to_vec();

    Ok(EncryptedBlob {
        format_version: CURRENT_ENVELOPE_VERSION,
        kdf,
        iterations,
        cipher,
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
    blob.validate()?;
    let aes_key = derive_aes_key(passphrase, &blob.salt, blob.iterations).await?;

    let subtle = subtle()?;
    let mut iv = blob.iv.clone();
    let gcm_params = AesGcmParams::new_with_u8_slice("AES-GCM", &mut iv);
    if let Some(mut additional_data) = blob.authenticated_header() {
        gcm_params.set_additional_data_u8_slice(&mut additional_data);
    }
    let promise = subtle
        .decrypt_with_object_and_u8_array(&gcm_params, &aes_key, &blob.ciphertext)
        .map_err(|_| "Decryption failed (wrong passphrase?)".to_string())?;
    let result = JsFuture::from(promise)
        .await
        .map_err(|_| "Decryption failed (wrong passphrase?)".to_string())?;

    Ok(Uint8Array::new(&result).to_vec())
}

#[cfg(all(test, target_arch = "wasm32"))]
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
    async fn rejects_modified_kdf_metadata() {
        let secret = b"32-byte-ed25519-secret-key-seed";
        let mut blob = encrypt("metadata passphrase", secret)
            .await
            .expect("encrypt should succeed");
        blob.iterations += 1;

        let result = decrypt("metadata passphrase", &blob).await;
        assert!(
            result.is_err(),
            "altered KDF metadata must fail authenticated decryption"
        );
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

#[cfg(test)]
mod envelope_policy_tests {
    use super::*;

    fn sample_blob(format_version: u8, iterations: u32) -> EncryptedBlob {
        EncryptedBlob {
            format_version,
            kdf: KeyDerivationFunction::Pbkdf2HmacSha256,
            iterations,
            cipher: IdentityCipher::Aes256Gcm,
            salt: vec![1; SALT_LEN],
            iv: vec![2; IV_LEN],
            ciphertext: vec![3; 48],
        }
    }

    #[test]
    fn legacy_work_factor_is_frozen_independently_of_test_builds() {
        assert_eq!(LEGACY_PBKDF2_ITERATIONS, 600_000);
        assert!(sample_blob(1, LEGACY_PBKDF2_ITERATIONS).validate().is_ok());
        assert!(sample_blob(1, PBKDF2_ITERATIONS).validate().is_err());
    }

    #[test]
    fn new_envelopes_bound_iterations_before_kdf_work() {
        assert!(
            sample_blob(CURRENT_ENVELOPE_VERSION, PBKDF2_ITERATIONS)
                .validate()
                .is_ok()
        );
        assert!(
            sample_blob(CURRENT_ENVELOPE_VERSION, 999)
                .validate()
                .is_err()
        );
        assert!(
            sample_blob(CURRENT_ENVELOPE_VERSION, MAX_PBKDF2_ITERATIONS + 1)
                .validate()
                .is_err()
        );
        assert!(sample_blob(99, PBKDF2_ITERATIONS).validate().is_err());
    }

    #[test]
    fn authenticated_header_changes_when_kdf_parameters_change() {
        let blob = sample_blob(CURRENT_ENVELOPE_VERSION, PBKDF2_ITERATIONS);
        let first = blob.authenticated_header().unwrap();
        let changed = sample_blob(CURRENT_ENVELOPE_VERSION, PBKDF2_ITERATIONS + 1)
            .authenticated_header()
            .unwrap();
        assert_ne!(first, changed);
        assert!(String::from_utf8(first).unwrap().contains(KDF_IDENTIFIER));
        assert!(
            String::from_utf8(changed)
                .unwrap()
                .contains(CIPHER_IDENTIFIER)
        );
    }
}
