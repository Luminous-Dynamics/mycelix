// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Cryptographic services for end-to-end encryption
//!
//! Uses X25519 for key exchange and ChaCha20-Poly1305 for message encryption.
//! This ensures messages are encrypted on the client side before being stored
//! on the DHT, providing true end-to-end privacy.

use base64::{
    engine::general_purpose::{STANDARD as BASE64, STANDARD_NO_PAD},
    Engine as _,
};
use chacha20poly1305::{
    aead::{Aead, KeyInit, OsRng},
    ChaCha20Poly1305, Nonce,
};
use rand::RngCore;
use hkdf::Hkdf;
use sha2::Sha256;
use x25519_dalek::{EphemeralSecret, PublicKey, StaticSecret};
use pqcrypto_kyber::kyber1024;
use pqcrypto_dilithium::dilithium3;
use pqcrypto_traits::kem::{PublicKey as KemPublicKey, SecretKey as KemSecretKey, SharedSecret as KemSharedSecret, Ciphertext as KemCiphertext};
use pqcrypto_traits::sign::{PublicKey as SignPublicKey, SecretKey as SignSecretKey, SignedMessage, DetachedSignature};

/// HKDF info string for email encryption key derivation
const HKDF_INFO_EMAIL: &[u8] = b"mycelix-mail-v1-email-encryption";
/// HKDF info string for hybrid PQC key derivation
const HKDF_INFO_HYBRID: &[u8] = b"mycelix-mail-v1-hybrid-pqc";

use crate::error::{AppError, AppResult};

/// Encrypted message envelope
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EncryptedEnvelope {
    /// Ephemeral public key used for this message (32 bytes, base64)
    pub ephemeral_pubkey: String,
    /// Nonce used for encryption (12 bytes, base64)
    pub nonce: String,
    /// Encrypted ciphertext (base64)
    pub ciphertext: String,
}

/// Key pair for encryption operations
pub struct KeyPair {
    pub secret: StaticSecret,
    pub public: PublicKey,
}

impl KeyPair {
    /// Generate a new random key pair
    pub fn generate() -> Self {
        let secret = StaticSecret::random_from_rng(OsRng);
        let public = PublicKey::from(&secret);
        Self { secret, public }
    }

    /// Create from existing secret bytes
    pub fn from_secret_bytes(bytes: [u8; 32]) -> Self {
        let secret = StaticSecret::from(bytes);
        let public = PublicKey::from(&secret);
        Self { secret, public }
    }

    /// Get public key as bytes
    pub fn public_bytes(&self) -> [u8; 32] {
        self.public.to_bytes()
    }
}

/// Kyber1024 key pair for post-quantum key encapsulation
pub struct KyberKeyPair {
    pub public: kyber1024::PublicKey,
    pub secret: kyber1024::SecretKey,
}

impl KyberKeyPair {
    /// Generate a new Kyber1024 key pair
    pub fn generate() -> Self {
        let (pk, sk) = kyber1024::keypair();
        Self { public: pk, secret: sk }
    }

    /// Get public key bytes
    pub fn public_bytes(&self) -> Vec<u8> {
        self.public.as_bytes().to_vec()
    }
}

/// Dilithium3 key pair for post-quantum signatures
pub struct DilithiumKeyPair {
    pub public: dilithium3::PublicKey,
    pub secret: dilithium3::SecretKey,
}

impl DilithiumKeyPair {
    /// Generate a new Dilithium3 key pair
    pub fn generate() -> Self {
        let (pk, sk) = dilithium3::keypair();
        Self { public: pk, secret: sk }
    }

    /// Get public key bytes
    pub fn public_bytes(&self) -> Vec<u8> {
        self.public.as_bytes().to_vec()
    }

    /// Sign a message
    pub fn sign(&self, message: &[u8]) -> Vec<u8> {
        let sig = dilithium3::detached_sign(message, &self.secret);
        sig.as_bytes().to_vec()
    }
}

/// Verify a Dilithium3 signature
pub fn verify_dilithium3(message: &[u8], signature: &[u8], public_key: &[u8]) -> AppResult<bool> {
    let pk = dilithium3::PublicKey::from_bytes(public_key)
        .map_err(|_| AppError::EncryptionError("Invalid Dilithium3 public key".to_string()))?;
    let sig = dilithium3::DetachedSignature::from_bytes(signature)
        .map_err(|_| AppError::EncryptionError("Invalid Dilithium3 signature".to_string()))?;
    Ok(dilithium3::verify_detached_signature(&sig, message, &pk).is_ok())
}

/// Encrypt a message for a recipient
///
/// Uses X25519 ECDH to derive a shared secret, then encrypts with ChaCha20-Poly1305.
/// The ephemeral public key and nonce are included in the envelope for decryption.
pub fn encrypt_for_recipient(
    plaintext: &[u8],
    recipient_pubkey: &[u8; 32],
) -> AppResult<EncryptedEnvelope> {
    // Generate ephemeral key pair for this message
    let ephemeral_secret = EphemeralSecret::random_from_rng(OsRng);
    let ephemeral_public = PublicKey::from(&ephemeral_secret);

    // Derive shared secret via ECDH
    let recipient_key = PublicKey::from(*recipient_pubkey);
    let shared_secret = ephemeral_secret.diffie_hellman(&recipient_key);

    // Derive encryption key via HKDF-SHA256 (NIST SP 800-56C compliant)
    // Using ephemeral pubkey as salt provides domain separation per message
    let hk = Hkdf::<Sha256>::new(Some(ephemeral_public.as_bytes()), shared_secret.as_bytes());
    let mut key_bytes = [0u8; 32];
    hk.expand(HKDF_INFO_EMAIL, &mut key_bytes)
        .map_err(|_| AppError::EncryptionError("HKDF expand failed".to_string()))?;

    // Create cipher and generate random nonce
    let cipher = ChaCha20Poly1305::new_from_slice(&key_bytes)
        .map_err(|e| AppError::EncryptionError(format!("Failed to create cipher: {}", e)))?;

    let mut nonce_bytes = [0u8; 12];
    OsRng.fill_bytes(&mut nonce_bytes);
    // Safety: verify RNG produced non-zero nonce (catastrophic if reused with same key)
    if nonce_bytes == [0u8; 12] {
        return Err(AppError::EncryptionError(
            "RNG failure: generated all-zero nonce".to_string(),
        ));
    }
    let nonce = Nonce::from_slice(&nonce_bytes);

    // Encrypt
    let ciphertext = cipher
        .encrypt(nonce, plaintext)
        .map_err(|e| AppError::EncryptionError(format!("Encryption failed: {}", e)))?;

    Ok(EncryptedEnvelope {
        ephemeral_pubkey: BASE64.encode(ephemeral_public.as_bytes()),
        nonce: BASE64.encode(nonce_bytes),
        ciphertext: BASE64.encode(ciphertext),
    })
}

/// Decrypt a message using recipient's secret key
pub fn decrypt_envelope(
    envelope: &EncryptedEnvelope,
    recipient_secret: &StaticSecret,
) -> AppResult<Vec<u8>> {
    // Decode ephemeral public key
    let ephemeral_bytes = BASE64
        .decode(&envelope.ephemeral_pubkey)
        .map_err(|_| AppError::EncryptionError("Invalid ephemeral pubkey".to_string()))?;

    if ephemeral_bytes.len() != 32 {
        return Err(AppError::EncryptionError(
            "Invalid ephemeral pubkey length".to_string(),
        ));
    }

    let mut ephemeral_arr = [0u8; 32];
    ephemeral_arr.copy_from_slice(&ephemeral_bytes);
    let ephemeral_pubkey = PublicKey::from(ephemeral_arr);

    // Derive shared secret
    let shared_secret = recipient_secret.diffie_hellman(&ephemeral_pubkey);

    // Derive encryption key via HKDF-SHA256 (must match encrypt_for_recipient)
    let hk = Hkdf::<Sha256>::new(Some(&ephemeral_bytes), shared_secret.as_bytes());
    let mut key_bytes = [0u8; 32];
    hk.expand(HKDF_INFO_EMAIL, &mut key_bytes)
        .map_err(|_| AppError::EncryptionError("HKDF expand failed".to_string()))?;

    // Create cipher
    let cipher = ChaCha20Poly1305::new_from_slice(&key_bytes)
        .map_err(|e| AppError::EncryptionError(format!("Failed to create cipher: {}", e)))?;

    // Decode nonce
    let nonce_bytes = BASE64
        .decode(&envelope.nonce)
        .map_err(|_| AppError::EncryptionError("Invalid nonce".to_string()))?;

    if nonce_bytes.len() != 12 {
        return Err(AppError::EncryptionError(
            "Invalid nonce length".to_string(),
        ));
    }

    let nonce = Nonce::from_slice(&nonce_bytes);

    // Decode ciphertext
    let ciphertext = BASE64
        .decode(&envelope.ciphertext)
        .map_err(|_| AppError::EncryptionError("Invalid ciphertext".to_string()))?;

    // Decrypt
    cipher
        .decrypt(nonce, ciphertext.as_ref())
        .map_err(|e| AppError::EncryptionError(format!("Decryption failed: {}", e)))
}

/// Simple symmetric encryption (for local storage, session data, etc.)
pub fn encrypt_symmetric(plaintext: &[u8], key: &[u8; 32]) -> AppResult<(Vec<u8>, [u8; 12])> {
    let cipher = ChaCha20Poly1305::new_from_slice(key)
        .map_err(|e| AppError::EncryptionError(format!("Failed to create cipher: {}", e)))?;

    let mut nonce_bytes = [0u8; 12];
    OsRng.fill_bytes(&mut nonce_bytes);
    // Safety: verify RNG produced non-zero nonce (catastrophic if reused with same key)
    if nonce_bytes == [0u8; 12] {
        return Err(AppError::EncryptionError(
            "RNG failure: generated all-zero nonce".to_string(),
        ));
    }
    let nonce = Nonce::from_slice(&nonce_bytes);

    let ciphertext = cipher
        .encrypt(nonce, plaintext)
        .map_err(|e| AppError::EncryptionError(format!("Encryption failed: {}", e)))?;

    Ok((ciphertext, nonce_bytes))
}

/// Simple symmetric decryption
pub fn decrypt_symmetric(
    ciphertext: &[u8],
    nonce: &[u8; 12],
    key: &[u8; 32],
) -> AppResult<Vec<u8>> {
    let cipher = ChaCha20Poly1305::new_from_slice(key)
        .map_err(|e| AppError::EncryptionError(format!("Failed to create cipher: {}", e)))?;

    let nonce = Nonce::from_slice(nonce);

    cipher
        .decrypt(nonce, ciphertext)
        .map_err(|e| AppError::EncryptionError(format!("Decryption failed: {}", e)))
}

/// Derive a key from a password using Argon2
pub fn derive_key_from_password(password: &str, salt: &[u8]) -> AppResult<[u8; 32]> {
    use argon2::{password_hash::SaltString, Argon2, PasswordHasher};

    // Create salt string (must be valid base64)
    let salt_b64 = STANDARD_NO_PAD.encode(salt);
    let salt_string = SaltString::from_b64(&salt_b64)
        .map_err(|e| AppError::EncryptionError(format!("Invalid salt: {}", e)))?;

    let argon2 = Argon2::default();
    let hash = argon2
        .hash_password(password.as_bytes(), &salt_string)
        .map_err(|e| AppError::EncryptionError(format!("Key derivation failed: {}", e)))?;

    // Extract 32 bytes from hash output
    let hash_bytes = hash.hash.ok_or_else(|| {
        AppError::EncryptionError("Failed to get hash output".to_string())
    })?;

    let mut key = [0u8; 32];
    key.copy_from_slice(&hash_bytes.as_bytes()[..32]);
    Ok(key)
}

/// Crypto suite selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CryptoSuiteId {
    /// Classical: X25519 key exchange + ChaCha20-Poly1305 + Ed25519 signing
    Classical,
    /// Post-quantum: Kyber1024 KEM + ChaCha20-Poly1305 + Dilithium3 signing
    PostQuantum,
    /// Hybrid: X25519 + Kyber1024 combined, ChaCha20-Poly1305, Dilithium3 signing
    /// Recommended for maximum security (quantum-resistant + classical fallback)
    Hybrid,
}

impl Default for CryptoSuiteId {
    fn default() -> Self {
        // Default to classical until PQC libraries are integrated
        Self::Classical
    }
}

/// PQC key exchange envelope (Kyber1024 KEM)
///
/// NOTE: This is the structural foundation for PQC integration.
/// Full Kyber1024/Dilithium3 operations require the `pqcrypto-kyber` and
/// `pqcrypto-dilithium` crates. When those are added to Cargo.toml,
/// replace the placeholder implementations below with real PQC operations.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PqcEncryptedEnvelope {
    /// Kyber1024 ciphertext (for key encapsulation)
    pub kyber_ciphertext: String,
    /// Classical X25519 ephemeral pubkey (for hybrid mode)
    pub classical_ephemeral_pubkey: Option<String>,
    /// ChaCha20-Poly1305 nonce (12 bytes, base64)
    pub nonce: String,
    /// Encrypted ciphertext (base64)
    pub ciphertext: String,
    /// Dilithium3 signature over (ciphertext || nonce) (base64)
    pub signature: String,
    /// Which suite was used
    pub suite: CryptoSuiteId,
}

/// Hybrid encrypt: combines X25519 shared secret with Kyber1024 shared secret.
///
/// The two shared secrets are concatenated and hashed with SHA-256 to produce
/// the final symmetric key. This ensures security if either algorithm is broken
/// (belt-and-suspenders approach per NIST SP 800-227 draft).
///
/// Currently uses classical-only as PQC crates are not yet in Cargo.toml.
/// When `pqcrypto-kyber` is available, this will perform real KEM.
pub fn encrypt_hybrid(
    plaintext: &[u8],
    recipient_x25519_pubkey: &[u8; 32],
    _recipient_kyber_pubkey: Option<&[u8]>,
) -> AppResult<PqcEncryptedEnvelope> {
    // Classical component: X25519 ECDH
    let ephemeral_secret = EphemeralSecret::random_from_rng(OsRng);
    let ephemeral_public = PublicKey::from(&ephemeral_secret);
    let recipient_key = PublicKey::from(*recipient_x25519_pubkey);
    let classical_shared = ephemeral_secret.diffie_hellman(&recipient_key);

    // PQC component: Kyber1024 Key Encapsulation Mechanism
    let (kyber_shared, kyber_ct) = if let Some(kyber_pk_bytes) = _recipient_kyber_pubkey {
        let kyber_pk = kyber1024::PublicKey::from_bytes(kyber_pk_bytes)
            .map_err(|_| AppError::EncryptionError("Invalid Kyber1024 public key".to_string()))?;
        let (ss, ct) = kyber1024::encapsulate(&kyber_pk);
        (Some(ss), KemCiphertext::as_bytes(&ct).to_vec())
    } else {
        (None, vec![])
    };

    // Derive key via HKDF-SHA256 from combined shared secrets.
    // In hybrid mode, both X25519 and Kyber secrets contribute to the IKM.
    // Currently classical-only until pqcrypto crates are added.
    let mut ikm = Vec::with_capacity(64);
    ikm.extend_from_slice(classical_shared.as_bytes());
    if let Some(ref ks) = kyber_shared {
        ikm.extend_from_slice(KemSharedSecret::as_bytes(ks));
    }
    let hk = Hkdf::<Sha256>::new(Some(ephemeral_public.as_bytes()), &ikm);
    let mut key_bytes = [0u8; 32];
    hk.expand(HKDF_INFO_HYBRID, &mut key_bytes)
        .map_err(|_| AppError::EncryptionError("HKDF expand failed".to_string()))?;

    // Encrypt with ChaCha20-Poly1305
    let cipher = ChaCha20Poly1305::new_from_slice(&key_bytes)
        .map_err(|e| AppError::EncryptionError(format!("Cipher init failed: {}", e)))?;

    let mut nonce_bytes = [0u8; 12];
    OsRng.fill_bytes(&mut nonce_bytes);
    // Safety: verify RNG produced non-zero nonce (catastrophic if reused with same key)
    if nonce_bytes == [0u8; 12] {
        return Err(AppError::EncryptionError(
            "RNG failure: generated all-zero nonce".to_string(),
        ));
    }
    let nonce = Nonce::from_slice(&nonce_bytes);

    let ciphertext = cipher
        .encrypt(nonce, plaintext)
        .map_err(|e| AppError::EncryptionError(format!("Encryption failed: {}", e)))?;

    // Signature is computed by the caller using their DilithiumKeyPair.sign()
    // The envelope carries the signature for verification by the recipient
    let signature_bytes = vec![]; // Caller must set this after creation

    Ok(PqcEncryptedEnvelope {
        kyber_ciphertext: BASE64.encode(&kyber_ct),
        classical_ephemeral_pubkey: Some(BASE64.encode(ephemeral_public.as_bytes())),
        nonce: BASE64.encode(nonce_bytes),
        ciphertext: BASE64.encode(&ciphertext),
        signature: BASE64.encode(&signature_bytes),
        suite: if kyber_shared.is_some() { CryptoSuiteId::Hybrid } else { CryptoSuiteId::Classical },
    })
}

/// Decrypt a hybrid/PQC envelope
pub fn decrypt_hybrid(
    envelope: &PqcEncryptedEnvelope,
    recipient_x25519_secret: &StaticSecret,
    _recipient_kyber_secret: Option<&[u8]>,
) -> AppResult<Vec<u8>> {
    // Classical component
    let ephemeral_pubkey_b64 = envelope.classical_ephemeral_pubkey.as_ref()
        .ok_or_else(|| AppError::EncryptionError("Missing classical ephemeral pubkey".into()))?;

    let ephemeral_bytes = BASE64.decode(ephemeral_pubkey_b64)
        .map_err(|_| AppError::EncryptionError("Invalid ephemeral pubkey base64".into()))?;

    if ephemeral_bytes.len() != 32 {
        return Err(AppError::EncryptionError("Ephemeral pubkey must be 32 bytes".into()));
    }

    let mut arr = [0u8; 32];
    arr.copy_from_slice(&ephemeral_bytes);
    let ephemeral_pubkey = PublicKey::from(arr);
    let classical_shared = recipient_x25519_secret.diffie_hellman(&ephemeral_pubkey);

    // Combine classical + PQC shared secrets for HKDF input
    let mut ikm = Vec::with_capacity(64);
    ikm.extend_from_slice(classical_shared.as_bytes());

    // Kyber decapsulation if PQC ciphertext is present
    if let Some(kyber_sk_bytes) = _recipient_kyber_secret {
        let kyber_ct_bytes = BASE64.decode(&envelope.kyber_ciphertext)
            .map_err(|_| AppError::EncryptionError("Invalid Kyber ciphertext base64".into()))?;
        if !kyber_ct_bytes.is_empty() {
            let kyber_sk = kyber1024::SecretKey::from_bytes(kyber_sk_bytes)
                .map_err(|_| AppError::EncryptionError("Invalid Kyber1024 secret key".into()))?;
            let kyber_ct = kyber1024::Ciphertext::from_bytes(&kyber_ct_bytes)
                .map_err(|_| AppError::EncryptionError("Invalid Kyber1024 ciphertext".into()))?;
            let kyber_shared = kyber1024::decapsulate(&kyber_ct, &kyber_sk);
            ikm.extend_from_slice(KemSharedSecret::as_bytes(&kyber_shared));
        }
    }

    let hk = Hkdf::<Sha256>::new(Some(&ephemeral_bytes), &ikm);
    let mut key_bytes = [0u8; 32];
    hk.expand(HKDF_INFO_HYBRID, &mut key_bytes)
        .map_err(|_| AppError::EncryptionError("HKDF expand failed".to_string()))?;

    // Decrypt
    let cipher = ChaCha20Poly1305::new_from_slice(&key_bytes)
        .map_err(|e| AppError::EncryptionError(format!("Cipher init failed: {}", e)))?;

    let nonce_bytes = BASE64.decode(&envelope.nonce)
        .map_err(|_| AppError::EncryptionError("Invalid nonce base64".into()))?;
    if nonce_bytes.len() != 12 {
        return Err(AppError::EncryptionError("Nonce must be 12 bytes".into()));
    }
    let nonce = Nonce::from_slice(&nonce_bytes);

    let ciphertext = BASE64.decode(&envelope.ciphertext)
        .map_err(|_| AppError::EncryptionError("Invalid ciphertext base64".into()))?;

    cipher.decrypt(nonce, ciphertext.as_ref())
        .map_err(|e| AppError::EncryptionError(format!("Decryption failed: {}", e)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encrypt_decrypt_roundtrip() {
        let plaintext = b"Hello, Mycelix!";

        // Generate recipient key pair
        let recipient = KeyPair::generate();

        // Encrypt for recipient
        let envelope = encrypt_for_recipient(plaintext, &recipient.public_bytes()).unwrap();

        // Decrypt with recipient's secret
        let decrypted = decrypt_envelope(&envelope, &recipient.secret).unwrap();

        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_symmetric_roundtrip() {
        let plaintext = b"Secret message";
        let key = [42u8; 32];

        let (ciphertext, nonce) = encrypt_symmetric(plaintext, &key).unwrap();
        let decrypted = decrypt_symmetric(&ciphertext, &nonce, &key).unwrap();

        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_wrong_key_fails() {
        let plaintext = b"Hello";

        let recipient1 = KeyPair::generate();
        let recipient2 = KeyPair::generate();

        // Encrypt for recipient1
        let envelope = encrypt_for_recipient(plaintext, &recipient1.public_bytes()).unwrap();

        // Try to decrypt with recipient2's key - should fail
        let result = decrypt_envelope(&envelope, &recipient2.secret);
        assert!(result.is_err());
    }

    #[test]
    fn test_hybrid_encrypt_decrypt_roundtrip() {
        let plaintext = b"Post-quantum secure message!";
        let recipient = KeyPair::generate();

        let envelope = encrypt_hybrid(plaintext, &recipient.public_bytes(), None).unwrap();
        assert_eq!(envelope.suite, CryptoSuiteId::Classical); // Until PQC crates added

        let decrypted = decrypt_hybrid(&envelope, &recipient.secret, None).unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_hybrid_wrong_key_fails() {
        let plaintext = b"Secret";
        let recipient1 = KeyPair::generate();
        let recipient2 = KeyPair::generate();

        let envelope = encrypt_hybrid(plaintext, &recipient1.public_bytes(), None).unwrap();
        let result = decrypt_hybrid(&envelope, &recipient2.secret, None);
        assert!(result.is_err());
    }

    #[test]
    fn test_crypto_suite_default_is_classical() {
        assert_eq!(CryptoSuiteId::default(), CryptoSuiteId::Classical);
    }

    #[test]
    fn test_empty_plaintext_roundtrip() {
        let plaintext = b"";
        let recipient = KeyPair::generate();
        let envelope = encrypt_for_recipient(plaintext, &recipient.public_bytes()).unwrap();
        let decrypted = decrypt_envelope(&envelope, &recipient.secret).unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_large_plaintext_roundtrip() {
        let plaintext = vec![0xAB; 10 * 1024 * 1024]; // 10 MB
        let recipient = KeyPair::generate();
        let envelope = encrypt_for_recipient(&plaintext, &recipient.public_bytes()).unwrap();
        let decrypted = decrypt_envelope(&envelope, &recipient.secret).unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_invalid_nonce_length_fails() {
        let recipient = KeyPair::generate();
        let envelope = EncryptedEnvelope {
            ephemeral_pubkey: BASE64.encode(recipient.public_bytes()),
            nonce: BASE64.encode([0u8; 5]), // Wrong length
            ciphertext: BASE64.encode(b"garbage"),
        };
        assert!(decrypt_envelope(&envelope, &recipient.secret).is_err());
    }

    #[test]
    fn test_invalid_ephemeral_key_length_fails() {
        let recipient = KeyPair::generate();
        let envelope = EncryptedEnvelope {
            ephemeral_pubkey: BASE64.encode([0u8; 16]), // Wrong length
            nonce: BASE64.encode([0u8; 12]),
            ciphertext: BASE64.encode(b"garbage"),
        };
        assert!(decrypt_envelope(&envelope, &recipient.secret).is_err());
    }

    #[test]
    fn test_symmetric_wrong_key_fails() {
        let plaintext = b"Secret";
        let key1 = [1u8; 32];
        let key2 = [2u8; 32];
        let (ciphertext, nonce) = encrypt_symmetric(plaintext, &key1).unwrap();
        assert!(decrypt_symmetric(&ciphertext, &nonce, &key2).is_err());
    }

    #[test]
    fn test_key_derivation_produces_32_bytes() {
        let key = derive_key_from_password("test_password", b"test_salt_16bytes").unwrap();
        assert_eq!(key.len(), 32);
    }

    #[test]
    fn test_different_passwords_different_keys() {
        let salt = b"same_salt_16bytes";
        let key1 = derive_key_from_password("password1", salt).unwrap();
        let key2 = derive_key_from_password("password2", salt).unwrap();
        assert_ne!(key1, key2);
    }

    #[test]
    fn test_kyber_keypair_generation() {
        let kp = KyberKeyPair::generate();
        // Kyber1024 public key is 1568 bytes
        assert_eq!(kp.public_bytes().len(), 1568);
    }

    #[test]
    fn test_dilithium_keypair_and_sign_verify() {
        let kp = DilithiumKeyPair::generate();
        let message = b"Test message for Dilithium signing";
        let signature = kp.sign(message);
        // Dilithium3 signature is 3293 bytes
        assert_eq!(signature.len(), 3293);

        let valid = verify_dilithium3(message, &signature, &kp.public_bytes()).unwrap();
        assert!(valid, "Valid signature must verify");

        // Wrong message should fail
        let invalid = verify_dilithium3(b"wrong message", &signature, &kp.public_bytes()).unwrap();
        assert!(!invalid, "Wrong message must fail verification");
    }

    #[test]
    fn test_hybrid_encrypt_decrypt_with_kyber() {
        let x25519_recipient = KeyPair::generate();
        let kyber_recipient = KyberKeyPair::generate();
        let plaintext = b"Hybrid PQC encrypted message!";

        let envelope = encrypt_hybrid(
            plaintext,
            &x25519_recipient.public_bytes(),
            Some(&kyber_recipient.public_bytes()),
        ).unwrap();

        assert_eq!(envelope.suite, CryptoSuiteId::Hybrid);
        assert!(!envelope.kyber_ciphertext.is_empty());

        let decrypted = decrypt_hybrid(
            &envelope,
            &x25519_recipient.secret,
            Some(KemSecretKey::as_bytes(&kyber_recipient.secret)),
        ).unwrap();

        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_hybrid_classical_only_still_works() {
        let x25519_recipient = KeyPair::generate();
        let plaintext = b"Classical-only hybrid message";

        let envelope = encrypt_hybrid(plaintext, &x25519_recipient.public_bytes(), None).unwrap();
        assert_eq!(envelope.suite, CryptoSuiteId::Classical);

        let decrypted = decrypt_hybrid(&envelope, &x25519_recipient.secret, None).unwrap();
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_dilithium_wrong_key_fails() {
        let kp1 = DilithiumKeyPair::generate();
        let kp2 = DilithiumKeyPair::generate();
        let message = b"Signed by kp1";
        let signature = kp1.sign(message);

        // Verify with wrong public key
        let result = verify_dilithium3(message, &signature, &kp2.public_bytes()).unwrap();
        assert!(!result, "Wrong public key must fail");
    }
}
