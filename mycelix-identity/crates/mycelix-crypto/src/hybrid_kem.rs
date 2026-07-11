// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Hybrid X25519 + ML-KEM-768 key encapsulation with XChaCha20-Poly1305 AEAD.
//!
//! Pure-Rust (RustCrypto) counterpart to the `pqc` modules, which use
//! `pqcrypto-*` C-FFI and cannot target `wasm32-unknown-unknown`. This module
//! is what browser/native clients use to encrypt payloads that are then stored
//! as opaque bytes on the Holochain DHT.
//!
//! Construction (never weaker than classical X25519; post-quantum secure if
//! *either* the X25519 or the ML-KEM shared secret remains hard):
//! ```text
//!   ss1        = X25519(eph_sk, recipient_x25519_pk)
//!   (ct, ss2)  = ML-KEM-768::encapsulate(recipient_ek)
//!   key        = HKDF-SHA256(salt = eph_x25519_pk || ct,
//!                            ikm  = ss1 || ss2,
//!                            info = "mycelix-hybrid-kem-v1")[..32]
//!   ciphertext = XChaCha20-Poly1305(key, random 24-byte nonce, plaintext)
//! ```
//! The ephemeral X25519 public key and the ML-KEM ciphertext are bound into the
//! HKDF salt (transcript binding) to resist ciphertext-substitution / re-encaps.
//!
//! # Stability
//! EXPERIMENTAL — feature `hybrid-rc`, off by default. The KEM combiner, AEAD
//! nonce policy, and wire format are pending a crypto audit before any
//! production wiring. See `PQC_ROADMAP_2026-07-07.md`.

use crate::error::CryptoError;
use chacha20poly1305::aead::Aead;
use chacha20poly1305::{KeyInit, XChaCha20Poly1305, XNonce};
use hkdf::Hkdf;
use ml_kem::kem::{Decapsulate, Encapsulate, KeyExport, Seed};
use ml_kem::ml_kem_768::{DecapsulationKey, EncapsulationKey};
use ml_kem::{Kem, MlKem768, TryKeyInit};
use rand::RngCore;
use rand::rngs::OsRng;
use sha2::Sha256;
use x25519_dalek::{EphemeralSecret, PublicKey, StaticSecret};

/// HKDF `info` domain separator. Bump the suffix if the construction changes.
const HKDF_INFO: &[u8] = b"mycelix-hybrid-kem-v1";
/// XChaCha20-Poly1305 nonce length (192-bit; safe to pick at random per message).
const NONCE_LEN: usize = 24;

/// Recipient keypair: an X25519 static secret plus an ML-KEM-768 decapsulation
/// key. Generate once per identity; publish [`HybridPublicKeys`] for senders.
pub struct HybridKemKeyPair {
    x25519_sk: StaticSecret,
    ml_kem_dk: DecapsulationKey,
    ml_kem_ek: EncapsulationKey,
}

/// A recipient's publishable public keys (safe to store on the DHT).
#[derive(Clone, Debug)]
pub struct HybridPublicKeys {
    /// X25519 public key (32 bytes).
    pub x25519: [u8; 32],
    /// ML-KEM-768 encapsulation key (1184 bytes).
    pub ml_kem: Vec<u8>,
}

/// A hybrid ciphertext. Every field is opaque bytes safe to store/transmit.
#[derive(Clone, Debug)]
pub struct HybridCiphertext {
    /// Ephemeral X25519 public key (32 bytes).
    pub x25519_eph_pk: [u8; 32],
    /// ML-KEM-768 ciphertext (1088 bytes).
    pub ml_kem_ct: Vec<u8>,
    /// XChaCha20-Poly1305 nonce (24 bytes).
    pub nonce: [u8; NONCE_LEN],
    /// AEAD ciphertext (plaintext length + 16-byte tag).
    pub ciphertext: Vec<u8>,
}

impl HybridKemKeyPair {
    /// Generate a fresh recipient keypair.
    pub fn generate() -> Self {
        let x25519_sk = StaticSecret::random_from_rng(OsRng);
        let (ml_kem_dk, ml_kem_ek) = MlKem768::generate_keypair();
        Self {
            x25519_sk,
            ml_kem_dk,
            ml_kem_ek,
        }
    }

    /// Public keys to publish so senders can [`seal`] to this recipient.
    pub fn public_keys(&self) -> HybridPublicKeys {
        HybridPublicKeys {
            x25519: PublicKey::from(&self.x25519_sk).to_bytes(),
            ml_kem: self.ml_kem_ek.to_bytes().to_vec(),
        }
    }

    /// Decrypt a [`HybridCiphertext`] sealed to this keypair.
    pub fn open(&self, ct: &HybridCiphertext) -> Result<Vec<u8>, CryptoError> {
        let eph_pk = PublicKey::from(ct.x25519_eph_pk);
        let ss1 = self.x25519_sk.diffie_hellman(&eph_pk);
        let ss2 = self
            .ml_kem_dk
            .decapsulate_slice(&ct.ml_kem_ct)
            .map_err(|_| CryptoError::Validation("ML-KEM decapsulation failed".into()))?;
        let key = derive_key(
            &ct.x25519_eph_pk,
            &ct.ml_kem_ct,
            ss1.as_bytes(),
            ss2.as_slice(),
        );
        let cipher = XChaCha20Poly1305::new_from_slice(&key)
            .map_err(|_| CryptoError::Validation("invalid AEAD key length".into()))?;
        cipher
            .decrypt(XNonce::from_slice(&ct.nonce), ct.ciphertext.as_ref())
            .map_err(|_| {
                CryptoError::Validation("AEAD decryption failed (tampered or wrong key)".into())
            })
    }

    /// Serialize the secret keypair for persistence (e.g. client keystore):
    /// `x25519_secret(32) || ML-KEM-768 seed`. The ML-KEM seed is the compact
    /// (FIPS 203) key representation; the encapsulation key is re-derived on
    /// load. Guard these bytes like any private key.
    pub fn to_bytes(&self) -> Result<Vec<u8>, CryptoError> {
        let seed = self
            .ml_kem_dk
            .to_seed()
            .ok_or_else(|| CryptoError::Validation("ML-KEM key has no recoverable seed".into()))?;
        let mut out = Vec::with_capacity(32 + seed.as_slice().len());
        out.extend_from_slice(&self.x25519_sk.to_bytes());
        out.extend_from_slice(seed.as_slice());
        Ok(out)
    }

    /// Reconstruct a keypair from [`to_bytes`](Self::to_bytes) output.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CryptoError> {
        if bytes.len() < 32 {
            return Err(CryptoError::Validation("keypair bytes too short".into()));
        }
        let x_sk: [u8; 32] = bytes[..32]
            .try_into()
            .map_err(|_| CryptoError::Validation("bad x25519 secret".into()))?;
        let x25519_sk = StaticSecret::from(x_sk);
        let seed = Seed::<MlKem768>::try_from(&bytes[32..])
            .map_err(|_| CryptoError::Validation("invalid ML-KEM seed length".into()))?;
        let ml_kem_dk = DecapsulationKey::from(seed);
        let ml_kem_ek = ml_kem_dk.encapsulation_key().clone();
        Ok(Self {
            x25519_sk,
            ml_kem_dk,
            ml_kem_ek,
        })
    }
}

/// Seal `plaintext` to a recipient's published [`HybridPublicKeys`].
pub fn seal(
    recipient: &HybridPublicKeys,
    plaintext: &[u8],
) -> Result<HybridCiphertext, CryptoError> {
    // Classical half: ephemeral X25519 ECDH.
    let eph_sk = EphemeralSecret::random_from_rng(OsRng);
    let eph_pk = PublicKey::from(&eph_sk);
    let recipient_x = PublicKey::from(recipient.x25519);
    let ss1 = eph_sk.diffie_hellman(&recipient_x);

    // Post-quantum half: ML-KEM-768 encapsulation to the recipient's key.
    let ek = EncapsulationKey::new_from_slice(&recipient.ml_kem)
        .map_err(|_| CryptoError::Validation("invalid ML-KEM encapsulation key".into()))?;
    let (ml_kem_ct, ss2) = ek.encapsulate();
    let ml_kem_ct = ml_kem_ct.as_slice().to_vec();

    // Combine + AEAD.
    let eph_pk_bytes = eph_pk.to_bytes();
    let key = derive_key(&eph_pk_bytes, &ml_kem_ct, ss1.as_bytes(), ss2.as_slice());
    let mut nonce = [0u8; NONCE_LEN];
    OsRng.fill_bytes(&mut nonce);
    let cipher = XChaCha20Poly1305::new_from_slice(&key)
        .map_err(|_| CryptoError::Validation("invalid AEAD key length".into()))?;
    let ciphertext = cipher
        .encrypt(XNonce::from_slice(&nonce), plaintext)
        .map_err(|_| CryptoError::Validation("AEAD encryption failed".into()))?;

    Ok(HybridCiphertext {
        x25519_eph_pk: eph_pk_bytes,
        ml_kem_ct,
        nonce,
        ciphertext,
    })
}

/// HKDF-SHA256 over the concatenated shared secrets, salted with the transcript
/// (ephemeral X25519 pk || ML-KEM ciphertext) so a ciphertext cannot be
/// re-bound to a different key. Returns a 32-byte AEAD key.
fn derive_key(eph_pk: &[u8; 32], ml_kem_ct: &[u8], ss1: &[u8], ss2: &[u8]) -> [u8; 32] {
    let mut salt = Vec::with_capacity(32 + ml_kem_ct.len());
    salt.extend_from_slice(eph_pk);
    salt.extend_from_slice(ml_kem_ct);

    let mut ikm = Vec::with_capacity(ss1.len() + ss2.len());
    ikm.extend_from_slice(ss1);
    ikm.extend_from_slice(ss2);

    let hk = Hkdf::<Sha256>::new(Some(&salt), &ikm);
    let mut okm = [0u8; 32];
    hk.expand(HKDF_INFO, &mut okm)
        .expect("HKDF-SHA256 with a 32-byte output length is always valid");
    okm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keypair_serialization_round_trip() {
        let kp = HybridKemKeyPair::generate();
        let bytes = kp.to_bytes().expect("to_bytes");
        let restored = HybridKemKeyPair::from_bytes(&bytes).expect("from_bytes");

        // The restored key must be functionally identical: it decrypts what the
        // original's published public keys seal, and exports the same pubkeys.
        let ct = seal(&kp.public_keys(), b"persisted key still works").expect("seal");
        assert_eq!(
            restored.open(&ct).expect("open"),
            b"persisted key still works"
        );
        assert_eq!(kp.public_keys().ml_kem, restored.public_keys().ml_kem);
        assert_eq!(kp.public_keys().x25519, restored.public_keys().x25519);
    }

    #[test]
    fn seal_open_round_trip() {
        let recipient = HybridKemKeyPair::generate();
        let pk = recipient.public_keys();
        let msg = b"the mitochondria is the powerhouse of the cell";

        let ct = seal(&pk, msg).expect("seal");
        let out = recipient.open(&ct).expect("open");
        assert_eq!(out, msg);
    }

    #[test]
    fn empty_plaintext_round_trips() {
        let recipient = HybridKemKeyPair::generate();
        let ct = seal(&recipient.public_keys(), b"").expect("seal");
        assert_eq!(recipient.open(&ct).expect("open"), b"");
    }

    #[test]
    fn tampered_ciphertext_is_rejected() {
        let recipient = HybridKemKeyPair::generate();
        let mut ct = seal(&recipient.public_keys(), b"secret").expect("seal");
        // Flip one byte of the AEAD ciphertext.
        ct.ciphertext[0] ^= 0x01;
        assert!(recipient.open(&ct).is_err(), "AEAD must reject tampering");
    }

    #[test]
    fn wrong_recipient_cannot_open() {
        let alice = HybridKemKeyPair::generate();
        let bob = HybridKemKeyPair::generate();
        let ct = seal(&alice.public_keys(), b"for alice only").expect("seal");
        // Bob's keypair must fail: at minimum the AEAD tag won't verify.
        assert!(bob.open(&ct).is_err(), "wrong recipient must not decrypt");
    }

    #[test]
    fn tampered_kem_ciphertext_is_rejected() {
        let recipient = HybridKemKeyPair::generate();
        let mut ct = seal(&recipient.public_keys(), b"secret").expect("seal");
        // Corrupt the ML-KEM ciphertext: ML-KEM's implicit rejection yields a
        // different shared secret, so the derived key differs and AEAD fails.
        ct.ml_kem_ct[0] ^= 0x01;
        assert!(
            recipient.open(&ct).is_err(),
            "corrupted KEM ct must not decrypt"
        );
    }
}
