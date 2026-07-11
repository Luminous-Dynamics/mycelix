// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Hybrid Ed25519 + ML-DSA-65 signatures (RustCrypto, wasm32-capable).
//!
//! The signature counterpart to [`crate::hybrid_kem`]. A hybrid signature is a
//! pair `(ed25519_sig, ml_dsa65_sig)` over the *same* message; verification
//! requires **both** to pass, so the scheme is at least as strong as the
//! stronger of the two — classically secure via Ed25519, post-quantum secure
//! via ML-DSA-65.
//!
//! Unlike `crate::pqc::hybrid` (native, `pqcrypto` C-FFI), this builds for
//! `wasm32-unknown-unknown`, so browser/native clients can produce and verify
//! hybrid signatures that zomes store as opaque bytes. (Note: on-chain zomes
//! can currently verify only the Ed25519 half — HDK has no ML-DSA verify host
//! function yet; the ML-DSA half must be verified client-side or in a native
//! sidecar. See PQC_ROADMAP_2026-07-07.md.)
//!
//! # Stability
//! EXPERIMENTAL — feature `hybrid-rc`, off by default. Pending crypto audit
//! before production wiring.

use crate::error::CryptoError;
use ed25519_dalek::{
    Signature as EdSignature, Signer as _, SigningKey as EdSigningKey, Verifier as _,
    VerifyingKey as EdVerifyingKey,
};
use ml_dsa::signature::{Keypair as _, Signer as MlSigner, Verifier as MlVerifier};
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, Generate as _, KeyExport as _, KeyInit as _, MlDsa65,
    Signature as MlSignature, SigningKey as MlSigningKey, VerifyingKey as MlVerifyingKey,
};
use rand::rngs::OsRng;

/// Hybrid signer holding both secret keys. Persist with
/// [`to_bytes`](HybridSigner::to_bytes) / restore with
/// [`from_bytes`](HybridSigner::from_bytes).
pub struct HybridSigner {
    ed: EdSigningKey,
    ml_dsa: MlSigningKey<MlDsa65>,
}

/// Publishable hybrid verifying keys.
#[derive(Clone, Debug)]
pub struct HybridVerifyingKeys {
    /// Ed25519 verifying key (32 bytes).
    pub ed25519: [u8; 32],
    /// ML-DSA-65 verifying key (1952 bytes).
    pub ml_dsa: Vec<u8>,
}

/// A hybrid signature: Ed25519 (64 bytes) + ML-DSA-65 (~3309 bytes).
#[derive(Clone, Debug)]
pub struct HybridSignature {
    /// Ed25519 signature (64 bytes).
    pub ed25519: [u8; 64],
    /// ML-DSA-65 signature.
    pub ml_dsa: Vec<u8>,
}

impl HybridSigner {
    /// Generate a fresh hybrid keypair.
    pub fn generate() -> Self {
        Self {
            ed: EdSigningKey::generate(&mut OsRng),
            ml_dsa: MlSigningKey::<MlDsa65>::generate(),
        }
    }

    /// Verifying keys to publish so others can [`verify`] this signer.
    pub fn verifying_keys(&self) -> HybridVerifyingKeys {
        HybridVerifyingKeys {
            ed25519: self.ed.verifying_key().to_bytes(),
            ml_dsa: self.ml_dsa.verifying_key().encode().as_slice().to_vec(),
        }
    }

    /// Sign `message` with both algorithms over the identical bytes.
    pub fn sign(&self, message: &[u8]) -> HybridSignature {
        let ed = self.ed.sign(message).to_bytes();
        let ml_dsa = self.ml_dsa.sign(message).encode().as_slice().to_vec();
        HybridSignature {
            ed25519: ed,
            ml_dsa,
        }
    }

    /// Serialize the secret signer for persistence:
    /// `ed25519_secret(32) || ML-DSA-65 seed(32)`. Both are the compact seed
    /// representations; expanded keys are re-derived on load. Guard as private.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(64);
        out.extend_from_slice(&self.ed.to_bytes());
        out.extend_from_slice(self.ml_dsa.to_bytes().as_slice());
        out
    }

    /// Reconstruct a signer from [`to_bytes`](Self::to_bytes) output.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, CryptoError> {
        if bytes.len() < 32 {
            return Err(CryptoError::Validation("signer bytes too short".into()));
        }
        let ed_sk: [u8; 32] = bytes[..32]
            .try_into()
            .map_err(|_| CryptoError::Validation("bad ed25519 secret".into()))?;
        let ed = EdSigningKey::from_bytes(&ed_sk);
        let ml_dsa = MlSigningKey::<MlDsa65>::new_from_slice(&bytes[32..])
            .map_err(|_| CryptoError::Validation("invalid ML-DSA signing key".into()))?;
        Ok(Self { ed, ml_dsa })
    }
}

/// Verify a hybrid signature. **Both** halves must verify against `message`.
pub fn verify(
    keys: &HybridVerifyingKeys,
    message: &[u8],
    sig: &HybridSignature,
) -> Result<(), CryptoError> {
    // --- Ed25519 (classical) ---
    let ed_vk = EdVerifyingKey::from_bytes(&keys.ed25519)
        .map_err(|_| CryptoError::Validation("invalid Ed25519 verifying key".into()))?;
    let ed_sig = EdSignature::from_bytes(&sig.ed25519);
    ed_vk
        .verify(message, &ed_sig)
        .map_err(|_| CryptoError::Validation("Ed25519 signature verification failed".into()))?;

    // --- ML-DSA-65 (post-quantum) ---
    let ml_encoded_vk = EncodedVerifyingKey::<MlDsa65>::try_from(keys.ml_dsa.as_slice())
        .map_err(|_| CryptoError::Validation("invalid ML-DSA verifying key length".into()))?;
    let ml_vk = MlVerifyingKey::<MlDsa65>::decode(&ml_encoded_vk);
    let ml_encoded_sig = EncodedSignature::<MlDsa65>::try_from(sig.ml_dsa.as_slice())
        .map_err(|_| CryptoError::Validation("invalid ML-DSA signature length".into()))?;
    let ml_sig = MlSignature::<MlDsa65>::decode(&ml_encoded_sig)
        .ok_or_else(|| CryptoError::Validation("undecodable ML-DSA signature".into()))?;
    ml_vk
        .verify(message, &ml_sig)
        .map_err(|_| CryptoError::Validation("ML-DSA signature verification failed".into()))?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signer_serialization_round_trip() {
        let signer = HybridSigner::generate();
        let bytes = signer.to_bytes();
        let restored = HybridSigner::from_bytes(&bytes).expect("from_bytes");

        // A signature from the restored signer must verify against the ORIGINAL's
        // published keys, and both must export identical verifying keys.
        let msg = b"persisted signer still works";
        let sig = restored.sign(msg);
        assert!(verify(&signer.verifying_keys(), msg, &sig).is_ok());
        assert_eq!(
            signer.verifying_keys().ml_dsa,
            restored.verifying_keys().ml_dsa
        );
        assert_eq!(
            signer.verifying_keys().ed25519,
            restored.verifying_keys().ed25519
        );
    }

    #[test]
    fn sign_verify_round_trip() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let msg = b"consciousness-first cryptography";
        let sig = signer.sign(msg);
        assert!(verify(&keys, msg, &sig).is_ok());
    }

    #[test]
    fn tampered_message_is_rejected() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let sig = signer.sign(b"authentic message");
        assert!(verify(&keys, b"forged message", &sig).is_err());
    }

    #[test]
    fn wrong_signer_is_rejected() {
        let alice = HybridSigner::generate();
        let mallory = HybridSigner::generate();
        let msg = b"from alice";
        let sig = alice.sign(msg);
        // Mallory's keys must not verify Alice's signature.
        assert!(verify(&mallory.verifying_keys(), msg, &sig).is_err());
    }

    #[test]
    fn forged_ml_dsa_half_is_rejected() {
        // A valid Ed25519 half must NOT be enough — the hybrid requires both.
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let msg = b"needs both halves";
        let mut sig = signer.sign(msg);
        sig.ml_dsa[0] ^= 0x01; // corrupt the PQC half only
        assert!(
            verify(&keys, msg, &sig).is_err(),
            "a broken ML-DSA half must fail the whole verify"
        );
    }

    #[test]
    fn forged_ed25519_half_is_rejected() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let msg = b"needs both halves";
        let mut sig = signer.sign(msg);
        sig.ed25519[0] ^= 0x01; // corrupt the classical half only
        assert!(
            verify(&keys, msg, &sig).is_err(),
            "a broken Ed25519 half must fail the whole verify"
        );
    }
}
