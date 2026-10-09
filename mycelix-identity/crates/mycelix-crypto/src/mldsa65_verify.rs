// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Narrow FIPS 204 ML-DSA-65 verification primitive for protocol adapters.
//!
//! This module intentionally does not implement a hybrid-signature combiner,
//! inspect an algorithm-tagged envelope, or authorize a key. Callers must bind
//! key authorization and protocol transcript construction separately.
//!
//! The verification operation is explicit: FIPS 204 ML-DSA.Verify with RFC
//! 9964's required empty context. This makes it harder for callers to
//! accidentally substitute a non-empty provider context or the pre-hashed
//! VerifyInternal operation.
//!
//! # Assurance status
//!
//! This is a reusable implementation seam, not a cryptographic qualification.
//! The selected `ml-dsa` dependency has not been independently audited. A
//! caller requiring qualified verification must add pinned known-answer and
//! adversarial corpus evidence, mutation testing, and independent
//! cross-implementation verification.

use crate::error::CryptoError;
use ml_dsa::{
    EncodedSignature, EncodedVerifyingKey, MlDsa65, Signature as MlSignature,
    VerifyingKey as MlVerifyingKey,
};

/// FIPS 204 ML-DSA-65 public-key encoding length.
pub const ML_DSA_65_PUBLIC_KEY_BYTES: usize = 1952;

/// FIPS 204 ML-DSA-65 signature encoding length.
pub const ML_DSA_65_SIGNATURE_BYTES: usize = 3309;

/// RFC 9964 requires the context string to be empty for ML-DSA-65.
pub const ML_DSA_65_EMPTY_CONTEXT: &[u8] = b"";

/// Verify an ML-DSA-65 signature with FIPS 204's empty-context operation.
///
/// The key and signature are the raw FIPS 204 encodings (not legacy
/// Dilithium encodings, COSE_Key structures, or pre-hashed inputs).
/// The function performs strict fixed-size admission before decoding.
///
/// This function establishes cryptographic signature validity only. It does
/// not establish that the key is authorized, current, non-revoked, or bound to
/// a caller's protocol transcript.
pub fn verify_with_empty_context(
    public_key: &[u8],
    message: &[u8],
    signature: &[u8],
) -> Result<(), CryptoError> {
    if public_key.len() != ML_DSA_65_PUBLIC_KEY_BYTES {
        return Err(CryptoError::Validation(
            "ML-DSA-65 public key must be exactly 1952 bytes".into(),
        ));
    }
    if signature.len() != ML_DSA_65_SIGNATURE_BYTES {
        return Err(CryptoError::Validation(
            "ML-DSA-65 signature must be exactly 3309 bytes".into(),
        ));
    }

    let encoded_key = EncodedVerifyingKey::<MlDsa65>::try_from(public_key)
        .map_err(|_| CryptoError::Validation("invalid ML-DSA-65 public key length".into()))?;
    let verifying_key = MlVerifyingKey::<MlDsa65>::decode(&encoded_key);

    let encoded_signature = EncodedSignature::<MlDsa65>::try_from(signature)
        .map_err(|_| CryptoError::Validation("invalid ML-DSA-65 signature length".into()))?;
    let decoded_signature = MlSignature::<MlDsa65>::decode(&encoded_signature)
        .ok_or_else(|| CryptoError::Validation("undecodable ML-DSA-65 signature".into()))?;

    if !verifying_key.verify_with_context(message, ML_DSA_65_EMPTY_CONTEXT, &decoded_signature) {
        return Err(CryptoError::Validation(
            "ML-DSA-65 empty-context signature verification failed".into(),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hybrid_sig::HybridSigner;

    #[test]
    fn accepts_valid_empty_context_ml_dsa_65_signature() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let message = b"standalone ML-DSA-65 verifier";
        let signature = signer.sign(message);

        assert_eq!(keys.ml_dsa.len(), ML_DSA_65_PUBLIC_KEY_BYTES);
        assert_eq!(signature.ml_dsa.len(), ML_DSA_65_SIGNATURE_BYTES);
        assert!(verify_with_empty_context(&keys.ml_dsa, message, &signature.ml_dsa).is_ok());
    }

    #[test]
    fn rejects_modified_message() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let signature = signer.sign(b"original message");

        assert!(verify_with_empty_context(
            &keys.ml_dsa,
            b"modified message",
            &signature.ml_dsa
        )
        .is_err());
    }

    #[test]
    fn rejects_modified_signature() {
        let signer = HybridSigner::generate();
        let keys = signer.verifying_keys();
        let message = b"message";
        let mut signature = signer.sign(message).ml_dsa;
        signature[0] ^= 0x01;

        assert!(verify_with_empty_context(&keys.ml_dsa, message, &signature).is_err());
    }

    #[test]
    fn rejects_wrong_key_and_signature_lengths_before_decode() {
        assert!(verify_with_empty_context(&[0u8; 1951], b"message", &[0u8; 3309]).is_err());
        assert!(verify_with_empty_context(&[0u8; 1952], b"message", &[0u8; 3308]).is_err());
    }

    #[test]
    fn rejects_signature_made_with_nonempty_context() {
        use ml_dsa::{Generate, KeyExport, MlDsa65, SigningKey};

        let signing_key = SigningKey::<MlDsa65>::generate();
        let message = b"context contract";
        let nonempty_context = b"must-not-be-accepted";
        let signature = signing_key
            .expanded_key()
            .sign_deterministic(message, nonempty_context)
            .expect("short context is valid for signing");
        let verifying_key = signing_key.verifying_key().encode().as_slice().to_vec();
        let signature_bytes = signature.encode().as_slice().to_vec();

        assert!(verify_with_empty_context(&verifying_key, message, &signature_bytes).is_err());
    }
}
