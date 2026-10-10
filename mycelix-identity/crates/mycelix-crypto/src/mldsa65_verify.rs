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
    use ml_dsa::{Generate as _, Keypair as _, MlDsa65, SigningKey};

    fn signed_fixture(message: &[u8], context: &[u8]) -> (Vec<u8>, Vec<u8>) {
        let signing_key = SigningKey::<MlDsa65>::generate();
        let verifying_key = signing_key.verifying_key().encode().as_slice().to_vec();
        let signature = signing_key
            .expanded_key()
            .sign_deterministic(message, context)
            .expect("test context length is within the ML-DSA limit")
            .encode()
            .as_slice()
            .to_vec();

        (verifying_key, signature)
    }

    #[test]
    fn accepts_valid_empty_context_ml_dsa_65_signature() {
        let message = b"standalone ML-DSA-65 verifier";
        let (public_key, signature) = signed_fixture(message, ML_DSA_65_EMPTY_CONTEXT);

        assert_eq!(public_key.len(), ML_DSA_65_PUBLIC_KEY_BYTES);
        assert_eq!(signature.len(), ML_DSA_65_SIGNATURE_BYTES);
        assert!(verify_with_empty_context(&public_key, message, &signature).is_ok());
    }

    #[test]
    fn rejects_modified_message() {
        let (public_key, signature) = signed_fixture(b"original message", ML_DSA_65_EMPTY_CONTEXT);

        assert!(verify_with_empty_context(&public_key, b"modified message", &signature).is_err());
    }

    #[test]
    fn rejects_modified_signature() {
        let message = b"message";
        let (public_key, mut signature) = signed_fixture(message, ML_DSA_65_EMPTY_CONTEXT);
        signature[0] ^= 0x01;

        assert!(verify_with_empty_context(&public_key, message, &signature).is_err());
    }

    #[test]
    fn rejects_wrong_key_and_signature_lengths_before_decode() {
        assert!(verify_with_empty_context(&[0u8; 1951], b"message", &[0u8; 3309]).is_err());
        assert!(verify_with_empty_context(&[0u8; 1952], b"message", &[0u8; 3308]).is_err());
    }

    #[test]
    fn rejects_signature_made_with_nonempty_context() {
        let message = b"context contract";
        let (public_key, signature) = signed_fixture(message, b"must-not-be-accepted");

        assert!(verify_with_empty_context(&public_key, message, &signature).is_err());
    }
}
