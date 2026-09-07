// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Algorithm/byte-shape binding theorem for Identity V2 verifier signatures.
//!
//! This crate does **not** perform cryptographic verification. It guarantees only
//! that the key/signature material presented to a future verifier is tagged with
//! the exact algorithm required by the canonical verifier-policy body and has the
//! exact raw byte lengths for that algorithm.
//!
//! Security split:
//!
//! - #248 owns the exact static verifier-policy tuple;
//! - this crate prevents signature-algorithm downgrade/substitution;
//! - #244 must provide branch-aware observed verifier-key lineage;
//! - a later native/hybrid-capable adapter verifies the exact #235 signing digest;
//! - only after #246 policy authority/currentness may positive evidence exist.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey, TaggedSignature};
use mycelix_kvector_verifier_policy_body::{
    validate_kvector_verifier_policy_body_v2, KVectorVerifierPolicyBodyV2,
};

pub const ED25519_SIGNATURE_SCHEME_ID_V2: &str = "ed25519-v1";
pub const HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2: &str =
    "hybrid-ed25519-mldsa65-v1";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorSignatureMaterialErrorV2 {
    PolicyBodyInvalid,
    UnsupportedSignatureScheme,
    PublicKeyAlgorithmMismatch,
    SignatureAlgorithmMismatch,
    PublicKeyLengthInvalid,
    SignatureLengthInvalid,
}

pub fn expected_algorithm_for_signature_scheme_v2(
    signature_scheme_id: &str,
) -> Result<AlgorithmId, KVectorSignatureMaterialErrorV2> {
    match signature_scheme_id {
        ED25519_SIGNATURE_SCHEME_ID_V2 => Ok(AlgorithmId::Ed25519),
        HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2 => {
            Ok(AlgorithmId::HybridEd25519MlDsa65)
        }
        _ => Err(KVectorSignatureMaterialErrorV2::UnsupportedSignatureScheme),
    }
}

/// Validate only the algorithm tag and exact byte shape for one policy-bound
/// verifier key/signature pair.
///
/// Success here is **not** signature verification. The returned `()` means only
/// that a future cryptographic verifier can consume an unambiguous algorithm/key/
/// signature tuple without silently downgrading a hybrid policy to one component.
pub fn validate_kvector_signature_material_for_policy_v2(
    policy: KVectorVerifierPolicyBodyV2<'_>,
    public_key: &TaggedPublicKey,
    signature: &TaggedSignature,
) -> Result<(), KVectorSignatureMaterialErrorV2> {
    validate_kvector_verifier_policy_body_v2(policy)
        .map_err(|_| KVectorSignatureMaterialErrorV2::PolicyBodyInvalid)?;

    let expected = expected_algorithm_for_signature_scheme_v2(policy.signature_scheme_id)?;
    if public_key.algorithm != expected {
        return Err(KVectorSignatureMaterialErrorV2::PublicKeyAlgorithmMismatch);
    }
    if signature.algorithm != expected {
        return Err(KVectorSignatureMaterialErrorV2::SignatureAlgorithmMismatch);
    }

    let expected_key_len = expected.public_key_size();
    if public_key.key_bytes.len() != expected_key_len {
        return Err(KVectorSignatureMaterialErrorV2::PublicKeyLengthInvalid);
    }

    let expected_signature_len = expected.signature_size();
    if signature.signature_bytes.len() != expected_signature_len {
        return Err(KVectorSignatureMaterialErrorV2::SignatureLengthInvalid);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn policy(signature_scheme_id: &'static str) -> KVectorVerifierPolicyBodyV2<'static> {
        KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:kvector-prod-v2",
            policy_version: "2.0.0",
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#key-1",
            signature_scheme_id,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
            max_record_lifetime_micros: 2_000_000,
        }
    }

    fn material(algorithm: AlgorithmId) -> (TaggedPublicKey, TaggedSignature) {
        (
            TaggedPublicKey {
                algorithm,
                key_bytes: vec![0x11; algorithm.public_key_size()],
            },
            TaggedSignature {
                algorithm,
                signature_bytes: vec![0x22; algorithm.signature_size()],
            },
        )
    }

    #[test]
    fn exact_scheme_to_algorithm_mapping_is_frozen() {
        assert_eq!(
            expected_algorithm_for_signature_scheme_v2(ED25519_SIGNATURE_SCHEME_ID_V2),
            Ok(AlgorithmId::Ed25519)
        );
        assert_eq!(
            expected_algorithm_for_signature_scheme_v2(
                HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2
            ),
            Ok(AlgorithmId::HybridEd25519MlDsa65)
        );
        assert_eq!(
            expected_algorithm_for_signature_scheme_v2("ed25519-component-of-hybrid"),
            Err(KVectorSignatureMaterialErrorV2::UnsupportedSignatureScheme)
        );
    }

    #[test]
    fn exact_hybrid_material_is_structurally_admitted() {
        let (key, signature) = material(AlgorithmId::HybridEd25519MlDsa65);
        assert_eq!(
            validate_kvector_signature_material_for_policy_v2(
                policy(HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2),
                &key,
                &signature,
            ),
            Ok(())
        );
        assert_eq!(key.key_bytes.len(), 32 + 1952);
        assert_eq!(signature.signature_bytes.len(), 64 + 3309);
    }

    #[test]
    fn hybrid_policy_rejects_ed25519_only_material() {
        let (key, signature) = material(AlgorithmId::Ed25519);
        assert_eq!(
            validate_kvector_signature_material_for_policy_v2(
                policy(HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2),
                &key,
                &signature,
            ),
            Err(KVectorSignatureMaterialErrorV2::PublicKeyAlgorithmMismatch)
        );
    }

    #[test]
    fn mixed_algorithm_key_and_signature_fail_closed() {
        let (hybrid_key, _) = material(AlgorithmId::HybridEd25519MlDsa65);
        let (_, ed_signature) = material(AlgorithmId::Ed25519);
        assert_eq!(
            validate_kvector_signature_material_for_policy_v2(
                policy(HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2),
                &hybrid_key,
                &ed_signature,
            ),
            Err(KVectorSignatureMaterialErrorV2::SignatureAlgorithmMismatch)
        );
    }

    #[test]
    fn public_fields_cannot_bypass_raw_length_recheck() {
        let algorithm = AlgorithmId::HybridEd25519MlDsa65;
        let malformed_key = TaggedPublicKey {
            algorithm,
            key_bytes: vec![0u8; algorithm.public_key_size() - 1],
        };
        let malformed_signature = TaggedSignature {
            algorithm,
            signature_bytes: vec![0u8; algorithm.signature_size() - 1],
        };
        let (_, valid_signature) = material(algorithm);
        let (valid_key, _) = material(algorithm);

        assert_eq!(
            validate_kvector_signature_material_for_policy_v2(
                policy(HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2),
                &malformed_key,
                &valid_signature,
            ),
            Err(KVectorSignatureMaterialErrorV2::PublicKeyLengthInvalid)
        );
        assert_eq!(
            validate_kvector_signature_material_for_policy_v2(
                policy(HYBRID_ED25519_ML_DSA65_SIGNATURE_SCHEME_ID_V2),
                &valid_key,
                &malformed_signature,
            ),
            Err(KVectorSignatureMaterialErrorV2::SignatureLengthInvalid)
        );
    }
}
