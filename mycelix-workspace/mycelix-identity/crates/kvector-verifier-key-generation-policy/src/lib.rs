// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact verifier-key generation identity theorem for Identity V2 K-vector records.
//!
//! This crate freezes facts intrinsic to one verifier-key generation. It deliberately
//! does not duplicate policy signature-scheme string mappings: #250 owns the mapping
//! from an accepted policy scheme ID to [`AlgorithmId`].
//!
//! Success proves only structural identity and historical time compatibility. It does
//! not prove DID-lineage currentness, revocation status, signature authenticity, policy
//! acceptance/currentness, or positive verification authority.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const VERIFIER_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_KEY_ID_MAX_LEN_V2: usize = 256;
pub const K_VECTOR_VERIFIER_KEY_GENERATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verifier-key-generation:v2\0";

/// One exact verifier-key generation.
///
/// `verifier_key_id` is required to be the canonical full DID URL
/// (`{verifier_did}#fragment`), not a context-dependent fragment.
#[derive(Debug, Clone, Copy)]
pub struct KVectorVerifierKeyGenerationV2<'a> {
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub public_key_bytes: &'a [u8],
    pub key_generation: u64,
    pub issued_at_micros: i64,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
}

/// Historical use asserted by an upper-layer signed verification record.
///
/// Signature-scheme naming is intentionally absent. A later #250-bound layer must map
/// its exact policy scheme ID to this exact `AlgorithmId`.
#[derive(Debug, Clone, Copy)]
pub struct KVectorVerifierKeyHistoricalUseV2<'a> {
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub verified_at_micros: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorVerifierKeyGenerationErrorV2 {
    VerifierDidInvalid,
    VerifierKeyIdInvalid,
    VerifierKeyIdNotCanonical,
    KeyGenerationInvalid,
    NonSignatureAlgorithm,
    PublicKeyLengthInvalid,
    PublicKeyAllZero,
    ValidityIntervalInvalid,
    IssuedAfterValidityStart,
    VerifierDidMismatch,
    VerifierKeyIdMismatch,
    AlgorithmMismatch,
    KeyNotYetValidAtVerification,
    KeyExpiredAtVerification,
}

fn valid_bounded_text(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_verifier_did(value: &str) -> bool {
    valid_bounded_text(value, VERIFIER_DID_MAX_LEN_V2) && value.starts_with("did:mycelix:")
}

fn validate_canonical_key_id(
    verifier_did: &str,
    verifier_key_id: &str,
) -> Result<(), KVectorVerifierKeyGenerationErrorV2> {
    if !valid_bounded_text(verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdInvalid);
    }
    let prefix = format!("{verifier_did}#");
    if !verifier_key_id.starts_with(&prefix) || verifier_key_id.len() <= prefix.len() {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdNotCanonical);
    }
    Ok(())
}

/// Validate one exact key generation structurally.
///
/// No current clock, DID history, policy state, or signature verifier participates.
pub fn validate_kvector_verifier_key_generation_v2(
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<(), KVectorVerifierKeyGenerationErrorV2> {
    if !valid_verifier_did(generation.verifier_did) {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierDidInvalid);
    }
    validate_canonical_key_id(generation.verifier_did, generation.verifier_key_id)?;
    if generation.key_generation == 0 {
        return Err(KVectorVerifierKeyGenerationErrorV2::KeyGenerationInvalid);
    }
    if !generation.algorithm.is_signature_algorithm() {
        return Err(KVectorVerifierKeyGenerationErrorV2::NonSignatureAlgorithm);
    }

    TaggedPublicKey::new(generation.algorithm, generation.public_key_bytes.to_vec())
        .map_err(|_| KVectorVerifierKeyGenerationErrorV2::PublicKeyLengthInvalid)?;
    if generation.public_key_bytes.iter().all(|byte| *byte == 0) {
        return Err(KVectorVerifierKeyGenerationErrorV2::PublicKeyAllZero);
    }

    if generation.valid_until_micros <= generation.valid_from_micros {
        return Err(KVectorVerifierKeyGenerationErrorV2::ValidityIntervalInvalid);
    }
    if generation.issued_at_micros > generation.valid_from_micros {
        return Err(KVectorVerifierKeyGenerationErrorV2::IssuedAfterValidityStart);
    }

    Ok(())
}

fn update_len_prefixed(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    let len = value.len() as u16;
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

/// Derive the deterministic identity of one exact verifier-key generation.
///
/// Every field participates. Reusing a human-readable key ID, or even the same public
/// key bytes, cannot hide a generation change because `key_generation` is committed.
pub fn derive_kvector_verifier_key_generation_digest_v2(
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], KVectorVerifierKeyGenerationErrorV2> {
    validate_kvector_verifier_key_generation_v2(generation)?;

    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_VERIFIER_KEY_GENERATION_DOMAIN_V2);
    update_len_prefixed(&mut hasher, 0x01, generation.verifier_did.as_bytes());
    update_len_prefixed(&mut hasher, 0x02, generation.verifier_key_id.as_bytes());

    hasher.update([0x03]);
    hasher.update(generation.algorithm.as_u16().to_be_bytes());
    update_len_prefixed(&mut hasher, 0x04, generation.public_key_bytes);

    hasher.update([0x05]);
    hasher.update(generation.key_generation.to_be_bytes());
    hasher.update([0x06]);
    hasher.update(generation.issued_at_micros.to_be_bytes());
    hasher.update([0x07]);
    hasher.update(generation.valid_from_micros.to_be_bytes());
    hasher.update([0x08]);
    hasher.update(generation.valid_until_micros.to_be_bytes());

    let digest = hasher.finalize();
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    Ok(out)
}

/// Prove that one exact key generation is structurally compatible with one asserted
/// historical verification instant.
///
/// This is not currentness: a generation can be historically valid here and revoked or
/// superseded now. A later observed-lineage/currentness layer must prove those facts.
pub fn validate_kvector_verifier_key_historical_use_v2(
    generation: KVectorVerifierKeyGenerationV2<'_>,
    historical_use: KVectorVerifierKeyHistoricalUseV2<'_>,
) -> Result<(), KVectorVerifierKeyGenerationErrorV2> {
    validate_kvector_verifier_key_generation_v2(generation)?;

    if !valid_verifier_did(historical_use.verifier_did) {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierDidInvalid);
    }
    validate_canonical_key_id(historical_use.verifier_did, historical_use.verifier_key_id)?;

    if historical_use.verifier_did != generation.verifier_did {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierDidMismatch);
    }
    if historical_use.verifier_key_id != generation.verifier_key_id {
        return Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdMismatch);
    }
    if historical_use.algorithm != generation.algorithm {
        return Err(KVectorVerifierKeyGenerationErrorV2::AlgorithmMismatch);
    }
    if historical_use.verified_at_micros < generation.valid_from_micros {
        return Err(KVectorVerifierKeyGenerationErrorV2::KeyNotYetValidAtVerification);
    }
    if historical_use.verified_at_micros >= generation.valid_until_micros {
        return Err(KVectorVerifierKeyGenerationErrorV2::KeyExpiredAtVerification);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    fn key() -> [u8; 32] {
        [0x42; 32]
    }

    fn generation<'a>(public_key_bytes: &'a [u8]) -> KVectorVerifierKeyGenerationV2<'a> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes,
            key_generation: 1,
            issued_at_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
        }
    }

    fn historical_use<'a>() -> KVectorVerifierKeyHistoricalUseV2<'a> {
        KVectorVerifierKeyHistoricalUseV2 {
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            algorithm: AlgorithmId::Ed25519,
            verified_at_micros: 2_000_000,
        }
    }

    #[test]
    fn exact_generation_digest_vector_is_frozen() {
        let key = key();
        let digest = derive_kvector_verifier_key_generation_digest_v2(generation(&key)).unwrap();
        assert_eq!(
            digest,
            [
                0x8f, 0xb1, 0xbf, 0x0f, 0x8e, 0x26, 0xa0, 0x0c, 0x31, 0x97, 0xf3, 0xbe,
                0xde, 0xb7, 0xb4, 0x30, 0x99, 0x7c, 0xa1, 0x66, 0xfb, 0x88, 0xfd, 0x9c,
                0xdc, 0xa8, 0xc7, 0x8d, 0x07, 0x94, 0x0d, 0x96,
            ]
        );
    }

    #[test]
    fn public_key_bytes_and_generation_both_change_identity() {
        let first_key = key();
        let second_key = [0x43; 32];
        let first = generation(&first_key);
        let changed_bytes = KVectorVerifierKeyGenerationV2 {
            public_key_bytes: &second_key,
            ..first
        };
        let changed_generation = KVectorVerifierKeyGenerationV2 {
            key_generation: 2,
            ..first
        };

        let first_digest = derive_kvector_verifier_key_generation_digest_v2(first).unwrap();
        assert_ne!(
            first_digest,
            derive_kvector_verifier_key_generation_digest_v2(changed_bytes).unwrap()
        );
        assert_ne!(
            first_digest,
            derive_kvector_verifier_key_generation_digest_v2(changed_generation).unwrap()
        );
    }

    #[test]
    fn canonical_full_key_id_is_required() {
        let key = key();
        let base = generation(&key);
        assert_eq!(validate_kvector_verifier_key_generation_v2(base), Ok(()));
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                verifier_key_id: "#key-1",
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdNotCanonical)
        );
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                verifier_key_id: "did:mycelix:other#key-1",
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdNotCanonical)
        );
    }

    #[test]
    fn algorithm_key_shape_generation_and_interval_fail_closed() {
        let key = key();
        let base = generation(&key);
        let short = [0x42; 31];
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                public_key_bytes: &short,
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::PublicKeyLengthInvalid)
        );
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                algorithm: AlgorithmId::MlKem768,
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::NonSignatureAlgorithm)
        );
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                key_generation: 0,
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::KeyGenerationInvalid)
        );
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(KVectorVerifierKeyGenerationV2 {
                valid_until_micros: base.valid_from_micros,
                ..base
            }),
            Err(KVectorVerifierKeyGenerationErrorV2::ValidityIntervalInvalid)
        );
    }

    #[test]
    fn all_zero_key_is_rejected() {
        let zero = [0u8; 32];
        assert_eq!(
            validate_kvector_verifier_key_generation_v2(generation(&zero)),
            Err(KVectorVerifierKeyGenerationErrorV2::PublicKeyAllZero)
        );
    }

    #[test]
    fn historical_use_requires_exact_identity_algorithm_and_time() {
        let key = key();
        let base = generation(&key);
        let use_context = historical_use();
        assert_eq!(
            validate_kvector_verifier_key_historical_use_v2(base, use_context),
            Ok(())
        );
        assert_eq!(
            validate_kvector_verifier_key_historical_use_v2(
                base,
                KVectorVerifierKeyHistoricalUseV2 {
                    verifier_key_id: "did:mycelix:verifier#key-2",
                    ..use_context
                }
            ),
            Err(KVectorVerifierKeyGenerationErrorV2::VerifierKeyIdMismatch)
        );
        assert_eq!(
            validate_kvector_verifier_key_historical_use_v2(
                base,
                KVectorVerifierKeyHistoricalUseV2 {
                    algorithm: AlgorithmId::MlDsa65,
                    ..use_context
                }
            ),
            Err(KVectorVerifierKeyGenerationErrorV2::AlgorithmMismatch)
        );
        assert_eq!(
            validate_kvector_verifier_key_historical_use_v2(
                base,
                KVectorVerifierKeyHistoricalUseV2 {
                    verified_at_micros: base.valid_from_micros - 1,
                    ..use_context
                }
            ),
            Err(KVectorVerifierKeyGenerationErrorV2::KeyNotYetValidAtVerification)
        );
        assert_eq!(
            validate_kvector_verifier_key_historical_use_v2(
                base,
                KVectorVerifierKeyHistoricalUseV2 {
                    verified_at_micros: base.valid_until_micros,
                    ..use_context
                }
            ),
            Err(KVectorVerifierKeyGenerationErrorV2::KeyExpiredAtVerification)
        );
    }

    #[test]
    fn signature_scheme_strings_are_not_part_of_this_theorem() {
        let source = include_str!("lib.rs");
        assert!(!source.contains("hybrid-ed25519-mldsa65-v1"));
        assert!(!source.contains("hybrid-ed25519-ml-dsa-65-v1"));
        assert!(!source.contains("ed25519-v1"));
    }
}
