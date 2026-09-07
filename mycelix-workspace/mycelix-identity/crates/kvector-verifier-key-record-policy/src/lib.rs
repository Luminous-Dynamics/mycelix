// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Holochain-free verifier-key record assertion theorem for Identity V2.
//!
//! This crate freezes the exact identity of one verifier key generation so a
//! signed K-vector verification record can bind more strongly than a reusable
//! human-readable key ID.
//!
//! It deliberately does **not** establish that a key is current, unrevoked,
//! institutionally accepted, or cryptographically used correctly. A later
//! history/currentness adapter must prove those facts independently.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const VERIFIER_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_KEY_ID_MAX_LEN_V2: usize = 128;
pub const SIGNATURE_SCHEME_ID_MAX_LEN_V2: usize = 64;
pub const K_VECTOR_VERIFIER_KEY_RECORD_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verifier-key-record:v2\0";

/// One exact verifier-key generation assertion.
///
/// `key_generation` is part of record identity even if DID, key ID and public
/// key bytes happen to be reused. The record itself is historical assertion
/// data, not proof that this generation remains current.
#[derive(Debug, Clone, Copy)]
pub struct KVectorVerifierKeyRecordV2<'a> {
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub algorithm: AlgorithmId,
    pub public_key_bytes: &'a [u8],
    pub key_generation: u64,
    pub issued_at_micros: i64,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
}

/// Exact historical use context asserted by a verification-record body.
///
/// Passing this check means only that the named key record structurally matches
/// the asserted verifier/key/scheme and covered the verification timestamp.
#[derive(Debug, Clone, Copy)]
pub struct KVectorVerifierKeyUseContextV2<'a> {
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub signature_scheme_id: &'a str,
    pub verified_at_micros: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorVerifierKeyRecordErrorV2 {
    VerifierDidInvalid,
    VerifierKeyIdInvalid,
    KeyGenerationInvalid,
    NonSignatureAlgorithm,
    PublicKeyLengthInvalid,
    PublicKeyAllZero,
    ValidityIntervalInvalid,
    IssuedAfterValidityStart,
    SignatureSchemeUnsupported,
    VerifierDidMismatch,
    VerifierKeyIdMismatch,
    SignatureSchemeMismatch,
    KeyNotYetValidAtVerification,
    KeyExpiredAtVerification,
}

fn valid_bounded_text(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    valid_bounded_text(value, VERIFIER_DID_MAX_LEN_V2) && value.starts_with("did:")
}

/// Canonical signature-scheme identifier corresponding to an algorithm-tagged
/// verifier key. Key-agreement and symmetric algorithms are rejected.
pub fn kvector_verifier_signature_scheme_id_v2(
    algorithm: AlgorithmId,
) -> Result<&'static str, KVectorVerifierKeyRecordErrorV2> {
    match algorithm {
        AlgorithmId::Ed25519 => Ok("ed25519-v1"),
        AlgorithmId::MlDsa65 => Ok("ml-dsa-65-v1"),
        AlgorithmId::MlDsa87 => Ok("ml-dsa-87-v1"),
        AlgorithmId::SlhDsaSha2_128s => Ok("slh-dsa-sha2-128s-v1"),
        AlgorithmId::SlhDsaShake128s => Ok("slh-dsa-shake-128s-v1"),
        AlgorithmId::HybridEd25519MlDsa65 => Ok("hybrid-ed25519-ml-dsa-65-v1"),
        AlgorithmId::MlKem768 | AlgorithmId::MlKem1024 | AlgorithmId::XChaCha20Poly1305 => {
            Err(KVectorVerifierKeyRecordErrorV2::NonSignatureAlgorithm)
        }
    }
}

/// Validate one exact verifier-key record structurally.
///
/// This does not inspect a DID history, revocation lineage or current time.
pub fn validate_kvector_verifier_key_record_v2(
    record: KVectorVerifierKeyRecordV2<'_>,
) -> Result<(), KVectorVerifierKeyRecordErrorV2> {
    if !valid_did(record.verifier_did) {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierDidInvalid);
    }
    if !valid_bounded_text(record.verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierKeyIdInvalid);
    }
    if record.key_generation == 0 {
        return Err(KVectorVerifierKeyRecordErrorV2::KeyGenerationInvalid);
    }
    if !record.algorithm.is_signature_algorithm() {
        return Err(KVectorVerifierKeyRecordErrorV2::NonSignatureAlgorithm);
    }
    kvector_verifier_signature_scheme_id_v2(record.algorithm)?;

    TaggedPublicKey::new(record.algorithm, record.public_key_bytes.to_vec())
        .map_err(|_| KVectorVerifierKeyRecordErrorV2::PublicKeyLengthInvalid)?;
    if record.public_key_bytes.iter().all(|byte| *byte == 0) {
        return Err(KVectorVerifierKeyRecordErrorV2::PublicKeyAllZero);
    }

    if record.valid_until_micros <= record.valid_from_micros {
        return Err(KVectorVerifierKeyRecordErrorV2::ValidityIntervalInvalid);
    }
    if record.issued_at_micros > record.valid_from_micros {
        return Err(KVectorVerifierKeyRecordErrorV2::IssuedAfterValidityStart);
    }

    Ok(())
}

fn update_len_prefixed(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    let len = value.len() as u16;
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

/// Derive the canonical identity of one exact verifier-key record generation.
///
/// This digest is intended to be embedded in the signed verification-record
/// transcript above this layer. It binds exact algorithm-tagged public-key bytes
/// and generation, so later key-ID reuse or rotation cannot be invisible.
pub fn derive_kvector_verifier_key_record_digest_v2(
    record: KVectorVerifierKeyRecordV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], KVectorVerifierKeyRecordErrorV2> {
    validate_kvector_verifier_key_record_v2(record)?;

    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_VERIFIER_KEY_RECORD_DOMAIN_V2);

    update_len_prefixed(&mut hasher, 0x01, record.verifier_did.as_bytes());
    update_len_prefixed(&mut hasher, 0x02, record.verifier_key_id.as_bytes());

    hasher.update([0x03]);
    hasher.update(record.algorithm.as_u16().to_be_bytes());

    update_len_prefixed(&mut hasher, 0x04, record.public_key_bytes);

    hasher.update([0x05]);
    hasher.update(record.key_generation.to_be_bytes());

    hasher.update([0x06]);
    hasher.update(record.issued_at_micros.to_be_bytes());

    hasher.update([0x07]);
    hasher.update(record.valid_from_micros.to_be_bytes());

    hasher.update([0x08]);
    hasher.update(record.valid_until_micros.to_be_bytes());

    let digest = hasher.finalize();
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    Ok(out)
}

/// Bind one exact key record to the verifier/key/scheme/timestamp asserted by a
/// verification record.
///
/// This is historical structural compatibility only. It does not prove the key
/// generation is current or unrevoked at observation/use time.
pub fn validate_kvector_verifier_key_use_v2(
    record: KVectorVerifierKeyRecordV2<'_>,
    context: KVectorVerifierKeyUseContextV2<'_>,
) -> Result<(), KVectorVerifierKeyRecordErrorV2> {
    validate_kvector_verifier_key_record_v2(record)?;
    if !valid_did(context.verifier_did) {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierDidInvalid);
    }
    if !valid_bounded_text(context.verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierKeyIdInvalid);
    }
    if !valid_bounded_text(context.signature_scheme_id, SIGNATURE_SCHEME_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierKeyRecordErrorV2::SignatureSchemeUnsupported);
    }
    if context.verifier_did != record.verifier_did {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierDidMismatch);
    }
    if context.verifier_key_id != record.verifier_key_id {
        return Err(KVectorVerifierKeyRecordErrorV2::VerifierKeyIdMismatch);
    }
    let expected_scheme = kvector_verifier_signature_scheme_id_v2(record.algorithm)?;
    if context.signature_scheme_id != expected_scheme {
        return Err(KVectorVerifierKeyRecordErrorV2::SignatureSchemeMismatch);
    }
    if context.verified_at_micros < record.valid_from_micros {
        return Err(KVectorVerifierKeyRecordErrorV2::KeyNotYetValidAtVerification);
    }
    if context.verified_at_micros >= record.valid_until_micros {
        return Err(KVectorVerifierKeyRecordErrorV2::KeyExpiredAtVerification);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ed25519_key() -> [u8; 32] {
        [0x42; 32]
    }

    fn record<'a>(key: &'a [u8]) -> KVectorVerifierKeyRecordV2<'a> {
        KVectorVerifierKeyRecordV2 {
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "verifier-key-1",
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: key,
            key_generation: 1,
            issued_at_micros: 1_699_999_990_000_000,
            valid_from_micros: 1_700_000_000_000_000,
            valid_until_micros: 1_700_003_600_000_000,
        }
    }

    fn use_context<'a>() -> KVectorVerifierKeyUseContextV2<'a> {
        KVectorVerifierKeyUseContextV2 {
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "verifier-key-1",
            signature_scheme_id: "ed25519-v1",
            verified_at_micros: 1_700_000_100_000_000,
        }
    }

    #[test]
    fn verifier_key_record_digest_vector_is_frozen() {
        let key = ed25519_key();
        let actual = derive_kvector_verifier_key_record_digest_v2(record(&key)).unwrap();
        assert_eq!(
            actual,
            [
                0xaa, 0xf7, 0x7d, 0x68, 0xfa, 0x66, 0xc8, 0xf7, 0x1a, 0x01, 0xec, 0x36,
                0x01, 0x84, 0xaf, 0x21, 0xad, 0x15, 0xb7, 0xd6, 0x3a, 0xcd, 0x0e, 0xbe,
                0xc0, 0x03, 0x81, 0x3d, 0x3b, 0xff, 0x67, 0x0e,
            ]
        );
    }

    #[test]
    fn key_generation_is_part_of_record_identity() {
        let key = ed25519_key();
        let first = record(&key);
        let second = KVectorVerifierKeyRecordV2 {
            key_generation: 2,
            ..first
        };
        assert_ne!(
            derive_kvector_verifier_key_record_digest_v2(first).unwrap(),
            derive_kvector_verifier_key_record_digest_v2(second).unwrap()
        );
    }

    #[test]
    fn algorithm_and_key_length_must_agree() {
        let key = ed25519_key();
        let base = record(&key);
        assert_eq!(validate_kvector_verifier_key_record_v2(base), Ok(()));
        assert_eq!(
            validate_kvector_verifier_key_record_v2(KVectorVerifierKeyRecordV2 {
                algorithm: AlgorithmId::MlDsa65,
                ..base
            }),
            Err(KVectorVerifierKeyRecordErrorV2::PublicKeyLengthInvalid)
        );
        assert_eq!(
            validate_kvector_verifier_key_record_v2(KVectorVerifierKeyRecordV2 {
                algorithm: AlgorithmId::MlKem768,
                ..base
            }),
            Err(KVectorVerifierKeyRecordErrorV2::NonSignatureAlgorithm)
        );
    }

    #[test]
    fn zero_and_malformed_keys_fail_closed() {
        let zero = [0u8; 32];
        assert_eq!(
            validate_kvector_verifier_key_record_v2(record(&zero)),
            Err(KVectorVerifierKeyRecordErrorV2::PublicKeyAllZero)
        );
        let short = [0x42; 31];
        assert_eq!(
            validate_kvector_verifier_key_record_v2(record(&short)),
            Err(KVectorVerifierKeyRecordErrorV2::PublicKeyLengthInvalid)
        );
    }

    #[test]
    fn validity_interval_and_generation_are_structural() {
        let key = ed25519_key();
        let base = record(&key);
        assert_eq!(
            validate_kvector_verifier_key_record_v2(KVectorVerifierKeyRecordV2 {
                key_generation: 0,
                ..base
            }),
            Err(KVectorVerifierKeyRecordErrorV2::KeyGenerationInvalid)
        );
        assert_eq!(
            validate_kvector_verifier_key_record_v2(KVectorVerifierKeyRecordV2 {
                valid_until_micros: base.valid_from_micros,
                ..base
            }),
            Err(KVectorVerifierKeyRecordErrorV2::ValidityIntervalInvalid)
        );
        assert_eq!(
            validate_kvector_verifier_key_record_v2(KVectorVerifierKeyRecordV2 {
                issued_at_micros: base.valid_from_micros + 1,
                ..base
            }),
            Err(KVectorVerifierKeyRecordErrorV2::IssuedAfterValidityStart)
        );
    }

    #[test]
    fn historical_use_requires_exact_identity_scheme_and_time_window() {
        let key = ed25519_key();
        let base = record(&key);
        let context = use_context();
        assert_eq!(validate_kvector_verifier_key_use_v2(base, context), Ok(()));
        assert_eq!(
            validate_kvector_verifier_key_use_v2(
                base,
                KVectorVerifierKeyUseContextV2 {
                    verifier_key_id: "verifier-key-2",
                    ..context
                }
            ),
            Err(KVectorVerifierKeyRecordErrorV2::VerifierKeyIdMismatch)
        );
        assert_eq!(
            validate_kvector_verifier_key_use_v2(
                base,
                KVectorVerifierKeyUseContextV2 {
                    signature_scheme_id: "hybrid-ed25519-ml-dsa-65-v1",
                    ..context
                }
            ),
            Err(KVectorVerifierKeyRecordErrorV2::SignatureSchemeMismatch)
        );
        assert_eq!(
            validate_kvector_verifier_key_use_v2(
                base,
                KVectorVerifierKeyUseContextV2 {
                    verified_at_micros: base.valid_until_micros,
                    ..context
                }
            ),
            Err(KVectorVerifierKeyRecordErrorV2::KeyExpiredAtVerification)
        );
    }

    #[test]
    fn hybrid_scheme_is_distinct_from_ed25519_half() {
        assert_eq!(
            kvector_verifier_signature_scheme_id_v2(AlgorithmId::HybridEd25519MlDsa65),
            Ok("hybrid-ed25519-ml-dsa-65-v1")
        );
        assert_ne!(
            kvector_verifier_signature_scheme_id_v2(AlgorithmId::HybridEd25519MlDsa65).unwrap(),
            kvector_verifier_signature_scheme_id_v2(AlgorithmId::Ed25519).unwrap()
        );
    }
}
