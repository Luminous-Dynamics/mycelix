// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Key-generation-bound signed verification-record theorem for Identity V2.
//!
//! This is the corrected successor to the earlier #235 draft. The record now commits
//! to one exact verifier-key generation digest before any cryptographic signature is
//! attempted. A reusable DID key label is therefore no longer the complete signing-key
//! identity.
//!
//! This crate still does not verify signatures, proof-backend execution, policy
//! authority/currentness, DID currentness, or positive verification authority.

#![forbid(unsafe_code)]

use mycelix_attestation_fulfillment_policy::FULFILLMENT_ID_LEN_V2;
use mycelix_kvector_proof_statement_policy::{
    derive_kvector_proof_statement_digest_v2, KVectorProofPublicStatementV2,
    K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2,
};
use mycelix_kvector_verifier_key_generation_policy::{
    derive_kvector_verifier_key_generation_digest_v2,
    validate_kvector_verifier_key_historical_use_v2, KVectorVerifierKeyGenerationErrorV2,
    KVectorVerifierKeyGenerationV2, KVectorVerifierKeyHistoricalUseV2,
};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const VERIFICATION_BACKEND_ID_MAX_LEN_V2: usize = 128;
pub const VERIFICATION_CIRCUIT_ID_MAX_LEN_V2: usize = 128;
pub const VERIFICATION_CIRCUIT_VERSION_MAX_LEN_V2: usize = 64;
pub const VERIFIER_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_KEY_ID_MAX_LEN_V2: usize = 256;
pub const SIGNATURE_SCHEME_ID_MAX_LEN_V2: usize = 64;

/// This draft V2 protocol was not yet qualified/released when #311 exposed the missing
/// generation binding. The repaired transcript therefore uses an explicitly stronger
/// domain so an old pre-repair digest cannot be confused with the generation-bound V2
/// transcript even if both artifacts coexist during review.
pub const K_VECTOR_VERIFICATION_RECORD_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verification-record:key-generation-bound:v2\0";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorProofVerificationOutcomeV2 {
    Accepted,
    Rejected,
}

impl KVectorProofVerificationOutcomeV2 {
    const fn tag(self) -> u8 {
        match self {
            Self::Accepted => 0x01,
            Self::Rejected => 0x02,
        }
    }
}

/// Canonical assertion whose digest a later cryptographic signature must authenticate.
///
/// `verifier_key_generation_sha256` commits the record to one exact generation from
/// #316: exact public-key bytes, algorithm, generation number and historical validity
/// interval. The human-readable key ID remains useful routing metadata but is no longer
/// sufficient signing-key identity.
#[derive(Debug, Clone, Copy)]
pub struct KVectorProofVerificationRecordBodyV2<'a> {
    pub fulfillment_id: &'a [u8],
    pub proof_statement_sha256: &'a [u8],
    pub proof_sha256: &'a [u8],
    pub backend_id: &'a str,
    pub circuit_id: &'a str,
    pub circuit_version: &'a str,
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub verifier_key_generation_sha256: &'a [u8],
    pub signature_scheme_id: &'a str,
    pub verification_policy_sha256: &'a [u8],
    pub outcome: KVectorProofVerificationOutcomeV2,
    pub verified_at_micros: i64,
    pub valid_until_micros: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorVerificationRecordErrorV2 {
    FulfillmentIdLengthInvalid,
    StatementDigestLengthInvalid,
    ProofDigestLengthInvalid,
    BackendIdInvalid,
    CircuitIdInvalid,
    CircuitVersionInvalid,
    VerifierDidInvalid,
    VerifierKeyIdInvalid,
    VerifierKeyIdNotCanonical,
    VerifierKeyGenerationDigestLengthInvalid,
    SignatureSchemeIdInvalid,
    VerificationPolicyDigestLengthInvalid,
    ValidityIntervalInvalid,
    FulfillmentIdMismatch,
    StatementDigestMismatch,
    ProofDigestMismatch,
    VerifierKeyGenerationDigestMismatch,
    VerifierKeyGeneration(KVectorVerifierKeyGenerationErrorV2),
    VerificationPolicyExpiredAtVerification,
}

fn valid_bounded_text(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    valid_bounded_text(value, VERIFIER_DID_MAX_LEN_V2) && value.starts_with("did:mycelix:")
}

fn valid_canonical_key_id(verifier_did: &str, verifier_key_id: &str) -> bool {
    if !valid_bounded_text(verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return false;
    }
    let prefix = format!("{verifier_did}#");
    verifier_key_id.starts_with(&prefix) && verifier_key_id.len() > prefix.len()
}

fn sha256_v2(bytes: &[u8]) -> [u8; SHA256_DIGEST_LEN_V2] {
    let digest = Sha256::digest(bytes);
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    out
}

pub fn validate_kvector_verification_record_body_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
) -> Result<(), KVectorVerificationRecordErrorV2> {
    if body.fulfillment_id.len() != FULFILLMENT_ID_LEN_V2 {
        return Err(KVectorVerificationRecordErrorV2::FulfillmentIdLengthInvalid);
    }
    if body.proof_statement_sha256.len() != K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2 {
        return Err(KVectorVerificationRecordErrorV2::StatementDigestLengthInvalid);
    }
    if body.proof_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(KVectorVerificationRecordErrorV2::ProofDigestLengthInvalid);
    }
    if !valid_bounded_text(body.backend_id, VERIFICATION_BACKEND_ID_MAX_LEN_V2) {
        return Err(KVectorVerificationRecordErrorV2::BackendIdInvalid);
    }
    if !valid_bounded_text(body.circuit_id, VERIFICATION_CIRCUIT_ID_MAX_LEN_V2) {
        return Err(KVectorVerificationRecordErrorV2::CircuitIdInvalid);
    }
    if !valid_bounded_text(
        body.circuit_version,
        VERIFICATION_CIRCUIT_VERSION_MAX_LEN_V2,
    ) {
        return Err(KVectorVerificationRecordErrorV2::CircuitVersionInvalid);
    }
    if !valid_did(body.verifier_did) {
        return Err(KVectorVerificationRecordErrorV2::VerifierDidInvalid);
    }
    if !valid_bounded_text(body.verifier_key_id, VERIFIER_KEY_ID_MAX_LEN_V2) {
        return Err(KVectorVerificationRecordErrorV2::VerifierKeyIdInvalid);
    }
    if !valid_canonical_key_id(body.verifier_did, body.verifier_key_id) {
        return Err(KVectorVerificationRecordErrorV2::VerifierKeyIdNotCanonical);
    }
    if body.verifier_key_generation_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(
            KVectorVerificationRecordErrorV2::VerifierKeyGenerationDigestLengthInvalid,
        );
    }
    if !valid_bounded_text(
        body.signature_scheme_id,
        SIGNATURE_SCHEME_ID_MAX_LEN_V2,
    ) {
        return Err(KVectorVerificationRecordErrorV2::SignatureSchemeIdInvalid);
    }
    if body.verification_policy_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(KVectorVerificationRecordErrorV2::VerificationPolicyDigestLengthInvalid);
    }
    if body.valid_until_micros <= body.verified_at_micros {
        return Err(KVectorVerificationRecordErrorV2::ValidityIntervalInvalid);
    }
    Ok(())
}

fn update_len_prefixed(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    let len = value.len() as u16;
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

/// Derive the exact digest a later verifier signature must authenticate.
///
/// Every record field participates, including the exact verifier-key generation digest.
pub fn derive_kvector_verification_record_signing_digest_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], KVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_body_v2(body)?;

    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_VERIFICATION_RECORD_DOMAIN_V2);

    hasher.update([0x01]);
    hasher.update(body.fulfillment_id);
    hasher.update([0x02]);
    hasher.update(body.proof_statement_sha256);
    hasher.update([0x03]);
    hasher.update(body.proof_sha256);

    update_len_prefixed(&mut hasher, 0x04, body.backend_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x05, body.circuit_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x06, body.circuit_version.as_bytes());
    update_len_prefixed(&mut hasher, 0x07, body.verifier_did.as_bytes());
    update_len_prefixed(&mut hasher, 0x08, body.verifier_key_id.as_bytes());

    hasher.update([0x09]);
    hasher.update(body.verifier_key_generation_sha256);

    update_len_prefixed(&mut hasher, 0x0A, body.signature_scheme_id.as_bytes());

    hasher.update([0x0B]);
    hasher.update(body.verification_policy_sha256);
    hasher.update([0x0C]);
    hasher.update([body.outcome.tag()]);
    hasher.update([0x0D]);
    hasher.update(body.verified_at_micros.to_be_bytes());
    hasher.update([0x0E]);
    hasher.update(body.valid_until_micros.to_be_bytes());

    let digest = hasher.finalize();
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    Ok(out)
}

/// Bind one record body to the exact #225 public statement and proof bytes it claims
/// were checked. This remains transcript agreement, not proof/signature authenticity.
pub fn validate_kvector_verification_record_binding_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
    public_statement: KVectorProofPublicStatementV2,
    proof_bytes: &[u8],
) -> Result<(), KVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_body_v2(body)?;

    if body.fulfillment_id != public_statement.fulfillment_id.as_slice() {
        return Err(KVectorVerificationRecordErrorV2::FulfillmentIdMismatch);
    }
    let statement_digest = derive_kvector_proof_statement_digest_v2(public_statement);
    if body.proof_statement_sha256 != statement_digest.as_slice() {
        return Err(KVectorVerificationRecordErrorV2::StatementDigestMismatch);
    }
    let proof_digest = sha256_v2(proof_bytes);
    if body.proof_sha256 != proof_digest.as_slice() {
        return Err(KVectorVerificationRecordErrorV2::ProofDigestMismatch);
    }
    Ok(())
}

/// Bind the signed record to one exact #316 key generation and prove that generation
/// covered the record's asserted verification instant.
///
/// The record does not duplicate an algorithm field. The generation digest commits the
/// algorithm; later #250 policy/material qualification must independently require the
/// policy signature scheme to map to the same algorithm before crypto execution.
pub fn validate_kvector_verification_record_key_generation_binding_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<(), KVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_body_v2(body)?;

    let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation)
        .map_err(KVectorVerificationRecordErrorV2::VerifierKeyGeneration)?;
    if body.verifier_key_generation_sha256 != generation_digest.as_slice() {
        return Err(KVectorVerificationRecordErrorV2::VerifierKeyGenerationDigestMismatch);
    }

    validate_kvector_verifier_key_historical_use_v2(
        generation,
        KVectorVerifierKeyHistoricalUseV2 {
            verifier_did: body.verifier_did,
            verifier_key_id: body.verifier_key_id,
            algorithm: generation.algorithm,
            verified_at_micros: body.verified_at_micros,
        },
    )
    .map_err(KVectorVerificationRecordErrorV2::VerifierKeyGeneration)
}

/// Compute the natural upper bound for later authenticated/authorized evidence.
///
/// Unlike the older draft, verifier-key expiry is not accepted as an independent scalar
/// supplied by the caller. The exact generation is re-bound first and its own validity
/// horizon is then intersected with record and policy horizons.
pub fn effective_kvector_verification_valid_until_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
    generation: KVectorVerifierKeyGenerationV2<'_>,
    verification_policy_valid_until_micros: i64,
) -> Result<i64, KVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_key_generation_binding_v2(body, generation)?;

    if verification_policy_valid_until_micros <= body.verified_at_micros {
        return Err(KVectorVerificationRecordErrorV2::VerificationPolicyExpiredAtVerification);
    }

    Ok(body
        .valid_until_micros
        .min(generation.valid_until_micros)
        .min(verification_policy_valid_until_micros))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_attestation_fulfillment_policy::{
        derive_attestation_fulfillment_id_v2, AttestationFulfillmentStatementV2,
    };
    use mycelix_kvector_proof_statement_policy::derive_kvector_proof_public_statement_v2;
    use mycelix_kvector_verifier_key_generation_policy::KVectorVerifierKeyGenerationV2;
    use mycelix_crypto::AlgorithmId;

    const PROOF: &[u8] = b"kvector-proof-v2-test";
    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    fn root_raw39() -> [u8; 39] {
        let mut root = [0u8; 39];
        for (index, byte) in root.iter_mut().enumerate() {
            *byte = index as u8;
        }
        root
    }

    fn commitment() -> [u8; 32] {
        [
            0x83, 0x44, 0xa1, 0x26, 0xd8, 0xb0, 0x0e, 0xdb, 0x36, 0xc4, 0x25, 0x3e,
            0x88, 0x1d, 0x24, 0x8f, 0x3b, 0x1f, 0xb1, 0x58, 0x70, 0xed, 0xc2, 0x26,
            0x14, 0x80, 0x18, 0x20, 0x73, 0xa6, 0x36, 0x22,
        ]
    }

    fn public_statement() -> KVectorProofPublicStatementV2 {
        let root = root_raw39();
        let commitment = commitment();
        let fulfillment = AttestationFulfillmentStatementV2 {
            request_root_raw39: &root,
            subject_did: "did:mycelix:subject",
            kvector_commitment: &commitment,
            trust_score_lower_scaled: 600_000,
            trust_score_upper_scaled: 700_000,
            credential_expires_at_micros: Some(1_700_000_000_000_000),
        };
        let expected_id = derive_attestation_fulfillment_id_v2(fulfillment).unwrap();
        assert_eq!(
            expected_id,
            [
                0x33, 0x29, 0x24, 0x63, 0x4f, 0x10, 0x32, 0x84, 0xb2, 0x21, 0x13, 0xb3,
                0x05, 0x6b, 0x59, 0xe9, 0x24, 0x62, 0x68, 0x80, 0x05, 0x01, 0x1a, 0x40,
                0xe5, 0x6f, 0xd0, 0x6d, 0xc1, 0x23, 0xac, 0x6f,
            ]
        );
        derive_kvector_proof_public_statement_v2(fulfillment).unwrap()
    }

    fn key_generation<'a>(key: &'a [u8]) -> KVectorVerifierKeyGenerationV2<'a> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: key,
            key_generation: 1,
            issued_at_micros: 1_699_999_000_000_000,
            valid_from_micros: 1_699_999_500_000_000,
            valid_until_micros: 1_700_002_000_000_000,
        }
    }

    fn body<'a>(
        fulfillment_id: &'a [u8],
        statement_digest: &'a [u8],
        proof_digest: &'a [u8],
        key_generation_digest: &'a [u8],
        policy_digest: &'a [u8],
    ) -> KVectorProofVerificationRecordBodyV2<'a> {
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id,
            proof_statement_sha256: statement_digest,
            proof_sha256: proof_digest,
            backend_id: "candidate-backend",
            circuit_id: "identity-kvector-v2",
            circuit_version: "0.1.0",
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            verifier_key_generation_sha256: key_generation_digest,
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: policy_digest,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros: 1_700_000_000_000_000,
            valid_until_micros: 1_700_003_600_000_000,
        }
    }

    #[test]
    fn generation_bound_verification_record_digest_vector_is_frozen() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let generation = key_generation(&key);
        let generation_digest =
            derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        assert_eq!(
            generation_digest,
            [
                0x18, 0x0e, 0xc0, 0x40, 0xc4, 0x2a, 0xec, 0x94, 0x02, 0x26, 0x31, 0x78,
                0xf2, 0x44, 0xea, 0xaa, 0x0a, 0x44, 0x64, 0xd2, 0x86, 0x7d, 0xd3, 0x2a,
                0x37, 0x00, 0x22, 0xe8, 0x83, 0x04, 0x86, 0xe3,
            ]
        );
        let policy_digest = [0x11; 32];
        let actual = derive_kvector_verification_record_signing_digest_v2(body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &generation_digest,
            &policy_digest,
        ))
        .unwrap();
        assert_eq!(
            actual,
            [
                0x41, 0x32, 0x09, 0x4c, 0xfc, 0xa1, 0x97, 0x2e, 0x47, 0x1b, 0xf2, 0x29,
                0xd9, 0x36, 0xb2, 0x81, 0x1a, 0xbb, 0x07, 0xad, 0x88, 0xfa, 0xfe, 0x19,
                0xc8, 0x62, 0x70, 0xb4, 0xd4, 0x61, 0x92, 0x41,
            ]
        );
    }

    #[test]
    fn record_binds_exact_statement_proof_and_generation() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let generation = key_generation(&key);
        let generation_digest =
            derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &generation_digest,
            &policy_digest,
        );

        assert_eq!(
            validate_kvector_verification_record_binding_v2(record, public, PROOF),
            Ok(())
        );
        assert_eq!(
            validate_kvector_verification_record_key_generation_binding_v2(record, generation),
            Ok(())
        );

        let other_key = [0x43; 32];
        let other_generation = key_generation(&other_key);
        assert_eq!(
            validate_kvector_verification_record_key_generation_binding_v2(
                record,
                other_generation
            ),
            Err(KVectorVerificationRecordErrorV2::VerifierKeyGenerationDigestMismatch)
        );
    }

    #[test]
    fn key_generation_change_changes_signed_record_digest() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let first = key_generation(&key);
        let second = KVectorVerifierKeyGenerationV2 {
            key_generation: 2,
            ..first
        };
        let first_digest = derive_kvector_verifier_key_generation_digest_v2(first).unwrap();
        let second_digest = derive_kvector_verifier_key_generation_digest_v2(second).unwrap();
        let policy_digest = [0x11; 32];

        let first_record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &first_digest,
            &policy_digest,
        );
        let second_record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &second_digest,
            &policy_digest,
        );
        assert_ne!(
            derive_kvector_verification_record_signing_digest_v2(first_record).unwrap(),
            derive_kvector_verification_record_signing_digest_v2(second_record).unwrap()
        );
    }

    #[test]
    fn backdated_record_cannot_bind_to_later_generation() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let later = KVectorVerifierKeyGenerationV2 {
            valid_from_micros: 1_700_001_000_000_000,
            issued_at_micros: 1_700_000_900_000_000,
            valid_until_micros: 1_700_004_000_000_000,
            ..key_generation(&key)
        };
        let later_digest = derive_kvector_verifier_key_generation_digest_v2(later).unwrap();
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &later_digest,
            &policy_digest,
        );

        assert_eq!(
            validate_kvector_verification_record_key_generation_binding_v2(record, later),
            Err(KVectorVerificationRecordErrorV2::VerifierKeyGeneration(
                KVectorVerifierKeyGenerationErrorV2::KeyNotYetValidAtVerification
            ))
        );
    }

    #[test]
    fn natural_validity_uses_exact_generation_not_caller_key_expiry_scalar() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let generation = key_generation(&key);
        let generation_digest =
            derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &generation_digest,
            &policy_digest,
        );

        assert_eq!(
            effective_kvector_verification_valid_until_v2(
                record,
                generation,
                1_700_001_000_000_000,
            ),
            Ok(1_700_001_000_000_000)
        );
    }

    #[test]
    fn malformed_record_fields_fail_closed() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let key = [0x42; 32];
        let generation = key_generation(&key);
        let generation_digest =
            derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &generation_digest,
            &policy_digest,
        );

        assert_eq!(
            validate_kvector_verification_record_body_v2(KVectorProofVerificationRecordBodyV2 {
                backend_id: "",
                ..record
            }),
            Err(KVectorVerificationRecordErrorV2::BackendIdInvalid)
        );
        assert_eq!(
            validate_kvector_verification_record_body_v2(KVectorProofVerificationRecordBodyV2 {
                verifier_key_id: "#key-1",
                ..record
            }),
            Err(KVectorVerificationRecordErrorV2::VerifierKeyIdNotCanonical)
        );
        assert_eq!(
            validate_kvector_verification_record_body_v2(KVectorProofVerificationRecordBodyV2 {
                valid_until_micros: record.verified_at_micros,
                ..record
            }),
            Err(KVectorVerificationRecordErrorV2::ValidityIntervalInvalid)
        );
    }
}
