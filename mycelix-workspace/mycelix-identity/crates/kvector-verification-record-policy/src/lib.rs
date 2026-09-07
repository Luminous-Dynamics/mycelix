// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Backend-neutral signed verification-record body theorem for Identity V2.
//!
//! This crate defines exactly **what a verifier signs** after checking one #225
//! K-vector proof statement. It does not verify signatures, decide whether a
//! verifier/policy is trusted, read current time, or turn a record into authority.
//!
//! Security split:
//!
//! - #210 owns semantic fulfillment identity;
//! - #225 owns the exact public proof statement and statement digest;
//! - this crate binds that statement digest to exact proof bytes plus
//!   backend/circuit/verifier/policy metadata in one deterministic record body;
//! - a later adapter verifies the actual signature and current policy/key state;
//! - only that stronger adapter may produce positive authority evidence.

#![forbid(unsafe_code)]

use mycelix_attestation_fulfillment_policy::FULFILLMENT_ID_LEN_V2;
use mycelix_kvector_proof_statement_policy::{
    derive_kvector_proof_statement_digest_v2, KVectorProofPublicStatementV2,
    K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2,
};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const VERIFICATION_BACKEND_ID_MAX_LEN_V2: usize = 128;
pub const VERIFICATION_CIRCUIT_ID_MAX_LEN_V2: usize = 128;
pub const VERIFICATION_CIRCUIT_VERSION_MAX_LEN_V2: usize = 64;
pub const VERIFIER_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_KEY_ID_MAX_LEN_V2: usize = 128;
pub const SIGNATURE_SCHEME_ID_MAX_LEN_V2: usize = 64;
pub const K_VECTOR_VERIFICATION_RECORD_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verification-record:v2\0";

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

/// Canonical body whose digest a later signature adapter must authenticate.
///
/// This is a **record assertion**, not a qualified capability. An `Accepted`
/// outcome is only meaningful after a later boundary verifies the signature,
/// verifier key, accepted policy, circuit qualification and currentness.
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
    SignatureSchemeIdInvalid,
    VerificationPolicyDigestLengthInvalid,
    ValidityIntervalInvalid,
    FulfillmentIdMismatch,
    StatementDigestMismatch,
    ProofDigestMismatch,
    VerifierKeyExpiredAtVerification,
    VerificationPolicyExpiredAtVerification,
}

fn valid_bounded_text(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    valid_bounded_text(value, VERIFIER_DID_MAX_LEN_V2) && value.starts_with("did:")
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

/// Derive the canonical digest a later verifier signature must authenticate.
///
/// Signature bytes are intentionally absent from this transcript. The record body
/// digest is stable across signature encodings while still binding the declared
/// signature scheme and verifier key identity.
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
    update_len_prefixed(&mut hasher, 0x09, body.signature_scheme_id.as_bytes());

    hasher.update([0x0A]);
    hasher.update(body.verification_policy_sha256);

    hasher.update([0x0B]);
    hasher.update([body.outcome.tag()]);

    hasher.update([0x0C]);
    hasher.update(body.verified_at_micros.to_be_bytes());

    hasher.update([0x0D]);
    hasher.update(body.valid_until_micros.to_be_bytes());

    let digest = hasher.finalize();
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    Ok(out)
}

/// Bind one verification-record body to the exact #225 public statement and proof
/// bytes it claims were checked.
///
/// This does **not** verify a signature and does not decide that the verifier or
/// policy is trusted/current. It only proves structural transcript agreement.
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

/// Compute the natural upper bound for a positive verification observation after a
/// later adapter has authenticated this record and resolved verifier-key/policy
/// expiry evidence.
///
/// The result is deliberately a timestamp, not a `Current`/`Verified` boolean.
/// A caller still needs an observed time and the actual signature/policy checks.
pub fn effective_kvector_verification_valid_until_v2(
    body: KVectorProofVerificationRecordBodyV2<'_>,
    verifier_key_valid_until_micros: i64,
    verification_policy_valid_until_micros: i64,
) -> Result<i64, KVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_body_v2(body)?;

    if verifier_key_valid_until_micros <= body.verified_at_micros {
        return Err(KVectorVerificationRecordErrorV2::VerifierKeyExpiredAtVerification);
    }
    if verification_policy_valid_until_micros <= body.verified_at_micros {
        return Err(KVectorVerificationRecordErrorV2::VerificationPolicyExpiredAtVerification);
    }

    Ok(body
        .valid_until_micros
        .min(verifier_key_valid_until_micros)
        .min(verification_policy_valid_until_micros))
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_attestation_fulfillment_policy::{
        derive_attestation_fulfillment_id_v2, AttestationFulfillmentStatementV2,
    };
    use mycelix_kvector_proof_statement_policy::derive_kvector_proof_public_statement_v2;

    const PROOF: &[u8] = b"kvector-proof-v2-test";

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

    fn body<'a>(
        fulfillment_id: &'a [u8],
        statement_digest: &'a [u8],
        proof_digest: &'a [u8],
        policy_digest: &'a [u8],
    ) -> KVectorProofVerificationRecordBodyV2<'a> {
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id,
            proof_statement_sha256: statement_digest,
            proof_sha256: proof_digest,
            backend_id: "candidate-backend",
            circuit_id: "identity-kvector-v2",
            circuit_version: "0.1.0",
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "verifier-key-1",
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: policy_digest,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros: 1_700_000_000_000_000,
            valid_until_micros: 1_700_003_600_000_000,
        }
    }

    #[test]
    fn verification_record_digest_vector_is_frozen() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let policy_digest = [0x11; 32];
        assert_eq!(
            proof_digest,
            [
                0xe9, 0x06, 0x44, 0x17, 0x7e, 0xbc, 0xbb, 0x04, 0x9d, 0xab, 0xa9, 0xdf,
                0xd4, 0xfc, 0x94, 0xae, 0xe5, 0x9d, 0xe5, 0x74, 0x67, 0xe1, 0xf6, 0x4e,
                0xec, 0xd9, 0x7d, 0x67, 0x5c, 0x4a, 0x8a, 0x4f,
            ]
        );
        let actual = derive_kvector_verification_record_signing_digest_v2(body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &policy_digest,
        ))
        .unwrap();
        assert_eq!(
            actual,
            [
                0x77, 0x99, 0x7a, 0x3e, 0x57, 0x93, 0xbe, 0xf2, 0x62, 0xb3, 0x4b, 0x68,
                0x64, 0x1b, 0xd9, 0xca, 0xd0, 0x9f, 0x31, 0x85, 0x61, 0x06, 0x57, 0x80,
                0xcf, 0x5e, 0xf0, 0xf5, 0x03, 0x17, 0x6c, 0x84,
            ]
        );
    }

    #[test]
    fn record_binds_exact_statement_and_proof() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &policy_digest,
        );
        assert_eq!(
            validate_kvector_verification_record_binding_v2(record, public, PROOF),
            Ok(())
        );
        assert_eq!(
            validate_kvector_verification_record_binding_v2(record, public, b"other-proof"),
            Err(KVectorVerificationRecordErrorV2::ProofDigestMismatch)
        );

        let wrong_statement_digest = [0u8; 32];
        let wrong = KVectorProofVerificationRecordBodyV2 {
            proof_statement_sha256: &wrong_statement_digest,
            ..record
        };
        assert_eq!(
            validate_kvector_verification_record_binding_v2(wrong, public, PROOF),
            Err(KVectorVerificationRecordErrorV2::StatementDigestMismatch)
        );
    }

    #[test]
    fn rejected_record_is_structurally_valid_but_not_equivalent_to_accepted() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let policy_digest = [0x11; 32];
        let accepted = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &policy_digest,
        );
        let rejected = KVectorProofVerificationRecordBodyV2 {
            outcome: KVectorProofVerificationOutcomeV2::Rejected,
            ..accepted
        };
        assert_eq!(validate_kvector_verification_record_body_v2(rejected), Ok(()));
        assert_ne!(
            derive_kvector_verification_record_signing_digest_v2(accepted),
            derive_kvector_verification_record_signing_digest_v2(rejected)
        );
    }

    #[test]
    fn natural_validity_is_capped_by_record_key_and_policy_expiry() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
            &policy_digest,
        );
        assert_eq!(
            effective_kvector_verification_valid_until_v2(
                record,
                1_700_002_000_000_000,
                1_700_001_000_000_000,
            ),
            Ok(1_700_001_000_000_000)
        );
        assert_eq!(
            effective_kvector_verification_valid_until_v2(
                record,
                record.verified_at_micros,
                1_700_001_000_000_000,
            ),
            Err(KVectorVerificationRecordErrorV2::VerifierKeyExpiredAtVerification)
        );
    }

    #[test]
    fn malformed_record_fields_fail_closed() {
        let public = public_statement();
        let statement_digest = derive_kvector_proof_statement_digest_v2(public);
        let proof_digest = sha256_v2(PROOF);
        let policy_digest = [0x11; 32];
        let record = body(
            &public.fulfillment_id,
            &statement_digest,
            &proof_digest,
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
                verifier_did: "verifier",
                ..record
            }),
            Err(KVectorVerificationRecordErrorV2::VerifierDidInvalid)
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