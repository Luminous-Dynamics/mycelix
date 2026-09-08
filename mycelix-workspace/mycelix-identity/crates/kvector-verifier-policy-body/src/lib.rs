// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Backend-neutral static verifier-policy body theorem for Identity V2 K-vector evidence.
//!
//! This is the repaired-stack convergence port of #248. The production policy
//! semantics and digest are unchanged; only test fixtures are updated for #324's
//! generation-bound verification-record shape.
//!
//! This crate freezes policy contents and static record compatibility only. It does not
//! decide who may publish a policy, whether a policy is accepted/current, or whether a
//! proof/signature is authentic.

#![forbid(unsafe_code)]

use mycelix_kvector_proof_statement_policy::K_VECTOR_PROOF_STATEMENT_DOMAIN_V2;
use mycelix_kvector_verification_record_policy::{
    validate_kvector_verification_record_body_v2, KVectorProofVerificationRecordBodyV2,
    SHA256_DIGEST_LEN_V2,
};
use sha2::{Digest, Sha256};

pub const K_VECTOR_VERIFIER_POLICY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verifier-policy:v2\0";
pub const VERIFIER_POLICY_ID_MAX_LEN_V2: usize = 128;
pub const VERIFIER_POLICY_VERSION_MAX_LEN_V2: usize = 64;
pub const VERIFIER_POLICY_BACKEND_ID_MAX_LEN_V2: usize = 128;
pub const VERIFIER_POLICY_CIRCUIT_ID_MAX_LEN_V2: usize = 128;
pub const VERIFIER_POLICY_CIRCUIT_VERSION_MAX_LEN_V2: usize = 64;
pub const VERIFIER_POLICY_DID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_POLICY_KEY_ID_MAX_LEN_V2: usize = 256;
pub const VERIFIER_POLICY_SIGNATURE_SCHEME_MAX_LEN_V2: usize = 128;

#[derive(Debug, Clone, Copy)]
pub struct KVectorVerifierPolicyBodyV2<'a> {
    pub policy_id: &'a str,
    pub policy_version: &'a str,
    pub backend_id: &'a str,
    pub circuit_id: &'a str,
    pub circuit_version: &'a str,
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub signature_scheme_id: &'a str,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
    pub max_record_lifetime_micros: i64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorVerifierPolicyBodyErrorV2 {
    PolicyIdInvalid,
    PolicyVersionInvalid,
    BackendIdInvalid,
    CircuitIdInvalid,
    CircuitVersionInvalid,
    VerifierDidInvalid,
    VerifierKeyIdInvalid,
    SignatureSchemeIdInvalid,
    ValidityIntervalInvalid,
    MaxRecordLifetimeInvalid,
    VerificationRecordInvalid,
    PolicyDigestMismatch,
    BackendMismatch,
    CircuitIdMismatch,
    CircuitVersionMismatch,
    VerifierDidMismatch,
    VerifierKeyIdMismatch,
    SignatureSchemeMismatch,
    VerificationBeforePolicyValidity,
    VerificationAfterPolicyValidity,
    RecordOutlivesPolicy,
    RecordLifetimeExceedsPolicy,
}

fn bounded_nonempty(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    bounded_nonempty(value, VERIFIER_POLICY_DID_MAX_LEN_V2) && value.starts_with("did:")
}

pub fn validate_kvector_verifier_policy_body_v2(
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<(), KVectorVerifierPolicyBodyErrorV2> {
    if !bounded_nonempty(policy.policy_id, VERIFIER_POLICY_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierPolicyBodyErrorV2::PolicyIdInvalid);
    }
    if !bounded_nonempty(policy.policy_version, VERIFIER_POLICY_VERSION_MAX_LEN_V2) {
        return Err(KVectorVerifierPolicyBodyErrorV2::PolicyVersionInvalid);
    }
    if !bounded_nonempty(policy.backend_id, VERIFIER_POLICY_BACKEND_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierPolicyBodyErrorV2::BackendIdInvalid);
    }
    if !bounded_nonempty(policy.circuit_id, VERIFIER_POLICY_CIRCUIT_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierPolicyBodyErrorV2::CircuitIdInvalid);
    }
    if !bounded_nonempty(
        policy.circuit_version,
        VERIFIER_POLICY_CIRCUIT_VERSION_MAX_LEN_V2,
    ) {
        return Err(KVectorVerifierPolicyBodyErrorV2::CircuitVersionInvalid);
    }
    if !valid_did(policy.verifier_did) {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerifierDidInvalid);
    }
    if !bounded_nonempty(policy.verifier_key_id, VERIFIER_POLICY_KEY_ID_MAX_LEN_V2) {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerifierKeyIdInvalid);
    }
    if !bounded_nonempty(
        policy.signature_scheme_id,
        VERIFIER_POLICY_SIGNATURE_SCHEME_MAX_LEN_V2,
    ) {
        return Err(KVectorVerifierPolicyBodyErrorV2::SignatureSchemeIdInvalid);
    }
    if policy.valid_until_micros <= policy.valid_from_micros {
        return Err(KVectorVerifierPolicyBodyErrorV2::ValidityIntervalInvalid);
    }
    let policy_lifetime = policy.valid_until_micros - policy.valid_from_micros;
    if policy.max_record_lifetime_micros <= 0
        || policy.max_record_lifetime_micros > policy_lifetime
    {
        return Err(KVectorVerifierPolicyBodyErrorV2::MaxRecordLifetimeInvalid);
    }
    Ok(())
}

fn update_len_prefixed(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    let len = value.len() as u16;
    hasher.update(len.to_be_bytes());
    hasher.update(value);
}

pub fn derive_kvector_verifier_policy_digest_v2(
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], KVectorVerifierPolicyBodyErrorV2> {
    validate_kvector_verifier_policy_body_v2(policy)?;

    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_VERIFIER_POLICY_DOMAIN_V2);
    update_len_prefixed(&mut hasher, 0x01, policy.policy_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x02, policy.policy_version.as_bytes());
    update_len_prefixed(&mut hasher, 0x03, policy.backend_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x04, policy.circuit_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x05, policy.circuit_version.as_bytes());
    update_len_prefixed(&mut hasher, 0x06, policy.verifier_did.as_bytes());
    update_len_prefixed(&mut hasher, 0x07, policy.verifier_key_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x08, policy.signature_scheme_id.as_bytes());
    update_len_prefixed(&mut hasher, 0x09, K_VECTOR_PROOF_STATEMENT_DOMAIN_V2);
    hasher.update([0x0a]);
    hasher.update(policy.valid_from_micros.to_be_bytes());
    hasher.update([0x0b]);
    hasher.update(policy.valid_until_micros.to_be_bytes());
    hasher.update([0x0c]);
    hasher.update(policy.max_record_lifetime_micros.to_be_bytes());

    Ok(hasher.finalize().into())
}

pub fn validate_kvector_verification_record_against_policy_body_v2(
    record: KVectorProofVerificationRecordBodyV2<'_>,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<(), KVectorVerifierPolicyBodyErrorV2> {
    validate_kvector_verification_record_body_v2(record)
        .map_err(|_| KVectorVerifierPolicyBodyErrorV2::VerificationRecordInvalid)?;
    let digest = derive_kvector_verifier_policy_digest_v2(policy)?;

    if record.verification_policy_sha256 != digest.as_slice() {
        return Err(KVectorVerifierPolicyBodyErrorV2::PolicyDigestMismatch);
    }
    if record.backend_id != policy.backend_id {
        return Err(KVectorVerifierPolicyBodyErrorV2::BackendMismatch);
    }
    if record.circuit_id != policy.circuit_id {
        return Err(KVectorVerifierPolicyBodyErrorV2::CircuitIdMismatch);
    }
    if record.circuit_version != policy.circuit_version {
        return Err(KVectorVerifierPolicyBodyErrorV2::CircuitVersionMismatch);
    }
    if record.verifier_did != policy.verifier_did {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerifierDidMismatch);
    }
    if record.verifier_key_id != policy.verifier_key_id {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerifierKeyIdMismatch);
    }
    if record.signature_scheme_id != policy.signature_scheme_id {
        return Err(KVectorVerifierPolicyBodyErrorV2::SignatureSchemeMismatch);
    }
    if record.verified_at_micros < policy.valid_from_micros {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerificationBeforePolicyValidity);
    }
    if record.verified_at_micros >= policy.valid_until_micros {
        return Err(KVectorVerifierPolicyBodyErrorV2::VerificationAfterPolicyValidity);
    }
    if record.valid_until_micros > policy.valid_until_micros {
        return Err(KVectorVerifierPolicyBodyErrorV2::RecordOutlivesPolicy);
    }
    let record_lifetime = record.valid_until_micros - record.verified_at_micros;
    if record_lifetime > policy.max_record_lifetime_micros {
        return Err(KVectorVerifierPolicyBodyErrorV2::RecordLifetimeExceedsPolicy);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_kvector_verification_record_policy::KVectorProofVerificationOutcomeV2;

    const POLICY_DIGEST_VECTOR: [u8; SHA256_DIGEST_LEN_V2] = [
        0x6c, 0x2d, 0x3f, 0x00, 0x20, 0x61, 0xe6, 0xf7, 0xe2, 0x21, 0x2e, 0x65, 0xe2, 0x78,
        0xcd, 0xe6, 0x58, 0x1b, 0xfc, 0x3d, 0x21, 0xaa, 0x95, 0xc7, 0x81, 0x20, 0xd8, 0x8f,
        0xab, 0x03, 0x7b, 0x43,
    ];

    fn policy() -> KVectorVerifierPolicyBodyV2<'static> {
        KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:kvector-prod-v2",
            policy_version: "2.0.0",
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#hybrid-1",
            signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
            max_record_lifetime_micros: 2_000_000,
        }
    }

    fn record<'a>(policy_digest: &'a [u8]) -> KVectorProofVerificationRecordBodyV2<'a> {
        static FULFILLMENT: [u8; 32] = [0x11; 32];
        static STATEMENT: [u8; 32] = [0x22; 32];
        static PROOF: [u8; 32] = [0x33; 32];
        static GENERATION: [u8; 32] = [0x44; 32];
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id: &FULFILLMENT,
            proof_statement_sha256: &STATEMENT,
            proof_sha256: &PROOF,
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#hybrid-1",
            verifier_key_generation_sha256: &GENERATION,
            signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
            verification_policy_sha256: policy_digest,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros: 2_000_000,
            valid_until_micros: 4_000_000,
        }
    }

    #[test]
    fn policy_digest_vector_is_frozen() {
        assert_eq!(
            derive_kvector_verifier_policy_digest_v2(policy()),
            Ok(POLICY_DIGEST_VECTOR)
        );
    }

    #[test]
    fn exact_record_tuple_matches_policy_body() {
        let digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(record(&digest), policy()),
            Ok(())
        );
    }

    #[test]
    fn policy_digest_and_static_tuple_mismatch_fail_closed() {
        let digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let wrong_digest = [0u8; SHA256_DIGEST_LEN_V2];
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(record(&wrong_digest), policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::PolicyDigestMismatch)
        );

        let mut wrong_backend = record(&digest);
        wrong_backend.backend_id = "other-backend";
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(wrong_backend, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::BackendMismatch)
        );

        let mut wrong_verifier = record(&digest);
        wrong_verifier.verifier_key_id = "did:mycelix:verifier#other";
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(wrong_verifier, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::VerifierKeyIdMismatch)
        );
    }

    #[test]
    fn record_validity_must_fit_inside_policy() {
        let digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();

        let mut before = record(&digest);
        before.verified_at_micros = 999_999;
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(before, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::VerificationBeforePolicyValidity)
        );

        let mut after = record(&digest);
        after.verified_at_micros = 9_000_000;
        after.valid_until_micros = 9_500_000;
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(after, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::VerificationAfterPolicyValidity)
        );

        let mut outlives = record(&digest);
        outlives.valid_until_micros = 9_000_001;
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(outlives, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::RecordOutlivesPolicy)
        );

        let mut too_long = record(&digest);
        too_long.valid_until_micros = 4_000_001;
        assert_eq!(
            validate_kvector_verification_record_against_policy_body_v2(too_long, policy()),
            Err(KVectorVerifierPolicyBodyErrorV2::RecordLifetimeExceedsPolicy)
        );
    }

    #[test]
    fn malformed_policy_fails_before_matching() {
        let malformed = KVectorVerifierPolicyBodyV2 {
            verifier_did: "not-a-did",
            ..policy()
        };
        assert_eq!(
            derive_kvector_verifier_policy_digest_v2(malformed),
            Err(KVectorVerifierPolicyBodyErrorV2::VerifierDidInvalid)
        );
    }
}
