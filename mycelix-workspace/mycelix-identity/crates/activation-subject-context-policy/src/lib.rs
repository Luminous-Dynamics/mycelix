// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cross-capability activation-subject composition for Identity V2.
//!
//! This pure layer proves that the exact #374 record body, #379 pure pre-crypto context,
//! #385 runtime-bound context, and #392 exact historical observation all refer to the
//! same verification-record subject before trusted activation (#337) may begin.
//!
//! Success means only "this exact historically observed generation is the generation
//! selected by this exact runtime-bound structural record context". No trusted time,
//! signature authenticity, policy currentness, or positive evidence is established here.

#![forbid(unsafe_code)]

use identity_security_observer_v2::QualifiedExactHistoricalGenerationObservationV2;
use mycelix_authority_scoped_kvector_verification_record_policy::{
    derive_authority_scoped_kvector_verification_record_signing_digest_v2,
    AuthorityScopedKVectorProofVerificationRecordBodyV2,
    AuthorityScopedKVectorVerificationRecordErrorV2,
};
use mycelix_authority_scoped_precrypto_context_policy::QualifiedAuthorityScopedPrecryptoContextV2;
use mycelix_crypto::AlgorithmId;
use mycelix_identity_runtime_precrypto_context_adapter::RuntimeBoundAuthorityScopedPrecryptoContextV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const ACTIVATION_SUBJECT_CONTEXT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:activation-subject-context:v2\0";

#[derive(Debug)]
pub struct QualifiedHistoricalActivationSubjectContextV2 {
    authority_domain_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    record_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    precrypto_context_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    runtime_bound_context_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    exact_observation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    activation_subject_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    generation_action_id: String,
    did_document_action_id: String,
    coverage_head_action_seq: u32,
    coverage_head_action_id: String,
    verifier_did: String,
    verifier_key_id: String,
    key_generation: u64,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    signature_scheme_id: String,
    record_verified_at_micros: i64,
    record_valid_until_micros: i64,
    generation_valid_from_micros: i64,
    generation_valid_until_micros: i64,
}

impl QualifiedHistoricalActivationSubjectContextV2 {
    pub fn authority_domain_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }

    pub fn record_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.record_signing_digest_sha256
    }

    pub fn precrypto_context_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.precrypto_context_digest_sha256
    }

    pub fn runtime_bound_context_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.runtime_bound_context_digest_sha256
    }

    pub fn exact_observation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.exact_observation_digest_sha256
    }

    pub fn scoped_generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_generation_digest_sha256
    }

    pub fn scoped_policy_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_policy_digest_sha256
    }

    pub fn generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.generation_sha256
    }

    pub fn activation_subject_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.activation_subject_digest_sha256
    }

    pub fn generation_action_id(&self) -> &str {
        &self.generation_action_id
    }

    pub fn did_document_action_id(&self) -> &str {
        &self.did_document_action_id
    }

    pub fn coverage_head_action_seq(&self) -> u32 {
        self.coverage_head_action_seq
    }

    pub fn coverage_head_action_id(&self) -> &str {
        &self.coverage_head_action_id
    }

    pub fn verifier_did(&self) -> &str {
        &self.verifier_did
    }

    pub fn verifier_key_id(&self) -> &str {
        &self.verifier_key_id
    }

    pub fn key_generation(&self) -> u64 {
        self.key_generation
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }

    pub fn signature_scheme_id(&self) -> &str {
        &self.signature_scheme_id
    }

    pub fn record_verified_at_micros(&self) -> i64 {
        self.record_verified_at_micros
    }

    pub fn record_valid_until_micros(&self) -> i64 {
        self.record_valid_until_micros
    }

    pub fn generation_valid_from_micros(&self) -> i64 {
        self.generation_valid_from_micros
    }

    pub fn generation_valid_until_micros(&self) -> i64 {
        self.generation_valid_until_micros
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActivationSubjectContextErrorV2 {
    RecordInvalid(AuthorityScopedKVectorVerificationRecordErrorV2),
    RecordAuthorityDomainMismatch,
    RuntimeAuthorityDomainMismatch,
    RuntimePrecryptoContextMismatch,
    PrecryptoRecordMismatch,
    ObservationRecordMismatch,
    ObservedGenerationDigestMismatch,
    ObservedVerifierDidMismatch,
    ObservedVerifierKeyIdMismatch,
}

impl From<AuthorityScopedKVectorVerificationRecordErrorV2> for ActivationSubjectContextErrorV2 {
    fn from(value: AuthorityScopedKVectorVerificationRecordErrorV2) -> Self {
        Self::RecordInvalid(value)
    }
}

#[derive(Clone, Copy)]
struct ActivationSubjectDigestPartsV2<'a> {
    authority_domain_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    record_signing_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    runtime_bound_context_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    exact_observation_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    generation_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    scoped_generation_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    scoped_policy_digest_sha256: &'a [u8; SHA256_DIGEST_LEN_V2],
    generation_action_id: &'a str,
    did_document_action_id: &'a str,
    coverage_head_action_seq: u32,
    coverage_head_action_id: &'a str,
    verifier_did: &'a str,
    verifier_key_id: &'a str,
    key_generation: u64,
    algorithm: AlgorithmId,
    signature_scheme_id: &'a str,
    record_verified_at_micros: i64,
    record_valid_until_micros: i64,
    generation_valid_from_micros: i64,
    generation_valid_until_micros: i64,
}

fn update_len_prefixed_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

fn derive_activation_subject_digest_v2(
    parts: ActivationSubjectDigestPartsV2<'_>,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(ACTIVATION_SUBJECT_CONTEXT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(parts.authority_domain_digest_sha256);
    hasher.update([0x02]);
    hasher.update(parts.record_signing_digest_sha256);
    hasher.update([0x03]);
    hasher.update(parts.runtime_bound_context_digest_sha256);
    hasher.update([0x04]);
    hasher.update(parts.exact_observation_digest_sha256);
    hasher.update([0x05]);
    hasher.update(parts.generation_sha256);
    hasher.update([0x06]);
    hasher.update(parts.scoped_generation_digest_sha256);
    hasher.update([0x07]);
    hasher.update(parts.scoped_policy_digest_sha256);
    update_len_prefixed_v2(&mut hasher, 0x08, parts.generation_action_id.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x09, parts.did_document_action_id.as_bytes());
    hasher.update([0x0a]);
    hasher.update(parts.coverage_head_action_seq.to_be_bytes());
    update_len_prefixed_v2(&mut hasher, 0x0b, parts.coverage_head_action_id.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x0c, parts.verifier_did.as_bytes());
    update_len_prefixed_v2(&mut hasher, 0x0d, parts.verifier_key_id.as_bytes());
    hasher.update([0x0e]);
    hasher.update(parts.key_generation.to_be_bytes());
    hasher.update([0x0f]);
    hasher.update(parts.algorithm.as_u16().to_be_bytes());
    update_len_prefixed_v2(&mut hasher, 0x10, parts.signature_scheme_id.as_bytes());
    hasher.update([0x11]);
    hasher.update(parts.record_verified_at_micros.to_be_bytes());
    hasher.update([0x12]);
    hasher.update(parts.record_valid_until_micros.to_be_bytes());
    hasher.update([0x13]);
    hasher.update(parts.generation_valid_from_micros.to_be_bytes());
    hasher.update([0x14]);
    hasher.update(parts.generation_valid_until_micros.to_be_bytes());
    hasher.finalize().into()
}

/// Freeze one exact activation subject only after every structural/provenance capability
/// is proven to refer to the same signed record and observed key generation.
pub fn qualify_historical_activation_subject_context_v2(
    record: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
    precrypto_context: &QualifiedAuthorityScopedPrecryptoContextV2,
    runtime_context: &RuntimeBoundAuthorityScopedPrecryptoContextV2,
    exact_observation: &QualifiedExactHistoricalGenerationObservationV2,
) -> Result<QualifiedHistoricalActivationSubjectContextV2, ActivationSubjectContextErrorV2> {
    let record_signing_digest_sha256 =
        derive_authority_scoped_kvector_verification_record_signing_digest_v2(record)?;

    if record.authority_domain_sha256 != precrypto_context.authority_domain_digest_sha256() {
        return Err(ActivationSubjectContextErrorV2::RecordAuthorityDomainMismatch);
    }
    if runtime_context.authority_domain_digest_sha256()
        != precrypto_context.authority_domain_digest_sha256()
    {
        return Err(ActivationSubjectContextErrorV2::RuntimeAuthorityDomainMismatch);
    }
    if runtime_context.precrypto_context_digest_sha256()
        != precrypto_context.precrypto_context_digest_sha256()
    {
        return Err(ActivationSubjectContextErrorV2::RuntimePrecryptoContextMismatch);
    }
    if record_signing_digest_sha256.as_slice()
        != precrypto_context.record_signing_digest_sha256()
    {
        return Err(ActivationSubjectContextErrorV2::PrecryptoRecordMismatch);
    }
    if record_signing_digest_sha256.as_slice() != exact_observation.record_signing_digest_sha256()
    {
        return Err(ActivationSubjectContextErrorV2::ObservationRecordMismatch);
    }

    let normalized = exact_observation.normalized_observation();
    let resolved = normalized.resolved_generation();
    if resolved.generation_sha256().as_slice()
        != record.base_record.verifier_key_generation_sha256
    {
        return Err(ActivationSubjectContextErrorV2::ObservedGenerationDigestMismatch);
    }
    if resolved.verifier_did() != record.base_record.verifier_did {
        return Err(ActivationSubjectContextErrorV2::ObservedVerifierDidMismatch);
    }
    if resolved.verifier_key_id() != record.base_record.verifier_key_id {
        return Err(ActivationSubjectContextErrorV2::ObservedVerifierKeyIdMismatch);
    }

    let coverage = normalized.coverage();
    let authority_domain_digest_sha256 = *precrypto_context.authority_domain_digest_sha256();
    let precrypto_context_digest_sha256 = *precrypto_context.precrypto_context_digest_sha256();
    let runtime_bound_context_digest_sha256 = *runtime_context.runtime_bound_context_digest_sha256();
    let exact_observation_digest_sha256 = *exact_observation.exact_observation_digest_sha256();
    let scoped_generation_digest_sha256 = *precrypto_context.scoped_generation_digest_sha256();
    let scoped_policy_digest_sha256 = *precrypto_context.scoped_policy_digest_sha256();
    let generation_sha256 = *resolved.generation_sha256();

    let generation_action_id = resolved.generation_action_id().to_string();
    let did_document_action_id = normalized.selected_did_document_action_id().to_string();
    let coverage_head_action_seq = coverage.valid_head_action_seq();
    let coverage_head_action_id = coverage.valid_head_action_id().to_string();
    let verifier_did = resolved.verifier_did().to_string();
    let verifier_key_id = resolved.verifier_key_id().to_string();
    let key_generation = resolved.key_generation();
    let algorithm = resolved.algorithm();
    let public_key_bytes = resolved.public_key_bytes().to_vec();
    let signature_scheme_id = record.base_record.signature_scheme_id.to_string();
    let record_verified_at_micros = record.base_record.verified_at_micros;
    let record_valid_until_micros = record.base_record.valid_until_micros;
    let generation_valid_from_micros = resolved.valid_from_micros();
    let generation_valid_until_micros = resolved.valid_until_micros();

    let activation_subject_digest_sha256 = derive_activation_subject_digest_v2(
        ActivationSubjectDigestPartsV2 {
            authority_domain_digest_sha256: &authority_domain_digest_sha256,
            record_signing_digest_sha256: &record_signing_digest_sha256,
            runtime_bound_context_digest_sha256: &runtime_bound_context_digest_sha256,
            exact_observation_digest_sha256: &exact_observation_digest_sha256,
            generation_sha256: &generation_sha256,
            scoped_generation_digest_sha256: &scoped_generation_digest_sha256,
            scoped_policy_digest_sha256: &scoped_policy_digest_sha256,
            generation_action_id: &generation_action_id,
            did_document_action_id: &did_document_action_id,
            coverage_head_action_seq,
            coverage_head_action_id: &coverage_head_action_id,
            verifier_did: &verifier_did,
            verifier_key_id: &verifier_key_id,
            key_generation,
            algorithm,
            signature_scheme_id: &signature_scheme_id,
            record_verified_at_micros,
            record_valid_until_micros,
            generation_valid_from_micros,
            generation_valid_until_micros,
        },
    );

    Ok(QualifiedHistoricalActivationSubjectContextV2 {
        authority_domain_digest_sha256,
        record_signing_digest_sha256,
        precrypto_context_digest_sha256,
        runtime_bound_context_digest_sha256,
        exact_observation_digest_sha256,
        scoped_generation_digest_sha256,
        scoped_policy_digest_sha256,
        generation_sha256,
        activation_subject_digest_sha256,
        generation_action_id,
        did_document_action_id,
        coverage_head_action_seq,
        coverage_head_action_id,
        verifier_did,
        verifier_key_id,
        key_generation,
        algorithm,
        public_key_bytes,
        signature_scheme_id,
        record_verified_at_micros,
        record_valid_until_micros,
        generation_valid_from_micros,
        generation_valid_until_micros,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest_parts<'a>(
        runtime: &'a [u8; SHA256_DIGEST_LEN_V2],
    ) -> ActivationSubjectDigestPartsV2<'a> {
        static AUTHORITY: [u8; SHA256_DIGEST_LEN_V2] = [0x11; SHA256_DIGEST_LEN_V2];
        static RECORD: [u8; SHA256_DIGEST_LEN_V2] = [0x22; SHA256_DIGEST_LEN_V2];
        static OBSERVATION: [u8; SHA256_DIGEST_LEN_V2] = [0x44; SHA256_DIGEST_LEN_V2];
        static GENERATION: [u8; SHA256_DIGEST_LEN_V2] = [0x55; SHA256_DIGEST_LEN_V2];
        static SCOPED_GENERATION: [u8; SHA256_DIGEST_LEN_V2] = [0x66; SHA256_DIGEST_LEN_V2];
        static SCOPED_POLICY: [u8; SHA256_DIGEST_LEN_V2] = [0x77; SHA256_DIGEST_LEN_V2];
        ActivationSubjectDigestPartsV2 {
            authority_domain_digest_sha256: &AUTHORITY,
            record_signing_digest_sha256: &RECORD,
            runtime_bound_context_digest_sha256: runtime,
            exact_observation_digest_sha256: &OBSERVATION,
            generation_sha256: &GENERATION,
            scoped_generation_digest_sha256: &SCOPED_GENERATION,
            scoped_policy_digest_sha256: &SCOPED_POLICY,
            generation_action_id: "uhCkkGENERATION",
            did_document_action_id: "uhCkkDIDDOC",
            coverage_head_action_seq: 42,
            coverage_head_action_id: "uhCkkHEAD",
            verifier_did: "did:mycelix:uhCAkVERIFIER",
            verifier_key_id: "did:mycelix:uhCAkVERIFIER#hybrid-1",
            key_generation: 3,
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
            record_verified_at_micros: 2_000_000,
            record_valid_until_micros: 4_000_000,
            generation_valid_from_micros: 1_000_000,
            generation_valid_until_micros: 9_000_000,
        }
    }

    #[test]
    fn frozen_activation_subject_digest_is_stable() {
        let runtime = [0x33; SHA256_DIGEST_LEN_V2];
        assert_eq!(
            derive_activation_subject_digest_v2(digest_parts(&runtime)),
            [
                0x98, 0x8c, 0x9e, 0xfa, 0x6e, 0xe2, 0xf4, 0x8d, 0xc2, 0xfc, 0x1a, 0xf1,
                0xe0, 0x0c, 0x39, 0x07, 0x07, 0x5b, 0xed, 0xaf, 0x5e, 0x8e, 0xa5, 0x37,
                0x67, 0x74, 0x21, 0xf5, 0xfd, 0x0f, 0xba, 0xdb,
            ]
        );
    }

    #[test]
    fn runtime_context_substitution_changes_activation_subject() {
        let first = [0x33; SHA256_DIGEST_LEN_V2];
        let mut second = first;
        second[7] ^= 0x80;
        assert_ne!(
            derive_activation_subject_digest_v2(digest_parts(&first)),
            derive_activation_subject_digest_v2(digest_parts(&second))
        );
    }

    #[test]
    fn qualified_result_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedHistoricalActivationSubjectContextV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedHistoricalActivationSubjectContextV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_digest_sha256:",
            "pub record_signing_digest_sha256:",
            "pub runtime_bound_context_digest_sha256:",
            "pub exact_observation_digest_sha256:",
            "pub generation_sha256:",
            "pub activation_subject_digest_sha256:",
            "pub generation_action_id:",
            "pub did_document_action_id:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
