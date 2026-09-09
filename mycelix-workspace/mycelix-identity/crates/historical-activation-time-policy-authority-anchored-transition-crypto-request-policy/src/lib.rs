// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Prepared cryptographic request for one authority-anchored historical time-policy transition.
//!
//! #430 freezes the exact administrative transition + signer subject + generic causal
//! authority-state anchor. This theorem binds that anchored signing digest to the exact #403
//! public-key generation and one exact signature. It performs no cryptographic verification.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedSignature};
use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2;
use mycelix_historical_activation_time_policy_authority_key_generation_policy::QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2;
use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::qualify_historical_activation_time_policy_transition_signer_authority_subject_v2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_CRYPTO_REQUEST_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-anchored-transition-crypto-request:v2\0";

#[derive(Debug)]
pub struct PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    anchored_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    signer_authority_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    signature_bytes: Vec<u8>,
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.authority_domain_sha256 }
    pub fn anchored_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.anchored_transition_signing_digest_sha256 }
    pub fn signer_authority_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.signer_authority_subject_sha256 }
    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.policy_authority_key_generation_sha256 }
    pub fn algorithm(&self) -> AlgorithmId { self.algorithm }
    pub fn public_key_bytes(&self) -> &[u8] { &self.public_key_bytes }
    pub fn signature_bytes(&self) -> &[u8] { &self.signature_bytes }
    pub fn crypto_request_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.crypto_request_digest_sha256 }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2 {
    AuthorityDomainMismatch,
    KeyGenerationMismatch,
    AlgorithmMismatch,
    SignerSubjectMismatch,
    SignatureAlgorithmMismatch,
    SignatureLengthInvalid,
    SignatureAllZero,
}

fn update_len_prefixed_u32_v2(hasher: &mut Sha256, tag: u8, value: &[u8]) {
    hasher.update([tag]);
    hasher.update((value.len() as u32).to_be_bytes());
    hasher.update(value);
}

pub fn prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(
    anchored: &PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2,
    key_generation: &QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    signature: &TaggedSignature,
) -> Result<PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2, AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2> {
    if anchored.authority_domain_sha256() != key_generation.authority_domain_sha256() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::AuthorityDomainMismatch);
    }
    if anchored.policy_authority_key_generation_sha256() != key_generation.generation_digest_sha256() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::KeyGenerationMismatch);
    }
    if anchored.algorithm() != key_generation.algorithm() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::AlgorithmMismatch);
    }
    let signer_subject =
        qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(key_generation);
    if anchored.signer_authority_subject_sha256() != signer_subject.subject_digest_sha256() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::SignerSubjectMismatch);
    }
    if signature.algorithm != key_generation.algorithm() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::SignatureAlgorithmMismatch);
    }
    if signature.signature_bytes.len() != key_generation.algorithm().signature_size() {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::SignatureLengthInvalid);
    }
    if signature.signature_bytes.iter().all(|byte| *byte == 0) {
        return Err(AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::SignatureAllZero);
    }

    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_CRYPTO_REQUEST_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(key_generation.authority_domain_sha256());
    hasher.update([0x02]);
    hasher.update(anchored.anchored_transition_signing_digest_sha256());
    hasher.update([0x03]);
    hasher.update(signer_subject.subject_digest_sha256());
    hasher.update([0x04]);
    hasher.update(key_generation.generation_digest_sha256());
    hasher.update([0x05]);
    hasher.update(key_generation.algorithm().as_u16().to_be_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x06, key_generation.public_key_bytes());
    update_len_prefixed_u32_v2(&mut hasher, 0x07, &signature.signature_bytes);
    let crypto_request_digest_sha256 = hasher.finalize().into();

    Ok(PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
        authority_domain_sha256: *key_generation.authority_domain_sha256(),
        anchored_transition_signing_digest_sha256: *anchored.anchored_transition_signing_digest_sha256(),
        signer_authority_subject_sha256: *signer_subject.subject_digest_sha256(),
        policy_authority_key_generation_sha256: *key_generation.generation_digest_sha256(),
        algorithm: key_generation.algorithm(),
        public_key_bytes: key_generation.public_key_bytes().to_vec(),
        signature_bytes: signature.signature_bytes.clone(),
        crypto_request_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_historical_activation_time_authority_policy::{qualify_static_historical_activation_time_authority_policy_v2, HistoricalActivationTimeAuthorityPolicyBodyV2};
    use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::{prepare_authority_anchored_historical_activation_time_policy_transition_v2, HistoricalActivationTimePolicyAuthorityStateAnchorV2};
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{qualify_historical_activation_time_policy_authority_key_generation_v2, HistoricalActivationTimePolicyAuthorityKeyGenerationV2};
    use mycelix_historical_activation_time_policy_transition_policy::{prepare_time_policy_adoption_transition_v2, HistoricalActivationTimePolicyAuthoritySignerV2};
    use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;
    use mycelix_identity_authority_domain_policy::{qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2};

    static DNA: [u8; 39] = [0x84,0x2d,0x24,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,1,2,3,4];
    static TIME_KEY_GENERATION: [u8; 32] = [0x33; 32];
    static AUTHORITY_STATE_TRANSITION: [u8; 32] = [0x55; 32];

    fn fixture() -> (PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2, QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2, SigningKey) {
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 { authority_domain_id: "mycelix-identity-v2", authority_domain_epoch: 1, dna_hash_raw_39: &DNA }).unwrap();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(&domain, HistoricalActivationTimePolicyAuthorityKeyGenerationV2 { policy_authority_id: "identity:policy-authority:bootstrap-v2", policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1", algorithm: AlgorithmId::Ed25519, public_key_bytes: &public_key, key_generation: 1 }).unwrap();
        let policy = qualify_static_historical_activation_time_authority_policy_v2(HistoricalActivationTimeAuthorityPolicyBodyV2 { policy_id: "time-policy:primary-v2", policy_version: 1, time_authority_id: "time:authority:primary-v2", time_authority_key_id: "time:authority:primary-v2#hybrid-1", time_authority_key_generation_sha256: &TIME_KEY_GENERATION, algorithm: AlgorithmId::HybridEd25519MlDsa65, time_basis: TimeBasisV2::UnixMicrosecondsUtc, utc_realization_id: "unix-utc-normalized-v1", max_uncertainty_before_micros: 10_000, max_uncertainty_after_micros: 10_000, max_receipt_lifetime_micros: 1_000_000, valid_from_micros: 1_000_000, valid_until_micros: 10_000_000 }).unwrap();
        let transition = prepare_time_policy_adoption_transition_v2(&domain, &policy, 1, None, HistoricalActivationTimePolicyAuthoritySignerV2 { policy_authority_id: generation.policy_authority_id(), policy_authority_key_id: generation.policy_authority_key_id(), policy_authority_key_generation_sha256: generation.generation_digest_sha256(), algorithm: generation.algorithm() }).unwrap();
        let subject = qualify_historical_activation_time_policy_transition_signer_authority_subject_v2(&generation);
        let anchored = prepare_authority_anchored_historical_activation_time_policy_transition_v2(&transition, &subject, HistoricalActivationTimePolicyAuthorityStateAnchorV2 { authority_state_generation: 1, authority_state_transition_digest: &AUTHORITY_STATE_TRANSITION }).unwrap();
        (anchored, generation, signing_key)
    }

    #[test]
    fn real_ed25519_anchored_request_has_frozen_digest() {
        let (anchored, generation, signing_key) = fixture();
        assert_eq!(anchored.anchored_transition_signing_digest_sha256(), &[0xa7,0x5e,0x9b,0xae,0x12,0x35,0x49,0x6b,0xfd,0xde,0x1a,0x3a,0xc5,0x2e,0x9d,0x03,0x25,0x8f,0x55,0x2f,0xfa,0xc2,0x47,0x53,0x98,0x1f,0x7a,0x8c,0xbb,0x0c,0x76,0xcd]);
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, signing_key.sign(anchored.anchored_transition_signing_digest_sha256()).to_bytes().to_vec()).unwrap();
        let request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(&anchored, &generation, &signature).unwrap();
        assert_eq!(request.crypto_request_digest_sha256(), &[0x40,0x92,0xc5,0xf7,0x41,0x97,0xac,0xdf,0x1d,0xea,0x0d,0x22,0xea,0xb2,0x72,0x4e,0xe9,0x66,0xfd,0x96,0x55,0x44,0x34,0x86,0xe8,0x6d,0x5a,0x43,0x43,0xd6,0x2b,0x0c]);
    }

    #[test]
    fn old_base_transition_signature_is_not_reusable() {
        let (anchored, generation, signing_key) = fixture();
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, signing_key.sign(anchored.base_transition_signing_digest_sha256()).to_bytes().to_vec()).unwrap();
        let request = prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(&anchored, &generation, &signature).unwrap();
        assert_ne!(request.crypto_request_digest_sha256(), &[0x40,0x92,0xc5,0xf7,0x41,0x97,0xac,0xdf,0x1d,0xea,0x0d,0x22,0xea,0xb2,0x72,0x4e,0xe9,0x66,0xfd,0x96,0x55,0x44,0x34,0x86,0xe8,0x6d,0x5a,0x43,0x43,0xd6,0x2b,0x0c]);
    }

    #[test]
    fn signature_algorithm_substitution_fails_closed() {
        let (anchored, generation, _) = fixture();
        let signature = TaggedSignature::new(AlgorithmId::MlDsa65, vec![0x77; AlgorithmId::MlDsa65.signature_size()]).unwrap();
        assert_eq!(prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(&anchored, &generation, &signature).unwrap_err(), AuthorityAnchoredTimePolicyTransitionCryptoRequestErrorV2::SignatureAlgorithmMismatch);
    }
}
