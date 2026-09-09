// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Strict Ed25519 authenticity for causal authority-anchored historical time-policy transitions.
//!
//! The backend is owned by this crate. It consumes only one opaque #432 request and verifies
//! exactly the #430 anchored signing digest. Hybrid/PQ algorithms remain fail-closed pending
//! exact standards/wire qualification (#252).

#![forbid(unsafe_code)]

use ed25519_dalek::{Signature, VerifyingKey};
use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_authority_anchored_transition_crypto_request_policy::PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_ED25519_AUTHENTICITY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-authority-anchored-transition-ed25519-authenticity:v2\0";

#[derive(Debug)]
pub struct CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    anchored_transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    signer_authority_subject_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    authenticity_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.authority_domain_sha256 }
    pub fn anchored_transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.anchored_transition_signing_digest_sha256 }
    pub fn signer_authority_subject_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.signer_authority_subject_sha256 }
    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.policy_authority_key_generation_sha256 }
    pub fn crypto_request_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.crypto_request_digest_sha256 }
    pub fn algorithm(&self) -> AlgorithmId { self.algorithm }
    pub fn authenticity_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] { &self.authenticity_digest_sha256 }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2 {
    AlgorithmPendingQualification,
    PublicKeyLengthInvalid,
    SignatureLengthInvalid,
    PublicKeyInvalid,
    SignatureInvalid,
}

fn derive_authenticity_digest_v2(
    request: &PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_AUTHORITY_ANCHORED_TRANSITION_ED25519_AUTHENTICITY_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(request.crypto_request_digest_sha256());
    hasher.finalize().into()
}

pub fn authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(
    request: &PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2,
) -> Result<CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2, AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2> {
    if request.algorithm() != AlgorithmId::Ed25519 {
        return Err(AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::AlgorithmPendingQualification);
    }
    let public_key_bytes: [u8; 32] = request.public_key_bytes().try_into().map_err(|_| AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::PublicKeyLengthInvalid)?;
    let signature_bytes: [u8; 64] = request.signature_bytes().try_into().map_err(|_| AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::SignatureLengthInvalid)?;
    let verifying_key = VerifyingKey::from_bytes(&public_key_bytes).map_err(|_| AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::PublicKeyInvalid)?;
    let signature = Signature::from_bytes(&signature_bytes);
    verifying_key
        .verify_strict(request.anchored_transition_signing_digest_sha256(), &signature)
        .map_err(|_| AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::SignatureInvalid)?;

    Ok(CryptographicallyAuthenticatedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2 {
        authority_domain_sha256: *request.authority_domain_sha256(),
        anchored_transition_signing_digest_sha256: *request.anchored_transition_signing_digest_sha256(),
        signer_authority_subject_sha256: *request.signer_authority_subject_sha256(),
        policy_authority_key_generation_sha256: *request.policy_authority_key_generation_sha256(),
        crypto_request_digest_sha256: *request.crypto_request_digest_sha256(),
        algorithm: request.algorithm(),
        authenticity_digest_sha256: derive_authenticity_digest_v2(request),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_crypto::TaggedSignature;
    use mycelix_historical_activation_time_authority_policy::{qualify_static_historical_activation_time_authority_policy_v2, HistoricalActivationTimeAuthorityPolicyBodyV2};
    use mycelix_historical_activation_time_policy_authority_anchored_transition_crypto_request_policy::prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2;
    use mycelix_historical_activation_time_policy_authority_anchored_transition_policy::{prepare_authority_anchored_historical_activation_time_policy_transition_v2, HistoricalActivationTimePolicyAuthorityStateAnchorV2, PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionV2};
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{qualify_historical_activation_time_policy_authority_key_generation_v2, HistoricalActivationTimePolicyAuthorityKeyGenerationV2, QualifiedHistoricalActivationTimePolicyAuthorityKeyGenerationV2};
    use mycelix_historical_activation_time_policy_transition_policy::{prepare_time_policy_adoption_transition_v2, HistoricalActivationTimePolicyAuthoritySignerV2};
    use mycelix_historical_activation_time_policy_transition_signer_authority_subject_policy::qualify_historical_activation_time_policy_transition_signer_authority_subject_v2;
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

    fn request(sign_old_base: bool) -> PreparedAuthorityAnchoredHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
        let (anchored, generation, signing_key) = fixture();
        let message = if sign_old_base { anchored.base_transition_signing_digest_sha256() } else { anchored.anchored_transition_signing_digest_sha256() };
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, signing_key.sign(message).to_bytes().to_vec()).unwrap();
        prepare_authority_anchored_historical_activation_time_policy_transition_crypto_request_v2(&anchored, &generation, &signature).unwrap()
    }

    #[test]
    fn valid_anchored_ed25519_signature_is_strictly_authenticated() {
        let request = request(false);
        assert_eq!(request.crypto_request_digest_sha256(), &[0x40,0x92,0xc5,0xf7,0x41,0x97,0xac,0xdf,0x1d,0xea,0x0d,0x22,0xea,0xb2,0x72,0x4e,0xe9,0x66,0xfd,0x96,0x55,0x44,0x34,0x86,0xe8,0x6d,0x5a,0x43,0x43,0xd6,0x2b,0x0c]);
        let authenticated = authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&request).unwrap();
        assert_eq!(authenticated.authenticity_digest_sha256(), &[0xca,0xde,0x2c,0x67,0x6c,0x07,0x9c,0x27,0x37,0x76,0x5f,0xf6,0x03,0x37,0x8d,0x2a,0x2b,0x90,0xe9,0x57,0x14,0x14,0xc8,0xa8,0x04,0xd5,0xd8,0xf4,0xbd,0x4b,0x2e,0xb1]);
    }

    #[test]
    fn old_base_transition_signature_fails_anchored_authenticity() {
        assert_eq!(authenticate_authority_anchored_historical_activation_time_policy_transition_ed25519_v2(&request(true)).unwrap_err(), AuthorityAnchoredTimePolicyTransitionAuthenticityErrorV2::SignatureInvalid);
    }
}
