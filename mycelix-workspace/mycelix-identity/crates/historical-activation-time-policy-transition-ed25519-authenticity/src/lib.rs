// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Ed25519-only cryptographic authenticity for historical time-policy transitions.
//!
//! This adapter consumes only one opaque #404 prepared transition crypto request,
//! verifies the exact #401 transition signing digest with a verifier-owned strict
//! Ed25519 backend, and emits an opaque cryptographic-authenticity capability.
//!
//! All non-Ed25519 algorithms fail closed before component verification. Success here
//! does not establish policy-authority key provenance/currentness, complete authenticated
//! transition lineage, accepted/current time policy, trusted time, or #337 activation.

#![forbid(unsafe_code)]

use ed25519_dalek::{Signature, VerifyingKey};
use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_transition_crypto_request_policy::PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_ED25519_AUTHENTICITY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:historical-activation-time-policy-transition-ed25519-authenticity:v2\0";

#[derive(Debug)]
pub struct CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2 {
    authority_domain_sha256: [u8; SHA256_DIGEST_LEN_V2],
    transition_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    policy_authority_key_generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
    crypto_request_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    algorithm: AlgorithmId,
    authenticity_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2 {
    pub fn authority_domain_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_sha256
    }

    pub fn transition_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.transition_signing_digest_sha256
    }

    pub fn policy_authority_key_generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.policy_authority_key_generation_sha256
    }

    pub fn crypto_request_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.crypto_request_digest_sha256
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn authenticity_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authenticity_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyTransitionAuthenticityErrorV2 {
    AlgorithmPendingQualification,
    PublicKeyLengthInvalid,
    SignatureLengthInvalid,
    PublicKeyInvalid,
    SignatureInvalid,
}

fn derive_transition_authenticity_digest_v2(
    request: &PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2,
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(HISTORICAL_ACTIVATION_TIME_POLICY_TRANSITION_ED25519_AUTHENTICITY_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(request.crypto_request_digest_sha256());
    hasher.finalize().into()
}

/// Strictly authenticate one Ed25519 historical time-policy transition.
///
/// The backend is owned here. No caller-supplied verifier object is accepted.
/// Hybrid/PQ algorithms fail before any component signature can be interpreted.
pub fn authenticate_historical_activation_time_policy_transition_ed25519_v2(
    request: &PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2,
) -> Result<
    CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2,
    HistoricalActivationTimePolicyTransitionAuthenticityErrorV2,
> {
    if request.algorithm() != AlgorithmId::Ed25519 {
        return Err(
            HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::AlgorithmPendingQualification,
        );
    }

    let public_key_bytes: [u8; 32] = request
        .public_key_bytes()
        .try_into()
        .map_err(|_| {
            HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::PublicKeyLengthInvalid
        })?;
    let signature_bytes: [u8; 64] = request
        .signature_bytes()
        .try_into()
        .map_err(|_| {
            HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::SignatureLengthInvalid
        })?;

    let verifying_key = VerifyingKey::from_bytes(&public_key_bytes).map_err(|_| {
        HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::PublicKeyInvalid
    })?;
    let signature = Signature::from_bytes(&signature_bytes);

    verifying_key
        .verify_strict(request.transition_signing_digest_sha256(), &signature)
        .map_err(|_| HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::SignatureInvalid)?;

    let authenticity_digest_sha256 = derive_transition_authenticity_digest_v2(request);
    Ok(CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2 {
        authority_domain_sha256: *request.authority_domain_sha256(),
        transition_signing_digest_sha256: *request.transition_signing_digest_sha256(),
        policy_authority_key_generation_sha256: *request.policy_authority_key_generation_sha256(),
        crypto_request_digest_sha256: *request.crypto_request_digest_sha256(),
        algorithm: request.algorithm(),
        authenticity_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_crypto::TaggedSignature;
    use mycelix_historical_activation_time_authority_policy::{
        qualify_static_historical_activation_time_authority_policy_v2,
        HistoricalActivationTimeAuthorityPolicyBodyV2,
    };
    use mycelix_historical_activation_time_policy_authority_key_generation_policy::{
        qualify_historical_activation_time_policy_authority_key_generation_v2,
        HistoricalActivationTimePolicyAuthorityKeyGenerationV2,
    };
    use mycelix_historical_activation_time_policy_transition_crypto_request_policy::{
        prepare_historical_activation_time_policy_transition_crypto_request_v2,
        PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2,
    };
    use mycelix_historical_activation_time_policy_transition_policy::{
        prepare_time_policy_adoption_transition_v2,
        HistoricalActivationTimePolicyAuthoritySignerV2,
    };
    use mycelix_historical_activation_time_receipt_policy::TimeBasisV2;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };

    static DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    static TIME_KEY_GENERATION: [u8; 32] = [0x33; 32];

    fn valid_request(
        tamper_signature: bool,
    ) -> PreparedHistoricalActivationTimePolicyTransitionCryptoRequestV2 {
        let signing_key = SigningKey::from_bytes(&[0x77; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        assert_eq!(
            public_key,
            [
                0xc8, 0x53, 0xad, 0x0f, 0x0c, 0xd2, 0xb6, 0x19, 0xae, 0xa9, 0x2c, 0xee,
                0xc4, 0xfd, 0x56, 0xa2, 0x4d, 0x64, 0x99, 0xd5, 0x84, 0xce, 0x79, 0x25,
                0x7e, 0x45, 0xcf, 0xd8, 0x13, 0x9b, 0x60, 0xa7,
            ]
        );

        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &public_key,
                key_generation: 1,
            },
        )
        .unwrap();
        assert_eq!(
            generation.generation_digest_sha256(),
            &[
                0x40, 0xb4, 0xf4, 0xb9, 0x80, 0xee, 0xd2, 0xa4, 0xb2, 0xe8, 0xc2, 0x52,
                0x06, 0xee, 0xd4, 0xdd, 0xfe, 0x19, 0x62, 0x3e, 0x4f, 0x20, 0x70, 0xd8,
                0x6d, 0x76, 0x9f, 0x1a, 0x63, 0xac, 0x79, 0xc9,
            ]
        );

        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &TIME_KEY_GENERATION,
                algorithm: AlgorithmId::HybridEd25519MlDsa65,
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                utc_realization_id: "unix-utc-normalized-v1",
                max_uncertainty_before_micros: 10_000,
                max_uncertainty_after_micros: 10_000,
                max_receipt_lifetime_micros: 1_000_000,
                valid_from_micros: 1_000_000,
                valid_until_micros: 10_000_000,
            },
        )
        .unwrap();
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain,
            &policy,
            1,
            None,
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation.policy_authority_id(),
                policy_authority_key_id: generation.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
                algorithm: generation.algorithm(),
            },
        )
        .unwrap();
        assert_eq!(
            transition.transition_signing_digest_sha256(),
            &[
                0x51, 0x40, 0x02, 0x25, 0x80, 0x9b, 0x98, 0xc2, 0x65, 0x16, 0x2e, 0x46,
                0x92, 0x81, 0x79, 0xf3, 0x8e, 0x21, 0x6a, 0xcd, 0x70, 0x74, 0x5c, 0xd9,
                0x80, 0x46, 0xc8, 0x54, 0xf7, 0x05, 0x0d, 0x7e,
            ]
        );

        let mut signature_bytes = signing_key
            .sign(transition.transition_signing_digest_sha256())
            .to_bytes()
            .to_vec();
        if tamper_signature {
            signature_bytes[0] ^= 0x01;
        }
        let signature = TaggedSignature::new(AlgorithmId::Ed25519, signature_bytes).unwrap();
        prepare_historical_activation_time_policy_transition_crypto_request_v2(
            &transition,
            &generation,
            &signature,
        )
        .unwrap()
    }

    #[test]
    fn valid_ed25519_transition_is_strictly_authenticated() {
        let request = valid_request(false);
        assert_eq!(
            request.crypto_request_digest_sha256(),
            &[
                0x22, 0xd4, 0xaa, 0x8e, 0xb7, 0x37, 0xa2, 0x2f, 0x39, 0x42, 0x15, 0x23,
                0xc3, 0x50, 0xd4, 0xd2, 0x34, 0xca, 0x79, 0xf7, 0xe8, 0x3c, 0xe4, 0x40,
                0x72, 0xaf, 0x0b, 0x46, 0xdb, 0x78, 0xfd, 0x82,
            ]
        );
        let authenticated =
            authenticate_historical_activation_time_policy_transition_ed25519_v2(&request)
                .unwrap();
        assert_eq!(authenticated.algorithm(), AlgorithmId::Ed25519);
        assert_eq!(
            authenticated.authenticity_digest_sha256(),
            &[
                0xbb, 0xdc, 0x2d, 0x4d, 0x08, 0x60, 0x85, 0xcc, 0xea, 0x24, 0xfd, 0x8a,
                0x9e, 0xed, 0x37, 0x85, 0x14, 0x32, 0x65, 0x5a, 0x69, 0x69, 0xe4, 0x0a,
                0x5a, 0x74, 0xd8, 0xdb, 0x23, 0x62, 0x57, 0xc2,
            ]
        );
    }

    #[test]
    fn tampered_transition_signature_fails_strict_verification() {
        let request = valid_request(true);
        assert_eq!(
            authenticate_historical_activation_time_policy_transition_ed25519_v2(&request)
                .unwrap_err(),
            HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::SignatureInvalid
        );
    }

    #[test]
    fn hybrid_transition_never_falls_back_to_ed25519_component_verification() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let public_key = vec![0x42; AlgorithmId::HybridEd25519MlDsa65.public_key_size()];
        let generation = qualify_historical_activation_time_policy_authority_key_generation_v2(
            &domain,
            HistoricalActivationTimePolicyAuthorityKeyGenerationV2 {
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#hybrid-1",
                algorithm: AlgorithmId::HybridEd25519MlDsa65,
                public_key_bytes: &public_key,
                key_generation: 1,
            },
        )
        .unwrap();
        let policy = qualify_static_historical_activation_time_authority_policy_v2(
            HistoricalActivationTimeAuthorityPolicyBodyV2 {
                policy_id: "time-policy:primary-v2",
                policy_version: 1,
                time_authority_id: "time:authority:primary-v2",
                time_authority_key_id: "time:authority:primary-v2#hybrid-1",
                time_authority_key_generation_sha256: &TIME_KEY_GENERATION,
                algorithm: AlgorithmId::HybridEd25519MlDsa65,
                time_basis: TimeBasisV2::UnixMicrosecondsUtc,
                utc_realization_id: "unix-utc-normalized-v1",
                max_uncertainty_before_micros: 10_000,
                max_uncertainty_after_micros: 10_000,
                max_receipt_lifetime_micros: 1_000_000,
                valid_from_micros: 1_000_000,
                valid_until_micros: 10_000_000,
            },
        )
        .unwrap();
        let transition = prepare_time_policy_adoption_transition_v2(
            &domain,
            &policy,
            1,
            None,
            HistoricalActivationTimePolicyAuthoritySignerV2 {
                policy_authority_id: generation.policy_authority_id(),
                policy_authority_key_id: generation.policy_authority_key_id(),
                policy_authority_key_generation_sha256: generation.generation_digest_sha256(),
                algorithm: generation.algorithm(),
            },
        )
        .unwrap();
        let signature = TaggedSignature::new(
            AlgorithmId::HybridEd25519MlDsa65,
            vec![0x55; AlgorithmId::HybridEd25519MlDsa65.signature_size()],
        )
        .unwrap();
        let request = prepare_historical_activation_time_policy_transition_crypto_request_v2(
            &transition,
            &generation,
            &signature,
        )
        .unwrap();
        assert_eq!(
            authenticate_historical_activation_time_policy_transition_ed25519_v2(&request)
                .unwrap_err(),
            HistoricalActivationTimePolicyTransitionAuthenticityErrorV2::AlgorithmPendingQualification
        );
    }

    #[test]
    fn authenticated_transition_fields_are_private() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2")
            .unwrap();
        let end = source[start..]
            .index("impl CryptographicallyAuthenticatedHistoricalActivationTimePolicyTransitionV2")
            .unwrap()
            + start;
        let result = &source[start..end];
        for field in [
            "pub transition_signing_digest_sha256:",
            "pub crypto_request_digest_sha256:",
            "pub authenticity_digest_sha256:",
        ] {
            assert!(!result.contains(field));
        }
    }
}
