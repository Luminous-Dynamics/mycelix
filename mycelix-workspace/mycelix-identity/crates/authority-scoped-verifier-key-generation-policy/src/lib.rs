// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Authority-scoped verifier-key generation identity for Identity V2.
//!
//! This theorem composes two already-qualified identities:
//! - #369: one exact Identity authority domain;
//! - #316: one exact intrinsic verifier-key generation.
//!
//! Success means only "this exact generation in this exact authority domain".
//! Runtime DNA provenance, domain acceptance/currentness, DID observation, trusted
//! activation, signature authenticity and policy authority remain separate proofs.

#![forbid(unsafe_code)]

use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use mycelix_kvector_verifier_key_generation_policy::{
    derive_kvector_verifier_key_generation_digest_v2, KVectorVerifierKeyGenerationErrorV2,
    KVectorVerifierKeyGenerationV2,
};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_SCOPED_VERIFIER_KEY_GENERATION_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verifier-key-generation:authority-scoped:v2\0";

/// Opaque identity capability for one intrinsic key generation under one exact
/// qualified authority-domain identity.
#[derive(Debug)]
pub struct QualifiedAuthorityScopedVerifierKeyGenerationV2 {
    authority_domain_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    intrinsic_generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedAuthorityScopedVerifierKeyGenerationV2 {
    pub fn authority_domain_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }

    pub fn intrinsic_generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.intrinsic_generation_digest_sha256
    }

    pub fn scoped_generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_generation_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityScopedVerifierKeyGenerationErrorV2 {
    IntrinsicGenerationInvalid(KVectorVerifierKeyGenerationErrorV2),
}

impl From<KVectorVerifierKeyGenerationErrorV2>
    for AuthorityScopedVerifierKeyGenerationErrorV2
{
    fn from(value: KVectorVerifierKeyGenerationErrorV2) -> Self {
        Self::IntrinsicGenerationInvalid(value)
    }
}

fn derive_scoped_digest_from_qualified_parts_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    intrinsic_generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(AUTHORITY_SCOPED_VERIFIER_KEY_GENERATION_DOMAIN_V2);
    hasher.update(authority_domain.digest_sha256());
    hasher.update(intrinsic_generation_digest_sha256);
    hasher.finalize().into()
}

/// Derive the scoped generation digest from an opaque authority-domain capability and
/// one structurally valid intrinsic generation. There is no API accepting a caller-
/// supplied raw domain digest in place of the #369 capability.
pub fn derive_authority_scoped_verifier_key_generation_digest_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], AuthorityScopedVerifierKeyGenerationErrorV2> {
    let intrinsic_generation_digest_sha256 =
        derive_kvector_verifier_key_generation_digest_v2(generation)?;
    Ok(derive_scoped_digest_from_qualified_parts_v2(
        authority_domain,
        intrinsic_generation_digest_sha256,
    ))
}

/// Qualify one exact verifier-key generation inside one exact Identity authority domain.
pub fn qualify_authority_scoped_verifier_key_generation_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<QualifiedAuthorityScopedVerifierKeyGenerationV2, AuthorityScopedVerifierKeyGenerationErrorV2>
{
    let intrinsic_generation_digest_sha256 =
        derive_kvector_verifier_key_generation_digest_v2(generation)?;
    let scoped_generation_digest_sha256 = derive_scoped_digest_from_qualified_parts_v2(
        authority_domain,
        intrinsic_generation_digest_sha256,
    );

    Ok(QualifiedAuthorityScopedVerifierKeyGenerationV2 {
        authority_domain_digest_sha256: *authority_domain.digest_sha256(),
        intrinsic_generation_digest_sha256,
        scoped_generation_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_crypto::AlgorithmId;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };

    const DNA_A: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];

    fn domain(dna: &[u8], epoch: u32) -> QualifiedIdentityAuthorityDomainV2 {
        qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: epoch,
            dna_hash_raw_39: dna,
        })
        .unwrap()
    }

    fn generation(key: &[u8], key_generation: u64) -> KVectorVerifierKeyGenerationV2<'_> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#key-1",
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: key,
            key_generation,
            issued_at_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
        }
    }

    #[test]
    fn frozen_scoped_generation_digest_is_stable() {
        let domain = domain(&DNA_A, 1);
        let key = [0x42; 32];
        assert_eq!(
            derive_authority_scoped_verifier_key_generation_digest_v2(&domain, generation(&key, 1))
                .unwrap(),
            [
                0x99, 0x9c, 0xfc, 0x8c, 0xbf, 0xc1, 0x86, 0x06, 0xcc, 0x58, 0x37, 0x37,
                0x63, 0xcf, 0x3d, 0x9a, 0xdd, 0x13, 0x69, 0xe3, 0x2c, 0x76, 0xd6, 0xe5,
                0xef, 0x5a, 0xcf, 0xe9, 0xf3, 0x8a, 0x9e, 0x7d,
            ]
        );
    }

    #[test]
    fn same_intrinsic_generation_in_different_dna_is_different_scoped_identity() {
        let first_domain = domain(&DNA_A, 1);
        let mut dna_b = DNA_A;
        dna_b[20] ^= 0x55;
        let second_domain = domain(&dna_b, 1);
        let key = [0x42; 32];
        let generation = generation(&key, 1);

        let first = qualify_authority_scoped_verifier_key_generation_v2(&first_domain, generation)
            .unwrap();
        let second = qualify_authority_scoped_verifier_key_generation_v2(&second_domain, generation)
            .unwrap();

        assert_eq!(
            first.intrinsic_generation_digest_sha256(),
            second.intrinsic_generation_digest_sha256()
        );
        assert_ne!(
            first.authority_domain_digest_sha256(),
            second.authority_domain_digest_sha256()
        );
        assert_ne!(
            first.scoped_generation_digest_sha256(),
            second.scoped_generation_digest_sha256()
        );
    }

    #[test]
    fn same_dna_under_different_epoch_is_different_scoped_identity() {
        let first_domain = domain(&DNA_A, 1);
        let second_domain = domain(&DNA_A, 2);
        let key = [0x42; 32];
        let generation = generation(&key, 1);
        assert_ne!(
            derive_authority_scoped_verifier_key_generation_digest_v2(&first_domain, generation)
                .unwrap(),
            derive_authority_scoped_verifier_key_generation_digest_v2(&second_domain, generation)
                .unwrap()
        );
    }

    #[test]
    fn changed_intrinsic_generation_changes_scoped_identity() {
        let domain = domain(&DNA_A, 1);
        let first_key = [0x42; 32];
        let second_key = [0x43; 32];
        let first = derive_authority_scoped_verifier_key_generation_digest_v2(
            &domain,
            generation(&first_key, 1),
        )
        .unwrap();
        let changed_generation = derive_authority_scoped_verifier_key_generation_digest_v2(
            &domain,
            generation(&first_key, 2),
        )
        .unwrap();
        let changed_key = derive_authority_scoped_verifier_key_generation_digest_v2(
            &domain,
            generation(&second_key, 1),
        )
        .unwrap();
        assert_ne!(first, changed_generation);
        assert_ne!(first, changed_key);
    }

    #[test]
    fn intrinsic_generation_validation_is_not_bypassed() {
        let domain = domain(&DNA_A, 1);
        let key = [0x42; 32];
        let invalid = KVectorVerifierKeyGenerationV2 {
            key_generation: 0,
            ..generation(&key, 1)
        };
        assert!(matches!(
            qualify_authority_scoped_verifier_key_generation_v2(&domain, invalid),
            Err(AuthorityScopedVerifierKeyGenerationErrorV2::IntrinsicGenerationInvalid(
                KVectorVerifierKeyGenerationErrorV2::KeyGenerationInvalid
            ))
        ));
    }

    #[test]
    fn qualified_scoped_generation_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedAuthorityScopedVerifierKeyGenerationV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedAuthorityScopedVerifierKeyGenerationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_digest_sha256:",
            "pub intrinsic_generation_digest_sha256:",
            "pub scoped_generation_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
