// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure authority-domain equality theorem for the Identity V2 pre-crypto verifier path.
//!
//! This layer composes the authority-scoped signed record (#374), scoped verifier-key
//! generation (#371), and scoped static verifier policy (#377). Success means these
//! structural components all describe one exact authority domain and mutually bind.
//!
//! It also derives one deterministic pre-crypto context digest. A later crypto-request
//! layer can therefore bind one opaque structural subject rather than independently
//! reassembling record, generation, policy, and authority-domain digests.
//!
//! It does not prove runtime DNA provenance, observed DID/key history, trusted
//! activation, signature authenticity, policy currentness, or positive evidence.

#![forbid(unsafe_code)]

use mycelix_authority_scoped_kvector_verification_record_policy::{
    derive_authority_scoped_kvector_verification_record_signing_digest_v2,
    validate_authority_scoped_kvector_verification_record_binding_v2,
    AuthorityScopedKVectorProofVerificationRecordBodyV2,
    AuthorityScopedKVectorVerificationRecordErrorV2,
};
use mycelix_authority_scoped_verifier_key_generation_policy::QualifiedAuthorityScopedVerifierKeyGenerationV2;
use mycelix_authority_scoped_verifier_policy::{
    validate_authority_scoped_verification_record_against_policy_v2,
    AuthorityScopedVerifierPolicyErrorV2, QualifiedAuthorityScopedVerifierPolicyV2,
};
use mycelix_kvector_verifier_key_generation_policy::KVectorVerifierKeyGenerationV2;
use mycelix_kvector_verifier_policy_body::KVectorVerifierPolicyBodyV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_SCOPED_PRECRYPTO_CONTEXT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:authority-scoped-precrypto-context:v2\0";

#[derive(Debug)]
pub struct QualifiedAuthorityScopedPrecryptoContextV2 {
    authority_domain_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    record_signing_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_generation_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    precrypto_context_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedAuthorityScopedPrecryptoContextV2 {
    pub fn authority_domain_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }

    pub fn record_signing_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.record_signing_digest_sha256
    }

    pub fn scoped_generation_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_generation_digest_sha256
    }

    pub fn scoped_policy_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_policy_digest_sha256
    }

    pub fn precrypto_context_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.precrypto_context_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityScopedPrecryptoContextErrorV2 {
    RecordGenerationDomainMismatch,
    RecordPolicyDomainMismatch,
    GenerationPolicyDomainMismatch,
    RecordBinding(AuthorityScopedKVectorVerificationRecordErrorV2),
    PolicyBinding(AuthorityScopedVerifierPolicyErrorV2),
}

impl From<AuthorityScopedKVectorVerificationRecordErrorV2>
    for AuthorityScopedPrecryptoContextErrorV2
{
    fn from(value: AuthorityScopedKVectorVerificationRecordErrorV2) -> Self {
        Self::RecordBinding(value)
    }
}

impl From<AuthorityScopedVerifierPolicyErrorV2> for AuthorityScopedPrecryptoContextErrorV2 {
    fn from(value: AuthorityScopedVerifierPolicyErrorV2) -> Self {
        Self::PolicyBinding(value)
    }
}

fn derive_precrypto_context_digest_v2(
    authority_domain_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    record_signing_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    scoped_generation_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
    scoped_policy_digest_sha256: &[u8; SHA256_DIGEST_LEN_V2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(AUTHORITY_SCOPED_PRECRYPTO_CONTEXT_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(authority_domain_digest_sha256);
    hasher.update([0x02]);
    hasher.update(record_signing_digest_sha256);
    hasher.update([0x03]);
    hasher.update(scoped_generation_digest_sha256);
    hasher.update([0x04]);
    hasher.update(scoped_policy_digest_sha256);
    hasher.finalize().into()
}

/// Qualify one pure structural pre-crypto context.
///
/// The explicit equality checks are intentionally redundant with lower-level binding
/// theorems: they make the complete authority-domain equality chain local and auditable.
pub fn qualify_authority_scoped_precrypto_context_v2(
    record: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
    scoped_generation: &QualifiedAuthorityScopedVerifierKeyGenerationV2,
    generation: KVectorVerifierKeyGenerationV2<'_>,
    scoped_policy: &QualifiedAuthorityScopedVerifierPolicyV2,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<QualifiedAuthorityScopedPrecryptoContextV2, AuthorityScopedPrecryptoContextErrorV2> {
    if record.authority_domain_sha256 != scoped_generation.authority_domain_digest_sha256() {
        return Err(AuthorityScopedPrecryptoContextErrorV2::RecordGenerationDomainMismatch);
    }
    if record.authority_domain_sha256 != scoped_policy.authority_domain_digest_sha256() {
        return Err(AuthorityScopedPrecryptoContextErrorV2::RecordPolicyDomainMismatch);
    }
    if scoped_generation.authority_domain_digest_sha256()
        != scoped_policy.authority_domain_digest_sha256()
    {
        return Err(AuthorityScopedPrecryptoContextErrorV2::GenerationPolicyDomainMismatch);
    }

    validate_authority_scoped_kvector_verification_record_binding_v2(
        record,
        scoped_generation,
        generation,
    )?;
    validate_authority_scoped_verification_record_against_policy_v2(
        record,
        scoped_policy,
        policy,
    )?;

    let record_signing_digest_sha256 =
        derive_authority_scoped_kvector_verification_record_signing_digest_v2(record)?;
    let authority_domain_digest_sha256 = *scoped_generation.authority_domain_digest_sha256();
    let scoped_generation_digest_sha256 = *scoped_generation.scoped_generation_digest_sha256();
    let scoped_policy_digest_sha256 = *scoped_policy.scoped_policy_digest_sha256();
    let precrypto_context_digest_sha256 = derive_precrypto_context_digest_v2(
        &authority_domain_digest_sha256,
        &record_signing_digest_sha256,
        &scoped_generation_digest_sha256,
        &scoped_policy_digest_sha256,
    );

    Ok(QualifiedAuthorityScopedPrecryptoContextV2 {
        authority_domain_digest_sha256,
        record_signing_digest_sha256,
        scoped_generation_digest_sha256,
        scoped_policy_digest_sha256,
        precrypto_context_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_scoped_verifier_key_generation_policy::qualify_authority_scoped_verifier_key_generation_v2;
    use mycelix_authority_scoped_verifier_policy::qualify_authority_scoped_verifier_policy_v2;
    use mycelix_crypto::AlgorithmId;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };
    use mycelix_kvector_verification_record_policy::{
        KVectorProofVerificationOutcomeV2, KVectorProofVerificationRecordBodyV2,
    };
    use mycelix_kvector_verifier_key_generation_policy::{
        derive_kvector_verifier_key_generation_digest_v2, KVectorVerifierKeyGenerationV2,
    };
    use mycelix_kvector_verifier_policy_body::{
        derive_kvector_verifier_policy_digest_v2, KVectorVerifierPolicyBodyV2,
    };

    const DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];
    const PRECRYPTO_CONTEXT_DIGEST_VECTOR: [u8; SHA256_DIGEST_LEN_V2] = [
        0x8f, 0x5c, 0x42, 0x2f, 0x95, 0xde, 0xe5, 0xb6, 0x2b, 0xd5, 0x80, 0xba, 0x33,
        0xd1, 0xa2, 0xfb, 0xb8, 0x29, 0x16, 0x8f, 0xb2, 0x42, 0x2e, 0x1d, 0x54, 0xac,
        0xb5, 0x9a, 0x35, 0x09, 0x56, 0xad,
    ];

    fn domain(dna: &[u8]) -> mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2 {
        qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: dna,
        })
        .unwrap()
    }

    fn generation(key: &[u8]) -> KVectorVerifierKeyGenerationV2<'_> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#hybrid-1",
            algorithm: AlgorithmId::HybridEd25519MlDsa65,
            public_key_bytes: key,
            key_generation: 1,
            issued_at_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
        }
    }

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

    fn record<'a>(
        generation_digest: &'a [u8],
        policy_digest: &'a [u8],
        domain_digest: &'a [u8],
        scoped_generation_digest: &'a [u8],
    ) -> AuthorityScopedKVectorProofVerificationRecordBodyV2<'a> {
        static FULFILLMENT: [u8; 32] = [0x11; 32];
        static STATEMENT: [u8; 32] = [0x22; 32];
        static PROOF: [u8; 32] = [0x33; 32];
        AuthorityScopedKVectorProofVerificationRecordBodyV2 {
            base_record: KVectorProofVerificationRecordBodyV2 {
                fulfillment_id: &FULFILLMENT,
                proof_statement_sha256: &STATEMENT,
                proof_sha256: &PROOF,
                backend_id: "winterfell-v2",
                circuit_id: "mycelix-kvector-range-v2",
                circuit_version: "2.0.0",
                verifier_did: "did:mycelix:verifier",
                verifier_key_id: "did:mycelix:verifier#hybrid-1",
                verifier_key_generation_sha256: generation_digest,
                signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
                verification_policy_sha256: policy_digest,
                outcome: KVectorProofVerificationOutcomeV2::Accepted,
                verified_at_micros: 2_000_000,
                valid_until_micros: 4_000_000,
            },
            authority_domain_sha256: domain_digest,
            verifier_key_generation_scoped_sha256: scoped_generation_digest,
        }
    }

    fn qualified_context_for_domain(
        domain: &mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2,
    ) -> QualifiedAuthorityScopedPrecryptoContextV2 {
        let key = vec![0x42; AlgorithmId::HybridEd25519MlDsa65.public_key_size()];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let scoped_generation =
            qualify_authority_scoped_verifier_key_generation_v2(domain, generation).unwrap();
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let scoped_policy = qualify_authority_scoped_verifier_policy_v2(domain, policy()).unwrap();
        let record = record(
            &generation_digest,
            &policy_digest,
            domain.digest_sha256(),
            scoped_generation.scoped_generation_digest_sha256(),
        );
        qualify_authority_scoped_precrypto_context_v2(
            record,
            &scoped_generation,
            generation,
            &scoped_policy,
            policy(),
        )
        .unwrap()
    }

    #[test]
    fn one_domain_record_generation_policy_context_qualifies_and_vector_is_frozen() {
        let domain = domain(&DNA);
        let context = qualified_context_for_domain(&domain);
        assert_eq!(context.authority_domain_digest_sha256(), domain.digest_sha256());
        assert_eq!(
            context.precrypto_context_digest_sha256(),
            &PRECRYPTO_CONTEXT_DIGEST_VECTOR
        );
    }

    #[test]
    fn different_authority_domain_changes_precrypto_context_identity() {
        let first_domain = domain(&DNA);
        let first = qualified_context_for_domain(&first_domain);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second_domain = domain(&other_dna);
        let second = qualified_context_for_domain(&second_domain);
        assert_ne!(
            first.precrypto_context_digest_sha256(),
            second.precrypto_context_digest_sha256()
        );
    }

    #[test]
    fn mixed_policy_domain_fails_before_precrypto_context() {
        let first = domain(&DNA);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second = domain(&other_dna);
        let key = vec![0x42; AlgorithmId::HybridEd25519MlDsa65.public_key_size()];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let scoped_generation = qualify_authority_scoped_verifier_key_generation_v2(&first, generation).unwrap();
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let wrong_policy = qualify_authority_scoped_verifier_policy_v2(&second, policy()).unwrap();
        let record = record(
            &generation_digest,
            &policy_digest,
            first.digest_sha256(),
            scoped_generation.scoped_generation_digest_sha256(),
        );

        assert!(matches!(
            qualify_authority_scoped_precrypto_context_v2(
                record,
                &scoped_generation,
                generation,
                &wrong_policy,
                policy(),
            ),
            Err(AuthorityScopedPrecryptoContextErrorV2::RecordPolicyDomainMismatch)
        ));
    }

    #[test]
    fn mixed_generation_domain_fails_before_precrypto_context() {
        let first = domain(&DNA);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second = domain(&other_dna);
        let key = vec![0x42; AlgorithmId::HybridEd25519MlDsa65.public_key_size()];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let wrong_generation = qualify_authority_scoped_verifier_key_generation_v2(&second, generation).unwrap();
        let scoped_policy = qualify_authority_scoped_verifier_policy_v2(&first, policy()).unwrap();
        let policy_digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let record = record(
            &generation_digest,
            &policy_digest,
            first.digest_sha256(),
            wrong_generation.scoped_generation_digest_sha256(),
        );

        assert!(matches!(
            qualify_authority_scoped_precrypto_context_v2(
                record,
                &wrong_generation,
                generation,
                &scoped_policy,
                policy(),
            ),
            Err(AuthorityScopedPrecryptoContextErrorV2::RecordGenerationDomainMismatch)
        ));
    }

    #[test]
    fn qualified_context_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedAuthorityScopedPrecryptoContextV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedAuthorityScopedPrecryptoContextV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_digest_sha256:",
            "pub record_signing_digest_sha256:",
            "pub scoped_generation_digest_sha256:",
            "pub scoped_policy_digest_sha256:",
            "pub precrypto_context_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
