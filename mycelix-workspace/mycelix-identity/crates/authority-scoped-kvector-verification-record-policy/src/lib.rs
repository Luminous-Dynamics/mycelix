// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Authority-scoped signed verification-record envelope for Identity V2.
//!
//! This theorem preserves #324 as the intrinsic generation-bound record body and adds
//! direct authority-domain commitment above it. A future signature must authenticate
//! this envelope digest, not the older #324 digest alone.

#![forbid(unsafe_code)]

use mycelix_authority_scoped_verifier_key_generation_policy::QualifiedAuthorityScopedVerifierKeyGenerationV2;
use mycelix_kvector_verification_record_policy::{
    derive_kvector_verification_record_signing_digest_v2,
    validate_kvector_verification_record_body_v2,
    validate_kvector_verification_record_key_generation_binding_v2,
    KVectorProofVerificationRecordBodyV2, KVectorVerificationRecordErrorV2,
};
use mycelix_kvector_verifier_key_generation_policy::KVectorVerifierKeyGenerationV2;
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_SCOPED_KVECTOR_VERIFICATION_RECORD_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verification-record:authority-scoped:v2\0";

/// Signed envelope above the #324 intrinsic record body.
#[derive(Debug, Clone, Copy)]
pub struct AuthorityScopedKVectorProofVerificationRecordBodyV2<'a> {
    pub base_record: KVectorProofVerificationRecordBodyV2<'a>,
    pub authority_domain_sha256: &'a [u8],
    pub verifier_key_generation_scoped_sha256: &'a [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityScopedKVectorVerificationRecordErrorV2 {
    BaseRecord(KVectorVerificationRecordErrorV2),
    AuthorityDomainDigestLengthInvalid,
    ScopedGenerationDigestLengthInvalid,
    AuthorityDomainMismatch,
    IntrinsicGenerationMismatch,
    ScopedGenerationMismatch,
}

impl From<KVectorVerificationRecordErrorV2> for AuthorityScopedKVectorVerificationRecordErrorV2 {
    fn from(value: KVectorVerificationRecordErrorV2) -> Self {
        Self::BaseRecord(value)
    }
}

pub fn validate_authority_scoped_kvector_verification_record_body_v2(
    body: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
) -> Result<(), AuthorityScopedKVectorVerificationRecordErrorV2> {
    validate_kvector_verification_record_body_v2(body.base_record)?;
    if body.authority_domain_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(
            AuthorityScopedKVectorVerificationRecordErrorV2::AuthorityDomainDigestLengthInvalid,
        );
    }
    if body.verifier_key_generation_scoped_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(
            AuthorityScopedKVectorVerificationRecordErrorV2::ScopedGenerationDigestLengthInvalid,
        );
    }
    Ok(())
}

/// Derive the exact digest the future verifier signature must authenticate.
///
/// The new transcript commits the already-generation-bound #324 digest plus a direct
/// authority-domain digest and the exact #371 authority-scoped generation digest.
pub fn derive_authority_scoped_kvector_verification_record_signing_digest_v2(
    body: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], AuthorityScopedKVectorVerificationRecordErrorV2> {
    validate_authority_scoped_kvector_verification_record_body_v2(body)?;
    let base_digest = derive_kvector_verification_record_signing_digest_v2(body.base_record)?;

    let mut hasher = Sha256::new();
    hasher.update(AUTHORITY_SCOPED_KVECTOR_VERIFICATION_RECORD_DOMAIN_V2);
    hasher.update([0x01]);
    hasher.update(base_digest);
    hasher.update([0x02]);
    hasher.update(body.authority_domain_sha256);
    hasher.update([0x03]);
    hasher.update(body.verifier_key_generation_scoped_sha256);
    Ok(hasher.finalize().into())
}

/// Bind the signed envelope to one opaque #371 scoped-generation capability and to the
/// exact #316 generation needed for #324's DID/key/time historical-use theorem.
pub fn validate_authority_scoped_kvector_verification_record_binding_v2(
    body: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
    scoped_generation: &QualifiedAuthorityScopedVerifierKeyGenerationV2,
    generation: KVectorVerifierKeyGenerationV2<'_>,
) -> Result<(), AuthorityScopedKVectorVerificationRecordErrorV2> {
    validate_authority_scoped_kvector_verification_record_body_v2(body)?;

    if body.authority_domain_sha256 != scoped_generation.authority_domain_digest_sha256() {
        return Err(AuthorityScopedKVectorVerificationRecordErrorV2::AuthorityDomainMismatch);
    }
    if body.base_record.verifier_key_generation_sha256
        != scoped_generation.intrinsic_generation_digest_sha256()
    {
        return Err(AuthorityScopedKVectorVerificationRecordErrorV2::IntrinsicGenerationMismatch);
    }
    if body.verifier_key_generation_scoped_sha256
        != scoped_generation.scoped_generation_digest_sha256()
    {
        return Err(AuthorityScopedKVectorVerificationRecordErrorV2::ScopedGenerationMismatch);
    }

    validate_kvector_verification_record_key_generation_binding_v2(body.base_record, generation)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_scoped_verifier_key_generation_policy::qualify_authority_scoped_verifier_key_generation_v2;
    use mycelix_crypto::AlgorithmId;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };
    use mycelix_kvector_verification_record_policy::KVectorProofVerificationOutcomeV2;
    use mycelix_kvector_verifier_key_generation_policy::{
        derive_kvector_verifier_key_generation_digest_v2, KVectorVerifierKeyGenerationV2,
    };

    const DNA: [u8; 39] = [
        0x84, 0x2d, 0x24, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
        0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x01, 0x02, 0x03, 0x04,
    ];

    fn generation(key: &[u8]) -> KVectorVerifierKeyGenerationV2<'_> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#key-1",
            algorithm: AlgorithmId::Ed25519,
            public_key_bytes: key,
            key_generation: 1,
            issued_at_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
        }
    }

    fn base_record<'a>(generation_digest: &'a [u8]) -> KVectorProofVerificationRecordBodyV2<'a> {
        static FULFILLMENT: [u8; 32] = [0x22; 32];
        static STATEMENT: [u8; 32] = [0x33; 32];
        static PROOF: [u8; 32] = [0x44; 32];
        static POLICY: [u8; 32] = [0x11; 32];
        KVectorProofVerificationRecordBodyV2 {
            fulfillment_id: &FULFILLMENT,
            proof_statement_sha256: &STATEMENT,
            proof_sha256: &PROOF,
            backend_id: "candidate-backend",
            circuit_id: "identity-kvector-v2",
            circuit_version: "0.1.0",
            verifier_did: "did:mycelix:verifier",
            verifier_key_id: "did:mycelix:verifier#key-1",
            verifier_key_generation_sha256: generation_digest,
            signature_scheme_id: "ed25519-v1",
            verification_policy_sha256: &POLICY,
            outcome: KVectorProofVerificationOutcomeV2::Accepted,
            verified_at_micros: 2_000_000,
            valid_until_micros: 8_000_000,
        }
    }

    #[test]
    fn authority_scoped_record_digest_vector_is_frozen() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let key = [0x42; 32];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let scoped = qualify_authority_scoped_verifier_key_generation_v2(&domain, generation).unwrap();
        let body = AuthorityScopedKVectorProofVerificationRecordBodyV2 {
            base_record: base_record(&generation_digest),
            authority_domain_sha256: domain.digest_sha256(),
            verifier_key_generation_scoped_sha256: scoped.scoped_generation_digest_sha256(),
        };
        assert_eq!(
            derive_authority_scoped_kvector_verification_record_signing_digest_v2(body).unwrap(),
            [
                0x91, 0x16, 0x5f, 0x6e, 0xe2, 0xc1, 0xf7, 0xd4, 0x7b, 0x7c, 0xb3, 0xdd,
                0x71, 0x4b, 0x92, 0xf7, 0xb7, 0x5c, 0xf1, 0x9a, 0x2e, 0x69, 0xff, 0xf0,
                0xd7, 0x4c, 0x91, 0xb9, 0xb8, 0xe7, 0x20, 0x66,
            ]
        );
        assert_eq!(
            validate_authority_scoped_kvector_verification_record_binding_v2(
                body,
                &scoped,
                generation,
            ),
            Ok(())
        );
    }

    #[test]
    fn cross_domain_replay_fails_binding_and_changes_signed_digest() {
        let first_domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second_domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &other_dna,
        })
        .unwrap();
        let key = [0x42; 32];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let first_scoped = qualify_authority_scoped_verifier_key_generation_v2(&first_domain, generation).unwrap();
        let second_scoped = qualify_authority_scoped_verifier_key_generation_v2(&second_domain, generation).unwrap();

        let first = AuthorityScopedKVectorProofVerificationRecordBodyV2 {
            base_record: base_record(&generation_digest),
            authority_domain_sha256: first_domain.digest_sha256(),
            verifier_key_generation_scoped_sha256: first_scoped.scoped_generation_digest_sha256(),
        };
        let second = AuthorityScopedKVectorProofVerificationRecordBodyV2 {
            base_record: base_record(&generation_digest),
            authority_domain_sha256: second_domain.digest_sha256(),
            verifier_key_generation_scoped_sha256: second_scoped.scoped_generation_digest_sha256(),
        };
        assert_ne!(
            derive_authority_scoped_kvector_verification_record_signing_digest_v2(first).unwrap(),
            derive_authority_scoped_kvector_verification_record_signing_digest_v2(second).unwrap()
        );
        assert_eq!(
            validate_authority_scoped_kvector_verification_record_binding_v2(
                first,
                &second_scoped,
                generation,
            ),
            Err(AuthorityScopedKVectorVerificationRecordErrorV2::AuthorityDomainMismatch)
        );
    }

    #[test]
    fn scoped_generation_substitution_fails_closed() {
        let domain = qualify_identity_authority_domain_v2(IdentityAuthorityDomainStatementV2 {
            authority_domain_id: "mycelix-identity-v2",
            authority_domain_epoch: 1,
            dna_hash_raw_39: &DNA,
        })
        .unwrap();
        let key = [0x42; 32];
        let generation = generation(&key);
        let generation_digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        let scoped = qualify_authority_scoped_verifier_key_generation_v2(&domain, generation).unwrap();
        let wrong = [0x55; 32];
        let body = AuthorityScopedKVectorProofVerificationRecordBodyV2 {
            base_record: base_record(&generation_digest),
            authority_domain_sha256: domain.digest_sha256(),
            verifier_key_generation_scoped_sha256: &wrong,
        };
        assert_eq!(
            validate_authority_scoped_kvector_verification_record_binding_v2(body, &scoped, generation),
            Err(AuthorityScopedKVectorVerificationRecordErrorV2::ScopedGenerationMismatch)
        );
    }
}
