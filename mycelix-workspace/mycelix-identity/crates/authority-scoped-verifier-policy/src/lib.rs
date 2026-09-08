// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Authority-domain-scoped static verifier-policy identity for Identity V2.
//!
//! This theorem composes the repaired-stack #248 static policy body with one opaque
//! #369 authority-domain identity. It does not change #248 policy semantics or claim
//! that a policy is accepted/current.

#![forbid(unsafe_code)]

use mycelix_authority_scoped_kvector_verification_record_policy::{
    validate_authority_scoped_kvector_verification_record_body_v2,
    AuthorityScopedKVectorProofVerificationRecordBodyV2,
    AuthorityScopedKVectorVerificationRecordErrorV2,
};
use mycelix_identity_authority_domain_policy::QualifiedIdentityAuthorityDomainV2;
use mycelix_kvector_verifier_policy_body::{
    derive_kvector_verifier_policy_digest_v2,
    validate_kvector_verification_record_against_policy_body_v2,
    KVectorVerifierPolicyBodyErrorV2, KVectorVerifierPolicyBodyV2,
};
use sha2::{Digest, Sha256};

pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const AUTHORITY_SCOPED_KVECTOR_VERIFIER_POLICY_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-verifier-policy:authority-scoped:v2\0";

#[derive(Debug)]
pub struct QualifiedAuthorityScopedVerifierPolicyV2 {
    authority_domain_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    base_policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
    scoped_policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedAuthorityScopedVerifierPolicyV2 {
    pub fn authority_domain_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.authority_domain_digest_sha256
    }

    pub fn base_policy_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.base_policy_digest_sha256
    }

    pub fn scoped_policy_digest_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.scoped_policy_digest_sha256
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthorityScopedVerifierPolicyErrorV2 {
    BasePolicy(KVectorVerifierPolicyBodyErrorV2),
    ScopedRecord(AuthorityScopedKVectorVerificationRecordErrorV2),
    AuthorityDomainMismatch,
    BasePolicyDigestMismatch,
}

impl From<KVectorVerifierPolicyBodyErrorV2> for AuthorityScopedVerifierPolicyErrorV2 {
    fn from(value: KVectorVerifierPolicyBodyErrorV2) -> Self {
        Self::BasePolicy(value)
    }
}

impl From<AuthorityScopedKVectorVerificationRecordErrorV2>
    for AuthorityScopedVerifierPolicyErrorV2
{
    fn from(value: AuthorityScopedKVectorVerificationRecordErrorV2) -> Self {
        Self::ScopedRecord(value)
    }
}

fn derive_scoped_policy_digest_from_qualified_parts_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    base_policy_digest_sha256: [u8; SHA256_DIGEST_LEN_V2],
) -> [u8; SHA256_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(AUTHORITY_SCOPED_KVECTOR_VERIFIER_POLICY_DOMAIN_V2);
    hasher.update(authority_domain.digest_sha256());
    hasher.update(base_policy_digest_sha256);
    hasher.finalize().into()
}

pub fn derive_authority_scoped_verifier_policy_digest_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<[u8; SHA256_DIGEST_LEN_V2], AuthorityScopedVerifierPolicyErrorV2> {
    let base_policy_digest_sha256 = derive_kvector_verifier_policy_digest_v2(policy)?;
    Ok(derive_scoped_policy_digest_from_qualified_parts_v2(
        authority_domain,
        base_policy_digest_sha256,
    ))
}

pub fn qualify_authority_scoped_verifier_policy_v2(
    authority_domain: &QualifiedIdentityAuthorityDomainV2,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<QualifiedAuthorityScopedVerifierPolicyV2, AuthorityScopedVerifierPolicyErrorV2> {
    let base_policy_digest_sha256 = derive_kvector_verifier_policy_digest_v2(policy)?;
    let scoped_policy_digest_sha256 = derive_scoped_policy_digest_from_qualified_parts_v2(
        authority_domain,
        base_policy_digest_sha256,
    );

    Ok(QualifiedAuthorityScopedVerifierPolicyV2 {
        authority_domain_digest_sha256: *authority_domain.digest_sha256(),
        base_policy_digest_sha256,
        scoped_policy_digest_sha256,
    })
}

/// Prove static compatibility between one authority-scoped signed record envelope and
/// one authority-scoped static policy identity.
///
/// Success does not mean the policy is accepted/current and does not authenticate the
/// record signature. It proves only exact domain equality plus #248 static compatibility.
pub fn validate_authority_scoped_verification_record_against_policy_v2(
    record: AuthorityScopedKVectorProofVerificationRecordBodyV2<'_>,
    scoped_policy: &QualifiedAuthorityScopedVerifierPolicyV2,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<(), AuthorityScopedVerifierPolicyErrorV2> {
    validate_authority_scoped_kvector_verification_record_body_v2(record)?;

    if record.authority_domain_sha256 != scoped_policy.authority_domain_digest_sha256() {
        return Err(AuthorityScopedVerifierPolicyErrorV2::AuthorityDomainMismatch);
    }

    let base_policy_digest = derive_kvector_verifier_policy_digest_v2(policy)?;
    if base_policy_digest.as_slice() != scoped_policy.base_policy_digest_sha256() {
        return Err(AuthorityScopedVerifierPolicyErrorV2::BasePolicyDigestMismatch);
    }

    validate_kvector_verification_record_against_policy_body_v2(record.base_record, policy)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_identity_authority_domain_policy::{
        qualify_identity_authority_domain_v2, IdentityAuthorityDomainStatementV2,
    };
    use mycelix_kvector_verification_record_policy::{
        KVectorProofVerificationOutcomeV2, KVectorProofVerificationRecordBodyV2,
    };

    const DNA: [u8; 39] = [
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
        policy_digest: &'a [u8],
        domain_digest: &'a [u8],
    ) -> AuthorityScopedKVectorProofVerificationRecordBodyV2<'a> {
        static FULFILLMENT: [u8; 32] = [0x11; 32];
        static STATEMENT: [u8; 32] = [0x22; 32];
        static PROOF: [u8; 32] = [0x33; 32];
        static GENERATION: [u8; 32] = [0x44; 32];
        static SCOPED_GENERATION: [u8; 32] = [0x55; 32];
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
                verifier_key_generation_sha256: &GENERATION,
                signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
                verification_policy_sha256: policy_digest,
                outcome: KVectorProofVerificationOutcomeV2::Accepted,
                verified_at_micros: 2_000_000,
                valid_until_micros: 4_000_000,
            },
            authority_domain_sha256: domain_digest,
            verifier_key_generation_scoped_sha256: &SCOPED_GENERATION,
        }
    }

    #[test]
    fn frozen_authority_scoped_policy_digest_is_stable() {
        let domain = domain(&DNA, 1);
        assert_eq!(
            derive_authority_scoped_verifier_policy_digest_v2(&domain, policy()).unwrap(),
            [
                0xfe, 0x15, 0x7b, 0x46, 0x63, 0xd2, 0x21, 0x3b, 0x5c, 0xf0, 0x96, 0x97,
                0xb4, 0x41, 0xfa, 0xc2, 0x82, 0x05, 0x16, 0x56, 0x42, 0xc3, 0x40, 0x1c,
                0xff, 0xab, 0x70, 0xa7, 0x1f, 0x88, 0xd1, 0x27,
            ]
        );
    }

    #[test]
    fn same_static_policy_in_different_dna_is_different_scoped_policy() {
        let first = domain(&DNA, 1);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second = domain(&other_dna, 1);
        let first_policy = qualify_authority_scoped_verifier_policy_v2(&first, policy()).unwrap();
        let second_policy = qualify_authority_scoped_verifier_policy_v2(&second, policy()).unwrap();
        assert_eq!(
            first_policy.base_policy_digest_sha256(),
            second_policy.base_policy_digest_sha256()
        );
        assert_ne!(
            first_policy.authority_domain_digest_sha256(),
            second_policy.authority_domain_digest_sha256()
        );
        assert_ne!(
            first_policy.scoped_policy_digest_sha256(),
            second_policy.scoped_policy_digest_sha256()
        );
    }

    #[test]
    fn record_and_policy_require_exact_domain_equality() {
        let first = domain(&DNA, 1);
        let mut other_dna = DNA;
        other_dna[20] ^= 0x55;
        let second = domain(&other_dna, 1);
        let scoped_first = qualify_authority_scoped_verifier_policy_v2(&first, policy()).unwrap();
        let scoped_second = qualify_authority_scoped_verifier_policy_v2(&second, policy()).unwrap();
        let base_digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let record = record(&base_digest, first.digest_sha256());

        assert_eq!(
            validate_authority_scoped_verification_record_against_policy_v2(
                record,
                &scoped_first,
                policy(),
            ),
            Ok(())
        );
        assert_eq!(
            validate_authority_scoped_verification_record_against_policy_v2(
                record,
                &scoped_second,
                policy(),
            ),
            Err(AuthorityScopedVerifierPolicyErrorV2::AuthorityDomainMismatch)
        );
    }

    #[test]
    fn base_policy_substitution_fails_closed() {
        let domain = domain(&DNA, 1);
        let scoped = qualify_authority_scoped_verifier_policy_v2(&domain, policy()).unwrap();
        let base_digest = derive_kvector_verifier_policy_digest_v2(policy()).unwrap();
        let record = record(&base_digest, domain.digest_sha256());
        let different = KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:different-v2",
            ..policy()
        };
        assert_eq!(
            validate_authority_scoped_verification_record_against_policy_v2(
                record,
                &scoped,
                different,
            ),
            Err(AuthorityScopedVerifierPolicyErrorV2::BasePolicyDigestMismatch)
        );
    }

    #[test]
    fn qualified_scoped_policy_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct QualifiedAuthorityScopedVerifierPolicyV2")
            .unwrap();
        let end = source[start..]
            .index("impl QualifiedAuthorityScopedVerifierPolicyV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for field in [
            "pub authority_domain_digest_sha256:",
            "pub base_policy_digest_sha256:",
            "pub scoped_policy_digest_sha256:",
        ] {
            assert!(!body.contains(field));
        }
    }
}
