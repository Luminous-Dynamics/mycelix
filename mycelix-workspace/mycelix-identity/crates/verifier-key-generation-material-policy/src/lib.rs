// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact verifier-key generation material assembly for Identity V2.
//!
//! This crate joins one untrusted normalized lifecycle assertion from the #340 generation
//! entry/action boundary to one opaque policy-neutral admitted authentication method from
//! #344. It reconstructs and revalidates the exact #316 generation tuple and digest.
//!
//! This remains a pure material theorem. It does not prove that the normalized lifecycle
//! assertion came from qualified Holochain activity, that the referenced DID action was
//! fetched under complete network coverage, that the generation was trusted-active at any
//! wall-clock instant, or that any signature verifies.

#![forbid(unsafe_code)]

use mycelix_did_authentication_method_material_policy::AdmittedDidAuthenticationMethodV2;
use mycelix_kvector_verifier_key_generation_policy::{
    derive_kvector_verifier_key_generation_digest_v2,
    validate_kvector_verifier_key_generation_v2,
    KVectorVerifierKeyGenerationErrorV2, KVectorVerifierKeyGenerationV2,
    SHA256_DIGEST_LEN_V2,
};

pub const ACTION_ID_MAX_LEN_V2: usize = 256;

/// Untrusted normalized lifecycle facts expected from one exact #340 generation create
/// action plus its immutable entry. A later observer must prove these fields came from the
/// exact qualified action before an assembled result may be promoted to observed evidence.
#[derive(Debug, Clone, Copy)]
pub struct VerifierKeyGenerationLifecycleAssertionV2<'a> {
    pub generation_action_id: &'a str,
    pub did_document_action_id: &'a str,
    pub previous_generation_action_id: Option<&'a str>,
    pub verifier_did: &'a str,
    pub verifier_key_id: &'a str,
    pub key_generation: u64,
    /// Immutable author/source-chain provenance timestamp in Unix microseconds.
    /// This becomes #316 `issued_at_micros`; it is not trusted activation time.
    pub generation_action_timestamp_micros: i64,
    pub valid_from_micros: i64,
    pub valid_until_micros: i64,
}

/// Opaque owned assembly of one exact #316 verifier-key generation from lifecycle facts
/// and exact admitted DID authentication material.
///
/// It is intentionally not `Clone` or `Copy`. This is still not authority: the later
/// observation bridge must prove source provenance/coverage and #337 must prove trusted
/// activation before historical signature eligibility.
#[derive(Debug)]
pub struct AssembledVerifierKeyGenerationV2 {
    generation_action_id: String,
    did_document_action_id: String,
    previous_generation_action_id: Option<String>,
    method: AdmittedDidAuthenticationMethodV2,
    key_generation: u64,
    issued_at_micros: i64,
    valid_from_micros: i64,
    valid_until_micros: i64,
    generation_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl AssembledVerifierKeyGenerationV2 {
    pub fn generation_action_id(&self) -> &str {
        &self.generation_action_id
    }

    pub fn did_document_action_id(&self) -> &str {
        &self.did_document_action_id
    }

    pub fn previous_generation_action_id(&self) -> Option<&str> {
        self.previous_generation_action_id.as_deref()
    }

    pub fn verifier_did(&self) -> &str {
        self.method.did()
    }

    pub fn verifier_key_id(&self) -> &str {
        self.method.canonical_key_id()
    }

    pub fn key_generation(&self) -> u64 {
        self.key_generation
    }

    pub fn issued_at_micros(&self) -> i64 {
        self.issued_at_micros
    }

    pub fn valid_from_micros(&self) -> i64 {
        self.valid_from_micros
    }

    pub fn valid_until_micros(&self) -> i64 {
        self.valid_until_micros
    }

    pub fn generation_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.generation_sha256
    }

    /// Borrow the exact #316 generation tuple reconstructed by this theorem.
    pub fn as_generation(&self) -> KVectorVerifierKeyGenerationV2<'_> {
        KVectorVerifierKeyGenerationV2 {
            verifier_did: self.method.did(),
            verifier_key_id: self.method.canonical_key_id(),
            algorithm: self.method.algorithm(),
            public_key_bytes: self.method.public_key_bytes(),
            key_generation: self.key_generation,
            issued_at_micros: self.issued_at_micros,
            valid_from_micros: self.valid_from_micros,
            valid_until_micros: self.valid_until_micros,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerifierKeyGenerationMaterialErrorV2 {
    GenerationActionIdInvalid,
    DidDocumentActionIdInvalid,
    PreviousGenerationActionIdInvalid,
    GenerationActionReusesDidDocumentAction,
    RootGenerationPredecessorInvalid,
    SuccessorGenerationPredecessorMissing,
    PreviousGenerationSelfReference,
    PreviousGenerationReusesDidDocumentAction,
    DidDocumentActionMismatch,
    VerifierDidMismatch,
    VerifierKeyIdMismatch,
    Generation(KVectorVerifierKeyGenerationErrorV2),
}

fn valid_action_id(value: &str) -> bool {
    !value.is_empty() && value.len() <= ACTION_ID_MAX_LEN_V2
}

/// Assemble one exact #316 generation from one normalized #340 lifecycle assertion and
/// one exact admitted DID authentication method bound to the referenced DID action.
///
/// `admitted_did_document_action_id` is supplied separately from the lifecycle assertion
/// so action substitution is an explicit checked relation rather than a convention.
pub fn assemble_verifier_key_generation_material_v2(
    lifecycle: VerifierKeyGenerationLifecycleAssertionV2<'_>,
    admitted_did_document_action_id: &str,
    method: AdmittedDidAuthenticationMethodV2,
) -> Result<AssembledVerifierKeyGenerationV2, VerifierKeyGenerationMaterialErrorV2> {
    if !valid_action_id(lifecycle.generation_action_id) {
        return Err(VerifierKeyGenerationMaterialErrorV2::GenerationActionIdInvalid);
    }
    if !valid_action_id(lifecycle.did_document_action_id)
        || !valid_action_id(admitted_did_document_action_id)
    {
        return Err(VerifierKeyGenerationMaterialErrorV2::DidDocumentActionIdInvalid);
    }
    if lifecycle.generation_action_id == lifecycle.did_document_action_id {
        return Err(
            VerifierKeyGenerationMaterialErrorV2::GenerationActionReusesDidDocumentAction,
        );
    }
    if lifecycle.key_generation == 0 {
        return Err(VerifierKeyGenerationMaterialErrorV2::Generation(
            KVectorVerifierKeyGenerationErrorV2::KeyGenerationInvalid,
        ));
    }

    match (lifecycle.key_generation, lifecycle.previous_generation_action_id) {
        (1, None) => {}
        (1, Some(_)) => {
            return Err(VerifierKeyGenerationMaterialErrorV2::RootGenerationPredecessorInvalid)
        }
        (_, None) => {
            return Err(
                VerifierKeyGenerationMaterialErrorV2::SuccessorGenerationPredecessorMissing,
            )
        }
        (_, Some(previous)) => {
            if !valid_action_id(previous) {
                return Err(
                    VerifierKeyGenerationMaterialErrorV2::PreviousGenerationActionIdInvalid,
                );
            }
            if previous == lifecycle.generation_action_id {
                return Err(VerifierKeyGenerationMaterialErrorV2::PreviousGenerationSelfReference);
            }
            if previous == lifecycle.did_document_action_id {
                return Err(
                    VerifierKeyGenerationMaterialErrorV2::PreviousGenerationReusesDidDocumentAction,
                );
            }
        }
    }

    if admitted_did_document_action_id != lifecycle.did_document_action_id {
        return Err(VerifierKeyGenerationMaterialErrorV2::DidDocumentActionMismatch);
    }
    if method.did() != lifecycle.verifier_did {
        return Err(VerifierKeyGenerationMaterialErrorV2::VerifierDidMismatch);
    }
    if method.canonical_key_id() != lifecycle.verifier_key_id {
        return Err(VerifierKeyGenerationMaterialErrorV2::VerifierKeyIdMismatch);
    }

    let generation = KVectorVerifierKeyGenerationV2 {
        verifier_did: method.did(),
        verifier_key_id: method.canonical_key_id(),
        algorithm: method.algorithm(),
        public_key_bytes: method.public_key_bytes(),
        key_generation: lifecycle.key_generation,
        issued_at_micros: lifecycle.generation_action_timestamp_micros,
        valid_from_micros: lifecycle.valid_from_micros,
        valid_until_micros: lifecycle.valid_until_micros,
    };
    validate_kvector_verifier_key_generation_v2(generation)
        .map_err(VerifierKeyGenerationMaterialErrorV2::Generation)?;
    let generation_sha256 = derive_kvector_verifier_key_generation_digest_v2(generation)
        .map_err(VerifierKeyGenerationMaterialErrorV2::Generation)?;

    Ok(AssembledVerifierKeyGenerationV2 {
        generation_action_id: lifecycle.generation_action_id.to_string(),
        did_document_action_id: lifecycle.did_document_action_id.to_string(),
        previous_generation_action_id: lifecycle.previous_generation_action_id.map(str::to_string),
        method,
        key_generation: lifecycle.key_generation,
        issued_at_micros: lifecycle.generation_action_timestamp_micros,
        valid_from_micros: lifecycle.valid_from_micros,
        valid_until_micros: lifecycle.valid_until_micros,
        generation_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_did_authentication_method_material_policy::resolve_admitted_did_authentication_method_v2;
    use mycelix_did_document_crypto_policy::{
        DidDocumentCryptoAdmissionViewV2, DidVerificationMethodAdmissionViewV2,
    };
    use mycelix_crypto::{AlgorithmId, TaggedPublicKey};

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#key-1";

    fn admitted_method(fill: u8) -> AdmittedDidAuthenticationMethodV2 {
        let algorithm = AlgorithmId::Ed25519;
        let multibase = TaggedPublicKey::new(
            algorithm,
            vec![fill; algorithm.public_key_size()],
        )
        .unwrap()
        .to_multibase();
        let methods = [DidVerificationMethodAdmissionViewV2 {
            id: "#key-1",
            type_: algorithm.did_verification_method_type(),
            controller: DID,
            public_key_multibase: &multibase,
            algorithm: Some(algorithm.as_u16()),
        }];
        let authentication = ["#key-1"];
        resolve_admitted_did_authentication_method_v2(
            DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &authentication,
                key_agreement: &[],
            },
            KEY_ID,
        )
        .unwrap()
    }

    fn lifecycle<'a>() -> VerifierKeyGenerationLifecycleAssertionV2<'a> {
        VerifierKeyGenerationLifecycleAssertionV2 {
            generation_action_id: "gen-action-1",
            did_document_action_id: "did-action-7",
            previous_generation_action_id: None,
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            key_generation: 1,
            generation_action_timestamp_micros: 900_000,
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
        }
    }

    #[test]
    fn exact_lifecycle_and_admitted_material_reconstruct_generation() {
        let method = admitted_method(0x42);
        let assembled = assemble_verifier_key_generation_material_v2(
            lifecycle(),
            "did-action-7",
            method,
        )
        .unwrap();

        assert_eq!(assembled.generation_action_id(), "gen-action-1");
        assert_eq!(assembled.did_document_action_id(), "did-action-7");
        assert_eq!(assembled.previous_generation_action_id(), None);
        assert_eq!(assembled.verifier_did(), DID);
        assert_eq!(assembled.verifier_key_id(), KEY_ID);
        assert_eq!(assembled.key_generation(), 1);
        assert_eq!(assembled.issued_at_micros(), 900_000);

        let generation = assembled.as_generation();
        assert_eq!(generation.algorithm, AlgorithmId::Ed25519);
        assert_eq!(generation.public_key_bytes, &[0x42; 32]);
        let digest = derive_kvector_verifier_key_generation_digest_v2(generation).unwrap();
        assert_eq!(assembled.generation_sha256(), &digest);
    }

    #[test]
    fn did_document_action_substitution_fails_closed() {
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                lifecycle(),
                "did-action-other",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::DidDocumentActionMismatch
        );
    }

    #[test]
    fn did_and_key_namespace_must_match_admitted_method() {
        let mut wrong_did = lifecycle();
        wrong_did.verifier_did = "did:mycelix:other";
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                wrong_did,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::VerifierDidMismatch
        );

        let mut wrong_key = lifecycle();
        wrong_key.verifier_key_id = "did:mycelix:verifier#key-2";
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                wrong_key,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::VerifierKeyIdMismatch
        );
    }

    #[test]
    fn zero_generation_reuses_exact_316_failure() {
        let mut assertion = lifecycle();
        assertion.key_generation = 0;
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                assertion,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::Generation(
                KVectorVerifierKeyGenerationErrorV2::KeyGenerationInvalid,
            )
        );
    }

    #[test]
    fn source_chain_timestamp_cannot_bypass_generation_validity_shape() {
        let mut assertion = lifecycle();
        assertion.generation_action_timestamp_micros = 1_000_001;
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                assertion,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::Generation(
                KVectorVerifierKeyGenerationErrorV2::IssuedAfterValidityStart,
            )
        );
    }

    #[test]
    fn all_zero_admitted_key_is_rejected_by_generation_theorem() {
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                lifecycle(),
                "did-action-7",
                admitted_method(0x00),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::Generation(
                KVectorVerifierKeyGenerationErrorV2::PublicKeyAllZero,
            )
        );
    }

    #[test]
    fn predecessor_shape_is_rechecked() {
        let mut root_with_parent = lifecycle();
        root_with_parent.previous_generation_action_id = Some("gen-action-0");
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                root_with_parent,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::RootGenerationPredecessorInvalid
        );

        let mut successor_without_parent = lifecycle();
        successor_without_parent.key_generation = 2;
        assert_eq!(
            assemble_verifier_key_generation_material_v2(
                successor_without_parent,
                "did-action-7",
                admitted_method(0x42),
            )
            .unwrap_err(),
            VerifierKeyGenerationMaterialErrorV2::SuccessorGenerationPredecessorMissing
        );
    }

    #[test]
    fn assembled_generation_fields_are_not_public() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct AssembledVerifierKeyGenerationV2")
            .unwrap();
        let end = source[start..]
            .index("impl AssembledVerifierKeyGenerationV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for public_field in [
            "pub generation_action_id:",
            "pub did_document_action_id:",
            "pub previous_generation_action_id:",
            "pub method:",
            "pub key_generation:",
            "pub issued_at_micros:",
            "pub valid_from_micros:",
            "pub valid_until_micros:",
            "pub generation_sha256:",
        ] {
            assert!(!body.contains(public_field));
        }
    }

    #[test]
    fn assembled_generation_is_not_clone_or_copy() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct AssembledVerifierKeyGenerationV2")
            .unwrap();
        let prefix = &source[..start];
        let derive_start = prefix.rfind("#[derive(").unwrap();
        let derive = &prefix[derive_start..];
        assert!(!derive.contains("Clone"));
        assert!(!derive.contains("Copy"));
    }
}
