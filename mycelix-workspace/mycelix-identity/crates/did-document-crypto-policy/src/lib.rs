// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Strict cryptographic admission theorem for Identity V2 DID documents.
//!
//! This crate defines the cryptographic structure that a newly admitted DID document
//! must satisfy before any coordinator/network/currentness logic is relevant.
//!
//! It intentionally rejects legacy non-canonical key encodings. Compatibility and
//! migration belong in an adapter before admission, never as exceptions inside this
//! theorem.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};

use mycelix_crypto::{AlgorithmId, TaggedPublicKey};

pub const DID_MAX_LEN_V2: usize = 256;
pub const METHOD_ID_MAX_LEN_V2: usize = 256;
pub const METHOD_TYPE_MAX_LEN_V2: usize = 256;
pub const CONTROLLER_MAX_LEN_V2: usize = 256;
pub const PUBLIC_KEY_MULTIBASE_MAX_LEN_V2: usize = 4096;
pub const MAX_VERIFICATION_METHODS_V2: usize = 100;
pub const MAX_AUTHENTICATION_REFERENCES_V2: usize = 100;
pub const MAX_KEY_AGREEMENT_REFERENCES_V2: usize = 100;

#[derive(Debug, Clone, Copy)]
pub struct DidVerificationMethodAdmissionViewV2<'a> {
    pub id: &'a str,
    pub type_: &'a str,
    pub controller: &'a str,
    pub public_key_multibase: &'a str,
    pub algorithm: Option<u16>,
}

#[derive(Debug, Clone, Copy)]
pub struct DidDocumentCryptoAdmissionViewV2<'a> {
    pub did: &'a str,
    pub verification_methods: &'a [DidVerificationMethodAdmissionViewV2<'a>],
    pub authentication: &'a [&'a str],
    pub key_agreement: &'a [&'a str],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidatedDidDocumentCryptoV2 {
    pub verification_method_count: usize,
    pub authentication_count: usize,
    pub key_agreement_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DidDocumentCryptoAdmissionErrorV2 {
    DocumentDidInvalid,
    VerificationMethodRequired,
    TooManyVerificationMethods,
    TooManyAuthenticationReferences,
    TooManyKeyAgreementReferences,
    MethodIdInvalid,
    DuplicateCanonicalMethodId,
    MethodTypeInvalid,
    MethodControllerInvalid,
    PublicKeyMultibaseInvalid,
    PublicKeyMultibaseNotCanonical,
    ExplicitAlgorithmRequired,
    AlgorithmUnknown,
    AlgorithmRoleUnsupported,
    MethodTypeAlgorithmMismatch,
    DecodedKeyAlgorithmMismatch,
    DecodedKeyLengthMismatch,
    AuthenticationReferenceInvalid,
    DuplicateAuthenticationReference,
    AuthenticationMethodMissing,
    AuthenticationAlgorithmInvalid,
    KeyAgreementReferenceInvalid,
    DuplicateKeyAgreementReference,
    KeyAgreementMethodMissing,
    KeyAgreementAlgorithmInvalid,
}

fn bounded_nonempty(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    bounded_nonempty(value, DID_MAX_LEN_V2) && value.starts_with("did:mycelix:")
}

fn is_key_agreement_algorithm(algorithm: AlgorithmId) -> bool {
    matches!(algorithm, AlgorithmId::MlKem768 | AlgorithmId::MlKem1024)
}

pub fn canonical_did_method_id_v2(
    document_did: &str,
    method_id: &str,
) -> Result<String, DidDocumentCryptoAdmissionErrorV2> {
    if !valid_did(document_did) || !bounded_nonempty(method_id, METHOD_ID_MAX_LEN_V2) {
        return Err(DidDocumentCryptoAdmissionErrorV2::MethodIdInvalid);
    }

    if let Some(fragment) = method_id.strip_prefix('#') {
        if fragment.is_empty() {
            return Err(DidDocumentCryptoAdmissionErrorV2::MethodIdInvalid);
        }
        return Ok(format!("{document_did}#{fragment}"));
    }

    let prefix = format!("{document_did}#");
    if method_id.starts_with(&prefix) && method_id.len() > prefix.len() {
        return Ok(method_id.to_string());
    }

    Err(DidDocumentCryptoAdmissionErrorV2::MethodIdInvalid)
}

pub fn validate_did_document_crypto_v2(
    document: DidDocumentCryptoAdmissionViewV2<'_>,
) -> Result<ValidatedDidDocumentCryptoV2, DidDocumentCryptoAdmissionErrorV2> {
    if !valid_did(document.did) {
        return Err(DidDocumentCryptoAdmissionErrorV2::DocumentDidInvalid);
    }
    if document.verification_methods.is_empty() {
        return Err(DidDocumentCryptoAdmissionErrorV2::VerificationMethodRequired);
    }
    if document.verification_methods.len() > MAX_VERIFICATION_METHODS_V2 {
        return Err(DidDocumentCryptoAdmissionErrorV2::TooManyVerificationMethods);
    }
    if document.authentication.len() > MAX_AUTHENTICATION_REFERENCES_V2 {
        return Err(DidDocumentCryptoAdmissionErrorV2::TooManyAuthenticationReferences);
    }
    if document.key_agreement.len() > MAX_KEY_AGREEMENT_REFERENCES_V2 {
        return Err(DidDocumentCryptoAdmissionErrorV2::TooManyKeyAgreementReferences);
    }

    let mut methods: BTreeMap<String, AlgorithmId> = BTreeMap::new();

    for method in document.verification_methods {
        let canonical_id = canonical_did_method_id_v2(document.did, method.id)?;
        if methods.contains_key(&canonical_id) {
            return Err(DidDocumentCryptoAdmissionErrorV2::DuplicateCanonicalMethodId);
        }

        if !bounded_nonempty(method.type_, METHOD_TYPE_MAX_LEN_V2) {
            return Err(DidDocumentCryptoAdmissionErrorV2::MethodTypeInvalid);
        }
        if !bounded_nonempty(method.controller, CONTROLLER_MAX_LEN_V2)
            || method.controller != document.did
        {
            return Err(DidDocumentCryptoAdmissionErrorV2::MethodControllerInvalid);
        }
        if !bounded_nonempty(
            method.public_key_multibase,
            PUBLIC_KEY_MULTIBASE_MAX_LEN_V2,
        ) {
            return Err(DidDocumentCryptoAdmissionErrorV2::PublicKeyMultibaseInvalid);
        }

        let declared_u16 = method
            .algorithm
            .ok_or(DidDocumentCryptoAdmissionErrorV2::ExplicitAlgorithmRequired)?;
        let declared = AlgorithmId::from_u16(declared_u16)
            .ok_or(DidDocumentCryptoAdmissionErrorV2::AlgorithmUnknown)?;

        if !declared.is_signature_algorithm() && !is_key_agreement_algorithm(declared) {
            return Err(DidDocumentCryptoAdmissionErrorV2::AlgorithmRoleUnsupported);
        }
        if method.type_ != declared.did_verification_method_type() {
            return Err(DidDocumentCryptoAdmissionErrorV2::MethodTypeAlgorithmMismatch);
        }

        let decoded = TaggedPublicKey::from_multibase(method.public_key_multibase)
            .map_err(|_| DidDocumentCryptoAdmissionErrorV2::PublicKeyMultibaseInvalid)?;
        if decoded.to_multibase() != method.public_key_multibase {
            return Err(DidDocumentCryptoAdmissionErrorV2::PublicKeyMultibaseNotCanonical);
        }
        if decoded.algorithm != declared {
            return Err(DidDocumentCryptoAdmissionErrorV2::DecodedKeyAlgorithmMismatch);
        }
        if decoded.key_bytes.len() != declared.public_key_size() {
            return Err(DidDocumentCryptoAdmissionErrorV2::DecodedKeyLengthMismatch);
        }

        methods.insert(canonical_id, declared);
    }

    let mut authentication_seen: BTreeSet<String> = BTreeSet::new();
    for reference in document.authentication {
        let canonical = canonical_did_method_id_v2(document.did, reference)
            .map_err(|_| DidDocumentCryptoAdmissionErrorV2::AuthenticationReferenceInvalid)?;
        if !authentication_seen.insert(canonical.clone()) {
            return Err(DidDocumentCryptoAdmissionErrorV2::DuplicateAuthenticationReference);
        }
        let algorithm = methods
            .get(&canonical)
            .ok_or(DidDocumentCryptoAdmissionErrorV2::AuthenticationMethodMissing)?;
        if !algorithm.is_signature_algorithm() {
            return Err(DidDocumentCryptoAdmissionErrorV2::AuthenticationAlgorithmInvalid);
        }
    }

    let mut key_agreement_seen: BTreeSet<String> = BTreeSet::new();
    for reference in document.key_agreement {
        let canonical = canonical_did_method_id_v2(document.did, reference)
            .map_err(|_| DidDocumentCryptoAdmissionErrorV2::KeyAgreementReferenceInvalid)?;
        if !key_agreement_seen.insert(canonical.clone()) {
            return Err(DidDocumentCryptoAdmissionErrorV2::DuplicateKeyAgreementReference);
        }
        let algorithm = methods
            .get(&canonical)
            .ok_or(DidDocumentCryptoAdmissionErrorV2::KeyAgreementMethodMissing)?;
        if !is_key_agreement_algorithm(*algorithm) {
            return Err(DidDocumentCryptoAdmissionErrorV2::KeyAgreementAlgorithmInvalid);
        }
    }

    Ok(ValidatedDidDocumentCryptoV2 {
        verification_method_count: document.verification_methods.len(),
        authentication_count: document.authentication.len(),
        key_agreement_count: document.key_agreement.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DID: &str = "did:mycelix:verifier";

    fn key(algorithm: AlgorithmId, fill: u8) -> String {
        TaggedPublicKey::new(algorithm, vec![fill; algorithm.public_key_size()])
            .unwrap()
            .to_multibase()
    }

    fn legacy_raw_ed25519_key(fill: u8) -> String {
        let encoded = bs58::encode(vec![fill; AlgorithmId::Ed25519.public_key_size()])
            .with_alphabet(bs58::Alphabet::BITCOIN)
            .into_string();
        format!("z{encoded}")
    }

    fn method<'a>(
        id: &'a str,
        type_: &'a str,
        key: &'a str,
        algorithm: Option<u16>,
    ) -> DidVerificationMethodAdmissionViewV2<'a> {
        DidVerificationMethodAdmissionViewV2 {
            id,
            type_,
            controller: DID,
            public_key_multibase: key,
            algorithm,
        }
    }

    #[test]
    fn valid_signature_and_kem_relationships_pass() {
        let ed = key(AlgorithmId::Ed25519, 0x11);
        let kem = key(AlgorithmId::MlKem768, 0x22);
        let methods = [
            method(
                "#sign-1",
                AlgorithmId::Ed25519.did_verification_method_type(),
                &ed,
                Some(AlgorithmId::Ed25519.as_u16()),
            ),
            method(
                "#kem-1",
                AlgorithmId::MlKem768.did_verification_method_type(),
                &kem,
                Some(AlgorithmId::MlKem768.as_u16()),
            ),
        ];
        let auth = ["#sign-1"];
        let ka = ["did:mycelix:verifier#kem-1"];

        let result = validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &auth,
            key_agreement: &ka,
        })
        .unwrap();

        assert_eq!(result.verification_method_count, 2);
        assert_eq!(result.authentication_count, 1);
        assert_eq!(result.key_agreement_count, 1);
    }

    #[test]
    fn legacy_raw_ed25519_multibase_is_rejected() {
        let legacy = legacy_raw_ed25519_key(0x12);
        let methods = [method(
            "#sign-1",
            AlgorithmId::Ed25519.did_verification_method_type(),
            &legacy,
            Some(AlgorithmId::Ed25519.as_u16()),
        )];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::PublicKeyMultibaseNotCanonical)
        );
    }

    #[test]
    fn fragment_and_full_method_duplicate_fails_closed() {
        let ed = key(AlgorithmId::Ed25519, 0x33);
        let first = method(
            "#sign-1",
            AlgorithmId::Ed25519.did_verification_method_type(),
            &ed,
            Some(AlgorithmId::Ed25519.as_u16()),
        );
        let second = DidVerificationMethodAdmissionViewV2 {
            id: "did:mycelix:verifier#sign-1",
            ..first
        };
        let methods = [first, second];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::DuplicateCanonicalMethodId)
        );
    }

    #[test]
    fn controller_must_equal_document_did() {
        let ed = key(AlgorithmId::Ed25519, 0x44);
        let methods = [DidVerificationMethodAdmissionViewV2 {
            controller: "did:mycelix:other",
            ..method(
                "#sign-1",
                AlgorithmId::Ed25519.did_verification_method_type(),
                &ed,
                Some(AlgorithmId::Ed25519.as_u16()),
            )
        }];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::MethodControllerInvalid)
        );
    }

    #[test]
    fn explicit_algorithm_and_type_are_required_to_agree() {
        let ed = key(AlgorithmId::Ed25519, 0x55);
        let missing = [method(
            "#sign-1",
            AlgorithmId::Ed25519.did_verification_method_type(),
            &ed,
            None,
        )];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &missing,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::ExplicitAlgorithmRequired)
        );

        let wrong_type = [method(
            "#sign-1",
            AlgorithmId::MlDsa65.did_verification_method_type(),
            &ed,
            Some(AlgorithmId::Ed25519.as_u16()),
        )];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &wrong_type,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::MethodTypeAlgorithmMismatch)
        );
    }

    #[test]
    fn decoded_multibase_algorithm_must_match_declaration() {
        let kem = key(AlgorithmId::MlKem768, 0x66);
        let methods = [method(
            "#sign-1",
            AlgorithmId::Ed25519.did_verification_method_type(),
            &kem,
            Some(AlgorithmId::Ed25519.as_u16()),
        )];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::DecodedKeyAlgorithmMismatch)
        );
    }

    #[test]
    fn authentication_must_reference_signature_method() {
        let kem = key(AlgorithmId::MlKem768, 0x77);
        let methods = [method(
            "#kem-1",
            AlgorithmId::MlKem768.did_verification_method_type(),
            &kem,
            Some(AlgorithmId::MlKem768.as_u16()),
        )];
        let auth = ["#kem-1"];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &auth,
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::AuthenticationAlgorithmInvalid)
        );
    }

    #[test]
    fn key_agreement_must_reference_kem_method() {
        let ed = key(AlgorithmId::Ed25519, 0x88);
        let methods = [method(
            "#sign-1",
            AlgorithmId::Ed25519.did_verification_method_type(),
            &ed,
            Some(AlgorithmId::Ed25519.as_u16()),
        )];
        let ka = ["#sign-1"];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &ka,
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::KeyAgreementAlgorithmInvalid)
        );
    }

    #[test]
    fn symmetric_algorithms_cannot_be_did_public_methods() {
        let symmetric = key(AlgorithmId::XChaCha20Poly1305, 0x00);
        let methods = [method(
            "#sym-1",
            AlgorithmId::XChaCha20Poly1305.did_verification_method_type(),
            &symmetric,
            Some(AlgorithmId::XChaCha20Poly1305.as_u16()),
        )];
        assert_eq!(
            validate_did_document_crypto_v2(DidDocumentCryptoAdmissionViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &[],
                key_agreement: &[],
            }),
            Err(DidDocumentCryptoAdmissionErrorV2::AlgorithmRoleUnsupported)
        );
    }
}
