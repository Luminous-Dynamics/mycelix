// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Policy-neutral admitted DID authentication-method material theorem for Identity V2.
//!
//! This crate resolves one exact canonical authentication method from one exact DID
//! document only after the complete document passes the #272 cryptographic admission
//! theorem. It owns no verifier policy, signature-scheme interpretation, generation
//! lineage, network observation, currentness, clock, or signature-authenticity logic.
//!
//! The result is deliberately owned and opaque so downstream generation assembly can
//! consume exact admitted key material without accepting a caller-constructed key DTO.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use mycelix_did_document_crypto_policy::{
    canonical_did_method_id_v2, validate_did_document_crypto_v2,
    DidDocumentCryptoAdmissionErrorV2, DidDocumentCryptoAdmissionViewV2,
    DidVerificationMethodAdmissionViewV2,
};

/// Opaque owned material for one exact authentication method from one exact admitted DID
/// document. Possession proves only document-local cryptographic admission + exact method
/// selection. It does not prove action provenance, generation identity, currentness,
/// revocation state, verifier-policy compatibility, or signature authenticity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AdmittedDidAuthenticationMethodV2 {
    did: String,
    canonical_key_id: String,
    algorithm: AlgorithmId,
    public_key_bytes: Vec<u8>,
    public_key_multibase: String,
}

impl AdmittedDidAuthenticationMethodV2 {
    pub fn did(&self) -> &str {
        &self.did
    }

    pub fn canonical_key_id(&self) -> &str {
        &self.canonical_key_id
    }

    pub fn algorithm(&self) -> AlgorithmId {
        self.algorithm
    }

    pub fn public_key_bytes(&self) -> &[u8] {
        &self.public_key_bytes
    }

    pub fn public_key_multibase(&self) -> &str {
        &self.public_key_multibase
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DidAuthenticationMethodMaterialErrorV2 {
    Admission(DidDocumentCryptoAdmissionErrorV2),
    TargetKeyIdInvalid,
    TargetKeyIdNotCanonical,
    TargetMethodMissing,
    TargetAuthenticationReferenceMissing,
    AdmittedDocumentInvariantBroken,
}

/// Resolve one exact policy-neutral authentication method from one exact admitted DID
/// document.
///
/// The whole document is admitted first. The target key ID must then already be the
/// canonical full DID URL, must select exactly one admitted verification method, and must
/// be referenced by the document's `authentication` relationship. The canonical multibase
/// is decoded again only to materialize the exact owned raw bytes carried by the result.
pub fn resolve_admitted_did_authentication_method_v2(
    document: DidDocumentCryptoAdmissionViewV2<'_>,
    target_key_id: &str,
) -> Result<AdmittedDidAuthenticationMethodV2, DidAuthenticationMethodMaterialErrorV2> {
    validate_did_document_crypto_v2(document)
        .map_err(DidAuthenticationMethodMaterialErrorV2::Admission)?;

    let canonical_target = canonical_did_method_id_v2(document.did, target_key_id)
        .map_err(|_| DidAuthenticationMethodMaterialErrorV2::TargetKeyIdInvalid)?;
    if canonical_target != target_key_id {
        return Err(DidAuthenticationMethodMaterialErrorV2::TargetKeyIdNotCanonical);
    }

    let mut selected: Option<DidVerificationMethodAdmissionViewV2<'_>> = None;
    for method in document.verification_methods {
        let canonical = canonical_did_method_id_v2(document.did, method.id)
            .map_err(|_| DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken)?;
        if canonical == canonical_target {
            if selected.replace(*method).is_some() {
                return Err(
                    DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken,
                );
            }
        }
    }
    let selected = selected.ok_or(DidAuthenticationMethodMaterialErrorV2::TargetMethodMissing)?;

    let mut authentication_reference_found = false;
    for reference in document.authentication {
        let canonical = canonical_did_method_id_v2(document.did, reference)
            .map_err(|_| DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken)?;
        if canonical == canonical_target {
            authentication_reference_found = true;
            break;
        }
    }
    if !authentication_reference_found {
        return Err(
            DidAuthenticationMethodMaterialErrorV2::TargetAuthenticationReferenceMissing,
        );
    }

    let declared_u16 = selected
        .algorithm
        .ok_or(DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken)?;
    let declared = AlgorithmId::from_u16(declared_u16)
        .ok_or(DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken)?;
    if !declared.is_signature_algorithm() {
        return Err(DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken);
    }

    let decoded = TaggedPublicKey::from_multibase(selected.public_key_multibase)
        .map_err(|_| DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken)?;
    if decoded.to_multibase() != selected.public_key_multibase
        || decoded.algorithm != declared
        || decoded.key_bytes.len() != declared.public_key_size()
    {
        return Err(DidAuthenticationMethodMaterialErrorV2::AdmittedDocumentInvariantBroken);
    }

    Ok(AdmittedDidAuthenticationMethodV2 {
        did: document.did.to_string(),
        canonical_key_id: canonical_target,
        algorithm: declared,
        public_key_bytes: decoded.key_bytes,
        public_key_multibase: selected.public_key_multibase.to_string(),
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

    fn method<'a>(
        id: &'a str,
        algorithm: AlgorithmId,
        public_key_multibase: &'a str,
    ) -> DidVerificationMethodAdmissionViewV2<'a> {
        DidVerificationMethodAdmissionViewV2 {
            id,
            type_: algorithm.did_verification_method_type(),
            controller: DID,
            public_key_multibase,
            algorithm: Some(algorithm.as_u16()),
        }
    }

    #[test]
    fn canonical_target_resolves_exact_authenticated_material() {
        let first = key(AlgorithmId::Ed25519, 0x31);
        let second = key(AlgorithmId::MlDsa65, 0x52);
        let methods = [
            method("#key-1", AlgorithmId::Ed25519, &first),
            method("#key-2", AlgorithmId::MlDsa65, &second),
        ];
        let authentication = ["#key-1", "did:mycelix:verifier#key-2"];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &authentication,
            key_agreement: &[],
        };

        let resolved = resolve_admitted_did_authentication_method_v2(
            document,
            "did:mycelix:verifier#key-2",
        )
        .unwrap();

        let expected = vec![0x52; AlgorithmId::MlDsa65.public_key_size()];
        assert_eq!(resolved.did(), DID);
        assert_eq!(resolved.canonical_key_id(), "did:mycelix:verifier#key-2");
        assert_eq!(resolved.algorithm(), AlgorithmId::MlDsa65);
        assert_eq!(resolved.public_key_bytes(), expected.as_slice());
        assert_eq!(resolved.public_key_multibase(), second);
    }

    #[test]
    fn target_key_id_must_already_be_canonical() {
        let ed = key(AlgorithmId::Ed25519, 0x11);
        let methods = [method("#key-1", AlgorithmId::Ed25519, &ed)];
        let authentication = ["#key-1"];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &authentication,
            key_agreement: &[],
        };

        assert_eq!(
            resolve_admitted_did_authentication_method_v2(document, "#key-1"),
            Err(DidAuthenticationMethodMaterialErrorV2::TargetKeyIdNotCanonical)
        );
    }

    #[test]
    fn target_must_be_explicit_authentication_method() {
        let ed = key(AlgorithmId::Ed25519, 0x22);
        let methods = [method("#key-1", AlgorithmId::Ed25519, &ed)];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &[],
            key_agreement: &[],
        };

        assert_eq!(
            resolve_admitted_did_authentication_method_v2(
                document,
                "did:mycelix:verifier#key-1",
            ),
            Err(
                DidAuthenticationMethodMaterialErrorV2::TargetAuthenticationReferenceMissing
            )
        );
    }

    #[test]
    fn key_agreement_method_cannot_substitute_for_authentication() {
        let kem = key(AlgorithmId::MlKem768, 0x44);
        let methods = [method("#kem-1", AlgorithmId::MlKem768, &kem)];
        let key_agreement = ["#kem-1"];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &[],
            key_agreement: &key_agreement,
        };

        assert_eq!(
            resolve_admitted_did_authentication_method_v2(
                document,
                "did:mycelix:verifier#kem-1",
            ),
            Err(
                DidAuthenticationMethodMaterialErrorV2::TargetAuthenticationReferenceMissing
            )
        );
    }

    #[test]
    fn complete_document_admission_precedes_target_selection() {
        let ed = key(AlgorithmId::Ed25519, 0x55);
        let first = method("#key-1", AlgorithmId::Ed25519, &ed);
        let second = DidVerificationMethodAdmissionViewV2 {
            id: "did:mycelix:verifier#key-1",
            ..first
        };
        let methods = [first, second];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &[],
            key_agreement: &[],
        };

        assert_eq!(
            resolve_admitted_did_authentication_method_v2(
                document,
                "did:mycelix:verifier#missing",
            ),
            Err(DidAuthenticationMethodMaterialErrorV2::Admission(
                DidDocumentCryptoAdmissionErrorV2::DuplicateCanonicalMethodId,
            ))
        );
    }

    #[test]
    fn absent_canonical_target_fails_closed() {
        let ed = key(AlgorithmId::Ed25519, 0x66);
        let methods = [method("#key-1", AlgorithmId::Ed25519, &ed)];
        let authentication = ["#key-1"];
        let document = DidDocumentCryptoAdmissionViewV2 {
            did: DID,
            verification_methods: &methods,
            authentication: &authentication,
            key_agreement: &[],
        };

        assert_eq!(
            resolve_admitted_did_authentication_method_v2(
                document,
                "did:mycelix:verifier#missing",
            ),
            Err(DidAuthenticationMethodMaterialErrorV2::TargetMethodMissing)
        );
    }

    #[test]
    fn admitted_method_capability_fields_are_verifier_owned() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct AdmittedDidAuthenticationMethodV2")
            .unwrap();
        let end = source[start..]
            .index("impl AdmittedDidAuthenticationMethodV2")
            .unwrap()
            + start;
        let body = &source[start..end];
        for public_field in [
            "pub did:",
            "pub canonical_key_id:",
            "pub algorithm:",
            "pub public_key_bytes:",
            "pub public_key_multibase:",
        ] {
            assert!(!body.contains(public_field));
        }
    }
}
