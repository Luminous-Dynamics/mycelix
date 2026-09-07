// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Strict DID verification-method theorem for Identity V2 verifier keys.
//!
//! This crate owns only **document-structural** key truth. It does not resolve DID
//! branches, deactivation, key rotation/currentness, or verify signatures.
//!
//! A successful result means one verification method in one supplied DID document:
//!
//! - has one canonical DID URL;
//! - is uniquely identified within that document;
//! - is controlled by the document DID;
//! - is explicitly referenced by the document's authentication relationship;
//! - declares one recognized signature algorithm;
//! - uses the exact DID method type for that algorithm;
//! - decodes to the same algorithm and exact raw key length;
//! - matches the exact verifier DID/key/signature scheme frozen by #248/#250.
//!
//! #244 still owns branch-aware DID/key lineage and deactivation/currentness.

#![forbid(unsafe_code)]

use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use mycelix_kvector_signature_material_policy::expected_algorithm_for_signature_scheme_v2;
use mycelix_kvector_verifier_policy_body::{
    validate_kvector_verifier_policy_body_v2, KVectorVerifierPolicyBodyV2,
};

pub const DID_MAX_LEN_V2: usize = 256;
pub const VERIFICATION_METHOD_ID_MAX_LEN_V2: usize = 256;
pub const VERIFICATION_METHOD_TYPE_MAX_LEN_V2: usize = 256;
pub const VERIFICATION_METHOD_CONTROLLER_MAX_LEN_V2: usize = 256;
pub const VERIFICATION_METHOD_MULTIBASE_MAX_LEN_V2: usize = 4096;

#[derive(Debug, Clone, Copy)]
pub struct DidVerificationMethodViewV2<'a> {
    pub id: &'a str,
    pub type_: &'a str,
    pub controller: &'a str,
    pub public_key_multibase: &'a str,
    pub algorithm: Option<u16>,
}

#[derive(Debug, Clone, Copy)]
pub struct DidVerificationDocumentViewV2<'a> {
    pub did: &'a str,
    pub verification_methods: &'a [DidVerificationMethodViewV2<'a>],
    pub authentication: &'a [&'a str],
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyBoundVerifierKeyV2 {
    pub canonical_key_id: String,
    pub public_key: TaggedPublicKey,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerifierKeyMethodErrorV2 {
    PolicyBodyInvalid,
    DocumentDidInvalid,
    PolicyVerifierDidMismatch,
    PolicyVerifierKeyIdNotCanonical,
    MethodIdInvalid,
    DuplicateCanonicalMethodId,
    PolicyVerifierMethodMissing,
    MethodTypeInvalid,
    MethodControllerInvalid,
    PublicKeyMultibaseInvalid,
    ExplicitAlgorithmRequired,
    AlgorithmUnknown,
    AlgorithmNotSignature,
    PolicySignatureSchemeUnsupported,
    PolicyAlgorithmMismatch,
    MethodTypeAlgorithmMismatch,
    DecodedKeyAlgorithmMismatch,
    DecodedKeyLengthMismatch,
    AuthenticationReferenceInvalid,
    PolicyVerifierKeyNotAuthenticated,
}

fn bounded_nonempty(value: &str, max_len: usize) -> bool {
    !value.is_empty() && value.len() <= max_len
}

fn valid_did(value: &str) -> bool {
    bounded_nonempty(value, DID_MAX_LEN_V2) && value.starts_with("did:")
}

pub fn canonical_verification_method_id_v2(
    document_did: &str,
    method_id: &str,
) -> Result<String, VerifierKeyMethodErrorV2> {
    if !valid_did(document_did)
        || !bounded_nonempty(method_id, VERIFICATION_METHOD_ID_MAX_LEN_V2)
    {
        return Err(VerifierKeyMethodErrorV2::MethodIdInvalid);
    }

    if let Some(fragment) = method_id.strip_prefix('#') {
        if fragment.is_empty() {
            return Err(VerifierKeyMethodErrorV2::MethodIdInvalid);
        }
        return Ok(format!("{document_did}#{fragment}"));
    }

    let prefix = format!("{document_did}#");
    if method_id.starts_with(&prefix) && method_id.len() > prefix.len() {
        return Ok(method_id.to_string());
    }

    Err(VerifierKeyMethodErrorV2::MethodIdInvalid)
}

fn validate_selected_method_v2(
    document_did: &str,
    method: DidVerificationMethodViewV2<'_>,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<TaggedPublicKey, VerifierKeyMethodErrorV2> {
    if !bounded_nonempty(method.type_, VERIFICATION_METHOD_TYPE_MAX_LEN_V2) {
        return Err(VerifierKeyMethodErrorV2::MethodTypeInvalid);
    }
    if !bounded_nonempty(
        method.controller,
        VERIFICATION_METHOD_CONTROLLER_MAX_LEN_V2,
    ) || method.controller != document_did
    {
        return Err(VerifierKeyMethodErrorV2::MethodControllerInvalid);
    }
    if !bounded_nonempty(
        method.public_key_multibase,
        VERIFICATION_METHOD_MULTIBASE_MAX_LEN_V2,
    ) {
        return Err(VerifierKeyMethodErrorV2::PublicKeyMultibaseInvalid);
    }

    let declared_u16 = method
        .algorithm
        .ok_or(VerifierKeyMethodErrorV2::ExplicitAlgorithmRequired)?;
    let declared = AlgorithmId::from_u16(declared_u16)
        .ok_or(VerifierKeyMethodErrorV2::AlgorithmUnknown)?;
    if !declared.is_signature_algorithm() {
        return Err(VerifierKeyMethodErrorV2::AlgorithmNotSignature);
    }

    let expected = expected_algorithm_for_signature_scheme_v2(policy.signature_scheme_id)
        .map_err(|_| VerifierKeyMethodErrorV2::PolicySignatureSchemeUnsupported)?;
    if declared != expected {
        return Err(VerifierKeyMethodErrorV2::PolicyAlgorithmMismatch);
    }

    if method.type_ != declared.did_verification_method_type() {
        return Err(VerifierKeyMethodErrorV2::MethodTypeAlgorithmMismatch);
    }

    let decoded = TaggedPublicKey::from_multibase(method.public_key_multibase)
        .map_err(|_| VerifierKeyMethodErrorV2::PublicKeyMultibaseInvalid)?;
    if decoded.algorithm != declared {
        return Err(VerifierKeyMethodErrorV2::DecodedKeyAlgorithmMismatch);
    }
    if decoded.key_bytes.len() != declared.public_key_size() {
        return Err(VerifierKeyMethodErrorV2::DecodedKeyLengthMismatch);
    }

    Ok(decoded)
}

/// Resolve one policy-selected verification method from a supplied DID document.
///
/// This function is intentionally branch-agnostic. The caller must not treat a
/// successful result as current key evidence until #244's network lineage/deactivation
/// resolver establishes which DID-document state is observed and non-conflicting.
pub fn resolve_policy_verifier_key_from_document_v2(
    document: DidVerificationDocumentViewV2<'_>,
    policy: KVectorVerifierPolicyBodyV2<'_>,
) -> Result<PolicyBoundVerifierKeyV2, VerifierKeyMethodErrorV2> {
    validate_kvector_verifier_policy_body_v2(policy)
        .map_err(|_| VerifierKeyMethodErrorV2::PolicyBodyInvalid)?;

    if !valid_did(document.did) {
        return Err(VerifierKeyMethodErrorV2::DocumentDidInvalid);
    }
    if document.did != policy.verifier_did {
        return Err(VerifierKeyMethodErrorV2::PolicyVerifierDidMismatch);
    }

    let policy_key_id = canonical_verification_method_id_v2(document.did, policy.verifier_key_id)
        .map_err(|_| VerifierKeyMethodErrorV2::PolicyVerifierKeyIdNotCanonical)?;
    if policy_key_id != policy.verifier_key_id {
        return Err(VerifierKeyMethodErrorV2::PolicyVerifierKeyIdNotCanonical);
    }

    let mut canonical_ids = Vec::with_capacity(document.verification_methods.len());
    let mut selected: Option<DidVerificationMethodViewV2<'_>> = None;

    for method in document.verification_methods {
        let canonical = canonical_verification_method_id_v2(document.did, method.id)?;
        if canonical_ids.contains(&canonical) {
            return Err(VerifierKeyMethodErrorV2::DuplicateCanonicalMethodId);
        }
        if canonical == policy_key_id {
            selected = Some(*method);
        }
        canonical_ids.push(canonical);
    }

    let selected = selected.ok_or(VerifierKeyMethodErrorV2::PolicyVerifierMethodMissing)?;

    let mut authenticated = false;
    for reference in document.authentication {
        let canonical = canonical_verification_method_id_v2(document.did, reference)
            .map_err(|_| VerifierKeyMethodErrorV2::AuthenticationReferenceInvalid)?;
        if canonical == policy_key_id {
            authenticated = true;
        }
    }
    if !authenticated {
        return Err(VerifierKeyMethodErrorV2::PolicyVerifierKeyNotAuthenticated);
    }

    let public_key = validate_selected_method_v2(document.did, selected, policy)?;
    Ok(PolicyBoundVerifierKeyV2 {
        canonical_key_id: policy_key_id,
        public_key,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DID: &str = "did:mycelix:verifier";
    const KEY_ID: &str = "did:mycelix:verifier#hybrid-1";

    fn policy() -> KVectorVerifierPolicyBodyV2<'static> {
        KVectorVerifierPolicyBodyV2 {
            policy_id: "policy:kvector-prod-v2",
            policy_version: "2.0.0",
            backend_id: "winterfell-v2",
            circuit_id: "mycelix-kvector-range-v2",
            circuit_version: "2.0.0",
            verifier_did: DID,
            verifier_key_id: KEY_ID,
            signature_scheme_id: "hybrid-ed25519-mldsa65-v1",
            valid_from_micros: 1_000_000,
            valid_until_micros: 9_000_000,
            max_record_lifetime_micros: 2_000_000,
        }
    }

    fn hybrid_multibase() -> String {
        TaggedPublicKey::new(
            AlgorithmId::HybridEd25519MlDsa65,
            vec![0x42; AlgorithmId::HybridEd25519MlDsa65.public_key_size()],
        )
        .unwrap()
        .to_multibase()
    }

    fn method<'a>(key: &'a str) -> DidVerificationMethodViewV2<'a> {
        DidVerificationMethodViewV2 {
            id: "#hybrid-1",
            type_: AlgorithmId::HybridEd25519MlDsa65.did_verification_method_type(),
            controller: DID,
            public_key_multibase: key,
            algorithm: Some(AlgorithmId::HybridEd25519MlDsa65.as_u16()),
        }
    }

    #[test]
    fn fragment_and_full_ids_canonicalize_identically() {
        assert_eq!(
            canonical_verification_method_id_v2(DID, "#hybrid-1").unwrap(),
            KEY_ID
        );
        assert_eq!(
            canonical_verification_method_id_v2(DID, KEY_ID).unwrap(),
            KEY_ID
        );
        assert_eq!(
            canonical_verification_method_id_v2(DID, "did:mycelix:other#hybrid-1"),
            Err(VerifierKeyMethodErrorV2::MethodIdInvalid)
        );
        assert_eq!(
            canonical_verification_method_id_v2(DID, "#"),
            Err(VerifierKeyMethodErrorV2::MethodIdInvalid)
        );
    }

    #[test]
    fn exact_policy_key_is_structurally_resolved() {
        let key = hybrid_multibase();
        let methods = [method(&key)];
        let auth = ["#hybrid-1"];
        let resolved = resolve_policy_verifier_key_from_document_v2(
            DidVerificationDocumentViewV2 {
                did: DID,
                verification_methods: &methods,
                authentication: &auth,
            },
            policy(),
        )
        .unwrap();
        assert_eq!(resolved.canonical_key_id, KEY_ID);
        assert_eq!(
            resolved.public_key.algorithm,
            AlgorithmId::HybridEd25519MlDsa65
        );
        assert_eq!(
            resolved.public_key.key_bytes.len(),
            AlgorithmId::HybridEd25519MlDsa65.public_key_size()
        );
    }

    #[test]
    fn fragment_full_duplicate_is_rejected() {
        let key = hybrid_multibase();
        let first = method(&key);
        let second = DidVerificationMethodViewV2 {
            id: KEY_ID,
            ..first
        };
        let methods = [first, second];
        let auth = ["#hybrid-1"];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::DuplicateCanonicalMethodId)
        );
    }

    #[test]
    fn controller_and_authentication_are_authority_relevant() {
        let key = hybrid_multibase();
        let wrong_controller = DidVerificationMethodViewV2 {
            controller: "did:mycelix:other",
            ..method(&key)
        };
        let methods = [wrong_controller];
        let auth = ["#hybrid-1"];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::MethodControllerInvalid)
        );

        let methods = [method(&key)];
        let no_auth: [&str; 0] = [];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &no_auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::PolicyVerifierKeyNotAuthenticated)
        );
    }

    #[test]
    fn declared_type_and_decoded_algorithm_must_agree() {
        let key = hybrid_multibase();
        let wrong_type = DidVerificationMethodViewV2 {
            type_: AlgorithmId::Ed25519.did_verification_method_type(),
            ..method(&key)
        };
        let methods = [wrong_type];
        let auth = ["#hybrid-1"];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::MethodTypeAlgorithmMismatch)
        );

        let ed_key = TaggedPublicKey::new(
            AlgorithmId::Ed25519,
            vec![0x24; AlgorithmId::Ed25519.public_key_size()],
        )
        .unwrap()
        .to_multibase();
        let decoded_mismatch = DidVerificationMethodViewV2 {
            public_key_multibase: &ed_key,
            ..method(&ed_key)
        };
        let methods = [decoded_mismatch];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::DecodedKeyAlgorithmMismatch)
        );
    }

    #[test]
    fn missing_or_policy_mismatched_algorithm_fails_closed() {
        let key = hybrid_multibase();
        let missing = DidVerificationMethodViewV2 {
            algorithm: None,
            ..method(&key)
        };
        let methods = [missing];
        let auth = ["#hybrid-1"];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::ExplicitAlgorithmRequired)
        );

        let ed_declared = DidVerificationMethodViewV2 {
            type_: AlgorithmId::Ed25519.did_verification_method_type(),
            algorithm: Some(AlgorithmId::Ed25519.as_u16()),
            ..method(&key)
        };
        let methods = [ed_declared];
        assert_eq!(
            resolve_policy_verifier_key_from_document_v2(
                DidVerificationDocumentViewV2 {
                    did: DID,
                    verification_methods: &methods,
                    authentication: &auth,
                },
                policy(),
            ),
            Err(VerifierKeyMethodErrorV2::PolicyAlgorithmMismatch)
        );
    }
}