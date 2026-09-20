// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Canonical, non-authoritative provenance bindings between Mycelix DIDs and
//! external evidence-signing keys.
//!
//! This crate intentionally does not perform cryptographic verification and does
//! not assign trust. It defines the exact semantic bytes that a Mycelix DID
//! controller can publish and an external signer (for example Xenia) can attest.
//! A future Holochain integrity zome can bind DID authorship to the committing
//! agent while an off-chain verifier independently verifies the Xenia signature.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use serde::{Deserialize, Serialize};
use thiserror::Error;

mod xenia_metadata;

pub use xenia_metadata::{
    XeniaAttestationObservation, XeniaBindingMismatch, validate_associated_key_use,
    validate_xenia_attestation_observation,
};

/// Stable schema for [`KeyDidBindingArtifact`].
pub const KEY_DID_BINDING_SCHEMA: &str = "mycelix-key-did-binding-v1";
/// Domain separation used by this crate's canonical bytes.
pub const KEY_DID_BINDING_DOMAIN: &[u8] = b"mycelix:identity:key-did-binding:v1";
/// Xenia generic artifact-attestation domain for these exact canonical bytes.
pub const XENIA_ARTIFACT_DOMAIN: &str = "mycelix-identity-key-binding";
/// Xenia domain-owned artifact schema for these exact canonical bytes.
pub const XENIA_ARTIFACT_SCHEMA: &str = KEY_DID_BINDING_SCHEMA;
/// Fingerprint algorithm used by Xenia evidence public-key bindings.
pub const XENIA_KEY_FINGERPRINT_ALGORITHM: &str = "blake3-256";

/// Maximum UTF-8 byte length for a DID identifier.
pub const MAX_DID_LEN: usize = 512;
/// Maximum UTF-8 byte length for a signature-suite label.
pub const MAX_SIGNATURE_SUITE_LEN: usize = 96;
/// Maximum UTF-8 byte length for an explicit association scope.
pub const MAX_SCOPE_LEN: usize = 256;

/// Declared use of an associated external key.
///
/// These values describe intended provenance use only. They are not permissions,
/// capabilities, trust levels, or governance roles.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum KeyAssociationPurpose {
    /// The key may sign primary evidence-artifact attestations.
    EvidenceAttestor,
    /// The key may countersign evidence artifacts as a witness.
    EvidenceWitness,
    /// The key may sign service-generated provenance artifacts.
    ServiceSigner,
    /// The key may sign research/instrument provenance artifacts.
    ResearchSigner,
}

impl KeyAssociationPurpose {
    /// Stable canonical tag. Do not renumber existing values.
    pub const fn tag(self) -> u8 {
        match self {
            Self::EvidenceAttestor => 1,
            Self::EvidenceWitness => 2,
            Self::ServiceSigner => 3,
            Self::ResearchSigner => 4,
        }
    }
}

/// Canonical association between one Mycelix DID and one Xenia-compatible
/// evidence-signing key fingerprint.
///
/// The same artifact is intended to participate in two independent checks:
///
/// 1. a Mycelix DID controller authors/publishes it;
/// 2. the referenced Xenia key signs its exact canonical bytes.
///
/// Neither side alone establishes the two-sided association. Even both together
/// establish only key↔DID possession/association evidence, not trust or truth.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KeyDidBindingArtifact {
    /// Must equal [`KEY_DID_BINDING_SCHEMA`].
    pub schema: String,
    /// DID claiming association with the external key.
    pub did: String,
    /// Fingerprint algorithm. V1 requires `blake3-256` to match Xenia.
    pub fingerprint_algorithm: String,
    /// Xenia evidence public-key fingerprint.
    pub xenia_key_fingerprint: [u8; 32],
    /// Stable Xenia signature-suite label associated with this key.
    pub xenia_signature_suite: String,
    /// Declared, non-authoritative use of the key.
    pub purpose: KeyAssociationPurpose,
    /// Explicit semantic scope, normally an artifact domain such as
    /// `symthaea-generativity-provenance`.
    pub scope: String,
}

impl KeyDidBindingArtifact {
    /// Construct and validate a v1 binding artifact.
    pub fn new(
        did: impl Into<String>,
        xenia_key_fingerprint: [u8; 32],
        xenia_signature_suite: impl Into<String>,
        purpose: KeyAssociationPurpose,
        scope: impl Into<String>,
    ) -> Result<Self, KeyDidBindingError> {
        let artifact = Self {
            schema: KEY_DID_BINDING_SCHEMA.to_string(),
            did: did.into(),
            fingerprint_algorithm: XENIA_KEY_FINGERPRINT_ALGORITHM.to_string(),
            xenia_key_fingerprint,
            xenia_signature_suite: xenia_signature_suite.into(),
            purpose,
            scope: scope.into(),
        };
        artifact.validate()?;
        Ok(artifact)
    }

    /// Validate the structural and semantic invariants of a persisted binding.
    pub fn validate(&self) -> Result<(), KeyDidBindingError> {
        if self.schema != KEY_DID_BINDING_SCHEMA {
            return Err(KeyDidBindingError::UnsupportedSchema(self.schema.clone()));
        }
        validate_bounded("did", &self.did, MAX_DID_LEN)?;
        if !self.did.starts_with("did:mycelix:") || self.did == "did:mycelix:" {
            return Err(KeyDidBindingError::InvalidDid);
        }
        if self.fingerprint_algorithm != XENIA_KEY_FINGERPRINT_ALGORITHM {
            return Err(KeyDidBindingError::UnsupportedFingerprintAlgorithm(
                self.fingerprint_algorithm.clone(),
            ));
        }
        if self.xenia_key_fingerprint == [0u8; 32] {
            return Err(KeyDidBindingError::EmptyKeyFingerprint);
        }
        validate_bounded(
            "xenia_signature_suite",
            &self.xenia_signature_suite,
            MAX_SIGNATURE_SUITE_LEN,
        )?;
        validate_xenia_signature_suite(&self.xenia_signature_suite)?;
        validate_bounded("scope", &self.scope, MAX_SCOPE_LEN)?;
        if self.scope == "*" {
            return Err(KeyDidBindingError::WildcardScopeForbidden);
        }
        Ok(())
    }

    /// Exact deterministic bytes intended for Xenia generic artifact attestation.
    ///
    /// Variable-length values are length-prefixed and the purpose uses an explicit
    /// stable tag. Serde/JSON ordering is never part of the cryptographic contract.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, KeyDidBindingError> {
        self.validate()?;
        let mut bytes = Vec::new();
        put_bytes(&mut bytes, KEY_DID_BINDING_DOMAIN);
        put_str(&mut bytes, KEY_DID_BINDING_SCHEMA);
        put_str(&mut bytes, &self.did);
        put_str(&mut bytes, XENIA_KEY_FINGERPRINT_ALGORITHM);
        put_bytes(&mut bytes, &self.xenia_key_fingerprint);
        put_str(&mut bytes, &self.xenia_signature_suite);
        bytes.push(self.purpose.tag());
        put_str(&mut bytes, &self.scope);
        Ok(bytes)
    }

    /// Xenia artifact-attestation subject reference for this binding.
    pub fn xenia_subject_ref(&self) -> &str {
        &self.did
    }

    /// Whether this binding's exact scope matches the requested artifact domain.
    ///
    /// This is intentionally exact-match only; v1 has no wildcard or prefix rules.
    pub fn applies_to_scope(&self, artifact_domain: &str) -> bool {
        self.scope == artifact_domain
    }
}

fn validate_xenia_signature_suite(label: &str) -> Result<(), KeyDidBindingError> {
    match label {
        "ed25519-rfc8032"
        | "ml-dsa-65-fips204"
        | "ml-dsa-87-fips204"
        | "slh-dsa-fips205" => Ok(()),
        _ => Err(KeyDidBindingError::UnsupportedXeniaSignatureSuite(
            label.to_string(),
        )),
    }
}

fn validate_bounded(
    field: &'static str,
    value: &str,
    max: usize,
) -> Result<(), KeyDidBindingError> {
    if value.trim().is_empty() {
        return Err(KeyDidBindingError::EmptyField(field));
    }
    if value.len() > max {
        return Err(KeyDidBindingError::FieldTooLong {
            field,
            max,
            found: value.len(),
        });
    }
    Ok(())
}

fn put_str(bytes: &mut Vec<u8>, value: &str) {
    put_bytes(bytes, value.as_bytes());
}

fn put_bytes(bytes: &mut Vec<u8>, value: &[u8]) {
    bytes.extend_from_slice(&(value.len() as u64).to_be_bytes());
    bytes.extend_from_slice(value);
}

/// Validation errors for DID↔Xenia key association artifacts.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum KeyDidBindingError {
    /// Persisted schema is not v1.
    #[error("unsupported key-DID binding schema: {0}")]
    UnsupportedSchema(String),
    /// A required field was empty.
    #[error("{0} must not be empty")]
    EmptyField(&'static str),
    /// A bounded field exceeded its v1 limit.
    #[error("{field} exceeds maximum length {max}: found {found}")]
    FieldTooLong {
        /// Field name.
        field: &'static str,
        /// Maximum UTF-8 byte length.
        max: usize,
        /// Observed UTF-8 byte length.
        found: usize,
    },
    /// DID is not a non-empty `did:mycelix:` identifier.
    #[error("binding DID must be a non-empty did:mycelix identifier")]
    InvalidDid,
    /// Fingerprint algorithm is unsupported by v1.
    #[error("unsupported Xenia key fingerprint algorithm: {0}")]
    UnsupportedFingerprintAlgorithm(String),
    /// All-zero public-key fingerprint is rejected.
    #[error("Xenia key fingerprint must not be all zero")]
    EmptyKeyFingerprint,
    /// Signature-suite label is not part of Xenia's current evidence suite registry.
    #[error("unsupported Xenia signature-suite label: {0}")]
    UnsupportedXeniaSignatureSuite(String),
    /// V1 requires explicit scopes and therefore rejects `*`.
    #[error("wildcard scope is forbidden; bind the key to an explicit semantic scope")]
    WildcardScopeForbidden,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binding() -> KeyDidBindingArtifact {
        KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [7u8; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap()
    }

    #[test]
    fn canonical_bytes_are_stable_across_serde_round_trip() {
        let artifact = binding();
        let before = artifact.canonical_bytes().unwrap();
        let encoded = serde_json::to_string(&artifact).unwrap();
        let decoded: KeyDidBindingArtifact = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded.canonical_bytes().unwrap(), before);
    }

    #[test]
    fn fingerprint_changes_canonical_identity() {
        let first = binding();
        let mut second = first.clone();
        second.xenia_key_fingerprint = [8u8; 32];
        assert_ne!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }

    #[test]
    fn did_changes_canonical_identity() {
        let first = binding();
        let mut second = first.clone();
        second.did = "did:mycelix:another-agent".into();
        assert_ne!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }

    #[test]
    fn scope_changes_canonical_identity() {
        let first = binding();
        let mut second = first.clone();
        second.scope = "mycelix-lineage-provenance".into();
        assert_ne!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }

    #[test]
    fn purpose_changes_canonical_identity() {
        let first = binding();
        let mut second = first.clone();
        second.purpose = KeyAssociationPurpose::EvidenceWitness;
        assert_ne!(first.canonical_bytes().unwrap(), second.canonical_bytes().unwrap());
    }

    #[test]
    fn wildcard_scope_is_rejected() {
        let err = KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [7u8; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "*",
        )
        .unwrap_err();
        assert_eq!(err, KeyDidBindingError::WildcardScopeForbidden);
    }

    #[test]
    fn unknown_suite_is_rejected() {
        let err = KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [7u8; 32],
            "future-magic-signature",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap_err();
        assert!(matches!(
            err,
            KeyDidBindingError::UnsupportedXeniaSignatureSuite(_)
        ));
    }

    #[test]
    fn all_zero_fingerprint_is_rejected() {
        let err = KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [0u8; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap_err();
        assert_eq!(err, KeyDidBindingError::EmptyKeyFingerprint);
    }

    #[test]
    fn scope_matching_is_exact() {
        let artifact = binding();
        assert!(artifact.applies_to_scope("symthaea-generativity-provenance"));
        assert!(!artifact.applies_to_scope("symthaea-generativity"));
        assert!(!artifact.applies_to_scope("mycelix-governance"));
    }
}
