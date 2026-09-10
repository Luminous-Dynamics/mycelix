// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical lifecycle artifacts authored cryptographically by the external
//! Xenia-compatible key side of a DID↔key association.
//!
//! DID-side Holochain lifecycle and key-side cryptographic lifecycle are separate
//! evidence planes. Neither can silently overwrite the other. Consumers should
//! preserve disagreement between them as evidence rather than applying last-write-wins.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{KeyDidBindingArtifact, XeniaAttestationObservation};

/// Stable schema for [`KeySideBindingLifecycleArtifact`].
pub const KEY_SIDE_LIFECYCLE_SCHEMA: &str = "mycelix-key-side-binding-lifecycle-v1";
/// Canonical byte-domain separator for key-side lifecycle artifacts.
pub const KEY_SIDE_LIFECYCLE_DOMAIN: &[u8] = b"mycelix:identity:key-side-binding-lifecycle:v1";
/// Xenia generic artifact-attestation domain for lifecycle artifacts.
pub const XENIA_KEY_LIFECYCLE_ARTIFACT_DOMAIN: &str = "mycelix-identity-key-lifecycle";
/// Domain-owned schema supplied to Xenia artifact attestation.
pub const XENIA_KEY_LIFECYCLE_ARTIFACT_SCHEMA: &str = KEY_SIDE_LIFECYCLE_SCHEMA;
/// Maximum UTF-8 byte length for a lifecycle explanation.
pub const MAX_KEY_SIDE_LIFECYCLE_REASON_LEN: usize = 512;

/// Action taken by the external key over its side of a DID association.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum KeySideLifecycleDisposition {
    /// The referenced key disavows this association for future use.
    Disavow,
    /// The old key provides continuity evidence toward a replacement binding.
    ///
    /// The replacement is **not** activated by this statement alone. The replacement
    /// key still needs its own Xenia possession proof and the DID controller still
    /// needs to publish/accept the replacement on the Mycelix side.
    Supersede,
}

impl KeySideLifecycleDisposition {
    const fn tag(self) -> u8 {
        match self {
            Self::Disavow => 1,
            Self::Supersede => 2,
        }
    }
}

/// Self-contained lifecycle statement to be signed by the key referenced in
/// `target` through Xenia's generic evidence-artifact attestation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct KeySideBindingLifecycleArtifact {
    /// Must equal [`KEY_SIDE_LIFECYCLE_SCHEMA`].
    pub schema: String,
    /// Exact association being disavowed or superseded.
    pub target: KeyDidBindingArtifact,
    /// Lifecycle operation.
    pub disposition: KeySideLifecycleDisposition,
    /// Proposed successor association for `Supersede`.
    pub replacement: Option<KeyDidBindingArtifact>,
    /// Optional bounded explanation; descriptive only.
    pub reason: Option<String>,
}

impl KeySideBindingLifecycleArtifact {
    /// Construct and validate a key-side lifecycle artifact.
    pub fn new(
        target: KeyDidBindingArtifact,
        disposition: KeySideLifecycleDisposition,
        replacement: Option<KeyDidBindingArtifact>,
        reason: Option<String>,
    ) -> Result<Self, KeySideLifecycleError> {
        let artifact = Self {
            schema: KEY_SIDE_LIFECYCLE_SCHEMA.to_string(),
            target,
            disposition,
            replacement,
            reason,
        };
        artifact.validate()?;
        Ok(artifact)
    }

    /// Validate persisted lifecycle structure without claiming a signature was verified.
    pub fn validate(&self) -> Result<(), KeySideLifecycleError> {
        if self.schema != KEY_SIDE_LIFECYCLE_SCHEMA {
            return Err(KeySideLifecycleError::UnsupportedSchema(self.schema.clone()));
        }
        self.target
            .validate()
            .map_err(|error| KeySideLifecycleError::InvalidTarget(error.to_string()))?;

        if let Some(reason) = &self.reason {
            if reason.len() > MAX_KEY_SIDE_LIFECYCLE_REASON_LEN {
                return Err(KeySideLifecycleError::ReasonTooLong {
                    max: MAX_KEY_SIDE_LIFECYCLE_REASON_LEN,
                    found: reason.len(),
                });
            }
            if reason.trim() != reason {
                return Err(KeySideLifecycleError::NonCanonicalReason);
            }
        }

        match (self.disposition, &self.replacement) {
            (KeySideLifecycleDisposition::Disavow, None) => Ok(()),
            (KeySideLifecycleDisposition::Disavow, Some(_)) => {
                Err(KeySideLifecycleError::DisavowHasReplacement)
            }
            (KeySideLifecycleDisposition::Supersede, None) => {
                Err(KeySideLifecycleError::SupersedeMissingReplacement)
            }
            (KeySideLifecycleDisposition::Supersede, Some(replacement)) => {
                replacement
                    .validate()
                    .map_err(|error| KeySideLifecycleError::InvalidReplacement(error.to_string()))?;
                if replacement.did != self.target.did {
                    return Err(KeySideLifecycleError::ReplacementDidMismatch);
                }
                if replacement.purpose != self.target.purpose {
                    return Err(KeySideLifecycleError::ReplacementPurposeMismatch);
                }
                if replacement.scope != self.target.scope {
                    return Err(KeySideLifecycleError::ReplacementScopeMismatch);
                }
                if replacement == &self.target {
                    return Err(KeySideLifecycleError::NoOpSupersession);
                }
                Ok(())
            }
        }
    }

    /// Exact deterministic bytes to authenticate with Xenia.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, KeySideLifecycleError> {
        self.validate()?;
        let target = self
            .target
            .canonical_bytes()
            .map_err(|error| KeySideLifecycleError::InvalidTarget(error.to_string()))?;

        let mut bytes = Vec::new();
        put_bytes(&mut bytes, KEY_SIDE_LIFECYCLE_DOMAIN);
        put_str(&mut bytes, KEY_SIDE_LIFECYCLE_SCHEMA);
        put_bytes(&mut bytes, &target);
        bytes.push(self.disposition.tag());

        match &self.replacement {
            None => bytes.push(0),
            Some(replacement) => {
                bytes.push(1);
                let replacement = replacement.canonical_bytes().map_err(|error| {
                    KeySideLifecycleError::InvalidReplacement(error.to_string())
                })?;
                put_bytes(&mut bytes, &replacement);
            }
        }

        match &self.reason {
            None => bytes.push(0),
            Some(reason) => {
                bytes.push(1);
                put_str(&mut bytes, reason);
            }
        }
        Ok(bytes)
    }

    /// Xenia artifact-attestation subject reference for this lifecycle artifact.
    pub fn xenia_subject_ref(&self) -> &str {
        &self.target.did
    }
}

/// Require metadata from an independently verified Xenia attestation to prove that
/// the **target key itself** signed this key-side lifecycle artifact.
///
/// This function does not perform cryptography. It only fail-closes the semantic
/// handoff after a caller has independently run the Xenia verifier.
pub fn validate_key_side_lifecycle_xenia_observation(
    lifecycle: &KeySideBindingLifecycleArtifact,
    observation: XeniaAttestationObservation<'_>,
) -> Result<(), KeySideLifecycleError> {
    lifecycle.validate()?;
    if observation.artifact_domain != XENIA_KEY_LIFECYCLE_ARTIFACT_DOMAIN {
        return Err(KeySideLifecycleError::XeniaArtifactDomainMismatch);
    }
    if observation.artifact_schema != XENIA_KEY_LIFECYCLE_ARTIFACT_SCHEMA {
        return Err(KeySideLifecycleError::XeniaArtifactSchemaMismatch);
    }
    if observation.subject_ref != lifecycle.target.did {
        return Err(KeySideLifecycleError::XeniaSubjectMismatch);
    }
    if observation.signer_key_fingerprint != &lifecycle.target.xenia_key_fingerprint {
        return Err(KeySideLifecycleError::XeniaSignerFingerprintMismatch);
    }
    if observation.signature_suite != lifecycle.target.xenia_signature_suite {
        return Err(KeySideLifecycleError::XeniaSignatureSuiteMismatch);
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

/// Errors for external-key-side association lifecycle artifacts.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum KeySideLifecycleError {
    /// Persisted schema is not v1.
    #[error("unsupported key-side lifecycle schema: {0}")]
    UnsupportedSchema(String),
    /// Target binding is structurally invalid.
    #[error("invalid target key-DID binding: {0}")]
    InvalidTarget(String),
    /// Replacement binding is structurally invalid.
    #[error("invalid replacement key-DID binding: {0}")]
    InvalidReplacement(String),
    /// Explanation exceeded the v1 bound.
    #[error("key-side lifecycle reason exceeds maximum {max}: found {found}")]
    ReasonTooLong {
        /// Maximum UTF-8 byte length.
        max: usize,
        /// Observed UTF-8 byte length.
        found: usize,
    },
    /// Explanation had leading/trailing whitespace.
    #[error("key-side lifecycle reason must be whitespace-canonical")]
    NonCanonicalReason,
    /// Disavow statements cannot propose a successor.
    #[error("key-side disavow must not include a replacement")]
    DisavowHasReplacement,
    /// Supersession requires a proposed successor.
    #[error("key-side supersession requires a replacement binding")]
    SupersedeMissingReplacement,
    /// Replacement must preserve the DID.
    #[error("replacement binding must preserve the same DID")]
    ReplacementDidMismatch,
    /// Replacement must preserve the descriptive purpose.
    #[error("replacement binding must preserve the same purpose")]
    ReplacementPurposeMismatch,
    /// Replacement must preserve the explicit scope.
    #[error("replacement binding must preserve the same scope")]
    ReplacementScopeMismatch,
    /// Replacing a binding with byte-for-byte equal semantics is meaningless.
    #[error("replacement binding must differ from the target")]
    NoOpSupersession,
    /// Xenia used the wrong semantic artifact domain.
    #[error("Xenia lifecycle attestation domain mismatch")]
    XeniaArtifactDomainMismatch,
    /// Xenia used the wrong domain-owned lifecycle schema.
    #[error("Xenia lifecycle attestation schema mismatch")]
    XeniaArtifactSchemaMismatch,
    /// Xenia subject reference did not equal the target DID.
    #[error("Xenia lifecycle subject mismatch")]
    XeniaSubjectMismatch,
    /// Xenia attestation was not signed by the target binding's key fingerprint.
    #[error("Xenia lifecycle signer fingerprint does not match target key")]
    XeniaSignerFingerprintMismatch,
    /// Xenia verified a signature suite different from the target binding's suite.
    #[error("Xenia lifecycle signature suite does not match target key suite")]
    XeniaSignatureSuiteMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{KeyAssociationPurpose, XeniaAttestationObservation};

    fn binding(fingerprint: u8) -> KeyDidBindingArtifact {
        KeyDidBindingArtifact::new(
            "did:mycelix:uhCAk-test-agent",
            [fingerprint; 32],
            "ed25519-rfc8032",
            KeyAssociationPurpose::EvidenceAttestor,
            "symthaea-generativity-provenance",
        )
        .unwrap()
    }

    #[test]
    fn disavow_is_self_contained_and_canonical() {
        let lifecycle = KeySideBindingLifecycleArtifact::new(
            binding(7),
            KeySideLifecycleDisposition::Disavow,
            None,
            Some("retiring signer".into()),
        )
        .unwrap();
        let first = lifecycle.canonical_bytes().unwrap();
        let encoded = serde_json::to_string(&lifecycle).unwrap();
        let decoded: KeySideBindingLifecycleArtifact = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded.canonical_bytes().unwrap(), first);
    }

    #[test]
    fn supersession_preserves_did_purpose_and_scope() {
        let lifecycle = KeySideBindingLifecycleArtifact::new(
            binding(7),
            KeySideLifecycleDisposition::Supersede,
            Some(binding(8)),
            None,
        );
        assert!(lifecycle.is_ok());

        let mut wrong_scope = binding(8);
        wrong_scope.scope = "mycelix-lineage-provenance".into();
        let err = KeySideBindingLifecycleArtifact::new(
            binding(7),
            KeySideLifecycleDisposition::Supersede,
            Some(wrong_scope),
            None,
        )
        .unwrap_err();
        assert_eq!(err, KeySideLifecycleError::ReplacementScopeMismatch);
    }

    #[test]
    fn no_op_supersession_is_rejected() {
        let same = binding(7);
        let err = KeySideBindingLifecycleArtifact::new(
            same.clone(),
            KeySideLifecycleDisposition::Supersede,
            Some(same),
            None,
        )
        .unwrap_err();
        assert_eq!(err, KeySideLifecycleError::NoOpSupersession);
    }

    #[test]
    fn target_key_must_be_the_xenia_signer() {
        let lifecycle = KeySideBindingLifecycleArtifact::new(
            binding(7),
            KeySideLifecycleDisposition::Disavow,
            None,
            None,
        )
        .unwrap();
        let correct = [7u8; 32];
        let observed = XeniaAttestationObservation {
            artifact_domain: XENIA_KEY_LIFECYCLE_ARTIFACT_DOMAIN,
            artifact_schema: XENIA_KEY_LIFECYCLE_ARTIFACT_SCHEMA,
            subject_ref: lifecycle.xenia_subject_ref(),
            signer_key_fingerprint: &correct,
            signature_suite: "ed25519-rfc8032",
        };
        assert_eq!(
            validate_key_side_lifecycle_xenia_observation(&lifecycle, observed),
            Ok(())
        );

        let wrong = [9u8; 32];
        let transplanted = XeniaAttestationObservation {
            artifact_domain: XENIA_KEY_LIFECYCLE_ARTIFACT_DOMAIN,
            artifact_schema: XENIA_KEY_LIFECYCLE_ARTIFACT_SCHEMA,
            subject_ref: lifecycle.xenia_subject_ref(),
            signer_key_fingerprint: &wrong,
            signature_suite: "ed25519-rfc8032",
        };
        assert_eq!(
            validate_key_side_lifecycle_xenia_observation(&lifecycle, transplanted),
            Err(KeySideLifecycleError::XeniaSignerFingerprintMismatch)
        );
    }

    #[test]
    fn lifecycle_domain_cannot_be_relabelled() {
        let lifecycle = KeySideBindingLifecycleArtifact::new(
            binding(7),
            KeySideLifecycleDisposition::Disavow,
            None,
            None,
        )
        .unwrap();
        let signer = [7u8; 32];
        let observed = XeniaAttestationObservation {
            artifact_domain: "mycelix-identity-key-binding",
            artifact_schema: XENIA_KEY_LIFECYCLE_ARTIFACT_SCHEMA,
            subject_ref: lifecycle.xenia_subject_ref(),
            signer_key_fingerprint: &signer,
            signature_suite: "ed25519-rfc8032",
        };
        assert_eq!(
            validate_key_side_lifecycle_xenia_observation(&lifecycle, observed),
            Err(KeySideLifecycleError::XeniaArtifactDomainMismatch)
        );
    }
}
