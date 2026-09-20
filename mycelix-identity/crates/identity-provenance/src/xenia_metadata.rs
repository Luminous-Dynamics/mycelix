// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure metadata checks between a Mycelix key-DID binding and an already
//! cryptographically verified Xenia artifact attestation.
//!
//! This module does not verify signatures. It prevents an integration layer from
//! verifying one Xenia object and accidentally pairing its result with a different
//! Mycelix semantic binding.

use thiserror::Error;

use crate::{
    KeyAssociationPurpose, KeyDidBindingArtifact, XENIA_ARTIFACT_DOMAIN, XENIA_ARTIFACT_SCHEMA,
};

/// Metadata copied from an independently verified Xenia artifact attestation.
///
/// Construction is intentionally ordinary: this crate cannot know whether a caller
/// really performed Xenia verification. The type is therefore named `Observation`,
/// not `Verified*`. The integration boundary remains responsible for producing it
/// only from a successful Xenia verifier result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XeniaAttestationObservation<'a> {
    /// Artifact domain returned by Xenia verification.
    pub artifact_domain: &'a str,
    /// Domain-owned artifact schema returned by Xenia verification.
    pub artifact_schema: &'a str,
    /// Subject reference returned by Xenia verification.
    pub subject_ref: &'a str,
    /// Signer key fingerprint returned by Xenia verification.
    pub signer_key_fingerprint: &'a [u8; 32],
    /// Signature-suite stable label returned by Xenia verification.
    pub signature_suite: &'a str,
}

/// Require an observed Xenia attestation to correspond exactly to this Mycelix
/// binding artifact.
///
/// Successful matching means only that the observed Xenia verifier output is
/// semantically consistent with the binding. It does not establish that the
/// observation itself came from a real verifier call.
pub fn validate_xenia_attestation_observation(
    binding: &KeyDidBindingArtifact,
    observation: XeniaAttestationObservation<'_>,
) -> Result<(), XeniaBindingMismatch> {
    binding
        .validate()
        .map_err(|error| XeniaBindingMismatch::InvalidBinding(error.to_string()))?;

    if observation.artifact_domain != XENIA_ARTIFACT_DOMAIN {
        return Err(XeniaBindingMismatch::ArtifactDomain);
    }
    if observation.artifact_schema != XENIA_ARTIFACT_SCHEMA {
        return Err(XeniaBindingMismatch::ArtifactSchema);
    }
    if observation.subject_ref != binding.did {
        return Err(XeniaBindingMismatch::Subject);
    }
    if observation.signer_key_fingerprint != &binding.xenia_key_fingerprint {
        return Err(XeniaBindingMismatch::SignerFingerprint);
    }
    if observation.signature_suite != binding.xenia_signature_suite {
        return Err(XeniaBindingMismatch::SignatureSuite);
    }
    Ok(())
}

/// Require that a bound key is being used for exactly its declared purpose and
/// scope.
///
/// V1 intentionally has no wildcard, hierarchy, inheritance, or implied-purpose
/// semantics.
pub fn validate_associated_key_use(
    binding: &KeyDidBindingArtifact,
    purpose: KeyAssociationPurpose,
    scope: &str,
) -> Result<(), XeniaBindingMismatch> {
    binding
        .validate()
        .map_err(|error| XeniaBindingMismatch::InvalidBinding(error.to_string()))?;
    if binding.purpose != purpose {
        return Err(XeniaBindingMismatch::Purpose);
    }
    if binding.scope != scope {
        return Err(XeniaBindingMismatch::Scope);
    }
    Ok(())
}

/// Semantic mismatches between a Mycelix DID-key association and Xenia output.
#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum XeniaBindingMismatch {
    /// The underlying Mycelix binding itself was invalid.
    #[error("invalid Mycelix key-DID binding: {0}")]
    InvalidBinding(String),
    /// Xenia attested under the wrong artifact domain.
    #[error("Xenia artifact domain does not match Mycelix key-binding domain")]
    ArtifactDomain,
    /// Xenia attested under the wrong domain-owned artifact schema.
    #[error("Xenia artifact schema does not match Mycelix key-binding schema")]
    ArtifactSchema,
    /// Xenia's attested subject differs from the bound DID.
    #[error("Xenia attestation subject does not match bound DID")]
    Subject,
    /// The signer fingerprint differs from the key fingerprint bound by the DID.
    #[error("Xenia signer fingerprint does not match bound key fingerprint")]
    SignerFingerprint,
    /// Xenia verified a different signature suite than the binding declares.
    #[error("Xenia signature suite does not match bound key suite")]
    SignatureSuite,
    /// The key is being used for a purpose other than the exact declared purpose.
    #[error("associated key purpose does not match requested use")]
    Purpose,
    /// The key is being used outside its exact declared semantic scope.
    #[error("associated key scope does not match requested use")]
    Scope,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::KeyDidBindingArtifact;

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
    fn exact_xenia_metadata_matches() {
        let binding = binding();
        let fingerprint = binding.xenia_key_fingerprint;
        let observed = XeniaAttestationObservation {
            artifact_domain: XENIA_ARTIFACT_DOMAIN,
            artifact_schema: XENIA_ARTIFACT_SCHEMA,
            subject_ref: &binding.did,
            signer_key_fingerprint: &fingerprint,
            signature_suite: &binding.xenia_signature_suite,
        };
        assert_eq!(validate_xenia_attestation_observation(&binding, observed), Ok(()));
    }

    #[test]
    fn signer_transplant_is_rejected() {
        let binding = binding();
        let wrong = [9u8; 32];
        let observed = XeniaAttestationObservation {
            artifact_domain: XENIA_ARTIFACT_DOMAIN,
            artifact_schema: XENIA_ARTIFACT_SCHEMA,
            subject_ref: &binding.did,
            signer_key_fingerprint: &wrong,
            signature_suite: &binding.xenia_signature_suite,
        };
        assert_eq!(
            validate_xenia_attestation_observation(&binding, observed),
            Err(XeniaBindingMismatch::SignerFingerprint)
        );
    }

    #[test]
    fn semantic_domain_relabel_is_rejected() {
        let binding = binding();
        let fingerprint = binding.xenia_key_fingerprint;
        let observed = XeniaAttestationObservation {
            artifact_domain: "mycelix-governance-authorization",
            artifact_schema: XENIA_ARTIFACT_SCHEMA,
            subject_ref: &binding.did,
            signer_key_fingerprint: &fingerprint,
            signature_suite: &binding.xenia_signature_suite,
        };
        assert_eq!(
            validate_xenia_attestation_observation(&binding, observed),
            Err(XeniaBindingMismatch::ArtifactDomain)
        );
    }

    #[test]
    fn use_is_exact_not_inherited() {
        let binding = binding();
        assert_eq!(
            validate_associated_key_use(
                &binding,
                KeyAssociationPurpose::EvidenceAttestor,
                "symthaea-generativity-provenance"
            ),
            Ok(())
        );
        assert_eq!(
            validate_associated_key_use(
                &binding,
                KeyAssociationPurpose::EvidenceWitness,
                "symthaea-generativity-provenance"
            ),
            Err(XeniaBindingMismatch::Purpose)
        );
        assert_eq!(
            validate_associated_key_use(
                &binding,
                KeyAssociationPurpose::EvidenceAttestor,
                "symthaea-generativity"
            ),
            Err(XeniaBindingMismatch::Scope)
        );
    }
}
