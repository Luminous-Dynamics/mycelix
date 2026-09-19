// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Protected-content boundary for stewarded exact representations.
//!
//! STEW-020 binds a STEW-002 `ProtectedRepresentation` envelope to opaque
//! ciphertext/container, cryptographic-profile, key-policy, access-policy,
//! threat-model, and revocation-semantics references. It contains neither
//! plaintext nor key material and grants no access or disclosure authority.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1,
};
use mycelix_stewardship_envelope::{PayloadDisclosureClassV1, StewardshipEnvelopeV1};

/// Stable profile identifier for this theorem.
pub const PROTECTED_CONTENT_PROFILE_V1: &str = "mycelix/protected-content/v1";

/// Maximum cultural-protocol references carried by one binding.
pub const MAX_CULTURAL_PROTOCOL_REFS_V1: usize = 32;

/// Typed protected-content binding identifier.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProtectedContentBindingIdV1(CanonicalIdV1);

impl ProtectedContentBindingIdV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Structural construction failures for a v1 protected-content binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtectedContentBindingErrorV1 {
    EnvelopeNotProtected,
    TooManyCulturalProtocolReferences,
    DuplicateCulturalProtocolReference,
}

impl fmt::Display for ProtectedContentBindingErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EnvelopeNotProtected => {
                f.write_str("protected-content binding requires a ProtectedRepresentation envelope")
            }
            Self::TooManyCulturalProtocolReferences => {
                f.write_str("too many cultural-protocol references for v1")
            }
            Self::DuplicateCulturalProtocolReference => {
                f.write_str("duplicate cultural-protocol reference")
            }
        }
    }
}

/// Metadata-only binding for a protected exact representation.
///
/// Deliberately absent fields include plaintext bytes, decryption keys, wrapped
/// key material, passwords, and any boolean claiming that access is authorized.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProtectedRepresentationBindingV1 {
    binding_id: ProtectedContentBindingIdV1,
    target: StewardedSubjectIdentityV1,
    ciphertext_ref: CanonicalIdV1,
    encryption_profile_ref: CanonicalIdV1,
    key_policy_ref: CanonicalIdV1,
    access_policy_ref: CanonicalIdV1,
    threat_model_ref: CanonicalIdV1,
    revocation_semantics_ref: CanonicalIdV1,
    cultural_protocol_refs: Vec<CanonicalIdV1>,
}

impl ProtectedRepresentationBindingV1 {
    #[allow(clippy::too_many_arguments)]
    pub fn from_envelope(
        envelope: &StewardshipEnvelopeV1,
        binding_id: ProtectedContentBindingIdV1,
        ciphertext_ref: CanonicalIdV1,
        encryption_profile_ref: CanonicalIdV1,
        key_policy_ref: CanonicalIdV1,
        access_policy_ref: CanonicalIdV1,
        threat_model_ref: CanonicalIdV1,
        revocation_semantics_ref: CanonicalIdV1,
        cultural_protocol_refs: Vec<CanonicalIdV1>,
    ) -> Result<Self, ProtectedContentBindingErrorV1> {
        if envelope.payload_disclosure() != PayloadDisclosureClassV1::ProtectedRepresentation {
            return Err(ProtectedContentBindingErrorV1::EnvelopeNotProtected);
        }
        if cultural_protocol_refs.len() > MAX_CULTURAL_PROTOCOL_REFS_V1 {
            return Err(ProtectedContentBindingErrorV1::TooManyCulturalProtocolReferences);
        }
        for (index, reference) in cultural_protocol_refs.iter().enumerate() {
            if cultural_protocol_refs[..index].contains(reference) {
                return Err(ProtectedContentBindingErrorV1::DuplicateCulturalProtocolReference);
            }
        }

        Ok(Self {
            binding_id,
            target: envelope.identity().clone(),
            ciphertext_ref,
            encryption_profile_ref,
            key_policy_ref,
            access_policy_ref,
            threat_model_ref,
            revocation_semantics_ref,
            cultural_protocol_refs,
        })
    }

    pub fn binding_id(&self) -> &ProtectedContentBindingIdV1 {
        &self.binding_id
    }

    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    pub fn ciphertext_ref(&self) -> &CanonicalIdV1 {
        &self.ciphertext_ref
    }

    pub fn encryption_profile_ref(&self) -> &CanonicalIdV1 {
        &self.encryption_profile_ref
    }

    pub fn key_policy_ref(&self) -> &CanonicalIdV1 {
        &self.key_policy_ref
    }

    pub fn access_policy_ref(&self) -> &CanonicalIdV1 {
        &self.access_policy_ref
    }

    pub fn threat_model_ref(&self) -> &CanonicalIdV1 {
        &self.threat_model_ref
    }

    pub fn revocation_semantics_ref(&self) -> &CanonicalIdV1 {
        &self.revocation_semantics_ref
    }

    pub fn cultural_protocol_refs(&self) -> &[CanonicalIdV1] {
        &self.cultural_protocol_refs
    }
}

/// A capability assertion is intentionally separate from authority.
///
/// This record can say that some principal is asserted to possess a technical
/// capability under a key/capability reference. It cannot say that exercising
/// that capability is culturally, contractually, or legally authorized.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecryptionCapabilityAssertionV1 {
    principal_ref: CanonicalIdV1,
    capability_ref: CanonicalIdV1,
    evidence_ref: CanonicalIdV1,
}

impl DecryptionCapabilityAssertionV1 {
    pub const fn new(
        principal_ref: CanonicalIdV1,
        capability_ref: CanonicalIdV1,
        evidence_ref: CanonicalIdV1,
    ) -> Self {
        Self {
            principal_ref,
            capability_ref,
            evidence_ref,
        }
    }

    pub fn principal_ref(&self) -> &CanonicalIdV1 {
        &self.principal_ref
    }

    pub fn capability_ref(&self) -> &CanonicalIdV1 {
        &self.capability_ref
    }

    pub fn evidence_ref(&self) -> &CanonicalIdV1 {
        &self.evidence_ref
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1,
    };
    use mycelix_stewardship_envelope::{PublicDescriptorV1, StewardshipEnvelopeErrorV1};

    fn id(value: &str) -> CanonicalIdV1 {
        CanonicalIdV1::new(value).unwrap()
    }

    fn identity() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:protected:oral-history").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:audio:1").unwrap(),
            kind: RepresentationKindV1::Audio,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [19; 32]),
        }
    }

    fn envelope(
        disclosure: PayloadDisclosureClassV1,
    ) -> Result<StewardshipEnvelopeV1, StewardshipEnvelopeErrorV1> {
        StewardshipEnvelopeV1::new(
            identity(),
            PublicDescriptorV1::new("Protected oral history", None).unwrap(),
            disclosure,
            vec![],
        )
    }

    fn protected_binding() -> ProtectedRepresentationBindingV1 {
        let envelope = envelope(PayloadDisclosureClassV1::ProtectedRepresentation).unwrap();
        ProtectedRepresentationBindingV1::from_envelope(
            &envelope,
            ProtectedContentBindingIdV1::new("protected-binding:1").unwrap(),
            id("ciphertext:object:1"),
            id("encryption-profile:xenia:1"),
            id("key-policy:1"),
            id("access-policy:1"),
            id("threat-model:1"),
            id("revocation-semantics:1"),
            vec![id("cultural-protocol:external:1")],
        )
        .unwrap()
    }

    #[test]
    fn protected_envelope_can_bind_without_plaintext_or_keys() {
        let binding = protected_binding();
        assert_eq!(binding.ciphertext_ref().as_str(), "ciphertext:object:1");
        assert_eq!(binding.key_policy_ref().as_str(), "key-policy:1");
        assert_eq!(binding.access_policy_ref().as_str(), "access-policy:1");
    }

    #[test]
    fn public_envelope_cannot_be_reinterpreted_as_protected_binding() {
        let public = envelope(PayloadDisclosureClassV1::PublicRepresentation).unwrap();
        let result = ProtectedRepresentationBindingV1::from_envelope(
            &public,
            ProtectedContentBindingIdV1::new("protected-binding:2").unwrap(),
            id("ciphertext:2"),
            id("encryption-profile:1"),
            id("key-policy:1"),
            id("access-policy:1"),
            id("threat-model:1"),
            id("revocation-semantics:1"),
            vec![],
        );
        assert_eq!(result, Err(ProtectedContentBindingErrorV1::EnvelopeNotProtected));
    }

    #[test]
    fn exact_envelope_identity_is_retained() {
        let binding = protected_binding();
        assert_eq!(binding.target(), &identity());
    }

    #[test]
    fn duplicate_cultural_protocol_refs_are_rejected() {
        let envelope = envelope(PayloadDisclosureClassV1::ProtectedRepresentation).unwrap();
        let duplicate = id("cultural-protocol:1");
        let result = ProtectedRepresentationBindingV1::from_envelope(
            &envelope,
            ProtectedContentBindingIdV1::new("protected-binding:3").unwrap(),
            id("ciphertext:3"),
            id("encryption-profile:1"),
            id("key-policy:1"),
            id("access-policy:1"),
            id("threat-model:1"),
            id("revocation-semantics:1"),
            vec![duplicate.clone(), duplicate],
        );
        assert_eq!(
            result,
            Err(ProtectedContentBindingErrorV1::DuplicateCulturalProtocolReference)
        );
    }

    #[test]
    fn technical_capability_never_becomes_authority_in_this_theorem() {
        let assertion = DecryptionCapabilityAssertionV1::new(
            id("principal:1"),
            id("capability:key-slot:1"),
            id("evidence:key-possession-test:1"),
        );
        assert_eq!(assertion.principal_ref().as_str(), "principal:1");
        assert_eq!(assertion.capability_ref().as_str(), "capability:key-slot:1");
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(PROTECTED_CONTENT_PROFILE_V1, "mycelix/protected-content/v1");
    }
}
