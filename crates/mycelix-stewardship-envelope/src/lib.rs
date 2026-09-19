// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral stewardship envelope theorem.
//!
//! STEW-002 composes structural identity with a public descriptor, a declared
//! payload disclosure class, and typed references. A reference records that an
//! external or later theorem object is associated with the envelope; it does not
//! establish that object's authority, authenticity, applicability, or validity.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1,
};

/// Stable profile identifier for STEW-002.
pub const STEWARDSHIP_ENVELOPE_PROFILE_V1: &str = "mycelix/stewardship-envelope/v1";

/// Maximum byte length of the human-facing public title.
pub const MAX_PUBLIC_TITLE_BYTES_V1: usize = 1024;
/// Maximum byte length of the optional public summary.
pub const MAX_PUBLIC_SUMMARY_BYTES_V1: usize = 8192;
/// Maximum number of typed references admitted by one v1 envelope.
pub const MAX_ENVELOPE_REFERENCES_V1: usize = 128;

/// Public descriptor validation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PublicDescriptorErrorV1 {
    /// A public descriptor needs a non-empty title.
    EmptyTitle,
    /// The title exceeds the v1 byte bound.
    TitleTooLong,
    /// The optional summary exceeds the v1 byte bound.
    SummaryTooLong,
}

impl fmt::Display for PublicDescriptorErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyTitle => f.write_str("public title must not be empty"),
            Self::TitleTooLong => f.write_str("public title exceeds v1 byte limit"),
            Self::SummaryTooLong => f.write_str("public summary exceeds v1 byte limit"),
        }
    }
}

/// Human-facing metadata that is explicitly safe to place in the envelope.
///
/// This type intentionally says nothing about whether additional protected
/// metadata exists elsewhere.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct PublicDescriptorV1 {
    title: String,
    summary: Option<String>,
}

impl PublicDescriptorV1 {
    /// Construct bounded public-facing metadata.
    pub fn new(
        title: impl Into<String>,
        summary: Option<String>,
    ) -> Result<Self, PublicDescriptorErrorV1> {
        let title = title.into();
        if title.trim().is_empty() {
            return Err(PublicDescriptorErrorV1::EmptyTitle);
        }
        if title.len() > MAX_PUBLIC_TITLE_BYTES_V1 {
            return Err(PublicDescriptorErrorV1::TitleTooLong);
        }
        if summary
            .as_ref()
            .is_some_and(|value| value.len() > MAX_PUBLIC_SUMMARY_BYTES_V1)
        {
            return Err(PublicDescriptorErrorV1::SummaryTooLong);
        }
        Ok(Self { title, summary })
    }

    /// Public display title.
    pub fn title(&self) -> &str {
        &self.title
    }

    /// Optional public summary.
    pub fn summary(&self) -> Option<&str> {
        self.summary.as_deref()
    }
}

/// What the envelope declares about payload disclosure.
///
/// These are descriptive envelope states, not proofs that confidentiality or
/// availability is correctly enforced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PayloadDisclosureClassV1 {
    /// The referenced representation is intended for public retrieval/use under
    /// whatever later policy theorem applies.
    PublicRepresentation,
    /// The representation is declared protected and must not be inferred to be
    /// public merely because this envelope is public.
    ProtectedRepresentation,
    /// The envelope intentionally carries metadata only; no representation is
    /// declared available through this envelope.
    MetadataOnly,
    /// The representation is known but currently unavailable through the
    /// stewarding system.
    Unavailable,
}

/// Closed v1 semantic kinds for references carried by an envelope.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EnvelopeReferenceKindV1 {
    /// Reference to a provenance object/graph/receipt.
    Provenance,
    /// Reference to a stewardship claim.
    StewardshipClaim,
    /// Reference to an access/use policy.
    AccessPolicy,
    /// Reference to a legal/rights policy or record.
    RightsPolicy,
    /// Reference to a community-controlled cultural protocol/label record.
    CulturalProtocol,
    /// Reference to a preservation manifest/receipt.
    PreservationManifest,
    /// Reference to a reciprocity policy/obligation/receipt.
    Reciprocity,
    /// Reference to a versioned interoperability projection receipt/profile.
    InteropProjection,
    /// Reference has a known association but is outside the closed v1 kinds.
    Other,
}

/// One typed, authority-neutral association from an envelope to another object.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct EnvelopeReferenceV1 {
    /// Semantic role of the reference.
    pub kind: EnvelopeReferenceKindV1,
    /// Opaque canonical identifier of the referenced object.
    pub reference: CanonicalIdV1,
}

impl EnvelopeReferenceV1 {
    /// Construct a typed reference from an opaque protocol identifier.
    pub fn new(
        kind: EnvelopeReferenceKindV1,
        reference: impl Into<String>,
    ) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self {
            kind,
            reference: CanonicalIdV1::new(reference)?,
        })
    }
}

/// Stewardship envelope validation failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StewardshipEnvelopeErrorV1 {
    /// Too many references were supplied.
    TooManyReferences,
    /// The same semantic kind and exact reference identifier appeared twice.
    DuplicateReference,
}

impl fmt::Display for StewardshipEnvelopeErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyReferences => f.write_str("too many envelope references for v1"),
            Self::DuplicateReference => f.write_str("duplicate typed envelope reference"),
        }
    }
}

/// Authority-neutral public stewardship envelope.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StewardshipEnvelopeV1 {
    identity: StewardedSubjectIdentityV1,
    descriptor: PublicDescriptorV1,
    payload_disclosure: PayloadDisclosureClassV1,
    references: Vec<EnvelopeReferenceV1>,
}

impl StewardshipEnvelopeV1 {
    /// Construct an envelope while preserving reference-kind separation.
    pub fn new(
        identity: StewardedSubjectIdentityV1,
        descriptor: PublicDescriptorV1,
        payload_disclosure: PayloadDisclosureClassV1,
        references: Vec<EnvelopeReferenceV1>,
    ) -> Result<Self, StewardshipEnvelopeErrorV1> {
        if references.len() > MAX_ENVELOPE_REFERENCES_V1 {
            return Err(StewardshipEnvelopeErrorV1::TooManyReferences);
        }

        for (index, reference) in references.iter().enumerate() {
            if references[..index].contains(reference) {
                return Err(StewardshipEnvelopeErrorV1::DuplicateReference);
            }
        }

        Ok(Self {
            identity,
            descriptor,
            payload_disclosure,
            references,
        })
    }

    /// Structural identity established by STEW-001 only.
    pub fn identity(&self) -> &StewardedSubjectIdentityV1 {
        &self.identity
    }

    /// Public human-facing descriptor.
    pub fn descriptor(&self) -> &PublicDescriptorV1 {
        &self.descriptor
    }

    /// Declared disclosure class; not a confidentiality proof.
    pub const fn payload_disclosure(&self) -> PayloadDisclosureClassV1 {
        self.payload_disclosure
    }

    /// All typed references in insertion order.
    pub fn references(&self) -> &[EnvelopeReferenceV1] {
        &self.references
    }

    /// Iterate references of one semantic kind without collapsing kinds.
    pub fn references_of_kind(
        &self,
        kind: EnvelopeReferenceKindV1,
    ) -> impl Iterator<Item = &EnvelopeReferenceV1> {
        self.references.iter().filter(move |reference| reference.kind == kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, IdentityRelationV1, RepresentationIdV1,
        RepresentationKindV1, RevisionIdV1, StewardedSubjectIdV1,
    };

    fn identity() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:oral-history:example").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:transcript:1").unwrap(),
            kind: RepresentationKindV1::Text,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [7; 32]),
        }
    }

    fn descriptor() -> PublicDescriptorV1 {
        PublicDescriptorV1::new(
            "Public descriptor — 文化 / culture",
            Some("Safe public metadata only".into()),
        )
        .unwrap()
    }

    #[test]
    fn public_descriptor_accepts_unicode_without_using_it_as_protocol_identity() {
        let descriptor = descriptor();
        assert!(descriptor.title().contains("文化"));
        assert_eq!(descriptor.summary(), Some("Safe public metadata only"));
    }

    #[test]
    fn public_descriptor_rejects_empty_and_overlong_values() {
        assert_eq!(
            PublicDescriptorV1::new("   ", None),
            Err(PublicDescriptorErrorV1::EmptyTitle)
        );
        assert_eq!(
            PublicDescriptorV1::new("a".repeat(MAX_PUBLIC_TITLE_BYTES_V1 + 1), None),
            Err(PublicDescriptorErrorV1::TitleTooLong)
        );
        assert_eq!(
            PublicDescriptorV1::new(
                "title",
                Some("s".repeat(MAX_PUBLIC_SUMMARY_BYTES_V1 + 1)),
            ),
            Err(PublicDescriptorErrorV1::SummaryTooLong)
        );
    }

    #[test]
    fn protected_disclosure_is_representable_without_payload_plaintext_field() {
        let envelope = StewardshipEnvelopeV1::new(
            identity(),
            descriptor(),
            PayloadDisclosureClassV1::ProtectedRepresentation,
            vec![],
        )
        .unwrap();
        assert_eq!(
            envelope.payload_disclosure(),
            PayloadDisclosureClassV1::ProtectedRepresentation
        );
    }

    #[test]
    fn references_do_not_change_structural_identity() {
        let base_identity = identity();
        let envelope = StewardshipEnvelopeV1::new(
            base_identity.clone(),
            descriptor(),
            PayloadDisclosureClassV1::MetadataOnly,
            vec![EnvelopeReferenceV1::new(
                EnvelopeReferenceKindV1::AccessPolicy,
                "policy:access:example-v1",
            )
            .unwrap()],
        )
        .unwrap();

        assert_eq!(
            base_identity.relation_to(envelope.identity()),
            IdentityRelationV1::ExactRepresentation
        );
    }

    #[test]
    fn duplicate_exact_typed_reference_is_rejected() {
        let reference = EnvelopeReferenceV1::new(
            EnvelopeReferenceKindV1::CulturalProtocol,
            "external:community-protocol:123",
        )
        .unwrap();
        let result = StewardshipEnvelopeV1::new(
            identity(),
            descriptor(),
            PayloadDisclosureClassV1::MetadataOnly,
            vec![reference.clone(), reference],
        );
        assert_eq!(
            result,
            Err(StewardshipEnvelopeErrorV1::DuplicateReference)
        );
    }

    #[test]
    fn same_identifier_may_have_distinct_semantic_reference_kinds() {
        let identifier = "artifact:external:123";
        let envelope = StewardshipEnvelopeV1::new(
            identity(),
            descriptor(),
            PayloadDisclosureClassV1::MetadataOnly,
            vec![
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::Provenance,
                    identifier,
                )
                .unwrap(),
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::InteropProjection,
                    identifier,
                )
                .unwrap(),
            ],
        )
        .unwrap();
        assert_eq!(envelope.references().len(), 2);
        assert_eq!(
            envelope
                .references_of_kind(EnvelopeReferenceKindV1::Provenance)
                .count(),
            1
        );
    }

    #[test]
    fn reference_count_is_bounded() {
        let references = (0..=MAX_ENVELOPE_REFERENCES_V1)
            .map(|index| {
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::Other,
                    format!("reference:{index}"),
                )
                .unwrap()
            })
            .collect();
        assert_eq!(
            StewardshipEnvelopeV1::new(
                identity(),
                descriptor(),
                PayloadDisclosureClassV1::MetadataOnly,
                references,
            ),
            Err(StewardshipEnvelopeErrorV1::TooManyReferences)
        );
    }

    #[test]
    fn reference_kind_filter_does_not_promote_other_kinds() {
        let envelope = StewardshipEnvelopeV1::new(
            identity(),
            descriptor(),
            PayloadDisclosureClassV1::MetadataOnly,
            vec![
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::StewardshipClaim,
                    "claim:stewardship:1",
                )
                .unwrap(),
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::AccessPolicy,
                    "policy:access:1",
                )
                .unwrap(),
            ],
        )
        .unwrap();

        assert_eq!(
            envelope
                .references_of_kind(EnvelopeReferenceKindV1::StewardshipClaim)
                .count(),
            1
        );
        assert_eq!(
            envelope
                .references_of_kind(EnvelopeReferenceKindV1::AccessPolicy)
                .count(),
            1
        );
        assert_eq!(
            envelope
                .references_of_kind(EnvelopeReferenceKindV1::RightsPolicy)
                .count(),
            0
        );
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            STEWARDSHIP_ENVELOPE_PROFILE_V1,
            "mycelix/stewardship-envelope/v1"
        );
    }
}
