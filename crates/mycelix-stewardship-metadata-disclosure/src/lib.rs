// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed disclosure-basis coverage for complete public STEW-002 envelopes.
//!
//! A successful construction means only that every public surface has at least
//! one explicit basis reference. The theorem does not verify those bases or
//! authorize publication.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};
use mycelix_stewardship_envelope::{
    EnvelopeReferenceKindV1, EnvelopeReferenceV1, StewardshipEnvelopeV1,
};

pub const METADATA_DISCLOSURE_PROFILE_V1: &str = "mycelix/metadata-disclosure/v1";
pub const MAX_DISCLOSURE_BASES_V1: usize = 256;

/// Exact public surface whose disclosure needs an explicit basis.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MetadataDisclosureSurfaceV1 {
    Existence,
    ExactIdentity,
    PublicTitle,
    PublicSummary,
    Reference {
        kind: EnvelopeReferenceKindV1,
        reference: CanonicalIdV1,
    },
}

impl MetadataDisclosureSurfaceV1 {
    pub fn for_reference(reference: &EnvelopeReferenceV1) -> Self {
        Self::Reference {
            kind: reference.kind,
            reference: reference.reference.clone(),
        }
    }
}

/// Opaque external basis for exposing one exact metadata surface.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct MetadataDisclosureBasisV1 {
    surface: MetadataDisclosureSurfaceV1,
    basis_ref: CanonicalIdV1,
}

impl MetadataDisclosureBasisV1 {
    pub fn new(
        surface: MetadataDisclosureSurfaceV1,
        basis_ref: impl Into<String>,
    ) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self {
            surface,
            basis_ref: CanonicalIdV1::new(basis_ref)?,
        })
    }

    pub fn surface(&self) -> &MetadataDisclosureSurfaceV1 {
        &self.surface
    }

    pub fn basis_ref(&self) -> &CanonicalIdV1 {
        &self.basis_ref
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PublicEnvelopeAdmissionErrorV1 {
    TooManyDisclosureBases,
    DuplicateDisclosureBasis,
    MissingExistenceBasis,
    MissingIdentityBasis,
    MissingTitleBasis,
    MissingSummaryBasis,
    MissingReferenceBasis,
}

impl fmt::Display for PublicEnvelopeAdmissionErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyDisclosureBases => f.write_str("too many metadata disclosure bases for v1"),
            Self::DuplicateDisclosureBasis => f.write_str("duplicate metadata disclosure basis"),
            Self::MissingExistenceBasis => f.write_str("public envelope lacks an explicit existence-disclosure basis"),
            Self::MissingIdentityBasis => f.write_str("public envelope lacks an explicit identity-disclosure basis"),
            Self::MissingTitleBasis => f.write_str("public envelope lacks an explicit title-disclosure basis"),
            Self::MissingSummaryBasis => f.write_str("public envelope summary lacks an explicit disclosure basis"),
            Self::MissingReferenceBasis => f.write_str("at least one public envelope reference lacks an explicit disclosure basis"),
        }
    }
}

fn has_surface(
    bases: &[MetadataDisclosureBasisV1],
    surface: &MetadataDisclosureSurfaceV1,
) -> bool {
    bases.iter().any(|basis| basis.surface() == surface)
}

/// Structurally complete candidate for publication of the *entire* STEW-002
/// public envelope. This is not an authorization result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublicEnvelopePublicationCandidateV1 {
    envelope: StewardshipEnvelopeV1,
    disclosure_bases: Vec<MetadataDisclosureBasisV1>,
}

impl PublicEnvelopePublicationCandidateV1 {
    pub fn new(
        envelope: StewardshipEnvelopeV1,
        disclosure_bases: Vec<MetadataDisclosureBasisV1>,
    ) -> Result<Self, PublicEnvelopeAdmissionErrorV1> {
        if disclosure_bases.len() > MAX_DISCLOSURE_BASES_V1 {
            return Err(PublicEnvelopeAdmissionErrorV1::TooManyDisclosureBases);
        }
        for (index, basis) in disclosure_bases.iter().enumerate() {
            if disclosure_bases[..index].contains(basis) {
                return Err(PublicEnvelopeAdmissionErrorV1::DuplicateDisclosureBasis);
            }
        }

        if !has_surface(&disclosure_bases, &MetadataDisclosureSurfaceV1::Existence) {
            return Err(PublicEnvelopeAdmissionErrorV1::MissingExistenceBasis);
        }
        if !has_surface(&disclosure_bases, &MetadataDisclosureSurfaceV1::ExactIdentity) {
            return Err(PublicEnvelopeAdmissionErrorV1::MissingIdentityBasis);
        }
        if !has_surface(&disclosure_bases, &MetadataDisclosureSurfaceV1::PublicTitle) {
            return Err(PublicEnvelopeAdmissionErrorV1::MissingTitleBasis);
        }
        if envelope.descriptor().summary().is_some()
            && !has_surface(&disclosure_bases, &MetadataDisclosureSurfaceV1::PublicSummary)
        {
            return Err(PublicEnvelopeAdmissionErrorV1::MissingSummaryBasis);
        }
        if envelope.references().iter().any(|reference| {
            !has_surface(
                &disclosure_bases,
                &MetadataDisclosureSurfaceV1::for_reference(reference),
            )
        }) {
            return Err(PublicEnvelopeAdmissionErrorV1::MissingReferenceBasis);
        }

        Ok(Self {
            envelope,
            disclosure_bases,
        })
    }

    pub fn envelope(&self) -> &StewardshipEnvelopeV1 {
        &self.envelope
    }

    pub fn disclosure_bases(&self) -> &[MetadataDisclosureBasisV1] {
        &self.disclosure_bases
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1, StewardedSubjectIdentityV1,
    };
    use mycelix_stewardship_envelope::{PayloadDisclosureClassV1, PublicDescriptorV1};

    fn identity() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:protected:1").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:encrypted:1").unwrap(),
            kind: RepresentationKindV1::Other,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [9; 32]),
        }
    }

    fn envelope(with_summary: bool, with_reference: bool) -> StewardshipEnvelopeV1 {
        let mut refs = Vec::new();
        if with_reference {
            refs.push(
                EnvelopeReferenceV1::new(
                    EnvelopeReferenceKindV1::CulturalProtocol,
                    "cultural-protocol:1",
                )
                .unwrap(),
            );
        }
        StewardshipEnvelopeV1::new(
            identity(),
            PublicDescriptorV1::new(
                "Public-safe title",
                with_summary.then(|| "Public-safe summary".to_string()),
            )
            .unwrap(),
            PayloadDisclosureClassV1::ProtectedRepresentation,
            refs,
        )
        .unwrap()
    }

    fn basis(surface: MetadataDisclosureSurfaceV1, id: &str) -> MetadataDisclosureBasisV1 {
        MetadataDisclosureBasisV1::new(surface, id).unwrap()
    }

    fn baseline_bases() -> Vec<MetadataDisclosureBasisV1> {
        vec![
            basis(MetadataDisclosureSurfaceV1::Existence, "basis:existence:1"),
            basis(MetadataDisclosureSurfaceV1::ExactIdentity, "basis:identity:1"),
            basis(MetadataDisclosureSurfaceV1::PublicTitle, "basis:title:1"),
        ]
    }

    #[test]
    fn minimal_public_envelope_requires_existence_identity_and_title_bases() {
        let candidate = PublicEnvelopePublicationCandidateV1::new(
            envelope(false, false),
            baseline_bases(),
        )
        .unwrap();
        assert_eq!(candidate.disclosure_bases().len(), 3);
    }

    #[test]
    fn missing_existence_fails_closed() {
        let result = PublicEnvelopePublicationCandidateV1::new(
            envelope(false, false),
            vec![
                basis(MetadataDisclosureSurfaceV1::ExactIdentity, "basis:identity:1"),
                basis(MetadataDisclosureSurfaceV1::PublicTitle, "basis:title:1"),
            ],
        );
        assert_eq!(result, Err(PublicEnvelopeAdmissionErrorV1::MissingExistenceBasis));
    }

    #[test]
    fn summary_requires_its_own_disclosure_basis() {
        let result = PublicEnvelopePublicationCandidateV1::new(
            envelope(true, false),
            baseline_bases(),
        );
        assert_eq!(result, Err(PublicEnvelopeAdmissionErrorV1::MissingSummaryBasis));

        let mut bases = baseline_bases();
        bases.push(basis(MetadataDisclosureSurfaceV1::PublicSummary, "basis:summary:1"));
        assert!(PublicEnvelopePublicationCandidateV1::new(envelope(true, false), bases).is_ok());
    }

    #[test]
    fn every_exact_reference_requires_a_disclosure_basis() {
        let protected = envelope(false, true);
        let result = PublicEnvelopePublicationCandidateV1::new(
            protected.clone(),
            baseline_bases(),
        );
        assert_eq!(result, Err(PublicEnvelopeAdmissionErrorV1::MissingReferenceBasis));

        let reference_surface = MetadataDisclosureSurfaceV1::for_reference(&protected.references()[0]);
        let mut bases = baseline_bases();
        bases.push(basis(reference_surface, "basis:reference:1"));
        assert!(PublicEnvelopePublicationCandidateV1::new(protected, bases).is_ok());
    }

    #[test]
    fn a_basis_for_one_reference_cannot_cover_a_different_reference() {
        let protected = envelope(false, true);
        let mut bases = baseline_bases();
        bases.push(basis(
            MetadataDisclosureSurfaceV1::Reference {
                kind: EnvelopeReferenceKindV1::AccessPolicy,
                reference: CanonicalIdV1::new("policy:other").unwrap(),
            },
            "basis:wrong-reference:1",
        ));
        assert_eq!(
            PublicEnvelopePublicationCandidateV1::new(protected, bases),
            Err(PublicEnvelopeAdmissionErrorV1::MissingReferenceBasis)
        );
    }

    #[test]
    fn duplicate_exact_basis_is_rejected() {
        let duplicate = basis(MetadataDisclosureSurfaceV1::Existence, "basis:existence:1");
        let result = PublicEnvelopePublicationCandidateV1::new(
            envelope(false, false),
            vec![
                duplicate.clone(),
                duplicate,
                basis(MetadataDisclosureSurfaceV1::ExactIdentity, "basis:identity:1"),
                basis(MetadataDisclosureSurfaceV1::PublicTitle, "basis:title:1"),
            ],
        );
        assert_eq!(result, Err(PublicEnvelopeAdmissionErrorV1::DuplicateDisclosureBasis));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(METADATA_DISCLOSURE_PROFILE_V1, "mycelix/metadata-disclosure/v1");
    }
}
