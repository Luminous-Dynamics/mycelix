// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Arts-oriented lineage profile over exact STEW-006 provenance edges.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{CanonicalIdErrorV1, CanonicalIdV1};
use mycelix_stewardship_provenance::{ProvenanceEdgeV1, ProvenanceRelationV1};

pub const CREATIVE_LINEAGE_PROFILE_V1: &str = "mycelix/creative-lineage/v1";
pub const MAX_CREATIVE_CONTEXT_REFS_V1: usize = 64;

/// Human-creative interpretation of a provenance relation.
///
/// Each variant is intentionally constrained to one STEW-006 relation so this
/// profile cannot contradict the underlying lineage theorem.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CreativeLineageKindV1 {
    Edition,
    Arrangement,
    Adaptation,
    Translation,
    Performance,
    Recording,
    Restoration,
    Remix,
    Digitization,
    Annotation,
    Excerpt,
}

impl CreativeLineageKindV1 {
    pub const fn required_relation(self) -> ProvenanceRelationV1 {
        match self {
            Self::Edition => ProvenanceRelationV1::VersionedFrom,
            Self::Arrangement | Self::Adaptation => ProvenanceRelationV1::DerivedFrom,
            Self::Translation => ProvenanceRelationV1::TranslatedFrom,
            Self::Performance => ProvenanceRelationV1::PerformedFrom,
            Self::Recording => ProvenanceRelationV1::RecordedFrom,
            Self::Restoration => ProvenanceRelationV1::RestoredFrom,
            Self::Remix => ProvenanceRelationV1::RemixedFrom,
            Self::Digitization => ProvenanceRelationV1::DigitizedFrom,
            Self::Annotation => ProvenanceRelationV1::AnnotatedFrom,
            Self::Excerpt => ProvenanceRelationV1::ExtractedFrom,
        }
    }
}

/// Context category attached to a creative lineage record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CreativeContextKindV1 {
    Instrumentation,
    Tuning,
    Language,
    PerformancePractice,
    Choreography,
    MaterialOrTechnique,
    VenueOrSetting,
    CulturalContext,
    InterpretiveNote,
    SourceEdition,
    Other,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CreativeContextRefV1 {
    kind: CreativeContextKindV1,
    reference: CanonicalIdV1,
}

impl CreativeContextRefV1 {
    pub fn new(
        kind: CreativeContextKindV1,
        reference: impl Into<String>,
    ) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self {
            kind,
            reference: CanonicalIdV1::new(reference)?,
        })
    }

    pub const fn kind(&self) -> CreativeContextKindV1 {
        self.kind
    }

    pub fn reference(&self) -> &CanonicalIdV1 {
        &self.reference
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CreativeLineageErrorV1 {
    RelationKindMismatch,
    TooManyContextReferences,
    DuplicateContextReference,
}

impl fmt::Display for CreativeLineageErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RelationKindMismatch => f.write_str("creative lineage kind contradicts the STEW-006 provenance relation"),
            Self::TooManyContextReferences => f.write_str("too many creative context references for v1"),
            Self::DuplicateContextReference => f.write_str("duplicate creative context reference"),
        }
    }
}

/// Creative profile around one exact evidence-bearing provenance edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CreativeLineageRecordV1 {
    lineage_id: CanonicalIdV1,
    edge: ProvenanceEdgeV1,
    kind: CreativeLineageKindV1,
    context_refs: Vec<CreativeContextRefV1>,
}

impl CreativeLineageRecordV1 {
    pub fn new(
        lineage_id: CanonicalIdV1,
        edge: ProvenanceEdgeV1,
        kind: CreativeLineageKindV1,
        context_refs: Vec<CreativeContextRefV1>,
    ) -> Result<Self, CreativeLineageErrorV1> {
        if edge.relation() != kind.required_relation() {
            return Err(CreativeLineageErrorV1::RelationKindMismatch);
        }
        if context_refs.len() > MAX_CREATIVE_CONTEXT_REFS_V1 {
            return Err(CreativeLineageErrorV1::TooManyContextReferences);
        }
        for (index, reference) in context_refs.iter().enumerate() {
            if context_refs[..index].contains(reference) {
                return Err(CreativeLineageErrorV1::DuplicateContextReference);
            }
        }
        Ok(Self {
            lineage_id,
            edge,
            kind,
            context_refs,
        })
    }

    pub fn lineage_id(&self) -> &CanonicalIdV1 {
        &self.lineage_id
    }

    pub fn edge(&self) -> &ProvenanceEdgeV1 {
        &self.edge
    }

    pub const fn kind(&self) -> CreativeLineageKindV1 {
        self.kind
    }

    pub fn context_refs(&self) -> &[CreativeContextRefV1] {
        &self.context_refs
    }

    pub fn contexts_of_kind(
        &self,
        kind: CreativeContextKindV1,
    ) -> impl Iterator<Item = &CreativeContextRefV1> {
        self.context_refs.iter().filter(move |item| item.kind == kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1, StewardedSubjectIdentityV1,
    };
    use mycelix_stewardship_provenance::{
        ProvenanceEvidenceRefV1, ProvenanceParticipantV1, ProvenanceParticipantRoleV1,
    };

    fn identity(id: &str, kind: RepresentationKindV1, fill: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:creative:work:1").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new(id).unwrap(),
            kind,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [fill; 32]),
        }
    }

    fn edge(relation: ProvenanceRelationV1) -> ProvenanceEdgeV1 {
        ProvenanceEdgeV1::new(
            CanonicalIdV1::new("provenance:edge:1").unwrap(),
            identity("representation:source", RepresentationKindV1::Score, 1),
            identity("representation:successor", RepresentationKindV1::Audio, 2),
            relation,
            vec![ProvenanceEvidenceRefV1::new("evidence:lineage:1").unwrap()],
            vec![
                ProvenanceParticipantV1::new(
                    "principal:participant:1",
                    ProvenanceParticipantRoleV1::PerformerAssertion,
                )
                .unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn performance_profile_requires_performed_from_provenance() {
        let record = CreativeLineageRecordV1::new(
            CanonicalIdV1::new("creative-lineage:performance:1").unwrap(),
            edge(ProvenanceRelationV1::PerformedFrom),
            CreativeLineageKindV1::Performance,
            vec![
                CreativeContextRefV1::new(
                    CreativeContextKindV1::PerformancePractice,
                    "context:practice:1",
                )
                .unwrap(),
            ],
        )
        .unwrap();
        assert_eq!(record.kind(), CreativeLineageKindV1::Performance);
    }

    #[test]
    fn profile_cannot_relabel_remix_as_translation() {
        let result = CreativeLineageRecordV1::new(
            CanonicalIdV1::new("creative-lineage:bad:1").unwrap(),
            edge(ProvenanceRelationV1::RemixedFrom),
            CreativeLineageKindV1::Translation,
            vec![],
        );
        assert_eq!(result, Err(CreativeLineageErrorV1::RelationKindMismatch));
    }

    #[test]
    fn arrangement_and_adaptation_are_explicit_derived_profiles() {
        for kind in [CreativeLineageKindV1::Arrangement, CreativeLineageKindV1::Adaptation] {
            let record = CreativeLineageRecordV1::new(
                CanonicalIdV1::new(format!("creative-lineage:{kind:?}")).unwrap(),
                edge(ProvenanceRelationV1::DerivedFrom),
                kind,
                vec![],
            )
            .unwrap();
            assert_eq!(record.edge().relation(), ProvenanceRelationV1::DerivedFrom);
        }
    }

    #[test]
    fn context_categories_remain_orthogonal_to_lineage() {
        let record = CreativeLineageRecordV1::new(
            CanonicalIdV1::new("creative-lineage:context:1").unwrap(),
            edge(ProvenanceRelationV1::RecordedFrom),
            CreativeLineageKindV1::Recording,
            vec![
                CreativeContextRefV1::new(CreativeContextKindV1::Tuning, "context:tuning:1").unwrap(),
                CreativeContextRefV1::new(CreativeContextKindV1::VenueOrSetting, "context:venue:1").unwrap(),
                CreativeContextRefV1::new(CreativeContextKindV1::CulturalContext, "context:culture:1").unwrap(),
            ],
        )
        .unwrap();
        assert_eq!(record.contexts_of_kind(CreativeContextKindV1::Tuning).count(), 1);
        assert_eq!(record.edge().relation(), ProvenanceRelationV1::RecordedFrom);
    }

    #[test]
    fn exact_duplicate_context_reference_is_rejected() {
        let context = CreativeContextRefV1::new(
            CreativeContextKindV1::Language,
            "context:language:1",
        )
        .unwrap();
        let result = CreativeLineageRecordV1::new(
            CanonicalIdV1::new("creative-lineage:duplicate:1").unwrap(),
            edge(ProvenanceRelationV1::TranslatedFrom),
            CreativeLineageKindV1::Translation,
            vec![context.clone(), context],
        );
        assert_eq!(result, Err(CreativeLineageErrorV1::DuplicateContextReference));
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(CREATIVE_LINEAGE_PROFILE_V1, "mycelix/creative-lineage/v1");
    }
}
