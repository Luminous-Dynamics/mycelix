// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-bearing typed provenance between exact representations.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1,
};

pub const PROVENANCE_EDGE_PROFILE_V1: &str = "mycelix/provenance-edge/v1";
pub const MAX_PROVENANCE_EVIDENCE_REFS_V1: usize = 32;
pub const MAX_PROVENANCE_PARTICIPANTS_V1: usize = 32;

/// Explicit successor relationship to a predecessor representation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ProvenanceRelationV1 {
    DerivedFrom,
    VersionedFrom,
    TranslatedFrom,
    RestoredFrom,
    DigitizedFrom,
    PerformedFrom,
    RemixedFrom,
    MigratedFrom,
    QuotedFrom,
    CopiedFrom,
    AnnotatedFrom,
    ExtractedFrom,
    RecordedFrom,
    Other,
}

/// Participant role is an assertion carried by the edge, not verified status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ProvenanceParticipantRoleV1 {
    CreatorAssertion,
    TranslatorAssertion,
    PerformerAssertion,
    RestorerAssertion,
    DigitizerAssertion,
    MigratorAssertion,
    RecorderAssertion,
    CuratorAssertion,
    EditorAssertion,
    OtherAssertion,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProvenanceEvidenceRefV1(CanonicalIdV1);

impl ProvenanceEvidenceRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ProvenanceParticipantV1 {
    pub principal: CanonicalIdV1,
    pub role: ProvenanceParticipantRoleV1,
}

impl ProvenanceParticipantV1 {
    pub fn new(
        principal: impl Into<String>,
        role: ProvenanceParticipantRoleV1,
    ) -> Result<Self, CanonicalIdErrorV1> {
        Ok(Self {
            principal: CanonicalIdV1::new(principal)?,
            role,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceEdgeErrorV1 {
    ExactSelfEdge,
    NoEvidenceReferences,
    TooManyEvidenceReferences,
    DuplicateEvidenceReference,
    TooManyParticipants,
    DuplicateParticipant,
}

impl fmt::Display for ProvenanceEdgeErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ExactSelfEdge => f.write_str("exact representation cannot be its own provenance predecessor"),
            Self::NoEvidenceReferences => f.write_str("provenance edge requires evidence references"),
            Self::TooManyEvidenceReferences => f.write_str("too many provenance evidence references for v1"),
            Self::DuplicateEvidenceReference => f.write_str("duplicate provenance evidence reference"),
            Self::TooManyParticipants => f.write_str("too many provenance participants for v1"),
            Self::DuplicateParticipant => f.write_str("duplicate provenance participant assertion"),
        }
    }
}

/// Evidence-bearing predecessor → successor provenance assertion.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProvenanceEdgeV1 {
    edge_id: CanonicalIdV1,
    predecessor: StewardedSubjectIdentityV1,
    successor: StewardedSubjectIdentityV1,
    relation: ProvenanceRelationV1,
    evidence_refs: Vec<ProvenanceEvidenceRefV1>,
    participants: Vec<ProvenanceParticipantV1>,
}

impl ProvenanceEdgeV1 {
    pub fn new(
        edge_id: CanonicalIdV1,
        predecessor: StewardedSubjectIdentityV1,
        successor: StewardedSubjectIdentityV1,
        relation: ProvenanceRelationV1,
        evidence_refs: Vec<ProvenanceEvidenceRefV1>,
        participants: Vec<ProvenanceParticipantV1>,
    ) -> Result<Self, ProvenanceEdgeErrorV1> {
        if predecessor == successor {
            return Err(ProvenanceEdgeErrorV1::ExactSelfEdge);
        }
        if evidence_refs.is_empty() {
            return Err(ProvenanceEdgeErrorV1::NoEvidenceReferences);
        }
        if evidence_refs.len() > MAX_PROVENANCE_EVIDENCE_REFS_V1 {
            return Err(ProvenanceEdgeErrorV1::TooManyEvidenceReferences);
        }
        for (index, reference) in evidence_refs.iter().enumerate() {
            if evidence_refs[..index].contains(reference) {
                return Err(ProvenanceEdgeErrorV1::DuplicateEvidenceReference);
            }
        }
        if participants.len() > MAX_PROVENANCE_PARTICIPANTS_V1 {
            return Err(ProvenanceEdgeErrorV1::TooManyParticipants);
        }
        for (index, participant) in participants.iter().enumerate() {
            if participants[..index].contains(participant) {
                return Err(ProvenanceEdgeErrorV1::DuplicateParticipant);
            }
        }

        Ok(Self {
            edge_id,
            predecessor,
            successor,
            relation,
            evidence_refs,
            participants,
        })
    }

    pub fn edge_id(&self) -> &CanonicalIdV1 {
        &self.edge_id
    }

    pub fn predecessor(&self) -> &StewardedSubjectIdentityV1 {
        &self.predecessor
    }

    pub fn successor(&self) -> &StewardedSubjectIdentityV1 {
        &self.successor
    }

    pub const fn relation(&self) -> ProvenanceRelationV1 {
        self.relation
    }

    pub fn evidence_refs(&self) -> &[ProvenanceEvidenceRefV1] {
        &self.evidence_refs
    }

    pub fn participants(&self) -> &[ProvenanceParticipantV1] {
        &self.participants
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1,
    };

    fn identity(rep: &str, digest_byte: u8) -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:work:example").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new(rep).unwrap(),
            kind: RepresentationKindV1::Text,
            content_digest: ContentDigestV1::new(
                DigestAlgorithmV1::Blake3_256,
                [digest_byte; 32],
            ),
        }
    }

    fn evidence(id: &str) -> ProvenanceEvidenceRefV1 {
        ProvenanceEvidenceRefV1::new(id).unwrap()
    }

    #[test]
    fn exact_self_edge_is_rejected() {
        let same = identity("representation:1", 1);
        let result = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:1").unwrap(),
            same.clone(),
            same,
            ProvenanceRelationV1::CopiedFrom,
            vec![evidence("evidence:1")],
            vec![],
        );
        assert_eq!(result, Err(ProvenanceEdgeErrorV1::ExactSelfEdge));
    }

    #[test]
    fn provenance_requires_evidence() {
        let result = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:2").unwrap(),
            identity("representation:source", 1),
            identity("representation:target", 2),
            ProvenanceRelationV1::TranslatedFrom,
            vec![],
            vec![],
        );
        assert_eq!(result, Err(ProvenanceEdgeErrorV1::NoEvidenceReferences));
    }

    #[test]
    fn translation_is_explicit_not_inferred_from_representation() {
        let edge = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:translation:1").unwrap(),
            identity("representation:english", 1),
            identity("representation:zulu", 2),
            ProvenanceRelationV1::TranslatedFrom,
            vec![evidence("evidence:translator-record:1")],
            vec![],
        )
        .unwrap();
        assert_eq!(edge.relation(), ProvenanceRelationV1::TranslatedFrom);
    }

    #[test]
    fn participant_roles_remain_assertions() {
        let participant = ProvenanceParticipantV1::new(
            "principal:translator:1",
            ProvenanceParticipantRoleV1::TranslatorAssertion,
        )
        .unwrap();
        let edge = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:3").unwrap(),
            identity("representation:source", 1),
            identity("representation:target", 2),
            ProvenanceRelationV1::TranslatedFrom,
            vec![evidence("evidence:1")],
            vec![participant.clone()],
        )
        .unwrap();
        assert_eq!(edge.participants(), &[participant]);
    }

    #[test]
    fn duplicate_evidence_and_participants_are_rejected() {
        let ev = evidence("evidence:1");
        let result = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:4").unwrap(),
            identity("representation:a", 1),
            identity("representation:b", 2),
            ProvenanceRelationV1::DerivedFrom,
            vec![ev.clone(), ev],
            vec![],
        );
        assert_eq!(result, Err(ProvenanceEdgeErrorV1::DuplicateEvidenceReference));

        let participant = ProvenanceParticipantV1::new(
            "principal:1",
            ProvenanceParticipantRoleV1::CuratorAssertion,
        )
        .unwrap();
        let result = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:5").unwrap(),
            identity("representation:a", 1),
            identity("representation:b", 2),
            ProvenanceRelationV1::RestoredFrom,
            vec![evidence("evidence:2")],
            vec![participant.clone(), participant],
        );
        assert_eq!(result, Err(ProvenanceEdgeErrorV1::DuplicateParticipant));
    }

    #[test]
    fn migration_preserves_both_exact_endpoints() {
        let source = identity("representation:legacy", 7);
        let target = identity("representation:migrated", 8);
        let edge = ProvenanceEdgeV1::new(
            CanonicalIdV1::new("edge:migration:1").unwrap(),
            source.clone(),
            target.clone(),
            ProvenanceRelationV1::MigratedFrom,
            vec![evidence("evidence:migration-log:1")],
            vec![],
        )
        .unwrap();
        assert_eq!(edge.predecessor(), &source);
        assert_eq!(edge.successor(), &target);
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(PROVENANCE_EDGE_PROFILE_V1, "mycelix/provenance-edge/v1");
    }
}
