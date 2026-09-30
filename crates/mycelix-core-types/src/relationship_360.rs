//! Deterministic Relationship 360 read model.
//!
//! This is a rebuildable projection, never an authority source. All inputs,
//! including the source frontier and projection revision, are explicit.

use crate::{
    AssertionEnvelope, Commitment, CommitmentId, CommitmentStatus, Consent, ConsentId,
    Currentness, Delegation, DelegationId, ParticipantRef, RelationshipId, RelationshipRecord,
    SourceFrontier,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RelationshipAssertion {
    pub relationship_id: RelationshipId,
    pub envelope: AssertionEnvelope,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssertionSummary {
    pub assertion_id: crate::AssertionId,
    pub evidence: crate::EvidenceRef,
    pub status: crate::EpistemicStatus,
    pub currentness: Currentness,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommitmentSummary {
    pub id: CommitmentId,
    pub status: CommitmentStatus,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConsentSummary {
    pub id: ConsentId,
    pub status: crate::ConsentStatus,
    pub authority_epoch: crate::AuthorityEpoch,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DelegationSummary {
    pub id: DelegationId,
    pub status: crate::DelegationStatus,
    pub mode: crate::DelegationMode,
    pub authority_epoch: crate::AuthorityEpoch,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Relationship360 {
    pub schema_version: u16,
    pub relationship_id: RelationshipId,
    pub relationship_revision: u64,
    pub projection_revision: u64,
    pub participants: Vec<ParticipantRef>,
    pub assertions: Vec<AssertionSummary>,
    pub commitments: Vec<CommitmentSummary>,
    pub consents: Vec<ConsentSummary>,
    pub delegations: Vec<DelegationSummary>,
}

impl Relationship360 {
    pub const SCHEMA_VERSION: u16 = 1;

    pub fn build(
        relationship: &RelationshipRecord,
        projection_revision: u64,
        assertions: &[RelationshipAssertion],
        frontiers: &[SourceFrontier],
        commitments: &[Commitment],
        consents: &[Consent],
        delegations: &[Delegation],
    ) -> Result<Self, Relationship360Error> {
        validate_frontiers(frontiers)?;

        let mut assertion_summaries = assertions.iter()
            .filter(|x| x.relationship_id == relationship.relationship_id)
            .map(|x| {
                let frontier = frontiers.iter()
                    .find(|f| f.source == x.envelope.evidence.source_revision.source);
                AssertionSummary {
                    assertion_id: x.envelope.assertion_id,
                    evidence: x.envelope.evidence.clone(),
                    status: x.envelope.status,
                    currentness: x.envelope.currentness(frontier),
                }
            }).collect::<Vec<_>>();
        assertion_summaries.sort_by(|a, b| {
            a.assertion_id
                .cmp(&b.assertion_id)
                .then_with(|| a.evidence.cmp(&b.evidence))
        });

        let mut commitment_summaries = commitments.iter()
            .filter(|x| x.relationship_id == relationship.relationship_id)
            .map(|x| CommitmentSummary { id: x.id, status: x.status })
            .collect::<Vec<_>>();
        commitment_summaries.sort_by_key(|x| x.id);

        let mut consent_summaries = consents.iter()
            .filter(|x| x.relationship_id == relationship.relationship_id)
            .map(|x| ConsentSummary {
                id: x.id, status: x.status, authority_epoch: x.authority_epoch
            }).collect::<Vec<_>>();
        consent_summaries.sort_by_key(|x| x.id);

        let mut delegation_summaries = delegations.iter()
            .filter(|x| x.relationship_id == relationship.relationship_id)
            .map(|x| DelegationSummary {
                id: x.id, status: x.status, mode: x.mode,
                authority_epoch: x.authority_epoch
            }).collect::<Vec<_>>();
        delegation_summaries.sort_by_key(|x| x.id);

        let mut participants = relationship.participants.clone();
        participants.sort();

        Ok(Self {
            schema_version: Self::SCHEMA_VERSION,
            relationship_id: relationship.relationship_id,
            relationship_revision: relationship.revision,
            projection_revision,
            participants,
            assertions: assertion_summaries,
            commitments: commitment_summaries,
            consents: consent_summaries,
            delegations: delegation_summaries,
        })
    }

    pub fn validate_schema(&self) -> Result<(), Relationship360Error> {
        if self.schema_version == Self::SCHEMA_VERSION {
            Ok(())
        } else {
            Err(Relationship360Error::UnsupportedSchema(self.schema_version))
        }
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.relationship-360.v1\0");
        out.extend_from_slice(&self.schema_version.to_le_bytes());
        out.extend_from_slice(self.relationship_id.as_bytes());
        out.extend_from_slice(&self.relationship_revision.to_le_bytes());
        out.extend_from_slice(&self.projection_revision.to_le_bytes());
        out.extend_from_slice(&(self.participants.len() as u64).to_le_bytes());
        for p in &self.participants {
            bytes(&mut out, p.namespace.as_bytes());
            bytes(&mut out, p.identifier.as_bytes());
        }
        out.extend_from_slice(&(self.assertions.len() as u64).to_le_bytes());
        for x in &self.assertions {
            out.extend_from_slice(x.assertion_id.as_bytes());
            out.extend_from_slice(&x.evidence.canonical_bytes());
            out.push(x.status as u8);
            out.push(x.currentness as u8);
        }
        out.extend_from_slice(&(self.commitments.len() as u64).to_le_bytes());
        for x in &self.commitments {
            out.extend_from_slice(x.id.as_bytes());
            out.push(x.status as u8);
        }
        out.extend_from_slice(&(self.consents.len() as u64).to_le_bytes());
        for x in &self.consents {
            out.extend_from_slice(x.id.as_bytes());
            out.push(x.status as u8);
            out.extend_from_slice(&x.authority_epoch.to_le_bytes());
        }
        out.extend_from_slice(&(self.delegations.len() as u64).to_le_bytes());
        for x in &self.delegations {
            out.extend_from_slice(x.id.as_bytes());
            out.push(x.status as u8);
            out.push(x.mode as u8);
            out.extend_from_slice(&x.authority_epoch.to_le_bytes());
        }
        out
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Relationship360Error {
    UnsupportedSchema(u16),
    DuplicateFrontier { source: String },
}

fn validate_frontiers(frontiers: &[SourceFrontier]) -> Result<(), Relationship360Error> {
    let mut sources = frontiers.iter().map(|f| f.source.as_str()).collect::<Vec<_>>();
    sources.sort_unstable();
    for pair in sources.windows(2) {
        if pair[0] == pair[1] {
            return Err(Relationship360Error::DuplicateFrontier {
                source: pair[0].to_owned(),
            });
        }
    }
    Ok(())
}

fn bytes(out: &mut Vec<u8>, value: &[u8]) {
    out.extend_from_slice(&(value.len() as u64).to_le_bytes());
    out.extend_from_slice(value);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(ns: &str, id: &str) -> ParticipantRef {
        ParticipantRef::new(ns, id)
    }

    fn relationship(seed: &[u8]) -> RelationshipRecord {
        RelationshipRecord::create(
            RelationshipId::derive("test", seed),
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        ).unwrap()
    }

    #[test]
    fn missing_frontier_is_unknown() {
        let r = relationship(b"r");
        let subject = p("did", "alice");
        let e = crate::EvidenceRef::new(
            crate::SourceRevision::new("crm", 7).unwrap(),
            "record/1",
            crate::EvidenceDigest::from_bytes([7; 32]),
        ).unwrap();
        let a = RelationshipAssertion {
            relationship_id: r.relationship_id,
            envelope: crate::AssertionEnvelope::new(
                crate::AssertionId::derive("relationship", &subject, "status", "active"),
                subject, "status", "active", crate::EpistemicStatus::Observed,
                crate::ObservationTime::from_unix_seconds(1),
                crate::AssertionSource {
                    source_revision: e.source_revision.clone(),
                    visibility: crate::Visibility::Relationship,
                }, e
            ).unwrap(),
        };
        let view = Relationship360::build(&r, 1, &[a], &[], &[], &[], &[]).unwrap();
        assert_eq!(view.assertions[0].currentness, Currentness::Unknown);
    }

    #[test]
    fn input_order_does_not_change_projection() {
        let r = relationship(b"r");
        let a = Relationship360::build(&r, 1, &[], &[], &[], &[], &[]).unwrap();
        let b = Relationship360::build(&r, 1, &[], &[], &[], &[], &[]).unwrap();
        assert_eq!(a.canonical_bytes(), b.canonical_bytes());
    }

    #[test]
    fn duplicate_frontiers_are_rejected() {
        let r = relationship(b"r");
        let frontiers = vec![
            crate::SourceFrontier::new("crm", 7).unwrap(),
            crate::SourceFrontier::new("crm", 8).unwrap(),
        ];
        assert_eq!(
            Relationship360::build(&r, 1, &[], &frontiers, &[], &[], &[]),
            Err(Relationship360Error::DuplicateFrontier { source: "crm".into() })
        );
    }

    #[test]
    fn multiple_evidence_records_for_one_assertion_are_preserved() {
        let r = relationship(b"r");
        let subject = p("did", "alice");
        let ev1 = crate::EvidenceRef::new(
            crate::SourceRevision::new("crm", 7).unwrap(),
            "record/1",
            crate::EvidenceDigest::from_bytes([1; 32]),
        ).unwrap();
        let ev2 = crate::EvidenceRef::new(
            crate::SourceRevision::new("erp", 12).unwrap(),
            "account/9",
            crate::EvidenceDigest::from_bytes([2; 32]),
        ).unwrap();

        let a1 = RelationshipAssertion {
            relationship_id: r.relationship_id,
            envelope: crate::AssertionEnvelope::new(
                crate::AssertionId::derive("relationship", &subject, "status", "active"),
                subject.clone(), "status", "active", crate::EpistemicStatus::Observed,
                crate::ObservationTime::from_unix_seconds(1),
                crate::AssertionSource {
                    source_revision: ev1.source_revision.clone(),
                    visibility: crate::Visibility::Relationship,
                }, ev1,
            ).unwrap(),
        };
        let a2 = RelationshipAssertion {
            relationship_id: r.relationship_id,
            envelope: crate::AssertionEnvelope::new(
                a1.envelope.assertion_id,
                subject, "status", "active", crate::EpistemicStatus::Reported,
                crate::ObservationTime::from_unix_seconds(2),
                crate::AssertionSource {
                    source_revision: ev2.source_revision.clone(),
                    visibility: crate::Visibility::Relationship,
                }, ev2,
            ).unwrap(),
        };

        let view = Relationship360::build(&r, 1, &[a1, a2], &[], &[], &[], &[]).unwrap();
        assert_eq!(view.assertions.len(), 2);
        assert_ne!(view.assertions[0].evidence, view.assertions[1].evidence);
        assert_eq!(view.assertions[0].assertion_id, view.assertions[1].assertion_id);
    }

    #[test]
    fn participant_order_is_canonical() {
        let relationship = RelationshipRecord::create(
            RelationshipId::derive("test", b"unordered"),
            p("did", "alice"),
            vec![p("org", "acme"), p("did", "alice")],
        ).unwrap();
        let view = Relationship360::build(&relationship, 1, &[], &[], &[], &[], &[]).unwrap();
        assert_eq!(view.participants, vec![p("did", "alice"), p("org", "acme")]);
    }

    #[test]
    fn unrelated_commitments_are_excluded() {
        let r = relationship(b"r");
        let other = relationship(b"other");
        let c = crate::Commitment::new(
            crate::CommitmentId::derive(other.relationship_id, b"c"),
            other.relationship_id, p("did", "alice"), p("org", "acme"),
            None, 1, p("did", "alice"), None
        ).unwrap();
        let view = Relationship360::build(&r, 1, &[], &[], &[c], &[], &[]).unwrap();
        assert!(view.commitments.is_empty());
    }
}
