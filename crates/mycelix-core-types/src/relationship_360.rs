//! Deterministic Relationship 360 read model.
//!
//! This is a rebuildable projection, never an authority source. Raw records
//! cross an explicit qualification boundary before projection.

use crate::{
    AssertionEnvelope, CommitmentId, CommitmentStatus, Currentness, DelegationId,
    ParticipantRef, QualifiedRelationshipInputs, QualificationDependency,
    QualificationDependencyResolver, RelationshipId, RelationshipRecord,
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
    pub id: crate::ConsentId,
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
    /// Exact external dependencies used during qualification.
    pub dependencies: Vec<QualificationDependency>,
}

impl Relationship360 {
    pub const SCHEMA_VERSION: u16 = 2;

    /// Exact dependencies used to construct the projection, as recorded by
    /// the qualification certificate. This is evidence, not authority.
    pub fn qualified_dependency_manifest(&self) -> &[QualificationDependency] {
        &self.dependencies
    }

    /// Qualify raw source records and then project them.
    ///
    /// The projection itself never filters unrelated or invalid authority
    /// records. They must cross the explicit qualification boundary first.
    pub fn build(
        relationship: &RelationshipRecord,
        projection_revision: u64,
        assertions: &[RelationshipAssertion],
        frontiers: &[crate::SourceFrontier],
        commitments: &[crate::Commitment],
        consents: &[crate::Consent],
        delegations: &[crate::Delegation],
    ) -> Result<Self, Relationship360Error> {
        let qualified = QualifiedRelationshipInputs::qualify(
            relationship,
            assertions,
            frontiers,
            commitments,
            consents,
            delegations,
        )
        .map_err(Relationship360Error::Qualification)?;
        Self::build_qualified(relationship, projection_revision, &qualified)
    }

    /// Resolve explicit external dependencies through a version-specific adapter
    /// before qualification and projection. The projection retains the exact
    /// dependency manifest in the qualification certificate.
    pub fn build_with_resolver<R: QualificationDependencyResolver>(
        relationship: &RelationshipRecord,
        projection_revision: u64,
        assertions: &[RelationshipAssertion],
        frontiers: &[crate::SourceFrontier],
        commitments: &[crate::Commitment],
        consents: &[crate::Consent],
        delegations: &[crate::Delegation],
        dependencies: &[QualificationDependency],
        resolver: &R,
    ) -> Result<Self, Relationship360Error> {
        let qualified = QualifiedRelationshipInputs::qualify_with_resolver(
            relationship,
            assertions,
            frontiers,
            commitments,
            consents,
            delegations,
            dependencies,
            resolver,
        )
        .map_err(Relationship360Error::Qualification)?;
        Self::build_qualified(relationship, projection_revision, &qualified)
    }

    /// Projection-only entry point. Its input type proves that qualification
    /// has already happened.
    pub fn build_qualified(
        relationship: &RelationshipRecord,
        projection_revision: u64,
        qualified: &QualifiedRelationshipInputs,
    ) -> Result<Self, Relationship360Error> {
        relationship
            .validate_schema()
            .map_err(|error| Relationship360Error::Qualification(
                crate::QualificationError::RelationshipSchema(error)
            ))?;

        if qualified.relationship_id != relationship.relationship_id {
            return Err(Relationship360Error::QualifiedRelationshipMismatch);
        }

        qualified
            .validate_certificate()
            .map_err(Relationship360Error::Qualification)?;

        let mut participants = relationship.participants.clone();
        participants.sort();

        let mut assertion_summaries = qualified
            .assertions
            .iter()
            .map(|x| {
                let frontier = qualified
                    .frontiers
                    .iter()
                    .find(|f| f.source == x.envelope.evidence.source_revision.source);
                AssertionSummary {
                    assertion_id: x.envelope.assertion_id,
                    evidence: x.envelope.evidence.clone(),
                    status: x.envelope.status,
                    currentness: x.envelope.currentness(frontier),
                }
            })
            .collect::<Vec<_>>();
        assertion_summaries.sort_by(|a, b| {
            a.assertion_id
                .cmp(&b.assertion_id)
                .then_with(|| a.evidence.cmp(&b.evidence))
        });

        let commitments = qualified
            .commitments
            .iter()
            .map(|x| CommitmentSummary { id: x.id, status: x.status })
            .collect();

        let consents = qualified
            .consents
            .iter()
            .map(|x| ConsentSummary {
                id: x.id,
                status: x.status,
                authority_epoch: x.authority_epoch,
            })
            .collect();

        let delegations = qualified
            .delegations
            .iter()
            .map(|x| DelegationSummary {
                id: x.id,
                status: x.status,
                mode: x.mode,
                authority_epoch: x.authority_epoch,
            })
            .collect();

        Ok(Self {
            schema_version: Self::SCHEMA_VERSION,
            relationship_id: relationship.relationship_id,
            relationship_revision: relationship.revision,
            projection_revision,
            participants,
            assertions: assertion_summaries,
            commitments,
            consents,
            delegations,
            dependencies: qualified.dependencies.clone(),
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
        out.extend_from_slice(b"mycelix.relationship-360.v2\0");
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

        out.extend_from_slice(&(self.dependencies.len() as u64).to_le_bytes());
        for dependency in &self.dependencies {
            dependency.canonical_bytes(&mut out);
        }

        out
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Relationship360Error {
    UnsupportedSchema(u16),
    Qualification(crate::QualificationError),
    QualifiedRelationshipMismatch,
}

fn bytes(out: &mut Vec<u8>, value: &[u8]) {
    out.extend_from_slice(&(value.len() as u64).to_le_bytes());
    out.extend_from_slice(value);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AssertionId, AssertionSource, EvidenceDigest, EvidenceRef, EpistemicStatus,
        ObservationTime, SourceFrontier, SourceRevision, Visibility,
    };

    fn p(ns: &str, id: &str) -> ParticipantRef {
        ParticipantRef::new(ns, id)
    }

    fn relationship(seed: &[u8]) -> RelationshipRecord {
        RelationshipRecord::create(
            RelationshipId::derive("test", seed),
            p("did", "alice"),
            vec![p("did", "alice"), p("org", "acme")],
        )
        .unwrap()
    }

    fn assertion(r: RelationshipId, source: &str, revision: u64, digest: u8) -> RelationshipAssertion {
        let evidence = EvidenceRef::new(
            SourceRevision::new(source, revision).unwrap(),
            "record/1",
            EvidenceDigest::from_bytes([digest; 32]),
        )
        .unwrap();
        let subject = p("did", "alice");
        RelationshipAssertion {
            relationship_id: r,
            envelope: crate::AssertionEnvelope::new(
                AssertionId::derive("relationship", &subject, "status", "active"),
                subject,
                "status",
                "active",
                EpistemicStatus::Observed,
                ObservationTime::from_unix_seconds(1),
                AssertionSource {
                    source_revision: evidence.source_revision.clone(),
                    visibility: Visibility::Relationship,
                },
                evidence,
            )
            .unwrap(),
        }
    }

    #[test]
    fn projection_requires_qualification() {
        let r = relationship(b"r");
        let a = assertion(r.relationship_id, "crm", 8, 1);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        assert!(matches!(
            Relationship360::build(&r, 1, &[a], &[frontier], &[], &[], &[]),
            Err(Relationship360Error::Qualification(
                crate::QualificationError::FutureEvidenceRevision { .. }
            ))
        ));
    }

    #[test]
    fn qualified_projection_is_deterministic() {
        let r = relationship(b"r");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = assertion(r.relationship_id, "crm", 7, 2);
        let frontier = SourceFrontier::new("crm", 7).unwrap();

        let q1 = crate::QualifiedRelationshipInputs::qualify(
            &r, &[a.clone(), b.clone()], &[frontier.clone()], &[], &[], &[]
        ).unwrap();
        let q2 = crate::QualifiedRelationshipInputs::qualify(
            &r, &[b, a], &[frontier], &[], &[], &[]
        ).unwrap();

        let left = Relationship360::build_qualified(&r, 1, &q1).unwrap();
        let right = Relationship360::build_qualified(&r, 1, &q2).unwrap();
        assert_eq!(left.canonical_bytes(), right.canonical_bytes());
    }

    #[test]
    fn multiple_evidence_records_for_one_assertion_are_preserved() {
        let r = relationship(b"r");
        let a1 = assertion(r.relationship_id, "crm", 7, 1);
        let a2 = assertion(r.relationship_id, "crm", 7, 2);
        let frontier = SourceFrontier::new("crm", 7).unwrap();

        let view = Relationship360::build(
            &r, 1, &[a1, a2], &[frontier], &[], &[], &[]
        ).unwrap();

        assert_eq!(view.assertions.len(), 2);
        assert_ne!(view.assertions[0].evidence, view.assertions[1].evidence);
        assert_eq!(view.assertions[0].assertion_id, view.assertions[1].assertion_id);
    }

    #[test]
    fn duplicate_frontiers_are_rejected() {
        let r = relationship(b"r");
        let frontiers = vec![
            SourceFrontier::new("crm", 7).unwrap(),
            SourceFrontier::new("crm", 8).unwrap(),
        ];
        assert!(matches!(
            Relationship360::build(&r, 1, &[], &frontiers, &[], &[], &[]),
            Err(Relationship360Error::Qualification(
                crate::QualificationError::DuplicateFrontier { .. }
            ))
        ));
    }

    #[test]
    fn resolver_qualified_projection_retains_dependency_manifest() {
        let r = relationship(b"resolver-projection");
        let id = [42u8; 32];
        let dependency = QualificationDependency::new(
            crate::QualificationDependencyKind::Commitment,
            id,
            b"action-hash/42".to_vec(),
        ).unwrap();
        let projection = Relationship360::build_with_resolver(
            &r,
            1,
            &[],
            &[],
            &[],
            &[],
            &[],
            &[dependency.clone()],
            &|requested| crate::QualificationDependencyResolution::Attested {
                observed: crate::QualificationDependencyObservation::from_dependency(requested),
            },
        ).unwrap();
        assert_eq!(projection.qualified_dependency_manifest(), &[dependency]);
    }

    #[test]
    fn unrelated_commitments_are_rejected_instead_of_filtered() {
        let r = relationship(b"r");
        let other = relationship(b"other");
        let c = crate::Commitment::new_content_bound(
            other.relationship_id,
            p("did", "alice"),
            p("org", "acme"),
            None,
            1,
            p("did", "alice"),
            None,
        ).unwrap();

        assert!(matches!(
            Relationship360::build(&r, 1, &[], &[], &[c], &[], &[]),
            Err(Relationship360Error::Qualification(
                crate::QualificationError::UnrelatedCommitment { .. }
            ))
        ));
    }
}
