//! Explicit qualification boundary for Relationship 360.
//!
//! Qualification is deliberately separate from projection. Raw records are
//! checked for semantic consistency, authorization invariants, evidence
//! grounding, and dependency integrity before they can enter the read model.
//! No qualification result is ambient authority: it is derived only from the
//! supplied values and their explicit dependencies.

use crate::{
    AssertionEnvelope, Commitment, CommitmentError, CommitmentEventKind, Consent,
    ConsentEventKind, ConsentStatus, Delegation, DelegationContext, DelegationError,
    DelegationStatus, ExecutionRequest, RelationshipAssertion, RelationshipId,
    RelationshipRecord, SourceFrontier, AssertionId, EvidenceRef, CommitmentId, ConsentId,
    DelegationId,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QualificationCertificate {
    pub schema_version: u16,
    pub input_digest: [u8; 32],
    pub assertion_evidence: Vec<(AssertionId, EvidenceRef)>,
    pub commitment_ids: Vec<CommitmentId>,
    pub consent_ids: Vec<ConsentId>,
    pub delegation_ids: Vec<DelegationId>,
    pub frontiers: Vec<SourceFrontier>,
}

impl QualificationCertificate {
    pub const SCHEMA_VERSION: u16 = 1;

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.qualification-certificate.v1\0");
        out.extend_from_slice(&self.schema_version.to_le_bytes());
        out.extend_from_slice(&self.input_digest);
        write_len(&mut out, self.assertion_evidence.len());
        for (id, evidence) in &self.assertion_evidence {
            out.extend_from_slice(id.as_bytes());
            write_bytes(&mut out, &evidence.canonical_bytes());
        }
        write_len(&mut out, self.commitment_ids.len());
        for id in &self.commitment_ids { out.extend_from_slice(id.as_bytes()); }
        write_len(&mut out, self.consent_ids.len());
        for id in &self.consent_ids { out.extend_from_slice(id.as_bytes()); }
        write_len(&mut out, self.delegation_ids.len());
        for id in &self.delegation_ids { out.extend_from_slice(id.as_bytes()); }
        write_len(&mut out, self.frontiers.len());
        for frontier in &self.frontiers {
            write_bytes(&mut out, frontier.source.as_bytes());
            out.extend_from_slice(&frontier.observed_revision.to_le_bytes());
        }
        out
    }

    pub fn digest(&self) -> [u8; 32] {
        *blake3::hash(&self.canonical_bytes()).as_bytes()
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct QualifiedRelationshipInputs {
    pub relationship_id: RelationshipId,
    pub assertions: Vec<RelationshipAssertion>,
    pub frontiers: Vec<SourceFrontier>,
    pub commitments: Vec<Commitment>,
    pub consents: Vec<Consent>,
    pub delegations: Vec<Delegation>,
    /// Deterministic evidence of the exact qualified dependency set.
    /// This is not authority and must never be treated as a bearer capability.
    pub certificate: QualificationCertificate,
}

impl QualifiedRelationshipInputs {
    pub fn validate_certificate(&self) -> Result<(), QualificationError> {
        let expected = qualification_input_digest(
            self.relationship_id,
            &self.assertions,
            &self.frontiers,
            &self.commitments,
            &self.consents,
            &self.delegations,
        );
        if self.certificate.schema_version != QualificationCertificate::SCHEMA_VERSION
            || self.certificate.input_digest != expected
            || self.certificate.assertion_evidence
                != self.assertions.iter()
                    .map(|a| (a.envelope.assertion_id, a.envelope.evidence.clone()))
                    .collect::<Vec<_>>()
            || self.certificate.commitment_ids != self.commitments.iter().map(|c| c.id).collect::<Vec<_>>()
            || self.certificate.consent_ids != self.consents.iter().map(|c| c.id).collect::<Vec<_>>()
            || self.certificate.delegation_ids != self.delegations.iter().map(|d| d.id).collect::<Vec<_>>()
            || self.certificate.frontiers != self.frontiers
        {
            return Err(QualificationError::CertificateMismatch);
        }
        Ok(())
    }

    pub fn qualify(
        relationship: &RelationshipRecord,
        assertions: &[RelationshipAssertion],
        frontiers: &[SourceFrontier],
        commitments: &[Commitment],
        consents: &[Consent],
        delegations: &[Delegation],
    ) -> Result<Self, QualificationError> {
        relationship
            .validate_schema()
            .map_err(QualificationError::RelationshipSchema)?;

        validate_frontiers(frontiers)?;

        let mut ordered_assertions = assertions.to_vec();
        ordered_assertions.sort_by(|a, b| {
            a.envelope
                .assertion_id
                .cmp(&b.envelope.assertion_id)
                .then_with(|| a.envelope.evidence.cmp(&b.envelope.evidence))
        });
        validate_assertions(relationship.relationship_id, &ordered_assertions, frontiers)?;

        let mut ordered_commitments = commitments.to_vec();
        ordered_commitments.sort_by_key(|c| c.id);
        validate_commitments(relationship.relationship_id, &ordered_commitments)?;

        let mut ordered_consents = consents.to_vec();
        ordered_consents.sort_by_key(|c| c.id);
        validate_consents(relationship.relationship_id, &ordered_consents)?;

        let mut ordered_delegations = delegations.to_vec();
        ordered_delegations.sort_by_key(|d| d.id);
        validate_delegations(relationship.relationship_id, &ordered_delegations)?;

        let mut canonical_frontiers = frontiers.to_vec();
        canonical_frontiers.sort_by(|a, b| a.source.cmp(&b.source));

        let certificate = QualificationCertificate {
            schema_version: QualificationCertificate::SCHEMA_VERSION,
            input_digest: qualification_input_digest(
                relationship.relationship_id,
                &ordered_assertions,
                &canonical_frontiers,
                &ordered_commitments,
                &ordered_consents,
                &ordered_delegations,
            ),
            assertion_evidence: ordered_assertions
                .iter()
                .map(|a| (a.envelope.assertion_id, a.envelope.evidence.clone()))
                .collect(),
            commitment_ids: ordered_commitments.iter().map(|c| c.id).collect(),
            consent_ids: ordered_consents.iter().map(|c| c.id).collect(),
            delegation_ids: ordered_delegations.iter().map(|d| d.id).collect(),
            frontiers: canonical_frontiers.clone(),
        };

        Ok(Self {
            relationship_id: relationship.relationship_id,
            assertions: ordered_assertions,
            frontiers: canonical_frontiers,
            commitments: ordered_commitments,
            consents: ordered_consents,
            delegations: ordered_delegations,
            certificate,
        })
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QualificationError {
    RelationshipSchema(crate::RelationshipError),
    DuplicateFrontier { source: String },
    UnrelatedAssertion { relationship_id: RelationshipId },
    AssertionSchema(crate::EvidenceError),
    AssertionEvidenceMismatch { assertion_id: crate::AssertionId },
    MissingEvidenceFrontier { source: String },
    FutureEvidenceRevision {
        assertion_id: crate::AssertionId,
        revision: u64,
        frontier: u64,
    },
    DuplicateAssertionEvidence {
        assertion_id: crate::AssertionId,
        evidence: crate::EvidenceRef,
    },
    UnrelatedCommitment { id: crate::CommitmentId },
    DuplicateCommitmentId { id: crate::CommitmentId },
    InvalidCommitment { id: crate::CommitmentId, error: crate::CommitmentError },
    CommitmentActorMismatch {
        id: crate::CommitmentId,
        event: CommitmentEventKind,
    },
    UnrelatedConsent { id: crate::ConsentId },
    DuplicateConsentId { id: crate::ConsentId },
    InvalidConsent { id: crate::ConsentId, reason: ConsentQualificationError },
    UnrelatedDelegation { id: crate::DelegationId },
    DuplicateDelegationId { id: crate::DelegationId },
    InvalidDelegation { id: crate::DelegationId, error: DelegationError },
    CertificateMismatch,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ConsentQualificationError {
    EmptyEvents,
    FirstEventNotGrant,
    GrantActorMismatch,
    EventActorMismatch { event: ConsentEventKind },
    StaleRevocationEpoch,
    RevocationStateMismatch,
    ExpiryStateMismatch,
    GrantEpochMismatch,
}

fn write_len(out: &mut Vec<u8>, len: usize) {
    out.extend_from_slice(&(len as u64).to_le_bytes());
}

fn write_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    write_len(out, bytes.len());
    out.extend_from_slice(bytes);
}

fn qualification_input_digest(
    relationship_id: RelationshipId,
    assertions: &[RelationshipAssertion],
    frontiers: &[SourceFrontier],
    commitments: &[Commitment],
    consents: &[Consent],
    delegations: &[Delegation],
) -> [u8; 32] {
    let mut out = Vec::new();
    out.extend_from_slice(b"mycelix.qualification-inputs.v1\0");
    out.extend_from_slice(relationship_id.as_bytes());

    write_len(&mut out, assertions.len());
    for assertion in assertions {
        out.extend_from_slice(assertion.envelope.assertion_id.as_bytes());
        write_bytes(&mut out, &assertion.envelope.evidence.canonical_bytes());
        out.extend_from_slice(&assertion.envelope.schema_version.to_le_bytes());
        out.extend_from_slice(&assertion.envelope.observed_at.as_unix_seconds().to_le_bytes());
    }

    write_len(&mut out, frontiers.len());
    for frontier in frontiers {
        write_bytes(&mut out, frontier.source.as_bytes());
        out.extend_from_slice(&frontier.observed_revision.to_le_bytes());
    }

    write_len(&mut out, commitments.len());
    for commitment in commitments {
        write_bytes(&mut out, &commitment.state_canonical_bytes());
    }

    write_len(&mut out, consents.len());
    for consent in consents {
        write_bytes(&mut out, &consent.state_canonical_bytes());
    }

    write_len(&mut out, delegations.len());
    for delegation in delegations {
        write_bytes(&mut out, &delegation.state_canonical_bytes());
    }

    *blake3::hash(&out).as_bytes()
}

fn validate_frontiers(frontiers: &[SourceFrontier]) -> Result<(), QualificationError> {
    let mut sources = frontiers.iter().map(|f| f.source.as_str()).collect::<Vec<_>>();
    sources.sort_unstable();
    for pair in sources.windows(2) {
        if pair[0] == pair[1] {
            return Err(QualificationError::DuplicateFrontier {
                source: pair[0].to_owned(),
            });
        }
    }
    Ok(())
}

fn validate_assertions(
    relationship_id: RelationshipId,
    assertions: &[RelationshipAssertion],
    frontiers: &[SourceFrontier],
) -> Result<(), QualificationError> {
    let mut previous: Option<(&crate::AssertionId, &crate::EvidenceRef)> = None;

    for assertion in assertions {
        if assertion.relationship_id != relationship_id {
            return Err(QualificationError::UnrelatedAssertion {
                relationship_id: assertion.relationship_id,
            });
        }

        assertion
            .envelope
            .validate_schema()
            .map_err(QualificationError::AssertionSchema)?;

        if assertion.envelope.source.source_revision
            != assertion.envelope.evidence.source_revision
        {
            return Err(QualificationError::AssertionEvidenceMismatch {
                assertion_id: assertion.envelope.assertion_id,
            });
        }

        let source = &assertion.envelope.evidence.source_revision.source;
        let frontier = frontiers.iter().find(|f| &f.source == source).ok_or_else(|| {
            QualificationError::MissingEvidenceFrontier {
                source: source.clone(),
            }
        })?;

        if assertion.envelope.evidence.source_revision.revision > frontier.observed_revision {
            return Err(QualificationError::FutureEvidenceRevision {
                assertion_id: assertion.envelope.assertion_id,
                revision: assertion.envelope.evidence.source_revision.revision,
                frontier: frontier.observed_revision,
            });
        }

        if let Some((previous_id, previous_evidence)) = previous {
            if previous_id == &assertion.envelope.assertion_id
                && previous_evidence == &assertion.envelope.evidence
            {
                return Err(QualificationError::DuplicateAssertionEvidence {
                    assertion_id: assertion.envelope.assertion_id,
                    evidence: assertion.envelope.evidence.clone(),
                });
            }
        }
        previous = Some((&assertion.envelope.assertion_id, &assertion.envelope.evidence));
    }

    Ok(())
}

fn validate_commitments(
    relationship_id: RelationshipId,
    commitments: &[Commitment],
) -> Result<(), QualificationError> {
    for pair in commitments.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(QualificationError::DuplicateCommitmentId { id: pair[0].id });
        }
    }

    for commitment in commitments {
        commitment
            .validate_identity()
            .map_err(|error| QualificationError::InvalidCommitment {
                id: commitment.id,
                error,
            })?;

        if commitment.relationship_id != relationship_id {
            return Err(QualificationError::UnrelatedCommitment { id: commitment.id });
        }
        if commitment.events.is_empty() {
            return Err(QualificationError::InvalidCommitment {
                id: commitment.id,
                error: CommitmentError::InvalidTransition {
                    from: commitment.status,
                    event: CommitmentEventKind::Request,
                },
            });
        }

        let first = &commitment.events[0];
        if first.kind != CommitmentEventKind::Request || first.actor != commitment.obligor {
            return Err(QualificationError::CommitmentActorMismatch {
                id: commitment.id,
                event: first.kind,
            });
        }

        let mut replay = Commitment::new(
            commitment.id,
            commitment.relationship_id,
            commitment.obligor.clone(),
            commitment.beneficiary.clone(),
            commitment.due_at,
            first.occurred_at,
            first.actor.clone(),
            first.evidence_ref.clone(),
        )
        .map_err(|error| QualificationError::InvalidCommitment {
            id: commitment.id,
            error,
        })?;

        for event in commitment.events.iter().skip(1) {
            replay
                .transition(event.clone())
                .map_err(|error| QualificationError::InvalidCommitment {
                    id: commitment.id,
                    error,
                })?;
        }

        if replay.status != commitment.status {
            return Err(QualificationError::InvalidCommitment {
                id: commitment.id,
                error: CommitmentError::InvalidTransition {
                    from: replay.status,
                    event: commitment
                        .events
                        .last()
                        .map(|event| event.kind)
                        .unwrap_or(CommitmentEventKind::Request),
                },
            });
        }
    }

    Ok(())
}

fn validate_consents(
    relationship_id: RelationshipId,
    consents: &[Consent],
) -> Result<(), QualificationError> {
    for pair in consents.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(QualificationError::DuplicateConsentId { id: pair[0].id });
        }
    }

    for consent in consents {
        if consent.relationship_id != relationship_id {
            return Err(QualificationError::UnrelatedConsent { id: consent.id });
        }

        if consent.events.is_empty() {
            return Err(QualificationError::InvalidConsent {
                id: consent.id,
                reason: ConsentQualificationError::EmptyEvents,
            });
        }

        let first = &consent.events[0];
        if first.kind != ConsentEventKind::Grant {
            return Err(QualificationError::InvalidConsent {
                id: consent.id,
                reason: ConsentQualificationError::FirstEventNotGrant,
            });
        }
        if first.actor != consent.grantor {
            return Err(QualificationError::InvalidConsent {
                id: consent.id,
                reason: ConsentQualificationError::GrantActorMismatch,
            });
        }
        if first.authority_epoch != consent.authority_epoch {
            return Err(QualificationError::InvalidConsent {
                id: consent.id,
                reason: ConsentQualificationError::GrantEpochMismatch,
            });
        }

        for event in consent.events.iter().skip(1) {
            if event.actor != consent.grantor {
                return Err(QualificationError::InvalidConsent {
                    id: consent.id,
                    reason: ConsentQualificationError::EventActorMismatch { event: event.kind },
                });
            }
            if event.kind == ConsentEventKind::Revoke
                && event.authority_epoch != consent.authority_epoch
            {
                return Err(QualificationError::InvalidConsent {
                    id: consent.id,
                    reason: ConsentQualificationError::StaleRevocationEpoch,
                });
            }
        }

        let last_kind = consent.events.last().map(|event| event.kind);
        match consent.status {
            ConsentStatus::Granted if last_kind != Some(ConsentEventKind::Grant) => {
                return Err(QualificationError::InvalidConsent {
                    id: consent.id,
                    reason: ConsentQualificationError::RevocationStateMismatch,
                });
            }
            ConsentStatus::Revoked if last_kind != Some(ConsentEventKind::Revoke) => {
                return Err(QualificationError::InvalidConsent {
                    id: consent.id,
                    reason: ConsentQualificationError::RevocationStateMismatch,
                });
            }
            ConsentStatus::Expired if last_kind != Some(ConsentEventKind::Expire) => {
                return Err(QualificationError::InvalidConsent {
                    id: consent.id,
                    reason: ConsentQualificationError::ExpiryStateMismatch,
                });
            }
            _ => {}
        }
    }

    Ok(())
}

fn validate_delegations(
    relationship_id: RelationshipId,
    delegations: &[Delegation],
) -> Result<(), QualificationError> {
    for pair in delegations.windows(2) {
        if pair[0].id == pair[1].id {
            return Err(QualificationError::DuplicateDelegationId { id: pair[0].id });
        }
    }

    // Validate every record before any parent-chain authorization is attempted.
    // Otherwise a valid child could temporarily authorize against a forged
    // parent that appears later in the deterministic ordering.
    for delegation in delegations {
        if delegation.relationship_id != relationship_id {
            return Err(QualificationError::UnrelatedDelegation { id: delegation.id });
        }

        delegation
            .validate_identity()
            .map_err(|error| QualificationError::InvalidDelegation {
                id: delegation.id,
                error,
            })?;

        let first = delegation.events.first().ok_or_else(|| {
            QualificationError::InvalidDelegation {
                id: delegation.id,
                error: DelegationError::NonMonotonicEventTime,
            }
        })?;
        if first.kind != crate::DelegationEventKind::Grant
            || first.actor != delegation.principal
            || first.occurred_at != delegation.granted_at
            || first.authority_epoch != delegation.authority_epoch
        {
            return Err(QualificationError::InvalidDelegation {
                id: delegation.id,
                error: DelegationError::StaleAuthorityEpoch,
            });
        }

        for event in delegation.events.iter().skip(1) {
            if event.occurred_at < delegation.granted_at {
                return Err(QualificationError::InvalidDelegation {
                    id: delegation.id,
                    error: DelegationError::NonMonotonicEventTime,
                });
            }
            if event.kind != crate::DelegationEventKind::Revoke
                || event.actor != delegation.principal
                || event.authority_epoch != delegation.authority_epoch
            {
                return Err(QualificationError::InvalidDelegation {
                    id: delegation.id,
                    error: DelegationError::StaleAuthorityEpoch,
                });
            }
        }

        match delegation.status {
            DelegationStatus::Active
                if delegation.events.iter().any(|e| e.kind == crate::DelegationEventKind::Revoke) =>
            {
                return Err(QualificationError::InvalidDelegation {
                    id: delegation.id,
                    error: DelegationError::AlreadyInactive,
                });
            }
            DelegationStatus::Revoked
                if delegation.events.last().map(|e| e.kind)
                    != Some(crate::DelegationEventKind::Revoke) =>
            {
                return Err(QualificationError::InvalidDelegation {
                    id: delegation.id,
                    error: DelegationError::AlreadyInactive,
                });
            }
            _ => {}
        }
    }

    let context = DelegationContext::from_slice(delegations)
        .map_err(|error| match error {
            DelegationError::DuplicateDelegationId { id } => {
                QualificationError::DuplicateDelegationId { id }
            }
            other => QualificationError::InvalidDelegation {
                id: delegations.first().map(|d| d.id).unwrap_or(crate::DelegationId::derive(
                    relationship_id,
                    b"qualification-empty",
                )),
                error: other,
            },
        })?;

    for delegation in delegations {
        let result = if delegation.status == DelegationStatus::Revoked {
            // Historical records must remain auditable/projectable without
            // silently turning revoked authority back into current authority.
            delegation.validate_parent_chain_structure(&context)
        } else {
            let request = ExecutionRequest {
                principal: delegation.principal.clone(),
                delegate: delegation.delegate.clone(),
                action: delegation.action.clone(),
                resource: delegation.resource.clone(),
                mode: delegation.mode,
                requested_at: delegation.granted_at,
                authority_epoch: delegation.authority_epoch,
            };
            delegation.authorize(&request, &context).map(|_| ())
        };

        result.map_err(|error| QualificationError::InvalidDelegation {
            id: delegation.id,
            error,
        })?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AccessMode, AssertionId, AssertionSource, ConsentPurpose, DataClass, DisclosureScope,
        EvidenceDigest, EvidenceRef, EpistemicStatus, ObservationTime, ParticipantRef,
        PrincipalRef, SourceRevision, Visibility,
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
            envelope: AssertionEnvelope::new(
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

    fn consent(r: RelationshipId) -> Consent {
        Consent::grant(
            crate::ConsentId::derive(r, b"consent"),
            r,
            PrincipalRef::new(p("did", "alice")),
            crate::AudienceRef::principal(PrincipalRef::new(p("did", "bob"))),
            ConsentPurpose::named("support").unwrap(),
            DisclosureScope::new(
                AccessMode::Disclose,
                vec![DataClass::Contact],
                vec!["email".into()],
            )
            .unwrap(),
            100,
            Some(200),
            7,
        )
        .unwrap()
    }

    #[test]
    fn tampered_commitment_identity_is_rejected() {
        let r = relationship(b"identity");
        let mut c = crate::Commitment::new(
            crate::CommitmentId::derive(r.relationship_id, b"c"),
            r.relationship_id,
            p("did", "alice"),
            p("org", "acme"),
            None,
            1,
            p("did", "alice"),
            None,
        )
        .unwrap();
        c.beneficiary = p("org", "other");

        assert!(matches!(
            QualifiedRelationshipInputs::qualify(&r, &[], &[], &[c], &[], &[]),
            Err(QualificationError::InvalidCommitment {
                error: crate::CommitmentError::IdentityMismatch,
                ..
            })
        ));
    }

    #[test]
    fn qualification_certificate_is_permutation_invariant() {
        let r = relationship(b"certificate");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = assertion(r.relationship_id, "crm", 7, 2);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        let left = QualifiedRelationshipInputs::qualify(
            &r, &[a.clone(), b.clone()], &[frontier.clone()], &[], &[], &[]
        ).unwrap();
        let right = QualifiedRelationshipInputs::qualify(
            &r, &[b, a], &[frontier], &[], &[], &[]
        ).unwrap();
        assert_eq!(left.certificate, right.certificate);
        assert_eq!(left.certificate.digest(), right.certificate.digest());
    }

    #[test]
    fn qualification_certificate_changes_when_dependency_changes() {
        let r = relationship(b"certificate-change");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = assertion(r.relationship_id, "crm", 7, 2);
        let left = QualifiedRelationshipInputs::qualify(
            &r, &[a], &[SourceFrontier::new("crm", 7).unwrap()], &[], &[], &[]
        ).unwrap();
        let right = QualifiedRelationshipInputs::qualify(
            &r, &[b], &[SourceFrontier::new("crm", 7).unwrap()], &[], &[], &[]
        ).unwrap();
        assert_ne!(left.certificate.digest(), right.certificate.digest());
    }

    #[test]
    fn qualification_certificate_tampering_is_rejected() {
        let r = relationship(b"certificate-tamper");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let mut qualified = QualifiedRelationshipInputs::qualify(
            &r,
            &[a],
            &[SourceFrontier::new("crm", 7).unwrap()],
            &[],
            &[],
            &[],
        ).unwrap();
        qualified.certificate.input_digest[0] ^= 1;

        assert_eq!(
            qualified.validate_certificate(),
            Err(QualificationError::CertificateMismatch)
        );
    }

    #[test]
    fn unrelated_records_are_rejected_instead_of_filtered() {
        let r = relationship(b"r");
        let other = relationship(b"other");
        let c = crate::Commitment::new(
            crate::CommitmentId::derive(other.relationship_id, b"c"),
            other.relationship_id,
            p("did", "alice"),
            p("org", "acme"),
            None,
            1,
            p("did", "alice"),
            None,
        )
        .unwrap();

        assert!(matches!(
            QualifiedRelationshipInputs::qualify(&r, &[], &[], &[c], &[], &[]),
            Err(QualificationError::UnrelatedCommitment { .. })
        ));
    }

    #[test]
    fn future_evidence_is_rejected() {
        let r = relationship(b"r");
        let a = assertion(r.relationship_id, "crm", 8, 1);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        assert!(matches!(
            QualifiedRelationshipInputs::qualify(&r, &[a], &[frontier], &[], &[], &[]),
            Err(QualificationError::FutureEvidenceRevision { .. })
        ));
    }

    #[test]
    fn duplicate_semantic_assertion_with_distinct_evidence_is_allowed() {
        let r = relationship(b"r");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = assertion(r.relationship_id, "crm", 7, 2);
        let frontier = SourceFrontier::new("crm", 7).unwrap();
        let qualified =
            QualifiedRelationshipInputs::qualify(&r, &[b, a], &[frontier], &[], &[], &[]).unwrap();
        assert_eq!(qualified.assertions.len(), 2);
        assert_eq!(qualified.assertions[0].envelope.assertion_id, qualified.assertions[1].envelope.assertion_id);
    }

    #[test]
    fn duplicate_exact_evidence_is_rejected_deterministically() {
        let r = relationship(b"r");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = a.clone();
        assert!(matches!(
            QualifiedRelationshipInputs::qualify(
                &r,
                &[b, a],
                &[SourceFrontier::new("crm", 7).unwrap()],
                &[],
                &[],
                &[]
            ),
            Err(QualificationError::DuplicateAssertionEvidence { .. })
        ));
    }

    #[test]
    fn stale_consent_revocation_event_is_rejected() {
        let r = relationship(b"r");
        let mut c = consent(r.relationship_id);
        c.events.push(crate::ConsentEvent {
            kind: crate::ConsentEventKind::Revoke,
            actor: c.grantor.clone(),
            occurred_at: 150,
            authority_epoch: 8,
        });
        c.status = ConsentStatus::Revoked;

        assert!(matches!(
            QualifiedRelationshipInputs::qualify(&r, &[], &[], &[], &[c], &[]),
            Err(QualificationError::InvalidConsent {
                reason: ConsentQualificationError::StaleRevocationEpoch,
                ..
            })
        ));
    }

    #[test]
    fn qualification_is_permutation_invariant() {
        let r = relationship(b"perm");
        let a = assertion(r.relationship_id, "crm", 7, 1);
        let b = assertion(r.relationship_id, "crm", 7, 2);
        let frontiers = vec![SourceFrontier::new("crm", 7).unwrap()];
        let left = QualifiedRelationshipInputs::qualify(
            &r, &[a.clone(), b.clone()], &frontiers, &[], &[], &[]
        ).unwrap();
        let right = QualifiedRelationshipInputs::qualify(
            &r, &[b, a], &frontiers, &[], &[], &[]
        ).unwrap();
        assert_eq!(left, right);
    }
}


