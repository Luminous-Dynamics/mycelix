//! Vertical-neutral commitment lifecycle primitives for the Relationship OS.
//! A commitment is a durable assertion about an expected action or state.
//! Recording one never proves acceptance, fulfillment, settlement, or external effect.

use std::fmt;

use crate::{ParticipantRef, RelationshipId};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct CommitmentId([u8; 32]);

impl CommitmentId {
    pub fn derive(relationship: RelationshipId, seed: &[u8]) -> Self {
        let mut h = blake3::Hasher::new();
        h.update(b"mycelix.commitment.v1\0");
        h.update(relationship.as_bytes());
        h.update(&(seed.len() as u64).to_le_bytes());
        h.update(seed);
        Self(*h.finalize().as_bytes())
    }

    pub fn derive_content_bound(commitment: &Commitment) -> Self {
        let mut h = blake3::Hasher::new();
        h.update(b"mycelix.commitment-content.v1\0");
        let canonical = commitment.canonical_bytes();
        h.update(&(canonical.len() as u64).to_le_bytes());
        h.update(&canonical);
        Self(*h.finalize().as_bytes())
    }

    pub const fn as_bytes(&self) -> &[u8; 32] { &self.0 }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum CommitmentStatus {
    Requested,
    Accepted,
    Declined,
    Active,
    PartiallyFulfilled,
    Fulfilled,
    Disputed,
    Cancelled,
    Expired,
}

impl CommitmentStatus {
    pub const fn is_terminal(self) -> bool {
        matches!(self, Self::Declined | Self::Fulfilled | Self::Cancelled | Self::Expired)
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum CommitmentEventKind {
    Request,
    Accept,
    Decline,
    Activate,
    PartialFulfillment,
    Fulfillment,
    Dispute,
    Cancel,
    Expire,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CommitmentEvent {
    pub kind: CommitmentEventKind,
    pub actor: ParticipantRef,
    pub occurred_at: i64,
    pub evidence_ref: Option<String>,
    pub source_revision: Option<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Commitment {
    pub id: CommitmentId,
    pub relationship_id: RelationshipId,
    pub obligor: ParticipantRef,
    pub beneficiary: ParticipantRef,
    pub status: CommitmentStatus,
    pub due_at: Option<i64>,
    pub events: Vec<CommitmentEvent>,
}

impl Commitment {
    pub fn new(
        id: CommitmentId,
        relationship_id: RelationshipId,
        obligor: ParticipantRef,
        beneficiary: ParticipantRef,
        due_at: Option<i64>,
        created_at: i64,
        actor: ParticipantRef,
        evidence_ref: Option<String>,
    ) -> Result<Self, CommitmentError> {
        if obligor == beneficiary {
            return Err(CommitmentError::SelfCommitment);
        }
        if actor != obligor {
            return Err(CommitmentError::UnauthorizedActor);
        }
        let first = CommitmentEvent {
            kind: CommitmentEventKind::Request,
            actor,
            occurred_at: created_at,
            evidence_ref,
            source_revision: None,
        };
        let mut commitment = Self {
            id: CommitmentId([0; 32]),
            relationship_id,
            obligor,
            beneficiary,
            status: CommitmentStatus::Requested,
            due_at,
            events: vec![first],
        };
        commitment.id = CommitmentId::derive_content_bound(&commitment);
        Ok(commitment)
    }

    pub fn canonical_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(b"mycelix.commitment-content.v1\0");
        out.extend_from_slice(self.relationship_id.as_bytes());
        write_participant(&mut out, &self.obligor);
        write_participant(&mut out, &self.beneficiary);
        match self.due_at {
            Some(value) => {
                out.push(1);
                out.extend_from_slice(&value.to_le_bytes());
            }
            None => out.push(0),
        }
        out
    }

    pub fn state_canonical_bytes(&self) -> Vec<u8> {
        let mut out = self.canonical_bytes();
        out.push(self.status as u8);
        out.extend_from_slice(&(self.events.len() as u64).to_le_bytes());
        for event in &self.events {
            out.push(event.kind as u8);
            write_participant(&mut out, &event.actor);
            out.extend_from_slice(&event.occurred_at.to_le_bytes());
            match &event.evidence_ref {
                Some(value) => {
                    out.push(1);
                    write_bytes(&mut out, value.as_bytes());
                }
                None => out.push(0),
            }
            match &event.source_revision {
                Some(value) => {
                    out.push(1);
                    write_bytes(&mut out, value.as_bytes());
                }
                None => out.push(0),
            }
        }
        out
    }

    pub fn validate_identity(&self) -> Result<(), CommitmentError> {
        if self.id != CommitmentId::derive_content_bound(self) {
            return Err(CommitmentError::IdentityMismatch);
        }
        Ok(())
    }

    pub fn transition(&mut self, event: CommitmentEvent) -> Result<(), CommitmentError> {
        let actor_allowed = match event.kind {
            CommitmentEventKind::Request => event.actor == self.obligor,
            CommitmentEventKind::Accept | CommitmentEventKind::Decline => {
                event.actor == self.beneficiary
            }
            CommitmentEventKind::Activate
            | CommitmentEventKind::PartialFulfillment
            | CommitmentEventKind::Fulfillment => event.actor == self.obligor,
            CommitmentEventKind::Dispute
            | CommitmentEventKind::Cancel
            | CommitmentEventKind::Expire => {
                event.actor == self.obligor || event.actor == self.beneficiary
            }
        };
        if !actor_allowed {
            return Err(CommitmentError::UnauthorizedActor);
        }

        let next = next_status(self.status, event.kind)
            .ok_or(CommitmentError::InvalidTransition {
                from: self.status,
                event: event.kind,
            })?;

        if let Some(previous) = self.events.last() {
            if event.occurred_at < previous.occurred_at {
                return Err(CommitmentError::NonMonotonicEventTime);
            }
        }

        if matches!(event.kind, CommitmentEventKind::Fulfillment | CommitmentEventKind::PartialFulfillment)
            && event.evidence_ref.is_none()
        {
            return Err(CommitmentError::FulfillmentRequiresEvidence);
        }

        self.status = next;
        self.events.push(event);
        Ok(())
    }
}

fn next_status(from: CommitmentStatus, event: CommitmentEventKind) -> Option<CommitmentStatus> {
    use CommitmentEventKind::*;
    use CommitmentStatus::*;
    match (from, event) {
        (Requested, Accept) => Some(Accepted),
        (Requested, Decline) => Some(Declined),
        (Accepted, Activate) => Some(Active),
        (Active, PartialFulfillment) => Some(PartiallyFulfilled),
        (PartiallyFulfilled, PartialFulfillment) => Some(PartiallyFulfilled),
        (Active, Fulfillment) | (PartiallyFulfilled, Fulfillment) => Some(Fulfilled),
        (Requested | Accepted | Active | PartiallyFulfilled, Dispute) => Some(Disputed),
        (Requested | Accepted | Active | PartiallyFulfilled | Disputed, Cancel) => Some(Cancelled),
        (Requested | Accepted | Active | PartiallyFulfilled, Expire) => Some(Expired),
        (Disputed, Activate) => Some(Active),
        _ => None,
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CommitmentError {
    SelfCommitment,
    UnauthorizedActor,
    IdentityMismatch,
    InvalidTransition { from: CommitmentStatus, event: CommitmentEventKind },
    NonMonotonicEventTime,
    FulfillmentRequiresEvidence,
}

impl fmt::Display for CommitmentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::SelfCommitment => write!(f, "obligor and beneficiary must differ"),
            Self::UnauthorizedActor => write!(f, "initial request must be authored by the obligor"),
            Self::IdentityMismatch => write!(f, "commitment identifier does not match canonical obligation content"),
            Self::InvalidTransition { from, event } => write!(f, "invalid transition from {from:?} via {event:?}"),
            Self::NonMonotonicEventTime => write!(f, "event time cannot move backwards"),
            Self::FulfillmentRequiresEvidence => write!(f, "fulfillment requires an evidence reference"),
        }
    }
}

impl std::error::Error for CommitmentError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(ns: &str, id: &str) -> ParticipantRef {
        ParticipantRef::new(ns, id)
    }

    fn event(kind: CommitmentEventKind, actor: ParticipantRef, t: i64, evidence: Option<&str>) -> CommitmentEvent {
        CommitmentEvent {
            kind,
            actor,
            occurred_at: t,
            evidence_ref: evidence.map(str::to_owned),
            source_revision: None,
        }
    }

    fn commitment() -> Commitment {
        let relationship = RelationshipId::derive("test", b"relationship");
        Commitment::new(
            CommitmentId::derive(relationship, b"commitment"),
            relationship,
            p("did", "alice"),
            p("org", "acme"),
            Some(200),
            100,
            p("did", "alice"),
            Some("e:request".into()),
        ).unwrap()
    }

    #[test]
    fn identity_is_bound_to_immutable_obligation_content() {
        let c = commitment();
        assert_eq!(c.id, CommitmentId::derive_content_bound(&c));
        assert!(c.validate_identity().is_ok());
    }

    #[test]
    fn immutable_content_tampering_breaks_identity() {
        let mut c = commitment();
        c.due_at = Some(201);
        assert_eq!(c.validate_identity(), Err(CommitmentError::IdentityMismatch));
    }

    #[test]
    fn lifecycle_events_do_not_change_identity() {
        let mut c = commitment();
        let id = c.id;
        c.transition(event(CommitmentEventKind::Accept, p("org", "acme"), 110, Some("e:accept"))).unwrap();
        assert_eq!(c.id, id);
        assert!(c.validate_identity().is_ok());
    }

    #[test]
    fn unauthorized_lifecycle_actor_is_rejected_without_mutation() {
        let mut c = commitment();
        let before = c.clone();
        assert_eq!(
            c.transition(event(CommitmentEventKind::Accept, p("did", "mallory"), 110, Some("e"))),
            Err(CommitmentError::UnauthorizedActor)
        );
        assert_eq!(c, before);
    }

    #[test]
    fn normal_lifecycle_is_explicit() {
        let mut c = commitment();
        c.transition(event(CommitmentEventKind::Accept, p("org", "acme"), 110, Some("e:accept"))).unwrap();
        c.transition(event(CommitmentEventKind::Activate, p("did", "alice"), 120, Some("e:activate"))).unwrap();
        c.transition(event(CommitmentEventKind::Fulfillment, p("did", "alice"), 150, Some("e:complete"))).unwrap();
        assert_eq!(c.status, CommitmentStatus::Fulfilled);
    }

    #[test]
    fn fulfillment_without_evidence_fails_closed() {
        let mut c = commitment();
        c.transition(event(CommitmentEventKind::Accept, p("org", "acme"), 110, Some("e:accept"))).unwrap();
        c.transition(event(CommitmentEventKind::Activate, p("did", "alice"), 120, Some("e:activate"))).unwrap();
        assert_eq!(
            c.transition(event(CommitmentEventKind::Fulfillment, p("did", "alice"), 150, None)),
            Err(CommitmentError::FulfillmentRequiresEvidence)
        );
        assert_eq!(c.status, CommitmentStatus::Active);
    }

    #[test]
    fn stale_event_is_rejected() {
        let mut c = commitment();
        assert_eq!(
            c.transition(event(CommitmentEventKind::Accept, p("org", "acme"), 99, Some("e"))),
            Err(CommitmentError::NonMonotonicEventTime)
        );
    }

    #[test]
    fn terminal_commitment_cannot_be_reactivated() {
        let mut c = commitment();
        c.transition(event(CommitmentEventKind::Decline, p("org", "acme"), 110, Some("e"))).unwrap();
        assert!(c.status.is_terminal());
        assert!(c.transition(event(CommitmentEventKind::Activate, p("did", "alice"), 120, Some("e"))).is_err());
    }

    #[test]
    fn partial_fulfillment_does_not_equal_fulfillment() {
        let mut c = commitment();
        c.transition(event(CommitmentEventKind::Accept, p("org", "acme"), 110, Some("e"))).unwrap();
        c.transition(event(CommitmentEventKind::Activate, p("did", "alice"), 120, Some("e"))).unwrap();
        c.transition(event(CommitmentEventKind::PartialFulfillment, p("did", "alice"), 130, Some("e:partial"))).unwrap();
        assert_eq!(c.status, CommitmentStatus::PartiallyFulfilled);
    }

    #[test]
    fn self_commitment_is_rejected() {
        let relationship = RelationshipId::derive("test", b"r");
        assert_eq!(
            Commitment::new(
                CommitmentId::derive(relationship, b"c"),
                relationship,
                p("did", "alice"),
                p("did", "alice"),
                None, 1, p("did", "alice"), None
            ),
            Err(CommitmentError::SelfCommitment)
        );
    }
}

fn write_participant(out: &mut Vec<u8>, participant: &ParticipantRef) {
    write_bytes(out, participant.namespace.as_bytes());
    write_bytes(out, participant.identifier.as_bytes());
}

fn write_bytes(out: &mut Vec<u8>, bytes: &[u8]) {
    out.extend_from_slice(&(bytes.len() as u64).to_le_bytes());
    out.extend_from_slice(bytes);
}
