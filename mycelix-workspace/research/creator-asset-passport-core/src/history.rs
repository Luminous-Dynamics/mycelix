//! Append-only grant/revocation history with deterministic replay.
//!
//! The history validates local sequencing and consistency only. Actor IDs and
//! evidence references are caller-provided; this module does not authenticate
//! identities, verify signatures, or prove that consent was legally effective.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

use crate::{EventId, GrantId, PartyId, PassportError, UsageGrant};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum HistoryError {
    EmptyEvidenceReference,
    EvidenceReferenceTooLong,
    InvalidEvidenceCharacter,
    IssueActorDoesNotMatchGrantIssuer,
    IssueEventAfterGrantEffectiveTime,
    DuplicateGrantId,
    GrantNotFound,
    GrantAlreadyRevoked,
    RevocationBeforeGrantStart,
    ConflictingEventId,
    NonMonotonicEventTime,
    SnapshotMismatch,
}

impl fmt::Display for HistoryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for HistoryError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GrantEventAction {
    Issued { grant: UsageGrant },
    Revoked { grant_id: GrantId },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GrantEvent {
    event_id: EventId,
    occurred_at: u64,
    actor: PartyId,
    evidence_ref: String,
    action: GrantEventAction,
}

impl GrantEvent {
    pub fn issue(
        event_id: EventId,
        occurred_at: u64,
        actor: PartyId,
        evidence_ref: impl AsRef<str>,
        grant: UsageGrant,
    ) -> Result<Self, HistoryError> {
        let evidence_ref = validate_evidence_ref(evidence_ref.as_ref())?;
        if actor != *grant.issued_by() {
            return Err(HistoryError::IssueActorDoesNotMatchGrantIssuer);
        }
        if occurred_at > grant.valid_from() {
            return Err(HistoryError::IssueEventAfterGrantEffectiveTime);
        }
        Ok(Self {
            event_id,
            occurred_at,
            actor,
            evidence_ref,
            action: GrantEventAction::Issued { grant },
        })
    }

    pub fn revoke(
        event_id: EventId,
        occurred_at: u64,
        actor: PartyId,
        evidence_ref: impl AsRef<str>,
        grant_id: GrantId,
    ) -> Result<Self, HistoryError> {
        Ok(Self {
            event_id,
            occurred_at,
            actor,
            evidence_ref: validate_evidence_ref(evidence_ref.as_ref())?,
            action: GrantEventAction::Revoked { grant_id },
        })
    }

    pub fn event_id(&self) -> &EventId {
        &self.event_id
    }

    pub fn occurred_at(&self) -> u64 {
        self.occurred_at
    }

    pub fn actor(&self) -> &PartyId {
        &self.actor
    }

    pub fn evidence_ref(&self) -> &str {
        &self.evidence_ref
    }

    pub fn action(&self) -> &GrantEventAction {
        &self.action
    }
}

fn validate_evidence_ref(value: &str) -> Result<String, HistoryError> {
    let value = value.trim();
    if value.is_empty() {
        return Err(HistoryError::EmptyEvidenceReference);
    }
    if value.len() > 256 {
        return Err(HistoryError::EvidenceReferenceTooLong);
    }
    if value.chars().any(char::is_control) {
        return Err(HistoryError::InvalidEvidenceCharacter);
    }
    Ok(value.to_owned())
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AppendOutcome {
    Appended,
    AlreadyPresent,
}

/// Append-only history for one passport scope or a collection of grants.
///
/// Events must be appended in nondecreasing event-time order. Exact event replay
/// is idempotent. Reusing an event ID for different content, issuing a grant ID
/// twice, or revoking a nonexistent/already-revoked grant fails without mutating
/// the history.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct GrantHistory {
    events: Vec<GrantEvent>,
    event_positions: BTreeMap<EventId, usize>,
    effective_grants: BTreeMap<GrantId, UsageGrant>,
}

impl GrantHistory {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn append(&mut self, event: GrantEvent) -> Result<AppendOutcome, HistoryError> {
        if let Some(index) = self.event_positions.get(event.event_id()) {
            return if self.events[*index] == event {
                Ok(AppendOutcome::AlreadyPresent)
            } else {
                Err(HistoryError::ConflictingEventId)
            };
        }

        if self
            .events
            .last()
            .is_some_and(|previous| event.occurred_at() < previous.occurred_at())
        {
            return Err(HistoryError::NonMonotonicEventTime);
        }

        // Validate and derive the state update before changing any persisted
        // in-memory state. A failed transition leaves events and grants intact.
        let update = match event.action() {
            GrantEventAction::Issued { grant } => {
                if self.effective_grants.contains_key(grant.id()) {
                    return Err(HistoryError::DuplicateGrantId);
                }
                if event.actor() != grant.issued_by() {
                    return Err(HistoryError::IssueActorDoesNotMatchGrantIssuer);
                }
                if event.occurred_at() > grant.valid_from() {
                    return Err(HistoryError::IssueEventAfterGrantEffectiveTime);
                }
                Some((grant.id().clone(), grant.clone()))
            }
            GrantEventAction::Revoked { grant_id } => {
                let grant = self
                    .effective_grants
                    .get(grant_id)
                    .ok_or(HistoryError::GrantNotFound)?;
                if grant.revocation_timestamp().is_some() {
                    return Err(HistoryError::GrantAlreadyRevoked);
                }
                let revoked =
                    grant
                        .revoked_at(event.occurred_at())
                        .map_err(|error| match error {
                            PassportError::GrantAlreadyRevoked => HistoryError::GrantAlreadyRevoked,
                            PassportError::RevocationBeforeEffectiveTime => {
                                HistoryError::RevocationBeforeGrantStart
                            }
                            _ => HistoryError::RevocationBeforeGrantStart,
                        })?;
                Some((grant_id.clone(), revoked))
            }
        };

        if let Some((grant_id, grant)) = update {
            self.effective_grants.insert(grant_id, grant);
        }
        let index = self.events.len();
        self.event_positions.insert(event.event_id().clone(), index);
        self.events.push(event);
        Ok(AppendOutcome::Appended)
    }

    pub fn replay(events: &[GrantEvent]) -> Result<Self, HistoryError> {
        let mut replayed = Self::new();
        for event in events {
            replayed.append(event.clone())?;
        }
        Ok(replayed)
    }

    /// Rebuild effective grants from a fresh replay and compare against this
    /// instance's derived state. This detects an inconsistent in-memory snapshot
    /// but is not a cryptographic signature or legal-validity verifier.
    pub fn verify_against_replay(&self) -> Result<(), HistoryError> {
        let rebuilt = Self::replay(&self.events)?;
        if rebuilt.effective_grants == self.effective_grants {
            Ok(())
        } else {
            Err(HistoryError::SnapshotMismatch)
        }
    }

    /// Reconstruct through the independent verifier path and compare with
    /// the state derived during append. Unlike verify_against_replay, this does
    /// not call GrantHistory::append to recreate its comparison state.
    pub fn verify_against_independent_replay(&self) -> Result<(), HistoryError> {
        let rebuilt = independently_reconstruct_effective_grants(&self.events)?;
        if rebuilt == self.effective_grants() {
            Ok(())
        } else {
            Err(HistoryError::SnapshotMismatch)
        }
    }

    pub fn events(&self) -> &[GrantEvent] {
        &self.events
    }

    pub fn effective_grants(&self) -> Vec<UsageGrant> {
        self.effective_grants.values().cloned().collect()
    }

    pub fn find_grant(&self, grant_id: &GrantId) -> Option<&UsageGrant> {
        self.effective_grants.get(grant_id)
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }
}

/// Reconstruct effective grants using a separate implementation rather than
/// delegating to GrantHistory::append. This is an independent code path for
/// differential verification of the event log; it still cannot authenticate
/// event authors or prove the truth/legal effect of referenced evidence.
pub fn independently_reconstruct_effective_grants(
    events: &[GrantEvent],
) -> Result<Vec<UsageGrant>, HistoryError> {
    let mut seen_events: BTreeMap<EventId, GrantEvent> = BTreeMap::new();
    let mut grants: BTreeMap<GrantId, UsageGrant> = BTreeMap::new();
    let mut last_event_time: Option<u64> = None;

    for event in events {
        if let Some(previous) = seen_events.get(event.event_id()) {
            if previous == event {
                continue;
            }
            return Err(HistoryError::ConflictingEventId);
        }

        if last_event_time.is_some_and(|last| event.occurred_at() < last) {
            return Err(HistoryError::NonMonotonicEventTime);
        }

        validate_evidence_ref(event.evidence_ref())?;

        match event.action() {
            GrantEventAction::Issued { grant } => {
                if event.actor() != grant.issued_by() {
                    return Err(HistoryError::IssueActorDoesNotMatchGrantIssuer);
                }
                if event.occurred_at() > grant.valid_from() {
                    return Err(HistoryError::IssueEventAfterGrantEffectiveTime);
                }
                if grants.contains_key(grant.id()) {
                    return Err(HistoryError::DuplicateGrantId);
                }
                grants.insert(grant.id().clone(), grant.clone());
            }
            GrantEventAction::Revoked { grant_id } => {
                let current = grants.get(grant_id).ok_or(HistoryError::GrantNotFound)?;
                if current.revocation_timestamp().is_some() {
                    return Err(HistoryError::GrantAlreadyRevoked);
                }
                let updated =
                    current
                        .revoked_at(event.occurred_at())
                        .map_err(|error| match error {
                            PassportError::GrantAlreadyRevoked => HistoryError::GrantAlreadyRevoked,
                            PassportError::RevocationBeforeEffectiveTime => {
                                HistoryError::RevocationBeforeGrantStart
                            }
                            _ => HistoryError::RevocationBeforeGrantStart,
                        })?;
                grants.insert(grant_id.clone(), updated);
            }
        }

        seen_events.insert(event.event_id().clone(), event.clone());
        last_event_time = Some(event.occurred_at());
    }

    Ok(grants.into_values().collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AssetId, UseAction};

    fn party(value: &str) -> PartyId {
        PartyId::new(value).unwrap()
    }

    fn grant(id: &str, valid_from: u64) -> UsageGrant {
        UsageGrant::new(
            GrantId::new(id).unwrap(),
            AssetId::new("recording-1").unwrap(),
            party("studio"),
            UseAction::Publish,
            "release-a",
            party("creator"),
            "consent://grant",
            valid_from,
            None,
        )
        .unwrap()
    }

    fn issue_event(event_id: &str, occurred_at: u64, grant: UsageGrant) -> GrantEvent {
        GrantEvent::issue(
            EventId::new(event_id).unwrap(),
            occurred_at,
            party("creator"),
            "evidence://issue",
            grant,
        )
        .unwrap()
    }

    fn revoke_event(event_id: &str, occurred_at: u64, grant_id: &str) -> GrantEvent {
        GrantEvent::revoke(
            EventId::new(event_id).unwrap(),
            occurred_at,
            party("creator"),
            "evidence://revoke",
            GrantId::new(grant_id).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn issue_then_revoke_reconstructs_effective_grant() {
        let issue = issue_event("evt-1", 5, grant("g-1", 10));
        let revoke = revoke_event("evt-2", 20, "g-1");
        let history = GrantHistory::replay(&[issue, revoke]).unwrap();
        let effective = history.find_grant(&GrantId::new("g-1").unwrap()).unwrap();
        assert_eq!(history.len(), 2);
        assert_eq!(effective.revocation_timestamp(), Some(20));
        assert!(history.verify_against_replay().is_ok());
    }

    #[test]
    fn independent_reconstruction_matches_append_derived_state() {
        let issue = issue_event("evt-1", 5, grant("g-1", 10));
        let revoke = revoke_event("evt-2", 20, "g-1");
        let history = GrantHistory::replay(&[issue.clone(), revoke.clone()]).unwrap();

        assert_eq!(
            independently_reconstruct_effective_grants(&[issue, revoke]).unwrap(),
            history.effective_grants()
        );
        assert!(history.verify_against_independent_replay().is_ok());
    }

    #[test]
    fn independent_reconstruction_rejects_conflicting_event_ids() {
        let events = [
            issue_event("evt-1", 5, grant("g-1", 10)),
            issue_event("evt-1", 5, grant("g-2", 10)),
        ];
        assert_eq!(
            independently_reconstruct_effective_grants(&events),
            Err(HistoryError::ConflictingEventId)
        );
    }

    #[test]
    fn independent_reconstruction_rejects_non_monotonic_events() {
        let events = [
            issue_event("evt-1", 5, grant("g-1", 10)),
            revoke_event("evt-2", 4, "g-1"),
        ];
        assert_eq!(
            independently_reconstruct_effective_grants(&events),
            Err(HistoryError::NonMonotonicEventTime)
        );
    }

    #[test]
    fn exact_event_replay_is_idempotent() {
        let event = issue_event("evt-1", 5, grant("g-1", 10));
        let mut history = GrantHistory::new();
        assert_eq!(history.append(event.clone()), Ok(AppendOutcome::Appended));
        assert_eq!(history.append(event), Ok(AppendOutcome::AlreadyPresent));
        assert_eq!(history.len(), 1);
    }

    #[test]
    fn conflicting_event_id_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        history
            .append(issue_event("evt-1", 5, grant("g-1", 10)))
            .unwrap();
        let conflicting = issue_event("evt-1", 5, grant("g-2", 10));
        assert_eq!(
            history.append(conflicting),
            Err(HistoryError::ConflictingEventId)
        );
        assert_eq!(history.len(), 1);
        assert!(history.find_grant(&GrantId::new("g-1").unwrap()).is_some());
        assert!(history.find_grant(&GrantId::new("g-2").unwrap()).is_none());
    }

    #[test]
    fn duplicate_grant_id_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        history
            .append(issue_event("evt-1", 5, grant("g-1", 10)))
            .unwrap();
        assert_eq!(
            history.append(issue_event("evt-2", 6, grant("g-1", 10))),
            Err(HistoryError::DuplicateGrantId)
        );
        assert_eq!(history.len(), 1);
    }

    #[test]
    fn revoking_unknown_grant_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        assert_eq!(
            history.append(revoke_event("evt-1", 20, "missing")),
            Err(HistoryError::GrantNotFound)
        );
        assert!(history.is_empty());
    }

    #[test]
    fn duplicate_revocation_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        history
            .append(issue_event("evt-1", 5, grant("g-1", 10)))
            .unwrap();
        history.append(revoke_event("evt-2", 20, "g-1")).unwrap();
        assert_eq!(
            history.append(revoke_event("evt-3", 21, "g-1")),
            Err(HistoryError::GrantAlreadyRevoked)
        );
        assert_eq!(history.len(), 2);
        assert_eq!(
            history
                .find_grant(&GrantId::new("g-1").unwrap())
                .unwrap()
                .revocation_timestamp(),
            Some(20)
        );
    }

    #[test]
    fn non_monotonic_time_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        history
            .append(issue_event("evt-1", 5, grant("g-1", 10)))
            .unwrap();
        assert_eq!(
            history.append(revoke_event("evt-2", 4, "g-1")),
            Err(HistoryError::NonMonotonicEventTime)
        );
        assert_eq!(history.len(), 1);
    }

    #[test]
    fn wrong_issuer_actor_and_late_issue_are_rejected() {
        let wrong_actor = GrantEvent::issue(
            EventId::new("evt-1").unwrap(),
            5,
            party("other"),
            "evidence://issue",
            grant("g-1", 10),
        );
        assert_eq!(
            wrong_actor,
            Err(HistoryError::IssueActorDoesNotMatchGrantIssuer)
        );

        let late = GrantEvent::issue(
            EventId::new("evt-2").unwrap(),
            11,
            party("creator"),
            "evidence://issue",
            grant("g-2", 10),
        );
        assert_eq!(late, Err(HistoryError::IssueEventAfterGrantEffectiveTime));
    }

    #[test]
    fn revocation_before_grant_start_is_rejected_without_mutation() {
        let mut history = GrantHistory::new();
        history
            .append(issue_event("evt-1", 5, grant("g-1", 10)))
            .unwrap();
        assert_eq!(
            history.append(revoke_event("evt-2", 9, "g-1")),
            Err(HistoryError::RevocationBeforeGrantStart)
        );
        assert_eq!(history.len(), 1);
    }

    #[test]
    fn replay_rejects_non_monotonic_exported_history() {
        let issue = issue_event("evt-1", 5, grant("g-1", 10));
        let revoke = revoke_event("evt-2", 4, "g-1");
        assert_eq!(
            GrantHistory::replay(&[issue, revoke]),
            Err(HistoryError::NonMonotonicEventTime)
        );
    }

    #[test]
    fn empty_evidence_reference_is_rejected() {
        let bad = GrantEvent::revoke(
            EventId::new("evt-1").unwrap(),
            20,
            party("creator"),
            "   ",
            GrantId::new("g-1").unwrap(),
        );
        assert_eq!(bad, Err(HistoryError::EmptyEvidenceReference));
    }
}
