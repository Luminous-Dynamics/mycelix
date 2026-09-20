// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Planner-grade Care digest v4 over already-classified authority evidence.

use hearth_care_completion_authority::{CompletionAuthorityAssessment, CompletionAuthorityState};
use hearth_care_ledger_contract::MemberId;
use hearth_care_occurrence_authority::{
    AssignmentAuthorityState, OccurrenceAuthorityAssessment, OccurrenceAuthoritySubject,
    TimeAuthorityState,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV4Item {
    pub subject: OccurrenceAuthoritySubject,
    pub occurrence_authority: OccurrenceAuthorityAssessment,
    pub completion_authority: CompletionAuthorityAssessment,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV4Member {
    pub tasks_completed: u64,
    pub known_actual_minutes: u64,
    pub unknown_actual_duration_count: u64,
    pub estimated_minutes_for_completed_tasks: u64,
    pub completed_tasks_without_estimate: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV4 {
    pub by_member: BTreeMap<MemberId, CareDigestV4Member>,
    pub item_count: u64,
    pub qualified_occurrence_count: u64,
    pub assignment_missing_count: u64,
    pub assignment_stale_count: u64,
    pub assignment_conflict_count: u64,
    pub time_missing_count: u64,
    pub time_stale_count: u64,
    pub time_conflict_count: u64,
    pub qualified_completion_count: u64,
    pub completion_missing_count: u64,
    pub completion_occurrence_unqualified_count: u64,
    pub completion_conflict_count: u64,
    pub completion_performer_mismatch_count: u64,
    pub completion_third_party_attestation_count: u64,
    pub agreeing_duplicate_assignment_evidence_count: u64,
    pub agreeing_duplicate_time_evidence_count: u64,
    pub agreeing_duplicate_completion_evidence_count: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CareDigestV4Error {
    InvalidSubject(String),
    OccurrenceAssessmentMismatch,
    CompletionAssessmentMismatch,
    CompletionAssessmentInconsistent,
    QualifiedPerformerMismatch,
    DuplicateOccurrenceRef(String),
    DuplicateOccurrenceId(String),
}

impl std::fmt::Display for CareDigestV4Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for CareDigestV4Error {}

/// Aggregate only already-proven authority. Raw evidence canonicalization belongs
/// to D1B/D1C; Digest v4 summarizes their exact classifications.
pub fn build_digest_v4(items: &[CareDigestV4Item]) -> Result<CareDigestV4, CareDigestV4Error> {
    let mut digest = CareDigestV4::default();
    let mut seen_refs = BTreeSet::new();
    let mut seen_ids = BTreeSet::new();

    for item in items {
        validate_item(item)?;
        if !seen_refs.insert(item.subject.occurrence_ref.clone()) {
            return Err(CareDigestV4Error::DuplicateOccurrenceRef(
                item.subject.occurrence_ref.clone(),
            ));
        }
        if !seen_ids.insert(item.subject.occurrence.id.0.clone()) {
            return Err(CareDigestV4Error::DuplicateOccurrenceId(
                item.subject.occurrence.id.0.clone(),
            ));
        }

        digest.item_count = digest.item_count.saturating_add(1);
        count_occurrence_authority(&mut digest, &item.occurrence_authority);
        count_completion_authority(&mut digest, &item.completion_authority);

        let CompletionAuthorityState::Qualified {
            performed_by,
            actual_minutes,
            duplicate_count,
            ..
        } = &item.completion_authority.state
        else {
            continue;
        };

        if performed_by.as_str() != item.subject.occurrence.assigned_to.0.as_str() {
            return Err(CareDigestV4Error::QualifiedPerformerMismatch);
        }

        digest.qualified_completion_count = digest.qualified_completion_count.saturating_add(1);
        digest.agreeing_duplicate_completion_evidence_count = digest
            .agreeing_duplicate_completion_evidence_count
            .saturating_add(u64::from(*duplicate_count));

        let stats = digest
            .by_member
            .entry(item.subject.occurrence.assigned_to.clone())
            .or_default();
        stats.tasks_completed = stats.tasks_completed.saturating_add(1);
        match actual_minutes {
            Some(minutes) => {
                stats.known_actual_minutes =
                    stats.known_actual_minutes.saturating_add(u64::from(*minutes));
            }
            None => {
                stats.unknown_actual_duration_count =
                    stats.unknown_actual_duration_count.saturating_add(1);
            }
        }
        match item.subject.occurrence.estimated_minutes {
            Some(minutes) => {
                stats.estimated_minutes_for_completed_tasks = stats
                    .estimated_minutes_for_completed_tasks
                    .saturating_add(u64::from(minutes));
            }
            None => {
                stats.completed_tasks_without_estimate =
                    stats.completed_tasks_without_estimate.saturating_add(1);
            }
        }
    }
    Ok(digest)
}

fn validate_item(item: &CareDigestV4Item) -> Result<(), CareDigestV4Error> {
    item.subject
        .validate()
        .map_err(|error| CareDigestV4Error::InvalidSubject(error.to_string()))?;
    if item.occurrence_authority.occurrence_ref.as_str() != item.subject.occurrence_ref.as_str()
        || item.occurrence_authority.occurrence_id.as_str() != item.subject.occurrence.id.0.as_str()
    {
        return Err(CareDigestV4Error::OccurrenceAssessmentMismatch);
    }
    if item.completion_authority.occurrence_ref.as_str() != item.subject.occurrence_ref.as_str()
        || item.completion_authority.occurrence_id.as_str() != item.subject.occurrence.id.0.as_str()
    {
        return Err(CareDigestV4Error::CompletionAssessmentMismatch);
    }
    match &item.completion_authority.state {
        CompletionAuthorityState::OccurrenceUnqualified { assignment, time } => {
            if item.occurrence_authority.is_qualified()
                || assignment != &item.occurrence_authority.assignment
                || time != &item.occurrence_authority.time
            {
                return Err(CareDigestV4Error::CompletionAssessmentInconsistent);
            }
        }
        _ if !item.occurrence_authority.is_qualified() => {
            return Err(CareDigestV4Error::CompletionAssessmentInconsistent);
        }
        _ => {}
    }
    Ok(())
}

fn count_occurrence_authority(digest: &mut CareDigestV4, value: &OccurrenceAuthorityAssessment) {
    let mut qualified = true;
    match &value.assignment {
        AssignmentAuthorityState::Missing => {
            qualified = false;
            digest.assignment_missing_count = digest.assignment_missing_count.saturating_add(1);
        }
        AssignmentAuthorityState::Stale { .. } => {
            qualified = false;
            digest.assignment_stale_count = digest.assignment_stale_count.saturating_add(1);
        }
        AssignmentAuthorityState::Conflict => {
            qualified = false;
            digest.assignment_conflict_count = digest.assignment_conflict_count.saturating_add(1);
        }
        AssignmentAuthorityState::Current { duplicate_count, .. } => {
            digest.agreeing_duplicate_assignment_evidence_count = digest
                .agreeing_duplicate_assignment_evidence_count
                .saturating_add(u64::from(*duplicate_count));
        }
    }
    match &value.time {
        TimeAuthorityState::Missing => {
            qualified = false;
            digest.time_missing_count = digest.time_missing_count.saturating_add(1);
        }
        TimeAuthorityState::Stale { .. } => {
            qualified = false;
            digest.time_stale_count = digest.time_stale_count.saturating_add(1);
        }
        TimeAuthorityState::Conflict => {
            qualified = false;
            digest.time_conflict_count = digest.time_conflict_count.saturating_add(1);
        }
        TimeAuthorityState::Current { duplicate_count, .. } => {
            digest.agreeing_duplicate_time_evidence_count = digest
                .agreeing_duplicate_time_evidence_count
                .saturating_add(u64::from(*duplicate_count));
        }
    }
    if qualified {
        digest.qualified_occurrence_count = digest.qualified_occurrence_count.saturating_add(1);
    }
}

fn count_completion_authority(digest: &mut CareDigestV4, value: &CompletionAuthorityAssessment) {
    match &value.state {
        CompletionAuthorityState::Missing => {
            digest.completion_missing_count = digest.completion_missing_count.saturating_add(1)
        }
        CompletionAuthorityState::OccurrenceUnqualified { .. } => {
            digest.completion_occurrence_unqualified_count = digest
                .completion_occurrence_unqualified_count
                .saturating_add(1)
        }
        CompletionAuthorityState::Conflict => {
            digest.completion_conflict_count = digest.completion_conflict_count.saturating_add(1)
        }
        CompletionAuthorityState::PerformerMismatch { .. } => {
            digest.completion_performer_mismatch_count = digest
                .completion_performer_mismatch_count
                .saturating_add(1)
        }
        CompletionAuthorityState::ThirdPartyAttestation { .. } => {
            digest.completion_third_party_attestation_count = digest
                .completion_third_party_attestation_count
                .saturating_add(1)
        }
        CompletionAuthorityState::Qualified { .. } => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_ledger_contract::{CareOccurrence, OccurrenceWindow, RecordId, ScheduleId};

    fn subject() -> OccurrenceAuthoritySubject {
        OccurrenceAuthoritySubject {
            occurrence_ref: "occurrence:entry:1".into(),
            occurrence: CareOccurrence::new(
                ScheduleId("schedule:1".into()),
                MemberId("alice".into()),
                OccurrenceWindow { start_micros: 100, end_micros: 200 },
                Some(30),
            ).unwrap(),
        }
    }
    fn authority(assignment: AssignmentAuthorityState, time: TimeAuthorityState) -> OccurrenceAuthorityAssessment {
        let s = subject();
        OccurrenceAuthorityAssessment { occurrence_ref: s.occurrence_ref, occurrence_id: s.occurrence.id.0, assignment, time }
    }
    fn current() -> OccurrenceAuthorityAssessment {
        authority(
            AssignmentAuthorityState::Current { canonical_record_id: RecordId("assignment:1".into()), duplicate_count: 0 },
            TimeAuthorityState::Current { canonical_record_ref: "time:1".into(), duplicate_count: 0 },
        )
    }
    fn completion(state: CompletionAuthorityState) -> CompletionAuthorityAssessment {
        let s = subject();
        CompletionAuthorityAssessment { occurrence_ref: s.occurrence_ref, occurrence_id: s.occurrence.id.0, state }
    }
    fn item(a: OccurrenceAuthorityAssessment, c: CompletionAuthorityAssessment) -> CareDigestV4Item {
        CareDigestV4Item { subject: subject(), occurrence_authority: a, completion_authority: c }
    }
    fn qualified() -> CompletionAuthorityAssessment {
        completion(CompletionAuthorityState::Qualified {
            self_authored_record_id: RecordId("completion:1".into()),
            performed_by: "alice".into(), actual_minutes: Some(25), duplicate_count: 0,
        })
    }

    #[test]
    fn qualified_work_preserves_actual_and_estimate_separately() {
        let digest = build_digest_v4(&[item(current(), qualified())]).unwrap();
        let stats = &digest.by_member[&MemberId("alice".into())];
        assert_eq!(stats.known_actual_minutes, 25);
        assert_eq!(stats.estimated_minutes_for_completed_tasks, 30);
    }

    #[test]
    fn stale_time_never_enters_workload_history() {
        let a = authority(
            AssignmentAuthorityState::Current { canonical_record_id: RecordId("assignment:1".into()), duplicate_count: 0 },
            TimeAuthorityState::Stale { canonical_record_ref: "time:1".into(), evidence_recurrence_state_ref: "r1".into(), current_recurrence_state_ref: "r2".into(), duplicate_count: 0 },
        );
        let digest = build_digest_v4(&[item(
            a.clone(),
            completion(CompletionAuthorityState::OccurrenceUnqualified { assignment: a.assignment.clone(), time: a.time.clone() }),
        )]).unwrap();
        assert_eq!(digest.time_stale_count, 1);
        assert!(digest.by_member.is_empty());
    }

    #[test]
    fn forged_qualified_completion_over_stale_occurrence_is_rejected() {
        let a = authority(
            AssignmentAuthorityState::Current { canonical_record_id: RecordId("assignment:1".into()), duplicate_count: 0 },
            TimeAuthorityState::Stale { canonical_record_ref: "time:1".into(), evidence_recurrence_state_ref: "r1".into(), current_recurrence_state_ref: "r2".into(), duplicate_count: 0 },
        );
        assert_eq!(build_digest_v4(&[item(a, qualified())]), Err(CareDigestV4Error::CompletionAssessmentInconsistent));
    }

    #[test]
    fn duplicate_logical_item_cannot_inflate_history() {
        let one = item(current(), qualified());
        assert!(matches!(
            build_digest_v4(&[one.clone(), one]),
            Err(CareDigestV4Error::DuplicateOccurrenceRef(_))
        ));
    }

    #[test]
    fn third_party_and_unknown_duration_do_not_become_guesses() {
        let mut unknown = qualified();
        if let CompletionAuthorityState::Qualified { actual_minutes, .. } = &mut unknown.state {
            *actual_minutes = None;
        }
        let guardian = completion(CompletionAuthorityState::ThirdPartyAttestation {
            canonical_record_id: RecordId("completion:g".into()),
            performed_by: "alice".into(), recorded_by: "guardian".into(), duplicate_count: 0,
        });
        let mut other_subject = subject();
        other_subject.occurrence_ref = "occurrence:entry:2".into();
        other_subject.occurrence = CareOccurrence::new(
            ScheduleId("schedule:1".into()), MemberId("alice".into()),
            OccurrenceWindow { start_micros: 300, end_micros: 400 }, Some(30),
        ).unwrap();
        let mut other_auth = current();
        other_auth.occurrence_ref = other_subject.occurrence_ref.clone();
        other_auth.occurrence_id = other_subject.occurrence.id.0.clone();
        let mut other_completion = guardian;
        other_completion.occurrence_ref = other_subject.occurrence_ref.clone();
        other_completion.occurrence_id = other_subject.occurrence.id.0.clone();

        let digest = build_digest_v4(&[
            item(current(), unknown),
            CareDigestV4Item { subject: other_subject, occurrence_authority: other_auth, completion_authority: other_completion },
        ]).unwrap();
        assert_eq!(digest.completion_third_party_attestation_count, 1);
        let stats = &digest.by_member[&MemberId("alice".into())];
        assert_eq!(stats.tasks_completed, 1);
        assert_eq!(stats.known_actual_minutes, 0);
        assert_eq!(stats.unknown_actual_duration_count, 1);
    }
}
