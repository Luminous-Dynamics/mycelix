// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure completion-authority theorem over fully qualified Care occurrences.

use hearth_care_ledger_contract::{
    canonicalize_completions, CompletionRecord, LedgerError, RecordId,
};
use hearth_care_occurrence_authority::{
    AssignmentAuthorityState, OccurrenceAuthorityAssessment, OccurrenceAuthoritySubject,
    TimeAuthorityState,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompletionAuthorityState {
    Missing,
    OccurrenceUnqualified {
        assignment: AssignmentAuthorityState,
        time: TimeAuthorityState,
    },
    Conflict,
    PerformerMismatch {
        canonical_record_id: RecordId,
        performed_by: String,
        assigned_to: String,
        duplicate_count: u32,
    },
    ThirdPartyAttestation {
        canonical_record_id: RecordId,
        performed_by: String,
        recorded_by: String,
        duplicate_count: u32,
    },
    Qualified {
        self_authored_record_id: RecordId,
        performed_by: String,
        actual_minutes: Option<u32>,
        duplicate_count: u32,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CompletionAuthorityAssessment {
    pub occurrence_ref: String,
    pub occurrence_id: String,
    pub state: CompletionAuthorityState,
}

impl CompletionAuthorityAssessment {
    pub fn is_qualified(&self) -> bool {
        matches!(&self.state, CompletionAuthorityState::Qualified { .. })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompletionAuthorityError {
    InvalidSubject(String),
    OccurrenceAssessmentMismatch,
    ForeignCompletionEvidence,
    Ledger(String),
    CountOverflow,
}

impl std::fmt::Display for CompletionAuthorityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for CompletionAuthorityError {}

/// Assess completion evidence for one already-assessed Care occurrence.
///
/// The supplied occurrence assessment is intentionally a prerequisite: a
/// completion cannot upgrade stale/missing/conflicted occurrence authority.
pub fn assess_completion_authority(
    subject: &OccurrenceAuthoritySubject,
    occurrence_authority: &OccurrenceAuthorityAssessment,
    completion_records: &[CompletionRecord],
) -> Result<CompletionAuthorityAssessment, CompletionAuthorityError> {
    subject
        .validate()
        .map_err(|error| CompletionAuthorityError::InvalidSubject(error.to_string()))?;

    if occurrence_authority.occurrence_ref != subject.occurrence_ref
        || occurrence_authority.occurrence_id != subject.occurrence.id.0
    {
        return Err(CompletionAuthorityError::OccurrenceAssessmentMismatch);
    }

    if completion_records
        .iter()
        .any(|record| record.completion.occurrence_id != subject.occurrence.id)
    {
        return Err(CompletionAuthorityError::ForeignCompletionEvidence);
    }

    if !occurrence_authority.is_qualified() {
        return Ok(assessment(
            subject,
            CompletionAuthorityState::OccurrenceUnqualified {
                assignment: occurrence_authority.assignment.clone(),
                time: occurrence_authority.time.clone(),
            },
        ));
    }

    if completion_records.is_empty() {
        return Ok(assessment(subject, CompletionAuthorityState::Missing));
    }

    let canonical = canonicalize_completions(completion_records)
        .map_err(|error| CompletionAuthorityError::Ledger(error.to_string()))?;
    let Some(completion) = canonical.get(&subject.occurrence.id) else {
        return Ok(assessment(subject, CompletionAuthorityState::Missing));
    };

    if completion.semantic_conflict {
        return Ok(assessment(subject, CompletionAuthorityState::Conflict));
    }

    let duplicate_count = u32::try_from(completion.duplicate_record_ids.len())
        .map_err(|_| CompletionAuthorityError::CountOverflow)?;
    let canonical_record = &completion.canonical;
    let performed_by = &canonical_record.completion.performed_by;
    let assigned_to = &subject.occurrence.assigned_to;

    if performed_by != assigned_to {
        return Ok(assessment(
            subject,
            CompletionAuthorityState::PerformerMismatch {
                canonical_record_id: canonical_record.record_id.clone(),
                performed_by: performed_by.0.clone(),
                assigned_to: assigned_to.0.clone(),
                duplicate_count,
            },
        ));
    }

    // Different recorders may independently attest the same underlying work.
    // Once performer/duration semantics are known to agree, prefer the
    // performer's own source-chain evidence if it exists rather than allowing
    // stable record ordering to demote self-authored evidence.
    let self_authored_record = completion_records
        .iter()
        .filter(|record| {
            record.completion.performed_by == *assigned_to
                && record.completion.recorded_by == record.completion.performed_by
                && record.completion.actual_minutes == canonical_record.completion.actual_minutes
        })
        .min_by(|left, right| left.record_id.cmp(&right.record_id));

    if let Some(record) = self_authored_record {
        return Ok(assessment(
            subject,
            CompletionAuthorityState::Qualified {
                self_authored_record_id: record.record_id.clone(),
                performed_by: record.completion.performed_by.0.clone(),
                actual_minutes: record.completion.actual_minutes,
                duplicate_count,
            },
        ));
    }

    Ok(assessment(
        subject,
        CompletionAuthorityState::ThirdPartyAttestation {
            canonical_record_id: canonical_record.record_id.clone(),
            performed_by: canonical_record.completion.performed_by.0.clone(),
            recorded_by: canonical_record.completion.recorded_by.0.clone(),
            duplicate_count,
        },
    ))
}

fn assessment(
    subject: &OccurrenceAuthoritySubject,
    state: CompletionAuthorityState,
) -> CompletionAuthorityAssessment {
    CompletionAuthorityAssessment {
        occurrence_ref: subject.occurrence_ref.clone(),
        occurrence_id: subject.occurrence.id.0.clone(),
        state,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_ledger_contract::{
        CareCompletion, CareOccurrence, MemberId, OccurrenceWindow, ScheduleId,
        CARE_LEDGER_SCHEMA_VERSION,
    };

    fn subject() -> OccurrenceAuthoritySubject {
        OccurrenceAuthoritySubject {
            occurrence_ref: "occurrence-entry:1".into(),
            occurrence: CareOccurrence::new(
                ScheduleId("schedule:1".into()),
                MemberId("alice".into()),
                OccurrenceWindow {
                    start_micros: 100,
                    end_micros: 200,
                },
                Some(30),
            )
            .unwrap(),
        }
    }

    fn occurrence_authority(qualified: bool) -> OccurrenceAuthorityAssessment {
        let subject = subject();
        OccurrenceAuthorityAssessment {
            occurrence_ref: subject.occurrence_ref,
            occurrence_id: subject.occurrence.id.0,
            assignment: if qualified {
                AssignmentAuthorityState::Current {
                    canonical_record_id: RecordId("assignment:1".into()),
                    duplicate_count: 0,
                }
            } else {
                AssignmentAuthorityState::Stale {
                    canonical_record_id: RecordId("assignment:1".into()),
                    evidence_state_ref: "assignment:a1".into(),
                    current_state_ref: "assignment:a2".into(),
                    evidence_assignee: "alice".into(),
                    current_assignee: "alice".into(),
                    duplicate_count: 0,
                }
            },
            time: TimeAuthorityState::Current {
                canonical_record_ref: "time:1".into(),
                duplicate_count: 0,
            },
        }
    }

    fn completion(record_id: &str, performer: &str, recorder: &str, minutes: u32) -> CompletionRecord {
        let occurrence = subject().occurrence;
        CompletionRecord {
            record_id: RecordId(record_id.into()),
            completion: CareCompletion {
                schema_version: CARE_LEDGER_SCHEMA_VERSION,
                occurrence_id: occurrence.id,
                performed_by: MemberId(performer.into()),
                recorded_by: MemberId(recorder.into()),
                completed_at_micros: 210,
                actual_minutes: Some(minutes),
                evidence_refs: vec![],
            },
        }
    }

    #[test]
    fn fully_qualified_self_authored_completion_is_qualified() {
        let result = assess_completion_authority(
            &subject(),
            &occurrence_authority(true),
            &[completion("completion:1", "alice", "alice", 25)],
        )
        .unwrap();
        assert!(result.is_qualified());
    }

    #[test]
    fn completion_cannot_upgrade_unqualified_occurrence() {
        let result = assess_completion_authority(
            &subject(),
            &occurrence_authority(false),
            &[completion("completion:1", "alice", "alice", 25)],
        )
        .unwrap();
        assert!(!result.is_qualified());
        assert!(matches!(
            result.state,
            CompletionAuthorityState::OccurrenceUnqualified { .. }
        ));
    }

    #[test]
    fn conflicting_work_semantics_fail_closed() {
        let result = assess_completion_authority(
            &subject(),
            &occurrence_authority(true),
            &[
                completion("completion:1", "alice", "alice", 25),
                completion("completion:2", "alice", "alice", 30),
            ],
        )
        .unwrap();
        assert_eq!(result.state, CompletionAuthorityState::Conflict);
    }

    #[test]
    fn third_party_only_attestation_is_visible_but_not_qualified() {
        let result = assess_completion_authority(
            &subject(),
            &occurrence_authority(true),
            &[completion("completion:1", "alice", "guardian", 25)],
        )
        .unwrap();
        assert!(matches!(
            result.state,
            CompletionAuthorityState::ThirdPartyAttestation { .. }
        ));
    }

    #[test]
    fn self_authored_evidence_wins_over_agreeing_guardian_record_order() {
        let result = assess_completion_authority(
            &subject(),
            &occurrence_authority(true),
            &[
                completion("completion:a-guardian", "alice", "guardian", 25),
                completion("completion:z-self", "alice", "alice", 25),
            ],
        )
        .unwrap();
        assert_eq!(
            result.state,
            CompletionAuthorityState::Qualified {
                self_authored_record_id: RecordId("completion:z-self".into()),
                performed_by: "alice".into(),
                actual_minutes: Some(25),
                duplicate_count: 1,
            }
        );
    }
}
