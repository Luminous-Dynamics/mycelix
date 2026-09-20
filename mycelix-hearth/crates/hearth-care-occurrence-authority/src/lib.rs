// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure authoritative read theorem for one concrete Care occurrence.
//! Assignment and recurrence/time authority remain independent channels.

use hearth_care_ledger_contract::{
    canonicalize_occurrence_assignment_bindings, AssignmentBindingError, CareOccurrence,
    OccurrenceAssignmentBindingRecord, RecordId,
};
use hearth_care_occurrence_admission::{
    CurrentAssignmentAuthority, CurrentRecurrenceAuthority,
};
use hearth_care_occurrence_time_evidence::{
    canonicalize_occurrence_recurrence_evidence, OccurrenceRecurrenceEvidenceRecord,
    TimeEvidenceError,
};
use serde::{Deserialize, Serialize};

const MAX_REF_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceAuthoritySubject {
    pub occurrence_ref: String,
    pub occurrence: CareOccurrence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum AssignmentAuthorityState {
    Missing,
    Current {
        canonical_record_id: RecordId,
        duplicate_count: u32,
    },
    Stale {
        canonical_record_id: RecordId,
        evidence_state_ref: String,
        current_state_ref: String,
        evidence_assignee: String,
        current_assignee: String,
        duplicate_count: u32,
    },
    Conflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeAuthorityState {
    Missing,
    Current {
        canonical_record_ref: String,
        duplicate_count: u32,
    },
    Stale {
        canonical_record_ref: String,
        evidence_recurrence_state_ref: String,
        current_recurrence_state_ref: String,
        duplicate_count: u32,
    },
    Conflict,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceAuthorityAssessment {
    pub occurrence_ref: String,
    pub occurrence_id: String,
    pub assignment: AssignmentAuthorityState,
    pub time: TimeAuthorityState,
}

impl OccurrenceAuthorityAssessment {
    pub fn is_qualified(&self) -> bool {
        matches!(&self.assignment, AssignmentAuthorityState::Current { .. })
            && matches!(&self.time, TimeAuthorityState::Current { .. })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OccurrenceAuthorityError {
    EmptyOccurrenceRef,
    OccurrenceRefTooLong(usize),
    InvalidOccurrence(String),
    InvalidCurrentAssignment(String),
    InvalidCurrentRecurrence(String),
    CurrentScheduleMismatch,
    ForeignAssignmentEvidence,
    ForeignTimeEvidence,
    AssignmentEvidence(String),
    TimeEvidence(String),
    AssignmentEvidenceMismatch,
    TimeEvidenceScheduleMismatch,
    TimeEvidenceWindowMismatch,
    CountOverflow,
}

impl std::fmt::Display for OccurrenceAuthorityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for OccurrenceAuthorityError {}

impl OccurrenceAuthoritySubject {
    pub fn validate(&self) -> Result<(), OccurrenceAuthorityError> {
        if self.occurrence_ref.trim().is_empty() {
            return Err(OccurrenceAuthorityError::EmptyOccurrenceRef);
        }
        if self.occurrence_ref.len() > MAX_REF_LEN {
            return Err(OccurrenceAuthorityError::OccurrenceRefTooLong(
                self.occurrence_ref.len(),
            ));
        }
        self.occurrence
            .validate()
            .map_err(|error| OccurrenceAuthorityError::InvalidOccurrence(error.to_string()))
    }
}

pub fn assess_occurrence_authority(
    subject: &OccurrenceAuthoritySubject,
    current_assignment: &CurrentAssignmentAuthority,
    current_recurrence: &CurrentRecurrenceAuthority,
    assignment_records: &[OccurrenceAssignmentBindingRecord],
    time_records: &[OccurrenceRecurrenceEvidenceRecord],
) -> Result<OccurrenceAuthorityAssessment, OccurrenceAuthorityError> {
    subject.validate()?;
    current_assignment
        .validate()
        .map_err(|e| OccurrenceAuthorityError::InvalidCurrentAssignment(e.to_string()))?;
    current_recurrence
        .validate()
        .map_err(|e| OccurrenceAuthorityError::InvalidCurrentRecurrence(e.to_string()))?;

    let schedule_ref = subject.occurrence.schedule_id.0.as_str();
    if current_assignment.schedule_ref.as_str() != schedule_ref
        || current_recurrence.schedule_ref.as_str() != schedule_ref
    {
        return Err(OccurrenceAuthorityError::CurrentScheduleMismatch);
    }

    if assignment_records
        .iter()
        .any(|r| r.binding.occurrence_id != subject.occurrence.id)
    {
        return Err(OccurrenceAuthorityError::ForeignAssignmentEvidence);
    }
    if time_records
        .iter()
        .any(|r| r.evidence.occurrence_ref != subject.occurrence_ref)
    {
        return Err(OccurrenceAuthorityError::ForeignTimeEvidence);
    }

    Ok(OccurrenceAuthorityAssessment {
        occurrence_ref: subject.occurrence_ref.clone(),
        occurrence_id: subject.occurrence.id.0.clone(),
        assignment: assess_assignment(subject, current_assignment, assignment_records)?,
        time: assess_time(subject, current_recurrence, time_records)?,
    })
}

fn assess_assignment(
    subject: &OccurrenceAuthoritySubject,
    current: &CurrentAssignmentAuthority,
    records: &[OccurrenceAssignmentBindingRecord],
) -> Result<AssignmentAuthorityState, OccurrenceAuthorityError> {
    if records.is_empty() {
        return Ok(AssignmentAuthorityState::Missing);
    }
    let canonical = match canonicalize_occurrence_assignment_bindings(records) {
        Ok(v) => v,
        Err(AssignmentBindingError::BindingConflict { occurrence_id })
            if occurrence_id == subject.occurrence.id =>
        {
            return Ok(AssignmentAuthorityState::Conflict);
        }
        Err(e) => return Err(OccurrenceAuthorityError::AssignmentEvidence(e.to_string())),
    };
    let Some(item) = canonical.get(&subject.occurrence.id) else {
        return Ok(AssignmentAuthorityState::Missing);
    };
    item.canonical
        .binding
        .validate_against(&subject.occurrence)
        .map_err(|_| OccurrenceAuthorityError::AssignmentEvidenceMismatch)?;

    let duplicates = u32::try_from(item.duplicate_record_ids.len())
        .map_err(|_| OccurrenceAuthorityError::CountOverflow)?;
    let binding = &item.canonical.binding;
    if binding.assignment_state_ref.as_str() == current.assignment_state_ref.as_str()
        && binding.assigned_to.0.as_str() == current.assignee.as_str()
    {
        Ok(AssignmentAuthorityState::Current {
            canonical_record_id: item.canonical.record_id.clone(),
            duplicate_count: duplicates,
        })
    } else {
        Ok(AssignmentAuthorityState::Stale {
            canonical_record_id: item.canonical.record_id.clone(),
            evidence_state_ref: binding.assignment_state_ref.clone(),
            current_state_ref: current.assignment_state_ref.clone(),
            evidence_assignee: binding.assigned_to.0.clone(),
            current_assignee: current.assignee.clone(),
            duplicate_count: duplicates,
        })
    }
}

fn assess_time(
    subject: &OccurrenceAuthoritySubject,
    current: &CurrentRecurrenceAuthority,
    records: &[OccurrenceRecurrenceEvidenceRecord],
) -> Result<TimeAuthorityState, OccurrenceAuthorityError> {
    if records.is_empty() {
        return Ok(TimeAuthorityState::Missing);
    }
    let canonical = match canonicalize_occurrence_recurrence_evidence(records) {
        Ok(v) => v,
        Err(TimeEvidenceError::EvidenceConflict(occurrence_ref))
            if occurrence_ref == subject.occurrence_ref =>
        {
            return Ok(TimeAuthorityState::Conflict);
        }
        Err(TimeEvidenceError::RecordIdentityCollision(_)) => {
            return Ok(TimeAuthorityState::Conflict);
        }
        Err(e) => return Err(OccurrenceAuthorityError::TimeEvidence(e.to_string())),
    };
    let Some(item) = canonical.get(&subject.occurrence_ref) else {
        return Ok(TimeAuthorityState::Missing);
    };
    let evidence = &item.canonical.evidence;
    if evidence.schedule_ref.as_str() != subject.occurrence.schedule_id.0.as_str() {
        return Err(OccurrenceAuthorityError::TimeEvidenceScheduleMismatch);
    }
    if evidence.window_start_utc_micros != subject.occurrence.window.start_micros
        || evidence.window_end_utc_micros != subject.occurrence.window.end_micros
    {
        return Err(OccurrenceAuthorityError::TimeEvidenceWindowMismatch);
    }

    let duplicates = u32::try_from(item.duplicate_record_refs.len())
        .map_err(|_| OccurrenceAuthorityError::CountOverflow)?;
    if evidence.recurrence_state_ref.as_str() == current.recurrence_state_ref.as_str() {
        Ok(TimeAuthorityState::Current {
            canonical_record_ref: item.canonical.record_ref.clone(),
            duplicate_count: duplicates,
        })
    } else {
        Ok(TimeAuthorityState::Stale {
            canonical_record_ref: item.canonical.record_ref.clone(),
            evidence_recurrence_state_ref: evidence.recurrence_state_ref.clone(),
            current_recurrence_state_ref: current.recurrence_state_ref.clone(),
            duplicate_count: duplicates,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_ledger_contract::{
        MemberId, OccurrenceAssignmentBinding, OccurrenceWindow, ScheduleId,
    };
    use hearth_care_occurrence_time_evidence::{
        CareOccurrenceRecurrenceEvidence, OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION,
    };
    use hearth_care_recurrence::{
        CivilResolution, ExpansionEngineEvidence, InstanceDisposition, LocalDate, LocalDateTime,
        LocalTime, TimeZoneId,
    };

    fn occurrence() -> CareOccurrence {
        CareOccurrence::new(
            ScheduleId("schedule:1".into()),
            MemberId("agent:alice".into()),
            OccurrenceWindow {
                start_micros: 1_000_000,
                end_micros: 1_801_000_000,
            },
            Some(20),
        )
        .unwrap()
    }

    fn subject() -> OccurrenceAuthoritySubject {
        OccurrenceAuthoritySubject {
            occurrence_ref: "occurrence-entry:1".into(),
            occurrence: occurrence(),
        }
    }

    fn assignment_record(id: &str, state: &str) -> OccurrenceAssignmentBindingRecord {
        let occurrence = occurrence();
        OccurrenceAssignmentBindingRecord {
            record_id: RecordId(id.into()),
            binding: OccurrenceAssignmentBinding::from_occurrence(&occurrence, state.into())
                .unwrap(),
        }
    }

    fn local(hour: u8) -> LocalDateTime {
        LocalDateTime {
            date: LocalDate {
                year: 2026,
                month: 9,
                day: 21,
            },
            time: LocalTime {
                hour,
                minute: 0,
                second: 0,
            },
        }
    }

    fn time_record(id: &str, state: &str) -> OccurrenceRecurrenceEvidenceRecord {
        let occurrence = occurrence();
        OccurrenceRecurrenceEvidenceRecord {
            record_ref: id.into(),
            evidence: CareOccurrenceRecurrenceEvidence {
                schema_version: OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION,
                occurrence_ref: "occurrence-entry:1".into(),
                schedule_ref: "schedule:1".into(),
                recurrence_state_ref: state.into(),
                instance_key: "instance:1".into(),
                timezone: TimeZoneId("Africa/Johannesburg".into()),
                engine: ExpansionEngineEvidence {
                    engine_id: "engine".into(),
                    engine_version: "1".into(),
                    tzdb_ref: "iana:2026c".into(),
                },
                original_start_local: local(18),
                requested_start_local: local(18),
                effective_start_local: local(18),
                window_start_utc_micros: occurrence.window.start_micros,
                window_end_utc_micros: occurrence.window.end_micros,
                resolution: CivilResolution::Exact,
                disposition: InstanceDisposition::Scheduled,
            },
        }
    }

    fn current_assignment(state: &str) -> CurrentAssignmentAuthority {
        CurrentAssignmentAuthority {
            schedule_ref: "schedule:1".into(),
            assignment_state_ref: state.into(),
            assignee: "agent:alice".into(),
        }
    }

    fn current_recurrence(state: &str) -> CurrentRecurrenceAuthority {
        CurrentRecurrenceAuthority {
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: state.into(),
        }
    }

    #[test]
    fn current_plus_current_is_the_only_qualified_state() {
        let result = assess_occurrence_authority(
            &subject(),
            &current_assignment("assignment:a1"),
            &current_recurrence("recurrence:r1"),
            &[assignment_record("assign:1", "assignment:a1")],
            &[time_record("time:1", "recurrence:r1")],
        )
        .unwrap();
        assert!(result.is_qualified());
    }

    #[test]
    fn same_values_do_not_refresh_stale_state_identity() {
        let result = assess_occurrence_authority(
            &subject(),
            &current_assignment("assignment:a2"),
            &current_recurrence("recurrence:r2"),
            &[assignment_record("assign:1", "assignment:a1")],
            &[time_record("time:1", "recurrence:r1")],
        )
        .unwrap();
        assert!(!result.is_qualified());
        assert!(matches!(result.assignment, AssignmentAuthorityState::Stale { .. }));
        assert!(matches!(result.time, TimeAuthorityState::Stale { .. }));
    }

    #[test]
    fn conflicts_remain_independent() {
        let result = assess_occurrence_authority(
            &subject(),
            &current_assignment("assignment:a2"),
            &current_recurrence("recurrence:r2"),
            &[
                assignment_record("assign:1", "assignment:a1"),
                assignment_record("assign:2", "assignment:a2"),
            ],
            &[
                time_record("time:1", "recurrence:r1"),
                time_record("time:2", "recurrence:r2"),
            ],
        )
        .unwrap();
        assert_eq!(result.assignment, AssignmentAuthorityState::Conflict);
        assert_eq!(result.time, TimeAuthorityState::Conflict);
    }

    #[test]
    fn harmless_duplicates_are_counted() {
        let result = assess_occurrence_authority(
            &subject(),
            &current_assignment("assignment:a1"),
            &current_recurrence("recurrence:r1"),
            &[
                assignment_record("assign:a", "assignment:a1"),
                assignment_record("assign:b", "assignment:a1"),
            ],
            &[
                time_record("time:a", "recurrence:r1"),
                time_record("time:b", "recurrence:r1"),
            ],
        )
        .unwrap();
        assert!(result.is_qualified());
        assert!(matches!(
            result.assignment,
            AssignmentAuthorityState::Current { duplicate_count: 1, .. }
        ));
        assert!(matches!(
            result.time,
            TimeAuthorityState::Current { duplicate_count: 1, .. }
        ));
    }

    #[test]
    fn mismatched_time_window_is_not_staleness() {
        let mut time = time_record("time:1", "recurrence:r1");
        time.evidence.window_end_utc_micros += 1;
        assert_eq!(
            assess_occurrence_authority(
                &subject(),
                &current_assignment("assignment:a1"),
                &current_recurrence("recurrence:r1"),
                &[],
                &[time],
            ),
            Err(OccurrenceAuthorityError::TimeEvidenceWindowMismatch)
        );
    }
}
