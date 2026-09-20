// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure stale-safe admission of recurrence evidence into a Care occurrence candidate.

use hearth_care_recurrence::{
    CareRecurrenceSpec, CivilResolution, ExpansionEngineEvidence, ExpansionReceipt,
    InstanceDisposition, LocalDateTime, TimeZoneId,
};
use serde::{Deserialize, Serialize};

const MAX_REF_LEN: usize = 1024;
const MAX_ASSIGNEE_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentRecurrenceAuthority {
    pub schedule_ref: String,
    pub recurrence_state_ref: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CurrentAssignmentAuthority {
    pub schedule_ref: String,
    pub assignment_state_ref: String,
    pub assignee: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceAdmissionRequest {
    pub instance_key: String,
    /// Optional planner/task-effort estimate. This is distinct from the
    /// recurrence window duration and is not inferred here.
    pub estimated_minutes: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdmittedCareOccurrence {
    pub schedule_ref: String,
    pub recurrence_state_ref: String,
    pub assignment_state_ref: String,
    pub assignee: String,
    pub instance_key: String,
    pub timezone: TimeZoneId,
    pub engine: ExpansionEngineEvidence,
    pub original_start_local: LocalDateTime,
    pub requested_start_local: LocalDateTime,
    pub effective_start_local: LocalDateTime,
    pub window_start_utc_micros: i64,
    pub window_end_utc_micros: i64,
    pub resolution: CivilResolution,
    pub disposition: InstanceDisposition,
    pub estimated_minutes: Option<u32>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionError {
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, len: usize },
    ScheduleMismatch,
    StaleRecurrenceState,
    StaleAssignmentState,
    ReceiptInvalid(String),
    InstanceNotFound(String),
    DuplicateInstance(String),
    InvalidWindow,
    InvalidEstimate(u32),
}

impl std::fmt::Display for AdmissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for AdmissionError {}

impl CurrentRecurrenceAuthority {
    pub fn validate(&self) -> Result<(), AdmissionError> {
        validate_ref("schedule_ref", &self.schedule_ref, MAX_REF_LEN)?;
        validate_ref(
            "recurrence_state_ref",
            &self.recurrence_state_ref,
            MAX_REF_LEN,
        )
    }
}

impl CurrentAssignmentAuthority {
    pub fn validate(&self) -> Result<(), AdmissionError> {
        validate_ref("schedule_ref", &self.schedule_ref, MAX_REF_LEN)?;
        validate_ref(
            "assignment_state_ref",
            &self.assignment_state_ref,
            MAX_REF_LEN,
        )?;
        validate_ref("assignee", &self.assignee, MAX_ASSIGNEE_LEN)
    }
}

impl OccurrenceAdmissionRequest {
    pub fn validate(&self) -> Result<(), AdmissionError> {
        validate_ref("instance_key", &self.instance_key, MAX_REF_LEN)?;
        if let Some(minutes) = self.estimated_minutes {
            if minutes == 0 || minutes > 7 * 24 * 60 {
                return Err(AdmissionError::InvalidEstimate(minutes));
            }
        }
        Ok(())
    }
}

/// Admit exactly one recurrence instance for later Care-ledger materialization.
///
/// This theorem intentionally takes the *current* recurrence and assignment
/// authority snapshots separately from the receipt. A receipt from an older
/// recurrence revision cannot become current merely because its clock window
/// happens to match the latest revision.
pub fn admit_occurrence(
    spec: &CareRecurrenceSpec,
    receipt: &ExpansionReceipt,
    recurrence: &CurrentRecurrenceAuthority,
    assignment: &CurrentAssignmentAuthority,
    request: &OccurrenceAdmissionRequest,
) -> Result<AdmittedCareOccurrence, AdmissionError> {
    recurrence.validate()?;
    assignment.validate()?;
    request.validate()?;

    if recurrence.schedule_ref != spec.schedule_ref
        || assignment.schedule_ref != spec.schedule_ref
    {
        return Err(AdmissionError::ScheduleMismatch);
    }
    if recurrence.recurrence_state_ref != spec.recurrence_state_ref {
        return Err(AdmissionError::StaleRecurrenceState);
    }
    if receipt.schedule_ref != recurrence.schedule_ref
        || receipt.recurrence_state_ref != recurrence.recurrence_state_ref
    {
        return Err(AdmissionError::StaleRecurrenceState);
    }

    receipt
        .validate_against(spec)
        .map_err(|error| AdmissionError::ReceiptInvalid(error.to_string()))?;

    let mut matched = receipt
        .occurrences
        .iter()
        .filter(|value| value.instance_key == request.instance_key);
    let occurrence = matched
        .next()
        .ok_or_else(|| AdmissionError::InstanceNotFound(request.instance_key.clone()))?;
    if matched.next().is_some() {
        return Err(AdmissionError::DuplicateInstance(
            request.instance_key.clone(),
        ));
    }
    if occurrence.end_utc_micros <= occurrence.start_utc_micros {
        return Err(AdmissionError::InvalidWindow);
    }

    Ok(AdmittedCareOccurrence {
        schedule_ref: spec.schedule_ref.clone(),
        recurrence_state_ref: recurrence.recurrence_state_ref.clone(),
        assignment_state_ref: assignment.assignment_state_ref.clone(),
        assignee: assignment.assignee.clone(),
        instance_key: occurrence.instance_key.clone(),
        timezone: receipt.timezone.clone(),
        engine: receipt.engine.clone(),
        original_start_local: occurrence.original_start_local.clone(),
        requested_start_local: occurrence.requested_start_local.clone(),
        effective_start_local: occurrence.effective_start_local.clone(),
        window_start_utc_micros: occurrence.start_utc_micros,
        window_end_utc_micros: occurrence.end_utc_micros,
        resolution: occurrence.resolution.clone(),
        disposition: occurrence.disposition.clone(),
        estimated_minutes: request.estimated_minutes,
    })
}

/// Optional helper for callers that want to prove an expected assignment
/// snapshot before admission. This keeps stale-state comparison explicit rather
/// than allowing a matching assignee string to substitute for state identity.
pub fn require_assignment_state(
    current: &CurrentAssignmentAuthority,
    expected_state_ref: &str,
) -> Result<(), AdmissionError> {
    current.validate()?;
    validate_ref(
        "expected_assignment_state_ref",
        expected_state_ref,
        MAX_REF_LEN,
    )?;
    if current.assignment_state_ref != expected_state_ref {
        return Err(AdmissionError::StaleAssignmentState);
    }
    Ok(())
}

fn validate_ref(
    field: &'static str,
    value: &str,
    max_len: usize,
) -> Result<(), AdmissionError> {
    if value.trim().is_empty() {
        return Err(AdmissionError::EmptyField(field));
    }
    if value.len() > max_len {
        return Err(AdmissionError::FieldTooLong {
            field,
            len: value.len(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_care_recurrence::{
        CivilTimePolicy, ExpansionRequest, FoldPolicy, GapPolicy, LocalDate, LocalTime,
        MissedOccurrencePolicy, RecurrenceEnd, RecurrencePattern, ResolvedOccurrence,
        RECURRENCE_SCHEMA_VERSION,
    };

    fn local(year: i32, month: u8, day: u8, hour: u8, minute: u8) -> LocalDateTime {
        LocalDateTime {
            date: LocalDate { year, month, day },
            time: LocalTime {
                hour,
                minute,
                second: 0,
            },
        }
    }

    fn spec(state: &str) -> CareRecurrenceSpec {
        CareRecurrenceSpec {
            schema_version: RECURRENCE_SCHEMA_VERSION,
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: state.into(),
            timezone: TimeZoneId("Africa/Johannesburg".into()),
            starts_local: local(2026, 9, 21, 18, 0),
            duration_minutes: 30,
            pattern: RecurrencePattern::Daily { interval: 1 },
            end: RecurrenceEnd::Count(2),
            civil_time_policy: CivilTimePolicy {
                gap: GapPolicy::Reject,
                fold: FoldPolicy::Reject,
            },
            missed_policy: MissedOccurrencePolicy::SurfaceForReview,
            exclusions: Vec::new(),
            overrides: Vec::new(),
        }
    }

    fn receipt(spec: &CareRecurrenceSpec) -> ExpansionReceipt {
        let original = spec.starts_local.clone();
        ExpansionReceipt {
            schema_version: RECURRENCE_SCHEMA_VERSION,
            schedule_ref: spec.schedule_ref.clone(),
            recurrence_state_ref: spec.recurrence_state_ref.clone(),
            timezone: spec.timezone.clone(),
            engine: ExpansionEngineEvidence {
                engine_id: "test-engine".into(),
                engine_version: "1".into(),
                tzdb_ref: "iana:test".into(),
            },
            request: ExpansionRequest {
                range_start_utc_micros: 0,
                range_end_utc_micros: 10_000_000_000,
                max_instances: 10,
            },
            occurrences: vec![ResolvedOccurrence {
                instance_key: spec.instance_key(&original).unwrap(),
                original_start_local: original.clone(),
                requested_start_local: original.clone(),
                effective_start_local: original,
                start_utc_micros: 1_000_000,
                end_utc_micros: 1_000_000 + 30 * 60_000_000,
                resolution: CivilResolution::Exact,
                disposition: InstanceDisposition::Scheduled,
            }],
        }
    }

    fn recurrence(state: &str) -> CurrentRecurrenceAuthority {
        CurrentRecurrenceAuthority {
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: state.into(),
        }
    }

    fn assignment(state: &str) -> CurrentAssignmentAuthority {
        CurrentAssignmentAuthority {
            schedule_ref: "schedule:1".into(),
            assignment_state_ref: state.into(),
            assignee: "agent:alice".into(),
        }
    }

    #[test]
    fn exact_current_evidence_is_admitted() {
        let spec = spec("recurrence:r1");
        let receipt = receipt(&spec);
        let request = OccurrenceAdmissionRequest {
            instance_key: receipt.occurrences[0].instance_key.clone(),
            estimated_minutes: Some(20),
        };
        let admitted = admit_occurrence(
            &spec,
            &receipt,
            &recurrence("recurrence:r1"),
            &assignment("assignment:a1"),
            &request,
        )
        .unwrap();
        assert_eq!(admitted.recurrence_state_ref, "recurrence:r1");
        assert_eq!(admitted.assignment_state_ref, "assignment:a1");
        assert_eq!(admitted.assignee, "agent:alice");
        assert_eq!(admitted.engine, receipt.engine);
    }

    #[test]
    fn old_receipt_is_rejected_after_recurrence_advances_even_if_time_matches() {
        let old_spec = spec("recurrence:r1");
        let old_receipt = receipt(&old_spec);
        let new_spec = spec("recurrence:r2");
        let request = OccurrenceAdmissionRequest {
            instance_key: old_receipt.occurrences[0].instance_key.clone(),
            estimated_minutes: None,
        };
        assert_eq!(
            admit_occurrence(
                &new_spec,
                &old_receipt,
                &recurrence("recurrence:r2"),
                &assignment("assignment:a1"),
                &request,
            ),
            Err(AdmissionError::StaleRecurrenceState)
        );
    }

    #[test]
    fn assignment_state_identity_is_not_interchangeable_with_same_assignee() {
        let current = assignment("assignment:a2");
        assert_eq!(
            require_assignment_state(&current, "assignment:a1"),
            Err(AdmissionError::StaleAssignmentState)
        );
        assert_eq!(current.assignee, "agent:alice");
    }

    #[test]
    fn unknown_instance_cannot_substitute_arbitrary_window() {
        let spec = spec("recurrence:r1");
        let receipt = receipt(&spec);
        let request = OccurrenceAdmissionRequest {
            instance_key: "care-recur-v1|unknown".into(),
            estimated_minutes: None,
        };
        assert!(matches!(
            admit_occurrence(
                &spec,
                &receipt,
                &recurrence("recurrence:r1"),
                &assignment("assignment:a1"),
                &request,
            ),
            Err(AdmissionError::InstanceNotFound(_))
        ));
    }

    #[test]
    fn schedule_mismatch_fails_closed() {
        let spec = spec("recurrence:r1");
        let receipt = receipt(&spec);
        let request = OccurrenceAdmissionRequest {
            instance_key: receipt.occurrences[0].instance_key.clone(),
            estimated_minutes: None,
        };
        let wrong_assignment = CurrentAssignmentAuthority {
            schedule_ref: "schedule:other".into(),
            assignment_state_ref: "assignment:a1".into(),
            assignee: "agent:alice".into(),
        };
        assert_eq!(
            admit_occurrence(
                &spec,
                &receipt,
                &recurrence("recurrence:r1"),
                &wrong_assignment,
                &request,
            ),
            Err(AdmissionError::ScheduleMismatch)
        );
    }

    #[test]
    fn invalid_effort_estimate_is_not_smuggled_through_time_evidence() {
        let spec = spec("recurrence:r1");
        let receipt = receipt(&spec);
        let request = OccurrenceAdmissionRequest {
            instance_key: receipt.occurrences[0].instance_key.clone(),
            estimated_minutes: Some(0),
        };
        assert_eq!(
            admit_occurrence(
                &spec,
                &receipt,
                &recurrence("recurrence:r1"),
                &assignment("assignment:a1"),
                &request,
            ),
            Err(AdmissionError::InvalidEstimate(0))
        );
    }
}
