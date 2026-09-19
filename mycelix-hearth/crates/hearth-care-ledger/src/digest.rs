use crate::{
    canonicalize_completions, canonicalize_occurrence_assignment_bindings,
    canonicalize_occurrences, AssignmentBindingError, CompletionRecord, LedgerError, MemberId,
    OccurrenceAssignmentBindingRecord, OccurrenceRecord,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV2Member {
    pub tasks_completed: u32,
    pub known_actual_minutes: u32,
    pub unknown_actual_duration_count: u32,
    /// Explicit estimate remains separate from measured/reported actual effort.
    pub estimated_minutes_for_completed_tasks: u32,
    pub completed_tasks_without_estimate: u32,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV2 {
    /// Work is attributed to `performed_by`, never merely to the recorder/attester.
    pub by_member: BTreeMap<MemberId, CareDigestV2Member>,
    pub orphan_completion_count: u32,
    pub conflicted_completion_count: u32,
    pub agreeing_duplicate_completion_count: u32,
}

/// Build an evidence-bearing digest without fabricating duration.
/// Duplicate completion records never inflate task counts.
pub fn build_digest_v2(
    occurrences: &[OccurrenceRecord],
    completions: &[CompletionRecord],
) -> Result<CareDigestV2, LedgerError> {
    let occurrences = canonicalize_occurrences(occurrences)?;
    let completions = canonicalize_completions(completions)?;
    let mut digest = CareDigestV2::default();

    for (occurrence_id, completion) in completions {
        let Some(occurrence) = occurrences.get(&occurrence_id) else {
            digest.orphan_completion_count = digest.orphan_completion_count.saturating_add(1);
            continue;
        };
        if completion.semantic_conflict {
            digest.conflicted_completion_count =
                digest.conflicted_completion_count.saturating_add(1);
            continue;
        }
        digest.agreeing_duplicate_completion_count = digest
            .agreeing_duplicate_completion_count
            .saturating_add(completion.duplicate_record_ids.len() as u32);

        let stats = digest
            .by_member
            .entry(completion.canonical.completion.performed_by.clone())
            .or_default();
        stats.tasks_completed = stats.tasks_completed.saturating_add(1);
        match completion.canonical.completion.actual_minutes {
            Some(minutes) => {
                stats.known_actual_minutes = stats.known_actual_minutes.saturating_add(minutes)
            }
            None => {
                stats.unknown_actual_duration_count =
                    stats.unknown_actual_duration_count.saturating_add(1)
            }
        }
        match occurrence.occurrence.estimated_minutes {
            Some(minutes) => {
                stats.estimated_minutes_for_completed_tasks = stats
                    .estimated_minutes_for_completed_tasks
                    .saturating_add(minutes)
            }
            None => {
                stats.completed_tasks_without_estimate =
                    stats.completed_tasks_without_estimate.saturating_add(1)
            }
        }
    }
    Ok(digest)
}

#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareDigestV3 {
    /// Only assignment-qualified, self-authored completion evidence contributes here.
    pub by_member: BTreeMap<MemberId, CareDigestV2Member>,
    pub qualified_occurrence_count: u32,
    pub unbound_occurrence_count: u32,
    pub orphan_assignment_binding_count: u32,
    pub agreeing_duplicate_assignment_binding_count: u32,
    pub qualified_completion_count: u32,
    pub unbound_completion_count: u32,
    pub orphan_completion_count: u32,
    pub conflicted_completion_count: u32,
    /// Recorder/attester differs from performer. Preserved, but not workload authority.
    pub third_party_attestation_count: u32,
    pub performer_mismatch_completion_count: u32,
    pub agreeing_duplicate_completion_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CareDigestV3Error {
    Ledger(LedgerError),
    AssignmentBinding(AssignmentBindingError),
}

impl std::fmt::Display for CareDigestV3Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for CareDigestV3Error {}

/// Build planner-grade workload history only from occurrences with one canonical
/// assignment binding and completions authored by the performer themselves.
///
/// Guardian/third-party attestations remain visible but do not become fairness
/// authority until a later performer-endorsement/delegation theorem proves it.
pub fn build_digest_v3(
    occurrences: &[OccurrenceRecord],
    assignment_bindings: &[OccurrenceAssignmentBindingRecord],
    completions: &[CompletionRecord],
) -> Result<CareDigestV3, CareDigestV3Error> {
    let occurrences = canonicalize_occurrences(occurrences).map_err(CareDigestV3Error::Ledger)?;
    let bindings = canonicalize_occurrence_assignment_bindings(assignment_bindings)
        .map_err(CareDigestV3Error::AssignmentBinding)?;
    let completions = canonicalize_completions(completions).map_err(CareDigestV3Error::Ledger)?;
    let mut digest = CareDigestV3::default();

    for (occurrence_id, binding) in &bindings {
        let Some(occurrence) = occurrences.get(occurrence_id) else {
            digest.orphan_assignment_binding_count =
                digest.orphan_assignment_binding_count.saturating_add(1);
            continue;
        };
        binding
            .canonical
            .binding
            .validate_against(&occurrence.occurrence)
            .map_err(CareDigestV3Error::AssignmentBinding)?;
        digest.agreeing_duplicate_assignment_binding_count = digest
            .agreeing_duplicate_assignment_binding_count
            .saturating_add(binding.duplicate_record_ids.len().min(u32::MAX as usize) as u32);
    }

    for occurrence_id in occurrences.keys() {
        if bindings.contains_key(occurrence_id) {
            digest.qualified_occurrence_count = digest.qualified_occurrence_count.saturating_add(1);
        } else {
            digest.unbound_occurrence_count = digest.unbound_occurrence_count.saturating_add(1);
        }
    }

    for (occurrence_id, completion) in completions {
        let Some(occurrence) = occurrences.get(&occurrence_id) else {
            digest.orphan_completion_count = digest.orphan_completion_count.saturating_add(1);
            continue;
        };
        if completion.semantic_conflict {
            digest.conflicted_completion_count =
                digest.conflicted_completion_count.saturating_add(1);
            continue;
        }
        if !bindings.contains_key(&occurrence_id) {
            digest.unbound_completion_count = digest.unbound_completion_count.saturating_add(1);
            continue;
        }
        if completion.canonical.completion.performed_by != occurrence.occurrence.assigned_to {
            digest.performer_mismatch_completion_count = digest
                .performer_mismatch_completion_count
                .saturating_add(1);
            continue;
        }
        if completion.canonical.completion.recorded_by
            != completion.canonical.completion.performed_by
        {
            digest.third_party_attestation_count =
                digest.third_party_attestation_count.saturating_add(1);
            continue;
        }

        digest.qualified_completion_count = digest.qualified_completion_count.saturating_add(1);
        digest.agreeing_duplicate_completion_count = digest
            .agreeing_duplicate_completion_count
            .saturating_add(completion.duplicate_record_ids.len().min(u32::MAX as usize) as u32);

        let stats = digest
            .by_member
            .entry(completion.canonical.completion.performed_by.clone())
            .or_default();
        stats.tasks_completed = stats.tasks_completed.saturating_add(1);
        match completion.canonical.completion.actual_minutes {
            Some(minutes) => {
                stats.known_actual_minutes = stats.known_actual_minutes.saturating_add(minutes)
            }
            None => {
                stats.unknown_actual_duration_count =
                    stats.unknown_actual_duration_count.saturating_add(1)
            }
        }
        match occurrence.occurrence.estimated_minutes {
            Some(minutes) => {
                stats.estimated_minutes_for_completed_tasks = stats
                    .estimated_minutes_for_completed_tasks
                    .saturating_add(minutes)
            }
            None => {
                stats.completed_tasks_without_estimate =
                    stats.completed_tasks_without_estimate.saturating_add(1)
            }
        }
    }

    Ok(digest)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum EffortProvenance {
    ActualCompletionEvidence,
    ExplicitOccurrenceEstimate,
    LegacyOneHourPerTask,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyEffortEstimate {
    pub tasks_completed: u32,
    pub estimated_minutes: u32,
    pub provenance: EffortProvenance,
}

/// Historical Care digests estimated one hour per completion. Preserve that
/// only as labeled legacy estimation, never as actual work evidence.
pub fn import_legacy_care_summary(
    tasks_completed: u32,
    _hours_hundredths: u32,
) -> LegacyEffortEstimate {
    LegacyEffortEstimate {
        tasks_completed,
        estimated_minutes: tasks_completed.saturating_mul(60),
        provenance: EffortProvenance::LegacyOneHourPerTask,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CareCompletion, CareOccurrence, OccurrenceAssignmentBinding, OccurrenceId,
        OccurrenceWindow, RecordId, ScheduleId,
    };

    fn occurrence() -> OccurrenceRecord {
        OccurrenceRecord {
            record_id: RecordId("occurrence-record".into()),
            occurrence: CareOccurrence::new(
                ScheduleId("schedule:1".into()),
                MemberId("alex".into()),
                OccurrenceWindow {
                    start_micros: 100,
                    end_micros: 200,
                },
                Some(30),
            )
            .unwrap(),
        }
    }

    fn completion(id: &OccurrenceId, performer: &str, recorder: &str) -> CompletionRecord {
        CompletionRecord {
            record_id: RecordId(format!("completion-record:{recorder}")),
            completion: CareCompletion {
                schema_version: crate::CARE_LEDGER_SCHEMA_VERSION,
                occurrence_id: id.clone(),
                performed_by: MemberId(performer.into()),
                recorded_by: MemberId(recorder.into()),
                completed_at_micros: 210,
                actual_minutes: Some(25),
                evidence_refs: vec![],
            },
        }
    }

    fn binding(occurrence: &OccurrenceRecord) -> OccurrenceAssignmentBindingRecord {
        OccurrenceAssignmentBindingRecord {
            record_id: RecordId("binding-record".into()),
            binding: OccurrenceAssignmentBinding::from_occurrence(
                &occurrence.occurrence,
                "assignment-state:1".into(),
            )
            .unwrap(),
        }
    }

    #[test]
    fn unbound_completion_does_not_enter_workload_totals() {
        let occurrence = occurrence();
        let completion = completion(&occurrence.occurrence.id, "alex", "alex");
        let digest = build_digest_v3(&[occurrence], &[], &[completion]).unwrap();
        assert_eq!(digest.unbound_occurrence_count, 1);
        assert_eq!(digest.unbound_completion_count, 1);
        assert!(digest.by_member.is_empty());
    }

    #[test]
    fn self_authored_qualified_completion_counts_normally() {
        let occurrence = occurrence();
        let binding = binding(&occurrence);
        let completion = completion(&occurrence.occurrence.id, "alex", "alex");
        let digest = build_digest_v3(&[occurrence], &[binding], &[completion]).unwrap();
        assert_eq!(digest.qualified_occurrence_count, 1);
        assert_eq!(digest.qualified_completion_count, 1);
        assert_eq!(digest.by_member[&MemberId("alex".into())].known_actual_minutes, 25);
    }

    #[test]
    fn third_party_attestation_is_visible_but_not_workload_authority() {
        let occurrence = occurrence();
        let binding = binding(&occurrence);
        let completion = completion(&occurrence.occurrence.id, "alex", "guardian");
        let digest = build_digest_v3(&[occurrence], &[binding], &[completion]).unwrap();
        assert_eq!(digest.third_party_attestation_count, 1);
        assert_eq!(digest.qualified_completion_count, 0);
        assert!(digest.by_member.is_empty());
    }

    #[test]
    fn performer_mismatch_is_visible_and_not_counted() {
        let occurrence = occurrence();
        let binding = binding(&occurrence);
        let completion = completion(&occurrence.occurrence.id, "mira", "mira");
        let digest = build_digest_v3(&[occurrence], &[binding], &[completion]).unwrap();
        assert_eq!(digest.performer_mismatch_completion_count, 1);
        assert!(digest.by_member.is_empty());
    }
}
