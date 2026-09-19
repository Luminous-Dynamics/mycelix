use crate::{
    canonicalize_completions, canonicalize_occurrences, CompletionRecord, LedgerError, MemberId,
    OccurrenceRecord,
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
