use crate::{CareOccurrence, LedgerError, MemberId, OccurrenceId, RecordId, ScheduleId};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const OCCURRENCE_ASSIGNMENT_BINDING_SCHEMA_VERSION: u16 = 1;
const MAX_STATE_REF_LEN: usize = 1024;

/// Exact responsibility state that authorized one concrete Care occurrence.
///
/// The occurrence identity remains `(schedule, window)`. This binding records
/// which assignment state justified the occurrence's assignee without changing
/// that real-world task identity.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceAssignmentBinding {
    pub schema_version: u16,
    pub occurrence_id: OccurrenceId,
    pub schedule_id: ScheduleId,
    pub assignment_state_ref: String,
    pub assigned_to: MemberId,
}

impl OccurrenceAssignmentBinding {
    pub fn from_occurrence(
        occurrence: &CareOccurrence,
        assignment_state_ref: String,
    ) -> Result<Self, AssignmentBindingError> {
        occurrence.validate().map_err(AssignmentBindingError::Ledger)?;
        validate_state_ref(&assignment_state_ref)?;
        let binding = Self {
            schema_version: OCCURRENCE_ASSIGNMENT_BINDING_SCHEMA_VERSION,
            occurrence_id: occurrence.id.clone(),
            schedule_id: occurrence.schedule_id.clone(),
            assignment_state_ref,
            assigned_to: occurrence.assigned_to.clone(),
        };
        binding.validate_against(occurrence)?;
        Ok(binding)
    }

    pub fn validate(&self) -> Result<(), AssignmentBindingError> {
        if self.schema_version != OCCURRENCE_ASSIGNMENT_BINDING_SCHEMA_VERSION {
            return Err(AssignmentBindingError::UnsupportedSchema(self.schema_version));
        }
        validate_state_ref(&self.assignment_state_ref)?;
        if self.occurrence_id.0.trim().is_empty() {
            return Err(AssignmentBindingError::EmptyOccurrenceId);
        }
        if self.schedule_id.0.trim().is_empty() {
            return Err(AssignmentBindingError::EmptyScheduleId);
        }
        if self.assigned_to.0.trim().is_empty() {
            return Err(AssignmentBindingError::EmptyAssignee);
        }
        Ok(())
    }

    pub fn validate_against(
        &self,
        occurrence: &CareOccurrence,
    ) -> Result<(), AssignmentBindingError> {
        self.validate()?;
        occurrence.validate().map_err(AssignmentBindingError::Ledger)?;
        if self.occurrence_id != occurrence.id {
            return Err(AssignmentBindingError::OccurrenceMismatch);
        }
        if self.schedule_id != occurrence.schedule_id {
            return Err(AssignmentBindingError::ScheduleMismatch);
        }
        if self.assigned_to != occurrence.assigned_to {
            return Err(AssignmentBindingError::AssigneeMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceAssignmentBindingRecord {
    pub record_id: RecordId,
    pub binding: OccurrenceAssignmentBinding,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanonicalOccurrenceAssignmentBinding {
    pub canonical: OccurrenceAssignmentBindingRecord,
    pub duplicate_record_ids: Vec<RecordId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssignmentBindingError {
    UnsupportedSchema(u16),
    EmptyOccurrenceId,
    EmptyScheduleId,
    EmptyAssignee,
    EmptyAssignmentStateRef,
    AssignmentStateRefTooLong(usize),
    OccurrenceMismatch,
    ScheduleMismatch,
    AssigneeMismatch,
    EmptyRecordId,
    BindingConflict { occurrence_id: OccurrenceId },
    Ledger(LedgerError),
}

impl std::fmt::Display for AssignmentBindingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for AssignmentBindingError {}

/// Canonicalize binding evidence without treating DHT arrival order as truth.
///
/// Repeated identical evidence collapses deterministically. Two different
/// assignment-state bindings for the same occurrence are an authority conflict,
/// even if both happen to name the same assignee.
pub fn canonicalize_occurrence_assignment_bindings(
    records: &[OccurrenceAssignmentBindingRecord],
) -> Result<
    BTreeMap<OccurrenceId, CanonicalOccurrenceAssignmentBinding>,
    AssignmentBindingError,
> {
    let mut grouped: BTreeMap<OccurrenceId, Vec<OccurrenceAssignmentBindingRecord>> =
        BTreeMap::new();
    let mut seen_records = BTreeSet::new();

    for record in records {
        if record.record_id.0.trim().is_empty() {
            return Err(AssignmentBindingError::EmptyRecordId);
        }
        record.binding.validate()?;
        if !seen_records.insert(record.record_id.clone()) {
            continue;
        }
        grouped
            .entry(record.binding.occurrence_id.clone())
            .or_default()
            .push(record.clone());
    }

    let mut out = BTreeMap::new();
    for (occurrence_id, mut group) in grouped {
        group.sort_by(|left, right| left.record_id.cmp(&right.record_id));
        let canonical = group[0].clone();
        if group
            .iter()
            .skip(1)
            .any(|other| other.binding != canonical.binding)
        {
            return Err(AssignmentBindingError::BindingConflict { occurrence_id });
        }
        let duplicate_record_ids = group
            .iter()
            .skip(1)
            .map(|record| record.record_id.clone())
            .collect();
        out.insert(
            occurrence_id,
            CanonicalOccurrenceAssignmentBinding {
                canonical,
                duplicate_record_ids,
            },
        );
    }
    Ok(out)
}

fn validate_state_ref(value: &str) -> Result<(), AssignmentBindingError> {
    if value.trim().is_empty() {
        return Err(AssignmentBindingError::EmptyAssignmentStateRef);
    }
    if value.len() > MAX_STATE_REF_LEN {
        return Err(AssignmentBindingError::AssignmentStateRefTooLong(value.len()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{OccurrenceWindow, CARE_LEDGER_SCHEMA_VERSION};

    fn occurrence(assignee: &str) -> CareOccurrence {
        CareOccurrence::new(
            ScheduleId("schedule:1".into()),
            MemberId(assignee.into()),
            OccurrenceWindow {
                start_micros: 100,
                end_micros: 200,
            },
            Some(30),
        )
        .unwrap()
    }

    #[test]
    fn binding_preserves_real_world_occurrence_identity() {
        let before = occurrence("alex");
        let after = occurrence("mira");
        assert_eq!(before.id, after.id);
        assert_eq!(before.schema_version, CARE_LEDGER_SCHEMA_VERSION);

        let left = OccurrenceAssignmentBinding::from_occurrence(&before, "state:a".into()).unwrap();
        let right = OccurrenceAssignmentBinding::from_occurrence(&after, "state:b".into()).unwrap();
        assert_eq!(left.occurrence_id, right.occurrence_id);
        assert_ne!(left, right);
    }

    #[test]
    fn binding_must_match_occurrence_assignee() {
        let occurrence = occurrence("alex");
        let mut binding =
            OccurrenceAssignmentBinding::from_occurrence(&occurrence, "state:a".into()).unwrap();
        binding.assigned_to = MemberId("mira".into());
        assert_eq!(
            binding.validate_against(&occurrence),
            Err(AssignmentBindingError::AssigneeMismatch)
        );
    }

    #[test]
    fn identical_duplicate_bindings_do_not_inflate_authority() {
        let occurrence = occurrence("alex");
        let binding =
            OccurrenceAssignmentBinding::from_occurrence(&occurrence, "state:a".into()).unwrap();
        let records = vec![
            OccurrenceAssignmentBindingRecord {
                record_id: RecordId("record:b".into()),
                binding: binding.clone(),
            },
            OccurrenceAssignmentBindingRecord {
                record_id: RecordId("record:a".into()),
                binding,
            },
        ];
        let canonical = canonicalize_occurrence_assignment_bindings(&records).unwrap();
        let item = canonical.get(&occurrence.id).unwrap();
        assert_eq!(item.canonical.record_id, RecordId("record:a".into()));
        assert_eq!(item.duplicate_record_ids, vec![RecordId("record:b".into())]);
    }

    #[test]
    fn different_state_refs_for_one_occurrence_fail_closed() {
        let occurrence = occurrence("alex");
        let records = vec![
            OccurrenceAssignmentBindingRecord {
                record_id: RecordId("record:a".into()),
                binding: OccurrenceAssignmentBinding::from_occurrence(
                    &occurrence,
                    "state:a".into(),
                )
                .unwrap(),
            },
            OccurrenceAssignmentBindingRecord {
                record_id: RecordId("record:b".into()),
                binding: OccurrenceAssignmentBinding::from_occurrence(
                    &occurrence,
                    "state:b".into(),
                )
                .unwrap(),
            },
        ];
        assert!(matches!(
            canonicalize_occurrence_assignment_bindings(&records),
            Err(AssignmentBindingError::BindingConflict { .. })
        ));
    }
}
