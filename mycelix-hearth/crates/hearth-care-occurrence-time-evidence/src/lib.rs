// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure content-addressed recurrence/time evidence for admitted Care occurrences.
//!
//! Assignment authority intentionally remains separate. This crate binds one
//! occurrence content reference to the exact recurrence state and civil/UTC
//! evidence that authorized its time window.

use hearth_care_occurrence_admission::AdmittedCareOccurrence;
use hearth_care_recurrence::{
    CivilResolution, ExpansionEngineEvidence, InstanceDisposition, LocalDateTime, TimeZoneId,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION: u16 = 1;
const MAX_REF_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareOccurrenceRecurrenceEvidence {
    pub schema_version: u16,
    /// Stable content identity of the CareOccurrence entry this evidence binds.
    pub occurrence_ref: String,
    pub schedule_ref: String,
    pub recurrence_state_ref: String,
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
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceRecurrenceEvidenceRecord {
    /// Stable storage/action/content reference used only for deterministic
    /// duplicate collapse. It is not part of the evidence semantics.
    pub record_ref: String,
    pub evidence: CareOccurrenceRecurrenceEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CanonicalOccurrenceRecurrenceEvidence {
    pub canonical: OccurrenceRecurrenceEvidenceRecord,
    pub duplicate_record_refs: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TimeEvidenceError {
    UnsupportedSchema(u16),
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, len: usize },
    InvalidTimeZone(String),
    InvalidEngine(String),
    InvalidLocalTime(String),
    InvalidWindow,
    ResolutionMismatch,
    DispositionMismatch,
    RecordIdentityCollision(String),
    EvidenceConflict(String),
}

impl std::fmt::Display for TimeEvidenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for TimeEvidenceError {}

impl CareOccurrenceRecurrenceEvidence {
    pub fn from_admitted(
        occurrence_ref: impl Into<String>,
        admitted: &AdmittedCareOccurrence,
    ) -> Result<Self, TimeEvidenceError> {
        let value = Self {
            schema_version: OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION,
            occurrence_ref: occurrence_ref.into(),
            schedule_ref: admitted.schedule_ref.clone(),
            recurrence_state_ref: admitted.recurrence_state_ref.clone(),
            instance_key: admitted.instance_key.clone(),
            timezone: admitted.timezone.clone(),
            engine: admitted.engine.clone(),
            original_start_local: admitted.original_start_local.clone(),
            requested_start_local: admitted.requested_start_local.clone(),
            effective_start_local: admitted.effective_start_local.clone(),
            window_start_utc_micros: admitted.window_start_utc_micros,
            window_end_utc_micros: admitted.window_end_utc_micros,
            resolution: admitted.resolution.clone(),
            disposition: admitted.disposition.clone(),
        };
        value.validate()?;
        Ok(value)
    }

    pub fn validate(&self) -> Result<(), TimeEvidenceError> {
        if self.schema_version != OCCURRENCE_TIME_EVIDENCE_SCHEMA_VERSION {
            return Err(TimeEvidenceError::UnsupportedSchema(self.schema_version));
        }
        validate_ref("occurrence_ref", &self.occurrence_ref)?;
        validate_ref("schedule_ref", &self.schedule_ref)?;
        validate_ref("recurrence_state_ref", &self.recurrence_state_ref)?;
        validate_ref("instance_key", &self.instance_key)?;
        self.timezone
            .validate()
            .map_err(|error| TimeEvidenceError::InvalidTimeZone(error.to_string()))?;
        self.engine
            .validate()
            .map_err(|error| TimeEvidenceError::InvalidEngine(error.to_string()))?;
        self.original_start_local
            .validate()
            .map_err(|error| TimeEvidenceError::InvalidLocalTime(error.to_string()))?;
        self.requested_start_local
            .validate()
            .map_err(|error| TimeEvidenceError::InvalidLocalTime(error.to_string()))?;
        self.effective_start_local
            .validate()
            .map_err(|error| TimeEvidenceError::InvalidLocalTime(error.to_string()))?;
        if self.window_end_utc_micros <= self.window_start_utc_micros {
            return Err(TimeEvidenceError::InvalidWindow);
        }

        match &self.resolution {
            CivilResolution::Exact => {
                if self.effective_start_local != self.requested_start_local {
                    return Err(TimeEvidenceError::ResolutionMismatch);
                }
            }
            CivilResolution::GapShiftForward { shift_seconds } => {
                if *shift_seconds == 0
                    || *shift_seconds > 86_400
                    || self.effective_start_local <= self.requested_start_local
                {
                    return Err(TimeEvidenceError::ResolutionMismatch);
                }
            }
            CivilResolution::FoldEarlier | CivilResolution::FoldLater => {
                if self.effective_start_local != self.requested_start_local {
                    return Err(TimeEvidenceError::ResolutionMismatch);
                }
            }
        }

        match &self.disposition {
            InstanceDisposition::Scheduled
                if self.original_start_local != self.requested_start_local =>
            {
                return Err(TimeEvidenceError::DispositionMismatch);
            }
            InstanceDisposition::MovedOverride
                if self.original_start_local == self.requested_start_local =>
            {
                return Err(TimeEvidenceError::DispositionMismatch);
            }
            _ => {}
        }
        Ok(())
    }
}

impl OccurrenceRecurrenceEvidenceRecord {
    pub fn validate(&self) -> Result<(), TimeEvidenceError> {
        validate_ref("record_ref", &self.record_ref)?;
        self.evidence.validate()
    }
}

/// Canonicalize all visible recurrence/time evidence by occurrence content
/// identity. Identical semantic duplicates collapse; different evidence for the
/// same occurrence fails closed. No last-writer/latest-record rule exists.
pub fn canonicalize_occurrence_recurrence_evidence(
    records: &[OccurrenceRecurrenceEvidenceRecord],
) -> Result<BTreeMap<String, CanonicalOccurrenceRecurrenceEvidence>, TimeEvidenceError> {
    let mut by_record: BTreeMap<String, CareOccurrenceRecurrenceEvidence> = BTreeMap::new();
    for record in records {
        record.validate()?;
        if let Some(existing) = by_record.get(&record.record_ref) {
            if existing != &record.evidence {
                return Err(TimeEvidenceError::RecordIdentityCollision(
                    record.record_ref.clone(),
                ));
            }
        } else {
            by_record.insert(record.record_ref.clone(), record.evidence.clone());
        }
    }

    let mut grouped: BTreeMap<String, Vec<OccurrenceRecurrenceEvidenceRecord>> = BTreeMap::new();
    for (record_ref, evidence) in by_record {
        grouped
            .entry(evidence.occurrence_ref.clone())
            .or_default()
            .push(OccurrenceRecurrenceEvidenceRecord {
                record_ref,
                evidence,
            });
    }

    let mut out = BTreeMap::new();
    for (occurrence_ref, mut group) in grouped {
        group.sort_by(|left, right| left.record_ref.cmp(&right.record_ref));
        let first = group
            .first()
            .expect("group is non-empty after insertion")
            .evidence
            .clone();
        if group.iter().any(|record| record.evidence != first) {
            return Err(TimeEvidenceError::EvidenceConflict(occurrence_ref));
        }
        let canonical = group.remove(0);
        let duplicate_record_refs = group.into_iter().map(|record| record.record_ref).collect();
        out.insert(
            occurrence_ref,
            CanonicalOccurrenceRecurrenceEvidence {
                canonical,
                duplicate_record_refs,
            },
        );
    }
    Ok(out)
}

fn validate_ref(field: &'static str, value: &str) -> Result<(), TimeEvidenceError> {
    if value.trim().is_empty() {
        return Err(TimeEvidenceError::EmptyField(field));
    }
    if value.len() > MAX_REF_LEN {
        return Err(TimeEvidenceError::FieldTooLong {
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
        CivilResolution, ExpansionEngineEvidence, InstanceDisposition, LocalDate, LocalTime,
    };
    use std::collections::BTreeSet;

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

    fn admitted(recurrence_state: &str) -> AdmittedCareOccurrence {
        AdmittedCareOccurrence {
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: recurrence_state.into(),
            assignment_state_ref: "assignment:a1".into(),
            assignee: "agent:alice".into(),
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
            window_start_utc_micros: 1_000_000,
            window_end_utc_micros: 1_000_000 + 30 * 60_000_000,
            resolution: CivilResolution::Exact,
            disposition: InstanceDisposition::Scheduled,
            estimated_minutes: Some(20),
        }
    }

    fn record(
        record_ref: &str,
        occurrence_ref: &str,
        recurrence_state: &str,
    ) -> OccurrenceRecurrenceEvidenceRecord {
        OccurrenceRecurrenceEvidenceRecord {
            record_ref: record_ref.into(),
            evidence: CareOccurrenceRecurrenceEvidence::from_admitted(
                occurrence_ref,
                &admitted(recurrence_state),
            )
            .unwrap(),
        }
    }

    #[test]
    fn admitted_evidence_preserves_recurrence_provenance() {
        let value = CareOccurrenceRecurrenceEvidence::from_admitted(
            "occurrence:1",
            &admitted("r1"),
        )
        .unwrap();
        assert_eq!(value.occurrence_ref, "occurrence:1");
        assert_eq!(value.recurrence_state_ref, "r1");
        assert_eq!(value.engine.tzdb_ref, "iana:2026c");
    }

    #[test]
    fn identical_duplicates_collapse_deterministically() {
        let canonical = canonicalize_occurrence_recurrence_evidence(&[
            record("record:b", "occurrence:1", "r1"),
            record("record:a", "occurrence:1", "r1"),
        ])
        .unwrap();
        let value = canonical.get("occurrence:1").unwrap();
        assert_eq!(value.canonical.record_ref, "record:a");
        assert_eq!(value.duplicate_record_refs, vec!["record:b"]);
    }

    #[test]
    fn stale_recurrence_state_is_a_conflict_even_with_same_window() {
        let error = canonicalize_occurrence_recurrence_evidence(&[
            record("record:a", "occurrence:1", "r1"),
            record("record:b", "occurrence:1", "r2"),
        ])
        .unwrap_err();
        assert_eq!(
            error,
            TimeEvidenceError::EvidenceConflict("occurrence:1".into())
        );
    }

    #[test]
    fn different_tzdb_is_a_conflict() {
        let first = record("record:a", "occurrence:1", "r1");
        let mut second = record("record:b", "occurrence:1", "r1");
        second.evidence.engine.tzdb_ref = "iana:future".into();
        assert!(matches!(
            canonicalize_occurrence_recurrence_evidence(&[first, second]),
            Err(TimeEvidenceError::EvidenceConflict(_))
        ));
    }

    #[test]
    fn different_occurrences_can_coexist() {
        let canonical = canonicalize_occurrence_recurrence_evidence(&[
            record("record:a", "occurrence:1", "r1"),
            record("record:b", "occurrence:2", "r1"),
        ])
        .unwrap();
        let keys: BTreeSet<_> = canonical.keys().cloned().collect();
        assert_eq!(
            keys,
            BTreeSet::from(["occurrence:1".to_string(), "occurrence:2".to_string()])
        );
    }

    #[test]
    fn record_identity_collision_fails_closed() {
        let first = record("record:a", "occurrence:1", "r1");
        let second = record("record:a", "occurrence:1", "r2");
        assert_eq!(
            canonicalize_occurrence_recurrence_evidence(&[first, second]).unwrap_err(),
            TimeEvidenceError::RecordIdentityCollision("record:a".into())
        );
    }
}
