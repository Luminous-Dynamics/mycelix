// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure append-only recurrence revision semantics for Hearth Care.
//!
//! A durable revision stores the recurrence definition but not its own state
//! reference. The DHT content hash becomes that state reference. Readers bind
//! `(schedule root, revision state ref)` back into an A6.1 `CareRecurrenceSpec`.

use std::collections::{BTreeMap, BTreeSet};

use hearth_care_recurrence::{
    CareRecurrenceSpec, CivilTimePolicy, LocalDateTime, MissedOccurrencePolicy,
    OccurrenceOverride, RecurrenceEnd, RecurrencePattern, TimeZoneId,
};
use serde::{Deserialize, Serialize};

pub const RECURRENCE_REVISION_SCHEMA_VERSION: u16 = 1;
const MAX_REF_LEN: usize = 1024;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareRecurrenceDefinition {
    pub recurrence_schema_version: u16,
    pub timezone: TimeZoneId,
    pub starts_local: LocalDateTime,
    pub duration_minutes: u32,
    pub pattern: RecurrencePattern,
    pub end: RecurrenceEnd,
    pub civil_time_policy: CivilTimePolicy,
    pub missed_policy: MissedOccurrencePolicy,
    pub exclusions: Vec<LocalDateTime>,
    pub overrides: Vec<OccurrenceOverride>,
}

impl CareRecurrenceDefinition {
    pub fn from_spec(spec: &CareRecurrenceSpec) -> Self {
        Self {
            recurrence_schema_version: spec.schema_version,
            timezone: spec.timezone.clone(),
            starts_local: spec.starts_local.clone(),
            duration_minutes: spec.duration_minutes,
            pattern: spec.pattern.clone(),
            end: spec.end.clone(),
            civil_time_policy: spec.civil_time_policy,
            missed_policy: spec.missed_policy,
            exclusions: spec.exclusions.clone(),
            overrides: spec.overrides.clone(),
        }
    }

    pub fn to_spec(&self, schedule_ref: String, state_ref: String) -> CareRecurrenceSpec {
        CareRecurrenceSpec {
            schema_version: self.recurrence_schema_version,
            schedule_ref,
            recurrence_state_ref: state_ref,
            timezone: self.timezone.clone(),
            starts_local: self.starts_local.clone(),
            duration_minutes: self.duration_minutes,
            pattern: self.pattern.clone(),
            end: self.end.clone(),
            civil_time_policy: self.civil_time_policy,
            missed_policy: self.missed_policy,
            exclusions: self.exclusions.clone(),
            overrides: self.overrides.clone(),
        }
    }

    pub fn validate(&self) -> Result<(), RecurrenceStateError> {
        self.to_spec("schedule:validation".into(), "state:validation".into())
            .validate()
            .map_err(|error| RecurrenceStateError::InvalidDefinition(error.to_string()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecurrenceRevision {
    pub schema_version: u16,
    /// Stable/root CareSchedule reference.
    pub schedule_ref: String,
    /// Exact content-state reference of the previous revision, if any.
    pub parent_state_ref: Option<String>,
    pub definition: CareRecurrenceDefinition,
}

impl RecurrenceRevision {
    pub fn validate(&self) -> Result<(), RecurrenceStateError> {
        if self.schema_version != RECURRENCE_REVISION_SCHEMA_VERSION {
            return Err(RecurrenceStateError::UnsupportedSchema(self.schema_version));
        }
        validate_ref("schedule_ref", &self.schedule_ref)?;
        if let Some(parent) = &self.parent_state_ref {
            validate_ref("parent_state_ref", parent)?;
        }
        self.definition.validate()
    }

    pub fn bound_spec(&self, state_ref: &str) -> Result<CareRecurrenceSpec, RecurrenceStateError> {
        self.validate()?;
        validate_ref("state_ref", state_ref)?;
        let spec = self
            .definition
            .to_spec(self.schedule_ref.clone(), state_ref.to_string());
        spec.validate()
            .map_err(|error| RecurrenceStateError::InvalidDefinition(error.to_string()))?;
        Ok(spec)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecurrenceRevisionRecord {
    /// Stable content identity supplied by the storage layer, intended to be the EntryHash.
    pub state_ref: String,
    pub revision: RecurrenceRevision,
}

impl RecurrenceRevisionRecord {
    pub fn validate(&self) -> Result<(), RecurrenceStateError> {
        validate_ref("state_ref", &self.state_ref)?;
        self.revision.validate()?;
        if self
            .revision
            .parent_state_ref
            .as_ref()
            .is_some_and(|parent| parent == &self.state_ref)
        {
            return Err(RecurrenceStateError::SelfParent(self.state_ref.clone()));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectiveRecurrenceState {
    pub schedule_ref: String,
    pub root_state_ref: String,
    pub effective_state_ref: String,
    pub effective_spec: CareRecurrenceSpec,
    pub revision_count: u32,
    pub duplicate_record_count: u32,
    pub orphan_record_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecurrenceStateError {
    UnsupportedSchema(u16),
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, len: usize },
    InvalidDefinition(String),
    StateCollision(String),
    ScheduleMismatch,
    SelfParent(String),
    NoRoot,
    MultipleRoots(Vec<String>),
    Fork { parent_state_ref: String, children: Vec<String> },
    Cycle(String),
    CountOverflow,
}

impl std::fmt::Display for RecurrenceStateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for RecurrenceStateError {}

/// Derive one authoritative recurrence head from append-only revision evidence.
///
/// Identical repeated records with the same `state_ref` collapse. Competing
/// children of the same parent fail closed. Disconnected orphan chains remain
/// visible through the returned counter but cannot become authority.
pub fn derive_effective_recurrence_state(
    records: &[RecurrenceRevisionRecord],
) -> Result<EffectiveRecurrenceState, RecurrenceStateError> {
    let mut canonical: BTreeMap<String, RecurrenceRevision> = BTreeMap::new();
    let mut duplicate_record_count: u32 = 0;

    for record in records {
        record.validate()?;
        match canonical.get(&record.state_ref) {
            Some(existing) if existing == &record.revision => {
                duplicate_record_count = duplicate_record_count
                    .checked_add(1)
                    .ok_or(RecurrenceStateError::CountOverflow)?;
            }
            Some(_) => {
                return Err(RecurrenceStateError::StateCollision(
                    record.state_ref.clone(),
                ))
            }
            None => {
                canonical.insert(record.state_ref.clone(), record.revision.clone());
            }
        }
    }

    if canonical.is_empty() {
        return Err(RecurrenceStateError::NoRoot);
    }

    let schedule_ref = canonical
        .values()
        .next()
        .expect("canonical is non-empty")
        .schedule_ref
        .clone();
    if canonical
        .values()
        .any(|revision| revision.schedule_ref != schedule_ref)
    {
        return Err(RecurrenceStateError::ScheduleMismatch);
    }

    let mut roots = canonical
        .iter()
        .filter_map(|(state_ref, revision)| {
            revision
                .parent_state_ref
                .is_none()
                .then_some(state_ref.clone())
        })
        .collect::<Vec<_>>();
    roots.sort();
    if roots.is_empty() {
        return Err(RecurrenceStateError::NoRoot);
    }
    if roots.len() != 1 {
        return Err(RecurrenceStateError::MultipleRoots(roots));
    }
    let root_state_ref = roots.remove(0);

    let mut children: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (state_ref, revision) in &canonical {
        let Some(parent) = &revision.parent_state_ref else {
            continue;
        };
        if canonical.contains_key(parent) {
            children
                .entry(parent.clone())
                .or_default()
                .push(state_ref.clone());
        }
    }
    for values in children.values_mut() {
        values.sort();
        values.dedup();
    }

    let mut reachable = BTreeSet::new();
    let mut current = root_state_ref.clone();
    loop {
        if !reachable.insert(current.clone()) {
            return Err(RecurrenceStateError::Cycle(current));
        }
        let Some(next) = children.get(&current) else {
            break;
        };
        if next.len() > 1 {
            return Err(RecurrenceStateError::Fork {
                parent_state_ref: current,
                children: next.clone(),
            });
        }
        current = next[0].clone();
    }

    let orphan_record_count = canonical
        .len()
        .checked_sub(reachable.len())
        .ok_or(RecurrenceStateError::CountOverflow)?;
    let orphan_record_count = u32::try_from(orphan_record_count)
        .map_err(|_| RecurrenceStateError::CountOverflow)?;
    let revision_count = u32::try_from(reachable.len())
        .map_err(|_| RecurrenceStateError::CountOverflow)?;

    let effective_revision = canonical
        .get(&current)
        .ok_or_else(|| RecurrenceStateError::Cycle(current.clone()))?;
    let effective_spec = effective_revision.bound_spec(&current)?;

    Ok(EffectiveRecurrenceState {
        schedule_ref,
        root_state_ref,
        effective_state_ref: current,
        effective_spec,
        revision_count,
        duplicate_record_count,
        orphan_record_count,
    })
}

fn validate_ref(field: &'static str, value: &str) -> Result<(), RecurrenceStateError> {
    if value.trim().is_empty() {
        return Err(RecurrenceStateError::EmptyField(field));
    }
    if value.len() > MAX_REF_LEN {
        return Err(RecurrenceStateError::FieldTooLong {
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
        CivilTimePolicy, LocalDate, LocalTime, RecurrenceEnd, RecurrencePattern,
        RECURRENCE_SCHEMA_VERSION,
    };

    fn definition(hour: u8) -> CareRecurrenceDefinition {
        CareRecurrenceDefinition {
            recurrence_schema_version: RECURRENCE_SCHEMA_VERSION,
            timezone: TimeZoneId("Africa/Johannesburg".into()),
            starts_local: LocalDateTime {
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
            },
            duration_minutes: 30,
            pattern: RecurrencePattern::Daily { interval: 1 },
            end: RecurrenceEnd::Never,
            civil_time_policy: CivilTimePolicy::default(),
            missed_policy: MissedOccurrencePolicy::SurfaceForReview,
            exclusions: Vec::new(),
            overrides: Vec::new(),
        }
    }

    fn revision(parent: Option<&str>, hour: u8) -> RecurrenceRevision {
        RecurrenceRevision {
            schema_version: RECURRENCE_REVISION_SCHEMA_VERSION,
            schedule_ref: "schedule:1".into(),
            parent_state_ref: parent.map(str::to_string),
            definition: definition(hour),
        }
    }

    fn record(state_ref: &str, parent: Option<&str>, hour: u8) -> RecurrenceRevisionRecord {
        RecurrenceRevisionRecord {
            state_ref: state_ref.into(),
            revision: revision(parent, hour),
        }
    }

    #[test]
    fn definition_binds_storage_identity_into_a61_spec() {
        let value = revision(None, 18);
        let spec = value.bound_spec("entry:abc").unwrap();
        assert_eq!(spec.schedule_ref, "schedule:1");
        assert_eq!(spec.recurrence_state_ref, "entry:abc");
        assert_eq!(spec.starts_local.time.hour, 18);
    }

    #[test]
    fn one_root_is_effective_state() {
        let result = derive_effective_recurrence_state(&[record("entry:a", None, 18)]).unwrap();
        assert_eq!(result.root_state_ref, "entry:a");
        assert_eq!(result.effective_state_ref, "entry:a");
        assert_eq!(result.revision_count, 1);
    }

    #[test]
    fn linear_chain_selects_exact_head() {
        let result = derive_effective_recurrence_state(&[
            record("entry:a", None, 18),
            record("entry:b", Some("entry:a"), 19),
            record("entry:c", Some("entry:b"), 20),
        ])
        .unwrap();
        assert_eq!(result.effective_state_ref, "entry:c");
        assert_eq!(result.effective_spec.starts_local.time.hour, 20);
        assert_eq!(result.revision_count, 3);
    }

    #[test]
    fn identical_duplicate_state_does_not_inflate_chain() {
        let a = record("entry:a", None, 18);
        let result = derive_effective_recurrence_state(&[a.clone(), a]).unwrap();
        assert_eq!(result.revision_count, 1);
        assert_eq!(result.duplicate_record_count, 1);
    }

    #[test]
    fn same_state_ref_with_different_content_is_conflict() {
        let error = derive_effective_recurrence_state(&[
            record("entry:a", None, 18),
            record("entry:a", None, 19),
        ])
        .unwrap_err();
        assert_eq!(error, RecurrenceStateError::StateCollision("entry:a".into()));
    }

    #[test]
    fn concurrent_children_fail_closed() {
        let error = derive_effective_recurrence_state(&[
            record("entry:a", None, 18),
            record("entry:b", Some("entry:a"), 19),
            record("entry:c", Some("entry:a"), 20),
        ])
        .unwrap_err();
        assert_eq!(
            error,
            RecurrenceStateError::Fork {
                parent_state_ref: "entry:a".into(),
                children: vec!["entry:b".into(), "entry:c".into()],
            }
        );
    }

    #[test]
    fn disconnected_evidence_cannot_become_authority() {
        let result = derive_effective_recurrence_state(&[
            record("entry:a", None, 18),
            record("entry:orphan", Some("entry:missing"), 21),
        ])
        .unwrap();
        assert_eq!(result.effective_state_ref, "entry:a");
        assert_eq!(result.orphan_record_count, 1);
    }

    #[test]
    fn multiple_roots_are_ambiguous() {
        let error = derive_effective_recurrence_state(&[
            record("entry:a", None, 18),
            record("entry:b", None, 19),
        ])
        .unwrap_err();
        assert_eq!(
            error,
            RecurrenceStateError::MultipleRoots(vec!["entry:a".into(), "entry:b".into()])
        );
    }

    #[test]
    fn self_parent_is_rejected_before_chain_derivation() {
        let error = derive_effective_recurrence_state(&[
            record("entry:a", Some("entry:a"), 18),
        ])
        .unwrap_err();
        assert_eq!(error, RecurrenceStateError::SelfParent("entry:a".into()));
    }
}
