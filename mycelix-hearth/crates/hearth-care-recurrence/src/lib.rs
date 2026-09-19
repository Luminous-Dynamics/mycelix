// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic recurrence intent and expansion-evidence contract for Hearth Care.
//!
//! This crate does not perform timezone lookup or recurrence expansion. It
//! defines canonical inputs and receipts used by a pinned expansion engine, so
//! DHT semantics never depend on an unversioned host timezone database.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const RECURRENCE_SCHEMA_VERSION: u16 = 1;
const MAX_REF_LEN: usize = 1024;
const MAX_TZ_LEN: usize = 128;
const MAX_ENGINE_ID_LEN: usize = 128;
const MAX_INSTANCES: u32 = 10_000;
const MAX_DURATION_MINUTES: u32 = 7 * 24 * 60;
const MICROS_PER_MINUTE: i64 = 60_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Weekday {
    Monday,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
    Sunday,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct LocalDate {
    pub year: i32,
    pub month: u8,
    pub day: u8,
}

impl LocalDate {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        if !(1900..=9999).contains(&self.year) || !(1..=12).contains(&self.month) {
            return Err(RecurrenceError::InvalidDate);
        }
        if self.day == 0 || self.day > days_in_month(self.year, self.month) {
            return Err(RecurrenceError::InvalidDate);
        }
        Ok(())
    }

    pub fn weekday(&self) -> Result<Weekday, RecurrenceError> {
        self.validate()?;
        let offsets = [0_i32, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4];
        let mut year = self.year;
        if self.month < 3 {
            year -= 1;
        }
        let sunday_zero = (year
            + year / 4
            - year / 100
            + year / 400
            + offsets[(self.month - 1) as usize]
            + i32::from(self.day))
            .rem_euclid(7);
        Ok(match sunday_zero {
            0 => Weekday::Sunday,
            1 => Weekday::Monday,
            2 => Weekday::Tuesday,
            3 => Weekday::Wednesday,
            4 => Weekday::Thursday,
            5 => Weekday::Friday,
            _ => Weekday::Saturday,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct LocalTime {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

impl LocalTime {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        if self.hour > 23 || self.minute > 59 || self.second > 59 {
            return Err(RecurrenceError::InvalidTime);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct LocalDateTime {
    pub date: LocalDate,
    pub time: LocalTime,
}

impl LocalDateTime {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        self.date.validate()?;
        self.time.validate()?;
        Ok(())
    }

    pub fn canonical(&self) -> String {
        format!(
            "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
            self.date.year,
            self.date.month,
            self.date.day,
            self.time.hour,
            self.time.minute,
            self.time.second
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TimeZoneId(pub String);

impl TimeZoneId {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        validate_ref("timezone", &self.0, MAX_TZ_LEN)?;
        if self.0.contains(char::is_whitespace)
            || self.0.contains('\\')
            || self.0.starts_with('/')
            || self.0.contains("..")
            || !self
                .0
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || matches!(c, '/' | '_' | '-' | '+' | '.'))
        {
            return Err(RecurrenceError::InvalidTimeZoneId);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GapPolicy {
    Reject,
    ShiftForward,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FoldPolicy {
    Reject,
    Earlier,
    Later,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CivilTimePolicy {
    pub gap: GapPolicy,
    pub fold: FoldPolicy,
}

impl Default for CivilTimePolicy {
    fn default() -> Self {
        Self {
            gap: GapPolicy::Reject,
            fold: FoldPolicy::Reject,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MissedOccurrencePolicy {
    SurfaceForReview,
    Skip,
    /// Preserve the original historical window as evidence; never move it to "now".
    MaterializeOriginalWindow,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum MonthlySelector {
    DayOfMonth(u8),
    NthWeekday { ordinal: i8, weekday: Weekday },
}

impl MonthlySelector {
    fn validate(&self) -> Result<(), RecurrenceError> {
        match self {
            Self::DayOfMonth(day) if (1..=31).contains(day) => Ok(()),
            Self::NthWeekday { ordinal, .. }
                if *ordinal != 0 && (-5..=5).contains(ordinal) => Ok(()),
            _ => Err(RecurrenceError::InvalidPattern),
        }
    }

    fn matches(&self, date: &LocalDate) -> Result<bool, RecurrenceError> {
        date.validate()?;
        match self {
            Self::DayOfMonth(day) => Ok(date.day == *day),
            Self::NthWeekday { ordinal, weekday } => {
                if date.weekday()? != *weekday {
                    return Ok(false);
                }
                let positive = ((date.day - 1) / 7 + 1) as i8;
                let negative = -(((days_in_month(date.year, date.month) - date.day) / 7 + 1) as i8);
                Ok(*ordinal == positive || *ordinal == negative)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecurrencePattern {
    Once,
    Daily { interval: u16 },
    Weekly {
        interval: u16,
        /// Sorted and unique for canonical serialization.
        weekdays: Vec<Weekday>,
    },
    Monthly { interval: u16, selector: MonthlySelector },
    Yearly { interval: u16, month: u8, day: u8 },
}

impl RecurrencePattern {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        match self {
            Self::Once => Ok(()),
            Self::Daily { interval } if *interval > 0 => Ok(()),
            Self::Weekly { interval, weekdays } => {
                if *interval == 0 || weekdays.is_empty() {
                    return Err(RecurrenceError::InvalidPattern);
                }
                let mut last: Option<Weekday> = None;
                for weekday in weekdays {
                    if last.is_some_and(|value| value >= *weekday) {
                        return Err(RecurrenceError::NonCanonicalCollection("weekdays"));
                    }
                    last = Some(*weekday);
                }
                Ok(())
            }
            Self::Monthly { interval, selector } if *interval > 0 => selector.validate(),
            Self::Yearly { interval, month, day }
                if *interval > 0 && (1..=12).contains(month) => {
                // Leap-year anchor allows an explicit Feb-29 yearly rule. Years in
                // which the date does not exist are skipped by the expansion theorem.
                LocalDate {
                    year: 2000,
                    month: *month,
                    day: *day,
                }
                .validate()
            }
            _ => Err(RecurrenceError::InvalidPattern),
        }
    }

    fn accepts_start(&self, date: &LocalDate) -> Result<bool, RecurrenceError> {
        Ok(match self {
            Self::Once | Self::Daily { .. } => true,
            Self::Weekly { weekdays, .. } => weekdays.binary_search(&date.weekday()?).is_ok(),
            Self::Monthly { selector, .. } => selector.matches(date)?,
            Self::Yearly { month, day, .. } => date.month == *month && date.day == *day,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum RecurrenceEnd {
    Never,
    UntilLocalDate(LocalDate),
    Count(u32),
}

impl RecurrenceEnd {
    fn validate(&self) -> Result<(), RecurrenceError> {
        match self {
            Self::Never => Ok(()),
            Self::UntilLocalDate(date) => date.validate(),
            Self::Count(value) if *value > 0 && *value <= MAX_INSTANCES => Ok(()),
            Self::Count(_) => Err(RecurrenceError::InvalidEnd),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum OverrideAction {
    Cancel,
    Move {
        new_start_local: LocalDateTime,
        duration_minutes: Option<u32>,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OccurrenceOverride {
    pub original_start_local: LocalDateTime,
    pub action: OverrideAction,
}

impl OccurrenceOverride {
    fn validate(&self) -> Result<(), RecurrenceError> {
        self.original_start_local.validate()?;
        match &self.action {
            OverrideAction::Cancel => Ok(()),
            OverrideAction::Move {
                new_start_local,
                duration_minutes,
            } => {
                new_start_local.validate()?;
                validate_duration(*duration_minutes)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CareRecurrenceSpec {
    pub schema_version: u16,
    /// Stable CareSchedule root reference.
    pub schedule_ref: String,
    /// Exact append-only recurrence revision/state reference.
    pub recurrence_state_ref: String,
    pub timezone: TimeZoneId,
    pub starts_local: LocalDateTime,
    pub duration_minutes: u32,
    pub pattern: RecurrencePattern,
    pub end: RecurrenceEnd,
    pub civil_time_policy: CivilTimePolicy,
    pub missed_policy: MissedOccurrencePolicy,
    /// Exact original local starts removed from the recurrence set. Sorted/unique.
    pub exclusions: Vec<LocalDateTime>,
    /// One-off cancellation/move semantics. Sorted by original_start_local.
    pub overrides: Vec<OccurrenceOverride>,
}

impl CareRecurrenceSpec {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        if self.schema_version != RECURRENCE_SCHEMA_VERSION {
            return Err(RecurrenceError::UnsupportedSchema(self.schema_version));
        }
        validate_ref("schedule_ref", &self.schedule_ref, MAX_REF_LEN)?;
        validate_ref("recurrence_state_ref", &self.recurrence_state_ref, MAX_REF_LEN)?;
        self.timezone.validate()?;
        self.starts_local.validate()?;
        validate_duration(Some(self.duration_minutes))?;
        self.pattern.validate()?;
        if !self.pattern.accepts_start(&self.starts_local.date)? {
            return Err(RecurrenceError::StartNotSynchronized);
        }
        self.end.validate()?;
        if let RecurrenceEnd::UntilLocalDate(until) = &self.end {
            if until < &self.starts_local.date {
                return Err(RecurrenceError::InvalidEnd);
            }
        }

        validate_sorted_unique_datetimes("exclusions", &self.exclusions)?;
        let mut last: Option<&LocalDateTime> = None;
        for value in &self.overrides {
            value.validate()?;
            if last.is_some_and(|previous| previous >= &value.original_start_local) {
                return Err(RecurrenceError::NonCanonicalCollection("overrides"));
            }
            if self.exclusions.binary_search(&value.original_start_local).is_ok() {
                return Err(RecurrenceError::ConflictingException);
            }
            last = Some(&value.original_start_local);
        }

        if matches!(&self.pattern, RecurrencePattern::Once) {
            if self.exclusions.iter().any(|value| value != &self.starts_local)
                || self.overrides.iter().any(|value| value.original_start_local != self.starts_local)
                || self.exclusions.len() > 1
                || self.overrides.len() > 1
            {
                return Err(RecurrenceError::InvalidPattern);
            }
        }
        Ok(())
    }

    pub fn instance_key(
        &self,
        original_start_local: &LocalDateTime,
    ) -> Result<String, RecurrenceError> {
        self.validate()?;
        original_start_local.validate()?;
        Ok(format!(
            "care-recur-v1|{}|{}|{}",
            self.schedule_ref.len(),
            self.schedule_ref,
            original_start_local.canonical()
        ))
    }

    fn expected_duration_minutes(&self, original: &LocalDateTime) -> u32 {
        self.overrides
            .iter()
            .find(|value| &value.original_start_local == original)
            .and_then(|value| match &value.action {
                OverrideAction::Move { duration_minutes, .. } => *duration_minutes,
                OverrideAction::Cancel => None,
            })
            .unwrap_or(self.duration_minutes)
    }

    fn requested_local_start(&self, original: &LocalDateTime) -> Option<&LocalDateTime> {
        if self.exclusions.binary_search(original).is_ok() {
            return None;
        }
        match self
            .overrides
            .iter()
            .find(|value| &value.original_start_local == original)
        {
            Some(OccurrenceOverride { action: OverrideAction::Cancel, .. }) => None,
            Some(OccurrenceOverride {
                action: OverrideAction::Move { new_start_local, .. },
                ..
            }) => Some(new_start_local),
            None => Some(original),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExpansionRequest {
    pub range_start_utc_micros: i64,
    pub range_end_utc_micros: i64,
    pub max_instances: u32,
}

impl ExpansionRequest {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        if self.range_end_utc_micros <= self.range_start_utc_micros {
            return Err(RecurrenceError::InvalidExpansionRange);
        }
        if self.max_instances == 0 || self.max_instances > MAX_INSTANCES {
            return Err(RecurrenceError::InvalidExpansionLimit(self.max_instances));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExpansionEngineEvidence {
    pub engine_id: String,
    pub engine_version: String,
    /// Version/hash/content identifier for the exact timezone rules used.
    pub tzdb_ref: String,
}

impl ExpansionEngineEvidence {
    pub fn validate(&self) -> Result<(), RecurrenceError> {
        validate_ref("engine_id", &self.engine_id, MAX_ENGINE_ID_LEN)?;
        validate_ref("engine_version", &self.engine_version, MAX_ENGINE_ID_LEN)?;
        validate_ref("tzdb_ref", &self.tzdb_ref, MAX_REF_LEN)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum CivilResolution {
    Exact,
    GapShiftForward { shift_seconds: u32 },
    FoldEarlier,
    FoldLater,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum InstanceDisposition {
    Scheduled,
    MovedOverride,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ResolvedOccurrence {
    pub instance_key: String,
    /// Recurrence identity before exclusions/overrides.
    pub original_start_local: LocalDateTime,
    /// Local wall-clock request after one-off override, before DST resolution.
    pub requested_start_local: LocalDateTime,
    /// Actual local wall-clock representation after gap/fold resolution.
    pub effective_start_local: LocalDateTime,
    pub start_utc_micros: i64,
    pub end_utc_micros: i64,
    pub resolution: CivilResolution,
    pub disposition: InstanceDisposition,
}

impl ResolvedOccurrence {
    fn validate_against(
        &self,
        spec: &CareRecurrenceSpec,
        request: &ExpansionRequest,
    ) -> Result<(), RecurrenceError> {
        self.original_start_local.validate()?;
        self.requested_start_local.validate()?;
        self.effective_start_local.validate()?;
        if self.instance_key != spec.instance_key(&self.original_start_local)? {
            return Err(RecurrenceError::InstanceKeyMismatch);
        }
        if self.start_utc_micros < request.range_start_utc_micros
            || self.start_utc_micros >= request.range_end_utc_micros
        {
            return Err(RecurrenceError::OccurrenceOutsideExpansionRange);
        }
        let expected_duration = i64::from(spec.expected_duration_minutes(&self.original_start_local))
            .saturating_mul(MICROS_PER_MINUTE);
        if self.end_utc_micros.checked_sub(self.start_utc_micros) != Some(expected_duration) {
            return Err(RecurrenceError::ResolvedDurationMismatch);
        }

        let Some(requested) = spec.requested_local_start(&self.original_start_local) else {
            return Err(RecurrenceError::ExcludedOccurrenceReturned);
        };
        if requested != &self.requested_start_local {
            return Err(RecurrenceError::OverrideMismatch);
        }
        let expected_disposition = if requested == &self.original_start_local {
            InstanceDisposition::Scheduled
        } else {
            InstanceDisposition::MovedOverride
        };
        if self.disposition != expected_disposition {
            return Err(RecurrenceError::OverrideMismatch);
        }

        match self.resolution {
            CivilResolution::Exact => {
                if self.effective_start_local != self.requested_start_local {
                    return Err(RecurrenceError::ResolutionPolicyMismatch);
                }
            }
            CivilResolution::GapShiftForward { shift_seconds }
                if spec.civil_time_policy.gap == GapPolicy::ShiftForward
                    && shift_seconds > 0
                    && shift_seconds <= 86_400
                    && self.effective_start_local > self.requested_start_local => {}
            CivilResolution::FoldEarlier
                if spec.civil_time_policy.fold == FoldPolicy::Earlier
                    && self.effective_start_local == self.requested_start_local => {}
            CivilResolution::FoldLater
                if spec.civil_time_policy.fold == FoldPolicy::Later
                    && self.effective_start_local == self.requested_start_local => {}
            _ => return Err(RecurrenceError::ResolutionPolicyMismatch),
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExpansionReceipt {
    pub schema_version: u16,
    pub schedule_ref: String,
    pub recurrence_state_ref: String,
    pub timezone: TimeZoneId,
    pub engine: ExpansionEngineEvidence,
    pub request: ExpansionRequest,
    pub occurrences: Vec<ResolvedOccurrence>,
}

impl ExpansionReceipt {
    /// Validate structural/canonical evidence. The actual timezone conversion is
    /// proven by the qualified adapter identified by `engine` + `tzdb_ref`.
    pub fn validate_against(&self, spec: &CareRecurrenceSpec) -> Result<(), RecurrenceError> {
        spec.validate()?;
        if self.schema_version != RECURRENCE_SCHEMA_VERSION {
            return Err(RecurrenceError::UnsupportedSchema(self.schema_version));
        }
        if self.schedule_ref != spec.schedule_ref
            || self.recurrence_state_ref != spec.recurrence_state_ref
            || self.timezone != spec.timezone
        {
            return Err(RecurrenceError::ReceiptBindingMismatch);
        }
        self.engine.validate()?;
        self.request.validate()?;
        if self.occurrences.len() > self.request.max_instances as usize {
            return Err(RecurrenceError::ExpansionLimitExceeded);
        }

        let mut keys = BTreeSet::new();
        let mut last: Option<(&i64, &str)> = None;
        for occurrence in &self.occurrences {
            occurrence.validate_against(spec, &self.request)?;
            if !keys.insert(occurrence.instance_key.clone()) {
                return Err(RecurrenceError::DuplicateResolvedInstance);
            }
            if let Some((previous_start, previous_key)) = last {
                if (*previous_start, previous_key) > (occurrence.start_utc_micros, occurrence.instance_key.as_str()) {
                    return Err(RecurrenceError::NonCanonicalCollection("occurrences"));
                }
            }
            last = Some((&occurrence.start_utc_micros, occurrence.instance_key.as_str()));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecurrenceError {
    UnsupportedSchema(u16),
    EmptyField(&'static str),
    FieldTooLong { field: &'static str, len: usize },
    InvalidDate,
    InvalidTime,
    InvalidTimeZoneId,
    InvalidDuration(u32),
    InvalidPattern,
    StartNotSynchronized,
    InvalidEnd,
    NonCanonicalCollection(&'static str),
    ConflictingException,
    InvalidExpansionRange,
    InvalidExpansionLimit(u32),
    ExpansionLimitExceeded,
    InstanceKeyMismatch,
    OccurrenceOutsideExpansionRange,
    ResolvedDurationMismatch,
    ResolutionPolicyMismatch,
    OverrideMismatch,
    ExcludedOccurrenceReturned,
    DuplicateResolvedInstance,
    ReceiptBindingMismatch,
}

impl std::fmt::Display for RecurrenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for RecurrenceError {}

fn validate_ref(field: &'static str, value: &str, max_len: usize) -> Result<(), RecurrenceError> {
    if value.trim().is_empty() {
        return Err(RecurrenceError::EmptyField(field));
    }
    if value.len() > max_len {
        return Err(RecurrenceError::FieldTooLong { field, len: value.len() });
    }
    Ok(())
}

fn validate_duration(value: Option<u32>) -> Result<(), RecurrenceError> {
    if let Some(minutes) = value {
        if minutes == 0 || minutes > MAX_DURATION_MINUTES {
            return Err(RecurrenceError::InvalidDuration(minutes));
        }
    }
    Ok(())
}

fn validate_sorted_unique_datetimes(
    field: &'static str,
    values: &[LocalDateTime],
) -> Result<(), RecurrenceError> {
    let mut last: Option<&LocalDateTime> = None;
    for value in values {
        value.validate()?;
        if last.is_some_and(|previous| previous >= value) {
            return Err(RecurrenceError::NonCanonicalCollection(field));
        }
        last = Some(value);
    }
    Ok(())
}

fn is_leap_year(year: i32) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}
fn days_in_month(year: i32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn local(year: i32, month: u8, day: u8, hour: u8, minute: u8) -> LocalDateTime {
        LocalDateTime {
            date: LocalDate { year, month, day },
            time: LocalTime { hour, minute, second: 0 },
        }
    }

    fn spec() -> CareRecurrenceSpec {
        CareRecurrenceSpec {
            schema_version: RECURRENCE_SCHEMA_VERSION,
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: "recurrence:1".into(),
            timezone: TimeZoneId("Africa/Johannesburg".into()),
            starts_local: local(2026, 9, 21, 18, 0),
            duration_minutes: 30,
            pattern: RecurrencePattern::Weekly {
                interval: 1,
                weekdays: vec![Weekday::Monday, Weekday::Friday],
            },
            end: RecurrenceEnd::Never,
            civil_time_policy: CivilTimePolicy::default(),
            missed_policy: MissedOccurrencePolicy::SurfaceForReview,
            exclusions: Vec::new(),
            overrides: Vec::new(),
        }
    }

    #[test]
    fn leap_day_validation_is_deterministic() {
        assert!(local(2024, 2, 29, 9, 0).validate().is_ok());
        assert_eq!(local(2025, 2, 29, 9, 0).validate(), Err(RecurrenceError::InvalidDate));
    }

    #[test]
    fn known_weekdays_are_stable() {
        assert_eq!(local(2026, 9, 21, 0, 0).date.weekday().unwrap(), Weekday::Monday);
    }

    #[test]
    fn start_must_match_weekly_rule() {
        let mut value = spec();
        value.starts_local = local(2026, 9, 22, 18, 0);
        assert_eq!(value.validate(), Err(RecurrenceError::StartNotSynchronized));
    }

    #[test]
    fn weekly_days_must_be_sorted_and_unique() {
        let mut value = spec();
        value.pattern = RecurrencePattern::Weekly {
            interval: 1,
            weekdays: vec![Weekday::Friday, Weekday::Monday],
        };
        assert_eq!(value.validate(), Err(RecurrenceError::NonCanonicalCollection("weekdays")));
    }

    #[test]
    fn instance_key_is_stable() {
        let value = spec();
        assert_eq!(
            value.instance_key(&local(2026, 9, 21, 18, 0)).unwrap(),
            "care-recur-v1|10|schedule:1|2026-09-21T18:00:00"
        );
    }

    #[test]
    fn exclusion_and_override_cannot_target_same_instance() {
        let mut value = spec();
        let target = local(2026, 9, 21, 18, 0);
        value.exclusions.push(target.clone());
        value.overrides.push(OccurrenceOverride {
            original_start_local: target,
            action: OverrideAction::Cancel,
        });
        assert_eq!(value.validate(), Err(RecurrenceError::ConflictingException));
    }

    #[test]
    fn once_can_be_cancelled_but_not_exception_another_date() {
        let mut value = spec();
        value.pattern = RecurrencePattern::Once;
        value.exclusions = vec![value.starts_local.clone()];
        assert!(value.validate().is_ok());
        value.exclusions = vec![local(2026, 9, 22, 18, 0)];
        assert_eq!(value.validate(), Err(RecurrenceError::InvalidPattern));
    }

    #[test]
    fn receipt_cannot_hide_dst_fold_policy() {
        let mut value = spec();
        value.timezone = TimeZoneId("America/New_York".into());
        value.starts_local = local(2026, 11, 2, 1, 30); // Monday; keeps weekly synchronization.
        let original = value.starts_local.clone();
        let receipt = ExpansionReceipt {
            schema_version: RECURRENCE_SCHEMA_VERSION,
            schedule_ref: value.schedule_ref.clone(),
            recurrence_state_ref: value.recurrence_state_ref.clone(),
            timezone: value.timezone.clone(),
            engine: ExpansionEngineEvidence {
                engine_id: "example".into(),
                engine_version: "1".into(),
                tzdb_ref: "tzdb:2026c".into(),
            },
            request: ExpansionRequest {
                range_start_utc_micros: 0,
                range_end_utc_micros: i64::MAX,
                max_instances: 10,
            },
            occurrences: vec![ResolvedOccurrence {
                instance_key: value.instance_key(&original).unwrap(),
                original_start_local: original.clone(),
                requested_start_local: original.clone(),
                effective_start_local: original,
                start_utc_micros: 1,
                end_utc_micros: 1 + 30 * MICROS_PER_MINUTE,
                resolution: CivilResolution::FoldEarlier,
                disposition: InstanceDisposition::Scheduled,
            }],
        };
        assert_eq!(receipt.validate_against(&value), Err(RecurrenceError::ResolutionPolicyMismatch));
    }

    #[test]
    fn moved_override_binds_requested_time_and_duration() {
        let mut value = spec();
        value.overrides.push(OccurrenceOverride {
            original_start_local: local(2026, 9, 21, 18, 0),
            action: OverrideAction::Move {
                new_start_local: local(2026, 9, 21, 19, 0),
                duration_minutes: Some(45),
            },
        });
        assert!(value.validate().is_ok());
        let original = local(2026, 9, 21, 18, 0);
        let moved = local(2026, 9, 21, 19, 0);
        let occurrence = ResolvedOccurrence {
            instance_key: value.instance_key(&original).unwrap(),
            original_start_local: original,
            requested_start_local: moved.clone(),
            effective_start_local: moved,
            start_utc_micros: 100,
            end_utc_micros: 100 + 45 * MICROS_PER_MINUTE,
            resolution: CivilResolution::Exact,
            disposition: InstanceDisposition::MovedOverride,
        };
        let request = ExpansionRequest {
            range_start_utc_micros: 0,
            range_end_utc_micros: i64::MAX,
            max_instances: 10,
        };
        assert!(occurrence.validate_against(&value, &request).is_ok());
    }

    #[test]
    fn receipt_duration_cannot_be_fabricated() {
        let value = spec();
        let original = value.starts_local.clone();
        let occurrence = ResolvedOccurrence {
            instance_key: value.instance_key(&original).unwrap(),
            original_start_local: original.clone(),
            requested_start_local: original.clone(),
            effective_start_local: original,
            start_utc_micros: 100,
            end_utc_micros: 101,
            resolution: CivilResolution::Exact,
            disposition: InstanceDisposition::Scheduled,
        };
        assert_eq!(
            occurrence.validate_against(
                &value,
                &ExpansionRequest {
                    range_start_utc_micros: 0,
                    range_end_utc_micros: i64::MAX,
                    max_instances: 10,
                }
            ),
            Err(RecurrenceError::ResolvedDurationMismatch)
        );
    }
}
