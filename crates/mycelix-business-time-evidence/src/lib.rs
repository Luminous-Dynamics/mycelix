// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Preregistered, transition-aware local-time evidence for Mycelix Business.
//!
//! This crate does not embed a timezone database and does not decide which civil-time rules are
//! legally or factually correct. It binds an externally supplied rule source and an exact UTC
//! offset schedule before evaluation so downstream qualification can reconstruct local time without
//! silently changing DST/history semantics after seeing results.

use mycelix_business_core::{Digest32, ReferenceId};
use sha2::{Digest, Sha256};

pub const TIME_EVIDENCE_IS_NON_AUTHORITATIVE: bool = true;
pub const MILLIS_PER_MINUTE: i128 = 60_000;
pub const MILLIS_PER_DAY: i128 = 86_400_000;
pub const MAX_UTC_OFFSET_MINUTES: i16 = 14 * 60;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UtcOffsetPeriod {
    /// Inclusive UTC start.
    pub start_unix_ms: u64,
    /// Exclusive UTC end.
    pub end_unix_ms: u64,
    pub utc_offset_minutes: i16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LocalTimeSchedule {
    pub timezone: ReferenceId,
    /// Opaque identity of the ruleset source, e.g. a tzdata build or jurisdictional schedule.
    pub rule_source: ReferenceId,
    pub rule_source_digest: Digest32,
    pub registered_at_unix_ms: u64,
    pub evaluation_start_unix_ms: u64,
    pub evaluation_end_unix_ms: u64,
    /// Must form exact contiguous coverage of the evaluation window.
    pub periods: Vec<UtcOffsetPeriod>,
    pub schedule_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScheduleError {
    ZeroRuleSourceDigest,
    ZeroScheduleDigest,
    InvalidEvaluationWindow,
    NotPreregistered,
    NoPeriods,
    InvalidPeriod { index: usize },
    OffsetOutOfRange { index: usize },
    FirstPeriodDoesNotStartAtEvaluation,
    LastPeriodDoesNotEndAtEvaluation,
    GapOrOverlap { previous: usize, next: usize },
    DigestMismatch,
}

impl LocalTimeSchedule {
    pub fn build(
        timezone: ReferenceId,
        rule_source: ReferenceId,
        rule_source_digest: Digest32,
        registered_at_unix_ms: u64,
        evaluation_start_unix_ms: u64,
        evaluation_end_unix_ms: u64,
        periods: Vec<UtcOffsetPeriod>,
    ) -> Result<Self, ScheduleError> {
        let mut schedule = Self {
            timezone,
            rule_source,
            rule_source_digest,
            registered_at_unix_ms,
            evaluation_start_unix_ms,
            evaluation_end_unix_ms,
            periods,
            schedule_digest: Digest32([0; 32]),
        };
        schedule.schedule_digest = compute_schedule_digest(&schedule);
        schedule.validate()?;
        Ok(schedule)
    }

    pub fn validate(&self) -> Result<(), ScheduleError> {
        if zero_digest(&self.rule_source_digest) {
            return Err(ScheduleError::ZeroRuleSourceDigest);
        }
        if zero_digest(&self.schedule_digest) {
            return Err(ScheduleError::ZeroScheduleDigest);
        }
        if self.evaluation_start_unix_ms == 0
            || self.evaluation_start_unix_ms >= self.evaluation_end_unix_ms
        {
            return Err(ScheduleError::InvalidEvaluationWindow);
        }
        if self.registered_at_unix_ms == 0
            || self.registered_at_unix_ms >= self.evaluation_start_unix_ms
        {
            return Err(ScheduleError::NotPreregistered);
        }
        if self.periods.is_empty() {
            return Err(ScheduleError::NoPeriods);
        }
        for (index, period) in self.periods.iter().enumerate() {
            if period.start_unix_ms >= period.end_unix_ms
                || period.start_unix_ms < self.evaluation_start_unix_ms
                || period.end_unix_ms > self.evaluation_end_unix_ms
            {
                return Err(ScheduleError::InvalidPeriod { index });
            }
            if period.utc_offset_minutes.abs() > MAX_UTC_OFFSET_MINUTES {
                return Err(ScheduleError::OffsetOutOfRange { index });
            }
        }
        if self.periods[0].start_unix_ms != self.evaluation_start_unix_ms {
            return Err(ScheduleError::FirstPeriodDoesNotStartAtEvaluation);
        }
        if self.periods[self.periods.len() - 1].end_unix_ms != self.evaluation_end_unix_ms {
            return Err(ScheduleError::LastPeriodDoesNotEndAtEvaluation);
        }
        for (index, pair) in self.periods.windows(2).enumerate() {
            if pair[0].end_unix_ms != pair[1].start_unix_ms {
                return Err(ScheduleError::GapOrOverlap {
                    previous: index,
                    next: index + 1,
                });
            }
        }
        if self.schedule_digest != compute_schedule_digest(self) {
            return Err(ScheduleError::DigestMismatch);
        }
        Ok(())
    }

    pub fn resolve(&self, unix_ms: u64) -> Result<LocalTimePoint, ResolveError> {
        self.validate().map_err(ResolveError::InvalidSchedule)?;
        if unix_ms < self.evaluation_start_unix_ms || unix_ms >= self.evaluation_end_unix_ms {
            return Err(ResolveError::OutsideEvaluationWindow);
        }
        let period = self
            .periods
            .iter()
            .find(|period| period.start_unix_ms <= unix_ms && unix_ms < period.end_unix_ms)
            .ok_or(ResolveError::NoCoveringPeriod)?;
        local_time_point(unix_ms, period.utc_offset_minutes)
    }

    /// Resolve a UTC interval and state whether any offset transition occurs inside it.
    pub fn resolve_window(
        &self,
        start_unix_ms: u64,
        end_unix_ms: u64,
    ) -> Result<ResolvedLocalWindow, ResolveError> {
        self.validate().map_err(ResolveError::InvalidSchedule)?;
        if start_unix_ms >= end_unix_ms
            || start_unix_ms < self.evaluation_start_unix_ms
            || end_unix_ms > self.evaluation_end_unix_ms
        {
            return Err(ResolveError::InvalidWindow);
        }
        let end_inclusive = end_unix_ms
            .checked_sub(1)
            .ok_or(ResolveError::ArithmeticOverflow)?;
        let start = self.resolve(start_unix_ms)?;
        let end = self.resolve(end_inclusive)?;
        let transition_count = self
            .periods
            .iter()
            .skip(1)
            .filter(|period| {
                start_unix_ms < period.start_unix_ms && period.start_unix_ms < end_unix_ms
            })
            .count();
        Ok(ResolvedLocalWindow {
            start,
            end_inclusive: end,
            transition_count: u32::try_from(transition_count)
                .map_err(|_| ResolveError::ArithmeticOverflow)?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LocalTimePoint {
    /// Days since 1970-01-01 in the resolved local clock. May be negative near the Unix epoch.
    pub local_day_index: i64,
    pub minute_of_day: u16,
    /// Monday=0 ... Sunday=6.
    pub weekday: u8,
    pub utc_offset_minutes: i16,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedLocalWindow {
    pub start: LocalTimePoint,
    pub end_inclusive: LocalTimePoint,
    pub transition_count: u32,
}

impl ResolvedLocalWindow {
    pub fn crosses_offset_transition(&self) -> bool {
        self.transition_count != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ResolveError {
    InvalidSchedule(ScheduleError),
    OutsideEvaluationWindow,
    InvalidWindow,
    NoCoveringPeriod,
    ArithmeticOverflow,
}

fn local_time_point(unix_ms: u64, offset_minutes: i16) -> Result<LocalTimePoint, ResolveError> {
    let shifted = i128::from(unix_ms)
        .checked_add(i128::from(offset_minutes) * MILLIS_PER_MINUTE)
        .ok_or(ResolveError::ArithmeticOverflow)?;
    let local_day = shifted.div_euclid(MILLIS_PER_DAY);
    let within_day = shifted.rem_euclid(MILLIS_PER_DAY);
    let local_day_index =
        i64::try_from(local_day).map_err(|_| ResolveError::ArithmeticOverflow)?;
    let minute_of_day = u16::try_from(within_day / MILLIS_PER_MINUTE)
        .map_err(|_| ResolveError::ArithmeticOverflow)?;
    let weekday = u8::try_from((local_day + 3).rem_euclid(7))
        .map_err(|_| ResolveError::ArithmeticOverflow)?;
    Ok(LocalTimePoint {
        local_day_index,
        minute_of_day,
        weekday,
        utc_offset_minutes: offset_minutes,
    })
}

fn compute_schedule_digest(schedule: &LocalTimeSchedule) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:local-time-schedule:v1");
    hash_str(&mut hasher, schedule.timezone.as_str());
    hash_str(&mut hasher, schedule.rule_source.as_str());
    hasher.update(schedule.rule_source_digest.0);
    hasher.update(schedule.registered_at_unix_ms.to_be_bytes());
    hasher.update(schedule.evaluation_start_unix_ms.to_be_bytes());
    hasher.update(schedule.evaluation_end_unix_ms.to_be_bytes());
    hasher.update((schedule.periods.len() as u64).to_be_bytes());
    for period in &schedule.periods {
        hasher.update(period.start_unix_ms.to_be_bytes());
        hasher.update(period.end_unix_ms.to_be_bytes());
        hasher.update(period.utc_offset_minutes.to_be_bytes());
    }
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn build(periods: Vec<UtcOffsetPeriod>) -> Result<LocalTimeSchedule, ScheduleError> {
        LocalTimeSchedule::build(
            id("timezone:test"),
            id("time-rules:test:v1"),
            Digest32::repeat(1),
            500,
            1_000,
            5_000,
            periods,
        )
    }

    #[test]
    fn fixed_offset_schedule_resolves_local_clock() {
        let schedule = build(vec![UtcOffsetPeriod {
            start_unix_ms: 1_000,
            end_unix_ms: 5_000,
            utc_offset_minutes: 120,
        }])
        .unwrap();
        let point = schedule.resolve(2_000).unwrap();
        assert_eq!(point.utc_offset_minutes, 120);
        assert!(TIME_EVIDENCE_IS_NON_AUTHORITATIVE);
    }

    #[test]
    fn transition_is_explicit_and_detectable() {
        let schedule = build(vec![
            UtcOffsetPeriod {
                start_unix_ms: 1_000,
                end_unix_ms: 3_000,
                utc_offset_minutes: 60,
            },
            UtcOffsetPeriod {
                start_unix_ms: 3_000,
                end_unix_ms: 5_000,
                utc_offset_minutes: 120,
            },
        ])
        .unwrap();
        assert_eq!(schedule.resolve(2_999).unwrap().utc_offset_minutes, 60);
        assert_eq!(schedule.resolve(3_000).unwrap().utc_offset_minutes, 120);
        let window = schedule.resolve_window(2_000, 4_000).unwrap();
        assert!(window.crosses_offset_transition());
        assert_eq!(window.transition_count, 1);
    }

    #[test]
    fn gap_or_overlap_fails_closed() {
        let gap = build(vec![
            UtcOffsetPeriod {
                start_unix_ms: 1_000,
                end_unix_ms: 2_500,
                utc_offset_minutes: 60,
            },
            UtcOffsetPeriod {
                start_unix_ms: 3_000,
                end_unix_ms: 5_000,
                utc_offset_minutes: 60,
            },
        ]);
        assert!(matches!(gap, Err(ScheduleError::GapOrOverlap { .. })));

        let overlap = build(vec![
            UtcOffsetPeriod {
                start_unix_ms: 1_000,
                end_unix_ms: 3_500,
                utc_offset_minutes: 60,
            },
            UtcOffsetPeriod {
                start_unix_ms: 3_000,
                end_unix_ms: 5_000,
                utc_offset_minutes: 120,
            },
        ]);
        assert!(matches!(overlap, Err(ScheduleError::GapOrOverlap { .. })));
    }

    #[test]
    fn schedule_rules_are_preregistered_and_digest_bound() {
        let schedule = build(vec![UtcOffsetPeriod {
            start_unix_ms: 1_000,
            end_unix_ms: 5_000,
            utc_offset_minutes: 120,
        }])
        .unwrap();
        let mut changed = schedule.clone();
        changed.periods[0].utc_offset_minutes = 60;
        assert_eq!(changed.validate(), Err(ScheduleError::DigestMismatch));

        let late = LocalTimeSchedule::build(
            id("timezone:test"),
            id("time-rules:test:v1"),
            Digest32::repeat(1),
            1_000,
            1_000,
            5_000,
            vec![UtcOffsetPeriod {
                start_unix_ms: 1_000,
                end_unix_ms: 5_000,
                utc_offset_minutes: 120,
            }],
        );
        assert_eq!(late, Err(ScheduleError::NotPreregistered));
    }

    #[test]
    fn resolution_outside_registered_window_fails() {
        let schedule = build(vec![UtcOffsetPeriod {
            start_unix_ms: 1_000,
            end_unix_ms: 5_000,
            utc_offset_minutes: 120,
        }])
        .unwrap();
        assert_eq!(
            schedule.resolve(5_000),
            Err(ResolveError::OutsideEvaluationWindow)
        );
    }
}
