// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Authoritative bounded composition for Hearth Care recurrence.
//!
//! This layer composes the typed recurrence contract, deterministic local
//! enumerator, and pinned bundled-tzdb resolver into one validated
//! `ExpansionReceipt`. It performs no assignment, DHT mutation, or completion
//! logic.

use std::collections::BTreeSet;

use hearth_care_recurrence::{
    CareRecurrenceSpec, ExpansionReceipt, ExpansionRequest, InstanceDisposition, LocalDate,
    LocalDateTime, OverrideAction, ResolvedOccurrence, RECURRENCE_SCHEMA_VERSION,
};
use hearth_care_recurrence_enumerator::{
    enumerate_original_starts, LocalExpansionBounds, MAX_LOCAL_OUTPUT,
};
use hearth_care_timezone_jiff::{
    engine_evidence, local_at_utc, resolve_local, TimeZoneEngineError,
};

pub const MAX_AUTHORITATIVE_EXCEPTION_ITEMS: usize = 512;
pub const LOCAL_SCAN_MARGIN_DAYS: i32 = 1;
const MICROS_PER_MINUTE: i64 = 60_000_000;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompositionError {
    InvalidContract(String),
    InvalidRequest(String),
    ExceptionLimitExceeded { count: usize, max: usize },
    TimeZone(String),
    Enumeration(String),
    DateOutOfRange,
    ArithmeticOverflow,
    EngineMismatch,
    ExceptionTargetsNonOccurrence(String),
    CivilTimeRejected(String),
    OutputLimitExceeded { count: usize, max: u32 },
    ReceiptValidation(String),
}

impl std::fmt::Display for CompositionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for CompositionError {}

/// Compose an authoritative recurrence receipt for one bounded UTC interval.
///
/// The caller supplies only the UTC request. Local scan bounds are derived from
/// the same pinned timezone database used for final resolution. Results are
/// never truncated: exceeding `request.max_instances` is an error.
pub fn compose_expansion(
    spec: &CareRecurrenceSpec,
    request: &ExpansionRequest,
) -> Result<ExpansionReceipt, CompositionError> {
    let exception_count = spec
        .exclusions
        .len()
        .checked_add(spec.overrides.len())
        .ok_or(CompositionError::ArithmeticOverflow)?;
    if exception_count > MAX_AUTHORITATIVE_EXCEPTION_ITEMS {
        return Err(CompositionError::ExceptionLimitExceeded {
            count: exception_count,
            max: MAX_AUTHORITATIVE_EXCEPTION_ITEMS,
        });
    }

    spec.validate()
        .map_err(|error| CompositionError::InvalidContract(error.to_string()))?;
    request
        .validate()
        .map_err(|error| CompositionError::InvalidRequest(error.to_string()))?;

    let engine = engine_evidence().map_err(timezone_error)?;
    let end_probe = request
        .range_end_utc_micros
        .checked_sub(1)
        .ok_or(CompositionError::ArithmeticOverflow)?;
    let localized_start = local_at_utc(&spec.timezone, request.range_start_utc_micros)
        .map_err(timezone_error)?;
    let localized_end = local_at_utc(&spec.timezone, end_probe).map_err(timezone_error)?;
    if localized_start.engine != engine || localized_end.engine != engine {
        return Err(CompositionError::EngineMismatch);
    }

    let (core_low, core_high) = if localized_start.local.date <= localized_end.local.date {
        (
            localized_start.local.date.clone(),
            localized_end.local.date.clone(),
        )
    } else {
        (
            localized_end.local.date.clone(),
            localized_start.local.date.clone(),
        )
    };
    let scan_start = add_days(&core_low, -LOCAL_SCAN_MARGIN_DAYS)?;
    let scan_end_exclusive = add_days(&core_high, LOCAL_SCAN_MARGIN_DAYS + 1)?;

    let scan_bounds = LocalExpansionBounds {
        start_inclusive: scan_start.clone(),
        end_exclusive: scan_end_exclusive.clone(),
        max_instances: MAX_LOCAL_OUTPUT,
    };
    let local_series = enumerate_original_starts(spec, &scan_bounds)
        .map_err(|error| CompositionError::Enumeration(error.to_string()))?;
    let mut candidates: BTreeSet<LocalDateTime> =
        local_series.original_starts.into_iter().collect();

    validate_relevant_exceptions(
        spec,
        &scan_start,
        &scan_end_exclusive,
        &mut candidates,
    )?;

    let mut occurrences = Vec::new();
    for original in candidates {
        let Some((requested_local, duration_minutes, disposition)) =
            transform_instance(spec, &original)
        else {
            continue;
        };

        // A move far outside the derived scan cannot resolve into this UTC
        // request under schema-v1's <=24h civil-gap adjustment bound.
        if !date_in_half_open(
            &requested_local.date,
            &scan_start,
            &scan_end_exclusive,
        ) {
            continue;
        }

        let instance_key = spec
            .instance_key(&original)
            .map_err(|error| CompositionError::InvalidContract(error.to_string()))?;
        let resolved = match resolve_local(
            &spec.timezone,
            &requested_local,
            spec.civil_time_policy,
        ) {
            Ok(value) => value,
            Err(TimeZoneEngineError::GapRejected | TimeZoneEngineError::FoldRejected) => {
                if date_in_closed(&requested_local.date, &core_low, &core_high) {
                    return Err(CompositionError::CivilTimeRejected(instance_key));
                }
                continue;
            }
            Err(error) => return Err(timezone_error(error)),
        };
        if resolved.engine != engine {
            return Err(CompositionError::EngineMismatch);
        }
        if resolved.start_utc_micros < request.range_start_utc_micros
            || resolved.start_utc_micros >= request.range_end_utc_micros
        {
            continue;
        }

        let duration_micros = i64::from(duration_minutes)
            .checked_mul(MICROS_PER_MINUTE)
            .ok_or(CompositionError::ArithmeticOverflow)?;
        let end_utc_micros = resolved
            .start_utc_micros
            .checked_add(duration_micros)
            .ok_or(CompositionError::ArithmeticOverflow)?;

        occurrences.push(ResolvedOccurrence {
            instance_key,
            original_start_local: original,
            requested_start_local: requested_local,
            effective_start_local: resolved.effective_local,
            start_utc_micros: resolved.start_utc_micros,
            end_utc_micros,
            resolution: resolved.resolution,
            disposition,
        });
    }

    occurrences.sort_by(|left, right| {
        (left.start_utc_micros, left.instance_key.as_str())
            .cmp(&(right.start_utc_micros, right.instance_key.as_str()))
    });
    if occurrences.len() > request.max_instances as usize {
        return Err(CompositionError::OutputLimitExceeded {
            count: occurrences.len(),
            max: request.max_instances,
        });
    }

    let receipt = ExpansionReceipt {
        schema_version: RECURRENCE_SCHEMA_VERSION,
        schedule_ref: spec.schedule_ref.clone(),
        recurrence_state_ref: spec.recurrence_state_ref.clone(),
        timezone: spec.timezone.clone(),
        engine,
        request: request.clone(),
        occurrences,
    };
    receipt
        .validate_against(spec)
        .map_err(|error| CompositionError::ReceiptValidation(error.to_string()))?;
    Ok(receipt)
}

fn validate_relevant_exceptions(
    spec: &CareRecurrenceSpec,
    scan_start: &LocalDate,
    scan_end_exclusive: &LocalDate,
    candidates: &mut BTreeSet<LocalDateTime>,
) -> Result<(), CompositionError> {
    for excluded in &spec.exclusions {
        if date_in_half_open(&excluded.date, scan_start, scan_end_exclusive)
            && !is_original_occurrence(spec, excluded)?
        {
            return Err(CompositionError::ExceptionTargetsNonOccurrence(
                excluded.canonical(),
            ));
        }
    }

    for value in &spec.overrides {
        let moved_into_scan = match &value.action {
            OverrideAction::Move {
                new_start_local, ..
            } => date_in_half_open(&new_start_local.date, scan_start, scan_end_exclusive),
            OverrideAction::Cancel => false,
        };
        let original_in_scan = date_in_half_open(
            &value.original_start_local.date,
            scan_start,
            scan_end_exclusive,
        );
        if original_in_scan || moved_into_scan {
            if !is_original_occurrence(spec, &value.original_start_local)? {
                return Err(CompositionError::ExceptionTargetsNonOccurrence(
                    value.original_start_local.canonical(),
                ));
            }
            if moved_into_scan {
                candidates.insert(value.original_start_local.clone());
            }
        }
    }
    Ok(())
}

fn is_original_occurrence(
    spec: &CareRecurrenceSpec,
    candidate: &LocalDateTime,
) -> Result<bool, CompositionError> {
    let next_date = add_days(&candidate.date, 1)?;
    let bounds = LocalExpansionBounds {
        start_inclusive: candidate.date.clone(),
        end_exclusive: next_date,
        max_instances: 1,
    };
    let result = enumerate_original_starts(spec, &bounds)
        .map_err(|error| CompositionError::Enumeration(error.to_string()))?;
    Ok(result.original_starts.binary_search(candidate).is_ok())
}

fn transform_instance(
    spec: &CareRecurrenceSpec,
    original: &LocalDateTime,
) -> Option<(LocalDateTime, u32, InstanceDisposition)> {
    if spec.exclusions.binary_search(original).is_ok() {
        return None;
    }
    match spec
        .overrides
        .binary_search_by(|value| value.original_start_local.cmp(original))
    {
        Ok(index) => match &spec.overrides[index].action {
            OverrideAction::Cancel => None,
            OverrideAction::Move {
                new_start_local,
                duration_minutes,
            } => Some((
                new_start_local.clone(),
                duration_minutes.unwrap_or(spec.duration_minutes),
                InstanceDisposition::MovedOverride,
            )),
        },
        Err(_) => Some((
            original.clone(),
            spec.duration_minutes,
            InstanceDisposition::Scheduled,
        )),
    }
}

fn timezone_error(error: TimeZoneEngineError) -> CompositionError {
    CompositionError::TimeZone(error.to_string())
}

fn date_in_half_open(date: &LocalDate, start: &LocalDate, end: &LocalDate) -> bool {
    date >= start && date < end
}

fn date_in_closed(date: &LocalDate, start: &LocalDate, end: &LocalDate) -> bool {
    date >= start && date <= end
}

fn add_days(date: &LocalDate, delta: i32) -> Result<LocalDate, CompositionError> {
    date.validate()
        .map_err(|error| CompositionError::InvalidContract(error.to_string()))?;
    let mut value = date.clone();
    if delta >= 0 {
        for _ in 0..delta {
            value = next_date(&value)?;
        }
    } else {
        for _ in 0..delta.unsigned_abs() {
            value = previous_date(&value)?;
        }
    }
    Ok(value)
}

fn next_date(date: &LocalDate) -> Result<LocalDate, CompositionError> {
    let max = days_in_month(date.year, date.month);
    if date.day < max {
        return Ok(LocalDate {
            year: date.year,
            month: date.month,
            day: date.day + 1,
        });
    }
    if date.month < 12 {
        return Ok(LocalDate {
            year: date.year,
            month: date.month + 1,
            day: 1,
        });
    }
    if date.year >= 9999 {
        return Err(CompositionError::DateOutOfRange);
    }
    Ok(LocalDate {
        year: date.year + 1,
        month: 1,
        day: 1,
    })
}

fn previous_date(date: &LocalDate) -> Result<LocalDate, CompositionError> {
    if date.day > 1 {
        return Ok(LocalDate {
            year: date.year,
            month: date.month,
            day: date.day - 1,
        });
    }
    if date.month > 1 {
        let month = date.month - 1;
        return Ok(LocalDate {
            year: date.year,
            month,
            day: days_in_month(date.year, month),
        });
    }
    if date.year <= 1900 {
        return Err(CompositionError::DateOutOfRange);
    }
    Ok(LocalDate {
        year: date.year - 1,
        month: 12,
        day: 31,
    })
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
    use hearth_care_recurrence::{
        CivilTimePolicy, FoldPolicy, GapPolicy, LocalTime, MissedOccurrencePolicy,
        OccurrenceOverride, RecurrenceEnd, RecurrencePattern, TimeZoneId,
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

    fn spec(
        timezone: &str,
        starts_local: LocalDateTime,
        pattern: RecurrencePattern,
        end: RecurrenceEnd,
    ) -> CareRecurrenceSpec {
        CareRecurrenceSpec {
            schema_version: RECURRENCE_SCHEMA_VERSION,
            schedule_ref: "schedule:1".into(),
            recurrence_state_ref: "recurrence:1".into(),
            timezone: TimeZoneId(timezone.into()),
            starts_local,
            duration_minutes: 30,
            pattern,
            end,
            civil_time_policy: CivilTimePolicy::default(),
            missed_policy: MissedOccurrencePolicy::SurfaceForReview,
            exclusions: Vec::new(),
            overrides: Vec::new(),
        }
    }

    fn exact_utc(timezone: &str, value: &LocalDateTime) -> i64 {
        resolve_local(
            &TimeZoneId(timezone.into()),
            value,
            CivilTimePolicy::default(),
        )
        .unwrap()
        .start_utc_micros
    }

    fn request_for_local_day_range(
        timezone: &str,
        start: LocalDateTime,
        end: LocalDateTime,
        max_instances: u32,
    ) -> ExpansionRequest {
        ExpansionRequest {
            range_start_utc_micros: exact_utc(timezone, &start),
            range_end_utc_micros: exact_utc(timezone, &end),
            max_instances,
        }
    }

    #[test]
    fn daily_series_composes_exact_receipt() {
        let value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 21, 18, 0),
            RecurrencePattern::Daily { interval: 1 },
            RecurrenceEnd::Count(3),
        );
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 24, 0, 0),
            10,
        );
        let receipt = compose_expansion(&value, &request).unwrap();
        assert_eq!(receipt.occurrences.len(), 3);
        assert!(receipt.validate_against(&value).is_ok());
    }

    #[test]
    fn exclusion_does_not_manufacture_replacement() {
        let mut value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 21, 18, 0),
            RecurrencePattern::Daily { interval: 1 },
            RecurrenceEnd::Count(3),
        );
        value.exclusions.push(local(2026, 9, 21, 18, 0));
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 25, 0, 0),
            10,
        );
        let receipt = compose_expansion(&value, &request).unwrap();
        assert_eq!(receipt.occurrences.len(), 2);
        assert_eq!(receipt.occurrences[0].original_start_local, local(2026, 9, 22, 18, 0));
    }

    #[test]
    fn moved_override_can_enter_window_from_outside_scan() {
        let mut value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 1, 18, 0),
            RecurrencePattern::Once,
            RecurrenceEnd::Never,
        );
        value.overrides.push(OccurrenceOverride {
            original_start_local: local(2026, 9, 1, 18, 0),
            action: OverrideAction::Move {
                new_start_local: local(2026, 9, 21, 18, 0),
                duration_minutes: Some(45),
            },
        });
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 22, 0, 0),
            10,
        );
        let receipt = compose_expansion(&value, &request).unwrap();
        assert_eq!(receipt.occurrences.len(), 1);
        assert_eq!(receipt.occurrences[0].original_start_local, local(2026, 9, 1, 18, 0));
        assert_eq!(receipt.occurrences[0].requested_start_local, local(2026, 9, 21, 18, 0));
        assert_eq!(receipt.occurrences[0].disposition, InstanceDisposition::MovedOverride);
    }

    #[test]
    fn override_targeting_non_occurrence_fails_closed() {
        let mut value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 1, 18, 0),
            RecurrencePattern::Daily { interval: 2 },
            RecurrenceEnd::Never,
        );
        value.overrides.push(OccurrenceOverride {
            original_start_local: local(2026, 9, 2, 18, 0),
            action: OverrideAction::Move {
                new_start_local: local(2026, 9, 21, 18, 0),
                duration_minutes: None,
            },
        });
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 22, 0, 0),
            10,
        );
        assert!(matches!(
            compose_expansion(&value, &request),
            Err(CompositionError::ExceptionTargetsNonOccurrence(_))
        ));
    }

    #[test]
    fn dst_gap_reject_surfaces_unresolved_obligation() {
        let value = spec(
            "America/New_York",
            local(2024, 3, 10, 2, 30),
            RecurrencePattern::Once,
            RecurrenceEnd::Never,
        );
        let request = request_for_local_day_range(
            "America/New_York",
            local(2024, 3, 10, 0, 0),
            local(2024, 3, 10, 5, 0),
            10,
        );
        assert!(matches!(
            compose_expansion(&value, &request),
            Err(CompositionError::CivilTimeRejected(_))
        ));
    }

    #[test]
    fn dst_gap_shift_forward_is_evidence_bearing() {
        let mut value = spec(
            "America/New_York",
            local(2024, 3, 10, 2, 30),
            RecurrencePattern::Once,
            RecurrenceEnd::Never,
        );
        value.civil_time_policy = CivilTimePolicy {
            gap: GapPolicy::ShiftForward,
            fold: FoldPolicy::Reject,
        };
        let request = request_for_local_day_range(
            "America/New_York",
            local(2024, 3, 10, 0, 0),
            local(2024, 3, 10, 5, 0),
            10,
        );
        let receipt = compose_expansion(&value, &request).unwrap();
        assert_eq!(receipt.occurrences.len(), 1);
        assert_eq!(receipt.occurrences[0].requested_start_local, local(2024, 3, 10, 2, 30));
        assert_eq!(receipt.occurrences[0].effective_start_local, local(2024, 3, 10, 3, 30));
    }

    #[test]
    fn result_limit_is_error_not_truncation() {
        let value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 21, 18, 0),
            RecurrencePattern::Daily { interval: 1 },
            RecurrenceEnd::Count(3),
        );
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 24, 0, 0),
            2,
        );
        assert_eq!(
            compose_expansion(&value, &request),
            Err(CompositionError::OutputLimitExceeded { count: 3, max: 2 })
        );
    }

    #[test]
    fn operational_exception_cap_precedes_expensive_validation() {
        let mut value = spec(
            "Africa/Johannesburg",
            local(2026, 9, 21, 18, 0),
            RecurrencePattern::Daily { interval: 1 },
            RecurrenceEnd::Never,
        );
        value.exclusions = vec![local(2026, 9, 21, 18, 0); MAX_AUTHORITATIVE_EXCEPTION_ITEMS + 1];
        let request = request_for_local_day_range(
            "Africa/Johannesburg",
            local(2026, 9, 21, 0, 0),
            local(2026, 9, 22, 0, 0),
            10,
        );
        assert_eq!(
            compose_expansion(&value, &request),
            Err(CompositionError::ExceptionLimitExceeded {
                count: MAX_AUTHORITATIVE_EXCEPTION_ITEMS + 1,
                max: MAX_AUTHORITATIVE_EXCEPTION_ITEMS,
            })
        );
    }
}
