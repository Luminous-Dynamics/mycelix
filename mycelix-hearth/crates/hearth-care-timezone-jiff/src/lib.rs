// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pinned, bundled-tzdb civil-time resolution for Hearth Care recurrence.
//!
//! This crate resolves one already-selected local recurrence instance and maps
//! exact UTC instants back to local civil time for bounded expansion. It does
//! not enumerate recurrence rules. All timezone lookup goes through
//! `TimeZoneDatabase::bundled()` with exact-pinned Jiff dependencies; implicit
//! global/system timezone lookup is intentionally absent.

use hearth_care_recurrence::{
    CivilResolution, CivilTimePolicy, ExpansionEngineEvidence, FoldPolicy, GapPolicy,
    LocalDate, LocalDateTime, LocalTime, TimeZoneId,
};
use jiff::{
    civil::DateTime,
    tz::{AmbiguousOffset, TimeZoneDatabase},
    Timestamp, Zoned,
};

pub const ENGINE_ID: &str = "hearth-jiff-tz-v1";
pub const JIFF_VERSION: &str = "0.2.37";
pub const JIFF_TZDB_CRATE_VERSION: &str = "0.1.8";

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TimeZoneEngineError {
    InvalidContract(String),
    BundledDatabaseUnavailable,
    MissingBundledDatabaseVersion,
    UnknownTimeZone(String),
    InvalidCivilDateTime(String),
    InvalidTimestamp(String),
    GapRejected,
    FoldRejected,
    JiffResolution(String),
    InvalidResolvedComponents,
    InvalidGapShift,
}

impl std::fmt::Display for TimeZoneEngineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for TimeZoneEngineError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedCivilTime {
    pub requested_local: LocalDateTime,
    pub effective_local: LocalDateTime,
    pub start_utc_micros: i64,
    pub resolution: CivilResolution,
    pub requested_timezone: TimeZoneId,
    pub canonical_timezone: String,
    pub engine: ExpansionEngineEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LocalizedUtcInstant {
    pub utc_micros: i64,
    pub local: LocalDateTime,
    pub requested_timezone: TimeZoneId,
    pub canonical_timezone: String,
    pub engine: ExpansionEngineEvidence,
}

/// Exact engine/tzdb identity emitted into recurrence expansion evidence.
pub fn engine_evidence() -> Result<ExpansionEngineEvidence, TimeZoneEngineError> {
    let version = jiff_tzdb::VERSION.ok_or(TimeZoneEngineError::MissingBundledDatabaseVersion)?;
    Ok(ExpansionEngineEvidence {
        engine_id: ENGINE_ID.into(),
        engine_version: format!("jiff:{JIFF_VERSION};jiff-tzdb:{JIFF_TZDB_CRATE_VERSION}"),
        tzdb_ref: format!("iana:{version}"),
    })
}

/// Resolve one local wall-clock request using only the exact bundled timezone DB.
///
/// A gap/fold is never silently resolved via Jiff's default Compatible policy.
/// The A6.1 `CivilTimePolicy` is the only authority for disambiguation here.
pub fn resolve_local(
    timezone: &TimeZoneId,
    requested: &LocalDateTime,
    policy: CivilTimePolicy,
) -> Result<ResolvedCivilTime, TimeZoneEngineError> {
    timezone
        .validate()
        .map_err(|error| TimeZoneEngineError::InvalidContract(error.to_string()))?;
    requested
        .validate()
        .map_err(|error| TimeZoneEngineError::InvalidContract(error.to_string()))?;

    let dt: DateTime = requested
        .canonical()
        .parse()
        .map_err(|error: jiff::Error| TimeZoneEngineError::InvalidCivilDateTime(error.to_string()))?;

    let (tz, canonical_timezone) = bundled_zone(timezone)?;
    let ambiguous = tz.to_ambiguous_zoned(dt);

    let (zoned, resolution) = match ambiguous.offset() {
        AmbiguousOffset::Unambiguous { .. } => (
            ambiguous
                .unambiguous()
                .map_err(|error| TimeZoneEngineError::JiffResolution(error.to_string()))?,
            CivilResolution::Exact,
        ),
        AmbiguousOffset::Gap { .. } => match policy.gap {
            GapPolicy::Reject => return Err(TimeZoneEngineError::GapRejected),
            GapPolicy::ShiftForward => {
                let zoned = ambiguous
                    .later()
                    .map_err(|error| TimeZoneEngineError::JiffResolution(error.to_string()))?;
                let shift = dt.duration_until(zoned.datetime()).as_secs();
                let shift_seconds = u32::try_from(shift)
                    .ok()
                    .filter(|seconds| *seconds > 0 && *seconds <= 86_400)
                    .ok_or(TimeZoneEngineError::InvalidGapShift)?;
                (
                    zoned,
                    CivilResolution::GapShiftForward { shift_seconds },
                )
            }
        },
        AmbiguousOffset::Fold { .. } => match policy.fold {
            FoldPolicy::Reject => return Err(TimeZoneEngineError::FoldRejected),
            FoldPolicy::Earlier => (
                ambiguous
                    .earlier()
                    .map_err(|error| TimeZoneEngineError::JiffResolution(error.to_string()))?,
                CivilResolution::FoldEarlier,
            ),
            FoldPolicy::Later => (
                ambiguous
                    .later()
                    .map_err(|error| TimeZoneEngineError::JiffResolution(error.to_string()))?,
                CivilResolution::FoldLater,
            ),
        },
        _ => {
            return Err(TimeZoneEngineError::JiffResolution(
                "unsupported future Jiff ambiguity classification".into(),
            ))
        }
    };

    Ok(ResolvedCivilTime {
        requested_local: requested.clone(),
        effective_local: contract_datetime(&zoned)?,
        start_utc_micros: zoned.timestamp().as_microsecond(),
        resolution,
        requested_timezone: timezone.clone(),
        canonical_timezone,
        engine: engine_evidence()?,
    })
}

/// Convert one exact UTC microsecond instant to local civil time using the same
/// explicitly bundled database. This is used only to derive bounded local scan
/// windows for recurrence expansion; it performs no recurrence logic.
pub fn local_at_utc(
    timezone: &TimeZoneId,
    utc_micros: i64,
) -> Result<LocalizedUtcInstant, TimeZoneEngineError> {
    timezone
        .validate()
        .map_err(|error| TimeZoneEngineError::InvalidContract(error.to_string()))?;
    let (tz, canonical_timezone) = bundled_zone(timezone)?;
    let timestamp = Timestamp::from_microsecond(utc_micros)
        .map_err(|error| TimeZoneEngineError::InvalidTimestamp(error.to_string()))?;
    let zoned = timestamp.to_zoned(tz);
    Ok(LocalizedUtcInstant {
        utc_micros,
        local: contract_datetime(&zoned)?,
        requested_timezone: timezone.clone(),
        canonical_timezone,
        engine: engine_evidence()?,
    })
}

fn bundled_zone(
    timezone: &TimeZoneId,
) -> Result<(jiff::tz::TimeZone, String), TimeZoneEngineError> {
    let db = TimeZoneDatabase::bundled();
    if db.is_definitively_empty() {
        return Err(TimeZoneEngineError::BundledDatabaseUnavailable);
    }
    let tz = db
        .get(&timezone.0)
        .map_err(|_| TimeZoneEngineError::UnknownTimeZone(timezone.0.clone()))?;
    let canonical = tz.iana_name().unwrap_or(&timezone.0).to_string();
    Ok((tz, canonical))
}

fn contract_datetime(zoned: &Zoned) -> Result<LocalDateTime, TimeZoneEngineError> {
    let dt = zoned.datetime();
    let year = i32::from(dt.year());
    let month = u8::try_from(dt.month()).map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    let day = u8::try_from(dt.day()).map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    let hour = u8::try_from(dt.hour()).map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    let minute = u8::try_from(dt.minute()).map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    let second = u8::try_from(dt.second()).map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    let value = LocalDateTime {
        date: LocalDate { year, month, day },
        time: LocalTime {
            hour,
            minute,
            second,
        },
    };
    value
        .validate()
        .map_err(|_| TimeZoneEngineError::InvalidResolvedComponents)?;
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn engine_is_bound_to_embedded_iana_version() {
        let evidence = engine_evidence().unwrap();
        assert_eq!(evidence.engine_id, ENGINE_ID);
        assert_eq!(evidence.tzdb_ref, "iana:2026c");
    }

    #[test]
    fn johannesburg_has_exact_resolution() {
        let result = resolve_local(
            &TimeZoneId("Africa/Johannesburg".into()),
            &local(2026, 9, 21, 18, 0),
            CivilTimePolicy::default(),
        )
        .unwrap();
        assert_eq!(result.resolution, CivilResolution::Exact);
        assert_eq!(result.effective_local, local(2026, 9, 21, 18, 0));
        assert_eq!(result.canonical_timezone, "Africa/Johannesburg");
    }

    #[test]
    fn exact_utc_to_local_uses_same_bundle() {
        let resolved = resolve_local(
            &TimeZoneId("Africa/Johannesburg".into()),
            &local(2026, 9, 21, 18, 0),
            CivilTimePolicy::default(),
        )
        .unwrap();
        let localized = local_at_utc(
            &TimeZoneId("Africa/Johannesburg".into()),
            resolved.start_utc_micros,
        )
        .unwrap();
        assert_eq!(localized.local, local(2026, 9, 21, 18, 0));
        assert_eq!(localized.engine, resolved.engine);
    }

    #[test]
    fn canonicalizes_zone_name_without_using_global_database() {
        let result = resolve_local(
            &TimeZoneId("america/new_york".into()),
            &local(2024, 7, 15, 17, 30),
            CivilTimePolicy::default(),
        )
        .unwrap();
        assert_eq!(result.canonical_timezone, "America/New_York");
    }

    #[test]
    fn gap_reject_is_fail_closed() {
        let result = resolve_local(
            &TimeZoneId("America/New_York".into()),
            &local(2024, 3, 10, 2, 30),
            CivilTimePolicy::default(),
        );
        assert_eq!(result, Err(TimeZoneEngineError::GapRejected));
    }

    #[test]
    fn gap_shift_forward_records_effective_time_and_shift() {
        let result = resolve_local(
            &TimeZoneId("America/New_York".into()),
            &local(2024, 3, 10, 2, 30),
            CivilTimePolicy {
                gap: GapPolicy::ShiftForward,
                fold: FoldPolicy::Reject,
            },
        )
        .unwrap();
        assert_eq!(
            result.resolution,
            CivilResolution::GapShiftForward { shift_seconds: 3_600 }
        );
        assert_eq!(result.effective_local, local(2024, 3, 10, 3, 30));
    }

    #[test]
    fn fold_requires_explicit_side() {
        let requested = local(2024, 11, 3, 1, 30);
        assert_eq!(
            resolve_local(
                &TimeZoneId("America/New_York".into()),
                &requested,
                CivilTimePolicy::default(),
            ),
            Err(TimeZoneEngineError::FoldRejected)
        );

        let earlier = resolve_local(
            &TimeZoneId("America/New_York".into()),
            &requested,
            CivilTimePolicy {
                gap: GapPolicy::Reject,
                fold: FoldPolicy::Earlier,
            },
        )
        .unwrap();
        let later = resolve_local(
            &TimeZoneId("America/New_York".into()),
            &requested,
            CivilTimePolicy {
                gap: GapPolicy::Reject,
                fold: FoldPolicy::Later,
            },
        )
        .unwrap();
        assert_eq!(earlier.resolution, CivilResolution::FoldEarlier);
        assert_eq!(later.resolution, CivilResolution::FoldLater);
        assert_eq!(earlier.effective_local, requested);
        assert_eq!(later.effective_local, requested);
        assert_eq!(later.start_utc_micros - earlier.start_utc_micros, 3_600_000_000);
    }

    #[test]
    fn invalid_zone_never_falls_back_to_system_or_utc() {
        let result = resolve_local(
            &TimeZoneId("Mars/Olympus_Mons".into()),
            &local(2026, 1, 1, 12, 0),
            CivilTimePolicy::default(),
        );
        assert_eq!(
            result,
            Err(TimeZoneEngineError::UnknownTimeZone(
                "Mars/Olympus_Mons".into()
            ))
        );
    }
}
