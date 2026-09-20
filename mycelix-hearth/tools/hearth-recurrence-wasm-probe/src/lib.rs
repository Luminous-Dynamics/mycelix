// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Non-production link probe for the qualified Care recurrence stack.
//!
//! This crate exists only to force the pinned bundled timezone resolver and
//! A6.4 composer through a real `wasm32-unknown-unknown` final link. It is not
//! imported by Hearth product zomes and has no authority role.

use hearth_care_recurrence::{
    CareRecurrenceSpec, CivilTimePolicy, ExpansionRequest, LocalDate, LocalDateTime,
    LocalTime, MissedOccurrencePolicy, RecurrenceEnd, RecurrencePattern, TimeZoneId,
    RECURRENCE_SCHEMA_VERSION,
};
use hearth_care_recurrence_composer::compose_expansion;
use hearth_care_timezone_jiff::resolve_local;

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

/// Execute one exact Johannesburg recurrence through the bundled timezone
/// resolver and the authoritative composer. Returning the resolved UTC instant
/// gives the linker a concrete dependency on both layers.
pub fn run_probe() -> Result<i64, String> {
    let timezone = TimeZoneId("Africa/Johannesburg".into());
    let starts_local = local(2026, 9, 21, 18, 0);
    let policy = CivilTimePolicy::default();
    let resolved = resolve_local(&timezone, &starts_local, policy)
        .map_err(|error| format!("timezone probe failed: {error}"))?;

    let spec = CareRecurrenceSpec {
        schema_version: RECURRENCE_SCHEMA_VERSION,
        schedule_ref: "qualification-schedule".into(),
        recurrence_state_ref: "qualification-recurrence-state".into(),
        timezone,
        starts_local,
        duration_minutes: 45,
        pattern: RecurrencePattern::Daily { interval: 1 },
        end: RecurrenceEnd::Count(1),
        civil_time_policy: policy,
        missed_policy: MissedOccurrencePolicy::SurfaceForReview,
        exclusions: Vec::new(),
        overrides: Vec::new(),
    };
    let request = ExpansionRequest {
        range_start_utc_micros: resolved
            .start_utc_micros
            .checked_sub(1_000_000)
            .ok_or_else(|| "probe range underflow".to_string())?,
        range_end_utc_micros: resolved
            .start_utc_micros
            .checked_add(3_600_000_000)
            .ok_or_else(|| "probe range overflow".to_string())?,
        max_instances: 2,
    };
    let receipt = compose_expansion(&spec, &request)
        .map_err(|error| format!("composer probe failed: {error}"))?;
    if receipt.occurrences.len() != 1 {
        return Err(format!(
            "expected one qualification occurrence, got {}",
            receipt.occurrences.len()
        ));
    }
    let occurrence = &receipt.occurrences[0];
    if occurrence.start_utc_micros != resolved.start_utc_micros {
        return Err("timezone/composer UTC mismatch".into());
    }
    Ok(occurrence.start_utc_micros)
}

/// Stable export used only to force a final WASM link during qualification.
#[unsafe(no_mangle)]
pub extern "C" fn hearth_recurrence_wasm_probe() -> i64 {
    run_probe().unwrap_or(i64::MIN)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_probe_executes_the_same_stack() {
        assert_ne!(run_probe().unwrap(), i64::MIN);
    }
}
