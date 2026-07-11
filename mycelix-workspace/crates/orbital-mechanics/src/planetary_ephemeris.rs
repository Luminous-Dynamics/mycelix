// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Real planetary ephemeris via VSOP87 (Meeus's published planetary theory,
//! wrapped from the `astro` crate) -- heliocentric position AND velocity as
//! a function of calendar date. Needed for gravity-assist flyby-chain
//! sequencing (Phase 3 of `SPACE_AUTOMATION_PLAN_2026-07-06.md`).
//!
//! `solar_system.rs`'s catalog has static orbital ELEMENTS (semi-major
//! axis, eccentricity, period) but no epoch-referenced phase angle -- it
//! can't tell you WHERE a planet is on a given date. This module can, using
//! the same published VSOP87 theory professional planning software uses.
//! Accuracy is arcsecond-level over centuries -- not JPL DE440/441
//! precision, but squarely "planning-grade", the bar `solar_system.rs`'s
//! own doc comment sets.
//!
//! Two dead entries already sat in `mycelix-workspace/Cargo.toml`'s
//! `[workspace.dependencies]` before this (`planetary-ephemeris = "0.1"`,
//! `astro-coords = "0.1"`) -- verified 2026-07-07 that NEITHER resolves on
//! crates.io at all (not yanked, never existed), and neither is used by any
//! member crate. `astro = "2.0.0"` here is a real, verified, building
//! dependency instead.

use nalgebra::Vector3;

pub use astro::planet::Planet;

use crate::solar_system::AU_KM;

/// Julian Day for a calendar date (UTC, Gregorian calendar).
pub fn julian_day(year: i16, month: u8, decimal_day: f64) -> f64 {
    astro::time::julian_day(&astro::time::Date {
        year,
        month,
        decimal_day,
        cal_type: astro::time::CalType::Gregorian,
    })
}

/// Heliocentric position, km, in VSOP87's ecliptic-of-date frame (see
/// `astro::planet::heliocent_coords` docs) -- real position, not a static
/// catalog radius.
pub fn heliocentric_position_km(planet: &Planet, jd: f64) -> Vector3<f64> {
    let (long, lat, radius_au) = astro::planet::heliocent_coords(planet, jd);
    let r_km = radius_au * AU_KM;
    Vector3::new(
        r_km * lat.cos() * long.cos(),
        r_km * lat.cos() * long.sin(),
        r_km * lat.sin(),
    )
}

/// Heliocentric velocity, km/s, via central-difference numerical
/// differentiation of `heliocentric_position_km`. VSOP87 positions are
/// smooth to arcsecond accuracy, so a small (half-day) step gives an
/// accurate velocity without needing a separate closed-form derivative of
/// the VSOP87 series.
pub fn heliocentric_velocity_kms(planet: &Planet, jd: f64) -> Vector3<f64> {
    const DT_DAYS: f64 = 0.5;
    const SECONDS_PER_DAY: f64 = 86400.0;
    let p_plus = heliocentric_position_km(planet, jd + DT_DAYS);
    let p_minus = heliocentric_position_km(planet, jd - DT_DAYS);
    (p_plus - p_minus) / (2.0 * DT_DAYS * SECONDS_PER_DAY)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// J2000.0 epoch, a standard reference date.
    fn j2000() -> f64 {
        julian_day(2000, 1, 1.5)
    }

    #[test]
    fn test_earth_distance_matches_known_astronomy() {
        // Earth's heliocentric distance varies within
        // a*(1-e) to a*(1+e) = ~0.983 to ~1.017 AU (eccentricity 0.0167).
        // Any real ephemeris must land in this well-known range.
        let pos = heliocentric_position_km(&Planet::Earth, j2000());
        let dist_au = pos.norm() / AU_KM;
        assert!(
            (0.983..1.017).contains(&dist_au),
            "Earth heliocentric distance {dist_au} AU outside known range"
        );
    }

    #[test]
    fn test_earth_speed_matches_known_astronomy() {
        // Earth's average orbital speed is ~29.78 km/s (well-known figure).
        // Instantaneous speed varies with eccentricity but should be
        // within a few percent of this at any date.
        let v = heliocentric_velocity_kms(&Planet::Earth, j2000());
        let speed = v.norm();
        assert!(
            (28.0..31.5).contains(&speed),
            "Earth heliocentric speed {speed} km/s outside expected range"
        );
    }

    #[test]
    fn test_jupiter_farther_and_slower_than_earth() {
        // Basic Kepler sanity: Jupiter orbits farther out and slower than
        // Earth -- cheap, real cross-check between two different planets'
        // ephemeris output.
        let jd = j2000();
        let earth_dist = heliocentric_position_km(&Planet::Earth, jd).norm();
        let jupiter_dist = heliocentric_position_km(&Planet::Jupiter, jd).norm();
        assert!(jupiter_dist > earth_dist * 4.0); // Jupiter is ~5.2 AU out

        let earth_speed = heliocentric_velocity_kms(&Planet::Earth, jd).norm();
        let jupiter_speed = heliocentric_velocity_kms(&Planet::Jupiter, jd).norm();
        assert!(jupiter_speed < earth_speed); // ~13 km/s vs ~29.78 km/s
    }

    #[test]
    fn test_position_is_deterministic_and_finite() {
        let jd = j2000();
        let a = heliocentric_position_km(&Planet::Saturn, jd);
        let b = heliocentric_position_km(&Planet::Saturn, jd);
        assert_eq!(a, b);
        assert!(a.iter().all(|c| c.is_finite()));
    }

    #[test]
    fn test_position_changes_meaningfully_over_a_year() {
        // Earth should have moved most of the way around its orbit after
        // ~1 year -- a coarse but real check that the ephemeris actually
        // varies with time rather than returning a fixed value.
        let jd0 = j2000();
        let p0 = heliocentric_position_km(&Planet::Earth, jd0);
        let p1 = heliocentric_position_km(&Planet::Earth, jd0 + 365.25 / 2.0);
        // Half a year later, Earth should be roughly on the opposite side
        // of the Sun -- position should have reversed sign-ish (large
        // separation, not a small drift).
        let separation = (p1 - p0).norm();
        assert!(
            separation > AU_KM,
            "expected >1 AU separation after half a year, got {} km",
            separation
        );
    }
}
