// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Patched-conic gravity-assist (planetary flyby) velocity change.
//!
//! Standard two-body "patched conic" approximation (Curtis, *Orbital
//! Mechanics for Engineering Students*, ch. 8; Vallado, *Fundamentals of
//! Astrodynamics and Applications*, ch. 12): treat the flyby as an
//! instantaneous hyperbolic deflection in the planet's own reference frame.
//! The spacecraft's speed RELATIVE TO THE PLANET is unchanged by the
//! encounter -- a fundamental invariant of this approximation -- only its
//! DIRECTION changes, by the hyperbolic turn angle. The heliocentric speed
//! change (the "free" energy a gravity assist is famous for) comes entirely
//! from vector-adding this rotated relative velocity back to the planet's
//! own (unchanged) heliocentric velocity.
//!
//! This is genuinely a "grav-craft" in the honest sense the space-
//! automation plan insists on: using gravity as it already exists to
//! reshape a trajectory, not modifying gravity or propulsion itself.

use nalgebra::{Unit, Vector3};

/// Hyperbolic turn angle (radians) for a flyby with the given
/// planet-relative approach speed and periapsis radius:
/// `delta = 2 * asin(1 / (1 + rp * v_inf^2 / mu))`.
pub fn turn_angle_rad(v_infinity_kms: f64, periapsis_radius_km: f64, mu_planet_km3_s2: f64) -> f64 {
    let e_term = 1.0 + periapsis_radius_km * v_infinity_kms.powi(2) / mu_planet_km3_s2;
    2.0 * (1.0 / e_term).asin()
}

/// Apply a patched-conic gravity assist. Rotates the incoming
/// planet-relative velocity by the turn angle within the plane containing
/// that relative-velocity vector and the planet's own heliocentric
/// velocity -- the common "in-plane" flyby geometry. Does not model
/// out-of-plane / 3D-targeted flybys.
///
/// `leading_side` selects which of the two real flyby geometries: passing
/// behind the planet (trailing-side) bends the trajectory to gain
/// heliocentric speed; passing in front (leading-side) bends it to lose
/// speed. Both are genuinely used in real mission design depending on
/// whether the next leg needs a speed boost or a brake.
///
/// Returns the outgoing HELIOCENTRIC velocity, km/s.
pub fn apply_gravity_assist(
    v_sc_heliocentric_in_kms: Vector3<f64>,
    v_planet_heliocentric_kms: Vector3<f64>,
    periapsis_radius_km: f64,
    mu_planet_km3_s2: f64,
    leading_side: bool,
) -> Vector3<f64> {
    let v_inf_in = v_sc_heliocentric_in_kms - v_planet_heliocentric_kms;
    let v_inf_mag = v_inf_in.norm();
    if v_inf_mag < 1e-9 {
        return v_sc_heliocentric_in_kms; // no relative motion, nothing to deflect
    }
    let delta = turn_angle_rad(v_inf_mag, periapsis_radius_km, mu_planet_km3_s2);

    // Rotation axis: normal to the plane containing v_inf_in and the
    // planet's heliocentric velocity (the flyby plane).
    let raw_axis = v_inf_in.cross(&v_planet_heliocentric_kms);
    let axis = if raw_axis.norm() > 1e-9 {
        Unit::new_normalize(raw_axis)
    } else {
        // Degenerate (parallel vectors) -- fall back to any perpendicular.
        Unit::new_normalize(v_inf_in.cross(&Vector3::z()))
    };
    let signed_delta = if leading_side { -delta } else { delta };

    let v_inf_out = rotate_rodrigues(&v_inf_in, &axis, signed_delta);
    v_inf_out + v_planet_heliocentric_kms
}

fn rotate_rodrigues(v: &Vector3<f64>, axis: &Unit<Vector3<f64>>, angle_rad: f64) -> Vector3<f64> {
    let k = axis.into_inner();
    let (sin_a, cos_a) = angle_rad.sin_cos();
    v * cos_a + k.cross(v) * sin_a + k * (k.dot(v)) * (1.0 - cos_a)
}

#[cfg(test)]
mod tests {
    use super::*;

    // Jupiter's gravitational parameter, km^3/s^2 (well-known constant).
    const MU_JUPITER: f64 = 1.26687e8;

    fn sample_scenario() -> (Vector3<f64>, Vector3<f64>) {
        // Illustrative incoming spacecraft velocity and Jupiter's
        // heliocentric velocity (order-of-magnitude realistic, not tied to
        // a specific real encounter date).
        let v_sc_in = Vector3::new(15.0, 20.0, 0.0);
        let v_jupiter = Vector3::new(0.0, 13.0, 0.0);
        (v_sc_in, v_jupiter)
    }

    #[test]
    fn test_relative_speed_is_conserved() {
        // The core patched-conic invariant: |v_infinity| unchanged by the
        // flyby, only its direction. If this fails, the rotation is wrong.
        let (v_sc_in, v_planet) = sample_scenario();
        let v_inf_in_mag = (v_sc_in - v_planet).norm();

        let v_out = apply_gravity_assist(v_sc_in, v_planet, 100_000.0, MU_JUPITER, false);
        let v_inf_out_mag = (v_out - v_planet).norm();

        assert!(
            (v_inf_in_mag - v_inf_out_mag).abs() < 1e-9,
            "expected |v_infinity| conserved: in={v_inf_in_mag}, out={v_inf_out_mag}"
        );
    }

    #[test]
    fn test_trailing_side_boosts_heliocentric_speed() {
        // Trailing-side (leading_side=false) flybys are the classic
        // "speed up" gravity assist (how Voyager gained energy at each
        // outer planet) -- must genuinely increase heliocentric speed
        // compared to the incoming speed, not just change direction
        // arbitrarily.
        let (v_sc_in, v_planet) = sample_scenario();
        let speed_in = v_sc_in.norm();

        let v_out = apply_gravity_assist(v_sc_in, v_planet, 100_000.0, MU_JUPITER, false);
        assert!(
            v_out.norm() > speed_in,
            "expected trailing-side assist to boost heliocentric speed: in={speed_in}, out={}",
            v_out.norm()
        );
    }

    #[test]
    fn test_leading_and_trailing_side_have_opposite_effect() {
        let (v_sc_in, v_planet) = sample_scenario();
        let v_trailing = apply_gravity_assist(v_sc_in, v_planet, 100_000.0, MU_JUPITER, false);
        let v_leading = apply_gravity_assist(v_sc_in, v_planet, 100_000.0, MU_JUPITER, true);
        // One must speed up, the other must NOT speed up as much (in this
        // symmetric turn-angle setup, leading-side should reduce speed
        // relative to trailing-side) -- verifies leading_side actually
        // does something, not a dead parameter.
        assert!(v_trailing.norm() > v_leading.norm());
    }

    #[test]
    fn test_closer_approach_gives_larger_turn_angle() {
        // A closer periapsis means a stronger deflection -- real, monotonic
        // physical relationship any correct implementation must have.
        let v_inf = 10.0;
        let close = turn_angle_rad(v_inf, 71_000.0, MU_JUPITER); // near cloud tops
        let far = turn_angle_rad(v_inf, 1_000_000.0, MU_JUPITER);
        assert!(
            close > far,
            "closer approach ({close}) should turn more than farther ({far})"
        );
    }

    #[test]
    fn test_zero_relative_velocity_returns_unchanged() {
        let v = Vector3::new(13.0, 0.0, 0.0);
        let out = apply_gravity_assist(v, v, 100_000.0, MU_JUPITER, false);
        assert_eq!(out, v);
    }
}
