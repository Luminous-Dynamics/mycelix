// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Clohessy-Wiltshire (Hill's) Relative Motion Equations
//!
//! Linearized relative dynamics of a chaser spacecraft with respect to a
//! target on a circular reference orbit, in the target's local-vertical-
//! local-horizontal (LVLH / Hill) frame:
//!
//! - x: radial, positive away from Earth center
//! - y: along-track, positive in the direction of orbital motion
//! - z: cross-track, positive out of the orbital plane (completes a
//!   right-handed frame)
//!
//! Governing equations (n = mean motion of the reference circular orbit):
//! ```text
//! x'' - 2n*y' - 3n²x = ax
//! y'' + 2n*x'        = ay
//! z'' + n²z          = az
//! ```
//!
//! With no control input (ax=ay=az=0), this has the closed-form state
//! transition solution used here (Clohessy & Wiltshire 1960; see e.g.
//! Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed.,
//! §6.6, or Curtis, *Orbital Mechanics for Engineering Students*, ch. 7).
//! Valid for close relative separations (well inside the reference orbit's
//! radius) and a circular (or near-circular) reference orbit -- standard
//! assumptions for final-approach rendezvous/proximity operations, not for
//! full-orbit relative motion.

use crate::coordinates::wgs84::MU;

/// Mean motion (rad/s) of a circular reference orbit at the given
/// semi-major axis (km): n = sqrt(mu / a^3).
pub fn mean_motion(semi_major_axis_km: f64) -> f64 {
    (MU / semi_major_axis_km.powi(3)).sqrt()
}

/// Relative state in the target's LVLH frame. Position in meters, velocity
/// in m/s -- deliberately finer units than the km/km-s used for absolute
/// orbital state elsewhere in this crate, since CW is meant for
/// close-proximity operations (meters to a few km), not whole-orbit scales.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelativeState {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
}

impl RelativeState {
    pub fn zero() -> Self {
        Self {
            position_m: [0.0; 3],
            velocity_mps: [0.0; 3],
        }
    }

    pub fn distance_m(&self) -> f64 {
        (self.position_m[0].powi(2) + self.position_m[1].powi(2) + self.position_m[2].powi(2))
            .sqrt()
    }

    pub fn speed_mps(&self) -> f64 {
        (self.velocity_mps[0].powi(2) + self.velocity_mps[1].powi(2) + self.velocity_mps[2].powi(2))
            .sqrt()
    }

    pub fn is_finite(&self) -> bool {
        self.position_m
            .iter()
            .chain(self.velocity_mps.iter())
            .all(|v| v.is_finite())
    }
}

/// Propagate a relative state forward by `dt_s` seconds using the
/// closed-form Clohessy-Wiltshire state transition matrix (no control
/// input -- ballistic relative motion under the reference orbit's
/// tidal/Coriolis terms only). `mean_motion_rad_s` is the reference
/// (target) orbit's mean motion -- see [`mean_motion`].
///
/// In-plane (x, y) and out-of-plane (z) motion are independent in the
/// linearized CW equations, so z is pure simple-harmonic motion at
/// frequency n while x/y follow the coupled radial/along-track solution.
pub fn propagate_cw(state: &RelativeState, mean_motion_rad_s: f64, dt_s: f64) -> RelativeState {
    let n = mean_motion_rad_s;
    let nt = n * dt_s;
    let (sin_nt, cos_nt) = nt.sin_cos();

    let x0 = state.position_m[0];
    let y0 = state.position_m[1];
    let z0 = state.position_m[2];
    let vx0 = state.velocity_mps[0];
    let vy0 = state.velocity_mps[1];
    let vz0 = state.velocity_mps[2];

    let x = (4.0 - 3.0 * cos_nt) * x0 + (sin_nt / n) * vx0 + (2.0 / n) * (1.0 - cos_nt) * vy0;
    let y = 6.0 * (sin_nt - nt) * x0 + y0 - (2.0 / n) * (1.0 - cos_nt) * vx0
        + (1.0 / n) * (4.0 * sin_nt - 3.0 * nt) * vy0;
    let z = z0 * cos_nt + (vz0 / n) * sin_nt;

    let vx = 3.0 * n * sin_nt * x0 + cos_nt * vx0 + 2.0 * sin_nt * vy0;
    let vy = 6.0 * n * (cos_nt - 1.0) * x0 - 2.0 * sin_nt * vx0 + (4.0 * cos_nt - 3.0) * vy0;
    let vz = -z0 * n * sin_nt + vz0 * cos_nt;

    RelativeState {
        position_m: [x, y, z],
        velocity_mps: [vx, vy, vz],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mean_motion_matches_leo_period() {
        // ~400km LEO altitude, semi-major axis = R_earth + 400.
        let a = crate::coordinates::wgs84::A + 400.0;
        let n = mean_motion(a);
        let period_s = 2.0 * std::f64::consts::PI / n;
        assert!(
            (period_s - 5554.0).abs() < 5.0,
            "expected ~5554s LEO period, got {period_s}"
        );
    }

    #[test]
    fn test_zero_state_stays_at_origin() {
        let n = mean_motion(crate::coordinates::wgs84::A + 400.0);
        let s = propagate_cw(&RelativeState::zero(), n, 100.0);
        assert_eq!(s, RelativeState::zero());
    }

    #[test]
    fn test_pure_radial_offset_is_not_stationary() {
        // A purely radial offset with zero relative velocity is NOT an
        // equilibrium of the CW equations (unlike free-particle intuition)
        // -- the -3n^2*x tidal term drives it away, and it couples into
        // along-track motion via the Coriolis-like -2n*y' term. This is a
        // well-known, textbook property of CW dynamics (a radial-only
        // "station-keeping" offset is unstable); verifying it here guards
        // against a sign error that would make radial offsets falsely
        // stable.
        let n = mean_motion(crate::coordinates::wgs84::A + 400.0);
        let s0 = RelativeState {
            position_m: [100.0, 0.0, 0.0],
            velocity_mps: [0.0, 0.0, 0.0],
        };
        let s1 = propagate_cw(&s0, n, 60.0);
        assert!(
            (s1.position_m[0] - 100.0).abs() > 1e-6 || s1.position_m[1].abs() > 1e-6,
            "expected a radial offset to drift under CW dynamics, got {s1:?}"
        );
    }

    #[test]
    fn test_cross_track_is_simple_harmonic() {
        // z-axis motion is decoupled and must be pure SHM: amplitude
        // preserved, and after a quarter period velocity/position swap
        // roles (z(T/4) ~= 0 if starting from rest, matching cos/sin).
        let n = mean_motion(crate::coordinates::wgs84::A + 400.0);
        let s0 = RelativeState {
            position_m: [0.0, 0.0, 50.0],
            velocity_mps: [0.0, 0.0, 0.0],
        };
        let quarter_period_s = (std::f64::consts::PI / 2.0) / n;
        let s1 = propagate_cw(&s0, n, quarter_period_s);
        assert!(
            s1.position_m[2].abs() < 1e-3,
            "expected z ~= 0 after a quarter period starting from rest, got {}",
            s1.position_m[2]
        );
        assert!(
            (s1.velocity_mps[2].abs() - 50.0 * n).abs() < 1e-3,
            "expected |vz| ~= z0*n after a quarter period, got {}",
            s1.velocity_mps[2]
        );
        // x/y must be completely unaffected by a pure z offset (decoupling).
        assert_eq!(s1.position_m[0], 0.0);
        assert_eq!(s1.position_m[1], 0.0);
    }

    #[test]
    fn test_propagation_is_deterministic() {
        let n = mean_motion(crate::coordinates::wgs84::A + 400.0);
        let s0 = RelativeState {
            position_m: [200.0, -500.0, 30.0],
            velocity_mps: [0.5, -1.0, 0.1],
        };
        let a = propagate_cw(&s0, n, 123.4);
        let b = propagate_cw(&s0, n, 123.4);
        assert_eq!(a, b);
    }

    #[test]
    fn test_finite_over_many_steps() {
        let n = mean_motion(crate::coordinates::wgs84::A + 400.0);
        let mut s = RelativeState {
            position_m: [1000.0, -2000.0, 100.0],
            velocity_mps: [0.1, -2.0, 0.0],
        };
        for _ in 0..10_000 {
            s = propagate_cw(&s, n, 1.0);
            assert!(s.is_finite());
        }
    }
}
