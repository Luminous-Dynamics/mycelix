// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Orbital Mechanics Library for Mycelix-Space
//!
//! This library provides:
//! - TLE parsing and validation
//! - SGP4/SDP4 orbital propagation
//! - Covariance matrix propagation (uncertainty tracking)
//! - Conjunction analysis (collision probability)
//! - Coordinate transformations (ECI, ECEF, geodetic)
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Orbital State Model                          │
//! ├─────────────────────────────────────────────────────────────────┤
//! │                                                                 │
//! │  TLE Input ──────► SGP4 Elements ──────► State Vector           │
//! │       │                   │                    │                │
//! │       │                   │                    ▼                │
//! │       │                   │           ┌──────────────┐          │
//! │       │                   │           │ Position (3) │          │
//! │       │                   │           │ Velocity (3) │          │
//! │       │                   │           │ Covariance   │          │
//! │       │                   │           │   (6x6)      │          │
//! │       │                   │           └──────────────┘          │
//! │       │                   │                    │                │
//! │       │                   ▼                    ▼                │
//! │       │           ┌─────────────┐    ┌─────────────────┐        │
//! │       │           │ Propagator  │───►│ Future State    │        │
//! │       │           │ (SGP4/SDP4) │    │ + Uncertainty   │        │
//! │       │           └─────────────┘    └─────────────────┘        │
//! │       │                                       │                 │
//! │       ▼                                       ▼                 │
//! │  ┌──────────┐                        ┌───────────────┐          │
//! │  │ Validate │                        │ Conjunction   │          │
//! │  │ Checksum │                        │ Analysis      │          │
//! │  └──────────┘                        └───────────────┘          │
//! └─────────────────────────────────────────────────────────────────┘
//! ```

pub mod atmosphere;
pub mod cdm;
pub mod cdm_parser;
pub mod clohessy_wiltshire;
pub mod conjunction;
pub mod conjunction_network;
pub mod coordinates;
pub mod covariance;
pub mod fusion;
pub mod gravity_assist;
pub mod keplerian;
pub mod lambert;
pub mod orbit_determination;
pub mod planetary_ephemeris;
pub mod propagator;
pub mod state;
pub mod tle;

pub use atmosphere::{
    DragConfig, density as atmospheric_density, drag_acceleration, estimate_lifetime,
};
pub use cdm::{
    CdmBuilder, CdmCovariance, CdmObjectMetadata, CdmRefFrame, CdmStateVector,
    ConjunctionDataMessage, DEFAULT_HARD_BODY_RADIUS_M, Maneuverable, PcMethod,
};
pub use cdm_parser::{CdmParseError, parse_cdm_kvn};
pub use clohessy_wiltshire::{RelativeState, mean_motion, propagate_cw};
pub use conjunction::{CollisionProbability, ConjunctionAssessment};
pub use covariance::CovarianceMatrix;
pub use fusion::{FusedEstimate, FusionPipeline, SensorMeasurement};
pub use gravity_assist::{apply_gravity_assist, turn_angle_rad};
pub use keplerian::{
    ImpulsiveManeuver, KeplerianElements, KeplerianError, collision_avoidance_maneuver,
    hohmann_transfer,
};
pub use lambert::{LambertSolution, TransferType, solve_lambert};
pub use orbit_determination::{ObservationRecord, ObservationType, OrbitDetermination, gauss_iod};
pub use planetary_ephemeris::{
    Planet, heliocentric_position_km, heliocentric_velocity_kms, julian_day,
};
pub use propagator::{PropagationError, Propagator};
pub use state::{OrbitalState, StateVector};
pub use tle::{TleParseError, TwoLineElement};

pub mod relativity;
pub mod solar_system;
#[cfg(test)]
mod validation;
