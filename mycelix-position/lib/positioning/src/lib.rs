// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Decentralized cooperative positioning library.
//!
//! Pure Rust implementation of positioning algorithms for peer-to-peer
//! localization without GPS/GNSS infrastructure. Body-agnostic: works
//! on Earth (WGS-84), Moon (IAU sphere), and Mars (IAU ellipsoid).
//!
//! # Architecture
//!
//! This library provides the mathematical primitives. It has zero
//! Holochain dependencies — the trust fabric and DHT storage live
//! in the zome layer above.
//!
//! # Modules
//!
//! - [`bodies`] — Celestial body trait + Earth/Moon/Mars implementations
//! - [`ranging`] — Sensor models: RSSI, LoRa ToA, UWB ToF, WiFi RTT
//! - [`trilateration`] — Core positioning: N ranges → position estimate
//! - [`coverage`] — GDOP/PDOP geometric dilution of precision
//! - [`kalman`] — Extended Kalman Filter for continuous tracking
//! - [`qualification`] — theorem dependency/requirement meta-structure

pub mod bodies;
pub mod coverage;
pub mod dead_reckoning;
pub mod fusion;
// These fixed-size numerical kernels intentionally retain explicit row/column
// index notation. It maps directly to the documented matrix equations and is
// easier to audit for transposition/indexing errors than iterator rewrites.
// Keep this exception local to the numerical modules rather than weakening the
// workspace-wide `-D warnings` qualification gate.
#[allow(clippy::needless_range_loop)]
pub mod kalman;
pub mod measurements;
pub mod navigation_runtime;
pub mod qualification;
pub mod ranging;
pub mod space_navigation;
#[allow(clippy::needless_range_loop)]
pub mod trilateration;

pub use bodies::{CelestialBody, Earth, Mars, Moon};
pub use coverage::{CoveragePoint, gdop, pdop};
pub use dead_reckoning::{PdrConfig, PedestrianDeadReckoning, barometric_altitude};
pub use fusion::{
    GaussianEstimate3D, PeerEstimate3D, PeerFusion3D, PublishableEstimate3D,
    covariance_intersection_3d,
};
pub use kalman::{FilterConfig, FilterState, PositionFilter};
pub use measurements::{
    Measurement, MeasurementModality, MeasurementProvenance, MeasurementValue, ReferenceFrame,
};
pub use navigation_runtime::{
    DomainNavigator, MeasurementRouter, MeasurementRoutingPolicy, MeasurementRoutingStats,
    NavigationFailoverMode, NavigationHealth, confidence_from_sigma, fix_age_s,
};
pub use qualification::{
    FacetStatus, QualificationError, QualificationFacet, QualificationManifest,
    QualificationRequirementProfile, RequirementEvaluation, TheoremDefinition, TheoremId,
    TheoremRegistry, UnacceptableFacet,
};
pub use ranging::{RangeEstimate, RangingMethod};
pub use space_navigation::{SpaceNavigationEstimate, SpaceNavigationEstimator};
pub use trilateration::{PositionEstimate, TrilaterationError, trilaterate_2d, trilaterate_3d};
