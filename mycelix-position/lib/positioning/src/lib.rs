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
//! - [`qualification_hardening`] — bounded non-recursive qualification evaluation
//! - [`qualification_admission`] — pre-parse wire admission + durable aggregate budgets
//! - [`qualification_envelope`] — version-first canonical qualification wire grammar
//! - [`qualification_digest`] — provider-neutral exact-byte digest identity contract

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
pub mod qualification_admission;
pub mod qualification_digest;
pub mod qualification_envelope;
pub mod qualification_hardening;
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
pub use qualification_admission::{
    AdmissionAggregateResource, AdmissionLimitField, AdmittedQualificationBytes,
    ProfiledQualificationRegistryV1, QUALIFICATION_ADMISSION_PROFILE_PREIMAGE_V1,
    QUALIFICATION_ADMISSION_PROFILE_SCHEMA_V1, QualificationAdmissionError,
    QualificationAdmissionProfileV1, admit_qualification_bytes,
    canonical_qualification_admission_profile_preimage_v1,
};
pub use qualification_digest::{
    DigestIdentifierField, QUALIFICATION_DIGEST_PROFILE_PREIMAGE_V1,
    QUALIFICATION_DIGEST_PROFILE_SCHEMA_V1, QualificationDigestError, QualificationDigestProfileV1,
    QualificationDigestProvider, QualificationDigestProviderError, QualificationDigestValueV1,
    QualificationEnvelopeDigestV1, canonical_qualification_digest_profile_preimage_v1,
    digest_admitted_qualification_envelope_v1,
};
pub use qualification_envelope::{
    DecodedQualificationEnvelope, QUALIFICATION_ENVELOPE_ENCODING_V1,
    QUALIFICATION_ENVELOPE_VERSION_V1, QualificationEnvelopeCodecV1, QualificationEnvelopeError,
    QualificationEnvelopeKind, QualificationEnvelopeLimit, QualificationEnvelopePayload,
};
pub use qualification_hardening::{
    EstablishmentBlocker, EstablishmentBlockerKind, EstablishmentExplanation,
    HardenedTheoremRegistry, QualificationHardeningError, QualificationLimits,
    QualificationResource,
};
pub use ranging::{RangeEstimate, RangingMethod};
pub use space_navigation::{SpaceNavigationEstimate, SpaceNavigationEstimator};
pub use trilateration::{PositionEstimate, TrilaterationError, trilaterate_2d, trilaterate_3d};
