// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix Civic Shared Types & Utilities
//!
//! Common functionality for all domain zomes in the Civic cluster:
//! - Constitutional anti-capture invariants
//! - Capture observation, uncertainty, hypothesis, and signal contracts
//! - Provenance-bearing institutional relationship graph contracts
//! - Deterministic institutional capture metrics
//! - Standards-bound OCDS/BODS ingestion contracts
//! - Reversible public-entity identity reconciliation
//! - Internal qualified equivalence-view projection engine
//! - Trust-rooted public identity qualification boundary
//! - Identity-resolution sensitivity diagnostics
//! - Explicit institutional robustness envelopes
//! - Evidence types shared across justice and media
//! - Status/phase traits for state machine validation
//! - Role-based authorization helpers
//! - Bridge types for cross-domain communication

pub mod anti_capture;
pub mod bridge_types;
pub mod capture_metrics;
pub mod capture_observation;
mod equivalence_view;
pub mod evidence;
pub mod identity_qualification;
mod identity_resolution;
mod identity_sensitivity;
pub mod institutional_graph;
pub mod institutional_robustness;
pub mod qualified_identity_resolution;
pub mod qualified_identity_sensitivity;
pub mod roles;
// The adapter has private serde projection fields that exist to preserve external
// standard structure even when AC-005 deliberately does not expose those fields.
#[allow(dead_code)]
pub mod standards_ingestion;
pub mod status;

pub use anti_capture::*;
pub use bridge_types::*;
pub use capture_metrics::*;
pub use capture_observation::*;
pub use equivalence_view::{
    EquivalenceComponent, EquivalenceViewError, IdentityProjectedObservation,
    QualifiedEquivalenceView,
};
pub use evidence::*;
pub use identity_qualification::*;
pub use identity_resolution::{
    EntityBindingEvidence, EntityIdentityLink, IdentityLinkStatus, IdentityResolutionError,
    IdentityResolutionViolation, IdentityVerification, IdentityVerificationKind,
};
pub use identity_sensitivity::{
    ComponentSensitivity, ExactDiagnosticRatio, IdentityProjectionSensitivityReport,
    IdentitySensitivityError, SupplierMemberCount,
};
pub use institutional_graph::*;
pub use institutional_robustness::*;
pub use qualified_identity_resolution::*;
pub use qualified_identity_sensitivity::*;
pub use roles::*;
pub use standards_ingestion::*;
pub use status::*;
