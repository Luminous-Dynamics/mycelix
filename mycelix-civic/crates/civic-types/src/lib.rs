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
//! - Evidence types shared across justice and media
//! - Status/phase traits for state machine validation
//! - Role-based authorization helpers
//! - Bridge types for cross-domain communication

pub mod anti_capture;
pub mod bridge_types;
pub mod capture_metrics;
pub mod capture_observation;
pub mod evidence;
mod identity_resolution;
pub mod institutional_graph;
pub mod qualified_identity_resolution;
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
pub use evidence::*;
pub use identity_resolution::{
    EntityBindingEvidence, EntityIdentityLink, IdentityLinkStatus, IdentityResolutionError,
    IdentityResolutionViolation, IdentityVerification, IdentityVerificationKind,
};
pub use institutional_graph::*;
pub use qualified_identity_resolution::*;
pub use roles::*;
pub use standards_ingestion::*;
pub use status::*;
