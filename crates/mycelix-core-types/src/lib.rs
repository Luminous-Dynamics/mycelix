// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix Core Types
//!
//! This crate provides the fundamental types used throughout the Mycelix ecosystem.
//!
//! Relationship OS primitives are intentionally vertical-neutral: relationship identity,
//! commitments, and consent are typed separately so that evidence, authority, and agent
//! reasoning cannot silently collapse into one permission or confidence value.

pub mod collective_field;
pub mod commitment;
pub mod consent;
pub mod epistemic;
pub mod harmonic;
pub mod k_vector;
pub mod moral;
pub mod relationship;
pub mod trust;
pub mod wisdom_engine;

pub use collective_field::*;
pub use commitment::*;
pub use consent::*;
pub use epistemic::*;
pub use harmonic::*;
pub use k_vector::*;
pub use moral::*;
pub use relationship::*;
pub use trust::*;
pub use wisdom_engine::*;

/// Mycelix protocol version
pub const PROTOCOL_VERSION: &str = "2.0";

/// Maximum Byzantine tolerance (45% of participants)
/// From MATL specification - system cannot guarantee safety beyond this
pub const MAX_BYZANTINE_TOLERANCE: f32 = 0.45;

/// Phi threshold levels for governance
pub mod phi_thresholds {
    /// Basic proposals (routine decisions)
    pub const BASIC: f32 = 0.3;
    /// Major proposals (significant changes)
    pub const MAJOR: f32 = 0.4;
    /// Constitutional proposals (fundamental changes)
    pub const CONSTITUTIONAL: f32 = 0.6;
}

// =============================================================================
// Python Bindings (PyO3)
// =============================================================================

#[cfg(feature = "python")]
use pyo3::prelude::*;

/// Python module entry point for mycelix_wisdom
#[cfg(feature = "python")]
#[pymodule]
fn mycelix_wisdom(m: &Bound<'_, PyModule>) -> PyResult<()> {
    wisdom_engine::python_bindings::register_wisdom_classes(m)?;

    m.add(
        "__doc__",
        "Mycelix WisdomEngine - Pattern intelligence for AI systems",
    )?;
    m.add("__version__", PROTOCOL_VERSION)?;

    Ok(())
}
