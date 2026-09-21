// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # Praxis Core
//!
//! Core types, wire contracts, cryptographic helpers, and provenance utilities
//! for Mycelix Praxis.
//!
//! This crate provides:
//! - Common data structures used across zomes
//! - Cryptographic primitives (hashing, signatures)
//! - Provenance tracking for models and credentials
//! - Authority-safe learning evidence and advisory capability-estimate contracts
//! - Provenance-complete attempt observations with explicit legacy incompleteness
//! - Learner-authored goal intent separated from evidence-derived progress projections
//! - Expiring recommendations bound to exact goal/evidence/projection dependencies
//! - Versioned adaptive-path plans separated from evidence-derived execution progress
//! - Descriptive session summaries and evidence-bound analytics projections
//! - Validation utilities
//! - Experimental Proof of Learning (PoL) analytics; PoL output is not, by itself,
//!   proof of mastery, cheating, identity, credential eligibility, or authorization
//! - Structured error handling with descriptive messages

mod benchmarks;
pub mod adaptive_path_state;
pub mod analytics_state;
pub mod attempt_evidence;
pub mod contracts;
pub mod crypto;
pub mod errors;
pub mod export_formats;
pub mod goal_state;
pub mod learning_evidence;
pub mod proof_of_learning;
pub mod provenance;
pub mod recommendation_state;
pub mod types;
pub mod validation;

pub use adaptive_path_state::*;
pub use analytics_state::*;
pub use attempt_evidence::*;
pub use contracts::*;
pub use crypto::*;
pub use errors::*;
pub use goal_state::*;
pub use learning_evidence::*;
pub use proof_of_learning::*;
pub use provenance::*;
pub use recommendation_state::*;
pub use types::*;

/// Current protocol version
pub const PROTOCOL_VERSION: &str = "0.1.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_version() {
        assert_eq!(PROTOCOL_VERSION, "0.1.0");
    }
}
