// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Fabrication Common - Shared Types for Mycelix Fabrication hApp
//!
//! This crate contains all the shared types, enums, and structures used across
//! the Fabrication hApp zomes. It implements the revolutionary features:
//!
//! - HDC-Encoded Parametric Designs (Generative CAD Commons)
//! - Proof of Grounded Fabrication (PoGF) for metabolic accountability
//! - Anticipatory Repair Loop for autopoietic maintenance
//! - Fabrication Process Monitoring (FPM) for deterministic process analysis

use hdi::prelude::*;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};


// =============================================================================
// VALIDATION HELPERS
// =============================================================================

/// Compute the canonical SHA-256 commitment for an FPM verifier public key.
/// The digest covers the exact encoded key bytes supplied to cryptographic verification.
pub fn fpm_verification_key_digest(public_key_sec1: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(public_key_sec1);
    hasher.finalize()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

pub mod fpm_context;
pub mod fpm_registration;
pub mod fpm_qualification;