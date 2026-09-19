// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Shared ZKP circuits for the Mycelix ecosystem.
//!
//! SECURITY: the historical `range_proof` / `HealthRangeAir` circuit is
//! underconstrained and tracked by issue #227. It is excluded from ordinary
//! `backend-winterfell` builds and may only be compiled with the explicit
//! `legacy-unqualified-range-proof` feature for historical/unqualified evidence
//! inspection. It must not be used as authority for Health, Finance, FL, or any
//! other range claim until a replacement theorem is adversarially qualified.
//!
//! `jurisdiction_proof` composes two instances of that same range circuit and is
//! therefore quarantined under the same explicit legacy feature. A host-side
//! jurisdiction commitment does not repair the underlying range theorem.
//!
//! `range_membership_v1` is the replacement **candidate** defined by
//! MYC-ZKP-RANGE-001R. It is feature-gated separately from backend availability
//! and from `full` until exact-head adversarial qualification executes. Its
//! Winterfell 0.13.1 profile makes no witness-privacy / zero-knowledge claim;
//! see issue #1899 and `ZKP_RANGE_WITNESS_PRIVACY_BOUNDARY.md`.

#[cfg(feature = "legacy-unqualified-range-proof")]
#[deprecated(
    note = "UNQUALIFIED: HealthRangeAir is underconstrained (issue #227); historical inspection only"
)]
pub mod range_proof;

#[cfg(feature = "legacy-unqualified-range-proof")]
#[deprecated(
    note = "UNQUALIFIED: jurisdiction proof depends on quarantined HealthRangeAir (issue #227)"
)]
pub mod jurisdiction_proof;

#[cfg(feature = "candidate-range-membership-v1")]
pub mod range_membership_v1;

#[cfg(feature = "backend-winterfell")]
pub mod review_integrity;

#[cfg(feature = "backend-winterfell")]
pub mod recursive_aggregation;

pub mod merkle_membership;
pub mod nullifier;

#[cfg(feature = "backend-winterfell")]
pub mod winterfell_bench;

#[cfg(feature = "backend-winterfell")]
pub mod winterfell_xor;
