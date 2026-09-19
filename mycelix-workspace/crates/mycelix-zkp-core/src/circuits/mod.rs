// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Shared ZKP circuits for the Mycelix ecosystem.
//!
//! Winterfell backend availability is not proof authority. Application-facing
//! AIRs with demonstrated theorem gaps are available only through explicit
//! legacy/unqualified features; measurement-only circuits are separately marked
//! experimental. See issues #227 and #1873.

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

#[cfg(feature = "legacy-unqualified-winterfell-authority")]
#[deprecated(
    note = "UNQUALIFIED: ReviewIntegrityAir does not bind its documented public-input/COI theorem (issue #1873)"
)]
pub mod review_integrity;

#[cfg(feature = "legacy-unqualified-winterfell-authority")]
#[deprecated(
    note = "UNQUALIFIED: RecursiveAggregationAir leaves documented aggregation/budget relations unconstrained (issue #1873)"
)]
pub mod recursive_aggregation;

pub mod merkle_membership;
pub mod nullifier;

#[cfg(feature = "experimental-winterfell-baselines")]
#[deprecated(
    note = "EXPERIMENTAL ONLY: benchmark depends on quarantined historical range AIR (issues #227/#1873)"
)]
pub mod winterfell_bench;

#[cfg(feature = "experimental-winterfell-baselines")]
#[deprecated(
    note = "EXPERIMENTAL ONLY: PrimeFieldXorAir is an unqualified measurement baseline (issue #1873)"
)]
pub mod winterfell_xor;
