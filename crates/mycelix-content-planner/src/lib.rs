// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic, authority-free Content Fabric placement proposals.
//!
//! This crate accepts only a CF-06A `PolicyQualifiedPoolV1`. It may rank and
//! recommend candidates, but it cannot weaken hard policy, create leases, move
//! bytes, authorize reads, or perform settlement.

mod model;
mod plan;
mod score;

pub use model::*;
pub use plan::plan_deterministic_v1;
