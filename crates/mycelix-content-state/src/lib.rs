// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Deterministic projection of Content Fabric evidence snapshots.
//!
//! The crate is intentionally pure: no Holochain calls, no network, no system
//! clock, no CAS mutation, and no planner authority.

mod model;
mod projection;

pub use model::*;
pub use projection::project_content_state_v1;
