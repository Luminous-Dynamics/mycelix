// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed hard-policy qualification for Content Fabric placement candidates.
//!
//! This crate deliberately contains no optimizer, Symthaea, Holochain, Iroh,
//! filesystem, marketplace, payment, or execution dependency. It only removes
//! candidates that cannot satisfy hard policy and validates selected subsets.

mod gate;
mod model;

pub use gate::evaluate_hard_policy_v1;
pub use model::*;
