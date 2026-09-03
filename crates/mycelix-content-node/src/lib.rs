// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

#![cfg_attr(test, allow(clippy::permissions_set_readonly_false))]

//! Crash-safe local CAS for Mycelix Content Fabric.
//!
//! This crate owns filesystem truth only. It deliberately does not expose HTTP,
//! Holochain, Iroh, marketplace, planner, or deletion authority.

mod error;
mod hasher;
mod store;

pub use error::CasErrorV1;
pub use store::{
    AuditReportV1, CapacityV1, CasConfigV1, LocalCasV1, PutOutcomeV1, VerifiedBlobV1,
};
