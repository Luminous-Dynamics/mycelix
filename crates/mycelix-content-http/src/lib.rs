// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Loopback-only read HTTP facade for Mycelix Content Fabric.
//!
//! The facade exposes verified immutable CAS reads and operational health only.
//! It deliberately contains no HTTP mutation, deletion, lease, marketplace,
//! planner, Holochain, or transport-discovery authority.

mod error;
mod range;
mod server;

pub use error::HttpFacadeErrorV1;
pub use range::{parse_single_range_v1, ByteRangeV1, RangeErrorV1};
pub use server::{serve, HttpConfigV1};
