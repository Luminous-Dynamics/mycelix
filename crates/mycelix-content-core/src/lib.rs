// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Pure Content Fabric vocabulary.
//!
//! This crate separates immutable byte identity, object composition,
//! publication/provenance, and storage intent. It deliberately contains no
//! Holochain, transport, async runtime, marketplace, or planner dependency.

mod blob;
mod canonical;
mod digest;
mod encryption;
mod error;
mod failure_domain;
mod manifest;
mod publication;
mod storage_policy;

pub use blob::*;
pub use digest::*;
pub use encryption::*;
pub use error::*;
pub use failure_domain::*;
pub use manifest::*;
pub use publication::*;
pub use storage_policy::*;
