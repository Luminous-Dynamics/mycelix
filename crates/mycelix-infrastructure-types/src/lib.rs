// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Versioned, transport-neutral infrastructure service envelopes for Mycelix.
//!
//! The lifecycle is intentionally small:
//! `Capability -> Offering -> Lease -> Receipt`.
//!
//! This crate contains no Holochain, marketplace, async-runtime, transport, or
//! billing dependency. It defines commitments and monotonic constraints only.

mod envelope;
mod error;
mod id;
mod resource;

pub use envelope::*;
pub use error::*;
pub use id::*;
pub use resource::*;
