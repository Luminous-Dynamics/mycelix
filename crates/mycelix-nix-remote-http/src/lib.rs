// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Time-bounded public remote Nix HTTP routing over CF-07A serving snapshots.
//!
//! CF-07B deliberately returns only an Axum router. It does not bind a socket,
//! authenticate principals/groups, mint exposure authority, or accept a raw
//! CF-07 catalog. Public requests are evaluated only as anonymous readers.

mod clock;
mod error;
mod server;

pub use clock::{RemoteClockErrorV1, RemoteClockV1, SystemUnixClockV1};
pub use error::{RemoteNixRouterConfigV1, RemoteNixRouterErrorV1};
pub use server::public_router;
