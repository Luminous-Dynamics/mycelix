// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed remote exposure admission for Content Fabric Nix cache entries.
//!
//! This crate does not listen on the network and does not mint authorization
//! evidence. It consumes a local CF-07 exposure catalog plus externally sourced
//! grant/revocation evidence and deterministically projects the subset that may
//! be considered by a future non-loopback adapter.

mod model;
mod project;
mod serving;

pub use model::*;
pub use project::project_remote_exposure_v1;
pub use serving::{
    project_remote_serving_snapshot_v1, RemoteServingProjectionErrorV1,
    RemoteServingSnapshotErrorV1, RemoteServingSnapshotV1,
};
