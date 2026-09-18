// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Concrete Linux observation collectors for FORGE-004D2B2.
//!
//! This crate performs host I/O and produces the pure evidence objects defined
//! by FORGE-004D2B1. Qualification remains in the evidence crate; collection
//! and policy evaluation are intentionally separate.

mod inside;
mod parent;

pub use inside::collect_inside_evidence;
pub use parent::{observe_parent_start, PendingParentObservation};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum IsolationCollectorError {
    #[error(transparent)]
    Core(#[from] mycelix_forge_core::ForgeCoreError),
    #[error(transparent)]
    Isolation(#[from] mycelix_forge_linux_isolation::IsolationPolicyError),
    #[error(transparent)]
    Evidence(#[from] mycelix_forge_linux_isolation_evidence::IsolationEvidenceError),
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error("non-UTF-8 filesystem path")]
    NonUtf8Path,
    #[error("non-UTF-8 environment entry")]
    NonUtf8Environment,
    #[error("non-UTF-8 bubblewrap status stream")]
    NonUtf8StatusStream,
    #[error("missing /proc status field {0}")]
    MissingProcStatusField(&'static str),
    #[error("malformed /proc status field {0}")]
    MalformedProcStatusField(&'static str),
    #[error("route table is not UTF-8")]
    NonUtf8RouteTable,
    #[error("malformed route table")]
    MalformedRouteTable,
    #[error("bubblewrap status did not contain child-pid")]
    MissingChildPid,
    #[error("invalid bubblewrap child PID {0}")]
    InvalidChildPid(u64),
    #[error("bubblewrap status did not contain exit-code")]
    MissingExitCode,
    #[error("invalid bubblewrap exit code {0}")]
    InvalidExitCode(i64),
    #[error("artifact path is not a regular file: {0:?}")]
    ArtifactNotRegularFile(std::path::PathBuf),
    #[error("artifact byte length overflow")]
    ArtifactTooLarge,
}
