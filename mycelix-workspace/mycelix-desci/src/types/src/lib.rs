// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Core epistemic claim types, error types, and the `StorageBackend` trait
//! for Mycelix-DeSci.
//!
//! Extracted out of `mycelix-desci-core` 2026-08-05, verbatim, specifically
//! to be dependency-light: no `sqlx`, no Holochain. `mycelix-desci-core`
//! needs `sqlx` (Postgres authority backend); the Holochain DHT storage
//! backend (`mycelix-desci-holochain-bridge`) needs `holochain_types`,
//! whose `rusqlite` dependency declares `links = "sqlite3"` and is
//! unconditionally incompatible -- in any single Cargo.lock, regardless of
//! feature gating -- with `sqlx`'s optional `sqlx-sqlite` dependency
//! (`sqlx-sqlite` needs `libsqlite3-sys ^0.28`, `rusqlite` 0.37 needs
//! `^0.35`). This crate is the shared, conflict-free base both depend on
//! instead of one depending on the other.

pub mod claims;
pub mod error;
pub mod storage;

// Core claim types
pub use claims::{
    ClaimContent, DesciClaim, EpistemicPosition, EpistemicTier, Provenance, Verification,
};

// Layer 1: LEM Cube (Charter v2.0)
pub use claims::{EmpiricalAxis, LEMCube, MaterialityAxis, NormativeAxis};

// Layer 3: Quality Metrics
pub use claims::QualityMetrics;

// Layer 4: Network Position
pub use claims::{ClaimRelation, ClaimRelationType, NetworkPosition};

// MATL Integration
pub use claims::MATLTrust;

// Unified Fingerprint
pub use claims::EpistemicFingerprint;

pub use error::{Error, Result};
pub use storage::{IpfsStorage, MemoryStorage, StorageBackend};
