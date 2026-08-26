// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Mycelix Civic Shared Types & Utilities
//!
//! Common functionality for all domain zomes in the Civic cluster:
//! - Evidence types shared across justice and media
//! - Status/phase traits for state machine validation
//! - Role-based authorization helpers
//! - Persistent delegated authority primitives
//! - Administrative record classification and retention primitives
//! - Bridge types for cross-domain communication

pub mod authority;
pub mod bridge_types;
pub mod evidence;
pub mod records;
pub mod roles;
pub mod status;

pub use authority::*;
pub use bridge_types::*;
pub use evidence::*;
pub use records::*;
pub use roles::*;
pub use status::*;
