// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Lease-complete authority current-freshness runtime.
//!
//! Split into include fragments so each security boundary remains reviewable while
//! compiling as one coordinator module.

include!("v04/types.rs");
include!("v04/root.rs");
include!("v04/evidence.rs");
include!("v04/runtime.rs");
