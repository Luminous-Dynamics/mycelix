// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Focused test harness for the pure food-lot reconciliation source.
//!
//! This compiles the exact production source file and its in-module unit tests.
//! It does not compile the Holochain zome, coordinator, storage adapter or DHT runtime.

#[path = "../../../mycelix-commons/zomes/food-distribution/integrity/src/food_lot_reconciliation.rs"]
pub mod food_lot_reconciliation;
