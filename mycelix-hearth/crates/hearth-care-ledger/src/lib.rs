// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure recurring-care ledger semantics for Hearth.
//!
//! A schedule is a durable template. Concrete work is represented by immutable
//! occurrence and completion evidence. No HDK/HDI dependency is used here so
//! zomes, clients, replay qualification, and planners can share one contract.

mod canonical;
mod digest;
mod model;

pub use canonical::*;
pub use digest::*;
pub use model::*;
