// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! API request handlers

pub mod authority_epoch;
pub mod claims;
pub mod credential_governance;
pub mod credentials;
pub mod query;
pub mod system;
pub mod trust;
pub mod zkp_review;

pub use authority_epoch::*;
pub use claims::*;
pub use credential_governance::*;
pub use credentials::*;
pub use query::*;
pub use system::*;
pub use trust::*;
pub use zkp_review::*;

pub mod scientific;
pub use scientific::*;
