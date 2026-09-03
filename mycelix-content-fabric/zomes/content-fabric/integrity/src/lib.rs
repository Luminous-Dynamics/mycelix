// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Integrity definitions for the Mycelix Content Fabric coordination DNA.
//! Bulk content never enters this DHT. Entries are append-only claims/evidence.

mod canonical;
mod types;
mod validation;

pub use canonical::*;
pub use types::*;

use hdi::prelude::*;

#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    validation::validate_content_fabric_op(op)
}
