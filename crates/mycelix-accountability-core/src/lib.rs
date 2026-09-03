// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cross-domain reciprocal accountability for the Sovereign Intelligence Fabric.
//!
//! Semantic/privacy validation lives in `protocol`; portable cryptographic
//! commitment encoding lives in `canonical`. Keeping those concerns separate
//! prevents Rust serializer details from becoming part of the public protocol.

mod canonical;
mod protocol;

pub use canonical::{
    ACCOUNTABILITY_CANONICAL_CODEC, AccountabilityCommitmentError, canonical_policy_bytes,
    canonical_pre_attestation_receipt_bytes, policy_commitment,
    pre_attestation_receipt_commitment, purpose_commitment,
};
pub use protocol::*;
