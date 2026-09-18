// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable, forge-independent authority primitives for Mycelix Forge.
//!
//! This crate deliberately separates the small project-identity/digest kernel
//! (`mycelix-forge-core`) from repository authority state. It does not verify
//! signatures, authenticate principals, talk to a forge, or persist authority.
//! Adapters must prove those facts separately.

pub use mycelix_forge_core::{
    Digest, DigestAlgorithm, ForgeCoreError, ProjectIdentity, ProjectIdentitySeed,
    ProtocolVersion, CURRENT_PROTOCOL_VERSION, GENESIS_NONCE_LEN,
};

mod authority;
mod quorum;
mod root_policy;

pub use authority::{
    AuthorityEpoch, AuthorityEpochParts, AuthorityError, Capability, CapabilityRule, PrincipalGrant,
    PrincipalId,
};
pub use quorum::{evaluate_structural_quorum, QuorumError, StructuralQuorum};
pub use root_policy::{
    BoundGenesisAuthority, GenesisBindingError, RootAuthorityPolicy,
};
