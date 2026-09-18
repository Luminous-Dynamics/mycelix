// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Portable repository-verification contracts and evidence bindings for Mycelix Forge.
//!
//! Canonical repository subjects live in the private `contract` module. The
//! public qualification entry point is the evidence-backed layer so callers
//! cannot accidentally collapse a declared monotonic-policy capability into a
//! bare adapter assertion with no lineage commitment.

mod contract;
mod evidence;

pub use contract::{
    AdapterIdentity, AdapterObservation, AdapterOutcome, GitObjectAlgorithm, GitObjectId,
    RepositoryAdoption, RepositoryPolicyState, RepositoryRef, RepositoryTip,
    RepositoryVerificationError, RepositoryVerificationRequest,
    StructurallyQualifiedRepositoryVerification, VerificationCapability, VerificationProfile,
};
pub use evidence::{
    qualify_evidence_backed_observation, BoundRepositoryPolicyGenesis,
    EvidenceBackedAdapterObservation, QualifiedRepositoryVerification, RepositoryEvidenceError,
};
