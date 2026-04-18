// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Proof Circuits
//!
//! zkSTARK proof circuits for federated learning verification.
//!
//! ## Available Circuits
//!
//! - [`RangeProof`] - Prove a value is within bounds
//! - [`MembershipProof`] - Prove element exists in Merkle tree
//! - [`GradientIntegrityProof`] - Prove valid FL contribution
//! - [`IdentityAssuranceProof`] - Prove identity meets assurance threshold
//! - [`VoteEligibilityProof`] - Prove voter qualification

pub mod range;
pub mod membership;
pub mod gradient;
pub mod identity;
pub mod vote;

pub use range::{RangeProof, RangeProofAir, RangePublicInputs};
pub use membership::{
    MembershipProof, MembershipProofAir, MembershipPublicInputs,
    MerklePathNode, build_merkle_tree, compute_leaf_hash,
};
pub use gradient::{
    GradientIntegrityProof, GradientIntegrityAir, GradientPublicInputs,
    compute_gradient_commitment,
};
pub use identity::{
    IdentityAssuranceProof, IdentityAssuranceAir, IdentityPublicInputs,
    ProofAssuranceLevel, ProofIdentityFactor, compute_identity_commitment,
};
pub use vote::{
    VoteEligibilityProof, VoteEligibilityAir, VotePublicInputs,
    ProofProposalType, ProofEligibilityRequirements, ProofVoterProfile,
    compute_voter_commitment,
};
