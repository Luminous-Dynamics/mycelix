// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! # mycelix-zkp-core
//!
//! Shared authenticated proof envelopes and circuit-specific ZKP primitives.
//!
//! Winterfell and Miden integrations are circuit-specific. The RISC Zero adapter is
//! currently structural-only and must not be treated as an operational verifier.
//! Dilithium5 provides optional post-quantum envelope authentication.
//!
//! Backend availability is not circuit authority. Proof-independent data models
//! such as jurisdiction geometry, registries, and attestation tiers remain usable
//! without enabling a proof backend; quarantined proof lineages remain behind
//! explicit legacy features.
//!
//! ## Feature Flags
//!
//! - `backend-winterfell`: Winterfell backend availability only
//! - `backend-risc0`: structural compatibility adapter; no verifier is linked
//! - `backend-dual`: ordinary backend compatibility feature
//! - `backend-miden`: Miden VM ZK-STARK backend
//! - `dilithium`: CRYSTALS-Dilithium5 PQ signatures
//! - `full`: ordinary backends + Dilithium; excludes quarantined features
//! - `legacy-unqualified-range-proof`: historical range/jurisdiction proof lineage
//! - `legacy-unqualified-winterfell-authority`: other known-underconstrained AIRs
//! - `experimental-winterfell-baselines`: measurement-only Winterfell baselines

pub mod backend;
pub mod circuits;
pub mod consciousness;
#[cfg(feature = "dilithium")]
pub mod dilithium;
pub mod domain;
pub mod error;
pub mod fixed_point;
pub mod jurisdiction_registry;
pub mod jurisdiction_types;
pub mod location_attestation;
#[cfg(feature = "backend-miden")]
pub mod miden_consciousness;
pub mod miden_parameterized;
pub mod pogq;
pub mod sovereign;
pub mod supply;
pub mod supply_verification;
pub mod types;
pub mod validation;

#[cfg(feature = "backend-winterfell")]
pub use winterfell;

// Re-exports
pub use backend::{BackendCapability, ProofBackend, backend_capability, select_backend};
pub use consciousness::{CivicTier, ConsciousnessProofRequest, ConsciousnessProofResult};
#[cfg(feature = "dilithium")]
pub use dilithium::DilithiumKeypair;
pub use domain::DomainTag;
pub use error::{ZkpError, ZkpResult};
pub use fixed_point::{FixedPoint, Q16_16_SCALE};
pub use jurisdiction_types::{COORD_BIAS, JurisdictionBox, MICRODEG_SCALE, biased_microdegrees};
pub use location_attestation::AttestationTier;
pub use pogq::{DualBackendComparison, PoGQPublicInputs, PoGQResult, PoGQWitness, simulate_pogq};
pub use supply::{
    SUPPLY_PROOF_STATEMENT_VERSION, SupplyProofKind, SupplyProofStatement,
    validate_supply_proof_envelope, validate_supply_statement,
};
pub use supply_verification::{
    SUPPLY_VERIFICATION_RECORD_VERSION, SupplyVerificationRecord,
    SupplyVerificationStatus, validate_supply_verification_record,
};
pub use types::{
    AUTHENTICATED_PROOF_PROTOCOL_VERSION, AuthenticatedProof, ProofBytes, ProofMetadata,
    ProofResult, VerificationResult,
};
pub use validation::{EnvelopeValidationPolicy, validate_authenticated_proof_envelope};

// Re-export shared ecosystem types
pub use proofs_commitment::{CommitmentHash, CommitmentScheme, Sha3Commitment};
pub use proofs_config::{ProofConfig, SecurityLevel, UseCase};
