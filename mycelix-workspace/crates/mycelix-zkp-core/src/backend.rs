// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Proof backend interfaces and explicit capability reporting.
//!
//! Ported from the Python `ZKBackend` ABC in
//! `mycelix-core/0TML/tests/integration/zkbackend_abstraction.py` (895 lines).
//!
//! Backend-family availability is deliberately separate from operational proof
//! readiness. A linked proof library may be usable by circuit-specific code while
//! the shared generic adapter remains unable to prove or verify any statement.

use crate::error::ZkpResult;
use crate::types::{BackendId, ProofResult, VerificationResult};

/// Public inputs for a ZK proof circuit.
///
/// Each circuit defines its own public input structure, but they all serialize
/// through this common wrapper for the backend interface.
#[derive(Clone, Debug)]
pub struct PublicInputs {
    /// Serialized public inputs (circuit-specific format).
    pub data: Vec<u8>,
    /// Human-readable description for logging.
    pub description: String,
}

impl PublicInputs {
    pub fn new(data: Vec<u8>, description: impl Into<String>) -> Self {
        Self {
            data,
            description: description.into(),
        }
    }

    pub fn from_value<T: serde::Serialize>(
        value: &T,
        description: impl Into<String>,
    ) -> ZkpResult<Self> {
        let data = serde_json::to_vec(value)
            .map_err(|e: serde_json::Error| crate::error::ZkpError::Serialization(e.to_string()))?;
        Ok(Self::new(data, description))
    }
}

/// Unified interface for operational ZKP adapters.
///
/// `is_available() == true` means this adapter itself can perform its advertised
/// prove/verify operations in the current build. It must not be used merely to
/// indicate that a backend dependency or backend family is linked.
pub trait ProofBackend: Send + Sync {
    fn name(&self) -> &str;

    fn version(&self) -> &str;

    fn id(&self) -> BackendId;

    /// Whether this concrete adapter can perform its advertised proof operations.
    fn is_available(&self) -> bool;

    fn prove(&self, public_inputs: &PublicInputs, witness: &[f32]) -> ZkpResult<ProofResult>;

    fn verify(
        &self,
        proof_bytes: &[u8],
        public_inputs: &PublicInputs,
    ) -> ZkpResult<VerificationResult>;
}

/// Winterfell backend-family adapter.
///
/// The Winterfell libraries are linked when `backend-winterfell` is enabled, but
/// this generic adapter has no AIR/statement identity and therefore cannot prove
/// or verify by itself. Circuit-specific implementations call Winterfell directly
/// with their exact AIR and public-input theorem.
#[cfg(feature = "backend-winterfell")]
pub mod winterfell_backend {
    use super::*;

    pub struct WinterfellBackend {
        version: String,
    }

    impl WinterfellBackend {
        pub fn new() -> Self {
            Self {
                version: "0.13.1".to_string(),
            }
        }
    }

    impl Default for WinterfellBackend {
        fn default() -> Self {
            Self::new()
        }
    }

    impl ProofBackend for WinterfellBackend {
        fn name(&self) -> &str {
            "Winterfell STARK"
        }

        fn version(&self) -> &str {
            &self.version
        }

        fn id(&self) -> BackendId {
            BackendId::Winterfell
        }

        fn is_available(&self) -> bool {
            false
        }

        fn prove(&self, _public_inputs: &PublicInputs, _witness: &[f32]) -> ZkpResult<ProofResult> {
            Err(crate::error::ZkpError::ProvingError(
                "generic Winterfell adapter has no circuit theorem; use a circuit-specific prover"
                    .into(),
            ))
        }

        fn verify(
            &self,
            _proof_bytes: &[u8],
            _public_inputs: &PublicInputs,
        ) -> ZkpResult<VerificationResult> {
            Err(crate::error::ZkpError::VerificationFailed(
                "generic Winterfell adapter has no circuit theorem; use a circuit-specific verifier"
                    .into(),
            ))
        }
    }
}

/// RISC Zero backend-family compatibility adapter.
///
/// No RISC Zero dependency/verifier is linked by the current feature, so this
/// adapter is structural-only and fail-closed.
#[cfg(feature = "backend-risc0")]
pub mod risc0_backend {
    use super::*;

    pub struct Risc0Backend {
        version: String,
    }

    impl Risc0Backend {
        pub fn new() -> Self {
            Self {
                version: "3.0.4".to_string(),
            }
        }
    }

    impl Default for Risc0Backend {
        fn default() -> Self {
            Self::new()
        }
    }

    impl ProofBackend for Risc0Backend {
        fn name(&self) -> &str {
            "RISC Zero zkVM"
        }

        fn version(&self) -> &str {
            &self.version
        }

        fn id(&self) -> BackendId {
            BackendId::Risc0
        }

        fn is_available(&self) -> bool {
            false
        }

        fn prove(&self, _public_inputs: &PublicInputs, _witness: &[f32]) -> ZkpResult<ProofResult> {
            Err(crate::error::ZkpError::ProvingError(
                "Use a circuit-specific prover with a RISC0 guest image".into(),
            ))
        }

        fn verify(
            &self,
            _proof_bytes: &[u8],
            _public_inputs: &PublicInputs,
        ) -> ZkpResult<VerificationResult> {
            Err(crate::error::ZkpError::VerificationFailed(
                "Use a circuit-specific verifier with the expected RISC0 image ID".into(),
            ))
        }
    }
}

/// Capability of a backend family in this shared crate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BackendCapability {
    /// Dependencies/support exist for circuit-specific implementations, but no
    /// generic theorem-independent prover/verifier is implied.
    CircuitSpecific,
    /// The public compatibility adapter exists, but no verifier is linked here.
    StructuralOnly,
    /// Identifier reserved for future protocol compatibility.
    Reserved,
}

/// Report backend-family capability without conflating it with operational proof
/// readiness or circuit qualification.
pub const fn backend_capability(backend: BackendId) -> BackendCapability {
    match backend {
        BackendId::Winterfell | BackendId::Miden => BackendCapability::CircuitSpecific,
        BackendId::Risc0 => BackendCapability::StructuralOnly,
        BackendId::Binius => BackendCapability::Reserved,
    }
}

/// Select a candidate backend **family** for a broad complexity hint.
///
/// This says only which linked family a caller might investigate. It does not
/// establish that a circuit implementation exists, that the generic adapter is
/// operational, or that any theorem is qualified.
pub fn select_backend_family(complexity: CircuitComplexity) -> Option<BackendId> {
    match complexity {
        CircuitComplexity::Simple => {
            #[cfg(feature = "backend-winterfell")]
            return Some(BackendId::Winterfell);
            #[cfg(all(not(feature = "backend-winterfell"), feature = "backend-miden"))]
            return Some(BackendId::Miden);
            #[cfg(all(
                not(feature = "backend-winterfell"),
                not(feature = "backend-miden")
            ))]
            return None;
        }
        CircuitComplexity::Complex => {
            #[cfg(feature = "backend-miden")]
            return Some(BackendId::Miden);
            #[cfg(all(not(feature = "backend-miden"), feature = "backend-winterfell"))]
            return Some(BackendId::Winterfell);
            #[cfg(all(
                not(feature = "backend-miden"),
                not(feature = "backend-winterfell")
            ))]
            return None;
        }
    }
}

/// Backward-compatible backend-family selector.
///
/// This historical name does **not** mean an operational generic prover/verifier
/// has been selected. New callers should use [`select_backend_family`] and then
/// resolve an exact circuit/profile through an independently qualified registry.
#[deprecated(
    since = "0.1.0",
    note = "selects a candidate backend family only; use select_backend_family and an exact qualified circuit/profile"
)]
pub fn select_backend(complexity: CircuitComplexity) -> Option<BackendId> {
    select_backend_family(complexity)
}

/// Coarse backend-family selection hint only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CircuitComplexity {
    Simple,
    Complex,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_public_inputs_creation() {
        let pi = PublicInputs::new(vec![1, 2, 3], "test inputs");
        assert_eq!(pi.data, vec![1, 2, 3]);
        assert_eq!(pi.description, "test inputs");
    }

    #[test]
    fn test_public_inputs_from_value() {
        let value = serde_json::json!({"threshold": 100, "round": 5});
        let pi = PublicInputs::from_value(&value, "json inputs").unwrap();
        assert!(!pi.data.is_empty());
    }

    #[test]
    fn test_select_backend_family_simple() {
        let backend = select_backend_family(CircuitComplexity::Simple);
        #[cfg(feature = "backend-winterfell")]
        assert_eq!(backend, Some(BackendId::Winterfell));
        #[cfg(all(not(feature = "backend-winterfell"), feature = "backend-miden"))]
        assert_eq!(backend, Some(BackendId::Miden));
        #[cfg(all(
            not(feature = "backend-winterfell"),
            not(feature = "backend-miden")
        ))]
        assert_eq!(backend, None);
    }

    #[test]
    fn test_select_backend_family_complex() {
        let backend = select_backend_family(CircuitComplexity::Complex);
        #[cfg(feature = "backend-miden")]
        assert_eq!(backend, Some(BackendId::Miden));
        #[cfg(all(not(feature = "backend-miden"), feature = "backend-winterfell"))]
        assert_eq!(backend, Some(BackendId::Winterfell));
        #[cfg(all(
            not(feature = "backend-miden"),
            not(feature = "backend-winterfell")
        ))]
        assert_eq!(backend, None);
    }

    #[allow(deprecated)]
    #[test]
    fn compatibility_selector_is_family_selector_only() {
        assert_eq!(
            select_backend(CircuitComplexity::Simple),
            select_backend_family(CircuitComplexity::Simple)
        );
    }

    #[test]
    fn test_backend_capabilities_are_truthful() {
        assert_eq!(
            backend_capability(BackendId::Winterfell),
            BackendCapability::CircuitSpecific
        );
        assert_eq!(
            backend_capability(BackendId::Risc0),
            BackendCapability::StructuralOnly
        );
        assert_eq!(
            backend_capability(BackendId::Miden),
            BackendCapability::CircuitSpecific
        );
        assert_eq!(
            backend_capability(BackendId::Binius),
            BackendCapability::Reserved
        );
    }

    #[cfg(feature = "backend-winterfell")]
    #[test]
    fn generic_winterfell_adapter_is_not_operational() {
        let b = winterfell_backend::WinterfellBackend::new();
        assert!(!b.is_available());
        assert_eq!(b.id(), BackendId::Winterfell);
        assert_eq!(backend_capability(b.id()), BackendCapability::CircuitSpecific);
        assert!(b.prove(&PublicInputs::new(vec![], "test"), &[]).is_err());
        assert!(b.verify(&[], &PublicInputs::new(vec![], "test")).is_err());
    }

    #[cfg(feature = "backend-risc0")]
    #[test]
    fn generic_risc0_adapter_is_not_operational() {
        let b = risc0_backend::Risc0Backend::new();
        assert!(!b.is_available());
        assert_eq!(b.id(), BackendId::Risc0);
        assert_eq!(backend_capability(b.id()), BackendCapability::StructuralOnly);
    }
}
