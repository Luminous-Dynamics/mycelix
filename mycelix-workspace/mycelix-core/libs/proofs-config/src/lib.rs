// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Centralized configuration for Mycelix ZK proof systems.
//!
//! This crate provides shared policy tiers and proof configuration. Policy tiers
//! express deployment intent; they are not, by themselves, measured cryptographic
//! security or proof qualification. Exact measured evidence lives in
//! [`security_evidence`].
//!
//! # Design Philosophy
//!
//! Rather than having proof policy scattered across multiple crates, this crate
//! centralizes compatibility-level configuration while keeping measured proof
//! security, theorem qualification, witness privacy, and application authority as
//! separate evidence layers.
//!
//! # Example
//!
//! ```
//! use proofs_config::{SecurityLevel, ProofConfig};
//!
//! // Policy-approved production configuration. This does not itself establish
//! // measured proof security or theorem qualification.
//! let config = ProofConfig::production();
//! assert!(config.security_level.is_policy_approved_for_production());
//!
//! // Get policy configuration for a specific use case.
//! let config = ProofConfig::for_use_case(proofs_config::UseCase::ConstitutionalVote);
//! assert_eq!(config.security_level, SecurityLevel::High);
//! ```

use serde::{Deserialize, Serialize};

pub mod security_evidence;
pub use security_evidence::{
    evaluate_security_target_v1, MeasuredProofSecurityV1, ProofSecurityProfileIdentityV1,
    ProofSecurityTargetV1, SecurityTargetEvaluationV1, SecurityTargetFailureV1,
};

/// Policy security tier for ZK-proof deployments.
///
/// These variants are retained as stable ecosystem/wire vocabulary. They express
/// increasing policy intent and select historical parameter presets; they are not
/// backend-independent measurements of a concrete proof's cryptographic security.
///
/// The numeric targets below are compatibility policy targets. Exact proof
/// security must be reconstructed from the concrete backend/hash/field/options/
/// statement profile and retained measured evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum SecurityLevel {
    /// Testing/development policy tier.
    Fast = 0,

    /// Internal/low-stakes policy tier.
    Optimized = 1,

    /// General production policy tier.
    Standard = 2,

    /// Critical-operation policy tier.
    ///
    /// This variant does **not** mean 264 measured security bits. Concrete proof
    /// evidence remains subject to hash/field/protocol ceilings.
    High = 3,
}

impl Default for SecurityLevel {
    fn default() -> Self {
        SecurityLevel::Standard
    }
}

impl SecurityLevel {
    /// Compatibility policy target in bits.
    ///
    /// This value is **not measured proof security**. It is an intent threshold
    /// used by legacy policy/configuration APIs. Concrete proofs must be evaluated
    /// with [`MeasuredProofSecurityV1`] against an explicit target.
    ///
    /// `High` is capped at 128 here rather than preserving the historical 264-bit
    /// generic claim, which could exceed known hash-collision ceilings.
    pub const fn policy_target_bits(&self) -> u32 {
        match self {
            SecurityLevel::Fast => 40,
            SecurityLevel::Optimized => 84,
            SecurityLevel::Standard => 96,
            SecurityLevel::High => 128,
        }
    }

    /// Backward-compatible alias for [`Self::policy_target_bits`].
    ///
    /// Despite the historical name, this does not establish measured
    /// cryptographic security for a concrete proof.
    pub const fn estimated_security_bits(&self) -> u32 {
        self.policy_target_bits()
    }

    /// Check whether this policy tier's compatibility target meets `min_bits`.
    ///
    /// This is a policy comparison, not proof-security qualification.
    pub const fn meets_minimum(&self, min_bits: u32) -> bool {
        self.policy_target_bits() >= min_bits
    }

    /// Whether policy permits this tier for a production deployment.
    ///
    /// This does not establish that any concrete proof is production-safe.
    pub const fn is_policy_approved_for_production(&self) -> bool {
        matches!(self, SecurityLevel::Standard | SecurityLevel::High)
    }

    /// Backward-compatible policy-only alias.
    ///
    /// Historical callers should migrate to [`Self::is_policy_approved_for_production`]
    /// and use measured evidence for cryptographic admission.
    pub const fn is_production_safe(&self) -> bool {
        self.is_policy_approved_for_production()
    }

    /// Check if this tier is testing-only.
    pub const fn is_testing_only(&self) -> bool {
        matches!(self, SecurityLevel::Fast)
    }

    /// Get the minimum policy tier whose compatibility target reaches `bits`.
    ///
    /// This does not say that a concrete proof at that tier has measured security
    /// equal to the returned target.
    pub const fn minimum_level_for_bits(bits: u32) -> Option<Self> {
        if bits <= 40 {
            Some(SecurityLevel::Fast)
        } else if bits <= 84 {
            Some(SecurityLevel::Optimized)
        } else if bits <= 96 {
            Some(SecurityLevel::Standard)
        } else if bits <= 128 {
            Some(SecurityLevel::High)
        } else {
            None
        }
    }

    /// Get the recommended policy tier for general production use.
    pub const fn production_default() -> Self {
        SecurityLevel::Standard
    }

    /// Get the strongest compatibility policy tier.
    pub const fn maximum() -> Self {
        SecurityLevel::High
    }

    /// Human-readable policy description.
    pub const fn description(&self) -> &'static str {
        match self {
            SecurityLevel::Fast => "Testing-only policy tier (40-bit target)",
            SecurityLevel::Optimized => "Internal policy tier (84-bit target)",
            SecurityLevel::Standard => "General production policy tier (96-bit target)",
            SecurityLevel::High => "Critical-operation policy tier (128-bit target ceiling)",
        }
    }
}

/// Use cases for ZK proofs in the Mycelix ecosystem.
///
/// Each use case maps to a policy tier based on sensitivity. This mapping does
/// not replace exact backend/profile measurement or theorem qualification.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum UseCase {
    /// Development and testing
    Testing,

    /// K-Vector trust metric proofs (standard operations)
    KVectorProof,

    /// Standard governance voting
    StandardVote,

    /// Treasury-related proposals
    TreasuryVote,

    /// Constitutional amendments
    ConstitutionalVote,

    /// Model governance decisions
    ModelGovernance,

    /// Emergency proposals
    EmergencyProposal,

    /// Identity verification
    IdentityVerification,

    /// Federated learning contribution proofs
    FederatedLearning,

    /// Cross-chain bridge operations
    CrossChainBridge,
}

impl UseCase {
    /// Get the recommended policy tier for this use case.
    pub const fn recommended_security_level(&self) -> SecurityLevel {
        match self {
            UseCase::Testing => SecurityLevel::Fast,
            UseCase::KVectorProof => SecurityLevel::Standard,
            UseCase::StandardVote => SecurityLevel::Standard,
            UseCase::TreasuryVote => SecurityLevel::High,
            UseCase::ConstitutionalVote => SecurityLevel::High,
            UseCase::ModelGovernance => SecurityLevel::Standard,
            UseCase::EmergencyProposal => SecurityLevel::Standard,
            UseCase::IdentityVerification => SecurityLevel::Standard,
            UseCase::FederatedLearning => SecurityLevel::Standard,
            UseCase::CrossChainBridge => SecurityLevel::High,
        }
    }

    /// Get the minimum policy tier accepted for this use case.
    pub const fn minimum_security_level(&self) -> SecurityLevel {
        match self {
            UseCase::Testing => SecurityLevel::Fast,
            UseCase::KVectorProof => SecurityLevel::Optimized,
            UseCase::StandardVote => SecurityLevel::Standard,
            UseCase::TreasuryVote => SecurityLevel::Standard,
            UseCase::ConstitutionalVote => SecurityLevel::High,
            UseCase::ModelGovernance => SecurityLevel::Optimized,
            UseCase::EmergencyProposal => SecurityLevel::Standard,
            UseCase::IdentityVerification => SecurityLevel::Standard,
            UseCase::FederatedLearning => SecurityLevel::Optimized,
            UseCase::CrossChainBridge => SecurityLevel::Standard,
        }
    }
}

/// Configuration for ZK proof generation and verification policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProofConfig {
    /// Policy tier for the proof.
    pub security_level: SecurityLevel,

    /// Whether to enforce strict policy checks.
    pub strict_mode: bool,

    /// Maximum proof size in bytes (0 = unlimited).
    pub max_proof_size: usize,

    /// Proof validity duration in seconds (0 = no expiry).
    pub validity_duration_secs: u64,
}

impl Default for ProofConfig {
    fn default() -> Self {
        Self::production()
    }
}

impl ProofConfig {
    /// Create a new configuration with the specified policy tier.
    pub const fn new(security_level: SecurityLevel) -> Self {
        Self {
            security_level,
            strict_mode: true,
            max_proof_size: 0,
            validity_duration_secs: 0,
        }
    }

    /// General production policy configuration with `Standard` tier.
    ///
    /// This does not establish measured cryptographic production safety.
    pub const fn production() -> Self {
        Self {
            security_level: SecurityLevel::Standard,
            strict_mode: true,
            max_proof_size: 1024 * 1024,
            validity_duration_secs: 3600,
        }
    }

    /// Critical-operation policy configuration.
    ///
    /// Concrete proofs still require measured security evidence and theorem
    /// qualification before application authority.
    pub const fn high_security() -> Self {
        Self {
            security_level: SecurityLevel::High,
            strict_mode: true,
            max_proof_size: 2 * 1024 * 1024,
            validity_duration_secs: 1800,
        }
    }

    /// Testing configuration - NOT FOR PRODUCTION.
    pub const fn testing() -> Self {
        Self {
            security_level: SecurityLevel::Fast,
            strict_mode: false,
            max_proof_size: 0,
            validity_duration_secs: 0,
        }
    }

    /// Get policy configuration for a specific use case.
    pub const fn for_use_case(use_case: UseCase) -> Self {
        let security_level = use_case.recommended_security_level();
        match use_case {
            UseCase::Testing => Self::testing(),
            UseCase::ConstitutionalVote | UseCase::TreasuryVote | UseCase::CrossChainBridge => {
                Self::high_security()
            }
            _ => Self {
                security_level,
                strict_mode: true,
                max_proof_size: 1024 * 1024,
                validity_duration_secs: 3600,
            },
        }
    }

    /// Whether this configuration is policy-approved for production deployment.
    ///
    /// This does not establish measured proof security or theorem qualification.
    pub const fn is_policy_approved_for_production(&self) -> bool {
        self.security_level.is_policy_approved_for_production() && self.strict_mode
    }

    /// Backward-compatible policy-only alias.
    pub const fn is_production_safe(&self) -> bool {
        self.is_policy_approved_for_production()
    }

    /// Builder method to set security policy tier.
    pub const fn with_security_level(mut self, level: SecurityLevel) -> Self {
        self.security_level = level;
        self
    }

    /// Builder method to set strict mode.
    pub const fn with_strict_mode(mut self, strict: bool) -> Self {
        self.strict_mode = strict;
        self
    }

    /// Builder method to set max proof size.
    pub const fn with_max_proof_size(mut self, size: usize) -> Self {
        self.max_proof_size = size;
        self
    }

    /// Builder method to set validity duration.
    pub const fn with_validity_duration(mut self, secs: u64) -> Self {
        self.validity_duration_secs = secs;
        self
    }
}

/// Compatibility policy target required for general production deployment.
///
/// This is not a measured proof-security result.
pub const PRODUCTION_MIN_SECURITY_BITS: u32 = 96;
/// Explicitly named alias for new policy code.
pub const PRODUCTION_MIN_POLICY_TARGET_BITS: u32 = PRODUCTION_MIN_SECURITY_BITS;

/// K-Vector specific constants
pub mod kvector {
    /// Number of K-Vector components
    pub const NUM_COMPONENTS: usize = 8;

    /// Scale factor for fixed-point representation (4 decimal places)
    pub const SCALE_FACTOR: u64 = 10_000;

    /// Maximum scaled value (1.0 * SCALE_FACTOR)
    pub const MAX_SCALED_VALUE: u64 = 10_000;

    /// K-Vector component names
    pub const COMPONENT_NAMES: [&str; NUM_COMPONENTS] = [
        "k_r",
        "k_a",
        "k_i",
        "k_p",
        "k_m",
        "k_s",
        "k_h",
        "k_topo",
    ];
}

/// Governance-specific policy constants
pub mod governance {
    use super::SecurityLevel;

    /// Minimum policy tier for standard votes
    pub const STANDARD_VOTE_MIN_SECURITY: SecurityLevel = SecurityLevel::Standard;

    /// Minimum policy tier for constitutional votes
    pub const CONSTITUTIONAL_VOTE_MIN_SECURITY: SecurityLevel = SecurityLevel::High;

    /// Minimum policy tier for treasury votes
    pub const TREASURY_VOTE_MIN_SECURITY: SecurityLevel = SecurityLevel::Standard;

    /// Default proof validity for votes (1 hour)
    pub const VOTE_PROOF_VALIDITY_SECS: u64 = 3600;

    /// Extended proof validity for constitutional votes (30 minutes - tighter)
    pub const CONSTITUTIONAL_PROOF_VALIDITY_SECS: u64 = 1800;
}

/// Historical Winterfell parameter presets selected by policy tier (feature-gated).
///
/// Selecting one of these presets does not establish its measured cryptographic
/// security or theorem qualification. Exact profiles must be measured separately.
#[cfg(feature = "winterfell")]
pub mod winterfell_options {
    use super::SecurityLevel;
    use winterfell::{BatchingMethod, ProofOptions};

    /// Get the historical Winterfell `ProofOptions` preset for a policy tier.
    ///
    /// Winterfell 0.13 requires batching methods to be explicit. We use linear
    /// batching for both constraint and DEEP composition so the compatibility
    /// presets do not silently adopt algebraic batching's additional soundness
    /// loss. These remain policy presets, not measured-security claims.
    pub fn proof_options_for_level(level: SecurityLevel) -> ProofOptions {
        match level {
            SecurityLevel::Fast => ProofOptions::new(
                28,
                8,
                0,
                winterfell::FieldExtension::None,
                4,
                31,
                BatchingMethod::Linear,
                BatchingMethod::Linear,
            ),
            SecurityLevel::Optimized => ProofOptions::new(
                40,
                8,
                16,
                winterfell::FieldExtension::None,
                4,
                31,
                BatchingMethod::Linear,
                BatchingMethod::Linear,
            ),
            SecurityLevel::Standard => ProofOptions::new(
                50,
                8,
                20,
                winterfell::FieldExtension::None,
                8,
                127,
                BatchingMethod::Linear,
                BatchingMethod::Linear,
            ),
            SecurityLevel::High => ProofOptions::new(
                100,
                16,
                24,
                winterfell::FieldExtension::Quadratic,
                8,
                255,
                BatchingMethod::Linear,
                BatchingMethod::Linear,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_security_level_ordering() {
        assert!(SecurityLevel::Fast < SecurityLevel::Optimized);
        assert!(SecurityLevel::Optimized < SecurityLevel::Standard);
        assert!(SecurityLevel::Standard < SecurityLevel::High);
    }

    #[test]
    fn test_policy_target_bits() {
        assert_eq!(SecurityLevel::Fast.policy_target_bits(), 40);
        assert_eq!(SecurityLevel::Optimized.policy_target_bits(), 84);
        assert_eq!(SecurityLevel::Standard.policy_target_bits(), 96);
        assert_eq!(SecurityLevel::High.policy_target_bits(), 128);
        assert_eq!(
            SecurityLevel::High.estimated_security_bits(),
            SecurityLevel::High.policy_target_bits()
        );
    }

    #[test]
    fn test_policy_production_approval() {
        assert!(!SecurityLevel::Fast.is_policy_approved_for_production());
        assert!(!SecurityLevel::Optimized.is_policy_approved_for_production());
        assert!(SecurityLevel::Standard.is_policy_approved_for_production());
        assert!(SecurityLevel::High.is_policy_approved_for_production());
        assert_eq!(
            SecurityLevel::High.is_production_safe(),
            SecurityLevel::High.is_policy_approved_for_production()
        );
    }

    #[test]
    fn test_policy_meets_minimum() {
        assert!(SecurityLevel::Standard.meets_minimum(96));
        assert!(SecurityLevel::Standard.meets_minimum(84));
        assert!(!SecurityLevel::Standard.meets_minimum(100));
        assert!(SecurityLevel::High.meets_minimum(128));
        assert!(!SecurityLevel::High.meets_minimum(129));
        assert!(!SecurityLevel::High.meets_minimum(200));
    }

    #[test]
    fn test_minimum_policy_level_for_bits() {
        assert_eq!(
            SecurityLevel::minimum_level_for_bits(40),
            Some(SecurityLevel::Fast)
        );
        assert_eq!(
            SecurityLevel::minimum_level_for_bits(50),
            Some(SecurityLevel::Optimized)
        );
        assert_eq!(
            SecurityLevel::minimum_level_for_bits(96),
            Some(SecurityLevel::Standard)
        );
        assert_eq!(
            SecurityLevel::minimum_level_for_bits(128),
            Some(SecurityLevel::High)
        );
        assert_eq!(SecurityLevel::minimum_level_for_bits(129), None);
        assert_eq!(SecurityLevel::minimum_level_for_bits(300), None);
    }

    #[test]
    fn test_use_case_security_levels() {
        assert_eq!(
            UseCase::Testing.recommended_security_level(),
            SecurityLevel::Fast
        );
        assert_eq!(
            UseCase::StandardVote.recommended_security_level(),
            SecurityLevel::Standard
        );
        assert_eq!(
            UseCase::ConstitutionalVote.recommended_security_level(),
            SecurityLevel::High
        );
        assert_eq!(
            UseCase::TreasuryVote.recommended_security_level(),
            SecurityLevel::High
        );
    }

    #[test]
    fn test_proof_config_production_policy() {
        let config = ProofConfig::production();
        assert!(config.is_policy_approved_for_production());
        assert_eq!(config.is_production_safe(), config.is_policy_approved_for_production());
        assert_eq!(config.security_level, SecurityLevel::Standard);
        assert!(config.strict_mode);
    }

    #[test]
    fn test_proof_config_for_use_case() {
        let config = ProofConfig::for_use_case(UseCase::ConstitutionalVote);
        assert_eq!(config.security_level, SecurityLevel::High);
        assert!(config.strict_mode);

        let config = ProofConfig::for_use_case(UseCase::Testing);
        assert_eq!(config.security_level, SecurityLevel::Fast);
        assert!(!config.strict_mode);
    }

    #[test]
    fn test_proof_config_builder() {
        let config = ProofConfig::new(SecurityLevel::Optimized)
            .with_strict_mode(true)
            .with_max_proof_size(512 * 1024)
            .with_validity_duration(7200);

        assert_eq!(config.security_level, SecurityLevel::Optimized);
        assert!(config.strict_mode);
        assert_eq!(config.max_proof_size, 512 * 1024);
        assert_eq!(config.validity_duration_secs, 7200);
    }

    #[test]
    fn test_kvector_constants() {
        assert_eq!(kvector::NUM_COMPONENTS, 8);
        assert_eq!(kvector::SCALE_FACTOR, 10_000);
        assert_eq!(kvector::COMPONENT_NAMES.len(), kvector::NUM_COMPONENTS);
    }

    #[test]
    fn test_serialization() {
        let config = ProofConfig::production();
        let json = serde_json::to_string(&config).unwrap();
        let parsed: ProofConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config, parsed);
    }

    #[test]
    fn test_security_level_serialization() {
        let level = SecurityLevel::Standard;
        let json = serde_json::to_string(&level).unwrap();
        assert_eq!(json, "\"Standard\"");
        let parsed: SecurityLevel = serde_json::from_str(&json).unwrap();
        assert_eq!(level, parsed);
    }
}
