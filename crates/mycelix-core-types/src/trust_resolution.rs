// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned cross-domain trust-resolution wire types.
//!
//! `TrustResolutionV1` is deliberately a **quarantine-only** protocol.
//! It can report structurally admitted credential state for diagnostics, but it
//! cannot represent cryptographic proof verification or elevated authority.
//!
//! This prevents a producer's local trust-tier enum from being reinterpreted as
//! a consumer's authority enum and prevents structural credential admission from
//! silently becoming consequential trust.
//!
//! A future protocol that carries accepted signed verification-record evidence
//! must use a new versioned type rather than weakening V1 in place.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Exact wire schema version for [`TrustResolutionV1`].
pub const TRUST_RESOLUTION_V1_SCHEMA: u16 = 1;

/// Structural tier vocabulary shared on the wire.
///
/// These names describe the credential tier only. They do not imply identity
/// verification, capability authority, proof validity, or downstream trust.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum StructuralTrustTierV1 {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

/// What a successful producer lookup can establish structurally.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum StructuralTrustStateV1 {
    /// No active structural credential was found.
    NoActiveCredential,
    /// An active structural credential reported this tier.
    ActiveTier(StructuralTrustTierV1),
}

/// Cryptographic proof-verification state in V1.
///
/// V1 intentionally has no `Verified` variant. The current credential path does
/// not yet carry the signed append-only verification records required to make
/// that claim safely across domains.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ProofVerificationStateV1 {
    NotEstablished,
}

/// Whether this resolution may influence consequential authority/weight.
///
/// V1 is permanently quarantine-only. Differentiated authority requires a new
/// protocol version backed by accepted signed verification evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum TrustAuthorityDispositionV1 {
    Quarantined,
}

/// Cross-domain trust resolution V1.
///
/// The explicit schema field is part of the serialized payload so consumers can
/// reject unknown versions rather than decoding them as a locally similar type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TrustResolutionV1 {
    pub schema_version: u16,
    pub structural: StructuralTrustStateV1,
    pub proof_verification: ProofVerificationStateV1,
    pub authority: TrustAuthorityDispositionV1,
}

impl TrustResolutionV1 {
    /// Construct the only authority disposition permitted by V1.
    pub const fn quarantined(structural: StructuralTrustStateV1) -> Self {
        Self {
            schema_version: TRUST_RESOLUTION_V1_SCHEMA,
            structural,
            proof_verification: ProofVerificationStateV1::NotEstablished,
            authority: TrustAuthorityDispositionV1::Quarantined,
        }
    }

    /// Validate the explicit wire version before a consumer interprets fields.
    pub fn validate_schema(&self) -> Result<(), TrustResolutionV1Error> {
        if self.schema_version == TRUST_RESOLUTION_V1_SCHEMA {
            Ok(())
        } else {
            Err(TrustResolutionV1Error::UnsupportedSchemaVersion {
                expected: TRUST_RESOLUTION_V1_SCHEMA,
                actual: self.schema_version,
            })
        }
    }
}

/// Contract-validation failures for V1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustResolutionV1Error {
    UnsupportedSchemaVersion { expected: u16, actual: u16 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn guardian_structural_tier_remains_quarantined() {
        let resolution = TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(
            StructuralTrustTierV1::Guardian,
        ));

        assert_eq!(resolution.schema_version, TRUST_RESOLUTION_V1_SCHEMA);
        assert_eq!(
            resolution.proof_verification,
            ProofVerificationStateV1::NotEstablished
        );
        assert_eq!(resolution.authority, TrustAuthorityDispositionV1::Quarantined);
        assert_eq!(resolution.validate_schema(), Ok(()));
    }

    #[test]
    fn no_credential_is_distinct_from_observer_credential() {
        let none = TrustResolutionV1::quarantined(StructuralTrustStateV1::NoActiveCredential);
        let observer = TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(
            StructuralTrustTierV1::Observer,
        ));

        assert_ne!(none.structural, observer.structural);
        assert_eq!(none.authority, observer.authority);
    }

    #[test]
    fn unknown_schema_version_fails_closed() {
        let mut resolution = TrustResolutionV1::quarantined(
            StructuralTrustStateV1::ActiveTier(StructuralTrustTierV1::Elevated),
        );
        resolution.schema_version = TRUST_RESOLUTION_V1_SCHEMA + 1;

        assert_eq!(
            resolution.validate_schema(),
            Err(TrustResolutionV1Error::UnsupportedSchemaVersion {
                expected: TRUST_RESOLUTION_V1_SCHEMA,
                actual: TRUST_RESOLUTION_V1_SCHEMA + 1,
            })
        );
        assert_eq!(resolution.authority, TrustAuthorityDispositionV1::Quarantined);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_quarantine_semantics() {
        let resolution = TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(
            StructuralTrustTierV1::Guardian,
        ));

        let encoded = serde_json::to_string(&resolution).expect("serialize TrustResolutionV1");
        let decoded: TrustResolutionV1 =
            serde_json::from_str(&encoded).expect("deserialize TrustResolutionV1");

        assert_eq!(decoded, resolution);
        assert_eq!(decoded.validate_schema(), Ok(()));
        assert_eq!(decoded.authority, TrustAuthorityDispositionV1::Quarantined);
        assert_eq!(
            decoded.proof_verification,
            ProofVerificationStateV1::NotEstablished
        );
    }
}
