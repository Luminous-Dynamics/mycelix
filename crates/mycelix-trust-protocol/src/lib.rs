// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned cross-domain trust-resolution wire types.
//!
//! `TrustResolutionV1` is deliberately a **quarantine-only** and
//! **observation-scoped** protocol. It can report what the producer established
//! from its current structural evidence, but it cannot claim global DHT
//! currentness, cryptographic proof verification, or elevated authority.
//!
//! A future protocol carrying accepted signed verification-record evidence must
//! use a new versioned type rather than weakening V1 in place.

#![forbid(unsafe_code)]

use serde::{Deserialize, Deserializer, Serialize};

pub const TRUST_RESOLUTION_V1_SCHEMA: u16 = 1;

/// Structural tier vocabulary shared on the wire.
///
/// These names describe credential tiers only. They do not imply identity
/// verification, capability authority, proof validity, or downstream trust.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum StructuralTrustTierV1 {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

/// What a successful producer observation established structurally.
///
/// The observation wording is normative: these variants must not be interpreted
/// as proof that the producer has a globally complete DHT view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StructuralTrustStateV1 {
    /// No active structural credential was established under the observed
    /// evidence available to this resolution.
    NoActiveCredentialObserved,
    /// The observed evidence contained one structurally active credential with
    /// this diagnostic tier.
    ObservedActiveTier(StructuralTrustTierV1),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProofVerificationStateV1 {
    NotEstablished,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrustAuthorityDispositionV1 {
    Quarantined,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TrustResolutionV1 {
    #[serde(deserialize_with = "deserialize_v1_schema")]
    schema_version: u16,
    structural: StructuralTrustStateV1,
    proof_verification: ProofVerificationStateV1,
    authority: TrustAuthorityDispositionV1,
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

    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    pub const fn structural(&self) -> StructuralTrustStateV1 {
        self.structural
    }

    pub const fn proof_verification(&self) -> ProofVerificationStateV1 {
        self.proof_verification
    }

    pub const fn authority(&self) -> TrustAuthorityDispositionV1 {
        self.authority
    }

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

fn deserialize_v1_schema<'de, D>(deserializer: D) -> Result<u16, D::Error>
where
    D: Deserializer<'de>,
{
    let actual = u16::deserialize(deserializer)?;
    if actual == TRUST_RESOLUTION_V1_SCHEMA {
        Ok(actual)
    } else {
        Err(serde::de::Error::custom(format_args!(
            "unsupported TrustResolutionV1 schema version: expected {}, got {}",
            TRUST_RESOLUTION_V1_SCHEMA, actual
        )))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustResolutionV1Error {
    UnsupportedSchemaVersion { expected: u16, actual: u16 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Serialize)]
    struct WireLikeTrustResolutionV1 {
        schema_version: u16,
        structural: StructuralTrustStateV1,
        proof_verification: ProofVerificationStateV1,
        authority: TrustAuthorityDispositionV1,
    }

    #[test]
    fn every_observed_structural_tier_remains_quarantined() {
        let tiers = [
            StructuralTrustTierV1::Observer,
            StructuralTrustTierV1::Basic,
            StructuralTrustTierV1::Standard,
            StructuralTrustTierV1::Elevated,
            StructuralTrustTierV1::Guardian,
        ];

        for tier in tiers {
            let resolution = TrustResolutionV1::quarantined(
                StructuralTrustStateV1::ObservedActiveTier(tier),
            );
            assert_eq!(resolution.schema_version(), TRUST_RESOLUTION_V1_SCHEMA);
            assert_eq!(resolution.proof_verification(), ProofVerificationStateV1::NotEstablished);
            assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
            assert_eq!(resolution.validate_schema(), Ok(()));
        }
    }

    #[test]
    fn observed_absence_is_distinct_from_observer_credential() {
        let none = TrustResolutionV1::quarantined(
            StructuralTrustStateV1::NoActiveCredentialObserved,
        );
        let observer = TrustResolutionV1::quarantined(
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Observer),
        );
        assert_ne!(none.structural(), observer.structural());
        assert_eq!(none.authority(), observer.authority());
    }

    #[test]
    fn structural_state_vocabulary_is_observation_scoped() {
        let active = format!(
            "{:?}",
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian)
        );
        let none = format!("{:?}", StructuralTrustStateV1::NoActiveCredentialObserved);
        assert!(active.contains("Observed"));
        assert!(none.contains("Observed"));
        assert!(!active.contains("Verified"));
        assert!(!active.contains("Authorized"));
    }

    #[test]
    fn unsupported_schema_is_rejected_during_json_deserialization() {
        let encoded = r#"{
            "schema_version": 2,
            "structural": "NoActiveCredentialObserved",
            "proof_verification": "NotEstablished",
            "authority": "Quarantined"
        }"#;
        assert!(serde_json::from_str::<TrustResolutionV1>(encoded).is_err());
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let encoded = r#"{
            "schema_version": 1,
            "structural": "NoActiveCredentialObserved",
            "proof_verification": "NotEstablished",
            "authority": "Quarantined",
            "future_authority": "Verified"
        }"#;
        assert!(serde_json::from_str::<TrustResolutionV1>(encoded).is_err());
    }

    #[test]
    fn serde_round_trip_preserves_quarantine_semantics() {
        let resolution = TrustResolutionV1::quarantined(
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian),
        );
        let encoded = serde_json::to_string(&resolution).unwrap();
        let decoded: TrustResolutionV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, resolution);
        assert_eq!(decoded.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn holochain_messagepack_round_trip_preserves_quarantine_semantics() {
        let resolution = TrustResolutionV1::quarantined(
            StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian),
        );
        let encoded = holochain_serialized_bytes::encode(&resolution).unwrap();
        let decoded: TrustResolutionV1 = holochain_serialized_bytes::decode(&encoded).unwrap();
        assert_eq!(decoded, resolution);
        assert_eq!(decoded.validate_schema(), Ok(()));
    }

    #[test]
    fn holochain_messagepack_rejects_unsupported_schema() {
        let unsupported = WireLikeTrustResolutionV1 {
            schema_version: TRUST_RESOLUTION_V1_SCHEMA + 1,
            structural: StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian),
            proof_verification: ProofVerificationStateV1::NotEstablished,
            authority: TrustAuthorityDispositionV1::Quarantined,
        };
        let encoded = holochain_serialized_bytes::encode(&unsupported).unwrap();
        assert!(holochain_serialized_bytes::decode::<_, TrustResolutionV1>(&encoded).is_err());
    }

    #[test]
    fn holochain_messagepack_rejects_extended_v1_payload() {
        #[derive(Debug, Serialize)]
        struct ExtendedTrustResolutionV1 {
            schema_version: u16,
            structural: StructuralTrustStateV1,
            proof_verification: ProofVerificationStateV1,
            authority: TrustAuthorityDispositionV1,
            future_authority: &'static str,
        }

        let extended = ExtendedTrustResolutionV1 {
            schema_version: TRUST_RESOLUTION_V1_SCHEMA,
            structural: StructuralTrustStateV1::ObservedActiveTier(StructuralTrustTierV1::Guardian),
            proof_verification: ProofVerificationStateV1::NotEstablished,
            authority: TrustAuthorityDispositionV1::Quarantined,
            future_authority: "Verified",
        };
        let encoded = holochain_serialized_bytes::encode(&extended).unwrap();
        assert!(holochain_serialized_bytes::decode::<_, TrustResolutionV1>(&encoded).is_err());
    }
}
