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

#![forbid(unsafe_code)]

use serde::{Deserialize, Deserializer, Serialize};

/// Exact wire schema version for [`TrustResolutionV1`].
pub const TRUST_RESOLUTION_V1_SCHEMA: u16 = 1;

/// Structural tier vocabulary shared on the wire.
///
/// These names describe the credential tier only. They do not imply identity
/// verification, capability authority, proof validity, or downstream trust.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum StructuralTrustTierV1 {
    Observer,
    Basic,
    Standard,
    Elevated,
    Guardian,
}

/// What a successful producer lookup can establish structurally.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProofVerificationStateV1 {
    NotEstablished,
}

/// Whether this resolution may influence consequential authority/weight.
///
/// V1 is permanently quarantine-only. Differentiated authority requires a new
/// protocol version backed by accepted signed verification evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrustAuthorityDispositionV1 {
    Quarantined,
}

/// Cross-domain trust resolution V1.
///
/// Construction is intentionally closed: external callers can only create V1
/// through [`TrustResolutionV1::quarantined`]. Deserialization rejects unknown
/// versions and unknown fields, so a producer cannot extend V1 in place while an
/// older consumer silently ignores new semantics.
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

    /// Exact schema version carried by this resolution.
    pub const fn schema_version(&self) -> u16 {
        self.schema_version
    }

    /// Structurally admitted credential state, for diagnostics only.
    pub const fn structural(&self) -> StructuralTrustStateV1 {
        self.structural
    }

    /// Cryptographic proof-verification state.
    pub const fn proof_verification(&self) -> ProofVerificationStateV1 {
        self.proof_verification
    }

    /// Consequential authority disposition.
    pub const fn authority(&self) -> TrustAuthorityDispositionV1 {
        self.authority
    }

    /// Defense-in-depth validation for values already present in memory.
    ///
    /// Normal wire deserialization rejects unsupported versions before a value
    /// can be produced, but this remains useful for tests and internal audits.
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

/// Contract-validation failures for V1.
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
    fn every_structural_tier_remains_quarantined() {
        let tiers = [
            StructuralTrustTierV1::Observer,
            StructuralTrustTierV1::Basic,
            StructuralTrustTierV1::Standard,
            StructuralTrustTierV1::Elevated,
            StructuralTrustTierV1::Guardian,
        ];

        for tier in tiers {
            let resolution =
                TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(tier));
            assert_eq!(resolution.schema_version(), TRUST_RESOLUTION_V1_SCHEMA);
            assert_eq!(
                resolution.proof_verification(),
                ProofVerificationStateV1::NotEstablished
            );
            assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
            assert_eq!(resolution.validate_schema(), Ok(()));
        }
    }

    #[test]
    fn no_credential_is_distinct_from_observer_credential() {
        let none = TrustResolutionV1::quarantined(StructuralTrustStateV1::NoActiveCredential);
        let observer = TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(
            StructuralTrustTierV1::Observer,
        ));

        assert_ne!(none.structural(), observer.structural());
        assert_eq!(none.authority(), observer.authority());
    }

    #[test]
    fn unsupported_schema_is_rejected_during_json_deserialization() {
        let encoded = r#"{
            "schema_version": 2,
            "structural": "NoActiveCredential",
            "proof_verification": "NotEstablished",
            "authority": "Quarantined"
        }"#;

        let decoded = serde_json::from_str::<TrustResolutionV1>(encoded);
        assert!(decoded.is_err(), "unsupported V1 schema must fail at decode");
    }

    #[test]
    fn validate_schema_is_defense_in_depth() {
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
        assert_eq!(resolution.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

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
        assert_eq!(decoded.authority(), TrustAuthorityDispositionV1::Quarantined);
        assert_eq!(
            decoded.proof_verification(),
            ProofVerificationStateV1::NotEstablished
        );
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let encoded = r#"{
            "schema_version": 1,
            "structural": "NoActiveCredential",
            "proof_verification": "NotEstablished",
            "authority": "Quarantined",
            "future_authority": "Verified"
        }"#;

        let decoded = serde_json::from_str::<TrustResolutionV1>(encoded);
        assert!(decoded.is_err(), "V1 must reject unknown fields");
    }

    #[test]
    fn holochain_messagepack_round_trip_preserves_quarantine_semantics() {
        let resolution = TrustResolutionV1::quarantined(StructuralTrustStateV1::ActiveTier(
            StructuralTrustTierV1::Guardian,
        ));

        let encoded = holochain_serialized_bytes::encode(&resolution)
            .expect("encode TrustResolutionV1 with Holochain canonical MessagePack");
        let decoded: TrustResolutionV1 = holochain_serialized_bytes::decode(&encoded)
            .expect("decode TrustResolutionV1 with Holochain canonical MessagePack");

        assert_eq!(decoded, resolution);
        assert_eq!(decoded.validate_schema(), Ok(()));
        assert_eq!(decoded.authority(), TrustAuthorityDispositionV1::Quarantined);
    }

    #[test]
    fn holochain_messagepack_rejects_unsupported_schema() {
        let unsupported = WireLikeTrustResolutionV1 {
            schema_version: TRUST_RESOLUTION_V1_SCHEMA + 1,
            structural: StructuralTrustStateV1::ActiveTier(StructuralTrustTierV1::Guardian),
            proof_verification: ProofVerificationStateV1::NotEstablished,
            authority: TrustAuthorityDispositionV1::Quarantined,
        };

        let encoded = holochain_serialized_bytes::encode(&unsupported)
            .expect("encode unsupported V1 payload for rejection test");
        let decoded = holochain_serialized_bytes::decode::<_, TrustResolutionV1>(&encoded);

        assert!(decoded.is_err(), "unsupported V1 schema must fail at decode");
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
            structural: StructuralTrustStateV1::ActiveTier(StructuralTrustTierV1::Guardian),
            proof_verification: ProofVerificationStateV1::NotEstablished,
            authority: TrustAuthorityDispositionV1::Quarantined,
            future_authority: "Verified",
        };

        let encoded = holochain_serialized_bytes::encode(&extended)
            .expect("encode extended V1 payload for rejection test");
        let decoded = holochain_serialized_bytes::decode::<_, TrustResolutionV1>(&encoded);

        assert!(decoded.is_err(), "V1 must reject extended Holochain payloads");
    }
}
