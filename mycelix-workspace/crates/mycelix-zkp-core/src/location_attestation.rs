// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
//! Proof-independent location-attestation envelopes and trust tiers.
//!
//! This module does **not** establish geographic truth and does not depend on the
//! quarantined jurisdiction/range proof lineage. A future qualified jurisdiction
//! proof may consume an attestation, but the following remain separate claims:
//!
//! ```text
//! envelope well formed
//!     != cryptographic attester signature verified
//!     != sensor/location claim trustworthy
//!     != jurisdiction containment proven
//! ```

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Trust tier claimed by a location-attestation source.
///
/// A tier identifies the intended source class; merely setting this enum does not
/// establish that the corresponding source actually produced or verified a claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum AttestationTier {
    /// Self-attested. No independent source truth is established.
    T0SelfAttested = 0,
    /// Phone GPS plus a device-key claim. Core verification is not implemented.
    T1PhoneGps = 1,
    /// Civic infrastructure / multi-party source. Core implementation absent.
    T2CivicBridge = 2,
    /// Hardware-backed attestation source. Core implementation absent.
    T3HardwareTee = 3,
    /// Threshold notary/oracle source. Core implementation absent.
    T4Notary = 4,
}

impl AttestationTier {
    /// Historical/default policy floor. This does not imply T1 is implemented by
    /// the core crate; verifiers must still require an operational source.
    pub const fn default_minimum() -> Self {
        AttestationTier::T1PhoneGps
    }

    pub const fn meets(self, required: AttestationTier) -> bool {
        (self as u8) >= (required as u8)
    }
}

/// A location attestation envelope on the prover side.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LocationAttestation {
    pub lat_degrees: f64,
    pub lng_degrees: f64,
    pub timestamp_unix: u64,
    pub attester_pubkey: Vec<u8>,
    pub attester_signature: Vec<u8>,
    pub tier: AttestationTier,
}

impl LocationAttestation {
    pub fn attester_pubkey_hash(&self) -> [u8; 32] {
        let digest = Sha256::digest(&self.attester_pubkey);
        let mut out = [0u8; 32];
        out.copy_from_slice(&digest);
        out
    }

    /// Whether the coordinate payload is finite and inside WGS-84 latitude /
    /// longitude domains.
    pub fn has_valid_coordinates(&self) -> bool {
        self.lat_degrees.is_finite()
            && self.lng_degrees.is_finite()
            && (-90.0..=90.0).contains(&self.lat_degrees)
            && (-180.0..=180.0).contains(&self.lng_degrees)
    }

    /// Strict freshness check. Future timestamps are not silently treated as age
    /// zero; callers that need clock-skew tolerance must express that policy
    /// separately rather than relying on saturating subtraction.
    pub fn is_fresh(&self, now_unix: u64, max_age_seconds: u64) -> bool {
        let Some(age) = now_unix.checked_sub(self.timestamp_unix) else {
            return false;
        };
        age <= max_age_seconds
    }
}

/// Trait implemented by cryptographically operational attestation sources.
///
/// `Ok(())` means this implementation actually verified the source-specific
/// authenticity relation it advertises. Envelope hygiene alone must not return
/// success from this method.
pub trait AttestationSource {
    fn tier(&self) -> AttestationTier;

    fn verify(&self, attestation: &LocationAttestation) -> Result<(), AttestationError>;
}

#[derive(Debug, thiserror::Error, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum AttestationError {
    #[error("tier mismatch: expected {expected:?}, got {got:?}")]
    TierMismatch {
        expected: AttestationTier,
        got: AttestationTier,
    },
    #[error("invalid location coordinates")]
    InvalidCoordinates,
    #[error("signature verification failed: {reason}")]
    SignatureInvalid { reason: String },
    #[error("attester pubkey not in trust set")]
    UntrustedAttester,
    #[error("attestation too old (age {age_seconds}s exceeds max {max_seconds}s)")]
    Stale { age_seconds: u64, max_seconds: u64 },
    #[error("attestation timestamp is in the future by {skew_seconds}s")]
    FutureTimestamp { skew_seconds: u64 },
    #[error("attestation source unavailable: {reason}")]
    SourceUnavailable { reason: String },
    #[error("tier {tier:?} not yet implemented")]
    NotImplemented { tier: AttestationTier },
}

/// T0 — explicit self-attestation.
///
/// A successful check establishes only that the envelope labels itself T0. It is
/// not independent evidence of physical location.
pub struct SelfAttested;

impl AttestationSource for SelfAttested {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T0SelfAttested
    }

    fn verify(&self, attestation: &LocationAttestation) -> Result<(), AttestationError> {
        if attestation.tier != AttestationTier::T0SelfAttested {
            return Err(AttestationError::TierMismatch {
                expected: AttestationTier::T0SelfAttested,
                got: attestation.tier,
            });
        }
        if !attestation.has_valid_coordinates() {
            return Err(AttestationError::InvalidCoordinates);
        }
        Ok(())
    }
}

/// T1 phone-GPS policy shell.
///
/// The core crate does not have a device-key signature verifier and therefore
/// **never** returns cryptographic success for T1 through `AttestationSource`.
/// Downstream platform integrations must provide the real signature/device trust
/// implementation. `validate_envelope_at` is available only for pre-verification
/// hygiene/policy checks.
pub struct PhoneGps {
    pub max_age_seconds: u64,
    /// Optional expected device-key set. Empty means no key pinning is applied by
    /// this envelope policy; it does not make the attester cryptographically trusted.
    pub trusted_device_pubkeys: Vec<Vec<u8>>,
}

impl Default for PhoneGps {
    fn default() -> Self {
        Self {
            max_age_seconds: 300,
            trusted_device_pubkeys: Vec::new(),
        }
    }
}

impl PhoneGps {
    fn validate_static_envelope(
        &self,
        attestation: &LocationAttestation,
    ) -> Result<(), AttestationError> {
        if attestation.tier != AttestationTier::T1PhoneGps {
            return Err(AttestationError::TierMismatch {
                expected: AttestationTier::T1PhoneGps,
                got: attestation.tier,
            });
        }
        if !attestation.has_valid_coordinates() {
            return Err(AttestationError::InvalidCoordinates);
        }
        if attestation.attester_pubkey.is_empty() {
            return Err(AttestationError::UntrustedAttester);
        }
        if !self.trusted_device_pubkeys.is_empty()
            && !self
                .trusted_device_pubkeys
                .iter()
                .any(|k| k == &attestation.attester_pubkey)
        {
            return Err(AttestationError::UntrustedAttester);
        }
        if attestation.attester_signature.is_empty() {
            return Err(AttestationError::SignatureInvalid {
                reason: "empty signature".to_string(),
            });
        }
        Ok(())
    }

    /// Validate envelope shape, configured key policy, coordinates and freshness
    /// at an explicit verifier time.
    ///
    /// Success here is **not** cryptographic signature verification.
    pub fn validate_envelope_at(
        &self,
        attestation: &LocationAttestation,
        now_unix: u64,
    ) -> Result<(), AttestationError> {
        self.validate_static_envelope(attestation)?;

        if attestation.timestamp_unix > now_unix {
            return Err(AttestationError::FutureTimestamp {
                skew_seconds: attestation.timestamp_unix - now_unix,
            });
        }
        let age = now_unix - attestation.timestamp_unix;
        if age > self.max_age_seconds {
            return Err(AttestationError::Stale {
                age_seconds: age,
                max_seconds: self.max_age_seconds,
            });
        }
        Ok(())
    }
}

impl AttestationSource for PhoneGps {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T1PhoneGps
    }

    fn verify(&self, attestation: &LocationAttestation) -> Result<(), AttestationError> {
        self.validate_static_envelope(attestation)?;
        Err(AttestationError::NotImplemented {
            tier: AttestationTier::T1PhoneGps,
        })
    }
}

pub struct CivicBridge;

impl AttestationSource for CivicBridge {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T2CivicBridge
    }

    fn verify(&self, _attestation: &LocationAttestation) -> Result<(), AttestationError> {
        Err(AttestationError::NotImplemented {
            tier: AttestationTier::T2CivicBridge,
        })
    }
}

pub struct HardwareTee;

impl AttestationSource for HardwareTee {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T3HardwareTee
    }

    fn verify(&self, _attestation: &LocationAttestation) -> Result<(), AttestationError> {
        Err(AttestationError::NotImplemented {
            tier: AttestationTier::T3HardwareTee,
        })
    }
}

pub struct NotaryOracle;

impl AttestationSource for NotaryOracle {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T4Notary
    }

    fn verify(&self, _attestation: &LocationAttestation) -> Result<(), AttestationError> {
        Err(AttestationError::NotImplemented {
            tier: AttestationTier::T4Notary,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_attestation(tier: AttestationTier) -> LocationAttestation {
        LocationAttestation {
            lat_degrees: -26.1625,
            lng_degrees: 27.8725,
            timestamp_unix: 1_713_400_000,
            attester_pubkey: vec![1, 2, 3, 4],
            attester_signature: vec![0xDE, 0xAD, 0xBE, 0xEF],
            tier,
        }
    }

    #[test]
    fn tier_serialization_is_stable() {
        assert_eq!(
            serde_json::to_string(&AttestationTier::T1PhoneGps).unwrap(),
            "\"T1PhoneGps\""
        );
    }

    #[test]
    fn pubkey_hash_stable() {
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        let b = sample_attestation(AttestationTier::T1PhoneGps);
        assert_eq!(a.attester_pubkey_hash(), b.attester_pubkey_hash());
    }

    #[test]
    fn strict_freshness_rejects_future_timestamp() {
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        assert!(a.is_fresh(1_713_400_060, 300));
        assert!(!a.is_fresh(1_713_401_000, 300));
        assert!(!a.is_fresh(1_713_399_999, 300));
    }

    #[test]
    fn invalid_coordinates_rejected() {
        let mut a = sample_attestation(AttestationTier::T0SelfAttested);
        a.lat_degrees = f64::NAN;
        assert_eq!(SelfAttested.verify(&a), Err(AttestationError::InvalidCoordinates));
    }

    #[test]
    fn self_attested_accepts_only_matching_low_authority_tier() {
        let src = SelfAttested;
        assert!(src.verify(&sample_attestation(AttestationTier::T0SelfAttested)).is_ok());
        assert!(matches!(
            src.verify(&sample_attestation(AttestationTier::T1PhoneGps)),
            Err(AttestationError::TierMismatch { .. })
        ));
    }

    #[test]
    fn phone_gps_nonempty_signature_is_not_cryptographic_success() {
        let src = PhoneGps::default();
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        assert_eq!(
            src.verify(&a),
            Err(AttestationError::NotImplemented {
                tier: AttestationTier::T1PhoneGps
            })
        );
    }

    #[test]
    fn phone_gps_requires_nonempty_signature_and_pubkey() {
        let src = PhoneGps::default();
        let mut a = sample_attestation(AttestationTier::T1PhoneGps);
        a.attester_signature.clear();
        assert!(matches!(
            src.verify(&a),
            Err(AttestationError::SignatureInvalid { .. })
        ));

        let mut b = sample_attestation(AttestationTier::T1PhoneGps);
        b.attester_pubkey.clear();
        assert_eq!(src.verify(&b), Err(AttestationError::UntrustedAttester));
    }

    #[test]
    fn phone_gps_allowlist_is_enforced() {
        let src = PhoneGps {
            max_age_seconds: 300,
            trusted_device_pubkeys: vec![vec![9, 9, 9, 9]],
        };
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        assert_eq!(src.verify(&a), Err(AttestationError::UntrustedAttester));
    }

    #[test]
    fn phone_gps_envelope_policy_checks_time_explicitly() {
        let src = PhoneGps::default();
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        assert!(src.validate_envelope_at(&a, 1_713_400_060).is_ok());
        assert!(matches!(
            src.validate_envelope_at(&a, 1_713_401_000),
            Err(AttestationError::Stale { .. })
        ));
        assert!(matches!(
            src.validate_envelope_at(&a, 1_713_399_999),
            Err(AttestationError::FutureTimestamp { .. })
        ));
    }

    #[test]
    fn higher_tiers_remain_fail_closed() {
        for src in [
            Box::new(CivicBridge) as Box<dyn AttestationSource>,
            Box::new(HardwareTee),
            Box::new(NotaryOracle),
        ] {
            let a = sample_attestation(src.tier());
            assert!(matches!(
                src.verify(&a),
                Err(AttestationError::NotImplemented { .. })
            ));
        }
    }

    #[test]
    fn tier_order_is_monotonic_metadata_only() {
        use AttestationTier::*;
        assert!(T4Notary.meets(T0SelfAttested));
        assert!(T2CivicBridge.meets(T1PhoneGps));
        assert!(!T0SelfAttested.meets(T1PhoneGps));
        assert!(T1PhoneGps.meets(T1PhoneGps));
    }
}
