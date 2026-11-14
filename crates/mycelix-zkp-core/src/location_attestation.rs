// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
//! Location-attestation data structures and partial trust-tier adapters.
//!
//! The Winterfell-backed jurisdiction circuit can prove that a supplied
//! coordinate is within a selected bounding box; it does not establish that
//! the coordinate is truthful or that a trusted device measured it. The
//! attestation source must be independently verified.
//!
//! **Implementation boundary:** T0 accepts matching self-attested tier
//! metadata. The current T1 phone-GPS adapter checks tier, an optional public-key
//! allowlist, and non-empty signature bytes; it does not verify the signature
//! cryptographically here and does not enforce `max_age_seconds` in
//! `PhoneGps::verify`. T2 civic, T3 hardware-TEE, and T4 notary adapters return
//! `NotImplemented`. The existence of an enum value or trust-tier label is not
//! evidence that an attestation mechanism is implemented.
//!
//! Callers must validate signature, key binding, freshness, device / sensor
//! provenance, and trust policy using a concrete adapter before treating the
//! coordinate as attested.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

pub use crate::circuits::jurisdiction_proof::AttestationTier;

/// Raw location-attestation data supplied by a producer.
///
/// The presence of a key and signature byte vector in this struct does not
/// mean that the signature was verified. The downstream caller must establish
/// the key's trust, signature validity, and freshness before passing the
/// coordinate into a jurisdiction proof.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LocationAttestation {
    /// Unbiased latitude in decimal degrees.
    pub lat_degrees: f64,
    /// Unbiased longitude in decimal degrees.
    pub lng_degrees: f64,
    /// Unix timestamp at which the attester produced this claim.
    pub timestamp_unix: u64,
    /// Attester's public identifier (e.g., device-key DID, civic-bridge
    /// agent pubkey, notary network root). Hashed for the proof.
    pub attester_pubkey: Vec<u8>,
    /// Claimed signature over `(lat, lng, timestamp)`.
    /// The current T1 adapter only checks that this vector is non-empty; it
    /// does not cryptographically verify these bytes in this crate.
    pub attester_signature: Vec<u8>,
    /// Which tier of attester produced this claim.
    pub tier: AttestationTier,
}

impl LocationAttestation {
    /// SHA-256 hash of the attester pubkey. This is what reaches the
    /// STARK's public input — the raw pubkey never does.
    pub fn attester_pubkey_hash(&self) -> [u8; 32] {
        let digest = Sha256::digest(&self.attester_pubkey);
        let mut out = [0u8; 32];
        out.copy_from_slice(&digest);
        out
    }

    /// Whether this attestation is within `max_age_seconds` of `now_unix`.
    /// Verifiers typically require very recent attestations (minutes,
    /// not days) to reduce the window for GPS replay.
    pub fn is_fresh(&self, now_unix: u64, max_age_seconds: u64) -> bool {
        now_unix.saturating_sub(self.timestamp_unix) <= max_age_seconds
    }
}

/// Trait for tier-specific attestation adapters. The contract requires a
/// concrete implementation to verify its claimed source; the current T1
/// adapter is only an envelope/allowlist check and T2–T4 are stubs. Callers
/// must not infer cryptographic verification from the trait name alone.
pub trait AttestationSource {
    /// Which tier this source produces.
    fn tier(&self) -> AttestationTier;

    /// Apply this adapter's implemented checks to `attestation`.
    /// `Ok(())` only means those checks passed; it does not imply signature
    /// verification or freshness unless the adapter explicitly performs them.
    fn verify(&self, attestation: &LocationAttestation) -> Result<(), AttestationError>;
}

/// Errors that can occur when producing or verifying an attestation.
#[derive(Debug, thiserror::Error, Clone, Serialize, Deserialize)]
pub enum AttestationError {
    /// The attestation's tier does not match this source.
    #[error("tier mismatch: expected {expected:?}, got {got:?}")]
    TierMismatch {
        expected: AttestationTier,
        got: AttestationTier,
    },
    /// Signature verification failed.
    #[error("signature verification failed: {reason}")]
    SignatureInvalid { reason: String },
    /// The attester's public key is not trusted by this source.
    #[error("attester pubkey not in trust set")]
    UntrustedAttester,
    /// The attestation is older than the source permits.
    #[error("attestation too old (age {age_seconds}s exceeds max {max_seconds}s)")]
    Stale { age_seconds: u64, max_seconds: u64 },
    /// The underlying platform (GPS, TEE, notary) is unavailable.
    #[error("attestation source unavailable: {reason}")]
    SourceUnavailable { reason: String },
    /// Feature not implemented at the current tier.
    #[error("tier {tier:?} not yet implemented")]
    NotImplemented { tier: AttestationTier },
}

/// T0 — self-attested. Trivially accepted; verifiers should reject
/// anything that only reaches this tier. Useful only for testing or
/// for cases where the verifier has independent out-of-band trust.
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
        Ok(())
    }
}

/// T1 — phone-GPS envelope check (partial).
///
/// The current core implementation checks tier equality, an optional
/// public-key allowlist, and non-empty signature bytes. It does NOT verify
/// the signature, validate a hardware-backed key attestation, or enforce
/// `max_age_seconds` inside `verify`. The `is_fresh` helper must be called
/// separately by a trusted caller. A downstream integration must implement
/// and test the missing signature and device-integrity checks before this
/// adapter can be described as verified phone-GPS attestation.
pub struct PhoneGps {
    /// Intended maximum attestation age in seconds. The current `verify`
    /// method does not enforce this value; callers must invoke `is_fresh`
    /// or add a time-aware verifier. Default: 300 (five minutes).
    pub max_age_seconds: u64,
    /// Acceptable device-pubkey set. If empty, any pubkey is accepted
    /// (useful for first-run when the verifier has no device-key
    /// expectation yet). Production verifiers SHOULD populate this
    /// from user-controlled onboarding.
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

impl AttestationSource for PhoneGps {
    fn tier(&self) -> AttestationTier {
        AttestationTier::T1PhoneGps
    }

    fn verify(&self, attestation: &LocationAttestation) -> Result<(), AttestationError> {
        if attestation.tier != AttestationTier::T1PhoneGps {
            return Err(AttestationError::TierMismatch {
                expected: AttestationTier::T1PhoneGps,
                got: attestation.tier,
            });
        }

        if !self.trusted_device_pubkeys.is_empty()
            && !self
                .trusted_device_pubkeys
                .iter()
                .any(|k| k == &attestation.attester_pubkey)
        {
            return Err(AttestationError::UntrustedAttester);
        }

        // This is only an envelope check. No cryptographic signature
        // verification or freshness check is performed by this adapter.
        if attestation.attester_signature.is_empty() {
            return Err(AttestationError::SignatureInvalid {
                reason: "empty signature".to_string(),
            });
        }
        Ok(())
    }
}

/// T2 — reserved for a civic-bridge attestation adapter.
///
/// The implementation in this crate is a stub and always returns
/// `NotImplemented`. No live civic observation or multi-party
/// attestation is verified by this type.
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

/// T3 — reserved for a hardware-backed attestation adapter.
///
/// The implementation in this crate is a stub and always returns
/// `NotImplemented`. It does not verify Android / iOS hardware attestation
/// or prove physical presence.
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

/// T4 — reserved for a notary-oracle attestation adapter.
///
/// The implementation in this crate is a stub and always returns
/// `NotImplemented`. No notary network signature or threshold is verified.
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
    fn pubkey_hash_stable() {
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        let b = sample_attestation(AttestationTier::T1PhoneGps);
        assert_eq!(a.attester_pubkey_hash(), b.attester_pubkey_hash());
    }

    #[test]
    fn freshness_window() {
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        assert!(a.is_fresh(1_713_400_060, 300));
        assert!(!a.is_fresh(1_713_401_000, 300));
    }

    #[test]
    fn self_attested_accepts_matching_tier() {
        let src = SelfAttested;
        let a = sample_attestation(AttestationTier::T0SelfAttested);
        assert!(src.verify(&a).is_ok());
    }

    #[test]
    fn self_attested_rejects_wrong_tier() {
        let src = SelfAttested;
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        match src.verify(&a) {
            Err(AttestationError::TierMismatch { .. }) => {}
            other => panic!("expected TierMismatch, got {other:?}"),
        }
    }

    #[test]
    fn phone_gps_requires_nonempty_signature() {
        let src = PhoneGps::default();
        let mut a = sample_attestation(AttestationTier::T1PhoneGps);
        a.attester_signature.clear();
        match src.verify(&a) {
            Err(AttestationError::SignatureInvalid { .. }) => {}
            other => panic!("expected SignatureInvalid, got {other:?}"),
        }
    }

    #[test]
    fn phone_gps_trusted_pubkey_allowlist() {
        let src = PhoneGps {
            max_age_seconds: 300,
            trusted_device_pubkeys: vec![vec![9, 9, 9, 9]],
        };
        let a = sample_attestation(AttestationTier::T1PhoneGps);
        match src.verify(&a) {
            Err(AttestationError::UntrustedAttester) => {}
            other => panic!("expected UntrustedAttester, got {other:?}"),
        }
    }

    #[test]
    fn higher_tiers_unimplemented() {
        for src in [
            Box::new(CivicBridge) as Box<dyn AttestationSource>,
            Box::new(HardwareTee),
            Box::new(NotaryOracle),
        ] {
            let a = sample_attestation(src.tier());
            match src.verify(&a) {
                Err(AttestationError::NotImplemented { .. }) => {}
                other => panic!(
                    "expected NotImplemented at tier {:?}, got {other:?}",
                    src.tier()
                ),
            }
        }
    }

    #[test]
    fn tier_re_export_matches_circuit_module() {
        // Sanity check that the re-export lines up with the circuit's enum.
        assert_eq!(
            AttestationTier::default_minimum(),
            AttestationTier::T1PhoneGps
        );
    }
}
