// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic, fail-closed validation for authenticated proof envelopes.
//!
//! Structural validation is intentionally separate from cryptographic proof and
//! signature verification. Passing this module means only that an envelope is
//! well-formed and acceptable under an explicit local policy.

use crate::error::{ZkpError, ZkpResult};
use crate::types::{
    AUTHENTICATED_PROOF_PROTOCOL_VERSION, AuthenticatedProof, BackendId,
};

/// Default maximum proof payload accepted by an application boundary (512 KiB).
pub const DEFAULT_MAX_PROOF_BYTES: usize = 512 * 1024;
/// Default maximum signature container size (16 KiB).
pub const DEFAULT_MAX_SIGNATURE_BYTES: usize = 16 * 1024;
/// Default maximum complete envelope size (544 KiB).
pub const DEFAULT_MAX_ENVELOPE_BYTES: usize = 544 * 1024;
/// Default maximum serialized domain-tag length.
pub const DEFAULT_MAX_DOMAIN_TAG_BYTES: usize = 128;

/// Local acceptance policy for an authenticated proof envelope.
#[derive(Clone, Debug)]
pub struct EnvelopeValidationPolicy {
    /// Exact protocol version accepted by this verifier.
    pub protocol_version: u32,
    /// Maximum proof payload length.
    pub max_proof_bytes: usize,
    /// Maximum signature container length.
    pub max_signature_bytes: usize,
    /// Maximum complete envelope length.
    pub max_envelope_bytes: usize,
    /// Maximum domain-tag length.
    pub max_domain_tag_bytes: usize,
    /// Maximum accepted proof age.
    pub max_age_seconds: u64,
    /// Maximum accepted timestamp skew into the future.
    pub max_future_skew_seconds: u64,
    /// Maximum authenticated energy claim.
    pub max_energy_millijoules: u64,
    /// Whether a non-empty signature is required.
    pub require_signature: bool,
    /// Optional backend allowlist. Empty means all known backend identifiers.
    pub allowed_backends: Vec<BackendId>,
}

impl Default for EnvelopeValidationPolicy {
    fn default() -> Self {
        Self {
            protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
            max_proof_bytes: DEFAULT_MAX_PROOF_BYTES,
            max_signature_bytes: DEFAULT_MAX_SIGNATURE_BYTES,
            max_envelope_bytes: DEFAULT_MAX_ENVELOPE_BYTES,
            max_domain_tag_bytes: DEFAULT_MAX_DOMAIN_TAG_BYTES,
            max_age_seconds: 60 * 60,
            max_future_skew_seconds: 5 * 60,
            // 1 GJ is intentionally generous; deployments should lower this.
            max_energy_millijoules: 1_000_000_000_000,
            require_signature: true,
            allowed_backends: Vec::new(),
        }
    }
}

/// Validate an authenticated proof envelope without claiming cryptographic validity.
///
/// `now_unix_seconds` is supplied by the caller so the function remains
/// deterministic and suitable for consensus-sensitive runtimes.
pub fn validate_authenticated_proof_envelope(
    proof: &AuthenticatedProof,
    policy: &EnvelopeValidationPolicy,
    now_unix_seconds: u64,
) -> ZkpResult<()> {
    if proof.metadata.protocol_version != policy.protocol_version {
        return Err(ZkpError::InvalidProofFormat(format!(
            "unsupported authenticated-proof protocol version: expected {}, got {}",
            policy.protocol_version, proof.metadata.protocol_version
        )));
    }

    if proof.proof.is_empty() {
        return Err(ZkpError::InvalidProofFormat("proof payload is empty".into()));
    }
    if proof.proof.len() > policy.max_proof_bytes {
        return Err(ZkpError::ProofTooLarge {
            size: proof.proof.len(),
            limit: policy.max_proof_bytes,
        });
    }

    if policy.require_signature && proof.signature.is_empty() {
        return Err(ZkpError::SignatureInvalid);
    }
    if proof.signature.len() > policy.max_signature_bytes {
        return Err(ZkpError::ProofTooLarge {
            size: proof.signature.len(),
            limit: policy.max_signature_bytes,
        });
    }
    if proof.size_bytes() > policy.max_envelope_bytes {
        return Err(ZkpError::ProofTooLarge {
            size: proof.size_bytes(),
            limit: policy.max_envelope_bytes,
        });
    }

    let domain = proof.metadata.domain_tag.as_bytes();
    if domain.is_empty() || domain.len() > policy.max_domain_tag_bytes {
        return Err(ZkpError::InvalidProofFormat(format!(
            "invalid domain-tag length: {}",
            domain.len()
        )));
    }
    if !proof.metadata.domain_tag.as_str().starts_with("ZTML:") {
        return Err(ZkpError::InvalidProofFormat(
            "domain tag must use the ZTML namespace".into(),
        ));
    }

    if proof.metadata.client_id == [0; 32] {
        return Err(ZkpError::InvalidProofFormat("client_id is zero".into()));
    }
    if proof.metadata.nonce == [0; 32] {
        return Err(ZkpError::InvalidProofFormat("nonce is zero".into()));
    }
    if proof.public_inputs_hash == [0; 32] {
        return Err(ZkpError::InvalidProofFormat(
            "public_inputs_hash is zero".into(),
        ));
    }

    let latest_allowed = now_unix_seconds.saturating_add(policy.max_future_skew_seconds);
    if proof.metadata.timestamp > latest_allowed {
        return Err(ZkpError::ProofExpired(format!(
            "timestamp {} is more than {} seconds in the future",
            proof.metadata.timestamp, policy.max_future_skew_seconds
        )));
    }
    let oldest_allowed = now_unix_seconds.saturating_sub(policy.max_age_seconds);
    if proof.metadata.timestamp < oldest_allowed {
        return Err(ZkpError::ProofExpired(format!(
            "timestamp {} is older than the policy cutoff {}",
            proof.metadata.timestamp, oldest_allowed
        )));
    }

    if proof.energy_millijoules > policy.max_energy_millijoules {
        return Err(ZkpError::InsufficientSecurity {
            required: format!(
                "energy claim <= {} mJ",
                policy.max_energy_millijoules
            ),
            actual: format!("{} mJ", proof.energy_millijoules),
        });
    }

    if !policy.allowed_backends.is_empty()
        && !policy.allowed_backends.contains(&proof.metadata.backend)
    {
        return Err(ZkpError::BackendUnavailable(format!(
            "backend {} is not allowed by verifier policy",
            proof.metadata.backend.as_str()
        )));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::DomainTag;
    use crate::types::{AuthenticatedProof, ProofMetadata};

    const NOW: u64 = 1_700_000_000;

    fn valid_proof() -> AuthenticatedProof {
        AuthenticatedProof {
            proof: vec![1, 2, 3],
            signature: vec![4, 5, 6],
            metadata: ProofMetadata {
                domain_tag: DomainTag::new("Supply", "Provenance", 2),
                protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
                client_id: [0x11; 32],
                timestamp: NOW,
                nonce: [0x22; 32],
                backend: BackendId::Miden,
            },
            public_inputs_hash: [0x33; 32],
            energy_millijoules: 1000,
        }
    }

    #[test]
    fn valid_envelope_passes() {
        validate_authenticated_proof_envelope(
            &valid_proof(),
            &EnvelopeValidationPolicy::default(),
            NOW,
        )
        .unwrap();
    }

    #[test]
    fn rejects_legacy_protocol() {
        let mut proof = valid_proof();
        proof.metadata.protocol_version = 1;
        assert!(validate_authenticated_proof_envelope(
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());
    }

    #[test]
    fn rejects_zero_nonce_and_public_inputs() {
        let mut proof = valid_proof();
        proof.metadata.nonce = [0; 32];
        assert!(validate_authenticated_proof_envelope(
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());

        let mut proof = valid_proof();
        proof.public_inputs_hash = [0; 32];
        assert!(validate_authenticated_proof_envelope(
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());
    }

    #[test]
    fn rejects_expired_and_future_timestamps() {
        let policy = EnvelopeValidationPolicy::default();
        let mut proof = valid_proof();
        proof.metadata.timestamp = NOW - policy.max_age_seconds - 1;
        assert!(validate_authenticated_proof_envelope(&proof, &policy, NOW).is_err());

        proof.metadata.timestamp = NOW + policy.max_future_skew_seconds + 1;
        assert!(validate_authenticated_proof_envelope(&proof, &policy, NOW).is_err());
    }

    #[test]
    fn enforces_backend_allowlist() {
        let mut policy = EnvelopeValidationPolicy::default();
        policy.allowed_backends = vec![BackendId::Winterfell];
        assert!(validate_authenticated_proof_envelope(&valid_proof(), &policy, NOW).is_err());
    }

    #[test]
    fn unsigned_mode_must_be_explicit() {
        let mut proof = valid_proof();
        proof.signature.clear();
        assert!(validate_authenticated_proof_envelope(
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());

        let mut policy = EnvelopeValidationPolicy::default();
        policy.require_signature = false;
        validate_authenticated_proof_envelope(&proof, &policy, NOW).unwrap();
    }
}
