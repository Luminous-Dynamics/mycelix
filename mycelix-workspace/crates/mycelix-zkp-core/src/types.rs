// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Core types for the DASTARK proof system.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::domain::DomainTag;

/// Raw proof bytes from any backend.
pub type ProofBytes = Vec<u8>;

/// Current canonical authenticated-proof envelope version.
///
/// Version 2 binds the backend identifier and canonical integer energy accounting
/// into the signed digest. Version 1 envelopes must not be accepted as v2.
pub const AUTHENTICATED_PROOF_PROTOCOL_VERSION: u32 = 2;

/// Domain prefix for the canonical authenticated-proof signing transcript.
pub const AUTHENTICATED_PROOF_SIGNING_DOMAIN: &[u8] =
    b"MYCELIX:AuthenticatedProof:SignedEnvelope:v2";

/// Result of proof generation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProofResult {
    /// The proof bytes (backend-specific format).
    pub proof: ProofBytes,
    /// Which backend produced this proof.
    pub backend: BackendId,
    /// Proving time in milliseconds.
    pub proving_time_ms: u64,
    /// Proof size in bytes.
    pub proof_size: usize,
}

/// Result of proof verification.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VerificationResult {
    /// Whether the proof is valid.
    pub valid: bool,
    /// Verification time in milliseconds.
    pub verification_time_ms: u64,
    /// Human-readable message (empty on success).
    pub message: String,
}

impl VerificationResult {
    pub fn ok(verification_time_ms: u64) -> Self {
        Self {
            valid: true,
            verification_time_ms,
            message: String::new(),
        }
    }

    pub fn fail(message: impl Into<String>, verification_time_ms: u64) -> Self {
        Self {
            valid: false,
            verification_time_ms,
            message: message.into(),
        }
    }
}

/// Identifies which backend produced a proof.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BackendId {
    Risc0,
    Winterfell,
    Binius, // Reserved for a future backend
    Miden,
}

impl BackendId {
    pub fn as_str(&self) -> &'static str {
        match self {
            BackendId::Risc0 => "risc0",
            BackendId::Winterfell => "winterfell",
            BackendId::Binius => "binius",
            BackendId::Miden => "miden",
        }
    }

    /// Stable wire identifier used by signed envelopes.
    ///
    /// These values are protocol data and must never be reordered or reused.
    pub const fn wire_id(self) -> u8 {
        match self {
            BackendId::Risc0 => 1,
            BackendId::Winterfell => 2,
            BackendId::Binius => 3,
            BackendId::Miden => 4,
        }
    }
}

/// Metadata attached to every proof for audit and replay protection.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ProofMetadata {
    /// Domain separation tag.
    pub domain_tag: DomainTag,
    /// Protocol version.
    pub protocol_version: u32,
    /// Agent/client identity (SHA-256 of public key).
    pub client_id: [u8; 32],
    /// Timestamp (Unix seconds).
    pub timestamp: u64,
    /// Random nonce (32 bytes) for replay protection.
    pub nonce: [u8; 32],
    /// Which backend was used.
    pub backend: BackendId,
}

/// An authenticated proof: ZK proof + Dilithium5 PQ signature + metadata.
///
/// This is the canonical proof format passed to Holochain zomes for verification.
/// Total size: ~200-220KB proof + 4.6KB signature + ~200B metadata.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AuthenticatedProof {
    /// Backend-specific proof bytes. Cryptographic meaning is circuit-specific.
    pub proof: ProofBytes,
    /// CRYSTALS-Dilithium5 signature over the canonical v2 envelope transcript.
    /// Empty if Dilithium is not enabled.
    pub signature: Vec<u8>,
    /// Proof metadata (domain, timestamp, nonce, client_id, backend).
    pub metadata: ProofMetadata,
    /// SHA-256 of any application-specific public inputs.
    pub public_inputs_hash: [u8; 32],
    /// Energy consumed during proof generation, in integer millijoules.
    ///
    /// This field is an authenticated measurement claim. Verifiers must apply a
    /// separate attestation policy before treating it as hardware-verified.
    pub energy_millijoules: u64,
}

impl AuthenticatedProof {
    /// Construct the canonical message digest signed by the authentication key.
    ///
    /// The transcript uses a fixed domain prefix, length-prefixes the variable-size
    /// domain tag, and binds every security-relevant envelope field including the
    /// backend and integer energy measurement.
    pub fn construct_signed_message(&self) -> Vec<u8> {
        let mut hasher = Sha256::new();
        let domain = self.metadata.domain_tag.as_bytes();
        let domain_len = u32::try_from(domain.len()).unwrap_or(u32::MAX);

        hasher.update(AUTHENTICATED_PROOF_SIGNING_DOMAIN);
        hasher.update(domain_len.to_le_bytes());
        hasher.update(domain);
        hasher.update(self.metadata.protocol_version.to_le_bytes());
        hasher.update([self.metadata.backend.wire_id()]);
        hasher.update(self.metadata.client_id);
        hasher.update(self.metadata.timestamp.to_le_bytes());
        hasher.update(self.metadata.nonce);
        hasher.update(self.public_inputs_hash);
        hasher.update(Sha256::digest(&self.proof));
        hasher.update(self.energy_millijoules.to_le_bytes());
        hasher.finalize().to_vec()
    }

    /// Get total size in bytes.
    pub fn size_bytes(&self) -> usize {
        self.proof.len()
            + self.signature.len()
            + 32 // client_id
            + 32 // nonce
            + 32 // public_inputs_hash
            + 8  // timestamp
            + 4  // protocol_version
            + 1  // backend wire id
            + 8  // energy_millijoules
            + self.metadata.domain_tag.as_bytes().len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_verification_result_ok() {
        let r = VerificationResult::ok(5);
        assert!(r.valid);
        assert!(r.message.is_empty());
    }

    #[test]
    fn test_verification_result_fail() {
        let r = VerificationResult::fail("bad proof", 3);
        assert!(!r.valid);
        assert_eq!(r.message, "bad proof");
    }

    #[test]
    fn test_backend_id_str() {
        assert_eq!(BackendId::Risc0.as_str(), "risc0");
        assert_eq!(BackendId::Winterfell.as_str(), "winterfell");
        assert_eq!(BackendId::Binius.as_str(), "binius");
        assert_eq!(BackendId::Miden.as_str(), "miden");
        assert_eq!(BackendId::Miden.wire_id(), 4);
    }

    #[test]
    fn test_authenticated_proof_signed_message_deterministic() {
        let meta = ProofMetadata {
            domain_tag: DomainTag::new("Test", "Unit", 1),
            protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
            client_id: [0xAA; 32],
            timestamp: 1700000000,
            nonce: [0xBB; 32],
            backend: BackendId::Winterfell,
        };
        let proof = AuthenticatedProof {
            proof: vec![1, 2, 3, 4],
            signature: vec![],
            metadata: meta,
            public_inputs_hash: [0xCC; 32],
            energy_millijoules: 0,
        };
        let msg1 = proof.construct_signed_message();
        let msg2 = proof.construct_signed_message();
        assert_eq!(msg1, msg2, "signed message must be deterministic");
        assert_eq!(msg1.len(), 32, "signed message is SHA-256 = 32 bytes");
    }

    #[test]
    fn test_signed_message_binds_backend_and_energy() {
        let metadata = ProofMetadata {
            domain_tag: DomainTag::new("Test", "Binding", 2),
            protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
            client_id: [0x11; 32],
            timestamp: 1_700_000_000,
            nonce: [0x22; 32],
            backend: BackendId::Winterfell,
        };
        let mut proof = AuthenticatedProof {
            proof: vec![1, 2, 3],
            signature: vec![],
            metadata,
            public_inputs_hash: [0x33; 32],
            energy_millijoules: 12_345,
        };
        let original = proof.construct_signed_message();

        proof.metadata.backend = BackendId::Miden;
        assert_ne!(original, proof.construct_signed_message());

        proof.metadata.backend = BackendId::Winterfell;
        proof.energy_millijoules += 1;
        assert_ne!(original, proof.construct_signed_message());
    }
}
