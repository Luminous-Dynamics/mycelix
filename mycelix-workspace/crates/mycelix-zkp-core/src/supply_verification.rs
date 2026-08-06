// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Append-only verification records for supply-chain proof results.
//!
//! A record is evidence about a verifier execution. It is not a substitute for
//! independently verifying the referenced proof and verifier signature.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::error::{ZkpError, ZkpResult};
use crate::supply::SupplyProofStatement;
use crate::types::{AuthenticatedProof, BackendId};

const VERIFICATION_RECORD_DOMAIN: &[u8] = b"MYCELIX:SupplyVerificationRecord:v1";
pub const SUPPLY_VERIFICATION_RECORD_VERSION: u32 = 1;
pub const MAX_VERIFIER_TEXT_BYTES: usize = 128;
pub const MAX_VERIFIER_SIGNATURE_BYTES: usize = 16 * 1024;

/// Explicit outcome of a verification attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupplyVerificationStatus {
    Valid,
    Invalid,
    Unsupported,
    Expired,
}

impl SupplyVerificationStatus {
    pub const fn wire_id(self) -> u8 {
        match self {
            Self::Valid => 1,
            Self::Invalid => 2,
            Self::Unsupported => 3,
            Self::Expired => 4,
        }
    }
}

/// Append-only attestation describing one verifier execution.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SupplyVerificationRecord {
    pub version: u32,
    pub statement_commitment: [u8; 32],
    pub proof_hash: [u8; 32],
    pub verifier_id: [u8; 32],
    pub verifier_implementation: String,
    pub verifier_version: String,
    pub verification_policy_hash: [u8; 32],
    pub backend: BackendId,
    pub status: SupplyVerificationStatus,
    pub verified_at: u64,
    /// Signature over [`Self::construct_signed_message`].
    pub verifier_signature: Vec<u8>,
}

impl SupplyVerificationRecord {
    /// Construct an unsigned record from the exact statement and proof inspected.
    #[allow(clippy::too_many_arguments)]
    pub fn unsigned(
        statement: &SupplyProofStatement,
        proof: &AuthenticatedProof,
        verifier_id: [u8; 32],
        verifier_implementation: impl Into<String>,
        verifier_version: impl Into<String>,
        verification_policy_hash: [u8; 32],
        status: SupplyVerificationStatus,
        verified_at: u64,
    ) -> Self {
        Self {
            version: SUPPLY_VERIFICATION_RECORD_VERSION,
            statement_commitment: statement.commitment(),
            proof_hash: Sha256::digest(&proof.proof).into(),
            verifier_id,
            verifier_implementation: verifier_implementation.into(),
            verifier_version: verifier_version.into(),
            verification_policy_hash,
            backend: proof.metadata.backend,
            status,
            verified_at,
            verifier_signature: Vec::new(),
        }
    }

    /// Canonical digest authenticated by the verifier identity.
    pub fn construct_signed_message(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        let implementation = self.verifier_implementation.as_bytes();
        let version = self.verifier_version.as_bytes();

        hasher.update(VERIFICATION_RECORD_DOMAIN);
        hasher.update(self.version.to_le_bytes());
        hasher.update(self.statement_commitment);
        hasher.update(self.proof_hash);
        hasher.update(self.verifier_id);
        hasher.update((implementation.len() as u32).to_le_bytes());
        hasher.update(implementation);
        hasher.update((version.len() as u32).to_le_bytes());
        hasher.update(version);
        hasher.update(self.verification_policy_hash);
        hasher.update([self.backend.wire_id()]);
        hasher.update([self.status.wire_id()]);
        hasher.update(self.verified_at.to_le_bytes());
        hasher.finalize().into()
    }
}

/// Validate record structure without claiming its signature is authentic.
pub fn validate_supply_verification_record(
    record: &SupplyVerificationRecord,
    require_signature: bool,
) -> ZkpResult<()> {
    if record.version != SUPPLY_VERIFICATION_RECORD_VERSION {
        return Err(ZkpError::InvalidProofFormat(format!(
            "unsupported supply verification record version: {}",
            record.version
        )));
    }
    if record.statement_commitment == [0; 32]
        || record.proof_hash == [0; 32]
        || record.verifier_id == [0; 32]
        || record.verification_policy_hash == [0; 32]
    {
        return Err(ZkpError::InvalidProofFormat(
            "verification record contains a zero security commitment".into(),
        ));
    }
    if record.verifier_implementation.is_empty()
        || record.verifier_implementation.len() > MAX_VERIFIER_TEXT_BYTES
        || record.verifier_version.is_empty()
        || record.verifier_version.len() > MAX_VERIFIER_TEXT_BYTES
    {
        return Err(ZkpError::InvalidProofFormat(
            "invalid verifier implementation or version text".into(),
        ));
    }
    if require_signature && record.verifier_signature.is_empty() {
        return Err(ZkpError::SignatureInvalid);
    }
    if record.verifier_signature.len() > MAX_VERIFIER_SIGNATURE_BYTES {
        return Err(ZkpError::ProofTooLarge {
            size: record.verifier_signature.len(),
            limit: MAX_VERIFIER_SIGNATURE_BYTES,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::DomainTag;
    use crate::supply::{SUPPLY_PROOF_STATEMENT_VERSION, SupplyProofKind};
    use crate::types::{
        AUTHENTICATED_PROOF_PROTOCOL_VERSION, ProofMetadata,
    };

    fn fixture() -> (SupplyProofStatement, AuthenticatedProof) {
        let statement = SupplyProofStatement {
            version: SUPPLY_PROOF_STATEMENT_VERSION,
            kind: SupplyProofKind::JurisdictionOfOrigin,
            subject_id: [1; 32],
            policy_hash: [2; 32],
            evidence_commitment: [3; 32],
            public_values_hash: [4; 32],
            issuer_id: [5; 32],
            valid_from: 100,
            valid_until: 200,
            nonce: [6; 32],
        };
        let proof = AuthenticatedProof {
            proof: vec![7; 64],
            signature: vec![8],
            metadata: ProofMetadata {
                domain_tag: DomainTag::new("Supply", "JurisdictionOrigin", 1),
                protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
                client_id: statement.issuer_id,
                timestamp: 150,
                nonce: statement.nonce,
                backend: BackendId::Miden,
            },
            public_inputs_hash: statement.commitment(),
            energy_millijoules: 10,
        };
        (statement, proof)
    }

    #[test]
    fn record_binds_status_backend_and_policy() {
        let (statement, proof) = fixture();
        let mut record = SupplyVerificationRecord::unsigned(
            &statement,
            &proof,
            [9; 32],
            "mycelix-miden-verifier",
            "0.23.5",
            [10; 32],
            SupplyVerificationStatus::Valid,
            160,
        );
        let original = record.construct_signed_message();
        record.status = SupplyVerificationStatus::Unsupported;
        assert_ne!(original, record.construct_signed_message());
        record.status = SupplyVerificationStatus::Valid;
        record.verification_policy_hash[0] ^= 1;
        assert_ne!(original, record.construct_signed_message());
    }

    #[test]
    fn signed_record_validation_is_explicit() {
        let (statement, proof) = fixture();
        let mut record = SupplyVerificationRecord::unsigned(
            &statement,
            &proof,
            [9; 32],
            "mycelix-miden-verifier",
            "0.23.5",
            [10; 32],
            SupplyVerificationStatus::Valid,
            160,
        );
        assert!(validate_supply_verification_record(&record, true).is_err());
        validate_supply_verification_record(&record, false).unwrap();
        record.verifier_signature = vec![11; 64];
        validate_supply_verification_record(&record, true).unwrap();
    }
}
