// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Typed, domain-separated statements for privacy-preserving supply-chain proofs.
//!
//! This module defines protocol commitments and validation rules. It deliberately
//! does not claim that any proof backend implements every statement kind.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::domain::DomainTag;
use crate::error::{ZkpError, ZkpResult};
use crate::types::AuthenticatedProof;
use crate::validation::{
    EnvelopeValidationPolicy, validate_authenticated_proof_envelope,
};

/// Version of the supply-chain statement commitment format.
pub const SUPPLY_PROOF_STATEMENT_VERSION: u32 = 1;
const SUPPLY_STATEMENT_DOMAIN: &[u8] = b"MYCELIX:SupplyProofStatement:v1";

/// Privacy-preserving statements supported by the supply-chain protocol layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SupplyProofKind {
    QuantityInRange,
    TemperatureWithinPolicy,
    CarbonBelowThreshold,
    JurisdictionOfOrigin,
    AuthorizedSupplierMembership,
    UniqueCertificationClaim,
}

impl SupplyProofKind {
    /// Stable protocol identifier. Values must never be reordered or reused.
    pub const fn wire_id(self) -> u8 {
        match self {
            Self::QuantityInRange => 1,
            Self::TemperatureWithinPolicy => 2,
            Self::CarbonBelowThreshold => 3,
            Self::JurisdictionOfOrigin => 4,
            Self::AuthorizedSupplierMembership => 5,
            Self::UniqueCertificationClaim => 6,
        }
    }

    /// Exact domain tag required for this proof statement.
    pub fn domain_tag(self) -> DomainTag {
        let proof_type = match self {
            Self::QuantityInRange => "QuantityRange",
            Self::TemperatureWithinPolicy => "TemperaturePolicy",
            Self::CarbonBelowThreshold => "CarbonThreshold",
            Self::JurisdictionOfOrigin => "JurisdictionOrigin",
            Self::AuthorizedSupplierMembership => "AuthorizedSupplierMembership",
            Self::UniqueCertificationClaim => "UniqueCertificationClaim",
        };
        DomainTag::new("Supply", proof_type, SUPPLY_PROOF_STATEMENT_VERSION)
    }
}

/// Public statement bound to a private supply-chain proof.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SupplyProofStatement {
    pub version: u32,
    pub kind: SupplyProofKind,
    /// SHA-256 identifier for a batch, shipment, product, or credential subject.
    pub subject_id: [u8; 32],
    /// Hash of the exact policy evaluated by the circuit.
    pub policy_hash: [u8; 32],
    /// Commitment to sensor data, credentials, or other private evidence.
    pub evidence_commitment: [u8; 32],
    /// Hash of public thresholds, jurisdiction set, or membership root.
    pub public_values_hash: [u8; 32],
    /// Identity expected to authenticate the proof envelope.
    pub issuer_id: [u8; 32],
    pub valid_from: u64,
    pub valid_until: u64,
    /// Must match the authenticated envelope nonce.
    pub nonce: [u8; 32],
}

impl SupplyProofStatement {
    /// Deterministic public-input commitment used by [`AuthenticatedProof`].
    pub fn commitment(&self) -> [u8; 32] {
        let mut hasher = Sha256::new();
        hasher.update(SUPPLY_STATEMENT_DOMAIN);
        hasher.update(self.version.to_le_bytes());
        hasher.update([self.kind.wire_id()]);
        hasher.update(self.subject_id);
        hasher.update(self.policy_hash);
        hasher.update(self.evidence_commitment);
        hasher.update(self.public_values_hash);
        hasher.update(self.issuer_id);
        hasher.update(self.valid_from.to_le_bytes());
        hasher.update(self.valid_until.to_le_bytes());
        hasher.update(self.nonce);
        hasher.finalize().into()
    }
}

/// Validate a supply statement independently of any proof bytes.
pub fn validate_supply_statement(
    statement: &SupplyProofStatement,
    now_unix_seconds: u64,
) -> ZkpResult<()> {
    if statement.version != SUPPLY_PROOF_STATEMENT_VERSION {
        return Err(ZkpError::InvalidProofFormat(format!(
            "unsupported supply statement version: {}",
            statement.version
        )));
    }
    if statement.subject_id == [0; 32]
        || statement.policy_hash == [0; 32]
        || statement.evidence_commitment == [0; 32]
        || statement.public_values_hash == [0; 32]
        || statement.issuer_id == [0; 32]
        || statement.nonce == [0; 32]
    {
        return Err(ZkpError::InvalidProofFormat(
            "supply statement contains a zero security commitment".into(),
        ));
    }
    if statement.valid_until < statement.valid_from {
        return Err(ZkpError::InvalidProofFormat(
            "supply statement validity interval is inverted".into(),
        ));
    }
    if now_unix_seconds < statement.valid_from {
        return Err(ZkpError::ProofExpired(format!(
            "supply statement is not valid before {}",
            statement.valid_from
        )));
    }
    if now_unix_seconds > statement.valid_until {
        return Err(ZkpError::ProofExpired(format!(
            "supply statement expired at {}",
            statement.valid_until
        )));
    }
    Ok(())
}

/// Validate exact binding between a supply statement and authenticated proof.
///
/// Passing this function does not verify the backend proof or signature. Callers
/// must perform those operations after structural and policy validation succeeds.
pub fn validate_supply_proof_envelope(
    statement: &SupplyProofStatement,
    proof: &AuthenticatedProof,
    policy: &EnvelopeValidationPolicy,
    now_unix_seconds: u64,
) -> ZkpResult<()> {
    validate_supply_statement(statement, now_unix_seconds)?;
    validate_authenticated_proof_envelope(proof, policy, now_unix_seconds)?;

    let expected_domain = statement.kind.domain_tag();
    if proof.metadata.domain_tag != expected_domain {
        return Err(ZkpError::DomainTagMismatch {
            expected: expected_domain.as_str().to_string(),
            actual: proof.metadata.domain_tag.as_str().to_string(),
        });
    }
    if proof.metadata.client_id != statement.issuer_id {
        return Err(ZkpError::ClientIdMismatch);
    }
    if proof.metadata.nonce != statement.nonce {
        return Err(ZkpError::InvalidProofFormat(
            "supply statement nonce does not match envelope nonce".into(),
        ));
    }
    if proof.public_inputs_hash != statement.commitment() {
        return Err(ZkpError::VerificationFailed(
            "authenticated proof is not bound to the supplied statement".into(),
        ));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{
        AUTHENTICATED_PROOF_PROTOCOL_VERSION, BackendId, ProofMetadata,
    };

    const NOW: u64 = 1_700_000_000;

    fn statement() -> SupplyProofStatement {
        SupplyProofStatement {
            version: SUPPLY_PROOF_STATEMENT_VERSION,
            kind: SupplyProofKind::TemperatureWithinPolicy,
            subject_id: [0x11; 32],
            policy_hash: [0x22; 32],
            evidence_commitment: [0x33; 32],
            public_values_hash: [0x44; 32],
            issuer_id: [0x55; 32],
            valid_from: NOW - 10,
            valid_until: NOW + 10,
            nonce: [0x66; 32],
        }
    }

    fn envelope(statement: &SupplyProofStatement) -> AuthenticatedProof {
        AuthenticatedProof {
            proof: vec![1, 2, 3],
            signature: vec![4, 5, 6],
            metadata: ProofMetadata {
                domain_tag: statement.kind.domain_tag(),
                protocol_version: AUTHENTICATED_PROOF_PROTOCOL_VERSION,
                client_id: statement.issuer_id,
                timestamp: NOW,
                nonce: statement.nonce,
                backend: BackendId::Miden,
            },
            public_inputs_hash: statement.commitment(),
            energy_millijoules: 100,
        }
    }

    #[test]
    fn statement_commitment_is_deterministic_and_kind_bound() {
        let mut statement = statement();
        let original = statement.commitment();
        assert_eq!(original, statement.commitment());
        statement.kind = SupplyProofKind::CarbonBelowThreshold;
        assert_ne!(original, statement.commitment());
    }

    #[test]
    fn valid_supply_envelope_passes_structural_binding() {
        let statement = statement();
        validate_supply_proof_envelope(
            &statement,
            &envelope(&statement),
            &EnvelopeValidationPolicy::default(),
            NOW,
        )
        .unwrap();
    }

    #[test]
    fn rejects_cross_kind_domain_replay() {
        let statement = statement();
        let mut proof = envelope(&statement);
        proof.metadata.domain_tag = SupplyProofKind::CarbonBelowThreshold.domain_tag();
        assert!(validate_supply_proof_envelope(
            &statement,
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());
    }

    #[test]
    fn rejects_mutated_policy_or_nonce() {
        let statement = statement();
        let proof = envelope(&statement);

        let mut mutated = statement.clone();
        mutated.policy_hash[0] ^= 1;
        assert!(validate_supply_proof_envelope(
            &mutated,
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());

        let mut mutated = statement.clone();
        mutated.nonce[0] ^= 1;
        assert!(validate_supply_proof_envelope(
            &mutated,
            &proof,
            &EnvelopeValidationPolicy::default(),
            NOW
        )
        .is_err());
    }
}
