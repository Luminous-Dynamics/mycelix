// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Backend-neutral cryptographic statement theorem for Identity V2 K-vector proofs.
//!
//! This crate freezes *what must be proven* before any Winterfell, RISC0, Miden,
//! or other backend is selected. It is a pure host-side relation oracle, not a ZK
//! verifier. A qualified backend must prove the same relation without revealing the
//! private witness.
//!
//! The relation is:
//!
//! 1. the private witness contains exactly the canonical 8 fixed-point components;
//! 2. every component and the weighted score satisfy the #216 authority theorem;
//! 3. the private witness opens the exact public K-vector commitment;
//! 4. that commitment and public score range are the ones hashed into #210's exact
//!    fulfillment ID;
//! 5. the resulting public proof statement has one deterministic digest suitable
//!    for signed verification records.

#![forbid(unsafe_code)]

use mycelix_attestation_fulfillment_policy::{
    derive_attestation_fulfillment_id_v2, AttestationFulfillmentStatementV2,
    FULFILLMENT_ID_LEN_V2, K_VECTOR_COMMITMENT_LEN_V2,
};
use mycelix_trust_authority_policy::{
    validate_kvector_witness_score_range_v2, weighted_trust_numerator_v2,
    K_VECTOR_DIMENSIONS_V2, K_VECTOR_WEIGHTS_PERCENT_V2, K_VECTOR_WEIGHT_SUM_V2,
    TRUST_SCORE_SCALE_V2,
};
use sha2::{Digest, Sha256};

pub const K_VECTOR_BLINDING_LEN_V2: usize = 32;
pub const K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2: usize = 32;
pub const K_VECTOR_COMMITMENT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-commitment:v2\0";
pub const K_VECTOR_PROOF_STATEMENT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:kvector-proof-statement:v2\0";
pub const K_VECTOR_COMMITMENT_SCHEME_ID_V2: &[u8] = b"sha256-blinded-model-bound-v2";

/// Private witness for the canonical Identity V2 trust proof relation.
///
/// The blinding value is part of the commitment opening and remains private.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KVectorProofWitnessV2 {
    pub components_scaled: [u64; K_VECTOR_DIMENSIONS_V2],
    pub blinding: [u8; K_VECTOR_BLINDING_LEN_V2],
}

/// Canonical public statement a proof backend must verify.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KVectorProofPublicStatementV2 {
    pub fulfillment_id: [u8; FULFILLMENT_ID_LEN_V2],
    pub kvector_commitment: [u8; K_VECTOR_COMMITMENT_LEN_V2],
    pub trust_score_lower_scaled: u64,
    pub trust_score_upper_scaled: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KVectorProofStatementErrorV2 {
    FulfillmentStatementInvalid,
    CommitmentLengthInvalid,
    WitnessComponentInvalid,
    BlindingAllZero,
    WitnessOutsidePublicRange,
    CommitmentMismatch,
}

fn copy_commitment_v2(
    commitment: &[u8],
) -> Result<[u8; K_VECTOR_COMMITMENT_LEN_V2], KVectorProofStatementErrorV2> {
    commitment
        .try_into()
        .map_err(|_| KVectorProofStatementErrorV2::CommitmentLengthInvalid)
}

/// Derive the canonical blinded commitment for one private V2 K-vector witness.
///
/// The model itself is bound into the transcript: dimensions, scale, weight sum,
/// exact ordered weights, ordered components, then 32-byte blinding. This prevents
/// a future backend from reinterpreting the same component bytes under a different
/// K-vector model while retaining the same commitment.
pub fn derive_kvector_commitment_v2(
    witness: KVectorProofWitnessV2,
) -> Result<[u8; K_VECTOR_COMMITMENT_LEN_V2], KVectorProofStatementErrorV2> {
    weighted_trust_numerator_v2(witness.components_scaled)
        .map_err(|_| KVectorProofStatementErrorV2::WitnessComponentInvalid)?;

    if witness.blinding.iter().all(|byte| *byte == 0) {
        return Err(KVectorProofStatementErrorV2::BlindingAllZero);
    }

    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_COMMITMENT_DOMAIN_V2);

    hasher.update([0x01]);
    hasher.update((K_VECTOR_DIMENSIONS_V2 as u16).to_be_bytes());

    hasher.update([0x02]);
    hasher.update(TRUST_SCORE_SCALE_V2.to_be_bytes());

    hasher.update([0x03]);
    hasher.update(K_VECTOR_WEIGHT_SUM_V2.to_be_bytes());

    hasher.update([0x04]);
    for (index, weight) in K_VECTOR_WEIGHTS_PERCENT_V2.into_iter().enumerate() {
        hasher.update([index as u8]);
        hasher.update(weight.to_be_bytes());
    }

    hasher.update([0x05]);
    for (index, component) in witness.components_scaled.into_iter().enumerate() {
        hasher.update([index as u8]);
        hasher.update(component.to_be_bytes());
    }

    hasher.update([0x06]);
    hasher.update(witness.blinding);

    let digest = hasher.finalize();
    let mut commitment = [0u8; K_VECTOR_COMMITMENT_LEN_V2];
    commitment.copy_from_slice(&digest);
    Ok(commitment)
}

/// Project #210's semantic fulfillment statement into canonical proof public inputs.
pub fn derive_kvector_proof_public_statement_v2(
    fulfillment: AttestationFulfillmentStatementV2<'_>,
) -> Result<KVectorProofPublicStatementV2, KVectorProofStatementErrorV2> {
    let fulfillment_id = derive_attestation_fulfillment_id_v2(fulfillment)
        .map_err(|_| KVectorProofStatementErrorV2::FulfillmentStatementInvalid)?;
    let kvector_commitment = copy_commitment_v2(fulfillment.kvector_commitment)?;

    Ok(KVectorProofPublicStatementV2 {
        fulfillment_id,
        kvector_commitment,
        trust_score_lower_scaled: fulfillment.trust_score_lower_scaled,
        trust_score_upper_scaled: fulfillment.trust_score_upper_scaled,
    })
}

/// Derive the canonical digest signed verification records should bind to.
///
/// The digest is independent of proof bytes/backend execution. It binds the exact
/// fulfillment effect, commitment, public range, commitment scheme, and canonical
/// model parameters.
pub fn derive_kvector_proof_statement_digest_v2(
    statement: KVectorProofPublicStatementV2,
) -> [u8; K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2] {
    let mut hasher = Sha256::new();
    hasher.update(K_VECTOR_PROOF_STATEMENT_DOMAIN_V2);

    hasher.update([0x01]);
    hasher.update(statement.fulfillment_id);

    hasher.update([0x02]);
    hasher.update(statement.kvector_commitment);

    hasher.update([0x03]);
    hasher.update(statement.trust_score_lower_scaled.to_be_bytes());

    hasher.update([0x04]);
    hasher.update(statement.trust_score_upper_scaled.to_be_bytes());

    hasher.update([0x05]);
    hasher.update((K_VECTOR_COMMITMENT_SCHEME_ID_V2.len() as u16).to_be_bytes());
    hasher.update(K_VECTOR_COMMITMENT_SCHEME_ID_V2);

    hasher.update([0x06]);
    hasher.update((K_VECTOR_DIMENSIONS_V2 as u16).to_be_bytes());

    hasher.update([0x07]);
    hasher.update(TRUST_SCORE_SCALE_V2.to_be_bytes());

    hasher.update([0x08]);
    for (index, weight) in K_VECTOR_WEIGHTS_PERCENT_V2.into_iter().enumerate() {
        hasher.update([index as u8]);
        hasher.update(weight.to_be_bytes());
    }

    hasher.update([0x09]);
    hasher.update(K_VECTOR_WEIGHT_SUM_V2.to_be_bytes());

    let digest = hasher.finalize();
    let mut out = [0u8; K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    out
}

/// Evaluate the exact private/public relation a future ZK backend must prove.
///
/// This function is a deterministic protocol oracle for tests and backend
/// qualification. Calling it locally is **not** proof verification.
pub fn evaluate_kvector_proof_relation_v2(
    fulfillment: AttestationFulfillmentStatementV2<'_>,
    witness: KVectorProofWitnessV2,
) -> Result<[u8; K_VECTOR_PROOF_STATEMENT_DIGEST_LEN_V2], KVectorProofStatementErrorV2> {
    let statement = derive_kvector_proof_public_statement_v2(fulfillment)?;

    validate_kvector_witness_score_range_v2(
        witness.components_scaled,
        statement.trust_score_lower_scaled,
        statement.trust_score_upper_scaled,
    )
    .map_err(|error| match error {
        mycelix_trust_authority_policy::TrustAuthorityPolicyErrorV2::ComponentOutOfRange {
            ..
        } => KVectorProofStatementErrorV2::WitnessComponentInvalid,
        _ => KVectorProofStatementErrorV2::WitnessOutsidePublicRange,
    })?;

    let commitment = derive_kvector_commitment_v2(witness)?;
    if commitment != statement.kvector_commitment {
        return Err(KVectorProofStatementErrorV2::CommitmentMismatch);
    }

    Ok(derive_kvector_proof_statement_digest_v2(statement))
}

#[cfg(test)]
mod tests {
    use super::*;

    const SUBJECT: &str = "did:mycelix:subject";

    fn root_raw39() -> [u8; 39] {
        let mut root = [0u8; 39];
        for (index, byte) in root.iter_mut().enumerate() {
            *byte = index as u8;
        }
        root
    }

    fn witness() -> KVectorProofWitnessV2 {
        KVectorProofWitnessV2 {
            components_scaled: [
                800_000, 600_000, 900_000, 700_000, 500_000, 400_000, 600_000, 500_000,
            ],
            blinding: [0x42; K_VECTOR_BLINDING_LEN_V2],
        }
    }

    fn fulfillment<'a>(
        root: &'a [u8],
        commitment: &'a [u8],
    ) -> AttestationFulfillmentStatementV2<'a> {
        AttestationFulfillmentStatementV2 {
            request_root_raw39: root,
            subject_did: SUBJECT,
            kvector_commitment: commitment,
            trust_score_lower_scaled: 600_000,
            trust_score_upper_scaled: 700_000,
            credential_expires_at_micros: Some(1_700_000_000_000_000),
        }
    }

    #[test]
    fn commitment_vector_is_frozen() {
        let actual = derive_kvector_commitment_v2(witness()).unwrap();
        let expected = [
            0x83, 0x44, 0xa1, 0x26, 0xd8, 0xb0, 0x0e, 0xdb, 0x36, 0xc4, 0x25, 0x3e,
            0x88, 0x1d, 0x24, 0x8f, 0x3b, 0x1f, 0xb1, 0x58, 0x70, 0xed, 0xc2, 0x26,
            0x14, 0x80, 0x18, 0x20, 0x73, 0xa6, 0x36, 0x22,
        ];
        assert_eq!(actual, expected);
    }

    #[test]
    fn public_statement_and_digest_vectors_are_frozen() {
        let root = root_raw39();
        let commitment = derive_kvector_commitment_v2(witness()).unwrap();
        let public = derive_kvector_proof_public_statement_v2(fulfillment(&root, &commitment))
            .unwrap();
        assert_eq!(
            public.fulfillment_id,
            [
                0x33, 0x29, 0x24, 0x63, 0x4f, 0x10, 0x32, 0x84, 0xb2, 0x21, 0x13, 0xb3,
                0x05, 0x6b, 0x59, 0xe9, 0x24, 0x62, 0x68, 0x80, 0x05, 0x01, 0x1a, 0x40,
                0xe5, 0x6f, 0xd0, 0x6d, 0xc1, 0x23, 0xac, 0x6f,
            ]
        );
        assert_eq!(
            derive_kvector_proof_statement_digest_v2(public),
            [
                0x92, 0x64, 0x7e, 0xe2, 0x0f, 0x3b, 0xdf, 0x02, 0x5d, 0x20, 0x96, 0xb6,
                0x64, 0xca, 0x1d, 0x0d, 0x92, 0x41, 0xca, 0x7c, 0x47, 0x8c, 0xb7, 0xfb,
                0xdf, 0xe6, 0x74, 0xc2, 0x9d, 0xf6, 0x0a, 0x62,
            ]
        );
    }

    #[test]
    fn exact_relation_accepts_matching_hidden_witness() {
        let root = root_raw39();
        let witness = witness();
        let commitment = derive_kvector_commitment_v2(witness).unwrap();
        assert_eq!(
            evaluate_kvector_proof_relation_v2(fulfillment(&root, &commitment), witness),
            Ok([
                0x92, 0x64, 0x7e, 0xe2, 0x0f, 0x3b, 0xdf, 0x02, 0x5d, 0x20, 0x96, 0xb6,
                0x64, 0xca, 0x1d, 0x0d, 0x92, 0x41, 0xca, 0x7c, 0x47, 0x8c, 0xb7, 0xfb,
                0xdf, 0xe6, 0x74, 0xc2, 0x9d, 0xf6, 0x0a, 0x62,
            ])
        );
    }

    #[test]
    fn commitment_binds_every_component_and_blinding() {
        let base = witness();
        let base_commitment = derive_kvector_commitment_v2(base).unwrap();

        for index in 0..K_VECTOR_DIMENSIONS_V2 {
            let mut changed = base;
            changed.components_scaled[index] ^= 1;
            assert_ne!(derive_kvector_commitment_v2(changed).unwrap(), base_commitment);
        }

        let mut changed_blinding = base;
        changed_blinding.blinding[7] ^= 1;
        assert_ne!(
            derive_kvector_commitment_v2(changed_blinding).unwrap(),
            base_commitment
        );
    }

    #[test]
    fn zero_blinding_and_out_of_range_components_fail_closed() {
        let mut zero_blinding = witness();
        zero_blinding.blinding = [0; K_VECTOR_BLINDING_LEN_V2];
        assert_eq!(
            derive_kvector_commitment_v2(zero_blinding),
            Err(KVectorProofStatementErrorV2::BlindingAllZero)
        );

        let mut invalid_component = witness();
        invalid_component.components_scaled[2] = TRUST_SCORE_SCALE_V2 + 1;
        assert_eq!(
            derive_kvector_commitment_v2(invalid_component),
            Err(KVectorProofStatementErrorV2::WitnessComponentInvalid)
        );
    }

    #[test]
    fn relation_rejects_commitment_and_range_mismatch() {
        let root = root_raw39();
        let witness = witness();
        let commitment = derive_kvector_commitment_v2(witness).unwrap();

        let wrong_commitment = [0xAA; K_VECTOR_COMMITMENT_LEN_V2];
        assert_eq!(
            evaluate_kvector_proof_relation_v2(fulfillment(&root, &wrong_commitment), witness),
            Err(KVectorProofStatementErrorV2::CommitmentMismatch)
        );

        let mut too_high_range = fulfillment(&root, &commitment);
        too_high_range.trust_score_lower_scaled = 700_000;
        too_high_range.trust_score_upper_scaled = 800_000;
        assert_eq!(
            evaluate_kvector_proof_relation_v2(too_high_range, witness),
            Err(KVectorProofStatementErrorV2::WitnessOutsidePublicRange)
        );
    }

    #[test]
    fn statement_digest_changes_when_public_semantics_change() {
        let root = root_raw39();
        let commitment = derive_kvector_commitment_v2(witness()).unwrap();
        let base = fulfillment(&root, &commitment);
        let base_public = derive_kvector_proof_public_statement_v2(base).unwrap();
        let base_digest = derive_kvector_proof_statement_digest_v2(base_public);

        let mut changed_range = base;
        changed_range.trust_score_lower_scaled = 610_000;
        let changed_public = derive_kvector_proof_public_statement_v2(changed_range).unwrap();
        assert_ne!(
            derive_kvector_proof_statement_digest_v2(changed_public),
            base_digest
        );

        let mut changed_expiry = base;
        changed_expiry.credential_expires_at_micros = None;
        let changed_public = derive_kvector_proof_public_statement_v2(changed_expiry).unwrap();
        assert_ne!(
            derive_kvector_proof_statement_digest_v2(changed_public),
            base_digest
        );
    }
}
