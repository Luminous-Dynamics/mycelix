// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Convergent semantic theorem for Mycelix attestation fulfillment V2.
//!
//! This crate deliberately knows nothing about Holochain actions, DHT/network
//! completeness, entry serialization, credential creation, or proof-system
//! implementation details. Adapters provide canonical request-root bytes and
//! verification-record facts derived from stronger boundaries.
//!
//! Security split:
//!
//! - semantic fulfillment identity is deterministic and independent of execution;
//! - public trust-score bounds use the canonical fixed-point authority theorem;
//! - proof encoding is evidence, not semantic identity;
//! - a bare `verified: bool` is not accepted by semantic aggregation;
//! - exact proof/claim verification evidence must first be qualified into an opaque
//!   token bound to the proof digest and fulfillment ID;
//! - equivalent retries converge; distinct semantic claims conflict.

#![forbid(unsafe_code)]

use mycelix_attestation_request_root_policy::DID_MAX_LEN_V1;
use mycelix_trust_authority_policy::{
    guaranteed_trust_tier_v2, validate_score_range_v2, GuaranteedTrustTierV2,
};
use sha2::{Digest, Sha256};

pub const REQUEST_ROOT_RAW39_LEN_V2: usize = 39;
pub const K_VECTOR_COMMITMENT_LEN_V2: usize = 32;
pub const FULFILLMENT_ID_LEN_V2: usize = 32;
pub const SHA256_DIGEST_LEN_V2: usize = 32;
pub const VERIFICATION_BACKEND_ID_MAX_LEN_V2: usize = 128;
pub const ATTESTATION_FULFILLMENT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:attestation-fulfillment:v2\0";

/// Public semantic statement whose deterministic identity converges retries.
///
/// Trust-score bounds are canonical fixed-point integers scaled by the authority
/// theorem (`1_000_000 == 1.0`). The range proof itself is intentionally absent.
/// Different valid proof encodings for this exact statement are retry-equivalent
/// evidence, not different effects.
#[derive(Debug, Clone, Copy)]
pub struct AttestationFulfillmentStatementV2<'a> {
    /// Canonical Holochain ActionHash raw representation (`get_raw_39()`).
    pub request_root_raw39: &'a [u8],
    pub subject_did: &'a str,
    pub kvector_commitment: &'a [u8],
    pub trust_score_lower_scaled: u64,
    pub trust_score_upper_scaled: u64,
    /// Optional credential expiry selected by this fulfillment statement.
    pub credential_expires_at_micros: Option<i64>,
}

/// Durable fulfillment claim carrying evidence plus its deterministic semantic ID.
#[derive(Debug, Clone, Copy)]
pub struct AttestationFulfillmentClaimV2<'a> {
    pub statement: AttestationFulfillmentStatementV2<'a>,
    /// Proof evidence is required but is not hashed into `fulfillment_id`.
    pub range_proof: &'a [u8],
    pub fulfillment_id: &'a [u8],
}

/// Outcome asserted by an external signed verification-record resolver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttestationProofVerificationOutcomeV2 {
    Accepted,
    Rejected,
}

/// Facts extracted from a signed/current verification-record boundary.
#[derive(Debug, Clone, Copy)]
pub struct AttestationProofVerificationRecordFactsV2<'a> {
    pub fulfillment_id: &'a [u8],
    pub proof_sha256: &'a [u8],
    pub verifier_did: &'a str,
    pub backend_id: &'a str,
    pub verification_policy_sha256: &'a [u8],
    pub verification_record_sha256: &'a [u8],
    pub outcome: AttestationProofVerificationOutcomeV2,
    pub signature_validated: bool,
    pub policy_accepted: bool,
    pub policy_current: bool,
}

/// Opaque capability proving that verification-record facts were structurally
/// qualified against one exact claim/proof.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QualifiedAttestationProofVerificationV2 {
    fulfillment_id: [u8; FULFILLMENT_ID_LEN_V2],
    proof_sha256: [u8; SHA256_DIGEST_LEN_V2],
}

impl QualifiedAttestationProofVerificationV2 {
    pub fn fulfillment_id(&self) -> &[u8; FULFILLMENT_ID_LEN_V2] {
        &self.fulfillment_id
    }

    pub fn proof_sha256(&self) -> &[u8; SHA256_DIGEST_LEN_V2] {
        &self.proof_sha256
    }
}

/// One observed claim plus authoritative actor identity and a proof-bound qualified
/// verification token.
#[derive(Debug, Clone, Copy)]
pub struct AttestationFulfillmentObservationV2<'a> {
    pub claim: AttestationFulfillmentClaimV2<'a>,
    pub action_author_did: &'a str,
    pub verification: QualifiedAttestationProofVerificationV2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttestationFulfillmentErrorV2 {
    RequestRootLengthInvalid,
    RequestRootMismatch,
    SubjectDidInvalid,
    SubjectMismatch,
    ActionAuthorDidInvalid,
    SubjectActorRequired,
    CommitmentLengthInvalid,
    RangeProofEmpty,
    TrustScoreRangeInvalid,
    FulfillmentIdLengthInvalid,
    FulfillmentIdMismatch,
    VerificationFulfillmentIdLengthInvalid,
    VerificationFulfillmentIdMismatch,
    VerificationProofDigestLengthInvalid,
    VerificationProofDigestMismatch,
    VerifierDidInvalid,
    VerificationBackendIdInvalid,
    VerificationPolicyDigestLengthInvalid,
    VerificationRecordDigestLengthInvalid,
    VerificationOutcomeRejected,
    VerificationSignatureNotValidated,
    VerificationPolicyNotAccepted,
    VerificationPolicyNotCurrent,
    VerificationWitnessMismatch,
}

/// Observation-scoped semantic aggregation for one canonical request root.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedAttestationFulfillmentSetV2 {
    Absent,
    Converged {
        fulfillment_id: [u8; FULFILLMENT_ID_LEN_V2],
        replicas: usize,
    },
    Conflict {
        distinct_claims: usize,
        replicas: usize,
    },
}

fn valid_did_shape_v2(did: &str) -> bool {
    !did.is_empty() && did.len() <= DID_MAX_LEN_V1 && did.starts_with("did:")
}

fn validate_statement_shape_v2(
    statement: AttestationFulfillmentStatementV2<'_>,
) -> Result<(), AttestationFulfillmentErrorV2> {
    if statement.request_root_raw39.len() != REQUEST_ROOT_RAW39_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::RequestRootLengthInvalid);
    }
    if !valid_did_shape_v2(statement.subject_did) {
        return Err(AttestationFulfillmentErrorV2::SubjectDidInvalid);
    }
    if statement.kvector_commitment.len() != K_VECTOR_COMMITMENT_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::CommitmentLengthInvalid);
    }
    validate_score_range_v2(
        statement.trust_score_lower_scaled,
        statement.trust_score_upper_scaled,
    )
    .map_err(|_| AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)?;
    Ok(())
}

fn validate_expected_context_v2(
    expected_request_root_raw39: &[u8],
    expected_request_subject_did: &str,
) -> Result<(), AttestationFulfillmentErrorV2> {
    if expected_request_root_raw39.len() != REQUEST_ROOT_RAW39_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::RequestRootLengthInvalid);
    }
    if !valid_did_shape_v2(expected_request_subject_did) {
        return Err(AttestationFulfillmentErrorV2::SubjectDidInvalid);
    }
    Ok(())
}

fn sha256_v2(bytes: &[u8]) -> [u8; SHA256_DIGEST_LEN_V2] {
    let digest = Sha256::digest(bytes);
    let mut out = [0u8; SHA256_DIGEST_LEN_V2];
    out.copy_from_slice(&digest);
    out
}

/// Derive the strongest authority tier guaranteed by this public fulfillment range.
///
/// This delegates to the canonical lower-bound-only authority theorem. Fulfillment
/// semantics do not maintain an independent tier mapping.
pub fn guaranteed_attestation_fulfillment_tier_v2(
    statement: AttestationFulfillmentStatementV2<'_>,
) -> Result<GuaranteedTrustTierV2, AttestationFulfillmentErrorV2> {
    validate_statement_shape_v2(statement)?;
    guaranteed_trust_tier_v2(
        statement.trust_score_lower_scaled,
        statement.trust_score_upper_scaled,
    )
    .map_err(|_| AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)
}

/// Derive the deterministic semantic ID for one typed fulfillment statement.
///
/// Framing is versioned and field-tagged. No wall-clock issuance time, attempt
/// action hash, proof encoding or verification-record identity participates in the
/// semantic fulfillment identity.
pub fn derive_attestation_fulfillment_id_v2(
    statement: AttestationFulfillmentStatementV2<'_>,
) -> Result<[u8; FULFILLMENT_ID_LEN_V2], AttestationFulfillmentErrorV2> {
    validate_statement_shape_v2(statement)?;

    let mut hasher = Sha256::new();
    hasher.update(ATTESTATION_FULFILLMENT_DOMAIN_V2);

    hasher.update([0x01]);
    hasher.update(statement.request_root_raw39);

    hasher.update([0x02]);
    let subject_len = statement.subject_did.len() as u16;
    hasher.update(subject_len.to_be_bytes());
    hasher.update(statement.subject_did.as_bytes());

    hasher.update([0x03]);
    hasher.update(statement.kvector_commitment);

    hasher.update([0x04]);
    hasher.update(statement.trust_score_lower_scaled.to_be_bytes());

    hasher.update([0x05]);
    hasher.update(statement.trust_score_upper_scaled.to_be_bytes());

    hasher.update([0x06]);
    match statement.credential_expires_at_micros {
        None => hasher.update([0x00]),
        Some(expires_at_micros) => {
            hasher.update([0x01]);
            hasher.update(expires_at_micros.to_be_bytes());
        }
    }

    let digest = hasher.finalize();
    let mut fulfillment_id = [0u8; FULFILLMENT_ID_LEN_V2];
    fulfillment_id.copy_from_slice(&digest);
    Ok(fulfillment_id)
}

fn validate_claim_and_id_v2(
    claim: AttestationFulfillmentClaimV2<'_>,
    expected_request_root_raw39: &[u8],
    expected_request_subject_did: &str,
    action_author_did: &str,
) -> Result<[u8; FULFILLMENT_ID_LEN_V2], AttestationFulfillmentErrorV2> {
    validate_expected_context_v2(expected_request_root_raw39, expected_request_subject_did)?;
    let expected_id = derive_attestation_fulfillment_id_v2(claim.statement)?;

    if claim.statement.request_root_raw39 != expected_request_root_raw39 {
        return Err(AttestationFulfillmentErrorV2::RequestRootMismatch);
    }
    if claim.statement.subject_did != expected_request_subject_did {
        return Err(AttestationFulfillmentErrorV2::SubjectMismatch);
    }
    if !valid_did_shape_v2(action_author_did) {
        return Err(AttestationFulfillmentErrorV2::ActionAuthorDidInvalid);
    }
    if action_author_did != claim.statement.subject_did {
        return Err(AttestationFulfillmentErrorV2::SubjectActorRequired);
    }
    if claim.range_proof.is_empty() {
        return Err(AttestationFulfillmentErrorV2::RangeProofEmpty);
    }
    if claim.fulfillment_id.len() != FULFILLMENT_ID_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::FulfillmentIdLengthInvalid);
    }
    if claim.fulfillment_id != expected_id.as_slice() {
        return Err(AttestationFulfillmentErrorV2::FulfillmentIdMismatch);
    }

    Ok(expected_id)
}

/// Validate one fulfillment claim against immutable request-root context.
pub fn validate_attestation_fulfillment_claim_v2(
    claim: AttestationFulfillmentClaimV2<'_>,
    expected_request_root_raw39: &[u8],
    expected_request_subject_did: &str,
    action_author_did: &str,
) -> Result<(), AttestationFulfillmentErrorV2> {
    validate_claim_and_id_v2(
        claim,
        expected_request_root_raw39,
        expected_request_subject_did,
        action_author_did,
    )
    .map(|_| ())
}

/// Qualify signed/current proof-verification facts against one exact claim.
///
/// This is deliberately a structural capability constructor, not a cryptographic
/// signature verifier. The future verification-record adapter must establish
/// `signature_validated`, `policy_accepted` and `policy_current` from authoritative
/// evidence; those facts must never be copied from an untrusted fulfillment caller.
pub fn qualify_attestation_proof_verification_v2(
    claim: AttestationFulfillmentClaimV2<'_>,
    record: AttestationProofVerificationRecordFactsV2<'_>,
) -> Result<QualifiedAttestationProofVerificationV2, AttestationFulfillmentErrorV2> {
    let expected_id = derive_attestation_fulfillment_id_v2(claim.statement)?;
    if claim.range_proof.is_empty() {
        return Err(AttestationFulfillmentErrorV2::RangeProofEmpty);
    }
    if claim.fulfillment_id.len() != FULFILLMENT_ID_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::FulfillmentIdLengthInvalid);
    }
    if claim.fulfillment_id != expected_id.as_slice() {
        return Err(AttestationFulfillmentErrorV2::FulfillmentIdMismatch);
    }

    if record.fulfillment_id.len() != FULFILLMENT_ID_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::VerificationFulfillmentIdLengthInvalid);
    }
    if record.fulfillment_id != expected_id.as_slice() {
        return Err(AttestationFulfillmentErrorV2::VerificationFulfillmentIdMismatch);
    }
    if record.proof_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::VerificationProofDigestLengthInvalid);
    }
    let actual_proof_sha256 = sha256_v2(claim.range_proof);
    if record.proof_sha256 != actual_proof_sha256.as_slice() {
        return Err(AttestationFulfillmentErrorV2::VerificationProofDigestMismatch);
    }
    if !valid_did_shape_v2(record.verifier_did) {
        return Err(AttestationFulfillmentErrorV2::VerifierDidInvalid);
    }
    if record.backend_id.is_empty() || record.backend_id.len() > VERIFICATION_BACKEND_ID_MAX_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::VerificationBackendIdInvalid);
    }
    if record.verification_policy_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::VerificationPolicyDigestLengthInvalid);
    }
    if record.verification_record_sha256.len() != SHA256_DIGEST_LEN_V2 {
        return Err(AttestationFulfillmentErrorV2::VerificationRecordDigestLengthInvalid);
    }
    if record.outcome != AttestationProofVerificationOutcomeV2::Accepted {
        return Err(AttestationFulfillmentErrorV2::VerificationOutcomeRejected);
    }
    if !record.signature_validated {
        return Err(AttestationFulfillmentErrorV2::VerificationSignatureNotValidated);
    }
    if !record.policy_accepted {
        return Err(AttestationFulfillmentErrorV2::VerificationPolicyNotAccepted);
    }
    if !record.policy_current {
        return Err(AttestationFulfillmentErrorV2::VerificationPolicyNotCurrent);
    }

    Ok(QualifiedAttestationProofVerificationV2 {
        fulfillment_id: expected_id,
        proof_sha256: actual_proof_sha256,
    })
}

/// Collapse observed **qualified** retry replicas for one request root without
/// choosing a chronological winner.
pub fn summarize_attestation_fulfillment_observations_v2(
    observations: &[AttestationFulfillmentObservationV2<'_>],
    expected_request_root_raw39: &[u8],
    expected_request_subject_did: &str,
) -> Result<ObservedAttestationFulfillmentSetV2, AttestationFulfillmentErrorV2> {
    validate_expected_context_v2(expected_request_root_raw39, expected_request_subject_did)?;

    if observations.is_empty() {
        return Ok(ObservedAttestationFulfillmentSetV2::Absent);
    }

    let mut distinct_ids: Vec<[u8; FULFILLMENT_ID_LEN_V2]> = Vec::new();
    for observation in observations {
        let fulfillment_id = validate_claim_and_id_v2(
            observation.claim,
            expected_request_root_raw39,
            expected_request_subject_did,
            observation.action_author_did,
        )?;
        let proof_sha256 = sha256_v2(observation.claim.range_proof);
        if observation.verification.fulfillment_id != fulfillment_id
            || observation.verification.proof_sha256 != proof_sha256
        {
            return Err(AttestationFulfillmentErrorV2::VerificationWitnessMismatch);
        }
        if !distinct_ids.contains(&fulfillment_id) {
            distinct_ids.push(fulfillment_id);
        }
    }

    if distinct_ids.len() == 1 {
        Ok(ObservedAttestationFulfillmentSetV2::Converged {
            fulfillment_id: distinct_ids[0],
            replicas: observations.len(),
        })
    } else {
        Ok(ObservedAttestationFulfillmentSetV2::Conflict {
            distinct_claims: distinct_ids.len(),
            replicas: observations.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SUBJECT: &str = "did:mycelix:subject";
    const VERIFIER: &str = "did:mycelix:verifier";

    fn root_raw39() -> [u8; REQUEST_ROOT_RAW39_LEN_V2] {
        let mut root = [0u8; REQUEST_ROOT_RAW39_LEN_V2];
        for (index, byte) in root.iter_mut().enumerate() {
            *byte = index as u8;
        }
        root
    }

    fn commitment() -> [u8; K_VECTOR_COMMITMENT_LEN_V2] {
        [0xA5; K_VECTOR_COMMITMENT_LEN_V2]
    }

    fn valid_statement<'a>(
        root: &'a [u8],
        commitment: &'a [u8],
    ) -> AttestationFulfillmentStatementV2<'a> {
        AttestationFulfillmentStatementV2 {
            request_root_raw39: root,
            subject_did: SUBJECT,
            kvector_commitment: commitment,
            trust_score_lower_scaled: 400_000,
            trust_score_upper_scaled: 600_000,
            credential_expires_at_micros: Some(1_700_000_000_000_000),
        }
    }

    fn claim_for<'a>(
        statement: AttestationFulfillmentStatementV2<'a>,
        proof: &'a [u8],
        fulfillment_id: &'a [u8],
    ) -> AttestationFulfillmentClaimV2<'a> {
        AttestationFulfillmentClaimV2 {
            statement,
            range_proof: proof,
            fulfillment_id,
        }
    }

    fn verification_facts<'a>(
        claim: AttestationFulfillmentClaimV2<'a>,
        proof_digest: &'a [u8],
        policy_digest: &'a [u8],
        record_digest: &'a [u8],
    ) -> AttestationProofVerificationRecordFactsV2<'a> {
        AttestationProofVerificationRecordFactsV2 {
            fulfillment_id: claim.fulfillment_id,
            proof_sha256: proof_digest,
            verifier_did: VERIFIER,
            backend_id: "qualified-kvector-v2",
            verification_policy_sha256: policy_digest,
            verification_record_sha256: record_digest,
            outcome: AttestationProofVerificationOutcomeV2::Accepted,
            signature_validated: true,
            policy_accepted: true,
            policy_current: true,
        }
    }

    fn qualify<'a>(
        claim: AttestationFulfillmentClaimV2<'a>,
    ) -> QualifiedAttestationProofVerificationV2 {
        let proof_digest = sha256_v2(claim.range_proof);
        let policy_digest = [0x11; SHA256_DIGEST_LEN_V2];
        let record_digest = [0x22; SHA256_DIGEST_LEN_V2];
        qualify_attestation_proof_verification_v2(
            claim,
            verification_facts(claim, &proof_digest, &policy_digest, &record_digest),
        )
        .expect("verification facts qualify")
    }

    #[test]
    fn deterministic_vector_is_frozen() {
        let root = root_raw39();
        let commitment = commitment();
        let actual = derive_attestation_fulfillment_id_v2(valid_statement(&root, &commitment))
            .expect("valid statement");
        let expected = [
            0x22, 0xa6, 0xf3, 0xa7, 0x71, 0xad, 0x8d, 0x36, 0x5e, 0x7a, 0xc9, 0x84,
            0x0a, 0x9a, 0xcb, 0x67, 0x40, 0x1b, 0x89, 0xc9, 0x25, 0xe1, 0xb3, 0x63,
            0xee, 0xa2, 0x92, 0xa9, 0x90, 0x59, 0xc1, 0x7e,
        ];
        assert_eq!(actual, expected);
    }

    #[test]
    fn fulfillment_delegates_guaranteed_tier_to_authority_theorem() {
        let root = root_raw39();
        let commitment = commitment();
        let mut statement = valid_statement(&root, &commitment);
        statement.trust_score_upper_scaled = 800_000;
        assert_eq!(
            guaranteed_attestation_fulfillment_tier_v2(statement),
            Ok(GuaranteedTrustTierV2::Standard)
        );
    }

    #[test]
    fn semantic_identity_excludes_proof_encoding() {
        let root = root_raw39();
        let commitment = commitment();
        let base = valid_statement(&root, &commitment);
        let base_id = derive_attestation_fulfillment_id_v2(base).unwrap();
        let first = claim_for(base, b"proof-a", &base_id);
        let second = claim_for(base, b"proof-b", &base_id);
        assert_eq!(first.fulfillment_id, second.fulfillment_id);
        assert_ne!(sha256_v2(first.range_proof), sha256_v2(second.range_proof));
    }

    #[test]
    fn every_semantic_statement_field_changes_identity() {
        let root = root_raw39();
        let commitment = commitment();
        let base = valid_statement(&root, &commitment);
        let base_id = derive_attestation_fulfillment_id_v2(base).unwrap();

        let mut other_root = root;
        other_root[10] ^= 1;
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                request_root_raw39: &other_root,
                ..base
            })
            .unwrap(),
            base_id
        );
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                subject_did: "did:mycelix:other",
                ..base
            })
            .unwrap(),
            base_id
        );
        let mut other_commitment = commitment;
        other_commitment[0] ^= 1;
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                kvector_commitment: &other_commitment,
                ..base
            })
            .unwrap(),
            base_id
        );
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                trust_score_lower_scaled: 410_000,
                ..base
            })
            .unwrap(),
            base_id
        );
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                trust_score_upper_scaled: 610_000,
                ..base
            })
            .unwrap(),
            base_id
        );
        assert_ne!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                credential_expires_at_micros: None,
                ..base
            })
            .unwrap(),
            base_id
        );
    }

    #[test]
    fn malformed_statement_and_range_fail_closed() {
        let root = root_raw39();
        let commitment = commitment();
        let base = valid_statement(&root, &commitment);
        let short_root = [0u8; REQUEST_ROOT_RAW39_LEN_V2 - 1];
        assert_eq!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                request_root_raw39: &short_root,
                ..base
            }),
            Err(AttestationFulfillmentErrorV2::RequestRootLengthInvalid)
        );
        assert_eq!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                trust_score_lower_scaled: 1_000_001,
                ..base
            }),
            Err(AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)
        );
        assert_eq!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                trust_score_upper_scaled: 1_000_001,
                ..base
            }),
            Err(AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)
        );
        assert_eq!(
            derive_attestation_fulfillment_id_v2(AttestationFulfillmentStatementV2 {
                trust_score_lower_scaled: 700_000,
                trust_score_upper_scaled: 600_000,
                ..base
            }),
            Err(AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)
        );
    }

    #[test]
    fn claim_binds_root_subject_actor_proof_and_semantic_id() {
        let root = root_raw39();
        let commitment = commitment();
        let statement = valid_statement(&root, &commitment);
        let id = derive_attestation_fulfillment_id_v2(statement).unwrap();
        let claim = claim_for(statement, b"proof-a", &id);
        assert_eq!(
            validate_attestation_fulfillment_claim_v2(claim, &root, SUBJECT, SUBJECT),
            Ok(())
        );
        assert_eq!(
            validate_attestation_fulfillment_claim_v2(
                claim,
                &root,
                SUBJECT,
                "did:mycelix:other"
            ),
            Err(AttestationFulfillmentErrorV2::SubjectActorRequired)
        );
        let empty = AttestationFulfillmentClaimV2 {
            range_proof: b"",
            ..claim
        };
        assert_eq!(
            validate_attestation_fulfillment_claim_v2(empty, &root, SUBJECT, SUBJECT),
            Err(AttestationFulfillmentErrorV2::RangeProofEmpty)
        );
    }

    #[test]
    fn verification_token_requires_exact_proof_claim_signature_and_policy_facts() {
        let root = root_raw39();
        let commitment = commitment();
        let statement = valid_statement(&root, &commitment);
        let id = derive_attestation_fulfillment_id_v2(statement).unwrap();
        let claim = claim_for(statement, b"proof-a", &id);
        let proof_digest = sha256_v2(claim.range_proof);
        let policy_digest = [0x11; SHA256_DIGEST_LEN_V2];
        let record_digest = [0x22; SHA256_DIGEST_LEN_V2];
        let valid = verification_facts(claim, &proof_digest, &policy_digest, &record_digest);
        let token = qualify_attestation_proof_verification_v2(claim, valid).unwrap();
        assert_eq!(token.fulfillment_id(), &id);
        assert_eq!(token.proof_sha256(), &proof_digest);

        let wrong_proof_digest = [0u8; SHA256_DIGEST_LEN_V2];
        assert_eq!(
            qualify_attestation_proof_verification_v2(
                claim,
                AttestationProofVerificationRecordFactsV2 {
                    proof_sha256: &wrong_proof_digest,
                    ..valid
                }
            ),
            Err(AttestationFulfillmentErrorV2::VerificationProofDigestMismatch)
        );
        assert_eq!(
            qualify_attestation_proof_verification_v2(
                claim,
                AttestationProofVerificationRecordFactsV2 {
                    outcome: AttestationProofVerificationOutcomeV2::Rejected,
                    ..valid
                }
            ),
            Err(AttestationFulfillmentErrorV2::VerificationOutcomeRejected)
        );
        assert_eq!(
            qualify_attestation_proof_verification_v2(
                claim,
                AttestationProofVerificationRecordFactsV2 {
                    signature_validated: false,
                    ..valid
                }
            ),
            Err(AttestationFulfillmentErrorV2::VerificationSignatureNotValidated)
        );
        assert_eq!(
            qualify_attestation_proof_verification_v2(
                claim,
                AttestationProofVerificationRecordFactsV2 {
                    policy_accepted: false,
                    ..valid
                }
            ),
            Err(AttestationFulfillmentErrorV2::VerificationPolicyNotAccepted)
        );
        assert_eq!(
            qualify_attestation_proof_verification_v2(
                claim,
                AttestationProofVerificationRecordFactsV2 {
                    policy_current: false,
                    ..valid
                }
            ),
            Err(AttestationFulfillmentErrorV2::VerificationPolicyNotCurrent)
        );
    }

    #[test]
    fn different_qualified_proof_encodings_for_same_statement_converge() {
        let root = root_raw39();
        let commitment = commitment();
        let statement = valid_statement(&root, &commitment);
        let id = derive_attestation_fulfillment_id_v2(statement).unwrap();
        let first = claim_for(statement, b"proof-encoding-a", &id);
        let second = claim_for(statement, b"proof-encoding-b", &id);
        let observations = [
            AttestationFulfillmentObservationV2 {
                claim: first,
                action_author_did: SUBJECT,
                verification: qualify(first),
            },
            AttestationFulfillmentObservationV2 {
                claim: second,
                action_author_did: SUBJECT,
                verification: qualify(second),
            },
        ];
        assert_eq!(
            summarize_attestation_fulfillment_observations_v2(&observations, &root, SUBJECT),
            Ok(ObservedAttestationFulfillmentSetV2::Converged {
                fulfillment_id: id,
                replicas: 2,
            })
        );
    }

    #[test]
    fn proof_token_cannot_be_reused_for_different_proof_encoding() {
        let root = root_raw39();
        let commitment = commitment();
        let statement = valid_statement(&root, &commitment);
        let id = derive_attestation_fulfillment_id_v2(statement).unwrap();
        let first = claim_for(statement, b"proof-a", &id);
        let second = claim_for(statement, b"proof-b", &id);
        let observations = [AttestationFulfillmentObservationV2 {
            claim: second,
            action_author_did: SUBJECT,
            verification: qualify(first),
        }];
        assert_eq!(
            summarize_attestation_fulfillment_observations_v2(&observations, &root, SUBJECT),
            Err(AttestationFulfillmentErrorV2::VerificationWitnessMismatch)
        );
    }

    #[test]
    fn distinct_qualified_statements_for_one_root_are_conflict_not_latest_wins() {
        let root = root_raw39();
        let commitment = commitment();
        let first_statement = valid_statement(&root, &commitment);
        let first_id = derive_attestation_fulfillment_id_v2(first_statement).unwrap();
        let first = claim_for(first_statement, b"proof-a", &first_id);

        let mut second_commitment = commitment;
        second_commitment[0] ^= 1;
        let second_statement = AttestationFulfillmentStatementV2 {
            kvector_commitment: &second_commitment,
            ..first_statement
        };
        let second_id = derive_attestation_fulfillment_id_v2(second_statement).unwrap();
        let second = claim_for(second_statement, b"proof-b", &second_id);

        let observations = [
            AttestationFulfillmentObservationV2 {
                claim: first,
                action_author_did: SUBJECT,
                verification: qualify(first),
            },
            AttestationFulfillmentObservationV2 {
                claim: second,
                action_author_did: SUBJECT,
                verification: qualify(second),
            },
        ];
        assert_eq!(
            summarize_attestation_fulfillment_observations_v2(&observations, &root, SUBJECT),
            Ok(ObservedAttestationFulfillmentSetV2::Conflict {
                distinct_claims: 2,
                replicas: 2,
            })
        );
    }
}
