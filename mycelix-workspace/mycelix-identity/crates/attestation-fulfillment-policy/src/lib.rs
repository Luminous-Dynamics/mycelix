// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Convergent semantic theorem for Mycelix attestation fulfillment V2.
//!
//! This crate owns **semantic fulfillment only**. It deliberately does not own
//! cryptographic proof verification, signed verification records, Holochain state,
//! credential creation, network completeness, or verifier-policy currentness.
//!
//! Security split:
//!
//! - semantic fulfillment identity is deterministic and independent of execution;
//! - public trust-score bounds use the canonical fixed-point authority theorem;
//! - proof encoding is evidence, not semantic identity;
//! - structurally valid retries may converge semantically before proof verification;
//! - semantic convergence is **not** cryptographic acceptance and grants no authority;
//! - cryptographic qualification belongs above the backend-neutral proof statement.

#![forbid(unsafe_code)]

use mycelix_attestation_request_root_policy::DID_MAX_LEN_V1;
use mycelix_trust_authority_policy::{
    guaranteed_trust_tier_v2, validate_score_range_v2, GuaranteedTrustTierV2,
};
use sha2::{Digest, Sha256};

pub const REQUEST_ROOT_RAW39_LEN_V2: usize = 39;
pub const K_VECTOR_COMMITMENT_LEN_V2: usize = 32;
pub const FULFILLMENT_ID_LEN_V2: usize = 32;
pub const ATTESTATION_FULFILLMENT_DOMAIN_V2: &[u8] =
    b"mycelix:identity:attestation-fulfillment:v2\0";

/// Public semantic statement whose deterministic identity converges retries.
///
/// Trust-score bounds are canonical fixed-point integers scaled by the authority
/// theorem (`1_000_000 == 1.0`). The range proof itself is intentionally absent
/// from this statement identity: proof bytes are evidence for the statement, not
/// the semantic effect being requested.
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

/// One semantic fulfillment claim carrying proof bytes plus its deterministic ID.
///
/// `range_proof` is intentionally opaque here. Non-empty proof bytes make the
/// claim structurally complete enough to enter evidence gathering, but **do not**
/// mean that the proof is valid, accepted, current, or authoritative.
#[derive(Debug, Clone, Copy)]
pub struct AttestationFulfillmentClaimV2<'a> {
    pub statement: AttestationFulfillmentStatementV2<'a>,
    pub range_proof: &'a [u8],
    pub fulfillment_id: &'a [u8],
}

/// One observed semantic claim plus authoritative action-author identity.
///
/// This type contains no proof-verification witness. A later verification-record
/// layer may filter/qualify claims cryptographically, but that stronger evidence is
/// intentionally outside this crate.
#[derive(Debug, Clone, Copy)]
pub struct AttestationFulfillmentObservationV2<'a> {
    pub claim: AttestationFulfillmentClaimV2<'a>,
    pub action_author_did: &'a str,
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
}

/// Observation-scoped aggregation of **semantic claims only**.
///
/// `ConvergedCandidate` means multiple observed claims describe the same semantic
/// effect. It does not imply that any proof has been cryptographically accepted.
/// A later verified-evidence layer must independently qualify proof bytes and bind
/// them to the backend-neutral proof-statement digest before an effect can become
/// authority-bearing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedAttestationFulfillmentClaimSetV2 {
    Absent,
    ConvergedCandidate {
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

/// Derive the strongest authority tier guaranteed by this public fulfillment range.
///
/// This delegates to the canonical lower-bound-only authority theorem. It is a
/// projection of the statement's **claimed/provable range**, not evidence that the
/// range has actually been proven. Positive authority still requires a qualified
/// cryptographic verification record above this layer.
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

/// Validate one semantic fulfillment claim against immutable request-root context.
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

/// Collapse observed **semantic** retry claims for one canonical request root.
///
/// Every claim is revalidated for root/subject/actor/ID/proof-presence structure.
/// Proof bytes are not cryptographically verified here. Exact semantic retries
/// become `ConvergedCandidate`; multiple semantic effects become `Conflict`.
/// Neither result grants authority.
pub fn summarize_attestation_fulfillment_claims_v2(
    observations: &[AttestationFulfillmentObservationV2<'_>],
    expected_request_root_raw39: &[u8],
    expected_request_subject_did: &str,
) -> Result<ObservedAttestationFulfillmentClaimSetV2, AttestationFulfillmentErrorV2> {
    validate_expected_context_v2(expected_request_root_raw39, expected_request_subject_did)?;

    if observations.is_empty() {
        return Ok(ObservedAttestationFulfillmentClaimSetV2::Absent);
    }

    let mut distinct_ids: Vec<[u8; FULFILLMENT_ID_LEN_V2]> = Vec::new();
    for observation in observations {
        let fulfillment_id = validate_claim_and_id_v2(
            observation.claim,
            expected_request_root_raw39,
            expected_request_subject_did,
            observation.action_author_did,
        )?;
        if !distinct_ids.contains(&fulfillment_id) {
            distinct_ids.push(fulfillment_id);
        }
    }

    if distinct_ids.len() == 1 {
        Ok(ObservedAttestationFulfillmentClaimSetV2::ConvergedCandidate {
            fulfillment_id: distinct_ids[0],
            replicas: observations.len(),
        })
    } else {
        Ok(ObservedAttestationFulfillmentClaimSetV2::Conflict {
            distinct_claims: distinct_ids.len(),
            replicas: observations.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SUBJECT: &str = "did:mycelix:subject";

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
        assert_ne!(first.range_proof, second.range_proof);
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
                trust_score_lower_scaled: 700_000,
                trust_score_upper_scaled: 600_000,
                ..base
            }),
            Err(AttestationFulfillmentErrorV2::TrustScoreRangeInvalid)
        );
    }

    #[test]
    fn claim_binds_root_subject_actor_proof_presence_and_semantic_id() {
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
    fn different_unverified_proof_encodings_converge_semantically_only() {
        let root = root_raw39();
        let commitment = commitment();
        let statement = valid_statement(&root, &commitment);
        let id = derive_attestation_fulfillment_id_v2(statement).unwrap();
        let first = claim_for(statement, b"unverified-proof-a", &id);
        let second = claim_for(statement, b"unverified-proof-b", &id);
        let observations = [
            AttestationFulfillmentObservationV2 {
                claim: first,
                action_author_did: SUBJECT,
            },
            AttestationFulfillmentObservationV2 {
                claim: second,
                action_author_did: SUBJECT,
            },
        ];
        assert_eq!(
            summarize_attestation_fulfillment_claims_v2(&observations, &root, SUBJECT),
            Ok(ObservedAttestationFulfillmentClaimSetV2::ConvergedCandidate {
                fulfillment_id: id,
                replicas: 2,
            })
        );
    }

    #[test]
    fn distinct_semantic_claims_for_one_root_are_conflict_not_latest_wins() {
        let root = root_raw39();
        let commitment = commitment();
        let first_statement = valid_statement(&root, &commitment);
        let first_id = derive_attestation_fulfillment_id_v2(first_statement).unwrap();
        let first = claim_for(first_statement, b"unverified-proof-a", &first_id);

        let mut second_commitment = commitment;
        second_commitment[0] ^= 1;
        let second_statement = AttestationFulfillmentStatementV2 {
            kvector_commitment: &second_commitment,
            ..first_statement
        };
        let second_id = derive_attestation_fulfillment_id_v2(second_statement).unwrap();
        let second = claim_for(second_statement, b"unverified-proof-b", &second_id);

        let observations = [
            AttestationFulfillmentObservationV2 {
                claim: first,
                action_author_did: SUBJECT,
            },
            AttestationFulfillmentObservationV2 {
                claim: second,
                action_author_did: SUBJECT,
            },
        ];
        assert_eq!(
            summarize_attestation_fulfillment_claims_v2(&observations, &root, SUBJECT),
            Ok(ObservedAttestationFulfillmentClaimSetV2::Conflict {
                distinct_claims: 2,
                replicas: 2,
            })
        );
    }
}