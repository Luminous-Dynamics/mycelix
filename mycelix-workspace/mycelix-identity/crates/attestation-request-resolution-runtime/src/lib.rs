// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Fail-closed observed-state adapter for Mycelix attestation request lineages.
//!
//! This crate owns no network traversal. A future HDK producer must gather one
//! canonical request creation root plus deduplicated CRUD observations, then hand
//! those observations here for structural/historical re-audit.
//!
//! The adapter deliberately distinguishes observed state from global truth. It
//! treats malformed historical roots, deletes, non-root updates, and invalid
//! lifecycle edges as errors. Multiple individually valid terminal siblings are
//! an explicit conflict rather than a "latest wins" race.

#![forbid(unsafe_code)]

use mycelix_attestation_request_policy::{
    AttestationRequestAssertionEqualityV1, AttestationRequestTransitionErrorV1,
    AttestationRequestTransitionV1, classify_attestation_request_actor_v1,
    validate_attestation_request_transition_v1,
};
use mycelix_attestation_request_root_policy::{
    AttestationRequestRootAssertionErrorV1, AttestationRequestRootAssertionV1,
    AttestationRequestStatusV1, validate_attestation_request_root_assertion_v1,
};
use trust_credential_integrity::{AttestationRequest, AttestationStatus};

/// One network-observed direct update of the canonical request creation root.
pub struct AttestationRequestUpdateObservationV1<'a> {
    pub request: &'a AttestationRequest,
    /// DID derived from the update action author by the HDK producer.
    pub author_did: &'a str,
    pub action_timestamp_micros: i64,
    /// Must be true only when `original_action_address` equals the canonical
    /// creation action. Historical nested update chains are not silently folded.
    pub parent_is_canonical_root: bool,
}

/// Complete evidence supplied for one canonical request creation root.
pub struct AttestationRequestLineageObservationV1<'a> {
    pub root: &'a AttestationRequest,
    /// DID derived from the canonical creation action author.
    pub root_author_did: &'a str,
    /// Subject DID whose `SubjectToRequest` index is being resolved.
    pub expected_subject_did: &'a str,
    /// Distinct update action observations after action-hash deduplication.
    pub updates: &'a [AttestationRequestUpdateObservationV1<'a>],
    /// Number of distinct valid delete actions observed for the lineage.
    pub observed_delete_count: usize,
    pub now_micros: i64,
}

/// Observation-scoped request state. None of these variants claim global DHT
/// completeness; they describe only the evidence gathered for this resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObservedAttestationRequestStateV1 {
    ObservedPending,
    ObservedFulfilled,
    ObservedDeclined,
    ObservedExpired,
    ObservedCancelled,
    /// No terminal update was observed, but immutable expiry has elapsed.
    ObservedExpiredByTime,
    /// Multiple distinct terminal sibling updates were individually conforming.
    /// No source-chain/network chronology is allowed to choose a winner.
    ObservedConflict { terminal_updates: usize },
}

/// Fail-closed defects in historical/request observation evidence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttestationRequestResolutionErrorV1 {
    /// The canonical creation action author does not match the immutable requester DID.
    RootAuthorMismatch,
    /// The root was reached through a subject index that does not match the immutable subject DID.
    RootSubjectMismatch,
    /// Canonical structural re-audit rejected the historical creation root.
    RootAssertionRejected {
        error: AttestationRequestRootAssertionErrorV1,
    },
    HistoricalDeleteObserved {
        count: usize,
    },
    NonRootUpdateObserved {
        index: usize,
    },
    UpdatePolicyRejected {
        index: usize,
        error: AttestationRequestTransitionErrorV1,
    },
    NonTerminalUpdateObserved {
        index: usize,
    },
    DuplicateRequestIdObserved {
        first_index: usize,
        second_index: usize,
    },
}

fn status_v1(status: &AttestationStatus) -> AttestationRequestStatusV1 {
    match status {
        AttestationStatus::Pending => AttestationRequestStatusV1::Pending,
        AttestationStatus::Fulfilled => AttestationRequestStatusV1::Fulfilled,
        AttestationStatus::Declined => AttestationRequestStatusV1::Declined,
        AttestationStatus::Expired => AttestationRequestStatusV1::Expired,
        AttestationStatus::Cancelled => AttestationRequestStatusV1::Cancelled,
    }
}

fn assertion_equality_v1(
    root: &AttestationRequest,
    update: &AttestationRequest,
) -> AttestationRequestAssertionEqualityV1 {
    AttestationRequestAssertionEqualityV1 {
        id: update.id == root.id,
        requester_did: update.requester_did == root.requester_did,
        subject_did: update.subject_did == root.subject_did,
        components: update.components == root.components,
        min_trust_score: update.min_trust_score == root.min_trust_score,
        min_tier: update.min_tier == root.min_tier,
        purpose: update.purpose == root.purpose,
        expires_at: update.expires_at == root.expires_at,
        created_at: update.created_at == root.created_at,
    }
}

/// Re-audit a canonical request creation root before any observed state is used.
///
/// The resolver owns only observation bindings around the canonical root theorem:
/// root action author -> requester DID, and subject index -> immutable subject DID.
/// All structural root validity delegates to #185's Holochain-free policy.
pub fn validate_attestation_request_root_v1(
    root: &AttestationRequest,
    root_author_did: &str,
    expected_subject_did: &str,
) -> Result<(), AttestationRequestResolutionErrorV1> {
    if root.requester_did != root_author_did {
        return Err(AttestationRequestResolutionErrorV1::RootAuthorMismatch);
    }
    if root.subject_did != expected_subject_did {
        return Err(AttestationRequestResolutionErrorV1::RootSubjectMismatch);
    }

    let assertion = AttestationRequestRootAssertionV1 {
        id: &root.id,
        requester_did: &root.requester_did,
        subject_did: &root.subject_did,
        component_count: root.components.len(),
        purpose: &root.purpose,
        min_trust_score: root.min_trust_score,
        status: status_v1(&root.status),
        created_at_micros: root.created_at.as_micros(),
        expires_at_micros: root.expires_at.as_micros(),
    };

    validate_attestation_request_root_assertion_v1(assertion).map_err(|error| {
        AttestationRequestResolutionErrorV1::RootAssertionRejected { error }
    })
}

/// Fail closed when two distinct canonical roots expose the same free-form request
/// ID to one subject. Callers must not let fulfillment become "first matching ID".
pub fn validate_unique_request_ids_v1(
    requests: &[&AttestationRequest],
) -> Result<(), AttestationRequestResolutionErrorV1> {
    for first in 0..requests.len() {
        for second in (first + 1)..requests.len() {
            if requests[first].id == requests[second].id {
                return Err(
                    AttestationRequestResolutionErrorV1::DuplicateRequestIdObserved {
                        first_index: first,
                        second_index: second,
                    },
                );
            }
        }
    }
    Ok(())
}

/// Resolve one canonical request lineage into an observation-scoped state.
///
/// Preconditions enforced by the future HDK producer but rechecked where possible:
/// canonical root actions and update actions are deduplicated by `ActionHash`;
/// each supplied update's parent relationship is stated explicitly; delete count
/// contains distinct delete actions.
pub fn resolve_attestation_request_lineage_v1(
    input: AttestationRequestLineageObservationV1<'_>,
) -> Result<ObservedAttestationRequestStateV1, AttestationRequestResolutionErrorV1> {
    validate_attestation_request_root_v1(
        input.root,
        input.root_author_did,
        input.expected_subject_did,
    )?;

    if input.observed_delete_count != 0 {
        return Err(AttestationRequestResolutionErrorV1::HistoricalDeleteObserved {
            count: input.observed_delete_count,
        });
    }

    for (index, observation) in input.updates.iter().enumerate() {
        if !observation.parent_is_canonical_root {
            return Err(AttestationRequestResolutionErrorV1::NonRootUpdateObserved {
                index,
            });
        }

        let transition = AttestationRequestTransitionV1 {
            original_status: AttestationRequestStatusV1::Pending,
            updated_status: status_v1(&observation.request.status),
            actor: classify_attestation_request_actor_v1(
                observation.author_did,
                &input.root.requester_did,
                &input.root.subject_did,
            ),
            assertion_equality: assertion_equality_v1(input.root, observation.request),
            update_timestamp_micros: observation.action_timestamp_micros,
            expires_at_micros: input.root.expires_at.as_micros(),
        };

        if let Err(error) = validate_attestation_request_transition_v1(transition) {
            return Err(AttestationRequestResolutionErrorV1::UpdatePolicyRejected {
                index,
                error,
            });
        }
    }

    match input.updates.len() {
        0 => {
            if input.now_micros < input.root.expires_at.as_micros() {
                Ok(ObservedAttestationRequestStateV1::ObservedPending)
            } else {
                Ok(ObservedAttestationRequestStateV1::ObservedExpiredByTime)
            }
        }
        1 => match status_v1(&input.updates[0].request.status) {
            AttestationRequestStatusV1::Fulfilled => {
                Ok(ObservedAttestationRequestStateV1::ObservedFulfilled)
            }
            AttestationRequestStatusV1::Declined => {
                Ok(ObservedAttestationRequestStateV1::ObservedDeclined)
            }
            AttestationRequestStatusV1::Expired => {
                Ok(ObservedAttestationRequestStateV1::ObservedExpired)
            }
            AttestationRequestStatusV1::Cancelled => {
                Ok(ObservedAttestationRequestStateV1::ObservedCancelled)
            }
            AttestationRequestStatusV1::Pending => {
                Err(AttestationRequestResolutionErrorV1::NonTerminalUpdateObserved {
                    index: 0,
                })
            }
        },
        count => Ok(ObservedAttestationRequestStateV1::ObservedConflict {
            terminal_updates: count,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use holochain_zome_types::prelude::Timestamp;
    use mycelix_attestation_request_policy::AttestationRequestTransitionErrorV1;
    use trust_credential_integrity::{KVectorComponent, TrustTier};

    const REQUESTER: &str = "did:mycelix:requester";
    const SUBJECT: &str = "did:mycelix:subject";

    fn root() -> AttestationRequest {
        AttestationRequest {
            id: "req-1".to_string(),
            requester_did: REQUESTER.to_string(),
            subject_did: SUBJECT.to_string(),
            components: vec![KVectorComponent::Reputation],
            min_trust_score: Some(0.5),
            min_tier: Some(TrustTier::Standard),
            purpose: "governance eligibility".to_string(),
            expires_at: Timestamp::from_micros(1_000),
            status: AttestationStatus::Pending,
            created_at: Timestamp::from_micros(100),
        }
    }

    fn with_status(root: &AttestationRequest, status: AttestationStatus) -> AttestationRequest {
        let mut update = root.clone();
        update.status = status;
        update
    }

    fn lineage<'a>(
        root: &'a AttestationRequest,
        updates: &'a [AttestationRequestUpdateObservationV1<'a>],
        delete_count: usize,
        now_micros: i64,
    ) -> AttestationRequestLineageObservationV1<'a> {
        AttestationRequestLineageObservationV1 {
            root,
            root_author_did: REQUESTER,
            expected_subject_did: SUBJECT,
            updates,
            observed_delete_count: delete_count,
            now_micros,
        }
    }

    fn root_error(
        error: AttestationRequestRootAssertionErrorV1,
    ) -> AttestationRequestResolutionErrorV1 {
        AttestationRequestResolutionErrorV1::RootAssertionRejected { error }
    }

    #[test]
    fn valid_root_is_accepted() {
        assert_eq!(
            validate_attestation_request_root_v1(&root(), REQUESTER, SUBJECT),
            Ok(())
        );
    }

    #[test]
    fn root_author_and_subject_binding_are_reaudited() {
        let request = root();
        assert_eq!(
            validate_attestation_request_root_v1(&request, "did:mycelix:other", SUBJECT),
            Err(AttestationRequestResolutionErrorV1::RootAuthorMismatch)
        );
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, "did:mycelix:other"),
            Err(AttestationRequestResolutionErrorV1::RootSubjectMismatch)
        );
    }

    #[test]
    fn malformed_root_shape_fails_closed_through_canonical_policy() {
        let mut request = root();
        request.components.clear();
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
            Err(root_error(AttestationRequestRootAssertionErrorV1::ComponentsEmpty))
        );

        let mut request = root();
        request.purpose.clear();
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
            Err(root_error(AttestationRequestRootAssertionErrorV1::PurposeInvalid))
        );

        let mut request = root();
        request.status = AttestationStatus::Fulfilled;
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
            Err(root_error(
                AttestationRequestRootAssertionErrorV1::RootStatusNotPending,
            ))
        );
    }

    #[test]
    fn historical_identifier_and_validity_bounds_are_reaudited() {
        let mut request = root();
        request.id = "r".repeat(257);
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
            Err(root_error(AttestationRequestRootAssertionErrorV1::RequestIdInvalid))
        );

        let mut request = root();
        request.expires_at = request.created_at;
        assert_eq!(
            validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
            Err(root_error(
                AttestationRequestRootAssertionErrorV1::InvalidValidityInterval,
            ))
        );
    }

    #[test]
    fn historical_nonfinite_and_out_of_bounds_thresholds_fail_closed() {
        for (score, expected) in [
            (
                f32::NAN,
                AttestationRequestRootAssertionErrorV1::MinTrustScoreNonFinite,
            ),
            (
                f32::INFINITY,
                AttestationRequestRootAssertionErrorV1::MinTrustScoreNonFinite,
            ),
            (
                -0.01,
                AttestationRequestRootAssertionErrorV1::MinTrustScoreOutOfBounds,
            ),
            (
                1.01,
                AttestationRequestRootAssertionErrorV1::MinTrustScoreOutOfBounds,
            ),
        ] {
            let mut request = root();
            request.min_trust_score = Some(score);
            assert_eq!(
                validate_attestation_request_root_v1(&request, REQUESTER, SUBJECT),
                Err(root_error(expected))
            );
        }
    }

    #[test]
    fn no_updates_is_pending_before_expiry_and_expired_at_boundary() {
        let request = root();
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &[], 0, 999)),
            Ok(ObservedAttestationRequestStateV1::ObservedPending)
        );
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &[], 0, 1_000)),
            Ok(ObservedAttestationRequestStateV1::ObservedExpiredByTime)
        );
    }

    #[test]
    fn each_single_terminal_state_resolves_only_with_the_right_actor() {
        let request = root();
        let cases = [
            (
                AttestationStatus::Fulfilled,
                SUBJECT,
                500,
                ObservedAttestationRequestStateV1::ObservedFulfilled,
            ),
            (
                AttestationStatus::Declined,
                SUBJECT,
                500,
                ObservedAttestationRequestStateV1::ObservedDeclined,
            ),
            (
                AttestationStatus::Cancelled,
                REQUESTER,
                500,
                ObservedAttestationRequestStateV1::ObservedCancelled,
            ),
            (
                AttestationStatus::Expired,
                SUBJECT,
                1_000,
                ObservedAttestationRequestStateV1::ObservedExpired,
            ),
        ];

        for (status, author, timestamp, expected) in cases {
            let updated = with_status(&request, status);
            let observations = [AttestationRequestUpdateObservationV1 {
                request: &updated,
                author_did: author,
                action_timestamp_micros: timestamp,
                parent_is_canonical_root: true,
            }];
            assert_eq!(
                resolve_attestation_request_lineage_v1(lineage(
                    &request,
                    &observations,
                    0,
                    2_000,
                )),
                Ok(expected)
            );
        }
    }

    #[test]
    fn wrong_actor_is_a_policy_error_not_a_state() {
        let request = root();
        let updated = with_status(&request, AttestationStatus::Fulfilled);
        let observations = [AttestationRequestUpdateObservationV1 {
            request: &updated,
            author_did: REQUESTER,
            action_timestamp_micros: 500,
            parent_is_canonical_root: true,
        }];
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &observations, 0, 500)),
            Err(AttestationRequestResolutionErrorV1::UpdatePolicyRejected {
                index: 0,
                error: AttestationRequestTransitionErrorV1::SubjectRequired,
            })
        );
    }

    #[test]
    fn historical_assertion_mutation_is_rejected_by_shared_policy() {
        let request = root();
        let mut updated = with_status(&request, AttestationStatus::Fulfilled);
        updated.purpose = "rewritten purpose".into();
        let observations = [AttestationRequestUpdateObservationV1 {
            request: &updated,
            author_did: SUBJECT,
            action_timestamp_micros: 500,
            parent_is_canonical_root: true,
        }];
        assert!(matches!(
            resolve_attestation_request_lineage_v1(lineage(&request, &observations, 0, 500)),
            Err(AttestationRequestResolutionErrorV1::UpdatePolicyRejected { index: 0, .. })
        ));
    }

    #[test]
    fn delete_is_unresolved_not_cancelled() {
        let request = root();
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &[], 1, 500)),
            Err(AttestationRequestResolutionErrorV1::HistoricalDeleteObserved {
                count: 1,
            })
        );
    }

    #[test]
    fn nested_or_nonroot_update_is_unresolved() {
        let request = root();
        let updated = with_status(&request, AttestationStatus::Fulfilled);
        let observations = [AttestationRequestUpdateObservationV1 {
            request: &updated,
            author_did: SUBJECT,
            action_timestamp_micros: 500,
            parent_is_canonical_root: false,
        }];
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &observations, 0, 500)),
            Err(AttestationRequestResolutionErrorV1::NonRootUpdateObserved {
                index: 0,
            })
        );
    }

    #[test]
    fn concurrent_terminal_siblings_are_conflict_even_when_status_matches() {
        let request = root();
        let first = with_status(&request, AttestationStatus::Fulfilled);
        let second = with_status(&request, AttestationStatus::Fulfilled);
        let observations = [
            AttestationRequestUpdateObservationV1 {
                request: &first,
                author_did: SUBJECT,
                action_timestamp_micros: 500,
                parent_is_canonical_root: true,
            },
            AttestationRequestUpdateObservationV1 {
                request: &second,
                author_did: SUBJECT,
                action_timestamp_micros: 600,
                parent_is_canonical_root: true,
            },
        ];
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &observations, 0, 700)),
            Ok(ObservedAttestationRequestStateV1::ObservedConflict {
                terminal_updates: 2,
            })
        );
    }

    #[test]
    fn different_terminal_siblings_are_conflict_not_latest_wins() {
        let request = root();
        let fulfilled = with_status(&request, AttestationStatus::Fulfilled);
        let declined = with_status(&request, AttestationStatus::Declined);
        let observations = [
            AttestationRequestUpdateObservationV1 {
                request: &fulfilled,
                author_did: SUBJECT,
                action_timestamp_micros: 500,
                parent_is_canonical_root: true,
            },
            AttestationRequestUpdateObservationV1 {
                request: &declined,
                author_did: SUBJECT,
                action_timestamp_micros: 900,
                parent_is_canonical_root: true,
            },
        ];
        assert_eq!(
            resolve_attestation_request_lineage_v1(lineage(&request, &observations, 0, 950)),
            Ok(ObservedAttestationRequestStateV1::ObservedConflict {
                terminal_updates: 2,
            })
        );
    }

    #[test]
    fn duplicate_request_ids_fail_subject_aggregation() {
        let first = root();
        let mut second = root();
        second.requester_did = "did:mycelix:other-requester".into();
        assert_eq!(
            validate_unique_request_ids_v1(&[&first, &second]),
            Err(AttestationRequestResolutionErrorV1::DuplicateRequestIdObserved {
                first_index: 0,
                second_index: 1,
            })
        );
    }

    #[test]
    fn status_mapping_is_exhaustive() {
        let cases = [
            (
                AttestationStatus::Pending,
                AttestationRequestStatusV1::Pending,
            ),
            (
                AttestationStatus::Fulfilled,
                AttestationRequestStatusV1::Fulfilled,
            ),
            (
                AttestationStatus::Declined,
                AttestationRequestStatusV1::Declined,
            ),
            (
                AttestationStatus::Expired,
                AttestationRequestStatusV1::Expired,
            ),
            (
                AttestationStatus::Cancelled,
                AttestationRequestStatusV1::Cancelled,
            ),
        ];
        for (local, canonical) in cases {
            assert_eq!(status_v1(&local), canonical);
        }
    }
}
