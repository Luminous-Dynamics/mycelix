// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Dependency-free lifecycle policy for Mycelix attestation requests.
//!
//! This crate owns the status/actor/time theorem shared by DHT write validation
//! and network-aware read-side resolution. It intentionally knows nothing about
//! Holochain actions, entry types, serialization, transport, credential issuance,
//! or proof verification.
//!
//! Adapters must provide an explicit equality witness for every requester-authored
//! assertion/audit field. That keeps immutability visible at the boundary instead
//! of collapsing it into a vague "request unchanged" boolean.

#![forbid(unsafe_code)]

/// Canonical V1 request lifecycle states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttestationRequestStatusV1 {
    Pending,
    Fulfilled,
    Declined,
    Expired,
    Cancelled,
}

/// Canonical actor relation to the immutable request assertion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttestationRequestActorV1 {
    Requester,
    Subject,
    Other,
}

/// Classify an update author relative to the immutable request participants.
///
/// Requester identity wins if a malformed historical request contains the same DID
/// for requester and subject. Creation policy should already forbid self-requests;
/// this ordering therefore keeps the lifecycle theorem deterministic without
/// silently granting subject authority to malformed data.
pub fn classify_attestation_request_actor_v1(
    committer_did: &str,
    requester_did: &str,
    subject_did: &str,
) -> AttestationRequestActorV1 {
    if committer_did == requester_did {
        AttestationRequestActorV1::Requester
    } else if committer_did == subject_did {
        AttestationRequestActorV1::Subject
    } else {
        AttestationRequestActorV1::Other
    }
}

/// Immutable requester-authored fields covered by the V1 lifecycle contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AttestationRequestImmutableFieldV1 {
    Id,
    RequesterDid,
    SubjectDid,
    Components,
    MinTrustScore,
    MinTier,
    Purpose,
    ExpiresAt,
    CreatedAt,
}

/// Explicit equality witness supplied by an adapter for every immutable field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttestationRequestAssertionEqualityV1 {
    pub id: bool,
    pub requester_did: bool,
    pub subject_did: bool,
    pub components: bool,
    pub min_trust_score: bool,
    pub min_tier: bool,
    pub purpose: bool,
    pub expires_at: bool,
    pub created_at: bool,
}

impl AttestationRequestAssertionEqualityV1 {
    /// Convenience constructor for tests/callers that have already established
    /// exact equality across all V1 immutable fields.
    pub const fn all_equal() -> Self {
        Self {
            id: true,
            requester_did: true,
            subject_did: true,
            components: true,
            min_trust_score: true,
            min_tier: true,
            purpose: true,
            expires_at: true,
            created_at: true,
        }
    }

    fn first_changed_field(self) -> Option<AttestationRequestImmutableFieldV1> {
        let fields = [
            (self.id, AttestationRequestImmutableFieldV1::Id),
            (
                self.requester_did,
                AttestationRequestImmutableFieldV1::RequesterDid,
            ),
            (
                self.subject_did,
                AttestationRequestImmutableFieldV1::SubjectDid,
            ),
            (
                self.components,
                AttestationRequestImmutableFieldV1::Components,
            ),
            (
                self.min_trust_score,
                AttestationRequestImmutableFieldV1::MinTrustScore,
            ),
            (self.min_tier, AttestationRequestImmutableFieldV1::MinTier),
            (self.purpose, AttestationRequestImmutableFieldV1::Purpose),
            (
                self.expires_at,
                AttestationRequestImmutableFieldV1::ExpiresAt,
            ),
            (
                self.created_at,
                AttestationRequestImmutableFieldV1::CreatedAt,
            ),
        ];

        fields
            .into_iter()
            .find_map(|(equal, field)| (!equal).then_some(field))
    }
}

/// Canonical inputs to the V1 lifecycle theorem.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttestationRequestTransitionV1 {
    pub original_status: AttestationRequestStatusV1,
    pub updated_status: AttestationRequestStatusV1,
    pub actor: AttestationRequestActorV1,
    pub assertion_equality: AttestationRequestAssertionEqualityV1,
    /// Update action timestamp in microseconds since the Unix epoch.
    pub update_timestamp_micros: i64,
    /// Immutable request expiration timestamp in the same unit/domain.
    pub expires_at_micros: i64,
}

/// Fail-closed reasons for rejecting a request transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttestationRequestTransitionErrorV1 {
    ImmutableAssertionChanged(AttestationRequestImmutableFieldV1),
    OriginalNotPending,
    SameStateUpdate,
    SubjectRequired,
    RequesterRequired,
    RequesterOrSubjectRequired,
    ExpiredTooEarly,
}

/// Validate one V1 request lifecycle transition.
///
/// Security theorem:
///
/// - requester-authored assertion/audit fields never change;
/// - only `Pending` may transition;
/// - fulfillment/decline are subject actions;
/// - cancellation is a requester action;
/// - expiration is requester-or-subject and cannot predate `expires_at`;
/// - same-state and terminal-source updates are rejected.
pub fn validate_attestation_request_transition_v1(
    transition: AttestationRequestTransitionV1,
) -> Result<(), AttestationRequestTransitionErrorV1> {
    if let Some(field) = transition.assertion_equality.first_changed_field() {
        return Err(AttestationRequestTransitionErrorV1::ImmutableAssertionChanged(
            field,
        ));
    }

    if transition.original_status != AttestationRequestStatusV1::Pending {
        return Err(AttestationRequestTransitionErrorV1::OriginalNotPending);
    }

    match transition.updated_status {
        AttestationRequestStatusV1::Pending => {
            Err(AttestationRequestTransitionErrorV1::SameStateUpdate)
        }
        AttestationRequestStatusV1::Fulfilled | AttestationRequestStatusV1::Declined => {
            if transition.actor == AttestationRequestActorV1::Subject {
                Ok(())
            } else {
                Err(AttestationRequestTransitionErrorV1::SubjectRequired)
            }
        }
        AttestationRequestStatusV1::Cancelled => {
            if transition.actor == AttestationRequestActorV1::Requester {
                Ok(())
            } else {
                Err(AttestationRequestTransitionErrorV1::RequesterRequired)
            }
        }
        AttestationRequestStatusV1::Expired => {
            if transition.actor != AttestationRequestActorV1::Requester
                && transition.actor != AttestationRequestActorV1::Subject
            {
                return Err(AttestationRequestTransitionErrorV1::RequesterOrSubjectRequired);
            }
            if transition.update_timestamp_micros < transition.expires_at_micros {
                return Err(AttestationRequestTransitionErrorV1::ExpiredTooEarly);
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transition(
        updated_status: AttestationRequestStatusV1,
        actor: AttestationRequestActorV1,
    ) -> AttestationRequestTransitionV1 {
        AttestationRequestTransitionV1 {
            original_status: AttestationRequestStatusV1::Pending,
            updated_status,
            actor,
            assertion_equality: AttestationRequestAssertionEqualityV1::all_equal(),
            update_timestamp_micros: 500,
            expires_at_micros: 1_000,
        }
    }

    #[test]
    fn actor_classification_is_exact_and_requester_first() {
        assert_eq!(
            classify_attestation_request_actor_v1("did:r", "did:r", "did:s"),
            AttestationRequestActorV1::Requester
        );
        assert_eq!(
            classify_attestation_request_actor_v1("did:s", "did:r", "did:s"),
            AttestationRequestActorV1::Subject
        );
        assert_eq!(
            classify_attestation_request_actor_v1("did:o", "did:r", "did:s"),
            AttestationRequestActorV1::Other
        );
        assert_eq!(
            classify_attestation_request_actor_v1("did:same", "did:same", "did:same"),
            AttestationRequestActorV1::Requester
        );
    }

    #[test]
    fn subject_may_fulfill_or_decline() {
        for status in [
            AttestationRequestStatusV1::Fulfilled,
            AttestationRequestStatusV1::Declined,
        ] {
            assert_eq!(
                validate_attestation_request_transition_v1(transition(
                    status,
                    AttestationRequestActorV1::Subject,
                )),
                Ok(())
            );
        }
    }

    #[test]
    fn requester_may_cancel() {
        assert_eq!(
            validate_attestation_request_transition_v1(transition(
                AttestationRequestStatusV1::Cancelled,
                AttestationRequestActorV1::Requester,
            )),
            Ok(())
        );
    }

    #[test]
    fn requester_or_subject_may_expire_at_boundary() {
        for actor in [
            AttestationRequestActorV1::Requester,
            AttestationRequestActorV1::Subject,
        ] {
            let mut input = transition(AttestationRequestStatusV1::Expired, actor);
            input.update_timestamp_micros = input.expires_at_micros;
            assert_eq!(validate_attestation_request_transition_v1(input), Ok(()));
        }
    }

    #[test]
    fn expiration_before_boundary_is_rejected() {
        let mut input = transition(
            AttestationRequestStatusV1::Expired,
            AttestationRequestActorV1::Subject,
        );
        input.update_timestamp_micros = 999;
        assert_eq!(
            validate_attestation_request_transition_v1(input),
            Err(AttestationRequestTransitionErrorV1::ExpiredTooEarly)
        );
    }

    #[test]
    fn requester_cannot_fulfill_or_decline() {
        for status in [
            AttestationRequestStatusV1::Fulfilled,
            AttestationRequestStatusV1::Declined,
        ] {
            assert_eq!(
                validate_attestation_request_transition_v1(transition(
                    status,
                    AttestationRequestActorV1::Requester,
                )),
                Err(AttestationRequestTransitionErrorV1::SubjectRequired)
            );
        }
    }

    #[test]
    fn subject_cannot_cancel() {
        assert_eq!(
            validate_attestation_request_transition_v1(transition(
                AttestationRequestStatusV1::Cancelled,
                AttestationRequestActorV1::Subject,
            )),
            Err(AttestationRequestTransitionErrorV1::RequesterRequired)
        );
    }

    #[test]
    fn unrelated_actor_cannot_expire_request() {
        let mut input = transition(
            AttestationRequestStatusV1::Expired,
            AttestationRequestActorV1::Other,
        );
        input.update_timestamp_micros = input.expires_at_micros;
        assert_eq!(
            validate_attestation_request_transition_v1(input),
            Err(AttestationRequestTransitionErrorV1::RequesterOrSubjectRequired)
        );
    }

    #[test]
    fn same_state_pending_update_is_rejected() {
        assert_eq!(
            validate_attestation_request_transition_v1(transition(
                AttestationRequestStatusV1::Pending,
                AttestationRequestActorV1::Subject,
            )),
            Err(AttestationRequestTransitionErrorV1::SameStateUpdate)
        );
    }

    #[test]
    fn every_terminal_source_state_is_rejected() {
        for original_status in [
            AttestationRequestStatusV1::Fulfilled,
            AttestationRequestStatusV1::Declined,
            AttestationRequestStatusV1::Expired,
            AttestationRequestStatusV1::Cancelled,
        ] {
            let mut input = transition(
                AttestationRequestStatusV1::Declined,
                AttestationRequestActorV1::Subject,
            );
            input.original_status = original_status;
            assert_eq!(
                validate_attestation_request_transition_v1(input),
                Err(AttestationRequestTransitionErrorV1::OriginalNotPending)
            );
        }
    }

    #[test]
    fn every_immutable_field_is_independently_enforced() {
        let cases = [
            (
                AttestationRequestImmutableFieldV1::Id,
                AttestationRequestAssertionEqualityV1 {
                    id: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::RequesterDid,
                AttestationRequestAssertionEqualityV1 {
                    requester_did: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::SubjectDid,
                AttestationRequestAssertionEqualityV1 {
                    subject_did: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::Components,
                AttestationRequestAssertionEqualityV1 {
                    components: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::MinTrustScore,
                AttestationRequestAssertionEqualityV1 {
                    min_trust_score: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::MinTier,
                AttestationRequestAssertionEqualityV1 {
                    min_tier: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::Purpose,
                AttestationRequestAssertionEqualityV1 {
                    purpose: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::ExpiresAt,
                AttestationRequestAssertionEqualityV1 {
                    expires_at: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
            (
                AttestationRequestImmutableFieldV1::CreatedAt,
                AttestationRequestAssertionEqualityV1 {
                    created_at: false,
                    ..AttestationRequestAssertionEqualityV1::all_equal()
                },
            ),
        ];

        for (expected_field, equality) in cases {
            let mut input = transition(
                AttestationRequestStatusV1::Fulfilled,
                AttestationRequestActorV1::Subject,
            );
            input.assertion_equality = equality;
            assert_eq!(
                validate_attestation_request_transition_v1(input),
                Err(AttestationRequestTransitionErrorV1::ImmutableAssertionChanged(
                    expected_field
                ))
            );
        }
    }

    #[test]
    fn non_expiration_transitions_do_not_require_expiry_boundary() {
        let input = transition(
            AttestationRequestStatusV1::Fulfilled,
            AttestationRequestActorV1::Subject,
        );
        assert!(input.update_timestamp_micros < input.expires_at_micros);
        assert_eq!(validate_attestation_request_transition_v1(input), Ok(()));
    }
}
