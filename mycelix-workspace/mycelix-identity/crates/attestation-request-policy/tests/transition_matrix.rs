// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_attestation_request_policy::{
    AttestationRequestActorV1, AttestationRequestAssertionEqualityV1, AttestationRequestStatusV1,
    AttestationRequestTransitionErrorV1, AttestationRequestTransitionV1,
    validate_attestation_request_transition_v1,
};

const STATUSES: [AttestationRequestStatusV1; 5] = [
    AttestationRequestStatusV1::Pending,
    AttestationRequestStatusV1::Fulfilled,
    AttestationRequestStatusV1::Declined,
    AttestationRequestStatusV1::Expired,
    AttestationRequestStatusV1::Cancelled,
];

const ACTORS: [AttestationRequestActorV1; 3] = [
    AttestationRequestActorV1::Requester,
    AttestationRequestActorV1::Subject,
    AttestationRequestActorV1::Other,
];

fn transition(
    original_status: AttestationRequestStatusV1,
    updated_status: AttestationRequestStatusV1,
    actor: AttestationRequestActorV1,
) -> AttestationRequestTransitionV1 {
    AttestationRequestTransitionV1 {
        original_status,
        updated_status,
        actor,
        assertion_equality: AttestationRequestAssertionEqualityV1::all_equal(),
        update_timestamp_micros: 1_000,
        expires_at_micros: 1_000,
    }
}

fn pending_transition_should_pass(
    updated_status: AttestationRequestStatusV1,
    actor: AttestationRequestActorV1,
) -> bool {
    match updated_status {
        AttestationRequestStatusV1::Pending => false,
        AttestationRequestStatusV1::Fulfilled | AttestationRequestStatusV1::Declined => {
            actor == AttestationRequestActorV1::Subject
        }
        AttestationRequestStatusV1::Expired => {
            actor == AttestationRequestActorV1::Requester
                || actor == AttestationRequestActorV1::Subject
        }
        AttestationRequestStatusV1::Cancelled => {
            actor == AttestationRequestActorV1::Requester
        }
    }
}

#[test]
fn pending_actor_target_truth_table_is_exhaustive() {
    for updated_status in STATUSES {
        for actor in ACTORS {
            let result = validate_attestation_request_transition_v1(transition(
                AttestationRequestStatusV1::Pending,
                updated_status,
                actor,
            ));
            assert_eq!(
                result.is_ok(),
                pending_transition_should_pass(updated_status, actor),
                "unexpected Pending -> {updated_status:?} disposition for {actor:?}: {result:?}"
            );
        }
    }
}

#[test]
fn every_terminal_source_rejects_every_actor_and_target() {
    for original_status in [
        AttestationRequestStatusV1::Fulfilled,
        AttestationRequestStatusV1::Declined,
        AttestationRequestStatusV1::Expired,
        AttestationRequestStatusV1::Cancelled,
    ] {
        for updated_status in STATUSES {
            for actor in ACTORS {
                assert_eq!(
                    validate_attestation_request_transition_v1(transition(
                        original_status,
                        updated_status,
                        actor,
                    )),
                    Err(AttestationRequestTransitionErrorV1::OriginalNotPending),
                    "terminal source {original_status:?} unexpectedly admitted target {updated_status:?} for {actor:?}"
                );
            }
        }
    }
}
