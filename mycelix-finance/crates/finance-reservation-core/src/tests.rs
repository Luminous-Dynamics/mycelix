use std::collections::BTreeSet;
use std::fmt::Write;

use mycelix_business_core::{
    ActionContractRef, AuthorizedIntentRef, Digest32, ExecutionAttemptRef, ReferenceId,
    ReservationId, SubjectRef,
};
use mycelix_business_decision::DecisionCapsuleRef;
use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_settlement::FinalityProfileRef;

use super::*;

fn reference(value: &str) -> ReferenceId {
    ReferenceId::new(value).expect("static canonical reference")
}

fn descriptor() -> ReservationDescriptor {
    ReservationDescriptor {
        commitment_profile_revision: RESERVATION_COMMITMENT_PROFILE_REVISION,
        reservation_id: ReservationId(reference("reservation:1")),
        request_id: reference("request:1"),
        finance_domain: reference("finance"),
        issuer_authority: reference("finance-authority:alpha"),
        issuer_profile: reference("issuer:finance:v1"),
        issuer_profile_revision: 1,
        issuer_profile_digest: Digest32::repeat(5),
        finance_policy_sequence: 9,
        finance_policy_digest: Digest32::repeat(6),
        subject: SubjectRef(reference("treasury:operating")),
        amount: AssetAmount::new(
            100,
            AssetId::new("USD.micro").expect("static asset identifier"),
        ),
        effect_class: reference("finance-effect:payment"),
        action_contract: ActionContractRef {
            semantic_id: reference("mycelix.finance.pay.v1"),
            digest: Digest32::repeat(1),
        },
        decision: DecisionCapsuleRef {
            id: reference("decision:1"),
            digest: Digest32::repeat(2),
        },
        authorized_intent: AuthorizedIntentRef(reference("intent:1")),
        intent_digest: Digest32::repeat(3),
        authority_lease_id: reference("authority:lease:1"),
        authority_epoch: 7,
        fencing_token: 11,
        aggregate_policy_keys: BTreeSet::from([
            reference("aggregate:counterparty:24h"),
            reference("aggregate:treasury:24h"),
        ]),
        idempotency_key: reference("payment:idem:1"),
        required_finality_profile: FinalityProfileRef {
            id: reference("finality:bank:v1"),
            revision: 1,
            digest: Digest32::repeat(4),
        },
        issued_at_unix_ms: 100,
        expires_at_unix_ms: 1_000,
    }
}

fn attempt(value: &str) -> ExecutionAttemptRef {
    ExecutionAttemptRef(reference(value))
}

fn to_hex(bytes: &[u8]) -> String {
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        write!(&mut output, "{byte:02x}").expect("write to String cannot fail");
    }
    output
}

fn consume(attempt_id: &str, idempotency: &str, at: u64) -> ReservationTransition {
    ReservationTransition::Consume {
        attempt: attempt(attempt_id),
        idempotency_key: reference(idempotency),
        at_unix_ms: at,
    }
}

#[test]
fn canonical_descriptor_golden_vector_is_stable() {
    let descriptor = descriptor();
    let bytes = canonical_descriptor_bytes(&descriptor).expect("valid descriptor");
    let commitment = reservation_commitment(&descriptor).expect("valid descriptor");

    assert_eq!(bytes.len(), 621);
    assert_eq!(
        to_hex(commitment.as_bytes()),
        "9fca16ee26def839f6d98d8b413c126a1fdd1267f8d6c065c7515941737538cd"
    );
}

#[test]
fn initial_and_consumed_state_golden_vectors_are_stable() {
    let descriptor = descriptor();
    let initial = ReservationState::initial(&descriptor).expect("valid initial state");
    assert_eq!(initial.sequence(), 1);
    assert_eq!(
        to_hex(initial.state_commitment().as_bytes()),
        "03e797473a7cf8cb8b1ce4fdd6170363c135b28765abcb553eb76fcd7c7599bf"
    );

    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("valid consume")
    .into_state();

    assert_eq!(consumed.sequence(), 2);
    assert_eq!(
        to_hex(consumed.state_commitment().as_bytes()),
        "5b2aedc5f5fd483a1ff2053057871ddf7be1ce30c3a52aac41c70ab472df2a1d"
    );
}

#[test]
fn published_json_vector_matches_executable_commitments() {
    let vector: serde_json::Value = serde_json::from_str(include_str!(
        "../test-vectors/reservation-v1.json"
    ))
    .expect("checked-in vector must be valid JSON");

    let descriptor = descriptor();
    let canonical = canonical_descriptor_bytes(&descriptor).expect("valid descriptor");
    let descriptor_hex = to_hex(
        reservation_commitment(&descriptor)
            .expect("valid descriptor")
            .as_bytes(),
    );
    let initial = ReservationState::initial(&descriptor).expect("valid initial state");
    let initial_hex = to_hex(initial.state_commitment().as_bytes());
    let consumed = apply_reservation_transition(
        &descriptor,
        &initial,
        initial.expected(),
        consume("attempt:1", "payment:idem:1", 400),
    )
    .expect("valid consume")
    .into_state();
    let consumed_hex = to_hex(consumed.state_commitment().as_bytes());

    assert_eq!(
        vector["expected"]["canonical_descriptor_length"].as_u64(),
        Some(canonical.len() as u64)
    );
    assert_eq!(
        vector["expected"]["descriptor_commitment_hex"].as_str(),
        Some(descriptor_hex.as_str())
    );
    assert_eq!(
        vector["expected"]["initial_state"]["state_commitment_hex"].as_str(),
        Some(initial_hex.as_str())
    );
    assert_eq!(
        vector["expected"]["consumed_state"]["state_commitment_hex"].as_str(),
        Some(consumed_hex.as_str())
    );
    assert_eq!(
        vector["descriptor"]["reservation_id"].as_str(),
        Some(descriptor.reservation_id.0.as_str())
    );
    assert_eq!(
        vector["descriptor"]["issuer_authority"].as_str(),
        Some(descriptor.issuer_authority.as_str())
    );
    assert_eq!(
        vector["descriptor"]["issuer_profile_revision"].as_u64(),
        Some(descriptor.issuer_profile_revision)
    );
    assert_eq!(
        vector["descriptor"]["finance_policy_sequence"].as_u64(),
        Some(descriptor.finance_policy_sequence)
    );
    assert_eq!(
        vector["descriptor"]["amount"]["asset"].as_str(),
        Some(descriptor.amount.asset().as_str())
    );
    assert_eq!(
        vector["descriptor"]["amount"]["atomic_units"].as_u64(),
        Some(descriptor.amount.atomic_units())
    );
}

include!("tests_descriptor.inc");
include!("tests_lifecycle.inc");
include!("tests_reconstruction.inc");