//! FIN-ECO-004B runtime qualification against a real Holochain conductor.
//!
//! These tests are ignored by default because they require the packed Finance
//! DNA. The dedicated qualification workflow builds/packs the exact product head
//! before running them.

use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use holochain::prelude::*;
use holochain::sweettest::*;
use reservation_authority::{
    AuthenticatedReservationConsumption, ConsumeReservationInput, EvidenceTransitionInput,
    ExpireReservationInput, IssueReservationInput, ReservationStateView,
    ReservationTransitionReceipt,
};
use reservation_authority_integrity::{
    ReservationDescriptorWire, ReservationLifecycleWire,
};

fn dna_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("dna")
        .join("mycelix_finance.dna")
}

async fn load_dna() -> DnaFile {
    SweetDnaFile::from_bundle(&dna_path())
        .await
        .expect("Failed to load Finance DNA bundle — pack dna/ first")
}

async fn setup() -> (SweetConductor, Vec<AgentPubKey>, Vec<SweetApp>) {
    let dna = load_dna().await;
    let mut conductor = SweetConductor::from_standard_config().await;
    let agents = SweetAgents::get(conductor.keystore(), 2).await;
    let apps = conductor
        .setup_app_for_agents("fin-eco-reservation-authority", &agents, &[dna])
        .await
        .expect("install Finance app for reservation authority test");
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
    (conductor, agents, apps.into())
}

fn unix_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("test clock before Unix epoch")
        .as_millis() as u64
}

fn descriptor(reservation_id: &str) -> ReservationDescriptorWire {
    descriptor_expiring_at(reservation_id, unix_ms() + 120_000)
}

fn descriptor_expiring_at(
    reservation_id: &str,
    expires_at_unix_ms: u64,
) -> ReservationDescriptorWire {
    ReservationDescriptorWire {
        commitment_profile_revision: 1,
        reservation_id: reservation_id.into(),
        request_id: format!("request:{reservation_id}"),
        finance_domain: "finance".into(),
        // Runtime overwrites this from the issuer cell's AgentPubKey.
        issuer_authority: "runtime-overwrites-this".into(),
        issuer_profile: "issuer:finance:test:v1".into(),
        issuer_profile_revision: 1,
        issuer_profile_digest: [5; 32],
        finance_policy_sequence: 1,
        finance_policy_digest: [6; 32],
        subject: "treasury:test".into(),
        asset: "USD.micro".into(),
        atomic_units: 100,
        effect_class: "finance-effect:test-payment".into(),
        action_contract_semantic_id: "mycelix.finance.test-pay.v1".into(),
        action_contract_digest: [1; 32],
        decision_id: "decision:test".into(),
        decision_digest: [2; 32],
        authorized_intent: "intent:test".into(),
        intent_digest: [3; 32],
        authority_lease_id: "authority:lease:test".into(),
        authority_epoch: 1,
        fencing_token: 1,
        aggregate_policy_keys: vec![
            "aggregate:counterparty:test".into(),
            "aggregate:treasury:test".into(),
        ],
        idempotency_key: format!("idem:{reservation_id}"),
        required_finality_profile_id: "finality:test:v1".into(),
        required_finality_profile_revision: 1,
        required_finality_profile_digest: [4; 32],
        issued_at_unix_ms: 0,
        expires_at_unix_ms,
    }
}

async fn issue(
    conductor: &SweetConductor,
    zome: &SweetZome,
    reservation_id: &str,
) -> ReservationStateView {
    conductor
        .call(
            zome,
            "issue_finance_reservation",
            IssueReservationInput {
                descriptor: descriptor(reservation_id),
            },
        )
        .await
}

async fn issue_with_expiry(
    conductor: &SweetConductor,
    zome: &SweetZome,
    reservation_id: &str,
    expires_at_unix_ms: u64,
) -> ReservationStateView {
    conductor
        .call(
            zome,
            "issue_finance_reservation",
            IssueReservationInput {
                descriptor: descriptor_expiring_at(reservation_id, expires_at_unix_ms),
            },
        )
        .await
}

fn consume_input(
    issued: &ReservationStateView,
    attempt: &str,
) -> ConsumeReservationInput {
    ConsumeReservationInput {
        root_action: issued.root_action.clone(),
        expected_descriptor_commitment: issued.descriptor_commitment,
        expected_sequence: issued.state_sequence,
        expected_state_commitment: issued.state_commitment,
        attempt: attempt.into(),
        idempotency_key: issued.descriptor.idempotency_key.clone(),
    }
}

fn evidence_input(
    issued: &ReservationStateView,
    evidence: &str,
) -> EvidenceTransitionInput {
    EvidenceTransitionInput {
        root_action: issued.root_action.clone(),
        expected_descriptor_commitment: issued.descriptor_commitment,
        expected_sequence: issued.state_sequence,
        expected_state_commitment: issued.state_commitment,
        evidence: evidence.into(),
    }
}

fn expire_input(issued: &ReservationStateView) -> ExpireReservationInput {
    ExpireReservationInput {
        root_action: issued.root_action.clone(),
        expected_descriptor_commitment: issued.descriptor_commitment,
        expected_sequence: issued.state_sequence,
        expected_state_commitment: issued.state_commitment,
    }
}

#[test]
#[ignore]
fn fin_eco_004b_runtime_reservation_authority() {
    std::thread::Builder::new()
        .stack_size(16 * 1024 * 1024)
        .spawn(|| {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap()
                .block_on(runtime_scenario());
        })
        .unwrap()
        .join()
        .unwrap();
}

async fn runtime_scenario() {
    let (conductor, agents, apps) = setup().await;
    let issuer = apps[0].cells()[0].zome("reservation_authority");
    let observer = apps[1].cells()[0].zome("reservation_authority");

    // 1. Runtime derives the canonical issuer identity from the actual author.
    let issued = issue(&conductor, &issuer, "reservation:runtime:one").await;
    assert_eq!(issued.issuer_agent, agents[0]);
    assert_eq!(
        issued.descriptor.issuer_authority,
        format!("did:mycelix:{}", agents[0])
    );
    assert!(matches!(issued.lifecycle, ReservationLifecycleWire::Active));
    assert_eq!(issued.state_sequence, 1);

    // 2. Duplicate issuance on the same issuer source chain fails closed.
    let duplicate: Result<ReservationStateView, _> = conductor
        .call_fallible(
            &issuer,
            "issue_finance_reservation",
            IssueReservationInput {
                descriptor: descriptor("reservation:runtime:one"),
            },
        )
        .await;
    assert!(duplicate.is_err(), "duplicate reservation ID must be rejected");

    // 3. Consume once. The returned proof carries the exact economic descriptor
    // and the Holochain transition receipt under the same descriptor commitment.
    let input = consume_input(&issued, "attempt:runtime:one");
    let first: AuthenticatedReservationConsumption = conductor
        .call(&issuer, "consume_finance_reservation", input.clone())
        .await;
    assert_eq!(first.descriptor.reservation_id, issued.descriptor.reservation_id);
    assert_eq!(first.descriptor.asset, "USD.micro");
    assert_eq!(first.descriptor.atomic_units, 100);
    assert_eq!(first.descriptor.effect_class, "finance-effect:test-payment");
    assert_eq!(first.descriptor.authority_epoch, 1);
    assert_eq!(first.descriptor.fencing_token, 1);
    assert_eq!(
        first.transition.descriptor_commitment,
        issued.descriptor_commitment
    );
    assert!(!first.transition.idempotent_replay);
    assert_eq!(first.transition.root_action, issued.root_action);
    assert_eq!(first.transition.prior_action, issued.root_action);
    assert_eq!(first.transition.resulting_sequence, 2);
    assert!(matches!(
        first.transition.resulting_lifecycle,
        ReservationLifecycleWire::Consumed { .. }
    ));

    // Exact same-attempt replay returns the same terminal action and commitment.
    let replay: AuthenticatedReservationConsumption = conductor
        .call(&issuer, "consume_finance_reservation", input.clone())
        .await;
    assert!(replay.transition.idempotent_replay);
    assert_eq!(
        replay.transition.transition_action,
        first.transition.transition_action
    );
    assert_eq!(
        replay.transition.resulting_state_commitment,
        first.transition.resulting_state_commitment
    );
    assert_eq!(replay.descriptor, first.descriptor);

    let unrelated_replay: Result<AuthenticatedReservationConsumption, _> = conductor
        .call_fallible(
            &issuer,
            "consume_finance_reservation",
            consume_input(&issued, "attempt:runtime:other"),
        )
        .await;
    assert!(
        unrelated_replay.is_err(),
        "different attempt must not reuse consumed reservation"
    );

    // Unknown external outcome is not permission to restore consumed capacity.
    let implicit_release: Result<ReservationTransitionReceipt, _> = conductor
        .call_fallible(
            &issuer,
            "release_finance_reservation",
            evidence_input(&issued, "evidence:external-outcome-unknown"),
        )
        .await;
    assert!(
        implicit_release.is_err(),
        "consumed reservation must not be implicitly restored by release"
    );

    // 4. Another cell can observe the state, but cannot mutate it because it is
    // not the issuer source chain for this reservation.
    tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    let observed: ReservationStateView = conductor
        .call(
            &observer,
            "get_finance_reservation_state",
            issued.root_action.clone(),
        )
        .await;
    assert_eq!(observed.state_sequence, 2);
    assert_eq!(
        observed.current_action,
        first.transition.transition_action
    );

    let foreign_mutation: Result<AuthenticatedReservationConsumption, _> = conductor
        .call_fallible(
            &observer,
            "consume_finance_reservation",
            consume_input(&issued, "attempt:foreign"),
        )
        .await;
    assert!(foreign_mutation.is_err(), "non-issuer cell must not mutate reservation");

    // 5. Competing consumes against one Active reservation: exactly one wins.
    let raced = issue(&conductor, &issuer, "reservation:runtime:race").await;
    let a = consume_input(&raced, "attempt:race:a");
    let b = consume_input(&raced, "attempt:race:b");

    let (ra, rb): (
        Result<AuthenticatedReservationConsumption, _>,
        Result<AuthenticatedReservationConsumption, _>,
    ) = tokio::join!(
        conductor.call_fallible(&issuer, "consume_finance_reservation", a),
        conductor.call_fallible(&issuer, "consume_finance_reservation", b),
    );

    let success_count = (ra.is_ok() as usize) + (rb.is_ok() as usize);
    assert_eq!(success_count, 1, "exactly one competing consume should commit");

    let final_state: ReservationStateView = conductor
        .call(
            &issuer,
            "get_finance_reservation_state",
            raced.root_action.clone(),
        )
        .await;
    assert_eq!(final_state.state_sequence, 2);
    assert!(matches!(
        final_state.lifecycle,
        ReservationLifecycleWire::Consumed { .. }
    ));

    // 6. Consume vs release: there is still exactly one terminal lineage.
    let consume_release = issue(&conductor, &issuer, "reservation:runtime:consume-release").await;
    let consume = consume_input(&consume_release, "attempt:consume-release");
    let release = evidence_input(&consume_release, "evidence:release-race");
    let (consume_result, release_result): (
        Result<AuthenticatedReservationConsumption, _>,
        Result<ReservationTransitionReceipt, _>,
    ) = tokio::join!(
        conductor.call_fallible(&issuer, "consume_finance_reservation", consume),
        conductor.call_fallible(&issuer, "release_finance_reservation", release),
    );
    assert_eq!(
        (consume_result.is_ok() as usize) + (release_result.is_ok() as usize),
        1,
        "consume vs release must produce one terminal effect"
    );
    let state: ReservationStateView = conductor
        .call(
            &issuer,
            "get_finance_reservation_state",
            consume_release.root_action.clone(),
        )
        .await;
    assert_eq!(state.state_sequence, 2);
    assert!(matches!(
        state.lifecycle,
        ReservationLifecycleWire::Consumed { .. } | ReservationLifecycleWire::Released { .. }
    ));

    // 7. Consume vs revoke: there is still exactly one terminal lineage.
    let consume_revoke = issue(&conductor, &issuer, "reservation:runtime:consume-revoke").await;
    let consume = consume_input(&consume_revoke, "attempt:consume-revoke");
    let revoke = evidence_input(&consume_revoke, "evidence:revoke-race");
    let (consume_result, revoke_result): (
        Result<AuthenticatedReservationConsumption, _>,
        Result<ReservationTransitionReceipt, _>,
    ) = tokio::join!(
        conductor.call_fallible(&issuer, "consume_finance_reservation", consume),
        conductor.call_fallible(&issuer, "revoke_finance_reservation", revoke),
    );
    assert_eq!(
        (consume_result.is_ok() as usize) + (revoke_result.is_ok() as usize),
        1,
        "consume vs revoke must produce one terminal effect"
    );
    let state: ReservationStateView = conductor
        .call(
            &issuer,
            "get_finance_reservation_state",
            consume_revoke.root_action.clone(),
        )
        .await;
    assert_eq!(state.state_sequence, 2);
    assert!(matches!(
        state.lifecycle,
        ReservationLifecycleWire::Consumed { .. } | ReservationLifecycleWire::Revoked { .. }
    ));

    // 8. Once the expiry boundary has passed, consumption must fail and explicit
    // expiry may commit the sole terminal transition.
    let expiry_at = unix_ms() + 1_500;
    let expiring = issue_with_expiry(
        &conductor,
        &issuer,
        "reservation:runtime:expiry",
        expiry_at,
    )
    .await;
    tokio::time::sleep(std::time::Duration::from_millis(1_800)).await;

    let consume_after_expiry: Result<AuthenticatedReservationConsumption, _> = conductor
        .call_fallible(
            &issuer,
            "consume_finance_reservation",
            consume_input(&expiring, "attempt:after-expiry"),
        )
        .await;
    assert!(
        consume_after_expiry.is_err(),
        "consumption at/after expiry must fail closed"
    );

    let expired: ReservationTransitionReceipt = conductor
        .call(
            &issuer,
            "expire_finance_reservation",
            expire_input(&expiring),
        )
        .await;
    assert_eq!(expired.resulting_sequence, 2);
    assert!(matches!(
        expired.resulting_lifecycle,
        ReservationLifecycleWire::Expired { .. }
    ));
}
