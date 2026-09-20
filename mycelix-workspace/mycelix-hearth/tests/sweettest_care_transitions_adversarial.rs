// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Direct adversarial qualification of immutable Care transition integrity.
//!
//! This test runs only against the generated adversarial Hearth DNA containing
//! qualification-only fake-entry and raw-publish zomes. Those zomes are never
//! part of the production Hearth DNA/hApp.

use holochain::prelude::*;
use holochain::sweettest::*;
use std::path::PathBuf;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum HearthType {
    Nuclear,
    Extended,
    Chosen,
    Blended,
    Multigenerational,
    Intentional,
    CoPod,
    Custom(String),
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum MemberRole {
    Founder,
    Elder,
    Adult,
    Youth,
    Child,
    Guest,
    Ancestor,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum MembershipStatus {
    Active,
    Invited,
    Departed,
    Ancestral,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum MembershipAdmission {
    Founder,
    Invitation {
        invitation_hash: ActionHash,
        response_hash: ActionHash,
    },
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct HearthMembership {
    hearth_hash: ActionHash,
    agent: AgentPubKey,
    role: MemberRole,
    status: MembershipStatus,
    display_name: String,
    joined_at: Timestamp,
    admission: MembershipAdmission,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateHearthInput {
    name: String,
    description: String,
    hearth_type: HearthType,
    max_members: Option<u32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct InviteMemberInput {
    hearth_hash: ActionHash,
    invitee_agent: AgentPubKey,
    proposed_role: MemberRole,
    message: String,
    expires_at: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct AcceptInvitationInput {
    invitation_hash: ActionHash,
    display_name: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum CareType {
    Childcare,
    Eldercare,
    PetCare,
    Chore,
    MealPrep,
    Medical,
    Emotional,
    Custom(String),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum Recurrence {
    Daily,
    Weekly,
    Monthly,
    Custom(String),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum CareScheduleStatus {
    Active,
    Paused,
    Completed,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CareSchedule {
    hearth_hash: ActionHash,
    care_type: CareType,
    title: String,
    description: String,
    assigned_to: AgentPubKey,
    recurrence: Recurrence,
    notes: String,
    status: CareScheduleStatus,
    completed_at: Option<Timestamp>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CreateCareScheduleInput {
    hearth_hash: ActionHash,
    care_type: CareType,
    title: String,
    description: String,
    assigned_to: AgentPubKey,
    recurrence: Recurrence,
    notes: String,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
struct CareCompletion {
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    assignee: AgentPubKey,
    actor: AgentPubKey,
    actor_membership_hash: ActionHash,
}

fn adversarial_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna-adversarial");
    path.push("mycelix_hearth.dna");
    path
}

fn schedule_input(
    hearth_hash: ActionHash,
    assigned_to: AgentPubKey,
    title: &str,
) -> CreateCareScheduleInput {
    CreateCareScheduleInput {
        hearth_hash,
        care_type: CareType::Chore,
        title: title.to_string(),
        description: "Direct transition-integrity adversarial subject".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

async fn find_membership(
    conductor: &SweetConductor,
    cell: &SweetCell,
    hearth_hash: ActionHash,
    agent: &AgentPubKey,
) -> (Record, HearthMembership) {
    let records: Vec<Record> = conductor
        .call(
            &cell.zome("hearth_kinship"),
            "get_hearth_members",
            hearth_hash,
        )
        .await;

    records
        .into_iter()
        .find_map(|record| {
            let membership = record
                .entry()
                .to_app_option::<HearthMembership>()
                .expect("membership must decode")?;
            (membership.agent == *agent).then_some((record, membership))
        })
        .expect("expected Hearth membership record")
}

fn raw_completion(
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    assignee: AgentPubKey,
    actor: AgentPubKey,
    actor_membership_hash: ActionHash,
) -> CareCompletion {
    CareCompletion {
        hearth_hash,
        schedule_hash,
        assignee,
        actor,
        actor_membership_hash,
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated adversarial Hearth DNA and Holochain conductor"]
async fn forged_transition_evidence_fails_closed() {
    let dna_file = SweetDnaFile::from_bundle(&adversarial_dna_path())
        .await
        .unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-transition-adversarial-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-transition-adversarial-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth_a: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Adversarial Hearth A".to_string(),
                description: "Direct transition integrity qualification".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    let hearth_a_hash = hearth_a.action_address().clone();

    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash: hearth_a_hash.clone(),
                invitee_agent: bob.agent_pubkey().clone(),
                proposed_role: MemberRole::Adult,
                message: "Join adversarial qualification".to_string(),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let bob_membership_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: invitation.action_address().clone(),
                display_name: "Bob".to_string(),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let (alice_membership_record, _) = find_membership(
        &alice_conductor,
        &alice,
        hearth_a_hash.clone(),
        alice.agent_pubkey(),
    )
    .await;

    let schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_a_hash.clone(),
                bob.agent_pubkey().clone(),
                "Adversarial completion subject",
            ),
        )
        .await;
    let schedule_hash = schedule.action_address().clone();

    await_consistency([&alice, &bob]).await.unwrap();

    // Positive canary: the qualification-only raw publisher can commit evidence
    // when the payload itself satisfies the real transition integrity theorem.
    let valid_raw: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    let valid_entry = valid_raw
        .entry()
        .to_app_option::<CareCompletion>()
        .expect("raw CareCompletion must decode")
        .expect("raw CareCompletion must contain its entry");
    assert_eq!(valid_entry.actor, bob.agent_pubkey().clone());
    assert_eq!(valid_entry.assignee, bob.agent_pubkey().clone());
    assert_eq!(
        valid_entry.actor_membership_hash,
        bob_membership_record.action_address().clone()
    );

    // Cross-zome type confusion: structurally compatible membership bytes from
    // another integrity zome must not become Kinship authority.
    let fake_membership_hash: ActionHash = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_fake_membership",
            HearthMembership {
                hearth_hash: hearth_a_hash.clone(),
                agent: bob.agent_pubkey().clone(),
                role: MemberRole::Adult,
                status: MembershipStatus::Active,
                display_name: "Fake Bob".to_string(),
                joined_at: Timestamp::now(),
                admission: MembershipAdmission::Founder,
            },
        )
        .await;

    let fake_membership_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                fake_membership_hash,
            ),
        )
        .await;
    assert!(
        fake_membership_attack.is_err(),
        "Serde-compatible membership from another integrity zome must be rejected"
    );

    // Cross-zome type confusion on the referenced schedule must likewise fail.
    let fake_schedule_hash: ActionHash = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_fake_schedule",
            CareSchedule {
                hearth_hash: hearth_a_hash.clone(),
                care_type: CareType::Chore,
                title: "Fake schedule".to_string(),
                description: "Structurally compatible but wrong zome".to_string(),
                assigned_to: bob.agent_pubkey().clone(),
                recurrence: Recurrence::Weekly,
                notes: String::new(),
                status: CareScheduleStatus::Active,
                completed_at: None,
            },
        )
        .await;

    let fake_schedule_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                fake_schedule_hash,
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        fake_schedule_attack.is_err(),
        "Serde-compatible schedule from another integrity zome must be rejected"
    );

    // A real Kinship membership belonging to Alice cannot authorize Bob.
    let wrong_actor_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                alice_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        wrong_actor_attack.is_err(),
        "another actor's canonical Kinship membership must not authorize Bob"
    );

    // Missing/unresolved membership evidence must fail closed rather than being
    // normalized into authority or a successful raw transition.
    let unresolved_membership_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                ActionHash::from_raw_36(vec![0x77; 36]),
            ),
        )
        .await;
    assert!(
        unresolved_membership_attack.is_err(),
        "unresolved membership evidence must fail closed"
    );

    // The signed Create author is Bob because Bob calls the harness. A payload
    // that merely claims Alice as `actor` must therefore be rejected before any
    // claimed membership can strengthen the transition.
    let actor_field_mismatch = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                alice.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        actor_field_mismatch.is_err(),
        "CareCompletion.actor must equal the signed Create action author"
    );

    // The referenced canonical schedule is assigned to Bob. Copying Alice into
    // the completion's assignee field must not rewrite that immutable binding.
    let assignee_field_mismatch = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                alice.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        assignee_field_mismatch.is_err(),
        "CareCompletion.assignee must equal the referenced CareSchedule assignee"
    );

    // An unresolved schedule reference must fail closed at the real integrity
    // boundary; malformed reference state is never equivalent to a valid task.
    let unresolved_schedule_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                ActionHash::from_raw_36(vec![0x66; 36]),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        unresolved_schedule_attack.is_err(),
        "unresolved CareSchedule reference must fail closed"
    );

    // A canonical Bob membership in a different Hearth is real evidence, but it
    // is not authority for Hearth A.
    let hearth_b: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Adversarial Hearth B".to_string(),
                description: "Wrong-Hearth membership source".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    let hearth_b_hash = hearth_b.action_address().clone();
    let (bob_hearth_b_membership, _) = find_membership(
        &bob_conductor,
        &bob,
        hearth_b_hash.clone(),
        bob.agent_pubkey(),
    )
    .await;

    let wrong_hearth_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash.clone(),
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_hearth_b_membership.action_address().clone(),
            ),
        )
        .await;
    assert!(
        wrong_hearth_attack.is_err(),
        "canonical membership from another Hearth must not authorize Hearth A"
    );

    // The schedule belongs to Hearth A. Claiming Hearth B in otherwise
    // canonical completion evidence must be rejected independently of the
    // membership-Hearth check above.
    let hearth_field_mismatch = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_b_hash,
                schedule_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        hearth_field_mismatch.is_err(),
        "CareCompletion.hearth_hash must equal the referenced CareSchedule Hearth"
    );

    // Finally prove revocation at the integrity boundary itself: bypassing the
    // safe production coordinator cannot revive the historical Active hash.
    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            bob_membership_record.action_address().clone(),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let stale_membership_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            raw_completion(
                hearth_a_hash,
                schedule_hash,
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_record.action_address().clone(),
            ),
        )
        .await;
    assert!(
        stale_membership_attack.is_err(),
        "historical Active membership must be rejected after a later departure revision"
    );
}
