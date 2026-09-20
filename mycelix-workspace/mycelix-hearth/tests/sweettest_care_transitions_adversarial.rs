// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Direct adversarial qualification of immutable Care transition integrity.
//!
//! The generated DNA appends qualification-only fixture zomes. Production
//! Hearth DNA never exposes raw completion publication or fake AppEntryDefs.

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

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
enum MemberRole {
    Founder,
    Elder,
    Adult,
    Youth,
    Child,
    Guest,
    Ancestor,
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
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join(".generated-care-transition-adversarial-dna")
        .join("mycelix_hearth_care_adversarial.dna")
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
        description: "Care transition adversarial qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
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

async fn create_hearth(
    conductor: &SweetConductor,
    cell: &SweetCell,
    name: &str,
) -> ActionHash {
    let record: Record = conductor
        .call(
            &cell.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: name.to_string(),
                description: "Adversarial Care transition qualification".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    record.action_address().clone()
}

async fn invite_and_accept(
    alice_conductor: &SweetConductor,
    alice: &SweetCell,
    bob_conductor: &SweetConductor,
    bob: &SweetCell,
    hearth_hash: ActionHash,
    label: &str,
) -> Record {
    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash,
                invitee_agent: bob.agent_pubkey().clone(),
                proposed_role: MemberRole::Adult,
                message: format!("Join {label}"),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([alice, bob]).await.unwrap();
    let membership: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: invitation.action_address().clone(),
                display_name: "Bob".to_string(),
            },
        )
        .await;
    await_consistency([alice, bob]).await.unwrap();
    membership
}

async fn membership_record_for(
    conductor: &SweetConductor,
    cell: &SweetCell,
    hearth_hash: ActionHash,
    agent: &AgentPubKey,
) -> Record {
    let records: Vec<Record> = conductor
        .call(
            &cell.zome("hearth_kinship"),
            "get_hearth_members",
            hearth_hash,
        )
        .await;
    records
        .into_iter()
        .find(|record| record.action().author() == agent)
        .expect("expected canonical membership record for agent")
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated adversarial Hearth DNA and Holochain conductor"]
async fn transition_integrity_rejects_forged_or_stale_authority_evidence() {
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

    let hearth_a = create_hearth(&alice_conductor, &alice, "Adversarial Hearth A").await;
    let bob_membership_a = invite_and_accept(
        &alice_conductor,
        &alice,
        &bob_conductor,
        &bob,
        hearth_a.clone(),
        "Hearth A",
    )
    .await;
    let alice_membership_a = membership_record_for(
        &alice_conductor,
        &alice,
        hearth_a.clone(),
        alice.agent_pubkey(),
    )
    .await;

    let schedule_a: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_a.clone(),
                bob.agent_pubkey().clone(),
                "Adversarial completion subject",
            ),
        )
        .await;
    let schedule_a_hash = schedule_a.action_address().clone();
    await_consistency([&alice, &bob]).await.unwrap();

    // Positive canary: direct raw publication succeeds only with canonical,
    // current, actor-bound, same-Hearth evidence.
    let positive: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a.clone(),
                schedule_a_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_a.action_address().clone(),
            ),
        )
        .await;
    assert_eq!(positive.action().author(), bob.agent_pubkey());

    // Attack 1: byte-compatible HearthMembership under a different AppEntryDef
    // is not canonical Kinship membership authority.
    let fake_membership: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_adversarial_fake"),
            "clone_membership_as_fake",
            bob_membership_a.action_address().clone(),
        )
        .await;
    let fake_membership_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a.clone(),
                schedule_a_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                fake_membership.action_address().clone(),
            ),
        )
        .await;
    assert!(
        fake_membership_attack.is_err(),
        "membership lookalike from another zome must be rejected"
    );

    // Attack 2: byte-compatible CareSchedule under a different AppEntryDef is
    // not a canonical legacy CareSchedule reference.
    let fake_schedule: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_adversarial_fake"),
            "clone_schedule_as_fake",
            schedule_a_hash.clone(),
        )
        .await;
    let fake_schedule_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a.clone(),
                fake_schedule.action_address().clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_a.action_address().clone(),
            ),
        )
        .await;
    assert!(
        fake_schedule_attack.is_err(),
        "schedule lookalike from another zome must be rejected"
    );

    // Attack 3: another actor's canonical membership cannot authorize Bob.
    let wrong_actor_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a.clone(),
                schedule_a_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                alice_membership_a.action_address().clone(),
            ),
        )
        .await;
    assert!(wrong_actor_attack.is_err());

    // Attack 4: Bob's valid membership from Hearth B cannot authorize a
    // completion bound to Hearth A.
    let hearth_b = create_hearth(&alice_conductor, &alice, "Adversarial Hearth B").await;
    let bob_membership_b = invite_and_accept(
        &alice_conductor,
        &alice,
        &bob_conductor,
        &bob,
        hearth_b,
        "Hearth B",
    )
    .await;
    let wrong_hearth_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a.clone(),
                schedule_a_hash.clone(),
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_b.action_address().clone(),
            ),
        )
        .await;
    assert!(wrong_hearth_attack.is_err());

    // Attack 5: Bob's historical Active membership becomes stale after a
    // canonical Departed revision appears on his source chain.
    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            bob_membership_a.action_address().clone(),
        )
        .await;
    await_consistency([&alice, &bob]).await.unwrap();

    let stale_membership_attack = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_adversarial_raw_completion"),
            "publish_raw_completion",
            raw_completion(
                hearth_a,
                schedule_a_hash,
                bob.agent_pubkey().clone(),
                bob.agent_pubkey().clone(),
                bob_membership_a.action_address().clone(),
            ),
        )
        .await;
    assert!(
        stale_membership_attack.is_err(),
        "historical Active membership must be rejected after departure"
    );
}
