// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Expected-red qualification for Care completion-index target provenance.
//!
//! On the vulnerable parent, a same-index Serde-compatible fake completion can
//! be accepted as the target of the real ScheduleToCompletions link. The green
//! child must make this exact test pass normally without changing the fixture.

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

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CareCompletion {
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    assignee: AgentPubKey,
    actor: AgentPubKey,
    actor_membership_hash: ActionHash,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct RawCompletionLinkInput {
    schedule_hash: ActionHash,
    completion_hash: ActionHash,
}

fn adversarial_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna-link-provenance-red");
    path.push("mycelix_hearth.dna");
    path
}

fn schedule_input(
    hearth_hash: ActionHash,
    assigned_to: AgentPubKey,
) -> CreateCareScheduleInput {
    CreateCareScheduleInput {
        hearth_hash,
        care_type: CareType::Chore,
        title: "Completion-link provenance subject".to_string(),
        description: "Expected-red completion target provenance test".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated link-provenance adversarial DNA"]
async fn fake_completion_link_target_is_rejected() {
    let dna_file = SweetDnaFile::from_bundle(&adversarial_dna_path())
        .await
        .unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-link-red-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-link-red-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Completion Link Red Hearth".to_string(),
                description: "Expected-red type provenance qualification".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    let hearth_hash = hearth.action_address().clone();

    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash: hearth_hash.clone(),
                invitee_agent: bob.agent_pubkey().clone(),
                proposed_role: MemberRole::Adult,
                message: "Join link provenance test".to_string(),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let bob_membership: Record = bob_conductor
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

    let schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(hearth_hash.clone(), bob.agent_pubkey().clone()),
        )
        .await;
    let schedule_hash = schedule.action_address().clone();

    await_consistency([&alice, &bob]).await.unwrap();

    let fake_completion: ActionHash = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_fake_completion",
            CareCompletion {
                hearth_hash,
                schedule_hash: schedule_hash.clone(),
                assignee: bob.agent_pubkey().clone(),
                actor: bob.agent_pubkey().clone(),
                actor_membership_hash: bob_membership.action_address().clone(),
            },
        )
        .await;

    let result = bob_conductor
        .call_fallible::<_, ActionHash>(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                schedule_hash,
                completion_hash: fake_completion,
            },
        )
        .await;

    assert!(
        result.is_err(),
        "NON_CANONICAL_COMPLETION_LINK_TARGET_MUST_BE_REJECTED"
    );
}
