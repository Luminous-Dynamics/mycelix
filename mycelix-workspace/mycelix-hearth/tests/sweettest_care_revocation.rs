// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Red/green Care authority tests for membership revocation.
//!
//! Historical object identity must not survive loss of active Hearth membership
//! as mutation authority.

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
struct CompleteTaskInput {
    schedule_hash: ActionHash,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct ProposeSwapInput {
    hearth_hash: ActionHash,
    original_schedule_hash: ActionHash,
    swap_date: Timestamp,
}

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
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
        description: "Revocation-bound Care authority test".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn departed_member_loses_care_transition_authority() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-revocation-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-revocation-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Revocation Hearth".to_string(),
                description: "Care membership revocation integration test".to_string(),
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
                message: "Join revocation test".to_string(),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

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

    await_consistency([&alice, &bob]).await.unwrap();

    // Positive canary: while Active, Bob can create and complete his own task.
    let active_canary: Record = bob_conductor
        .call(
            &bob.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Active self-assigned canary",
            ),
        )
        .await;
    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_care"),
            "complete_task",
            CompleteTaskInput {
                schedule_hash: active_canary.action_address().clone(),
            },
        )
        .await;

    // This self-authored/self-assigned schedule remains Active across departure.
    let completion_subject: Record = bob_conductor
        .call(
            &bob.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Post-departure completion subject",
            ),
        )
        .await;

    // Alice also freezes a schedule targeting Bob while Bob is still Active.
    // After departure it must not become the basis for a new swap proposal.
    let proposal_subject: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Post-departure proposal subject",
            ),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            membership.action_address().clone(),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    // This is reachable on the unfixed parent because Bob is both the original
    // author and historical assignee, so the integrity author-match rule does
    // not mask the missing current-membership check.
    let completion_after_departure = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care"),
            "complete_task",
            CompleteTaskInput {
                schedule_hash: completion_subject.action_address().clone(),
            },
        )
        .await;
    assert!(
        completion_after_departure.is_err(),
        "departed assignee must lose Care completion authority"
    );

    let proposal_to_departed = alice_conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "propose_swap",
            ProposeSwapInput {
                hearth_hash,
                original_schedule_hash: proposal_subject.action_address().clone(),
                swap_date: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;
    assert!(
        proposal_to_departed.is_err(),
        "a new CareSwap must not target a departed schedule assignee"
    );
}
