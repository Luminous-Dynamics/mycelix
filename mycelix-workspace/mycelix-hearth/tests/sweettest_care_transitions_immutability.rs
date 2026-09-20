// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Direct qualification of immutable CareCompletion evidence and append-only
//! ScheduleToCompletions indexes through the qualification-only attack harness.

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

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct UpdateCompletionInput {
    original_action_hash: ActionHash,
    completion: CareCompletion,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct CompletionLinkInput {
    schedule_hash: ActionHash,
    completion_hash: ActionHash,
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
        description: "Immutable transition evidence qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated adversarial Hearth DNA and Holochain conductor"]
async fn completion_evidence_is_immutable_and_index_links_are_append_only() {
    let dna_file = SweetDnaFile::from_bundle(&adversarial_dna_path())
        .await
        .unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-transition-immutability-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-transition-immutability-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Immutable Evidence Hearth".to_string(),
                description: "CareCompletion immutability qualification".to_string(),
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
                message: "Join immutable evidence qualification".to_string(),
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
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Immutable completion subject",
            ),
        )
        .await;
    let schedule_hash = schedule.action_address().clone();

    let other_schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Wrong-base link subject",
            ),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let completion = CareCompletion {
        hearth_hash: hearth_hash.clone(),
        schedule_hash: schedule_hash.clone(),
        assignee: bob.agent_pubkey().clone(),
        actor: bob.agent_pubkey().clone(),
        actor_membership_hash: bob_membership.action_address().clone(),
    };

    // Positive raw canary: valid immutable evidence can be committed through
    // the qualification-only publisher.
    let completion_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            completion.clone(),
        )
        .await;
    let completion_hash = completion_record.action_address().clone();

    // The same bytes cannot be written as an Update action. Immutability is a
    // consensus rule, not merely an application convention.
    let update_attempt = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "update_completion_unchecked",
            UpdateCompletionInput {
                original_action_hash: completion_hash.clone(),
                completion: completion.clone(),
            },
        )
        .await;
    assert!(update_attempt.is_err(), "CareCompletion update must be rejected");

    let delete_attempt = bob_conductor
        .call_fallible::<_, ActionHash>(
            &bob.zome("hearth_care_transition_test_harness"),
            "delete_completion_unchecked",
            completion_hash.clone(),
        )
        .await;
    assert!(delete_attempt.is_err(), "CareCompletion delete must be rejected");

    // A correctly authored index link is allowed. The link is only an index;
    // the target CareCompletion already carries the authority evidence.
    let valid_link_hash: ActionHash = bob_conductor
        .call(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            CompletionLinkInput {
                schedule_hash: schedule_hash.clone(),
                completion_hash: completion_hash.clone(),
            },
        )
        .await;

    // A different canonical CareSchedule cannot be used as the base for this
    // completion because the completion itself names `schedule_hash`.
    let wrong_base_attempt = bob_conductor
        .call_fallible::<_, ActionHash>(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            CompletionLinkInput {
                schedule_hash: other_schedule.action_address().clone(),
                completion_hash: completion_hash.clone(),
            },
        )
        .await;
    assert!(
        wrong_base_attempt.is_err(),
        "ScheduleToCompletions base must match CareCompletion.schedule_hash"
    );

    // Alice cannot index Bob-authored evidence even though she is Hearth
    // founder; link authorship remains bound to the CareCompletion actor.
    let wrong_link_author = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            CompletionLinkInput {
                schedule_hash: schedule_hash.clone(),
                completion_hash: completion_hash.clone(),
            },
        )
        .await;
    assert!(
        wrong_link_author.is_err(),
        "ScheduleToCompletions link author must equal the completion actor"
    );

    let delete_link_attempt = bob_conductor
        .call_fallible::<_, ActionHash>(
            &bob.zome("hearth_care_transition_test_harness"),
            "delete_completion_link_unchecked",
            valid_link_hash,
        )
        .await;
    assert!(
        delete_link_attempt.is_err(),
        "ScheduleToCompletions links are append-only and cannot be deleted"
    );

    // Failed mutation/link attacks must not erase either the accepted evidence
    // or its valid append-only index.
    await_consistency([&alice, &bob]).await.unwrap();
    let evidence: Vec<Record> = alice_conductor
        .call(
            &alice.zome("hearth_care_transitions"),
            "get_schedule_completion_evidence",
            schedule_hash,
        )
        .await;
    assert_eq!(evidence.len(), 1, "valid completion evidence must remain indexed");
    assert_eq!(
        evidence[0].action_address(),
        &completion_hash,
        "readback must return the original immutable completion action"
    );
}
