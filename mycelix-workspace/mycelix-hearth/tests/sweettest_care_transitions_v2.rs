// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Immutable Care transition qualification through a real SweetConductor.
//!
//! These tests exercise the public v2 coordinator path. Integrity remains the
//! authority boundary; browser/client filtering is not treated as permission.

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
struct CompleteTaskV2Input {
    schedule_hash: ActionHash,
}

#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
struct CareCompletion {
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    assignee: AgentPubKey,
    actor: AgentPubKey,
    actor_membership_hash: ActionHash,
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
        description: "Immutable Care transition qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

fn decode_completion(record: &Record) -> CareCompletion {
    record
        .entry()
        .to_app_option::<CareCompletion>()
        .expect("CareCompletion entry must decode")
        .expect("CareCompletion record must contain its entry")
}

fn assert_deterministic_order(records: &[Record]) {
    for pair in records.windows(2) {
        let left = &pair[0];
        let right = &pair[1];
        let ordered = left.action().timestamp() < right.action().timestamp()
            || (left.action().timestamp() == right.action().timestamp()
                && left.action_address() <= right.action_address());
        assert!(ordered, "completion evidence must use deterministic action-time/hash order");
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn immutable_completion_preserves_authority_and_provenance() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-transitions-v2-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-transitions-v2-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Immutable Care Hearth".to_string(),
                description: "Care transition authority test".to_string(),
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
                message: "Join immutable Care qualification".to_string(),
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

    // Self-completion: Bob is both actor and assignee, and the coordinator must
    // derive rather than accept his canonical membership proof hash.
    let self_schedule: Record = bob_conductor
        .call(
            &bob.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Bob self-completion",
            ),
        )
        .await;
    let self_schedule_hash = self_schedule.action_address().clone();

    let first_completion_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: self_schedule_hash.clone(),
            },
        )
        .await;
    let first_completion = decode_completion(&first_completion_record);

    assert_eq!(first_completion.hearth_hash, hearth_hash);
    assert_eq!(first_completion.schedule_hash, self_schedule_hash);
    assert_eq!(first_completion.actor, bob.agent_pubkey().clone());
    assert_eq!(first_completion.assignee, bob.agent_pubkey().clone());
    assert_eq!(
        first_completion.actor_membership_hash,
        bob_membership.action_address().clone(),
        "self-completion must bind Bob's canonical Active membership revision"
    );
    assert_eq!(
        first_completion_record.action().author(),
        &first_completion.actor,
        "signed completion author must equal CareCompletion.actor"
    );

    // Duplicate valid completion is additional immutable provenance, not a
    // last-write-wins mutation of the first completion.
    let second_completion_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: self_schedule_hash.clone(),
            },
        )
        .await;
    assert_ne!(
        first_completion_record.action_address(),
        second_completion_record.action_address(),
        "duplicate completion must append a distinct immutable action"
    );

    let self_evidence: Vec<Record> = bob_conductor
        .call(
            &bob.zome("hearth_care_transitions"),
            "get_schedule_completion_evidence",
            self_schedule_hash.clone(),
        )
        .await;
    assert_eq!(self_evidence.len(), 2);
    assert_deterministic_order(&self_evidence);
    for record in &self_evidence {
        let evidence = decode_completion(record);
        assert_eq!(evidence.schedule_hash, self_schedule_hash);
        assert_eq!(evidence.actor, bob.agent_pubkey().clone());
        assert_eq!(evidence.assignee, bob.agent_pubkey().clone());
    }

    // Guardian-on-behalf completion must preserve distinct actor and assignee
    // provenance rather than rewriting the task as Alice's assignment.
    let guardian_schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Guardian on behalf",
            ),
        )
        .await;
    let guardian_schedule_hash = guardian_schedule.action_address().clone();

    let guardian_completion_record: Record = alice_conductor
        .call(
            &alice.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: guardian_schedule_hash.clone(),
            },
        )
        .await;
    let guardian_completion = decode_completion(&guardian_completion_record);
    assert_eq!(guardian_completion.hearth_hash, hearth_hash);
    assert_eq!(guardian_completion.schedule_hash, guardian_schedule_hash);
    assert_eq!(guardian_completion.actor, alice.agent_pubkey().clone());
    assert_eq!(guardian_completion.assignee, bob.agent_pubkey().clone());
    assert_ne!(guardian_completion.actor, guardian_completion.assignee);

    // Freeze a subject before Bob departs. Historical assignment identity must
    // not become permanent transition authority after membership revocation.
    let departure_subject: Record = bob_conductor
        .call(
            &bob.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Post-departure v2 subject",
            ),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            bob_membership.action_address().clone(),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let rejected_after_departure = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transitions"),
            "complete_task_v2",
            CompleteTaskV2Input {
                schedule_hash: departure_subject.action_address().clone(),
            },
        )
        .await;
    assert!(
        rejected_after_departure.is_err(),
        "departed assignee must not retain v2 completion authority"
    );

    // Revocation affects future authority, not historical provenance. The two
    // completion attestations authored while Bob was Active remain readable.
    await_consistency([&alice, &bob]).await.unwrap();
    let evidence_after_departure: Vec<Record> = alice_conductor
        .call(
            &alice.zome("hearth_care_transitions"),
            "get_schedule_completion_evidence",
            self_schedule_hash,
        )
        .await;
    assert_eq!(evidence_after_departure.len(), 2);
    assert_deterministic_order(&evidence_after_departure);
}
