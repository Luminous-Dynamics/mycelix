// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Direct payload-binding qualification for immutable CareCompletion evidence.
//!
//! Runs only against #2050's generated adversarial DNA. Production Hearth
//! exposes none of the raw publication externs used by this test.

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
        description: "CareCompletion payload-binding qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
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
                description: "CareCompletion payload-binding qualification".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(8),
            },
        )
        .await;
    record.action_address().clone()
}

async fn founder_membership_hash(
    conductor: &SweetConductor,
    cell: &SweetCell,
    hearth_hash: ActionHash,
) -> ActionHash {
    let records: Vec<Record> = conductor
        .call(
            &cell.zome("hearth_kinship"),
            "get_hearth_members",
            hearth_hash,
        )
        .await;

    records
        .into_iter()
        .find(|record| record.action().author() == cell.agent_pubkey())
        .expect("founder membership must exist")
        .action_address()
        .clone()
}

fn completion(
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    assignee: AgentPubKey,
    actor: AgentPubKey,
    membership_hash: ActionHash,
) -> CareCompletion {
    CareCompletion {
        hearth_hash,
        schedule_hash,
        assignee,
        actor,
        actor_membership_hash: membership_hash,
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated adversarial Hearth DNA and Holochain conductor"]
async fn completion_payload_is_bound_to_signed_actor_and_schedule_identity() {
    let dna_file = SweetDnaFile::from_bundle(&adversarial_dna_path())
        .await
        .unwrap();
    let mut conductor = SweetConductor::from_standard_config().await;
    let (alice,) = conductor
        .setup_app("care-transition-payload-binding", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let hearth_a = create_hearth(&conductor, &alice, "Payload Hearth A").await;
    let membership_a = founder_membership_hash(&conductor, &alice, hearth_a.clone()).await;

    let schedule: Record = conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_a.clone(),
                alice.agent_pubkey().clone(),
                "Canonical payload subject",
            ),
        )
        .await;
    let schedule_hash = schedule.action_address().clone();

    // Positive canary: every identity field agrees with the signed action and
    // referenced canonical CareSchedule.
    let valid: Record = conductor
        .call(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            completion(
                hearth_a.clone(),
                schedule_hash.clone(),
                alice.agent_pubkey().clone(),
                alice.agent_pubkey().clone(),
                membership_a.clone(),
            ),
        )
        .await;
    assert_eq!(valid.action().author(), alice.agent_pubkey());

    // Payload actor is a consensus claim, not a caller-selected alias. The
    // signed Create action author must equal CareCompletion.actor.
    let forged_actor = AgentPubKey::from_raw_36(vec![0x41; 36]);
    let actor_mismatch = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            completion(
                hearth_a.clone(),
                schedule_hash.clone(),
                alice.agent_pubkey().clone(),
                forged_actor,
                membership_a.clone(),
            ),
        )
        .await;
    assert!(
        actor_mismatch.is_err(),
        "CareCompletion.actor must equal the signed Create action author"
    );

    // The completion Hearth must come from the referenced schedule, even when
    // the actor holds a perfectly valid membership in some other Hearth.
    let hearth_b = create_hearth(&conductor, &alice, "Payload Hearth B").await;
    let membership_b = founder_membership_hash(&conductor, &alice, hearth_b.clone()).await;
    let hearth_mismatch = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            completion(
                hearth_b,
                schedule_hash.clone(),
                alice.agent_pubkey().clone(),
                alice.agent_pubkey().clone(),
                membership_b,
            ),
        )
        .await;
    assert!(
        hearth_mismatch.is_err(),
        "CareCompletion.hearth_hash must match the referenced CareSchedule"
    );

    // The assignee is schedule identity evidence. A caller cannot rewrite who
    // the task belonged to inside otherwise well-formed immutable evidence.
    let forged_assignee = AgentPubKey::from_raw_36(vec![0x42; 36]);
    let assignee_mismatch = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            completion(
                hearth_a,
                schedule_hash,
                forged_assignee,
                alice.agent_pubkey().clone(),
                membership_a,
            ),
        )
        .await;
    assert!(
        assignee_mismatch.is_err(),
        "CareCompletion.assignee must match the referenced CareSchedule"
    );
}
