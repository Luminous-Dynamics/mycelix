// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Direct immutability/link qualification for CareCompletion evidence.
//!
//! Runs only against #2050's generated adversarial DNA. Production Hearth
//! exposes none of the raw mutation/link externs used by this test.

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
struct CreateHearthInput {
    name: String,
    description: String,
    hearth_type: HearthType,
    max_members: Option<u32>,
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

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct RawCompletionUpdateInput {
    original_action_hash: ActionHash,
    completion: CareCompletion,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct RawCompletionLinkInput {
    base: ActionHash,
    target: ActionHash,
    tag: Vec<u8>,
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
        description: "Care transition immutability qualification".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

fn completion(
    hearth_hash: ActionHash,
    schedule_hash: ActionHash,
    actor: AgentPubKey,
    membership_hash: ActionHash,
) -> CareCompletion {
    CareCompletion {
        hearth_hash,
        schedule_hash,
        assignee: actor.clone(),
        actor,
        actor_membership_hash: membership_hash,
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires generated adversarial Hearth DNA and Holochain conductor"]
async fn completion_evidence_and_index_links_are_immutable_and_bound() {
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
                name: "Immutable Care Hearth".to_string(),
                description: "CareCompletion/link integrity qualification".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(8),
            },
        )
        .await;
    let hearth_hash = hearth.action_address().clone();

    let memberships: Vec<Record> = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "get_hearth_members",
            hearth_hash.clone(),
        )
        .await;
    let alice_membership = memberships
        .into_iter()
        .find(|record| record.action().author() == alice.agent_pubkey())
        .expect("founder membership must exist");

    let schedule: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                alice.agent_pubkey().clone(),
                "Canonical immutable subject",
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
                alice.agent_pubkey().clone(),
                "Wrong-base subject",
            ),
        )
        .await;
    let other_schedule_hash = other_schedule.action_address().clone();

    let canonical_completion = completion(
        hearth_hash,
        schedule_hash.clone(),
        alice.agent_pubkey().clone(),
        alice_membership.action_address().clone(),
    );

    // Positive canary: the raw qualification publisher can create a completion
    // that satisfies the real transition integrity theorem.
    let completion_record: Record = alice_conductor
        .call(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_completion_unchecked",
            canonical_completion.clone(),
        )
        .await;
    let completion_hash = completion_record.action_address().clone();

    // Positive canary: the actor may create the exact empty-tag schedule index.
    let valid_link_hash: ActionHash = alice_conductor
        .call(
            &alice.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                base: schedule_hash.clone(),
                target: completion_hash.clone(),
                tag: Vec::new(),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let update_attempt = alice_conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care_transition_test_harness"),
            "update_completion_unchecked",
            RawCompletionUpdateInput {
                original_action_hash: completion_hash.clone(),
                completion: canonical_completion.clone(),
            },
        )
        .await;
    assert!(update_attempt.is_err(), "CareCompletion must be immutable");

    let delete_attempt = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "delete_completion_unchecked",
            completion_hash.clone(),
        )
        .await;
    assert!(delete_attempt.is_err(), "CareCompletion must not be deletable");

    let delete_link_attempt = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "delete_completion_link_unchecked",
            valid_link_hash,
        )
        .await;
    assert!(
        delete_link_attempt.is_err(),
        "ScheduleToCompletions index links must be append-only"
    );

    let wrong_base_attempt = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                base: other_schedule_hash.clone(),
                target: completion_hash.clone(),
                tag: Vec::new(),
            },
        )
        .await;
    assert!(
        wrong_base_attempt.is_err(),
        "completion must not be indexed under a different CareSchedule"
    );

    let tagged_link_attempt = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                base: schedule_hash.clone(),
                target: completion_hash.clone(),
                tag: vec![0x01],
            },
        )
        .await;
    assert!(
        tagged_link_attempt.is_err(),
        "transition index links must reject non-empty tags"
    );

    let wrong_target_attempt = alice_conductor
        .call_fallible::<_, ActionHash>(
            &alice.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                base: schedule_hash.clone(),
                target: other_schedule_hash,
                tag: Vec::new(),
            },
        )
        .await;
    assert!(
        wrong_target_attempt.is_err(),
        "non-CareCompletion link targets must fail closed"
    );

    let wrong_author_attempt = bob_conductor
        .call_fallible::<_, ActionHash>(
            &bob.zome("hearth_care_transition_test_harness"),
            "create_completion_link_unchecked",
            RawCompletionLinkInput {
                base: schedule_hash,
                target: completion_hash,
                tag: Vec::new(),
            },
        )
        .await;
    assert!(
        wrong_author_attempt.is_err(),
        "only the CareCompletion actor may author its schedule index link"
    );
}
