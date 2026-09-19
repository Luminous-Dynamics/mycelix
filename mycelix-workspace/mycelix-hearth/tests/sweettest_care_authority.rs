// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Care authority/binding regressions exercised through a real SweetConductor.
//!
//! These tests intentionally call the coordinator boundary rather than browser
//! helpers. Client-side filtering is not authority.

use holochain::prelude::*;
use holochain::sweettest::*;
use std::path::PathBuf;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum HearthType {
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
pub enum MemberRole {
    Founder,
    Elder,
    Adult,
    Youth,
    Child,
    Guest,
    Ancestor,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CreateHearthInput {
    pub name: String,
    pub description: String,
    pub hearth_type: HearthType,
    pub max_members: Option<u32>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct InviteMemberInput {
    pub hearth_hash: ActionHash,
    pub invitee_agent: AgentPubKey,
    pub proposed_role: MemberRole,
    pub message: String,
    pub expires_at: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct AcceptInvitationInput {
    pub invitation_hash: ActionHash,
    pub display_name: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum CareType {
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
pub enum Recurrence {
    Daily,
    Weekly,
    Monthly,
    Custom(String),
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CreateCareScheduleInput {
    pub hearth_hash: ActionHash,
    pub care_type: CareType,
    pub title: String,
    pub description: String,
    pub assigned_to: AgentPubKey,
    pub recurrence: Recurrence,
    pub notes: String,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ProposeSwapInput {
    pub hearth_hash: ActionHash,
    pub original_schedule_hash: ActionHash,
    pub swap_date: Timestamp,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PlannedMeal {
    pub day: String,
    pub meal_type: String,
    pub recipe: String,
    pub servings: u32,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CreateMealPlanInput {
    pub hearth_hash: ActionHash,
    pub week_start: Timestamp,
    pub meals: Vec<PlannedMeal>,
    pub shopper: AgentPubKey,
    pub cook: AgentPubKey,
    pub dietary_notes: String,
}

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
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
                description: "Care authority integration test".to_string(),
                hearth_type: HearthType::Chosen,
                max_members: Some(12),
            },
        )
        .await;
    record.action_address().clone()
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
        description: "Authority-bound care work".to_string(),
        assigned_to,
        recurrence: Recurrence::Weekly,
        notes: String::new(),
    }
}

async fn schedule_count(
    conductor: &SweetConductor,
    cell: &SweetCell,
    hearth_hash: ActionHash,
) -> usize {
    let schedules: Vec<Record> = conductor
        .call(
            &cell.zome("hearth_care"),
            "get_hearth_schedule",
            hearth_hash,
        )
        .await;
    schedules.len()
}

async fn meal_plan_count(
    conductor: &SweetConductor,
    cell: &SweetCell,
    hearth_hash: ActionHash,
) -> usize {
    let plans: Vec<Record> = conductor
        .call(
            &cell.zome("hearth_care"),
            "get_hearth_meal_plans",
            hearth_hash,
        )
        .await;
    plans.len()
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn care_targets_require_active_same_hearth_membership() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("care-authority-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("care-authority-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth_hash = create_hearth(&alice_conductor, &alice, "Authority Hearth").await;

    // Positive canary: the founder is an active member and may be assigned.
    let _: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                alice.agent_pubkey().clone(),
                "Founder duty",
            ),
        )
        .await;
    assert_eq!(schedule_count(&alice_conductor, &alice, hearth_hash.clone()).await, 1);

    // A syntactically valid AgentPubKey is not membership evidence.
    let non_member = AgentPubKey::from_raw_36(vec![0x42; 36]);
    let rejected_non_member = alice_conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                non_member,
                "Must not exist",
            ),
        )
        .await;
    assert!(
        rejected_non_member.is_err(),
        "non-member assignee must be rejected by hearth_care"
    );
    assert_eq!(schedule_count(&alice_conductor, &alice, hearth_hash.clone()).await, 1);

    // Establish Bob as an actual Active member through the canonical invite flow.
    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash: hearth_hash.clone(),
                invitee_agent: bob.agent_pubkey().clone(),
                proposed_role: MemberRole::Adult,
                message: "Join authority test".to_string(),
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

    let _: Record = alice_conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Active member duty",
            ),
        )
        .await;
    assert_eq!(schedule_count(&alice_conductor, &alice, hearth_hash.clone()).await, 2);

    // The same identity must stop being assignable after its membership departs.
    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            membership.action_address().clone(),
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let rejected_departed = alice_conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_hash.clone(),
                bob.agent_pubkey().clone(),
                "Departed member duty",
            ),
        )
        .await;
    assert!(
        rejected_departed.is_err(),
        "departed assignee must be rejected by hearth_care"
    );
    assert_eq!(schedule_count(&alice_conductor, &alice, hearth_hash).await, 2);
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn meal_plan_targets_require_active_same_hearth_membership() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut conductor = SweetConductor::from_standard_config().await;
    let (alice,) = conductor
        .setup_app("care-meal-authority", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let hearth_hash = create_hearth(&conductor, &alice, "Meal Authority Hearth").await;
    let outsider = AgentPubKey::from_raw_36(vec![0x51; 36]);

    let rejected_shopper = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "create_meal_plan",
            CreateMealPlanInput {
                hearth_hash: hearth_hash.clone(),
                week_start: Timestamp::now(),
                meals: Vec::new(),
                shopper: outsider.clone(),
                cook: alice.agent_pubkey().clone(),
                dietary_notes: String::new(),
            },
        )
        .await;
    assert!(rejected_shopper.is_err(), "non-member shopper must be rejected");
    assert_eq!(meal_plan_count(&conductor, &alice, hearth_hash.clone()).await, 0);

    let rejected_cook = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "create_meal_plan",
            CreateMealPlanInput {
                hearth_hash: hearth_hash.clone(),
                week_start: Timestamp::now(),
                meals: Vec::new(),
                shopper: alice.agent_pubkey().clone(),
                cook: outsider,
                dietary_notes: String::new(),
            },
        )
        .await;
    assert!(rejected_cook.is_err(), "non-member cook must be rejected");
    assert_eq!(meal_plan_count(&conductor, &alice, hearth_hash).await, 0);
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires packed Hearth DNA and Holochain conductor"]
async fn care_swap_rejects_cross_hearth_schedule_binding() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let mut conductor = SweetConductor::from_standard_config().await;
    let (alice,) = conductor
        .setup_app("care-swap-authority", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let hearth_a = create_hearth(&conductor, &alice, "Hearth A").await;
    let hearth_b = create_hearth(&conductor, &alice, "Hearth B").await;

    let schedule: Record = conductor
        .call(
            &alice.zome("hearth_care"),
            "create_care_schedule",
            schedule_input(
                hearth_b.clone(),
                alice.agent_pubkey().clone(),
                "Hearth B duty",
            ),
        )
        .await;

    let rejected = conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care"),
            "propose_swap",
            ProposeSwapInput {
                hearth_hash: hearth_a,
                original_schedule_hash: schedule.action_address().clone(),
                swap_date: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    assert!(
        rejected.is_err(),
        "a schedule from Hearth B must not be bound into a swap in Hearth A"
    );

    // Source schedule remains present in its actual Hearth; the rejected swap
    // path has no public swap-list query, so the rejection itself is the
    // conductor-level evidence for this binding theorem.
    assert_eq!(schedule_count(&conductor, &alice, hearth_b).await, 1);
}
