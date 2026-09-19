// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Expected-red Kinship membership epoch regressions.
//!
//! These tests use the qualification-only raw coordinator in the generated
//! adversarial DNA. They must later run unchanged as ordinary green tests.

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

fn adversarial_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna-adversarial");
    path.push("mycelix_hearth.dna");
    path
}

async fn setup_joined_member(
    suffix: &str,
) -> (
    SweetConductor,
    SweetConductor,
    SweetCell,
    SweetCell,
    Record,
    HearthMembership,
) {
    let dna_file = SweetDnaFile::from_bundle(&adversarial_dna_path())
        .await
        .unwrap();
    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app(&format!("membership-red-alice-{suffix}"), &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app(&format!("membership-red-bob-{suffix}"), &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: format!("Membership Red {suffix}"),
                description: "Canonical membership replay qualification".to_string(),
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
                hearth_hash,
                invitee_agent: bob.agent_pubkey().clone(),
                proposed_role: MemberRole::Adult,
                message: "Join membership replay qualification".to_string(),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    await_consistency([&alice, &bob]).await.unwrap();

    let membership_record: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: invitation.action_address().clone(),
                display_name: "Bob".to_string(),
            },
        )
        .await;
    let membership = membership_record
        .entry()
        .to_app_option::<HearthMembership>()
        .expect("accepted HearthMembership must decode")
        .expect("accepted HearthMembership must contain its entry");
    assert_eq!(membership.agent, bob.agent_pubkey().clone());
    assert_eq!(membership.status, MembershipStatus::Active);

    await_consistency([&alice, &bob]).await.unwrap();

    (
        alice_conductor,
        bob_conductor,
        alice,
        bob,
        membership_record,
        membership,
    )
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "expected-red until HEARTH-KINSHIP-AUTH-001 is repaired"]
async fn departed_membership_admission_replay_is_rejected() {
    let (alice_conductor, bob_conductor, alice, bob, membership_record, membership) =
        setup_joined_member("replay").await;

    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "leave_hearth",
            membership_record.action_address().clone(),
        )
        .await;
    await_consistency([&alice, &bob]).await.unwrap();

    let replay = bob_conductor
        .call_fallible::<_, Record>(
            &bob.zome("hearth_care_transition_test_harness"),
            "publish_membership_unchecked",
            membership,
        )
        .await;

    assert!(
        replay.is_err(),
        "departed membership admission must not be replayable"
    );

    // Keep both conductors alive until after the assertion so network teardown
    // cannot be confused with the expected integrity result.
    drop(alice_conductor);
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "expected-red until HEARTH-KINSHIP-AUTH-001 is repaired"]
async fn membership_create_author_must_match_member() {
    let (alice_conductor, _bob_conductor, alice, _bob, _membership_record, membership) =
        setup_joined_member("author").await;

    let forged_author = alice_conductor
        .call_fallible::<_, Record>(
            &alice.zome("hearth_care_transition_test_harness"),
            "publish_membership_unchecked",
            membership,
        )
        .await;

    assert!(
        forged_author.is_err(),
        "membership Create author must equal the claimed member"
    );
}
