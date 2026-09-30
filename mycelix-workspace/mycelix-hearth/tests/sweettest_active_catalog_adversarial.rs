// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Adversarial qualification for the canonical Active Hearth catalog.
//!
//! The important admission invariant is that one agent/Hearth pair cannot
//! acquire two Active memberships through the public invitation flow. The
//! catalog then proves the admitted state remains cardinality-one.

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

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
}

/// Two invitations may exist, but only one can be consumed into an Active
/// membership for the same agent/Hearth pair. The catalog must expose exactly
/// that one authoritative membership rather than inheriting invitation count.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_duplicate_active_membership_admission_is_rejected_and_catalog_remains_unique() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("test-app", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("test-app", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Conflicting Catalog Test".into(),
                description: "Two active memberships must fail closed".into(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(4),
            },
        )
        .await;
    let hearth_hash = hearth.action_address().clone();
    let bob_agent = bob.agent_pubkey().clone();

    let invite = |message: &str| InviteMemberInput {
        hearth_hash: hearth_hash.clone(),
        invitee_agent: bob_agent.clone(),
        proposed_role: MemberRole::Adult,
        message: message.into(),
        expires_at: Timestamp::from_micros(
            Timestamp::now().as_micros() + 86_400_000_000,
        ),
    };

    let first: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            invite("First independent admission"),
        )
        .await;

    let second: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            invite("Second independent admission"),
        )
        .await;

    tokio::time::sleep(std::time::Duration::from_secs(10)).await;

    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: first.action_address().clone(),
                display_name: "Bob One".into(),
            },
        )
        .await;

    tokio::time::sleep(std::time::Duration::from_secs(10)).await;

    // The second invitation remains valid, but the membership guard must
    // reject consuming it while Bob is already Active in this Hearth.
    let second_invitation = second.action_address().clone();
    let join = tokio::spawn(async move {
        let _: Record = bob_conductor
            .call(
                &bob.zome("hearth_kinship"),
                "accept_invitation",
                AcceptInvitationInput {
                    invitation_hash: second_invitation,
                    display_name: "Bob Two".into(),
                },
            )
            .await;
    });

    assert!(
        join.await.is_err(),
        "second Active admission must be rejected rather than creating conflicting authority"
    );

    let catalog: Vec<serde_json::Value> = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "get_my_active_hearths",
            (),
        )
        .await;

    assert_eq!(
        catalog.len(),
        1,
        "one admitted membership must yield exactly one canonical Active Hearth"
    );
}

/// Membership authority is append-only: a member must depart via the
/// constrained Active -> Departed update rather than deleting the evidence
/// leaf that the canonical catalog consumes.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_active_membership_delete_is_rejected() {
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();

    let mut alice_conductor = SweetConductor::from_standard_config().await;
    let mut bob_conductor = SweetConductor::from_standard_config().await;

    let (alice,) = alice_conductor
        .setup_app("test-app", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = bob_conductor
        .setup_app("test-app", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();

    SweetConductor::exchange_peer_info([&alice_conductor, &bob_conductor]).await;

    let hearth: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Delete Protection Test".into(),
                description: "Membership evidence must not be deletable".into(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(4),
            },
        )
        .await;
    let hearth_hash = hearth.action_address().clone();
    let bob_agent = bob.agent_pubkey().clone();

    let invitation: Record = alice_conductor
        .call(
            &alice.zome("hearth_kinship"),
            "invite_member",
            InviteMemberInput {
                hearth_hash,
                invitee_agent: bob_agent,
                proposed_role: MemberRole::Adult,
                message: "Delete protection".into(),
                expires_at: Timestamp::from_micros(
                    Timestamp::now().as_micros() + 86_400_000_000,
                ),
            },
        )
        .await;

    tokio::time::sleep(std::time::Duration::from_secs(10)).await;

    let membership: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: invitation.action_address().clone(),
                display_name: "Bob".into(),
            },
        )
        .await;
    let membership_hash = membership.action_address().clone();

    let join = tokio::spawn(async move {
        let _: Record = bob_conductor
            .call(
                &bob.zome("hearth_kinship"),
                "delete_entry",
                membership_hash,
            )
            .await;
    });

    assert!(
        join.await.is_err(),
        "deleting Active membership evidence must be rejected; departure must use the lifecycle update"
    );
}
