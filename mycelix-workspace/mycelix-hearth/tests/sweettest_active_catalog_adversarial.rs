// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Adversarial qualification for the canonical Active Hearth catalog.
//!
//! The important case here is two independently admitted memberships for the
//! same agent/Hearth pair. The catalog must not silently select one authority.

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

/// Two independently accepted invitations create two Active memberships for
/// one agent/Hearth pair. The canonical catalog must fail closed rather than
/// selecting one of the conflicting authority records.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_active_catalog_fails_closed_on_conflicting_active_memberships() {
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

    let _: Record = bob_conductor
        .call(
            &bob.zome("hearth_kinship"),
            "accept_invitation",
            AcceptInvitationInput {
                invitation_hash: second.action_address().clone(),
                display_name: "Bob Two".into(),
            },
        )
        .await;

    tokio::time::sleep(std::time::Duration::from_secs(10)).await;

    // SweetConductor::call unwraps a zome error and panics. Isolate only the
    // catalog call in a task so an earlier setup/acceptance failure cannot
    // satisfy the test accidentally.
    let join = tokio::spawn(async move {
        let _: serde_json::Value = bob_conductor
            .call(&bob.zome("hearth_kinship"), "get_my_active_hearths", ())
            .await;
    });

    assert!(
        join.await.is_err(),
        "canonical Active Hearth catalog must fail closed on conflicting active memberships"
    );
}
