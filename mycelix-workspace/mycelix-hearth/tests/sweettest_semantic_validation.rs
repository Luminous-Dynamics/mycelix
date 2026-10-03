// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Holochain 0.7 semantic validation qualification.
//!
//! These tests deliberately exercise normal coordinator write paths with values
//! that violate deterministic integrity invariants. The assertions require the
//! validator's own rejection reason to cross the zome-call boundary; a generic
//! `Err` is not sufficient evidence because coordinator authorization could fail
//! for unrelated reasons.
//!
//! Runtime claim ceiling:
//! - each ignored test is intended to establish observed integrity rejection once
//!   executed in the pinned Holochain 0.7 environment;
//! - source/unit coverage alone does not substitute for these runtime cases.

use holochain::prelude::*;
use holochain::sweettest::*;
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Clone, Debug, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CreateHearthInput {
    name: String,
    description: String,
    hearth_type: HearthType,
    max_members: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum DecisionType {
    Consensus,
    MajorityVote,
    ElderDecision,
    GuardianDecision,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum MemberRole {
    Founder,
    Elder,
    Adult,
    Youth,
    Child,
    Guest,
    Ancestor,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CreateDecisionInput {
    hearth_hash: ActionHash,
    title: String,
    description: String,
    decision_type: DecisionType,
    eligible_roles: Vec<MemberRole>,
    options: Vec<String>,
    deadline: Timestamp,
    quorum_bp: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum ResourceType {
    Tool,
    Vehicle,
    Book,
    Kitchen,
    Electronics,
    Clothing,
    Custom(String),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct RegisterResourceInput {
    hearth_hash: ActionHash,
    name: String,
    description: String,
    resource_type: ResourceType,
    condition: String,
    location: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum StoryType {
    Memory,
    Tradition,
    Recipe,
    Wisdom,
    Origin,
    Migration,
    Custom(String),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
enum HearthVisibility {
    AllMembers,
    AdultsOnly,
    GuardiansOnly,
    Specified(Vec<AgentPubKey>),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CreateStoryInput {
    hearth_hash: ActionHash,
    title: String,
    content: String,
    story_type: StoryType,
    media_hashes: Vec<ActionHash>,
    tags: Vec<String>,
    visibility: HearthVisibility,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CreateCollectionInput {
    hearth_hash: ActionHash,
    name: String,
    description: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct AddToCollectionInput {
    collection_hash: ActionHash,
    story_hash: ActionHash,
}

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop();
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
}

async fn setup_alice() -> (SweetConductor, SweetCell) {
    let mut conductor = SweetConductor::standard().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let (alice,) = conductor
        .setup_app("test-app", &[dna_file])
        .await
        .unwrap()
        .into_tuple();
    (conductor, alice)
}

async fn create_test_hearth(conductor: &SweetConductor, alice: &SweetCell) -> ActionHash {
    let record: Record = conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Semantic Validation Hearth".into(),
                description: "Runtime integrity qualification fixture".into(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(8),
            },
        )
        .await;
    record.action_address().clone()
}

fn assert_integrity_rejection<T, E: std::fmt::Debug>(result: Result<T, E>, expected_reason: &str) {
    let err = result.expect_err("invalid input must be rejected by the zome");
    let debug = format!("{err:?}");
    assert!(
        debug.contains(expected_reason),
        "expected integrity rejection reason {expected_reason:?}, got {debug}"
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_decision_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: ExternResult<Record> = conductor
        .call_fallible(
            &alice.zome("hearth_decisions"),
            "create_decision",
            CreateDecisionInput {
                hearth_hash,
                title: String::new(),
                description: "The empty title is the boundary under test.".into(),
                decision_type: DecisionType::MajorityVote,
                eligible_roles: vec![MemberRole::Founder],
                options: vec!["Yes".into(), "No".into()],
                deadline: Timestamp::now(),
                quorum_bp: Some(5000),
            },
        )
        .await;

    assert_integrity_rejection(result, "Decision title cannot be empty");
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_resource_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: ExternResult<Record> = conductor
        .call_fallible(
            &alice.zome("hearth_resources"),
            "register_resource",
            RegisterResourceInput {
                hearth_hash,
                name: String::new(),
                description: "The empty name is the boundary under test.".into(),
                resource_type: ResourceType::Tool,
                condition: "Good".into(),
                location: "Workshop".into(),
            },
        )
        .await;

    assert_integrity_rejection(result, "Resource name cannot be empty");
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_story_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: ExternResult<Record> = conductor
        .call_fallible(
            &alice.zome("hearth_stories"),
            "create_story",
            CreateStoryInput {
                hearth_hash,
                title: String::new(),
                content: "The empty title is the boundary under test.".into(),
                story_type: StoryType::Memory,
                media_hashes: vec![],
                tags: vec![],
                visibility: HearthVisibility::AllMembers,
            },
        )
        .await;

    assert_integrity_rejection(result, "Story title cannot be empty");
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_cross_hearth_collection_story_link_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_a = create_test_hearth(&conductor, &alice).await;
    let hearth_b = {
        let record: Record = conductor
            .call(
                &alice.zome("hearth_kinship"),
                "create_hearth",
                CreateHearthInput {
                    name: "Semantic Validation Hearth B".into(),
                    description: "Second hearth for link isolation qualification".into(),
                    hearth_type: HearthType::Intentional,
                    max_members: Some(8),
                },
            )
            .await;
        record.action_address().clone()
    };

    let collection: Record = conductor
        .call(
            &alice.zome("hearth_stories"),
            "create_collection",
            CreateCollectionInput {
                hearth_hash: hearth_a,
                name: "Hearth A Collection".into(),
                description: "Collection used for cross-hearth link rejection".into(),
            },
        )
        .await;

    let story: Record = conductor
        .call(
            &alice.zome("hearth_stories"),
            "create_story",
            CreateStoryInput {
                hearth_hash: hearth_b,
                title: "Hearth B Story".into(),
                content: "This story deliberately belongs to another hearth.".into(),
                story_type: StoryType::Memory,
                media_hashes: vec![],
                tags: vec![],
                visibility: HearthVisibility::AllMembers,
            },
        )
        .await;

    let result = conductor
        .call_fallible(
            &alice.zome("hearth_stories"),
            "add_to_collection",
            AddToCollectionInput {
                collection_hash: collection.action_address().clone(),
                story_hash: story.action_address().clone(),
            },
        )
        .await;

    assert_integrity_rejection(
        result,
        "CollectionToStories story belongs to a different hearth",
    );
}
