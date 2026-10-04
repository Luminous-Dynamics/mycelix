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
use holochain::{
    conductor::api::error::ConductorApiError,
    conductor::CellError,
    core::workflow::WorkflowError,
    core::SourceChainError,
};
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

#[derive(Clone, Debug, Serialize, Deserialize)]
struct UpdateStoryInput {
    story_hash: ActionHash,
    title: String,
    content: String,
    tags: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CrossClusterNotificationInput {
    schema_version: u8,
    source_cluster: String,
    source_zome: String,
    event_type: String,
    target_clusters: Vec<String>,
    target_agents: Vec<String>,
    payload: String,
    priority: u8,
    created_at: Timestamp,
    expires_at: Option<Timestamp>,
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

fn expected_reason(test_name: &str) -> String {
    let manifest: serde_json::Value = serde_json::from_str(include_str!(
        "hearth-07-semantic-validation-cases.json"
    ))
    .expect("semantic-validation case manifest must be valid JSON");

    manifest["cases"]
        .as_array()
        .and_then(|cases| {
            cases.iter().find_map(|case| {
                (case["test"].as_str() == Some(test_name))
                    .then(|| case["invariant"].as_str())
                    .flatten()
            })
        })
        .unwrap_or_else(|| panic!("semantic manifest has no invariant for {test_name}"))
        .to_string()
}

fn assert_integrity_rejection<T>(result: Result<T, ConductorApiError>, expected_reason: &str) {
    let err = result.expect_err("invalid input must be rejected by the zome");
    match err {
        ConductorApiError::CellError(CellError::WorkflowError(wfe)) => match *wfe {
            WorkflowError::SourceChainError(SourceChainError::InvalidCommit(reason)) => {
                assert!(
                    reason.contains(expected_reason),
                    "expected integrity rejection reason {expected_reason:?}, got {reason:?}"
                );
            }
            other => panic!("expected SourceChainError::InvalidCommit, got {other:?}"),
        },
        other => panic!("expected ConductorApiError::CellError(WorkflowError(SourceChainError::InvalidCommit)), got {other:?}"),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_decision_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: Result<Record, _> = conductor
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
                deadline: Timestamp::from_micros(Timestamp::now().as_micros() + 3_600_000_000),
                quorum_bp: Some(5000),
            },
        )
        .await;

    assert_integrity_rejection(result, &expected_reason("test_invalid_decision_entry_reaches_integrity_validation"));
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_resource_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: Result<Record, _> = conductor
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

    assert_integrity_rejection(result, &expected_reason("test_invalid_resource_entry_reaches_integrity_validation"));
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_story_entry_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let result: Result<Record, _> = conductor
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

    assert_integrity_rejection(result, &expected_reason("test_invalid_story_entry_reaches_integrity_validation"));
}



#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_story_update_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;
    let hearth_hash = create_test_hearth(&conductor, &alice).await;

    let story: Record = conductor
        .call(
            &alice.zome("hearth_stories"),
            "create_story",
            CreateStoryInput {
                hearth_hash,
                title: "Valid story".into(),
                content: "Valid content before adversarial update.".into(),
                story_type: StoryType::Memory,
                media_hashes: vec![],
                tags: vec![],
                visibility: HearthVisibility::AllMembers,
            },
        )
        .await;

    let result: Result<Record, _> = conductor
        .call_fallible(
            &alice.zome("hearth_stories"),
            "update_story",
            UpdateStoryInput {
                story_hash: story.action_address().clone(),
                title: String::new(),
                content: "The empty title is the boundary under test.".into(),
                tags: vec![],
            },
        )
        .await;

    assert_integrity_rejection(result, &expected_reason("test_invalid_story_update_reaches_integrity_validation"));
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor (nix develop)"]
async fn test_invalid_bridge_notification_reaches_integrity_validation() {
    let (conductor, alice) = setup_alice().await;

    let result: Result<ActionHash, _> = conductor
        .call_fallible(
            &alice.zome("hearth_bridge"),
            "receive_notification",
            CrossClusterNotificationInput {
                schema_version: 1,
                source_cluster: String::new(),
                source_zome: "test".into(),
                event_type: "test_event".into(),
                target_clusters: vec![],
                target_agents: vec![],
                payload: "{}".into(),
                priority: 1,
                created_at: Timestamp::from_micros(1),
                expires_at: None,
            },
        )
        .await;

    assert_integrity_rejection(result, &expected_reason("test_invalid_bridge_notification_reaches_integrity_validation"));
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
        &expected_reason("test_cross_hearth_collection_story_link_reaches_integrity_validation"),
    );
}


#[test]
fn test_semantic_case_manifest_is_structurally_valid() {
    let manifest: serde_json::Value = serde_json::from_str(include_str!(
        "hearth-07-semantic-validation-cases.json"
    ))
    .expect("semantic-validation case manifest must be valid JSON");

    assert_eq!(
        manifest["schema_version"],
        "HEARTH-SEMANTIC-0.7-CASESET-1"
    );
    assert!(
        manifest["claim_ceiling"]
            .as_str()
            .expect("claim ceiling must be a string")
            .contains("RuntimeQualificationPending"),
        "semantic manifest must retain the runtime-pending evidence ceiling"
    );

    let cases = manifest["cases"]
        .as_array()
        .expect("semantic manifest must contain a cases array");
    assert!(!cases.is_empty(), "semantic manifest must contain at least one case");

    let allowed_surfaces = [
        "CreateEntry",
        "CreateRecord",
        "Update",
        "Delete",
        "Link.CreateLink",
        "Link.DeleteLink",
    ];
    let mut seen_ids = std::collections::BTreeSet::new();
    let mut seen_tests = std::collections::BTreeSet::new();

    for case in cases {
        let case_id = case["case_id"]
            .as_str()
            .expect("every semantic case needs a case_id");
        let test = case["test"]
            .as_str()
            .expect("every semantic case needs a test name");
        let zome = case["zome"]
            .as_str()
            .expect("every semantic case needs a zome");
        let operation = case["operation"]
            .as_str()
            .expect("every semantic case needs an operation");
        let invariant = case["invariant"]
            .as_str()
            .expect("every semantic case needs an invariant");
        let invariant_code = case["invariant_code"]
            .as_str()
            .expect("every semantic case needs invariant_code");
        let expected_result = case["expected_result"]
            .as_str()
            .expect("every semantic case needs expected_result");
        let boundary = case["boundary"]
            .as_str()
            .expect("every semantic case needs a boundary");
        let validator_source = case["validator_source"]
            .as_str()
            .expect("every semantic case needs a validator_source");
        let validator_symbol = case["validator_symbol"]
            .as_str()
            .expect("every semantic case needs a validator_symbol");

        assert!(seen_ids.insert(case_id), "duplicate semantic case_id: {case_id}");
        assert!(seen_tests.insert(test), "duplicate semantic test name: {test}");
        assert!(
            case_id.starts_with("SEM-"),
            "semantic case_id must use SEM-* namespace: {case_id}"
        );
        assert!(!zome.is_empty(), "semantic case zome must not be empty");
        assert!(!operation.is_empty(), "semantic case operation must not be empty");
        assert!(!invariant.is_empty(), "semantic case invariant must not be empty");
        assert!(
            !invariant_code.is_empty(),
            "{case_id} invariant_code must not be empty"
        );
        assert!(
            !invariant_code.contains('\n'),
            "{case_id} invariant_code must be a single predicate expression"
        );
        assert_eq!(
            expected_result, "Invalid",
            "{case_id} must declare the Invalid validation result"
        );
        assert_eq!(
            boundary, "integrity_validation",
            "{case_id} must target the integrity validation boundary"
        );
        assert!(!validator_source.is_empty(), "{case_id} validator_source must not be empty");
        assert!(
            validator_source.ends_with("src/lib.rs"),
            "{case_id} validator_source must point to a Rust source file"
        );
        assert!(!validator_symbol.is_empty(), "{case_id} validator_symbol must not be empty");

        let surfaces = case["operation_surface"]
            .as_array()
            .expect("every semantic case needs operation_surface");
        assert!(
            !surfaces.is_empty(),
            "{case_id} operation_surface must not be empty"
        );
        for surface in surfaces {
            let surface = surface
                .as_str()
                .expect("operation_surface values must be strings");
            assert!(
                allowed_surfaces.contains(&surface),
                "{case_id} contains unknown Holochain operation surface {surface:?}"
            );
        }
    }
}

