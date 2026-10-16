// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Holochain 0.6 integration tests for authoring-conductor publication evidence.
//!
//! Run with:
//!   cd mycelix-hearth && nix develop
//!   hc dna pack dna/
//!   cd tests
//!   cargo test --release --test sweettest_publication_evidence -- --ignored --test-threads=1
//!
//! These tests use the actual HDK get_validation_receipts host function. Unit
//! tests over DTO aggregation do not substitute for these conductor-level checks.

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

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
enum PublicationEvidenceAvailability {
    NotAvailableToCaller,
    NoOperationData,
    Observed,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
struct PublicationOperationEvidence {
    operation_type: String,
    receipt_record_count: u32,
    valid_receipt_record_count: u32,
    rejected_receipt_record_count: u32,
    abandoned_receipt_record_count: u32,
    receipts_complete: bool,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
struct ActionPublicationEvidence {
    action_hash: ActionHash,
    checked_at_micros: i64,
    availability: PublicationEvidenceAvailability,
    operation_count: u32,
    receipt_record_count: u32,
    receipt_threshold_complete_operation_count: u32,
    rejected_operation_count: u32,
    all_receipt_thresholds_complete: bool,
    operations: Vec<PublicationOperationEvidence>,
}

fn hearth_dna_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop(); // tests/ -> mycelix-hearth/
    path.push("dna");
    path.push("mycelix_hearth.dna");
    path
}

fn assert_summary_is_self_consistent(report: &ActionPublicationEvidence) {
    assert_eq!(report.operation_count as usize, report.operations.len());
    assert_eq!(
        report.receipt_record_count,
        report
            .operations
            .iter()
            .map(|operation| operation.receipt_record_count)
            .fold(0u32, u32::saturating_add)
    );
    assert_eq!(
        report.receipt_threshold_complete_operation_count,
        report
            .operations
            .iter()
            .filter(|operation| operation.receipts_complete)
            .count() as u32
    );
    assert_eq!(
        report.rejected_operation_count,
        report
            .operations
            .iter()
            .filter(|operation| operation.rejected_receipt_record_count > 0)
            .count() as u32
    );
    assert_eq!(
        report.all_receipt_thresholds_complete,
        !report.operations.is_empty()
            && report
                .operations
                .iter()
                .all(|operation| operation.receipts_complete)
    );

    match report.availability {
        PublicationEvidenceAvailability::NotAvailableToCaller
        | PublicationEvidenceAvailability::NoOperationData => {
            assert!(report.operations.is_empty());
            assert_eq!(report.operation_count, 0);
            assert_eq!(report.receipt_record_count, 0);
            assert!(!report.all_receipt_thresholds_complete);
        }
        PublicationEvidenceAvailability::Observed => {
            assert!(!report.operations.is_empty());
        }
    }

    for operation in &report.operations {
        assert_eq!(
            operation.receipt_record_count,
            operation
                .valid_receipt_record_count
                .saturating_add(operation.rejected_receipt_record_count)
                .saturating_add(operation.abandoned_receipt_record_count)
        );
    }
}

/// A source-chain action authored on this conductor produces a stable,
/// correctly bound summary. It may have zero or partial receipts at query time.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor and packed DNA (nix develop)"]
async fn publication_evidence_summarizes_an_action_on_its_authoring_conductor() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let (alice,) = conductor
        .setup_app("publication-evidence-author", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let created_record: Record = conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Publication Evidence Hearth".to_string(),
                description: "Publication evidence integration fixture".to_string(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(10),
            },
        )
        .await;
    let action_hash = created_record.action_address().clone();
    let report: ActionPublicationEvidence = conductor
        .call(
            &alice.zome("hearth_bridge"),
            "get_action_publication_evidence",
            action_hash.clone(),
        )
        .await;

    assert_eq!(report.action_hash, action_hash);
    assert!(report.checked_at_micros > 0);
    assert_summary_is_self_consistent(&report);
}

/// A different conductor must not return receipts for a remote action merely
/// because the caller knows a valid ActionHash. Empty evidence is not success.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires two Holochain conductors and packed DNA (nix develop)"]
async fn publication_evidence_does_not_claim_receipts_for_another_conductor() {
    let mut author_conductor = SweetConductor::from_standard_config().await;
    let mut observer_conductor = SweetConductor::from_standard_config().await;

    let dna_for_author = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let dna_for_observer = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let (author,) = author_conductor
        .setup_app("publication-evidence-remote-author", &[dna_for_author])
        .await
        .unwrap()
        .into_tuple();
    let (observer,) = observer_conductor
        .setup_app("publication-evidence-observer", &[dna_for_observer])
        .await
        .unwrap()
        .into_tuple();

    let remote_record: Record = author_conductor
        .call(
            &author.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Remote Author Hearth".to_string(),
                description: "Publication evidence remote-author fixture".to_string(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(10),
            },
        )
        .await;
    let remote_action_hash = remote_record.action_address().clone();
    let report: ActionPublicationEvidence = observer_conductor
        .call(
            &observer.zome("hearth_bridge"),
            "get_action_publication_evidence",
            remote_action_hash.clone(),
        )
        .await;

    assert_eq!(report.action_hash, remote_action_hash);
    assert_eq!(
        report.availability,
        PublicationEvidenceAvailability::NotAvailableToCaller
    );
    assert_eq!(report.operation_count, 0);
    assert_eq!(report.receipt_record_count, 0);
    assert!(!report.all_receipt_thresholds_complete);
    assert_summary_is_self_consistent(&report);
}


/// HDK 0.6 exposes the conductor's receipt store across agents on that
/// conductor. Hearth narrows this to the current author to avoid cross-agent
/// receipt metadata disclosure.
#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires Holochain conductor and packed DNA (nix develop)"]
async fn publication_evidence_does_not_expose_another_local_agents_receipts() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna_file = SweetDnaFile::from_bundle(&hearth_dna_path()).await.unwrap();
    let (alice,) = conductor
        .setup_app("publication-evidence-alice", &[dna_file.clone()])
        .await
        .unwrap()
        .into_tuple();
    let (bob,) = conductor
        .setup_app("publication-evidence-bob", &[dna_file])
        .await
        .unwrap()
        .into_tuple();

    let alice_record: Record = conductor
        .call(
            &alice.zome("hearth_kinship"),
            "create_hearth",
            CreateHearthInput {
                name: "Alice Publication Evidence Hearth".to_string(),
                description: "Author-scope isolation fixture".to_string(),
                hearth_type: HearthType::Nuclear,
                max_members: Some(10),
            },
        )
        .await;
    let alice_action_hash = alice_record.action_address().clone();

    let report: ActionPublicationEvidence = conductor
        .call(
            &bob.zome("hearth_bridge"),
            "get_action_publication_evidence",
            alice_action_hash.clone(),
        )
        .await;

    assert_eq!(report.action_hash, alice_action_hash);
    assert_eq!(
        report.availability,
        PublicationEvidenceAvailability::NotAvailableToCaller
    );
    assert_eq!(report.operation_count, 0);
    assert_eq!(report.receipt_record_count, 0);
    assert!(!report.all_receipt_thresholds_complete);
    assert_summary_is_self_consistent(&report);
}
