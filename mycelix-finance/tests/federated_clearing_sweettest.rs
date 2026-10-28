// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Sweettest contract for the federated-clearing evidence zome.
//!
//! These are real-conductor tests. They are ignored by default because they
//! require the built mycelix_finance DNA bundle and a working Holochain toolchain.

use federated_clearing::{FederatedPublicationReceipt};
use federated_clearing_integrity::{
    ClearingDirection, FederatedObligationClaim, FederatedReservationManifest,
    FederatedSettlementEvidence,
};
use holochain::prelude::*;
use holochain::sweettest::*;
use std::path::PathBuf;

fn dna_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("dna")
        .join("mycelix_finance.dna")
}

async fn load_dna() -> DnaFile {
    SweetDnaFile::from_bundle(&dna_path())
        .await
        .expect("build/pack mycelix_finance DNA before running federated-clearing Sweettests")
}

fn manifest(home_agent: AgentPubKey, id: &str) -> FederatedReservationManifest {
    FederatedReservationManifest {
        schema_version: 1,
        manifest_id: id.into(),
        issuer_community_did: "did:mycelix:issuer".into(),
        community_did: "did:mycelix:community-a".into(),
        instrument_id: "UNIT-1".into(),
        profile_revision: 1,
        agreement_id: "agreement-1".into(),
        direction: ClearingDirection::AToB,
        shard_id: "shard-a".into(),
        epoch: 4,
        reserved_units: "1000".into(),
        home_agent,
        authority_evidence_digest_hex: "a".repeat(64),
        created_at: Timestamp::from_micros(100),
    }
}

fn claim(home_agent: AgentPubKey, manifest_action_hash: ActionHash, event_id: &str) -> FederatedObligationClaim {
    FederatedObligationClaim {
        schema_version: 1,
        event_id: event_id.into(),
        request_id: format!("request:{event_id}"),
        principal_did: "did:mycelix:alice".into(),
        community_did: "did:mycelix:community-a".into(),
        instrument_id: "UNIT-1".into(),
        profile_revision: 1,
        agreement_id: "agreement-1".into(),
        direction: ClearingDirection::AToB,
        reservation_manifest_action_hash: manifest_action_hash,
        shard_id: "shard-a".into(),
        epoch: 4,
        home_agent,
        amount_units: "60".into(),
        canonical_work_digest_hex: "b".repeat(64),
        payload_digest_hex: "c".repeat(64),
        verifier_version: "sweettest-verifier-v1".into(),
        verifier_receipt_digest_hex: "d".repeat(64),
        postgres_receipt_digest_hex: "e".repeat(64),
        created_at: Timestamp::from_micros(200),
    }
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "requires the built finance DNA and Holochain conductor"]
async fn test_manifest_claim_and_settlement_are_immutable_and_idempotent() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app1 = conductor.setup_app("federated-clearing-home", &[dna.clone()]).await.unwrap();
    let app2 = conductor.setup_app("federated-clearing-non-home", &[dna]).await.unwrap();
    let cell1 = app1.cells()[0].clone();
    let cell2 = app2.cells()[0].clone();
    let home_agent = app1.agent().clone();

    // First publication and lost-ack style exact retry return the same action.
    let m = manifest(home_agent.clone(), "manifest-sweettest-001");
    let first: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_reservation_manifest", m.clone())
        .await;
    assert!(!first.replayed);
    let replay: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_reservation_manifest", m.clone())
        .await;
    assert!(replay.replayed);
    assert_eq!(first.action_hash, replay.action_hash);

    // Only the designated home agent can publish the manifest.
    let wrong_home: Result<FederatedPublicationReceipt, _> = conductor
        .call_fallible(&cell2.zome("federated_clearing"), "publish_reservation_manifest", m.clone())
        .await;
    assert!(wrong_home.is_err());

    // The claim is immutable, exact-unit, and linked to the manifest.
    let c = claim(home_agent.clone(), first.action_hash.clone(), "outbox-event-001");
    let published: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_obligation_claim", c.clone())
        .await;
    assert!(!published.replayed);
    let claim_replay: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_obligation_claim", c.clone())
        .await;
    assert!(claim_replay.replayed);
    assert_eq!(published.action_hash, claim_replay.action_hash);

    // Another agent cannot author a claim for a reservation owned by the first.
    let wrong_claim_author: Result<FederatedPublicationReceipt, _> = conductor
        .call_fallible(&cell2.zome("federated_clearing"), "publish_obligation_claim", c.clone())
        .await;
    assert!(wrong_claim_author.is_err());

    // Fractional units fail closed at the DHT integrity boundary.
    let mut fractional = claim(home_agent.clone(), first.action_hash.clone(), "outbox-event-fractional");
    fractional.amount_units = "1.5".into();
    let fractional_result: Result<FederatedPublicationReceipt, _> = conductor
        .call_fallible(&cell1.zome("federated_clearing"), "publish_obligation_claim", fractional)
        .await;
    assert!(fractional_result.is_err());

    // Settlement must reference the claim and reproduce exact instrument/direction/units.
    let settlement = FederatedSettlementEvidence {
        schema_version: 1,
        settlement_id: "settlement-sweettest-001".into(),
        event_id: "settlement-event-sweettest-001".into(),
        obligation_claim_action_hash: published.action_hash.clone(),
        community_did: c.community_did.clone(),
        instrument_id: c.instrument_id.clone(),
        direction: c.direction,
        amount_units: c.amount_units.clone(),
        settlement_digest_hex: "f".repeat(64),
        authority_evidence_digest_hex: "1".repeat(64),
        home_agent: home_agent.clone(),
        created_at: Timestamp::from_micros(300),
    };
    let settled: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_settlement_evidence", settlement.clone())
        .await;
    assert!(!settled.replayed);
    let settled_replay: FederatedPublicationReceipt = conductor
        .call(&cell1.zome("federated_clearing"), "publish_settlement_evidence", settlement.clone())
        .await;
    assert!(settled_replay.replayed);
    assert_eq!(settled.action_hash, settled_replay.action_hash);

    let mut invalid_settlement = settlement;
    invalid_settlement.settlement_id = "settlement-sweettest-invalid".into();
    invalid_settlement.event_id = "settlement-event-sweettest-invalid".into();
    invalid_settlement.amount_units = "59".into();
    let invalid_result: Result<FederatedPublicationReceipt, _> = conductor
        .call_fallible(&cell1.zome("federated_clearing"), "publish_settlement_evidence", invalid_settlement)
        .await;
    assert!(invalid_result.is_err());
}
