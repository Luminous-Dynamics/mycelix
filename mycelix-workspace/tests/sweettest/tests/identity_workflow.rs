// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Identity hApp sweettest integration tests.
//!
//! Tests DID creation, resolution, and multi-agent DHT propagation
//! using the Holochain sweettest framework with real conductors.
//!
//! Prerequisites:
//!   cd mycelix-identity && cargo build --release --target wasm32-unknown-unknown
//!   hc dna pack dna/ -o dna/mycelix_identity_dna.dna
//!
//! Run: cargo test -p mycelix-sweettest -- --ignored identity
//!
//! Updated for Holochain 0.6 sweettest API.

extern crate holochain_serialized_bytes;

mod harness;

use harness::*;
use holochain::prelude::*;
use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use serial_test::serial;

// ---------------------------------------------------------------------------
// Mirror types for deserialization (sweettest can't import WASM crate types)
// ---------------------------------------------------------------------------

#[derive(serde::Serialize, serde::Deserialize, SerializedBytes, Debug, Clone)]
struct VerificationMethod {
    id: String,
    #[serde(rename = "type", alias = "type_")]
    type_: String,
    controller: String,
    #[serde(rename = "publicKeyMultibase", alias = "public_key_multibase")]
    public_key_multibase: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    algorithm: Option<u16>,
}

#[derive(serde::Serialize, serde::Deserialize, SerializedBytes, Debug, Clone)]
struct ServiceEndpoint {
    id: String,
    #[serde(rename = "type", alias = "type_")]
    type_: String,
    #[serde(rename = "serviceEndpoint", alias = "service_endpoint")]
    service_endpoint: String,
}

#[derive(serde::Serialize, serde::Deserialize, SerializedBytes, Debug)]
struct DidDocument {
    id: String,
    controller: AgentPubKey,
    #[serde(rename = "verificationMethod", alias = "verification_method")]
    verification_method: Vec<VerificationMethod>,
    authentication: Vec<String>,
    #[serde(rename = "keyAgreement", alias = "key_agreement", default)]
    key_agreement: Vec<String>,
    service: Vec<ServiceEndpoint>,
    created: Timestamp,
    updated: Timestamp,
    version: u32,
}

/// Test: Single agent creates and retrieves a DID document.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_create_and_resolve_did() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 1).await;

    let alice = &agents[0];

    // Create a DID for Alice
    let did_record: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;

    // The record should have a valid action hash
    let action_hash = did_record.action_hashed().hash.clone();
    assert!(
        !action_hash.as_ref().is_empty(),
        "DID record should have valid hash"
    );

    // Retrieve DID document by agent pub key
    let retrieved: Option<Record> = alice
        .call_zome_fn(
            "did_registry",
            "get_did_document",
            alice.agent_pubkey.clone(),
        )
        .await;

    assert!(retrieved.is_some(), "Should retrieve Alice's DID document");

    // Deserialize and verify content
    let record = retrieved.unwrap();
    let did_doc: DidDocument = record
        .entry()
        .to_app_option()
        .expect("Failed to deserialize DID")
        .expect("DID entry is None");

    assert!(
        !did_doc.verification_method.is_empty(),
        "DID should have at least one verification method"
    );
    assert_eq!(did_doc.version, 1, "Initial DID version should be 1");
    assert!(
        did_doc.id.starts_with("did:mycelix:"),
        "DID id should start with did:mycelix:"
    );
}

/// Test: DID created by one agent is visible to another via DHT.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_did_cross_agent_resolution() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 2).await;

    let alice = &agents[0];
    let bob = &agents[1];

    // Alice creates her DID
    let _: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;

    wait_for_dht_sync().await;

    // Bob resolves Alice's DID by agent key
    let resolved: Option<Record> = bob
        .call_zome_fn(
            "did_registry",
            "get_did_document",
            alice.agent_pubkey.clone(),
        )
        .await;

    assert!(resolved.is_some(), "Bob should resolve Alice's DID via DHT");

    // Deserialize and verify content
    let record = resolved.unwrap();
    let did_doc: DidDocument = record
        .entry()
        .to_app_option()
        .expect("Failed to deserialize DID")
        .expect("DID entry is None");

    assert!(
        did_doc.id.starts_with("did:mycelix:"),
        "Resolved DID id should start with did:mycelix:"
    );
    assert!(
        !did_doc.verification_method.is_empty(),
        "Resolved DID should have at least one verification method"
    );
}

/// Test: DID deactivation is propagated across agents.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_did_deactivation_propagation() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 2).await;

    let alice = &agents[0];
    let bob = &agents[1];

    // Alice creates and then deactivates her DID
    let _: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;

    wait_for_dht_sync().await;

    // Alice deactivates
    let _: Record = alice
        .call_zome_fn(
            "did_registry",
            "deactivate_did",
            "Key compromised".to_string(),
        )
        .await;

    wait_for_dht_sync().await;

    // Bob checks if Alice's DID is active (using did: format)
    let did_string = format!("did:mycelix:{}", alice.agent_pubkey);

    let is_active: bool = bob
        .call_zome_fn("did_registry", "is_did_active", did_string)
        .await;

    assert!(
        !is_active,
        "Alice's deactivated DID should show as inactive to Bob"
    );
}

/// Test: Service endpoint management.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_service_endpoint_crud() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 1).await;

    let alice = &agents[0];

    // Create DID first
    let _: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;

    // Add a service endpoint using W3C camelCase wire format
    let service = ServiceEndpoint {
        id: "mail-endpoint".into(),
        type_: "MycelixMail".into(),
        service_endpoint: "https://mail.mycelix.net/alice".into(),
    };

    let _: Record = alice
        .call_zome_fn("did_registry", "add_service_endpoint", service)
        .await;

    // Retrieve DID and verify service is attached
    let did_record: Option<Record> = alice
        .call_zome_fn(
            "did_registry",
            "get_did_document",
            alice.agent_pubkey.clone(),
        )
        .await;

    assert!(
        did_record.is_some(),
        "DID document should exist with service endpoint"
    );

    // Deserialize and verify service endpoint content
    let record = did_record.unwrap();
    let did_doc: DidDocument = record
        .entry()
        .to_app_option()
        .expect("Failed to deserialize DID")
        .expect("DID entry is None");

    let mail_svc = did_doc.service.iter().find(|s| s.id == "mail-endpoint");
    assert!(mail_svc.is_some(), "Should find mail-endpoint service");
    let svc = mail_svc.unwrap();
    assert_eq!(
        svc.type_, "MycelixMail",
        "Service type should be MycelixMail"
    );
    assert_eq!(
        svc.service_endpoint, "https://mail.mycelix.net/alice",
        "Service endpoint URL should match"
    );
}

// ---------------------------------------------------------------------------
// Verification for crates/mycelix-leptos-core/src/did_registry.rs (identity
// roadmap step 4, PR #8) — that module was shipped with two load-bearing
// assumptions marked as unverified against a real conductor. These tests
// close that gap.
// ---------------------------------------------------------------------------

/// `did_registry::ensure_did_anchored()` treats a `create_did` failure as
/// success specifically when the error message contains "already has a DID
/// document" — this verifies that string is actually what the zome
/// produces on a second `create_did` call, rather than assuming it from
/// reading the zome source.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_create_did_twice_is_the_idempotency_error_the_client_expects() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 1).await;
    let alice = &agents[0];

    let _: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;

    let second: Result<Record, _> = alice
        .call_zome_fn_fallible("did_registry", "create_did", ())
        .await;

    let err = second.expect_err("second create_did must fail, not silently succeed");
    let msg = format!("{err:?}");
    assert!(
        msg.contains("already has a DID document"),
        "error message must contain the exact substring did_registry.rs's \
         ensure_did_anchored() matches on for idempotency — got: {msg}"
    );
}

/// `did_registry::ensure_browser_key_anchored()` builds its
/// `VerificationMethod.public_key_multibase` via
/// `mycelix_crypto::TaggedPublicKey::new(AlgorithmId::Ed25519, ..).to_multibase()`
/// and submits it through `add_verification_method`. This verifies that
/// exact encoding round-trips through the real zome's validation
/// (`validate_multibase_key`) and actually lands in the stored document —
/// not a client-side assumption about what the zome accepts.
#[tokio::test(flavor = "multi_thread")]
#[serial]
#[ignore] // Requires DNA bundle
async fn test_add_verification_method_accepts_the_clients_multibase_encoding() {
    let agents = setup_test_agents(&DnaPaths::identity(), "mycelix-identity", 1).await;
    let alice = &agents[0];

    let did_record: Record = alice.call_zome_fn("did_registry", "create_did", ()).await;
    let did_doc: DidDocument = did_record
        .entry()
        .to_app_option()
        .expect("Failed to deserialize DID")
        .expect("DID entry is None");
    let did_id = did_doc.id.clone();

    // Simulate a browser-generated Ed25519 public key the same way
    // did_registry.rs does: raw 32 bytes, tagged, multibase-encoded.
    let browser_pubkey_bytes = [42u8; 32];
    let public_key_multibase =
        TaggedPublicKey::new(AlgorithmId::Ed25519, browser_pubkey_bytes.to_vec())
            .expect("32 bytes is a valid Ed25519 key length")
            .to_multibase();

    let method = VerificationMethod {
        id: format!("{did_id}#browser-key-1"),
        type_: AlgorithmId::Ed25519
            .did_verification_method_type()
            .to_string(),
        controller: did_id.clone(),
        public_key_multibase: public_key_multibase.clone(),
        algorithm: Some(AlgorithmId::Ed25519.as_u16()),
    };

    let _: Record = alice
        .call_zome_fn("did_registry", "add_verification_method", method)
        .await;

    let updated: Option<Record> = alice
        .call_zome_fn(
            "did_registry",
            "get_did_document",
            alice.agent_pubkey.clone(),
        )
        .await;
    let updated_doc: DidDocument = updated
        .expect("DID document should still exist")
        .entry()
        .to_app_option()
        .expect("Failed to deserialize updated DID")
        .expect("DID entry is None");

    let browser_method = updated_doc
        .verification_method
        .iter()
        .find(|m| m.id == format!("{did_id}#browser-key-1"));
    assert!(
        browser_method.is_some(),
        "browser key verification method must be present after add_verification_method"
    );
    assert_eq!(
        browser_method.unwrap().public_key_multibase,
        public_key_multibase,
        "stored multibase key must round-trip unchanged"
    );

    // The original Holochain-agent-anchored method (#keys-1) must still be
    // present and untouched — add_verification_method must not deprecate
    // or remove it (that's rotate_key's job, deliberately not used here).
    assert!(
        updated_doc
            .verification_method
            .iter()
            .any(|m| m.id == format!("{did_id}#keys-1")),
        "original anchor key must remain untouched by add_verification_method"
    );
}
