// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Deterministic DSID lifecycle laboratory.
//!
//! These tests exercise the live Identity DNA through Holochain Sweettest,
//! emit one machine-readable evidence record per scenario, and intentionally
//! keep network state isolated to in-process SweetConductor instances.
//!
//! Run:
//!   cargo test --manifest-path mycelix-identity/tests/Cargo.toml \
//!     --test dsid_lifecycle_lab -- --include-ignored --nocapture --test-threads=1
//!
//! The fixed DNA network seed comes from mycelix-identity/dna/dna.yaml. Each
//! scenario provisions fresh agents/conductors, so evidence is isolated
//! without depending on a public Holochain network.

use holochain::prelude::*;
use holochain::sweettest::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::PathBuf;

const SELF_RECOVERY_DEFAULT_TIME_LOCK: u64 = 7 * 24 * 3600;

const EVIDENCE_SCHEMA: u16 = 1;

#[derive(Debug, Serialize)]
struct Evidence {
    schema_version: u16,
    scenario_id: &'static str,
    scenario_name: &'static str,
    commit_sha: String,
    dna_hash: String,
    agents: BTreeMap<&'static str, String>,
    action_hashes: Vec<String>,
    entry_hashes: Vec<String>,
    expected: String,
    observed: String,
    pass: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
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

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct VerificationMethod {
    id: String,
    #[serde(rename = "type", alias = "type_")]
    type_: String,
    controller: String,
    #[serde(rename = "publicKeyMultibase", alias = "public_key_multibase")]
    public_key_multibase: String,
    #[serde(default)]
    algorithm: Option<u16>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct ServiceEndpoint {
    id: String,
    #[serde(rename = "type", alias = "type_")]
    type_: String,
    #[serde(rename = "serviceEndpoint", alias = "service_endpoint")]
    service_endpoint: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidDocumentView {
    id: String,
    controller: String,
    verification_methods: Vec<DidVerificationMethodView>,
    key_agreements: Vec<String>,
    services: Vec<DidServiceView>,
    created: i64,
    updated: i64,
    version: u32,
    active: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidVerificationMethodView {
    id: String,
    type_name: String,
    controller: String,
    public_key_multibase: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidServiceView {
    id: String,
    type_name: String,
    endpoint: String,
}


#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct MfaFactorView {
    factor_type: MfaFactorType,
    factor_id: String,
    enrolled_at: i64,
    last_verified: i64,
    effective_strength: f32,
    active: bool,
    metadata: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct MfaStateView {
    did: String,
    factors: Vec<MfaFactorView>,
    assurance_level: MfaAssuranceLevel,
    effective_strength: f32,
    category_count: u8,
    updated: i64,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
enum MfaFactorType {
    PrimaryKeyPair,
    HardwareKey,
    Biometric,
    SocialRecovery,
    ReputationAttestation,
    GitcoinPassport,
    VerifiableCredential,
    RecoveryPhrase,
    SecurityQuestions,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
enum MfaAssuranceLevel {
    Anonymous,
    Basic,
    Verified,
    HighlyAssured,
    ConstitutionallyCritical,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
struct CredentialView {
    id: String,
    subject_did: String,
    issuer_did: String,
    credential_type: Vec<String>,
    claims: serde_json::Value,
    issued_at: i64,
    expires_at: Option<i64>,
    valid_from: String,
    valid_until: Option<String>,
    revoked: bool,
    schema_id: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidResolutionMetadataView {
    error: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidDocumentMetadataView {
    created: String,
    updated: String,
    deactivated: bool,
    version_id: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidResolutionView {
    did_document: Option<DidDocumentView>,
    resolution_metadata: DidResolutionMetadataView,
    document_metadata: Option<DidDocumentMetadataView>,
}

fn dna_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("dna")
        .join("mycelix_identity_dna.dna")
}

async fn load_dna() -> DnaFile {
    SweetDnaFile::from_bundle(&dna_path())
        .await
        .expect("identity DNA bundle must exist; run hc dna pack dna/ first")
}

fn commit_sha() -> String {
    std::env::var("GITHUB_SHA").unwrap_or_else(|_| "local-uncommitted".to_string())
}

fn decode_entry<T: serde::de::DeserializeOwned>(record: &Record) -> Option<T> {
    match record.entry().as_option()? {
        Entry::App(bytes) => {
            let sb = SerializedBytes::from(bytes.to_owned());
            rmp_serde::from_slice(sb.bytes()).ok()
        }
        _ => None,
    }
}

fn action_hash(record: &Record) -> String {
    record.action_address().to_string()
}

fn entry_hash(record: &Record) -> Option<String> {
    record
        .entry()
        .as_option()
        .map(|entry| EntryHash::with_data_sync(entry).to_string())
}

fn emit_evidence(
    scenario_id: &'static str,
    scenario_name: &'static str,
    dna: &DnaFile,
    agents: BTreeMap<&'static str, String>,
    records: &[&Record],
    expected: impl Into<String>,
    observed: impl Into<String>,
    pass: bool,
) {
    let evidence = Evidence {
        schema_version: EVIDENCE_SCHEMA,
        scenario_id,
        scenario_name,
        commit_sha: commit_sha(),
        dna_hash: dna.dna_hash().to_string(),
        agents,
        action_hashes: records.iter().map(|r| action_hash(r)).collect(),
        entry_hashes: records.iter().filter_map(|r| entry_hash(r)).collect(),
        expected: expected.into(),
        observed: observed.into(),
        pass,
    };
    println!(
        "DSID_EVIDENCE {}",
        serde_json::to_string(&evidence).expect("evidence must serialize")
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_001_create_and_load_canonical_identity() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-alice", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: DidDocumentView = conductor
        .call(&cell.zome("did_registry"), "create_did_view", ())
        .await;
    let canonical_record: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "get_did_document",
            agent.clone(),
        )
        .await
        .expect("canonical DID record must exist");
    let loaded: Option<DidDocumentView> = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await;

    assert!(created.id.starts_with("did:mycelix:"));
    assert_eq!(created.controller, format!("did:mycelix:{}", agent));
    assert_eq!(created.version, 1);
    assert!(created.active);
    assert_eq!(loaded, Some(created.clone()));

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    let observed = format!(
        "canonical_did={} version={} active={}",
        created.id, created.version, created.active
    );
    emit_evidence(
        "DSID-001",
        "create-and-load-canonical-identity",
        &dna,
        agents,
        &[&canonical_record],
        "One canonical active DID is created from the conductor-owned agent key and can be rendered through the browser-safe view.",
        observed,
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_002_initial_security_state_is_present() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-security", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&did_record).expect("DID entry must decode");

    let mfa_ready: bool = conductor
        .call(&cell.zome("mfa"), "has_mfa_state", did.id.clone())
        .await;
    let recovery_config: Option<Record> = conductor
        .call(&cell.zome("recovery"), "get_self_recovery_config", did.id.clone())
        .await;

    assert!(mfa_ready, "DID creation must initialize MFA");
    assert!(
        recovery_config.is_some(),
        "DID creation must initialize progressive self-recovery"
    );

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-002",
        "initial-security-state",
        &dna,
        agents,
        &[&did_record],
        "DID creation produces MFA state and an active self-recovery configuration.",
        format!("mfa_ready={} recovery_present={}", mfa_ready, recovery_config.is_some()),
        mfa_ready && recovery_config.is_some(),
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_003_duplicate_creation_is_rejected() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-duplicate", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let first: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let duplicate: Result<Record, _> = conductor
        .call_fallible(&cell.zome("did_registry"), "create_did", ())
        .await;

    assert!(duplicate.is_err(), "second DID creation must be rejected");

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-003",
        "duplicate-creation-rejected",
        &dna,
        agents,
        &[&first],
        "An agent can have at most one DID document.",
        format!("duplicate_rejected={}", duplicate.is_err()),
        duplicate.is_err(),
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_004_multi_agent_resolution_after_consistency() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-alice-resolver", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-bob-resolver", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();

    let alice_record: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;
    let bob_record: Record = conductor
        .call(&bob.zome("did_registry"), "create_did", ())
        .await;

    await_consistency(&[alice.clone(), bob.clone()])
        .await
        .expect("fresh peers must reach consistency before cross-agent resolution");

    let alice_doc: DidDocument = decode_entry(&alice_record).expect("Alice DID entry must decode");

    let resolved: Option<Record> = conductor
        .call(&bob.zome("did_registry"), "resolve_did", alice_doc.id.clone())
        .await;
    let resolved = resolved.expect("Bob must resolve Alice after consistency");

    let resolved_doc: DidDocument = decode_entry(&resolved).expect("resolved DID entry must decode");
    assert_eq!(resolved_doc.id, alice_doc.id);
    assert_eq!(resolved_doc.controller, alice_app.agent());

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    emit_evidence(
        "DSID-004",
        "multi-agent-resolution-after-consistency",
        &dna,
        agents,
        &[&alice_record, &bob_record, &resolved],
        "A peer can resolve another peer's canonical DID after DHT consistency.",
        format!("resolved_did={}", resolved_doc.id),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_005_deactivation_is_observable_and_terminal() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-deactivate", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = did_record
        .entry()
        .to_app_option()
        .expect("DID entry must decode")
        .expect("DID entry must be present");

    let active_before: bool = conductor
        .call(&cell.zome("did_registry"), "is_did_active", did.id.clone())
        .await;
    let deactivated: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "deactivate_did",
            "DSID deterministic lifecycle test".to_string(),
        )
        .await;
    let active_after: bool = conductor
        .call(&cell.zome("did_registry"), "is_did_active", did.id.clone())
        .await;
    let view_after: Option<DidDocumentView> = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await;

    assert!(active_before);
    assert!(!active_after);
    assert_eq!(view_after.as_ref().map(|view| view.active), Some(false));

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-005",
        "deactivation-is-observable-and-terminal",
        &dna,
        agents,
        &[&did_record, &deactivated],
        "A DID transitions from active to inactive and remains inactive after deactivation.",
        format!("active_before={} active_after={}", active_before, active_after),
        active_before && !active_after,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_006_self_recovery_projection_matches_canonical_state() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-recovery-view", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&did_record).expect("DID entry must decode");

    let view: Option<serde_json::Value> = conductor
        .call(&cell.zome("recovery"), "get_self_recovery_view", did.id.clone())
        .await;
    let view = view.expect("self-recovery projection must exist after DID creation");

    assert_eq!(view["did"], did.id);
    assert_eq!(view["anchors"].as_array().map(Vec::len), Some(0));
    assert_eq!(view["anchor_threshold"], 1u64);
    assert_eq!(view["time_lock_secs"], SELF_RECOVERY_DEFAULT_TIME_LOCK);
    assert_eq!(view["active"], true);
    assert_eq!(view["superseded_by_social"], false);

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-006",
        "self-recovery-projection-matches-canonical-state",
        &dna,
        agents,
        &[&did_record],
        "A newly-created DID exposes its canonical self-recovery configuration without leaking owner keys or raw Holochain Records.",
        format!(
            "anchors={} threshold={} time_lock_secs={} active={} superseded={}",
            view["anchors"].as_array().map(Vec::len).unwrap_or_default(),
            view["anchor_threshold"],
            view["time_lock_secs"],
            view["active"],
            view["superseded_by_social"]
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_007_browser_projection_redacts_mfa_material() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-mfa-redaction", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&did_record).expect("DID entry must decode");

    let view: Option<MfaStateView> = conductor
        .call(&cell.zome("mfa"), "get_mfa_view", did.id.clone())
        .await;

    let view = view.expect("MFA projection must exist after DID creation");
    assert_eq!(view.did, did.id);
    assert_eq!(view.factors.len(), 1);
    assert!(view.factors[0].factor_id.contains('…'));
    assert_ne!(view.factors[0].factor_id, format!("sha256:{}", agent));
    assert!(view.factors[0].metadata.is_empty());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-007",
        "browser-projection-redacts-mfa-material",
        &dna,
        agents,
        &[&did_record],
        "The browser-facing MFA projection exposes only a masked identifier and no raw factor metadata.",
        format!(
            "factor_count={} masked_id={} metadata_empty={}",
            view.factors.len(),
            view.factors[0].factor_id,
            view.factors[0].metadata.is_empty()
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_008_updated_did_remains_canonical() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-update", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: DidDocumentView = conductor
        .call(&cell.zome("did_registry"), "create_did_view", ())
        .await;

    let update_input = serde_json::json!({
        "verificationMethod": null,
        "authentication": null,
        "keyAgreement": null,
        "service": [{
            "id": format!("{}#service-1", created.id),
            "type": "DiagnosticsService",
            "serviceEndpoint": "https://identity.mycelix.net/diagnostics"
        }]
    });

    let _updated: Record = conductor
        .call(&cell.zome("did_registry"), "update_did_document", update_input)
        .await;

    let loaded: Option<DidDocumentView> = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await;

    let loaded = loaded.expect("updated DID must remain resolvable");
    assert_eq!(loaded.id, created.id);
    assert_eq!(loaded.controller, agent.to_string());
    assert_eq!(loaded.version, 2);
    assert_eq!(loaded.services.len(), 1);
    assert_eq!(loaded.services[0].endpoint, "https://identity.mycelix.net/diagnostics");

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-008",
        "updated-did-remains-canonical",
        &dna,
        agents,
        &[],
        "A legitimate controller update replaces the canonical AgentToDid link and remains the resolved DID state.",
        format!(
            "did={} version={} services={}",
            loaded.id,
            loaded.version,
            loaded.services.len()
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_009_credential_projection_preserves_w3c_dates_and_revocation_state() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-credential-view", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();
    let subject_did = format!("did:mycelix:{}", agent);

    let _did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;

    let issue_input = serde_json::json!({
        "subject_did": subject_did,
        "schema_id": "",
        "claims": {
            "purpose": "DSID qualification",
            "version": 1
        },
        "credential_types": ["QualificationCredential"],
        "issuer_name": "Mycelix DSID Laboratory",
        "expiration_days": 30,
        "enable_revocation": false,
        "strict_schema": false
    });

    let issued: Record = conductor
        .call(&cell.zome("verifiable_credential"), "issue_credential", issue_input)
        .await;

    let held: Vec<CredentialView> = conductor
        .call(&cell.zome("verifiable_credential"), "get_my_credentials_view", ())
        .await;

    assert_eq!(held.len(), 1);
    let credential = &held[0];
    assert_eq!(credential.subject_did, format!("did:mycelix:{}", agent));
    assert_eq!(credential.issuer_did, format!("did:mycelix:{}", agent));
    assert!(credential.credential_type.iter().any(|t| t == "QualificationCredential"));
    assert_eq!(credential.claims["purpose"], "DSID qualification");
    assert!(!credential.valid_from.is_empty());
    assert!(credential.valid_until.is_some());
    assert!(!credential.revoked);
    assert!(credential.schema_id.is_none());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-009",
        "credential-projection-preserves-w3c-dates-and-revocation-state",
        &dna,
        agents,
        &[&issued],
        "A real issued credential crosses the browser boundary without its proof envelope while preserving canonical W3C dates and active revocation state.",
        format!(
            "credential_id={} valid_from={} valid_until_present={} revoked={}",
            credential.id,
            credential.valid_from,
            credential.valid_until.is_some(),
            credential.revoked
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_010_cross_agent_resolution_projection_matches_did() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app(
        "dsid-alice-browser-resolver",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let bob_app = conductor.setup_app(
        "dsid-bob-browser-resolver",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();

    let alice_record: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;
    let alice_did: DidDocumentView = conductor
        .call(&alice.zome("did_registry"), "get_my_did_view", ())
        .await
        .expect("Alice DID view must exist");

    let _bob_did: DidDocumentView = conductor
        .call(&bob.zome("did_registry"), "create_did_view", ())
        .await;

    await_consistency(&[alice.clone(), bob.clone()])
        .await
        .expect("peers must reach consistency before cross-agent resolution");

    let resolved: Option<DidDocumentView> = conductor
        .call(
            &bob.zome("did_registry"),
            "resolve_did_view",
            alice_did.id.clone(),
        )
        .await;

    let resolved = resolved.expect("Bob must resolve Alice's typed DID view");
    assert_eq!(resolved.id, alice_did.id);
    assert_eq!(resolved.controller, alice_did.controller);
    assert_eq!(resolved.controller, format!("did:mycelix:{}", alice_app.agent()));
    assert_eq!(resolved.version, alice_did.version);
    assert_eq!(resolved.active, alice_did.active);

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    emit_evidence(
        "DSID-010",
        "cross-agent-resolution-projection-matches-did",
        &dna,
        agents,
        &[&alice_record],
        "Cross-agent DID resolution exposes the same canonical state through a typed projection without requiring browser-side Holochain Record decoding.",
        format!(
            "resolved_did={} version={} active={}",
            resolved.id,
            resolved.version,
            resolved.active
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_011_resolution_metadata_tracks_deactivation() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-resolution-metadata",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did_view: DidDocumentView = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await
        .expect("DID view must exist");

    let before: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            did_view.id.clone(),
        )
        .await;

    assert!(before.did_document.is_some());
    assert_eq!(before.resolution_metadata.error, None);
    assert_eq!(
        before.document_metadata.as_ref().map(|m| m.deactivated),
        Some(false)
    );
    assert_eq!(
        before.document_metadata.as_ref().map(|m| m.version_id.as_str()),
        Some("1")
    );
    assert_eq!(
        before.document_metadata.as_ref().map(|m| m.created.as_str()),
        Some(did_view.created.as_str())
    );

    let deactivated: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "deactivate_did",
            "DSID resolution metadata test".to_string(),
        )
        .await;

    let after: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            did_view.id.clone(),
        )
        .await;

    assert!(after.did_document.is_some());
    assert_eq!(after.resolution_metadata.error, None);
    assert_eq!(
        after.document_metadata.as_ref().map(|m| m.deactivated),
        Some(true)
    );
    assert_eq!(
        after.document_metadata.as_ref().map(|m| m.version_id.as_str()),
        Some("1")
    );

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-011",
        "resolution-metadata-tracks-deactivation",
        &dna,
        agents,
        &[&created, &deactivated],
        "DID resolution returns explicit document metadata and marks a deactivated DID as deactivated without discarding the resolved document.",
        format!(
            "did={} before_deactivated={} after_deactivated={} version_id={}",
            did_view.id,
            before.document_metadata.as_ref().map(|m| m.deactivated).unwrap_or(false),
            after.document_metadata.as_ref().map(|m| m.deactivated).unwrap_or(false),
            after.document_metadata
                .as_ref()
                .map(|m| m.version_id.as_str())
                .unwrap_or("")
        ),
        true,
    );
}
