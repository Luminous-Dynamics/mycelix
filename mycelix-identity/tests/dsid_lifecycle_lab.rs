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
use mycelix_crypto::{AlgorithmId, TaggedPublicKey};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
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
    evidence_scope: &'static str,
    expected: String,
    observed: String,
    pass: bool,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
enum RecoveryStatusMirror {
    Pending,
    Approved,
    ReadyToExecute,
    Completed,
    Rejected,
    Cancelled,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct RecoveryRequestMirror {
    id: String,
    did: String,
    new_agent: AgentPubKey,
    initiated_by: String,
    recovery_config_action_hash: ActionHash,
    reason: String,
    status: RecoveryStatusMirror,
    created: Timestamp,
    time_lock_expires: Option<Timestamp>,
    approval_certificate: Option<ActionHash>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidDocument {
    id: String,
    controller: AgentPubKey,
    #[serde(rename = "verificationMethod", alias = "verification_method")]
    verification_method: Vec<VerificationMethod>,
    authentication: Vec<String>,
    #[serde(rename = "assertionMethod", alias = "assertion_method", default)]
    assertion_method: Vec<String>,
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
    assertion_methods: Vec<String>,
    key_agreements: Vec<String>,
    services: Vec<DidServiceView>,
    created: String,
    updated: String,
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


#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
struct MfaFactorView {
    factor_type: MfaFactorType,
    factor_id: String,
    enrolled_at: i64,
    last_verified: i64,
    effective_strength: f32,
    active: bool,
    metadata: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq)]
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
struct DidVerificationMethodWireView {
    id: String,
    #[serde(rename = "type")]
    type_name: String,
    controller: String,
    #[serde(rename = "publicKeyMultibase")]
    public_key_multibase: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidServiceWireView {
    id: String,
    #[serde(rename = "type")]
    type_name: String,
    #[serde(rename = "serviceEndpoint")]
    endpoint: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidDocumentWireView {
    #[serde(rename = "@context")]
    context: Vec<String>,
    id: String,
    controller: String,
    #[serde(rename = "verificationMethod")]
    verification_methods: Vec<DidVerificationMethodWireView>,
    authentication: Vec<String>,
    #[serde(rename = "assertionMethod", default)]
    assertion_method: Vec<String>,
    #[serde(rename = "keyAgreement", default)]
    key_agreement: Vec<String>,
    service: Vec<DidServiceWireView>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidResolutionErrorView {
    #[serde(rename = "type")]
    type_uri: String,
    title: String,
    detail: String,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidResolutionMetadataView {
    #[serde(rename = "contentType")]
    content_type: Option<String>,
    error: Option<DidResolutionErrorView>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidDocumentMetadataView {
    #[serde(default)]
    created: Option<String>,
    #[serde(default)]
    updated: Option<String>,
    #[serde(default)]
    deactivated: Option<bool>,
    #[serde(rename = "versionId", default)]
    version_id: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
struct DidResolutionView {
    #[serde(rename = "didDocument")]
    did_document: Option<DidDocumentWireView>,
    #[serde(rename = "didResolutionMetadata")]
    resolution_metadata: DidResolutionMetadataView,
    #[serde(rename = "didDocumentMetadata")]
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
    std::env::var("DSID_QUALIFIED_HEAD_SHA")
        .or_else(|_| std::env::var("GITHUB_SHA"))
        .unwrap_or_else(|_| "local-uncommitted".to_string())
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
        evidence_scope: if records.is_empty() { "absence-bound" } else { "record-bound" },
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

    let updated: Record = conductor
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
        &[&did_record, &updated],
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
        before.resolution_metadata.content_type.as_deref(),
        Some("application/did")
    );

    let wire_doc = before.did_document.as_ref().expect("wire DID document must exist");
    assert_eq!(wire_doc.id, did_view.id);
    assert_eq!(wire_doc.controller, did_view.controller);
    assert_eq!(wire_doc.context, vec!["https://www.w3.org/ns/did/v1.1"]);
    assert_eq!(wire_doc.verification_methods.len(), 1);
    assert!(!wire_doc.verification_methods[0].public_key_multibase.is_empty());

    let wire = serde_json::to_value(&before).expect("DID resolution result must serialize");
    assert!(wire.get("didDocument").is_some());
    assert!(wire.get("didResolutionMetadata").is_some());
    assert!(wire.get("didDocumentMetadata").is_some());
    assert!(wire["didDocumentMetadata"].get("versionId").is_some());
    assert!(wire["didDocumentMetadata"].get("version_id").is_none());
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

    assert!(after.did_document.is_none());
    assert_eq!(after.resolution_metadata.error, None);
    assert_eq!(after.resolution_metadata.content_type, None);
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
        "DID resolution returns explicit deactivation metadata and withholds the deactivated DID document from the current resolution result; historical state remains separately addressable.",
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


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_012_substrate_discovery_is_cross_agent_and_author_bound() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app(
        "dsid-alice-substrate",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let bob_app = conductor.setup_app(
        "dsid-bob-substrate",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let role = "identity-resolver";
    let metadata = serde_json::json!({
        "metadata": {
            "role": role,
            "api_version": 1,
            "capabilities": ["resolve_did_view", "resolve_did_resolution"]
        }
    });

    let alice_did: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;

    let registration: Record = conductor
        .call(
            &alice.zome("did_registry"),
            "register_substrate",
            metadata,
        )
        .await;

    await_consistency(&[alice.clone(), bob.clone()])
        .await
        .expect("substrate discovery must synchronize before cross-agent lookup");

    let providers: Vec<AgentPubKey> = conductor
        .call(&bob.zome("did_registry"), "resolve_substrate", role.to_string())
        .await;

    assert!(providers.contains(alice_app.agent()));

    let _deactivated: Record = conductor
        .call(
            &alice.zome("did_registry"),
            "deactivate_did",
            "DSID substrate provider deactivation test".to_string(),
        )
        .await;

    await_consistency(&[alice.clone(), bob.clone()])
        .await
        .expect("deactivation must synchronize before provider re-resolution");

    let providers_after: Vec<AgentPubKey> = conductor
        .call(&bob.zome("did_registry"), "resolve_substrate", role.to_string())
        .await;

    assert!(!providers_after.contains(alice_app.agent()));

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    emit_evidence(
        "DSID-012",
        "substrate-discovery-cross-agent-and-author-bound",
        &dna,
        agents,
        &[&alice_did, &registration],
        "A substrate role advertisement resolves cross-agent while the DID is active and is excluded after that DID is deactivated.",
        format!(
            "role={} active_providers={:?} providers_after_deactivation={:?}",
            role, providers, providers_after
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_013_malformed_did_identifiers_fail_closed() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-invalid-did",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;

    let wrong_method: Result<Option<DidDocumentView>, _> = conductor
        .call_fallible(
            &cell.zome("did_registry"),
            "resolve_did_view",
            "did:other:not-mycelix".to_string(),
        )
        .await;

    let malformed_agent: Result<Option<DidDocumentView>, _> = conductor
        .call_fallible(
            &cell.zome("did_registry"),
            "resolve_did_view",
            "did:mycelix:not-a-valid-agent-key".to_string(),
        )
        .await;

    assert!(wrong_method.is_err());
    assert!(malformed_agent.is_err());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-013",
        "malformed-did-identifiers-fail-closed",
        &dna,
        agents,
        &[&created],
        "Malformed or foreign DID method identifiers must not resolve through the Mycelix DID resolver.",
        format!(
            "wrong_method_rejected={} malformed_agent_rejected={}",
            wrong_method.is_err(),
            malformed_agent.is_err()
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_014_initial_verification_key_is_canonical_multibase() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-key-encoding",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;

    let did: DidDocumentView = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await
        .expect("DID view must exist");

    assert_eq!(did.verification_methods.len(), 1);
    let method = &did.verification_methods[0];
    let decoded = TaggedPublicKey::from_multibase(&method.public_key_multibase)
        .expect("initial verification method must be canonical multibase");

    assert_eq!(decoded.algorithm, AlgorithmId::Ed25519);
    assert_eq!(decoded.key_bytes.len(), 32);
    assert_eq!(decoded.to_multibase(), method.public_key_multibase);

    let raw_agent = agent.get_raw_36();
    assert_eq!(&raw_agent[4..], decoded.key_bytes.as_slice());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-014",
        "initial-verification-key-is-canonical-multibase",
        &dna,
        agents,
        &[&record],
        "The initial DID verification method encodes the exact Holochain Ed25519 agent key as multicodec-prefixed base58btc multibase.",
        format!(
            "algorithm={:?} raw_key_bytes={} round_trip={}",
            decoded.algorithm,
            decoded.key_bytes.len(),
            decoded.to_multibase() == method.public_key_multibase
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_015_verification_method_rotation_preserves_historical_reference() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-key-rotation",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let initial: DidDocument = decode_entry(&created).expect("DID entry must decode");
    let old_key_id = initial
        .verification_method
        .first()
        .expect("initial verification method must exist")
        .id
        .clone();

    let new_public_key = TaggedPublicKey::new(AlgorithmId::Ed25519, vec![0x42; 32])
        .expect("test Ed25519 key must be valid")
        .to_multibase();
    let new_key_id = format!("{}#keys-2", initial.id);

    let rotated: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "rotate_key",
            serde_json::json!({
                "old_key_id": old_key_id,
                "new_method": {
                    "id": new_key_id,
                    "type": "Ed25519VerificationKey2020",
                    "controller": initial.id,
                    "publicKeyMultibase": new_public_key,
                    "algorithm": AlgorithmId::Ed25519.as_u16()
                }
            }),
        )
        .await;

    let updated: DidDocument = decode_entry(&rotated).expect("rotated DID entry must decode");
    assert_eq!(updated.version, initial.version + 1);
    assert_eq!(updated.controller, agent);
    assert!(updated
        .verification_method
        .iter()
        .any(|method| method.id == initial.verification_method[0].id));
    assert!(updated
        .verification_method
        .iter()
        .any(|method| method.id == new_key_id));
    assert!(!updated
        .authentication
        .iter()
        .any(|id| id == &initial.verification_method[0].id));
    assert!(updated.authentication.iter().any(|id| id == &new_key_id));

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-015",
        "verification-method-rotation-preserves-historical-reference",
        &dna,
        agents,
        &[&created, &rotated],
        "Verification-method rotation preserves the historical DID URL identifier while moving authentication authority to the new method.",
        format!(
            "version={} old_id_preserved={} old_auth_removed={} new_auth_active={}",
            updated.version,
            updated.verification_method.iter().any(|method| method.id == initial.verification_method[0].id),
            !updated.authentication.iter().any(|id| id == &initial.verification_method[0].id),
            updated.authentication.iter().any(|id| id == &new_key_id)
        ),
        true,
    );
}

    
#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_016_historical_versions_are_deterministically_resolvable() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-version-history",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let initial: DidDocument = decode_entry(&created).expect("initial DID must decode");

    let update_one: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "update_did_document",
            serde_json::json!({
                "verificationMethod": null,
                "authentication": null,
                "keyAgreement": null,
                "service": [{
                    "id": format!("{}#service-v2", initial.id),
                    "type": "VersionTwoService",
                    "serviceEndpoint": "https://identity.mycelix.net/v2"
                }]
            }),
        )
        .await;
    let update_one_doc: DidDocument = decode_entry(&update_one).expect("version 2 must decode");

    let update_two: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "update_did_document",
            serde_json::json!({
                "verificationMethod": null,
                "authentication": null,
                "keyAgreement": null,
                "service": [{
                    "id": format!("{}#service-v3", initial.id),
                    "type": "VersionThreeService",
                    "serviceEndpoint": "https://identity.mycelix.net/v3"
                }]
            }),
        )
        .await;
    let update_two_doc: DidDocument = decode_entry(&update_two).expect("version 3 must decode");

    assert_eq!(update_one_doc.version, 2);
    assert_eq!(update_two_doc.version, 3);

    for expected_version in [1u32, 2u32, 3u32] {
        let historical: Option<Record> = conductor
            .call(
                &cell.zome("did_registry"),
                "resolve_did_version",
                serde_json::json!({
                    "did": initial.id,
                    "version": expected_version
                }),
            )
            .await;
        let historical = historical.expect("every committed version must remain resolvable");
        let document: DidDocument = decode_entry(&historical).expect("historical DID must decode");
        assert_eq!(document.id, initial.id);
        assert_eq!(document.version, expected_version);
    }

    let missing: Option<Record> = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_version",
            serde_json::json!({
                "did": initial.id,
                "version": 99
            }),
        )
        .await;
    assert!(missing.is_none(), "unknown version must return not-found rather than guessing");

    let current: DidDocument = decode_entry(
        &conductor
            .call(&cell.zome("did_registry"), "get_my_did", ())
            .await,
    )
    .expect("canonical DID must decode");
    assert_eq!(current.version, 3);

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-016",
        "historical-versions-are-deterministically-resolvable",
        &dna,
        agents,
        &[&created, &update_one, &update_two],
        "Each committed DID version remains addressable by exact version number while canonical resolution advances to the newest version.",
        format!("current_version={} historical_versions=1,2,3 unknown_version_returns_none=true", current.version),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_017_not_found_resolution_uses_structured_w3c_error() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-resolution-error",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let missing_agent = AgentPubKey::from_raw_36(vec![7u8; 36]);
    let missing_did = format!("did:mycelix:{}", missing_agent);

    let result: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            missing_did.clone(),
        )
        .await;

    assert!(result.did_document.is_none());
    let metadata = result.document_metadata.expect("not-found must return an empty document metadata object");
    assert!(metadata.created.is_none());
    assert!(metadata.updated.is_none());
    assert!(metadata.deactivated.is_none());
    assert!(metadata.version_id.is_none());
    let error = result.resolution_metadata.error.expect("not-found must carry an error object");
    assert_eq!(error.type_uri, "https://www.w3.org/ns/did#NOT_FOUND");
    assert_eq!(error.title, "DID not found");
    assert!(!error.detail.is_empty());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-017",
        "not-found-resolution-uses-structured-w3c-error",
        &dna,
        agents,
        &[],
        "A syntactically valid but unregistered DID returns a structured W3C resolution error with didDocument=null.",
        format!(
            "did={} error_type={} title={}",
            missing_did, error.type_uri, error.title
        ),
        true,
    );
}


#[test]
fn conformance_vectors_parse_as_identity_inputs() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("docs")
        .join("identity")
        .join("did-mycelix-conformance-vectors-v0.1.json");
    let corpus: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).expect("conformance corpus must exist"))
            .expect("conformance corpus must be valid JSON");

    let vectors = corpus["did_vectors"]
        .as_array()
        .expect("did_vectors must be an array");

    let valid = vectors
        .iter()
        .find(|vector| vector["expected"] == "accept")
        .expect("at least one accepting DID vector is required");
    let valid_did = valid["input"].as_str().expect("valid DID must be a string");
    let valid_identifier = valid_did
        .strip_prefix("did:mycelix:")
        .expect("valid vector must use did:mycelix");
    AgentPubKey::try_from(valid_identifier)
        .expect("accepting vector must contain a valid Holochain AgentPubKey");

    let checksum_invalid = vectors
        .iter()
        .find(|vector| vector["id"] == "DID-002")
        .expect("DID-002 checksum-invalid vector must exist");
    let invalid_identifier = checksum_invalid["input"]
        .as_str()
        .expect("checksum-invalid DID must be a string")
        .strip_prefix("did:mycelix:")
        .expect("DID-002 must use did:mycelix");
    assert!(
        AgentPubKey::try_from(invalid_identifier).is_err(),
        "DID-002 must fail Holochain AgentPubKey parsing"
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_018_initial_mfa_factor_is_bound_to_agent_hash() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-mfa-primary-hash",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&did_record).expect("DID entry must decode");

    let state: serde_json::Value = conductor
        .call(&cell.zome("mfa"), "get_mfa_state", did.id.clone())
        .await
        .expect("MFA state must exist after DID creation");
    let factors = state["state"]["factors"]
        .as_array()
        .expect("MFA factors must be an array");
    assert_eq!(factors.len(), 1);

    let mut hasher = Sha256::new();
    hasher.update(agent.get_raw_39());
    let expected_factor_id = format!("sha256:{:x}", hasher.finalize());
    assert_eq!(
        factors[0]["factor_id"],
        expected_factor_id,
        "initial MFA factor must bind to the verifier's canonical SHA-256 contract"
    );

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-018",
        "initial-mfa-factor-is-bound-to-agent-hash",
        &dna,
        agents,
        &[&did_record],
        "DID creation initializes a primary MFA factor whose identifier exactly matches SHA-256 of the canonical AgentPubKey representation.",
        format!("factor_id_matches_expected={}", factors[0]["factor_id"] == expected_factor_id),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_019_unsupported_did_method_returns_structured_error() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-method-dispatch",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;

    let result: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            "did:key:z6Mkunsupported".to_string(),
        )
        .await;

    assert!(result.did_document.is_none());
    assert!(result.document_metadata.is_none());
    let error = result
        .resolution_metadata
        .error
        .expect("unsupported method must carry a structured error");
    assert_eq!(error.type_uri, "https://www.w3.org/ns/did#METHOD_NOT_SUPPORTED");
    assert_eq!(error.title, "DID method not supported");
    assert!(!error.detail.is_empty());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-019",
        "unsupported-did-method-returns-structured-error",
        &dna,
        agents,
        &[&created],
        "A syntactically valid DID using an unsupported method returns METHOD_NOT_SUPPORTED rather than INVALID_DID.",
        format!("error_type={} title={}", error.type_uri, error.title),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_020_recovery_configuration_cannot_be_created_for_another_did() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app(
        "dsid-recovery-owner-alice",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let bob_app = conductor.setup_app(
        "dsid-recovery-owner-bob",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();

    let did_record: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&did_record).expect("Alice DID must decode");

    let forged_setup: Result<Record, _> = conductor
        .call_fallible(
            &bob.zome("recovery"),
            "setup_recovery",
            serde_json::json!({
                "did": did.id,
                "trustees": [
                    format!("did:mycelix:{}", bob_app.agent()),
                    "did:mycelix:trustee-2",
                    "did:mycelix:trustee-3"
                ],
                "threshold": 2,
                "time_lock": 604800
            }),
        )
        .await;

    assert!(forged_setup.is_err(), "non-owner must not configure another DID's recovery state");

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    emit_evidence(
        "DSID-020",
        "recovery-configuration-cannot-be-created-for-another-did",
        &dna,
        agents,
        &[&did_record],
        "Recovery configuration must be authored only by the DID controller and must not be attachable to another agent's DID.",
        format!("cross_owner_setup_rejected={}", forged_setup.is_err()),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_021_self_recovery_anchor_updates_are_replacement_agent_bound() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-self-recovery-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-self-recovery-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-self-recovery-carol", std::slice::from_ref(&dna)).await.unwrap();

    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    let _: Record = conductor.call(&alice.zome("did_registry"), "create_did", ()).await;
    let alice_did = format!("did:mycelix:{}", alice_app.agent());

    let _: Record = conductor.call(
        &alice.zome("recovery"),
        "add_verification_anchor",
        serde_json::json!({
            "did": alice_did,
            "anchor": { "EmailHash": "sha256:deterministic-anchor" }
        }),
    ).await;

    let bob_attempt: Result<Record, _> = conductor.call_fallible(
        &bob.zome("recovery"),
        "initiate_self_recovery",
        serde_json::json!({
            "did": alice_did.clone(),
            "new_agent": bob_app.agent(),
            "reason": "DSID replacement-agent binding",
            "initial_anchor": { "EmailHash": "sha256:deterministic-anchor" }
        }),
    ).await;

    let carol_attempt: Result<Record, _> = conductor.call_fallible(
        &carol.zome("recovery"),
        "initiate_self_recovery",
        serde_json::json!({
            "did": alice_did,
            "new_agent": carol_app.agent(),
            "reason": "DSID proof-of-control gate",
            "initial_anchor": { "EmailHash": "sha256:deterministic-anchor" }
        }),
    ).await;

    assert!(bob_attempt.is_err());
    assert!(carol_attempt.is_err());

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-021",
        "self-recovery-anchor-updates-are-replacement-agent-bound",
        &dna,
        agents,
        &[],
        "Self-recovery must not create or advance an executable request from identifier/hash equality alone; proof-of-control is required.",
        format!(
            "designated_replacement_blocked={} unrelated_replacement_blocked={}",
            bob_attempt.is_err(),
            carol_attempt.is_err()
        ),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_022_recovery_quorum_is_derived_cross_agent_from_dht() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app(
        "dsid-quorum-alice",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let bob_app = conductor.setup_app(
        "dsid-quorum-bob",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let carol_app = conductor.setup_app(
        "dsid-quorum-carol",
        std::slice::from_ref(&dna),
    ).await.unwrap();

    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    let _alice_did: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;
    let _bob_did: Record = conductor
        .call(&bob.zome("did_registry"), "create_did", ())
        .await;
    let _carol_did: Record = conductor
        .call(&carol.zome("did_registry"), "create_did", ())
        .await;

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let setup: Record = conductor
        .call(
            &alice.zome("recovery"),
            "setup_recovery",
            serde_json::json!({
                "did": alice_did,
                "trustees": [alice_did.clone(), bob_did.clone(), carol_did.clone()],
                "threshold": 2,
                "time_lock": 86400
            }),
        )
        .await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()])
        .await
        .expect("recovery config must synchronize");

    let request: Record = conductor
        .call(
            &alice.zome("recovery"),
            "initiate_recovery",
            serde_json::json!({
                "did": alice_did,
                "initiator_did": alice_did.clone(),
                "new_agent": bob_app.agent(),
                "reason": "DSID cross-agent quorum"
            }),
        )
        .await;

    let request_data: RecoveryRequestMirror =
        request.entry().to_app_option().expect("request must decode").expect("request entry present");

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()])
        .await
        .expect("recovery request must synchronize");

    let bob_vote: Record = conductor
        .call(
            &bob.zome("recovery"),
            "vote_on_recovery",
            serde_json::json!({
                "request_id": request_data.id.clone(),
                "trustee_did": bob_did,
                "vote": "Approve"
            }),
        )
        .await;

    let carol_vote: Record = conductor
        .call(
            &carol.zome("recovery"),
            "vote_on_recovery",
            serde_json::json!({
                "request_id": request_data.id.clone(),
                "trustee_did": carol_did,
                "vote": "Approve"
            }),
        )
        .await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()])
        .await
        .expect("all trustee votes must synchronize");

    let derived: serde_json::Value = conductor
        .call(
            &bob.zome("recovery"),
            "get_recovery_status",
            request_data.id.clone(),
        )
        .await
        .expect("DHT-derived recovery status must exist");

    assert_eq!(derived["status"], "Approved");
    assert_eq!(derived["approve_count"], 3u64);
    assert_eq!(derived["reject_count"], 0u64);
    assert_eq!(derived["threshold"], 2u64);
    assert_eq!(derived["trustee_count"], 3u64);

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-022",
        "recovery-quorum-is-derived-cross-agent-from-dht",
        &dna,
        agents,
        &[&setup, &request, &bob_vote, &carol_vote],
        "Recovery quorum is computed from the complete DHT-visible vote set rather than requiring cross-agent mutation of the initiator's RecoveryRequest source-chain entry.",
        format!(
            "status={} approvals={} rejects={} threshold={}",
            derived["status"],
            derived["approve_count"],
            derived["reject_count"],
            derived["threshold"]
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_023_orphan_recovery_votes_are_rejected() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-orphan-vote",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let result: Result<Record, _> = conductor
        .call_fallible(
            &cell.zome("recovery"),
            "vote_on_recovery",
            serde_json::json!({
                "request_id": "recovery:nonexistent:1",
                "trustee_did": format!("did:mycelix:{}", agent),
                "vote": "Approve"
            }),
        )
        .await;

    assert!(result.is_err(), "orphan vote must be rejected");

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-023",
        "orphan-recovery-votes-are-rejected",
        &dna,
        agents,
        &[],
        "A recovery vote must reference an existing DHT-visible recovery request.",
        format!("orphan_vote_rejected={}", result.is_err()),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_024_non_trustee_recovery_votes_are_rejected() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app(
        "dsid-nontrustee-alice",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let bob_app = conductor.setup_app(
        "dsid-nontrustee-bob",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let carol_app = conductor.setup_app(
        "dsid-nontrustee-carol",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let dave_app = conductor.setup_app(
        "dsid-nontrustee-dave",
        std::slice::from_ref(&dna),
    ).await.unwrap();

    let alice = alice_app.cells()[0].clone();
    let dave = dave_app.cells()[0].clone();

    let _alice_did: Record = conductor
        .call(&alice.zome("did_registry"), "create_did", ())
        .await;
    let _bob_did: Record = conductor
        .call(&bob_app.cells()[0].zome("did_registry"), "create_did", ())
        .await;
    let _carol_did: Record = conductor
        .call(&carol_app.cells()[0].zome("did_registry"), "create_did", ())
        .await;
    let _dave_did: Record = conductor
        .call(&dave.zome("did_registry"), "create_did", ())
        .await;

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());
    let dave_did = format!("did:mycelix:{}", dave_app.agent());

    let setup: Record = conductor
        .call(
            &alice.zome("recovery"),
            "setup_recovery",
            serde_json::json!({
                "did": alice_did,
                "trustees": [alice_did.clone(), bob_did, carol_did],
                "threshold": 2,
                "time_lock": 86400
            }),
        )
        .await;

    await_consistency(&[alice.clone(), dave.clone()])
        .await
        .expect("recovery config must reach DHT");

    let request: Record = conductor
        .call(
            &alice.zome("recovery"),
            "initiate_recovery",
            serde_json::json!({
                "did": alice_did,
                "initiator_did": format!("did:mycelix:{}", alice_app.agent()),
                "new_agent": bob_app.agent(),
                "reason": "DSID non-trustee vote"
            }),
        )
        .await;
    let request_data: RecoveryRequestMirror =
        request.entry().to_app_option().expect("request must decode").expect("request entry present");

    await_consistency(&[alice.clone(), dave.clone()])
        .await
        .expect("request must reach DHT");

    let forged: Result<Record, _> = conductor
        .call_fallible(
            &dave.zome("recovery"),
            "vote_on_recovery",
            serde_json::json!({
                "request_id": request_data.id,
                "trustee_did": dave_did,
                "vote": "Approve"
            }),
        )
        .await;

    assert!(forged.is_err(), "non-trustee vote must be rejected");

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("dave", dave_app.agent().to_string());
    emit_evidence(
        "DSID-024",
        "non-trustee-recovery-votes-are-rejected",
        &dna,
        agents,
        &[&setup, &request],
        "A caller that is not a configured trustee cannot author a recovery vote.",
        format!("non_trustee_vote_rejected={}", forged.is_err()),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_025_self_recovery_latest_update_is_canonical() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-self-recovery-latest",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let _did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did = format!("did:mycelix:{}", agent);

    let anchor = serde_json::json!({
        "EmailHash": "sha256:canonical-latest-regression"
    });
    let updated: Record = conductor
        .call(
            &cell.zome("recovery"),
            "add_verification_anchor",
            serde_json::json!({
                "did": did,
                "anchor": anchor
            }),
        )
        .await;

    let view: Option<SelfRecoveryConfigView> = conductor
        .call(&cell.zome("recovery"), "get_self_recovery_view", did.clone())
        .await;
    let view = view.expect("self-recovery config must remain readable after update");

    assert_eq!(view.did, did);
    assert_eq!(view.anchors.len(), 1);
    assert_eq!(view.anchor_threshold, 1);
    assert_eq!(view.time_lock_secs, 14 * 24 * 3600);
    assert!(view.active);

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-025",
        "self-recovery-latest-update-is-canonical",
        &dna,
        agents,
        &[&updated],
        "The canonical self-recovery read path must follow the update chain and expose the newly enrolled anchor rather than the original zero-anchor configuration.",
        format!(
            "anchors={} threshold={} timelock_secs={}",
            view.anchors.len(),
            view.anchor_threshold,
            view.time_lock_secs
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_026_resolution_json_omits_absent_optional_metadata() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-resolution-json-shape",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let _did_record: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;

    let did = format!("did:mycelix:{}", agent);
    let result: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            did.clone(),
        )
        .await;

    let wire = serde_json::to_value(&result).expect("resolution must serialize");
    assert!(wire["didResolutionMetadata"].get("contentType").is_some());
    assert!(wire["didResolutionMetadata"].get("error").is_none());

    let deactivated: Record = conductor
        .call(
            &cell.zome("did_registry"),
            "deactivate_did",
            "DSID optional metadata shape".to_string(),
        )
        .await;

    let after: DidResolutionView = conductor
        .call(
            &cell.zome("did_registry"),
            "resolve_did_resolution",
            did.clone(),
        )
        .await;
    let after_wire = serde_json::to_value(&after).expect("deactivated resolution must serialize");
    assert!(after_wire["didResolutionMetadata"].get("contentType").is_none());
    assert!(after_wire["didResolutionMetadata"].get("error").is_none());
    assert_eq!(after_wire["didDocument"], serde_json::Value::Null);
    assert_eq!(after_wire["didDocumentMetadata"]["deactivated"], true);

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-026",
        "resolution-json-omits-absent-optional-metadata",
        &dna,
        agents,
        &[&deactivated],
        "Optional DID resolution metadata fields are omitted when absent rather than serialized as JSON null.",
        format!(
            "active_error_omitted={} deactivated_content_type_omitted={} deactivated_error_omitted={}",
            wire["didResolutionMetadata"].get("error").is_none(),
            after_wire["didResolutionMetadata"].get("contentType").is_none(),
            after_wire["didResolutionMetadata"].get("error").is_none()
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_027_generic_update_rejects_legacy_untagged_did_key() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app(
        "dsid-strict-did-key",
        std::slice::from_ref(&dna),
    ).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let _created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let current: DidDocumentView = conductor
        .call(&cell.zome("did_registry"), "get_my_did_view", ())
        .await
        .expect("canonical DID must exist");

    let raw = bs58::encode([0x42u8; 32])
        .with_alphabet(bs58::Alphabet::BITCOIN)
        .into_string();
    let legacy_multibase = format!("z{}", raw);

    let mut methods = current.verification_methods
        .iter()
        .map(|method| serde_json::json!({
            "id": method.id,
            "type": method.type_name,
            "controller": method.controller,
            "publicKeyMultibase": method.public_key_multibase
        }))
        .collect::<Vec<_>>();
    methods[0]["publicKeyMultibase"] = serde_json::Value::String(legacy_multibase);

    let result: Result<Record, _> = conductor
        .call_fallible(
            &cell.zome("did_registry"),
            "update_did_document",
            serde_json::json!({
                "verificationMethod": methods,
                "authentication": current.verification_methods
                    .iter()
                    .map(|method| method.id.clone())
                    .collect::<Vec<_>>(),
                "keyAgreement": null,
                "service": []
            }),
        )
        .await;

    assert!(result.is_err(), "generic DID update must reject legacy untagged verification keys");

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-027",
        "generic-update-rejects-legacy-untagged-did-key",
        &dna,
        agents,
        &[],
        "The DID integrity boundary must reject legacy raw-key encoding even when the generic coordinator update path is used.",
        format!("legacy_key_update_rejected={}", result.is_err()),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_028_recovery_time_lock_arming_is_cross_agent_safe() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-arm-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-arm-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-arm-carol", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    for cell in [&alice, &bob, &carol] {
        let _: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    }

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let setup: Record = conductor.call(&alice.zome("recovery"), "setup_recovery", serde_json::json!({
        "did": alice_did,
        "trustees": [alice_did.clone(), bob_did.clone(), carol_did.clone()],
        "threshold": 2,
        "time_lock": 86400
    })).await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(&alice.zome("recovery"), "initiate_recovery", serde_json::json!({
        "did": alice_did,
        "initiator_did": alice_did.clone(),
        "new_agent": bob_app.agent(),
        "reason": "DSID time-lock arming"
    })).await;
    let req: RecoveryRequestMirror = request.entry().to_app_option().unwrap().unwrap();

    for (cell, trustee) in [(&bob, bob_did), (&carol, carol_did)] {
        let _: Record = conductor.call(&cell.zome("recovery"), "vote_on_recovery", serde_json::json!({
            "request_id": req.id.clone(),
            "trustee_did": trustee,
            "vote": "Approve"
        })).await;
    }

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let armed: Record = conductor.call(&alice.zome("recovery"), "arm_recovery_time_lock", req.id.clone()).await;
    let armed_req: RecoveryRequestMirror = armed.entry().to_app_option().unwrap().unwrap();
    assert_eq!(armed_req.status, RecoveryStatusMirror::Approved);
    assert!(armed_req.time_lock_expires.is_some());
    assert!(armed_req.approval_certificate.is_some());

    let observed: Option<Record> = conductor.call(&bob.zome("recovery"), "get_recovery_request", req.id.clone()).await;
    let observed_req: RecoveryRequestMirror = observed.unwrap().entry().to_app_option().unwrap().unwrap();
    assert_eq!(observed_req.status, RecoveryStatusMirror::Approved);
    assert!(observed_req.time_lock_expires.is_some());

    let early: Result<Record, _> = conductor.call_fallible(&bob.zome("recovery"), "execute_recovery", req.id).await;
    assert!(early.is_err());

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-028",
        "recovery-time-lock-arming-is-cross-agent-safe",
        &dna,
        agents,
        &[&setup, &request, &armed],
        "The original request author can arm a DHT-derived approved quorum, and the replacement agent can read the armed request without being able to execute before the lock expires.",
        format!(
            "armed_status={:?} timelock_present={} certificate_present={} early_execute_rejected={}",
            observed_req.status,
            observed_req.time_lock_expires.is_some(),
            observed_req.approval_certificate.is_some(),
            early.is_err()
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_029_recovery_request_id_is_deterministically_derived() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-request-id", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-request-id-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-request-id-carol", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    let _: Record = conductor.call(&alice.zome("did_registry"), "create_did", ()).await;
    let _: Record = conductor.call(&bob.zome("did_registry"), "create_did", ()).await;

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let _: Record = conductor.call(
        &alice.zome("recovery"),
        "setup_recovery",
        serde_json::json!({
            "did": alice_did,
            "trustees": [alice_did.clone(), bob_did, carol_did],
            "threshold": 2,
            "time_lock": 86400
        }),
    ).await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(
        &alice.zome("recovery"),
        "initiate_recovery",
        serde_json::json!({
            "did": alice_did,
            "initiator_did": format!("did:mycelix:{}", alice_app.agent()),
            "new_agent": bob_app.agent(),
            "reason": "DSID deterministic request ID"
        }),
    ).await;

    let decoded: RecoveryRequestMirror = request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();

    assert_eq!(
        decoded.id,
        format!("recovery:{}:{}", decoded.did, decoded.created.as_micros())
    );

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-029",
        "recovery-request-id-is-deterministically-derived",
        &dna,
        agents,
        &[&request],
        "Recovery request identity must be a deterministic function of the protected DID and immutable creation timestamp.",
        format!("request_id_matches_derivation={}", decoded.id == format!("recovery:{}:{}", decoded.did, decoded.created.as_micros())),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_030_rejected_recovery_cannot_resurrect_in_derived_status() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-terminal-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-terminal-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-terminal-carol", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    for cell in [&alice, &bob, &carol] {
        let _: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    }

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let _: Record = conductor.call(&alice.zome("recovery"), "setup_recovery", serde_json::json!({
        "did": alice_did,
        "trustees": [alice_did.clone(), bob_did.clone(), carol_did.clone()],
        "threshold": 3,
        "time_lock": 86400
    })).await;
    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(&alice.zome("recovery"), "initiate_recovery", serde_json::json!({
        "did": alice_did,
        "initiator_did": format!("did:mycelix:{}", alice_app.agent()),
        "new_agent": bob_app.agent(),
        "reason": "DSID terminal rejection"
    })).await;
    let req: RecoveryRequestMirror = request.entry().to_app_option().unwrap().unwrap();

    let _: Record = conductor.call(&bob.zome("recovery"), "vote_on_recovery", serde_json::json!({
        "request_id": req.id.clone(),
        "trustee_did": bob_did,
        "vote": "Reject"
    })).await;
    let _: Record = conductor.call(&carol.zome("recovery"), "vote_on_recovery", serde_json::json!({
        "request_id": req.id.clone(),
        "trustee_did": carol_did,
        "vote": "Reject"
    })).await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();
    let status: serde_json::Value = conductor.call(&bob.zome("recovery"), "get_recovery_status", req.id.clone()).await.unwrap();
    assert_eq!(status["status"], "Rejected");

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-030",
        "rejected-recovery-cannot-resurrect-in-derived-status",
        &dna,
        agents,
        &[&request],
        "A rejected recovery request remains terminal in the DHT-derived quorum projection.",
        format!("derived_status={}", status["status"]),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_031_recovery_request_pins_config_snapshot() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-config-pin-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-config-pin-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-config-pin-carol", std::slice::from_ref(&dna)).await.unwrap();

    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    for cell in [&alice, &bob, &carol] {
        let _: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    }

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let config: Record = conductor.call(
        &alice.zome("recovery"),
        "setup_recovery",
        serde_json::json!({
            "did": alice_did,
            "trustees": [alice_did.clone(), bob_did, carol_did],
            "threshold": 2,
            "time_lock": 86400
        }),
    ).await;

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(
        &alice.zome("recovery"),
        "initiate_recovery",
        serde_json::json!({
            "did": alice_did,
            "initiator_did": format!("did:mycelix:{}", alice_app.agent()),
            "new_agent": bob_app.agent(),
            "reason": "DSID config snapshot pin"
        }),
    ).await;

    let req: RecoveryRequestMirror = request.entry().to_app_option().unwrap().unwrap();
    assert_eq!(req.recovery_config_action_hash, config.action_address());

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-031",
        "recovery-request-pins-config-snapshot",
        &dna,
        agents,
        &[&config, &request],
        "Every recovery request must remain governed by the exact recovery configuration snapshot that existed when the request was created.",
        format!(
            "config_snapshot_bound={}",
            req.recovery_config_action_hash == config.action_address()
        ),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_032_recovery_request_reader_follows_latest_update() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-request-latest-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-request-latest-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-request-latest-carol", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    for cell in [&alice, &bob, &carol] {
        let _: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    }

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let _: Record = conductor.call(&alice.zome("recovery"), "setup_recovery", serde_json::json!({
        "did": alice_did.clone(),
        "trustees": [alice_did.clone(), bob_did.clone(), carol_did.clone()],
        "threshold": 3,
        "time_lock": 86400
    })).await;
    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(&alice.zome("recovery"), "initiate_recovery", serde_json::json!({
        "did": alice_did.clone(),
        "initiator_did": alice_did.clone(),
        "new_agent": bob_app.agent(),
        "reason": "DSID latest request reader"
    })).await;
    let req: RecoveryRequestMirror = request.entry().to_app_option().unwrap().unwrap();

    let cancelled: Record = conductor.call(&alice.zome("recovery"), "cancel_recovery", req.id.clone()).await;
    let cancelled_req: RecoveryRequestMirror = cancelled.entry().to_app_option().unwrap().unwrap();
    assert_eq!(cancelled_req.status, RecoveryStatusMirror::Cancelled);

    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let observed: Option<Record> = conductor.call(&bob.zome("recovery"), "get_recovery_request", req.id).await;
    let observed_req: RecoveryRequestMirror = observed.unwrap().entry().to_app_option().unwrap().unwrap();
    assert_eq!(observed_req.status, RecoveryStatusMirror::Cancelled);

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-032",
        "recovery-request-reader-follows-latest-update",
        &dna,
        agents,
        &[&request, &cancelled],
        "The DHT request-ID reader must follow the RecoveryRequest update chain and expose the latest canonical request state.",
        format!("observed_latest_status={:?}", observed_req.status),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_033_cancel_uses_pinned_recovery_config() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let alice_app = conductor.setup_app("dsid-cancel-pin-alice", std::slice::from_ref(&dna)).await.unwrap();
    let bob_app = conductor.setup_app("dsid-cancel-pin-bob", std::slice::from_ref(&dna)).await.unwrap();
    let carol_app = conductor.setup_app("dsid-cancel-pin-carol", std::slice::from_ref(&dna)).await.unwrap();
    let alice = alice_app.cells()[0].clone();
    let bob = bob_app.cells()[0].clone();
    let carol = carol_app.cells()[0].clone();

    for cell in [&alice, &bob, &carol] {
        let _: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    }

    let alice_did = format!("did:mycelix:{}", alice_app.agent());
    let bob_did = format!("did:mycelix:{}", bob_app.agent());
    let carol_did = format!("did:mycelix:{}", carol_app.agent());

    let config: Record = conductor.call(&alice.zome("recovery"), "setup_recovery", serde_json::json!({
        "did": alice_did.clone(),
        "trustees": [alice_did.clone(), bob_did.clone(), carol_did.clone()],
        "threshold": 2,
        "time_lock": 86400
    })).await;
    await_consistency(&[alice.clone(), bob.clone(), carol.clone()]).await.unwrap();

    let request: Record = conductor.call(&alice.zome("recovery"), "initiate_recovery", serde_json::json!({
        "did": alice_did.clone(),
        "initiator_did": alice_did,
        "new_agent": bob_app.agent(),
        "reason": "DSID cancel pinned config"
    })).await;
    let req: RecoveryRequestMirror = request.entry().to_app_option().unwrap().unwrap();
    assert_eq!(req.recovery_config_action_hash, config.action_address());

    let cancelled: Record = conductor.call(&alice.zome("recovery"), "cancel_recovery", req.id).await;
    let cancelled_req: RecoveryRequestMirror = cancelled.entry().to_app_option().unwrap().unwrap();
    assert_eq!(cancelled_req.status, RecoveryStatusMirror::Cancelled);

    let mut agents = BTreeMap::new();
    agents.insert("alice", alice_app.agent().to_string());
    agents.insert("bob", bob_app.agent().to_string());
    agents.insert("carol", carol_app.agent().to_string());
    emit_evidence(
        "DSID-033",
        "cancel-uses-pinned-recovery-config",
        &dna,
        agents,
        &[&config, &request, &cancelled],
        "Cancellation must authorize against the exact recovery configuration snapshot pinned to the request.",
        format!("pinned_config_matches={}", cancelled_req.recovery_config_action_hash == config.action_address()),
        true,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_034_canonical_did_reads_follow_latest_update_chain() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor.setup_app("dsid-did-latest", std::slice::from_ref(&dna)).await.unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor.call(&cell.zome("did_registry"), "create_did", ()).await;
    let initial: DidDocument = decode_entry(&created).unwrap();

    let updated: Record = conductor.call(
        &cell.zome("did_registry"),
        "update_did_document",
        serde_json::json!({
            "verificationMethod": null,
            "authentication": null,
            "keyAgreement": null,
            "service": [{
                "id": format!("{}#latest", initial.id),
                "type": "LatestStateProbe",
                "serviceEndpoint": "https://identity.mycelix.net/latest"
            }]
        }),
    ).await;
    let updated_doc: DidDocument = decode_entry(&updated).unwrap();
    assert_eq!(updated_doc.version, initial.version + 1);

    let canonical: Record = conductor.call(&cell.zome("did_registry"), "get_my_did", ()).await;
    let canonical_doc: DidDocument = decode_entry(&canonical).unwrap();
    assert_eq!(canonical_doc.version, updated_doc.version);
    assert_eq!(canonical_doc.updated, updated_doc.updated);

    let did = format!("did:mycelix:{}", agent);
    let resolved: DidResolutionView = conductor.call(
        &cell.zome("did_registry"),
        "resolve_did_resolution",
        did,
    ).await;
    let metadata = resolved.document_metadata.expect("successful resolution must return document metadata");
    let expected_version_id = updated_doc.version.to_string();
    assert_eq!(metadata.version_id.as_deref(), Some(expected_version_id.as_str()));
    assert!(metadata.updated.is_some());

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-034",
        "canonical-did-reads-follow-latest-update-chain",
        &dna,
        agents,
        &[&created, &updated, &canonical],
        "Canonical DID reads must follow the latest valid update chain rather than dereferencing only the original AgentToDid action.",
        format!(
            "canonical_version={} resolved_version_id={} latest_update_visible={}",
            canonical_doc.version,
            metadata.version_id.as_deref().unwrap_or("missing"),
            canonical_doc.version == updated_doc.version
        ),
        true,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_035_request_status_update_is_cross_agent_and_dht_indexed() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let requester_app = conductor
        .setup_app("dsid-vc-requester", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let issuer_app = conductor
        .setup_app("dsid-vc-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let requester = requester_app.cells()[0].clone();
    let issuer = issuer_app.cells()[0].clone();
    let requester_did = format!("did:mycelix:{}", requester_app.agent());
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());

    let _: Record = conductor
        .call(&requester.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;

    let request: Record = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "request_credential",
            serde_json::json!({
                "issuer_did": issuer_did,
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID cross-agent request"
                },
                "evidence": []
            }),
        )
        .await;

    let request_id = request
        .entry()
        .to_app_option::<serde_json::Value>()
        .ok()
        .flatten()
        .and_then(|v| v.get("id").and_then(serde_json::Value::as_str).map(str::to_owned))
        .expect("credential request entry must expose its ID");

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("requester and issuer must reach DHT consistency");

    let pending_before: Vec<Record> = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "get_pending_requests",
            format!("did:mycelix:{}", issuer_app.agent()),
        )
        .await;
    assert_eq!(pending_before.len(), 1);

    let under_review: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id,
                "new_status": "UnderReview"
            }),
        )
        .await;

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("updated request must reach the requester after issuer commit");

    let pending_after_review: Vec<Record> = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "get_pending_requests",
            format!("did:mycelix:{}", issuer_app.agent()),
        )
        .await;
    assert_eq!(pending_after_review.len(), 1);

    let approved: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id,
                "new_status": "Approved"
            }),
        )
        .await;

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("approved request must reach DHT consistency");

    let pending_after_approval: Vec<Record> = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "get_pending_requests",
            format!("did:mycelix:{}", issuer_app.agent()),
        )
        .await;
    assert!(pending_after_approval.is_empty());

    let under_review_status = under_review
        .entry()
        .to_app_option::<serde_json::Value>()
        .ok()
        .flatten()
        .and_then(|v| v.get("status").cloned());
    let approved_status = approved
        .entry()
        .to_app_option::<serde_json::Value>()
        .ok()
        .flatten()
        .and_then(|v| v.get("status").cloned());

    let mut agents = BTreeMap::new();
    agents.insert("requester", requester_app.agent().to_string());
    agents.insert("issuer", issuer_app.agent().to_string());
    emit_evidence(
        "DSID-035",
        "request-status-update-is-cross-agent-and-dht-indexed",
        &dna,
        agents,
        &[&request, &under_review, &approved],
        "A requester-owned credential request must be discoverable by its target issuer through the DHT index, and the issuer must be able to append authorized status transitions without source-chain confusion.",
        format!(
            "pending_before={} under_review_status={} pending_after_review={} approved_status={} pending_after_approval={}",
            pending_before.len(),
            under_review_status.map(|v| v.to_string()).unwrap_or_else(|| "missing".into()),
            pending_after_review.len(),
            approved_status.map(|v| v.to_string()).unwrap_or_else(|| "missing".into()),
            pending_after_approval.len()
        ),
        pending_before.len() == 1
            && under_review_status.as_ref() == Some(&serde_json::Value::String("UnderReview".into()))
            && pending_after_review.len() == 1
            && approved_status.as_ref() == Some(&serde_json::Value::String("Approved".into()))
            && pending_after_approval.is_empty(),
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_036_issued_state_proves_credential_fulfillment() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let requester_app = conductor
        .setup_app("dsid-vc-issued-requester", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let issuer_app = conductor
        .setup_app("dsid-vc-issued-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let requester = requester_app.cells()[0].clone();
    let issuer = issuer_app.cells()[0].clone();
    let requester_did = format!("did:mycelix:{}", requester_app.agent());
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());

    let _: Record = conductor
        .call(&requester.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;

    let request: Record = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "request_credential",
            serde_json::json!({
                "issuer_did": issuer_did,
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID proof-carrying issuance"
                },
                "evidence": []
            }),
        )
        .await;

    let request_value: serde_json::Value = request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let request_id = request_value["id"]
        .as_str()
        .expect("credential request must have an ID")
        .to_owned();

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("requester and issuer must reach DHT consistency");

    let _: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id,
                "new_status": "UnderReview"
            }),
        )
        .await;
    let approved: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id,
                "new_status": "Approved"
            }),
        )
        .await;

    let free_issued: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id,
                "new_status": "Issued"
            }),
        )
        .await;
    assert!(
        free_issued.is_err(),
        "Issued must require a proof-carrying credential fulfillment"
    );

    let credential: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential_for_request",
            serde_json::json!({
                "request_id": request_id,
                "claims": {
                    "degree": "DSID proof-carrying issuance"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Qualification Issuer",
                "expiration_days": 365,
                "enable_revocation": true,
                "strict_schema": false
            }),
        )
        .await;

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("credential fulfillment and request update must reach consistency");

    let final_request: Option<Record> = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "get_credential_request",
            serde_json::json!({
                "issuer_did": format!("did:mycelix:{}", issuer_app.agent()),
                "request_id": request_id
            }),
        )
        .await;
    let final_request = final_request.expect("issued request must resolve through the issuer DHT index");
    let final_value: serde_json::Value = final_request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();

    assert_eq!(final_value["status"], "Issued");
    let issued_action = final_value["issued_credential"]
        .as_str()
        .expect("Issued request must carry a credential action hash");
    assert_eq!(issued_action, credential.action_address().to_string());

    let credential_value: serde_json::Value = credential
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    assert_eq!(
        credential_value["issuer"]["id"]
            .as_str()
            .or_else(|| credential_value["issuer"].as_str()),
        Some(issuer_did.as_str())
    );
    assert_eq!(
        credential_value["credentialSubject"]["id"].as_str(),
        Some(requester_did.as_str())
    );

    assert_eq!(
        credential_value["proof"]["cryptosuite"].as_str(),
        Some("eddsa-jcs-2022"),
        "newly issued request-bound Mycelix credentials must use the W3C JCS proof profile"
    );

    assert_eq!(
        credential_value["proof"]["verificationMethod"].as_str(),
        Some(format!(
            "{}#keys-1-multikey",
            issuer_did
        ).as_str()),
        "JCS credentials must resolve their W3C proof through the DID's Multikey representation"
    );
    assert_eq!(
        credential_value["proof"]["@context"],
        credential_value["@context"],
        "W3C JCS proof must serialize the credential @context in its proof configuration"
    );
    assert!(credential_value["proof"].get("algorithm").is_none());

    let verified: serde_json::Value = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "verify_credential",
            credential_value["id"].clone(),
        )
        .await;
    assert_eq!(verified["valid"], true);

    let duplicate_fulfillment: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "issue_credential_for_request",
            serde_json::json!({
                "request_id": request_id,
                "claims": {
                    "degree": "DSID proof-carrying issuance"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Qualification Issuer",
                "expiration_days": 365,
                "enable_revocation": true,
                "strict_schema": false
            }),
        )
        .await;
    assert!(
        duplicate_fulfillment.is_err(),
        "an Issued request must not be fulfilled twice"
    );

    let mut agents = BTreeMap::new();
    agents.insert("requester", requester_app.agent().to_string());
    agents.insert("issuer", issuer_app.agent().to_string());
    emit_evidence(
        "DSID-036",
        "issued-state-proves-credential-fulfillment",
        &dna,
        agents,
        &[&request, &approved, &credential, &final_request],
        "The Issued request state must carry the exact credential ActionHash that fulfills the request, and issuance must be cryptographically consistent with the request's issuer, subject, and schema.",
        format!(
            "approved_record_present={} free_issued_rejected={} issued_pointer_matches={} native_cryptosuite={} credential_verified={} duplicate_fulfillment_rejected={}",
            approved.entry().to_app_option::<serde_json::Value>().is_ok(),
            free_issued.is_err(),
            issued_action == credential.action_address().to_string(),
            credential_value["proof"]["cryptosuite"].as_str() == Some("eddsa-jcs-2022"),
            verified["valid"] == true,
            duplicate_fulfillment.is_err()
        ),
        free_issued.is_err()
            && final_value["status"] == "Issued"
            && issued_action == credential.action_address().to_string()
            && credential_value["proof"]["cryptosuite"].as_str() == Some("eddsa-jcs-2022")
            && verified["valid"] == true
            && duplicate_fulfillment.is_err(),
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_037_assertion_method_authorizes_w3c_multikey() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let app = conductor
        .setup_app("dsid-assertion-method", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let cell = app.cells()[0].clone();
    let agent = app.agent().clone();

    let created: Record = conductor
        .call(&cell.zome("did_registry"), "create_did", ())
        .await;
    let did: DidDocument = decode_entry(&created).expect("DID entry must decode");

    let multikey_id = format!("{}#keys-1-multikey", did.id);
    let legacy_id = format!("{}#keys-1", did.id);

    assert!(
        did.assertion_method.contains(&multikey_id),
        "W3C Multikey must be explicitly authorized for assertionMethod"
    );
    assert!(
        did.assertion_method.contains(&legacy_id),
        "Legacy assertion authorization must remain available for compatibility"
    );

    let multikey = did
        .verification_method
        .iter()
        .find(|method| method.id == multikey_id)
        .expect("W3C Multikey verification method must exist");
    assert_eq!(multikey.type_, "Multikey");

    let wire: DidDocumentWireView = conductor
        .call(&cell.zome("did_registry"), "resolve_did_wire", did.id.clone())
        .await;
    assert!(wire.assertion_method.contains(&multikey_id));
    assert!(
        wire.context.contains(&"https://w3id.org/security/multikey/v1".to_string()),
        "DID wire output must declare the Multikey vocabulary context"
    );

    let mut agents = BTreeMap::new();
    agents.insert("alice", agent.to_string());
    emit_evidence(
        "DSID-037",
        "assertion-method-authorizes-w3c-multikey",
        &dna,
        agents,
        &[&created],
        "A DID using W3C Data Integrity assertionMethod proofs must explicitly authorize the exact Multikey verification method in both canonical and wire document representations.",
        format!(
            "multikey_present={} assertion_authorized={} wire_assertion_authorized={} legacy_authorized={}",
            did.verification_method.iter().any(|m| m.id == multikey_id),
            did.assertion_method.contains(&multikey_id),
            wire.assertion_method.contains(&multikey_id),
            did.assertion_method.contains(&legacy_id)
        ),
        did.verification_method.iter().any(|m| m.id == multikey_id)
            && did.assertion_method.contains(&multikey_id)
            && wire.assertion_method.contains(&multikey_id)
            && did.assertion_method.contains(&legacy_id),
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_038_deactivated_issuer_fails_closed_in_credential_verification() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let issuer_app = conductor
        .setup_app("dsid-deactivated-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let holder_app = conductor
        .setup_app("dsid-deactivated-holder", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let issuer = issuer_app.cells()[0].clone();
    let holder = holder_app.cells()[0].clone();
    let holder_did = format!("did:mycelix:{}", holder_app.agent());

    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&holder.zome("did_registry"), "create_did", ())
        .await;

    let credential: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential",
            serde_json::json!({
                "subject_did": holder_did,
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID active-issuer credential"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Qualification Issuer",
                "expiration_days": 365,
                "enable_revocation": true,
                "strict_schema": false
            }),
        )
        .await;

    let credential_value: serde_json::Value = credential
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let credential_id = credential_value["id"]
        .as_str()
        .expect("issued credential must have an ID")
        .to_owned();

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("issuer and holder must reach DHT consistency");

    let active: serde_json::Value = conductor
        .call(
            &holder.zome("verifiable_credential"),
            "verify_credential",
            credential_id.clone(),
        )
        .await;
    assert_eq!(
        active["valid"], true,
        "credential must verify while its issuer DID is active: {active}"
    );

    let _: Record = conductor
        .call(
            &issuer.zome("did_registry"),
            "deactivate_did",
            "DSID issuer deactivation qualification",
        )
        .await;

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("deactivation must reach holder DHT consistency");

    let inactive: serde_json::Value = conductor
        .call(
            &holder.zome("verifiable_credential"),
            "verify_credential",
            credential_id.clone(),
        )
        .await;

    assert_eq!(inactive["valid"], false);
    let errors = inactive["errors"]
        .as_array()
        .expect("failed verification must include errors");
    assert!(
        errors
            .iter()
            .any(|error| error.as_str() == Some("Issuer DID is not active")),
        "deactivated issuer must be an explicit verification failure: {inactive}"
    );

    let issuer_active: bool = conductor
        .call(
            &holder.zome("did_registry"),
            "is_did_active",
            issuer_did.clone(),
        )
        .await;
    assert!(!issuer_active);

    let mut agents = BTreeMap::new();
    agents.insert("issuer", issuer_app.agent().to_string());
    agents.insert("holder", holder_app.agent().to_string());
    emit_evidence(
        "DSID-038",
        "deactivated-issuer-fails-closed",
        &dna,
        agents,
        &[&credential],
        "A credential that is cryptographically valid must fail current-state verification after its issuer DID is deactivated.",
        format!(
            "verified_active={} verified_after_deactivation={} explicit_issuer_inactive_error={} did_registry_active={}",
            active["valid"] == true,
            inactive["valid"] == true,
            errors.iter().any(|error| error.as_str() == Some("Issuer DID is not active")),
            issuer_active
        ),
        active["valid"] == true
            && inactive["valid"] == false
            && errors
                .iter()
                .any(|error| error.as_str() == Some("Issuer DID is not active"))
            && !issuer_active,
    );
}


#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_039_presigned_proof_admission_fails_closed() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let issuer_app = conductor
        .setup_app("dsid-presigned-admission", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let issuer = issuer_app.cells()[0].clone();
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());

    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;

    let native_tamper: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "issue_credential_with_proof",
            serde_json::json!({
                "credential": {
                    "@context": ["https://www.w3.org/ns/credentials/v2"],
                    "id": "urn:uuid:dsid-039-native-tamper",
                    "type": ["VerifiableCredential"],
                    "issuer": issuer_did.clone(),
                    "validFrom": "2026-10-04T00:00:00Z",
                    "credentialSubject": {
                        "id": issuer_did,
                        "claim": "tampered"
                    },
                    "credentialSchema": {
                        "id": "mycelix:schema:test:v1",
                        "type": "JsonSchema"
                    },
                    "proof": {
                        "type": "DataIntegrityProof",
                        "created": "2026-10-04T00:00:00Z",
                        "verificationMethod": format!("{}#keys-1", issuer_did),
                        "proofPurpose": "assertionMethod",
                        "proofValue": "znot-a-signature",
                        "cryptosuite": "mycelix-blake2b-ed25519-2026",
                        "algorithm": 60673
                    },
                    "mycelix_schema_id": "mycelix:schema:test:v1",
                    "mycelix_created": 0
                }
            }),
        )
        .await;
    assert!(
        native_tamper.is_err(),
        "tampered native proof must be rejected before a credential record is committed"
    );

    let jcs_tamper: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "issue_credential_with_proof",
            serde_json::json!({
                "credential": {
                    "@context": ["https://www.w3.org/ns/credentials/v2"],
                    "id": "urn:uuid:dsid-039-jcs-tamper",
                    "type": ["VerifiableCredential"],
                    "issuer": issuer_did.clone(),
                    "validFrom": "2026-10-04T00:00:00Z",
                    "credentialSubject": {
                        "id": issuer_did,
                        "claim": "tampered"
                    },
                    "credentialSchema": {
                        "id": "mycelix:schema:test:v1",
                        "type": "JsonSchema"
                    },
                    "proof": {
                        "type": "DataIntegrityProof",
                        "created": "2026-10-04T00:00:00Z",
                        "verificationMethod": format!("{}#keys-1-multikey", issuer_did),
                        "proofPurpose": "assertionMethod",
                        "proofValue": "z1",
                        "cryptosuite": "eddsa-jcs-2022",
                        "@context": ["https://www.w3.org/ns/credentials/v2"]
                    },
                    "mycelix_schema_id": "mycelix:schema:test:v1",
                    "mycelix_created": 0
                }
            }),
        )
        .await;
    assert!(
        jcs_tamper.is_err(),
        "tampered JCS proof must be rejected at the integrity boundary"
    );

    let unsupported_pqc: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "issue_credential_with_proof",
            serde_json::json!({
                "credential": {
                    "@context": ["https://www.w3.org/ns/credentials/v2"],
                    "id": "urn:uuid:dsid-039-pqc",
                    "type": ["VerifiableCredential"],
                    "issuer": issuer_did.clone(),
                    "validFrom": "2026-10-04T00:00:00Z",
                    "credentialSubject": {
                        "id": issuer_did,
                        "claim": "unsupported suite"
                    },
                    "credentialSchema": {
                        "id": "mycelix:schema:test:v1",
                        "type": "JsonSchema"
                    },
                    "proof": {
                        "type": "DataIntegrityProof",
                        "created": "2026-10-04T00:00:00Z",
                        "verificationMethod": format!("{}#keys-1", issuer_did),
                        "proofPurpose": "assertionMethod",
                        "proofValue": "z1",
                        "cryptosuite": "hybrid-eddsa-mldsa65-rdfc-2024"
                    },
                    "mycelix_schema_id": "mycelix:schema:test:v1",
                    "mycelix_created": 0
                }
            }),
        )
        .await;
    assert!(
        unsupported_pqc.is_err(),
        "unsupported PQC/hybrid suites must be rejected rather than admitted unverified"
    );

    let mut agents = BTreeMap::new();
    agents.insert("issuer", issuer_app.agent().to_string());
    emit_evidence(
        "DSID-039",
        "presigned-proof-admission-fails-closed",
        &dna,
        agents,
        &[],
        "Pre-signed credential paths must never commit cryptographically unverifiable proofs: malformed legacy/native and JCS signatures are rejected at the DHT integrity boundary, while unsupported PQC/hybrid suites are rejected before commit.",
        format!(
            "native_tamper_rejected={} jcs_tamper_rejected={} unsupported_pqc_rejected={}",
            native_tamper.is_err(),
            jcs_tamper.is_err(),
            unsupported_pqc.is_err()
        ),
        native_tamper.is_err() && jcs_tamper.is_err() && unsupported_pqc.is_err(),
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_040_request_claims_are_bound_to_issued_credential() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let requester_app = conductor
        .setup_app("dsid-vc-claims-requester", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let issuer_app = conductor
        .setup_app("dsid-vc-claims-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let requester = requester_app.cells()[0].clone();
    let issuer = issuer_app.cells()[0].clone();
    let requester_did = format!("did:mycelix:{}", requester_app.agent());
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());

    let _: Record = conductor
        .call(&requester.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;

    let request: Record = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "request_credential",
            serde_json::json!({
                "issuer_did": issuer_did,
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID requested claim"
                },
                "evidence": []
            }),
        )
        .await;

    let request_value: serde_json::Value = request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let request_id = request_value["id"]
        .as_str()
        .expect("credential request must have an ID")
        .to_owned();

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("requester and issuer must reach DHT consistency");

    let _: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id.clone(),
                "new_status": "UnderReview"
            }),
        )
        .await;
    let _: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id.clone(),
                "new_status": "Approved"
            }),
        )
        .await;

    let mismatch: Result<Record, _> = conductor
        .call_fallible(
            &issuer.zome("verifiable_credential"),
            "issue_credential_for_request",
            serde_json::json!({
                "request_id": request_id.clone(),
                "claims": {
                    "degree": "DSID forged claim"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Claims Issuer",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false
            }),
        )
        .await;
    assert!(
        mismatch.is_err(),
        "issuance must reject claims that do not satisfy the approved request"
    );

    let issued_after_mismatch: Vec<Record> = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "get_my_issued_credentials",
            (),
        )
        .await;
    assert!(
        issued_after_mismatch.is_empty(),
        "claim mismatch must be rejected before an orphan credential is committed"
    );

    let credential: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential_for_request",
            serde_json::json!({
                "request_id": request_id.clone(),
                "claims": {
                    "degree": "DSID requested claim",
                    "issuer_note": "additional attestation"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Claims Issuer",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false
            }),
        )
        .await;

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("successful credential fulfillment must reach the requester");

    let final_request: Option<Record> = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "get_credential_request",
            serde_json::json!({
                "issuer_did": issuer_did.clone(),
                "request_id": request_id.clone()
            }),
        )
        .await;
    let final_request = final_request.expect("fulfilled request must resolve through issuer index");
    let final_value: serde_json::Value = final_request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();

    assert_eq!(final_value["status"], "Issued");
    let expected_action = credential.action_address().to_string();
    assert_eq!(
        final_value["issued_credential"].as_str(),
        Some(expected_action.as_str())
    );

    let credential_value: serde_json::Value = credential
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    assert_eq!(
        credential_value["credentialSubject"]["claims"]["degree"].as_str(),
        Some("DSID requested claim")
    );
    assert_eq!(
        credential_value["credentialSubject"]["claims"]["issuer_note"].as_str(),
        Some("additional attestation")
    );

    let mut agents = BTreeMap::new();
    agents.insert("requester", requester_app.agent().to_string());
    agents.insert("issuer", issuer_app.agent().to_string());
    emit_evidence(
        "DSID-040",
        "request-claims-are-bound-to-issued-credential",
        &dna,
        agents,
        &[&request, &credential, &final_request],
        "An Approved credential request is fulfilled only by a credential whose claims contain every requested claim/value; a rejected mismatch must not leave an orphan credential behind.",
        format!(
            "mismatch_rejected={} no_orphan_after_mismatch={} issued_status={} action_binding={} requested_claim_preserved={} extra_claim_preserved={}",
            mismatch.is_err(),
            issued_after_mismatch.is_empty(),
            final_value["status"],
            final_value["issued_credential"].as_str() == Some(expected_action.as_str()),
            credential_value["credentialSubject"]["claims"]["degree"] == "DSID requested claim",
            credential_value["credentialSubject"]["claims"]["issuer_note"] == "additional attestation"
        ),
        mismatch.is_err()
            && issued_after_mismatch.is_empty()
            && final_value["status"] == "Issued"
            && final_value["issued_credential"].as_str() == Some(expected_action.as_str())
            && credential_value["credentialSubject"]["claims"]["degree"] == "DSID requested claim"
            && credential_value["credentialSubject"]["claims"]["issuer_note"] == "additional attestation",
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_041_w3c_jcs_presentation_is_challenge_bound() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let issuer_app = conductor
        .setup_app("dsid-vp-jcs-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let holder_app = conductor
        .setup_app("dsid-vp-jcs-holder", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let issuer = issuer_app.cells()[0].clone();
    let holder = holder_app.cells()[0].clone();
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());
    let holder_did = format!("did:mycelix:{}", holder_app.agent());

    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&holder.zome("did_registry"), "create_did", ())
        .await;

    let credential: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential",
            serde_json::json!({
                "subject_did": holder_did,
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID VP challenge binding"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID VP Issuer",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false
            }),
        )
        .await;
    let credential_value: serde_json::Value = credential
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let credential_id = credential_value["id"]
        .as_str()
        .expect("credential must expose an ID")
        .to_owned();

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("issuer and holder must reach DHT consistency");

    let challenge = "dsid-041-challenge".to_string();
    let domain = "identity.mycelix.test".to_string();
    let presentation: Record = conductor
        .call(
            &holder.zome("verifiable_credential"),
            "create_presentation",
            serde_json::json!({
                "credential_ids": [credential_id.clone()],
                "challenge": challenge.clone(),
                "domain": domain.clone()
            }),
        )
        .await;

    let presentation_value: serde_json::Value = presentation
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();

    assert_eq!(
        presentation_value["proof"]["cryptosuite"].as_str(),
        Some("eddsa-jcs-2022")
    );
    assert_eq!(
        presentation_value["proof"]["verificationMethod"].as_str(),
        Some(format!("{}#keys-1-multikey", holder_did).as_str())
    );
    assert_eq!(
        presentation_value["proof"]["@context"],
        presentation_value["@context"],
        "JCS presentation proof must carry the presentation @context"
    );
    assert_eq!(
        presentation_value["proof"]["proofPurpose"].as_str(),
        Some("authentication")
    );
    assert_eq!(
        presentation_value["proof"]["challenge"].as_str(),
        Some(challenge.as_str())
    );
    assert_eq!(
        presentation_value["proof"]["domain"].as_str(),
        Some(domain.as_str())
    );

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("presentation must reach verifier peer");

    let verified: serde_json::Value = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "verify_presentation",
            serde_json::json!({
                "presentation_hash": presentation.action_address(),
                "expected_challenge": challenge.clone(),
                "expected_domain": domain.clone()
            }),
        )
        .await;
    assert_eq!(
        verified["valid"], true,
        "W3C JCS presentation must verify with the matching challenge and domain: {verified}"
    );

    let wrong_challenge: serde_json::Value = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "verify_presentation",
            serde_json::json!({
                "presentation_hash": presentation.action_address(),
                "expected_challenge": "wrong-challenge",
                "expected_domain": domain
            }),
        )
        .await;
    assert_eq!(
        wrong_challenge["valid"], false,
        "presentation verification must fail closed for a mismatched challenge"
    );

    let _: Record = conductor
        .call(
            &holder.zome("did_registry"),
            "deactivate_did",
            "DSID holder deactivation qualification",
        )
        .await;

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("holder deactivation must reach the verifier");

    let deactivated_holder: serde_json::Value = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "verify_presentation",
            serde_json::json!({
                "presentation_hash": presentation.action_address(),
                "expected_challenge": challenge.clone(),
                "expected_domain": "identity.mycelix.test"
            }),
        )
        .await;
    assert_eq!(
        deactivated_holder["valid"], false,
        "presentation must fail closed after holder DID deactivation"
    );
    assert!(
        deactivated_holder["errors"]
            .as_array()
            .is_some_and(|errors| errors.iter().any(|error| {
                error.as_str() == Some("Holder DID is not active")
            })),
        "deactivated holder must produce an explicit current-state failure: {deactivated_holder}"
    );

    let mut agents = BTreeMap::new();
    agents.insert("issuer", issuer_app.agent().to_string());
    agents.insert("holder", holder_app.agent().to_string());
    emit_evidence(
        "DSID-041",
        "w3c-jcs-presentation-is-challenge-bound",
        &dna,
        agents,
        &[&credential, &presentation],
        "New verifiable presentations use W3C eddsa-jcs-2022 proofs authorized by the holder's Multikey authentication method, carry the proof context, and fail closed on challenge mismatch.",
        format!(
            "cryptosuite={} proof_context_bound={} authenticated_method={} matching_challenge_valid={} wrong_challenge_invalid={} deactivated_holder_invalid={}",
            presentation_value["proof"]["cryptosuite"].as_str().unwrap_or("missing"),
            presentation_value["proof"]["@context"] == presentation_value["@context"],
            presentation_value["proof"]["verificationMethod"].as_str()
                == Some(format!("{}#keys-1-multikey", holder_did).as_str()),
            verified["valid"] == true,
            wrong_challenge["valid"] == false,
            deactivated_holder["valid"] == false
        ),
        presentation_value["proof"]["cryptosuite"].as_str() == Some("eddsa-jcs-2022")
            && presentation_value["proof"]["@context"] == presentation_value["@context"]
            && presentation_value["proof"]["proofPurpose"].as_str() == Some("authentication")
            && verified["valid"] == true
            && wrong_challenge["valid"] == false
            && deactivated_holder["valid"] == false,
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_042_key_rotation_revokes_stale_presentation_authorization() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let issuer_app = conductor
        .setup_app("dsid-vp-rotation-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let holder_app = conductor
        .setup_app("dsid-vp-rotation-holder", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let issuer = issuer_app.cells()[0].clone();
    let holder = holder_app.cells()[0].clone();
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());
    let holder_did = format!("did:mycelix:{}", holder_app.agent());

    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;
    let created_holder: Record = conductor
        .call(&holder.zome("did_registry"), "create_did", ())
        .await;

    let credential: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential",
            serde_json::json!({
                "subject_did": holder_did.clone(),
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID rotation authorization"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Rotation Issuer",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false
            }),
        )
        .await;

    let credential_value: serde_json::Value = credential
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let credential_id = credential_value["id"]
        .as_str()
        .expect("credential must expose an ID")
        .to_owned();

    let initial: DidDocument = decode_entry(&created_holder).expect("holder DID must decode");
    let old_key_id = initial
        .verification_method
        .iter()
        .find(|method| method.id.ends_with("#keys-1"))
        .map(|method| method.id.clone())
        .expect("initial Ed25519 key must exist");
    let new_key_id = format!("{}#keys-2", holder_did);
    let raw_key = [42u8; 32];
    let new_public_key = TaggedPublicKey::new(AlgorithmId::Ed25519, raw_key.to_vec())
        .expect("synthetic Ed25519 key must have a valid Multikey encoding")
        .to_multibase();

    let presentation: Record = conductor
        .call(
            &holder.zome("verifiable_credential"),
            "create_presentation",
            serde_json::json!({
                "credential_ids": [credential_id.clone()],
                "challenge": "dsid-042-before-rotation"
            }),
        )
        .await;

    let before_rotation: serde_json::Value = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "verify_presentation",
            serde_json::json!({
                "presentation_hash": presentation.action_address(),
                "expected_challenge": "dsid-042-before-rotation"
            }),
        )
        .await;
    assert_eq!(
        before_rotation["valid"], true,
        "presentation must verify before the holder's authorization is rotated"
    );

    let rotated: Record = conductor
        .call(
            &holder.zome("did_registry"),
            "rotate_key",
            serde_json::json!({
                "old_key_id": old_key_id,
                "new_method": {
                    "id": new_key_id.clone(),
                    "type": "Ed25519VerificationKey2020",
                    "controller": holder_did,
                    "publicKeyMultibase": new_public_key,
                    "algorithm": 0xed01
                }
            }),
        )
        .await;

    let rotated_doc: DidDocument = decode_entry(&rotated).expect("rotated DID must decode");
    assert!(!rotated_doc
        .authentication
        .iter()
        .any(|method| method == &format!("{}#keys-1-multikey", format!("did:mycelix:{}", holder_app.agent()))),
        "rotation must remove the old canonical Multikey from authentication"
    );

    await_consistency(&[issuer.clone(), holder.clone()])
        .await
        .expect("credential and DID rotation must reach DHT consistency");

    let stale_existing: serde_json::Value = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "verify_presentation",
            serde_json::json!({
                "presentation_hash": presentation.action_address(),
                "expected_challenge": "dsid-042-before-rotation"
            }),
        )
        .await;
    assert_eq!(
        stale_existing["valid"], false,
        "an already-created presentation must fail current-state verification after authorization rotation"
    );

    let stale_presentation: Result<Record, _> = conductor
        .call_fallible(
            &holder.zome("verifiable_credential"),
            "create_presentation",
            serde_json::json!({
                "credential_ids": [credential_id],
                "challenge": "dsid-042-challenge"
            }),
        )
        .await;
    assert!(
        stale_presentation.is_err(),
        "presentation creation must fail once the canonical JCS Multikey is no longer authorized"
    );

    let mut agents = BTreeMap::new();
    agents.insert("issuer", issuer_app.agent().to_string());
    agents.insert("holder", holder_app.agent().to_string());
    emit_evidence(
        "DSID-042",
        "key-rotation-revokes-stale-presentation-authorization",
        &dna,
        agents,
        &[&credential, &created_holder, &rotated, &presentation],
        "After key rotation removes the historical canonical Multikey from DID authentication, the holder must not mint new JCS presentations using that stale verification method.",
        format!(
            "old_multikey_auth_removed={} pre_rotation_valid={} existing_presentation_invalidated={} stale_creation_rejected={}",
            !rotated_doc.authentication.iter().any(|method| {
                method == &format!("{}#keys-1-multikey", holder_app.agent())
            }),
            before_rotation["valid"] == true,
            stale_existing["valid"] == false,
            stale_presentation.is_err()
        ),
        !rotated_doc.authentication.iter().any(|method| {
            method == &format!("{}#keys-1-multikey", holder_app.agent())
        })
            && before_rotation["valid"] == true
            && stale_existing["valid"] == false
            && stale_presentation.is_err(),
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore]
async fn dsid_043_request_bound_issuance_is_deterministically_idempotent() {
    let mut conductor = SweetConductor::from_standard_config().await;
    let dna = load_dna().await;
    let requester_app = conductor
        .setup_app("dsid-vc-idempotent-requester", std::slice::from_ref(&dna))
        .await
        .unwrap();
    let issuer_app = conductor
        .setup_app("dsid-vc-idempotent-issuer", std::slice::from_ref(&dna))
        .await
        .unwrap();

    let requester = requester_app.cells()[0].clone();
    let issuer = issuer_app.cells()[0].clone();
    let requester_did = format!("did:mycelix:{}", requester_app.agent());
    let issuer_did = format!("did:mycelix:{}", issuer_app.agent());

    let _: Record = conductor
        .call(&requester.zome("did_registry"), "create_did", ())
        .await;
    let _: Record = conductor
        .call(&issuer.zome("did_registry"), "create_did", ())
        .await;

    let request: Record = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "request_credential",
            serde_json::json!({
                "issuer_did": issuer_did.clone(),
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID deterministic fulfillment"
                },
                "evidence": []
            }),
        )
        .await;

    let request_value: serde_json::Value = request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();
    let request_id = request_value["id"]
        .as_str()
        .expect("request ID must exist")
        .to_owned();

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("request must reach the issuer");

    let _: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id.clone(),
                "new_status": "UnderReview"
            }),
        )
        .await;
    let _: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "update_request_status",
            serde_json::json!({
                "request_id": request_id.clone(),
                "new_status": "Approved"
            }),
        )
        .await;

    let mut material = Vec::new();
    material.extend_from_slice(issuer_did.as_bytes());
    material.push(0);
    material.extend_from_slice(request_id.as_bytes());
    let deterministic_id = format!(
        "urn:mycelix:request-credential:{}",
        bs58::encode(holo_hash::blake2b_256(&material))
            .with_alphabet(bs58::Alphabet::BITCOIN)
            .into_string()
    );

    let preexisting: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential",
            serde_json::json!({
                "subject_did": requester_did.clone(),
                "schema_id": "mycelix:schema:education:degree:v1",
                "claims": {
                    "degree": "DSID deterministic fulfillment"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "DSID Idempotency Issuer",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false,
                "credential_id": deterministic_id.clone(),
                "proof_profile": "W3cEddsaJcs2022"
            }),
        )
        .await;

    let fulfilled: Record = conductor
        .call(
            &issuer.zome("verifiable_credential"),
            "issue_credential_for_request",
            serde_json::json!({
                "request_id": request_id.clone(),
                "claims": {
                    "degree": "DSID deterministic fulfillment"
                },
                "credential_types": ["QualificationCredential"],
                "issuer_name": "Ignored because deterministic credential already exists",
                "expiration_days": 365,
                "enable_revocation": false,
                "strict_schema": false
            }),
        )
        .await;

    assert_eq!(
        fulfilled.action_address(),
        preexisting.action_address(),
        "request-bound issuance must reuse the deterministic pre-existing credential"
    );

    await_consistency(&[requester.clone(), issuer.clone()])
        .await
        .expect("idempotent fulfillment must reach requester");

    let final_request: Option<Record> = conductor
        .call(
            &requester.zome("verifiable_credential"),
            "get_credential_request",
            serde_json::json!({
                "issuer_did": issuer_did,
                "request_id": request_id
            }),
        )
        .await;
    let final_request = final_request.expect("fulfilled request must resolve");
    let final_value: serde_json::Value = final_request
        .entry()
        .to_app_option()
        .unwrap()
        .unwrap();

    assert_eq!(final_value["status"], "Issued");
    assert_eq!(
        final_value["issued_credential"].as_str(),
        Some(preexisting.action_address().to_string().as_str())
    );

    let mut agents = BTreeMap::new();
    agents.insert("requester", requester_app.agent().to_string());
    agents.insert("issuer", issuer_app.agent().to_string());
    emit_evidence(
        "DSID-043",
        "request-bound-issuance-is-deterministically-idempotent",
        &dna,
        agents,
        &[&request, &preexisting, &fulfilled, &final_request],
        "Request-bound issuance derives one deterministic credential ID from issuer and request ID; an existing valid fulfillment is reused and then cryptographically bound into Issued state instead of minting a second credential.",
        format!(
            "deterministic_id_present={} reused_action={} issued_status={} issued_pointer_matches={}",
            deterministic_id.starts_with("urn:mycelix:request-credential:"),
            fulfilled.action_address() == preexisting.action_address(),
            final_value["status"],
            final_value["issued_credential"].as_str()
                == Some(preexisting.action_address().to_string().as_str())
        ),
        deterministic_id.starts_with("urn:mycelix:request-credential:")
            && fulfilled.action_address() == preexisting.action_address()
            && final_value["status"] == "Issued"
            && final_value["issued_credential"].as_str()
                == Some(preexisting.action_address().to_string().as_str()),
    );
}

