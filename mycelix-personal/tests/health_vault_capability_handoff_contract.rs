// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Machine-checkable contract tests for the synthetic Health Vault capability-handoff corpus.
//!
//! These tests validate fixture shape, invariants, schema ID coverage, and mirror parity.
//! They do NOT validate cryptography, Holochain runtime authorization, or conductor behavior.

use serde::Deserialize;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureCorpus {
    schema_version: String,
    status: String,
    claim_ceiling: String,
    invariants: Vec<String>,
    fixtures: Vec<Fixture>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    id: String,
    title: String,
    operation: String,
    category: String,
    setup: Vec<String>,
    expected: Expected,
    measurements: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Expected {
    protocol_result: ProtocolResult,
    failure_code: Option<String>,
    effective_claim_delta: u8,
    protected_payload_bytes_returned: u64,
}

#[derive(Debug, Clone, Copy, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum ProtocolResult {
    Success,
    Denied,
    Unavailable,
    Retryable,
}

const CORPUS_REL: &str = "docs/research/fixtures/HEALTH_VAULT_CAPABILITY_HANDOFF_V1.json";
const SCHEMA_REL: &str = "docs/research/fixtures/HEALTH_VAULT_CAPABILITY_HANDOFF_V1.schema.json";
const DESIGN_REL: &str = "docs/research/HEALTH_VAULT_CAPABILITY_HANDOFF_V0.md";

fn repository_root() -> PathBuf {
    let start = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for candidate in start.ancestors() {
        if candidate.join(CORPUS_REL).is_file()
            && candidate.join("mycelix-workspace").join(CORPUS_REL).is_file()
            && candidate.join(DESIGN_REL).is_file()
        {
            return candidate.to_path_buf();
        }
    }
    panic!(
        "could not locate repository root containing both Health Vault corpus projections          (manifest dir: {})",
        start.display()
    );
}

fn read_repo_file(root: &Path, relative: &str) -> String {
    fs::read_to_string(root.join(relative))
        .unwrap_or_else(|error| panic!("could not read {}: {error}", root.join(relative).display()))
}

fn load_corpus(root: &Path, relative: &str) -> FixtureCorpus {
    let text = read_repo_file(root, relative);
    serde_json::from_str(&text)
        .unwrap_or_else(|error| panic!("invalid corpus JSON at {relative}: {error}"))
}

fn expected_ids() -> Vec<String> {
    (1..=26).map(|n| format!("HCH-{n:03}")).collect()
}

#[test]
fn corpus_has_exact_stable_ids_and_typed_fail_closed_results() {
    let root = repository_root();
    let corpus = load_corpus(&root, CORPUS_REL);
    let ids: Vec<String> = corpus.fixtures.iter().map(|fixture| fixture.id.clone()).collect();

    assert_eq!(
        corpus.schema_version,
        "mycelix.health-vault.capability-handoff-fixtures.v1"
    );
    assert_eq!(corpus.status, "synthetic_fixture_contract");
    assert!(!corpus.claim_ceiling.trim().is_empty());
    assert!(corpus.invariants.len() >= 5);
    assert!(corpus.invariants.iter().all(|invariant| !invariant.trim().is_empty()));
    assert_eq!(corpus.fixtures.len(), 26);
    assert_eq!(ids, expected_ids(), "fixture IDs must be stable, ordered and unique");

    for fixture in &corpus.fixtures {
        assert!(!fixture.title.trim().is_empty(), "{} has an empty title", fixture.id);
        assert!(!fixture.operation.trim().is_empty(), "{} has an empty operation", fixture.id);
        assert!(!fixture.category.trim().is_empty(), "{} has an empty category", fixture.id);
        assert!(!fixture.setup.is_empty(), "{} has no setup conditions", fixture.id);
        assert!(fixture.setup.iter().all(|item| !item.trim().is_empty()));
        assert!(fixture.measurements.iter().all(|item| !item.trim().is_empty()));
        assert_eq!(
            fixture.expected.protected_payload_bytes_returned,
            0,
            "{} must never return protected health bytes in the handoff-contract boundary",
            fixture.id
        );
        assert!(
            fixture.expected.effective_claim_delta <= 1,
            "{} must not create multiple effective claims",
            fixture.id
        );

        match fixture.expected.protocol_result {
            ProtocolResult::Success => assert!(
                fixture.expected.failure_code.is_none(),
                "{} success should not carry a failure code",
                fixture.id
            ),
            ProtocolResult::Denied | ProtocolResult::Unavailable | ProtocolResult::Retryable => {
                assert!(
                    fixture.expected.failure_code.as_deref().is_some_and(|code| !code.is_empty()),
                    "{} non-success must use a typed failure code",
                    fixture.id
                );
            }
        }
    }

    let claim_increases: Vec<&str> = corpus
        .fixtures
        .iter()
        .filter(|fixture| fixture.expected.effective_claim_delta == 1)
        .map(|fixture| fixture.id.as_str())
        .collect();
    assert_eq!(claim_increases, vec!["HCH-001"]);

    let duplicate_delivery = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-009").unwrap();
    let lost_ack_retry = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-024").unwrap();
    for fixture in [duplicate_delivery, lost_ack_retry] {
        assert_eq!(fixture.expected.protocol_result, ProtocolResult::Success);
        assert_eq!(fixture.expected.effective_claim_delta, 0);
    }

    let unavailable = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-016").unwrap();
    assert_eq!(unavailable.expected.protocol_result, ProtocolResult::Unavailable);
    assert_eq!(unavailable.expected.failure_code.as_deref(), Some("source_unavailable"));

    let leak_positive = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-018").unwrap();
    assert_eq!(leak_positive.expected.protocol_result, ProtocolResult::Success);
    let leak_mutation = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-025").unwrap();
    assert_eq!(leak_mutation.expected.protocol_result, ProtocolResult::Denied);
    assert_eq!(leak_mutation.expected.failure_code.as_deref(), Some("secret_leak_detected"));

    let generic_key = corpus.fixtures.iter().find(|fixture| fixture.id == "HCH-026").unwrap();
    assert_eq!(generic_key.expected.protocol_result, ProtocolResult::Denied);
    assert_eq!(generic_key.expected.failure_code.as_deref(), Some("recipient_key_binding_unqualified"));
}

#[test]
fn json_schema_requires_each_stable_fixture_id_exactly_once() {
    let root = repository_root();
    let corpus = load_corpus(&root, CORPUS_REL);
    let schema_text = read_repo_file(&root, SCHEMA_REL);
    let schema: serde_json::Value = serde_json::from_str(&schema_text)
        .unwrap_or_else(|error| panic!("invalid JSON Schema JSON: {error}"));

    assert_eq!(
        schema["$schema"].as_str(),
        Some("https://json-schema.org/draft/2020-12/schema")
    );
    assert_eq!(
        schema["$id"].as_str(),
        Some("urn:mycelix:schema:health-vault-capability-handoff-v1")
    );
    assert_eq!(
        schema["properties"]["status"]["const"].as_str(),
        Some("synthetic_fixture_contract")
    );
    assert_eq!(schema["properties"]["fixtures"]["minItems"].as_u64(), Some(25));
    assert_eq!(schema["properties"]["fixtures"]["maxItems"].as_u64(), Some(25));

    let rules = schema["properties"]["fixtures"]["allOf"]
        .as_array()
        .expect("fixture array must carry exact-ID constraints");
    assert_eq!(rules.len(), 26);
    let expected = expected_ids();
    let schema_ids: Vec<String> = rules
        .iter()
        .map(|rule| {
            assert_eq!(rule["minContains"].as_u64(), Some(1));
            assert_eq!(rule["maxContains"].as_u64(), Some(1));
            rule["contains"]["properties"]["id"]["const"]
                .as_str()
                .expect("every ID rule must contain a const")
                .to_string()
        })
        .collect();
    assert_eq!(schema_ids, expected);
    assert_eq!(
        corpus.fixtures.iter().map(|fixture| fixture.id.clone()).collect::<Vec<_>>(),
        expected
    );
}

#[test]
fn root_and_workspace_health_vault_contract_projections_are_byte_identical() {
    let root = repository_root();
    for relative in [
        DESIGN_REL,
        CORPUS_REL,
        SCHEMA_REL,
        "mycelix-personal/tests/Cargo.toml",
        "mycelix-personal/tests/health_vault_capability_handoff_contract.rs",
    ] {
        let workspace_relative = format!("mycelix-workspace/{relative}");
        let canonical = read_repo_file(&root, relative);
        let workspace = read_repo_file(&root, &workspace_relative);
        assert_eq!(canonical, workspace, "mirror drift at {relative}");
    }
}
