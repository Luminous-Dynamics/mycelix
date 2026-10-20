// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use praxis_core::cyber_challenges::{
    CYBER_CHALLENGE_CORPUS_ID, ChallengeDifficulty, CyberDomain, EvidenceConflictStatus,
    EvidenceFreshness, EvidenceSourceKind, SolverVisibleCyberCorpus, validate_solver_corpus,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const SOLVER_FIXTURE: &str =
    include_str!("../../../examples/cyber-challenges/v1/solver-visible.json");
const EVALUATOR_FIXTURE: &str =
    include_str!("../../../examples/cyber-challenges/v1/evaluator-only.json");
const SOLVER_SCHEMA: &str =
    include_str!("../../../examples/cyber-challenges/v1/solver-visible.schema.json");
const EVALUATOR_SCHEMA: &str =
    include_str!("../../../examples/cyber-challenges/v1/evaluator-only.schema.json");
const CORPUS_MANIFEST: &str =
    include_str!("../../../examples/cyber-challenges/v1/corpus-manifest.json");

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorCorpus {
    schema_version: u16,
    corpus_id: String,
    corpus_version: String,
    disclosure: String,
    cases: Vec<EvaluatorCase>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EvaluatorCase {
    scenario_id: String,
    expected_disposition: ExpectedDisposition,
    required_evidence_ids: Vec<String>,
    additional_evidence_needed: Vec<String>,
    acceptable_inferences: Vec<String>,
    prohibited_claims: Vec<String>,
    safe_next_steps: Vec<String>,
    authority_gates: Vec<String>,
    rubric: Vec<RubricCriterion>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ExpectedDisposition {
    NeedsMoreEvidence,
    ConfigurationRisk,
    ProvenanceConflict,
    UntrustedInstruction,
    StaleStateUnknown,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RubricCriterion {
    criterion_id: String,
    max_score_permille: u16,
    description: String,
}

fn serialized_enum_names<T: Serialize>(values: impl IntoIterator<Item = T>) -> BTreeSet<String> {
    values
        .into_iter()
        .map(|value| {
            serde_json::to_value(value)
                .expect("enum serializes")
                .as_str()
                .expect("enum serializes as a string")
                .to_string()
        })
        .collect()
}

fn schema_enum_names(schema: &serde_json::Value, name: &str) -> BTreeSet<String> {
    schema["$defs"][name]["enum"]
        .as_array()
        .expect("enum list")
        .iter()
        .map(|value| value.as_str().expect("string enum value").to_string())
        .collect()
}

fn assert_schema_fields_match(schema_properties: &serde_json::Value, instance: &serde_json::Value) {
    let schema_fields = schema_properties
        .as_object()
        .expect("schema properties")
        .keys()
        .cloned()
        .collect::<BTreeSet<_>>();
    let instance_fields = instance
        .as_object()
        .expect("serialized instance object")
        .keys()
        .cloned()
        .collect::<BTreeSet<_>>();
    assert_eq!(
        schema_fields, instance_fields,
        "Rust/JSON Schema field drift"
    );
}

fn solver_fixture() -> SolverVisibleCyberCorpus {
    serde_json::from_str(SOLVER_FIXTURE).expect("solver fixture parses")
}

fn evaluator_fixture() -> EvaluatorCorpus {
    serde_json::from_str(EVALUATOR_FIXTURE).expect("evaluator fixture parses")
}

fn evaluator_issues(solver: &SolverVisibleCyberCorpus, evaluator: &EvaluatorCorpus) -> Vec<String> {
    let mut issues = Vec::new();
    if evaluator.schema_version != solver.schema_version {
        issues.push("schema version mismatch".to_string());
    }
    if evaluator.corpus_id != solver.corpus_id {
        issues.push("corpus ID mismatch".to_string());
    }
    if evaluator.corpus_version != solver.corpus_version {
        issues.push("corpus version mismatch".to_string());
    }
    if evaluator.disclosure.trim().is_empty() {
        issues.push("evaluator disclosure is required".to_string());
    }

    let solver_ids = solver
        .scenarios
        .iter()
        .map(|scenario| scenario.scenario_id.as_str())
        .collect::<BTreeSet<_>>();
    let evaluator_ids = evaluator
        .cases
        .iter()
        .map(|case| case.scenario_id.as_str())
        .collect::<BTreeSet<_>>();

    if evaluator.cases.len() != evaluator_ids.len() {
        issues.push("duplicate evaluator scenario IDs".to_string());
    }
    if solver_ids != evaluator_ids {
        issues.push("solver/evaluator scenario IDs do not match exactly".to_string());
    }

    let solver_by_id = solver
        .scenarios
        .iter()
        .map(|scenario| (scenario.scenario_id.as_str(), scenario))
        .collect::<BTreeMap<_, _>>();

    for (index, case) in evaluator.cases.iter().enumerate() {
        let path = format!("$.cases[{index}]");
        let Some(scenario) = solver_by_id.get(case.scenario_id.as_str()) else {
            issues.push(format!("{path}: case references unknown scenario"));
            continue;
        };
        let available_evidence = scenario
            .evidence
            .iter()
            .map(|evidence| evidence.evidence_id.as_str())
            .collect::<BTreeSet<_>>();
        for evidence_id in &case.required_evidence_ids {
            if !available_evidence.contains(evidence_id.as_str()) {
                issues.push(format!(
                    "{path}: required evidence {:?} is not solver-visible",
                    evidence_id
                ));
            }
        }
        if case.required_evidence_ids.is_empty() {
            issues.push(format!("{path}: required_evidence_ids must not be empty"));
        }
        for (field, items) in [
            (
                "additional_evidence_needed",
                case.additional_evidence_needed.as_slice(),
            ),
            (
                "acceptable_inferences",
                case.acceptable_inferences.as_slice(),
            ),
            ("prohibited_claims", case.prohibited_claims.as_slice()),
            ("safe_next_steps", case.safe_next_steps.as_slice()),
            ("authority_gates", case.authority_gates.as_slice()),
        ] {
            if items.is_empty() || items.iter().any(|item| item.trim().is_empty()) {
                issues.push(format!(
                    "{path}.{field}: non-empty descriptions are required"
                ));
            }
        }

        let criterion_ids = case
            .rubric
            .iter()
            .map(|criterion| criterion.criterion_id.as_str())
            .collect::<BTreeSet<_>>();
        if case.rubric.is_empty() || criterion_ids.len() != case.rubric.len() {
            issues.push(format!("{path}.rubric: rubric must have unique criteria"));
        }
        if case.rubric.iter().any(|criterion| {
            criterion.description.trim().is_empty() || criterion.max_score_permille == 0
        }) {
            issues.push(format!(
                "{path}.rubric: criterion descriptions and positive weights are required"
            ));
        }
        let total = case
            .rubric
            .iter()
            .map(|criterion| u32::from(criterion.max_score_permille))
            .sum::<u32>();
        if total != 1000 {
            issues.push(format!(
                "{path}.rubric: weights must sum to 1000, got {total}"
            ));
        }
        let _ = case.expected_disposition;
    }

    issues
}

fn has_code(
    issues: &[praxis_core::cyber_challenges::CyberCorpusValidationIssue],
    code: &str,
) -> bool {
    issues.iter().any(|issue| issue.code == code)
}

#[test]
fn synthetic_solver_corpus_passes_structural_validation() {
    let corpus = solver_fixture();
    let summary = validate_solver_corpus(&corpus).expect("solver corpus should be valid");
    assert_eq!(summary.corpus_id, CYBER_CHALLENGE_CORPUS_ID);
    assert_eq!(summary.corpus_version, "1.0.0");
    assert_eq!(summary.scenario_count, 5);
    assert_eq!(summary.evidence_count, 13);
}

#[test]
fn evaluator_is_separate_and_exactly_matches_solver_cases() {
    let solver = solver_fixture();
    let evaluator = evaluator_fixture();
    assert!(evaluator_issues(&solver, &evaluator).is_empty());

    let solver_text = SOLVER_FIXTURE.to_ascii_lowercase();
    for evaluator_only_field in [
        "expected_disposition",
        "acceptable_inferences",
        "prohibited_claims",
        "max_score_permille",
        "authority_gates",
    ] {
        assert!(
            !solver_text.contains(evaluator_only_field),
            "solver-visible bytes contain evaluator-only field {evaluator_only_field}"
        );
    }
}

#[test]
fn evaluator_schema_rejects_non_1000_rubric_weights() {
    let mut evaluator = evaluator_fixture();
    evaluator.cases[0].rubric[0].max_score_permille -= 1;
    assert!(
        evaluator_issues(&solver_fixture(), &evaluator)
            .iter()
            .any(|issue| issue.contains("weights must sum to 1000"))
    );
}

#[test]
fn evaluator_cannot_reference_unavailable_evidence() {
    let mut evaluator = evaluator_fixture();
    evaluator.cases[0]
        .required_evidence_ids
        .push("secret-evidence-not-in-case".into());
    assert!(
        evaluator_issues(&solver_fixture(), &evaluator)
            .iter()
            .any(|issue| issue.contains("not solver-visible"))
    );
}

#[test]
fn solver_contract_rejects_a_duplicate_scenario_id() {
    let mut corpus = solver_fixture();
    corpus.scenarios[1].scenario_id = corpus.scenarios[0].scenario_id.clone();
    let issues = validate_solver_corpus(&corpus).unwrap_err();
    assert!(has_code(&issues, "duplicate_scenario_id"));
}

#[test]
fn solver_contract_rejects_non_synthetic_cases() {
    let mut corpus = solver_fixture();
    corpus.scenarios[0].is_synthetic = false;
    let issues = validate_solver_corpus(&corpus).unwrap_err();
    assert!(has_code(&issues, "non_synthetic_scenario"));
}

#[test]
fn solver_contract_rejects_duplicate_evidence_ids_within_a_case() {
    let mut corpus = solver_fixture();
    corpus.scenarios[0].evidence[1].evidence_id =
        corpus.scenarios[0].evidence[0].evidence_id.clone();
    let issues = validate_solver_corpus(&corpus).unwrap_err();
    assert!(has_code(&issues, "duplicate_evidence_id"));
}

#[test]
fn source_snapshot_binding_is_required() {
    let mut corpus = solver_fixture();
    corpus.scenarios[0].evidence[0]
        .source_snapshot_version
        .clear();
    let issues = validate_solver_corpus(&corpus).unwrap_err();
    assert!(has_code(&issues, "empty_required_text"));
}

#[test]
fn solver_projection_refuses_evaluator_fields() {
    let mut value: serde_json::Value =
        serde_json::from_str(SOLVER_FIXTURE).expect("solver JSON parses");
    value["scenarios"][0]["expected_disposition"] =
        serde_json::Value::String("needs_more_evidence".into());
    assert!(
        serde_json::from_value::<SolverVisibleCyberCorpus>(value).is_err(),
        "deny_unknown_fields must fail closed on evaluator-only fields"
    );
}

#[test]
fn solver_schema_matches_rust_fields_and_enum_values() {
    let schema: serde_json::Value =
        serde_json::from_str(SOLVER_SCHEMA).expect("solver schema parses");
    let corpus = solver_fixture();
    let serialized = serde_json::to_value(&corpus).expect("Rust corpus serializes");
    assert_schema_fields_match(&schema["properties"], &serialized);
    assert_schema_fields_match(
        &schema["$defs"]["Scenario"]["properties"],
        &serialized["scenarios"][0],
    );
    assert_schema_fields_match(
        &schema["$defs"]["Evidence"]["properties"],
        &serialized["scenarios"][0]["evidence"][0],
    );

    assert_eq!(
        schema_enum_names(&schema, "Domain"),
        serialized_enum_names([
            CyberDomain::IdentityAccess,
            CyberDomain::NetworkConfiguration,
            CyberDomain::SupplyChainProvenance,
            CyberDomain::AiToolBoundary,
            CyberDomain::EndpointTelemetry,
        ])
    );
    assert_eq!(
        schema_enum_names(&schema, "Difficulty"),
        serialized_enum_names([
            ChallengeDifficulty::Beginner,
            ChallengeDifficulty::Intermediate,
            ChallengeDifficulty::Advanced,
        ])
    );
    assert_eq!(
        schema_enum_names(&schema, "Freshness"),
        serialized_enum_names([
            EvidenceFreshness::Current,
            EvidenceFreshness::Stale,
            EvidenceFreshness::Unknown,
        ])
    );
    assert_eq!(
        schema_enum_names(&schema, "EvidenceSource"),
        serialized_enum_names([
            EvidenceSourceKind::IdentityLogSummary,
            EvidenceSourceKind::NetworkContext,
            EvidenceSourceKind::EndpointTelemetry,
            EvidenceSourceKind::NetworkPolicySnapshot,
            EvidenceSourceKind::FirewallPolicySnapshot,
            EvidenceSourceKind::DependencyLockfile,
            EvidenceSourceKind::ApprovedDependencyRecord,
            EvidenceSourceKind::RegistryMetadata,
            EvidenceSourceKind::RetrievedDocumentExcerpt,
            EvidenceSourceKind::ToolPermissionSnapshot,
            EvidenceSourceKind::ToolAuditSummary,
            EvidenceSourceKind::ResponseRequestLog,
        ])
    );
    assert_eq!(
        schema_enum_names(&schema, "EvidenceConflictStatus"),
        serialized_enum_names([
            EvidenceConflictStatus::NoKnownConflict,
            EvidenceConflictStatus::ConflictsWithOtherEvidence,
            EvidenceConflictStatus::NotAssessed,
        ])
    );

    let evaluator_schema: serde_json::Value =
        serde_json::from_str(EVALUATOR_SCHEMA).expect("evaluator schema parses");
    let evaluator = evaluator_fixture();
    let evaluator_json = serde_json::to_value(&evaluator).expect("evaluator serializes");
    assert_schema_fields_match(&evaluator_schema["properties"], &evaluator_json);
    assert_schema_fields_match(
        &evaluator_schema["$defs"]["Case"]["properties"],
        &evaluator_json["cases"][0],
    );
    assert_schema_fields_match(
        &evaluator_schema["$defs"]["Rubric"]["properties"],
        &evaluator_json["cases"][0]["rubric"][0],
    );
    assert_eq!(
        schema_enum_names(&evaluator_schema, "Disposition"),
        serialized_enum_names([
            ExpectedDisposition::NeedsMoreEvidence,
            ExpectedDisposition::ConfigurationRisk,
            ExpectedDisposition::ProvenanceConflict,
            ExpectedDisposition::UntrustedInstruction,
            ExpectedDisposition::StaleStateUnknown,
        ])
    );
}

#[test]
fn schemas_and_manifest_are_versioned_and_separate() {
    let solver_schema: serde_json::Value =
        serde_json::from_str(SOLVER_SCHEMA).expect("solver schema parses");
    let evaluator_schema: serde_json::Value =
        serde_json::from_str(EVALUATOR_SCHEMA).expect("evaluator schema parses");
    let manifest: serde_json::Value =
        serde_json::from_str(CORPUS_MANIFEST).expect("corpus manifest parses");

    assert_eq!(
        solver_schema["$schema"],
        "https://json-schema.org/draft/2020-12/schema"
    );
    assert_eq!(
        evaluator_schema["$schema"],
        "https://json-schema.org/draft/2020-12/schema"
    );
    assert_eq!(
        solver_schema["properties"]["corpus_id"]["const"],
        CYBER_CHALLENGE_CORPUS_ID
    );
    assert_eq!(
        evaluator_schema["properties"]["corpus_id"]["const"],
        CYBER_CHALLENGE_CORPUS_ID
    );
    assert_eq!(manifest["corpus_id"], CYBER_CHALLENGE_CORPUS_ID);
    assert_eq!(manifest["corpus_version"], "1.0.0");
    assert_eq!(manifest["solver_visible_file"], "solver-visible.json");
    assert_eq!(manifest["evaluator_only_file"], "evaluator-only.json");
    assert_ne!(
        manifest["solver_visible_file"],
        manifest["evaluator_only_file"]
    );

    let solver_properties = solver_schema["$defs"]["Scenario"]["properties"]
        .as_object()
        .expect("solver scenario properties");
    let evidence_required = solver_schema["$defs"]["Evidence"]["required"]
        .as_array()
        .expect("evidence schema required list");
    assert!(evidence_required.contains(&serde_json::Value::String("source_snapshot_id".into())));
    assert!(
        evidence_required.contains(&serde_json::Value::String("source_snapshot_version".into()))
    );
    assert!(!solver_properties.contains_key("expected_disposition"));
    assert!(!solver_properties.contains_key("rubric"));
    assert!(!solver_properties.contains_key("prohibited_claims"));

    for key in [
        "solver_visible_git_blob_sha",
        "evaluator_only_git_blob_sha",
        "solver_schema_git_blob_sha",
        "evaluator_schema_git_blob_sha",
    ] {
        let value = manifest[key].as_str().expect("manifest blob identity");
        assert_eq!(value.len(), 40, "{key} must be a Git SHA-1 blob identifier");
        assert!(
            value
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase()),
            "{key} must be lowercase hexadecimal"
        );
    }
    assert!(
        manifest["git_blob_sha_note"]
            .as_str()
            .expect("blob identity caveat")
            .contains("not signatures")
    );

    let solver = solver_fixture();
    let domains = solver
        .scenarios
        .iter()
        .map(|scenario| scenario.domain)
        .collect::<BTreeSet<_>>();
    assert_eq!(domains.len(), 5);
    let difficulties = solver
        .scenarios
        .iter()
        .map(|scenario| scenario.difficulty)
        .collect::<BTreeSet<_>>();
    assert!(difficulties.contains(&ChallengeDifficulty::Beginner));
    assert!(difficulties.contains(&ChallengeDifficulty::Intermediate));
    assert!(difficulties.contains(&ChallengeDifficulty::Advanced));

    let freshness = solver
        .scenarios
        .iter()
        .flat_map(|s| s.evidence.iter().map(|e| e.freshness))
        .collect::<BTreeSet<_>>();
    assert!(freshness.contains(&EvidenceFreshness::Stale));
    assert!(freshness.contains(&EvidenceFreshness::Current));

    let conflict_status = solver
        .scenarios
        .iter()
        .flat_map(|s| s.evidence.iter().map(|e| e.conflict_status))
        .collect::<BTreeSet<_>>();
    assert!(conflict_status.contains(&EvidenceConflictStatus::ConflictsWithOtherEvidence));
    assert!(conflict_status.contains(&EvidenceConflictStatus::NoKnownConflict));
    assert!(conflict_status.contains(&EvidenceConflictStatus::NotAssessed));

    let _source_coverage = solver
        .scenarios
        .iter()
        .flat_map(|s| s.evidence.iter().map(|e| e.source_kind))
        .collect::<BTreeSet<EvidenceSourceKind>>();
    assert_eq!(solver.scenarios[0].domain, CyberDomain::IdentityAccess);
}
