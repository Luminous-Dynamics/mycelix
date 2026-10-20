use std::{
    collections::{BTreeMap, BTreeSet},
    env, fs,
    path::PathBuf,
    process::ExitCode,
};

use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha2::{Digest, Sha256};

const EXPECTED_CONTRACT_ID: &str = "CIVOS-MACQ-001";
const EXPECTED_VERSION: &str = "0.1.0";
const EXPECTED_STATUS: &str = "proposed-unmeasured";

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Corpus {
    contract_id: String,
    version: String,
    status: String,
    source_baseline: SourceBaseline,
    fixture_policy: FixturePolicy,
    scenarios: Vec<Scenario>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SourceBaseline {
    repository: String,
    commit: String,
    purpose: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct FixturePolicy {
    data_class: String,
    wall_clock: String,
    randomness: String,
    unknown_applicability: String,
    global_score: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    id: String,
    title: String,
    targets: Vec<String>,
    #[serde(rename = "class")]
    scenario_class: ScenarioClass,
    setup: Vec<String>,
    actions: Vec<String>,
    expected: Vec<String>,
    negative_controls: Vec<String>,
    observables: Vec<String>,
    requires_capability: Option<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
enum ScenarioClass {
    HardGate,
    Behavioral,
    PerformanceBaseline,
}

impl ScenarioClass {
    fn as_str(self) -> &'static str {
        match self {
            Self::HardGate => "hard_gate",
            Self::Behavioral => "behavioral",
            Self::PerformanceBaseline => "performance_baseline",
        }
    }
}

#[derive(Debug, Serialize)]
struct QualificationReceipt {
    receipt_contract: &'static str,
    receipt_version: &'static str,
    corpus_sha256: String,
    schema_sha256: String,
    validator_source_sha256: String,
    validator_manifest_sha256: String,
    dependency_lock_sha256: String,
    corpus_contract_id: String,
    corpus_version: String,
    corpus_status: String,
    source_baseline: SourceBaseline,
    scenario_count: usize,
    scenario_class_counts: BTreeMap<String, usize>,
    target_counts: BTreeMap<String, usize>,
    scenario_ids: Vec<String>,
    corpus_validation: &'static str,
    scenario_execution: &'static str,
    benchmark_result: Option<String>,
    claim_boundary: &'static str,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("CIVOS-MACQ validation failed: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let corpus_path = PathBuf::from(args.next().ok_or_else(usage)?);
    let schema_path = PathBuf::from(args.next().ok_or_else(usage)?);
    let mut receipt_path: Option<PathBuf> = None;
    let mut expected_corpus_sha256: Option<String> = None;

    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--receipt" => {
                receipt_path = Some(PathBuf::from(
                    args.next()
                        .ok_or("--receipt requires a file path".to_string())?,
                ));
            }
            "--expected-corpus-sha256" => {
                expected_corpus_sha256 = Some(args.next().ok_or(
                    "--expected-corpus-sha256 requires a 64-character digest".to_string(),
                )?);
            }
            other => return Err(format!("unknown argument {other:?}\n{}", usage())),
        }
    }

    let corpus_bytes = fs::read(&corpus_path)
        .map_err(|error| format!("cannot read corpus {}: {error}", corpus_path.display()))?;
    let schema_bytes = fs::read(&schema_path)
        .map_err(|error| format!("cannot read schema {}: {error}", schema_path.display()))?;
    let corpus_sha256 = sha256_hex(&corpus_bytes);
    let schema_sha256 = sha256_hex(&schema_bytes);

    if let Some(expected) = expected_corpus_sha256 {
        if !is_lower_hex_digest(&expected) {
            return Err(
                "--expected-corpus-sha256 must be 64 lowercase hexadecimal characters".into(),
            );
        }
        if expected != corpus_sha256 {
            return Err(format!(
                "corpus digest mismatch: expected {expected}, actual {corpus_sha256}"
            ));
        }
    }

    let corpus_value: Value = serde_json::from_slice(&corpus_bytes)
        .map_err(|error| format!("corpus JSON is invalid: {error}"))?;
    let schema_value: Value = serde_json::from_slice(&schema_bytes)
        .map_err(|error| format!("schema JSON is invalid: {error}"))?;

    jsonschema::meta::validate(&schema_value)
        .map_err(|error| format!("JSON Schema is invalid under its meta-schema: {error}"))?;
    let validator = jsonschema::validator_for(&schema_value)
        .map_err(|error| format!("cannot compile JSON Schema validator: {error}"))?;
    let schema_errors: Vec<String> = validator
        .iter_errors(&corpus_value)
        .map(|error| format!("{}: {error}", error.instance_path()))
        .collect();
    if !schema_errors.is_empty() {
        return Err(format!(
            "corpus does not conform to the JSON Schema:\n{}",
            schema_errors.join("\n")
        ));
    }

    let corpus: Corpus = serde_json::from_value(corpus_value)
        .map_err(|error| format!("typed corpus decoding failed: {error}"))?;
    validate_semantics(&corpus)?;

    let mut scenario_class_counts = BTreeMap::new();
    let mut target_counts = BTreeMap::new();
    let scenario_ids: Vec<String> = corpus
        .scenarios
        .iter()
        .map(|scenario| scenario.id.clone())
        .collect();
    for scenario in &corpus.scenarios {
        *scenario_class_counts
            .entry(scenario.scenario_class.as_str().to_string())
            .or_insert(0) += 1;
        for target in &scenario.targets {
            *target_counts.entry(target.clone()).or_insert(0) += 1;
        }
    }

    let validator_source_sha256 = sha256_hex(include_bytes!("main.rs"));
    let validator_manifest_sha256 = sha256_hex(include_bytes!("../Cargo.toml"));
    let dependency_lock_sha256 = sha256_hex(include_bytes!("../Cargo.lock"));

    let receipt = QualificationReceipt {
        receipt_contract: "CIVOS-MACQ-CORPUS-VALIDATION",
        receipt_version: "0.1.0",
        corpus_sha256,
        schema_sha256,
        validator_source_sha256,
        validator_manifest_sha256,
        dependency_lock_sha256,
        corpus_contract_id: corpus.contract_id,
        corpus_version: corpus.version,
        corpus_status: corpus.status,
        source_baseline: corpus.source_baseline,
        scenario_count: corpus.scenarios.len(),
        scenario_class_counts,
        target_counts,
        scenario_ids,
        corpus_validation: "PASS",
        scenario_execution: "NOT_EXECUTED",
        benchmark_result: None,
        claim_boundary: "This receipt validates corpus structure and policy only. It does not execute scenarios or assert that any implementation passes them.",
    };

    let receipt_json = serde_json::to_string_pretty(&receipt)
        .map_err(|error| format!("cannot serialize receipt: {error}"))?;
    if let Some(path) = receipt_path {
        fs::write(&path, format!("{receipt_json}\n"))
            .map_err(|error| format!("cannot write receipt {}: {error}", path.display()))?;
    }
    println!("{receipt_json}");
    Ok(())
}

fn usage() -> String {
    "usage: civos-benchmark-validator <corpus.json> <schema.json> [--receipt <receipt.json>] [--expected-corpus-sha256 <sha256>]".into()
}

fn validate_semantics(corpus: &Corpus) -> Result<(), String> {
    if corpus.contract_id != EXPECTED_CONTRACT_ID {
        return Err(format!("unexpected contract_id {:?}", corpus.contract_id));
    }
    if corpus.version != EXPECTED_VERSION {
        return Err(format!(
            "corpus version must be bumped deliberately from {EXPECTED_VERSION} before changing the frozen contract"
        ));
    }
    if corpus.status != EXPECTED_STATUS {
        return Err(format!(
            "corpus status must remain {EXPECTED_STATUS:?} until scenario execution receipts justify a status change"
        ));
    }
    if corpus.source_baseline.repository.trim().is_empty()
        || corpus.source_baseline.purpose.trim().is_empty()
        || !is_lower_hex_commit(&corpus.source_baseline.commit)
    {
        return Err("source_baseline must contain a repository, purpose and 40-character lowercase commit SHA".into());
    }
    if corpus.fixture_policy.data_class != "synthetic-only" {
        return Err("fixture_policy.data_class must remain synthetic-only".into());
    }
    let wall_clock = corpus.fixture_policy.wall_clock.to_ascii_lowercase();
    if !wall_clock.contains("fixed") && !wall_clock.contains("injected") {
        return Err("fixture policy must specify a fixed or injected clock".into());
    }
    if !corpus
        .fixture_policy
        .randomness
        .to_ascii_lowercase()
        .contains("deterministic")
    {
        return Err("fixture policy must make randomness deterministic and recordable".into());
    }
    if !corpus
        .fixture_policy
        .unknown_applicability
        .to_ascii_lowercase()
        .contains("never pass")
    {
        return Err(
            "unknown or uncovered applicability must never be silently treated as a pass".into(),
        );
    }
    if !corpus
        .fixture_policy
        .global_score
        .to_ascii_lowercase()
        .contains("none")
    {
        return Err(
            "the benchmark contract must not collapse safety gates into a global score".into(),
        );
    }
    if corpus.scenarios.is_empty() {
        return Err("at least one scenario is required".into());
    }

    let mut seen_ids = BTreeSet::new();
    let mut has_hard_gate = false;
    for scenario in &corpus.scenarios {
        if !valid_scenario_id(&scenario.id) {
            return Err(format!(
                "scenario id {:?} does not match the contract ID pattern",
                scenario.id
            ));
        }
        if !seen_ids.insert(scenario.id.as_str()) {
            return Err(format!("duplicate scenario id {:?}", scenario.id));
        }
        if scenario.title.trim().is_empty() {
            return Err(format!("scenario {} has an empty title", scenario.id));
        }
        if scenario.targets.is_empty() {
            return Err(format!(
                "scenario {} must name at least one target",
                scenario.id
            ));
        }
        let mut targets = BTreeSet::new();
        for target in &scenario.targets {
            if target.trim().is_empty() || !targets.insert(target) {
                return Err(format!(
                    "scenario {} has an empty or duplicate target {:?}",
                    scenario.id, target
                ));
            }
        }
        for (name, values) in [
            ("setup", &scenario.setup),
            ("actions", &scenario.actions),
            ("expected", &scenario.expected),
            ("negative_controls", &scenario.negative_controls),
            ("observables", &scenario.observables),
        ] {
            if values.is_empty() || values.iter().any(|value| value.trim().is_empty()) {
                return Err(format!(
                    "scenario {} must have non-empty {name} entries",
                    scenario.id
                ));
            }
        }
        if scenario.scenario_class == ScenarioClass::HardGate {
            has_hard_gate = true;
            if scenario.negative_controls.is_empty() {
                return Err(format!(
                    "hard-gate scenario {} needs a negative control",
                    scenario.id
                ));
            }
        }
        if scenario.scenario_class == ScenarioClass::PerformanceBaseline {
            let observable_text = scenario.observables.join(" ").to_ascii_lowercase();
            if !["latency", "throughput", "resource", "cpu", "memory"]
                .iter()
                .any(|keyword| observable_text.contains(keyword))
            {
                return Err(format!(
                    "performance scenario {} needs a latency/throughput/resource observable",
                    scenario.id
                ));
            }
        }
    }
    if !has_hard_gate {
        return Err("at least one hard-gate scenario is required".into());
    }
    Ok(())
}

fn valid_scenario_id(id: &str) -> bool {
    let Some((prefix, suffix)) = id.rsplit_once('-') else {
        return false;
    };
    if suffix.len() != 3 || !suffix.bytes().all(|byte| byte.is_ascii_digit()) {
        return false;
    }
    !prefix.is_empty()
        && prefix.split('-').all(|part| {
            !part.is_empty()
                && part
                    .bytes()
                    .all(|byte| byte.is_ascii_uppercase() || byte.is_ascii_digit())
        })
}

fn is_lower_hex_commit(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn is_lower_hex_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const CORPUS: &str = include_str!("../../../docs/research/civos-multi-agent-scenarios-v0.json");
    const SCHEMA: &str =
        include_str!("../../../docs/research/civos-multi-agent-scenarios-v0.schema.json");

    fn current_corpus() -> Corpus {
        serde_json::from_str(CORPUS).expect("checked-in corpus must decode")
    }

    #[test]
    fn checked_in_corpus_passes_schema_and_semantic_qualification() {
        let corpus_value: Value = serde_json::from_str(CORPUS).unwrap();
        let schema_value: Value = serde_json::from_str(SCHEMA).unwrap();
        jsonschema::meta::validate(&schema_value).expect("schema must be valid draft 2020-12");
        let validator = jsonschema::validator_for(&schema_value).unwrap();
        assert!(validator.is_valid(&corpus_value));
        validate_semantics(&serde_json::from_value(corpus_value).unwrap()).unwrap();
    }

    #[test]
    fn frozen_corpus_has_21_unique_ids_until_the_contract_version_changes() {
        let corpus = current_corpus();
        assert_eq!(corpus.scenarios.len(), 21);
        let mut ids = BTreeSet::new();
        for scenario in &corpus.scenarios {
            assert!(ids.insert(scenario.id.as_str()));
        }
    }

    #[test]
    fn duplicate_scenario_ids_are_rejected() {
        let mut corpus = current_corpus();
        let first = corpus.scenarios[0].clone();
        corpus.scenarios.push(first);
        assert!(validate_semantics(&corpus).is_err());
    }

    #[test]
    fn non_synthetic_fixture_policy_is_rejected() {
        let mut corpus = current_corpus();
        corpus.fixture_policy.data_class = "production".into();
        assert!(validate_semantics(&corpus).is_err());
    }

    #[test]
    fn qualified_status_is_rejected_without_execution_receipts() {
        let mut corpus = current_corpus();
        corpus.status = "qualified".into();
        assert!(validate_semantics(&corpus).is_err());
    }

    #[test]
    fn uppercase_or_malformed_source_commit_is_rejected() {
        let mut corpus = current_corpus();
        corpus.source_baseline.commit = "A".repeat(40);
        assert!(validate_semantics(&corpus).is_err());
        corpus.source_baseline.commit = "g".repeat(40);
        assert!(validate_semantics(&corpus).is_err());
    }
}
