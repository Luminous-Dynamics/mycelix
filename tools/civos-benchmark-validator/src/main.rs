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

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct CoverageManifest {
    contract_id: String,
    version: String,
    status: String,
    source_corpus: CoverageSourceCorpus,
    coverage_policy: CoveragePolicy,
    mappings: Vec<ScenarioMapping>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct CoverageSourceCorpus {
    contract_id: String,
    version: String,
    sha256: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct CoveragePolicy {
    unmapped_is_pass: bool,
    qualified_claims_enabled: bool,
    global_score: String,
    evidence_rule: String,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
enum ExecutionProfile {
    Unit,
    MultiAgent,
    Browser,
    FaultInjection,
    CrossVersion,
    ProtocolInterop,
    RuntimePerformance,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
enum MappingStatus {
    Unmapped,
    CandidatePartial,
    ImplementedUnqualified,
}

impl MappingStatus {
    fn as_str(self) -> &'static str {
        match self {
            Self::Unmapped => "unmapped",
            Self::CandidatePartial => "candidate_partial",
            Self::ImplementedUnqualified => "implemented_unqualified",
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct ScenarioMapping {
    scenario_id: String,
    primary_owner: String,
    execution_profile: ExecutionProfile,
    mapping_status: MappingStatus,
    test_references: Vec<TestReference>,
    known_gaps: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct TestReference {
    path: String,
    symbol: String,
    source_commit: String,
    purpose: String,
}

#[derive(Debug, Serialize)]
struct QualificationReceipt {
    receipt_contract: &'static str,
    receipt_version: &'static str,
    corpus_sha256: String,
    schema_sha256: String,
    coverage_sha256: Option<String>,
    coverage_schema_sha256: Option<String>,
    coverage_validation: &'static str,
    coverage_mapping_status_counts: BTreeMap<String, usize>,
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
    coverage_claim_boundary: &'static str,
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
    let mut coverage_path: Option<PathBuf> = None;
    let mut coverage_schema_path: Option<PathBuf> = None;

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
            "--coverage" => {
                coverage_path = Some(PathBuf::from(
                    args.next()
                        .ok_or("--coverage requires a file path".to_string())?,
                ));
            }
            "--coverage-schema" => {
                coverage_schema_path = Some(PathBuf::from(
                    args.next()
                        .ok_or("--coverage-schema requires a file path".to_string())?,
                ));
            }
            other => return Err(format!("unknown argument {other:?}\n{}", usage())),
        }
    }

    if coverage_path.is_some() != coverage_schema_path.is_some() {
        return Err("--coverage and --coverage-schema must be provided together".into());
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

    let mut coverage_sha256: Option<String> = None;
    let mut coverage_schema_sha256: Option<String> = None;
    let mut coverage_validation = "NOT_PROVIDED";
    let mut coverage_mapping_status_counts = BTreeMap::new();
    if let (Some(path), Some(schema_path)) = (coverage_path.as_ref(), coverage_schema_path.as_ref())
    {
        let coverage_bytes = fs::read(path).map_err(|error| {
            format!("cannot read coverage manifest {}: {error}", path.display())
        })?;
        let coverage_schema_bytes = fs::read(schema_path).map_err(|error| {
            format!(
                "cannot read coverage schema {}: {error}",
                schema_path.display()
            )
        })?;
        let coverage_digest = sha256_hex(&coverage_bytes);
        let coverage_schema_digest = sha256_hex(&coverage_schema_bytes);
        let coverage_value: Value = serde_json::from_slice(&coverage_bytes)
            .map_err(|error| format!("coverage manifest JSON is invalid: {error}"))?;
        let coverage_schema_value: Value = serde_json::from_slice(&coverage_schema_bytes)
            .map_err(|error| format!("coverage schema JSON is invalid: {error}"))?;

        jsonschema::meta::validate(&coverage_schema_value).map_err(|error| {
            format!("coverage JSON Schema is invalid under its meta-schema: {error}")
        })?;
        let coverage_validator = jsonschema::validator_for(&coverage_schema_value)
            .map_err(|error| format!("cannot compile coverage JSON Schema validator: {error}"))?;
        let coverage_schema_errors: Vec<String> = coverage_validator
            .iter_errors(&coverage_value)
            .map(|error| format!("{}: {error}", error.instance_path()))
            .collect();
        if !coverage_schema_errors.is_empty() {
            return Err(format!(
                "coverage manifest does not conform to the JSON Schema:\\n{}",
                coverage_schema_errors.join("\\n")
            ));
        }

        let coverage: CoverageManifest = serde_json::from_value(coverage_value)
            .map_err(|error| format!("typed coverage manifest decoding failed: {error}"))?;
        validate_coverage_semantics(&coverage, &corpus, &corpus_sha256)?;
        for mapping in &coverage.mappings {
            *coverage_mapping_status_counts
                .entry(mapping.mapping_status.as_str().to_string())
                .or_insert(0) += 1;
        }
        coverage_sha256 = Some(coverage_digest);
        coverage_schema_sha256 = Some(coverage_schema_digest);
        coverage_validation = "PASS";
    }

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
        coverage_sha256,
        coverage_schema_sha256,
        coverage_validation,
        coverage_mapping_status_counts,
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
        coverage_claim_boundary: "Coverage mappings identify source-linked test candidates only. They are not execution evidence; no scenario is qualified by this receipt.",
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
    "usage: civos-benchmark-validator <corpus.json> <schema.json> [--receipt <receipt.json>] [--expected-corpus-sha256 <sha256>] [--coverage <coverage.json> --coverage-schema <coverage.schema.json>]".into()
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

fn validate_coverage_semantics(
    coverage: &CoverageManifest,
    corpus: &Corpus,
    actual_corpus_sha256: &str,
) -> Result<(), String> {
    if coverage.contract_id != "CIVOS-MACQ-COVERAGE-001" {
        return Err(format!(
            "unexpected coverage contract_id {:?}",
            coverage.contract_id
        ));
    }
    if coverage.version != "0.1.0" {
        return Err(
            "coverage manifest version must be bumped deliberately before editing the frozen map"
                .into(),
        );
    }
    if coverage.status != "planning-only" {
        return Err("coverage manifest status must remain planning-only; a mapping is not a qualification receipt".into());
    }
    if coverage.source_corpus.contract_id != corpus.contract_id
        || coverage.source_corpus.version != corpus.version
    {
        return Err(
            "coverage manifest refers to a different source corpus contract/version".into(),
        );
    }
    if coverage.source_corpus.sha256 != actual_corpus_sha256 {
        return Err(format!(
            "coverage source-corpus digest mismatch: manifest {}, actual {}",
            coverage.source_corpus.sha256, actual_corpus_sha256
        ));
    }
    if coverage.coverage_policy.unmapped_is_pass {
        return Err("unmapped scenarios must never count as pass".into());
    }
    if coverage.coverage_policy.qualified_claims_enabled {
        return Err("coverage mapping v0 cannot emit qualified claims".into());
    }
    if !coverage
        .coverage_policy
        .global_score
        .to_ascii_lowercase()
        .contains("none")
    {
        return Err("coverage policy must not collapse scenarios into a global score".into());
    }
    if coverage.coverage_policy.evidence_rule.trim().is_empty() {
        return Err("coverage policy must explain the evidence boundary".into());
    }
    if coverage.mappings.len() != corpus.scenarios.len() {
        return Err(format!(
            "coverage must map every corpus scenario exactly once: expected {}, found {}",
            corpus.scenarios.len(),
            coverage.mappings.len()
        ));
    }

    let mut seen_ids = BTreeSet::new();
    for (scenario, mapping) in corpus.scenarios.iter().zip(&coverage.mappings) {
        if mapping.scenario_id != scenario.id {
            return Err(format!(
                "coverage mapping order/identity mismatch: expected {}, found {}",
                scenario.id, mapping.scenario_id
            ));
        }
        if !seen_ids.insert(mapping.scenario_id.as_str()) {
            return Err(format!(
                "duplicate coverage mapping {:?}",
                mapping.scenario_id
            ));
        }
        if mapping.primary_owner.trim().is_empty() {
            return Err(format!(
                "{} has an empty primary owner",
                mapping.scenario_id
            ));
        }
        if mapping.known_gaps.is_empty()
            || mapping.known_gaps.iter().any(|gap| gap.trim().is_empty())
        {
            return Err(format!(
                "{} must list concrete known gaps",
                mapping.scenario_id
            ));
        }
        let mut ref_keys = BTreeSet::new();
        for reference in &mapping.test_references {
            if reference.path.trim().is_empty()
                || reference.path.starts_with('/')
                || reference.path.split('/').any(|part| part == "..")
                || reference.symbol.trim().is_empty()
                || reference.purpose.trim().is_empty()
                || !is_lower_hex_commit(&reference.source_commit)
            {
                return Err(format!(
                    "{} has an invalid source-linked test reference",
                    mapping.scenario_id
                ));
            }
            if !ref_keys.insert((
                reference.path.as_str(),
                reference.symbol.as_str(),
                reference.source_commit.as_str(),
            )) {
                return Err(format!(
                    "{} has a duplicate test reference",
                    mapping.scenario_id
                ));
            }
        }
        match mapping.mapping_status {
            MappingStatus::Unmapped if !mapping.test_references.is_empty() => {
                return Err(format!(
                    "{} is unmapped but lists test references",
                    mapping.scenario_id
                ));
            }
            MappingStatus::CandidatePartial | MappingStatus::ImplementedUnqualified
                if mapping.test_references.is_empty() =>
            {
                return Err(format!(
                    "{} claims a mapping status without any source-linked test reference",
                    mapping.scenario_id
                ));
            }
            _ => {}
        }
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
    const COVERAGE: &str =
        include_str!("../../../docs/research/civos-multi-agent-coverage-v0.json");
    const COVERAGE_SCHEMA: &str =
        include_str!("../../../docs/research/civos-multi-agent-coverage-v0.schema.json");

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

    fn current_coverage() -> CoverageManifest {
        serde_json::from_str(COVERAGE).expect("checked-in coverage manifest must decode")
    }

    fn current_coverage_sha256() -> String {
        sha256_hex(CORPUS.as_bytes())
    }

    #[test]
    fn checked_in_coverage_manifest_matches_frozen_corpus_and_never_claims_qualification() {
        let corpus = current_corpus();
        let coverage_value: Value = serde_json::from_str(COVERAGE).unwrap();
        let schema_value: Value = serde_json::from_str(COVERAGE_SCHEMA).unwrap();
        jsonschema::meta::validate(&schema_value)
            .expect("coverage schema must be valid draft 2020-12");
        let validator = jsonschema::validator_for(&schema_value).unwrap();
        assert!(validator.is_valid(&coverage_value));
        let coverage: CoverageManifest = serde_json::from_value(coverage_value).unwrap();
        validate_coverage_semantics(&coverage, &corpus, &current_coverage_sha256()).unwrap();
        assert_eq!(coverage.mappings.len(), 21);
        assert_eq!(
            coverage
                .mappings
                .iter()
                .filter(|m| m.mapping_status == MappingStatus::Unmapped)
                .count(),
            18
        );
        assert_eq!(
            coverage
                .mappings
                .iter()
                .filter(|m| m.mapping_status == MappingStatus::CandidatePartial)
                .count(),
            3
        );
    }

    #[test]
    fn coverage_with_missing_or_reordered_scenario_mapping_is_rejected() {
        let corpus = current_corpus();
        let mut coverage = current_coverage();
        coverage.mappings.pop();
        assert!(
            validate_coverage_semantics(&coverage, &corpus, &current_coverage_sha256()).is_err()
        );

        let mut coverage = current_coverage();
        coverage.mappings.swap(0, 1);
        assert!(
            validate_coverage_semantics(&coverage, &corpus, &current_coverage_sha256()).is_err()
        );
    }

    #[test]
    fn coverage_cannot_claim_a_different_corpus_digest() {
        let corpus = current_corpus();
        let mut coverage = current_coverage();
        coverage.source_corpus.sha256 = "0".repeat(64);
        assert!(
            validate_coverage_semantics(&coverage, &corpus, &current_coverage_sha256()).is_err()
        );
    }

    #[test]
    fn unmapped_scenario_cannot_list_a_test_reference() {
        let corpus = current_corpus();
        let mut coverage = current_coverage();
        let unmapped = coverage
            .mappings
            .iter_mut()
            .find(|mapping| mapping.mapping_status == MappingStatus::Unmapped)
            .unwrap();
        unmapped.test_references.push(TestReference {
            path: "src/lib.rs".into(),
            symbol: "test_candidate".into(),
            source_commit: "a".repeat(40),
            purpose: "synthetic negative control".into(),
        });
        assert!(
            validate_coverage_semantics(&coverage, &corpus, &current_coverage_sha256()).is_err()
        );
    }

    #[test]
    fn a_qualified_mapping_status_is_not_part_of_the_v0_contract() {
        let mut value: Value = serde_json::from_str(COVERAGE).unwrap();
        value["mappings"][0]["mapping_status"] = Value::String("qualified".into());
        assert!(serde_json::from_value::<CoverageManifest>(value).is_err());
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
