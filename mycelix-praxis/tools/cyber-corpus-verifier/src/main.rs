use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::env;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::process;

const DEFAULT_MANIFEST: &str = "docs/fixtures/cyber-public-seed-v1.manifest.json";
const EXPECTED_FILES: [&str; 4] = [
    "docs/fixtures/cyber-public-seed-v1.jsonl",
    "docs/fixtures/cyber-public-seed-v1.oracle.jsonl",
    "schemas/cyber-challenge-v1.schema.json",
    "schemas/cyber-oracle-v1.schema.json",
];

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    manifest_version: String,
    purpose: String,
    source_base_commit: String,
    seed_snapshot_commit: String,
    schema_versions: SchemaVersions,
    case_count: usize,
    oracle_count: usize,
    case_ids: Vec<String>,
    files: BTreeMap<String, FileIdentity>,
    qualification_state: String,
    non_claims: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaVersions {
    solver: String,
    oracle: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileIdentity {
    sha256: String,
    utf8_bytes: usize,
    record_count: Option<usize>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Challenge {
    schema_version: String,
    scenario_id: String,
    profile: String,
    domain: String,
    learning_objectives: Vec<String>,
    environment: Environment,
    solver_view: SolverView,
    reference_snapshots: Vec<ReferenceSnapshot>,
    claim_ceiling: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Environment {
    synthetic: bool,
    snapshot_id: String,
    snapshot_version: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SolverView {
    task: String,
    assets: Vec<Asset>,
    observations: Vec<Observation>,
    allowed_actions: Vec<String>,
    safety_invariants: Vec<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Asset {
    asset_id: String,
    asset_type: String,
    description: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    evidence_id: String,
    source: String,
    observed_at: String,
    currentness: String,
    integrity: String,
    statement: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceSnapshot {
    framework: String,
    object_id: String,
    release: String,
    snapshot_uri: String,
    snapshot_sha256: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Oracle {
    schema_version: String,
    scenario_id: String,
    rubric_version: String,
    expected_disposition: String,
    required_evidence_refs: Vec<String>,
    acceptable_claims: Vec<String>,
    prohibited_claims: Vec<String>,
    next_best_steps: Vec<String>,
    authority_expectation: String,
    rationale: String,
}

#[derive(Debug, serde::Serialize)]
struct FileResult {
    sha256: String,
    utf8_bytes: usize,
    record_count: Option<usize>,
}

#[derive(Debug, serde::Serialize)]
struct Report {
    report_version: &'static str,
    disposition: &'static str,
    manifest_version: String,
    source_base_commit: String,
    seed_snapshot_commit: String,
    solver_schema_version: String,
    oracle_schema_version: String,
    case_count: usize,
    oracle_count: usize,
    case_ids: Vec<String>,
    files: BTreeMap<String, FileResult>,
    non_claims: Vec<&'static str>,
}

fn fail(message: impl Into<String>) -> String {
    message.into()
}

fn safe_relative_path(value: &str) -> Result<PathBuf, String> {
    let path = Path::new(value);
    if path.as_os_str().is_empty() || path.is_absolute() {
        return Err(fail(format!("manifest path must be non-empty and relative: {value}")));
    }
    for component in path.components() {
        if !matches!(component, Component::Normal(_)) {
            return Err(fail(format!("manifest path contains a non-normal component: {value}")));
        }
    }
    Ok(path.to_path_buf())
}

fn digest(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn valid_token(value: &str, prefix: Option<&str>) -> bool {
    if value.is_empty()
        || !value
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
        || !value.as_bytes()[0].is_ascii_alphanumeric()
    {
        return false;
    }
    match prefix {
        Some(prefix) => value.starts_with(prefix) && value.len() >= prefix.len() + 3,
        None => true,
    }
}

fn unique_values(values: &[String]) -> bool {
    values.iter().collect::<BTreeSet<_>>().len() == values.len()
}

fn check_schema_contract(bytes: &[u8], label: &str, expected_version: &str) -> Result<(), String> {
    let value: Value = serde_json::from_slice(bytes)
        .map_err(|e| fail(format!("{label}: schema JSON parse failed: {e}")))?;
    if value.get("$schema").and_then(Value::as_str)
        != Some("https://json-schema.org/draft/2020-12/schema")
    {
        return Err(fail(format!("{label}: unexpected JSON Schema dialect")));
    }
    if value.get("type").and_then(Value::as_str) != Some("object")
        || value.get("additionalProperties").and_then(Value::as_bool) != Some(false)
    {
        return Err(fail(format!("{label}: root must be a strict object schema")));
    }
    let version_const = value
        .get("properties")
        .and_then(|v| v.get("schema_version"))
        .and_then(|v| v.get("const"))
        .and_then(Value::as_str);
    if version_const != Some(expected_version) {
        return Err(fail(format!("{label}: schema_version const mismatch")));
    }
    let required = value.get("required").and_then(Value::as_array)
        .ok_or_else(|| fail(format!("{label}: root required list is missing")))?;
    if !required.iter().any(|v| v.as_str() == Some("schema_version"))
        || !required.iter().any(|v| v.as_str() == Some("scenario_id"))
    {
        return Err(fail(format!("{label}: root requires schema_version and scenario_id")));
    }
    Ok(())
}

fn parse_jsonl<T: for<'de> Deserialize<'de>>(bytes: &[u8], label: &str) -> Result<Vec<T>, String> {
    let text = std::str::from_utf8(bytes)
        .map_err(|e| fail(format!("{label}: not valid UTF-8: {e}")))?;
    if !text.ends_with('\n') {
        return Err(fail(format!("{label}: JSONL must end with LF")));
    }
    let mut records = Vec::new();
    for (index, line) in text.split_terminator('\n').enumerate() {
        if line.trim().is_empty() {
            return Err(fail(format!("{label}: blank record at line {}", index + 1)));
        }
        let value = serde_json::from_str::<T>(line)
            .map_err(|e| fail(format!("{label}: JSON parse failed at line {}: {e}", index + 1)))?;
        records.push(value);
    }
    if records.is_empty() {
        return Err(fail(format!("{label}: corpus is empty")));
    }
    Ok(records)
}

fn validate_challenge(case: &Challenge) -> Result<BTreeSet<String>, String> {
    let id = &case.scenario_id;
    if case.schema_version != "cyber-challenge-v1"
        || !valid_token(id, Some("cyber-"))
        || case.profile != "isolated-synthetic-v1"
        || case.domain.trim().len() < 3
        || case.learning_objectives.is_empty()
        || case.learning_objectives.iter().any(|x| x.trim().len() < 5)
        || !case.environment.synthetic
        || case.environment.snapshot_id.trim().is_empty()
        || case.environment.snapshot_version.trim().is_empty()
        || case.solver_view.task.trim().len() < 20
        || case.solver_view.assets.is_empty()
        || case.solver_view.observations.is_empty()
        || case.solver_view.allowed_actions.is_empty()
        || case.solver_view.safety_invariants.is_empty()
        || case.solver_view.safety_invariants.iter().any(|x| x.trim().len() < 8)
        || case.claim_ceiling.trim().len() < 20
    {
        return Err(fail(format!("{id}: challenge has invalid required fields")));
    }
    if !unique_values(&case.learning_objectives) || !unique_values(&case.solver_view.allowed_actions) {
        return Err(fail(format!("{id}: duplicate learning objective or allowed action")));
    }

    let allowed_actions = [
        "inspect_evidence",
        "request_more_evidence",
        "preserve_evidence",
        "propose_containment",
        "propose_quarantine",
        "request_fresh_authorization",
        "reconcile_state",
        "abstain",
    ];
    for action in &case.solver_view.allowed_actions {
        if !allowed_actions.contains(&action.as_str()) {
            return Err(fail(format!("{id}: unknown action intent {action}")));
        }
    }

    let mut asset_ids = BTreeSet::new();
    for asset in &case.solver_view.assets {
        if !valid_token(&asset.asset_id, None)
            || asset.asset_type.trim().is_empty()
            || asset.description.trim().len() < 5
            || !asset_ids.insert(asset.asset_id.as_str())
        {
            return Err(fail(format!("{id}: malformed or duplicate asset")));
        }
    }

    let mut evidence_ids = BTreeSet::new();
    for obs in &case.solver_view.observations {
        if !valid_token(&obs.evidence_id, None)
            || obs.source.trim().len() < 2
            || obs.observed_at.trim().len() < 10
            || obs.statement.trim().len() < 5
            || !matches!(obs.currentness.as_str(), "current" | "stale" | "disputed" | "unknown")
            || !matches!(obs.integrity.as_str(), "verified" | "unverified" | "mismatch" | "unknown")
            || !evidence_ids.insert(obs.evidence_id.clone())
        {
            return Err(fail(format!("{id}: malformed or duplicate observation/evidence ID")));
        }
    }

    for reference in &case.reference_snapshots {
        if reference.framework.trim().len() < 2
            || reference.object_id.trim().len() < 2
            || reference.release.trim().is_empty()
            || !(reference.snapshot_uri.starts_with("https://") || reference.snapshot_uri.starts_with("http://"))
            || reference.snapshot_sha256.len() != 64
            || !reference.snapshot_sha256.bytes().all(|b| b.is_ascii_hexdigit())
        {
            return Err(fail(format!("{id}: incomplete or unpinned external reference")));
        }
    }
    Ok(evidence_ids)
}

fn validate_oracle(oracle: &Oracle) -> Result<(), String> {
    let allowed_dispositions = [
        "not_applicable_on_supplied_build",
        "approved_change_consistent_not_proof_of_no_compromise",
        "insufficient_evidence_keep_suspicion_unconfirmed",
        "artifact_provenance_mismatch_block_promotion",
        "untrusted_instruction_rejected",
        "authorization_expired_block_effect",
        "duplicate_and_reordered_telemetry_not_independent",
        "outcome_unknown_reconcile_before_retry",
    ];
    if oracle.schema_version != "cyber-oracle-v1"
        || !valid_token(&oracle.scenario_id, Some("cyber-"))
        || oracle.rubric_version.trim().is_empty()
        || !allowed_dispositions.contains(&oracle.expected_disposition.as_str())
        || oracle.required_evidence_refs.is_empty()
        || !unique_values(&oracle.required_evidence_refs)
        || oracle.acceptable_claims.is_empty()
        || oracle.prohibited_claims.is_empty()
        || oracle.next_best_steps.is_empty()
        || oracle.authority_expectation.trim().len() < 10
        || oracle.rationale.trim().len() < 20
    {
        return Err(fail(format!("{}: malformed evaluator oracle", oracle.scenario_id)));
    }
    Ok(())
}

fn validate_oracle_evidence_refs(
    oracle: &Oracle,
    evidence: &BTreeSet<String>,
) -> Result<(), String> {
    for evidence_ref in &oracle.required_evidence_refs {
        if !evidence.contains(evidence_ref) {
            return Err(fail(format!(
                "oracle {} references absent evidence ID {}",
                oracle.scenario_id, evidence_ref
            )));
        }
    }
    Ok(())
}

fn verify(root: &Path, manifest_relative: &str) -> Result<Report, String> {
    let manifest_path = safe_relative_path(manifest_relative)?;
    let manifest_bytes = fs::read(root.join(&manifest_path))
        .map_err(|e| fail(format!("cannot read manifest {}: {e}", manifest_path.display())))?;
    let manifest: Manifest = serde_json::from_slice(&manifest_bytes)
        .map_err(|e| fail(format!("cannot parse manifest: {e}")))?;

    if manifest.manifest_version != "cyber-public-seed-manifest-v1"
        || manifest.schema_versions.solver != "cyber-challenge-v1"
        || manifest.schema_versions.oracle != "cyber-oracle-v1"
        || manifest.qualification_state != "UNSCORED"
        || manifest.case_ids.is_empty()
        || !unique_values(&manifest.case_ids)
        || manifest.non_claims.is_empty()
        || manifest.source_base_commit.trim().is_empty()
        || manifest.seed_snapshot_commit.trim().is_empty()
        || manifest.purpose.trim().is_empty()
    {
        return Err(fail("manifest metadata, version, or state is invalid"));
    }
    let expected_keys: BTreeSet<String> = EXPECTED_FILES.iter().map(|s| s.to_string()).collect();
    let actual_keys: BTreeSet<String> = manifest.files.keys().cloned().collect();
    if expected_keys != actual_keys {
        return Err(fail(format!(
            "manifest file set mismatch: expected {:?}, found {:?}",
            expected_keys, actual_keys
        )));
    }

    // Verify exact file bytes before parsing any scenario or oracle records.
    let mut byte_files = BTreeMap::<String, Vec<u8>>::new();
    let mut file_results = BTreeMap::<String, FileResult>::new();
    for (relative, identity) in &manifest.files {
        let safe = safe_relative_path(relative)?;
        let bytes = fs::read(root.join(&safe))
            .map_err(|e| fail(format!("cannot read {}: {e}", safe.display())))?;
        let actual_digest = digest(&bytes);
        if actual_digest != identity.sha256 {
            return Err(fail(format!(
                "SHA-256 mismatch for {relative}: expected {}, got {}",
                identity.sha256, actual_digest
            )));
        }
        if bytes.len() != identity.utf8_bytes {
            return Err(fail(format!(
                "byte-length mismatch for {relative}: expected {}, got {}",
                identity.utf8_bytes, bytes.len()
            )));
        }
        if let Some(expected_count) = identity.record_count {
            let text = std::str::from_utf8(&bytes)
                .map_err(|e| fail(format!("{relative}: not valid UTF-8: {e}")))?;
            let actual_count = text.lines().count();
            if actual_count != expected_count {
                return Err(fail(format!(
                    "record count mismatch for {relative}: expected {}, got {}",
                    expected_count, actual_count
                )));
            }
        }
        file_results.insert(relative.clone(), FileResult {
            sha256: actual_digest,
            utf8_bytes: bytes.len(),
            record_count: identity.record_count,
        });
        byte_files.insert(relative.clone(), bytes);
    }

    check_schema_contract(
        &byte_files["schemas/cyber-challenge-v1.schema.json"],
        "challenge schema",
        "cyber-challenge-v1",
    )?;
    check_schema_contract(
        &byte_files["schemas/cyber-oracle-v1.schema.json"],
        "oracle schema",
        "cyber-oracle-v1",
    )?;

    let cases = parse_jsonl::<Challenge>(
        &byte_files["docs/fixtures/cyber-public-seed-v1.jsonl"],
        "solver seed",
    )?;
    let oracles = parse_jsonl::<Oracle>(
        &byte_files["docs/fixtures/cyber-public-seed-v1.oracle.jsonl"],
        "oracle seed",
    )?;

    if cases.len() != manifest.case_count || oracles.len() != manifest.oracle_count {
        return Err(fail(format!(
            "manifest count mismatch: cases {}/{}, oracles {}/{}",
            cases.len(), manifest.case_count, oracles.len(), manifest.oracle_count
        )));
    }
    if cases.len() != oracles.len() {
        return Err(fail("solver and oracle record counts differ"));
    }

    let mut case_ids = Vec::new();
    let mut case_evidence = BTreeMap::<String, BTreeSet<String>>::new();
    for case in &cases {
        if case_evidence.contains_key(&case.scenario_id) {
            return Err(fail(format!("duplicate scenario ID {}", case.scenario_id)));
        }
        case_evidence.insert(case.scenario_id.clone(), validate_challenge(case)?);
        case_ids.push(case.scenario_id.clone());
    }
    if case_ids != manifest.case_ids {
        return Err(fail("manifest case_ids do not exactly match solver corpus order"));
    }

    let mut oracle_ids = BTreeSet::new();
    for oracle in &oracles {
        validate_oracle(oracle)?;
        if !oracle_ids.insert(oracle.scenario_id.clone()) {
            return Err(fail(format!("duplicate oracle scenario ID {}", oracle.scenario_id)));
        }
        let Some(evidence) = case_evidence.get(&oracle.scenario_id) else {
            return Err(fail(format!("oracle {} has no solver-visible scenario", oracle.scenario_id)));
        };
        validate_oracle_evidence_refs(oracle, evidence)?;
    }
    let case_id_set: BTreeSet<String> = case_evidence.keys().cloned().collect();
    if oracle_ids != case_id_set {
        return Err(fail("solver and oracle scenario ID sets differ"));
    }

    Ok(Report {
        report_version: "cyber-corpus-structural-report-v1",
        disposition: "STRUCTURAL_PASS_NOT_CAPABILITY_PASS",
        manifest_version: manifest.manifest_version,
        source_base_commit: manifest.source_base_commit,
        seed_snapshot_commit: manifest.seed_snapshot_commit,
        solver_schema_version: manifest.schema_versions.solver,
        oracle_schema_version: manifest.schema_versions.oracle,
        case_count: cases.len(),
        oracle_count: oracles.len(),
        case_ids,
        files: file_results,
        non_claims: vec![
            "This report establishes byte identity and bounded structural conformance only.",
            "It does not validate the full JSON Schema dialect against a meta-schema.",
            "It does not score the semantic correctness of a solver or an evaluator rubric.",
            "The public seed and answer key are not a blind capability benchmark.",
            "It does not establish real-world security, learner efficacy, or execution authority.",
        ],
    })
}

fn main() {
    let mut args = env::args().skip(1);
    let root = PathBuf::from(args.next().unwrap_or_else(|| "mycelix-praxis".to_string()));
    let manifest = args.next().unwrap_or_else(|| DEFAULT_MANIFEST.to_string());
    if args.next().is_some() {
        eprintln!("usage: cyber-corpus-verifier [praxis-root] [manifest-relative-path]");
        process::exit(2);
    }

    match verify(&root, &manifest) {
        Ok(report) => {
            println!("{}", serde_json::to_string_pretty(&report).expect("report serialization is infallible"));
        }
        Err(error) => {
            eprintln!("cyber-corpus-verifier: FAIL: {error}");
            process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_known_answer() {
        assert_eq!(
            digest(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn paths_cannot_escape_root() {
        assert!(safe_relative_path("../Cargo.toml").is_err());
        assert!(safe_relative_path("/etc/passwd").is_err());
        assert!(safe_relative_path("docs/fixtures/seed.jsonl").is_ok());
    }

    #[test]
    fn parser_rejects_blank_jsonl_records_and_missing_final_lf() {
        assert!(parse_jsonl::<Value>(b"{\"ok\":true}\n\n", "test").is_err());
        assert!(parse_jsonl::<Value>(b"{\"ok\":true}", "test").is_err());
    }

    #[test]
    fn strict_challenge_deserialization_rejects_unknown_fields() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let corpus = fs::read_to_string(root.join("docs/fixtures/cyber-public-seed-v1.jsonl"))
            .expect("public challenge fixture should be readable");
        let mut value: Value = serde_json::from_str(corpus.lines().next().expect("first record"))
            .expect("challenge fixture should be JSON");
        value.as_object_mut().expect("challenge is an object")
            .insert("unrecognized_field".to_string(), Value::Bool(true));
        assert!(serde_json::from_value::<Challenge>(value).is_err());
    }

    #[test]
    fn strict_oracle_deserialization_rejects_unknown_fields() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let corpus = fs::read_to_string(root.join("docs/fixtures/cyber-public-seed-v1.oracle.jsonl"))
            .expect("public oracle fixture should be readable");
        let mut value: Value = serde_json::from_str(corpus.lines().next().expect("first record"))
            .expect("oracle fixture should be JSON");
        value.as_object_mut().expect("oracle is an object")
            .insert("unrecognized_field".to_string(), Value::Bool(true));
        assert!(serde_json::from_value::<Oracle>(value).is_err());
    }

    #[test]
    fn oracle_cannot_reference_absent_evidence() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let scenarios = fs::read_to_string(root.join("docs/fixtures/cyber-public-seed-v1.jsonl"))
            .expect("public challenge fixture should be readable");
        let oracle_text = fs::read_to_string(root.join("docs/fixtures/cyber-public-seed-v1.oracle.jsonl"))
            .expect("public oracle fixture should be readable");
        let challenge: Challenge = serde_json::from_str(scenarios.lines().next().expect("first record"))
            .expect("challenge fixture should deserialize");
        let mut oracle: Oracle = serde_json::from_str(oracle_text.lines().next().expect("first record"))
            .expect("oracle fixture should deserialize");
        let evidence = validate_challenge(&challenge).expect("challenge evidence should validate");
        oracle.required_evidence_refs.push("missing-evidence-999".to_string());
        assert!(validate_oracle_evidence_refs(&oracle, &evidence).is_err());
    }

    #[test]
    fn public_seed_passes_structural_verification_only() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
        let report = verify(&root, DEFAULT_MANIFEST).expect("checked-in public seed should conform");
        assert_eq!(report.disposition, "STRUCTURAL_PASS_NOT_CAPABILITY_PASS");
        assert_eq!(report.case_count, 8);
        assert_eq!(report.oracle_count, 8);
        assert_eq!(report.case_ids.len(), 8);
        assert_eq!(report.files.len(), EXPECTED_FILES.len());
    }
}
