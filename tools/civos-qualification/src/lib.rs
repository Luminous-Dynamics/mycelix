use base64::{Engine as _, engine::general_purpose::STANDARD};
use ed25519_dalek::{Signature, VerifyingKey};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::{Component, Path},
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ContractSummary {
    pub contract_id: String,
    pub version: String,
    pub scenario_count: usize,
    pub hard_gate_count: usize,
    pub behavioral_count: usize,
    pub performance_baseline_count: usize,
    pub corpus_sha256: String,
    pub schema_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationExit {
    Qualified,
    Failed,
    Incomplete,
}

impl QualificationExit {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Qualified => "QUALIFIED",
            Self::Failed => "FAILED",
            Self::Incomplete => "INCOMPLETE",
        }
    }

    pub fn exit_code(&self) -> i32 {
        match self {
            Self::Qualified => 0,
            Self::Failed => 1,
            Self::Incomplete => 2,
        }
    }
}

/// Domain separation prevents a signature made for another protocol or
/// receipt version from being reused as CivOS qualification evidence.
const RECEIPT_SIGNATURE_DOMAIN: &[u8] = b"LUMINOUS-DYNAMICS/CIVOS-MACQ-001/RECEIPT/V0.1\0";

/// Return the domain-separated message bytes signed by trusted receipt producers.
///
/// The function hashes the exact receipt bytes, then prefixes the digest with a
/// versioned protocol domain. Do not parse and reserialize the receipt before
/// calling this function; the signature is intended to bind the original bytes.
pub fn receipt_signature_message(receipt_bytes: &[u8]) -> Vec<u8> {
    let digest = Sha256::digest(receipt_bytes);
    let mut message = Vec::with_capacity(RECEIPT_SIGNATURE_DOMAIN.len() + digest.len());
    message.extend_from_slice(RECEIPT_SIGNATURE_DOMAIN);
    message.extend_from_slice(&digest);
    message
}

/// Verify a detached Ed25519 signature over the exact receipt bytes.
///
/// `trusted_public_key` is exactly 32 raw Ed25519 public-key bytes.
/// `signature_base64` is the detached 64-byte signature encoded as Base64.
/// `expected_public_key_sha256` must come from an independent trust
/// configuration (for example a protected CI variable or reviewed allowlist),
/// not from the receipt or the same untrusted source that supplied the key.
///
/// This proves the receipt was signed by the private key corresponding to the
/// pinned public key and binds the signature to the exact bytes. It does not,
/// by itself, prove the test oracle is correct or that the pinned signer is
/// authorized to attest a specific production environment.
pub fn verify_receipt_signature(
    receipt_bytes: &[u8],
    signature_base64: &str,
    trusted_public_key: &[u8],
    expected_public_key_sha256: &str,
) -> Result<String, String> {
    let actual_fingerprint = sha256_hex(trusted_public_key);
    if actual_fingerprint != expected_public_key_sha256 {
        return Err(format!(
            "trusted receipt public-key fingerprint mismatch: expected {expected_public_key_sha256}, actual {actual_fingerprint}"
        ));
    }

    let public_key_bytes: [u8; 32] = trusted_public_key
        .try_into()
        .map_err(|_| "trusted Ed25519 public key must contain exactly 32 raw bytes".to_string())?;
    let verifying_key = VerifyingKey::from_bytes(&public_key_bytes)
        .map_err(|error| format!("trusted Ed25519 public key is invalid: {error}"))?;

    let signature_bytes = STANDARD
        .decode(signature_base64.trim())
        .map_err(|error| format!("detached receipt signature is not valid Base64: {error}"))?;
    let signature_array: [u8; 64] = signature_bytes
        .as_slice()
        .try_into()
        .map_err(|_| "detached Ed25519 signature must contain exactly 64 bytes".to_string())?;
    let signature = Signature::from_bytes(&signature_array);
    verifying_key
        .verify_strict(&receipt_signature_message(receipt_bytes), &signature)
        .map_err(|_| "detached receipt signature verification failed".to_string())?;

    Ok(actual_fingerprint)
}

fn parse_json(text: &str, label: &str) -> Result<Value, String> {
    serde_json::from_str(text).map_err(|error| format!("{label} is not valid JSON: {error}"))
}

pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn validate_schema_document(schema: &Value, label: &str) -> Result<(), String> {
    jsonschema::draft202012::meta::validate(schema)
        .map_err(|error| format!("{label} is not a valid Draft 2020-12 schema: {error}"))
}

fn validate_instance(schema: &Value, instance: &Value, label: &str) -> Result<(), String> {
    jsonschema::draft202012::validate(schema, instance)
        .map_err(|error| format!("{label} does not satisfy its schema: {error}"))
}

fn required_string<'a>(value: &'a Value, pointer: &str) -> Result<&'a str, String> {
    value
        .pointer(pointer)
        .and_then(Value::as_str)
        .filter(|text| !text.trim().is_empty())
        .ok_or_else(|| format!("missing or empty required string at {pointer}"))
}

pub fn validate_contract(corpus_text: &str, schema_text: &str) -> Result<ContractSummary, String> {
    let corpus = parse_json(corpus_text, "scenario corpus")?;
    let schema = parse_json(schema_text, "scenario schema")?;
    validate_schema_document(&schema, "scenario schema")?;
    validate_instance(&schema, &corpus, "scenario corpus")?;

    let contract_id = required_string(&corpus, "/contract_id")?.to_string();
    let version = required_string(&corpus, "/version")?.to_string();
    let scenarios = corpus
        .get("scenarios")
        .and_then(Value::as_array)
        .ok_or_else(|| "scenario corpus has no scenarios array".to_string())?;

    let mut ids = BTreeSet::new();
    let mut hard_gate_count = 0;
    let mut behavioral_count = 0;
    let mut performance_baseline_count = 0;

    for (index, scenario) in scenarios.iter().enumerate() {
        let id = required_string(scenario, "/id")
            .map_err(|error| format!("scenario index {index}: {error}"))?;
        if !ids.insert(id.to_string()) {
            return Err(format!("duplicate scenario id: {id}"));
        }

        let class = required_string(scenario, "/class")?;
        match class {
            "hard_gate" => hard_gate_count += 1,
            "behavioral" => behavioral_count += 1,
            "performance_baseline" => performance_baseline_count += 1,
            _ => return Err(format!("scenario {id} has unknown class {class}")),
        }

        let targets = scenario
            .get("targets")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("scenario {id} has no targets array"))?;
        if targets.is_empty() {
            return Err(format!("scenario {id} has no declared targets"));
        }
        let negative_controls = scenario
            .get("negative_controls")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("scenario {id} has no negative_controls array"))?;
        if class == "hard_gate" && negative_controls.is_empty() {
            return Err(format!(
                "hard-gate scenario {id} has no negative control; reject non-falsifiable safety fixtures"
            ));
        }
    }

    if hard_gate_count == 0 {
        return Err("scenario corpus defines no hard_gate cases; fail closed".into());
    }
    if contract_id != "CIVOS-MACQ-001" {
        return Err(format!("unexpected contract_id: {contract_id}"));
    }

    Ok(ContractSummary {
        contract_id,
        version,
        scenario_count: scenarios.len(),
        hard_gate_count,
        behavioral_count,
        performance_baseline_count,
        corpus_sha256: sha256_hex(corpus_text.as_bytes()),
        schema_sha256: sha256_hex(schema_text.as_bytes()),
    })
}

fn evidence_digest_set(execution: &Value) -> BTreeSet<String> {
    execution
        .get("evidence")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(|entry| entry.get("sha256").and_then(Value::as_str))
        .map(str::to_string)
        .collect()
}

fn verify_local_hashed_file(root: &Path, record: &Value, label: &str) -> Result<String, String> {
    let declared_path = required_string(record, "/path")?;
    let relative = Path::new(declared_path);
    if relative.is_absolute()
        || !relative
            .components()
            .all(|part| matches!(part, Component::Normal(_)))
    {
        return Err(format!(
            "{label} path must be a safe relative path: {declared_path}"
        ));
    }

    let canonical_root = fs::canonicalize(root)
        .map_err(|error| format!("cannot resolve evidence root {}: {error}", root.display()))?;
    let candidate = fs::canonicalize(canonical_root.join(relative))
        .map_err(|error| format!("cannot resolve {label} {declared_path}: {error}"))?;
    if !candidate.starts_with(&canonical_root) {
        return Err(format!(
            "{label} path escapes the declared evidence root: {declared_path}"
        ));
    }

    let bytes = fs::read(&candidate)
        .map_err(|error| format!("cannot read {label} {declared_path}: {error}"))?;
    let actual = sha256_hex(&bytes);
    let declared = required_string(record, "/sha256")?;
    if actual != declared {
        return Err(format!(
            "{label} digest mismatch for {declared_path}: declared {declared}, actual {actual}"
        ));
    }
    Ok(actual)
}

fn verify_receipt_files(receipt: &Value, evidence_root: &Path) -> Result<(), String> {
    let artifacts = receipt
        .get("artifacts")
        .and_then(Value::as_array)
        .ok_or_else(|| "receipt artifacts are missing or not an array".to_string())?;
    let mut artifact_digests = BTreeSet::new();
    for (index, artifact) in artifacts.iter().enumerate() {
        let digest =
            verify_local_hashed_file(evidence_root, artifact, &format!("artifact[{index}]"))?;
        artifact_digests.insert(digest);
    }

    let fault_schedule_digest = required_string(receipt, "/environment/fault_schedule_sha256")?;
    if !artifact_digests.contains(fault_schedule_digest) {
        return Err(
            "fault_schedule_sha256 must identify one of the locally verified receipt artifacts"
                .into(),
        );
    }

    let executions = receipt
        .get("executions")
        .and_then(Value::as_array)
        .ok_or_else(|| "receipt executions are missing or not an array".to_string())?;
    for (execution_index, execution) in executions.iter().enumerate() {
        let scenario_id = required_string(execution, "/scenario_id")?;
        let evidence = execution
            .get("evidence")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("execution {scenario_id} has no evidence array"))?;
        for (evidence_index, item) in evidence.iter().enumerate() {
            verify_local_hashed_file(
                evidence_root,
                item,
                &format!("execution[{execution_index}] {scenario_id} evidence[{evidence_index}]"),
            )?;
        }
    }
    Ok(())
}

fn checks_for_execution(execution: &Value, evidence: &BTreeSet<String>) -> Result<(), String> {
    let checks = execution
        .get("checks")
        .and_then(Value::as_array)
        .ok_or_else(|| "execution checks are missing or not an array".to_string())?;
    for check in checks {
        let name = required_string(check, "/name")?;
        let references = check
            .get("evidence_sha256")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("check {name} has no evidence_sha256 array"))?;
        if references.is_empty() {
            return Err(format!("check {name} has no evidence references"));
        }
        for reference in references {
            let digest = reference
                .as_str()
                .ok_or_else(|| format!("check {name} has a non-string evidence reference"))?;
            if !evidence.contains(digest) {
                return Err(format!(
                    "check {name} references evidence digest {digest} not present in this execution"
                ));
            }
        }
    }
    Ok(())
}

/// Evaluate only the selected target. Hard-gate failures veto qualification;
/// missing, skipped, unknown, not-applicable, dirty-source, or evidence-free
/// hard gates are INCOMPLETE, never PASS.
pub fn evaluate_receipt(
    corpus_text: &str,
    contract_schema_text: &str,
    receipt_schema_text: &str,
    receipt_text: &str,
    target: &str,
    evidence_root: &Path,
) -> Result<Value, String> {
    let summary = validate_contract(corpus_text, contract_schema_text)?;
    let corpus = parse_json(corpus_text, "scenario corpus")?;
    let receipt_schema = parse_json(receipt_schema_text, "receipt schema")?;
    let receipt = parse_json(receipt_text, "qualification receipt")?;

    validate_schema_document(&receipt_schema, "receipt schema")?;
    validate_instance(&receipt_schema, &receipt, "qualification receipt")?;
    verify_receipt_files(&receipt, evidence_root)?;

    if required_string(&receipt, "/contract_id")? != summary.contract_id {
        return Err("receipt contract_id does not match the scenario corpus".into());
    }
    if required_string(&receipt, "/contract_version")? != summary.version {
        return Err("receipt contract_version does not match the scenario corpus".into());
    }
    if required_string(&receipt, "/corpus_sha256")? != summary.corpus_sha256 {
        return Err("receipt corpus_sha256 does not match the exact corpus bytes".into());
    }
    if required_string(&receipt, "/schema_sha256")? != summary.schema_sha256 {
        return Err("receipt schema_sha256 does not match the exact scenario schema bytes".into());
    }
    let receipt_target = required_string(&receipt, "/environment/target")?;
    if receipt_target != target {
        return Err(format!(
            "receipt target {receipt_target} does not match requested target {target}"
        ));
    }

    let scenarios = corpus
        .get("scenarios")
        .and_then(Value::as_array)
        .ok_or_else(|| "scenario corpus has no scenarios array".to_string())?;
    let mut scoped_scenarios = BTreeMap::<String, String>::new();
    for scenario in scenarios {
        let id = required_string(scenario, "/id")?.to_string();
        let class = required_string(scenario, "/class")?.to_string();
        let applies_to_target = scenario
            .get("targets")
            .and_then(Value::as_array)
            .is_some_and(|targets| targets.iter().any(|item| item.as_str() == Some(target)));
        if applies_to_target {
            scoped_scenarios.insert(id, class);
        }
    }
    if scoped_scenarios.is_empty() {
        return Ok(json!({
            "contract_id": summary.contract_id,
            "target": target,
            "status": "INCOMPLETE",
            "reason": "no scenarios target this adapter; absence of coverage is not qualification",
            "release_qualified": false,
            "hard_gate": {"passed": 0, "failed": 0, "incomplete": 0},
            "coverage": {"in_scope": 0, "passed": 0, "failed": 0, "incomplete": 0},
            "performance_baseline": {"in_scope": 0, "passed": 0, "failed": 0, "incomplete": 0},
            "missing_scenarios": []
        }));
    }

    let mut executions_by_id = BTreeMap::<String, &Value>::new();
    let executions = receipt
        .get("executions")
        .and_then(Value::as_array)
        .ok_or_else(|| "receipt has no executions array".to_string())?;
    for execution in executions {
        let id = required_string(execution, "/scenario_id")?.to_string();
        let execution_target = required_string(execution, "/target")?;
        if execution_target != target {
            return Err(format!(
                "execution {id} target {execution_target} differs from selected target {target}"
            ));
        }
        if !scoped_scenarios.contains_key(&id) {
            return Err(format!(
                "receipt includes scenario {id}, which is not declared for target {target}"
            ));
        }
        if executions_by_id.insert(id.clone(), execution).is_some() {
            return Err(format!(
                "duplicate receipt execution for scenario {id} and target {target}"
            ));
        }

        let applicability = required_string(execution, "/applicability")?;
        if applicability == "applicable" {
            let evidence = evidence_digest_set(execution);
            checks_for_execution(execution, &evidence)?;
        }
    }

    let tree_state = required_string(&receipt, "/source/tree_state")?;
    let source_is_clean = tree_state == "clean";
    let mut hard_gate_passed = 0usize;
    let mut hard_gate_failed = 0usize;
    let mut hard_gate_incomplete = 0usize;
    let mut coverage_passed = 0usize;
    let mut coverage_failed = 0usize;
    let mut coverage_incomplete = 0usize;
    let mut performance_passed = 0usize;
    let mut performance_failed = 0usize;
    let mut performance_incomplete = 0usize;
    let mut missing_scenarios = Vec::<String>::new();
    let mut scenario_results = Vec::<Value>::new();

    for (id, class) in &scoped_scenarios {
        let Some(execution) = executions_by_id.get(id).copied() else {
            missing_scenarios.push(id.clone());
            match class.as_str() {
                "hard_gate" => hard_gate_incomplete += 1,
                "performance_baseline" => performance_incomplete += 1,
                _ => coverage_incomplete += 1,
            }
            scenario_results.push(json!({
                "scenario_id": id,
                "class": class,
                "status": "missing_execution"
            }));
            continue;
        };

        let applicability = required_string(execution, "/applicability")?;
        let outcome = required_string(execution, "/outcome")?;
        let evidence = evidence_digest_set(execution);
        let checks = execution
            .get("checks")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("scenario {id} checks are missing"))?;
        let has_check_failure = checks
            .iter()
            .any(|check| check.get("outcome").and_then(Value::as_str) == Some("failed"));
        let has_unknown_check = checks
            .iter()
            .any(|check| check.get("outcome").and_then(Value::as_str) == Some("unknown"));
        let all_checks_passed = !checks.is_empty()
            && checks
                .iter()
                .all(|check| check.get("outcome").and_then(Value::as_str) == Some("passed"));
        let is_performance = class == "performance_baseline";
        let performance_has_measurements = execution
            .get("measurements")
            .and_then(Value::as_object)
            .is_some_and(|measurements| !measurements.is_empty());

        // An observed failure must never be hidden by marking the scenario
        // not applicable. Contradictory reports are conservatively failures.
        let execution_status = if outcome == "failed" || has_check_failure {
            "failed"
        } else if applicability == "not_applicable" {
            "not_applicable"
        } else if !source_is_clean {
            "incomplete"
        } else if outcome == "passed"
            && all_checks_passed
            && !evidence.is_empty()
            && (!is_performance || performance_has_measurements)
        {
            "passed"
        } else if outcome == "unknown"
            || outcome == "skipped"
            || outcome == "not_run"
            || has_unknown_check
        {
            "incomplete"
        } else {
            "incomplete"
        };

        match class.as_str() {
            "hard_gate" => match execution_status {
                "passed" => hard_gate_passed += 1,
                "failed" => hard_gate_failed += 1,
                _ => hard_gate_incomplete += 1,
            },
            "performance_baseline" => match execution_status {
                "passed" => performance_passed += 1,
                "failed" => performance_failed += 1,
                _ => performance_incomplete += 1,
            },
            _ => match execution_status {
                "passed" => coverage_passed += 1,
                "failed" => coverage_failed += 1,
                _ => coverage_incomplete += 1,
            },
        }
        scenario_results.push(json!({
            "scenario_id": id,
            "class": class,
            "status": execution_status,
            "outcome": outcome,
            "applicability": applicability,
            "evidence_count": evidence.len(),
            "check_count": checks.len()
        }));
    }

    let safety_status = if hard_gate_failed > 0 {
        "FAILED"
    } else if hard_gate_incomplete > 0 || !source_is_clean {
        "INCOMPLETE"
    } else {
        "SAFETY_GATES_PASSED"
    };
    let all_scope_complete = coverage_incomplete == 0
        && coverage_failed == 0
        && performance_incomplete == 0
        && performance_failed == 0
        && missing_scenarios.is_empty();
    let release_qualified =
        safety_status == "SAFETY_GATES_PASSED" && all_scope_complete && source_is_clean;
    let status = if hard_gate_failed > 0 || coverage_failed > 0 || performance_failed > 0 {
        QualificationExit::Failed
    } else if release_qualified {
        QualificationExit::Qualified
    } else {
        QualificationExit::Incomplete
    };

    Ok(json!({
        "contract_id": summary.contract_id,
        "contract_version": summary.version,
        "target": target,
        "status": status.as_str(),
        "safety_gate_status": safety_status,
        "release_qualified": release_qualified,
        "source_tree_clean": source_is_clean,
        "corpus_sha256": summary.corpus_sha256,
        "schema_sha256": summary.schema_sha256,
        "hard_gate": {
            "passed": hard_gate_passed,
            "failed": hard_gate_failed,
            "incomplete": hard_gate_incomplete
        },
        "coverage": {
            "in_scope": coverage_passed + coverage_failed + coverage_incomplete,
            "passed": coverage_passed,
            "failed": coverage_failed,
            "incomplete": coverage_incomplete
        },
        "performance_baseline": {
            "in_scope": performance_passed + performance_failed + performance_incomplete,
            "passed": performance_passed,
            "failed": performance_failed,
            "incomplete": performance_incomplete
        },
        "missing_scenarios": missing_scenarios,
        "scenario_results": scenario_results
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    const CORPUS: &str = include_str!("../../../docs/research/civos-multi-agent-scenarios-v0.json");
    const CONTRACT_SCHEMA: &str =
        include_str!("../../../docs/research/civos-multi-agent-scenarios-v0.schema.json");
    const RECEIPT_SCHEMA: &str =
        include_str!("../../../docs/research/civos-multi-agent-receipt-v0.schema.json");
    const SYNTHETIC_EVIDENCE: &[u8] = include_bytes!("../fixtures/synthetic-evidence.log");

    fn valid_receipt(target: &str) -> Value {
        let corpus: Value = serde_json::from_str(CORPUS).unwrap();
        let digest = sha256_hex(SYNTHETIC_EVIDENCE);
        let evidence_path = "fixtures/synthetic-evidence.log";
        let executions: Vec<Value> = corpus["scenarios"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|scenario| {
                scenario["targets"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .any(|item| item.as_str() == Some(target))
            })
            .map(|scenario| {
                let id = scenario["id"].as_str().unwrap();
                let mut execution = json!({
                    "scenario_id": id,
                    "target": target,
                    "applicability": "applicable",
                    "outcome": "passed",
                    "evidence": [{"kind":"test_log","sha256":digest,"path":evidence_path}],
                    "checks": [{
                        "name":"synthetic contract fixture",
                        "outcome":"passed",
                        "evidence_sha256":[digest]
                    }]
                });
                if scenario["class"] == "performance_baseline" {
                    execution["measurements"] = json!({"p95_ms": 10.0, "rss_bytes": 1024});
                }
                execution
            })
            .collect();
        json!({
            "receipt_id":"synthetic-receipt-001",
            "contract_id":"CIVOS-MACQ-001",
            "contract_version":"0.1.0",
            "corpus_sha256":sha256_hex(CORPUS.as_bytes()),
            "schema_sha256":sha256_hex(CONTRACT_SCHEMA.as_bytes()),
            "source":{
                "repository":"Luminous-Dynamics/mycelix",
                "commit":"b".repeat(40),
                "tree_state":"clean"
            },
            "runtime":{
                "holochain":"0.7.0",
                "hdk":"0.7.0",
                "hdi":"0.8.0",
                "conductor":"synthetic-runtime",
                "client":"synthetic-client",
                "harness":"synthetic-harness"
            },
            "artifacts":[{"kind":"fixture","sha256":digest,"path":evidence_path}],
            "environment":{
                "target":target,
                "peer_count":3,
                "seed":"fixed-seed-001",
                "fault_schedule_sha256":digest,
                "timeout_policy":"deterministic bounded timeout"
            },
            "executions":executions
        })
    }

    fn evaluate(receipt: &Value) -> Result<Value, String> {
        evaluate_receipt(
            CORPUS,
            CONTRACT_SCHEMA,
            RECEIPT_SCHEMA,
            &receipt.to_string(),
            "finance",
            Path::new(env!("CARGO_MANIFEST_DIR")),
        )
    }

    #[test]
    fn detached_signature_authenticates_exact_receipt_bytes_and_pinned_key() {
        use ed25519_dalek::{Signer, SigningKey};

        let receipt = br#"{"receipt_id":"synthetic-receipt-001","status":"PASS"}"#;
        let signing_key = SigningKey::from_bytes(&[43u8; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let fingerprint = sha256_hex(&public_key);
        let signature = signing_key.sign(&receipt_signature_message(receipt));
        let signature_base64 = STANDARD.encode(signature.to_bytes());

        assert_eq!(
            verify_receipt_signature(receipt, &signature_base64, &public_key, &fingerprint)
                .unwrap(),
            fingerprint
        );
    }

    #[test]
    fn detached_signature_rejects_receipt_byte_changes_and_wrong_fingerprint() {
        use ed25519_dalek::{Signer, SigningKey};

        let receipt = br#"{"receipt_id":"synthetic-receipt-001","status":"PASS"}"#;
        let signing_key = SigningKey::from_bytes(&[44u8; 32]);
        let public_key = signing_key.verifying_key().to_bytes();
        let fingerprint = sha256_hex(&public_key);
        let signature = signing_key.sign(&receipt_signature_message(receipt));
        let signature_base64 = STANDARD.encode(signature.to_bytes());

        let changed_receipt = br#"{ "receipt_id":"synthetic-receipt-001","status":"PASS"}"#;
        assert!(
            verify_receipt_signature(changed_receipt, &signature_base64, &public_key, &fingerprint)
                .unwrap_err()
                .contains("signature verification failed")
        );
        assert!(
            verify_receipt_signature(receipt, &signature_base64, &public_key, &"0".repeat(64))
                .unwrap_err()
                .contains("fingerprint mismatch")
        );
    }

    #[test]
    fn shipped_scenario_contract_and_draft_2020_12_schema_validate() {
        let summary = validate_contract(CORPUS, CONTRACT_SCHEMA).unwrap();
        assert_eq!(summary.contract_id, "CIVOS-MACQ-001");
        assert_eq!(summary.scenario_count, 21);
        assert!(summary.hard_gate_count > 0);
        assert_eq!(summary.corpus_sha256.len(), 64);
        assert_eq!(summary.schema_sha256.len(), 64);
    }

    #[test]
    fn duplicate_scenario_ids_are_rejected_even_when_schema_accepts_item_shape() {
        let mut corpus: Value = serde_json::from_str(CORPUS).unwrap();
        let first_id = corpus["scenarios"][0]["id"].clone();
        corpus["scenarios"][1]["id"] = first_id;
        assert!(
            validate_contract(&corpus.to_string(), CONTRACT_SCHEMA)
                .unwrap_err()
                .contains("duplicate scenario id")
        );
    }

    #[test]
    fn receipt_can_only_qualify_after_all_finance_hard_gates_have_evidence() {
        let report = evaluate(&valid_receipt("finance")).unwrap();
        assert_eq!(report["status"], "QUALIFIED");
        assert_eq!(report["safety_gate_status"], "SAFETY_GATES_PASSED");
        assert_eq!(report["release_qualified"], true);
    }

    #[test]
    fn missing_hard_gate_execution_is_incomplete_not_pass() {
        let mut receipt = valid_receipt("finance");
        let executions = receipt["executions"].as_array_mut().unwrap();
        let index = executions
            .iter()
            .position(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        executions.remove(index);
        let report = evaluate(&receipt).unwrap();
        assert_eq!(report["status"], "INCOMPLETE");
        assert_eq!(report["safety_gate_status"], "INCOMPLETE");
        assert_eq!(report["release_qualified"], false);
        assert!(
            report["missing_scenarios"]
                .as_array()
                .unwrap()
                .iter()
                .any(|id| id == "CIV-CAP-001")
        );
    }

    #[test]
    fn a_hard_gate_failure_vetoes_release_even_if_other_scenarios_are_missing() {
        let mut receipt = valid_receipt("finance");
        let executions = receipt["executions"].as_array_mut().unwrap();
        let entry = executions
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["outcome"] = json!("failed");
        let report = evaluate(&receipt).unwrap();
        assert_eq!(report["status"], "FAILED");
        assert_eq!(report["safety_gate_status"], "FAILED");
        assert_eq!(report["release_qualified"], false);
    }

    #[test]
    fn not_applicable_cannot_mask_an_observed_hard_gate_failure() {
        let mut receipt = valid_receipt("finance");
        let entry = receipt["executions"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["applicability"] = json!("not_applicable");
        entry["applicability_reason"] = json!("contradictory synthetic report");
        entry["outcome"] = json!("failed");
        let report = evaluate(&receipt).unwrap();
        assert_eq!(report["status"], "FAILED");
        assert_eq!(report["safety_gate_status"], "FAILED");
        assert_eq!(report["release_qualified"], false);
    }

    #[test]
    fn unknown_and_not_applicable_hard_gates_are_never_passes() {
        let mut unknown = valid_receipt("finance");
        let entry = unknown["executions"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["outcome"] = json!("unknown");
        assert_eq!(evaluate(&unknown).unwrap()["status"], "INCOMPLETE");

        let mut not_applicable = valid_receipt("finance");
        let entry = not_applicable["executions"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["applicability"] = json!("not_applicable");
        entry["applicability_reason"] = json!("synthetic coverage gap");
        assert_eq!(evaluate(&not_applicable).unwrap()["status"], "INCOMPLETE");
    }

    #[test]
    fn receipt_from_another_corpus_revision_is_rejected() {
        let mut receipt = valid_receipt("finance");
        receipt["corpus_sha256"] = json!("0".repeat(64));
        assert!(evaluate(&receipt).unwrap_err().contains("corpus_sha256"));
    }

    #[test]
    fn unlinked_check_evidence_is_rejected() {
        let mut receipt = valid_receipt("finance");
        let entry = receipt["executions"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["checks"][0]["evidence_sha256"] = json!(["c".repeat(64)]);
        assert!(
            evaluate(&receipt)
                .unwrap_err()
                .contains("not present in this execution")
        );
    }

    #[test]
    fn evidence_paths_cannot_escape_the_declared_root() {
        let mut receipt = valid_receipt("finance");
        let entry = receipt["executions"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|item| item["scenario_id"] == "CIV-CAP-001")
            .unwrap();
        entry["evidence"][0]["path"] = json!("../../Cargo.toml");
        assert!(
            evaluate(&receipt)
                .unwrap_err()
                .contains("safe relative path")
        );
    }

    #[test]
    fn receipt_rejects_a_digest_that_does_not_match_evidence_bytes() {
        let mut receipt = valid_receipt("finance");
        let invalid_digest = "0".repeat(64);
        receipt["artifacts"][0]["sha256"] = json!(invalid_digest.clone());
        receipt["environment"]["fault_schedule_sha256"] = json!(invalid_digest.clone());
        for execution in receipt["executions"].as_array_mut().unwrap() {
            execution["evidence"][0]["sha256"] = json!(invalid_digest.clone());
            for check in execution["checks"].as_array_mut().unwrap() {
                check["evidence_sha256"] = json!([invalid_digest.clone()]);
            }
        }
        assert!(evaluate(&receipt).unwrap_err().contains("digest mismatch"));
    }

    #[test]
    fn dirty_source_tree_is_never_qualified() {
        let mut receipt = valid_receipt("finance");
        receipt["source"]["tree_state"] = json!("dirty");
        let report = evaluate(&receipt).unwrap();
        assert_eq!(report["status"], "INCOMPLETE");
        assert_eq!(report["release_qualified"], false);
    }
}
