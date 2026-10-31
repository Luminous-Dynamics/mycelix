use cedar_policy::{Authorizer, Context, Decision, Entities, EntityUid, PolicySet, Request, Schema, ValidationMode, Validator};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha1::Sha1;
use sha2::{Digest, Sha256};
use scientific_policy_metamorphic::shrink::{
    minimize_json_with_provenance, PredicateProvenance, MAX_SOURCE_INPUT_BYTES,
};
use std::cell::Cell;
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode, Stdio};
use std::str::FromStr;
use std::time::Instant;

const EXPECTED_BASELINE_REVISION: &str = "f89b4bcdcfb901961e2ca11a577775acbf53a74d";
const EXPECTED_BASELINE_SOURCE_BLOB_SHA: &str = "dd9ea9267cdae2e2c67244e8a90821defaacefb3";
const EXPECTED_CASE_COUNT: usize = 62_464;
const MAX_INPUT_BYTES: u64 = 256 * 1024 * 1024;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct InputManifest {
    generator_id: String,
    generator_version: String,
    baseline_revision: String,
    baseline_source_path: String,
    baseline_source_blob_sha: String,
    policy_id: String,
    policy_version: String,
    case_count: usize,
    empty_stream_cases: usize,
    existing_stream_cases: usize,
    distinct_role_subsets: usize,
    action_count: usize,
    expected_decision_status: String,
    claim_ceiling: String,
    corpus_file: String,
    corpus_sha256: String,
    engine_inputs_file: String,
    engine_inputs_sha256: String,
    engine_input_count: usize,
    engine_input_format: String,
    encoding: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct EngineInputCase {
    case_id: String,
    cedar: CedarCase,
    rego: RegoCase,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CedarCase {
    case_id: String,
    request: CedarRequest,
    entities: Vec<Value>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CedarRequest {
    principal: String,
    action: String,
    resource: String,
    context: Value,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RegoCase {
    case_id: String,
    input: Value,
}

#[derive(Debug, Serialize)]
struct DecisionRecord<'a> {
    case_id: &'a str,
    engine: &'static str,
    decision: &'static str,
    determining_policy_ids: &'a [String],
}

#[derive(Debug, Serialize)]
struct TimingSummary {
    count: usize,
    p50_ns: u128,
    p95_ns: u128,
    p99_ns: u128,
    max_ns: u128,
    mean_ns: u128,
}

#[derive(Debug, Serialize)]
struct RunManifest {
    engine: &'static str,
    engine_version: &'static str,
    evaluation_method: &'static str,
    generator_id: String,
    generator_version: String,
    baseline_revision: String,
    baseline_source_path: String,
    baseline_source_blob_sha: String,
    policy_id: String,
    policy_version: String,
    input_file: String,
    input_sha256: String,
    policy_sha256: String,
    schema_sha256: String,
    output_file: String,
    output_sha256: String,
    case_count: usize,
    allow_count: usize,
    deny_count: usize,
    indeterminate_count: usize,
    warmup_cases: usize,
    measured_cases: usize,
    construction_latency: TimingSummary,
    authorization_latency: TimingSummary,
    end_to_end_latency: TimingSummary,
    total_wall_ns: u128,
    throughput_cases_per_second: f64,
    peak_rss_kib: Option<u64>,
    operating_system: &'static str,
    architecture: &'static str,
    logical_cpus: usize,
    claim_ceiling: &'static str,
}

fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

fn length_prefixed_material(parts: &[&[u8]]) -> Vec<u8> {
    let mut output = Vec::new();
    for part in parts {
        output.extend_from_slice(&(part.len() as u64).to_be_bytes());
        output.extend_from_slice(part);
    }
    output
}

fn git_blob_oid(bytes: &[u8]) -> String {
    let mut hasher = Sha1::new();
    hasher.update(format!("blob {}\0", bytes.len()).as_bytes());
    hasher.update(bytes);
    hex::encode(hasher.finalize())
}

fn verify_exact_file(path: &Path, expected_sha256: &str) -> Result<(), String> {
    let bytes = fs::read(path).map_err(|error| format!("read {}: {error}", path.display()))?;
    if sha256(&bytes) != expected_sha256 {
        return Err(format!("SHA-256 mismatch for {}", path.display()));
    }
    Ok(())
}

fn percentile(values: &[u128], percentile: usize) -> u128 {
    if values.is_empty() {
        return 0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    let rank = (percentile * sorted.len()).div_ceil(100).max(1);
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn timing_summary(values: &[u128]) -> TimingSummary {
    let sum = values.iter().copied().sum::<u128>();
    TimingSummary {
        count: values.len(),
        p50_ns: percentile(values, 50),
        p95_ns: percentile(values, 95),
        p99_ns: percentile(values, 99),
        max_ns: values.iter().copied().max().unwrap_or(0),
        mean_ns: if values.is_empty() { 0 } else { sum / values.len() as u128 },
    }
}

fn peak_rss_kib() -> Option<u64> {
    let status = fs::read_to_string("/proc/self/status").ok()?;
    status.lines().find_map(|line| {
        let rest = line.strip_prefix("VmHWM:")?;
        rest.split_whitespace().next()?.parse().ok()
    })
}

fn parse_entity_uid_text(value: &str, label: &str) -> Result<(String, String), String> {
    let (entity_type, id_json) = value
        .rsplit_once("::")
        .ok_or_else(|| format!("{label} is not a qualified Cedar entity UID"))?;
    if entity_type.is_empty() {
        return Err(format!("{label} has an empty entity type"));
    }
    let entity_id: String = serde_json::from_str(id_json)
        .map_err(|error| format!("{label} entity ID is not a JSON string: {error}"))?;
    if entity_id.is_empty() {
        return Err(format!("{label} entity ID is empty"));
    }
    Ok((entity_type.to_owned(), entity_id))
}

fn validate_explicit_entities(case: &EngineInputCase, line_index: usize) -> Result<(), String> {
    let principal = parse_entity_uid_text(
        &case.cedar.request.principal,
        &format!("Cedar principal at line {}", line_index + 1),
    )?;
    let resource = parse_entity_uid_text(
        &case.cedar.request.resource,
        &format!("Cedar resource at line {}", line_index + 1),
    )?;
    if principal.0 != "Mycelix::Actor" || resource.0 != "Mycelix::Claim" {
        return Err(format!(
            "Cedar request at line {} must reference Mycelix::Actor principal and Mycelix::Claim resource",
            line_index + 1
        ));
    }

    let mut uids = BTreeSet::new();
    let mut actor_count = 0usize;
    let mut claim_count = 0usize;
    let mut contains_principal = false;
    let mut contains_resource = false;
    for entity in &case.cedar.entities {
        let uid = entity
            .get("uid")
            .and_then(Value::as_object)
            .ok_or_else(|| format!("Cedar entity uid must be an object at line {}", line_index + 1))?;
        let entity_type = uid
            .get("type")
            .and_then(Value::as_str)
            .filter(|value| !value.is_empty())
            .ok_or_else(|| format!("Cedar entity type is missing at line {}", line_index + 1))?;
        let entity_id = uid
            .get("id")
            .and_then(Value::as_str)
            .filter(|value| !value.is_empty())
            .ok_or_else(|| format!("Cedar entity ID is missing at line {}", line_index + 1))?;
        if !matches!(entity_type, "Mycelix::Actor" | "Mycelix::Claim") {
            return Err(format!(
                "unexpected explicit Cedar entity type {entity_type} at line {}",
                line_index + 1
            ));
        }
        let key = (entity_type.to_owned(), entity_id.to_owned());
        if !uids.insert(key.clone()) {
            return Err(format!("duplicate Cedar entity UID at line {}", line_index + 1));
        }
        contains_principal |= key == principal;
        contains_resource |= key == resource;
        match entity_type {
            "Mycelix::Actor" => actor_count += 1,
            "Mycelix::Claim" => claim_count += 1,
            _ => unreachable!(),
        }
        if !entity.get("attrs").is_some_and(Value::is_object)
            || !entity.get("parents").is_some_and(Value::is_array)
        {
            return Err(format!(
                "Cedar entity requires object attrs and array parents at line {}",
                line_index + 1
            ));
        }
    }
    if actor_count == 0 || claim_count != 1 || !contains_principal || !contains_resource {
        return Err(format!(
            "Cedar explicit entity set at line {} must include the referenced principal Actor and exactly one referenced Claim; extra unique Actor entities are permitted",
            line_index + 1
        ));
    }
    Ok(())
}

fn validate_manifest(manifest: &InputManifest) -> Result<(), String> {
    if manifest.generator_id != "mycelix-scientific-policy-space"
        || manifest.generator_version != "0.1.0"
        || manifest.baseline_revision != EXPECTED_BASELINE_REVISION
        || manifest.baseline_source_path
            != "mycelix-desci/src/core/src/scientific_governance.rs"
        || manifest.baseline_source_blob_sha != EXPECTED_BASELINE_SOURCE_BLOB_SHA
        || manifest.policy_id != "mycelix-default-scientific-authorization"
        || manifest.policy_version != "1.0.0"
        || manifest.case_count != EXPECTED_CASE_COUNT
        || manifest.empty_stream_cases != 1_024
        || manifest.existing_stream_cases != 61_440
        || manifest.distinct_role_subsets != 128
        || manifest.action_count != 8
        || manifest.expected_decision_status != "pending_authentic_rust_oracle"
        || manifest.corpus_file != "scientific-policy-decision-space-v0.1.jsonl"
        || manifest.engine_inputs_file != "scientific-policy-engine-inputs-v0.1.jsonl"
        || manifest.engine_input_count != EXPECTED_CASE_COUNT
        || manifest.engine_input_format != "case_id+cedar_native_request+rego_input"
        || manifest.encoding != "utf-8-jsonl-one-record-per-line"
    {
        return Err("input manifest does not match the pinned v0.1 corpus contract".to_owned());
    }
    for (label, digest) in [
        ("abstract corpus", manifest.corpus_sha256.as_str()),
        ("engine input corpus", manifest.engine_inputs_sha256.as_str()),
    ] {
        if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
            return Err(format!("{label} manifest digest is malformed"));
        }
    }
    if manifest.claim_ceiling.is_empty() {
        return Err("abstract corpus claim ceiling is missing".to_owned());
    }
    Ok(())
}

fn load_cases(path: &Path, expected_sha256: &str) -> Result<Vec<EngineInputCase>, String> {
    let metadata = fs::metadata(path).map_err(|error| format!("stat engine input corpus: {error}"))?;
    if metadata.len() > MAX_INPUT_BYTES {
        return Err(format!("engine input corpus exceeds {MAX_INPUT_BYTES} byte cap"));
    }
    let bytes = fs::read(path).map_err(|error| format!("read engine input corpus: {error}"))?;
    if sha256(&bytes) != expected_sha256 {
        return Err("engine input corpus SHA-256 does not match its manifest".to_owned());
    }
    let mut cases = Vec::with_capacity(EXPECTED_CASE_COUNT);
    let mut ids = BTreeSet::new();
    for (line_index, line) in BufReader::new(bytes.as_slice()).lines().enumerate() {
        let line = line.map_err(|error| format!("read JSONL line {}: {error}", line_index + 1))?;
        if line.trim().is_empty() {
            return Err(format!("blank JSONL record at line {}", line_index + 1));
        }
        let case: EngineInputCase = serde_json::from_str(&line)
            .map_err(|error| format!("parse JSONL line {}: {error}", line_index + 1))?;
        if case.case_id != case.cedar.case_id || case.case_id != case.rego.case_id {
            return Err(format!("cross-engine case ID mismatch at line {}", line_index + 1));
        }
        if case.rego.input.get("action").is_none()
            || case.rego.input.get("actor").is_none()
            || case.rego.input.get("claim").is_none()
            || case.rego.input.get("attestation").is_none()
            || case.rego.input.get("targetAttestation").is_none()
        {
            return Err(format!("Rego input is missing a required source-policy field at line {}", line_index + 1));
        }
        validate_explicit_entities(&case, line_index)?;
        if !case.cedar.request.context.is_object() {
            return Err(format!("Cedar request context must be a JSON record at line {}", line_index + 1));
        }
        if !ids.insert(case.case_id.clone()) {
            return Err(format!("duplicate case ID at line {}", line_index + 1));
        }
        cases.push(case);
    }
    if cases.len() != EXPECTED_CASE_COUNT {
        return Err(format!("expected {EXPECTED_CASE_COUNT} input rows, got {}", cases.len()));
    }
    Ok(cases)
}

fn evaluate_case(
    authorizer: &Authorizer,
    policy_set: &PolicySet,
    schema: &Schema,
    case: &EngineInputCase,
) -> Result<(&'static str, Vec<String>, u128, u128), String> {
    let construction_start = Instant::now();
    let principal = EntityUid::from_str(&case.cedar.request.principal)
        .map_err(|error| format!("invalid Cedar principal for {}: {error:?}", case.case_id))?;
    let action = EntityUid::from_str(&case.cedar.request.action)
        .map_err(|error| format!("invalid Cedar action for {}: {error:?}", case.case_id))?;
    let resource = EntityUid::from_str(&case.cedar.request.resource)
        .map_err(|error| format!("invalid Cedar resource for {}: {error:?}", case.case_id))?;
    let entities_json = Value::Array(case.cedar.entities.clone());
    let entities = Entities::from_json_value(entities_json, Some(schema))
        .map_err(|error| format!("invalid Cedar entities for {}: {error:?}", case.case_id))?;
    let context = Context::from_json_value(
        case.cedar.request.context.clone(),
        Some((schema, &action)),
    )
    .map_err(|error| format!("invalid Cedar context for {}: {error:?}", case.case_id))?;
    let request = Request::new(principal, action, resource, context, Some(schema))
        .map_err(|error| format!("invalid Cedar request for {}: {error:?}", case.case_id))?;
    let construction_ns = construction_start.elapsed().as_nanos();

    let authorization_start = Instant::now();
    let response = authorizer.is_authorized(&request, policy_set, &entities);
    let authorization_ns = authorization_start.elapsed().as_nanos();
    if let Some(error) = response.diagnostics().errors().next() {
        return Err(format!(
            "Cedar evaluation diagnostic for {} (not a deny): {error:?}",
            case.case_id
        ));
    }

    let decision = match response.decision() {
        Decision::Allow => "allow",
        Decision::Deny => "deny",
    };
    let mut determining_policy_ids = response
        .diagnostics()
        .reason()
        .map(ToString::to_string)
        .collect::<Vec<_>>();
    determining_policy_ids.sort();
    Ok((decision, determining_policy_ids, construction_ns, authorization_ns))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn git_blob_oid_matches_the_empty_blob_known_answer() {
        assert_eq!(
            git_blob_oid(b""),
            "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391"
        );
    }

    #[test]
    fn timing_percentiles_use_nearest_rank() {
        let values = [10_u128, 20, 30, 40, 50];
        assert_eq!(percentile(&values, 50), 30);
        assert_eq!(percentile(&values, 95), 50);
        assert_eq!(percentile(&values, 99), 50);
    }

    #[test]
    fn explicit_entity_set_allows_unique_unreferenced_actor_but_preserves_request_bindings() {
        let mut case = EngineInputCase {
            case_id: "case-1".to_owned(),
            cedar: CedarCase {
                case_id: "case-1".to_owned(),
                request: CedarRequest {
                    principal: "Mycelix::Actor::\"actor:candidate\"".to_owned(),
                    action: "Mycelix::Action::\"evidence_attached\"".to_owned(),
                    resource: "Mycelix::Claim::\"claim:1\"".to_owned(),
                    context: serde_json::json!({"attestationKind":"ordinary"}),
                },
                entities: vec![
                    serde_json::json!({
                        "uid":{"type":"Mycelix::Actor","id":"actor:candidate"},
                        "attrs":{"actorId":"actor:candidate","roles":["contributor"],"organizations":[]},
                        "parents":[]
                    }),
                    serde_json::json!({
                        "uid":{"type":"Mycelix::Claim","id":"claim:1"},
                        "attrs":{"exists":true},
                        "parents":[]
                    }),
                ],
            },
            rego: RegoCase {
                case_id: "case-1".to_owned(),
                input: serde_json::json!({"action":"evidence_attached"}),
            },
        };
        assert!(validate_explicit_entities(&case, 0).is_ok());

        case.cedar.entities.push(serde_json::json!({
            "uid":{"type":"Mycelix::Actor","id":"actor:metamorphic-irrelevant"},
            "attrs":{"actorId":"actor:metamorphic-irrelevant","roles":[],"organizations":[]},
            "parents":[]
        }));
        assert!(validate_explicit_entities(&case, 0).is_ok());

        let duplicate = case.cedar.entities[2].clone();
        case.cedar.entities.push(duplicate);
        assert!(validate_explicit_entities(&case, 0).is_err());
    }

    #[test]
    fn explicit_entity_set_rejects_missing_referenced_principal_or_resource() {
        let case = EngineInputCase {
            case_id: "case-2".to_owned(),
            cedar: CedarCase {
                case_id: "case-2".to_owned(),
                request: CedarRequest {
                    principal: "Mycelix::Actor::\"actor:missing\"".to_owned(),
                    action: "Mycelix::Action::\"evidence_attached\"".to_owned(),
                    resource: "Mycelix::Claim::\"claim:2\"".to_owned(),
                    context: serde_json::json!({}),
                },
                entities: vec![serde_json::json!({
                    "uid":{"type":"Mycelix::Claim","id":"claim:2"},
                    "attrs":{"exists":true},
                    "parents":[]
                })],
            },
            rego: RegoCase {
                case_id: "case-2".to_owned(),
                input: serde_json::json!({"action":"evidence_attached"}),
            },
        };
        assert!(validate_explicit_entities(&case, 0).is_err());
    }

    #[test]
    fn input_manifest_rejects_fabricated_expected_decision_status() {
        let manifest = InputManifest {
            generator_id: "mycelix-scientific-policy-space".to_owned(),
            generator_version: "0.1.0".to_owned(),
            baseline_revision: EXPECTED_BASELINE_REVISION.to_owned(),
            baseline_source_path: "mycelix-desci/src/core/src/scientific_governance.rs".to_owned(),
            baseline_source_blob_sha: EXPECTED_BASELINE_SOURCE_BLOB_SHA.to_owned(),
            policy_id: "mycelix-default-scientific-authorization".to_owned(),
            policy_version: "1.0.0".to_owned(),
            case_count: EXPECTED_CASE_COUNT,
            empty_stream_cases: 1_024,
            existing_stream_cases: 61_440,
            distinct_role_subsets: 128,
            action_count: 8,
            expected_decision_status: "pending_authentic_rust_oracle".to_owned(),
            claim_ceiling: "bounded abstract input corpus".to_owned(),
            corpus_file: "scientific-policy-decision-space-v0.1.jsonl".to_owned(),
            corpus_sha256: "a".repeat(64),
            engine_inputs_file: "scientific-policy-engine-inputs-v0.1.jsonl".to_owned(),
            engine_inputs_sha256: "b".repeat(64),
            engine_input_count: EXPECTED_CASE_COUNT,
            engine_input_format: "case_id+cedar_native_request+rego_input".to_owned(),
            encoding: "utf-8-jsonl-one-record-per-line".to_owned(),
        };
        assert!(validate_manifest(&manifest).is_ok());
        let mut changed = manifest;
        changed.expected_decision_status = "expected_allow_deny_attached".to_owned();
        assert!(validate_manifest(&changed).is_err());
    }
}

#[derive(Debug, Clone, Serialize)]
struct ShrinkPairSummary {
    predicate: String,
    evaluation_method: String,
    initial_case_id: String,
    initial_cedar_decision: String,
    initial_opa_decision: String,
    minimized_case_id: String,
    minimized_cedar_decision: String,
    minimized_opa_decision: String,
    candidate_evaluations: usize,
    opa_candidate_process_evaluations: usize,
    cedar_candidate_latency: TimingSummary,
    opa_cli_candidate_latency: TimingSummary,
    minimization_wall_ns: u128,
    accepted_reductions: usize,
    complete: bool,
    is_one_minimal: bool,
    claim_ceiling: String,
}

fn opa_decision(
    opa_binary: &Path,
    rego_policy_path: &Path,
    input: &Value,
) -> Result<(String, String, u128), String> {
    let started = Instant::now();
    let input_bytes = serde_json::to_vec(input)
        .map_err(|error| format!("serialize OPA candidate input: {error}"))?;
    let mut child = Command::new(opa_binary)
        .arg("eval")
        .arg("--strict")
        .arg("--timeout=5s")
        .arg("--format=json")
        .arg("--stdin-input")
        .arg("--data")
        .arg(rego_policy_path)
        .arg("data.mycelix.authz.decision")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("start OPA candidate evaluation: {error}"))?;
    child
        .stdin
        .take()
        .ok_or_else(|| "OPA stdin pipe was unavailable".to_owned())?
        .write_all(&input_bytes)
        .map_err(|error| format!("write OPA candidate input: {error}"))?;
    let output = child
        .wait_with_output()
        .map_err(|error| format!("wait for OPA candidate evaluation: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "OPA candidate evaluation exited {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    let response: Value = serde_json::from_slice(&output.stdout)
        .map_err(|error| format!("parse OPA candidate response: {error}"))?;
    let value = response
        .pointer("/result/0/expressions/0/value")
        .ok_or_else(|| "OPA response omitted the decision value".to_owned())?;
    let effect = value
        .get("effect")
        .and_then(Value::as_str)
        .ok_or_else(|| "OPA decision omitted effect".to_owned())?;
    let rule = value
        .get("rule")
        .and_then(Value::as_str)
        .ok_or_else(|| "OPA decision omitted rule identifier".to_owned())?;
    if !matches!(effect, "allow" | "deny") {
        return Err(format!("OPA returned unsupported decision effect: {effect}"));
    }
    Ok((effect.to_owned(), rule.to_owned(), started.elapsed().as_nanos()))
}

fn evaluate_policy_pair(
    candidate: &Value,
    authorizer: &Authorizer,
    policy_set: &PolicySet,
    schema: &Schema,
    opa_binary: &Path,
    rego_policy_path: &Path,
) -> Result<(String, String, Vec<String>, u128, u128), String> {
    let case: EngineInputCase = match serde_json::from_value(candidate.clone()) {
        Ok(case) => case,
        Err(_) => return Ok(("invalid".to_owned(), "invalid".to_owned(), Vec::new(), 0, 0)),
    };
    if case.case_id != case.cedar.case_id || case.case_id != case.rego.case_id {
        return Ok(("invalid".to_owned(), "invalid".to_owned(), Vec::new(), 0, 0));
    }
    if validate_explicit_entities(&case, 0).is_err()
        || !case.cedar.request.context.is_object()
        || !case.rego.input.is_object()
    {
        return Ok(("invalid".to_owned(), "invalid".to_owned(), Vec::new(), 0, 0));
    }
    let (cedar_decision, cedar_policy_ids, construction_ns, authorization_ns) =
        match evaluate_case(authorizer, policy_set, schema, &case) {
            Ok(result) => result,
            Err(message) if message.starts_with("Cedar evaluation diagnostic") => {
                return Err(message);
            }
            Err(_) => return Ok(("invalid".to_owned(), "invalid".to_owned(), Vec::new(), 0, 0)),
        };
    let (opa_effect, _opa_rule, opa_elapsed_ns) =
        opa_decision(opa_binary, rego_policy_path, &case.rego.input)?;
    Ok((
        cedar_decision.to_owned(),
        opa_effect,
        cedar_policy_ids,
        construction_ns + authorization_ns,
        opa_elapsed_ns,
    ))
}

fn run_shrink_pair(args: &[std::ffi::OsString]) -> Result<(), String> {
    if args.len() != 7 {
        return Err(
            "usage: --shrink-pair <candidate.json> <opa-binary> <rego-policy.rego> <cedar-policy.cedar> <cedar-schema.json> <output-dir> <max-candidates>"
                .to_owned(),
        );
    }
    let input_path = PathBuf::from(&args[0]);
    let opa_binary = PathBuf::from(&args[1]);
    let rego_policy_path = PathBuf::from(&args[2]);
    let cedar_policy_path = PathBuf::from(&args[3]);
    let cedar_schema_path = PathBuf::from(&args[4]);
    let output_directory = PathBuf::from(&args[5]);
    let max_candidates = args[6]
        .to_str()
        .ok_or_else(|| "max-candidates must be UTF-8".to_owned())?
        .parse::<usize>()
        .map_err(|error| format!("invalid max-candidates: {error}"))?;
    if max_candidates == 0 || max_candidates > 2_000 {
        return Err("max-candidates must be between 1 and 2000".to_owned());
    }

    let source_bytes = fs::read(&input_path)
        .map_err(|error| format!("read original counterexample: {error}"))?;
    if source_bytes.len() > MAX_SOURCE_INPUT_BYTES {
        return Err(format!(
            "counterexample exceeds {MAX_SOURCE_INPUT_BYTES} byte shrinker limit"
        ));
    }
    let initial_value: Value = serde_json::from_slice(&source_bytes)
        .map_err(|error| format!("parse original counterexample: {error}"))?;
    let initial_case: EngineInputCase = serde_json::from_value(initial_value.clone())
        .map_err(|error| format!("parse original engine case: {error}"))?;

    let opa_version = Command::new(&opa_binary)
        .arg("version")
        .output()
        .map_err(|error| format!("read OPA version: {error}"))?;
    if !opa_version.status.success() {
        return Err(format!(
            "OPA version command failed: {}",
            String::from_utf8_lossy(&opa_version.stderr)
        ));
    }
    let opa_version_text = String::from_utf8_lossy(&opa_version.stdout).into_owned();
    if !opa_version_text.contains("1.22.0") {
        return Err(format!("expected OPA 1.22.0, got {opa_version_text:?}"));
    }

    let cedar_policy_bytes = fs::read(&cedar_policy_path)
        .map_err(|error| format!("read Cedar policy: {error}"))?;
    let cedar_schema_bytes = fs::read(&cedar_schema_path)
        .map_err(|error| format!("read Cedar schema: {error}"))?;
    let rego_policy_bytes = fs::read(&rego_policy_path)
        .map_err(|error| format!("read Rego policy: {error}"))?;
    let opa_binary_bytes = fs::read(&opa_binary)
        .map_err(|error| format!("read OPA binary for provenance: {error}"))?;
    let runner_binary_path = env::current_exe()
        .map_err(|error| format!("resolve Cedar runner binary: {error}"))?;
    let runner_binary_bytes = fs::read(&runner_binary_path)
        .map_err(|error| format!("read Cedar runner binary for provenance: {error}"))?;

    let cedar_policy_source = std::str::from_utf8(&cedar_policy_bytes)
        .map_err(|error| format!("Cedar policy is not UTF-8: {error}"))?;
    let cedar_schema_source = std::str::from_utf8(&cedar_schema_bytes)
        .map_err(|error| format!("Cedar schema is not UTF-8: {error}"))?;
    let policy_set: PolicySet = cedar_policy_source
        .parse()
        .map_err(|error| format!("parse Cedar policy: {error:?}"))?;
    let schema = Schema::from_json_str(cedar_schema_source)
        .map_err(|error| format!("parse Cedar schema: {error:?}"))?;
    let validation = Validator::new(schema.clone()).validate(&policy_set, ValidationMode::Strict);
    if !validation.validation_passed() {
        return Err(format!("Cedar policy failed strict schema validation: {validation:?}"));
    }
    let authorizer = Authorizer::new();

    let (initial_cedar, initial_opa, _, _, _) = evaluate_policy_pair(
        &initial_value,
        &authorizer,
        &policy_set,
        &schema,
        &opa_binary,
        &rego_policy_path,
    )?;
    if !matches!(initial_cedar.as_str(), "allow" | "deny")
        || !matches!(initial_opa.as_str(), "allow" | "deny")
        || initial_cedar == initial_opa
    {
        return Err(format!(
            "input is not a valid Cedar/OPA decision counterexample: Cedar={initial_cedar}, OPA={initial_opa}"
        ));
    }

    let mut protected_paths = BTreeSet::new();
    for pointer in [
        "/case_id",
        "/cedar/case_id",
        "/cedar/request/principal",
        "/cedar/request/action",
        "/cedar/request/resource",
        "/cedar/entities",
        "/rego/case_id",
        "/rego/input/action",
        "/rego/input/actor/roles",
        "/rego/input/claim/exists",
    ] {
        protected_paths.insert(pointer.to_owned());
    }

    let engine_material =
        length_prefixed_material(&[&runner_binary_bytes, &opa_binary_bytes]);
    let policy_material =
        length_prefixed_material(&[&cedar_policy_bytes, &rego_policy_bytes]);
    let evaluation_config = serde_json::to_vec(&serde_json::json!({
        "cedar_sdk": "4.14.0",
        "opa_version": opa_version_text.trim(),
        "opa_query": "data.mycelix.authz.decision",
        "opa_flags": ["eval", "--strict", "--timeout=5s", "--format=json", "--stdin-input"],
        "predicate": "preserve_initial_cedar_opa_decision_divergence",
        "max_candidate_evaluations": max_candidates,
        "protected_paths": protected_paths
    }))
    .map_err(|error| format!("serialize evaluation configuration: {error}"))?;
    let provenance = PredicateProvenance::from_bytes(
        "cedar-policy+opa",
        "cedar-sdk=4.14.0;opa=1.22.0",
        &engine_material,
        &policy_material,
        &cedar_schema_bytes,
        &evaluation_config,
    );

    let mut cedar_candidate_times = Vec::new();
    let mut opa_cli_candidate_times = Vec::new();
    let opa_candidate_process_evaluations = Cell::new(0usize);
    let minimization_started = Instant::now();
    let result = minimize_json_with_provenance(
        &initial_value,
        &source_bytes,
        "cedar-opa-decision-divergence",
        provenance,
        &protected_paths,
        max_candidates,
        |candidate| {
            let (cedar, opa, _, cedar_ns, opa_ns) = evaluate_policy_pair(
                candidate,
                &authorizer,
                &policy_set,
                &schema,
                &opa_binary,
                &rego_policy_path,
            )?;
            if matches!(cedar.as_str(), "allow" | "deny") {
                cedar_candidate_times.push(cedar_ns);
                opa_cli_candidate_times.push(opa_ns);
                opa_candidate_process_evaluations.set(
                    opa_candidate_process_evaluations.get() + 1
                );
            }
            Ok(cedar == initial_cedar && opa == initial_opa && cedar != opa)
        },
    )
    .map_err(|error| format!("counterexample minimization incomplete: {error}"))?;
    let minimization_wall_ns = minimization_started.elapsed().as_nanos();

    let (minimized_cedar, minimized_opa, _, _, _) = evaluate_policy_pair(
        &result.minimized_value,
        &authorizer,
        &policy_set,
        &schema,
        &opa_binary,
        &rego_policy_path,
    )?;
    fs::create_dir_all(&output_directory)
        .map_err(|error| format!("create shrink output directory: {error}"))?;
    let minimized_bytes = serde_json::to_vec_pretty(&result.minimized_value)
        .map_err(|error| format!("serialize minimized counterexample: {error}"))?;
    fs::write(output_directory.join("minimized-counterexample.json"), &minimized_bytes)
        .map_err(|error| format!("write minimized counterexample: {error}"))?;
    let trace_bytes = serde_json::to_vec_pretty(&result.trace)
        .map_err(|error| format!("serialize shrink trace: {error}"))?;
    fs::write(output_directory.join("shrink-trace.json"), &trace_bytes)
        .map_err(|error| format!("write shrink trace: {error}"))?;
    let summary = ShrinkPairSummary {
        predicate: "cedar-opa-decision-divergence".to_owned(),
        evaluation_method: "Cedar in-process request construction+authorization; OPA one-shot CLI per candidate including startup/policy loading/input parsing".to_owned(),
        initial_case_id: initial_case.case_id,
        initial_cedar_decision: initial_cedar,
        initial_opa_decision: initial_opa,
        minimized_case_id: result.minimized_value["case_id"]
            .as_str()
            .ok_or_else(|| "minimized case ID is missing".to_owned())?
            .to_owned(),
        minimized_cedar_decision: minimized_cedar,
        minimized_opa_decision: minimized_opa,
        candidate_evaluations: result.trace.candidate_evaluations,
        opa_candidate_process_evaluations: opa_candidate_process_evaluations.get(),
        cedar_candidate_latency: timing_summary(&cedar_candidate_times),
        opa_cli_candidate_latency: timing_summary(&opa_cli_candidate_times),
        minimization_wall_ns,
        accepted_reductions: result.trace.accepted_reductions,
        complete: result.trace.complete,
        is_one_minimal: result.trace.is_one_minimal,
        claim_ceiling: result.trace.claim_ceiling.clone(),
    };
    let summary_bytes = serde_json::to_vec_pretty(&summary)
        .map_err(|error| format!("serialize shrink summary: {error}"))?;
    fs::write(output_directory.join("shrink-summary.json"), &summary_bytes)
        .map_err(|error| format!("write shrink summary: {error}"))?;
    println!("initial_cedar_decision={}", summary.initial_cedar_decision);
    println!("initial_opa_decision={}", summary.initial_opa_decision);
    println!("candidate_evaluations={}", summary.candidate_evaluations);
    println!("accepted_reductions={}", summary.accepted_reductions);
    println!("complete={}", summary.complete);
    println!("is_one_minimal={}", summary.is_one_minimal);
    Ok(())
}

fn run() -> Result<(), String> {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_default();
    let Some(manifest_path) = args.next() else {
        return Err(format!(
            "usage: {} <generator-manifest.json> <output-directory>",
            program.to_string_lossy()
        ));
    };
    let Some(output_directory) = args.next() else {
        return Err("expected an output directory".to_owned());
    };
    if args.next().is_some() {
        return Err("too many arguments".to_owned());
    }
    let manifest_path = PathBuf::from(manifest_path);
    let output_directory = PathBuf::from(output_directory);
    let manifest_bytes = fs::read(&manifest_path)
        .map_err(|error| format!("read generator manifest: {error}"))?;
    let input_manifest: InputManifest = serde_json::from_slice(&manifest_bytes)
        .map_err(|error| format!("parse generator manifest: {error}"))?;
    validate_manifest(&input_manifest)?;

    let corpus_directory = manifest_path.parent().unwrap_or_else(|| Path::new("."));
    let abstract_corpus_path = corpus_directory.join(&input_manifest.corpus_file);
    verify_exact_file(&abstract_corpus_path, &input_manifest.corpus_sha256)?;
    let engine_inputs_path = corpus_directory.join(&input_manifest.engine_inputs_file);
    let cases = load_cases(&engine_inputs_path, &input_manifest.engine_inputs_sha256)?;

    let baseline_source_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../")
        .join(&input_manifest.baseline_source_path);
    let baseline_source_bytes = fs::read(&baseline_source_path)
        .map_err(|error| format!("read pinned Rust authorization source: {error}"))?;
    let actual_blob_oid = git_blob_oid(&baseline_source_bytes);
    if actual_blob_oid != EXPECTED_BASELINE_SOURCE_BLOB_SHA {
        return Err(format!(
            "checked-out authorization source Git blob mismatch: expected {EXPECTED_BASELINE_SOURCE_BLOB_SHA}, got {actual_blob_oid}"
        ));
    }

    let policy_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/research/scientific-policy-dsl-bakeoff-v0.1/cedar-policy.cedar");
    let schema_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/research/scientific-policy-dsl-bakeoff-v0.1/cedar-schema.json");
    let policy_bytes = fs::read(&policy_path)
        .map_err(|error| format!("read Cedar policy: {error}"))?;
    let schema_bytes = fs::read(&schema_path)
        .map_err(|error| format!("read Cedar schema: {error}"))?;
    let policy_source = std::str::from_utf8(&policy_bytes)
        .map_err(|error| format!("Cedar policy is not UTF-8: {error}"))?;
    let schema_source = std::str::from_utf8(&schema_bytes)
        .map_err(|error| format!("Cedar schema is not UTF-8: {error}"))?;
    let policy_set: PolicySet = policy_source.parse()
        .map_err(|error| format!("parse Cedar policy: {error:?}"))?;
    let schema = Schema::from_json_str(schema_source)
        .map_err(|error| format!("parse Cedar JSON schema: {error:?}"))?;
    let validator = Validator::new(schema.clone());
    let validation = validator.validate(&policy_set, ValidationMode::Strict);
    if !validation.validation_passed() {
        return Err(format!(
            "Cedar policy failed strict schema validation: {validation:?}"
        ));
    }
    let authorizer = Authorizer::new();

    fs::create_dir_all(&output_directory)
        .map_err(|error| format!("create output directory: {error}"))?;
    let output_path = output_directory.join("cedar-decisions-v0.1.jsonl");
    let output_file = File::create(&output_path)
        .map_err(|error| format!("create Cedar decision output: {error}"))?;
    let mut writer = BufWriter::new(output_file);
    let mut output_hasher = Sha256::new();
    let warmup_cases = cases.len().min(256);
    let mut construction_times = Vec::with_capacity(cases.len() - warmup_cases);
    let mut authorization_times = Vec::with_capacity(cases.len() - warmup_cases);
    let mut end_to_end_times = Vec::with_capacity(cases.len() - warmup_cases);
    let mut allow_count = 0_usize;
    let mut deny_count = 0_usize;

    // Warm the in-process authorizer and request construction before measuring.
    for case in cases.iter().take(warmup_cases) {
        evaluate_case(&authorizer, &policy_set, &schema, case)?;
    }

    let wall_start = Instant::now();
    for (case_index, case) in cases.iter().enumerate() {
        let start = Instant::now();
        let (decision, determining_policy_ids, construction_ns, authorization_ns) =
            evaluate_case(&authorizer, &policy_set, &schema, case)?;
        let total_ns = start.elapsed().as_nanos();
        if decision == "allow" {
            allow_count += 1;
        } else {
            deny_count += 1;
        }
        if case_index >= warmup_cases {
            construction_times.push(construction_ns);
            authorization_times.push(authorization_ns);
            end_to_end_times.push(total_ns);
        }

        let record = DecisionRecord {
            case_id: &case.case_id,
            engine: "cedar-policy-4.14.0",
            decision,
            determining_policy_ids: &determining_policy_ids,
        };
        let mut row = serde_json::to_vec(&record)
            .map_err(|error| format!("serialize Cedar decision row: {error}"))?;
        row.push(b'\n');
        writer.write_all(&row)
            .map_err(|error| format!("write Cedar decision row: {error}"))?;
        output_hasher.update(&row);
    }
    writer.flush().map_err(|error| format!("flush Cedar decision output: {error}"))?;
    let total_wall_ns = wall_start.elapsed().as_nanos();
    let output_sha256 = hex::encode(output_hasher.finalize());
    let peak_rss = peak_rss_kib();
    let logical_cpus = std::thread::available_parallelism().map_or(1, usize::from);
    let throughput = if total_wall_ns == 0 {
        0.0
    } else {
        cases.len() as f64 / (total_wall_ns as f64 / 1_000_000_000.0)
    };

    let output_manifest = RunManifest {
        engine: "cedar-policy",
        engine_version: "4.14.0",
        evaluation_method: "in-process-per-request; 256-case prewarm excluded from latency distributions; wall time covers the full output pass and writes but excludes prewarm",
        generator_id: input_manifest.generator_id,
        generator_version: input_manifest.generator_version,
        baseline_revision: input_manifest.baseline_revision,
        baseline_source_path: input_manifest.baseline_source_path,
        baseline_source_blob_sha: input_manifest.baseline_source_blob_sha,
        policy_id: input_manifest.policy_id,
        policy_version: input_manifest.policy_version,
        input_file: input_manifest.engine_input_file,
        input_sha256: input_manifest.engine_inputs_sha256,
        policy_sha256: sha256(&policy_bytes),
        schema_sha256: sha256(&schema_bytes),
        output_file: "cedar-decisions-v0.1.jsonl".to_owned(),
        output_sha256,
        case_count: cases.len(),
        allow_count,
        deny_count,
        indeterminate_count: 0,
        warmup_cases,
        measured_cases: end_to_end_times.len(),
        construction_latency: timing_summary(&construction_times),
        authorization_latency: timing_summary(&authorization_times),
        end_to_end_latency: timing_summary(&end_to_end_times),
        total_wall_ns,
        throughput_cases_per_second: throughput,
        peak_rss_kib: peak_rss,
        operating_system: env::consts::OS,
        architecture: env::consts::ARCH,
        logical_cpus,
        claim_ceiling: concat!(
            "Cedar engine decisions over the shared source-pinned abstract input corpus; ",
            "not Rust/Cedar parity, formal equivalence or credential-registry correctness; ",
            "authorization gate and evidence semantics beyond this pure policy remain outside scope"
        ),
    };
    let manifest_bytes = serde_json::to_vec_pretty(&output_manifest)
        .map_err(|error| format!("serialize Cedar run manifest: {error}"))?;
    fs::write(output_directory.join("cedar-run-manifest-v0.1.json"), &manifest_bytes)
        .map_err(|error| format!("write Cedar run manifest: {error}"))?;
    println!("case_count={}", cases.len());
    println!("allow_count={allow_count}");
    println!("deny_count={deny_count}");
    println!("measured_cases={}", end_to_end_times.len());
    println!("output_sha256={output_sha256}");
    println!(
        "end_to_end_p50_ns={}",
        percentile(&end_to_end_times, 50)
    );
    println!(
        "end_to_end_p95_ns={}",
        percentile(&end_to_end_times, 95)
    );
    println!(
        "end_to_end_p99_ns={}",
        percentile(&end_to_end_times, 99)
    );
    Ok(())
}

fn main() -> ExitCode {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.first().and_then(|value| value.to_str()) == Some("--shrink-pair") {
        return match run_shrink_pair(&args[1..]) {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("indeterminate: {error}");
                ExitCode::from(2)
            }
        };
    }
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("indeterminate: {error}");
            ExitCode::from(2)
        }
    }
}
