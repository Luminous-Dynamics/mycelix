use cedar_policy::{Authorizer, Context, Decision, Entities, EntityUid, PolicySet, Request, Schema, ValidationMode, Validator};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use sha1::Sha1;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::env;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
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
        if case.cedar.entities.len() != 2 {
            return Err(format!("Cedar input must contain actor and claim only; action entities come from the pinned schema (line {})", line_index + 1));
        }
        for entity in &case.cedar.entities {
            if entity["uid"]["type"].as_str().is_none()
                || entity["uid"]["id"].as_str().is_none()
                || entity.get("uid").and_then(|uid| uid.get("entity_type")).is_some()
            {
                return Err(format!("Cedar entity UID must use the native type/id shape at line {}", line_index + 1));
            }
        }
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
) -> Result<(&'static str, u128, u128), String> {
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

    let decision = match response.decision() {
        Decision::Allow => "allow",
        Decision::Deny => "deny",
    };
    Ok((decision, construction_ns, authorization_ns))
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
        let (decision, construction_ns, authorization_ns) =
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
        evaluation_method: "in-process-per-request-with-256-case-warmup; wall time includes warmup and all output writes",
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
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("indeterminate: {error}");
            ExitCode::from(2)
        }
    }
}
