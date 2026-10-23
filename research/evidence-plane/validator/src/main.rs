use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{
    env,
    fs::{self, File},
    io::{BufReader, Read},
    path::{Component, Path, PathBuf},
    process,
    time::{SystemTime, UNIX_EPOCH},
};

fn read_json(path: &str) -> Value {
    let bytes = fs::read(path).unwrap_or_else(|err| {
        eprintln!("cannot read {path}: {err}");
        process::exit(2);
    });
    serde_json::from_slice(&bytes).unwrap_or_else(|err| {
        eprintln!("invalid JSON in {path}: {err}");
        process::exit(2);
    })
}

fn schema_errors(schema: &Value, receipt: &Value) -> Result<Vec<String>, String> {
    jsonschema::meta::validate(schema).map_err(|err| format!("schema is invalid: {err}"))?;
    let validator = jsonschema::draft202012::options()
        .should_validate_formats(true)
        .build(schema)
        .map_err(|err| format!("cannot build Draft 2020-12 validator: {err}"))?;

    Ok(validator
        .iter_errors(receipt)
        .map(|err| format!("{err} at instance path {}", err.instance_path()))
        .collect())
}

fn required_str<'a>(value: &'a Value, pointer: &str) -> Result<&'a str, String> {
    value
        .pointer(pointer)
        .and_then(Value::as_str)
        .filter(|text| !text.is_empty())
        .ok_or_else(|| format!("required non-empty string is missing at {pointer}"))
}

fn optional_str<'a>(value: &'a Value, pointer: &str) -> Result<Option<&'a str>, String> {
    match value.pointer(pointer) {
        Some(Value::String(text)) if !text.is_empty() => Ok(Some(text.as_str())),
        Some(Value::Null) => Ok(None),
        Some(_) => Err(format!("expected a non-empty string or null at {pointer}")),
        None => Err(format!("required field is missing at {pointer}")),
    }
}

fn safe_artifact_path(root: &Path, relative: &str) -> Result<PathBuf, String> {
    if relative.contains('\\') || relative.contains(':') || relative.contains('\0') {
        return Err(format!("non-portable or unsafe artifact path rejected: {relative}"));
    }

    let rel = Path::new(relative);
    if rel.is_absolute()
        || rel
            .components()
            .any(|component| !matches!(component, Component::Normal(_)))
    {
        return Err(format!("artifact path must be relative and contain no traversal: {relative}"));
    }

    let canonical_root = fs::canonicalize(root)
        .map_err(|err| format!("cannot resolve artifact root {}: {err}", root.display()))?;
    let candidate = fs::canonicalize(canonical_root.join(rel))
        .map_err(|err| format!("cannot resolve artifact {relative}: {err}"))?;
    if !candidate.starts_with(&canonical_root) {
        return Err(format!("artifact path escapes declared root: {relative}"));
    }

    let metadata = fs::metadata(&candidate)
        .map_err(|err| format!("cannot stat artifact {relative}: {err}"))?;
    if !metadata.is_file() {
        return Err(format!("artifact is not a regular file: {relative}"));
    }

    Ok(candidate)
}

fn sha256_and_len(path: &Path) -> Result<(String, u64), String> {
    let file = File::open(path)
        .map_err(|err| format!("cannot open {}: {err}", path.display()))?;
    let mut reader = BufReader::new(file);
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    let mut length = 0_u64;

    loop {
        let read = reader
            .read(&mut buffer)
            .map_err(|err| format!("cannot read {}: {err}", path.display()))?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
        length = length
            .checked_add(read as u64)
            .ok_or_else(|| format!("file length overflow for {}", path.display()))?;
    }

    Ok((format!("{:x}", hasher.finalize()), length))
}

fn verify_one(
    root: &Path,
    relative: &str,
    expected_sha256: &str,
    expected_bytes: Option<u64>,
    label: &str,
) -> Result<(), String> {
    let path = safe_artifact_path(root, relative)?;
    let (actual_sha256, actual_bytes) = sha256_and_len(&path)?;

    if actual_sha256 != expected_sha256 {
        return Err(format!(
            "{label}: SHA-256 mismatch for {relative}: expected {expected_sha256}, actual {actual_sha256}"
        ));
    }
    if let Some(expected) = expected_bytes {
        if actual_bytes != expected {
            return Err(format!(
                "{label}: byte-count mismatch for {relative}: expected {expected}, actual {actual_bytes}"
            ));
        }
    }
    Ok(())
}

fn verify_artifacts(root: &Path, receipt: &Value) -> Result<(), String> {
    let tier = required_str(receipt, "/evidence_tier")?;
    let subject_hash = optional_str(receipt, "/subject/artifact_sha256")?;
    let subject_path = required_str(receipt, "/subject/artifact_path")?;

    if matches!(tier, "S2" | "S3" | "S4") && subject_hash.is_none() {
        return Err(format!("{tier} receipt has no subject artifact hash"));
    }
    if let Some(digest) = subject_hash {
        verify_one(root, subject_path, digest, None, "subject")?;
    }

    let sources = receipt
        .get("sources")
        .and_then(Value::as_array)
        .ok_or_else(|| "sources must be an array".to_string())?;
    let mut source_ids = std::collections::HashSet::new();

    for (index, source) in sources.iter().enumerate() {
        let id = source
            .get("source_id")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("source {index} is missing source_id"))?;
        if !source_ids.insert(id.to_string()) {
            return Err(format!("duplicate source_id: {id}"));
        }

        let raw_ingested = source
            .get("raw_bytes_ingested")
            .and_then(Value::as_bool)
            .ok_or_else(|| format!("source {id} has invalid raw_bytes_ingested"))?;
        let artifact_path = source
            .get("artifact_path")
            .ok_or_else(|| format!("source {id} is missing artifact_path"))?;
        let source_hash = source
            .get("sha256")
            .ok_or_else(|| format!("source {id} is missing sha256"))?;
        let byte_count = source
            .get("byte_count")
            .ok_or_else(|| format!("source {id} is missing byte_count"))?;

        match (artifact_path, source_hash) {
            (Value::String(path), Value::String(digest)) => {
                let count = byte_count.as_u64().ok_or_else(|| {
                    format!("source {id} has an artifact path/hash but no valid byte_count")
                })?;
                if !raw_ingested {
                    return Err(format!(
                        "source {id} has a local artifact path/hash but raw_bytes_ingested is false"
                    ));
                }
                verify_one(root, path, digest, Some(count), &format!("source {id}"))?;
            }
            (Value::Null, Value::Null) if !raw_ingested => {}
            (Value::Null, Value::Null) => {
                return Err(format!("source {id} is marked ingested but has no local path/hash"));
            }
            _ => {
                return Err(format!(
                    "source {id} must declare artifact_path and sha256 together, or null together"
                ));
            }
        }
    }

    let transforms = receipt
        .get("transforms")
        .and_then(Value::as_array)
        .ok_or_else(|| "transforms must be an array".to_string())?;
    for (index, transform) in transforms.iter().enumerate() {
        let name = transform
            .get("name")
            .and_then(Value::as_str)
            .unwrap_or("<unnamed>");
        let input_ids = transform
            .get("input_source_ids")
            .and_then(Value::as_array)
            .ok_or_else(|| format!("transform {index} has invalid input_source_ids"))?;
        for input in input_ids {
            let input_id = input
                .as_str()
                .ok_or_else(|| format!("transform {name} has a non-string source ID"))?;
            if !source_ids.contains(input_id) {
                return Err(format!(
                    "transform {name} references unknown source_id {input_id}"
                ));
            }
        }

        let output_hash = optional_str(transform, "/output_artifact_sha256")?;
        let output_path = optional_str(transform, "/output_artifact_path")?;
        match (output_path, output_hash) {
            (Some(path), Some(digest)) => {
                verify_one(root, path, digest, None, &format!("transform {name} output"))?;
            }
            (None, None) => {}
            _ => {
                return Err(format!(
                    "transform {name} must declare output_artifact_path and output_artifact_sha256 together"
                ));
            }
        }
    }

    Ok(())
}

fn print_usage() {
    eprintln!("usage:");
    eprintln!("  evidence-plane-validator validate <schema.json> <receipt.json>");
    eprintln!("  evidence-plane-validator verify <schema.json> <receipt.json> --artifact-root <dir>");
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 3 {
        print_usage();
        process::exit(2);
    }

    let command = &args[0];
    let schema = read_json(&args[1]);
    let receipt = read_json(&args[2]);

    match schema_errors(&schema, &receipt) {
        Err(err) => {
            eprintln!("ERROR: {err}");
            process::exit(2);
        }
        Ok(errors) if !errors.is_empty() => {
            for error in errors {
                eprintln!("INVALID: {error}");
            }
            process::exit(1);
        }
        Ok(_) => {}
    }

    match command.as_str() {
        "validate" if args.len() == 3 => {
            println!("receipt conforms to the Evidence Plane schema; hashes, permissions and scientific validity have not been verified");
        }
        "verify" if args.len() == 5 && args[3] == "--artifact-root" => {
            let root = Path::new(&args[4]);
            if let Err(err) = verify_artifacts(root, &receipt) {
                eprintln!("VERIFY FAILED: {err}");
                process::exit(1);
            }
            println!("schema and declared local artifact hashes/byte counts verified; source truth, reuse permission and scientific/operational validity are not established by this check");
        }
        _ => {
            print_usage();
            process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{schema_errors, sha256_and_len, verify_artifacts};
    use serde_json::{json, Value};
    use std::{
        fs,
        path::PathBuf,
        time::{SystemTime, UNIX_EPOCH},
    };

    fn evidence_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
    }

    fn schema() -> Value {
        let path = evidence_dir().join("receipt-v0.schema.json");
        serde_json::from_slice(&fs::read(path).expect("read schema")).expect("parse schema JSON")
    }

    fn fixture() -> Value {
        let path = evidence_dir().join("examples/synthetic-planned-receipt-v0.json");
        serde_json::from_slice(&fs::read(path).expect("read fixture")).expect("parse fixture JSON")
    }

    fn errors(instance: &Value) -> Vec<String> {
        schema_errors(&schema(), instance).expect("schema is valid")
    }

    fn temp_root() -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock after epoch")
            .as_nanos();
        let root = std::env::temp_dir().join(format!(
            "evidence-plane-test-{}-{nonce}",
            std::process::id()
        ));
        fs::create_dir_all(&root).expect("create temporary test root");
        root
    }

    fn put(root: &std::path::Path, relative: &str, bytes: &[u8]) {
        let path = root.join(relative);
        fs::create_dir_all(path.parent().expect("parent")).expect("create parents");
        fs::write(path, bytes).expect("write test artifact");
    }

    fn hash(path: &std::path::Path) -> (String, u64) {
        sha256_and_len(path).expect("hash test artifact")
    }

    #[test]
    fn schema_document_passes_its_meta_schema() {
        assert!(jsonschema::meta::validate(&schema()).is_ok());
    }

    #[test]
    fn synthetic_planned_receipt_is_valid() {
        let value = fixture();
        assert!(errors(&value).is_empty(), "{:?}", errors(&value));
    }

    #[test]
    fn planned_receipt_cannot_claim_domain_validation() {
        let mut value = fixture();
        value["claim_ceiling"] = json!("domain_validated");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn planned_receipt_cannot_promote_its_evidence_tier() {
        let mut value = fixture();
        value["evidence_tier"] = json!("S4");
        value["claim_ceiling"] = json!("domain_validated");
        value["run_status"] = json!("domain_validated");
        value["gates"]["domain_validation"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn planned_receipt_cannot_mark_an_unknown_gate_as_pass() {
        let mut value = fixture();
        value["gates"]["source_integrity"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn missing_gate_is_rejected() {
        let mut value = fixture();
        value["gates"].as_object_mut().expect("gates object").remove("exact_head");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn malformed_sha256_is_rejected() {
        let mut value = fixture();
        value["subject"]["artifact_sha256"] = json!("sha256:not-a-hex-digest");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn invalid_timestamp_is_rejected_when_format_checks_are_enabled() {
        let mut value = fixture();
        value["created_at"] = json!("yesterday, sometime");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn unknown_root_fields_are_rejected() {
        let mut value = fixture();
        value["trust_me"] = json!(true);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn ingested_source_requires_byte_count_hash_and_path() {
        let mut value = fixture();
        value["sources"] = json!([{
            "source_id":"fixture-source",
            "url":"https://example.org/data.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":true,
            "byte_count":12,
            "sha256":null,
            "artifact_path":"data/fixture.csv",
            "rights_status":"open_license",
            "contact_status":"not_required",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        }]);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn s2_receipt_rejects_unresolved_stewardship() {
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["evidence_tier"] = json!("S2");
        value["claim_ceiling"] = json!("source_reproducible");
        value["subject"]["artifact_sha256"] = json!("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        value["sources"] = json!([{
            "source_id":"fixture-source",
            "url":"https://example.org/data.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":true,
            "byte_count":12,
            "sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "artifact_path":"data/fixture.csv",
            "rights_status":"open_license",
            "contact_status":"acknowledged_not_contacted",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        }]);
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn s4_requires_independent_verification_and_exact_head() {
        let mut value = fixture();
        value["run_status"] = json!("domain_validated");
        value["evidence_tier"] = json!("S4");
        value["claim_ceiling"] = json!("domain_validated");
        value["gates"]["domain_validation"] = json!("pass");
        assert!(!errors(&value).is_empty());
    }

    #[test]
    fn verify_recomputes_subject_sha256() {
        let root = temp_root();
        put(&root, "artifact.bin", b"immutable evidence bytes");
        let (digest, _) = hash(&root.join("artifact.bin"));
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["subject"]["artifact_path"] = json!("artifact.bin");
        value["subject"]["artifact_sha256"] = json!(digest);
        assert!(errors(&value).is_empty(), "{:?}", errors(&value));
        assert!(verify_artifacts(&root, &value).is_ok());
        fs::remove_dir_all(root).expect("cleanup");
    }

    #[test]
    fn verify_rejects_subject_hash_mismatch() {
        let root = temp_root();
        put(&root, "artifact.bin", b"actual bytes");
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["subject"]["artifact_path"] = json!("artifact.bin");
        value["subject"]["artifact_sha256"] = json!("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        assert!(verify_artifacts(&root, &value).unwrap_err().contains("SHA-256 mismatch"));
        fs::remove_dir_all(root).expect("cleanup");
    }

    #[test]
    fn verify_rejects_parent_path_traversal() {
        let root = temp_root();
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["subject"]["artifact_path"] = json!("../outside.bin");
        value["subject"]["artifact_sha256"] = json!("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        assert!(verify_artifacts(&root, &value).unwrap_err().contains("traversal"));
        fs::remove_dir_all(root).expect("cleanup");
    }

    #[test]
    fn verify_checks_source_byte_count_and_hash() {
        let root = temp_root();
        put(&root, "data/source.csv", b"a,b\n1,2\n");
        put(&root, "result.json", b"result");
        let (source_hash, source_len) = hash(&root.join("data/source.csv"));
        let (subject_hash, _) = hash(&root.join("result.json"));
        let mut value = fixture();
        value["run_status"] = json!("executed");
        value["subject"]["artifact_path"] = json!("result.json");
        value["subject"]["artifact_sha256"] = json!(subject_hash);
        value["sources"] = json!([{
            "source_id":"src-1",
            "url":"https://example.org/source.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":true,
            "byte_count":source_len,
            "sha256":source_hash,
            "artifact_path":"data/source.csv",
            "rights_status":"open_license",
            "contact_status":"not_required",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        }]);
        assert!(errors(&value).is_empty(), "{:?}", errors(&value));
        assert!(verify_artifacts(&root, &value).is_ok());
        fs::remove_dir_all(root).expect("cleanup");
    }

    #[test]
    fn verify_rejects_duplicate_source_ids() {
        let root = temp_root();
        let base_source = json!({
            "source_id":"duplicate",
            "url":"https://example.org/source.csv",
            "observed_at":"2026-10-22T00:00:00Z",
            "media_type":"text/csv",
            "raw_bytes_ingested":false,
            "byte_count":null,
            "sha256":null,
            "artifact_path":null,
            "rights_status":"unknown",
            "contact_status":"unknown",
            "temporal_coverage":null,
            "spatial_coverage":null,
            "metadata_conflicts":[]
        });
        let mut value = fixture();
        value["sources"] = json!([base_source.clone(), base_source]);
        assert!(verify_artifacts(&root, &value).unwrap_err().contains("duplicate source_id"));
        fs::remove_dir_all(root).expect("cleanup");
    }

    #[test]
    fn verify_rejects_transform_referencing_unknown_source() {
        let root = temp_root();
        let mut value = fixture();
        value["transforms"] = json!([{
            "name":"normalize",
            "version":"1",
            "implementation_commit":null,
            "parameters_sha256":null,
            "command_digest":null,
            "input_source_ids":["not-present"],
            "output_artifact_sha256":null,
            "output_artifact_path":null,
            "notes":"test only"
        }]);
        assert!(verify_artifacts(&root, &value).unwrap_err().contains("unknown source_id"));
        fs::remove_dir_all(root).expect("cleanup");
    }
}
