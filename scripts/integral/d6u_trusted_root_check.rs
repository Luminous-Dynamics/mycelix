#![forbid(unsafe_code)]

//! Dependency-free Rust preflight for the D6U trusted source closure.
//!
//! This program does not execute candidate source. It rejects symlinked roots and
//! path components, snapshots each pinned file once, and computes the Git blob
//! identity from those exact bytes. Full behavioral tests for the Python candidate
//! programs remain separate and must still run in the restricted evaluator.

use std::env;
use std::fs;
use std::io::Write;
use std::path::{Component, Path, PathBuf};
use std::process::{Command, Stdio};

#[derive(Clone, Copy)]
struct Pin {
    label: &'static str,
    path: &'static str,
    blob: &'static str,
}

// These values are intentionally literal. The source file itself is independently
// blob-pinned by the calling workflow. Updating any pin requires reviewing the
// policy and the complete trust closure, not just accepting a new digest.
const EXPECTED_EVALUATION_SCOPE: &str = "policy-fingerprint,workflow-and-five-program-closure,canonical-path-and-symlink-containment,candidate-root-identity-and-cli-path-preservation,validated-source-snapshot-import,alias-pin-consistency,provenance-binding,artifact-integrity,redirect-safety,optimized-mode-refusal";

const EXPECTED_RECEIPT_KEYS: [&str; 21] = [
    "record_schema",
    "status",
    "repository",
    "candidate_sha",
    "candidate_verifier_blob_sha",
    "candidate_trusted_workflow_blob_sha",
    "candidate_fetcher_blob_sha",
    "candidate_attestation_verifier_blob_sha",
    "candidate_predicate_emitter_blob_sha",
    "candidate_retention_verifier_blob_sha",
    "candidate_policy_blob_sha",
    "trusted_evaluator_blob_sha",
    "trusted_rust_preflight_blob_sha",
    "trusted_source_commit_sha",
    "workflow_file_commit_sha",
    "workflow_ref",
    "run_id",
    "run_attempt",
    "claim_ceiling",
    "evaluation_scope",
    "container_image",
];

#[derive(Debug)]
struct ReceiptSummary {
    candidate_sha: String,
    run_id: u64,
    run_attempt: u32,
    scope_items: usize,
}

fn is_lower_hex(value: &str, length: usize) -> bool {
    value.len() == length
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn canonical_positive_integer<T>(value: &str, field: &str) -> Result<T, String>
where
    T: std::str::FromStr + ToString,
{
    let parsed = value
        .parse::<T>()
        .map_err(|_| format!("receipt field {field} is not a positive integer"))?;
    if parsed.to_string() != value || parsed.to_string().starts_with('0') {
        return Err(format!("receipt field {field} is not canonical"));
    }
    Ok(parsed)
}

fn validate_evaluation_scope(value: &str) -> Result<usize, String> {
    let items: Vec<_> = value.split(',').collect();
    if value != EXPECTED_EVALUATION_SCOPE
        || items.iter().any(|item| item.is_empty())
        || items.iter().collect::<std::collections::BTreeSet<_>>().len() != items.len()
    {
        return Err("receipt evaluation_scope differs from the exact trusted inventory".to_string());
    }
    Ok(items.len())
}

fn parse_receipt_bytes(bytes: &[u8]) -> Result<ReceiptSummary, String> {
    let source =
        std::str::from_utf8(bytes).map_err(|error| format!("receipt is not UTF-8: {error}"))?;
    if !source.ends_with('\n') || source.contains('\r') {
        return Err("receipt must use LF line endings and end with a newline".to_string());
    }

    let mut fields = std::collections::BTreeMap::<String, String>::new();
    for (index, line) in source.lines().enumerate() {
        if line.is_empty() {
            return Err(format!("receipt contains an empty line at {}", index + 1));
        }
        let (key, value) = line
            .split_once('=')
            .ok_or_else(|| format!("receipt line {} lacks '='", index + 1))?;
        if key.is_empty()
            || !key
                .bytes()
                .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_')
            || value.is_empty()
            || value.contains('=')
        {
            return Err(format!("receipt line {} is malformed", index + 1));
        }
        if fields.insert(key.to_string(), value.to_string()).is_some() {
            return Err(format!("receipt contains duplicate key: {key}"));
        }
    }

    let actual_keys: std::collections::BTreeSet<_> = fields.keys().map(String::as_str).collect();
    let expected_keys: std::collections::BTreeSet<_> = EXPECTED_RECEIPT_KEYS.into_iter().collect();
    if actual_keys != expected_keys {
        let missing: Vec<_> = expected_keys.difference(&actual_keys).copied().collect();
        let extra: Vec<_> = actual_keys.difference(&expected_keys).copied().collect();
        return Err(format!(
            "receipt schema mismatch; missing={missing:?}, extra={extra:?}"
        ));
    }

    let get = |key: &str| -> Result<&str, String> {
        fields
            .get(key)
            .map(String::as_str)
            .ok_or_else(|| format!("receipt is missing field {key}"))
    };

    if get("record_schema")? != "d6u-independent-verifier-evaluation/v2" {
        return Err("unexpected receipt schema version".to_string());
    }
    if get("status")? != "passed" {
        return Err("receipt status is not passed".to_string());
    }
    if get("repository")? != "Luminous-Dynamics/mycelix" {
        return Err("receipt repository identity mismatch".to_string());
    }
    if get("claim_ceiling")? != "ReferenceModelOnly" {
        return Err("receipt claim ceiling mismatch".to_string());
    }
    if get("workflow_ref")?
        != "Luminous-Dynamics/mycelix/.github/workflows/d6u-trusted-verifier-candidate-check.yml@refs/heads/main"
    {
        return Err("receipt workflow ref is not the trusted main workflow".to_string());
    }
    if get("container_image")?
        != "python:3.12-slim-bookworm@sha256:9901e0a8d75037d8242ed43155cbcb2d1f61be1356383d8054afb59fd50e39c4"
    {
        return Err("receipt container image identity mismatch".to_string());
    }

    for field in [
        "candidate_sha",
        "trusted_source_commit_sha",
        "workflow_file_commit_sha",
    ] {
        if !is_lower_hex(get(field)?, 40) {
            return Err(format!("receipt field {field} is not a canonical Git SHA"));
        }
    }
    for field in [
        "candidate_verifier_blob_sha",
        "candidate_trusted_workflow_blob_sha",
        "candidate_fetcher_blob_sha",
        "candidate_attestation_verifier_blob_sha",
        "candidate_predicate_emitter_blob_sha",
        "candidate_retention_verifier_blob_sha",
        "candidate_policy_blob_sha",
        "trusted_evaluator_blob_sha",
        "trusted_rust_preflight_blob_sha",
    ] {
        if !is_lower_hex(get(field)?, 40) {
            return Err(format!("receipt field {field} is not a canonical Git blob SHA"));
        }
    }

    let run_id = canonical_positive_integer::<u64>(get("run_id")?, "run_id")?;
    let run_attempt = canonical_positive_integer::<u32>(get("run_attempt")?, "run_attempt")?;
    let scope_items = validate_evaluation_scope(get("evaluation_scope")?)?;

    Ok(ReceiptSummary {
        candidate_sha: get("candidate_sha")?.to_string(),
        run_id,
        run_attempt,
        scope_items,
    })
}

fn validate_receipt_file(path: &Path) -> Result<ReceiptSummary, String> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot inspect receipt {}: {error}", path.display()))?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err("receipt path must be a regular file, not a symlink".to_string());
    }
    let bytes = fs::read(path)
        .map_err(|error| format!("cannot read receipt {}: {error}", path.display()))?;
    parse_receipt_bytes(&bytes)
}

const TRUSTED_PINS: [Pin; 7] = [
    Pin {
        label: "policy",
        path: "docs/integral/d6u-trusted-builder-policy.json",
        blob: "f7c7581017cc5773243a2d25093e5de3cd522e2e",
    },
    Pin {
        label: "attestation workflow",
        path: ".github/workflows/d6u-trusted-evidence-attestation.yml",
        blob: "652f81c65cdf6d6138ff2c7f4be5b0bd74ae3d61",
    },
    Pin {
        label: "trusted artifact verifier",
        path: "scripts/integral/verify_d6u_trusted_artifacts.py",
        blob: "de5b3275dda9cbeaa880d26a9ff2dc0e58d7d65e",
    },
    Pin {
        label: "trusted artifact fetcher",
        path: "scripts/integral/fetch_d6u_trusted_artifact.py",
        blob: "4c4d67f6f9a77c512728a8723cf2aa708687a395",
    },
    Pin {
        label: "attestation verifier",
        path: "scripts/integral/verify_d6u_trusted_attestation.py",
        blob: "91331c11f6c6de76f44c8d977629971193e2c0ba",
    },
    Pin {
        label: "attestation predicate emitter",
        path: "scripts/integral/emit_d6u_trusted_attestation_predicate.py",
        blob: "ce32e161d3f45918ba8ea5545d02e233af331ead",
    },
    Pin {
        label: "attestation retention verifier",
        path: "scripts/integral/verify_d6u_trusted_attestation_retention.py",
        blob: "df1e4fd9ba3c924037ef7ae97d087751ac8e122f",
    },
];

fn safe_relative_components(value: &str) -> Result<Vec<PathBuf>, String> {
    if value.is_empty() || value.contains('\\') {
        return Err(format!("unsafe relative path spelling: {value:?}"));
    }

    let mut parts = Vec::new();
    for component in Path::new(value).components() {
        match component {
            Component::Normal(part) => parts.push(PathBuf::from(part)),
            Component::CurDir | Component::ParentDir | Component::RootDir | Component::Prefix(_) => {
                return Err(format!("non-canonical relative path: {value:?}"));
            }
        }
    }

    if parts.is_empty() {
        return Err("empty relative path".to_string());
    }
    Ok(parts)
}

fn candidate_root_from_arg(value: &str) -> Result<PathBuf, String> {
    if !value.starts_with('/') || value.contains('\\') {
        return Err(format!("candidate root is not a canonical absolute path: {value:?}"));
    }

    // Path::components() normalizes some spelling differences, so reject those
    // from the original CLI string before constructing a Path.
    let components: Vec<_> = value.split('/').collect();
    if components.first() != Some(&"")
        || components
            .iter()
            .skip(1)
            .any(|part| part.is_empty() || *part == "." || *part == "..")
    {
        return Err(format!("candidate root is not canonically spelled: {value:?}"));
    }

    Ok(PathBuf::from(value))
}

fn reject_symlinked_path_components(path: &Path) -> Result<(), String> {
    if !path.is_absolute() {
        return Err(format!("candidate root is not absolute: {}", path.display()));
    }

    let components: Vec<_> = path.components().collect();
    let mut cursor = PathBuf::new();
    for (index, component) in components.iter().enumerate() {
        match component {
            Component::RootDir => cursor.push(Path::new("/")),
            Component::Normal(part) => cursor.push(part),
            Component::CurDir | Component::ParentDir | Component::Prefix(_) => {
                return Err(format!("candidate root is not canonical: {}", path.display()));
            }
        }

        let metadata = fs::symlink_metadata(&cursor).map_err(|error| {
            format!("cannot inspect candidate-root component {}: {error}", cursor.display())
        })?;
        if metadata.file_type().is_symlink() {
            return Err(format!("candidate root traverses a symlink: {}", cursor.display()));
        }
        if index + 1 < components.len() && !metadata.is_dir() {
            return Err(format!(
                "candidate-root component is not a directory: {}",
                cursor.display()
            ));
        }
    }

    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot inspect candidate root {}: {error}", path.display()))?;
    if metadata.file_type().is_symlink() || !metadata.is_dir() {
        return Err(format!("candidate root is not a real directory: {}", path.display()));
    }
    Ok(())
}

fn read_candidate_snapshot(root: &Path, relative: &str) -> Result<Vec<u8>, String> {
    let components = safe_relative_components(relative)?;
    let mut cursor = root.to_path_buf();

    for (index, component) in components.iter().enumerate() {
        cursor.push(component);
        let metadata = fs::symlink_metadata(&cursor)
            .map_err(|error| format!("cannot inspect pinned path {relative}: {error}"))?;
        if metadata.file_type().is_symlink() {
            return Err(format!("pinned path traverses a symlink: {relative}"));
        }
        if index + 1 < components.len() && !metadata.is_dir() {
            return Err(format!("pinned path parent is not a directory: {relative}"));
        }
        if index + 1 == components.len() && !metadata.is_file() {
            return Err(format!("pinned path is not a regular file: {relative}"));
        }
    }

    // Hash the bytes returned here, rather than asking Git to reopen this path.
    // No candidate source is executed by this preflight.
    fs::read(&cursor).map_err(|error| format!("cannot read pinned path {relative}: {error}"))
}

fn git_blob_sha1(snapshot: &[u8]) -> Result<String, String> {
    let mut child = Command::new("git")
        .args(["hash-object", "--stdin"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| format!("cannot start git hash-object: {error}"))?;

    child
        .stdin
        .as_mut()
        .ok_or_else(|| "git hash-object stdin unavailable".to_string())?
        .write_all(snapshot)
        .map_err(|error| format!("cannot send source snapshot to git hash-object: {error}"))?;
    drop(child.stdin.take());

    let output = child
        .wait_with_output()
        .map_err(|error| format!("cannot collect git hash-object result: {error}"))?;
    if !output.status.success() {
        return Err(format!(
            "git hash-object failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }

    let digest = String::from_utf8(output.stdout)
        .map_err(|error| format!("git hash-object emitted non-UTF-8 output: {error}"))?
        .trim()
        .to_string();
    if digest.len() != 40 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Err(format!("git hash-object emitted a malformed SHA-1: {digest:?}"));
    }
    Ok(digest)
}

fn evaluate(candidate_root: &Path) -> Result<(), String> {
    reject_symlinked_path_components(candidate_root)?;

    for pin in TRUSTED_PINS {
        let snapshot = read_candidate_snapshot(candidate_root, pin.path)?;
        let observed = git_blob_sha1(&snapshot)?;
        if observed != pin.blob {
            return Err(format!(
                "trusted {label} blob mismatch for {path}: expected {expected}, observed {observed}",
                label = pin.label,
                path = pin.path,
                expected = pin.blob,
            ));
        }
        println!("D6U Rust source preflight: {label}: PASS ({observed})", label = pin.label);
    }

    println!("D6U Rust trusted-root preflight: PASS");
    Ok(())
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let result = match args.as_slice() {
        [_, command, candidate_root] if command == "--candidate-root" => {
            match candidate_root_from_arg(candidate_root) {
                Ok(path) => evaluate(&path).map(|()| ()),
                Err(error) => Err(error),
            }
        }
        [_, command, receipt_path] if command == "--validate-receipt" => {
            validate_receipt_file(Path::new(receipt_path)).map(|summary| {
                println!(
                    "D6U receipt validation: PASS candidate_sha={} run_id={} run_attempt={} scope_items={}",
                    summary.candidate_sha, summary.run_id, summary.run_attempt, summary.scope_items
                );
            })
        }
        _ => {
            eprintln!("usage: d6u_trusted_root_check --candidate-root ABSOLUTE_PATH");
            eprintln!("   or: d6u_trusted_root_check --validate-receipt RECEIPT_PATH");
            std::process::exit(2);
        }
    };

    if let Err(error) = result {
        eprintln!("D6U trusted-root/receipt preflight: REFUSED: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn scratch_dir(label: &str) -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("clock should be after Unix epoch")
            .as_nanos();
        let path = env::temp_dir().join(format!(
            "d6u-rust-root-check-{label}-{}-{nonce}",
            std::process::id()
        ));
        fs::create_dir_all(&path).expect("scratch directory should be created");
        path
    }

    #[test]
    fn rejects_noncanonical_or_host_absolute_relative_paths() {
        assert!(safe_relative_components("docs/integral/policy.json").is_ok());
        assert!(safe_relative_components("../outside").is_err());
        assert!(safe_relative_components("docs/../outside").is_err());
        assert!(safe_relative_components("/etc/passwd").is_err());
        assert!(safe_relative_components(r"docs\policy.json").is_err());
        assert!(safe_relative_components(".").is_err());
        assert!(safe_relative_components("").is_err());

        assert!(candidate_root_from_arg("/tmp/candidate").is_ok());
        assert!(candidate_root_from_arg("relative/candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/../candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/./candidate").is_err());
        assert!(candidate_root_from_arg("/tmp//candidate").is_err());
        assert!(candidate_root_from_arg("/tmp/candidate/").is_err());
        assert!(candidate_root_from_arg(r"/tmp\\candidate").is_err());
    }

    #[test]
    fn git_blob_identity_uses_exact_source_bytes() {
        assert_eq!(
            git_blob_sha1(b"").expect("git hash-object should be available"),
            "e69de29bb2d1d6434b8b29ae775ad8c2e48c5391"
        );
        assert_ne!(
            git_blob_sha1(b"validated snapshot\n").expect("git hash-object should be available"),
            git_blob_sha1(b"reopened path\n").expect("git hash-object should be available")
        );
    }

    fn valid_receipt() -> String {
        format!(
            concat!(
                "record_schema=d6u-independent-verifier-evaluation/v2\n",
                "status=passed\n",
                "repository=Luminous-Dynamics/mycelix\n",
                "candidate_sha={}\n",
                "candidate_verifier_blob_sha={}\n",
                "candidate_trusted_workflow_blob_sha={}\n",
                "candidate_fetcher_blob_sha={}\n",
                "candidate_attestation_verifier_blob_sha={}\n",
                "candidate_predicate_emitter_blob_sha={}\n",
                "candidate_retention_verifier_blob_sha={}\n",
                "candidate_policy_blob_sha={}\n",
                "trusted_evaluator_blob_sha={}\n",
                "trusted_rust_preflight_blob_sha={}\n",
                "trusted_source_commit_sha={}\n",
                "workflow_file_commit_sha={}\n",
                "workflow_ref=Luminous-Dynamics/mycelix/.github/workflows/d6u-trusted-verifier-candidate-check.yml@refs/heads/main\n",
                "run_id=12\n",
                "run_attempt=2\n",
                "claim_ceiling=ReferenceModelOnly\n",
                "evaluation_scope={}\n",
                "container_image=python:3.12-slim-bookworm@sha256:9901e0a8d75037d8242ed43155cbcb2d1f61be1356383d8054afb59fd50e39c4\n"
            ),
            "a".repeat(40),
            "b".repeat(40),
            "c".repeat(40),
            "d".repeat(40),
            "e".repeat(40),
            "f".repeat(40),
            "1".repeat(40),
            "2".repeat(40),
            "3".repeat(40),
            "4".repeat(40),
            "5".repeat(40),
            "6".repeat(40),
            EXPECTED_EVALUATION_SCOPE,
        )
    }

    #[test]
    fn validates_typed_attempt_bound_receipt() {
        let bytes = valid_receipt();
        let parsed = parse_receipt_bytes(bytes.as_bytes()).expect("valid fixture should parse");
        assert_eq!(parsed.candidate_sha, "a".repeat(40));
        assert_eq!(parsed.run_id, 12);
        assert_eq!(parsed.run_attempt, 2);
        assert_eq!(parsed.scope_items, 10);
    }

    #[test]
    fn rejects_duplicate_missing_extra_and_untrusted_receipt_fields() {
        let bytes = valid_receipt();
        assert!(parse_receipt_bytes(format!("{bytes}run_id=13\n").as_bytes()).is_err());
        assert!(parse_receipt_bytes(bytes.replace("run_attempt=2\n", "").as_bytes()).is_err());
        assert!(parse_receipt_bytes(format!("{bytes}extra=attacker-controlled\n").as_bytes()).is_err());
        assert!(parse_receipt_bytes(bytes.replace("status=passed", "status=failed").as_bytes()).is_err());
        assert!(parse_receipt_bytes(bytes.replace("claim_ceiling=ReferenceModelOnly", "claim_ceiling=OperationallyQualified").as_bytes()).is_err());
    }

    #[test]
    fn rejects_scope_duplicates_reordering_and_unreviewed_extensions() {
        assert!(validate_evaluation_scope(EXPECTED_EVALUATION_SCOPE).is_ok());
        assert!(validate_evaluation_scope("policy-fingerprint,policy-fingerprint").is_err());
        assert!(validate_evaluation_scope("workflow-and-five-program-closure,policy-fingerprint").is_err());
        assert!(validate_evaluation_scope("policy-fingerprint,workflow-and-five-program-closure,unknown-scope").is_err());
    }

    #[test]
    fn rejects_a_candidate_root_reached_through_a_symlink() {
        use std::os::unix::fs::symlink;

        let scratch = scratch_dir("root-symlink");
        let real_root = scratch.join("real");
        fs::create_dir(&real_root).expect("real root should be created");
        let alias = scratch.join("alias");
        symlink(&real_root, &alias).expect("symlink should be created");

        assert!(reject_symlinked_path_components(&real_root).is_ok());
        assert!(reject_symlinked_path_components(&alias).is_err());
        fs::remove_dir_all(scratch).expect("scratch tree should be removed");
    }

    #[test]
    fn rejects_symlinked_intermediate_file_components() {
        use std::os::unix::fs::symlink;

        let scratch = scratch_dir("nested-symlink");
        let root = scratch.join("root");
        let real_directory = root.join("real");
        fs::create_dir_all(&real_directory).expect("real directory should be created");
        fs::write(real_directory.join("file.txt"), b"fixture")
            .expect("fixture file should be written");
        symlink(&real_directory, root.join("alias")).expect("symlink should be created");

        assert!(read_candidate_snapshot(&root, "real/file.txt").is_ok());
        assert!(read_candidate_snapshot(&root, "alias/file.txt").is_err());
        assert!(read_candidate_snapshot(&root, "../outside").is_err());
        fs::remove_dir_all(scratch).expect("scratch tree should be removed");
    }
}
