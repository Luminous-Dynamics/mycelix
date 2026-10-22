//! Byte-verifying source intake for NRVCA benchmark sources.
//!
//! A receipt identifies the exact bytes read from a local regular file. It does
//! not authenticate the publisher, grant usage rights, validate source truth,
//! or mean the content is fit for external evaluation.

use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

const HASH_BUFFER_BYTES: usize = 64 * 1024;
pub const HASH_SOURCE_USAGE: &str = "Usage: nrvca-validator hash-source --file PATH --source-id ID --publisher NAME --source-uri URI --publication-id ID --vintage VINTAGE --captured-at RFC3339 --rights-status open_reuse|permission_granted|evaluation_only|restricted|prohibited|unknown --rights-evidence-uri URI --evaluation-allowed true|false --redistribution-allowed true|false --locator-id ID --locator TEXT --extraction-method METHOD --tool-version VERSION [--expected-sha256 HEX] [--output PATH]";
const KNOWN_FLAGS: [&str; 17] = [
    "--file",
    "--source-id",
    "--publisher",
    "--source-uri",
    "--publication-id",
    "--vintage",
    "--captured-at",
    "--rights-status",
    "--rights-evidence-uri",
    "--evaluation-allowed",
    "--redistribution-allowed",
    "--locator-id",
    "--locator",
    "--extraction-method",
    "--tool-version",
    "--expected-sha256",
    "--output",
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HashSourceOutput {
    pub receipt: Value,
    pub output_path: Option<PathBuf>,
}

#[derive(Debug, Clone)]
struct SourceMetadata {
    source_id: String,
    publisher: String,
    source_uri: String,
    publication_id: String,
    vintage: String,
    captured_at: String,
    rights_status: String,
    rights_evidence_uri: String,
    evaluation_allowed: bool,
    redistribution_allowed: bool,
    locator_id: String,
    locator: String,
    extraction_method: String,
    tool_version: String,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_sha256(text: &str) -> bool {
    text.len() == 64 && text.bytes().all(|b| b.is_ascii_hexdigit())
}

fn hex_encode(bytes: &[u8]) -> String {
    let mut hex = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        use std::fmt::Write as _;
        // Writing to a String is infallible.
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

#[cfg(test)]
fn sha256_hex(bytes: &[u8]) -> String {
    hex_encode(&Sha256::digest(bytes))
}

fn valid_rfc3339(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() < 20
        || bytes.get(4) != Some(&b'-')
        || bytes.get(7) != Some(&b'-')
        || !matches!(bytes.get(10), Some(b'T' | b't'))
        || bytes.get(13) != Some(&b':')
        || bytes.get(16) != Some(&b':')
    {
        return false;
    }
    let number = |start: usize, length: usize| -> Option<u32> {
        let slice = bytes.get(start..start + length)?;
        if !slice.iter().all(u8::is_ascii_digit) {
            return None;
        }
        std::str::from_utf8(slice).ok()?.parse().ok()
    };
    let (Some(month), Some(day), Some(hour), Some(minute), Some(second)) = (
        number(5, 2), number(8, 2), number(11, 2), number(14, 2), number(17, 2)
    ) else { return false; };
    let Some(year) = number(0, 4) else { return false; };
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let month_days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    if day == 0 || day > month_days || hour > 23 || minute > 59 || second > 59 {
        return false;
    }

    let mut cursor = 19usize;
    if bytes.get(cursor) == Some(&b'.') {
        cursor += 1;
        let start = cursor;
        while bytes.get(cursor).is_some_and(u8::is_ascii_digit) { cursor += 1; }
        if cursor == start { return false; }
    }
    match bytes.get(cursor) {
        Some(b'Z' | b'z') => cursor + 1 == bytes.len(),
        Some(b'+' | b'-') => {
            if cursor + 6 != bytes.len() || bytes.get(cursor + 3) != Some(&b':') {
                return false;
            }
            let Some(offset_hour) = number(cursor + 1, 2) else { return false; };
            let Some(offset_minute) = number(cursor + 4, 2) else { return false; };
            offset_hour <= 23 && offset_minute <= 59
        }
        _ => false,
    }
}

fn rights_are_coherent(metadata: &SourceMetadata) -> bool {
    match metadata.rights_status.as_str() {
        "open_reuse" => metadata.evaluation_allowed && metadata.redistribution_allowed,
        "permission_granted" => metadata.evaluation_allowed,
        "evaluation_only" => metadata.evaluation_allowed && !metadata.redistribution_allowed,
        "restricted" | "prohibited" | "unknown" => {
            !metadata.evaluation_allowed && !metadata.redistribution_allowed
        }
        _ => false,
    }
}

fn required<'a>(values: &'a BTreeMap<String, String>, flag: &str) -> Result<&'a str, String> {
    let value = values.get(flag).map(String::as_str).ok_or_else(|| format!("missing required argument {flag}"))?;
    if !nonempty(value) {
        return Err(format!("{flag} must be nonempty"));
    }
    Ok(value)
}

fn required_bool(values: &BTreeMap<String, String>, flag: &str) -> Result<bool, String> {
    match required(values, flag)? {
        "true" => Ok(true),
        "false" => Ok(false),
        other => Err(format!("{flag} must be true or false, got {other:?}")),
    }
}

fn parse_arguments(args: &[String]) -> Result<BTreeMap<String, String>, String> {
    if args.len() == 1 && (args[0] == "--help" || args[0] == "-h") {
        return Err(HASH_SOURCE_USAGE.to_owned());
    }
    if args.len() % 2 != 0 {
        return Err("each hash-source flag must have exactly one value".to_owned());
    }
    let mut values = BTreeMap::new();
    for pair in args.chunks_exact(2) {
        let flag = &pair[0];
        if !flag.starts_with("--") || !KNOWN_FLAGS.contains(&flag.as_str()) || flag == "--help" {
            return Err(format!("unknown hash-source argument: {flag}"));
        }
        if values.insert(flag.clone(), pair[1].clone()).is_some() {
            return Err(format!("duplicate hash-source argument: {flag}"));
        }
    }
    Ok(values)
}

/// Parse the hash-source CLI flags, hash the actual input file, optionally
/// compare a supplied digest, and build a JSON source record for NRVCA-016.
pub fn receipt_from_cli(args: &[String]) -> Result<HashSourceOutput, String> {
    let values = parse_arguments(args)?;
    let metadata = SourceMetadata {
        source_id: required(&values, "--source-id")?.to_owned(),
        publisher: required(&values, "--publisher")?.to_owned(),
        source_uri: required(&values, "--source-uri")?.to_owned(),
        publication_id: required(&values, "--publication-id")?.to_owned(),
        vintage: required(&values, "--vintage")?.to_owned(),
        captured_at: required(&values, "--captured-at")?.to_owned(),
        rights_status: required(&values, "--rights-status")?.to_owned(),
        rights_evidence_uri: required(&values, "--rights-evidence-uri")?.to_owned(),
        evaluation_allowed: required_bool(&values, "--evaluation-allowed")?,
        redistribution_allowed: required_bool(&values, "--redistribution-allowed")?,
        locator_id: required(&values, "--locator-id")?.to_owned(),
        locator: required(&values, "--locator")?.to_owned(),
        extraction_method: required(&values, "--extraction-method")?.to_owned(),
        tool_version: required(&values, "--tool-version")?.to_owned(),
    };
    if !valid_rfc3339(&metadata.captured_at) {
        return Err("--captured-at must be a valid RFC 3339 timestamp with a timezone".to_owned());
    }
    if !rights_are_coherent(&metadata) {
        return Err("rights status/evaluation/redistribution flags are inconsistent; hash was not admitted as a valid source receipt".to_owned());
    }

    let expected = values.get("--expected-sha256").map(String::as_str);
    if expected.is_some_and(|digest| !valid_sha256(digest)) {
        return Err("--expected-sha256 must be exactly 64 hexadecimal characters".to_owned());
    }
    let input = required(&values, "--file")?;
    let receipt = hash_source_file(Path::new(input), &metadata, expected)?;
    let output_path = values.get("--output").map(PathBuf::from);
    Ok(HashSourceOutput { receipt, output_path })
}

fn hash_source_file(
    input_path: &Path,
    metadata: &SourceMetadata,
    expected_sha256: Option<&str>,
) -> Result<Value, String> {
    let resolved = fs::canonicalize(input_path)
        .map_err(|error| format!("cannot resolve source path {}: {error}", input_path.display()))?;
    let mut file = File::open(&resolved)
        .map_err(|error| format!("cannot open source bytes {}: {error}", resolved.display()))?;
    let before = file.metadata().map_err(|error| format!("cannot stat source file: {error}"))?;
    if !before.is_file() {
        return Err(format!("source input must be a regular file: {}", resolved.display()));
    }
    let before_modified = before.modified().map_err(|error| format!("cannot read source modification time: {error}"))?;
    let before_size = before.len();

    let mut hasher = Sha256::new();
    let mut buffer = [0u8; HASH_BUFFER_BYTES];
    let mut bytes_hashed = 0u64;
    loop {
        let count = file.read(&mut buffer)
            .map_err(|error| format!("failed while reading source bytes: {error}"))?;
        if count == 0 { break; }
        hasher.update(&buffer[..count]);
        bytes_hashed = bytes_hashed.checked_add(count as u64)
            .ok_or_else(|| "source byte count overflow".to_owned())?;
    }

    let after = file.metadata().map_err(|error| format!("cannot restat source file: {error}"))?;
    let after_modified = after.modified().map_err(|error| format!("cannot reread source modification time: {error}"))?;
    if bytes_hashed != before_size || after.len() != before_size || before_modified != after_modified {
        return Err("source file size or modification time changed during hashing; freeze/copy the file and retry".to_owned());
    }

    let digest_bytes = hasher.finalize();
    let digest = hex_encode(&digest_bytes);
    if let Some(expected) = expected_sha256 {
        if !digest.eq_ignore_ascii_case(expected) {
            return Err(format!("source SHA-256 mismatch: expected {}, computed {}", expected.to_ascii_lowercase(), digest));
        }
    }

    let hashed_at_unix_seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| format!("system clock is before Unix epoch: {error}"))?
        .as_secs();
    let file_name = resolved.file_name().and_then(|name| name.to_str()).unwrap_or("<non-utf8-filename>");
    let source = json!({
        "source_id": metadata.source_id,
        "publisher": metadata.publisher,
        "source_uri": metadata.source_uri,
        "publication_id": metadata.publication_id,
        "vintage": metadata.vintage,
        "captured_at": metadata.captured_at,
        "bytes_acquired": true,
        "sha256": digest,
        "file_size_bytes": bytes_hashed,
        "extraction": {
            "method": metadata.extraction_method,
            "tool_version": metadata.tool_version
        },
        "locators": [{
            "locator_id": metadata.locator_id,
            "locator": metadata.locator
        }],
        "rights": {
            "status": metadata.rights_status,
            "evaluation_allowed": metadata.evaluation_allowed,
            "redistribution_allowed": metadata.redistribution_allowed,
            "evidence_uri": metadata.rights_evidence_uri
        },
        "acquisition_receipt": {
            "method": "local_file_sha256_stream",
            "resolved_path": resolved.to_string_lossy(),
            "file_name": file_name,
            "bytes_hashed": bytes_hashed,
            "hashed_at_unix_seconds": hashed_at_unix_seconds,
            "consistency_check": "byte_count_size_and_modified_time_unchanged_during_read"
        }
    });
    Ok(source)
}

/// Create the receipt file without overwriting an existing path. This prevents a
/// second capture from silently replacing the first receipt.
pub fn write_receipt_new(path: &Path, receipt: &Value) -> Result<(), String> {
    let serialized = serde_json::to_vec_pretty(receipt)
        .map_err(|error| format!("cannot serialize source receipt: {error}"))?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)
        .map_err(|error| format!("cannot create receipt {} without overwriting: {error}", path.display()))?;
    if let Err(error) = file.write_all(&serialized).and_then(|_| file.write_all(b"\n")).and_then(|_| file.sync_all()) {
        let _ = fs::remove_file(path);
        return Err(format!("cannot persist receipt {}: {error}", path.display()));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT: AtomicU64 = AtomicU64::new(0);

    struct TempFile(PathBuf);
    impl TempFile {
        fn with_bytes(bytes: &[u8]) -> Self {
            let id = NEXT.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!("nrvca-intake-{}-{id}.bin", std::process::id()));
            fs::write(&path, bytes).expect("write test bytes");
            Self(path)
        }
    }
    impl Drop for TempFile {
        fn drop(&mut self) { let _ = fs::remove_file(&self.0); }
    }

    fn args_for(path: &Path) -> Vec<String> {
        vec![
            "--file".into(), path.display().to_string(),
            "--source-id".into(), "source-test".into(),
            "--publisher".into(), "Test publisher".into(),
            "--source-uri".into(), "https://example.invalid/source.pdf".into(),
            "--publication-id".into(), "TEST-2026".into(),
            "--vintage".into(), "2026-Q2".into(),
            "--captured-at".into(), "2026-10-22T17:00:00+02:00".into(),
            "--rights-status".into(), "open_reuse".into(),
            "--rights-evidence-uri".into(), "https://example.invalid/rights".into(),
            "--evaluation-allowed".into(), "true".into(),
            "--redistribution-allowed".into(), "true".into(),
            "--locator-id".into(), "table-1-row-2".into(),
            "--locator".into(), "page 4, table 1, row 2".into(),
            "--extraction-method".into(), "PDF text extraction".into(),
            "--tool-version".into(), "test-intake-0".into(),
        ]
    }

    #[test]
    fn capture_timestamp_requires_timezone_and_valid_calendar_fields() {
        assert!(valid_rfc3339("2026-10-22T17:00:00+02:00"));
        assert!(valid_rfc3339("2024-02-29T23:59:59Z"));
        assert!(!valid_rfc3339("2024-02-29T23:59:60Z"));
        assert!(!valid_rfc3339("2026-02-29T17:00:00Z"));
        assert!(!valid_rfc3339("2026-10-22T17:00:00"));
        assert!(!valid_rfc3339("2026-13-22T17:00:00Z"));
    }

    #[test]
    fn sha256_known_answer_matches_rfc_compatible_vector() {
        assert_eq!(sha256_hex(b"abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
        assert_eq!(sha256_hex(b""), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    }

    #[test]
    fn actual_file_bytes_produce_a_source_record_compatible_with_nrvc_a016() {
        let temp = TempFile::with_bytes(b"real bytes in a synthetic test");
        let parsed = receipt_from_cli(&args_for(&temp.0)).expect("source receipt");
        let bytes = b"real bytes in a synthetic test";
        assert_eq!(parsed.receipt["bytes_acquired"], true);
        assert_eq!(parsed.receipt["file_size_bytes"], bytes.len() as u64);
        assert_eq!(parsed.receipt["sha256"], sha256_hex(bytes));
        assert_eq!(parsed.receipt["acquisition_receipt"]["bytes_hashed"], bytes.len() as u64);
        assert!(parsed.receipt["acquisition_receipt"]["resolved_path"].as_str().unwrap().ends_with("nrvca-intake"));
        assert!(parsed.receipt["rights"]["evaluation_allowed"].as_bool().unwrap());
        let admission = crate::external_validity::validate_source_record(&parsed.receipt, "/source");
        assert!(admission.is_empty(), "receipt should satisfy NRVCA-016 source manifest: {admission:?}");
        assert_eq!(parsed.output_path, None);
    }

    #[test]
    fn expected_digest_mismatch_fails_without_returning_a_receipt() {
        let temp = TempFile::with_bytes(b"test content");
        let mut args = args_for(&temp.0);
        args.extend(["--expected-sha256".into(), "0".repeat(64)]);
        let error = receipt_from_cli(&args).unwrap_err();
        assert!(error.contains("SHA-256 mismatch"));
    }

    #[test]
    fn malformed_digest_and_incoherent_rights_are_rejected() {
        let temp = TempFile::with_bytes(b"test content");
        let mut args = args_for(&temp.0);
        args.extend(["--expected-sha256".into(), "xyz".into()]);
        assert!(receipt_from_cli(&args).unwrap_err().contains("64 hexadecimal"));

        let mut args = args_for(&temp.0);
        let index = args.iter().position(|item| item == "--redistribution-allowed").unwrap() + 1;
        args[index] = "false".into();
        assert!(receipt_from_cli(&args).unwrap_err().contains("rights status"));
    }

    #[test]
    fn duplicate_unknown_or_missing_arguments_are_rejected() {
        let temp = TempFile::with_bytes(b"test content");
        let mut duplicate = args_for(&temp.0);
        duplicate.extend(["--publisher".into(), "another publisher".into()]);
        assert!(receipt_from_cli(&duplicate).unwrap_err().contains("duplicate"));

        let mut unknown = args_for(&temp.0);
        unknown.extend(["--trust-me".into(), "yes".into()]);
        assert!(receipt_from_cli(&unknown).unwrap_err().contains("unknown"));

        assert!(receipt_from_cli(&["--file".into()]).unwrap_err().contains("exactly one value"));
    }

    #[test]
    fn receipt_writer_does_not_overwrite_an_existing_file() {
        let id = NEXT.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!("nrvca-receipt-{}-{id}.json", std::process::id()));
        fs::write(&path, b"preserve-existing").expect("write existing sentinel");
        let error = write_receipt_new(&path, &json!({"sha256": "test"})).unwrap_err();
        assert!(error.contains("without overwriting"));
        assert_eq!(fs::read(&path).expect("read sentinel"), b"preserve-existing");
        let _ = fs::remove_file(path);
    }

    #[test]
    fn expected_digest_is_case_insensitive_and_reproducible() {
        let temp = TempFile::with_bytes(b"same captured bytes");
        let expected = sha256_hex(b"same captured bytes").to_uppercase();
        let mut args = args_for(&temp.0);
        args.extend(["--expected-sha256".into(), expected]);
        let first = receipt_from_cli(&args).unwrap();
        let second = receipt_from_cli(&args).unwrap();
        assert_eq!(first.receipt["sha256"], second.receipt["sha256"]);
    }
}
