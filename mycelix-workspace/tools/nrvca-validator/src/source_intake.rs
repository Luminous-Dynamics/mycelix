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
    sync::atomic::{AtomicU64, Ordering},
};

const HASH_BUFFER_BYTES: usize = 64 * 1024;
static NEXT_TEMP_OBJECT: AtomicU64 = AtomicU64::new(0);
pub const HASH_SOURCE_USAGE: &str = "Usage: nrvca-validator hash-source --file PATH --source-id ID --publisher NAME --source-uri URI --publication-id ID --vintage VINTAGE --captured-at RFC3339 --rights-status open_reuse|permission_granted|evaluation_only|restricted|prohibited|unknown --rights-evidence-uri URI --evaluation-allowed true|false --redistribution-allowed true|false --locator-id ID --locator TEXT --extraction-method METHOD --tool-version VERSION [--expected-sha256 HEX] [--output PATH]";
pub const CAPTURE_SOURCE_USAGE: &str = "Usage: nrvca-validator capture-source --file PATH --store-dir DIR --source-id ID --publisher NAME --source-uri URI --publication-id ID --vintage VINTAGE --captured-at RFC3339 --rights-status open_reuse|permission_granted|evaluation_only|restricted|prohibited|unknown --rights-evidence-uri URI --evaluation-allowed true|false --redistribution-allowed true|false --locator-id ID --locator TEXT --extraction-method METHOD --tool-version VERSION [--expected-sha256 HEX] [--output PATH]";
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
        || !matches!(bytes.get(10).copied(), Some(b'T' | b't'))
        || bytes.get(13) != Some(&b':')
        || bytes.get(16) != Some(&b':')
    {
        return false;
    }
    let number = |start: usize, length: usize| -> Option<u32> {
        let slice = bytes.get(start..start + length)?;
        if !slice.iter().all(|byte| byte.is_ascii_digit()) {
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
        while bytes.get(cursor).is_some_and(|byte| byte.is_ascii_digit()) { cursor += 1; }
        if cursor == start { return false; }
    }
    match bytes.get(cursor).copied() {
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

#[derive(Debug, Clone)]
struct ParsedSourceArgs {
    metadata: SourceMetadata,
    input_path: PathBuf,
    expected_sha256: Option<String>,
    output_path: Option<PathBuf>,
}

fn parse_source_args(args: &[String]) -> Result<ParsedSourceArgs, String> {
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
        return Err("rights status/evaluation/redistribution flags are inconsistent; source was not admitted".to_owned());
    }

    let expected_sha256 = values.get("--expected-sha256").cloned();
    if expected_sha256.as_deref().is_some_and(|digest| !valid_sha256(digest)) {
        return Err("--expected-sha256 must be exactly 64 hexadecimal characters".to_owned());
    }
    let input_path = PathBuf::from(required(&values, "--file")?);
    let output_path = values.get("--output").map(PathBuf::from);
    Ok(ParsedSourceArgs { metadata, input_path, expected_sha256, output_path })
}

/// Parse hash-source flags, hash the actual input file, and build a source record.
pub fn receipt_from_cli(args: &[String]) -> Result<HashSourceOutput, String> {
    let parsed = parse_source_args(args)?;
    let receipt = hash_source_file(&parsed.input_path, &parsed.metadata, parsed.expected_sha256.as_deref())?;
    Ok(HashSourceOutput { receipt, output_path: parsed.output_path })
}

fn strip_store_dir(args: &[String]) -> Result<(Vec<String>, PathBuf), String> {
    if args.len() == 1 && matches!(args[0].as_str(), "--help" | "-h") {
        return Err(CAPTURE_SOURCE_USAGE.to_owned());
    }
    if args.len() % 2 != 0 {
        return Err("each capture-source flag must have exactly one value".to_owned());
    }
    let mut filtered = Vec::with_capacity(args.len());
    let mut store_dir: Option<PathBuf> = None;
    for pair in args.chunks_exact(2) {
        if pair[0] == "--store-dir" {
            if store_dir.is_some() {
                return Err("duplicate capture-source argument: --store-dir".to_owned());
            }
            if !nonempty(&pair[1]) {
                return Err("--store-dir must be nonempty".to_owned());
            }
            store_dir = Some(PathBuf::from(&pair[1]));
        } else {
            filtered.push(pair[0].clone());
            filtered.push(pair[1].clone());
        }
    }
    let store_dir = store_dir.ok_or_else(|| "missing required argument --store-dir".to_owned())?;
    Ok((filtered, store_dir))
}

/// Capture actual input bytes into a content-addressed store in the same pass
/// that computes SHA-256. The public receipt contains only a store-relative path.
pub fn capture_from_cli(args: &[String]) -> Result<HashSourceOutput, String> {
    let (source_args, store_dir) = strip_store_dir(args)?;
    let parsed = parse_source_args(&source_args)?;
    let receipt = capture_source_file(
        &parsed.input_path,
        &store_dir,
        &parsed.metadata,
        parsed.expected_sha256.as_deref(),
    )?;
    Ok(HashSourceOutput { receipt, output_path: parsed.output_path })
}

struct TemporaryObject(PathBuf);
impl Drop for TemporaryObject {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

fn create_temporary_object(staging_dir: &Path) -> Result<(PathBuf, File), String> {
    for _ in 0..32 {
        let sequence = NEXT_TEMP_OBJECT.fetch_add(1, Ordering::Relaxed);
        let path = staging_dir.join(format!(".capture-{}-{sequence}.tmp", std::process::id()));
        match OpenOptions::new().write(true).read(true).create_new(true).open(&path) {
            Ok(file) => return Ok((path, file)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(format!("cannot create temporary content object {}: {error}", path.display())),
        }
    }
    Err("could not allocate a unique temporary content object after 32 attempts".to_owned())
}

fn hash_regular_file(path: &Path) -> Result<(String, u64), String> {
    let metadata = fs::symlink_metadata(path)
        .map_err(|error| format!("cannot stat existing content object {}: {error}", path.display()))?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(format!("content-addressed object must be a regular non-symlink file: {}", path.display()));
    }
    let mut file = File::open(path)
        .map_err(|error| format!("cannot open existing content object {}: {error}", path.display()))?;
    let opened = file.metadata().map_err(|error| format!("cannot stat opened content object: {error}"))?;
    if !opened.is_file() {
        return Err(format!("opened content object is not regular: {}", path.display()));
    }
    let initial_size = opened.len();
    let initial_modified = opened.modified().map_err(|error| format!("cannot read object modification time: {error}"))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; HASH_BUFFER_BYTES];
    let mut count = 0u64;
    loop {
        let read = file.read(&mut buffer).map_err(|error| format!("cannot verify existing object: {error}"))?;
        if read == 0 { break; }
        hasher.update(&buffer[..read]);
        count = count.checked_add(read as u64).ok_or_else(|| "object byte count overflow".to_owned())?;
    }
    let after = file.metadata().map_err(|error| format!("cannot restat object: {error}"))?;
    let after_modified = after.modified().map_err(|error| format!("cannot reread object modification time: {error}"))?;
    if count != initial_size || after.len() != initial_size || initial_modified != after_modified {
        return Err(format!("existing content object changed during verification: {}", path.display()));
    }
    Ok((hex_encode(&hasher.finalize()), count))
}

#[cfg(unix)]
fn sync_directory(path: &Path) -> Result<(), String> {
    File::open(path)
        .and_then(|file| file.sync_all())
        .map_err(|error| format!("cannot sync directory {}: {error}", path.display()))
}

#[cfg(not(unix))]
fn sync_directory(_path: &Path) -> Result<(), String> {
    // Standard library has no portable directory-handle fsync API.
    Ok(())
}

fn capture_source_file(
    input_path: &Path,
    store_dir: &Path,
    metadata: &SourceMetadata,
    expected_sha256: Option<&str>,
) -> Result<Value, String> {
    let input_resolved = fs::canonicalize(input_path)
        .map_err(|error| format!("cannot resolve input file {}: {error}", input_path.display()))?;
    let path_metadata = fs::metadata(&input_resolved)
        .map_err(|error| format!("cannot stat input path {}: {error}", input_resolved.display()))?;
    if !path_metadata.is_file() {
        return Err(format!("source input must be a regular file: {}", input_resolved.display()));
    }
    let mut input = File::open(&input_resolved)
        .map_err(|error| format!("cannot open source bytes {}: {error}", input_resolved.display()))?;
    let before = input.metadata().map_err(|error| format!("cannot stat opened input: {error}"))?;
    if !before.is_file() {
        return Err(format!("opened source input is not a regular file: {}", input_resolved.display()));
    }
    let before_size = before.len();
    let before_modified = before.modified().map_err(|error| format!("cannot read input modification time: {error}"))?;

    fs::create_dir_all(store_dir)
        .map_err(|error| format!("cannot create content store {}: {error}", store_dir.display()))?;
    let store_root = fs::canonicalize(store_dir)
        .map_err(|error| format!("cannot resolve content store {}: {error}", store_dir.display()))?;
    if !fs::metadata(&store_root).map_err(|error| format!("cannot stat content store: {error}"))?.is_dir() {
        return Err("content store root must be a directory".to_owned());
    }
    let object_root = store_root.join("objects").join("sha256");
    fs::create_dir_all(&object_root)
        .map_err(|error| format!("cannot create object directory {}: {error}", object_root.display()))?;
    let object_root = fs::canonicalize(&object_root)
        .map_err(|error| format!("cannot resolve object directory: {error}"))?;
    if !object_root.starts_with(&store_root) {
        return Err("resolved object directory escapes the selected content store".to_owned());
    }

    let (temp_path, mut temp_file) = create_temporary_object(&object_root)?;
    let _temporary_object = TemporaryObject(temp_path.clone());
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; HASH_BUFFER_BYTES];
    let mut bytes_captured = 0u64;
    loop {
        let count = input.read(&mut buffer).map_err(|error| format!("failed reading source bytes: {error}"))?;
        if count == 0 { break; }
        temp_file.write_all(&buffer[..count]).map_err(|error| format!("failed writing staged object: {error}"))?;
        hasher.update(&buffer[..count]);
        bytes_captured = bytes_captured.checked_add(count as u64)
            .ok_or_else(|| "captured source byte count overflow".to_owned())?;
    }
    temp_file.flush().map_err(|error| format!("cannot flush staged object: {error}"))?;
    temp_file.sync_all().map_err(|error| format!("cannot sync staged object: {error}"))?;

    let after = input.metadata().map_err(|error| format!("cannot restat input file: {error}"))?;
    let after_modified = after.modified().map_err(|error| format!("cannot reread input modification time: {error}"))?;
    if bytes_captured != before_size || after.len() != before_size || before_modified != after_modified {
        return Err("source file size or modification time changed during capture; use an immutable copy and retry".to_owned());
    }

    let digest = hex_encode(&hasher.finalize());
    if let Some(expected) = expected_sha256 {
        if !digest.eq_ignore_ascii_case(expected) {
            return Err(format!("source SHA-256 mismatch: expected {}, captured {}", expected.to_ascii_lowercase(), digest));
        }
    }

    let prefix = &digest[..2];
    let shard_path = object_root.join(prefix);
    fs::create_dir_all(&shard_path)
        .map_err(|error| format!("cannot create digest shard: {error}"))?;
    let shard_path = fs::canonicalize(&shard_path)
        .map_err(|error| format!("cannot resolve digest shard: {error}"))?;
    if !shard_path.starts_with(&object_root) {
        return Err("resolved digest shard escapes the object root".to_owned());
    }
    let object_path = shard_path.join(&digest);
    let relative_object_path = PathBuf::from("objects").join("sha256").join(prefix).join(&digest);

    // Make the staged inode read-only before the atomic no-overwrite promotion.
    let mut temp_permissions = temp_file.metadata().map_err(|error| format!("cannot stat staged object: {error}"))?.permissions();
    temp_permissions.set_readonly(true);
    temp_file.set_permissions(temp_permissions).map_err(|error| format!("cannot protect staged object: {error}"))?;
    temp_file.sync_all().map_err(|error| format!("cannot sync protected staged object: {error}"))?;
    drop(temp_file);

    match fs::hard_link(&temp_path, &object_path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            let object_meta = fs::symlink_metadata(&object_path)
                .map_err(|error| format!("cannot inspect pre-existing content object: {error}"))?;
            if object_meta.file_type().is_symlink() || !object_meta.is_file() {
                return Err(format!("pre-existing digest path is not a regular file: {}", object_path.display()));
            }
            let (existing_digest, existing_size) = hash_regular_file(&object_path)?;
            if existing_digest != digest || existing_size != bytes_captured {
                return Err(format!("pre-existing content object failed digest/size verification: {}", object_path.display()));
            }
        }
        Err(error) => return Err(format!("atomic no-overwrite promotion failed for {}: {error}", object_path.display())),
    }
    sync_directory(&shard_path)?;
    let (stored_digest, stored_size) = hash_regular_file(&object_path)?;
    if stored_digest != digest || stored_size != bytes_captured {
        return Err("stored object did not verify after promotion".to_owned());
    }

    let hashed_at_unix_seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|error| format!("system clock is before Unix epoch: {error}"))?
        .as_secs();
    let relative_object_path_string = relative_object_path.to_string_lossy().replace('\\', "/");
    Ok(json!({
        "source_id": metadata.source_id,
        "publisher": metadata.publisher,
        "source_uri": metadata.source_uri,
        "publication_id": metadata.publication_id,
        "vintage": metadata.vintage,
        "captured_at": metadata.captured_at,
        "bytes_acquired": true,
        "sha256": digest,
        "file_size_bytes": bytes_captured,
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
            "method": "content_addressed_store_capture_sha256",
            "object_uri": format!("sha256:{digest}"),
            "object_path": relative_object_path_string,
            "bytes_hashed": bytes_captured,
            "hashed_at_unix_seconds": hashed_at_unix_seconds,
            "consistency_check": "captured_stream_and_stored_object_digest_size_verified",
            "storage_policy": "read_only_by_default_no_overwrite_verify_existing_object"
        }
    }))
}

fn hash_source_file(
    input_path: &Path,
    metadata: &SourceMetadata,
    expected_sha256: Option<&str>,
) -> Result<Value, String> {
    let resolved = fs::canonicalize(input_path)
        .map_err(|error| format!("cannot resolve source path {}: {error}", input_path.display()))?;
    // Check the canonical target before opening it: opening a FIFO first can
    // block waiting for a writer, even though it is not a valid evidence file.
    let path_metadata = fs::metadata(&resolved)
        .map_err(|error| format!("cannot stat source path {}: {error}", resolved.display()))?;
    if !path_metadata.is_file() {
        return Err(format!("source input must be a regular file: {}", resolved.display()));
    }
    let mut file = File::open(&resolved)
        .map_err(|error| format!("cannot open source bytes {}: {error}", resolved.display()))?;
    let before = file.metadata().map_err(|error| format!("cannot stat opened source file: {error}"))?;
    if !before.is_file() {
        return Err(format!("opened source input is not a regular file: {}", resolved.display()));
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

    struct TempDir(PathBuf);
    impl TempDir {
        fn new() -> Self {
            let id = NEXT.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!("nrvca-store-{}-{id}", std::process::id()));
            fs::create_dir(&path).expect("create unique test store");
            Self(path)
        }
    }
    impl Drop for TempDir {
        fn drop(&mut self) { let _ = fs::remove_dir_all(&self.0); }
    }

    fn capture_args_for(path: &Path, store: &Path) -> Vec<String> {
        let mut args = args_for(path);
        args.extend(["--store-dir".into(), store.display().to_string()]);
        args
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
    fn actual_file_bytes_produce_a_source_record_compatible_with_nrvca_016() {
        let temp = TempFile::with_bytes(b"real bytes in a synthetic test");
        let parsed = receipt_from_cli(&args_for(&temp.0)).expect("source receipt");
        let bytes = b"real bytes in a synthetic test";
        assert_eq!(parsed.receipt["bytes_acquired"], true);
        assert_eq!(parsed.receipt["file_size_bytes"], bytes.len() as u64);
        assert_eq!(parsed.receipt["sha256"], sha256_hex(bytes));
        assert_eq!(parsed.receipt["acquisition_receipt"]["bytes_hashed"], bytes.len() as u64);
        assert!(parsed.receipt["acquisition_receipt"]["resolved_path"].as_str().unwrap().contains("nrvca-intake-"));
        assert!(parsed.receipt["rights"]["evaluation_allowed"].as_bool().unwrap());
        let admission = crate::external_validity::validate_source_record(&parsed.receipt, "/source");
        assert!(admission.is_empty(), "receipt should satisfy NRVCA-016 source manifest: {admission:?}");
        assert_eq!(parsed.output_path, None);
    }

    #[test]
    fn malformed_capture_timestamp_is_rejected_before_hashing() {
        let temp = TempFile::with_bytes(b"test content");
        let mut args = args_for(&temp.0);
        let index = args.iter().position(|item| item == "--captured-at").unwrap() + 1;
        args[index] = "2026-10-22T17:00:00".into();
        assert!(receipt_from_cli(&args).unwrap_err().contains("RFC 3339"));
    }

    #[test]
    fn directory_input_is_rejected_before_file_content_open() {
        let args = args_for(&std::env::temp_dir());
        let error = receipt_from_cli(&args).unwrap_err();
        assert!(error.contains("regular file"));
    }

    #[test]
    fn capture_stores_the_exact_bytes_under_the_computed_digest() {
        let input = TempFile::with_bytes(b"content-addressed test bytes");
        let store = TempDir::new();
        let output = capture_from_cli(&capture_args_for(&input.0, &store.0)).expect("capture source bytes");
        let digest = sha256_hex(b"content-addressed test bytes");
        assert_eq!(output.receipt["sha256"], digest);
        assert_eq!(output.receipt["file_size_bytes"], b"content-addressed test bytes".len() as u64);
        assert_eq!(output.receipt["acquisition_receipt"]["object_uri"], format!("sha256:{digest}"));
        assert_eq!(output.receipt["acquisition_receipt"]["object_path"], format!("objects/sha256/{}/{digest}", &digest[..2]));
        assert!(output.receipt["acquisition_receipt"].get("resolved_path").is_none());

        let relative = output.receipt["acquisition_receipt"]["object_path"].as_str().unwrap();
        let object = store.0.join(relative);
        assert_eq!(fs::read(&object).expect("read captured object"), b"content-addressed test bytes");
        assert!(fs::metadata(object).expect("object metadata").permissions().readonly());
        let admission = crate::external_validity::validate_source_record(&output.receipt, "/source");
        assert!(admission.is_empty(), "captured receipt must meet source admission contract: {admission:?}");
    }

    #[test]
    fn identical_bytes_reuse_verified_object_without_overwriting_it() {
        let input_a = TempFile::with_bytes(b"same bytes");
        let input_b = TempFile::with_bytes(b"same bytes");
        let store = TempDir::new();
        let first = capture_from_cli(&capture_args_for(&input_a.0, &store.0)).expect("first capture");
        let second = capture_from_cli(&capture_args_for(&input_b.0, &store.0)).expect("repeat identical capture");
        assert_eq!(first.receipt["sha256"], second.receipt["sha256"]);
        assert_eq!(first.receipt["acquisition_receipt"]["object_path"], second.receipt["acquisition_receipt"]["object_path"]);
        let object_root = store.0.join("objects").join("sha256");
        let files: Vec<_> = fs::read_dir(object_root.join(first.receipt["sha256"].as_str().unwrap().get(..2).unwrap()))
            .expect("read digest shard").collect();
        assert_eq!(files.len(), 1);
    }

    #[test]
    fn mismatch_does_not_promote_a_content_object() {
        let input = TempFile::with_bytes(b"must not be promoted");
        let store = TempDir::new();
        let mut args = capture_args_for(&input.0, &store.0);
        args.extend(["--expected-sha256".into(), "0".repeat(64)]);
        let error = capture_from_cli(&args).unwrap_err();
        assert!(error.contains("SHA-256 mismatch"));
        let object_root = store.0.join("objects").join("sha256");
        assert_eq!(fs::read_dir(object_root).expect("read empty object root").count(), 0);
    }

    #[test]
    fn tampered_preexisting_digest_object_is_rejected_on_reuse() {
        let input = TempFile::with_bytes(b"expected immutable content");
        let store = TempDir::new();
        let digest = sha256_hex(b"expected immutable content");
        let shard = store.0.join("objects").join("sha256").join(&digest[..2]);
        fs::create_dir_all(&shard).expect("create shard");
        let object = shard.join(&digest);
        fs::write(&object, b"corrupt bytes").expect("write counterfeit object");
        let error = capture_from_cli(&capture_args_for(&input.0, &store.0)).unwrap_err();
        assert!(error.contains("failed digest/size verification") || error.contains("stored object"));
    }

    #[test]
    fn duplicate_store_dir_and_escaping_store_symlinks_fail_closed() {
        let input = TempFile::with_bytes(b"store containment");
        let store = TempDir::new();
        let mut duplicate = capture_args_for(&input.0, &store.0);
        duplicate.extend(["--store-dir".into(), store.0.display().to_string()]);
        assert!(capture_from_cli(&duplicate).unwrap_err().contains("duplicate"));

        #[cfg(unix)]
        {
            use std::os::unix::fs::symlink;
            let outside = TempDir::new();
            let linked_store = TempDir::new();
            fs::remove_dir_all(linked_store.0.join("objects")).ok();
            symlink(&outside.0, linked_store.0.join("objects")).expect("create escaping object-root symlink");
            let error = capture_from_cli(&capture_args_for(&input.0, &linked_store.0)).unwrap_err();
            assert!(error.contains("escapes the selected content store") || error.contains("escapes the object root"));
        }
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
