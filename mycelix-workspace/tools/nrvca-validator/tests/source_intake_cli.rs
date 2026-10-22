use serde_json::Value;
use std::{
    fs::{self, OpenOptions},
    io::Write,
    path::PathBuf,
    process::{Command, Output},
    sync::atomic::{AtomicU64, Ordering},
};

static NEXT: AtomicU64 = AtomicU64::new(0);

struct TempFile(PathBuf);

impl TempFile {
    fn with_bytes(bytes: &[u8]) -> Self {
        let id = NEXT.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "nrvca-intake-cli-{}-{id}.bin",
            std::process::id()
        ));
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .expect("create unique temporary file");
        file.write_all(bytes).expect("write temporary bytes");
        file.sync_all().expect("sync temporary bytes");
        Self(path)
    }

}

impl Drop for TempFile {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

fn args_for(path: &std::path::Path) -> Vec<String> {
    vec![
        "--file".into(), path.display().to_string(),
        "--source-id".into(), "source-cli-test".into(),
        "--publisher".into(), "Synthetic test publisher".into(),
        "--source-uri".into(), "https://example.invalid/source.pdf".into(),
        "--publication-id".into(), "TEST-2026".into(),
        "--vintage".into(), "2026-Q2".into(),
        "--captured-at".into(), "2026-10-22T17:00:00+02:00".into(),
        "--rights-status".into(), "open_reuse".into(),
        "--rights-evidence-uri".into(), "https://example.invalid/rights".into(),
        "--evaluation-allowed".into(), "true".into(),
        "--redistribution-allowed".into(), "true".into(),
        "--locator-id".into(), "table-row".into(),
        "--locator".into(), "Synthetic test page 1, table 1, row 1".into(),
        "--extraction-method".into(), "synthetic CLI test".into(),
        "--tool-version".into(), "test-only-v0".into(),
    ]
}

fn run(args: &[String]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_nrvca-validator"))
        .arg("hash-source")
        .args(args)
        .output()
        .expect("execute nrvca-validator binary")
}

#[test]
fn cli_hashes_actual_bytes_and_emits_parseable_source_record() {
    let source = TempFile::with_bytes(b"abc");
    let output = run(&args_for(&source.0));
    assert!(output.status.success(), "stderr: {}", String::from_utf8_lossy(&output.stderr));
    let receipt: Value = serde_json::from_slice(&output.stdout).expect("stdout is source receipt JSON");
    assert_eq!(receipt["sha256"], "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    assert_eq!(receipt["file_size_bytes"], 3);
    assert_eq!(receipt["bytes_acquired"], true);
}

#[test]
fn cli_expected_digest_mismatch_exits_unsuccessfully_without_receipt_stdout() {
    let source = TempFile::with_bytes(b"abc");
    let mut args = args_for(&source.0);
    args.extend(["--expected-sha256".into(), "0".repeat(64)]);
    let output = run(&args);
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(String::from_utf8_lossy(&output.stderr).contains("SHA-256 mismatch"));
}

#[test]
fn cli_never_overwrites_existing_receipt_output() {
    let source = TempFile::with_bytes(b"abc");
    let existing = TempFile::with_bytes(b"keep-this-sentinel");
    let mut args = args_for(&source.0);
    args.extend(["--output".into(), existing.0.display().to_string()]);
    let output = run(&args);
    assert!(!output.status.success());
    assert_eq!(fs::read(&existing.0).expect("read existing receipt"), b"keep-this-sentinel");
}

#[test]
fn cli_hash_source_help_is_successful() {
    let output = run(&["--help".into()]);
    assert!(output.status.success());
    assert!(String::from_utf8_lossy(&output.stdout).contains("--expected-sha256"));
}
