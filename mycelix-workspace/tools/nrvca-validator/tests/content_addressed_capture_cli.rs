use serde_json::Value;
use std::{
    fs::{self, OpenOptions},
    io::Write,
    path::PathBuf,
    process::{Command, Output},
    sync::atomic::{AtomicU64, Ordering},
};

static NEXT: AtomicU64 = AtomicU64::new(0);

struct TempRoot(PathBuf);

impl TempRoot {
    fn new() -> Self {
        let id = NEXT.fetch_add(1, Ordering::Relaxed);
        let path = std::env::temp_dir().join(format!(
            "nrvca-evidence-store-cli-{}-{id}",
            std::process::id()
        ));
        fs::create_dir(&path).expect("create unique temporary root");
        Self(path)
    }

    fn file(&self, name: &str, bytes: &[u8]) -> PathBuf {
        let path = self.0.join(name);
        let mut file = OpenOptions::new().write(true).create_new(true).open(&path)
            .expect("create test input");
        file.write_all(bytes).expect("write test input");
        file.sync_all().expect("sync test input");
        path
    }
}

impl Drop for TempRoot {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn base_args(input: &std::path::Path, store: &std::path::Path) -> Vec<String> {
    vec![
        "--file".into(), input.display().to_string(),
        "--store-dir".into(), store.display().to_string(),
        "--source-id".into(), "cli-source-1".into(),
        "--publisher".into(), "Synthetic CLI publisher".into(),
        "--source-uri".into(), "https://example.invalid/source.pdf".into(),
        "--publication-id".into(), "TEST-2026".into(),
        "--vintage".into(), "2026-Q2".into(),
        "--captured-at".into(), "2026-10-22T17:00:00+02:00".into(),
        "--rights-status".into(), "open_reuse".into(),
        "--rights-evidence-uri".into(), "https://example.invalid/rights".into(),
        "--evaluation-allowed".into(), "true".into(),
        "--redistribution-allowed".into(), "true".into(),
        "--locator-id".into(), "table-1-row-1".into(),
        "--locator".into(), "Synthetic page 1, table 1, row 1".into(),
        "--extraction-method".into(), "synthetic command integration test".into(),
        "--tool-version".into(), "cli-test-v0".into(),
    ]
}

fn run(args: &[String]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_nrvca-validator"))
        .arg("capture-source")
        .args(args)
        .output()
        .expect("execute nrvca-validator capture-source")
}

#[test]
fn cli_captures_actual_bytes_to_content_addressed_object_and_prints_relative_receipt() {
    let root = TempRoot::new();
    let input = root.file("source.bin", b"abc");
    let store = root.0.join("store");
    let output = run(&base_args(&input, &store));
    assert!(output.status.success(), "stderr: {}", String::from_utf8_lossy(&output.stderr));

    let receipt: Value = serde_json::from_slice(&output.stdout).expect("stdout must be JSON receipt");
    let digest = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    assert_eq!(receipt["sha256"], digest);
    assert_eq!(receipt["file_size_bytes"], 3);
    assert_eq!(receipt["acquisition_receipt"]["object_uri"], format!("sha256:{digest}"));
    assert!(receipt["acquisition_receipt"].get("resolved_path").is_none());

    let relative = receipt["acquisition_receipt"]["object_path"].as_str().unwrap();
    assert_eq!(relative, format!("objects/sha256/ba/{digest}"));
    let object = store.join(relative);
    assert_eq!(fs::read(&object).expect("read captured bytes"), b"abc");
    assert!(fs::metadata(object).expect("read object metadata").permissions().readonly());
}

#[test]
fn cli_digest_mismatch_fails_and_leaves_no_promoted_object() {
    let root = TempRoot::new();
    let input = root.file("source.bin", b"abc");
    let store = root.0.join("store");
    let mut args = base_args(&input, &store);
    args.extend(["--expected-sha256".into(), "0".repeat(64)]);

    let output = run(&args);
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(String::from_utf8_lossy(&output.stderr).contains("SHA-256 mismatch"));
    assert_eq!(fs::read_dir(store.join("objects/sha256")).expect("empty object root").count(), 0);
}

#[test]
fn cli_reuses_matching_existing_object_after_verification() {
    let root = TempRoot::new();
    let input1 = root.file("first.bin", b"same bytes");
    let input2 = root.file("second.bin", b"same bytes");
    let store = root.0.join("store");
    let first = run(&base_args(&input1, &store));
    assert!(first.status.success(), "stderr: {}", String::from_utf8_lossy(&first.stderr));
    let second = run(&base_args(&input2, &store));
    assert!(second.status.success(), "stderr: {}", String::from_utf8_lossy(&second.stderr));
    let first_receipt: Value = serde_json::from_slice(&first.stdout).unwrap();
    let second_receipt: Value = serde_json::from_slice(&second.stdout).unwrap();
    assert_eq!(first_receipt["sha256"], second_receipt["sha256"]);
    assert_eq!(first_receipt["acquisition_receipt"]["object_path"], second_receipt["acquisition_receipt"]["object_path"]);
    let digest = first_receipt["sha256"].as_str().unwrap();
    let shard = store.join("objects/sha256").join(&digest[..2]);
    assert_eq!(fs::read_dir(shard).expect("read digest shard").count(), 1);
}

#[test]
fn cli_capture_help_documents_store_directory() {
    let output = run(&["--help".into()]);
    assert!(output.status.success());
    assert!(String::from_utf8_lossy(&output.stdout).contains("--store-dir"));
}
