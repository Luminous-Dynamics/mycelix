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
            "nrvca-store-audit-cli-{}-{id}",
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

fn run(args: &[String]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_nrvca-validator"))
        .arg("verify-store")
        .args(args)
        .output()
        .expect("execute nrvca-validator verify-store")
}

fn capture_args(input: &std::path::Path, store: &std::path::Path) -> Vec<String> {
    vec![
        "--file".into(), input.display().to_string(),
        "--store-dir".into(), store.display().to_string(),
        "--source-id".into(), "audit-test-source".into(),
        "--publisher".into(), "Synthetic audit test".into(),
        "--source-uri".into(), "https://example.invalid/source.bin".into(),
        "--publication-id".into(), "AUDIT-2026".into(),
        "--vintage".into(), "2026".into(),
        "--captured-at".into(), "2026-10-23T08:00:00+02:00".into(),
        "--rights-status".into(), "open_reuse".into(),
        "--rights-evidence-uri".into(), "https://example.invalid/rights".into(),
        "--evaluation-allowed".into(), "true".into(),
        "--redistribution-allowed".into(), "true".into(),
        "--locator-id".into(), "synthetic-locator".into(),
        "--locator".into(), "synthetic fixture bytes".into(),
        "--extraction-method".into(), "integration test".into(),
        "--tool-version".into(), "test-version".into(),
    ]
}

fn capture(root: &TempRoot, store: &std::path::Path, name: &str, bytes: &[u8]) {
    let input = root.file(name, bytes);
    let output = Command::new(env!("CARGO_BIN_EXE_nrvca-validator"))
        .arg("capture-source")
        .args(capture_args(&input, store))
        .output()
        .expect("execute capture-source");
    assert!(output.status.success(), "capture stderr: {}", String::from_utf8_lossy(&output.stderr));
}

#[test]
fn cli_reports_nonempty_store_as_pass_with_exact_counts() {
    let root = TempRoot::new();
    let store = root.0.join("store");
    capture(&root, &store, "source.bin", b"abc");
    let output = run(&["--store-dir".into(), store.display().to_string()]);
    assert!(output.status.success(), "stderr: {}", String::from_utf8_lossy(&output.stderr));
    let report: Value = serde_json::from_slice(&output.stdout).expect("valid audit JSON");
    assert_eq!(report["schema_version"], "nrvca-content-store-audit/0.1");
    assert_eq!(report["status"], "pass");
    assert_eq!(report["objects_verified"], 1);
    assert_eq!(report["objects_failed"], 0);
    assert_eq!(report["bytes_verified"], 3);
    assert_eq!(report["diagnostics"].as_array().unwrap().len(), 0);
}

#[test]
fn cli_empty_store_is_not_misreported_as_pass() {
    let root = TempRoot::new();
    let store = root.0.join("empty-store");
    fs::create_dir(&store).expect("create empty store");
    let output = run(&["--store-dir".into(), store.display().to_string()]);
    assert!(!output.status.success(), "empty store must not pass a fail-closed evidence gate");
    assert!(String::from_utf8_lossy(&output.stderr).contains("no evidence objects were verified"));
    let report: Value = serde_json::from_slice(&output.stdout).expect("valid empty report");
    assert_eq!(report["status"], "empty");
    assert_eq!(report["objects_verified"], 0);
}

#[test]
fn cli_corruption_emits_failure_report_and_returns_nonzero_without_repair() {
    let root = TempRoot::new();
    let store = root.0.join("store");
    capture(&root, &store, "source.bin", b"captured evidence");
    let shard = fs::read_dir(store.join("objects/sha256")).expect("shard root")
        .next().expect("one shard").expect("read shard").path();
    let object = fs::read_dir(shard).expect("list shard")
        .next().expect("one object").expect("read object").path();
    let permissions = fs::metadata(&object).expect("object metadata").permissions();
    let mut writable = permissions.clone();
    writable.set_readonly(false);
    fs::set_permissions(&object, writable).expect("temporarily allow tampering");
    fs::write(&object, b"corrupted evidence").expect("tamper bytes");
    let corrupted = fs::read(&object).expect("record corrupted bytes");

    let output = run(&["--store-dir".into(), store.display().to_string()]);
    assert!(!output.status.success(), "failed audit must exit nonzero");
    let report: Value = serde_json::from_slice(&output.stdout).expect("failure should still be machine-readable JSON");
    assert_eq!(report["status"], "failed");
    assert!(report["diagnostics"].as_array().unwrap().iter().any(|item| item["code"] == "STORE-DIGEST-MISMATCH"));
    assert_eq!(fs::read(object).expect("audit must not repair or rewrite object"), corrupted);
}

#[test]
fn cli_help_advertises_store_directory() {
    let output = run(&["--help".into()]);
    assert!(output.status.success());
    assert!(String::from_utf8_lossy(&output.stdout).contains("verify-store --store-dir DIR"));
}
