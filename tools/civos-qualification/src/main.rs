use civos_qualification::{
    evaluate_receipt, sha256_hex, validate_contract, verify_receipt_signature,
};
use serde_json::json;
use std::{
    env, fs,
    path::Path,
    process::{self, Command},
};

fn value_after(args: &[String], flag: &str) -> Result<String, String> {
    let index = args
        .iter()
        .position(|arg| arg == flag)
        .ok_or_else(|| format!("missing required option {flag}"))?;
    args.get(index + 1)
        .filter(|value| !value.starts_with("--"))
        .cloned()
        .ok_or_else(|| format!("option {flag} requires a value"))
}

fn read(path: &str, label: &str) -> Result<String, String> {
    fs::read_to_string(path).map_err(|error| format!("cannot read {label} at {path}: {error}"))
}

fn git_output(source_root: &Path, args: &[&str]) -> Result<String, String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(source_root)
        .args(args)
        .output()
        .map_err(|error| {
            format!(
                "could not execute git in {}: {error}",
                source_root.display()
            )
        })?;
    if !output.status.success() {
        return Err(format!(
            "git {} failed in {}: {}",
            args.join(" "),
            source_root.display(),
            String::from_utf8_lossy(&output.stderr).trim()
        ));
    }
    String::from_utf8(output.stdout)
        .map(|value| value.trim().to_string())
        .map_err(|error| format!("git returned non-UTF-8 output: {error}"))
}

fn verify_receipt_checkout(receipt_text: &str, source_root: &Path) -> Result<(), String> {
    let receipt: serde_json::Value = serde_json::from_str(receipt_text)
        .map_err(|error| format!("qualification receipt is not valid JSON: {error}"))?;
    let declared_commit = receipt
        .pointer("/source/commit")
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| "receipt is missing source.commit".to_string())?;
    let actual_commit = git_output(source_root, &["rev-parse", "--verify", "HEAD"])?;
    if declared_commit != actual_commit {
        return Err(format!(
            "receipt source commit {declared_commit} does not match checkout HEAD {actual_commit}"
        ));
    }

    let declared_tree = receipt
        .pointer("/source/tree_state")
        .and_then(serde_json::Value::as_str)
        .ok_or_else(|| "receipt is missing source.tree_state".to_string())?;
    let status = git_output(
        source_root,
        &["status", "--porcelain", "--untracked-files=all"],
    )?;
    let actual_tree_clean = status.is_empty();
    match declared_tree {
        "clean" if !actual_tree_clean => Err(
            "receipt claims a clean tree but git status reports tracked/untracked changes; keep evidence outside the source tree"
                .into(),
        ),
        "dirty" if actual_tree_clean => Err(
            "receipt claims a dirty tree but git status is clean".into(),
        ),
        "clean" | "dirty" | "unknown" => Ok(()),
        other => Err(format!("unknown source tree state {other}")),
    }
}

fn run(args: &[String]) -> Result<i32, String> {
    let Some(command) = args.first().map(String::as_str) else {
        return Err(usage().into());
    };
    match command {
        "validate-contract" => {
            let corpus_path = value_after(args, "--corpus")?;
            let schema_path = value_after(args, "--schema")?;
            let corpus = read(&corpus_path, "scenario corpus")?;
            let schema = read(&schema_path, "scenario schema")?;
            let summary = validate_contract(&corpus, &schema)?;
            println!(
                "{}",
                serde_json::to_string_pretty(&json!({
                    "status":"CONTRACT_VALID",
                    "contract_id":summary.contract_id,
                    "version":summary.version,
                    "scenario_count":summary.scenario_count,
                    "hard_gate_count":summary.hard_gate_count,
                    "behavioral_count":summary.behavioral_count,
                    "performance_baseline_count":summary.performance_baseline_count,
                    "corpus_sha256":summary.corpus_sha256,
                    "schema_sha256":summary.schema_sha256
                }))
                .map_err(|error| error.to_string())?
            );
            Ok(0)
        }
        "evaluate-receipt" => {
            let corpus_path = value_after(args, "--corpus")?;
            let schema_path = value_after(args, "--schema")?;
            let receipt_schema_path = value_after(args, "--receipt-schema")?;
            let receipt_path = value_after(args, "--receipt")?;
            let signature_path = value_after(args, "--signature")?;
            let public_key_path = value_after(args, "--trusted-public-key")?;
            let public_key_fingerprint = value_after(args, "--trusted-key-sha256")?;
            let target = value_after(args, "--target")?;
            let evidence_root = value_after(args, "--evidence-root")?;
            let source_root = value_after(args, "--source-root")?;
            let corpus = read(&corpus_path, "scenario corpus")?;
            let schema = read(&schema_path, "scenario schema")?;
            let receipt_schema = read(&receipt_schema_path, "receipt schema")?;
            let receipt = read(&receipt_path, "qualification receipt")?;
            let signature = read(&signature_path, "detached receipt signature")?;
            let public_key = fs::read(&public_key_path)
                .map_err(|error| format!("cannot read trusted public key at {public_key_path}: {error}"))?;
            // The fingerprint is pinned in independently managed trust config;
            // never take the expected fingerprint from the receipt itself.
            let signer_fingerprint = verify_receipt_signature(
                receipt.as_bytes(),
                &signature,
                &public_key,
                &public_key_fingerprint,
            )?;
            verify_receipt_checkout(&receipt, Path::new(&source_root))?;
            let mut report = evaluate_receipt(
                &corpus,
                &schema,
                &receipt_schema,
                &receipt,
                &target,
                Path::new(&evidence_root),
            )?;
            report["receipt_authentication"] = json!({
                "status": "verified",
                "algorithm": "Ed25519",
                "domain": "LUMINOUS-DYNAMICS/CIVOS-MACQ-001/RECEIPT/V0.1",
                "trusted_public_key_sha256": signer_fingerprint,
                "exact_receipt_bytes_sha256": sha256_hex(receipt.as_bytes())
            });
            let status = report
                .get("status")
                .and_then(|value| value.as_str())
                .unwrap_or("INCOMPLETE");
            println!(
                "{}",
                serde_json::to_string_pretty(&report).map_err(|error| error.to_string())?
            );
            Ok(match status {
                "QUALIFIED" => 0,
                "FAILED" => 1,
                _ => 2,
            })
        }
        "help" | "--help" | "-h" => {
            println!("{}", usage());
            Ok(0)
        }
        other => Err(format!("unknown command {other}\n{}", usage())),
    }
}

fn usage() -> &'static str {
    "CivOS fail-closed qualification tool\n\n\
civos-qualify validate-contract --corpus PATH --schema PATH\n\
civos-qualify evaluate-receipt --corpus PATH --schema PATH --receipt-schema PATH --receipt PATH --signature PATH --trusted-public-key RAW32_PATH --trusted-key-sha256 SHA256 --target TARGET --evidence-root PATH --source-root PATH\n\n\
The detached Base64 signature covers the exact receipt bytes with a domain-separated Ed25519 signature. --trusted-key-sha256 must be pinned in independent trusted configuration; never derive it from the receipt or an untrusted artifact. --trusted-public-key is exactly 32 raw bytes. Evidence paths are relative to --evidence-root and every referenced file must match its declared SHA-256. --source-root must be the exact repository checkout; the CLI matches source.commit to git HEAD and checks the claimed tree state.\n\n\
Exit codes: 0 qualified/valid, 1 hard-gate failure, 2 incomplete or invalid input."
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    match run(&args) {
        Ok(code) => process::exit(code),
        Err(error) => {
            eprintln!("civos-qualify: {error}");
            process::exit(2);
        }
    }
}
