// SPDX-License-Identifier: AGPL-3.0-or-later
use johannesburg_water_event_graph_validator::{sha256_hex, validate};
use serde_json::{Value, json};
use std::env;
use std::fs;
use std::path::PathBuf;

fn read_json(path: &PathBuf) -> Result<(Vec<u8>, Value), String> {
    let bytes = fs::read(path).map_err(|error| format!("read {}: {error}", path.display()))?;
    let value = serde_json::from_slice(&bytes)
        .map_err(|error| format!("parse {}: {error}", path.display()))?;
    Ok((bytes, value))
}

fn valid_revision(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn run() -> Result<i32, String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() > 5 {
        return Err(
            "usage: johannesburg-water-event-graph-validator [REGISTER] [GRAPH] [NEGATIVE] [SCHEMA] [RECEIPT]"
                .to_owned(),
        );
    }
    let register_path = PathBuf::from(args.first().map(String::as_str).unwrap_or(
        "docs/research/nrvca/johannesburg-water-service-sources-v0.1.json",
    ));
    let graph_path = PathBuf::from(args.get(1).map(String::as_str).unwrap_or(
        "docs/research/nrvca/johannesburg-water-event-graph-v0.1.json",
    ));
    let negative_path = PathBuf::from(args.get(2).map(String::as_str).unwrap_or(
        "docs/research/nrvca/johannesburg-water-event-graph-negative-controls-v0.1.json",
    ));
    let schema_path = PathBuf::from(args.get(3).map(String::as_str).unwrap_or(
        "docs/research/nrvca/johannesburg-water-event-graph-v0.1.schema.json",
    ));
    let receipt_path = PathBuf::from(args.get(4).map(String::as_str).unwrap_or(
        "artifacts/johannesburg-water-event-graph/receipt.json",
    ));

    let (register_bytes, register) = read_json(&register_path)?;
    let (graph_bytes, graph) = read_json(&graph_path)?;
    let (negative_bytes, negative) = read_json(&negative_path)?;
    let (schema_bytes, schema) = read_json(&schema_path)?;

    let evaluation = validate(
        &register,
        &graph,
        &negative,
        &schema,
        &register_bytes,
    )
    .map_err(|errors| errors.join("\n"))?;

    let revision = env::var("WATER_EVENT_GRAPH_REVISION")
        .or_else(|_| env::var("GITHUB_SHA"))
        .ok()
        .filter(|value| valid_revision(value));
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let lock_path = manifest_dir.join("Cargo.lock");
    let lock_bytes = fs::read(&lock_path)
        .map_err(|error| format!("read {}: {error}", lock_path.display()))?;
    let manifest_path = manifest_dir.join("Cargo.toml");
    let manifest_bytes = fs::read(&manifest_path)
        .map_err(|error| format!("read {}: {error}", manifest_path.display()))?;
    let receipt = json!({
        "schema_version": "johannesburg-water-event-graph-run-v0.1",
        "verification_state": evaluation["verification_state"],
        "qualification_status": "NotRun",
        "implementation_revision": revision,
        "register_sha256": sha256_hex(&register_bytes),
        "graph_sha256": sha256_hex(&graph_bytes),
        "negative_controls_sha256": sha256_hex(&negative_bytes),
        "schema_sha256": sha256_hex(&schema_bytes),
        "cargo_manifest_sha256": sha256_hex(&manifest_bytes),
        "cargo_lock_sha256": sha256_hex(&lock_bytes),
        "evaluation": evaluation
    });

    if let Some(parent) = receipt_path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent)
            .map_err(|error| format!("create {}: {error}", parent.display()))?;
    }
    let serialized = serde_json::to_string_pretty(&receipt)
        .map_err(|error| format!("serialize receipt: {error}"))?
        + "\n";
    fs::write(&receipt_path, serialized.as_bytes())
        .map_err(|error| format!("write {}: {error}", receipt_path.display()))?;
    println!("{serialized}");
    Ok(0)
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(error) => {
            eprintln!("JOHANNESBURG_WATER_EVENT_GRAPH_ERROR: {error}");
            std::process::exit(1);
        }
    }
}
