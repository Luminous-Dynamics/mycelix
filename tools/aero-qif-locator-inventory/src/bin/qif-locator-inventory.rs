use aero_qif_ingest_qualification::QifDocumentRole;
use aero_qif_locator_inventory::{
    derive_trace_locator_inventory, inventory_required_locators, validate_profile, RawQifDocument,
    TraceLocatorProfileV1,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    env,
    fs,
    path::{Path, PathBuf},
    process::ExitCode,
};

struct Input {
    qpid: String,
    role: QifDocumentRole,
    path: PathBuf,
    bytes: Vec<u8>,
}

fn usage() -> &'static str {
    "Usage:
  qif-locator-inventory --inventory-id ID --profile PROFILE.json
    --results-qpid UUID --results RESULTS.qif
    --plan UUID=PLAN.qif [--plan UUID=PLAN2.qif ...]
    --inventory-out INVENTORY.json --required-locators-out REQUIRED.json
    --report-out REPORT.json

The tool reads staged local files only. QIF source URIs are never dereferenced."
}

fn parse_plan(value: &str) -> Result<(String, PathBuf), String> {
    let (qpid, path) = value.split_once('=')
        .ok_or_else(|| format!("invalid --plan value (expected QPID=PATH): {value}"))?;
    if qpid.trim().is_empty() || path.trim().is_empty() {
        return Err(format!("invalid --plan value (expected QPID=PATH): {value}"));
    }
    Ok((qpid.to_owned(), PathBuf::from(path)))
}

fn read_bounded(path: &Path, maximum: u64) -> Result<Vec<u8>, String> {
    let metadata = fs::metadata(path)
        .map_err(|error| format!("cannot stat {}: {error}", path.display()))?;
    if !metadata.is_file() {
        return Err(format!("input is not a regular file: {}", path.display()));
    }
    if metadata.len() > maximum {
        return Err(format!("input exceeds profile max_source_bytes: {}", path.display()));
    }
    let bytes = fs::read(path)
        .map_err(|error| format!("cannot read {}: {error}", path.display()))?;
    if bytes.len() as u64 > maximum {
        return Err(format!("input grew beyond profile max_source_bytes: {}", path.display()));
    }
    Ok(bytes)
}

fn write_json<T: Serialize>(path: &Path, value: &T) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value)
        .map_err(|error| format!("cannot serialize {}: {error}", path.display()))?;
    fs::write(path, &bytes)
        .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|byte| format!("{byte:02x}")).collect()
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let mut inventory_id = None;
    let mut profile_path = None;
    let mut results_qpid = None;
    let mut results_path = None;
    let mut plans: Vec<(String, PathBuf)> = Vec::new();
    let mut inventory_out = None;
    let mut required_locators_out = None;
    let mut report_out = None;

    while let Some(arg) = args.next() {
        if arg == "--help" || arg == "-h" {
            println!("{}", usage());
            return Ok(());
        }
        let value = args.next().ok_or_else(|| format!("missing value after {arg}"))?;
        match arg.as_str() {
            "--inventory-id" => inventory_id = Some(value),
            "--profile" => profile_path = Some(PathBuf::from(value)),
            "--results-qpid" => results_qpid = Some(value),
            "--results" => results_path = Some(PathBuf::from(value)),
            "--plan" => plans.push(parse_plan(&value)?),
            "--inventory-out" => inventory_out = Some(PathBuf::from(value)),
            "--required-locators-out" => required_locators_out = Some(PathBuf::from(value)),
            "--report-out" => report_out = Some(PathBuf::from(value)),
            "--help" | "-h" => return Err(usage().into()),
            _ => return Err(format!("unknown option {arg}
{}", usage())),
        }
    }

    let inventory_id = inventory_id.ok_or_else(|| usage().to_owned())?;
    let profile_path = profile_path.ok_or_else(|| usage().to_owned())?;
    let results_qpid = results_qpid.ok_or_else(|| usage().to_owned())?;
    let results_path = results_path.ok_or_else(|| usage().to_owned())?;
    let inventory_out = inventory_out.ok_or_else(|| usage().to_owned())?;
    let required_locators_out = required_locators_out.ok_or_else(|| usage().to_owned())?;
    let report_out = report_out.ok_or_else(|| usage().to_owned())?;
    if plans.is_empty() {
        return Err("--plan must be supplied at least once".into());
    }

    let profile_metadata = fs::metadata(&profile_path)
        .map_err(|error| format!("cannot stat profile {}: {error}", profile_path.display()))?;
    if !profile_metadata.is_file() || profile_metadata.len() > 1024 * 1024 {
        return Err("profile must be a regular file no larger than 1 MiB".into());
    }
    let profile_bytes = fs::read(&profile_path)
        .map_err(|error| format!("cannot read profile {}: {error}", profile_path.display()))?;
    if profile_bytes.len() > 1024 * 1024 {
        return Err("profile grew beyond 1 MiB while being read".into());
    }
    let profile: TraceLocatorProfileV1 = serde_json::from_slice(&profile_bytes)
        .map_err(|error| format!("invalid profile {}: {error}", profile_path.display()))?;
    validate_profile(&profile).map_err(|error| format!("invalid profile: {error}"))?;

    let input_paths = std::iter::once(&results_path)
        .chain(plans.iter().map(|(_, path)| path))
        .collect::<Vec<_>>();
    if input_paths.iter().any(|path| **path == profile_path)
        || input_paths.iter().any(|path| **path == inventory_out)
        || input_paths.iter().any(|path| **path == required_locators_out)
        || input_paths.iter().any(|path| **path == report_out)
        || inventory_out == required_locators_out
        || inventory_out == report_out
        || required_locators_out == report_out
    {
        return Err("profile, source inputs, inventory output and report output must use distinct paths".into());
    }

    let mut inputs = Vec::with_capacity(plans.len() + 1);
    inputs.push(Input {
        qpid: results_qpid,
        role: QifDocumentRole::Results,
        bytes: read_bounded(&results_path, profile.limits.max_source_bytes)?,
        path: results_path,
    });
    for (qpid, path) in plans {
        inputs.push(Input {
            qpid,
            role: QifDocumentRole::Plan,
            bytes: read_bounded(&path, profile.limits.max_source_bytes)?,
            path,
        });
    }

    let documents = inputs.iter().map(|input| RawQifDocument {
        qpid: &input.qpid,
        role: input.role,
        bytes: input.bytes.as_slice(),
    }).collect::<Vec<_>>();

    let (inventory, report) = derive_trace_locator_inventory(
        &inventory_id,
        &profile,
        &documents,
    ).map_err(|error| format!("inventory derivation failed: {error}"))?;

    let required_locators = inventory_required_locators(&inventory)
        .map_err(|error| format!("cannot derive required locator set: {error}"))?;
    write_json(&inventory_out, &inventory)?;
    write_json(&required_locators_out, &required_locators)?;
    write_json(&report_out, &report)?;

    let inventory_bytes = serde_json::to_vec_pretty(&inventory)
        .map_err(|error| format!("cannot serialize inventory digest: {error}"))?;
    let report_bytes = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize report digest: {error}"))?;
    eprintln!("inventory_sha256={}", inventory.inventory_sha256);
    eprintln!("inventory_file_sha256={}", sha256_hex(&inventory_bytes));
    eprintln!("report_file_sha256={}", sha256_hex(&report_bytes));
    eprintln!("source_documents={}", inventory.body.documents.len());
    eprintln!("source_files={}", inputs.iter().map(|input| input.path.display().to_string()).collect::<Vec<_>>().join(","));
    eprintln!("locators={}", inventory.body.locators.len());
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
