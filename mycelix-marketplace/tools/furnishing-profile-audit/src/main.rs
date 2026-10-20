use furnishing_profile_audit::validate_fixture;
use serde_json::Value;
use std::{env, fs, path::PathBuf, process::ExitCode};

fn main() -> ExitCode {
    let default_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../contracts/fixtures/furnishing-catalog-profile-v0.1.json");
    let path = env::args_os().nth(1).map(PathBuf::from).unwrap_or(default_path);

    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("ERROR: cannot read {}: {error}", path.display());
            return ExitCode::FAILURE;
        }
    };
    let document: Value = match serde_json::from_slice(&bytes) {
        Ok(document) => document,
        Err(error) => {
            eprintln!("ERROR: invalid JSON in {}: {error}", path.display());
            return ExitCode::FAILURE;
        }
    };

    let errors = validate_fixture(&document);
    if errors.is_empty() {
        println!("PASS: synthetic furnishing profile semantic audit");
        println!("Input: {}", path.display());
        println!("Scope: contract invariants only; not product verification or JSON Schema validation");
        ExitCode::SUCCESS
    } else {
        eprintln!("FAIL: {} furnishing profile audit finding(s)", errors.len());
        for error in errors {
            eprintln!(" - {error}");
        }
        ExitCode::FAILURE
    }
}
