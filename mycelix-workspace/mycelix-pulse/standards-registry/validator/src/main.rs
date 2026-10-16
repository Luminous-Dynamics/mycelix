use mycelix_pulse_standards_registry_check::validate_registry;
use serde_json::Value;
use std::{env, fs, path::PathBuf, process::ExitCode};

fn main() -> ExitCode {
    let mut registry_path = PathBuf::from("standards-registry/registry.v1.json");
    let mut schema_path = PathBuf::from("standards-registry/schema.v1.json");
    let mut json_output = false;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--registry" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --registry requires a path");
                    return ExitCode::from(2);
                };
                registry_path = PathBuf::from(value);
            }
            "--schema" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --schema requires a path");
                    return ExitCode::from(2);
                };
                schema_path = PathBuf::from(value);
            }
            "--json" => json_output = true,
            "--help" | "-h" => {
                println!(
                    "Usage: standards-registry-check [--registry PATH] [--schema PATH] [--json]\n\n                     Default paths are relative to the Pulse workspace root."
                );
                return ExitCode::SUCCESS;
            }
            unknown => {
                eprintln!("error: unknown argument '{unknown}'");
                return ExitCode::from(2);
            }
        }
    }

    let registry = match read_json(&registry_path) {
        Ok(value) => value,
        Err(message) => {
            eprintln!("{message}");
            return ExitCode::from(2);
        }
    };
    let schema = match read_json(&schema_path) {
        Ok(value) => value,
        Err(message) => {
            eprintln!("{message}");
            return ExitCode::from(2);
        }
    };

    let errors = validate_registry(&registry, &schema);
    if json_output {
        let report = serde_json::json!({
            "valid": errors.is_empty(),
            "registry_path": registry_path,
            "schema_path": schema_path,
            "error_count": errors.len(),
            "errors": errors
        });
        println!("{}", serde_json::to_string_pretty(&report).expect("JSON report serializes"));
    } else if errors.is_empty() {
        let entry_count = registry
            .get("entries")
            .and_then(Value::as_array)
            .map_or(0, Vec::len);
        println!(
            "VALID: {entry_count} standards records; unique IDs and relation targets checked; no support state is accepted without its required evidence."
        );
        println!("Note: this is the versioned registry-contract validator, not a generic JSON Schema engine or standards conformance test.");
    } else {
        eprintln!("INVALID: {} registry contract error(s):", errors.len());
        for error in &errors {
            eprintln!("  - {error}");
        }
    }

    if errors.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

fn read_json(path: &PathBuf) -> Result<Value, String> {
    let text = fs::read_to_string(path)
        .map_err(|error| format!("error: cannot read {}: {error}", path.display()))?;
    serde_json::from_str(&text)
        .map_err(|error| format!("error: invalid JSON in {}: {error}", path.display()))
}
