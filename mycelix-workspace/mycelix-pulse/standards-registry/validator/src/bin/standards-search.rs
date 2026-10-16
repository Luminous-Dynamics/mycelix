use mycelix_pulse_standards_registry_check::{query_registry, validate_registry, RegistryQuery};
use serde_json::Value;
use std::{env, fs, path::PathBuf, process::ExitCode};

fn main() -> ExitCode {
    let mut registry_path = PathBuf::from("standards-registry/registry.v1.json");
    let mut schema_path = PathBuf::from("standards-registry/schema.v1.json");
    let mut query = RegistryQuery::default();

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
            "--query" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --query requires text");
                    return ExitCode::from(2);
                };
                query.text = Some(value);
            }
            "--layer" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --layer requires a layer ID");
                    return ExitCode::from(2);
                };
                query.layer = Some(value);
            }
            "--source-status" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --source-status requires a status");
                    return ExitCode::from(2);
                };
                query.source_status = Some(value);
            }
            "--support-status" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --support-status requires a status");
                    return ExitCode::from(2);
                };
                query.support_status = Some(value);
            }
            "--errata-status" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --errata-status requires a disposition");
                    return ExitCode::from(2);
                };
                query.errata_status = Some(value);
            }
            "--limit" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --limit requires an integer");
                    return ExitCode::from(2);
                };
                match value.parse::<usize>() {
                    Ok(value) => query.limit = value,
                    Err(_) => {
                        eprintln!("error: --limit must be an integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--help" | "-h" => {
                println!("Usage: standards-search [OPTIONS]");
                println!();
                println!("Options:");
                println!("  --query TEXT              Match ID, title, authority, URL, notes, or evidence text");
                println!("  --layer LAYER             Filter by physical/link/network/transport/routing/etc.");
                println!("  --source-status STATUS    Filter source publication status");
                println!("  --support-status STATUS   Filter Pulse implementation evidence state");
                println!("  --errata-status STATUS    Filter reported/verified/held/rejected errata");
                println!("  --limit N                 Bound results to 1..=100 (default: 20)");
                println!("  --registry PATH           Registry JSON path");
                println!("  --schema PATH             JSON Schema path");
                println!();
                println!("Output is source-linked registry metadata, not proof of support or conformance.");
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

    let validation_errors = validate_registry(&registry, &schema);
    if !validation_errors.is_empty() {
        eprintln!("INVALID registry; refusing to return query results:");
        for error in validation_errors {
            eprintln!("  - {error}");
        }
        return ExitCode::FAILURE;
    }

    match query_registry(&registry, &query) {
        Ok(report) => {
            println!(
                "{}",
                serde_json::to_string_pretty(&report).expect("query report serializes")
            );
            ExitCode::SUCCESS
        }
        Err(errors) => {
            eprintln!("INVALID query:");
            for error in errors {
                eprintln!("  - {error}");
            }
            ExitCode::FAILURE
        }
    }
}

fn read_json(path: &PathBuf) -> Result<Value, String> {
    let text = fs::read_to_string(path)
        .map_err(|error| format!("error: cannot read {}: {error}", path.display()))?;
    serde_json::from_str(&text)
        .map_err(|error| format!("error: invalid JSON in {}: {error}", path.display()))
}
