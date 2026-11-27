use battery_passport_projection_audit::audit_projection;
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut registry_path = None;
    let mut registry_schema_path = None;
    let mut projection_schema_path = None;
    let mut projection_path = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--registry" => registry_path = args.next(),
            "--registry-schema" => registry_schema_path = args.next(),
            "--projection-schema" => projection_schema_path = args.next(),
            "--projection" => projection_path = args.next(),
            _ => {
                eprintln!("unknown argument: {arg}");
                return ExitCode::FAILURE;
            }
        }
    }
    let (
        Some(registry_path),
        Some(registry_schema_path),
        Some(projection_schema_path),
        Some(projection_path),
    ) = (
        registry_path,
        registry_schema_path,
        projection_schema_path,
        projection_path,
    )
    else {
        eprintln!(
            "usage: battery-passport-projection-audit --registry PATH --registry-schema PATH --projection-schema PATH --projection PATH"
        );
        return ExitCode::FAILURE;
    };

    let read = |path: &str| match fs::read(path) {
        Ok(bytes) => Ok(bytes),
        Err(error) => Err(format!("cannot read {path}: {error}")),
    };
    let registry = match read(&registry_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let registry_schema = match read(&registry_schema_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let projection_schema = match read(&projection_schema_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let projection = match read(&projection_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::FAILURE;
        }
    };

    match audit_projection(&registry_schema, &registry, &projection_schema, &projection) {
        Ok(report) => match serde_json::to_string_pretty(&report) {
            Ok(output) => {
                println!("{output}");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("cannot serialize audit report: {error}");
                ExitCode::FAILURE
            }
        },
        Err(error) => {
            eprintln!("battery passport projection audit failed: {error}");
            ExitCode::FAILURE
        }
    }
}
