use cross_domain_disruption_v1_validator::validate_fixture_dir;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = std::env::args_os().skip(1);
    let directory = match (args.next(), args.next()) {
        (None, None) => PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../mycelix-workspace/docs/ops-intel/fixtures"),
        (Some(path), None) => PathBuf::from(path),
        _ => {
            eprintln!("usage: cross-domain-disruption-v1-validator [FIXTURE_DIR]");
            return ExitCode::from(2);
        }
    };

    let errors = validate_fixture_dir(&directory);
    if errors.is_empty() {
        println!("CrossDomainDisruptionV1 Rust fixture preflight: PASS");
        println!("Validated six JSON fixtures, 20 structural predicates, and 23 mutation descriptors.");
        println!("Claim ceiling: structural fixture consistency only; no canonical encoding, reasoning, truth, or authority qualification.");
        ExitCode::SUCCESS
    } else {
        eprintln!("CrossDomainDisruptionV1 Rust fixture preflight: FAIL ({} findings)", errors.len());
        for error in errors {
            eprintln!("- {error}");
        }
        ExitCode::FAILURE
    }
}
