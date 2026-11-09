//! Minimal offline CLI for the Maritime Commons handoff manifest verifier.

use maritime_handoff_verifier::{Disposition, validate_manifest_bytes};
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = env::args_os();
    let program = args.next().unwrap_or_default();

    let Some(path) = args.next() else {
        eprintln!("usage: {} <manifest.json>", program.to_string_lossy());
        return ExitCode::from(64);
    };

    if args.next().is_some() {
        eprintln!("expected exactly one manifest path");
        return ExitCode::from(64);
    }

    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("could not read manifest: {error}");
            return ExitCode::from(66);
        }
    };

    let report = match validate_manifest_bytes(&bytes) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("{error}");
            return ExitCode::from(65);
        }
    };

    if let Err(error) = serde_json::to_writer_pretty(std::io::stdout().lock(), &report) {
        eprintln!("could not write validation report: {error}");
        return ExitCode::from(74);
    }

    match report.disposition {
        Disposition::Pass => ExitCode::SUCCESS,
        Disposition::NotEvaluable | Disposition::Unknown => ExitCode::from(2),
        Disposition::Reject | Disposition::Conflict => ExitCode::from(1),
    }
}
