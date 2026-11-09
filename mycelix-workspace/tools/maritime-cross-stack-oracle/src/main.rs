//! Offline CLI for the Maritime Commons cross-stack golden-trace oracle.
use maritime_cross_stack_oracle::evaluate_bytes;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 4 {
        eprintln!("usage: maritime-cross-stack-oracle <golden-corpus.json> <oracle-inputs.json> <handoff-template.json> <handoff-verifier-report.json>");
        return ExitCode::from(64);
    }
    let mut data = Vec::with_capacity(4);
    for path in &args {
        match fs::read(path) {
            Ok(bytes) => data.push(bytes),
            Err(error) => { eprintln!("could not read input: {error}"); return ExitCode::from(66); }
        }
    }
    match evaluate_bytes(&data[0], &data[1], &data[2], &data[3]) {
        Ok(report) => {
            if let Err(error) = serde_json::to_writer_pretty(std::io::stdout().lock(), &report) {
                eprintln!("could not write report: {error}");
                return ExitCode::from(74);
            }
            if report.disposition == "pass" { ExitCode::SUCCESS } else { ExitCode::from(1) }
        }
        Err(error) => {
            eprintln!("oracle input rejected: {error:?}");
            ExitCode::from(65)
        }
    }
}
