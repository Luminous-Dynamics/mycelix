use std::{env, fs, io::{self, Write}, process::ExitCode};

use emsr_causal_intervention_benchmark::score_candidate;

const MAX_INPUT_BYTES: u64 = 64 * 1024 * 1024;

fn read_input(path: &str) -> Result<Vec<u8>, String> {
    let metadata = fs::metadata(path).map_err(|error| format!("cannot stat {path}: {error}"))?;
    if metadata.len() > MAX_INPUT_BYTES {
        return Err(format!("input {path} exceeds {MAX_INPUT_BYTES} bytes"));
    }
    fs::read(path).map_err(|error| format!("cannot read {path}: {error}"))
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let public_path = args.next().ok_or_else(|| {
        "usage: causal-evaluator <public-scenario.json> <hidden-truth.json> <candidate-report.json>".to_owned()
    })?;
    let truth_path = args.next().ok_or_else(|| "hidden truth path required".to_owned())?;
    let report_path = args.next().ok_or_else(|| "candidate report path required".to_owned())?;
    if args.next().is_some() {
        return Err("expected exactly three input paths".to_owned());
    }

    // This binary belongs in the evaluator-controlled environment only.
    let public_bytes = read_input(&public_path)?;
    let truth_bytes = read_input(&truth_path)?;
    let report_bytes = read_input(&report_path)?;
    let score = score_candidate(&public_bytes, &truth_bytes, &report_bytes);
    let output = serde_json::to_vec_pretty(&score)
        .map_err(|error| format!("cannot serialize evaluation score: {error}"))?;
    io::stdout()
        .write_all(&output)
        .and_then(|_| io::stdout().write_all(b"\n"))
        .map_err(|error| format!("cannot write evaluation score: {error}"))?;
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
