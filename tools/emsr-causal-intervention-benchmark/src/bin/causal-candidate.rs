use std::{
    env, fs,
    io::{self, Write},
    process::ExitCode,
};

use emsr_causal_intervention_benchmark::evaluate_public_bytes;

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
    let path = args
        .next()
        .ok_or_else(|| "usage: causal-candidate <public-scenario.json>".to_owned())?;
    if args.next().is_some() {
        return Err("expected exactly one public input path".to_owned());
    }
    let bytes = read_input(&path)?;
    let report = evaluate_public_bytes(&bytes);
    let output = serde_json::to_vec_pretty(&report)
        .map_err(|error| format!("cannot serialize candidate report: {error}"))?;
    io::stdout()
        .write_all(&output)
        .and_then(|_| io::stdout().write_all(b"\n"))
        .map_err(|error| format!("cannot write candidate report: {error}"))?;
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
