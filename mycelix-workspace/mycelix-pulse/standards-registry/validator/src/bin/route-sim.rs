use mycelix_pulse_standards_registry_check::simulation::simulate_corpus;
use serde_json::Value;
use std::{env, fs, path::PathBuf, process::ExitCode};

fn main() -> ExitCode {
    let mut trace_path = PathBuf::from("benchmarks/traces.v1.json");
    let mut messages = None;
    let mut seed = None;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--traces" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --traces requires a path");
                    return ExitCode::from(2);
                };
                trace_path = PathBuf::from(value);
            }
            "--messages" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --messages requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) => messages = Some(value),
                    Err(_) => {
                        eprintln!("error: --messages must be a non-negative integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--seed" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --seed requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) => seed = Some(value),
                    Err(_) => {
                        eprintln!("error: --seed must be a non-negative integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--help" | "-h" => {
                println!("Usage: route-sim [--traces PATH] [--messages COUNT] [--seed N]");
                println!();
                println!("Runs deterministic synthetic path comparisons.");
                println!("Output never establishes real delivery or hardware qualification.");
                return ExitCode::SUCCESS;
            }
            unknown => {
                eprintln!("error: unknown argument '{unknown}'");
                return ExitCode::from(2);
            }
        }
    }

    let text = match fs::read_to_string(&trace_path) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("error: cannot read {}: {error}", trace_path.display());
            return ExitCode::from(2);
        }
    };
    let corpus: Value = match serde_json::from_str(&text) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("error: invalid trace JSON in {}: {error}", trace_path.display());
            return ExitCode::from(2);
        }
    };

    let report = match simulate_corpus(&corpus, messages, seed) {
        Ok(report) => report,
        Err(errors) => {
            eprintln!("INVALID trace corpus:");
            for error in errors {
                eprintln!("  - {error}");
            }
            return ExitCode::FAILURE;
        }
    };
    println!("{}", serde_json::to_string_pretty(&report).expect("report serializes"));
    ExitCode::SUCCESS
}
