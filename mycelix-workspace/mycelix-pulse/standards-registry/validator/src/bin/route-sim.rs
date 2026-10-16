use mycelix_pulse_standards_registry_check::{
    evaluation::evaluate_paired_seeds,
    simulation::simulate_corpus,
};
use serde_json::Value;
use std::{env, fs, path::PathBuf, process::ExitCode};

fn main() -> ExitCode {
    let mut trace_path = PathBuf::from("benchmarks/traces.v1.json");
    let mut messages = None;
    let mut seed = None;
    let mut seed_count = 30_u64;
    let mut candidate = String::from("sequential_failover");
    let mut evaluate = false;
    let mut subject_sha = None::<String>;
    let mut input_manifest_sha256 = None::<String>;
    let mut seed_count_explicit = false;

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
                    Ok(value) if value > 0 => messages = Some(value),
                    _ => {
                        eprintln!("error: --messages must be a positive integer");
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
            "--evaluate" => evaluate = true,
            "--seed-count" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --seed-count requires a number");
                    return ExitCode::from(2);
                };
                match value.parse::<u64>() {
                    Ok(value) => {
                        seed_count = value;
                        seed_count_explicit = true;
                    }
                    Err(_) => {
                        eprintln!("error: --seed-count must be a non-negative integer");
                        return ExitCode::from(2);
                    }
                }
            }
            "--candidate" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --candidate requires a strategy");
                    return ExitCode::from(2);
                };
                candidate = value;
            }
            "--subject-sha" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --subject-sha requires a commit SHA");
                    return ExitCode::from(2);
                };
                subject_sha = Some(value);
            }
            "--input-manifest-sha256" => {
                let Some(value) = args.next() else {
                    eprintln!("error: --input-manifest-sha256 requires a SHA-256 digest");
                    return ExitCode::from(2);
                };
                input_manifest_sha256 = Some(value);
            }
            "--help" | "-h" => {
                println!("Usage: route-sim [--traces PATH] [--messages COUNT] [--seed N]");
                println!("       route-sim --evaluate [--traces PATH] [--messages COUNT]");
                println!(concat!(
                    "                 [--seed FIRST] ",
                    "[--seed-count COUNT] [--candidate STRATEGY]"
                ));
                println!("                 --subject-sha SHA --input-manifest-sha256 SHA256");
                println!();
                println!(concat!(
                    "Strategies: preferred_single_path, ",
                    "sequential_failover, parallel_redundancy"
                ));
                println!(concat!(
                    "Evaluation compares paired synthetic seeds ",
                    "on the declared holdout split."
                ));
                println!(concat!(
                    "Neither mode establishes authenticated delivery ",
                    "or hardware qualification."
                ));
                return ExitCode::SUCCESS;
            }
            unknown => {
                eprintln!("error: unknown argument '{unknown}'");
                return ExitCode::from(2);
            }
        }
    }

    if !evaluate
        && (seed_count_explicit || subject_sha.is_some() || input_manifest_sha256.is_some())
    {
        eprintln!(concat!(
            "error: --seed-count, --subject-sha and ",
            "--input-manifest-sha256 require --evaluate"
        ));
        return ExitCode::from(2);
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

    if evaluate {
        let Some(subject_sha) = subject_sha else {
            eprintln!("error: --evaluate requires --subject-sha");
            return ExitCode::from(2);
        };
        let Some(input_manifest_sha256) = input_manifest_sha256 else {
            eprintln!("error: --evaluate requires --input-manifest-sha256");
            return ExitCode::from(2);
        };
        if !valid_subject_sha(&subject_sha) {
            eprintln!("error: --subject-sha must be a 40- or 64-character hexadecimal commit SHA");
            return ExitCode::from(2);
        }
        if !is_hex_digest(&input_manifest_sha256) {
            eprintln!(concat!(
                "error: --input-manifest-sha256 must be a ",
                "64-character hexadecimal SHA-256 digest"
            ));
            return ExitCode::from(2);
        }

        let first_seed = seed.or_else(|| corpus.get("seed").and_then(Value::as_u64)).unwrap_or(0);
        let mut report = match evaluate_paired_seeds(
            &corpus,
            messages,
            first_seed,
            seed_count,
            &candidate,
        ) {
            Ok(report) => report,
            Err(errors) => {
                eprintln!("INVALID evaluation inputs:");
                for error in errors {
                    eprintln!("  - {error}");
                }
                return ExitCode::FAILURE;
            }
        };
        report["subject_sha"] = Value::String(subject_sha);
        report["input_manifest_sha256"] = Value::String(input_manifest_sha256);
        report["evidence_manifest_bound"] = Value::Bool(true);
        println!("{}", serde_json::to_string_pretty(&report).expect("report serializes"));
        return ExitCode::SUCCESS;
    }

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

fn valid_subject_sha(value: &str) -> bool {
    (value.len() == 40 || value.len() == 64)
        && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

fn is_hex_digest(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}
