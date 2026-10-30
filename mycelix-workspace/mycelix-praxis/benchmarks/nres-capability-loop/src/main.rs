use praxis_nres_capability_loop::{evaluate, evaluate_resilience_trajectories, export_evidence_envelope};
use serde_json::Value;
use std::process::ExitCode;

const NRES002: &str = include_str!("../../../examples/courses/civic-resilience-policy-benchmark-fixture-001.json");
const NRES005: &str = include_str!("../../../examples/courses/civic-resilience-capability-loop-fixture-001.json");
const NRES007: &str = include_str!("../../../examples/courses/civic-resilience-cascade-recovery-fixture-001.json");

fn main() -> ExitCode {
    let nres002: Value = match serde_json::from_str(NRES002) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid NRES-002 fixture: {error}");
            return ExitCode::from(2);
        }
    };
    let nres005: Value = match serde_json::from_str(NRES005) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid NRES-005 fixture: {error}");
            return ExitCode::from(2);
        }
    };
    let nres007: Value = match serde_json::from_str(NRES007) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("invalid NRES-007 fixture: {error}");
            return ExitCode::from(2);
        }
    };

    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = if args.iter().any(|arg| arg == "--trajectory") {
        evaluate_resilience_trajectories(&nres007)
    } else if args.iter().any(|arg| arg == "--envelope") {
        export_evidence_envelope(&nres002, &nres005)
    } else {
        evaluate(&nres002, &nres005)
    };

    match result {
        Ok(output) => match serde_json::to_string_pretty(&output) {
            Ok(encoded) => {
                println!("{encoded}");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("could not encode benchmark output: {error}");
                ExitCode::from(2)
            }
        },
        Err(errors) => {
            for error in errors {
                eprintln!("FAIL: {error}");
            }
            ExitCode::from(1)
        }
    }
}
