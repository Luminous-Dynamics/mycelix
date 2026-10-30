use praxis_nres_capability_loop::{
    evaluate, evaluate_joint_service_forecasts, evaluate_probability_forecasts,
    evaluate_resilience_trajectories, export_evidence_envelope,
};
use serde_json::Value;
use std::process::ExitCode;

const NRES002: &str = include_str!("../../../examples/courses/civic-resilience-policy-benchmark-fixture-001.json");
const NRES005: &str = include_str!("../../../examples/courses/civic-resilience-capability-loop-fixture-001.json");
const NRES007: &str = include_str!("../../../examples/courses/civic-resilience-cascade-recovery-fixture-001.json");
const NRES008: &str = include_str!("../../../examples/courses/civic-resilience-probability-abstention-fixture-001.json");
const NRES009: &str = include_str!("../../../examples/courses/civic-resilience-joint-service-forecast-fixture-001.json");

fn parse_fixture(contents: &str, label: &str) -> Result<Value, ExitCode> {
    serde_json::from_str(contents).map_err(|error| {
        eprintln!("invalid {label} fixture: {error}");
        ExitCode::from(2)
    })
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    if args.iter().any(|arg| arg == "--joint") {
        let nres009 = match parse_fixture(NRES009, "NRES-009") {
            Ok(value) => value,
            Err(code) => return code,
        };
        return emit(evaluate_joint_service_forecasts(&nres009));
    }
    if args.iter().any(|arg| arg == "--probability") {
        let nres008 = match parse_fixture(NRES008, "NRES-008") {
            Ok(value) => value,
            Err(code) => return code,
        };
        return emit(evaluate_probability_forecasts(&nres008));
    }
    if args.iter().any(|arg| arg == "--trajectory") {
        let nres007 = match parse_fixture(NRES007, "NRES-007") {
            Ok(value) => value,
            Err(code) => return code,
        };
        return emit(evaluate_resilience_trajectories(&nres007));
    }
    if args.iter().any(|arg| arg == "--envelope") {
        let nres002 = match parse_fixture(NRES002, "NRES-002") {
            Ok(value) => value,
            Err(code) => return code,
        };
        let nres005 = match parse_fixture(NRES005, "NRES-005") {
            Ok(value) => value,
            Err(code) => return code,
        };
        return emit(export_evidence_envelope(&nres002, &nres005));
    }

    let nres002 = match parse_fixture(NRES002, "NRES-002") {
        Ok(value) => value,
        Err(code) => return code,
    };
    let nres005 = match parse_fixture(NRES005, "NRES-005") {
        Ok(value) => value,
        Err(code) => return code,
    };
    emit(evaluate(&nres002, &nres005))
}

fn emit(result: Result<Value, Vec<String>>) -> ExitCode {
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
