use gcpn_track_a_replay_evaluator::evaluate_bytes;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let path = env::args().nth(1).unwrap_or_else(|| {
        "mycelix-workspace/mycelix-praxis/docs/research/GCPN_TRACK_A_REPLAY_DEMO_RUN_V0_1.json"
            .to_string()
    });
    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_replay_run {path}: {error}");
            return ExitCode::from(2);
        }
    };

    let report = evaluate_bytes(&bytes);
    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot_encode_evaluation_report: {error}");
            return ExitCode::from(2);
        }
    }

    if report.status == "CONFORMS_SYNTHETIC_ONLY" {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}
