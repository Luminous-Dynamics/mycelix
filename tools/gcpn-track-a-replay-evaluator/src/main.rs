use gcpn_track_a_replay_evaluator::evaluate;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let run_path = args.next().unwrap_or_else(|| {
        "tools/gcpn-track-a-replay-evaluator/fixtures/track_a_smoke_run_v0_1.json".to_string()
    });
    let awareness_path = args.next().unwrap_or_else(|| {
        "mycelix-workspace/mycelix-praxis/docs/research/GCPN_CIV_AUTONOMOUS_AWARENESS_CASES_V0_1.json".to_string()
    });
    let comparison_path = args.next().unwrap_or_else(|| {
        "mycelix-workspace/mycelix-praxis/docs/research/GCPN_AWARENESS_INDUSTRY_COMPARISON_CASES_V0_1.json".to_string()
    });

    let run = match fs::read(&run_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_run {run_path}: {error}");
            return ExitCode::from(2);
        }
    };
    let awareness = match fs::read(&awareness_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_awareness_manifest {awareness_path}: {error}");
            return ExitCode::from(2);
        }
    };
    let comparison = match fs::read(&comparison_path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_comparison_manifest {comparison_path}: {error}");
            return ExitCode::from(2);
        }
    };

    let report = evaluate(&awareness, &comparison, &run);
    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot_encode_report: {error}");
            return ExitCode::from(2);
        }
    }
    if report.evaluation_status == "SCORED_SYNTHETIC_SMOKE_ONLY"
        || report.evaluation_status == "SCORED_TRACK_A_REPLAY_ONLY"
    {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}
