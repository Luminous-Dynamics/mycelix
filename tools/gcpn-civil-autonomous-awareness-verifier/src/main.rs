use gcpn_civil_autonomous_awareness_verifier::verify;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let path = env::args().nth(1).unwrap_or_else(|| {
        "mycelix-workspace/mycelix-praxis/docs/research/GCPN_CIV_AUTONOMOUS_AWARENESS_CASES_V0_1.json".to_string()
    });
    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_fixture {path}: {error}");
            return ExitCode::from(2);
        }
    };

    let report = verify(&bytes);
    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot_encode_report: {error}");
            return ExitCode::from(2);
        }
    }
    if report.status == "CONFORMS" {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}
