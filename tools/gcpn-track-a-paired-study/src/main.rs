use gcpn_track_a_paired_study::{evaluate_frozen_fixture, evaluate_with_expected_sha256};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args = env::args().skip(1).collect::<Vec<_>>();
    if args.len() > 2 {
        eprintln!("usage: gcpn-track-a-paired-study [study.json [expected_sha256]]");
        return ExitCode::from(2);
    }

    let report = if args.is_empty() {
        let fixture = include_bytes!("../fixtures/paired_study_synthetic.json");
        evaluate_frozen_fixture(fixture)
    } else {
        let path = &args[0];
        let bytes = match fs::read(path) {
            Ok(bytes) => bytes,
            Err(error) => {
                eprintln!("cannot_read_study {path}: {error}");
                return ExitCode::from(2);
            }
        };
        evaluate_with_expected_sha256(&bytes, args.get(1).map(String::as_str))
    };

    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot_encode_study_report: {error}");
            return ExitCode::from(2);
        }
    }

    match report.status {
        "SYNTHETIC_PREVIEW" | "INSUFFICIENT_INDEPENDENT_UNITS" | "CONTROLLED_PAIRED_RESULT" => {
            ExitCode::SUCCESS
        }
        "BLOCKED" => ExitCode::from(3),
        "INCOMPARABLE" => ExitCode::from(4),
        _ => ExitCode::from(1),
    }
}
