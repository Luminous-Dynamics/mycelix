use gcpn_uas_offline_dataset_verifier::verify_bytes;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let path = env::args().nth(1).unwrap_or_else(|| {
        "mycelix-workspace/mycelix-praxis/docs/research/GCPN_UAS_OFFLINE_DATASET_REGISTER_V0_1.json"
            .to_string()
    });
    let bytes = match fs::read(&path) {
        Ok(bytes) => bytes,
        Err(error) => {
            eprintln!("cannot_read_dataset_register {path}: {error}");
            return ExitCode::from(2);
        }
    };

    let report = verify_bytes(&bytes);
    match serde_json::to_string_pretty(&report) {
        Ok(output) => println!("{output}"),
        Err(error) => {
            eprintln!("cannot_encode_dataset_register_report: {error}");
            return ExitCode::from(2);
        }
    }
    if report.status == "CONFORMS_CANDIDATES_NOT_ADMITTED" {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}
