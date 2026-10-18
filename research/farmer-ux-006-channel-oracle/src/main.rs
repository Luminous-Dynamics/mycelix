use farmer_channel_oracle::{find_repository_root, verify_repository};
use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut root: Option<PathBuf> = None;
    let mut report_path: Option<PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--root" => {
                let Some(value) = args.next() else {
                    eprintln!("--root requires a directory");
                    return ExitCode::from(2);
                };
                root = Some(PathBuf::from(value));
            }
            "--report-json" => {
                let Some(value) = args.next() else {
                    eprintln!("--report-json requires a file path");
                    return ExitCode::from(2);
                };
                report_path = Some(PathBuf::from(value));
            }
            "--help" | "-h" => {
                println!("farmer-channel-oracle [--root DIR] [--report-json FILE]");
                return ExitCode::SUCCESS;
            }
            unknown => {
                eprintln!("unknown argument: {unknown}");
                return ExitCode::from(2);
            }
        }
    }

    let root = match root {
        Some(path) => path,
        None => match find_repository_root() {
            Ok(path) => path,
            Err(error) => {
                eprintln!("{error}");
                return ExitCode::from(2);
            }
        },
    };

    let report = match verify_repository(&root) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("verification could not complete: {error}");
            return ExitCode::from(2);
        }
    };
    let output = match serde_json::to_string_pretty(&report) {
        Ok(output) => output,
        Err(error) => {
            eprintln!("cannot serialize report: {error}");
            return ExitCode::from(2);
        }
    };

    if let Some(path) = report_path {
        if let Err(error) = fs::write(&path, format!("{output}\n")) {
            eprintln!("cannot write {}: {error}", path.display());
            return ExitCode::from(2);
        }
    }
    println!("{output}");
    if report.status == "PASS" {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
