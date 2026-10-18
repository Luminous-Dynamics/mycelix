use farmer_journal_harness::{default_repository_root, verify_repository};
use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

fn usage() -> &'static str {
    "Usage: farmer-journal-harness [--root REPOSITORY_ROOT] [--report-json OUTPUT_PATH]"
}

fn main() -> ExitCode {
    let mut root = None;
    let mut report_path = None;
    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--root" => root = args.next().map(PathBuf::from),
            "--report-json" => report_path = args.next().map(PathBuf::from),
            "--help" | "-h" => {
                println!("{}", usage());
                return ExitCode::SUCCESS;
            }
            other => {
                eprintln!("unknown argument {other:?}\n{}", usage());
                return ExitCode::from(2);
            }
        }
    }
    let root = match root {
        Some(root) => root,
        None => match default_repository_root() {
            Ok(root) => root,
            Err(error) => { eprintln!("{error}"); return ExitCode::from(2); }
        }
    };
    let report = match verify_repository(&root) {
        Ok(report) => report,
        Err(error) => { eprintln!("farmer journal harness: {error}"); return ExitCode::from(2); }
    };
    let output = match serde_json::to_string_pretty(&report) {
        Ok(value) => format!("{value}\n"),
        Err(error) => { eprintln!("report serialization failed: {error}"); return ExitCode::from(2); }
    };
    if let Some(path) = report_path {
        if let Err(error) = std::fs::write(&path, output.as_bytes()) {
            eprintln!("cannot write {}: {error}", path.display());
            return ExitCode::from(2);
        }
    }
    print!("{output}");
    if report.status == "PASS" { ExitCode::SUCCESS } else { ExitCode::from(1) }
}
