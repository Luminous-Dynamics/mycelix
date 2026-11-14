use fsre_decision_time_oracle::validate_and_report;
use std::{env, fs, process};

fn main() {
    let mut args = env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: fsre-decision-time-oracle <decision-evidence.psv>");
        process::exit(2);
    };
    if args.next().is_some() {
        eprintln!("expected exactly one decision-evidence.psv input");
        process::exit(2);
    }
    let source = match fs::read_to_string(&path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("could not read {path}: {error}");
            process::exit(2);
        }
    };
    match validate_and_report(&source) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("invalid decision-time evidence input: {error}");
            process::exit(2);
        }
    }
}
