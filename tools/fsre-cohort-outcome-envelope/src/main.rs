use fsre_cohort_outcome_envelope::validate_and_report;
use std::{env, fs, process};

fn main() {
    let paths: Vec<String> = env::args().skip(1).collect();
    if paths.is_empty() {
        eprintln!("usage: fsre-cohort-outcome-envelope <cohort-lane.csv> <cohort-lane.csv> [...]");
        process::exit(2);
    }

    let mut sources = Vec::with_capacity(paths.len());
    for path in &paths {
        match fs::read_to_string(path) {
            Ok(source) => sources.push(source),
            Err(error) => {
                eprintln!("could not read {path}: {error}");
                process::exit(2);
            }
        }
    }

    match validate_and_report(&sources) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("invalid cohort outcome envelope: {error}");
            process::exit(2);
        }
    }
}
