use fsre_cohort_outcome_envelope::validate_and_report_with_manifest;
use std::{env, fs, process};

fn main() {
    let mut paths = env::args().skip(1);
    let Some(manifest_path) = paths.next() else {
        eprintln!("usage: fsre-cohort-outcome-envelope <cohort-manifest.psv> <cohort-lane.csv> <cohort-lane.csv> [...]");
        process::exit(2);
    };
    let lane_paths: Vec<String> = paths.collect();
    if lane_paths.is_empty() {
        eprintln!("at least one cohort lane is required");
        process::exit(2);
    }

    let manifest = match fs::read_to_string(&manifest_path) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("could not read {manifest_path}: {error}");
            process::exit(2);
        }
    };
    let mut sources = Vec::with_capacity(lane_paths.len());
    for path in &lane_paths {
        match fs::read_to_string(path) {
            Ok(source) => sources.push(source),
            Err(error) => {
                eprintln!("could not read {path}: {error}");
                process::exit(2);
            }
        }
    }

    match validate_and_report_with_manifest(&manifest, &sources) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("invalid cohort outcome envelope: {error}");
            process::exit(2);
        }
    }
}
