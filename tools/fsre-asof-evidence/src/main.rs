use fsre_asof_evidence::validate_and_report;
use std::{env, fs, process};

fn main() {
    let mut args = env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: fsre-asof-evidence <evidence-snapshot.psv>");
        process::exit(2);
    };
    if args.next().is_some() {
        eprintln!("expected exactly one evidence-snapshot.psv input");
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
            eprintln!("invalid as-of evidence input: {error}");
            process::exit(2);
        }
    }
}
