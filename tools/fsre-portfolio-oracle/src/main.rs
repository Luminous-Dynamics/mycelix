use fsre_portfolio_oracle::validate_and_report;
use std::{env, fs, process};

fn main() {
    let mut arguments = env::args().skip(1);
    let Some(path) = arguments.next() else {
        eprintln!("usage: fsre-portfolio-oracle <portfolio-plan.psv>");
        process::exit(2);
    };
    if arguments.next().is_some() {
        eprintln!("expected exactly one portfolio-plan.psv input");
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
            eprintln!("invalid portfolio input: {error}");
            process::exit(2);
        }
    }
}
