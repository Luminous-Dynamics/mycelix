use fsre_assurance_composer::compose;
use std::{env, fs, process};

fn main() {
    let mut args = env::args().skip(1);
    let Some(decision_path) = args.next() else {
        eprintln!("usage: fsre-assurance-composer <decision-evidence.psv> <portfolio-plan.psv>");
        process::exit(2);
    };
    let Some(portfolio_path) = args.next() else {
        eprintln!("expected decision-evidence.psv and portfolio-plan.psv");
        process::exit(2);
    };
    if args.next().is_some() {
        eprintln!("expected exactly two input paths");
        process::exit(2);
    }
    let decision = match fs::read_to_string(&decision_path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("could not read {decision_path}: {error}");
            process::exit(2);
        }
    };
    let portfolio = match fs::read_to_string(&portfolio_path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("could not read {portfolio_path}: {error}");
            process::exit(2);
        }
    };
    match compose(&decision, &portfolio) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("assurance composition failed: {error}");
            process::exit(2);
        }
    }
}
