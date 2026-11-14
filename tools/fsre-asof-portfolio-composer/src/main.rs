use fsre_asof_portfolio_composer::compose;
use std::{env, fs, process};

fn main() {
    let mut args = env::args().skip(1);
    let Some(asof_path) = args.next() else {
        eprintln!("usage: fsre-asof-portfolio-composer <asof-evidence.psv> <portfolio-plan.psv>");
        process::exit(2);
    };
    let Some(portfolio_path) = args.next() else {
        eprintln!("expected asof-evidence.psv and portfolio-plan.psv");
        process::exit(2);
    };
    if args.next().is_some() {
        eprintln!("expected exactly two input paths");
        process::exit(2);
    }
    let asof = match fs::read_to_string(&asof_path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("could not read {asof_path}: {error}");
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
    match compose(&asof, &portfolio) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("as-of portfolio composition failed: {error}");
            process::exit(2);
        }
    }
}
