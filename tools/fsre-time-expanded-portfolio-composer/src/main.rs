use fsre_time_expanded_portfolio_composer::validate_and_report;
use std::{env, fs, process};

fn main() {
    let mut args = env::args().skip(1);
    let Some(plan_path) = args.next() else {
        eprintln!("usage: fsre-time-expanded-portfolio-composer <plan.psv> <resource-stock-flow.psv> [...]");
        process::exit(2);
    };
    let stock_paths: Vec<String> = args.collect();
    if stock_paths.is_empty() {
        eprintln!("at least one resource stock-flow file is required");
        process::exit(2);
    }
    let plan = match fs::read_to_string(&plan_path) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("could not read {plan_path}: {error}");
            process::exit(2);
        }
    };
    let mut stocks = Vec::new();
    for path in &stock_paths {
        match fs::read_to_string(path) {
            Ok(value) => stocks.push(value),
            Err(error) => {
                eprintln!("could not read {path}: {error}");
                process::exit(2);
            }
        }
    }
    match validate_and_report(&plan, &stocks) {
        Ok(report) => print!("{report}"),
        Err(error) => {
            eprintln!("invalid time-expanded portfolio input: {error}");
            process::exit(2);
        }
    }
}
