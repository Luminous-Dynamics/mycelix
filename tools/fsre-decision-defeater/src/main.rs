use fsre_decision_defeater::analyze;
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-decision-defeater <decision-episode.psv>");
        return ExitCode::from(2);
    }

    let source = match fs::read_to_string(&args[1]) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("cannot read {}: {error}", args[1]);
            return ExitCode::from(2);
        }
    };

    match analyze(&source) {
        Ok(Ok(report)) => {
            print!("{}", report.render());
            if report.status == "SEARCH_LIMIT_EXCEEDED" {
                ExitCode::from(4)
            } else {
                ExitCode::SUCCESS
            }
        }
        Ok(Err(qualification)) => {
            print!("{}", qualification.render());
            ExitCode::from(3)
        }
        Err(error) => {
            eprintln!("decision-defeater analysis failed: {error}");
            ExitCode::from(2)
        }
    }
}
