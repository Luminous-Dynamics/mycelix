use fsre_decision_episode::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-decision-episode <input.psv>");
        return ExitCode::from(2);
    }
    let source = match fs::read_to_string(&args[1]) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("input_error: {error}");
            return ExitCode::from(2);
        }
    };
    match parse_input(&source) {
        Ok(problem) => {
            let report = analyze(&problem);
            print!("{}", report.render());
            if report.terminal_status == "DECISION_ASSURANCE_COMPLETE_WITHIN_SCOPE" {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(3)
            }
        }
        Err(error) => {
            eprintln!("input_error: {error}");
            ExitCode::from(2)
        }
    }
}
