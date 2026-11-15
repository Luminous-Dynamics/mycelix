use fsre_assurance_acquisition::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-assurance-acquisition <input.psv>");
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
            match report.result.status.as_str() {
                "EXACT_COMPLETE_COVER_FOUND" | "NO_ACQUISITIONS_REQUIRED" => ExitCode::SUCCESS,
                "SEARCH_LIMIT_EXCEEDED_NO_OPTIMALITY_CLAIM" => ExitCode::from(4),
                _ => ExitCode::from(3),
            }
        }
        Err(error) => {
            eprintln!("input_error: {error}");
            ExitCode::from(2)
        }
    }
}
