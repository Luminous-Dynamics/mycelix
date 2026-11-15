use fsre_worldset_stability::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-worldset-stability <input.psv>");
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
            let analysis = analyze(&problem);
            print!("{}", analysis.render());
            if analysis.supported.status == "SEARCH_LIMIT_EXCEEDED"
                || analysis.hypothetical.status == "SEARCH_LIMIT_EXCEEDED"
            {
                ExitCode::from(3)
            } else {
                ExitCode::SUCCESS
            }
        }
        Err(error) => {
            eprintln!("input_error: {error}");
            ExitCode::from(2)
        }
    }
}
