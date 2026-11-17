use fsre_capture_recapture::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-capture-recapture <input.psv>");
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
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("input_error: {error}");
            ExitCode::from(2)
        }
    }
}
