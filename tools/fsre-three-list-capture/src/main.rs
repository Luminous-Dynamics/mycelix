use fsre_three_list_capture::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-three-list-capture <capture-patterns.psv>");
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
            if report.status == "COMPUTED_FINITE_THREE_LIST_SENSITIVITY_INTERVAL"
                || report.status == "COMPUTED_WITHIN_DECLARED_FRAME_ONLY_TARGET_TOTAL_WITHHELD"
            {
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
