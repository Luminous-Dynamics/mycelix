use fsre_three_list_stratified::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-three-list-stratified <stratified-capture-patterns.psv>");
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
            if report.aggregate_status == "COMPUTED_STRATIFIED_THREE_LIST_INTERVAL"
                || report.aggregate_status
                    == "COMPUTED_HIDDEN_BOUNDS_POPULATION_TOTAL_WITHHELD_FRAME_INCOMPLETE"
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
