use fsre_partial_id_lp_certificate::{analyze, parse_input};
use std::{env, fs, process::ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-partial-id-lp-certificate <lp-certificate.psv>");
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
        Ok(model) => match analyze(&model) {
            Ok(report) => {
                print!("{}", report.render());
                if report.optimality_status == "OPTIMALITY_PROVEN_BY_EXACT_PRIMAL_DUAL_EQUALITY" {
                    ExitCode::SUCCESS
                } else {
                    ExitCode::from(3)
                }
            }
            Err(error) => {
                eprintln!("verification_error: {error}");
                ExitCode::from(4)
            }
        },
        Err(error) => {
            eprintln!("input_error: {error}");
            ExitCode::from(2)
        }
    }
}
