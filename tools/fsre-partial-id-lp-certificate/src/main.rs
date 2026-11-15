use fsre_partial_id_lp_certificate::{analyze, parse_input, render_mps};
use std::{env, fs, process::ExitCode};

fn read_input(path: &str) -> Result<String, String> {
    fs::read_to_string(path).map_err(|error| format!("input_error: {error}"))
}

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();

    if args.len() == 3 && args[1] == "--export-mps" {
        let source = match read_input(&args[2]) {
            Ok(source) => source,
            Err(error) => {
                eprintln!("{error}");
                return ExitCode::from(2);
            }
        };
        return match parse_input(&source).and_then(|model| render_mps(&model)) {
            Ok(mps) => {
                print!("{mps}");
                ExitCode::SUCCESS
            }
            Err(error) => {
                eprintln!("mps_export_error: {error}");
                ExitCode::from(4)
            }
        };
    }

    if args.len() != 2 {
        eprintln!(
            "usage: fsre-partial-id-lp-certificate <certificate.psv>\n\
             or: fsre-partial-id-lp-certificate --export-mps <certificate.psv>"
        );
        return ExitCode::from(2);
    }

    let source = match read_input(&args[1]) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("{error}");
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
