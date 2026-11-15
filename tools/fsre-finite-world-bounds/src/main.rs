use std::{env, fs, process::ExitCode};
use fsre_finite_world_bounds::analyze;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-finite-world-bounds <admissible-worlds.psv>");
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
            ExitCode::SUCCESS
        }
        Ok(Err(qualification)) => {
            print!("{}", qualification.render());
            ExitCode::from(3)
        }
        Err(error) => {
            eprintln!("finite-world analysis failed: {error}");
            ExitCode::from(2)
        }
    }
}
