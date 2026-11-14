use std::{env, fs, process::ExitCode};
use fsre_frame_dependency_audit::audit;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: fsre-frame-dependency-audit <source-frames.psv>");
        return ExitCode::from(2);
    }
    let source = match fs::read_to_string(&args[1]) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("cannot read {}: {error}", args[1]);
            return ExitCode::from(2);
        }
    };
    match audit(&source) {
        Ok(Ok(report)) => {
            print!("{}", report.render());
            ExitCode::SUCCESS
        }
        Ok(Err(qualification)) => {
            print!("{}", qualification.render());
            ExitCode::from(3)
        }
        Err(error) => {
            eprintln!("frame dependency audit failed: {error}");
            ExitCode::from(2)
        }
    }
}
