use fsre_green_drop_replay::validate_and_report;
use std::env;
use std::fs;

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        eprintln!("usage: fsre-green-drop-replay <public-summary.psv>");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut arguments = env::args().skip(1);
    let input_path = arguments
        .next()
        .ok_or_else(|| "missing public-summary path".to_owned())?;
    if arguments.next().is_some() {
        return Err("expected exactly one public-summary path".to_owned());
    }

    let source = fs::read_to_string(&input_path)
        .map_err(|error| format!("could not read {input_path}: {error}"))?;
    let report = validate_and_report(&source)?;
    print!("{report}");
    Ok(())
}
