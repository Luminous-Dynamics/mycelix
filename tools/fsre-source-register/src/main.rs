use fsre_source_register::validate_and_report;
use std::env;
use std::fs;

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        eprintln!("usage: fsre-source-register <source-register.psv>");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut arguments = env::args().skip(1);
    let input_path = arguments
        .next()
        .ok_or_else(|| "missing source-register path".to_owned())?;
    if arguments.next().is_some() {
        return Err("expected exactly one source-register path".to_owned());
    }
    let source = fs::read_to_string(&input_path)
        .map_err(|error| format!("could not read {input_path}: {error}"))?;
    let report = validate_and_report(&source).map_err(|error| error.to_string())?;
    print!("{}", report.render());
    Ok(())
}
