use ipb_finality_mutation_lab::{generate_cases, MutationReport};
use std::{env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidInput, message.into()))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 4 {
        return Err(invalid(
            "usage: ipb-finality-mutation-lab <candidate-input.json> <oracle.json> <report.json>",
        ));
    }
    let cases = generate_cases();
    let report = MutationReport::from_cases(&cases);
    fs::write(&args[1], MutationReport::input_manifest_json(&cases))?;
    fs::write(&args[2], MutationReport::oracle_json(&cases))?;
    fs::write(&args[3], report.report_json())?;
    if report.survivor_count > 0 || report.false_permit_count > 0 {
        return Err(invalid(format!(
            "reference mutation failures: {} survivors, {} false permits",
            report.survivor_count, report.false_permit_count
        )));
    }
    eprintln!(
        "LISE mutation lab: {} cases ({} controls, {} mutants), {} killed, {} survivors",
        cases.len(), report.control_count, report.mutant_count,
        report.killed_count, report.survivor_count
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("ipb-finality-mutation-lab: {error}");
        std::process::exit(2);
    }
}
