use fsre_recovery_qualification::{
    evaluate, parse_debt_items, parse_predicate_evidence, parse_scenario_and_options,
};
use std::env;
use std::fs;

fn main() {
    if let Err(error) = run() {
        eprintln!("error: {error}");
        eprintln!(
            "usage: fsre-recovery-qualification <scenario-options.psv> <debt-items.psv> <predicate-evidence.psv>"
        );
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let arguments: Vec<String> = env::args().skip(1).collect();
    if arguments.len() != 3 {
        return Err("expected exactly three input files".to_owned());
    }

    let scenario_source = fs::read_to_string(&arguments[0])
        .map_err(|error| format!("could not read {}: {error}", arguments[0]))?;
    let debt_source = fs::read_to_string(&arguments[1])
        .map_err(|error| format!("could not read {}: {error}", arguments[1]))?;
    let evidence_source = fs::read_to_string(&arguments[2])
        .map_err(|error| format!("could not read {}: {error}", arguments[2]))?;

    let (scenario, options) =
        parse_scenario_and_options(&scenario_source).map_err(|error| error.to_string())?;
    let debt_items = parse_debt_items(&debt_source).map_err(|error| error.to_string())?;
    let evidence =
        parse_predicate_evidence(&evidence_source).map_err(|error| error.to_string())?;
    let report =
        evaluate(&scenario, &options, &debt_items, &evidence).map_err(|error| error.to_string())?;
    print!("{}", report.render());
    Ok(())
}
