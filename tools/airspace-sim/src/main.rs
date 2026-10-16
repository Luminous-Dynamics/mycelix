use std::{env, fs, process::ExitCode};

use airspace_replay::{replay_set, ScenarioSet};

fn main() -> ExitCode {
    let mut verify = false;
    let mut input_path = None;

    for argument in env::args().skip(1) {
        match argument.as_str() {
            "--verify" => verify = true,
            "--help" | "-h" => {
                println!(
                    "airspace-replay [--verify] [scenario-set.json]\n\
                     Replays deterministic synthetic fixtures. Results are not flight authorization or a safety determination."
                );
                return ExitCode::SUCCESS;
            }
            _ if input_path.is_none() => input_path = Some(argument),
            _ => {
                eprintln!("error: provide at most one scenario-set path");
                return ExitCode::from(2);
            }
        }
    }

    let input = match input_path {
        Some(path) => match fs::read_to_string(&path) {
            Ok(contents) => contents,
            Err(error) => {
                eprintln!("error: unable to read scenario set: {error}");
                return ExitCode::from(2);
            }
        },
        None => include_str!("../fixtures/scenarios.json").to_owned(),
    };

    let set: ScenarioSet = match serde_json::from_str(&input) {
        Ok(value) => value,
        Err(error) => {
            eprintln!("error: scenario set is not valid for this schema: {error}");
            return ExitCode::from(2);
        }
    };

    if set.schema != "airspace-replay-scenarios-v1" {
        eprintln!("error: unsupported scenario schema '{}'", set.schema);
        return ExitCode::from(2);
    }
    if set.status != "synthetic_research_fixture"
        || set.authority_ceiling != "planning_simulation_semantics_only"
    {
        eprintln!("error: scenario set must declare the synthetic research authority ceiling");
        return ExitCode::from(2);
    }

    let output = replay_set(&set);
    match serde_json::to_string_pretty(&output) {
        Ok(rendered) => println!("{rendered}"),
        Err(error) => {
            eprintln!("error: unable to serialize replay output: {error}");
            return ExitCode::from(2);
        }
    }

    if verify && !output.all_expectations_match {
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}
