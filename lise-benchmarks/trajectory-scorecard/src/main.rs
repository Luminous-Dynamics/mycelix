use lise_trajectory_scorecard::evaluate;
use serde_json::Value;
use std::{env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidInput, message.into()))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 5 {
        return Err(invalid(
            "usage: lise-trajectory-scorecard <challenge.json> <oracle.json> <submissions.json> <report.json>",
        ));
    }
    let challenge: Value = serde_json::from_slice(&fs::read(&args[1])?)?;
    let oracle: Value = serde_json::from_slice(&fs::read(&args[2])?)?;
    let submissions: Value = serde_json::from_slice(&fs::read(&args[3])?)?;
    let report = evaluate(&challenge, &oracle, &submissions);
    fs::write(&args[4], serde_json::to_vec_pretty(&report)?)?;
    eprintln!(
        "LISE trajectory scorecard: candidates={}, episodes={}, dispatch_eligible=false",
        report["candidate_count"], report["challenge_episode_count"]
    );
    if report["status"] == "INVALID_INPUT" {
        return Err(invalid("trajectory benchmark inputs invalid; see report"));
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("lise-trajectory-scorecard: {error}");
        std::process::exit(2);
    }
}
