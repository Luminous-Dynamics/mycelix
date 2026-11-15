use lise_devsecops_agent_reference::evaluate_bundle;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidInput, message.into()))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 4 {
        return Err(invalid(
            "usage: devsecops-agent-reference <cases.json> <oracle.json> <report.json>",
        ));
    }
    let cases_bytes = fs::read(&args[1])?;
    let oracle_bytes = fs::read(&args[2])?;
    let cases: Value = serde_json::from_slice(&cases_bytes)?;
    let oracle: Value = serde_json::from_slice(&oracle_bytes)?;
    let mut report = evaluate_bundle(&cases, &oracle);
    report["input_sha256"] = serde_json::json!({
        "cases": format!("{:x}", Sha256::digest(&cases_bytes)),
        "oracle": format!("{:x}", Sha256::digest(&oracle_bytes))
    });
    fs::write(&args[3], serde_json::to_vec_pretty(&report)?)?;
    if report["conformant"].as_bool() != Some(true) {
        return Err(invalid(
            "DevSecOps reference bundle did not match its separate oracle",
        ));
    }
    eprintln!(
        "DevSecOps reference: {} scenarios, {} conformant; dispatch_eligible=false",
        report["case_count"], report["conformant_case_count"]
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("devsecops-agent-reference: {error}");
        std::process::exit(2);
    }
}
