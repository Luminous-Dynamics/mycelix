use iae_lise_conformance::evaluate_vertical_slice;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidInput, message.into()))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 4 {
        return Err(invalid(
            "usage: iae-lise-conformance <iae-fixture.json> <iae-submission.json> <report.json>",
        ));
    }
    let fixture_bytes = fs::read(&args[1])?;
    let submission_bytes = fs::read(&args[2])?;
    let fixture: Value = serde_json::from_slice(&fixture_bytes)?;
    let submission: Value = serde_json::from_slice(&submission_bytes)?;
    let mut report = evaluate_vertical_slice(&fixture, &submission);
    report["input_digests"] = serde_json::json!({
        "fixture_sha256": sha256(&fixture_bytes),
        "submission_sha256": sha256(&submission_bytes)
    });
    fs::write(&args[3], serde_json::to_vec_pretty(&report)?)?;
    if report["conformant"].as_bool() != Some(true) {
        return Err(invalid("IAE-to-LISE bridge conformance failed; see report"));
    }
    eprintln!(
        "IAE to LISE conformance: {} cases, {} passed; dispatch_eligible=false",
        report["case_count"], report["conformant_case_count"]
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("iae-lise-conformance: {error}");
        std::process::exit(2);
    }
}
