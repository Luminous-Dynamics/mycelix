use lise_route_mediation_reference::evaluate;
use serde_json::Value;
use sha2::{Digest, Sha256};
use std::{env, error::Error, fs, io};

fn invalid(message: impl Into<String>) -> Box<dyn Error> {
    Box::new(io::Error::new(io::ErrorKind::InvalidInput, message.into()))
}

fn run() -> Result<(), Box<dyn Error>> {
    let args = env::args().collect::<Vec<_>>();
    if args.len() != 3 {
        return Err(invalid(
            "usage: route-mediation-reference <route-inventory.json> <report.json>",
        ));
    }
    let input_bytes = fs::read(&args[1])?;
    let input: Value = serde_json::from_slice(&input_bytes)?;
    let mut report = evaluate(&input);
    report["input_sha256"] = serde_json::json!(format!("{:x}", Sha256::digest(&input_bytes)));
    fs::write(&args[2], serde_json::to_vec_pretty(&report)?)?;
    let status = report["status"].as_str().unwrap_or("INVALID_INPUT");
    eprintln!(
        "route mediation: status={status}, paths={}, dispatch_eligible=false",
        report["path_count"]
    );
    if status == "INVALID_INPUT" {
        return Err(invalid("route inventory input invalid; see report"));
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("route-mediation-reference: {error}");
        std::process::exit(2);
    }
}
