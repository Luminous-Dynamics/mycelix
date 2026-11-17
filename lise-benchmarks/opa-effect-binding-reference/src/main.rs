use lise_opa_effect_binding_reference::evaluate;
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
            "usage: opa-effect-binding-reference <fixture.json> <report.json>",
        ));
    }
    let input_bytes = fs::read(&args[1])?;
    let input: Value = serde_json::from_slice(&input_bytes)?;
    let mut result = evaluate(&input);
    result["input_sha256"] = serde_json::json!(format!("{:x}", Sha256::digest(&input_bytes)));
    fs::write(&args[2], serde_json::to_vec_pretty(&result)?)?;
    eprintln!(
        "OPA effect binding: overall={}, dispatch_eligible=false",
        result["overall"]
    );
    if result["overall"] == "INVALID_INPUT" {
        return Err(invalid("OPA receipt fixture is invalid; see report"));
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("opa-effect-binding-reference: {error}");
        std::process::exit(2);
    }
}
