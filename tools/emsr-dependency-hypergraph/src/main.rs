use emsr_dependency_hypergraph::{evaluate_bytes, independently_verify};
use std::env;
use std::fs;

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("evaluate") if args.len() == 4 => {
            let input = read_bounded(&args[2])?;
            let report = evaluate_bytes(&input);
            write_json(&args[3], &report)
        }
        Some("verify") if args.len() == 4 => {
            let input = read_bounded(&args[2])?;
            let report = read_bounded(&args[3])?;
            let verification = independently_verify(&input, &report);
            println!("{}", serde_json::to_string_pretty(&verification).map_err(|e| e.to_string())?);
            if verification.status == "verified" {
                Ok(())
            } else {
                Err("independent reference verifier rejected the report".to_owned())
            }
        }
        _ => Err(
            "usage: emsr-dependency-hypergraph evaluate <input.json> <report.json>\n       emsr-dependency-hypergraph verify <input.json> <report.json>".to_owned()
        )
    }
}

fn read_bounded(path: &str) -> Result<Vec<u8>, String> {
    let metadata = fs::metadata(path).map_err(|e| format!("cannot inspect {path}: {e}"))?;
    if metadata.len() > 16 * 1024 * 1024 {
        return Err(format!("input exceeds 16 MiB: {path}"));
    }
    fs::read(path).map_err(|e| format!("cannot read {path}: {e}"))
}

fn write_json(path: &str, value: &impl serde::Serialize) -> Result<(), String> {
    let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
    fs::write(path, bytes).map_err(|e| format!("cannot write {path}: {e}"))
}
