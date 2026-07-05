use net_steward_discovery::{load_incident_capsule_from_directory, verify_incident_capsule};
use net_steward_schema::IncidentCapsule;
use std::env;
use std::fs;

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: net-steward-verify-capsule <path-to-incident-capsule.json>");
        std::process::exit(1);
    }

    let target_path = std::path::Path::new(&args[1]);
    let capsule: IncidentCapsule = if target_path.is_dir() {
        match load_incident_capsule_from_directory(target_path) {
            Ok(cap) => cap,
            Err(e) => {
                eprintln!(
                    "Error loading Incident Capsule from directory {}: {}",
                    args[1], e
                );
                std::process::exit(1);
            }
        }
    } else {
        let content = match fs::read_to_string(target_path) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Error reading file {}: {}", args[1], e);
                std::process::exit(1);
            }
        };
        match serde_json::from_str(&content) {
            Ok(cap) => cap,
            Err(e) => {
                eprintln!("Error parsing Incident Capsule JSON: {}", e);
                std::process::exit(1);
            }
        }
    };

    let report = verify_incident_capsule(&capsule);

    println!("capsule_id: {}", report.capsule_id);
    println!("schema: {}", report.schema_version);
    println!(
        "hashes: {}",
        if report.hashes_valid {
            "valid"
        } else {
            "invalid"
        }
    );
    println!(
        "evidence ledger: {}",
        if report.evidence_ledger_valid {
            "valid chain"
        } else {
            "invalid chain"
        }
    );
    println!(
        "rollback plan: {}",
        if report.rollback_plan_valid {
            "dry-run only"
        } else {
            "requires validation"
        }
    );
    println!(
        "security events: {}",
        if report.security_events_valid {
            "evidence hashes present"
        } else {
            "missing evidence hashes"
        }
    );
    println!("proof status: {:?}", report.proof_status);
    println!(
        "mutation claims: {}",
        if report.mutation_claims_found {
            "FOUND (UNAUTHORIZED)"
        } else {
            "none"
        }
    );
    println!(
        "result: {}",
        if report.result_passed { "PASS" } else { "FAIL" }
    );

    if !report.result_passed {
        std::process::exit(1);
    }
}
