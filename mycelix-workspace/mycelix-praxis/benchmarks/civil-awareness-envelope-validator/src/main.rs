use std::process::ExitCode;

use civil_awareness_envelope_validator::{Disposition, EnvelopeValidator, MAX_ENVELOPE_BYTES, ValidationReport};

fn emit(report: &ValidationReport) {
    match serde_json::to_string_pretty(report) {
        Ok(json) => println!("{json}"),
        Err(_) => {
            eprintln!("validator.output_serialization_failed");
        }
    }
}

fn main() -> ExitCode {
    let mut args = std::env::args_os();
    let _program = args.next();
    let Some(path) = args.next() else {
        eprintln!("usage: civil-awareness-envelope-validator <envelope.json>");
        return ExitCode::from(64);
    };
    if args.next().is_some() {
        eprintln!("usage: civil-awareness-envelope-validator <envelope.json>");
        return ExitCode::from(64);
    }

    let metadata = match std::fs::metadata(&path) {
        Ok(metadata) => metadata,
        Err(_) => {
            eprintln!("input.read_failed");
            return ExitCode::from(74);
        }
    };
    if metadata.len() > MAX_ENVELOPE_BYTES as u64 {
        let validator = match EnvelopeValidator::from_embedded() {
            Ok(validator) => validator,
            Err(_) => {
                eprintln!("validator.initialization_failed");
                return ExitCode::from(70);
            }
        };
        let report = validator.validate_bytes(&vec![0; MAX_ENVELOPE_BYTES + 1]);
        emit(&report);
        return ExitCode::from(2);
    }
    let bytes = match std::fs::read(&path) {
        Ok(bytes) => bytes,
        Err(_) => {
            eprintln!("input.read_failed");
            return ExitCode::from(74);
        }
    };
    let validator = match EnvelopeValidator::from_embedded() {
        Ok(validator) => validator,
        Err(_) => {
            eprintln!("validator.initialization_failed");
            return ExitCode::from(70);
        }
    };
    let report = validator.validate_bytes(&bytes);
    emit(&report);

    match report.disposition {
        Disposition::StructurallyValidRequiresAuthorityReceipt => ExitCode::SUCCESS,
        Disposition::Rejected => ExitCode::from(2),
        Disposition::Quarantined => ExitCode::from(3),
        Disposition::Unresolved => ExitCode::from(4),
    }
}
