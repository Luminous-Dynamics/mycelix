use std::io::Read;
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

    // Bound bytes read, rather than trusting metadata length alone. A file
    // can grow between metadata() and read(); take(MAX+1) caps memory even
    // under that race and lets the validator reject an oversized input.
    let file = match std::fs::File::open(&path) {
        Ok(file) => file,
        Err(_) => {
            eprintln!("input.read_failed");
            return ExitCode::from(74);
        }
    };
    let mut bytes = Vec::with_capacity(MAX_ENVELOPE_BYTES.min(64 * 1024));
    let read_result = file
        .take((MAX_ENVELOPE_BYTES + 1) as u64)
        .read_to_end(&mut bytes);
    if read_result.is_err() {
        eprintln!("input.read_failed");
        return ExitCode::from(74);
    }
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
