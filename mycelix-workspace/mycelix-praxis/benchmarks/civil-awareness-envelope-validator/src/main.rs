use std::ffi::OsStr;
use std::io::{Read, Write};
use std::process::ExitCode;

use civil_awareness_envelope_validator::{
    Disposition, EnvelopeValidator, ObservabilityEvaluator, ObservabilityEvaluationReport,
    ObservabilityAssessment, ValidationReport, MAX_ENVELOPE_BYTES,
};
use serde::Serialize;

const MAX_OBSERVABILITY_BYTES: usize = 2 * 1024 * 1024;

fn emit_json<T: Serialize>(value: &T) -> bool {
    match serde_json::to_vec_pretty(value) {
        Ok(mut json) => {
            json.push(b'\n');
            std::io::stdout().write_all(&json).is_ok()
        }
        Err(_) => false,
    }
}

fn read_bounded(path: &OsStr, max_bytes: usize) -> Result<Vec<u8>, ()> {
    let file = std::fs::File::open(path).map_err(|_| ())?;
    let mut bytes = Vec::with_capacity(max_bytes.min(64 * 1024));
    file.take((max_bytes + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|_| ())?;
    if bytes.len() > max_bytes {
        return Err(());
    }
    Ok(bytes)
}

fn usage() -> ExitCode {
    eprintln!(
        "usage: civil-awareness-envelope-validator <envelope.json>\n       civil-awareness-envelope-validator --observability-dgp\n       civil-awareness-envelope-validator --observability-case <case.json>"
    );
    ExitCode::from(64)
}

fn run_observability_dgp() -> ExitCode {
    let evaluator = match ObservabilityEvaluator::from_embedded() {
        Ok(evaluator) => evaluator,
        Err(_) => {
            eprintln!("observability.initialization_failed");
            return ExitCode::from(70);
        }
    };
    let report: ObservabilityEvaluationReport = match evaluator.evaluate_embedded() {
        Ok(report) => report,
        Err(_) => {
            eprintln!("observability.dgp_evaluation_failed");
            return ExitCode::from(2);
        }
    };
    if !emit_json(&report) {
        eprintln!("observability.output_write_failed");
        return ExitCode::from(74);
    }
    ExitCode::SUCCESS
}

fn run_observability_case(path: &OsStr) -> ExitCode {
    let bytes = match read_bounded(path, MAX_OBSERVABILITY_BYTES) {
        Ok(bytes) => bytes,
        Err(_) => {
            eprintln!("observability.input_read_or_size_failure");
            return ExitCode::from(74);
        }
    };
    let evaluator = match ObservabilityEvaluator::from_embedded() {
        Ok(evaluator) => evaluator,
        Err(_) => {
            eprintln!("observability.initialization_failed");
            return ExitCode::from(70);
        }
    };
    let assessment: ObservabilityAssessment = match evaluator.assess_case_bytes(&bytes) {
        Ok(assessment) => assessment,
        Err(_) => {
            eprintln!("observability.case_validation_failed");
            return ExitCode::from(2);
        }
    };
    if !emit_json(&assessment) {
        eprintln!("observability.output_write_failed");
        return ExitCode::from(74);
    }
    ExitCode::SUCCESS
}

fn run_envelope(path: &OsStr) -> ExitCode {
    let bytes = match read_bounded(path, MAX_ENVELOPE_BYTES) {
        Ok(bytes) => bytes,
        Err(_) => {
            eprintln!("input.read_or_size_failure");
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
    let report: ValidationReport = validator.validate_bytes(&bytes);
    if !emit_json(&report) {
        eprintln!("validator.output_write_failed");
        return ExitCode::from(74);
    }

    match report.disposition {
        Disposition::StructurallyValidRequiresAuthorityReceipt => ExitCode::SUCCESS,
        Disposition::Rejected => ExitCode::from(2),
        Disposition::Quarantined => ExitCode::from(3),
        Disposition::Unresolved => ExitCode::from(4),
    }
}

fn main() -> ExitCode {
    let mut args = std::env::args_os();
    let _program = args.next();
    let Some(first) = args.next() else {
        return usage();
    };

    match first.to_str() {
        Some("--observability-dgp") => {
            if args.next().is_some() {
                return usage();
            }
            run_observability_dgp()
        }
        Some("--observability-case") => {
            let Some(path) = args.next() else {
                return usage();
            };
            if args.next().is_some() {
                return usage();
            }
            run_observability_case(&path)
        }
        _ => {
            if args.next().is_some() {
                return usage();
            }
            run_envelope(&first)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    fn temporary_path(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "civil-awareness-{label}-{}-{}.json",
            std::process::id(),
            std::thread::current().name().unwrap_or("test")
        ))
    }

    #[test]
    fn bounded_reader_rejects_files_over_limit() {
        let path = temporary_path("oversize");
        fs::write(&path, b"12345").expect("write fixture");
        let result = read_bounded(path.as_os_str(), 4);
        let _ = fs::remove_file(&path);
        assert!(result.is_err());
    }

    #[test]
    fn bounded_reader_accepts_file_at_limit() {
        let path = temporary_path("at-limit");
        fs::write(&path, b"1234").expect("write fixture");
        let result = read_bounded(path.as_os_str(), 4);
        let _ = fs::remove_file(&path);
        assert_eq!(result.expect("file is at limit"), b"1234");
    }

    #[test]
    fn observability_cli_limits_are_not_the_envelope_limit() {
        assert_eq!(MAX_OBSERVABILITY_BYTES, 2 * 1024 * 1024);
        assert_eq!(MAX_ENVELOPE_BYTES, 2 * 1024 * 1024);
    }
}
