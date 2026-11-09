use civil_awareness_cross_stack_oracle::evaluate_bytes;
use std::{env, fs::File, io::Read, process::ExitCode};
use civil_awareness_cross_stack_oracle::MAX_INPUT_BYTES;

fn main() -> ExitCode {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 4 {
        eprintln!("usage: civil-awareness-cross-stack-oracle <golden-corpus.json> <oracle-inputs.json> <handoff-template.json> <handoff-verifier-report.json>");
        return ExitCode::from(64);
    }

    let mut data = Vec::with_capacity(4);
    for path in &args {
        let file = match File::open(path) {
            Ok(file) => file,
            Err(_) => {
                eprintln!("oracle.input_read_failed");
                return ExitCode::from(66);
            }
        };
        let mut bytes = Vec::with_capacity(64 * 1024);
        if file
            .take((MAX_INPUT_BYTES + 1) as u64)
            .read_to_end(&mut bytes)
            .is_err()
        {
            eprintln!("oracle.input_read_failed");
            return ExitCode::from(66);
        }
        if bytes.len() > MAX_INPUT_BYTES {
            eprintln!("oracle.input_too_large");
            return ExitCode::from(65);
        }
        data.push(bytes);
    }

    match evaluate_bytes(&data[0], &data[1], &data[2], &data[3]) {
        Ok(report) => {
            if let Err(_) = serde_json::to_writer_pretty(std::io::stdout().lock(), &report) {
                eprintln!("oracle.output_write_failed");
                return ExitCode::from(74);
            }
            if report.disposition == "PASS" {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(1)
            }
        }
        Err(error) => {
            eprintln!("oracle.input_rejected: {error:?}");
            ExitCode::from(65)
        }
    }
}
