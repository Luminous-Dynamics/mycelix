use std::{env, fs, process::ExitCode};
use fsre_temporal_identity_lineage::replay;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();
    if args.len() != 4 {
        eprintln!("usage: fsre-temporal-identity-lineage <events.psv> <recorded-cutoff> <effective-cutoff>");
        return ExitCode::from(2);
    }
    let recorded = match args[2].parse::<u64>() {
        Ok(value) => value,
        Err(error) => { eprintln!("invalid recorded cutoff: {error}"); return ExitCode::from(2); }
    };
    let effective = match args[3].parse::<u64>() {
        Ok(value) => value,
        Err(error) => { eprintln!("invalid effective cutoff: {error}"); return ExitCode::from(2); }
    };
    let source = match fs::read_to_string(&args[1]) {
        Ok(value) => value,
        Err(error) => { eprintln!("cannot read {}: {error}", args[1]); return ExitCode::from(2); }
    };
    match replay(&source, recorded, effective) {
        Ok(report) => { print!("{}", report.render()); ExitCode::SUCCESS }
        Err(error) => { eprintln!("lineage validation failed: {error}"); ExitCode::from(1) }
    }
}
