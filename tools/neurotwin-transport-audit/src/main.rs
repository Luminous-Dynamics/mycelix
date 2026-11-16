use std::env;
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::process;

use neurotwin_transport_audit::{DEFAULT_MIN_REPORT_N, evaluate, parse_tsv, render_json};

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 2 {
        return Err(
            "usage: neurotwin-transport-audit INPUT.tsv OUTPUT.json".to_string(),
        );
    }
    if args[0] == args[1] {
        return Err("input and output paths must be different".to_string());
    }

    let input = fs::read_to_string(&args[0])
        .map_err(|_| "could not read local prediction input".to_string())?;
    let rows = parse_tsv(&input)?;
    if rows.len() < DEFAULT_MIN_REPORT_N {
        return Err("overall cohort is below the built-in reporting minimum".to_string());
    }
    let report = evaluate(&rows)?;
    let mut output = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[1])
        .map_err(|_| "output path must be new and writable; existing files are never overwritten".to_string())?;
    output
        .write_all(render_json(&report).as_bytes())
        .map_err(|_| "could not write aggregate transport report".to_string())?;
    eprintln!("aggregate transport report written; row-level data were not emitted");
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("neurotwin-transport-audit: {error}");
        process::exit(2);
    }
}
