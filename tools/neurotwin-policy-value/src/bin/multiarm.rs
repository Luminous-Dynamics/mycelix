use std::env;
use std::fs;
use std::process;

use neurotwin_policy_value::multi_arm::{DEFAULT_MIN_REPORT_N, evaluate, parse_tsv, render_json};

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() < 2 || args.len() > 3 {
        return Err(
            "usage: neurotwin-multiarm-policy-value INPUT.tsv OUTPUT.json [MIN_REPORT_N]"
                .to_string(),
        );
    }

    let minimum_report_n = if let Some(value) = args.get(2) {
        value
            .parse::<usize>()
            .map_err(|_| "MIN_REPORT_N must be a positive integer".to_string())?
    } else {
        DEFAULT_MIN_REPORT_N
    };
    if minimum_report_n < DEFAULT_MIN_REPORT_N {
        return Err(format!(
            "MIN_REPORT_N cannot be below the built-in privacy floor of {DEFAULT_MIN_REPORT_N}"
        ));
    }
    if args[0] == args[1] {
        return Err("input and output paths must be different".to_string());
    }

    let input = fs::read_to_string(&args[0])
        .map_err(|_| "could not read input TSV; input contents were not logged".to_string())?;
    let dataset = parse_tsv(&input)?;
    if dataset.observations.len() < minimum_report_n {
        return Err(
            "aggregate output suppressed because the cohort is below the reporting minimum"
                .to_string(),
        );
    }

    let report = evaluate(&dataset)?;
    fs::write(&args[1], render_json(&report))
        .map_err(|_| "could not write aggregate multi-arm JSON output".to_string())?;
    eprintln!(
        "aggregate multi-arm report written for {} observations; row-level data were not emitted",
        report.n
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("neurotwin-multiarm-policy-value: {error}");
        process::exit(2);
    }
}
