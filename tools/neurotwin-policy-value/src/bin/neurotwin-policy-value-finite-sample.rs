use std::env;
use std::fs;
use std::io::Write;
use std::process;

use neurotwin_policy_value::finite_sample::{RandomizationDesign, evaluate_finite_sample};
use neurotwin_policy_value::{DEFAULT_MIN_REPORT_N, parse_tsv};

fn parse_finite(value: &str, label: &str) -> Result<f64, String> {
    let parsed = value
        .parse::<f64>()
        .map_err(|_| format!("{label} must be a finite number"))?;
    if parsed.is_finite() {
        Ok(parsed)
    } else {
        Err(format!("{label} must be a finite number"))
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if !(8..=10).contains(&args.len()) {
        return Err(
            "usage: neurotwin-policy-value-finite-sample INPUT.tsv OUTPUT.json --randomization-unit participant --assignment-mechanism independent_bernoulli --outcome-bounds LOWER,UPPER [CONFIDENCE_DELTA] [MIN_REPORT_N]"
                .to_string(),
        );
    }
    if args[2] != "--randomization-unit" || args[3] != "participant" {
        return Err("v0.1 requires --randomization-unit participant".to_string());
    }
    if args[4] != "--assignment-mechanism" || args[5] != "independent_bernoulli" {
        return Err(
            "v0.1 requires --assignment-mechanism independent_bernoulli; blocked, stratified, cluster and adaptive designs are unsupported"
                .to_string(),
        );
    }
    if args[6] != "--outcome-bounds" {
        return Err("missing required --outcome-bounds LOWER,UPPER declaration".to_string());
    }
    if args[0] == args[1] {
        return Err("input and output paths must be different".to_string());
    }

    let (lower_text, upper_text) = args[7]
        .split_once(',')
        .ok_or_else(|| "outcome bounds must use LOWER,UPPER syntax".to_string())?;
    let lower = parse_finite(lower_text, "outcome lower bound")?;
    let upper = parse_finite(upper_text, "outcome upper bound")?;
    if lower >= upper {
        return Err("outcome lower bound must be strictly below upper bound".to_string());
    }

    let confidence_delta = if let Some(value) = args.get(8) {
        parse_finite(value, "confidence delta")?
    } else {
        0.05
    };
    if !(0.0..1.0).contains(&confidence_delta) {
        return Err("confidence delta must be strictly between zero and one".to_string());
    }

    let minimum_report_n = if let Some(value) = args.get(9) {
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

    let input = fs::read_to_string(&args[0])
        .map_err(|_| "could not read input TSV; input contents were not logged".to_string())?;
    let observations = parse_tsv(&input)?;
    if observations.len() < minimum_report_n {
        return Err(
            "aggregate output suppressed because the cohort is below the reporting minimum"
                .to_string(),
        );
    }

    let report = evaluate_finite_sample(
        &observations,
        RandomizationDesign::IndependentBernoulliParticipant,
        lower,
        upper,
        confidence_delta,
    )?;
    let output = report.render_json();

    let mut file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&args[1])
        .map_err(|_| {
            "could not create aggregate output; existing files are never overwritten".to_string()
        })?;
    file.write_all(output.as_bytes())
        .map_err(|_| "could not write aggregate JSON output".to_string())?;

    eprintln!(
        "finite-sample aggregate report written for {} observations; row-level data were not emitted",
        report.n
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("neurotwin-policy-value-finite-sample: {error}");
        process::exit(2);
    }
}
