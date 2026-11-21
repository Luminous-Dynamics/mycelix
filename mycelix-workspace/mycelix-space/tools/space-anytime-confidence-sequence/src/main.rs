use space_anytime_confidence_sequence::{confidence_sequence, BetGrid, ConditionalMeanEvidence};
use std::{env, process};

fn usage() -> &'static str {
    "Usage: space-anytime-confidence-sequence <delta> <binary-observations> <model-id> <evidence-ref>"
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 4 || args.iter().any(|arg| arg == "--help" || arg == "-h") {
        eprintln!("{}", usage());
        process::exit(64);
    }
    let delta = args[0].parse::<f64>().unwrap_or_else(|error| {
        eprintln!("invalid delta: {error}");
        process::exit(64);
    });
    let mut observations = Vec::with_capacity(args[1].len());
    for (index, byte) in args[1].bytes().enumerate() {
        match byte {
            b'0' => observations.push(false),
            b'1' => observations.push(true),
            other => {
                eprintln!("observation {index} must be 0 or 1, got byte {other}");
                process::exit(64);
            }
        }
    }
    let evidence = ConditionalMeanEvidence {
        model_id: args[2].clone(),
        evidence_ref: args[3].clone(),
    };
    match confidence_sequence(&observations, delta, &BetGrid::default_grid(), evidence) {
        Ok(report) => {
            println!("{{");
            println!("  \"method\": \"{}\",", report.method);
            println!("  \"delta\": {:.8},", report.delta);
            println!("  \"one_sided_confidence_level\": {:.8},", report.one_sided_confidence_level);
            println!("  \"bet_grid_id\": \"{}\",", report.bet_grid_id);
            println!("  \"conditional_mean_model_id\": \"{}\",", report.conditional_mean_model_id);
            println!("  \"conditional_mean_evidence_ref\": \"{}\",", report.conditional_mean_evidence_ref);
            println!("  \"null_hypothesis\": \"{}\",", report.null_hypothesis);
            println!("  \"claim_ceiling\": \"{}\",", report.claim_ceiling);
            println!("  \"points\": [");
            for (index, point) in report.points.iter().enumerate() {
                let comma = if index + 1 == report.points.len() { "" } else { "," };
                println!(
                    "    {{\"time\": {}, \"observed_violations\": {}, \"empirical_rate\": {:.10}, \"upper_confidence_bound\": {:.10}}}{}",
                    point.time, point.observed_violations, point.empirical_rate,
                    point.upper_confidence_bound, comma
                );
            }
            println!("  ]");
            println!("}}");
        }
        Err(error) => {
            eprintln!("cannot produce anytime confidence sequence: {error}");
            process::exit(2);
        }
    }
}
