use space_weighted_anytime_confidence::{
    BettingGrid, ConditionalMeanEvidence, bounded_mean_confidence_sequence,
};
use std::{env, process};

fn usage() -> &'static str {
    "Usage: space-weighted-anytime-confidence <range-lower> <range-upper> <delta> <comma-separated-values> <sampling-design-id> <conditional-mean-model-id> <evidence-ref> <sample-order-digest>"
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 8 || args.iter().any(|arg| arg == "--help" || arg == "-h") {
        eprintln!("{}", usage());
        process::exit(64);
    }
    let parse = |index: usize, name: &str| -> f64 {
        args[index].parse::<f64>().unwrap_or_else(|error| {
            eprintln!("invalid {name}: {error}");
            process::exit(64);
        })
    };
    let lower = parse(0, "range lower");
    let upper = parse(1, "range upper");
    let delta = parse(2, "delta");
    let values = args[3]
        .split(',')
        .map(|part| part.parse::<f64>())
        .collect::<Result<Vec<_>, _>>()
        .unwrap_or_else(|error| {
            eprintln!("invalid observation value: {error}");
            process::exit(64);
        });
    let evidence = ConditionalMeanEvidence {
        sampling_design_id: args[4].clone(),
        conditional_mean_model_id: args[5].clone(),
        evidence_ref: args[6].clone(),
        sample_order_digest: args[7].clone(),
    };

    match bounded_mean_confidence_sequence(
        &values,
        lower,
        upper,
        delta,
        &BettingGrid::default_grid(),
        evidence,
    ) {
        Ok(report) => {
            println!("{{");
            println!("  \"method\": \"{}\",", report.method);
            println!("  \"range_lower\": {:.12},", report.range_lower);
            println!("  \"range_upper\": {:.12},", report.range_upper);
            println!("  \"delta\": {:.8},", report.delta);
            println!(
                "  \"two_sided_confidence_level\": {:.8},",
                report.two_sided_confidence_level
            );
            println!("  \"bet_grid_id\": \"{}\",", report.bet_grid_id);
            println!(
                "  \"sampling_design_id\": \"{}\",",
                report.sampling_design_id
            );
            println!(
                "  \"conditional_mean_model_id\": \"{}\",",
                report.conditional_mean_model_id
            );
            println!(
                "  \"conditional_mean_evidence_ref\": \"{}\",",
                report.conditional_mean_evidence_ref
            );
            println!(
                "  \"sample_order_digest\": \"{}\",",
                report.sample_order_digest
            );
            println!("  \"estimand_scope\": \"{}\",", report.estimand_scope);
            println!("  \"points\": [");
            for (index, point) in report.points.iter().enumerate() {
                let comma = if index + 1 == report.points.len() {
                    ""
                } else {
                    ","
                };
                println!(
                    "    {{\"time\": {}, \"observed_mean\": {:.12}, \"lower_confidence_bound\": {:.12}, \"upper_confidence_bound\": {:.12}}}{}",
                    point.time,
                    point.observed_mean,
                    point.lower_confidence_bound,
                    point.upper_confidence_bound,
                    comma
                );
            }
            println!("  ]");
            println!("}}");
        }
        Err(error) => {
            eprintln!("cannot produce bounded-variable confidence sequence: {error}");
            process::exit(2);
        }
    }
}
