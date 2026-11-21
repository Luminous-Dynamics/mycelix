use chance_constraint_confidence::{clopper_pearson_upper, IidBernoulliEvidence};
use std::{env, process};

fn usage() -> &'static str {
    "Usage: chance-constraint-confidence <violations> <trials> <confidence-level> <sampling-design-id> <independence-evidence-ref>"
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 5 || args.iter().any(|arg| arg == "--help" || arg == "-h") {
        eprintln!("{}", usage());
        process::exit(64);
    }
    let violations = args[0].parse::<u64>().unwrap_or_else(|error| {
        eprintln!("invalid violations: {error}");
        process::exit(64);
    });
    let trials = args[1].parse::<u64>().unwrap_or_else(|error| {
        eprintln!("invalid trials: {error}");
        process::exit(64);
    });
    let confidence = args[2].parse::<f64>().unwrap_or_else(|error| {
        eprintln!("invalid confidence level: {error}");
        process::exit(64);
    });
    let evidence = IidBernoulliEvidence {
        sampling_design_id: args[3].clone(),
        evidence_ref: args[4].clone(),
    };

    match clopper_pearson_upper(violations, trials, confidence, evidence) {
        Ok(r) => println!(
            concat!(
                "{{\n",
                "  \"method\": \"{}\",\n",
                "  \"violations\": {},\n",
                "  \"trials\": {},\n",
                "  \"empirical_violation_rate\": {:.12},\n",
                "  \"confidence_level\": {:.6},\n",
                "  \"one_sided_alpha\": {:.6},\n",
                "  \"upper_confidence_bound\": {:.12},\n",
                "  \"sampling_design_id\": \"{}\",\n",
                "  \"independence_evidence_ref\": \"{}\",\n",
                "  \"probability_assumption\": \"{}\",\n",
                "  \"claim_ceiling\": \"{}\"\n",
                "}}"
            ),
            r.method,
            r.violations,
            r.trials,
            r.empirical_violation_rate,
            r.confidence_level,
            r.one_sided_alpha,
            r.upper_confidence_bound,
            r.sampling_design_id,
            r.independence_evidence_ref,
            r.probability_assumption,
            r.claim_ceiling,
        ),
        Err(error) => {
            eprintln!("cannot issue exact binomial bound: {error}");
            process::exit(2);
        }
    }
}
