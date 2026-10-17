use std::env;
use std::process;

use relationship_matching_eval::{
    corpus_fingerprint_fnv1a64, evaluate, recommend, synthetic_market, EvaluationMetrics,
    RankingPolicy,
};

fn main() {
    if let Err(error) = run() {
        eprintln!("relationship-matching-eval: {error}");
        eprintln!("usage: relationship-matching-eval [seed] [even-participant-count] [top-k]");
        process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() > 3 {
        return Err("too many arguments".to_string());
    }

    let seed = parse_or_default(args.first(), 20_261_017u64, "seed")?;
    let participant_count = parse_or_default(args.get(1), 200usize, "participant count")?;
    let top_k = parse_or_default(args.get(2), 5usize, "top-k")?;

    let market = synthetic_market(seed, participant_count)?;
    let one_sided_rows = recommend(&market, top_k, RankingPolicy::OneSidedUtility)?;
    let mutual_rows = recommend(&market, top_k, RankingPolicy::MutualUtility)?;
    let one_sided_metrics = evaluate(&market, &one_sided_rows);
    let mutual_metrics = evaluate(&market, &mutual_rows);
    let fingerprint = corpus_fingerprint_fnv1a64(&market);

    println!("{{");
    println!("  \"schema\": \"relationship-matching-eval-report-v1\",");
    println!("  \"seed\": {seed},");
    println!("  \"participant_count\": {participant_count},");
    println!("  \"top_k\": {top_k},");
    println!("  \"corpus_fingerprint_fnv1a64\": \"{fingerprint}\",");
    println!("  \"fingerprint_is_cryptographic\": false,");
    println!("  \"interpretation\": \"mutual shortlist is not human acceptance or relationship quality\",");
    println!("  \"one_sided_utility\": {{");
    print_metrics(one_sided_metrics);
    println!("  }},");
    println!("  \"mutual_utility\": {{");
    print_metrics(mutual_metrics);
    println!("  }}");
    println!("}}");

    Ok(())
}

fn parse_or_default<T>(
    value: Option<&String>,
    default: T,
    label: &str,
) -> Result<T, String>
where
    T: std::str::FromStr,
{
    match value {
        Some(value) => value
            .parse::<T>()
            .map_err(|_| format!("invalid {label}: {value}")),
        None => Ok(default),
    }
}

fn print_metrics(metrics: EvaluationMetrics) {
    println!("    \"feasible_pair_count\": {},", metrics.feasible_pair_count);
    println!("    \"eligible_candidate_users\": {},", metrics.eligible_candidate_users);
    println!("    \"exposed_candidate_users\": {},", metrics.exposed_candidate_users);
    println!("    \"candidate_coverage\": {:.6},", metrics.candidate_coverage);
    println!("    \"exposure_gini\": {:.6},", metrics.exposure_gini);
    println!("    \"recommendation_count\": {},", metrics.recommendation_count);
    println!(
        "    \"mutual_shortlist_pair_count\": {},",
        metrics.mutual_shortlist_pair_count
    );
    println!(
        "    \"invalid_recommendation_count\": {}",
        metrics.invalid_recommendation_count
    );
}
