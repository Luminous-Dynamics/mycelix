use std::env;
use std::process;

use relationship_matching_eval::{
    corpus_fingerprint_fnv1a64, evaluate, recommend, synthetic_market, EvaluationMetrics,
    RankingPolicy,
};

const POLICY_NAMES: [&str; 4] = [
    "feasibility_only",
    "one_sided_utility",
    "mutual_utility",
    "exposure_balanced_mutual",
];

#[derive(Clone)]
struct SeedResult {
    seed: u64,
    corpus_fingerprint: String,
    metrics: [EvaluationMetrics; 4],
}

fn main() {
    if let Err(error) = run() {
        eprintln!("relationship-matching-eval: {error}");
        eprintln!(
            "usage: relationship-matching-eval [seed-start] [even-participant-count] [top-k] [seed-count]"
        );
        process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() > 4 {
        return Err("too many arguments".to_string());
    }

    let seed_start = parse_or_default(args.first(), 20_261_017u64, "seed-start")?;
    let participant_count = parse_or_default(args.get(1), 200usize, "participant count")?;
    let top_k = parse_or_default(args.get(2), 5usize, "top-k")?;
    let seed_count = parse_or_default(args.get(3), 30usize, "seed count")?;

    if participant_count < 4 || participant_count % 2 != 0 || participant_count > 2_000 {
        return Err("participant count must be even and in 4..=2000".to_string());
    }
    if top_k == 0 || top_k > participant_count {
        return Err("top-k must be in 1..=participant-count".to_string());
    }
    if seed_count == 0 || seed_count > 1_000 {
        return Err("seed count must be in 1..=1000".to_string());
    }

    let mut results = Vec::with_capacity(seed_count);
    for offset in 0..seed_count {
        let seed = seed_start.wrapping_add(offset as u64);
        results.push(evaluate_seed(seed, participant_count, top_k)?);
    }

    let relative_gini_reductions: Vec<f64> = results
        .iter()
        .filter_map(|result| {
            let baseline = result.metrics[1].exposure_gini;
            if baseline > 0.0 {
                Some(1.0 - result.metrics[3].exposure_gini / baseline)
            } else {
                None
            }
        })
        .collect();

    println!("{{");
    println!("  \"schema\": \"relationship-matching-eval-report-v1\",");
    println!("  \"seed_start\": {seed_start},");
    println!("  \"seed_count\": {seed_count},");
    println!("  \"participant_count\": {participant_count},");
    println!("  \"top_k\": {top_k},");
    println!("  \"fingerprint_is_cryptographic\": false,");
    println!("  \"interpretation\": \"synthetic discovery eligibility is not pair-specific consent; reciprocal shortlist is not human acceptance or relationship quality\",");
    println!("  \"policies\": {{");
    for (index, name) in POLICY_NAMES.iter().enumerate() {
        println!("    \"{name}\": {{");
        print_policy_summary(&results, index, "      ");
        println!("    }}{}", if index + 1 == POLICY_NAMES.len() { "" } else { "," });
    }
    println!("  }},");
    println!("  \"exposure_balanced_vs_one_sided\": {{");
    println!("    \"comparable_seed_count\": {},", relative_gini_reductions.len());
    println!(
        "    \"improved_seed_count\": {},",
        relative_gini_reductions.iter().filter(|value| **value > 0.0).count()
    );
    print_stats_object("relative_gini_reduction", &relative_gini_reductions, "    ", false);
    println!("  }},");
    println!("  \"per_seed\": [");
    for (index, result) in results.iter().enumerate() {
        println!("    {{");
        println!("      \"seed\": {},", result.seed);
        println!(
            "      \"corpus_fingerprint_fnv1a64\": \"{}\",",
            result.corpus_fingerprint
        );
        println!("      \"policies\": {{");
        for (policy_index, name) in POLICY_NAMES.iter().enumerate() {
            println!("        \"{name}\": {{");
            print_metrics(result.metrics[policy_index], "          ");
            println!(
                "        }}{}",
                if policy_index + 1 == POLICY_NAMES.len() { "" } else { "," }
            );
        }
        println!("      }}");
        println!(
            "    }}{}",
            if index + 1 == results.len() { "" } else { "," }
        );
    }
    println!("  ]");
    println!("}}");

    Ok(())
}

fn evaluate_seed(
    seed: u64,
    participant_count: usize,
    top_k: usize,
) -> Result<SeedResult, String> {
    let market = synthetic_market(seed, participant_count)?;
    let feasibility_rows = recommend(&market, top_k, RankingPolicy::FeasibilityOnly)?;
    let one_sided_rows = recommend(&market, top_k, RankingPolicy::OneSidedUtility)?;
    let mutual_rows = recommend(&market, top_k, RankingPolicy::MutualUtility)?;
    let balanced_rows = recommend(&market, top_k, RankingPolicy::ExposureBalancedMutual)?;

    let metrics = [
        evaluate(&market, &feasibility_rows),
        evaluate(&market, &one_sided_rows),
        evaluate(&market, &mutual_rows),
        evaluate(&market, &balanced_rows),
    ];

    Ok(SeedResult {
        seed,
        corpus_fingerprint: corpus_fingerprint_fnv1a64(&market),
        metrics,
    })
}

fn print_policy_summary(results: &[SeedResult], policy_index: usize, indent: &str) {
    let metrics: Vec<EvaluationMetrics> = results
        .iter()
        .map(|result| result.metrics[policy_index])
        .collect();

    println!(
        "{indent}\"feasible_pair_count_mean\": {:.6},",
        mean(&metrics.iter().map(|item| item.feasible_pair_count as f64).collect::<Vec<_>>())
    );
    println!(
        "{indent}\"eligible_candidate_users_mean\": {:.6},",
        mean(&metrics.iter().map(|item| item.eligible_candidate_users as f64).collect::<Vec<_>>())
    );
    println!(
        "{indent}\"exposed_candidate_users_mean\": {:.6},",
        mean(&metrics.iter().map(|item| item.exposed_candidate_users as f64).collect::<Vec<_>>())
    );
    println!(
        "{indent}\"invalid_recommendation_count_total\": {},",
        metrics.iter().map(|item| item.invalid_recommendation_count).sum::<usize>()
    );

    print_stats_object(
        "candidate_coverage",
        &metrics.iter().map(|item| item.candidate_coverage).collect::<Vec<_>>(),
        indent,
        true,
    );
    print_stats_object(
        "exposure_gini",
        &metrics.iter().map(|item| item.exposure_gini).collect::<Vec<_>>(),
        indent,
        true,
    );
    print_stats_object(
        "recommendation_count",
        &metrics.iter().map(|item| item.recommendation_count as f64).collect::<Vec<_>>(),
        indent,
        true,
    );
    print_stats_object(
        "mutual_shortlist_pair_count",
        &metrics.iter().map(|item| item.mutual_shortlist_pair_count as f64).collect::<Vec<_>>(),
        indent,
        false,
    );
}

fn print_stats_object(name: &str, values: &[f64], indent: &str, trailing_comma: bool) {
    let (average, sample_sd, minimum, maximum, ci_low, ci_high) = stats(values);
    println!("{indent}\"{name}\": {{");
    println!("{indent}  \"mean\": {average:.8},");
    println!("{indent}  \"sample_sd\": {sample_sd:.8},");
    println!("{indent}  \"min\": {minimum:.8},");
    println!("{indent}  \"max\": {maximum:.8},");
    if values.len() > 1 {
        println!("{indent}  \"ci95_normal_approx\": [{ci_low:.8}, {ci_high:.8}]");
    } else {
        println!("{indent}  \"ci95_normal_approx\": null");
    }
    println!(
        "{indent}}}{}",
        if trailing_comma { "," } else { "" }
    );
}

fn print_metrics(metrics: EvaluationMetrics, indent: &str) {
    println!(
        "{indent}\"feasible_pair_count\": {},",
        metrics.feasible_pair_count
    );
    println!(
        "{indent}\"eligible_candidate_users\": {},",
        metrics.eligible_candidate_users
    );
    println!(
        "{indent}\"exposed_candidate_users\": {},",
        metrics.exposed_candidate_users
    );
    println!(
        "{indent}\"candidate_coverage\": {:.8},",
        metrics.candidate_coverage
    );
    println!(
        "{indent}\"exposure_gini\": {:.8},",
        metrics.exposure_gini
    );
    println!(
        "{indent}\"recommendation_count\": {},",
        metrics.recommendation_count
    );
    println!(
        "{indent}\"mutual_shortlist_pair_count\": {},",
        metrics.mutual_shortlist_pair_count
    );
    println!(
        "{indent}\"invalid_recommendation_count\": {}",
        metrics.invalid_recommendation_count
    );
}

fn stats(values: &[f64]) -> (f64, f64, f64, f64, f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    let average = mean(values);
    let minimum = values.iter().copied().fold(f64::INFINITY, f64::min);
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let sample_sd = if values.len() > 1 {
        let squared_deviations = values
            .iter()
            .map(|value| (value - average).powi(2))
            .sum::<f64>();
        (squared_deviations / (values.len() - 1) as f64).sqrt()
    } else {
        0.0
    };
    let margin = 1.96 * sample_sd / (values.len() as f64).sqrt();

    (
        average,
        sample_sd,
        minimum,
        maximum,
        average - margin,
        average + margin,
    )
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
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
