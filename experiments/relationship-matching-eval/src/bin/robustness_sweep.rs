use std::env;
use std::process;

use relationship_matching_eval::{
    corpus_fingerprint_fnv1a64, evaluate, recommend, synthetic_market_for_scenario,
    EvaluationMetrics, MarketScenario, RankingPolicy,
};

const POLICY_NAMES: [&str; 4] = [
    "feasibility_only",
    "one_sided_utility",
    "mutual_utility",
    "exposure_balanced_mutual",
];

const POLICIES: [RankingPolicy; 4] = [
    RankingPolicy::FeasibilityOnly,
    RankingPolicy::OneSidedUtility,
    RankingPolicy::MutualUtility,
    RankingPolicy::ExposureBalancedMutual,
];

#[derive(Clone)]
struct SeedRow {
    seed: u64,
    corpus_fingerprint: String,
    metrics: [EvaluationMetrics; 4],
}

fn main() {
    if let Err(error) = run() {
        eprintln!("relationship-matching-robustness: {error}");
        eprintln!(
            "usage: relationship-matching-robustness [seed-start] [even-participant-count] [top-k] [seed-count]"
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

    let source_commit_sha = read_optional_commit_sha()?;
    let rustc_version = env::var("RUSTC_VERSION").ok();
    let mut scenario_results: Vec<(MarketScenario, Vec<SeedRow>)> =
        Vec::with_capacity(MarketScenario::ALL.len());

    for scenario in MarketScenario::ALL {
        let mut rows = Vec::with_capacity(seed_count);
        for offset in 0..seed_count {
            let seed = seed_start.wrapping_add(offset as u64);
            let market = synthetic_market_for_scenario(seed, participant_count, scenario)?;
            let mut policy_metrics = Vec::with_capacity(POLICIES.len());

            for policy in POLICIES {
                let recommendations = recommend(&market, top_k, policy)?;
                let metrics = evaluate(&market, &recommendations);
                if metrics.invalid_recommendation_count != 0 {
                    return Err(format!(
                        "invalid recommendation rows in scenario {} seed {} policy {:?}: {}",
                        scenario.name(),
                        seed,
                        policy,
                        metrics.invalid_recommendation_count
                    ));
                }
                policy_metrics.push(metrics);
            }

            let metrics: [EvaluationMetrics; 4] = policy_metrics
                .try_into()
                .map_err(|_| "internal policy metric count mismatch".to_string())?;

            rows.push(SeedRow {
                seed,
                corpus_fingerprint: corpus_fingerprint_fnv1a64(&market),
                metrics,
            });
        }
        scenario_results.push((scenario, rows));
    }

    println!("{{");
    println!("  \"schema\": \"relationship-matching-robustness-v1\",");
    println!("  \"seed_start\": {seed_start},");
    println!("  \"seed_count\": {seed_count},");
    println!("  \"participant_count\": {participant_count},");
    println!("  \"top_k\": {top_k},");
    println!(
        "  \"source_commit_sha\": {},",
        source_commit_sha
            .as_deref()
            .map(json_string)
            .unwrap_or_else(|| "null".to_string())
    );
    println!(
        "  \"rustc_version\": {},",
        rustc_version
            .as_deref()
            .map(json_string)
            .unwrap_or_else(|| "null".to_string())
    );
    println!("  \"target_os\": {},", json_string(env::consts::OS));
    println!("  \"target_arch\": {},", json_string(env::consts::ARCH));
    println!("  \"fingerprints_are_cryptographic\": false,");
    println!("  \"scenario_count\": {},", scenario_results.len());
    println!("  \"interpretation\": \"synthetic stress tests measure robustness of ranking and allocation only; they do not establish real-user fairness, consent, safety, or relationship quality\",");
    println!("  \"scenarios\": [");

    for (scenario_index, (scenario, rows)) in scenario_results.iter().enumerate() {
        println!("    {{");
        println!("      \"name\": {},", json_string(scenario.name()));
        println!("      \"seed_count\": {},", rows.len());
        println!("      \"policies\": {{");
        for policy_index in 0..POLICY_NAMES.len() {
            println!("        \"{}\": {{", POLICY_NAMES[policy_index]);
            print_policy_summary(rows, policy_index, "          ");
            println!(
                "        }}{}",
                if policy_index + 1 == POLICY_NAMES.len() { "" } else { "," }
            );
        }
        println!("      }},");
        println!("      \"relative_reduction_vs_one_sided_utility\": {{");
        let raw_reductions = relative_reductions(rows, false);
        let normalized_reductions = relative_reductions(rows, true);
        println!("        \"raw_exposure_gini\": {{");
        print_stats_values(&raw_reductions, "          ");
        println!("        }},");
        println!("        \"opportunity_normalized_exposure_gini\": {{");
        print_stats_values(&normalized_reductions, "          ");
        println!("        }},");
        println!(
            "        \"raw_gini_improved_seed_count\": {},",
            raw_reductions.iter().filter(|value| **value > 0.0).count()
        );
        println!(
            "        \"opportunity_normalized_gini_improved_seed_count\": {}",
            normalized_reductions.iter().filter(|value| **value > 0.0).count()
        );
        println!("      }},");
        println!("      \"per_seed\": [");
        for (row_index, row) in rows.iter().enumerate() {
            println!("        {{");
            println!("          \"seed\": {},", row.seed);
            println!(
                "          \"corpus_fingerprint_fnv1a64\": {},",
                json_string(&row.corpus_fingerprint)
            );
            println!("          \"policies\": {{");
            for policy_index in 0..POLICY_NAMES.len() {
                println!("            \"{}\": {{", POLICY_NAMES[policy_index]);
                print_seed_metrics(row.metrics[policy_index], "              ");
                println!(
                    "            }}{}",
                    if policy_index + 1 == POLICY_NAMES.len() { "" } else { "," }
                );
            }
            println!("          }}");
            println!(
                "        }}{}",
                if row_index + 1 == rows.len() { "" } else { "," }
            );
        }
        println!("      ]");
        println!(
            "    }}{}",
            if scenario_index + 1 == scenario_results.len() { "" } else { "," }
        );
    }

    println!("  ]");
    println!("}}");
    Ok(())
}

fn print_policy_summary(rows: &[SeedRow], policy_index: usize, indent: &str) {
    for (name, values) in [
        (
            "candidate_coverage",
            rows.iter()
                .filter(|row| row.metrics[policy_index].eligible_candidate_users > 0)
                .map(|row| row.metrics[policy_index].candidate_coverage)
                .collect::<Vec<_>>(),
        ),
        (
            "exposure_gini",
            rows.iter()
                .filter(|row| row.metrics[policy_index].eligible_candidate_users > 0)
                .map(|row| row.metrics[policy_index].exposure_gini)
                .collect::<Vec<_>>(),
        ),
        (
            "opportunity_normalized_exposure_gini",
            rows.iter()
                .filter(|row| row.metrics[policy_index].eligible_candidate_users > 0)
                .map(|row| row.metrics[policy_index].opportunity_normalized_exposure_gini)
                .collect::<Vec<_>>(),
        ),
        (
            "mutual_shortlist_pair_count",
            rows.iter()
                .map(|row| row.metrics[policy_index].mutual_shortlist_pair_count as f64)
                .collect::<Vec<_>>(),
        ),
        (
            "recommendation_count",
            rows.iter()
                .map(|row| row.metrics[policy_index].recommendation_count as f64)
                .collect::<Vec<_>>(),
        ),
    ] {
        println!("{}\"{}\": {{", indent, name);
        print_stats_values(&values, &format!("{indent}  "));
        println!("{} }},", indent);
    }

    println!(
        "{}\"feasible_pair_count_mean\": {:.8},",
        indent,
        mean(&rows.iter().map(|row| row.metrics[policy_index].feasible_pair_count as f64).collect::<Vec<_>>())
    );
    println!(
        "{}\"invalid_recommendation_count_total\": {}",
        indent,
        rows.iter().map(|row| row.metrics[policy_index].invalid_recommendation_count).sum::<usize>()
    );
}

fn print_stats_values(values: &[f64], indent: &str) {
    if values.is_empty() {
        println!("{}\"n\": 0,", indent);
        println!("{}\"mean\": null,", indent);
        println!("{}\"sample_sd\": null,", indent);
        println!("{}\"min\": null,", indent);
        println!("{}\"max\": null,", indent);
        println!("{}\"ci95_normal_approx\": null", indent);
        return;
    }

    let (average, sample_sd, minimum, maximum, ci_low, ci_high) = stats(values);
    println!("{}\"n\": {},", indent, values.len());
    println!("{}\"mean\": {:.8},", indent, average);
    println!("{}\"sample_sd\": {:.8},", indent, sample_sd);
    println!("{}\"min\": {:.8},", indent, minimum);
    println!("{}\"max\": {:.8},", indent, maximum);
    if values.len() > 1 {
        println!(
            "{}\"ci95_normal_approx\": [{:.8}, {:.8}]",
            indent, ci_low, ci_high
        );
    } else {
        println!("{}\"ci95_normal_approx\": null", indent);
    }
}

fn print_seed_metrics(metrics: EvaluationMetrics, indent: &str) {
    println!("{}\"feasible_pair_count\": {},", indent, metrics.feasible_pair_count);
    println!("{}\"eligible_candidate_users\": {},", indent, metrics.eligible_candidate_users);
    println!("{}\"exposed_candidate_users\": {},", indent, metrics.exposed_candidate_users);
    if metrics.eligible_candidate_users == 0 {
        println!("{}\"candidate_coverage\": null,", indent);
        println!("{}\"exposure_gini\": null,", indent);
        println!("{}\"opportunity_normalized_exposure_gini\": null,", indent);
    } else {
        println!("{}\"candidate_coverage\": {:.8},", indent, metrics.candidate_coverage);
        println!("{}\"exposure_gini\": {:.8},", indent, metrics.exposure_gini);
        println!(
            "{}\"opportunity_normalized_exposure_gini\": {:.8},",
            indent, metrics.opportunity_normalized_exposure_gini
        );
    }
    println!("{}\"recommendation_count\": {},", indent, metrics.recommendation_count);
    println!(
        "{}\"mutual_shortlist_pair_count\": {},",
        indent, metrics.mutual_shortlist_pair_count
    );
    println!(
        "{}\"invalid_recommendation_count\": {}",
        indent, metrics.invalid_recommendation_count
    );
}

fn relative_reductions(rows: &[SeedRow], opportunity_normalized: bool) -> Vec<f64> {
    rows.iter()
        .filter_map(|row| {
            let baseline = if opportunity_normalized {
                row.metrics[1].opportunity_normalized_exposure_gini
            } else {
                row.metrics[1].exposure_gini
            };
            let compared = if opportunity_normalized {
                row.metrics[3].opportunity_normalized_exposure_gini
            } else {
                row.metrics[3].exposure_gini
            };
            if baseline > 0.0 {
                Some(1.0 - compared / baseline)
            } else {
                None
            }
        })
        .collect()
}

fn stats(values: &[f64]) -> (f64, f64, f64, f64, f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    let average = mean(values);
    let minimum = values.iter().copied().fold(f64::INFINITY, f64::min);
    let maximum = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let sample_sd = if values.len() > 1 {
        let squared_deviations = values.iter().map(|value| (value - average).powi(2)).sum::<f64>();
        (squared_deviations / (values.len() - 1) as f64).sqrt()
    } else {
        0.0
    };
    let margin = 1.96 * sample_sd / (values.len() as f64).sqrt();
    (average, sample_sd, minimum, maximum, average - margin, average + margin)
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

fn read_optional_commit_sha() -> Result<Option<String>, String> {
    let Some(value) = env::var("GIT_COMMIT_SHA").ok() else {
        return Ok(None);
    };
    let value = value.trim();
    if (value.len() != 40 && value.len() != 64)
        || !value.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("GIT_COMMIT_SHA must be an exact 40- or 64-character hexadecimal commit SHA".to_string());
    }
    Ok(Some(value.to_ascii_lowercase()))
}

fn json_string(value: &str) -> String {
    let mut output = String::with_capacity(value.len() + 2);
    output.push('"');
    for character in value.chars() {
        match character {
            '"' => output.push_str("\\\""),
            '\\' => output.push_str("\\\\"),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            character if character.is_control() => {
                output.push_str(&format!("\\u{:04x}", character as u32));
            }
            character => output.push(character),
        }
    }
    output.push('"');
    output
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
        Some(value) => value.parse::<T>().map_err(|_| format!("invalid {label}: {value}")),
        None => Ok(default),
    }
}
