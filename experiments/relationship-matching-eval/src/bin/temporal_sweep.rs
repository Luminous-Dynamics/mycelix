use std::collections::BTreeMap;
use std::env;
use std::process;

use relationship_matching_eval::{
    accumulate_exposure_history, corpus_fingerprint_fnv1a64,
    evaluate_cumulative_exposure, recommend, recommend_exposure_balanced_with_history,
    synthetic_market_for_scenario, CumulativeExposureMetrics, MarketScenario, RankingPolicy,
};

const STRATEGY_NAMES: [&str; 3] = [
    "one_sided_repeated",
    "batch_balanced_repeated",
    "historical_exposure_balanced",
];

#[derive(Clone)]
struct SeedResult {
    seed: u64,
    corpus_fingerprint: String,
    metrics: [CumulativeExposureMetrics; 3],
}

fn main() {
    if let Err(error) = run() {
        eprintln!("relationship-matching-temporal-sweep: {error}");
        eprintln!(
            "usage: relationship-matching-temporal-sweep [seed-start] [even-participant-count] [top-k] [seed-count] [round-count]"
        );
        process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() > 5 {
        return Err("too many arguments".to_string());
    }

    let seed_start = parse_or_default(args.first(), 20_261_017u64, "seed-start")?;
    let participant_count = parse_or_default(args.get(1), 200usize, "participant count")?;
    let top_k = parse_or_default(args.get(2), 5usize, "top-k")?;
    let seed_count = parse_or_default(args.get(3), 30usize, "seed count")?;
    let round_count = parse_or_default(args.get(4), 10usize, "round count")?;

    if participant_count < 4 || participant_count % 2 != 0 || participant_count > 2_000 {
        return Err("participant count must be even and in 4..=2000".to_string());
    }
    if top_k == 0 || top_k > participant_count {
        return Err("top-k must be in 1..=participant-count".to_string());
    }
    if seed_count == 0 || seed_count > 1_000 {
        return Err("seed count must be in 1..=1000".to_string());
    }
    if round_count == 0 || round_count > 1_000 {
        return Err("round count must be in 1..=1000".to_string());
    }

    let source_commit_sha = read_optional_commit_sha()?;
    let rustc_version = env::var("RUSTC_VERSION").ok();
    let mut scenario_results: Vec<(MarketScenario, Vec<SeedResult>)> =
        Vec::with_capacity(MarketScenario::ALL.len());

    for scenario in MarketScenario::ALL {
        let mut rows = Vec::with_capacity(seed_count);
        for offset in 0..seed_count {
            let seed = seed_start.wrapping_add(offset as u64);
            let market = synthetic_market_for_scenario(seed, participant_count, scenario)?;
            let mut histories: [BTreeMap<u32, usize>; 3] =
                [BTreeMap::new(), BTreeMap::new(), BTreeMap::new()];

            for _round in 0..round_count {
                let one_sided = recommend(&market, top_k, RankingPolicy::OneSidedUtility)?;
                let batch_balanced =
                    recommend(&market, top_k, RankingPolicy::ExposureBalancedMutual)?;
                let historical_balanced = recommend_exposure_balanced_with_history(
                    &market,
                    top_k,
                    &histories[2],
                )?;

                for (index, recommendations) in [
                    one_sided.as_slice(),
                    batch_balanced.as_slice(),
                    historical_balanced.as_slice(),
                ]
                .iter()
                .enumerate()
                {
                    // All recommendation rows originate from the same eligibility-gated
                    // APIs. History accumulation accepts repeated pairs across rounds,
                    // which are the subject of this cumulative-exposure experiment.
                    accumulate_exposure_history(&mut histories[index], recommendations)?;
                }
            }

            let metrics: [CumulativeExposureMetrics; 3] = [
                evaluate_cumulative_exposure(&market, &histories[0])?,
                evaluate_cumulative_exposure(&market, &histories[1])?,
                evaluate_cumulative_exposure(&market, &histories[2])?,
            ];

            rows.push(SeedResult {
                seed,
                corpus_fingerprint: corpus_fingerprint_fnv1a64(&market),
                metrics,
            });
        }
        scenario_results.push((scenario, rows));
    }

    println!("{{");
    println!("  \"schema\": \"relationship-matching-temporal-exposure-v1\",");
    println!("  \"seed_start\": {seed_start},");
    println!("  \"seed_count\": {seed_count},");
    println!("  \"participant_count\": {participant_count},");
    println!("  \"top_k\": {top_k},");
    println!("  \"round_count\": {round_count},");
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
    println!("  \"scenario_count\": {},", scenario_results.len());
    println!("  \"interpretation\": \"synthetic cumulative exposure only; this is not evidence of consent, human acceptance, or relationship quality\",");
    println!("  \"strategies\": [");
    for (index, name) in STRATEGY_NAMES.iter().enumerate() {
        println!("    {}{}", json_string(name), if index + 1 == STRATEGY_NAMES.len() { "" } else { "," });
    }
    println!("  ],");
    println!("  \"scenarios\": [");

    for (scenario_index, (scenario, rows)) in scenario_results.iter().enumerate() {
        println!("    {{");
        println!("      \"name\": {},", json_string(scenario.name()));
        println!("      \"seed_count\": {},", rows.len());
        println!("      \"strategies\": {{");
        for strategy_index in 0..STRATEGY_NAMES.len() {
            println!("        {}: {{", json_string(STRATEGY_NAMES[strategy_index]));
            print_strategy_summary(rows, strategy_index, "          ");
            println!(
                "        }}{}",
                if strategy_index + 1 == STRATEGY_NAMES.len() { "" } else { "," }
            );
        }
        println!("      }},");
        println!("      \"relative_raw_gini_reduction\": {{");
        let versus_one_sided = relative_reduction(rows, 0, 2, false);
        let versus_batch_balanced = relative_reduction(rows, 1, 2, false);
        println!("        \"historical_vs_one_sided\": {{");
        print_stats_values(&versus_one_sided, "          ");
        println!("        }},");
        println!("        \"historical_vs_batch_balanced\": {{");
        print_stats_values(&versus_batch_balanced, "          ");
        println!("        }}");
        println!("      }},");
        println!("      \"per_seed\": [");
        for (row_index, row) in rows.iter().enumerate() {
            println!("        {{");
            println!("          \"seed\": {},", row.seed);
            println!(
                "          \"corpus_fingerprint_fnv1a64\": {},",
                json_string(&row.corpus_fingerprint)
            );
            println!("          \"strategies\": {{");
            for strategy_index in 0..STRATEGY_NAMES.len() {
                println!("            {}: {{", json_string(STRATEGY_NAMES[strategy_index]));
                print_seed_metrics(row.metrics[strategy_index], "              ");
                println!(
                    "            }}{}",
                    if strategy_index + 1 == STRATEGY_NAMES.len() { "" } else { "," }
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

fn print_strategy_summary(rows: &[SeedResult], strategy_index: usize, indent: &str) {
    for (name, values) in [
        (
            "candidate_coverage",
            rows.iter().map(|row| row.metrics[strategy_index].candidate_coverage).collect::<Vec<_>>(),
        ),
        (
            "exposure_gini",
            rows.iter().map(|row| row.metrics[strategy_index].exposure_gini).collect::<Vec<_>>(),
        ),
        (
            "opportunity_normalized_exposure_gini",
            rows.iter()
                .map(|row| row.metrics[strategy_index].opportunity_normalized_exposure_gini)
                .collect::<Vec<_>>(),
        ),
    ] {
        println!("{}{}: {{", indent, json_string(name));
        print_stats_values(&values, &format!("{indent}  "));
        println!("{} }},", indent);
    }
    println!(
        "{}\"eligible_candidate_users_mean\": {:.8},",
        indent,
        mean(&rows.iter().map(|row| row.metrics[strategy_index].eligible_candidate_users as f64).collect::<Vec<_>>())
    );
    println!(
        "{}\"exposed_candidate_users_mean\": {:.8},",
        indent,
        mean(&rows.iter().map(|row| row.metrics[strategy_index].exposed_candidate_users as f64).collect::<Vec<_>>())
    );
    println!(
        "{}\"total_exposure_mean\": {:.8}",
        indent,
        mean(&rows.iter().map(|row| row.metrics[strategy_index].total_exposure as f64).collect::<Vec<_>>())
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

fn print_seed_metrics(metrics: CumulativeExposureMetrics, indent: &str) {
    println!("{}\"feasible_pair_count\": {},", indent, metrics.feasible_pair_count);
    println!("{}\"eligible_candidate_users\": {},", indent, metrics.eligible_candidate_users);
    println!("{}\"exposed_candidate_users\": {},", indent, metrics.exposed_candidate_users);
    println!("{}\"candidate_coverage\": {:.8},", indent, metrics.candidate_coverage);
    println!("{}\"total_exposure\": {},", indent, metrics.total_exposure);
    println!("{}\"exposure_gini\": {:.8},", indent, metrics.exposure_gini);
    println!(
        "{}\"opportunity_normalized_exposure_gini\": {:.8}",
        indent, metrics.opportunity_normalized_exposure_gini
    );
}

fn relative_reduction(rows: &[SeedResult], baseline: usize, compared: usize, opportunity: bool) -> Vec<f64> {
    rows.iter()
        .filter_map(|row| {
            let base = if opportunity {
                row.metrics[baseline].opportunity_normalized_exposure_gini
            } else {
                row.metrics[baseline].exposure_gini
            };
            let new = if opportunity {
                row.metrics[compared].opportunity_normalized_exposure_gini
            } else {
                row.metrics[compared].exposure_gini
            };
            if base > 0.0 { Some(1.0 - new / base) } else { None }
        })
        .collect()
}

fn stats(values: &[f64]) -> (f64, f64, f64, f64, f64, f64) {
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
    if values.is_empty() { 0.0 } else { values.iter().sum::<f64>() / values.len() as f64 }
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
            character if character.is_control() => output.push_str(&format!("\\u{:04x}", character as u32)),
            character => output.push(character),
        }
    }
    output.push('"');
    output
}

fn parse_or_default<T>(value: Option<&String>, default: T, label: &str) -> Result<T, String>
where
    T: std::str::FromStr,
{
    match value {
        Some(value) => value.parse::<T>().map_err(|_| format!("invalid {label}: {value}")),
        None => Ok(default),
    }
}
