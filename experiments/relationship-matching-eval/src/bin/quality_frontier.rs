use std::collections::BTreeMap;
use std::env;
use std::process;

use relationship_matching_eval::{
    accumulate_exposure_history, corpus_fingerprint_fnv1a64, directed_utility,
    eligible_pair, evaluate_cumulative_exposure, recommend,
    recommend_exposure_balanced_with_history, recommend_quality_bounded_exposure_with_history,
    synthetic_market_for_scenario, CumulativeExposureMetrics, MarketScenario, Participant,
    Recommendation, RankingPolicy,
};

const STRATEGY_NAMES: [&str; 7] = [
    "one_sided_utility",
    "mutual_utility",
    "batch_balanced_mutual",
    "opportunity_history_balanced",
    "quality_floor_70",
    "quality_floor_80",
    "quality_floor_90",
];

const QUALITY_FLOORS: [u8; 3] = [70, 80, 90];

#[derive(Clone)]
struct SeedResult {
    seed: u64,
    fingerprint: String,
    exposure_metrics: [CumulativeExposureMetrics; 7],
    quality_retention: [Option<f64>; 7],
}

fn main() {
    if let Err(error) = run() {
        eprintln!("relationship-matching-quality-frontier: {error}");
        eprintln!(
            "usage: cargo run --bin quality_frontier -- [seed-start] [even-participant-count] [top-k] [seed-count] [round-count]"
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
    let mut scenarios: Vec<(MarketScenario, Vec<SeedResult>)> =
        Vec::with_capacity(MarketScenario::ALL.len());

    for scenario in MarketScenario::ALL {
        let mut scenario_rows = Vec::with_capacity(seed_count);
        for offset in 0..seed_count {
            let seed = seed_start.wrapping_add(offset as u64);
            let market = synthetic_market_for_scenario(seed, participant_count, scenario)?;
            scenario_rows.push(evaluate_market(&market, seed, top_k, round_count)?);
        }
        scenarios.push((scenario, scenario_rows));
    }

    println!("{{");
    println!("  \"schema\": \"relationship-matching-quality-frontier-v1\",");
    println!("  \"seed_start\": {seed_start},");
    println!("  \"seed_count\": {seed_count},");
    println!("  \"participant_count\": {participant_count},");
    println!("  \"top_k\": {top_k},");
    println!("  \"round_count\": {round_count},");
    println!(
        "  \"source_commit_sha\": {},",
        source_commit_sha.as_deref().map(json_string).unwrap_or_else(|| "null".to_string())
    );
    println!(
        "  \"rustc_version\": {},",
        rustc_version.as_deref().map(json_string).unwrap_or_else(|| "null".to_string())
    );
    println!("  \"target_os\": {},", json_string(env::consts::OS));
    println!("  \"target_arch\": {},", json_string(env::consts::ARCH));
    println!("  \"scenario_count\": {},", scenarios.len());
    println!("  \"interpretation\": \"quality retention is a synthetic mutual-utility-floor proxy, not human acceptance or relationship quality; exposure metrics are cumulative over fixed synthetic markets\",");
    println!("  \"strategies\": [");
    for (index, name) in STRATEGY_NAMES.iter().enumerate() {
        println!("    {}{}", json_string(name), if index + 1 == STRATEGY_NAMES.len() { "" } else { "," });
    }
    println!("  ],");
    println!("  \"scenarios\": [");

    for (scenario_index, (scenario, rows)) in scenarios.iter().enumerate() {
        println!("    {{");
        println!("      \"name\": {},", json_string(scenario.name()));
        println!("      \"seed_count\": {},", rows.len());
        println!("      \"strategy_summary\": {{");
        for strategy_index in 0..STRATEGY_NAMES.len() {
            println!("        {}: {{", json_string(STRATEGY_NAMES[strategy_index]));
            print_metric_summary(rows, strategy_index, "          ");
            println!(
                "        }}{}",
                if strategy_index + 1 == STRATEGY_NAMES.len() { "" } else { "," }
            );
        }
        println!("      }},");
        println!("      \"raw_gini_reduction_vs_batch_balanced\": {{");
        for (offset, strategy_index) in [4usize, 5, 6].iter().enumerate() {
            print_reduction(
                rows,
                *strategy_index,
                2,
                STRATEGY_NAMES[*strategy_index],
                "        ",
                offset < 2,
            );
        }
        println!("      }},");
        println!("      \"raw_gini_reduction_vs_opportunity_history\": {{");
        for (offset, strategy_index) in [4usize, 5, 6].iter().enumerate() {
            print_reduction(
                rows,
                *strategy_index,
                3,
                STRATEGY_NAMES[*strategy_index],
                "        ",
                offset < 2,
            );
        }
        println!("      }},");
        println!("      \"per_seed\": [");
        for (row_index, row) in rows.iter().enumerate() {
            println!("        {{");
            println!("          \"seed\": {},", row.seed);
            println!("          \"corpus_fingerprint_fnv1a64\": {},", json_string(&row.fingerprint));
            println!("          \"strategies\": {{");
            for strategy_index in 0..STRATEGY_NAMES.len() {
                println!("            {}: {{", json_string(STRATEGY_NAMES[strategy_index]));
                print_seed_metrics(
                    row.exposure_metrics[strategy_index],
                    row.quality_retention[strategy_index],
                    "              ",
                );
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
            if scenario_index + 1 == scenarios.len() { "" } else { "," }
        );
    }
    println!("  ]");
    println!("}}");
    Ok(())
}

fn evaluate_market(
    market: &[Participant],
    seed: u64,
    top_k: usize,
    round_count: usize,
) -> Result<SeedResult, String> {
    let mut histories: [BTreeMap<u32, usize>; 7] = std::array::from_fn(|_| BTreeMap::new());
    let mut quality_samples: [Vec<f64>; 7] = std::array::from_fn(|_| Vec::new());

    for _round in 0..round_count {
        let recommendations: [Vec<Recommendation>; 7] = [
            recommend(market, top_k, RankingPolicy::OneSidedUtility)?,
            recommend(market, top_k, RankingPolicy::MutualUtility)?,
            recommend(market, top_k, RankingPolicy::ExposureBalancedMutual)?,
            recommend_exposure_balanced_with_history(market, top_k, &histories[3])?,
            recommend_quality_bounded_exposure_with_history(market, top_k, &histories[4], QUALITY_FLOORS[0])?,
            recommend_quality_bounded_exposure_with_history(market, top_k, &histories[5], QUALITY_FLOORS[1])?,
            recommend_quality_bounded_exposure_with_history(market, top_k, &histories[6], QUALITY_FLOORS[2])?,
        ];

        for (index, rows) in recommendations.iter().enumerate() {
            if rows.iter().any(|row| {
                let viewer = market.iter().find(|person| person.id == row.viewer_id);
                let candidate = market.iter().find(|person| person.id == row.candidate_id);
                !matches!((viewer, candidate), (Some(a), Some(b)) if eligible_pair(a, b))
            }) {
                return Err(format!(
                    "strategy {} emitted an ineligible row for seed {}",
                    STRATEGY_NAMES[index], seed
                ));
            }
            append_quality_retention(market, rows, &mut quality_samples[index]);
            accumulate_exposure_history(&mut histories[index], rows)?;
        }
    }

    let exposure_metrics_vec: Vec<CumulativeExposureMetrics> = histories
        .iter()
        .map(|history| evaluate_cumulative_exposure(market, history))
        .collect::<Result<Vec<_>, _>>()?;
    let exposure_metrics: [CumulativeExposureMetrics; 7] = exposure_metrics_vec
        .try_into()
        .map_err(|_| "internal metric count mismatch".to_string())?;
    let quality_retention: [Option<f64>; 7] = std::array::from_fn(|index| {
        let values = &quality_samples[index];
        if values.is_empty() {
            None
        } else {
            Some(values.iter().sum::<f64>() / values.len() as f64)
        }
    });

    Ok(SeedResult {
        seed,
        fingerprint: corpus_fingerprint_fnv1a64(market),
        exposure_metrics,
        quality_retention,
    })
}

fn append_quality_retention(
    market: &[Participant],
    recommendations: &[Recommendation],
    output: &mut Vec<f64>,
) {
    for row in recommendations {
        let Some(viewer) = market.iter().find(|person| person.id == row.viewer_id) else {
            continue;
        };
        let chosen = row.viewer_utility.min(row.candidate_utility);
        let best = market
            .iter()
            .filter(|person| eligible_pair(viewer, person))
            .map(|person| directed_utility(viewer, person).min(directed_utility(person, viewer)))
            .max()
            .unwrap_or(0);
        if best > 0 {
            output.push(chosen as f64 / best as f64);
        }
    }
}

fn print_metric_summary(rows: &[SeedResult], strategy_index: usize, indent: &str) {
    let metrics: Vec<CumulativeExposureMetrics> =
        rows.iter().map(|row| row.exposure_metrics[strategy_index]).collect();
    print_stats(
        "candidate_coverage",
        &metrics
            .iter()
            .filter(|metric| metric.eligible_candidate_users > 0)
            .map(|metric| metric.candidate_coverage)
            .collect::<Vec<_>>(),
        indent,
        true,
    );
    print_stats(
        "raw_exposure_gini",
        &metrics
            .iter()
            .filter(|metric| metric.eligible_candidate_users > 0)
            .map(|metric| metric.exposure_gini)
            .collect::<Vec<_>>(),
        indent,
        true,
    );
    print_stats(
        "opportunity_normalized_exposure_gini",
        &metrics
            .iter()
            .filter(|metric| metric.eligible_candidate_users > 0)
            .map(|metric| metric.opportunity_normalized_exposure_gini)
            .collect::<Vec<_>>(),
        indent,
        true,
    );
    let retention: Vec<f64> = rows
        .iter()
        .filter_map(|row| row.quality_retention[strategy_index])
        .collect();
    print_stats("relative_mutual_floor_retention", &retention, indent, false);
}

fn print_reduction(
    rows: &[SeedResult],
    strategy_index: usize,
    baseline_index: usize,
    key: &str,
    indent: &str,
    trailing_comma: bool,
) {
    let reductions: Vec<f64> = rows
        .iter()
        .filter_map(|row| {
            let baseline = row.exposure_metrics[baseline_index];
            let compared = row.exposure_metrics[strategy_index];
            if baseline.eligible_candidate_users > 0 && baseline.exposure_gini > 0.0 {
                Some(1.0 - compared.exposure_gini / baseline.exposure_gini)
            } else {
                None
            }
        })
        .collect();
    print_stats(key, &reductions, indent, trailing_comma);
}

fn print_stats(name: &str, values: &[f64], indent: &str, trailing_comma: bool) {
    let property_suffix = if trailing_comma { "," } else { "" };
    if values.is_empty() {
        println!(
            "{}{}: {{\"n\": 0, \"mean\": null, \"sample_sd\": null, \"min\": null, \"max\": null, \"ci95_normal_approx\": null}}{}",
            indent, json_string(name), property_suffix
        );
        return;
    }
    let average = values.iter().sum::<f64>() / values.len() as f64;
    let min = values.iter().copied().fold(f64::INFINITY, f64::min);
    let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let sd = if values.len() > 1 {
        (values
            .iter()
            .map(|value| (value - average).powi(2))
            .sum::<f64>()
            / (values.len() - 1) as f64)
            .sqrt()
    } else {
        0.0
    };
    let margin = 1.96 * sd / (values.len() as f64).sqrt();
    println!(
        "{}{}: {{\"n\": {}, \"mean\": {:.8}, \"sample_sd\": {:.8}, \"min\": {:.8}, \"max\": {:.8}, \"ci95_normal_approx\": {}}}{}",
        indent,
        json_string(name),
        values.len(),
        average,
        sd,
        min,
        max,
        if values.len() > 1 {
            format!("[{:.8}, {:.8}]", average - margin, average + margin)
        } else {
            "null".to_string()
        },
        property_suffix
    );
}

fn print_seed_metrics(
    metrics: CumulativeExposureMetrics,
    quality_retention: Option<f64>,
    indent: &str,
) {
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
    println!("{}\"total_exposure\": {},", indent, metrics.total_exposure);
    println!(
        "{}\"relative_mutual_floor_retention\": {}",
        indent,
        quality_retention
            .map(|value| format!("{value:.8}"))
            .unwrap_or_else(|| "null".to_string())
    );
}

fn read_optional_commit_sha() -> Result<Option<String>, String> {
    let Some(value) = env::var("GIT_COMMIT_SHA").ok() else {
        return Ok(None);
    };
    let value = value.trim();
    if (value.len() != 40 && value.len() != 64) || !value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
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
