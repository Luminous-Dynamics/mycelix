use fsre_service_trajectory::{evaluate, Observation, ServiceContract};
use std::collections::BTreeMap;
use std::env;
use std::fs;

fn main() {
    if let Err(message) = run() {
        eprintln!("error: {message}");
        eprintln!("usage: fsre-service-trajectory <observations.csv>");
        std::process::exit(2);
    }
}

fn run() -> Result<(), String> {
    let mut arguments = env::args().skip(1);
    let input_path = arguments
        .next()
        .ok_or_else(|| "missing input CSV path".to_owned())?;
    if arguments.next().is_some() {
        return Err("expected exactly one input CSV path".to_owned());
    }

    let source = fs::read_to_string(&input_path)
        .map_err(|error| format!("could not read {input_path}: {error}"))?;
    let (contract, observations) = parse_input(&source)?;
    let report = evaluate(&contract, &observations).map_err(|error| error.to_string())?;

    let recovery_status = if report.first_known_breach_at_seconds.is_none() {
        "NO_KNOWN_BREACH"
    } else if report.safe_recovery_at_seconds.is_some() {
        "SAFE_RECOVERY_OBSERVED"
    } else {
        "SAFE_RECOVERY_NOT_OBSERVED_WITHIN_HORIZON"
    };

    println!("report_schema=fsre-service-trajectory-report.v0.1");
    println!("service_id={}", report.service_id);
    println!("horizon_seconds={}", report.horizon_seconds);
    println!("known_seconds={}", report.known_seconds);
    println!("unknown_seconds={}", report.unknown_seconds);
    println!("full_service_seconds={}", report.full_service_seconds);
    println!("safe_degraded_seconds={}", report.safe_degraded_seconds);
    println!(
        "below_minimum_service_seconds={}",
        report.below_minimum_service_seconds
    );
    println!(
        "quality_noncompliant_seconds={}",
        report.quality_noncompliant_seconds
    );
    println!("unsafe_service_seconds={}", report.unsafe_service_seconds);
    println!(
        "longest_unsafe_episode_seconds={}",
        report.longest_unsafe_episode_seconds
    );
    println!(
        "service_deficit_fraction_seconds={:.6}",
        report.service_deficit_fraction_seconds
    );
    println!(
        "first_known_breach_at_seconds={}",
        display_option(report.first_known_breach_at_seconds, "none")
    );
    println!("recovery_status={recovery_status}");
    println!(
        "safe_recovery_at_seconds={}",
        display_option(report.safe_recovery_at_seconds, "not_observed")
    );
    println!(
        "time_to_safe_recovery_seconds={}",
        display_option(report.time_to_safe_recovery_seconds, "not_observed")
    );
    println!(
        "full_recovery_at_seconds={}",
        display_option(report.full_recovery_at_seconds, "not_observed")
    );
    println!(
        "time_to_full_recovery_seconds={}",
        display_option(report.time_to_full_recovery_seconds, "not_observed")
    );

    Ok(())
}

fn display_option(value: Option<u64>, absent: &str) -> String {
    value
        .map(|seconds| seconds.to_string())
        .unwrap_or_else(|| absent.to_owned())
}

fn parse_input(source: &str) -> Result<(ServiceContract, Vec<Observation>), String> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut observations = Vec::new();

    for (index, raw_line) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = raw_line.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim().to_owned();
                let value = value.trim().to_owned();
                if metadata.insert(key.clone(), value).is_some() {
                    return Err(format!("line {line_number}: duplicate metadata key {key}"));
                }
            }
            continue;
        }

        let columns: Vec<&str> = line.split(',').map(str::trim).collect();
        if columns.len() != 3 {
            return Err(format!(
                "line {line_number}: expected at_seconds,service_fraction,quality_compliant"
            ));
        }

        let at_seconds = columns[0]
            .parse::<u64>()
            .map_err(|error| format!("line {line_number}: invalid at_seconds: {error}"))?;
        let service_fraction = columns[1]
            .parse::<f64>()
            .map_err(|error| format!("line {line_number}: invalid service_fraction: {error}"))?;
        let quality_compliant = match columns[2] {
            "true" => true,
            "false" => false,
            _ => {
                return Err(format!(
                    "line {line_number}: quality_compliant must be true or false"
                ));
            }
        };

        observations.push(Observation {
            at_seconds,
            service_fraction,
            quality_compliant,
        });
    }

    let service_id = take_required(&mut metadata, "service_id")?;
    let horizon_start_seconds = take_u64(&mut metadata, "horizon_start_seconds")?;
    let horizon_end_seconds = take_u64(&mut metadata, "horizon_end_seconds")?;
    let minimum_safe_service_fraction = take_f64(&mut metadata, "minimum_safe_service_fraction")?;
    let max_observation_hold_seconds = take_u64(&mut metadata, "max_observation_hold_seconds")?;
    let recovery_stability_seconds = take_u64(&mut metadata, "recovery_stability_seconds")?;
    let quality_required = match take_required(&mut metadata, "quality_required")?.as_str() {
        "true" => true,
        "false" => false,
        _ => return Err("quality_required must be true or false".to_owned()),
    };

    if let Some((unknown_key, _)) = metadata.into_iter().next() {
        return Err(format!("unknown metadata key: {unknown_key}"));
    }

    Ok((
        ServiceContract {
            service_id,
            horizon_start_seconds,
            horizon_end_seconds,
            minimum_safe_service_fraction,
            max_observation_hold_seconds,
            recovery_stability_seconds,
            quality_required,
        },
        observations,
    ))
}

fn take_required(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<String, String> {
    metadata
        .remove(key)
        .ok_or_else(|| format!("missing metadata key: {key}"))
}

fn take_u64(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<u64, String> {
    let value = take_required(metadata, key)?;
    value
        .parse::<u64>()
        .map_err(|error| format!("metadata {key}: {error}"))
}

fn take_f64(metadata: &mut BTreeMap<String, String>, key: &str) -> Result<f64, String> {
    let value = take_required(metadata, key)?;
    value
        .parse::<f64>()
        .map_err(|error| format!("metadata {key}: {error}"))
}
