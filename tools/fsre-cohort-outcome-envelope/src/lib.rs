#![forbid(unsafe_code)]

// Multi-cohort distributional reporting over the existing single-lane trajectory
// evaluator. This crate does not infer population-wide completeness or equity weights.

use fsre_service_trajectory::{evaluate, Observation, ServiceContract, TrajectoryReport};
use std::collections::{BTreeMap, BTreeSet};

const COLUMN_COUNT: usize = 3;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct InputError(pub String);

impl std::fmt::Display for InputError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for InputError {}

#[derive(Clone, Debug)]
struct CohortLane {
    service_family_id: String,
    cohort_set_id: String,
    cohort_id: String,
    population_denominator: Option<u64>,
    population_unit: String,
    contract: ServiceContract,
    observations: Vec<Observation>,
}

#[derive(Clone, Debug)]
struct EvaluatedLane {
    cohort_id: String,
    population_denominator: Option<u64>,
    population_unit: String,
    minimum_safe_service_fraction: f64,
    trajectory: TrajectoryReport,
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 160
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/'))
}

fn required<'a>(metadata: &'a BTreeMap<String, String>, key: &str) -> Result<&'a str, InputError> {
    let value = metadata
        .get(key)
        .ok_or_else(|| InputError(format!("missing metadata key {key}")))?;
    if value.is_empty() {
        return Err(InputError(format!("empty metadata key {key}")));
    }
    Ok(value)
}

fn parse_u64(value: &str, key: &str) -> Result<u64, InputError> {
    value
        .parse::<u64>()
        .map_err(|error| InputError(format!("invalid {key}: {error}")))
}

fn parse_f64(value: &str, key: &str) -> Result<f64, InputError> {
    value
        .parse::<f64>()
        .map_err(|error| InputError(format!("invalid {key}: {error}")))
}

fn parse_bool(value: &str, key: &str) -> Result<bool, InputError> {
    match value {
        "true" => Ok(true),
        "false" => Ok(false),
        _ => Err(InputError(format!("{key} must be true or false"))),
    }
}

fn parse_lane(source: &str) -> Result<CohortLane, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut observations = Vec::new();

    for (offset, raw) in source.lines().enumerate() {
        let line_number = offset + 1;
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            if let Some((key, value)) = comment.trim().split_once('=') {
                let key = key.trim().to_owned();
                let value = value.trim().to_owned();
                if metadata.insert(key.clone(), value).is_some() {
                    return Err(InputError(format!(
                        "line {line_number}: duplicate metadata key {key}"
                    )));
                }
            }
            continue;
        }

        let columns: Vec<&str> = line.split(',').map(str::trim).collect();
        if columns.len() != COLUMN_COUNT {
            return Err(InputError(format!(
                "line {line_number}: expected at_seconds,service_fraction,quality_compliant"
            )));
        }
        let at_seconds = parse_u64(columns[0], "at_seconds")?;
        let service_fraction = parse_f64(columns[1], "service_fraction")?;
        let quality_compliant = parse_bool(columns[2], "quality_compliant")?;
        observations.push(Observation {
            at_seconds,
            service_fraction,
            quality_compliant,
        });
    }

    let service_id = required(&metadata, "service_id")?.to_owned();
    let service_family_id = required(&metadata, "service_family_id")?.to_owned();
    let cohort_set_id = required(&metadata, "cohort_set_id")?.to_owned();
    let cohort_id = required(&metadata, "cohort_id")?.to_owned();
    let population_value = required(&metadata, "population_denominator")?;
    let population_unit = required(&metadata, "population_unit")?.to_owned();

    for (key, value) in [
        ("service_id", service_id.as_str()),
        ("service_family_id", service_family_id.as_str()),
        ("cohort_set_id", cohort_set_id.as_str()),
        ("cohort_id", cohort_id.as_str()),
        ("population_unit", population_unit.as_str()),
    ] {
        if !valid_id(value) {
            return Err(InputError(format!("invalid identifier for {key}")));
        }
    }

    let population_denominator = if population_value == "UNKNOWN" {
        None
    } else {
        let value = parse_u64(population_value, "population_denominator")?;
        if value == 0 {
            return Err(InputError(
                "population_denominator must be positive or UNKNOWN".to_owned(),
            ));
        }
        Some(value)
    };

    let horizon_start_seconds = parse_u64(
        required(&metadata, "horizon_start_seconds")?,
        "horizon_start_seconds",
    )?;
    let horizon_end_seconds = parse_u64(
        required(&metadata, "horizon_end_seconds")?,
        "horizon_end_seconds",
    )?;
    let minimum_safe_service_fraction = parse_f64(
        required(&metadata, "minimum_safe_service_fraction")?,
        "minimum_safe_service_fraction",
    )?;
    let max_observation_hold_seconds = parse_u64(
        required(&metadata, "max_observation_hold_seconds")?,
        "max_observation_hold_seconds",
    )?;
    let recovery_stability_seconds = parse_u64(
        required(&metadata, "recovery_stability_seconds")?,
        "recovery_stability_seconds",
    )?;
    let quality_required = parse_bool(
        required(&metadata, "quality_required")?,
        "quality_required",
    )?;

    if let Some((unknown_key, _)) = metadata.into_iter().next() {
        return Err(InputError(format!("unknown metadata key {unknown_key}")));
    }

    Ok(CohortLane {
        service_family_id,
        cohort_set_id,
        cohort_id,
        population_denominator,
        population_unit,
        contract: ServiceContract {
            service_id,
            horizon_start_seconds,
            horizon_end_seconds,
            minimum_safe_service_fraction,
            max_observation_hold_seconds,
            recovery_stability_seconds,
            quality_required,
        },
        observations,
    })
}

fn percentile_nearest_rank(values: &[f64], percentile: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    let rank = (percentile * sorted.len() as f64).ceil() as usize;
    sorted.get(rank.saturating_sub(1).min(sorted.len() - 1)).copied()
}

fn display_float(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.6}"))
        .unwrap_or_else(|| "NOT_ESTIMABLE".to_owned())
}

fn display_u64(value: Option<u64>) -> String {
    value
        .map(|number| number.to_string())
        .unwrap_or_else(|| "NOT_OBSERVED".to_owned())
}

fn display_denominator(value: Option<u64>) -> String {
    value
        .map(|number| number.to_string())
        .unwrap_or_else(|| "UNKNOWN".to_owned())
}

fn evaluate_lanes(sources: &[String]) -> Result<Vec<EvaluatedLane>, InputError> {
    if sources.is_empty() {
        return Err(InputError("at least one cohort lane is required".to_owned()));
    }

    let mut parsed = Vec::with_capacity(sources.len());
    for source in sources {
        parsed.push(parse_lane(source)?);
    }

    let baseline = parsed[0].clone();
    let mut cohort_ids = BTreeSet::new();
    let mut evaluated = Vec::with_capacity(parsed.len());

    for lane in parsed {
        if !cohort_ids.insert(lane.cohort_id.clone()) {
            return Err(InputError(format!(
                "duplicate cohort_id {} in cohort set",
                lane.cohort_id
            )));
        }
        if lane.service_family_id != baseline.service_family_id
            || lane.cohort_set_id != baseline.cohort_set_id
            || lane.contract.service_id != baseline.contract.service_id
        {
            return Err(InputError(
                "all lanes must share service_family_id, cohort_set_id and service_id".to_owned(),
            ));
        }
        if lane.contract.horizon_start_seconds != baseline.contract.horizon_start_seconds
            || lane.contract.horizon_end_seconds != baseline.contract.horizon_end_seconds
            || lane.contract.max_observation_hold_seconds
                != baseline.contract.max_observation_hold_seconds
            || lane.contract.recovery_stability_seconds
                != baseline.contract.recovery_stability_seconds
            || lane.contract.quality_required != baseline.contract.quality_required
        {
            return Err(InputError(
                "all lanes must share horizon, observation hold, recovery stability and quality policy"
                    .to_owned(),
            ));
        }
        if lane.population_unit != baseline.population_unit {
            return Err(InputError(
                "all lanes must use the same population_denominator unit".to_owned(),
            ));
        }

        let report = evaluate(&lane.contract, &lane.observations)
            .map_err(|error| InputError(format!("cohort {}: {error}", lane.cohort_id)))?;
        evaluated.push(EvaluatedLane {
            cohort_id: lane.cohort_id,
            population_denominator: lane.population_denominator,
            population_unit: lane.population_unit,
            minimum_safe_service_fraction: lane.contract.minimum_safe_service_fraction,
            trajectory: report,
        });
    }

    evaluated.sort_by(|left, right| left.cohort_id.cmp(&right.cohort_id));
    Ok(evaluated)
}

fn compose(sources: &[String]) -> Result<String, InputError> {
    let lanes = evaluate_lanes(sources)?;
    let first_source = parse_lane(&sources[0])?;
    let cohort_count = lanes.len();
    let horizon_seconds = first_source.contract.horizon_end_seconds
        - first_source.contract.horizon_start_seconds;

    let unknown_lanes: Vec<&EvaluatedLane> = lanes
        .iter()
        .filter(|lane| lane.trajectory.unknown_seconds > 0)
        .collect();
    let unknown_seconds_total: u128 = lanes
        .iter()
        .map(|lane| u128::from(lane.trajectory.unknown_seconds))
        .sum();
    let disposition = if unknown_lanes.is_empty() {
        "ESTIMABLE_FOR_DECLARED_LANES"
    } else {
        "NOT_ESTIMABLE"
    };

    let deficits: Vec<f64> = lanes
        .iter()
        .map(|lane| lane.trajectory.service_deficit_fraction_seconds)
        .collect();
    let quality_seconds: Vec<f64> = lanes
        .iter()
        .map(|lane| lane.trajectory.quality_noncompliant_seconds as f64)
        .collect();
    let safe_recovery_times: Vec<f64> = lanes
        .iter()
        .filter_map(|lane| lane.trajectory.time_to_safe_recovery_seconds.map(|v| v as f64))
        .collect();

    let worst_deficit = lanes.iter().max_by(|left, right| {
        left.trajectory
            .service_deficit_fraction_seconds
            .total_cmp(&right.trajectory.service_deficit_fraction_seconds)
    });
    let worst_unsafe_episode = lanes.iter().max_by_key(|lane| {
        lane.trajectory.longest_unsafe_episode_seconds
    });
    let worst_quality = lanes.iter().max_by_key(|lane| {
        lane.trajectory.quality_noncompliant_seconds
    });
    let max_below_floor = lanes.iter().max_by_key(|lane| {
        lane.trajectory.below_minimum_service_seconds
    });

    let min_deficit = deficits.iter().copied().fold(f64::INFINITY, f64::min);
    let max_deficit = deficits.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let breached_count = lanes
        .iter()
        .filter(|lane| lane.trajectory.first_known_breach_at_seconds.is_some())
        .count();
    let recovered_count = lanes
        .iter()
        .filter(|lane| lane.trajectory.safe_recovery_at_seconds.is_some())
        .count();
    let unrecovered_breach_count = lanes
        .iter()
        .filter(|lane| {
            lane.trajectory.first_known_breach_at_seconds.is_some()
                && lane.trajectory.safe_recovery_at_seconds.is_none()
        })
        .count();

    let mut lines = vec![
        "report_schema=fsre-cohort-outcome-envelope-report.v0.1".to_owned(),
        format!("service_family_id={}", first_source.service_family_id),
        format!("service_id={}", first_source.contract.service_id),
        format!("cohort_set_id={}", first_source.cohort_set_id),
        format!("horizon_start_seconds={}", first_source.contract.horizon_start_seconds),
        format!("horizon_end_seconds={}", first_source.contract.horizon_end_seconds),
        format!("horizon_seconds={horizon_seconds}"),
        format!("cohort_count={cohort_count}"),
        format!("cohort_outcome_disposition={disposition}"),
        "cohort_set_completeness=NOT_ESTABLISHED_BY_THIS_TOOL".to_owned(),
        "population_weighted_aggregation=NOT_COMPUTED".to_owned(),
        "summary_semantics=UNWEIGHTED_DISTRIBUTION_ACROSS_DECLARED_COHORT_LANES".to_owned(),
        format!("unknown_cohort_count={}", unknown_lanes.len()),
        format!("unknown_cohort_seconds_total={unknown_seconds_total}"),
        format!(
            "known_only_service_deficit_fraction_seconds_p50={}",
            display_float(percentile_nearest_rank(&deficits, 0.50))
        ),
        format!(
            "known_only_service_deficit_fraction_seconds_p90={}",
            display_float(percentile_nearest_rank(&deficits, 0.90))
        ),
        format!(
            "known_only_service_deficit_fraction_seconds_max={}",
            display_float(worst_deficit.map(|lane| lane.trajectory.service_deficit_fraction_seconds))
        ),
        format!(
            "known_only_service_deficit_worst_cohort={}",
            worst_deficit.map(|lane| lane.cohort_id.as_str()).unwrap_or("NONE")
        ),
        format!(
            "known_only_service_deficit_gap={:.6}",
            if deficits.is_empty() { 0.0 } else { max_deficit - min_deficit }
        ),
        format!(
            "known_only_quality_noncompliant_seconds_p90={}",
            display_float(percentile_nearest_rank(&quality_seconds, 0.90))
        ),
        format!(
            "max_quality_noncompliant_seconds={}",
            worst_quality
                .map(|lane| lane.trajectory.quality_noncompliant_seconds.to_string())
                .unwrap_or_else(|| "NOT_ESTIMABLE".to_owned())
        ),
        format!(
            "max_quality_noncompliant_cohort={}",
            worst_quality.map(|lane| lane.cohort_id.as_str()).unwrap_or("NONE")
        ),
        format!(
            "max_longest_unsafe_episode_seconds={}",
            worst_unsafe_episode
                .map(|lane| lane.trajectory.longest_unsafe_episode_seconds.to_string())
                .unwrap_or_else(|| "NOT_ESTIMABLE".to_owned())
        ),
        format!(
            "max_longest_unsafe_episode_cohort={}",
            worst_unsafe_episode.map(|lane| lane.cohort_id.as_str()).unwrap_or("NONE")
        ),
        format!(
            "max_below_minimum_service_seconds={}",
            max_below_floor
                .map(|lane| lane.trajectory.below_minimum_service_seconds.to_string())
                .unwrap_or_else(|| "NOT_ESTIMABLE".to_owned())
        ),
        format!(
            "max_below_minimum_service_cohort={}",
            max_below_floor.map(|lane| lane.cohort_id.as_str()).unwrap_or("NONE")
        ),
        format!("cohorts_with_known_breach={breached_count}"),
        format!("cohorts_with_safe_recovery_observed={recovered_count}"),
        format!("breached_cohorts_without_safe_recovery={unrecovered_breach_count}"),
        format!(
            "known_only_time_to_safe_recovery_seconds_p50={}",
            display_float(percentile_nearest_rank(&safe_recovery_times, 0.50))
        ),
        format!(
            "known_only_time_to_safe_recovery_seconds_p90={}",
            display_float(percentile_nearest_rank(&safe_recovery_times, 0.90))
        ),
    ];

    for lane in lanes {
        let report = lane.trajectory;
        lines.push(format!(
            "cohort={}|{}|{}|{:.6}|{}|{}|{:.6}|{}|{}|{}|{}|{}|{}|{}",
            lane.cohort_id,
            display_denominator(lane.population_denominator),
            lane.population_unit,
            lane.minimum_safe_service_fraction,
            report.known_seconds,
            report.unknown_seconds,
            report.service_deficit_fraction_seconds,
            report.unsafe_service_seconds,
            report.quality_noncompliant_seconds,
            report.below_minimum_service_seconds,
            report.longest_unsafe_episode_seconds,
            display_u64(report.first_known_breach_at_seconds),
            display_u64(report.time_to_safe_recovery_seconds),
            if report.first_known_breach_at_seconds.is_none() {
                "NO_KNOWN_BREACH"
            } else if report.safe_recovery_at_seconds.is_some() {
                "SAFE_RECOVERY_OBSERVED"
            } else {
                "SAFE_RECOVERY_NOT_OBSERVED"
            }
        ));
    }

    lines.extend([
        "unknown_time_is_not_treated_as_safe_or_failed=TRUE".to_owned(),
        "deficit_and_percentiles_use_known_observation_time_only=TRUE".to_owned(),
        "service_quality_is_reported_separately_from_quantity_deficit=TRUE".to_owned(),
        "cohort_weights_or_social_priorities_are_not_inferred=TRUE".to_owned(),
        "source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned(),
        "observed_service_truth=NOT_INDEPENDENTLY_AUTHENTICATED_BY_THIS_TOOL".to_owned(),
        "population_coverage=NOT_ESTABLISHED".to_owned(),
        "operational_authority=NONE".to_owned(),
        "operational_actuation=NOT_AUTHORISED".to_owned(),
    ]);
    Ok(lines.join("\n") + "\n")
}

/// Compose several cohort-specific service trajectory inputs using the existing
/// fsre-service-trajectory evaluator. Unknown intervals remain visible and prevent
/// an estimable-for-declared-lanes disposition.
pub fn validate_and_report(lane_sources: &[String]) -> Result<String, InputError> {
    compose(lane_sources)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOSPITAL: &str = include_str!("../examples/hospital.csv");
    const HOUSEHOLDS: &str = include_str!("../examples/households.csv");
    const REMOTE: &str = include_str!("../examples/remote-community.csv");
    const EXPECTED: &str = include_str!("../fixtures/envelope.expected.txt");

    fn run(sources: &[&str]) -> String {
        let owned: Vec<String> = sources.iter().map(|source| (*source).to_owned()).collect();
        validate_and_report(&owned).expect("envelope")
    }

    #[test]
    fn three_cohort_envelope_matches_golden_report() {
        assert_eq!(run(&[HOSPITAL, HOUSEHOLDS, REMOTE]), EXPECTED);
    }

    #[test]
    fn the_worst_cohort_is_not_hidden_by_better_cohort_outcomes() {
        let report = run(&[HOSPITAL, HOUSEHOLDS, REMOTE]);
        assert!(report.contains("known_only_service_deficit_fraction_seconds_max=28.000000"));
        assert!(report.contains("known_only_service_deficit_worst_cohort=households"));
        assert!(report.contains("max_longest_unsafe_episode_cohort=remote-community"));
    }

    #[test]
    fn quality_noncompliance_is_not_hidden_by_full_service_quantity() {
        let report = run(&[HOSPITAL, HOUSEHOLDS, REMOTE]);
        assert!(report.contains("max_quality_noncompliant_seconds=30"));
        assert!(report.contains("max_quality_noncompliant_cohort=hospital"));
        assert!(report.contains("cohort=hospital|500|persons|0.500000|100|0|0.000000|30|30|0|30|20|30|SAFE_RECOVERY_OBSERVED"));
    }

    #[test]
    fn unknown_observation_time_prevents_estimable_envelope_disposition() {
        let changed = HOUSEHOLDS.replace("40,0.6,true", "90,0.6,true");
        let report = run(&[HOSPITAL, &changed, REMOTE]);
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("unknown_cohort_count=1"));
        assert!(report.contains("unknown_time_is_not_treated_as_safe_or_failed=TRUE"));
    }

    #[test]
    fn mismatched_horizons_are_rejected() {
        let changed = HOUSEHOLDS.replace("# horizon_end_seconds=100", "# horizon_end_seconds=101");
        let owned = vec![HOSPITAL.to_owned(), changed, REMOTE.to_owned()];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("all lanes must share horizon"));
    }

    #[test]
    fn duplicate_cohort_ids_are_rejected() {
        let changed = REMOTE.replace("# cohort_id=remote-community", "# cohort_id=households");
        let owned = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), changed];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("duplicate cohort_id households"));
    }

    #[test]
    fn row_order_does_not_change_the_envelope() {
        assert_eq!(
            run(&[HOSPITAL, HOUSEHOLDS, REMOTE]),
            run(&[REMOTE, HOSPITAL, HOUSEHOLDS])
        );
    }

    #[test]
    fn unknown_population_denominator_is_preserved_without_weighting() {
        let changed = REMOTE.replace("# population_denominator=250", "# population_denominator=UNKNOWN");
        let report = run(&[HOSPITAL, HOUSEHOLDS, &changed]);
        assert!(report.contains("population_weighted_aggregation=NOT_COMPUTED"));
        assert!(report.contains("cohort=remote-community|UNKNOWN|persons|"));
    }

    #[test]
    fn denominator_units_must_match_before_cross_cohort_comparison() {
        let changed = REMOTE.replace("# population_unit=persons", "# population_unit=connections");
        let owned = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), changed];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("same population_denominator unit"));
    }
}
