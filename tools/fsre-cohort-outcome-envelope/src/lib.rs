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
    outcome_snapshot_id: String,
    delivery_source_owner: String,
    delivery_source_ref: String,
    quality_source_owner: String,
    quality_source_ref: String,
    denominator_source_owner: String,
    denominator_source_ref: String,
    identity_source_owner: String,
    identity_source_ref: String,
    population_denominator: Option<u64>,
    population_unit: String,
    contract: ServiceContract,
    observations: Vec<Observation>,
}

#[derive(Clone, Debug)]
struct EvaluatedLane {
    cohort_id: String,
    outcome_snapshot_id: String,
    identity_source_owner: String,
    identity_source_ref: String,
    delivery_source_owner: String,
    delivery_source_ref: String,
    quality_source_owner: String,
    quality_source_ref: String,
    denominator_source_owner: String,
    denominator_source_ref: String,
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
    let outcome_snapshot_id = required(&metadata, "outcome_snapshot_id")?.to_owned();
    let delivery_source_owner = required(&metadata, "delivery_source_owner")?.to_owned();
    let delivery_source_ref = required(&metadata, "delivery_source_ref")?.to_owned();
    let quality_source_owner = required(&metadata, "quality_source_owner")?.to_owned();
    let quality_source_ref = required(&metadata, "quality_source_ref")?.to_owned();
    let denominator_source_owner = required(&metadata, "denominator_source_owner")?.to_owned();
    let denominator_source_ref = required(&metadata, "denominator_source_ref")?.to_owned();
    let identity_source_owner = required(&metadata, "identity_source_owner")?.to_owned();
    let identity_source_ref = required(&metadata, "identity_source_ref")?.to_owned();
    let population_value = required(&metadata, "population_denominator")?;
    let population_unit = required(&metadata, "population_unit")?.to_owned();

    for (key, value) in [
        ("service_id", service_id.as_str()),
        ("service_family_id", service_family_id.as_str()),
        ("cohort_set_id", cohort_set_id.as_str()),
        ("cohort_id", cohort_id.as_str()),
        ("outcome_snapshot_id", outcome_snapshot_id.as_str()),
        ("delivery_source_owner", delivery_source_owner.as_str()),
        ("delivery_source_ref", delivery_source_ref.as_str()),
        ("quality_source_owner", quality_source_owner.as_str()),
        ("quality_source_ref", quality_source_ref.as_str()),
        ("denominator_source_owner", denominator_source_owner.as_str()),
        ("denominator_source_ref", denominator_source_ref.as_str()),
        ("identity_source_owner", identity_source_owner.as_str()),
        ("identity_source_ref", identity_source_ref.as_str()),
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

    if let Some(unknown_key) = metadata.keys().find(|key| {
        !matches!(
            key.as_str(),
            "service_id"
                | "service_family_id"
                | "cohort_set_id"
                | "cohort_id"
                | "outcome_snapshot_id"
                | "delivery_source_owner"
                | "delivery_source_ref"
                | "quality_source_owner"
                | "quality_source_ref"
                | "denominator_source_owner"
                | "denominator_source_ref"
                | "identity_source_owner"
                | "identity_source_ref"
                | "population_denominator"
                | "population_unit"
                | "horizon_start_seconds"
                | "horizon_end_seconds"
                | "minimum_safe_service_fraction"
                | "max_observation_hold_seconds"
                | "recovery_stability_seconds"
                | "quality_required"
        )
    }) {
        return Err(InputError(format!("unknown metadata key {unknown_key}")));
    }

    Ok(CohortLane {
        service_family_id,
        cohort_set_id,
        cohort_id,
        outcome_snapshot_id,
        delivery_source_owner,
        delivery_source_ref,
        quality_source_owner,
        quality_source_ref,
        denominator_source_owner,
        denominator_source_ref,
        identity_source_owner,
        identity_source_ref,
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


const MANIFEST_HEADER: &str = "record_type|cohort_id|population_denominator|population_unit|identity_source_owner|identity_source_ref|denominator_source_owner|denominator_source_ref|valid_from_seconds|valid_until_seconds";

#[derive(Clone, Debug)]
struct ManifestCohort {
    population_denominator: Option<u64>,
    population_unit: String,
    identity_source_owner: String,
    identity_source_ref: String,
    denominator_source_owner: String,
    denominator_source_ref: String,
    valid_from_seconds: u64,
    valid_until_seconds: u64,
}

#[derive(Clone, Debug)]
struct CohortManifest {
    manifest_id: String,
    manifest_snapshot_id: String,
    manifest_owner: String,
    manifest_ref: String,
    service_id: String,
    service_family_id: String,
    cohort_set_id: String,
    outcome_snapshot_id: String,
    population_unit: String,
    cohorts: BTreeMap<String, ManifestCohort>,
}

fn parse_manifest(source: &str) -> Result<CohortManifest, InputError> {
    let mut metadata = BTreeMap::<String, String>::new();
    let mut cohorts = BTreeMap::<String, ManifestCohort>::new();
    let mut header_seen = false;

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
                        "manifest line {line_number}: duplicate metadata key {key}"
                    )));
                }
            }
            continue;
        }
        if !header_seen {
            if line != MANIFEST_HEADER {
                return Err(InputError(format!(
                    "manifest line {line_number}: unexpected cohort manifest header"
                )));
            }
            header_seen = true;
            continue;
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if fields.len() != 10 || fields[0] != "COHORT" {
            return Err(InputError(format!(
                "manifest line {line_number}: expected COHORT row with 9 values"
            )));
        }
        let cohort_id = fields[1].to_owned();
        if !valid_id(&cohort_id) {
            return Err(InputError(format!("manifest line {line_number}: invalid cohort_id")));
        }
        let population_denominator = if fields[2] == "UNKNOWN" {
            None
        } else {
            let value = parse_u64(fields[2], "manifest population_denominator")?;
            if value == 0 {
                return Err(InputError(format!(
                    "manifest line {line_number}: population_denominator must be positive or UNKNOWN"
                )));
            }
            Some(value)
        };
        let population_unit = fields[3].to_owned();
        let identity_source_owner = fields[4].to_owned();
        let identity_source_ref = fields[5].to_owned();
        let denominator_source_owner = fields[6].to_owned();
        let denominator_source_ref = fields[7].to_owned();
        for (key, value) in [
            ("population_unit", population_unit.as_str()),
            ("identity_source_owner", identity_source_owner.as_str()),
            ("identity_source_ref", identity_source_ref.as_str()),
            ("denominator_source_owner", denominator_source_owner.as_str()),
            ("denominator_source_ref", denominator_source_ref.as_str()),
        ] {
            if !valid_id(value) {
                return Err(InputError(format!(
                    "manifest line {line_number}: invalid {key}"
                )));
            }
        }
        let valid_from_seconds = parse_u64(fields[8], "manifest valid_from_seconds")?;
        let valid_until_seconds = parse_u64(fields[9], "manifest valid_until_seconds")?;
        if valid_from_seconds >= valid_until_seconds {
            return Err(InputError(format!(
                "manifest line {line_number}: cohort membership validity interval is empty"
            )));
        }
        let cohort = ManifestCohort {
            population_denominator,
            population_unit,
            identity_source_owner,
            identity_source_ref,
            denominator_source_owner,
            denominator_source_ref,
            valid_from_seconds,
            valid_until_seconds,
        };
        if cohorts.insert(cohort_id.clone(), cohort).is_some() {
            return Err(InputError(format!(
                "manifest line {line_number}: duplicate cohort_id {cohort_id}"
            )));
        }
    }

    if !header_seen || cohorts.is_empty() {
        return Err(InputError(
            "cohort manifest requires a header and at least one COHORT row".to_owned(),
        ));
    }

    let manifest_id = required(&metadata, "manifest_id")?.to_owned();
    let manifest_snapshot_id = required(&metadata, "manifest_snapshot_id")?.to_owned();
    let manifest_owner = required(&metadata, "manifest_owner")?.to_owned();
    let manifest_ref = required(&metadata, "manifest_ref")?.to_owned();
    let service_id = required(&metadata, "service_id")?.to_owned();
    let service_family_id = required(&metadata, "service_family_id")?.to_owned();
    let cohort_set_id = required(&metadata, "cohort_set_id")?.to_owned();
    let outcome_snapshot_id = required(&metadata, "outcome_snapshot_id")?.to_owned();
    let population_unit = required(&metadata, "population_unit")?.to_owned();

    for (key, value) in [
        ("manifest_id", manifest_id.as_str()),
        ("manifest_snapshot_id", manifest_snapshot_id.as_str()),
        ("manifest_owner", manifest_owner.as_str()),
        ("manifest_ref", manifest_ref.as_str()),
        ("service_id", service_id.as_str()),
        ("service_family_id", service_family_id.as_str()),
        ("cohort_set_id", cohort_set_id.as_str()),
        ("outcome_snapshot_id", outcome_snapshot_id.as_str()),
        ("population_unit", population_unit.as_str()),
    ] {
        if !valid_id(value) {
            return Err(InputError(format!("invalid manifest metadata {key}")));
        }
    }
    if cohorts
        .values()
        .any(|cohort| cohort.population_unit != population_unit)
    {
        return Err(InputError(
            "manifest cohort population units do not match manifest population_unit".to_owned(),
        ));
    }
    if let Some(key) = metadata.keys().find(|key| {
        !matches!(
            key.as_str(),
            "manifest_id"
                | "manifest_snapshot_id"
                | "manifest_owner"
                | "manifest_ref"
                | "service_id"
                | "service_family_id"
                | "cohort_set_id"
                | "outcome_snapshot_id"
                | "population_unit"
        )
    }) {
        return Err(InputError(format!("unknown cohort manifest metadata key {key}")));
    }

    Ok(CohortManifest {
        manifest_id,
        manifest_snapshot_id,
        manifest_owner,
        manifest_ref,
        service_id,
        service_family_id,
        cohort_set_id,
        outcome_snapshot_id,
        population_unit,
        cohorts,
    })
}

fn qualify_manifest(
    manifest_source: &str,
    lane_sources: &[String],
    report: &str,
) -> Result<String, InputError> {
    let manifest = parse_manifest(manifest_source)?;
    let lanes: Vec<CohortLane> = lane_sources
        .iter()
        .map(|source| parse_lane(source))
        .collect::<Result<_, _>>()?;
    if lanes.is_empty() {
        return Err(InputError("at least one cohort lane is required".to_owned()));
    }

    let first = &lanes[0];
    let mut received = BTreeMap::<String, &CohortLane>::new();
    for lane in &lanes {
        received.insert(lane.cohort_id.clone(), lane);
    }

    let scope_match = manifest.service_id == first.contract.service_id
        && manifest.service_family_id == first.service_family_id
        && manifest.cohort_set_id == first.cohort_set_id
        && manifest.outcome_snapshot_id == first.outcome_snapshot_id
        && manifest.population_unit == first.population_unit;

    let expected_ids: BTreeSet<String> = manifest.cohorts.keys().cloned().collect();
    let received_ids: BTreeSet<String> = received.keys().cloned().collect();
    let missing: Vec<String> = expected_ids.difference(&received_ids).cloned().collect();
    let unexpected: Vec<String> = received_ids.difference(&expected_ids).cloned().collect();

    let mut mismatch_ids = Vec::new();
    for cohort_id in expected_ids.intersection(&received_ids) {
        let expected = manifest.cohorts.get(cohort_id).expect("set membership checked");
        let actual = received.get(cohort_id).expect("set membership checked");
        let valid_for_horizon = expected.valid_from_seconds <= first.contract.horizon_start_seconds
            && expected.valid_until_seconds >= first.contract.horizon_end_seconds;
        let fields_match = expected.population_denominator == actual.population_denominator
            && expected.population_unit == actual.population_unit
            && expected.denominator_source_owner == actual.denominator_source_owner
            && expected.denominator_source_ref == actual.denominator_source_ref
            && expected.identity_source_owner == actual.identity_source_owner
            && expected.identity_source_ref == actual.identity_source_ref
            && valid_for_horizon;
        if !fields_match {
            mismatch_ids.push(cohort_id.clone());
        }
    }

    let complete = scope_match && missing.is_empty() && unexpected.is_empty() && mismatch_ids.is_empty();
    let lane_disposition = report
        .lines()
        .find_map(|line| line.strip_prefix("cohort_outcome_disposition="))
        .unwrap_or("NOT_ESTIMABLE");
    let final_disposition = if !complete || lane_disposition != "ESTIMABLE_FOR_DECLARED_LANES" {
        "NOT_ESTIMABLE"
    } else {
        "ESTIMABLE_FOR_DECLARED_MANIFEST"
    };

    let mut lines: Vec<String> = report.lines().map(str::to_owned).collect();
    for line in &mut lines {
        if line == "cohort_set_completeness=NOT_ESTABLISHED_BY_THIS_TOOL" {
            *line = if complete {
                "cohort_set_completeness=COMPLETE_FOR_DECLARED_MANIFEST".to_owned()
            } else {
                "cohort_set_completeness=INCOMPLETE_OR_MISMATCHED_TO_DECLARED_MANIFEST".to_owned()
            };
        } else if line == "cohort_outcome_disposition=ESTIMABLE_FOR_DECLARED_LANES"
            || line == "cohort_outcome_disposition=NOT_ESTIMABLE"
        {
            *line = format!("cohort_outcome_disposition={final_disposition}");
        }
    }
    lines.extend([
        format!("manifest_id={}", manifest.manifest_id),
        format!("manifest_snapshot_id={}", manifest.manifest_snapshot_id),
        format!("manifest_owner={}", manifest.manifest_owner),
        format!("manifest_ref={}", manifest.manifest_ref),
        format!("manifest_expected_cohort_count={}", manifest.cohorts.len()),
        format!("manifest_received_cohort_count={}", lanes.len()),
        format!("manifest_scope_match={scope_match}"),
        format!("manifest_disposition={}", if complete {
            "COMPLETE_FOR_DECLARED_MANIFEST"
        } else {
            "INCOMPLETE_OR_MISMATCHED"
        }),
        format!("manifest_missing_cohort_ids={}", if missing.is_empty() {
            "NONE".to_owned()
        } else {
            missing.join(",")
        }),
        format!("manifest_unexpected_cohort_ids={}", if unexpected.is_empty() {
            "NONE".to_owned()
        } else {
            unexpected.join(",")
        }),
        format!("manifest_mismatched_cohort_ids={}", if mismatch_ids.is_empty() {
            "NONE".to_owned()
        } else {
            mismatch_ids.join(",")
        }),
        "manifest_completeness_scope=DECLARED_MANIFEST_ONLY_NOT_REAL_POPULATION_AUTHENTICATION".to_owned(),
        "manifest_source_authenticity=NOT_AUTHENTICATED_BY_THIS_TOOL".to_owned(),
    ]);
    Ok(lines.join("\n") + "\n")
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
            || lane.outcome_snapshot_id != baseline.outcome_snapshot_id
        {
            return Err(InputError(
                "all lanes must share service_family_id, cohort_set_id, service_id and outcome_snapshot_id".to_owned(),
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
            outcome_snapshot_id: lane.outcome_snapshot_id,
            identity_source_owner: lane.identity_source_owner,
            identity_source_ref: lane.identity_source_ref,
            delivery_source_owner: lane.delivery_source_owner,
            delivery_source_ref: lane.delivery_source_ref,
            quality_source_owner: lane.quality_source_owner,
            quality_source_ref: lane.quality_source_ref,
            denominator_source_owner: lane.denominator_source_owner,
            denominator_source_ref: lane.denominator_source_ref,
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
        format!("outcome_snapshot_id={}", first_source.outcome_snapshot_id),
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
            "cohort={}|{}|{}|{:.6}|{}|{}|{}|{}|{:.6}|{}|{}|{}|{}|{}|{}|{}",
            lane.cohort_id,
            display_denominator(lane.population_denominator),
            lane.population_unit,
            lane.minimum_safe_service_fraction,
            report.known_seconds,
            report.unknown_seconds,
            report.full_service_seconds,
            report.safe_degraded_seconds,
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
        lines.push(format!(
            "cohort_evidence={}|{}|{}|{}|{}|{}|{}|{}|{}",
            lane.cohort_id,
            lane.identity_source_owner,
            lane.identity_source_ref,
            lane.delivery_source_owner,
            lane.delivery_source_ref,
            lane.quality_source_owner,
            lane.quality_source_ref,
            lane.denominator_source_owner,
            lane.denominator_source_ref
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

/// Validate the submitted cohort lanes against an owner-declared, frozen cohort manifest.
/// A complete result is only complete relative to the manifest; the manifest itself is not authenticated.
pub fn validate_and_report_with_manifest(
    manifest_source: &str,
    lane_sources: &[String],
) -> Result<String, InputError> {
    let report = compose(lane_sources)?;
    qualify_manifest(manifest_source, lane_sources, &report)
}

#[cfg(test)]
mod tests {
    use super::*;

    const HOSPITAL: &str = include_str!("../examples/hospital.csv");
    const HOUSEHOLDS: &str = include_str!("../examples/households.csv");
    const REMOTE: &str = include_str!("../examples/remote-community.csv");
    const EXPECTED: &str = include_str!("../fixtures/envelope.expected.txt");
    const MANIFEST: &str = include_str!("../fixtures/cohort-manifest.psv");
    const MANIFEST_EXPECTED: &str = include_str!("../fixtures/envelope.manifest.expected.txt");

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
        assert!(report.contains("cohort=hospital|500|persons|0.500000|100|0|70|0|0.000000|30|30|0|30|20|30|SAFE_RECOVERY_OBSERVED"));
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
    fn outcome_snapshot_must_match_across_cohort_lanes() {
        let changed = HOUSEHOLDS.replace(
            "# outcome_snapshot_id=synthetic-outcome-snapshot-01",
            "# outcome_snapshot_id=other-outcome-snapshot",
        );
        let owned = vec![HOSPITAL.to_owned(), changed, REMOTE.to_owned()];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("outcome_snapshot_id"));
    }

    #[test]
    fn missing_quality_source_owner_is_rejected() {
        let changed = HOSPITAL.replace(
            "# quality_source_owner=laboratory",
            "# quality_source_owner=",
        );
        let owned = vec![changed, HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("empty metadata key quality_source_owner"));
    }

    #[test]
    fn manifest_complete_cohort_set_matches_golden_report() {
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        assert_eq!(validate_and_report_with_manifest(MANIFEST, &lanes).unwrap(), MANIFEST_EXPECTED);
    }

    #[test]
    fn manifest_missing_lane_prevents_completeness_claim() {
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned()];
        let report = validate_and_report_with_manifest(MANIFEST, &lanes).unwrap();
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
        assert!(report.contains("cohort_set_completeness=INCOMPLETE_OR_MISMATCHED_TO_DECLARED_MANIFEST"));
        assert!(report.contains("manifest_missing_cohort_ids=remote-community"));
    }

    #[test]
    fn unexpected_lane_prevents_completeness_claim() {
        let extra = REMOTE.replace("# cohort_id=remote-community", "# cohort_id=other-community");
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), extra];
        let report = validate_and_report_with_manifest(MANIFEST, &lanes).unwrap();
        assert!(report.contains("manifest_unexpected_cohort_ids=other-community"));
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn manifest_identity_reference_mismatch_is_not_accepted() {
        let changed = MANIFEST.replace(
            "COHORT|households|1000|persons|cohort-register|synthetic:cohort:households",
            "COHORT|households|1000|persons|cohort-register|synthetic:cohort:other-households",
        );
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        let report = validate_and_report_with_manifest(&changed, &lanes).unwrap();
        assert!(report.contains("manifest_mismatched_cohort_ids=households"));
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn manifest_scope_mismatch_is_not_accepted() {
        let changed = MANIFEST.replace(
            "# service_family_id=potable-water",
            "# service_family_id=other-service",
        );
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        let report = validate_and_report_with_manifest(&changed, &lanes).unwrap();
        assert!(report.contains("manifest_scope_match=false"));
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn manifest_denominator_mismatch_is_not_accepted() {
        let changed = MANIFEST.replace("COHORT|households|1000|", "COHORT|households|999|");
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        let report = validate_and_report_with_manifest(&changed, &lanes).unwrap();
        assert!(report.contains("manifest_mismatched_cohort_ids=households"));
        assert!(report.contains("cohort_outcome_disposition=NOT_ESTIMABLE"));
    }

    #[test]
    fn manifest_membership_must_cover_the_complete_horizon() {
        let changed = MANIFEST.replace(
            "COHORT|remote-community|250|persons|",
            "COHORT|remote-community|250|persons|",
        ).replace(
            "|0|1000",
            "|0|99",
        );
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        let report = validate_and_report_with_manifest(&changed, &lanes).unwrap();
        assert!(report.contains("manifest_mismatched_cohort_ids=hospital,households,remote-community"));
    }

    #[test]
    fn manifest_row_order_does_not_change_report() {
        let changed = MANIFEST.lines().collect::<Vec<_>>();
        let header = changed.iter().position(|line| line.starts_with("record_type|")).unwrap();
        let metadata = changed[..header].to_vec();
        let mut rows = changed[header + 1..].to_vec();
        rows.reverse();
        let reordered = format!("{}\n{}\n{}\n", metadata.join("\n"), MANIFEST_HEADER, rows.join("\n"));
        let lanes = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), REMOTE.to_owned()];
        assert_eq!(
            validate_and_report_with_manifest(MANIFEST, &lanes).unwrap(),
            validate_and_report_with_manifest(&reordered, &lanes).unwrap()
        );
    }

    #[test]
    fn denominator_units_must_match_before_cross_cohort_comparison() {
        let changed = REMOTE.replace("# population_unit=persons", "# population_unit=connections");
        let owned = vec![HOSPITAL.to_owned(), HOUSEHOLDS.to_owned(), changed];
        let error = validate_and_report(&owned).unwrap_err();
        assert!(error.0.contains("same population_denominator unit"));
    }
}
