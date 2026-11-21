//! Synthetic-first event-aware seasonal GHG coverage and integration auditor.
//!
//! This tool refuses to label an integration as a qualified seasonal total.
//! It reports a partial-support trapezoidal integral and independent coverage
//! diagnostics. Measurement covariance / uncertainty propagation is not
//! implemented in v0.1, so complete coverage remains unqualified.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::env;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::Path;

const AUDIT_VERSION: &str = "0.1.0";
const FIXTURE_STATUS: &str = "synthetic_only_not_empirical_evidence";
const INTEGRAL_BOUNDARY: &str = "Partial-support trapezoidal integral only. No seasonal or annual emissions total is qualified; uncertainty/covariance propagation, censoring models and external field validation are not implemented.";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FixtureEnvelope {
    fixture_status: String,
    fixture_version: String,
    campaign: Campaign,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Campaign {
    campaign_id: String,
    study_id: String,
    site_id: String,
    experimental_unit_id: String,
    treatment_assignment_id: String,
    gas_species: String,
    mass_basis: String,
    flux_unit: String,
    window_start: String,
    window_end: String,
    maximum_gap_seconds: f64,
    boundary_tolerance_seconds: f64,
    schedule_slots: Vec<ScheduleSlot>,
    events: Vec<EventWindow>,
    observations: Vec<Observation>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScheduleSlot {
    slot_id: String,
    scheduled_at: String,
    event_id: Option<String>,
    event_relation: String,
    missing_reason: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct EventWindow {
    event_id: String,
    event_type: String,
    window_start: String,
    occurred_at: String,
    window_end: String,
    minimum_before_observations: usize,
    minimum_after_observations: usize,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Observation {
    observation_id: String,
    scheduled_slot_id: String,
    observed_at: String,
    gas_species: String,
    mass_basis: String,
    flux_value: Option<f64>,
    flux_unit: String,
    quantification_status: String,
    detection_limit: Option<f64>,
    qc_state: String,
    standard_uncertainty: Option<f64>,
    uncertainty_group_id: Option<String>,
}

#[derive(Debug, Serialize)]
struct AuditReport {
    audit_version: &'static str,
    status: &'static str,
    fixture_status: &'static str,
    fixture_version: String,
    campaign_id: String,
    gas_species: String,
    mass_basis: String,
    flux_unit: String,
    window_start: String,
    window_end: String,
    input_sha256: String,
    schedule_slot_count: usize,
    usable_observation_count: usize,
    missing_schedule_slot_ids: Vec<String>,
    unusable_schedule_slot_ids: Vec<String>,
    schedule_coverage_fraction: f64,
    boundary_coverage_complete: bool,
    maximum_observed_gap_seconds: Option<f64>,
    maximum_schedule_gap_seconds: Option<f64>,
    excessive_observation_gap_count: usize,
    unintegrated_gap_count: usize,
    event_coverages: Vec<EventCoverage>,
    event_coverage_complete: bool,
    integrated_partial_mg_m2: Option<f64>,
    integrated_support_seconds: f64,
    uncovered_seconds: f64,
    integration_support_fraction: f64,
    censored_observation_count: usize,
    flagged_or_rejected_observation_count: usize,
    uncertainty_status: &'static str,
    uncertainty_group_count: usize,
    qualification_reasons: Vec<String>,
    interpretation_boundary: &'static str,
}

#[derive(Debug, Serialize)]
struct EventCoverage {
    event_id: String,
    event_type: String,
    observed_before: usize,
    required_before: usize,
    observed_after: usize,
    required_after: usize,
    qualified: bool,
}

#[derive(Clone, Copy)]
struct Point {
    scheduled_at: f64,
    flux: f64,
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write as _;
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn nonblank(value: &str, name: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.trim() != value {
        return Err(format!(
            "{name} must be non-empty and have no surrounding whitespace"
        ));
    }
    Ok(())
}

fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    let year = year - if month <= 2 { 1 } else { 0 };
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month = i64::from(month);
    let day = i64::from(day);
    let day_of_year = (153 * (month + if month > 2 { -3 } else { 9 }) + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

/// Strict UTC RFC3339 seconds: YYYY-MM-DDTHH:MM:SSZ (no offsets/fractions in v0.1).
fn parse_utc_seconds(value: &str) -> Result<f64, String> {
    let bytes = value.as_bytes();
    if bytes.len() != 20
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || bytes[10] != b'T'
        || bytes[13] != b':'
        || bytes[16] != b':'
        || bytes[19] != b'Z'
        || !value.is_ascii()
    {
        return Err(format!(
            "timestamp must be UTC RFC3339 YYYY-MM-DDTHH:MM:SSZ: {value}"
        ));
    }
    let part = |start: usize, end: usize, label: &str| -> Result<u32, String> {
        value[start..end]
            .parse::<u32>()
            .map_err(|_| format!("invalid {label} in timestamp {value}"))
    };
    let year = part(0, 4, "year")?;
    let month = part(5, 7, "month")?;
    let day = part(8, 10, "day")?;
    let hour = part(11, 13, "hour")?;
    let minute = part(14, 16, "minute")?;
    let second = part(17, 19, "second")?;
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return Err(format!("invalid month in timestamp {value}")),
    };
    if day == 0 || day > days_in_month || hour > 23 || minute > 59 || second > 59 {
        return Err(format!("invalid calendar time in timestamp {value}"));
    }
    let days = days_from_civil(i64::from(year), month, day);
    Ok(
        (days * 86_400 + i64::from(hour) * 3_600 + i64::from(minute) * 60 + i64::from(second))
            as f64,
    )
}

fn expected_flux_unit(gas: &str, basis: &str) -> Result<&'static str, String> {
    match (gas, basis) {
        ("N2O", "N2O") => Ok("mg_N2O_m2_h"),
        ("N2O", "N2O_N") => Ok("mg_N2O_N_m2_h"),
        ("CH4", "CH4") => Ok("mg_CH4_m2_h"),
        ("CO2", "CO2") => Ok("mg_CO2_m2_h"),
        _ => Err(format!(
            "unsupported gas species / mass basis: {gas} / {basis}"
        )),
    }
}

fn is_usable(observation: &Observation) -> bool {
    observation.qc_state == "pass"
        && observation.quantification_status == "quantified"
        && observation.flux_value.is_some()
}

fn audit_bytes(bytes: &[u8]) -> Result<AuditReport, String> {
    let envelope: FixtureEnvelope =
        serde_json::from_slice(bytes).map_err(|error| format!("invalid fixture JSON: {error}"))?;
    if envelope.fixture_status != FIXTURE_STATUS {
        return Err("v0.1 accepts synthetic-only campaign envelopes".to_string());
    }
    nonblank(&envelope.fixture_version, "fixture_version")?;
    let campaign = envelope.campaign;

    for (value, name) in [
        (&campaign.campaign_id, "campaign_id"),
        (&campaign.study_id, "study_id"),
        (&campaign.site_id, "site_id"),
        (&campaign.experimental_unit_id, "experimental_unit_id"),
        (&campaign.treatment_assignment_id, "treatment_assignment_id"),
    ] {
        nonblank(value, name)?;
    }

    let expected_unit = expected_flux_unit(&campaign.gas_species, &campaign.mass_basis)?;
    if campaign.flux_unit != expected_unit {
        return Err(format!("campaign flux unit must be {expected_unit}"));
    }
    if !campaign.maximum_gap_seconds.is_finite() || campaign.maximum_gap_seconds <= 0.0 {
        return Err("maximum_gap_seconds must be finite and positive".to_string());
    }
    if !campaign.boundary_tolerance_seconds.is_finite() || campaign.boundary_tolerance_seconds < 0.0
    {
        return Err("boundary_tolerance_seconds must be finite and non-negative".to_string());
    }

    let window_start = parse_utc_seconds(&campaign.window_start)?;
    let window_end = parse_utc_seconds(&campaign.window_end)?;
    if window_start >= window_end {
        return Err("integration window must have positive duration".to_string());
    }
    if campaign.schedule_slots.len() < 2 {
        return Err("at least two expected schedule slots are required".to_string());
    }
    if campaign.events.is_empty() {
        return Err("campaign must declare at least one relevant event window".to_string());
    }

    let mut events = HashMap::new();
    for event in &campaign.events {
        nonblank(&event.event_id, "event.event_id")?;
        nonblank(&event.event_type, "event.event_type")?;
        if !matches!(
            event.event_type.as_str(),
            "rainfall"
                | "irrigation"
                | "fertilizer"
                | "manure"
                | "tillage"
                | "wetting_drying"
                | "other"
        ) {
            return Err(format!(
                "event {} has unsupported event type",
                event.event_id
            ));
        }
        let start = parse_utc_seconds(&event.window_start)?;
        let occurred = parse_utc_seconds(&event.occurred_at)?;
        let end = parse_utc_seconds(&event.window_end)?;
        if !(window_start <= start && start <= occurred && occurred <= end && end <= window_end) {
            return Err(format!(
                "event {} has invalid / out-of-campaign window",
                event.event_id
            ));
        }
        if events.insert(event.event_id.as_str(), event).is_some() {
            return Err(format!("duplicate event_id: {}", event.event_id));
        }
    }

    let mut slot_map = HashMap::new();
    let mut slot_times = Vec::new();
    let mut previous_time = None;
    for slot in &campaign.schedule_slots {
        nonblank(&slot.slot_id, "schedule_slot.slot_id")?;
        let time = parse_utc_seconds(&slot.scheduled_at)?;
        if time < window_start || time > window_end {
            return Err(format!(
                "slot {} is outside integration window",
                slot.slot_id
            ));
        }
        if previous_time.is_some_and(|previous| time <= previous) {
            return Err("schedule slots must be strictly chronological".to_string());
        }
        if !matches!(slot.event_relation.as_str(), "none" | "before" | "after") {
            return Err(format!(
                "slot {} has unsupported event relation",
                slot.slot_id
            ));
        }
        match (slot.event_id.as_deref(), slot.event_relation.as_str()) {
            (None, "none") => {}
            (Some(event_id), "before" | "after") => {
                let event = events.get(event_id).ok_or_else(|| {
                    format!("slot {} refers to unknown event {event_id}", slot.slot_id)
                })?;
                let event_start = parse_utc_seconds(&event.window_start)?;
                let event_at = parse_utc_seconds(&event.occurred_at)?;
                let event_end = parse_utc_seconds(&event.window_end)?;
                if slot.event_relation == "before" && !(event_start <= time && time < event_at) {
                    return Err(format!(
                        "slot {} is not inside its declared pre-event window",
                        slot.slot_id
                    ));
                }
                if slot.event_relation == "after" && !(event_at < time && time <= event_end) {
                    return Err(format!(
                        "slot {} is not inside its declared post-event window",
                        slot.slot_id
                    ));
                }
            }
            _ => {
                return Err(format!(
                    "slot {} has inconsistent event id / relation",
                    slot.slot_id
                ));
            }
        }
        if slot_map.insert(slot.slot_id.as_str(), slot).is_some() {
            return Err(format!("duplicate schedule slot id: {}", slot.slot_id));
        }
        previous_time = Some(time);
        slot_times.push(time);
    }

    let mut observation_by_slot: HashMap<&str, &Observation> = HashMap::new();
    let mut observation_ids = HashSet::new();
    for observation in &campaign.observations {
        nonblank(&observation.observation_id, "observation.observation_id")?;
        nonblank(&observation.scheduled_slot_id, "observation.scheduled_slot_id")?;
        if !observation_ids.insert(observation.observation_id.as_str()) {
            return Err(format!(
                "duplicate observation_id: {}",
                observation.observation_id
            ));
        }
        let slot = slot_map
            .get(observation.scheduled_slot_id.as_str())
            .ok_or_else(|| {
                format!(
                    "observation {} refers to unknown slot",
                    observation.observation_id
                )
            })?;
        let observed_time = parse_utc_seconds(&observation.observed_at)?;
        let expected_time = parse_utc_seconds(&slot.scheduled_at)?;
        if (observed_time - expected_time).abs() > 1.0 {
            return Err(format!(
                "observation {} does not match scheduled time",
                observation.observation_id
            ));
        }
        if observation.gas_species != campaign.gas_species
            || observation.mass_basis != campaign.mass_basis
            || observation.flux_unit != campaign.flux_unit
        {
            return Err(format!(
                "observation {} has incompatible gas / basis / unit",
                observation.observation_id
            ));
        }
        if !matches!(
            observation.qc_state.as_str(),
            "pass" | "flagged" | "rejected"
        ) {
            return Err(format!(
                "observation {} has unsupported QC status",
                observation.observation_id
            ));
        }
        match observation.quantification_status.as_str() {
            "quantified" => {
                let value = observation.flux_value.ok_or_else(|| {
                    format!(
                        "observation {} is quantified but has no flux value",
                        observation.observation_id
                    )
                })?;
                if !value.is_finite() {
                    return Err(format!(
                        "observation {} has non-finite flux",
                        observation.observation_id
                    ));
                }
                if observation.detection_limit.is_some() {
                    return Err(format!(
                        "quantified observation {} must not have detection limit",
                        observation.observation_id
                    ));
                }
            }
            "left_censored" => {
                if observation.flux_value.is_some() {
                    return Err(format!(
                        "left-censored observation {} must not carry a point flux",
                        observation.observation_id
                    ));
                }
                let limit = observation.detection_limit.ok_or_else(|| {
                    format!(
                        "left-censored observation {} requires detection limit",
                        observation.observation_id
                    )
                })?;
                if !limit.is_finite() || limit < 0.0 {
                    return Err(format!(
                        "observation {} has invalid detection limit",
                        observation.observation_id
                    ));
                }
            }
            _ => {
                return Err(format!(
                    "observation {} has unsupported quantification status",
                    observation.observation_id
                ));
            }
        }
        if let Some(uncertainty) = observation.standard_uncertainty {
            if !uncertainty.is_finite() || uncertainty < 0.0 {
                return Err(format!(
                    "observation {} has invalid standard uncertainty",
                    observation.observation_id
                ));
            }
        }
        if let Some(group) = &observation.uncertainty_group_id {
            nonblank(group, "uncertainty_group_id")?;
        }
        if observation_by_slot
            .insert(observation.scheduled_slot_id.as_str(), observation)
            .is_some()
        {
            return Err(format!(
                "more than one observation maps to slot {}",
                observation.scheduled_slot_id
            ));
        }
    }

    let first_slot = slot_times[0];
    let last_slot = *slot_times.last().expect("at least two schedule slots");
    let first_slot_id = campaign.schedule_slots[0].slot_id.as_str();
    let last_slot_id = campaign.schedule_slots[campaign.schedule_slots.len() - 1]
        .slot_id
        .as_str();
    let boundary_schedule_near = first_slot - window_start <= campaign.boundary_tolerance_seconds
        && window_end - last_slot <= campaign.boundary_tolerance_seconds;

    let mut missing_schedule_slot_ids = Vec::new();
    let mut unusable_schedule_slot_ids = Vec::new();
    let mut usable_by_slot: HashMap<&str, &Observation> = HashMap::new();
    let mut censored_observation_count = 0;
    let mut flagged_or_rejected_observation_count = 0;
    let mut uncertainty_groups = HashSet::new();

    for slot in &campaign.schedule_slots {
        match observation_by_slot.get(slot.slot_id.as_str()) {
            None => {
                if slot
                    .missing_reason
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .is_empty()
                {
                    return Err(format!(
                        "missing slot {} must have an explicit missing_reason",
                        slot.slot_id
                    ));
                }
                missing_schedule_slot_ids.push(slot.slot_id.clone());
            }
            Some(observation) => {
                if slot.missing_reason.is_some() {
                    return Err(format!(
                        "observed slot {} must not also declare missing_reason",
                        slot.slot_id
                    ));
                }
                if !is_usable(observation) {
                    unusable_schedule_slot_ids.push(slot.slot_id.clone());
                    if observation.quantification_status == "left_censored" {
                        censored_observation_count += 1;
                    }
                    if observation.qc_state != "pass" {
                        flagged_or_rejected_observation_count += 1;
                    }
                } else {
                    usable_by_slot.insert(slot.slot_id.as_str(), *observation);
                }
                if let Some(group) = &observation.uncertainty_group_id {
                    uncertainty_groups.insert(group.clone());
                }
            }
        }
    }

    let boundary_coverage_complete = boundary_schedule_near
        && usable_by_slot.contains_key(first_slot_id)
        && usable_by_slot.contains_key(last_slot_id);

    let mut max_schedule_gap: Option<f64> = None;
    for pair in slot_times.windows(2) {
        let gap = pair[1] - pair[0];
        max_schedule_gap = Some(max_schedule_gap.map_or(gap, |value| value.max(gap)));
    }
    let max_observed_gap = {
        let mut usable_times = Vec::new();
        for slot in &campaign.schedule_slots {
            if usable_by_slot.contains_key(slot.slot_id.as_str()) {
                usable_times.push(parse_utc_seconds(&slot.scheduled_at)?);
            }
        }
        let mut maximum = None;
        for pair in usable_times.windows(2) {
            let gap = pair[1] - pair[0];
            maximum = Some(maximum.map_or(gap, |value: f64| value.max(gap)));
        }
        maximum
    };

    let mut excessive_observation_gap_count = 0usize;
    let mut unintegrated_gap_count = 0usize;
    let mut integrated = 0.0;
    let mut integrated_support_seconds = 0.0;
    for index in 0..campaign.schedule_slots.len() - 1 {
        let left_slot = &campaign.schedule_slots[index];
        let right_slot = &campaign.schedule_slots[index + 1];
        let left_time = slot_times[index];
        let right_time = slot_times[index + 1];
        let gap = right_time - left_time;
        let left = usable_by_slot.get(left_slot.slot_id.as_str());
        let right = usable_by_slot.get(right_slot.slot_id.as_str());
        if gap > campaign.maximum_gap_seconds {
            excessive_observation_gap_count += 1;
            unintegrated_gap_count += 1;
            continue;
        }
        if let (Some(left), Some(right)) = (left, right) {
            let left_flux = left.flux_value.expect("usable observation has flux");
            let right_flux = right.flux_value.expect("usable observation has flux");
            integrated += (left_flux + right_flux) / 2.0 * gap / 3_600.0;
            integrated_support_seconds += gap;
        } else {
            unintegrated_gap_count += 1;
        }
    }

    let total_duration = window_end - window_start;
    let uncovered_seconds = (total_duration - integrated_support_seconds).max(0.0);
    let integration_support_fraction = integrated_support_seconds / total_duration;
    let schedule_coverage_fraction =
        usable_by_slot.len() as f64 / campaign.schedule_slots.len() as f64;

    let mut event_coverages = Vec::new();
    let mut event_coverage_complete = true;
    for event in &campaign.events {
        let mut observed_before = 0;
        let mut observed_after = 0;
        for slot in &campaign.schedule_slots {
            if slot.event_id.as_deref() == Some(event.event_id.as_str())
                && usable_by_slot.contains_key(slot.slot_id.as_str())
            {
                match slot.event_relation.as_str() {
                    "before" => observed_before += 1,
                    "after" => observed_after += 1,
                    _ => {}
                }
            }
        }
        let qualified = observed_before >= event.minimum_before_observations
            && observed_after >= event.minimum_after_observations;
        event_coverage_complete &= qualified;
        event_coverages.push(EventCoverage {
            event_id: event.event_id.clone(),
            event_type: event.event_type.clone(),
            observed_before,
            required_before: event.minimum_before_observations,
            observed_after,
            required_after: event.minimum_after_observations,
            qualified,
        });
    }

    let schedule_complete = missing_schedule_slot_ids.is_empty()
        && unusable_schedule_slot_ids.is_empty()
        && excessive_observation_gap_count == 0;
    let integration_complete = schedule_complete
        && boundary_coverage_complete
        && unintegrated_gap_count == 0
        && (integrated_support_seconds - total_duration).abs() <= 1.0;

    let mut reasons = Vec::new();
    if !boundary_coverage_complete {
        reasons.push(
            "leading or trailing integration-window boundary is not observed within tolerance"
                .to_string(),
        );
    }
    if !missing_schedule_slot_ids.is_empty() {
        reasons.push(format!(
            "{} expected schedule slots are missing",
            missing_schedule_slot_ids.len()
        ));
    }
    if !unusable_schedule_slot_ids.is_empty() {
        reasons.push(format!(
            "{} observations are censored or fail QC",
            unusable_schedule_slot_ids.len()
        ));
    }
    if excessive_observation_gap_count > 0 {
        reasons.push(format!(
            "{} schedule gaps exceed maximum_gap_seconds",
            excessive_observation_gap_count
        ));
    }
    if !event_coverage_complete {
        reasons.push(
            "one or more predeclared event windows lack required before/after observations"
                .to_string(),
        );
    }
    if !integration_complete && !reasons.iter().any(|reason| reason.contains("coverage")) {
        reasons.push("observed segments do not cover the full integration window".to_string());
    }
    reasons.push("uncertainty and covariance propagation are not implemented in v0.1".to_string());

    let status = if !event_coverage_complete {
        "incomplete_event_coverage"
    } else if !integration_complete {
        "partial_integral_only"
    } else {
        "coverage_complete_uncertainty_unqualified"
    };

    Ok(AuditReport {
        audit_version: AUDIT_VERSION,
        status,
        fixture_status: FIXTURE_STATUS,
        fixture_version: envelope.fixture_version,
        campaign_id: campaign.campaign_id,
        gas_species: campaign.gas_species,
        mass_basis: campaign.mass_basis,
        flux_unit: campaign.flux_unit,
        window_start: campaign.window_start,
        window_end: campaign.window_end,
        input_sha256: sha256_hex(bytes),
        schedule_slot_count: campaign.schedule_slots.len(),
        usable_observation_count: usable_by_slot.len(),
        missing_schedule_slot_ids,
        unusable_schedule_slot_ids,
        schedule_coverage_fraction,
        boundary_coverage_complete,
        maximum_observed_gap_seconds: max_observed_gap,
        maximum_schedule_gap_seconds: max_schedule_gap,
        excessive_observation_gap_count,
        unintegrated_gap_count,
        event_coverages,
        event_coverage_complete,
        integrated_partial_mg_m2: if integrated_support_seconds > 0.0 {
            Some(integrated)
        } else {
            None
        },
        integrated_support_seconds,
        uncovered_seconds,
        integration_support_fraction,
        censored_observation_count,
        flagged_or_rejected_observation_count,
        uncertainty_status: "not_propagated_in_v0_1",
        uncertainty_group_count: uncertainty_groups.len(),
        qualification_reasons: reasons,
        interpretation_boundary: INTEGRAL_BOUNDARY,
    })
}

fn write_new_file(path: &Path, content: &[u8]) -> io::Result<()> {
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(content)?;
    file.write_all(b"\n")?;
    file.sync_all()
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(format!(
            "usage: {} <synthetic-campaign.json> <new-report.json>",
            args.first()
                .map(String::as_str)
                .unwrap_or("seasonal-ghg-coverage")
        )
        .into());
    }
    let input = fs::read(&args[1])?;
    let output_path = Path::new(&args[2]);
    if output_path.exists() {
        return Err(format!("refusing to overwrite report: {}", output_path.display()).into());
    }
    let report = audit_bytes(&input).map_err(io::Error::other)?;
    let report_bytes = serde_json::to_vec_pretty(&report)?;
    write_new_file(output_path, &report_bytes)?;
    println!(
        "audited campaign {}; status={}; observed={}/{}; integral_support={:.4}",
        report.campaign_id,
        report.status,
        report.usable_observation_count,
        report.schedule_slot_count,
        report.integration_support_fraction
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("seasonal GHG coverage audit failed: {error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn fixture() -> Value {
        serde_json::from_str(include_str!(
            "../fixtures/synthetic-seasonal-ghg-campaign.json"
        ))
        .expect("synthetic fixture parses")
    }

    fn audit(value: &Value) -> Result<AuditReport, String> {
        audit_bytes(value.to_string().as_bytes())
    }

    fn add_missing_schedule_observations(value: &mut Value) {
        let observations = value["campaign"]["observations"].as_array_mut().unwrap();
        observations.push(serde_json::json!({
            "observation_id":"obs-04",
            "scheduled_slot_id":"s3",
            "observed_at":"2026-11-20T11:00:00Z",
            "gas_species":"N2O",
            "mass_basis":"N2O",
            "flux_value":0.5,
            "flux_unit":"mg_N2O_m2_h",
            "quantification_status":"quantified",
            "detection_limit":null,
            "qc_state":"pass",
            "standard_uncertainty":0.04,
            "uncertainty_group_id":"shared-instrument-01"
        }));
        observations.push(serde_json::json!({
            "observation_id":"obs-05",
            "scheduled_slot_id":"s4",
            "observed_at":"2026-11-20T12:00:00Z",
            "gas_species":"N2O",
            "mass_basis":"N2O",
            "flux_value":0.3,
            "flux_unit":"mg_N2O_m2_h",
            "quantification_status":"quantified",
            "detection_limit":null,
            "qc_state":"pass",
            "standard_uncertainty":0.02,
            "uncertainty_group_id":"shared-instrument-01"
        }));
        value["campaign"]["schedule_slots"][3]["missing_reason"] = Value::Null;
        value["campaign"]["schedule_slots"][4]["missing_reason"] = Value::Null;
    }

    #[test]
    fn missing_event_window_is_not_a_qualified_seasonal_total() {
        let report = audit(&fixture()).unwrap();
        assert_eq!(report.status, "incomplete_event_coverage");
        assert_eq!(report.schedule_slot_count, 5);
        assert_eq!(report.usable_observation_count, 3);
        assert_eq!(report.missing_schedule_slot_ids.len(), 2);
        assert!(!report.event_coverage_complete);
        assert!(!report.uncertainty_status.is_empty());
        assert!(report.integrated_partial_mg_m2.is_some());
        assert!(report.integration_support_fraction < 1.0);
    }

    #[test]
    fn complete_schedule_and_events_still_do_not_qualify_without_uncertainty() {
        let mut value = fixture();
        add_missing_schedule_observations(&mut value);
        let report = audit(&value).unwrap();
        assert_eq!(report.status, "coverage_complete_uncertainty_unqualified");
        assert!(report.boundary_coverage_complete);
        assert!(report.event_coverage_complete);
        assert!((report.integration_support_fraction - 1.0).abs() < 1e-12);
        assert_eq!(report.uncertainty_status, "not_propagated_in_v0_1");
    }

    #[test]
    fn exact_piecewise_linear_fixture_has_hand_calculated_partial_integral() {
        let report = audit(&fixture()).unwrap();
        // 08:00→09:00: mean flux = 0.3 mg m-2 h-1 over one hour.
        assert!((report.integrated_partial_mg_m2.unwrap() - 0.95).abs() < 1e-12);
        assert!((report.integrated_support_seconds - 7200.0).abs() < 1e-12);
        assert!((report.uncovered_seconds - 7200.0).abs() < 1e-12);
    }

    #[test]
    fn missing_start_or_end_observation_breaks_boundary_coverage() {
        let report = audit(&fixture()).unwrap();
        assert!(!report.boundary_coverage_complete);
        assert!(report.missing_schedule_slot_ids.contains(&"s4".to_string()));
    }

    #[test]
    fn left_censored_observation_is_not_zero_and_blocks_qualification() {
        let mut value = fixture();
        value["campaign"]["observations"][1]["quantification_status"] =
            Value::String("left_censored".to_string());
        value["campaign"]["observations"][1]["flux_value"] = Value::Null;
        value["campaign"]["observations"][1]["detection_limit"] = serde_json::json!(0.05);
        let report = audit(&value).unwrap();
        assert_eq!(report.censored_observation_count, 1);
        assert!(
            report
                .unusable_schedule_slot_ids
                .contains(&"s1".to_string())
        );
        assert!(!report.event_coverage_complete);
    }

    #[test]
    fn excessive_schedule_gap_is_not_integrated() {
        let mut value = fixture();
        value["campaign"]["maximum_gap_seconds"] = serde_json::json!(1800);
        let report = audit(&value).unwrap();
        assert_eq!(report.excessive_observation_gap_count, 4);
        assert!((report.integrated_support_seconds - 0.0).abs() < 1e-12);
        assert_eq!(report.integrated_partial_mg_m2, None);
    }

    #[test]
    fn incompatible_mass_basis_is_rejected() {
        let mut value = fixture();
        value["campaign"]["mass_basis"] = Value::String("N2O_N".to_string());
        assert!(audit(&value).is_err());
    }

    #[test]
    fn duplicate_schedule_slots_are_rejected() {
        let mut value = fixture();
        value["campaign"]["schedule_slots"][1]["slot_id"] =
            value["campaign"]["schedule_slots"][0]["slot_id"].clone();
        assert!(audit(&value).is_err());
    }

    #[test]
    fn duplicate_observations_for_one_slot_are_rejected() {
        let mut value = fixture();
        let first = value["campaign"]["observations"][0].clone();
        value["campaign"]["observations"]
            .as_array_mut()
            .unwrap()
            .push(first);
        assert!(audit(&value).is_err());
    }

    #[test]
    fn observations_cannot_be_moved_to_another_schedule_time() {
        let mut value = fixture();
        value["campaign"]["observations"][0]["observed_at"] =
            Value::String("2026-11-20T08:30:00Z".to_string());
        assert!(audit(&value).is_err());
    }

    #[test]
    fn blank_missing_reason_is_rejected() {
        let mut value = fixture();
        value["campaign"]["schedule_slots"][3]["missing_reason"] = Value::String(" ".to_string());
        assert!(audit(&value).is_err());
    }

    #[test]
    fn non_utc_timestamp_is_rejected() {
        assert!(parse_utc_seconds("2026-11-20T10:00:00+02:00").is_err());
    }

    #[test]
    fn event_window_order_is_validated() {
        let mut value = fixture();
        value["campaign"]["events"][0]["occurred_at"] =
            Value::String("2026-11-20T08:15:00Z".to_string());
        assert!(audit(&value).is_err());
    }
}
