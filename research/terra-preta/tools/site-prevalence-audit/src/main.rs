use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::HashSet,
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const EVENT_ELIGIBILITY_MM: f64 = 2.0;
const VT_VARIANTS: [(&str, &str); 3] = [
    ("vt_maximum_ksat", "maximum depth-weighted Rosetta Ksat"),
    ("vt_median_ksat", "median depth-weighted Rosetta Ksat"),
    ("vt_median_plus_sd_ksat", "median depth-weighted Rosetta Ksat plus one standard deviation"),
];

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    site_id: String,
    ecoregion_id: String,
    source_sha256: String,
    source_evidence_ref: String,
    rainfall_minimum_event_mm: f64,
    site_events: Vec<SiteEvent>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SiteEvent {
    event_id: String,
    rainfall_total_mm: f64,
    soil_moisture_available: bool,
    declared_sensor_count_10_to_100_cm: usize,
    declared_profile_count: usize,
    vt_maximum_ksat: UnitEvidence,
    vt_median_ksat: UnitEvidence,
    vt_median_plus_sd_ksat: UnitEvidence,
    nsr: UnitEvidence,
    evidence_ref: String,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct UnitEvidence {
    declared_units: usize,
    evaluable_units: usize,
    positive_units: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum EventMethodOutcome {
    Positive,
    Negative,
    Indeterminate,
    NotEligible,
    NoSoilMoistureData,
}

#[derive(Serialize)]
struct EventAudit {
    event_id: String,
    rainfall_total_mm: f64,
    rainfall_eligible: bool,
    soil_moisture_available: bool,
    included_in_site_probability_denominator: bool,
    vt_maximum_ksat: EventMethodOutcome,
    vt_median_ksat: EventMethodOutcome,
    vt_median_plus_sd_ksat: EventMethodOutcome,
    nsr: EventMethodOutcome,
}

#[derive(Serialize)]
struct MethodSummary {
    method_id: String,
    threshold_description: String,
    positive_event_count: usize,
    negative_event_count: usize,
    indeterminate_event_count: usize,
    site_probability_denominator_events: usize,
    method_evaluable_event_count: usize,
    conditional_probability_among_evaluable_events: Option<f64>,
    identification_lower_bound: Option<f64>,
    identification_upper_bound: Option<f64>,
    method_coverage_fraction: Option<f64>,
    interpretation_boundary: &'static str,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    site_id: String,
    ecoregion_id: String,
    rainfall_minimum_event_mm: f64,
    total_input_event_count: usize,
    rainfall_eligible_event_count: usize,
    rainfall_ineligible_event_count: usize,
    soil_moisture_unavailable_event_count: usize,
    site_probability_denominator_event_count: usize,
    methods: Vec<MethodSummary>,
    event_audits: Vec<EventAudit>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn validate_evidence(
    evidence: &UnitEvidence,
    declared_units: usize,
    label: &str,
) -> Result<(), String> {
    if evidence.declared_units != declared_units {
        return Err(format!(
            "{label}.declared_units={} does not match site-event roster count {declared_units}",
            evidence.declared_units
        ));
    }
    if evidence.evaluable_units > evidence.declared_units {
        return Err(format!("{label}.evaluable_units exceeds declared_units"));
    }
    if evidence.positive_units > evidence.evaluable_units {
        return Err(format!("{label}.positive_units exceeds evaluable_units"));
    }
    Ok(())
}

fn validate(input: &Input) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    for (value, label) in [
        (&input.study_id, "study_id"),
        (&input.site_id, "site_id"),
        (&input.ecoregion_id, "ecoregion_id"),
        (&input.source_evidence_ref, "source_evidence_ref"),
    ] {
        nonblank(value, label)?;
    }
    if input.source_sha256.len() != 64
        || !input.source_sha256.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if !input.rainfall_minimum_event_mm.is_finite()
        || input.rainfall_minimum_event_mm <= 0.0
    {
        return Err("rainfall_minimum_event_mm must be finite and greater than zero".to_owned());
    }
    if input.rainfall_minimum_event_mm != EVENT_ELIGIBILITY_MM {
        return Err(format!(
            "v0.1 reproduces the 2025 study's inclusive {EVENT_ELIGIBILITY_MM} mm event-eligibility rule"
        ));
    }
    if input.site_events.is_empty() {
        return Err("site_events must not be empty".to_owned());
    }

    let mut ids = HashSet::new();
    for (index, event) in input.site_events.iter().enumerate() {
        nonblank(&event.event_id, &format!("site_events[{index}].event_id"))?;
        nonblank(&event.evidence_ref, &format!("site_events[{index}].evidence_ref"))?;
        if !ids.insert(event.event_id.as_str()) {
            return Err(format!("duplicate event_id {:?}", event.event_id));
        }
        if !event.rainfall_total_mm.is_finite() || event.rainfall_total_mm < 0.0 {
            return Err(format!(
                "site_events[{index}].rainfall_total_mm must be finite and non-negative"
            ));
        }
        if event.soil_moisture_available {
            if event.declared_sensor_count_10_to_100_cm == 0 || event.declared_profile_count == 0 {
                return Err(format!(
                    "site_events[{index}] has soil moisture available but a zero sensor/profile roster"
                ));
            }
        } else if event.declared_sensor_count_10_to_100_cm != 0
            || event.declared_profile_count != 0
        {
            return Err(format!(
                "site_events[{index}] has no soil-moisture data but declares sensors or profiles"
            ));
        }

        validate_evidence(
            &event.vt_maximum_ksat,
            event.declared_sensor_count_10_to_100_cm,
            &format!("site_events[{index}].vt_maximum_ksat"),
        )?;
        validate_evidence(
            &event.vt_median_ksat,
            event.declared_sensor_count_10_to_100_cm,
            &format!("site_events[{index}].vt_median_ksat"),
        )?;
        validate_evidence(
            &event.vt_median_plus_sd_ksat,
            event.declared_sensor_count_10_to_100_cm,
            &format!("site_events[{index}].vt_median_plus_sd_ksat"),
        )?;
        validate_evidence(
            &event.nsr,
            event.declared_profile_count,
            &format!("site_events[{index}].nsr"),
        )?;
        if !event.soil_moisture_available {
            for (evidence, label) in [
                (&event.vt_maximum_ksat, "vt_maximum_ksat"),
                (&event.vt_median_ksat, "vt_median_ksat"),
                (&event.vt_median_plus_sd_ksat, "vt_median_plus_sd_ksat"),
                (&event.nsr, "nsr"),
            ] {
                if evidence.evaluable_units != 0 || evidence.positive_units != 0 {
                    return Err(format!(
                        "site_events[{index}].{label} cannot be evaluable without soil-moisture data"
                    ));
                }
            }
        }
    }
    Ok(())
}

fn event_outcome(
    rainfall_eligible: bool,
    moisture_available: bool,
    evidence: &UnitEvidence,
) -> EventMethodOutcome {
    if !rainfall_eligible {
        EventMethodOutcome::NotEligible
    } else if !moisture_available {
        EventMethodOutcome::NoSoilMoistureData
    } else if evidence.positive_units > 0 {
        EventMethodOutcome::Positive
    } else if evidence.declared_units > 0
        && evidence.evaluable_units == evidence.declared_units
    {
        EventMethodOutcome::Negative
    } else {
        EventMethodOutcome::Indeterminate
    }
}

fn method_summary(
    method_id: &str,
    description: &str,
    outcomes: &[EventMethodOutcome],
    denominator: usize,
) -> MethodSummary {
    let positive = outcomes.iter().filter(|value| **value == EventMethodOutcome::Positive).count();
    let negative = outcomes.iter().filter(|value| **value == EventMethodOutcome::Negative).count();
    let indeterminate = outcomes.iter().filter(|value| **value == EventMethodOutcome::Indeterminate).count();
    let evaluable = positive + negative;

    let (conditional, lower, upper, coverage) = if denominator == 0 {
        (None, None, None, None)
    } else {
        (
            (evaluable > 0).then_some(positive as f64 / evaluable as f64),
            Some(positive as f64 / denominator as f64),
            Some((positive + indeterminate) as f64 / denominator as f64),
            Some(evaluable as f64 / denominator as f64),
        )
    };

    MethodSummary {
        method_id: method_id.to_owned(),
        threshold_description: description.to_owned(),
        positive_event_count: positive,
        negative_event_count: negative,
        indeterminate_event_count: indeterminate,
        site_probability_denominator_events: denominator,
        method_evaluable_event_count: evaluable,
        conditional_probability_among_evaluable_events: conditional,
        identification_lower_bound: lower,
        identification_upper_bound: upper,
        method_coverage_fraction: coverage,
        interpretation_boundary: "The lower/upper bounds treat every indeterminate eligible event as negative/positive respectively; they are identification bounds, not confidence intervals.",
    }
}

fn evaluate(input: &Input, input_sha256: String) -> Report {
    let mut eligible = 0usize;
    let mut ineligible = 0usize;
    let mut moisture_unavailable = 0usize;
    let mut denominator = 0usize;
    let mut vt_max = Vec::with_capacity(input.site_events.len());
    let mut vt_median = Vec::with_capacity(input.site_events.len());
    let mut vt_median_sd = Vec::with_capacity(input.site_events.len());
    let mut nsr = Vec::with_capacity(input.site_events.len());
    let mut audits = Vec::with_capacity(input.site_events.len());

    for event in &input.site_events {
        let rainfall_eligible = event.rainfall_total_mm >= EVENT_ELIGIBILITY_MM;
        if rainfall_eligible {
            eligible += 1;
        } else {
            ineligible += 1;
        }
        let in_probability_denominator = rainfall_eligible && event.soil_moisture_available;
        if rainfall_eligible && !event.soil_moisture_available {
            moisture_unavailable += 1;
        }
        if in_probability_denominator {
            denominator += 1;
        }

        let outcomes = [
            event_outcome(rainfall_eligible, event.soil_moisture_available, &event.vt_maximum_ksat),
            event_outcome(rainfall_eligible, event.soil_moisture_available, &event.vt_median_ksat),
            event_outcome(rainfall_eligible, event.soil_moisture_available, &event.vt_median_plus_sd_ksat),
            event_outcome(rainfall_eligible, event.soil_moisture_available, &event.nsr),
        ];
        vt_max.push(outcomes[0]);
        vt_median.push(outcomes[1]);
        vt_median_sd.push(outcomes[2]);
        nsr.push(outcomes[3]);
        audits.push(EventAudit {
            event_id: event.event_id.clone(),
            rainfall_total_mm: event.rainfall_total_mm,
            rainfall_eligible,
            soil_moisture_available: event.soil_moisture_available,
            included_in_site_probability_denominator: in_probability_denominator,
            vt_maximum_ksat: outcomes[0],
            vt_median_ksat: outcomes[1],
            vt_median_plus_sd_ksat: outcomes[2],
            nsr: outcomes[3],
        });
    }

    let methods = vec![
        method_summary(VT_VARIANTS[0].0, VT_VARIANTS[0].1, &vt_max, denominator),
        method_summary(VT_VARIANTS[1].0, VT_VARIANTS[1].1, &vt_median, denominator),
        method_summary(VT_VARIANTS[2].0, VT_VARIANTS[2].1, &vt_median_sd, denominator),
        method_summary("nsr", "nonsequential-response profile diagnostic", &nsr, denominator),
    ];

    Report {
        audit_version: "0.1.0",
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        site_id: input.site_id.clone(),
        ecoregion_id: input.ecoregion_id.clone(),
        rainfall_minimum_event_mm: EVENT_ELIGIBILITY_MM,
        total_input_event_count: input.site_events.len(),
        rainfall_eligible_event_count: eligible,
        rainfall_ineligible_event_count: ineligible,
        soil_moisture_unavailable_event_count: moisture_unavailable,
        site_probability_denominator_event_count: denominator,
        methods,
        event_audits: audits,
        warnings: vec![
            "Each input row must represent one unique site-level rainfall event, not one sensor, depth or plot record. Aggregate multiple profiles/sensors within each storm before running this audit.".to_owned(),
            "A detected positive in at least one valid unit is retained even when other units are unavailable; a negative requires every declared unit to be evaluable and negative.".to_owned(),
            "Indeterminate eligible events remain inside the study's soil-moisture-available denominator and create identification bounds; they are never silently converted to negative.".to_owned(),
            "VT maximum, median and median-plus-standard-deviation Ksat variants are separate estimands, not pooled thresholds.".to_owned(),
            "The output is descriptive per-site prevalence, not a confidence interval or causal estimate; events are temporally dependent and site sample size remains the number of sites for site-level transfer.".to_owned(),
        ],
        interpretation_boundary: "This reproduces the event eligibility and site probability denominator of Li et al. (2025) while making method coverage explicit. VT/NSR are soil-moisture-derived proxies, not tracer-confirmed physical pathway truth.",
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: site-prevalence-audit <site-events.json> <new-report.json>",
        ).into());
    }
    let bytes = fs::read(&args[1])?;
    let digest = format!("{:x}", Sha256::digest(&bytes));
    let input: Input = serde_json::from_slice(&bytes)?;
    validate(&input).map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
    let report = evaluate(&input, digest);
    let encoded = serde_json::to_vec_pretty(&report)?;
    let mut output = OpenOptions::new().write(true).create_new(true).open(&args[2])
        .map_err(|error| io::Error::new(error.kind(), format!("refusing overwrite or cannot create report: {error}")))?;
    output.write_all(&encoded)?;
    output.write_all(b"\n")?;
    output.sync_all()?;
    println!(
        "site prevalence audit complete: site={} eligible_events={} moisture_available_denominator={} methods={} input_sha256={}",
        report.site_id,
        report.rainfall_eligible_event_count,
        report.site_probability_denominator_event_count,
        report.methods.len(),
        report.input_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("site prevalence audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn evidence(declared: usize, evaluable: usize, positive: usize) -> UnitEvidence {
        UnitEvidence {
            declared_units: declared,
            evaluable_units: evaluable,
            positive_units: positive,
        }
    }

    fn fixture() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic-2025-neon".to_owned(),
            site_id: "SYN1".to_owned(),
            ecoregion_id: "synthetic-ecoregion".to_owned(),
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic-fixture-only".to_owned(),
            rainfall_minimum_event_mm: 2.0,
            site_events: vec![
                SiteEvent {
                    event_id: "E1".to_owned(),
                    rainfall_total_mm: 6.0,
                    soil_moisture_available: true,
                    declared_sensor_count_10_to_100_cm: 4,
                    declared_profile_count: 2,
                    vt_maximum_ksat: evidence(4, 4, 1),
                    vt_median_ksat: evidence(4, 4, 2),
                    vt_median_plus_sd_ksat: evidence(4, 4, 0),
                    nsr: evidence(2, 2, 1),
                    evidence_ref: "synthetic-event-1".to_owned(),
                },
                SiteEvent {
                    event_id: "E2".to_owned(),
                    rainfall_total_mm: 2.0,
                    soil_moisture_available: true,
                    declared_sensor_count_10_to_100_cm: 4,
                    declared_profile_count: 2,
                    vt_maximum_ksat: evidence(4, 4, 0),
                    vt_median_ksat: evidence(4, 4, 2),
                    vt_median_plus_sd_ksat: evidence(4, 4, 0),
                    nsr: evidence(2, 2, 0),
                    evidence_ref: "synthetic-event-2".to_owned(),
                },
                SiteEvent {
                    event_id: "E3".to_owned(),
                    rainfall_total_mm: 4.0,
                    soil_moisture_available: true,
                    declared_sensor_count_10_to_100_cm: 4,
                    declared_profile_count: 2,
                    vt_maximum_ksat: evidence(4, 2, 0),
                    vt_median_ksat: evidence(4, 2, 1),
                    vt_median_plus_sd_ksat: evidence(4, 2, 0),
                    nsr: evidence(2, 1, 0),
                    evidence_ref: "synthetic-event-3".to_owned(),
                },
                SiteEvent {
                    event_id: "E4".to_owned(),
                    rainfall_total_mm: 1.9,
                    soil_moisture_available: true,
                    declared_sensor_count_10_to_100_cm: 4,
                    declared_profile_count: 2,
                    vt_maximum_ksat: evidence(4, 4, 4),
                    vt_median_ksat: evidence(4, 4, 4),
                    vt_median_plus_sd_ksat: evidence(4, 4, 4),
                    nsr: evidence(2, 2, 2),
                    evidence_ref: "synthetic-event-4".to_owned(),
                },
                SiteEvent {
                    event_id: "E5".to_owned(),
                    rainfall_total_mm: 8.0,
                    soil_moisture_available: false,
                    declared_sensor_count_10_to_100_cm: 0,
                    declared_profile_count: 0,
                    vt_maximum_ksat: evidence(0, 0, 0),
                    vt_median_ksat: evidence(0, 0, 0),
                    vt_median_plus_sd_ksat: evidence(0, 0, 0),
                    nsr: evidence(0, 0, 0),
                    evidence_ref: "synthetic-event-5".to_owned(),
                },
            ],
        }
    }

    #[test]
    fn uses_two_mm_inclusive_and_available_moisture_denominator() {
        let input = fixture();
        validate(&input).unwrap();
        let report = evaluate(&input, "b".repeat(64));
        assert_eq!(report.rainfall_eligible_event_count, 4);
        assert_eq!(report.rainfall_ineligible_event_count, 1);
        assert_eq!(report.soil_moisture_unavailable_event_count, 1);
        assert_eq!(report.site_probability_denominator_event_count, 3);
    }

    #[test]
    fn retains_indeterminate_events_as_identification_bounds() {
        let input = fixture();
        let report = evaluate(&input, "c".repeat(64));
        let vt = &report.methods[0];
        assert_eq!(vt.positive_event_count, 1);
        assert_eq!(vt.negative_event_count, 1);
        assert_eq!(vt.indeterminate_event_count, 1);
        assert_eq!(vt.conditional_probability_among_evaluable_events, Some(0.5));
        assert!((vt.identification_lower_bound.unwrap() - 1.0 / 3.0).abs() < 1e-12);
        assert!((vt.identification_upper_bound.unwrap() - 2.0 / 3.0).abs() < 1e-12);
        assert!((vt.method_coverage_fraction.unwrap() - 2.0 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn keeps_vt_threshold_variants_and_nsr_separate() {
        let input = fixture();
        let report = evaluate(&input, "d".repeat(64));
        assert_eq!(report.methods.len(), 4);
        assert_eq!(report.methods[1].positive_event_count, 3);
        assert_eq!(report.methods[2].positive_event_count, 0);
        assert_eq!(report.methods[3].positive_event_count, 1);
        assert_eq!(report.event_audits[0].vt_maximum_ksat, EventMethodOutcome::Positive);
        assert_eq!(report.event_audits[3].vt_maximum_ksat, EventMethodOutcome::NotEligible);
    }

    #[test]
    fn refuses_to_treat_partial_nonresponse_as_a_negative() {
        let input = fixture();
        let report = evaluate(&input, "e".repeat(64));
        assert_eq!(report.event_audits[2].nsr, EventMethodOutcome::Indeterminate);
    }

    #[test]
    fn rejects_duplicate_site_event_ids() {
        let mut input = fixture();
        input.site_events[1].event_id = "E1".to_owned();
        assert!(validate(&input).unwrap_err().contains("duplicate event_id"));
    }

    #[test]
    fn rejects_inconsistent_unit_counts() {
        let mut input = fixture();
        input.site_events[0].vt_maximum_ksat.positive_units = 5;
        assert!(validate(&input).unwrap_err().contains("exceeds evaluable_units"));
    }
}
