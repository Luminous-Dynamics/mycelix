use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{HashMap, HashSet},
    env,
    error::Error,
    fs,
    fs::OpenOptions,
    io::{self, Write},
};

const RAINFALL_ELIGIBILITY_MM: f64 = 2.0;
const BAND_COUNT: usize = 4;
const DEFAULT_BOOTSTRAP_REPLICATES: usize = 2000;

const INTENSITY_BANDS: [(&str, f64, Option<f64>); BAND_COUNT] = [
    ("lt_5_mm_h", f64::NEG_INFINITY, Some(5.0)),
    ("5_to_lt_12_mm_h", 5.0, Some(12.0)),
    ("12_to_lt_13_mm_h", 12.0, Some(13.0)),
    ("gte_13_mm_h", 13.0, None),
];

const METHODS: [(&str, &str); 4] = [
    ("vt_maximum_ksat", "velocity threshold: maximum depth-weighted Rosetta Ksat"),
    ("vt_median_ksat", "velocity threshold: median depth-weighted Rosetta Ksat"),
    ("vt_median_plus_sd_ksat", "velocity threshold: median plus one standard deviation depth-weighted Rosetta Ksat"),
    ("nsr", "nonsequential response"),
];

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum MethodOutcome {
    Positive,
    Negative,
    Indeterminate,
    NotEligible,
    NoSoilMoistureData,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    schema_version: String,
    study_id: String,
    source_sha256: String,
    source_evidence_ref: String,
    rainfall_minimum_event_mm: f64,
    bootstrap_replicates: usize,
    bootstrap_seed: u64,
    events: Vec<SiteEvent>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SiteEvent {
    site_id: String,
    ecoregion_id: String,
    event_id: String,
    rainfall_total_mm: f64,
    peak_intensity_mm_h: f64,
    soil_moisture_available: bool,
    vt_maximum_ksat: MethodOutcome,
    vt_median_ksat: MethodOutcome,
    vt_median_plus_sd_ksat: MethodOutcome,
    nsr: MethodOutcome,
    evidence_ref: String,
}

#[derive(Clone, Copy, Debug, Default)]
struct Counts {
    positive: usize,
    negative: usize,
    indeterminate: usize,
}

impl Counts {
    fn add(&mut self, outcome: MethodOutcome) {
        match outcome {
            MethodOutcome::Positive => self.positive += 1,
            MethodOutcome::Negative => self.negative += 1,
            MethodOutcome::Indeterminate => self.indeterminate += 1,
            MethodOutcome::NotEligible | MethodOutcome::NoSoilMoistureData => {}
        }
    }

    fn evaluable(self) -> usize {
        self.positive + self.negative
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct SiteBandCounts {
    methods: [Counts; 4],
    event_count: usize,
}

#[derive(Serialize)]
struct MethodBandSummary {
    method_id: String,
    event_count: usize,
    site_count: usize,
    positive_event_count: usize,
    negative_event_count: usize,
    indeterminate_event_count: usize,
    evaluable_event_count: usize,
    conditional_probability_among_evaluable_events: Option<f64>,
    identification_lower_bound: Option<f64>,
    identification_upper_bound: Option<f64>,
    site_cluster_bootstrap_95pct: Option<[f64; 2]>,
    valid_bootstrap_replicates: usize,
    interpretation_boundary: &'static str,
}

#[derive(Serialize)]
struct BandSummary {
    band_id: String,
    lower_bound_mm_h: f64,
    upper_bound_exclusive_mm_h: Option<f64>,
    qualifying_event_count: usize,
    site_count: usize,
    methods: Vec<MethodBandSummary>,
}

#[derive(Serialize)]
struct Report {
    audit_version: &'static str,
    input_sha256: String,
    source_sha256: String,
    study_id: String,
    total_input_event_count: usize,
    eligible_event_count: usize,
    ineligible_event_count: usize,
    no_soil_moisture_event_count: usize,
    unique_site_count: usize,
    bootstrap_replicates_requested: usize,
    bootstrap_seed: u64,
    bands: Vec<BandSummary>,
    warnings: Vec<String>,
    interpretation_boundary: &'static str,
}

fn nonblank(value: &str, label: &str) -> Result<(), String> {
    if value.trim().is_empty() {
        return Err(format!("{label} must not be blank"));
    }
    Ok(())
}

fn outcome_valid_for_event(
    outcome: MethodOutcome,
    rainfall_eligible: bool,
    soil_moisture_available: bool,
) -> bool {
    if !rainfall_eligible {
        outcome == MethodOutcome::NotEligible
    } else if !soil_moisture_available {
        outcome == MethodOutcome::NoSoilMoistureData
    } else {
        matches!(
            outcome,
            MethodOutcome::Positive | MethodOutcome::Negative | MethodOutcome::Indeterminate
        )
    }
}

fn band_index(intensity: f64) -> usize {
    if intensity < 5.0 {
        0
    } else if intensity < 12.0 {
        1
    } else if intensity < 13.0 {
        2
    } else {
        3
    }
}

fn validate(input: &Input) -> Result<(), String> {
    if input.schema_version != "0.1.0" {
        return Err(format!("unsupported schema_version {:?}", input.schema_version));
    }
    nonblank(&input.study_id, "study_id")?;
    nonblank(&input.source_evidence_ref, "source_evidence_ref")?;
    if input.source_sha256.len() != 64
        || !input.source_sha256.bytes().all(|byte| byte.is_ascii_hexdigit())
    {
        return Err("source_sha256 must contain exactly 64 hexadecimal characters".to_owned());
    }
    if input.rainfall_minimum_event_mm != RAINFALL_ELIGIBILITY_MM {
        return Err("v0.1 requires the inclusive 2.0 mm 2025-paper rainfall eligibility rule".to_owned());
    }
    if input.bootstrap_replicates < 100 || input.bootstrap_replicates > 100_000 {
        return Err("bootstrap_replicates must be within [100, 100000]".to_owned());
    }
    if input.bootstrap_seed == 0 {
        return Err("bootstrap_seed must be non-zero".to_owned());
    }
    if input.events.is_empty() {
        return Err("events must not be empty".to_owned());
    }

    let mut seen = HashSet::new();
    let mut site_ecoregions = HashMap::<&str, &str>::new();
    for (index, event) in input.events.iter().enumerate() {
        for (value, label) in [
            (&event.site_id, "site_id"),
            (&event.ecoregion_id, "ecoregion_id"),
            (&event.event_id, "event_id"),
            (&event.evidence_ref, "evidence_ref"),
        ] {
            nonblank(value, &format!("events[{index}].{label}"))?;
        }
        if !seen.insert((event.site_id.as_str(), event.event_id.as_str())) {
            return Err(format!(
                "duplicate rainfall event {:?} for site {:?}",
                event.event_id, event.site_id
            ));
        }
        if let Some(previous_ecoregion) = site_ecoregions.insert(
            event.site_id.as_str(),
            event.ecoregion_id.as_str(),
        ) {
            if previous_ecoregion != event.ecoregion_id {
                return Err(format!(
                    "site {:?} is assigned to multiple ecoregions",
                    event.site_id
                ));
            }
        }
        if !event.rainfall_total_mm.is_finite() || event.rainfall_total_mm < 0.0 {
            return Err(format!("events[{index}].rainfall_total_mm must be finite and non-negative"));
        }
        if !event.peak_intensity_mm_h.is_finite() || event.peak_intensity_mm_h < 0.0 {
            return Err(format!("events[{index}].peak_intensity_mm_h must be finite and non-negative"));
        }
        let eligible = event.rainfall_total_mm >= RAINFALL_ELIGIBILITY_MM;
        for (outcome, method) in [
            (event.vt_maximum_ksat, "vt_maximum_ksat"),
            (event.vt_median_ksat, "vt_median_ksat"),
            (event.vt_median_plus_sd_ksat, "vt_median_plus_sd_ksat"),
            (event.nsr, "nsr"),
        ] {
            if !outcome_valid_for_event(outcome, eligible, event.soil_moisture_available) {
                return Err(format!(
                    "events[{index}].{method} is inconsistent with rainfall eligibility or soil-moisture availability"
                ));
            }
        }
        if eligible && event.soil_moisture_available && event.peak_intensity_mm_h <= 0.0 {
            return Err(format!("events[{index}] qualifying wet event must have positive peak_intensity_mm_h"));
        }
    }
    Ok(())
}

struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut value = self.state;
        value ^= value << 13;
        value ^= value >> 7;
        value ^= value << 17;
        self.state = value;
        value
    }

    fn index(&mut self, upper: usize) -> usize {
        (self.next_u64() % upper as u64) as usize
    }
}

fn percentile(sorted: &[f64], quantile: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    let position = quantile * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        Some(sorted[lower])
    } else {
        let fraction = position - lower as f64;
        Some(sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction)
    }
}

fn method_outcomes(event: &SiteEvent) -> [MethodOutcome; 4] {
    [
        event.vt_maximum_ksat,
        event.vt_median_ksat,
        event.vt_median_plus_sd_ksat,
        event.nsr,
    ]
}

fn evaluate(input: &Input, input_sha256: String) -> Report {
    let mut site_index = HashMap::<&str, usize>::new();
    let mut site_ids = Vec::<String>::new();
    let mut site_ecoregions = HashMap::<String, String>::new();
    let mut site_band_counts = Vec::<[SiteBandCounts; BAND_COUNT]>::new();
    let mut eligible_count = 0usize;
    let mut ineligible_count = 0usize;
    let mut no_moisture_count = 0usize;

    for event in &input.events {
        if !site_index.contains_key(event.site_id.as_str()) {
            let index = site_ids.len();
            site_index.insert(event.site_id.as_str(), index);
            site_ids.push(event.site_id.clone());
            site_ecoregions.insert(event.site_id.clone(), event.ecoregion_id.clone());
            site_band_counts.push([SiteBandCounts::default(); BAND_COUNT]);
        }

        let eligible = event.rainfall_total_mm >= RAINFALL_ELIGIBILITY_MM;
        if !eligible {
            ineligible_count += 1;
            continue;
        }
        eligible_count += 1;
        if !event.soil_moisture_available {
            no_moisture_count += 1;
            continue;
        }

        let site = *site_index.get(event.site_id.as_str()).expect("site was inserted above");
        let band = band_index(event.peak_intensity_mm_h);
        site_band_counts[site][band].event_count += 1;
        for (method_index, outcome) in method_outcomes(event).iter().copied().enumerate() {
            site_band_counts[site][band].methods[method_index].add(outcome);
        }
    }

    let mut rng = XorShift64::new(input.bootstrap_seed);
    let mut bands = Vec::with_capacity(BAND_COUNT);

    for band_index_value in 0..BAND_COUNT {
        let (band_id, lower, upper) = INTENSITY_BANDS[band_index_value];
        let mut band_event_count = 0usize;
        let mut sites_with_events = 0usize;
        let mut totals = [Counts::default(); 4];

        for site_bands in &site_band_counts {
            let site_band = site_bands[band_index_value];
            if site_band.event_count > 0 {
                sites_with_events += 1;
                band_event_count += site_band.event_count;
            }
            for method in 0..4 {
                totals[method].positive += site_band.methods[method].positive;
                totals[method].negative += site_band.methods[method].negative;
                totals[method].indeterminate += site_band.methods[method].indeterminate;
            }
        }

        let mut method_summaries = Vec::with_capacity(4);
        for method_index in 0..4 {
            let counts = totals[method_index];
            let evaluable = counts.evaluable();
            let conditional_probability = (evaluable > 0)
                .then_some(counts.positive as f64 / evaluable as f64);
            let identification_lower = (evaluable + counts.indeterminate > 0)
                .then_some(counts.positive as f64 / (evaluable + counts.indeterminate) as f64);
            let identification_upper = (evaluable + counts.indeterminate > 0)
                .then_some((counts.positive + counts.indeterminate) as f64 / (evaluable + counts.indeterminate) as f64);

            let mut bootstrap_values = Vec::with_capacity(input.bootstrap_replicates);
            for _ in 0..input.bootstrap_replicates {
                let mut replicate = Counts::default();
                for _ in 0..site_ids.len() {
                    let sampled_site = rng.index(site_ids.len());
                    let sampled_counts = site_band_counts[sampled_site][band_index_value].methods[method_index];
                    replicate.positive += sampled_counts.positive;
                    replicate.negative += sampled_counts.negative;
                    replicate.indeterminate += sampled_counts.indeterminate;
                }
                let replicate_evaluable = replicate.evaluable();
                if replicate_evaluable > 0 {
                    bootstrap_values.push(replicate.positive as f64 / replicate_evaluable as f64);
                }
            }
            bootstrap_values.sort_by(f64::total_cmp);
            let bootstrap_ci = if bootstrap_values.len() >= 100 {
                Some([
                    percentile(&bootstrap_values, 0.025).unwrap(),
                    percentile(&bootstrap_values, 0.975).unwrap(),
                ])
            } else {
                None
            };

            method_summaries.push(MethodBandSummary {
                method_id: METHODS[method_index].0.to_owned(),
                event_count: counts.positive + counts.negative + counts.indeterminate,
                site_count: sites_with_events,
                positive_event_count: counts.positive,
                negative_event_count: counts.negative,
                indeterminate_event_count: counts.indeterminate,
                evaluable_event_count: evaluable,
                conditional_probability_among_evaluable_events: conditional_probability,
                identification_lower_bound: identification_lower,
                identification_upper_bound: identification_upper,
                site_cluster_bootstrap_95pct: bootstrap_ci,
                valid_bootstrap_replicates: bootstrap_values.len(),
                interpretation_boundary: "Bootstrap resamples whole sites, retaining within-site event dependence. Interval conditions on method-evaluable events; missing/indeterminate cases have separate identification bounds. This is descriptive, not causal.",
            });
        }

        bands.push(BandSummary {
            band_id: band_id.to_owned(),
            lower_bound_mm_h: if lower.is_finite() { lower } else { 0.0 },
            upper_bound_exclusive_mm_h: upper,
            qualifying_event_count: band_event_count,
            site_count: sites_with_events,
            methods: method_summaries,
        });
    }

    Report {
        audit_version: "0.1.0",
        input_sha256,
        source_sha256: input.source_sha256.clone(),
        study_id: input.study_id.clone(),
        total_input_event_count: input.events.len(),
        eligible_event_count: eligible_count,
        ineligible_event_count: ineligible_count,
        no_soil_moisture_event_count: no_moisture_count,
        unique_site_count: site_ids.len(),
        bootstrap_replicates_requested: input.bootstrap_replicates,
        bootstrap_seed: input.bootstrap_seed,
        bands,
        warnings: vec![
            "Peak rainfall intensity bands are fixed at <5, 5–<12, 12–<13 and >=13 mm/h to inspect the threshold/plateau pattern reported by Li et al. (2025); these are descriptive bins, not causal discontinuities.".to_owned(),
            "The input has one event row per unique site/event. Site is the bootstrap cluster; do not expand sensor, profile, depth or plot rows into independent units.".to_owned(),
            "All bands condition on the >=2 mm event rule and soil-moisture availability used in the 2025 site-prevalence definition.".to_owned(),
            "Indeterminate method events remain in identification bounds and are not silently treated as negative. Cluster-bootstrap intervals condition on evaluable events.".to_owned(),
            "The audit does not adjust for antecedent moisture, texture, vegetation, climate or runoff. It cannot estimate a causal intensity threshold or validate the physical mechanism.".to_owned(),
        ],
        interpretation_boundary: "This is a site-clustered descriptive intensity-regime audit for soil-moisture-derived VT/NSR proxies. It is not a trained random forest, a partial-dependence estimate, a causal response curve or tracer-confirmed preferential flow.",
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: intensity-regime-audit <site-event-drivers.json> <new-report.json>",
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
        "intensity-regime audit complete: events={} eligible={} sites={} bands={} bootstrap_reps={} source_sha256={}",
        report.total_input_event_count,
        report.eligible_event_count,
        report.unique_site_count,
        report.bands.len(),
        report.bootstrap_replicates_requested,
        report.source_sha256
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("intensity-regime audit failed closed: {error}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn event(site: &str, id: &str, intensity: f64, vt: MethodOutcome, nsr: MethodOutcome) -> SiteEvent {
        SiteEvent {
            site_id: site.to_owned(),
            ecoregion_id: "eco".to_owned(),
            event_id: id.to_owned(),
            rainfall_total_mm: 2.0,
            peak_intensity_mm_h: intensity,
            soil_moisture_available: true,
            vt_maximum_ksat: vt,
            vt_median_ksat: vt,
            vt_median_plus_sd_ksat: vt,
            nsr,
            evidence_ref: format!("synthetic-{site}-{id}"),
        }
    }

    fn fixture() -> Input {
        Input {
            schema_version: "0.1.0".to_owned(),
            study_id: "synthetic-intensity-regimes".to_owned(),
            source_sha256: "a".repeat(64),
            source_evidence_ref: "synthetic-only".to_owned(),
            rainfall_minimum_event_mm: 2.0,
            bootstrap_replicates: 500,
            bootstrap_seed: 7,
            events: vec![
                event("S1", "E1", 4.9, MethodOutcome::Negative, MethodOutcome::Negative),
                event("S1", "E2", 5.0, MethodOutcome::Positive, MethodOutcome::Negative),
                event("S2", "E3", 11.9, MethodOutcome::Positive, MethodOutcome::Positive),
                event("S2", "E4", 12.0, MethodOutcome::Negative, MethodOutcome::Negative),
                event("S3", "E5", 12.9, MethodOutcome::Indeterminate, MethodOutcome::Negative),
                event("S3", "E6", 13.0, MethodOutcome::Positive, MethodOutcome::Indeterminate),
            ],
        }
    }

    #[test]
    fn assigns_intensity_boundaries_exactly() {
        assert_eq!(band_index(4.999), 0);
        assert_eq!(band_index(5.0), 1);
        assert_eq!(band_index(11.999), 1);
        assert_eq!(band_index(12.0), 2);
        assert_eq!(band_index(12.999), 2);
        assert_eq!(band_index(13.0), 3);
    }

    #[test]
    fn bootstraps_sites_not_individual_events_and_preserves_missing_bounds() {
        let input = fixture();
        validate(&input).unwrap();
        let report_a = evaluate(&input, "b".repeat(64));
        let report_b = evaluate(&input, "b".repeat(64));
        assert_eq!(report_a.bands.len(), 4);
        assert_eq!(report_a.bands[1].qualifying_event_count, 1);
        assert_eq!(report_a.bands[1].site_count, 1);
        assert_eq!(report_a.bands[1].methods[0].positive_event_count, 1);
        assert_eq!(report_a.bands[1].methods[0].negative_event_count, 0);
        assert_eq!(report_a.bands[2].methods[0].indeterminate_event_count, 1);
        assert_eq!(report_a.bands[2].methods[0].identification_lower_bound, Some(0.0));
        assert_eq!(report_a.bands[2].methods[0].identification_upper_bound, Some(1.0));
        assert_eq!(report_a.bands[3].methods[0].site_cluster_bootstrap_95pct, report_b.bands[3].methods[0].site_cluster_bootstrap_95pct);
    }

    #[test]
    fn rejects_site_assigned_to_multiple_ecoregions() {
        let mut input = fixture();
        input.events[1].ecoregion_id = "other-ecoregion".to_owned();
        assert!(validate(&input).unwrap_err().contains("assigned to multiple ecoregions"));
    }

    #[test]
    fn rejects_duplicate_event_within_site() {
        let mut input = fixture();
        input.events[1].event_id = input.events[0].event_id.clone();
        input.events[1].site_id = input.events[0].site_id.clone();
        assert!(validate(&input).unwrap_err().contains("duplicate rainfall event"));
    }

    #[test]
    fn rejects_outcome_inconsistent_with_event_eligibility() {
        let mut input = fixture();
        input.events[0].rainfall_total_mm = 1.99;
        assert!(validate(&input).unwrap_err().contains("inconsistent"));
    }
}
