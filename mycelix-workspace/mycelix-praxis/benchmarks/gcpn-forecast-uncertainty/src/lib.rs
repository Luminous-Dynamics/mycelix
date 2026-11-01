//! Paired event-cluster bootstrap for synthetic forecast comparisons.
//!
//! Resampling unit: event_id, for this fixture only. This is not evidence that
//! event IDs are independent weather systems. Real evaluations must supply an
//! independently governed dependence_cluster_id and resample that cluster.

use std::collections::{BTreeMap, BTreeSet};

use gcpn_forecast_calibration::{
    parse_fixture, ForecastRecord, ForecastStatus, ObservationStatus, Partition,
    FIXTURE as FORECAST_FIXTURE,
};

pub const BOOTSTRAP_REPLICATES: u32 = 20_000;
pub const BOOTSTRAP_SEED: u64 = 0x6a09_e667_f3bc_c909;
pub const EXPECTED_SOURCE_FIXTURE_FNV1A64: u64 = 0x81af_0b67_7919_6770;
pub const EXPECTED_EVENT_CLUSTERS: usize = 12;

#[derive(Debug, Clone, PartialEq, Eq)]
struct EventMetadata {
    hazard: String,
    region: String,
    lead_hours: u32,
    event_window_start: u64,
    outcome: Option<bool>,
    observation_status: ObservationStatus,
}
impl From<&ForecastRecord> for EventMetadata {
    fn from(r: &ForecastRecord) -> Self {
        Self {
            hazard: r.hazard.clone(),
            region: r.region.clone(),
            lead_hours: r.lead_hours,
            event_window_start: r.event_window_start,
            outcome: r.outcome,
            observation_status: r.observation_status,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EventScores {
    pub event_id: String,
    pub hazard: String,
    pub lead_hours: u32,
    pub outcome: bool,
    pub candidate_brier: f64,
    pub climatology_brier: f64,
    pub lead_reference_brier: f64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ClusterDiagnostics {
    pub evaluation_event_ids_seen: u32,
    pub complete_event_clusters: u32,
    pub incomplete_event_clusters: u32,
    pub partition_leak_events: u32,
    pub metadata_mismatch_events: u32,
    pub exact_duplicate_rows: u32,
    pub conflicting_duplicate_clusters: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ComparisonInterval {
    pub baseline_model: String,
    /// Candidate Brier loss minus baseline Brier loss; negative favours candidate.
    pub point_mean_difference: f64,
    pub percentile_95_low: f64,
    pub percentile_95_high: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BootstrapReport {
    pub source_record_count: u32,
    pub source_fixture_fingerprint: u64,
    pub event_scores: Vec<EventScores>,
    pub diagnostics: ClusterDiagnostics,
    pub candidate_brier: f64,
    pub climatology_brier: f64,
    pub lead_reference_brier: f64,
    pub seed: u64,
    pub replicates: u32,
    pub candidate_vs_climatology: ComparisonInterval,
    pub candidate_vs_lead_reference: ComparisonInterval,
}

impl BootstrapReport {
    /// Frozen synthetic-fixture integrity oracle; not a statistical significance test.
    pub fn fixture_oracle_matches(&self) -> bool {
        self.source_record_count == 65
            && self.source_fixture_fingerprint == EXPECTED_SOURCE_FIXTURE_FNV1A64
            && self.event_scores.len() == EXPECTED_EVENT_CLUSTERS
            && self.diagnostics.complete_event_clusters == 12
            && self.diagnostics.exact_duplicate_rows == 1
            && self.diagnostics.partition_leak_events == 1
            && self.diagnostics.metadata_mismatch_events == 1
            && self.seed == BOOTSTRAP_SEED
            && self.replicates == BOOTSTRAP_REPLICATES
            && close(self.candidate_brier, 0.17168333333333333)
            && close(self.climatology_brier, 0.25)
            && close(self.lead_reference_brier, 0.04)
            && close(self.candidate_vs_climatology.point_mean_difference, -0.07831666666666666)
            && close(self.candidate_vs_climatology.percentile_95_low, -0.24)
            && close(self.candidate_vs_climatology.percentile_95_high, 0.16420833333333332)
            && close(self.candidate_vs_lead_reference.point_mean_difference, 0.13168333333333335)
            && close(self.candidate_vs_lead_reference.percentile_95_low, -0.03)
            && close(self.candidate_vs_lead_reference.percentile_95_high, 0.3742083333333333)
    }
}

fn close(a: f64, b: f64) -> bool { (a - b).abs() <= 1e-12 }
fn brier(probability: f64, outcome: bool) -> f64 {
    let y = if outcome { 1.0 } else { 0.0 };
    (probability - y).powi(2)
}
fn eligible(r: &ForecastRecord) -> bool {
    r.partition == Partition::Evaluation
        && r.status == ForecastStatus::Valid
        && r.issue_time < r.event_window_start
        && r.observation_status == ObservationStatus::Independent
        && r.outcome.is_some()
        && r.probability.is_some_and(|p| p.is_finite() && (0.0..=1.0).contains(&p))
        && r.reference_probability.is_finite()
        && (0.0..=1.0).contains(&r.reference_probability)
}

pub fn build_event_clusters(
    records: &[ForecastRecord],
) -> (Vec<EventScores>, ClusterDiagnostics) {
    let mut partitions: BTreeMap<String, BTreeSet<Partition>> = BTreeMap::new();
    let mut metadata: BTreeMap<String, EventMetadata> = BTreeMap::new();
    let mut metadata_mismatches = BTreeSet::new();
    let mut by_event: BTreeMap<String, Vec<&ForecastRecord>> = BTreeMap::new();

    for r in records {
        partitions.entry(r.event_id.clone()).or_default().insert(r.partition);
        let current = EventMetadata::from(r);
        if let Some(old) = metadata.get(&r.event_id) {
            if old != &current { metadata_mismatches.insert(r.event_id.clone()); }
        } else {
            metadata.insert(r.event_id.clone(), current);
        }
        if r.partition == Partition::Evaluation {
            by_event.entry(r.event_id.clone()).or_default().push(r);
        }
    }

    let partition_leaks: BTreeSet<String> = partitions.iter()
        .filter_map(|(id, set)| (set.len() > 1).then_some(id.clone()))
        .collect();

    let mut diagnostics = ClusterDiagnostics {
        evaluation_event_ids_seen: by_event.len() as u32,
        partition_leak_events: partition_leaks.len() as u32,
        metadata_mismatch_events: metadata_mismatches.len() as u32,
        ..ClusterDiagnostics::default()
    };
    let mut clusters = Vec::new();

    for (event_id, event_rows) in by_event {
        if partition_leaks.contains(&event_id) || metadata_mismatches.contains(&event_id) {
            continue;
        }

        let mut model_records: BTreeMap<String, &ForecastRecord> = BTreeMap::new();
        let mut conflicting_duplicate = false;

        for r in event_rows {
            if !eligible(r) { continue; }
            if let Some(existing) = model_records.get(&r.model_id) {
                if *existing == r {
                    diagnostics.exact_duplicate_rows += 1;
                } else {
                    conflicting_duplicate = true;
                }
            } else {
                model_records.insert(r.model_id.clone(), r);
            }
        }

        if conflicting_duplicate {
            diagnostics.conflicting_duplicate_clusters += 1;
            continue;
        }

        let required = ["candidate", "climatology", "lead_reference"];
        if required.iter().any(|name| !model_records.contains_key(*name)) {
            diagnostics.incomplete_event_clusters += 1;
            continue;
        }

        let candidate = model_records["candidate"];
        let climatology = model_records["climatology"];
        let lead_reference = model_records["lead_reference"];
        let outcome = candidate.outcome.expect("eligibility requires an outcome");
        let candidate_probability = candidate.probability.expect("eligibility requires a probability");
        let climatology_probability = climatology.probability.expect("eligibility requires a probability");
        let lead_probability = lead_reference.probability.expect("eligibility requires a probability");

        clusters.push(EventScores {
            event_id,
            hazard: candidate.hazard.clone(),
            lead_hours: candidate.lead_hours,
            outcome,
            candidate_brier: brier(candidate_probability, outcome),
            climatology_brier: brier(climatology_probability, outcome),
            lead_reference_brier: brier(lead_probability, outcome),
        });
    }

    diagnostics.complete_event_clusters = clusters.len() as u32;
    (clusters, diagnostics)
}

#[derive(Debug, Clone)]
struct XorShift64Star { state: u64 }
impl XorShift64Star {
    fn new(seed: u64) -> Self {
        Self { state: if seed == 0 { 0x9e37_79b9_7f4a_7c15 } else { seed } }
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn index(&mut self, upper: usize) -> usize {
        (self.next_u64() % upper as u64) as usize
    }
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    let position = p * (sorted.len() - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    sorted[lower] + (sorted[upper] - sorted[lower]) * (position - lower as f64)
}

fn mean(values: impl Iterator<Item = f64>, count: usize) -> f64 {
    values.sum::<f64>() / count as f64
}

pub fn paired_event_bootstrap(
    clusters: &[EventScores],
    seed: u64,
    replicates: u32,
) -> Result<(ComparisonInterval, ComparisonInterval), String> {
    if clusters.len() < 2 { return Err("at least two event clusters are required".into()); }
    if replicates < 100 { return Err("at least 100 bootstrap replicates are required".into()); }

    let n = clusters.len();
    let point_clim = mean(clusters.iter().map(|c| c.candidate_brier - c.climatology_brier), n);
    let point_lead = mean(clusters.iter().map(|c| c.candidate_brier - c.lead_reference_brier), n);
    let mut clim_samples = Vec::with_capacity(replicates as usize);
    let mut lead_samples = Vec::with_capacity(replicates as usize);
    let mut rng = XorShift64Star::new(seed);

    for _ in 0..replicates {
        let mut clim_sum = 0.0;
        let mut lead_sum = 0.0;
        // One draw selects a complete event cluster; all paired model losses
        // travel together. There is no row-wise or model-wise resampling.
        for _ in 0..n {
            let c = &clusters[rng.index(n)];
            clim_sum += c.candidate_brier - c.climatology_brier;
            lead_sum += c.candidate_brier - c.lead_reference_brier;
        }
        clim_samples.push(clim_sum / n as f64);
        lead_samples.push(lead_sum / n as f64);
    }

    clim_samples.sort_by(f64::total_cmp);
    lead_samples.sort_by(f64::total_cmp);
    Ok((
        ComparisonInterval {
            baseline_model: "climatology".into(),
            point_mean_difference: point_clim,
            percentile_95_low: percentile(&clim_samples, 0.025),
            percentile_95_high: percentile(&clim_samples, 0.975),
        },
        ComparisonInterval {
            baseline_model: "lead_reference".into(),
            point_mean_difference: point_lead,
            percentile_95_low: percentile(&lead_samples, 0.025),
            percentile_95_high: percentile(&lead_samples, 0.975),
        },
    ))
}

pub fn analyze_records(records: &[ForecastRecord], seed: u64, replicates: u32) -> Result<BootstrapReport, String> {
    let (event_scores, diagnostics) = build_event_clusters(records);
    let (candidate_vs_climatology, candidate_vs_lead_reference) =
        paired_event_bootstrap(&event_scores, seed, replicates)?;

    let n = event_scores.len();
    let candidate_brier = mean(event_scores.iter().map(|c| c.candidate_brier), n);
    let climatology_brier = mean(event_scores.iter().map(|c| c.climatology_brier), n);
    let lead_reference_brier = mean(event_scores.iter().map(|c| c.lead_reference_brier), n);

    Ok(BootstrapReport {
        source_record_count: records.len() as u32,
        source_fixture_fingerprint: fnv1a64(FORECAST_FIXTURE.as_bytes()),
        event_scores,
        diagnostics,
        candidate_brier,
        climatology_brier,
        lead_reference_brier,
        seed,
        replicates,
        candidate_vs_climatology,
        candidate_vs_lead_reference,
    })
}

pub fn fixture_report() -> Result<BootstrapReport, String> {
    let records = parse_fixture(FORECAST_FIXTURE)?;
    let report = analyze_records(&records, BOOTSTRAP_SEED, BOOTSTRAP_REPLICATES)?;
    if !report.fixture_oracle_matches() {
        return Err("paired event-cluster bootstrap fixture oracle did not reproduce".into());
    }
    Ok(report)
}

pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    fn source_records() -> Vec<ForecastRecord> {
        parse_fixture(FORECAST_FIXTURE).unwrap()
    }

    #[test]
    fn frozen_fixture_yields_twelve_complete_paired_event_clusters() {
        let r = fixture_report().unwrap();
        assert_eq!(r.source_record_count, 65);
        assert_eq!(r.event_scores.len(), 12);
        assert_eq!(r.diagnostics.complete_event_clusters, 12);
        assert_eq!(r.diagnostics.exact_duplicate_rows, 1);
        assert_eq!(r.diagnostics.partition_leak_events, 1);
        assert_eq!(r.diagnostics.metadata_mismatch_events, 1);
        assert!(r.fixture_oracle_matches());
    }

    #[test]
    fn paired_bootstrap_is_deterministic_and_both_intervals_show_uncertainty() {
        let a = fixture_report().unwrap();
        let b = fixture_report().unwrap();
        assert_eq!(a.candidate_vs_climatology, b.candidate_vs_climatology);
        assert_eq!(a.candidate_vs_lead_reference, b.candidate_vs_lead_reference);
        assert!(a.candidate_vs_climatology.percentile_95_low <= 0.0);
        assert!(a.candidate_vs_climatology.percentile_95_high >= 0.0);
        assert!(a.candidate_vs_lead_reference.percentile_95_low <= 0.0);
        assert!(a.candidate_vs_lead_reference.percentile_95_high >= 0.0);
    }

    #[test]
    fn removing_one_model_record_removes_the_whole_paired_event_cluster() {
        let mut records = source_records();
        let index = records.iter().position(|r| {
            r.event_id == "event-heat-72-eval-b" && r.model_id == "candidate"
        }).unwrap();
        records.remove(index);
        let (clusters, diagnostics) = build_event_clusters(&records);
        assert_eq!(clusters.len(), 11);
        assert!(diagnostics.incomplete_event_clusters >= 1);
    }

    #[test]
    fn conflicting_duplicate_model_record_excludes_its_event_cluster() {
        let mut records = source_records();
        let mut duplicate = records.iter().find(|r| {
            r.event_id == "event-heat-72-eval-b" && r.model_id == "candidate"
        }).unwrap().clone();
        duplicate.probability = Some(0.2);
        records.push(duplicate);
        let (clusters, diagnostics) = build_event_clusters(&records);
        assert_eq!(clusters.len(), 11);
        assert_eq!(diagnostics.conflicting_duplicate_clusters, 1);
    }

    #[test]
    fn exact_duplicate_rows_are_deduplicated_before_resampling() {
        let (clusters, diagnostics) = build_event_clusters(&source_records());
        assert_eq!(clusters.len(), 12);
        assert_eq!(diagnostics.exact_duplicate_rows, 1);
    }

    #[test]
    fn paired_draw_count_is_event_cluster_count_not_forecast_row_count() {
        let records = source_records();
        let (clusters, _) = build_event_clusters(&records);
        assert_eq!(clusters.len(), 12);
        assert_eq!(records.iter().filter(|r| r.partition == Partition::Evaluation).count(), 46);
        // Each bootstrap draw indexes one EventScores row, which contains all
        // three model losses and one shared outcome for that event.
        let mut rng = XorShift64Star::new(BOOTSTRAP_SEED);
        let draw: Vec<usize> = (0..clusters.len()).map(|_| rng.index(clusters.len())).collect();
        assert_eq!(draw.len(), 12);
        assert!(draw.iter().all(|i| *i < clusters.len()));
    }

    #[test]
    fn evaluation_partition_and_metadata_controls_are_excluded() {
        let (clusters, d) = build_event_clusters(&source_records());
        assert_eq!(clusters.len(), 12);
        assert_eq!(d.partition_leak_events, 1);
        assert_eq!(d.metadata_mismatch_events, 1);
    }
}
