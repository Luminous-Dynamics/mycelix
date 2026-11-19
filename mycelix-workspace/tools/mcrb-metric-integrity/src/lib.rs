//! Measurement-integrity gate for the Mycelix Coordination and Recovery Benchmark.
//!
//! This crate validates raw timing/peer/counter evidence before any aggregate
//! performance metric can be emitted. It deliberately does not trim outliers.
//! A batch that is invalid or incomplete never receives aggregate percentiles.

use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrityStatus {
    Valid,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MetricIssue {
    InvalidConfiguration(&'static str),
    EmptyExpectedPeerId,
    DuplicateExpectedPeer(String),
    ClockForUnexpectedPeer(String),
    DuplicateClockObservation(String),
    MissingClockObservation(String),
    EmptyClockSource(String),
    EmptyClockSyncMethod(String),
    NonFiniteClockOffset(String),
    InvalidClockUncertainty(String),
    ClockUncertaintyAboveLimit(String),
    NonMonotonicDurationClock(String),
    SampleForUnexpectedPeer(String),
    MissingPeerSamples(String),
    NonFiniteDuration(usize),
    NegativeDuration(usize),
    DurationOutsideDeclaredWindow(usize),
    CommittedPublicationsExceedExpected,
    ObservationsExceedExpected,
    PublicationCounterIncomplete,
    ObservationCounterIncomplete,
    NoDurationSamples,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MetricConfig {
    /// Largest valid local operation duration for this workload.
    pub maximum_duration_ms: f64,
    /// Durations above this value are preserved and marked as outliers.
    pub outlier_threshold_ms: f64,
    /// Maximum permitted clock synchronization uncertainty per host.
    pub maximum_clock_uncertainty_ms: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ClockObservation {
    pub peer_id: String,
    pub clock_source: String,
    pub sync_method: String,
    /// Signed offset is allowed; uncertainty is the bound on that estimate.
    pub offset_ms: f64,
    pub uncertainty_ms: f64,
    /// True only when measured durations use a monotonic clock.
    pub monotonic_duration_clock: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DurationSample {
    pub peer_id: String,
    pub operation_id: String,
    pub duration_ms: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OperationCounters {
    pub expected_publications: u64,
    pub committed_publications: u64,
    pub expected_peer_observations: u64,
    pub observed_peer_observations: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MeasurementBatch {
    pub expected_peers: Vec<String>,
    pub clock_observations: Vec<ClockObservation>,
    pub duration_samples: Vec<DurationSample>,
    pub counters: OperationCounters,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Percentiles {
    pub p50_ms: f64,
    pub p95_ms: f64,
    pub p99_ms: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct IntegrityReport {
    pub status: IntegrityStatus,
    /// Count of all submitted samples; outliers remain included.
    pub raw_sample_count: usize,
    /// Original indexes into MeasurementBatch::duration_samples.
    pub outlier_sample_indexes: Vec<usize>,
    pub issues: Vec<MetricIssue>,
    /// Present only when all integrity gates pass.
    pub percentiles: Option<Percentiles>,
    /// True only when the full batch is valid and complete.
    pub aggregate_eligible: bool,
}

fn push_unique(issues: &mut Vec<MetricIssue>, issue: MetricIssue) {
    if !issues.contains(&issue) {
        issues.push(issue);
    }
}

fn nearest_rank(sorted: &[f64], quantile: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    let rank = (quantile * sorted.len() as f64).ceil() as usize;
    let index = rank.saturating_sub(1).min(sorted.len() - 1);
    sorted.get(index).copied()
}

fn compute_percentiles(samples: &[DurationSample]) -> Option<Percentiles> {
    let mut values: Vec<f64> = samples.iter().map(|sample| sample.duration_ms).collect();
    if values.is_empty() || values.iter().any(|value| !value.is_finite() || *value < 0.0) {
        return None;
    }
    values.sort_by(f64::total_cmp);
    Some(Percentiles {
        p50_ms: nearest_rank(&values, 0.50)?,
        p95_ms: nearest_rank(&values, 0.95)?,
        p99_ms: nearest_rank(&values, 0.99)?,
    })
}

/// Validate a measurement batch before deriving aggregate performance metrics.
///
/// Outliers inside the declared duration window are retained and reported by
/// original sample index. A duration outside the declared window invalidates
/// the batch. No sample is dropped, clipped, winsorized or silently rewritten.
pub fn validate_batch(config: &MetricConfig, batch: &MeasurementBatch) -> IntegrityReport {
    let mut issues = Vec::new();
    let mut outlier_sample_indexes = Vec::new();

    if !config.maximum_duration_ms.is_finite() || config.maximum_duration_ms <= 0.0 {
        push_unique(&mut issues, MetricIssue::InvalidConfiguration("maximum_duration_ms"));
    }
    if !config.outlier_threshold_ms.is_finite() || config.outlier_threshold_ms < 0.0 {
        push_unique(&mut issues, MetricIssue::InvalidConfiguration("outlier_threshold_ms"));
    }
    if !config.maximum_clock_uncertainty_ms.is_finite()
        || config.maximum_clock_uncertainty_ms < 0.0
    {
        push_unique(
            &mut issues,
            MetricIssue::InvalidConfiguration("maximum_clock_uncertainty_ms"),
        );
    }
    if config.outlier_threshold_ms > config.maximum_duration_ms {
        push_unique(
            &mut issues,
            MetricIssue::InvalidConfiguration("outlier_threshold_exceeds_maximum_duration"),
        );
    }

    let mut expected = BTreeSet::new();
    for peer_id in &batch.expected_peers {
        if peer_id.trim().is_empty() {
            push_unique(&mut issues, MetricIssue::EmptyExpectedPeerId);
        } else if !expected.insert(peer_id.clone()) {
            push_unique(
                &mut issues,
                MetricIssue::DuplicateExpectedPeer(peer_id.clone()),
            );
        }
    }

    let mut clocks = BTreeMap::new();
    for clock in &batch.clock_observations {
        if !expected.contains(&clock.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::ClockForUnexpectedPeer(clock.peer_id.clone()),
            );
            continue;
        }
        if clocks.contains_key(&clock.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::DuplicateClockObservation(clock.peer_id.clone()),
            );
            continue;
        }
        clocks.insert(clock.peer_id.clone(), clock);

        if clock.clock_source.trim().is_empty() {
            push_unique(
                &mut issues,
                MetricIssue::EmptyClockSource(clock.peer_id.clone()),
            );
        }
        if clock.sync_method.trim().is_empty() {
            push_unique(
                &mut issues,
                MetricIssue::EmptyClockSyncMethod(clock.peer_id.clone()),
            );
        }
        if !clock.offset_ms.is_finite() {
            push_unique(
                &mut issues,
                MetricIssue::NonFiniteClockOffset(clock.peer_id.clone()),
            );
        }
        if !clock.uncertainty_ms.is_finite() || clock.uncertainty_ms < 0.0 {
            push_unique(
                &mut issues,
                MetricIssue::InvalidClockUncertainty(clock.peer_id.clone()),
            );
        } else if clock.uncertainty_ms > config.maximum_clock_uncertainty_ms {
            push_unique(
                &mut issues,
                MetricIssue::ClockUncertaintyAboveLimit(clock.peer_id.clone()),
            );
        }
        if !clock.monotonic_duration_clock {
            push_unique(
                &mut issues,
                MetricIssue::NonMonotonicDurationClock(clock.peer_id.clone()),
            );
        }
    }

    for peer_id in &expected {
        if !clocks.contains_key(peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::MissingClockObservation(peer_id.clone()),
            );
        }
    }

    let mut sample_counts: BTreeMap<String, usize> = BTreeMap::new();
    for (index, sample) in batch.duration_samples.iter().enumerate() {
        if !expected.contains(&sample.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::SampleForUnexpectedPeer(sample.peer_id.clone()),
            );
            continue;
        }
        *sample_counts.entry(sample.peer_id.clone()).or_default() += 1;

        if !sample.duration_ms.is_finite() {
            push_unique(&mut issues, MetricIssue::NonFiniteDuration(index));
            continue;
        }
        if sample.duration_ms < 0.0 {
            push_unique(&mut issues, MetricIssue::NegativeDuration(index));
            continue;
        }
        if sample.duration_ms > config.maximum_duration_ms {
            push_unique(
                &mut issues,
                MetricIssue::DurationOutsideDeclaredWindow(index),
            );
            continue;
        }
        if sample.duration_ms > config.outlier_threshold_ms {
            // Preserve the raw sample and expose its index; do not trim it.
            outlier_sample_indexes.push(index);
        }
    }

    for peer_id in &expected {
        if sample_counts.get(peer_id).copied().unwrap_or(0) == 0 {
            push_unique(
                &mut issues,
                MetricIssue::MissingPeerSamples(peer_id.clone()),
            );
        }
    }

    let counters = batch.counters;
    if counters.committed_publications > counters.expected_publications {
        push_unique(
            &mut issues,
            MetricIssue::CommittedPublicationsExceedExpected,
        );
    } else if counters.committed_publications < counters.expected_publications {
        push_unique(&mut issues, MetricIssue::PublicationCounterIncomplete);
    }
    if counters.observed_peer_observations > counters.expected_peer_observations {
        push_unique(&mut issues, MetricIssue::ObservationsExceedExpected);
    } else if counters.observed_peer_observations < counters.expected_peer_observations {
        push_unique(&mut issues, MetricIssue::ObservationCounterIncomplete);
    }

    if batch.duration_samples.is_empty() {
        push_unique(&mut issues, MetricIssue::NoDurationSamples);
    }

    let has_invalid = issues.iter().any(|issue| {
        !matches!(
            issue,
            MetricIssue::MissingClockObservation(_)
                | MetricIssue::MissingPeerSamples(_)
                | MetricIssue::PublicationCounterIncomplete
                | MetricIssue::ObservationCounterIncomplete
                | MetricIssue::NoDurationSamples
        )
    });
    let has_incomplete = !issues.is_empty();

    let status = if has_invalid {
        IntegrityStatus::Invalid
    } else if has_incomplete {
        IntegrityStatus::Incomplete
    } else {
        IntegrityStatus::Valid
    };

    let aggregate_eligible = status == IntegrityStatus::Valid;
    let percentiles = if aggregate_eligible {
        compute_percentiles(&batch.duration_samples)
    } else {
        None
    };

    IntegrityReport {
        status,
        raw_sample_count: batch.duration_samples.len(),
        outlier_sample_indexes,
        issues,
        percentiles,
        aggregate_eligible,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_batch() -> (MetricConfig, MeasurementBatch) {
        (
            MetricConfig {
                maximum_duration_ms: 100.0,
                outlier_threshold_ms: 35.0,
                maximum_clock_uncertainty_ms: 5.0,
            },
            MeasurementBatch {
                expected_peers: vec!["peer-a".into(), "peer-b".into()],
                clock_observations: vec![
                    ClockObservation {
                        peer_id: "peer-a".into(),
                        clock_source: "chrony".into(),
                        sync_method: "NTP".into(),
                        offset_ms: -0.5,
                        uncertainty_ms: 1.0,
                        monotonic_duration_clock: true,
                    },
                    ClockObservation {
                        peer_id: "peer-b".into(),
                        clock_source: "chrony".into(),
                        sync_method: "NTP".into(),
                        offset_ms: 0.75,
                        uncertainty_ms: 2.0,
                        monotonic_duration_clock: true,
                    },
                ],
                duration_samples: vec![
                    DurationSample { peer_id: "peer-a".into(), operation_id: "op-1".into(), duration_ms: 10.0 },
                    DurationSample { peer_id: "peer-a".into(), operation_id: "op-2".into(), duration_ms: 20.0 },
                    DurationSample { peer_id: "peer-b".into(), operation_id: "op-3".into(), duration_ms: 30.0 },
                    DurationSample { peer_id: "peer-b".into(), operation_id: "op-4".into(), duration_ms: 40.0 },
                ],
                counters: OperationCounters {
                    expected_publications: 2,
                    committed_publications: 2,
                    expected_peer_observations: 4,
                    observed_peer_observations: 4,
                },
            },
        )
    }

    #[test]
    fn valid_batch_emits_percentiles_and_preserves_outliers() {
        let (config, batch) = valid_batch();
        let report = validate_batch(&config, &batch);

        assert_eq!(report.status, IntegrityStatus::Valid);
        assert!(report.aggregate_eligible);
        assert_eq!(report.raw_sample_count, 4);
        assert_eq!(report.outlier_sample_indexes, vec![3]);
        assert_eq!(report.percentiles.unwrap().p50_ms, 20.0);
        assert_eq!(report.percentiles.unwrap().p95_ms, 40.0);
        assert_eq!(report.percentiles.unwrap().p99_ms, 40.0);
        // The original 40ms sample is still present; it was not trimmed.
        assert_eq!(batch.duration_samples[3].duration_ms, 40.0);
    }

    #[test]
    fn non_finite_and_out_of_window_samples_block_aggregates() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].duration_ms = f64::NAN;
        batch.duration_samples[1].duration_ms = 1_000.0;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(!report.aggregate_eligible);
        assert!(report.percentiles.is_none());
        assert!(report.issues.contains(&MetricIssue::NonFiniteDuration(0)));
        assert!(report.issues.contains(&MetricIssue::DurationOutsideDeclaredWindow(1)));
        assert_eq!(report.raw_sample_count, 4);
    }

    #[test]
    fn negative_duration_is_rejected() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].duration_ms = -1.0;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report.issues.contains(&MetricIssue::NegativeDuration(0)));
    }

    #[test]
    fn missing_peer_sample_makes_batch_incomplete() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples.retain(|sample| sample.peer_id != "peer-b");

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Incomplete);
        assert!(!report.aggregate_eligible);
        assert!(report.percentiles.is_none());
        assert!(report.issues.contains(&MetricIssue::MissingPeerSamples("peer-b".into())));
    }

    #[test]
    fn excessive_clock_uncertainty_blocks_aggregates() {
        let (config, mut batch) = valid_batch();
        batch.clock_observations[1].uncertainty_ms = 9.0;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report.issues.contains(&MetricIssue::ClockUncertaintyAboveLimit("peer-b".into())));
        assert!(report.percentiles.is_none());
    }

    #[test]
    fn counter_overflow_is_invalid_and_counter_shortfall_is_incomplete() {
        let (config, mut batch) = valid_batch();
        batch.counters.observed_peer_observations = 5;
        let invalid = validate_batch(&config, &batch);
        assert_eq!(invalid.status, IntegrityStatus::Invalid);
        assert!(invalid.issues.contains(&MetricIssue::ObservationsExceedExpected));

        batch.counters.observed_peer_observations = 3;
        let incomplete = validate_batch(&config, &batch);
        assert_eq!(incomplete.status, IntegrityStatus::Incomplete);
        assert!(incomplete.issues.contains(&MetricIssue::ObservationCounterIncomplete));
    }

    #[test]
    fn wall_clock_durations_are_not_accepted() {
        let (config, mut batch) = valid_batch();
        batch.clock_observations[0].monotonic_duration_clock = false;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report.issues.contains(&MetricIssue::NonMonotonicDurationClock("peer-a".into())));
    }

    #[test]
    fn empty_batch_cannot_emit_a_performance_claim() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples.clear();

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Incomplete);
        assert!(!report.aggregate_eligible);
        assert!(report.percentiles.is_none());
    }
}
