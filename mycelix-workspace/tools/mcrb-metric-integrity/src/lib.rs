//! Measurement-integrity gate for the Mycelix Coordination and Recovery Benchmark.
//!
//! Local operation durations are derived from raw monotonic tick pairs and
//! pinned per-host clock identity/epoch/frequency. Caller-supplied duration
//! values are deliberately not accepted as the source of truth.
//!
//! Cross-host publish/observe intervals are calculated from matched event
//! digests and uncertainty-bounded clock calibration. The pure verifier does
//! not authenticate clock sources or event origins; receipt provenance remains separate.

use std::collections::{BTreeMap, BTreeSet};

pub mod attestation;
pub mod cross_host;
pub mod offer_ledger;
pub mod operation_contract;
pub mod qops;
use crate::cross_host::{
    CrossHostLatencyInterval, CrossHostTimingError, PublishObserveEventPair, WallClockCalibration,
    derive_latency_interval,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrityStatus {
    Valid,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossHostTimingStatus {
    NotEvaluated,
    Valid,
    Invalid,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct ExpectedCrossHostPair {
    pub operation_id: String,
    pub record_digest: String,
    pub publisher_peer_id: String,
    pub observer_peer_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MetricIssue {
    InvalidConfiguration(&'static str),
    NoExpectedPeers,
    EmptyExpectedPeerId,
    DuplicateExpectedPeer(String),
    ClockForUnexpectedPeer(String),
    DuplicateClockObservation(String),
    MissingClockObservation(String),
    EmptyClockSource(String),
    EmptyClockSyncMethod(String),
    EmptyMonotonicClockId(String),
    EmptyMonotonicClockEpoch(String),
    InvalidTicksPerSecond(String),
    NonFiniteClockOffset(String),
    InvalidClockUncertainty(String),
    ClockUncertaintyAboveLimit(String),
    NonMonotonicDurationClock(String),
    SampleForUnexpectedPeer(String),
    EmptyOperationId(usize),
    DuplicatePeerOperationSample(String, String),
    MissingPeerSamples(String),
    MissingClockForSample(usize),
    MonotonicClockIdMismatch(usize),
    MonotonicClockEpochMismatch(usize),
    ReversedMonotonicTicks(usize),
    DurationOutsideDeclaredWindow(usize),
    CommittedPublicationsExceedExpected,
    ObservationsExceedExpected,
    PublicationCounterIncomplete,
    ObservationCounterIncomplete,
    NoDurationSamples,
    InvalidExpectedCrossHostPair(usize),
    UnexpectedCrossHostCalibration(String),
    DuplicateCrossHostCalibration(String),
    MissingCrossHostCalibration(String),
    UnexpectedCrossHostEventPair(usize),
    DuplicateCrossHostEventPair(String, String),
    MissingCrossHostEventPair(String, String),
    InvalidCrossHostEventPair(usize, CrossHostTimingError),
    CrossHostPointEstimateWithheld(usize),
}

#[derive(Debug, Clone, PartialEq)]
pub struct MetricConfig {
    /// Largest valid local operation duration for this workload.
    pub maximum_duration_ms: f64,
    /// Durations above this value are preserved and marked as outliers.
    pub outlier_threshold_ms: f64,
    /// Maximum permitted wall-clock synchronization uncertainty per host.
    pub maximum_clock_uncertainty_ms: f64,
    /// Maximum combined uncertainty for emitting a cross-host point estimate.
    pub maximum_cross_host_point_uncertainty_ms: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ClockObservation {
    pub peer_id: String,
    /// Human-readable identifier for the monotonic clock implementation.
    pub clock_source: String,
    /// Wall-clock synchronization method (NTP, PTP, lab reference, etc.).
    pub sync_method: String,
    /// Signed local-wall-clock minus reference-clock offset, in milliseconds.
    pub offset_ms: f64,
    /// Bound on the offset estimate, in milliseconds.
    pub uncertainty_ms: f64,
    /// Must be true; this declares the clock family, not proof of the sample.
    pub monotonic_duration_clock: bool,
    /// Stable clock identity used by the raw tick samples.
    pub monotonic_clock_id: String,
    /// Clock epoch / boot generation; samples cannot cross this boundary.
    pub monotonic_clock_epoch: String,
    /// Monotonic ticks per second for this host and epoch.
    pub ticks_per_second: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DurationSample {
    pub peer_id: String,
    pub operation_id: String,
    pub monotonic_clock_id: String,
    pub monotonic_clock_epoch: String,
    pub start_ticks: u64,
    pub end_ticks: u64,
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
    pub expected_cross_host_pairs: Vec<ExpectedCrossHostPair>,
    pub cross_host_calibrations: Vec<WallClockCalibration>,
    pub cross_host_event_pairs: Vec<PublishObserveEventPair>,
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
    /// Count of all submitted raw tick samples; no samples are removed.
    pub raw_sample_count: usize,
    /// Count of samples whose duration was derived from start/end ticks.
    pub raw_monotonic_tick_derived_count: usize,
    /// Original indexes into MeasurementBatch::duration_samples.
    pub outlier_sample_indexes: Vec<usize>,
    pub issues: Vec<MetricIssue>,
    /// Present only when all integrity gates pass.
    pub percentiles: Option<Percentiles>,
    /// Per-peer distributions prevent a slow peer being hidden by pooling.
    pub per_peer_percentiles: Option<BTreeMap<String, Percentiles>>,
    /// True only when local monotonic timing and the full local batch are valid.
    pub aggregate_eligible: bool,
    /// Separate cross-host state; NotEvaluated does not mean PASS.
    pub cross_host_status: CrossHostTimingStatus,
    pub cross_host_issues: Vec<MetricIssue>,
    pub cross_host_latency_intervals: Vec<CrossHostLatencyInterval>,
    pub cross_host_point_latency_emitted_count: usize,
    pub cross_host_aggregate_eligible: bool,
}

/// Opaque result of validating a raw measurement batch. Callers may inspect
/// its report, but cannot construct a Verified value from public status fields.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedMetricIntegrity {
    report: IntegrityReport,
}

impl VerifiedMetricIntegrity {
    pub fn report(&self) -> &IntegrityReport {
        &self.report
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum MetricIntegrityVerification {
    Verified(VerifiedMetricIntegrity),
    Rejected(IntegrityReport),
}

impl MetricIntegrityVerification {
    pub fn verified(&self) -> Option<&VerifiedMetricIntegrity> {
        match self {
            Self::Verified(value) => Some(value),
            Self::Rejected(_) => None,
        }
    }

    pub fn rejected_report(&self) -> Option<&IntegrityReport> {
        match self {
            Self::Verified(_) => None,
            Self::Rejected(report) => Some(report),
        }
    }
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

fn compute_percentiles(raw_values: &[f64]) -> Option<Percentiles> {
    if raw_values.is_empty()
        || raw_values
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
    {
        return None;
    }

    let mut values = raw_values.to_vec();
    values.sort_by(f64::total_cmp);
    Some(Percentiles {
        p50_ms: nearest_rank(&values, 0.50)?,
        p95_ms: nearest_rank(&values, 0.95)?,
        p99_ms: nearest_rank(&values, 0.99)?,
    })
}

fn is_incomplete_issue(issue: &MetricIssue) -> bool {
    matches!(
        issue,
        MetricIssue::MissingClockObservation(_)
            | MetricIssue::MissingClockForSample(_)
            | MetricIssue::MissingPeerSamples(_)
            | MetricIssue::PublicationCounterIncomplete
            | MetricIssue::ObservationCounterIncomplete
            | MetricIssue::NoDurationSamples
    )
}


fn validate_cross_host_batch(
    config: &MetricConfig,
    batch: &MeasurementBatch,
) -> (
    CrossHostTimingStatus,
    Vec<MetricIssue>,
    Vec<CrossHostLatencyInterval>,
    usize,
    bool,
) {
    let mut issues = Vec::new();
    let mut intervals = Vec::new();
    let mut point_count = 0usize;

    if batch.expected_cross_host_pairs.is_empty() {
        if !batch.cross_host_calibrations.is_empty() || !batch.cross_host_event_pairs.is_empty() {
            for calibration in &batch.cross_host_calibrations {
                push_unique(
                    &mut issues,
                    MetricIssue::UnexpectedCrossHostCalibration(calibration.peer_id.clone()),
                );
            }
            for (index, _) in batch.cross_host_event_pairs.iter().enumerate() {
                push_unique(
                    &mut issues,
                    MetricIssue::UnexpectedCrossHostEventPair(index),
                );
            }
            return (
                CrossHostTimingStatus::Invalid,
                issues,
                intervals,
                point_count,
                false,
            );
        }
        return (
            CrossHostTimingStatus::NotEvaluated,
            issues,
            intervals,
            point_count,
            false,
        );
    }

    if !config.maximum_cross_host_point_uncertainty_ms.is_finite()
        || config.maximum_cross_host_point_uncertainty_ms < 0.0
    {
        push_unique(
            &mut issues,
            MetricIssue::InvalidConfiguration("maximum_cross_host_point_uncertainty_ms"),
        );
    }

    type PairKey = (String, String, String, String);
    let expected_peers: BTreeSet<String> = batch.expected_peers.iter().cloned().collect();
    let mut expected_keys = BTreeSet::<PairKey>::new();
    let mut participants = BTreeSet::<String>::new();

    for (index, pair) in batch.expected_cross_host_pairs.iter().enumerate() {
        let invalid = pair.operation_id.trim().is_empty()
            || pair.record_digest.trim().is_empty()
            || pair.publisher_peer_id.trim().is_empty()
            || pair.observer_peer_id.trim().is_empty()
            || pair.publisher_peer_id == pair.observer_peer_id
            || !expected_peers.contains(&pair.publisher_peer_id)
            || !expected_peers.contains(&pair.observer_peer_id);

        let key = (
            pair.operation_id.clone(),
            pair.record_digest.clone(),
            pair.publisher_peer_id.clone(),
            pair.observer_peer_id.clone(),
        );
        if invalid || !expected_keys.insert(key) {
            push_unique(&mut issues, MetricIssue::InvalidExpectedCrossHostPair(index));
            continue;
        }
        participants.insert(pair.publisher_peer_id.clone());
        participants.insert(pair.observer_peer_id.clone());
    }

    let mut calibrations = BTreeMap::<String, &WallClockCalibration>::new();
    for calibration in &batch.cross_host_calibrations {
        if !participants.contains(&calibration.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::UnexpectedCrossHostCalibration(calibration.peer_id.clone()),
            );
            continue;
        }
        if calibrations.contains_key(&calibration.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::DuplicateCrossHostCalibration(calibration.peer_id.clone()),
            );
            continue;
        }
        calibrations.insert(calibration.peer_id.clone(), calibration);
    }

    for peer_id in &participants {
        if !calibrations.contains_key(peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::MissingCrossHostCalibration(peer_id.clone()),
            );
        }
    }

    let mut event_pairs = BTreeMap::<PairKey, (usize, &PublishObserveEventPair)>::new();
    for (index, pair) in batch.cross_host_event_pairs.iter().enumerate() {
        let key = (
            pair.operation_id.clone(),
            pair.record_digest.clone(),
            pair.publisher_peer_id.clone(),
            pair.observer_peer_id.clone(),
        );
        if !expected_keys.contains(&key) {
            push_unique(
                &mut issues,
                MetricIssue::UnexpectedCrossHostEventPair(index),
            );
            continue;
        }
        if event_pairs.contains_key(&key) {
            push_unique(
                &mut issues,
                MetricIssue::DuplicateCrossHostEventPair(
                    pair.operation_id.clone(),
                    format!("{}->{}", pair.publisher_peer_id, pair.observer_peer_id),
                ),
            );
            continue;
        }
        event_pairs.insert(key, (index, pair));
    }

    for pair in &batch.expected_cross_host_pairs {
        let key = (
            pair.operation_id.clone(),
            pair.record_digest.clone(),
            pair.publisher_peer_id.clone(),
            pair.observer_peer_id.clone(),
        );
        let Some((index, event_pair)) = event_pairs.get(&key).copied() else {
            push_unique(
                &mut issues,
                MetricIssue::MissingCrossHostEventPair(
                    pair.operation_id.clone(),
                    format!("{}->{}", pair.publisher_peer_id, pair.observer_peer_id),
                ),
            );
            continue;
        };

        let (Some(publisher), Some(observer)) = (
            calibrations.get(&pair.publisher_peer_id).copied(),
            calibrations.get(&pair.observer_peer_id).copied(),
        ) else {
            // Missing calibration has already been recorded above.
            continue;
        };

        match derive_latency_interval(
            event_pair,
            publisher,
            observer,
            config.maximum_cross_host_point_uncertainty_ms,
        ) {
            Ok(interval) => {
                if interval.point_latency_ms.is_some() {
                    point_count += 1;
                }
                intervals.push(interval);
            }
            Err(error) => push_unique(
                &mut issues,
                MetricIssue::InvalidCrossHostEventPair(index, error),
            ),
        }
    }

    let has_invalid = issues.iter().any(|issue| {
        !matches!(
            issue,
            MetricIssue::MissingCrossHostCalibration(_)
                | MetricIssue::MissingCrossHostEventPair(_, _)
        )
    });
    let has_incomplete = !issues.is_empty();
    let status = if has_invalid {
        CrossHostTimingStatus::Invalid
    } else if has_incomplete {
        CrossHostTimingStatus::Incomplete
    } else {
        CrossHostTimingStatus::Valid
    };
    let aggregate_eligible =
        status == CrossHostTimingStatus::Valid && intervals.len() == expected_keys.len();

    (
        status,
        issues,
        intervals,
        point_count,
        aggregate_eligible,
    )
}

/// Derive duration using integer arithmetic from a single monotonic clock
/// identity/epoch. It is impossible to derive a negative duration: reversed
/// ticks are returned as a distinct validation failure.
fn derive_duration_ms(
    sample: &DurationSample,
    clock: &ClockObservation,
    sample_index: usize,
    issues: &mut Vec<MetricIssue>,
) -> Option<f64> {
    if sample.monotonic_clock_id != clock.monotonic_clock_id {
        push_unique(
            issues,
            MetricIssue::MonotonicClockIdMismatch(sample_index),
        );
        return None;
    }
    if sample.monotonic_clock_epoch != clock.monotonic_clock_epoch {
        push_unique(
            issues,
            MetricIssue::MonotonicClockEpochMismatch(sample_index),
        );
        return None;
    }
    let Some(delta_ticks) = sample.end_ticks.checked_sub(sample.start_ticks) else {
        push_unique(issues, MetricIssue::ReversedMonotonicTicks(sample_index));
        return None;
    };
    if clock.ticks_per_second == 0 {
        push_unique(
            issues,
            MetricIssue::InvalidTicksPerSecond(clock.peer_id.clone()),
        );
        return None;
    }

    // u64 * 1e9 is bounded well below u128::MAX. Keeping the conversion in
    // integer nanoseconds avoids trusting or rounding a supplied duration.
    let duration_ns =
        (delta_ticks as u128 * 1_000_000_000_u128) / clock.ticks_per_second as u128;
    let duration_ms = duration_ns as f64 / 1_000_000.0;
    if !duration_ms.is_finite() {
        push_unique(
            issues,
            MetricIssue::DurationOutsideDeclaredWindow(sample_index),
        );
        return None;
    }
    Some(duration_ms)
}

/// Validate a measurement batch before deriving aggregate performance metrics.
///
/// Local durations are computed only from raw monotonic tick pairs whose
/// clock ID, epoch and tick frequency match a per-peer ClockObservation.
/// Outliers inside the declared duration window are retained and reported by
/// original sample index. No sample is dropped, clipped, winsorized or
/// silently rewritten.
pub fn validate_batch(config: &MetricConfig, batch: &MeasurementBatch) -> IntegrityReport {
    let mut issues = Vec::new();
    let mut outlier_sample_indexes = Vec::new();

    if !config.maximum_duration_ms.is_finite() || config.maximum_duration_ms <= 0.0 {
        push_unique(
            &mut issues,
            MetricIssue::InvalidConfiguration("maximum_duration_ms"),
        );
    }
    if !config.outlier_threshold_ms.is_finite() || config.outlier_threshold_ms < 0.0 {
        push_unique(
            &mut issues,
            MetricIssue::InvalidConfiguration("outlier_threshold_ms"),
        );
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
    if batch.expected_peers.is_empty() {
        push_unique(&mut issues, MetricIssue::NoExpectedPeers);
    }
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
        if clock.monotonic_clock_id.trim().is_empty() {
            push_unique(
                &mut issues,
                MetricIssue::EmptyMonotonicClockId(clock.peer_id.clone()),
            );
        }
        if clock.monotonic_clock_epoch.trim().is_empty() {
            push_unique(
                &mut issues,
                MetricIssue::EmptyMonotonicClockEpoch(clock.peer_id.clone()),
            );
        }
        if clock.ticks_per_second == 0 {
            push_unique(
                &mut issues,
                MetricIssue::InvalidTicksPerSecond(clock.peer_id.clone()),
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
    let mut operation_ids = BTreeSet::new();
    let mut derived_samples: Vec<(String, f64)> = Vec::new();
    let mut raw_monotonic_tick_derived_count = 0usize;

    for (index, sample) in batch.duration_samples.iter().enumerate() {
        if !expected.contains(&sample.peer_id) {
            push_unique(
                &mut issues,
                MetricIssue::SampleForUnexpectedPeer(sample.peer_id.clone()),
            );
            continue;
        }
        *sample_counts.entry(sample.peer_id.clone()).or_default() += 1;

        if sample.operation_id.trim().is_empty() {
            push_unique(&mut issues, MetricIssue::EmptyOperationId(index));
        } else if !operation_ids.insert((sample.peer_id.clone(), sample.operation_id.clone())) {
            push_unique(
                &mut issues,
                MetricIssue::DuplicatePeerOperationSample(
                    sample.peer_id.clone(),
                    sample.operation_id.clone(),
                ),
            );
        }

        let Some(clock) = clocks.get(&sample.peer_id) else {
            push_unique(&mut issues, MetricIssue::MissingClockForSample(index));
            continue;
        };
        let Some(duration_ms) = derive_duration_ms(sample, clock, index, &mut issues) else {
            continue;
        };
        raw_monotonic_tick_derived_count += 1;
        if duration_ms > config.maximum_duration_ms {
            push_unique(
                &mut issues,
                MetricIssue::DurationOutsideDeclaredWindow(index),
            );
            continue;
        }
        if duration_ms > config.outlier_threshold_ms {
            // Preserve the raw ticks and expose the original sample index.
            outlier_sample_indexes.push(index);
        }
        derived_samples.push((sample.peer_id.clone(), duration_ms));
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

    let has_invalid = issues.iter().any(|issue| !is_incomplete_issue(issue));
    let has_incomplete = !issues.is_empty();

    let status = if has_invalid {
        IntegrityStatus::Invalid
    } else if has_incomplete {
        IntegrityStatus::Incomplete
    } else {
        IntegrityStatus::Valid
    };

    let aggregate_eligible = status == IntegrityStatus::Valid;
    let (percentiles, per_peer_percentiles) = if aggregate_eligible {
        let global_values: Vec<f64> = derived_samples.iter().map(|sample| sample.1).collect();
        let mut peer_values: BTreeMap<String, Vec<f64>> = BTreeMap::new();
        for (peer_id, duration_ms) in &derived_samples {
            peer_values
                .entry(peer_id.clone())
                .or_default()
                .push(*duration_ms);
        }
        let peer_percentiles = peer_values
            .iter()
            .filter_map(|(peer_id, values)| {
                compute_percentiles(values).map(|percentiles| (peer_id.clone(), percentiles))
            })
            .collect();

        (
            compute_percentiles(&global_values),
            Some(peer_percentiles),
        )
    } else {
        (None, None)
    };

    let (
        cross_host_status,
        cross_host_issues,
        cross_host_latency_intervals,
        cross_host_point_latency_emitted_count,
        cross_host_aggregate_eligible,
    ) = validate_cross_host_batch(config, batch);

    IntegrityReport {
        status,
        raw_sample_count: batch.duration_samples.len(),
        raw_monotonic_tick_derived_count,
        outlier_sample_indexes,
        issues,
        percentiles,
        per_peer_percentiles,
        aggregate_eligible,
        cross_host_status,
        cross_host_issues,
        cross_host_latency_intervals,
        cross_host_point_latency_emitted_count,
        cross_host_aggregate_eligible,
    }
}

/// Verify a complete raw measurement batch and return an opaque capability
/// only when the local timing/sample integrity contract passes. Cross-host
/// visibility remains a separately enforced gate when the scenario requires it.
pub fn verify_measurement_batch(
    config: &MetricConfig,
    batch: &MeasurementBatch,
) -> MetricIntegrityVerification {
    let report = validate_batch(config, batch);
    if report.status == IntegrityStatus::Valid
        && report.aggregate_eligible
        && report.issues.is_empty()
    {
        MetricIntegrityVerification::Verified(VerifiedMetricIntegrity { report })
    } else {
        MetricIntegrityVerification::Rejected(report)
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
                maximum_cross_host_point_uncertainty_ms: 10.0,
            },
            MeasurementBatch {
                expected_peers: vec!["peer-a".into(), "peer-b".into()],
                expected_cross_host_pairs: vec![],
                cross_host_calibrations: vec![],
                cross_host_event_pairs: vec![],
                clock_observations: vec![
                    ClockObservation {
                        peer_id: "peer-a".into(),
                        clock_source: "CLOCK_MONOTONIC".into(),
                        sync_method: "chrony/NTP".into(),
                        offset_ms: -0.5,
                        uncertainty_ms: 1.0,
                        monotonic_duration_clock: true,
                        monotonic_clock_id: "clock-a".into(),
                        monotonic_clock_epoch: "boot-a".into(),
                        ticks_per_second: 1_000_000_000,
                    },
                    ClockObservation {
                        peer_id: "peer-b".into(),
                        clock_source: "CLOCK_MONOTONIC".into(),
                        sync_method: "chrony/NTP".into(),
                        offset_ms: 0.75,
                        uncertainty_ms: 2.0,
                        monotonic_duration_clock: true,
                        monotonic_clock_id: "clock-b".into(),
                        monotonic_clock_epoch: "boot-b".into(),
                        ticks_per_second: 1_000_000_000,
                    },
                ],
                duration_samples: vec![
                    DurationSample {
                        peer_id: "peer-a".into(),
                        operation_id: "op-1".into(),
                        monotonic_clock_id: "clock-a".into(),
                        monotonic_clock_epoch: "boot-a".into(),
                        start_ticks: 100_000_000,
                        end_ticks: 110_000_000,
                    },
                    DurationSample {
                        peer_id: "peer-a".into(),
                        operation_id: "op-2".into(),
                        monotonic_clock_id: "clock-a".into(),
                        monotonic_clock_epoch: "boot-a".into(),
                        start_ticks: 110_000_000,
                        end_ticks: 130_000_000,
                    },
                    DurationSample {
                        peer_id: "peer-b".into(),
                        operation_id: "op-3".into(),
                        monotonic_clock_id: "clock-b".into(),
                        monotonic_clock_epoch: "boot-b".into(),
                        start_ticks: 200_000_000,
                        end_ticks: 230_000_000,
                    },
                    DurationSample {
                        peer_id: "peer-b".into(),
                        operation_id: "op-4".into(),
                        monotonic_clock_id: "clock-b".into(),
                        monotonic_clock_epoch: "boot-b".into(),
                        start_ticks: 230_000_000,
                        end_ticks: 270_000_000,
                    },
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
    fn valid_batch_derives_global_and_per_peer_percentiles_from_ticks() {
        let (config, batch) = valid_batch();
        let report = validate_batch(&config, &batch);

        assert_eq!(report.status, IntegrityStatus::Valid);
        assert!(report.aggregate_eligible);
        assert_eq!(report.raw_sample_count, 4);
        assert_eq!(report.raw_monotonic_tick_derived_count, 4);
        assert_eq!(report.outlier_sample_indexes, vec![3]);

        let global = report.percentiles.unwrap();
        assert_eq!(global.p50_ms, 20.0);
        assert_eq!(global.p95_ms, 40.0);
        assert_eq!(global.p99_ms, 40.0);

        let peers = report.per_peer_percentiles.unwrap();
        assert_eq!(peers["peer-a"].p50_ms, 10.0);
        assert_eq!(peers["peer-b"].p50_ms, 30.0);
        assert_eq!(batch.duration_samples[3].start_ticks, 230_000_000);
    }

    #[test]
    fn reversed_ticks_invalidate_aggregates() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].start_ticks = 120_000_000;
        batch.duration_samples[0].end_ticks = 110_000_000;
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(!report.aggregate_eligible);
        assert!(report.issues.contains(&MetricIssue::ReversedMonotonicTicks(0)));
    }

    #[test]
    fn clock_epoch_mismatch_invalidates_sample() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].monotonic_clock_epoch = "different-boot".into();
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::MonotonicClockEpochMismatch(0)));
    }

    #[test]
    fn clock_id_mismatch_invalidates_sample() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].monotonic_clock_id = "other-clock".into();
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::MonotonicClockIdMismatch(0)));
    }

    #[test]
    fn zero_tick_frequency_invalidates_batch() {
        let (config, mut batch) = valid_batch();
        batch.clock_observations[0].ticks_per_second = 0;
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::InvalidTicksPerSecond("peer-a".into())));
    }

    #[test]
    fn derived_duration_outside_window_invalidates_aggregates() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[0].end_ticks = 2_000_000_000;
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::DurationOutsideDeclaredWindow(0)));
        assert!(report.percentiles.is_none());
    }

    #[test]
    fn missing_peer_sample_makes_batch_incomplete() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples.retain(|sample| sample.peer_id != "peer-b");
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Incomplete);
        assert!(!report.aggregate_eligible);
        assert!(report.percentiles.is_none());
        assert!(report
            .issues
            .contains(&MetricIssue::MissingPeerSamples("peer-b".into())));
    }

    #[test]
    fn excessive_clock_uncertainty_blocks_aggregates() {
        let (config, mut batch) = valid_batch();
        batch.clock_observations[1].uncertainty_ms = 9.0;
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::ClockUncertaintyAboveLimit("peer-b".into())));
        assert!(report.percentiles.is_none());
    }

    #[test]
    fn counter_overflow_is_invalid_and_counter_shortfall_is_incomplete() {
        let (config, mut batch) = valid_batch();
        batch.counters.observed_peer_observations = 5;
        let invalid = validate_batch(&config, &batch);
        assert_eq!(invalid.status, IntegrityStatus::Invalid);
        assert!(invalid
            .issues
            .contains(&MetricIssue::ObservationsExceedExpected));

        batch.counters.observed_peer_observations = 3;
        let incomplete = validate_batch(&config, &batch);
        assert_eq!(incomplete.status, IntegrityStatus::Incomplete);
        assert!(incomplete
            .issues
            .contains(&MetricIssue::ObservationCounterIncomplete));
    }

    #[test]
    fn wall_clock_duration_declaration_is_rejected() {
        let (config, mut batch) = valid_batch();
        batch.clock_observations[0].monotonic_duration_clock = false;
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report
            .issues
            .contains(&MetricIssue::NonMonotonicDurationClock("peer-a".into())));
    }

    #[test]
    fn duplicate_operation_ids_are_rejected_per_peer() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[3].operation_id = "op-3".into();
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Invalid);
        assert!(report.issues.contains(
            &MetricIssue::DuplicatePeerOperationSample("peer-b".into(), "op-3".into())
        ));
    }

    #[test]
    fn same_logical_operation_may_be_observed_by_multiple_peers() {
        let (config, mut batch) = valid_batch();
        batch.duration_samples[2].operation_id = "op-1".into();
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Valid);
        assert!(report.aggregate_eligible);
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

    fn valid_cross_host_data() -> (Vec<ExpectedCrossHostPair>, Vec<WallClockCalibration>, Vec<PublishObserveEventPair>) {
        let expected = vec![ExpectedCrossHostPair {
            operation_id: "publish-1".into(),
            record_digest: "sha256:record-1".into(),
            publisher_peer_id: "peer-a".into(),
            observer_peer_id: "peer-b".into(),
        }];
        let calibrations = vec![
            WallClockCalibration {
                peer_id: "peer-a".into(),
                calibration_id: "cal-a".into(),
                offset_ms: 1.0,
                uncertainty_ms: 1.0,
                valid_from_unix_ms: 9_000,
                valid_until_unix_ms: 12_000,
            },
            WallClockCalibration {
                peer_id: "peer-b".into(),
                calibration_id: "cal-b".into(),
                offset_ms: -1.0,
                uncertainty_ms: 1.0,
                valid_from_unix_ms: 9_000,
                valid_until_unix_ms: 12_000,
            },
        ];
        let events = vec![PublishObserveEventPair {
            operation_id: "publish-1".into(),
            record_digest: "sha256:record-1".into(),
            publisher_peer_id: "peer-a".into(),
            observer_peer_id: "peer-b".into(),
            publisher_calibration_id: "cal-a".into(),
            observer_calibration_id: "cal-b".into(),
            published_at_local_unix_ms: 10_000,
            first_valid_observation_at_local_unix_ms: 10_100,
        }];
        (expected, calibrations, events)
    }

    #[test]
    fn cross_host_intervals_are_eligible_only_for_complete_exact_event_pairs() {
        let (config, mut batch) = valid_batch();
        let (expected, calibrations, events) = valid_cross_host_data();
        batch.expected_cross_host_pairs = expected;
        batch.cross_host_calibrations = calibrations;
        batch.cross_host_event_pairs = events;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Valid);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::Valid);
        assert!(report.cross_host_aggregate_eligible);
        assert_eq!(report.cross_host_latency_intervals.len(), 1);
        assert_eq!(report.cross_host_point_latency_emitted_count, 1);
    }

    #[test]
    fn cross_host_missing_observation_is_incomplete_not_zero_latency() {
        let (config, mut batch) = valid_batch();
        let (expected, calibrations, _) = valid_cross_host_data();
        batch.expected_cross_host_pairs = expected;
        batch.cross_host_calibrations = calibrations;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::Incomplete);
        assert!(!report.cross_host_aggregate_eligible);
        assert!(report.cross_host_latency_intervals.is_empty());
        assert!(report.cross_host_issues.iter().any(|issue| {
            matches!(issue, MetricIssue::MissingCrossHostEventPair(_, _))
        }));
    }

    #[test]
    fn cross_host_wrong_record_digest_is_invalid() {
        let (config, mut batch) = valid_batch();
        let (expected, calibrations, mut events) = valid_cross_host_data();
        events[0].record_digest = "sha256:wrong-record".into();
        batch.expected_cross_host_pairs = expected;
        batch.cross_host_calibrations = calibrations;
        batch.cross_host_event_pairs = events;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::Invalid);
        assert!(!report.cross_host_aggregate_eligible);
        assert!(report.cross_host_issues.iter().any(|issue| {
            matches!(issue, MetricIssue::UnexpectedCrossHostEventPair(0))
        }));
    }

    #[test]
    fn cross_host_missing_calibration_is_incomplete() {
        let (config, mut batch) = valid_batch();
        let (expected, calibrations, events) = valid_cross_host_data();
        batch.expected_cross_host_pairs = expected;
        batch.cross_host_calibrations = calibrations.into_iter().take(1).collect();
        batch.cross_host_event_pairs = events;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::Incomplete);
        assert!(!report.cross_host_aggregate_eligible);
        assert!(report.cross_host_issues.iter().any(|issue| {
            matches!(issue, MetricIssue::MissingCrossHostCalibration(peer) if peer == "peer-b")
        }));
    }

    #[test]
    fn cross_host_duplicate_calibration_invalidates_intervals() {
        let (config, mut batch) = valid_batch();
        let (expected, mut calibrations, events) = valid_cross_host_data();
        calibrations.push(calibrations[0].clone());
        batch.expected_cross_host_pairs = expected;
        batch.cross_host_calibrations = calibrations;
        batch.cross_host_event_pairs = events;

        let report = validate_batch(&config, &batch);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::Invalid);
        assert!(!report.cross_host_aggregate_eligible);
        assert!(report.cross_host_issues.iter().any(|issue| {
            matches!(issue, MetricIssue::DuplicateCrossHostCalibration(peer) if peer == "peer-a")
        }));
    }

    #[test]
    fn local_only_batch_keeps_cross_host_explicitly_not_evaluated() {
        let (config, batch) = valid_batch();
        let report = validate_batch(&config, &batch);
        assert_eq!(report.status, IntegrityStatus::Valid);
        assert_eq!(report.cross_host_status, CrossHostTimingStatus::NotEvaluated);
        assert!(!report.cross_host_aggregate_eligible);
    }

}
