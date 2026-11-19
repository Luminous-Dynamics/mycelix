//! Cross-host publish/observe timing with explicit uncertainty intervals.
//!
//! This is a pure calculation primitive. It does not attest the calibration
//! source or prove that an event came from a Holochain conductor. A caller
//! must bind the event pair and calibration records to raw/signed artifacts.

#[derive(Debug, Clone, PartialEq)]
pub struct WallClockCalibration {
    pub peer_id: String,
    pub calibration_id: String,
    // Convention: local wall time = reference wall time + offset_ms.
    pub offset_ms: f64,
    pub uncertainty_ms: f64,
    pub valid_from_unix_ms: u64,
    pub valid_until_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PublishObserveEventPair {
    pub operation_id: String,
    pub record_digest: String,
    pub publisher_peer_id: String,
    pub observer_peer_id: String,
    pub publisher_calibration_id: String,
    pub observer_calibration_id: String,
    pub published_at_local_unix_ms: u64,
    pub first_valid_observation_at_local_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CrossHostLatencyInterval {
    pub operation_id: String,
    pub record_digest: String,
    pub publisher_peer_id: String,
    pub observer_peer_id: String,
    pub lower_bound_ms: f64,
    pub upper_bound_ms: f64,
    pub combined_clock_uncertainty_ms: f64,
    pub point_latency_ms: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CrossHostTimingError {
    EmptyOperationId,
    EmptyRecordDigest,
    SamePublisherAndObserver,
    PublisherCalibrationPeerMismatch,
    ObserverCalibrationPeerMismatch,
    PublisherCalibrationIdMismatch,
    ObserverCalibrationIdMismatch,
    InvalidPublisherCalibration,
    InvalidObserverCalibration,
    PublisherEventOutsideCalibration,
    ObserverEventOutsideCalibration,
    InvalidMaximumUncertainty,
    NonFiniteCorrectedLatency,
    ObservationPrecedesPublicationBeyondUncertainty,
}

fn validate_calibration(
    calibration: &WallClockCalibration,
) -> Result<(), CrossHostTimingError> {
    if calibration.calibration_id.trim().is_empty()
        || calibration.peer_id.trim().is_empty()
        || !calibration.offset_ms.is_finite()
        || !calibration.uncertainty_ms.is_finite()
        || calibration.uncertainty_ms < 0.0
        || calibration.valid_until_unix_ms < calibration.valid_from_unix_ms
    {
        return Err(CrossHostTimingError::InvalidPublisherCalibration);
    }
    Ok(())
}

fn timestamp_within_calibration(timestamp: u64, calibration: &WallClockCalibration) -> bool {
    timestamp >= calibration.valid_from_unix_ms
        && timestamp <= calibration.valid_until_unix_ms
}

/// Derive an interval after correcting each local wall-clock reading by its
/// host calibration. Uncertainties are added conservatively. No point estimate
/// is emitted when combined uncertainty exceeds the configured limit.
pub fn derive_latency_interval(
    pair: &PublishObserveEventPair,
    publisher: &WallClockCalibration,
    observer: &WallClockCalibration,
    maximum_point_uncertainty_ms: f64,
) -> Result<CrossHostLatencyInterval, CrossHostTimingError> {
    if pair.operation_id.trim().is_empty() {
        return Err(CrossHostTimingError::EmptyOperationId);
    }
    if pair.record_digest.trim().is_empty() {
        return Err(CrossHostTimingError::EmptyRecordDigest);
    }
    if pair.publisher_peer_id == pair.observer_peer_id {
        return Err(CrossHostTimingError::SamePublisherAndObserver);
    }
    if !maximum_point_uncertainty_ms.is_finite() || maximum_point_uncertainty_ms < 0.0 {
        return Err(CrossHostTimingError::InvalidMaximumUncertainty);
    }
    if publisher.peer_id != pair.publisher_peer_id {
        return Err(CrossHostTimingError::PublisherCalibrationPeerMismatch);
    }
    if observer.peer_id != pair.observer_peer_id {
        return Err(CrossHostTimingError::ObserverCalibrationPeerMismatch);
    }
    if publisher.calibration_id != pair.publisher_calibration_id {
        return Err(CrossHostTimingError::PublisherCalibrationIdMismatch);
    }
    if observer.calibration_id != pair.observer_calibration_id {
        return Err(CrossHostTimingError::ObserverCalibrationIdMismatch);
    }
    validate_calibration(publisher)?;
    validate_calibration(observer)
        .map_err(|_| CrossHostTimingError::InvalidObserverCalibration)?;

    if !timestamp_within_calibration(pair.published_at_local_unix_ms, publisher) {
        return Err(CrossHostTimingError::PublisherEventOutsideCalibration);
    }
    if !timestamp_within_calibration(
        pair.first_valid_observation_at_local_unix_ms,
        observer,
    ) {
        return Err(CrossHostTimingError::ObserverEventOutsideCalibration);
    }

    // Offset convention: local = reference + offset, so corrected reference
    // time is local - offset. Convert integer wall timestamps only at this
    // boundary; all event/calibration identity checks occur first.
    let corrected_publish =
        pair.published_at_local_unix_ms as f64 - publisher.offset_ms;
    let corrected_observation =
        pair.first_valid_observation_at_local_unix_ms as f64 - observer.offset_ms;
    let center = corrected_observation - corrected_publish;
    let uncertainty = publisher.uncertainty_ms + observer.uncertainty_ms;
    if !center.is_finite() || !uncertainty.is_finite() {
        return Err(CrossHostTimingError::NonFiniteCorrectedLatency);
    }

    let lower_unclamped = center - uncertainty;
    let upper = center + uncertainty;
    if upper < 0.0 {
        return Err(CrossHostTimingError::ObservationPrecedesPublicationBeyondUncertainty);
    }

    // Latency cannot be negative. If the uncertainty lower bound crosses
    // zero, zero is the strongest physically meaningful lower bound.
    let lower = lower_unclamped.max(0.0);
    let point_latency_ms = if center >= 0.0 && uncertainty <= maximum_point_uncertainty_ms {
        Some(center)
    } else {
        None
    };

    Ok(CrossHostLatencyInterval {
        operation_id: pair.operation_id.clone(),
        record_digest: pair.record_digest.clone(),
        publisher_peer_id: pair.publisher_peer_id.clone(),
        observer_peer_id: pair.observer_peer_id.clone(),
        lower_bound_ms: lower,
        upper_bound_ms: upper,
        combined_clock_uncertainty_ms: uncertainty,
        point_latency_ms,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibrations() -> (WallClockCalibration, WallClockCalibration) {
        (
            WallClockCalibration {
                peer_id: "peer-a".into(),
                calibration_id: "cal-a-1".into(),
                offset_ms: 2.0,
                uncertainty_ms: 2.0,
                valid_from_unix_ms: 9_000,
                valid_until_unix_ms: 12_000,
            },
            WallClockCalibration {
                peer_id: "peer-b".into(),
                calibration_id: "cal-b-1".into(),
                offset_ms: -1.0,
                uncertainty_ms: 3.0,
                valid_from_unix_ms: 9_000,
                valid_until_unix_ms: 12_000,
            },
        )
    }

    fn event_pair() -> PublishObserveEventPair {
        PublishObserveEventPair {
            operation_id: "op-42".into(),
            record_digest: "sha256:abcd".into(),
            publisher_peer_id: "peer-a".into(),
            observer_peer_id: "peer-b".into(),
            publisher_calibration_id: "cal-a-1".into(),
            observer_calibration_id: "cal-b-1".into(),
            published_at_local_unix_ms: 10_000,
            first_valid_observation_at_local_unix_ms: 11_001,
        }
    }

    #[test]
    fn applies_both_offsets_and_combines_uncertainty() {
        let (publisher, observer) = calibrations();
        let interval =
            derive_latency_interval(&event_pair(), &publisher, &observer, 10.0).unwrap();
        assert_eq!(interval.point_latency_ms, Some(1_004.0));
        assert_eq!(interval.lower_bound_ms, 999.0);
        assert_eq!(interval.upper_bound_ms, 1_009.0);
        assert_eq!(interval.combined_clock_uncertainty_ms, 5.0);
    }

    #[test]
    fn excessive_uncertainty_preserves_interval_but_withholds_point_estimate() {
        let (publisher, mut observer) = calibrations();
        observer.uncertainty_ms = 20.0;
        let interval =
            derive_latency_interval(&event_pair(), &publisher, &observer, 10.0).unwrap();
        assert!(interval.point_latency_ms.is_none());
        assert_eq!(interval.lower_bound_ms, 982.0);
        assert_eq!(interval.upper_bound_ms, 1_026.0);
    }

    #[test]
    fn rejects_event_outside_calibration_window() {
        let (mut publisher, observer) = calibrations();
        publisher.valid_until_unix_ms = 9_999;
        let error = derive_latency_interval(&event_pair(), &publisher, &observer, 10.0)
            .expect_err("stale publisher calibration must be rejected");
        assert_eq!(error, CrossHostTimingError::PublisherEventOutsideCalibration);
    }

    #[test]
    fn rejects_wrong_peer_or_calibration_identity() {
        let (publisher, observer) = calibrations();
        let mut pair = event_pair();
        pair.observer_peer_id = "peer-c".into();
        assert_eq!(
            derive_latency_interval(&pair, &publisher, &observer, 10.0).unwrap_err(),
            CrossHostTimingError::ObserverCalibrationPeerMismatch
        );

        let mut pair = event_pair();
        pair.publisher_calibration_id = "stale-cal".into();
        assert_eq!(
            derive_latency_interval(&pair, &publisher, &observer, 10.0).unwrap_err(),
            CrossHostTimingError::PublisherCalibrationIdMismatch
        );
    }

    #[test]
    fn rejects_unpaired_or_self_observed_events() {
        let (publisher, observer) = calibrations();
        let mut pair = event_pair();
        pair.record_digest.clear();
        assert_eq!(
            derive_latency_interval(&pair, &publisher, &observer, 10.0).unwrap_err(),
            CrossHostTimingError::EmptyRecordDigest
        );

        let mut pair = event_pair();
        pair.observer_peer_id = pair.publisher_peer_id.clone();
        assert_eq!(
            derive_latency_interval(&pair, &publisher, &observer, 10.0).unwrap_err(),
            CrossHostTimingError::SamePublisherAndObserver
        );
    }

    #[test]
    fn rejects_observation_that_precedes_publication_beyond_uncertainty() {
        let (publisher, mut observer) = calibrations();
        observer.offset_ms = 100.0;
        observer.uncertainty_ms = 1.0;
        let error = derive_latency_interval(&event_pair(), &publisher, &observer, 10.0)
            .expect_err("impossible event order must not become a negative latency");
        assert_eq!(
            error,
            CrossHostTimingError::ObservationPrecedesPublicationBeyondUncertainty
        );
    }

    #[test]
    fn allows_zero_lower_bound_when_uncertainty_overlaps_zero() {
        let (publisher, mut observer) = calibrations();
        observer.offset_ms = 999.0;
        observer.uncertainty_ms = 10.0;
        let mut pair = event_pair();
        pair.first_valid_observation_at_local_unix_ms = 10_000;
        let interval = derive_latency_interval(&pair, &publisher, &observer, 20.0).unwrap();
        assert_eq!(interval.lower_bound_ms, 0.0);
        assert!(interval.upper_bound_ms >= 0.0);
        assert!(interval.point_latency_ms.is_none());
    }
}
