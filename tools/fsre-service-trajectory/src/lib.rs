use std::fmt;

#[derive(Clone, Debug)]
pub struct Observation {
    pub at_seconds: u64,
    pub service_fraction: f64,
    pub quality_compliant: bool,
}

#[derive(Clone, Debug)]
pub struct ServiceContract {
    pub service_id: String,
    pub horizon_start_seconds: u64,
    pub horizon_end_seconds: u64,
    pub minimum_safe_service_fraction: f64,
    pub max_observation_hold_seconds: u64,
    pub recovery_stability_seconds: u64,
    pub quality_required: bool,
}

#[derive(Clone, Debug)]
pub struct TrajectoryReport {
    pub service_id: String,
    pub horizon_seconds: u64,
    pub known_seconds: u64,
    pub unknown_seconds: u64,
    pub full_service_seconds: u64,
    pub safe_degraded_seconds: u64,
    pub below_minimum_service_seconds: u64,
    pub quality_noncompliant_seconds: u64,
    pub unsafe_service_seconds: u64,
    pub longest_unsafe_episode_seconds: u64,
    pub service_deficit_fraction_seconds: f64,
    pub first_known_breach_at_seconds: Option<u64>,
    pub safe_recovery_at_seconds: Option<u64>,
    pub time_to_safe_recovery_seconds: Option<u64>,
    pub full_recovery_at_seconds: Option<u64>,
    pub time_to_full_recovery_seconds: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum EvaluationError {
    EmptyServiceId,
    InvalidHorizon,
    InvalidServiceFloor,
    ZeroObservationHold,
    ZeroRecoveryStability,
    NoObservations,
    ObservationOutsideHorizon,
    DuplicateTimestamp,
    OutOfOrderTimestamp,
    NonFiniteServiceFraction,
    ServiceFractionOutOfRange,
}

impl fmt::Display for EvaluationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::EmptyServiceId => "service_id must not be empty",
            Self::InvalidHorizon => {
                "horizon_end_seconds must be greater than horizon_start_seconds"
            }
            Self::InvalidServiceFloor => {
                "minimum_safe_service_fraction must be finite and within [0, 1]"
            }
            Self::ZeroObservationHold => "max_observation_hold_seconds must be greater than zero",
            Self::ZeroRecoveryStability => "recovery_stability_seconds must be greater than zero",
            Self::NoObservations => "at least one observation is required",
            Self::ObservationOutsideHorizon => {
                "every observation timestamp must be within the half-open horizon"
            }
            Self::DuplicateTimestamp => "observation timestamps must not be duplicated",
            Self::OutOfOrderTimestamp => "observations must be ordered by timestamp",
            Self::NonFiniteServiceFraction => "service_fraction must be finite",
            Self::ServiceFractionOutOfRange => "service_fraction must be within [0, 1]",
        };
        f.write_str(message)
    }
}

impl std::error::Error for EvaluationError {}

#[derive(Clone, Copy)]
struct Segment {
    start_seconds: u64,
    end_seconds: u64,
    service_fraction: f64,
    quality_compliant: bool,
    known: bool,
}

impl Segment {
    fn duration_seconds(self) -> u64 {
        self.end_seconds - self.start_seconds
    }
}

pub fn evaluate(
    contract: &ServiceContract,
    observations: &[Observation],
) -> Result<TrajectoryReport, EvaluationError> {
    if contract.service_id.trim().is_empty() {
        return Err(EvaluationError::EmptyServiceId);
    }
    if contract.horizon_end_seconds <= contract.horizon_start_seconds {
        return Err(EvaluationError::InvalidHorizon);
    }
    if !contract.minimum_safe_service_fraction.is_finite()
        || !(0.0..=1.0).contains(&contract.minimum_safe_service_fraction)
    {
        return Err(EvaluationError::InvalidServiceFloor);
    }
    if contract.max_observation_hold_seconds == 0 {
        return Err(EvaluationError::ZeroObservationHold);
    }
    if contract.recovery_stability_seconds == 0 {
        return Err(EvaluationError::ZeroRecoveryStability);
    }
    if observations.is_empty() {
        return Err(EvaluationError::NoObservations);
    }

    let mut previous_timestamp = None;
    for observation in observations {
        if observation.at_seconds < contract.horizon_start_seconds
            || observation.at_seconds >= contract.horizon_end_seconds
        {
            return Err(EvaluationError::ObservationOutsideHorizon);
        }
        if let Some(previous) = previous_timestamp {
            if observation.at_seconds == previous {
                return Err(EvaluationError::DuplicateTimestamp);
            }
            if observation.at_seconds < previous {
                return Err(EvaluationError::OutOfOrderTimestamp);
            }
        }
        if !observation.service_fraction.is_finite() {
            return Err(EvaluationError::NonFiniteServiceFraction);
        }
        if !(0.0..=1.0).contains(&observation.service_fraction) {
            return Err(EvaluationError::ServiceFractionOutOfRange);
        }
        previous_timestamp = Some(observation.at_seconds);
    }

    let segments = build_segments(contract, observations);
    let mut report = TrajectoryReport {
        service_id: contract.service_id.clone(),
        horizon_seconds: contract.horizon_end_seconds - contract.horizon_start_seconds,
        known_seconds: 0,
        unknown_seconds: 0,
        full_service_seconds: 0,
        safe_degraded_seconds: 0,
        below_minimum_service_seconds: 0,
        quality_noncompliant_seconds: 0,
        unsafe_service_seconds: 0,
        longest_unsafe_episode_seconds: 0,
        service_deficit_fraction_seconds: 0.0,
        first_known_breach_at_seconds: None,
        safe_recovery_at_seconds: None,
        time_to_safe_recovery_seconds: None,
        full_recovery_at_seconds: None,
        time_to_full_recovery_seconds: None,
    };

    let mut current_unsafe_episode = 0_u64;
    for segment in &segments {
        let duration = segment.duration_seconds();
        if !segment.known {
            report.unknown_seconds += duration;
            current_unsafe_episode = 0;
            continue;
        }

        report.known_seconds += duration;
        report.service_deficit_fraction_seconds +=
            (1.0 - segment.service_fraction) * duration as f64;

        if !segment.quality_compliant {
            report.quality_noncompliant_seconds += duration;
        }
        if segment.service_fraction < contract.minimum_safe_service_fraction {
            report.below_minimum_service_seconds += duration;
        }

        let safe = is_safe(contract, *segment);
        if safe {
            current_unsafe_episode = 0;
            if segment.service_fraction >= 1.0 {
                report.full_service_seconds += duration;
            } else {
                report.safe_degraded_seconds += duration;
            }
        } else {
            report.unsafe_service_seconds += duration;
            current_unsafe_episode += duration;
            report.longest_unsafe_episode_seconds = report
                .longest_unsafe_episode_seconds
                .max(current_unsafe_episode);
            if report.first_known_breach_at_seconds.is_none() {
                report.first_known_breach_at_seconds = Some(segment.start_seconds);
            }
        }
    }

    if let Some(first_breach) = report.first_known_breach_at_seconds {
        report.safe_recovery_at_seconds = first_sustained_start(
            &segments,
            first_breach,
            contract.recovery_stability_seconds,
            |segment| is_safe(contract, *segment),
        );
        report.full_recovery_at_seconds = first_sustained_start(
            &segments,
            first_breach,
            contract.recovery_stability_seconds,
            |segment| is_safe(contract, *segment) && segment.service_fraction >= 1.0,
        );
        report.time_to_safe_recovery_seconds = report
            .safe_recovery_at_seconds
            .map(|recovery| recovery - first_breach);
        report.time_to_full_recovery_seconds = report
            .full_recovery_at_seconds
            .map(|recovery| recovery - first_breach);
    }

    Ok(report)
}

fn is_safe(contract: &ServiceContract, segment: Segment) -> bool {
    segment.service_fraction >= contract.minimum_safe_service_fraction
        && (!contract.quality_required || segment.quality_compliant)
}

fn build_segments(contract: &ServiceContract, observations: &[Observation]) -> Vec<Segment> {
    let mut segments = Vec::with_capacity(observations.len() * 2 + 1);
    let mut cursor = contract.horizon_start_seconds;

    for (index, observation) in observations.iter().enumerate() {
        let next_timestamp = observations
            .get(index + 1)
            .map(|next| next.at_seconds)
            .unwrap_or(contract.horizon_end_seconds);

        if observation.at_seconds > cursor {
            segments.push(unknown_segment(cursor, observation.at_seconds));
        }

        let hold_end = observation
            .at_seconds
            .saturating_add(contract.max_observation_hold_seconds)
            .min(contract.horizon_end_seconds);
        let known_end = next_timestamp.min(hold_end);

        if known_end > observation.at_seconds {
            segments.push(Segment {
                start_seconds: observation.at_seconds,
                end_seconds: known_end,
                service_fraction: observation.service_fraction,
                quality_compliant: observation.quality_compliant,
                known: true,
            });
        }
        if next_timestamp > known_end {
            segments.push(unknown_segment(known_end, next_timestamp));
        }
        cursor = next_timestamp;
    }

    if cursor < contract.horizon_end_seconds {
        segments.push(unknown_segment(cursor, contract.horizon_end_seconds));
    }

    segments
}

fn unknown_segment(start_seconds: u64, end_seconds: u64) -> Segment {
    Segment {
        start_seconds,
        end_seconds,
        service_fraction: 0.0,
        quality_compliant: false,
        known: false,
    }
}

fn first_sustained_start<F>(
    segments: &[Segment],
    after_seconds: u64,
    required_seconds: u64,
    mut qualifies: F,
) -> Option<u64>
where
    F: FnMut(&Segment) -> bool,
{
    let mut streak_start = None;
    let mut streak_seconds = 0_u64;
    let mut previous_end = None;

    for segment in segments {
        if segment.end_seconds <= after_seconds {
            continue;
        }

        let clipped_start = segment.start_seconds.max(after_seconds);
        if clipped_start >= segment.end_seconds {
            continue;
        }

        if !segment.known || !qualifies(segment) {
            streak_start = None;
            streak_seconds = 0;
            previous_end = None;
            continue;
        }

        if previous_end != Some(clipped_start) {
            streak_start = Some(clipped_start);
            streak_seconds = 0;
        }

        streak_seconds += segment.end_seconds - clipped_start;
        if streak_seconds >= required_seconds {
            return streak_start;
        }
        previous_end = Some(segment.end_seconds);
    }

    None
}

#[cfg(test)]
mod tests {
    use super::{evaluate, EvaluationError, Observation, ServiceContract};

    fn observation(at_seconds: u64, service_fraction: f64, quality_compliant: bool) -> Observation {
        Observation {
            at_seconds,
            service_fraction,
            quality_compliant,
        }
    }

    fn contract(horizon_end_seconds: u64) -> ServiceContract {
        ServiceContract {
            service_id: "synthetic-water-service".to_owned(),
            horizon_start_seconds: 0,
            horizon_end_seconds,
            minimum_safe_service_fraction: 0.5,
            max_observation_hold_seconds: 1_000,
            recovery_stability_seconds: 10,
            quality_required: true,
        }
    }

    #[test]
    fn healthy_service_has_no_breach_or_recovery_event() {
        let result = evaluate(&contract(100), &[observation(0, 1.0, true)]).unwrap();
        assert_eq!(result.known_seconds, 100);
        assert_eq!(result.unknown_seconds, 0);
        assert_eq!(result.full_service_seconds, 100);
        assert_eq!(result.first_known_breach_at_seconds, None);
        assert_eq!(result.safe_recovery_at_seconds, None);
    }

    #[test]
    fn safe_degradation_is_separate_from_service_deficit() {
        let observations = [
            observation(0, 1.0, true),
            observation(10, 0.6, true),
            observation(30, 1.0, true),
        ];
        let result = evaluate(&contract(60), &observations).unwrap();
        assert_eq!(result.safe_degraded_seconds, 20);
        assert_eq!(result.below_minimum_service_seconds, 0);
        assert!((result.service_deficit_fraction_seconds - 8.0).abs() < 1e-9);
        assert_eq!(result.first_known_breach_at_seconds, None);
    }

    #[test]
    fn outage_recovery_uses_declared_stability_window() {
        let observations = [
            observation(0, 1.0, true),
            observation(10, 0.2, true),
            observation(30, 1.0, true),
        ];
        let result = evaluate(&contract(60), &observations).unwrap();
        assert_eq!(result.below_minimum_service_seconds, 20);
        assert_eq!(result.unsafe_service_seconds, 20);
        assert_eq!(result.longest_unsafe_episode_seconds, 20);
        assert!((result.service_deficit_fraction_seconds - 16.0).abs() < 1e-9);
        assert_eq!(result.first_known_breach_at_seconds, Some(10));
        assert_eq!(result.safe_recovery_at_seconds, Some(30));
        assert_eq!(result.time_to_safe_recovery_seconds, Some(20));
        assert_eq!(result.full_recovery_at_seconds, Some(30));
    }

    #[test]
    fn required_quality_failure_is_unsafe_even_when_availability_is_above_floor() {
        let observations = [
            observation(0, 1.0, true),
            observation(10, 0.9, false),
            observation(20, 1.0, true),
        ];
        let result = evaluate(&contract(40), &observations).unwrap();
        assert_eq!(result.below_minimum_service_seconds, 0);
        assert_eq!(result.quality_noncompliant_seconds, 10);
        assert_eq!(result.unsafe_service_seconds, 10);
        assert_eq!(result.first_known_breach_at_seconds, Some(10));
        assert_eq!(result.safe_recovery_at_seconds, Some(20));
        assert_eq!(result.full_recovery_at_seconds, Some(20));
    }

    #[test]
    fn stale_observation_tail_is_unknown_not_assumed_healthy_or_failed() {
        let mut service_contract = contract(40);
        service_contract.max_observation_hold_seconds = 10;
        let observations = [observation(0, 1.0, true), observation(30, 0.8, true)];
        let result = evaluate(&service_contract, &observations).unwrap();
        assert_eq!(result.known_seconds, 20);
        assert_eq!(result.unknown_seconds, 20);
        assert_eq!(result.full_service_seconds, 10);
        assert_eq!(result.safe_degraded_seconds, 10);
        assert!((result.service_deficit_fraction_seconds - 2.0).abs() < 1e-9);
        assert_eq!(result.first_known_breach_at_seconds, None);
    }

    #[test]
    fn a_short_recovery_does_not_satisfy_the_stability_window() {
        let observations = [
            observation(0, 1.0, true),
            observation(10, 0.2, true),
            observation(20, 1.0, true),
            observation(25, 0.2, true),
            observation(30, 1.0, true),
        ];
        let result = evaluate(&contract(60), &observations).unwrap();
        assert_eq!(result.first_known_breach_at_seconds, Some(10));
        assert_eq!(result.safe_recovery_at_seconds, Some(30));
        assert_eq!(result.time_to_safe_recovery_seconds, Some(20));
    }

    #[test]
    fn duplicate_observation_timestamps_are_rejected() {
        let observations = [observation(0, 1.0, true), observation(0, 0.5, true)];
        assert_eq!(
            evaluate(&contract(60), &observations).unwrap_err(),
            EvaluationError::DuplicateTimestamp
        );
    }

    #[test]
    fn unknown_prefix_is_not_backfilled_from_the_first_observation() {
        let observations = [observation(20, 0.8, true), observation(40, 1.0, true)];
        let result = evaluate(&contract(60), &observations).unwrap();
        assert_eq!(result.unknown_seconds, 20);
        assert_eq!(result.known_seconds, 40);
        assert_eq!(result.safe_degraded_seconds, 20);
        assert_eq!(result.full_service_seconds, 20);
        assert_eq!(result.first_known_breach_at_seconds, None);
        assert_eq!(
            result.known_seconds + result.unknown_seconds,
            result.horizon_seconds
        );
    }

    #[test]
    fn unknown_gap_resets_recovery_stability_window() {
        let mut service_contract = contract(80);
        service_contract.max_observation_hold_seconds = 10;
        service_contract.recovery_stability_seconds = 15;
        let observations = [
            observation(0, 1.0, true),
            observation(10, 0.2, true),
            observation(20, 1.0, true),
            observation(40, 1.0, true),
            observation(50, 1.0, true),
            observation(60, 1.0, true),
        ];
        let result = evaluate(&service_contract, &observations).unwrap();
        assert_eq!(result.first_known_breach_at_seconds, Some(10));
        assert_eq!(result.safe_recovery_at_seconds, Some(40));
        assert_eq!(result.full_recovery_at_seconds, Some(40));
        assert_eq!(result.time_to_safe_recovery_seconds, Some(30));
        assert_eq!(result.unknown_seconds, 20);
    }

    #[test]
    fn quality_noncompliance_is_reported_but_not_unsafe_when_quality_is_not_required() {
        let mut service_contract = contract(30);
        service_contract.quality_required = false;
        let observations = [observation(0, 0.8, false)];
        let result = evaluate(&service_contract, &observations).unwrap();
        assert_eq!(result.quality_noncompliant_seconds, 30);
        assert_eq!(result.unsafe_service_seconds, 0);
        assert_eq!(result.safe_degraded_seconds, 30);
        assert_eq!(result.first_known_breach_at_seconds, None);
    }

    #[test]
    fn out_of_order_observations_are_rejected() {
        let observations = [observation(20, 1.0, true), observation(10, 1.0, true)];
        assert_eq!(
            evaluate(&contract(60), &observations).unwrap_err(),
            EvaluationError::OutOfOrderTimestamp
        );
    }

    #[test]
    fn unknown_intervals_break_unsafe_episode_contiguity() {
        let mut service_contract = contract(100);
        service_contract.max_observation_hold_seconds = 10;
        let observations = [observation(0, 0.2, true), observation(30, 0.2, true)];
        let result = evaluate(&service_contract, &observations).unwrap();
        assert_eq!(result.unsafe_service_seconds, 20);
        assert_eq!(result.longest_unsafe_episode_seconds, 10);
        assert_eq!(result.unknown_seconds, 80);
        assert_eq!(
            result.full_service_seconds
                + result.safe_degraded_seconds
                + result.unsafe_service_seconds,
            result.known_seconds
        );
        assert_eq!(
            result.known_seconds + result.unknown_seconds,
            result.horizon_seconds
        );
    }


    #[test]
    fn bounded_exhaustive_observation_matrix_preserves_accounting_invariants() {
        let service_fractions = [0.0, 0.5, 1.0];

        // Four observations, each with three service levels and two quality states:
        // 6^4 = 1,296 deterministic input trajectories.
        for case in 0_u64..1_296 {
            let mut encoded = case;
            let mut observations = Vec::with_capacity(4);
            for at_seconds in [0, 2, 4, 6] {
                let digit = (encoded % 6) as usize;
                encoded /= 6;
                observations.push(observation(
                    at_seconds,
                    service_fractions[digit / 2],
                    digit % 2 == 1,
                ));
            }

            // Vary staleness, required-quality policy and recovery stability.
            for max_observation_hold_seconds in 1..=3 {
                for quality_required in [false, true] {
                    for recovery_stability_seconds in 1..=4 {
                        let mut service_contract = contract(8);
                        service_contract.max_observation_hold_seconds =
                            max_observation_hold_seconds;
                        service_contract.quality_required = quality_required;
                        service_contract.recovery_stability_seconds =
                            recovery_stability_seconds;

                        let result = evaluate(&service_contract, &observations).unwrap();
                        assert_eq!(
                            result.known_seconds + result.unknown_seconds,
                            result.horizon_seconds,
                            "time accounting failed for case {case}"
                        );
                        assert_eq!(
                            result.full_service_seconds
                                + result.safe_degraded_seconds
                                + result.unsafe_service_seconds,
                            result.known_seconds,
                            "service-state accounting failed for case {case}"
                        );
                        assert!(
                            result.below_minimum_service_seconds
                                <= result.unsafe_service_seconds,
                            "below-floor service must be unsafe for case {case}"
                        );
                        assert!(
                            result.quality_noncompliant_seconds <= result.known_seconds,
                            "quality exposure exceeds known time for case {case}"
                        );
                        assert!(
                            result.longest_unsafe_episode_seconds
                                <= result.unsafe_service_seconds,
                            "longest unsafe episode exceeds total unsafe time for case {case}"
                        );
                        assert!(
                            result.service_deficit_fraction_seconds >= 0.0
                                && result.service_deficit_fraction_seconds
                                    <= result.known_seconds as f64 + 1e-9,
                            "service deficit outside its mathematical bounds for case {case}"
                        );

                        if let Some(breach) = result.first_known_breach_at_seconds {
                            assert!(breach < result.horizon_seconds);
                            assert!(result.unsafe_service_seconds > 0);
                        }
                        if let Some(recovery) = result.safe_recovery_at_seconds {
                            let breach = result
                                .first_known_breach_at_seconds
                                .expect("recovery requires a prior known breach");
                            assert!(recovery >= breach);
                            assert_eq!(
                                result.time_to_safe_recovery_seconds,
                                Some(recovery - breach)
                            );
                        }
                        if let Some(recovery) = result.full_recovery_at_seconds {
                            let breach = result
                                .first_known_breach_at_seconds
                                .expect("full recovery requires a prior known breach");
                            assert!(recovery >= breach);
                            assert_eq!(
                                result.time_to_full_recovery_seconds,
                                Some(recovery - breach)
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn invalid_service_fraction_is_rejected() {
        let observations = [observation(0, 1.1, true)];
        assert_eq!(
            evaluate(&contract(60), &observations).unwrap_err(),
            EvaluationError::ServiceFractionOutOfRange
        );
    }
}
