//! Calibration and exposure-qualified detection metrics.
//!
//! This module separates score calibration, source coverage and detector
//! performance. A prediction score is not a coverage assertion or evidence
//! of source independence. These metrics are descriptive; clustered
//! uncertainty intervals must use a qualified group-resampling procedure.

use std::collections::BTreeSet;

use serde::Serialize;

#[derive(Debug, Clone, PartialEq)]
pub struct ProbabilityOutcome {
    pub predicted_probability: f64,
    pub outcome: bool,
    /// Steward-defined top-level block retained for cluster-aware inference.
    pub source_group_id: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ReliabilityBin {
    pub lower_bound: f64,
    pub upper_bound: f64,
    pub sample_count: usize,
    pub mean_predicted_probability: f64,
    pub observed_frequency: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CalibrationMetrics {
    pub sample_count: usize,
    pub source_group_count: usize,
    pub brier_score: f64,
    pub log_loss: f64,
    pub prevalence_baseline_brier: f64,
    pub brier_skill_score: Option<f64>,
    /// Equal-width-bin expected calibration error. Descriptive only.
    pub expected_calibration_error: f64,
    pub reliability_bins: Vec<ReliabilityBin>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DetectionTrial {
    pub event_present: bool,
    pub observation_exposed: bool,
    pub detected: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ProbabilityOfDetection {
    pub detected_eligible_events: u64,
    pub eligible_events_with_exposure: u64,
    pub estimate: f64,
    pub wilson_95_lower: f64,
    pub wilson_95_upper: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct FalseAlarmRates {
    pub false_alerts: u64,
    pub false_event_clusters: u64,
    pub eligible_source_milliseconds: u64,
    pub false_alerts_per_source_hour: f64,
    pub false_event_clusters_per_source_hour: f64,
}

/// Compute Brier/log-loss scores and an equal-width reliability diagram.
///
/// Only supplied eligible outcomes are included. Callers must exclude cases
/// with no valid observation exposure and report that excluded exposure
/// separately. source_group_id is counted but does not establish independence.
pub fn score_calibration(
    observations: &[ProbabilityOutcome],
    bin_count: usize,
) -> Result<CalibrationMetrics, String> {
    if observations.is_empty() {
        return Err("calibration.no_eligible_outcomes".into());
    }
    if !(2..=100).contains(&bin_count) {
        return Err("calibration.bin_count_out_of_range".into());
    }

    let mut brier_sum = 0.0;
    let mut log_loss_sum = 0.0;
    let mut positive_count = 0usize;
    let mut group_ids = BTreeSet::new();
    let mut bins = vec![BinAccumulator::default(); bin_count];

    for observation in observations {
        let probability = observation.predicted_probability;
        if !probability.is_finite() || !(0.0..=1.0).contains(&probability) {
            return Err("calibration.probability_out_of_range".into());
        }
        if observation.source_group_id.trim().is_empty() {
            return Err("calibration.source_group_id_missing".into());
        }
        group_ids.insert(observation.source_group_id.as_str());

        let outcome = if observation.outcome { 1.0 } else { 0.0 };
        let error = probability - outcome;
        brier_sum += error * error;
        positive_count += usize::from(observation.outcome);

        // Clamp only for numerical stability of log loss. Brier score keeps
        // the exact supplied probability, including 0 and 1.
        let safe_probability = probability.clamp(f64::EPSILON, 1.0 - f64::EPSILON);
        log_loss_sum += if observation.outcome {
            -safe_probability.ln()
        } else {
            -(1.0 - safe_probability).ln()
        };

        let index = if probability == 1.0 {
            bin_count - 1
        } else {
            ((probability * bin_count as f64).floor() as usize).min(bin_count - 1)
        };
        bins[index].count += 1;
        bins[index].probability_sum += probability;
        bins[index].outcome_sum += outcome;
    }

    let sample_count = observations.len();
    let prevalence = positive_count as f64 / sample_count as f64;
    let baseline_brier = prevalence * (1.0 - prevalence);
    let brier_score = brier_sum / sample_count as f64;
    let brier_skill_score =
        (baseline_brier > 0.0).then_some(1.0 - (brier_score / baseline_brier));

    let mut expected_calibration_error = 0.0;
    let mut reliability_bins = Vec::with_capacity(bin_count);
    for (index, accumulator) in bins.into_iter().enumerate() {
        let lower_bound = index as f64 / bin_count as f64;
        let upper_bound = (index + 1) as f64 / bin_count as f64;
        if accumulator.count == 0 {
            reliability_bins.push(ReliabilityBin {
                lower_bound,
                upper_bound,
                sample_count: 0,
                mean_predicted_probability: 0.0,
                observed_frequency: 0.0,
            });
            continue;
        }
        let count = accumulator.count as f64;
        let mean_predicted = accumulator.probability_sum / count;
        let observed_frequency = accumulator.outcome_sum / count;
        expected_calibration_error += (accumulator.count as f64 / sample_count as f64)
            * (mean_predicted - observed_frequency).abs();
        reliability_bins.push(ReliabilityBin {
            lower_bound,
            upper_bound,
            sample_count: accumulator.count,
            mean_predicted_probability: mean_predicted,
            observed_frequency,
        });
    }

    Ok(CalibrationMetrics {
        sample_count,
        source_group_count: group_ids.len(),
        brier_score,
        log_loss: log_loss_sum / sample_count as f64,
        prevalence_baseline_brier: baseline_brier,
        brier_skill_score,
        expected_calibration_error,
        reliability_bins,
    })
}

/// Estimate Pd only over independently confirmed events during valid source
/// observation exposure. If no eligible event had exposure, returns None
/// (undefined), never zero. Detection without exposure is invalid input.
pub fn probability_of_detection(
    trials: &[DetectionTrial],
) -> Result<Option<ProbabilityOfDetection>, String> {
    let mut eligible = 0u64;
    let mut detected = 0u64;
    for trial in trials {
        if trial.detected && (!trial.event_present || !trial.observation_exposed) {
            return Err("detection.detected_without_eligible_event_exposure".into());
        }
        if trial.event_present && trial.observation_exposed {
            eligible += 1;
            if trial.detected {
                detected += 1;
            }
        }
    }
    if eligible == 0 {
        return Ok(None);
    }
    let estimate = detected as f64 / eligible as f64;
    let (lower, upper) = wilson_interval(detected, eligible, 1.959_963_984_540_054);
    Ok(Some(ProbabilityOfDetection {
        detected_eligible_events: detected,
        eligible_events_with_exposure: eligible,
        estimate,
        wilson_95_lower: lower,
        wilson_95_upper: upper,
    }))
}

/// Compute false alert and false event-cluster rates per eligible source-hour.
/// Milliseconds are used as the input exposure unit. Zero exposure is an
/// error, not a zero false-alarm rate. Event-cluster counts must come from the
/// owner-native event grouping stage.
pub fn false_alarm_rates(
    false_alerts: u64,
    false_event_clusters: u64,
    eligible_source_milliseconds: u64,
) -> Result<FalseAlarmRates, String> {
    if eligible_source_milliseconds == 0 {
        return Err("false_alarm.zero_eligible_source_exposure".into());
    }
    if false_event_clusters > false_alerts {
        return Err("false_alarm.event_clusters_exceed_alerts".into());
    }
    let source_hours = eligible_source_milliseconds as f64 / 3_600_000.0;
    Ok(FalseAlarmRates {
        false_alerts,
        false_event_clusters,
        eligible_source_milliseconds,
        false_alerts_per_source_hour: false_alerts as f64 / source_hours,
        false_event_clusters_per_source_hour: false_event_clusters as f64 / source_hours,
    })
}

#[derive(Debug, Default)]
struct BinAccumulator {
    count: usize,
    probability_sum: f64,
    outcome_sum: f64,
}

fn wilson_interval(successes: u64, trials: u64, z: f64) -> (f64, f64) {
    let n = trials as f64;
    let p = successes as f64 / n;
    let z_squared = z * z;
    let denominator = 1.0 + z_squared / n;
    let center = (p + z_squared / (2.0 * n)) / denominator;
    let half_width = z
        * ((p * (1.0 - p) / n) + (z_squared / (4.0 * n * n))).sqrt()
        / denominator;
    ((center - half_width).max(0.0), (center + half_width).min(1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(probability: f64, outcome: bool, group: &str) -> ProbabilityOutcome {
        ProbabilityOutcome {
            predicted_probability: probability,
            outcome,
            source_group_id: group.to_owned(),
        }
    }

    #[test]
    fn brier_and_log_loss_match_hand_calculated_vector() {
        let observations = [
            observation(0.9, true, "block-a"),
            observation(0.8, true, "block-a"),
            observation(0.2, false, "block-b"),
            observation(0.1, false, "block-b"),
        ];
        let metrics = score_calibration(&observations, 5).unwrap();
        assert!((metrics.brier_score - 0.025).abs() < 1e-12);
        let expected_log_loss =
            (-(0.9_f64).ln() - (0.8_f64).ln() - (0.8_f64).ln() - (0.9_f64).ln()) / 4.0;
        assert!((metrics.log_loss - expected_log_loss).abs() < 1e-12);
        assert_eq!(metrics.prevalence_baseline_brier, 0.25);
        assert!((metrics.brier_skill_score.unwrap() - 0.9).abs() < 1e-12);
        assert_eq!(metrics.source_group_count, 2);
    }

    #[test]
    fn reliability_bins_keep_empty_bins_and_count_probability_one() {
        let observations = [
            observation(0.0, false, "a"),
            observation(0.5, true, "a"),
            observation(1.0, true, "b"),
        ];
        let metrics = score_calibration(&observations, 4).unwrap();
        assert_eq!(metrics.reliability_bins.len(), 4);
        assert_eq!(metrics.reliability_bins[0].sample_count, 1);
        assert_eq!(metrics.reliability_bins[1].sample_count, 0);
        assert_eq!(metrics.reliability_bins[2].sample_count, 1);
        assert_eq!(metrics.reliability_bins[3].sample_count, 1);
    }

    #[test]
    fn constant_outcome_has_no_brier_skill_denominator() {
        let observations = [
            observation(0.1, false, "a"),
            observation(0.2, false, "b"),
        ];
        let metrics = score_calibration(&observations, 2).unwrap();
        assert_eq!(metrics.brier_skill_score, None);
    }

    #[test]
    fn invalid_probability_or_missing_group_is_rejected() {
        let bad_probability = [observation(1.1, true, "a")];
        assert_eq!(
            score_calibration(&bad_probability, 2).unwrap_err(),
            "calibration.probability_out_of_range"
        );
        let missing_group = [observation(0.5, true, " ")];
        assert_eq!(
            score_calibration(&missing_group, 2).unwrap_err(),
            "calibration.source_group_id_missing"
        );
    }

    #[test]
    fn detection_probability_excludes_events_without_observation_exposure() {
        let trials = [
            DetectionTrial {
                event_present: true,
                observation_exposed: true,
                detected: true,
            },
            DetectionTrial {
                event_present: true,
                observation_exposed: true,
                detected: false,
            },
            DetectionTrial {
                event_present: true,
                observation_exposed: false,
                detected: false,
            },
            DetectionTrial {
                event_present: false,
                observation_exposed: true,
                detected: false,
            },
        ];
        let pd = probability_of_detection(&trials).unwrap().unwrap();
        assert_eq!(pd.eligible_events_with_exposure, 2);
        assert_eq!(pd.detected_eligible_events, 1);
        assert_eq!(pd.estimate, 0.5);
        assert!(pd.wilson_95_lower < 0.5);
        assert!(pd.wilson_95_upper > 0.5);
    }

    #[test]
    fn zero_eligible_exposure_makes_pd_undefined() {
        let trials = [DetectionTrial {
            event_present: true,
            observation_exposed: false,
            detected: false,
        }];
        assert_eq!(probability_of_detection(&trials).unwrap(), None);
    }

    #[test]
    fn detection_without_exposure_is_invalid_not_a_hit() {
        let trials = [DetectionTrial {
            event_present: true,
            observation_exposed: false,
            detected: true,
        }];
        assert_eq!(
            probability_of_detection(&trials).unwrap_err(),
            "detection.detected_without_eligible_event_exposure"
        );
    }

    #[test]
    fn false_alarm_rates_use_eligible_source_hour_denominator() {
        let rates = false_alarm_rates(12, 3, 7_200_000).unwrap();
        assert_eq!(rates.false_alerts_per_source_hour, 6.0);
        assert_eq!(rates.false_event_clusters_per_source_hour, 1.5);
    }

    #[test]
    fn false_event_clusters_cannot_exceed_false_alerts() {
        assert_eq!(
            false_alarm_rates(2, 3, 3_600_000).unwrap_err(),
            "false_alarm.event_clusters_exceed_alerts"
        );
    }

    #[test]
    fn false_alarm_rate_is_undefined_without_source_exposure() {
        assert_eq!(
            false_alarm_rates(0, 0, 0).unwrap_err(),
            "false_alarm.zero_eligible_source_exposure"
        );
    }
}
