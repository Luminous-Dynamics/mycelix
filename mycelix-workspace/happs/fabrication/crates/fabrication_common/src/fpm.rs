//! Deterministic Fabrication Process Monitoring (FPM) primitives.
//!
//! This module deliberately separates:
//! - observations: measured sensor values;
//! - detection: deterministic inference from an observation and baseline;
//! - intervention: intentionally out of scope for this pure library;
//! - assessment: intentionally out of scope until a separate, versioned model exists.
//!
//! The implementation is pure: no wall clock, DHT state, host calls, or mutable
//! global state are consulted.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::SensorSnapshot;

pub const FPM_SCHEMA_VERSION: &str = "fpm.v1";
pub const DEFAULT_ROBUST_Z_THRESHOLD: f32 = 3.5;
pub const MIN_BASELINE_SAMPLES: usize = 5;

/// A single multimodal process observation.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ProcessObservation {
    /// Monotonic source sequence number supplied by the acquisition layer.
    pub sequence: u64,
    pub sensor: SensorSnapshot,
}

/// Features currently interpreted by the deterministic FPM detector.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub enum ProcessFeature {
    HotendTemperature,
    BedTemperature,
    ExtruderCurrent,
    VibrationRms,
    FilamentTension,
}

impl ProcessFeature {
    fn required_for_baseline(self) -> bool {
        !matches!(self, Self::FilamentTension)
    }

    fn observed(self, sensor: &SensorSnapshot) -> Option<f32> {
        match self {
            Self::HotendTemperature => Some(sensor.hotend_temp),
            Self::BedTemperature => Some(sensor.bed_temp),
            Self::ExtruderCurrent => Some(sensor.stepper_currents[3]),
            Self::VibrationRms => Some(sensor.vibration_rms),
            Self::FilamentTension => sensor.filament_tension,
        }
    }

    fn anomaly_type(self) -> ProcessAnomalyType {
        match self {
            Self::HotendTemperature | Self::BedTemperature => {
                ProcessAnomalyType::TemperatureDeviation
            },
            Self::ExtruderCurrent => ProcessAnomalyType::ExtrusionInconsistency,
            Self::VibrationRms => ProcessAnomalyType::VibrationAnomaly,
            Self::FilamentTension => ProcessAnomalyType::FilamentSlip,
        }
    }
}

/// FPM-native anomaly classes. These are inference labels, not proof of physical defects.
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub enum ProcessAnomalyType {
    ExtrusionInconsistency,
    TemperatureDeviation,
    VibrationAnomaly,
    FilamentSlip,
}

/// Robust baseline statistics for one feature.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BaselineFeature {
    pub feature: ProcessFeature,
    pub center: f32,
    /// Median absolute deviation (MAD), not a probability or confidence.
    pub median_absolute_deviation: f32,
    pub sample_count: u32,
}

/// Deterministic baseline profile committed to exact source data and detector version.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct BaselineProfile {
    pub schema_version: String,
    pub detector_id: String,
    pub detector_version: String,
    pub source_observation_count: u64,
    /// SHA-256 commitment to the exact ordered source observations.
    pub source_observations_digest: String,
    pub features: Vec<BaselineFeature>,
}

/// Configuration for deterministic anomaly inference.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct DetectorConfig {
    pub detector_id: String,
    pub detector_version: String,
    /// Threshold in robust-z units. This is a decision threshold, not probability.
    pub robust_z_threshold: f32,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            detector_id: "fpm.robust-deviation".to_string(),
            detector_version: "1".to_string(),
            robust_z_threshold: DEFAULT_ROBUST_Z_THRESHOLD,
        }
    }
}

impl DetectorConfig {
    pub fn validate(&self) -> Result<(), FpmError> {
        if self.detector_id.trim().is_empty() {
            return Err(FpmError::InvalidConfiguration(
                "detector_id cannot be empty".to_string(),
            ));
        }
        if self.detector_version.trim().is_empty() {
            return Err(FpmError::InvalidConfiguration(
                "detector_version cannot be empty".to_string(),
            ));
        }
        if !self.robust_z_threshold.is_finite() || self.robust_z_threshold <= 0.0 {
            return Err(FpmError::InvalidConfiguration(
                "robust_z_threshold must be finite and greater than zero".to_string(),
            ));
        }
        Ok(())
    }
}

/// Feature-level evidence emitted by the detector.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct FeatureEvaluation {
    pub feature: ProcessFeature,
    pub observed: f32,
    pub center: f32,
    pub median_absolute_deviation: f32,
    /// None when the baseline has zero dispersion.
    pub robust_z: Option<f32>,
    /// Normalized detector score in [0, 1]. Never a probability.
    pub anomaly_score: f32,
    pub anomalous: bool,
}

/// Provenance tying an inference to its exact inputs and detector identity.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct DetectionProvenance {
    pub schema_version: String,
    pub detector_id: String,
    pub detector_version: String,
    pub baseline_digest: String,
    pub observation_digest: String,
    pub input_digest: String,
}

/// Deterministic anomaly inference. This is not a physical-defect claim.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ProcessDetection {
    pub status: DetectionStatus,
    /// Maximum feature anomaly score in [0, 1].
    pub anomaly_score: f32,
    pub anomaly_types: Vec<ProcessAnomalyType>,
    pub evaluations: Vec<FeatureEvaluation>,
    pub provenance: DetectionProvenance,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionStatus {
    Normal,
    Anomalous,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FpmError {
    InvalidConfiguration(String),
    EmptyBaseline,
    NonMonotonicSequence,
    NonFiniteObservation { sequence: u64 },
    InsufficientBaseline {
        feature: ProcessFeature,
        samples: usize,
        minimum: usize,
    },
    MissingBaselineFeature(ProcessFeature),
    NonFiniteBaseline,
}

impl std::fmt::Display for FpmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidConfiguration(msg) => write!(f, "invalid FPM configuration: {msg}"),
            Self::EmptyBaseline => write!(f, "baseline requires at least one observation"),
            Self::NonMonotonicSequence => {
                write!(f, "baseline observation sequence is not strictly increasing")
            }
            Self::NonFiniteObservation { sequence } => {
                write!(f, "observation {sequence} contains a non-finite sensor value")
            }
            Self::InsufficientBaseline { feature, samples, minimum } => {
                write!(
                    f,
                    "feature {feature:?} has {samples} baseline samples; minimum is {minimum}"
                )
            }
            Self::MissingBaselineFeature(feature) => {
                write!(f, "baseline is missing required feature {feature:?}")
            }
            Self::NonFiniteBaseline => write!(f, "baseline contains a non-finite statistic"),
        }
    }
}

fn validate_sensor(sensor: &SensorSnapshot, sequence: u64) -> Result<(), FpmError> {
    let required = [
        sensor.hotend_temp,
        sensor.bed_temp,
        sensor.stepper_currents[0],
        sensor.stepper_currents[1],
        sensor.stepper_currents[2],
        sensor.stepper_currents[3],
        sensor.vibration_rms,
    ];
    if required.iter().any(|value| !value.is_finite())
        || sensor
            .filament_tension
            .is_some_and(|value| !value.is_finite())
        || sensor.ambient_temp.is_some_and(|value| !value.is_finite())
        || sensor.humidity.is_some_and(|value| !value.is_finite())
    {
        return Err(FpmError::NonFiniteObservation { sequence });
    }
    Ok(())
}

fn median(mut values: Vec<f32>) -> f32 {
    values.sort_by(|a, b| a.total_cmp(b));
    let mid = values.len() / 2;
    if values.len() % 2 == 0 {
        (values[mid - 1] + values[mid]) / 2.0
    } else {
        values[mid]
    }
}

fn feature_baseline(
    feature: ProcessFeature,
    observations: &[ProcessObservation],
) -> Option<BaselineFeature> {
    let values: Vec<f32> = observations
        .iter()
        .filter_map(|observation| feature.observed(&observation.sensor))
        .collect();
    if values.is_empty() {
        return None;
    }
    let center = median(values.clone());
    let deviations: Vec<f32> = values.into_iter().map(|value| (value - center).abs()).collect();
    Some(BaselineFeature {
        feature,
        center,
        median_absolute_deviation: median(deviations),
        sample_count: observations
            .iter()
            .filter(|observation| feature.observed(&observation.sensor).is_some())
            .count() as u32,
    })
}

impl BaselineProfile {
    pub fn from_observations(
        config: &DetectorConfig,
        observations: &[ProcessObservation],
    ) -> Result<Self, FpmError> {
        config.validate()?;
        if observations.is_empty() {
            return Err(FpmError::EmptyBaseline);
        }

        for pair in observations.windows(2) {
            if pair[0].sequence >= pair[1].sequence {
                return Err(FpmError::NonMonotonicSequence);
            }
        }
        for observation in observations {
            validate_sensor(&observation.sensor, observation.sequence)?;
        }

        let mut features = Vec::new();
        for feature in [
            ProcessFeature::HotendTemperature,
            ProcessFeature::BedTemperature,
            ProcessFeature::ExtruderCurrent,
            ProcessFeature::VibrationRms,
            ProcessFeature::FilamentTension,
        ] {
            if let Some(baseline) = feature_baseline(feature, observations) {
                if baseline.median_absolute_deviation.is_finite()
                    && baseline.center.is_finite()
                {
                    features.push(baseline);
                } else {
                    return Err(FpmError::NonFiniteBaseline);
                }
            } else if feature.required_for_baseline() {
                return Err(FpmError::InsufficientBaseline {
                    feature,
                    samples: 0,
                    minimum: MIN_BASELINE_SAMPLES,
                });
            }
        }

        for feature in [
            ProcessFeature::HotendTemperature,
            ProcessFeature::BedTemperature,
            ProcessFeature::ExtruderCurrent,
            ProcessFeature::VibrationRms,
        ] {
            let sample_count = features
                .iter()
                .find(|baseline| baseline.feature == feature)
                .map(|baseline| baseline.sample_count as usize)
                .unwrap_or(0);
            if sample_count < MIN_BASELINE_SAMPLES {
                return Err(FpmError::InsufficientBaseline {
                    feature,
                    samples: sample_count,
                    minimum: MIN_BASELINE_SAMPLES,
                });
            }
        }

        // Optional channels are only admitted into the baseline once they have
        // enough observations to support a stable statistic. Their absence does not
        // invalidate an otherwise sufficient required-channel baseline.
        features.retain(|baseline| {
            baseline.feature.required_for_baseline()
                || baseline.sample_count as usize >= MIN_BASELINE_SAMPLES
        });

        let source_observations_digest = digest_json(observations)?;
        let profile = Self {
            schema_version: FPM_SCHEMA_VERSION.to_string(),
            detector_id: config.detector_id.clone(),
            detector_version: config.detector_version.clone(),
            source_observation_count: observations.len() as u64,
            source_observations_digest,
            features,
        };
        Ok(profile)
    }

    fn feature(&self, feature: ProcessFeature) -> Option<&BaselineFeature> {
        self.features.iter().find(|baseline| baseline.feature == feature)
    }

    pub fn digest(&self) -> Result<String, FpmError> {
        digest_json(self)
    }
}

fn digest_json<T: Serialize>(value: &T) -> Result<String, FpmError> {
    let bytes = serde_json::to_vec(value)
        .map_err(|error| {
            FpmError::InvalidConfiguration(format!(
                "failed to serialize FPM input: {error}"
            ))
        })?;
    Ok(hex_digest(&bytes))
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    let mut output = String::with_capacity(digest.len() * 2);
    for byte in digest {
        output.push_str(&format!("{byte:02x}"));
    }
    output
}

pub fn analyze(
    config: &DetectorConfig,
    baseline: &BaselineProfile,
    observation: &ProcessObservation,
) -> Result<ProcessDetection, FpmError> {
    config.validate()?;
    validate_sensor(&observation.sensor, observation.sequence)?;

    if baseline.schema_version != FPM_SCHEMA_VERSION {
        return Err(FpmError::InvalidConfiguration(format!(
            "baseline schema version {} is incompatible with {}",
            baseline.schema_version, FPM_SCHEMA_VERSION
        )));
    }
    if baseline.detector_id != config.detector_id {
        return Err(FpmError::InvalidConfiguration(format!(
            "baseline detector id {} does not match config {}",
            baseline.detector_id, config.detector_id
        )));
    }
    if baseline.detector_version != config.detector_version {
        return Err(FpmError::InvalidConfiguration(format!(
            "baseline detector version {} does not match config {}",
            baseline.detector_version, config.detector_version
        )));
    }
    if baseline.source_observation_count == 0 || baseline.source_observations_digest.is_empty() {
        return Err(FpmError::InvalidConfiguration(
            "baseline source observation commitment is missing".to_string(),
        ));
    }

    let baseline_digest = baseline.digest()?;
    let observation_digest = digest_json(observation)?;
    let input_digest = digest_json(&(
        FPM_SCHEMA_VERSION,
        config,
        baseline,
        observation,
    ))?;

    let mut evaluations = Vec::new();
    let mut anomaly_types = Vec::new();
    let mut anomaly_score = 0.0f32;

    for feature in [
        ProcessFeature::HotendTemperature,
        ProcessFeature::BedTemperature,
        ProcessFeature::ExtruderCurrent,
        ProcessFeature::VibrationRms,
        ProcessFeature::FilamentTension,
    ] {
        let Some(observed) = feature.observed(&observation.sensor) else {
            continue;
        };
        let Some(baseline_feature) = baseline.feature(feature) else {
            if feature.required_for_baseline() {
                return Err(FpmError::MissingBaselineFeature(feature));
            }
            continue;
        };

        let delta = (observed - baseline_feature.center).abs();
        let (robust_z, score, anomalous) = if baseline_feature.median_absolute_deviation > 0.0 {
            // Divide before multiplying by the scale constant so finite but
            // extreme MAD values cannot overflow into an artificial zero z-score.
            let raw_z = (delta / baseline_feature.median_absolute_deviation) / 1.4826;
            // Preserve a finite evidence value even for extreme ratios that
            // overflow f32. The normalized anomaly score remains bounded.
            let z = if raw_z.is_finite() { raw_z } else { f32::MAX };
            let score = (z / config.robust_z_threshold).clamp(0.0, 1.0);
            (Some(z), score, z >= config.robust_z_threshold)
        } else {
            let anomalous = delta > 0.0;
            let score = if anomalous { 1.0 } else { 0.0 };
            (None, score, anomalous)
        };

        let evaluation = FeatureEvaluation {
            feature,
            observed,
            center: baseline_feature.center,
            median_absolute_deviation: baseline_feature.median_absolute_deviation,
            robust_z,
            anomaly_score: score,
            anomalous,
        };
        if anomalous {
            let anomaly_type = feature.anomaly_type();
            if !anomaly_types.contains(&anomaly_type) {
                anomaly_types.push(anomaly_type);
            }
        }
        anomaly_score = anomaly_score.max(score);
        evaluations.push(evaluation);
    }

    // Feature traversal above is fixed and deterministic, so anomaly_types
    // already have a stable order. Avoid depending on Debug formatting for
    // semantic ordering.
    Ok(ProcessDetection {
        status: if anomaly_types.is_empty() {
            DetectionStatus::Normal
        } else {
            DetectionStatus::Anomalous
        },
        anomaly_score,
        anomaly_types,
        evaluations,
        provenance: DetectionProvenance {
            schema_version: FPM_SCHEMA_VERSION.to_string(),
            detector_id: config.detector_id.clone(),
            detector_version: config.detector_version.clone(),
            baseline_digest,
            observation_digest,
            input_digest,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sensor(
        hotend: f32,
        bed: f32,
        extruder: f32,
        vibration: f32,
        filament: Option<f32>,
    ) -> SensorSnapshot {
        SensorSnapshot {
            hotend_temp: hotend,
            bed_temp: bed,
            stepper_currents: [0.8, 0.8, 1.0, extruder],
            vibration_rms: vibration,
            filament_tension: filament,
            ambient_temp: Some(22.0),
            humidity: Some(40.0),
        }
    }

    fn observation(sequence: u64, hotend: f32) -> ProcessObservation {
        ProcessObservation {
            sequence,
            sensor: sensor(hotend, 60.0, 1.0, 0.05, None),
        }
    }

    fn baseline() -> BaselineProfile {
        let observations = (0..10)
            .map(|sequence| observation(sequence, 210.0 + (sequence as f32 % 2.0) * 0.2))
            .collect::<Vec<_>>();
        BaselineProfile::from_observations(&DetectorConfig::default(), &observations)
            .expect("valid baseline")
    }

    #[test]
    fn baseline_requires_observations() {
        assert_eq!(
            BaselineProfile::from_observations(&DetectorConfig::default(), &[]),
            Err(FpmError::EmptyBaseline)
        );
    }

    #[test]
    fn baseline_requires_monotonic_sequences() {
        let observations = vec![observation(2, 210.0), observation(1, 210.0)];
        assert_eq!(
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations),
            Err(FpmError::NonMonotonicSequence)
        );
    }

    #[test]
    fn baseline_rejects_nonfinite_sensor_values() {
        let observations = vec![observation(0, f32::NAN)];
        assert_eq!(
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations),
            Err(FpmError::NonFiniteObservation { sequence: 0 })
        );
    }

    #[test]
    fn baseline_requires_minimum_required_feature_samples() {
        let observations = (0..MIN_BASELINE_SAMPLES - 1)
            .map(|sequence| observation(sequence as u64, 210.0))
            .collect::<Vec<_>>();
        assert!(matches!(
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations),
            Err(FpmError::InsufficientBaseline {
                feature: ProcessFeature::HotendTemperature,
                ..
            })
        ));
    }

    #[test]
    fn baseline_commits_exact_source_observations() {
        let config = DetectorConfig::default();
        let mut observations = (0..10)
            .map(|sequence| observation(sequence, 210.0))
            .collect::<Vec<_>>();
        let first = BaselineProfile::from_observations(&config, &observations).expect("baseline");
        observations[9] = observation(9, 211.0);
        let second = BaselineProfile::from_observations(&config, &observations).expect("baseline");
        assert_ne!(
            first.source_observations_digest,
            second.source_observations_digest
        );
    }

    #[test]
    fn baseline_is_deterministic() {
        let first = baseline();
        let second = baseline();
        assert_eq!(first, second);
        assert_eq!(first.digest().expect("digest"), second.digest().expect("digest"));
    }

    #[test]
    fn normal_observation_produces_normal_detection() {
        let config = DetectorConfig::default();
        let result = analyze(&config, &baseline(), &observation(100, 210.1)).expect("analysis");
        assert_eq!(result.status, DetectionStatus::Normal);
        assert!(result.anomaly_types.is_empty());
        assert!(result.anomaly_score < 1.0);
    }

    #[test]
    fn outlier_produces_temperature_anomaly() {
        let config = DetectorConfig::default();
        let result = analyze(&config, &baseline(), &observation(100, 250.0)).expect("analysis");
        assert_eq!(result.status, DetectionStatus::Anomalous);
        assert!(result
            .anomaly_types
            .contains(&ProcessAnomalyType::TemperatureDeviation));
        assert!(result
            .evaluations
            .iter()
            .any(|evaluation| evaluation.feature == ProcessFeature::HotendTemperature
                && evaluation.anomalous));
    }

    #[test]
    fn optional_filament_tension_needs_a_stable_baseline() {
        let observations = (0..10)
            .map(|sequence| ProcessObservation {
                sequence,
                sensor: sensor(
                    210.0,
                    60.0,
                    1.0,
                    0.05,
                    if sequence == 0 { Some(100.0) } else { None },
                ),
            })
            .collect::<Vec<_>>();
        let profile =
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations)
                .expect("required baseline");
        assert!(profile.feature(ProcessFeature::FilamentTension).is_none());
    }

    #[test]
    fn extreme_deviation_keeps_robust_z_finite() {
        let mut observations = Vec::new();
        for sequence in 0..10 {
            observations.push(ProcessObservation {
                sequence,
                sensor: sensor(210.0 + sequence as f32 * 1e-20, 60.0, 1.0, 0.05, None),
            });
        }
        let profile =
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations)
                .expect("baseline");
        let result = analyze(
            &DetectorConfig::default(),
            &profile,
            &observation(100, 250.0),
        )
        .expect("analysis");
        let evaluation = result
            .evaluations
            .iter()
            .find(|item| item.feature == ProcessFeature::HotendTemperature)
            .expect("temperature evaluation");
        assert!(evaluation.robust_z.is_some_and(|z| z.is_finite()));
    }

    #[test]
    fn optional_filament_tension_is_analyzed_only_when_present() {
        let observations = (0..10)
            .map(|sequence| ProcessObservation {
                sequence,
                sensor: sensor(210.0, 60.0, 1.0, 0.05, Some(100.0)),
            })
            .collect::<Vec<_>>();
        let baseline =
            BaselineProfile::from_observations(&DetectorConfig::default(), &observations)
                .expect("baseline");

        let observation = ProcessObservation {
            sequence: 100,
            sensor: sensor(210.0, 60.0, 1.0, 0.05, Some(500.0)),
        };
        let result =
            analyze(&DetectorConfig::default(), &baseline, &observation).expect("analysis");
        assert_eq!(result.status, DetectionStatus::Anomalous);
        assert!(result.anomaly_types.contains(&ProcessAnomalyType::FilamentSlip));
    }

    #[test]
    fn nonfinite_observation_is_rejected_closed() {
        let mut bad = observation(100, 210.0);
        bad.sensor.vibration_rms = f32::INFINITY;
        assert_eq!(
            analyze(&DetectorConfig::default(), &baseline(), &bad),
            Err(FpmError::NonFiniteObservation { sequence: 100 })
        );
    }

    #[test]
    fn detector_id_must_match_baseline() {
        let mut config = DetectorConfig::default();
        config.detector_id = "different-detector".to_string();
        assert!(matches!(
            analyze(&config, &baseline(), &observation(100, 210.0)),
            Err(FpmError::InvalidConfiguration(_))
        ));
    }

    #[test]
    fn detector_version_must_match_baseline() {
        let mut config = DetectorConfig::default();
        config.detector_version = "2".to_string();
        assert!(matches!(
            analyze(&config, &baseline(), &observation(100, 210.0)),
            Err(FpmError::InvalidConfiguration(_))
        ));
    }

    #[test]
    fn changed_observation_changes_input_digest() {
        let config = DetectorConfig::default();
        let first = analyze(&config, &baseline(), &observation(100, 210.0)).expect("analysis");
        let second = analyze(&config, &baseline(), &observation(100, 210.2)).expect("analysis");
        assert_ne!(
            first.provenance.input_digest,
            second.provenance.input_digest
        );
    }

    #[test]
    fn detection_does_not_emit_confidence_or_probability() {
        let json = serde_json::to_string(
            &analyze(&DetectorConfig::default(), &baseline(), &observation(100, 210.0))
                .expect("analysis"),
        )
        .expect("serialize");
        assert!(!json.contains("confidence"));
        assert!(!json.contains("probability"));
    }
}
