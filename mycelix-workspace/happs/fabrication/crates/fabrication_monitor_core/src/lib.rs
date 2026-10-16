//! Deterministic, Holochain-independent process-monitoring core.
//!
//! Detects deviations from a reference trace bound to one exact design,
//! recipe, equipment, material, environment and calibration profile. It does not infer part conformity, defect
//! probability, or safe machine actions. Hardware safety authority stays elsewhere.
//!
//! Thresholds and unit-specific noise floors are supplied by a versioned profile;
//! this crate intentionally provides no universal factory defaults.
#![forbid(unsafe_code)]

use std::fmt;

/// Number of independently monitored scalar signals in v1.
pub const SIGNAL_COUNT: usize = 10;

/// Exact identity binding for a process-specific reference trace.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProcessProfile {
    /// Immutable design revision identifier or digest.
    pub design_revision: String,
    /// Immutable process recipe revision identifier or digest.
    pub recipe_revision: String,
    /// Exact machine identity and relevant configuration revision.
    pub equipment_profile: String,
    /// Material grade/condition profile used for this run.
    pub material_profile: String,
    /// Declared environment profile for this baseline's applicability.
    pub environment_profile: String,
    /// Calibration and measurement-system profile identifier.
    pub calibration_profile: String,
}

/// One normalized observation at a specific point in a monitoring session.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorFrame {
    /// Monitoring-session identity.
    pub session_id: String,
    /// Strictly increasing frame sequence within the session.
    pub sequence: u64,
    /// Source timestamp in milliseconds, strictly increasing within the session.
    pub timestamp_ms: u64,
    /// Exact design, recipe, equipment, material, environment and calibration identity.
    pub profile: ProcessProfile,
    /// Hotend temperature in degrees Celsius.
    pub hotend_temp_c: f64,
    /// Build-plate temperature in degrees Celsius.
    pub bed_temp_c: f64,
    /// Stepper motor currents [X, Y, Z, extruder] in milliamperes.
    pub stepper_current_ma: [f64; 4],
    /// Vibration RMS in units of g.
    pub vibration_rms_g: f64,
    /// Optional filament tension in grams.
    pub filament_tension_g: Option<f64>,
    /// Optional ambient temperature in degrees Celsius.
    pub ambient_temp_c: Option<f64>,
    /// Optional relative humidity percentage in the range 0-100.
    pub humidity_pct: Option<f64>,
}

/// Signals understood by the v1 detector, in canonical metric-array order.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Signal {
    /// Hotend temperature.
    HotendTemperature,
    /// Bed temperature.
    BedTemperature,
    /// X stepper current.
    XCurrent,
    /// Y stepper current.
    YCurrent,
    /// Z stepper current.
    ZCurrent,
    /// Extruder stepper current.
    ExtruderCurrent,
    /// Vibration RMS.
    VibrationRms,
    /// Optional filament tension.
    FilamentTension,
    /// Optional ambient temperature.
    AmbientTemperature,
    /// Optional relative humidity percentage.
    Humidity,
}

impl Signal {
    /// All v1 signals in stable metric-array order.
    pub const ALL: [Signal; SIGNAL_COUNT] = [
        Signal::HotendTemperature,
        Signal::BedTemperature,
        Signal::XCurrent,
        Signal::YCurrent,
        Signal::ZCurrent,
        Signal::ExtruderCurrent,
        Signal::VibrationRms,
        Signal::FilamentTension,
        Signal::AmbientTemperature,
        Signal::Humidity,
    ];

    /// Stable index into the detector's minimum-scale array.
    pub const fn index(self) -> usize {
        match self {
            Signal::HotendTemperature => 0,
            Signal::BedTemperature => 1,
            Signal::XCurrent => 2,
            Signal::YCurrent => 3,
            Signal::ZCurrent => 4,
            Signal::ExtruderCurrent => 5,
            Signal::VibrationRms => 6,
            Signal::FilamentTension => 7,
            Signal::AmbientTemperature => 8,
            Signal::Humidity => 9,
        }
    }

    const fn is_optional(self) -> bool {
        matches!(
            self,
            Signal::FilamentTension | Signal::AmbientTemperature | Signal::Humidity
        )
    }
}

/// Versioned detector configuration for one qualified operating profile.
///
/// Minimum scales use each signal's native units. They prevent division by
/// zero; they are not quality limits and must be tuned for the actual process
/// and measurement system being evaluated.
#[derive(Debug, Clone, PartialEq)]
pub struct DetectorConfig {
    /// Stable detector identifier.
    pub detector_id: String,
    /// Detector implementation version.
    pub detector_version: String,
    /// Immutable exact threshold/noise-floor configuration identifier or digest.
    pub configuration_id: String,
    /// Exact healthy-reference trace/corpus identifier, preferably a content digest.
    pub reference_dataset_id: String,
    /// Robust-z threshold at which a deviation is reported.
    pub robust_z_threshold: f64,
    /// Unit-specific minimum scales, indexed by Signal::index().
    pub minimum_scale_by_signal: [f64; SIGNAL_COUNT],
    /// Minimum number of healthy reference observations required per signal.
    pub minimum_baseline_samples: usize,
    /// Maximum tolerated timestamp interval between adjacent frames.
    pub maximum_sample_gap_ms: u64,
}

impl DetectorConfig {
    /// Validate all profile-controlled parameters.
    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.detector_id.trim().is_empty() {
            return Err(ConfigError::EmptyDetectorId);
        }
        if self.detector_version.trim().is_empty() {
            return Err(ConfigError::EmptyDetectorVersion);
        }
        if self.configuration_id.trim().is_empty() {
            return Err(ConfigError::EmptyConfigurationId);
        }
        if self.reference_dataset_id.trim().is_empty() {
            return Err(ConfigError::EmptyReferenceDatasetId);
        }
        if !self.robust_z_threshold.is_finite() || self.robust_z_threshold <= 0.0 {
            return Err(ConfigError::InvalidThreshold);
        }
        if self.minimum_baseline_samples < 3 {
            return Err(ConfigError::TooFewRequiredBaselineSamples);
        }
        if self.maximum_sample_gap_ms == 0 {
            return Err(ConfigError::ZeroMaximumSampleGap);
        }
        for signal in Signal::ALL {
            let scale = self.minimum_scale_by_signal[signal.index()];
            if !scale.is_finite() || scale <= 0.0 {
                return Err(ConfigError::InvalidMinimumScale(signal));
            }
        }
        Ok(())
    }
}

/// Configuration validation failure.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ConfigError {
    /// Detector ID was empty.
    EmptyDetectorId,
    /// Detector version was empty.
    EmptyDetectorVersion,
    /// Exact detector-configuration identity was empty.
    EmptyConfigurationId,
    /// Exact reference dataset identity was empty.
    EmptyReferenceDatasetId,
    /// Robust-z threshold was non-finite or non-positive.
    InvalidThreshold,
    /// Required minimum baseline size was below three.
    TooFewRequiredBaselineSamples,
    /// Maximum permitted sample interval was zero.
    ZeroMaximumSampleGap,
    /// A signal's minimum scale was non-finite or non-positive.
    InvalidMinimumScale(Signal),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "invalid detector configuration: {self:?}")
    }
}
impl std::error::Error for ConfigError {}

/// Failure to construct a reference baseline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BaselineError {
    /// One or more process-profile identity fields were empty.
    EmptyProcessProfileIdentity,
    /// Detector profile was invalid.
    InvalidConfig(ConfigError),
    /// The reference set was too small.
    InsufficientSamples { found: usize, required: usize },
    /// A reference frame used another design, recipe, equipment, material,
    /// environment or calibration profile.
    ProcessProfileMismatch { sample_index: usize },
    /// A session sequence or timestamp identity was duplicated in the reference set.
    DuplicateReferenceObservation { sample_index: usize },
    /// A reference frame had invalid input.
    InvalidSample { sample_index: usize, reason: InputError },
    /// A mandatory signal was unavailable in the reference set.
    MissingRequiredSignal(Signal),
}

impl fmt::Display for BaselineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "cannot fit reference baseline: {self:?}")
    }
}
impl std::error::Error for BaselineError {}

/// Invalid observation detail.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum InputError {
    /// Session ID was empty.
    EmptySessionId,
    /// A mandatory signal was not finite.
    NonFinite(Signal),
    /// A non-negative physical signal had a negative value.
    Negative(Signal),
    /// Relative humidity was outside its supported 0-100 percentage range.
    OutOfRange(Signal),
}

/// Robust location and scale estimated from a healthy reference trace.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct SignalBaseline {
    /// Median of reference values.
    pub median: f64,
    /// max(1.4826 * MAD, configured minimum scale).
    pub robust_scale: f64,
    /// Number of reference observations for this signal.
    pub sample_count: usize,
}

/// A reference model bound to exact process and measurement identities plus detector config.
#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceBaseline {
    /// Exact profile represented by this baseline.
    pub process_profile: ProcessProfile,
    /// Detector and threshold configuration used with this baseline.
    pub detector_config: DetectorConfig,
    /// Per-signal robust statistics; filament tension may be uncalibrated.
    pub signals: [Option<SignalBaseline>; SIGNAL_COUNT],
    /// Number of input frames used to fit the baseline.
    pub reference_frame_count: usize,
}

impl ReferenceBaseline {
    /// Fit a deterministic baseline from explicitly selected healthy reference frames.
    ///
    /// Input order does not affect fitted statistics. All frames must share the
    /// exact same design, recipe, equipment, material, environment and calibration identity.
    pub fn fit(
        process_profile: ProcessProfile,
        detector_config: DetectorConfig,
        healthy_frames: &[SensorFrame],
    ) -> Result<Self, BaselineError> {
        validate_process_profile(&process_profile)?;
        detector_config.validate().map_err(BaselineError::InvalidConfig)?;

        if healthy_frames.len() < detector_config.minimum_baseline_samples {
            return Err(BaselineError::InsufficientSamples {
                found: healthy_frames.len(),
                required: detector_config.minimum_baseline_samples,
            });
        }

        let mut seen_sequences = std::collections::BTreeSet::new();
        let mut seen_timestamps = std::collections::BTreeSet::new();

        for (sample_index, frame) in healthy_frames.iter().enumerate() {
            if frame.profile != process_profile {
                return Err(BaselineError::ProcessProfileMismatch { sample_index });
            }
            validate_frame(frame).map_err(|reason| BaselineError::InvalidSample {
                sample_index,
                reason,
            })?;

            let sequence_key = (frame.session_id.clone(), frame.sequence);
            let timestamp_key = (frame.session_id.clone(), frame.timestamp_ms);
            if !seen_sequences.insert(sequence_key) || !seen_timestamps.insert(timestamp_key) {
                return Err(BaselineError::DuplicateReferenceObservation { sample_index });
            }
        }

        let signals: [Option<SignalBaseline>; SIGNAL_COUNT] = std::array::from_fn(|index| {
            let signal = Signal::ALL[index];
            let mut values: Vec<f64> = healthy_frames
                .iter()
                .filter_map(|frame| signal_value(frame, signal))
                .collect();

            if values.len() < detector_config.minimum_baseline_samples {
                return None;
            }

            let center = median(&mut values);
            let mut absolute_deviations: Vec<f64> =
                values.iter().map(|value| (value - center).abs()).collect();
            let mad = median(&mut absolute_deviations);
            let robust_scale = (1.4826 * mad)
                .max(detector_config.minimum_scale_by_signal[signal.index()]);

            Some(SignalBaseline {
                median: center,
                robust_scale,
                sample_count: values.len(),
            })
        });

        for signal in Signal::ALL {
            if !signal.is_optional() && signals[signal.index()].is_none() {
                return Err(BaselineError::MissingRequiredSignal(signal));
            }
        }

        Ok(Self {
            process_profile,
            detector_config,
            signals,
            reference_frame_count: healthy_frames.len(),
        })
    }
}

/// Classification of an observed deviation; these are not physical-defect labels.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DeviationKind {
    /// Hotend or bed temperature differs from the healthy reference distribution.
    TemperatureDeviation,
    /// X/Y/Z stepper current differs from the healthy reference distribution.
    MotorCurrentDeviation,
    /// Extruder stepper current differs from the healthy reference distribution.
    ExtrusionCurrentDeviation,
    /// Vibration RMS differs from the healthy reference distribution.
    VibrationDeviation,
    /// Filament tension differs from the healthy reference distribution.
    FilamentTensionDeviation,
    /// Ambient relative humidity differs from the healthy reference distribution.
    HumidityDeviation,
}

/// One deviation from the fitted healthy reference distribution.
#[derive(Debug, Clone, PartialEq)]
pub struct DetectedDeviation {
    /// Signal that departed from its reference distribution.
    pub signal: Signal,
    /// Rule-based deviation category, not a diagnosed physical defect.
    pub kind: DeviationKind,
    /// Current observation in native signal units.
    pub observed_value: f64,
    /// Reference median in native signal units.
    pub baseline_median: f64,
    /// Absolute robust z-score using the fitted robust scale.
    pub robust_z_score: f64,
    /// Profile-configured robust-z threshold.
    pub threshold: f64,
}

/// Why an observation was or was not evaluated.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationDisposition {
    /// Every available signal had a baseline and passed input/ordering checks.
    Evaluated,
    /// Available signals were evaluated, but a present signal lacked baseline coverage.
    EvaluatedWithUncalibratedSignals,
    /// Incoming profile did not exactly match design, recipe and calibration identity.
    ProcessProfileMismatch,
    /// Incoming session ID differed from the engine's bound session.
    SessionMismatch,
    /// Frame input contained invalid values.
    InvalidObservation(InputError),
    /// Sequence did not increase strictly.
    NonMonotonicSequence,
    /// Source timestamp did not increase strictly.
    NonMonotonicTimestamp,
    /// Sequence positions were missing or timestamp gap exceeded the configured limit.
    DataGap { missing_sequences: u64, elapsed_ms: u64 },
}

/// Complete deterministic output for one observation.
#[derive(Debug, Clone, PartialEq)]
pub struct ObservationReport {
    /// Immutable exact threshold/noise-floor configuration identity used for this report.
    pub configuration_id: String,
    /// Exact healthy-reference dataset or corpus identifier used by the baseline.
    pub reference_dataset_id: String,
    /// Session identity to which this report belongs.
    pub session_id: String,
    /// Incoming observation sequence.
    pub sequence: u64,
    /// Incoming source timestamp in milliseconds.
    pub timestamp_ms: u64,
    /// Exact process profile observed.
    pub process_profile: ProcessProfile,
    /// Detector identity.
    pub detector_id: String,
    /// Detector version.
    pub detector_version: String,
    /// Number of reference frames used to fit the baseline.
    pub reference_frame_count: usize,
    /// Evaluation disposition.
    pub disposition: ObservationDisposition,
    /// Deviations found in calibrated signals.
    pub deviations: Vec<DetectedDeviation>,
    /// Present signals that could not be compared with this baseline.
    pub uncalibrated_signals: Vec<Signal>,
}

/// Session-bound advisory detector. It has no I/O or machine-control interface.
#[derive(Debug, Clone)]
pub struct MonitoringEngine {
    baseline: ReferenceBaseline,
    session_id: String,
    last_position: Option<(u64, u64)>,
}

impl MonitoringEngine {
    /// Bind a fitted baseline to one live monitoring session.
    pub fn new(
        baseline: ReferenceBaseline,
        session_id: impl Into<String>,
    ) -> Result<Self, EngineError> {
        let session_id = session_id.into();
        if session_id.trim().is_empty() {
            return Err(EngineError::EmptySessionId);
        }
        Ok(Self {
            baseline,
            session_id,
            last_position: None,
        })
    }

    /// Evaluate one frame without external I/O or hidden state.
    ///
    /// Duplicate/reordered observations are rejected. A sequence gap or oversized
    /// timestamp gap is reported as unknown and is not scored.
    pub fn observe(&mut self, frame: &SensorFrame) -> ObservationReport {
        if frame.profile != self.baseline.process_profile {
            return self.report(
                frame,
                ObservationDisposition::ProcessProfileMismatch,
                vec![],
                vec![],
            );
        }
        if frame.session_id != self.session_id {
            return self.report(frame, ObservationDisposition::SessionMismatch, vec![], vec![]);
        }
        if let Err(reason) = validate_frame(frame) {
            return self.report(
                frame,
                ObservationDisposition::InvalidObservation(reason),
                vec![],
                vec![],
            );
        }

        if let Some((last_sequence, last_timestamp_ms)) = self.last_position {
            if frame.sequence <= last_sequence {
                return self.report(
                    frame,
                    ObservationDisposition::NonMonotonicSequence,
                    vec![],
                    vec![],
                );
            }
            if frame.timestamp_ms <= last_timestamp_ms {
                return self.report(
                    frame,
                    ObservationDisposition::NonMonotonicTimestamp,
                    vec![],
                    vec![],
                );
            }

            let missing_sequences = frame.sequence.saturating_sub(last_sequence).saturating_sub(1);
            let elapsed_ms = frame.timestamp_ms - last_timestamp_ms;
            if missing_sequences > 0
                || elapsed_ms > self.baseline.detector_config.maximum_sample_gap_ms
            {
                self.last_position = Some((frame.sequence, frame.timestamp_ms));
                return self.report(
                    frame,
                    ObservationDisposition::DataGap { missing_sequences, elapsed_ms },
                    vec![],
                    vec![],
                );
            }
        }

        self.last_position = Some((frame.sequence, frame.timestamp_ms));
        let mut deviations = Vec::new();
        let mut uncalibrated_signals = Vec::new();

        for signal in Signal::ALL {
            let Some(observed_value) = signal_value(frame, signal) else {
                continue;
            };
            let Some(reference) = self.baseline.signals[signal.index()] else {
                uncalibrated_signals.push(signal);
                continue;
            };

            let robust_z_score = (observed_value - reference.median).abs() / reference.robust_scale;
            if robust_z_score > self.baseline.detector_config.robust_z_threshold {
                deviations.push(DetectedDeviation {
                    signal,
                    kind: deviation_kind(signal),
                    observed_value,
                    baseline_median: reference.median,
                    robust_z_score,
                    threshold: self.baseline.detector_config.robust_z_threshold,
                });
            }
        }

        let disposition = if uncalibrated_signals.is_empty() {
            ObservationDisposition::Evaluated
        } else {
            ObservationDisposition::EvaluatedWithUncalibratedSignals
        };
        self.report(frame, disposition, deviations, uncalibrated_signals)
    }

    fn report(
        &self,
        frame: &SensorFrame,
        disposition: ObservationDisposition,
        deviations: Vec<DetectedDeviation>,
        uncalibrated_signals: Vec<Signal>,
    ) -> ObservationReport {
        ObservationReport {
            configuration_id: self.baseline.detector_config.configuration_id.clone(),
            reference_dataset_id: self.baseline.detector_config.reference_dataset_id.clone(),
            session_id: frame.session_id.clone(),
            sequence: frame.sequence,
            timestamp_ms: frame.timestamp_ms,
            process_profile: frame.profile.clone(),
            detector_id: self.baseline.detector_config.detector_id.clone(),
            detector_version: self.baseline.detector_config.detector_version.clone(),
            reference_frame_count: self.baseline.reference_frame_count,
            disposition,
            deviations,
            uncalibrated_signals,
        }
    }
}

/// Monitoring-engine construction failure.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum EngineError {
    /// Session identity was empty.
    EmptySessionId,
}
impl fmt::Display for EngineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "cannot create monitoring engine: {self:?}")
    }
}
impl std::error::Error for EngineError {}

fn validate_process_profile(profile: &ProcessProfile) -> Result<(), BaselineError> {
    if profile.design_revision.trim().is_empty()
        || profile.recipe_revision.trim().is_empty()
        || profile.equipment_profile.trim().is_empty()
        || profile.material_profile.trim().is_empty()
        || profile.environment_profile.trim().is_empty()
        || profile.calibration_profile.trim().is_empty()
    {
        return Err(BaselineError::EmptyProcessProfileIdentity);
    }
    Ok(())
}

fn validate_frame(frame: &SensorFrame) -> Result<(), InputError> {
    if frame.session_id.trim().is_empty() {
        return Err(InputError::EmptySessionId);
    }
    for signal in Signal::ALL {
        if let Some(value) = signal_value(frame, signal) {
            if !value.is_finite() {
                return Err(InputError::NonFinite(signal));
            }
            if value < 0.0 && matches!(
                signal,
                Signal::VibrationRms | Signal::FilamentTension | Signal::Humidity
            ) {
                return Err(InputError::Negative(signal));
            }
            if signal == Signal::Humidity && value > 100.0 {
                return Err(InputError::OutOfRange(signal));
            }
        }
    }
    Ok(())
}

fn signal_value(frame: &SensorFrame, signal: Signal) -> Option<f64> {
    match signal {
        Signal::HotendTemperature => Some(frame.hotend_temp_c),
        Signal::BedTemperature => Some(frame.bed_temp_c),
        Signal::XCurrent => Some(frame.stepper_current_ma[0]),
        Signal::YCurrent => Some(frame.stepper_current_ma[1]),
        Signal::ZCurrent => Some(frame.stepper_current_ma[2]),
        Signal::ExtruderCurrent => Some(frame.stepper_current_ma[3]),
        Signal::VibrationRms => Some(frame.vibration_rms_g),
        Signal::FilamentTension => frame.filament_tension_g,
        Signal::AmbientTemperature => frame.ambient_temp_c,
        Signal::Humidity => frame.humidity_pct,
    }
}

fn median(values: &mut [f64]) -> f64 {
    values.sort_by(f64::total_cmp);
    let middle = values.len() / 2;
    if values.len().is_multiple_of(2) {
        (values[middle - 1] + values[middle]) / 2.0
    } else {
        values[middle]
    }
}

const fn deviation_kind(signal: Signal) -> DeviationKind {
    match signal {
        Signal::HotendTemperature | Signal::BedTemperature => DeviationKind::TemperatureDeviation,
        Signal::XCurrent | Signal::YCurrent | Signal::ZCurrent => {
            DeviationKind::MotorCurrentDeviation
        }
        Signal::ExtruderCurrent => DeviationKind::ExtrusionCurrentDeviation,
        Signal::VibrationRms => DeviationKind::VibrationDeviation,
        Signal::FilamentTension => DeviationKind::FilamentTensionDeviation,
        Signal::AmbientTemperature => DeviationKind::TemperatureDeviation,
        Signal::Humidity => DeviationKind::HumidityDeviation,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> ProcessProfile {
        ProcessProfile {
            design_revision: "design:bracket@sha256:test-a".into(),
            recipe_revision: "recipe:pla-0.2mm@sha256:test-b".into(),
            equipment_profile: "equipment:printer-7:config-r3".into(),
            material_profile: "material:PLA-grade-A:lot-2026-10-01".into(),
            environment_profile: "environment:enclosed-lab-profile-v1".into(),
            calibration_profile: "calibration:printer-7:2026-10-01".into(),
        }
    }

    fn config() -> DetectorConfig {
        DetectorConfig {
            detector_id: "cincinnati.robust-reference".into(),
            detector_version: "1.0.0".into(),
            configuration_id: "config:robust-z-v1-test-profile".into(),
            reference_dataset_id: "fixture:synthetic-healthy-reference-v1".into(),
            robust_z_threshold: 3.5,
            minimum_scale_by_signal: [
                0.5, 0.5, 10.0, 10.0, 10.0, 10.0, 0.01, 2.0, 0.5, 2.0,
            ],
            minimum_baseline_samples: 5,
            maximum_sample_gap_ms: 1_000,
        }
    }

    fn frame(session_id: &str, sequence: u64, timestamp_ms: u64) -> SensorFrame {
        let i = sequence as f64;
        SensorFrame {
            session_id: session_id.into(),
            sequence,
            timestamp_ms,
            profile: profile(),
            hotend_temp_c: 200.0 + i * 0.1,
            bed_temp_c: 60.0 + i * 0.1,
            stepper_current_ma: [100.0 + i, 200.0 + i, 150.0 + i, 300.0 + 2.0 * i],
            vibration_rms_g: 0.10 + i * 0.001,
            filament_tension_g: Some(50.0 + i * 0.5),
            ambient_temp_c: Some(25.0 + i * 0.05),
            humidity_pct: Some(45.0 + i * 0.2),
        }
    }

    fn healthy_frames() -> Vec<SensorFrame> {
        (0..7).map(|i| frame("reference", i, i * 100)).collect()
    }

    fn engine(session_id: &str) -> MonitoringEngine {
        let baseline = ReferenceBaseline::fit(profile(), config(), &healthy_frames()).unwrap();
        MonitoringEngine::new(baseline, session_id).unwrap()
    }

    #[test]
    fn identical_inputs_produce_identical_reports() {
        let sample = frame("live", 1, 100);
        assert_eq!(engine("live").observe(&sample), engine("live").observe(&sample));
    }

    #[test]
    fn healthy_reference_like_frame_has_no_deviations() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.hotend_temp_c = 200.3;
        sample.bed_temp_c = 60.3;
        sample.stepper_current_ma = [103.0, 203.0, 153.0, 306.0];
        sample.vibration_rms_g = 0.103;
        sample.filament_tension_g = Some(51.5);
        let report = engine.observe(&sample);
        assert_eq!(report.disposition, ObservationDisposition::Evaluated);
        assert!(report.deviations.is_empty());
    }

    #[test]
    fn hotend_drift_is_reported_as_temperature_deviation() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.hotend_temp_c = 230.0;
        let report = engine.observe(&sample);
        assert!(report.deviations.iter().any(|d| {
            d.signal == Signal::HotendTemperature
                && d.kind == DeviationKind::TemperatureDeviation
                && d.robust_z_score > d.threshold
        }));
    }

    #[test]
    fn extruder_current_drift_is_not_labeled_as_a_physical_defect() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.stepper_current_ma[3] = 500.0;
        let report = engine.observe(&sample);
        assert!(report.deviations.iter().any(|d| {
            d.signal == Signal::ExtruderCurrent
                && d.kind == DeviationKind::ExtrusionCurrentDeviation
        }));
    }

    #[test]
    fn vibration_drift_is_detected() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.vibration_rms_g = 0.8;
        assert!(engine
            .observe(&sample)
            .deviations
            .iter()
            .any(|d| d.signal == Signal::VibrationRms));
    }


    #[test]
    fn missing_reference_dataset_identity_is_rejected() {
        let mut detector_config = config();
        detector_config.reference_dataset_id.clear();
        assert_eq!(
            detector_config.validate(),
            Err(ConfigError::EmptyReferenceDatasetId)
        );
    }

    #[test]
    fn baseline_requires_minimum_reference_count() {
        let error =
            ReferenceBaseline::fit(profile(), config(), &healthy_frames()[..4]).unwrap_err();
        assert_eq!(error, BaselineError::InsufficientSamples { found: 4, required: 5 });
    }

    #[test]
    fn non_finite_observation_is_rejected() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.hotend_temp_c = f64::NAN;
        let report = engine.observe(&sample);
        assert_eq!(
            report.disposition,
            ObservationDisposition::InvalidObservation(InputError::NonFinite(
                Signal::HotendTemperature
            ))
        );
        assert!(report.deviations.is_empty());
    }

    #[test]
    fn changed_recipe_cannot_reuse_baseline_silently() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.profile.recipe_revision = "recipe:changed".into();
        assert_eq!(
            engine.observe(&sample).disposition,
            ObservationDisposition::ProcessProfileMismatch
        );
    }

    #[test]
    fn duplicate_or_reordered_sequence_is_rejected() {
        let mut engine = engine("live");
        assert_eq!(
            engine.observe(&frame("live", 10, 1_000)).disposition,
            ObservationDisposition::Evaluated
        );
        assert_eq!(
            engine.observe(&frame("live", 9, 1_100)).disposition,
            ObservationDisposition::NonMonotonicSequence
        );
    }

    #[test]
    fn sequence_gap_is_not_scored_and_next_contiguous_frame_recovers() {
        let mut engine = engine("live");
        engine.observe(&frame("live", 1, 100));
        let gap = engine.observe(&frame("live", 3, 300));
        assert_eq!(
            gap.disposition,
            ObservationDisposition::DataGap {
                missing_sequences: 1,
                elapsed_ms: 200
            }
        );
        assert_eq!(
            engine.observe(&frame("live", 4, 400)).disposition,
            ObservationDisposition::Evaluated
        );
    }

    #[test]
    fn optional_sensor_without_baseline_is_explicitly_unqualified() {
        let mut references = healthy_frames();
        for sample in &mut references {
            sample.filament_tension_g = None;
        }
        let baseline = ReferenceBaseline::fit(profile(), config(), &references).unwrap();
        let mut engine = MonitoringEngine::new(baseline, "live").unwrap();
        let report = engine.observe(&frame("live", 1, 100));
        assert_eq!(report.disposition, ObservationDisposition::EvaluatedWithUncalibratedSignals);
        assert_eq!(report.uncalibrated_signals, vec![Signal::FilamentTension]);
    }

    #[test]
    fn zero_mad_uses_profile_floor_instead_of_dividing_by_zero() {
        let mut references = vec![frame("reference", 0, 0); 7];
        for (index, sample) in references.iter_mut().enumerate() {
            sample.sequence = index as u64;
            sample.timestamp_ms = index as u64 * 100;
            sample.hotend_temp_c = 200.0;
        }
        let baseline = ReferenceBaseline::fit(profile(), config(), &references).unwrap();
        let stats = baseline.signals[Signal::HotendTemperature.index()].unwrap();
        assert_eq!(stats.median, 200.0);
        assert_eq!(stats.robust_scale, config().minimum_scale_by_signal[0]);
        assert!(stats.robust_scale.is_finite() && stats.robust_scale > 0.0);
    }


    #[test]
    fn duplicate_reference_identity_cannot_inflate_baseline_sample_count() {
        let mut references = healthy_frames();
        references.push(references[0].clone());
        let error = ReferenceBaseline::fit(profile(), config(), &references).unwrap_err();
        assert_eq!(
            error,
            BaselineError::DuplicateReferenceObservation { sample_index: 7 }
        );
    }

    #[test]
    fn negative_vibration_rms_is_rejected() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.vibration_rms_g = -1.0;
        assert_eq!(
            engine.observe(&sample).disposition,
            ObservationDisposition::InvalidObservation(InputError::Negative(Signal::VibrationRms))
        );
    }


    #[test]
    fn signed_motor_current_is_evaluated_as_a_deviation_not_rejected() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.stepper_current_ma[0] = -50.0;
        let report = engine.observe(&sample);
        assert_eq!(report.disposition, ObservationDisposition::Evaluated);
        assert!(report.deviations.iter().any(|deviation| {
            deviation.signal == Signal::XCurrent
                && deviation.kind == DeviationKind::MotorCurrentDeviation
        }));
    }

    #[test]
    fn report_binds_exact_reference_dataset_identity() {
        let mut engine = engine("live");
        let report = engine.observe(&frame("live", 1, 100));
        assert_eq!(
            report.reference_dataset_id,
            "fixture:synthetic-healthy-reference-v1"
        );
        assert_eq!(
            report.configuration_id,
            "config:robust-z-v1-test-profile"
        );
    }

    #[test]
    fn changed_calibration_profile_is_not_admitted() {
        let mut engine = engine("live");
        let mut sample = frame("live", 1, 100);
        sample.profile.calibration_profile = "calibration:new".into();
        assert_eq!(
            engine.observe(&sample).disposition,
            ObservationDisposition::ProcessProfileMismatch
        );
    }
}
