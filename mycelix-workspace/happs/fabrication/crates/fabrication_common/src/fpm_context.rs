//! Exact-context safety boundary for Fabrication Process Monitoring.
//!
//! This layer prevents a baseline learned under one declared process context
//! from being reused for a different context or incomplete registration.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::fpm::{self, BaselineProfile, DetectorConfig, FpmError, ProcessDetection, ProcessObservation};

pub const FPM_CONTEXT_SCHEMA_VERSION: &str = "fpm.context.v1";
const MAX_LABEL_BYTES: usize = 128;
const SHA256_HEX_LEN: usize = 64;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProcessContext {
    pub phase: Option<String>,
    pub layer_number: Option<u32>,
    pub move_class: Option<String>,
    pub commanded_speed_mm_s: Option<u16>,
    pub target_hotend_temp_c: Option<u16>,
    pub target_bed_temp_c: Option<u16>,
    pub acquisition_clock_domain: String,
    pub sensor_source_id: String,
    pub calibration_profile_digest: String,
}

impl ProcessContext {
    pub fn validate_for_selection(&self) -> Result<(), ContextualFpmError> {
        validate_label(&self.acquisition_clock_domain, "acquisition_clock_domain")?;
        validate_label(&self.sensor_source_id, "sensor_source_id")?;
        validate_digest(&self.calibration_profile_digest, "calibration_profile_digest")?;
        for (name, value) in [
            ("phase", self.phase.as_deref()),
            ("move_class", self.move_class.as_deref()),
        ] {
            if let Some(value) = value {
                validate_label(value, name)?;
            }
        }
        if self.phase.is_none()
            && self.layer_number.is_none()
            && self.move_class.is_none()
            && self.commanded_speed_mm_s.is_none()
            && self.target_hotend_temp_c.is_none()
            && self.target_bed_temp_c.is_none()
        {
            return Err(ContextualFpmError::IncompleteContext(
                "at least one process discriminator is required".into(),
            ));
        }
        Ok(())
    }

    pub fn digest(&self) -> Result<String, ContextualFpmError> {
        let bytes = serde_json::to_vec(self).map_err(|e| {
            ContextualFpmError::Serialization(format!("failed to serialize context: {e}"))
        })?;
        Ok(hex_digest(&bytes))
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContextualObservation {
    pub observation: ProcessObservation,
    pub context: ProcessContext,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContextualBaselineProfile {
    pub schema_version: String,
    pub context: ProcessContext,
    pub context_digest: String,
    pub detector_id: String,
    pub detector_version: String,
    pub baseline: BaselineProfile,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContextualDetectionProvenance {
    pub schema_version: String,
    pub context_digest: String,
    pub baseline_context_digest: String,
    pub fpm_input_digest: String,
    pub combined_input_digest: String,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct ContextualDetection {
    pub detection: ProcessDetection,
    pub provenance: ContextualDetectionProvenance,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ContextualFpmError {
    IncompleteContext(String),
    InvalidContext(String),
    ContextMismatch,
    Serialization(String),
    Detector(FpmError),
}

impl std::fmt::Display for ContextualFpmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IncompleteContext(r) => write!(f, "incomplete FPM context: {r}"),
            Self::InvalidContext(r) => write!(f, "invalid FPM context: {r}"),
            Self::ContextMismatch => write!(f, "observation context does not exactly match baseline"),
            Self::Serialization(r) => write!(f, "FPM context serialization error: {r}"),
            Self::Detector(e) => write!(f, "FPM detector error: {e}"),
        }
    }
}

impl From<FpmError> for ContextualFpmError {
    fn from(value: FpmError) -> Self { Self::Detector(value) }
}

fn validate_label(value: &str, field: &str) -> Result<(), ContextualFpmError> {
    if value.trim().is_empty() {
        return Err(ContextualFpmError::InvalidContext(format!("{field} cannot be empty")));
    }
    if value.len() > MAX_LABEL_BYTES {
        return Err(ContextualFpmError::InvalidContext(format!(
            "{field} cannot exceed {MAX_LABEL_BYTES} bytes"
        )));
    }
    Ok(())
}

fn validate_digest(value: &str, field: &str) -> Result<(), ContextualFpmError> {
    if value.len() != SHA256_HEX_LEN || !value.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(ContextualFpmError::InvalidContext(format!(
            "{field} must be a {SHA256_HEX_LEN}-character hexadecimal SHA-256 digest"
        )));
    }
    Ok(())
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().iter().map(|b| format!("{b:02x}")).collect()
}

impl ContextualBaselineProfile {
    pub fn from_observations(
        config: &DetectorConfig,
        observations: &[ContextualObservation],
    ) -> Result<Self, ContextualFpmError> {
        let first = observations.first().ok_or_else(|| {
            ContextualFpmError::IncompleteContext("at least one observation is required".into())
        })?;
        let context = first.context.clone();
        context.validate_for_selection()?;
        if observations.iter().any(|o| o.context != context) {
            return Err(ContextualFpmError::ContextMismatch);
        }
        let raw: Vec<ProcessObservation> =
            observations.iter().map(|o| o.observation.clone()).collect();
        let baseline = BaselineProfile::from_observations(config, &raw)?;
        Ok(Self {
            schema_version: FPM_CONTEXT_SCHEMA_VERSION.into(),
            context_digest: context.digest()?,
            context,
            detector_id: config.detector_id.clone(),
            detector_version: config.detector_version.clone(),
            baseline,
        })
    }

    pub fn digest(&self) -> Result<String, ContextualFpmError> {
        let bytes = serde_json::to_vec(self).map_err(|e| {
            ContextualFpmError::Serialization(format!("failed to serialize baseline: {e}"))
        })?;
        Ok(hex_digest(&bytes))
    }
}

pub fn analyze(
    config: &DetectorConfig,
    baseline: &ContextualBaselineProfile,
    observation: &ContextualObservation,
) -> Result<ContextualDetection, ContextualFpmError> {
    baseline.context.validate_for_selection()?;
    observation.context.validate_for_selection()?;
    if baseline.schema_version != FPM_CONTEXT_SCHEMA_VERSION {
        return Err(ContextualFpmError::InvalidContext("unsupported context schema".into()));
    }
    if baseline.detector_id != config.detector_id
        || baseline.detector_version != config.detector_version
    {
        return Err(ContextualFpmError::InvalidContext(
            "contextual baseline detector identity/version does not match config".into(),
        ));
    }
    let context_digest = observation.context.digest()?;
    if context_digest != baseline.context_digest || observation.context != baseline.context {
        return Err(ContextualFpmError::ContextMismatch);
    }
    let detection = fpm::analyze(config, &baseline.baseline, &observation.observation)?;
    let combined_input_digest = digest_parts(&(
        FPM_CONTEXT_SCHEMA_VERSION,
        &detection,
        &baseline.context_digest,
        &context_digest,
    ))?;
    Ok(ContextualDetection {
        provenance: ContextualDetectionProvenance {
            schema_version: FPM_CONTEXT_SCHEMA_VERSION.into(),
            context_digest,
            baseline_context_digest: baseline.context_digest.clone(),
            fpm_input_digest: detection.provenance.input_digest.clone(),
            combined_input_digest,
        },
        detection,
    })
}

fn digest_parts<T: Serialize>(value: &T) -> Result<String, ContextualFpmError> {
    let bytes = serde_json::to_vec(value).map_err(|e| {
        ContextualFpmError::Serialization(format!("failed to serialize contextual input: {e}"))
    })?;
    Ok(hex_digest(&bytes))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{fpm::{DetectionStatus, ProcessAnomalyType}, SensorSnapshot};

    fn context() -> ProcessContext {
        ProcessContext {
            phase: Some("perimeter".into()),
            layer_number: Some(7),
            move_class: Some("extrusion".into()),
            commanded_speed_mm_s: Some(45),
            target_hotend_temp_c: Some(210),
            target_bed_temp_c: Some(60),
            acquisition_clock_domain: "printer-clock-1".into(),
            sensor_source_id: "sensor-array-1".into(),
            calibration_profile_digest:
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
        }
    }

    fn obs(ctx: ProcessContext, sequence: u64, hotend: f32) -> ContextualObservation {
        ContextualObservation {
            observation: ProcessObservation {
                sequence,
                sensor: SensorSnapshot {
                    hotend_temp: hotend,
                    bed_temp: 60.0,
                    stepper_currents: [0.8, 0.8, 1.0, 1.0],
                    vibration_rms: 0.05,
                    filament_tension: Some(100.0),
                    ambient_temp: Some(22.0),
                    humidity: Some(40.0),
                },
            },
            context: ctx,
        }
    }

    fn baseline(ctx: ProcessContext) -> ContextualBaselineProfile {
        let samples = (0..10)
            .map(|n| obs(ctx.clone(), n, 210.0 + (n % 2) as f32 * 0.2))
            .collect::<Vec<_>>();
        ContextualBaselineProfile::from_observations(&DetectorConfig::default(), &samples)
            .expect("valid contextual baseline")
    }

    #[test]
    fn mixed_context_baseline_is_rejected() {
        let ctx = context();
        let mut samples = (0..10).map(|n| obs(ctx.clone(), n, 210.0)).collect::<Vec<_>>();
        let mut other = ctx.clone();
        other.move_class = Some("travel".into());
        samples[9].context = other;
        assert_eq!(
            ContextualBaselineProfile::from_observations(&DetectorConfig::default(), &samples),
            Err(ContextualFpmError::ContextMismatch)
        );
    }

    #[test]
    fn missing_process_context_is_rejected() {
        let mut ctx = context();
        ctx.phase = None;
        ctx.layer_number = None;
        ctx.move_class = None;
        ctx.commanded_speed_mm_s = None;
        ctx.target_hotend_temp_c = None;
        ctx.target_bed_temp_c = None;
        assert!(matches!(
            ctx.validate_for_selection(),
            Err(ContextualFpmError::IncompleteContext(_))
        ));
    }

    #[test]
    fn mismatch_cannot_be_normal() {
        let config = DetectorConfig::default();
        let base = baseline(context());
        let mut other = context();
        other.layer_number = Some(8);
        let result = analyze(&config, &base, &obs(other, 100, 210.0));
        assert_eq!(result, Err(ContextualFpmError::ContextMismatch));
    }

    #[test]
    fn calibration_change_is_a_mismatch() {
        let config = DetectorConfig::default();
        let base = baseline(context());
        let mut other = context();
        other.calibration_profile_digest =
            "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
        assert_eq!(
            analyze(&config, &base, &obs(other, 100, 210.0)),
            Err(ContextualFpmError::ContextMismatch)
        );
    }

    #[test]
    fn exact_context_retains_detector_evidence() {
        let config = DetectorConfig::default();
        let base = baseline(context());
        let result = analyze(&config, &base, &obs(context(), 100, 250.0)).expect("analysis");
        assert_eq!(result.detection.status, DetectionStatus::Anomalous);
        assert!(result.detection.anomaly_types.contains(
            &ProcessAnomalyType::TemperatureDeviation
        ));
        assert_ne!(
            result.provenance.fpm_input_digest,
            result.provenance.combined_input_digest
        );
    }

    #[test]
    fn context_digest_changes_with_process_context() {
        let a = context();
        let mut b = a.clone();
        b.commanded_speed_mm_s = Some(46);
        assert_ne!(a.digest().expect("a"), b.digest().expect("b"));
    }

    #[test]
    fn invalid_calibration_digest_is_rejected() {
        let mut ctx = context();
        ctx.calibration_profile_digest = "invalid".into();
        assert!(matches!(
            ctx.validate_for_selection(),
            Err(ContextualFpmError::InvalidContext(_))
        ));
    }

    #[test]
    fn deterministic_contextual_baseline() {
        let config = DetectorConfig::default();
        let a = baseline(context());
        let b = baseline(context());
        assert_eq!(a, b);
        assert_eq!(a.digest().expect("a"), b.digest().expect("b"));
        let _ = config;
    }
}
