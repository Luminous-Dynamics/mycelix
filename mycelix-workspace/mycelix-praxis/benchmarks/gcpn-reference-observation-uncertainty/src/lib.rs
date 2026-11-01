//! Known-error observation verification for GCPN.
//!
//! Under the declared additive model
//! observed = latent_target + known_bias + epsilon,
//! with E[epsilon] = 0 and known Var(epsilon), the following estimator is
//! unbiased for latent-target squared error:
//! (forecast - (observed - bias))^2 - Var(epsilon).
//!
//! It is not a universal latent-truth estimator. Unknown bias, unknown
//! covariance, support mismatch, source/calibration mismatch and post-cutoff
//! evidence disable or reject qualification rather than invent assumptions.
//! Negative finite-sample corrected MSE values are retained, not clipped.

use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PredictandContract {
    pub predictand_id: String,
    pub unit: String,
    pub temporal_support_id: String,
    pub forecast_spatial_support_digest: String,
    pub observation_source_id: String,
    pub observation_source_version: String,
    pub calibration_epoch_id: String,
    pub observation_model_id: String,
    pub as_of_cutoff: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObservationErrorModel {
    pub known_bias: Option<f64>,
    pub measurement_variance: Option<f64>,
    pub representativeness_variance: Option<f64>,
    /// Covariance between measurement and representativeness error.
    /// Unknown covariance disables the corrected estimator.
    pub cross_error_covariance: Option<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastObservationPair {
    pub case_id: String,
    pub independent_system_id: String,
    pub forecast_value: f64,
    pub observed_value: f64,
    pub source_id: String,
    pub source_version: String,
    pub calibration_epoch_id: String,
    pub observation_model_id: String,
    pub temporal_support_id: String,
    pub native_spatial_support_digest: String,
    pub evaluated_spatial_support_digest: String,
    /// Identity of an observation operator already applied upstream.
    pub observation_operator_id: Option<String>,
    pub recorded_at: u64,
    /// Shared calibration/network/retrieval error lineage root.
    pub error_lineage_root: String,
    pub error_model: ObservationErrorModel,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CorrectionUnavailableReason {
    UnknownBias,
    UnknownMeasurementVariance,
    UnknownRepresentativenessVariance,
    UnknownCrossErrorCovariance,
}

impl CorrectionUnavailableReason {
    pub fn label(self) -> &'static str {
        match self {
            Self::UnknownBias => "unknown_bias",
            Self::UnknownMeasurementVariance => "unknown_measurement_variance",
            Self::UnknownRepresentativenessVariance => "unknown_representativeness_variance",
            Self::UnknownCrossErrorCovariance => "unknown_cross_error_covariance",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ErrorComponentsSummary {
    pub mean_known_bias: Option<f64>,
    pub mean_measurement_variance: Option<f64>,
    pub mean_representativeness_variance: Option<f64>,
    pub mean_twice_cross_covariance: Option<f64>,
    pub mean_total_error_variance: Option<f64>,
    pub unique_error_lineage_roots: usize,
    pub independent_systems: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ObservationVerificationReport {
    pub predictand_id: String,
    pub unit: String,
    pub observation_source_id: String,
    pub observation_source_version: String,
    pub calibration_epoch_id: String,
    pub observation_model_id: String,
    pub as_of_cutoff: u64,
    pub cases: usize,
    /// MSE against raw reference observations (observation-space metric).
    pub naive_observation_mse: f64,
    /// Unbiased latent-target MSE estimator under the fully specified model.
    /// It may be negative in a finite sample and is never clipped.
    pub error_corrected_mse: Option<f64>,
    pub correction_unavailable_reason: Option<CorrectionUnavailableReason>,
    pub error_components: ErrorComponentsSummary,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContractViolation {
    EmptyFrame,
    InvalidIdentifier,
    NonFiniteForecast,
    NonFiniteObservation,
    DuplicateCaseId,
    SourceMismatch,
    SourceVersionMismatch,
    CalibrationEpochMismatch,
    ObservationModelMismatch,
    TemporalSupportMismatch,
    SpatialSupportMismatch,
    MissingObservationOperator,
    ObservationRecordedAfterCutoff,
    InvalidErrorVariance,
    NonFiniteErrorModel,
    IncoherentErrorCovariance,
}

impl ContractViolation {
    pub fn label(self) -> &'static str {
        match self {
            Self::EmptyFrame => "empty_frame",
            Self::InvalidIdentifier => "invalid_identifier",
            Self::NonFiniteForecast => "non_finite_forecast",
            Self::NonFiniteObservation => "non_finite_observation",
            Self::DuplicateCaseId => "duplicate_case_id",
            Self::SourceMismatch => "source_mismatch",
            Self::SourceVersionMismatch => "source_version_mismatch",
            Self::CalibrationEpochMismatch => "calibration_epoch_mismatch",
            Self::ObservationModelMismatch => "observation_model_mismatch",
            Self::TemporalSupportMismatch => "temporal_support_mismatch",
            Self::SpatialSupportMismatch => "spatial_support_mismatch",
            Self::MissingObservationOperator => "missing_observation_operator",
            Self::ObservationRecordedAfterCutoff => "observation_after_cutoff",
            Self::InvalidErrorVariance => "invalid_error_variance",
            Self::NonFiniteErrorModel => "non_finite_error_model",
            Self::IncoherentErrorCovariance => "incoherent_error_covariance",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContractError {
    pub case_id: Option<String>,
    pub violations: Vec<ContractViolation>,
}

impl ContractError {
    fn new(case_id: Option<String>, violation: ContractViolation) -> Self {
        Self { case_id, violations: vec![violation] }
    }
}

impl PredictandContract {
    pub fn validate(&self) -> Result<(), ContractError> {
        for value in [
            self.predictand_id.as_str(),
            self.unit.as_str(),
            self.temporal_support_id.as_str(),
            self.forecast_spatial_support_digest.as_str(),
            self.observation_source_id.as_str(),
            self.observation_source_version.as_str(),
            self.calibration_epoch_id.as_str(),
            self.observation_model_id.as_str(),
        ] {
            if !valid_identifier(value) {
                return Err(ContractError::new(None, ContractViolation::InvalidIdentifier));
            }
        }
        Ok(())
    }
}

pub fn evaluate(
    contract: &PredictandContract,
    pairs: &[ForecastObservationPair],
) -> Result<ObservationVerificationReport, ContractError> {
    contract.validate()?;
    if pairs.is_empty() {
        return Err(ContractError::new(None, ContractViolation::EmptyFrame));
    }

    let mut case_ids = BTreeSet::new();
    let mut lineage_roots = BTreeSet::new();
    let mut system_ids = BTreeSet::new();
    let mut naive_squared_error_sum = 0.0;
    let mut corrected_squared_error_sum = 0.0;
    let mut total_bias = 0.0;
    let mut total_measurement_variance = 0.0;
    let mut total_representativeness_variance = 0.0;
    let mut total_twice_covariance = 0.0;
    let mut all_bias_known = true;
    let mut all_measurement_known = true;
    let mut all_representativeness_known = true;
    let mut all_covariance_known = true;

    for pair in pairs {
        for value in [
            pair.case_id.as_str(),
            pair.independent_system_id.as_str(),
            pair.source_id.as_str(),
            pair.source_version.as_str(),
            pair.calibration_epoch_id.as_str(),
            pair.observation_model_id.as_str(),
            pair.temporal_support_id.as_str(),
            pair.native_spatial_support_digest.as_str(),
            pair.evaluated_spatial_support_digest.as_str(),
            pair.error_lineage_root.as_str(),
        ] {
            if !valid_identifier(value) {
                return Err(ContractError::new(
                    Some(pair.case_id.clone()),
                    ContractViolation::InvalidIdentifier,
                ));
            }
        }
        if !case_ids.insert(pair.case_id.as_str()) {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::DuplicateCaseId,
            ));
        }
        if !pair.forecast_value.is_finite() {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::NonFiniteForecast,
            ));
        }
        if !pair.observed_value.is_finite() {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::NonFiniteObservation,
            ));
        }
        if pair.source_id != contract.observation_source_id {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::SourceMismatch,
            ));
        }
        if pair.source_version != contract.observation_source_version {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::SourceVersionMismatch,
            ));
        }
        if pair.calibration_epoch_id != contract.calibration_epoch_id {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::CalibrationEpochMismatch,
            ));
        }
        if pair.observation_model_id != contract.observation_model_id {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::ObservationModelMismatch,
            ));
        }
        if pair.temporal_support_id != contract.temporal_support_id {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::TemporalSupportMismatch,
            ));
        }
        if pair.recorded_at > contract.as_of_cutoff {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::ObservationRecordedAfterCutoff,
            ));
        }
        if pair.evaluated_spatial_support_digest != contract.forecast_spatial_support_digest {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::SpatialSupportMismatch,
            ));
        }
        if pair.native_spatial_support_digest != pair.evaluated_spatial_support_digest
            && pair.observation_operator_id.as_ref().is_none_or(|value| !valid_identifier(value))
        {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::MissingObservationOperator,
            ));
        }

        validate_error_model(pair)?;

        naive_squared_error_sum += (pair.forecast_value - pair.observed_value).powi(2);

        if let Some(value) = pair.error_model.known_bias {
            total_bias += value;
        } else {
            all_bias_known = false;
        }
        if let Some(value) = pair.error_model.measurement_variance {
            total_measurement_variance += value;
        } else {
            all_measurement_known = false;
        }
        if let Some(value) = pair.error_model.representativeness_variance {
            total_representativeness_variance += value;
        } else {
            all_representativeness_known = false;
        }
        if let Some(value) = pair.error_model.cross_error_covariance {
            total_twice_covariance += 2.0 * value;
        } else {
            all_covariance_known = false;
        }

        lineage_roots.insert(pair.error_lineage_root.as_str());
        system_ids.insert(pair.independent_system_id.as_str());
    }

    let count = pairs.len() as f64;
    let correction_unavailable_reason = if !all_bias_known {
        Some(CorrectionUnavailableReason::UnknownBias)
    } else if !all_measurement_known {
        Some(CorrectionUnavailableReason::UnknownMeasurementVariance)
    } else if !all_representativeness_known {
        Some(CorrectionUnavailableReason::UnknownRepresentativenessVariance)
    } else if !all_covariance_known {
        Some(CorrectionUnavailableReason::UnknownCrossErrorCovariance)
    } else {
        None
    };

    let total_variance_complete =
        all_measurement_known && all_representativeness_known && all_covariance_known;
    let total_variance_sum = total_measurement_variance
        + total_representativeness_variance
        + total_twice_covariance;
    if total_variance_complete && !total_variance_sum.is_finite() {
        return Err(ContractError::new(None, ContractViolation::NonFiniteErrorModel));
    }

    let mean_measurement = all_measurement_known.then_some(total_measurement_variance / count);
    let mean_representativeness =
        all_representativeness_known.then_some(total_representativeness_variance / count);
    let mean_twice_covariance = all_covariance_known.then_some(total_twice_covariance / count);
    let mean_total_variance = if total_variance_complete {
        Some(total_variance_sum / count)
    } else {
        None
    };

    let mut error_corrected_mse = None;
    if correction_unavailable_reason.is_none() {
        for pair in pairs {
            let bias = pair.error_model.known_bias.unwrap();
            let measurement = pair.error_model.measurement_variance.unwrap();
            let representativeness = pair.error_model.representativeness_variance.unwrap();
            let covariance = pair.error_model.cross_error_covariance.unwrap();
            let total_variance = measurement + representativeness + (2.0 * covariance);
            let adjusted_observation = pair.observed_value - bias;
            corrected_squared_error_sum +=
                (pair.forecast_value - adjusted_observation).powi(2) - total_variance;
        }
        error_corrected_mse = Some(corrected_squared_error_sum / count);
    }

    Ok(ObservationVerificationReport {
        predictand_id: contract.predictand_id.clone(),
        unit: contract.unit.clone(),
        observation_source_id: contract.observation_source_id.clone(),
        observation_source_version: contract.observation_source_version.clone(),
        calibration_epoch_id: contract.calibration_epoch_id.clone(),
        observation_model_id: contract.observation_model_id.clone(),
        as_of_cutoff: contract.as_of_cutoff,
        cases: pairs.len(),
        naive_observation_mse: naive_squared_error_sum / count,
        error_corrected_mse,
        correction_unavailable_reason,
        error_components: ErrorComponentsSummary {
            mean_known_bias: all_bias_known.then_some(total_bias / count),
            mean_measurement_variance: mean_measurement,
            mean_representativeness_variance: mean_representativeness,
            mean_twice_cross_covariance: mean_twice_covariance,
            mean_total_error_variance: mean_total_variance,
            unique_error_lineage_roots: lineage_roots.len(),
            independent_systems: system_ids.len(),
        },
    })
}

fn validate_error_model(pair: &ForecastObservationPair) -> Result<(), ContractError> {
    let bias = pair.error_model.known_bias;
    let measurement = pair.error_model.measurement_variance;
    let representativeness = pair.error_model.representativeness_variance;
    let covariance = pair.error_model.cross_error_covariance;

    for value in [bias, measurement, representativeness, covariance].into_iter().flatten() {
        if !value.is_finite() {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::NonFiniteErrorModel,
            ));
        }
    }
    for variance in [measurement, representativeness].into_iter().flatten() {
        if variance < 0.0 {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::InvalidErrorVariance,
            ));
        }
    }

    if let (Some(measurement), Some(representativeness), Some(covariance)) =
        (measurement, representativeness, covariance)
    {
        // Positive semidefiniteness of the 2x2 covariance matrix:
        // cov^2 <= variance_measurement * variance_representativeness.
        let determinant = measurement * representativeness - covariance.powi(2);
        if !determinant.is_finite() {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::NonFiniteErrorModel,
            ));
        }
        if determinant < -1e-12 {
            return Err(ContractError::new(
                Some(pair.case_id.clone()),
                ContractViolation::IncoherentErrorCovariance,
            ));
        }
    }
    Ok(())
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 256
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contract() -> PredictandContract {
        PredictandContract {
            predictand_id: "temperature-2m".into(),
            unit: "kelvin".into(),
            temporal_support_id: "hourly-mean".into(),
            forecast_spatial_support_digest: "grid-cell-001".into(),
            observation_source_id: "station-network".into(),
            observation_source_version: "release-7".into(),
            calibration_epoch_id: "calibration-2026-01".into(),
            observation_model_id: "additive-error-model-v1".into(),
            as_of_cutoff: 100,
        }
    }

    fn pair(
        id: &str,
        forecast: f64,
        observed: f64,
        bias: Option<f64>,
        measurement: Option<f64>,
        representativeness: Option<f64>,
        covariance: Option<f64>,
        lineage: &str,
    ) -> ForecastObservationPair {
        ForecastObservationPair {
            case_id: id.into(),
            independent_system_id: format!("system-{id}"),
            forecast_value: forecast,
            observed_value: observed,
            source_id: "station-network".into(),
            source_version: "release-7".into(),
            calibration_epoch_id: "calibration-2026-01".into(),
            observation_model_id: "additive-error-model-v1".into(),
            temporal_support_id: "hourly-mean".into(),
            native_spatial_support_digest: "grid-cell-001".into(),
            evaluated_spatial_support_digest: "grid-cell-001".into(),
            observation_operator_id: None,
            recorded_at: 90,
            error_lineage_root: lineage.into(),
            error_model: ObservationErrorModel {
                known_bias: bias,
                measurement_variance: measurement,
                representativeness_variance: representativeness,
                cross_error_covariance: covariance,
            },
        }
    }

    #[test]
    fn zero_error_model_matches_naive_observation_mse() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), Some(0.0), Some(0.0), Some(0.0), "sensor-a"),
            pair("b", 2.0, 4.0, Some(0.0), Some(0.0), Some(0.0), Some(0.0), "sensor-b"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!((report.naive_observation_mse - 2.5).abs() < 1e-12);
        assert!((report.error_corrected_mse.unwrap() - 2.5).abs() < 1e-12);
    }

    #[test]
    fn known_additive_bias_is_removed_before_squared_error_correction() {
        let data = vec![
            pair("a", 10.0, 13.0, Some(3.0), Some(0.0), Some(0.0), Some(0.0), "sensor-a"),
            pair("b", 12.0, 15.0, Some(3.0), Some(0.0), Some(0.0), Some(0.0), "sensor-b"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!((report.naive_observation_mse - 9.0).abs() < 1e-12);
        assert!(report.error_corrected_mse.unwrap().abs() < 1e-12);
        assert!((report.error_components.mean_known_bias.unwrap() - 3.0).abs() < 1e-12);
    }

    #[test]
    fn measurement_and_representativeness_variance_are_subtracted_separately() {
        let data = vec![
            pair("a", 2.0, 4.0, Some(0.0), Some(1.5), Some(2.0), Some(0.0), "sensor-a"),
            pair("b", 2.0, 4.0, Some(0.0), Some(1.5), Some(2.0), Some(0.0), "sensor-b"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!((report.naive_observation_mse - 4.0).abs() < 1e-12);
        assert!((report.error_corrected_mse.unwrap() - 0.5).abs() < 1e-12);
        assert!((report.error_components.mean_measurement_variance.unwrap() - 1.5).abs() < 1e-12);
        assert!((report.error_components.mean_representativeness_variance.unwrap() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn heteroscedastic_error_uses_case_specific_variance() {
        let data = vec![
            pair("a", 0.0, 1.0, Some(0.0), Some(0.5), Some(0.0), Some(0.0), "sensor-a"),
            pair("b", 0.0, 3.0, Some(0.0), Some(2.5), Some(0.0), Some(0.0), "sensor-b"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!((report.naive_observation_mse - 5.0).abs() < 1e-12);
        assert!((report.error_components.mean_total_error_variance.unwrap() - 1.5).abs() < 1e-12);
        assert!((report.error_corrected_mse.unwrap() - 3.5).abs() < 1e-12);
    }

    #[test]
    fn negative_finite_sample_corrected_mse_is_not_clipped() {
        let data = vec![
            pair("a", 1.0, 1.0, Some(0.0), Some(2.0), Some(0.0), Some(0.0), "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!(report.error_corrected_mse.unwrap() < 0.0);
        assert!((report.error_corrected_mse.unwrap() + 2.0).abs() < 1e-12);
    }

    #[test]
    fn unknown_covariance_disables_corrected_estimate() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), None, "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert_eq!(report.correction_unavailable_reason, Some(CorrectionUnavailableReason::UnknownCrossErrorCovariance));
        assert_eq!(report.error_corrected_mse, None);
    }

    #[test]
    fn unknown_bias_disables_corrected_estimate() {
        let data = vec![
            pair("a", 1.0, 2.0, None, Some(1.0), Some(1.0), Some(0.0), "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert_eq!(report.correction_unavailable_reason, Some(CorrectionUnavailableReason::UnknownBias));
    }

    #[test]
    fn unknown_measurement_variance_disables_corrected_estimate() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), None, Some(1.0), Some(0.0), "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert_eq!(report.correction_unavailable_reason, Some(CorrectionUnavailableReason::UnknownMeasurementVariance));
    }

    #[test]
    fn unknown_representativeness_variance_disables_corrected_estimate() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), Some(1.0), None, Some(0.0), "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert_eq!(report.correction_unavailable_reason, Some(CorrectionUnavailableReason::UnknownRepresentativenessVariance));
    }

    #[test]
    fn valid_measurement_representativeness_covariance_is_included() {
        let data = vec![
            pair("a", 2.0, 4.0, Some(0.0), Some(1.0), Some(2.0), Some(0.5), "sensor-a"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert!(report.error_corrected_mse.unwrap().abs() < 1e-12);
        assert!((report.error_components.mean_twice_cross_covariance.unwrap() - 1.0).abs() < 1e-12);
        assert!((report.error_components.mean_total_error_variance.unwrap() - 4.0).abs() < 1e-12);
    }

    #[test]
    fn incoherent_error_covariance_is_rejected() {
        let record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(2.0), "sensor-a");
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::IncoherentErrorCovariance]
        );
    }

    #[test]
    fn common_error_lineage_is_reported_separately_from_case_count() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), Some(0.1), Some(0.1), Some(0.0), "shared-network-root"),
            pair("b", 1.0, 2.0, Some(0.0), Some(0.1), Some(0.1), Some(0.0), "shared-network-root"),
            pair("c", 1.0, 2.0, Some(0.0), Some(0.1), Some(0.1), Some(0.0), "independent-sensor"),
        ];
        let report = evaluate(&contract(), &data).unwrap();
        assert_eq!(report.cases, 3);
        assert_eq!(report.error_components.unique_error_lineage_roots, 2);
        assert_eq!(report.error_components.independent_systems, 3);
    }

    #[test]
    fn spatial_support_mismatch_requires_a_declared_observation_operator() {
        let mut record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        record.native_spatial_support_digest = "point-sensor-7".into();
        assert_eq!(
            evaluate(&contract(), &[record.clone()]).unwrap_err().violations,
            vec![ContractViolation::MissingObservationOperator]
        );
        record.observation_operator_id = Some("point-to-cell-mean-v1".into());
        assert!(evaluate(&contract(), &[record]).is_ok());
    }

    #[test]
    fn evaluated_support_must_match_the_predictand() {
        let mut record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        record.evaluated_spatial_support_digest = "wrong-grid-cell".into();
        record.observation_operator_id = Some("operator-v1".into());
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::SpatialSupportMismatch]
        );
    }

    #[test]
    fn temporal_support_must_match() {
        let mut record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        record.temporal_support_id = "instantaneous".into();
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::TemporalSupportMismatch]
        );
    }

    #[test]
    fn source_version_and_calibration_epoch_are_frozen() {
        let mut record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        record.source_version = "release-8".into();
        assert_eq!(
            evaluate(&contract(), &[record.clone()]).unwrap_err().violations,
            vec![ContractViolation::SourceVersionMismatch]
        );
        record.source_version = "release-7".into();
        record.calibration_epoch_id = "calibration-2026-02".into();
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::CalibrationEpochMismatch]
        );
    }

    #[test]
    fn post_cutoff_observations_are_rejected() {
        let mut record = pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        record.recorded_at = 101;
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::ObservationRecordedAfterCutoff]
        );
    }

    #[test]
    fn negative_variance_is_rejected() {
        let record = pair("a", 1.0, 2.0, Some(0.0), Some(-1.0), Some(1.0), Some(0.0), "sensor-a");
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::InvalidErrorVariance]
        );
    }

    #[test]
    fn nonfinite_values_are_rejected() {
        let record = pair("a", f64::NAN, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a");
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violations,
            vec![ContractViolation::NonFiniteForecast]
        );
    }

    #[test]
    fn duplicate_case_ids_are_rejected() {
        let data = vec![
            pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a"),
            pair("a", 2.0, 3.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-b"),
        ];
        assert_eq!(
            evaluate(&contract(), &data).unwrap_err().violations,
            vec![ContractViolation::DuplicateCaseId]
        );
    }

    #[test]
    fn empty_pair_set_is_rejected() {
        assert_eq!(
            evaluate(&contract(), &[]).unwrap_err().violations,
            vec![ContractViolation::EmptyFrame]
        );
    }

    #[test]
    fn report_preserves_predictand_units_and_source_lineage() {
        let report = evaluate(
            &contract(),
            &[pair("a", 1.0, 2.0, Some(0.0), Some(1.0), Some(1.0), Some(0.0), "sensor-a")],
        )
        .unwrap();
        assert_eq!(report.predictand_id, "temperature-2m");
        assert_eq!(report.unit, "kelvin");
        assert_eq!(report.observation_source_version, "release-7");
        assert_eq!(report.calibration_epoch_id, "calibration-2026-01");
    }
}
