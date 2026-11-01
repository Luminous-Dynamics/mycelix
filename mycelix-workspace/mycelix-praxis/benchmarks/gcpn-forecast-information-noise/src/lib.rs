//! Known-truth forecast information/noise attribution diagnostics for GCPN.
//!
//! Each case is a weighted vector on a frozen spatial/temporal support.
//! Forecast bias is reported separately and removed before the projection
//! diagnostic. For bias-adjusted forecast anomaly F and verifying anomaly T:
//!
//!   p  = <F,T>_w / ||T||_w
//!   FI = p / ||T||_w
//!   IE = abs(||T||_w - p)
//!   NE = ||F - projection_T(F)||_w
//!
//! Under the same normalized weighted inner product:
//!   debiased_RMSE^2 = IE^2 + NE^2.
//!
//! FI is deliberately not clipped to [-1, 1]. A zero truth-anomaly norm makes
//! projection diagnostics unavailable, not zero. This package does not fit or
//! apply calibrators, create forecasts, or estimate sampling confidence
//! intervals. The independent-system bootstrap remains a separate programme.

use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PredictandContract {
    pub predictand_id: String,
    pub unit: String,
    pub climatology_id: String,
    pub forecast_source_id: String,
    pub forecast_source_version: String,
    pub verification_source_id: String,
    pub verification_source_version: String,
    pub temporal_support_id: String,
    pub spatial_support_digest: String,
    pub evaluation_period_start: u64,
    pub as_of_cutoff: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PostProcessingProvenance {
    pub operation_id: String,
    pub operation_version: String,
    pub fit_data_digest: String,
    /// Latest transaction/valid time used to fit the operation.
    pub trained_through: u64,
    /// Whether confirmatory evaluation outcomes were used in fitting.
    pub uses_evaluation_outcomes: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastCase {
    pub case_id: String,
    pub independent_system_id: String,
    pub forecast_values: Vec<f64>,
    pub verification_values: Vec<f64>,
    pub climatology_values: Vec<f64>,
    /// Positive area/time weights, aligned with all three vectors.
    pub weights: Vec<f64>,
    pub forecast_source_id: String,
    pub forecast_source_version: String,
    pub verification_source_id: String,
    pub verification_source_version: String,
    pub climatology_id: String,
    pub temporal_support_id: String,
    pub spatial_support_digest: String,
    /// Transaction time when the verifying vector became available.
    pub recorded_at: u64,
    pub post_processing: Option<PostProcessingProvenance>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecompositionStatus {
    Qualified,
    DegenerateTruthAnomaly,
}

impl DecompositionStatus {
    pub fn label(self) -> &'static str {
        match self {
            Self::Qualified => "qualified",
            Self::DegenerateTruthAnomaly => "degenerate_truth_anomaly",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CaseMetrics {
    pub case_id: String,
    pub independent_system_id: String,
    pub cells: usize,
    pub total_weight: f64,
    /// Weighted mean forecast minus verifying value, before debiasing.
    pub raw_bias: f64,
    /// RMSE of forecast values against verifying values, before debiasing.
    pub raw_rmse: f64,
    /// RMSE after removing the declared case-mean bias.
    pub debiased_rmse: f64,
    /// Weighted RMS amplitude of the bias-adjusted forecast anomaly.
    pub forecast_activity: f64,
    /// Weighted RMS amplitude of the verifying anomaly.
    pub truth_activity: f64,
    /// Signed projection information. It may be negative or exceed one.
    pub forecast_information: Option<f64>,
    /// Error parallel to the verifying-anomaly direction.
    pub information_error: Option<f64>,
    /// Error orthogonal to the verifying-anomaly direction.
    pub noise_error: Option<f64>,
    pub decomposition_status: DecompositionStatus,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ForecastInformationReport {
    pub predictand_id: String,
    pub unit: String,
    pub climatology_id: String,
    pub forecast_source_id: String,
    pub forecast_source_version: String,
    pub verification_source_id: String,
    pub verification_source_version: String,
    pub temporal_support_id: String,
    pub spatial_support_digest: String,
    pub evaluation_period_start: u64,
    pub as_of_cutoff: u64,
    pub cases: Vec<CaseMetrics>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContractViolation {
    InvalidIdentifier,
    InvalidEvaluationWindow,
    EmptyCaseSet,
    DuplicateCaseId,
    MissingSystemId,
    VectorDimensionMismatch,
    EmptyVector,
    NonFiniteValue,
    InvalidWeight,
    ZeroTotalWeight,
    ForecastSourceMismatch,
    ForecastVersionMismatch,
    VerificationSourceMismatch,
    VerificationVersionMismatch,
    ClimatologyMismatch,
    TemporalSupportMismatch,
    SpatialSupportMismatch,
    VerificationAfterCutoff,
    MissingPostProcessingIdentifier,
    MissingFitDataDigest,
    CalibrationUsedEvaluationOutcomes,
    CalibrationTrainingOverlapsEvaluation,
    NonFiniteMetric,
    DecompositionIdentityFailure,
}

impl ContractViolation {
    pub fn label(self) -> &'static str {
        match self {
            Self::InvalidIdentifier => "invalid_identifier",
            Self::InvalidEvaluationWindow => "invalid_evaluation_window",
            Self::EmptyCaseSet => "empty_case_set",
            Self::DuplicateCaseId => "duplicate_case_id",
            Self::MissingSystemId => "missing_system_id",
            Self::VectorDimensionMismatch => "vector_dimension_mismatch",
            Self::EmptyVector => "empty_vector",
            Self::NonFiniteValue => "non_finite_value",
            Self::InvalidWeight => "invalid_weight",
            Self::ZeroTotalWeight => "zero_total_weight",
            Self::ForecastSourceMismatch => "forecast_source_mismatch",
            Self::ForecastVersionMismatch => "forecast_version_mismatch",
            Self::VerificationSourceMismatch => "verification_source_mismatch",
            Self::VerificationVersionMismatch => "verification_version_mismatch",
            Self::ClimatologyMismatch => "climatology_mismatch",
            Self::TemporalSupportMismatch => "temporal_support_mismatch",
            Self::SpatialSupportMismatch => "spatial_support_mismatch",
            Self::VerificationAfterCutoff => "verification_after_cutoff",
            Self::MissingPostProcessingIdentifier => "missing_postprocessing_identifier",
            Self::MissingFitDataDigest => "missing_fit_data_digest",
            Self::CalibrationUsedEvaluationOutcomes => "calibration_used_evaluation_outcomes",
            Self::CalibrationTrainingOverlapsEvaluation => {
                "calibration_training_overlaps_evaluation"
            }
            Self::NonFiniteMetric => "non_finite_metric",
            Self::DecompositionIdentityFailure => "decomposition_identity_failure",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ContractError {
    pub case_id: Option<String>,
    pub violation: ContractViolation,
}

impl ContractError {
    fn new(case_id: Option<String>, violation: ContractViolation) -> Self {
        Self { case_id, violation }
    }
}

pub fn evaluate(
    contract: &PredictandContract,
    cases: &[ForecastCase],
) -> Result<ForecastInformationReport, ContractError> {
    validate_contract(contract)?;
    if cases.is_empty() {
        return Err(ContractError::new(None, ContractViolation::EmptyCaseSet));
    }

    let mut case_ids = BTreeSet::new();
    let mut metrics = Vec::with_capacity(cases.len());

    for case in cases {
        validate_case(contract, case)?;
        if !case_ids.insert(case.case_id.as_str()) {
            return Err(ContractError::new(
                Some(case.case_id.clone()),
                ContractViolation::DuplicateCaseId,
            ));
        }

        let metric = evaluate_case(case)?;
        metrics.push(metric);
    }

    metrics.sort_by(|left, right| left.case_id.cmp(&right.case_id));

    Ok(ForecastInformationReport {
        predictand_id: contract.predictand_id.clone(),
        unit: contract.unit.clone(),
        climatology_id: contract.climatology_id.clone(),
        forecast_source_id: contract.forecast_source_id.clone(),
        forecast_source_version: contract.forecast_source_version.clone(),
        verification_source_id: contract.verification_source_id.clone(),
        verification_source_version: contract.verification_source_version.clone(),
        temporal_support_id: contract.temporal_support_id.clone(),
        spatial_support_digest: contract.spatial_support_digest.clone(),
        evaluation_period_start: contract.evaluation_period_start,
        as_of_cutoff: contract.as_of_cutoff,
        cases: metrics,
    })
}

fn validate_contract(contract: &PredictandContract) -> Result<(), ContractError> {
    for value in [
        contract.predictand_id.as_str(),
        contract.unit.as_str(),
        contract.climatology_id.as_str(),
        contract.forecast_source_id.as_str(),
        contract.forecast_source_version.as_str(),
        contract.verification_source_id.as_str(),
        contract.verification_source_version.as_str(),
        contract.temporal_support_id.as_str(),
        contract.spatial_support_digest.as_str(),
    ] {
        if !valid_identifier(value) {
            return Err(ContractError::new(
                None,
                ContractViolation::InvalidIdentifier,
            ));
        }
    }
    if contract.evaluation_period_start > contract.as_of_cutoff {
        return Err(ContractError::new(
            None,
            ContractViolation::InvalidEvaluationWindow,
        ));
    }
    Ok(())
}

fn validate_case(contract: &PredictandContract, case: &ForecastCase) -> Result<(), ContractError> {
    let case_id = Some(case.case_id.clone());
    for value in [
        case.case_id.as_str(),
        case.independent_system_id.as_str(),
        case.forecast_source_id.as_str(),
        case.forecast_source_version.as_str(),
        case.verification_source_id.as_str(),
        case.verification_source_version.as_str(),
        case.climatology_id.as_str(),
        case.temporal_support_id.as_str(),
        case.spatial_support_digest.as_str(),
    ] {
        if !valid_identifier(value) {
            return Err(ContractError::new(
                case_id,
                ContractViolation::InvalidIdentifier,
            ));
        }
    }

    if case.recorded_at > contract.as_of_cutoff {
        return Err(ContractError::new(
            case_id,
            ContractViolation::VerificationAfterCutoff,
        ));
    }
    if case.forecast_source_id != contract.forecast_source_id {
        return Err(ContractError::new(
            case_id,
            ContractViolation::ForecastSourceMismatch,
        ));
    }
    if case.forecast_source_version != contract.forecast_source_version {
        return Err(ContractError::new(
            case_id,
            ContractViolation::ForecastVersionMismatch,
        ));
    }
    if case.verification_source_id != contract.verification_source_id {
        return Err(ContractError::new(
            case_id,
            ContractViolation::VerificationSourceMismatch,
        ));
    }
    if case.verification_source_version != contract.verification_source_version {
        return Err(ContractError::new(
            case_id,
            ContractViolation::VerificationVersionMismatch,
        ));
    }
    if case.climatology_id != contract.climatology_id {
        return Err(ContractError::new(
            case_id,
            ContractViolation::ClimatologyMismatch,
        ));
    }
    if case.temporal_support_id != contract.temporal_support_id {
        return Err(ContractError::new(
            case_id,
            ContractViolation::TemporalSupportMismatch,
        ));
    }
    if case.spatial_support_digest != contract.spatial_support_digest {
        return Err(ContractError::new(
            case_id,
            ContractViolation::SpatialSupportMismatch,
        ));
    }

    let length = case.forecast_values.len();
    if length == 0 {
        return Err(ContractError::new(case_id, ContractViolation::EmptyVector));
    }
    if case.verification_values.len() != length
        || case.climatology_values.len() != length
        || case.weights.len() != length
    {
        return Err(ContractError::new(
            case_id,
            ContractViolation::VectorDimensionMismatch,
        ));
    }

    if case
        .forecast_values
        .iter()
        .chain(&case.verification_values)
        .chain(&case.climatology_values)
        .any(|value| !value.is_finite())
    {
        return Err(ContractError::new(
            case_id,
            ContractViolation::NonFiniteValue,
        ));
    }
    if case
        .weights
        .iter()
        .any(|weight| !weight.is_finite() || *weight <= 0.0)
    {
        return Err(ContractError::new(
            case_id,
            ContractViolation::InvalidWeight,
        ));
    }

    let total_weight = case.weights.iter().sum::<f64>();
    if !total_weight.is_finite() {
        return Err(ContractError::new(
            case_id,
            ContractViolation::NonFiniteMetric,
        ));
    }
    if total_weight <= 0.0 {
        return Err(ContractError::new(
            case_id,
            ContractViolation::ZeroTotalWeight,
        ));
    }

    if let Some(post) = &case.post_processing {
        if !valid_identifier(&post.operation_id) || !valid_identifier(&post.operation_version) {
            return Err(ContractError::new(
                case_id,
                ContractViolation::MissingPostProcessingIdentifier,
            ));
        }
        if !valid_identifier(&post.fit_data_digest) {
            return Err(ContractError::new(
                case_id,
                ContractViolation::MissingFitDataDigest,
            ));
        }
        if post.uses_evaluation_outcomes {
            return Err(ContractError::new(
                case_id,
                ContractViolation::CalibrationUsedEvaluationOutcomes,
            ));
        }
        if post.trained_through >= contract.evaluation_period_start {
            return Err(ContractError::new(
                case_id,
                ContractViolation::CalibrationTrainingOverlapsEvaluation,
            ));
        }
    }

    Ok(())
}

fn evaluate_case(case: &ForecastCase) -> Result<CaseMetrics, ContractError> {
    let case_id = Some(case.case_id.clone());
    let total_weight = case.weights.iter().sum::<f64>();

    let mut raw_bias_sum = 0.0;
    let mut raw_squared_error_sum = 0.0;
    let mut truth_activity_squared_sum = 0.0;
    let mut raw_forecast_anomaly_squared_sum = 0.0;

    for index in 0..case.forecast_values.len() {
        let weight = case.weights[index];
        let forecast = case.forecast_values[index];
        let observed = case.verification_values[index];
        let climatology = case.climatology_values[index];
        let raw_error = forecast - observed;
        raw_bias_sum += weight * raw_error;
        raw_squared_error_sum += weight * raw_error * raw_error;

        let truth_anomaly = observed - climatology;
        let forecast_anomaly = forecast - climatology;
        truth_activity_squared_sum += weight * truth_anomaly * truth_anomaly;
        raw_forecast_anomaly_squared_sum += weight * forecast_anomaly * forecast_anomaly;
    }

    let raw_bias = raw_bias_sum / total_weight;
    let raw_rmse = (raw_squared_error_sum / total_weight).sqrt();

    // Remove only the declared case-mean forecast bias. Reliability or
    // calibration gains are not silently folded into this intrinsic diagnostic.
    let truth_activity = (truth_activity_squared_sum / total_weight).sqrt();

    let mut forecast_activity_squared_sum = 0.0;
    let mut dot_product = 0.0;
    let mut debiased_squared_error_sum = 0.0;

    for index in 0..case.forecast_values.len() {
        let weight = case.weights[index];
        let observed = case.verification_values[index];
        let climatology = case.climatology_values[index];
        let truth_anomaly = observed - climatology;
        let forecast_anomaly = case.forecast_values[index] - climatology;

        // Center out the forecast anomaly's spatial mean component and remove
        // the case-mean raw bias. This is a declared diagnostic convention.
        let debiased_forecast_anomaly = forecast_anomaly - raw_bias;
        forecast_activity_squared_sum +=
            weight * debiased_forecast_anomaly * debiased_forecast_anomaly;
        dot_product += weight * debiased_forecast_anomaly * truth_anomaly;
        let error = debiased_forecast_anomaly - truth_anomaly;
        debiased_squared_error_sum += weight * error * error;
    }

    let forecast_activity = (forecast_activity_squared_sum / total_weight).sqrt();
    let debiased_rmse = (debiased_squared_error_sum / total_weight).sqrt();

    let (forecast_information, information_error, noise_error, decomposition_status) =
        if truth_activity_squared_sum <= 0.0 {
            (
                None,
                None,
                None,
                DecompositionStatus::DegenerateTruthAnomaly,
            )
        } else {
            let projection = dot_product / total_weight / truth_activity;
            let forecast_information = projection / truth_activity;
            let information_error = (truth_activity - projection).abs();

            let orthogonal_squared =
                forecast_activity * forecast_activity - projection * projection;
            let tolerance = 1e-12 * forecast_activity.max(1.0).powi(2);
            if orthogonal_squared < -tolerance {
                return Err(ContractError::new(
                    case_id,
                    ContractViolation::DecompositionIdentityFailure,
                ));
            }
            let noise_error = orthogonal_squared.max(0.0).sqrt();

            let reconstructed =
                (information_error * information_error + noise_error * noise_error).sqrt();
            let identity_tolerance = 1e-10 * debiased_rmse.max(1.0);
            if (reconstructed - debiased_rmse).abs() > identity_tolerance {
                return Err(ContractError::new(
                    case_id,
                    ContractViolation::DecompositionIdentityFailure,
                ));
            }

            (
                Some(forecast_information),
                Some(information_error),
                Some(noise_error),
                DecompositionStatus::Qualified,
            )
        };

    for value in [
        raw_bias,
        raw_rmse,
        debiased_rmse,
        forecast_activity,
        truth_activity,
    ] {
        if !value.is_finite() {
            return Err(ContractError::new(
                case_id,
                ContractViolation::NonFiniteMetric,
            ));
        }
    }

    if forecast_information.is_some_and(|value| !value.is_finite())
        || information_error.is_some_and(|value| !value.is_finite())
        || noise_error.is_some_and(|value| !value.is_finite())
    {
        return Err(ContractError::new(
            case_id,
            ContractViolation::NonFiniteMetric,
        ));
    }

    Ok(CaseMetrics {
        case_id: case.case_id.clone(),
        independent_system_id: case.independent_system_id.clone(),
        cells: case.forecast_values.len(),
        total_weight,
        raw_bias,
        raw_rmse,
        debiased_rmse,
        forecast_activity,
        truth_activity,
        forecast_information,
        information_error,
        noise_error,
        decomposition_status,
    })
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
            climatology_id: "climatology-v1".into(),
            forecast_source_id: "forecast-model".into(),
            forecast_source_version: "model-v1".into(),
            verification_source_id: "truth-source".into(),
            verification_source_version: "truth-v1".into(),
            temporal_support_id: "daily-mean".into(),
            spatial_support_digest: "grid-cell-set-1".into(),
            evaluation_period_start: 100,
            as_of_cutoff: 200,
        }
    }

    fn case(
        id: &str,
        forecast: Vec<f64>,
        truth: Vec<f64>,
        climatology: Vec<f64>,
        weights: Vec<f64>,
    ) -> ForecastCase {
        ForecastCase {
            case_id: id.into(),
            independent_system_id: format!("system-{id}"),
            forecast_values: forecast,
            verification_values: truth,
            climatology_values: climatology,
            weights,
            forecast_source_id: "forecast-model".into(),
            forecast_source_version: "model-v1".into(),
            verification_source_id: "truth-source".into(),
            verification_source_version: "truth-v1".into(),
            climatology_id: "climatology-v1".into(),
            temporal_support_id: "daily-mean".into(),
            spatial_support_digest: "grid-cell-set-1".into(),
            recorded_at: 150,
            post_processing: None,
        }
    }

    fn near(left: f64, right: f64) {
        assert!((left - right).abs() < 1e-10, "left={left}, right={right}");
    }

    #[test]
    fn perfect_forecast_has_zero_error_and_unit_information() {
        let record = case(
            "perfect",
            vec![1.0, -1.0, 0.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 1.0],
        );
        let result = evaluate(&contract(), &[record]).unwrap();
        let metrics = &result.cases[0];
        near(metrics.raw_rmse, 0.0);
        near(metrics.debiased_rmse, 0.0);
        near(metrics.forecast_information.unwrap(), 1.0);
        near(metrics.information_error.unwrap(), 0.0);
        near(metrics.noise_error.unwrap(), 0.0);
        assert_eq!(metrics.decomposition_status, DecompositionStatus::Qualified);
    }

    #[test]
    fn smoothing_can_improve_rmse_while_worsening_information_error() {
        let truth = vec![1.0, -1.0, 0.0];
        let climatology = vec![0.0, 0.0, 0.0];
        let weights = vec![1.0, 1.0, 1.0];

        // This forecast has no projection error but carries orthogonal noise.
        let noisy = case(
            "noisy-informative",
            vec![2.0, 0.0, -2.0],
            truth.clone(),
            climatology.clone(),
            weights.clone(),
        );
        // Shrinkage reduces total RMSE but removes half of the signal amplitude.
        let smooth = case(
            "smoothed",
            vec![0.5, -0.5, 0.0],
            truth,
            climatology,
            weights,
        );
        let report = evaluate(&contract(), &[noisy, smooth]).unwrap();
        let noisy_metrics = report
            .cases
            .iter()
            .find(|m| m.case_id == "noisy-informative")
            .unwrap();
        let smooth_metrics = report
            .cases
            .iter()
            .find(|m| m.case_id == "smoothed")
            .unwrap();

        assert!(smooth_metrics.raw_rmse < noisy_metrics.raw_rmse);
        assert!(
            smooth_metrics.information_error.unwrap() > noisy_metrics.information_error.unwrap()
        );
        assert!(smooth_metrics.noise_error.unwrap() < noisy_metrics.noise_error.unwrap());
    }

    #[test]
    fn orthogonal_noise_changes_noise_error_not_projection_information() {
        let record = case(
            "orthogonal",
            vec![2.0, 0.0, -2.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        near(metrics.forecast_information.unwrap(), 1.0);
        near(metrics.information_error.unwrap(), 0.0);
        near(metrics.noise_error.unwrap(), (2.0_f64).sqrt());
    }

    #[test]
    fn overactive_forecast_information_can_exceed_one() {
        let record = case(
            "overactive",
            vec![2.0, -2.0, 0.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        near(metrics.forecast_information.unwrap(), 2.0);
        near(
            metrics.information_error.unwrap(),
            (2.0_f64 * (2.0_f64 / 3.0).sqrt()),
        );
        near(metrics.noise_error.unwrap(), 0.0);
    }

    #[test]
    fn negatively_informative_forecast_is_not_clipped() {
        let record = case(
            "negative-information",
            vec![-1.0, 1.0, 0.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        near(metrics.forecast_information.unwrap(), -1.0);
        near(
            metrics.information_error.unwrap(),
            2.0 * (2.0_f64 / 3.0).sqrt(),
        );
    }

    #[test]
    fn bias_is_reported_separately_and_removed_from_projection() {
        let record = case(
            "biased",
            vec![3.0, 1.0, 2.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        near(metrics.raw_bias, 2.0);
        near(metrics.raw_rmse, 2.0);
        near(metrics.debiased_rmse, 0.0);
        near(metrics.forecast_information.unwrap(), 1.0);
        near(metrics.information_error.unwrap(), 0.0);
    }

    #[test]
    fn zero_truth_anomaly_returns_explicit_degenerate_status() {
        let record = case(
            "zero-truth",
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            vec![1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        assert_eq!(
            metrics.decomposition_status,
            DecompositionStatus::DegenerateTruthAnomaly
        );
        assert_eq!(metrics.forecast_information, None);
        assert_eq!(metrics.information_error, None);
        assert_eq!(metrics.noise_error, None);
        assert!(metrics.debiased_rmse.is_finite());
    }

    #[test]
    fn nonuniform_positive_weights_are_applied_to_projection() {
        let record = case(
            "weighted",
            vec![1.0, -2.0],
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![1.0, 3.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        near(metrics.forecast_information.unwrap(), 1.375);
    }

    #[test]
    fn common_positive_weight_rescaling_preserves_metrics() {
        let base = case(
            "same",
            vec![1.0, 0.0, -1.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 2.0, 3.0],
        );
        let mut scaled = base.clone();
        scaled.weights = scaled.weights.iter().map(|weight| weight * 7.0).collect();
        let first = &evaluate(&contract(), &[base]).unwrap().cases[0];
        let second = &evaluate(&contract(), &[scaled]).unwrap().cases[0];
        near(first.raw_rmse, second.raw_rmse);
        near(first.debiased_rmse, second.debiased_rmse);
        near(
            first.forecast_information.unwrap(),
            second.forecast_information.unwrap(),
        );
        near(
            first.information_error.unwrap(),
            second.information_error.unwrap(),
        );
        near(first.noise_error.unwrap(), second.noise_error.unwrap());
    }

    #[test]
    fn paired_cell_and_weight_reordering_preserves_metrics() {
        let first = case(
            "reorder",
            vec![1.0, 0.0, -1.0],
            vec![1.0, -1.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![1.0, 2.0, 3.0],
        );
        let mut second = first.clone();
        second.forecast_values.reverse();
        second.verification_values.reverse();
        second.climatology_values.reverse();
        second.weights.reverse();
        let a = &evaluate(&contract(), &[first]).unwrap().cases[0];
        let b = &evaluate(&contract(), &[second]).unwrap().cases[0];
        near(a.raw_bias, b.raw_bias);
        near(
            a.forecast_information.unwrap(),
            b.forecast_information.unwrap(),
        );
        near(a.information_error.unwrap(), b.information_error.unwrap());
        near(a.noise_error.unwrap(), b.noise_error.unwrap());
    }

    #[test]
    fn zero_weights_are_rejected() {
        let record = case(
            "zero-weight",
            vec![1.0, -1.0],
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![1.0, 0.0],
        );
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::InvalidWeight
        );
    }

    #[test]
    fn negative_weights_are_rejected() {
        let record = case(
            "negative-weight",
            vec![1.0, -1.0],
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![1.0, -1.0],
        );
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::InvalidWeight
        );
    }

    #[test]
    fn dimension_mismatch_is_rejected() {
        let mut record = case(
            "dimension",
            vec![1.0, -1.0],
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![1.0, 1.0],
        );
        record.weights.pop();
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::VectorDimensionMismatch
        );
    }

    #[test]
    fn nonfinite_values_are_rejected() {
        let record = case(
            "nan",
            vec![f64::NAN, 0.0],
            vec![1.0, -1.0],
            vec![0.0, 0.0],
            vec![1.0, 1.0],
        );
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::NonFiniteValue
        );
    }

    #[test]
    fn all_zero_truth_norm_is_degenerate_not_nan() {
        let record = case(
            "zero",
            vec![0.0, 0.0],
            vec![5.0, 5.0],
            vec![5.0, 5.0],
            vec![1.0, 1.0],
        );
        let metrics = &evaluate(&contract(), &[record]).unwrap().cases[0];
        assert_eq!(metrics.forecast_information, None);
        assert_eq!(
            metrics.decomposition_status,
            DecompositionStatus::DegenerateTruthAnomaly
        );
    }

    #[test]
    fn duplicate_case_ids_are_rejected() {
        let a = case("same", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        let b = a.clone();
        assert_eq!(
            evaluate(&contract(), &[a, b]).unwrap_err().violation,
            ContractViolation::DuplicateCaseId
        );
    }

    #[test]
    fn postprocessing_trained_on_confirmatory_outcomes_is_rejected() {
        let mut record = case("leak", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.post_processing = Some(PostProcessingProvenance {
            operation_id: "calibration-v1".into(),
            operation_version: "v1".into(),
            fit_data_digest: "fit-digest-1".into(),
            trained_through: 90,
            uses_evaluation_outcomes: true,
        });
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::CalibrationUsedEvaluationOutcomes
        );
    }

    #[test]
    fn postprocessing_training_window_must_precede_evaluation() {
        let mut record = case("overlap", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.post_processing = Some(PostProcessingProvenance {
            operation_id: "calibration-v1".into(),
            operation_version: "v1".into(),
            fit_data_digest: "fit-digest-1".into(),
            trained_through: 100,
            uses_evaluation_outcomes: false,
        });
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::CalibrationTrainingOverlapsEvaluation
        );
    }

    #[test]
    fn clean_held_out_postprocessing_provenance_is_accepted() {
        let mut record = case("heldout", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.post_processing = Some(PostProcessingProvenance {
            operation_id: "calibration-v1".into(),
            operation_version: "v1".into(),
            fit_data_digest: "fit-digest-1".into(),
            trained_through: 99,
            uses_evaluation_outcomes: false,
        });
        assert!(evaluate(&contract(), &[record]).is_ok());
    }

    #[test]
    fn source_version_mismatch_is_rejected() {
        let mut record = case("version", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.verification_source_version = "truth-v2".into();
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::VerificationVersionMismatch
        );
    }

    #[test]
    fn support_mismatch_is_rejected() {
        let mut record = case("support", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.spatial_support_digest = "other-grid".into();
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::SpatialSupportMismatch
        );
    }

    #[test]
    fn post_cutoff_verification_is_rejected() {
        let mut record = case("late", vec![1.0], vec![1.0], vec![0.0], vec![1.0]);
        record.recorded_at = 201;
        assert_eq!(
            evaluate(&contract(), &[record]).unwrap_err().violation,
            ContractViolation::VerificationAfterCutoff
        );
    }

    #[test]
    fn empty_case_set_is_rejected() {
        assert_eq!(
            evaluate(&contract(), &[]).unwrap_err().violation,
            ContractViolation::EmptyCaseSet
        );
    }
}
