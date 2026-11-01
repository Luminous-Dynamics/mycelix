use gcpn_reference_observation_uncertainty::{
    ForecastObservationPair, ObservationErrorModel, PredictandContract, evaluate,
};

fn main() {
    let contract = PredictandContract {
        predictand_id: "temperature-2m".into(),
        unit: "kelvin".into(),
        temporal_support_id: "hourly-mean".into(),
        forecast_spatial_support_digest: "grid-cell-001".into(),
        observation_source_id: "station-network".into(),
        observation_source_version: "release-7".into(),
        calibration_epoch_id: "calibration-2026-01".into(),
        observation_model_id: "known-additive-error-v1".into(),
        as_of_cutoff: 100,
    };

    let pairs = vec![
        record("case-1", 291.0, 293.0, "sensor-lineage-a"),
        record("case-2", 292.0, 294.0, "sensor-lineage-b"),
        record("case-3", 289.0, 291.0, "sensor-lineage-c"),
    ];

    match evaluate(&contract, &pairs) {
        Ok(report) => {
            println!("GCPN reference-observation qualification v0.1");
            println!("status: synthetic_known_error_method_demonstration");
            println!("predictand: {} [{}]", report.predictand_id, report.unit);
            println!(
                "reference: {}@{} calibration={}",
                report.observation_source_id,
                report.observation_source_version,
                report.calibration_epoch_id
            );
            println!("as_of_cutoff: {}", report.as_of_cutoff);
            println!("cases: {}", report.cases);
            println!("naive_observation_mse: {:.8}", report.naive_observation_mse);
            println!(
                "error_corrected_latent_mse_estimator: {:?}",
                report.error_corrected_mse
            );
            println!(
                "correction_unavailable_reason: {:?}",
                report.correction_unavailable_reason
            );
            println!(
                "mean_known_bias: {:?}",
                report.error_components.mean_known_bias
            );
            println!(
                "mean_measurement_variance: {:?}",
                report.error_components.mean_measurement_variance
            );
            println!(
                "mean_representativeness_variance: {:?}",
                report.error_components.mean_representativeness_variance
            );
            println!(
                "mean_twice_cross_covariance: {:?}",
                report.error_components.mean_twice_cross_covariance
            );
            println!(
                "unique_error_lineage_roots: {}",
                report.error_components.unique_error_lineage_roots
            );
            println!(
                "non_claim: no universal latent-truth identification or forecast skill is asserted"
            );
        }
        Err(error) => {
            eprintln!("observation contract rejected: {:?}", error.violations);
            std::process::exit(2);
        }
    }
}

fn record(case_id: &str, forecast: f64, observed: f64, lineage: &str) -> ForecastObservationPair {
    ForecastObservationPair {
        case_id: case_id.into(),
        independent_system_id: "synthetic-system".into(),
        forecast_value: forecast,
        observed_value: observed,
        source_id: "station-network".into(),
        source_version: "release-7".into(),
        calibration_epoch_id: "calibration-2026-01".into(),
        observation_model_id: "known-additive-error-v1".into(),
        temporal_support_id: "hourly-mean".into(),
        native_spatial_support_digest: "grid-cell-001".into(),
        evaluated_spatial_support_digest: "grid-cell-001".into(),
        observation_operator_id: None,
        recorded_at: 90,
        error_lineage_root: lineage.into(),
        error_model: ObservationErrorModel {
            known_bias: Some(2.0),
            measurement_variance: Some(0.25),
            representativeness_variance: Some(0.5),
            cross_error_covariance: Some(0.0),
        },
    }
}
