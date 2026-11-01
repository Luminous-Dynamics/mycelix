use gcpn_forecast_information_noise::{
    evaluate, ForecastCase, PostProcessingProvenance, PredictandContract,
};

fn main() {
    let contract = PredictandContract {
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
    };

    let cases = vec![
        case(
            "noisy-informative",
            vec![2.0, 0.0, -2.0],
            vec![1.0, -1.0, 0.0],
        ),
        case(
            "smoothed",
            vec![0.5, -0.5, 0.0],
            vec![1.0, -1.0, 0.0],
        ),
    ];

    match evaluate(&contract, &cases) {
        Ok(report) => {
            println!("GCPN forecast information/noise attribution v0.1");
            println!("status: deterministic_known_truth_method_demonstration");
            println!("predictand: {} [{}]", report.predictand_id, report.unit);
            println!("forecast: {}@{}", report.forecast_source_id, report.forecast_source_version);
            println!("verification: {}@{}", report.verification_source_id, report.verification_source_version);
            println!("climatology: {}", report.climatology_id);
            println!("evaluation_period_start: {}", report.evaluation_period_start);
            println!("case,raw_bias,raw_rmse,debiased_rmse,forecast_activity,truth_activity,FI,IE,NE,status");
            for item in &report.cases {
                println!(
                    "{},{:.6},{:.6},{:.6},{:.6},{:.6},{:?},{:?},{:?},{}",
                    item.case_id,
                    item.raw_bias,
                    item.raw_rmse,
                    item.debiased_rmse,
                    item.forecast_activity,
                    item.truth_activity,
                    item.forecast_information,
                    item.information_error,
                    item.noise_error,
                    item.decomposition_status.label()
                );
            }
            println!("non_claim: no real forecast superiority or calibration quality is assessed");
        }
        Err(error) => {
            eprintln!("forecast verification contract rejected: {}", error.violation.label());
            std::process::exit(2);
        }
    }
}

fn case(id: &str, forecast: Vec<f64>, truth: Vec<f64>) -> ForecastCase {
    ForecastCase {
        case_id: id.into(),
        independent_system_id: format!("system-{id}"),
        forecast_values: forecast,
        verification_values: truth,
        climatology_values: vec![0.0, 0.0, 0.0],
        weights: vec![1.0, 1.0, 1.0],
        forecast_source_id: "forecast-model".into(),
        forecast_source_version: "model-v1".into(),
        verification_source_id: "truth-source".into(),
        verification_source_version: "truth-v1".into(),
        climatology_id: "climatology-v1".into(),
        temporal_support_id: "daily-mean".into(),
        spatial_support_digest: "grid-cell-set-1".into(),
        recorded_at: 150,
        post_processing: Some(PostProcessingProvenance {
            operation_id: "declared-no-postprocessing".into(),
            operation_version: "v1".into(),
            fit_data_digest: "synthetic-fixture".into(),
            trained_through: 50,
            uses_evaluation_outcomes: false,
        }),
    }
}
