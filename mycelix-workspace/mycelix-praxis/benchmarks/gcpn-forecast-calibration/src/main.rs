use gcpn_forecast_calibration::{
    drift_fixture_report,
    fixture_report,
    Partition,
};

fn show_optional(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.6}"))
        .unwrap_or_else(|| "n/a".to_owned())
}

fn main() {
    match (fixture_report(), drift_fixture_report()) {
        (Ok(report), Ok(drift)) => {
            println!("GCPN synthetic probabilistic forecast verification v0.2");
            println!("records: {}", report.record_count);
            println!("scored_records: {}", report.scored_record_count);
            println!("data_quality_controls:");
            println!("  duplicate_records: {}", report.duplicate_records);
            println!("  late_forecasts: {}", report.late_forecasts);
            println!("  unverified_outcome_records: {}", report.unverified_outcome_records);
            println!("  missing_outcome_records: {}", report.missing_outcome_records);
            println!("  missing_probability_records: {}", report.missing_probability_records);
            println!("  invalid_probability_records: {}", report.invalid_probability_records);
            println!("  invalid_reference_probability_records: {}", report.invalid_reference_probability_records);
            println!("  out_of_domain_records: {}", report.out_of_domain_records);
            println!("  abstentions: {}", report.abstentions);
            println!("  partition_leak_events: {}", report.partition_leak_events);
            println!("  event_metadata_mismatch_events: {}", report.event_metadata_mismatch_events);
            println!("fixture_oracle: {}", if report.fixture_oracle_matches() { "PASS" } else { "FAIL" });

            println!();
            println!("model_summary,model,partition,n,brier,brier_reliability,brier_resolution,brier_uncertainty,brier_residual,distinct_probabilities,log_loss,reference_brier,brier_skill,calibration_in_large,ece,mean_entropy_nats");
            for summary in &report.model_summaries {
                let v = &summary.values;
                println!(
                    "overall,{},{},{},{:.6},{:.6},{:.6},{:.6},{:.12},{},{:.6},{:.6},{},{:.6},{:.6},{:.6}",
                    summary.model_id,
                    summary.partition.label(),
                    v.sample_count,
                    v.brier_score,
                    v.brier_reliability,
                    v.brier_resolution,
                    v.brier_uncertainty,
                    v.brier_decomposition_residual,
                    v.distinct_probability_count,
                    v.log_loss,
                    v.reference_brier_score,
                    show_optional(v.brier_skill_score),
                    v.calibration_in_large,
                    v.expected_calibration_error,
                    v.mean_forecast_entropy_nats
                );
            }

            println!();
            println!("stratum,model,hazard,lead_hours,partition,n,brier,reliability,resolution,uncertainty,residual,brier_skill,ece,mean_probability,event_frequency");
            for stratum in &report.strata {
                let v = &stratum.values;
                println!(
                    "hazard-lead,{},{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.12},{},{:.6},{:.6},{:.6}",
                    stratum.model_id,
                    stratum.hazard,
                    stratum.lead_hours,
                    stratum.partition.label(),
                    v.sample_count,
                    v.brier_score,
                    v.brier_reliability,
                    v.brier_resolution,
                    v.brier_uncertainty,
                    v.brier_decomposition_residual,
                    show_optional(v.brier_skill_score),
                    v.expected_calibration_error,
                    v.mean_forecast_probability,
                    v.observed_event_frequency
                );
            }

            println!();
            println!("drift_profile,n,mean_brier,calibration_in_large,max_cusum,first_drift_sequence,rolling_windows");
            for profile in &drift.profiles {
                let first = profile.first_drift_sequence
                    .map(|value| value.to_string())
                    .unwrap_or_else(|| "none".to_owned());
                println!(
                    "{},{},{:.6},{:.6},{:.6},{},{}",
                    profile.profile_id,
                    profile.sample_count,
                    profile.mean_brier_score,
                    profile.calibration_in_large,
                    profile.max_cusum,
                    first,
                    profile.windows.len()
                );
                for window in &profile.windows {
                    println!(
                        "  window={},start={},end={},n={},brier={:.6},calibration_in_large={:.6},observed_frequency={:.6}",
                        window.window_index,
                        window.start_sequence,
                        window.end_sequence,
                        window.sample_count,
                        window.brier_score,
                        window.calibration_in_large,
                        window.observed_event_frequency
                    );
                }
            }
            println!("drift_fixture_oracle: {}", if drift.fixture_oracle_matches() { "PASS" } else { "FAIL" });
            println!("drift_parameters: window_size=5, cusum_allowance=0.1, threshold=3.0, minimum_observations=5");
            println!("interpretation: scoring decomposition and sequential drift are offline synthetic diagnostics only; no operational invalidation or alert trigger");

            if !report.fixture_oracle_matches() || !drift.fixture_oracle_matches() {
                std::process::exit(1);
            }
        }
        (Err(error), _) | (_, Err(error)) => {
            eprintln!("forecast fixture error: {error}");
            std::process::exit(2);
        }
    }
}
