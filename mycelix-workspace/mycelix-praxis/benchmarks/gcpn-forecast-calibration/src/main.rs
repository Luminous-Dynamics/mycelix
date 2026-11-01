use gcpn_forecast_calibration::{fixture_report, Partition};

fn show_optional(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.6}"))
        .unwrap_or_else(|| "n/a".to_owned())
}

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("GCPN synthetic probabilistic forecast verification v0.1");
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
            println!("model_summary,model,partition,n,brier,log_loss,reference_brier,brier_skill,calibration_in_large,ece,mean_probability,event_frequency,mean_entropy_nats");
            for summary in &report.model_summaries {
                let v = &summary.values;
                println!(
                    "overall,{},{},{},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6}",
                    summary.model_id,
                    summary.partition.label(),
                    v.sample_count,
                    v.brier_score,
                    v.log_loss,
                    v.reference_brier_score,
                    show_optional(v.brier_skill_score),
                    v.calibration_in_large,
                    v.expected_calibration_error,
                    v.mean_forecast_probability,
                    v.observed_event_frequency,
                    v.mean_forecast_entropy_nats
                );
            }

            println!();
            println!("stratum,model,hazard,lead_hours,partition,n,brier,log_loss,reference_brier,brier_skill,calibration_in_large,ece,mean_probability,event_frequency,mean_entropy_nats");
            for stratum in &report.strata {
                let v = &stratum.values;
                println!(
                    "hazard-lead,{},{},{},{},{},{:.6},{:.6},{:.6},{},{:.6},{:.6},{:.6},{:.6},{:.6}",
                    stratum.model_id,
                    stratum.hazard,
                    stratum.lead_hours,
                    stratum.partition.label(),
                    v.sample_count,
                    v.brier_score,
                    v.log_loss,
                    v.reference_brier_score,
                    show_optional(v.brier_skill_score),
                    v.calibration_in_large,
                    v.expected_calibration_error,
                    v.mean_forecast_probability,
                    v.observed_event_frequency,
                    v.mean_forecast_entropy_nats
                );
            }

            println!();
            println!("group,model,hazard,region,lead_hours,partition,n,brier,brier_skill,ece");
            for metric in &report.metrics {
                let v = &metric.values;
                println!(
                    "hazard-region-lead,{},{},{},{},{},{},{:.6},{},{:.6}",
                    metric.model_id,
                    metric.hazard,
                    metric.region,
                    metric.lead_hours,
                    metric.partition.label(),
                    v.sample_count,
                    v.brier_score,
                    show_optional(v.brier_skill_score),
                    v.expected_calibration_error
                );
            }

            let candidate = report.model_summary("candidate", Partition::Evaluation);
            let heat_72 = report.stratum("candidate", "heat", 72, Partition::Evaluation);
            println!();
            println!("subgroup_diagnostic:");
            if let Some(v) = candidate {
                println!("  candidate_evaluation_brier: {:.6}", v.brier_score);
                println!("  candidate_evaluation_brier_skill: {}", show_optional(v.brier_skill_score));
            }
            if let Some(v) = heat_72 {
                println!("  candidate_heat_72h_brier: {:.6}", v.brier_score);
                println!("  candidate_heat_72h_brier_skill: {}", show_optional(v.brier_skill_score));
                println!("  candidate_heat_72h_samples: {}", v.sample_count);
            }
            println!("interpretation: proper scoring and calibration diagnostics on synthetic cases only; no operational forecast or warning policy");

            if !report.fixture_oracle_matches() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("forecast fixture error: {error}");
            std::process::exit(2);
        }
    }
}
