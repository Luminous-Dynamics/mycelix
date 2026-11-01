use gcpn_lifecycle_cost::{fixture_report, Architecture};

fn show_optional(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.4}"))
        .unwrap_or_else(|| "undefined".to_owned())
}

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("GCPN synthetic lifecycle cost and accessible reach benchmark v0.1");
            println!("cost_records: {}", report.cost_record_count);
            println!("service_outcome_records: {}", report.service_outcome_count);
            println!("scenarios: {}", report.scenario_count);
            println!("architectures: {}", report.architecture_count);
            println!("cost_ranges_monotonic: {}", report.cost_ranges_monotonic);
            println!("service_denominators_consistent: {}", report.service_denominators_consistent);
            println!("outcome_counts_valid: {}", report.outcome_counts_valid);
            println!("all_architectures_have_profiles: {}", report.all_architectures_have_profiles);
            println!("fixture_oracle: {}", if report.fixture_oracle_matches() { "PASS" } else { "FAIL" });

            println!();
            println!("architecture,scenario,horizon_years,discount_rate,intended,unique_reached,accessible_reached,duplicate_receipts,low_lifecycle_pv,base_lifecycle_pv,high_lifecycle_pv,base_annualized_cost,base_cost_per_intended,base_cost_per_unique_reached,base_cost_per_accessible_reached,annual_staff_hours");
            for evaluation in &report.evaluations {
                println!(
                    "{},{},{},{:.4},{},{},{},{},{:.4},{:.4},{:.4},{:.4},{},{},{},{}",
                    evaluation.architecture.label(),
                    evaluation.scenario_id,
                    evaluation.service_horizon_years,
                    evaluation.discount_rate,
                    evaluation.intended_receivers,
                    evaluation.unique_receivers_reached,
                    evaluation.accessible_receivers_reached,
                    evaluation.duplicate_channel_receipts,
                    evaluation.low_lifecycle_present_value,
                    evaluation.base_lifecycle_present_value,
                    evaluation.high_lifecycle_present_value,
                    evaluation.base_annualized_cost,
                    show_optional(evaluation.base_cost_per_intended_receiver),
                    show_optional(evaluation.base_cost_per_unique_receiver_reached),
                    show_optional(evaluation.base_cost_per_accessible_receiver_reached),
                    evaluation.annual_staff_hours
                );
            }

            println!();
            println!("architecture,scenario,component,low_present_value,base_present_value,high_present_value");
            for evaluation in &report.evaluations {
                for component in &evaluation.components {
                    println!(
                        "{},{},{},{:.4},{:.4},{:.4}",
                        evaluation.architecture.label(),
                        evaluation.scenario_id,
                        component.component,
                        component.low_present_value,
                        component.base_present_value,
                        component.high_present_value
                    );
                }
            }
            println!("interpretation: synthetic cost units and service outcomes only; no market price or procurement estimate; no automatic architecture winner");

            if !report.fixture_oracle_matches() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("lifecycle-cost fixture error: {error}");
            std::process::exit(2);
        }
    }
}
