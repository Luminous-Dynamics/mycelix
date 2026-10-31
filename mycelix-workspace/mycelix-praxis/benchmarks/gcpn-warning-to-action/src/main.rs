use gcpn_warning_to_action::fixture_report;

fn show_mean(value: Option<f64>) -> String {
    value
        .map(|mean| format!("{mean:.2}"))
        .unwrap_or_else(|| "n/a".to_owned())
}

fn main() {
    match fixture_report() {
        Ok(report) => {
            println!("GCPN synthetic warning-to-action evidence benchmark v0.1");
            println!("cases: {}", report.case_count);
            println!("hard_gates: {}", if report.hard_gates_pass() { "PASS" } else { "FAIL" });
            println!("missing_lineage: {}", report.missing_lineage);
            println!("unauthorized_decisions: {}", report.unauthorized_decisions);
            println!("unauthorized_activations: {}", report.unauthorized_activations);
            println!("unapproved_fund_releases: {}", report.unapproved_fund_releases);
            println!("unauthorized_action_starts: {}", report.unauthorized_action_starts);
            println!("false_completions: {}", report.false_completions);
            println!("population_overclaims: {}", report.population_overclaims);
            println!("service_accounting_mismatches: {}", report.service_accounting_mismatches);
            println!("grievance_suppression: {}", report.grievance_suppression);
            println!("grievance_resolution_overcounts: {}", report.grievance_resolution_overcounts);
            println!("impact_overclaims: {}", report.impact_overclaims);
            println!("chronology_violations: {}", report.chronology_violations);
            println!("activations: {}", report.activation_count);
            println!("funds_released_records: {}", report.funds_released_count);
            println!("actions_started: {}", report.actions_started_count);
            println!("actions_completed: {}", report.actions_completed_count);
            println!("intended_people_sum: {}", report.intended_people_total);
            println!("unique_people_served_sum: {}", report.unique_people_served_total);
            println!("service_coverage_ratio_over_fixture_cases: {:.4}", report.service_coverage_ratio());
            println!("warning_to_decision_mean_ticks: {}", show_mean(report.warning_to_decision.mean_ticks()));
            println!("decision_to_finance_mean_ticks: {}", show_mean(report.decision_to_finance.mean_ticks()));
            println!("finance_to_action_mean_ticks: {}", show_mean(report.finance_to_action.mean_ticks()));
            println!("action_to_completion_mean_ticks: {}", show_mean(report.action_to_completion.mean_ticks()));
            println!("interpretation: synthetic process evidence only; no causal impact or lives-saved claim");
            if !report.hard_gates_pass() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
