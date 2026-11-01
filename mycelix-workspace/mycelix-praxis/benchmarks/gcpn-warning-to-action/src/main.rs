use gcpn_warning_to_action::{parse_fixture, fixture_report, FIXTURE};

fn show_mean(value: Option<f64>) -> String {
    value
        .map(|mean| format!("{mean:.2}"))
        .unwrap_or_else(|| "n/a".to_owned())
}

fn main() {
    match (parse_fixture(FIXTURE), fixture_report()) {
        (Ok(cases), Ok(report)) => {
            println!("GCPN synthetic warning-to-action evidence benchmark v0.1.1");
            println!("fixture_oracle: {}", if report.fixture_oracle_matches() { "PASS" } else { "FAIL" });
            println!("cases: {}", report.case_count);
            println!("expected_negative_control_violations:");
            println!("  missing_lineage: {}", report.missing_lineage);
            println!("  unauthorized_decisions: {}", report.unauthorized_decisions);
            println!("  unauthorized_activations: {}", report.unauthorized_activations);
            println!("  unapproved_fund_releases: {}", report.unapproved_fund_releases);
            println!("  unauthorized_action_starts: {}", report.unauthorized_action_starts);
            println!("  false_completions: {}", report.false_completions);
            println!("  population_overclaims: {}", report.population_overclaims);
            println!("  service_accounting_mismatches: {}", report.service_accounting_mismatches);
            println!("  grievance_suppression: {}", report.grievance_suppression);
            println!("  grievance_resolution_overcounts: {}", report.grievance_resolution_overcounts);
            println!("  impact_overclaims: {}", report.impact_overclaims);
            println!("  chronology_violations: {}", report.chronology_violations);
            println!("event_totals_across_fixture_rows (not population estimates):");
            println!("  activations: {}", report.activation_count);
            println!("  funds_released_records: {}", report.funds_released_count);
            println!("  actions_started: {}", report.actions_started_count);
            println!("  actions_completed_assertions: {}", report.actions_completed_count);
            println!("  intended_people_sum: {}", report.intended_people_total);
            println!("  unique_people_served_sum: {}", report.unique_people_served_total);
            println!("mean_synthetic_timing_ticks:");
            println!("  warning_to_decision: {}", show_mean(report.warning_to_decision.mean_ticks()));
            println!("  decision_to_finance: {}", show_mean(report.decision_to_finance.mean_ticks()));
            println!("  finance_to_action: {}", show_mean(report.finance_to_action.mean_ticks()));
            println!("  decision_to_unfunded_action: {}", show_mean(report.decision_to_action.mean_ticks()));
            println!("  action_to_completion: {}", show_mean(report.action_to_completion.mean_ticks()));

            println!();
            println!("case,decision,finance_required,funds_released,action_started,action_completed,intended_people,unique_people_served,service_coverage,duplicate_service_records,grievances_reported,grievances_recorded,impact_claim_submitted");
            for case in &cases {
                let service_coverage = f64::from(case.unique_people_served) / f64::from(case.intended_people);
                println!(
                    "{},{:?},{},{},{},{},{},{},{:.4},{},{},{},{}",
                    case.case_id,
                    case.decision,
                    case.finance_required,
                    case.funds_released,
                    case.action_started,
                    case.action_completed,
                    case.intended_people,
                    case.unique_people_served,
                    service_coverage,
                    case.duplicate_service_records,
                    case.grievances_reported,
                    case.grievances_recorded,
                    case.impact_claim_submitted
                );
            }
            println!("interpretation: negative controls are expected in this fixture; PASS means the evaluator detected their exact oracle counts");
            if !report.fixture_oracle_matches() {
                std::process::exit(1);
            }
        }
        (Err(error), _) | (_, Err(error)) => {
            eprintln!("fixture error: {error}");
            std::process::exit(2);
        }
    }
}
