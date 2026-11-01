use gcpn_integrated_evaluation::integrated_report;

fn main() {
    match integrated_report() {
        Ok(report) => {
            println!("GCPN integrated synthetic evaluation v0.2");
            println!();
            println!("[1] Civil record integrity");
            println!("  cases: {}", report.civil_record.case_count);
            println!("  hard_gates: {}", if report.civil_record.integrity_gates_pass { "PASS" } else { "FAIL" });
            println!("  unauthorized_alerts: {}", report.civil_record.unauthorized_alerts);
            println!("  authority_unbound_alerts: {}", report.civil_record.authority_unbound_alerts);
            println!("  uncertainty_hidden_alerts: {}", report.civil_record.uncertainty_hidden_alerts);
            println!("  sensitive_provenance_leaks: {}", report.civil_record.sensitive_provenance_leaks);

            println!();
            println!("[2] Topology resilience");
            println!("  scenarios: {}", report.topology.scenario_count);
            println!("  architecture_comparisons: {}", report.topology.comparison_rows);
            println!("  common_receiver_denominators: {}", report.topology.receiver_denominators_consistent);
            println!("  authority_unavailable_blocks_all: {}", report.topology.authority_unavailable_blocks_all);
            println!("  local_first_survives_interregional_partition: {}", report.topology.local_first_survives_interregional_partition);
            println!("  local_first_survives_regional_service_loss: {}", report.topology.local_first_survives_origin_regional_service_loss);

            println!();
            println!("[3] Last-mile delivery and accessibility");
            println!("  scenarios: {}", report.last_mile.scenario_count);
            println!("  intended_receivers_per_scenario: {}", report.last_mile.intended_receivers_per_scenario);
            println!("  counting_invariants: {}", report.last_mile.all_counting_invariants_hold);
            println!("  all_channels_unique_reach: {}", report.last_mile.all_channels_unique_reach);
            println!("  duplicate_channel_receipts: {}", report.last_mile.all_channels_duplicate_receipts);
            println!("  accessible_reach_with_mobile_data_down: {}", report.last_mile.mobile_data_loss_accessible_reach);
            println!("  reached_without_accessible_format: {}", report.last_mile.mobile_data_loss_accessibility_gap);
            println!("  community_relay_loss_uncovered: {}", report.last_mile.community_relay_loss_uncovered);

            println!();
            println!("[4] Warning-to-action evidence");
            println!("  cases: {}", report.warning_to_action.case_count);
            println!("  negative_control_oracle: {}", report.warning_to_action.fixture_oracle_matches);
            println!("  fixture_contains_expected_failures: {}", report.warning_to_action.fixture_contains_expected_failures);
            println!("  valid_funded_action: {}", report.warning_to_action.valid_funded_positive_control_passes);
            println!("  valid_unfunded_action: {}", report.warning_to_action.valid_unfunded_positive_control_passes);
            println!("  legitimate_abstention: {}", report.warning_to_action.authorized_abstention_positive_control_passes);

            println!();
            println!("integrated_fixture_reproduction: {}", if report.all_fixture_oracles_reproduced() { "PASS" } else { "FAIL" });
            println!("interpretation: separate non-compensatory fixture oracles; no blended quality score or real-world performance claim");
            if !report.all_fixture_oracles_reproduced() {
                std::process::exit(1);
            }
        }
        Err(error) => {
            eprintln!("integration error: {error}");
            std::process::exit(2);
        }
    }
}
