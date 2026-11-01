use gcpn_integrated_evaluation::integrated_report;

fn main() {
    match integrated_report() {
        Ok(report) => {
            println!("GCPN integrated synthetic evaluation v0.4");
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
            println!("[5] Probabilistic forecast verification");
            println!("  records: {}", report.forecast.record_count);
            println!("  scored_records: {}", report.forecast.scored_record_count);
            println!("  fixture_oracle: {}", report.forecast.fixture_oracle_matches);
            println!("  duplicate_records: {}", report.forecast.duplicate_records);
            println!("  late_forecasts: {}", report.forecast.late_forecasts);
            println!("  unverified_outcomes: {}", report.forecast.unverified_outcome_records);
            println!("  missing_outcomes: {}", report.forecast.missing_outcome_records);
            println!("  invalid_probabilities: {}", report.forecast.invalid_probability_records);
            println!("  invalid_reference_probabilities: {}", report.forecast.invalid_reference_probability_records);
            println!("  out_of_domain: {}", report.forecast.out_of_domain_records);
            println!("  abstentions: {}", report.forecast.abstentions);
            println!("  calibration_samples: {}", report.forecast.candidate_calibration_samples);
            println!("  candidate_evaluation_samples: {}", report.forecast.candidate_evaluation_samples);
            println!("  candidate_evaluation_brier: {:.6}", report.forecast.candidate_evaluation_brier);
            println!("  candidate_evaluation_log_loss: {:.6}", report.forecast.candidate_evaluation_log_loss);
            println!("  candidate_evaluation_ece: {:.6}", report.forecast.candidate_evaluation_ece);
            println!("  candidate_evaluation_calibration_in_large: {:.6}", report.forecast.candidate_evaluation_calibration_in_large);
            println!("  candidate_brier_skill_vs_climatology: {:.6}", report.forecast.candidate_evaluation_brier_skill);
            println!("  lead_reference_evaluation_brier: {:.6}", report.forecast.lead_reference_evaluation_brier);
            println!("  lead_reference_brier_skill_vs_climatology: {:.6}", report.forecast.lead_reference_evaluation_brier_skill);
            println!("  heat_72h_candidate_samples: {}", report.forecast.candidate_heat_72h_evaluation_samples);
            println!("  heat_72h_candidate_brier: {:.6}", report.forecast.candidate_heat_72h_evaluation_brier);
            println!("  heat_72h_candidate_brier_skill: {:.6}", report.forecast.candidate_heat_72h_evaluation_brier_skill);

            println!();
            println!("[6] Lifecycle cost and accessible reach");
            println!("  cost_records: {}", report.lifecycle_cost.cost_record_count);
            println!("  service_outcome_records: {}", report.lifecycle_cost.service_outcome_count);
            println!("  scenarios: {}", report.lifecycle_cost.scenario_count);
            println!("  cost_fixture_oracle: {}", report.lifecycle_cost.fixture_oracle_matches);
            println!("  low_base_high_monotonic: {}", report.lifecycle_cost.cost_ranges_monotonic);
            println!("  common_receiver_denominators: {}", report.lifecycle_cost.service_denominators_consistent);
            println!("  zero_reach_unit_cost_undefined: {}", report.lifecycle_cost.zero_accessible_cost_is_undefined);
            println!("  central_hub_restoration_scenario_scoped: {}", report.lifecycle_cost.central_hub_restoration_is_scenario_scoped);
            println!("  centralized_baseline_lifecycle_pv: {:.2}", report.lifecycle_cost.centralized_baseline_lifecycle_pv);
            println!("  regional_baseline_lifecycle_pv: {:.2}", report.lifecycle_cost.regional_baseline_lifecycle_pv);
            println!("  local_first_baseline_lifecycle_pv: {:.2}", report.lifecycle_cost.local_first_baseline_lifecycle_pv);
            println!("  centralized_cost_per_accessible_receiver: {:.4}", report.lifecycle_cost.centralized_cost_per_accessible_receiver);
            println!("  regional_cost_per_accessible_receiver: {:.4}", report.lifecycle_cost.regional_cost_per_accessible_receiver);
            println!("  local_first_cost_per_accessible_receiver: {:.4}", report.lifecycle_cost.local_first_cost_per_accessible_receiver);
            println!("  annual_staff_hours_centralized: {:.0}", report.lifecycle_cost.centralized_staff_hours);
            println!("  annual_staff_hours_regional: {:.0}", report.lifecycle_cost.regional_staff_hours);
            println!("  annual_staff_hours_local_first: {:.0}", report.lifecycle_cost.local_first_staff_hours);

            println!();
            println!("integrated_fixture_reproduction: {}", if report.all_fixture_oracles_reproduced() { "PASS" } else { "FAIL" });
            println!("interpretation: six separate non-compensatory fixture gates; no blended quality score or real-world performance claim");
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
