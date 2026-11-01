//! Integrated synthetic GCPN evaluation.
//!
//! The four constituent benchmark modules retain distinct, non-compensatory
//! evidence gates. This crate verifies their composition; it does not create
//! alerts, decide public policy, release funding, or direct civil protection.

use gcpn_civil_conformance::fixture_report as civil_fixture_report;
use gcpn_last_mile::fixture_evaluations as last_mile_fixture_evaluations;
use gcpn_topology_resilience::{
    fixture_comparison as topology_fixture_comparison, Architecture,
};
use gcpn_warning_to_action::{
    evaluate as evaluate_warning_action,
    parse_fixture as parse_warning_action_fixture,
    fixture_report as warning_action_fixture_report,
    FIXTURE as WARNING_ACTION_FIXTURE,
};
use gcpn_forecast_calibration::{
    fixture_report as forecast_fixture_report,
    Partition as ForecastPartition,
};
use gcpn_lifecycle_cost::{
    fixture_report as lifecycle_cost_fixture_report,
    Architecture as CostArchitecture,
};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CivilRecordSummary {
    pub case_count: u32,
    pub integrity_gates_pass: bool,
    pub unauthorized_alerts: u32,
    pub authority_unbound_alerts: u32,
    pub uncertainty_hidden_alerts: u32,
    pub sensitive_provenance_leaks: u32,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TopologySummary {
    pub scenario_count: u32,
    pub comparison_rows: u32,
    pub receiver_denominators_consistent: bool,
    pub authority_unavailable_blocks_all: bool,
    pub local_first_survives_interregional_partition: bool,
    pub local_first_survives_origin_regional_service_loss: bool,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LastMileSummary {
    pub scenario_count: u32,
    pub intended_receivers_per_scenario: u64,
    pub all_counting_invariants_hold: bool,
    pub all_channels_unique_reach: u64,
    pub all_channels_duplicate_receipts: u64,
    pub mobile_data_loss_accessible_reach: u64,
    pub mobile_data_loss_accessibility_gap: u64,
    pub community_relay_loss_uncovered: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct WarningActionSummary {
    pub case_count: u32,
    /// True means that the known negative controls were detected exactly.
    pub fixture_oracle_matches: bool,
    /// The negative-control fixture is expected to contain hard-gate failures.
    pub fixture_contains_expected_failures: bool,
    pub valid_funded_positive_control_passes: bool,
    pub valid_unfunded_positive_control_passes: bool,
    pub authorized_abstention_positive_control_passes: bool,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct ForecastSummary {
    pub record_count: u32,
    pub scored_record_count: u32,
    pub fixture_oracle_matches: bool,
    pub duplicate_records: u32,
    pub late_forecasts: u32,
    pub unverified_outcome_records: u32,
    pub missing_outcome_records: u32,
    pub invalid_probability_records: u32,
    pub invalid_reference_probability_records: u32,
    pub out_of_domain_records: u32,
    pub abstentions: u32,
    pub partition_leak_events: u32,
    pub event_metadata_mismatch_events: u32,
    pub candidate_calibration_samples: u64,
    pub candidate_calibration_brier: f64,
    pub candidate_evaluation_samples: u64,
    pub candidate_evaluation_brier: f64,
    pub candidate_evaluation_log_loss: f64,
    pub candidate_evaluation_ece: f64,
    pub candidate_evaluation_calibration_in_large: f64,
    pub candidate_evaluation_brier_skill: f64,
    pub lead_reference_evaluation_samples: u64,
    pub lead_reference_evaluation_brier: f64,
    pub lead_reference_evaluation_brier_skill: f64,
    pub candidate_heat_72h_evaluation_samples: u64,
    pub candidate_heat_72h_evaluation_brier: f64,
    pub candidate_heat_72h_evaluation_brier_skill: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct LifecycleCostSummary {
    pub cost_record_count: u32,
    pub service_outcome_count: u32,
    pub scenario_count: u32,
    pub fixture_oracle_matches: bool,
    pub cost_ranges_monotonic: bool,
    pub service_denominators_consistent: bool,
    pub outcome_counts_valid: bool,
    pub all_architectures_have_profiles: bool,
    pub centralized_baseline_lifecycle_pv: f64,
    pub regional_baseline_lifecycle_pv: f64,
    pub local_first_baseline_lifecycle_pv: f64,
    pub centralized_baseline_annualized_cost: f64,
    pub regional_baseline_annualized_cost: f64,
    pub local_first_baseline_annualized_cost: f64,
    pub centralized_cost_per_accessible_receiver: f64,
    pub regional_cost_per_accessible_receiver: f64,
    pub local_first_cost_per_accessible_receiver: f64,
    pub centralized_annual_staff_hours: f64,
    pub regional_annual_staff_hours: f64,
    pub local_first_annual_staff_hours: f64,
    pub centralized_staff_hours_over_horizon: f64,
    pub regional_staff_hours_over_horizon: f64,
    pub local_first_staff_hours_over_horizon: f64,
    pub zero_accessible_cost_is_undefined: bool,
    pub central_hub_restoration_is_scenario_scoped: bool,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct IntegratedReport {
    pub civil_record: CivilRecordSummary,
    pub topology: TopologySummary,
    pub last_mile: LastMileSummary,
    pub warning_to_action: WarningActionSummary,
    pub forecast: ForecastSummary,
    pub lifecycle_cost: LifecycleCostSummary,
}

impl IntegratedReport {
    /// Reproduction status, not a weighted system-quality score.
    /// Every constituent oracle remains visible through its own summary.
    pub fn all_fixture_oracles_reproduced(&self) -> bool {
        self.civil_record.integrity_gates_pass
            && self.topology.scenario_count == 11
            && self.topology.comparison_rows == 33
            && self.topology.receiver_denominators_consistent
            && self.topology.authority_unavailable_blocks_all
            && self.topology.local_first_survives_interregional_partition
            && self.topology.local_first_survives_origin_regional_service_loss
            && self.last_mile.scenario_count == 10
            && self.last_mile.intended_receivers_per_scenario == 2_700
            && self.last_mile.all_counting_invariants_hold
            && self.last_mile.all_channels_unique_reach == 2_700
            && self.last_mile.all_channels_duplicate_receipts == 3_750
            && self.last_mile.mobile_data_loss_accessible_reach == 2_450
            && self.last_mile.mobile_data_loss_accessibility_gap == 250
            && self.last_mile.community_relay_loss_uncovered == 500
            && self.warning_to_action.fixture_oracle_matches
            && self.warning_to_action.fixture_contains_expected_failures
            && self.warning_to_action.valid_funded_positive_control_passes
            && self.warning_to_action.valid_unfunded_positive_control_passes
            && self.warning_to_action.authorized_abstention_positive_control_passes
            && self.forecast.fixture_oracle_matches
            && self.forecast.record_count == 65
            && self.forecast.scored_record_count == 54
            && self.forecast.partition_leak_events == 1
            && self.forecast.event_metadata_mismatch_events == 1
            && self.forecast.duplicate_records == 1
            && self.forecast.late_forecasts == 1
            && self.forecast.unverified_outcome_records == 1
            && self.forecast.missing_outcome_records == 1
            && self.forecast.invalid_probability_records == 1
            && self.forecast.out_of_domain_records == 1
            && self.forecast.abstentions == 1
            && self.forecast.candidate_calibration_samples == 6
            && self.forecast.candidate_evaluation_samples == 12
            && self.forecast.lead_reference_evaluation_samples == 12
            && self.forecast.candidate_heat_72h_evaluation_samples == 2
            && (self.forecast.candidate_calibration_brier - 0.01).abs() < 1e-12
            && (self.forecast.candidate_evaluation_brier - 0.17168333333333333).abs() < 1e-12
            && self.forecast.candidate_evaluation_log_loss.is_finite()
            && self.forecast.candidate_evaluation_ece.is_finite()
            && self.forecast.candidate_evaluation_calibration_in_large.is_finite()
            && (self.forecast.candidate_evaluation_brier_skill - 0.3132666666666667).abs() < 1e-12
            && (self.forecast.lead_reference_evaluation_brier_skill - 0.84).abs() < 1e-12
            && (self.forecast.candidate_heat_72h_evaluation_brier_skill - (-2.9204)).abs() < 1e-12
            && (self.forecast.lead_reference_evaluation_brier - 0.04).abs() < 1e-12
            && (self.forecast.candidate_heat_72h_evaluation_brier - 0.9801).abs() < 1e-12
            && self.forecast.candidate_evaluation_brier_skill > 0.0
            && self.forecast.lead_reference_evaluation_brier < self.forecast.candidate_evaluation_brier
            && self.forecast.candidate_heat_72h_evaluation_brier_skill < 0.0
            && self.lifecycle_cost.fixture_oracle_matches
            && self.lifecycle_cost.cost_record_count == 24
            && self.lifecycle_cost.service_outcome_count == 21
            && self.lifecycle_cost.scenario_count == 7
            && self.lifecycle_cost.cost_ranges_monotonic
            && self.lifecycle_cost.service_denominators_consistent
            && self.lifecycle_cost.outcome_counts_valid
            && self.lifecycle_cost.all_architectures_have_profiles
            && (self.lifecycle_cost.centralized_baseline_lifecycle_pv - 28_026.378583683407).abs() < 1e-6
            && (self.lifecycle_cost.regional_baseline_lifecycle_pv - 31_443.190239418658).abs() < 1e-6
            && (self.lifecycle_cost.local_first_baseline_lifecycle_pv - 38_847.468928902585).abs() < 1e-6
            && (self.lifecycle_cost.centralized_cost_per_accessible_receiver - 15.57021032426856).abs() < 1e-6
            && (self.lifecycle_cost.regional_cost_per_accessible_receiver - 16.99631904833441).abs() < 1e-6
            && (self.lifecycle_cost.local_first_cost_per_accessible_receiver - 20.44603627836978).abs() < 1e-6
            && self.lifecycle_cost.centralized_annual_staff_hours == 1_600.0
            && self.lifecycle_cost.regional_annual_staff_hours == 1_950.0
            && self.lifecycle_cost.local_first_annual_staff_hours == 2_400.0
            && self.lifecycle_cost.centralized_staff_hours_over_horizon == 8_000.0
            && self.lifecycle_cost.regional_staff_hours_over_horizon == 9_750.0
            && self.lifecycle_cost.local_first_staff_hours_over_horizon == 12_000.0
            && self.lifecycle_cost.zero_accessible_cost_is_undefined
            && self.lifecycle_cost.central_hub_restoration_is_scenario_scoped
    }
}

pub fn integrated_report() -> Result<IntegratedReport, String> {
    let civil = civil_fixture_report().map_err(|e| format!("civil record fixture: {e}"))?;
    let topology_rows = topology_fixture_comparison()
        .map_err(|e| format!("topology fixture: {e}"))?;
    let last_mile_rows = last_mile_fixture_evaluations()
        .map_err(|e| format!("last-mile fixture: {e}"))?;
    let warning_action = warning_action_fixture_report()
        .map_err(|e| format!("warning-to-action fixture: {e}"))?;
    let forecast = forecast_fixture_report()
        .map_err(|e| format!("forecast-verification fixture: {e}"))?;
    let lifecycle_cost = lifecycle_cost_fixture_report()
        .map_err(|e| format!("lifecycle-cost fixture: {e}"))?;

    let centralized_cost = lifecycle_cost
        .evaluation(CostArchitecture::Centralized, "all-available")
        .ok_or_else(|| "centralized base cost evaluation missing".to_owned())?;
    let regional_cost = lifecycle_cost
        .evaluation(CostArchitecture::Regional, "all-available")
        .ok_or_else(|| "regional base cost evaluation missing".to_owned())?;
    let local_first_cost = lifecycle_cost
        .evaluation(CostArchitecture::LocalFirst, "all-available")
        .ok_or_else(|| "local-first base cost evaluation missing".to_owned())?;
    let zero_reach_cost = lifecycle_cost
        .evaluation(CostArchitecture::Centralized, "origin-authority-unavailable")
        .ok_or_else(|| "zero-reach cost evaluation missing".to_owned())?;
    let central_hub_fault_cost = lifecycle_cost
        .evaluation(CostArchitecture::Centralized, "central-hub-outage")
        .ok_or_else(|| "central hub fault cost evaluation missing".to_owned())?;

    let central_hub_restoration_is_scenario_scoped =
        central_hub_fault_cost.components.iter().any(|component| component.component == "scenario_restoration")
            && centralized_cost.components.iter().all(|component| component.component != "scenario_restoration");

    let candidate_calibration = forecast
        .model_summary("candidate", ForecastPartition::Calibration)
        .ok_or_else(|| "candidate calibration metrics missing".to_owned())?;
    let candidate_evaluation = forecast
        .model_summary("candidate", ForecastPartition::Evaluation)
        .ok_or_else(|| "candidate evaluation metrics missing".to_owned())?;
    let lead_reference_evaluation = forecast
        .model_summary("lead_reference", ForecastPartition::Evaluation)
        .ok_or_else(|| "lead-reference evaluation metrics missing".to_owned())?;
    let candidate_heat_72h = forecast
        .stratum("candidate", "heat", 72, ForecastPartition::Evaluation)
        .ok_or_else(|| "candidate heat/72h evaluation stratum missing".to_owned())?;

    let topology_scenario_count = topology_rows
        .iter()
        .map(|row| row.scenario_id.as_str())
        .collect::<std::collections::HashSet<_>>()
        .len() as u32;

    let receiver_denominators_consistent = topology_rows.iter().all(|row| {
        topology_rows
            .iter()
            .filter(|other| other.scenario_id.as_str() == row.scenario_id.as_str())
            .all(|other| other.outcome.intended_receivers == row.outcome.intended_receivers)
    });

    let authority_unavailable_blocks_all = topology_rows
        .iter()
        .filter(|row| row.scenario_id == "origin-authority-unavailable")
        .all(|row| row.outcome.delivered() == 0);

    let local_first_survives_interregional_partition = topology_rows.iter().any(|row| {
        row.scenario_id == "interregional-partition"
            && row.architecture == Architecture::LocalFirstFederation
            && row.outcome.local_delivered == 1_000
            && row.outcome.remote_delivered() == 0
    });

    let local_first_survives_origin_regional_service_loss = topology_rows.iter().any(|row| {
        row.scenario_id == "origin-regional-service-outage"
            && row.architecture == Architecture::LocalFirstFederation
            && row.outcome.local_delivered == 1_000
            && row.outcome.remote_delivered() == 0
    });

    let intended_receivers_per_scenario = last_mile_rows
        .first()
        .map(|row| row.intended_receivers)
        .unwrap_or(0);

    let all_counting_invariants_hold = last_mile_rows.iter().all(|row| {
        row.intended_receivers == intended_receivers_per_scenario
            && row.unique_receivers_reached <= row.intended_receivers
            && row.accessible_receivers_reached <= row.unique_receivers_reached
            && row.uncovered_receivers
                == row.intended_receivers - row.unique_receivers_reached
            && row.channel_receipt_total() >= row.unique_receivers_reached
            && row.channel_receipt_total() - row.unique_receivers_reached
                == row.duplicate_channel_deliveries
            && row.cohorts.iter().all(|cohort| {
                cohort.accessible_delivered <= cohort.unique_delivered
                    && cohort.unique_delivered <= cohort.intended
            })
    });

    let get_last_mile = |scenario_id: &str| {
        last_mile_rows.iter().find(|row| row.scenario_id.as_str() == scenario_id)
    };

    let all_channels = get_last_mile("all-channels-up");
    let mobile_data_down = get_last_mile("mobile-data-down");
    let community_relay_down = get_last_mile("community-relay-down");

    let action_cases = parse_warning_action_fixture(WARNING_ACTION_FIXTURE)
        .map_err(|e| format!("warning-to-action case parser: {e}"))?;
    let find_action_case = |id: &str| {
        action_cases.iter().find(|case| case.case_id == id)
    };

    let funded_positive_control_passes = find_action_case("valid-complete-chain")
        .map(|case| evaluate_warning_action(std::slice::from_ref(case)).hard_gates_pass())
        .unwrap_or(false);

    let unfunded_positive_control_passes = find_action_case("authorized-unfunded-action")
        .map(|case| evaluate_warning_action(std::slice::from_ref(case)).hard_gates_pass())
        .unwrap_or(false);

    let authorized_abstention_positive_control_passes =
        find_action_case("legitimate-abstention")
            .map(|case| evaluate_warning_action(std::slice::from_ref(case)).hard_gates_pass())
            .unwrap_or(false);

    let report = IntegratedReport {
        civil_record: CivilRecordSummary {
            case_count: civil.cases,
            integrity_gates_pass: civil.conformance_passes(),
            unauthorized_alerts: civil.unauthorized_alerts,
            authority_unbound_alerts: civil.authority_unbound_alerts,
            uncertainty_hidden_alerts: civil.uncertainty_hidden_alerts,
            sensitive_provenance_leaks: civil.sensitive_provenance_leaks,
        },
        topology: TopologySummary {
            scenario_count: topology_scenario_count,
            comparison_rows: topology_rows.len() as u32,
            receiver_denominators_consistent,
            authority_unavailable_blocks_all,
            local_first_survives_interregional_partition,
            local_first_survives_origin_regional_service_loss,
        },
        last_mile: LastMileSummary {
            scenario_count: last_mile_rows.len() as u32,
            intended_receivers_per_scenario,
            all_counting_invariants_hold,
            all_channels_unique_reach: all_channels
                .map(|row| row.unique_receivers_reached)
                .unwrap_or(0),
            all_channels_duplicate_receipts: all_channels
                .map(|row| row.duplicate_channel_deliveries)
                .unwrap_or(0),
            mobile_data_loss_accessible_reach: mobile_data_down
                .map(|row| row.accessible_receivers_reached)
                .unwrap_or(0),
            mobile_data_loss_accessibility_gap: mobile_data_down
                .map(|row| row.reached_without_accessible_format)
                .unwrap_or(0),
            community_relay_loss_uncovered: community_relay_down
                .map(|row| row.uncovered_receivers)
                .unwrap_or(0),
        },
        warning_to_action: WarningActionSummary {
            case_count: warning_action.case_count,
            fixture_oracle_matches: warning_action.fixture_oracle_matches(),
            fixture_contains_expected_failures: !warning_action.hard_gates_pass(),
            valid_funded_positive_control_passes: funded_positive_control_passes,
            valid_unfunded_positive_control_passes: unfunded_positive_control_passes,
            authorized_abstention_positive_control_passes,
        },
        forecast: ForecastSummary {
            record_count: forecast.record_count,
            scored_record_count: forecast.scored_record_count,
            fixture_oracle_matches: forecast.fixture_oracle_matches(),
            duplicate_records: forecast.duplicate_records,
            late_forecasts: forecast.late_forecasts,
            unverified_outcome_records: forecast.unverified_outcome_records,
            missing_outcome_records: forecast.missing_outcome_records,
            invalid_probability_records: forecast.invalid_probability_records,
            invalid_reference_probability_records: forecast.invalid_reference_probability_records,
            out_of_domain_records: forecast.out_of_domain_records,
            abstentions: forecast.abstentions,
            partition_leak_events: forecast.partition_leak_events,
            event_metadata_mismatch_events: forecast.event_metadata_mismatch_events,
            candidate_calibration_samples: candidate_calibration.sample_count,
            candidate_calibration_brier: candidate_calibration.brier_score,
            candidate_evaluation_samples: candidate_evaluation.sample_count,
            candidate_evaluation_brier: candidate_evaluation.brier_score,
            candidate_evaluation_log_loss: candidate_evaluation.log_loss,
            candidate_evaluation_ece: candidate_evaluation.expected_calibration_error,
            candidate_evaluation_calibration_in_large: candidate_evaluation.calibration_in_large,
            candidate_evaluation_brier_skill: candidate_evaluation.brier_skill_score.unwrap_or(0.0),
            lead_reference_evaluation_samples: lead_reference_evaluation.sample_count,
            lead_reference_evaluation_brier: lead_reference_evaluation.brier_score,
            lead_reference_evaluation_brier_skill: lead_reference_evaluation.brier_skill_score.unwrap_or(0.0),
            candidate_heat_72h_evaluation_samples: candidate_heat_72h.sample_count,
            candidate_heat_72h_evaluation_brier: candidate_heat_72h.brier_score,
            candidate_heat_72h_evaluation_brier_skill: candidate_heat_72h.brier_skill_score.unwrap_or(0.0),
        },
        lifecycle_cost: LifecycleCostSummary {
            cost_record_count: lifecycle_cost.cost_record_count,
            service_outcome_count: lifecycle_cost.service_outcome_count,
            scenario_count: lifecycle_cost.scenario_count,
            fixture_oracle_matches: lifecycle_cost.fixture_oracle_matches(),
            cost_ranges_monotonic: lifecycle_cost.cost_ranges_monotonic,
            service_denominators_consistent: lifecycle_cost.service_denominators_consistent,
            outcome_counts_valid: lifecycle_cost.outcome_counts_valid,
            all_architectures_have_profiles: lifecycle_cost.all_architectures_have_profiles,
            centralized_baseline_lifecycle_pv: centralized_cost.base_lifecycle_present_value,
            regional_baseline_lifecycle_pv: regional_cost.base_lifecycle_present_value,
            local_first_baseline_lifecycle_pv: local_first_cost.base_lifecycle_present_value,
            centralized_baseline_annualized_cost: centralized_cost.base_annualized_cost,
            regional_baseline_annualized_cost: regional_cost.base_annualized_cost,
            local_first_baseline_annualized_cost: local_first_cost.base_annualized_cost,
            centralized_cost_per_accessible_receiver: centralized_cost.base_cost_per_accessible_receiver_reached.unwrap_or(0.0),
            regional_cost_per_accessible_receiver: regional_cost.base_cost_per_accessible_receiver_reached.unwrap_or(0.0),
            local_first_cost_per_accessible_receiver: local_first_cost.base_cost_per_accessible_receiver_reached.unwrap_or(0.0),
            centralized_annual_staff_hours: centralized_cost.annual_staff_hours,
            regional_annual_staff_hours: regional_cost.annual_staff_hours,
            local_first_annual_staff_hours: local_first_cost.annual_staff_hours,
            centralized_staff_hours_over_horizon: centralized_cost.staff_hours_over_horizon,
            regional_staff_hours_over_horizon: regional_cost.staff_hours_over_horizon,
            local_first_staff_hours_over_horizon: local_first_cost.staff_hours_over_horizon,
            zero_accessible_cost_is_undefined: zero_reach_cost.base_cost_per_accessible_receiver_reached.is_none(),
            central_hub_restoration_is_scenario_scoped,
        },
    };

    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_component_fixture_fingerprints_are_pinned_at_integration_boundary() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf29ce484222325u64, |mut hash, byte| {
                hash ^= u64::from(*byte);
                hash.wrapping_mul(0x100000001b3)
            })
        }

        assert_eq!(
            fnv1a64(gcpn_civil_conformance::FIXTURE.as_bytes()),
            0x8e5118bf74fc7b13
        );
        assert_eq!(
            fnv1a64(gcpn_topology_resilience::FIXTURE.as_bytes()),
            0xe6f6679f94c93e31
        );
        assert_eq!(
            fnv1a64(gcpn_last_mile::COHORT_FIXTURE.as_bytes()),
            0x59f956045ca16c3d
        );
        assert_eq!(
            fnv1a64(gcpn_last_mile::SCENARIO_FIXTURE.as_bytes()),
            0x2a7269d9a7d4caee
        );
        assert_eq!(
            fnv1a64(gcpn_warning_to_action::FIXTURE.as_bytes()),
            0x4084062a6c04598f
        );
        assert_eq!(
            fnv1a64(gcpn_forecast_calibration::FIXTURE.as_bytes()),
            0x81af0b6779196770
        );
        assert_eq!(
            fnv1a64(gcpn_lifecycle_cost::COST_FIXTURE.as_bytes()),
            0x52072da4d6fb1ba6
        );
        assert_eq!(
            fnv1a64(gcpn_lifecycle_cost::OUTCOME_FIXTURE.as_bytes()),
            0x376c727b26d887aa
        );
    }

    #[test]
    fn fifth_forecast_module_retains_quality_and_subgroup_regression_separately() {
        let report = integrated_report().unwrap();
        assert!(report.forecast.fixture_oracle_matches);
        assert_eq!(report.forecast.record_count, 65);
        assert_eq!(report.forecast.scored_record_count, 54);
        assert_eq!(report.lifecycle_cost.cost_record_count, 24);
        assert_eq!(report.forecast.candidate_calibration_samples, 6);
        assert_eq!(report.forecast.candidate_evaluation_samples, 12);
        assert_eq!(report.forecast.lead_reference_evaluation_samples, 12);
        assert_eq!(report.forecast.candidate_heat_72h_evaluation_samples, 2);
        assert!(report.forecast.candidate_evaluation_brier_skill > 0.0);
        assert!(report.forecast.lead_reference_evaluation_brier < report.forecast.candidate_evaluation_brier);
        assert!(report.forecast.candidate_heat_72h_evaluation_brier_skill < 0.0);
    }

    #[test]
    fn forecast_subgroup_failure_cannot_be_masked_by_the_other_four_modules() {
        let mut report = integrated_report().unwrap();
        assert!(report.all_fixture_oracles_reproduced());

        report.forecast.candidate_heat_72h_evaluation_brier_skill = 0.0;
        assert!(!report.all_fixture_oracles_reproduced());
        assert!(report.civil_record.integrity_gates_pass);
        assert!(report.topology.receiver_denominators_consistent);
        assert!(report.last_mile.all_counting_invariants_hold);
        assert!(report.warning_to_action.fixture_oracle_matches);
    }

    #[test]
    fn lifecycle_cost_remains_separate_from_reach_and_accessibility() {
        let report = integrated_report().unwrap();
        assert!(report.lifecycle_cost.fixture_oracle_matches);
        assert_eq!(report.lifecycle_cost.cost_record_count, 24);
        assert_eq!(report.lifecycle_cost.service_outcome_count, 21);
        assert_eq!(report.lifecycle_cost.scenario_count, 7);
        assert!(report.lifecycle_cost.cost_ranges_monotonic);
        assert!(report.lifecycle_cost.service_denominators_consistent);
        assert!(report.lifecycle_cost.outcome_counts_valid);
        assert!(report.lifecycle_cost.all_architectures_have_profiles);
        assert!(report.lifecycle_cost.zero_accessible_cost_is_undefined);
        assert!(report.lifecycle_cost.central_hub_restoration_is_scenario_scoped);
        assert!(report.lifecycle_cost.centralized_baseline_lifecycle_pv
            < report.lifecycle_cost.local_first_baseline_lifecycle_pv);
        assert!(report.lifecycle_cost.centralized_cost_per_accessible_receiver
            < report.lifecycle_cost.local_first_cost_per_accessible_receiver);
    }

    #[test]
    fn lifecycle_cost_failure_cannot_be_offset_by_the_other_five_modules() {
        let mut report = integrated_report().unwrap();
        assert!(report.all_fixture_oracles_reproduced());

        report.lifecycle_cost.cost_ranges_monotonic = false;
        assert!(!report.all_fixture_oracles_reproduced());
        assert!(report.civil_record.integrity_gates_pass);
        assert!(report.topology.receiver_denominators_consistent);
        assert!(report.last_mile.all_counting_invariants_hold);
        assert!(report.warning_to_action.fixture_oracle_matches);
        assert!(report.forecast.fixture_oracle_matches);
    }

    #[test]
    fn every_module_retains_its_own_expected_fixture_result() {
        let report = integrated_report().unwrap();
        assert!(report.civil_record.integrity_gates_pass);
        assert_eq!(report.topology.scenario_count, 11);
        assert_eq!(report.topology.comparison_rows, 33);
        assert_eq!(report.last_mile.scenario_count, 10);
        assert_eq!(report.warning_to_action.case_count, 11);
        assert_eq!(report.forecast.record_count, 65);
        assert_eq!(report.forecast.scored_record_count, 54);
        assert!(report.warning_to_action.fixture_oracle_matches);
        assert!(report.warning_to_action.fixture_contains_expected_failures);
    }

    #[test]
    fn topology_authority_and_local_continuity_invariants_compose() {
        let report = integrated_report().unwrap();
        assert!(report.topology.receiver_denominators_consistent);
        assert!(report.topology.authority_unavailable_blocks_all);
        assert!(report.topology.local_first_survives_interregional_partition);
        assert!(report.topology.local_first_survives_origin_regional_service_loss);
    }

    #[test]
    fn last_mile_deduplication_and_accessibility_gaps_remain_visible() {
        let report = integrated_report().unwrap();
        assert!(report.last_mile.all_counting_invariants_hold);
        assert_eq!(report.last_mile.all_channels_unique_reach, 2_700);
        assert_eq!(report.last_mile.all_channels_duplicate_receipts, 3_750);
        assert_eq!(report.last_mile.mobile_data_loss_accessible_reach, 2_450);
        assert_eq!(report.last_mile.mobile_data_loss_accessibility_gap, 250);
        assert_eq!(report.last_mile.community_relay_loss_uncovered, 500);
    }

    #[test]
    fn funded_and_unfunded_action_positive_controls_are_both_supported() {
        let report = integrated_report().unwrap();
        assert!(report.warning_to_action.valid_funded_positive_control_passes);
        assert!(report.warning_to_action.valid_unfunded_positive_control_passes);
        assert!(report.warning_to_action.authorized_abstention_positive_control_passes);
    }

    #[test]
    fn one_module_failure_cannot_be_offset_by_another_module_success() {
        let mut report = integrated_report().unwrap();
        assert!(report.all_fixture_oracles_reproduced());

        report.last_mile.all_counting_invariants_hold = false;
        assert!(!report.all_fixture_oracles_reproduced());
        assert!(report.civil_record.integrity_gates_pass);
        assert!(report.topology.receiver_denominators_consistent);
        assert!(report.warning_to_action.fixture_oracle_matches);
    }

    #[test]
    fn all_fixture_oracles_reproduce_without_a_weighted_composite_score() {
        let report = integrated_report().unwrap();
        assert!(report.all_fixture_oracles_reproduced());
        // Module metrics are still exposed separately; no weighted score is computed.
        assert_eq!(report.civil_record.case_count, 9);
        assert_eq!(report.topology.comparison_rows, 33);
        assert_eq!(report.last_mile.intended_receivers_per_scenario, 2_700);
        assert_eq!(report.warning_to_action.case_count, 11);
    }
}
