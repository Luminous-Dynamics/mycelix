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

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct IntegratedReport {
    pub civil_record: CivilRecordSummary,
    pub topology: TopologySummary,
    pub last_mile: LastMileSummary,
    pub warning_to_action: WarningActionSummary,
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

    let topology_scenario_count = topology_rows
        .iter()
        .map(|row| row.scenario_id.as_str())
        .collect::<std::collections::HashSet<_>>()
        .len() as u32;

    let receiver_denominators_consistent = topology_rows.iter().all(|row| {
        topology_rows
            .iter()
            .filter(|other| other.scenario_id == row.scenario_id)
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
    }

    #[test]
    fn every_module_retains_its_own_expected_fixture_result() {
        let report = integrated_report().unwrap();
        assert!(report.civil_record.integrity_gates_pass);
        assert_eq!(report.topology.scenario_count, 11);
        assert_eq!(report.topology.comparison_rows, 33);
        assert_eq!(report.last_mile.scenario_count, 10);
        assert_eq!(report.warning_to_action.case_count, 11);
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
