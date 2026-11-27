use nrvca_validator::comparability::{
    compare_kpi_contracts, MeasurementStatus, TargetPolicyStatus,
};
use serde_json::Value;

const FIXTURE: &str =
    include_str!("../../../../fixtures/nrvca-water-service-kpi-comparability-v1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("water KPI fixture parses")
}

fn status_index(status: MeasurementStatus) -> &'static str {
    status.as_str()
}

#[test]
fn water_kpi_event_boundary_cases_match_all_frozen_known_answers() {
    let root = fixture();
    assert_eq!(root["benchmark_id"], "NRVCA-WATER-KPI-EVENT-SEMANTICS-V1");
    let cases = root["cases"].as_array().expect("cases array");
    assert_eq!(cases.len(), 8);

    for item in cases {
        let result = compare_kpi_contracts(&item["left"], &item["right"]);
        assert_eq!(
            status_index(result.measurement.status),
            item["expected"]["measurement_comparability"].as_str().unwrap(),
            "{}: measurement changed={:?} unknown={:?}",
            item["case_id"],
            result.measurement.changed_fields,
            result.measurement.unknown_fields
        );
        assert_eq!(
            result.target_policy.status.as_str(),
            item["expected"]["target_comparability"].as_str().unwrap(),
            "{}: target changed={:?} unknown={:?}",
            item["case_id"],
            result.target_policy.changed_fields,
            result.target_policy.unknown_fields
        );

        let changed: Vec<&str> = result.measurement.changed_fields.clone();
        let expected_changed: Vec<&str> = item["expected"]["measurement_changed_fields"]
            .as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
        assert_eq!(changed, expected_changed, "{} changed fields", item["case_id"]);

        let unknown: Vec<&str> = result.measurement.unknown_fields.clone();
        let expected_unknown: Vec<&str> = item["expected"]["measurement_unknown_fields"]
            .as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
        assert_eq!(unknown, expected_unknown, "{} unknown fields", item["case_id"]);

        let target_changed: Vec<&str> = result.target_policy.changed_fields.clone();
        let expected_target_changed: Vec<&str> = item["expected"]["target_changed_fields"]
            .as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
        assert_eq!(target_changed, expected_target_changed, "{} target changed fields", item["case_id"]);

        let target_unknown: Vec<&str> = result.target_policy.unknown_fields.clone();
        let expected_target_unknown: Vec<&str> = item["expected"]["target_unknown_fields"]
            .as_array().unwrap().iter().map(|v| v.as_str().unwrap()).collect();
        assert_eq!(target_unknown, expected_target_unknown, "{} target unknown fields", item["case_id"]);
    }
}

#[test]
fn response_repair_and_pressure_restoration_are_distinct_stop_events() {
    let root = fixture();
    for id in [
        "WSR-KPI-001-response-vs-repair",
        "WSR-KPI-002-repair-vs-pressure-restoration",
    ] {
        let item = root["cases"].as_array().unwrap().iter()
            .find(|case| case["case_id"].as_str() == Some(id))
            .expect("expected water KPI case");
        let result = compare_kpi_contracts(&item["left"], &item["right"]);
        assert_eq!(result.measurement.status, MeasurementStatus::NotComparable);
        assert_eq!(result.measurement.changed_fields, vec!["clock_stop"]);
    }
}

#[test]
fn target_change_is_not_misreported_as_measurement_definition_drift() {
    let root = fixture();
    let item = root["cases"].as_array().unwrap().iter()
        .find(|case| case["case_id"].as_str() == Some("WSR-KPI-004-target-policy-only"))
        .expect("target-only case exists");
    let result = compare_kpi_contracts(&item["left"], &item["right"]);
    assert_eq!(result.measurement.status, MeasurementStatus::Comparable);
    assert_eq!(result.target_policy.status, TargetPolicyStatus::ChangedTargetPolicy);
}

#[test]
fn missing_terminal_event_abstains_and_matching_fingerprint_cannot_hide_drift() {
    let root = fixture();
    let cases = root["cases"].as_array().unwrap();
    let missing = cases.iter()
        .find(|case| case["case_id"].as_str() == Some("WSR-KPI-005-missing-stop-event"))
        .expect("missing stop case exists");
    let missing_result = compare_kpi_contracts(&missing["left"], &missing["right"]);
    assert_eq!(missing_result.measurement.status, MeasurementStatus::Indeterminate);
    assert_eq!(missing_result.measurement.unknown_fields, vec!["clock_stop"]);

    let lie = cases.iter()
        .find(|case| case["case_id"].as_str() == Some("WSR-KPI-006-fingerprint-cannot-hide-stop-drift"))
        .expect("fingerprint negative control exists");
    let lie_result = compare_kpi_contracts(&lie["left"], &lie["right"]);
    assert_eq!(lie_result.measurement.status, MeasurementStatus::NotComparable);
    assert_eq!(lie_result.measurement.changed_fields, vec!["clock_stop"]);
}
