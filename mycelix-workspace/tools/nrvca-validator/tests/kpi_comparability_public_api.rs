use nrvca_validator::comparability::{
    compare_kpi_contracts, MeasurementStatus, TargetPolicyStatus, MEASUREMENT_FIELDS,
    TARGET_POLICY_FIELDS,
};
use serde_json::{json, Value};

const FIXTURE: &str = include_str!("../../../../fixtures/nrvca-kpi-dual-axis-comparability-synthetic-v0.1.json");

fn fixture() -> Value {
    serde_json::from_str(FIXTURE).expect("frozen synthetic benchmark parses")
}

fn case<'a>(root: &'a Value, id: &str) -> &'a Value {
    root["cases"].as_array().expect("cases array")
        .iter().find(|item| item["case_id"].as_str() == Some(id))
        .unwrap_or_else(|| panic!("benchmark case missing: {id}"))
}

#[test]
fn public_api_matches_all_24_frozen_known_answers_and_aggregate_counts() {
    let root = fixture();
    let mut measurement = [0usize; 3];
    let mut target = [0usize; 3];

    for item in root["cases"].as_array().expect("cases array") {
        let result = compare_kpi_contracts(&item["left"], &item["right"]);
        assert_eq!(
            result.measurement.status.as_str(),
            item["expected"]["measurement_comparability"].as_str().unwrap(),
            "{} measurement evidence: changed={:?}, unknown={:?}",
            item["case_id"], result.measurement.changed_fields, result.measurement.unknown_fields
        );
        assert_eq!(
            result.target_policy.status.as_str(),
            item["expected"]["target_comparability"].as_str().unwrap(),
            "{} target evidence: changed={:?}, unknown={:?}",
            item["case_id"], result.target_policy.changed_fields, result.target_policy.unknown_fields
        );

        match result.measurement.status {
            MeasurementStatus::Comparable => measurement[0] += 1,
            MeasurementStatus::NotComparable => measurement[1] += 1,
            MeasurementStatus::Indeterminate => measurement[2] += 1,
        }
        match result.target_policy.status {
            TargetPolicyStatus::SameTargetPolicy => target[0] += 1,
            TargetPolicyStatus::ChangedTargetPolicy => target[1] += 1,
            TargetPolicyStatus::Indeterminate => target[2] += 1,
        }
    }

    assert_eq!(measurement, [10, 12, 2]);
    assert_eq!(target, [18, 4, 2]);
}

#[test]
fn changing_each_critical_measurement_field_is_detected() {
    let root = fixture();
    let base = case(&root, "exact-same-contract");
    for field in MEASUREMENT_FIELDS {
        let left = base["left"].clone();
        let mut right = base["right"].clone();
        right["measurement"][field] = json!(format!("deliberate-mismatch-{field}"));
        let result = compare_kpi_contracts(&left, &right);
        assert_eq!(result.measurement.status, MeasurementStatus::NotComparable, "{field}");
        assert!(result.measurement.changed_fields.contains(&field), "{field}");
    }
}

#[test]
fn changing_each_target_policy_field_is_detected_only_on_target_axis() {
    let root = fixture();
    let base = case(&root, "exact-same-contract");
    for field in TARGET_POLICY_FIELDS {
        let left = base["left"].clone();
        let mut right = base["right"].clone();
        right["target"][field] = if field == "value" { json!(91) } else { json!(format!("changed-{field}")) };
        let result = compare_kpi_contracts(&left, &right);
        assert_eq!(result.measurement.status, MeasurementStatus::Comparable, "{field}");
        assert_eq!(result.target_policy.status, TargetPolicyStatus::ChangedTargetPolicy, "{field}");
        assert!(result.target_policy.changed_fields.contains(&field), "{field}");
    }
}

#[test]
fn missing_and_null_fields_abstain_but_known_mismatch_takes_precedence() {
    let root = fixture();
    let base = case(&root, "exact-same-contract");
    let left = base["left"].clone();
    let mut right = base["right"].clone();

    right["measurement"]["denominator_definition"] = Value::Null;
    let missing_only = compare_kpi_contracts(&left, &right);
    assert_eq!(missing_only.measurement.status, MeasurementStatus::Indeterminate);
    assert!(missing_only.measurement.unknown_fields.contains(&"denominator_definition"));

    right["measurement"]["unit"] = json!("hours");
    let mismatch_and_unknown = compare_kpi_contracts(&left, &right);
    assert_eq!(mismatch_and_unknown.measurement.status, MeasurementStatus::NotComparable);
    assert!(mismatch_and_unknown.measurement.changed_fields.contains(&"unit"));
    assert!(mismatch_and_unknown.measurement.unknown_fields.contains(&"denominator_definition"));
}

#[test]
fn labels_local_ids_source_vintages_and_declared_fingerprints_are_not_trusted() {
    let root = fixture();
    for id in ["wording-only-label-change", "source-vintage-only", "local-id-only"] {
        let item = case(&root, id);
        assert_eq!(
            compare_kpi_contracts(&item["left"], &item["right"]).measurement.status,
            MeasurementStatus::Comparable,
            "{id}"
        );
    }

    let lie = case(&root, "fingerprint-lie");
    let result = compare_kpi_contracts(&lie["left"], &lie["right"]);
    assert_eq!(result.measurement.status, MeasurementStatus::NotComparable);
    assert!(!result.measurement.changed_fields.is_empty());
}

#[test]
fn unknown_axis_shapes_fail_closed_and_receipts_are_deterministic() {
    let missing_left = json!({});
    let missing_right = json!({});
    let first = compare_kpi_contracts(&missing_left, &missing_right);
    let second = compare_kpi_contracts(&missing_left, &missing_right);
    assert_eq!(first, second);
    assert_eq!(first.measurement.status, MeasurementStatus::Indeterminate);
    assert_eq!(first.target_policy.status, TargetPolicyStatus::Indeterminate);

    let root = fixture();
    let item = case(&root, "fingerprint-lie");
    assert_eq!(
        compare_kpi_contracts(&item["left"], &item["right"]),
        compare_kpi_contracts(&item["left"], &item["right"])
    );
}

#[test]
fn decisions_and_field_receipts_are_symmetric_for_all_frozen_pairs() {
    let root = fixture();
    for item in root["cases"].as_array().expect("cases array") {
        let forward = compare_kpi_contracts(&item["left"], &item["right"]);
        let reverse = compare_kpi_contracts(&item["right"], &item["left"]);
        assert_eq!(forward.measurement.status, reverse.measurement.status, "{}", item["case_id"]);
        assert_eq!(forward.measurement.changed_fields, reverse.measurement.changed_fields, "{}", item["case_id"]);
        assert_eq!(forward.measurement.unknown_fields, reverse.measurement.unknown_fields, "{}", item["case_id"]);
        assert_eq!(forward.measurement.unsupported_fields, reverse.measurement.unsupported_fields, "{}", item["case_id"]);
        assert_eq!(forward.target_policy.status, reverse.target_policy.status, "{}", item["case_id"]);
        assert_eq!(forward.target_policy.changed_fields, reverse.target_policy.changed_fields, "{}", item["case_id"]);
        assert_eq!(forward.target_policy.unknown_fields, reverse.target_policy.unknown_fields, "{}", item["case_id"]);
        assert_eq!(forward.target_policy.unsupported_fields, reverse.target_policy.unsupported_fields, "{}", item["case_id"]);
    }
}

#[test]
fn new_unversioned_policy_fields_force_abstention() {
    let root = fixture();
    let base = case(&root, "exact-same-contract");
    let mut left = base["left"].clone();
    let mut right = base["right"].clone();
    left["target"]["comparison_operator"] = json!("at_least");
    right["target"]["comparison_operator"] = json!("at_most");

    let result = compare_kpi_contracts(&left, &right);
    assert_eq!(result.measurement.status, MeasurementStatus::Comparable);
    assert_eq!(result.target_policy.status, TargetPolicyStatus::Indeterminate);
    assert_eq!(
        result.target_policy.unsupported_fields,
        vec!["comparison_operator".to_owned()]
    );
}
