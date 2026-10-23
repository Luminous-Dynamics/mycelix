// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use praxis_core::assessment_evidence::AssessmentBenchmarkRun;
use praxis_core::study_context::{
    ObservationStatus, StudyContextIssue, StudyContextRun, StudyContextSummary, StudyPeriod, SupportAvailability,
    validate_study_context, validate_study_context_binding,
};
use serde_json::{Value, json};
use std::collections::BTreeSet;

fn fixture() -> StudyContextRun {
    serde_json::from_str(include_str!(
        "../../../examples/assessment-benchmarks/v1/study_context_synthetic.json"
    ))
    .expect("synthetic study context fixture must parse")
}

fn assessment_fixture() -> AssessmentBenchmarkRun {
    serde_json::from_str(include_str!(
        "../../../examples/assessment-benchmarks/v1/ai_scaffold_fading_synthetic.json"
    ))
    .expect("assessment fixture must parse")
}

fn has_code(result: Result<StudyContextSummary, Vec<StudyContextIssue>>, code: &str) -> bool {
    result
        .err()
        .unwrap_or_default()
        .iter()
        .any(|issue| issue.code == code)
}


#[test]
fn schema_matches_rust_wire_fields_and_enum_values() {
    let schema: Value = serde_json::from_str(include_str!(
        "../../../examples/assessment-benchmarks/v1/study_context.schema.json"
    ))
    .expect("published JSON Schema must parse");
    let run = fixture();
    let sample = serde_json::to_value(&run).expect("fixture serializes to JSON");

    fn assert_keys_match(schema: &Value, instance: &Value) {
        let expected = schema["properties"]
            .as_object()
            .expect("schema properties")
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        let actual = instance
            .as_object()
            .expect("sample object")
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>();
        assert_eq!(actual, expected, "schema and Rust wire fields drifted");
    }

    assert_eq!(schema["properties"]["schema_version"]["const"], 1);
    assert_keys_match(&schema, &sample);
    assert_keys_match(
        &schema["$defs"]["StudyContextObservation"],
        &sample["observations"][0],
    );

    let periods = [
        StudyPeriod::Baseline,
        StudyPeriod::Intervention,
        StudyPeriod::FollowUp7Day,
        StudyPeriod::FollowUp30Day,
    ]
    .into_iter()
    .map(|item| serde_json::to_value(item).unwrap().as_str().unwrap().to_string())
    .collect::<BTreeSet<_>>();
    let schema_periods = schema["$defs"]["StudyPeriod"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_string())
        .collect::<BTreeSet<_>>();
    assert_eq!(periods, schema_periods);

    let availability = [
        SupportAvailability::NotOffered,
        SupportAvailability::Available,
        SupportAvailability::Unavailable,
        SupportAvailability::Unknown,
    ]
    .into_iter()
    .map(|item| serde_json::to_value(item).unwrap().as_str().unwrap().to_string())
    .collect::<BTreeSet<_>>();
    let schema_availability = schema["$defs"]["SupportAvailability"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_string())
        .collect::<BTreeSet<_>>();
    assert_eq!(availability, schema_availability);

    let statuses = [
        ObservationStatus::Complete,
        ObservationStatus::Partial,
        ObservationStatus::NotCollected,
        ObservationStatus::LostToFollowUp,
        ObservationStatus::Withdrawn,
    ]
    .into_iter()
    .map(|item| serde_json::to_value(item).unwrap().as_str().unwrap().to_string())
    .collect::<BTreeSet<_>>();
    let schema_statuses = schema["$defs"]["ObservationStatus"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_string())
        .collect::<BTreeSet<_>>();
    assert_eq!(statuses, schema_statuses);
}

#[test]
fn synthetic_context_fixture_is_structurally_valid() {
    let summary = validate_study_context(&fixture()).expect("synthetic context fixture valid");
    assert_eq!(summary.protocol_id, "PRAxis-AI-SCAFFOLD-001");
    assert_eq!(summary.observation_count, 12);
    assert_eq!(summary.participant_count, 3);
    assert_eq!(summary.missing_or_partial_count, 3);
    assert_eq!(summary.period_counts.get("baseline"), Some(&3));
    assert_eq!(summary.period_counts.get("intervention"), Some(&3));
    assert_eq!(summary.status_counts.get("lost_to_follow_up"), Some(&1));
    assert_eq!(summary.status_counts.get("not_collected"), Some(&1));
}

#[test]
fn null_is_not_silently_interpreted_as_observed_zero() {
    let mut run = fixture();
    assert!(validate_study_context(&run).is_ok());
    run.observations[0].support_invocations = None;
    assert!(has_code(
        validate_study_context(&run),
        "incomplete_complete_observation"
    ));

    run.observations[0].support_invocations = Some(0);
    assert!(validate_study_context(&run).is_ok());
}

#[test]
fn task_completions_cannot_exceed_attempts() {
    let mut run = fixture();
    run.observations[0].completed_learning_tasks = Some(7);
    assert!(has_code(
        validate_study_context(&run),
        "completions_exceed_attempts"
    ));
}

#[test]
fn support_use_is_rejected_when_support_was_unavailable() {
    let mut run = fixture();
    run.observations[0].support_availability = SupportAvailability::Unavailable;
    run.observations[0].support_invocations = Some(1);
    run.observations[0].support_seconds = Some(10);
    assert!(has_code(
        validate_study_context(&run),
        "use_when_support_not_available"
    ));
    assert!(has_code(
        validate_study_context(&run),
        "invocation_without_confirmed_availability"
    ));
}

#[test]
fn participant_assignment_cannot_drift_across_periods() {
    let mut run = fixture();
    run.observations[1].assigned_arm_id = "ai_hint_fade".into();
    assert!(has_code(
        validate_study_context(&run),
        "participant_arm_drift"
    ));
}

#[test]
fn duplicate_participant_period_is_rejected() {
    let mut run = fixture();
    run.observations[1].period = StudyPeriod::Baseline;
    assert!(has_code(
        validate_study_context(&run),
        "duplicate_participant_period"
    ));
}

#[test]
fn missing_follow_up_cannot_be_encoded_as_zero_activity() {
    let mut run = fixture();
    let lost = run
        .observations
        .iter_mut()
        .find(|item| item.observation_status == ObservationStatus::LostToFollowUp)
        .unwrap();
    lost.support_invocations = Some(0);
    assert!(has_code(
        validate_study_context(&run),
        "unobserved_metrics_present"
    ));
}

#[test]
fn context_binds_to_assessment_protocol_participants_arms_and_digest() {
    let context = fixture();
    let assessment = assessment_fixture();
    let digest = "a".repeat(64);
    let summary = validate_study_context_binding(&context, &assessment, &digest)
        .expect("matching assessment and context artifacts should bind");
    assert_eq!(summary.participant_count, 3);
}

#[test]
fn assessment_digest_mismatch_fails_closed() {
    let context = fixture();
    let assessment = assessment_fixture();
    let observed = "b".repeat(64);
    let result = validate_study_context_binding(&context, &assessment, &observed);
    assert!(has_code(result, "assessment_digest_mismatch"));
}

#[test]
fn context_participant_must_exist_in_bound_assessment() {
    let mut context = fixture();
    context.observations[0].participant_ref = "unregistered-synthetic-participant".into();
    let assessment = assessment_fixture();
    let digest = "a".repeat(64);
    let result = validate_study_context_binding(&context, &assessment, &digest);
    assert!(has_code(result, "unknown_assessment_participant"));
}

#[test]
fn context_arm_must_match_bound_assessment_assignment() {
    let mut context = fixture();
    context.observations[0].assigned_arm_id = "ai_direct_answer".into();
    context.observations[1].assigned_arm_id = "ai_direct_answer".into();
    context.observations[2].assigned_arm_id = "ai_direct_answer".into();
    let assessment = assessment_fixture();
    let digest = "a".repeat(64);
    let result = validate_study_context_binding(&context, &assessment, &digest);
    assert!(has_code(result, "assessment_arm_mismatch"));
}

#[test]
fn invalid_digest_and_unknown_wire_fields_are_rejected() {
    let mut run = fixture();
    run.assessment_run_sha256 = "not-a-digest".into();
    assert!(has_code(
        validate_study_context(&run),
        "invalid_assessment_digest"
    ));

    let mut wire = serde_json::to_value(fixture()).unwrap();
    wire["unrecognized_authority"] = json!(true);
    let parsed: Result<StudyContextRun, _> = serde_json::from_value(wire);
    assert!(parsed.is_err(), "unknown wire fields must not be ignored");
}

#[test]
fn assessment_protocol_mismatch_is_rejected() {
    let mut context = fixture();
    context.protocol_id = "different-protocol".into();
    let assessment = assessment_fixture();
    let digest = "a".repeat(64);
    let result = validate_study_context_binding(&context, &assessment, &digest);
    assert!(has_code(result, "protocol_mismatch"));
}
