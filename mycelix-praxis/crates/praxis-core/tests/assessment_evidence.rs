// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use praxis_core::assessment_evidence::{
    AssessmentBenchmarkRun, AssessmentPhase, AssistanceMode, ValidationIssue, ValidationSummary,
    validate_assessment_benchmark,
};
use std::collections::BTreeSet;

const DAY_SECONDS: i64 = 86_400;

fn fixture() -> AssessmentBenchmarkRun {
    serde_json::from_str(include_str!(
        "../../../examples/assessment-benchmarks/v1/ai_scaffold_fading_synthetic.json"
    ))
    .expect("synthetic benchmark fixture must parse")
}

fn has_code(result: Result<ValidationSummary, Vec<ValidationIssue>>, code: &str) -> bool {
    result
        .err()
        .unwrap_or_default()
        .iter()
        .any(|issue| issue.code == code)
}

#[test]
fn json_schema_matches_rust_wire_fields_and_enum_values() {
    let schema: serde_json::Value = serde_json::from_str(include_str!(
        "../../../examples/assessment-benchmarks/v1/assessment_benchmark.schema.json"
    ))
    .expect("published JSON Schema must parse");
    let sample = serde_json::to_value(fixture()).expect("fixture serializes to JSON");

    fn assert_keys_match(schema: &serde_json::Value, instance: &serde_json::Value) {
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
    assert_keys_match(&schema["$defs"]["BenchmarkArm"], &sample["arms"][0]);
    assert_keys_match(
        &schema["$defs"]["AssessmentEpisode"],
        &sample["episodes"][0],
    );
    assert_keys_match(
        &schema["$defs"]["AssessmentAttempt"],
        &sample["episodes"][0]["attempts"][0],
    );

    let phase_names = [
        AssessmentPhase::Baseline,
        AssessmentPhase::Practice,
        AssessmentPhase::ImmediatePost,
        AssessmentPhase::Transfer,
        AssessmentPhase::Retention14Day,
        AssessmentPhase::Retention30Day,
    ]
    .into_iter()
    .map(|phase| {
        serde_json::to_value(phase)
            .unwrap()
            .as_str()
            .unwrap()
            .to_string()
    })
    .collect::<BTreeSet<_>>();
    let schema_phases = schema["$defs"]["AssessmentPhase"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_string())
        .collect::<BTreeSet<_>>();
    assert_eq!(
        phase_names, schema_phases,
        "phase enum drifted from JSON Schema"
    );

    let assistance_names = [
        AssistanceMode::Unaided,
        AssistanceMode::Hint,
        AssistanceMode::Scaffolded,
        AssistanceMode::ToolAssisted,
        AssistanceMode::DirectAnswer,
        AssistanceMode::Unknown,
    ]
    .into_iter()
    .map(|assistance| {
        serde_json::to_value(assistance)
            .unwrap()
            .as_str()
            .unwrap()
            .to_string()
    })
    .collect::<BTreeSet<_>>();
    let schema_assistance = schema["$defs"]["AssistanceMode"]["enum"]
        .as_array()
        .unwrap()
        .iter()
        .map(|value| value.as_str().unwrap().to_string())
        .collect::<BTreeSet<_>>();
    assert_eq!(
        assistance_names, schema_assistance,
        "assistance enum drifted from JSON Schema"
    );
}

#[test]
fn synthetic_comparison_fixture_is_structurally_valid() {
    let summary = validate_assessment_benchmark(&fixture()).expect("fixture valid");
    assert_eq!(summary.episode_count, 3);
    assert_eq!(summary.attempt_count, 21);
    assert_eq!(summary.arm_counts.get("active_retrieval"), Some(&1));
    assert_eq!(summary.arm_counts.get("ai_hint_fade"), Some(&1));
    assert_eq!(summary.arm_counts.get("ai_direct_answer"), Some(&1));
}

#[test]
fn assisted_transfer_is_rejected() {
    let mut run = fixture();
    run.episodes[1]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Transfer)
        .unwrap()
        .assistance = AssistanceMode::DirectAnswer;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "assisted_independent_phase"
    ));
}

#[test]
fn practice_form_cannot_be_reused_for_transfer() {
    let mut run = fixture();
    let form = run.episodes[0]
        .attempts
        .iter()
        .find(|a| a.phase == AssessmentPhase::Practice)
        .unwrap()
        .task_form_id
        .clone();
    run.episodes[0]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Transfer)
        .unwrap()
        .task_form_id = form;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "practice_form_reused_as_anchor"
    ));
}

#[test]
fn high_score_cannot_make_early_retention_valid() {
    let mut run = fixture();
    let last = run.episodes[0]
        .attempts
        .iter()
        .filter(|a| a.phase == AssessmentPhase::Practice)
        .map(|a| a.completed_at_unix_seconds)
        .max()
        .unwrap();
    let retention = run.episodes[0]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Retention14Day)
        .unwrap();
    retention.completed_at_unix_seconds = last + 13 * DAY_SECONDS;
    retention.score_permille = 1000;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "retention_too_early"
    ));
}

#[test]
fn malformed_negative_timestamp_is_rejected_without_panicking() {
    let mut run = fixture();
    run.episodes[0]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Retention14Day)
        .unwrap()
        .completed_at_unix_seconds = i64::MIN;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "invalid_timestamp"
    ));
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "retention_too_early"
    ));
}

#[test]
fn unknown_assistance_is_not_silently_treated_as_unaided() {
    let mut run = fixture();
    run.episodes[0]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Practice)
        .unwrap()
        .assistance = AssistanceMode::Unknown;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "unknown_assistance"
    ));
}

#[test]
fn rubric_mismatch_is_rejected() {
    let mut run = fixture();
    run.episodes[2].attempts[0].rubric_version = "rubric-v0".into();
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "rubric_version_mismatch"
    ));
}

#[test]
fn scores_above_one_thousand_are_rejected() {
    let mut run = fixture();
    run.episodes[1].attempts[0].score_permille = 1001;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "score_out_of_range"
    ));
}

#[test]
fn duplicate_attempt_ids_are_rejected() {
    let mut run = fixture();
    let id = run.episodes[0].attempts[0].attempt_id.clone();
    run.episodes[1].attempts[0].attempt_id = id;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "duplicate_attempt_id"
    ));
}

#[test]
fn practice_assistance_must_match_assigned_arm() {
    let mut run = fixture();
    run.episodes[1]
        .attempts
        .iter_mut()
        .find(|a| a.phase == AssessmentPhase::Practice)
        .unwrap()
        .assistance = AssistanceMode::DirectAnswer;
    assert!(has_code(
        validate_assessment_benchmark(&run),
        "assistance_arm_mismatch"
    ));
}

#[test]
fn unknown_study_arm_is_rejected() {
    let mut run = fixture();
    run.episodes[0].arm_id = "unregistered-arm".into();
    assert!(has_code(validate_assessment_benchmark(&run), "unknown_arm"));
}
