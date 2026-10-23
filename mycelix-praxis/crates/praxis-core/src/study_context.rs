// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Adjacent study-context contract for learning-support availability, uptake,
//! ordinary-coursework participation, and explicit missingness.
//!
//! This does not change assessment-evidence v1, calculate intervention effects,
//! or determine mastery. Counts and availability are context measures, not outcomes.

use crate::assessment_evidence::{validate_assessment_benchmark, AssessmentBenchmarkRun};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub const STUDY_CONTEXT_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StudyContextRun {
    pub schema_version: u16,
    pub protocol_id: String,
    /// SHA-256 of the exact assessment artifact bytes, not a digest of parsed fields.
    pub assessment_run_sha256: String,
    pub observations: Vec<StudyContextObservation>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StudyContextObservation {
    /// Must match an opaque participant_ref in the bound assessment benchmark.
    pub participant_ref: String,
    /// The assigned arm, not a claim about which support the participant used.
    pub assigned_arm_id: String,
    pub period: StudyPeriod,
    pub support_availability: SupportAvailability,
    pub observation_status: ObservationStatus,
    /// Missing values are null; zero means the metric was observed and its value was zero.
    #[serde(deserialize_with = "deserialize_required_option")]
    pub support_invocations: Option<u32>,
    #[serde(deserialize_with = "deserialize_required_option")]
    pub support_seconds: Option<u32>,
    #[serde(deserialize_with = "deserialize_required_option")]
    pub scheduled_learning_tasks: Option<u32>,
    #[serde(deserialize_with = "deserialize_required_option")]
    pub attempted_learning_tasks: Option<u32>,
    #[serde(deserialize_with = "deserialize_required_option")]
    pub completed_learning_tasks: Option<u32>,
    #[serde(deserialize_with = "deserialize_required_option")]
    pub independent_practice_attempts: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StudyPeriod {
    Baseline,
    Intervention,
    #[serde(rename = "follow_up_7_day")]
    FollowUp7Day,
    #[serde(rename = "follow_up_30_day")]
    FollowUp30Day,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SupportAvailability {
    NotOffered,
    Available,
    Unavailable,
    /// Availability itself was not observed. Never infer zero use from this state.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ObservationStatus {
    /// All defined context measures were observed, including explicit zero values.
    Complete,
    /// Some measures were observed and at least one measure or availability is missing.
    Partial,
    /// No measures were collected for this participant-period.
    NotCollected,
    /// The scheduled follow-up was not observed; missing is not treated as zero.
    LostToFollowUp,
    /// The participant withdrew for this period; missing is not treated as zero.
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StudyContextIssue {
    pub code: String,
    pub path: String,
    pub message: String,
}

/// Summary counts only. It is not a learner score or an intervention-effect estimate.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct StudyContextSummary {
    pub protocol_id: String,
    pub observation_count: usize,
    pub participant_count: usize,
    pub missing_or_partial_count: usize,
    pub period_counts: BTreeMap<String, usize>,
    pub status_counts: BTreeMap<String, usize>,
}

/// Fail-closed structural checks for the context companion artifact.
/// This does not validate the bound assessment artifact or verify its digest against bytes.
pub fn validate_study_context(
    run: &StudyContextRun,
) -> Result<StudyContextSummary, Vec<StudyContextIssue>> {
    let mut issues = Vec::new();

    if run.schema_version != STUDY_CONTEXT_SCHEMA_VERSION {
        push_issue(
            &mut issues,
            "unsupported_schema_version",
            "$.schema_version",
            format!(
                "expected {}, got {}",
                STUDY_CONTEXT_SCHEMA_VERSION, run.schema_version
            ),
        );
    }
    if run.protocol_id.trim().is_empty() {
        push_issue(
            &mut issues,
            "empty_protocol_id",
            "$.protocol_id",
            "protocol_id is required",
        );
    }
    if !is_sha256_hex(&run.assessment_run_sha256) {
        push_issue(
            &mut issues,
            "invalid_assessment_digest",
            "$.assessment_run_sha256",
            "assessment_run_sha256 must be 64 lowercase hexadecimal characters",
        );
    }
    if run.observations.is_empty() {
        push_issue(
            &mut issues,
            "empty_observations",
            "$.observations",
            "at least one participant-period observation is required",
        );
    }

    let mut participant_arms = BTreeMap::<String, String>::new();
    let mut participant_periods = BTreeSet::<(String, StudyPeriod)>::new();
    let mut arm_ids = BTreeSet::<String>::new();
    let mut period_counts = BTreeMap::<String, usize>::new();
    let mut status_counts = BTreeMap::<String, usize>::new();
    let mut participant_refs = BTreeSet::<String>::new();
    let mut missing_or_partial_count = 0usize;

    for (index, observation) in run.observations.iter().enumerate() {
        let path = format!("$.observations[{index}]");

        if observation.participant_ref.trim().is_empty() {
            push_issue(
                &mut issues,
                "empty_participant_ref",
                format!("{path}.participant_ref"),
                "participant_ref is required",
            );
        } else {
            participant_refs.insert(observation.participant_ref.clone());
        }
        if observation.assigned_arm_id.trim().is_empty() {
            push_issue(
                &mut issues,
                "empty_assigned_arm",
                format!("{path}.assigned_arm_id"),
                "assigned_arm_id is required",
            );
        } else {
            arm_ids.insert(observation.assigned_arm_id.clone());
            if let Some(previous_arm) = participant_arms
                .insert(observation.participant_ref.clone(), observation.assigned_arm_id.clone())
            {
                if previous_arm != observation.assigned_arm_id {
                    push_issue(
                        &mut issues,
                        "participant_arm_drift",
                        format!("{path}.assigned_arm_id"),
                        "assigned arm must remain constant for a participant across periods",
                    );
                }
            }
        }

        if !participant_periods.insert((
            observation.participant_ref.clone(),
            observation.period,
        )) {
            push_issue(
                &mut issues,
                "duplicate_participant_period",
                format!("{path}.period"),
                "a participant may have at most one context observation per period",
            );
        }

        *period_counts.entry(period_name(observation.period).to_string()).or_default() += 1;
        *status_counts
            .entry(status_name(observation.observation_status).to_string())
            .or_default() += 1;

        validate_measurement_status(observation, &path, &mut issues);
        validate_count_relations(observation, &path, &mut issues);

        if observation.observation_status != ObservationStatus::Complete {
            missing_or_partial_count += 1;
        }
    }

    if arm_ids.len() < 2 {
        push_issue(
            &mut issues,
            "insufficient_observed_arms",
            "$.observations",
            "a comparative context artifact must represent at least two assigned arms",
        );
    }

    if issues.is_empty() {
        Ok(StudyContextSummary {
            protocol_id: run.protocol_id.clone(),
            observation_count: run.observations.len(),
            participant_count: participant_refs.len(),
            missing_or_partial_count,
            period_counts,
            status_counts,
        })
    } else {
        Err(issues)
    }
}

/// Validate the context artifact against a previously validated assessment run and a digest
/// computed from the exact assessment artifact bytes. This function never computes effects.
pub fn validate_study_context_binding(
    context: &StudyContextRun,
    assessment: &AssessmentBenchmarkRun,
    observed_assessment_sha256: &str,
) -> Result<StudyContextSummary, Vec<StudyContextIssue>> {
    let mut issues = Vec::new();
    let summary = match validate_study_context(context) {
        Ok(summary) => Some(summary),
        Err(context_issues) => {
            issues.extend(context_issues);
            None
        }
    };

    if let Err(assessment_issues) = validate_assessment_benchmark(assessment) {
        for assessment_issue in assessment_issues {
            issues.push(StudyContextIssue {
                code: format!("assessment_{}", assessment_issue.code),
                path: format!("assessment{}", assessment_issue.path.trim_start_matches('$')),
                message: assessment_issue.message,
            });
        }
    }

    if context.protocol_id != assessment.protocol_id {
        push_issue(
            &mut issues,
            "protocol_mismatch",
            "$.protocol_id",
            "context protocol_id must match the assessment benchmark protocol_id",
        );
    }
    if !is_sha256_hex(observed_assessment_sha256) {
        push_issue(
            &mut issues,
            "invalid_observed_assessment_digest",
            "$.assessment_run_sha256",
            "the supplied observed digest must be 64 lowercase hexadecimal characters",
        );
    } else if context.assessment_run_sha256 != observed_assessment_sha256 {
        push_issue(
            &mut issues,
            "assessment_digest_mismatch",
            "$.assessment_run_sha256",
            "context artifact is not bound to the supplied exact assessment artifact digest",
        );
    }

    let declared_arms = assessment
        .arms
        .iter()
        .map(|arm| arm.arm_id.as_str())
        .collect::<BTreeSet<_>>();
    let participant_arms = assessment
        .episodes
        .iter()
        .map(|episode| (episode.participant_ref.as_str(), episode.arm_id.as_str()))
        .collect::<BTreeMap<_, _>>();

    for (index, observation) in context.observations.iter().enumerate() {
        let path = format!("$.observations[{index}]");
        if !declared_arms.contains(observation.assigned_arm_id.as_str()) {
            push_issue(
                &mut issues,
                "unknown_assessment_arm",
                format!("{path}.assigned_arm_id"),
                "assigned arm is not declared by the assessment benchmark",
            );
        }
        match participant_arms.get(observation.participant_ref.as_str()) {
            None => push_issue(
                &mut issues,
                "unknown_assessment_participant",
                format!("{path}.participant_ref"),
                "participant_ref is absent from the bound assessment benchmark",
            ),
            Some(assessment_arm) if *assessment_arm != observation.assigned_arm_id => push_issue(
                &mut issues,
                "assessment_arm_mismatch",
                format!("{path}.assigned_arm_id"),
                "assigned arm differs from the participant's assessment episode arm",
            ),
            Some(_) => {}
        }
    }

    if issues.is_empty() {
        Ok(summary.expect("valid context must have a summary"))
    } else {
        Err(issues)
    }
}

fn deserialize_required_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

fn validate_measurement_status(
    observation: &StudyContextObservation,
    path: &str,
    issues: &mut Vec<StudyContextIssue>,
) {
    let values = [
        observation.support_invocations,
        observation.support_seconds,
        observation.scheduled_learning_tasks,
        observation.attempted_learning_tasks,
        observation.completed_learning_tasks,
        observation.independent_practice_attempts,
    ];
    let present_count = values.iter().filter(|value| value.is_some()).count();
    let missing_count = values.len() - present_count;

    match observation.observation_status {
        ObservationStatus::Complete => {
            if missing_count > 0 || observation.support_availability == SupportAvailability::Unknown {
                push_issue(
                    issues,
                    "incomplete_complete_observation",
                    path,
                    "complete observations require all counters and known support availability; null is not zero",
                );
            }
        }
        ObservationStatus::Partial => {
            if present_count == 0
                || (missing_count == 0
                    && observation.support_availability != SupportAvailability::Unknown)
            {
                push_issue(
                    issues,
                    "invalid_partial_observation",
                    path,
                    "partial observations require at least one observed value and at least one missing value or unknown availability",
                );
            }
        }
        ObservationStatus::NotCollected
        | ObservationStatus::LostToFollowUp
        | ObservationStatus::Withdrawn => {
            if present_count > 0 || observation.support_availability != SupportAvailability::Unknown {
                push_issue(
                    issues,
                    "unobserved_metrics_present",
                    path,
                    "not-collected, lost, or withdrawn periods must use null counters and unknown availability",
                );
            }
        }
    }
}

fn validate_count_relations(
    observation: &StudyContextObservation,
    path: &str,
    issues: &mut Vec<StudyContextIssue>,
) {
    if let (Some(scheduled), Some(attempted)) = (
        observation.scheduled_learning_tasks,
        observation.attempted_learning_tasks,
    ) {
        if attempted > scheduled {
            push_issue(
                issues,
                "attempts_exceed_scheduled",
                format!("{path}.attempted_learning_tasks"),
                "attempted tasks cannot exceed scheduled task opportunities",
            );
        }
    }
    if let (Some(attempted), Some(completed)) = (
        observation.attempted_learning_tasks,
        observation.completed_learning_tasks,
    ) {
        if completed > attempted {
            push_issue(
                issues,
                "completions_exceed_attempts",
                format!("{path}.completed_learning_tasks"),
                "completed tasks cannot exceed attempted tasks",
            );
        }
    }

    let blocked = matches!(
        observation.support_availability,
        SupportAvailability::NotOffered | SupportAvailability::Unavailable
    );
    if blocked
        && (observation.support_invocations.is_some_and(|count| count > 0)
            || observation.support_seconds.is_some_and(|seconds| seconds > 0))
    {
        push_issue(
            issues,
            "use_when_support_not_available",
            path,
            "positive support use is inconsistent with support not being offered or being unavailable",
        );
    }
    if observation.support_invocations == Some(0)
        && observation.support_seconds.is_some_and(|seconds| seconds > 0)
    {
        push_issue(
            issues,
            "support_duration_without_invocation",
            format!("{path}.support_seconds"),
            "positive support duration requires at least one recorded invocation",
        );
    }
    if observation.support_invocations.is_some_and(|count| count > 0)
        && observation.support_availability != SupportAvailability::Available
    {
        push_issue(
            issues,
            "invocation_without_confirmed_availability",
            format!("{path}.support_availability"),
            "positive support use requires availability to be recorded as available",
        );
    }
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn period_name(period: StudyPeriod) -> &'static str {
    match period {
        StudyPeriod::Baseline => "baseline",
        StudyPeriod::Intervention => "intervention",
        StudyPeriod::FollowUp7Day => "follow_up_7_day",
        StudyPeriod::FollowUp30Day => "follow_up_30_day",
    }
}

fn status_name(status: ObservationStatus) -> &'static str {
    match status {
        ObservationStatus::Complete => "complete",
        ObservationStatus::Partial => "partial",
        ObservationStatus::NotCollected => "not_collected",
        ObservationStatus::LostToFollowUp => "lost_to_follow_up",
        ObservationStatus::Withdrawn => "withdrawn",
    }
}

fn push_issue(
    issues: &mut Vec<StudyContextIssue>,
    code: &str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(StudyContextIssue {
        code: code.to_string(),
        path: path.into(),
        message: message.into(),
    });
}
