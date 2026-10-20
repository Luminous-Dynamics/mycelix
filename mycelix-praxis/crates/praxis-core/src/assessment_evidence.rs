// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Versioned assessment-evidence contract for durable-learning benchmarks.
//! Structural validity does not establish mastery, causality, credential validity,
//! or that an intervention is superior.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashSet};

const DAY_SECONDS: i64 = 86_400;
pub const ASSESSMENT_BENCHMARK_SCHEMA_VERSION: u16 = 1;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssessmentBenchmarkRun {
    pub schema_version: u16,
    pub protocol_id: String,
    pub rubric_version: String,
    pub require_30_day_retention: bool,
    pub arms: Vec<BenchmarkArm>,
    pub episodes: Vec<AssessmentEpisode>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BenchmarkArm {
    pub arm_id: String,
    pub intervention: String,
    pub description: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssessmentEpisode {
    /// Synthetic in fixtures; real identity mappings must be stored separately.
    pub participant_ref: String,
    pub arm_id: String,
    pub task_family_id: String,
    pub attempts: Vec<AssessmentAttempt>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssessmentAttempt {
    pub attempt_id: String,
    pub phase: AssessmentPhase,
    pub task_family_id: String,
    pub task_form_id: String,
    pub task_form_version: String,
    pub assistance: AssistanceMode,
    /// Rubric-normalized integer from 0 through 1000.
    pub score_permille: u16,
    pub completed_at_unix_seconds: i64,
    pub rubric_version: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssessmentPhase {
    Baseline,
    Practice,
    ImmediatePost,
    Transfer,
    Retention14Day,
    Retention30Day,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssistanceMode {
    /// No instructional help; any permitted ordinary task tools must be stated in the protocol.
    Unaided,
    Hint,
    Scaffolded,
    ToolAssisted,
    DirectAnswer,
    /// Preserved explicitly, but ineligible for controlled comparison analysis.
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationIssue {
    pub code: String,
    pub path: String,
    pub message: String,
}

/// Counts only. This is not a learner score or a finding that an intervention worked.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ValidationSummary {
    pub protocol_id: String,
    pub episode_count: usize,
    pub attempt_count: usize,
    pub arm_counts: BTreeMap<String, usize>,
}

/// Fail-closed structural validation before benchmark analysis.
pub fn validate_assessment_benchmark(
    run: &AssessmentBenchmarkRun,
) -> Result<ValidationSummary, Vec<ValidationIssue>> {
    let mut issues = Vec::new();
    if run.schema_version != ASSESSMENT_BENCHMARK_SCHEMA_VERSION {
        issue(&mut issues, "unsupported_schema_version", "$.schema_version",
            format!("expected {}, got {}", ASSESSMENT_BENCHMARK_SCHEMA_VERSION, run.schema_version));
    }
    if run.protocol_id.trim().is_empty() {
        issue(&mut issues, "empty_protocol_id", "$.protocol_id", "protocol_id is required");
    }
    if run.rubric_version.trim().is_empty() {
        issue(&mut issues, "empty_rubric_version", "$.rubric_version", "rubric_version is required");
    }
    if run.arms.len() < 2 {
        issue(&mut issues, "insufficient_arms", "$.arms", "comparative benchmark needs at least two arms");
    }
    if run.episodes.is_empty() {
        issue(&mut issues, "empty_episodes", "$.episodes", "at least one episode is required");
    }

    let mut declared_arms = BTreeSet::new();
    for (i, arm) in run.arms.iter().enumerate() {
        let path = format!("$.arms[{i}]");
        if arm.arm_id.trim().is_empty() {
            issue(&mut issues, "empty_arm_id", format!("{path}.arm_id"), "arm_id is required");
        }
        if !declared_arms.insert(arm.arm_id.as_str()) {
            issue(&mut issues, "duplicate_arm_id", format!("{path}.arm_id"), "arm_id must be unique");
        }
        if arm.intervention.trim().is_empty() {
            issue(&mut issues, "empty_intervention", format!("{path}.intervention"), "intervention is required");
        }
        if arm.description.trim().is_empty() {
            issue(&mut issues, "empty_arm_description", format!("{path}.description"), "description is required");
        }
    }

    let mut participants = HashSet::new();
    let mut attempt_ids = HashSet::new();
    let mut represented_arms = BTreeMap::<String, usize>::new();
    let mut attempt_count = 0usize;

    for (episode_i, episode) in run.episodes.iter().enumerate() {
        let base = format!("$.episodes[{episode_i}]");
        if episode.participant_ref.trim().is_empty() {
            issue(&mut issues, "empty_participant_ref", format!("{base}.participant_ref"), "participant_ref is required");
        } else if !participants.insert(episode.participant_ref.as_str()) {
            issue(&mut issues, "duplicate_participant_episode", format!("{base}.participant_ref"),
                "participant must have one episode per protocol and task family");
        }

        if !declared_arms.contains(episode.arm_id.as_str()) {
            issue(&mut issues, "unknown_arm", format!("{base}.arm_id"), "episode references an undeclared arm");
        } else {
            *represented_arms.entry(episode.arm_id.clone()).or_default() += 1;
        }
        if episode.task_family_id.trim().is_empty() {
            issue(&mut issues, "empty_task_family", format!("{base}.task_family_id"), "task_family_id is required");
        }
        if episode.attempts.is_empty() {
            issue(&mut issues, "empty_attempts", format!("{base}.attempts"), "episode has no attempts");
            continue;
        }

        let mut by_phase = BTreeMap::<AssessmentPhase, Vec<&AssessmentAttempt>>::new();
        let mut practice_forms = BTreeSet::new();
        for (attempt_i, attempt) in episode.attempts.iter().enumerate() {
            attempt_count += 1;
            let path = format!("{base}.attempts[{attempt_i}]");
            if attempt.attempt_id.trim().is_empty() {
                issue(&mut issues, "empty_attempt_id", format!("{path}.attempt_id"), "attempt_id is required");
            } else if !attempt_ids.insert(attempt.attempt_id.as_str()) {
                issue(&mut issues, "duplicate_attempt_id", format!("{path}.attempt_id"), "attempt_id must be globally unique");
            }
            if attempt.task_family_id != episode.task_family_id {
                issue(&mut issues, "task_family_mismatch", format!("{path}.task_family_id"),
                    "attempt task family must match its episode");
            }
            if attempt.task_form_id.trim().is_empty() {
                issue(&mut issues, "empty_task_form_id", format!("{path}.task_form_id"), "task_form_id is required");
            }
            if attempt.task_form_version.trim().is_empty() {
                issue(&mut issues, "empty_task_form_version", format!("{path}.task_form_version"), "task_form_version is required");
            }
            if attempt.score_permille > 1000 {
                issue(&mut issues, "score_out_of_range", format!("{path}.score_permille"), "score must be in [0, 1000]");
            }
            if attempt.completed_at_unix_seconds < 0 {
                issue(&mut issues, "invalid_timestamp", format!("{path}.completed_at_unix_seconds"),
                    "timestamp must be a non-negative Unix timestamp");
            }
            if attempt.rubric_version != run.rubric_version {
                issue(&mut issues, "rubric_version_mismatch", format!("{path}.rubric_version"),
                    "attempt rubric version must match the run");
            }
            if attempt.assistance == AssistanceMode::Unknown {
                issue(&mut issues, "unknown_assistance", format!("{path}.assistance"),
                    "unknown assistance is ineligible for controlled benchmark analysis");
            }
            if requires_unaided(attempt.phase) && attempt.assistance != AssistanceMode::Unaided {
                issue(&mut issues, "assisted_independent_phase", format!("{path}.assistance"),
                    "independent assessment phase must be unaided");
            }
            if attempt.phase == AssessmentPhase::Practice {
                practice_forms.insert(attempt.task_form_id.as_str());
            }
            by_phase.entry(attempt.phase).or_default().push(attempt);
        }

        for (phase, attempts) in &by_phase {
            if *phase != AssessmentPhase::Practice && attempts.len() > 1 {
                issue(&mut issues, "duplicate_anchor_phase", format!("{base}.attempts"),
                    format!("phase {:?} may occur at most once", phase));
            }
        }

        let baseline = unique_phase(&by_phase, AssessmentPhase::Baseline);
        let practice = by_phase.get(&AssessmentPhase::Practice).cloned().unwrap_or_default();
        let immediate = unique_phase(&by_phase, AssessmentPhase::ImmediatePost);
        let transfer = unique_phase(&by_phase, AssessmentPhase::Transfer);
        let retention14 = unique_phase(&by_phase, AssessmentPhase::Retention14Day);
        let retention30 = unique_phase(&by_phase, AssessmentPhase::Retention30Day);

        if baseline.is_none() {
            issue(&mut issues, "missing_baseline", format!("{base}.attempts"), "baseline is required");
        }
        if practice.is_empty() {
            issue(&mut issues, "missing_practice", format!("{base}.attempts"), "at least one practice attempt is required");
        }
        if transfer.is_none() {
            issue(&mut issues, "missing_transfer", format!("{base}.attempts"), "independent transfer is required");
        }
        if retention14.is_none() {
            issue(&mut issues, "missing_retention_14d", format!("{base}.attempts"), "14-day retention is required");
        }
        if run.require_30_day_retention && retention30.is_none() {
            issue(&mut issues, "missing_retention_30d", format!("{base}.attempts"), "30-day retention is required by protocol");
        }

        // Anchor forms must be distinct from one another and from every practice form.
        let anchors: Vec<(&str, &AssessmentAttempt)> = [
            baseline.map(|a| ("baseline", a)),
            immediate.map(|a| ("immediate_post", a)),
            transfer.map(|a| ("transfer", a)),
            retention14.map(|a| ("retention_14_day", a)),
            retention30.map(|a| ("retention_30_day", a)),
        ].into_iter().flatten().collect();
        let mut anchor_forms = BTreeMap::<&str, &str>::new();
        for (phase_name, attempt) in &anchors {
            if let Some(prior) = anchor_forms.insert(attempt.task_form_id.as_str(), *phase_name) {
                issue(&mut issues, "reused_anchor_form", format!("{base}.attempts"),
                    format!("form {:?} reused by {} and {}", attempt.task_form_id, prior, phase_name));
            }
            if practice_forms.contains(attempt.task_form_id.as_str()) {
                issue(&mut issues, "practice_form_reused_as_anchor", format!("{base}.attempts"),
                    format!("form {:?} is used for both practice and assessment", attempt.task_form_id));
            }
        }

        let first_practice = practice.iter().min_by_key(|a| a.completed_at_unix_seconds).copied();
        let last_practice = practice.iter().max_by_key(|a| a.completed_at_unix_seconds).copied();
        if let (Some(baseline), Some(first)) = (baseline, first_practice) {
            if baseline.completed_at_unix_seconds >= first.completed_at_unix_seconds {
                issue(&mut issues, "baseline_order", format!("{base}.attempts"), "baseline must precede practice");
            }
        }
        if let (Some(last), Some(post)) = (last_practice, immediate) {
            if last.completed_at_unix_seconds >= post.completed_at_unix_seconds {
                issue(&mut issues, "immediate_post_order", format!("{base}.attempts"), "immediate post-test must follow practice");
            }
        }
        if let (Some(last), Some(transfer)) = (last_practice, transfer) {
            let predecessor = immediate.unwrap_or(last);
            if predecessor.completed_at_unix_seconds >= transfer.completed_at_unix_seconds {
                issue(&mut issues, "transfer_order", format!("{base}.attempts"), "transfer must follow immediate post-test or practice");
            }
        }
        if let (Some(transfer), Some(retention)) = (transfer, retention14) {
            if transfer.completed_at_unix_seconds >= retention.completed_at_unix_seconds {
                issue(&mut issues, "retention_order", format!("{base}.attempts"), "14-day retention must follow transfer");
            }
        }
        if let (Some(retention14), Some(retention30)) = (retention14, retention30) {
            if retention14.completed_at_unix_seconds >= retention30.completed_at_unix_seconds {
                issue(&mut issues, "retention_order", format!("{base}.attempts"), "30-day retention must follow 14-day retention");
            }
        }
        if let (Some(last), Some(retention)) = (last_practice, retention14) {
            if retention.completed_at_unix_seconds - last.completed_at_unix_seconds < 14 * DAY_SECONDS {
                issue(&mut issues, "retention_too_early", format!("{base}.attempts"),
                    "14-day retention must be at least 14 days after final practice");
            }
        }
        if let (Some(last), Some(retention)) = (last_practice, retention30) {
            if retention.completed_at_unix_seconds - last.completed_at_unix_seconds < 30 * DAY_SECONDS {
                issue(&mut issues, "retention_too_early", format!("{base}.attempts"),
                    "30-day retention must be at least 30 days after final practice");
            }
        }
    }

    for arm in &declared_arms {
        if !represented_arms.contains_key(*arm) {
            issue(&mut issues, "empty_arm", "$.arms", format!("declared arm {:?} has no episodes", arm));
        }
    }

    if issues.is_empty() {
        Ok(ValidationSummary {
            protocol_id: run.protocol_id.clone(),
            episode_count: run.episodes.len(),
            attempt_count,
            arm_counts: represented_arms,
        })
    } else {
        Err(issues)
    }
}

fn requires_unaided(phase: AssessmentPhase) -> bool {
    matches!(phase,
        AssessmentPhase::Baseline
        | AssessmentPhase::ImmediatePost
        | AssessmentPhase::Transfer
        | AssessmentPhase::Retention14Day
        | AssessmentPhase::Retention30Day)
}

fn unique_phase<'a>(
    phases: &'a BTreeMap<AssessmentPhase, Vec<&'a AssessmentAttempt>>,
    phase: AssessmentPhase,
) -> Option<&'a AssessmentAttempt> {
    phases.get(&phase).and_then(|attempts| {
        if attempts.len() == 1 { attempts.first().copied() } else { None }
    })
}

fn issue(
    issues: &mut Vec<ValidationIssue>,
    code: &str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(ValidationIssue { code: code.to_string(), path: path.into(), message: message.into() });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> AssessmentBenchmarkRun {
        serde_json::from_str(include_str!(
            "../../../examples/assessment-benchmarks/v1/ai_scaffold_fading_synthetic.json"
        )).expect("synthetic benchmark fixture must parse")
    }

    fn has_code(result: Result<ValidationSummary, Vec<ValidationIssue>>, code: &str) -> bool {
        result.err().unwrap_or_default().iter().any(|issue| issue.code == code)
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
        run.episodes[1].attempts.iter_mut()
            .find(|a| a.phase == AssessmentPhase::Transfer).unwrap().assistance = AssistanceMode::DirectAnswer;
        assert!(has_code(validate_assessment_benchmark(&run), "assisted_independent_phase"));
    }

    #[test]
    fn practice_form_cannot_be_reused_for_transfer() {
        let mut run = fixture();
        let form = run.episodes[0].attempts.iter()
            .find(|a| a.phase == AssessmentPhase::Practice).unwrap().task_form_id.clone();
        run.episodes[0].attempts.iter_mut()
            .find(|a| a.phase == AssessmentPhase::Transfer).unwrap().task_form_id = form;
        assert!(has_code(validate_assessment_benchmark(&run), "practice_form_reused_as_anchor"));
    }

    #[test]
    fn high_score_cannot_make_early_retention_valid() {
        let mut run = fixture();
        let last = run.episodes[0].attempts.iter()
            .filter(|a| a.phase == AssessmentPhase::Practice)
            .map(|a| a.completed_at_unix_seconds).max().unwrap();
        let retention = run.episodes[0].attempts.iter_mut()
            .find(|a| a.phase == AssessmentPhase::Retention14Day).unwrap();
        retention.completed_at_unix_seconds = last + 13 * DAY_SECONDS;
        retention.score_permille = 1000;
        assert!(has_code(validate_assessment_benchmark(&run), "retention_too_early"));
    }

    #[test]
    fn unknown_assistance_is_not_silently_treated_as_unaided() {
        let mut run = fixture();
        run.episodes[0].attempts.iter_mut()
            .find(|a| a.phase == AssessmentPhase::Practice).unwrap().assistance = AssistanceMode::Unknown;
        assert!(has_code(validate_assessment_benchmark(&run), "unknown_assistance"));
    }

    #[test]
    fn rubric_mismatch_is_rejected() {
        let mut run = fixture();
        run.episodes[2].attempts[0].rubric_version = "rubric-v0".into();
        assert!(has_code(validate_assessment_benchmark(&run), "rubric_version_mismatch"));
    }

    #[test]
    fn scores_above_one_thousand_are_rejected() {
        let mut run = fixture();
        run.episodes[1].attempts[0].score_permille = 1001;
        assert!(has_code(validate_assessment_benchmark(&run), "score_out_of_range"));
    }

    #[test]
    fn duplicate_attempt_ids_are_rejected() {
        let mut run = fixture();
        let id = run.episodes[0].attempts[0].attempt_id.clone();
        run.episodes[1].attempts[0].attempt_id = id;
        assert!(has_code(validate_assessment_benchmark(&run), "duplicate_attempt_id"));
    }

    #[test]
    fn unknown_study_arm_is_rejected() {
        let mut run = fixture();
        run.episodes[0].arm_id = "unregistered-arm".into();
        assert!(has_code(validate_assessment_benchmark(&run), "unknown_arm"));
    }
}
