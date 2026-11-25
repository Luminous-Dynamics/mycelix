// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! # Advisory Job Matching Engine
//!
//! Runs locally over the subset of opportunities retrieved by a client. This
//! module ranks explicit profile signals; it does not verify credentials, make
//! hiring decisions, or establish professional eligibility.
//!
//! Candidate skills and education passed here may be self-declared. Callers must
//! preserve that provenance in the UI and must not present this score as a
//! probability of hiring success or proof of competence.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

/// A job posting as returned by the zome (simplified view for matching).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JobView {
    pub title: String,
    pub organization: String,
    pub required_skills: Vec<String>,
    pub preferred_skills: Vec<String>,
    pub education_level: Option<String>,
    pub remote_ok: bool,
    /// Legacy profile signal threshold (0-1000). Advisory only; never a hard gate.
    pub vitality_minimum: Option<u16>,
}

/// How the candidate's declared education relates to the posting.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EducationRequirementStatus {
    /// The posting has no education requirement.
    NotRequired,
    /// The posting has a requirement, but the candidate supplied no declaration.
    NotProvided,
    /// The declared label exactly matches the posting's label.
    DeclaredMatch,
    /// The candidate supplied a different label; equivalency is not inferred.
    DeclaredMismatch,
}

/// Compatibility-only status for the legacy aggregate profile signal.
///
/// This signal is deliberately excluded from ranking and qualification decisions.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LegacyProfileSignalStatus {
    /// No legacy threshold was specified by the posting.
    NotRequired,
    /// A threshold exists, but no aggregate signal was supplied.
    NotEvaluated,
    /// The supplied aggregate is below the posting's legacy threshold.
    BelowLegacyThreshold,
    /// The supplied aggregate is at or above the posting's legacy threshold.
    MeetsLegacyThreshold,
}

/// Advisory ranking result. The score is a heuristic, not a probability.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JobMatch {
    pub job: JobView,
    /// Heuristic over declared skill and education strings, bounded to 0..=1.
    pub score: f32,
    /// Normalized, unique required-skill labels shared with the profile.
    pub matched_required: Vec<String>,
    /// Normalized, unique preferred-skill labels shared with the profile.
    pub matched_preferred: Vec<String>,
    pub education_status: EducationRequirementStatus,
    /// Informational only; this field never removes the job from results.
    pub legacy_profile_signal_status: LegacyProfileSignalStatus,
}

/// Normalize a free-text skill/education label for exact, case-insensitive
/// comparison. This is not a substitute for a shared skill taxonomy.
fn normalize_label(value: &str) -> String {
    value.trim().to_lowercase()
}

fn unique_normalized(values: &[String]) -> Vec<String> {
    let mut seen = HashSet::new();
    values
        .iter()
        .map(|value| normalize_label(value))
        .filter(|value| !value.is_empty() && seen.insert(value.clone()))
        .collect()
}

/// Compute an advisory heuristic over the candidate's declared profile.
///
/// Components:
/// - Required skill coverage: up to 0.6.
/// - Preferred skill overlap: up to 0.2.
/// - Exact declared education-label match: 0.2.
///
/// Missing requirements award no points. A PhD label is not treated as an
/// automatic substitute for an unrelated or differently named requirement.
pub fn score_job(
    job: &JobView,
    agent_skills: &HashSet<String>,
    agent_education: Option<&str>,
) -> JobMatch {
    let normalized_agent_skills: HashSet<String> = agent_skills
        .iter()
        .map(|skill| normalize_label(skill))
        .filter(|skill| !skill.is_empty())
        .collect();

    let required_skills = unique_normalized(&job.required_skills);
    let preferred_skills = unique_normalized(&job.preferred_skills);

    let matched_required: Vec<String> = required_skills
        .iter()
        .filter(|skill| normalized_agent_skills.contains(*skill))
        .cloned()
        .collect();

    // No requirements means no evidence-based skill score, not a moderate default.
    let required_score = if required_skills.is_empty() {
        0.0
    } else {
        (matched_required.len() as f32 / required_skills.len() as f32) * 0.6
    };

    let matched_preferred: Vec<String> = preferred_skills
        .iter()
        .filter(|skill| normalized_agent_skills.contains(*skill))
        .cloned()
        .collect();
    let preferred_score = (matched_preferred.len() as f32 * 0.02).min(0.2);

    let required_education = job
        .education_level
        .as_deref()
        .map(str::trim)
        .filter(|value| !value.is_empty());

    let declared_education = agent_education
        .map(str::trim)
        .filter(|value| !value.is_empty());

    let education_status = match (required_education, declared_education) {
        (None, _) => EducationRequirementStatus::NotRequired,
        (Some(_), None) => EducationRequirementStatus::NotProvided,
        (Some(required), Some(declared))
            if normalize_label(required) == normalize_label(declared) =>
        {
            EducationRequirementStatus::DeclaredMatch
        }
        (Some(_), Some(_)) => EducationRequirementStatus::DeclaredMismatch,
    };

    let education_score = if education_status == EducationRequirementStatus::DeclaredMatch {
        0.2
    } else {
        0.0
    };

    let total = required_score + preferred_score + education_score;
    let legacy_profile_signal_status = if job.vitality_minimum.is_some() {
        LegacyProfileSignalStatus::NotEvaluated
    } else {
        LegacyProfileSignalStatus::NotRequired
    };

    JobMatch {
        job: job.clone(),
        score: total.clamp(0.0, 1.0),
        matched_required,
        matched_preferred,
        education_status,
        legacy_profile_signal_status,
    }
}

/// Rank every supplied opportunity by advisory profile-signal heuristic.
///
/// The legacy profile signal is accepted only to populate a compatibility
/// annotation. Missing or below-threshold signals do not exclude an opportunity.
/// There is intentionally no score-threshold parameter: zero/unknown evidence
/// may lower a result's rank, but it cannot silently remove that opportunity.
pub fn rank_jobs(
    jobs: &[JobView],
    agent_skills: &HashSet<String>,
    agent_education: Option<&str>,
    agent_avg_vitality: Option<u16>,
    limit: usize,
) -> Vec<JobMatch> {
    let mut matches: Vec<JobMatch> = jobs
        .iter()
        .map(|job| {
            let mut matched = score_job(job, agent_skills, agent_education);
            matched.legacy_profile_signal_status =
                match (job.vitality_minimum, agent_avg_vitality) {
                    (None, _) => LegacyProfileSignalStatus::NotRequired,
                    (Some(_), None) => LegacyProfileSignalStatus::NotEvaluated,
                    (Some(minimum), Some(value)) if value >= minimum => {
                        LegacyProfileSignalStatus::MeetsLegacyThreshold
                    }
                    (Some(_), Some(_)) => LegacyProfileSignalStatus::BelowLegacyThreshold,
                };
            matched
        })
        .collect();

    matches.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    matches.truncate(limit);
    matches
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rust_dev_job() -> JobView {
        JobView {
            title: "Rust Developer".into(),
            organization: "Luminous Dynamics".into(),
            required_skills: vec!["rust".into(), "holochain".into(), "wasm".into()],
            preferred_skills: vec!["leptos".into(), "nix".into()],
            education_level: Some("Bachelor's".into()),
            remote_ok: true,
            vitality_minimum: None,
        }
    }

    #[test]
    fn perfect_declared_profile_scores_without_claiming_verification() {
        let skills: HashSet<String> = ["rust", "holochain", "wasm", "leptos", "nix"]
            .iter()
            .map(|skill| skill.to_string())
            .collect();
        let matched = score_job(&rust_dev_job(), &skills, Some("Bachelor's"));
        assert!((matched.score - 0.84).abs() < 0.00001);
        assert_eq!(matched.matched_required.len(), 3);
        assert_eq!(
            matched.education_status,
            EducationRequirementStatus::DeclaredMatch
        );
    }

    #[test]
    fn no_match_has_no_declared_education_credit() {
        let skills: HashSet<String> = ["python", "django"]
            .iter()
            .map(|skill| skill.to_string())
            .collect();
        let matched = score_job(&rust_dev_job(), &skills, None);
        assert!(matched.score < 0.1);
        assert!(matched.matched_required.is_empty());
        assert_eq!(
            matched.education_status,
            EducationRequirementStatus::NotProvided
        );
    }

    #[test]
    fn partial_skill_match_is_bounded_and_explained() {
        let skills: HashSet<String> = ["rust", "wasm"]
            .iter()
            .map(|skill| skill.to_string())
            .collect();
        let matched = score_job(&rust_dev_job(), &skills, Some("Bachelor's"));
        assert!((matched.score - 0.6).abs() < f32::EPSILON);
        assert_eq!(matched.matched_required, vec!["rust", "wasm"]);
    }

    #[test]
    fn no_education_requirement_awards_no_education_bonus() {
        let mut job = rust_dev_job();
        job.required_skills.clear();
        job.preferred_skills.clear();
        job.education_level = None;
        let skills = HashSet::new();
        let matched = score_job(&job, &skills, Some("PhD"));
        assert_eq!(matched.score, 0.0);
        assert_eq!(
            matched.education_status,
            EducationRequirementStatus::NotRequired
        );
    }

    #[test]
    fn phd_is_not_an_automatic_substitute_for_another_label() {
        let job = rust_dev_job();
        let matched = score_job(&job, &HashSet::new(), Some("PhD"));
        assert_eq!(
            matched.education_status,
            EducationRequirementStatus::DeclaredMismatch
        );
        assert_eq!(matched.score, 0.0);
    }

    #[test]
    fn empty_education_declaration_is_treated_as_not_provided() {
        let matched = score_job(&rust_dev_job(), &HashSet::new(), Some("  "));
        assert_eq!(
            matched.education_status,
            EducationRequirementStatus::NotProvided
        );
        assert_eq!(matched.score, 0.0);
    }

    #[test]
    fn no_required_skills_does_not_create_a_default_match_score() {
        let mut job = rust_dev_job();
        job.required_skills.clear();
        job.preferred_skills.clear();
        job.education_level = None;
        let matched = score_job(&job, &HashSet::new(), None);
        assert_eq!(matched.score, 0.0);
    }

    #[test]
    fn skill_labels_are_case_normalized_and_deduplicated() {
        let mut job = rust_dev_job();
        job.required_skills = vec!["Rust".into(), " rust ".into(), "Holochain".into()];
        let skills: HashSet<String> = [" RUST ", "holochain"]
            .iter()
            .map(|skill| skill.to_string())
            .collect();
        let matched = score_job(&job, &skills, None);
        assert_eq!(matched.matched_required, vec!["rust", "holochain"]);
        assert!((matched.score - 0.6).abs() < f32::EPSILON);
    }

    #[test]
    fn ranked_jobs_preserve_unknown_legacy_signal_as_unknown() {
        let mut job = rust_dev_job();
        job.vitality_minimum = Some(700);
        let ranked = rank_jobs(&[job], &HashSet::new(), None, None, 10);
        assert_eq!(ranked.len(), 1);
        assert_eq!(
            ranked[0].legacy_profile_signal_status,
            LegacyProfileSignalStatus::NotEvaluated
        );
    }

    #[test]
    fn below_threshold_legacy_signal_does_not_exclude_job() {
        let mut job = rust_dev_job();
        job.vitality_minimum = Some(700);
        let ranked = rank_jobs(&[job], &HashSet::new(), None, Some(500), 10);
        assert_eq!(ranked.len(), 1);
        assert_eq!(
            ranked[0].legacy_profile_signal_status,
            LegacyProfileSignalStatus::BelowLegacyThreshold
        );
    }

    #[test]
    fn sufficient_legacy_signal_is_only_an_annotation() {
        let mut job = rust_dev_job();
        job.vitality_minimum = Some(700);
        let ranked = rank_jobs(&[job], &HashSet::new(), None, Some(800), 10);
        assert_eq!(ranked.len(), 1);
        assert_eq!(
            ranked[0].legacy_profile_signal_status,
            LegacyProfileSignalStatus::MeetsLegacyThreshold
        );
    }

    #[test]
    fn education_requirement_and_profile_signals_do_not_overwrite_each_other() {
        let mut job = rust_dev_job();
        job.vitality_minimum = Some(700);
        let matched = rank_jobs(&[job], &HashSet::new(), Some("Bachelor's"), None, 10);
        assert_eq!(matched.len(), 1);
        assert_eq!(
            matched[0].education_status,
            EducationRequirementStatus::DeclaredMatch
        );
        assert_eq!(
            matched[0].legacy_profile_signal_status,
            LegacyProfileSignalStatus::NotEvaluated
        );
    }

    #[test]
    fn zero_evidence_does_not_remove_an_opportunity() {
        let job = rust_dev_job();
        let ranked = rank_jobs(&[job], &HashSet::new(), None, None, 10);
        assert_eq!(ranked.len(), 1);
        assert_eq!(ranked[0].score, 0.0);
    }

    #[test]
    fn limit_is_applied_after_ranking_all_opportunities() {
        let jobs = vec![rust_dev_job(), {
            let mut job = rust_dev_job();
            job.title = "Different job".into();
            job.required_skills = vec!["python".into()];
            job
        }];
        let skills: HashSet<String> = ["rust", "holochain", "wasm"]
            .iter()
            .map(|skill| skill.to_string())
            .collect();
        let ranked = rank_jobs(&jobs, &skills, Some("Bachelor's"), None, 1);
        assert_eq!(ranked.len(), 1);
        assert_eq!(ranked[0].job.title, "Rust Developer");
    }
}
