// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Solver-visible final-decision wire contract for Praxis cyber evidence tasks.
//!
//! This module contains no evaluator answer key, truth labels, or scoring weights.
//! Validation checks structure and references against the scenario context supplied
//! to the solver. It does not decide whether evidence is truthful, fresh, sufficient,
//! or strong enough to justify a real-world security action.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const DECISION_SUBMISSION_SCHEMA_VERSION: u16 = 1;
pub const DECISION_SUBMISSION_BENCHMARK_ID: &str = "PRAXIS-CYBER-DECISION-001";

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionSubmission {
    pub schema_version: u16,
    pub benchmark_id: String,
    pub scenario_id: String,
    pub outcome: DecisionOutcome,
}

/// The tagged union prevents a submission from simultaneously claiming, abstaining,
/// and requesting more evidence. No free-form answer is used as a scoring primitive.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum DecisionOutcome {
    Claim {
        hypothesis_id: String,
        /// Observed evidence IDs, never request IDs.
        cited_evidence_ids: Vec<String>,
        /// Optional and only meaningful when a preregistered metric will score it.
        confidence: Option<f64>,
        rationale: DecisionRationale,
    },
    Abstain {
        reason: AbstentionReason,
        cited_evidence_ids: Vec<String>,
        missing_evidence: Vec<MissingEvidenceKind>,
    },
    RequestMoreEvidence {
        /// Catalog request IDs already filtered by the caller's authority and budget.
        request_ids: Vec<String>,
        reason: EvidenceRequestReason,
    },
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DecisionRationale {
    pub supporting_evidence_ids: Vec<String>,
    pub contradictory_evidence_ids: Vec<String>,
    pub missing_evidence: Vec<MissingEvidenceKind>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AbstentionReason {
    InsufficientEvidence,
    ConflictingEvidence,
    StaleEvidence,
    UnavailableEvidence,
    AmbiguousEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissingEvidenceKind {
    IndependentCorroboration,
    FreshnessVerification,
    ConflictResolution,
    SourceCoverage,
    EffectReceipt,
    AuthorityConfirmation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceRequestReason {
    ResolveAmbiguity,
    VerifyFreshness,
    ResolveConflict,
    FillCoverageGap,
    ConfirmEffectReceipt,
    SeekIndependentCorroboration,
}

/// All lists must be derived from the solver-visible scenario. In particular,
/// `authorized_request_ids` must exclude out-of-catalog, unauthorized, and
/// budget-ineligible requests before validation.
#[derive(Clone, Copy, Debug)]
pub struct DecisionContext<'a> {
    pub scenario_id: &'a str,
    pub hypothesis_ids: &'a [String],
    pub available_evidence_ids: &'a [String],
    pub authorized_request_ids: &'a [String],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DecisionValidationIssue {
    pub code: String,
    pub path: String,
    pub message: String,
}

/// Validate wire-contract shape and reference membership only.
///
/// This is not an evaluator: it does not adjudicate evidence truth, freshness,
/// relevance, sufficiency, claim correctness, or whether a requested action took
/// effect. Those judgments require a separate evaluator rubric and receipts.
pub fn validate_decision_submission(
    submission: &DecisionSubmission,
    context: &DecisionContext<'_>,
) -> Result<(), Vec<DecisionValidationIssue>> {
    let mut issues = Vec::new();

    if submission.schema_version != DECISION_SUBMISSION_SCHEMA_VERSION {
        issue(
            &mut issues,
            "unsupported_schema_version",
            "$.schema_version",
            "unsupported decision-submission schema version",
        );
    }
    required_text(
        &mut issues,
        "$.benchmark_id",
        &submission.benchmark_id,
        "benchmark ID",
    );
    if submission.benchmark_id != DECISION_SUBMISSION_BENCHMARK_ID {
        issue(
            &mut issues,
            "unexpected_benchmark_id",
            "$.benchmark_id",
            "unexpected decision-submission benchmark ID",
        );
    }
    required_text(
        &mut issues,
        "$.scenario_id",
        &submission.scenario_id,
        "scenario ID",
    );
    if submission.scenario_id != context.scenario_id {
        issue(
            &mut issues,
            "scenario_mismatch",
            "$.scenario_id",
            "submission scenario does not match the supplied solver-visible context",
        );
    }

    match &submission.outcome {
        DecisionOutcome::Claim {
            hypothesis_id,
            cited_evidence_ids,
            confidence,
            rationale,
        } => {
            required_text(
                &mut issues,
                "$.outcome.hypothesis_id",
                hypothesis_id,
                "hypothesis ID",
            );
            if !context.hypothesis_ids.iter().any(|id| id == hypothesis_id) {
                issue(
                    &mut issues,
                    "unknown_hypothesis_id",
                    "$.outcome.hypothesis_id",
                    "claim references a hypothesis outside the scenario",
                );
            }

            validate_evidence_ids(
                &mut issues,
                "$.outcome.cited_evidence_ids",
                cited_evidence_ids,
                context.available_evidence_ids,
                true,
            );
            let cited = cited_evidence_ids
                .iter()
                .map(String::as_str)
                .collect::<HashSet<_>>();

            if rationale.supporting_evidence_ids.is_empty() {
                issue(
                    &mut issues,
                    "missing_supporting_evidence",
                    "$.outcome.rationale.supporting_evidence_ids",
                    "a definitive claim must identify at least one supporting evidence ID",
                );
            }
            validate_evidence_ids(
                &mut issues,
                "$.outcome.rationale.supporting_evidence_ids",
                &rationale.supporting_evidence_ids,
                context.available_evidence_ids,
                false,
            );
            for (index, evidence_id) in rationale.supporting_evidence_ids.iter().enumerate() {
                if !cited.contains(evidence_id.as_str()) {
                    issue(
                        &mut issues,
                        "supporting_evidence_not_cited",
                        format!("$.outcome.rationale.supporting_evidence_ids[{index}]"),
                        "supporting evidence must also appear in cited_evidence_ids",
                    );
                }
            }

            validate_evidence_ids(
                &mut issues,
                "$.outcome.rationale.contradictory_evidence_ids",
                &rationale.contradictory_evidence_ids,
                context.available_evidence_ids,
                false,
            );
            for (index, evidence_id) in rationale.contradictory_evidence_ids.iter().enumerate() {
                if !cited.contains(evidence_id.as_str()) {
                    issue(
                        &mut issues,
                        "contradictory_evidence_not_cited",
                        format!("$.outcome.rationale.contradictory_evidence_ids[{index}]"),
                        "identified contradictory evidence must also appear in cited_evidence_ids",
                    );
                }
            }
            validate_missing_evidence(
                &mut issues,
                "$.outcome.rationale.missing_evidence",
                &rationale.missing_evidence,
            );

            if let Some(value) = confidence {
                if !value.is_finite() || !(0.0..=1.0).contains(value) {
                    issue(
                        &mut issues,
                        "confidence_out_of_range",
                        "$.outcome.confidence",
                        "confidence must be finite and within the inclusive range [0, 1]",
                    );
                }
            }
        }
        DecisionOutcome::Abstain {
            cited_evidence_ids,
            missing_evidence,
            ..
        } => {
            validate_evidence_ids(
                &mut issues,
                "$.outcome.cited_evidence_ids",
                cited_evidence_ids,
                context.available_evidence_ids,
                false,
            );
            validate_missing_evidence(
                &mut issues,
                "$.outcome.missing_evidence",
                missing_evidence,
            );
            if cited_evidence_ids.is_empty() && missing_evidence.is_empty() {
                issue(
                    &mut issues,
                    "missing_decision_rationale",
                    "$.outcome",
                    "an abstention must cite observed evidence or name a structured missing-evidence kind",
                );
            }
        }
        DecisionOutcome::RequestMoreEvidence { request_ids, .. } => {
            if request_ids.is_empty() {
                issue(
                    &mut issues,
                    "missing_request_id",
                    "$.outcome.request_ids",
                    "request_more_evidence must name at least one authorized catalog request",
                );
            }
            let mut seen = HashSet::new();
            for (index, request_id) in request_ids.iter().enumerate() {
                let path = format!("$.outcome.request_ids[{index}]");
                if request_id.trim().is_empty() {
                    issue(
                        &mut issues,
                        "empty_required_text",
                        path.clone(),
                        "request ID must not be empty",
                    );
                }
                if !seen.insert(request_id.as_str()) {
                    issue(
                        &mut issues,
                        "duplicate_request_id",
                        path.clone(),
                        "request IDs must be unique within a submission",
                    );
                }
                if !context
                    .authorized_request_ids
                    .iter()
                    .any(|authorized| authorized == request_id)
                {
                    issue(
                        &mut issues,
                        "unknown_or_unauthorized_request_id",
                        path,
                        "request ID is not in the supplied authority- and budget-filtered request set",
                    );
                }
            }
        }
    }

    if issues.is_empty() {
        Ok(())
    } else {
        Err(issues)
    }
}

fn validate_evidence_ids(
    issues: &mut Vec<DecisionValidationIssue>,
    path: &str,
    ids: &[String],
    available_ids: &[String],
    require_nonempty: bool,
) {
    if require_nonempty && ids.is_empty() {
        issue(
            issues,
            "missing_evidence_citation",
            path,
            "at least one observed evidence ID is required",
        );
    }

    let mut seen = HashSet::new();
    for (index, evidence_id) in ids.iter().enumerate() {
        let item_path = format!("{path}[{index}]");
        if evidence_id.trim().is_empty() {
            issue(
                issues,
                "empty_required_text",
                item_path.clone(),
                "evidence ID must not be empty",
            );
        }
        if !seen.insert(evidence_id.as_str()) {
            issue(
                issues,
                "duplicate_evidence_id",
                item_path.clone(),
                "evidence IDs must be unique within each reference list",
            );
        }
        if !available_ids.iter().any(|available| available == evidence_id) {
            issue(
                issues,
                "unknown_evidence_id",
                item_path,
                "reference must identify observed evidence available in the scenario",
            );
        }
    }
}

fn validate_missing_evidence(
    issues: &mut Vec<DecisionValidationIssue>,
    path: &str,
    kinds: &[MissingEvidenceKind],
) {
    let mut seen = HashSet::new();
    for (index, kind) in kinds.iter().enumerate() {
        if !seen.insert(*kind) {
            issue(
                issues,
                "duplicate_missing_evidence_kind",
                format!("{path}[{index}]"),
                "missing-evidence kinds must be unique within each list",
            );
        }
    }
}

fn required_text(issues: &mut Vec<DecisionValidationIssue>, path: &str, value: &str, label: &str) {
    if value.trim().is_empty() {
        issue(
            issues,
            "empty_required_text",
            path,
            format!("{label} is required"),
        );
    }
}

fn issue(
    issues: &mut Vec<DecisionValidationIssue>,
    code: &str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(DecisionValidationIssue {
        code: code.to_string(),
        path: path.into(),
        message: message.into(),
    });
}
