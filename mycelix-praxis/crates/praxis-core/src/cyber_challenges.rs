// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Solver-visible portion of Praxis's synthetic cyber-engineering challenge corpus.
//!
//! This module deliberately does not embed or load the evaluator-only fixture.
//! The evaluator belongs in test/qualification tooling and must never be included
//! in a learner-facing runtime projection.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const CYBER_CHALLENGE_SCHEMA_VERSION: u16 = 1;
pub const CYBER_CHALLENGE_CORPUS_ID: &str = "PRAXIS-CYBER-001";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolverVisibleCyberCorpus {
    pub schema_version: u16,
    pub corpus_id: String,
    pub corpus_version: String,
    pub scenarios: Vec<SolverVisibleScenario>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolverVisibleScenario {
    pub scenario_id: String,
    pub is_synthetic: bool,
    pub domain: CyberDomain,
    pub difficulty: ChallengeDifficulty,
    pub title: String,
    pub objective: String,
    pub prompt: String,
    pub evidence: Vec<EvidenceObservation>,
    pub allowed_next_steps: Vec<String>,
    pub action_authority_boundary: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Hash, PartialOrd)]
#[serde(rename_all = "snake_case")]
pub enum CyberDomain {
    IdentityAccess,
    NetworkConfiguration,
    SupplyChainProvenance,
    AiToolBoundary,
    EndpointTelemetry,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Hash, PartialOrd)]
#[serde(rename_all = "snake_case")]
pub enum ChallengeDifficulty {
    Beginner,
    Intermediate,
    Advanced,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceObservation {
    pub evidence_id: String,
    pub source_kind: EvidenceSourceKind,
    pub freshness: EvidenceFreshness,
    /// Minutes relative to the scenario reference time; negative means older.
    pub observed_at_offset_minutes: i64,
    pub observation: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Hash, PartialOrd)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceSourceKind {
    IdentityLogSummary,
    NetworkContext,
    EndpointTelemetry,
    NetworkPolicySnapshot,
    FirewallPolicySnapshot,
    DependencyLockfile,
    ApprovedDependencyRecord,
    RegistryMetadata,
    RetrievedDocumentExcerpt,
    ToolPermissionSnapshot,
    ToolAuditSummary,
    ResponseRequestLog,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, Hash, PartialOrd)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceFreshness {
    Current,
    Stale,
    Conflicting,
    Unknown,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CyberCorpusValidationIssue {
    pub code: String,
    pub path: String,
    pub message: String,
}

/// Structural summary only: it is not a learner score, credential, or finding
/// that any learner has demonstrated cyber-engineering capability.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SolverCorpusSummary {
    pub corpus_id: String,
    pub corpus_version: String,
    pub scenario_count: usize,
    pub evidence_count: usize,
}

/// Validate only the solver-visible contract. Evaluator fixtures are deliberately
/// excluded from this production function and must be checked by separate test
/// tooling, never shipped to the learner application.
pub fn validate_solver_corpus(
    corpus: &SolverVisibleCyberCorpus,
) -> Result<SolverCorpusSummary, Vec<CyberCorpusValidationIssue>> {
    let mut issues = Vec::new();

    if corpus.schema_version != CYBER_CHALLENGE_SCHEMA_VERSION {
        issue(
            &mut issues,
            "unsupported_schema_version",
            "$.schema_version",
            format!(
                "expected {}, got {}",
                CYBER_CHALLENGE_SCHEMA_VERSION, corpus.schema_version
            ),
        );
    }
    if corpus.corpus_id != CYBER_CHALLENGE_CORPUS_ID {
        issue(
            &mut issues,
            "unexpected_corpus_id",
            "$.corpus_id",
            format!("expected {:?}, got {:?}", CYBER_CHALLENGE_CORPUS_ID, corpus.corpus_id),
        );
    }
    if corpus.corpus_version.trim().is_empty() {
        issue(
            &mut issues,
            "empty_corpus_version",
            "$.corpus_version",
            "corpus version is required",
        );
    }
    if corpus.scenarios.is_empty() {
        issue(
            &mut issues,
            "empty_scenarios",
            "$.scenarios",
            "at least one synthetic scenario is required",
        );
    }

    let mut scenario_ids = HashSet::new();
    let mut evidence_count = 0usize;

    for (scenario_index, scenario) in corpus.scenarios.iter().enumerate() {
        let base = format!("$.scenarios[{scenario_index}]");

        if !valid_scenario_id(&scenario.scenario_id) {
            issue(
                &mut issues,
                "invalid_scenario_id",
                format!("{base}.scenario_id"),
                "scenario ID must use CYB-UPPERCASE-CODE-000 form",
            );
        }
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            issue(
                &mut issues,
                "duplicate_scenario_id",
                format!("{base}.scenario_id"),
                format!("duplicate scenario ID {:?}", scenario.scenario_id),
            );
        }
        if !scenario.is_synthetic {
            issue(
                &mut issues,
                "non_synthetic_scenario",
                format!("{base}.is_synthetic"),
                "this public conformance corpus must contain synthetic scenarios only",
            );
        }
        required_text(
            &mut issues,
            &format!("{base}.title"),
            &scenario.title,
            "title",
        );
        required_text(
            &mut issues,
            &format!("{base}.objective"),
            &scenario.objective,
            "objective",
        );
        required_text(
            &mut issues,
            &format!("{base}.prompt"),
            &scenario.prompt,
            "prompt",
        );
        required_text(
            &mut issues,
            &format!("{base}.action_authority_boundary"),
            &scenario.action_authority_boundary,
            "action authority boundary",
        );

        if scenario.evidence.is_empty() {
            issue(
                &mut issues,
                "empty_evidence",
                format!("{base}.evidence"),
                "a challenge must expose at least one evidence observation",
            );
        }
        if scenario.allowed_next_steps.is_empty() {
            issue(
                &mut issues,
                "empty_allowed_next_steps",
                format!("{base}.allowed_next_steps"),
                "permitted, bounded next steps must be declared",
            );
        }

        let mut evidence_ids = HashSet::new();
        for (evidence_index, evidence) in scenario.evidence.iter().enumerate() {
            evidence_count += 1;
            let path = format!("{base}.evidence[{evidence_index}]");
            if evidence.evidence_id.trim().is_empty() {
                issue(
                    &mut issues,
                    "empty_evidence_id",
                    format!("{path}.evidence_id"),
                    "evidence ID is required",
                );
            } else if !evidence_ids.insert(evidence.evidence_id.as_str()) {
                issue(
                    &mut issues,
                    "duplicate_evidence_id",
                    format!("{path}.evidence_id"),
                    format!("duplicate evidence ID {:?}", evidence.evidence_id),
                );
            }
            required_text(
                &mut issues,
                &format!("{path}.observation"),
                &evidence.observation,
                "observation",
            );
        }

        for (step_index, step) in scenario.allowed_next_steps.iter().enumerate() {
            required_text(
                &mut issues,
                &format!("{base}.allowed_next_steps[{step_index}]"),
                step,
                "allowed next step",
            );
        }
    }

    if issues.is_empty() {
        Ok(SolverCorpusSummary {
            corpus_id: corpus.corpus_id.clone(),
            corpus_version: corpus.corpus_version.clone(),
            scenario_count: corpus.scenarios.len(),
            evidence_count,
        })
    } else {
        Err(issues)
    }
}

fn valid_scenario_id(value: &str) -> bool {
    let mut parts = value.split('-');
    let Some(prefix) = parts.next() else {
        return false;
    };
    let Some(domain) = parts.next() else {
        return false;
    };
    let Some(sequence) = parts.next() else {
        return false;
    };
    parts.next().is_none()
        && prefix == "CYB"
        && !domain.is_empty()
        && domain.bytes().all(|byte| byte.is_ascii_uppercase())
        && sequence.len() == 3
        && sequence.bytes().all(|byte| byte.is_ascii_digit())
}

fn required_text(
    issues: &mut Vec<CyberCorpusValidationIssue>,
    path: &str,
    value: &str,
    label: &str,
) {
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
    issues: &mut Vec<CyberCorpusValidationIssue>,
    code: &str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(CyberCorpusValidationIssue {
        code: code.to_string(),
        path: path.into(),
        message: message.into(),
    });
}
