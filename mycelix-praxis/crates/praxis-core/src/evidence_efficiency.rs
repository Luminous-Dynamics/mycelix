// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Solver-visible candidate requests for the Praxis evidence-acquisition benchmark.
//!
//! This module contains no hidden hypothesis, expected outcome, or evaluator rubric.
//! The reference evaluator lives in the integration-test harness so it cannot be
//! linked into a learner-facing library or WASM artifact by accident.

use serde::{Deserialize, Serialize};
use std::collections::HashSet;

pub const EVIDENCE_EFFICIENCY_SCHEMA_VERSION: u16 = 1;
pub const EVIDENCE_EFFICIENCY_BENCHMARK_ID: &str = "PRAXIS-CYBER-ACQ-001";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceRequestCatalog {
    pub schema_version: u16,
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub scenarios: Vec<RequestScenario>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RequestScenario {
    pub scenario_id: String,
    /// Candidate hypotheses are learner-visible framing, not the evaluator's answer.
    pub hypotheses: Vec<String>,
    pub requests: Vec<EvidenceRequest>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceRequest {
    pub request_id: String,
    pub source_kind: EvidenceSourceKind,
    pub purpose: String,
    pub required_authority: RequestAuthority,
    /// Synthetic relative cost units for fair, reproducible policy comparison.
    pub cost_units: u32,
    /// Declared worst-case response latency bound, not measured elapsed time.
    pub latency_bound_minutes: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceSourceKind {
    IdentitySessionAudit,
    EndpointContext,
    TokenUseAudit,
    SensorHealth,
    SharedEgressSummary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestAuthority {
    ReadOnlyTelemetry,
    IncidentCommander,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CatalogValidationIssue {
    pub code: String,
    pub path: String,
    pub message: String,
}

/// Structural validation only. This does not score a policy or imply cyber competence.
pub fn validate_request_catalog(
    catalog: &EvidenceRequestCatalog,
) -> Result<(), Vec<CatalogValidationIssue>> {
    let mut issues = Vec::new();
    if catalog.schema_version != EVIDENCE_EFFICIENCY_SCHEMA_VERSION {
        issue(
            &mut issues,
            "unsupported_schema_version",
            "$.schema_version",
            "unsupported evidence-acquisition schema version",
        );
    }
    if catalog.benchmark_id != EVIDENCE_EFFICIENCY_BENCHMARK_ID {
        issue(
            &mut issues,
            "unexpected_benchmark_id",
            "$.benchmark_id",
            "unexpected benchmark ID",
        );
    }
    required_text(
        &mut issues,
        "$.benchmark_version",
        &catalog.benchmark_version,
        "benchmark version",
    );
    if catalog.scenarios.is_empty() {
        issue(
            &mut issues,
            "empty_scenarios",
            "$.scenarios",
            "at least one scenario is required",
        );
    }

    let mut scenario_ids = HashSet::new();
    for (scenario_index, scenario) in catalog.scenarios.iter().enumerate() {
        let base = format!("$.scenarios[{scenario_index}]");
        required_text(
            &mut issues,
            &format!("{base}.scenario_id"),
            &scenario.scenario_id,
            "scenario ID",
        );
        if !scenario_ids.insert(scenario.scenario_id.as_str()) {
            issue(
                &mut issues,
                "duplicate_scenario_id",
                format!("{base}.scenario_id"),
                "scenario IDs must be unique",
            );
        }
        if scenario.hypotheses.len() < 2 {
            issue(
                &mut issues,
                "insufficient_hypotheses",
                format!("{base}.hypotheses"),
                "at least two competing hypotheses are required",
            );
        }
        let mut hypothesis_ids = HashSet::new();
        for (index, hypothesis) in scenario.hypotheses.iter().enumerate() {
            required_text(
                &mut issues,
                &format!("{base}.hypotheses[{index}]"),
                hypothesis,
                "hypothesis ID",
            );
            if !hypothesis_ids.insert(hypothesis.as_str()) {
                issue(
                    &mut issues,
                    "duplicate_hypothesis_id",
                    format!("{base}.hypotheses[{index}]"),
                    "hypothesis IDs must be unique within a scenario",
                );
            }
        }
        if scenario.requests.is_empty() {
            issue(
                &mut issues,
                "empty_requests",
                format!("{base}.requests"),
                "at least one candidate evidence request is required",
            );
        }
        let mut request_ids = HashSet::new();
        for (request_index, request) in scenario.requests.iter().enumerate() {
            let path = format!("{base}.requests[{request_index}]");
            required_text(
                &mut issues,
                &format!("{path}.request_id"),
                &request.request_id,
                "request ID",
            );
            if !request_ids.insert(request.request_id.as_str()) {
                issue(
                    &mut issues,
                    "duplicate_request_id",
                    format!("{path}.request_id"),
                    "request IDs must be unique within a scenario",
                );
            }
            required_text(
                &mut issues,
                &format!("{path}.purpose"),
                &request.purpose,
                "request purpose",
            );
            if request.cost_units == 0 {
                issue(
                    &mut issues,
                    "zero_cost",
                    format!("{path}.cost_units"),
                    "cost must be positive to prevent free-request gaming",
                );
            }
            if request.latency_bound_minutes == 0 {
                issue(
                    &mut issues,
                    "zero_latency_bound",
                    format!("{path}.latency_bound_minutes"),
                    "latency bound must be positive",
                );
            }
        }
    }

    if issues.is_empty() {
        Ok(())
    } else {
        Err(issues)
    }
}

fn required_text(issues: &mut Vec<CatalogValidationIssue>, path: &str, value: &str, label: &str) {
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
    issues: &mut Vec<CatalogValidationIssue>,
    code: &str,
    path: impl Into<String>,
    message: impl Into<String>,
) {
    issues.push(CatalogValidationIssue {
        code: code.to_string(),
        path: path.into(),
        message: message.into(),
    });
}
