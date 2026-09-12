// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for decomposed descendant-generation policy metrics.
//!
//! A policy frontier's founded-descendant count is not the same as the number of
//! descendants that complete maturity or reproduce again. This record preserves
//! those three quantities separately and binds them to exact dynamic + semantic
//! evidence subjects without recomputing either engineering construction.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeReproductionPolicyFrontierEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_GENERATION_POLICY_METRICS_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_ROWS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeGenerationPolicyMetricRowV1 {
    pub maturity_periods: u64,
    pub founded_descendants: u32,
    pub maturity_completed_descendants: u32,
    pub descendant_reproduction_transitions: u32,
}

impl RegenerativeGenerationPolicyMetricRowV1 {
    fn validate(&self, reproductive_case_count: u32) -> Result<(), String> {
        if self.maturity_periods == 0 {
            return Err("generation-policy metric maturity_periods must be positive".into());
        }
        if self.founded_descendants < reproductive_case_count {
            return Err("founded descendants cannot be fewer than reproductive parent cases".into());
        }
        if self.maturity_completed_descendants > self.founded_descendants {
            return Err("maturity-completed descendants cannot exceed founded descendants".into());
        }
        if self.descendant_reproduction_transitions > self.maturity_completed_descendants {
            return Err("reproduction transitions cannot exceed maturity-completed descendants".into());
        }
        let expected_founded = reproductive_case_count
            .checked_add(self.descendant_reproduction_transitions)
            .ok_or_else(|| "generation-policy metric count overflow".to_string())?;
        if self.founded_descendants != expected_founded {
            return Err(
                "founded descendants must equal reproductive cases plus later transitions".into(),
            );
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeGenerationPolicyMetricsEvidenceV1 {
    pub schema_version: u8,
    pub metrics_id: String,
    /// Raw content digest of the exact policy-frontier provenance record.
    pub parent_policy_frontier_content_digest: String,
    pub metrics_fixture_binding: String,
    pub symtropy_dynamic_metrics_binding: String,
    /// Closed-form semantic projection subject.
    pub symthaea_projection_binding: String,
    /// Exact semantic metric-fixture subject.
    pub symthaea_metrics_binding: String,
    pub reproductive_case_count: u32,
    /// Strictly increasing by maturity period.
    pub rows: Vec<RegenerativeGenerationPolicyMetricRowV1>,
}

impl RegenerativeGenerationPolicyMetricsEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_GENERATION_POLICY_METRICS_SCHEMA_V1 {
            return Err(format!(
                "unsupported generation policy metrics schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.metrics_id) {
            return Err("metrics_id is not canonical".into());
        }
        if !lower_hex_64(&self.parent_policy_frontier_content_digest) {
            return Err("parent_policy_frontier_content_digest must be lowercase 64-hex".into());
        }
        for (field, binding) in [
            ("metrics_fixture_binding", self.metrics_fixture_binding.as_str()),
            (
                "symtropy_dynamic_metrics_binding",
                self.symtropy_dynamic_metrics_binding.as_str(),
            ),
            ("symthaea_projection_binding", self.symthaea_projection_binding.as_str()),
            ("symthaea_metrics_binding", self.symthaea_metrics_binding.as_str()),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.reproductive_case_count == 0 {
            return Err("generation policy metrics require reproductive cases".into());
        }
        if self.rows.is_empty() || self.rows.len() > MAX_ROWS {
            return Err(format!("generation policy metrics require 1..={MAX_ROWS} rows"));
        }

        let mut previous_maturity = 0u64;
        for row in &self.rows {
            row.validate(self.reproductive_case_count)?;
            if row.maturity_periods <= previous_maturity {
                return Err("generation policy metric rows must be strictly maturity-sorted".into());
            }
            previous_maturity = row.maturity_periods;
        }
        Ok(())
    }

    pub fn row_for_maturity(
        &self,
        maturity_periods: u64,
    ) -> Option<&RegenerativeGenerationPolicyMetricRowV1> {
        self.rows
            .iter()
            .find(|row| row.maturity_periods == maturity_periods)
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize generation policy metrics: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-generation-policy-metrics-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn to_maritime_envelope(
        &self,
        platform_id: impl Into<String>,
        generation: u64,
        sequence: u64,
        observed_at_us: u64,
        event_evidence_binding: impl Into<String>,
    ) -> Result<MaritimeEvidenceEnvelope, String> {
        let envelope = MaritimeEvidenceEnvelope::new(
            platform_id,
            generation,
            sequence,
            observed_at_us,
            MaritimeEvidenceKind::LogisticsEvent,
            self.to_payload_json()?,
            event_evidence_binding,
        );
        envelope.validate()?;
        Ok(envelope)
    }
}

pub fn verify_regenerative_generation_policy_metrics_evidence(
    parent: &RegenerativeReproductionPolicyFrontierEvidenceV1,
    metrics: &RegenerativeGenerationPolicyMetricsEvidenceV1,
) -> Result<(), String> {
    parent.validate()?;
    metrics.validate()?;
    if metrics.parent_policy_frontier_content_digest != parent.content_digest()? {
        return Err("generation policy metrics parent digest mismatch".into());
    }
    if metrics.reproductive_case_count != parent.reproductive_case_count {
        return Err("generation policy metrics reproductive case count mismatch".into());
    }
    if metrics.rows.len() != parent.outcomes.len() {
        return Err("generation policy metrics row coverage differs from policy frontier".into());
    }
    for (row, outcome) in metrics.rows.iter().zip(&parent.outcomes) {
        if row.maturity_periods != outcome.maturity_periods {
            return Err("generation policy metrics maturity ordering disagrees with parent".into());
        }
        if row.founded_descendants != outcome.total_successful_handoff_count {
            return Err("founded descendant count disagrees with parent successful handoffs".into());
        }
    }
    Ok(())
}

fn canonical_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_ID_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn canonical_reference(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_BINDING_BYTES
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}
