// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for descendant reproduction-policy frontiers.
//!
//! This record preserves how an exact controlled lineage-depth experiment changes
//! when only the descendant maturity policy changes. Mycelix validates provenance,
//! policy ordering, histogram arithmetic, and parent composition; it does not run
//! industrial dynamics or recompute semantic support qualification.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativeMultigenerationDepthEvidenceV1,
    RegenerativeSuccessorDepthBucketV1, RegenerativeSuccessorDepthEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_REPRODUCTION_POLICY_FRONTIER_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_POLICIES: usize = 256;
const MAX_BUCKETS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeReproductionPolicyOutcomeV1 {
    /// Minimum complete descendant periods required before another handoff may occur.
    pub maturity_periods: u64,
    pub total_successful_handoff_count: u32,
    /// Exact descendant-generation depth histogram for this policy.
    pub generation_depth_buckets: Vec<RegenerativeSuccessorDepthBucketV1>,
}

impl RegenerativeReproductionPolicyOutcomeV1 {
    fn validate(&self, reproductive_case_count: u32) -> Result<(), String> {
        if self.maturity_periods == 0 {
            return Err("reproduction-policy maturity_periods must be positive".into());
        }
        if self.generation_depth_buckets.is_empty()
            || self.generation_depth_buckets.len() > MAX_BUCKETS
        {
            return Err(format!(
                "reproduction-policy outcome requires 1..={MAX_BUCKETS} depth buckets"
            ));
        }

        let mut previous_depth = 0u64;
        let mut case_total = 0u32;
        let mut weighted_handoffs = 0u64;
        for bucket in &self.generation_depth_buckets {
            if bucket.complete_periods == 0 || bucket.complete_periods <= previous_depth {
                return Err(
                    "policy depth buckets must have strictly increasing positive depth".into(),
                );
            }
            if bucket.case_count == 0 {
                return Err("policy depth buckets may not contain zero-count entries".into());
            }
            previous_depth = bucket.complete_periods;
            case_total = case_total
                .checked_add(bucket.case_count)
                .ok_or_else(|| "policy case-count overflow".to_string())?;
            weighted_handoffs = weighted_handoffs
                .checked_add(
                    bucket
                        .complete_periods
                        .checked_mul(u64::from(bucket.case_count))
                        .ok_or_else(|| "policy weighted-handoff overflow".to_string())?,
                )
                .ok_or_else(|| "policy weighted-handoff overflow".to_string())?;
        }
        if case_total != reproductive_case_count {
            return Err("policy depth buckets must cover every reproductive case".into());
        }
        if weighted_handoffs != u64::from(self.total_successful_handoff_count) {
            return Err(
                "policy weighted depth histogram must equal total successful handoffs".into(),
            );
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeReproductionPolicyFrontierEvidenceV1 {
    pub schema_version: u8,
    pub policy_frontier_id: String,
    /// Raw content digest of the exact Mycelix successor-depth record whose physical
    /// starting frontier is held fixed across every policy row.
    pub parent_successor_depth_content_digest: String,
    /// Raw content digest of the exact one-period multi-generation baseline record.
    pub baseline_multigeneration_content_digest: String,
    pub fixture_content_binding: String,
    pub symtropy_policy_frontier_binding: String,
    pub symthaea_policy_frontier_binding: String,
    pub policy_dimension_binding: String,
    pub reproductive_case_count: u32,
    /// Strictly increasing by `maturity_periods`.
    pub outcomes: Vec<RegenerativeReproductionPolicyOutcomeV1>,
    /// Upstream comparative claim that increasing maturity strictly reduced the
    /// aggregate number of successful handoffs over this exact controlled frontier.
    pub longer_maturity_strictly_reduces_total_handoffs: bool,
}

impl RegenerativeReproductionPolicyFrontierEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_REPRODUCTION_POLICY_FRONTIER_SCHEMA_V1 {
            return Err(format!(
                "unsupported reproduction policy frontier schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.policy_frontier_id) {
            return Err("policy_frontier_id is not canonical".into());
        }
        if !lower_hex_64(&self.parent_successor_depth_content_digest)
            || !lower_hex_64(&self.baseline_multigeneration_content_digest)
        {
            return Err("composed policy-frontier digests must be lowercase 64-hex".into());
        }
        for (field, binding) in [
            ("fixture_content_binding", self.fixture_content_binding.as_str()),
            (
                "symtropy_policy_frontier_binding",
                self.symtropy_policy_frontier_binding.as_str(),
            ),
            (
                "symthaea_policy_frontier_binding",
                self.symthaea_policy_frontier_binding.as_str(),
            ),
            ("policy_dimension_binding", self.policy_dimension_binding.as_str()),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.reproductive_case_count == 0 {
            return Err("policy frontier requires at least one reproductive case".into());
        }
        if self.outcomes.is_empty() || self.outcomes.len() > MAX_POLICIES {
            return Err(format!("policy frontier requires 1..={MAX_POLICIES} outcomes"));
        }

        let mut previous_maturity = 0u64;
        let mut previous_handoffs: Option<u32> = None;
        for outcome in &self.outcomes {
            outcome.validate(self.reproductive_case_count)?;
            if outcome.maturity_periods <= previous_maturity {
                return Err("policy outcomes must be strictly ordered by maturity_periods".into());
            }
            if self.longer_maturity_strictly_reduces_total_handoffs {
                if let Some(previous) = previous_handoffs {
                    if outcome.total_successful_handoff_count >= previous {
                        return Err(
                            "strict maturity monotonicity claim requires decreasing handoff counts"
                                .into(),
                        );
                    }
                }
            }
            previous_maturity = outcome.maturity_periods;
            previous_handoffs = Some(outcome.total_successful_handoff_count);
        }
        Ok(())
    }

    pub fn outcome_for_maturity(
        &self,
        maturity_periods: u64,
    ) -> Option<&RegenerativeReproductionPolicyOutcomeV1> {
        self.outcomes
            .iter()
            .find(|outcome| outcome.maturity_periods == maturity_periods)
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize reproduction policy frontier: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-reproduction-policy-frontier-v1\0");
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

pub fn verify_regenerative_reproduction_policy_frontier_evidence(
    parent_depth: &RegenerativeSuccessorDepthEvidenceV1,
    baseline: &RegenerativeMultigenerationDepthEvidenceV1,
    frontier: &RegenerativeReproductionPolicyFrontierEvidenceV1,
) -> Result<(), String> {
    parent_depth.validate()?;
    baseline.validate()?;
    frontier.validate()?;

    if frontier.parent_successor_depth_content_digest != parent_depth.content_digest()? {
        return Err("policy frontier parent successor-depth digest mismatch".into());
    }
    if frontier.baseline_multigeneration_content_digest != baseline.content_digest()? {
        return Err("policy frontier baseline multigeneration digest mismatch".into());
    }
    if frontier.reproductive_case_count != parent_depth.reproduction_ready_cases
        || frontier.reproductive_case_count != baseline.reproductive_case_count
    {
        return Err("policy frontier reproductive case count disagrees with parent evidence".into());
    }

    let one_period = frontier
        .outcome_for_maturity(1)
        .ok_or_else(|| "policy frontier must carry the one-period baseline outcome".to_string())?;
    if one_period.total_successful_handoff_count != baseline.total_successful_handoff_count
        || one_period.generation_depth_buckets != baseline.generation_depth_buckets
    {
        return Err("one-period policy outcome disagrees with exact multigeneration baseline".into());
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
