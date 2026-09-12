// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for multi-generation regenerative lineage depth.
//!
//! This composes an exact successor-depth record with independent dynamic and
//! semantic multi-generation subjects. Mycelix verifies linkage and arithmetic
//! identities; it does not execute industrial simulation or support qualification.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativeSuccessorDepthBucketV1,
    RegenerativeSuccessorDepthEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_MULTIGENERATION_DEPTH_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_BUCKETS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeMultigenerationDepthEvidenceV1 {
    pub schema_version: u8,
    pub multigeneration_id: String,
    /// Raw content digest of the exact Mycelix successor-depth record.
    pub parent_successor_depth_content_digest: String,
    /// Exact dynamic multi-generation subject.
    pub symtropy_multigeneration_binding: String,
    /// Exact independent semantic/static multi-generation subject.
    pub symthaea_multigeneration_binding: String,
    /// Exact evidence binding for the reproduction policy used by both subjects.
    pub reproduction_policy_binding: String,
    pub reproductive_case_count: u32,
    pub initial_handoff_count: u32,
    pub descendant_handoff_count: u32,
    pub total_successful_handoff_count: u32,
    /// Exact descendant-generation histogram, strictly increasing by depth.
    pub generation_depth_buckets: Vec<RegenerativeSuccessorDepthBucketV1>,
    /// Upstream comparative claim: temporal successor runway and descendant-generation
    /// depth matched case-for-case under the bound reproduction policy.
    pub temporal_depth_matches_generation_depth: bool,
}

impl RegenerativeMultigenerationDepthEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_MULTIGENERATION_DEPTH_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative multigeneration depth schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.multigeneration_id) {
            return Err("multigeneration_id is not canonical".into());
        }
        if !lower_hex_64(&self.parent_successor_depth_content_digest) {
            return Err("parent_successor_depth_content_digest must be lowercase 64-hex".into());
        }
        for (field, binding) in [
            (
                "symtropy_multigeneration_binding",
                self.symtropy_multigeneration_binding.as_str(),
            ),
            (
                "symthaea_multigeneration_binding",
                self.symthaea_multigeneration_binding.as_str(),
            ),
            ("reproduction_policy_binding", self.reproduction_policy_binding.as_str()),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.reproductive_case_count == 0 {
            return Err("multigeneration depth requires at least one reproductive case".into());
        }
        if self.initial_handoff_count != self.reproductive_case_count {
            return Err("initial_handoff_count must equal reproductive_case_count".into());
        }
        let total_handoffs = self
            .initial_handoff_count
            .checked_add(self.descendant_handoff_count)
            .ok_or_else(|| "multigeneration handoff count overflow".to_string())?;
        if total_handoffs != self.total_successful_handoff_count {
            return Err("total_successful_handoff_count must equal initial + descendant handoffs".into());
        }
        if self.generation_depth_buckets.is_empty()
            || self.generation_depth_buckets.len() > MAX_BUCKETS
        {
            return Err(format!(
                "multigeneration depth requires 1..={MAX_BUCKETS} depth buckets"
            ));
        }

        let mut previous_depth = 0u64;
        let mut case_total = 0u32;
        let mut weighted_handoff_total = 0u64;
        for bucket in &self.generation_depth_buckets {
            if bucket.complete_periods == 0 || bucket.complete_periods <= previous_depth {
                return Err(
                    "generation-depth buckets must have strictly increasing positive depths"
                        .into(),
                );
            }
            if bucket.case_count == 0 {
                return Err("generation-depth buckets may not contain zero-count entries".into());
            }
            previous_depth = bucket.complete_periods;
            case_total = case_total
                .checked_add(bucket.case_count)
                .ok_or_else(|| "generation-depth case count overflow".to_string())?;
            weighted_handoff_total = weighted_handoff_total
                .checked_add(
                    bucket
                        .complete_periods
                        .checked_mul(u64::from(bucket.case_count))
                        .ok_or_else(|| "generation-depth weighted count overflow".to_string())?,
                )
                .ok_or_else(|| "generation-depth weighted count overflow".to_string())?;
        }
        if case_total != self.reproductive_case_count {
            return Err("generation-depth bucket total must equal reproductive_case_count".into());
        }
        if weighted_handoff_total != u64::from(self.total_successful_handoff_count) {
            return Err(
                "weighted generation-depth histogram must equal total successful handoffs".into(),
            );
        }
        Ok(())
    }

    pub fn max_generation_depth(&self) -> u64 {
        self.generation_depth_buckets
            .last()
            .map(|bucket| bucket.complete_periods)
            .unwrap_or(0)
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize multigeneration depth evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-multigeneration-depth-v1\0");
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

pub fn verify_regenerative_multigeneration_depth_evidence(
    parent: &RegenerativeSuccessorDepthEvidenceV1,
    multigeneration: &RegenerativeMultigenerationDepthEvidenceV1,
) -> Result<(), String> {
    parent.validate()?;
    multigeneration.validate()?;
    if multigeneration.parent_successor_depth_content_digest != parent.content_digest()? {
        return Err("multigeneration parent successor-depth digest mismatch".into());
    }
    if multigeneration.reproductive_case_count != parent.reproduction_ready_cases {
        return Err("multigeneration reproductive case count disagrees with parent depth".into());
    }
    if multigeneration.temporal_depth_matches_generation_depth
        && multigeneration.generation_depth_buckets != parent.depth_buckets
    {
        return Err(
            "temporal-depth equivalence claim requires exact parent/generation histogram agreement"
                .into(),
        );
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
