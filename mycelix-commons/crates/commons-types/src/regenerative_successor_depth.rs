// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for successor-lineage depth distributions derived from a controlled
//! regenerative viability frontier.
//!
//! This record composes an existing Mycelix frontier record with exact dynamic and
//! semantic successor-depth subjects. Mycelix validates the histogram and linkage;
//! it does not recompute the successor simulation or static support theorem.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativeViabilityFrontierEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_SUCCESSOR_DEPTH_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_BUCKETS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSuccessorDepthBucketV1 {
    pub complete_periods: u64,
    pub case_count: u32,
}

/// Strict provenance summary for the successor-lineage depth distribution attached
/// to one exact parent viability frontier.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSuccessorDepthEvidenceV1 {
    pub schema_version: u8,
    pub depth_frontier_id: String,
    /// Raw content digest of the exact Mycelix parent viability-frontier record.
    pub parent_frontier_content_digest: String,
    /// Exact second shared fixture identity.
    pub depth_fixture_content_binding: String,
    /// Exact dynamic successor-depth subject.
    pub symtropy_depth_binding: String,
    /// Exact independent static successor-depth subject.
    pub symthaea_depth_binding: String,
    pub total_cases: u32,
    pub reproduction_ready_cases: u32,
    pub no_successor_cases: u32,
    /// Exact-horizon histogram, strictly increasing by `complete_periods`.
    pub depth_buckets: Vec<RegenerativeSuccessorDepthBucketV1>,
}

impl RegenerativeSuccessorDepthEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_SUCCESSOR_DEPTH_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative successor depth schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.depth_frontier_id) {
            return Err("depth_frontier_id is not canonical".into());
        }
        if !lower_hex_64(&self.parent_frontier_content_digest) {
            return Err("parent_frontier_content_digest must be lowercase 64-hex".into());
        }
        for (field, binding) in [
            (
                "depth_fixture_content_binding",
                self.depth_fixture_content_binding.as_str(),
            ),
            ("symtropy_depth_binding", self.symtropy_depth_binding.as_str()),
            ("symthaea_depth_binding", self.symthaea_depth_binding.as_str()),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.total_cases == 0 {
            return Err("successor-depth frontier requires at least one case".into());
        }
        if self.depth_buckets.is_empty() || self.depth_buckets.len() > MAX_BUCKETS {
            return Err(format!("successor-depth frontier requires 1..={MAX_BUCKETS} buckets"));
        }
        let mut previous_periods = 0u64;
        let mut bucket_total = 0u32;
        for bucket in &self.depth_buckets {
            if bucket.complete_periods == 0 || bucket.complete_periods <= previous_periods {
                return Err("successor-depth buckets must have strictly increasing positive periods".into());
            }
            if bucket.case_count == 0 {
                return Err("successor-depth buckets may not contain zero-count entries".into());
            }
            previous_periods = bucket.complete_periods;
            bucket_total = bucket_total
                .checked_add(bucket.case_count)
                .ok_or_else(|| "successor-depth bucket count overflow".to_string())?;
        }
        if bucket_total != self.reproduction_ready_cases {
            return Err("successor-depth bucket total must equal reproduction_ready_cases".into());
        }
        let total = self
            .reproduction_ready_cases
            .checked_add(self.no_successor_cases)
            .ok_or_else(|| "successor-depth total count overflow".to_string())?;
        if total != self.total_cases {
            return Err("total_cases must equal reproduction_ready + no_successor".into());
        }
        Ok(())
    }

    pub fn cases_at_least(&self, complete_periods: u64) -> u32 {
        self.depth_buckets
            .iter()
            .filter(|bucket| bucket.complete_periods >= complete_periods)
            .map(|bucket| bucket.case_count)
            .sum()
    }

    pub fn max_observed_depth(&self) -> u64 {
        self.depth_buckets
            .last()
            .map(|bucket| bucket.complete_periods)
            .unwrap_or(0)
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize successor depth evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-successor-depth-v1\0");
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

/// Verify that a successor-depth record composes exactly with its Mycelix parent
/// viability-frontier record.
pub fn verify_regenerative_successor_depth_evidence(
    parent: &RegenerativeViabilityFrontierEvidenceV1,
    depth: &RegenerativeSuccessorDepthEvidenceV1,
) -> Result<(), String> {
    parent.validate()?;
    depth.validate()?;
    if depth.parent_frontier_content_digest != parent.content_digest()? {
        return Err("successor-depth parent frontier digest mismatch".into());
    }
    if depth.total_cases != parent.total_cases {
        return Err("successor-depth total case count disagrees with parent frontier".into());
    }
    if depth.reproduction_ready_cases != parent.reproduction_ready_cases {
        return Err(
            "successor-depth reproduction-ready count disagrees with parent frontier".into(),
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
