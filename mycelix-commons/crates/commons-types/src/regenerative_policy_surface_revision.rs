// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Immutable supersession provenance for regenerative policy-sensitivity surfaces.
//!
//! A changed physical assumption creates a new evidence subject. Prior policy
//! surfaces remain immutable and may be superseded by a separately validated new
//! surface; they are never rewritten in place.

use crate::{
    verify_regenerative_policy_sensitivity_surface_evidence, MaritimeEvidenceEnvelope,
    MaritimeEvidenceKind, RegenerativePolicySensitivitySurfaceEvidenceV1,
    RegenerativeViabilityEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_POLICY_SURFACE_REVISION_SCHEMA_V1: u8 = 1;
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativePolicySurfaceRevisionCauseV1 {
    AssumptionChange,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativePolicySurfaceRevisionEvidenceV1 {
    pub schema_version: u8,
    pub revision_id: String,
    pub cause: RegenerativePolicySurfaceRevisionCauseV1,
    pub prior_surface_content_digest: String,
    pub successor_surface_content_digest: String,
    /// Exact dynamic evidence establishing the changed carried state.
    pub dynamic_assumption_change_binding: String,
    /// Exact semantic/calibration evidence classifying the comparison.
    pub calibration_evidence_binding: String,
}

impl RegenerativePolicySurfaceRevisionEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_POLICY_SURFACE_REVISION_SCHEMA_V1 {
            return Err(format!(
                "unsupported policy surface revision schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.revision_id) {
            return Err("policy surface revision_id is not canonical".into());
        }
        if !lower_hex_64(&self.prior_surface_content_digest)
            || !lower_hex_64(&self.successor_surface_content_digest)
        {
            return Err("policy surface revision digests must be lowercase 64-hex".into());
        }
        if self.prior_surface_content_digest == self.successor_surface_content_digest {
            return Err("policy surface revision must name distinct evidence subjects".into());
        }
        for binding in [
            &self.dynamic_assumption_change_binding,
            &self.calibration_evidence_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("policy surface revision binding is not canonical".into());
            }
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize policy surface revision: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-policy-surface-revision-v1\0");
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

pub fn verify_regenerative_policy_surface_revision_evidence(
    prior_viability: &RegenerativeViabilityEvidenceV1,
    prior_surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
    successor_viability: &RegenerativeViabilityEvidenceV1,
    successor_surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
    revision: &RegenerativePolicySurfaceRevisionEvidenceV1,
) -> Result<(), String> {
    verify_regenerative_policy_sensitivity_surface_evidence(prior_viability, prior_surface)?;
    verify_regenerative_policy_sensitivity_surface_evidence(
        successor_viability,
        successor_surface,
    )?;
    revision.validate()?;

    if revision.prior_surface_content_digest != prior_surface.content_digest()? {
        return Err("prior policy-surface digest mismatch".into());
    }
    if revision.successor_surface_content_digest != successor_surface.content_digest()? {
        return Err("successor policy-surface digest mismatch".into());
    }
    if prior_surface.surface_id == successor_surface.surface_id {
        return Err("superseding policy surface requires a distinct surface_id".into());
    }
    if prior_viability.genome_binding != successor_viability.genome_binding
        || prior_viability.lineage_evidence_binding != successor_viability.lineage_evidence_binding
    {
        return Err(
            "assumption-change surface revision must stay on the same Genome lineage subject"
                .into(),
        );
    }
    if prior_viability.content_digest()? == successor_viability.content_digest()? {
        return Err("assumption change requires a distinct viability evidence subject".into());
    }
    if prior_surface.physical_successor_reproduction_horizon
        == successor_surface.physical_successor_reproduction_horizon
        && prior_surface.physical_regenerative_viability_horizon
            == successor_surface.physical_regenerative_viability_horizon
        && prior_surface.fully_modeled_successor_reproduction
            == successor_surface.fully_modeled_successor_reproduction
        && prior_surface.fully_modeled_regenerative_viability
            == successor_surface.fully_modeled_regenerative_viability
    {
        return Err("surface revision does not change the carried physical coordinate".into());
    }

    if prior_surface.policy_points.len() != successor_surface.policy_points.len() {
        return Err("assumption-change revision changed the policy family".into());
    }
    for (prior, successor) in prior_surface
        .policy_points
        .iter()
        .zip(&successor_surface.policy_points)
    {
        if prior.reproduction_policy_id != successor.reproduction_policy_id
            || prior.reproduction_policy_evidence_binding
                != successor.reproduction_policy_evidence_binding
            || prior.maturity_periods != successor.maturity_periods
        {
            return Err("assumption-change revision changed the policy family".into());
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
