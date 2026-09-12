// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for intergenerational regenerative support-basis qualification.
//!
//! Symthaea owns semantic qualification of quantity/time-basis continuity and
//! Symtropy owns runtime inventory/accounting behavior. Mycelix preserves those
//! claims, checks canonical/arithmetic consistency, and prevents an unsafe basis
//! record from authorizing a scalar policy-sensitivity surface.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativePolicySensitivitySurfaceEvidenceV1,
    RegenerativeViabilityEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_ASSESSMENTS: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSupportBasisAssessmentEvidenceV1 {
    pub source_dependency_id: String,
    pub successor_dependency_id: String,
    pub quantity_basis_equivalence_binding: String,
    pub source_stockpile_draw_units_per_period: u64,
    pub successor_stockpile_draw_units_per_period: u64,
    pub source_period_duration_ms: u64,
    pub successor_period_duration_ms: u64,
    pub normalized_stockpile_draw_rate_preserved: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSupportBasisEvidenceV1 {
    pub schema_version: u8,
    pub basis_evidence_id: String,
    /// Exact Mycelix viability record for the physical source subject.
    pub viability_evidence_content_digest: String,
    /// Exact upstream engineering evidence.
    pub symthaea_support_basis_binding: String,
    pub symtropy_support_basis_binding: String,
    pub shared_support_basis_fixture_binding: String,
    pub basis_policy_id: String,
    pub basis_policy_evidence_binding: String,
    pub source_model_binding: String,
    pub successor_model_binding: String,
    /// Strictly sorted by `(source_dependency_id, successor_dependency_id)`.
    pub assessments: Vec<RegenerativeSupportBasisAssessmentEvidenceV1>,
    pub scalar_runway_projection_safe: bool,
    /// Optional exact policy-surface content digest. This MUST be absent when the
    /// basis is unsafe. A safe record may exist before a surface has been produced.
    pub authorized_policy_surface_content_digest: Option<String>,
}

impl RegenerativeSupportBasisEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_SUPPORT_BASIS_EVIDENCE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative support-basis schema version {}",
                self.schema_version
            ));
        }
        for id in [&self.basis_evidence_id, &self.basis_policy_id] {
            if !canonical_id(id) {
                return Err("support-basis identifier is not canonical".into());
            }
        }
        if !lower_hex_64(&self.viability_evidence_content_digest) {
            return Err("support-basis viability digest must be lowercase 64-hex".into());
        }
        for binding in [
            &self.symthaea_support_basis_binding,
            &self.symtropy_support_basis_binding,
            &self.shared_support_basis_fixture_binding,
            &self.basis_policy_evidence_binding,
            &self.source_model_binding,
            &self.successor_model_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("support-basis binding is not canonical".into());
            }
        }
        if self.assessments.is_empty() {
            return Err("support-basis evidence requires at least one assessment".into());
        }
        if self.assessments.len() > MAX_ASSESSMENTS {
            return Err(format!(
                "too many support-basis assessments (max {MAX_ASSESSMENTS})"
            ));
        }
        if self.assessments.windows(2).any(|pair| {
            (
                pair[0].source_dependency_id.as_str(),
                pair[0].successor_dependency_id.as_str(),
            ) >= (
                pair[1].source_dependency_id.as_str(),
                pair[1].successor_dependency_id.as_str(),
            )
        }) {
            return Err("support-basis assessments must be strictly sorted and unique".into());
        }

        let mut all_preserved = true;
        for assessment in &self.assessments {
            if !canonical_id(&assessment.source_dependency_id)
                || !canonical_id(&assessment.successor_dependency_id)
            {
                return Err("support-basis assessment dependency ID is not canonical".into());
            }
            if !canonical_reference(&assessment.quantity_basis_equivalence_binding) {
                return Err("support-basis quantity-basis binding is not canonical".into());
            }
            if assessment.source_period_duration_ms == 0
                || assessment.successor_period_duration_ms == 0
            {
                return Err("support-basis period durations must be positive".into());
            }
            let expected_preserved = u128::from(
                assessment.source_stockpile_draw_units_per_period,
            ) * u128::from(assessment.successor_period_duration_ms)
                == u128::from(assessment.successor_stockpile_draw_units_per_period)
                    * u128::from(assessment.source_period_duration_ms);
            if assessment.normalized_stockpile_draw_rate_preserved != expected_preserved {
                return Err(
                    "support-basis preserved flag disagrees with carried rate arithmetic".into(),
                );
            }
            all_preserved &= expected_preserved;
        }

        if self.scalar_runway_projection_safe != all_preserved {
            return Err(
                "support-basis scalar projection flag disagrees with transfer assessments".into(),
            );
        }
        if !self.scalar_runway_projection_safe
            && self.authorized_policy_surface_content_digest.is_some()
        {
            return Err(
                "unsafe support basis cannot authorize a scalar policy surface".into(),
            );
        }
        if let Some(digest) = &self.authorized_policy_surface_content_digest {
            if !lower_hex_64(digest) {
                return Err("authorized policy-surface digest must be lowercase 64-hex".into());
            }
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize support-basis evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-support-basis-evidence-v1\0");
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

pub fn verify_regenerative_support_basis_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
) -> Result<(), String> {
    viability.validate()?;
    basis.validate()?;
    if basis.viability_evidence_content_digest != viability.content_digest()? {
        return Err("support-basis viability-evidence digest mismatch".into());
    }
    Ok(())
}

pub fn verify_regenerative_support_basis_surface_authorization(
    basis: &RegenerativeSupportBasisEvidenceV1,
    surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> Result<(), String> {
    basis.validate()?;
    surface.validate()?;
    if !basis.scalar_runway_projection_safe {
        return Err("unsafe support basis cannot authorize a policy surface".into());
    }
    let expected = basis
        .authorized_policy_surface_content_digest
        .as_ref()
        .ok_or_else(|| "support-basis record does not bind an authorized surface".to_string())?;
    if expected != &surface.content_digest()? {
        return Err("support-basis authorized surface digest mismatch".into());
    }
    if basis.viability_evidence_content_digest != surface.viability_evidence_content_digest {
        return Err("support-basis and surface refer to different viability subjects".into());
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
