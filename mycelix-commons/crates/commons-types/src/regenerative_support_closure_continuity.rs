// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for intergenerational role-support closure continuity.
//!
//! Symthaea owns graph/isomorphism qualification and Symtropy owns dynamic handoff
//! behavior. Mycelix preserves the exact closure/root claims and composes them with
//! support-basis evidence without re-running either engineering theorem.

use crate::{
    verify_regenerative_support_basis_surface_authorization, MaritimeEvidenceEnvelope,
    MaritimeEvidenceKind, RegenerativePolicySensitivitySurfaceEvidenceV1,
    RegenerativeSupportBasisEvidenceV1, RegenerativeViabilityEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_SUPPORT_CLOSURE_CONTINUITY_SCHEMA_V1: u8 = 1;
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_ITEMS: usize = 4096;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSupportClosureRootPairEvidenceV1 {
    pub source_dependency_id: String,
    pub successor_dependency_id: String,
    pub transfer_qualified: bool,
    pub basis_qualified: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeSupportClosureContinuityEvidenceV1 {
    pub schema_version: u8,
    pub continuity_evidence_id: String,
    pub viability_evidence_content_digest: String,
    pub support_basis_evidence_content_digest: String,
    pub symthaea_continuity_binding: String,
    pub symtropy_topology_binding: String,
    pub shared_topology_fixture_binding: String,
    pub source_model_binding: String,
    pub successor_model_binding: String,
    pub source_support_binding: String,
    pub successor_support_binding: String,
    pub source_role_support_dependency_refs: Vec<String>,
    pub successor_role_support_dependency_refs: Vec<String>,
    pub finite_root_pairs: Vec<RegenerativeSupportClosureRootPairEvidenceV1>,
    pub topology_isomorphic: bool,
    pub scalar_runway_projection_safe: bool,
    /// Required when the upstream continuity theorem refuses scalar projection.
    pub rejection_binding: Option<String>,
    /// Optional exact policy surface authorized by a safe closure record.
    pub authorized_policy_surface_content_digest: Option<String>,
}

impl RegenerativeSupportClosureContinuityEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_SUPPORT_CLOSURE_CONTINUITY_SCHEMA_V1 {
            return Err(format!(
                "unsupported support-closure continuity schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.continuity_evidence_id) {
            return Err("support-closure continuity ID is not canonical".into());
        }
        for digest in [
            &self.viability_evidence_content_digest,
            &self.support_basis_evidence_content_digest,
        ] {
            if !lower_hex_64(digest) {
                return Err("support-closure parent digest must be lowercase 64-hex".into());
            }
        }
        for binding in [
            &self.symthaea_continuity_binding,
            &self.symtropy_topology_binding,
            &self.shared_topology_fixture_binding,
            &self.source_model_binding,
            &self.successor_model_binding,
            &self.source_support_binding,
            &self.successor_support_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("support-closure binding is not canonical".into());
            }
        }
        validate_refs(
            "source_role_support_dependency_refs",
            &self.source_role_support_dependency_refs,
        )?;
        validate_refs(
            "successor_role_support_dependency_refs",
            &self.successor_role_support_dependency_refs,
        )?;
        if self.finite_root_pairs.is_empty() || self.finite_root_pairs.len() > MAX_ITEMS {
            return Err("support-closure finite root pair cardinality is invalid".into());
        }
        for pair in &self.finite_root_pairs {
            if !canonical_id(&pair.source_dependency_id)
                || !canonical_id(&pair.successor_dependency_id)
            {
                return Err("support-closure root pair ID is not canonical".into());
            }
        }
        if self.finite_root_pairs.windows(2).any(|pair| {
            (
                pair[0].source_dependency_id.as_str(),
                pair[0].successor_dependency_id.as_str(),
            ) >= (
                pair[1].source_dependency_id.as_str(),
                pair[1].successor_dependency_id.as_str(),
            )
        }) {
            return Err("support-closure root pairs must be strictly sorted and unique".into());
        }

        let roots_safe = self
            .finite_root_pairs
            .iter()
            .all(|pair| pair.transfer_qualified && pair.basis_qualified);
        let expected_safe = self.topology_isomorphic && roots_safe;
        if self.scalar_runway_projection_safe != expected_safe {
            return Err(
                "support-closure scalar safety disagrees with topology/root evidence".into(),
            );
        }
        if self.scalar_runway_projection_safe {
            if self.rejection_binding.is_some() {
                return Err("safe support closure cannot carry a rejection binding".into());
            }
        } else {
            let rejection = self
                .rejection_binding
                .as_ref()
                .ok_or_else(|| "unsafe support closure requires a rejection binding".to_string())?;
            if !canonical_reference(rejection) {
                return Err("support-closure rejection binding is not canonical".into());
            }
            if self.authorized_policy_surface_content_digest.is_some() {
                return Err("unsafe support closure cannot authorize a policy surface".into());
            }
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
            .map_err(|error| format!("failed to serialize support-closure evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-support-closure-continuity-v1\0");
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

pub fn verify_regenerative_support_closure_continuity_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
) -> Result<(), String> {
    viability.validate()?;
    basis.validate()?;
    continuity.validate()?;
    if continuity.viability_evidence_content_digest != viability.content_digest()? {
        return Err("support-closure viability digest mismatch".into());
    }
    if continuity.support_basis_evidence_content_digest != basis.content_digest()? {
        return Err("support-closure basis digest mismatch".into());
    }
    if basis.viability_evidence_content_digest != continuity.viability_evidence_content_digest {
        return Err("support-closure and basis refer to different viability subjects".into());
    }
    if continuity.source_model_binding != basis.source_model_binding
        || continuity.successor_model_binding != basis.successor_model_binding
    {
        return Err("support-closure model bindings disagree with basis evidence".into());
    }
    if continuity.source_model_binding != viability.closure_model_binding {
        return Err("support-closure source model disagrees with viability evidence".into());
    }
    if continuity.source_support_binding != viability.flow_support_binding {
        return Err("support-closure source support graph disagrees with viability evidence".into());
    }

    for root in &continuity.finite_root_pairs {
        if root.basis_qualified {
            let matching = basis.assessments.iter().find(|assessment| {
                assessment.source_dependency_id == root.source_dependency_id
                    && assessment.successor_dependency_id == root.successor_dependency_id
            });
            let assessment = matching.ok_or_else(|| {
                "support-closure marks a root basis-qualified but parent basis lacks the pair"
                    .to_string()
            })?;
            let rate_preserved = assessment.normalized_stockpile_draw_rate_preserved;
            let period_preserved =
                assessment.source_period_duration_ms == assessment.successor_period_duration_ms;
            if !(rate_preserved && period_preserved) {
                return Err(
                    "support-closure root claims basis qualification from an unsafe parent pair"
                        .into(),
                );
            }
        }
    }
    Ok(())
}

pub fn verify_regenerative_support_closure_surface_authorization(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
    surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> Result<(), String> {
    // Full continuity composition is mandatory here; callers cannot skip the
    // source model/support identity checks and jump straight to surface authorization.
    verify_regenerative_support_closure_continuity_evidence(viability, basis, continuity)?;
    verify_regenerative_support_basis_surface_authorization(basis, surface)?;
    if !continuity.scalar_runway_projection_safe {
        return Err("unsafe support closure cannot authorize a policy surface".into());
    }
    let expected = continuity
        .authorized_policy_surface_content_digest
        .as_ref()
        .ok_or_else(|| "support-closure record does not bind an authorized surface".to_string())?;
    if expected != &surface.content_digest()? {
        return Err("support-closure authorized surface digest mismatch".into());
    }
    if continuity.viability_evidence_content_digest != surface.viability_evidence_content_digest {
        return Err("support-closure and surface refer to different viability subjects".into());
    }
    Ok(())
}

fn validate_refs(field: &str, refs: &[String]) -> Result<(), String> {
    if refs.is_empty() || refs.len() > MAX_ITEMS {
        return Err(format!("{field} cardinality is invalid"));
    }
    for value in refs {
        if !canonical_reference(value) {
            return Err(format!("{field} contains a non-canonical reference"));
        }
    }
    if refs.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(format!("{field} must be strictly sorted and unique"));
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
