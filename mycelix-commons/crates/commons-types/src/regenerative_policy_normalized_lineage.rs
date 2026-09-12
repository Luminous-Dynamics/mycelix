// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for policy-normalized regenerative-lineage profiles.
//!
//! The bound Symthaea profile keeps physical support-qualified runway separate from
//! policy-dependent descendant-generation semantics. Mycelix verifies agreement
//! with existing viability provenance and preserves the policy projection as an
//! upstream evidence claim; it does not recompute physical closure or policy math.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativeViabilityEvidenceV1,
    RegenerativeViabilityHorizonV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_POLICY_NORMALIZED_LINEAGE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeGenerationCountEvidenceV1 {
    Finite(u64),
    IndefiniteUnderStaticModel,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativePolicyNormalizedLineageEvidenceV1 {
    pub schema_version: u8,
    pub normalized_profile_id: String,
    /// Raw content digest of the exact Mycelix viability record whose physical
    /// support claims are carried by this normalized view.
    pub viability_evidence_content_digest: String,
    /// Exact upstream Symthaea normalized-profile subject.
    pub symthaea_normalized_profile_binding: String,
    pub reproduction_policy_id: String,
    pub reproduction_policy_evidence_binding: String,
    pub maturity_periods: u64,
    /// Physical/static quantities preserved independently of policy choice.
    pub physical_successor_reproduction_horizon: RegenerativeViabilityHorizonV1,
    pub physical_regenerative_viability_horizon: RegenerativeViabilityHorizonV1,
    pub fully_modeled_successor_reproduction: bool,
    pub fully_modeled_regenerative_viability: bool,
    /// Policy-dependent quantities preserved from the exact normalized profile.
    pub founded_descendant_generations: RegenerativeGenerationCountEvidenceV1,
    pub maturity_completed_descendant_generations: RegenerativeGenerationCountEvidenceV1,
    pub descendant_reproduction_transitions: RegenerativeGenerationCountEvidenceV1,
    pub terminal_generation_residual_periods: Option<u64>,
}

impl RegenerativePolicyNormalizedLineageEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_POLICY_NORMALIZED_LINEAGE_SCHEMA_V1 {
            return Err(format!(
                "unsupported policy-normalized lineage schema version {}",
                self.schema_version
            ));
        }
        for value in [&self.normalized_profile_id, &self.reproduction_policy_id] {
            if !canonical_id(value) {
                return Err("policy-normalized lineage identifier is not canonical".into());
            }
        }
        if !lower_hex_64(&self.viability_evidence_content_digest) {
            return Err("viability_evidence_content_digest must be lowercase 64-hex".into());
        }
        for binding in [
            &self.symthaea_normalized_profile_binding,
            &self.reproduction_policy_evidence_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("policy-normalized lineage binding is not canonical".into());
            }
        }
        if self.maturity_periods == 0 {
            return Err("policy-normalized lineage maturity_periods must be positive".into());
        }

        match (
            self.founded_descendant_generations,
            self.maturity_completed_descendant_generations,
            self.descendant_reproduction_transitions,
            self.terminal_generation_residual_periods,
        ) {
            (
                RegenerativeGenerationCountEvidenceV1::Finite(founded),
                RegenerativeGenerationCountEvidenceV1::Finite(matured),
                RegenerativeGenerationCountEvidenceV1::Finite(transitions),
                Some(residual),
            ) => {
                if matured > founded || transitions > matured || transitions > founded {
                    return Err("finite generation counts violate ordering constraints".into());
                }
                if founded == 0 {
                    if matured != 0 || transitions != 0 || residual != 0 {
                        return Err("zero founded generations require zero terminal metrics".into());
                    }
                } else if residual == 0 {
                    return Err("positive founded generation count requires positive terminal residual".into());
                }
            }
            (
                RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                None,
            ) => {}
            _ => {
                return Err(
                    "generation-count conditioning and terminal residual must be internally consistent"
                        .into(),
                )
            }
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize normalized lineage evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-policy-normalized-lineage-v1\0");
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

pub fn verify_regenerative_policy_normalized_lineage_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    normalized: &RegenerativePolicyNormalizedLineageEvidenceV1,
) -> Result<(), String> {
    viability.validate()?;
    normalized.validate()?;
    if normalized.viability_evidence_content_digest != viability.content_digest()? {
        return Err("normalized lineage viability-evidence digest mismatch".into());
    }
    if normalized.physical_regenerative_viability_horizon
        != viability.regenerative_viability_horizon
    {
        return Err("normalized physical regenerative horizon disagrees with viability evidence".into());
    }
    if normalized.fully_modeled_regenerative_viability
        != viability.fully_modeled_regenerative_viability
    {
        return Err("normalized overall modeling flag disagrees with viability evidence".into());
    }

    let construction = viability
        .roles
        .iter()
        .find(|role| role.role_id == "successor_construction")
        .ok_or_else(|| "viability evidence lacks successor_construction role".to_string())?;
    let qualification = viability
        .roles
        .iter()
        .find(|role| role.role_id == "successor_qualification")
        .ok_or_else(|| "viability evidence lacks successor_qualification role".to_string())?;
    let expected_reproduction_horizon = min_horizon(
        construction.static_horizon,
        qualification.static_horizon,
    );
    if normalized.physical_successor_reproduction_horizon != expected_reproduction_horizon {
        return Err("normalized successor-reproduction horizon disagrees with viability roles".into());
    }
    let expected_fully_modeled = construction.fully_modeled_support && qualification.fully_modeled_support;
    if normalized.fully_modeled_successor_reproduction != expected_fully_modeled {
        return Err("normalized successor-reproduction modeling flag disagrees with viability roles".into());
    }
    Ok(())
}

fn min_horizon(
    left: RegenerativeViabilityHorizonV1,
    right: RegenerativeViabilityHorizonV1,
) -> RegenerativeViabilityHorizonV1 {
    match (left, right) {
        (
            RegenerativeViabilityHorizonV1::FinitePeriods(left),
            RegenerativeViabilityHorizonV1::FinitePeriods(right),
        ) => RegenerativeViabilityHorizonV1::FinitePeriods(left.min(right)),
        (RegenerativeViabilityHorizonV1::FinitePeriods(value), _)
        | (_, RegenerativeViabilityHorizonV1::FinitePeriods(value)) => {
            RegenerativeViabilityHorizonV1::FinitePeriods(value)
        }
        _ => RegenerativeViabilityHorizonV1::IndefiniteUnderStaticModel,
    }
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
