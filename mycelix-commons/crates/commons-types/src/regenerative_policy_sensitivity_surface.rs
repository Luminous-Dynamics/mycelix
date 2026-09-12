// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for regenerative reproduction-policy sensitivity surfaces.
//!
//! Mycelix preserves the fixed physical coordinate plus policy-indexed descendant
//! outcomes established upstream. It validates provenance composition, canonical
//! policy ordering, conditioning, arithmetic feasibility, and monotonicity, but
//! does not recompute Symthaea projection math or execute Symtropy industrial dynamics.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind, RegenerativeGenerationCountEvidenceV1,
    RegenerativeViabilityEvidenceV1, RegenerativeViabilityHorizonV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_POLICY_POINTS: usize = 256;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativePolicySensitivityPointEvidenceV1 {
    pub reproduction_policy_id: String,
    pub reproduction_policy_evidence_binding: String,
    pub maturity_periods: u64,
    pub founded_descendant_generations: RegenerativeGenerationCountEvidenceV1,
    pub maturity_completed_descendant_generations: RegenerativeGenerationCountEvidenceV1,
    pub descendant_reproduction_transitions: RegenerativeGenerationCountEvidenceV1,
    pub terminal_generation_residual_periods: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativePolicySensitivitySurfaceEvidenceV1 {
    pub schema_version: u8,
    pub surface_id: String,
    /// Exact Mycelix viability record that owns the invariant physical coordinate.
    pub viability_evidence_content_digest: String,
    /// Exact engineering subjects establishing semantic and dynamic surfaces.
    pub symthaea_surface_binding: String,
    pub symtropy_surface_binding: String,
    /// Shared byte-identical fixture/evidence contract.
    pub shared_surface_fixture_binding: String,
    pub physical_successor_reproduction_horizon: RegenerativeViabilityHorizonV1,
    pub physical_regenerative_viability_horizon: RegenerativeViabilityHorizonV1,
    pub fully_modeled_successor_reproduction: bool,
    pub fully_modeled_regenerative_viability: bool,
    /// Strictly increasing maturity policy family.
    pub policy_points: Vec<RegenerativePolicySensitivityPointEvidenceV1>,
}

impl RegenerativePolicySensitivitySurfaceEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_POLICY_SENSITIVITY_SURFACE_SCHEMA_V1 {
            return Err(format!(
                "unsupported policy sensitivity surface schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.surface_id) {
            return Err("policy sensitivity surface_id is not canonical".into());
        }
        if !lower_hex_64(&self.viability_evidence_content_digest) {
            return Err("viability_evidence_content_digest must be lowercase 64-hex".into());
        }
        for binding in [
            &self.symthaea_surface_binding,
            &self.symtropy_surface_binding,
            &self.shared_surface_fixture_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("policy sensitivity surface binding is not canonical".into());
            }
        }
        if self.policy_points.len() < 2 {
            return Err("policy sensitivity surface requires at least two policy points".into());
        }
        if self.policy_points.len() > MAX_POLICY_POINTS {
            return Err(format!(
                "too many policy sensitivity points (max {MAX_POLICY_POINTS})"
            ));
        }

        let finite_physical_horizon = matches!(
            self.physical_successor_reproduction_horizon,
            RegenerativeViabilityHorizonV1::FinitePeriods(_)
        );
        let mut policy_ids = BTreeSet::new();
        let mut policy_bindings = BTreeSet::new();
        let mut previous_maturity = None;
        let mut previous_finite_counts: Option<(u64, u64, u64)> = None;

        for point in &self.policy_points {
            if !canonical_id(&point.reproduction_policy_id) {
                return Err("policy sensitivity point policy ID is not canonical".into());
            }
            if !canonical_reference(&point.reproduction_policy_evidence_binding) {
                return Err("policy sensitivity point evidence binding is not canonical".into());
            }
            if !policy_ids.insert(point.reproduction_policy_id.as_str()) {
                return Err("policy sensitivity surface contains duplicate policy IDs".into());
            }
            if !policy_bindings.insert(point.reproduction_policy_evidence_binding.as_str()) {
                return Err(
                    "policy sensitivity surface contains duplicate policy evidence bindings".into(),
                );
            }
            if point.maturity_periods == 0 {
                return Err("policy sensitivity maturity_periods must be positive".into());
            }
            if previous_maturity.is_some_and(|previous| previous >= point.maturity_periods) {
                return Err(
                    "policy sensitivity points must be strictly maturity-sorted".into(),
                );
            }
            previous_maturity = Some(point.maturity_periods);

            match (
                point.founded_descendant_generations,
                point.maturity_completed_descendant_generations,
                point.descendant_reproduction_transitions,
                point.terminal_generation_residual_periods,
            ) {
                (
                    RegenerativeGenerationCountEvidenceV1::Finite(founded),
                    RegenerativeGenerationCountEvidenceV1::Finite(matured),
                    RegenerativeGenerationCountEvidenceV1::Finite(transitions),
                    Some(residual),
                ) => {
                    if !finite_physical_horizon {
                        return Err(
                            "finite policy outcomes require a finite physical successor horizon"
                                .into(),
                        );
                    }
                    if matured > founded || transitions > matured {
                        return Err(
                            "finite policy sensitivity counts violate ordering constraints".into(),
                        );
                    }
                    if founded == 0 {
                        if matured != 0 || transitions != 0 || residual != 0 {
                            return Err(
                                "zero founded generations require zero terminal metrics".into(),
                            );
                        }
                    } else if residual == 0 {
                        return Err(
                            "positive founded generations require positive terminal residual".into(),
                        );
                    }
                    if let Some((previous_founded, previous_matured, previous_transitions)) =
                        previous_finite_counts
                    {
                        if founded > previous_founded
                            || matured > previous_matured
                            || transitions > previous_transitions
                        {
                            return Err(
                                "stricter maturity policy cannot improve finite descendant outcomes"
                                    .into(),
                            );
                        }
                    }
                    previous_finite_counts = Some((founded, matured, transitions));
                }
                (
                    RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                    RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                    RegenerativeGenerationCountEvidenceV1::IndefiniteUnderStaticModel,
                    None,
                ) => {
                    if finite_physical_horizon {
                        return Err(
                            "indefinite policy outcomes require an indefinite physical successor horizon"
                                .into(),
                        );
                    }
                    previous_finite_counts = None;
                }
                _ => {
                    return Err(
                        "policy sensitivity generation-count conditioning is inconsistent".into(),
                    )
                }
            }
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize policy sensitivity surface: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-policy-sensitivity-surface-v1\0");
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

pub fn verify_regenerative_policy_sensitivity_surface_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    surface: &RegenerativePolicySensitivitySurfaceEvidenceV1,
) -> Result<(), String> {
    viability.validate()?;
    surface.validate()?;
    if surface.viability_evidence_content_digest != viability.content_digest()? {
        return Err("policy sensitivity viability-evidence digest mismatch".into());
    }
    if surface.physical_regenerative_viability_horizon
        != viability.regenerative_viability_horizon
    {
        return Err("policy sensitivity overall physical horizon disagrees with viability evidence".into());
    }
    if surface.fully_modeled_regenerative_viability
        != viability.fully_modeled_regenerative_viability
    {
        return Err("policy sensitivity overall modeling flag disagrees with viability evidence".into());
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
    let expected_reproduction_horizon =
        min_horizon(construction.static_horizon, qualification.static_horizon);
    if surface.physical_successor_reproduction_horizon != expected_reproduction_horizon {
        return Err(
            "policy sensitivity successor-reproduction horizon disagrees with viability roles".into(),
        );
    }
    let expected_fully_modeled =
        construction.fully_modeled_support && qualification.fully_modeled_support;
    if surface.fully_modeled_successor_reproduction != expected_fully_modeled {
        return Err(
            "policy sensitivity successor-reproduction modeling flag disagrees with viability roles"
                .into(),
        );
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
