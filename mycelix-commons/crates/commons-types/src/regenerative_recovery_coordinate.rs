// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: Apache-2.0 OR MIT
//! Provenance for disturbance-conditioned regenerative recovery.
//!
//! Symthaea owns semantic recovery qualification and Symtropy owns deterministic
//! execution. Mycelix preserves those exact subjects, composes them with the
//! nominal support-closure theorem, and verifies only structural/arithmetic
//! consistency. It does not infer physical repair feasibility or grant authority.

use crate::{
    verify_regenerative_support_closure_continuity_evidence, MaritimeEvidenceEnvelope,
    MaritimeEvidenceKind, RegenerativeSupportBasisEvidenceV1,
    RegenerativeSupportClosureContinuityEvidenceV1, RegenerativeViabilityEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1: u8 = 1;
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeRecoveryFlowKindEvidenceV1 {
    Production,
    Recycling,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryCoordinateEvidenceV1 {
    pub schema_version: u8,
    pub recovery_evidence_id: String,
    pub viability_evidence_content_digest: String,
    pub support_closure_continuity_content_digest: String,
    pub symthaea_recovery_binding: String,
    pub symtropy_recovery_binding: String,
    pub semantic_recovery_fixture_binding: String,
    pub dynamic_recovery_fixture_binding: String,
    pub successor_model_binding: String,
    pub successor_support_binding: String,
    pub disturbance_id: String,
    pub disturbance_evidence_binding: String,
    pub target_dependency_id: String,
    pub flow_kind: RegenerativeRecoveryFlowKindEvidenceV1,
    pub healthy_units_per_period: u64,
    pub degraded_units_per_period: u64,
    pub reserve_dependency_id: String,
    pub reserve_units_per_recovery: u64,
    pub reserve_units_before: u64,
    pub reserve_units_after: u64,
    pub dynamic_recovery_receipt_binding: String,
    pub recovery_qualified: bool,
    pub disturbance_conditioned_recovery_authorized: bool,
    pub observed_maturity_periods: u64,
    pub unrecoverable_first_unavailable_period: u64,
    pub recoverable_maturity_completed: bool,
}

impl RegenerativeRecoveryCoordinateEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_RECOVERY_COORDINATE_EVIDENCE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative recovery-coordinate schema version {}",
                self.schema_version
            ));
        }
        for id in [
            &self.recovery_evidence_id,
            &self.disturbance_id,
            &self.target_dependency_id,
            &self.reserve_dependency_id,
        ] {
            if !canonical_id(id) {
                return Err("recovery-coordinate identifier is not canonical".into());
            }
        }
        for digest in [
            &self.viability_evidence_content_digest,
            &self.support_closure_continuity_content_digest,
        ] {
            if !lower_hex_64(digest) {
                return Err("recovery-coordinate parent digest must be lowercase 64-hex".into());
            }
        }
        for binding in [
            &self.symthaea_recovery_binding,
            &self.symtropy_recovery_binding,
            &self.semantic_recovery_fixture_binding,
            &self.dynamic_recovery_fixture_binding,
            &self.successor_model_binding,
            &self.successor_support_binding,
            &self.disturbance_evidence_binding,
            &self.dynamic_recovery_receipt_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("recovery-coordinate binding is not canonical".into());
            }
        }
        if self.target_dependency_id == self.reserve_dependency_id {
            return Err("recovery target and reserve must be distinct".into());
        }
        if self.healthy_units_per_period == 0 {
            return Err("healthy recovery flow must be positive".into());
        }
        if self.degraded_units_per_period >= self.healthy_units_per_period {
            return Err("disturbance must strictly degrade the qualified flow".into());
        }
        if self.reserve_units_per_recovery == 0 {
            return Err("recovery reserve cost must be positive".into());
        }
        if self.reserve_units_before < self.reserve_units_per_recovery {
            return Err("recovery reserve is insufficient for the claimed receipt".into());
        }
        let expected_after = self
            .reserve_units_before
            .checked_sub(self.reserve_units_per_recovery)
            .ok_or_else(|| "recovery reserve arithmetic underflow".to_string())?;
        if self.reserve_units_after != expected_after {
            return Err("recovery reserve receipt arithmetic mismatch".into());
        }
        if !self.recovery_qualified && self.disturbance_conditioned_recovery_authorized {
            return Err("unqualified recovery cannot authorize a disturbance claim".into());
        }
        if self.observed_maturity_periods == 0 {
            return Err("recovery observation window must be positive".into());
        }
        if self.unrecoverable_first_unavailable_period == 0
            || self.unrecoverable_first_unavailable_period > self.observed_maturity_periods
        {
            return Err("unrecoverable failure period is outside the observation window".into());
        }
        if self.recoverable_maturity_completed
            && !self.disturbance_conditioned_recovery_authorized
        {
            return Err("completed recovery maturity claim requires recovery authorization".into());
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize recovery-coordinate evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-recovery-coordinate-evidence-v1\0");
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

/// Compose one recovery-coordinate record with the exact nominal continuity
/// subject it extends.
pub fn verify_regenerative_recovery_coordinate_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
    recovery: &RegenerativeRecoveryCoordinateEvidenceV1,
) -> Result<(), String> {
    verify_regenerative_support_closure_continuity_evidence(viability, basis, continuity)?;
    recovery.validate()?;
    if !continuity.scalar_runway_projection_safe {
        return Err("recovery coordinate requires a safe nominal support-closure subject".into());
    }
    if recovery.viability_evidence_content_digest != viability.content_digest()? {
        return Err("recovery-coordinate viability digest mismatch".into());
    }
    if recovery.support_closure_continuity_content_digest != continuity.content_digest()? {
        return Err("recovery-coordinate support-closure digest mismatch".into());
    }
    if recovery.successor_model_binding != continuity.successor_model_binding {
        return Err("recovery-coordinate successor model mismatch".into());
    }
    if recovery.successor_support_binding != continuity.successor_support_binding {
        return Err("recovery-coordinate successor support mismatch".into());
    }

    let target_ref = format!("dependency:{}", recovery.target_dependency_id);
    if !continuity
        .successor_role_support_dependency_refs
        .iter()
        .any(|reference| reference == &target_ref)
    {
        return Err("recovery target is outside the nominal role-support closure".into());
    }
    let reserve_ref = format!("dependency:{}", recovery.reserve_dependency_id);
    if continuity
        .successor_role_support_dependency_refs
        .iter()
        .any(|reference| reference == &reserve_ref)
    {
        return Err("recovery reserve is inside the nominal role-support closure".into());
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
