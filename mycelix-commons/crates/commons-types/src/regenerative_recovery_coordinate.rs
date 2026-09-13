// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: Apache-2.0 OR MIT
//! Provenance for disturbance-conditioned regenerative recovery.
//!
//! Symthaea owns semantic recovery qualification and Symtropy owns deterministic
//! execution. Mycelix preserves those exact subjects, composes them with the
//! nominal support-closure theorem, and verifies structural/arithmetic consistency.
//! Recovery reserve state is explicitly external to the nominal closure subject.
//!
//! Reserve spends are provenance-linear: sequence zero is the unique first spend
//! for one reserve subject, while every later spend must name the exact predecessor
//! evidence digest and continue its quantity/sequence state.

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
    pub successor_profile_id: String,
    pub successor_profile_evidence_binding: String,
    pub successor_model_binding: String,
    pub successor_support_binding: String,
    pub recovery_policy_id: String,
    pub recovery_policy_evidence_binding: String,
    pub recovery_qualification_binding: String,
    pub disturbance_id: String,
    pub disturbance_evidence_binding: String,
    pub dynamic_disturbance_observation_binding: String,
    pub target_dependency_id: String,
    pub flow_kind: RegenerativeRecoveryFlowKindEvidenceV1,
    pub healthy_units_per_period: u64,
    pub degraded_units_per_period: u64,
    pub external_recovery_reserve_id: String,
    pub external_recovery_reserve_binding: String,
    pub external_recovery_reserve_initial_units: u64,
    pub external_recovery_reserve_units_at_qualification: u64,
    pub reserve_units_per_recovery: u64,
    pub reserve_units_before: u64,
    pub reserve_units_after: u64,
    pub reserve_spend_sequence_before: u64,
    pub reserve_spend_sequence_after: u64,
    pub previous_recovery_evidence_content_digest: Option<String>,
    pub reserve_external_to_nominal_closure: bool,
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
            &self.successor_profile_id,
            &self.recovery_policy_id,
            &self.disturbance_id,
            &self.target_dependency_id,
            &self.external_recovery_reserve_id,
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
        if let Some(previous) = &self.previous_recovery_evidence_content_digest {
            if !lower_hex_64(previous) {
                return Err("previous recovery digest must be lowercase 64-hex".into());
            }
        }
        for binding in [
            &self.symthaea_recovery_binding,
            &self.symtropy_recovery_binding,
            &self.semantic_recovery_fixture_binding,
            &self.dynamic_recovery_fixture_binding,
            &self.successor_profile_evidence_binding,
            &self.successor_model_binding,
            &self.successor_support_binding,
            &self.recovery_policy_evidence_binding,
            &self.recovery_qualification_binding,
            &self.disturbance_evidence_binding,
            &self.dynamic_disturbance_observation_binding,
            &self.external_recovery_reserve_binding,
            &self.dynamic_recovery_receipt_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("recovery-coordinate binding is not canonical".into());
            }
        }
        if self.target_dependency_id == self.external_recovery_reserve_id {
            return Err("recovery target and external reserve IDs must be distinct".into());
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
        if self.external_recovery_reserve_initial_units
            < self.external_recovery_reserve_units_at_qualification
        {
            return Err("recovery qualification quantity exceeds reserve genesis".into());
        }
        if self.external_recovery_reserve_units_at_qualification < self.reserve_units_per_recovery {
            return Err("external recovery reserve was insufficient at qualification".into());
        }
        if self.reserve_units_before > self.external_recovery_reserve_initial_units {
            return Err("recovery receipt exceeds reserve genesis quantity".into());
        }
        let expected_after = self
            .reserve_units_before
            .checked_sub(self.reserve_units_per_recovery)
            .ok_or_else(|| "recovery reserve arithmetic underflow".to_string())?;
        if self.reserve_units_after != expected_after {
            return Err("recovery reserve receipt arithmetic mismatch".into());
        }
        let expected_sequence_after = self
            .reserve_spend_sequence_before
            .checked_add(1)
            .ok_or_else(|| "recovery reserve spend sequence overflow".to_string())?;
        if self.reserve_spend_sequence_after != expected_sequence_after {
            return Err("recovery reserve spend sequence is not monotonic".into());
        }
        if self.reserve_spend_sequence_before == 0 {
            if self.previous_recovery_evidence_content_digest.is_some() {
                return Err("first reserve spend cannot name a predecessor".into());
            }
            if self.reserve_units_before != self.external_recovery_reserve_units_at_qualification {
                return Err("first reserve spend must start from the qualification snapshot".into());
            }
        } else if self.previous_recovery_evidence_content_digest.is_none() {
            return Err("later reserve spend requires exact predecessor evidence".into());
        }
        if !self.reserve_external_to_nominal_closure {
            return Err("recovery reserve must remain external to the nominal closure".into());
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

pub fn verify_regenerative_recovery_reserve_lineage(
    previous: Option<&RegenerativeRecoveryCoordinateEvidenceV1>,
    current: &RegenerativeRecoveryCoordinateEvidenceV1,
) -> Result<(), String> {
    current.validate()?;
    match (previous, current.reserve_spend_sequence_before) {
        (None, 0) => {
            if current.previous_recovery_evidence_content_digest.is_some() {
                return Err("first recovery evidence cannot name a predecessor".into());
            }
        }
        (None, _) => return Err("later recovery evidence requires its predecessor record".into()),
        (Some(_), 0) => return Err("sequence-zero recovery evidence cannot follow a predecessor".into()),
        (Some(previous), _) => {
            previous.validate()?;
            let expected_digest = previous.content_digest()?;
            if current.previous_recovery_evidence_content_digest.as_deref()
                != Some(expected_digest.as_str())
            {
                return Err("recovery predecessor digest mismatch".into());
            }
            if current.external_recovery_reserve_id != previous.external_recovery_reserve_id
                || current.external_recovery_reserve_binding
                    != previous.external_recovery_reserve_binding
                || current.external_recovery_reserve_initial_units
                    != previous.external_recovery_reserve_initial_units
            {
                return Err("recovery reserve genesis subject changed across spend lineage".into());
            }
            if current.viability_evidence_content_digest
                != previous.viability_evidence_content_digest
                || current.support_closure_continuity_content_digest
                    != previous.support_closure_continuity_content_digest
            {
                return Err("recovery reserve lineage crossed nominal viability subjects".into());
            }
            if current.reserve_spend_sequence_before != previous.reserve_spend_sequence_after {
                return Err("recovery reserve spend sequence fork or gap detected".into());
            }
            if current.reserve_units_before != previous.reserve_units_after {
                return Err("recovery reserve quantity fork or reset detected".into());
            }
        }
    }
    Ok(())
}

pub fn verify_regenerative_recovery_coordinate_evidence(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
    recovery: &RegenerativeRecoveryCoordinateEvidenceV1,
) -> Result<(), String> {
    verify_regenerative_recovery_coordinate_evidence_with_predecessor(
        viability,
        basis,
        continuity,
        None,
        recovery,
    )
}

pub fn verify_regenerative_recovery_coordinate_evidence_with_predecessor(
    viability: &RegenerativeViabilityEvidenceV1,
    basis: &RegenerativeSupportBasisEvidenceV1,
    continuity: &RegenerativeSupportClosureContinuityEvidenceV1,
    previous_recovery: Option<&RegenerativeRecoveryCoordinateEvidenceV1>,
    recovery: &RegenerativeRecoveryCoordinateEvidenceV1,
) -> Result<(), String> {
    verify_regenerative_support_closure_continuity_evidence(viability, basis, continuity)?;
    verify_regenerative_recovery_reserve_lineage(previous_recovery, recovery)?;
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
    if !recovery.recovery_qualified || !recovery.disturbance_conditioned_recovery_authorized {
        return Err("recovery-coordinate record is not semantically authorized".into());
    }

    let target_ref = format!("dependency:{}", recovery.target_dependency_id);
    if !continuity
        .successor_role_support_dependency_refs
        .iter()
        .any(|reference| reference == &target_ref)
    {
        return Err("recovery target is outside the nominal role-support closure".into());
    }
    let reserve_ref = format!("dependency:{}", recovery.external_recovery_reserve_id);
    if continuity
        .successor_role_support_dependency_refs
        .iter()
        .any(|reference| reference == &reserve_ref)
    {
        return Err("external recovery reserve collides with nominal role-support closure".into());
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
