// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Recipe-free provenance for regenerative-lineage viability evidence.
//!
//! This module preserves a compact, queryable summary of an upstream viability
//! assessment plus opaque bindings to the exact lineage, static report, and dynamic
//! simulation evidence. Mycelix does not recompute or certify Symthaea/Symtropy
//! engineering semantics; it validates canonical provenance shape and continuity.

use crate::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use serde::{Deserialize, Serialize};

/// Current regenerative-viability evidence schema version.
pub const REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_REFERENCE_BYTES: usize = 512;
const MAX_ROLES: usize = 64;
const MAX_REFERENCES_PER_ROLE: usize = 64;

/// Upstream-reported conservative support horizon.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeViabilityHorizonV1 {
    FinitePeriods(u64),
    IndefiniteUnderStaticModel,
}

/// Provenance summary for one named viability role.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeViabilityRoleEvidenceV1 {
    /// Canonical role label, for example `operation` or `successor_qualification`.
    pub role_id: String,
    /// Conservative horizon reported by the bound static evidence.
    pub static_horizon: RegenerativeViabilityHorizonV1,
    /// First completed simulation tick where the role was observed unavailable.
    /// `None` means no such dynamic observation is asserted by this record.
    pub dynamic_first_unavailable_tick: Option<u64>,
    /// Whether the upstream report says this role's support path is fully modeled.
    pub fully_modeled_support: bool,
    /// Opaque references to root limiting dependencies/evidence, sorted and unique.
    pub limiting_dependency_refs: Vec<String>,
}

impl RegenerativeViabilityRoleEvidenceV1 {
    fn validate(&self) -> Result<(), String> {
        if !canonical_id(&self.role_id) {
            return Err("role_id is not canonical".into());
        }
        validate_refs(
            "limiting_dependency_refs",
            &self.limiting_dependency_refs,
            MAX_REFERENCES_PER_ROLE,
        )?;
        if matches!(
            self.static_horizon,
            RegenerativeViabilityHorizonV1::FinitePeriods(_)
        ) && self.limiting_dependency_refs.is_empty()
        {
            return Err("finite role horizon requires limiting dependency evidence".into());
        }
        Ok(())
    }
}

/// Strict provenance record tying one Genome lineage to static and dynamic viability evidence.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeViabilityEvidenceV1 {
    pub schema_version: u8,
    /// Exact Genome manifest binding whose viability is being summarized.
    pub genome_binding: String,
    /// Binding/digest of the Mycelix genome-lineage provenance record.
    pub lineage_evidence_binding: String,
    /// Exact upstream role-profile evidence binding.
    pub viability_profile_binding: String,
    /// Exact static support-qualified viability report binding.
    pub static_viability_report_binding: String,
    /// Exact dynamic simulation/run evidence binding.
    pub dynamic_simulation_binding: String,
    /// Exact closure-model evidence binding used by the static assessment.
    pub closure_model_binding: String,
    /// Exact flow-support graph evidence binding used by the static assessment.
    pub flow_support_binding: String,
    /// Duration of one common static/dynamic period when the two evidence sources
    /// declare the same time basis. This field is evidence metadata, not a clock.
    pub period_duration_ms: u64,
    /// Role observations sorted strictly by `role_id`.
    pub roles: Vec<RegenerativeViabilityRoleEvidenceV1>,
    /// Upstream-reported conservative viability horizon for the whole lineage state.
    pub regenerative_viability_horizon: RegenerativeViabilityHorizonV1,
    /// Roles establishing the finite overall bound, sorted and unique.
    pub limiting_role_ids: Vec<String>,
    /// Whether the upstream result says all viability support paths are fully modeled.
    pub fully_modeled_regenerative_viability: bool,
}

impl RegenerativeViabilityEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative viability evidence schema version {}",
                self.schema_version
            ));
        }
        for (field, binding) in [
            ("genome_binding", self.genome_binding.as_str()),
            ("lineage_evidence_binding", self.lineage_evidence_binding.as_str()),
            ("viability_profile_binding", self.viability_profile_binding.as_str()),
            (
                "static_viability_report_binding",
                self.static_viability_report_binding.as_str(),
            ),
            (
                "dynamic_simulation_binding",
                self.dynamic_simulation_binding.as_str(),
            ),
            ("closure_model_binding", self.closure_model_binding.as_str()),
            ("flow_support_binding", self.flow_support_binding.as_str()),
        ] {
            if !canonical_reference(binding, MAX_BINDING_BYTES) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.period_duration_ms == 0 {
            return Err("period_duration_ms must be nonzero".into());
        }
        if self.roles.is_empty() {
            return Err("regenerative viability evidence requires at least one role".into());
        }
        if self.roles.len() > MAX_ROLES {
            return Err(format!("too many viability roles (max {MAX_ROLES})"));
        }
        for role in &self.roles {
            role.validate()?;
        }
        if self
            .roles
            .windows(2)
            .any(|pair| pair[0].role_id >= pair[1].role_id)
        {
            return Err("roles must be strictly sorted by role_id and duplicate-free".into());
        }

        if self.limiting_role_ids.len() > MAX_ROLES {
            return Err(format!("too many limiting roles (max {MAX_ROLES})"));
        }
        for role_id in &self.limiting_role_ids {
            if !canonical_id(role_id) {
                return Err("limiting_role_ids contains a non-canonical role".into());
            }
        }
        if self
            .limiting_role_ids
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
        {
            return Err("limiting_role_ids must be strictly sorted and duplicate-free".into());
        }
        for limiting_role_id in &self.limiting_role_ids {
            if !self.roles.iter().any(|role| &role.role_id == limiting_role_id) {
                return Err("limiting role is absent from role evidence".into());
            }
        }
        match self.regenerative_viability_horizon {
            RegenerativeViabilityHorizonV1::FinitePeriods(_) if self.limiting_role_ids.is_empty() => {
                return Err("finite regenerative viability horizon requires a limiting role".into());
            }
            RegenerativeViabilityHorizonV1::IndefiniteUnderStaticModel
                if !self.limiting_role_ids.is_empty() =>
            {
                return Err(
                    "indefinite static viability cannot name a finite limiting role".into(),
                );
            }
            _ => {}
        }
        Ok(())
    }

    /// Serialize the canonical V1 provenance payload.
    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize regenerative viability evidence: {error}"))
    }

    /// Stable content identity independent of maritime event sequence/transport.
    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-viability-evidence-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    /// Carry this evidence over the existing maritime store-forward path.
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

fn validate_refs(field: &str, refs: &[String], max_count: usize) -> Result<(), String> {
    if refs.len() > max_count {
        return Err(format!("{field} contains too many references (max {max_count})"));
    }
    for reference in refs {
        if !canonical_reference(reference, MAX_REFERENCE_BYTES) {
            return Err(format!("{field} contains a non-canonical opaque reference"));
        }
    }
    if refs.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(format!("{field} must be strictly sorted and duplicate-free"));
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

fn canonical_reference(value: &str, max_bytes: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_bytes
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> RegenerativeViabilityEvidenceV1 {
        RegenerativeViabilityEvidenceV1 {
            schema_version: REGENERATIVE_VIABILITY_EVIDENCE_SCHEMA_V1,
            genome_binding: "genome:manta-v2:sha256:example".into(),
            lineage_evidence_binding: "lineage:blake3:example".into(),
            viability_profile_binding: "viability-profile:manta-v2:v1".into(),
            static_viability_report_binding: "symthaea-report:manta-v2:sha256:example".into(),
            dynamic_simulation_binding: "symtropy-run:manta-v2:sha256:example".into(),
            closure_model_binding: "closure-model:manta-v2:sha256:example".into(),
            flow_support_binding: "flow-support:manta-v2:sha256:example".into(),
            period_duration_ms: 1,
            roles: vec![
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "operation".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(100),
                    dynamic_first_unavailable_tick: Some(101),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:platform-spares".into()],
                },
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "successor_construction".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(40),
                    dynamic_first_unavailable_tick: Some(41),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:forge-tooling".into()],
                },
                RegenerativeViabilityRoleEvidenceV1 {
                    role_id: "successor_qualification".into(),
                    static_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(30),
                    dynamic_first_unavailable_tick: Some(31),
                    fully_modeled_support: true,
                    limiting_dependency_refs: vec!["dependency:metrology".into()],
                },
            ],
            regenerative_viability_horizon: RegenerativeViabilityHorizonV1::FinitePeriods(30),
            limiting_role_ids: vec!["successor_qualification".into()],
            fully_modeled_regenerative_viability: true,
        }
    }

    #[test]
    fn viability_evidence_round_trips_strictly_and_has_stable_identity_shape() {
        let evidence = fixture();
        let json = evidence.to_payload_json().unwrap();
        let decoded: RegenerativeViabilityEvidenceV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, evidence);
        let digest = evidence.content_digest().unwrap();
        assert_eq!(digest.len(), 64);
        assert!(digest.bytes().all(|byte| byte.is_ascii_hexdigit()));
    }

    #[test]
    fn role_order_unknown_limiter_and_missing_finite_limiter_fail_closed() {
        let mut unsorted = fixture();
        unsorted.roles.swap(0, 2);
        assert!(unsorted.validate().is_err());

        let mut unknown_role = fixture();
        unknown_role.limiting_role_ids = vec!["unknown_role".into()];
        assert!(unknown_role.validate().is_err());

        let mut no_limiter = fixture();
        no_limiter.limiting_role_ids.clear();
        assert!(no_limiter.validate().is_err());
    }

    #[test]
    fn finite_role_horizon_requires_root_limiter_evidence() {
        let mut evidence = fixture();
        evidence.roles[0].limiting_dependency_refs.clear();
        assert!(evidence.validate().is_err());
    }

    #[test]
    fn opaque_external_conditioning_can_remain_non_numeric_uncertainty() {
        let mut evidence = fixture();
        evidence.roles[2].static_horizon =
            RegenerativeViabilityHorizonV1::IndefiniteUnderStaticModel;
        evidence.roles[2].dynamic_first_unavailable_tick = None;
        evidence.roles[2].fully_modeled_support = false;
        evidence.roles[2].limiting_dependency_refs.clear();
        evidence.regenerative_viability_horizon = RegenerativeViabilityHorizonV1::FinitePeriods(40);
        evidence.limiting_role_ids = vec!["successor_construction".into()];
        evidence.fully_modeled_regenerative_viability = false;
        assert_eq!(evidence.validate(), Ok(()));
    }

    #[test]
    fn viability_evidence_fits_existing_maritime_transport() {
        let envelope = fixture()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                5,
                0,
                1_788_900_002_000_000,
                "evidence:regenerative-viability-event-01",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
        let decoded: RegenerativeViabilityEvidenceV1 =
            serde_json::from_str(&envelope.payload_json).unwrap();
        assert_eq!(decoded, fixture());
    }

    #[test]
    fn payload_contains_no_recipe_control_or_authority_fields() {
        let json = fixture().to_payload_json().unwrap();
        for forbidden in [
            "toolpath",
            "temperature_setpoint",
            "pressure_setpoint",
            "actuator_command",
            "enrichment",
            "fuel_fabrication",
            "reactor_control",
            "authority_override",
        ] {
            assert!(!json.contains(forbidden));
        }
    }
}
