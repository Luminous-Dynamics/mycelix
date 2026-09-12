// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Provenance for controlled regenerative-lineage viability frontiers.
//!
//! This record preserves exact cross-repo frontier identities and summary counts.
//! Mycelix validates canonical shape and arithmetic consistency; it does not rerun
//! Symtropy dynamics or recompute Symthaea semantic reserve qualification.

use crate::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_REFS: usize = 64;

/// Strict provenance summary for one controlled viability-frontier campaign.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeViabilityFrontierEvidenceV1 {
    pub schema_version: u8,
    pub frontier_id: String,
    /// Exact content identity of the shared cross-repo frontier fixture.
    pub fixture_content_binding: String,
    /// Exact dynamic frontier subject/run identity.
    pub symtropy_frontier_binding: String,
    /// Exact semantic frontier interpretation/policy identity.
    pub symthaea_frontier_binding: String,
    /// Opaque identity of the reserve policy semantics applied to this frontier.
    pub bootstrap_policy_binding: String,
    /// Controlled dimensions varied by the campaign, sorted and unique.
    pub varied_dimension_refs: Vec<String>,
    /// Fixed scenario/context references, sorted and unique.
    pub fixed_context_refs: Vec<String>,
    pub total_cases: u32,
    /// Cases where construction and qualification overlap dynamically.
    pub role_overlap_cases: u32,
    /// Subset of role-overlap cases that also satisfy bootstrap-preserving
    /// reproductive reserve and handoff evidence.
    pub reproduction_ready_cases: u32,
    /// Role-overlap cases rejected by the stricter bootstrap requirement.
    pub role_overlap_without_bootstrap_cases: u32,
    /// Cases with no construction+qualification role-overlap window at all.
    pub no_role_overlap_cases: u32,
}

impl RegenerativeViabilityFrontierEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative viability frontier schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.frontier_id) {
            return Err("frontier_id is not canonical".into());
        }
        for (field, binding) in [
            ("fixture_content_binding", self.fixture_content_binding.as_str()),
            ("symtropy_frontier_binding", self.symtropy_frontier_binding.as_str()),
            ("symthaea_frontier_binding", self.symthaea_frontier_binding.as_str()),
            ("bootstrap_policy_binding", self.bootstrap_policy_binding.as_str()),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        validate_refs("varied_dimension_refs", &self.varied_dimension_refs)?;
        validate_refs("fixed_context_refs", &self.fixed_context_refs)?;
        if self.varied_dimension_refs.is_empty() {
            return Err("frontier requires at least one varied dimension".into());
        }
        if self.total_cases == 0 {
            return Err("frontier requires at least one case".into());
        }

        let overlap_partition = self
            .reproduction_ready_cases
            .checked_add(self.role_overlap_without_bootstrap_cases)
            .ok_or_else(|| "frontier role-overlap count overflow".to_string())?;
        if overlap_partition != self.role_overlap_cases {
            return Err(
                "role_overlap_cases must equal reproduction_ready + overlap_without_bootstrap"
                    .into(),
            );
        }
        let total_partition = self
            .role_overlap_cases
            .checked_add(self.no_role_overlap_cases)
            .ok_or_else(|| "frontier total case count overflow".to_string())?;
        if total_partition != self.total_cases {
            return Err("total_cases must equal role_overlap + no_role_overlap".into());
        }
        if self.reproduction_ready_cases > self.role_overlap_cases {
            return Err("reproduction-ready cases cannot exceed role-overlap cases".into());
        }
        Ok(())
    }

    /// True when the evidenced bootstrap-preserving frontier is strictly narrower
    /// than the dynamic role-overlap frontier.
    pub fn bootstrap_frontier_is_strictly_narrower(&self) -> bool {
        self.reproduction_ready_cases < self.role_overlap_cases
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize viability frontier evidence: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-viability-frontier-v1\0");
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

fn validate_refs(field: &str, refs: &[String]) -> Result<(), String> {
    if refs.len() > MAX_REFS {
        return Err(format!("{field} contains too many references (max {MAX_REFS})"));
    }
    for reference in refs {
        if !canonical_reference(reference) {
            return Err(format!("{field} contains a non-canonical reference"));
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

fn canonical_reference(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_BINDING_BYTES
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> RegenerativeViabilityFrontierEvidenceV1 {
        RegenerativeViabilityFrontierEvidenceV1 {
            schema_version: REGENERATIVE_VIABILITY_FRONTIER_SCHEMA_V1,
            frontier_id: "manta-forge-viability-frontier-v1".into(),
            fixture_content_binding:
                "git-blob:a6348dd1d94de454114e3de4bad6086d080c4157".into(),
            symtropy_frontier_binding:
                "symtropy-pr:779:c3c7f09853075457f430f61eba7dee0a603f29bf".into(),
            symthaea_frontier_binding:
                "symthaea-pr:1873:552830efa215e49dca3cb341c91b1e6fa062ec6c".into(),
            bootstrap_policy_binding: "bootstrap-policy:manta-forge-frontier-v1".into(),
            varied_dimension_refs: vec![
                "dimension:forge-tooling-stock".into(),
                "dimension:metrology-recovery-tick".into(),
            ],
            fixed_context_refs: vec![
                "fixed:metrology-initial-stock:1".into(),
                "fixed:reactor-service-stock:8".into(),
                "fixed:structural-stock:18".into(),
            ],
            total_cases: 42,
            role_overlap_cases: 25,
            reproduction_ready_cases: 18,
            role_overlap_without_bootstrap_cases: 7,
            no_role_overlap_cases: 17,
        }
    }

    #[test]
    fn frontier_round_trips_and_preserves_stricter_bootstrap_relation() {
        let evidence = fixture();
        evidence.validate().unwrap();
        assert!(evidence.bootstrap_frontier_is_strictly_narrower());
        let payload = evidence.to_payload_json().unwrap();
        let decoded: RegenerativeViabilityFrontierEvidenceV1 =
            serde_json::from_str(&payload).unwrap();
        assert_eq!(decoded, evidence);
        assert_eq!(evidence.content_digest().unwrap().len(), 64);
    }

    #[test]
    fn impossible_count_partitions_fail_closed() {
        let mut evidence = fixture();
        evidence.role_overlap_without_bootstrap_cases = 8;
        assert!(evidence.validate().is_err());

        let mut evidence = fixture();
        evidence.no_role_overlap_cases = 16;
        assert!(evidence.validate().is_err());
    }

    #[test]
    fn frontier_fits_existing_maritime_transport() {
        let envelope = fixture()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                7,
                0,
                1_788_900_004_000_000,
                "evidence:regenerative-viability-frontier-event-01",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    }
}
