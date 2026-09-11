// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Recipe-free lineage provenance for regenerative design genomes.
//!
//! This module carries opaque evidence *about* a versioned successor design. It
//! does not embed BOM contents, manufacturing recipes, process setpoints, actuator
//! commands, or authority decisions. Outer Commons authorship and upstream
//! qualification/session evidence remain separate proofs.

use crate::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use serde::{Deserialize, Serialize};

/// Current regenerative-genome-lineage schema version.
pub const REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_REFERENCE_BYTES: usize = 512;
const MAX_REFERENCES_PER_CLASS: usize = 64;

/// Strict provenance record for one qualified regenerative-design genome.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeGenomeLineageEvidenceV1 {
    /// Exact schema version.
    pub schema_version: u8,
    /// Canonical genome identifier.
    pub genome_id: String,
    /// Opaque binding to the exact genome manifest.
    pub genome_binding: String,
    /// Optional parent genome binding. `None` marks a lineage root.
    pub parent_genome_binding: Option<String>,
    /// Opaque binding to the exact closure model used for this genome.
    pub closure_model_binding: String,
    /// Opaque binding to the derived supportability/viability report.
    pub supportability_report_binding: String,
    /// Opaque evidence that this genome passed its qualification gate.
    pub qualification_binding: String,
    /// Requirement-level evidence references, strictly sorted and unique.
    pub requirement_evidence_refs: Vec<String>,
    /// Explicit substitution-evidence references used in this lineage step.
    pub substitution_evidence_refs: Vec<String>,
}

impl RegenerativeGenomeLineageEvidenceV1 {
    /// Validate canonical shape and minimum lineage evidence.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative genome lineage schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.genome_id) {
            return Err("genome_id is empty, oversized, padded, whitespace-bearing, or control-bearing".into());
        }
        for (field, binding) in [
            ("genome_binding", self.genome_binding.as_str()),
            ("closure_model_binding", self.closure_model_binding.as_str()),
            (
                "supportability_report_binding",
                self.supportability_report_binding.as_str(),
            ),
            ("qualification_binding", self.qualification_binding.as_str()),
        ] {
            if !canonical_reference(binding, MAX_BINDING_BYTES) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if let Some(parent) = &self.parent_genome_binding {
            if !canonical_reference(parent, MAX_BINDING_BYTES) {
                return Err("parent_genome_binding is not a canonical opaque binding".into());
            }
            if parent == &self.genome_binding {
                return Err("genome lineage cannot name itself as its parent".into());
            }
        }

        validate_refs("requirement_evidence_refs", &self.requirement_evidence_refs)?;
        validate_refs(
            "substitution_evidence_refs",
            &self.substitution_evidence_refs,
        )?;
        if self.requirement_evidence_refs.is_empty() {
            return Err("genome lineage requires requirement evidence".into());
        }
        if !self.substitution_evidence_refs.is_empty() && self.parent_genome_binding.is_none() {
            return Err("substitution-derived lineage requires a parent genome binding".into());
        }
        Ok(())
    }

    /// Serialize the canonical V1 provenance payload.
    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize regenerative genome lineage: {error}"))
    }

    /// Stable identity of this lineage record, independent of maritime sequence.
    ///
    /// This digest is an integrity/content identifier only. It authenticates
    /// neither the genome author nor the qualification authority.
    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-genome-lineage-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    /// Carry this lineage record over the existing maritime store-forward path.
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
    if refs.len() > MAX_REFERENCES_PER_CLASS {
        return Err(format!(
            "{field} contains too many references (max {MAX_REFERENCES_PER_CLASS})"
        ));
    }
    for reference in refs {
        if !canonical_reference(reference, MAX_REFERENCE_BYTES) {
            return Err(format!("{field} contains a non-canonical opaque reference"));
        }
    }
    if refs.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(format!(
            "{field} must be strictly sorted and duplicate-free"
        ));
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

    fn fixture() -> RegenerativeGenomeLineageEvidenceV1 {
        RegenerativeGenomeLineageEvidenceV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            genome_id: "manta-genome-v2".into(),
            genome_binding: "genome:manta-v2:sha256:example".into(),
            parent_genome_binding: Some("genome:manta-v1:sha256:example".into()),
            closure_model_binding: "model:manta-forge-v2:sha256:example".into(),
            supportability_report_binding: "supportability:manta-v2:30-period".into(),
            qualification_binding: "qualification:manta-v2".into(),
            requirement_evidence_refs: vec![
                "requirement:req-electronics".into(),
                "requirement:req-reactor-service".into(),
                "requirement:req-structure".into(),
            ],
            substitution_evidence_refs: vec!["substitution:electronics-local-v1".into()],
        }
    }

    #[test]
    fn lineage_round_trips_strictly_and_pins_content_identity() {
        let evidence = fixture();
        let json = evidence.to_payload_json().unwrap();
        let decoded: RegenerativeGenomeLineageEvidenceV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, evidence);
        assert_eq!(
            evidence.content_digest().unwrap(),
            "a319bd837e9035f576ac5854d194dd3dbd351a5eae2511407c45135517775ca6"
        );
    }

    #[test]
    fn unknown_fields_and_noncanonical_reference_sets_fail_closed() {
        let json = fixture().to_payload_json().unwrap();
        let injected = json.replacen(
            "{\"schema_version\":1,",
            "{\"schema_version\":1,\"authority_override\":true,",
            1,
        );
        assert!(serde_json::from_str::<RegenerativeGenomeLineageEvidenceV1>(&injected).is_err());

        let mut duplicate = fixture();
        duplicate.requirement_evidence_refs = vec![
            "requirement:req-a".into(),
            "requirement:req-a".into(),
        ];
        assert!(duplicate.validate().is_err());

        let mut unsorted = fixture();
        unsorted.substitution_evidence_refs = vec![
            "substitution:z".into(),
            "substitution:a".into(),
        ];
        assert!(unsorted.validate().is_err());
    }

    #[test]
    fn lineage_cannot_self_parent_or_claim_substitution_without_parent() {
        let mut self_parent = fixture();
        self_parent.parent_genome_binding = Some(self_parent.genome_binding.clone());
        assert!(self_parent.validate().is_err());

        let mut root_with_substitution = fixture();
        root_with_substitution.parent_genome_binding = None;
        assert!(root_with_substitution.validate().is_err());
    }

    #[test]
    fn canonical_lineage_fits_maritime_transport_and_pins_outer_identity() {
        let envelope = fixture()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                4,
                0,
                1_788_900_001_000_000,
                "evidence:qualified-genome-event-01",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
        assert_eq!(
            envelope.content_digest().unwrap(),
            "ca5ac3b0c79012f852646e1bf6a61c2f07a8caf41abc69e676ff5bf949bae285"
        );
        let decoded: RegenerativeGenomeLineageEvidenceV1 =
            serde_json::from_str(&envelope.payload_json).unwrap();
        assert_eq!(decoded, fixture());
    }

    #[test]
    fn payload_contains_no_embedded_recipe_or_control_fields() {
        let json = fixture().to_payload_json().unwrap();
        for forbidden in [
            "toolpath",
            "temperature_setpoint",
            "pressure_setpoint",
            "actuator_command",
            "enrichment",
            "fuel_fabrication",
            "reactor_control",
        ] {
            assert!(!json.contains(forbidden));
        }
    }
}
