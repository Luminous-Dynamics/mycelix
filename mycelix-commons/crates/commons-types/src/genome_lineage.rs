// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Recipe-free lineage evidence for regenerative design generations.
//!
//! This module records which versioned design/genome succeeded which prior design,
//! which closure/substitution evidence justified that transition, and which
//! qualification evidence accepted the successor. It deliberately contains no CAD,
//! bill-of-material detail, manufacturing recipe, process setpoint, actuator command,
//! or nuclear fuel-cycle procedure.

use crate::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_REFERENCE_BYTES: usize = 512;
const MAX_REFERENCE_COUNT: usize = 64;

/// Strict provenance record for one accepted successor design/genome.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeGenomeLineageV1 {
    pub schema_version: u8,
    /// Canonical lineage event identifier.
    pub lineage_id: String,
    /// Opaque binding to the accepted successor genome/manifest.
    pub successor_genome_binding: String,
    /// Opaque binding to the immediate parent genome/manifest, when one exists.
    pub parent_genome_binding: Option<String>,
    /// Opaque binding to the exact closure model used for successor qualification.
    pub closure_model_binding: String,
    /// Explicit substitution-evidence references used by this successor.
    /// These references are audit evidence only; they do not themselves authorize a substitution.
    pub substitution_evidence_refs: Vec<String>,
    /// Opaque evidence that the successor satisfied its required metrology/requalification gates.
    pub qualification_binding: String,
    /// Opaque producer/authority evidence for whoever issued this lineage record.
    pub producer_evidence_binding: String,
}

impl RegenerativeGenomeLineageV1 {
    /// Validate shape, canonical references and lineage semantics.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative genome lineage schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.lineage_id) {
            return Err("lineage_id is empty, oversized, padded, whitespace-bearing, or control-bearing".into());
        }
        for (field, binding) in [
            ("successor_genome_binding", self.successor_genome_binding.as_str()),
            ("closure_model_binding", self.closure_model_binding.as_str()),
            ("qualification_binding", self.qualification_binding.as_str()),
            ("producer_evidence_binding", self.producer_evidence_binding.as_str()),
        ] {
            if !canonical_binding(binding) {
                return Err(format!("{field} is malformed"));
            }
        }
        if let Some(parent) = &self.parent_genome_binding {
            if !canonical_binding(parent) {
                return Err("parent_genome_binding is malformed".into());
            }
            if parent == &self.successor_genome_binding {
                return Err("successor genome must differ from its parent genome".into());
            }
        }
        validate_refs(&self.substitution_evidence_refs)?;
        Ok(())
    }

    /// Canonical JSON representation carried by the maritime store-forward envelope.
    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize regenerative genome lineage: {error}"))
    }

    /// Transport-independent content identity for this lineage record.
    ///
    /// This digest proves only canonical record identity. It does not authenticate
    /// the producer, validate the underlying genome, or prove qualification by itself.
    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-genome-lineage-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    /// Wrap this lineage record in the existing maritime evidence transport.
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

fn validate_refs(refs: &[String]) -> Result<(), String> {
    if refs.len() > MAX_REFERENCE_COUNT {
        return Err(format!("too many substitution evidence references (max {MAX_REFERENCE_COUNT})"));
    }
    for reference in refs {
        if !canonical_reference(reference) {
            return Err("substitution_evidence_refs contains a non-canonical opaque reference".into());
        }
    }
    if refs.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err("substitution_evidence_refs must be strictly sorted and duplicate-free".into());
    }
    Ok(())
}

fn canonical_id(value: &str) -> bool {
    canonical_text(value, MAX_ID_BYTES) && !value.chars().any(char::is_whitespace)
}

fn canonical_binding(value: &str) -> bool {
    canonical_text(value, MAX_BINDING_BYTES)
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
}

fn canonical_reference(value: &str) -> bool {
    canonical_text(value, MAX_REFERENCE_BYTES)
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
}

fn canonical_text(value: &str, max_bytes: usize) -> bool {
    !value.is_empty()
        && value.len() <= max_bytes
        && value.trim() == value
        && !value.chars().any(char::is_control)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture() -> RegenerativeGenomeLineageV1 {
        RegenerativeGenomeLineageV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            lineage_id: "manta-genome-lineage-0007".into(),
            successor_genome_binding: "genome:manta-v7:blake3:successor".into(),
            parent_genome_binding: Some("genome:manta-v6:blake3:parent".into()),
            closure_model_binding: "closure-model:manta-forge-v7:blake3:model".into(),
            substitution_evidence_refs: vec![
                "substitution:control-electronics-v2".into(),
                "substitution:pump-motor-v3".into(),
            ],
            qualification_binding: "qualification:manta-v7:accepted".into(),
            producer_evidence_binding: "producer-evidence:forge-02-session-91".into(),
        }
    }

    #[test]
    fn strict_round_trip_and_identity_are_stable() {
        let lineage = fixture();
        let json = lineage.to_payload_json().unwrap();
        let decoded: RegenerativeGenomeLineageV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, lineage);
        assert_eq!(decoded.content_digest().unwrap(), lineage.content_digest().unwrap());
    }

    #[test]
    fn same_parent_as_successor_is_rejected() {
        let mut lineage = fixture();
        lineage.parent_genome_binding = Some(lineage.successor_genome_binding.clone());
        assert!(lineage.validate().is_err());
    }

    #[test]
    fn substitution_refs_are_sorted_and_unique() {
        let mut duplicate = fixture();
        duplicate.substitution_evidence_refs = vec![
            "substitution:a".into(),
            "substitution:a".into(),
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
    fn unknown_fields_are_rejected() {
        let json = fixture().to_payload_json().unwrap();
        let injected = json.replacen(
            "{\"schema_version\":1,",
            "{\"schema_version\":1,\"manufacturing_recipe\":\"forbidden\",",
            1,
        );
        assert!(serde_json::from_str::<RegenerativeGenomeLineageV1>(&injected).is_err());
    }

    #[test]
    fn lineage_payload_contains_no_recipe_or_control_surface() {
        let json = fixture().to_payload_json().unwrap();
        for forbidden in [
            "toolpath",
            "temperature_setpoint",
            "pressure_setpoint",
            "reactor_control",
            "fuel_fabrication",
            "manufacturing_recipe",
            "actuator_command",
        ] {
            assert!(!json.contains(forbidden));
        }
    }

    #[test]
    fn lineage_fits_existing_maritime_transport() {
        let envelope = fixture()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                3,
                18,
                1_788_900_000_100_000,
                "evidence:qualified-genome-lineage-18",
            )
            .unwrap();
        assert!(envelope.validate().is_ok());
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
    }
}
