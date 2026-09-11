// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical component provenance for regenerative maritime infrastructure.
//!
//! This module records evidence *about* manufacturing/remanufacturing; it does
//! not authenticate the producer and contains no manufacturing recipe or control
//! instructions. Outer Commons authorship remains governed by the bridge
//! integrity zome, while upstream producer/session evidence remains opaque.

use crate::{MaritimeEvidenceEnvelope, MaritimeEvidenceKind};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const REGENERATIVE_COMPONENT_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_REFERENCE_BYTES: usize = 512;
const MAX_REFERENCES_PER_CLASS: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComponentOriginV1 {
    Manufactured,
    Remanufactured,
}

/// Strict, recipe-free provenance payload for one qualified component instance.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeComponentEvidenceV1 {
    pub schema_version: u8,
    pub component_id: String,
    /// Opaque binding to the design/BOM/capability definition that was built.
    pub design_binding: String,
    pub origin: ComponentOriginV1,
    /// Qualified material/input batch references, strictly sorted and unique.
    pub material_batch_refs: Vec<String>,
    /// Recovered/recycled input references, strictly sorted and unique.
    pub recycled_input_refs: Vec<String>,
    /// Evidence references for production/remanufacturing, never embedded recipes.
    pub process_evidence_refs: Vec<String>,
    /// Metrology/NDE/calibration evidence references.
    pub metrology_evidence_refs: Vec<String>,
    /// Opaque binding to producer identity/authorization evidence owned upstream.
    pub producer_evidence_binding: String,
    /// Opaque evidence that this exact instance passed its qualification gate.
    pub qualification_binding: String,
}

impl RegenerativeComponentEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_COMPONENT_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative component schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.component_id, MAX_ID_BYTES) {
            return Err("component_id is empty, oversized, padded, whitespace-bearing, or control-bearing".into());
        }
        for (field, value) in [
            ("design_binding", self.design_binding.as_str()),
            ("producer_evidence_binding", self.producer_evidence_binding.as_str()),
            ("qualification_binding", self.qualification_binding.as_str()),
        ] {
            if !canonical_text(value, MAX_BINDING_BYTES) {
                return Err(format!(
                    "{field} is empty, oversized, padded, or contains control bytes"
                ));
            }
        }

        validate_refs("material_batch_refs", &self.material_batch_refs)?;
        validate_refs("recycled_input_refs", &self.recycled_input_refs)?;
        validate_refs("process_evidence_refs", &self.process_evidence_refs)?;
        validate_refs("metrology_evidence_refs", &self.metrology_evidence_refs)?;

        if self.material_batch_refs.is_empty() && self.recycled_input_refs.is_empty() {
            return Err("component evidence requires at least one material or recycled input reference".into());
        }
        if self.process_evidence_refs.is_empty() {
            return Err("component evidence requires process evidence".into());
        }
        if self.metrology_evidence_refs.is_empty() {
            return Err("component evidence requires metrology/inspection evidence".into());
        }
        if self.origin == ComponentOriginV1::Remanufactured && self.recycled_input_refs.is_empty() {
            return Err("remanufactured component requires at least one recovered/recycled input reference".into());
        }

        let material_refs: BTreeSet<&str> = self.material_batch_refs.iter().map(String::as_str).collect();
        if let Some(overlap) = self
            .recycled_input_refs
            .iter()
            .map(String::as_str)
            .find(|reference| material_refs.contains(reference))
        {
            return Err(format!(
                "input reference appears in both material and recycled classes: {overlap}"
            ));
        }
        Ok(())
    }

    /// Serialize the exact canonical V1 representation used as maritime payload JSON.
    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize regenerative component evidence: {error}"))
    }

    /// Stable identity of the component-evidence record, independent of transport sequence.
    ///
    /// This is a content identifier only. It does not authenticate the producer,
    /// qualify the component, or replace the outer Commons action signature.
    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-component-evidence-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    /// Wrap this provenance payload in the existing store-forward maritime envelope.
    ///
    /// `event_evidence_binding` belongs to the upstream event/session evidence
    /// provider. It is deliberately distinct from this record's content digest,
    /// producer evidence and qualification binding.
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
        if !canonical_reference(reference) {
            return Err(format!(
                "{field} contains a non-canonical opaque reference"
            ));
        }
    }
    if refs.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(format!(
            "{field} must be strictly sorted and duplicate-free"
        ));
    }
    Ok(())
}

fn canonical_reference(value: &str) -> bool {
    canonical_text(value, MAX_REFERENCE_BYTES)
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
}

fn canonical_id(value: &str, max_bytes: usize) -> bool {
    canonical_text(value, max_bytes) && !value.chars().any(char::is_whitespace)
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

    fn fixture() -> RegenerativeComponentEvidenceV1 {
        RegenerativeComponentEvidenceV1 {
            schema_version: REGENERATIVE_COMPONENT_SCHEMA_V1,
            component_id: "pump-impeller-0042".into(),
            design_binding: "design:impeller-v7:sha256:example".into(),
            origin: ComponentOriginV1::Remanufactured,
            material_batch_refs: vec!["batch:alloy-feedstock-003".into()],
            recycled_input_refs: vec!["recovered:impeller-0031".into()],
            process_evidence_refs: vec!["process-evidence:run-991".into()],
            metrology_evidence_refs: vec![
                "metrology:balance-991".into(),
                "metrology:dimensions-991".into(),
            ],
            producer_evidence_binding: "producer-evidence:forge-02-session-77".into(),
            qualification_binding: "qualification:impeller-0042".into(),
        }
    }

    #[test]
    fn qualified_component_round_trips_strictly() {
        let evidence = fixture();
        let json = evidence.to_payload_json().unwrap();
        let decoded: RegenerativeComponentEvidenceV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, evidence);
        assert!(decoded.validate().is_ok());
    }

    #[test]
    fn unknown_v1_fields_are_rejected() {
        let json = fixture().to_payload_json().unwrap();
        let injected = json.replacen(
            "{\"schema_version\":1,",
            "{\"schema_version\":1,\"authority_override\":true,",
            1,
        );
        assert!(serde_json::from_str::<RegenerativeComponentEvidenceV1>(&injected).is_err());
    }

    #[test]
    fn evidence_sets_are_canonical_and_complete() {
        let mut duplicate = fixture();
        duplicate.metrology_evidence_refs = vec!["metrology:a".into(), "metrology:a".into()];
        assert!(duplicate.validate().is_err());

        let mut unsorted = fixture();
        unsorted.material_batch_refs = vec!["batch:z".into(), "batch:a".into()];
        assert!(unsorted.validate().is_err());

        let mut free_form = fixture();
        free_form.process_evidence_refs = vec!["process evidence with instructions".into()];
        assert!(free_form.validate().is_err());

        let mut no_metrology = fixture();
        no_metrology.metrology_evidence_refs.clear();
        assert!(no_metrology.validate().is_err());
    }

    #[test]
    fn material_and_recycled_inputs_cannot_double_count_same_reference() {
        let mut evidence = fixture();
        evidence.recycled_input_refs = vec!["batch:alloy-feedstock-003".into()];
        assert!(evidence.validate().unwrap_err().contains("both material and recycled"));
    }

    #[test]
    fn remanufactured_origin_requires_recovered_input() {
        let mut evidence = fixture();
        evidence.recycled_input_refs.clear();
        assert_eq!(
            evidence.validate(),
            Err("remanufactured component requires at least one recovered/recycled input reference".into())
        );
    }

    #[test]
    fn component_content_identity_is_transport_independent_and_sensitive() {
        let evidence = fixture();
        let digest = evidence.content_digest().unwrap();
        assert_eq!(digest.len(), 64);

        let decoded: RegenerativeComponentEvidenceV1 =
            serde_json::from_str(&evidence.to_payload_json().unwrap()).unwrap();
        assert_eq!(decoded.content_digest().unwrap(), digest);

        let mut changed = evidence;
        changed.qualification_binding = "qualification:impeller-0042-r2".into();
        assert_ne!(changed.content_digest().unwrap(), digest);
    }

    #[test]
    fn component_payload_fits_existing_maritime_transport() {
        let envelope = fixture()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                3,
                17,
                1_788_900_000_000_000,
                "evidence:qualified-forge-event-17",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
        assert!(envelope.validate().is_ok());
        let decoded: RegenerativeComponentEvidenceV1 =
            serde_json::from_str(&envelope.payload_json).unwrap();
        assert_eq!(decoded, fixture());
    }

    #[test]
    fn payload_contains_no_embedded_process_recipe_fields() {
        let json = fixture().to_payload_json().unwrap();
        for forbidden in [
            "temperature_setpoint",
            "pressure_setpoint",
            "toolpath",
            "enrichment",
            "fuel_fabrication",
            "actuator_command",
        ] {
            assert!(!json.contains(forbidden));
        }
    }
}
