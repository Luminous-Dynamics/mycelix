// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Recipe-free provenance for regenerative industrial-epoch handoffs.
//!
//! This record preserves claims produced by upstream semantic qualification and
//! dynamic accounting while independently binding the exact predecessor/successor
//! Mycelix Genome-lineage records. It does not recompute engineering semantics,
//! inventory conservation, qualification validity, or physical manufacturability.

use crate::{
    verify_regenerative_genome_lineage_successor, MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeGenomeLineageEvidenceV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const REGENERATIVE_EPOCH_HANDOFF_PROVENANCE_SCHEMA_V1: u8 = 1;

const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;
const MAX_ITEMS: usize = 128;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeEpochTransferProvenanceV1 {
    pub source_dependency_id: String,
    pub successor_dependency_id: String,
    pub transferred_units: u64,
    pub retired_units: u64,
    pub transfer_qualification_binding: String,
    pub safeguarded_continuity_binding: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeEpochExternalAdmissionProvenanceV1 {
    pub successor_dependency_id: String,
    pub units: u64,
    pub inventory_evidence_binding: String,
    pub admission_qualification_binding: String,
    pub safeguarded_admission_binding: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeEpochHandoffProvenanceV1 {
    pub schema_version: u8,
    pub handoff_id: String,
    pub source_epoch_binding: String,
    pub successor_epoch_binding: String,
    pub source_genome_binding: String,
    pub successor_genome_binding: String,
    /// Exact content identity of the predecessor Mycelix lineage record.
    pub source_lineage_content_digest: String,
    /// Exact content identity of the successor Mycelix lineage record.
    pub successor_lineage_content_digest: String,
    pub source_closure_model_binding: String,
    pub successor_closure_model_binding: String,
    /// Opaque binding to the exact Symthaea handoff-qualification result.
    pub symthaea_handoff_qualification_binding: String,
    /// Opaque binding to the exact Symtropy accounting receipt.
    pub symtropy_handoff_receipt_binding: String,
    /// Binding to the exact cross-repo fixture/scenario definition when applicable.
    pub cross_repo_fixture_binding: String,
    /// Sorted/unique by `(source_dependency_id, successor_dependency_id)`.
    pub transfers: Vec<RegenerativeEpochTransferProvenanceV1>,
    /// Sorted/unique by `successor_dependency_id`.
    pub external_admissions: Vec<RegenerativeEpochExternalAdmissionProvenanceV1>,
}

impl RegenerativeEpochHandoffProvenanceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_EPOCH_HANDOFF_PROVENANCE_SCHEMA_V1 {
            return Err(format!(
                "unsupported regenerative epoch handoff provenance schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.handoff_id) {
            return Err("handoff_id is not canonical".into());
        }
        for (field, binding) in [
            ("source_epoch_binding", self.source_epoch_binding.as_str()),
            ("successor_epoch_binding", self.successor_epoch_binding.as_str()),
            ("source_genome_binding", self.source_genome_binding.as_str()),
            ("successor_genome_binding", self.successor_genome_binding.as_str()),
            (
                "source_closure_model_binding",
                self.source_closure_model_binding.as_str(),
            ),
            (
                "successor_closure_model_binding",
                self.successor_closure_model_binding.as_str(),
            ),
            (
                "symthaea_handoff_qualification_binding",
                self.symthaea_handoff_qualification_binding.as_str(),
            ),
            (
                "symtropy_handoff_receipt_binding",
                self.symtropy_handoff_receipt_binding.as_str(),
            ),
            (
                "cross_repo_fixture_binding",
                self.cross_repo_fixture_binding.as_str(),
            ),
        ] {
            if !canonical_reference(binding) {
                return Err(format!("{field} is not a canonical opaque binding"));
            }
        }
        if self.source_epoch_binding == self.successor_epoch_binding {
            return Err("source and successor epoch bindings must be distinct".into());
        }
        if self.source_genome_binding == self.successor_genome_binding {
            return Err("source and successor genome bindings must be distinct".into());
        }
        if !lower_hex_64(&self.source_lineage_content_digest)
            || !lower_hex_64(&self.successor_lineage_content_digest)
        {
            return Err("lineage content digests must be canonical lowercase 64-hex values".into());
        }
        if self.transfers.len() > MAX_ITEMS || self.external_admissions.len() > MAX_ITEMS {
            return Err(format!("handoff evidence exceeds {MAX_ITEMS} records per class"));
        }
        if self.transfers.is_empty() && self.external_admissions.is_empty() {
            return Err("handoff provenance requires at least one transfer or external admission".into());
        }

        let mut successor_targets = BTreeSet::new();
        for transfer in &self.transfers {
            if !canonical_id(&transfer.source_dependency_id)
                || !canonical_id(&transfer.successor_dependency_id)
            {
                return Err("transfer contains a non-canonical dependency id".into());
            }
            if transfer.transferred_units == 0 {
                return Err("transfer record must carry a positive transferred quantity".into());
            }
            if !canonical_reference(&transfer.transfer_qualification_binding) {
                return Err("transfer qualification binding is not canonical".into());
            }
            if let Some(binding) = &transfer.safeguarded_continuity_binding {
                if !canonical_reference(binding) {
                    return Err("safeguarded continuity binding is not canonical".into());
                }
            }
            if !successor_targets.insert(transfer.successor_dependency_id.clone()) {
                return Err("multiple predecessor transfers target the same successor dependency".into());
            }
        }
        if self.transfers.windows(2).any(|pair| {
            (
                pair[0].source_dependency_id.as_str(),
                pair[0].successor_dependency_id.as_str(),
            ) >= (
                pair[1].source_dependency_id.as_str(),
                pair[1].successor_dependency_id.as_str(),
            )
        }) {
            return Err("transfer records must be strictly sorted and duplicate-free".into());
        }

        for admission in &self.external_admissions {
            if !canonical_id(&admission.successor_dependency_id) {
                return Err("external admission contains a non-canonical dependency id".into());
            }
            if admission.units == 0 {
                return Err("external admission must carry a positive quantity".into());
            }
            for binding in [
                &admission.inventory_evidence_binding,
                &admission.admission_qualification_binding,
            ] {
                if !canonical_reference(binding) {
                    return Err("external admission binding is not canonical".into());
                }
            }
            if let Some(binding) = &admission.safeguarded_admission_binding {
                if !canonical_reference(binding) {
                    return Err("safeguarded admission binding is not canonical".into());
                }
            }
        }
        if self
            .external_admissions
            .windows(2)
            .any(|pair| pair[0].successor_dependency_id >= pair[1].successor_dependency_id)
        {
            return Err("external admissions must be strictly sorted and duplicate-free".into());
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize regenerative epoch handoff: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-epoch-handoff-v1\0");
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

/// Verify the Mycelix-owned lineage bindings surrounding one epoch handoff.
///
/// This does not verify the Symthaea qualification or Symtropy accounting receipt;
/// those remain opaque upstream evidence identities.
pub fn verify_regenerative_epoch_handoff_provenance(
    source_lineage: &RegenerativeGenomeLineageEvidenceV1,
    successor_lineage: &RegenerativeGenomeLineageEvidenceV1,
    handoff: &RegenerativeEpochHandoffProvenanceV1,
) -> Result<(), String> {
    handoff.validate()?;
    verify_regenerative_genome_lineage_successor(source_lineage, successor_lineage)?;
    if handoff.source_genome_binding != source_lineage.genome_binding {
        return Err("handoff source genome binding does not match predecessor lineage".into());
    }
    if handoff.successor_genome_binding != successor_lineage.genome_binding {
        return Err("handoff successor genome binding does not match successor lineage".into());
    }
    if handoff.source_closure_model_binding != source_lineage.closure_model_binding
        || handoff.successor_closure_model_binding != successor_lineage.closure_model_binding
    {
        return Err("handoff closure-model binding does not match Genome lineage evidence".into());
    }
    if handoff.source_lineage_content_digest != source_lineage.content_digest()? {
        return Err("handoff predecessor lineage digest mismatch".into());
    }
    if handoff.successor_lineage_content_digest != successor_lineage.content_digest()? {
        return Err("handoff successor lineage digest mismatch".into());
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1;

    fn source_lineage() -> RegenerativeGenomeLineageEvidenceV1 {
        RegenerativeGenomeLineageEvidenceV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            genome_id: "genome-v1".into(),
            genome_binding: "genome:manta-v1".into(),
            parent_genome_binding: None,
            closure_model_binding: "model:manta-v1".into(),
            supportability_report_binding: "supportability:manta-v1".into(),
            qualification_binding: "qualification:manta-v1".into(),
            requirement_evidence_refs: vec!["requirement:operation-v1".into()],
            substitution_evidence_refs: Vec::new(),
        }
    }

    fn successor_lineage() -> RegenerativeGenomeLineageEvidenceV1 {
        RegenerativeGenomeLineageEvidenceV1 {
            schema_version: REGENERATIVE_GENOME_LINEAGE_SCHEMA_V1,
            genome_id: "genome-v2".into(),
            genome_binding: "genome:manta-v2".into(),
            parent_genome_binding: Some("genome:manta-v1".into()),
            closure_model_binding: "model:manta-v2".into(),
            supportability_report_binding: "supportability:manta-v2".into(),
            qualification_binding: "qualification:manta-v2".into(),
            requirement_evidence_refs: vec!["requirement:operation-v2".into()],
            substitution_evidence_refs: vec!["substitution:qualified-v1-v2".into()],
        }
    }

    fn handoff() -> RegenerativeEpochHandoffProvenanceV1 {
        let source = source_lineage();
        let successor = successor_lineage();
        RegenerativeEpochHandoffProvenanceV1 {
            schema_version: REGENERATIVE_EPOCH_HANDOFF_PROVENANCE_SCHEMA_V1,
            handoff_id: "handoff-manta-v1-v2".into(),
            source_epoch_binding: "epoch:manta-v1".into(),
            successor_epoch_binding: "epoch:manta-v2".into(),
            source_genome_binding: source.genome_binding.clone(),
            successor_genome_binding: successor.genome_binding.clone(),
            source_lineage_content_digest: source.content_digest().unwrap(),
            successor_lineage_content_digest: successor.content_digest().unwrap(),
            source_closure_model_binding: source.closure_model_binding.clone(),
            successor_closure_model_binding: successor.closure_model_binding.clone(),
            symthaea_handoff_qualification_binding: "symthaea-handoff:qualified-v1-v2".into(),
            symtropy_handoff_receipt_binding: "symtropy-handoff:receipt-v1-v2".into(),
            cross_repo_fixture_binding:
                "git-blob:725aa1eadf417eae1be95829b0e93fe7e9c9ee60".into(),
            transfers: vec![
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "metrology-v1".into(),
                    successor_dependency_id: "metrology-v2".into(),
                    transferred_units: 19,
                    retired_units: 10,
                    transfer_qualification_binding: "qualification:metrology-v1-v2".into(),
                    safeguarded_continuity_binding: None,
                },
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "reactor-service-v1".into(),
                    successor_dependency_id: "reactor-service-v2".into(),
                    transferred_units: 6,
                    retired_units: 0,
                    transfer_qualification_binding: "qualification:reactor-service-v1-v2".into(),
                    safeguarded_continuity_binding: Some(
                        "safeguarded:reactor-service-continuity-v1-v2".into(),
                    ),
                },
                RegenerativeEpochTransferProvenanceV1 {
                    source_dependency_id: "spares-v1".into(),
                    successor_dependency_id: "spares-v2".into(),
                    transferred_units: 79,
                    retired_units: 20,
                    transfer_qualification_binding: "qualification:spares-v1-v2".into(),
                    safeguarded_continuity_binding: None,
                },
            ],
            external_admissions: vec![RegenerativeEpochExternalAdmissionProvenanceV1 {
                successor_dependency_id: "spares-v2".into(),
                units: 5,
                inventory_evidence_binding: "external:qualified-spares".into(),
                admission_qualification_binding: "qualification:external-spares-v2".into(),
                safeguarded_admission_binding: None,
            }],
        }
    }

    #[test]
    fn exact_lineage_records_bind_the_epoch_handoff() {
        let source = source_lineage();
        let successor = successor_lineage();
        let record = handoff();
        assert_eq!(
            verify_regenerative_epoch_handoff_provenance(&source, &successor, &record),
            Ok(())
        );
        assert_eq!(record.transfers.len(), 3);
        assert_eq!(record.external_admissions.len(), 1);
        assert_eq!(record.content_digest().unwrap().len(), 64);
    }

    #[test]
    fn tampered_lineage_digest_fails_closed() {
        let source = source_lineage();
        let successor = successor_lineage();
        let mut record = handoff();
        record.source_lineage_content_digest = "0".repeat(64);
        assert!(
            verify_regenerative_epoch_handoff_provenance(&source, &successor, &record).is_err()
        );
    }

    #[test]
    fn duplicate_successor_transfer_target_fails_closed() {
        let mut record = handoff();
        record.transfers[2].successor_dependency_id = "metrology-v2".into();
        assert!(record.validate().is_err());
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let json = handoff().to_payload_json().unwrap();
        let injected = json.replacen(
            "{\"schema_version\":1,",
            "{\"schema_version\":1,\"authority_override\":true,",
            1,
        );
        assert!(serde_json::from_str::<RegenerativeEpochHandoffProvenanceV1>(&injected).is_err());
    }

    #[test]
    fn handoff_fits_existing_maritime_store_forward_path() {
        let envelope = handoff()
            .to_maritime_envelope(
                "manta-civil-demo-01",
                5,
                0,
                1_789_000_000_000_000,
                "evidence:epoch-handoff-v1-v2",
            )
            .unwrap();
        assert_eq!(envelope.kind, MaritimeEvidenceKind::LogisticsEvent);
        let decoded: RegenerativeEpochHandoffProvenanceV1 =
            serde_json::from_str(&envelope.payload_json).unwrap();
        assert_eq!(decoded, handoff());
    }
}
