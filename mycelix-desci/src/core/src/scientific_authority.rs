// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Explicit authority classes for scientific information.
//!
//! This module does not create a new source of scientific truth. It names the
//! authority boundaries already implicit in the append-only scientific event
//! kernel so that derived analysis, model inference, and advisory assessments
//! cannot be mistaken for canonical evidence or attestations.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const MAX_DERIVED_SOURCE_INPUTS: usize = 4096;
const MAX_DERIVED_IDENTIFIER_BYTES: usize = 512;

/// Authority class carried by scientific information as it moves between
/// canonical history, external artifacts, human attestations, and derived
/// analysis layers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScientificAuthorityClass {
    /// An event accepted through the governed append-only scientific history.
    /// The envelope itself is authoritative history, not scientific evidence.
    CanonicalEvent,
    /// A content-addressed external artifact eligible for governed evidence
    /// admission.
    ExternalEvidence,
    /// A signed human or institutional assertion eligible for governed
    /// attestation admission.
    HumanAttestation,
    /// A deterministic or statistical view computed from canonical inputs.
    DerivedProjection,
    /// A relationship, hypothesis, or conclusion produced by a model/reasoner.
    ModelInference,
    /// Non-binding advice such as critique, prioritization, or review assistance.
    AdvisoryAssessment,
}

impl ScientificAuthorityClass {
    /// Whether this class represents scientific source material rather than a
    /// history envelope or a derived/advisory product.
    ///
    /// Concrete admission and scientific-maturity effects still belong to the
    /// governed scientific event path and its evidence policy.
    pub const fn is_scientific_source_material(self) -> bool {
        matches!(self, Self::ExternalEvidence | Self::HumanAttestation)
    }

    /// Whether this class is necessarily derived/advisory and therefore must
    /// not be promoted to evidence or attestation merely by conversion.
    pub const fn is_derived(self) -> bool {
        matches!(
            self,
            Self::DerivedProjection | Self::ModelInference | Self::AdvisoryAssessment
        )
    }

    /// Whether this class is eligible to enter the governed evidence-admission
    /// path. Returning `true` does not bypass validation or authorization.
    pub const fn eligible_for_governed_evidence_admission(self) -> bool {
        matches!(self, Self::ExternalEvidence)
    }

    /// Whether this class is eligible to enter the governed attestation-
    /// admission path. Returning `true` does not bypass validation,
    /// authorization, identity, or signature checks.
    pub const fn eligible_for_governed_attestation_admission(self) -> bool {
        matches!(self, Self::HumanAttestation)
    }
}

/// Metadata describing a derived scientific product without granting it
/// evidentiary authority.
///
/// Fields are intentionally private. Construction and deserialization both go
/// through the same validation path so a wire payload cannot relabel a derived
/// product as evidence or a human attestation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(try_from = "DerivedScientificProductWire")]
pub struct DerivedScientificProduct {
    authority_class: ScientificAuthorityClass,
    /// Stable identifiers for the canonical/source inputs from which this was
    /// derived. Interpretation of identifier syntax belongs to the producer.
    source_inputs: Vec<String>,
    /// Versioned policy/rule/model identifier responsible for the derivation.
    producer: String,
}

#[derive(Debug, Clone, Deserialize)]
struct DerivedScientificProductWire {
    authority_class: ScientificAuthorityClass,
    source_inputs: Vec<String>,
    producer: String,
}

impl TryFrom<DerivedScientificProductWire> for DerivedScientificProduct {
    type Error = &'static str;

    fn try_from(value: DerivedScientificProductWire) -> Result<Self, Self::Error> {
        Self::new(value.authority_class, value.source_inputs, value.producer)
    }
}

impl DerivedScientificProduct {
    pub fn new(
        authority_class: ScientificAuthorityClass,
        mut source_inputs: Vec<String>,
        producer: impl Into<String>,
    ) -> Result<Self, &'static str> {
        if !authority_class.is_derived() {
            return Err("derived scientific products require a derived/advisory authority class");
        }
        if source_inputs.is_empty() {
            return Err("derived scientific products require at least one source input");
        }
        if source_inputs.len() > MAX_DERIVED_SOURCE_INPUTS {
            return Err("derived scientific product has too many source inputs");
        }

        let mut unique = BTreeSet::new();
        for source in &source_inputs {
            if source.is_empty()
                || source.trim() != source
                || source.len() > MAX_DERIVED_IDENTIFIER_BYTES
                || source.chars().any(char::is_control)
            {
                return Err(
                    "derived scientific product source inputs must be canonical non-empty identifiers",
                );
            }
            if !unique.insert(source.clone()) {
                return Err("derived scientific product source inputs must not contain duplicates");
            }
        }
        source_inputs.sort();

        let producer = producer.into();
        if producer.is_empty()
            || producer.trim() != producer
            || producer.len() > MAX_DERIVED_IDENTIFIER_BYTES
            || producer.chars().any(char::is_control)
        {
            return Err("derived scientific products require a canonical producer identifier");
        }

        Ok(Self {
            authority_class,
            source_inputs,
            producer,
        })
    }

    pub const fn authority_class(&self) -> ScientificAuthorityClass {
        self.authority_class
    }

    pub fn source_inputs(&self) -> &[String] {
        &self.source_inputs
    }

    pub fn producer(&self) -> &str {
        &self.producer
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derived_classes_are_not_source_material_or_admission_candidates() {
        for class in [
            ScientificAuthorityClass::DerivedProjection,
            ScientificAuthorityClass::ModelInference,
            ScientificAuthorityClass::AdvisoryAssessment,
        ] {
            assert!(class.is_derived());
            assert!(!class.is_scientific_source_material());
            assert!(!class.eligible_for_governed_evidence_admission());
            assert!(!class.eligible_for_governed_attestation_admission());
        }
    }

    #[test]
    fn canonical_event_is_history_not_evidence() {
        let class = ScientificAuthorityClass::CanonicalEvent;
        assert!(!class.is_derived());
        assert!(!class.is_scientific_source_material());
        assert!(!class.eligible_for_governed_evidence_admission());
        assert!(!class.eligible_for_governed_attestation_admission());
    }

    #[test]
    fn evidence_and_attestation_admission_are_narrow() {
        assert!(ScientificAuthorityClass::ExternalEvidence.is_scientific_source_material());
        assert!(
            ScientificAuthorityClass::ExternalEvidence.eligible_for_governed_evidence_admission()
        );
        assert!(
            !ScientificAuthorityClass::ExternalEvidence
                .eligible_for_governed_attestation_admission()
        );

        assert!(ScientificAuthorityClass::HumanAttestation.is_scientific_source_material());
        assert!(
            ScientificAuthorityClass::HumanAttestation
                .eligible_for_governed_attestation_admission()
        );
        assert!(
            !ScientificAuthorityClass::HumanAttestation.eligible_for_governed_evidence_admission()
        );
    }

    #[test]
    fn derived_product_rejects_non_derived_authority_classes() {
        for class in [
            ScientificAuthorityClass::CanonicalEvent,
            ScientificAuthorityClass::ExternalEvidence,
            ScientificAuthorityClass::HumanAttestation,
        ] {
            assert!(DerivedScientificProduct::new(
                class,
                vec!["claim:1".to_string()],
                "test-producer-v1",
            )
            .is_err());
        }
    }

    #[test]
    fn derived_product_requires_canonical_provenance_and_producer() {
        assert!(DerivedScientificProduct::new(
            ScientificAuthorityClass::ModelInference,
            Vec::new(),
            "model-v1",
        )
        .is_err());

        assert!(DerivedScientificProduct::new(
            ScientificAuthorityClass::ModelInference,
            vec![" claim:1 ".to_string()],
            "model-v1",
        )
        .is_err());

        assert!(DerivedScientificProduct::new(
            ScientificAuthorityClass::DerivedProjection,
            vec!["claim:1".to_string()],
            " model-v1 ",
        )
        .is_err());

        assert!(DerivedScientificProduct::new(
            ScientificAuthorityClass::DerivedProjection,
            vec!["claim:1".to_string(), "claim:1".to_string()],
            "model-v1",
        )
        .is_err());
    }

    #[test]
    fn derived_product_canonicalizes_source_order() {
        let product = DerivedScientificProduct::new(
            ScientificAuthorityClass::AdvisoryAssessment,
            vec!["claim:z".to_string(), "artifact:a".to_string()],
            "review-assistant-v1",
        )
        .unwrap();

        assert_eq!(
            product.source_inputs(),
            &["artifact:a".to_string(), "claim:z".to_string()]
        );
        assert_eq!(product.producer(), "review-assistant-v1");
    }

    #[test]
    fn deserialization_cannot_bypass_authority_class_validation() {
        let json = r#"{
            "authority_class":"external_evidence",
            "source_inputs":["claim:1"],
            "producer":"model-v1"
        }"#;
        assert!(serde_json::from_str::<DerivedScientificProduct>(json).is_err());
    }

    #[test]
    fn valid_product_round_trips_through_serde() {
        let product = DerivedScientificProduct::new(
            ScientificAuthorityClass::ModelInference,
            vec!["claim:2".to_string(), "claim:1".to_string()],
            "reasoner-v3",
        )
        .unwrap();
        let json = serde_json::to_string(&product).unwrap();
        let round_trip: DerivedScientificProduct = serde_json::from_str(&json).unwrap();
        assert_eq!(round_trip, product);
        assert_eq!(round_trip.authority_class(), ScientificAuthorityClass::ModelInference);
    }
}
