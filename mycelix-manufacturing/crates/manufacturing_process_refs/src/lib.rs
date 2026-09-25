// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Migration-safe typed references for Mycelix manufacturing operations.
//!
//! This crate does not own manufacturing physics, process quantities, machine execution,
//! commercial offers, or capacity scheduling. It carries exact references to canonical
//! engineering subjects so existing stringly-typed manufacturing records can migrate without
//! changing historical DHT entry shapes in place.
//!
//! Core boundary:
//!
//! ```text
//! typed process reference
//! != referenced engineering subject resolved
//! != resource qualified
//! != resource available
//! != execution authorized
//! ```

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const MAX_TOKEN_BYTES: usize = 512;
const MAX_DIGEST_HEX_BYTES: usize = 512;

fn canonical_token(field: &str, value: &str) -> Result<(), String> {
    if value.is_empty() {
        return Err(format!("{field} must not be empty"));
    }
    if value.len() > MAX_TOKEN_BYTES {
        return Err(format!("{field} exceeds {MAX_TOKEN_BYTES} bytes"));
    }
    if value.trim() != value || value.chars().any(char::is_control) {
        return Err(format!("{field} must be canonical and contain no control characters"));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum DigestAlgorithmV1 {
    Blake3,
    Sha256,
    Other(String),
}

impl DigestAlgorithmV1 {
    fn validate(&self) -> Result<(), String> {
        if let Self::Other(name) = self {
            canonical_token("digest.algorithm.other", name)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct DigestRefV1 {
    pub algorithm: DigestAlgorithmV1,
    pub hex: String,
}

impl DigestRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.algorithm.validate()?;
        if self.hex.is_empty() || self.hex.len() > MAX_DIGEST_HEX_BYTES {
            return Err("digest hex must be non-empty and bounded".into());
        }
        if !self
            .hex
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
        {
            return Err("digest hex must be lowercase hexadecimal".into());
        }
        if matches!(self.algorithm, DigestAlgorithmV1::Blake3 | DigestAlgorithmV1::Sha256)
            && self.hex.len() != 64
        {
            return Err("BLAKE3/SHA-256 digest must contain exactly 64 hex characters".into());
        }
        Ok(())
    }
}

/// Exact external engineering subject reference.
///
/// The namespace identifies the owning semantic system. `subject_id` is not trusted merely
/// because it is well formed; composition layers must resolve it against the referenced system.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ExternalSubjectRefV1 {
    pub namespace: String,
    pub subject_id: String,
    pub semantic_version: String,
    pub content_digest: DigestRefV1,
}

impl ExternalSubjectRefV1 {
    pub fn validate(&self) -> Result<(), String> {
        canonical_token("subject.namespace", &self.namespace)?;
        canonical_token("subject.subject_id", &self.subject_id)?;
        canonical_token("subject.semantic_version", &self.semantic_version)?;
        self.content_digest.validate()
    }
}

macro_rules! semantic_ref_newtype {
    ($name:ident) => {
        #[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub ExternalSubjectRefV1);

        impl $name {
            pub fn validate(&self) -> Result<(), String> {
                self.0.validate()
            }
        }
    };
}

semantic_ref_newtype!(ProcessDefinitionRefV1);
semantic_ref_newtype!(ProcessRecipeRefV1);
semantic_ref_newtype!(CapabilityRequirementRefV1);
semantic_ref_newtype!(ProcessStateRefV1);
semantic_ref_newtype!(InspectionProfileRefV1);
semantic_ref_newtype!(CapabilityProfileRefV1);
semantic_ref_newtype!(ResourceSubjectRefV1);
semantic_ref_newtype!(ProviderSubjectRefV1);
semantic_ref_newtype!(SiteSubjectRefV1);
semantic_ref_newtype!(EvidenceSubjectRefV1);
semantic_ref_newtype!(ConfigurationProfileRefV1);

/// Legacy operation fields retained only for migration/navigation.
///
/// These strings never satisfy a typed process/capability requirement by themselves.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyOperationMetadataV1 {
    pub operation_name: Option<String>,
    pub machine_type: Option<String>,
    pub tooling: Option<String>,
}

impl LegacyOperationMetadataV1 {
    pub fn validate(&self) -> Result<(), String> {
        for (field, value) in [
            ("legacy.operation_name", self.operation_name.as_deref()),
            ("legacy.machine_type", self.machine_type.as_deref()),
            ("legacy.tooling", self.tooling.as_deref()),
        ] {
            if let Some(value) = value {
                canonical_token(field, value)?;
            }
        }
        Ok(())
    }
}

/// Migration-safe typed references for one manufacturing routing step.
///
/// Operational timing, scheduling, and resource assignment remain in Mycelix work-order/planning
/// records. This subject only binds the exact engineering semantics that the step intends.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TypedRoutingStepRefsV1 {
    pub sequence: u32,
    pub process_definition: ProcessDefinitionRefV1,
    pub recipe: Option<ProcessRecipeRefV1>,
    pub capability_requirement: Option<CapabilityRequirementRefV1>,
    #[serde(default)]
    pub input_states: Vec<ProcessStateRefV1>,
    #[serde(default)]
    pub output_states: Vec<ProcessStateRefV1>,
    #[serde(default)]
    pub inspection_profiles: Vec<InspectionProfileRefV1>,
    #[serde(default)]
    pub legacy_metadata: LegacyOperationMetadataV1,
}

impl TypedRoutingStepRefsV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.sequence == 0 {
            return Err("routing sequence must be positive".into());
        }
        self.process_definition.validate()?;
        if let Some(recipe) = &self.recipe {
            recipe.validate()?;
        }
        if let Some(requirement) = &self.capability_requirement {
            requirement.validate()?;
        }
        for state in &self.input_states {
            state.validate()?;
        }
        for state in &self.output_states {
            state.validate()?;
        }
        for profile in &self.inspection_profiles {
            profile.validate()?;
        }
        validate_unique_refs("input_states", &self.input_states)?;
        validate_unique_refs("output_states", &self.output_states)?;
        validate_unique_refs("inspection_profiles", &self.inspection_profiles)?;
        self.legacy_metadata.validate()
    }

    /// Whether this routing step has enough typed references to stop treating legacy labels as
    /// engineering authority. This does not mean any external subject has been resolved.
    pub fn is_typed_for_capability_matching(&self) -> bool {
        self.capability_requirement.is_some()
    }
}

fn validate_unique_refs<T>(field: &str, refs: &[T]) -> Result<(), String>
where
    T: Ord + Clone + std::fmt::Debug,
{
    let mut seen = BTreeSet::new();
    for reference in refs {
        if !seen.insert(reference.clone()) {
            return Err(format!("duplicate reference in {field}: {reference:?}"));
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ResourceOfferEvidenceClassV1 {
    Declared,
    ManufacturerSpecified,
    ObservedCapability,
    QualifiedUnderProfile,
    ProductionQualifiedUnderProfile,
}

/// Engineering capability claim published by a provider/resource.
///
/// Price, lead time, calendar capacity, queue depth, and contractual terms are deliberately absent
/// because they are operational/commercial state rather than engineering capability.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ManufacturingResourceOfferV1 {
    pub provider: ProviderSubjectRefV1,
    pub site: SiteSubjectRefV1,
    pub resource: ResourceSubjectRefV1,
    pub process_definition: ProcessDefinitionRefV1,
    pub capability_profile: CapabilityProfileRefV1,
    pub configuration_profile: Option<ConfigurationProfileRefV1>,
    pub evidence_class: ResourceOfferEvidenceClassV1,
    #[serde(default)]
    pub evidence_refs: Vec<EvidenceSubjectRefV1>,
    pub valid_from_unix_s: u64,
    pub valid_until_unix_s: u64,
    pub display_label: Option<String>,
}

impl ManufacturingResourceOfferV1 {
    pub fn validate(&self) -> Result<(), String> {
        self.provider.validate()?;
        self.site.validate()?;
        self.resource.validate()?;
        self.process_definition.validate()?;
        self.capability_profile.validate()?;
        if let Some(config) = &self.configuration_profile {
            config.validate()?;
        }
        for evidence in &self.evidence_refs {
            evidence.validate()?;
        }
        validate_unique_refs("resource_offer.evidence_refs", &self.evidence_refs)?;
        if self.evidence_refs.is_empty() {
            return Err("resource offer requires at least one evidence reference".into());
        }
        if self.valid_from_unix_s >= self.valid_until_unix_s {
            return Err("resource offer validity window must be ordered".into());
        }
        if let Some(label) = &self.display_label {
            canonical_token("resource_offer.display_label", label)?;
        }
        Ok(())
    }

    pub fn is_valid_at(&self, unix_s: u64) -> bool {
        unix_s >= self.valid_from_unix_s && unix_s < self.valid_until_unix_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(fill: char) -> DigestRefV1 {
        DigestRefV1 {
            algorithm: DigestAlgorithmV1::Blake3,
            hex: std::iter::repeat_n(fill, 64).collect(),
        }
    }

    fn subject(namespace: &str, id: &str, fill: char) -> ExternalSubjectRefV1 {
        ExternalSubjectRefV1 {
            namespace: namespace.into(),
            subject_id: id.into(),
            semantic_version: "1".into(),
            content_digest: digest(fill),
        }
    }

    #[test]
    fn typed_routing_refs_preserve_legacy_metadata_without_trusting_it() {
        let step = TypedRoutingStepRefsV1 {
            sequence: 1,
            process_definition: ProcessDefinitionRefV1(subject(
                "symthaea.mfg-proc",
                "cnc-milling-v1",
                'a',
            )),
            recipe: None,
            capability_requirement: Some(CapabilityRequirementRefV1(subject(
                "symthaea.mfg-capability",
                "req-1",
                'b',
            ))),
            input_states: vec![],
            output_states: vec![],
            inspection_profiles: vec![],
            legacy_metadata: LegacyOperationMetadataV1 {
                operation_name: Some("Mill profile".into()),
                machine_type: Some("CNC5Axis".into()),
                tooling: None,
            },
        };
        assert!(step.validate().is_ok());
        assert!(step.is_typed_for_capability_matching());
    }

    #[test]
    fn duplicate_typed_refs_fail_closed() {
        let state = ProcessStateRefV1(subject("symthaea.material-state", "stock", 'c'));
        let step = TypedRoutingStepRefsV1 {
            sequence: 1,
            process_definition: ProcessDefinitionRefV1(subject(
                "symthaea.mfg-proc",
                "mill",
                'a',
            )),
            recipe: None,
            capability_requirement: None,
            input_states: vec![state.clone(), state],
            output_states: vec![],
            inspection_profiles: vec![],
            legacy_metadata: LegacyOperationMetadataV1::default(),
        };
        assert!(step.validate().unwrap_err().contains("duplicate reference"));
    }

    #[test]
    fn malformed_nested_ref_fails_closed() {
        let malformed = ProcessStateRefV1(ExternalSubjectRefV1 {
            namespace: "symthaea.material-state".into(),
            subject_id: "stock".into(),
            semantic_version: "1".into(),
            content_digest: DigestRefV1 {
                algorithm: DigestAlgorithmV1::Sha256,
                hex: "ABC".into(),
            },
        });
        let step = TypedRoutingStepRefsV1 {
            sequence: 1,
            process_definition: ProcessDefinitionRefV1(subject(
                "symthaea.mfg-proc",
                "mill",
                'a',
            )),
            recipe: None,
            capability_requirement: None,
            input_states: vec![malformed],
            output_states: vec![],
            inspection_profiles: vec![],
            legacy_metadata: LegacyOperationMetadataV1::default(),
        };
        assert!(step.validate().is_err());
    }

    #[test]
    fn exact_digest_validation_rejects_uppercase_or_wrong_length() {
        let upper = DigestRefV1 {
            algorithm: DigestAlgorithmV1::Sha256,
            hex: "A".repeat(64),
        };
        assert!(upper.validate().is_err());
        let short = DigestRefV1 {
            algorithm: DigestAlgorithmV1::Blake3,
            hex: "a".repeat(63),
        };
        assert!(short.validate().is_err());
    }

    #[test]
    fn resource_offer_requires_evidence_and_validity() {
        let mut offer = ManufacturingResourceOfferV1 {
            provider: ProviderSubjectRefV1(subject("mycelix.provider", "provider-1", 'a')),
            site: SiteSubjectRefV1(subject("mycelix.site", "site-1", 'b')),
            resource: ResourceSubjectRefV1(subject("eng-catalog.resource", "mill-1", 'c')),
            process_definition: ProcessDefinitionRefV1(subject(
                "symthaea.mfg-proc",
                "cnc-milling-v1",
                'd',
            )),
            capability_profile: CapabilityProfileRefV1(subject(
                "symthaea.mfg-capability",
                "cap-1",
                'e',
            )),
            configuration_profile: None,
            evidence_class: ResourceOfferEvidenceClassV1::QualifiedUnderProfile,
            evidence_refs: vec![EvidenceSubjectRefV1(subject(
                "mycelix.evidence",
                "qual-receipt-1",
                'f',
            ))],
            valid_from_unix_s: 100,
            valid_until_unix_s: 200,
            display_label: Some("5-axis cell".into()),
        };
        assert!(offer.validate().is_ok());
        assert!(offer.is_valid_at(150));
        assert!(!offer.is_valid_at(200));

        offer.evidence_refs.clear();
        assert!(offer.validate().is_err());
    }

    #[test]
    fn serde_round_trip_preserves_exact_refs() {
        let reference = ProcessDefinitionRefV1(subject(
            "symthaea.mfg-proc",
            "material-extrusion-v1",
            'a',
        ));
        let json = serde_json::to_string(&reference).unwrap();
        let decoded: ProcessDefinitionRefV1 = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, reference);
    }
}
