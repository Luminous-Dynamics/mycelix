// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Circularity lifecycle-event profiles over the canonical Economic Reality Graph.
//!
//! ```text
//! ERG event bound to Circularity profile
//! != environmental benefit
//! != material recovery yield
//! != circularity certification
//! != regulatory compliance
//! ```

use mycelix_economic_reality_graph::{
    EconomicMaterialEventV1, ExactErgSubjectRefV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const PROFILE_DOMAIN: &str = "mycelix-circularity::event-profile-v1";
const BINDING_DOMAIN: &str = "mycelix-circularity::event-binding-v1";

#[derive(Debug, Error, PartialEq, Eq)]
pub enum CircularityError {
    #[error("{0} must be a canonical non-empty token")]
    NonCanonical(&'static str),
    #[error("duplicate external reference: {0}")]
    DuplicateExternalRef(String),
    #[error("profile requires an explicit chain-of-custody strategy")]
    MissingChainOfCustody,
    #[error("profile requires an exact material-flow account reference")]
    MissingMaterialFlowAccount,
    #[error("ERG event is bound to a different event-profile subject")]
    EventProfileMismatch,
    #[error("{0} requires at least one input resource")]
    MissingInput(&'static str),
    #[error("{0} requires at least one output resource")]
    MissingOutput(&'static str),
    #[error("ERG validation failed: {0}")]
    Erg(String),
}

fn canonical(field: &'static str, value: &str) -> Result<(), CircularityError> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(CircularityError::NonCanonical(field));
    }
    Ok(())
}

fn hash_field(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum CircularityEventKindV1 {
    Return,
    Service,
    Repair,
    Reuse,
    Resell,
    Refurbish,
    Remanufacture,
    HarvestParts,
    Recycle,
    RecoverMaterial,
    Dispose,
}

impl CircularityEventKindV1 {
    fn tag(self) -> &'static str {
        match self {
            Self::Return => "return",
            Self::Service => "service",
            Self::Repair => "repair",
            Self::Reuse => "reuse",
            Self::Resell => "resell",
            Self::Refurbish => "refurbish",
            Self::Remanufacture => "remanufacture",
            Self::HarvestParts => "harvest-parts",
            Self::Recycle => "recycle",
            Self::RecoverMaterial => "recover-material",
            Self::Dispose => "dispose",
        }
    }

    fn requires_material_account(self) -> bool {
        matches!(
            self,
            Self::HarvestParts | Self::Recycle | Self::RecoverMaterial
        )
    }

    fn requires_input(self) -> bool {
        true
    }

    fn requires_output(self) -> bool {
        !matches!(self, Self::Dispose)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum CircularityDispositionV1 {
    ReturnedForAssessment,
    ReusedAsIs,
    RepairedAndReturnedToUse,
    Refurbished,
    Remanufactured,
    ComponentHarvested,
    MaterialRecycled,
    MaterialRecovered,
    Downcycled,
    Disposed,
    UnknownDisposition,
}

impl CircularityDispositionV1 {
    fn tag(self) -> &'static str {
        match self {
            Self::ReturnedForAssessment => "returned-for-assessment",
            Self::ReusedAsIs => "reused-as-is",
            Self::RepairedAndReturnedToUse => "repaired-and-returned-to-use",
            Self::Refurbished => "refurbished",
            Self::Remanufactured => "remanufactured",
            Self::ComponentHarvested => "component-harvested",
            Self::MaterialRecycled => "material-recycled",
            Self::MaterialRecovered => "material-recovered",
            Self::Downcycled => "downcycled",
            Self::Disposed => "disposed",
            Self::UnknownDisposition => "unknown-disposition",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ChainOfCustodyStrategyV1 {
    IdentityPreserved,
    Segregated,
    ControlledBlending,
    MassBalance,
    BookAndClaim,
}

impl ChainOfCustodyStrategyV1 {
    fn tag(self) -> &'static str {
        match self {
            Self::IdentityPreserved => "identity-preserved",
            Self::Segregated => "segregated",
            Self::ControlledBlending => "controlled-blending",
            Self::MassBalance => "mass-balance",
            Self::BookAndClaim => "book-and-claim",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CircularityAuthorityCeilingV1 {
    LifecycleEventProfileAndReferencesOnly,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CircularityEventProfileV1 {
    pub semantic_version: String,
    pub event_kind: CircularityEventKindV1,
    pub disposition: CircularityDispositionV1,
    pub chain_of_custody: Option<ChainOfCustodyStrategyV1>,
    pub material_flow_account_ref: Option<ExactErgSubjectRefV1>,
    #[serde(default)]
    pub external_requirement_refs: Vec<ExactErgSubjectRefV1>,
    pub authority_ceiling: CircularityAuthorityCeilingV1,
    pub display_label: Option<String>,
}

impl CircularityEventProfileV1 {
    pub fn validate(&self) -> Result<(), CircularityError> {
        canonical("profile.semantic_version", &self.semantic_version)?;
        if self.event_kind.requires_material_account() {
            if self.chain_of_custody.is_none() {
                return Err(CircularityError::MissingChainOfCustody);
            }
            if self.material_flow_account_ref.is_none() {
                return Err(CircularityError::MissingMaterialFlowAccount);
            }
        }
        if let Some(reference) = &self.material_flow_account_ref {
            reference
                .validate()
                .map_err(|err| CircularityError::Erg(err.to_string()))?;
        }
        validate_unique_external_refs(&self.external_requirement_refs)?;
        if let Some(label) = &self.display_label {
            canonical("profile.display_label", label)?;
        }
        Ok(())
    }

    pub fn profile_id(&self) -> Result<String, CircularityError> {
        self.validate()?;
        let mut requirements = self.external_requirement_refs.clone();
        requirements.sort();

        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, PROFILE_DOMAIN);
        hash_field(&mut hasher, &self.semantic_version);
        hash_field(&mut hasher, self.event_kind.tag());
        hash_field(&mut hasher, self.disposition.tag());
        hash_field(
            &mut hasher,
            self.chain_of_custody
                .map(ChainOfCustodyStrategyV1::tag)
                .unwrap_or("none"),
        );
        hash_field(&mut hasher, "material-flow-account");
        if let Some(reference) = &self.material_flow_account_ref {
            hash_field(
                &mut hasher,
                &reference
                    .ref_id()
                    .map_err(|err| CircularityError::Erg(err.to_string()))?,
            );
        }
        hash_field(&mut hasher, "external-requirements");
        for reference in requirements {
            hash_field(
                &mut hasher,
                &reference
                    .ref_id()
                    .map_err(|err| CircularityError::Erg(err.to_string()))?,
            );
        }
        hash_field(
            &mut hasher,
            "authority:lifecycle-event-profile-and-references-only",
        );
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn exact_ref(&self) -> Result<ExactErgSubjectRefV1, CircularityError> {
        let id = self.profile_id()?;
        Ok(ExactErgSubjectRefV1 {
            namespace: "mycelix.circularity.event-profile".into(),
            subject_id: id.clone(),
            semantic_version: self.semantic_version.clone(),
            content_blake3: id,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CircularityEventBindingV1 {
    pub profile: CircularityEventProfileV1,
    pub event: EconomicMaterialEventV1,
}

impl CircularityEventBindingV1 {
    pub fn validate(&self) -> Result<(), CircularityError> {
        self.profile.validate()?;
        self.event
            .validate()
            .map_err(|err| CircularityError::Erg(err.to_string()))?;
        if self.event.event_profile_ref != self.profile.exact_ref()? {
            return Err(CircularityError::EventProfileMismatch);
        }
        if self.profile.event_kind.requires_input() && self.event.inputs.is_empty() {
            return Err(CircularityError::MissingInput(self.profile.event_kind.tag()));
        }
        if self.profile.event_kind.requires_output() && self.event.outputs.is_empty() {
            return Err(CircularityError::MissingOutput(self.profile.event_kind.tag()));
        }
        Ok(())
    }

    pub fn binding_id(&self) -> Result<String, CircularityError> {
        self.validate()?;
        let event_id = self
            .event
            .event_id()
            .map_err(|err| CircularityError::Erg(err.to_string()))?;
        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, BINDING_DOMAIN);
        hash_field(&mut hasher, &self.profile.profile_id()?);
        hash_field(&mut hasher, &event_id.0);
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn claims_environmental_benefit(&self) -> bool {
        false
    }

    pub fn claims_regulatory_compliance(&self) -> bool {
        false
    }
}

fn validate_unique_external_refs(refs: &[ExactErgSubjectRefV1]) -> Result<(), CircularityError> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        reference
            .validate()
            .map_err(|err| CircularityError::Erg(err.to_string()))?;
        let id = reference
            .ref_id()
            .map_err(|err| CircularityError::Erg(err.to_string()))?;
        if !seen.insert(id.clone()) {
            return Err(CircularityError::DuplicateExternalRef(id));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_economic_reality_graph::{
        EconomicResourceSubjectV1, ErgAuthorityCeilingV1, EventResourceEdgeV1,
        EventResourceRoleV1,
    };

    fn exact(namespace: &str, id: &str, fill: char) -> ExactErgSubjectRefV1 {
        ExactErgSubjectRefV1 {
            namespace: namespace.into(),
            subject_id: id.into(),
            semantic_version: "1".into(),
            content_blake3: std::iter::repeat_n(fill, 64).collect(),
        }
    }

    fn resource(id: &str, fill: char) -> ExactErgSubjectRefV1 {
        EconomicResourceSubjectV1 {
            semantic_version: "1".into(),
            resource_profile_ref: exact("mycelix.erg-profile", "article", fill),
            external_state_refs: vec![exact("symthaea.material-state", id, fill)],
            authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
            display_label: Some(id.into()),
        }
        .exact_ref()
        .unwrap()
    }

    fn repair_profile() -> CircularityEventProfileV1 {
        CircularityEventProfileV1 {
            semantic_version: "1".into(),
            event_kind: CircularityEventKindV1::Repair,
            disposition: CircularityDispositionV1::RepairedAndReturnedToUse,
            chain_of_custody: Some(ChainOfCustodyStrategyV1::IdentityPreserved),
            material_flow_account_ref: None,
            external_requirement_refs: vec![exact("symthaea.lifecycle", "repair-plan", 'a')],
            authority_ceiling:
                CircularityAuthorityCeilingV1::LifecycleEventProfileAndReferencesOnly,
            display_label: Some("Repair profile".into()),
        }
    }

    fn event_for(profile: &CircularityEventProfileV1) -> EconomicMaterialEventV1 {
        EconomicMaterialEventV1 {
            semantic_version: "1".into(),
            event_profile_ref: profile.exact_ref().unwrap(),
            inputs: vec![EventResourceEdgeV1 {
                role: EventResourceRoleV1::Transformed,
                resource_ref: resource("before", 'b'),
            }],
            outputs: vec![EventResourceEdgeV1 {
                role: EventResourceRoleV1::Transformed,
                resource_ref: resource("after", 'c'),
            }],
            party_refs: vec![exact("mycelix.party", "repairer", 'd')],
            site_refs: vec![exact("mycelix.site", "repair-site", 'e')],
            effective_time_ref: exact("mycelix.time", "t1", 'f'),
            process_or_agreement_refs: vec![exact("symthaea.mfg-plan", "repair-plan", '1')],
            evidence_refs: vec![exact("field.evidence", "post-repair-inspection", '2')],
            revision_links: vec![],
            authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
            display_label: Some("Repair event".into()),
        }
    }

    #[test]
    fn valid_repair_binding_preserves_event_authority_ceiling() {
        let profile = repair_profile();
        let binding = CircularityEventBindingV1 {
            event: event_for(&profile),
            profile,
        };
        assert!(binding.validate().is_ok());
        assert!(!binding.claims_environmental_benefit());
        assert!(!binding.claims_regulatory_compliance());
    }

    #[test]
    fn wrong_event_profile_rejects() {
        let profile = repair_profile();
        let mut event = event_for(&profile);
        event.event_profile_ref = exact("mycelix.circularity.event-profile", "wrong", '9');
        let binding = CircularityEventBindingV1 { profile, event };
        assert_eq!(binding.validate(), Err(CircularityError::EventProfileMismatch));
    }

    #[test]
    fn recycle_profile_requires_custody_and_material_flow_account() {
        let mut profile = repair_profile();
        profile.event_kind = CircularityEventKindV1::Recycle;
        profile.disposition = CircularityDispositionV1::MaterialRecycled;
        profile.chain_of_custody = None;
        assert_eq!(profile.validate(), Err(CircularityError::MissingChainOfCustody));

        profile.chain_of_custody = Some(ChainOfCustodyStrategyV1::MassBalance);
        assert_eq!(
            profile.validate(),
            Err(CircularityError::MissingMaterialFlowAccount)
        );
    }

    #[test]
    fn dispose_requires_input_but_not_output() {
        let profile = CircularityEventProfileV1 {
            semantic_version: "1".into(),
            event_kind: CircularityEventKindV1::Dispose,
            disposition: CircularityDispositionV1::Disposed,
            chain_of_custody: None,
            material_flow_account_ref: None,
            external_requirement_refs: vec![],
            authority_ceiling:
                CircularityAuthorityCeilingV1::LifecycleEventProfileAndReferencesOnly,
            display_label: None,
        };
        let mut event = event_for(&profile);
        event.outputs.clear();
        assert!(CircularityEventBindingV1 {
            profile: profile.clone(),
            event: event.clone(),
        }
        .validate()
        .is_ok());

        event.inputs.clear();
        assert_eq!(
            CircularityEventBindingV1 { profile, event }.validate(),
            Err(CircularityError::Erg(
                "economic/material event requires at least one input or output resource edge".into()
            ))
        );
    }

    #[test]
    fn disposition_or_custody_change_changes_profile_identity() {
        let a = repair_profile();
        let mut b = a.clone();
        b.disposition = CircularityDispositionV1::Refurbished;
        assert_ne!(a.profile_id().unwrap(), b.profile_id().unwrap());

        let mut c = a.clone();
        c.chain_of_custody = Some(ChainOfCustodyStrategyV1::Segregated);
        assert_ne!(a.profile_id().unwrap(), c.profile_id().unwrap());
    }

    #[test]
    fn display_label_does_not_change_profile_identity() {
        let a = repair_profile();
        let mut b = a.clone();
        b.display_label = Some("renamed profile".into());
        assert_eq!(a.profile_id().unwrap(), b.profile_id().unwrap());
    }

    #[test]
    fn serialization_preserves_binding_identity() {
        let profile = repair_profile();
        let binding = CircularityEventBindingV1 {
            event: event_for(&profile),
            profile,
        };
        let encoded = serde_json::to_string(&binding).unwrap();
        let decoded: CircularityEventBindingV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(binding.binding_id().unwrap(), decoded.binding_id().unwrap());
    }
}
