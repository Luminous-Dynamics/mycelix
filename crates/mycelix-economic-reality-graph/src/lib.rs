// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical resource/event substrate for the Mycelix Economic Reality Graph.
//!
//! The core owns deterministic event/resource identity and graph relationships only.
//!
//! ```text
//! canonical event
//! != claim about that event
//! != physical measurement
//! != legal/compliance decision
//! ```

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

const REF_DOMAIN: &str = "mycelix-economic-reality-graph::exact-subject-ref-v1";
const RESOURCE_DOMAIN: &str = "mycelix-economic-reality-graph::resource-subject-v1";
const EVENT_DOMAIN: &str = "mycelix-economic-reality-graph::event-subject-v1";

#[derive(Debug, Error, PartialEq, Eq)]
pub enum ErgError {
    #[error("{field} must be a canonical non-empty token")]
    NonCanonical { field: &'static str },
    #[error("content digest must be exactly 64 lowercase hexadecimal characters")]
    InvalidDigest,
    #[error("duplicate reference in {field}: {value}")]
    DuplicateReference { field: &'static str, value: String },
    #[error("duplicate event resource edge: {0}")]
    DuplicateResourceEdge(String),
    #[error("economic/material event requires at least one input or output resource edge")]
    EmptyEventResources,
}

fn canonical(field: &'static str, value: &str) -> Result<(), ErgError> {
    if value.is_empty() || value.trim() != value || value.chars().any(char::is_control) {
        return Err(ErgError::NonCanonical { field });
    }
    Ok(())
}

fn digest64(value: &str) -> Result<(), ErgError> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    {
        return Err(ErgError::InvalidDigest);
    }
    Ok(())
}

fn hash_field(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ExactErgSubjectRefV1 {
    pub namespace: String,
    pub subject_id: String,
    pub semantic_version: String,
    pub content_blake3: String,
}

impl ExactErgSubjectRefV1 {
    pub fn validate(&self) -> Result<(), ErgError> {
        canonical("subject.namespace", &self.namespace)?;
        canonical("subject.subject_id", &self.subject_id)?;
        canonical("subject.semantic_version", &self.semantic_version)?;
        digest64(&self.content_blake3)
    }

    pub fn ref_id(&self) -> Result<String, ErgError> {
        self.validate()?;
        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, REF_DOMAIN);
        hash_field(&mut hasher, &self.namespace);
        hash_field(&mut hasher, &self.subject_id);
        hash_field(&mut hasher, &self.semantic_version);
        hash_field(&mut hasher, &self.content_blake3);
        Ok(hasher.finalize().to_hex().to_string())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ResourceSubjectId(pub String);

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct EconomicMaterialEventId(pub String);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ErgAuthorityCeilingV1 {
    ResourceEventGraphAndReferencesOnly,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicResourceSubjectV1 {
    pub semantic_version: String,
    pub resource_profile_ref: ExactErgSubjectRefV1,
    #[serde(default)]
    pub external_state_refs: Vec<ExactErgSubjectRefV1>,
    pub authority_ceiling: ErgAuthorityCeilingV1,
    pub display_label: Option<String>,
}

impl EconomicResourceSubjectV1 {
    pub fn validate(&self) -> Result<(), ErgError> {
        canonical("resource.semantic_version", &self.semantic_version)?;
        self.resource_profile_ref.validate()?;
        validate_unique_refs("resource.external_state_refs", &self.external_state_refs)?;
        if let Some(label) = &self.display_label {
            canonical("resource.display_label", label)?;
        }
        Ok(())
    }

    pub fn resource_id(&self) -> Result<ResourceSubjectId, ErgError> {
        self.validate()?;
        let mut states = self.external_state_refs.clone();
        states.sort();

        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, RESOURCE_DOMAIN);
        hash_field(&mut hasher, &self.semantic_version);
        hash_field(&mut hasher, &self.resource_profile_ref.ref_id()?);
        hash_field(&mut hasher, "external-states");
        for state in states {
            hash_field(&mut hasher, &state.ref_id()?);
        }
        hash_field(&mut hasher, "authority:resource-event-graph-and-references-only");
        Ok(ResourceSubjectId(hasher.finalize().to_hex().to_string()))
    }

    pub fn exact_ref(&self) -> Result<ExactErgSubjectRefV1, ErgError> {
        let id = self.resource_id()?.0;
        Ok(ExactErgSubjectRefV1 {
            namespace: "mycelix.erg.resource".into(),
            subject_id: id.clone(),
            semantic_version: self.semantic_version.clone(),
            content_blake3: id,
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EventResourceRoleV1 {
    Consumed,
    Transformed,
    Combined,
    SplitFrom,
    ComponentOf,
    RecoveredFrom,
    Transferred,
    CustodyInput,
    CustodyOutput,
    ReferenceOnly,
    Custom(String),
}

impl EventResourceRoleV1 {
    fn validate(&self) -> Result<(), ErgError> {
        if let Self::Custom(value) = self {
            canonical("event_resource_role.custom", value)?;
        }
        Ok(())
    }

    fn tag(&self) -> String {
        match self {
            Self::Consumed => "consumed".into(),
            Self::Transformed => "transformed".into(),
            Self::Combined => "combined".into(),
            Self::SplitFrom => "split-from".into(),
            Self::ComponentOf => "component-of".into(),
            Self::RecoveredFrom => "recovered-from".into(),
            Self::Transferred => "transferred".into(),
            Self::CustodyInput => "custody-input".into(),
            Self::CustodyOutput => "custody-output".into(),
            Self::ReferenceOnly => "reference-only".into(),
            Self::Custom(value) => format!("custom:{value}"),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventResourceEdgeV1 {
    pub role: EventResourceRoleV1,
    pub resource_ref: ExactErgSubjectRefV1,
}

impl EventResourceEdgeV1 {
    pub fn validate(&self) -> Result<(), ErgError> {
        self.role.validate()?;
        self.resource_ref.validate()
    }

    fn identity_key(&self) -> Result<String, ErgError> {
        Ok(format!("{}:{}", self.role.tag(), self.resource_ref.ref_id()?))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum EventRevisionKindV1 {
    CorrectionOf,
    Supersedes,
    RetractionOf,
}

impl EventRevisionKindV1 {
    fn tag(self) -> &'static str {
        match self {
            Self::CorrectionOf => "correction-of",
            Self::Supersedes => "supersedes",
            Self::RetractionOf => "retraction-of",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EventRevisionLinkV1 {
    pub kind: EventRevisionKindV1,
    pub prior_event_ref: ExactErgSubjectRefV1,
}

impl EventRevisionLinkV1 {
    pub fn validate(&self) -> Result<(), ErgError> {
        self.prior_event_ref.validate()
    }

    fn identity_key(&self) -> Result<String, ErgError> {
        Ok(format!("{}:{}", self.kind.tag(), self.prior_event_ref.ref_id()?))
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EconomicMaterialEventV1 {
    pub semantic_version: String,
    pub event_profile_ref: ExactErgSubjectRefV1,
    #[serde(default)]
    pub inputs: Vec<EventResourceEdgeV1>,
    #[serde(default)]
    pub outputs: Vec<EventResourceEdgeV1>,
    #[serde(default)]
    pub party_refs: Vec<ExactErgSubjectRefV1>,
    #[serde(default)]
    pub site_refs: Vec<ExactErgSubjectRefV1>,
    pub effective_time_ref: ExactErgSubjectRefV1,
    #[serde(default)]
    pub process_or_agreement_refs: Vec<ExactErgSubjectRefV1>,
    #[serde(default)]
    pub evidence_refs: Vec<ExactErgSubjectRefV1>,
    #[serde(default)]
    pub revision_links: Vec<EventRevisionLinkV1>,
    pub authority_ceiling: ErgAuthorityCeilingV1,
    pub display_label: Option<String>,
}

impl EconomicMaterialEventV1 {
    pub fn validate(&self) -> Result<(), ErgError> {
        canonical("event.semantic_version", &self.semantic_version)?;
        self.event_profile_ref.validate()?;
        self.effective_time_ref.validate()?;
        if self.inputs.is_empty() && self.outputs.is_empty() {
            return Err(ErgError::EmptyEventResources);
        }
        validate_resource_edges("event.inputs", &self.inputs)?;
        validate_resource_edges("event.outputs", &self.outputs)?;
        validate_unique_refs("event.party_refs", &self.party_refs)?;
        validate_unique_refs("event.site_refs", &self.site_refs)?;
        validate_unique_refs(
            "event.process_or_agreement_refs",
            &self.process_or_agreement_refs,
        )?;
        validate_unique_refs("event.evidence_refs", &self.evidence_refs)?;
        validate_revision_links(&self.revision_links)?;
        if let Some(label) = &self.display_label {
            canonical("event.display_label", label)?;
        }
        Ok(())
    }

    pub fn event_id(&self) -> Result<EconomicMaterialEventId, ErgError> {
        self.validate()?;

        let mut inputs = self
            .inputs
            .iter()
            .map(EventResourceEdgeV1::identity_key)
            .collect::<Result<Vec<_>, _>>()?;
        inputs.sort();
        let mut outputs = self
            .outputs
            .iter()
            .map(EventResourceEdgeV1::identity_key)
            .collect::<Result<Vec<_>, _>>()?;
        outputs.sort();
        let mut parties = ref_ids(&self.party_refs)?;
        parties.sort();
        let mut sites = ref_ids(&self.site_refs)?;
        sites.sort();
        let mut process_refs = ref_ids(&self.process_or_agreement_refs)?;
        process_refs.sort();
        let mut evidence = ref_ids(&self.evidence_refs)?;
        evidence.sort();
        let mut revisions = self
            .revision_links
            .iter()
            .map(EventRevisionLinkV1::identity_key)
            .collect::<Result<Vec<_>, _>>()?;
        revisions.sort();

        let mut hasher = blake3::Hasher::new();
        hash_field(&mut hasher, EVENT_DOMAIN);
        hash_field(&mut hasher, &self.semantic_version);
        hash_field(&mut hasher, &self.event_profile_ref.ref_id()?);
        hash_list(&mut hasher, "inputs", &inputs);
        hash_list(&mut hasher, "outputs", &outputs);
        hash_list(&mut hasher, "parties", &parties);
        hash_list(&mut hasher, "sites", &sites);
        hash_field(&mut hasher, "effective-time");
        hash_field(&mut hasher, &self.effective_time_ref.ref_id()?);
        hash_list(&mut hasher, "process-or-agreement", &process_refs);
        hash_list(&mut hasher, "evidence", &evidence);
        hash_list(&mut hasher, "revision-links", &revisions);
        hash_field(&mut hasher, "authority:resource-event-graph-and-references-only");
        Ok(EconomicMaterialEventId(
            hasher.finalize().to_hex().to_string(),
        ))
    }

    pub fn exact_ref(&self) -> Result<ExactErgSubjectRefV1, ErgError> {
        let id = self.event_id()?.0;
        Ok(ExactErgSubjectRefV1 {
            namespace: "mycelix.erg.event".into(),
            subject_id: id.clone(),
            semantic_version: self.semantic_version.clone(),
            content_blake3: id,
        })
    }
}

fn validate_unique_refs(
    field: &'static str,
    refs: &[ExactErgSubjectRefV1],
) -> Result<(), ErgError> {
    let mut seen = BTreeSet::new();
    for reference in refs {
        reference.validate()?;
        let key = reference.ref_id()?;
        if !seen.insert(key.clone()) {
            return Err(ErgError::DuplicateReference { field, value: key });
        }
    }
    Ok(())
}

fn validate_resource_edges(
    field: &'static str,
    edges: &[EventResourceEdgeV1],
) -> Result<(), ErgError> {
    let mut seen = BTreeSet::new();
    for edge in edges {
        edge.validate()?;
        let key = edge.identity_key()?;
        if !seen.insert(key.clone()) {
            return Err(ErgError::DuplicateResourceEdge(format!("{field}:{key}")));
        }
    }
    Ok(())
}

fn validate_revision_links(links: &[EventRevisionLinkV1]) -> Result<(), ErgError> {
    let mut seen = BTreeSet::new();
    for link in links {
        link.validate()?;
        let key = link.identity_key()?;
        if !seen.insert(key.clone()) {
            return Err(ErgError::DuplicateReference {
                field: "event.revision_links",
                value: key,
            });
        }
    }
    Ok(())
}

fn ref_ids(refs: &[ExactErgSubjectRefV1]) -> Result<Vec<String>, ErgError> {
    refs.iter().map(ExactErgSubjectRefV1::ref_id).collect()
}

fn hash_list(hasher: &mut blake3::Hasher, label: &str, values: &[String]) {
    hash_field(hasher, label);
    for value in values {
        hash_field(hasher, value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exact(namespace: &str, id: &str, fill: char) -> ExactErgSubjectRefV1 {
        ExactErgSubjectRefV1 {
            namespace: namespace.into(),
            subject_id: id.into(),
            semantic_version: "1".into(),
            content_blake3: std::iter::repeat_n(fill, 64).collect(),
        }
    }

    fn resource() -> EconomicResourceSubjectV1 {
        EconomicResourceSubjectV1 {
            semantic_version: "1".into(),
            resource_profile_ref: exact("mycelix.erg-profile", "manufactured-article", 'a'),
            external_state_refs: vec![exact("symthaea.material-state", "article-state-a", 'b')],
            authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
            display_label: Some("Article A".into()),
        }
    }

    fn event() -> EconomicMaterialEventV1 {
        let input = resource().exact_ref().unwrap();
        let mut output_resource = resource();
        output_resource.external_state_refs = vec![exact(
            "symthaea.material-state",
            "article-state-serviced",
            'c',
        )];
        let output = output_resource.exact_ref().unwrap();

        EconomicMaterialEventV1 {
            semantic_version: "1".into(),
            event_profile_ref: exact("mycelix.erg-event-profile", "service", 'd'),
            inputs: vec![EventResourceEdgeV1 {
                role: EventResourceRoleV1::Transformed,
                resource_ref: input,
            }],
            outputs: vec![EventResourceEdgeV1 {
                role: EventResourceRoleV1::Transformed,
                resource_ref: output,
            }],
            party_refs: vec![exact("mycelix.party", "service-provider", 'e')],
            site_refs: vec![exact("mycelix.site", "service-site", 'f')],
            effective_time_ref: exact("mycelix.time", "time-subject", '1'),
            process_or_agreement_refs: vec![exact("symthaea.mfg-plan", "repair-plan", '2')],
            evidence_refs: vec![exact("field.evidence", "inspection-receipt", '3')],
            revision_links: vec![],
            authority_ceiling: ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly,
            display_label: Some("Service event".into()),
        }
    }

    #[test]
    fn display_metadata_does_not_change_resource_or_event_identity() {
        let a = resource();
        let mut b = a.clone();
        b.display_label = Some("renamed resource".into());
        assert_eq!(a.resource_id().unwrap(), b.resource_id().unwrap());

        let e1 = event();
        let mut e2 = e1.clone();
        e2.display_label = Some("renamed event".into());
        assert_eq!(e1.event_id().unwrap(), e2.event_id().unwrap());
    }

    #[test]
    fn resource_state_change_changes_resource_identity() {
        let a = resource();
        let mut b = a.clone();
        b.external_state_refs = vec![exact("symthaea.material-state", "different-state", '9')];
        assert_ne!(a.resource_id().unwrap(), b.resource_id().unwrap());
    }

    #[test]
    fn event_collection_order_is_canonicalized() {
        let mut a = event();
        a.evidence_refs.push(exact("field.evidence", "second", '4'));
        let mut b = a.clone();
        b.evidence_refs.reverse();
        assert_eq!(a.event_id().unwrap(), b.event_id().unwrap());
    }

    #[test]
    fn duplicate_resource_edge_rejects() {
        let mut e = event();
        e.inputs.push(e.inputs[0].clone());
        assert!(matches!(
            e.validate(),
            Err(ErgError::DuplicateResourceEdge(_))
        ));
    }

    #[test]
    fn empty_resource_event_rejects() {
        let mut e = event();
        e.inputs.clear();
        e.outputs.clear();
        assert_eq!(e.validate(), Err(ErgError::EmptyEventResources));
    }

    #[test]
    fn correction_is_new_event_and_preserves_prior_ref() {
        let original = event();
        let original_ref = original.exact_ref().unwrap();
        let mut correction = event();
        correction.revision_links.push(EventRevisionLinkV1 {
            kind: EventRevisionKindV1::CorrectionOf,
            prior_event_ref: original_ref.clone(),
        });
        assert_ne!(original.event_id().unwrap(), correction.event_id().unwrap());
        assert_eq!(
            correction.revision_links[0].prior_event_ref,
            original_ref
        );
    }

    #[test]
    fn malformed_external_ref_fails_closed() {
        let mut e = event();
        e.evidence_refs[0].content_blake3 = "BAD".into();
        assert_eq!(e.validate(), Err(ErgError::InvalidDigest));
    }

    #[test]
    fn authority_ceiling_contains_no_claim_or_compliance_authority() {
        assert_eq!(
            event().authority_ceiling,
            ErgAuthorityCeilingV1::ResourceEventGraphAndReferencesOnly
        );
    }

    #[test]
    fn serde_round_trip_preserves_exact_event_identity() {
        let e = event();
        let encoded = serde_json::to_string(&e).unwrap();
        let decoded: EconomicMaterialEventV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(e.event_id().unwrap(), decoded.event_id().unwrap());
    }
}
