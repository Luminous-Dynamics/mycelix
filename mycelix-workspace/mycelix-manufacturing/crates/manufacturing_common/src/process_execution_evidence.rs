//! Additive links from manufacturing execution to the existing Economic Reality Graph.
//!
//! This bundle stores references only. It does not duplicate resource/event/claim payloads,
//! resolve source objects, authenticate publishers, establish process qualification, or authorize
//! execution. Those semantics remain with their owning graph and domain validators.

use crate::process_registry::{
    ContentDigestV1, IndustrialArtifactKindV1, ProcessReferenceValidationError,
    TypedRoutingStepRefV1,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const PROCESS_EXECUTION_EVIDENCE_SCHEMA_V1: &str = "mycelix-process-execution-evidence/1";

/// The kind of source graph object targeted by an opaque, revision-bound reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GraphEntityKindV1 {
    Resource,
    Event,
    Claim,
}

/// An opaque reference into an externally owned event/resource graph.
///
/// `graph_system`, `entity_ref`, `revision`, and `content_digest` identify the source
/// object; they do not assert that the object exists or is authentic.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GraphEntityRefV1 {
    pub graph_system: String,
    pub entity_kind: GraphEntityKindV1,
    pub entity_ref: String,
    pub revision: String,
    pub content_digest: ContentDigestV1,
}

impl GraphEntityRefV1 {
    pub fn validate(&self, path: &str) -> Result<(), ProcessReferenceValidationError> {
        for (field, value) in [
            ("graph_system", self.graph_system.as_str()),
            ("entity_ref", self.entity_ref.as_str()),
            ("revision", self.revision.as_str()),
        ] {
            validate_token(value, &format!("{path}.{field}"))?;
        }
        self.content_digest
            .validate(&format!("{path}.content_digest"))
    }

    fn require_kind(
        &self,
        expected: GraphEntityKindV1,
        path: &str,
    ) -> Result<(), ProcessReferenceValidationError> {
        if self.entity_kind != expected {
            return Err(ProcessReferenceValidationError::new(
                "graph_ref.wrong_kind",
                format!("{path}: expected {expected:?}, got {:?}", self.entity_kind),
            ));
        }
        Ok(())
    }

    fn identity_key(&self) -> (String, GraphEntityKindV1, String) {
        (
            self.graph_system.clone(),
            self.entity_kind,
            self.entity_ref.clone(),
        )
    }
}

/// Sidecar which binds a process execution event to one exact typed routing snapshot,
/// the resource objects consumed/produced, supporting events, and any conformity claims.
///
/// The referenced event graph remains authoritative for event payloads, physical measurements,
/// claim lifecycle, corrections, chain of custody, and evidence provenance.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProcessExecutionEvidenceBundleV1 {
    pub schema_version: String,
    pub execution_event_ref: GraphEntityRefV1,
    pub routing_step_ref: TypedRoutingStepRefV1,
    pub input_resource_refs: Vec<GraphEntityRefV1>,
    pub output_resource_refs: Vec<GraphEntityRefV1>,
    pub supporting_event_refs: Vec<GraphEntityRefV1>,
    pub conformity_claim_refs: Vec<GraphEntityRefV1>,
}

impl ProcessExecutionEvidenceBundleV1 {
    pub fn validate(&self) -> Result<(), ProcessReferenceValidationError> {
        if self.schema_version != PROCESS_EXECUTION_EVIDENCE_SCHEMA_V1 {
            return Err(ProcessReferenceValidationError::new(
                "execution_bundle.unsupported_schema",
                "schema_version",
            ));
        }

        self.execution_event_ref.validate("execution_event_ref")?;
        self.execution_event_ref
            .require_kind(GraphEntityKindV1::Event, "execution_event_ref")?;
        self.routing_step_ref.validate()?;

        validate_ref_group(
            &self.input_resource_refs,
            GraphEntityKindV1::Resource,
            "input_resource_refs",
        )?;
        validate_ref_group(
            &self.output_resource_refs,
            GraphEntityKindV1::Resource,
            "output_resource_refs",
        )?;
        validate_ref_group(
            &self.supporting_event_refs,
            GraphEntityKindV1::Event,
            "supporting_event_refs",
        )?;
        validate_ref_group(
            &self.conformity_claim_refs,
            GraphEntityKindV1::Claim,
            "conformity_claim_refs",
        )?;

        if self.routing_step_ref.process_definition_ref.kind
            != IndustrialArtifactKindV1::ProcessDefinition
        {
            // Defensive cross-contract guard; routing_step_ref.validate should already enforce this.
            return Err(ProcessReferenceValidationError::new(
                "execution_bundle.invalid_process_ref",
                "routing_step_ref.process_definition_ref.kind",
            ));
        }
        Ok(())
    }
}

fn validate_ref_group(
    refs: &[GraphEntityRefV1],
    expected_kind: GraphEntityKindV1,
    path: &str,
) -> Result<(), ProcessReferenceValidationError> {
    let mut seen = BTreeSet::new();
    for (index, reference) in refs.iter().enumerate() {
        let item_path = format!("{path}[{index}]");
        reference.validate(&item_path)?;
        reference.require_kind(expected_kind, &format!("{item_path}.entity_kind"))?;
        if !seen.insert(reference.identity_key()) {
            return Err(ProcessReferenceValidationError::new(
                "graph_ref.duplicate",
                item_path,
            ));
        }
    }
    Ok(())
}

fn validate_token(value: &str, path: &str) -> Result<(), ProcessReferenceValidationError> {
    if value.is_empty() {
        return Err(ProcessReferenceValidationError::new(
            "graph_ref.empty_field",
            path,
        ));
    }
    if value.trim() != value || value.chars().any(char::is_control) {
        return Err(ProcessReferenceValidationError::new(
            "graph_ref.non_canonical",
            path,
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::process_registry::{
        CanonicalIndustrialArtifactRefV1, DigestAlgorithmV1, TYPED_ROUTING_BINDING_SCHEMA_V1,
    };
    use serde_json::json;

    fn digest(marker: char) -> ContentDigestV1 {
        ContentDigestV1 {
            algorithm: DigestAlgorithmV1::Sha256,
            hex: marker.to_string().repeat(64),
        }
    }

    fn artifact_ref() -> CanonicalIndustrialArtifactRefV1 {
        CanonicalIndustrialArtifactRefV1 {
            kind: IndustrialArtifactKindV1::ProcessDefinition,
            namespace: "luminous".to_owned(),
            identifier_scheme: "process-id".to_owned(),
            identifier: "process:cnc-milling/v1".to_owned(),
            semantic_version: "1.0.0".to_owned(),
            content_digest: digest('a'),
        }
    }

    fn routing_step() -> TypedRoutingStepRefV1 {
        TypedRoutingStepRefV1 {
            schema_version: TYPED_ROUTING_BINDING_SCHEMA_V1.to_owned(),
            source_routing_system: "mycelix-manufacturing".to_owned(),
            source_routing_ref: "routing:fixture-A".to_owned(),
            source_routing_revision: "rev-7".to_owned(),
            source_routing_digest: digest('b'),
            step_sequence: 1,
            process_definition_ref: artifact_ref(),
            capability_requirement_ref: None,
            recipe_ref: None,
            legacy_operation_name: Some("Milling".to_owned()),
            legacy_machine_type: Some("CNC3Axis".to_owned()),
        }
    }

    fn graph_ref(kind: GraphEntityKindV1, id: &str, marker: char) -> GraphEntityRefV1 {
        GraphEntityRefV1 {
            graph_system: "mycelix-erg".to_owned(),
            entity_kind: kind,
            entity_ref: id.to_owned(),
            revision: "rev-1".to_owned(),
            content_digest: digest(marker),
        }
    }

    fn bundle() -> ProcessExecutionEvidenceBundleV1 {
        ProcessExecutionEvidenceBundleV1 {
            schema_version: PROCESS_EXECUTION_EVIDENCE_SCHEMA_V1.to_owned(),
            execution_event_ref: graph_ref(GraphEntityKindV1::Event, "event:execution-1", 'c'),
            routing_step_ref: routing_step(),
            input_resource_refs: vec![graph_ref(
                GraphEntityKindV1::Resource,
                "resource:stock-1",
                'd',
            )],
            output_resource_refs: vec![graph_ref(
                GraphEntityKindV1::Resource,
                "resource:part-1",
                'e',
            )],
            supporting_event_refs: vec![graph_ref(
                GraphEntityKindV1::Event,
                "event:inspection-1",
                'f',
            )],
            conformity_claim_refs: vec![graph_ref(
                GraphEntityKindV1::Claim,
                "claim:inspection-1",
                '1',
            )],
        }
    }

    #[test]
    fn valid_bundle_round_trips_with_graph_provenance_and_route_snapshot() {
        let original = bundle();
        original.validate().expect("valid evidence links");
        let encoded = serde_json::to_string(&original).expect("serialize");
        let decoded: ProcessExecutionEvidenceBundleV1 =
            serde_json::from_str(&encoded).expect("deserialize");
        assert_eq!(decoded, original);
        assert_eq!(decoded.routing_step_ref.source_routing_revision, "rev-7");
        assert_eq!(
            decoded.input_resource_refs[0].entity_ref,
            "resource:stock-1"
        );
    }

    #[test]
    fn execution_must_reference_an_event() {
        let mut value = bundle();
        value.execution_event_ref.entity_kind = GraphEntityKindV1::Resource;
        assert_eq!(value.validate().unwrap_err().code, "graph_ref.wrong_kind");
    }

    #[test]
    fn reference_groups_enforce_their_declared_entity_kinds() {
        let mut value = bundle();
        value.output_resource_refs[0].entity_kind = GraphEntityKindV1::Claim;
        assert_eq!(value.validate().unwrap_err().code, "graph_ref.wrong_kind");

        let mut value = bundle();
        value.conformity_claim_refs[0].entity_kind = GraphEntityKindV1::Event;
        assert_eq!(value.validate().unwrap_err().code, "graph_ref.wrong_kind");
    }

    #[test]
    fn same_external_identity_cannot_be_smuggled_in_with_another_revision() {
        let mut value = bundle();
        let mut duplicate = value.input_resource_refs[0].clone();
        duplicate.revision = "rev-2".to_owned();
        duplicate.content_digest = digest('9');
        value.input_resource_refs.push(duplicate);
        assert_eq!(value.validate().unwrap_err().code, "graph_ref.duplicate");
    }

    #[test]
    fn source_ref_must_have_canonical_non_empty_fields_and_digest() {
        let mut value = bundle();
        value.execution_event_ref.revision = " rev-1".to_owned();
        assert_eq!(
            value.validate().unwrap_err().code,
            "graph_ref.non_canonical"
        );

        let mut value = bundle();
        value.execution_event_ref.content_digest.hex = "abcd".to_owned();
        assert_eq!(value.validate().unwrap_err().code, "digest.invalid_length");
    }

    #[test]
    fn execution_bundle_rejects_unknown_schema_and_unknown_json_fields() {
        let mut value = bundle();
        value.schema_version = "mycelix-process-execution-evidence/99".to_owned();
        assert_eq!(
            value.validate().unwrap_err().code,
            "execution_bundle.unsupported_schema"
        );

        let mut encoded = serde_json::to_value(bundle()).expect("serialize");
        encoded
            .as_object_mut()
            .expect("object")
            .insert("qualified".to_owned(), json!(true));
        assert!(serde_json::from_value::<ProcessExecutionEvidenceBundleV1>(encoded).is_err());
    }

    #[test]
    fn missing_optional_evidence_does_not_invent_claims() {
        let mut value = bundle();
        value.supporting_event_refs.clear();
        value.conformity_claim_refs.clear();
        value.validate().expect("references may be absent");
        let encoded = serde_json::to_value(value).expect("serialize");
        assert!(encoded.get("verified").is_none());
        assert!(encoded.get("authorized").is_none());
        assert!(encoded.get("qualified").is_none());
    }

    #[test]
    fn duplicate_supporting_events_are_rejected() {
        let mut value = bundle();
        value
            .supporting_event_refs
            .push(value.supporting_event_refs[0].clone());
        assert_eq!(value.validate().unwrap_err().code, "graph_ref.duplicate");
    }
}
