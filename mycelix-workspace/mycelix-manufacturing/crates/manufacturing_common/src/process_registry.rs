//! Typed references to canonical manufacturing-process artifacts.
//!
//! This module stores references rather than copying process physics, quantities, recipes,
//! machine capability, or qualification semantics. The DTOs are additive: existing Holochain
//! OperationEntry and RoutingEntry layouts are not changed.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const PROCESS_REGISTRY_SCHEMA_V1: &str = "mycelix-manufacturing-process-registry/1";
pub const TYPED_ROUTING_BINDING_SCHEMA_V1: &str = "mycelix-typed-routing-binding/1";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProcessReferenceValidationError {
    /// Stable machine-readable violation code.
    pub code: &'static str,
    /// JSONPath-like diagnostic location.
    pub path: String,
}

impl ProcessReferenceValidationError {
    fn new(code: &'static str, path: impl Into<String>) -> Self {
        Self {
            code,
            path: path.into(),
        }
    }
}

impl std::fmt::Display for ProcessReferenceValidationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.path)
    }
}

impl std::error::Error for ProcessReferenceValidationError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DigestAlgorithmV1 {
    Sha256,
    Blake3,
}

/// Content digest for exact externally-owned bytes.
///
/// Validation checks syntax only. Recompute the digest from source bytes whenever those bytes
/// are available. A well-formed digest string is not proof of source truth.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContentDigestV1 {
    pub algorithm: DigestAlgorithmV1,
    pub hex: String,
}

impl ContentDigestV1 {
    pub fn validate(&self, path: &str) -> Result<(), ProcessReferenceValidationError> {
        if self.hex.len() != 64 {
            return Err(ProcessReferenceValidationError::new(
                "digest.invalid_length",
                path,
            ));
        }
        if !self
            .hex
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
        {
            return Err(ProcessReferenceValidationError::new(
                "digest.invalid_hex",
                path,
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndustrialArtifactKindV1 {
    ProcessDefinition,
    ProcessRecipe,
    CapabilityRequirement,
    ProcessCapabilityProfile,
    ProcessModelProfile,
}

/// Namespaced reference interpreted only under the declared scheme, version and digest.
///
/// The identifier is opaque to Mycelix. Neither a syntactically valid identifier nor a digest
/// field proves that the external artifact exists, was authenticated, or is qualified.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CanonicalIndustrialArtifactRefV1 {
    pub kind: IndustrialArtifactKindV1,
    pub namespace: String,
    pub identifier_scheme: String,
    pub identifier: String,
    pub semantic_version: String,
    pub content_digest: ContentDigestV1,
}

impl CanonicalIndustrialArtifactRefV1 {
    pub fn validate(&self, path: &str) -> Result<(), ProcessReferenceValidationError> {
        validate_token("token.empty", &format!("{path}.namespace"), &self.namespace)?;
        validate_token(
            "token.empty",
            &format!("{path}.identifier_scheme"),
            &self.identifier_scheme,
        )?;
        validate_token(
            "token.empty",
            &format!("{path}.identifier"),
            &self.identifier,
        )?;
        validate_token(
            "token.empty",
            &format!("{path}.semantic_version"),
            &self.semantic_version,
        )?;
        self.content_digest
            .validate(&format!("{path}.content_digest"))
    }

    fn require_kind(
        &self,
        expected: IndustrialArtifactKindV1,
        path: &str,
    ) -> Result<(), ProcessReferenceValidationError> {
        if self.kind != expected {
            return Err(ProcessReferenceValidationError::new(
                "artifact.wrong_kind",
                format!("{path}: expected {expected:?}, got {:?}", self.kind),
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegistryLifecycleV1 {
    Active,
    Deprecated,
    Retired,
}

/// Discoverability record for an externally-owned process definition.
///
/// Labels and legacy aliases are navigation/migration metadata, not semantic identity.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProcessRegistryEntryV1 {
    pub schema_version: String,
    pub process_definition_ref: CanonicalIndustrialArtifactRefV1,
    pub display_label: Option<String>,
    /// Namespaced taxonomy references, not a closed global enum.
    pub family_refs: Vec<String>,
    /// Legacy strings for migration/discovery only, never capability evidence.
    pub legacy_operation_aliases: Vec<String>,
    /// Registry lifecycle only; not engineering qualification.
    pub lifecycle: RegistryLifecycleV1,
    pub supersedes: Vec<CanonicalIndustrialArtifactRefV1>,
}

impl ProcessRegistryEntryV1 {
    pub fn validate(&self) -> Result<(), ProcessReferenceValidationError> {
        if self.schema_version != PROCESS_REGISTRY_SCHEMA_V1 {
            return Err(ProcessReferenceValidationError::new(
                "registry.unsupported_schema",
                "schema_version",
            ));
        }
        self.process_definition_ref
            .validate("process_definition_ref")?;
        self.process_definition_ref.require_kind(
            IndustrialArtifactKindV1::ProcessDefinition,
            "process_definition_ref.kind",
        )?;
        if let Some(label) = &self.display_label {
            validate_token("token.empty", "display_label", label)?;
        }

        let mut families = BTreeSet::new();
        for (index, family_ref) in self.family_refs.iter().enumerate() {
            validate_namespaced_ref(family_ref, &format!("family_refs[{index}]"))?;
            if !families.insert(family_ref) {
                return Err(ProcessReferenceValidationError::new(
                    "registry.duplicate_family_ref",
                    format!("family_refs[{index}]"),
                ));
            }
        }

        let mut aliases = BTreeSet::new();
        for (index, alias) in self.legacy_operation_aliases.iter().enumerate() {
            validate_token(
                "token.empty",
                &format!("legacy_operation_aliases[{index}]"),
                alias,
            )?;
            if !aliases.insert(alias) {
                return Err(ProcessReferenceValidationError::new(
                    "registry.duplicate_legacy_alias",
                    format!("legacy_operation_aliases[{index}]"),
                ));
            }
        }

        let mut predecessors = BTreeSet::new();
        for (index, previous) in self.supersedes.iter().enumerate() {
            let path = format!("supersedes[{index}]");
            previous.validate(&path)?;
            previous.require_kind(
                IndustrialArtifactKindV1::ProcessDefinition,
                &format!("{path}.kind"),
            )?;
            if previous == &self.process_definition_ref {
                return Err(ProcessReferenceValidationError::new(
                    "registry.self_supersession",
                    path,
                ));
            }
            if !predecessors.insert(previous) {
                return Err(ProcessReferenceValidationError::new(
                    "registry.duplicate_superseded_ref",
                    path,
                ));
            }
        }
        Ok(())
    }
}

/// Additive sidecar binding one legacy routing step to canonical process semantics.
///
/// The routing digest must be recomputed from the exact source-routing bytes by its adapter.
/// Legacy names remain compatibility metadata and never qualify a route.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TypedRoutingStepRefV1 {
    pub schema_version: String,
    pub source_routing_system: String,
    pub source_routing_ref: String,
    pub source_routing_revision: String,
    pub source_routing_digest: ContentDigestV1,
    pub step_sequence: u32,
    pub process_definition_ref: CanonicalIndustrialArtifactRefV1,
    pub capability_requirement_ref: Option<CanonicalIndustrialArtifactRefV1>,
    pub recipe_ref: Option<CanonicalIndustrialArtifactRefV1>,
    pub legacy_operation_name: Option<String>,
    pub legacy_machine_type: Option<String>,
}

impl TypedRoutingStepRefV1 {
    pub fn validate(&self) -> Result<(), ProcessReferenceValidationError> {
        if self.schema_version != TYPED_ROUTING_BINDING_SCHEMA_V1 {
            return Err(ProcessReferenceValidationError::new(
                "routing.unsupported_schema",
                "schema_version",
            ));
        }
        validate_token(
            "token.empty",
            "source_routing_system",
            &self.source_routing_system,
        )?;
        validate_token(
            "token.empty",
            "source_routing_ref",
            &self.source_routing_ref,
        )?;
        validate_token(
            "token.empty",
            "source_routing_revision",
            &self.source_routing_revision,
        )?;
        self.source_routing_digest
            .validate("source_routing_digest")?;
        if self.step_sequence == 0 {
            return Err(ProcessReferenceValidationError::new(
                "routing.zero_sequence",
                "step_sequence",
            ));
        }
        self.process_definition_ref
            .validate("process_definition_ref")?;
        self.process_definition_ref.require_kind(
            IndustrialArtifactKindV1::ProcessDefinition,
            "process_definition_ref.kind",
        )?;
        if let Some(capability) = &self.capability_requirement_ref {
            capability.validate("capability_requirement_ref")?;
            capability.require_kind(
                IndustrialArtifactKindV1::CapabilityRequirement,
                "capability_requirement_ref.kind",
            )?;
        }
        if let Some(recipe) = &self.recipe_ref {
            recipe.validate("recipe_ref")?;
            recipe.require_kind(IndustrialArtifactKindV1::ProcessRecipe, "recipe_ref.kind")?;
        }
        if let Some(name) = &self.legacy_operation_name {
            validate_token("token.empty", "legacy_operation_name", name)?;
        }
        if let Some(machine_type) = &self.legacy_machine_type {
            validate_token("token.empty", "legacy_machine_type", machine_type)?;
        }
        Ok(())
    }
}

/// Enforce one source routing snapshot and unique step sequences within a sidecar batch.
/// This is structural validation; it does not resolve or authorize the source routing.
pub fn validate_typed_routing_steps(
    steps: &[TypedRoutingStepRefV1],
) -> Result<(), ProcessReferenceValidationError> {
    let Some(first) = steps.first() else {
        return Ok(());
    };
    first.validate()?;

    let mut sequences = BTreeSet::new();
    for (index, step) in steps.iter().enumerate() {
        let path = format!("steps[{index}]");
        step.validate()?;
        if step.source_routing_system != first.source_routing_system
            || step.source_routing_ref != first.source_routing_ref
            || step.source_routing_revision != first.source_routing_revision
            || step.source_routing_digest != first.source_routing_digest
        {
            return Err(ProcessReferenceValidationError::new(
                "routing.snapshot_mismatch",
                path,
            ));
        }
        if !sequences.insert(step.step_sequence) {
            return Err(ProcessReferenceValidationError::new(
                "routing.duplicate_sequence",
                format!("{path}.step_sequence"),
            ));
        }
    }
    Ok(())
}

fn validate_namespaced_ref(value: &str, path: &str) -> Result<(), ProcessReferenceValidationError> {
    validate_token("token.empty", path, value)?;
    let Some((namespace, local)) = value.split_once(':') else {
        return Err(ProcessReferenceValidationError::new(
            "reference.missing_namespace",
            path,
        ));
    };
    validate_token("token.empty", &format!("{path}.namespace"), namespace)?;
    validate_token("token.empty", &format!("{path}.local"), local)
}

fn validate_token(
    empty_code: &'static str,
    path: &str,
    value: &str,
) -> Result<(), ProcessReferenceValidationError> {
    if value.is_empty() {
        return Err(ProcessReferenceValidationError::new(empty_code, path));
    }
    if value.trim() != value || value.chars().any(char::is_control) {
        return Err(ProcessReferenceValidationError::new(
            "token.non_canonical",
            path,
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn digest(hex: &str) -> ContentDigestV1 {
        ContentDigestV1 {
            algorithm: DigestAlgorithmV1::Sha256,
            hex: hex.to_owned(),
        }
    }

    fn artifact(
        kind: IndustrialArtifactKindV1,
        id: &str,
        marker: char,
    ) -> CanonicalIndustrialArtifactRefV1 {
        CanonicalIndustrialArtifactRefV1 {
            kind,
            namespace: "luminous".to_owned(),
            identifier_scheme: format!("{kind:?}-id").to_lowercase(),
            identifier: id.to_owned(),
            semantic_version: "1.0.0".to_owned(),
            content_digest: digest(&marker.to_string().repeat(64)),
        }
    }

    fn process_ref() -> CanonicalIndustrialArtifactRefV1 {
        artifact(
            IndustrialArtifactKindV1::ProcessDefinition,
            "process:cnc-milling/v1",
            'a',
        )
    }

    fn recipe_ref() -> CanonicalIndustrialArtifactRefV1 {
        artifact(
            IndustrialArtifactKindV1::ProcessRecipe,
            "recipe:fixture-mill-01",
            'b',
        )
    }

    fn capability_ref() -> CanonicalIndustrialArtifactRefV1 {
        artifact(
            IndustrialArtifactKindV1::CapabilityRequirement,
            "capability:inspection",
            'c',
        )
    }

    fn registry_entry() -> ProcessRegistryEntryV1 {
        ProcessRegistryEntryV1 {
            schema_version: PROCESS_REGISTRY_SCHEMA_V1.to_owned(),
            process_definition_ref: process_ref(),
            display_label: Some("CNC milling".to_owned()),
            family_refs: vec!["iso-astm:material-removal".to_owned()],
            legacy_operation_aliases: vec!["MILL-01".to_owned()],
            lifecycle: RegistryLifecycleV1::Active,
            supersedes: vec![],
        }
    }

    fn routing_step(sequence: u32) -> TypedRoutingStepRefV1 {
        TypedRoutingStepRefV1 {
            schema_version: TYPED_ROUTING_BINDING_SCHEMA_V1.to_owned(),
            source_routing_system: "mycelix-manufacturing".to_owned(),
            source_routing_ref: "routing:fixture-A".to_owned(),
            source_routing_revision: "rev-7".to_owned(),
            source_routing_digest: digest(&"d".repeat(64)),
            step_sequence: sequence,
            process_definition_ref: process_ref(),
            capability_requirement_ref: Some(capability_ref()),
            recipe_ref: Some(recipe_ref()),
            legacy_operation_name: Some("Milling".to_owned()),
            legacy_machine_type: Some("CNC3Axis".to_owned()),
        }
    }

    #[test]
    fn digest_accepts_sha256_and_blake3_lowercase_hex() {
        assert!(digest(&"a".repeat(64)).validate("digest").is_ok());
        let blake3 = ContentDigestV1 {
            algorithm: DigestAlgorithmV1::Blake3,
            hex: "0123456789abcdef".repeat(4),
        };
        assert!(blake3.validate("digest").is_ok());
    }

    #[test]
    fn digest_rejects_uppercase_and_wrong_length() {
        assert_eq!(
            digest(&"A".repeat(64)).validate("digest").unwrap_err().code,
            "digest.invalid_hex"
        );
        assert_eq!(
            digest("abcd").validate("digest").unwrap_err().code,
            "digest.invalid_length"
        );
    }

    #[test]
    fn registry_rejects_unknown_schema_and_wrong_artifact_kind() {
        let mut entry = registry_entry();
        entry.schema_version = "mycelix-manufacturing-process-registry/99".to_owned();
        assert_eq!(
            entry.validate().unwrap_err().code,
            "registry.unsupported_schema"
        );

        let mut entry = registry_entry();
        entry.process_definition_ref.kind = IndustrialArtifactKindV1::ProcessRecipe;
        assert_eq!(entry.validate().unwrap_err().code, "artifact.wrong_kind");
    }

    #[test]
    fn labels_and_legacy_aliases_do_not_change_canonical_artifact_reference() {
        let original = registry_entry();
        let mut relabelled = original.clone();
        relabelled.display_label = Some("Metal removal operation".to_owned());
        relabelled.legacy_operation_aliases = vec!["OLD-ROUTE-NAME".to_owned()];
        assert_eq!(
            original.process_definition_ref,
            relabelled.process_definition_ref
        );
        assert!(original.validate().is_ok());
        assert!(relabelled.validate().is_ok());
    }

    #[test]
    fn registry_rejects_duplicate_or_unnamespaced_family_refs() {
        let mut entry = registry_entry();
        entry.family_refs.push(entry.family_refs[0].clone());
        assert_eq!(
            entry.validate().unwrap_err().code,
            "registry.duplicate_family_ref"
        );

        let mut entry = registry_entry();
        entry.family_refs = vec!["material-removal".to_owned()];
        assert_eq!(
            entry.validate().unwrap_err().code,
            "reference.missing_namespace"
        );
    }

    #[test]
    fn registry_rejects_duplicate_aliases_and_self_supersession() {
        let mut entry = registry_entry();
        entry
            .legacy_operation_aliases
            .push(entry.legacy_operation_aliases[0].clone());
        assert_eq!(
            entry.validate().unwrap_err().code,
            "registry.duplicate_legacy_alias"
        );

        let mut entry = registry_entry();
        entry.supersedes.push(entry.process_definition_ref.clone());
        assert_eq!(
            entry.validate().unwrap_err().code,
            "registry.self_supersession"
        );
    }

    #[test]
    fn routing_binding_checks_process_recipe_and_capability_kinds() {
        assert!(routing_step(1).validate().is_ok());

        let mut wrong_recipe = routing_step(1);
        wrong_recipe.recipe_ref = Some(capability_ref());
        assert_eq!(
            wrong_recipe.validate().unwrap_err().code,
            "artifact.wrong_kind"
        );

        let mut wrong_capability = routing_step(1);
        wrong_capability.capability_requirement_ref = Some(recipe_ref());
        assert_eq!(
            wrong_capability.validate().unwrap_err().code,
            "artifact.wrong_kind"
        );
    }

    #[test]
    fn routing_batch_rejects_duplicate_sequence_and_mixed_revisions() {
        assert!(validate_typed_routing_steps(&[routing_step(1), routing_step(2)]).is_ok());
        assert_eq!(
            validate_typed_routing_steps(&[routing_step(1), routing_step(1)])
                .unwrap_err()
                .code,
            "routing.duplicate_sequence"
        );

        let mut other_revision = routing_step(2);
        other_revision.source_routing_revision = "rev-8".to_owned();
        assert_eq!(
            validate_typed_routing_steps(&[routing_step(1), other_revision])
                .unwrap_err()
                .code,
            "routing.snapshot_mismatch"
        );
    }

    #[test]
    fn typed_reference_round_trip_preserves_namespace_scheme_version_and_digest() {
        let reference = process_ref();
        let encoded = serde_json::to_string(&reference).unwrap();
        let decoded: CanonicalIndustrialArtifactRefV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, reference);
        assert_eq!(decoded.namespace, "luminous");
        assert_eq!(decoded.identifier_scheme, "processdefinition-id");
        assert_eq!(decoded.semantic_version, "1.0.0");
        assert_eq!(decoded.content_digest.hex, "a".repeat(64));
    }

    #[test]
    fn unknown_namespace_remains_representable_without_qualification_fields() {
        let mut reference = process_ref();
        reference.namespace = "future-vendor.example".to_owned();
        reference.identifier = "novel-process/type-x".to_owned();
        assert!(reference.validate("process_definition_ref").is_ok());
        let json = serde_json::to_value(reference).unwrap();
        assert!(json.get("qualified").is_none());
        assert!(json.get("resolved").is_none());
        assert!(json.get("authorized").is_none());
    }

    #[test]
    fn legacy_names_cannot_replace_missing_typed_process_reference() {
        let mut step = routing_step(1);
        step.process_definition_ref.identifier.clear();
        step.legacy_operation_name = Some("well known good operation".to_owned());
        step.legacy_machine_type = Some("CNC3Axis".to_owned());
        assert_eq!(step.validate().unwrap_err().code, "token.empty");
    }

    #[test]
    fn serde_rejects_unknown_fields_in_external_refs() {
        let value = serde_json::to_value(process_ref()).unwrap();
        let mut object = value.as_object().unwrap().clone();
        object.insert("qualified".to_owned(), json!(true));
        assert!(serde_json::from_value::<CanonicalIndustrialArtifactRefV1>(json!(object)).is_err());
    }
}
