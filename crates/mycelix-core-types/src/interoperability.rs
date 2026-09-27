// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Transport-neutral semantic reference primitives.
//!
//! These types identify a schema context and an object within that context.
//! They deliberately do **not** establish authenticity, trust, authority,
//! semantic equivalence, evidence quality, verification, or content binding.
//! They also do not define a canonical byte encoding or semantic commitment.

use core::fmt;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Maximum UTF-8 byte length accepted for a schema namespace.
pub const MAX_SCHEMA_NAMESPACE_BYTES: usize = 256;
/// Maximum UTF-8 byte length accepted for a schema name.
pub const MAX_SCHEMA_NAME_BYTES: usize = 128;
/// Maximum UTF-8 byte length accepted for a schema version.
pub const MAX_SCHEMA_VERSION_BYTES: usize = 128;
/// Maximum UTF-8 byte length accepted for an opaque object identifier.
pub const MAX_OBJECT_ID_BYTES: usize = 1024;
/// Maximum UTF-8 byte length accepted for an object version.
pub const MAX_OBJECT_VERSION_BYTES: usize = 128;

/// Validation error for [`SchemaRef`] and [`SemanticRef`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReferenceValidationError {
    /// A required field was empty.
    Empty { field: &'static str },
    /// Leading or trailing whitespace would make identity ambiguous.
    SurroundingWhitespace { field: &'static str },
    /// A control character was present in an identifier field.
    ControlCharacter { field: &'static str },
    /// A field exceeded its wire-safety byte bound.
    TooLong {
        field: &'static str,
        max_bytes: usize,
        actual_bytes: usize,
    },
}

impl fmt::Display for ReferenceValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty { field } => write!(f, "{field} must not be empty"),
            Self::SurroundingWhitespace { field } => {
                write!(f, "{field} must not contain leading or trailing whitespace")
            }
            Self::ControlCharacter { field } => {
                write!(f, "{field} must not contain control characters")
            }
            Self::TooLong {
                field,
                max_bytes,
                actual_bytes,
            } => write!(
                f,
                "{field} exceeds maximum length of {max_bytes} bytes (got {actual_bytes})"
            ),
        }
    }
}

impl std::error::Error for ReferenceValidationError {}

fn validate_component(
    field: &'static str,
    value: &str,
    max_bytes: usize,
) -> Result<(), ReferenceValidationError> {
    if value.is_empty() {
        return Err(ReferenceValidationError::Empty { field });
    }
    if value.trim() != value {
        return Err(ReferenceValidationError::SurroundingWhitespace { field });
    }
    if value.chars().any(char::is_control) {
        return Err(ReferenceValidationError::ControlCharacter { field });
    }
    if value.len() > max_bytes {
        return Err(ReferenceValidationError::TooLong {
            field,
            max_bytes,
            actual_bytes: value.len(),
        });
    }
    Ok(())
}

/// Identifies a semantic schema without asserting equivalence to any other schema.
///
/// All three fields are part of identity. In particular, equal schema names under
/// different namespaces or versions remain distinct.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "RawSchemaRef"))]
pub struct SchemaRef {
    namespace: String,
    name: String,
    version: String,
}

impl SchemaRef {
    /// Construct a validated schema reference.
    ///
    /// Inputs are preserved exactly. The constructor rejects surrounding
    /// whitespace rather than normalizing it, because normalization could collapse
    /// externally controlled identifiers.
    pub fn new(
        namespace: impl Into<String>,
        name: impl Into<String>,
        version: impl Into<String>,
    ) -> Result<Self, ReferenceValidationError> {
        let namespace = namespace.into();
        let name = name.into();
        let version = version.into();

        validate_component("namespace", &namespace, MAX_SCHEMA_NAMESPACE_BYTES)?;
        validate_component("name", &name, MAX_SCHEMA_NAME_BYTES)?;
        validate_component("version", &version, MAX_SCHEMA_VERSION_BYTES)?;

        Ok(Self {
            namespace,
            name,
            version,
        })
    }

    /// Namespace that owns/interprets the schema identifier.
    pub fn namespace(&self) -> &str {
        &self.namespace
    }

    /// Schema-local name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Explicit schema revision/version string.
    pub fn version(&self) -> &str {
        &self.version
    }
}

#[cfg(feature = "serde")]
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSchemaRef {
    namespace: String,
    name: String,
    version: String,
}

#[cfg(feature = "serde")]
impl TryFrom<RawSchemaRef> for SchemaRef {
    type Error = ReferenceValidationError;

    fn try_from(raw: RawSchemaRef) -> Result<Self, Self::Error> {
        Self::new(raw.namespace, raw.name, raw.version)
    }
}

/// Opaque reference to an object under an explicit semantic schema.
///
/// `object_id` is intentionally not parsed as a URL, Holochain hash, DID, UUID,
/// or content digest. Domain adapters retain ownership of those semantics.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "RawSemanticRef"))]
pub struct SemanticRef {
    schema: SchemaRef,
    object_id: String,
    object_version: Option<String>,
}

impl SemanticRef {
    /// Construct a reference to an unversioned (or externally versioned) object.
    pub fn new(
        schema: SchemaRef,
        object_id: impl Into<String>,
    ) -> Result<Self, ReferenceValidationError> {
        let object_id = object_id.into();
        validate_component("object_id", &object_id, MAX_OBJECT_ID_BYTES)?;

        Ok(Self {
            schema,
            object_id,
            object_version: None,
        })
    }

    /// Construct a reference with an explicit object-local version.
    pub fn new_versioned(
        schema: SchemaRef,
        object_id: impl Into<String>,
        object_version: impl Into<String>,
    ) -> Result<Self, ReferenceValidationError> {
        let object_id = object_id.into();
        let object_version = object_version.into();

        validate_component("object_id", &object_id, MAX_OBJECT_ID_BYTES)?;
        validate_component(
            "object_version",
            &object_version,
            MAX_OBJECT_VERSION_BYTES,
        )?;

        Ok(Self {
            schema,
            object_id,
            object_version: Some(object_version),
        })
    }

    /// Schema context required to interpret the referenced object.
    pub fn schema(&self) -> &SchemaRef {
        &self.schema
    }

    /// Opaque object identifier, interpreted only by the owning schema/domain.
    pub fn object_id(&self) -> &str {
        &self.object_id
    }

    /// Optional object-local version/revision identifier.
    pub fn object_version(&self) -> Option<&str> {
        self.object_version.as_deref()
    }
}

#[cfg(feature = "serde")]
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawSemanticRef {
    schema: SchemaRef,
    object_id: String,
    object_version: Option<String>,
}

#[cfg(feature = "serde")]
impl TryFrom<RawSemanticRef> for SemanticRef {
    type Error = ReferenceValidationError;

    fn try_from(raw: RawSemanticRef) -> Result<Self, Self::Error> {
        match raw.object_version {
            Some(version) => Self::new_versioned(raw.schema, raw.object_id, version),
            None => Self::new(raw.schema, raw.object_id),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn core_epistemic() -> SchemaRef {
        SchemaRef::new("mycelix.core", "epistemic", "current").unwrap()
    }

    fn knowledge_epistemic() -> SchemaRef {
        SchemaRef::new("mycelix.knowledge.claims", "epistemic", "current").unwrap()
    }

    #[test]
    fn namespace_distinguishes_same_schema_name() {
        let core = SchemaRef::new("mycelix.core", "epistemic", "v1").unwrap();
        let knowledge =
            SchemaRef::new("mycelix.knowledge.claims", "epistemic", "v1").unwrap();
        assert_ne!(core, knowledge);
    }

    #[test]
    fn version_is_part_of_schema_identity() {
        let v1 = SchemaRef::new("example.org", "claims", "v1").unwrap();
        let v2 = SchemaRef::new("example.org", "claims", "v2").unwrap();
        assert_ne!(v1, v2);
    }

    #[test]
    fn same_object_id_under_different_schemas_is_distinct() {
        let core = SemanticRef::new(core_epistemic(), "claim:42").unwrap();
        let knowledge = SemanticRef::new(knowledge_epistemic(), "claim:42").unwrap();
        assert_ne!(core, knowledge);
    }

    #[test]
    fn core_and_knowledge_epistemic_refs_are_distinct() {
        assert_ne!(core_epistemic(), knowledge_epistemic());
    }

    #[test]
    fn eight_and_twelve_harmony_schemas_are_distinct() {
        let core = SchemaRef::new("mycelix.core", "harmonics-8", "current").unwrap();
        let knowledge =
            SchemaRef::new("mycelix.knowledge.claims", "harmonics-12", "current").unwrap();
        assert_ne!(core, knowledge);
    }

    #[test]
    fn object_version_is_part_of_semantic_identity() {
        let schema = SchemaRef::new("example.org", "artifact", "v1").unwrap();
        let v1 = SemanticRef::new_versioned(schema.clone(), "item:7", "1").unwrap();
        let v2 = SemanticRef::new_versioned(schema, "item:7", "2").unwrap();
        assert_ne!(v1, v2);
    }

    #[test]
    fn accepts_opaque_uri_and_holochain_like_identifiers() {
        let schema = SchemaRef::new("example.org", "evidence", "2026-09").unwrap();
        let uri = SemanticRef::new(schema.clone(), "urn:example:evidence/42?rev=3#part-a").unwrap();
        assert_eq!(uri.object_id(), "urn:example:evidence/42?rev=3#part-a");

        let holo = SemanticRef::new(schema, "uhCAkX1234567890abcdef...").unwrap();
        assert_eq!(holo.object_id(), "uhCAkX1234567890abcdef...");
    }

    #[test]
    fn rejects_empty_required_components() {
        assert_eq!(
            SchemaRef::new("", "claims", "v1"),
            Err(ReferenceValidationError::Empty { field: "namespace" })
        );

        let schema = SchemaRef::new("example.org", "claims", "v1").unwrap();
        assert_eq!(
            SemanticRef::new(schema, ""),
            Err(ReferenceValidationError::Empty { field: "object_id" })
        );
    }

    #[test]
    fn rejects_surrounding_whitespace_without_normalizing() {
        assert_eq!(
            SchemaRef::new(" example.org", "claims", "v1"),
            Err(ReferenceValidationError::SurroundingWhitespace {
                field: "namespace"
            })
        );

        let schema = SchemaRef::new("example.org", "claims", "v1").unwrap();
        assert_eq!(
            SemanticRef::new(schema, "claim:42 "),
            Err(ReferenceValidationError::SurroundingWhitespace {
                field: "object_id"
            })
        );
    }

    #[test]
    fn rejects_control_characters() {
        assert_eq!(
            SchemaRef::new("example.org", "claim\nset", "v1"),
            Err(ReferenceValidationError::ControlCharacter { field: "name" })
        );
    }

    #[test]
    fn rejects_values_over_wire_safety_bounds() {
        let too_long_namespace = "n".repeat(MAX_SCHEMA_NAMESPACE_BYTES + 1);
        assert_eq!(
            SchemaRef::new(too_long_namespace, "claims", "v1"),
            Err(ReferenceValidationError::TooLong {
                field: "namespace",
                max_bytes: MAX_SCHEMA_NAMESPACE_BYTES,
                actual_bytes: MAX_SCHEMA_NAMESPACE_BYTES + 1,
            })
        );
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_full_identity() {
        let original = SemanticRef::new_versioned(
            SchemaRef::new("mycelix.knowledge.claims", "evidence", "v3").unwrap(),
            "evidence:abc123",
            "17",
        )
        .unwrap();

        let json = serde_json::to_string(&original).unwrap();
        let decoded: SemanticRef = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, original);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_deserialization_rejects_invalid_schema_components() {
        let json = r#"{"namespace":" example.org","name":"claims","version":"v1"}"#;
        let decoded = serde_json::from_str::<SchemaRef>(json);
        assert!(decoded.is_err());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_deserialization_rejects_invalid_object_version() {
        let json = r#"{
            "schema":{"namespace":"example.org","name":"claims","version":"v1"},
            "object_id":"claim:42",
            "object_version":""
        }"#;
        let decoded = serde_json::from_str::<SemanticRef>(json);
        assert!(decoded.is_err());
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_deserialization_rejects_unknown_identity_fields() {
        let json = r#"{
            "namespace":"example.org",
            "name":"claims",
            "version":"v1",
            "semantic_revision":"surprise"
        }"#;
        let decoded = serde_json::from_str::<SchemaRef>(json);
        assert!(decoded.is_err());
    }
}
