use serde::{Deserialize, Serialize};

use crate::{
    Commitment32, SchemaRefV1, SemanticCoreError, SemanticEnvironmentV1, SemanticIdV1,
    SemanticProfileRefV1,
};

impl SemanticEnvironmentV1 {
    /// Derive the deterministic MYC-SEM-001C outer environment commitment.
    ///
    /// This profile is fixed by `mycelix-semantic-core` and is distinct from
    /// the environment's domain `canonicalization_profile`.
    pub fn commitment(&self) -> Commitment32 {
        crate::canonical::derive_environment_commitment(self)
    }

    /// Produce the exact v1 environment reference.
    pub fn reference(&self) -> SemanticEnvironmentRefV1 {
        SemanticEnvironmentRefV1 {
            commitment_profile_revision: crate::canonical::SEMANTIC_COMMITMENT_PROFILE_REVISION,
            commitment: self.commitment(),
        }
    }
}

/// Exact reference to a committed semantic environment.
///
/// The profile revision identifies the outer Mycelix semantic commitment
/// procedure, not the environment's domain canonicalization profile.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticEnvironmentRefV1 {
    commitment_profile_revision: u16,
    commitment: Commitment32,
}

impl SemanticEnvironmentRefV1 {
    /// Construct an opaque environment reference.
    ///
    /// References are identity material, not positive authority receipts. A
    /// caller holding the full environment should prefer
    /// [`SemanticEnvironmentV1::reference`].
    pub const fn new(commitment_profile_revision: u16, commitment: Commitment32) -> Self {
        Self {
            commitment_profile_revision,
            commitment,
        }
    }

    /// Outer semantic commitment profile revision.
    pub const fn commitment_profile_revision(&self) -> u16 {
        self.commitment_profile_revision
    }

    /// Exact environment commitment.
    pub const fn commitment(&self) -> Commitment32 {
        self.commitment
    }
}

/// Exact semantic subject coordinates under one environment.
///
/// Repeating the schema at the subject boundary permits domain-specific
/// sub-schemas while the environment continues to bind the broader semantic
/// interpretation context.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticSubjectRefV1 {
    environment: SemanticEnvironmentRefV1,
    domain: SemanticIdV1,
    schema: SchemaRefV1,
    subject_id: SemanticIdV1,
}

impl SemanticSubjectRefV1 {
    /// Construct exact semantic subject coordinates.
    pub fn new(
        environment: SemanticEnvironmentRefV1,
        domain: impl Into<String>,
        schema: SchemaRefV1,
        subject_id: impl Into<String>,
    ) -> Result<Self, SemanticCoreError> {
        Ok(Self {
            environment,
            domain: SemanticIdV1::new(domain)?,
            schema,
            subject_id: SemanticIdV1::new(subject_id)?,
        })
    }

    /// Exact semantic environment.
    pub const fn environment(&self) -> &SemanticEnvironmentRefV1 {
        &self.environment
    }

    /// Domain-local semantic namespace.
    pub const fn domain(&self) -> &SemanticIdV1 {
        &self.domain
    }

    /// Exact schema under which the subject is interpreted.
    pub const fn schema(&self) -> &SchemaRefV1 {
        &self.schema
    }

    /// Domain-local subject identifier.
    pub const fn subject_id(&self) -> &SemanticIdV1 {
        &self.subject_id
    }

    /// Deterministic v1 semantic subject commitment.
    pub fn commitment(&self) -> Commitment32 {
        crate::canonical::derive_subject_commitment(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: u8) -> Commitment32 {
        Commitment32::from_bytes([byte; 32])
    }

    fn profile(id: &str, revision: u64, byte: u8) -> SemanticProfileRefV1 {
        SemanticProfileRefV1::new(id, revision, digest(byte)).unwrap()
    }

    fn environment() -> SemanticEnvironmentV1 {
        SemanticEnvironmentV1::new(
            SchemaRefV1::new(profile("schema/base", 1, 1)),
            profile("interpretation/base", 2, 2),
            profile("identity/base", 3, 3),
            profile("authority/base", 4, 4),
            profile("temporal/base", 5, 5),
            profile("canonical/domain-v1", 1, 6),
        )
    }

    #[test]
    fn environment_commitment_golden_vector_v1() {
        let environment = environment();
        assert_eq!(
            environment.commitment().to_hex(),
            "283f04d533916528a7054f9afe8958526cd3fdd94e29d9e990828af18dd12343"
        );
        assert_eq!(
            environment.reference().commitment_profile_revision(),
            crate::SEMANTIC_COMMITMENT_PROFILE_REVISION
        );
    }

    #[test]
    fn environment_profile_role_substitution_changes_commitment() {
        let original = environment();
        let swapped = SemanticEnvironmentV1::new(
            original.schema().clone(),
            original.interpretation_profile().clone(),
            original.authority_profile().clone(),
            original.identity_profile().clone(),
            original.temporal_profile().clone(),
            original.canonicalization_profile().clone(),
        );
        assert_ne!(original.commitment(), swapped.commitment());
    }

    #[test]
    fn subject_commitment_golden_vector_v1() {
        let environment = environment();
        let subject = SemanticSubjectRefV1::new(
            environment.reference(),
            "personal",
            environment.schema().clone(),
            "did:mycelix:test/profile",
        )
        .unwrap();

        assert_eq!(
            subject.commitment().to_hex(),
            "5a88314b454bf23478c91af6adeff8a03bef0b8d4bd2f0c031b27321a50eb323"
        );
    }

    #[test]
    fn same_text_under_different_environment_is_not_same_subject() {
        let a = environment();
        let b = SemanticEnvironmentV1::new(
            a.schema().clone(),
            a.interpretation_profile().clone(),
            a.identity_profile().clone(),
            profile("authority/base", 5, 4),
            a.temporal_profile().clone(),
            a.canonicalization_profile().clone(),
        );

        let subject_a = SemanticSubjectRefV1::new(
            a.reference(),
            "personal",
            a.schema().clone(),
            "did:mycelix:test/profile",
        )
        .unwrap();
        let subject_b = SemanticSubjectRefV1::new(
            b.reference(),
            "personal",
            b.schema().clone(),
            "did:mycelix:test/profile",
        )
        .unwrap();

        assert_ne!(subject_a.commitment(), subject_b.commitment());
    }

    #[test]
    fn subject_wire_shape_is_closed() {
        let environment = environment();
        let subject = SemanticSubjectRefV1::new(
            environment.reference(),
            "personal",
            environment.schema().clone(),
            "did:mycelix:test/profile",
        )
        .unwrap();
        let mut value = serde_json::to_value(subject).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("authorized".into(), serde_json::Value::Bool(true));
        let decoded: Result<SemanticSubjectRefV1, _> = serde_json::from_value(value);
        assert!(decoded.is_err());
    }
}
