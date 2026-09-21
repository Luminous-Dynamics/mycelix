use core::fmt;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Maximum UTF-8 byte length for a semantic identifier component.
pub const MAX_SEMANTIC_TEXT_BYTES: usize = 256;

/// Construction and decoding failures for the semantic foundation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SemanticCoreError {
    /// Text was empty, too long, or contained a Unicode control character.
    InvalidText,
    /// A commitment was not exactly 32 hexadecimal bytes.
    InvalidCommitmentHex,
}

impl fmt::Display for SemanticCoreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidText => f.write_str(
                "semantic text must be non-empty, bounded to 256 UTF-8 bytes, and contain no control characters",
            ),
            Self::InvalidCommitmentHex => {
                f.write_str("commitment must be exactly 64 hexadecimal characters")
            }
        }
    }
}

impl std::error::Error for SemanticCoreError {}

/// Bounded UTF-8 text used by semantic identifiers.
///
/// This type establishes shape only. It does not assign meaning, authority, or
/// namespace ownership to the contained text.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(transparent)]
pub struct BoundedSemanticTextV1(String);

impl BoundedSemanticTextV1 {
    /// Construct validated bounded semantic text.
    pub fn new(value: impl Into<String>) -> Result<Self, SemanticCoreError> {
        let value = value.into();
        if value.is_empty()
            || value.len() > MAX_SEMANTIC_TEXT_BYTES
            || value.chars().any(char::is_control)
        {
            return Err(SemanticCoreError::InvalidText);
        }
        Ok(Self(value))
    }

    /// Borrow the validated UTF-8 text.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl fmt::Display for BoundedSemanticTextV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for BoundedSemanticTextV1 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::new(value).map_err(serde::de::Error::custom)
    }
}

/// Exact opaque 32-byte commitment.
///
/// The bytes have no intrinsic semantic meaning. The profile or protocol using
/// the commitment determines what was committed and under which hash/profile.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Commitment32([u8; 32]);

impl Commitment32 {
    /// Construct from exact bytes.
    pub const fn from_bytes(bytes: [u8; 32]) -> Self {
        Self(bytes)
    }

    /// Borrow exact bytes.
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    /// Parse exactly 64 hexadecimal characters.
    pub fn from_hex(value: &str) -> Result<Self, SemanticCoreError> {
        let bytes = value.as_bytes();
        if bytes.len() != 64 {
            return Err(SemanticCoreError::InvalidCommitmentHex);
        }

        let mut out = [0_u8; 32];
        for (index, pair) in bytes.chunks_exact(2).enumerate() {
            let high =
                decode_hex_nibble(pair[0]).ok_or(SemanticCoreError::InvalidCommitmentHex)?;
            let low =
                decode_hex_nibble(pair[1]).ok_or(SemanticCoreError::InvalidCommitmentHex)?;
            out[index] = (high << 4) | low;
        }
        Ok(Self(out))
    }

    /// Encode using canonical lower-case hexadecimal text.
    pub fn to_hex(self) -> String {
        const HEX: &[u8; 16] = b"0123456789abcdef";
        let mut out = String::with_capacity(64);
        for byte in self.0 {
            out.push(char::from(HEX[usize::from(byte >> 4)]));
            out.push(char::from(HEX[usize::from(byte & 0x0f)]));
        }
        out
    }
}

impl fmt::Display for Commitment32 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.to_hex())
    }
}

impl Serialize for Commitment32 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&self.to_hex())
    }
}

impl<'de> Deserialize<'de> for Commitment32 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = String::deserialize(deserializer)?;
        Self::from_hex(&value).map_err(serde::de::Error::custom)
    }
}

fn decode_hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        b'A'..=b'F' => Some(value - b'A' + 10),
        _ => None,
    }
}

/// Semantic identifier text whose interpretation is supplied by an explicit
/// environment/profile in later tranches.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SemanticIdV1(BoundedSemanticTextV1);

impl SemanticIdV1 {
    /// Construct a bounded semantic identifier.
    pub fn new(value: impl Into<String>) -> Result<Self, SemanticCoreError> {
        Ok(Self(BoundedSemanticTextV1::new(value)?))
    }

    /// Borrow the semantic identifier.
    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

impl fmt::Display for SemanticIdV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(f)
    }
}

/// Exact reference to one versioned semantic profile.
///
/// The digest is opaque: this type does not claim how the profile was produced,
/// whether it is trusted, or whether it is current.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticProfileRefV1 {
    id: SemanticIdV1,
    revision: u64,
    digest: Commitment32,
}

impl SemanticProfileRefV1 {
    /// Construct an exact profile reference.
    pub fn new(
        id: impl Into<String>,
        revision: u64,
        digest: Commitment32,
    ) -> Result<Self, SemanticCoreError> {
        Ok(Self {
            id: SemanticIdV1::new(id)?,
            revision,
            digest,
        })
    }

    /// Profile identifier.
    pub fn id(&self) -> &SemanticIdV1 {
        &self.id
    }

    /// Exact profile revision.
    pub const fn revision(&self) -> u64 {
        self.revision
    }

    /// Exact opaque profile digest.
    pub const fn digest(&self) -> Commitment32 {
        self.digest
    }
}

/// Type-safe schema-profile reference.
///
/// This wrapper prevents a generic profile reference from being accidentally
/// supplied at an API boundary that specifically requires a schema profile.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SchemaRefV1(SemanticProfileRefV1);

impl SchemaRefV1 {
    /// Construct from an exact semantic profile reference.
    pub const fn new(profile: SemanticProfileRefV1) -> Self {
        Self(profile)
    }

    /// Borrow the underlying exact profile reference.
    pub const fn profile(&self) -> &SemanticProfileRefV1 {
        &self.0
    }
}

/// Exact semantic environment used to interpret later semantic objects.
///
/// The environment binds profiles; it does not claim that any profile is trusted,
/// current, compatible with another environment, or authoritative for a caller.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SemanticEnvironmentV1 {
    schema: SchemaRefV1,
    interpretation_profile: SemanticProfileRefV1,
    identity_profile: SemanticProfileRefV1,
    authority_profile: SemanticProfileRefV1,
    temporal_profile: SemanticProfileRefV1,
    canonicalization_profile: SemanticProfileRefV1,
}

impl SemanticEnvironmentV1 {
    /// Construct an exact semantic environment.
    pub const fn new(
        schema: SchemaRefV1,
        interpretation_profile: SemanticProfileRefV1,
        identity_profile: SemanticProfileRefV1,
        authority_profile: SemanticProfileRefV1,
        temporal_profile: SemanticProfileRefV1,
        canonicalization_profile: SemanticProfileRefV1,
    ) -> Self {
        Self {
            schema,
            interpretation_profile,
            identity_profile,
            authority_profile,
            temporal_profile,
            canonicalization_profile,
        }
    }

    /// Exact schema profile.
    pub const fn schema(&self) -> &SchemaRefV1 {
        &self.schema
    }

    /// Exact interpretation profile.
    pub const fn interpretation_profile(&self) -> &SemanticProfileRefV1 {
        &self.interpretation_profile
    }

    /// Exact identity-semantics profile.
    pub const fn identity_profile(&self) -> &SemanticProfileRefV1 {
        &self.identity_profile
    }

    /// Exact authority-semantics profile.
    pub const fn authority_profile(&self) -> &SemanticProfileRefV1 {
        &self.authority_profile
    }

    /// Exact temporal-semantics profile.
    pub const fn temporal_profile(&self) -> &SemanticProfileRefV1 {
        &self.temporal_profile
    }

    /// Exact canonicalization profile for semantic values interpreted in this
    /// environment.
    ///
    /// This field does not define how `SemanticEnvironmentV1` itself is committed;
    /// that envelope commitment profile is introduced separately in MYC-SEM-001C.
    pub const fn canonicalization_profile(&self) -> &SemanticProfileRefV1 {
        &self.canonicalization_profile
    }

    /// Derive the deterministic MYC-SEM-001C outer environment commitment.
    ///
    /// This outer commitment profile is fixed by `mycelix-semantic-core` and is
    /// distinct from the environment's domain `canonicalization_profile`.
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

    #[test]
    fn bounded_text_rejects_empty_control_and_oversized_values() {
        assert_eq!(
            BoundedSemanticTextV1::new("").unwrap_err(),
            SemanticCoreError::InvalidText
        );
        assert_eq!(
            BoundedSemanticTextV1::new("a\nb").unwrap_err(),
            SemanticCoreError::InvalidText
        );
        assert_eq!(
            BoundedSemanticTextV1::new("x".repeat(MAX_SEMANTIC_TEXT_BYTES + 1)).unwrap_err(),
            SemanticCoreError::InvalidText
        );
        assert!(BoundedSemanticTextV1::new("x".repeat(MAX_SEMANTIC_TEXT_BYTES)).is_ok());
    }

    #[test]
    fn commitment_hex_round_trip_is_lowercase_canonical() {
        let input = "AA".repeat(32);
        let commitment = Commitment32::from_hex(&input).unwrap();
        assert_eq!(commitment, Commitment32::from_bytes([0xaa; 32]));
        assert_eq!(commitment.to_hex(), "aa".repeat(32));
    }

    #[test]
    fn commitment_rejects_wrong_length_and_non_hex() {
        assert_eq!(
            Commitment32::from_hex("00").unwrap_err(),
            SemanticCoreError::InvalidCommitmentHex
        );
        assert_eq!(
            Commitment32::from_hex(&"gg".repeat(32)).unwrap_err(),
            SemanticCoreError::InvalidCommitmentHex
        );
    }

    #[test]
    fn serde_revalidates_semantic_text() {
        let decoded: Result<SemanticIdV1, _> = serde_json::from_str("\"bad\\nidentifier\"");
        assert!(decoded.is_err());
    }

    #[test]
    fn profile_wire_shape_is_closed() {
        let json = format!(
            r#"{{"id":"schema/personal","revision":1,"digest":"{}","extra":true}}"#,
            digest(7)
        );
        let decoded: Result<SemanticProfileRefV1, _> = serde_json::from_str(&json);
        assert!(decoded.is_err());
    }

    #[test]
    fn schema_reference_preserves_exact_profile_identity() {
        let profile = SemanticProfileRefV1::new("schema/personal", 7, digest(9)).unwrap();
        let schema = SchemaRefV1::new(profile.clone());
        assert_eq!(schema.profile(), &profile);
        assert_eq!(schema.profile().revision(), 7);
        assert_eq!(schema.profile().digest(), digest(9));
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
    fn environment_preserves_profile_roles_exactly() {
        let environment = environment();
        assert_eq!(environment.schema().profile().id().as_str(), "schema/base");
        assert_eq!(
            environment.interpretation_profile().id().as_str(),
            "interpretation/base"
        );
        assert_eq!(environment.identity_profile().id().as_str(), "identity/base");
        assert_eq!(environment.authority_profile().id().as_str(), "authority/base");
        assert_eq!(environment.temporal_profile().id().as_str(), "temporal/base");
        assert_eq!(
            environment.canonicalization_profile().id().as_str(),
            "canonical/domain-v1"
        );
    }

    #[test]
    fn environment_wire_shape_is_closed() {
        let mut value = serde_json::to_value(environment()).unwrap();
        value
            .as_object_mut()
            .unwrap()
            .insert("current".into(), serde_json::Value::Bool(true));
        let decoded: Result<SemanticEnvironmentV1, _> = serde_json::from_value(value);
        assert!(decoded.is_err());
    }

    #[test]
    fn environment_round_trip_preserves_exact_profiles() {
        let environment = environment();
        let encoded = serde_json::to_string(&environment).unwrap();
        let decoded: SemanticEnvironmentV1 = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, environment);
    }

    #[test]
    fn environment_commitment_golden_vector_v1() {
        let environment = environment();
        assert_eq!(
            environment.commitment().to_hex(),
            "72cc14b9f38fc9cb917da7e98d777443d980ad28ca12231cfec0c92919d09d03"
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
            "d78cf4c70f6dffef2285188f98a556c232c67f689300a0acd3d8df21a8c647bf"
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
