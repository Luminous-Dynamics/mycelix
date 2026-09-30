// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Transport-neutral semantic contracts for Pulse social features.
//!
//! These types deliberately do not grant authority or imply storage.
//! Holochain entries, UI projections, and federation adapters must establish
//! their own evidence and authorization boundaries around these values.

use serde::{Deserialize, Serialize};

pub const SOCIAL_SCHEMA_VERSION_V1: u8 = 1;
pub const MAX_OBJECT_ID_BYTES: usize = 256;
pub const MAX_REVISION_ID_BYTES: usize = 256;
pub const MAX_PROVENANCE_BYTES: usize = 256;
pub const MAX_EXTERNAL_ID_BYTES: usize = 1024;
pub const MAX_CUSTOM_AUDIENCE_BYTES: usize = 1024;

/// Logical identity of a social object. This is not an ActionHash or UI key.
#[derive(Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct SocialObjectIdV1(pub String);

/// Exact semantic revision of one logical social object.
#[derive(Clone, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub struct SocialRevisionV1(pub String);

/// The kind of semantic object being referenced.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum SocialObjectKind {
    ChatMessage,
    Post,
    Comment,
    Reaction,
    Profile,
    Follow,
    Friendship,
    Space,
    Membership,
    Report,
}

/// Audience intent. This is descriptive policy input, not authorization.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub enum SocialAudience {
    Public,
    Followers,
    SpaceMembers,
    Direct,
    Custom(String),
}

/// Provenance class for a semantic reference.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum SocialProvenance {
    MycelixCanonical,
    ProviderObservation,
    LocalProjection,
}

/// Stable semantic reference. It intentionally does not contain UI state.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SocialObjectRefV1 {
    pub schema_version: u8,
    pub object_id: SocialObjectIdV1,
    pub revision: SocialRevisionV1,
    pub kind: SocialObjectKind,
    pub provenance: SocialProvenance,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum SocialRelationKind {
    Follow,
    Friendship,
    Subscription,
    Membership,
}

/// Relationship observations never grant a capability by themselves.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SocialRelationV1 {
    pub schema_version: u8,
    pub relation: SocialRelationKind,
    pub subject: String,
    pub target: String,
    pub provenance: SocialProvenance,
}

/// An externally observed social object. Provider identity remains separate
/// from Mycelix identity and semantic object identity.
#[derive(Clone, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SocialObservationV1 {
    pub schema_version: u8,
    pub provider: String,
    pub external_id: String,
    pub observed_at_micros: i64,
    pub object: SocialObjectRefV1,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SocialContractError {
    UnsupportedSchemaVersion(u8),
    EmptyField(&'static str),
    FieldTooLong(&'static str),
    InvalidTimestamp,
    InvalidCustomAudience,
}

impl SocialObjectIdV1 {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        validate_bounded(&self.0, "object_id", MAX_OBJECT_ID_BYTES)
    }
}

impl SocialRevisionV1 {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        validate_bounded(&self.0, "revision", MAX_REVISION_ID_BYTES)
    }
}

impl SocialAudience {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        if let Self::Custom(value) = self {
            validate_bounded(value, "custom_audience", MAX_CUSTOM_AUDIENCE_BYTES)
                .map_err(|error| match error {
                    SocialContractError::EmptyField(_) => {
                        SocialContractError::InvalidCustomAudience
                    }
                    other => other,
                })?;
        }
        Ok(())
    }
}

impl SocialObjectRefV1 {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        if self.schema_version != SOCIAL_SCHEMA_VERSION_V1 {
            return Err(SocialContractError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        self.object_id.validate()?;
        self.revision.validate()?;
        Ok(())
    }
}

impl SocialObservationV1 {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        if self.schema_version != SOCIAL_SCHEMA_VERSION_V1 {
            return Err(SocialContractError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        validate_bounded(&self.provider, "provider", MAX_PROVENANCE_BYTES)?;
        validate_bounded(&self.external_id, "external_id", MAX_EXTERNAL_ID_BYTES)?;
        if self.observed_at_micros < 0 {
            return Err(SocialContractError::InvalidTimestamp);
        }
        self.object.validate()
    }
}

impl SocialRelationV1 {
    pub fn validate(&self) -> Result<(), SocialContractError> {
        if self.schema_version != SOCIAL_SCHEMA_VERSION_V1 {
            return Err(SocialContractError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        validate_bounded(&self.subject, "subject", MAX_OBJECT_ID_BYTES)?;
        validate_bounded(&self.target, "target", MAX_OBJECT_ID_BYTES)?;
        Ok(())
    }
}

fn validate_bounded(
    value: &str,
    field: &'static str,
    max_bytes: usize,
) -> Result<(), SocialContractError> {
    if value.trim().is_empty() {
        return Err(SocialContractError::EmptyField(field));
    }
    if value.len() > max_bytes {
        return Err(SocialContractError::FieldTooLong(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object_ref() -> SocialObjectRefV1 {
        SocialObjectRefV1 {
            schema_version: SOCIAL_SCHEMA_VERSION_V1,
            object_id: SocialObjectIdV1("post-001".into()),
            revision: SocialRevisionV1("rev-001".into()),
            kind: SocialObjectKind::Post,
            provenance: SocialProvenance::MycelixCanonical,
        }
    }

    #[test]
    fn chat_message_kind_has_stable_serde_name() {
        let encoded = serde_json::to_string(&SocialObjectKind::ChatMessage).unwrap();
        assert_eq!(encoded, "\"ChatMessage\"");
        assert_eq!(
            serde_json::from_str::<SocialObjectKind>(&encoded).unwrap(),
            SocialObjectKind::ChatMessage
        );
    }

    #[test]
    fn valid_reference_is_accepted() {
        assert!(object_ref().validate().is_ok());
    }

    #[test]
    fn logical_identity_and_revision_are_distinct() {
        let a = object_ref();
        let mut b = a.clone();
        b.revision = SocialRevisionV1("rev-002".into());
        assert_eq!(a.object_id, b.object_id);
        assert_ne!(a.revision, b.revision);
    }

    #[test]
    fn unknown_schema_version_fails_closed() {
        let mut reference = object_ref();
        reference.schema_version = 2;
        assert_eq!(
            reference.validate(),
            Err(SocialContractError::UnsupportedSchemaVersion(2))
        );
    }

    #[test]
    fn empty_identity_fails_closed() {
        let mut reference = object_ref();
        reference.object_id = SocialObjectIdV1(" ".into());
        assert_eq!(
            reference.validate(),
            Err(SocialContractError::EmptyField("object_id"))
        );
    }

    #[test]
    fn oversized_identity_fails_closed() {
        let mut reference = object_ref();
        reference.object_id = SocialObjectIdV1("x".repeat(MAX_OBJECT_ID_BYTES + 1));
        assert_eq!(
            reference.validate(),
            Err(SocialContractError::FieldTooLong("object_id"))
        );
    }

    #[test]
    fn nested_observation_reference_must_validate() {
        let mut observation = SocialObservationV1 {
            schema_version: SOCIAL_SCHEMA_VERSION_V1,
            provider: "activitypub".into(),
            external_id: "object-1".into(),
            observed_at_micros: 1,
            object: object_ref(),
        };
        observation.object.schema_version = 2;
        assert_eq!(
            observation.validate(),
            Err(SocialContractError::UnsupportedSchemaVersion(2))
        );
    }

    #[test]
    fn nested_observation_identity_must_validate() {
        let mut observation = SocialObservationV1 {
            schema_version: SOCIAL_SCHEMA_VERSION_V1,
            provider: "activitypub".into(),
            external_id: "object-1".into(),
            observed_at_micros: 1,
            object: object_ref(),
        };
        observation.object.object_id = SocialObjectIdV1(" ".into());
        assert_eq!(
            observation.validate(),
            Err(SocialContractError::EmptyField("object_id"))
        );
    }

    #[test]
    fn negative_observation_time_fails_closed() {
        let observation = SocialObservationV1 {
            schema_version: SOCIAL_SCHEMA_VERSION_V1,
            provider: "activitypub".into(),
            external_id: "https://example.test/objects/1".into(),
            observed_at_micros: -1,
            object: object_ref(),
        };
        assert_eq!(
            observation.validate(),
            Err(SocialContractError::InvalidTimestamp)
        );
    }

    #[test]
    fn relation_validation_does_not_authorize() {
        let relation = SocialRelationV1 {
            schema_version: SOCIAL_SCHEMA_VERSION_V1,
            relation: SocialRelationKind::Membership,
            subject: "agent-a".into(),
            target: "space-1".into(),
            provenance: SocialProvenance::MycelixCanonical,
        };
        assert!(relation.validate().is_ok());
        // Deliberately no capability/authorization field exists.
    }

    #[test]
    fn serde_unknown_fields_fail_closed() {
        let encoded = r#"{"schema_version":1,"object_id":"post-001","revision":"rev-001","kind":"Post","provenance":"MycelixCanonical","capability":"admin"}"#;
        let decoded = serde_json::from_str::<SocialObjectRefV1>(encoded);
        assert!(decoded.is_err());
    }

    #[test]
    fn serde_unknown_enum_value_fails() {
        let encoded = r#"{"schema_version":1,"relation":"Blocked","subject":"a","target":"b","provenance":"MycelixCanonical"}"#;
        let decoded = serde_json::from_str::<SocialRelationV1>(encoded);
        assert!(decoded.is_err());
    }
}
