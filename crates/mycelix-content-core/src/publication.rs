use mycelix_infrastructure_types::{PartyIdV1, UnixMillisV1};
use serde::{Deserialize, Serialize};

use crate::{
    canonical::{append_option_bytes, record_id},
    ContentErrorV1, ObjectIdV1, CONTENT_SCHEMA_V1,
};

const MAX_LOGICAL_NAME_BYTES_V1: usize = 256;
const MAX_VERSION_LABEL_BYTES_V1: usize = 128;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct LogicalNameV1(String);

impl LogicalNameV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, ContentErrorV1> {
        let value = value.into();
        let candidate = Self(value);
        candidate.validate()?;
        Ok(candidate)
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.0.is_empty()
            || self.0.len() > MAX_LOGICAL_NAME_BYTES_V1
            || self.0.chars().any(char::is_control)
        {
            return Err(ContentErrorV1::InvalidLogicalName);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct VersionLabelV1(String);

impl VersionLabelV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, ContentErrorV1> {
        let value = value.into();
        let candidate = Self(value);
        candidate.validate()?;
        Ok(candidate)
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.0.is_empty()
            || self.0.len() > MAX_VERSION_LABEL_BYTES_V1
            || self.0.chars().any(char::is_control)
        {
            return Err(ContentErrorV1::InvalidVersionLabel);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PublicationIdV1(pub [u8; 32]);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PublicationRecordV1 {
    pub schema_version: u16,
    pub id: PublicationIdV1,
    pub object_id: ObjectIdV1,
    pub logical_name: Option<LogicalNameV1>,
    pub version: Option<VersionLabelV1>,
    pub publisher: PartyIdV1,
    pub published_at: UnixMillisV1,
    pub previous: Option<PublicationIdV1>,
}

impl PublicationRecordV1 {
    pub fn new(
        object_id: ObjectIdV1,
        logical_name: Option<LogicalNameV1>,
        version: Option<VersionLabelV1>,
        publisher: PartyIdV1,
        published_at: UnixMillisV1,
        previous: Option<PublicationIdV1>,
    ) -> Result<Self, ContentErrorV1> {
        validate_fields(logical_name.as_ref(), version.as_ref())?;
        let id = publication_id(
            object_id,
            logical_name.as_ref(),
            version.as_ref(),
            publisher,
            published_at,
            previous,
        );
        Ok(Self {
            schema_version: CONTENT_SCHEMA_V1,
            id,
            object_id,
            logical_name,
            version,
            publisher,
            published_at,
            previous,
        })
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.schema_version != CONTENT_SCHEMA_V1 {
            return Err(ContentErrorV1::UnsupportedSchemaVersion);
        }
        validate_fields(self.logical_name.as_ref(), self.version.as_ref())?;
        if self.id != self.recompute_id()? {
            return Err(ContentErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<PublicationIdV1, ContentErrorV1> {
        validate_fields(self.logical_name.as_ref(), self.version.as_ref())?;
        Ok(publication_id(
            self.object_id,
            self.logical_name.as_ref(),
            self.version.as_ref(),
            self.publisher,
            self.published_at,
            self.previous,
        ))
    }
}

fn validate_fields(
    name: Option<&LogicalNameV1>,
    version: Option<&VersionLabelV1>,
) -> Result<(), ContentErrorV1> {
    if let Some(name) = name {
        name.validate()?;
    }
    if let Some(version) = version {
        version.validate()?;
    }
    Ok(())
}

fn publication_id(
    object_id: ObjectIdV1,
    logical_name: Option<&LogicalNameV1>,
    version: Option<&VersionLabelV1>,
    publisher: PartyIdV1,
    published_at: UnixMillisV1,
    previous: Option<PublicationIdV1>,
) -> PublicationIdV1 {
    let mut body = Vec::new();
    body.extend_from_slice(&object_id.0);
    append_option_bytes(&mut body, logical_name.map(|v| v.as_str().as_bytes()));
    append_option_bytes(&mut body, version.map(|v| v.as_str().as_bytes()));
    body.extend_from_slice(&publisher.0);
    body.extend_from_slice(&published_at.0.to_be_bytes());
    match previous {
        Some(previous) => {
            body.push(1);
            body.extend_from_slice(&previous.0);
        }
        None => body.push(0),
    }
    PublicationIdV1(record_id("publication", CONTENT_SCHEMA_V1, &body))
}
