use serde::{Deserialize, Serialize};

use crate::{
    canonical::{append_field, append_option_bytes, record_id},
    BlobDescriptorV1, ContentErrorV1, MediaTypeV1,
};

pub const CONTENT_SCHEMA_V1: u16 = 1;
const MAX_BLOBS_PER_OBJECT_V1: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ObjectIdV1(pub [u8; 32]);

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObjectManifestV1 {
    pub schema_version: u16,
    pub id: ObjectIdV1,
    pub blobs: Vec<BlobDescriptorV1>,
    pub total_size_bytes: u64,
    pub object_media_type: Option<MediaTypeV1>,
}

impl ObjectManifestV1 {
    pub fn new(
        blobs: Vec<BlobDescriptorV1>,
        object_media_type: Option<MediaTypeV1>,
    ) -> Result<Self, ContentErrorV1> {
        validate_parts(&blobs, object_media_type.as_ref())?;
        let total_size_bytes = total_size(&blobs)?;
        let id = object_id(&blobs, total_size_bytes, object_media_type.as_ref());
        Ok(Self {
            schema_version: CONTENT_SCHEMA_V1,
            id,
            blobs,
            total_size_bytes,
            object_media_type,
        })
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if self.schema_version != CONTENT_SCHEMA_V1 {
            return Err(ContentErrorV1::UnsupportedSchemaVersion);
        }
        validate_parts(&self.blobs, self.object_media_type.as_ref())?;
        let total = total_size(&self.blobs)?;
        if total != self.total_size_bytes {
            return Err(ContentErrorV1::TotalSizeMismatch);
        }
        if self.id != object_id(&self.blobs, total, self.object_media_type.as_ref()) {
            return Err(ContentErrorV1::IdMismatch);
        }
        Ok(())
    }

    pub fn recompute_id(&self) -> Result<ObjectIdV1, ContentErrorV1> {
        validate_parts(&self.blobs, self.object_media_type.as_ref())?;
        let total = total_size(&self.blobs)?;
        Ok(object_id(
            &self.blobs,
            total,
            self.object_media_type.as_ref(),
        ))
    }
}

fn validate_parts(
    blobs: &[BlobDescriptorV1],
    media_type: Option<&MediaTypeV1>,
) -> Result<(), ContentErrorV1> {
    if blobs.is_empty() {
        return Err(ContentErrorV1::EmptyManifest);
    }
    if blobs.len() > MAX_BLOBS_PER_OBJECT_V1 {
        return Err(ContentErrorV1::TooManyBlobs);
    }
    for blob in blobs {
        blob.validate()?;
    }
    if let Some(media_type) = media_type {
        media_type.validate()?;
    }
    Ok(())
}

fn total_size(blobs: &[BlobDescriptorV1]) -> Result<u64, ContentErrorV1> {
    blobs.iter().try_fold(0u64, |total, blob| {
        total
            .checked_add(blob.size_bytes)
            .ok_or(ContentErrorV1::TotalSizeOverflow)
    })
}

fn object_id(
    blobs: &[BlobDescriptorV1],
    total_size_bytes: u64,
    media_type: Option<&MediaTypeV1>,
) -> ObjectIdV1 {
    let mut body = Vec::new();
    body.extend_from_slice(&(blobs.len() as u32).to_be_bytes());
    for blob in blobs {
        append_field(&mut body, &blob.canonical_bytes());
    }
    body.extend_from_slice(&total_size_bytes.to_be_bytes());
    append_option_bytes(&mut body, media_type.map(|m| m.as_str().as_bytes()));
    ObjectIdV1(record_id("object-manifest", CONTENT_SCHEMA_V1, &body))
}
