use serde::{Deserialize, Serialize};

use crate::{
    canonical::{append_field, append_option_bytes},
    ContentDigestV1, ContentErrorV1, DigestAlgorithmV1,
};

const MAX_MEDIA_TYPE_BYTES_V1: usize = 255;

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct MediaTypeV1(String);

impl MediaTypeV1 {
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
        let bytes = self.0.as_bytes();
        if bytes.is_empty()
            || bytes.len() > MAX_MEDIA_TYPE_BYTES_V1
            || !bytes.is_ascii()
            || !self.0.contains('/')
            || bytes.iter().any(|b| b.is_ascii_control())
        {
            return Err(ContentErrorV1::InvalidMediaType);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct BlobDescriptorV1 {
    pub digest: ContentDigestV1,
    pub size_bytes: u64,
    pub media_type: Option<MediaTypeV1>,
}

impl BlobDescriptorV1 {
    pub fn from_bytes(
        algorithm: DigestAlgorithmV1,
        bytes: &[u8],
        media_type: Option<MediaTypeV1>,
    ) -> Self {
        Self {
            digest: ContentDigestV1::compute(algorithm, bytes),
            size_bytes: bytes.len() as u64,
            media_type,
        }
    }

    pub fn validate(&self) -> Result<(), ContentErrorV1> {
        if let Some(media_type) = &self.media_type {
            media_type.validate()?;
        }
        Ok(())
    }

    pub(crate) fn canonical_bytes(&self) -> Vec<u8> {
        let digest = self.digest.canonical_bytes();
        let mut out = Vec::new();
        append_field(&mut out, &digest);
        out.extend_from_slice(&self.size_bytes.to_be_bytes());
        append_option_bytes(
            &mut out,
            self.media_type.as_ref().map(|m| m.as_str().as_bytes()),
        );
        out
    }
}
