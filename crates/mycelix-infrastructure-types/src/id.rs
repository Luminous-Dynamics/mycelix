use serde::{Deserialize, Serialize};

use crate::InfrastructureErrorV1;

const ID_MAGIC: &[u8] = b"MYCELIX-INFRA-ID\0";
const PAYLOAD_MAGIC: &[u8] = b"MYCELIX-INFRA-PAYLOAD\0";

/// Stable 256-bit identifier used by the infrastructure envelope family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct StableIdV1(pub [u8; 32]);

impl StableIdV1 {
    pub const ZERO: Self = Self([0; 32]);

    /// Derive a domain-separated identifier from canonical byte fields.
    pub fn derive(
        domain: &str,
        schema_version: u16,
        fields: &[&[u8]],
    ) -> Result<Self, InfrastructureErrorV1> {
        if !is_valid_token(domain) {
            return Err(InfrastructureErrorV1::InvalidSchemaTag);
        }

        let mut hasher = blake3::Hasher::new();
        hasher.update(ID_MAGIC);
        put_field(&mut hasher, domain.as_bytes());
        hasher.update(&schema_version.to_be_bytes());
        hasher.update(&(fields.len() as u64).to_be_bytes());
        for field in fields {
            put_field(&mut hasher, field);
        }
        Ok(Self(*hasher.finalize().as_bytes()))
    }
}

/// Opaque application principal. Adapters map Holochain agents, keys, accounts, etc. into it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PartyIdV1(pub [u8; 32]);

impl PartyIdV1 {
    pub const ZERO: Self = Self([0; 32]);
}

/// Commitment to a payload encoded by the payload's owning schema.
///
/// The infrastructure crate never serializes generic payloads itself. This prevents
/// unordered maps or transport-specific encodings from silently changing stable IDs.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PayloadCommitmentV1 {
    pub schema: String,
    pub digest: StableIdV1,
}

impl PayloadCommitmentV1 {
    pub fn from_canonical_bytes(
        schema: impl Into<String>,
        canonical_bytes: &[u8],
    ) -> Result<Self, InfrastructureErrorV1> {
        let schema = schema.into();
        if !is_valid_token(&schema) {
            return Err(InfrastructureErrorV1::InvalidSchemaTag);
        }

        let mut hasher = blake3::Hasher::new();
        hasher.update(PAYLOAD_MAGIC);
        put_field(&mut hasher, schema.as_bytes());
        put_field(&mut hasher, canonical_bytes);
        Ok(Self {
            schema,
            digest: StableIdV1(*hasher.finalize().as_bytes()),
        })
    }

    pub fn validate(&self) -> Result<(), InfrastructureErrorV1> {
        if is_valid_token(&self.schema) {
            Ok(())
        } else {
            Err(InfrastructureErrorV1::InvalidSchemaTag)
        }
    }

    pub(crate) fn canonical_fields(&self) -> [&[u8]; 2] {
        [self.schema.as_bytes(), &self.digest.0]
    }
}

pub(crate) fn is_valid_token(token: &str) -> bool {
    let bytes = token.as_bytes();
    !bytes.is_empty()
        && bytes.len() <= 64
        && bytes[0].is_ascii_lowercase()
        && bytes.iter().all(|b| {
            b.is_ascii_lowercase()
                || b.is_ascii_digit()
                || matches!(*b, b'.' | b'_' | b'/' | b'-' | b':')
        })
}

fn put_field(hasher: &mut blake3::Hasher, field: &[u8]) {
    hasher.update(&(field.len() as u64).to_be_bytes());
    hasher.update(field);
}
