// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Deterministic canonical bytes for security-sensitive Mycelix protocols.
//!
//! The format is deliberately small and restrictive. It avoids signing generic
//! JSON/MessagePack/Serde output, where map ordering, number representation, or
//! library changes can create cross-implementation ambiguity.
//!
//! ## Canonical v1 wire format
//!
//! ```text
//! magic              = b"MYCELIX-CANONICAL\0\x01"
//! domain_len          = u16 big-endian
//! domain              = ASCII bytes
//! schema_version      = u16 big-endian (non-zero)
//! fields...           = strictly increasing field tags
//!
//! field =
//!   tag                u16 big-endian, non-zero
//!   type               u8
//!   payload_len        u32 big-endian
//!   payload            exact bytes for that type
//! ```
//!
//! Security-sensitive schemas should use machine identifiers (`field_ascii`) and
//! typed integers/digests. `field_utf8` is byte-exact UTF-8: this crate does not
//! silently normalize human text. There is intentionally no float or map type.

use sha2::{Digest, Sha256};
use std::fmt;

pub const MAGIC: &[u8] = b"MYCELIX-CANONICAL\0\x01";
pub const FORMAT_VERSION: u8 = 1;
pub const MAX_DOMAIN_BYTES: usize = 128;
pub const MAX_FIELD_BYTES: usize = 1_048_576;
pub const MAX_DOCUMENT_BYTES: usize = 4_194_304;
pub const MAX_LIST_ITEMS: usize = 4096;

const TYPE_ASCII: u8 = 1;
const TYPE_UTF8: u8 = 2;
const TYPE_U64: u8 = 3;
const TYPE_U32: u8 = 4;
const TYPE_U16: u8 = 5;
const TYPE_BOOL: u8 = 6;
const TYPE_DIGEST32: u8 = 7;
const TYPE_BYTES: u8 = 8;
const TYPE_ASCII_LIST: u8 = 9;
const TYPE_NESTED: u8 = 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct CanonicalDigest(pub [u8; 32]);

impl CanonicalDigest {
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    pub fn to_hex(&self) -> String {
        hex_bytes(&self.0)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanonicalDocument {
    pub bytes: Vec<u8>,
    pub sha256: CanonicalDigest,
}

impl CanonicalDocument {
    pub fn verify_digest(&self) -> bool {
        sha256_digest(&self.bytes) == self.sha256
    }
}

/// Stateful writer that enforces monotonically increasing field tags.
///
/// Field tags are part of the schema contract. A field MUST NOT be silently
/// repurposed in a later schema version. Additive/reinterpreted changes require
/// a new schema version and conformance vectors.
pub struct CanonicalBuilder {
    bytes: Vec<u8>,
    last_tag: u16,
}

impl CanonicalBuilder {
    pub fn new(domain: &str, schema_version: u16) -> Result<Self, CanonicalError> {
        if schema_version == 0 {
            return Err(CanonicalError::ZeroSchemaVersion);
        }
        validate_ascii(domain, "domain")?;
        if domain.len() > MAX_DOMAIN_BYTES {
            return Err(CanonicalError::DomainTooLong);
        }
        if domain.len() > u16::MAX as usize {
            return Err(CanonicalError::DomainTooLong);
        }

        let mut bytes = Vec::with_capacity(MAGIC.len() + domain.len() + 16);
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&(domain.len() as u16).to_be_bytes());
        bytes.extend_from_slice(domain.as_bytes());
        bytes.extend_from_slice(&schema_version.to_be_bytes());

        Ok(Self { bytes, last_tag: 0 })
    }

    pub fn field_ascii(&mut self, tag: u16, value: &str) -> Result<&mut Self, CanonicalError> {
        validate_ascii(value, "ascii field")?;
        self.push_field(tag, TYPE_ASCII, value.as_bytes())?;
        Ok(self)
    }

    /// Byte-exact UTF-8. No Unicode normalization is performed.
    pub fn field_utf8(&mut self, tag: u16, value: &str) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_UTF8, value.as_bytes())?;
        Ok(self)
    }

    pub fn field_u64(&mut self, tag: u16, value: u64) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_U64, &value.to_be_bytes())?;
        Ok(self)
    }

    pub fn field_u32(&mut self, tag: u16, value: u32) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_U32, &value.to_be_bytes())?;
        Ok(self)
    }

    pub fn field_u16(&mut self, tag: u16, value: u16) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_U16, &value.to_be_bytes())?;
        Ok(self)
    }

    pub fn field_bool(&mut self, tag: u16, value: bool) -> Result<&mut Self, CanonicalError> {
        let payload = [u8::from(value)];
        self.push_field(tag, TYPE_BOOL, &payload)?;
        Ok(self)
    }

    pub fn field_digest32(
        &mut self,
        tag: u16,
        value: &[u8; 32],
    ) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_DIGEST32, value)?;
        Ok(self)
    }

    pub fn field_bytes(&mut self, tag: u16, value: &[u8]) -> Result<&mut Self, CanonicalError> {
        self.push_field(tag, TYPE_BYTES, value)?;
        Ok(self)
    }

    /// Canonical list of ASCII strings. Item order is semantically significant.
    /// Schemas that model sets must sort/deduplicate before calling this method.
    pub fn field_ascii_list(
        &mut self,
        tag: u16,
        values: &[String],
    ) -> Result<&mut Self, CanonicalError> {
        if values.len() > MAX_LIST_ITEMS {
            return Err(CanonicalError::TooManyListItems);
        }
        let mut payload = Vec::new();
        payload.extend_from_slice(&(values.len() as u32).to_be_bytes());
        for value in values {
            validate_ascii(value, "ASCII list item")?;
            if value.len() > u32::MAX as usize {
                return Err(CanonicalError::FieldTooLong);
            }
            payload.extend_from_slice(&(value.len() as u32).to_be_bytes());
            payload.extend_from_slice(value.as_bytes());
        }
        self.push_field(tag, TYPE_ASCII_LIST, &payload)?;
        Ok(self)
    }

    /// Embed a complete canonical document as a typed nested value.
    /// The nested magic/domain/schema remain signed bytes and cannot be confused
    /// with an untyped raw byte field.
    pub fn field_nested(
        &mut self,
        tag: u16,
        nested: &CanonicalDocument,
    ) -> Result<&mut Self, CanonicalError> {
        if !nested.verify_digest() {
            return Err(CanonicalError::NestedDigestMismatch);
        }
        self.push_field(tag, TYPE_NESTED, &nested.bytes)?;
        Ok(self)
    }

    pub fn finish(self) -> Result<CanonicalDocument, CanonicalError> {
        if self.bytes.len() > MAX_DOCUMENT_BYTES {
            return Err(CanonicalError::DocumentTooLong);
        }
        let sha256 = sha256_digest(&self.bytes);
        Ok(CanonicalDocument {
            bytes: self.bytes,
            sha256,
        })
    }

    fn push_field(
        &mut self,
        tag: u16,
        type_tag: u8,
        payload: &[u8],
    ) -> Result<(), CanonicalError> {
        if tag == 0 {
            return Err(CanonicalError::ZeroFieldTag);
        }
        if tag <= self.last_tag {
            return Err(CanonicalError::FieldTagsNotIncreasing {
                previous: self.last_tag,
                next: tag,
            });
        }
        if payload.len() > MAX_FIELD_BYTES || payload.len() > u32::MAX as usize {
            return Err(CanonicalError::FieldTooLong);
        }

        let projected = self
            .bytes
            .len()
            .checked_add(2 + 1 + 4)
            .and_then(|len| len.checked_add(payload.len()))
            .ok_or(CanonicalError::LengthOverflow)?;
        if projected > MAX_DOCUMENT_BYTES {
            return Err(CanonicalError::DocumentTooLong);
        }

        self.bytes.extend_from_slice(&tag.to_be_bytes());
        self.bytes.push(type_tag);
        self.bytes
            .extend_from_slice(&(payload.len() as u32).to_be_bytes());
        self.bytes.extend_from_slice(payload);
        self.last_tag = tag;
        Ok(())
    }
}

pub fn sha256_digest(bytes: &[u8]) -> CanonicalDigest {
    let digest = Sha256::digest(bytes);
    let mut out = [0u8; 32];
    out.copy_from_slice(&digest);
    CanonicalDigest(out)
}

/// Decode hexadecimal strictly for conformance tooling.
pub fn decode_hex(input: &str) -> Result<Vec<u8>, CanonicalError> {
    if input.len() % 2 != 0 {
        return Err(CanonicalError::InvalidHex);
    }
    let mut out = Vec::with_capacity(input.len() / 2);
    let bytes = input.as_bytes();
    for pair in bytes.chunks_exact(2) {
        let high = hex_nibble(pair[0]).ok_or(CanonicalError::InvalidHex)?;
        let low = hex_nibble(pair[1]).ok_or(CanonicalError::InvalidHex)?;
        out.push((high << 4) | low);
    }
    Ok(out)
}

fn hex_nibble(value: u8) -> Option<u8> {
    match value {
        b'0'..=b'9' => Some(value - b'0'),
        b'a'..=b'f' => Some(value - b'a' + 10),
        b'A'..=b'F' => Some(value - b'A' + 10),
        _ => None,
    }
}

fn hex_bytes(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn validate_ascii(value: &str, field: &'static str) -> Result<(), CanonicalError> {
    if value.is_empty() {
        return Err(CanonicalError::Empty(field));
    }
    if !value.is_ascii() {
        return Err(CanonicalError::NonAscii(field));
    }
    if value.bytes().any(|byte| byte < 0x20 || byte == 0x7f) {
        return Err(CanonicalError::AsciiControlCharacter(field));
    }
    Ok(())
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CanonicalError {
    Empty(&'static str),
    NonAscii(&'static str),
    AsciiControlCharacter(&'static str),
    ZeroSchemaVersion,
    DomainTooLong,
    ZeroFieldTag,
    FieldTagsNotIncreasing { previous: u16, next: u16 },
    FieldTooLong,
    DocumentTooLong,
    TooManyListItems,
    LengthOverflow,
    NestedDigestMismatch,
    InvalidHex,
}

impl fmt::Display for CanonicalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Empty(field) => write!(f, "{field} must not be empty"),
            Self::NonAscii(field) => write!(f, "{field} must be ASCII"),
            Self::AsciiControlCharacter(field) => {
                write!(f, "{field} contains an ASCII control character")
            }
            Self::ZeroSchemaVersion => write!(f, "schema version must be non-zero"),
            Self::DomainTooLong => write!(f, "canonical domain is too long"),
            Self::ZeroFieldTag => write!(f, "field tag zero is reserved"),
            Self::FieldTagsNotIncreasing { previous, next } => write!(
                f,
                "field tags must be strictly increasing (previous={previous}, next={next})"
            ),
            Self::FieldTooLong => write!(f, "canonical field exceeds maximum size"),
            Self::DocumentTooLong => write!(f, "canonical document exceeds maximum size"),
            Self::TooManyListItems => write!(f, "canonical list has too many items"),
            Self::LengthOverflow => write!(f, "canonical length arithmetic overflow"),
            Self::NestedDigestMismatch => write!(f, "nested canonical document digest mismatch"),
            Self::InvalidHex => write!(f, "invalid hexadecimal string"),
        }
    }
}

impl std::error::Error for CanonicalError {}

#[cfg(test)]
mod tests {
    use super::*;

    const VECTOR_HEX: &str = "4d5943454c49582d43414e4f4e4943414c00010014676f7665726e616e63652e657865637574696f6e0001000101000000064d49502d3432000207000000201111111111111111111111111111111111111111111111111111111111111111000303000000080000000000000007";
    const VECTOR_SHA256: &str =
        "00019268e6e8cd9b78f98588655e000a14653eecf7a10d3c8d93f1b991c7d3ec";

    fn governance_vector() -> CanonicalDocument {
        let mut builder = CanonicalBuilder::new("governance.execution", 1).unwrap();
        builder.field_ascii(1, "MIP-42").unwrap();
        builder.field_digest32(2, &[0x11; 32]).unwrap();
        builder.field_u64(3, 7).unwrap();
        builder.finish().unwrap()
    }

    #[test]
    fn published_governance_vector_matches_exact_bytes_and_digest() {
        let document = governance_vector();
        assert_eq!(document.bytes, decode_hex(VECTOR_HEX).unwrap());
        assert_eq!(document.sha256.to_hex(), VECTOR_SHA256);
        assert!(document.verify_digest());
    }

    #[test]
    fn field_order_is_part_of_the_schema_and_out_of_order_is_rejected() {
        let mut builder = CanonicalBuilder::new("test.order", 1).unwrap();
        builder.field_u64(2, 1).unwrap();
        assert_eq!(
            builder.field_u64(1, 2).unwrap_err(),
            CanonicalError::FieldTagsNotIncreasing {
                previous: 2,
                next: 1
            }
        );
    }

    #[test]
    fn duplicate_field_tag_is_rejected() {
        let mut builder = CanonicalBuilder::new("test.duplicate", 1).unwrap();
        builder.field_bool(1, true).unwrap();
        assert_eq!(
            builder.field_bool(1, false).unwrap_err(),
            CanonicalError::FieldTagsNotIncreasing {
                previous: 1,
                next: 1
            }
        );
    }

    #[test]
    fn domains_prevent_cross_protocol_signature_confusion() {
        let mut a = CanonicalBuilder::new("governance.execution", 1).unwrap();
        a.field_ascii(1, "MIP-42").unwrap();
        let a = a.finish().unwrap();

        let mut b = CanonicalBuilder::new("identity.recovery", 1).unwrap();
        b.field_ascii(1, "MIP-42").unwrap();
        let b = b.finish().unwrap();

        assert_ne!(a.bytes, b.bytes);
        assert_ne!(a.sha256, b.sha256);
    }

    #[test]
    fn schema_version_changes_signed_bytes() {
        let mut a = CanonicalBuilder::new("test.version", 1).unwrap();
        a.field_u16(1, 7).unwrap();
        let a = a.finish().unwrap();

        let mut b = CanonicalBuilder::new("test.version", 2).unwrap();
        b.field_u16(1, 7).unwrap();
        let b = b.finish().unwrap();

        assert_ne!(a.sha256, b.sha256);
    }

    #[test]
    fn integers_are_big_endian_and_typed() {
        let mut u16_doc = CanonicalBuilder::new("test.integer", 1).unwrap();
        u16_doc.field_u16(1, 0x0102).unwrap();
        let u16_doc = u16_doc.finish().unwrap();
        assert!(u16_doc.bytes.ends_with(&[0x01, 0x02]));

        let mut u32_doc = CanonicalBuilder::new("test.integer", 1).unwrap();
        u32_doc.field_u32(1, 0x0000_0102).unwrap();
        let u32_doc = u32_doc.finish().unwrap();
        assert_ne!(u16_doc.sha256, u32_doc.sha256);
    }

    #[test]
    fn nested_document_is_bound_with_its_own_domain_and_schema() {
        let mut inner = CanonicalBuilder::new("inner.policy", 1).unwrap();
        inner.field_ascii(1, "policy:v1").unwrap();
        let inner = inner.finish().unwrap();

        let mut outer = CanonicalBuilder::new("outer.authorization", 1).unwrap();
        outer.field_nested(1, &inner).unwrap();
        let outer = outer.finish().unwrap();

        assert!(outer.bytes.windows(MAGIC.len()).any(|window| window == MAGIC));
        assert!(outer.verify_digest());
    }

    #[test]
    fn tampered_nested_document_is_rejected() {
        let mut inner = CanonicalBuilder::new("inner.policy", 1).unwrap();
        inner.field_ascii(1, "policy:v1").unwrap();
        let mut inner = inner.finish().unwrap();
        *inner.bytes.last_mut().unwrap() ^= 1;

        let mut outer = CanonicalBuilder::new("outer.authorization", 1).unwrap();
        assert_eq!(
            outer.field_nested(1, &inner).unwrap_err(),
            CanonicalError::NestedDigestMismatch
        );
    }

    #[test]
    fn machine_ascii_rejects_unicode_and_control_characters() {
        let mut builder = CanonicalBuilder::new("test.ascii", 1).unwrap();
        assert!(matches!(
            builder.field_ascii(1, "café"),
            Err(CanonicalError::NonAscii(_))
        ));

        let mut builder = CanonicalBuilder::new("test.ascii", 1).unwrap();
        assert!(matches!(
            builder.field_ascii(1, "abc\ndef"),
            Err(CanonicalError::AsciiControlCharacter(_))
        ));
    }

    #[test]
    fn utf8_is_explicitly_byte_exact_not_silently_normalized() {
        let precomposed = "é";
        let decomposed = "e\u{301}";

        let mut a = CanonicalBuilder::new("test.utf8", 1).unwrap();
        a.field_utf8(1, precomposed).unwrap();
        let a = a.finish().unwrap();

        let mut b = CanonicalBuilder::new("test.utf8", 1).unwrap();
        b.field_utf8(1, decomposed).unwrap();
        let b = b.finish().unwrap();

        assert_ne!(a.sha256, b.sha256);
    }

    #[test]
    fn ascii_list_order_is_explicitly_semantic() {
        let mut a = CanonicalBuilder::new("test.list", 1).unwrap();
        a.field_ascii_list(1, &["alice".into(), "bob".into()])
            .unwrap();
        let a = a.finish().unwrap();

        let mut b = CanonicalBuilder::new("test.list", 1).unwrap();
        b.field_ascii_list(1, &["bob".into(), "alice".into()])
            .unwrap();
        let b = b.finish().unwrap();

        assert_ne!(a.sha256, b.sha256);
    }

    #[test]
    fn hex_decoder_rejects_odd_or_invalid_input() {
        assert_eq!(decode_hex("abc").unwrap_err(), CanonicalError::InvalidHex);
        assert_eq!(decode_hex("zz").unwrap_err(), CanonicalError::InvalidHex);
    }
}
