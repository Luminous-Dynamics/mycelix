//! ASSURE-V1 L0 canonical CBOR admission.
//!
//! This module proves only that exact bytes satisfy the frozen codec profile
//! under explicit resource limits. It does not validate assurance graphs,
//! evaluate claims, establish authority, or emit positive qualification.

use std::collections::BTreeSet;

use cbor2::Value;
use serde::Deserialize;

/// Conservative decode limits for ASSURE-V1 L0.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodeLimits {
    pub max_input_bytes: u32,
    /// Caller-controlled parser recursion is intentionally bounded to u8 so
    /// untrusted inputs cannot request an unbounded stack budget.
    pub max_nesting_depth: u8,
    pub max_total_items: u32,
    pub max_array_len: u32,
    pub max_map_len: u32,
    pub max_bytes_len: u32,
    pub max_text_bytes: u32,
}

impl DecodeLimits {
    pub const V1_INITIAL: Self = Self {
        max_input_bytes: 16 * 1024 * 1024,
        max_nesting_depth: 64,
        max_total_items: 200_000,
        max_array_len: 100_000,
        max_map_len: 100_000,
        max_bytes_len: 16 * 1024 * 1024,
        max_text_bytes: 64 * 1024,
    };
}

/// Exact canonical ASSURE-V1 L0 bytes.
///
/// The third-party CBOR parser's dynamic value type is intentionally not part
/// of this public witness.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CanonicalCbor {
    bytes: Box<[u8]>,
}

impl CanonicalCbor {
    #[must_use]
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }

    #[must_use]
    pub fn into_bytes(self) -> Box<[u8]> {
        self.bytes
    }
}

/// Stable ASSURE-V1 decode-stage failure code.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeFailureCode {
    MalformedEncoding,
    NonCanonicalEncoding,
    UnsupportedSchema,
    ResourceLimitExceeded,
    ForbiddenValueKind,
    InvalidMapKey,
    DuplicateMapKey,
}

impl DecodeFailureCode {
    #[must_use]
    pub const fn stable_code(self) -> &'static str {
        match self {
            Self::MalformedEncoding => "D001_MALFORMED_ENCODING",
            Self::NonCanonicalEncoding => "D002_NON_CANONICAL_ENCODING",
            Self::UnsupportedSchema => "D003_UNSUPPORTED_SCHEMA",
            Self::ResourceLimitExceeded => "D004_RESOURCE_LIMIT_EXCEEDED",
            Self::ForbiddenValueKind => "D005_FORBIDDEN_VALUE_KIND",
            Self::InvalidMapKey => "D006_INVALID_MAP_KEY",
            Self::DuplicateMapKey => "D007_DUPLICATE_MAP_KEY",
        }
    }
}

/// Resource class whose active L0 bound was exceeded.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResourceLimit {
    InputBytes,
    NestingDepth,
    TotalItems,
    ArrayLength,
    MapLength,
    ByteStringLength,
    TextLength,
}

/// CBOR value kind forbidden by the ASSURE-V1 L0 profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ForbiddenValueKind {
    Float,
    Tag,
    Null,
    Simple,
    Unknown,
}

/// Deterministic witness for an L0 decode failure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DecodeFailureWitness {
    None,
    ResourceLimit(ResourceLimit),
    ForbiddenValueKind(ForbiddenValueKind),
    InvalidMapKey,
    DuplicateMapKey(u64),
}

/// L0 decode failure.
///
/// Third-party parser error text is deliberately not exposed as normative
/// output because it is not part of the ASSURE-V1 specification.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodeFailure {
    code: DecodeFailureCode,
    witness: DecodeFailureWitness,
}

impl DecodeFailure {
    const fn new(code: DecodeFailureCode, witness: DecodeFailureWitness) -> Self {
        Self { code, witness }
    }

    #[must_use]
    pub const fn code(&self) -> DecodeFailureCode {
        self.code
    }

    #[must_use]
    pub const fn stable_code(&self) -> &'static str {
        self.code.stable_code()
    }

    #[must_use]
    pub const fn witness(&self) -> DecodeFailureWitness {
        self.witness
    }
}

/// Validate one exact ASSURE-V1 L0 canonical CBOR item.
///
/// Success establishes only canonical codec conformance under `limits`.
pub fn decode_canonical(
    input: &[u8],
    limits: &DecodeLimits,
) -> Result<CanonicalCbor, DecodeFailure> {
    if input.len() > limits.max_input_bytes as usize {
        return Err(resource_failure(ResourceLimit::InputBytes));
    }

    let mut deserializer = cbor2::de::Deserializer::from_slice_with_recursion_limit(
        input,
        usize::from(limits.max_nesting_depth),
    );
    let value = Value::deserialize(&mut deserializer).map_err(parser_failure)?;

    if deserializer.offset() != input.len() {
        return Err(malformed_failure());
    }

    validate_profile(&value, limits)?;

    let canonical = cbor2::to_canonical_vec(&value).map_err(|_| malformed_failure())?;

    if canonical.as_slice() != input {
        return Err(DecodeFailure::new(
            DecodeFailureCode::NonCanonicalEncoding,
            DecodeFailureWitness::None,
        ));
    }

    Ok(CanonicalCbor {
        bytes: input.to_vec().into_boxed_slice(),
    })
}

fn malformed_failure() -> DecodeFailure {
    DecodeFailure::new(
        DecodeFailureCode::MalformedEncoding,
        DecodeFailureWitness::None,
    )
}

fn parser_failure(error: cbor2::de::Error) -> DecodeFailure {
    match error {
        cbor2::de::Error::RecursionLimitExceeded => resource_failure(ResourceLimit::NestingDepth),
        _ => malformed_failure(),
    }
}

fn resource_failure(limit: ResourceLimit) -> DecodeFailure {
    DecodeFailure::new(
        DecodeFailureCode::ResourceLimitExceeded,
        DecodeFailureWitness::ResourceLimit(limit),
    )
}

fn validate_profile(value: &Value, limits: &DecodeLimits) -> Result<(), DecodeFailure> {
    let mut stack = vec![(value, 1_u16)];
    let mut total_items = 0_u32;
    let max_nesting_depth = u16::from(limits.max_nesting_depth);

    while let Some((current, depth)) = stack.pop() {
        if depth > max_nesting_depth {
            return Err(resource_failure(ResourceLimit::NestingDepth));
        }

        total_items = total_items
            .checked_add(1)
            .ok_or_else(|| resource_failure(ResourceLimit::TotalItems))?;
        if total_items > limits.max_total_items {
            return Err(resource_failure(ResourceLimit::TotalItems));
        }

        match current {
            Value::Integer(_) | Value::Bool(_) => {}
            Value::Bytes(bytes) => {
                if bytes.len() > limits.max_bytes_len as usize {
                    return Err(resource_failure(ResourceLimit::ByteStringLength));
                }
            }
            Value::Text(text) => {
                if text.len() > limits.max_text_bytes as usize {
                    return Err(resource_failure(ResourceLimit::TextLength));
                }
            }
            Value::Array(items) => {
                if items.len() > limits.max_array_len as usize {
                    return Err(resource_failure(ResourceLimit::ArrayLength));
                }
                let child_depth = depth
                    .checked_add(1)
                    .ok_or_else(|| resource_failure(ResourceLimit::NestingDepth))?;
                for item in items.iter().rev() {
                    stack.push((item, child_depth));
                }
            }
            Value::Map(entries) => {
                if entries.len() > limits.max_map_len as usize {
                    return Err(resource_failure(ResourceLimit::MapLength));
                }

                let child_depth = depth
                    .checked_add(1)
                    .ok_or_else(|| resource_failure(ResourceLimit::NestingDepth))?;
                let mut keys = BTreeSet::new();

                for (key, mapped) in entries.iter().rev() {
                    let key_id = match key {
                        Value::Integer(integer) => {
                            let signed = i128::from(*integer);
                            u64::try_from(signed).map_err(|_| {
                                DecodeFailure::new(
                                    DecodeFailureCode::InvalidMapKey,
                                    DecodeFailureWitness::InvalidMapKey,
                                )
                            })?
                        }
                        _ => {
                            return Err(DecodeFailure::new(
                                DecodeFailureCode::InvalidMapKey,
                                DecodeFailureWitness::InvalidMapKey,
                            ));
                        }
                    };

                    if !keys.insert(key_id) {
                        return Err(DecodeFailure::new(
                            DecodeFailureCode::DuplicateMapKey,
                            DecodeFailureWitness::DuplicateMapKey(key_id),
                        ));
                    }

                    stack.push((mapped, child_depth));
                    stack.push((key, child_depth));
                }
            }
            Value::Float(_) => {
                return Err(forbidden_failure(ForbiddenValueKind::Float));
            }
            Value::Tag(_, _) => {
                return Err(forbidden_failure(ForbiddenValueKind::Tag));
            }
            Value::Null => {
                return Err(forbidden_failure(ForbiddenValueKind::Null));
            }
            Value::Simple(_) => {
                return Err(forbidden_failure(ForbiddenValueKind::Simple));
            }
            _ => {
                return Err(forbidden_failure(ForbiddenValueKind::Unknown));
            }
        }
    }

    Ok(())
}

fn forbidden_failure(kind: ForbiddenValueKind) -> DecodeFailure {
    DecodeFailure::new(
        DecodeFailureCode::ForbiddenValueKind,
        DecodeFailureWitness::ForbiddenValueKind(kind),
    )
}
