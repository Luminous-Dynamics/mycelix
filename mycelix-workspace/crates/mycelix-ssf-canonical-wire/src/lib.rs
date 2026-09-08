// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Provider-neutral canonical wire primitives for SSF evidence binding.
//!
//! This crate defines the byte-level grammar future semantic encoders must use.
//! It deliberately performs no hashing, signing, verification, qualification,
//! policy evaluation, or authority creation.
//!
//! Rust memory layout, `Debug`, enum discriminants, and platform endianness are
//! never canonical wire format. Semantic profiles must assign explicit stable
//! tags and write fields in an explicitly documented order.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

/// Canonical wire grammar version.
pub const CANONICAL_WIRE_VERSION_V1: u16 = 1;

/// Fixed protocol magic. This is emitted verbatim before version/domain tags.
pub const CANONICAL_WIRE_MAGIC_V1: &[u8] = b"MYCELIX-SSF-CANONICAL\0";

/// Closed top-level domain tag for canonical replay-evidence qualification.
///
/// Later domains must use distinct tags rather than reusing this one with a
/// different semantic payload.
pub const REPLAY_EVIDENCE_DECISION_DOMAIN_TAG_V1: u16 = 1;

/// Error-independent sink used by canonical encoders.
///
/// A future crypto adapter may implement this sink directly over a canonical
/// transcript/hasher; tests or transport adapters may write into a byte buffer.
pub trait CanonicalWireSinkV1 {
    type Error;

    fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
}

/// Write the fixed SSF canonical-wire header for one closed semantic domain.
pub fn write_header_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    domain_tag: u16,
) -> Result<(), S::Error> {
    sink.write(CANONICAL_WIRE_MAGIC_V1)?;
    write_u16_be_v1(sink, CANONICAL_WIRE_VERSION_V1)?;
    write_u16_be_v1(sink, domain_tag)
}

pub fn write_u8_v1<S: CanonicalWireSinkV1>(sink: &mut S, value: u8) -> Result<(), S::Error> {
    sink.write(&[value])
}

pub fn write_u16_be_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: u16,
) -> Result<(), S::Error> {
    sink.write(&value.to_be_bytes())
}

pub fn write_u32_be_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: u32,
) -> Result<(), S::Error> {
    sink.write(&value.to_be_bytes())
}

pub fn write_u64_be_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    value: u64,
) -> Result<(), S::Error> {
    sink.write(&value.to_be_bytes())
}

/// Write a fixed-size byte sequence with no implicit length field.
///
/// The enclosing semantic profile is responsible for fixing `N`.
pub fn write_fixed_bytes_v1<S: CanonicalWireSinkV1, const N: usize>(
    sink: &mut S,
    bytes: &[u8; N],
) -> Result<(), S::Error> {
    sink.write(bytes)
}

/// Write a variable byte string using an unsigned 32-bit big-endian length.
///
/// Values longer than `u32::MAX` are deliberately unsupported so canonical
/// framing is independent of machine pointer width.
pub fn write_len_prefixed_bytes_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    bytes: &[u8],
) -> Result<(), CanonicalWireWriteErrorV1<S::Error>> {
    let len = u32::try_from(bytes.len()).map_err(|_| CanonicalWireWriteErrorV1::LengthOverflow)?;
    write_u32_be_v1(sink, len).map_err(CanonicalWireWriteErrorV1::Sink)?;
    sink.write(bytes).map_err(CanonicalWireWriteErrorV1::Sink)
}

/// Canonical option tag. Semantic payload follows only for `Some`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CanonicalOptionTagV1 {
    None = 0,
    Some = 1,
}

pub fn write_option_tag_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    tag: CanonicalOptionTagV1,
) -> Result<(), S::Error> {
    write_u8_v1(sink, tag as u8)
}

/// Canonical boolean tag. Booleans must never be encoded using Rust layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CanonicalBoolTagV1 {
    False = 0,
    True = 1,
}

pub fn write_bool_tag_v1<S: CanonicalWireSinkV1>(
    sink: &mut S,
    tag: CanonicalBoolTagV1,
) -> Result<(), S::Error> {
    write_u8_v1(sink, tag as u8)
}

/// Errors introduced by canonical framing itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanonicalWireWriteErrorV1<E> {
    LengthOverflow,
    Sink(E),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[derive(Default)]
    struct VecSink(Vec<u8>);

    impl CanonicalWireSinkV1 for VecSink {
        type Error = ();

        fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.extend_from_slice(bytes);
            Ok(())
        }
    }

    #[test]
    fn header_is_exact_and_versioned() {
        let mut sink = VecSink::default();
        write_header_v1(&mut sink, REPLAY_EVIDENCE_DECISION_DOMAIN_TAG_V1).unwrap();

        let mut expected = CANONICAL_WIRE_MAGIC_V1.to_vec();
        expected.extend_from_slice(&CANONICAL_WIRE_VERSION_V1.to_be_bytes());
        expected.extend_from_slice(&REPLAY_EVIDENCE_DECISION_DOMAIN_TAG_V1.to_be_bytes());
        assert_eq!(sink.0, expected);
    }

    #[test]
    fn integers_are_big_endian_and_width_fixed() {
        let mut sink = VecSink::default();
        write_u16_be_v1(&mut sink, 0x0102).unwrap();
        write_u32_be_v1(&mut sink, 0x0304_0506).unwrap();
        write_u64_be_v1(&mut sink, 0x0708_090a_0b0c_0d0e).unwrap();
        assert_eq!(
            sink.0,
            [
                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
                0x0d, 0x0e,
            ]
        );
    }

    #[test]
    fn option_and_boolean_tags_are_explicit() {
        let mut sink = VecSink::default();
        write_option_tag_v1(&mut sink, CanonicalOptionTagV1::None).unwrap();
        write_option_tag_v1(&mut sink, CanonicalOptionTagV1::Some).unwrap();
        write_bool_tag_v1(&mut sink, CanonicalBoolTagV1::False).unwrap();
        write_bool_tag_v1(&mut sink, CanonicalBoolTagV1::True).unwrap();
        assert_eq!(sink.0, [0, 1, 0, 1]);
    }

    #[test]
    fn length_prefix_prevents_concatenation_ambiguity() {
        let mut a = VecSink::default();
        write_len_prefixed_bytes_v1(&mut a, b"ab").unwrap();
        write_len_prefixed_bytes_v1(&mut a, b"c").unwrap();

        let mut b = VecSink::default();
        write_len_prefixed_bytes_v1(&mut b, b"a").unwrap();
        write_len_prefixed_bytes_v1(&mut b, b"bc").unwrap();

        assert_ne!(a.0, b.0);
    }

    #[test]
    fn fixed_bytes_are_written_verbatim() {
        let mut sink = VecSink::default();
        write_fixed_bytes_v1(&mut sink, &[7_u8; 32]).unwrap();
        assert_eq!(sink.0, [7_u8; 32]);
    }
}
