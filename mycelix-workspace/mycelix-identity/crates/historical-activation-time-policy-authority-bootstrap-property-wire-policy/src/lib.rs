// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Strict raw DNA-property wire envelope for the historical time-policy authority bootstrap declaration.
//!
//! This crate is deliberately below any Holochain runtime adapter. It defines the exact byte
//! representation that a later provisioning/runtime layer may commit as DNA properties and
//! inspect before semantic declaration use.
//!
//! The ordering is normative:
//!
//! raw bytes -> magic -> family -> envelope version -> object kind -> flags -> bounded payload
//! -> declaration format version -> exact semantic fields -> #411 canonical declaration.
//!
//! Unknown/future envelope or declaration versions fail before later semantic fields are parsed.
//! Success establishes only that one byte string is the canonical V2 property representation of
//! one #411 declaration. It establishes no DNA provenance, domain acceptance, root legitimacy,
//! signer authority, transition authenticity, policy currentness, or trusted time.

#![forbid(unsafe_code)]

use core::str;
use mycelix_crypto::AlgorithmId;
use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::{
    canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2,
    CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2,
    HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
};
use mycelix_historical_activation_time_policy_authority_bootstrap_subject_policy::HistoricalActivationTimePolicyAuthorityScopeV2;

pub const BOOTSTRAP_PROPERTY_MAGIC_V2: &[u8; 8] = b"MXTPABV2";
pub const BOOTSTRAP_PROPERTY_FAMILY_V2: u16 = 1;
pub const BOOTSTRAP_PROPERTY_ENVELOPE_VERSION_V2: u16 = 1;
pub const BOOTSTRAP_PROPERTY_OBJECT_KIND_V2: u16 = 1;
pub const BOOTSTRAP_PROPERTY_FLAGS_V2: u16 = 0;
pub const BOOTSTRAP_PROPERTY_HEADER_LEN_V2: usize = 20;
pub const BOOTSTRAP_PROPERTY_MAX_PAYLOAD_LEN_V2: usize = 4096;
pub const BOOTSTRAP_PROPERTY_KNOWN_SCOPE_MASK_V2: u8 = 0x0f;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2 {
    Truncated,
    MagicMismatch,
    FamilyMismatch,
    UnsupportedEnvelopeVersion,
    ObjectKindMismatch,
    NonZeroFlags,
    PayloadTooLarge,
    PayloadLengthMismatch,
    UnexpectedTag { expected: u8, actual: u8 },
    UnsupportedDeclarationFormatVersion,
    InvalidUtf8,
    UnknownAlgorithm,
    UnknownScopeBits,
    InvalidScope,
    TrailingPayloadBytes,
    LengthOverflow,
    DeclarationSemantic(HistoricalActivationTimePolicyAuthorityBootstrapDeclarationErrorV2),
}

struct Reader<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl<'a> Reader<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, position: 0 }
    }

    fn take(
        &mut self,
        len: usize,
    ) -> Result<&'a [u8], HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
        let end = self
            .position
            .checked_add(len)
            .ok_or(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated)?;
        if end > self.bytes.len() {
            return Err(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated);
        }
        let out = &self.bytes[self.position..end];
        self.position = end;
        Ok(out)
    }

    fn read_u8(
        &mut self,
    ) -> Result<u8, HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
        let bytes = self.take(1)?;
        bytes
            .first()
            .copied()
            .ok_or(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated)
    }

    fn read_u16(
        &mut self,
    ) -> Result<u16, HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
        let bytes: [u8; 2] = self
            .take(2)?
            .try_into()
            .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated)?;
        Ok(u16::from_be_bytes(bytes))
    }

    fn read_u32(
        &mut self,
    ) -> Result<u32, HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
        let bytes: [u8; 4] = self
            .take(4)?
            .try_into()
            .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated)?;
        Ok(u32::from_be_bytes(bytes))
    }

    fn read_u64(
        &mut self,
    ) -> Result<u64, HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
        let bytes: [u8; 8] = self
            .take(8)?
            .try_into()
            .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::Truncated)?;
        Ok(u64::from_be_bytes(bytes))
    }

    const fn remaining(&self) -> usize {
        self.bytes.len() - self.position
    }
}

fn expect_tag(
    reader: &mut Reader<'_>,
    expected: u8,
) -> Result<(), HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    let actual = reader.read_u8()?;
    if actual != expected {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnexpectedTag {
                expected,
                actual,
            },
        );
    }
    Ok(())
}

fn read_len_prefixed_u16<'a>(
    reader: &mut Reader<'a>,
    expected_tag: u8,
) -> Result<&'a [u8], HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    expect_tag(reader, expected_tag)?;
    let len = usize::from(reader.read_u16()?);
    reader.take(len)
}

fn read_len_prefixed_u32<'a>(
    reader: &mut Reader<'a>,
    expected_tag: u8,
) -> Result<&'a [u8], HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    expect_tag(reader, expected_tag)?;
    let len = usize::try_from(reader.read_u32()?)
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::LengthOverflow)?;
    reader.take(len)
}

fn append_len_prefixed_u16(
    out: &mut Vec<u8>,
    tag: u8,
    bytes: &[u8],
) -> Result<(), HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    let len = u16::try_from(bytes.len())
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::LengthOverflow)?;
    out.push(tag);
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

fn append_len_prefixed_u32(
    out: &mut Vec<u8>,
    tag: u8,
    bytes: &[u8],
) -> Result<(), HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    let len = u32::try_from(bytes.len())
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::LengthOverflow)?;
    out.push(tag);
    out.extend_from_slice(&len.to_be_bytes());
    out.extend_from_slice(bytes);
    Ok(())
}

fn scope_mask(scope: HistoricalActivationTimePolicyAuthorityScopeV2) -> u8 {
    let mut mask = 0u8;
    if scope.allows_adopt() {
        mask |= 1 << 0;
    }
    if scope.allows_supersede() {
        mask |= 1 << 1;
    }
    if scope.allows_revoke() {
        mask |= 1 << 2;
    }
    if scope.allows_signer_rotation() {
        mask |= 1 << 3;
    }
    mask
}

fn decode_scope(
    mask: u8,
) -> Result<
    HistoricalActivationTimePolicyAuthorityScopeV2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
> {
    if mask & !BOOTSTRAP_PROPERTY_KNOWN_SCOPE_MASK_V2 != 0 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnknownScopeBits,
        );
    }
    HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(
        mask & (1 << 0) != 0,
        mask & (1 << 1) != 0,
        mask & (1 << 2) != 0,
        mask & (1 << 3) != 0,
    )
    .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::InvalidScope)
}

pub fn encode_historical_activation_time_policy_authority_bootstrap_property_v2(
    declaration: &CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
) -> Result<Vec<u8>, HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2> {
    let mut payload = Vec::new();

    payload.push(0x01);
    payload.extend_from_slice(&BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2.to_be_bytes());
    append_len_prefixed_u16(&mut payload, 0x02, declaration.constitution_id().as_bytes())?;
    payload.push(0x03);
    payload.extend_from_slice(&declaration.constitution_version().to_be_bytes());
    append_len_prefixed_u16(&mut payload, 0x04, declaration.policy_authority_id().as_bytes())?;
    append_len_prefixed_u16(
        &mut payload,
        0x05,
        declaration.policy_authority_key_id().as_bytes(),
    )?;
    payload.push(0x06);
    payload.extend_from_slice(&declaration.algorithm().as_u16().to_be_bytes());
    append_len_prefixed_u32(&mut payload, 0x07, declaration.public_key_bytes())?;
    payload.push(0x08);
    payload.extend_from_slice(&declaration.key_generation().to_be_bytes());
    payload.push(0x09);
    payload.push(scope_mask(declaration.scope()));

    if payload.len() > BOOTSTRAP_PROPERTY_MAX_PAYLOAD_LEN_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::PayloadTooLarge,
        );
    }
    let payload_len = u32::try_from(payload.len())
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::LengthOverflow)?;

    let mut out = Vec::with_capacity(BOOTSTRAP_PROPERTY_HEADER_LEN_V2 + payload.len());
    out.extend_from_slice(BOOTSTRAP_PROPERTY_MAGIC_V2);
    out.extend_from_slice(&BOOTSTRAP_PROPERTY_FAMILY_V2.to_be_bytes());
    out.extend_from_slice(&BOOTSTRAP_PROPERTY_ENVELOPE_VERSION_V2.to_be_bytes());
    out.extend_from_slice(&BOOTSTRAP_PROPERTY_OBJECT_KIND_V2.to_be_bytes());
    out.extend_from_slice(&BOOTSTRAP_PROPERTY_FLAGS_V2.to_be_bytes());
    out.extend_from_slice(&payload_len.to_be_bytes());
    out.extend_from_slice(&payload);
    Ok(out)
}

pub fn decode_historical_activation_time_policy_authority_bootstrap_property_v2(
    bytes: &[u8],
) -> Result<
    CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
> {
    let mut header = Reader::new(bytes);
    if header.take(BOOTSTRAP_PROPERTY_MAGIC_V2.len())? != BOOTSTRAP_PROPERTY_MAGIC_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::MagicMismatch,
        );
    }
    if header.read_u16()? != BOOTSTRAP_PROPERTY_FAMILY_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::FamilyMismatch,
        );
    }

    let envelope_version = header.read_u16()?;
    if envelope_version != BOOTSTRAP_PROPERTY_ENVELOPE_VERSION_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnsupportedEnvelopeVersion,
        );
    }

    if header.read_u16()? != BOOTSTRAP_PROPERTY_OBJECT_KIND_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::ObjectKindMismatch,
        );
    }
    if header.read_u16()? != BOOTSTRAP_PROPERTY_FLAGS_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::NonZeroFlags,
        );
    }
    let payload_len = usize::try_from(header.read_u32()?)
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::LengthOverflow)?;
    if payload_len > BOOTSTRAP_PROPERTY_MAX_PAYLOAD_LEN_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::PayloadTooLarge,
        );
    }
    if header.remaining() != payload_len {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::PayloadLengthMismatch,
        );
    }

    let payload = header.take(payload_len)?;
    let mut reader = Reader::new(payload);

    expect_tag(&mut reader, 0x01)?;
    let declaration_format_version = reader.read_u16()?;
    if declaration_format_version != BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnsupportedDeclarationFormatVersion,
        );
    }

    let constitution_id = str::from_utf8(read_len_prefixed_u16(&mut reader, 0x02)?)
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::InvalidUtf8)?;
    expect_tag(&mut reader, 0x03)?;
    let constitution_version = reader.read_u32()?;
    let policy_authority_id = str::from_utf8(read_len_prefixed_u16(&mut reader, 0x04)?)
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::InvalidUtf8)?;
    let policy_authority_key_id = str::from_utf8(read_len_prefixed_u16(&mut reader, 0x05)?)
        .map_err(|_| HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::InvalidUtf8)?;
    expect_tag(&mut reader, 0x06)?;
    let algorithm = AlgorithmId::from_u16(reader.read_u16()?)
        .ok_or(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnknownAlgorithm)?;
    let public_key_bytes = read_len_prefixed_u32(&mut reader, 0x07)?;
    expect_tag(&mut reader, 0x08)?;
    let key_generation = reader.read_u64()?;
    expect_tag(&mut reader, 0x09)?;
    let scope = decode_scope(reader.read_u8()?)?;

    if reader.remaining() != 0 {
        return Err(
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::TrailingPayloadBytes,
        );
    }

    canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
            format_version: declaration_format_version,
            constitution_id,
            constitution_version,
            policy_authority_id,
            policy_authority_key_id,
            algorithm,
            public_key_bytes,
            key_generation,
            scope,
        },
    )
    .map_err(HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::DeclarationSemantic)
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::{
        canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2,
        HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    };
    use sha2::{Digest, Sha256};

    static PUBLIC_KEY: [u8; 32] = [0x66; 32];

    fn full_scope() -> HistoricalActivationTimePolicyAuthorityScopeV2 {
        HistoricalActivationTimePolicyAuthorityScopeV2::from_permissions(true, true, true, true)
            .unwrap()
    }

    fn canonical() -> CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
        canonicalize_historical_activation_time_policy_authority_bootstrap_declaration_v2(
            HistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2 {
                format_version: BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2,
                constitution_id: "mycelix-identity-v2-policy-authority",
                constitution_version: 1,
                policy_authority_id: "identity:policy-authority:bootstrap-v2",
                policy_authority_key_id: "identity:policy-authority:bootstrap-v2#ed25519-1",
                algorithm: AlgorithmId::Ed25519,
                public_key_bytes: &PUBLIC_KEY,
                key_generation: 1,
                scope: full_scope(),
            },
        )
        .unwrap()
    }

    #[test]
    fn frozen_full_scope_property_envelope_is_stable() {
        let encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        assert_eq!(encoded.len(), 210);
        assert_eq!(&encoded[..8], BOOTSTRAP_PROPERTY_MAGIC_V2);
        assert_eq!(
            Sha256::digest(&encoded).as_slice(),
            &[
                0xac, 0x5a, 0x2d, 0x63, 0xe6, 0xb9, 0xeb, 0x53, 0x0a, 0xe3, 0x2f, 0x7d,
                0xc2, 0x7d, 0x39, 0x79, 0xa1, 0xaf, 0x0f, 0x87, 0x52, 0x3f, 0xd5, 0x6b,
                0xc4, 0x15, 0xb6, 0xfb, 0x69, 0x12, 0x4b, 0xc7,
            ]
        );
    }

    #[test]
    fn canonical_roundtrip_preserves_declaration_identity() {
        let input = canonical();
        let encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&input)
                .unwrap();
        let decoded =
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap();
        assert_eq!(
            decoded.declaration_digest_sha256(),
            input.declaration_digest_sha256()
        );
        assert_eq!(decoded.constitution_id(), input.constitution_id());
        assert_eq!(decoded.policy_authority_key_id(), input.policy_authority_key_id());
        assert_eq!(decoded.public_key_bytes(), input.public_key_bytes());
    }

    #[test]
    fn future_envelope_version_fails_before_payload_interpretation() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        encoded[10] = 0;
        encoded[11] = BOOTSTRAP_PROPERTY_ENVELOPE_VERSION_V2 as u8 + 1;
        encoded[16..20].copy_from_slice(&u32::MAX.to_be_bytes());
        encoded.truncate(BOOTSTRAP_PROPERTY_HEADER_LEN_V2);
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnsupportedEnvelopeVersion
        );
    }

    #[test]
    fn future_declaration_version_fails_before_semantic_fields() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        encoded[21] = 0;
        encoded[22] = BOOTSTRAP_DECLARATION_FORMAT_VERSION_V2 as u8 + 1;
        encoded[23] = 0xff;
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnsupportedDeclarationFormatVersion
        );
    }

    #[test]
    fn family_substitution_fails_closed() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        encoded[9] ^= 1;
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::FamilyMismatch
        );
    }

    #[test]
    fn payload_length_mismatch_and_trailing_bytes_fail_closed() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        encoded.push(0);
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::PayloadLengthMismatch
        );

        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        let payload_len = u32::from_be_bytes([encoded[16], encoded[17], encoded[18], encoded[19]]);
        let new_len = payload_len + 1;
        encoded[16..20].copy_from_slice(&new_len.to_be_bytes());
        encoded.push(0);
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::TrailingPayloadBytes
        );
    }

    #[test]
    fn unknown_scope_bits_fail_closed() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        let last = encoded.len() - 1;
        encoded[last] = 0x80;
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnknownScopeBits
        );
    }

    #[test]
    fn unexpected_field_order_fails_closed() {
        let mut encoded =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(&canonical())
                .unwrap();
        encoded[23] = 0x55;
        assert_eq!(
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&encoded)
                .unwrap_err(),
            HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::UnexpectedTag {
                expected: 0x02,
                actual: 0x55,
            }
        );
    }

    #[test]
    fn production_surface_contains_no_runtime_authority_shortcut() {
        let source = include_str!("lib.rs");
        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        for forbidden in [
            "dna_info(",
            "#[hdk_extern]",
            "RuntimeQualified",
            "ExactHistoricalActivationTimePolicyAuthorityBootstrapSubjectV2",
            "ExactHistoricalActivationTimePolicyAuthorityKeyGenerationV2",
            "from_trusted_configuration",
            "is_authorized",
            "verify_strict",
            "positive_evidence",
            "trust_score",
        ] {
            assert!(!production.contains(forbidden));
        }
    }
}
