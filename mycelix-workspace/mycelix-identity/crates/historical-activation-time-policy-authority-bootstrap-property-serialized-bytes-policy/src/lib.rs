// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Exact Holochain `SerializedBytes` preservation for the bootstrap property wire.
//!
//! #428 freezes the canonical raw property bytes. This theorem proves the narrow bridge needed
//! before a DNA/runtime adapter exists: wrapping those exact bytes through Holochain's explicit
//! custom-byte path preserves them byte-for-byte in `SerializedBytes`, and reading `.bytes()`
//! recovers the same V2 envelope for #428 validation.
//!
//! `UnsafeBytes` is deliberately contained inside the one `TryFrom` implementation below. It is
//! not exposed as a general application encoding mechanism.
//!
//! This crate does not inspect a running DNA, does not derive a DNA hash, and does not establish
//! deployment acceptance, constitutional legitimacy, signer authority, policy currentness, or
//! trusted time.

#![forbid(unsafe_code)]

use holochain_serialized_bytes::{SerializedBytes, UnsafeBytes};
use mycelix_historical_activation_time_policy_authority_bootstrap_declaration_policy::CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2;
use mycelix_historical_activation_time_policy_authority_bootstrap_property_wire_policy::{
    decode_historical_activation_time_policy_authority_bootstrap_property_v2,
    encode_historical_activation_time_policy_authority_bootstrap_property_v2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
};

#[derive(Debug)]
pub struct PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2 {
    properties: SerializedBytes,
}

impl PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2 {
    pub fn as_serialized_bytes(&self) -> &SerializedBytes {
        &self.properties
    }

    pub fn raw_property_bytes(&self) -> &[u8] {
        self.properties.bytes()
    }

    pub fn into_serialized_bytes(self) -> SerializedBytes {
        self.properties
    }
}

impl TryFrom<&CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2>
    for PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2
{
    type Error = HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2;

    fn try_from(
        declaration: &CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    ) -> Result<Self, Self::Error> {
        let property_bytes =
            encode_historical_activation_time_policy_authority_bootstrap_property_v2(declaration)?;
        let properties = SerializedBytes::from(UnsafeBytes::from(property_bytes));
        Ok(Self { properties })
    }
}

pub fn prepare_historical_activation_time_policy_authority_bootstrap_properties_v2(
    declaration: &CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
) -> Result<
    PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
> {
    PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2::try_from(declaration)
}

pub fn decode_historical_activation_time_policy_authority_bootstrap_properties_v2(
    properties: &SerializedBytes,
) -> Result<
    CanonicalHistoricalActivationTimePolicyAuthorityBootstrapDeclarationV2,
    HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2,
> {
    decode_historical_activation_time_policy_authority_bootstrap_property_v2(properties.bytes())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

    fn frozen_property_bytes() -> Vec<u8> {
        let constitution_id = b"mycelix-identity-v2-policy-authority";
        let policy_authority_id = b"identity:policy-authority:bootstrap-v2";
        let policy_authority_key_id = b"identity:policy-authority:bootstrap-v2#ed25519-1";
        let public_key = [0x66u8; 32];

        let mut payload = Vec::new();
        payload.push(0x01);
        payload.extend_from_slice(&1u16.to_be_bytes());
        payload.push(0x02);
        payload.extend_from_slice(&(constitution_id.len() as u16).to_be_bytes());
        payload.extend_from_slice(constitution_id);
        payload.push(0x03);
        payload.extend_from_slice(&1u32.to_be_bytes());
        payload.push(0x04);
        payload.extend_from_slice(&(policy_authority_id.len() as u16).to_be_bytes());
        payload.extend_from_slice(policy_authority_id);
        payload.push(0x05);
        payload.extend_from_slice(&(policy_authority_key_id.len() as u16).to_be_bytes());
        payload.extend_from_slice(policy_authority_key_id);
        payload.push(0x06);
        payload.extend_from_slice(&0xed01u16.to_be_bytes());
        payload.push(0x07);
        payload.extend_from_slice(&(public_key.len() as u32).to_be_bytes());
        payload.extend_from_slice(&public_key);
        payload.push(0x08);
        payload.extend_from_slice(&1u64.to_be_bytes());
        payload.push(0x09);
        payload.push(0x0f);

        let mut out = Vec::new();
        out.extend_from_slice(b"MXTPABV2");
        out.extend_from_slice(&1u16.to_be_bytes());
        out.extend_from_slice(&1u16.to_be_bytes());
        out.extend_from_slice(&1u16.to_be_bytes());
        out.extend_from_slice(&0u16.to_be_bytes());
        out.extend_from_slice(&(payload.len() as u32).to_be_bytes());
        out.extend_from_slice(&payload);
        out
    }

    #[test]
    fn custom_byte_wrapper_preserves_exact_frozen_envelope() {
        let expected = frozen_property_bytes();
        assert_eq!(expected.len(), 210);
        assert_eq!(
            Sha256::digest(&expected).as_slice(),
            &[
                0xac, 0x5a, 0x2d, 0x63, 0xe6, 0xb9, 0xeb, 0x53, 0x0a, 0xe3, 0x2f, 0x7d,
                0xc2, 0x7d, 0x39, 0x79, 0xa1, 0xaf, 0x0f, 0x87, 0x52, 0x3f, 0xd5, 0x6b,
                0xc4, 0x15, 0xb6, 0xfb, 0x69, 0x12, 0x4b, 0xc7,
            ]
        );

        let declaration =
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&expected)
                .unwrap();
        let prepared =
            PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2::try_from(
                &declaration,
            )
            .unwrap();

        assert_eq!(prepared.raw_property_bytes(), expected.as_slice());
        assert_eq!(prepared.as_serialized_bytes().bytes(), expected.as_slice());
    }

    #[test]
    fn serialized_bytes_roundtrip_preserves_declaration_identity() {
        let expected = frozen_property_bytes();
        let declaration =
            decode_historical_activation_time_policy_authority_bootstrap_property_v2(&expected)
                .unwrap();
        let original_digest = *declaration.declaration_digest_sha256();

        let serialized =
            prepare_historical_activation_time_policy_authority_bootstrap_properties_v2(
                &declaration,
            )
            .unwrap()
            .into_serialized_bytes();
        let decoded =
            decode_historical_activation_time_policy_authority_bootstrap_properties_v2(
                &serialized,
            )
            .unwrap();

        assert_eq!(serialized.bytes(), expected.as_slice());
        assert_eq!(decoded.declaration_digest_sha256(), &original_digest);
    }

    #[test]
    fn messagepack_like_or_nested_bytes_do_not_bypass_wire_gate() {
        for bytes in [
            vec![0x81, 0xa1, b'v', 0x01],
            {
                let mut nested = vec![0xc4, 210];
                nested.extend_from_slice(&frozen_property_bytes());
                nested
            },
        ] {
            let serialized = SerializedBytes::from(UnsafeBytes::from(bytes));
            assert_eq!(
                decode_historical_activation_time_policy_authority_bootstrap_properties_v2(
                    &serialized,
                )
                .unwrap_err(),
                HistoricalActivationTimePolicyAuthorityBootstrapPropertyWireErrorV2::MagicMismatch
            );
        }
    }

    #[test]
    fn prepared_wrapper_is_opaque_and_has_no_authority_constructor() {
        let source = include_str!("lib.rs");
        let start = source
            .index("pub struct PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2")
            .unwrap();
        let end = source[start..]
            .index("impl PreparedHistoricalActivationTimePolicyAuthorityBootstrapPropertiesV2")
            .unwrap()
            + start;
        assert!(!source[start..end].contains("pub properties:"));

        let production = &source[..source.index("#[cfg(test)]").unwrap()];
        let trusted_constructor = ["from", "trusted", "configuration"].join("_");
        let authorization_boolean = ["is", "authorized"].join("_");
        assert!(!production.contains(&trusted_constructor));
        assert!(!production.contains(&authorization_boolean));
    }
}
