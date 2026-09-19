// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Canonical Holochain hash wire representation for browser zome calls.
//!
//! Holochain `HoloHash` values (`ActionHash`, `AgentPubKey`, etc.) serialize
//! across ExternIO as raw 39-byte binary values, not UTF-8/base64 strings.
//! This type gives WASM clients that intentionally avoid `holo_hash` the same
//! serde shape without pulling Holochain host-side dependencies into the
//! browser build.

use base64::Engine;
use base64::engine::general_purpose::{STANDARD, URL_SAFE_NO_PAD};
use serde::de::{SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;

pub const HOLO_HASH_WIRE_LEN: usize = 39;
pub const HOLO_HASH_PREFIX_LEN: usize = 3;

/// Primitive HoloHash kinds currently required by browser application inputs.
///
/// The byte prefixes are the raw 3-byte values represented by Holochain 0.6's
/// published display prefixes:
///
/// - Agent / AgentPubKey: `uhCAk` -> `[0x84, 0x20, 0x24]`
/// - Action / ActionHash: `uhCkk` -> `[0x84, 0x29, 0x24]`
///
/// We intentionally model only kinds that browser call sites currently need.
/// Other valid 39-byte HoloHashes can still use [`HoloHashBytes`] generically.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HoloHashKind {
    Agent,
    Action,
}

impl HoloHashKind {
    pub const fn prefix(self) -> [u8; HOLO_HASH_PREFIX_LEN] {
        match self {
            Self::Agent => [0x84, 0x20, 0x24],
            Self::Action => [0x84, 0x29, 0x24],
        }
    }

    pub const fn label(self) -> &'static str {
        match self {
            Self::Agent => "AgentPubKey",
            Self::Action => "ActionHash",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct HoloHashBytes(Vec<u8>);

impl HoloHashBytes {
    /// Construct from the exact 39-byte HoloHash wire representation.
    ///
    /// This validates length only. Use [`Self::require_kind`] or one of the
    /// typed carrier constructors before sending a value to a zome parameter
    /// whose concrete HoloHash type is known.
    pub fn from_raw_39(bytes: Vec<u8>) -> Result<Self, String> {
        if bytes.len() != HOLO_HASH_WIRE_LEN {
            return Err(format!(
                "HoloHash wire value must be exactly {HOLO_HASH_WIRE_LEN} bytes, got {}",
                bytes.len()
            ));
        }
        Ok(Self(bytes))
    }

    /// Parse the repository's internal raw-hash base64 representation.
    ///
    /// This is deliberately named `raw_base64`: it is *not* HoloHash's
    /// user-facing multibase/display string. It is only a reversible textual
    /// carrier for the exact 39 wire bytes already received from a conductor.
    pub fn from_raw_base64(value: &str) -> Result<Self, String> {
        let bytes = STANDARD
            .decode(value)
            .map_err(|error| format!("invalid raw HoloHash base64: {error}"))?;
        Self::from_raw_39(bytes)
    }

    /// Parse a raw-base64 carrier and require that its 3-byte prefix is the
    /// Holochain ActionHash prefix.
    pub fn from_action_raw_base64(value: &str) -> Result<Self, String> {
        Self::from_raw_base64(value)?.require_kind(HoloHashKind::Action)
    }

    /// Parse Holochain's human/display representation (`u` + base64url/no-pad).
    ///
    /// This is intentionally separate from [`Self::from_raw_base64`] so callers
    /// cannot silently mix the UI display form with the repository's internal
    /// reversible carrier used for ActionHash round-tripping.
    pub fn from_holochain_display(value: &str) -> Result<Self, String> {
        let encoded = value
            .strip_prefix('u')
            .ok_or_else(|| "HoloHash display value must start with 'u'".to_string())?;
        let bytes = URL_SAFE_NO_PAD
            .decode(encoded)
            .map_err(|error| format!("invalid HoloHash display base64url: {error}"))?;
        Self::from_raw_39(bytes)
    }

    /// Parse a Holochain display value and require an AgentPubKey prefix.
    pub fn from_agent_display(value: &str) -> Result<Self, String> {
        Self::from_holochain_display(value)?.require_kind(HoloHashKind::Agent)
    }

    /// Return the browser-relevant primitive kind when its prefix is recognized.
    pub fn kind(&self) -> Option<HoloHashKind> {
        let prefix = self.0.get(..HOLO_HASH_PREFIX_LEN)?;
        if prefix == HoloHashKind::Agent.prefix() {
            Some(HoloHashKind::Agent)
        } else if prefix == HoloHashKind::Action.prefix() {
            Some(HoloHashKind::Action)
        } else {
            None
        }
    }

    /// Require this value to carry the expected HoloHash type prefix.
    pub fn require_kind(self, expected: HoloHashKind) -> Result<Self, String> {
        let expected_prefix = expected.prefix();
        if self.0[..HOLO_HASH_PREFIX_LEN] == expected_prefix {
            return Ok(self);
        }

        let actual = self
            .kind()
            .map(HoloHashKind::label)
            .unwrap_or("another HoloHash kind");
        Err(format!(
            "expected {}, but value carries {} prefix {:02x?}",
            expected.label(),
            actual,
            &self.0[..HOLO_HASH_PREFIX_LEN]
        ))
    }

    pub fn as_raw_39(&self) -> &[u8] {
        &self.0
    }

    pub fn into_raw_39(self) -> Vec<u8> {
        self.0
    }

    pub fn to_raw_base64(&self) -> String {
        STANDARD.encode(&self.0)
    }

    /// Render Holochain's canonical user/display form (`u` + base64url/no-pad).
    pub fn to_holochain_display(&self) -> String {
        format!("u{}", URL_SAFE_NO_PAD.encode(&self.0))
    }
}

impl Serialize for HoloHashBytes {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_bytes(&self.0)
    }
}

impl<'de> Deserialize<'de> for HoloHashBytes {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct HoloHashVisitor;

        impl<'de> Visitor<'de> for HoloHashVisitor {
            type Value = HoloHashBytes;

            fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str("a 39-byte HoloHash binary value")
            }

            fn visit_bytes<E>(self, value: &[u8]) -> Result<Self::Value, E>
            where
                E: serde::de::Error,
            {
                HoloHashBytes::from_raw_39(value.to_vec()).map_err(E::custom)
            }

            fn visit_byte_buf<E>(self, value: Vec<u8>) -> Result<Self::Value, E>
            where
                E: serde::de::Error,
            {
                HoloHashBytes::from_raw_39(value).map_err(E::custom)
            }

            // Accept array-encoded bytes on reads for compatibility with older
            // rmp-serde shapes, but serialization always emits MessagePack Bin.
            fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let mut bytes = Vec::with_capacity(sequence.size_hint().unwrap_or(0));
                while let Some(byte) = sequence.next_element::<u8>()? {
                    bytes.push(byte);
                }
                HoloHashBytes::from_raw_39(bytes).map_err(serde::de::Error::custom)
            }
        }

        deserializer.deserialize_any(HoloHashVisitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{decode, encode};

    fn hash_of_kind(kind: HoloHashKind) -> HoloHashBytes {
        let mut bytes = vec![0u8; HOLO_HASH_WIRE_LEN];
        bytes[..HOLO_HASH_PREFIX_LEN].copy_from_slice(&kind.prefix());
        HoloHashBytes::from_raw_39(bytes).unwrap()
    }

    #[test]
    fn strict_length_is_enforced() {
        assert!(HoloHashBytes::from_raw_39(vec![0; 38]).is_err());
        assert!(HoloHashBytes::from_raw_39(vec![0; 40]).is_err());
        assert!(HoloHashBytes::from_raw_39(vec![0; 39]).is_ok());
    }

    #[test]
    fn published_agent_and_action_prefixes_match_display_forms() {
        let agent = hash_of_kind(HoloHashKind::Agent);
        let action = hash_of_kind(HoloHashKind::Action);
        assert!(agent.to_holochain_display().starts_with("uhCAk"));
        assert!(action.to_holochain_display().starts_with("uhCkk"));
        assert_eq!(agent.kind(), Some(HoloHashKind::Agent));
        assert_eq!(action.kind(), Some(HoloHashKind::Action));
    }

    #[test]
    fn typed_constructors_reject_cross_kind_values() {
        let agent = hash_of_kind(HoloHashKind::Agent);
        let action = hash_of_kind(HoloHashKind::Action);

        assert!(HoloHashBytes::from_agent_display(&agent.to_holochain_display()).is_ok());
        assert!(HoloHashBytes::from_agent_display(&action.to_holochain_display()).is_err());
        assert!(HoloHashBytes::from_action_raw_base64(&action.to_raw_base64()).is_ok());
        assert!(HoloHashBytes::from_action_raw_base64(&agent.to_raw_base64()).is_err());
    }

    #[test]
    fn raw_base64_roundtrips_exact_wire_bytes() {
        let hash = hash_of_kind(HoloHashKind::Action);
        let encoded = hash.to_raw_base64();
        let decoded = HoloHashBytes::from_raw_base64(&encoded).unwrap();
        assert_eq!(decoded, hash);
    }

    #[test]
    fn holochain_display_roundtrips_exact_wire_bytes() {
        let hash = hash_of_kind(HoloHashKind::Agent);
        let display = hash.to_holochain_display();
        assert!(display.starts_with('u'));
        assert!(!display.contains('='));
        let decoded = HoloHashBytes::from_holochain_display(&display).unwrap();
        assert_eq!(decoded, hash);
        assert!(HoloHashBytes::from_holochain_display(&hash.to_raw_base64()).is_err());
    }

    #[test]
    fn messagepack_serialization_is_binary_not_string() {
        let hash = hash_of_kind(HoloHashKind::Action);
        let raw_base64 = hash.to_raw_base64();

        let hash_wire = encode(&hash).unwrap();
        let string_wire = encode(&raw_base64).unwrap();

        assert_ne!(hash_wire, string_wire);
        // 39 bytes are encoded by MessagePack as bin8: 0xc4, length 39.
        assert_eq!(hash_wire.first().copied(), Some(0xc4));
        assert_eq!(hash_wire.get(1).copied(), Some(39));
        assert_eq!(decode::<HoloHashBytes>(&hash_wire).unwrap(), hash);
    }
}
