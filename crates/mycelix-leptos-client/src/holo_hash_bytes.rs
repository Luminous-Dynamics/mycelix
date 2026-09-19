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
use base64::engine::general_purpose::STANDARD;
use serde::de::{SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;

pub const HOLO_HASH_WIRE_LEN: usize = 39;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct HoloHashBytes(Vec<u8>);

impl HoloHashBytes {
    /// Construct from the exact 39-byte HoloHash wire representation.
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

    pub fn as_raw_39(&self) -> &[u8] {
        &self.0
    }

    pub fn into_raw_39(self) -> Vec<u8> {
        self.0
    }

    pub fn to_raw_base64(&self) -> String {
        STANDARD.encode(&self.0)
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

    #[test]
    fn strict_length_is_enforced() {
        assert!(HoloHashBytes::from_raw_39(vec![0; 38]).is_err());
        assert!(HoloHashBytes::from_raw_39(vec![0; 40]).is_err());
        assert!(HoloHashBytes::from_raw_39(vec![0; 39]).is_ok());
    }

    #[test]
    fn raw_base64_roundtrips_exact_wire_bytes() {
        let hash = HoloHashBytes::from_raw_39((0u8..39).collect()).unwrap();
        let encoded = hash.to_raw_base64();
        let decoded = HoloHashBytes::from_raw_base64(&encoded).unwrap();
        assert_eq!(decoded, hash);
    }

    #[test]
    fn messagepack_serialization_is_binary_not_string() {
        let hash = HoloHashBytes::from_raw_39(vec![0xab; 39]).unwrap();
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
