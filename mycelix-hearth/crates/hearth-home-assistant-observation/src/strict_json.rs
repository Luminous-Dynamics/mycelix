// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Bounded JSON decoding that rejects duplicate object keys.
//!
//! Ordinary serde_json::Value deserialization accepts JSON objects with
//! repeated keys and retains one value, erasing evidence that the input was
//! ambiguous. Protocol and projection gates need to see the exact parsed
//! structure, so this decoder rejects duplicate keys at every nesting level
//! and enforces a frame byte ceiling before allocating the JSON value tree.
//!
//! This is a parser helper, not a WebSocket transport. It does not establish
//! peer identity, TLS validity, freshness, authority, or physical outcome.

use serde::de::{DeserializeSeed, Error as DeError, MapAccess, SeqAccess, Visitor};
use serde_json::{Map, Number, Value};
use std::fmt;

pub const MAX_HA_WS_JSON_FRAME_BYTES: usize = 8 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrictJsonError {
    FrameTooLarge,
    MalformedOrDuplicateKey,
}

/// Owned JSON value whose source bytes passed the bounded duplicate-key check.
#[derive(Debug, Clone, PartialEq)]
pub struct StrictJsonValue {
    value: Value,
}

impl StrictJsonValue {
    pub fn as_value(&self) -> &Value {
        &self.value
    }

    /// Obtain a nested read-only view that retains the strict-decoding marker.
    pub fn get(&self, key: &str) -> Option<StrictJsonValueRef<'_>> {
        self.value.get(key).map(|value| StrictJsonValueRef { value })
    }
}

/// Read-only view into a StrictJsonValue. It has no public constructor, so an
/// arbitrary serde_json::Value cannot be promoted to a trusted parsed view.
#[derive(Debug, Clone, Copy)]
pub struct StrictJsonValueRef<'a> {
    value: &'a Value,
}

impl<'a> StrictJsonValueRef<'a> {
    pub fn as_value(self) -> &'a Value {
        self.value
    }

    pub fn get(self, key: &str) -> Option<StrictJsonValueRef<'a>> {
        self.value.get(key).map(|value| StrictJsonValueRef { value })
    }
}

impl fmt::Display for StrictJsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::FrameTooLarge => write!(f, "Home Assistant JSON frame exceeds the byte ceiling"),
            Self::MalformedOrDuplicateKey => {
                write!(f, "Home Assistant JSON frame is malformed or contains duplicate object keys")
            }
        }
    }
}

impl std::error::Error for StrictJsonError {}

struct ValueSeed;

impl<'de> DeserializeSeed<'de> for ValueSeed {
    type Value = Value;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        deserializer.deserialize_any(ValueVisitor)
    }
}

struct ValueVisitor;

impl<'de> Visitor<'de> for ValueVisitor {
    type Value = Value;

    fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str("bounded JSON without duplicate object keys")
    }

    fn visit_bool<E>(self, value: bool) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::Bool(value))
    }

    fn visit_i64<E>(self, value: i64) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::Number(Number::from(value)))
    }

    fn visit_u64<E>(self, value: u64) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::Number(Number::from(value)))
    }

    fn visit_f64<E>(self, value: f64) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Number::from_f64(value)
            .map(Value::Number)
            .ok_or_else(|| E::custom("non-finite JSON numbers are not admitted"))
    }

    fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::String(value.to_owned()))
    }

    fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::String(value))
    }

    fn visit_unit<E>(self) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::Null)
    }

    fn visit_none<E>(self) -> Result<Self::Value, E>
    where
        E: DeError,
    {
        Ok(Value::Null)
    }

    fn visit_some<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        ValueSeed.deserialize(deserializer)
    }

    fn visit_seq<A>(self, mut sequence: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut values = Vec::new();
        while let Some(value) = sequence.next_element_seed(ValueSeed)? {
            values.push(value);
        }
        Ok(Value::Array(values))
    }

    fn visit_map<A>(self, mut object: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>,
    {
        let mut values = Map::new();
        while let Some(key) = object.next_key::<String>()? {
            if values.contains_key(&key) {
                // Do not include the key or surrounding payload in diagnostics:
                // keys may contain household-specific entity identifiers.
                return Err(A::Error::custom("duplicate JSON object key"));
            }
            let value = object.next_value_seed(ValueSeed)?;
            values.insert(key, value);
        }
        Ok(Value::Object(values))
    }
}

/// Parse one bounded JSON frame, rejecting duplicate object keys at every
/// nesting depth and any trailing non-whitespace data. Error values deliberately
/// avoid reflecting raw bytes, tokens, entity IDs, or arbitrary provider text.
pub fn parse_bounded_json(input: &[u8]) -> Result<StrictJsonValue, StrictJsonError> {
    if input.len() > MAX_HA_WS_JSON_FRAME_BYTES {
        return Err(StrictJsonError::FrameTooLarge);
    }

    let mut deserializer = serde_json::Deserializer::from_slice(input);
    let value = ValueSeed
        .deserialize(&mut deserializer)
        .map_err(|_| StrictJsonError::MalformedOrDuplicateKey)?;
    deserializer
        .end()
        .map_err(|_| StrictJsonError::MalformedOrDuplicateKey)?;
    Ok(StrictJsonValue { value })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_valid_nested_json_without_changing_values() {
        let input = br#"{
            "type": "event",
            "id": 7,
            "event": {
                "a": {
                    "sensor.example": {
                        "s": "21.5",
                        "a": {"unit_of_measurement": "\\u00b0C"}
                    }
                },
                "c": {},
                "r": []
            }
        }"#;
        let parsed = parse_bounded_json(input).unwrap();
        assert_eq!(parsed.as_value().pointer("/id").and_then(Value::as_u64), Some(7));
        assert_eq!(
            parsed.as_value().pointer("/event/a/sensor.example/s").and_then(Value::as_str),
            Some("21.5")
        );
    }

    #[test]
    fn rejects_duplicate_top_level_protocol_keys() {
        assert_eq!(
            parse_bounded_json(br#"{"type":"result","id":1,"id":2}"#),
            Err(StrictJsonError::MalformedOrDuplicateKey)
        );
    }

    #[test]
    fn rejects_duplicate_nested_state_and_metadata_keys() {
        assert_eq!(
            parse_bounded_json(
                br#"{"event":{"a":{"sensor.example":{"s":"off","s":"on","a":{}}}}}"#
            ),
            Err(StrictJsonError::MalformedOrDuplicateKey)
        );
        assert_eq!(
            parse_bounded_json(
                br#"{"event":{"a":{"sensor.example":{"s":"on","a":{"device_class":"door","device_class":"motion"}}}}}"#
            ),
            Err(StrictJsonError::MalformedOrDuplicateKey)
        );
    }

    #[test]
    fn rejects_duplicate_keys_inside_arrays_and_nested_objects() {
        assert_eq!(
            parse_bounded_json(br#"{"items":[{"kind":"state","kind":"service"}]}"#),
            Err(StrictJsonError::MalformedOrDuplicateKey)
        );
    }

    #[test]
    fn rejects_oversized_frame_before_json_parsing() {
        let input = vec![b' '; MAX_HA_WS_JSON_FRAME_BYTES + 1];
        assert_eq!(parse_bounded_json(&input), Err(StrictJsonError::FrameTooLarge));
    }

    #[test]
    fn rejects_trailing_payload_after_a_valid_json_value() {
        assert_eq!(
            parse_bounded_json(br#"{"id":1} {"id":2}"#),
            Err(StrictJsonError::MalformedOrDuplicateKey)
        );
    }

    #[test]
    fn parse_errors_do_not_echo_sensitive_payload_text() {
        let payload = br#"{"access_token":"fixture-secret","id":1,"id":2}"#;
        let error = parse_bounded_json(payload).unwrap_err();
        let rendered = format!("{error} {error:?}");
        assert!(!rendered.contains("fixture-secret"));
        assert!(!rendered.contains("access_token"));
    }
}
