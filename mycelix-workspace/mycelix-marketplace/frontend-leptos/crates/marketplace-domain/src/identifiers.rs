use base64::Engine as _;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use serde::{Deserialize, Serialize};
use std::{fmt, str::FromStr};

const HOLO_HASH_BYTES: usize = 39;

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum HoloHashParseError {
    #[error("Holochain hash must use the u-prefixed base64url representation")]
    MissingPrefix,
    #[error("invalid Holochain base64url payload: {0}")]
    InvalidBase64(String),
    #[error("Holochain hash decoded to {actual} bytes; expected {expected}")]
    InvalidLength { expected: usize, actual: usize },
}

fn decode_holochain_b64(value: &str) -> Result<Vec<u8>, HoloHashParseError> {
    let encoded = value
        .strip_prefix('u')
        .ok_or(HoloHashParseError::MissingPrefix)?;
    let bytes = URL_SAFE_NO_PAD
        .decode(encoded)
        .map_err(|error| HoloHashParseError::InvalidBase64(error.to_string()))?;
    if bytes.len() != HOLO_HASH_BYTES {
        return Err(HoloHashParseError::InvalidLength {
            expected: HOLO_HASH_BYTES,
            actual: bytes.len(),
        });
    }
    Ok(bytes)
}

macro_rules! opaque_hash_type {
    ($name:ident) => {
        #[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(#[serde(with = "serde_bytes")] pub Vec<u8>);

        impl $name {
            pub fn new(bytes: Vec<u8>) -> Self {
                Self(bytes)
            }

            pub fn as_bytes(&self) -> &[u8] {
                &self.0
            }

            pub fn to_holochain_b64(&self) -> String {
                format!("u{}", URL_SAFE_NO_PAD.encode(&self.0))
            }
        }

        impl FromStr for $name {
            type Err = HoloHashParseError;

            fn from_str(value: &str) -> Result<Self, Self::Err> {
                decode_holochain_b64(value).map(Self)
            }
        }

        impl TryFrom<&str> for $name {
            type Error = HoloHashParseError;

            fn try_from(value: &str) -> Result<Self, Self::Error> {
                value.parse()
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                formatter.write_str(&self.to_holochain_b64())
            }
        }
    };
}

opaque_hash_type!(ActionHash);
opaque_hash_type!(AgentPubKey);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct TimestampMicros(pub i64);

impl TimestampMicros {
    pub fn as_micros(self) -> i64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn action_hash_round_trips_through_route_representation() {
        let hash = ActionHash::new((0..HOLO_HASH_BYTES as u8).collect());
        let encoded = hash.to_string();
        assert_eq!(encoded.parse::<ActionHash>().unwrap(), hash);
    }

    #[test]
    fn parser_rejects_missing_prefix_and_wrong_length() {
        assert_eq!(
            "not-a-hash".parse::<ActionHash>().unwrap_err(),
            HoloHashParseError::MissingPrefix
        );
        assert!(matches!(
            "uAQ".parse::<AgentPubKey>().unwrap_err(),
            HoloHashParseError::InvalidLength { .. }
        ));
    }
}
