// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical subject identities for Mycelix regenerative systems.
//!
//! REGEN-002 deliberately defines identity only. These types do not establish
//! existence, ownership, availability, quality, safety, suitability, currentness,
//! evidence sufficiency, governance legitimacy, or execution authority.
//!
//! The canonical textual form is:
//!
//! `regen:v1:<kind>:<local-token>`
//!
//! Local tokens are bounded lowercase ASCII identifiers. The typed wrapper and
//! exact kind prefix prevent a valid identifier of one regenerative subject from
//! being silently reinterpreted as another subject kind.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

use core::{fmt, str::FromStr};

#[cfg(feature = "serde")]
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};

/// Maximum UTF-8 byte length of the local, kind-specific identifier token.
pub const MAX_LOCAL_TOKEN_BYTES: usize = 128;

/// Regenerative identity validation failures.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegenerativeIdError {
    /// The canonical string is missing the exact subject-kind prefix.
    WrongPrefix {
        /// Exact canonical prefix required for the requested subject kind.
        expected: &'static str,
    },
    /// The local token is empty.
    EmptyToken,
    /// The local token exceeds the bounded identity size.
    TokenTooLong {
        /// Actual UTF-8 byte length of the rejected local token.
        actual: usize,
        /// Maximum permitted UTF-8 byte length.
        max: usize,
    },
    /// The local token must begin with an ASCII lowercase letter or digit.
    InvalidTokenStart,
    /// The local token must end with an ASCII lowercase letter or digit.
    InvalidTokenEnd,
    /// The local token contains a character outside the canonical grammar.
    InvalidTokenCharacter {
        /// Zero-based byte index of the rejected character.
        index: usize,
        /// Rejected noncanonical byte value.
        byte: u8,
    },
}

impl fmt::Display for RegenerativeIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongPrefix { expected } => {
                write!(f, "regenerative id must begin with exact prefix {expected}")
            }
            Self::EmptyToken => f.write_str("regenerative local token must not be empty"),
            Self::TokenTooLong { actual, max } => write!(
                f,
                "regenerative local token is {actual} bytes; maximum is {max}"
            ),
            Self::InvalidTokenStart => f.write_str(
                "regenerative local token must start with an ASCII lowercase letter or digit",
            ),
            Self::InvalidTokenEnd => f.write_str(
                "regenerative local token must end with an ASCII lowercase letter or digit",
            ),
            Self::InvalidTokenCharacter { index, byte } => write!(
                f,
                "invalid regenerative local-token byte 0x{byte:02x} at byte index {index}"
            ),
        }
    }
}

impl std::error::Error for RegenerativeIdError {}

fn is_lower_alnum(byte: u8) -> bool {
    byte.is_ascii_lowercase() || byte.is_ascii_digit()
}

fn validate_local_token(token: &str) -> Result<(), RegenerativeIdError> {
    if token.is_empty() {
        return Err(RegenerativeIdError::EmptyToken);
    }
    if token.len() > MAX_LOCAL_TOKEN_BYTES {
        return Err(RegenerativeIdError::TokenTooLong {
            actual: token.len(),
            max: MAX_LOCAL_TOKEN_BYTES,
        });
    }

    let bytes = token.as_bytes();
    if !is_lower_alnum(bytes[0]) {
        return Err(RegenerativeIdError::InvalidTokenStart);
    }
    if !is_lower_alnum(bytes[bytes.len() - 1]) {
        return Err(RegenerativeIdError::InvalidTokenEnd);
    }

    for (index, byte) in bytes.iter().copied().enumerate() {
        if !(is_lower_alnum(byte) || matches!(byte, b'-' | b'_' | b'.')) {
            return Err(RegenerativeIdError::InvalidTokenCharacter { index, byte });
        }
    }

    Ok(())
}

macro_rules! define_regenerative_id {
    ($name:ident, $kind:literal, $docs:literal) => {
        #[doc = $docs]
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            /// Exact canonical prefix for this subject kind.
            pub const PREFIX: &'static str = concat!("regen:v1:", $kind, ":");

            /// Construct a canonical subject identifier from a local token.
            pub fn new(local_token: impl AsRef<str>) -> Result<Self, RegenerativeIdError> {
                let local_token = local_token.as_ref();
                validate_local_token(local_token)?;
                Ok(Self(format!("{}{}", Self::PREFIX, local_token)))
            }

            /// Parse and validate the complete canonical textual identity.
            pub fn parse_canonical(value: impl AsRef<str>) -> Result<Self, RegenerativeIdError> {
                let value = value.as_ref();
                let Some(local_token) = value.strip_prefix(Self::PREFIX) else {
                    return Err(RegenerativeIdError::WrongPrefix {
                        expected: Self::PREFIX,
                    });
                };
                validate_local_token(local_token)?;
                Ok(Self(value.to_owned()))
            }

            /// Return the complete canonical textual identity.
            pub fn as_str(&self) -> &str {
                &self.0
            }

            /// Return the validated kind-local token.
            pub fn local_token(&self) -> &str {
                &self.0[Self::PREFIX.len()..]
            }
        }

        impl AsRef<str> for $name {
            fn as_ref(&self) -> &str {
                self.as_str()
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(self.as_str())
            }
        }

        impl FromStr for $name {
            type Err = RegenerativeIdError;

            fn from_str(s: &str) -> Result<Self, Self::Err> {
                Self::parse_canonical(s)
            }
        }

        #[cfg(feature = "serde")]
        impl Serialize for $name {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: Serializer,
            {
                serializer.serialize_str(self.as_str())
            }
        }

        #[cfg(feature = "serde")]
        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
            where
                D: Deserializer<'de>,
            {
                let value = String::deserialize(deserializer)?;
                Self::parse_canonical(value).map_err(D::Error::custom)
            }
        }
    };
}

define_regenerative_id!(
    RegenerativeSiteId,
    "site",
    "Stable semantic identity of a regenerative study, farm, garden, campus, or community site."
);
define_regenerative_id!(
    SoilPlotId,
    "soil-plot",
    "Stable semantic identity of one soil plot or bounded soil-treatment subject."
);
define_regenerative_id!(
    BiomassLotId,
    "biomass-lot",
    "Stable semantic identity of one biomass feedstock lot."
);
define_regenerative_id!(
    BiocharBatchId,
    "biochar-batch",
    "Stable semantic identity of one biochar production batch."
);
define_regenerative_id!(
    CompostBatchId,
    "compost-batch",
    "Stable semantic identity of one compost production batch."
);
define_regenerative_id!(
    CoCompostedAmendmentBatchId,
    "co-composted-amendment-batch",
    "Stable semantic identity of one co-composted amendment batch."
);
define_regenerative_id!(
    RegenerativeRecipeId,
    "recipe",
    "Stable semantic identity of a regenerative procedure/recipe lineage."
);
define_regenerative_id!(
    FieldTrialId,
    "field-trial",
    "Stable semantic identity of one regenerative field trial."
);
define_regenerative_id!(
    TreatmentArmId,
    "treatment-arm",
    "Stable semantic identity of one treatment or control arm within a trial."
);
define_regenerative_id!(
    RegenerativeFacilityId,
    "facility",
    "Stable semantic identity of one regenerative processing or service facility."
);
define_regenerative_id!(
    RegenerativeProjectId,
    "project",
    "Stable semantic identity of one regenerative project."
);
define_regenerative_id!(
    QualityProfileId,
    "quality-profile",
    "Stable semantic identity of one adopted quality-profile lineage."
);
define_regenerative_id!(
    RegenerativeEvidenceBundleId,
    "evidence-bundle",
    "Stable semantic identity of one regenerative evidence-bundle lineage."
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_identity_contains_schema_kind_and_local_token() {
        let id = RegenerativeSiteId::new("garden-01").unwrap();
        assert_eq!(id.as_str(), "regen:v1:site:garden-01");
        assert_eq!(id.local_token(), "garden-01");
        assert_eq!(id.to_string(), id.as_str());
    }

    #[test]
    fn typed_kind_prefix_prevents_cross_kind_substitution() {
        let site = RegenerativeSiteId::new("shared-01").unwrap();
        let plot = SoilPlotId::new("shared-01").unwrap();
        assert_ne!(site.as_str(), plot.as_str());
        assert!(SoilPlotId::parse_canonical(site.as_str()).is_err());
        assert!(RegenerativeSiteId::parse_canonical(plot.as_str()).is_err());
    }

    #[test]
    fn local_token_grammar_is_canonical_and_fail_closed() {
        for bad in [
            "",
            "Uppercase",
            "with:colon",
            "with space",
            "-leading",
            "trailing-",
            "unicode-é",
            "slash/value",
        ] {
            assert!(RegenerativeSiteId::new(bad).is_err(), "accepted {bad:?}");
        }

        for good in ["a", "site-01", "site_01", "site.01", "a1-b2_c3.d4"] {
            assert!(RegenerativeSiteId::new(good).is_ok(), "rejected {good:?}");
        }
    }

    #[test]
    fn token_length_is_bounded_by_bytes() {
        let max = "a".repeat(MAX_LOCAL_TOKEN_BYTES);
        assert!(RegenerativeSiteId::new(&max).is_ok());
        assert_eq!(
            RegenerativeSiteId::new(format!("{max}a")),
            Err(RegenerativeIdError::TokenTooLong {
                actual: MAX_LOCAL_TOKEN_BYTES + 1,
                max: MAX_LOCAL_TOKEN_BYTES,
            })
        );
    }

    #[test]
    fn every_frozen_subject_kind_has_a_distinct_prefix() {
        let prefixes = [
            RegenerativeSiteId::PREFIX,
            SoilPlotId::PREFIX,
            BiomassLotId::PREFIX,
            BiocharBatchId::PREFIX,
            CompostBatchId::PREFIX,
            CoCompostedAmendmentBatchId::PREFIX,
            RegenerativeRecipeId::PREFIX,
            FieldTrialId::PREFIX,
            TreatmentArmId::PREFIX,
            RegenerativeFacilityId::PREFIX,
            RegenerativeProjectId::PREFIX,
            QualityProfileId::PREFIX,
            RegenerativeEvidenceBundleId::PREFIX,
        ];
        let unique: std::collections::BTreeSet<_> = prefixes.into_iter().collect();
        assert_eq!(unique.len(), prefixes.len());
    }

    #[test]
    fn semantic_identity_does_not_contain_quality_or_authority_fields() {
        let id = BiocharBatchId::new("batch-2026-001").unwrap();
        assert_eq!(id.as_str(), "regen:v1:biochar-batch:batch-2026-001");
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trip_preserves_canonical_identity() {
        let original = FieldTrialId::new("trial-001").unwrap();
        let encoded = serde_json::to_string(&original).unwrap();
        assert_eq!(encoded, "\"regen:v1:field-trial:trial-001\"");
        let decoded: FieldTrialId = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, original);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_revalidates_untrusted_identity_bytes() {
        assert!(serde_json::from_str::<FieldTrialId>("\"regen:v1:field-trial:BAD\"").is_err());
        assert!(serde_json::from_str::<FieldTrialId>("\"regen:v1:site:trial-001\"").is_err());
    }
}
